//! Layered topology traversal and resumable multi-z A* search.

use super::*;

impl Ord for RelaxedQueueEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .cmp(&self.cost)
            .then_with(|| other.sequence.cmp(&self.sequence))
    }
}

impl PartialOrd for RelaxedQueueEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PortalHeuristic {
    pub(super) fn new(
        topology: &Topology,
        goal: LayeredNode,
        min_target_distance: i32,
        is_flying: bool,
        links: &[NavLink],
    ) -> Self {
        let mut portals = vec![goal];
        for link in links {
            if let (Some(source), Some(destination)) = (
                node_for_xyz(topology, link.source.0, link.source.1, link.source.2),
                node_for_xyz(
                    topology,
                    link.destination.0,
                    link.destination.1,
                    link.destination.2,
                ),
            ) {
                if !portals.contains(&source) {
                    portals.push(source);
                }
                if !portals.contains(&destination) {
                    portals.push(destination);
                }
            }
        }

        // Reverse Dijkstra over the relaxed portal graph. Geometric movement is represented as a
        // complete graph between portals; links add directed weighted edges.
        let mut distances = HashMap::new();
        let mut frontier = BinaryHeap::new();
        let mut sequence = 0;
        for &portal in &portals {
            if let Some(cost) = relaxed_distance(portal, goal, min_target_distance, is_flying)
                && distances.get(&portal).is_none_or(|&old| cost < old)
            {
                distances.insert(portal, cost);
                sequence += 1;
                frontier.push(RelaxedQueueEntry {
                    node: portal,
                    cost,
                    sequence,
                });
            }
        }

        while let Some(current) = frontier.pop() {
            if distances.get(&current.node).copied() != Some(current.cost) {
                continue;
            }
            for &candidate in &portals {
                let Some(edge_cost) = relaxed_distance(candidate, current.node, 0, is_flying)
                else {
                    continue;
                };
                let candidate_cost = current.cost.saturating_add(edge_cost);
                if distances
                    .get(&candidate)
                    .is_none_or(|&old| candidate_cost < old)
                {
                    distances.insert(candidate, candidate_cost);
                    sequence += 1;
                    frontier.push(RelaxedQueueEntry {
                        node: candidate,
                        cost: candidate_cost,
                        sequence,
                    });
                }
            }
            for link in links {
                let Some(destination) = node_for_xyz(
                    topology,
                    link.destination.0,
                    link.destination.1,
                    link.destination.2,
                ) else {
                    continue;
                };
                if destination != current.node {
                    continue;
                }
                let Some(source) =
                    node_for_xyz(topology, link.source.0, link.source.1, link.source.2)
                else {
                    continue;
                };
                let candidate_cost = current.cost.saturating_add(link.cost);
                if distances
                    .get(&source)
                    .is_none_or(|&old| candidate_cost < old)
                {
                    distances.insert(source, candidate_cost);
                    sequence += 1;
                    frontier.push(RelaxedQueueEntry {
                        node: source,
                        cost: candidate_cost,
                        sequence,
                    });
                }
            }
        }

        let source_distances = links
            .iter()
            .filter_map(|link| {
                let source = node_for_xyz(topology, link.source.0, link.source.1, link.source.2)?;
                Some((source, distances.get(&source).copied()?))
            })
            .collect();
        Self {
            goal,
            min_target_distance,
            is_flying,
            source_distances,
        }
    }

    fn estimate(&self, node: LayeredNode) -> u32 {
        let mut best = relaxed_distance(node, self.goal, self.min_target_distance, self.is_flying)
            .unwrap_or(0);
        for &(source, distance) in &self.source_distances {
            let Some(to_source) = relaxed_distance(node, source, 0, self.is_flying) else {
                continue;
            };
            best = best.min(to_source.saturating_add(distance));
        }
        best
    }
}

impl Ord for LayeredQueueEntry {
    /// Orders entries by lowest estimated total cost first.
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .estimate
            .cmp(&self.estimate)
            .then_with(|| other.cost.cmp(&self.cost))
            .then_with(|| other.sequence.cmp(&self.sequence))
    }
}

impl PartialOrd for LayeredQueueEntry {
    /// Delegates partial ordering to the queue ordering.
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl LayeredGrid {
    /// Resolves a layered node to its z-level.
    fn z_for(&self, node: LayeredNode) -> Option<i16> {
        self.topology
            .layer_to_z
            .get(&(node.group, node.layer))
            .copied()
    }

    /// Looks up and caches one layered turf.
    fn lookup(
        &mut self,
        node: LayeredNode,
        pass_info: &ByondValue,
    ) -> Result<Option<TurfInfo>, NavPathError> {
        tracy_span!("navmap.layered_lookup");
        if let Some(hit) = self.cache.get(&node) {
            return Ok(*hit);
        }
        let Some(z) = self.z_for(node) else {
            self.cache.insert(node, None);
            return Ok(None);
        };
        let coords = (node.x, node.y, z);
        let entry = match cached_nav_pass(coords) {
            Some(nav_pass) if nav_pass & BAKED_FLAG != 0 => {
                if let Some((open_edges, vertical_open)) =
                    cached_unconditional_edges(nav_pass, self.is_flying)
                {
                    Some(TurfInfo {
                        open_edges: open_edges | (vertical_open << 24),
                        simulated: nav_pass_is_simulated(nav_pass),
                    })
                } else {
                    let turf = locate_xyz(ByondXYZ::new(node.x, node.y, z))?;
                    if turf.is_null() {
                        None
                    } else {
                        let (open_edges, vertical_open, nav_pass) =
                            self.resolve_edges(coords, &turf, pass_info)?;
                        Some(TurfInfo {
                            open_edges: open_edges | (vertical_open << 24),
                            simulated: nav_pass_is_simulated(nav_pass),
                        })
                    }
                }
            }
            _ => {
                let turf = locate_xyz(ByondXYZ::new(node.x, node.y, z))?;
                if turf.is_null() {
                    None
                } else {
                    let (open_edges, vertical_open, nav_pass) =
                        self.resolve_edges(coords, &turf, pass_info)?;
                    Some(TurfInfo {
                        open_edges: open_edges | (vertical_open << 24),
                        simulated: nav_pass_is_simulated(nav_pass),
                    })
                }
            }
        };
        self.cache.insert(node, entry);
        Ok(entry)
    }

    /// Resolves horizontal and vertical edges for a layered turf.
    fn resolve_edges(
        &self,
        coords: TurfCoords,
        turf: &ByondValue,
        pass_info: &ByondValue,
    ) -> Result<(i32, i32, i32), NavPathError> {
        let mut nav_pass = cached_nav_pass(coords).unwrap_or(read_nav_pass(turf)?);
        if nav_pass & BAKED_FLAG == 0 {
            let _: () = turf.call("nav_bake", std::iter::empty::<ByondValue>())?;
            nav_pass = read_nav_pass(turf)?;
            if nav_pass & BAKED_FLAG == 0 {
                return Ok((0, 0, nav_pass));
            }
            cache_nav_pass(coords, nav_pass);
        }
        let mut open = 0;
        for dir in CARDINALS {
            if nav_pass & (dir << if self.is_flying { FLYING_SHIFT } else { 0 }) == 0 {
                continue;
            }
            if nav_pass & (dir << COND_SHIFT) == 0
                || evaluate_conditional_edge(turf, dir, self.pass_flags, pass_info)?
            {
                open |= dir;
            }
        }
        let mut vertical = 0;
        if self.is_flying {
            for dir in [UP, DOWN] {
                let nav_bit = dir << FLYING_SHIFT;
                let cond_bit = dir << COND_SHIFT;
                if nav_pass & nav_bit == 0 {
                    continue;
                }
                if nav_pass & cond_bit == 0
                    || evaluate_conditional_edge(turf, dir, self.pass_flags, pass_info)?
                {
                    vertical |= dir;
                }
            }
        }
        Ok((open, vertical, nav_pass))
    }

    /// Returns cached node data when the node is occupiable.
    fn occupiable_info(
        &mut self,
        node: LayeredNode,
        pass_info: &ByondValue,
    ) -> Result<Option<TurfInfo>, NavPathError> {
        if self.max_range > 0
            && chebyshev_distance((node.x, node.y), (self.start.x, self.start.y)) > self.max_range
        {
            return Ok(None);
        }
        if self.avoid == Some(node) {
            return Ok(None);
        }
        Ok(self
            .lookup(node, pass_info)?
            .filter(|info| turf_allowed(self.simulated_only, info.simulated)))
    }

    /// Checks whether a layered node can be occupied.
    fn can_occupy(
        &mut self,
        node: LayeredNode,
        pass_info: &ByondValue,
    ) -> Result<bool, NavPathError> {
        Ok(self.occupiable_info(node, pass_info)?.is_some())
    }

    /// Evaluates both cardinal routes around a diagonal.
    fn diagonal_routes(
        &mut self,
        from: LayeredNode,
        to: LayeredNode,
        pass_info: &ByondValue,
    ) -> Result<DiagonalRoutes, NavPathError> {
        let Some(source) = self.lookup(from, pass_info)? else {
            return Ok(DiagonalRoutes {
                north_south_first: false,
                east_west_first: false,
            });
        };
        let dx = to.x - from.x;
        let dy = to.y - from.y;
        let ns = if dy > 0 { NORTH } else { SOUTH };
        let ew = if dx > 0 { EAST } else { WEST };
        if source.open_edges & ns == 0 && source.open_edges & ew == 0 {
            return Ok(DiagonalRoutes {
                north_south_first: false,
                east_west_first: false,
            });
        }
        if self.occupiable_info(to, pass_info)?.is_none() {
            return Ok(DiagonalRoutes {
                north_south_first: false,
                east_west_first: false,
            });
        }
        let ns_node = LayeredNode {
            x: from.x,
            y: to.y,
            ..from
        };
        let ew_node = LayeredNode {
            x: to.x,
            y: from.y,
            ..from
        };
        Ok(DiagonalRoutes {
            north_south_first: source.open_edges & ns != 0
                && self
                    .occupiable_info(ns_node, pass_info)?
                    .is_some_and(|info| info.open_edges & ew != 0),
            east_west_first: source.open_edges & ew != 0
                && self
                    .occupiable_info(ew_node, pass_info)?
                    .is_some_and(|info| info.open_edges & ns != 0),
        })
    }

    /// Returns horizontal, vertical, and linked successors.
    fn successors(
        &mut self,
        from: LayeredNode,
        pass_info: &ByondValue,
    ) -> Result<Vec<(LayeredNode, u32, LayeredAction)>, NavPathError> {
        tracy_span!("navmap.layered_successors");
        let Some(source) = self.lookup(from, pass_info)? else {
            return Ok(Vec::new());
        };
        let mut out = Vec::with_capacity(12);
        for (step, dx, dy, cost) in STEPS {
            let to = LayeredNode {
                x: from.x + dx,
                y: from.y + dy,
                ..from
            };
            let walkable = if dx == 0 || dy == 0 {
                source.open_edges & step != 0 && self.can_occupy(to, pass_info)?
            } else {
                let routes = self.diagonal_routes(from, to, pass_info)?;
                routes.north_south_first || routes.east_west_first
            };
            if walkable {
                out.push((to, cost, LayeredAction::None));
            }
        }
        if self.is_flying {
            let vertical = source.open_edges >> 24;
            for (dir, delta) in [(UP, 1), (DOWN, -1)] {
                if vertical & dir == 0 {
                    continue;
                }
                let to = LayeredNode {
                    layer: from.layer + delta,
                    ..from
                };
                if self.can_occupy(to, pass_info)? {
                    out.push((to, 10, LayeredAction::Vertical(dir)));
                }
            }
        }
        let Some(source_z) = self.z_for(from) else {
            return Ok(out);
        };
        if let Some(links) = self
            .links_by_source
            .get(&(from.x, from.y, source_z))
            .cloned()
        {
            for link in links {
                let Some(destination_layer) = self.topology.z_to_layer.get(&link.destination.2)
                else {
                    continue;
                };
                let to = LayeredNode {
                    group: destination_layer.group,
                    layer: destination_layer.layer,
                    x: link.destination.0,
                    y: link.destination.1,
                };
                if !self.can_occupy(to, pass_info)? {
                    continue;
                }
                tracy_span!("navmap.link_can_plan");
                let allowed: ByondValue = call_global(
                    "navmap_pathfinder_link_can_plan",
                    [ByondValue::new_num(link.id as f32), pass_info.clone()],
                )?;
                if truthy(&allowed) {
                    out.push((to, link.cost, LayeredAction::Link(link.id)));
                }
            }
        }
        Ok(out)
    }
}

/// Result of one multi-z search slice.

impl MultiSearchJob {
    /// Advances the search for one time slice.
    pub(super) fn step(
        &mut self,
        pass_info: &ByondValue,
    ) -> Result<MultiJobProgress, NavPathError> {
        tracy_span!("navmap.multi_search_step");
        self.last_touched = Instant::now();
        if topology_generation() != self.grid.topology.generation {
            return Ok(MultiJobProgress::Stale);
        }
        let started = Instant::now();
        if !self.initialized {
            self.initialized = true;
            if !self.grid.can_occupy(self.grid.start, pass_info)? {
                return Ok(MultiJobProgress::NoPath);
            }
            self.push(self.grid.start, 0);
        }
        while started.elapsed() < SLICE_BUDGET {
            let Some(current) = self.frontier.pop() else {
                return Ok(MultiJobProgress::NoPath);
            };
            if self.costs.get(&current.node).copied() != Some(current.cost) {
                continue;
            }
            if current.node.group == self.goal.group
                && current.node.layer == self.goal.layer
                && chebyshev_distance((current.node.x, current.node.y), (self.goal.x, self.goal.y))
                    <= self.min_target_distance
            {
                return Ok(MultiJobProgress::Found(current.node));
            }
            for (next, movement_cost, action) in self.grid.successors(current.node, pass_info)? {
                let next_cost = current.cost.saturating_add(movement_cost);
                if self.grid.max_cost > 0 && next_cost > self.grid.max_cost {
                    continue;
                }
                if self.costs.get(&next).is_none_or(|&old| next_cost < old) {
                    self.previous.insert(next, (current.node, action));
                    self.push(next, next_cost);
                }
            }
        }
        Ok(MultiJobProgress::InProgress)
    }

    /// Queues a node with its current best cost.
    fn push(&mut self, node: LayeredNode, cost: u32) {
        self.costs.insert(node, cost);
        self.sequence += 1;
        self.frontier.push(LayeredQueueEntry {
            node,
            cost,
            estimate: cost.saturating_add(self.heuristic.estimate(node)),
            sequence: self.sequence,
        });
    }

    /// Reconstructs nodes and actions from the goal.
    fn reconstruct(&self, mut node: LayeredNode) -> (Vec<LayeredNode>, Vec<LayeredAction>) {
        let mut nodes = vec![node];
        // Keep each action beside its destination while unwinding; the start
        // node receives a sentinel after both lists are reversed.
        let mut actions = Vec::new();
        while let Some((previous, action)) = self.previous.get(&node).copied() {
            nodes.push(previous);
            actions.push(action);
            node = previous;
        }
        nodes.reverse();
        actions.reverse();
        actions.insert(0, LayeredAction::None);
        (nodes, actions)
    }

    /// Rechecks and converts a layered path to DM values.
    pub(super) fn final_path(
        &mut self,
        goal: LayeredNode,
        pass_info: &ByondValue,
    ) -> Result<(ByondValue, ByondValue), NavPathError> {
        let (raw_nodes, raw_actions) = self.reconstruct(goal);
        let mut nodes = vec![raw_nodes[0]];
        let mut actions = vec![LayeredAction::None];
        for (index, next) in raw_nodes.iter().copied().enumerate().skip(1) {
            let previous = *nodes.last().unwrap();
            if previous.group == next.group
                && previous.layer == next.layer
                && previous.x != next.x
                && previous.y != next.y
                && matches!(raw_actions[index], LayeredAction::None)
            {
                let routes = self.grid.diagonal_routes(previous, next, pass_info)?;
                let intermediate = match self.diagonal_handling {
                    DiagonalHandling::RemoveAll if routes.north_south_first => Some(LayeredNode {
                        x: previous.x,
                        y: next.y,
                        ..previous
                    }),
                    DiagonalHandling::RemoveAll if routes.east_west_first => Some(LayeredNode {
                        x: next.x,
                        y: previous.y,
                        ..previous
                    }),
                    DiagonalHandling::RemoveAll => return empty_list_pair(),
                    DiagonalHandling::RemoveClunky
                        if !routes.north_south_first && routes.east_west_first =>
                    {
                        Some(LayeredNode {
                            x: next.x,
                            y: previous.y,
                            ..previous
                        })
                    }
                    _ => None,
                };
                if let Some(intermediate) = intermediate {
                    nodes.push(intermediate);
                    actions.push(LayeredAction::None);
                }
            }
            nodes.push(next);
            actions.push(raw_actions[index]);
        }
        let mut path_values = Vec::new();
        for node in &nodes {
            let Some(z) = self.grid.z_for(*node) else {
                return empty_list_pair();
            };
            let turf = locate_xyz(ByondXYZ::new(node.x, node.y, z))?;
            if turf.is_null() {
                return empty_list_pair();
            }
            path_values.push(turf);
        }
        if self.skip_first && !path_values.is_empty() {
            let _ = path_values.remove(0);
            actions.remove(0);
        }
        let path = path_values.to_byond()?;
        let action_values = actions
            .into_iter()
            .map(|action| match action {
                LayeredAction::None => ByondValue::NULL,
                LayeredAction::Vertical(dir) => ByondValue::new_num(dir as f32),
                LayeredAction::Link(id) => ByondValue::new_num(-(id as f32)),
            })
            .collect::<Vec<_>>();
        let action_list = action_values.to_byond()?;
        Ok((path, action_list))
    }
}
