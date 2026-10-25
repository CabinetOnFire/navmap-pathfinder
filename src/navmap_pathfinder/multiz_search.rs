//! Layered topology traversal and resumable multi-z A* search.

use super::search::{expand_path_diagonals, nav_link_estimate, nav_link_source_distances};
use super::*;

impl NavLinkHeuristic {
    pub(super) fn new(
        topology: &Topology,
        goal: LayeredNode,
        min_target_distance: i32,
        is_flying: bool,
        nav_links: &[NavLink],
    ) -> Self {
        let edges: Vec<_> = nav_links
            .iter()
            .filter_map(|nav_link| {
                Some((
                    node_for_xyz(
                        topology,
                        nav_link.source.0,
                        nav_link.source.1,
                        nav_link.source.2,
                    )?,
                    node_for_xyz(
                        topology,
                        nav_link.destination.0,
                        nav_link.destination.1,
                        nav_link.destination.2,
                    )?,
                    nav_link.cost,
                ))
            })
            .collect();
        let source_distances =
            nav_link_source_distances(goal, min_target_distance, &edges, |from, to, distance| {
                relaxed_distance(from, to, distance, is_flying)
            });
        Self {
            goal,
            min_target_distance,
            is_flying,
            source_distances,
        }
    }

    fn estimate(&self, node: LayeredNode) -> u32 {
        nav_link_estimate(
            node,
            self.goal,
            self.min_target_distance,
            &self.source_distances,
            |from, to, distance| relaxed_distance(from, to, distance, self.is_flying),
        )
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
        let entry = match cached_nav_data(coords) {
            Some(data) if data.nav_pass & BAKED_FLAG != 0 => {
                if let Some((open_edges, vertical_open)) =
                    cached_unconditional_edges(data.nav_pass, self.is_flying)
                {
                    Some(TurfInfo {
                        open_edges: open_edges | (vertical_open << 24),
                        simulated: nav_pass_is_simulated(data.nav_pass),
                        cost_flags: data.nav_cost_flags,
                    })
                } else {
                    let turf = locate_xyz(ByondXYZ::new(node.x, node.y, z))?;
                    if turf.is_null() {
                        None
                    } else {
                        let (open_edges, vertical_open, nav_pass, cost_flags) =
                            self.resolve_edges(coords, &turf, pass_info)?;
                        Some(TurfInfo {
                            open_edges: open_edges | (vertical_open << 24),
                            simulated: nav_pass_is_simulated(nav_pass),
                            cost_flags,
                        })
                    }
                }
            }
            _ => {
                let turf = locate_xyz(ByondXYZ::new(node.x, node.y, z))?;
                if turf.is_null() {
                    None
                } else {
                    let (open_edges, vertical_open, nav_pass, cost_flags) =
                        self.resolve_edges(coords, &turf, pass_info)?;
                    Some(TurfInfo {
                        open_edges: open_edges | (vertical_open << 24),
                        simulated: nav_pass_is_simulated(nav_pass),
                        cost_flags,
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
    ) -> Result<(i32, i32, i32, u32), NavPathError> {
        let cached = cached_nav_data(coords);
        let (mut nav_pass, mut cost_flags) = match cached {
            Some(data) => (data.nav_pass, data.nav_cost_flags),
            None => (read_nav_pass(turf)?, read_nav_cost_flags(turf)?),
        };
        if nav_pass & BAKED_FLAG == 0 {
            let _: () = turf.call("nav_bake", std::iter::empty::<ByondValue>())?;
            nav_pass = read_nav_pass(turf)?;
            cost_flags = read_nav_cost_flags(turf)?;
            if nav_pass & BAKED_FLAG == 0 {
                return Ok((0, 0, nav_pass, cost_flags));
            }
            cache_nav_data(coords, nav_pass, cost_flags);
        }
        let (open, vertical) = resolve_packed_edges(nav_pass, self.is_flying, true, |dir| {
            evaluate_conditional_edge(turf, dir, self.pass_flags, pass_info)
        })?;
        Ok((open, vertical, nav_pass, cost_flags))
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

    /// Returns horizontal, vertical, and navigation-link successors.
    fn successors(
        &mut self,
        from: LayeredNode,
        pass_info: &ByondValue,
    ) -> Result<Vec<(LayeredNode, u32, SearchAction)>, NavPathError> {
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
            if dx == 0 || dy == 0 {
                if source.open_edges & step == 0 {
                    continue;
                }
                let Some(destination) = self.occupiable_info(to, pass_info)? else {
                    continue;
                };
                out.push((
                    to,
                    self.costs.step_cost(cost, destination.cost_flags),
                    SearchAction::None,
                ));
            } else {
                let routes = self.diagonal_routes(from, to, pass_info)?;
                if !routes.north_south_first && !routes.east_west_first {
                    continue;
                }
                let Some(destination) = self.occupiable_info(to, pass_info)? else {
                    continue;
                };
                let expand_all = self.diagonal_handling == DiagonalHandling::RemoveAll;
                let expand_clunky = self.diagonal_handling == DiagonalHandling::RemoveClunky
                    && !routes.north_south_first
                    && routes.east_west_first;
                if expand_all || expand_clunky {
                    let north_south_mid = LayeredNode { x: from.x, y: to.y, ..from };
                    let east_west_mid = LayeredNode { x: to.x, y: from.y, ..from };
                    let mut selected: Option<(u32, LayeredNode)> = None;
                    if routes.north_south_first
                        && let Some(mid) = self.occupiable_info(north_south_mid, pass_info)?
                    {
                        selected = Some((
                            self.costs.step_cost(10, mid.cost_flags)
                                .saturating_add(self.costs.step_cost(10, destination.cost_flags)),
                            north_south_mid,
                        ));
                    }
                    if routes.east_west_first
                        && let Some(mid) = self.occupiable_info(east_west_mid, pass_info)?
                    {
                        let candidate = self.costs.step_cost(10, mid.cost_flags)
                            .saturating_add(self.costs.step_cost(10, destination.cost_flags));
                        if selected.is_none_or(|(old, _)| candidate < old) {
                            selected = Some((candidate, east_west_mid));
                        }
                    }
                    if let Some((route_cost, mid)) = selected {
                        out.push((
                            to,
                            route_cost,
                            SearchAction::Diagonal(mid.x, mid.y),
                        ));
                    }
                } else {
                    out.push((
                        to,
                        self.costs.step_cost(cost, destination.cost_flags),
                        SearchAction::None,
                    ));
                }
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
                if let Some(destination) = self.occupiable_info(to, pass_info)? {
                    out.push((
                        to,
                        self.costs.step_cost(10, destination.cost_flags),
                        SearchAction::Vertical(dir),
                    ));
                }
            }
        }
        if !self.allow_nav_links {
            return Ok(out);
        }
        let Some(source_z) = self.z_for(from) else {
            return Ok(out);
        };
        if let Some(nav_links) = self
            .nav_links_by_source
            .get(&(from.x, from.y, source_z))
            .cloned()
        {
            for nav_link in nav_links {
                let Some(destination_layer) = self.topology.z_to_layer.get(&nav_link.destination.2)
                else {
                    continue;
                };
                let to = LayeredNode {
                    group: destination_layer.group,
                    layer: destination_layer.layer,
                    x: nav_link.destination.0,
                    y: nav_link.destination.1,
                };
                let Some(destination) = self.occupiable_info(to, pass_info)? else {
                    continue;
                };
                tracy_span!("navmap.link_can_plan");
                let allowed: ByondValue = call_global(
                    "navmap_pathfinder_link_can_plan",
                    [ByondValue::new_num(nav_link.id as f32), pass_info.clone()],
                )?;
                if truthy(&allowed) {
                    out.push((
                        to,
                        self.costs.step_cost(nav_link.cost, destination.cost_flags),
                        SearchAction::NavLink(nav_link.id),
                    ));
                }
            }
        }
        Ok(out)
    }
}

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
        if self.grid.costs.has_modifiers() && nav_cost_generation() != self.grid.cost_generation {
            return Ok(MultiJobProgress::StaleCosts);
        }
        let goal = self.goal;
        let min_target_distance = self.min_target_distance;
        let heuristic_scale = self.grid.costs.heuristic_scale;
        if !self.search.is_initialized() {
            if !self.grid.can_occupy(self.grid.start, pass_info)? {
                return Ok(MultiJobProgress::NoPath);
            }
            let estimate = self.heuristic.estimate(self.grid.start);
            self.search.initialize(
                self.grid.start,
                CostProfile::scale_heuristic(estimate, heuristic_scale),
            );
        }
        let progress = self.search.step(
            Some(self.grid.max_cost),
            |node| {
                node.group == goal.group
                    && node.layer == goal.layer
                    && chebyshev_distance((node.x, node.y), (goal.x, goal.y)) <= min_target_distance
            },
            |node| self.grid.successors(node, pass_info),
            |node| CostProfile::scale_heuristic(self.heuristic.estimate(node), heuristic_scale),
        )?;
        if self.grid.costs.has_modifiers() && nav_cost_generation() != self.grid.cost_generation {
            return Ok(MultiJobProgress::StaleCosts);
        }
        match progress {
            SearchProgress::InProgress => Ok(MultiJobProgress::InProgress),
            SearchProgress::NoPath => Ok(MultiJobProgress::NoPath),
            SearchProgress::Stale => Ok(MultiJobProgress::Stale),
            SearchProgress::StaleCosts => Ok(MultiJobProgress::StaleCosts),
            SearchProgress::Found(nodes, actions) => Ok(MultiJobProgress::Found(nodes, actions)),
        }
    }

    /// Rechecks and converts a layered path to DM values.
    pub(super) fn final_path(
        &mut self,
        raw_nodes: Vec<LayeredNode>,
        raw_actions: Vec<SearchAction>,
        pass_info: &ByondValue,
    ) -> Result<(ByondValue, ByondValue), NavPathError> {
        let Some((nodes, mut actions)) = expand_path_diagonals(
            &raw_nodes,
            &raw_actions,
            self.diagonal_handling,
            |node| (node.x, node.y),
            |node, (x, y)| LayeredNode { x, y, ..node },
            |from, to, action| {
                from.group == to.group
                    && from.layer == to.layer
                    && matches!(action, SearchAction::None | SearchAction::Diagonal(_, _))
            },
            |from, to| self.grid.diagonal_routes(from, to, pass_info),
        )?
        else {
            return empty_list_pair();
        };
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
        Ok((path, actions_to_byond(actions)?))
    }
}
