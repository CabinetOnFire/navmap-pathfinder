//! Layered topology traversal and resumable multi-z A* search.

use super::grid::{HorizontalGridAccess, diagonal_routes, horizontal_successors, resolve_turf};
use super::search::{expand_path_diagonals, nav_link_estimate};
use super::*;

impl NavLinkHeuristic {
    pub(super) fn from_source_distances(
        goal: LayeredNode,
        min_target_distance: i32,
        is_flying: bool,
        source_distances: Vec<(LayeredNode, u32)>,
    ) -> Self {
        Self {
            goal,
            min_target_distance,
            is_flying,
            source_distances,
        }
    }

    fn estimate(&self, node: LayeredNode) -> Option<u32> {
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
        let entry = resolve_turf(
            (node.x, node.y, z),
            self.is_flying,
            true,
            self.pass_flags,
            pass_info,
        )?;
        self.cache.insert(node, entry);
        Ok(entry)
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
        diagonal_routes(self, source, from, (to.x, to.y), pass_info)
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
        let mut out = horizontal_successors(self, from, pass_info)?;
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
        let nav_links_by_source = Arc::clone(&self.nav_links_by_source);
        if let Some(nav_links) = nav_links_by_source.get(&(from.x, from.y, source_z)) {
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

impl HorizontalGridAccess<LayeredNode> for LayeredGrid {
    fn horizontal_info(
        &mut self,
        node: LayeredNode,
        pass_info: &ByondValue,
    ) -> Result<Option<TurfInfo>, NavPathError> {
        self.occupiable_info(node, pass_info)
    }

    fn horizontal_node(&self, reference: LayeredNode, position: Coord) -> LayeredNode {
        LayeredNode {
            x: position.0,
            y: position.1,
            ..reference
        }
    }

    fn horizontal_position(&self, node: LayeredNode) -> Coord {
        (node.x, node.y)
    }

    fn horizontal_cost(&mut self, base: u32, flags: u32) -> u32 {
        self.costs.step_cost(base, flags)
    }

    fn diagonal_handling(&self) -> DiagonalHandling {
        self.diagonal_handling
    }
}

impl MultiSearchJob {
    pub(super) fn cost_changes_stale(&mut self) -> bool {
        if !self.grid.costs.has_modifiers() {
            return false;
        }
        let grid = &self.grid;
        let Some(generation) = cost_generation_since(
            self.grid.cost_generation,
            self.grid.costs.modifier_mask(),
            |(x, y, z)| {
                let layer = grid.topology.z_to_layer.get(&z)?;
                let node = LayeredNode {
                    group: layer.group,
                    layer: layer.layer,
                    x,
                    y,
                };
                grid.cache
                    .get(&node)?
                    .as_ref()
                    .map(|info| info.cost_generation)
            },
        ) else {
            return true;
        };
        self.grid.cost_generation = generation;
        false
    }

    pub(super) fn final_stale_status(
        &mut self,
        pass_info: &ByondValue,
    ) -> Result<Option<&'static str>, NavPathError> {
        if topology_generation() != self.grid.topology.generation
            || (self.grid.allow_nav_links && nav_link_generation() != self.grid.topology.generation)
        {
            return Ok(Some("stale_topology"));
        }
        if pass_flags_from_byond(pass_info)? != self.grid.pass_flags {
            return Ok(Some("stale_pass_flags"));
        }
        if self.cost_changes_stale() {
            return Ok(Some("stale_costs"));
        }
        Ok(None)
    }

    /// Advances the search for one time slice.
    pub(super) fn step(
        &mut self,
        pass_info: &ByondValue,
    ) -> Result<SearchProgress<LayeredNode>, NavPathError> {
        tracy_span!("navmap.multi_search_step");
        self.last_touched = Instant::now();
        if topology_generation() != self.grid.topology.generation {
            return Ok(SearchProgress::Stale);
        }
        if self.grid.allow_nav_links && nav_link_generation() != self.grid.topology.generation {
            return Ok(SearchProgress::Stale);
        }
        if pass_flags_from_byond(pass_info)? != self.grid.pass_flags {
            return Ok(SearchProgress::StalePassFlags);
        }
        if self.cost_changes_stale() {
            return Ok(SearchProgress::StaleCosts);
        }
        let deadline = Instant::now() + SLICE_BUDGET;
        if let Some(builder) = &mut self.heuristic_builder {
            let is_flying = self.grid.is_flying;
            if !builder.advance(deadline, |from, to, distance| {
                relaxed_distance(from, to, distance, is_flying)
            }) {
                return Ok(SearchProgress::InProgress);
            }
            self.heuristic = NavLinkHeuristic::from_source_distances(
                self.goal,
                self.min_target_distance,
                self.grid.is_flying,
                self.heuristic_builder
                    .take()
                    .and_then(NavLinkDistanceBuilder::finish)
                    .unwrap_or_default(),
            );
        }
        let goal = self.goal;
        let min_target_distance = self.min_target_distance;
        let heuristic_scale = self.grid.costs.heuristic_scale;
        if !self.search.is_initialized() {
            if !self.grid.can_occupy(self.grid.start, pass_info)? {
                return Ok(SearchProgress::NoPath);
            }
            let Some(estimate) = self.heuristic.estimate(self.grid.start) else {
                return Ok(SearchProgress::NoPath);
            };
            self.search.initialize(
                self.grid.start,
                CostProfile::scale_heuristic(estimate, heuristic_scale),
            );
        }
        let progress = self.search.step_until(
            deadline,
            Some(self.grid.max_cost),
            |node| {
                node.group == goal.group
                    && node.layer == goal.layer
                    && chebyshev_distance((node.x, node.y), (goal.x, goal.y)) <= min_target_distance
            },
            |node| self.grid.successors(node, pass_info),
            |node| {
                self.heuristic
                    .estimate(node)
                    .map(|estimate| CostProfile::scale_heuristic(estimate, heuristic_scale))
            },
        )?;
        if self.cost_changes_stale() {
            return Ok(SearchProgress::StaleCosts);
        }
        Ok(progress)
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
