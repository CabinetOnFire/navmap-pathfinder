//! Single-z turf lookup, blocker evaluation, and successor generation.

use super::*;

impl Grid {
    /// Looks up and caches one turf's navigation data.
    pub(super) fn lookup(
        &mut self,
        x: i16,
        y: i16,
        pass_info: &ByondValue,
    ) -> Result<Option<TurfInfo>, NavPathError> {
        tracy_span!("navmap.grid_lookup");
        // A search must use one consistent view of each turf, even if it yields and resumes.
        if let Some(hit) = self.cache.get(&(x, y)) {
            return Ok(*hit);
        }

        let coords = (x, y, self.z);
        let entry = match cached_nav_data(coords) {
            // Published baked data is authoritative for unconditional edges. Avoid crossing the BYOND FFI for the overwhelmingly common static-map case.
            Some(data) if data.nav_pass & BAKED_FLAG != 0 => {
                if let Some((open_edges, _)) = cached_unconditional_edges(data.nav_pass, self.is_flying)
                {
                    Some(TurfInfo {
                        open_edges,
                        simulated: nav_pass_is_simulated(data.nav_pass),
                        cost_flags: data.nav_cost_flags,
                    })
                } else {
                    let turf = locate_xyz(ByondXYZ::new(x, y, self.z))?;
                    if turf.is_null() {
                        None
                    } else {
                        let (open_edges, nav_pass, cost_flags) =
                            self.resolve_edges(coords, &turf, pass_info)?;
                        Some(TurfInfo {
                            open_edges,
                            simulated: nav_pass_is_simulated(nav_pass),
                            cost_flags,
                        })
                    }
                }
            }
            _ => {
                let turf = locate_xyz(ByondXYZ::new(x, y, self.z))?;
                if turf.is_null() {
                    None
                } else {
                    let (open_edges, nav_pass, cost_flags) = self.resolve_edges(coords, &turf, pass_info)?;
                    Some(TurfInfo {
                        open_edges,
                        simulated: nav_pass_is_simulated(nav_pass),
                        cost_flags,
                    })
                }
            }
        };
        self.cache.insert((x, y), entry);
        Ok(entry)
    }

    /// Resolves packed edges and live conditional blockers.
    pub(super) fn resolve_edges(
        &self,
        coords: TurfCoords,
        turf: &ByondValue,
        pass_info: &ByondValue,
    ) -> Result<(i32, i32, u32), NavPathError> {
        let cached = cached_nav_data(coords);
        let (mut nav_pass, mut cost_flags) = match cached {
            Some(data) => (data.nav_pass, data.nav_cost_flags),
            None => (read_nav_pass(turf)?, read_nav_cost_flags(turf)?),
        };
        if nav_pass & BAKED_FLAG == 0 {
            // Bake dirty turfs before trusting their edge bits.
            let _: () = turf.call("nav_bake", std::iter::empty::<ByondValue>())?;
            nav_pass = read_nav_pass(turf)?;
            cost_flags = read_nav_cost_flags(turf)?;
            if nav_pass & BAKED_FLAG == 0 {
                return Ok((0, nav_pass, cost_flags));
            }
            cache_nav_data(coords, nav_pass, cost_flags);
        }

        let (open, _) = resolve_packed_edges(nav_pass, self.is_flying, false, |dir| {
            evaluate_conditional_edge(turf, dir, self.pass_flags, pass_info)
        })?;
        Ok((open, nav_pass, cost_flags))
    }

    /// Returns cached turf data when the coordinate is occupiable.
    pub(super) fn occupiable_info(
        &mut self,
        coord: Coord,
        pass_info: &ByondValue,
    ) -> Result<Option<TurfInfo>, NavPathError> {
        if !coordinate_allowed(coord, self.start, self.max_range, self.avoid) {
            return Ok(None);
        }
        Ok(self
            .lookup(coord.0, coord.1, pass_info)?
            .filter(|info| turf_allowed(self.simulated_only, info.simulated)))
    }

    /// Checks whether a coordinate can be occupied.
    pub(super) fn can_occupy(
        &mut self,
        coord: Coord,
        pass_info: &ByondValue,
    ) -> Result<bool, NavPathError> {
        Ok(self.occupiable_info(coord, pass_info)?.is_some())
    }

    /// Evaluates both cardinal routes around a diagonal.
    pub(super) fn diagonal_routes_from_source(
        &mut self,
        source: TurfInfo,
        from: Coord,
        to: Coord,
        pass_info: &ByondValue,
        stop_after_first: bool,
    ) -> Result<DiagonalRoutes, NavPathError> {
        let dx = to.0 - from.0;
        let dy = to.1 - from.1;
        let north_south_dir = if dy > 0 { NORTH } else { SOUTH };
        let east_west_dir = if dx > 0 { EAST } else { WEST };
        let north_south = (from.0, from.1 + dy);
        let east_west = (from.0 + dx, from.1);

        let north_south_open = source.open_edges & north_south_dir != 0;
        let east_west_open = source.open_edges & east_west_dir != 0;
        if !north_south_open && !east_west_open {
            return Ok(DiagonalRoutes {
                north_south_first: false,
                east_west_first: false,
            });
        }

        // Both routes share the destination. Resolve it once instead of repeating the lookup
        // and occupancy checks for each route.
        if self.occupiable_info(to, pass_info)?.is_none() {
            return Ok(DiagonalRoutes {
                north_south_first: false,
                east_west_first: false,
            });
        }

        // A diagonal is legal only if at least one two-cardinal route through its corner is legal.
        let north_south_first = north_south_open
            && self
                .occupiable_info(north_south, pass_info)?
                .is_some_and(|info| info.open_edges & east_west_dir != 0);
        if stop_after_first && north_south_first {
            return Ok(DiagonalRoutes {
                north_south_first: true,
                east_west_first: false,
            });
        }
        let east_west_first = east_west_open
            && self
                .occupiable_info(east_west, pass_info)?
                .is_some_and(|info| info.open_edges & north_south_dir != 0);
        Ok(DiagonalRoutes {
            north_south_first,
            east_west_first,
        })
    }

    /// Evaluates a diagonal using its source turf.
    pub(super) fn diagonal_routes(
        &mut self,
        from: Coord,
        to: Coord,
        pass_info: &ByondValue,
    ) -> Result<DiagonalRoutes, NavPathError> {
        let Some(source) = self.lookup(from.0, from.1, pass_info)? else {
            return Ok(DiagonalRoutes {
                north_south_first: false,
                east_west_first: false,
            });
        };
        self.diagonal_routes_from_source(source, from, to, pass_info, false)
    }

    /// Returns all walkable cardinal and diagonal successors.
    pub(super) fn successors(
        &mut self,
        from: Coord,
        pass_info: &ByondValue,
    ) -> Result<Vec<(Coord, u32, SearchAction)>, NavPathError> {
        tracy_span!("navmap.grid_successors");
        let Some(source) = self.lookup(from.0, from.1, pass_info)? else {
            return Ok(Vec::new());
        };
        let mut out = Vec::with_capacity(8);
        for (step, dx, dy, cost) in STEPS {
            let to = (from.0 + dx, from.1 + dy);
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
                let routes = self.diagonal_routes_from_source(source, from, to, pass_info, false)?;
                if !routes.north_south_first && !routes.east_west_first {
                    continue;
                }
                let Some(destination) = self.occupiable_info(to, pass_info)? else {
                    continue;
                };
                let north_south_mid = (from.0, to.1);
                let east_west_mid = (to.0, from.1);
                let expand_all = self.diagonal_handling == DiagonalHandling::RemoveAll;
                let expand_clunky = self.diagonal_handling == DiagonalHandling::RemoveClunky
                    && !routes.north_south_first
                    && routes.east_west_first;
                if expand_all || expand_clunky {
                    let mut selected: Option<(u32, Coord)> = None;
                    if routes.north_south_first {
                        if let Some(mid) = self.occupiable_info(north_south_mid, pass_info)? {
                            selected = Some((
                                self.costs.step_cost(10, mid.cost_flags)
                                    .saturating_add(self.costs.step_cost(10, destination.cost_flags)),
                                north_south_mid,
                            ));
                        }
                    }
                    if routes.east_west_first {
                        if let Some(mid) = self.occupiable_info(east_west_mid, pass_info)? {
                            let candidate = self.costs.step_cost(10, mid.cost_flags)
                                .saturating_add(self.costs.step_cost(10, destination.cost_flags));
                            if selected.is_none_or(|(old, _)| candidate < old) {
                                selected = Some((candidate, east_west_mid));
                            }
                        }
                    }
                    if let Some((route_cost, (mid_x, mid_y))) = selected {
                        out.push((to, route_cost, SearchAction::Diagonal(mid_x, mid_y)));
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
        if let Some(nav_links) = self.nav_links_by_source.get(&(from.0, from.1)).cloned() {
            for nav_link in nav_links {
                let to = (nav_link.destination.0, nav_link.destination.1);
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
