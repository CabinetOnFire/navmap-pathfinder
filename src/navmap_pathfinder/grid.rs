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
        let entry = match cached_nav_pass(coords) {
            // Published baked data is authoritative for unconditional edges. Avoid crossing
            // the BYOND FFI for the overwhelmingly common static-map case.
            Some(nav_pass) if nav_pass & BAKED_FLAG != 0 => {
                if let Some((open_edges, _)) = cached_unconditional_edges(nav_pass, self.is_flying)
                {
                    Some(TurfInfo {
                        open_edges,
                        simulated: nav_pass_is_simulated(nav_pass),
                    })
                } else {
                    let turf = locate_xyz(ByondXYZ::new(x, y, self.z))?;
                    if turf.is_null() {
                        None
                    } else {
                        let (open_edges, nav_pass) =
                            self.resolve_edges(coords, &turf, pass_info)?;
                        Some(TurfInfo {
                            open_edges,
                            simulated: nav_pass_is_simulated(nav_pass),
                        })
                    }
                }
            }
            _ => {
                let turf = locate_xyz(ByondXYZ::new(x, y, self.z))?;
                if turf.is_null() {
                    None
                } else {
                    let (open_edges, nav_pass) = self.resolve_edges(coords, &turf, pass_info)?;
                    Some(TurfInfo {
                        open_edges,
                        simulated: nav_pass_is_simulated(nav_pass),
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
    ) -> Result<(i32, i32), NavPathError> {
        let mut nav_pass = cached_nav_pass(coords).unwrap_or(read_nav_pass(turf)?);
        if nav_pass & BAKED_FLAG == 0 {
            // Bake dirty turfs before trusting their edge bits.
            let _: () = turf.call("nav_bake", std::iter::empty::<ByondValue>())?;
            nav_pass = read_nav_pass(turf)?;
            if nav_pass & BAKED_FLAG == 0 {
                return Ok((0, nav_pass));
            }
            cache_nav_pass(coords, nav_pass);
        }

        let class_shift = if self.is_flying { FLYING_SHIFT } else { 0 };
        let mut open = 0;
        for dir in CARDINALS {
            if nav_pass & (dir << class_shift) == 0 {
                continue;
            }
            // Conditional edges require a live DM blocker check.
            if nav_pass & (dir << COND_SHIFT) == 0
                || self.evaluate_conditional_edge(turf, dir, pass_info)?
            {
                open |= dir;
            }
        }
        Ok((open, nav_pass))
    }

    /// Checks live blockers for one conditional edge.
    pub(super) fn evaluate_conditional_edge(
        &self,
        turf: &ByondValue,
        dir: i32,
        pass_info: &ByondValue,
    ) -> Result<bool, NavPathError> {
        tracy_span!("navmap.conditional_edge");
        let blockers_list: ByondValue = turf.read_var("nav_blockers")?;
        if blockers_list.is_null() {
            return Ok(true);
        }
        let entries = match blockers_list
            .read_list_index::<_, ByondValue>(&ByondValue::new_num(dir as f32))
        {
            Ok(value) if value.is_list() => value.read_list()?,
            _ => return Ok(true),
        };

        for entry in entries {
            if entry.is_number() {
                // Numeric blockers encode the pass_flags needed for this edge.
                if self.pass_flags & entry.get_number()? as i32 == 0 {
                    return Ok(false);
                }
                continue;
            }

            let blocker_loc: ByondValue = entry.read_var("loc")?;
            // Blockers are stored on the outgoing edge. A blocker on the destination
            // therefore sees the reverse direction when deciding whether it can be crossed.
            let eval_dir = if blocker_loc == *turf {
                dir
            } else {
                reverse_dir(dir)
            };
            let result = entry.call(
                "CanAStarPass",
                &[ByondValue::new_num(eval_dir as f32), pass_info.clone()],
            )?;
            if !truthy(&result) {
                return Ok(false);
            }
        }
        Ok(true)
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
    ) -> Result<Vec<(Coord, u32)>, NavPathError> {
        tracy_span!("navmap.grid_successors");
        let Some(source) = self.lookup(from.0, from.1, pass_info)? else {
            return Ok(Vec::new());
        };
        let mut out = Vec::with_capacity(8);
        for (step, dx, dy, cost) in STEPS {
            let to = (from.0 + dx, from.1 + dy);
            let walkable = if dx == 0 || dy == 0 {
                source.open_edges & step != 0 && self.can_occupy(to, pass_info)?
            } else {
                let routes = self.diagonal_routes_from_source(source, from, to, pass_info, true)?;
                routes.north_south_first || routes.east_west_first
            };
            if walkable {
                out.push((to, cost));
            }
        }
        Ok(out)
    }
}
