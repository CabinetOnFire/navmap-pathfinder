//! Single-z turf lookup, blocker evaluation, and successor generation.

use super::*;

/// Resolves a turf snapshot for either grid specialization.
pub(super) fn resolve_turf(
    coords: TurfCoords,
    is_flying: bool,
    include_vertical: bool,
    pass_flags: i32,
    pass_info: &ByondValue,
) -> Result<Option<TurfInfo>, NavPathError> {
    tracy_span!("navmap.resolve_turf");
    let cached = cached_nav_data(coords);
    if let Some(data) = cached.filter(|data| data.nav_pass & BAKED_FLAG != 0)
        && let Some((open_edges, vertical_edges)) =
            cached_unconditional_edges(data.nav_pass, is_flying)
    {
        return Ok(Some(TurfInfo {
            open_edges: open_edges
                | (if include_vertical {
                    vertical_edges << 24
                } else {
                    0
                }),
            simulated: nav_pass_is_simulated(data.nav_pass),
            cost_flags: data.nav_cost_flags,
            cost_generation: nav_cost_generation(),
        }));
    }

    let turf = locate_xyz(ByondXYZ::new(coords.0, coords.1, coords.2))?;
    if turf.is_null() {
        return Ok(None);
    }
    let (mut nav_pass, mut cost_flags) = if let Some(data) = cached {
        (data.nav_pass, data.nav_cost_flags)
    } else {
        (read_nav_pass(&turf)?, read_nav_cost_flags(&turf)?)
    };
    if nav_pass & BAKED_FLAG == 0 {
        let _: () = turf.call("nav_bake", std::iter::empty::<ByondValue>())?;
        nav_pass = read_nav_pass(&turf)?;
        cost_flags = read_nav_cost_flags(&turf)?;
    }
    cache_nav_data(coords, nav_pass, cost_flags)?;
    // nav_bake may publish a new cost snapshot. Stamp the resolved edges with the generation
    // that observed that snapshot, before conditional-blocker callbacks can interleave changes.
    let cost_generation = nav_cost_generation();
    if nav_pass & BAKED_FLAG == 0 {
        return Ok(Some(TurfInfo {
            open_edges: 0,
            simulated: nav_pass_is_simulated(nav_pass),
            cost_flags,
            cost_generation,
        }));
    }
    let (open_edges, vertical_edges) =
        resolve_packed_edges(nav_pass, is_flying, include_vertical, |dir| {
            evaluate_conditional_edge(&turf, dir, pass_flags, pass_info)
        })?;
    Ok(Some(TurfInfo {
        open_edges: open_edges
            | (if include_vertical {
                vertical_edges << 24
            } else {
                0
            }),
        simulated: nav_pass_is_simulated(nav_pass),
        cost_flags,
        cost_generation,
    }))
}

/// Operations shared by the flat and layered grids for horizontal movement.
pub(super) trait HorizontalGridAccess<N: Copy> {
    fn horizontal_info(
        &mut self,
        node: N,
        pass_info: &ByondValue,
    ) -> Result<Option<TurfInfo>, NavPathError>;
    fn horizontal_node(&self, reference: N, position: Coord) -> N;
    fn horizontal_position(&self, node: N) -> Coord;
    fn horizontal_cost(&mut self, base: u32, flags: u32) -> u32;
    fn diagonal_handling(&self) -> DiagonalHandling;
}

pub(super) fn diagonal_routes<G, N>(
    grid: &mut G,
    source: TurfInfo,
    from: N,
    to: Coord,
    pass_info: &ByondValue,
) -> Result<DiagonalRoutes, NavPathError>
where
    G: HorizontalGridAccess<N>,
    N: Copy,
{
    let from_position = grid.horizontal_position(from);
    let dx = to.0 - from_position.0;
    let dy = to.1 - from_position.1;
    let north_south_dir = if dy > 0 { NORTH } else { SOUTH };
    let east_west_dir = if dx > 0 { EAST } else { WEST };
    let north_south_open = source.open_edges & north_south_dir != 0;
    let east_west_open = source.open_edges & east_west_dir != 0;
    if (!north_south_open && !east_west_open)
        || grid
            .horizontal_info(grid.horizontal_node(from, to), pass_info)?
            .is_none()
    {
        return Ok(DiagonalRoutes {
            north_south_first: false,
            east_west_first: false,
        });
    }
    let north_south_mid = (from_position.0, to.1);
    let east_west_mid = (to.0, from_position.1);
    Ok(DiagonalRoutes {
        north_south_first: north_south_open
            && grid
                .horizontal_info(grid.horizontal_node(from, north_south_mid), pass_info)?
                .is_some_and(|info| info.open_edges & east_west_dir != 0),
        east_west_first: east_west_open
            && grid
                .horizontal_info(grid.horizontal_node(from, east_west_mid), pass_info)?
                .is_some_and(|info| info.open_edges & north_south_dir != 0),
    })
}

/// Builds shared cardinal and diagonal successors while leaving vertical moves and links to each grid.
pub(super) fn horizontal_successors<G, N>(
    grid: &mut G,
    from: N,
    pass_info: &ByondValue,
) -> Result<Vec<(N, u32, SearchAction)>, NavPathError>
where
    G: HorizontalGridAccess<N>,
    N: Copy,
{
    let Some(source) = grid.horizontal_info(from, pass_info)? else {
        return Ok(Vec::new());
    };
    let from_position = grid.horizontal_position(from);
    let mut out = Vec::with_capacity(8);
    for (step, dx, dy, cost) in STEPS {
        let to = (from_position.0 + dx, from_position.1 + dy);
        let destination_node = grid.horizontal_node(from, to);
        if dx == 0 || dy == 0 {
            if source.open_edges & step == 0 {
                continue;
            }
            let Some(destination) = grid.horizontal_info(destination_node, pass_info)? else {
                continue;
            };
            out.push((
                destination_node,
                grid.horizontal_cost(cost, destination.cost_flags),
                SearchAction::None,
            ));
            continue;
        }

        let routes = diagonal_routes(grid, source, from, to, pass_info)?;
        if !routes.north_south_first && !routes.east_west_first {
            continue;
        }
        let Some(destination) = grid.horizontal_info(destination_node, pass_info)? else {
            continue;
        };
        let expand_all = grid.diagonal_handling() == DiagonalHandling::RemoveAll;
        let expand_clunky = grid.diagonal_handling() == DiagonalHandling::RemoveClunky
            && !routes.north_south_first
            && routes.east_west_first;
        if !expand_all && !expand_clunky {
            out.push((
                destination_node,
                grid.horizontal_cost(cost, destination.cost_flags),
                SearchAction::None,
            ));
            continue;
        }

        let mut selected: Option<(u32, Coord)> = None;
        for (route_valid, intermediate) in [
            (routes.north_south_first, (from_position.0, to.1)),
            (routes.east_west_first, (to.0, from_position.1)),
        ] {
            if !route_valid {
                continue;
            }
            let Some(mid) =
                grid.horizontal_info(grid.horizontal_node(from, intermediate), pass_info)?
            else {
                continue;
            };
            let candidate = grid
                .horizontal_cost(10, mid.cost_flags)
                .saturating_add(grid.horizontal_cost(10, destination.cost_flags));
            if selected.is_none_or(|(old, _)| candidate < old) {
                selected = Some((candidate, intermediate));
            }
        }
        if let Some((route_cost, (mid_x, mid_y))) = selected {
            out.push((
                destination_node,
                route_cost,
                SearchAction::Diagonal(mid_x, mid_y),
            ));
        }
    }
    Ok(out)
}

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

        let entry = resolve_turf(
            (x, y, self.z),
            self.is_flying,
            false,
            self.pass_flags,
            pass_info,
        )?;
        self.cache.insert((x, y), entry);
        Ok(entry)
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
        diagonal_routes(self, source, from, to, pass_info)
    }

    /// Returns all walkable cardinal and diagonal successors.
    pub(super) fn successors(
        &mut self,
        from: Coord,
        pass_info: &ByondValue,
    ) -> Result<Vec<(Coord, u32, SearchAction)>, NavPathError> {
        tracy_span!("navmap.grid_successors");
        let mut out = horizontal_successors(self, from, pass_info)?;
        let nav_links_by_source = Arc::clone(&self.nav_links_by_source);
        let source_z = self.z;
        if let Some(nav_links) = nav_links_by_source.get(&(from.0, from.1, source_z)) {
            for nav_link in nav_links
                .iter()
                .filter(|link| link.destination.2 == source_z)
            {
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

impl HorizontalGridAccess<Coord> for Grid {
    fn horizontal_info(
        &mut self,
        node: Coord,
        pass_info: &ByondValue,
    ) -> Result<Option<TurfInfo>, NavPathError> {
        self.occupiable_info(node, pass_info)
    }

    fn horizontal_node(&self, _reference: Coord, position: Coord) -> Coord {
        position
    }

    fn horizontal_position(&self, node: Coord) -> Coord {
        node
    }

    fn horizontal_cost(&mut self, base: u32, flags: u32) -> u32 {
        self.costs.step_cost(base, flags)
    }

    fn diagonal_handling(&self) -> DiagonalHandling {
        self.diagonal_handling
    }
}
