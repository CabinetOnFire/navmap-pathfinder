//! Shared generic A* core and single-z search specialization.

use super::*;
use std::cmp::Ordering;
use std::hash::Hash;

#[derive(Clone, Copy, Eq, PartialEq)]
struct NavLinkQueueEntry<N> {
    node: N,
    cost: u32,
    sequence: u64,
}

impl<N: Eq> Ord for NavLinkQueueEntry<N> {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .cmp(&self.cost)
            .then_with(|| other.sequence.cmp(&self.sequence))
    }
}

impl<N: Eq> PartialOrd for NavLinkQueueEntry<N> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct QueueEntry<N> {
    node: N,
    cost: u32,
    estimate: u32,
    sequence: u64,
}

impl<N: Eq> Ord for QueueEntry<N> {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .estimate
            .cmp(&self.estimate)
            .then_with(|| other.cost.cmp(&self.cost))
            .then_with(|| other.sequence.cmp(&self.sequence))
    }
}

impl<N: Eq> PartialOrd for QueueEntry<N> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Progress from one slice of the shared A* core.
pub(super) enum SearchProgress<N> {
    InProgress,
    Found(Vec<N>, Vec<SearchAction>),
    NoPath,
    Stale,
    StaleCosts,
    StalePassFlags,
}

/// Shared A* frontier, cost relaxation, stale-entry handling, and path reconstruction.
pub(super) struct AStarCore<N> {
    frontier: BinaryHeap<QueueEntry<N>>,
    costs: HashMap<N, u32>,
    previous: HashMap<N, (N, SearchAction)>,
    initialized: bool,
    sequence: u64,
}

impl<N: Eq> Default for AStarCore<N> {
    fn default() -> Self {
        Self {
            frontier: BinaryHeap::new(),
            costs: HashMap::default(),
            previous: HashMap::default(),
            initialized: false,
            sequence: 0,
        }
    }
}

impl<N> AStarCore<N>
where
    N: Copy + Eq + Hash,
{
    /// Advances one statically dispatched A* slice over a specialized grid.
    #[cfg(test)]
    pub(super) fn step<E, IsGoal, Successors, Heuristic>(
        &mut self,
        max_cost: Option<u32>,
        is_goal: IsGoal,
        successors: Successors,
        heuristic: Heuristic,
    ) -> Result<SearchProgress<N>, E>
    where
        IsGoal: FnMut(N) -> bool,
        Successors: FnMut(N) -> Result<Vec<(N, u32, SearchAction)>, E>,
        Heuristic: FnMut(N) -> Option<u32>,
    {
        self.step_until(
            Instant::now() + SLICE_BUDGET,
            max_cost,
            is_goal,
            successors,
            heuristic,
        )
    }

    pub(super) fn step_until<E, IsGoal, Successors, Heuristic>(
        &mut self,
        deadline: Instant,
        max_cost: Option<u32>,
        mut is_goal: IsGoal,
        mut successors: Successors,
        mut heuristic: Heuristic,
    ) -> Result<SearchProgress<N>, E>
    where
        IsGoal: FnMut(N) -> bool,
        Successors: FnMut(N) -> Result<Vec<(N, u32, SearchAction)>, E>,
        Heuristic: FnMut(N) -> Option<u32>,
    {
        while Instant::now() < deadline {
            let Some(current) = self.frontier.pop() else {
                return Ok(SearchProgress::NoPath);
            };
            if self.costs.get(&current.node).copied() != Some(current.cost) {
                // A cheaper route was queued after this entry.
                continue;
            }
            if is_goal(current.node) {
                let (nodes, actions) = self.reconstruct(current.node);
                return Ok(SearchProgress::Found(nodes, actions));
            }
            for (next, movement_cost, action) in successors(current.node)? {
                let next_cost = current.cost.saturating_add(movement_cost);
                if max_cost.is_some_and(|limit| limit > 0 && next_cost > limit) {
                    continue;
                }
                let Some(estimate) = heuristic(next) else {
                    continue;
                };
                if self.costs.get(&next).is_none_or(|&old| next_cost < old) {
                    self.previous.insert(next, (current.node, action));
                    self.push(next, next_cost, estimate);
                }
            }
        }
        Ok(SearchProgress::InProgress)
    }

    pub(super) fn is_initialized(&self) -> bool {
        self.initialized
    }

    pub(super) fn initialize(&mut self, start: N, estimate: u32) {
        self.initialized = true;
        self.push(start, 0, estimate);
    }

    fn push(&mut self, node: N, cost: u32, estimate: u32) {
        self.costs.insert(node, cost);
        self.sequence += 1;
        self.frontier.push(QueueEntry {
            node,
            cost,
            estimate: cost.saturating_add(estimate),
            sequence: self.sequence,
        });
    }

    fn reconstruct(&self, mut node: N) -> (Vec<N>, Vec<SearchAction>) {
        let mut nodes = Vec::new();
        let mut actions = Vec::new();
        loop {
            nodes.push(node);
            if let Some((previous, action)) = self.previous.get(&node).copied() {
                actions.push(action);
                node = previous;
            } else {
                actions.push(SearchAction::None);
                break;
            }
        }
        nodes.reverse();
        actions.reverse();
        (nodes, actions)
    }
}

/// Incrementally builds relaxed source-to-goal estimates for directed navigation links.
pub(super) struct NavLinkDistanceBuilder<N> {
    goal: N,
    min_target_distance: i32,
    edges: Vec<(N, N, u32)>,
    nodes: Vec<N>,
    node_set: HashSet<N>,
    distances: HashMap<N, u32>,
    frontier: BinaryHeap<NavLinkQueueEntry<N>>,
    sequence: u64,
    collect_index: usize,
    seed_index: usize,
    current: Option<NavLinkQueueEntry<N>>,
    candidate_index: usize,
    edge_index: usize,
    source_distances: Option<Vec<(N, u32)>>,
}

impl<N> NavLinkDistanceBuilder<N>
where
    N: Copy + Eq + Hash,
{
    pub(super) fn new(goal: N, min_target_distance: i32, edges: Vec<(N, N, u32)>) -> Self {
        let mut builder = Self {
            goal,
            min_target_distance,
            edges,
            nodes: Vec::new(),
            node_set: HashSet::default(),
            distances: HashMap::default(),
            frontier: BinaryHeap::new(),
            sequence: 0,
            collect_index: 0,
            seed_index: 0,
            current: None,
            candidate_index: 0,
            edge_index: 0,
            source_distances: None,
        };
        builder.node_set.insert(goal);
        builder.nodes.push(goal);
        builder
    }

    /// Returns true when all link estimates are ready.
    pub(super) fn advance<Relaxed>(&mut self, deadline: Instant, relaxed_distance: Relaxed) -> bool
    where
        Relaxed: FnMut(N, N, i32) -> Option<u32>,
    {
        self.advance_with_clock_check::<64, _>(deadline, relaxed_distance)
    }

    #[cfg(test)]
    pub(super) fn advance_checking_every_work_unit<Relaxed>(
        &mut self,
        deadline: Instant,
        relaxed_distance: Relaxed,
    ) -> bool
    where
        Relaxed: FnMut(N, N, i32) -> Option<u32>,
    {
        self.advance_with_clock_check::<1, _>(deadline, relaxed_distance)
    }

    fn advance_with_clock_check<const CHECK_INTERVAL: usize, Relaxed>(
        &mut self,
        deadline: Instant,
        mut relaxed_distance: Relaxed,
    ) -> bool
    where
        Relaxed: FnMut(N, N, i32) -> Option<u32>,
    {
        let mut work_since_clock_check = 0;
        loop {
            if work_since_clock_check == 0 && Instant::now() >= deadline {
                return false;
            }
            work_since_clock_check = (work_since_clock_check + 1) % CHECK_INTERVAL;
            if self.collect_index < self.edges.len() {
                let (source, destination, _) = self.edges[self.collect_index];
                self.collect_index += 1;
                if self.node_set.insert(source) {
                    self.nodes.push(source);
                }
                if self.node_set.insert(destination) {
                    self.nodes.push(destination);
                }
                continue;
            }
            if self.seed_index < self.nodes.len() {
                let node = self.nodes[self.seed_index];
                self.seed_index += 1;
                if let Some(cost) = relaxed_distance(node, self.goal, self.min_target_distance) {
                    relax_nav_link_distance(
                        node,
                        cost,
                        &mut self.distances,
                        &mut self.frontier,
                        &mut self.sequence,
                    );
                }
                continue;
            }
            if self.current.is_none() {
                let Some(current) = self.frontier.pop() else {
                    self.source_distances = Some(
                        self.edges
                            .iter()
                            .filter_map(|&(source, _, _)| {
                                Some((source, *self.distances.get(&source)?))
                            })
                            .collect(),
                    );
                    return true;
                };
                if self.distances.get(&current.node).copied() != Some(current.cost) {
                    continue;
                }
                self.current = Some(current);
                self.candidate_index = 0;
                self.edge_index = 0;
            }
            let Some(current) = self.current else {
                continue;
            };
            if self.candidate_index < self.nodes.len() {
                let candidate = self.nodes[self.candidate_index];
                self.candidate_index += 1;
                if let Some(edge_cost) = relaxed_distance(candidate, current.node, 0) {
                    relax_nav_link_distance(
                        candidate,
                        current.cost.saturating_add(edge_cost),
                        &mut self.distances,
                        &mut self.frontier,
                        &mut self.sequence,
                    );
                }
                continue;
            }
            if self.edge_index < self.edges.len() {
                let (source, destination, link_cost) = self.edges[self.edge_index];
                self.edge_index += 1;
                if destination == current.node {
                    relax_nav_link_distance(
                        source,
                        current.cost.saturating_add(link_cost),
                        &mut self.distances,
                        &mut self.frontier,
                        &mut self.sequence,
                    );
                }
                continue;
            }
            self.current = None;
        }
    }

    pub(super) fn finish(self) -> Option<Vec<(N, u32)>> {
        self.source_distances
    }
}

fn relax_nav_link_distance<N: Copy + Eq + Hash>(
    node: N,
    cost: u32,
    distances: &mut HashMap<N, u32>,
    frontier: &mut BinaryHeap<NavLinkQueueEntry<N>>,
    sequence: &mut u64,
) {
    if distances.get(&node).is_none_or(|&old| cost < old) {
        distances.insert(node, cost);
        *sequence += 1;
        frontier.push(NavLinkQueueEntry {
            node,
            cost,
            sequence: *sequence,
        });
    }
}

/// Estimates a node through either direct geometry or any navigation-link source.
pub(super) fn nav_link_estimate<N, Relaxed>(
    node: N,
    goal: N,
    min_target_distance: i32,
    source_distances: &[(N, u32)],
    mut relaxed_distance: Relaxed,
) -> Option<u32>
where
    N: Copy,
    Relaxed: FnMut(N, N, i32) -> Option<u32>,
{
    let mut best = relaxed_distance(node, goal, min_target_distance);
    for &(source, distance) in source_distances {
        let Some(to_source) = relaxed_distance(node, source, 0) else {
            continue;
        };
        let through_link = to_source.saturating_add(distance);
        best = Some(best.map_or(through_link, |estimate| estimate.min(through_link)));
    }
    best
}

impl FlatNavLinkHeuristic {
    pub(super) fn from_source_distances(
        goal: Coord,
        min_target_distance: i32,
        source_distances: Vec<(Coord, u32)>,
    ) -> Self {
        Self {
            goal,
            min_target_distance,
            source_distances,
            builder: None,
        }
    }

    pub(super) fn with_builder(mut self, builder: NavLinkDistanceBuilder<Coord>) -> Self {
        self.builder = Some(builder);
        self
    }

    pub(super) fn estimate(&self, node: Coord) -> Option<u32> {
        nav_link_estimate(
            node,
            self.goal,
            self.min_target_distance,
            &self.source_distances,
            |from, to, distance| Some(heuristic_to_target_range(from, to, distance)),
        )
    }
}

/// Chooses a cardinal intermediate for a diagonal, or fails when a forced expansion is blocked.
pub(super) fn diagonal_intermediate(
    from: Coord,
    to: Coord,
    handling: DiagonalHandling,
    routes: DiagonalRoutes,
) -> Result<Option<Coord>, ()> {
    match handling {
        DiagonalHandling::DoNothing => Ok(None),
        DiagonalHandling::RemoveAll if routes.north_south_first => Ok(Some((from.0, to.1))),
        DiagonalHandling::RemoveAll if routes.east_west_first => Ok(Some((to.0, from.1))),
        DiagonalHandling::RemoveClunky if !routes.north_south_first && routes.east_west_first => {
            Ok(Some((to.0, from.1)))
        }
        DiagonalHandling::RemoveAll => Err(()),
        DiagonalHandling::RemoveClunky => Ok(None),
    }
}

type ExpandedPath<N> = (Vec<N>, Vec<SearchAction>);

/// Expands diagonal nodes while preserving the action associated with each destination.
pub(super) fn expand_path_diagonals<N, E, Coordinates, MakeIntermediate, CanExpand, Routes>(
    nodes: &[N],
    actions: &[SearchAction],
    handling: DiagonalHandling,
    mut coordinates: Coordinates,
    mut make_intermediate: MakeIntermediate,
    mut can_expand: CanExpand,
    mut routes_for: Routes,
) -> Result<Option<ExpandedPath<N>>, E>
where
    N: Copy,
    Coordinates: FnMut(N) -> Coord,
    MakeIntermediate: FnMut(N, Coord) -> N,
    CanExpand: FnMut(N, N, SearchAction) -> bool,
    Routes: FnMut(N, N) -> Result<DiagonalRoutes, E>,
{
    let Some((&start, rest)) = nodes.split_first() else {
        return Ok(Some((Vec::new(), Vec::new())));
    };
    let mut expanded_nodes = vec![start];
    let mut expanded_actions = vec![SearchAction::None];
    let mut previous = start;
    for (index, &next) in rest.iter().enumerate() {
        let action = actions.get(index + 1).copied().unwrap_or_default();
        let from_coord = coordinates(previous);
        let to_coord = coordinates(next);
        if can_expand(previous, next, action)
            && from_coord.0 != to_coord.0
            && from_coord.1 != to_coord.1
        {
            let intermediate = if let SearchAction::Diagonal(x, y) = action {
                Ok(Some((x, y)))
            } else {
                let routes = routes_for(previous, next)?;
                diagonal_intermediate(from_coord, to_coord, handling, routes)
            };
            match intermediate {
                Ok(Some(intermediate)) => {
                    expanded_nodes.push(make_intermediate(previous, intermediate));
                    expanded_actions.push(SearchAction::None);
                }
                Ok(None) => {}
                Err(()) => return Ok(None),
            }
        }
        expanded_nodes.push(next);
        expanded_actions.push(action);
        previous = next;
    }
    Ok(Some((expanded_nodes, expanded_actions)))
}

impl SearchJob {
    pub(super) fn cost_changes_stale(&mut self) -> bool {
        if !self.grid.costs.has_modifiers() {
            return false;
        }
        let grid = &self.grid;
        let Some(generation) = cost_generation_since(
            self.grid.cost_generation,
            self.grid.costs.modifier_mask(),
            |(x, y, z)| {
                if z != grid.z {
                    return None;
                }
                grid.cache
                    .get(&(x, y))
                    .and_then(|info| info.as_ref())
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
        if self.topology_generation.is_some_and(|generation| {
            topology_generation() != generation || nav_link_generation() != generation
        }) {
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

    /// Creates a single-z search with the shared generic A* core.
    pub(super) fn new(
        grid: Grid,
        goal: Coord,
        min_target_distance: i32,
        diagonal_handling: DiagonalHandling,
        skip_first: bool,
        topology_generation: Option<u64>,
        heuristic: Option<FlatNavLinkHeuristic>,
    ) -> Self {
        Self {
            grid,
            goal,
            min_target_distance,
            diagonal_handling,
            skip_first,
            search: AStarCore::default(),
            topology_generation,
            heuristic,
            last_touched: Instant::now(),
        }
    }

    /// Advances the search for one time slice.
    pub(super) fn step(
        &mut self,
        pass_info: &ByondValue,
    ) -> Result<SearchProgress<Coord>, NavPathError> {
        self.last_touched = Instant::now();
        if self.topology_generation.is_some_and(|generation| {
            topology_generation() != generation || nav_link_generation() != generation
        }) {
            return Ok(SearchProgress::Stale);
        }
        if pass_flags_from_byond(pass_info)? != self.grid.pass_flags {
            return Ok(SearchProgress::StalePassFlags);
        }
        if self.cost_changes_stale() {
            return Ok(SearchProgress::StaleCosts);
        }
        let deadline = Instant::now() + SLICE_BUDGET;
        if let Some(heuristic) = &mut self.heuristic {
            let heuristic_ready = if let Some(builder) = &mut heuristic.builder {
                builder.advance(deadline, |from, to, distance| {
                    Some(heuristic_to_target_range(from, to, distance))
                })
            } else {
                true
            };
            if !heuristic_ready {
                return Ok(SearchProgress::InProgress);
            }
            if heuristic.builder.is_some() {
                heuristic.source_distances = heuristic
                    .builder
                    .take()
                    .and_then(NavLinkDistanceBuilder::finish)
                    .unwrap_or_default();
                heuristic.builder = None;
            }
        }
        let goal = self.goal;
        let min_target_distance = self.min_target_distance;
        let heuristic_scale = self.grid.costs.heuristic_scale;
        if !self.search.is_initialized() {
            if !self.grid.can_occupy(self.grid.start, pass_info)? {
                return Ok(SearchProgress::NoPath);
            }
            let estimate = self
                .heuristic
                .as_ref()
                .and_then(|heuristic| heuristic.estimate(self.grid.start))
                .unwrap_or_else(|| {
                    heuristic_to_target_range(self.grid.start, goal, min_target_distance)
                });
            self.search.initialize(
                self.grid.start,
                CostProfile::scale_heuristic(estimate, heuristic_scale),
            );
        }
        let progress = self.search.step_until(
            deadline,
            Some(self.grid.max_cost),
            |coord| chebyshev_distance(coord, goal) <= min_target_distance,
            |coord| self.grid.successors(coord, pass_info),
            |coord| {
                let estimate = self
                    .heuristic
                    .as_ref()
                    .and_then(|heuristic| heuristic.estimate(coord))
                    .unwrap_or_else(|| heuristic_to_target_range(coord, goal, min_target_distance));
                Some(CostProfile::scale_heuristic(estimate, heuristic_scale))
            },
        )?;
        if self.cost_changes_stale() {
            return Ok(SearchProgress::StaleCosts);
        }
        Ok(progress)
    }

    /// Rechecks and converts a coordinate path and action sidecar to DM values.
    pub(super) fn final_path(
        &mut self,
        nodes: Vec<Coord>,
        actions: Vec<SearchAction>,
        pass_info: &ByondValue,
    ) -> Result<(ByondValue, ByondValue), NavPathError> {
        let Some((nodes, mut actions)) = expand_path_diagonals(
            &nodes,
            &actions,
            self.diagonal_handling,
            |node| node,
            |_, coord| coord,
            |_, _, action| matches!(action, SearchAction::None | SearchAction::Diagonal(_, _)),
            |from, to| self.grid.diagonal_routes(from, to, pass_info),
        )?
        else {
            return empty_list_pair();
        };
        let nodes = if self.skip_first && !nodes.is_empty() {
            actions.remove(0);
            nodes.into_iter().skip(1).collect()
        } else {
            nodes
        };
        let mut turfs = Vec::with_capacity(nodes.len());
        for (x, y) in nodes {
            let turf = locate_xyz(ByondXYZ::new(x, y, self.grid.z))?;
            if turf.is_null() {
                return empty_list_pair();
            }
            turfs.push(turf);
        }
        Ok((turfs.to_byond()?, actions_to_byond(actions)?))
    }
}
