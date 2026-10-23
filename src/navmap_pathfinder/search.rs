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
}

/// Shared A* frontier, cost relaxation, stale-entry handling, and path reconstruction.
pub(super) struct AStarCore<N> {
    frontier: BinaryHeap<QueueEntry<N>>,
    costs: HashMap<N, u32>,
    previous: HashMap<N, (N, SearchAction)>,
    initialized: bool,
    sequence: u64,
}

impl<N> Default for AStarCore<N> {
    fn default() -> Self {
        Self {
            frontier: BinaryHeap::new(),
            costs: HashMap::new(),
            previous: HashMap::new(),
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
    pub(super) fn step<E, IsGoal, Successors, Heuristic>(
        &mut self,
        max_cost: Option<u32>,
        mut is_goal: IsGoal,
        mut successors: Successors,
        mut heuristic: Heuristic,
    ) -> Result<SearchProgress<N>, E>
    where
        IsGoal: FnMut(N) -> bool,
        Successors: FnMut(N) -> Result<Vec<(N, u32, SearchAction)>, E>,
        Heuristic: FnMut(N) -> u32,
    {
        let started = Instant::now();
        while started.elapsed() < SLICE_BUDGET {
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
                if self.costs.get(&next).is_none_or(|&old| next_cost < old) {
                    self.previous.insert(next, (current.node, action));
                    self.push(next, next_cost, heuristic(next));
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

/// Builds relaxed source-to-goal estimates for directed navigation links.
pub(super) fn nav_link_source_distances<N, Relaxed>(
    goal: N,
    min_target_distance: i32,
    nav_link_edges: &[(N, N, u32)],
    mut relaxed_distance: Relaxed,
) -> Vec<(N, u32)>
where
    N: Copy + Eq + Hash,
    Relaxed: FnMut(N, N, i32) -> Option<u32>,
{
    let mut nav_links = vec![goal];
    for &(source, destination, _) in nav_link_edges {
        if !nav_links.contains(&source) {
            nav_links.push(source);
        }
        if !nav_links.contains(&destination) {
            nav_links.push(destination);
        }
    }

    let mut distances = HashMap::new();
    let mut frontier = BinaryHeap::new();
    let mut sequence = 0;
    for &nav_link in &nav_links {
        if let Some(cost) = relaxed_distance(nav_link, goal, min_target_distance)
            && distances.get(&nav_link).is_none_or(|&old| cost < old)
        {
            distances.insert(nav_link, cost);
            sequence += 1;
            frontier.push(NavLinkQueueEntry {
                node: nav_link,
                cost,
                sequence,
            });
        }
    }

    while let Some(current) = frontier.pop() {
        if distances.get(&current.node).copied() != Some(current.cost) {
            continue;
        }
        for &candidate in &nav_links {
            let Some(edge_cost) = relaxed_distance(candidate, current.node, 0) else {
                continue;
            };
            relax_nav_link_distance(
                candidate,
                current.cost.saturating_add(edge_cost),
                &mut distances,
                &mut frontier,
                &mut sequence,
            );
        }
        for &(source, destination, nav_link_cost) in nav_link_edges {
            if destination == current.node {
                relax_nav_link_distance(
                    source,
                    current.cost.saturating_add(nav_link_cost),
                    &mut distances,
                    &mut frontier,
                    &mut sequence,
                );
            }
        }
    }

    nav_link_edges
        .iter()
        .filter_map(|&(source, _, _)| Some((source, *distances.get(&source)?)))
        .collect()
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
) -> u32
where
    N: Copy,
    Relaxed: FnMut(N, N, i32) -> Option<u32>,
{
    let mut best = relaxed_distance(node, goal, min_target_distance).unwrap_or(0);
    for &(source, distance) in source_distances {
        let Some(to_source) = relaxed_distance(node, source, 0) else {
            continue;
        };
        best = best.min(to_source.saturating_add(distance));
    }
    best
}

impl FlatNavLinkHeuristic {
    pub(super) fn new(goal: Coord, min_target_distance: i32, nav_links: &[NavLink]) -> Self {
        let edges: Vec<_> = nav_links
            .iter()
            .map(|nav_link| {
                (
                    (nav_link.source.0, nav_link.source.1),
                    (nav_link.destination.0, nav_link.destination.1),
                    nav_link.cost,
                )
            })
            .collect();
        let source_distances =
            nav_link_source_distances(goal, min_target_distance, &edges, |from, to, distance| {
                Some(heuristic_to_target_range(from, to, distance))
            });
        Self {
            goal,
            min_target_distance,
            source_distances,
        }
    }

    pub(super) fn estimate(&self, node: Coord) -> u32 {
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

/// Expands diagonal nodes while preserving the action associated with each destination.
pub(super) fn expand_path_diagonals<N, E, Coordinates, MakeIntermediate, CanExpand, Routes>(
    nodes: &[N],
    actions: &[SearchAction],
    handling: DiagonalHandling,
    mut coordinates: Coordinates,
    mut make_intermediate: MakeIntermediate,
    mut can_expand: CanExpand,
    mut routes_for: Routes,
) -> Result<Option<(Vec<N>, Vec<SearchAction>)>, E>
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
        if self
            .topology_generation
            .is_some_and(|generation| topology_generation() != generation)
        {
            return Ok(SearchProgress::Stale);
        }
        if self.grid.costs.has_modifiers() && nav_cost_generation() != self.grid.cost_generation {
            return Ok(SearchProgress::StaleCosts);
        }
        let goal = self.goal;
        let min_target_distance = self.min_target_distance;
        if !self.search.is_initialized() {
            if !self.grid.can_occupy(self.grid.start, pass_info)? {
                return Ok(SearchProgress::NoPath);
            }
            let estimate = self.heuristic.as_ref().map_or_else(
                || heuristic_to_target_range(self.grid.start, goal, min_target_distance),
                |heuristic| heuristic.estimate(self.grid.start),
            );
            self.search.initialize(self.grid.start, estimate);
        }
        let progress = self.search.step(
            Some(self.grid.max_cost),
            |coord| chebyshev_distance(coord, goal) <= min_target_distance,
            |coord| self.grid.successors(coord, pass_info),
            |coord| {
                self.heuristic.as_ref().map_or_else(
                    || heuristic_to_target_range(coord, goal, min_target_distance),
                    |heuristic| heuristic.estimate(coord),
                )
            },
        )?;
        if self.grid.costs.has_modifiers() && nav_cost_generation() != self.grid.cost_generation {
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
