//! A* pathfinding over a /tg/station-style navmap; DM supplies packed passability data.
//! Searches yield in short resumable slices to avoid monopolizing the server tick.

use meowtonin::misc::locate_xyz;
use meowtonin::{ByondError, ByondValue, ByondXYZ, ToByond, byond_fn, call_global};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::sync::{
    LazyLock, Mutex,
    atomic::{AtomicU64, Ordering as AtomicOrdering},
};
use std::time::{Duration, Instant};
use thiserror::Error;

#[cfg(feature = "tracy")]
macro_rules! tracy_span {
    ($name:expr) => {
        let _tracy_span = tracy_client::span!($name);
    };
}

#[cfg(not(feature = "tracy"))]
macro_rules! tracy_span {
    ($name:expr) => {
        let _tracy_span = ();
    };
}

/// North direction bit.
const NORTH: i32 = 1;
/// South direction bit.
const SOUTH: i32 = 2;
/// East direction bit.
const EAST: i32 = 4;
/// West direction bit.
const WEST: i32 = 8;
/// Upward direction bit.
const UP: i32 = 1 << 4;
/// Downward direction bit.
const DOWN: i32 = 1 << 5;
/// Cardinal directions considered by A*.
const CARDINALS: [i32; 4] = [NORTH, SOUTH, EAST, WEST];

/// Flying-edge bit offset.
const FLYING_SHIFT: i32 = 6;
/// Conditional-edge bit offset.
const COND_SHIFT: i32 = 12;
/// Baked-turf flag.
const BAKED_FLAG: i32 = 1 << 18;
/// Simulated-turf flag.
const SIMULATED_FLAG: i32 = 1 << 19;

/// DM value for preserving diagonals.
const DIAGONAL_DO_NOTHING: i32 = 0;
/// DM value for removing every diagonal.
const DIAGONAL_REMOVE_ALL: i32 = 1;
/// DM value for removing only clunky diagonals.
const DIAGONAL_REMOVE_CLUNKY: i32 = 2;

/// Maximum time spent in one resumable search slice.
const SLICE_BUDGET: Duration = Duration::from_millis(5);
/// Time before an inactive search job expires.
const IDLE_TIMEOUT: Duration = Duration::from_secs(30);

/// Movement deltas and costs for cardinal and diagonal steps.
const STEPS: [(i32, i16, i16, u32); 8] = [
    (NORTH, 0, 1, 10),
    (SOUTH, 0, -1, 10),
    (EAST, 1, 0, 10),
    (WEST, -1, 0, 10),
    (NORTH | EAST, 1, 1, 14),
    (SOUTH | EAST, 1, -1, 14),
    (NORTH | WEST, -1, 1, 14),
    (SOUTH | WEST, -1, -1, 14),
];

/// A turf position including its z-level.
type TurfCoords = (i16, i16, i16);
/// A two-dimensional turf position.
type Coord = (i16, i16);

/// Cached packed navmap values received from DM.
static NAV_PASS_CACHE: LazyLock<Mutex<HashMap<TurfCoords, i32>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));
/// Active single-z search jobs.
static SEARCH_JOBS: LazyLock<Mutex<HashMap<u64, SearchJob>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));
/// Active multi-z search jobs.
static MULTI_SEARCH_JOBS: LazyLock<Mutex<HashMap<u64, MultiSearchJob>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));
/// Native multi-z topology.
static TOPOLOGY: LazyLock<Mutex<Topology>> = LazyLock::new(|| Mutex::new(Topology::default()));
/// Registered cross-z navigation links.
static NAV_LINKS: LazyLock<Mutex<HashMap<u64, NavLink>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));
/// Monotonic identifier source for search jobs.
static NEXT_JOB_ID: AtomicU64 = AtomicU64::new(1);

#[derive(Error, Debug)]
enum NavPathError {
    #[error(transparent)]
    Byond(#[from] ByondError),
    #[error("start and end turfs are on different z-levels")]
    DifferentZLevels,
    #[error("argument was not a turf")]
    NotATurf,
    #[error("navmap bulk update list length must be a multiple of 4")]
    InvalidBulkUpdateLength,
}

/// Controls how diagonal steps are returned to DM.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum DiagonalHandling {
    DoNothing,
    RemoveAll,
    RemoveClunky,
}

impl DiagonalHandling {
    /// Converts a DM numeric mode into the native enum.
    fn from_byond(value: &ByondValue) -> Self {
        match value.get_number().unwrap_or(DIAGONAL_DO_NOTHING as f32) as i32 {
            DIAGONAL_REMOVE_ALL => Self::RemoveAll,
            DIAGONAL_REMOVE_CLUNKY => Self::RemoveClunky,
            _ => Self::DoNothing,
        }
    }
}

#[derive(Clone, Copy)]
struct TurfInfo {
    /// Open cardinal edges for the current mover.
    open_edges: i32,
    /// Whether the turf is simulated rather than space.
    simulated: bool,
}

#[derive(Clone, Copy)]
struct DiagonalRoutes {
    /// Whether north/south-first is valid.
    north_south_first: bool,
    /// Whether east/west-first is valid.
    east_west_first: bool,
}

/// Single-z search grid and its per-job cache.
struct Grid {
    /// Search z-level.
    z: i16,
    /// Pass flags for conditional blockers.
    pass_flags: i32,
    /// Whether flying edges should be used.
    is_flying: bool,
    /// Search origin.
    start: Coord,
    /// Maximum horizontal search range.
    max_range: i32,
    /// Whether only simulated turfs are allowed.
    simulated_only: bool,
    /// Optional turf excluded from the search.
    avoid: Option<Coord>,
    /// Cached turf lookups.
    cache: HashMap<Coord, Option<TurfInfo>>,
}

impl Grid {
    /// Looks up and caches one turf's navigation data.
    fn lookup(
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
    fn resolve_edges(
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
    fn evaluate_conditional_edge(
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
        let entries = match blockers_list.read_list_index::<_, ByondValue>(&dir.to_string()) {
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
    fn occupiable_info(
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
    fn can_occupy(&mut self, coord: Coord, pass_info: &ByondValue) -> Result<bool, NavPathError> {
        Ok(self.occupiable_info(coord, pass_info)?.is_some())
    }

    /// Evaluates both cardinal routes around a diagonal.
    fn diagonal_routes_from_source(
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
    fn diagonal_routes(
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
    fn successors(
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

#[derive(Clone, Copy, Eq, PartialEq)]
struct QueueEntry {
    /// Turf represented by this queue entry.
    coord: Coord,
    /// Cost already spent reaching the turf.
    cost: u32,
    /// Cost plus the heuristic estimate.
    estimate: u32,
    /// Tie-breaker preserving queue order.
    sequence: u64,
}

impl Ord for QueueEntry {
    /// Orders entries by lowest estimated path cost.
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .estimate
            .cmp(&self.estimate)
            .then_with(|| other.cost.cmp(&self.cost))
            .then_with(|| other.sequence.cmp(&self.sequence))
    }
}

impl PartialOrd for QueueEntry {
    /// Delegates partial ordering to the total queue ordering.
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Result of one single-z search slice.
enum JobProgress {
    InProgress,
    Found(Vec<Coord>),
    NoPath,
}

/// Resumable single-z A* search state.
struct SearchJob {
    /// Grid and cached passability data.
    grid: Grid,
    /// Search target.
    goal: Coord,
    /// Acceptable distance from the target.
    min_target_distance: i32,
    /// Requested diagonal output mode.
    diagonal_handling: DiagonalHandling,
    /// Whether the first returned turf is omitted.
    skip_first: bool,
    /// Frontier ordered by estimated total cost.
    frontier: BinaryHeap<QueueEntry>,
    /// Best known cost for each visited turf.
    costs: HashMap<Coord, u32>,
    /// Previous turf for path reconstruction.
    previous: HashMap<Coord, Coord>,
    /// Whether the start node has been queued.
    initialized: bool,
    /// Sequence used to break queue ties.
    sequence: u64,
    /// Last time this job was resumed.
    last_touched: Instant,
}

impl SearchJob {
    /// Creates an empty resumable search.
    fn new(
        grid: Grid,
        goal: Coord,
        min_target_distance: i32,
        diagonal_handling: DiagonalHandling,
        skip_first: bool,
    ) -> Self {
        tracy_span!("navmap.portal_heuristic_build");
        Self {
            grid,
            goal,
            min_target_distance,
            diagonal_handling,
            skip_first,
            frontier: BinaryHeap::new(),
            costs: HashMap::new(),
            previous: HashMap::new(),
            initialized: false,
            sequence: 0,
            last_touched: Instant::now(),
        }
    }

    /// Advances the search for one time slice.
    fn step(&mut self, pass_info: &ByondValue) -> Result<JobProgress, NavPathError> {
        self.last_touched = Instant::now();
        let started = Instant::now();
        if !self.initialized {
            self.initialized = true;
            if !self.grid.can_occupy(self.grid.start, pass_info)? {
                return Ok(JobProgress::NoPath);
            }
            self.push(self.grid.start, 0);
        }

        while started.elapsed() < SLICE_BUDGET {
            let Some(current) = self.frontier.pop() else {
                return Ok(JobProgress::NoPath);
            };
            if self.costs.get(&current.coord).copied() != Some(current.cost) {
                // A cheaper route was queued after this entry.
                continue;
            }
            if chebyshev_distance(current.coord, self.goal) <= self.min_target_distance {
                // Reaching the requested radius is sufficient; the destination itself need not
                // be occupied.
                return Ok(JobProgress::Found(self.reconstruct(current.coord)));
            }
            for (next, movement_cost) in self.grid.successors(current.coord, pass_info)? {
                let next_cost = current.cost + movement_cost;
                if self.costs.get(&next).is_none_or(|&old| next_cost < old) {
                    self.previous.insert(next, current.coord);
                    self.push(next, next_cost);
                }
            }
        }
        // Return control to DM before a large search can monopolize the server tick.
        Ok(JobProgress::InProgress)
    }

    /// Queues a node with its current best cost.
    fn push(&mut self, coord: Coord, cost: u32) {
        self.costs.insert(coord, cost);
        self.sequence += 1;
        self.frontier.push(QueueEntry {
            coord,
            cost,
            estimate: cost + heuristic_to_target_range(coord, self.goal, self.min_target_distance),
            sequence: self.sequence,
        });
    }

    /// Reconstructs a start-to-goal coordinate path.
    fn reconstruct(&self, mut goal: Coord) -> Vec<Coord> {
        let mut path = vec![goal];
        while let Some(&previous) = self.previous.get(&goal) {
            path.push(previous);
            goal = previous;
        }
        path.reverse();
        path
    }

    /// Rechecks and converts a coordinate path to DM turfs.
    fn final_path(
        &mut self,
        nodes: Vec<Coord>,
        pass_info: &ByondValue,
    ) -> Result<ByondValue, NavPathError> {
        // Recheck each diagonal before returning it. This also supplies the cardinal step used
        // when the caller requests diagonals to be expanded.
        let Some(nodes) = expand_diagonals_checked(&nodes, self.diagonal_handling, |from, to| {
            self.grid.diagonal_routes(from, to, pass_info)
        })?
        else {
            return empty_list();
        };
        let mut turfs = Vec::new();
        for (x, y) in apply_skip_first(nodes, self.skip_first) {
            let turf = locate_xyz(ByondXYZ::new(x, y, self.grid.z))?;
            if turf.is_null() {
                return empty_list();
            }
            turfs.push(turf);
        }
        Ok(turfs.to_byond()?)
    }
}

#[derive(Clone, Copy, Debug, Default)]
/// Maps a z-level group to its ordered layer.
struct LayerLocation {
    /// Connected z-level group identifier.
    group: i32,
    /// Zero-based layer within the group.
    layer: i16,
}

#[derive(Clone, Debug, Default)]
/// Native multi-z topology snapshot.
struct Topology {
    /// Generation used to invalidate active searches.
    generation: u64,
    /// Mapping from z-level to group/layer.
    z_to_layer: HashMap<i16, LayerLocation>,
    /// Reverse mapping from group/layer to z-level.
    layer_to_z: HashMap<(i32, i16), i16>,
}

#[derive(Clone, Copy, Debug)]
/// A reusable cross-z transition.
struct NavLink {
    /// Stable link identifier.
    id: u64,
    /// Link source turf.
    source: TurfCoords,
    /// Link destination turf.
    destination: TurfCoords,
    /// Traversal cost.
    cost: u32,
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
/// A coordinate paired with its topology layer.
struct LayeredNode {
    /// Connected z-level group identifier.
    group: i32,
    /// Layer within the connected group.
    layer: i16,
    /// Horizontal x coordinate.
    x: i16,
    /// Horizontal y coordinate.
    y: i16,
}

#[derive(Clone, Copy, Debug)]
/// Action used to reach a multi-z node.
enum LayeredAction {
    None,
    Vertical(i32),
    Link(u64),
}

/// A lower-bound route model for multi-z A*.
///
/// The model deliberately ignores walls, conditional blockers, and mover-specific link
/// eligibility. It therefore cannot overestimate the real graph, while the portal nodes let it
/// account for useful cross-z links instead of collapsing the heuristic to zero everywhere.
struct PortalHeuristic {
    goal: LayeredNode,
    min_target_distance: i32,
    is_flying: bool,
    /// Relaxed distance from each link source to the target region.
    source_distances: Vec<(LayeredNode, u32)>,
}

#[derive(Clone, Copy, Eq, PartialEq)]
struct RelaxedQueueEntry {
    node: LayeredNode,
    cost: u32,
    sequence: u64,
}

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
    fn new(
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
            if let Some(cost) = relaxed_distance(portal, goal, min_target_distance, is_flying) {
                if distances.get(&portal).is_none_or(|&old| cost < old) {
                    distances.insert(portal, cost);
                    sequence += 1;
                    frontier.push(RelaxedQueueEntry {
                        node: portal,
                        cost,
                        sequence,
                    });
                }
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
/// Queue entry for multi-z A*.
struct LayeredQueueEntry {
    /// Node represented by this entry.
    node: LayeredNode,
    /// Cost already spent reaching the node.
    cost: u32,
    /// Cost plus the relaxed lower-bound estimate.
    estimate: u32,
    /// Tie-breaker preserving queue order.
    sequence: u64,
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

/// Multi-z search grid and its per-job cache.
struct LayeredGrid {
    /// Topology snapshot used by this search.
    topology: Topology,
    /// Pass flags for conditional blockers.
    pass_flags: i32,
    /// Whether flying edges should be used.
    is_flying: bool,
    /// Search origin node.
    start: LayeredNode,
    /// Maximum horizontal search range.
    max_range: i32,
    /// Maximum total path cost, or zero for unlimited.
    max_cost: u32,
    /// Whether only simulated turfs are allowed.
    simulated_only: bool,
    /// Optional node excluded from the search.
    avoid: Option<LayeredNode>,
    /// Cross-z links indexed by their source turf for O(out-degree) expansion.
    links_by_source: HashMap<TurfCoords, Vec<NavLink>>,
    /// Cached node lookups.
    cache: HashMap<LayeredNode, Option<TurfInfo>>,
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
enum MultiJobProgress {
    InProgress,
    Found(LayeredNode),
    NoPath,
    Stale,
}

/// Resumable multi-z A* search state.
struct MultiSearchJob {
    /// Layered grid and topology snapshot.
    grid: LayeredGrid,
    /// Search target node.
    goal: LayeredNode,
    /// Admissible lower-bound estimate including cross-z portals.
    heuristic: PortalHeuristic,
    /// Acceptable distance from the target.
    min_target_distance: i32,
    /// Requested diagonal output mode.
    diagonal_handling: DiagonalHandling,
    /// Whether the first returned turf is omitted.
    skip_first: bool,
    /// Frontier ordered by path cost.
    frontier: BinaryHeap<LayeredQueueEntry>,
    /// Best known cost for each visited node.
    costs: HashMap<LayeredNode, u32>,
    /// Previous node and action for reconstruction.
    previous: HashMap<LayeredNode, (LayeredNode, LayeredAction)>,
    /// Whether the start node has been queued.
    initialized: bool,
    /// Sequence used to break queue ties.
    sequence: u64,
    /// Last time this job was resumed.
    last_touched: Instant,
}

impl MultiSearchJob {
    /// Advances the search for one time slice.
    fn step(&mut self, pass_info: &ByondValue) -> Result<MultiJobProgress, NavPathError> {
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
    fn final_path(
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

/// Returns empty path and action lists for a multi-z result.
fn empty_list_pair() -> Result<(ByondValue, ByondValue), NavPathError> {
    Ok((empty_list()?, ByondValue::new_list()?))
}

/// Reads the current native topology generation.
fn topology_generation() -> u64 {
    TOPOLOGY
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .generation
}

/// Evaluates live blockers for one packed edge.
fn evaluate_conditional_edge(
    turf: &ByondValue,
    dir: i32,
    pass_flags: i32,
    pass_info: &ByondValue,
) -> Result<bool, NavPathError> {
    tracy_span!("navmap.conditional_edge_multiz");
    let blockers_list: ByondValue = turf.read_var("nav_blockers")?;
    if blockers_list.is_null() {
        return Ok(true);
    }
    let entries = match blockers_list.read_list_index::<_, ByondValue>(&dir.to_string()) {
        Ok(value) if value.is_list() => value.read_list()?,
        _ => return Ok(true),
    };
    for entry in entries {
        if entry.is_number() {
            if pass_flags & entry.get_number()? as i32 == 0 {
                return Ok(false);
            }
            continue;
        }
        let blocker_loc: ByondValue = entry.read_var("loc")?;
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

/// Converts world coordinates into a layered node.
fn node_for_xyz(topology: &Topology, x: i16, y: i16, z: i16) -> Option<LayeredNode> {
    let layer = topology.z_to_layer.get(&z)?;
    Some(LayeredNode {
        group: layer.group,
        layer: layer.layer,
        x,
        y,
    })
}

/// Builds a multi-z search from FFI arguments.
fn new_multi_search_job(
    start: ByondValue,
    end: ByondValue,
    pass_info: &ByondValue,
    is_flying: ByondValue,
    max_range: ByondValue,
    min_target_distance: ByondValue,
    simulated_only: ByondValue,
    avoid_turf: ByondValue,
    diagonal_handling: ByondValue,
    skip_first: ByondValue,
    max_path_cost: ByondValue,
) -> Result<Option<MultiSearchJob>, NavPathError> {
    let start_xyz = start.xyz().ok_or(NavPathError::NotATurf)?;
    let end_xyz = end.xyz().ok_or(NavPathError::NotATurf)?;
    let topology = TOPOLOGY.lock().unwrap_or_else(|p| p.into_inner()).clone();
    let start_node = node_for_xyz(&topology, start_xyz.x(), start_xyz.y(), start_xyz.z());
    let goal_node = node_for_xyz(&topology, end_xyz.x(), end_xyz.y(), end_xyz.z());
    let (Some(start_node), Some(goal_node)) = (start_node, goal_node) else {
        return Ok(None);
    };
    let max_range = max_range.get_number().unwrap_or(0.0).max(0.0) as i32;
    let min_target_distance = min_target_distance.get_number().unwrap_or(0.0).max(0.0) as i32;
    let max_path_cost = max_path_cost.get_number().unwrap_or(0.0).max(0.0) as u32;
    let avoid = if avoid_turf.is_null() {
        None
    } else {
        let xyz = avoid_turf.xyz().ok_or(NavPathError::NotATurf)?;
        node_for_xyz(&topology, xyz.x(), xyz.y(), xyz.z())
    };
    let pass_flags = pass_info
        .read_var::<_, ByondValue>("pass_flags")
        .and_then(|value| value.get_number())
        .map(|number| number as i32)
        .unwrap_or(0);
    let links: Vec<NavLink> = NAV_LINKS
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
        .values()
        .copied()
        .collect();
    let mut links_by_source: HashMap<TurfCoords, Vec<NavLink>> = HashMap::new();
    for link in &links {
        links_by_source.entry(link.source).or_default().push(*link);
    }
    let is_flying = truthy(&is_flying);
    Ok(Some(MultiSearchJob {
        grid: LayeredGrid {
            topology: topology.clone(),
            pass_flags,
            is_flying,
            start: start_node,
            max_range,
            max_cost: max_path_cost,
            simulated_only: truthy(&simulated_only),
            avoid,
            links_by_source,
            cache: HashMap::new(),
        },
        goal: goal_node,
        heuristic: PortalHeuristic::new(
            &topology,
            goal_node,
            min_target_distance,
            is_flying,
            &links,
        ),
        min_target_distance,
        diagonal_handling: DiagonalHandling::from_byond(&diagonal_handling),
        skip_first: truthy(&skip_first),
        frontier: BinaryHeap::new(),
        costs: HashMap::new(),
        previous: HashMap::new(),
        initialized: false,
        sequence: 0,
        last_touched: Instant::now(),
    }))
}

/// Reads one packed navmap value from the shared cache.
fn cached_nav_pass(coords: TurfCoords) -> Option<i32> {
    NAV_PASS_CACHE
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
        .get(&coords)
        .copied()
}

/// Decodes cached baked edges when no live conditional blocker lookup is needed.
///
/// A cached value may be used without locating the turf only when none of the selected
/// movement-class edges are conditional. Vertical edges are returned separately because the
/// layered search stores them outside the horizontal edge bit range in `TurfInfo`.
fn cached_unconditional_edges(nav_pass: i32, is_flying: bool) -> Option<(i32, i32)> {
    let class_shift = if is_flying { FLYING_SHIFT } else { 0 };
    let class_mask = CARDINALS
        .iter()
        .fold(0, |mask, &dir| mask | (dir << class_shift));
    if CARDINALS
        .iter()
        .any(|&dir| nav_pass & (dir << class_shift) != 0 && nav_pass & (dir << COND_SHIFT) != 0)
    {
        return None;
    }
    let open = (nav_pass & class_mask) >> class_shift;
    let vertical = if is_flying {
        let vertical_mask = (UP | DOWN) << FLYING_SHIFT;
        if [UP, DOWN].iter().any(|&dir| {
            nav_pass & (dir << FLYING_SHIFT) != 0 && nav_pass & (dir << COND_SHIFT) != 0
        }) {
            return None;
        }
        (nav_pass & vertical_mask) >> FLYING_SHIFT
    } else {
        0
    };
    Some((open, vertical))
}

/// Stores one packed navmap value in the shared cache.
fn cache_nav_pass(coords: TurfCoords, nav_pass: i32) {
    // DM publishes both baked values and invalidations here. Keeping an unbaked value is
    // intentional: resolve_edges will force a fresh bake before using it.
    NAV_PASS_CACHE
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
        .insert(coords, nav_pass);
}

/// Returns the opposite cardinal or vertical direction.
fn reverse_dir(dir: i32) -> i32 {
    match dir {
        NORTH => SOUTH,
        SOUTH => NORTH,
        EAST => WEST,
        WEST => EAST,
        UP => DOWN,
        DOWN => UP,
        other => other,
    }
}

/// Reads packed navmap data from a DM turf.
fn read_nav_pass(turf: &ByondValue) -> Result<i32, NavPathError> {
    Ok(turf
        .read_var::<_, ByondValue>("nav_pass")
        .and_then(|value| value.get_number())
        .map(|number| number as i32)
        .unwrap_or(0))
}

/// Converts a DM value to the pathfinder's truth value.
fn truthy(value: &ByondValue) -> bool {
    !value.is_null() && (!value.is_number() || value.get_number().is_ok_and(|number| number != 0.0))
}

/// Checks range and avoidance constraints for a coordinate.
fn coordinate_allowed(coord: Coord, start: Coord, max_range: i32, avoid: Option<Coord>) -> bool {
    (max_range <= 0 || chebyshev_distance(coord, start) <= max_range) && avoid != Some(coord)
}

/// Checks whether a turf matches the simulated-only filter.
fn turf_allowed(simulated_only: bool, simulated: bool) -> bool {
    !simulated_only || simulated
}

/// Checks the packed simulated-turf flag.
fn nav_pass_is_simulated(nav_pass: i32) -> bool {
    nav_pass & SIMULATED_FLAG != 0
}

/// Returns grid distance using the larger axis delta.
fn chebyshev_distance(a: Coord, b: Coord) -> i32 {
    (a.0 - b.0).unsigned_abs().max((a.1 - b.1).unsigned_abs()) as i32
}

/// Estimates remaining octile cost to the target radius.
fn heuristic_to_target_range(coord: Coord, goal: Coord, target_distance: i32) -> u32 {
    // Octile distance remains admissible for cardinal cost 10 and diagonal cost 14.
    let dx = ((coord.0 - goal.0).unsigned_abs() as i32 - target_distance).max(0) as u32;
    let dy = ((coord.1 - goal.1).unsigned_abs() as i32 - target_distance).max(0) as u32;
    14 * dx.min(dy) + 10 * dx.abs_diff(dy)
}

/// Returns a relaxed lower-bound cost between layered nodes.
fn relaxed_distance(
    from: LayeredNode,
    to: LayeredNode,
    target_distance: i32,
    is_flying: bool,
) -> Option<u32> {
    if from.group != to.group || (!is_flying && from.layer != to.layer) {
        return None;
    }
    let dx = ((from.x - to.x).unsigned_abs() as i32 - target_distance).max(0) as u32;
    let dy = ((from.y - to.y).unsigned_abs() as i32 - target_distance).max(0) as u32;
    let horizontal = 14 * dx.min(dy) + 10 * dx.abs_diff(dy);
    let vertical = if is_flying {
        from.layer.abs_diff(to.layer) as u32 * 10
    } else {
        0
    };
    Some(horizontal.saturating_add(vertical))
}

#[cfg(test)]
/// Expands diagonals without fallible native lookups.
fn expand_diagonals<F>(
    nodes: &[Coord],
    handling: DiagonalHandling,
    mut routes_for: F,
) -> Option<Vec<Coord>>
where
    F: FnMut(Coord, Coord) -> DiagonalRoutes,
{
    let Some((&start, rest)) = nodes.split_first() else {
        return Some(Vec::new());
    };
    let mut expanded = vec![start];
    let mut previous = start;
    for &next in rest {
        if next.0 != previous.0 && next.1 != previous.1 {
            let routes = routes_for(previous, next);
            let intermediate = match handling {
                DiagonalHandling::DoNothing => None,
                DiagonalHandling::RemoveAll if routes.north_south_first => {
                    Some((previous.0, next.1))
                }
                DiagonalHandling::RemoveAll if routes.east_west_first => Some((next.0, previous.1)),
                DiagonalHandling::RemoveClunky
                    if !routes.north_south_first && routes.east_west_first =>
                {
                    Some((next.0, previous.1))
                }
                DiagonalHandling::RemoveClunky => None,
                DiagonalHandling::RemoveAll => return None,
            };
            if let Some(intermediate) = intermediate {
                expanded.push(intermediate);
            }
        }
        expanded.push(next);
        previous = next;
    }
    Some(expanded)
}

/// Expands diagonals while rechecking native passability.
fn expand_diagonals_checked<F, E>(
    nodes: &[Coord],
    handling: DiagonalHandling,
    mut routes_for: F,
) -> Result<Option<Vec<Coord>>, E>
where
    F: FnMut(Coord, Coord) -> Result<DiagonalRoutes, E>,
{
    let Some((&start, rest)) = nodes.split_first() else {
        return Ok(Some(Vec::new()));
    };
    let mut expanded = vec![start];
    let mut previous = start;
    for &next in rest {
        if next.0 != previous.0 && next.1 != previous.1 {
            let routes = routes_for(previous, next)?;
            let intermediate = match handling {
                DiagonalHandling::DoNothing => None,
                DiagonalHandling::RemoveAll if routes.north_south_first => {
                    Some((previous.0, next.1))
                }
                DiagonalHandling::RemoveAll if routes.east_west_first => Some((next.0, previous.1)),
                DiagonalHandling::RemoveClunky
                    if !routes.north_south_first && routes.east_west_first =>
                {
                    Some((next.0, previous.1))
                }
                // Clunky handling only replaces a diagonal when its vertical-first route is
                // blocked. A valid vertical-first route keeps the direct diagonal.
                DiagonalHandling::RemoveClunky => None,
                DiagonalHandling::RemoveAll => return Ok(None),
            };
            if let Some(intermediate) = intermediate {
                expanded.push(intermediate);
            }
        }
        expanded.push(next);
        previous = next;
    }
    Ok(Some(expanded))
}

/// Removes the starting node when requested by DM.
fn apply_skip_first(mut nodes: Vec<Coord>, skip_first: bool) -> Vec<Coord> {
    if skip_first && !nodes.is_empty() {
        nodes.remove(0);
    }
    nodes
}

/// Creates an empty DM list.
fn empty_list() -> Result<ByondValue, NavPathError> {
    Ok(Vec::<ByondValue>::new().to_byond()?)
}

/// Builds a standard native pathfinder response.
fn status_response(
    status: &str,
    job_id: Option<u64>,
    path: Option<ByondValue>,
    error: Option<&str>,
) -> Result<ByondValue, NavPathError> {
    let mut result = ByondValue::new_list()?;
    result.write_list_index(
        ByondValue::new_string("status"),
        ByondValue::new_string(status),
    )?;
    if let Some(job_id) = job_id {
        result.write_list_index(
            ByondValue::new_string("job_id"),
            ByondValue::new_string(job_id.to_string()),
        )?;
    }
    if let Some(path) = path {
        result.write_list_index(ByondValue::new_string("path"), path)?;
    }
    if let Some(error) = error {
        result.write_list_index(
            ByondValue::new_string("error"),
            ByondValue::new_string(error),
        )?;
    }
    Ok(result)
}

/// Builds a pathfinder response with action data.
fn status_response_with_actions(
    status: &str,
    job_id: Option<u64>,
    path: Option<ByondValue>,
    actions: Option<ByondValue>,
    error: Option<&str>,
) -> Result<ByondValue, NavPathError> {
    let mut result = status_response(status, job_id, path, error)?;
    if let Some(actions) = actions {
        result.write_list_index(ByondValue::new_string("actions"), actions)?;
    }
    Ok(result)
}

/// Parses a numeric or string job identifier.
fn parse_job_id(value: &ByondValue) -> Option<u64> {
    if value.is_number() {
        let number = value.get_number().ok()?;
        return (number >= 0.0 && number.fract() == 0.0).then_some(number as u64);
    }
    value.get_string().ok()?.parse().ok()
}

/// Removes expired single-z jobs.
fn prune_expired_jobs(jobs: &mut HashMap<u64, SearchJob>) {
    let now = Instant::now();
    jobs.retain(|_, job| now.duration_since(job.last_touched) < IDLE_TIMEOUT);
}

/// Removes expired multi-z jobs.
fn prune_expired_multi_jobs(jobs: &mut HashMap<u64, MultiSearchJob>) {
    let now = Instant::now();
    jobs.retain(|_, job| now.duration_since(job.last_touched) < IDLE_TIMEOUT);
}

/// Runs one single-z job slice and builds its response.
fn run_job(
    mut job: SearchJob,
    job_id: u64,
    pass_info: &ByondValue,
) -> Result<(Option<SearchJob>, ByondValue), NavPathError> {
    match job.step(pass_info)? {
        JobProgress::InProgress => Ok((
            Some(job),
            status_response("in_progress", Some(job_id), None, None)?,
        )),
        JobProgress::NoPath => Ok((
            None,
            status_response("no_path", None, Some(empty_list()?), None)?,
        )),
        JobProgress::Found(nodes) => {
            let path = job.final_path(nodes, pass_info)?;
            Ok((None, status_response("complete", None, Some(path), None)?))
        }
    }
}

/// Runs one multi-z job slice and builds its response.
fn run_multi_job(
    mut job: MultiSearchJob,
    job_id: u64,
    pass_info: &ByondValue,
) -> Result<(Option<MultiSearchJob>, ByondValue), NavPathError> {
    match job.step(pass_info)? {
        MultiJobProgress::InProgress => Ok((
            Some(job),
            status_response("in_progress", Some(job_id), None, None)?,
        )),
        MultiJobProgress::Stale => Ok((
            None,
            status_response("stale_topology", Some(job_id), None, None)?,
        )),
        MultiJobProgress::NoPath => Ok((
            None,
            status_response_with_actions(
                "no_path",
                None,
                Some(empty_list()?),
                Some(ByondValue::new_list()?),
                None,
            )?,
        )),
        MultiJobProgress::Found(goal) => {
            let (path, actions) = job.final_path(goal, pass_info)?;
            Ok((
                None,
                status_response_with_actions("complete", None, Some(path), Some(actions), None)?,
            ))
        }
    }
}

#[allow(clippy::too_many_arguments)]
/// Builds a single-z search from FFI arguments.
fn new_search_job(
    start: ByondValue,
    end: ByondValue,
    pass_info: &ByondValue,
    is_flying: ByondValue,
    max_range: ByondValue,
    min_target_distance: ByondValue,
    simulated_only: ByondValue,
    avoid_turf: ByondValue,
    diagonal_handling: ByondValue,
    skip_first: ByondValue,
) -> Result<Option<SearchJob>, NavPathError> {
    let start_xyz = start.xyz().ok_or(NavPathError::NotATurf)?;
    let end_xyz = end.xyz().ok_or(NavPathError::NotATurf)?;
    let (sx, sy, sz) = (start_xyz.x(), start_xyz.y(), start_xyz.z());
    let (ex, ey, ez) = (end_xyz.x(), end_xyz.y(), end_xyz.z());
    if sz != ez {
        return Err(NavPathError::DifferentZLevels);
    }
    let max_range = max_range.get_number().unwrap_or(0.0).max(0.0) as i32;
    let min_target_distance = min_target_distance.get_number().unwrap_or(0.0).max(0.0) as i32;
    let avoid = if avoid_turf.is_null() {
        None
    } else {
        let xyz = avoid_turf.xyz().ok_or(NavPathError::NotATurf)?;
        let (x, y, z) = (xyz.x(), xyz.y(), xyz.z());
        (z == sz).then_some((x, y))
    };
    if avoid == Some((sx, sy))
        || (max_range > 0
            && chebyshev_distance((sx, sy), (ex, ey))
                > max_range.saturating_add(min_target_distance))
    {
        // There can be no route when the start is forbidden or the target radius cannot be
        // reached within max_range.
        return Ok(None);
    }
    let pass_flags = pass_info
        .read_var::<_, ByondValue>("pass_flags")
        .and_then(|value| value.get_number())
        .map(|number| number as i32)
        .unwrap_or(0);
    Ok(Some(SearchJob::new(
        Grid {
            z: sz,
            pass_flags,
            is_flying: truthy(&is_flying),
            start: (sx, sy),
            max_range,
            simulated_only: truthy(&simulated_only),
            avoid,
            cache: HashMap::new(),
        },
        (ex, ey),
        min_target_distance,
        DiagonalHandling::from_byond(&diagonal_handling),
        truthy(&skip_first),
    )))
}

/// Blocking version of pathfinder wrapper. Prefer the async start/resume API for
/// movement loops; this intentionally does not return to DM between slices.
#[allow(clippy::too_many_arguments)]
#[byond_fn]
#[allow(dead_code)]
fn navmap_pathfinder_ffi(
    start: ByondValue,
    end: ByondValue,
    pass_info: ByondValue,
    is_flying: ByondValue,
    max_range: ByondValue,
    min_target_distance: ByondValue,
    simulated_only: ByondValue,
    avoid_turf: ByondValue,
    diagonal_handling: ByondValue,
    skip_first: ByondValue,
    allow_multiz: ByondValue,
    max_path_cost: ByondValue,
) -> Result<ByondValue, NavPathError> {
    tracy_span!("navmap.pathfinder_blocking");
    if truthy(&allow_multiz) {
        let Some(mut job) = new_multi_search_job(
            start,
            end,
            &pass_info,
            is_flying,
            max_range,
            min_target_distance,
            simulated_only,
            avoid_turf,
            diagonal_handling,
            skip_first,
            max_path_cost,
        )?
        else {
            return empty_list();
        };
        loop {
            match job.step(&pass_info)? {
                MultiJobProgress::InProgress => continue,
                MultiJobProgress::NoPath | MultiJobProgress::Stale => return empty_list(),
                MultiJobProgress::Found(goal) => {
                    let (path, _) = job.final_path(goal, &pass_info)?;
                    return Ok(path);
                }
            }
        }
    }
    let Some(mut job) = new_search_job(
        start,
        end,
        &pass_info,
        is_flying,
        max_range,
        min_target_distance,
        simulated_only,
        avoid_turf,
        diagonal_handling,
        skip_first,
    )?
    else {
        return empty_list();
    };
    loop {
        match job.step(&pass_info)? {
            JobProgress::InProgress => continue,
            JobProgress::NoPath => return empty_list(),
            JobProgress::Found(nodes) => return job.final_path(nodes, &pass_info),
        }
    }
}

/// Starts a resumable native pathfinding job.
#[allow(clippy::too_many_arguments)]
#[byond_fn]
#[allow(dead_code)]
fn navmap_pathfinder_start_ffi(
    start: ByondValue,
    end: ByondValue,
    pass_info: ByondValue,
    is_flying: ByondValue,
    max_range: ByondValue,
    min_target_distance: ByondValue,
    simulated_only: ByondValue,
    avoid_turf: ByondValue,
    diagonal_handling: ByondValue,
    skip_first: ByondValue,
    allow_multiz: ByondValue,
    max_path_cost: ByondValue,
) -> Result<ByondValue, NavPathError> {
    tracy_span!("navmap.pathfinder_start");
    let job_id = NEXT_JOB_ID.fetch_add(1, AtomicOrdering::Relaxed);
    if truthy(&allow_multiz) {
        let Some(job) = new_multi_search_job(
            start,
            end,
            &pass_info,
            is_flying,
            max_range,
            min_target_distance,
            simulated_only,
            avoid_turf,
            diagonal_handling,
            skip_first,
            max_path_cost,
        )?
        else {
            return status_response("no_path", None, Some(empty_list()?), None);
        };
        match run_multi_job(job, job_id, &pass_info) {
            Ok((Some(job), response)) => {
                let mut jobs = MULTI_SEARCH_JOBS.lock().unwrap_or_else(|p| p.into_inner());
                prune_expired_multi_jobs(&mut jobs);
                jobs.insert(job_id, job);
                return Ok(response);
            }
            Ok((None, response)) => return Ok(response),
            Err(error) => {
                return status_response("error", Some(job_id), None, Some(&error.to_string()));
            }
        }
    }
    let Some(job) = new_search_job(
        start,
        end,
        &pass_info,
        is_flying,
        max_range,
        min_target_distance,
        simulated_only,
        avoid_turf,
        diagonal_handling,
        skip_first,
    )?
    else {
        return status_response("no_path", None, Some(empty_list()?), None);
    };
    match run_job(job, job_id, &pass_info) {
        Ok((Some(job), response)) => {
            let mut jobs = SEARCH_JOBS
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            prune_expired_jobs(&mut jobs);
            jobs.insert(job_id, job);
            Ok(response)
        }
        Ok((None, response)) => Ok(response),
        Err(error) => status_response("error", Some(job_id), None, Some(&error.to_string())),
    }
}

/// Resumes and returns the next result for a native pathfinding job.
#[byond_fn]
#[allow(dead_code)]
fn navmap_pathfinder_resume_ffi(
    job_id: ByondValue,
    pass_info: ByondValue,
) -> Result<ByondValue, NavPathError> {
    tracy_span!("navmap.pathfinder_resume");
    let Some(job_id) = parse_job_id(&job_id) else {
        return status_response(
            "error",
            None,
            None,
            Some("invalid navmap pathfinder job ID"),
        );
    };
    let job = {
        let mut jobs = SEARCH_JOBS
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        prune_expired_jobs(&mut jobs);
        jobs.remove(&job_id)
    };
    if let Some(job) = job {
        return match run_job(job, job_id, &pass_info) {
            Ok((Some(job), response)) => {
                SEARCH_JOBS
                    .lock()
                    .unwrap_or_else(|poisoned| poisoned.into_inner())
                    .insert(job_id, job);
                Ok(response)
            }
            Ok((None, response)) => Ok(response),
            Err(error) => status_response("error", Some(job_id), None, Some(&error.to_string())),
        };
    }
    let multi_job = {
        let mut jobs = MULTI_SEARCH_JOBS.lock().unwrap_or_else(|p| p.into_inner());
        prune_expired_multi_jobs(&mut jobs);
        jobs.remove(&job_id)
    };
    let Some(job) = multi_job else {
        return status_response(
            "error",
            Some(job_id),
            None,
            Some("unknown or expired navmap pathfinder job"),
        );
    };
    match run_multi_job(job, job_id, &pass_info) {
        Ok((Some(job), response)) => {
            MULTI_SEARCH_JOBS
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .insert(job_id, job);
            Ok(response)
        }
        Ok((None, response)) => Ok(response),
        Err(error) => status_response("error", Some(job_id), None, Some(&error.to_string())),
    }
}

/// Cancels an active native pathfinding job.
#[byond_fn]
#[allow(dead_code)]
fn navmap_pathfinder_cancel_ffi(job_id: ByondValue) -> Result<ByondValue, NavPathError> {
    let Some(job_id) = parse_job_id(&job_id) else {
        return status_response(
            "error",
            None,
            None,
            Some("invalid navmap pathfinder job ID"),
        );
    };
    let removed = {
        let mut jobs = SEARCH_JOBS
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        prune_expired_jobs(&mut jobs);
        jobs.remove(&job_id).is_some()
    };
    let removed_multi = if removed {
        false
    } else {
        let mut jobs = MULTI_SEARCH_JOBS.lock().unwrap_or_else(|p| p.into_inner());
        prune_expired_multi_jobs(&mut jobs);
        jobs.remove(&job_id).is_some()
    };
    let removed = removed || removed_multi;
    status_response(
        if removed { "cancelled" } else { "error" },
        Some(job_id),
        None,
        (!removed).then_some("unknown or expired navmap pathfinder job"),
    )
}

/// Updates one cached turf's packed navmap value.
#[byond_fn]
#[allow(dead_code)]
fn navmap_update_ffi(
    x: ByondValue,
    y: ByondValue,
    z: ByondValue,
    nav_pass: ByondValue,
) -> Result<ByondValue, NavPathError> {
    // Replacing the cached packed value immediately prevents searches from using stale edges.
    cache_nav_pass(
        coords_from_values(&x, &y, &z)?,
        nav_pass.get_number()? as i32,
    );
    Ok(ByondValue::NULL)
}

/// Updates cached packed values for a batch of turfs.
#[byond_fn]
#[allow(dead_code)]
fn navmap_bulk_update_ffi(flat_list: ByondValue) -> Result<ByondValue, NavPathError> {
    let values = flat_list.read_list()?;
    if !values.len().is_multiple_of(4) {
        return Err(NavPathError::InvalidBulkUpdateLength);
    }
    // DM sends x, y, z, nav_pass tuples so a whole z-level can be published in one call.
    for entry in values.chunks_exact(4) {
        cache_nav_pass(
            coords_from_values(&entry[0], &entry[1], &entry[2])?,
            entry[3].get_number()? as i32,
        );
    }
    Ok(ByondValue::NULL)
}

/// Replaces the native multi-z topology snapshot.
#[byond_fn]
#[allow(dead_code)]
fn navmap_topology_update_ffi(
    generation: ByondValue,
    flat_list: ByondValue,
) -> Result<ByondValue, NavPathError> {
    let values = flat_list.read_list()?;
    if !values.len().is_multiple_of(3) {
        return Err(NavPathError::InvalidBulkUpdateLength);
    }
    let mut topology = Topology {
        generation: generation.get_number().unwrap_or(0.0).max(0.0) as u64,
        ..Topology::default()
    };
    for entry in values.chunks_exact(3) {
        let group = entry[0].get_number()? as i32;
        let layer = entry[1].get_number()? as i16;
        let z = entry[2].get_number()? as i16;
        topology
            .z_to_layer
            .insert(z, LayerLocation { group, layer });
        topology.layer_to_z.insert((group, layer), z);
    }
    *TOPOLOGY.lock().unwrap_or_else(|p| p.into_inner()) = topology;
    Ok(ByondValue::new_num((values.len() / 3) as f32))
}

/// Returns native topology and link cache sizes for diagnostics.
#[byond_fn]
#[allow(dead_code)]
fn navmap_pathfinder_state_ffi() -> Result<ByondValue, NavPathError> {
    let topology = TOPOLOGY.lock().unwrap_or_else(|p| p.into_inner()).clone();
    let links = NAV_LINKS.lock().unwrap_or_else(|p| p.into_inner()).len();
    let mut result = ByondValue::new_list()?;
    result.write_list_index(
        ByondValue::new_string("generation"),
        ByondValue::new_num(topology.generation as f32),
    )?;
    result.write_list_index(
        ByondValue::new_string("z_count"),
        ByondValue::new_num(topology.z_to_layer.len() as f32),
    )?;
    result.write_list_index(
        ByondValue::new_string("layer_count"),
        ByondValue::new_num(topology.layer_to_z.len() as f32),
    )?;
    result.write_list_index(
        ByondValue::new_string("link_count"),
        ByondValue::new_num(links as f32),
    )?;
    Ok(result)
}

/// Replaces the native cross-z navigation links.
#[byond_fn]
#[allow(dead_code)]
fn navmap_links_update_ffi(flat_list: ByondValue) -> Result<ByondValue, NavPathError> {
    let values = flat_list.read_list()?;
    if !values.len().is_multiple_of(8) {
        return Err(NavPathError::InvalidBulkUpdateLength);
    }
    let mut links = NAV_LINKS.lock().unwrap_or_else(|p| p.into_inner());
    links.clear();
    for entry in values.chunks_exact(8) {
        let id = entry[0].get_number()?.max(0.0) as u64;
        let source = (
            entry[1].get_number()? as i16,
            entry[2].get_number()? as i16,
            entry[3].get_number()? as i16,
        );
        let destination = (
            entry[4].get_number()? as i16,
            entry[5].get_number()? as i16,
            entry[6].get_number()? as i16,
        );
        let cost = entry[7].get_number()?.max(1.0) as u32;
        links.insert(
            id,
            NavLink {
                id,
                source,
                destination,
                cost,
            },
        );
    }
    Ok(ByondValue::new_num(links.len() as f32))
}

/// Converts separate DM coordinates into a turf tuple.
fn coords_from_values(
    x: &ByondValue,
    y: &ByondValue,
    z: &ByondValue,
) -> Result<TurfCoords, NavPathError> {
    Ok((
        x.get_number()? as i16,
        y.get_number()? as i16,
        z.get_number()? as i16,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Packed cache values survive invalidation updates.
    #[test]
    fn cache_retains_simulated_invalidations() {
        let coords = (i16::MIN, i16::MIN, i16::MIN);
        cache_nav_pass(coords, BAKED_FLAG | SIMULATED_FLAG | NORTH);
        assert_eq!(
            cached_nav_pass(coords),
            Some(BAKED_FLAG | SIMULATED_FLAG | NORTH)
        );
        cache_nav_pass(coords, 0);
        assert_eq!(cached_nav_pass(coords), Some(0));
    }

    /// Range, space, and avoidance helpers reject invalid targets.
    #[test]
    fn option_helpers_cover_target_range_space_and_avoidance() {
        assert!(chebyshev_distance((7, 7), (10, 10)) <= 3);
        assert!(!turf_allowed(true, false));
        assert!(turf_allowed(false, false));
        assert!(!coordinate_allowed((12, 12), (10, 10), 2, Some((12, 12))));
        assert!(!coordinate_allowed((13, 10), (10, 10), 2, None));
    }

    /// Diagonal expansion preserves each requested output mode.
    #[test]
    fn path_shape_and_diagonal_modes_are_preserved() {
        let path = [(0, 0), (1, 1)];
        let both_routes = |_, _| DiagonalRoutes {
            north_south_first: true,
            east_west_first: true,
        };
        assert_eq!(
            expand_diagonals(&path, DiagonalHandling::RemoveAll, both_routes),
            Some(vec![(0, 0), (0, 1), (1, 1)])
        );
        let east_west_only = |_, _| DiagonalRoutes {
            north_south_first: false,
            east_west_first: true,
        };
        assert_eq!(
            expand_diagonals(&path, DiagonalHandling::RemoveClunky, east_west_only),
            Some(vec![(0, 0), (1, 0), (1, 1)])
        );
        assert_eq!(
            expand_diagonals(&path, DiagonalHandling::RemoveClunky, both_routes),
            Some(vec![(0, 0), (1, 1)])
        );
        assert_eq!(apply_skip_first(vec![(0, 0), (1, 0)], true), vec![(1, 0)]);
    }

    /// Queue ordering prefers the lowest estimated cost.
    #[test]
    fn queue_prefers_lowest_estimate() {
        let mut queue = BinaryHeap::new();
        queue.push(QueueEntry {
            coord: (0, 0),
            cost: 10,
            estimate: 20,
            sequence: 1,
        });
        queue.push(QueueEntry {
            coord: (1, 0),
            cost: 5,
            estimate: 10,
            sequence: 2,
        });
        assert_eq!(queue.pop().unwrap().coord, (1, 0));
    }

    /// Creates a small job fixture with a chosen activity time.
    fn test_job(last_touched: Instant) -> SearchJob {
        let mut job = SearchJob::new(
            Grid {
                z: 1,
                pass_flags: 0,
                is_flying: false,
                start: (1, 1),
                max_range: 0,
                simulated_only: false,
                avoid: None,
                cache: HashMap::new(),
            },
            (2, 2),
            0,
            DiagonalHandling::DoNothing,
            true,
        );
        job.last_touched = last_touched;
        job
    }

    /// Job pruning removes stale entries but keeps active ones.
    #[test]
    fn job_registry_prunes_expired_jobs_without_affecting_active_jobs() {
        let now = Instant::now();
        let mut jobs = HashMap::from([
            (1, test_job(now - IDLE_TIMEOUT - Duration::from_millis(1))),
            (2, test_job(now)),
            (3, test_job(now)),
        ]);
        prune_expired_jobs(&mut jobs);
        assert!(!jobs.contains_key(&1));
        assert!(jobs.contains_key(&2));
        assert!(jobs.contains_key(&3));
    }
}
