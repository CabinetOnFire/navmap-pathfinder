//! A* pathfinding over a /tg/station-style navmap; DM supplies packed passability data.
//! Searches yield in short resumable slices to avoid monopolizing the server tick.

use meowtonin::misc::locate_xyz;
use meowtonin::{ByondError, ByondValue, ByondXYZ, ToByond, call_global};
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

mod byond_api;
mod grid;
mod multiz_search;
mod search;
#[cfg(test)]
#[path = "navmap_pathfinder/tests.rs"]
mod tests;

const NORTH: i32 = 1;
const SOUTH: i32 = 2;
const EAST: i32 = 4;
const WEST: i32 = 8;
const UP: i32 = 1 << 4;
const DOWN: i32 = 1 << 5;
const CARDINALS: [i32; 4] = [NORTH, SOUTH, EAST, WEST];

const FLYING_SHIFT: i32 = 6;
const COND_SHIFT: i32 = 12;
const BAKED_FLAG: i32 = 1 << 18;
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

/// Maps a z-level group to its ordered layer.
#[derive(Clone, Copy, Debug, Default)]
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

/// Queue entry for multi-z A*.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct LayeredQueueEntry {
    /// Node represented by this queue entry.
    node: LayeredNode,
    /// Cost already spent reaching the node.
    cost: u32,
    /// Cost plus the relaxed lower-bound estimate.
    estimate: u32,
    /// Tie-breaker preserving queue order.
    sequence: u64,
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
    let entries =
        match blockers_list.read_list_index::<_, ByondValue>(&ByondValue::new_num(dir as f32)) {
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
