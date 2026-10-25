//! A* pathfinding over a /tg/station-style navmap; DM supplies packed passability data.
//! Searches yield in short resumable slices to avoid monopolizing the server tick.

use meowtonin::misc::locate_xyz;
use meowtonin::{ByondError, ByondValue, ByondXYZ, ToByond, call_global};
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
use search::{AStarCore, SearchProgress};
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

#[derive(Clone, Copy, Default)]
struct CachedNavData {
    nav_pass: i32,
    nav_cost_flags: u32,
}

/// Cached packed passability and terrain-cost values received from DM.
static NAV_PASS_CACHE: LazyLock<Mutex<HashMap<TurfCoords, CachedNavData>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));
/// Generation used to invalidate yielded searches after a published cost change.
static NAV_COST_GENERATION: AtomicU64 = AtomicU64::new(0);
/// Active single-z search jobs.
static SEARCH_JOBS: LazyLock<Mutex<HashMap<u64, SearchJob>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));
/// Active multi-z search jobs.
static MULTI_SEARCH_JOBS: LazyLock<Mutex<HashMap<u64, MultiSearchJob>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));
/// Native multi-z topology.
static TOPOLOGY: LazyLock<Mutex<Topology>> = LazyLock::new(|| Mutex::new(Topology::default()));
/// Registered navigation links.
static NAV_LINKS: LazyLock<Mutex<HashMap<u64, NavLink>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));
/// Registered navigation links indexed by source turf for cheap search snapshots.
static NAV_LINKS_BY_SOURCE: LazyLock<Mutex<HashMap<TurfCoords, Vec<NavLink>>>> =
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
    #[error("navmap bulk update list length must be a multiple of 5")]
    InvalidBulkUpdateLength,
    #[error("navmap topology update list length must be a multiple of 3")]
    InvalidTopologyUpdateLength,
    #[error("nav cost modifiers must be flag/multiplier pairs")]
    InvalidCostModifiers,
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
    /// Turf-level terrain modifier flags.
    cost_flags: u32,
}

/// Per-search modifier weights and cached combined multipliers by turf flag combination.
struct CostProfile {
    modifiers: Vec<(u32, f32)>,
    combined: HashMap<u32, f32>,
    /// Conservative lower bound for any terrain multiplier in this request.
    heuristic_scale: f32,
}

impl CostProfile {
    fn from_byond(value: &ByondValue) -> Result<Self, NavPathError> {
        let values = value.read_list()?;
        if !values.len().is_multiple_of(2) {
            return Err(NavPathError::InvalidCostModifiers);
        }
        let mut modifiers = Vec::with_capacity(values.len() / 2);
        for pair in values.chunks_exact(2) {
            let flag_value = pair[0].get_number()?;
            let multiplier = pair[1].get_number()?;
            if !flag_value.is_finite()
                || flag_value <= 0.0
                || flag_value.fract() != 0.0
                || !multiplier.is_finite()
                || multiplier <= 0.0
            {
                return Err(NavPathError::InvalidCostModifiers);
            }
            let flag = flag_value as u32;
            if !flag.is_power_of_two() {
                return Err(NavPathError::InvalidCostModifiers);
            }
            modifiers.push((flag, multiplier));
        }
        let mut heuristic_scale = 1.0_f32;
        for &(_, multiplier) in &modifiers {
            if multiplier < 1.0 {
                heuristic_scale *= multiplier;
            }
        }
        if heuristic_scale > 0.0 && heuristic_scale < 1.0 {
            // Round down one representable step so floating-point rounding cannot make the
            // heuristic exceed the cheapest discounted movement cost.
            heuristic_scale = f32::from_bits(heuristic_scale.to_bits() - 1);
        }
        Ok(Self {
            modifiers,
            combined: HashMap::new(),
            heuristic_scale,
        })
    }

    fn multiplier(&mut self, flags: u32) -> f32 {
        if let Some(multiplier) = self.combined.get(&flags) {
            return *multiplier;
        }
        let mut multiplier = 1.0;
        for &(flag, weight) in &self.modifiers {
            if flags & flag != 0 {
                multiplier *= weight;
            }
        }
        self.combined.insert(flags, multiplier);
        multiplier
    }

    fn step_cost(&mut self, base: u32, flags: u32) -> u32 {
        if base == 0 {
            return 0;
        }
        ((base as f32) * self.multiplier(flags)).ceil().max(1.0) as u32
    }

    fn scale_heuristic(estimate: u32, scale: f32) -> u32 {
        if scale >= 1.0 {
            return estimate;
        }
        ((estimate as f64) * (scale as f64)).floor() as u32
    }

    fn has_modifiers(&self) -> bool {
        !self.modifiers.is_empty()
    }
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
    /// Maximum total path cost, or zero for unlimited.
    max_cost: u32,
    /// Requested diagonal output mode used when pricing expanded diagonal moves.
    diagonal_handling: DiagonalHandling,
    /// Whether only simulated turfs are allowed.
    simulated_only: bool,
    /// Optional turf excluded from the search.
    avoid: Option<Coord>,
    /// Same-z navigation links included in this search, indexed by source coordinate.
    nav_links_by_source: HashMap<Coord, Vec<NavLink>>,
    /// Cached turf lookups.
    cache: HashMap<Coord, Option<TurfInfo>>,
    /// Search-specific terrain cost weights.
    costs: CostProfile,
    /// Cost generation at search start.
    cost_generation: u64,
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
    /// Shared A* frontier, relaxation, and reconstruction state.
    search: AStarCore<Coord>,
    /// Topology generation for the same-z navigation-link snapshot, when links were included.
    topology_generation: Option<u64>,
    /// Navigation-link lower bound for routes that may use same-z links.
    heuristic: Option<FlatNavLinkHeuristic>,
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
/// A reusable navigation transition.
struct NavLink {
    /// Stable navigation-link identifier.
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

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
/// Action used to reach a search node.
enum SearchAction {
    #[default]
    None,
    /// Cardinal intermediate selected when a diagonal is charged as two cardinal steps.
    Diagonal(i16, i16),
    Vertical(i32),
    NavLink(u64),
}

/// A lower-bound route model for multi-z A*.
///
/// The model deliberately ignores walls, conditional blockers, and mover-specific link
/// eligibility. It therefore cannot overestimate the real graph, while link nodes let it account
/// for useful cross-z links instead of collapsing the heuristic to zero everywhere.
struct NavLinkHeuristic {
    goal: LayeredNode,
    min_target_distance: i32,
    is_flying: bool,
    /// Relaxed distance from each navigation-link source to the target region.
    source_distances: Vec<(LayeredNode, u32)>,
}

/// Same-z navigation-link lower bound, built only when the requested floor has links.
struct FlatNavLinkHeuristic {
    goal: Coord,
    min_target_distance: i32,
    source_distances: Vec<(Coord, u32)>,
}

/// Multi-z search grid and its per-job cache.
struct LayeredGrid {
    /// Topology snapshot used by this search.
    topology: Topology,
    /// Pass flags for conditional blockers.
    pass_flags: i32,
    /// Whether flying edges should be used.
    is_flying: bool,
    /// Whether registered navigation links may be traversed.
    allow_nav_links: bool,
    /// Search origin node.
    start: LayeredNode,
    /// Maximum horizontal search range.
    max_range: i32,
    /// Maximum total path cost, or zero for unlimited.
    max_cost: u32,
    /// Diagonal mode used when pricing moves DM returns as cardinal steps.
    diagonal_handling: DiagonalHandling,
    /// Whether only simulated turfs are allowed.
    simulated_only: bool,
    /// Optional node excluded from the search.
    avoid: Option<LayeredNode>,
    /// Cross-z navigation links indexed by their source turf for O(out-degree) expansion.
    nav_links_by_source: HashMap<TurfCoords, Vec<NavLink>>,
    /// Cached node lookups.
    cache: HashMap<LayeredNode, Option<TurfInfo>>,
    /// Search-specific terrain cost weights.
    costs: CostProfile,
    /// Cost generation at search start.
    cost_generation: u64,
}

/// Result of one multi-z search slice.
enum MultiJobProgress {
    InProgress,
    Found(Vec<LayeredNode>, Vec<SearchAction>),
    NoPath,
    Stale,
    StaleCosts,
}

/// Resumable multi-z A* search state.
struct MultiSearchJob {
    /// Layered grid and topology snapshot.
    grid: LayeredGrid,
    /// Search target node.
    goal: LayeredNode,
    /// Admissible lower-bound estimate including cross-z navigation links.
    heuristic: NavLinkHeuristic,
    /// Acceptable distance from the target.
    min_target_distance: i32,
    /// Requested diagonal output mode.
    diagonal_handling: DiagonalHandling,
    /// Whether the first returned turf is omitted.
    skip_first: bool,
    /// Shared A* frontier, relaxation, and reconstruction state.
    search: AStarCore<LayeredNode>,
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

/// Evaluates numeric masks and live weakrefs for one packed edge.
fn evaluate_conditional_edge(
    turf: &ByondValue,
    dir: i32,
    pass_flags: i32,
    pass_info: &ByondValue,
) -> Result<bool, NavPathError> {
    tracy_span!("navmap.conditional_edge");
    let blockers_list: ByondValue = turf.read_var("nav_blockers")?;
    if blockers_list.is_null() {
        return Ok(true);
    }
    let entries_list =
        match blockers_list.read_list_index::<_, ByondValue>(&ByondValue::new_num(dir as f32)) {
            Ok(value) if value.is_list() => value,
            _ => return Ok(true),
        };
    let entries: Vec<_> = entries_list
        .read_list()?
        .into_iter()
        .map(std::mem::ManuallyDrop::new)
        .collect();
    let eval_dir = ByondValue::new_num(dir as f32);
    for entry in entries.iter() {
        if entry.is_number() {
            if pass_flags & entry.get_number()? as i32 == 0 {
                return Ok(false);
            }
            continue;
        }
        // Resolve and evaluate in one DM callback, without a native loc read.
        let result = turf.call("nav_blocker_can_pass", [&**entry, &eval_dir, pass_info])?;
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
    allow_nav_links: ByondValue,
    cost_modifiers: ByondValue,
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
    let costs = CostProfile::from_byond(&cost_modifiers)?;
    let cost_generation = nav_cost_generation();
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
    let allow_nav_links = truthy(&allow_nav_links);
    let nav_links_by_source = if allow_nav_links {
        NAV_LINKS_BY_SOURCE
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .clone()
    } else {
        HashMap::new()
    };
    let nav_links: Vec<NavLink> = nav_links_by_source.values().flatten().copied().collect();
    let is_flying = truthy(&is_flying);
    Ok(Some(MultiSearchJob {
        grid: LayeredGrid {
            topology: topology.clone(),
            pass_flags,
            is_flying,
            allow_nav_links,
            start: start_node,
            max_range,
            max_cost: max_path_cost,
            diagonal_handling: DiagonalHandling::from_byond(&diagonal_handling),
            simulated_only: truthy(&simulated_only),
            avoid,
            nav_links_by_source,
            cache: HashMap::new(),
            costs,
            cost_generation,
        },
        goal: goal_node,
        heuristic: NavLinkHeuristic::new(
            &topology,
            goal_node,
            min_target_distance,
            is_flying,
            &nav_links,
        ),
        min_target_distance,
        diagonal_handling: DiagonalHandling::from_byond(&diagonal_handling),
        skip_first: truthy(&skip_first),
        search: AStarCore::default(),
        last_touched: Instant::now(),
    }))
}

/// Reads one packed navmap value from the shared cache.
#[cfg(test)]
fn cached_nav_pass(coords: TurfCoords) -> Option<i32> {
    cached_nav_data(coords).map(|data| data.nav_pass)
}

#[cfg(test)]
fn cached_nav_cost_flags(coords: TurfCoords) -> Option<u32> {
    cached_nav_data(coords).map(|data| data.nav_cost_flags)
}

fn cached_nav_data(coords: TurfCoords) -> Option<CachedNavData> {
    NAV_PASS_CACHE
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
        .get(&coords)
        .copied()
}

fn read_nav_cost_flags(turf: &ByondValue) -> Result<u32, NavPathError> {
    let value: ByondValue = turf.call("get_nav_cost_flags", std::iter::empty::<ByondValue>())?;
    Ok(value
        .get_number()
        .map(|number| if number < 0.0 { 0 } else { number as u32 })
        .unwrap_or(0))
}

fn nav_cost_generation() -> u64 {
    NAV_COST_GENERATION.load(AtomicOrdering::Acquire)
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

/// Resolves movement-class edge bits and optional flying vertical edges using shared packed rules.
fn resolve_packed_edges<E, Evaluate>(
    nav_pass: i32,
    is_flying: bool,
    include_vertical: bool,
    mut evaluate_conditional: Evaluate,
) -> Result<(i32, i32), E>
where
    Evaluate: FnMut(i32) -> Result<bool, E>,
{
    let class_shift = if is_flying { FLYING_SHIFT } else { 0 };
    let mut open = 0;
    for dir in CARDINALS {
        if nav_pass & (dir << class_shift) == 0 {
            continue;
        }
        if nav_pass & (dir << COND_SHIFT) == 0 || evaluate_conditional(dir)? {
            open |= dir;
        }
    }
    let mut vertical = 0;
    if include_vertical && is_flying {
        for dir in [UP, DOWN] {
            if nav_pass & (dir << FLYING_SHIFT) == 0 {
                continue;
            }
            if nav_pass & (dir << COND_SHIFT) == 0 || evaluate_conditional(dir)? {
                vertical |= dir;
            }
        }
    }
    Ok((open, vertical))
}

/// Stores one packed navmap value in the shared cache.
#[cfg(test)]
fn cache_nav_pass(coords: TurfCoords, nav_pass: i32) {
    let cost_flags = cached_nav_cost_flags(coords).unwrap_or(0);
    cache_nav_data(coords, nav_pass, cost_flags);
}

/// Stores packed passability and modifier flags, invalidating cost-aware jobs on changes.
fn cache_nav_data(coords: TurfCoords, nav_pass: i32, nav_cost_flags: u32) {
    // DM publishes both baked values and invalidations here. Keeping an unbaked value is
    // intentional: resolve_edges will force a fresh bake before using it.
    let mut cache = NAV_PASS_CACHE
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    let previous = cache.insert(
        coords,
        CachedNavData {
            nav_pass,
            nav_cost_flags,
        },
    );
    if previous.is_some_and(|data| data.nav_cost_flags != nav_cost_flags) {
        NAV_COST_GENERATION.fetch_add(1, AtomicOrdering::Release);
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

/// Encodes per-node traversal actions in the shared native result format.
fn actions_to_byond(actions: Vec<SearchAction>) -> Result<ByondValue, NavPathError> {
    Ok(actions
        .into_iter()
        .map(|action| match action {
            SearchAction::None | SearchAction::Diagonal(_, _) => ByondValue::NULL,
            SearchAction::Vertical(dir) => ByondValue::new_num(dir as f32),
            SearchAction::NavLink(id) => ByondValue::new_num(-(id as f32)),
        })
        .collect::<Vec<_>>()
        .to_byond()?)
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
            match search::diagonal_intermediate(previous, next, handling, routes) {
                Ok(Some(intermediate)) => expanded.push(intermediate),
                Ok(None) => {}
                Err(()) => return None,
            }
        }
        expanded.push(next);
        previous = next;
    }
    Some(expanded)
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
        SearchProgress::InProgress => Ok((
            Some(job),
            status_response("in_progress", Some(job_id), None, None)?,
        )),
        SearchProgress::Stale => Ok((
            None,
            status_response("stale_topology", Some(job_id), None, None)?,
        )),
        SearchProgress::StaleCosts => Ok((
            None,
            status_response("stale_costs", Some(job_id), None, None)?,
        )),
        SearchProgress::NoPath => Ok((
            None,
            status_response_with_actions(
                "no_path",
                None,
                Some(empty_list()?),
                Some(ByondValue::new_list()?),
                None,
            )?,
        )),
        SearchProgress::Found(nodes, actions) => {
            let (path, actions) = job.final_path(nodes, actions, pass_info)?;
            Ok((
                None,
                status_response_with_actions("complete", None, Some(path), Some(actions), None)?,
            ))
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
        MultiJobProgress::StaleCosts => Ok((
            None,
            status_response("stale_costs", Some(job_id), None, None)?,
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
        MultiJobProgress::Found(nodes, actions) => {
            let (path, actions) = job.final_path(nodes, actions, pass_info)?;
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
    max_path_cost: ByondValue,
    allow_nav_links: ByondValue,
    cost_modifiers: ByondValue,
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
    let max_path_cost = max_path_cost.get_number().unwrap_or(0.0).max(0.0) as u32;
    let costs = CostProfile::from_byond(&cost_modifiers)?;
    let cost_generation = nav_cost_generation();
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
    let mut nav_links_by_source: HashMap<Coord, Vec<NavLink>> = HashMap::new();
    if truthy(&allow_nav_links) {
        let nav_links_by_world_source = NAV_LINKS_BY_SOURCE
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        for (&(x, y, z), nav_links) in nav_links_by_world_source.iter() {
            if z != sz {
                continue;
            }
            for &nav_link in nav_links {
                if nav_link.destination.2 == sz {
                    nav_links_by_source
                        .entry((x, y))
                        .or_default()
                        .push(nav_link);
                }
            }
        }
    }
    let nav_links: Vec<NavLink> = nav_links_by_source.values().flatten().copied().collect();
    let topology_generation = (!nav_links.is_empty()).then(topology_generation);
    let heuristic = (!nav_links.is_empty())
        .then(|| FlatNavLinkHeuristic::new((ex, ey), min_target_distance, &nav_links));
    Ok(Some(SearchJob::new(
        Grid {
            z: sz,
            pass_flags,
            is_flying: truthy(&is_flying),
            start: (sx, sy),
            max_range,
            max_cost: max_path_cost,
            diagonal_handling: DiagonalHandling::from_byond(&diagonal_handling),
            simulated_only: truthy(&simulated_only),
            avoid,
            nav_links_by_source,
            cache: HashMap::new(),
            costs,
            cost_generation,
        },
        (ex, ey),
        min_target_distance,
        DiagonalHandling::from_byond(&diagonal_handling),
        truthy(&skip_first),
        topology_generation,
        heuristic,
    )))
}
