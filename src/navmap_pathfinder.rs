//! A* pathfidnding over a /tg/station style navmap
//!
//! From the DM side we send a bitfield of data for each turfs passability in each direction
//! 4 bits for directions on ground, 4 bits for directions when flying, and 4 bits of "is there something I need to check on the DM side"
//! There is also a bit for whether this turf is baked at all (If not, we need to FORCE a bake on the DM side, so not ideal)
//! Lastly, there's a bit to check if the turf is simulated (not space) to see if we should even path here.
//!
//! This system creates jobs for each pathfinding pass so we don't get
//! stuck on massive paths for too long wihtout yielding back to DM

use meowtonin::misc::locate_xyz;
use meowtonin::{ByondError, ByondValue, ByondXYZ, ToByond, byond_fn};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::sync::{
    LazyLock, Mutex,
    atomic::{AtomicU64, Ordering as AtomicOrdering},
};
use std::time::{Duration, Instant};
use thiserror::Error;

const NORTH: i32 = 1;
const SOUTH: i32 = 2;
const EAST: i32 = 4;
const WEST: i32 = 8;
const CARDINALS: [i32; 4] = [NORTH, SOUTH, EAST, WEST];

const FLYING_SHIFT: i32 = 4;
const COND_SHIFT: i32 = 8;
const BAKED_FLAG: i32 = 1 << 12;
const SIMULATED_FLAG: i32 = 1 << 13;

const DIAGONAL_DO_NOTHING: i32 = 0;
const DIAGONAL_REMOVE_ALL: i32 = 1;
const DIAGONAL_REMOVE_CLUNKY: i32 = 2;

const SLICE_BUDGET: Duration = Duration::from_millis(5);
const IDLE_TIMEOUT: Duration = Duration::from_secs(30);

///Costs for steps, 10 for cardinal 14 for diagonal
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

type TurfCoords = (i16, i16, i16);
type Coord = (i16, i16);

static NAV_PASS_CACHE: LazyLock<Mutex<HashMap<TurfCoords, i32>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));
static SEARCH_JOBS: LazyLock<Mutex<HashMap<u64, SearchJob>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum DiagonalHandling {
    DoNothing,
    RemoveAll,
    RemoveClunky,
}

impl DiagonalHandling {
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
    open_edges: i32,
    simulated: bool,
}

#[derive(Clone, Copy)]
struct DiagonalRoutes {
    north_south_first: bool,
    east_west_first: bool,
}

struct Grid {
    z: i16,
    pass_flags: i32,
    is_flying: bool,
    start: Coord,
    max_range: i32,
    simulated_only: bool,
    avoid: Option<Coord>,
    cache: HashMap<Coord, Option<TurfInfo>>,
}

impl Grid {
    fn lookup(
        &mut self,
        x: i16,
        y: i16,
        pass_info: &ByondValue,
    ) -> Result<Option<TurfInfo>, NavPathError> {
        // A search must use one consistent view of each turf, even if it yields and resumes.
        if let Some(hit) = self.cache.get(&(x, y)) {
            return Ok(*hit);
        }

        let turf = locate_xyz(ByondXYZ::new(x, y, self.z))?;
        let entry = if turf.is_null() {
            None
        } else {
            let (open_edges, nav_pass) = self.resolve_edges((x, y, self.z), &turf, pass_info)?;
            Some(TurfInfo {
                open_edges,
                simulated: nav_pass_is_simulated(nav_pass),
            })
        };
        self.cache.insert((x, y), entry);
        Ok(entry)
    }

    fn resolve_edges(
        &self,
        coords: TurfCoords,
        turf: &ByondValue,
        pass_info: &ByondValue,
    ) -> Result<(i32, i32), NavPathError> {
        let mut nav_pass = cached_nav_pass(coords).unwrap_or(read_nav_pass(turf)?);
        if nav_pass & BAKED_FLAG == 0 {
            // Dirty turfs have no trustworthy edge bits. Ask DM to bake now; if that still
            // does not produce a baked value, treat every edge as blocked for this search. (should not happen but hey)
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
            // Conditional edges are passable only when their live blocker check agrees. this is probably the most expensive part since we're calling back to DM
            if nav_pass & (dir << COND_SHIFT) == 0
                || self.evaluate_conditional_edge(turf, dir, pass_info)?
            {
                open |= dir;
            }
        }
        Ok((open, nav_pass))
    }

    fn evaluate_conditional_edge(
        &self,
        turf: &ByondValue,
        dir: i32,
        pass_info: &ByondValue,
    ) -> Result<bool, NavPathError> {
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

    fn can_occupy(&mut self, coord: Coord, pass_info: &ByondValue) -> Result<bool, NavPathError> {
        Ok(self.occupiable_info(coord, pass_info)?.is_some())
    }

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

    fn successors(
        &mut self,
        from: Coord,
        pass_info: &ByondValue,
    ) -> Result<Vec<(Coord, u32)>, NavPathError> {
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
    coord: Coord,
    cost: u32,
    estimate: u32,
    sequence: u64,
}

impl Ord for QueueEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .estimate
            .cmp(&self.estimate)
            .then_with(|| other.cost.cmp(&self.cost))
            .then_with(|| other.sequence.cmp(&self.sequence))
    }
}

impl PartialOrd for QueueEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

enum JobProgress {
    InProgress,
    Found(Vec<Coord>),
    NoPath,
}

struct SearchJob {
    grid: Grid,
    goal: Coord,
    min_target_distance: i32,
    diagonal_handling: DiagonalHandling,
    skip_first: bool,
    frontier: BinaryHeap<QueueEntry>,
    costs: HashMap<Coord, u32>,
    previous: HashMap<Coord, Coord>,
    initialized: bool,
    sequence: u64,
    last_touched: Instant,
}

impl SearchJob {
    fn new(
        grid: Grid,
        goal: Coord,
        min_target_distance: i32,
        diagonal_handling: DiagonalHandling,
        skip_first: bool,
    ) -> Self {
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

    fn reconstruct(&self, mut goal: Coord) -> Vec<Coord> {
        let mut path = vec![goal];
        while let Some(&previous) = self.previous.get(&goal) {
            path.push(previous);
            goal = previous;
        }
        path.reverse();
        path
    }

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

fn cached_nav_pass(coords: TurfCoords) -> Option<i32> {
    NAV_PASS_CACHE
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
        .get(&coords)
        .copied()
}

fn cache_nav_pass(coords: TurfCoords, nav_pass: i32) {
    // DM publishes both baked values and invalidations here. Keeping an unbaked value is
    // intentional: resolve_edges will force a fresh bake before using it.
    NAV_PASS_CACHE
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
        .insert(coords, nav_pass);
}

fn reverse_dir(dir: i32) -> i32 {
    match dir {
        NORTH => SOUTH,
        SOUTH => NORTH,
        EAST => WEST,
        WEST => EAST,
        other => other,
    }
}

fn read_nav_pass(turf: &ByondValue) -> Result<i32, NavPathError> {
    Ok(turf
        .read_var::<_, ByondValue>("nav_pass")
        .and_then(|value| value.get_number())
        .map(|number| number as i32)
        .unwrap_or(0))
}

fn truthy(value: &ByondValue) -> bool {
    !value.is_null() && (!value.is_number() || value.get_number().is_ok_and(|number| number != 0.0))
}

fn coordinate_allowed(coord: Coord, start: Coord, max_range: i32, avoid: Option<Coord>) -> bool {
    (max_range <= 0 || chebyshev_distance(coord, start) <= max_range) && avoid != Some(coord)
}

fn turf_allowed(simulated_only: bool, simulated: bool) -> bool {
    !simulated_only || simulated
}

fn nav_pass_is_simulated(nav_pass: i32) -> bool {
    nav_pass & SIMULATED_FLAG != 0
}

fn chebyshev_distance(a: Coord, b: Coord) -> i32 {
    (a.0 - b.0).unsigned_abs().max((a.1 - b.1).unsigned_abs()) as i32
}

fn heuristic_to_target_range(coord: Coord, goal: Coord, target_distance: i32) -> u32 {
    // Octile distance remains admissible for cardinal cost 10 and diagonal cost 14.
    let dx = ((coord.0 - goal.0).unsigned_abs() as i32 - target_distance).max(0) as u32;
    let dy = ((coord.1 - goal.1).unsigned_abs() as i32 - target_distance).max(0) as u32;
    14 * dx.min(dy) + 10 * dx.abs_diff(dy)
}

#[cfg(test)]
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

fn apply_skip_first(mut nodes: Vec<Coord>, skip_first: bool) -> Vec<Coord> {
    if skip_first && !nodes.is_empty() {
        nodes.remove(0);
    }
    nodes
}

fn empty_list() -> Result<ByondValue, NavPathError> {
    Ok(Vec::<ByondValue>::new().to_byond()?)
}

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

fn parse_job_id(value: &ByondValue) -> Option<u64> {
    if value.is_number() {
        let number = value.get_number().ok()?;
        return (number >= 0.0 && number.fract() == 0.0).then_some(number as u64);
    }
    value.get_string().ok()?.parse().ok()
}

fn prune_expired_jobs(jobs: &mut HashMap<u64, SearchJob>) {
    let now = Instant::now();
    jobs.retain(|_, job| now.duration_since(job.last_touched) < IDLE_TIMEOUT);
}

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

#[allow(clippy::too_many_arguments)]
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
) -> Result<ByondValue, NavPathError> {
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
) -> Result<ByondValue, NavPathError> {
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
    let job_id = NEXT_JOB_ID.fetch_add(1, AtomicOrdering::Relaxed);
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

#[byond_fn]
#[allow(dead_code)]
fn navmap_pathfinder_resume_ffi(
    job_id: ByondValue,
    pass_info: ByondValue,
) -> Result<ByondValue, NavPathError> {
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
    let Some(job) = job else {
        return status_response(
            "error",
            Some(job_id),
            None,
            Some("unknown or expired navmap pathfinder job"),
        );
    };
    match run_job(job, job_id, &pass_info) {
        Ok((Some(job), response)) => {
            SEARCH_JOBS
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner())
                .insert(job_id, job);
            Ok(response)
        }
        Ok((None, response)) => Ok(response),
        Err(error) => status_response("error", Some(job_id), None, Some(&error.to_string())),
    }
}

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
    status_response(
        if removed { "cancelled" } else { "error" },
        Some(job_id),
        None,
        (!removed).then_some("unknown or expired navmap pathfinder job"),
    )
}

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

    #[test]
    fn option_helpers_cover_target_range_space_and_avoidance() {
        assert!(chebyshev_distance((7, 7), (10, 10)) <= 3);
        assert!(!turf_allowed(true, false));
        assert!(turf_allowed(false, false));
        assert!(!coordinate_allowed((12, 12), (10, 10), 2, Some((12, 12))));
        assert!(!coordinate_allowed((13, 10), (10, 10), 2, None));
    }

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
