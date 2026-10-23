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
