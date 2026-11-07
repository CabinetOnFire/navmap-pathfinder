use super::search::nav_link_estimate;
use super::*;

/// Packed cache values survive invalidation updates.
#[test]
fn cache_retains_simulated_invalidations() {
    let coords = (3, 2, 1);
    cache_nav_pass(coords, BAKED_FLAG | SIMULATED_FLAG | NORTH);
    assert_eq!(
        cached_nav_pass(coords),
        Some(BAKED_FLAG | SIMULATED_FLAG | NORTH)
    );
    cache_nav_pass(coords, 0);
    assert_eq!(cached_nav_pass(coords), Some(0));
}

#[test]
fn dense_cache_grows_without_losing_snapshots() {
    let mut cache = NavPassCache::default();
    cache.grow_to(2, 2, 1);
    let baked = CachedNavData {
        nav_pass: BAKED_FLAG | NORTH,
        nav_cost_flags: 4,
    };
    assert_eq!(cache.insert((2, 2, 1), baked), None);
    assert_eq!(cache.insert((1, 1, 1), CachedNavData::default()), None);

    cache.grow_to(4, 3, 2);
    assert_eq!(cache.get((2, 2, 1)).unwrap().nav_pass, BAKED_FLAG | NORTH);
    assert_eq!(cache.get((1, 1, 1)).unwrap().nav_pass, 0);
    assert_eq!(
        cache.insert(
            (4, 3, 2),
            CachedNavData {
                nav_pass: 0,
                nav_cost_flags: 8
            }
        ),
        None
    );
    assert_eq!(cache.get((4, 3, 2)).unwrap().nav_cost_flags, 8);
    assert_eq!(cache.get((5, 1, 1)), None);
    assert_eq!(cache.get((0, 1, 1)), None);
}

#[test]
fn published_topology_updates_leave_active_snapshots_unchanged() {
    let original = Arc::new(Topology {
        generation: 1,
        ..Topology::default()
    });
    let active_job_snapshot = Arc::clone(&original);
    let mut published = Arc::clone(&original);
    Arc::make_mut(&mut published).generation = 2;

    assert_eq!(active_job_snapshot.generation, 1);
    assert_eq!(published.generation, 2);
    assert!(!Arc::ptr_eq(&active_job_snapshot, &published));
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
}

/// Queue ordering prefers the lowest estimated cost.
#[test]
fn queue_prefers_lowest_estimate() {
    let mut search = AStarCore::default();
    search.initialize(0, 0);
    let progress = search
        .step::<(), _, _, _>(
            None,
            |node| node == 1,
            |node| {
                if node == 0 {
                    Ok(vec![(1, 5, SearchAction::None), (2, 1, SearchAction::None)])
                } else {
                    Ok(Vec::new())
                }
            },
            |node| Some(if node == 1 { 0 } else { 20 }),
        )
        .unwrap();
    let SearchProgress::Found(path, _) = progress else {
        panic!("expected the lower-estimate node to be selected first");
    };
    assert_eq!(path, [0, 1]);
}

#[test]
fn relaxed_link_model_prunes_disconnected_branches_and_preserves_zero() {
    let mut builder = NavLinkDistanceBuilder::new(0, 0, vec![(1, 2, 1), (3, 4, 1)]);
    let ready = builder.advance(Instant::now() + Duration::from_secs(1), |from, to, _| {
        if from == to {
            Some(0)
        } else if from == 2 && to == 0 {
            Some(10)
        } else {
            None
        }
    });
    assert!(ready);
    let source_distances = builder.finish().unwrap();
    let relaxed = |from, to, _| {
        if from == to {
            Some(0)
        } else if from == 2 && to == 0 {
            Some(10)
        } else {
            None
        }
    };
    assert_eq!(
        nav_link_estimate(1, 0, 0, &source_distances, relaxed),
        Some(11)
    );
    assert_eq!(nav_link_estimate(3, 0, 0, &source_distances, relaxed), None);
    assert_eq!(
        nav_link_estimate(0, 0, 0, &source_distances, relaxed),
        Some(0)
    );
    assert_eq!(heuristic_to_target_range((0, 0), (3, 0), 2), 10);
    assert_eq!(CostProfile::scale_heuristic(30, 0.25), 7);
}

#[test]
#[ignore = "microbenchmark for NavLinkDistanceBuilder deadline polling"]
fn benchmark_builder_deadline_polling() {
    let edges: Vec<_> = (1..=96).map(|node| (node, node + 100, 1)).collect();
    let build = |check_every_unit: bool| {
        let mut builder = NavLinkDistanceBuilder::new(0, 0, edges.clone());
        let ready = if check_every_unit {
            builder.advance_checking_every_work_unit(
                Instant::now() + Duration::from_secs(30),
                |from, to, _| Some(if from == to { 0 } else { 1 }),
            )
        } else {
            builder.advance(Instant::now() + Duration::from_secs(30), |from, to, _| {
                Some(if from == to { 0 } else { 1 })
            })
        };
        assert!(ready);
        std::hint::black_box(builder.finish());
    };
    for _ in 0..5 {
        build(true);
        build(false);
    }
    let time = |check_every_unit| {
        let started = Instant::now();
        for _ in 0..30 {
            build(check_every_unit);
        }
        started.elapsed()
    };
    let each_unit = time(true);
    let batched = time(false);
    eprintln!("builder deadline polling: each-unit={each_unit:?}, every-64={batched:?}");
}

#[test]
fn astar_does_not_queue_nodes_proven_unreachable_by_heuristic() {
    let mut search = AStarCore::default();
    search.initialize(0, 0);
    let progress = search
        .step::<(), _, _, _>(
            None,
            |node| node == 2,
            |node| {
                if node == 0 {
                    Ok(vec![(1, 1, SearchAction::None), (2, 1, SearchAction::None)])
                } else {
                    Ok(Vec::new())
                }
            },
            |node| (node != 1).then_some(0),
        )
        .unwrap();
    let SearchProgress::Found(path, _) = progress else {
        panic!("expected the reachable branch to find the goal");
    };
    assert_eq!(path, [0, 2]);
}

#[test]
fn baked_cost_snapshot_avoids_false_stale_and_detects_later_change() {
    let coords = (7, 9, 2);
    let mut history = CostChangeHistory {
        generation: 2,
        changes: VecDeque::from([CostChange {
            generation: 2,
            coords,
            changed_flags: 1,
        }]),
    };
    let observed = |changed_coords| (changed_coords == coords).then_some(2);
    assert_eq!(
        cost_generation_since_in_history(&history, 1, 1, observed),
        Some(2)
    );

    history.generation = 3;
    history.changes.push_back(CostChange {
        generation: 3,
        coords,
        changed_flags: 1,
    });
    let observed = |changed_coords| (changed_coords == coords).then_some(2);
    assert_eq!(
        cost_generation_since_in_history(&history, 2, 1, observed),
        None
    );
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
            max_cost: 0,
            diagonal_handling: DiagonalHandling::DoNothing,
            simulated_only: false,
            avoid: None,
            nav_links_by_source: Arc::new(HashMap::default()),
            cache: HashMap::default(),
            costs: CostProfile {
                modifiers: Vec::new(),
                combined: HashMap::default(),
                heuristic_scale: 1.0,
            },
            cost_generation: 0,
        },
        (2, 2),
        0,
        DiagonalHandling::DoNothing,
        true,
        None,
        None,
    );
    job.last_touched = last_touched;
    job
}

/// Job pruning removes stale entries but keeps active ones.
#[test]
fn job_registry_prunes_expired_jobs_without_affecting_active_jobs() {
    let now = Instant::now();
    let mut jobs = HashMap::default();
    jobs.extend([
        (1, test_job(now - IDLE_TIMEOUT - Duration::from_millis(1))),
        (2, test_job(now)),
        (3, test_job(now)),
    ]);
    prune_expired_jobs(&mut jobs);
    assert!(!jobs.contains_key(&1));
    assert!(jobs.contains_key(&2));
    assert!(jobs.contains_key(&3));
}
