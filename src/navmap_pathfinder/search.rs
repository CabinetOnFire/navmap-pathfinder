//! Resumable single-z A* search implementation.

use super::*;

impl SearchJob {
    /// Creates an empty resumable search.
    pub(super) fn new(
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
    pub(super) fn step(&mut self, pass_info: &ByondValue) -> Result<JobProgress, NavPathError> {
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
    pub(super) fn final_path(
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
