//! BYOND-exported procedures for searches and navmap cache updates.

use super::*;
use meowtonin::byond_fn;

macro_rules! return_status_error {
    ($body:block) => {{
        let result = (|| -> Result<ByondValue, NavPathError> { $body })();
        match result {
            Ok(response) => Ok(response),
            Err(error) => status_response("error", None, None, Some(&error.to_string())),
        }
    }};
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
    allow_nav_links: ByondValue,
    cost_modifiers: ByondValue,
) -> Result<ByondValue, NavPathError> {
    return_status_error!({
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
                allow_nav_links,
                cost_modifiers,
            )?
            else {
                return status_response_with_actions(
                    "no_path",
                    None,
                    Some(empty_list()?),
                    Some(ByondValue::new_list()?),
                    None,
                );
            };
            loop {
                match job.step(&pass_info)? {
                    SearchProgress::InProgress => continue,
                    SearchProgress::NoPath => {
                        return status_response_with_actions(
                            "no_path",
                            None,
                            Some(empty_list()?),
                            Some(ByondValue::new_list()?),
                            None,
                        );
                    }
                    SearchProgress::Stale => {
                        return status_response("stale_topology", None, None, None);
                    }
                    SearchProgress::StaleCosts => {
                        return status_response("stale_costs", None, None, None);
                    }
                    SearchProgress::StalePassFlags => {
                        return status_response("stale_pass_flags", None, None, None);
                    }
                    SearchProgress::Found(nodes, actions) => {
                        let (path, actions) = job.final_path(nodes, actions, &pass_info)?;
                        if let Some(status) = job.final_stale_status(&pass_info)? {
                            return status_response(status, None, None, None);
                        }
                        return status_response_with_actions(
                            "complete",
                            None,
                            Some(path),
                            Some(actions),
                            None,
                        );
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
            max_path_cost,
            allow_nav_links,
            cost_modifiers,
        )?
        else {
            return status_response_with_actions(
                "no_path",
                None,
                Some(empty_list()?),
                Some(ByondValue::new_list()?),
                None,
            );
        };
        loop {
            match job.step(&pass_info)? {
                SearchProgress::InProgress => continue,
                SearchProgress::Stale => {
                    return status_response("stale_topology", None, None, None);
                }
                SearchProgress::StaleCosts => {
                    return status_response("stale_costs", None, None, None);
                }
                SearchProgress::StalePassFlags => {
                    return status_response("stale_pass_flags", None, None, None);
                }
                SearchProgress::NoPath => {
                    return status_response_with_actions(
                        "no_path",
                        None,
                        Some(empty_list()?),
                        Some(ByondValue::new_list()?),
                        None,
                    );
                }
                SearchProgress::Found(nodes, actions) => {
                    let (path, actions) = job.final_path(nodes, actions, &pass_info)?;
                    if let Some(status) = job.final_stale_status(&pass_info)? {
                        return status_response(status, None, None, None);
                    }
                    return status_response_with_actions(
                        "complete",
                        None,
                        Some(path),
                        Some(actions),
                        None,
                    );
                }
            }
        }
    })
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
    allow_nav_links: ByondValue,
    cost_modifiers: ByondValue,
) -> Result<ByondValue, NavPathError> {
    return_status_error!({
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
                allow_nav_links,
                cost_modifiers,
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
            max_path_cost,
            allow_nav_links,
            cost_modifiers,
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
    })
}

/// Resumes and returns the next result for a native pathfinding job.
#[byond_fn]
#[allow(dead_code)]
fn navmap_pathfinder_resume_ffi(
    job_id: ByondValue,
    pass_info: ByondValue,
) -> Result<ByondValue, NavPathError> {
    return_status_error!({
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
                Err(error) => {
                    status_response("error", Some(job_id), None, Some(&error.to_string()))
                }
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
    })
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
    nav_cost_flags: ByondValue,
) -> Result<ByondValue, NavPathError> {
    // Replacing cached values immediately prevents stale edges and terrain costs.
    cache_nav_data(
        coords_from_values(&x, &y, &z)?,
        nav_pass.get_number()? as i32,
        nav_cost_flags.get_number().unwrap_or(0.0).max(0.0) as u32,
    )?;
    Ok(ByondValue::NULL)
}

/// Updates cached packed values for a batch of turfs.
#[byond_fn]
#[allow(dead_code)]
fn navmap_bulk_update_ffi(flat_list: ByondValue) -> Result<ByondValue, NavPathError> {
    let values = flat_list.read_list()?;
    if !values.len().is_multiple_of(5) {
        return Err(NavPathError::InvalidBulkUpdateLength);
    }
    // DM sends x, y, z, nav_pass, and cost flags for every turf.
    for entry in values.chunks_exact(5) {
        cache_nav_data(
            coords_from_values(&entry[0], &entry[1], &entry[2])?,
            entry[3].get_number()? as i32,
            entry[4].get_number().unwrap_or(0.0).max(0.0) as u32,
        )?;
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
    return_status_error!({
        let values = flat_list.read_list()?;
        if !values.len().is_multiple_of(3) {
            return Err(NavPathError::InvalidTopologyUpdateLength);
        }
        let generation = nonnegative_generation(&generation)?;
        let mut topology = Topology {
            generation,
            ..Topology::default()
        };
        for entry in values.chunks_exact(3) {
            let group = topology_i32(&entry[0])?;
            let layer = topology_i16(&entry[1])?;
            let z = topology_i16(&entry[2])?;
            topology
                .z_to_layer
                .insert(z, LayerLocation { group, layer });
            topology.layer_to_z.insert((group, layer), z);
        }
        *TOPOLOGY.lock().unwrap_or_else(|p| p.into_inner()) = Arc::new(topology);
        Ok(ByondValue::new_num((values.len() / 3) as f32))
    })
}

/// Returns native topology and navigation-link cache sizes for diagnostics.
#[byond_fn]
#[allow(dead_code)]
fn navmap_pathfinder_state_ffi() -> Result<ByondValue, NavPathError> {
    let topology = TOPOLOGY.lock().unwrap_or_else(|p| p.into_inner()).clone();
    let nav_links = NAV_LINKS.lock().unwrap_or_else(|p| p.into_inner()).len();
    let mut result = ByondValue::new_list()?;
    result.write_list_index(
        ByondValue::new_string("generation"),
        ByondValue::new_num(topology.generation as f32),
    )?;
    result.write_list_index(
        ByondValue::new_string("links_generation"),
        ByondValue::new_num(topology.links_generation as f32),
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
        ByondValue::new_string("nav_link_count"),
        ByondValue::new_num(nav_links as f32),
    )?;
    Ok(result)
}

/// Replaces the native navigation-link cache.
#[byond_fn]
#[allow(dead_code)]
fn navmap_links_update_ffi(
    generation: ByondValue,
    flat_list: ByondValue,
) -> Result<ByondValue, NavPathError> {
    return_status_error!({
        let values = flat_list.read_list()?;
        if !values.len().is_multiple_of(8) {
            return Err(NavPathError::InvalidNavLinks);
        }
        let generation = nonnegative_integer(&generation)?;
        let mut nav_links = HashMap::new();
        let mut nav_links_by_source: HashMap<TurfCoords, Vec<NavLink>> = HashMap::new();
        for entry in values.chunks_exact(8) {
            let id = nonnegative_integer(&entry[0])?;
            if id == 0 {
                return Err(NavPathError::InvalidNavLinks);
            }
            let source = (
                integer_i16(&entry[1])?,
                integer_i16(&entry[2])?,
                integer_i16(&entry[3])?,
            );
            let destination = (
                integer_i16(&entry[4])?,
                integer_i16(&entry[5])?,
                integer_i16(&entry[6])?,
            );
            if source.0 <= 0
                || source.1 <= 0
                || source.2 <= 0
                || destination.0 <= 0
                || destination.1 <= 0
                || destination.2 <= 0
            {
                return Err(NavPathError::InvalidNavLinks);
            }
            let cost = positive_u32(&entry[7])?;
            let nav_link = NavLink {
                id,
                source,
                destination,
                cost,
            };
            if nav_links.insert(id, nav_link).is_some() {
                return Err(NavPathError::InvalidNavLinks);
            }
            nav_links_by_source
                .entry(source)
                .or_default()
                .push(nav_link);
        }
        let nav_link_count = nav_links.len();
        *NAV_LINKS.lock().unwrap_or_else(|p| p.into_inner()) = nav_links;
        *NAV_LINKS_BY_SOURCE
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = Arc::new(nav_links_by_source);
        let mut topology = TOPOLOGY.lock().unwrap_or_else(|p| p.into_inner());
        Arc::make_mut(&mut topology).links_generation = generation;
        Ok(ByondValue::new_num(nav_link_count as f32))
    })
}

fn nonnegative_generation(value: &ByondValue) -> Result<u64, NavPathError> {
    let number = value
        .get_number()
        .map_err(|_| NavPathError::InvalidTopologyUpdateLength)?;
    if !number.is_finite() || number < 0.0 || number.fract() != 0.0 || number > u64::MAX as f32 {
        return Err(NavPathError::InvalidTopologyUpdateLength);
    }
    Ok(number as u64)
}

fn nonnegative_integer(value: &ByondValue) -> Result<u64, NavPathError> {
    let number = value
        .get_number()
        .map_err(|_| NavPathError::InvalidNavLinks)?;
    if !number.is_finite() || number < 0.0 || number.fract() != 0.0 || number > u64::MAX as f32 {
        return Err(NavPathError::InvalidNavLinks);
    }
    Ok(number as u64)
}

pub(super) fn nonnegative_i32(value: &ByondValue) -> Result<i32, NavPathError> {
    let number = value
        .get_number()
        .map_err(|_| NavPathError::InvalidSearchArgument)?;
    if !number.is_finite() || number < 0.0 || number.fract() != 0.0 || number > i32::MAX as f32 {
        return Err(NavPathError::InvalidSearchArgument);
    }
    Ok(number as i32)
}

pub(super) fn nonnegative_u32(value: &ByondValue) -> Result<u32, NavPathError> {
    let number = value
        .get_number()
        .map_err(|_| NavPathError::InvalidSearchArgument)?;
    if !number.is_finite() || number < 0.0 || number.fract() != 0.0 || number > u32::MAX as f32 {
        return Err(NavPathError::InvalidSearchArgument);
    }
    Ok(number as u32)
}

fn integer_i16(value: &ByondValue) -> Result<i16, NavPathError> {
    let number = value
        .get_number()
        .map_err(|_| NavPathError::InvalidNavLinks)?;
    if !number.is_finite()
        || number.fract() != 0.0
        || number < i16::MIN as f32
        || number > i16::MAX as f32
    {
        return Err(NavPathError::InvalidNavLinks);
    }
    Ok(number as i16)
}

fn positive_u32(value: &ByondValue) -> Result<u32, NavPathError> {
    let number = value
        .get_number()
        .map_err(|_| NavPathError::InvalidNavLinks)?;
    if !number.is_finite() || number < 1.0 || number.fract() != 0.0 || number > u32::MAX as f32 {
        return Err(NavPathError::InvalidNavLinks);
    }
    Ok(number as u32)
}

fn topology_i32(value: &ByondValue) -> Result<i32, NavPathError> {
    let number = value
        .get_number()
        .map_err(|_| NavPathError::InvalidTopologyUpdateLength)?;
    if !number.is_finite()
        || number.fract() != 0.0
        || number < i32::MIN as f32
        || number > i32::MAX as f32
    {
        return Err(NavPathError::InvalidTopologyUpdateLength);
    }
    Ok(number as i32)
}

fn topology_i16(value: &ByondValue) -> Result<i16, NavPathError> {
    let number = value
        .get_number()
        .map_err(|_| NavPathError::InvalidTopologyUpdateLength)?;
    if !number.is_finite()
        || number.fract() != 0.0
        || number < i16::MIN as f32
        || number > i16::MAX as f32
    {
        return Err(NavPathError::InvalidTopologyUpdateLength);
    }
    Ok(number as i16)
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
