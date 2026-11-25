//! Multi-layer octilinear PathFinder negotiated-congestion autorouter with
//! parallel multi-start, bad-pass undo, checkpointing, and initial-session resume.

use std::cmp::Ordering;
use std::collections::{BTreeMap, HashSet};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering as AtomicOrdering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::astar::{astar_connect, unpack_node, PassParams, SearchScratch};
use crate::board::{Board, NetRoute, Pad, RoutedSegment, RoutedVia};
use crate::connectivity::{count_all_unconnected, count_unconnected_pairs, score_routes, RouteScore};
use crate::drc::{check_drc, DrcReport};
use crate::fanout::{analyze_fanout, FanoutPlan};
use crate::geometry::Point;
use crate::grid::{NetFootprint, RoutingGrid};
use crate::optimize::route_wirelength;
use crate::signal;

/// Configuration for the autorouter.
#[derive(Debug, Clone)]
pub struct RouterConfig {
    pub max_passes: usize,
    pub ignored_nets: HashSet<String>,
    /// Override DSN `(via_at_smd on|off)` when `Some`.
    pub via_at_smd: Option<bool>,
    pub threads: usize,
    pub timeout_secs: Option<u64>,
    pub optimize: bool,
    /// Number of routing starts (default 4). Start 0 runs canonical order; additional
    /// shuffled starts run in parallel only if start 0 leaves unrouted pairs or DRC issues.
    pub multi_start: usize,
    /// Optional pre-routed net wiring (e.g. from `--initial-session`) to resume from.
    pub initial_routes: Option<Vec<NetRoute>>,
    /// Optional cancellation flag checked between passes and net routes.
    pub stop_flag: Option<Arc<AtomicBool>>,
}

impl Default for RouterConfig {
    fn default() -> Self {
        Self {
            max_passes: 20,
            ignored_nets: HashSet::new(),
            via_at_smd: None,
            threads: 1,
            timeout_secs: None,
            optimize: true,
            multi_start: 4,
            initial_routes: None,
            stop_flag: None,
        }
    }
}

/// Summary statistics from a routing run.
#[derive(Debug, Clone)]
pub struct RouteStats {
    pub total_nets: usize,
    pub routable_nets: usize,
    pub routed_nets: usize,
    pub unconnected_pairs: usize,
    pub total_segments: usize,
    pub total_vias: usize,
    pub total_wirelength_um: f64,
    pub passes_run: usize,
    pub starts_run: usize,
    pub drc_violations: usize,
    pub dense_bga_pads: usize,
    pub pofv_capable_pads: usize,
    pub grid_step_um: f64,
    pub stopped_early: bool,
    pub elapsed_ms: u128,
}

/// Output of routing a board.
#[derive(Debug, Clone)]
pub struct RouteResult {
    pub routes: Vec<NetRoute>,
    pub stats: RouteStats,
}

#[derive(Clone)]
pub(crate) struct Terminal {
    pub(crate) exact: Point,
    pub(crate) pad_id: Option<usize>,
    pub(crate) layers: Vec<usize>,
    /// `(sig_layer, gx, gy, optional_via_pos, pad_layer_for_stub)`.
    pub(crate) entries: Vec<(usize, usize, usize, Option<Point>, usize)>,
}

struct StartOutcome {
    routes: Vec<NetRoute>,
    score: RouteScore,
    passes_run: usize,
    stopped_early: bool,
}

/// Routes all non-ignored nets on `board` using multi-layer PathFinder negotiated congestion.
pub fn route_board(board: &Board, config: &RouterConfig) -> RouteResult {
    let none: Option<&fn(&[NetRoute], RouteScore)> = None;
    route_board_with_checkpoint(board, config, none)
}

/// Routes `board`, invoking `on_best` whenever a strictly better routing solution is found.
pub fn route_board_with_checkpoint<F>(
    board: &Board,
    config: &RouterConfig,
    on_best: Option<&F>,
) -> RouteResult
where
    F: Fn(&[NetRoute], RouteScore) + Sync,
{
    let start = Instant::now();
    let deadline = config.timeout_secs.map(|s| start + Duration::from_secs(s));

    let fanout = analyze_fanout(board);
    let base_grid = RoutingGrid::build(board, &fanout, config.threads);

    let routable_nets: Vec<usize> = board
        .nets
        .iter()
        .filter(|n| !n.ignored && n.pad_ids.len() >= 2)
        .map(|n| n.id)
        .collect();

    let net_components: Vec<Vec<Vec<Terminal>>> = board
        .nets
        .iter()
        .map(|n| {
            if n.ignored || n.pad_ids.len() < 2 {
                Vec::new()
            } else {
                build_net_components(board, &base_grid, &fanout, n.id)
            }
        })
        .collect();

    let mut canonical_order = routable_nets.clone();
    canonical_order.sort_by(|&a, &b| {
        net_span(board, a)
            .partial_cmp(&net_span(board, b))
            .unwrap_or(Ordering::Equal)
    });

    let global_best = Mutex::new(RouteScore::WORST);
    let notify_best = |routes: &[NetRoute], score: RouteScore| {
        if let Ok(mut guard) = global_best.lock() {
            if score.is_better_than(*guard) {
                *guard = score;
                drop(guard);
                if let Some(cb) = on_best {
                    cb(routes, score);
                }
            }
        }
    };

    // Start 0 uses the canonical net ordering; boards routed cleanly pay zero multi-start cost.
    let outcome0 = run_single_start(
        board,
        &base_grid,
        &net_components,
        &routable_nets,
        &canonical_order,
        config,
        deadline,
        &notify_best,
        None,
    );

    let total_starts = config.multi_start.max(1);
    let mut best_outcome = outcome0;
    let mut starts_run = 1usize;

    if total_starts > 1
        && !best_outcome.score.is_perfect()
        && !is_cancelled(deadline, config.stop_flag.as_ref(), None)
        && (config.max_passes > 0 || config.initial_routes.is_none())
    {
        let remaining = total_starts - 1;
        let worker_count = config.threads.max(1).min(remaining);
        let next_seed = AtomicUsize::new(1);
        let found_perfect = AtomicBool::new(false);
        let collected: Mutex<Vec<(usize, StartOutcome)>> = Mutex::new(Vec::with_capacity(remaining));

        std::thread::scope(|s| {
            for _ in 0..worker_count {
                let base_ref = &base_grid;
                let comps_ref = &net_components;
                let routable_ref = &routable_nets;
                let canon_ref = &canonical_order;
                let notify_ref = &notify_best;
                let next_ref = &next_seed;
                let perfect_ref = &found_perfect;
                let collected_ref = &collected;
                s.spawn(move || loop {
                    if perfect_ref.load(AtomicOrdering::Relaxed)
                        || is_cancelled(deadline, config.stop_flag.as_ref(), None)
                    {
                        break;
                    }
                    let seed = next_ref.fetch_add(1, AtomicOrdering::SeqCst);
                    if seed >= total_starts {
                        break;
                    }
                    let order = seeded_net_order(canon_ref, seed);
                    let res = run_single_start(
                        board,
                        base_ref,
                        comps_ref,
                        routable_ref,
                        &order,
                        config,
                        deadline,
                        notify_ref,
                        Some(perfect_ref),
                    );
                    if res.score.is_perfect() {
                        perfect_ref.store(true, AtomicOrdering::Relaxed);
                    }
                    if let Ok(mut guard) = collected_ref.lock() {
                        guard.push((seed, res));
                    }
                });
            }
        });

        let mut extras = collected.into_inner().unwrap_or_default();
        extras.sort_by_key(|(seed, _)| *seed);
        starts_run += extras.len();
        for (_seed, cand) in extras {
            if cand.score.is_better_than(best_outcome.score) {
                best_outcome = cand;
            }
        }
    }

    let mut routes = best_outcome.routes;
    let final_drc = check_drc(board, &routes);
    if std::env::var_os("VIAHOP_DEBUG").is_some() {
        log_unconnected_debug(board, &base_grid, &fanout, &net_components, &routable_nets, &routes);
    }

    let unconnected_pairs = count_all_unconnected(board, &net_components, &mut routes, &routable_nets);
    let routed_nets = routable_nets
        .iter()
        .filter(|&&nid| routes[nid].unconnected_pairs == 0)
        .count();
    let total_segments: usize = routes.iter().map(|r| r.segments.len()).sum();
    let total_vias: usize = routes.iter().map(|r| r.vias.len()).sum();
    let total_wirelength_um: f64 = routes.iter().map(route_wirelength).sum();

    let stats = RouteStats {
        total_nets: board.nets.len(),
        routable_nets: routable_nets.len(),
        routed_nets,
        unconnected_pairs,
        total_segments,
        total_vias,
        total_wirelength_um,
        passes_run: best_outcome.passes_run,
        starts_run,
        drc_violations: final_drc.violations.len(),
        dense_bga_pads: fanout.dense_bga_count,
        pofv_capable_pads: fanout.pofv_capable_count,
        grid_step_um: base_grid.step,
        stopped_early: best_outcome.stopped_early,
        elapsed_ms: start.elapsed().as_millis(),
    };

    RouteResult { routes, stats }
}

#[allow(clippy::too_many_arguments)]
fn run_single_start<F>(
    board: &Board,
    base_grid: &RoutingGrid,
    net_components: &[Vec<Vec<Terminal>>],
    routable_nets: &[usize],
    net_order: &[usize],
    config: &RouterConfig,
    deadline: Option<Instant>,
    notify_best: &F,
    peer_perfect: Option<&AtomicBool>,
) -> StartOutcome
where
    F: Fn(&[NetRoute], RouteScore),
{
    let mut grid = base_grid.clone();
    let mut routes: Vec<NetRoute> = (0..board.nets.len())
        .map(|id| NetRoute {
            net_id: id,
            ..Default::default()
        })
        .collect();
    let mut footprints: Vec<NetFootprint> = (0..board.nets.len())
        .map(|_| NetFootprint::default())
        .collect();

    let has_initial = config.initial_routes.is_some();
    let mut best_routes: Vec<NetRoute> = routes.clone();
    let mut best_score = RouteScore::WORST;
    let mut last_drc: Option<DrcReport> = None;

    if let Some(ref init) = config.initial_routes {
        for nr in init {
            if nr.net_id < routes.len() && !board.nets[nr.net_id].ignored {
                routes[nr.net_id] = nr.clone();
            }
        }
        for &nid in routable_nets {
            routes[nid].unconnected_pairs =
                count_unconnected_pairs(board, &net_components[nid], &routes[nid]);
            if !routes[nid].segments.is_empty() || !routes[nid].vias.is_empty() {
                footprints[nid] = grid.add_footprint(board, &routes[nid]);
            }
        }
        let (init_score, init_drc) = score_routes(board, &routes);
        best_score = init_score;
        best_routes = routes.clone();
        last_drc = Some(init_drc);
        notify_best(&best_routes, best_score);
    } else {
        for &nid in routable_nets {
            routes[nid].unconnected_pairs = net_components[nid].len().saturating_sub(1);
        }
    }

    let n3d = grid.sig_layers.len() * grid.n2d();
    let mut scratch = SearchScratch::new(n3d);
    let effective_passes = if has_initial {
        config.max_passes
    } else {
        config.max_passes.max(1)
    };
    let mut passes_run = 0;
    let mut stopped_early = false;

    for pass in 0..effective_passes {
        if is_cancelled(deadline, config.stop_flag.as_ref(), peer_perfect) {
            stopped_early = true;
            break;
        }
        passes_run = pass + 1;

        let params = PassParams {
            present_factor: if pass == 0 {
                0.75
            } else {
                // Keep present_factor bounded so long nets never stall on a single congested cell.
                (1.6 * 1.38_f32.powi((pass as i32).min(12))).min(110.0)
            },
            h_weight: if pass < 3 { 1.30 } else { 1.18 },
            strict_mode: false,
            max_expansions: 1_200_000,
        };

        let to_route: Vec<usize> = if pass == 0 && !has_initial {
            net_order.to_vec()
        } else {
            let mut dirty: Vec<usize> = net_order
                .iter()
                .copied()
                .filter(|&nid| {
                    routes[nid].unconnected_pairs > 0
                        || (routes[nid].segments.is_empty()
                            && routes[nid].vias.is_empty()
                            && net_components[nid].len() >= 2)
                        || grid.has_conflict(&footprints[nid])
                })
                .collect();
            if dirty.is_empty() || pass == 0 || pass % 3 == 0 {
                let drc = last_drc
                    .take()
                    .unwrap_or_else(|| check_drc(board, &routes));
                if drc.violations.is_empty() && dirty.is_empty() {
                    last_drc = Some(drc);
                    break;
                }
                if !drc.violations.is_empty() {
                    grid.bump_drc_history(&drc, 2.5);
                    for nid in drc.conflicting_nets {
                        if !dirty.contains(&nid) && !board.nets[nid].ignored {
                            dirty.push(nid);
                        }
                    }
                }
            }
            if dirty.is_empty() {
                break;
            }
            if pass > 0 {
                dirty.sort_by(|&a, &b| {
                    routes[b]
                        .unconnected_pairs
                        .cmp(&routes[a].unconnected_pairs)
                        .then_with(|| {
                            net_span(board, b)
                                .partial_cmp(&net_span(board, a))
                                .unwrap_or(Ordering::Equal)
                        })
                });
            }
            dirty
        };

        last_drc = None;
        for &nid in &to_route {
            if is_cancelled(deadline, config.stop_flag.as_ref(), peer_perfect) {
                stopped_early = true;
                break;
            }
            grid.remove_footprint(&footprints[nid]);
            let nr = route_single_net(board, &grid, &net_components[nid], nid, &params, &mut scratch);
            footprints[nid] = grid.add_footprint(board, &nr);
            routes[nid] = nr;
        }

        let congested = grid.bump_history(&footprints, 1.0 + pass as f32 * 0.4);
        let unconn: usize = routes.iter().map(|r| r.unconnected_pairs).sum();

        // Run full continuous DRC when clean, every 3rd pass, on unconnected improvements, or on final pass.
        let need_drc = congested == 0
            || pass % 3 == 2
            || pass + 1 == effective_passes
            || unconn < best_score.unconnected_pairs;
        if need_drc {
            let (pass_score, pass_drc) = score_routes(board, &routes);
            if pass_score.is_better_than(best_score) {
                best_score = pass_score;
                best_routes = routes.clone();
                notify_best(&best_routes, best_score);
            }
            if pass_score.is_perfect() {
                last_drc = Some(pass_drc);
                break;
            }
            if !pass_drc.violations.is_empty() && (congested == 0 || unconn == 0) {
                grid.bump_drc_history(&pass_drc, 3.5);
            }
            // Undo bad pass once PathFinder has stabilized past initial congestion escalation.
            if pass >= 3
                && best_score.unconnected_pairs < usize::MAX
                && best_score.drc_violations <= pass_score.drc_violations + 5
            {
                let bad_limit =
                    best_score.unconnected_pairs + (best_score.unconnected_pairs / 3).max(4);
                if pass_score.unconnected_pairs > bad_limit {
                    routes = best_routes.clone();
                    rebuild_occupancy(board, &mut grid, &routes, &mut footprints);
                    last_drc = None;
                    continue;
                }
            }
            last_drc = Some(pass_drc);
        }
    }

    // Rescue + DRC convergence loop: punch through remaining unconnected nets, then displace peers.
    if effective_passes > 0 && !best_score.is_perfect() {
        for rescue_round in 0..10 {
            if is_cancelled(deadline, config.stop_flag.as_ref(), peer_perfect) {
                stopped_early = true;
                break;
            }
            let unconn_nets: Vec<usize> = net_order
                .iter()
                .copied()
                .filter(|&nid| routes[nid].unconnected_pairs > 0)
                .collect();
            let drc_cur = last_drc.take().unwrap_or_else(|| check_drc(board, &routes));
            let cong_nets: Vec<usize> = net_order
                .iter()
                .copied()
                .filter(|&nid| grid.has_conflict(&footprints[nid]))
                .collect();

            if unconn_nets.is_empty() && drc_cur.violations.is_empty() && cong_nets.is_empty() {
                break;
            }

            let mut protected: HashSet<usize> = HashSet::new();
            let drc_after = if !unconn_nets.is_empty() {
                let rescue_params = PassParams {
                    present_factor: 8.0 + rescue_round as f32 * 4.0,
                    h_weight: 1.85,
                    strict_mode: false,
                    max_expansions: 800_000,
                };
                for nid in unconn_nets {
                    if is_cancelled(deadline, config.stop_flag.as_ref(), peer_perfect) {
                        stopped_early = true;
                        break;
                    }
                    grid.remove_footprint(&footprints[nid]);
                    let nr = route_single_net(
                        board,
                        &grid,
                        &net_components[nid],
                        nid,
                        &rescue_params,
                        &mut scratch,
                    );
                    footprints[nid] = grid.add_footprint(board, &nr);
                    routes[nid] = nr;
                    protected.insert(nid);
                }
                check_drc(board, &routes)
            } else {
                drc_cur
            };

            if !drc_after.violations.is_empty() {
                grid.bump_drc_history(&drc_after, 5.0 + rescue_round as f32 * 2.0);
            }
            grid.bump_history(&footprints, 2.5 + rescue_round as f32);

            let mut peers: Vec<usize> = net_order
                .iter()
                .copied()
                .filter(|&nid| {
                    !protected.contains(&nid)
                        && (grid.has_conflict(&footprints[nid])
                            || drc_after.conflicting_nets.contains(&nid))
                })
                .collect();
            if peers.is_empty() {
                peers.extend(
                    drc_after
                        .conflicting_nets
                        .into_iter()
                        .filter(|&nid| !board.nets[nid].ignored),
                );
            }
            peers.sort_by(|&a, &b| {
                net_span(board, a)
                    .partial_cmp(&net_span(board, b))
                    .unwrap_or(Ordering::Equal)
            });

            let peer_params = PassParams {
                present_factor: (60.0 + rescue_round as f32 * 25.0).min(180.0),
                h_weight: 1.35,
                strict_mode: rescue_round >= 6,
                max_expansions: 600_000,
            };
            for nid in peers {
                if is_cancelled(deadline, config.stop_flag.as_ref(), peer_perfect) {
                    stopped_early = true;
                    break;
                }
                let old_unconn = routes[nid].unconnected_pairs;
                grid.remove_footprint(&footprints[nid]);
                let nr = route_single_net(
                    board,
                    &grid,
                    &net_components[nid],
                    nid,
                    &peer_params,
                    &mut scratch,
                );
                let final_nr = if peer_params.strict_mode && nr.unconnected_pairs > old_unconn {
                    let soft = PassParams {
                        strict_mode: false,
                        present_factor: 120.0,
                        h_weight: 1.85,
                        max_expansions: 700_000,
                    };
                    route_single_net(board, &grid, &net_components[nid], nid, &soft, &mut scratch)
                } else {
                    nr
                };
                footprints[nid] = grid.add_footprint(board, &final_nr);
                routes[nid] = final_nr;
            }

            let (round_score, round_drc) = score_routes(board, &routes);
            if round_score.is_better_than(best_score) {
                best_score = round_score;
                best_routes = routes.clone();
                notify_best(&best_routes, best_score);
            }
            last_drc = Some(round_drc);
            if best_score.is_perfect() {
                break;
            }
        }
    }

    // Always optimize from the best snapshot rather than a degraded late pass.
    if best_score.unconnected_pairs < usize::MAX {
        routes = best_routes;
    }

    count_all_unconnected(board, net_components, &mut routes, routable_nets);
    let (pre_score, _) = score_routes(board, &routes);
    if config.optimize {
        let pre_opt = routes.clone();
        crate::optimize::optimize_routes(board, &mut routes);
        count_all_unconnected(board, net_components, &mut routes, routable_nets);
        let (post_score, _) = score_routes(board, &routes);
        if post_score.unconnected_pairs <= pre_score.unconnected_pairs
            && post_score.drc_violations <= pre_score.drc_violations
        {
            best_score = post_score;
            notify_best(&routes, best_score);
        } else {
            routes = pre_opt;
            best_score = pre_score;
        }
    } else {
        best_score = pre_score;
    }

    StartOutcome {
        routes,
        score: best_score,
        passes_run,
        stopped_early,
    }
}

fn route_single_net(
    board: &Board,
    grid: &RoutingGrid,
    components: &[Vec<Terminal>],
    net_id: usize,
    params: &PassParams,
    scratch: &mut SearchScratch,
) -> NetRoute {
    let mut nr = NetRoute {
        net_id,
        ..Default::default()
    };
    if components.len() < 2 {
        return nr;
    }

    let rules = &board.nets[net_id].rules;
    let mst_edges = build_mst(components);
    let n2d = grid.n2d();

    // Only contains already-routed copper grid nodes (never unrouted POFV anchors).
    let mut comp_nodes: Vec<Vec<(usize, usize, usize)>> = vec![Vec::new(); components.len()];
    let mut uf: Vec<usize> = (0..components.len()).collect();
    let mut via_positions: Vec<Point> = Vec::new();

    for (ca, cb) in mst_edges {
        let ra = uf_find(&mut uf, ca);
        let rb = uf_find(&mut uf, cb);
        if ra == rb {
            continue;
        }

        let src_entries = collect_comp_anchors(components, &uf, ra, n2d, grid.cols);
        let dst_entries = collect_comp_anchors(components, &uf, rb, n2d, grid.cols);
        let src_extra = &comp_nodes[ra];
        let dst_extra = &comp_nodes[rb];

        if (src_entries.is_empty() && src_extra.is_empty())
            || (dst_entries.is_empty() && dst_extra.is_empty())
        {
            nr.unconnected_pairs += 1;
            continue;
        }

        let min_via_dist = (2.0 * rules.via_radius + rules.via_via).max(rules.via_drill + 205.0);
        let path_opt = astar_connect(
            grid,
            net_id,
            &src_entries,
            src_extra,
            &dst_entries,
            dst_extra,
            &via_positions,
            min_via_dist,
            params,
            scratch,
        )
        .or_else(|| {
            let retry = PassParams {
                present_factor: if params.strict_mode {
                    params.present_factor
                } else {
                    params.present_factor.min(10.0)
                },
                h_weight: (params.h_weight + 1.0).max(2.35),
                strict_mode: params.strict_mode,
                max_expansions: params.max_expansions.max(2_000_000),
            };
            astar_connect(
                grid,
                net_id,
                &src_entries,
                src_extra,
                &dst_entries,
                dst_extra,
                &via_positions,
                min_via_dist,
                &retry,
                scratch,
            )
        });
        let Some(path) = path_opt else {
            nr.unconnected_pairs += 1;
            continue;
        };

        let first_node = path[0];
        let last_node = *path.last().unwrap();

        if !node_in_extra(first_node, src_extra, n2d, grid.cols) {
            if let Some((exact, via_opt, pad_layer)) = match_anchor(&src_entries, first_node) {
                emit_landing(
                    board,
                    grid,
                    net_id,
                    first_node,
                    exact,
                    via_opt,
                    pad_layer,
                    true,
                    rules,
                    &mut via_positions,
                    &mut nr,
                );
            }
        }

        // Coalesce consecutive collinear grid steps into single segments immediately during routing.
        let mut active_run: Option<(usize, Point, (isize, isize))> = None;
        let mut run_end = Point::new(0.0, 0.0);
        for pair in path.windows(2) {
            let (sl1, gx1, gy1) = unpack_node(pair[0], n2d, grid.cols);
            let (sl2, gx2, gy2) = unpack_node(pair[1], n2d, grid.cols);
            let p1 = grid.pt_of(gx1, gy1);
            let p2 = grid.pt_of(gx2, gy2);
            if sl1 != sl2 {
                if let Some((layer, start_pt, _)) = active_run.take() {
                    if start_pt.dist(run_end) > 0.5 {
                        nr.segments.push(RoutedSegment {
                            layer,
                            a: start_pt,
                            b: run_end,
                            width: rules.width,
                        });
                    }
                }
                push_unique_via(board, net_id, &mut via_positions, &mut nr, p1, rules);
            } else {
                let bl = grid.sig_layers[sl1];
                let dir = (gx2 as isize - gx1 as isize, gy2 as isize - gy1 as isize);
                match active_run {
                    Some((layer, _, rdir)) if layer == bl && rdir == dir => {
                        run_end = p2;
                    }
                    _ => {
                        if let Some((layer, start_pt, _)) = active_run.take() {
                            if start_pt.dist(run_end) > 0.5 {
                                nr.segments.push(RoutedSegment {
                                    layer,
                                    a: start_pt,
                                    b: run_end,
                                    width: rules.width,
                                });
                            }
                        }
                        active_run = Some((bl, p1, dir));
                        run_end = p2;
                    }
                }
            }
        }
        if let Some((layer, start_pt, _)) = active_run.take() {
            if start_pt.dist(run_end) > 0.5 {
                nr.segments.push(RoutedSegment {
                    layer,
                    a: start_pt,
                    b: run_end,
                    width: rules.width,
                });
            }
        }

        if !node_in_extra(last_node, dst_extra, n2d, grid.cols) {
            if let Some((exact, via_opt, pad_layer)) = match_anchor(&dst_entries, last_node) {
                emit_landing(
                    board,
                    grid,
                    net_id,
                    last_node,
                    exact,
                    via_opt,
                    pad_layer,
                    false,
                    rules,
                    &mut via_positions,
                    &mut nr,
                );
            }
        }

        let mut merged = std::mem::take(&mut comp_nodes[rb]);
        comp_nodes[ra].append(&mut merged);
        for &node in &path {
            comp_nodes[ra].push(unpack_node(node, n2d, grid.cols));
        }
        uf[rb] = ra;
    }

    nr
}

#[allow(clippy::too_many_arguments)]
fn emit_landing(
    board: &Board,
    grid: &RoutingGrid,
    net_id: usize,
    node: u32,
    exact: Point,
    via_opt: Option<Point>,
    pad_layer: usize,
    is_start: bool,
    rules: &crate::board::NetRules,
    via_positions: &mut Vec<Point>,
    nr: &mut NetRoute,
) {
    let (sl, gx, gy) = unpack_node(node, grid.n2d(), grid.cols);
    let gp = grid.pt_of(gx, gy);
    let bl = grid.sig_layers[sl];
    if let Some(vp) = via_opt {
        if bl != pad_layer {
            if exact.dist(vp) > 1.0 {
                let (a, b) = if is_start { (exact, vp) } else { (vp, exact) };
                nr.segments.push(RoutedSegment {
                    layer: pad_layer,
                    a,
                    b,
                    width: rules.width,
                });
            }
            push_unique_via(board, net_id, via_positions, nr, vp, rules);
        }
        if vp.dist(gp) > 1.0 {
            let (a, b) = if is_start { (vp, gp) } else { (gp, vp) };
            nr.segments.push(RoutedSegment {
                layer: bl,
                a,
                b,
                width: rules.width,
            });
        }
    } else if exact.dist(gp) > 1.0 {
        let (a, b) = if is_start { (exact, gp) } else { (gp, exact) };
        nr.segments.push(RoutedSegment {
            layer: bl,
            a,
            b,
            width: rules.width,
        });
    }
}

pub(crate) fn build_net_components(
    board: &Board,
    grid: &RoutingGrid,
    fanout: &FanoutPlan,
    net_id: usize,
) -> Vec<Vec<Terminal>> {
    let pads = &board.nets[net_id].pad_ids;
    if pads.len() < 2 {
        return Vec::new();
    }

    let mut nodes: Vec<(Point, Vec<usize>, Option<usize>)> = Vec::new();
    for &pid in pads {
        let p = &board.pads[pid];
        let ls: Vec<usize> = p.layer_shapes.iter().map(|(l, _)| *l).collect();
        nodes.push((p.center, ls, Some(pid)));
    }
    let pad_count = nodes.len();

    let mut fixed_edges: Vec<(usize, usize)> = Vec::new();
    for fv in &board.fixed_vias {
        if fv.net_id == Some(net_id) {
            nodes.push((fv.pos, fv.layers.clone(), None));
        }
    }
    for fs in &board.fixed_segments {
        if fs.net_id == Some(net_id) {
            let ia = nodes.len();
            nodes.push((fs.a, vec![fs.layer], None));
            let ib = nodes.len();
            nodes.push((fs.b, vec![fs.layer], None));
            fixed_edges.push((ia, ib));
        }
    }

    let mut uf: Vec<usize> = (0..nodes.len()).collect();
    for (a, b) in fixed_edges {
        let ra = uf_find(&mut uf, a);
        let rb = uf_find(&mut uf, b);
        uf[rb] = ra;
    }

    for i in 0..nodes.len() {
        for j in (i + 1)..nodes.len() {
            let touches = match (nodes[i].2, nodes[j].2) {
                (Some(p1), Some(p2)) => pads_physically_touch(&board.pads[p1], &board.pads[p2]),
                (Some(p), None) => pad_touches_point(&board.pads[p], nodes[j].0, &nodes[j].1, 20.0),
                (None, Some(p)) => pad_touches_point(&board.pads[p], nodes[i].0, &nodes[i].1, 20.0),
                (None, None) => {
                    nodes[i].1.iter().any(|l| nodes[j].1.contains(l))
                        && nodes[i].0.dist(nodes[j].0) <= 15.0
                }
            };
            if touches {
                let ra = uf_find(&mut uf, i);
                let rb = uf_find(&mut uf, j);
                uf[rb] = ra;
            }
        }
    }

    let mut sbuf = Vec::new();
    let mut comp_map: BTreeMap<usize, Vec<Terminal>> = BTreeMap::new();
    for (i, (pt, layers, pad_opt)) in nodes.iter().enumerate() {
        let root = uf_find(&mut uf, i);
        if i >= pad_count && !comp_map.contains_key(&root) {
            continue;
        }
        let entries =
            compute_terminal_entries(board, grid, fanout, net_id, *pt, layers, *pad_opt, &mut sbuf);
        comp_map.entry(root).or_default().push(Terminal {
            exact: *pt,
            pad_id: *pad_opt,
            layers: layers.clone(),
            entries,
        });
    }

    comp_map.into_values().collect()
}

#[allow(clippy::too_many_arguments)]
fn compute_terminal_entries(
    board: &Board,
    grid: &RoutingGrid,
    fanout: &FanoutPlan,
    net_id: usize,
    exact: Point,
    layers: &[usize],
    pad_opt: Option<usize>,
    sbuf: &mut Vec<usize>,
) -> Vec<(usize, usize, usize, Option<Point>, usize)> {
    let mut entries = Vec::new();
    let (gx, gy) = grid.snap(exact);
    let c2 = gy * grid.cols + gx;
    let width = board.nets[net_id].rules.width;

    let mut primary_pad_layer = None;
    for &bl in layers {
        if let Some(sl) = grid.board_to_sig[bl] {
            if primary_pad_layer.is_none() {
                primary_pad_layer = Some(bl);
            }
            entries.push((sl, gx, gy, None, bl));
            for dy in -2_isize..=2 {
                for dx in -2_isize..=2 {
                    if dx == 0 && dy == 0 {
                        continue;
                    }
                    let nx = gx as isize + dx;
                    let ny = gy as isize + dy;
                    if nx < 0 || ny < 0 || nx >= grid.cols as isize || ny >= grid.rows as isize {
                        continue;
                    }
                    let (nxu, nyu) = (nx as usize, ny as usize);
                    let nc2 = nyu * grid.cols + nxu;
                    if grid.wire_allowed(sl, nc2, net_id)
                        && board.check_segment_static_with_buf(
                            net_id,
                            bl,
                            exact,
                            grid.pt_of(nxu, nyu),
                            width,
                            0.0,
                            sbuf,
                        )
                    {
                        entries.push((sl, nxu, nyu, None, bl));
                    }
                }
            }
        }
    }

    if let Some(pid) = pad_opt {
        let pf = &fanout.pads[pid];
        if pf.pofv_allowed {
            let pad_bl = primary_pad_layer.unwrap_or(0);
            for (sl, &bl) in grid.sig_layers.iter().enumerate() {
                if bl != pad_bl && grid.wire_allowed(sl, c2, net_id) {
                    entries.push((sl, gx, gy, Some(exact), pad_bl));
                }
            }
        } else if let Some(db_pos) = pf.dogbone_via {
            let (dgx, dgy) = grid.snap(db_pos);
            let dc2 = dgy * grid.cols + dgx;
            let pad_bl = primary_pad_layer.unwrap_or(0);
            for (sl, &bl) in grid.sig_layers.iter().enumerate() {
                if bl != pad_bl && grid.wire_allowed(sl, dc2, net_id) {
                    entries.push((sl, dgx, dgy, Some(db_pos), pad_bl));
                }
            }
        }
    }

    entries
}

pub(crate) fn build_mst(components: &[Vec<Terminal>]) -> Vec<(usize, usize)> {
    let n = components.len();
    if n < 2 {
        return Vec::new();
    }
    let mut in_tree = vec![false; n];
    let mut best_dist = vec![f64::INFINITY; n];
    let mut best_from = vec![0usize; n];
    in_tree[0] = true;
    for j in 1..n {
        best_dist[j] = comp_distance(&components[0], &components[j]);
        best_from[j] = 0;
    }

    let mut edges = Vec::with_capacity(n - 1);
    for _ in 1..n {
        let mut pick = None;
        let mut min_d = f64::INFINITY;
        for j in 0..n {
            if !in_tree[j] && best_dist[j] < min_d {
                min_d = best_dist[j];
                pick = Some(j);
            }
        }
        let Some(u) = pick else { break };
        in_tree[u] = true;
        edges.push((best_from[u], u));
        for j in 0..n {
            if !in_tree[j] {
                let d = comp_distance(&components[u], &components[j]);
                if d < best_dist[j] {
                    best_dist[j] = d;
                    best_from[j] = u;
                }
            }
        }
    }
    edges.sort_by(|&(a1, b1), &(a2, b2)| {
        comp_distance(&components[a1], &components[b1])
            .partial_cmp(&comp_distance(&components[a2], &components[b2]))
            .unwrap_or(Ordering::Equal)
    });
    edges
}

fn collect_comp_anchors(
    components: &[Vec<Terminal>],
    uf: &[usize],
    root: usize,
    n2d: usize,
    cols: usize,
) -> Vec<(u32, Point, Option<Point>, usize)> {
    let mut out = Vec::new();
    for (ci, comp) in components.iter().enumerate() {
        let mut r = ci;
        while uf[r] != r {
            r = uf[r];
        }
        if r != root {
            continue;
        }
        for t in comp {
            for &(sl, gx, gy, via_opt, pad_bl) in &t.entries {
                out.push(((sl * n2d + gy * cols + gx) as u32, t.exact, via_opt, pad_bl));
            }
        }
    }
    out
}

fn rebuild_occupancy(
    board: &Board,
    grid: &mut RoutingGrid,
    routes: &[NetRoute],
    footprints: &mut [NetFootprint],
) {
    grid.clear_occupancy();
    for (nid, nr) in routes.iter().enumerate() {
        if nr.segments.is_empty() && nr.vias.is_empty() {
            footprints[nid] = NetFootprint::default();
        } else {
            footprints[nid] = grid.add_footprint(board, nr);
        }
    }
}

#[inline]
fn is_cancelled(
    deadline: Option<Instant>,
    stop_flag: Option<&Arc<AtomicBool>>,
    peer_perfect: Option<&AtomicBool>,
) -> bool {
    deadline.is_some_and(|dl| Instant::now() >= dl)
        || signal::stop_requested(stop_flag)
        || peer_perfect.is_some_and(|p| p.load(AtomicOrdering::Relaxed))
}

fn seeded_net_order(canonical: &[usize], seed: usize) -> Vec<usize> {
    let mut order = canonical.to_vec();
    if seed == 1 {
        order.reverse();
        return order;
    }
    let mut state = 0x9E37_79B9_7F4A_7C15_u64
        ^ (seed as u64).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    for i in (1..order.len()).rev() {
        state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^= z >> 31;
        let j = (z as usize) % (i + 1);
        order.swap(i, j);
    }
    order
}

fn comp_distance(a: &[Terminal], b: &[Terminal]) -> f64 {
    let mut min_d = f64::INFINITY;
    for ta in a {
        for tb in b {
            min_d = min_d.min(ta.exact.dist(tb.exact));
        }
    }
    min_d
}

fn node_in_extra(node: u32, extra: &[(usize, usize, usize)], n2d: usize, cols: usize) -> bool {
    extra.contains(&unpack_node(node, n2d, cols))
}

fn pads_physically_touch(a: &Pad, b: &Pad) -> bool {
    if !a.bbox.intersects(b.bbox) {
        return false;
    }
    for &(la, ref sa) in &a.layer_shapes {
        if let Some(sb) = b.shape_on_layer(la) {
            if sa.dist_to_circle(b.center, 1.0) <= 0.0 || sb.dist_to_circle(a.center, 1.0) <= 0.0 {
                return true;
            }
        }
    }
    false
}

fn pad_touches_point(pad: &Pad, pt: Point, layers: &[usize], radius: f64) -> bool {
    layers.iter().any(|&l| {
        pad.shape_on_layer(l)
            .is_some_and(|sh| sh.dist_to_circle(pt, radius) <= 0.0)
    })
}

fn match_anchor(
    anchors: &[(u32, Point, Option<Point>, usize)],
    node: u32,
) -> Option<(Point, Option<Point>, usize)> {
    anchors
        .iter()
        .find(|&&(n, _, _, _)| n == node)
        .map(|&(_, pt, via_opt, bl)| (pt, via_opt, bl))
}

fn push_unique_via(
    board: &Board,
    net_id: usize,
    via_positions: &mut Vec<Point>,
    nr: &mut NetRoute,
    pos: Point,
    rules: &crate::board::NetRules,
) {
    if via_positions.iter().any(|p| p.dist_sq(pos) < 25.0) {
        return;
    }
    if board
        .fixed_vias
        .iter()
        .any(|fv| fv.net_id == Some(net_id) && fv.pos.dist_sq(pos) < 25.0)
    {
        return;
    }
    via_positions.push(pos);
    nr.vias.push(RoutedVia {
        pos,
        padstack: rules.via_padstack.clone(),
        radius: rules.via_radius,
        drill: rules.via_drill,
        layers: rules.via_layers.clone(),
    });
}

fn net_span(board: &Board, net_id: usize) -> f64 {
    let ids = &board.nets[net_id].pad_ids;
    if ids.len() < 2 {
        return 0.0;
    }
    let mut bb = board.pads[ids[0]].bbox;
    for &pid in &ids[1..] {
        bb = bb.union(board.pads[pid].bbox);
    }
    (bb.max_x - bb.min_x) + (bb.max_y - bb.min_y)
}

pub(crate) fn uf_find(uf: &mut [usize], mut x: usize) -> usize {
    while uf[x] != x {
        uf[x] = uf[uf[x]];
        x = uf[x];
    }
    x
}

fn log_unconnected_debug(
    board: &Board,
    grid: &RoutingGrid,
    fanout: &FanoutPlan,
    net_components: &[Vec<Vec<Terminal>>],
    routable_nets: &[usize],
    routes: &[NetRoute],
) {
    for &nid in routable_nets {
        if routes[nid].unconnected_pairs == 0 {
            continue;
        }
        let net = &board.nets[nid];
        let pads_info: Vec<String> = net
            .pad_ids
            .iter()
            .map(|&p| {
                let pad = &board.pads[p];
                let (gx, gy) = grid.snap(pad.center);
                let c2 = gy * grid.cols + gx;
                let open_layers: Vec<usize> = (0..grid.sig_layers.len())
                    .filter(|&sl| grid.wire_allowed(sl, c2, nid))
                    .collect();
                format!(
                    "{}-{}@({:.1},{:.1})[open_sl={:?},via_ok={},pofv={}]",
                    pad.refdes,
                    pad.pin_name,
                    pad.center.x,
                    pad.center.y,
                    open_layers,
                    grid.via_allowed(c2, nid),
                    fanout.pads[p].pofv_allowed
                )
            })
            .collect();
        eprintln!(
            "DEBUG UNCONN net={} unconn_pairs={} comps={} pads={:?}",
            net.name,
            routes[nid].unconnected_pairs,
            net_components[nid].len(),
            pads_info,
        );
    }
}
