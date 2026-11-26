//! Fast sub-second static feasibility, sealed-BGA-row, and fixed-copper preflight check.

use std::fmt::Write;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::astar::SearchScratch;
use crate::board::Board;
use crate::fanout::{BgaCluster, analyze_fanout, dogbone_candidates};
use crate::geometry::Point;
use crate::grid::RoutingGrid;
use crate::report::{closest_terminals, json_str, solo_pair_block_reason, terminal_label};
use crate::router::{build_mst, build_net_components};
use crate::signal;

/// Summary of a dense BGA component cluster discovered during preflight.
#[derive(Debug, Clone)]
pub struct BgaClusterSummary {
    pub refdes: String,
    pub total_pads: usize,
    pub dense_pads: usize,
    pub max_ring: usize,
    pub pitch_um: f64,
    pub sealed_count: usize,
}

/// A routable BGA pad with no legal surface, POFV, or dogbone escape channel.
#[derive(Debug, Clone)]
pub struct SealedPadInfo {
    pub refdes: String,
    pub pin: String,
    pub net: String,
    pub pos: Point,
    pub ring: usize,
    pub blocker: String,
}

/// A routable net pin-pair that fails solo empty-grid static routing.
#[derive(Debug, Clone)]
pub struct BlockedNetPair {
    pub net: String,
    pub class_name: String,
    pub from_pin: String,
    pub from_pos: Point,
    pub to_pin: String,
    pub to_pos: Point,
    pub reason: String,
}

/// Static DRC violation detected on pre-existing fixed copper before routing.
#[derive(Debug, Clone)]
pub struct FixedViolation {
    pub kind: &'static str,
    pub net_a: Option<String>,
    pub net_b: Option<String>,
    pub layer: Option<String>,
    pub pos: Point,
    pub gap_um: f64,
    pub required_um: f64,
    pub blocker: String,
}

/// Complete machine-readable static preflight feasibility report (`--preflight`).
#[derive(Debug, Clone)]
pub struct PreflightReport {
    pub ok: bool,
    pub stopped_early: bool,
    pub total_nets: usize,
    pub routable_nets: usize,
    pub grid_step_um: f64,
    pub via_at_smd: bool,
    pub bga_clusters: Vec<BgaClusterSummary>,
    pub sealed_pads: Vec<SealedPadInfo>,
    pub blocked_nets: Vec<BlockedNetPair>,
    pub fixed_violations: Vec<FixedViolation>,
    pub elapsed_ms: u128,
}

/// Runs fast static feasibility analysis without performing multi-pass routing.
pub fn run_preflight(board: &Board, threads: usize) -> PreflightReport {
    run_preflight_with_options(board, threads, None, None)
}

/// Runs static feasibility analysis with optional soft timeout and cancellation flag.
pub fn run_preflight_with_options(
    board: &Board,
    threads: usize,
    timeout_secs: Option<u64>,
    stop_flag: Option<Arc<AtomicBool>>,
) -> PreflightReport {
    let start = Instant::now();
    let deadline = timeout_secs.map(|s| start + Duration::from_secs(s));
    let fanout = analyze_fanout(board);
    let grid = RoutingGrid::build(board, &fanout, threads);

    let mut bga_clusters = Vec::with_capacity(fanout.bga_clusters.len());
    let mut sealed_pads = Vec::new();
    for cluster in &fanout.bga_clusters {
        let dense_pads = cluster
            .pad_ids
            .iter()
            .filter(|&&pid| fanout.pads[pid].is_dense_bga)
            .count();
        bga_clusters.push(BgaClusterSummary {
            refdes: cluster.refdes.clone(),
            total_pads: cluster.pad_ids.len(),
            dense_pads,
            max_ring: cluster.max_ring,
            pitch_um: cluster.pitch_um,
            sealed_count: cluster.sealed_pad_ids.len(),
        });

        for &pid in &cluster.sealed_pad_ids {
            let pad = &board.pads[pid];
            let net_name = pad
                .net_id
                .map_or_else(|| "no-net".to_string(), |id| board.nets[id].name.clone());
            let ring = fanout.pads[pid].bga_ring.unwrap_or(0);
            let blocker = diagnose_sealed_pad(board, pid, cluster);
            sealed_pads.push(SealedPadInfo {
                refdes: pad.refdes.clone(),
                pin: pad.pin_name.clone(),
                net: net_name,
                pos: pad.center,
                ring,
                blocker,
            });
        }
    }

    let routable_nets: Vec<usize> = board
        .nets
        .iter()
        .filter(|n| !n.ignored && n.pad_ids.len() >= 2)
        .map(|n| n.id)
        .collect();

    let (blocked_nets, stopped_early) = check_routable_nets_solo(
        board,
        &grid,
        &fanout,
        &routable_nets,
        threads,
        deadline,
        stop_flag.as_ref(),
    );
    let fixed_violations = check_fixed_copper(board);
    let ok = !stopped_early
        && sealed_pads.is_empty()
        && blocked_nets.is_empty()
        && fixed_violations.is_empty();

    PreflightReport {
        ok,
        stopped_early,
        total_nets: board.nets.len(),
        routable_nets: routable_nets.len(),
        grid_step_um: grid.step,
        via_at_smd: board.via_at_smd,
        bga_clusters,
        sealed_pads,
        blocked_nets,
        fixed_violations,
        elapsed_ms: start.elapsed().as_millis(),
    }
}

impl PreflightReport {
    /// Serializes the preflight report as structured JSON.
    pub fn to_json(&self) -> String {
        let mut out = String::with_capacity(2048);
        let _ = writeln!(out, "{{");
        let _ = writeln!(out, "  \"ok\": {},", self.ok);
        let _ = writeln!(out, "  \"stopped_early\": {},", self.stopped_early);
        let _ = writeln!(out, "  \"total_nets\": {},", self.total_nets);
        let _ = writeln!(out, "  \"routable_nets\": {},", self.routable_nets);
        let _ = writeln!(out, "  \"grid_step_um\": {:.1},", self.grid_step_um);
        let _ = writeln!(out, "  \"via_at_smd\": {},", self.via_at_smd);

        let _ = writeln!(out, "  \"bga_clusters\": [");
        for (i, c) in self.bga_clusters.iter().enumerate() {
            let comma = if i + 1 < self.bga_clusters.len() {
                ","
            } else {
                ""
            };
            let _ = writeln!(
                out,
                "    {{\"refdes\":{},\"total_pads\":{},\"dense_pads\":{},\"max_ring\":{},\"pitch_um\":{:.1},\"sealed_count\":{}}}{comma}",
                json_str(&c.refdes),
                c.total_pads,
                c.dense_pads,
                c.max_ring,
                c.pitch_um,
                c.sealed_count,
            );
        }
        let _ = writeln!(out, "  ],");

        let _ = writeln!(out, "  \"sealed_pads\": [");
        for (i, p) in self.sealed_pads.iter().enumerate() {
            let comma = if i + 1 < self.sealed_pads.len() {
                ","
            } else {
                ""
            };
            let _ = writeln!(
                out,
                "    {{\"refdes\":{},\"pin\":{},\"net\":{},\"pos_um\":[{:.1},{:.1}],\"ring\":{},\"blocker\":{}}}{comma}",
                json_str(&p.refdes),
                json_str(&p.pin),
                json_str(&p.net),
                p.pos.x,
                p.pos.y,
                p.ring,
                json_str(&p.blocker),
            );
        }
        let _ = writeln!(out, "  ],");

        let _ = writeln!(out, "  \"blocked_nets\": [");
        for (i, b) in self.blocked_nets.iter().enumerate() {
            let comma = if i + 1 < self.blocked_nets.len() {
                ","
            } else {
                ""
            };
            let _ = writeln!(
                out,
                "    {{\"net\":{},\"class\":{},\"from_pin\":{},\"from_um\":[{:.1},{:.1}],\"to_pin\":{},\"to_um\":[{:.1},{:.1}],\"reason\":{}}}{comma}",
                json_str(&b.net),
                json_str(&b.class_name),
                json_str(&b.from_pin),
                b.from_pos.x,
                b.from_pos.y,
                json_str(&b.to_pin),
                b.to_pos.x,
                b.to_pos.y,
                json_str(&b.reason),
            );
        }
        let _ = writeln!(out, "  ],");

        let _ = writeln!(out, "  \"fixed_violations\": [");
        for (i, v) in self.fixed_violations.iter().enumerate() {
            let comma = if i + 1 < self.fixed_violations.len() {
                ","
            } else {
                ""
            };
            let net_a = v
                .net_a
                .as_deref()
                .map_or_else(|| "null".to_string(), json_str);
            let net_b = v
                .net_b
                .as_deref()
                .map_or_else(|| "null".to_string(), json_str);
            let layer = v
                .layer
                .as_deref()
                .map_or_else(|| "null".to_string(), json_str);
            let _ = writeln!(
                out,
                "    {{\"kind\":{},\"net_a\":{},\"net_b\":{},\"layer\":{},\"pos_um\":[{:.1},{:.1}],\"gap_um\":{:.2},\"required_um\":{:.2},\"blocker\":{}}}{comma}",
                json_str(v.kind),
                net_a,
                net_b,
                layer,
                v.pos.x,
                v.pos.y,
                v.gap_um,
                v.required_um,
                json_str(&v.blocker),
            );
        }
        let _ = writeln!(out, "  ],");
        let _ = writeln!(out, "  \"elapsed_ms\": {}", self.elapsed_ms);
        let _ = write!(out, "}}");
        out
    }
}

#[inline]
fn is_stopped(deadline: Option<Instant>, stop_flag: Option<&Arc<AtomicBool>>) -> bool {
    deadline.is_some_and(|dl| Instant::now() >= dl) || signal::stop_requested(stop_flag)
}

fn check_routable_nets_solo(
    board: &Board,
    grid: &RoutingGrid,
    fanout: &crate::fanout::FanoutPlan,
    routable_nets: &[usize],
    threads: usize,
    deadline: Option<Instant>,
    stop_flag: Option<&Arc<AtomicBool>>,
) -> (Vec<BlockedNetPair>, bool) {
    if routable_nets.is_empty() {
        return (Vec::new(), is_stopped(deadline, stop_flag));
    }
    if is_stopped(deadline, stop_flag) {
        return (Vec::new(), true);
    }
    let n3d = grid.sig_layers.len() * grid.n2d();
    let worker_count = threads.clamp(1, 8).min(routable_nets.len());
    let next_idx = AtomicUsize::new(0);
    let stopped = AtomicBool::new(false);
    let collected: Mutex<Vec<(usize, usize, BlockedNetPair)>> = Mutex::new(Vec::new());

    std::thread::scope(|s| {
        for _ in 0..worker_count {
            let next_ref = &next_idx;
            let stopped_ref = &stopped;
            let out_ref = &collected;
            s.spawn(move || {
                let mut scratch = SearchScratch::new(n3d);
                let mut local = Vec::new();
                loop {
                    if is_stopped(deadline, stop_flag) {
                        stopped_ref.store(true, Ordering::Relaxed);
                        break;
                    }
                    let idx = next_ref.fetch_add(1, Ordering::Relaxed);
                    if idx >= routable_nets.len() {
                        break;
                    }
                    let nid = routable_nets[idx];
                    let net = &board.nets[nid];
                    let components = build_net_components(board, grid, fanout, nid);
                    if components.len() < 2 {
                        continue;
                    }
                    let mst_edges = build_mst(&components);

                    for (edge_idx, (ca, cb)) in mst_edges.into_iter().enumerate() {
                        if is_stopped(deadline, stop_flag) {
                            stopped_ref.store(true, Ordering::Relaxed);
                            break;
                        }
                        if let Some(r) = solo_pair_block_reason(
                            board,
                            grid,
                            nid,
                            &components[ca],
                            &components[cb],
                            &mut scratch,
                        ) {
                            let (ta, tb) = closest_terminals(&components[ca], &components[cb]);
                            local.push((
                                nid,
                                edge_idx,
                                BlockedNetPair {
                                    net: net.name.clone(),
                                    class_name: net.class_name.clone(),
                                    from_pin: terminal_label(board, ta),
                                    from_pos: ta.exact,
                                    to_pin: terminal_label(board, tb),
                                    to_pos: tb.exact,
                                    reason: r.to_string(),
                                },
                            ));
                        }
                    }
                }
                if !local.is_empty()
                    && let Ok(mut guard) = out_ref.lock()
                {
                    guard.extend(local);
                }
            });
        }
    });

    let mut items = collected.into_inner().unwrap_or_default();
    items.sort_by_key(|(nid, eidx, _)| (*nid, *eidx));
    (
        items.into_iter().map(|(_, _, pair)| pair).collect(),
        stopped.load(Ordering::Relaxed),
    )
}

fn check_fixed_copper(board: &Board) -> Vec<FixedViolation> {
    let mut violations = Vec::new();
    let mut buf = Vec::new();

    for (sid, fs) in board.fixed_segments.iter().enumerate() {
        if fs.is_synthetic {
            continue;
        }
        if let Some(b) = board.diagnose_fixed_segment_static(sid, -0.5, &mut buf) {
            violations.push(FixedViolation {
                kind: "fixed_seg_static",
                net_a: fs.net_id.map(|id| board.nets[id].name.clone()),
                net_b: b.net_b.map(|id| board.nets[id].name.clone()),
                layer: Some(board.layer_names[fs.layer].clone()),
                pos: Point::new((fs.a.x + fs.b.x) * 0.5, (fs.a.y + fs.b.y) * 0.5),
                gap_um: b.gap_um,
                required_um: b.required_um,
                blocker: b.label,
            });
        }
    }

    for (vid, fv) in board.fixed_vias.iter().enumerate() {
        if let Some(b) = board.diagnose_fixed_via_static(vid, -0.5, &mut buf) {
            violations.push(FixedViolation {
                kind: "fixed_via_static",
                net_a: fv.net_id.map(|id| board.nets[id].name.clone()),
                net_b: b.net_b.map(|id| board.nets[id].name.clone()),
                layer: None,
                pos: fv.pos,
                gap_um: b.gap_um,
                required_um: b.required_um,
                blocker: b.label,
            });
        }
    }

    violations
}

fn diagnose_sealed_pad(board: &Board, pid: usize, cluster: &BgaCluster) -> String {
    let pad = &board.pads[pid];
    let net_id = pad.net_id.expect("sealed pad must belong to a net");
    let pad_layer = cluster.pad_layer;
    let rules = &board.nets[net_id].rules;
    let has_other_sig = rules.via_layers.contains(&pad_layer)
        && board
            .signal_layers
            .iter()
            .any(|&bl| bl != pad_layer && rules.via_layers.contains(&bl));
    if !has_other_sig {
        return "no_other_signal_layer".to_string();
    }

    if board.via_at_smd
        && let Some(b) = board.diagnose_via_static(net_id, pad.center, 0.0)
    {
        return format!(
            "pofv blocked by {} (gap {:.1}um < {:.1}um)",
            b.label, b.gap_um, b.required_um
        );
    }

    if let Some(&other_pid) = cluster.pofv_conflicts.get(&pid) {
        let other = &board.pads[other_pid];
        let other_net_id = other.net_id.expect("conflicting POFV pad must have a net");
        let other_rules = &board.nets[other_net_id].rules;
        let d = pad.center.dist(other.center);
        let copper_gap = d - rules.via_radius - other_rules.via_radius;
        let copper_req = board.via_to_via_clearance(net_id, Some(other_net_id));
        let hole_gap = d - (rules.via_drill + other_rules.via_drill) * 0.5;
        let hole_req = board.hole_to_hole_um + 5.0;
        let (gap, req) = if other_net_id != net_id && copper_req - copper_gap >= hole_req - hole_gap
        {
            (copper_gap, copper_req)
        } else {
            (hole_gap, hole_req)
        };
        return format!(
            "pofv blocked by adjacent POFV pad {}-{} ({}) (gap {:.1}um < {:.1}um)",
            other.refdes, other.pin_name, board.nets[other_net_id].name, gap, req,
        );
    }

    let prefix = if board.via_at_smd {
        ""
    } else {
        "via_at_smd disabled; "
    };
    for cand in dogbone_candidates(pad, cluster.centroid, cluster.pitch_um, cluster.basis_theta) {
        if let Some(b) = board.diagnose_via_static(net_id, cand, 0.0) {
            return format!(
                "{prefix}dogbone blocked by {} (gap {:.1}um < {:.1}um)",
                b.label, b.gap_um, b.required_um
            );
        }
        if let Some(b) =
            board.diagnose_segment_static(net_id, pad_layer, pad.center, cand, rules.width, 0.0)
        {
            return format!(
                "{prefix}dogbone stub blocked by {} (gap {:.1}um < {:.1}um)",
                b.label, b.gap_um, b.required_um
            );
        }
    }

    let (sin_t, cos_t) = cluster.basis_theta.sin_cos();
    let pu = pad.center.x * cos_t + pad.center.y * sin_t;
    let env = cluster
        .uv_bounds
        .expand(rules.width + rules.clearance + 1.0);
    let reach = (env.max_x - pu).max(pad.max_radius + rules.width + rules.clearance + 1.0);
    let end = Point::new(pad.center.x + cos_t * reach, pad.center.y + sin_t * reach);
    if let Some(b) =
        board.diagnose_segment_static(net_id, pad_layer, pad.center, end, rules.width, 0.0)
    {
        return format!(
            "{prefix}surface blocked by {} and dogbone slots occupied by adjacent BGA vias",
            b.label
        );
    }

    format!("{prefix}dogbone slots occupied by adjacent BGA escape vias")
}
