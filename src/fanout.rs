//! POFV (via-in-pad) detection and coordinated multi-ring BGA escape planning.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use crate::board::{Board, Pad};
use crate::geometry::{BBox, Point, point_to_segment_dist, segment_to_segment_dist};

/// Fanout metadata computed per physical pad.
#[derive(Debug, Clone)]
pub struct PadFanout {
    pub pad_id: usize,
    /// True when the SMD pad is tightly surrounded (e.g., 0.40/0.50mm BGA).
    pub is_dense_bga: bool,
    /// True when a centered POFV via on this SMD pad clears all other-net obstacles.
    pub pofv_allowed: bool,
    /// Optional non-colliding off-center dogbone via when POFV is off or blocked.
    pub dogbone_via: Option<Point>,
    /// Nearest other-net pad center distance on the same layer (micrometers).
    pub nearest_neighbor_pitch: f64,
    /// Concentric ring depth inside a dense BGA cluster (0 = outer perimeter, 1+ = inner).
    pub bga_ring: Option<usize>,
    /// Preferred escape board layer index balanced across signal layers by ring depth.
    pub preferred_escape_layer: Option<usize>,
}

/// Summary of a dense BGA component cluster, its concentric rings, and sealed pads.
#[derive(Debug, Clone)]
pub struct BgaCluster {
    pub refdes: String,
    pub pad_layer: usize,
    pub pad_ids: Vec<usize>,
    pub pitch_um: f64,
    pub max_ring: usize,
    pub sealed_pad_ids: Vec<usize>,
    pub(crate) centroid: Point,
    pub(crate) basis_theta: f64,
    pub(crate) uv_bounds: BBox,
    pub(crate) pofv_conflicts: BTreeMap<usize, usize>,
}

#[derive(Debug, Clone, Default)]
pub struct FanoutPlan {
    pub pads: Vec<PadFanout>,
    pub dense_bga_count: usize,
    pub pofv_capable_count: usize,
    pub bga_clusters: Vec<BgaCluster>,
}

#[derive(Clone, Copy)]
struct PlacedVia {
    pad_id: usize,
    net_id: usize,
    pos: Point,
    radius: f64,
    drill: f64,
    stub: Option<(usize, Point, Point, f64)>,
}

/// Analyzes every pad to classify dense BGAs, compute ring depths, and plan escapes.
pub fn analyze_fanout(board: &Board) -> FanoutPlan {
    let mut plan = FanoutPlan {
        pads: Vec::with_capacity(board.pads.len()),
        dense_bga_count: 0,
        pofv_capable_count: 0,
        bga_clusters: Vec::new(),
    };
    let mut neighbors = Vec::new();
    let mut sbuf = Vec::new();
    let mut dense_groups: BTreeSet<(String, usize)> = BTreeSet::new();

    for pad in &board.pads {
        let mut info = PadFanout {
            pad_id: pad.id,
            is_dense_bga: false,
            pofv_allowed: false,
            dogbone_via: None,
            nearest_neighbor_pitch: f64::INFINITY,
            bga_ring: None,
            preferred_escape_layer: None,
        };

        let Some(net_id) = pad.net_id else {
            plan.pads.push(info);
            continue;
        };
        let net = &board.nets[net_id];
        if net.ignored || net.pad_ids.len() < 2 || !pad.is_smd {
            plan.pads.push(info);
            continue;
        }

        let Some(&(pad_layer, _)) = pad.layer_shapes.first() else {
            plan.pads.push(info);
            continue;
        };

        let rules = &net.rules;
        let qbox = BBox::from_point(pad.center, 1200.0);
        board.pad_index.query(qbox, &mut neighbors);

        let mut min_center_dist = f64::INFINITY;
        let mut min_copper_gap = f64::INFINITY;
        let mut card_quads = 0u8;
        let mut diag_quads = 0u8;

        for &nid in &neighbors {
            if nid == pad.id {
                continue;
            }
            let other = &board.pads[nid];
            if !other.on_layer(pad_layer) {
                continue;
            }
            let d = pad.center.dist(other.center);
            if d < 1e-3 {
                continue;
            }
            if other.net_id != Some(net_id) {
                min_center_dist = min_center_dist.min(d);
                if let Some(osh) = other.shape_on_layer(pad_layer) {
                    let gap = osh.dist_to_circle(pad.center, pad.max_radius);
                    min_copper_gap = min_copper_gap.min(gap);
                }
            }
            if d <= 900.0 && other.refdes == pad.refdes {
                let dx = other.center.x - pad.center.x;
                let dy = other.center.y - pad.center.y;
                if dx > 50.0 && dx.abs() >= dy.abs() {
                    card_quads |= 1;
                } else if dx < -50.0 && dx.abs() >= dy.abs() {
                    card_quads |= 2;
                } else if dy > 50.0 && dy.abs() >= dx.abs() {
                    card_quads |= 4;
                } else if dy < -50.0 && dy.abs() >= dx.abs() {
                    card_quads |= 8;
                }
                if dx > 50.0 && dy > 50.0 {
                    diag_quads |= 1;
                } else if dx < -50.0 && dy > 50.0 {
                    diag_quads |= 2;
                } else if dx < -50.0 && dy < -50.0 {
                    diag_quads |= 4;
                } else if dx > 50.0 && dy < -50.0 {
                    diag_quads |= 8;
                }
            }
        }

        info.nearest_neighbor_pitch = min_center_dist;
        let trace_channel_needed = rules.width + 2.0 * rules.clearance;
        let cannot_pass_trace = min_copper_gap < trace_channel_needed;
        let surrounded = card_quads.count_ones().max(diag_quads.count_ones()) >= 3;
        info.is_dense_bga = (surrounded && min_center_dist <= 850.0)
            || (min_center_dist <= 550.0 && cannot_pass_trace);

        if info.is_dense_bga {
            plan.dense_bga_count += 1;
            dense_groups.insert((pad.refdes.clone(), pad_layer));
        }

        let has_other_sig = rules.via_layers.contains(&pad_layer)
            && board
                .signal_layers
                .iter()
                .any(|&bl| bl != pad_layer && rules.via_layers.contains(&bl));
        if board.via_at_smd
            && has_other_sig
            && board.check_via_static_with_buf(net_id, pad.center, 0.0, &mut sbuf)
        {
            info.pofv_allowed = true;
            plan.pofv_capable_count += 1;
        }

        plan.pads.push(info);
    }

    // Group all SMD pads by (refdes, pad_layer) for components with dense pads.
    let mut comp_pads: BTreeMap<(String, usize), Vec<usize>> = BTreeMap::new();
    for pad in &board.pads {
        if pad.is_smd
            && let Some(&(pad_layer, _)) = pad.layer_shapes.first()
            && dense_groups.contains(&(pad.refdes.clone(), pad_layer))
        {
            comp_pads
                .entry((pad.refdes.clone(), pad_layer))
                .or_default()
                .push(pad.id);
        }
    }

    for ((refdes, pad_layer), pad_ids) in comp_pads {
        if let Some(cluster) = plan_bga_cluster(
            board,
            &refdes,
            pad_layer,
            pad_ids,
            &mut plan.pads,
            &mut plan.pofv_capable_count,
            &mut sbuf,
        ) {
            plan.bga_clusters.push(cluster);
        }
    }

    // Retain per-pad dogbone fallback for isolated non-BGA dense pads outside clusters.
    let mut fallback_vias: Vec<PlacedVia> = Vec::new();
    for pad in &board.pads {
        let pf = &plan.pads[pad.id];
        if !pf.is_dense_bga || pf.bga_ring.is_some() || pf.pofv_allowed || pf.dogbone_via.is_some()
        {
            continue;
        }
        let Some(net_id) = pad.net_id else {
            continue;
        };
        let Some(&(pad_layer, _)) = pad.layer_shapes.first() else {
            continue;
        };
        let rules = &board.nets[net_id].rules;
        let has_other_sig = rules.via_layers.contains(&pad_layer)
            && board
                .signal_layers
                .iter()
                .any(|&bl| bl != pad_layer && rules.via_layers.contains(&bl));
        if !has_other_sig || pad_has_fixed_escape(board, pad, net_id, pad_layer) {
            continue;
        }
        if let Some(cand) = find_non_colliding_dogbone(
            board,
            pad,
            net_id,
            pad_layer,
            pad.center,
            pf.nearest_neighbor_pitch,
            0.0,
            &fallback_vias,
            &mut sbuf,
        ) {
            plan.pads[pad.id].dogbone_via = Some(cand);
            fallback_vias.push(PlacedVia {
                pad_id: pad.id,
                net_id,
                pos: cand,
                radius: rules.via_radius,
                drill: rules.via_drill,
                stub: Some((pad_layer, pad.center, cand, rules.width)),
            });
        }
    }

    plan
}

/// Returns candidate dogbone via positions ordered outward in the BGA's `(u, v)` basis.
pub(crate) fn dogbone_candidates(
    pad: &Pad,
    centroid: Point,
    pitch: f64,
    basis_theta: f64,
) -> [Point; 8] {
    let offset = if pitch.is_finite() && pitch > 100.0 {
        pitch * 0.5
    } else {
        350.0
    };
    let (sin_t, cos_t) = basis_theta.sin_cos();
    let vx = pad.center.x - centroid.x;
    let vy = pad.center.y - centroid.y;
    let vu = vx * cos_t + vy * sin_t;
    let vv = -vx * sin_t + vy * cos_t;
    let su = if vu >= 0.0 { 1.0 } else { -1.0 };
    let sv = if vv >= 0.0 { 1.0 } else { -1.0 };
    let (d2, d3) = if vu.abs() >= vv.abs() {
        ((su, -sv), (-su, sv))
    } else {
        ((-su, sv), (su, -sv))
    };
    let dirs = [
        (su, sv),
        d2,
        d3,
        (-su, -sv),
        (su, 0.0),
        (0.0, sv),
        (-su, 0.0),
        (0.0, -sv),
    ];
    dirs.map(|(du, dv)| {
        let ox = (du * cos_t - dv * sin_t) * offset;
        let oy = (du * sin_t + dv * cos_t) * offset;
        Point::new(pad.center.x + ox, pad.center.y + oy)
    })
}

/// Checks whether a BGA pad can legally escape strictly outside the cluster envelope on surface.
#[allow(clippy::too_many_arguments)]
pub(crate) fn has_clear_surface_exit(
    board: &Board,
    pad: &Pad,
    net_id: usize,
    pad_layer: usize,
    pitch: f64,
    basis_theta: f64,
    uv_bounds: BBox,
    sbuf: &mut Vec<usize>,
) -> bool {
    let rules = &board.nets[net_id].rules;
    let (sin_t, cos_t) = basis_theta.sin_cos();
    let pu = pad.center.x * cos_t + pad.center.y * sin_t;
    let pv = -pad.center.x * sin_t + pad.center.y * cos_t;
    let env = uv_bounds.expand(rules.width + rules.clearance + 1.0);
    let min_reach = pad.max_radius + rules.width + rules.clearance + 1.0;

    let uv_offset = |du: f64, dv: f64| -> Point {
        Point::new(
            pad.center.x + du * cos_t - dv * sin_t,
            pad.center.y + du * sin_t + dv * cos_t,
        )
    };

    let inv_s2 = std::f64::consts::FRAC_1_SQRT_2;
    let rays = [
        (1.0, 0.0),
        (-1.0, 0.0),
        (0.0, 1.0),
        (0.0, -1.0),
        (inv_s2, inv_s2),
        (-inv_s2, inv_s2),
        (inv_s2, -inv_s2),
        (-inv_s2, -inv_s2),
    ];
    for (du, dv) in rays {
        let t = ray_exit_dist(pu, pv, du, dv, env).max(min_reach);
        let end = uv_offset(du * t, dv * t);
        if board.check_segment_static_with_buf(
            net_id,
            pad_layer,
            pad.center,
            end,
            rules.width,
            0.0,
            sbuf,
        ) {
            return true;
        }
    }

    // Check 45-degree quad-center dogleg + cardinal continuation strictly outside `env`.
    if pitch.is_finite() && pitch > 100.0 {
        let half = pitch * 0.5;
        for (su, sv) in [(1.0, 1.0), (-1.0, 1.0), (1.0, -1.0), (-1.0, -1.0)] {
            let mu = pu + su * half;
            let mv = pv + sv * half;
            let mid = uv_offset(su * half, sv * half);
            if !board.check_segment_static_with_buf(
                net_id,
                pad_layer,
                pad.center,
                mid,
                rules.width,
                0.0,
                sbuf,
            ) {
                continue;
            }
            if mu <= env.min_x || mu >= env.max_x || mv <= env.min_y || mv >= env.max_y {
                return true;
            }
            let tu = ray_exit_dist(mu, mv, su, 0.0, env).max(pitch);
            let tv = ray_exit_dist(mu, mv, 0.0, sv, env).max(pitch);
            let end_u = Point::new(mid.x + su * tu * cos_t, mid.y + su * tu * sin_t);
            let end_v = Point::new(mid.x - sv * tv * sin_t, mid.y + sv * tv * cos_t);
            if board.check_segment_static_with_buf(
                net_id,
                pad_layer,
                mid,
                end_u,
                rules.width,
                0.0,
                sbuf,
            ) || board.check_segment_static_with_buf(
                net_id,
                pad_layer,
                mid,
                end_v,
                rules.width,
                0.0,
                sbuf,
            ) {
                return true;
            }
        }
    }
    false
}

fn ray_exit_dist(u: f64, v: f64, du: f64, dv: f64, env: BBox) -> f64 {
    let tu = if du > 1e-9 {
        (env.max_x - u) / du
    } else if du < -1e-9 {
        (env.min_x - u) / du
    } else {
        f64::INFINITY
    };
    let tv = if dv > 1e-9 {
        (env.max_y - v) / dv
    } else if dv < -1e-9 {
        (env.min_y - v) / dv
    } else {
        f64::INFINITY
    };
    tu.min(tv).max(0.0)
}

#[allow(clippy::too_many_arguments)]
fn plan_bga_cluster(
    board: &Board,
    refdes: &str,
    pad_layer: usize,
    pad_ids: Vec<usize>,
    pad_fanouts: &mut [PadFanout],
    pofv_capable_count: &mut usize,
    sbuf: &mut Vec<usize>,
) -> Option<BgaCluster> {
    if pad_ids.len() < 9 {
        return None;
    }
    let n = pad_ids.len();

    // Compute median nearest-neighbor pitch across dense pads in the component.
    let mut nn_dists = Vec::new();
    for (i, &pid) in pad_ids.iter().enumerate() {
        if !pad_fanouts[pid].is_dense_bga {
            continue;
        }
        let c = board.pads[pid].center;
        let mut best = f64::INFINITY;
        for (j, &qid) in pad_ids.iter().enumerate() {
            if i == j {
                continue;
            }
            let d = c.dist(board.pads[qid].center);
            if d > 10.0 && d < best {
                best = d;
            }
        }
        if best.is_finite() {
            nn_dists.push(best);
        }
    }
    if nn_dists.is_empty() {
        return None;
    }
    nn_dists.sort_by(|a, b| a.total_cmp(b));
    let pitch_um = nn_dists[nn_dists.len() / 2];

    // Detect dominant nearest-neighbor orientation (mod 90 deg) via 4-fold circular mean.
    let mut sum_c4 = 0.0;
    let mut sum_s4 = 0.0;
    let nn_ang_thresh = pitch_um * 1.25;
    for (i, &pid) in pad_ids.iter().enumerate() {
        let ci = board.pads[pid].center;
        for &qid in &pad_ids[(i + 1)..] {
            let cj = board.pads[qid].center;
            let d = ci.dist(cj);
            if d > 10.0 && d <= nn_ang_thresh {
                let ang4 = 4.0 * (cj.y - ci.y).atan2(cj.x - ci.x);
                sum_c4 += ang4.cos();
                sum_s4 += ang4.sin();
            }
        }
    }
    let half_pi = std::f64::consts::FRAC_PI_2;
    let mut basis_theta = if sum_c4.hypot(sum_s4) > 1e-6 {
        (0.25 * sum_s4.atan2(sum_c4)).rem_euclid(half_pi)
    } else {
        0.0
    };
    if basis_theta < 1e-3 || (half_pi - basis_theta) < 1e-3 {
        basis_theta = 0.0;
    }
    let (sin_t, cos_t) = basis_theta.sin_cos();

    let mut uv = Vec::with_capacity(n);
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    let mut uv_min_x = f64::INFINITY;
    let mut uv_min_y = f64::INFINITY;
    let mut uv_max_x = f64::NEG_INFINITY;
    let mut uv_max_y = f64::NEG_INFINITY;
    for &pid in &pad_ids {
        let p = &board.pads[pid];
        let u = p.center.x * cos_t + p.center.y * sin_t;
        let v = -p.center.x * sin_t + p.center.y * cos_t;
        uv.push((u, v));
        sum_x += p.center.x;
        sum_y += p.center.y;
        uv_min_x = uv_min_x.min(u - p.max_radius);
        uv_max_x = uv_max_x.max(u + p.max_radius);
        uv_min_y = uv_min_y.min(v - p.max_radius);
        uv_max_y = uv_max_y.max(v + p.max_radius);
    }
    let centroid = Point::new(sum_x / n as f64, sum_y / n as f64);
    let uv_bounds = BBox::new(uv_min_x, uv_min_y, uv_max_x, uv_max_y);

    // Build adjacency and cardinal line-of-sight masks in the rotated (u, v) basis.
    let pitch_thresh = (pitch_um * 1.30).max(pitch_um + 50.0).min(1150.0);
    let corridor = (pitch_um * 0.45).max(10.0);
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
    let mut blocked_axes: Vec<u8> = vec![0; n];
    for i in 0..n {
        let (ui, vi) = uv[i];
        for j in (i + 1)..n {
            let (uj, vj) = uv[j];
            let du = uj - ui;
            let dv = vj - vi;
            let d = du.hypot(dv);
            if d <= 10.0 {
                continue;
            }
            if du > corridor && dv.abs() <= corridor {
                blocked_axes[i] |= 1;
                blocked_axes[j] |= 2;
            } else if du < -corridor && dv.abs() <= corridor {
                blocked_axes[i] |= 2;
                blocked_axes[j] |= 1;
            }
            if dv > corridor && du.abs() <= corridor {
                blocked_axes[i] |= 4;
                blocked_axes[j] |= 8;
            } else if dv < -corridor && du.abs() <= corridor {
                blocked_axes[i] |= 8;
                blocked_axes[j] |= 4;
            }
            if d <= pitch_thresh {
                adj[i].push(j);
                adj[j].push(i);
            }
        }
    }

    let mut ring: Vec<Option<usize>> = vec![None; n];
    let mut queue = VecDeque::new();
    for i in 0..n {
        if blocked_axes[i].count_ones() < 4 {
            ring[i] = Some(0);
            queue.push_back(i);
        }
    }
    while let Some(u) = queue.pop_front() {
        let nr = ring[u].unwrap() + 1;
        for &v in &adj[u] {
            if ring[v].is_none() {
                ring[v] = Some(nr);
                queue.push_back(v);
            }
        }
    }

    let mut rings = vec![0usize; n];
    let mut max_ring = 0usize;
    for i in 0..n {
        let r = ring[i].unwrap_or_else(|| {
            let min_d = (0..n)
                .filter(|&j| ring[j] == Some(0))
                .map(|j| (uv[i].0 - uv[j].0).hypot(uv[i].1 - uv[j].1))
                .fold(f64::INFINITY, f64::min);
            if min_d.is_finite() {
                (min_d / pitch_um).round().max(1.0) as usize
            } else {
                1
            }
        });
        rings[i] = r;
        max_ring = max_ring.max(r);
    }
    if max_ring < 1 {
        return None;
    }

    for (i, &pid) in pad_ids.iter().enumerate() {
        pad_fanouts[pid].bga_ring = Some(rings[i]);
    }

    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&a, &b| {
        rings[b]
            .cmp(&rings[a])
            .then_with(|| pad_ids[a].cmp(&pad_ids[b]))
    });

    let non_pad_sig: Vec<usize> = board
        .signal_layers
        .iter()
        .copied()
        .filter(|&bl| bl != pad_layer)
        .collect();

    // Pass 1: evaluate surface/fixed escapes, assign reachable layers, reserve valid POFV vias.
    let mut surface_exit = vec![false; n];
    let mut fixed_escape = vec![false; n];
    let mut placed_vias: Vec<PlacedVia> = Vec::new();
    let mut pofv_conflicts: BTreeMap<usize, usize> = BTreeMap::new();

    for &i in &order {
        let pid = pad_ids[i];
        let pad = &board.pads[pid];
        let Some(net_id) = pad.net_id else {
            continue;
        };
        let net = &board.nets[net_id];
        if net.ignored || net.pad_ids.len() < 2 {
            continue;
        }
        let r = rings[i];
        let rules = &net.rules;
        let clear_surf = has_clear_surface_exit(
            board,
            pad,
            net_id,
            pad_layer,
            pitch_um,
            basis_theta,
            uv_bounds,
            sbuf,
        );
        let has_fix = pad_has_fixed_escape(board, pad, net_id, pad_layer);
        surface_exit[i] = clear_surf;
        fixed_escape[i] = has_fix;

        let reachable_sig: Vec<usize> = if rules.via_layers.contains(&pad_layer) {
            non_pad_sig
                .iter()
                .copied()
                .filter(|bl| rules.via_layers.contains(bl))
                .collect()
        } else {
            Vec::new()
        };
        if r == 0 && clear_surf {
            pad_fanouts[pid].preferred_escape_layer = Some(pad_layer);
        } else if !reachable_sig.is_empty() {
            let idx = r.saturating_sub(1) % reachable_sig.len();
            pad_fanouts[pid].preferred_escape_layer = Some(reachable_sig[idx]);
        }

        if pad_fanouts[pid].pofv_allowed && (r >= 1 || !clear_surf) {
            if let Some(conflict_pid) =
                find_via_conflict(board, net_id, pad.center, None, &placed_vias)
            {
                pad_fanouts[pid].pofv_allowed = false;
                *pofv_capable_count = pofv_capable_count.saturating_sub(1);
                pofv_conflicts.insert(pid, conflict_pid);
            } else {
                placed_vias.push(PlacedVia {
                    pad_id: pid,
                    net_id,
                    pos: pad.center,
                    radius: rules.via_radius,
                    drill: rules.via_drill,
                    stub: None,
                });
            }
        }
    }

    // Pass 2: assign non-colliding dogbone vias from innermost ring outward and record sealed pads.
    let mut sealed_pad_ids = Vec::new();
    for &i in &order {
        let pid = pad_ids[i];
        let pad = &board.pads[pid];
        let Some(net_id) = pad.net_id else {
            continue;
        };
        let net = &board.nets[net_id];
        if net.ignored || net.pad_ids.len() < 2 {
            continue;
        }
        let r = rings[i];
        let rules = &net.rules;
        let has_other_sig = rules.via_layers.contains(&pad_layer)
            && non_pad_sig.iter().any(|bl| rules.via_layers.contains(bl));
        let needs_dogbone = has_other_sig
            && !pad_fanouts[pid].pofv_allowed
            && !fixed_escape[i]
            && (r > 0 || !surface_exit[i]);
        if needs_dogbone
            && let Some(cand) = find_non_colliding_dogbone(
                board,
                pad,
                net_id,
                pad_layer,
                centroid,
                pitch_um,
                basis_theta,
                &placed_vias,
                sbuf,
            )
        {
            pad_fanouts[pid].dogbone_via = Some(cand);
            placed_vias.push(PlacedVia {
                pad_id: pid,
                net_id,
                pos: cand,
                radius: rules.via_radius,
                drill: rules.via_drill,
                stub: Some((pad_layer, pad.center, cand, rules.width)),
            });
        }
        let pf = &pad_fanouts[pid];
        if !surface_exit[i] && !fixed_escape[i] && !pf.pofv_allowed && pf.dogbone_via.is_none() {
            sealed_pad_ids.push(pid);
        }
    }
    sealed_pad_ids.sort_unstable();

    Some(BgaCluster {
        refdes: refdes.to_string(),
        pad_layer,
        pad_ids,
        pitch_um,
        max_ring,
        sealed_pad_ids,
        centroid,
        basis_theta,
        uv_bounds,
        pofv_conflicts,
    })
}

fn pad_has_fixed_escape(board: &Board, pad: &Pad, net_id: usize, pad_layer: usize) -> bool {
    let Some(sh) = pad.shape_on_layer(pad_layer) else {
        return false;
    };
    let has_via = board.fixed_vias.iter().any(|fv| {
        fv.net_id == Some(net_id)
            && fv
                .layers
                .iter()
                .any(|&l| l != pad_layer && board.signal_layers.contains(&l))
            && sh.dist_to_circle(fv.pos, fv.radius) <= 2.0
    });
    if has_via {
        return true;
    }
    board.fixed_segments.iter().any(|fs| {
        fs.net_id == Some(net_id)
            && fs.layer == pad_layer
            && sh.dist_to_capsule(fs.a, fs.b, fs.half_w) <= 2.0
    })
}

fn find_via_conflict(
    board: &Board,
    net_id: usize,
    pos: Point,
    stub: Option<(usize, Point, Point, f64)>,
    placed_vias: &[PlacedVia],
) -> Option<usize> {
    let rules = &board.nets[net_id].rules;
    for pv in placed_vias {
        let pv_layers = &board.nets[pv.net_id].rules.via_layers;
        let layers_overlap = rules.via_layers.iter().any(|l| pv_layers.contains(l));
        if layers_overlap {
            let d = pos.dist(pv.pos);
            let min_hole = (rules.via_drill + pv.drill) * 0.5 + board.hole_to_hole_um + 5.0;
            if pv.net_id == net_id {
                if d >= 5.0 && d < min_hole - 1e-4 {
                    return Some(pv.pad_id);
                }
            } else {
                let vv_clr = board.via_to_via_clearance(net_id, Some(pv.net_id));
                if d - rules.via_radius - pv.radius < vv_clr - 1e-4 || d < min_hole - 1e-4 {
                    return Some(pv.pad_id);
                }
            }
        }

        if pv.net_id == net_id {
            continue;
        }
        if let Some((slayer, sa, sb, sw)) = pv.stub
            && rules.via_layers.contains(&slayer)
        {
            let vw_clr = board.via_to_wire_clearance(net_id, Some(pv.net_id));
            let gap = point_to_segment_dist(pos, sa, sb) - rules.via_radius - sw * 0.5;
            if gap < vw_clr - 1e-4 {
                return Some(pv.pad_id);
            }
        }
        if let Some((layer, a, b, w)) = stub {
            let half_w = w * 0.5;
            if pv_layers.contains(&layer) {
                let vw_clr = board.via_to_wire_clearance(net_id, Some(pv.net_id));
                let stub_gap = point_to_segment_dist(pv.pos, a, b) - pv.radius - half_w;
                if stub_gap < vw_clr - 1e-4 {
                    return Some(pv.pad_id);
                }
            }
            if let Some((slayer, sa, sb, sw)) = pv.stub
                && slayer == layer
            {
                let ww_clr = board.wire_to_other_clearance(net_id, Some(pv.net_id));
                let seg_gap = segment_to_segment_dist(a, b, sa, sb) - half_w - sw * 0.5;
                if seg_gap < ww_clr - 1e-4 {
                    return Some(pv.pad_id);
                }
            }
        }
    }
    None
}

#[allow(clippy::too_many_arguments)]
fn find_non_colliding_dogbone(
    board: &Board,
    pad: &Pad,
    net_id: usize,
    layer: usize,
    centroid: Point,
    pitch: f64,
    basis_theta: f64,
    placed_vias: &[PlacedVia],
    sbuf: &mut Vec<usize>,
) -> Option<Point> {
    let rules = &board.nets[net_id].rules;
    for cand in dogbone_candidates(pad, centroid, pitch, basis_theta) {
        if !board.check_via_static_with_buf(net_id, cand, 0.0, sbuf) {
            continue;
        }
        if !board.check_segment_static_with_buf(
            net_id,
            layer,
            pad.center,
            cand,
            rules.width,
            0.0,
            sbuf,
        ) {
            continue;
        }
        if find_via_conflict(
            board,
            net_id,
            cand,
            Some((layer, pad.center, cand, rules.width)),
            placed_vias,
        )
        .is_none()
        {
            return Some(cand);
        }
    }
    None
}
