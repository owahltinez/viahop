//! Multi-layer octilinear weighted A* search with stale-entry pruning and via clearance checks.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::geometry::Point;
use crate::grid::RoutingGrid;

const SQRT2: f32 = std::f32::consts::SQRT_2;

pub(crate) struct PassParams {
    pub(crate) present_factor: f32,
    pub(crate) h_weight: f32,
    pub(crate) strict_mode: bool,
    pub(crate) max_expansions: usize,
}

/// Reusable A* scratch buffers reset in O(1) via generation counters.
pub(crate) struct SearchScratch {
    g_score: Vec<f32>,
    parent: Vec<u32>,
    seen_gen: Vec<u32>,
    target_gen: Vec<u32>,
    search_gen: u32,
    tgen: u32,
    heap: BinaryHeap<HeapNode>,
}

impl SearchScratch {
    pub(crate) fn new(n3d: usize) -> Self {
        Self {
            g_score: vec![f32::INFINITY; n3d],
            parent: vec![u32::MAX; n3d],
            seen_gen: vec![0; n3d],
            target_gen: vec![0; n3d],
            search_gen: 0,
            tgen: 0,
            heap: BinaryHeap::with_capacity(4096),
        }
    }

    fn next_search(&mut self) {
        self.search_gen = self.search_gen.wrapping_add(1);
        if self.search_gen == 0 {
            self.seen_gen.fill(0);
            self.search_gen = 1;
        }
        self.tgen = self.tgen.wrapping_add(1);
        if self.tgen == 0 {
            self.target_gen.fill(0);
            self.tgen = 1;
        }
        self.heap.clear();
    }
}

#[derive(Copy, Clone)]
struct HeapNode {
    f: f32,
    g: f32,
    node: u32,
}

impl PartialEq for HeapNode {
    fn eq(&self, other: &Self) -> bool {
        self.f.to_bits() == other.f.to_bits() && self.node == other.node
    }
}
impl Eq for HeapNode {}
impl PartialOrd for HeapNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for HeapNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f
            .total_cmp(&self.f)
            .then_with(|| other.node.cmp(&self.node))
    }
}

#[inline]
pub(crate) fn unpack_node(node: u32, n2d: usize, cols: usize) -> (usize, usize, usize) {
    let idx = node as usize;
    let sl = idx / n2d;
    let rem = idx % n2d;
    (sl, rem % cols, rem / cols)
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn astar_connect(
    grid: &RoutingGrid,
    net_id: usize,
    src_anchors: &[(u32, Point, Option<Point>, usize)],
    src_extra: &[(usize, usize, usize)],
    dst_anchors: &[(u32, Point, Option<Point>, usize)],
    dst_extra: &[(usize, usize, usize)],
    existing_vias: &[Point],
    min_via_dist: f64,
    params: &PassParams,
    scratch: &mut SearchScratch,
) -> Option<Vec<u32>> {
    scratch.next_search();
    let search_gen = scratch.search_gen;
    let tgen = scratch.tgen;
    let n2d = grid.n2d();
    let cols = grid.cols;
    let rows = grid.rows;
    let num_sig = grid.sig_layers.len();
    let min_via_sq = min_via_dist * min_via_dist;

    let mut target_pts: Vec<(usize, usize)> = Vec::new();
    let mut has_target = false;

    let mut mark_target = |node: u32, record_pt: bool| {
        scratch.target_gen[node as usize] = tgen;
        if record_pt {
            let (_, gx, gy) = unpack_node(node, n2d, cols);
            target_pts.push((gx, gy));
        }
        has_target = true;
    };

    for &(node, _, _, _) in dst_anchors {
        mark_target(node, true);
    }
    let dst_sample_step = (dst_extra.len() / 16).max(1);
    for (i, &(sl, gx, gy)) in dst_extra.iter().enumerate() {
        mark_target((sl * n2d + gy * cols + gx) as u32, i % dst_sample_step == 0);
    }
    if !has_target {
        return None;
    }
    target_pts.sort_unstable();
    target_pts.dedup();
    // Cap heuristic anchor count so evaluation stays O(1) even on huge nets.
    if target_pts.len() > 24 {
        let stride = target_pts.len().div_ceil(24);
        target_pts = target_pts.into_iter().step_by(stride).collect();
    }

    let h_weight = params.h_weight;
    let heuristic = |gx: usize, gy: usize| -> f32 {
        let mut best = f32::INFINITY;
        for &(tx, ty) in &target_pts {
            let dx = gx.abs_diff(tx) as f32;
            let dy = gy.abs_diff(ty) as f32;
            let (mn, mx) = if dx < dy { (dx, dy) } else { (dy, dx) };
            let d = mx + (SQRT2 - 1.0) * mn;
            if d < best {
                best = d;
            }
        }
        best * h_weight
    };

    let mut seed = |node: u32, init_cost: f32| {
        let idx = node as usize;
        if scratch.seen_gen[idx] == search_gen && scratch.g_score[idx] <= init_cost {
            return;
        }
        scratch.seen_gen[idx] = search_gen;
        scratch.g_score[idx] = init_cost;
        scratch.parent[idx] = u32::MAX;
        let (_, gx, gy) = unpack_node(node, n2d, cols);
        scratch.heap.push(HeapNode {
            f: init_cost + heuristic(gx, gy),
            g: init_cost,
            node,
        });
    };

    for &(node, _, via_opt, _) in src_anchors {
        let init_c = if via_opt.is_some() { 4.5 } else { 0.0 };
        seed(node, init_c);
    }
    let step_skip = (src_extra.len() / 64).max(1);
    for &(sl, gx, gy) in src_extra.iter().step_by(step_skip) {
        seed((sl * n2d + gy * cols + gx) as u32, 0.0);
    }

    const DIRS: [(isize, isize, f32, bool, bool); 8] = [
        (1, 0, 1.0, true, false),
        (-1, 0, 1.0, true, false),
        (0, 1, 1.0, false, true),
        (0, -1, 1.0, false, true),
        (1, 1, SQRT2, false, false),
        (1, -1, SQRT2, false, false),
        (-1, 1, SQRT2, false, false),
        (-1, -1, SQRT2, false, false),
    ];

    let max_expansions = params.max_expansions;
    let mut expansions = 0usize;
    let via_base_cost = 6.5_f32;
    let cols_i = cols as isize;
    let rows_i = rows as isize;

    while let Some(HeapNode { g, node, .. }) = scratch.heap.pop() {
        let idx = node as usize;
        // Skip stale heap entries superseded by a cheaper path to the same node.
        if g > scratch.g_score[idx] {
            continue;
        }
        if scratch.target_gen[idx] == tgen {
            return Some(reconstruct_path(node, &scratch.parent));
        }
        expansions += 1;
        if expansions > max_expansions {
            return None;
        }
        let (sl, gx, gy) = unpack_node(node, n2d, cols);
        let prefer_h = sl % 2 == 0;
        let sl_base = sl * n2d;
        let c2 = gy * cols + gx;

        for &(dx, dy, dist, is_h, is_v) in &DIRS {
            let nx = gx as isize + dx;
            let ny = gy as isize + dy;
            if nx < 0 || ny < 0 || nx >= cols_i || ny >= rows_i {
                continue;
            }
            let nxu = nx as usize;
            let nyu = ny as usize;
            let nc2 = nyu * cols + nxu;
            let nidx = sl_base + nc2;
            let is_target = scratch.target_gen[nidx] == tgen;

            if !is_target && !grid.wire_allowed(sl, nc2, net_id) {
                continue;
            }

            let occ = grid.wire_occ[nidx];
            if params.strict_mode && occ > 0 && !is_target {
                continue;
            }

            let dir_bias = if (prefer_h && is_v) || (!prefer_h && is_h) {
                1.12
            } else {
                1.0
            };
            let hist = grid.wire_hist[nidx];
            let cong_mult = (1.0 + occ as f32 * params.present_factor) * (1.0 + hist);
            let ng = g + dist * dir_bias * cong_mult;

            if scratch.seen_gen[nidx] != search_gen || ng < scratch.g_score[nidx] {
                scratch.seen_gen[nidx] = search_gen;
                scratch.g_score[nidx] = ng;
                scratch.parent[nidx] = node;
                scratch.heap.push(HeapNode {
                    f: ng + heuristic(nxu, nyu),
                    g: ng,
                    node: nidx as u32,
                });
            }
        }

        if num_sig > 1 && grid.via_allowed(c2, net_id) {
            let vocc = grid.via_occ[c2];
            if params.strict_mode && vocc > 0 {
                continue;
            }
            let skip_start_pofv = scratch.parent[idx] == u32::MAX
                && src_anchors.iter().any(|&(n, _, vo, _)| n == node && vo.is_some());
            if skip_start_pofv {
                continue;
            }
            let p_via = grid.pt_of(gx, gy);
            let via_hole_ok = !existing_vias.iter().any(|&ev| {
                let d2 = ev.dist_sq(p_via);
                d2 >= 25.0 && d2 < min_via_sq
            }) && !recent_path_via_too_close(
                node,
                &scratch.parent,
                grid,
                n2d,
                cols,
                p_via,
                min_via_sq,
            );
            if via_hole_ok {
                let vhist = grid.via_hist[c2];
                let vcost =
                    via_base_cost * (1.0 + vocc as f32 * params.present_factor) * (1.0 + vhist);
                let h_here = heuristic(gx, gy);
                for nsl in 0..num_sig {
                    if nsl == sl {
                        continue;
                    }
                    let nidx = nsl * n2d + c2;
                    let is_target = scratch.target_gen[nidx] == tgen;
                    if !is_target && !grid.wire_allowed(nsl, c2, net_id) {
                        continue;
                    }
                    let wocc = grid.wire_occ[nidx];
                    if params.strict_mode && wocc > 0 && !is_target {
                        continue;
                    }
                    let ng = g + vcost + wocc as f32 * params.present_factor;
                    if scratch.seen_gen[nidx] != search_gen || ng < scratch.g_score[nidx] {
                        scratch.seen_gen[nidx] = search_gen;
                        scratch.g_score[nidx] = ng;
                        scratch.parent[nidx] = node;
                        scratch.heap.push(HeapNode {
                            f: ng + h_here,
                            g: ng,
                            node: nidx as u32,
                        });
                    }
                }
            }
        }
    }

    None
}

fn recent_path_via_too_close(
    start_node: u32,
    parent: &[u32],
    grid: &RoutingGrid,
    n2d: usize,
    cols: usize,
    p_via: Point,
    min_via_sq: f64,
) -> bool {
    let mut cur = start_node;
    for _ in 0..10 {
        let prev = parent[cur as usize];
        if prev == u32::MAX {
            break;
        }
        if (cur as usize) / n2d != (prev as usize) / n2d {
            let rem = (cur as usize) % n2d;
            let d2 = grid.pt_of(rem % cols, rem / cols).dist_sq(p_via);
            if d2 >= 25.0 && d2 < min_via_sq {
                return true;
            }
        }
        cur = prev;
    }
    false
}

fn reconstruct_path(mut cur: u32, parent: &[u32]) -> Vec<u32> {
    let mut path = Vec::new();
    while cur != u32::MAX {
        path.push(cur);
        cur = parent[cur as usize];
    }
    path.reverse();
    path
}
