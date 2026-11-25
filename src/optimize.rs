//! Post-route geometry optimizer: stub pruning, collinear merging, multi-segment
//! octilinear pull-tight shortcutting, and via-pair elimination with continuous DRC gating.

use std::collections::{HashMap, HashSet};

use crate::board::{Board, NetRoute, RoutedSegment, RoutedVia, SpatialGrid};
use crate::geometry::{point_to_segment_dist, segment_to_segment_dist, BBox, Point};

/// Spatial index over all routed segments and vias for fast dynamic clearance checks.
struct RouteSpatialIndex {
    segs: Vec<Option<(usize, RoutedSegment)>>,
    vias: Vec<Option<(usize, RoutedVia)>>,
    seg_grid: SpatialGrid,
    via_grid: SpatialGrid,
    net_seg_slots: Vec<Vec<usize>>,
    net_via_slots: Vec<Vec<usize>>,
}

impl RouteSpatialIndex {
    fn build(board: &Board, routes: &[NetRoute]) -> Self {
        let mut idx = Self {
            segs: Vec::new(),
            vias: Vec::new(),
            seg_grid: SpatialGrid::new(board.bounds, 1000.0),
            via_grid: SpatialGrid::new(board.bounds, 1000.0),
            net_seg_slots: vec![Vec::new(); routes.len()],
            net_via_slots: vec![Vec::new(); routes.len()],
        };
        for (i, nr) in routes.iter().enumerate() {
            idx.insert_net(i, nr);
        }
        idx
    }

    fn insert_net(&mut self, net_idx: usize, nr: &NetRoute) {
        for seg in &nr.segments {
            let slot = self.segs.len();
            let bb = BBox::from_points(&[seg.a, seg.b], seg.width * 0.5);
            self.segs.push(Some((nr.net_id, seg.clone())));
            self.seg_grid.insert(slot, bb);
            self.net_seg_slots[net_idx].push(slot);
        }
        for via in &nr.vias {
            let slot = self.vias.len();
            let bb = BBox::from_point(via.pos, via.radius);
            self.vias.push(Some((nr.net_id, via.clone())));
            self.via_grid.insert(slot, bb);
            self.net_via_slots[net_idx].push(slot);
        }
    }

    fn update_net(&mut self, net_idx: usize, nr: &NetRoute) {
        for slot in self.net_seg_slots[net_idx].drain(..) {
            self.segs[slot] = None;
        }
        for slot in self.net_via_slots[net_idx].drain(..) {
            self.vias[slot] = None;
        }
        self.insert_net(net_idx, nr);
    }

    fn clears_segment(
        &self,
        board: &Board,
        net_id: usize,
        cand: &RoutedSegment,
        buf: &mut Vec<usize>,
    ) -> bool {
        let half_w = cand.width * 0.5;
        let rules = &board.nets[net_id].rules;
        let margin = rules.clearance.max(rules.via_wire) + rules.via_radius + 100.0;
        let qbox = BBox::from_points(&[cand.a, cand.b], half_w + margin);

        self.seg_grid.query(qbox, buf);
        for &slot in buf.iter() {
            let Some((other_net, ref seg)) = self.segs[slot] else {
                continue;
            };
            if other_net == net_id || seg.layer != cand.layer {
                continue;
            }
            let req = board.wire_to_other_clearance(net_id, Some(other_net));
            let gap =
                segment_to_segment_dist(cand.a, cand.b, seg.a, seg.b) - half_w - seg.width * 0.5;
            if gap < req + 0.5 {
                return false;
            }
        }

        self.via_grid.query(qbox, buf);
        for &slot in buf.iter() {
            let Some((other_net, ref via)) = self.vias[slot] else {
                continue;
            };
            if other_net == net_id || !via.layers.contains(&cand.layer) {
                continue;
            }
            let req = board.via_to_wire_clearance(net_id, Some(other_net));
            let gap = point_to_segment_dist(via.pos, cand.a, cand.b) - half_w - via.radius;
            if gap < req + 0.5 {
                return false;
            }
        }
        true
    }
}

/// Optimizes routed nets in-place while preserving electrical connectivity and DRC cleanliness.
pub fn optimize_routes(board: &Board, routes: &mut [NetRoute]) {
    let mut index = RouteSpatialIndex::build(board, routes);
    let mut qbuf = Vec::new();

    for _round in 0..3 {
        let mut any_changed = false;
        for (idx, nr) in routes.iter_mut().enumerate() {
            if nr.segments.is_empty() && nr.vias.is_empty() {
                continue;
            }
            let before_segs = nr.segments.len();
            let before_vias = nr.vias.len();
            let before_len = route_wirelength(nr);

            prune_dangling_stubs(board, nr);
            merge_collinear_segments(nr);
            let pulled = pull_tight_chains(board, nr, &index, &mut qbuf);
            if pulled {
                merge_collinear_segments(nr);
            }
            let flattened = flatten_via_chains(board, nr, &index, &mut qbuf);
            if flattened {
                prune_dangling_stubs(board, nr);
                merge_collinear_segments(nr);
            }
            remove_redundant_vias(board, nr);

            let after_len = route_wirelength(nr);
            let changed = pulled
                || flattened
                || nr.segments.len() != before_segs
                || nr.vias.len() != before_vias
                || (before_len - after_len).abs() > 1.0;
            if changed {
                index.update_net(idx, nr);
                any_changed = true;
            }
        }
        if !any_changed {
            break;
        }
    }
}

/// Computes total routed copper length of a net in micrometers.
pub fn route_wirelength(nr: &NetRoute) -> f64 {
    nr.segments.iter().map(|s| s.a.dist(s.b)).sum()
}

#[inline]
fn pt_key(p: Point) -> (i64, i64) {
    ((p.x * 10.0).round() as i64, (p.y * 10.0).round() as i64)
}

fn collect_net_anchors(board: &Board, nr: &NetRoute) -> HashSet<(usize, i64, i64)> {
    let mut anchors = HashSet::new();
    for &pid in &board.nets[nr.net_id].pad_ids {
        let p = &board.pads[pid];
        let k = pt_key(p.center);
        for &(l, _) in &p.layer_shapes {
            anchors.insert((l, k.0, k.1));
        }
    }
    for v in &nr.vias {
        let k = pt_key(v.pos);
        for &l in &v.layers {
            anchors.insert((l, k.0, k.1));
        }
    }
    for fv in &board.fixed_vias {
        if fv.net_id == Some(nr.net_id) {
            let k = pt_key(fv.pos);
            for &l in &fv.layers {
                anchors.insert((l, k.0, k.1));
            }
        }
    }
    for fs in &board.fixed_segments {
        if fs.net_id == Some(nr.net_id) {
            let ka = pt_key(fs.a);
            let kb = pt_key(fs.b);
            anchors.insert((fs.layer, ka.0, ka.1));
            anchors.insert((fs.layer, kb.0, kb.1));
        }
    }
    anchors
}

/// Removes dead-end trace segments whose free endpoint is not anchored at a pad,
/// fixed wire/via, or routed via.
fn prune_dangling_stubs(board: &Board, nr: &mut NetRoute) {
    let anchors = collect_net_anchors(board, nr);
    loop {
        let mut degree: HashMap<(usize, i64, i64), usize> = HashMap::new();
        for s in &nr.segments {
            let ka = pt_key(s.a);
            let kb = pt_key(s.b);
            *degree.entry((s.layer, ka.0, ka.1)).or_insert(0) += 1;
            *degree.entry((s.layer, kb.0, kb.1)).or_insert(0) += 1;
        }
        let before = nr.segments.len();
        nr.segments.retain(|s| {
            let ka = pt_key(s.a);
            let kb = pt_key(s.b);
            let a_dead = degree[&(s.layer, ka.0, ka.1)] == 1
                && !anchors.contains(&(s.layer, ka.0, ka.1));
            let b_dead = degree[&(s.layer, kb.0, kb.1)] == 1
                && !anchors.contains(&(s.layer, kb.0, kb.1));
            !(a_dead || b_dead)
        });
        if nr.segments.len() == before {
            break;
        }
    }
}

/// Merges adjacent collinear segments on the same layer & width (skipping tee junctions and anchors).
fn merge_collinear_segments(nr: &mut NetRoute) {
    if nr.segments.len() < 2 {
        return;
    }
    let via_keys: HashSet<(i64, i64)> = nr.vias.iter().map(|v| pt_key(v.pos)).collect();
    loop {
        let mut degree: HashMap<(usize, i64, i64), Vec<usize>> = HashMap::new();
        for (i, s) in nr.segments.iter().enumerate() {
            let ka = pt_key(s.a);
            let kb = pt_key(s.b);
            degree.entry((s.layer, ka.0, ka.1)).or_default().push(i);
            degree.entry((s.layer, kb.0, kb.1)).or_default().push(i);
        }
        let mut merged = false;
        for (&(_layer, kx, ky), idxs) in &degree {
            if idxs.len() != 2 || via_keys.contains(&(kx, ky)) {
                continue;
            }
            let (i, j) = (idxs[0], idxs[1]);
            if i == j {
                continue;
            }
            let s1 = &nr.segments[i];
            let s2 = &nr.segments[j];
            if s1.layer != s2.layer || (s1.width - s2.width).abs() > 0.1 {
                continue;
            }
            let Some((p0, mid, p2)) = shared_endpoint(s1, s2) else {
                continue;
            };
            if is_collinear(p0, mid, p2) {
                let new_seg = RoutedSegment {
                    layer: s1.layer,
                    a: p0,
                    b: p2,
                    width: s1.width,
                };
                let (lo, hi) = if i < j { (i, j) } else { (j, i) };
                nr.segments.remove(hi);
                nr.segments[lo] = new_seg;
                merged = true;
                break;
            }
        }
        if !merged {
            break;
        }
    }
}

struct SegmentChain {
    layer: usize,
    width: f64,
    seg_indices: Vec<usize>,
    points: Vec<Point>,
}

/// Extracts maximal unbranched same-layer chains whose interior points have no pads,
/// vias, tees, or T-junction contacts from other segments.
fn extract_chains(board: &Board, nr: &NetRoute) -> Vec<SegmentChain> {
    let anchors = collect_net_anchors(board, nr);
    let mut adj: HashMap<(usize, i64, i64), Vec<usize>> = HashMap::new();
    for (i, s) in nr.segments.iter().enumerate() {
        let ka = pt_key(s.a);
        let kb = pt_key(s.b);
        adj.entry((s.layer, ka.0, ka.1)).or_default().push(i);
        adj.entry((s.layer, kb.0, kb.1)).or_default().push(i);
    }

    // Mark segments touched in their interior by any anchor or other segment endpoint.
    let mut has_interior_tap = vec![false; nr.segments.len()];
    let mut check_pts: Vec<(usize, Point)> = Vec::new();
    for &(l, kx, ky) in &anchors {
        check_pts.push((l, Point::new(kx as f64 * 0.1, ky as f64 * 0.1)));
    }
    for s in &nr.segments {
        check_pts.push((s.layer, s.a));
        check_pts.push((s.layer, s.b));
    }
    for (i, s) in nr.segments.iter().enumerate() {
        let ka = pt_key(s.a);
        let kb = pt_key(s.b);
        for &(l, pt) in &check_pts {
            if l != s.layer {
                continue;
            }
            let kp = pt_key(pt);
            if kp == ka || kp == kb {
                continue;
            }
            if point_to_segment_dist(pt, s.a, s.b) <= s.width * 0.5 + 2.0 {
                has_interior_tap[i] = true;
                break;
            }
        }
    }

    let is_interior = |layer: usize, width: f64, p: Point| -> Option<(usize, usize)> {
        let k = pt_key(p);
        if anchors.contains(&(layer, k.0, k.1)) {
            return None;
        }
        let inc = adj.get(&(layer, k.0, k.1))?;
        if inc.len() != 2 {
            return None;
        }
        let (a, b) = (inc[0], inc[1]);
        if has_interior_tap[a] || has_interior_tap[b] {
            return None;
        }
        if (nr.segments[a].width - width).abs() > 0.1 || (nr.segments[b].width - width).abs() > 0.1
        {
            return None;
        }
        Some((a, b))
    };

    let mut visited = vec![false; nr.segments.len()];
    let mut chains = Vec::new();

    for start_idx in 0..nr.segments.len() {
        if visited[start_idx] || has_interior_tap[start_idx] {
            continue;
        }
        let layer = nr.segments[start_idx].layer;
        let width = nr.segments[start_idx].width;
        visited[start_idx] = true;

        let mut seg_indices = vec![start_idx];
        let mut points = vec![nr.segments[start_idx].a, nr.segments[start_idx].b];

        // Extend forward from points.last().
        loop {
            let cur_seg = *seg_indices.last().unwrap();
            let tail = *points.last().unwrap();
            let Some((a, b)) = is_interior(layer, width, tail) else {
                break;
            };
            let next_seg = if a == cur_seg { b } else { a };
            if visited[next_seg] {
                break;
            }
            visited[next_seg] = true;
            let ns = &nr.segments[next_seg];
            let next_pt = if pt_key(ns.a) == pt_key(tail) {
                ns.b
            } else {
                ns.a
            };
            seg_indices.push(next_seg);
            points.push(next_pt);
        }

        // Extend backward from points[0].
        loop {
            let cur_seg = seg_indices[0];
            let head = points[0];
            let Some((a, b)) = is_interior(layer, width, head) else {
                break;
            };
            let prev_seg = if a == cur_seg { b } else { a };
            if visited[prev_seg] {
                break;
            }
            visited[prev_seg] = true;
            let ps = &nr.segments[prev_seg];
            let prev_pt = if pt_key(ps.b) == pt_key(head) {
                ps.a
            } else {
                ps.b
            };
            seg_indices.insert(0, prev_seg);
            points.insert(0, prev_pt);
        }

        chains.push(SegmentChain {
            layer,
            width,
            seg_indices,
            points,
        });
    }
    chains
}

/// Pulls multi-segment staircases and detours tight using 1- or 2-segment octilinear shortcuts.
fn pull_tight_chains(
    board: &Board,
    nr: &mut NetRoute,
    index: &RouteSpatialIndex,
    qbuf: &mut Vec<usize>,
) -> bool {
    let chains = extract_chains(board, nr);
    let mut remove_mask = vec![false; nr.segments.len()];
    let mut additions: Vec<RoutedSegment> = Vec::new();
    let mut any_changed = false;

    for chain in chains {
        if chain.points.len() < 3 {
            continue;
        }
        let pts = &chain.points;
        let mut new_pts = vec![pts[0]];
        let mut i = 0;
        let mut chain_changed = false;

        while i + 1 < pts.len() {
            let max_j = (i + 8).min(pts.len() - 1);
            let mut chosen: Option<(usize, Option<Point>)> = None;

            for j in (i + 2..=max_j).rev() {
                let sub_len: f64 = pts[i..=j].windows(2).map(|w| w[0].dist(w[1])).sum();
                let a = pts[i];
                let b = pts[j];

                // 1. Try single direct octilinear segment.
                if is_octilinear(a, b)
                    && a.dist(b) + 1.0 < sub_len
                    && segment_clear(board, nr.net_id, chain.layer, a, b, chain.width, index, qbuf)
                {
                    chosen = Some((j, None));
                    break;
                }

                // 2. Try two-segment 45-degree + H/V octilinear dogleg shortcut.
                if j >= i + 3 || !is_octilinear(a, b) {
                    for mid in octilinear_elbows(a, b) {
                        let new_len = a.dist(mid) + mid.dist(b);
                        if new_len + 5.0 < sub_len
                            && segment_clear(
                                board,
                                nr.net_id,
                                chain.layer,
                                a,
                                mid,
                                chain.width,
                                index,
                                qbuf,
                            )
                            && segment_clear(
                                board,
                                nr.net_id,
                                chain.layer,
                                mid,
                                b,
                                chain.width,
                                index,
                                qbuf,
                            )
                        {
                            chosen = Some((j, Some(mid)));
                            break;
                        }
                    }
                    if chosen.is_some() {
                        break;
                    }
                }
            }

            if let Some((j, mid_opt)) = chosen {
                if let Some(mid) = mid_opt {
                    new_pts.push(mid);
                }
                new_pts.push(pts[j]);
                i = j;
                chain_changed = true;
            } else {
                new_pts.push(pts[i + 1]);
                i += 1;
            }
        }

        if chain_changed {
            for &si in &chain.seg_indices {
                remove_mask[si] = true;
            }
            for pair in new_pts.windows(2) {
                if pair[0].dist(pair[1]) > 0.5 {
                    additions.push(RoutedSegment {
                        layer: chain.layer,
                        a: pair[0],
                        b: pair[1],
                        width: chain.width,
                    });
                }
            }
            any_changed = true;
        }
    }

    if any_changed {
        let mut kept = Vec::with_capacity(nr.segments.len());
        for (i, seg) in nr.segments.drain(..).enumerate() {
            if !remove_mask[i] {
                kept.push(seg);
            }
        }
        kept.extend(additions);
        nr.segments = kept;
    }
    any_changed
}

/// Moves inter-via chains onto another layer already connected at both ends so redundant vias drop.
fn flatten_via_chains(
    board: &Board,
    nr: &mut NetRoute,
    index: &RouteSpatialIndex,
    qbuf: &mut Vec<usize>,
) -> bool {
    if nr.vias.is_empty() || board.signal_layers.len() < 2 {
        return false;
    }
    let chains = extract_chains(board, nr);
    let mut remove_mask = vec![false; nr.segments.len()];
    let mut additions: Vec<RoutedSegment> = Vec::new();
    let mut any_changed = false;

    for chain in chains {
        let start = chain.points[0];
        let end = *chain.points.last().unwrap();
        let start_layers = layers_at_point(board, nr, start, chain.layer);
        let end_layers = layers_at_point(board, nr, end, chain.layer);
        if start_layers.is_empty() || end_layers.is_empty() {
            continue;
        }

        let mut target_layer = None;
        let mut replacement: Option<Vec<RoutedSegment>> = None;

        for &cand_l in &board.signal_layers {
            if cand_l == chain.layer
                || !start_layers.contains(&cand_l)
                || !end_layers.contains(&cand_l)
            {
                continue;
            }
            // Prefer a direct 1- or 2-segment octilinear path on cand_l, else move existing points.
            if is_octilinear(start, end)
                && segment_clear(board, nr.net_id, cand_l, start, end, chain.width, index, qbuf)
            {
                target_layer = Some(cand_l);
                replacement = Some(vec![RoutedSegment {
                    layer: cand_l,
                    a: start,
                    b: end,
                    width: chain.width,
                }]);
                break;
            }
            let mut elbow_ok = None;
            for mid in octilinear_elbows(start, end) {
                if segment_clear(board, nr.net_id, cand_l, start, mid, chain.width, index, qbuf)
                    && segment_clear(board, nr.net_id, cand_l, mid, end, chain.width, index, qbuf)
                {
                    elbow_ok = Some(vec![
                        RoutedSegment {
                            layer: cand_l,
                            a: start,
                            b: mid,
                            width: chain.width,
                        },
                        RoutedSegment {
                            layer: cand_l,
                            a: mid,
                            b: end,
                            width: chain.width,
                        },
                    ]);
                    break;
                }
            }
            if let Some(segs) = elbow_ok {
                target_layer = Some(cand_l);
                replacement = Some(segs);
                break;
            }
            let all_clear = chain.points.windows(2).all(|w| {
                segment_clear(board, nr.net_id, cand_l, w[0], w[1], chain.width, index, qbuf)
            });
            if all_clear {
                target_layer = Some(cand_l);
                replacement = Some(
                    chain
                        .points
                        .windows(2)
                        .map(|w| RoutedSegment {
                            layer: cand_l,
                            a: w[0],
                            b: w[1],
                            width: chain.width,
                        })
                        .collect(),
                );
                break;
            }
        }

        if let (Some(_), Some(new_segs)) = (target_layer, replacement) {
            for &si in &chain.seg_indices {
                remove_mask[si] = true;
            }
            additions.extend(new_segs);
            any_changed = true;
        }
    }

    if any_changed {
        let mut kept = Vec::with_capacity(nr.segments.len());
        for (i, seg) in nr.segments.drain(..).enumerate() {
            if !remove_mask[i] {
                kept.push(seg);
            }
        }
        kept.extend(additions);
        nr.segments = kept;
    }
    any_changed
}

/// Lists layers (excluding `cur_layer`) that are electrically connected at `pt` for this net.
fn layers_at_point(board: &Board, nr: &NetRoute, pt: Point, cur_layer: usize) -> Vec<usize> {
    let pk = pt_key(pt);
    let has_via = nr.vias.iter().any(|v| pt_key(v.pos) == pk);
    let mut layers = HashSet::new();
    for s in &nr.segments {
        if s.layer != cur_layer && (pt_key(s.a) == pk || pt_key(s.b) == pk) {
            layers.insert(s.layer);
        }
    }
    for &pid in &board.nets[nr.net_id].pad_ids {
        let p = &board.pads[pid];
        if p.center.dist(pt) <= p.max_radius + 5.0 {
            for &(l, _) in &p.layer_shapes {
                if l != cur_layer {
                    layers.insert(l);
                }
            }
        }
    }
    for fs in &board.fixed_segments {
        if fs.net_id == Some(nr.net_id)
            && fs.layer != cur_layer
            && (pt_key(fs.a) == pk || pt_key(fs.b) == pk)
        {
            layers.insert(fs.layer);
        }
    }
    if !has_via {
        // Without a via at `pt`, only pad-accessible layers qualify.
        layers.retain(|&l| {
            board.nets[nr.net_id].pad_ids.iter().any(|&pid| {
                let p = &board.pads[pid];
                p.center.dist(pt) <= p.max_radius + 5.0 && p.on_layer(l)
            })
        });
    }
    layers.into_iter().collect()
}

/// Removes vias that do not actually connect segments/pads on at least two distinct layers.
fn remove_redundant_vias(board: &Board, nr: &mut NetRoute) {
    nr.vias.retain(|via| {
        let vk = pt_key(via.pos);
        let mut layers: HashSet<usize> = HashSet::new();
        for s in &nr.segments {
            if pt_key(s.a) == vk || pt_key(s.b) == vk {
                layers.insert(s.layer);
            }
        }
        for &pid in &board.nets[nr.net_id].pad_ids {
            let p = &board.pads[pid];
            if p.center.dist(via.pos) <= p.max_radius + 5.0 {
                for &(l, _) in &p.layer_shapes {
                    layers.insert(l);
                }
            }
        }
        for fs in &board.fixed_segments {
            if fs.net_id == Some(nr.net_id) && (pt_key(fs.a) == vk || pt_key(fs.b) == vk) {
                layers.insert(fs.layer);
            }
        }
        layers.len() >= 2
    });
}

#[allow(clippy::too_many_arguments)]
fn segment_clear(
    board: &Board,
    net_id: usize,
    layer: usize,
    a: Point,
    b: Point,
    width: f64,
    index: &RouteSpatialIndex,
    qbuf: &mut Vec<usize>,
) -> bool {
    let cand = RoutedSegment { layer, a, b, width };
    board.check_segment_static(net_id, layer, a, b, width, 0.5)
        && index.clears_segment(board, net_id, &cand, qbuf)
}

/// Returns the two canonical 45-degree + axis-aligned elbow midpoints between `a` and `b`.
fn octilinear_elbows(a: Point, b: Point) -> [Point; 2] {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let d = dx.abs().min(dy.abs());
    let sx = dx.signum();
    let sy = dy.signum();
    let m1 = Point::new(a.x + sx * d, a.y + sy * d);
    let m2 = Point::new(b.x - sx * d, b.y - sy * d);
    [m1, m2]
}

fn shared_endpoint(s1: &RoutedSegment, s2: &RoutedSegment) -> Option<(Point, Point, Point)> {
    let (a1, b1) = (pt_key(s1.a), pt_key(s1.b));
    let (a2, b2) = (pt_key(s2.a), pt_key(s2.b));
    if b1 == a2 {
        Some((s1.a, s1.b, s2.b))
    } else if b1 == b2 {
        Some((s1.a, s1.b, s2.a))
    } else if a1 == a2 {
        Some((s1.b, s1.a, s2.b))
    } else if a1 == b2 {
        Some((s1.b, s1.a, s2.a))
    } else {
        None
    }
}

fn is_collinear(a: Point, b: Point, c: Point) -> bool {
    let ux = b.x - a.x;
    let uy = b.y - a.y;
    let vx = c.x - b.x;
    let vy = c.y - b.y;
    let cross = (ux * vy - uy * vx).abs();
    let dot = ux * vx + uy * vy;
    cross < 1e-2 && dot > 0.0
}

fn is_octilinear(a: Point, b: Point) -> bool {
    let dx = (b.x - a.x).abs();
    let dy = (b.y - a.y).abs();
    dx < 1.0 || dy < 1.0 || (dx - dy).abs() < 1.0
}
