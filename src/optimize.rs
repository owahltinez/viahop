//! Post-route geometry optimizer: stub pruning, O(N) collinear merging, multi-segment
//! octilinear pull-tight shortcutting, and via-pair elimination with continuous DRC gating.

use std::collections::{HashMap, HashSet};

use crate::board::{Board, NetRoute, RoutedSegment, RoutedVia, SpatialGrid};
use crate::geometry::{BBox, Point, point_to_segment_dist, segment_to_segment_dist};

/// Spatial index over routed segments and vias for fast continuous clearance checks.
pub(crate) struct RouteSpatialIndex {
    segs: Vec<Option<(usize, RoutedSegment)>>,
    vias: Vec<Option<(usize, RoutedVia)>>,
    seg_grid: SpatialGrid,
    via_grid: SpatialGrid,
    net_seg_slots: Vec<Vec<usize>>,
    net_via_slots: Vec<Vec<usize>>,
    max_board_clr: f64,
}

impl RouteSpatialIndex {
    fn build(board: &Board, routes: &[NetRoute]) -> Self {
        Self::build_filtered(board, routes, |_| true)
    }

    pub(crate) fn build_filtered(
        board: &Board,
        routes: &[NetRoute],
        include: impl Fn(usize) -> bool,
    ) -> Self {
        let max_via_r = board
            .nets
            .iter()
            .map(|n| n.rules.via_radius)
            .chain(
                routes
                    .iter()
                    .filter(|nr| include(nr.net_id))
                    .flat_map(|nr| nr.vias.iter().map(|v| v.radius)),
            )
            .fold(0.0_f64, f64::max);
        let max_via_d = board
            .nets
            .iter()
            .map(|n| n.rules.via_drill)
            .chain(
                routes
                    .iter()
                    .filter(|nr| include(nr.net_id))
                    .flat_map(|nr| nr.vias.iter().map(|v| v.drill)),
            )
            .fold(0.0_f64, f64::max);
        let max_board_clr = (board.max_net_clearance + max_via_r)
            .max(board.hole_to_hole_um + max_via_d * 0.5 + 100.0)
            .max(250.0);
        let mut idx = Self {
            segs: Vec::new(),
            vias: Vec::new(),
            seg_grid: SpatialGrid::new(board.bounds, 1000.0),
            via_grid: SpatialGrid::new(board.bounds, 1000.0),
            net_seg_slots: vec![Vec::new(); routes.len()],
            net_via_slots: vec![Vec::new(); routes.len()],
            max_board_clr,
        };
        for (i, nr) in routes.iter().enumerate() {
            if include(nr.net_id) {
                idx.insert_net(i, nr);
            }
        }
        idx
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.segs.is_empty() && self.vias.is_empty()
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

    pub(crate) fn clears_segment(
        &self,
        board: &Board,
        net_id: usize,
        cand: &RoutedSegment,
        margin: f64,
        buf: &mut Vec<usize>,
    ) -> bool {
        let half_w = cand.width * 0.5;
        let qbox = BBox::from_points(&[cand.a, cand.b], half_w + self.max_board_clr + 50.0);

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
            if gap < req + margin {
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
            if gap < req + margin {
                return false;
            }
        }
        true
    }

    #[allow(clippy::too_many_arguments)]
    fn clears_via_spec(
        &self,
        board: &Board,
        net_id: usize,
        pos: Point,
        radius: f64,
        drill: f64,
        layers: &[usize],
        margin: f64,
        buf: &mut Vec<usize>,
    ) -> bool {
        let qbox = BBox::from_point(pos, radius + self.max_board_clr + 50.0);

        self.via_grid.query(qbox, buf);
        for &slot in buf.iter() {
            let Some((other_net, ref ov)) = self.vias[slot] else {
                continue;
            };
            if other_net == net_id || !layers.iter().any(|l| ov.layers.contains(l)) {
                continue;
            }
            let dist = pos.dist(ov.pos);
            let req = board.via_to_via_clearance(net_id, Some(other_net));
            let min_hole_c2c = (drill + ov.drill) * 0.5 + board.hole_to_hole_um;
            if dist - radius - ov.radius < req + margin || dist < min_hole_c2c + margin {
                return false;
            }
        }

        self.seg_grid.query(qbox, buf);
        for &slot in buf.iter() {
            let Some((other_net, ref seg)) = self.segs[slot] else {
                continue;
            };
            if other_net == net_id || !layers.contains(&seg.layer) {
                continue;
            }
            let req = board.via_to_wire_clearance(net_id, Some(other_net));
            let gap = point_to_segment_dist(pos, seg.a, seg.b) - radius - seg.width * 0.5;
            if gap < req + margin {
                return false;
            }
        }
        true
    }

    pub(crate) fn clears_via(
        &self,
        board: &Board,
        net_id: usize,
        pos: Point,
        margin: f64,
        buf: &mut Vec<usize>,
    ) -> bool {
        let rules = &board.nets[net_id].rules;
        self.clears_via_spec(
            board,
            net_id,
            pos,
            rules.via_radius,
            rules.via_drill,
            &rules.via_layers,
            margin,
            buf,
        )
    }
}

/// Optimizes routed nets in-place while preserving electrical connectivity and DRC cleanliness.
pub fn optimize_routes(board: &Board, routes: &mut [NetRoute]) {
    optimize_routes_subset(board, routes, None);
}

/// Optimizes routed nets in-place, editing only `target_mask` when set while indexing all nets.
pub(crate) fn optimize_routes_subset(
    board: &Board,
    routes: &mut [NetRoute],
    target_mask: Option<&[bool]>,
) {
    let mut index = RouteSpatialIndex::build(board, routes);
    let mut qbuf = Vec::new();
    let mut sbuf = Vec::new();

    for _round in 0..3 {
        let mut any_changed = false;
        for (idx, nr) in routes.iter_mut().enumerate() {
            if let Some(mask) = target_mask
                && !mask.get(nr.net_id).copied().unwrap_or(false)
            {
                continue;
            }
            if nr.segments.is_empty() && nr.vias.is_empty() {
                continue;
            }
            let before_segs = nr.segments.len();
            let before_vias = nr.vias.len();
            let before_len = route_wirelength(nr);

            prune_dangling_stubs(board, nr);
            let snapped = snap_pofv_vias_for_net(board, nr, &index, &mut qbuf, &mut sbuf);
            merge_collinear_segments(nr);
            let pulled = pull_tight_chains(board, nr, &index, &mut qbuf, &mut sbuf);
            if pulled {
                merge_collinear_segments(nr);
            }
            let flattened = flatten_via_chains(board, nr, &index, &mut qbuf, &mut sbuf);
            if flattened {
                prune_dangling_stubs(board, nr);
                merge_collinear_segments(nr);
            }
            prune_dead_vias(board, nr);

            let after_len = route_wirelength(nr);
            let changed = snapped
                || pulled
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

    for nr in routes.iter_mut() {
        if let Some(mask) = target_mask
            && !mask.get(nr.net_id).copied().unwrap_or(false)
        {
            continue;
        }
        nr.segments = split_segments_at_junctions(&nr.segments, &nr.vias);
    }
}

/// Applies final route hygiene: dead-via pruning, POFV via snapping, and T-junction splitting.
pub fn finalize_routes(board: &Board, routes: &mut [NetRoute]) {
    finalize_routes_subset(board, routes, None);
}

/// Applies final route hygiene, editing only `target_mask` when set while indexing all nets.
pub(crate) fn finalize_routes_subset(
    board: &Board,
    routes: &mut [NetRoute],
    target_mask: Option<&[bool]>,
) {
    for nr in routes.iter_mut() {
        if let Some(mask) = target_mask
            && !mask.get(nr.net_id).copied().unwrap_or(false)
        {
            continue;
        }
        prune_dead_vias(board, nr);
    }
    snap_pofv_vias_subset(board, routes, target_mask);
    for nr in routes.iter_mut() {
        if let Some(mask) = target_mask
            && !mask.get(nr.net_id).copied().unwrap_or(false)
        {
            continue;
        }
        prune_dead_vias(board, nr);
        nr.segments = split_segments_at_junctions(&nr.segments, &nr.vias);
    }
}

/// Snaps off-center routed vias falling within a same-net SMD pad to exact `pad.center` when DRC-safe.
pub fn snap_pofv_vias(board: &Board, routes: &mut [NetRoute]) {
    snap_pofv_vias_subset(board, routes, None);
}

fn snap_pofv_vias_subset(board: &Board, routes: &mut [NetRoute], target_mask: Option<&[bool]>) {
    if !board.via_at_smd {
        return;
    }
    let mut index = RouteSpatialIndex::build(board, routes);
    let mut qbuf = Vec::new();
    let mut sbuf = Vec::new();
    for (idx, nr) in routes.iter_mut().enumerate() {
        if let Some(mask) = target_mask
            && !mask.get(nr.net_id).copied().unwrap_or(false)
        {
            continue;
        }
        if snap_pofv_vias_for_net(board, nr, &index, &mut qbuf, &mut sbuf) {
            index.update_net(idx, nr);
        }
    }
}

fn snap_pofv_vias_for_net(
    board: &Board,
    nr: &mut NetRoute,
    index: &RouteSpatialIndex,
    qbuf: &mut Vec<usize>,
    sbuf: &mut Vec<usize>,
) -> bool {
    if !board.via_at_smd || nr.vias.is_empty() {
        return false;
    }
    let net_id = nr.net_id;
    let smd_pads: Vec<&crate::board::Pad> = board.nets[net_id]
        .pad_ids
        .iter()
        .map(|&pid| &board.pads[pid])
        .filter(|p| p.is_smd)
        .collect();
    if smd_pads.is_empty() {
        return false;
    }

    let mut any_changed = false;
    let mut vi = 0;
    while vi < nr.vias.len() {
        let old_via = nr.vias[vi].clone();
        let old_pos = old_via.pos;

        // Skip vias already centered on any same-net SMD pad on their layers.
        let already_centered = smd_pads.iter().any(|&pad| {
            old_pos.dist(pad.center) <= 1.0
                && pad
                    .layer_shapes
                    .iter()
                    .any(|&(l, _)| old_via.layers.contains(&l))
        });
        if already_centered {
            vi += 1;
            continue;
        }

        let target_pad = smd_pads
            .iter()
            .filter_map(|&pad| {
                let inside = pad.layer_shapes.iter().any(|&(l, ref sh)| {
                    old_via.layers.contains(&l) && sh.dist_to_circle(old_pos, 0.0) <= 0.0
                });
                inside.then(|| (old_pos.dist(pad.center), pad))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, pad)| pad);

        let Some(pad) = target_pad else {
            vi += 1;
            continue;
        };
        let new_pos = pad.center;

        let via_ok = board.check_routed_via_static_with_buf(
            net_id,
            new_pos,
            old_via.radius,
            old_via.drill,
            &old_via.layers,
            0.0,
            sbuf,
        ) && index.clears_via_spec(
            board,
            net_id,
            new_pos,
            old_via.radius,
            old_via.drill,
            &old_via.layers,
            0.0,
            qbuf,
        ) && nr.vias.iter().enumerate().all(|(j, sv)| {
            if j == vi || !old_via.layers.iter().any(|l| sv.layers.contains(l)) {
                return true;
            }
            let dist = new_pos.dist(sv.pos);
            let min_hole_c2c = (old_via.drill + sv.drill) * 0.5 + board.hole_to_hole_um;
            (dist < 1.0 && old_via.layers.iter().all(|l| sv.layers.contains(l)))
                || dist >= min_hole_c2c
        }) && board.fixed_vias.iter().all(|fv| {
            if fv.net_id != Some(net_id) {
                return true;
            }
            let min_hole_c2c = (old_via.drill + fv.drill) * 0.5 + board.hole_to_hole_um;
            new_pos.dist(fv.pos) >= min_hole_c2c
        });
        if !via_ok {
            vi += 1;
            continue;
        }

        // Preserve electrical contact with any static copper touching old_pos by copper overlap.
        let static_contact_ok = board.fixed_segments.iter().all(|fs| {
            if fs.net_id != Some(net_id) || !old_via.layers.contains(&fs.layer) {
                return true;
            }
            let tol = old_via.radius + fs.half_w;
            point_to_segment_dist(old_pos, fs.a, fs.b) > tol + 2.0
                || point_to_segment_dist(new_pos, fs.a, fs.b) <= tol
        }) && board.fixed_vias.iter().all(|fv| {
            if fv.net_id != Some(net_id) || !old_via.layers.iter().any(|l| fv.layers.contains(l)) {
                return true;
            }
            let tol = old_via.radius + fv.radius;
            old_pos.dist(fv.pos) > tol + 2.0 || new_pos.dist(fv.pos) <= tol
        }) && nr.vias.iter().enumerate().all(|(j, sv)| {
            if j == vi || !old_via.layers.iter().any(|l| sv.layers.contains(l)) {
                return true;
            }
            let tol = old_via.radius + sv.radius;
            old_pos.dist(sv.pos) > tol + 2.0 || new_pos.dist(sv.pos) <= tol
        }) && board.planes.iter().all(|fp| {
            if fp.net_id != Some(net_id) || !old_via.layers.contains(&fp.layer) {
                return true;
            }
            fp.shape.dist_to_circle(old_pos, old_via.radius) > 2.0
                || fp.shape.dist_to_circle(new_pos, old_via.radius) <= 0.0
        }) && board.nets[net_id].pad_ids.iter().all(|&pid| {
            board.pads[pid].layer_shapes.iter().all(|&(l, ref sh)| {
                !old_via.layers.contains(&l)
                    || sh.dist_to_circle(old_pos, old_via.radius) > 2.0
                    || sh.dist_to_circle(new_pos, old_via.radius) <= 0.0
            })
        });
        if !static_contact_ok {
            vi += 1;
            continue;
        }

        // Split interior T-junctions first so moving old_pos never pivots a trunk off a branch.
        let base_segs = split_segments_at_junctions(&nr.segments, &nr.vias);
        let mut cand_segs: Vec<RoutedSegment> = Vec::with_capacity(base_segs.len() + 2);
        let mut segs_ok = true;
        let seg_ok = |ns: &RoutedSegment, sbuf: &mut Vec<usize>, qbuf: &mut Vec<usize>| -> bool {
            board.check_segment_static_with_buf(net_id, ns.layer, ns.a, ns.b, ns.width, 0.0, sbuf)
                && index.clears_segment(board, net_id, ns, 0.0, qbuf)
        };

        for seg in &base_segs {
            if !old_via.layers.contains(&seg.layer) {
                cand_segs.push(seg.clone());
                continue;
            }
            let a_hit = seg.a.dist(old_pos) <= 1.5;
            let b_hit = seg.b.dist(old_pos) <= 1.5;
            if a_hit || b_hit {
                let na = if a_hit { new_pos } else { seg.a };
                let nb = if b_hit { new_pos } else { seg.b };
                if na.dist(nb) > 0.5 {
                    let ns = RoutedSegment {
                        layer: seg.layer,
                        a: na,
                        b: nb,
                        width: seg.width,
                    };
                    if !seg_ok(&ns, sbuf, qbuf) {
                        segs_ok = false;
                        break;
                    }
                    cand_segs.push(ns);
                }
            } else if point_to_segment_dist(old_pos, seg.a, seg.b) <= 0.5 {
                for (na, nb) in [(seg.a, new_pos), (new_pos, seg.b)] {
                    if na.dist(nb) > 0.5 {
                        let ns = RoutedSegment {
                            layer: seg.layer,
                            a: na,
                            b: nb,
                            width: seg.width,
                        };
                        if !seg_ok(&ns, sbuf, qbuf) {
                            segs_ok = false;
                            break;
                        }
                        cand_segs.push(ns);
                    }
                }
                if !segs_ok {
                    break;
                }
            } else {
                let tol = old_via.radius + seg.width * 0.5;
                if point_to_segment_dist(old_pos, seg.a, seg.b) <= tol + 2.0
                    && point_to_segment_dist(new_pos, seg.a, seg.b) > tol
                {
                    segs_ok = false;
                    break;
                }
                cand_segs.push(seg.clone());
            }
        }
        if !segs_ok {
            vi += 1;
            continue;
        }

        nr.segments = cand_segs;
        let dup_exists = nr.vias.iter().enumerate().any(|(j, sv)| {
            j != vi
                && sv.pos.dist(new_pos) < 1.0
                && old_via.layers.iter().all(|l| sv.layers.contains(l))
        });
        if dup_exists {
            nr.vias.remove(vi);
        } else {
            nr.vias[vi].pos = new_pos;
            vi += 1;
        }
        any_changed = true;
    }

    any_changed
}

/// Splits segments at interior same-layer T-junction endpoints and via centers.
pub fn split_segments_at_junctions(
    segments: &[RoutedSegment],
    vias: &[RoutedVia],
) -> Vec<RoutedSegment> {
    if segments.is_empty() {
        return Vec::new();
    }
    let mut pts_by_layer: HashMap<usize, Vec<Point>> = HashMap::new();
    let mut seen_pts: HashSet<(usize, i64, i64)> = HashSet::new();
    let mut push_pt = |layer: usize, pt: Point| {
        let k = pt_key(pt);
        if seen_pts.insert((layer, k.0, k.1)) {
            pts_by_layer.entry(layer).or_default().push(pt);
        }
    };
    for s in segments {
        push_pt(s.layer, s.a);
        push_pt(s.layer, s.b);
    }
    for v in vias {
        for &l in &v.layers {
            push_pt(l, v.pos);
        }
    }

    type SegKey = (usize, (i64, i64), (i64, i64), i64);
    let mut out = Vec::with_capacity(segments.len());
    let mut seen_segs: HashSet<SegKey> = HashSet::new();
    let mut push_seg = |layer: usize, a: Point, b: Point, width: f64| {
        if a.dist(b) <= 0.5 {
            return;
        }
        let ka = pt_key(a);
        let kb = pt_key(b);
        let (k0, k1) = if ka <= kb { (ka, kb) } else { (kb, ka) };
        let wk = (width * 10.0).round() as i64;
        if seen_segs.insert((layer, k0, k1, wk)) {
            out.push(RoutedSegment { layer, a, b, width });
        }
    };

    let mut splits: Vec<(f64, Point)> = Vec::new();
    for s in segments {
        let len = s.a.dist(s.b);
        if len <= 0.5 {
            continue;
        }
        splits.clear();
        if let Some(layer_pts) = pts_by_layer.get(&s.layer) {
            let ux = (s.b.x - s.a.x) / len;
            let uy = (s.b.y - s.a.y) / len;
            let min_x = s.a.x.min(s.b.x) - 1.0;
            let max_x = s.a.x.max(s.b.x) + 1.0;
            let min_y = s.a.y.min(s.b.y) - 1.0;
            let max_y = s.a.y.max(s.b.y) + 1.0;
            for &pt in layer_pts {
                if pt.x < min_x || pt.x > max_x || pt.y < min_y || pt.y > max_y {
                    continue;
                }
                let dx = pt.x - s.a.x;
                let dy = pt.y - s.a.y;
                let t = dx * ux + dy * uy;
                if t <= 1.0 || t >= len - 1.0 {
                    continue;
                }
                let perp = (dx * (-uy) + dy * ux).abs();
                if perp <= 0.5 {
                    splits.push((t, pt));
                }
            }
        }
        if splits.is_empty() {
            push_seg(s.layer, s.a, s.b, s.width);
            continue;
        }
        splits.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut cur = s.a;
        for &(_, pt) in &splits {
            if cur.dist(pt) > 0.5 {
                push_seg(s.layer, cur, pt, s.width);
                cur = pt;
            }
        }
        push_seg(s.layer, cur, s.b, s.width);
    }
    out
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
            let a_dead =
                degree[&(s.layer, ka.0, ka.1)] == 1 && !anchors.contains(&(s.layer, ka.0, ka.1));
            let b_dead =
                degree[&(s.layer, kb.0, kb.1)] == 1 && !anchors.contains(&(s.layer, kb.0, kb.1));
            !(a_dead || b_dead)
        });
        if nr.segments.len() == before {
            break;
        }
    }
}

/// Merges adjacent collinear segments on the same layer & width in O(N) time.
fn merge_collinear_segments(nr: &mut NetRoute) {
    if nr.segments.len() < 2 {
        return;
    }
    let via_keys: HashSet<(i64, i64)> = nr.vias.iter().map(|v| pt_key(v.pos)).collect();
    let mut adj: HashMap<(usize, i64, i64), Vec<usize>> = HashMap::new();
    for (i, s) in nr.segments.iter().enumerate() {
        let ka = pt_key(s.a);
        let kb = pt_key(s.b);
        adj.entry((s.layer, ka.0, ka.1)).or_default().push(i);
        adj.entry((s.layer, kb.0, kb.1)).or_default().push(i);
    }

    let mut used = vec![false; nr.segments.len()];
    let mut out = Vec::with_capacity(nr.segments.len());

    let next_collinear = |cur_seg: usize,
                          tip: Point,
                          prev: Point,
                          layer: usize,
                          width: f64,
                          used: &[bool]|
     -> Option<(usize, Point)> {
        let tk = pt_key(tip);
        if via_keys.contains(&tk) {
            return None;
        }
        let inc = adj.get(&(layer, tk.0, tk.1))?;
        if inc.len() != 2 {
            return None;
        }
        let other = if inc[0] == cur_seg { inc[1] } else { inc[0] };
        if used[other] {
            return None;
        }
        let os = &nr.segments[other];
        if os.layer != layer || (os.width - width).abs() > 0.1 {
            return None;
        }
        let next_pt = if pt_key(os.a) == tk { os.b } else { os.a };
        if is_collinear(prev, tip, next_pt) {
            Some((other, next_pt))
        } else {
            None
        }
    };

    for i in 0..nr.segments.len() {
        if used[i] {
            continue;
        }
        used[i] = true;
        let layer = nr.segments[i].layer;
        let width = nr.segments[i].width;
        let mut a = nr.segments[i].a;
        let mut b = nr.segments[i].b;

        let mut cur_b = i;
        while let Some((nxt, pt)) = next_collinear(cur_b, b, a, layer, width, &used) {
            used[nxt] = true;
            b = pt;
            cur_b = nxt;
        }
        let mut cur_a = i;
        while let Some((nxt, pt)) = next_collinear(cur_a, a, b, layer, width, &used) {
            used[nxt] = true;
            a = pt;
            cur_a = nxt;
        }
        out.push(RoutedSegment { layer, a, b, width });
    }
    nr.segments = out;
}

struct SegmentChain {
    layer: usize,
    width: f64,
    seg_indices: Vec<usize>,
    points: Vec<Point>,
}

/// Extracts maximal unbranched same-layer chains whose interior vertices have no pads,
/// vias, tees, or interior T-junction taps.
fn extract_chains(board: &Board, nr: &NetRoute) -> Vec<SegmentChain> {
    let anchors = collect_net_anchors(board, nr);
    let mut adj: HashMap<(usize, i64, i64), Vec<usize>> = HashMap::new();
    for (i, s) in nr.segments.iter().enumerate() {
        let ka = pt_key(s.a);
        let kb = pt_key(s.b);
        adj.entry((s.layer, ka.0, ka.1)).or_default().push(i);
        adj.entry((s.layer, kb.0, kb.1)).or_default().push(i);
    }

    let mut has_interior_tap = vec![false; nr.segments.len()];
    let mut pts_by_layer: HashMap<usize, Vec<Point>> = HashMap::new();
    for &(l, kx, ky) in &anchors {
        pts_by_layer
            .entry(l)
            .or_default()
            .push(Point::new(kx as f64 * 0.1, ky as f64 * 0.1));
    }
    for s in &nr.segments {
        let list = pts_by_layer.entry(s.layer).or_default();
        list.push(s.a);
        list.push(s.b);
    }
    for (i, s) in nr.segments.iter().enumerate() {
        let Some(layer_pts) = pts_by_layer.get(&s.layer) else {
            continue;
        };
        let ka = pt_key(s.a);
        let kb = pt_key(s.b);
        let tol = s.width * 0.5 + 2.0;
        let min_x = s.a.x.min(s.b.x) - tol;
        let max_x = s.a.x.max(s.b.x) + tol;
        let min_y = s.a.y.min(s.b.y) - tol;
        let max_y = s.a.y.max(s.b.y) + tol;
        for &pt in layer_pts {
            if pt.x < min_x || pt.x > max_x || pt.y < min_y || pt.y > max_y {
                continue;
            }
            let kp = pt_key(pt);
            if kp == ka || kp == kb {
                continue;
            }
            if point_to_segment_dist(pt, s.a, s.b) <= tol {
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

        loop {
            let cur = *seg_indices.last().unwrap();
            let tail = *points.last().unwrap();
            let Some((a, b)) = is_interior(layer, width, tail) else {
                break;
            };
            let next_seg = if a == cur { b } else { a };
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
        loop {
            let cur = seg_indices[0];
            let head = points[0];
            let Some((a, b)) = is_interior(layer, width, head) else {
                break;
            };
            let prev_seg = if a == cur { b } else { a };
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
    sbuf: &mut Vec<usize>,
) -> bool {
    let chains = extract_chains(board, nr);
    let mut remove_mask = vec![false; nr.segments.len()];
    let mut additions = Vec::new();
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
                let (a, b) = (pts[i], pts[j]);
                let direct = RoutedSegment {
                    layer: chain.layer,
                    a,
                    b,
                    width: chain.width,
                };
                if is_octilinear(a, b)
                    && a.dist(b) + 1.0 < sub_len
                    && segment_clear(board, nr.net_id, &direct, index, qbuf, sbuf)
                {
                    chosen = Some((j, None));
                    break;
                }
                if j >= i + 3 || !is_octilinear(a, b) {
                    for mid in octilinear_elbows(a, b) {
                        let s1 = RoutedSegment {
                            layer: chain.layer,
                            a,
                            b: mid,
                            width: chain.width,
                        };
                        let s2 = RoutedSegment {
                            layer: chain.layer,
                            a: mid,
                            b,
                            width: chain.width,
                        };
                        if a.dist(mid) + mid.dist(b) + 5.0 < sub_len
                            && segment_clear(board, nr.net_id, &s1, index, qbuf, sbuf)
                            && segment_clear(board, nr.net_id, &s2, index, qbuf, sbuf)
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
        replace_segments(nr, &remove_mask, additions);
    }
    any_changed
}

/// Moves inter-via chains onto another layer already connected at both ends so redundant vias drop.
fn flatten_via_chains(
    board: &Board,
    nr: &mut NetRoute,
    index: &RouteSpatialIndex,
    qbuf: &mut Vec<usize>,
    sbuf: &mut Vec<usize>,
) -> bool {
    if nr.vias.is_empty() || board.signal_layers.len() < 2 {
        return false;
    }
    let chains = extract_chains(board, nr);
    let mut remove_mask = vec![false; nr.segments.len()];
    let mut additions = Vec::new();
    let mut any_changed = false;

    for chain in chains {
        let start = chain.points[0];
        let end = *chain.points.last().unwrap();
        let start_layers = alt_layers_at(board, nr, start, chain.layer);
        let end_layers = alt_layers_at(board, nr, end, chain.layer);
        if start_layers.is_empty() || end_layers.is_empty() {
            continue;
        }

        let mut replacement: Option<Vec<RoutedSegment>> = None;
        for &cand_l in &board.signal_layers {
            if cand_l == chain.layer
                || !start_layers.contains(&cand_l)
                || !end_layers.contains(&cand_l)
            {
                continue;
            }
            let direct = RoutedSegment {
                layer: cand_l,
                a: start,
                b: end,
                width: chain.width,
            };
            if is_octilinear(start, end)
                && segment_clear(board, nr.net_id, &direct, index, qbuf, sbuf)
            {
                replacement = Some(vec![direct]);
                break;
            }
            for mid in octilinear_elbows(start, end) {
                let s1 = RoutedSegment {
                    layer: cand_l,
                    a: start,
                    b: mid,
                    width: chain.width,
                };
                let s2 = RoutedSegment {
                    layer: cand_l,
                    a: mid,
                    b: end,
                    width: chain.width,
                };
                if segment_clear(board, nr.net_id, &s1, index, qbuf, sbuf)
                    && segment_clear(board, nr.net_id, &s2, index, qbuf, sbuf)
                {
                    replacement = Some(vec![s1, s2]);
                    break;
                }
            }
            if replacement.is_some() {
                break;
            }
            let moved: Vec<RoutedSegment> = chain
                .points
                .windows(2)
                .map(|w| RoutedSegment {
                    layer: cand_l,
                    a: w[0],
                    b: w[1],
                    width: chain.width,
                })
                .collect();
            if moved
                .iter()
                .all(|s| segment_clear(board, nr.net_id, s, index, qbuf, sbuf))
            {
                replacement = Some(moved);
                break;
            }
        }

        if let Some(new_segs) = replacement {
            for &si in &chain.seg_indices {
                remove_mask[si] = true;
            }
            additions.extend(new_segs);
            any_changed = true;
        }
    }

    if any_changed {
        replace_segments(nr, &remove_mask, additions);
    }
    any_changed
}

fn replace_segments(nr: &mut NetRoute, remove_mask: &[bool], additions: Vec<RoutedSegment>) {
    let mut kept = Vec::with_capacity(nr.segments.len());
    for (i, seg) in nr.segments.drain(..).enumerate() {
        if !remove_mask[i] {
            kept.push(seg);
        }
    }
    kept.extend(additions);
    nr.segments = kept;
}

/// Returns true when `via` electrically connects at least two distinct layers it spans.
pub fn via_is_active(
    board: &Board,
    net_id: usize,
    segments: &[RoutedSegment],
    other_vias: &[RoutedVia],
    via: &RoutedVia,
) -> bool {
    let pk = pt_key(via.pos);
    let mut layers = HashSet::new();
    for s in segments {
        if via.layers.contains(&s.layer)
            && (pt_key(s.a) == pk
                || pt_key(s.b) == pk
                || point_to_segment_dist(via.pos, s.a, s.b) <= via.radius + s.width * 0.5 + 1.0)
        {
            layers.insert(s.layer);
        }
    }
    for &pid in &board.nets[net_id].pad_ids {
        let p = &board.pads[pid];
        if p.center.dist(via.pos) > p.max_radius + via.radius + 5.0 {
            continue;
        }
        for &(l, ref sh) in &p.layer_shapes {
            if via.layers.contains(&l) && sh.dist_to_circle(via.pos, via.radius) <= 2.0 {
                layers.insert(l);
            }
        }
    }
    for fs in &board.fixed_segments {
        if fs.net_id == Some(net_id)
            && via.layers.contains(&fs.layer)
            && (pt_key(fs.a) == pk
                || pt_key(fs.b) == pk
                || point_to_segment_dist(via.pos, fs.a, fs.b) <= via.radius + fs.half_w + 1.0)
        {
            layers.insert(fs.layer);
        }
    }
    for fp in &board.planes {
        if fp.net_id == Some(net_id)
            && via.layers.contains(&fp.layer)
            && fp.shape.dist_to_circle(via.pos, via.radius) <= 2.0
        {
            layers.insert(fp.layer);
        }
    }
    for fv in &board.fixed_vias {
        if fv.net_id == Some(net_id) && via.pos.dist(fv.pos) <= via.radius + fv.radius + 1.0 {
            for &l in &fv.layers {
                if via.layers.contains(&l) {
                    layers.insert(l);
                }
            }
        }
    }
    for ov in other_vias {
        if !std::ptr::eq(ov, via) && via.pos.dist(ov.pos) <= via.radius + ov.radius + 1.0 {
            for &l in &ov.layers {
                if via.layers.contains(&l) {
                    layers.insert(l);
                }
            }
        }
    }
    layers.len() >= 2
}

/// Lists layers (excluding `cur_layer`) strictly anchored at vertex `pt` for this net.
fn alt_layers_at(board: &Board, nr: &NetRoute, pt: Point, cur_layer: usize) -> Vec<usize> {
    let pk = pt_key(pt);
    let has_via = nr.vias.iter().any(|v| pt_key(v.pos) == pk);
    let mut layers = HashSet::new();
    for &pid in &board.nets[nr.net_id].pad_ids {
        let p = &board.pads[pid];
        if p.center.dist(pt) > p.max_radius + 5.0 {
            continue;
        }
        for &(l, ref sh) in &p.layer_shapes {
            if l != cur_layer && sh.dist_to_circle(pt, 0.0) <= 2.0 {
                layers.insert(l);
            }
        }
    }
    if has_via {
        for s in &nr.segments {
            if s.layer != cur_layer && (pt_key(s.a) == pk || pt_key(s.b) == pk) {
                layers.insert(s.layer);
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
        for fv in &board.fixed_vias {
            if fv.net_id == Some(nr.net_id) && pt_key(fv.pos) == pk {
                for &l in &fv.layers {
                    if l != cur_layer {
                        layers.insert(l);
                    }
                }
            }
        }
        for fp in &board.planes {
            if fp.net_id == Some(nr.net_id)
                && fp.layer != cur_layer
                && fp.shape.dist_to_circle(pt, 0.0) <= 2.0
            {
                layers.insert(fp.layer);
            }
        }
    }
    layers.into_iter().collect()
}

/// Removes vias that do not actually connect copper on at least two distinct layers.
pub fn prune_dead_vias(board: &Board, nr: &mut NetRoute) {
    let net_id = nr.net_id;
    let segments = &nr.segments;
    loop {
        let before = nr.vias.len();
        let snapshot = nr.vias.clone();
        nr.vias = snapshot
            .iter()
            .filter(|via| via_is_active(board, net_id, segments, &snapshot, via))
            .cloned()
            .collect();
        if nr.vias.len() == before {
            break;
        }
    }
}

fn segment_clear(
    board: &Board,
    net_id: usize,
    cand: &RoutedSegment,
    index: &RouteSpatialIndex,
    qbuf: &mut Vec<usize>,
    sbuf: &mut Vec<usize>,
) -> bool {
    board.check_segment_static_with_buf(net_id, cand.layer, cand.a, cand.b, cand.width, 0.5, sbuf)
        && index.clears_segment(board, net_id, cand, 0.5, qbuf)
}

fn octilinear_elbows(a: Point, b: Point) -> [Point; 2] {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let d = dx.abs().min(dy.abs());
    let (sx, sy) = (dx.signum(), dy.signum());
    [
        Point::new(a.x + sx * d, a.y + sy * d),
        Point::new(b.x - sx * d, b.y - sy * d),
    ]
}

fn is_collinear(a: Point, b: Point, c: Point) -> bool {
    let (ux, uy) = (b.x - a.x, b.y - a.y);
    let (vx, vy) = (c.x - b.x, c.y - b.y);
    (ux * vy - uy * vx).abs() < 1e-2 && (ux * vx + uy * vy) > 0.0
}

fn is_octilinear(a: Point, b: Point) -> bool {
    let dx = (b.x - a.x).abs();
    let dy = (b.y - a.y).abs();
    dx < 1.0 || dy < 1.0 || (dx - dy).abs() < 1.0
}
