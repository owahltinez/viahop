//! Multi-layer routing grid, static obstacle rasterization, and PathFinder occupancy footprints.

use std::sync::Arc;

use crate::board::{Board, NetRoute, NetRules};
use crate::drc::DrcReport;
use crate::fanout::FanoutPlan;
use crate::geometry::{
    BBox, Point, point_in_polygon, point_to_polygon_edge_dist, point_to_segment_dist_sq,
};

pub const STATIC_OPEN: u32 = 0;
pub const STATIC_BLOCKED: u32 = u32::MAX;

struct ExtraMasks<'a> {
    wire_mask: &'a mut [u32],
    via_mask: &'a mut [u32],
    thresholds: &'a [(f64, f64)],
}

#[derive(Clone, Copy)]
struct GridGeom {
    origin_x: f64,
    origin_y: f64,
    step: f64,
    cols: usize,
    rows: usize,
}

impl GridGeom {
    #[inline]
    fn n2d(self) -> usize {
        self.cols * self.rows
    }

    #[inline]
    fn x_of(self, gx: usize) -> f64 {
        self.origin_x + gx as f64 * self.step
    }

    #[inline]
    fn y_of(self, gy: usize) -> f64 {
        self.origin_y + gy as f64 * self.step
    }

    fn bbox_cells(self, bb: BBox) -> (usize, usize, usize, usize) {
        let gx0 = (((bb.min_x - self.origin_x) / self.step).floor() as isize)
            .clamp(0, (self.cols - 1) as isize) as usize;
        let gy0 = (((bb.min_y - self.origin_y) / self.step).floor() as isize)
            .clamp(0, (self.rows - 1) as isize) as usize;
        let gx1 = (((bb.max_x - self.origin_x) / self.step).ceil() as isize)
            .clamp(0, (self.cols - 1) as isize) as usize;
        let gy1 = (((bb.max_y - self.origin_y) / self.step).ceil() as isize)
            .clamp(0, (self.rows - 1) as isize) as usize;
        (gx0, gy0, gx1, gy1)
    }

    #[allow(clippy::too_many_arguments)]
    fn rasterize_segment(
        self,
        board_to_sig: &[Option<usize>],
        static_wire: &mut [u32],
        static_via: &mut [u32],
        mut extra: Option<ExtraMasks<'_>>,
        layer: usize,
        a: Point,
        b: Point,
        owner: u32,
        wire_thresh: f64,
        via_thresh: f64,
    ) {
        let mut max_thresh = wire_thresh.max(via_thresh);
        if let Some(ref ex) = extra {
            for &(wt, vt) in ex.thresholds {
                max_thresh = max_thresh.max(wt).max(vt);
            }
        }
        let (x0, y0, x1, y1) = self.bbox_cells(BBox::from_points(&[a, b], max_thresh + 10.0));
        let sig_opt = board_to_sig[layer];
        let n2d = self.n2d();
        let wire_sq = wire_thresh * wire_thresh;
        let via_sq = via_thresh * via_thresh;

        for gy in y0..=y1 {
            let y = self.y_of(gy);
            let row = gy * self.cols;
            for gx in x0..=x1 {
                let p = Point::new(self.x_of(gx), y);
                let d2 = point_to_segment_dist_sq(p, a, b);
                let c2 = row + gx;
                if let Some(sl) = sig_opt {
                    let idx = sl * n2d + c2;
                    if d2 < wire_sq {
                        merge_owner(&mut static_wire[idx], owner);
                    } else if let Some(ref mut ex) = extra {
                        for (bit, &(wt, _)) in ex.thresholds.iter().enumerate() {
                            if d2 < wt * wt {
                                ex.wire_mask[idx] |= 1u32 << bit;
                            }
                        }
                    }
                }
                if d2 < via_sq {
                    merge_owner(&mut static_via[c2], owner);
                } else if let Some(ref mut ex) = extra {
                    for (bit, &(_, vt)) in ex.thresholds.iter().enumerate() {
                        if d2 < vt * vt {
                            ex.via_mask[c2] |= 1u32 << bit;
                        }
                    }
                }
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn rasterize_via(
        self,
        sig_layers: &[usize],
        static_wire: &mut [u32],
        static_via: &mut [u32],
        mut extra: Option<ExtraMasks<'_>>,
        pos: Point,
        layers: &[usize],
        owner: u32,
        wire_thresh: f64,
        via_thresh: f64,
    ) {
        let mut max_thresh = wire_thresh.max(via_thresh);
        if let Some(ref ex) = extra {
            for &(wt, vt) in ex.thresholds {
                max_thresh = max_thresh.max(wt).max(vt);
            }
        }
        let (x0, y0, x1, y1) = self.bbox_cells(BBox::from_point(pos, max_thresh + 15.0));
        let n2d = self.n2d();
        let wire_sq = wire_thresh * wire_thresh;
        let via_sq = via_thresh * via_thresh;

        for gy in y0..=y1 {
            let y = self.y_of(gy);
            let row = gy * self.cols;
            for gx in x0..=x1 {
                let p = Point::new(self.x_of(gx), y);
                let d2 = p.dist_sq(pos);
                let c2 = row + gx;
                if d2 < wire_sq {
                    for (sl, &bl) in sig_layers.iter().enumerate() {
                        if layers.contains(&bl) {
                            merge_owner(&mut static_wire[sl * n2d + c2], owner);
                        }
                    }
                } else if let Some(ref mut ex) = extra {
                    let mut bits = 0u32;
                    for (bit, &(wt, _)) in ex.thresholds.iter().enumerate() {
                        if d2 < wt * wt {
                            bits |= 1u32 << bit;
                        }
                    }
                    if bits != 0 {
                        for (sl, &bl) in sig_layers.iter().enumerate() {
                            if layers.contains(&bl) {
                                ex.wire_mask[sl * n2d + c2] |= bits;
                            }
                        }
                    }
                }
                if d2 < via_sq {
                    let via_owner = if d2 < 25.0 { owner } else { STATIC_BLOCKED };
                    merge_owner(&mut static_via[c2], via_owner);
                } else if let Some(ref mut ex) = extra {
                    for (bit, &(_, vt)) in ex.thresholds.iter().enumerate() {
                        if d2 < vt * vt {
                            ex.via_mask[c2] |= 1u32 << bit;
                        }
                    }
                }
            }
        }
    }
}

#[derive(Clone, Copy)]
struct ProbeRules {
    half_w: f64,
    clearance: f64,
    via_r: f64,
    via_drill: f64,
    via_via: f64,
    via_wire: f64,
}

impl ProbeRules {
    fn from_rules(r: &NetRules) -> Self {
        Self {
            half_w: r.width * 0.5,
            clearance: r.clearance,
            via_r: r.via_radius,
            via_drill: r.via_drill,
            via_via: r.via_via,
            via_wire: r.via_wire,
        }
    }

    fn approx_eq(self, other: Self) -> bool {
        (self.half_w - other.half_w).abs() <= 1e-6
            && (self.clearance - other.clearance).abs() <= 1e-6
            && (self.via_r - other.via_r).abs() <= 1e-6
            && (self.via_drill - other.via_drill).abs() <= 1e-6
            && (self.via_via - other.via_via).abs() <= 1e-6
            && (self.via_wire - other.via_wire).abs() <= 1e-6
    }

    fn max_with(self, other: Self) -> Self {
        Self {
            half_w: self.half_w.max(other.half_w),
            clearance: self.clearance.max(other.clearance),
            via_r: self.via_r.max(other.via_r),
            via_drill: self.via_drill.max(other.via_drill),
            via_via: self.via_via.max(other.via_via),
            via_wire: self.via_wire.max(other.via_wire),
        }
    }
}

/// Extra per-profile frozen copper block masks when target nets have heterogeneous rules.
struct MixedFrozenMasks {
    net_bit: Vec<u32>,
    wire_mask: Vec<u32>,
    via_mask: Vec<u32>,
}

/// Dynamic grid occupancy footprint recorded per routed net for O(1) rip-up/commit.
#[derive(Clone, Default)]
pub struct NetFootprint {
    pub wire_halo: Vec<u32>,
    pub via_halo: Vec<u32>,
    pub wire_centers: Vec<u32>,
    pub via_centers: Vec<u32>,
}

/// Multi-layer routing grid with PathFinder present + history congestion costs.
#[derive(Clone)]
pub struct RoutingGrid {
    pub origin_x: f64,
    pub origin_y: f64,
    pub step: f64,
    pub cols: usize,
    pub rows: usize,
    pub sig_layers: Vec<usize>,
    /// Maps board layer index -> signal-layer slot (0..num_sig), or None for power layers.
    pub board_to_sig: Vec<Option<usize>>,
    /// Per `(sl, cell2d)`: 0 = open, u32::MAX = blocked, net_id+1 = reserved for net_id.
    pub static_wire: Arc<[u32]>,
    /// Per `cell2d`: 0 = open, u32::MAX = blocked, net_id+1 = reserved for net_id.
    pub static_via: Arc<[u32]>,
    frozen_mixed: Option<Arc<MixedFrozenMasks>>,
    pub wire_occ: Vec<u16>,
    pub wire_hist: Vec<f32>,
    pub via_occ: Vec<u16>,
    pub via_hist: Vec<f32>,
    pub(crate) max_half_w: f64,
    max_via_r: f64,
    max_via_drill: f64,
}

impl RoutingGrid {
    pub fn build(board: &Board, fanout: &FanoutPlan, threads: usize) -> Self {
        let step = choose_grid_step(board);
        let pad_margin = step * 2.0;
        let origin_x = ((board.bounds.min_x - pad_margin) / step).floor() * step;
        let origin_y = ((board.bounds.min_y - pad_margin) / step).floor() * step;
        let max_x = ((board.bounds.max_x + pad_margin) / step).ceil() * step;
        let max_y = ((board.bounds.max_y + pad_margin) / step).ceil() * step;
        let cols = (((max_x - origin_x) / step).round() as usize).max(4) + 1;
        let rows = (((max_y - origin_y) / step).round() as usize).max(4) + 1;
        let geom = GridGeom {
            origin_x,
            origin_y,
            step,
            cols,
            rows,
        };

        let sig_layers = board.signal_layers.clone();
        let num_sig = sig_layers.len();
        let mut board_to_sig = vec![None; board.num_layers()];
        for (sl, &bl) in sig_layers.iter().enumerate() {
            board_to_sig[bl] = Some(sl);
        }

        let n2d = geom.n2d();
        let n3d = num_sig * n2d;
        let mut static_wire = vec![STATIC_OPEN; n3d];
        let mut static_via = vec![STATIC_OPEN; n2d];

        let (def, def_via_smd) = typical_rules(board);
        let (max_half_w, max_via_r, max_via_drill) =
            max_routable_dims(board, def.half_w, def.via_r, def.via_drill);

        // 1. Mark cells outside board boundary (honoring configured edge clearance).
        if board.boundary.len() >= 3 {
            let edge_clr = board.edge_clearance_um.map_or_else(
                || def.clearance.max(265.0),
                |ec| if ec > 0.0 { ec + 5.0 } else { 0.0 },
            );
            let wire_edge_margin = max_half_w + edge_clr;
            let via_edge_margin = max_via_r + edge_clr;
            let mut row_wire_blocked = vec![false; n2d];
            let mut row_via_blocked = vec![false; n2d];
            let worker_count = threads.clamp(1, 8).min(rows.max(1));

            std::thread::scope(|s| {
                let chunk_rows = rows.div_ceil(worker_count);
                let wire_chunks = row_wire_blocked.chunks_mut(chunk_rows * cols);
                let via_chunks = row_via_blocked.chunks_mut(chunk_rows * cols);
                for (ci, (wchunk, vchunk)) in wire_chunks.zip(via_chunks).enumerate() {
                    let start_row = ci * chunk_rows;
                    let boundary = &board.boundary;
                    s.spawn(move || {
                        let local_rows = wchunk.len() / cols;
                        for lr in 0..local_rows {
                            let gy = start_row + lr;
                            let y = origin_y + gy as f64 * step;
                            let base = lr * cols;
                            for gx in 0..cols {
                                let p = Point::new(origin_x + gx as f64 * step, y);
                                if !point_in_polygon(p, boundary) {
                                    wchunk[base + gx] = true;
                                    vchunk[base + gx] = true;
                                } else {
                                    let d = point_to_polygon_edge_dist(p, boundary);
                                    if d < wire_edge_margin {
                                        wchunk[base + gx] = true;
                                    }
                                    if d < via_edge_margin {
                                        vchunk[base + gx] = true;
                                    }
                                }
                            }
                        }
                    });
                }
            });

            for idx in 0..n2d {
                if row_wire_blocked[idx] {
                    for sl in 0..num_sig {
                        static_wire[sl * n2d + idx] = STATIC_BLOCKED;
                    }
                }
                if row_via_blocked[idx] {
                    static_via[idx] = STATIC_BLOCKED;
                }
            }
        }

        // 2. Rasterize keepouts (structure cutouts honor edge clearance + 5um buffer, min 150um).
        let cutout_margin = board.edge_clearance_um.unwrap_or(145.0).max(145.0) + 5.0;
        for k in &board.keepouts {
            let block_wire = k.kind != crate::dsn::KeepoutKind::Via;
            let block_via = k.kind != crate::dsn::KeepoutKind::Wire;
            let k_margin = if k.is_structure && k.kind == crate::dsn::KeepoutKind::All {
                cutout_margin
            } else {
                10.0
            };
            let (x0, y0, x1, y1) = geom.bbox_cells(k.bbox.expand(max_via_r + k_margin + 30.0));
            for gy in y0..=y1 {
                let y = geom.y_of(gy);
                for gx in x0..=x1 {
                    let p = Point::new(geom.x_of(gx), y);
                    let c2 = gy * cols + gx;
                    if block_wire && k.shape.dist_to_circle(p, max_half_w) < k_margin {
                        for (sl, &bl) in sig_layers.iter().enumerate() {
                            if k.applies_to_layer(bl) {
                                static_wire[sl * n2d + c2] = STATIC_BLOCKED;
                            }
                        }
                    }
                    if block_via && k.shape.dist_to_circle(p, max_via_r) < k_margin {
                        static_via[c2] = STATIC_BLOCKED;
                    }
                }
            }
        }

        // 3. Rasterize pads (preserving owner id so same-net traces and POFV vias enter cleanly).
        for pad in &board.pads {
            let owner = pad.net_id.map_or(STATIC_BLOCKED, |id| (id as u32) + 1);
            let wire_clr = pad.net_id.map_or(def.clearance, |id| {
                board.nets[id].rules.clearance.max(def.clearance)
            });
            let via_clr = if pad.is_smd {
                pad.net_id.map_or(def_via_smd, |id| {
                    board.nets[id].rules.via_smd.max(def_via_smd)
                })
            } else {
                pad.net_id.map_or(def.via_via, |id| {
                    board.nets[id].rules.via_via.max(def.via_via)
                })
            };
            let expand = def.via_r + via_clr.max(wire_clr) + def.half_w + 10.0;
            let (x0, y0, x1, y1) = geom.bbox_cells(pad.bbox.expand(expand));
            let allow_pofv_here =
                pad.is_smd && board.via_at_smd && fanout.pads[pad.id].pofv_allowed;

            for gy in y0..=y1 {
                let y = geom.y_of(gy);
                for gx in x0..=x1 {
                    let p = Point::new(geom.x_of(gx), y);
                    let c2 = gy * cols + gx;
                    let mut hits_via = false;
                    for &(bl, ref sh) in &pad.layer_shapes {
                        if let Some(sl) = board_to_sig[bl]
                            && sh.dist_to_circle(p, def.half_w) < wire_clr
                        {
                            merge_owner(&mut static_wire[sl * n2d + c2], owner);
                        }
                        if !hits_via && sh.dist_to_circle(p, def.via_r) < via_clr {
                            hits_via = true;
                        }
                    }
                    if hits_via {
                        let via_owner = if allow_pofv_here || (board.via_at_smd && pad.is_smd) {
                            owner
                        } else {
                            STATIC_BLOCKED
                        };
                        merge_owner(&mut static_via[c2], via_owner);
                    }
                }
            }
        }

        // 4. Rasterize fixed wires.
        for fs in &board.fixed_segments {
            let owner = fs.net_id.map_or(STATIC_BLOCKED, |id| (id as u32) + 1);
            let w_clr = fs.net_id.map_or(def.clearance, |id| {
                board.nets[id].rules.clearance.max(def.clearance)
            });
            let v_clr = fs.net_id.map_or(def.via_wire, |id| {
                board.nets[id].rules.via_wire.max(def.via_wire)
            });
            let wire_thresh = fs.half_w + def.half_w + w_clr;
            let via_thresh = fs.half_w + def.via_r + v_clr;
            geom.rasterize_segment(
                &board_to_sig,
                &mut static_wire,
                &mut static_via,
                None,
                fs.layer,
                fs.a,
                fs.b,
                owner,
                wire_thresh,
                via_thresh,
            );
        }

        // 5. Rasterize fixed vias.
        for fv in &board.fixed_vias {
            let owner = fv.net_id.map_or(STATIC_BLOCKED, |id| (id as u32) + 1);
            let vw_clr = fv.net_id.map_or(def.via_wire, |id| {
                board.nets[id].rules.via_wire.max(def.via_wire)
            });
            let vv_clr = fv.net_id.map_or(def.via_via, |id| {
                board.nets[id].rules.via_via.max(def.via_via)
            });
            let hole_c2c = (fv.drill + max_via_drill) * 0.5 + board.hole_to_hole_um + 5.0;
            let wire_thresh = fv.radius + def.half_w + vw_clr;
            let via_thresh = (fv.radius + max_via_r + vv_clr).max(hole_c2c);
            geom.rasterize_via(
                &sig_layers,
                &mut static_wire,
                &mut static_via,
                None,
                fv.pos,
                &fv.layers,
                owner,
                wire_thresh,
                via_thresh,
            );
        }

        // 6. Rasterize signal-layer planes.
        for fp in &board.planes {
            if board_to_sig[fp.layer].is_none() {
                continue;
            }
            let owner = fp.net_id.map_or(STATIC_BLOCKED, |id| (id as u32) + 1);
            let w_clr = fp.net_id.map_or(def.clearance, |id| {
                board.nets[id].rules.clearance.max(def.clearance)
            });
            let v_clr = fp.net_id.map_or(def.via_wire, |id| {
                board.nets[id].rules.via_wire.max(def.via_wire)
            });
            let expand = def.via_r + v_clr.max(w_clr) + def.half_w + 10.0;
            let (x0, y0, x1, y1) = geom.bbox_cells(fp.bbox.expand(expand));
            let sig_opt = board_to_sig[fp.layer];
            for gy in y0..=y1 {
                let y = geom.y_of(gy);
                for gx in x0..=x1 {
                    let p = Point::new(geom.x_of(gx), y);
                    let d = fp.shape.dist_to_circle(p, 0.0);
                    let c2 = gy * cols + gx;
                    if let Some(sl) = sig_opt
                        && d - def.half_w < w_clr
                    {
                        merge_owner(&mut static_wire[sl * n2d + c2], owner);
                    }
                    if d - def.via_r < v_clr {
                        merge_owner(&mut static_via[c2], owner);
                    }
                }
            }
        }

        Self {
            origin_x,
            origin_y,
            step,
            cols,
            rows,
            sig_layers,
            board_to_sig,
            static_wire: Arc::from(static_wire),
            static_via: Arc::from(static_via),
            frozen_mixed: None,
            wire_occ: vec![0; n3d],
            wire_hist: vec![0.0; n3d],
            via_occ: vec![0; n2d],
            via_hist: vec![0.0; n2d],
            max_half_w,
            max_via_r,
            max_via_drill,
        }
    }

    #[inline]
    fn geom(&self) -> GridGeom {
        GridGeom {
            origin_x: self.origin_x,
            origin_y: self.origin_y,
            step: self.step,
            cols: self.cols,
            rows: self.rows,
        }
    }

    #[inline]
    pub fn n2d(&self) -> usize {
        self.cols * self.rows
    }

    #[inline]
    pub fn x_of(&self, gx: usize) -> f64 {
        self.origin_x + gx as f64 * self.step
    }

    #[inline]
    pub fn y_of(&self, gy: usize) -> f64 {
        self.origin_y + gy as f64 * self.step
    }

    #[inline]
    pub fn pt_of(&self, gx: usize, gy: usize) -> Point {
        Point::new(self.x_of(gx), self.y_of(gy))
    }

    #[inline]
    pub fn snap(&self, p: Point) -> (usize, usize) {
        let gx = ((p.x - self.origin_x) / self.step).round() as isize;
        let gy = ((p.y - self.origin_y) / self.step).round() as isize;
        (
            gx.clamp(0, (self.cols - 1) as isize) as usize,
            gy.clamp(0, (self.rows - 1) as isize) as usize,
        )
    }

    pub fn bbox_cells(&self, bb: BBox) -> (usize, usize, usize, usize) {
        self.geom().bbox_cells(bb)
    }

    #[inline]
    pub fn wire_allowed(&self, sl: usize, c2: usize, net_id: usize) -> bool {
        let idx = sl * self.n2d() + c2;
        let v = self.static_wire[idx];
        if v != STATIC_OPEN && v != (net_id as u32) + 1 {
            return false;
        }
        if let Some(ref mixed) = self.frozen_mixed {
            let bit = mixed.net_bit.get(net_id).copied().unwrap_or(0);
            if bit != 0 && (mixed.wire_mask[idx] & bit) != 0 {
                return false;
            }
        }
        true
    }

    #[inline]
    pub fn via_allowed(&self, c2: usize, net_id: usize) -> bool {
        let v = self.static_via[c2];
        if v != STATIC_OPEN && v != (net_id as u32) + 1 {
            return false;
        }
        if let Some(ref mixed) = self.frozen_mixed {
            let bit = mixed.net_bit.get(net_id).copied().unwrap_or(0);
            if bit != 0 && (mixed.via_mask[c2] & bit) != 0 {
                return false;
            }
        }
        true
    }

    /// Rasterizes non-target frozen routes into static obstacle layers (with per-profile halos when mixed).
    pub(crate) fn rasterize_frozen_routes(
        &mut self,
        board: &Board,
        routes: &[NetRoute],
        target_mask: &[bool],
    ) {
        let frozen_routes: Vec<&NetRoute> = routes
            .iter()
            .filter(|nr| {
                nr.net_id < board.nets.len()
                    && !target_mask.get(nr.net_id).copied().unwrap_or(false)
                    && !board.nets[nr.net_id].ignored
                    && (!nr.segments.is_empty() || !nr.vias.is_empty())
            })
            .collect();
        if frozen_routes.is_empty() {
            return;
        }

        let (min_probe, extra_profiles, net_bit) = target_probe_rules(board, target_mask);
        let geom = self.geom();
        let n2d = geom.n2d();
        let n3d = self.sig_layers.len() * n2d;
        let mut mixed_masks = if extra_profiles.is_empty() {
            None
        } else {
            Some((vec![0u32; n3d], vec![0u32; n2d]))
        };
        let mut prof_thresh = Vec::with_capacity(extra_profiles.len());

        let board_to_sig = &self.board_to_sig;
        let sig_layers = &self.sig_layers;
        let static_wire = unique_arc_slice(&mut self.static_wire);
        let static_via = unique_arc_slice(&mut self.static_via);

        for nr in frozen_routes {
            let owner = (nr.net_id as u32) + 1;
            let rules = &board.nets[nr.net_id].rules;
            let w_clr = rules.clearance.max(min_probe.clearance);
            let vw_clr = rules.via_wire.max(min_probe.via_wire);
            let vv_clr = rules.via_via.max(min_probe.via_via);

            for seg in &nr.segments {
                let half_w = seg.width * 0.5;
                let wire_thresh = half_w + min_probe.half_w + w_clr;
                let via_thresh = half_w + min_probe.via_r + vw_clr;
                prof_thresh.clear();
                for p in &extra_profiles {
                    prof_thresh.push((
                        half_w + p.half_w + rules.clearance.max(p.clearance),
                        half_w + p.via_r + rules.via_wire.max(p.via_wire),
                    ));
                }
                let extra = mixed_masks.as_mut().map(|(wm, vm)| ExtraMasks {
                    wire_mask: wm.as_mut_slice(),
                    via_mask: vm.as_mut_slice(),
                    thresholds: prof_thresh.as_slice(),
                });
                geom.rasterize_segment(
                    board_to_sig,
                    static_wire,
                    static_via,
                    extra,
                    seg.layer,
                    seg.a,
                    seg.b,
                    owner,
                    wire_thresh,
                    via_thresh,
                );
            }

            for via in &nr.vias {
                let hole_c2c =
                    (via.drill + min_probe.via_drill) * 0.5 + board.hole_to_hole_um + 5.0;
                let wire_thresh = via.radius + min_probe.half_w + vw_clr;
                let via_thresh = (via.radius + min_probe.via_r + vv_clr).max(hole_c2c);
                prof_thresh.clear();
                for p in &extra_profiles {
                    let p_hole = (via.drill + p.via_drill) * 0.5 + board.hole_to_hole_um + 5.0;
                    prof_thresh.push((
                        via.radius + p.half_w + rules.via_wire.max(p.via_wire),
                        (via.radius + p.via_r + rules.via_via.max(p.via_via)).max(p_hole),
                    ));
                }
                let extra = mixed_masks.as_mut().map(|(wm, vm)| ExtraMasks {
                    wire_mask: wm.as_mut_slice(),
                    via_mask: vm.as_mut_slice(),
                    thresholds: prof_thresh.as_slice(),
                });
                geom.rasterize_via(
                    sig_layers,
                    static_wire,
                    static_via,
                    extra,
                    via.pos,
                    &via.layers,
                    owner,
                    wire_thresh,
                    via_thresh,
                );
            }
        }

        self.frozen_mixed = mixed_masks.map(|(wire_mask, via_mask)| {
            Arc::new(MixedFrozenMasks {
                net_bit,
                wire_mask,
                via_mask,
            })
        });
    }

    pub fn add_footprint(&mut self, board: &Board, route: &NetRoute) -> NetFootprint {
        let rules = &board.nets[route.net_id].rules;
        let wire_r = rules.width + rules.clearance + 4.0;
        let via_wire_r = rules.via_radius + rules.width * 0.5 + rules.via_wire + 4.0;
        let via_via_r = board
            .min_via_spacing(route.net_id)
            .max(rules.via_radius + self.max_via_r + rules.via_via)
            .max((rules.via_drill + self.max_via_drill) * 0.5 + board.hole_to_hole_um + 5.0)
            + 4.0;
        let n2d = self.n2d();

        let wire_r_sq = wire_r * wire_r;
        let via_wire_r_sq = via_wire_r * via_wire_r;
        let center_r_sq = (self.step * 0.55) * (self.step * 0.55);

        let mut wire_halo = Vec::new();
        let mut via_halo = Vec::new();
        let mut wire_centers = Vec::new();
        let mut via_centers = Vec::new();

        for seg in &route.segments {
            let Some(sl) = self.board_to_sig[seg.layer] else {
                continue;
            };
            let sl_base = sl * n2d;
            let bb = BBox::from_points(&[seg.a, seg.b], wire_r.max(via_wire_r));
            let (x0, y0, x1, y1) = self.bbox_cells(bb);
            for gy in y0..=y1 {
                let y = self.y_of(gy);
                let row = gy * self.cols;
                for gx in x0..=x1 {
                    let p = Point::new(self.x_of(gx), y);
                    let d2 = point_to_segment_dist_sq(p, seg.a, seg.b);
                    let c2 = row + gx;
                    if d2 < wire_r_sq {
                        wire_halo.push((sl_base + c2) as u32);
                    }
                    if d2 < via_wire_r_sq {
                        via_halo.push(c2 as u32);
                    }
                    if d2 <= center_r_sq {
                        wire_centers.push((sl_base + c2) as u32);
                    }
                }
            }
        }

        for via in &route.vias {
            let (vgx, vgy) = self.snap(via.pos);
            via_centers.push((vgy * self.cols + vgx) as u32);

            let v_wire_r = via_wire_r.max(via.radius + rules.width * 0.5 + rules.via_wire + 4.0);
            let v_via_r = via_via_r
                .max(via.radius + self.max_via_r + rules.via_via + 4.0)
                .max((via.drill + self.max_via_drill) * 0.5 + board.hole_to_hole_um + 9.0);
            let v_wire_r_sq = v_wire_r * v_wire_r;
            let v_via_r_sq = v_via_r * v_via_r;
            let bb = BBox::from_point(via.pos, v_via_r.max(v_wire_r));
            let (x0, y0, x1, y1) = self.bbox_cells(bb);
            for gy in y0..=y1 {
                let y = self.y_of(gy);
                let row = gy * self.cols;
                for gx in x0..=x1 {
                    let p = Point::new(self.x_of(gx), y);
                    let d2 = p.dist_sq(via.pos);
                    let c2 = row + gx;
                    if d2 < v_wire_r_sq {
                        for (sl, &bl) in self.sig_layers.iter().enumerate() {
                            if via.layers.contains(&bl) {
                                wire_halo.push((sl * n2d + c2) as u32);
                            }
                        }
                    }
                    if d2 < v_via_r_sq {
                        via_halo.push(c2 as u32);
                    }
                }
            }
        }

        wire_halo.sort_unstable();
        wire_halo.dedup();
        via_halo.sort_unstable();
        via_halo.dedup();
        wire_centers.sort_unstable();
        wire_centers.dedup();
        via_centers.sort_unstable();
        via_centers.dedup();

        for &c in &wire_halo {
            self.wire_occ[c as usize] = self.wire_occ[c as usize].saturating_add(1);
        }
        for &c in &via_halo {
            self.via_occ[c as usize] = self.via_occ[c as usize].saturating_add(1);
        }

        NetFootprint {
            wire_halo,
            via_halo,
            wire_centers,
            via_centers,
        }
    }

    pub fn remove_footprint(&mut self, fp: &NetFootprint) {
        for &c in &fp.wire_halo {
            self.wire_occ[c as usize] = self.wire_occ[c as usize].saturating_sub(1);
        }
        for &c in &fp.via_halo {
            self.via_occ[c as usize] = self.via_occ[c as usize].saturating_sub(1);
        }
    }

    /// Clears dynamic occupancy counters while preserving accumulated congestion history.
    pub fn clear_occupancy(&mut self) {
        self.wire_occ.fill(0);
        self.via_occ.fill(0);
    }

    /// Clears both dynamic occupancy and history (produces a clean grid for diagnosis/restarts).
    pub fn clear_dynamic(&mut self) {
        self.wire_occ.fill(0);
        self.wire_hist.fill(0.0);
        self.via_occ.fill(0);
        self.via_hist.fill(0.0);
    }

    pub fn has_conflict(&self, fp: &NetFootprint) -> bool {
        fp.wire_centers
            .iter()
            .any(|&c| self.wire_occ[c as usize] > 1)
            || fp.via_centers.iter().any(|&c| self.via_occ[c as usize] > 1)
    }

    pub fn bump_history(&mut self, footprints: &[NetFootprint], inc: f32) -> usize {
        let mut congested_nets = 0;
        for fp in footprints {
            let mut hit = false;
            for &c in &fp.wire_centers {
                if self.wire_occ[c as usize] > 1 {
                    self.wire_hist[c as usize] += inc;
                    hit = true;
                }
            }
            for &c in &fp.via_centers {
                if self.via_occ[c as usize] > 1 {
                    self.via_hist[c as usize] += inc;
                    hit = true;
                }
            }
            if hit {
                congested_nets += 1;
            }
        }
        congested_nets
    }

    pub fn bump_drc_history(&mut self, drc: &DrcReport, inc: f32) {
        let n2d = self.n2d();
        for v in &drc.violations {
            let (gx, gy) = self.snap(v.pos);
            for dy in -2_isize..=2 {
                for dx in -2_isize..=2 {
                    let nx = gx as isize + dx;
                    let ny = gy as isize + dy;
                    if nx < 0 || ny < 0 || nx >= self.cols as isize || ny >= self.rows as isize {
                        continue;
                    }
                    let c2 = ny as usize * self.cols + nx as usize;
                    if let Some(bl) = v.layer {
                        if let Some(sl) = self.board_to_sig[bl] {
                            self.wire_hist[sl * n2d + c2] += inc;
                        }
                        if v.kind == "via_seg" {
                            self.via_hist[c2] += inc;
                        }
                    } else {
                        self.via_hist[c2] += inc;
                        for sl in 0..self.sig_layers.len() {
                            self.wire_hist[sl * n2d + c2] += inc * 0.5;
                        }
                    }
                }
            }
        }
    }
}

fn unique_arc_slice(arc: &mut Arc<[u32]>) -> &mut [u32] {
    if Arc::get_mut(arc).is_none() {
        *arc = Arc::from(arc.as_ref());
    }
    Arc::get_mut(arc).expect("uniquely owned after clone")
}

fn merge_owner(slot: &mut u32, owner: u32) {
    if *slot == STATIC_OPEN {
        *slot = owner;
    } else if *slot != owner {
        *slot = STATIC_BLOCKED;
    }
}

fn choose_grid_step(board: &Board) -> f64 {
    let w = (board.bounds.max_x - board.bounds.min_x).max(1000.0);
    let h = (board.bounds.max_y - board.bounds.min_y).max(1000.0);
    let sig = board.signal_layers.len().max(1) as f64;
    for &cand in &[50.0, 100.0, 125.0, 200.0, 250.0, 500.0] {
        if (w / cand) * (h / cand) * sig <= 3_000_000.0 {
            return cand;
        }
    }
    500.0
}

fn max_routable_dims(
    board: &Board,
    def_half_w: f64,
    def_via_r: f64,
    def_via_drill: f64,
) -> (f64, f64, f64) {
    board.nets.iter().filter(|n| !n.ignored).fold(
        (def_half_w, def_via_r, def_via_drill),
        |(hw, vr, vd), n| {
            (
                hw.max(n.rules.width * 0.5),
                vr.max(n.rules.via_radius),
                vd.max(n.rules.via_drill),
            )
        },
    )
}

fn typical_rules(board: &Board) -> (ProbeRules, f64) {
    if let Some(n) = board.nets.iter().find(|n| !n.ignored) {
        return (ProbeRules::from_rules(&n.rules), n.rules.via_smd);
    }
    (
        ProbeRules {
            half_w: 50.0,
            clearance: 100.0,
            via_r: 150.0,
            via_drill: 180.0,
            via_via: 100.0,
            via_wire: 100.0,
        },
        100.0,
    )
}

fn target_probe_rules(
    board: &Board,
    target_mask: &[bool],
) -> (ProbeRules, Vec<ProbeRules>, Vec<u32>) {
    let mut found = false;
    let mut min_p = ProbeRules {
        half_w: f64::INFINITY,
        clearance: f64::INFINITY,
        via_r: f64::INFINITY,
        via_drill: f64::INFINITY,
        via_via: f64::INFINITY,
        via_wire: f64::INFINITY,
    };
    let mut targets: Vec<(usize, ProbeRules)> = Vec::new();
    for n in &board.nets {
        if n.ignored || n.pad_ids.len() < 2 || !target_mask.get(n.id).copied().unwrap_or(false) {
            continue;
        }
        let p = ProbeRules::from_rules(&n.rules);
        found = true;
        min_p.half_w = min_p.half_w.min(p.half_w);
        min_p.clearance = min_p.clearance.min(p.clearance);
        min_p.via_r = min_p.via_r.min(p.via_r);
        min_p.via_drill = min_p.via_drill.min(p.via_drill);
        min_p.via_via = min_p.via_via.min(p.via_via);
        min_p.via_wire = min_p.via_wire.min(p.via_wire);
        targets.push((n.id, p));
    }
    if !found {
        return (typical_rules(board).0, Vec::new(), Vec::new());
    }

    let mut extra_profiles: Vec<ProbeRules> = Vec::new();
    let mut net_bit = vec![0u32; board.nets.len()];
    for (nid, p) in targets {
        if p.approx_eq(min_p) {
            continue;
        }
        let slot = if let Some(idx) = extra_profiles.iter().position(|e| e.approx_eq(p)) {
            idx
        } else if extra_profiles.len() < 32 {
            extra_profiles.push(p);
            extra_profiles.len() - 1
        } else {
            extra_profiles[31] = extra_profiles[31].max_with(p);
            31
        };
        net_bit[nid] = 1u32 << slot;
    }
    (min_p, extra_profiles, net_bit)
}
