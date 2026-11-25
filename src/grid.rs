//! Multi-layer routing grid, static obstacle rasterization, and PathFinder occupancy footprints.

use std::sync::Arc;

use crate::board::{Board, NetRoute};
use crate::drc::DrcReport;
use crate::fanout::FanoutPlan;
use crate::geometry::{
    point_in_polygon, point_to_polygon_edge_dist, point_to_segment_dist, point_to_segment_dist_sq,
    BBox, Point,
};

pub const STATIC_OPEN: u32 = 0;
pub const STATIC_BLOCKED: u32 = u32::MAX;

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
    pub wire_occ: Vec<u16>,
    pub wire_hist: Vec<f32>,
    pub via_occ: Vec<u16>,
    pub via_hist: Vec<f32>,
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

        let sig_layers = board.signal_layers.clone();
        let num_sig = sig_layers.len();
        let mut board_to_sig = vec![None; board.num_layers()];
        for (sl, &bl) in sig_layers.iter().enumerate() {
            board_to_sig[bl] = Some(sl);
        }

        let n2d = cols * rows;
        let n3d = num_sig * n2d;
        let mut static_wire = vec![STATIC_OPEN; n3d];
        let mut static_via = vec![STATIC_OPEN; n2d];

        let (def_half_w, def_clr, def_via_r, def_via_drill, def_via_smd, def_via_via, def_via_wire) =
            typical_rules(board);

        // 1. Mark cells outside board boundary (with conservative 265um edge clearance).
        if board.boundary.len() >= 3 {
            let edge_clr = def_clr.max(265.0);
            let wire_edge_margin = def_half_w + edge_clr;
            let via_edge_margin = def_via_r + edge_clr;
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

        let bbox_cells = |bb: BBox| -> (usize, usize, usize, usize) {
            let gx0 = (((bb.min_x - origin_x) / step).floor() as isize)
                .clamp(0, (cols - 1) as isize) as usize;
            let gy0 = (((bb.min_y - origin_y) / step).floor() as isize)
                .clamp(0, (rows - 1) as isize) as usize;
            let gx1 = (((bb.max_x - origin_x) / step).ceil() as isize)
                .clamp(0, (cols - 1) as isize) as usize;
            let gy1 = (((bb.max_y - origin_y) / step).ceil() as isize)
                .clamp(0, (rows - 1) as isize) as usize;
            (gx0, gy0, gx1, gy1)
        };

        // 2. Rasterize keepouts (structure cutouts carry extra margin for curved Edge.Cuts clearance).
        for k in &board.keepouts {
            let block_wire = k.kind != crate::dsn::KeepoutKind::Via;
            let block_via = k.kind != crate::dsn::KeepoutKind::Wire;
            let k_margin = if k.is_structure && k.kind == crate::dsn::KeepoutKind::All {
                150.0
            } else {
                10.0
            };
            let (x0, y0, x1, y1) = bbox_cells(k.bbox.expand(def_via_r + k_margin + 30.0));
            for gy in y0..=y1 {
                let y = origin_y + gy as f64 * step;
                for gx in x0..=x1 {
                    let p = Point::new(origin_x + gx as f64 * step, y);
                    let c2 = gy * cols + gx;
                    if block_wire && k.shape.dist_to_circle(p, def_half_w) < k_margin {
                        for (sl, &bl) in sig_layers.iter().enumerate() {
                            if k.applies_to_layer(bl) {
                                static_wire[sl * n2d + c2] = STATIC_BLOCKED;
                            }
                        }
                    }
                    if block_via && k.shape.dist_to_circle(p, def_via_r) < k_margin {
                        static_via[c2] = STATIC_BLOCKED;
                    }
                }
            }
        }

        // 3. Rasterize pads (preserving owner id so same-net traces and POFV vias enter cleanly).
        for pad in &board.pads {
            let owner = pad.net_id.map_or(STATIC_BLOCKED, |id| (id as u32) + 1);
            let wire_clr = pad
                .net_id
                .map_or(def_clr, |id| board.nets[id].rules.clearance.max(def_clr));
            let via_clr = if pad.is_smd {
                pad.net_id
                    .map_or(def_via_smd, |id| board.nets[id].rules.via_smd.max(def_via_smd))
            } else {
                pad.net_id
                    .map_or(def_via_via, |id| board.nets[id].rules.via_via.max(def_via_via))
            };
            let expand = def_via_r + via_clr.max(wire_clr) + def_half_w + 10.0;
            let (x0, y0, x1, y1) = bbox_cells(pad.bbox.expand(expand));
            let allow_pofv_here = pad.is_smd && board.via_at_smd && fanout.pads[pad.id].pofv_allowed;

            for gy in y0..=y1 {
                let y = origin_y + gy as f64 * step;
                for gx in x0..=x1 {
                    let p = Point::new(origin_x + gx as f64 * step, y);
                    let c2 = gy * cols + gx;
                    let mut hits_via = false;
                    for &(bl, ref sh) in &pad.layer_shapes {
                        if let Some(sl) = board_to_sig[bl] {
                            if sh.dist_to_circle(p, def_half_w) < wire_clr {
                                merge_owner(&mut static_wire[sl * n2d + c2], owner);
                            }
                        }
                        if !hits_via && sh.dist_to_circle(p, def_via_r) < via_clr {
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
            let w_clr = fs
                .net_id
                .map_or(def_clr, |id| board.nets[id].rules.clearance.max(def_clr));
            let v_clr = fs
                .net_id
                .map_or(def_via_wire, |id| board.nets[id].rules.via_wire.max(def_via_wire));
            let expand = def_via_r + v_clr.max(w_clr) + def_half_w + 10.0;
            let (x0, y0, x1, y1) = bbox_cells(fs.bbox.expand(expand));
            let sig_opt = board_to_sig[fs.layer];

            for gy in y0..=y1 {
                let y = origin_y + gy as f64 * step;
                for gx in x0..=x1 {
                    let p = Point::new(origin_x + gx as f64 * step, y);
                    let d = point_to_segment_dist(p, fs.a, fs.b) - fs.half_w;
                    let c2 = gy * cols + gx;
                    if let Some(sl) = sig_opt {
                        if d - def_half_w < w_clr {
                            merge_owner(&mut static_wire[sl * n2d + c2], owner);
                        }
                    }
                    if d - def_via_r < v_clr {
                        merge_owner(&mut static_via[c2], owner);
                    }
                }
            }
        }

        // 5. Rasterize fixed vias.
        for fv in &board.fixed_vias {
            let owner = fv.net_id.map_or(STATIC_BLOCKED, |id| (id as u32) + 1);
            let vw_clr = fv
                .net_id
                .map_or(def_via_wire, |id| board.nets[id].rules.via_wire.max(def_via_wire));
            let vv_clr = fv
                .net_id
                .map_or(def_via_via, |id| board.nets[id].rules.via_via.max(def_via_via));
            let hole_c2c = (fv.drill + def_via_drill) * 0.5 + 205.0;
            let expand = (fv.radius + def_via_r + vv_clr.max(vw_clr))
                .max(hole_c2c)
                + 15.0;
            let (x0, y0, x1, y1) = bbox_cells(BBox::from_point(fv.pos, expand));

            for gy in y0..=y1 {
                let y = origin_y + gy as f64 * step;
                for gx in x0..=x1 {
                    let p = Point::new(origin_x + gx as f64 * step, y);
                    let center_d = p.dist(fv.pos);
                    let d = center_d - fv.radius;
                    let c2 = gy * cols + gx;
                    if d - def_half_w < vw_clr {
                        for (sl, &bl) in sig_layers.iter().enumerate() {
                            if fv.layers.contains(&bl) {
                                merge_owner(&mut static_wire[sl * n2d + c2], owner);
                            }
                        }
                    }
                    if d - def_via_r < vv_clr || center_d < hole_c2c {
                        let via_owner = if center_d < 5.0 {
                            owner
                        } else {
                            STATIC_BLOCKED
                        };
                        merge_owner(&mut static_via[c2], via_owner);
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
            wire_occ: vec![0; n3d],
            wire_hist: vec![0.0; n3d],
            via_occ: vec![0; n2d],
            via_hist: vec![0.0; n2d],
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

    #[inline]
    pub fn wire_allowed(&self, sl: usize, c2: usize, net_id: usize) -> bool {
        let v = self.static_wire[sl * self.n2d() + c2];
        v == STATIC_OPEN || v == (net_id as u32) + 1
    }

    #[inline]
    pub fn via_allowed(&self, c2: usize, net_id: usize) -> bool {
        let v = self.static_via[c2];
        v == STATIC_OPEN || v == (net_id as u32) + 1
    }

    pub fn add_footprint(&mut self, board: &Board, route: &NetRoute) -> NetFootprint {
        let rules = &board.nets[route.net_id].rules;
        let wire_r = rules.width + rules.clearance + 4.0;
        let via_wire_r = rules.via_radius + rules.width * 0.5 + rules.via_wire + 4.0;
        let via_via_r = (2.0 * rules.via_radius + rules.via_via)
            .max(rules.via_drill + 205.0)
            + 4.0;
        let n2d = self.n2d();
        let num_sig = self.sig_layers.len();

        let wire_r_sq = wire_r * wire_r;
        let via_wire_r_sq = via_wire_r * via_wire_r;
        let via_via_r_sq = via_via_r * via_via_r;
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

            let bb = BBox::from_point(via.pos, via_via_r.max(via_wire_r));
            let (x0, y0, x1, y1) = self.bbox_cells(bb);
            for gy in y0..=y1 {
                let y = self.y_of(gy);
                let row = gy * self.cols;
                for gx in x0..=x1 {
                    let p = Point::new(self.x_of(gx), y);
                    let d2 = p.dist_sq(via.pos);
                    let c2 = row + gx;
                    if d2 < via_wire_r_sq {
                        for sl in 0..num_sig {
                            if via.layers.contains(&self.sig_layers[sl]) {
                                wire_halo.push((sl * n2d + c2) as u32);
                            }
                        }
                    }
                    if d2 < via_via_r_sq {
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
        fp.wire_centers.iter().any(|&c| self.wire_occ[c as usize] > 1)
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

fn typical_rules(board: &Board) -> (f64, f64, f64, f64, f64, f64, f64) {
    if let Some(n) = board.nets.iter().find(|n| !n.ignored) {
        let r = &n.rules;
        return (
            r.width * 0.5,
            r.clearance,
            r.via_radius,
            r.via_drill,
            r.via_smd,
            r.via_via,
            r.via_wire,
        );
    }
    (50.0, 100.0, 150.0, 180.0, 100.0, 100.0, 100.0)
}
