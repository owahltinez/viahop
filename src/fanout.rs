//! POFV (via-in-pad) detection and SMD breakout candidate analysis.

use crate::board::{Board, Pad};
use crate::geometry::{BBox, Point};

/// Fanout metadata computed per physical pad.
#[derive(Debug, Clone)]
pub struct PadFanout {
    pub pad_id: usize,
    /// True when the SMD pad is surrounded tightly enough (e.g., 0.40/0.50mm BGA)
    /// that surface trace escape is constricted.
    pub is_dense_bga: bool,
    /// True when a centered POFV via on this SMD pad clears all other-net obstacles.
    pub pofv_allowed: bool,
    /// Optional off-center dogbone via position if centered POFV is blocked or disabled.
    pub dogbone_via: Option<Point>,
    /// Nearest other-net pad center distance on the same layer (micrometers).
    pub nearest_neighbor_pitch: f64,
}

#[derive(Debug, Clone, Default)]
pub struct FanoutPlan {
    pub pads: Vec<PadFanout>,
    pub dense_bga_count: usize,
    pub pofv_capable_count: usize,
}

/// Analyzes every pad on the board to identify inner BGA pads and legal POFV / dogbone escapes.
pub fn analyze_fanout(board: &Board) -> FanoutPlan {
    let mut plan = FanoutPlan {
        pads: Vec::with_capacity(board.pads.len()),
        dense_bga_count: 0,
        pofv_capable_count: 0,
    };
    let mut neighbors = Vec::new();
    let mut sbuf = Vec::new();

    for pad in &board.pads {
        let mut info = PadFanout {
            pad_id: pad.id,
            is_dense_bga: false,
            pofv_allowed: false,
            dogbone_via: None,
            nearest_neighbor_pitch: f64::INFINITY,
        };

        let Some(net_id) = pad.net_id else {
            plan.pads.push(info);
            continue;
        };
        if board.nets[net_id].ignored || !pad.is_smd {
            plan.pads.push(info);
            continue;
        }

        let Some(&(pad_layer, _)) = pad.layer_shapes.first() else {
            plan.pads.push(info);
            continue;
        };

        let rules = &board.nets[net_id].rules;
        let search_r = 1200.0;
        let qbox = BBox::from_point(pad.center, search_r);
        board.pad_index.query(qbox, &mut neighbors);

        let mut min_center_dist = f64::INFINITY;
        let mut min_copper_gap = f64::INFINITY;
        let mut tight_quadrants = 0u8;

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
                    tight_quadrants |= 1;
                } else if dx < -50.0 && dx.abs() >= dy.abs() {
                    tight_quadrants |= 2;
                } else if dy > 50.0 && dy.abs() >= dx.abs() {
                    tight_quadrants |= 4;
                } else if dy < -50.0 && dy.abs() >= dx.abs() {
                    tight_quadrants |= 8;
                }
            }
        }

        info.nearest_neighbor_pitch = min_center_dist;
        let trace_channel_needed = rules.width + 2.0 * rules.clearance;
        // Pad-to-pad copper gap cannot pass a trace, or surrounded on >=3 sides at <=0.85mm pitch.
        let cannot_pass_trace = min_copper_gap < trace_channel_needed;
        info.is_dense_bga = (tight_quadrants.count_ones() >= 3 && min_center_dist <= 850.0)
            || (min_center_dist <= 550.0 && cannot_pass_trace);

        if info.is_dense_bga {
            plan.dense_bga_count += 1;
        }

        if board.via_at_smd && board.check_via_static_with_buf(net_id, pad.center, 0.0, &mut sbuf) {
            info.pofv_allowed = true;
            plan.pofv_capable_count += 1;
        } else if info.is_dense_bga {
            // Try 45-degree dogbone escape offsets around the pad if POFV is off or blocked.
            info.dogbone_via =
                find_dogbone_via(board, pad, net_id, pad_layer, min_center_dist, &mut sbuf);
        }

        plan.pads.push(info);
    }

    plan
}

pub(crate) fn dogbone_candidates(center: Point, pitch: f64) -> [Point; 8] {
    let offset = if pitch.is_finite() && pitch > 100.0 {
        pitch * 0.5
    } else {
        350.0
    };
    [
        (1.0, 1.0),
        (-1.0, 1.0),
        (1.0, -1.0),
        (-1.0, -1.0),
        (1.0, 0.0),
        (-1.0, 0.0),
        (0.0, 1.0),
        (0.0, -1.0),
    ]
    .map(|(dx, dy)| Point::new(center.x + dx * offset, center.y + dy * offset))
}

fn find_dogbone_via(
    board: &Board,
    pad: &Pad,
    net_id: usize,
    layer: usize,
    pitch: f64,
    sbuf: &mut Vec<usize>,
) -> Option<Point> {
    let rules = &board.nets[net_id].rules;
    dogbone_candidates(pad.center, pitch)
        .into_iter()
        .find(|&cand| {
            board.check_via_static_with_buf(net_id, cand, 0.0, sbuf)
                && board.check_segment_static_with_buf(
                    net_id,
                    layer,
                    pad.center,
                    cand,
                    rules.width,
                    0.0,
                    sbuf,
                )
        })
}
