//! Exact continuous-geometry DRC verification over routed segments and vias.

use std::collections::HashSet;

use crate::board::{Board, NetRoute, RoutedSegment, RoutedVia, SpatialGrid};
use crate::geometry::{BBox, Point, Shape, segment_to_segment_dist};

/// Exact continuous-geometry DRC violation.
#[derive(Debug, Clone)]
pub struct DrcViolation {
    pub net_a: usize,
    pub net_b: Option<usize>,
    pub layer: Option<usize>,
    pub pos: Point,
    pub kind: &'static str,
    pub gap_um: f64,
    pub required_um: f64,
}

#[derive(Debug, Clone, Default)]
pub struct DrcReport {
    pub violations: Vec<DrcViolation>,
    pub conflicting_nets: HashSet<usize>,
}

/// Runs an exact continuous-geometry DRC check over all routed segments and vias.
pub fn check_drc(board: &Board, routes: &[NetRoute]) -> DrcReport {
    check_drc_scoped(board, routes, None)
}

/// Runs exact continuous-geometry DRC, restricting checks to violations involving `target_mask`.
pub(crate) fn check_drc_scoped(
    board: &Board,
    routes: &[NetRoute],
    target_mask: Option<&[bool]>,
) -> DrcReport {
    let mut report = DrcReport::default();
    let is_target =
        |nid: usize| -> bool { target_mask.is_none_or(|m| m.get(nid).copied().unwrap_or(false)) };

    let mut all_segs: Vec<(usize, &RoutedSegment, BBox)> = Vec::new();
    let mut all_vias: Vec<(usize, &RoutedVia, BBox)> = Vec::new();
    for nr in routes {
        for seg in &nr.segments {
            let bb = BBox::from_points(&[seg.a, seg.b], seg.width * 0.5);
            all_segs.push((nr.net_id, seg, bb));
        }
        for via in &nr.vias {
            let bb = BBox::from_point(via.pos, via.radius);
            all_vias.push((nr.net_id, via, bb));
        }
    }

    let mut seg_grid = SpatialGrid::new(board.bounds, 1000.0);
    for (i, &(_, _, bb)) in all_segs.iter().enumerate() {
        seg_grid.insert(i, bb);
    }
    let mut via_grid = SpatialGrid::new(board.bounds, 1000.0);
    for (i, &(_, _, bb)) in all_vias.iter().enumerate() {
        via_grid.insert(i, bb);
    }

    let max_board_clr = board
        .nets
        .iter()
        .map(|n| n.rules.clearance)
        .fold(0.0_f64, f64::max);
    let max_board_vclr = board
        .nets
        .iter()
        .map(|n| n.rules.via_via.max(n.rules.via_wire))
        .fold(220.0_f64, f64::max);

    let mut buf = Vec::new();
    let mut static_buf = Vec::new();

    // 1. Check routed segments against static obstacles, other routed segments, and non-target vias.
    for (idx, &(net_a, seg, bb)) in all_segs.iter().enumerate() {
        if !is_target(net_a) {
            continue;
        }
        let half_w = seg.width * 0.5;
        if !board.check_segment_static_with_buf(
            net_a,
            seg.layer,
            seg.a,
            seg.b,
            seg.width,
            -0.5,
            &mut static_buf,
        ) {
            report.conflicting_nets.insert(net_a);
            report.violations.push(DrcViolation {
                net_a,
                net_b: None,
                layer: Some(seg.layer),
                pos: Point::new((seg.a.x + seg.b.x) * 0.5, (seg.a.y + seg.b.y) * 0.5),
                kind: "seg_static",
                gap_um: 0.0,
                required_um: board.nets[net_a].rules.clearance,
            });
        }

        let max_clr = board.nets[net_a].rules.clearance.max(max_board_clr) + 50.0;
        seg_grid.query(bb.expand(max_clr), &mut buf);
        for &j in &buf {
            let (net_b, seg_b, _) = all_segs[j];
            let b_target = is_target(net_b);
            if b_target && j <= idx {
                continue;
            }
            if net_a == net_b || seg.layer != seg_b.layer {
                continue;
            }
            let req = board.wire_to_other_clearance(net_a, Some(net_b));
            let gap = segment_to_segment_dist(seg.a, seg.b, seg_b.a, seg_b.b)
                - half_w
                - seg_b.width * 0.5;
            if gap < req - 0.5 {
                report.conflicting_nets.insert(net_a);
                if b_target {
                    report.conflicting_nets.insert(net_b);
                }
                report.violations.push(DrcViolation {
                    net_a,
                    net_b: Some(net_b),
                    layer: Some(seg.layer),
                    pos: seg.a,
                    kind: "seg_seg",
                    gap_um: gap,
                    required_um: req,
                });
            }
        }

        if target_mask.is_some() {
            let max_vw = board.nets[net_a].rules.via_wire.max(max_board_vclr) + 50.0;
            via_grid.query(bb.expand(max_vw), &mut buf);
            for &j in &buf {
                let (net_b, via_b, _) = all_vias[j];
                if is_target(net_b) || net_a == net_b || !via_b.layers.contains(&seg.layer) {
                    continue;
                }
                let req = board.via_to_wire_clearance(net_b, Some(net_a));
                let sh = Shape::Circle {
                    center: via_b.pos,
                    radius: via_b.radius,
                };
                let gap = sh.dist_to_capsule(seg.a, seg.b, half_w);
                if gap < req - 0.5 {
                    report.conflicting_nets.insert(net_a);
                    report.violations.push(DrcViolation {
                        net_a: net_b,
                        net_b: Some(net_a),
                        layer: Some(seg.layer),
                        pos: via_b.pos,
                        kind: "via_seg",
                        gap_um: gap,
                        required_um: req,
                    });
                }
            }
        }
    }

    // 2. Check routed vias against static obstacles, other routed vias, and other routed segments.
    for (idx, &(net_a, via, bb)) in all_vias.iter().enumerate() {
        if !is_target(net_a) {
            continue;
        }
        if !board.check_via_static_with_buf(net_a, via.pos, -0.5, &mut static_buf) {
            report.conflicting_nets.insert(net_a);
            report.violations.push(DrcViolation {
                net_a,
                net_b: None,
                layer: None,
                pos: via.pos,
                kind: "via_static",
                gap_um: 0.0,
                required_um: board.nets[net_a].rules.via_smd,
            });
        }

        let max_vclr = board.nets[net_a]
            .rules
            .via_via
            .max(board.nets[net_a].rules.via_wire)
            .max(max_board_vclr)
            + 50.0;
        via_grid.query(bb.expand(max_vclr), &mut buf);
        for &j in &buf {
            let (net_b, via_b, _) = all_vias[j];
            let b_target = is_target(net_b);
            if b_target && j <= idx {
                continue;
            }
            if !via.layers.iter().any(|l| via_b.layers.contains(l)) {
                continue;
            }
            let dist = via.pos.dist(via_b.pos);
            let min_hole_c2c = (via.drill + via_b.drill) * 0.5 + 200.0;
            if net_a == net_b {
                if dist >= 1.0 && dist < min_hole_c2c - 0.5 {
                    report.conflicting_nets.insert(net_a);
                    report.violations.push(DrcViolation {
                        net_a,
                        net_b: Some(net_b),
                        layer: None,
                        pos: via.pos,
                        kind: "via_hole_same_net",
                        gap_um: dist - (via.drill + via_b.drill) * 0.5,
                        required_um: 200.0,
                    });
                }
                continue;
            }
            let req = board.via_to_via_clearance(net_a, Some(net_b));
            let gap = dist - via.radius - via_b.radius;
            if gap < req - 0.5 || dist < min_hole_c2c - 0.5 {
                report.conflicting_nets.insert(net_a);
                if b_target {
                    report.conflicting_nets.insert(net_b);
                }
                report.violations.push(DrcViolation {
                    net_a,
                    net_b: Some(net_b),
                    layer: None,
                    pos: via.pos,
                    kind: "via_via",
                    gap_um: gap,
                    required_um: req,
                });
            }
        }

        seg_grid.query(bb.expand(max_vclr), &mut buf);
        for &j in &buf {
            let (net_b, seg_b, _) = all_segs[j];
            if net_a == net_b || !via.layers.contains(&seg_b.layer) {
                continue;
            }
            let req = board.via_to_wire_clearance(net_a, Some(net_b));
            let sh = Shape::Circle {
                center: via.pos,
                radius: via.radius,
            };
            let gap = sh.dist_to_capsule(seg_b.a, seg_b.b, seg_b.width * 0.5);
            if gap < req - 0.5 {
                report.conflicting_nets.insert(net_a);
                if is_target(net_b) {
                    report.conflicting_nets.insert(net_b);
                }
                report.violations.push(DrcViolation {
                    net_a,
                    net_b: Some(net_b),
                    layer: Some(seg_b.layer),
                    pos: via.pos,
                    kind: "via_seg",
                    gap_um: gap,
                    required_um: req,
                });
            }
        }
    }

    report
}
