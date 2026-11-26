//! Exact continuous-geometry DRC verification over routed segments and vias.

use std::collections::HashSet;

use crate::board::{Board, NetRoute, RoutedSegment, RoutedVia, SpatialGrid};
use crate::geometry::{BBox, Point, Shape, segment_to_segment_dist};

const DRC_EPS: f64 = 1e-3;

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
    let mut report = DrcReport::default();

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

    let mut buf = Vec::new();
    let mut static_buf = Vec::new();
    let max_wire_clr = board.max_net_clearance;
    let max_via_clr = board.max_net_clearance.max(board.hole_to_hole_um);

    // 1. Check routed segments against static obstacles & other routed segments.
    for (idx, &(net_a, seg, bb)) in all_segs.iter().enumerate() {
        let half_w = seg.width * 0.5;
        if !board.check_segment_static_with_buf(
            net_a,
            seg.layer,
            seg.a,
            seg.b,
            seg.width,
            0.0,
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

        seg_grid.query(bb.expand(max_wire_clr + 50.0), &mut buf);
        for &j in &buf {
            if j <= idx {
                continue;
            }
            let (net_b, seg_b, _) = all_segs[j];
            if net_a == net_b || seg.layer != seg_b.layer {
                continue;
            }
            let req = board.wire_to_other_clearance(net_a, Some(net_b));
            let gap = segment_to_segment_dist(seg.a, seg.b, seg_b.a, seg_b.b)
                - half_w
                - seg_b.width * 0.5;
            if gap < req - DRC_EPS {
                report.conflicting_nets.insert(net_a);
                report.conflicting_nets.insert(net_b);
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
    }

    // 2. Check routed vias against static obstacles, other routed vias, and other routed segments.
    for (idx, &(net_a, via, bb)) in all_vias.iter().enumerate() {
        if !board.check_routed_via_static_with_buf(
            net_a,
            via.pos,
            via.radius,
            via.drill,
            &via.layers,
            0.0,
            &mut static_buf,
        ) {
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

        let via_query_r = max_via_clr + via.drill * 0.5 + 50.0;
        via_grid.query(bb.expand(via_query_r), &mut buf);
        for &j in &buf {
            if j <= idx {
                continue;
            }
            let (net_b, via_b, _) = all_vias[j];
            if !via.layers.iter().any(|l| via_b.layers.contains(l)) {
                continue;
            }
            let dist = via.pos.dist(via_b.pos);
            let hole_gap = dist - (via.drill + via_b.drill) * 0.5;
            let min_hole_c2c = (via.drill + via_b.drill) * 0.5 + board.hole_to_hole_um;
            if net_a == net_b {
                if dist < min_hole_c2c - DRC_EPS {
                    report.conflicting_nets.insert(net_a);
                    report.violations.push(DrcViolation {
                        net_a,
                        net_b: Some(net_b),
                        layer: None,
                        pos: via.pos,
                        kind: "via_hole_same_net",
                        gap_um: hole_gap,
                        required_um: board.hole_to_hole_um,
                    });
                }
                continue;
            }
            let req = board.via_to_via_clearance(net_a, Some(net_b));
            let gap = dist - via.radius - via_b.radius;
            let copper_bad = gap < req - DRC_EPS;
            let hole_bad = dist < min_hole_c2c - DRC_EPS;
            if copper_bad || hole_bad {
                report.conflicting_nets.insert(net_a);
                report.conflicting_nets.insert(net_b);
                let (gap_um, required_um) = if copper_bad {
                    (gap, req)
                } else {
                    (hole_gap, board.hole_to_hole_um)
                };
                report.violations.push(DrcViolation {
                    net_a,
                    net_b: Some(net_b),
                    layer: None,
                    pos: via.pos,
                    kind: "via_via",
                    gap_um,
                    required_um,
                });
            }
        }

        seg_grid.query(bb.expand(max_via_clr + 50.0), &mut buf);
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
            if gap < req - DRC_EPS {
                report.conflicting_nets.insert(net_a);
                report.conflicting_nets.insert(net_b);
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
