use std::collections::HashSet;

use viahop::geometry::{Point, point_to_polygon_edge_dist, segment_to_polygon_edge_dist};
use viahop::optimize::{
    finalize_routes, optimize_routes, prune_dead_vias, snap_pofv_vias, split_segments_at_junctions,
    via_is_active,
};
use viahop::{
    Board, BoardOptions, DsnDesign, NetRoute, RoutedSegment, RoutedVia, RouterConfig, check_drc,
    parse_ses, route, ses, write_ses,
};

fn base_two_layer_dsn() -> &'static str {
    r#"(pcb parity.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 10000 8000))
    (via "Via[0-1]_400:200_um")
    (rule (width 200) (clearance 200))
  )
  (placement
    (component P2
      (place J1 1500 4000 front 0)
      (place J2 8500 4000 front 0)
    )
  )
  (library
    (image P2
      (pin SMD_500 1 0 -1200)
      (pin SMD_500 2 0 1200)
    )
    (padstack SMD_500 (shape (circle F.Cu 500)) (attach off))
    (padstack "Via[0-1]_400:200_um"
      (shape (circle F.Cu 400))
      (shape (circle B.Cu 400))
      (attach off)
    )
  )
  (network
    (net NET_A (pins J1-1 J2-2))
    (net NET_B (pins J1-2 J2-1))
    (class default NET_A NET_B
      (circuit (use_via "Via[0-1]_400:200_um"))
      (rule (width 200) (clearance 200))
    )
  )
  (wiring)
)"#
}

fn make_via(pos: Point) -> RoutedVia {
    RoutedVia {
        pos,
        padstack: "Via[0-1]_400:200_um".to_string(),
        radius: 200.0,
        drill: 200.0,
        layers: vec![0, 1],
    }
}

#[test]
fn hole_to_hole_custom_spacing_enforced_in_drc_and_routing() {
    let mut design = DsnDesign::parse(base_two_layer_dsn()).unwrap();
    design
        .apply_rules_file("(rules (rule (hole_to_hole 350)))")
        .unwrap();
    assert_eq!(design.default_rules.hole_to_hole, Some(350.0));

    // Board::from_dsn picks up .rules hole_to_hole.
    let board_rules = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    assert_eq!(board_rules.hole_to_hole_um, 350.0);

    // Explicit BoardOptions overrides .rules.
    let board_strict = Board::from_dsn_with_options(
        &design,
        &HashSet::new(),
        None,
        BoardOptions {
            hole_to_hole_um: Some(500.0),
            edge_clearance_um: None,
        },
    )
    .unwrap();
    let board_loose = Board::from_dsn_with_options(
        &design,
        &HashSet::new(),
        None,
        BoardOptions {
            hole_to_hole_um: Some(250.0),
            edge_clearance_um: None,
        },
    )
    .unwrap();

    // Two vias with 200um drills spaced 650um c2c -> 450um hole-to-hole gap, 250um copper gap (>200um).
    let same_net_routes = vec![
        NetRoute {
            net_id: 0,
            segments: Vec::new(),
            vias: vec![
                make_via(Point::new(4000.0, 4000.0)),
                make_via(Point::new(4650.0, 4000.0)),
            ],
            unconnected_pairs: 0,
        },
        NetRoute {
            net_id: 1,
            ..Default::default()
        },
    ];
    assert_eq!(
        check_drc(&board_loose, &same_net_routes).violations.len(),
        0
    );
    let strict_same = check_drc(&board_strict, &same_net_routes);
    assert_eq!(strict_same.violations.len(), 1);
    assert_eq!(strict_same.violations[0].kind, "via_hole_same_net");
    assert_eq!(strict_same.violations[0].required_um, 500.0);

    // Other-net pair at 650um c2c also fails at 500um hole-to-hole even though copper gap is 250um > 200um.
    let diff_net_routes = vec![
        NetRoute {
            net_id: 0,
            segments: Vec::new(),
            vias: vec![make_via(Point::new(4000.0, 4000.0))],
            unconnected_pairs: 0,
        },
        NetRoute {
            net_id: 1,
            segments: Vec::new(),
            vias: vec![make_via(Point::new(4650.0, 4000.0))],
            unconnected_pairs: 0,
        },
    ];
    assert_eq!(
        check_drc(&board_loose, &diff_net_routes).violations.len(),
        0
    );
    let strict_diff = check_drc(&board_strict, &diff_net_routes);
    assert_eq!(strict_diff.violations.len(), 1);
    assert_eq!(strict_diff.violations[0].kind, "via_via");
    assert_eq!(strict_diff.violations[0].required_um, 500.0);

    // Full routing with hole_to_hole_um = 600um must keep every via pair >= 600um hole-to-hole.
    let cfg = RouterConfig {
        max_passes: 15,
        hole_to_hole_um: Some(600.0),
        ..Default::default()
    };
    let res = route(&design, &cfg).unwrap();
    assert_eq!(res.board.hole_to_hole_um, 600.0);
    assert_eq!(res.stats.routed_nets, 2);
    assert_eq!(res.stats.drc_violations, 0);
    let all_vias: Vec<&RoutedVia> = res.routes.iter().flat_map(|r| &r.vias).collect();
    for (i, va) in all_vias.iter().enumerate() {
        for vb in &all_vias[i + 1..] {
            let hole_gap = va.pos.dist(vb.pos) - (va.drill + vb.drill) * 0.5;
            assert!(
                hole_gap >= 600.0 - 1e-3,
                "routed vias violated 600um hole-to-hole: gap={hole_gap:.2}"
            );
        }
    }
}

#[test]
fn edge_clearance_enforced_for_traces_vias_and_concave_boundary() {
    // L-shaped concave boundary with an inner re-entrant corner at (5000, 4000).
    let concave_dsn = r#"(pcb concave.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (path pcb 0  0 0  10000 0  10000 4000  5000 4000  5000 8000  0 8000  0 0))
    (via "Via[0-1]_400:200_um")
    (rule (width 200) (clearance 200))
  )
  (placement
    (component P1
      (place J1 2000 6000 front 0)
      (place J2 8000 2000 front 0)
    )
  )
  (library
    (image P1 (pin SMD_500 1 0 0))
    (padstack SMD_500 (shape (circle F.Cu 500)) (attach off))
    (padstack "Via[0-1]_400:200_um"
      (shape (circle F.Cu 400))
      (shape (circle B.Cu 400))
      (attach off)
    )
  )
  (network
    (net SIG (pins J1-1 J2-1))
    (class default SIG (circuit (use_via "Via[0-1]_400:200_um")) (rule (width 200) (clearance 200)))
  )
  (wiring)
)"#;

    // Verify both `(clearance <v> (type edge))` and `(edge_clearance <v>)` in .rules.
    let mut design = DsnDesign::parse(concave_dsn).unwrap();
    design
        .apply_rules_file("(rules (rule (clearance 320 (type edge))))")
        .unwrap();
    assert_eq!(design.default_rules.edge_clearance, Some(320.0));
    design
        .apply_rules_file("(rules (rule (edge_clearance 400)))")
        .unwrap();
    assert_eq!(design.default_rules.edge_clearance, Some(400.0));

    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    assert_eq!(board.edge_clearance_um, Some(400.0));

    // Segment running at y = 450 (half_w=100 -> edge gap=350 < 400) must fail DRC.
    let edge_trace = vec![NetRoute {
        net_id: 0,
        segments: vec![RoutedSegment {
            layer: 0,
            a: Point::new(2000.0, 450.0),
            b: Point::new(7000.0, 450.0),
            width: 200.0,
        }],
        vias: Vec::new(),
        unconnected_pairs: 0,
    }];
    assert_eq!(check_drc(&board, &edge_trace).violations.len(), 1);

    // Via at y = 550 (radius=200 -> edge gap=350 < 400) must fail DRC; at y = 650 passes.
    let bad_via = vec![NetRoute {
        net_id: 0,
        segments: Vec::new(),
        vias: vec![make_via(Point::new(3000.0, 550.0))],
        unconnected_pairs: 0,
    }];
    assert_eq!(check_drc(&board, &bad_via).violations.len(), 1);
    let ok_via = vec![NetRoute {
        net_id: 0,
        segments: Vec::new(),
        vias: vec![make_via(Point::new(3000.0, 650.0))],
        unconnected_pairs: 0,
    }];
    assert_eq!(check_drc(&board, &ok_via).violations.len(), 0);

    // Diagonal segment grazing the concave corner (5000, 4000) in its interior (~41um gap < 400um).
    let corner_graze = vec![NetRoute {
        net_id: 0,
        segments: vec![RoutedSegment {
            layer: 0,
            a: Point::new(3800.0, 5000.0),
            b: Point::new(6000.0, 2800.0),
            width: 200.0,
        }],
        vias: Vec::new(),
        unconnected_pairs: 0,
    }];
    assert_eq!(
        check_drc(&board, &corner_graze).violations.len(),
        1,
        "concave corner graze in segment interior must be caught"
    );

    // Full routing with edge_clearance_um = 600um routes cleanly around the corner and edges.
    let cfg = RouterConfig {
        max_passes: 15,
        edge_clearance_um: Some(600.0),
        ..Default::default()
    };
    let res = route(&design, &cfg).unwrap();
    assert_eq!(res.stats.routed_nets, 1);
    assert_eq!(res.stats.drc_violations, 0);
}

#[test]
fn zero_slack_drc_catches_sub_half_micron_violations() {
    let design = DsnDesign::parse(base_two_layer_dsn()).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();

    // 1. Segment-to-segment: width=200, req=200 -> c2c=400.0 is clean, 399.7 (0.3um short) fails.
    let seg_violation = vec![
        NetRoute {
            net_id: 0,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(3000.0, 4000.0),
                b: Point::new(7000.0, 4000.0),
                width: 200.0,
            }],
            vias: Vec::new(),
            unconnected_pairs: 0,
        },
        NetRoute {
            net_id: 1,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(3000.0, 4399.7),
                b: Point::new(7000.0, 4399.7),
                width: 200.0,
            }],
            vias: Vec::new(),
            unconnected_pairs: 0,
        },
    ];
    let rep = check_drc(&board, &seg_violation);
    assert_eq!(rep.violations.len(), 1);
    assert_eq!(rep.violations[0].kind, "seg_seg");

    let seg_exact = vec![
        seg_violation[0].clone(),
        NetRoute {
            net_id: 1,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(3000.0, 4400.0),
                b: Point::new(7000.0, 4400.0),
                width: 200.0,
            }],
            vias: Vec::new(),
            unconnected_pairs: 0,
        },
    ];
    assert_eq!(check_drc(&board, &seg_exact).violations.len(), 0);

    // 2. Segment-to-pad: J1-2 (NET_B) at (1500, 5200) r=250; req c2c=550.0, so 549.7 fails.
    let seg_pad_violation = vec![
        NetRoute {
            net_id: 0,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(1500.0, 4650.3),
                b: Point::new(3000.0, 4650.3),
                width: 200.0,
            }],
            vias: Vec::new(),
            unconnected_pairs: 0,
        },
        NetRoute {
            net_id: 1,
            ..Default::default()
        },
    ];
    assert_eq!(check_drc(&board, &seg_pad_violation).violations.len(), 1);

    // 3. Via-to-segment: via r=200, seg half_w=100, req=200 -> c2c=500.0; 499.7 fails.
    let via_seg_violation = vec![
        NetRoute {
            net_id: 0,
            segments: Vec::new(),
            vias: vec![make_via(Point::new(5000.0, 4000.0))],
            unconnected_pairs: 0,
        },
        NetRoute {
            net_id: 1,
            segments: vec![RoutedSegment {
                layer: 1,
                a: Point::new(3000.0, 4499.7),
                b: Point::new(7000.0, 4499.7),
                width: 200.0,
            }],
            vias: Vec::new(),
            unconnected_pairs: 0,
        },
    ];
    let rep_vs = check_drc(&board, &via_seg_violation);
    assert_eq!(rep_vs.violations.len(), 1);
    assert_eq!(rep_vs.violations[0].kind, "via_seg");

    // 4. Same-net hole-to-hole: drill=200, hole_to_hole=200 -> c2c=400.0; 399.7 fails.
    let hole_violation = vec![
        NetRoute {
            net_id: 0,
            segments: Vec::new(),
            vias: vec![
                make_via(Point::new(4000.0, 4000.0)),
                make_via(Point::new(4399.7, 4000.0)),
            ],
            unconnected_pairs: 0,
        },
        NetRoute {
            net_id: 1,
            ..Default::default()
        },
    ];
    assert_eq!(check_drc(&board, &hole_violation).violations.len(), 1);
}

#[test]
fn signal_layer_plane_blocks_other_nets_and_allows_same_net() {
    // B.Cu POUR_NET plane x=[3500,6500] y=[1800,6200] blocks OTHER_NET while allowing POUR_NET.
    let plane_dsn = r#"(pcb sig_plane.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 10000 8000))
    (plane POUR_NET (polygon B.Cu 0  3500 1800  6500 1800  6500 6200  3500 6200  3500 1800))
    (via "Via[0-1]_400:200_um")
    (rule (width 200) (clearance 200))
  )
  (placement
    (component P1
      (place A1 1500 4000 back 0)
      (place A2 8500 4000 back 0)
      (place P1 4200 4000 back 0)
      (place P2 5800 4000 back 0)
    )
  )
  (library
    (image P1 (pin SMD_500 1 0 0))
    (padstack SMD_500 (shape (circle F.Cu 500)) (attach off))
    (padstack "Via[0-1]_400:200_um"
      (shape (circle F.Cu 400))
      (shape (circle B.Cu 400))
      (attach off)
    )
  )
  (network
    (net OTHER_NET (pins A1-1 A2-1))
    (net POUR_NET (pins P1-1 P2-1))
    (class default OTHER_NET POUR_NET
      (circuit (use_via "Via[0-1]_400:200_um"))
      (rule (width 200) (clearance 200))
    )
  )
  (wiring)
)"#;

    let design = DsnDesign::parse(plane_dsn).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    assert_eq!(board.planes.len(), 1);
    assert!(board.signal_layers.contains(&1));

    let other_id = board.net_name_to_id["OTHER_NET"];
    let pour_id = board.net_name_to_id["POUR_NET"];

    // Continuous static checks: OTHER_NET trace/via inside B.Cu plane fail; POUR_NET passes.
    assert!(!board.check_segment_static(
        other_id,
        1,
        Point::new(3000.0, 4000.0),
        Point::new(7000.0, 4000.0),
        200.0,
        0.0
    ));
    assert!(!board.check_via_static(other_id, Point::new(5000.0, 3000.0), 0.0));
    assert!(board.check_segment_static(
        other_id,
        0,
        Point::new(3000.0, 4000.0),
        Point::new(7000.0, 4000.0),
        200.0,
        0.0
    ));
    assert!(board.check_segment_static(
        pour_id,
        1,
        Point::new(4200.0, 4000.0),
        Point::new(5800.0, 4000.0),
        200.0,
        0.0
    ));
    assert!(board.check_via_static(pour_id, Point::new(5000.0, 3000.0), 0.0));

    // Full autoroute: both nets route with 0 DRC and OTHER_NET clears the B.Cu plane completely.
    let res = route(
        &design,
        &RouterConfig {
            max_passes: 15,
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(res.stats.routed_nets, 2);
    assert_eq!(res.stats.drc_violations, 0);

    let plane = &res.board.planes[0];
    for seg in &res.routes[other_id].segments {
        if seg.layer == 1 {
            let gap = plane.shape.dist_to_capsule(seg.a, seg.b, seg.width * 0.5);
            assert!(
                gap >= 200.0 - 1e-3,
                "OTHER_NET B.Cu segment violated plane clearance: gap={gap:.2}"
            );
        }
    }
    for via in &res.routes[other_id].vias {
        let gap = plane.shape.dist_to_circle(via.pos, via.radius);
        assert!(
            gap >= 200.0 - 1e-3,
            "OTHER_NET via violated B.Cu plane clearance: gap={gap:.2}"
        );
    }
    assert!(
        !res.routes[pour_id].segments.is_empty(),
        "POUR_NET should route cleanly inside its plane"
    );
}

#[test]
fn pofv_via_snapping_centers_off_center_via_and_rolls_back_on_conflict() {
    let pofv_dsn = r#"(pcb pofv_snap.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 10000 8000))
    (via "Via[0-1]_300:150_um")
    (rule (width 150) (clearance 150))
    (control (via_at_smd on))
  )
  (placement
    (component P1
      (place U1 2000 4000 front 0)
      (place U2 7000 4000 back 0)
      (place B1 2000 6500 back 0)
      (place B2 7000 6500 back 0)
    )
  )
  (library
    (image P1 (pin SMD_500 1 0 0))
    (padstack SMD_500 (shape (circle F.Cu 500)) (attach off))
    (padstack "Via[0-1]_300:150_um"
      (shape (circle F.Cu 300))
      (shape (circle B.Cu 300))
      (attach off)
    )
  )
  (network
    (net SIG_A (pins U1-1 U2-1))
    (net SIG_B (pins B1-1 B2-1))
    (class default SIG_A SIG_B
      (circuit (use_via "Via[0-1]_300:150_um"))
      (rule (width 150) (clearance 150))
    )
  )
  (wiring)
)"#;

    let design = DsnDesign::parse(pofv_dsn).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), Some(true)).unwrap();
    let off_center = Point::new(2090.0, 4040.0);
    let pad_center = Point::new(2000.0, 4000.0);

    // Case 1: Clean snap to exact pad.center, zero-length F.Cu stub dropped, B.Cu reattached.
    let mut routes = vec![
        NetRoute {
            net_id: 0,
            segments: vec![
                RoutedSegment {
                    layer: 0,
                    a: pad_center,
                    b: off_center,
                    width: 150.0,
                },
                RoutedSegment {
                    layer: 1,
                    a: off_center,
                    b: Point::new(7000.0, 4000.0),
                    width: 150.0,
                },
            ],
            vias: vec![RoutedVia {
                pos: off_center,
                padstack: "Via[0-1]_300:150_um".to_string(),
                radius: 150.0,
                drill: 150.0,
                layers: vec![0, 1],
            }],
            unconnected_pairs: 0,
        },
        NetRoute {
            net_id: 1,
            segments: vec![RoutedSegment {
                layer: 1,
                a: Point::new(2000.0, 6500.0),
                b: Point::new(7000.0, 6500.0),
                width: 150.0,
            }],
            vias: Vec::new(),
            unconnected_pairs: 0,
        },
    ];
    snap_pofv_vias(&board, &mut routes);
    assert_eq!(routes[0].vias.len(), 1);
    assert!(
        routes[0].vias[0].pos.dist(pad_center) < 1e-6,
        "via must snap to exact pad center, got {:?}",
        routes[0].vias[0].pos
    );
    assert_eq!(
        routes[0].segments.len(),
        1,
        "zero-length stub must be removed"
    );
    assert!(routes[0].segments[0].a.dist(pad_center) < 1e-6);
    assert_eq!(check_drc(&board, &routes).violations.len(), 0);

    // Case 2: Conflict on B.Cu at x=1650 blocks snapping from x=2090 (gap=215 >= 150) to x=2000 (gap=125 < 150).
    let mut blocked_routes = vec![
        NetRoute {
            net_id: 0,
            segments: vec![
                RoutedSegment {
                    layer: 0,
                    a: pad_center,
                    b: off_center,
                    width: 150.0,
                },
                RoutedSegment {
                    layer: 1,
                    a: off_center,
                    b: Point::new(7000.0, 4000.0),
                    width: 150.0,
                },
            ],
            vias: vec![RoutedVia {
                pos: off_center,
                padstack: "Via[0-1]_300:150_um".to_string(),
                radius: 150.0,
                drill: 150.0,
                layers: vec![0, 1],
            }],
            unconnected_pairs: 0,
        },
        NetRoute {
            net_id: 1,
            segments: vec![RoutedSegment {
                layer: 1,
                a: Point::new(1650.0, 2000.0),
                b: Point::new(1650.0, 6000.0),
                width: 150.0,
            }],
            vias: Vec::new(),
            unconnected_pairs: 0,
        },
    ];
    assert_eq!(check_drc(&board, &blocked_routes).violations.len(), 0);
    snap_pofv_vias(&board, &mut blocked_routes);
    assert!(
        blocked_routes[0].vias[0].pos.dist(off_center) < 1e-6,
        "conflicting snap must roll back"
    );
    assert_eq!(check_drc(&board, &blocked_routes).violations.len(), 0);
}

#[test]
fn ses_export_splits_t_junctions_via_taps_and_prunes_dead_vias() {
    let four_layer_dsn = r#"(pcb hygiene.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer In1.Cu (type power) (property (index 1)))
    (layer B.Cu (type signal) (property (index 2)))
    (boundary (rect pcb 0 0 10000 8000))
    (plane PWR_NET (polygon In1.Cu 0  1000 1000  9000 1000  9000 7000  1000 7000  1000 1000))
    (via "Via[0-2]_400:200_um")
    (rule (width 200) (clearance 200))
    (control (via_at_smd on))
  )
  (placement
    (component P1
      (place U1 1500 3000 front 0)
      (place U2 8500 3000 front 0)
      (place U3 4500 5500 front 0)
      (place U4 6500 1500 back 0)
      (place P1 2000 6000 front 0)
      (place P2 7000 6000 front 0)
    )
  )
  (library
    (image P1 (pin SMD_500 1 0 0))
    (padstack SMD_500 (shape (circle F.Cu 500)) (attach off))
    (padstack "Via[0-2]_400:200_um"
      (shape (circle F.Cu 400))
      (shape (circle In1.Cu 400))
      (shape (circle B.Cu 400))
      (attach off)
    )
  )
  (network
    (net SIG (pins U1-1 U2-1 U3-1 U4-1))
    (net PWR_NET (pins P1-1 P2-1))
    (class default SIG PWR_NET
      (circuit (use_via "Via[0-2]_400:200_um"))
      (rule (width 200) (clearance 200))
    )
  )
  (wiring)
)"#;

    let design = DsnDesign::parse(four_layer_dsn).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), Some(true)).unwrap();
    let via3 = |pos: Point| RoutedVia {
        pos,
        padstack: "Via[0-2]_400:200_um".to_string(),
        radius: 200.0,
        drill: 200.0,
        layers: vec![0, 1, 2],
    };

    let active_via_pos = Point::new(6500.0, 3000.0);
    let dead_via_pos = Point::new(2500.0, 3000.0);
    let plane_via_pos = Point::new(2000.0, 6000.0);

    let routes = vec![
        NetRoute {
            net_id: 0,
            segments: vec![
                // Unsplit trunk spanning through T-junction (4500, 3000) and active via (6500, 3000).
                RoutedSegment {
                    layer: 0,
                    a: Point::new(1500.0, 3000.0),
                    b: Point::new(8500.0, 3000.0),
                    width: 200.0,
                },
                RoutedSegment {
                    layer: 0,
                    a: Point::new(4500.0, 3000.0),
                    b: Point::new(4500.0, 5500.0),
                    width: 200.0,
                },
                RoutedSegment {
                    layer: 2,
                    a: active_via_pos,
                    b: Point::new(6500.0, 1500.0),
                    width: 200.0,
                },
            ],
            vias: vec![via3(active_via_pos), via3(dead_via_pos)],
            unconnected_pairs: 0,
        },
        NetRoute {
            net_id: 1,
            segments: vec![RoutedSegment {
                layer: 0,
                a: plane_via_pos,
                b: Point::new(3000.0, 6000.0),
                width: 200.0,
            }],
            // Touches F.Cu segment + In1.Cu power plane (2 distinct layers) -> must be preserved!
            vias: vec![via3(plane_via_pos)],
            unconnected_pairs: 0,
        },
    ];

    assert!(via_is_active(
        &board,
        0,
        &routes[0].segments,
        &routes[0].vias,
        &routes[0].vias[0]
    ));
    assert!(!via_is_active(
        &board,
        0,
        &routes[0].segments,
        &routes[0].vias,
        &routes[0].vias[1]
    ));
    assert!(via_is_active(
        &board,
        1,
        &routes[1].segments,
        &routes[1].vias,
        &routes[1].vias[0]
    ));

    let ses_text = ses::write_ses(&design, &board, &routes);
    let parsed = parse_ses(&ses_text, &board).unwrap();

    // SIG: dead via pruned, active via preserved, trunk split at (4500, 3000) and (6500, 3000).
    assert_eq!(parsed[0].vias.len(), 1);
    assert!(parsed[0].vias[0].pos.dist(active_via_pos) < 1.0);
    let fcu_segs: Vec<&RoutedSegment> =
        parsed[0].segments.iter().filter(|s| s.layer == 0).collect();
    assert_eq!(
        fcu_segs.len(),
        4,
        "trunk must split into 3 segments plus the tee branch: {fcu_segs:?}"
    );
    for s in &fcu_segs {
        let crosses_tee = s.a.y == 3000.0
            && s.b.y == 3000.0
            && s.a.x.min(s.b.x) < 4499.0
            && s.a.x.max(s.b.x) > 4501.0;
        let crosses_via = s.a.y == 3000.0
            && s.b.y == 3000.0
            && s.a.x.min(s.b.x) < 6499.0
            && s.a.x.max(s.b.x) > 6501.0;
        assert!(!crosses_tee, "segment did not split at T-junction: {s:?}");
        assert!(!crosses_via, "segment did not split at via tap: {s:?}");
    }

    // PWR_NET: power-plane stitching via is preserved.
    assert_eq!(parsed[1].vias.len(), 1);
    assert!(parsed[1].vias[0].pos.dist(plane_via_pos) < 1.0);

    // Also verify split_segments_at_junctions + finalize_routes directly and via route() with optimize=false.
    let split_direct = split_segments_at_junctions(&routes[0].segments, &routes[0].vias);
    assert_eq!(
        split_direct.iter().filter(|s| s.layer == 0).count(),
        5,
        "splits at dead via too before pruning"
    );
    let mut all_mut = routes.clone();
    finalize_routes(&board, &mut all_mut);
    assert_eq!(all_mut[0].vias.len(), 1);

    let res_no_opt = route(
        &design,
        &RouterConfig {
            max_passes: 0,
            optimize: false,
            via_at_smd: Some(true),
            initial_routes: Some(routes),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(
        res_no_opt.routes[0].vias.len(),
        1,
        "dead single-layer via must be pruned even when optimize=false"
    );
    let ses_no_opt = write_ses(&design, &res_no_opt);
    assert!(!ses_no_opt.contains("25000 30000"));
}

#[test]
fn cli_hole_to_hole_and_edge_clearance_flags_and_validation() {
    let bin = env!("CARGO_BIN_EXE_viahop");
    let tmp_dir = std::env::temp_dir();
    let dsn_path = tmp_dir.join("viahop_drc_parity_cli.dsn");
    let rules_path = tmp_dir.join("viahop_drc_parity_cli.rules");
    let init_ses_path = tmp_dir.join("viahop_drc_parity_cli_init.ses");
    let ses_path = tmp_dir.join("viahop_drc_parity_cli.ses");

    let design = DsnDesign::parse(base_two_layer_dsn()).unwrap();
    let loose_board = Board::from_dsn_with_options(
        &design,
        &HashSet::new(),
        None,
        BoardOptions {
            hole_to_hole_um: Some(150.0),
            edge_clearance_um: Some(150.0),
        },
    )
    .unwrap();

    // Initial session passes at 150/150 (trace at y=350 -> 250um edge gap; vias 500um c2c -> 300um hole gap).
    let init_routes = vec![
        NetRoute {
            net_id: 0,
            segments: vec![
                RoutedSegment {
                    layer: 0,
                    a: Point::new(1500.0, 2800.0),
                    b: Point::new(1500.0, 350.0),
                    width: 200.0,
                },
                RoutedSegment {
                    layer: 0,
                    a: Point::new(1500.0, 350.0),
                    b: Point::new(4500.0, 350.0),
                    width: 200.0,
                },
                RoutedSegment {
                    layer: 1,
                    a: Point::new(4500.0, 350.0),
                    b: Point::new(5000.0, 350.0),
                    width: 200.0,
                },
                RoutedSegment {
                    layer: 0,
                    a: Point::new(5000.0, 350.0),
                    b: Point::new(7500.0, 350.0),
                    width: 200.0,
                },
                RoutedSegment {
                    layer: 0,
                    a: Point::new(7500.0, 350.0),
                    b: Point::new(7500.0, 5200.0),
                    width: 200.0,
                },
                RoutedSegment {
                    layer: 0,
                    a: Point::new(7500.0, 5200.0),
                    b: Point::new(8500.0, 5200.0),
                    width: 200.0,
                },
            ],
            vias: vec![
                make_via(Point::new(4500.0, 350.0)),
                make_via(Point::new(5000.0, 350.0)),
            ],
            unconnected_pairs: 0,
        },
        NetRoute {
            net_id: 1,
            segments: vec![
                RoutedSegment {
                    layer: 0,
                    a: Point::new(1500.0, 5200.0),
                    b: Point::new(5000.0, 5200.0),
                    width: 200.0,
                },
                RoutedSegment {
                    layer: 1,
                    a: Point::new(5000.0, 5200.0),
                    b: Point::new(5000.0, 2800.0),
                    width: 200.0,
                },
                RoutedSegment {
                    layer: 1,
                    a: Point::new(5000.0, 2800.0),
                    b: Point::new(8500.0, 2800.0),
                    width: 200.0,
                },
            ],
            vias: vec![
                make_via(Point::new(5000.0, 5200.0)),
                make_via(Point::new(8500.0, 2800.0)),
            ],
            unconnected_pairs: 0,
        },
    ];
    assert_eq!(check_drc(&loose_board, &init_routes).violations.len(), 0);

    std::fs::write(&dsn_path, base_two_layer_dsn()).unwrap();
    std::fs::write(
        &rules_path,
        "(rules (rule (hole_to_hole 150) (edge_clearance 150)))",
    )
    .unwrap();
    std::fs::write(
        &init_ses_path,
        ses::write_ses(&design, &loose_board, &init_routes),
    )
    .unwrap();

    // 1. With .rules (150/150) and -mp 0, the initial session passes with 0 DRC violations.
    let loose_out = std::process::Command::new(bin)
        .args([
            "-i",
            dsn_path.to_str().unwrap(),
            "-o",
            ses_path.to_str().unwrap(),
            "-dr",
            rules_path.to_str().unwrap(),
            "--initial-session",
            init_ses_path.to_str().unwrap(),
            "-mp",
            "0",
            "--json-summary",
        ])
        .output()
        .unwrap();
    assert!(
        loose_out.status.success(),
        "stderr: {}",
        String::from_utf8_lossy(&loose_out.stderr)
    );
    assert!(String::from_utf8_lossy(&loose_out.stdout).contains("\"drc_violations\":0"));

    // 2. Stricter CLI overrides (400/450) beat .rules and flag violations on the same session at -mp 0.
    let strict_check = std::process::Command::new(bin)
        .args([
            "-i",
            dsn_path.to_str().unwrap(),
            "-o",
            ses_path.to_str().unwrap(),
            "-dr",
            rules_path.to_str().unwrap(),
            "--initial-session",
            init_ses_path.to_str().unwrap(),
            "--hole-to-hole",
            "400",
            "--edge-clearance=450",
            "-mp",
            "0",
            "--json-summary",
        ])
        .output()
        .unwrap();
    assert!(strict_check.status.success());
    let strict_check_json = String::from_utf8_lossy(&strict_check.stdout);
    assert!(
        !strict_check_json.contains("\"drc_violations\":0"),
        "CLI overrides must flag violations over looser .rules: {strict_check_json}"
    );

    // 3. Rerouting with CLI overrides (400/450) resolves all violations and emits compliant SES geometry.
    let ok = std::process::Command::new(bin)
        .args([
            "-i",
            dsn_path.to_str().unwrap(),
            "-o",
            ses_path.to_str().unwrap(),
            "-dr",
            rules_path.to_str().unwrap(),
            "--initial-session",
            init_ses_path.to_str().unwrap(),
            "--hole-to-hole",
            "400",
            "--edge-clearance=450",
            "-mp",
            "15",
            "--json-summary",
        ])
        .output()
        .unwrap();
    assert!(
        ok.status.success(),
        "stderr: {}",
        String::from_utf8_lossy(&ok.stderr)
    );
    let stdout = String::from_utf8_lossy(&ok.stdout);
    assert!(stdout.contains("\"routed_nets\":2"));
    assert!(stdout.contains("\"drc_violations\":0"));

    let strict_board = Board::from_dsn_with_options(
        &design,
        &HashSet::new(),
        None,
        BoardOptions {
            hole_to_hole_um: Some(400.0),
            edge_clearance_um: Some(450.0),
        },
    )
    .unwrap();
    let emitted = parse_ses(&std::fs::read_to_string(&ses_path).unwrap(), &strict_board).unwrap();
    assert_eq!(check_drc(&strict_board, &emitted).violations.len(), 0);
    for nr in &emitted {
        for seg in &nr.segments {
            let gap = segment_to_polygon_edge_dist(seg.a, seg.b, &strict_board.boundary)
                - seg.width * 0.5;
            assert!(gap >= 450.0 - 1.0, "emitted seg edge gap {gap:.2} < 450");
        }
    }
    let vias: Vec<&RoutedVia> = emitted.iter().flat_map(|r| &r.vias).collect();
    for (i, va) in vias.iter().enumerate() {
        let edge_gap = point_to_polygon_edge_dist(va.pos, &strict_board.boundary) - va.radius;
        assert!(
            edge_gap >= 450.0 - 1.0,
            "emitted via edge gap {edge_gap:.2} < 450"
        );
        for vb in &vias[i + 1..] {
            let hole_gap = va.pos.dist(vb.pos) - (va.drill + vb.drill) * 0.5;
            assert!(
                hole_gap >= 400.0 - 1.0,
                "emitted hole gap {hole_gap:.2} < 400"
            );
        }
    }

    // Invalid negative or non-finite values must exit with status 2.
    for bad_args in [
        vec![
            "-i",
            dsn_path.to_str().unwrap(),
            "-o",
            ses_path.to_str().unwrap(),
            "--hole-to-hole",
            "-10",
        ],
        vec![
            "-i",
            dsn_path.to_str().unwrap(),
            "-o",
            ses_path.to_str().unwrap(),
            "--edge-clearance",
            "NaN",
        ],
        vec![
            "-i",
            dsn_path.to_str().unwrap(),
            "-o",
            ses_path.to_str().unwrap(),
            "--edge-clearance",
            "inf",
        ],
    ] {
        let out = std::process::Command::new(bin)
            .args(&bad_args)
            .output()
            .unwrap();
        assert_eq!(
            out.status.code(),
            Some(2),
            "expected exit code 2 for {bad_args:?}"
        );
    }

    for p in [&dsn_path, &rules_path, &init_ses_path, &ses_path] {
        let _ = std::fs::remove_file(p);
    }
}

#[test]
fn pofv_snap_guards_centered_vias_external_vias_tjunctions_and_copper_overlap() {
    let dsn_src = r#"(pcb pofv_guards.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 10000 8000))
    (via "Via[0-1]_300:150_um")
    (rule (width 150) (clearance 150))
    (control (via_at_smd on))
  )
  (placement
    (component P1
      (place A1 2000 4000 front 0)
      (place A2 2600 4000 front 0)
      (place R1 6000 4000 front 0)
    )
  )
  (library
    (image P1 (pin SMD_RECT 1 0 0))
    (padstack SMD_RECT (shape (rect F.Cu -400 -100 400 100)) (attach off))
    (padstack "Via[0-1]_300:150_um"
      (shape (circle F.Cu 300))
      (shape (circle B.Cu 300))
      (attach off)
    )
  )
  (network
    (net NET_A (pins A1-1 A2-1 R1-1))
    (class default NET_A (circuit (use_via "Via[0-1]_300:150_um")) (rule (width 150) (clearance 150)))
  )
  (wiring
    (wire (path B.Cu 150 6180 4260 6500 4260) (net NET_A) (type fix))
  )
)"#;

    let design = DsnDesign::parse(dsn_src).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), Some(true)).unwrap();
    let mk_via = |pos: Point| RoutedVia {
        pos,
        padstack: "Via[0-1]_300:150_um".to_string(),
        radius: 150.0,
        drill: 150.0,
        layers: vec![0, 1],
    };

    // 1. Already-centered POFV via on overlapping adjacent same-net SMD pad A1 must NOT yank to A2.
    let mut centered_routes = vec![NetRoute {
        net_id: 0,
        segments: vec![RoutedSegment {
            layer: 1,
            a: Point::new(2000.0, 4000.0),
            b: Point::new(2000.0, 6000.0),
            width: 150.0,
        }],
        vias: vec![mk_via(Point::new(2000.0, 4000.0))],
        unconnected_pairs: 0,
    }];
    snap_pofv_vias(&board, &mut centered_routes);
    assert!(
        centered_routes[0].vias[0]
            .pos
            .dist(Point::new(2000.0, 4000.0))
            < 1e-6,
        "already-centered via must stay at A1, got {:?}",
        centered_routes[0].vias[0].pos
    );

    // 2. External via at (6000, 4220) is outside rect [-400..400, -100..100] even though d=220 <= max_radius (~412).
    let ext_pos = Point::new(6000.0, 4220.0);
    let mut ext_routes = vec![NetRoute {
        net_id: 0,
        segments: vec![RoutedSegment {
            layer: 1,
            a: ext_pos,
            b: Point::new(6000.0, 6000.0),
            width: 150.0,
        }],
        vias: vec![mk_via(ext_pos)],
        unconnected_pairs: 0,
    }];
    snap_pofv_vias(&board, &mut ext_routes);
    assert!(
        ext_routes[0].vias[0].pos.dist(ext_pos) < 1e-6,
        "external via outside pad copper must not snap, got {:?}",
        ext_routes[0].vias[0].pos
    );

    // 3. Off-center via inside R1 touching a fixed B.Cu segment by copper overlap must not snap and disconnect it.
    let overlap_pos = Point::new(6180.0, 4080.0);
    let mut fixed_overlap_routes = vec![NetRoute {
        net_id: 0,
        segments: Vec::new(),
        vias: vec![mk_via(overlap_pos)],
        unconnected_pairs: 0,
    }];
    snap_pofv_vias(&board, &mut fixed_overlap_routes);
    assert!(
        fixed_overlap_routes[0].vias[0].pos.dist(overlap_pos) < 1e-6,
        "via must not snap away from copper-overlapping fixed segment"
    );

    // 4. Snapping an off-center via on an unsplit trunk must split the interior T-junction first.
    let off_pos = Point::new(5900.0, 4080.0);
    let mut tee_routes = vec![NetRoute {
        net_id: 0,
        segments: vec![
            RoutedSegment {
                layer: 1,
                a: Point::new(4000.0, 4080.0),
                b: off_pos,
                width: 150.0,
            },
            RoutedSegment {
                layer: 1,
                a: Point::new(5000.0, 4080.0),
                b: Point::new(5000.0, 6000.0),
                width: 150.0,
            },
        ],
        vias: vec![mk_via(off_pos)],
        unconnected_pairs: 0,
    }];
    snap_pofv_vias(&board, &mut tee_routes);
    assert!(tee_routes[0].vias[0].pos.dist(Point::new(6000.0, 4000.0)) < 1e-6);
    assert!(
        tee_routes[0].segments.iter().any(|s| s.layer == 1
            && (s.a.dist(Point::new(5000.0, 4080.0)) < 1.0
                || s.b.dist(Point::new(5000.0, 4080.0)) < 1.0)
            && s.a.y == 4080.0
            && s.b.y == 4080.0),
        "T-junction at (5000, 4080) must remain anchored after via snap: {:?}",
        tee_routes[0].segments
    );
}

#[test]
fn stacked_and_fixed_vias_retained_and_grazing_trace_not_opened_by_flatten() {
    let dsn_src = r#"(pcb via_hygiene_adv.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer In1.Cu (type signal) (property (index 1)))
    (layer B.Cu (type signal) (property (index 2)))
    (boundary (rect pcb 0 0 10000 8000))
    (via "Via[0-2]_400:200_um")
    (rule (width 200) (clearance 200))
  )
  (placement
    (component P1
      (place U1 1500 3000 front 0)
      (place U2 8500 3000 front 0)
      (place U3 4000 3250 front 0)
      (place U4 6000 3250 front 0)
    )
  )
  (library
    (image P1 (pin SMD_500 1 0 0))
    (padstack SMD_500 (shape (circle F.Cu 500)) (attach off))
    (padstack "Via[0-2]_400:200_um"
      (shape (circle F.Cu 400))
      (shape (circle In1.Cu 400))
      (shape (circle B.Cu 400))
      (attach off)
    )
    (padstack "Via[1-2]_300:150_um"
      (shape (circle In1.Cu 300))
      (shape (circle B.Cu 300))
      (attach off)
    )
  )
  (network
    (net SIG (pins U1-1 U2-1 U3-1 U4-1))
    (class default SIG (circuit (use_via "Via[0-2]_400:200_um")) (rule (width 200) (clearance 200)))
  )
  (wiring
    (via "Via[1-2]_300:150_um" 7000 5000 (net SIG) (type fix))
  )
)"#;

    let design = DsnDesign::parse(dsn_src).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();

    // 1. Stacked blind/buried vias [0,1] + [1,2] and blind [0,1] bridging to fixed [1,2] via are active.
    let mut stacked = NetRoute {
        net_id: 0,
        segments: vec![
            RoutedSegment {
                layer: 0,
                a: Point::new(1500.0, 3000.0),
                b: Point::new(4000.0, 3000.0),
                width: 200.0,
            },
            RoutedSegment {
                layer: 2,
                a: Point::new(4000.0, 3000.0),
                b: Point::new(6000.0, 3000.0),
                width: 200.0,
            },
            RoutedSegment {
                layer: 0,
                a: Point::new(7000.0, 5000.0),
                b: Point::new(8500.0, 5000.0),
                width: 200.0,
            },
        ],
        vias: vec![
            RoutedVia {
                pos: Point::new(4000.0, 3000.0),
                padstack: "Via[0-1]_300:150_um".to_string(),
                radius: 150.0,
                drill: 150.0,
                layers: vec![0, 1],
            },
            RoutedVia {
                pos: Point::new(4000.0, 3000.0),
                padstack: "Via[1-2]_300:150_um".to_string(),
                radius: 150.0,
                drill: 150.0,
                layers: vec![1, 2],
            },
            RoutedVia {
                pos: Point::new(7000.0, 5000.0),
                padstack: "Via[0-1]_300:150_um".to_string(),
                radius: 150.0,
                drill: 150.0,
                layers: vec![0, 1],
            },
        ],
        unconnected_pairs: 0,
    };
    assert!(via_is_active(
        &board,
        0,
        &stacked.segments,
        &stacked.vias,
        &stacked.vias[0]
    ));
    assert!(via_is_active(
        &board,
        0,
        &stacked.segments,
        &stacked.vias,
        &stacked.vias[1]
    ));
    assert!(via_is_active(
        &board,
        0,
        &stacked.segments,
        &stacked.vias,
        &stacked.vias[2]
    ));
    prune_dead_vias(&board, &mut stacked);
    assert_eq!(
        stacked.vias.len(),
        3,
        "stacked blind/buried and fixed-via-bridging vias must all be retained"
    );

    // 2. Grazing F.Cu trace (250um from via center) must not trigger flatten_via_chains into an open gap.
    let mut grazing = vec![NetRoute {
        net_id: 0,
        segments: vec![
            RoutedSegment {
                layer: 0,
                a: Point::new(1500.0, 3000.0),
                b: Point::new(1500.0, 3250.0),
                width: 200.0,
            },
            RoutedSegment {
                layer: 0,
                a: Point::new(1500.0, 3250.0),
                b: Point::new(4000.0, 3250.0),
                width: 200.0,
            },
            RoutedSegment {
                layer: 2,
                a: Point::new(3500.0, 3000.0),
                b: Point::new(6500.0, 3000.0),
                width: 200.0,
            },
            RoutedSegment {
                layer: 0,
                a: Point::new(6000.0, 3250.0),
                b: Point::new(8500.0, 3250.0),
                width: 200.0,
            },
            RoutedSegment {
                layer: 0,
                a: Point::new(8500.0, 3250.0),
                b: Point::new(8500.0, 3000.0),
                width: 200.0,
            },
        ],
        vias: vec![
            RoutedVia {
                pos: Point::new(3500.0, 3000.0),
                padstack: "Via[0-2]_400:200_um".to_string(),
                radius: 200.0,
                drill: 200.0,
                layers: vec![0, 1, 2],
            },
            RoutedVia {
                pos: Point::new(6500.0, 3000.0),
                padstack: "Via[0-2]_400:200_um".to_string(),
                radius: 200.0,
                drill: 200.0,
                layers: vec![0, 1, 2],
            },
        ],
        unconnected_pairs: 0,
    }];
    optimize_routes(&board, &mut grazing);
    assert_eq!(
        grazing[0].vias.len(),
        2,
        "flatten_via_chains must not drop vias bridging merely grazing traces"
    );
    assert!(
        grazing[0].segments.iter().any(|s| s.layer == 2),
        "B.Cu segment must not flatten onto F.Cu with an open gap to grazing traces"
    );
}

#[test]
fn internal_cutout_honors_edge_clearance_400() {
    let cutout_dsn = r#"(pcb cutout.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 10000 8000))
    (keepout "" (rect signal 4500 3500 5500 4500))
    (via "Via[0-1]_400:200_um")
    (rule (width 200) (clearance 200))
  )
  (placement
    (component P1
      (place J1 1500 4000 front 0)
      (place J2 8500 4000 front 0)
    )
  )
  (library
    (image P1 (pin SMD_500 1 0 0))
    (padstack SMD_500 (shape (circle F.Cu 500)) (attach off))
    (padstack "Via[0-1]_400:200_um"
      (shape (circle F.Cu 400))
      (shape (circle B.Cu 400))
      (attach off)
    )
  )
  (network
    (net SIG (pins J1-1 J2-1))
    (class default SIG (circuit (use_via "Via[0-1]_400:200_um")) (rule (width 200) (clearance 200)))
  )
  (wiring)
)"#;

    let design = DsnDesign::parse(cutout_dsn).unwrap();
    let board = Board::from_dsn_with_options(
        &design,
        &HashSet::new(),
        None,
        BoardOptions {
            hole_to_hole_um: None,
            edge_clearance_um: Some(400.0),
        },
    )
    .unwrap();

    // Segment at y=3050 (half_w=100 -> 350um gap to cutout y=3500 < 400) must fail; y=2990 (410um) passes.
    assert!(!board.check_segment_static(
        0,
        0,
        Point::new(3000.0, 3050.0),
        Point::new(7000.0, 3050.0),
        200.0,
        0.0
    ));
    assert!(board.check_segment_static(
        0,
        0,
        Point::new(3000.0, 2990.0),
        Point::new(7000.0, 2990.0),
        200.0,
        0.0
    ));
    // Via at y=2950 (r=200 -> 350um gap < 400) fails; y=2890 (410um) passes.
    assert!(!board.check_via_static(0, Point::new(5000.0, 2950.0), 0.0));
    assert!(board.check_via_static(0, Point::new(5000.0, 2890.0), 0.0));

    let res = route(
        &design,
        &RouterConfig {
            max_passes: 15,
            edge_clearance_um: Some(400.0),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(res.stats.routed_nets, 1);
    assert_eq!(res.stats.drc_violations, 0);
    let cutout = &res.board.keepouts[0];
    for seg in &res.routes[0].segments {
        let gap = cutout.shape.dist_to_capsule(seg.a, seg.b, seg.width * 0.5);
        assert!(
            gap >= 400.0 - 1e-3,
            "routed seg violated 400um cutout clearance: {gap:.2}"
        );
    }
}

#[test]
fn drc_checks_non_default_via_padstack_and_overlapping_same_net_vias() {
    let design = DsnDesign::parse(base_two_layer_dsn()).unwrap();
    let board = Board::from_dsn_with_options(
        &design,
        &HashSet::new(),
        None,
        BoardOptions {
            hole_to_hole_um: Some(200.0),
            edge_clearance_um: Some(300.0),
        },
    )
    .unwrap();

    // 1. Oversized via (r=300, drill=400) at y=550 has 250um edge gap (<300) and must fail DRC.
    let normal_via_route = vec![
        NetRoute {
            net_id: 0,
            segments: Vec::new(),
            vias: vec![make_via(Point::new(5000.0, 550.0))],
            unconnected_pairs: 0,
        },
        NetRoute {
            net_id: 1,
            ..Default::default()
        },
    ];
    assert_eq!(check_drc(&board, &normal_via_route).violations.len(), 0);

    let big_via_route = vec![
        NetRoute {
            net_id: 0,
            segments: Vec::new(),
            vias: vec![RoutedVia {
                pos: Point::new(5000.0, 550.0),
                padstack: "Via[0-1]_600:400_um".to_string(),
                radius: 300.0,
                drill: 400.0,
                layers: vec![0, 1],
            }],
            unconnected_pairs: 0,
        },
        NetRoute {
            net_id: 1,
            ..Default::default()
        },
    ];
    let big_rep = check_drc(&board, &big_via_route);
    assert_eq!(big_rep.violations.len(), 1);
    assert_eq!(big_rep.violations[0].kind, "via_static");

    // 2. Overlapping same-net routed vias (dist = 0.0 and dist = 0.5) must be caught by check_drc.
    for offset in [0.0, 0.5] {
        let dup_vias = vec![
            NetRoute {
                net_id: 0,
                segments: Vec::new(),
                vias: vec![
                    make_via(Point::new(5000.0, 4000.0)),
                    make_via(Point::new(5000.0 + offset, 4000.0)),
                ],
                unconnected_pairs: 0,
            },
            NetRoute {
                net_id: 1,
                ..Default::default()
            },
        ];
        let rep = check_drc(&board, &dup_vias);
        assert_eq!(
            rep.violations.len(),
            1,
            "overlapping same-net vias at offset={offset} must fail DRC"
        );
        assert_eq!(rep.violations[0].kind, "via_hole_same_net");
    }
}

#[test]
fn edge_pad_exit_rejects_legal_pad_graze_and_routes_edge_adjacent_pad() {
    // J1 is legal (pad_edge_dist=450 >= 400); J3 is edge-adjacent (pad_edge_dist=100 < 400).
    let edge_dsn = r#"(pcb edge_pads.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 10000 8000))
    (via "Via[0-1]_400:200_um")
    (rule (width 200) (clearance 200) (edge_clearance 400))
  )
  (placement
    (component P1
      (place J1 2000 700 front 0)
      (place J2 8000 4000 front 0)
      (place J3 4000 350 front 0)
      (place J4 8000 2000 front 0)
    )
  )
  (library
    (image P1 (pin SMD_500 1 0 0))
    (padstack SMD_500 (shape (circle F.Cu 500)) (attach off))
    (padstack "Via[0-1]_400:200_um"
      (shape (circle F.Cu 400))
      (shape (circle B.Cu 400))
      (attach off)
    )
  )
  (network
    (net LEGAL_NET (pins J1-1 J2-1))
    (net EDGE_NET (pins J3-1 J4-1))
    (class default LEGAL_NET EDGE_NET
      (circuit (use_via "Via[0-1]_400:200_um"))
      (rule (width 200) (clearance 200))
    )
  )
  (wiring)
)"#;

    let design = DsnDesign::parse(edge_dsn).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    assert_eq!(board.edge_clearance_um, Some(400.0));

    // Passing Some(0.0) cleanly disables DSN/rules edge clearance.
    let board_disabled = Board::from_dsn_with_options(
        &design,
        &HashSet::new(),
        None,
        BoardOptions {
            hole_to_hole_um: None,
            edge_clearance_um: Some(0.0),
        },
    )
    .unwrap();
    assert_eq!(board_disabled.edge_clearance_um, Some(0.0));

    // 1. Trace touching board-facing edge (y=450) of legal pad J1 extends copper to y=350 (< 400) and must fail.
    let graze_legal = vec![
        NetRoute {
            net_id: 0,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(2000.0, 450.0),
                b: Point::new(2000.0, 700.0),
                width: 200.0,
            }],
            vias: Vec::new(),
            unconnected_pairs: 0,
        },
        NetRoute {
            net_id: 1,
            ..Default::default()
        },
    ];
    assert_eq!(
        check_drc(&board, &graze_legal).violations.len(),
        1,
        "trace grazing board-facing edge of a legal pad must fail DRC"
    );

    // 2. Full routing must route BOTH the legal pad and the edge-adjacent pad cleanly with 0 DRC.
    let res = route(
        &design,
        &RouterConfig {
            max_passes: 15,
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(
        res.stats.routed_nets, 2,
        "edge-adjacent pad must remain routable"
    );
    assert_eq!(res.stats.drc_violations, 0);
}

#[test]
fn multi_class_wide_and_large_drill_nets_route_cleanly() {
    let multi_dsn = r#"(pcb multiclass.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 12000 8000))
    (via "Via[0-1]_300:150_um" "Via[0-1]_600:350_um")
    (rule (width 150) (clearance 150))
  )
  (placement
    (component P2
      (place J1 2000 4000 front 0)
      (place J2 10000 4000 front 0)
    )
  )
  (library
    (image P2
      (pin SMD_500 1 0 -1500)
      (pin SMD_500 2 0 1500)
    )
    (padstack SMD_500 (shape (circle F.Cu 500)) (attach off))
    (padstack "Via[0-1]_300:150_um"
      (shape (circle F.Cu 300))
      (shape (circle B.Cu 300))
      (attach off)
    )
    (padstack "Via[0-1]_600:350_um"
      (shape (circle F.Cu 600))
      (shape (circle B.Cu 600))
      (attach off)
    )
  )
  (network
    (net SIG (pins J1-1 J2-2))
    (net PWR (pins J1-2 J2-1))
    (class signal SIG
      (circuit (use_via "Via[0-1]_300:150_um"))
      (rule (width 150) (clearance 150))
    )
    (class power PWR
      (circuit (use_via "Via[0-1]_600:350_um"))
      (rule (width 400) (clearance 200))
    )
  )
  (wiring)
)"#;

    let design = DsnDesign::parse(multi_dsn).unwrap();
    let res = route(
        &design,
        &RouterConfig {
            max_passes: 20,
            hole_to_hole_um: Some(500.0),
            edge_clearance_um: Some(400.0),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(res.stats.routed_nets, 2);
    assert_eq!(res.stats.drc_violations, 0);
}
