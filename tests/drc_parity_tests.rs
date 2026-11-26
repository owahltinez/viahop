use std::collections::HashSet;

use viahop::geometry::Point;
use viahop::optimize::{finalize_routes, snap_pofv_vias, split_t_junctions, via_is_active};
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
            hole_to_hole_um: 500.0,
            edge_clearance_um: None,
        },
    )
    .unwrap();
    let board_loose = Board::from_dsn_with_options(
        &design,
        &HashSet::new(),
        None,
        BoardOptions {
            hole_to_hole_um: 250.0,
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
    assert_eq!(board.fixed_planes.len(), 1);
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

    let plane = &res.board.fixed_planes[0];
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
        &routes[0].vias[0]
    ));
    assert!(!via_is_active(
        &board,
        0,
        &routes[0].segments,
        &routes[0].vias[1]
    ));
    assert!(via_is_active(
        &board,
        1,
        &routes[1].segments,
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

    // Also verify split_t_junctions + finalize_routes directly and via route() with optimize=false.
    let mut direct = routes[0].clone();
    split_t_junctions(&mut direct);
    assert_eq!(
        direct.segments.iter().filter(|s| s.layer == 0).count(),
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
    let ses_path = tmp_dir.join("viahop_drc_parity_cli.ses");
    std::fs::write(&dsn_path, base_two_layer_dsn()).unwrap();
    std::fs::write(
        &rules_path,
        "(rules (rule (hole_to_hole 150) (edge_clearance 150)))",
    )
    .unwrap();

    // Valid invocation with CLI overrides over .rules.
    let ok = std::process::Command::new(bin)
        .args([
            "-i",
            dsn_path.to_str().unwrap(),
            "-o",
            ses_path.to_str().unwrap(),
            "-dr",
            rules_path.to_str().unwrap(),
            "--hole-to-hole",
            "300",
            "--edge-clearance=350",
            "-mp",
            "10",
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

    for p in [&dsn_path, &rules_path, &ses_path] {
        let _ = std::fs::remove_file(p);
    }
}
