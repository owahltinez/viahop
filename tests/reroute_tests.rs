use std::collections::HashSet;

use viahop::geometry::Point;
use viahop::{
    Board, DsnDesign, NetRoute, RoutedSegment, RoutedVia, RouterConfig, check_drc, parse_ses,
    route, write_ses,
};

fn assert_route_exact_eq(actual: &NetRoute, expected: &NetRoute, label: &str) {
    assert_eq!(actual.net_id, expected.net_id, "{label}: net_id mismatch");
    assert_eq!(
        actual.segments.len(),
        expected.segments.len(),
        "{label}: segment count changed"
    );
    for (i, (a, e)) in actual.segments.iter().zip(&expected.segments).enumerate() {
        assert_eq!(a.layer, e.layer, "{label}: seg[{i}].layer changed");
        assert_eq!(
            a.a.x.to_bits(),
            e.a.x.to_bits(),
            "{label}: seg[{i}].a.x changed"
        );
        assert_eq!(
            a.a.y.to_bits(),
            e.a.y.to_bits(),
            "{label}: seg[{i}].a.y changed"
        );
        assert_eq!(
            a.b.x.to_bits(),
            e.b.x.to_bits(),
            "{label}: seg[{i}].b.x changed"
        );
        assert_eq!(
            a.b.y.to_bits(),
            e.b.y.to_bits(),
            "{label}: seg[{i}].b.y changed"
        );
        assert_eq!(
            a.width.to_bits(),
            e.width.to_bits(),
            "{label}: seg[{i}].width changed"
        );
    }
    assert_eq!(
        actual.vias.len(),
        expected.vias.len(),
        "{label}: via count changed"
    );
    for (i, (a, e)) in actual.vias.iter().zip(&expected.vias).enumerate() {
        assert_eq!(
            a.pos.x.to_bits(),
            e.pos.x.to_bits(),
            "{label}: via[{i}].pos.x changed"
        );
        assert_eq!(
            a.pos.y.to_bits(),
            e.pos.y.to_bits(),
            "{label}: via[{i}].pos.y changed"
        );
        assert_eq!(a.padstack, e.padstack, "{label}: via[{i}].padstack changed");
        assert_eq!(
            a.radius.to_bits(),
            e.radius.to_bits(),
            "{label}: via[{i}].radius changed"
        );
        assert_eq!(
            a.drill.to_bits(),
            e.drill.to_bits(),
            "{label}: via[{i}].drill changed"
        );
        assert_eq!(a.layers, e.layers, "{label}: via[{i}].layers changed");
    }
}

fn three_net_dsn() -> &'static str {
    r#"(pcb three_net.dsn
  (parser (string_quote ") (space_in_quoted_tokens on) (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 12000 10000))
    (via "Via[0-1]_400:200_um")
    (rule (width 200) (clearance 200))
  )
  (placement
    (component HDR3
      (place J1 1500 5000 front 0)
      (place J2 10500 5000 front 0)
    )
  )
  (library
    (image HDR3
      (pin SMD_500 1 0 -2500)
      (pin SMD_500 2 0 0)
      (pin SMD_500 3 0 2500)
    )
    (padstack SMD_500 (shape (circle F.Cu 500)) (attach off))
    (padstack "Via[0-1]_400:200_um"
      (shape (circle F.Cu 400))
      (shape (circle B.Cu 400))
      (attach off)
    )
  )
  (network
    (net /Net/A (pins J1-1 J2-1))
    (net /Net/B (pins J1-2 J2-2))
    (net /Net/C (pins J1-3 J2-3))
    (class default /Net/A /Net/B /Net/C
      (circuit (use_via "Via[0-1]_400:200_um"))
      (rule (width 200) (clearance 200))
    )
  )
  (wiring)
)"#
}

fn four_net_repair_dsn() -> &'static str {
    r#"(pcb repair.dsn
  (parser (string_quote ") (space_in_quoted_tokens on) (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 14000 14000))
    (via "Via[0-1]_400:200_um")
    (rule (width 200) (clearance 200))
  )
  (placement
    (component P1
      (place A1 2000 2000 front 0)
      (place A2 8000 5000 front 0)
      (place B1 2000 5000 front 0)
      (place B2 8000 2000 front 0)
      (place X1 2000 10000 front 0)
      (place X2 12000 10000 front 0)
      (place Y1 2000 12000 front 0)
      (place Y2 12000 12000 front 0)
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
    (net /Net/TARGET (pins A1-1 A2-1))
    (net /Net/NEIGHBOR (pins B1-1 B2-1))
    (net /Net/FROZEN_X (pins X1-1 X2-1))
    (net /Net/FROZEN_Y (pins Y1-1 Y2-1))
    (class default /Net/TARGET /Net/NEIGHBOR /Net/FROZEN_X /Net/FROZEN_Y
      (circuit (use_via "Via[0-1]_400:200_um"))
      (rule (width 200) (clearance 200))
    )
  )
  (wiring)
)"#
}

fn detour_wall_dsn() -> &'static str {
    r#"(pcb detour_wall.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 20000 16000))
    (keepout "" (rect signal 9500 3500 10500 12500))
    (via "Via[0-1]_400:200_um")
    (rule (width 200) (clearance 200))
  )
  (placement
    (component P1
      (place T1 6000 8000 front 0)
      (place T2 14000 8000 front 0)
      (place F1 2000 14500 front 0)
      (place F2 6000 14500 front 0)
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
    (net /Net/DETOUR (pins T1-1 T2-1))
    (net /Net/FROZEN (pins F1-1 F2-1))
    (class default /Net/DETOUR /Net/FROZEN
      (circuit (use_via "Via[0-1]_400:200_um"))
      (rule (width 200) (clearance 200))
    )
  )
  (wiring)
)"#
}

fn multi_class_dsn() -> &'static str {
    r#"(pcb multi_class.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 12000 12000))
    (via "Via[0-1]_400:200_um")
    (rule (width 200) (clearance 200))
  )
  (placement
    (component P1
      (place C1 1500 2000 front 0)
      (place C2 10500 2000 front 0)
      (place C3 1500 4500 front 0)
      (place C4 10500 4500 front 0)
      (place I1 1500 7000 front 0)
      (place I2 10500 7000 front 0)
      (place D1 1500 9500 front 0)
      (place D2 10500 9500 front 0)
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
    (net /CLK/A (pins C1-1 C2-1))
    (net /CLK/B (pins C3-1 C4-1))
    (net /CLK/IGN (pins I1-1 I2-1))
    (net /DATA/0 (pins D1-1 D2-1))
    (class clk /CLK/A /CLK/B /CLK/IGN
      (circuit (use_via "Via[0-1]_400:200_um"))
      (rule (width 200) (clearance 200))
    )
    (class data /DATA/0
      (circuit (use_via "Via[0-1]_400:200_um"))
      (rule (width 200) (clearance 200))
    )
  )
  (wiring)
)"#
}

#[test]
fn frozen_nets_remain_verbatim_while_target_reroutes_cleanly() {
    let design = DsnDesign::parse(three_net_dsn()).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    let id_a = board.net_name_to_id["/Net/A"];
    let id_b = board.net_name_to_id["/Net/B"];
    let id_c = board.net_name_to_id["/Net/C"];

    // Build initial routes where /Net/B has an unoptimized staircase and redundant via pair.
    let mut initial = vec![
        NetRoute {
            net_id: id_a,
            ..Default::default()
        },
        NetRoute {
            net_id: id_b,
            segments: vec![
                RoutedSegment {
                    layer: 0,
                    a: Point::new(1500.0, 5000.0),
                    b: Point::new(3500.0, 5000.0),
                    width: 200.0,
                },
                RoutedSegment {
                    layer: 0,
                    a: Point::new(3500.0, 5000.0),
                    b: Point::new(3500.0, 6200.0),
                    width: 200.0,
                },
                RoutedSegment {
                    layer: 0,
                    a: Point::new(3500.0, 6200.0),
                    b: Point::new(5000.0, 6200.0),
                    width: 200.0,
                },
                RoutedSegment {
                    layer: 0,
                    a: Point::new(5000.0, 6200.0),
                    b: Point::new(5000.0, 5000.0),
                    width: 200.0,
                },
                RoutedSegment {
                    layer: 1,
                    a: Point::new(5000.0, 5000.0),
                    b: Point::new(7500.0, 5000.0),
                    width: 200.0,
                },
                RoutedSegment {
                    layer: 0,
                    a: Point::new(7500.0, 5000.0),
                    b: Point::new(10500.0, 5000.0),
                    width: 200.0,
                },
            ],
            vias: vec![
                RoutedVia {
                    pos: Point::new(5000.0, 5000.0),
                    padstack: "Via[0-1]_400:200_um".to_string(),
                    radius: 200.0,
                    drill: 200.0,
                    layers: vec![0, 1],
                },
                RoutedVia {
                    pos: Point::new(7500.0, 5000.0),
                    padstack: "Via[0-1]_400:200_um".to_string(),
                    radius: 200.0,
                    drill: 200.0,
                    layers: vec![0, 1],
                },
            ],
            unconnected_pairs: 0,
        },
        NetRoute {
            net_id: id_c,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(1500.0, 7500.0),
                b: Point::new(10500.0, 7500.0),
                width: 200.0,
            }],
            vias: Vec::new(),
            unconnected_pairs: 0,
        },
    ];
    initial.sort_by_key(|r| r.net_id);
    let frozen_b = initial[id_b].clone();
    let frozen_c = initial[id_c].clone();

    let cfg = RouterConfig {
        max_passes: 15,
        optimize: true,
        initial_routes: Some(initial),
        reroute_nets: Some(HashSet::from(["/Net/A".to_string()])),
        reroute_pad_um: Some(3000.0),
        ..Default::default()
    };
    let res = route(&design, &cfg).unwrap();

    assert_eq!(res.stats.routable_nets, 3);
    assert_eq!(res.stats.routed_nets, 3);
    assert_eq!(res.stats.unconnected_pairs, 0);
    assert_eq!(res.stats.drc_violations, 0);
    assert!(!res.routes[id_a].segments.is_empty());
    assert_route_exact_eq(&res.routes[id_b], &frozen_b, "/Net/B frozen");
    assert_route_exact_eq(&res.routes[id_c], &frozen_c, "/Net/C frozen");
}

#[test]
fn repairs_drc_violating_net_without_moving_clean_neighbors() {
    let design = DsnDesign::parse(four_net_repair_dsn()).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    let id_target = board.net_name_to_id["/Net/TARGET"];
    let id_neighbor = board.net_name_to_id["/Net/NEIGHBOR"];
    let id_fx = board.net_name_to_id["/Net/FROZEN_X"];
    let id_fy = board.net_name_to_id["/Net/FROZEN_Y"];

    // 1. Pure repair: /Net/TARGET shorts directly across clean /Net/NEIGHBOR on F.Cu.
    let mut clean_base = vec![
        NetRoute {
            net_id: id_target,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(2000.0, 2000.0),
                b: Point::new(8000.0, 5000.0),
                width: 200.0,
            }],
            vias: Vec::new(),
            unconnected_pairs: 0,
        },
        NetRoute {
            net_id: id_neighbor,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(2000.0, 5000.0),
                b: Point::new(8000.0, 2000.0),
                width: 200.0,
            }],
            vias: Vec::new(),
            unconnected_pairs: 0,
        },
        NetRoute {
            net_id: id_fx,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(2000.0, 10000.0),
                b: Point::new(12000.0, 10000.0),
                width: 200.0,
            }],
            vias: Vec::new(),
            unconnected_pairs: 0,
        },
        NetRoute {
            net_id: id_fy,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(2000.0, 12000.0),
                b: Point::new(12000.0, 12000.0),
                width: 200.0,
            }],
            vias: Vec::new(),
            unconnected_pairs: 0,
        },
    ];
    clean_base.sort_by_key(|r| r.net_id);
    assert!(
        !check_drc(&board, &clean_base).violations.is_empty(),
        "initial session must contain intentional TARGET/NEIGHBOR short"
    );
    let frozen_neighbor = clean_base[id_neighbor].clone();

    let repair_cfg = RouterConfig {
        max_passes: 15,
        initial_routes: Some(clean_base.clone()),
        reroute_nets: Some(HashSet::from(["/Net/TARGET".to_string()])),
        ..Default::default()
    };
    let repaired = route(&design, &repair_cfg).unwrap();
    assert_eq!(repaired.stats.routed_nets, 4);
    assert_eq!(repaired.stats.unconnected_pairs, 0);
    assert_eq!(repaired.stats.drc_violations, 0);
    assert_route_exact_eq(
        &repaired.routes[id_neighbor],
        &frozen_neighbor,
        "/Net/NEIGHBOR must stay untouched",
    );

    // 2. Pre-existing DRC between two frozen nets must not trigger multi-start, yet stats stay honest.
    let mut with_frozen_drc = clean_base;
    with_frozen_drc[id_fy].segments = vec![
        RoutedSegment {
            layer: 0,
            a: Point::new(2000.0, 12000.0),
            b: Point::new(7000.0, 10000.0),
            width: 200.0,
        },
        RoutedSegment {
            layer: 0,
            a: Point::new(7000.0, 10000.0),
            b: Point::new(12000.0, 12000.0),
            width: 200.0,
        },
    ];
    let frozen_fx = with_frozen_drc[id_fx].clone();
    let frozen_fy = with_frozen_drc[id_fy].clone();

    let scoped_cfg = RouterConfig {
        max_passes: 15,
        multi_start: 4,
        threads: 2,
        initial_routes: Some(with_frozen_drc),
        reroute_nets: Some(HashSet::from(["/Net/TARGET".to_string()])),
        ..Default::default()
    };
    let scoped = route(&design, &scoped_cfg).unwrap();
    assert_eq!(
        scoped.stats.starts_run, 1,
        "unrelated frozen-vs-frozen DRC must not force extra multi-start runs"
    );
    assert_eq!(scoped.stats.unconnected_pairs, 0);
    assert!(
        scoped.stats.drc_violations >= 1,
        "final stats must honestly report pre-existing full-board DRC"
    );
    let full_drc = check_drc(&scoped.board, &scoped.routes);
    assert!(
        !full_drc.conflicting_nets.contains(&id_target),
        "target net must have 0 DRC violations after repair"
    );
    assert_route_exact_eq(&scoped.routes[id_neighbor], &frozen_neighbor, "NEIGHBOR");
    assert_route_exact_eq(&scoped.routes[id_fx], &frozen_fx, "FROZEN_X");
    assert_route_exact_eq(&scoped.routes[id_fy], &frozen_fy, "FROZEN_Y");
}

#[test]
fn local_bbox_clipping_falls_back_when_detour_leaves_pad_box() {
    let design = DsnDesign::parse(detour_wall_dsn()).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    let id_detour = board.net_name_to_id["/Net/DETOUR"];
    let id_frozen = board.net_name_to_id["/Net/FROZEN"];

    let mut initial = vec![
        NetRoute {
            net_id: id_detour,
            ..Default::default()
        },
        NetRoute {
            net_id: id_frozen,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(2000.0, 14500.0),
                b: Point::new(6000.0, 14500.0),
                width: 200.0,
            }],
            vias: Vec::new(),
            unconnected_pairs: 0,
        },
    ];
    initial.sort_by_key(|r| r.net_id);
    let frozen_orig = initial[id_frozen].clone();

    // Pad Y is 8000; 1000um pad box covers Y in [6750, 9250], fully blocked by wall [3500, 12500].
    let cfg = RouterConfig {
        max_passes: 15,
        initial_routes: Some(initial),
        reroute_nets: Some(HashSet::from(["/Net/DETOUR".to_string()])),
        reroute_pad_um: Some(1000.0),
        ..Default::default()
    };
    let res = route(&design, &cfg).unwrap();

    assert_eq!(res.stats.routed_nets, 2);
    assert_eq!(res.stats.unconnected_pairs, 0);
    assert_eq!(res.stats.drc_violations, 0);
    assert_route_exact_eq(&res.routes[id_frozen], &frozen_orig, "/Net/FROZEN");
    let left_box = res.routes[id_detour]
        .segments
        .iter()
        .any(|s| s.a.y < 3500.0 || s.b.y < 3500.0 || s.a.y > 12500.0 || s.b.y > 12500.0);
    assert!(
        left_box,
        "fallback must route outside initial 1mm pad box to clear keepout wall"
    );
}

#[test]
fn reroute_matches_by_net_class_and_supports_no_initial_routes() {
    let design = DsnDesign::parse(multi_class_dsn()).unwrap();
    let ignored = HashSet::from(["/CLK/IGN".to_string()]);
    let board = Board::from_dsn(&design, &ignored, None).unwrap();
    let id_clk_a = board.net_name_to_id["/CLK/A"];
    let id_clk_b = board.net_name_to_id["/CLK/B"];
    let id_clk_ign = board.net_name_to_id["/CLK/IGN"];
    let id_data = board.net_name_to_id["/DATA/0"];

    // 1. Without initial_routes, reroute_nets by class routes only non-ignored nets in that class.
    let partial_cfg = RouterConfig {
        max_passes: 10,
        ignored_nets: ignored.clone(),
        reroute_nets: Some(HashSet::from(["clk".to_string()])),
        reroute_pad_um: Some(2000.0),
        ..Default::default()
    };
    let partial = route(&design, &partial_cfg).unwrap();
    assert_eq!(partial.stats.routable_nets, 3);
    assert_eq!(partial.stats.routed_nets, 2);
    assert_eq!(partial.stats.unconnected_pairs, 1);
    assert_eq!(partial.stats.drc_violations, 0);
    assert_eq!(partial.stats.starts_run, 1);
    assert!(!partial.routes[id_clk_a].segments.is_empty());
    assert!(!partial.routes[id_clk_b].segments.is_empty());
    assert!(partial.routes[id_clk_ign].segments.is_empty());
    assert!(partial.routes[id_data].segments.is_empty());

    // 2. With initial_routes, rerouting class "clk" freezes class "data" verbatim.
    let mut initial = partial.routes;
    initial[id_data].segments = vec![
        RoutedSegment {
            layer: 0,
            a: Point::new(1500.0, 9500.0),
            b: Point::new(5000.0, 9500.0),
            width: 200.0,
        },
        RoutedSegment {
            layer: 0,
            a: Point::new(5000.0, 9500.0),
            b: Point::new(5000.0, 10500.0),
            width: 200.0,
        },
        RoutedSegment {
            layer: 0,
            a: Point::new(5000.0, 10500.0),
            b: Point::new(10500.0, 10500.0),
            width: 200.0,
        },
        RoutedSegment {
            layer: 0,
            a: Point::new(10500.0, 10500.0),
            b: Point::new(10500.0, 9500.0),
            width: 200.0,
        },
    ];
    let frozen_data = initial[id_data].clone();

    let class_cfg = RouterConfig {
        max_passes: 10,
        ignored_nets: ignored,
        initial_routes: Some(initial),
        reroute_nets: Some(HashSet::from(["clk".to_string()])),
        ..Default::default()
    };
    let res = route(&design, &class_cfg).unwrap();
    assert_eq!(res.stats.routed_nets, 3);
    assert_eq!(res.stats.unconnected_pairs, 0);
    assert_eq!(res.stats.drc_violations, 0);
    assert!(res.routes[id_clk_ign].segments.is_empty());
    assert_route_exact_eq(&res.routes[id_data], &frozen_data, "/DATA/0 frozen");
}

#[test]
fn cli_reroute_nets_and_pad_validation() {
    let bin = env!("CARGO_BIN_EXE_viahop");
    let tmp_dir = std::env::temp_dir();
    let pid = std::process::id();
    let dsn_path = tmp_dir.join(format!("viahop_reroute_{pid}.dsn"));
    let init_ses_path = tmp_dir.join(format!("viahop_reroute_init_{pid}.ses"));
    let out_ses_path = tmp_dir.join(format!("viahop_reroute_out_{pid}.ses"));

    std::fs::write(&dsn_path, three_net_dsn()).unwrap();

    // Seed an initial session with an unoptimized staircase on /Net/B so we can verify it stays frozen.
    let design = DsnDesign::parse(three_net_dsn()).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    let id_a = board.net_name_to_id["/Net/A"];
    let id_b = board.net_name_to_id["/Net/B"];
    let id_c = board.net_name_to_id["/Net/C"];
    let mut init_routes = vec![
        NetRoute {
            net_id: id_a,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(1500.0, 2500.0),
                b: Point::new(10500.0, 5000.0),
                width: 200.0,
            }],
            vias: Vec::new(),
            unconnected_pairs: 1,
        },
        NetRoute {
            net_id: id_b,
            segments: vec![
                RoutedSegment {
                    layer: 0,
                    a: Point::new(1500.0, 5000.0),
                    b: Point::new(4000.0, 5000.0),
                    width: 200.0,
                },
                RoutedSegment {
                    layer: 0,
                    a: Point::new(4000.0, 5000.0),
                    b: Point::new(4000.0, 6200.0),
                    width: 200.0,
                },
                RoutedSegment {
                    layer: 0,
                    a: Point::new(4000.0, 6200.0),
                    b: Point::new(6000.0, 6200.0),
                    width: 200.0,
                },
                RoutedSegment {
                    layer: 0,
                    a: Point::new(6000.0, 6200.0),
                    b: Point::new(6000.0, 5000.0),
                    width: 200.0,
                },
                RoutedSegment {
                    layer: 0,
                    a: Point::new(6000.0, 5000.0),
                    b: Point::new(10500.0, 5000.0),
                    width: 200.0,
                },
            ],
            vias: Vec::new(),
            unconnected_pairs: 0,
        },
        NetRoute {
            net_id: id_c,
            ..Default::default()
        },
    ];
    init_routes.sort_by_key(|r| r.net_id);
    let init_ses = write_ses(
        &design,
        &viahop::RouteResult {
            board: board.clone(),
            routes: init_routes.clone(),
            stats: viahop::RouteStats {
                total_nets: 3,
                routable_nets: 3,
                routed_nets: 1,
                unconnected_pairs: 2,
                total_segments: 6,
                total_vias: 0,
                total_wirelength_um: 0.0,
                passes_run: 0,
                starts_run: 1,
                drc_violations: 0,
                dense_bga_pads: 0,
                pofv_capable_pads: 0,
                grid_step_um: 50.0,
                stopped_early: false,
                elapsed_ms: 0,
            },
        },
    );
    std::fs::write(&init_ses_path, &init_ses).unwrap();

    // Exercise repeatable --reroute-nets and --reroute-pad-mm with --initial-session.
    let out = std::process::Command::new(bin)
        .args([
            "-i",
            dsn_path.to_str().unwrap(),
            "-o",
            out_ses_path.to_str().unwrap(),
            "--initial-session",
            init_ses_path.to_str().unwrap(),
            "--reroute-nets",
            "/Net/A",
            "--reroute-nets",
            "/Net/C",
            "--reroute-pad-mm",
            "3.5",
            "--json-summary",
        ])
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "stderr: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(stdout.contains("\"routed_nets\":3"), "stdout: {stdout}");
    assert!(stdout.contains("\"drc_violations\":0"), "stdout: {stdout}");

    let parsed_init = parse_ses(&init_ses, &board).unwrap();
    let parsed_out = parse_ses(&std::fs::read_to_string(&out_ses_path).unwrap(), &board).unwrap();
    assert_route_exact_eq(
        &parsed_out[id_b],
        &parsed_init[id_b],
        "CLI frozen /Net/B must match initial session verbatim",
    );
    assert_eq!(check_drc(&board, &parsed_out).violations.len(), 0);

    // Reject non-positive and invalid --reroute-pad-mm values with clear errors.
    for bad in ["0", "-2.5", "abc", "NaN", "inf"] {
        let res = std::process::Command::new(bin)
            .args([
                "-i",
                dsn_path.to_str().unwrap(),
                "-o",
                out_ses_path.to_str().unwrap(),
                "--reroute-pad-mm",
                bad,
            ])
            .output()
            .unwrap();
        assert!(!res.status.success(), "expected failure for pad={bad}");
        let err = String::from_utf8_lossy(&res.stderr);
        assert!(
            err.contains("--reroute-pad-mm"),
            "missing flag in error for pad={bad}: {err}"
        );
    }

    let empty_nets = std::process::Command::new(bin)
        .args([
            "-i",
            dsn_path.to_str().unwrap(),
            "-o",
            out_ses_path.to_str().unwrap(),
            "--reroute-nets",
            " , ",
        ])
        .output()
        .unwrap();
    assert!(!empty_nets.status.success());

    for p in [&dsn_path, &init_ses_path, &out_ses_path] {
        let _ = std::fs::remove_file(p);
    }
}
