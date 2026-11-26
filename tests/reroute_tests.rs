use std::collections::HashSet;
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};

use viahop::fanout::analyze_fanout;
use viahop::geometry::Point;
use viahop::grid::RoutingGrid;
use viahop::optimize::route_wirelength;
use viahop::{
    Board, DrcReport, DrcViolation, DsnDesign, NetRoute, RoutedSegment, RoutedVia, RouterConfig,
    check_drc, parse_ses, route, route_with_checkpoint,
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
    let init_ses = viahop::ses::write_ses(&design, &board, &init_routes);
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
                "--reroute-nets",
                "/Net/A",
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

    // Reject standalone --reroute-pad-mm without --reroute-nets.
    let pad_without_nets = std::process::Command::new(bin)
        .args([
            "-i",
            dsn_path.to_str().unwrap(),
            "-o",
            out_ses_path.to_str().unwrap(),
            "--reroute-pad-mm",
            "3.0",
        ])
        .output()
        .unwrap();
    assert!(!pad_without_nets.status.success());
    let err = String::from_utf8_lossy(&pad_without_nets.stderr);
    assert!(
        err.contains("--reroute-pad-mm requires --reroute-nets"),
        "unexpected stderr: {err}"
    );

    // Reject unknown --reroute-nets target and all-ignored target via CLI.
    let unknown_net = std::process::Command::new(bin)
        .args([
            "-i",
            dsn_path.to_str().unwrap(),
            "-o",
            out_ses_path.to_str().unwrap(),
            "--reroute-nets",
            "/Net/DOES_NOT_EXIST",
        ])
        .output()
        .unwrap();
    assert!(!unknown_net.status.success());
    let err = String::from_utf8_lossy(&unknown_net.stderr);
    assert!(
        err.contains("/Net/DOES_NOT_EXIST"),
        "unexpected stderr: {err}"
    );

    let ignored_target = std::process::Command::new(bin)
        .args([
            "-i",
            dsn_path.to_str().unwrap(),
            "-o",
            out_ses_path.to_str().unwrap(),
            "--ignore",
            "/Net/A",
            "--reroute-nets",
            "/Net/A",
        ])
        .output()
        .unwrap();
    assert!(!ignored_target.status.success());
    let err = String::from_utf8_lossy(&ignored_target.stderr);
    assert!(err.contains("ignored"), "unexpected stderr: {err}");

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

fn pofv_four_layer_dsn() -> &'static str {
    r#"(pcb pofv_frozen.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer In1.Cu (type signal) (property (index 1)))
    (layer In2.Cu (type signal) (property (index 2)))
    (layer B.Cu (type signal) (property (index 3)))
    (boundary (rect pcb 0 0 12000 10000))
    (keepout "" (rect F.Cu 4500 500 5500 9500))
    (via "Via[0-3]_400:200_um")
    (rule (width 200) (clearance 200))
  )
  (placement
    (component P1
      (place T1 2000 5000 front 0)
      (place T2 9000 5000 front 0)
      (place F1 2000 1500 front 0)
      (place F2 2000 8500 front 0)
    )
  )
  (library
    (image P1 (pin SMD_500 1 0 0))
    (padstack SMD_500 (shape (circle F.Cu 500)) (attach off))
    (padstack "Via[0-3]_400:200_um"
      (shape (circle F.Cu 400))
      (shape (circle In1.Cu 400))
      (shape (circle In2.Cu 400))
      (shape (circle B.Cu 400))
      (attach off)
    )
  )
  (network
    (net /Net/T (pins T1-1 T2-1))
    (net /Net/F_IN2 (pins F1-1 F2-1))
    (class default /Net/T /Net/F_IN2
      (circuit (use_via "Via[0-3]_400:200_um"))
      (rule (width 200) (clearance 200))
    )
  )
  (wiring)
)"#
}

fn dogbone_and_stub_dsn() -> &'static str {
    r#"(pcb dogbone_stub.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 14000 12000))
    (via "Via[0-1]_300:150_um")
    (rule (width 120) (clearance 120))
  )
  (placement
    (component BGA5
      (place U1 5000 5000 front 0)
    )
    (component P1
      (place T2 9000 5000 front 0)
      (place FA1 5400 9000 front 0)
      (place FA2 8000 9000 front 0)
      (place FB1 9320 2000 front 0)
      (place FB2 9320 8000 front 0)
    )
  )
  (library
    (image BGA5
      (pin BGA_PAD C 0 0)
      (pin BGA_PAD L -800 0)
      (pin BGA_PAD R 800 0)
      (pin BGA_PAD D 0 -800)
      (pin BGA_PAD U 0 800)
    )
    (image P1 (pin BGA_PAD 1 0 0))
    (padstack BGA_PAD (shape (circle F.Cu 250)) (attach off))
    (padstack "Via[0-1]_300:150_um"
      (shape (circle F.Cu 300))
      (shape (circle B.Cu 300))
      (attach off)
    )
  )
  (network
    (net /Net/BGA_TARGET (pins U1-C T2-1))
    (net /Net/F_DOGBONE (pins FA1-1 FA2-1))
    (net /Net/F_STUB (pins FB1-1 FB2-1))
    (net /Net/G1 (pins U1-L))
    (net /Net/G2 (pins U1-R))
    (net /Net/G3 (pins U1-D))
    (net /Net/G4 (pins U1-U))
    (class default /Net/BGA_TARGET /Net/F_DOGBONE /Net/F_STUB /Net/G1 /Net/G2 /Net/G3 /Net/G4
      (circuit (use_via "Via[0-1]_300:150_um"))
      (rule (width 120) (clearance 120))
    )
  )
  (wiring)
)"#
}

#[test]
fn terminal_entries_avoid_frozen_copper_for_pofv_dogbone_and_stubs() {
    // 1. POFV: frozen In2.Cu trace passes directly under target F.Cu SMD pad T1 at (2000, 5000).
    let design = DsnDesign::parse(pofv_four_layer_dsn()).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), Some(true)).unwrap();
    let id_t = board.net_name_to_id["/Net/T"];
    let id_f = board.net_name_to_id["/Net/F_IN2"];
    let mut init = vec![
        NetRoute {
            net_id: id_t,
            ..Default::default()
        },
        NetRoute {
            net_id: id_f,
            segments: vec![RoutedSegment {
                layer: 2,
                a: Point::new(2000.0, 1500.0),
                b: Point::new(2000.0, 8500.0),
                width: 200.0,
            }],
            vias: vec![
                RoutedVia {
                    pos: Point::new(2000.0, 1500.0),
                    padstack: "Via[0-3]_400:200_um".to_string(),
                    radius: 200.0,
                    drill: 200.0,
                    layers: vec![0, 1, 2, 3],
                },
                RoutedVia {
                    pos: Point::new(2000.0, 8500.0),
                    padstack: "Via[0-3]_400:200_um".to_string(),
                    radius: 200.0,
                    drill: 200.0,
                    layers: vec![0, 1, 2, 3],
                },
            ],
            unconnected_pairs: 0,
        },
    ];
    init.sort_by_key(|r| r.net_id);
    let frozen_f = init[id_f].clone();

    let res = route(
        &design,
        &RouterConfig {
            max_passes: 15,
            via_at_smd: Some(true),
            initial_routes: Some(init),
            reroute_nets: Some(HashSet::from(["/Net/T".to_string()])),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(res.stats.unconnected_pairs, 0);
    assert_eq!(res.stats.drc_violations, 0);
    assert_route_exact_eq(&res.routes[id_f], &frozen_f, "/Net/F_IN2 frozen");
    assert!(
        res.routes[id_t]
            .vias
            .iter()
            .all(|v| v.pos.dist(Point::new(2000.0, 5000.0)) > 450.0),
        "target net must not drop a POFV via through frozen In2.Cu trace under T1"
    );

    // 2. Dogbone + +-2-cell pad stub: frozen F.Cu trace blocks (+400,+400) dogbone and T2 +2 stub.
    let db_design = DsnDesign::parse(dogbone_and_stub_dsn()).unwrap();
    let db_board = Board::from_dsn(&db_design, &HashSet::new(), Some(false)).unwrap();
    let id_bga = db_board.net_name_to_id["/Net/BGA_TARGET"];
    let id_fdb = db_board.net_name_to_id["/Net/F_DOGBONE"];
    let id_fst = db_board.net_name_to_id["/Net/F_STUB"];

    let mut db_init: Vec<NetRoute> = (0..db_board.nets.len())
        .map(|net_id| NetRoute {
            net_id,
            ..Default::default()
        })
        .collect();
    // Crosses directly over the default (+400, +400) = (5400, 5400) dogbone via of U1-C.
    db_init[id_fdb].segments = vec![
        RoutedSegment {
            layer: 0,
            a: Point::new(5400.0, 5400.0),
            b: Point::new(5400.0, 9000.0),
            width: 120.0,
        },
        RoutedSegment {
            layer: 0,
            a: Point::new(5400.0, 9000.0),
            b: Point::new(8000.0, 9000.0),
            width: 120.0,
        },
    ];
    // Passes at x=9320 next to T2(9000,5000): legal to pad (135um gap) but blocks +2 stub at x=9100.
    db_init[id_fst].segments = vec![RoutedSegment {
        layer: 0,
        a: Point::new(9320.0, 2000.0),
        b: Point::new(9320.0, 8000.0),
        width: 120.0,
    }];
    let frozen_fdb = db_init[id_fdb].clone();
    let frozen_fst = db_init[id_fst].clone();

    let db_res = route(
        &db_design,
        &RouterConfig {
            max_passes: 15,
            via_at_smd: Some(false),
            initial_routes: Some(db_init),
            reroute_nets: Some(HashSet::from(["/Net/BGA_TARGET".to_string()])),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(db_res.stats.unconnected_pairs, 0);
    assert_eq!(db_res.stats.drc_violations, 0);
    assert!(!db_res.routes[id_bga].segments.is_empty());
    assert_route_exact_eq(&db_res.routes[id_fdb], &frozen_fdb, "/Net/F_DOGBONE frozen");
    assert_route_exact_eq(&db_res.routes[id_fst], &frozen_fst, "/Net/F_STUB frozen");
}

#[test]
fn frozen_vias_and_via_seg_drc_converge_to_zero_drc() {
    let design = DsnDesign::parse(four_net_repair_dsn()).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();

    // 1. Direct check: bump_drc_history must bump via_hist for via_seg violations.
    let fanout = analyze_fanout(&board);
    let mut grid = RoutingGrid::build(&board, &fanout, 1);
    let pos = Point::new(5000.0, 5000.0);
    let (gx, gy) = grid.snap(pos);
    let c2 = gy * grid.cols + gx;
    assert_eq!(grid.via_hist[c2], 0.0);
    let report = DrcReport {
        violations: vec![DrcViolation {
            net_a: 0,
            net_b: Some(1),
            layer: Some(0),
            pos,
            kind: "via_seg",
            gap_um: 50.0,
            required_um: 200.0,
        }],
        conflicting_nets: HashSet::from([0]),
    };
    grid.bump_drc_history(&report, 3.0);
    assert!(
        grid.via_hist[c2] >= 3.0,
        "bump_drc_history must bump via_hist for via_seg, got {}",
        grid.via_hist[c2]
    );

    // 2. Frozen via directly in target path + initial via_seg violation against frozen B.Cu copper.
    let id_t = board.net_name_to_id["/Net/TARGET"];
    let id_n = board.net_name_to_id["/Net/NEIGHBOR"];
    let id_fx = board.net_name_to_id["/Net/FROZEN_X"];
    let id_fy = board.net_name_to_id["/Net/FROZEN_Y"];

    let mut init = vec![
        // /Net/TARGET has a via right on top of /Net/NEIGHBOR's frozen B.Cu segment (via_seg DRC).
        NetRoute {
            net_id: id_t,
            segments: vec![
                RoutedSegment {
                    layer: 0,
                    a: Point::new(2000.0, 2000.0),
                    b: Point::new(5000.0, 3500.0),
                    width: 200.0,
                },
                RoutedSegment {
                    layer: 1,
                    a: Point::new(5000.0, 3500.0),
                    b: Point::new(8000.0, 5000.0),
                    width: 200.0,
                },
            ],
            vias: vec![
                RoutedVia {
                    pos: Point::new(5000.0, 3500.0),
                    padstack: "Via[0-1]_400:200_um".to_string(),
                    radius: 200.0,
                    drill: 200.0,
                    layers: vec![0, 1],
                },
                RoutedVia {
                    pos: Point::new(8000.0, 5000.0),
                    padstack: "Via[0-1]_400:200_um".to_string(),
                    radius: 200.0,
                    drill: 200.0,
                    layers: vec![0, 1],
                },
            ],
            unconnected_pairs: 0,
        },
        // /Net/NEIGHBOR is frozen with a via at (5000, 3500) blocking both F.Cu and B.Cu.
        NetRoute {
            net_id: id_n,
            segments: vec![
                RoutedSegment {
                    layer: 0,
                    a: Point::new(2000.0, 5000.0),
                    b: Point::new(5000.0, 3500.0),
                    width: 200.0,
                },
                RoutedSegment {
                    layer: 1,
                    a: Point::new(5000.0, 3500.0),
                    b: Point::new(8000.0, 2000.0),
                    width: 200.0,
                },
            ],
            vias: vec![
                RoutedVia {
                    pos: Point::new(5000.0, 3500.0),
                    padstack: "Via[0-1]_400:200_um".to_string(),
                    radius: 200.0,
                    drill: 200.0,
                    layers: vec![0, 1],
                },
                RoutedVia {
                    pos: Point::new(8000.0, 2000.0),
                    padstack: "Via[0-1]_400:200_um".to_string(),
                    radius: 200.0,
                    drill: 200.0,
                    layers: vec![0, 1],
                },
            ],
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
    init.sort_by_key(|r| r.net_id);
    assert!(
        check_drc(&board, &init)
            .violations
            .iter()
            .any(|v| v.kind == "via_seg"),
        "initial session must contain a via_seg violation against frozen copper"
    );
    let frozen_n = init[id_n].clone();

    let res = route(
        &design,
        &RouterConfig {
            max_passes: 15,
            initial_routes: Some(init),
            reroute_nets: Some(HashSet::from(["/Net/TARGET".to_string()])),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(res.stats.routed_nets, 4);
    assert_eq!(res.stats.unconnected_pairs, 0);
    assert_eq!(res.stats.drc_violations, 0);
    assert_route_exact_eq(&res.routes[id_n], &frozen_n, "/Net/NEIGHBOR frozen");
}

fn mixed_rules_dsn() -> &'static str {
    r#"(pcb mixed_rules.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 14000 12000))
    (keepout "" (rect F.Cu 5500 5900 6500 8200))
    (via "Via[0-1]_400:200_um")
    (rule (width 200) (clearance 200))
  )
  (placement
    (component P1
      (place F1A 1500 4000 front 0)
      (place F1B 12500 4000 front 0)
      (place F2A 1500 5000 front 0)
      (place F2B 12500 5000 front 0)
      (place N1 1500 4500 front 0)
      (place N2 12500 4500 front 0)
      (place W1 1500 6000 front 0)
      (place W2 12500 6000 front 0)
    )
  )
  (library
    (image P1 (pin SMD_300 1 0 0))
    (padstack SMD_300 (shape (circle F.Cu 300)) (attach off))
    (padstack "Via[0-1]_400:200_um"
      (shape (circle F.Cu 400))
      (shape (circle B.Cu 400))
      (attach off)
    )
  )
  (network
    (net /Net/F1 (pins F1A-1 F1B-1))
    (net /Net/F2 (pins F2A-1 F2B-1))
    (net /Net/NARROW (pins N1-1 N2-1))
    (net /Net/WIDE (pins W1-1 W2-1))
    (class frozen /Net/F1 /Net/F2
      (circuit (use_via "Via[0-1]_400:200_um"))
      (rule (width 200) (clearance 200))
    )
    (class narrow /Net/NARROW
      (circuit (use_via "Via[0-1]_400:200_um"))
      (rule (width 150) (clearance 150))
    )
    (class wide /Net/WIDE
      (circuit (use_via "Via[0-1]_400:200_um"))
      (rule (width 600) (clearance 350))
    )
  )
  (wiring)
)"#
}

#[test]
fn mixed_narrow_and_wide_target_classes_next_to_frozen_copper() {
    let design = DsnDesign::parse(mixed_rules_dsn()).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    let id_f1 = board.net_name_to_id["/Net/F1"];
    let id_f2 = board.net_name_to_id["/Net/F2"];
    let id_n = board.net_name_to_id["/Net/NARROW"];
    let id_w = board.net_name_to_id["/Net/WIDE"];

    let mut init = vec![
        NetRoute {
            net_id: id_f1,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(1500.0, 4000.0),
                b: Point::new(12500.0, 4000.0),
                width: 200.0,
            }],
            vias: Vec::new(),
            unconnected_pairs: 0,
        },
        NetRoute {
            net_id: id_f2,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(1500.0, 5000.0),
                b: Point::new(12500.0, 5000.0),
                width: 200.0,
            }],
            vias: Vec::new(),
            unconnected_pairs: 0,
        },
        NetRoute {
            net_id: id_n,
            ..Default::default()
        },
        NetRoute {
            net_id: id_w,
            ..Default::default()
        },
    ];
    init.sort_by_key(|r| r.net_id);
    let frozen_f1 = init[id_f1].clone();
    let frozen_f2 = init[id_f2].clone();

    let res = route(
        &design,
        &RouterConfig {
            max_passes: 15,
            initial_routes: Some(init),
            reroute_nets: Some(HashSet::from([
                "/Net/NARROW".to_string(),
                "/Net/WIDE".to_string(),
            ])),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(res.stats.routed_nets, 4);
    assert_eq!(res.stats.unconnected_pairs, 0);
    assert_eq!(res.stats.drc_violations, 0);
    assert_route_exact_eq(&res.routes[id_f1], &frozen_f1, "/Net/F1 frozen");
    assert_route_exact_eq(&res.routes[id_f2], &frozen_f2, "/Net/F2 frozen");
}

fn bounds_and_corridor_trap_dsn() -> &'static str {
    r#"(pcb bounds_trap.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (boundary (rect pcb 0 0 14000 10000))
    (keepout "" (rect signal 2000 6400 12000 7500))
    (keepout "" (rect signal 2900 5350 7200 5600))
    (keepout "" (rect signal 6800 3850 7200 5600))
    (via "Via[0-0]_400:200_um")
    (rule (width 200) (clearance 200))
  )
  (placement
    (component P1
      (place A1 3500 5000 front 0)
      (place A2 10500 5000 front 0)
      (place B1 4300 5000 front 0)
      (place B2 9700 5000 front 0)
      (place F1 1500 8500 front 0)
      (place F2 3000 8500 front 0)
    )
  )
  (library
    (image P1 (pin SMD_400 1 0 0))
    (padstack SMD_400 (shape (circle F.Cu 400)) (attach off))
    (padstack "Via[0-0]_400:200_um" (shape (circle F.Cu 400)) (attach off))
  )
  (network
    (net /Net/A (pins A1-1 A2-1))
    (net /Net/B (pins B1-1 B2-1))
    (net /Net/FROZEN (pins F1-1 F2-1))
    (class default /Net/A /Net/B /Net/FROZEN
      (circuit (use_via "Via[0-0]_400:200_um"))
      (rule (width 200) (clearance 200))
    )
  )
  (wiring)
)"#
}

#[test]
fn search_bounds_restricts_local_path_and_widens_on_congestion_trap() {
    let design = DsnDesign::parse(bounds_and_corridor_trap_dsn()).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    let id_a = board.net_name_to_id["/Net/A"];
    let id_b = board.net_name_to_id["/Net/B"];
    let id_f = board.net_name_to_id["/Net/FROZEN"];

    let mut base_init = vec![
        NetRoute {
            net_id: id_a,
            ..Default::default()
        },
        NetRoute {
            net_id: id_b,
            ..Default::default()
        },
        NetRoute {
            net_id: id_f,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(1500.0, 8500.0),
                b: Point::new(3000.0, 8500.0),
                width: 200.0,
            }],
            vias: Vec::new(),
            unconnected_pairs: 0,
        },
    ];
    base_init.sort_by_key(|r| r.net_id);
    let frozen_orig = base_init[id_f].clone();

    // 1. Bounded single-net reroute (pad=1100um -> Y in [3700, 6300]) takes the inside hook slot.
    let bounded = route(
        &design,
        &RouterConfig {
            max_passes: 10,
            initial_routes: Some(base_init.clone()),
            reroute_nets: Some(HashSet::from(["/Net/A".to_string()])),
            reroute_pad_um: Some(1100.0),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(bounded.stats.drc_violations, 0);
    assert_eq!(bounded.routes[id_a].unconnected_pairs, 0);
    assert!(
        bounded.routes[id_a]
            .segments
            .iter()
            .all(|s| s.a.y >= 3650.0 && s.b.y >= 3650.0),
        "bounded search must keep clean single-net route inside pad box"
    );

    // 2. Unbounded reroute (large pad) takes the strictly shorter outside bypass below y=3700.
    let unbounded = route(
        &design,
        &RouterConfig {
            max_passes: 10,
            initial_routes: Some(base_init.clone()),
            reroute_nets: Some(HashSet::from(["/Net/A".to_string()])),
            reroute_pad_um: Some(3500.0),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(unbounded.stats.drc_violations, 0);
    assert_eq!(unbounded.routes[id_a].unconnected_pairs, 0);
    assert!(
        unbounded.routes[id_a]
            .segments
            .iter()
            .any(|s| s.a.y < 3650.0 || s.b.y < 3650.0),
        "unbounded search should use the shorter outside bypass"
    );
    assert!(
        route_wirelength(&unbounded.routes[id_a]) + 500.0 < route_wirelength(&bounded.routes[id_a]),
        "outside bypass must be strictly shorter than bounded hook detour"
    );

    // 3. Two target nets share the 1-track inside slot: pass >= 2 widens to unbounded to escape.
    let widened = route(
        &design,
        &RouterConfig {
            max_passes: 15,
            initial_routes: Some(base_init),
            reroute_nets: Some(HashSet::from(["/Net/A".to_string(), "/Net/B".to_string()])),
            reroute_pad_um: Some(1100.0),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(widened.stats.routed_nets, 3);
    assert_eq!(widened.stats.unconnected_pairs, 0);
    assert_eq!(widened.stats.drc_violations, 0);
    assert_route_exact_eq(&widened.routes[id_f], &frozen_orig, "/Net/FROZEN");
    let escaped_box = [&widened.routes[id_a], &widened.routes[id_b]]
        .iter()
        .any(|r| r.segments.iter().any(|s| s.a.y < 3650.0 || s.b.y < 3650.0));
    assert!(
        escaped_box,
        "2-net congestion trap inside bounded box must widen to unbounded detour"
    );
}

#[test]
fn early_cancellation_preserves_initial_session_target_routes() {
    let design = DsnDesign::parse(three_net_dsn()).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    let id_a = board.net_name_to_id["/Net/A"];
    let id_b = board.net_name_to_id["/Net/B"];
    let id_c = board.net_name_to_id["/Net/C"];

    let mut initial = vec![
        NetRoute {
            net_id: id_a,
            segments: vec![
                RoutedSegment {
                    layer: 0,
                    a: Point::new(1500.0, 2500.0),
                    b: Point::new(5000.0, 2500.0),
                    width: 200.0,
                },
                RoutedSegment {
                    layer: 0,
                    a: Point::new(5000.0, 2500.0),
                    b: Point::new(10500.0, 2500.0),
                    width: 200.0,
                },
            ],
            vias: Vec::new(),
            unconnected_pairs: 0,
        },
        NetRoute {
            net_id: id_b,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(1500.0, 5000.0),
                b: Point::new(10500.0, 5000.0),
                width: 200.0,
            }],
            vias: Vec::new(),
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

    let checkpoints: Arc<Mutex<Vec<Vec<NetRoute>>>> = Arc::new(Mutex::new(Vec::new()));
    let cp_ref = Arc::clone(&checkpoints);
    let stop = Arc::new(AtomicBool::new(true));
    let res = route_with_checkpoint(
        &design,
        &RouterConfig {
            max_passes: 10,
            initial_routes: Some(initial.clone()),
            reroute_nets: Some(HashSet::from(["/Net/A".to_string()])),
            stop_flag: Some(stop),
            ..Default::default()
        },
        move |_b, routes, _score| {
            cp_ref.lock().unwrap().push(routes.to_vec());
        },
    )
    .unwrap();

    assert!(res.stats.stopped_early);
    assert_eq!(res.stats.passes_run, 0);
    assert_route_exact_eq(
        &res.routes[id_a],
        &initial[id_a],
        "early cancel must preserve target /Net/A",
    );
    assert_route_exact_eq(&res.routes[id_b], &initial[id_b], "frozen /Net/B");
    assert_route_exact_eq(&res.routes[id_c], &initial[id_c], "frozen /Net/C");
    for snap in checkpoints.lock().unwrap().iter() {
        assert!(
            !snap[id_a].segments.is_empty(),
            "checkpoint must never emit a wiped target net"
        );
    }
}

#[test]
fn rejects_unmatched_and_all_ignored_reroute_targets() {
    let design = DsnDesign::parse(multi_class_dsn()).unwrap();

    let err_unknown = route(
        &design,
        &RouterConfig {
            reroute_nets: Some(HashSet::from(["/Net/MISSING".to_string()])),
            ..Default::default()
        },
    )
    .unwrap_err();
    assert!(
        err_unknown.contains("/Net/MISSING"),
        "unexpected error: {err_unknown}"
    );

    let err_cp = route_with_checkpoint(
        &design,
        &RouterConfig {
            reroute_nets: Some(HashSet::from([
                "/CLK/A".to_string(),
                "no_such_class".to_string(),
            ])),
            ..Default::default()
        },
        |_b, _r, _s| {},
    )
    .unwrap_err();
    assert!(
        err_cp.contains("no_such_class"),
        "unexpected error: {err_cp}"
    );

    let err_empty = route(
        &design,
        &RouterConfig {
            reroute_nets: Some(HashSet::new()),
            ..Default::default()
        },
    )
    .unwrap_err();
    assert!(
        err_empty.contains("--reroute-nets"),
        "unexpected error: {err_empty}"
    );

    let err_ignored = route(
        &design,
        &RouterConfig {
            ignored_nets: HashSet::from(["/CLK/IGN".to_string()]),
            reroute_nets: Some(HashSet::from(["/CLK/IGN".to_string()])),
            ..Default::default()
        },
    )
    .unwrap_err();
    assert!(
        err_ignored.contains("ignored"),
        "unexpected error: {err_ignored}"
    );
}
