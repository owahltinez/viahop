use std::collections::HashSet;
use std::process::Command;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;

use viahop::geometry::{Point, point_to_segment_dist, segment_to_segment_dist};
use viahop::{
    Board, DsnDesign, NetRoute, RouteReport, RouteStats, RoutedSegment, RoutedVia, RouterConfig,
    analyze_fanout, check_drc, route, run_preflight, run_preflight_with_options,
};

fn clean_two_layer_dsn() -> &'static str {
    r#"(pcb clean_preflight.dsn
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
    (component HDR2
      (place J1 1500 4000 front 0)
      (place J2 8500 4000 front 0)
    )
  )
  (library
    (image HDR2
      (pin SMD_500 1 0 -1500)
      (pin SMD_500 2 0 1500)
    )
    (padstack SMD_500 (shape (circle F.Cu 500)) (attach off))
    (padstack "Via[0-1]_400:200_um"
      (shape (circle F.Cu 400))
      (shape (circle B.Cu 400))
      (attach off)
    )
  )
  (network
    (net /Net/A (pins J1-1 J2-2))
    (net /Net/B (pins J1-2 J2-1))
    (class default /Net/A /Net/B
      (circuit (use_via "Via[0-1]_400:200_um"))
      (rule (width 200) (clearance 200))
    )
  )
  (wiring)
)"#
}

fn blocked_and_open_dsn() -> &'static str {
    r#"(pcb diag_board.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 10000 6000))
    (keepout "" (rect signal 4500 0 5500 6000))
    (via "Via[0-1]_400:200_um")
    (rule (width 200) (clearance 200))
  )
  (placement
    (component P1
      (place A1 1500 1500 front 0)
      (place A2 8500 1500 front 0)
      (place B1 1500 4500 front 0)
      (place B2 3500 4500 front 0)
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
    (net /Net/BLOCKED (pins A1-1 A2-1))
    (net /Net/OPEN (pins B1-1 B2-1))
    (class default /Net/BLOCKED /Net/OPEN
      (circuit (use_via "Via[0-1]_400:200_um"))
      (rule (width 200) (clearance 200))
    )
  )
  (wiring)
)"#
}

#[allow(clippy::too_many_arguments)]
fn dense_bga_dsn(
    grid_n: i32,
    pitch: i32,
    pad_dia: i32,
    via_dia: i32,
    via_drill: i32,
    width: i32,
    clr: i32,
    via_clr: i32,
    pofv_on: bool,
    extra_wiring: &str,
) -> String {
    let mut pins = String::new();
    let mut nets = String::new();
    let mut class_nets = String::new();
    let mut target_pins = String::new();
    let half = grid_n / 2;
    let mut idx = 0;

    for r in 0..grid_n {
        for c in 0..grid_n {
            let x = (c - half) * pitch;
            let y = (r - half) * pitch;
            let pname = format!("P{r}_{c}");
            pins.push_str(&format!("      (pin BGA_PAD {pname} {x} {y})\n"));

            // Spread destination headers around the perimeter to avoid congestion bottlenecks.
            let tx = ((idx % 13) - 6) * 1000;
            let ty = if idx < 13 { 0 } else { 1800 };
            target_pins.push_str(&format!("      (pin HDR_300 T{idx} {tx} {ty})\n"));

            let nname = format!("/BGA/N{idx}");
            nets.push_str(&format!("    (net {nname} (pins U1-{pname} J1-T{idx}))\n"));
            class_nets.push(' ');
            class_nets.push_str(&nname);
            idx += 1;
        }
    }

    let pofv_str = if pofv_on { "on" } else { "off" };
    let via_name = format!("Via[0-5]_{via_dia}:{via_drill}_um");
    format!(
        r#"(pcb bga_{grid_n}x{grid_n}.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer In1.Cu (type power) (property (index 1)))
    (layer In2.Cu (type signal) (property (index 2)))
    (layer In3.Cu (type power) (property (index 3)))
    (layer In4.Cu (type signal) (property (index 4)))
    (layer B.Cu (type signal) (property (index 5)))
    (boundary (rect pcb 0 0 22000 18000))
    (via "{via_name}")
    (rule
      (width {width})
      (clearance {clr})
      (clearance {via_clr} (type via_wire))
      (clearance {via_clr} (type via_via))
      (clearance {via_clr} (type via_smd))
    )
    (control (via_at_smd {pofv_str}))
  )
  (placement
    (component BGA_GRID (place U1 11000 5000 front 0))
    (component HDR_TGT (place J1 11000 13500 front 0))
  )
  (library
    (image BGA_GRID
{pins}    )
    (image HDR_TGT
{target_pins}    )
    (padstack BGA_PAD (shape (circle F.Cu {pad_dia})) (attach off))
    (padstack HDR_300 (shape (circle F.Cu 300)) (attach off))
    (padstack "{via_name}"
      (shape (circle F.Cu {via_dia}))
      (shape (circle In1.Cu {via_dia}))
      (shape (circle In2.Cu {via_dia}))
      (shape (circle In3.Cu {via_dia}))
      (shape (circle In4.Cu {via_dia}))
      (shape (circle B.Cu {via_dia}))
      (attach off)
    )
  )
  (network
{nets}    (net GND (pins))
    (class kicad_default{class_nets}
      (circuit (use_via "{via_name}"))
      (rule (width {width}) (clearance {clr}) (clearance {via_clr} (type via_wire)) (clearance {via_clr} (type via_via)) (clearance {via_clr} (type via_smd)))
    )
  )
  (wiring
{extra_wiring}  )
)"#
    )
}

#[test]
fn clean_board_passes_preflight_api_and_cli() {
    let design = DsnDesign::parse(clean_two_layer_dsn()).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    let report = run_preflight(&board, 2);

    assert!(report.ok, "clean board must pass preflight: {report:?}");
    assert_eq!(report.routable_nets, 2);
    assert!(report.sealed_pads.is_empty());
    assert!(report.blocked_nets.is_empty());
    assert!(report.fixed_violations.is_empty());
    let json = report.to_json();
    assert!(json.contains("\"ok\": true"));

    let bin = env!("CARGO_BIN_EXE_viahop");
    let tmp = std::env::temp_dir().join("viahop_preflight_clean.dsn");
    let tmp_rep = std::env::temp_dir().join("viahop_preflight_clean.json");
    std::fs::write(&tmp, clean_two_layer_dsn()).unwrap();

    // Note: -o/--output is intentionally omitted for --preflight.
    let out = Command::new(bin)
        .args([
            "-i",
            tmp.to_str().unwrap(),
            "--preflight",
            "--report",
            tmp_rep.to_str().unwrap(),
            "--json-summary",
        ])
        .output()
        .unwrap();
    assert_eq!(
        out.status.code(),
        Some(0),
        "stderr: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(stdout.contains("\"ok\": true"));
    let rep_text = std::fs::read_to_string(&tmp_rep).unwrap();
    assert!(rep_text.contains("\"ok\": true"));

    let _ = std::fs::remove_file(&tmp);
    let _ = std::fs::remove_file(&tmp_rep);
}

#[test]
fn blocked_board_fails_preflight_api_and_cli() {
    let design = DsnDesign::parse(blocked_and_open_dsn()).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    let report = run_preflight(&board, 2);

    assert!(!report.ok);
    assert_eq!(report.blocked_nets.len(), 1);
    assert_eq!(report.blocked_nets[0].net, "/Net/BLOCKED");
    assert_eq!(
        report.blocked_nets[0].reason,
        "static_obstacle_or_rule_blockage"
    );

    let bin = env!("CARGO_BIN_EXE_viahop");
    let tmp = std::env::temp_dir().join("viahop_preflight_blocked.dsn");
    std::fs::write(&tmp, blocked_and_open_dsn()).unwrap();
    let out = Command::new(bin)
        .args(["-i", tmp.to_str().unwrap(), "--preflight", "--json-summary"])
        .output()
        .unwrap();
    assert_eq!(out.status.code(), Some(1));
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(stdout.contains("\"ok\": false"));
    assert!(stdout.contains("/Net/BLOCKED"));
    assert!(stdout.contains("static_obstacle_or_rule_blockage"));

    let _ = std::fs::remove_file(&tmp);
}

#[test]
fn sealed_dense_bga_caught_by_preflight_no_pofv_and_fixed_via_block() {
    // 1. 3x3 0.40 mm BGA with --no-via-at-smd seals center ball P1_1.
    let dsn_str = dense_bga_dsn(3, 400, 240, 250, 150, 92, 92, 105, false, "");
    let design = DsnDesign::parse(&dsn_str).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), Some(false)).unwrap();
    let report = run_preflight(&board, 2);

    assert!(!report.ok);
    assert_eq!(report.bga_clusters.len(), 1);
    assert_eq!(report.bga_clusters[0].refdes, "U1");
    assert_eq!(report.bga_clusters[0].max_ring, 1);
    assert_eq!(report.sealed_pads.len(), 1);
    let sp = &report.sealed_pads[0];
    assert_eq!(sp.refdes, "U1");
    assert_eq!(sp.pin, "P1_1");
    assert_eq!(sp.ring, 1);
    assert!(
        sp.blocker.contains("via_at_smd disabled") && sp.blocker.contains("pad U1-"),
        "unexpected blocker: {}",
        sp.blocker
    );

    let bin = env!("CARGO_BIN_EXE_viahop");
    let tmp = std::env::temp_dir().join("viahop_preflight_sealed.dsn");
    std::fs::write(&tmp, &dsn_str).unwrap();
    let out = Command::new(bin)
        .args([
            "-i",
            tmp.to_str().unwrap(),
            "--no-via-at-smd",
            "--preflight",
            "--json-summary",
        ])
        .output()
        .unwrap();
    assert_eq!(out.status.code(), Some(1));
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(stdout.contains("\"pin\":\"P1_1\""));
    let _ = std::fs::remove_file(&tmp);

    // 2. Same BGA with POFV enabled, but a fixed GND via sits at center ball U1-P1_1.
    let blocked_via = "    (via \"Via[0-5]_250:150_um\" 11000 5000 (net GND) (type fix))\n";
    let dsn_fixed = dense_bga_dsn(3, 400, 240, 250, 150, 92, 92, 105, true, blocked_via);
    let design_fixed = DsnDesign::parse(&dsn_fixed).unwrap();
    let mut ignored = HashSet::new();
    ignored.insert("GND".to_string());
    let board_fixed = Board::from_dsn(&design_fixed, &ignored, Some(true)).unwrap();
    let report_fixed = run_preflight(&board_fixed, 2);

    assert!(!report_fixed.ok);
    assert!(
        report_fixed
            .sealed_pads
            .iter()
            .any(|p| p.pin == "P1_1" && p.blocker.contains("fixed_via (GND)")),
        "expected P1_1 sealed by fixed_via (GND), got: {:?}",
        report_fixed.sealed_pads
    );
    assert!(
        !report_fixed.fixed_violations.is_empty(),
        "fixed GND via overlapping signal pad must also report fixed_violations"
    );
}

#[test]
fn five_by_five_dense_040mm_bga_rings_non_colliding_dogbones_and_pofv_routing() {
    // Part A: 5x5 0.40 mm BGA with microvia (160:80) and POFV off -> non-colliding dogbones.
    let dsn_dogbone = dense_bga_dsn(5, 400, 180, 160, 80, 80, 80, 80, false, "");
    let design_db = DsnDesign::parse(&dsn_dogbone).unwrap();
    let board_db = Board::from_dsn(&design_db, &HashSet::new(), Some(false)).unwrap();
    let fanout_db = analyze_fanout(&board_db);

    assert_eq!(fanout_db.bga_clusters.len(), 1);
    let cluster = &fanout_db.bga_clusters[0];
    assert_eq!(cluster.refdes, "U1");
    assert_eq!(cluster.pad_ids.len(), 25);
    assert!((cluster.pitch_um - 400.0).abs() < 1.0);
    assert_eq!(cluster.max_ring, 2);
    assert!(
        cluster.sealed_pad_ids.is_empty(),
        "all 5x5 inner balls should receive non-colliding dogbones: {:?}",
        cluster.sealed_pad_ids
    );

    let mut ring_counts = [0usize; 3];
    let mut inner_dogbones = Vec::new();
    for &pid in &cluster.pad_ids {
        let pf = &fanout_db.pads[pid];
        let r = pf.bga_ring.expect("every BGA pad must have a ring depth");
        ring_counts[r] += 1;
        match r {
            0 => {
                assert_eq!(pf.preferred_escape_layer, Some(0), "ring 0 prefers F.Cu");
                assert!(pf.dogbone_via.is_none(), "ring 0 should escape on surface");
            }
            1 => {
                assert_eq!(pf.preferred_escape_layer, Some(2), "ring 1 prefers In2.Cu");
                let pos = pf.dogbone_via.expect("ring 1 needs dogbone when POFV off");
                inner_dogbones.push((
                    board_db.pads[pid].net_id.unwrap(),
                    board_db.pads[pid].center,
                    pos,
                ));
            }
            2 => {
                assert_eq!(pf.preferred_escape_layer, Some(4), "ring 2 prefers In4.Cu");
                let pos = pf.dogbone_via.expect("ring 2 needs dogbone when POFV off");
                inner_dogbones.push((
                    board_db.pads[pid].net_id.unwrap(),
                    board_db.pads[pid].center,
                    pos,
                ));
            }
            other => panic!("unexpected ring depth {other}"),
        }
    }
    assert_eq!(ring_counts, [16, 8, 1]);
    assert_eq!(inner_dogbones.len(), 9);

    // Verify no two assigned dogbones collide on via-via, drill, stub-via, or stub-stub clearances.
    for i in 0..inner_dogbones.len() {
        for j in (i + 1)..inner_dogbones.len() {
            let (na, pad_a, via_a) = inner_dogbones[i];
            let (nb, pad_b, via_b) = inner_dogbones[j];
            let ra = board_db.nets[na].rules.via_radius;
            let rb = board_db.nets[nb].rules.via_radius;
            let da = board_db.nets[na].rules.via_drill;
            let db = board_db.nets[nb].rules.via_drill;
            let ha = board_db.nets[na].rules.width * 0.5;
            let hb = board_db.nets[nb].rules.width * 0.5;
            let dist = via_a.dist(via_b);
            let vv_clr = board_db.via_to_via_clearance(na, Some(nb));
            let vw_clr = board_db.via_to_wire_clearance(na, Some(nb));
            let ww_clr = board_db.wire_to_other_clearance(na, Some(nb));
            assert!(
                dist - ra - rb >= vv_clr - 1e-3,
                "dogbones {i} and {j} violate via_via clearance: dist={dist}"
            );
            assert!(
                dist >= (da + db) * 0.5 + 205.0 - 1e-3,
                "dogbones {i} and {j} violate drill spacing: dist={dist}"
            );
            let gap_va_sb = point_to_segment_dist(via_a, pad_b, via_b) - ra - hb;
            let gap_vb_sa = point_to_segment_dist(via_b, pad_a, via_a) - rb - ha;
            assert!(
                gap_va_sb >= vw_clr - 1e-3 && gap_vb_sa >= vw_clr - 1e-3,
                "dogbones {i} and {j} violate via-to-stub clearance: {gap_va_sb}, {gap_vb_sa} < {vw_clr}"
            );
            let gap_ss = segment_to_segment_dist(pad_a, via_a, pad_b, via_b) - ha - hb;
            assert!(
                gap_ss >= ww_clr - 1e-3,
                "dogbones {i} and {j} violate stub-to-stub clearance: {gap_ss} < {ww_clr}"
            );
        }
    }

    // End-to-end preflight, routing, and 0-DRC verification for dogbone BGA.
    let db_preflight = run_preflight(&board_db, 4);
    assert!(
        db_preflight.ok,
        "5x5 dogbone BGA must pass preflight: {db_preflight:?}"
    );
    let res_db = route(
        &design_db,
        &RouterConfig {
            max_passes: 25,
            via_at_smd: Some(false),
            threads: 4,
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(res_db.stats.routable_nets, 25);
    assert_eq!(
        res_db.stats.routed_nets, 25,
        "all 25 dogbone balls must route"
    );
    assert_eq!(res_db.stats.unconnected_pairs, 0);
    let drc_db = check_drc(&res_db.board, &res_db.routes);
    assert_eq!(
        drc_db.violations.len(),
        0,
        "dogbone BGA DRC violations: {:?}",
        drc_db.violations
    );

    // Part B: 5x5 0.40 mm BGA with HDI microvias (150:75) and POFV on -> clean 0-DRC route for all 25 balls.
    let dsn_pofv = dense_bga_dsn(5, 400, 220, 150, 75, 70, 75, 75, true, "");
    let design_pofv = DsnDesign::parse(&dsn_pofv).unwrap();
    let board_pofv = Board::from_dsn(&design_pofv, &HashSet::new(), Some(true)).unwrap();
    let pf_report = run_preflight(&board_pofv, 4);
    assert!(
        pf_report.ok,
        "5x5 0.40mm POFV BGA must pass preflight: {pf_report:?}"
    );
    assert_eq!(pf_report.bga_clusters[0].max_ring, 2);

    let res = route(
        &design_pofv,
        &RouterConfig {
            max_passes: 25,
            via_at_smd: Some(true),
            threads: 4,
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(res.stats.routable_nets, 25);
    assert_eq!(res.stats.routed_nets, 25, "all 25 BGA balls must route");
    assert_eq!(res.stats.unconnected_pairs, 0);
    let drc = check_drc(&res.board, &res.routes);
    assert_eq!(
        res.stats.drc_violations, 0,
        "DRC violations: {:?}",
        drc.violations
    );
}

#[test]
fn static_drc_attributes_exact_blockers_in_drc_and_report_json() {
    let dsn = r#"(pcb blocker_attr.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 10000 8000))
    (keepout "" (rect signal 4000 4000 5000 5000))
    (via "Via[0-1]_400:200_um")
    (rule (width 200) (clearance 200))
  )
  (placement
    (component P1
      (place U1 2000 2000 front 0)
      (place U2 8000 2000 front 0)
      (place U3 5000 2000 front 0)
    )
    (component IMG_KO
      (place K1 7000 6000 front 0)
    )
  )
  (library
    (image P1 (pin SMD_500 1 0 0))
    (image IMG_KO
      (pin SMD_500 1 0 -1000)
      (keepout "" (rect F.Cu -400 -400 400 400))
    )
    (padstack SMD_500 (shape (circle F.Cu 500)) (attach off))
    (padstack "Via[0-1]_400:200_um"
      (shape (circle F.Cu 400))
      (shape (circle B.Cu 400))
      (attach off)
    )
  )
  (network
    (net /Net/SIG (pins U1-1 U2-1))
    (net GND (pins U3-1 K1-1))
    (class default /Net/SIG GND
      (circuit (use_via "Via[0-1]_400:200_um"))
      (rule (width 200) (clearance 200))
    )
  )
  (wiring
    (wire (path F.Cu 200 2000 6000 3000 6000) (net GND) (type fix))
    (via "Via[0-1]_400:200_um" 3000 4000 (net GND) (type fix))
  )
)"#;

    let design = DsnDesign::parse(dsn).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    let sig_id = board.net_name_to_id["/Net/SIG"];
    let gnd_id = board.net_name_to_id["GND"];

    // Construct routed segments & via intentionally hitting each static obstacle category.
    let routes = vec![
        NetRoute {
            net_id: sig_id,
            segments: vec![
                // 1. Hits pad U3-1 (GND) at (5000, 2000).
                RoutedSegment {
                    layer: 0,
                    a: Point::new(4500.0, 2000.0),
                    b: Point::new(5500.0, 2000.0),
                    width: 200.0,
                },
                // 2. Hits structure keepout at [4000..5000, 4000..5000].
                RoutedSegment {
                    layer: 0,
                    a: Point::new(4200.0, 4500.0),
                    b: Point::new(4800.0, 4500.0),
                    width: 200.0,
                },
                // 3. Hits image keepout on K1 at [6600..7400, 5600..6400].
                RoutedSegment {
                    layer: 0,
                    a: Point::new(6800.0, 6000.0),
                    b: Point::new(7200.0, 6000.0),
                    width: 200.0,
                },
                // 4. Exits board boundary (x > 10000).
                RoutedSegment {
                    layer: 0,
                    a: Point::new(9500.0, 1000.0),
                    b: Point::new(10500.0, 1000.0),
                    width: 200.0,
                },
                // 5. Hits fixed wire (GND) at (2000,6000)-(3000,6000).
                RoutedSegment {
                    layer: 0,
                    a: Point::new(2500.0, 5800.0),
                    b: Point::new(2500.0, 6200.0),
                    width: 200.0,
                },
            ],
            vias: vec![
                // 6. Via hits fixed_via (GND) at (3000, 4000).
                RoutedVia {
                    pos: Point::new(3100.0, 4000.0),
                    padstack: "Via[0-1]_400:200_um".to_string(),
                    radius: 200.0,
                    drill: 200.0,
                    layers: vec![0, 1],
                },
            ],
            unconnected_pairs: 0,
        },
        NetRoute {
            net_id: gnd_id,
            ..Default::default()
        },
    ];

    let drc = check_drc(&board, &routes);
    assert_eq!(drc.violations.len(), 6, "violations: {:?}", drc.violations);

    let blockers: Vec<&str> = drc
        .violations
        .iter()
        .map(|v| {
            v.blocker
                .as_deref()
                .expect("static violation needs blocker")
        })
        .collect();
    assert_eq!(blockers[0], "pad U3-1 (GND)");
    assert_eq!(drc.violations[0].net_b, Some(gnd_id));
    assert!(drc.violations[0].gap_um < drc.violations[0].required_um);
    assert_eq!(blockers[1], "keepout (structure)");
    assert_eq!(blockers[2], "keepout (image)");
    assert_eq!(blockers[3], "board_boundary");
    assert_eq!(blockers[4], "fixed_wire (GND)");
    assert_eq!(blockers[5], "fixed_via (GND)");

    let stats = RouteStats {
        total_nets: 2,
        routable_nets: 2,
        routed_nets: 2,
        unconnected_pairs: 0,
        total_segments: 5,
        total_vias: 1,
        total_wirelength_um: 3000.0,
        passes_run: 0,
        starts_run: 1,
        drc_violations: drc.violations.len(),
        dense_bga_pads: 0,
        pofv_capable_pads: 0,
        grid_step_um: 50.0,
        stopped_early: false,
        elapsed_ms: 0,
    };
    let report = RouteReport::build(&board, &routes, stats, false);
    assert_eq!(report.violations.len(), 6);
    assert_eq!(
        report.violations[0].blocker.as_deref(),
        Some("pad U3-1 (GND)")
    );
    let json = report.to_json();
    assert!(json.contains("\"blocker\":\"pad U3-1 (GND)\""));
    assert!(json.contains("\"blocker\":\"keepout (structure)\""));
    assert!(json.contains("\"blocker\":\"keepout (image)\""));
    assert!(json.contains("\"blocker\":\"board_boundary\""));
    assert!(json.contains("\"blocker\":\"fixed_wire (GND)\""));
    assert!(json.contains("\"blocker\":\"fixed_via (GND)\""));
}

#[test]
fn depopulated_interior_bga_keeps_inner_ring_and_detects_sealed() {
    // 5x5 0.40mm BGA missing center ball P2_2; POFV off and large via blocks dogbones.
    let mut pins = String::new();
    let mut nets = String::new();
    let mut class_nets = String::new();
    let mut target_pins = String::new();
    let mut idx = 0;
    for r in 0..5 {
        for c in 0..5 {
            if r == 2 && c == 2 {
                continue;
            }
            let x = (c - 2) * 400;
            let y = (r - 2) * 400;
            let pname = format!("P{r}_{c}");
            pins.push_str(&format!("      (pin BGA_PAD {pname} {x} {y})\n"));
            let tx = ((idx % 12) - 6) * 1000;
            let ty = if idx < 12 { 0 } else { 1800 };
            target_pins.push_str(&format!("      (pin HDR_300 T{idx} {tx} {ty})\n"));
            let nname = format!("/DEPOP/N{idx}");
            nets.push_str(&format!("    (net {nname} (pins U1-{pname} J1-T{idx}))\n"));
            class_nets.push(' ');
            class_nets.push_str(&nname);
            idx += 1;
        }
    }
    let dsn = format!(
        r#"(pcb depop_bga.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer In1.Cu (type power) (property (index 1)))
    (layer In2.Cu (type signal) (property (index 2)))
    (layer B.Cu (type signal) (property (index 3)))
    (boundary (rect pcb 0 0 22000 18000))
    (via "Via[0-3]_450:250_um")
    (rule (width 92) (clearance 92) (clearance 105 (type via_wire)) (clearance 105 (type via_via)) (clearance 105 (type via_smd)))
    (control (via_at_smd off))
  )
  (placement
    (component BGA_DEPOP (place U1 11000 5000 front 0))
    (component HDR_TGT (place J1 11000 13500 front 0))
  )
  (library
    (image BGA_DEPOP
{pins}    )
    (image HDR_TGT
{target_pins}    )
    (padstack BGA_PAD (shape (circle F.Cu 240)) (attach off))
    (padstack HDR_300 (shape (circle F.Cu 300)) (attach off))
    (padstack "Via[0-3]_450:250_um"
      (shape (circle F.Cu 450)) (shape (circle In1.Cu 450))
      (shape (circle In2.Cu 450)) (shape (circle B.Cu 450))
      (attach off)
    )
  )
  (network
{nets}    (class default{class_nets}
      (circuit (use_via "Via[0-3]_450:250_um"))
      (rule (width 92) (clearance 92) (clearance 105 (type via_wire)) (clearance 105 (type via_via)) (clearance 105 (type via_smd)))
    )
  )
  (wiring)
)"#
    );

    let design = DsnDesign::parse(&dsn).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), Some(false)).unwrap();
    let fanout = analyze_fanout(&board);
    assert_eq!(fanout.bga_clusters.len(), 1);
    let cluster = &fanout.bga_clusters[0];
    assert_eq!(cluster.max_ring, 1);

    // Every ball adjacent to the hollow center cavity must stay ring 1 (never ring 0) and be sealed.
    for pin in [
        "P1_2", "P2_1", "P2_3", "P3_2", "P1_1", "P1_3", "P3_1", "P3_3",
    ] {
        let pad = board
            .pads
            .iter()
            .find(|p| p.refdes == "U1" && p.pin_name == pin)
            .unwrap();
        assert_eq!(
            fanout.pads[pad.id].bga_ring,
            Some(1),
            "{pin} beside hollow center must be ring 1"
        );
        assert!(
            cluster.sealed_pad_ids.contains(&pad.id),
            "{pin} must be sealed (cannot stop inside empty cavity)"
        );
    }
    let report = run_preflight(&board, 2);
    assert!(!report.ok);
    assert_eq!(report.sealed_pads.len(), 8);
}

#[test]
fn multi_ball_interior_net_bga_requires_exit_outside_envelope() {
    // 5x5 0.40mm BGA where inner balls P2_2 and P2_3 share one net, blocked by outer P2_4.
    let mut pins = String::new();
    let mut nets = String::new();
    let mut class_nets = String::new();
    let mut target_pins = String::new();
    let mut idx = 0;
    let mut shared_pins = vec!["J1-TS".to_string()];
    target_pins.push_str("      (pin HDR_300 TS 0 -1200)\n");

    for r in 0..5 {
        for c in 0..5 {
            let x = (c - 2) * 400;
            let y = (r - 2) * 400;
            let pname = format!("P{r}_{c}");
            pins.push_str(&format!("      (pin BGA_PAD {pname} {x} {y})\n"));
            if r == 2 && (c == 2 || c == 3) {
                shared_pins.push(format!("U1-{pname}"));
                continue;
            }
            let tx = ((idx % 12) - 6) * 1000;
            let ty = if idx < 12 { 0 } else { 1800 };
            target_pins.push_str(&format!("      (pin HDR_300 T{idx} {tx} {ty})\n"));
            let nname = format!("/MB/N{idx}");
            nets.push_str(&format!("    (net {nname} (pins U1-{pname} J1-T{idx}))\n"));
            class_nets.push(' ');
            class_nets.push_str(&nname);
            idx += 1;
        }
    }
    nets.push_str(&format!(
        "    (net /MB/SHARED (pins {}))\n",
        shared_pins.join(" ")
    ));
    class_nets.push_str(" /MB/SHARED");

    let dsn = format!(
        r#"(pcb multi_ball.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer In1.Cu (type power) (property (index 1)))
    (layer In2.Cu (type signal) (property (index 2)))
    (layer B.Cu (type signal) (property (index 3)))
    (boundary (rect pcb 0 0 22000 18000))
    (via "Via[0-3]_450:250_um")
    (rule (width 92) (clearance 92) (clearance 105 (type via_wire)) (clearance 105 (type via_via)) (clearance 105 (type via_smd)))
    (control (via_at_smd off))
  )
  (placement
    (component BGA_MB (place U1 11000 5000 front 0))
    (component HDR_TGT (place J1 11000 13500 front 0))
  )
  (library
    (image BGA_MB
{pins}    )
    (image HDR_TGT
{target_pins}    )
    (padstack BGA_PAD (shape (circle F.Cu 240)) (attach off))
    (padstack HDR_300 (shape (circle F.Cu 300)) (attach off))
    (padstack "Via[0-3]_450:250_um"
      (shape (circle F.Cu 450)) (shape (circle In1.Cu 450))
      (shape (circle In2.Cu 450)) (shape (circle B.Cu 450))
      (attach off)
    )
  )
  (network
{nets}    (class default{class_nets}
      (circuit (use_via "Via[0-3]_450:250_um"))
      (rule (width 92) (clearance 92) (clearance 105 (type via_wire)) (clearance 105 (type via_via)) (clearance 105 (type via_smd)))
    )
  )
  (wiring)
)"#
    );

    let design = DsnDesign::parse(&dsn).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), Some(false)).unwrap();
    let report = run_preflight(&board, 2);
    assert!(!report.ok);
    for pin in ["P2_2", "P2_3"] {
        assert!(
            report.sealed_pads.iter().any(|s| s.pin == pin),
            "interior multi-ball net pad {pin} must be detected as sealed: {:?}",
            report.sealed_pads
        );
    }
}

#[test]
fn multi_segment_no_net_fixed_wire_passes_preflight() {
    // Contiguous multi-segment no-net wire + joined no-net wire + no-net pad & via must pass.
    let dsn_clean = r#"(pcb nonet_wire_clean.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 12000 8000))
    (via "Via[0-1]_400:200_um")
    (rule (width 200) (clearance 200))
  )
  (placement
    (component HDR2
      (place J1 1500 2000 front 0)
      (place J2 10500 2000 front 0)
    )
    (component NPAD
      (place M1 2000 6000 front 0)
    )
  )
  (library
    (image HDR2 (pin SMD_500 1 0 0))
    (image NPAD (pin SMD_500 1 0 0))
    (padstack SMD_500 (shape (circle F.Cu 500)) (attach off))
    (padstack "Via[0-1]_400:200_um"
      (shape (circle F.Cu 400)) (shape (circle B.Cu 400))
      (attach off)
    )
  )
  (network
    (net /SIG (pins J1-1 J2-1))
    (class default /SIG
      (circuit (use_via "Via[0-1]_400:200_um"))
      (rule (width 200) (clearance 200))
    )
  )
  (wiring
    (wire (path F.Cu 200 2000 6000 4000 6000 6000 6000 6000 5000) (net "") (type fix))
    (wire (path F.Cu 200 6000 5000 8000 5000) (net "") (type fix))
    (via "Via[0-1]_400:200_um" 8000 5000 (net "") (type fix))
  )
)"#;
    let design = DsnDesign::parse(dsn_clean).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    let report = run_preflight(&board, 2);
    assert!(
        report.fixed_violations.is_empty(),
        "contiguous no-net polyline/pad/via must pass: {:?}",
        report.fixed_violations
    );
    assert!(report.ok, "clean no-net board must pass: {report:?}");

    // Separate parallel no-net wire 50um away (no shared endpoints) must still fail.
    let dsn_bad = dsn_clean.replace(
        "(via \"Via[0-1]_400:200_um\" 8000 5000 (net \"\") (type fix))",
        "(via \"Via[0-1]_400:200_um\" 8000 5000 (net \"\") (type fix))\n    (wire (path F.Cu 200 2500 6250 3500 6250) (net \"\") (type fix))",
    );
    let design_bad = DsnDesign::parse(&dsn_bad).unwrap();
    let board_bad = Board::from_dsn(&design_bad, &HashSet::new(), None).unwrap();
    let report_bad = run_preflight(&board_bad, 2);
    assert!(
        !report_bad.fixed_violations.is_empty(),
        "parallel non-touching no-net fixed wires violating clearance must fail"
    );
}

#[test]
fn mixed_net_class_fixed_copper_violation_detected() {
    // LOW_V (100um clr) fixed wires sit 300um gap from HV (500um clr) fixed wire & fixed via.
    let dsn = r#"(pcb mixed_hv_fixed.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 14000 10000))
    (via "Via[0-1]_400:200_um")
    (rule (width 200) (clearance 100))
  )
  (placement
    (component P1
      (place U1 1500 1500 front 0)
      (place U2 12500 1500 front 0)
      (place U3 1500 8500 front 0)
      (place U4 12500 8500 front 0)
    )
  )
  (library
    (image P1 (pin SMD_400 1 0 0))
    (padstack SMD_400 (shape (circle F.Cu 400)) (attach off))
    (padstack "Via[0-1]_400:200_um"
      (shape (circle F.Cu 400)) (shape (circle B.Cu 400))
      (attach off)
    )
  )
  (network
    (net /LOW_V (pins U1-1 U2-1))
    (net /HV (pins U3-1 U4-1))
    (class LOW_CLASS /LOW_V
      (circuit (use_via "Via[0-1]_400:200_um"))
      (rule (width 200) (clearance 100) (clearance 100 (type via_wire)) (clearance 100 (type via_via)))
    )
    (class HV_CLASS /HV
      (circuit (use_via "Via[0-1]_400:200_um"))
      (rule (width 200) (clearance 500) (clearance 500 (type via_wire)) (clearance 500 (type via_via)))
    )
  )
  (wiring
    (wire (path F.Cu 200 3000 4000 6000 4000) (net /LOW_V) (type fix))
    (wire (path F.Cu 200 3000 4500 6000 4500) (net /HV) (type fix))
    (wire (path F.Cu 200 9000 4000 11000 4000) (net /LOW_V) (type fix))
    (via "Via[0-1]_400:200_um" 10000 4600 (net /HV) (type fix))
  )
)"#;
    let design = DsnDesign::parse(dsn).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    let report = run_preflight(&board, 2);

    assert!(!report.ok);
    assert!(
        report
            .fixed_violations
            .iter()
            .any(|v| v.net_a.as_deref() == Some("/LOW_V") && v.blocker == "fixed_wire (/HV)"),
        "expected LOW_V fixed wire vs HV fixed wire violation: {:?}",
        report.fixed_violations
    );
    assert!(
        report
            .fixed_violations
            .iter()
            .any(|v| v.net_a.as_deref() == Some("/LOW_V") && v.blocker == "fixed_via (/HV)"),
        "expected LOW_V fixed wire vs HV fixed via violation: {:?}",
        report.fixed_violations
    );
}

#[test]
fn fixed_200um_gnd_thermal_via_array_and_keepout_pass_preflight() {
    // 200um hole-to-hole GND via array (400um c2c, 200um drill), disjoint blind vias, and cutout wire.
    let dsn = r#"(pcb thermal_vias.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer In1.Cu (type signal) (property (index 1)))
    (layer In2.Cu (type signal) (property (index 2)))
    (layer B.Cu (type signal) (property (index 3)))
    (boundary (rect pcb 0 0 12000 8000))
    (keepout "" (rect signal 7000 5000 8000 6000))
    (via "Via[0-3]_350:200_um")
    (rule (width 150) (clearance 150))
  )
  (placement
    (component P1
      (place U1 1500 1500 front 0)
      (place U2 10500 1500 front 0)
    )
  )
  (library
    (image P1 (pin SMD_400 1 0 0))
    (padstack SMD_400 (shape (circle F.Cu 400)) (attach off))
    (padstack "Via[0-3]_350:200_um"
      (shape (circle F.Cu 350)) (shape (circle In1.Cu 350))
      (shape (circle In2.Cu 350)) (shape (circle B.Cu 350))
      (attach off)
    )
    (padstack "Via[0-1]_300:150_um"
      (shape (circle F.Cu 300)) (shape (circle In1.Cu 300))
      (attach off)
    )
    (padstack "Via[2-3]_300:150_um"
      (shape (circle In2.Cu 300)) (shape (circle B.Cu 300))
      (attach off)
    )
  )
  (network
    (net /SIG (pins U1-1 U2-1))
    (net GND (pins))
    (class default /SIG GND
      (circuit (use_via "Via[0-3]_350:200_um"))
      (rule (width 150) (clearance 150))
    )
  )
  (wiring
    (via "Via[0-3]_350:200_um" 3000 5000 (net GND) (type fix))
    (via "Via[0-3]_350:200_um" 3400 5000 (net GND) (type fix))
    (via "Via[0-3]_350:200_um" 3000 5400 (net GND) (type fix))
    (via "Via[0-3]_350:200_um" 3400 5400 (net GND) (type fix))
    (via "Via[0-1]_300:150_um" 5000 5000 (net GND) (type fix))
    (via "Via[2-3]_300:150_um" 5150 5000 (net GND) (type fix))
    (wire (path F.Cu 150 6800 4895 8200 4895) (net GND) (type fix))
  )
)"#;
    let design = DsnDesign::parse(dsn).unwrap();
    let mut ignored = HashSet::new();
    ignored.insert("GND".to_string());
    let board = Board::from_dsn(&design, &ignored, None).unwrap();
    let report = run_preflight(&board, 2);
    assert!(
        report.fixed_violations.is_empty(),
        "valid 200um GND thermal vias, staggered blind vias, and cutout-adjacent fixed wire must pass: {:?}",
        report.fixed_violations
    );
    assert!(report.ok);
}

#[test]
fn dual_sided_component_clusters_top_and_bottom_independently() {
    let mut pins = String::new();
    let mut nets = String::new();
    let mut class_nets = String::new();
    let mut target_pins = String::new();
    let mut idx = 0;

    for (side, ps, prefix) in [("F", "PAD_F", "T"), ("B", "PAD_B", "B")] {
        for r in 0..3 {
            for c in 0..3 {
                let x = (c - 1) * 400;
                let y = (r - 1) * 400;
                let pname = format!("{side}{r}_{c}");
                pins.push_str(&format!("      (pin {ps} {pname} {x} {y})\n"));
                let tx = ((idx % 9) - 4) * 1000;
                let ty = if side == "F" { 0 } else { 1500 };
                let tps = if side == "F" { "HDR_F" } else { "HDR_B" };
                target_pins.push_str(&format!("      (pin {tps} {prefix}{idx} {tx} {ty})\n"));
                let nname = format!("/DUAL/N{idx}");
                nets.push_str(&format!(
                    "    (net {nname} (pins U1-{pname} J1-{prefix}{idx}))\n"
                ));
                class_nets.push(' ');
                class_nets.push_str(&nname);
                idx += 1;
            }
        }
    }

    let dsn = format!(
        r#"(pcb dual_sided.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer In1.Cu (type signal) (property (index 1)))
    (layer In2.Cu (type signal) (property (index 2)))
    (layer B.Cu (type signal) (property (index 3)))
    (boundary (rect pcb 0 0 18000 14000))
    (via "Via[0-3]_160:80_um")
    (rule (width 80) (clearance 80) (clearance 80 (type via_wire)) (clearance 80 (type via_via)) (clearance 80 (type via_smd)))
    (control (via_at_smd off))
  )
  (placement
    (component DUAL_MOD (place U1 9000 4500 front 0))
    (component HDR_TGT (place J1 9000 10500 front 0))
  )
  (library
    (image DUAL_MOD
{pins}    )
    (image HDR_TGT
{target_pins}    )
    (padstack PAD_F (shape (circle F.Cu 180)) (attach off))
    (padstack PAD_B (shape (circle B.Cu 180)) (attach off))
    (padstack HDR_F (shape (circle F.Cu 300)) (attach off))
    (padstack HDR_B (shape (circle B.Cu 300)) (attach off))
    (padstack "Via[0-3]_160:80_um"
      (shape (circle F.Cu 160)) (shape (circle In1.Cu 160))
      (shape (circle In2.Cu 160)) (shape (circle B.Cu 160))
      (attach off)
    )
  )
  (network
{nets}    (class default{class_nets}
      (circuit (use_via "Via[0-3]_160:80_um"))
      (rule (width 80) (clearance 80) (clearance 80 (type via_wire)) (clearance 80 (type via_via)) (clearance 80 (type via_smd)))
    )
  )
  (wiring)
)"#
    );

    let design = DsnDesign::parse(&dsn).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), Some(false)).unwrap();
    let fanout = analyze_fanout(&board);
    assert_eq!(
        fanout.bga_clusters.len(),
        2,
        "dual-sided U1 must produce one cluster per side: {:?}",
        fanout.bga_clusters
    );
    let layers: Vec<usize> = fanout.bga_clusters.iter().map(|c| c.pad_layer).collect();
    assert_eq!(layers, vec![0, 3]);
    for c in &fanout.bga_clusters {
        assert_eq!(c.refdes, "U1");
        assert_eq!(c.pad_ids.len(), 9);
        assert_eq!(c.max_ring, 1);
    }
}

#[test]
fn non_bga_0201_and_two_row_qfn_skip_bga_clusters() {
    // Dense 0.40mm 0201 passive (2 pads), 1-pad TP, and 2x10 QFN/FPC connector (20 pads).
    let mut fpc_pins = String::new();
    let mut tgt_pins = String::new();
    let mut nets = String::new();
    let mut class_nets = String::new();
    let mut idx = 0;

    for row in 0..2 {
        for col in 0..10 {
            let x = (col - 5) * 400;
            let y = row * 400;
            let pname = format!("P{row}_{col}");
            fpc_pins.push_str(&format!("      (pin FINE_PAD {pname} {x} {y})\n"));
            let tx = ((idx % 12) - 6) * 1000;
            let ty = if idx < 12 { 0 } else { 1800 };
            tgt_pins.push_str(&format!("      (pin HDR_300 T{idx} {tx} {ty})\n"));
            let nname = format!("/FPC/N{idx}");
            nets.push_str(&format!("    (net {nname} (pins J2-{pname} J1-T{idx}))\n"));
            class_nets.push(' ');
            class_nets.push_str(&nname);
            idx += 1;
        }
    }
    tgt_pins.push_str("      (pin HDR_300 R1A -5000 3200)\n");
    tgt_pins.push_str("      (pin HDR_300 R1B -3000 3200)\n");
    tgt_pins.push_str("      (pin HDR_300 TP1T -1000 3200)\n");
    nets.push_str("    (net /R1/A (pins R1-1 J1-R1A))\n");
    nets.push_str("    (net /R1/B (pins R1-2 J1-R1B))\n");
    nets.push_str("    (net /TP/1 (pins TP1-1 J1-TP1T))\n");
    class_nets.push_str(" /R1/A /R1/B /TP/1");

    let dsn = format!(
        r#"(pcb non_bga_dense.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 22000 18000))
    (via "Via[0-1]_250:150_um")
    (rule (width 92) (clearance 92) (clearance 105 (type via_wire)) (clearance 105 (type via_via)) (clearance 105 (type via_smd)))
    (control (via_at_smd off))
  )
  (placement
    (component FPC2X10 (place J2 11000 4000 front 0))
    (component R0201 (place R1 4000 4000 front 0))
    (component TP1_IMG (place TP1 4000 6000 front 0))
    (component HDR_TGT (place J1 11000 12000 front 0))
  )
  (library
    (image FPC2X10
{fpc_pins}    )
    (image R0201
      (pin FINE_PAD 1 -200 0)
      (pin FINE_PAD 2 200 0)
    )
    (image TP1_IMG (pin FINE_PAD 1 0 0))
    (image HDR_TGT
{tgt_pins}    )
    (padstack FINE_PAD (shape (circle F.Cu 240)) (attach off))
    (padstack HDR_300 (shape (circle F.Cu 300)) (attach off))
    (padstack "Via[0-1]_250:150_um"
      (shape (circle F.Cu 250)) (shape (circle B.Cu 250))
      (attach off)
    )
  )
  (network
{nets}    (class default{class_nets}
      (circuit (use_via "Via[0-1]_250:150_um"))
      (rule (width 92) (clearance 92) (clearance 105 (type via_wire)) (clearance 105 (type via_via)) (clearance 105 (type via_smd)))
    )
  )
  (wiring)
)"#
    );

    let design = DsnDesign::parse(&dsn).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), Some(false)).unwrap();
    let fanout = analyze_fanout(&board);
    assert!(
        fanout.bga_clusters.is_empty(),
        "0201, 1-pad TP, and 2-row QFN/FPC must not enter bga_clusters: {:?}",
        fanout.bga_clusters
    );
    let report = run_preflight(&board, 2);
    assert!(report.sealed_pads.is_empty());
    assert!(report.bga_clusters.is_empty());
}

#[test]
fn rotated_45_degree_bga_detects_rings_and_dogbones() {
    // 5x5 0.40mm BGA rotated 45 deg with microvia (160:80) and POFV off.
    let dsn_rot = dense_bga_dsn(5, 400, 180, 160, 80, 80, 80, 80, false, "").replace(
        "(component BGA_GRID (place U1 11000 5000 front 0))",
        "(component BGA_GRID (place U1 11000 5000 front 45))",
    );
    let design = DsnDesign::parse(&dsn_rot).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), Some(false)).unwrap();
    let fanout = analyze_fanout(&board);

    assert_eq!(fanout.bga_clusters.len(), 1);
    let cluster = &fanout.bga_clusters[0];
    assert_eq!(cluster.pad_ids.len(), 25);
    assert!((cluster.pitch_um - 400.0).abs() < 1.0);
    assert_eq!(cluster.max_ring, 2);
    assert!(
        cluster.sealed_pad_ids.is_empty(),
        "45-deg rotated 5x5 dogbone BGA must have no sealed pads: {:?}",
        cluster.sealed_pad_ids
    );
    let mut ring_counts = [0usize; 3];
    for &pid in &cluster.pad_ids {
        ring_counts[fanout.pads[pid].bga_ring.unwrap()] += 1;
    }
    assert_eq!(ring_counts, [16, 8, 1]);
    let report = run_preflight(&board, 2);
    assert!(
        report.ok,
        "45-deg rotated BGA must pass preflight: {report:?}"
    );

    // 3x3 0.40mm 45-deg rotated BGA with large via (250:150) and POFV off seals center P1_1.
    let dsn_rot_sealed = dense_bga_dsn(3, 400, 240, 250, 150, 92, 92, 105, false, "").replace(
        "(component BGA_GRID (place U1 11000 5000 front 0))",
        "(component BGA_GRID (place U1 11000 5000 front 45))",
    );
    let design_s = DsnDesign::parse(&dsn_rot_sealed).unwrap();
    let board_s = Board::from_dsn(&design_s, &HashSet::new(), Some(false)).unwrap();
    let report_s = run_preflight(&board_s, 2);
    assert!(!report_s.ok);
    assert_eq!(report_s.sealed_pads.len(), 1);
    assert_eq!(report_s.sealed_pads[0].pin, "P1_1");
    assert_eq!(report_s.sealed_pads[0].ring, 1);
}

#[test]
fn blind_microvia_via_layers_filters_escape_layers() {
    // 6-layer 5x5 BGA where the blind microvia only spans F.Cu -> In2.Cu (layers 0, 1, 2).
    let base = dense_bga_dsn(5, 400, 220, 150, 75, 70, 75, 75, true, "");
    let dsn = base
        .replace(
            "(shape (circle In3.Cu 150))\n      (shape (circle In4.Cu 150))\n      (shape (circle B.Cu 150))\n",
            "",
        )
        .replace("Via[0-5]_150:75_um", "Via[0-2]_150:75_um");
    let design = DsnDesign::parse(&dsn).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), Some(true)).unwrap();
    let fanout = analyze_fanout(&board);

    assert_eq!(fanout.bga_clusters.len(), 1);
    for &pid in &fanout.bga_clusters[0].pad_ids {
        let pf = &fanout.pads[pid];
        let r = pf.bga_ring.unwrap();
        if r == 0 {
            assert_eq!(pf.preferred_escape_layer, Some(0));
        } else {
            assert_eq!(
                pf.preferred_escape_layer,
                Some(2),
                "ring {r} blind microvia [0-2] must only target In2.Cu (2), not In4.Cu/B.Cu"
            );
        }
    }

    // Buried via [In2.Cu, In4.Cu] cannot reach F.Cu pads at all -> inner balls are sealed.
    let buried = base
        .replace(
            "(shape (circle F.Cu 150))\n      (shape (circle In1.Cu 150))\n",
            "",
        )
        .replace("(shape (circle B.Cu 150))\n", "")
        .replace("Via[0-5]_150:75_um", "Via[2-4]_150:75_um");
    let design_b = DsnDesign::parse(&buried).unwrap();
    let board_b = Board::from_dsn(&design_b, &HashSet::new(), Some(true)).unwrap();
    let report_b = run_preflight(&board_b, 2);
    assert!(!report_b.ok);
    assert_eq!(report_b.sealed_pads.len(), 9);
    assert!(
        report_b
            .sealed_pads
            .iter()
            .all(|s| s.blocker == "no_other_signal_layer")
    );
}

#[test]
fn adjacent_pofv_clearance_conflict_flags_sealed_with_pad_blocker() {
    // 4x4 0.40mm BGA with 250:200 POFV vias clears pads but fails 205um pairwise drill spacing.
    let dsn = dense_bga_dsn(4, 400, 220, 250, 200, 80, 80, 80, true, "");
    let design = DsnDesign::parse(&dsn).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), Some(true)).unwrap();
    let report = run_preflight(&board, 2);

    assert!(!report.ok);
    assert!(
        !report.sealed_pads.is_empty(),
        "adjacent POFV conflict must seal conflicting inner balls"
    );
    assert!(
        report
            .sealed_pads
            .iter()
            .any(|s| s.blocker.contains("adjacent POFV pad U1-")),
        "blocker must name the conflicting adjacent POFV pad: {:?}",
        report.sealed_pads
    );
}

#[test]
fn preflight_honors_timeout_and_stop_flag() {
    let design = DsnDesign::parse(clean_two_layer_dsn()).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();

    let rep_timeout = run_preflight_with_options(&board, 2, Some(0), None);
    assert!(rep_timeout.stopped_early);
    assert!(!rep_timeout.ok);
    assert!(rep_timeout.to_json().contains("\"stopped_early\": true"));

    let stop = Arc::new(AtomicBool::new(true));
    let rep_stop = run_preflight_with_options(&board, 2, None, Some(stop));
    assert!(rep_stop.stopped_early);
    assert!(!rep_stop.ok);
}
