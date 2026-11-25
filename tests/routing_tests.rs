use std::collections::HashSet;
use std::ffi::{CStr, CString};
use std::ptr;
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};

use viahop::ffi::{viahop_free_str, viahop_route_file, viahop_route_str, viahop_version};
use viahop::geometry::Point;
use viahop::optimize::{optimize_routes, route_wirelength};
use viahop::{
    check_drc, parse_ses, route, route_with_checkpoint, write_ses, Board, DsnDesign, NetRoute,
    RouteReport, RouteScore, RoutedSegment, RoutedVia, RouterConfig, UnroutedKind,
};

fn two_layer_crossing_dsn() -> &'static str {
    r#"(pcb crossing.dsn
  (parser
    (string_quote ")
    (space_in_quoted_tokens on)
    (host_cad "KiCad's Pcbnew")
    (host_version "10.0.1")
  )
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 10000 10000))
    (keepout "" (rect signal 4500 4500 5500 5500))
    (via "Via[0-1]_400:200_um")
    (rule (width 200) (clearance 200))
  )
  (placement
    (component HDR2
      (place J1 1500 5000 front 0)
      (place J2 8500 5000 front 0)
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

fn four_layer_power_plane_dsn() -> &'static str {
    r#"(pcb four_layer.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer In1.Cu (type power) (property (index 1)))
    (layer In2.Cu (type signal) (property (index 2)))
    (layer B.Cu (type signal) (property (index 3)))
    (boundary (rect pcb 0 0 12000 8000))
    (via "Via[0-3]_300:150_um")
    (rule (width 150) (clearance 150))
  )
  (placement
    (component P2
      (place U1 2000 4000 front 0)
      (place U2 10000 4000 back 180)
    )
  )
  (library
    (image P2
      (pin P400 1 0 -1000)
      (pin P400 2 0 1000)
    )
    (padstack P400 (shape (rect F.Cu -200 -200 200 200)) (attach off))
    (padstack "Via[0-3]_300:150_um"
      (shape (circle F.Cu 300))
      (shape (circle In1.Cu 300))
      (shape (circle In2.Cu 300))
      (shape (circle B.Cu 300))
      (attach off)
    )
  )
  (network
    (net SIG1 (pins U1-1 U2-1))
    (net SIG2 (pins U1-2 U2-2))
    (net EMPTY_NET (pins))
    (class default SIG1 SIG2 (circuit (use_via "Via[0-3]_300:150_um")) (rule (width 150) (clearance 150)))
  )
  (wiring)
)"#
}

fn blocked_and_open_dsn() -> &'static str {
    // Full vertical keepout wall isolates X > 6000 from X < 4000 for /Net/BLOCKED,
    // while /Net/OPEN stays cleanly on the left side.
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

fn dense_040mm_bga_pofv_dsn() -> String {
    // 3x3 inner BGA at 0.40 mm pitch (pad dia 240 um, via dia 250 um, trace 92 um, clr 92/105 um)
    let mut pins = String::new();
    let mut nets = String::new();
    let mut class_nets = String::new();
    let mut target_pins = String::new();
    let mut idx = 0;
    for r in 0..3 {
        for c in 0..3 {
            let x = (c - 1) * 400;
            let y = (r - 1) * 400;
            let pname = format!("P{r}_{c}");
            pins.push_str(&format!("      (pin BGA_240 {pname} {x} {y})\n"));
            let tx = (idx - 4) * 800;
            target_pins.push_str(&format!("      (pin HDR_300 T{idx} {tx} 0)\n"));
            let nname = format!("/BGA/N{idx}");
            nets.push_str(&format!("    (net {nname} (pins U1-{pname} J1-T{idx}))\n"));
            class_nets.push(' ');
            class_nets.push_str(&nname);
            idx += 1;
        }
    }
    format!(
        r#"(pcb bga_pofv.dsn
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
    (boundary (rect pcb 0 0 14000 12000))
    (via "Via[0-5]_250:150_um")
    (rule
      (width 92)
      (clearance 92)
      (clearance 105 (type via_wire))
      (clearance 105 (type via_via))
      (clearance 105 (type via_smd))
    )
    (control (via_at_smd on))
  )
  (placement
    (component BGA3X3 (place U1 7000 4000 front 0))
    (component HDR9 (place J1 7000 9500 front 0))
  )
  (library
    (image BGA3X3
{pins}    )
    (image HDR9
{target_pins}    )
    (padstack BGA_240 (shape (circle F.Cu 240)) (attach off))
    (padstack HDR_300 (shape (circle F.Cu 300)) (attach off))
    (padstack "Via[0-5]_250:150_um"
      (shape (circle F.Cu 250))
      (shape (circle In1.Cu 250))
      (shape (circle In2.Cu 250))
      (shape (circle In3.Cu 250))
      (shape (circle In4.Cu 250))
      (shape (circle B.Cu 250))
      (attach off)
    )
  )
  (network
{nets}    (class kicad_default{class_nets}
      (circuit (use_via "Via[0-5]_250:150_um"))
      (rule (width 92) (clearance 92) (clearance 105 (type via_wire)) (clearance 105 (type via_via)) (clearance 105 (type via_smd)))
    )
  )
  (wiring)
)"#
    )
}

#[test]
fn routes_two_layer_crossing_around_keepout_with_zero_drc() {
    let design = DsnDesign::parse(two_layer_crossing_dsn()).unwrap();
    let cfg = RouterConfig {
        max_passes: 15,
        ..Default::default()
    };
    let res = route(&design, &cfg).unwrap();
    assert_eq!(res.stats.routable_nets, 2);
    assert_eq!(res.stats.routed_nets, 2);
    assert_eq!(res.stats.unconnected_pairs, 0);
    assert!(res.stats.total_wirelength_um > 0.0);
    let drc = check_drc(&res.board, &res.routes);
    assert_eq!(drc.violations.len(), 0, "DRC violations: {:?}", drc.violations);

    let ses = write_ses(&design, &res);
    assert!(ses.contains("(session crossing"));
    assert!(ses.contains("(was_is"));
    assert!(ses.contains("(library_out"));
    assert!(ses.contains("(network_out"));
    assert!(ses.contains("(net /Net/A"));
    assert!(ses.contains("(net /Net/B"));
    assert!(ses.contains("(resolution um 10)"));
}

#[test]
fn respects_power_layer_and_back_side_component_transform() {
    let design = DsnDesign::parse(four_layer_power_plane_dsn()).unwrap();
    let mut cfg = RouterConfig::default();
    cfg.ignored_nets.insert("SIG2".to_string());
    let res = route(&design, &cfg).unwrap();

    assert_eq!(res.stats.routable_nets, 1);
    assert_eq!(res.stats.routed_nets, 1);
    assert_eq!(res.stats.drc_violations, 0);

    for nr in &res.routes {
        for seg in &nr.segments {
            assert_ne!(seg.layer, 1, "must never route on power layer In1.Cu");
        }
    }
    assert!(res.stats.total_vias >= 1);
}

#[test]
fn dense_040mm_bga_routes_inner_ball_via_pofv() {
    let dsn_str = dense_040mm_bga_pofv_dsn();
    let design = DsnDesign::parse(&dsn_str).unwrap();
    let cfg = RouterConfig {
        max_passes: 20,
        via_at_smd: Some(true),
        ..Default::default()
    };
    let res = route(&design, &cfg).unwrap();
    assert_eq!(res.stats.routable_nets, 9);
    assert_eq!(res.stats.routed_nets, 9, "all 9 BGA balls must route");
    assert_eq!(res.stats.unconnected_pairs, 0);
    assert!(res.stats.dense_bga_pads >= 1, "must detect dense inner BGA pads");
    assert!(res.stats.pofv_capable_pads >= 9, "all 9 BGA pads should allow centered POFV");
    assert_eq!(res.stats.drc_violations, 0, "POFV vias must produce 0 DRC violations");
}

#[test]
fn applies_external_rules_file_and_mil_units() {
    let mil_dsn = r#"(pcb mil_test.dsn
      (parser (string_quote "))
      (resolution mil 10)
      (unit mil)
      (structure
        (layer F.Cu (type signal) (property (index 0)))
        (boundary (rect pcb 0 0 500 500))
        (rule (width 10) (clearance 10))
      )
      (placement (component R (place R1 100 100 front 0) (place R2 400 100 front 0)))
      (library
        (image R (pin P 1 0 0))
        (padstack P (shape (circle F.Cu 20)) (attach off))
      )
      (network (net N1 (pins R1-1 R2-1)))
      (wiring)
    )"#;
    let mut design = DsnDesign::parse(mil_dsn).unwrap();
    assert!((design.default_rules.width.unwrap() - 254.0).abs() < 1e-3);
    design.apply_rules_file("(rules (rule (width 8) (clearance 8)))").unwrap();
    assert!((design.default_rules.width.unwrap() - 203.2).abs() < 1e-3);
    let res = route(&design, &RouterConfig::default()).unwrap();
    assert_eq!(res.stats.routed_nets, 1);
    assert_eq!(res.stats.drc_violations, 0);
}

#[test]
fn ses_roundtrip_and_initial_session_resume() {
    let design = DsnDesign::parse(two_layer_crossing_dsn()).unwrap();

    // First route only /Net/A to create a partial checkpoint session.
    let mut partial_cfg = RouterConfig::default();
    partial_cfg.ignored_nets.insert("/Net/B".to_string());
    let partial_res = route(&design, &partial_cfg).unwrap();
    assert_eq!(partial_res.stats.routed_nets, 1);
    let partial_ses = write_ses(&design, &partial_res);

    // Parse partial SES back onto the full board and verify round-trip geometry.
    let full_board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    let parsed_routes = parse_ses(&partial_ses, &full_board).unwrap();
    let net_a_id = full_board.net_name_to_id["/Net/A"];
    let net_b_id = full_board.net_name_to_id["/Net/B"];
    assert!(!parsed_routes[net_a_id].segments.is_empty());
    assert!(parsed_routes[net_b_id].segments.is_empty());

    // Resume routing with the partial session; /Net/B completes with 0 DRC.
    let resume_cfg = RouterConfig {
        max_passes: 15,
        initial_routes: Some(parsed_routes),
        ..Default::default()
    };
    let full_res = route(&design, &resume_cfg).unwrap();
    assert_eq!(full_res.stats.routed_nets, 2);
    assert_eq!(full_res.stats.unconnected_pairs, 0);
    assert_eq!(full_res.stats.drc_violations, 0);

    // Resume a completed session with 0 passes (verify/optimize only).
    let full_ses = write_ses(&design, &full_res);
    let reparsed = parse_ses(&full_ses, &full_board).unwrap();
    let opt_only_cfg = RouterConfig {
        max_passes: 0,
        initial_routes: Some(reparsed),
        ..Default::default()
    };
    let opt_res = route(&design, &opt_only_cfg).unwrap();
    assert_eq!(opt_res.stats.passes_run, 0);
    assert_eq!(opt_res.stats.routed_nets, 2);
    assert_eq!(opt_res.stats.drc_violations, 0);
}

#[test]
fn optimizer_pulls_tight_and_eliminates_redundant_via_pair() {
    let open_dsn = r#"(pcb opt_test.dsn
      (parser (string_quote "))
      (resolution um 10)
      (unit um)
      (structure
        (layer F.Cu (type signal) (property (index 0)))
        (layer B.Cu (type signal) (property (index 1)))
        (boundary (rect pcb 0 0 10000 6000))
        (via "Via[0-1]_400:200_um")
        (rule (width 200) (clearance 200))
      )
      (placement
        (component P1 (place J1 1500 3000 front 0) (place J2 8500 3000 front 0))
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
        (net N1 (pins J1-1 J2-1))
        (class default N1 (circuit (use_via "Via[0-1]_400:200_um")) (rule (width 200) (clearance 200)))
      )
      (wiring)
    )"#;
    let design = DsnDesign::parse(open_dsn).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();

    // Deliberately jagged staircase + unnecessary F.Cu -> B.Cu -> F.Cu hop on an open board.
    let mut routes = vec![NetRoute {
        net_id: 0,
        segments: vec![
            RoutedSegment { layer: 0, a: Point::new(1500.0, 3000.0), b: Point::new(2500.0, 3000.0), width: 200.0 },
            RoutedSegment { layer: 0, a: Point::new(2500.0, 3000.0), b: Point::new(2500.0, 4200.0), width: 200.0 },
            RoutedSegment { layer: 0, a: Point::new(2500.0, 4200.0), b: Point::new(3500.0, 4200.0), width: 200.0 },
            RoutedSegment { layer: 0, a: Point::new(3500.0, 4200.0), b: Point::new(3500.0, 3000.0), width: 200.0 },
            RoutedSegment { layer: 1, a: Point::new(3500.0, 3000.0), b: Point::new(6500.0, 3000.0), width: 200.0 },
            RoutedSegment { layer: 0, a: Point::new(6500.0, 3000.0), b: Point::new(8500.0, 3000.0), width: 200.0 },
        ],
        vias: vec![
            RoutedVia {
                pos: Point::new(3500.0, 3000.0),
                padstack: "Via[0-1]_400:200_um".to_string(),
                radius: 200.0,
                drill: 200.0,
                layers: vec![0, 1],
            },
            RoutedVia {
                pos: Point::new(6500.0, 3000.0),
                padstack: "Via[0-1]_400:200_um".to_string(),
                radius: 200.0,
                drill: 200.0,
                layers: vec![0, 1],
            },
        ],
        unconnected_pairs: 0,
    }];

    let before_len = route_wirelength(&routes[0]);
    optimize_routes(&board, &mut routes);
    let after_len = route_wirelength(&routes[0]);

    assert_eq!(routes[0].vias.len(), 0, "redundant via pair must be eliminated");
    assert!(after_len < before_len - 1000.0, "pull-tight should shorten staircase detour");
    assert_eq!(routes[0].segments.len(), 1, "straight open route should collapse to 1 segment");
    assert_eq!(check_drc(&board, &routes).violations.len(), 0);
}

#[test]
fn multi_start_skips_when_perfect_and_explores_when_unrouted() {
    let easy = DsnDesign::parse(two_layer_crossing_dsn()).unwrap();
    let easy_res = route(
        &easy,
        &RouterConfig {
            multi_start: 4,
            threads: 2,
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(easy_res.stats.starts_run, 1, "perfect start 0 must skip extra starts");

    let hard = DsnDesign::parse(blocked_and_open_dsn()).unwrap();
    let hard_res = route(
        &hard,
        &RouterConfig {
            max_passes: 3,
            multi_start: 3,
            threads: 2,
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(hard_res.stats.starts_run, 3, "unrouted start 0 should run all multi-start seeds");
    assert_eq!(hard_res.stats.routed_nets, 1);
    assert_eq!(hard_res.stats.unconnected_pairs, 1);
}

#[test]
fn checkpoint_callback_and_stop_flag() {
    let design = DsnDesign::parse(two_layer_crossing_dsn()).unwrap();
    let scores: Arc<Mutex<Vec<RouteScore>>> = Arc::new(Mutex::new(Vec::new()));
    let scores_cb = Arc::clone(&scores);
    let res = route_with_checkpoint(&design, &RouterConfig::default(), move |_b, _r, score| {
        scores_cb.lock().unwrap().push(score);
    })
    .unwrap();
    assert_eq!(res.stats.routed_nets, 2);
    let recorded = scores.lock().unwrap();
    assert!(!recorded.is_empty(), "checkpoint callback must fire on improvement");
    for w in recorded.windows(2) {
        assert!(w[1].is_better_than(w[0]), "checkpoint scores must improve monotonically");
    }

    let stop = Arc::new(AtomicBool::new(true));
    let stopped_res = route(
        &design,
        &RouterConfig {
            stop_flag: Some(stop),
            ..Default::default()
        },
    )
    .unwrap();
    assert!(stopped_res.stats.stopped_early);
    assert_eq!(stopped_res.stats.passes_run, 0);
}

#[test]
fn report_and_diagnose_distinguishes_blocked_vs_congestion() {
    let design = DsnDesign::parse(blocked_and_open_dsn()).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();

    // Leave both nets unrouted so /Net/BLOCKED is statically blocked while /Net/OPEN is routable alone.
    let empty_routes: Vec<NetRoute> = (0..board.nets.len())
        .map(|net_id| NetRoute {
            net_id,
            unconnected_pairs: 1,
            ..Default::default()
        })
        .collect();
    let stats = viahop::RouteStats {
        total_nets: 2,
        routable_nets: 2,
        routed_nets: 0,
        unconnected_pairs: 2,
        total_segments: 0,
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
    };

    let report = RouteReport::build(&board, &empty_routes, stats, true);
    assert_eq!(report.unrouted.len(), 2);
    let blocked = report.unrouted.iter().find(|u| u.net == "/Net/BLOCKED").unwrap();
    let open = report.unrouted.iter().find(|u| u.net == "/Net/OPEN").unwrap();
    assert_eq!(blocked.diagnosis, Some(UnroutedKind::Blocked));
    assert_eq!(open.diagnosis, Some(UnroutedKind::Congestion));

    let json = report.to_json();
    assert!(json.contains("\"diagnosis\":\"blocked\""));
    assert!(json.contains("\"diagnosis\":\"congestion\""));
}

#[test]
fn c_abi_smoke_test() {
    unsafe {
        let ver = CStr::from_ptr(viahop_version()).to_str().unwrap();
        assert!(!ver.is_empty());

        let dsn_c = CString::new(two_layer_crossing_dsn()).unwrap();
        let mut out_ses = ptr::null_mut();
        let mut out_json = ptr::null_mut();
        let mut out_err = ptr::null_mut();
        let rc = viahop_route_str(
            dsn_c.as_ptr(),
            10,
            ptr::null(),
            -1,
            1,
            30,
            &mut out_ses,
            &mut out_json,
            &mut out_err,
        );
        assert_eq!(rc, 0);
        assert!(out_err.is_null());
        let ses = CStr::from_ptr(out_ses).to_str().unwrap();
        let json = CStr::from_ptr(out_json).to_str().unwrap();
        assert!(ses.contains("(session crossing"));
        assert!(json.contains("\"routed_nets\":2"));
        assert!(json.contains("\"drc_violations\":0"));
        viahop_free_str(out_ses);
        viahop_free_str(out_json);

        let tmp_dir = std::env::temp_dir();
        let tmp_in = tmp_dir.join("viahop_ffi_in.dsn");
        let tmp_out = tmp_dir.join("viahop_ffi_out.ses");
        std::fs::write(&tmp_in, two_layer_crossing_dsn()).unwrap();
        let in_c = CString::new(tmp_in.to_str().unwrap()).unwrap();
        let out_c = CString::new(tmp_out.to_str().unwrap()).unwrap();
        let mut err_buf = [0 as std::os::raw::c_char; 256];
        let rc2 = viahop_route_file(
            in_c.as_ptr(),
            out_c.as_ptr(),
            10,
            ptr::null(),
            1,
            1,
            30,
            err_buf.as_mut_ptr(),
            err_buf.len(),
        );
        assert_eq!(rc2, 0);
        assert!(std::fs::read_to_string(&tmp_out).unwrap().contains("(session crossing"));
        let _ = std::fs::remove_file(&tmp_in);
        let _ = std::fs::remove_file(&tmp_out);
    }
}

#[test]
fn parses_malformed_dsn_with_stray_closing_paren() {
    let broken = two_layer_crossing_dsn().replace(
        "    (rule (width 200) (clearance 200))\n  )\n  (placement",
        "    (rule (width 200) (clearance 200))\n  ))\n  (placement",
    );
    let design = DsnDesign::parse(&broken).unwrap();
    assert_eq!(design.nets.len(), 2);
    let res = route(&design, &RouterConfig::default()).unwrap();
    assert_eq!(res.stats.routable_nets, 2);
    assert_eq!(res.stats.routed_nets, 2);
    assert_eq!(res.stats.drc_violations, 0);
}

#[test]
fn cli_flags_report_diagnose_and_initial_session() {
    let bin = env!("CARGO_BIN_EXE_viahop");
    let tmp_dir = std::env::temp_dir();
    let tmp_in = tmp_dir.join("viahop_cli_in.dsn");
    let tmp_partial = tmp_dir.join("viahop_cli_partial.ses");
    let tmp_out = tmp_dir.join("viahop_cli_out.ses");
    let tmp_diag_in = tmp_dir.join("viahop_cli_diag.dsn");
    let tmp_diag_out = tmp_dir.join("viahop_cli_diag.ses");
    let tmp_report = tmp_dir.join("viahop_cli_report.json");

    std::fs::write(&tmp_in, two_layer_crossing_dsn()).unwrap();

    // 1. Route only /Net/A into a checkpoint SES.
    let out1 = std::process::Command::new(bin)
        .args([
            "-de",
            tmp_in.to_str().unwrap(),
            "-do",
            tmp_partial.to_str().unwrap(),
            "-mp",
            "10",
            "-inc",
            "/Net/B",
            "-mt",
            "2",
            "--multi-start=2",
            "-dl",
            "--via-at-smd",
            "--max-time=30",
            "--json-summary",
        ])
        .output()
        .unwrap();
    assert!(out1.status.success(), "stderr: {}", String::from_utf8_lossy(&out1.stderr));
    let stdout1 = String::from_utf8_lossy(&out1.stdout);
    assert!(stdout1.contains("\"routable_nets\":1"));
    assert!(stdout1.contains("\"routed_nets\":1"));

    // 2. Resume with --initial-session and route the full board.
    let out2 = std::process::Command::new(bin)
        .args([
            "-i",
            tmp_in.to_str().unwrap(),
            "-o",
            tmp_out.to_str().unwrap(),
            "--initial-session",
            tmp_partial.to_str().unwrap(),
            "--json-summary",
        ])
        .output()
        .unwrap();
    assert!(out2.status.success(), "stderr: {}", String::from_utf8_lossy(&out2.stderr));
    assert!(String::from_utf8_lossy(&out2.stdout).contains("\"routed_nets\":2"));
    let ses = std::fs::read_to_string(&tmp_out).unwrap();
    assert!(ses.contains("(net /Net/A"));
    assert!(ses.contains("(net /Net/B"));

    // 3. Run --report + --diagnose on blocked board.
    std::fs::write(&tmp_diag_in, blocked_and_open_dsn()).unwrap();
    let out3 = std::process::Command::new(bin)
        .args([
            "-i",
            tmp_diag_in.to_str().unwrap(),
            "-o",
            tmp_diag_out.to_str().unwrap(),
            "-mp",
            "2",
            "--multi-start=2",
            "--report",
            tmp_report.to_str().unwrap(),
            "--diagnose",
            "-q",
        ])
        .output()
        .unwrap();
    assert!(out3.status.success(), "stderr: {}", String::from_utf8_lossy(&out3.stderr));
    let report_json = std::fs::read_to_string(&tmp_report).unwrap();
    assert!(report_json.contains("/Net/BLOCKED"));
    assert!(report_json.contains("\"diagnosis\":\"blocked\""));

    let ver_out = std::process::Command::new(bin).arg("-V").output().unwrap();
    assert!(ver_out.status.success());
    assert!(String::from_utf8_lossy(&ver_out.stdout).contains("viahop"));

    for p in [&tmp_in, &tmp_partial, &tmp_out, &tmp_diag_in, &tmp_diag_out, &tmp_report] {
        let _ = std::fs::remove_file(p);
    }
}
