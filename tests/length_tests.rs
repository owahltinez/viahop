use std::collections::HashSet;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;

use viahop::geometry::Point;
use viahop::length::{evaluate_lengths, net_total_length_um, tune_match_groups};
use viahop::optimize::optimize_routes;
use viahop::{
    Board, DsnDesign, LengthConstraints, NetRoute, RouteReport, RouteScore, RoutedSegment,
    RoutedVia, RouterConfig, check_drc, parse_match_group_spec, parse_max_length_spec, parse_ses,
    route, route_with_checkpoint, write_ses,
};

fn qspi_bus_dsn() -> &'static str {
    // 4-net bus with unequal spans: CLK=6mm, IO0=8mm, IO1=10mm, IO2=12mm, 3mm lane spacing.
    r#"(pcb qspi_bus.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 22000 18000))
    (via "Via[0-1]_400:200_um")
    (rule (width 150) (clearance 150))
  )
  (placement
    (component P1
      (place S0 2000 3000 front 0)
      (place D0 8000 3000 front 0)
      (place S1 2000 6500 front 0)
      (place D1 10000 6500 front 0)
      (place S2 2000 10000 front 0)
      (place D2 12000 10000 front 0)
      (place S3 2000 13500 front 0)
      (place D3 14000 13500 front 0)
    )
  )
  (library
    (image P1 (pin SMD_400 1 0 0))
    (padstack SMD_400 (shape (circle F.Cu 400)) (attach off))
    (padstack "Via[0-1]_400:200_um"
      (shape (circle F.Cu 400))
      (shape (circle B.Cu 400))
      (attach off)
    )
  )
  (network
    (net CLK (pins S0-1 D0-1))
    (net IO0 (pins S1-1 D1-1))
    (net IO1 (pins S2-1 D2-1))
    (net IO2 (pins S3-1 D3-1))
    (class default CLK IO0 IO1 IO2
      (circuit (use_via "Via[0-1]_400:200_um"))
      (rule (width 150) (clearance 150))
    )
  )
  (wiring)
)"#
}

fn diff_pair_dsn() -> &'static str {
    // 2-net differential pair with unequal natural lengths (~6mm and ~7.5mm) on an open board.
    r#"(pcb diff_pair.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 18000 14000))
    (via "Via[0-1]_400:200_um")
    (rule (width 150) (clearance 150))
  )
  (placement
    (component P1
      (place A1 2000 4500 front 0)
      (place A2 8000 4500 front 0)
      (place B1 2000 9500 front 0)
      (place B2 9500 9500 front 0)
    )
  )
  (library
    (image P1 (pin SMD_400 1 0 0))
    (padstack SMD_400 (shape (circle F.Cu 400)) (attach off))
    (padstack "Via[0-1]_400:200_um"
      (shape (circle F.Cu 400))
      (shape (circle B.Cu 400))
      (attach off)
    )
  )
  (network
    (net USB_DP (pins A1-1 A2-1))
    (net USB_DM (pins B1-1 B2-1))
    (class default USB_DP USB_DM
      (circuit (use_via "Via[0-1]_400:200_um"))
      (rule (width 150) (clearance 150))
    )
  )
  (wiring)
)"#
}

fn series_chain_dsn() -> &'static str {
    // Series resistor chain: U1 -> /PDM_CLK -> R1 -> /Audio/PDM_CLK_MIC -> U2, plus crossing net.
    r#"(pcb series_chain.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 18000 12000))
    (via "Via[0-1]_400:200_um")
    (rule (width 150) (clearance 150))
  )
  (placement
    (component P1
      (place U1 2000 6000 front 0)
      (place U2 15000 6000 front 0)
      (place X1 8500 2000 front 0)
      (place X2 8500 10000 front 0)
    )
    (component R0402
      (place R1 8000 6000 front 0)
    )
  )
  (library
    (image P1 (pin SMD_400 1 0 0))
    (image R0402
      (pin SMD_400 1 -600 0)
      (pin SMD_400 2 600 0)
    )
    (padstack SMD_400 (shape (circle F.Cu 400)) (attach off))
    (padstack "Via[0-1]_400:200_um"
      (shape (circle F.Cu 400))
      (shape (circle B.Cu 400))
      (attach off)
    )
  )
  (network
    (net /PDM_CLK (pins U1-1 R1-1))
    (net /Audio/PDM_CLK_MIC (pins R1-2 U2-1))
    (net /CROSS (pins X1-1 X2-1))
    (class default /PDM_CLK /Audio/PDM_CLK_MIC /CROSS
      (circuit (use_via "Via[0-1]_400:200_um"))
      (rule (width 150) (clearance 150))
    )
  )
  (wiring)
)"#
}

fn one_side_blocked_dsn() -> &'static str {
    // Keepout wall sits directly above SHORT (y=4250..5600) so meanders must go below (-y).
    r#"(pcb blocked_side.dsn
  (parser (string_quote ") (host_cad "KiCad's Pcbnew") (host_version "10.0.1"))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 18000 12000))
    (keepout "" (rect F.Cu 2800 4350 9200 5800))
    (via "Via[0-1]_400:200_um")
    (rule (width 150) (clearance 150))
  )
  (placement
    (component P1
      (place A1 2000 4000 front 0)
      (place A2 10000 4000 front 0)
      (place B1 2000 9000 front 0)
      (place B2 14000 9000 front 0)
    )
  )
  (library
    (image P1 (pin SMD_400 1 0 0))
    (padstack SMD_400 (shape (circle F.Cu 400)) (attach off))
    (padstack "Via[0-1]_400:200_um"
      (shape (circle F.Cu 400))
      (shape (circle B.Cu 400))
      (attach off)
    )
  )
  (network
    (net SHORT (pins A1-1 A2-1))
    (net LONG (pins B1-1 B2-1))
    (class default SHORT LONG
      (circuit (use_via "Via[0-1]_400:200_um"))
      (rule (width 150) (clearance 150))
    )
  )
  (wiring)
)"#
}

#[test]
fn parses_max_length_and_match_group_specs() {
    let ml_mm = parse_max_length_spec("CLK=22mm").unwrap();
    assert_eq!(ml_mm.nets, vec!["CLK"]);
    assert!((ml_mm.max_um - 22_000.0).abs() < 1e-6);

    let ml_default = parse_max_length_spec("CLK=12.5").unwrap();
    assert!((ml_default.max_um - 12_500.0).abs() < 1e-6);

    let ml_um = parse_max_length_spec("NET_A=8500um").unwrap();
    assert!((ml_um.max_um - 8500.0).abs() < 1e-6);

    let ml_mil = parse_max_length_spec("NET_B=100mil").unwrap();
    assert!((ml_mil.max_um - 2540.0).abs() < 1e-6);

    let ml_chain = parse_max_length_spec("/PDM_CLK + /Audio/PDM_CLK_MIC = 40MM").unwrap();
    assert_eq!(ml_chain.nets, vec!["/PDM_CLK", "/Audio/PDM_CLK_MIC"]);
    assert!((ml_chain.max_um - 40_000.0).abs() < 1e-6);

    let ml_comma = parse_max_length_spec("USB_D+,+3V3=35mm").unwrap();
    assert_eq!(ml_comma.nets, vec!["USB_D+", "+3V3"]);
    assert!((ml_comma.max_um - 35_000.0).abs() < 1e-6);

    let ml_trailing_plus = parse_max_length_spec("USB_D+=15mm").unwrap();
    assert_eq!(ml_trailing_plus.nets, vec!["USB_D+"]);

    let ml_leading_plus = parse_max_length_spec("+3V3=20mm").unwrap();
    assert_eq!(ml_leading_plus.nets, vec!["+3V3"]);

    for bad in [
        "NO_EQUALS",
        "=10mm",
        "+=10mm",
        "++=10mm",
        "A++B=10mm",
        "A,,B=10mm",
        ",A=10mm",
        "A,=10mm",
        "A+A=10mm",
        "A,A=10mm",
        "A=0mm",
        "A=-5mm",
        "A=10cm",
        "A=NaN",
        "A=inf",
        "A=1e306mm",
        "A=1e307mil",
        "A=",
    ] {
        assert!(
            parse_max_length_spec(bad).is_err(),
            "expected error for max-length spec: {bad}"
        );
    }

    let mg = parse_match_group_spec("QSPI:CLK,IO0,IO1:0.5mm").unwrap();
    assert_eq!(mg.name, "QSPI");
    assert_eq!(mg.nets, vec!["CLK", "IO0", "IO1"]);
    assert!((mg.tolerance_um - 500.0).abs() < 1e-6);
    assert_eq!(mg.target_um, None);

    let mg_tgt = parse_match_group_spec("DP:USB_DP,USB_DM:100um@15mm").unwrap();
    assert_eq!(mg_tgt.name, "DP");
    assert_eq!(mg_tgt.nets, vec!["USB_DP", "USB_DM"]);
    assert!((mg_tgt.tolerance_um - 100.0).abs() < 1e-6);
    assert!((mg_tgt.target_um.unwrap() - 15_000.0).abs() < 1e-6);

    let mg_mil = parse_match_group_spec("BUS:A,B:10mil@500mil").unwrap();
    assert!((mg_mil.tolerance_um - 254.0).abs() < 1e-6);
    assert!((mg_mil.target_um.unwrap() - 12_700.0).abs() < 1e-6);

    for bad in [
        "MISSING_COLONS",
        "ONLY:ONE",
        ":A,B:0.5mm",
        "G:ONLY_ONE:0.5mm",
        "G:A,,B:0.5mm",
        "G:A,A:0.5mm",
        "G:A,B:0mm",
        "G:A,B:-0.5mm",
        "G:A,B:1e306mm",
        "G:A,B:0.5mm@0mm",
        "G:A,B:0.5mm@-10mm",
        "G:A,B:0.5mm@1e306mm",
        "G:A,B:0.5mm@",
        "G:A,B:bad",
    ] {
        assert!(
            parse_match_group_spec(bad).is_err(),
            "expected error for match-group spec: {bad}"
        );
    }
}

#[test]
fn routes_max_length_single_net_series_chain_and_reports_impossible_budget() {
    let design = DsnDesign::parse(series_chain_dsn()).unwrap();

    // 1. Achievable single-net and 2-net '+' chain constraints (direct chain sum ~11.8mm).
    let mut constraints = LengthConstraints::default();
    constraints
        .max_lengths
        .push(parse_max_length_spec("/PDM_CLK=6.5mm").unwrap());
    constraints
        .max_lengths
        .push(parse_max_length_spec("/PDM_CLK+/Audio/PDM_CLK_MIC=13.5mm").unwrap());

    let cfg = RouterConfig {
        max_passes: 15,
        length_constraints: constraints.clone(),
        ..Default::default()
    };
    let res = route(&design, &cfg).unwrap();
    assert_eq!(res.stats.unconnected_pairs, 0);
    assert_eq!(res.stats.drc_violations, 0);
    assert_eq!(res.stats.length_violations, 0);
    assert_eq!(check_drc(&res.board, &res.routes).violations.len(), 0);

    let lr = evaluate_lengths(&res.board, &res.routes, &constraints);
    assert!(lr.violations.is_empty());
    assert!(lr.max_lengths.iter().all(|m| m.passed));
    assert!(lr.max_lengths[1].actual_um <= 13_500.0 + 0.5);

    // 2. Impossibly tight budget (~11.8mm minimum vs 5mm cap) must report a violation cleanly with 0 DRC.
    let mut tight = LengthConstraints::default();
    tight
        .max_lengths
        .push(parse_max_length_spec("/PDM_CLK+/Audio/PDM_CLK_MIC=5mm").unwrap());
    let tight_res = route(
        &design,
        &RouterConfig {
            max_passes: 8,
            multi_start: 1,
            length_constraints: tight.clone(),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(tight_res.stats.unconnected_pairs, 0);
    assert_eq!(tight_res.stats.drc_violations, 0);
    assert_eq!(tight_res.stats.length_violations, 1);

    let report = RouteReport::build_with_constraints(
        &tight_res.board,
        &tight_res.routes,
        tight_res.stats.clone(),
        false,
        &tight,
    );
    assert_eq!(report.length_report.violations.len(), 1);
    assert_eq!(report.length_report.violations[0].kind, "max_length");
    assert!(report.length_report.violations[0].actual_um > 10_000.0);
    assert!(!report.length_report.max_lengths[0].passed);
    let json = report.to_json();
    assert!(json.contains("\"length_violations\":1"));
    assert!(json.contains("\"kind\":\"max_length\""));

    // 3. Rip-up pass shortens a detoured initial session route below --max-length.
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    let pdm_id = board.net_name_to_id["/PDM_CLK"];
    let mut init_routes: Vec<NetRoute> = (0..board.nets.len())
        .map(|net_id| NetRoute {
            net_id,
            ..Default::default()
        })
        .collect();
    // Deliberate huge 14mm detour for /PDM_CLK (pads at x=2000..7400, y=6000).
    init_routes[pdm_id].segments = vec![
        RoutedSegment {
            layer: 0,
            a: Point::new(2000.0, 6000.0),
            b: Point::new(2000.0, 2000.0),
            width: 150.0,
        },
        RoutedSegment {
            layer: 0,
            a: Point::new(2000.0, 2000.0),
            b: Point::new(7400.0, 2000.0),
            width: 150.0,
        },
        RoutedSegment {
            layer: 0,
            a: Point::new(7400.0, 2000.0),
            b: Point::new(7400.0, 6000.0),
            width: 150.0,
        },
    ];
    let mut shorten_c = LengthConstraints::default();
    shorten_c
        .max_lengths
        .push(parse_max_length_spec("/PDM_CLK=6.5mm").unwrap());
    let shortened = route(
        &design,
        &RouterConfig {
            max_passes: 10,
            optimize: false,
            initial_routes: Some(init_routes),
            length_constraints: shorten_c,
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(shortened.stats.drc_violations, 0);
    assert_eq!(shortened.stats.length_violations, 0);
    let final_pdm = net_total_length_um(&shortened.board, &shortened.routes, pdm_id);
    assert!(
        final_pdm <= 6500.0 + 0.5,
        "expected rip-up to shorten /PDM_CLK below 6.5mm, got {final_pdm:.1}um"
    );
}

#[test]
fn fixed_segments_are_included_in_length_evaluation() {
    let dsn_with_fixed = r#"(pcb fixed_len.dsn
  (parser (string_quote "))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (boundary (rect pcb 0 0 12000 6000))
    (rule (width 150) (clearance 150))
  )
  (placement
    (component P1 (place J1 2000 3000 front 0) (place J2 8000 3000 front 0))
  )
  (library
    (image P1 (pin SMD_400 1 0 0))
    (padstack SMD_400 (shape (circle F.Cu 400)) (attach off))
  )
  (network (net N1 (pins J1-1 J2-1)))
  (wiring
    (wire (path F.Cu 150 2000 3000 5000 3000) (net N1) (type fix))
  )
)"#;
    let design = DsnDesign::parse(dsn_with_fixed).unwrap();
    let mut constraints = LengthConstraints::default();
    // Fixed segment is 3mm + routed segment is 3mm = 6mm total; 5mm cap must flag violation.
    constraints
        .max_lengths
        .push(parse_max_length_spec("N1=5mm").unwrap());
    let res = route(
        &design,
        &RouterConfig {
            max_passes: 5,
            multi_start: 1,
            length_constraints: constraints.clone(),
            ..Default::default()
        },
    )
    .unwrap();
    let total = net_total_length_um(&res.board, &res.routes, 0);
    assert!(
        total >= 5900.0,
        "total length must include 3mm fixed segment + 3mm routed, got {total:.1}um"
    );
    assert_eq!(res.stats.length_violations, 1);
    let lr = evaluate_lengths(&res.board, &res.routes, &constraints);
    assert!(!lr.max_lengths[0].passed);
}

#[test]
fn tunes_four_net_qspi_bus_and_diff_pair_with_zero_drc() {
    // 1. 4-net QSPI bus matched within 0.5mm tolerance.
    let qspi_design = DsnDesign::parse(qspi_bus_dsn()).unwrap();
    let mut qspi_c = LengthConstraints::default();
    qspi_c
        .match_groups
        .push(parse_match_group_spec("QSPI:CLK,IO0,IO1,IO2:0.5mm").unwrap());

    for opt in [true, false] {
        let res = route(
            &qspi_design,
            &RouterConfig {
                max_passes: 15,
                optimize: opt,
                length_constraints: qspi_c.clone(),
                ..Default::default()
            },
        )
        .unwrap();
        assert_eq!(res.stats.routed_nets, 4);
        assert_eq!(res.stats.unconnected_pairs, 0);
        assert_eq!(res.stats.drc_violations, 0);
        assert_eq!(res.stats.length_violations, 0);
        assert_eq!(
            res.stats.starts_run, 1,
            "cleanly tuned start 0 should skip extra starts"
        );
        let drc = check_drc(&res.board, &res.routes);
        assert!(drc.violations.is_empty(), "DRC: {:?}", drc.violations);

        let lr = evaluate_lengths(&res.board, &res.routes, &qspi_c);
        assert!(lr.violations.is_empty(), "length report: {lr:?}");
        let mg = &lr.match_groups[0];
        assert!(mg.passed);
        assert!(
            mg.skew_um <= 500.0 + 0.5,
            "skew {:.1}um exceeded 500um (opt={opt}): {:?}",
            mg.skew_um,
            mg.members
        );
        // CLK started at 6mm and had to meander to ~12mm.
        let clk_id = res.board.net_name_to_id["CLK"];
        assert!(
            res.routes[clk_id].segments.len() > 3,
            "expected meander segments on CLK"
        );
        assert!(net_total_length_um(&res.board, &res.routes, clk_id) >= 11_500.0);

        // Verify SES round-trip preserves matched lengths and 0 DRC.
        let ses_text = write_ses(&qspi_design, &res);
        let reparsed = parse_ses(&ses_text, &res.board).unwrap();
        assert_eq!(check_drc(&res.board, &reparsed).violations.len(), 0);
        let lr_rt = evaluate_lengths(&res.board, &reparsed, &qspi_c);
        assert!(lr_rt.match_groups[0].passed);
    }

    // 2. 2-net differential pair with explicit @target (both shorter than 12mm target).
    let dp_design = DsnDesign::parse(diff_pair_dsn()).unwrap();
    let mut dp_c = LengthConstraints::default();
    dp_c.match_groups
        .push(parse_match_group_spec("DP:USB_DP,USB_DM:0.2mm@12mm").unwrap());
    let dp_res = route(
        &dp_design,
        &RouterConfig {
            max_passes: 15,
            length_constraints: dp_c.clone(),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(dp_res.stats.unconnected_pairs, 0);
    assert_eq!(dp_res.stats.drc_violations, 0);
    assert_eq!(dp_res.stats.length_violations, 0);
    let dp_lr = evaluate_lengths(&dp_res.board, &dp_res.routes, &dp_c);
    assert!(dp_lr.match_groups[0].passed, "dp report: {dp_lr:?}");
    for m in &dp_lr.match_groups[0].members {
        assert!(
            (m.length_um - 12_000.0).abs() <= 200.5,
            "{} length {:.1}um not within 200um of 12000um",
            m.net,
            m.length_um
        );
    }
}

#[test]
fn meander_tuning_avoids_blocked_side_and_respects_max_length_cap() {
    let design = DsnDesign::parse(one_side_blocked_dsn()).unwrap();
    let mut constraints = LengthConstraints::default();
    constraints
        .match_groups
        .push(parse_match_group_spec("PAIR:SHORT,LONG:0.3mm").unwrap());

    let res = route(
        &design,
        &RouterConfig {
            max_passes: 15,
            length_constraints: constraints.clone(),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(res.stats.unconnected_pairs, 0);
    assert_eq!(res.stats.drc_violations, 0);
    assert_eq!(res.stats.length_violations, 0);
    assert_eq!(check_drc(&res.board, &res.routes).violations.len(), 0);

    let short_id = res.board.net_name_to_id["SHORT"];
    // Meanders on SHORT must go into the open y < 4000 region, never into the keepout above y=4350.
    let min_y = res.routes[short_id]
        .segments
        .iter()
        .map(|s| s.a.y.min(s.b.y))
        .fold(f64::INFINITY, f64::min);
    assert!(
        min_y < 3600.0,
        "expected meanders on open -y side below 4000, min_y={min_y:.1}"
    );
    let lr = evaluate_lengths(&res.board, &res.routes, &constraints);
    assert!(lr.match_groups[0].passed);

    // Verify tune_match_groups clamps below a member's --max-length budget.
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    let mut manual_routes = vec![
        NetRoute {
            net_id: 0,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(2000.0, 4000.0),
                b: Point::new(10000.0, 4000.0),
                width: 150.0,
            }],
            ..Default::default()
        },
        NetRoute {
            net_id: 1,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(2000.0, 9000.0),
                b: Point::new(14000.0, 9000.0),
                width: 150.0,
            }],
            ..Default::default()
        },
    ];
    let mut capped = LengthConstraints::default();
    capped
        .max_lengths
        .push(parse_max_length_spec("SHORT=9.5mm").unwrap());
    capped
        .match_groups
        .push(parse_match_group_spec("PAIR:SHORT,LONG:0.3mm").unwrap());
    tune_match_groups(&board, &mut manual_routes, &capped);
    let short_len = net_total_length_um(&board, &manual_routes, 0);
    assert!(
        short_len <= 9500.0 + 0.5,
        "tuning must not exceed SHORT's 9.5mm max-length cap, got {short_len:.1}um"
    );
    assert_eq!(check_drc(&board, &manual_routes).violations.len(), 0);
}

#[test]
fn meander_tuning_handles_diagonal_segments_and_neighbor_net_obstacles() {
    let dsn = r#"(pcb diag_neighbor.dsn
  (parser (string_quote "))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (boundary (rect pcb 0 0 18000 16000))
    (rule (width 150) (clearance 150))
  )
  (placement
    (component P1
      (place A1 2000 2000 front 0)
      (place A2 8000 8000 front 0)
      (place B1 2000 14000 front 0)
      (place B2 14000 14000 front 0)
      (place C1 1500 2500 front 0)
      (place C2 7500 8500 front 0)
    )
  )
  (library
    (image P1 (pin SMD_400 1 0 0))
    (padstack SMD_400 (shape (circle F.Cu 400)) (attach off))
  )
  (network
    (net DIAG (pins A1-1 A2-1))
    (net REF (pins B1-1 B2-1))
    (net NEIGHBOR (pins C1-1 C2-1))
    (class default DIAG REF NEIGHBOR (rule (width 150) (clearance 150)))
  )
  (wiring)
)"#;
    let design = DsnDesign::parse(dsn).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    // Exact 45-degree DIAG segment (~8.49mm) with NEIGHBOR running parallel 707um away on +n side.
    let mut routes = vec![
        NetRoute {
            net_id: 0,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(2000.0, 2000.0),
                b: Point::new(8000.0, 8000.0),
                width: 150.0,
            }],
            ..Default::default()
        },
        NetRoute {
            net_id: 1,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(2000.0, 14000.0),
                b: Point::new(14000.0, 14000.0),
                width: 150.0,
            }],
            ..Default::default()
        },
        NetRoute {
            net_id: 2,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(1500.0, 2500.0),
                b: Point::new(7500.0, 8500.0),
                width: 150.0,
            }],
            ..Default::default()
        },
    ];
    let mut constraints = LengthConstraints::default();
    constraints
        .match_groups
        .push(parse_match_group_spec("G:DIAG,REF:0.3mm").unwrap());

    tune_match_groups(&board, &mut routes, &constraints);
    assert_eq!(check_drc(&board, &routes).violations.len(), 0);
    let lr = evaluate_lengths(&board, &routes, &constraints);
    assert!(
        lr.match_groups[0].passed,
        "45-degree segment should tune cleanly away from neighbor: {lr:?}"
    );
    assert!(routes[0].segments.len() > 3);
}

#[test]
fn cli_handles_max_length_match_group_and_report() {
    let bin = env!("CARGO_BIN_EXE_viahop");
    let tmp_dir = std::env::temp_dir();
    let pid = std::process::id();
    let tmp_in = tmp_dir.join(format!("viahop_len_{pid}.dsn"));
    let tmp_out = tmp_dir.join(format!("viahop_len_{pid}.ses"));
    let tmp_report = tmp_dir.join(format!("viahop_len_{pid}.json"));

    std::fs::write(&tmp_in, qspi_bus_dsn()).unwrap();

    let out = std::process::Command::new(bin)
        .args([
            "-i",
            tmp_in.to_str().unwrap(),
            "-o",
            tmp_out.to_str().unwrap(),
            "--max-length",
            "IO2=14mm",
            "--max-length=CLK=14mm",
            "--match-group",
            "QSPI:CLK,IO0,IO1,IO2:0.5mm",
            "--report",
            tmp_report.to_str().unwrap(),
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
    assert!(stdout.contains("\"drc_violations\":0"), "stdout: {stdout}");
    assert!(
        stdout.contains("\"length_violations\":0"),
        "stdout: {stdout}"
    );

    let report = std::fs::read_to_string(&tmp_report).unwrap();
    assert!(report.contains("\"length_checks\""));
    assert!(report.contains("\"match_groups\""));
    assert!(report.contains("\"name\":\"QSPI\""));
    assert!(report.contains("\"passed\":true"));
    assert!(report.contains("\"length_violations\": [\n  ]"));

    // Invalid spec must exit with status 2.
    let bad = std::process::Command::new(bin)
        .args([
            "-i",
            tmp_in.to_str().unwrap(),
            "-o",
            tmp_out.to_str().unwrap(),
            "--match-group",
            "BAD:ONLY_ONE:0.5mm",
        ])
        .output()
        .unwrap();
    assert_eq!(bad.status.code(), Some(2));

    for p in [&tmp_in, &tmp_out, &tmp_report] {
        let _ = std::fs::remove_file(p);
    }
}

#[test]
fn bump_transition_band_has_no_gaps_after_ses_roundtrip_and_optimize() {
    let design = DsnDesign::parse(diff_pair_dsn()).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();

    // With width=150 and clr=150, c=150 and 2c=300; test both (2c, 2c+0.1] and just above 2c+0.5.
    for delta in [248.57_f64, 249.55_f64] {
        let mut routes = vec![
            NetRoute {
                net_id: 0,
                segments: vec![RoutedSegment {
                    layer: 0,
                    a: Point::new(2000.0, 4500.0),
                    b: Point::new(8000.0, 4500.0),
                    width: 150.0,
                }],
                ..Default::default()
            },
            NetRoute {
                net_id: 1,
                segments: vec![RoutedSegment {
                    layer: 0,
                    a: Point::new(2000.0, 9500.0),
                    b: Point::new(9500.0, 9500.0),
                    width: 150.0,
                }],
                ..Default::default()
            },
        ];
        let target_um = 6000.0 + delta;
        let mut constraints = LengthConstraints::default();
        constraints
            .match_groups
            .push(parse_match_group_spec(&format!("DP:USB_DP,USB_DM:1um@{target_um}um")).unwrap());

        tune_match_groups(&board, &mut routes, &constraints);
        assert!(routes[0].segments.len() >= 4, "expected meander on USB_DP");
        for seg in &routes[0].segments {
            assert!(
                seg.a.dist(seg.b) >= 0.5,
                "degenerate segment {:.4}um for delta={delta}",
                seg.a.dist(seg.b)
            );
        }
        assert_eq!(check_drc(&board, &routes).violations.len(), 0);

        let ses = viahop::ses::write_ses(&design, &board, &routes);
        let mut reparsed = parse_ses(&ses, &board).unwrap();
        let pre_report = RouteReport::build_with_constraints(
            &board,
            &reparsed,
            viahop::RouteStats {
                total_nets: 2,
                routable_nets: 2,
                routed_nets: 2,
                unconnected_pairs: 0,
                total_segments: reparsed[0].segments.len() + reparsed[1].segments.len(),
                total_vias: 0,
                total_wirelength_um: 0.0,
                passes_run: 0,
                starts_run: 1,
                drc_violations: 0,
                length_violations: 0,
                dense_bga_pads: 0,
                pofv_capable_pads: 0,
                grid_step_um: 50.0,
                stopped_early: false,
                elapsed_ms: 0,
            },
            false,
            &LengthConstraints::default(),
        );
        assert!(
            pre_report.unrouted.is_empty(),
            "SES roundtrip opened gap for delta={delta}: {:?}",
            pre_report.unrouted
        );

        optimize_routes(&board, &mut reparsed);
        let post_report = RouteReport::build(&board, &reparsed, pre_report.stats, false);
        assert!(
            post_report.unrouted.is_empty(),
            "optimize_routes disconnected trace for delta={delta}: {:?}",
            post_report.unrouted
        );
        assert_eq!(check_drc(&board, &reparsed).violations.len(), 0);
    }
}

#[test]
fn tapped_segments_preserve_connectivity_for_offcenter_pad_via_and_x_crossing() {
    // 1. Off-center pad + off-center via tapping the interior of a routed segment.
    let dsn_pad_via = r#"(pcb tapped_pad_via.dsn
  (parser (string_quote "))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary (rect pcb 0 0 22000 14000))
    (via "Via[0-1]_400:200_um")
    (rule (width 150) (clearance 150))
  )
  (placement
    (component P1
      (place A1 2000 4000 front 0)
      (place A2 10000 4000 front 0)
      (place A3 5500 4220 front 0)
      (place A4 7500 4180 back 0)
      (place B1 2000 10000 front 0)
      (place B2 16000 10000 front 0)
    )
  )
  (library
    (image P1 (pin SMD_400 1 0 0))
    (padstack SMD_400 (shape (circle F.Cu 400)) (attach off))
    (padstack "Via[0-1]_400:200_um"
      (shape (circle F.Cu 400))
      (shape (circle B.Cu 400))
      (attach off)
    )
  )
  (network
    (net TAPPED (pins A1-1 A2-1 A3-1 A4-1))
    (net REF (pins B1-1 B2-1))
    (class default TAPPED REF
      (circuit (use_via "Via[0-1]_400:200_um"))
      (rule (width 150) (clearance 150))
    )
  )
  (wiring)
)"#;
    let design = DsnDesign::parse(dsn_pad_via).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    // Single segment A1->A2 is grazed off-center by pad A3 (y=4220) and via at (7500, 4180).
    let mut routes = vec![
        NetRoute {
            net_id: 0,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(2000.0, 4000.0),
                b: Point::new(10000.0, 4000.0),
                width: 150.0,
            }],
            vias: vec![RoutedVia {
                pos: Point::new(7500.0, 4180.0),
                padstack: "Via[0-1]_400:200_um".to_string(),
                radius: 200.0,
                drill: 100.0,
                layers: vec![0, 1],
            }],
            ..Default::default()
        },
        NetRoute {
            net_id: 1,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(2000.0, 10000.0),
                b: Point::new(16000.0, 10000.0),
                width: 150.0,
            }],
            ..Default::default()
        },
    ];
    let mut constraints = LengthConstraints::default();
    constraints
        .match_groups
        .push(parse_match_group_spec("G:TAPPED,REF:0.5mm").unwrap());

    tune_match_groups(&board, &mut routes, &constraints);
    // Tapped segment must not be broken by meandering.
    assert_eq!(routes[0].segments.len(), 1);
    let rep = RouteReport::build(&board, &routes, dummy_stats(), false);
    assert!(
        rep.unrouted.is_empty(),
        "off-center pad/via tap lost connectivity: {:?}",
        rep.unrouted
    );
    assert_eq!(check_drc(&board, &routes).violations.len(), 0);

    // 2. X-crossing between two same-net segments must not be meandered apart.
    let dsn_x = r#"(pcb x_cross.dsn
  (parser (string_quote "))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (boundary (rect pcb 0 0 22000 16000))
    (rule (width 150) (clearance 150))
  )
  (placement
    (component P1
      (place P1 2000 6000 front 0)
      (place P2 10000 6000 front 0)
      (place P3 6000 2000 front 0)
      (place P4 6000 10000 front 0)
      (place R1 2000 13500 front 0)
      (place R2 20000 13500 front 0)
    )
  )
  (library
    (image P1 (pin SMD_400 1 0 0))
    (padstack SMD_400 (shape (circle F.Cu 400)) (attach off))
  )
  (network
    (net CROSS (pins P1-1 P2-1 P3-1 P4-1))
    (net LONG (pins R1-1 R2-1))
    (class default CROSS LONG (rule (width 150) (clearance 150)))
  )
  (wiring)
)"#;
    let x_design = DsnDesign::parse(dsn_x).unwrap();
    let x_board = Board::from_dsn(&x_design, &HashSet::new(), None).unwrap();
    let mut x_routes = vec![
        NetRoute {
            net_id: 0,
            segments: vec![
                RoutedSegment {
                    layer: 0,
                    a: Point::new(2000.0, 6000.0),
                    b: Point::new(10000.0, 6000.0),
                    width: 150.0,
                },
                RoutedSegment {
                    layer: 0,
                    a: Point::new(6000.0, 2000.0),
                    b: Point::new(6000.0, 10000.0),
                    width: 150.0,
                },
            ],
            ..Default::default()
        },
        NetRoute {
            net_id: 1,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(2000.0, 13500.0),
                b: Point::new(20000.0, 13500.0),
                width: 150.0,
            }],
            ..Default::default()
        },
    ];
    let mut x_c = LengthConstraints::default();
    x_c.match_groups
        .push(parse_match_group_spec("X:CROSS,LONG:0.5mm").unwrap());
    tune_match_groups(&x_board, &mut x_routes, &x_c);
    assert_eq!(x_routes[0].segments.len(), 2);
    let x_rep = RouteReport::build(&x_board, &x_routes, dummy_stats(), false);
    assert!(
        x_rep.unrouted.is_empty(),
        "X-crossing lost connectivity: {:?}",
        x_rep.unrouted
    );
}

#[test]
fn validates_unknown_ignored_and_rejects_unrouted_length_false_passes() {
    let design = DsnDesign::parse(qspi_bus_dsn()).unwrap();

    // Unknown net in --max-length and --match-group fails cleanly in route and route_with_checkpoint.
    let mut unk_ml = LengthConstraints::default();
    unk_ml
        .max_lengths
        .push(parse_max_length_spec("NO_SUCH_NET=10mm").unwrap());
    let err = route(
        &design,
        &RouterConfig {
            length_constraints: unk_ml.clone(),
            ..Default::default()
        },
    )
    .unwrap_err();
    assert!(err.contains("NO_SUCH_NET"), "unexpected err: {err}");
    assert!(
        route_with_checkpoint(
            &design,
            &RouterConfig {
                length_constraints: unk_ml,
                ..Default::default()
            },
            |_, _, _| {}
        )
        .is_err()
    );

    let mut unk_mg = LengthConstraints::default();
    unk_mg
        .match_groups
        .push(parse_match_group_spec("G:CLK,MISSING:0.5mm").unwrap());
    let err = route(
        &design,
        &RouterConfig {
            length_constraints: unk_mg,
            ..Default::default()
        },
    )
    .unwrap_err();
    assert!(err.contains("MISSING"), "unexpected err: {err}");

    // Ignored net in --max-length or --match-group fails validation cleanly.
    let mut ignored = HashSet::new();
    ignored.insert("CLK".to_string());
    let mut ign_ml = LengthConstraints::default();
    ign_ml
        .max_lengths
        .push(parse_max_length_spec("CLK=10mm").unwrap());
    let err = route(
        &design,
        &RouterConfig {
            ignored_nets: ignored.clone(),
            length_constraints: ign_ml,
            ..Default::default()
        },
    )
    .unwrap_err();
    assert!(err.contains("ignored"), "unexpected err: {err}");

    let mut ign_mg = LengthConstraints::default();
    ign_mg
        .match_groups
        .push(parse_match_group_spec("G:CLK,IO0:0.5mm").unwrap());
    let err = route(
        &design,
        &RouterConfig {
            ignored_nets: ignored,
            length_constraints: ign_mg,
            ..Default::default()
        },
    )
    .unwrap_err();
    assert!(err.contains("ignored"), "unexpected err: {err}");

    // Unrouted nets must never silently report 0.0um passed: true.
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    let empty_routes: Vec<NetRoute> = (0..board.nets.len())
        .map(|net_id| NetRoute {
            net_id,
            ..Default::default()
        })
        .collect();
    let mut both = LengthConstraints::default();
    both.max_lengths
        .push(parse_max_length_spec("CLK=20mm").unwrap());
    both.match_groups
        .push(parse_match_group_spec("G:CLK,IO0:0.5mm").unwrap());
    let lr = evaluate_lengths(&board, &empty_routes, &both);
    assert!(!lr.max_lengths[0].passed);
    assert!(!lr.match_groups[0].passed);
    assert_eq!(lr.violations.len(), 2);

    // Partially routed net with unconnected_pairs > 0 must also fail.
    let mut partial = empty_routes;
    partial[0].segments.push(RoutedSegment {
        layer: 0,
        a: Point::new(2000.0, 3000.0),
        b: Point::new(4000.0, 3000.0),
        width: 150.0,
    });
    partial[0].unconnected_pairs = 1;
    partial[1].segments.push(RoutedSegment {
        layer: 0,
        a: Point::new(2000.0, 6500.0),
        b: Point::new(10000.0, 6500.0),
        width: 150.0,
    });
    let lr_part = evaluate_lengths(&board, &partial, &both);
    assert!(!lr_part.max_lengths[0].passed);
    assert!(!lr_part.match_groups[0].passed);
    assert_eq!(lr_part.violations.len(), 2);

    // CLI exits non-zero when --max-length references an unknown net.
    let bin = env!("CARGO_BIN_EXE_viahop");
    let tmp_dir = std::env::temp_dir();
    let pid = std::process::id();
    let tmp_in = tmp_dir.join(format!("viahop_unk_{pid}.dsn"));
    let tmp_out = tmp_dir.join(format!("viahop_unk_{pid}.ses"));
    std::fs::write(&tmp_in, qspi_bus_dsn()).unwrap();
    let out = std::process::Command::new(bin)
        .args([
            "-i",
            tmp_in.to_str().unwrap(),
            "-o",
            tmp_out.to_str().unwrap(),
            "--max-length",
            "GHOST=10mm",
        ])
        .output()
        .unwrap();
    assert!(!out.status.success());
    assert!(
        String::from_utf8_lossy(&out.stderr).contains("GHOST"),
        "stderr: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    let _ = std::fs::remove_file(&tmp_in);
    let _ = std::fs::remove_file(&tmp_out);
}

#[test]
fn resolves_plus_net_names_in_max_length_constraints() {
    let dsn = r#"(pcb plus_nets.dsn
  (parser (string_quote "))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (boundary (rect pcb 0 0 18000 14000))
    (rule (width 150) (clearance 150))
  )
  (placement
    (component P1
      (place A1 2000 3000 front 0)
      (place A2 8000 3000 front 0)
      (place B1 2000 6500 front 0)
      (place B2 8000 6500 front 0)
      (place C1 2000 10000 front 0)
      (place C2 8000 10000 front 0)
    )
  )
  (library
    (image P1 (pin SMD_400 1 0 0))
    (padstack SMD_400 (shape (circle F.Cu 400)) (attach off))
  )
  (network
    (net USB_D+ (pins A1-1 A2-1))
    (net +3V3 (pins B1-1 B2-1))
    (net /USB/D+_FILT (pins C1-1 C2-1))
    (class default USB_D+ +3V3 /USB/D+_FILT (rule (width 150) (clearance 150)))
  )
  (wiring)
)"#;
    let design = DsnDesign::parse(dsn).unwrap();
    let mut constraints = LengthConstraints::default();
    constraints
        .max_lengths
        .push(parse_max_length_spec("USB_D+=8mm").unwrap());
    constraints
        .max_lengths
        .push(parse_max_length_spec("+3V3=8mm").unwrap());
    constraints
        .max_lengths
        .push(parse_max_length_spec("/USB/D+_FILT=8mm").unwrap());
    constraints
        .max_lengths
        .push(parse_max_length_spec("USB_D+,/USB/D+_FILT=15mm").unwrap());

    let res = route(
        &design,
        &RouterConfig {
            max_passes: 10,
            length_constraints: constraints.clone(),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(res.stats.unconnected_pairs, 0);
    assert_eq!(res.stats.drc_violations, 0);
    assert_eq!(res.stats.length_violations, 0);

    let lr = evaluate_lengths(&res.board, &res.routes, &constraints);
    assert!(lr.violations.is_empty(), "{lr:?}");
    assert_eq!(lr.max_lengths[2].nets, vec!["/USB/D+_FILT"]);
    assert_eq!(lr.max_lengths[3].nets, vec!["USB_D+", "/USB/D+_FILT"]);
    assert!(lr.max_lengths.iter().all(|m| m.passed));
}

#[test]
fn overlapping_match_groups_converge_across_shared_net() {
    let design = DsnDesign::parse(qspi_bus_dsn()).unwrap();
    // List shorter group first: G1(CLK=6mm, IO0=8mm) before G2(CLK=6mm, IO2=12mm).
    let mut constraints = LengthConstraints::default();
    constraints
        .match_groups
        .push(parse_match_group_spec("G1:CLK,IO0:0.5mm").unwrap());
    constraints
        .match_groups
        .push(parse_match_group_spec("G2:CLK,IO2:0.5mm").unwrap());

    let res = route(
        &design,
        &RouterConfig {
            max_passes: 15,
            length_constraints: constraints.clone(),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(res.stats.unconnected_pairs, 0);
    assert_eq!(res.stats.drc_violations, 0);
    assert_eq!(res.stats.length_violations, 0);
    let lr = evaluate_lengths(&res.board, &res.routes, &constraints);
    assert!(lr.violations.is_empty(), "{lr:?}");
    assert!(lr.match_groups.iter().all(|g| g.passed));
}

#[test]
fn max_length_displaces_competing_unconstrained_net() {
    // Single-layer crossing: BLOCKER has a shorter span (6mm) than FAST (10mm) and occupies the center.
    let dsn = r#"(pcb compete_max_len.dsn
  (parser (string_quote "))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (boundary (rect pcb 0 0 18000 14000))
    (rule (width 150) (clearance 150))
  )
  (placement
    (component P1
      (place F1 2000 7000 front 0)
      (place F2 12000 7000 front 0)
      (place B1 7000 4000 front 0)
      (place B2 7000 10000 front 0)
    )
  )
  (library
    (image P1 (pin SMD_400 1 0 0))
    (padstack SMD_400 (shape (circle F.Cu 400)) (attach off))
  )
  (network
    (net BLOCKER (pins B1-1 B2-1))
    (net FAST (pins F1-1 F2-1))
    (class default BLOCKER FAST (rule (width 150) (clearance 150)))
  )
  (wiring)
)"#;
    let design = DsnDesign::parse(dsn).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    let blocker_id = board.net_name_to_id["BLOCKER"];
    let fast_id = board.net_name_to_id["FAST"];

    // Start from an initial session where BLOCKER cuts straight across and FAST detours 16mm around it.
    let mut init = vec![
        NetRoute {
            net_id: 0,
            ..Default::default()
        },
        NetRoute {
            net_id: 1,
            ..Default::default()
        },
    ];
    init[blocker_id].segments = vec![RoutedSegment {
        layer: 0,
        a: Point::new(7000.0, 4000.0),
        b: Point::new(7000.0, 10000.0),
        width: 150.0,
    }];
    init[fast_id].segments = vec![
        RoutedSegment {
            layer: 0,
            a: Point::new(2000.0, 7000.0),
            b: Point::new(2000.0, 12000.0),
            width: 150.0,
        },
        RoutedSegment {
            layer: 0,
            a: Point::new(2000.0, 12000.0),
            b: Point::new(12000.0, 12000.0),
            width: 150.0,
        },
        RoutedSegment {
            layer: 0,
            a: Point::new(12000.0, 12000.0),
            b: Point::new(12000.0, 7000.0),
            width: 150.0,
        },
    ];

    let mut constraints = LengthConstraints::default();
    constraints
        .max_lengths
        .push(parse_max_length_spec("FAST=11mm").unwrap());

    let res = route(
        &design,
        &RouterConfig {
            max_passes: 15,
            initial_routes: Some(init),
            length_constraints: constraints.clone(),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(res.stats.unconnected_pairs, 0);
    assert_eq!(res.stats.drc_violations, 0);
    assert_eq!(res.stats.length_violations, 0);
    let fast_len = net_total_length_um(&res.board, &res.routes, fast_id);
    assert!(
        fast_len <= 11_000.0 + 0.5,
        "expected FAST <= 11mm after displacing BLOCKER, got {fast_len:.1}um"
    );
}

#[test]
fn initial_session_preserves_pre_matched_routes_across_optimize() {
    // Narrow 700um corridor between keepouts: pull-tight can collapse SHORT to 6mm, but trombone tuning cannot re-add 6mm.
    let dsn = r#"(pcb keep_pre_matched.dsn
  (parser (string_quote "))
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (boundary (rect pcb 0 0 18000 14000))
    (keepout "" (rect F.Cu 3200 1200 6800 3600))
    (keepout "" (rect F.Cu 3200 4400 6800 7200))
    (rule (width 150) (clearance 150))
  )
  (placement
    (component P1
      (place A1 2000 4000 front 0)
      (place A2 8000 4000 front 0)
      (place B1 2000 11000 front 0)
      (place B2 14000 11000 front 0)
    )
  )
  (library
    (image P1 (pin SMD_400 1 0 0))
    (padstack SMD_400 (shape (circle F.Cu 400)) (attach off))
  )
  (network
    (net SHORT (pins A1-1 A2-1))
    (net LONG (pins B1-1 B2-1))
    (class default SHORT LONG (rule (width 150) (clearance 150)))
  )
  (wiring)
)"#;
    let design = DsnDesign::parse(dsn).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();

    // Pre-matched initial session: SHORT takes a clean 12mm outer detour around the top keepout.
    let init = vec![
        NetRoute {
            net_id: 0,
            segments: vec![
                RoutedSegment {
                    layer: 0,
                    a: Point::new(2000.0, 4000.0),
                    b: Point::new(2000.0, 7700.0),
                    width: 150.0,
                },
                RoutedSegment {
                    layer: 0,
                    a: Point::new(2000.0, 7700.0),
                    b: Point::new(8000.0, 7700.0),
                    width: 150.0,
                },
                RoutedSegment {
                    layer: 0,
                    a: Point::new(8000.0, 7700.0),
                    b: Point::new(8000.0, 4000.0),
                    width: 150.0,
                },
            ],
            ..Default::default()
        },
        NetRoute {
            net_id: 1,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(2000.0, 11000.0),
                b: Point::new(14000.0, 11000.0),
                width: 150.0,
            }],
            ..Default::default()
        },
    ];
    let mut constraints = LengthConstraints::default();
    constraints
        .match_groups
        .push(parse_match_group_spec("PAIR:SHORT,LONG:1.5mm").unwrap());

    assert_eq!(check_drc(&board, &init).violations.len(), 0);
    assert!(
        evaluate_lengths(&board, &init, &constraints)
            .violations
            .is_empty()
    );

    let res = route(
        &design,
        &RouterConfig {
            max_passes: 5,
            optimize: true,
            initial_routes: Some(init),
            length_constraints: constraints.clone(),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(res.stats.unconnected_pairs, 0);
    assert_eq!(res.stats.drc_violations, 0);
    assert_eq!(res.stats.length_violations, 0);
    let lr = evaluate_lengths(&res.board, &res.routes, &constraints);
    assert!(lr.match_groups[0].passed, "{lr:?}");
}

#[test]
fn small_tolerance_and_at_target_skew_edge_case() {
    let design = DsnDesign::parse(diff_pair_dsn()).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();

    // 1. Sub-5um tolerance: 4um initial skew with 2um tolerance must still trigger tuning and pass.
    let mut small_routes = vec![
        NetRoute {
            net_id: 0,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(2000.0, 4500.0),
                b: Point::new(8000.0, 4500.0),
                width: 150.0,
            }],
            ..Default::default()
        },
        NetRoute {
            net_id: 1,
            segments: vec![
                RoutedSegment {
                    layer: 0,
                    a: Point::new(2000.0, 9500.0),
                    b: Point::new(5000.0, 9500.0),
                    width: 150.0,
                },
                RoutedSegment {
                    layer: 0,
                    a: Point::new(5000.0, 9500.0),
                    b: Point::new(8004.0, 9500.0),
                    width: 150.0,
                },
            ],
            ..Default::default()
        },
    ];
    let mut small_c = LengthConstraints::default();
    small_c
        .match_groups
        .push(parse_match_group_spec("FINE:USB_DP,USB_DM:2um").unwrap());
    tune_match_groups(&board, &mut small_routes, &small_c);
    let lr_small = evaluate_lengths(&board, &small_routes, &small_c);
    assert!(
        lr_small.match_groups[0].passed,
        "expected <5um tolerance tuning to pass: {lr_small:?}"
    );
    assert!(lr_small.match_groups[0].skew_um <= 2.5);
    assert_eq!(check_drc(&board, &small_routes).violations.len(), 0);

    // 2. @target skew edge case: USB_DM sits at 10450um in (10000, 10500]; USB_DP must tune past 10000um to keep skew <= 500um.
    let mut tgt_routes = vec![
        NetRoute {
            net_id: 0,
            segments: vec![RoutedSegment {
                layer: 0,
                a: Point::new(2000.0, 4500.0),
                b: Point::new(8000.0, 4500.0),
                width: 150.0,
            }],
            ..Default::default()
        },
        NetRoute {
            net_id: 1,
            segments: vec![
                RoutedSegment {
                    layer: 0,
                    a: Point::new(2000.0, 9500.0),
                    b: Point::new(2000.0, 11000.0),
                    width: 150.0,
                },
                RoutedSegment {
                    layer: 0,
                    a: Point::new(2000.0, 11000.0),
                    b: Point::new(9450.0, 11000.0),
                    width: 150.0,
                },
                RoutedSegment {
                    layer: 0,
                    a: Point::new(9450.0, 11000.0),
                    b: Point::new(9500.0, 9500.0),
                    width: 150.0,
                },
            ],
            ..Default::default()
        },
    ];
    let dm_len = net_total_length_um(&board, &tgt_routes, 1);
    assert!((10_300.0..=10_500.0).contains(&dm_len), "dm_len={dm_len}");
    let mut tgt_c = LengthConstraints::default();
    tgt_c
        .match_groups
        .push(parse_match_group_spec("DP:USB_DP,USB_DM:0.5mm@10mm").unwrap());
    tune_match_groups(&board, &mut tgt_routes, &tgt_c);
    let lr_tgt = evaluate_lengths(&board, &tgt_routes, &tgt_c);
    assert!(
        lr_tgt.match_groups[0].passed,
        "expected @target skew cap to pass: {lr_tgt:?}"
    );
    assert!(lr_tgt.match_groups[0].skew_um <= 500.5);
    assert_eq!(check_drc(&board, &tgt_routes).violations.len(), 0);
}

#[test]
fn route_score_prefers_lower_length_excess_and_respects_stop_flag() {
    let untuned = RouteScore {
        unconnected_pairs: 0,
        drc_violations: 0,
        length_violations: 1,
        length_excess_um: 5500.0,
        total_vias: 0,
        wirelength_um: 18_000.0,
    };
    let partially_tuned = RouteScore {
        unconnected_pairs: 0,
        drc_violations: 0,
        length_violations: 1,
        length_excess_um: 100.0,
        total_vias: 2,
        wirelength_um: 23_400.0,
    };
    assert!(partially_tuned.is_better_than(untuned));
    assert!(!untuned.is_better_than(partially_tuned));

    // Pre-set stop_flag must skip meander tuning cleanly.
    let design = DsnDesign::parse(qspi_bus_dsn()).unwrap();
    let board = Board::from_dsn(&design, &HashSet::new(), None).unwrap();
    let init: Vec<NetRoute> = [
        (0, 2000.0, 3000.0, 8000.0, 3000.0),
        (1, 2000.0, 6500.0, 10000.0, 6500.0),
        (2, 2000.0, 10000.0, 12000.0, 10000.0),
        (3, 2000.0, 13500.0, 14000.0, 13500.0),
    ]
    .into_iter()
    .map(|(net_id, x1, y1, x2, y2)| NetRoute {
        net_id,
        segments: vec![RoutedSegment {
            layer: 0,
            a: Point::new(x1, y1),
            b: Point::new(x2, y2),
            width: 150.0,
        }],
        ..Default::default()
    })
    .collect();
    let mut constraints = LengthConstraints::default();
    constraints
        .match_groups
        .push(parse_match_group_spec("QSPI:CLK,IO0,IO1,IO2:0.5mm").unwrap());
    let stop = Arc::new(AtomicBool::new(true));
    let res = route(
        &design,
        &RouterConfig {
            max_passes: 5,
            initial_routes: Some(init),
            length_constraints: constraints,
            stop_flag: Some(stop),
            ..Default::default()
        },
    )
    .unwrap();
    assert!(res.stats.stopped_early);
    assert_eq!(res.routes[board.net_name_to_id["CLK"]].segments.len(), 1);
}

fn dummy_stats() -> viahop::RouteStats {
    viahop::RouteStats {
        total_nets: 2,
        routable_nets: 2,
        routed_nets: 2,
        unconnected_pairs: 0,
        total_segments: 0,
        total_vias: 0,
        total_wirelength_um: 0.0,
        passes_run: 0,
        starts_run: 1,
        drc_violations: 0,
        length_violations: 0,
        dense_bga_pads: 0,
        pofv_capable_pads: 0,
        grid_step_um: 50.0,
        stopped_early: false,
        elapsed_ms: 0,
    }
}
