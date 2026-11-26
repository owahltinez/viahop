use std::collections::HashSet;

use viahop::geometry::Point;
use viahop::{
    Board, DsnDesign, LengthConstraints, LengthViolationKind, NetRoute, RouteReport, RoutedSegment,
    RouterConfig, check_drc, evaluate_lengths, net_total_length_um, parse_match_group_spec,
    parse_max_length_spec, parse_ses, route, tune_match_groups, write_ses,
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

    for bad in [
        "NO_EQUALS",
        "=10mm",
        "A+=10mm",
        "+A=10mm",
        "A++B=10mm",
        "A=0mm",
        "A=-5mm",
        "A=10cm",
        "A=NaN",
        "A=inf",
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
        "G:A,B:0.5mm@0mm",
        "G:A,B:0.5mm@-10mm",
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
    assert_eq!(
        report.length_report.violations[0].kind,
        LengthViolationKind::MaxLength
    );
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
