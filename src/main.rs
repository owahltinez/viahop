//! Headless CLI compatible with Freerouting command-line flags plus multi-start,
//! checkpointing, initial-session resume, and diagnostic JSON reporting.

use std::collections::HashSet;
use std::env;
use std::fs;
use std::path::Path;
use std::process::ExitCode;
use std::sync::Mutex;

use viahop::{
    parse_ses, route_board_with_checkpoint, ses, signal, Board, DsnDesign, RouteReport,
    RouteScore, RouterConfig,
};

struct CliArgs {
    input_dsn: String,
    output_ses: String,
    rules_file: Option<String>,
    initial_session: Option<String>,
    report_file: Option<String>,
    diagnose: bool,
    max_passes: usize,
    multi_start: usize,
    ignored_nets: HashSet<String>,
    threads: usize,
    via_at_smd: Option<bool>,
    timeout_secs: Option<u64>,
    disable_log: bool,
    json_summary: bool,
}

fn main() -> ExitCode {
    let raw: Vec<String> = env::args().skip(1).collect();
    match parse_args(&raw) {
        Ok(Action::Help) => {
            print_help();
            ExitCode::SUCCESS
        }
        Ok(Action::Version) => {
            println!("viahop {}", env!("CARGO_PKG_VERSION"));
            ExitCode::SUCCESS
        }
        Ok(Action::Run(cli)) => match run(*cli) {
            Ok(()) => ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("error: {e}");
                ExitCode::FAILURE
            }
        },
        Err(e) => {
            eprintln!("error: {e}\nRun with -h/--help for usage.");
            ExitCode::from(2)
        }
    }
}

enum Action {
    Help,
    Version,
    Run(Box<CliArgs>),
}

fn parse_args(args: &[String]) -> Result<Action, String> {
    if args.is_empty() {
        return Ok(Action::Help);
    }
    let mut input_dsn: Option<String> = None;
    let mut output_ses: Option<String> = None;
    let mut rules_file: Option<String> = None;
    let mut initial_session: Option<String> = None;
    let mut report_file: Option<String> = None;
    let mut diagnose = false;
    let mut max_passes: usize = 20;
    let mut multi_start: usize = 4;
    let mut ignored_nets = HashSet::new();
    let mut threads: usize = 1;
    let mut via_at_smd: Option<bool> = None;
    let mut timeout_secs: Option<u64> = None;
    let mut disable_log = false;
    let mut json_summary = false;

    let mut i = 0;
    while i < args.len() {
        let raw_arg = args[i].as_str();
        let (flag, inline_val) = match raw_arg.split_once('=') {
            Some((k, v)) if raw_arg.starts_with("--") => (k, Some(v)),
            _ => (raw_arg, None),
        };
        let next_val = |idx: &mut usize, f: &str, inline: Option<&str>| -> Result<String, String> {
            if let Some(v) = inline {
                return Ok(v.to_string());
            }
            *idx += 1;
            args.get(*idx)
                .cloned()
                .ok_or_else(|| format!("missing value for {f}"))
        };
        match flag {
            "-h" | "--help" => return Ok(Action::Help),
            "-V" | "--version" => return Ok(Action::Version),
            "-de" | "--input" => input_dsn = Some(next_val(&mut i, flag, inline_val)?),
            "-do" | "--output" => output_ses = Some(next_val(&mut i, flag, inline_val)?),
            "-dr" | "--rules" => rules_file = Some(next_val(&mut i, flag, inline_val)?),
            "--initial-session" => initial_session = Some(next_val(&mut i, flag, inline_val)?),
            "--report" => report_file = Some(next_val(&mut i, flag, inline_val)?),
            "--diagnose" => diagnose = true,
            "-mp" | "--max-passes" => {
                let v = next_val(&mut i, flag, inline_val)?;
                max_passes = v
                    .parse::<usize>()
                    .map_err(|_| format!("invalid integer for {flag}: {v}"))?;
            }
            "--multi-start" => {
                let v = next_val(&mut i, flag, inline_val)?;
                multi_start = v
                    .parse::<usize>()
                    .map_err(|_| format!("invalid integer for {flag}: {v}"))?
                    .max(1);
            }
            "-inc" | "--ignore-net-classes" => {
                let v = next_val(&mut i, flag, inline_val)?;
                for item in v.split(',') {
                    let trimmed = item.trim();
                    if !trimmed.is_empty() {
                        ignored_nets.insert(trimmed.to_string());
                    }
                }
            }
            "-mt" | "--threads" => {
                let v = next_val(&mut i, flag, inline_val)?;
                threads = v
                    .parse::<usize>()
                    .map_err(|_| format!("invalid integer for {flag}: {v}"))?
                    .max(1);
            }
            "-dl" | "--disable-log" => disable_log = true,
            "--via-at-smd" => via_at_smd = Some(true),
            "--no-via-at-smd" => via_at_smd = Some(false),
            "--timeout" | "--max-time" => {
                let v = next_val(&mut i, flag, inline_val)?;
                timeout_secs = Some(
                    v.parse::<u64>()
                        .map_err(|_| format!("invalid integer for {flag}: {v}"))?,
                );
            }
            "--json-summary" => json_summary = true,
            other => return Err(format!("unknown argument: {other}")),
        }
        i += 1;
    }

    let input_dsn = input_dsn.ok_or("missing required input DSN (-de <INPUT.dsn>)")?;
    let output_ses = output_ses.ok_or("missing required output SES (-do <OUTPUT.ses>)")?;

    Ok(Action::Run(Box::new(CliArgs {
        input_dsn,
        output_ses,
        rules_file,
        initial_session,
        report_file,
        diagnose,
        max_passes,
        multi_start,
        ignored_nets,
        threads,
        via_at_smd,
        timeout_secs,
        disable_log,
        json_summary,
    })))
}

fn run(cli: CliArgs) -> Result<(), String> {
    signal::install_signal_handlers();

    let dsn_text = fs::read_to_string(&cli.input_dsn)
        .map_err(|e| format!("failed to read {}: {e}", cli.input_dsn))?;
    let mut design = DsnDesign::parse(&dsn_text)?;
    if let Some(ref rules_path) = cli.rules_file {
        let rules_text = fs::read_to_string(rules_path)
            .map_err(|e| format!("failed to read rules file {rules_path}: {e}"))?;
        design.apply_rules_file(&rules_text)?;
    }

    let board = Board::from_dsn(&design, &cli.ignored_nets, cli.via_at_smd)?;
    let initial_routes = if let Some(ref ses_path) = cli.initial_session {
        let ses_text = fs::read_to_string(ses_path)
            .map_err(|e| format!("failed to read initial session {ses_path}: {e}"))?;
        Some(parse_ses(&ses_text, &board)?)
    } else {
        None
    };

    let config = RouterConfig {
        max_passes: cli.max_passes,
        ignored_nets: cli.ignored_nets,
        via_at_smd: cli.via_at_smd,
        threads: cli.threads,
        timeout_secs: cli.timeout_secs,
        optimize: true,
        multi_start: cli.multi_start,
        initial_routes,
        stop_flag: None,
    };

    // Atomically checkpoint `-do` whenever a strictly better routing solution is found.
    let write_lock = Mutex::new(());
    let out_path = cli.output_ses.clone();
    let checkpoint = |routes: &[viahop::NetRoute], _score: RouteScore| {
        if let Ok(_guard) = write_lock.lock() {
            let ses_text = ses::write_ses(&design, &board, routes);
            let _ = write_atomic(&out_path, &ses_text);
        }
    };

    let inner = route_board_with_checkpoint(&board, &config, Some(&checkpoint));
    let ses_text = ses::write_ses(&design, &board, &inner.routes);
    write_atomic(&cli.output_ses, &ses_text)
        .map_err(|e| format!("failed to write {}: {e}", cli.output_ses))?;

    if let Some(ref report_path) = cli.report_file {
        let report = RouteReport::build(&board, &inner.routes, inner.stats.clone(), cli.diagnose);
        write_atomic(report_path, &report.to_json())
            .map_err(|e| format!("failed to write report {report_path}: {e}"))?;
    }

    let s = &inner.stats;
    if cli.json_summary {
        println!("{}", s.to_json());
    } else if !cli.disable_log {
        let stopped = if s.stopped_early { " [stopped]" } else { "" };
        eprintln!(
            "viahop: routed {}/{} nets ({} unconnected pairs), {} segs, {} vias, {:.1}mm wire, {} passes ({} starts), {} DRC issues, grid={}um, {}ms{stopped}",
            s.routed_nets,
            s.routable_nets,
            s.unconnected_pairs,
            s.total_segments,
            s.total_vias,
            s.total_wirelength_um / 1000.0,
            s.passes_run,
            s.starts_run,
            s.drc_violations,
            s.grid_step_um,
            s.elapsed_ms,
        );
    }
    Ok(())
}

fn write_atomic(path: &str, contents: &str) -> std::io::Result<()> {
    let target = Path::new(path);
    let tmp = format!("{}.{}.viahop.tmp", path, std::process::id());
    fs::write(&tmp, contents)?;
    fs::rename(&tmp, target)
}

fn print_help() {
    println!(
        "viahop {} - Fast, standalone, zero-dependency Specctra DSN/SES PCB autorouter\n\n\
USAGE:\n  viahop -de <INPUT.dsn> -do <OUTPUT.ses> [OPTIONS]\n\n\
OPTIONS:\n  \
  -de, --input <FILE>          Input Specctra DSN file\n  \
  -do, --output <FILE>         Output Specctra SES file (checkpointed on improvement)\n  \
  -mp, --max-passes <N>        Max rip-up-and-reroute passes per start (default: 20)\n  \
  --multi-start <N>            Parallel shuffled starts if unrouted remain (default: 4)\n  \
  --initial-session <FILE>     Resume routing/optimization from an existing SES file\n  \
  --report <FILE>              Write detailed JSON report (stats, unrouted, violations)\n  \
  --diagnose                   With --report, classify unrouted pairs (congestion vs blocked)\n  \
  -inc <LIST>                  Comma-separated ignored net classes or net names\n  \
  -mt, --threads <N>           Worker threads (default: 1)\n  \
  -dr, --rules <FILE>          Optional Specctra rules file\n  \
  -dl, --disable-log           Suppress summary log on stderr\n  \
  --via-at-smd                 Enable POFV (via-in-pad) on SMD pads\n  \
  --no-via-at-smd              Disable POFV (via-in-pad) on SMD pads\n  \
  --timeout, --max-time <SECS> Soft timeout in seconds (writes best board so far)\n  \
  --json-summary               Print JSON routing summary to stdout\n  \
  -h, --help                   Show this help message\n  \
  -V, --version                Show version",
        env!("CARGO_PKG_VERSION")
    );
}
