//! Structured JSON routing report and unrouted-connection diagnosis (`--report`, `--diagnose`).

use std::fmt::Write;

use crate::astar::{PassParams, SearchScratch, astar_connect};
use crate::board::{Board, NetRoute};
use crate::connectivity::connected_component_clusters;
use crate::drc::check_drc;
use crate::fanout::analyze_fanout;
use crate::geometry::Point;
use crate::grid::RoutingGrid;
use crate::router::{RouteStats, Terminal, build_mst, build_net_components};

/// Classification of why an unconnected net pair failed to route.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum UnroutedKind {
    /// Routable alone on an empty board; blocked by congestion from other nets.
    Congestion,
    /// Unroutable even alone on an empty board due to static geometry or design rules.
    Blocked,
}

#[derive(Debug, Clone)]
pub struct UnroutedConnection {
    pub net: String,
    pub class_name: String,
    pub from_pin: String,
    pub from_pos: Point,
    pub to_pin: String,
    pub to_pos: Point,
    pub diagnosis: Option<UnroutedKind>,
    pub reason: Option<String>,
}

#[derive(Debug, Clone)]
pub struct ViolationDetail {
    pub kind: String,
    pub net_a: String,
    pub net_b: Option<String>,
    pub layer: Option<String>,
    pub pos: Point,
    pub gap_um: f64,
    pub required_um: f64,
}

/// Complete machine-readable routing report for `--report` / `--diagnose`.
#[derive(Debug, Clone)]
pub struct RouteReport {
    pub stats: RouteStats,
    pub unrouted: Vec<UnroutedConnection>,
    pub violations: Vec<ViolationDetail>,
}

impl RouteReport {
    /// Builds a detailed report from a completed routing run. When `diagnose` is true,
    /// each unrouted pair is test-routed alone on an empty grid to distinguish congestion
    /// from hard static/rule blockages.
    pub fn build(board: &Board, routes: &[NetRoute], stats: RouteStats, diagnose: bool) -> Self {
        let drc = check_drc(board, routes);
        let violations = drc
            .violations
            .iter()
            .map(|v| ViolationDetail {
                kind: v.kind.to_string(),
                net_a: board.nets[v.net_a].name.clone(),
                net_b: v.net_b.map(|id| board.nets[id].name.clone()),
                layer: v.layer.map(|l| board.layer_names[l].clone()),
                pos: v.pos,
                gap_um: v.gap_um,
                required_um: v.required_um,
            })
            .collect();

        let unrouted = diagnose_unrouted_pairs(board, routes, diagnose);
        Self {
            stats,
            unrouted,
            violations,
        }
    }

    pub fn to_json(&self) -> String {
        let mut out = String::with_capacity(2048);
        let _ = writeln!(out, "{{");
        let _ = writeln!(out, "  \"stats\": {},", self.stats.to_json());

        let _ = writeln!(out, "  \"unrouted\": [");
        for (i, u) in self.unrouted.iter().enumerate() {
            let comma = if i + 1 < self.unrouted.len() { "," } else { "" };
            let diag = match u.diagnosis {
                Some(UnroutedKind::Congestion) => "\"congestion\"",
                Some(UnroutedKind::Blocked) => "\"blocked\"",
                None => "null",
            };
            let reason = match &u.reason {
                Some(r) => json_str(r),
                None => "null".to_string(),
            };
            let _ = writeln!(
                out,
                "    {{\"net\":{},\"class\":{},\"from_pin\":{},\"from_um\":[{:.1},{:.1}],\"to_pin\":{},\"to_um\":[{:.1},{:.1}],\"diagnosis\":{},\"reason\":{}}}{comma}",
                json_str(&u.net),
                json_str(&u.class_name),
                json_str(&u.from_pin),
                u.from_pos.x,
                u.from_pos.y,
                json_str(&u.to_pin),
                u.to_pos.x,
                u.to_pos.y,
                diag,
                reason,
            );
        }
        let _ = writeln!(out, "  ],");

        let _ = writeln!(out, "  \"violations\": [");
        for (i, v) in self.violations.iter().enumerate() {
            let comma = if i + 1 < self.violations.len() {
                ","
            } else {
                ""
            };
            let net_b = v
                .net_b
                .as_deref()
                .map_or_else(|| "null".to_string(), json_str);
            let layer = v
                .layer
                .as_deref()
                .map_or_else(|| "null".to_string(), json_str);
            let _ = writeln!(
                out,
                "    {{\"kind\":{},\"net_a\":{},\"net_b\":{},\"layer\":{},\"pos_um\":[{:.1},{:.1}],\"gap_um\":{:.2},\"required_um\":{:.2}}}{comma}",
                json_str(&v.kind),
                json_str(&v.net_a),
                net_b,
                layer,
                v.pos.x,
                v.pos.y,
                v.gap_um,
                v.required_um,
            );
        }
        let _ = writeln!(out, "  ]");
        let _ = write!(out, "}}");
        out
    }
}

fn diagnose_unrouted_pairs(
    board: &Board,
    routes: &[NetRoute],
    diagnose: bool,
) -> Vec<UnroutedConnection> {
    let fanout = analyze_fanout(board);
    let grid = RoutingGrid::build(board, &fanout, 1);
    let n2d = grid.n2d();
    let mut scratch = SearchScratch::new(grid.sig_layers.len() * n2d);
    let mut out = Vec::new();

    for net in &board.nets {
        if net.ignored || net.pad_ids.len() < 2 {
            continue;
        }
        let components = build_net_components(board, &grid, &fanout, net.id);
        if components.len() < 2 {
            continue;
        }
        let nr = routes.get(net.id).cloned().unwrap_or_else(|| NetRoute {
            net_id: net.id,
            ..Default::default()
        });
        let clusters = connected_component_clusters(board, &components, &nr);
        if clusters.len() <= 1 {
            continue;
        }

        let cluster_terms: Vec<Vec<Terminal>> = clusters
            .iter()
            .map(|cids| {
                cids.iter()
                    .flat_map(|&cid| components[cid].iter().cloned())
                    .collect()
            })
            .collect();
        let mst_edges = build_mst(&cluster_terms);
        let min_via_dist = board.min_via_spacing(net.id);

        for (ca, cb) in mst_edges {
            let (ta, tb) = closest_terminals(&cluster_terms[ca], &cluster_terms[cb]);
            let (diagnosis, reason) = if diagnose {
                let src = cluster_anchors(&cluster_terms[ca], n2d, grid.cols);
                let dst = cluster_anchors(&cluster_terms[cb], n2d, grid.cols);
                if src.is_empty() || dst.is_empty() {
                    (
                        Some(UnroutedKind::Blocked),
                        Some("no_legal_terminal_entry".to_string()),
                    )
                } else {
                    let params = PassParams {
                        present_factor: 1.0,
                        h_weight: 1.15,
                        strict_mode: true,
                        max_expansions: 1_200_000,
                    };
                    let solo = astar_connect(
                        &grid,
                        net.id,
                        &src,
                        &[],
                        &dst,
                        &[],
                        &[],
                        min_via_dist,
                        &params,
                        &mut scratch,
                    );
                    if solo.is_some() {
                        (
                            Some(UnroutedKind::Congestion),
                            Some("routable_alone_blocked_by_other_nets".to_string()),
                        )
                    } else {
                        (
                            Some(UnroutedKind::Blocked),
                            Some("static_obstacle_or_rule_blockage".to_string()),
                        )
                    }
                }
            } else {
                (None, None)
            };

            out.push(UnroutedConnection {
                net: net.name.clone(),
                class_name: net.class_name.clone(),
                from_pin: terminal_label(board, ta),
                from_pos: ta.exact,
                to_pin: terminal_label(board, tb),
                to_pos: tb.exact,
                diagnosis,
                reason,
            });
        }
    }

    out
}

fn cluster_anchors(
    terms: &[Terminal],
    n2d: usize,
    cols: usize,
) -> Vec<(u32, Point, Option<Point>, usize)> {
    let mut out = Vec::new();
    for t in terms {
        for &(sl, gx, gy, via_opt, pad_bl) in &t.entries {
            out.push(((sl * n2d + gy * cols + gx) as u32, t.exact, via_opt, pad_bl));
        }
    }
    out
}

fn closest_terminals<'a>(a: &'a [Terminal], b: &'a [Terminal]) -> (&'a Terminal, &'a Terminal) {
    let mut best = (&a[0], &b[0]);
    let mut best_d = f64::INFINITY;
    for ta in a {
        for tb in b {
            let d = ta.exact.dist(tb.exact);
            if d < best_d {
                best_d = d;
                best = (ta, tb);
            }
        }
    }
    best
}

fn terminal_label(board: &Board, t: &Terminal) -> String {
    if let Some(pid) = t.pad_id {
        let p = &board.pads[pid];
        format!("{}-{}", p.refdes, p.pin_name)
    } else {
        format!("fixed@({:.0},{:.0})", t.exact.x, t.exact.y)
    }
}

pub fn json_str(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for ch in s.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
    out
}
