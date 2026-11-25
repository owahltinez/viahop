//! Structured JSON routing report and unrouted-connection diagnosis (`--report`, `--diagnose`).

use std::fmt::Write;

use crate::board::{Board, NetRoute};
use crate::drc::check_drc;
use crate::geometry::Point;
use crate::router::{diagnose_unrouted_pairs, RouteStats};

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
            let comma = if i + 1 < self.violations.len() { "," } else { "" };
            let net_b = v.net_b.as_deref().map_or_else(|| "null".to_string(), json_str);
            let layer = v.layer.as_deref().map_or_else(|| "null".to_string(), json_str);
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
