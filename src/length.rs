//! Length and skew constraints (`--max-length`, `--match-group`) with DRC-gated meander tuning.

use std::cmp::Ordering;
use std::collections::{HashMap, HashSet};
use std::f64::consts::SQRT_2;

use crate::board::{Board, NetRoute, RoutedSegment};
use crate::connectivity::count_unconnected_pairs;
use crate::drc::check_drc;
use crate::geometry::{BBox, Point, point_to_segment_dist, segment_to_segment_dist};
use crate::optimize::{
    RouteSpatialIndex, find_tapped_segments, is_octilinear, route_wirelength, segment_clear,
};
use crate::router::{Terminal, build_physical_components};

const LENGTH_TOL_EPS_UM: f64 = 0.5;
const MIN_BUMP_H_UM: f64 = 0.4;

/// Maximum total copper length budget across one net or a chained series of nets.
#[derive(Debug, Clone, PartialEq)]
pub struct MaxLengthConstraint {
    pub nets: Vec<String>,
    pub max_um: f64,
}

/// Matched-length skew constraint across two or more nets with an optional target length.
#[derive(Debug, Clone, PartialEq)]
pub struct MatchGroupConstraint {
    pub name: String,
    pub nets: Vec<String>,
    pub tolerance_um: f64,
    pub target_um: Option<f64>,
}

/// Collection of max-length and match-group constraints for a routing run.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct LengthConstraints {
    pub max_lengths: Vec<MaxLengthConstraint>,
    pub match_groups: Vec<MatchGroupConstraint>,
}

impl LengthConstraints {
    pub fn is_empty(&self) -> bool {
        self.max_lengths.is_empty() && self.match_groups.is_empty()
    }

    /// Validates that every constrained net exists on `board` and is not ignored.
    pub fn validate(&self, board: &Board) -> Result<(), String> {
        for ml in &self.max_lengths {
            let label = ml.nets.join("+");
            for (name, nid_opt) in resolve_max_length_nets(board, ml) {
                let Some(nid) = nid_opt else {
                    return Err(format!(
                        "--max-length '{label}' references unknown net '{name}'"
                    ));
                };
                if board.nets[nid].ignored {
                    return Err(format!(
                        "--max-length '{label}' references ignored net '{name}'"
                    ));
                }
            }
        }
        for mg in &self.match_groups {
            for name in &mg.nets {
                let Some(&nid) = board.net_name_to_id.get(name) else {
                    return Err(format!(
                        "--match-group '{}' references unknown net '{name}'",
                        mg.name
                    ));
                };
                if board.nets[nid].ignored {
                    return Err(format!(
                        "--match-group '{}' references ignored net '{name}'",
                        mg.name
                    ));
                }
            }
        }
        Ok(())
    }

    pub(crate) fn max_length_net_ids(&self, board: &Board) -> HashSet<usize> {
        let mut ids = HashSet::new();
        for ml in &self.max_lengths {
            for (_, nid_opt) in resolve_max_length_nets(board, ml) {
                if let Some(nid) = nid_opt {
                    ids.insert(nid);
                }
            }
        }
        ids
    }

    pub(crate) fn violating_max_length_net_ids(
        &self,
        board: &Board,
        routes: &[NetRoute],
    ) -> HashSet<usize> {
        let mut ids = HashSet::new();
        for ml in &self.max_lengths {
            let m = measure_max_length(board, routes, ml);
            if m.exceeds_budget {
                ids.extend(m.net_ids);
            }
        }
        ids
    }
}

/// A single violated max-length or match-group constraint.
#[derive(Debug, Clone)]
pub struct LengthViolation {
    pub kind: &'static str,
    pub name: String,
    pub nets: Vec<String>,
    pub actual_um: f64,
    pub limit_um: f64,
    pub excess_um: f64,
}

/// Evaluated result for a single `--max-length` constraint.
#[derive(Debug, Clone)]
pub struct MaxLengthResult {
    pub nets: Vec<String>,
    pub actual_um: f64,
    pub max_um: f64,
    pub passed: bool,
}

/// Measured copper length of one member net in a match group.
#[derive(Debug, Clone)]
pub struct MatchGroupMember {
    pub net: String,
    pub length_um: f64,
}

/// Evaluated result for a single `--match-group` constraint.
#[derive(Debug, Clone)]
pub struct MatchGroupResult {
    pub name: String,
    pub members: Vec<MatchGroupMember>,
    pub min_um: f64,
    pub max_um: f64,
    pub target_um: f64,
    pub skew_um: f64,
    pub tolerance_um: f64,
    pub passed: bool,
}

/// Complete length evaluation report across all configured constraints.
#[derive(Debug, Clone, Default)]
pub struct LengthReport {
    pub max_lengths: Vec<MaxLengthResult>,
    pub match_groups: Vec<MatchGroupResult>,
    pub violations: Vec<LengthViolation>,
}

/// Parses a length value with optional `mm`, `um`, or `mil` suffix (defaults to `mm`).
fn parse_length_um(raw: &str) -> Result<f64, String> {
    let s = raw.trim();
    if s.is_empty() {
        return Err("empty length value".to_string());
    }
    let lower = s.to_ascii_lowercase();
    let (num_str, scale) = if let Some(prefix) = lower.strip_suffix("mil") {
        (&s[..prefix.len()], 25.4)
    } else if let Some(prefix) = lower.strip_suffix("mm") {
        (&s[..prefix.len()], 1000.0)
    } else if let Some(prefix) = lower.strip_suffix("um") {
        (&s[..prefix.len()], 1.0)
    } else {
        (s, 1000.0)
    };
    let val = num_str
        .trim()
        .parse::<f64>()
        .map_err(|_| format!("invalid length value: {raw}"))?;
    let um = val * scale;
    if !val.is_finite() || val <= 0.0 || !um.is_finite() || um <= 0.0 {
        return Err(format!("length must be positive and finite: {raw}"));
    }
    Ok(um)
}

/// Parses `NET[,NET2...|+NET2...]=LEN` (e.g. `CLK=22mm`, `/A,/B=40mm`, or `/A+/B=40mm`).
pub fn parse_max_length_spec(spec: &str) -> Result<MaxLengthConstraint, String> {
    let (lhs_raw, rhs) = spec
        .split_once('=')
        .ok_or_else(|| format!("invalid --max-length spec (expected NET=LEN): {spec}"))?;
    let lhs = lhs_raw.trim();
    let mut nets = Vec::new();
    let mut seen = HashSet::new();

    if lhs.contains(',') {
        for tok in lhs.split(',') {
            let name = tok.trim();
            if name.is_empty() {
                return Err(format!("empty net name in --max-length spec: {spec}"));
            }
            if !seen.insert(name.to_string()) {
                return Err(format!(
                    "duplicate net '{name}' in --max-length spec: {spec}"
                ));
            }
            nets.push(name.to_string());
        }
    } else if lhs.starts_with('+') || lhs.ends_with('+') {
        if lhs.chars().all(|c| c == '+') {
            return Err(format!("empty net name in --max-length spec: {spec}"));
        }
        nets.push(lhs.to_string());
    } else {
        for tok in lhs.split('+') {
            let name = tok.trim();
            if name.is_empty() {
                return Err(format!("empty net name in --max-length spec: {spec}"));
            }
            if !seen.insert(name.to_string()) {
                return Err(format!(
                    "duplicate net '{name}' in --max-length spec: {spec}"
                ));
            }
            nets.push(name.to_string());
        }
    }

    let max_um =
        parse_length_um(rhs).map_err(|e| format!("invalid --max-length value in '{spec}': {e}"))?;
    Ok(MaxLengthConstraint { nets, max_um })
}

/// Parses `NAME:NET1,NET2,...:TOL[@TARGET]` (e.g. `QSPI:CLK,IO0,IO1:0.5mm` or `DP:P,N:0.1mm@15mm`).
pub fn parse_match_group_spec(spec: &str) -> Result<MatchGroupConstraint, String> {
    let (name_raw, rest) = spec.split_once(':').ok_or_else(|| {
        format!("invalid --match-group spec (expected NAME:NET1,NET2,...:TOL[@TARGET]): {spec}")
    })?;
    let (nets_raw, tol_raw) = rest.rsplit_once(':').ok_or_else(|| {
        format!("invalid --match-group spec (expected NAME:NET1,NET2,...:TOL[@TARGET]): {spec}")
    })?;
    let name = name_raw.trim();
    if name.is_empty() {
        return Err(format!("empty group name in --match-group spec: {spec}"));
    }
    let mut nets = Vec::new();
    let mut seen = HashSet::new();
    for tok in nets_raw.split(',') {
        let n = tok.trim();
        if n.is_empty() {
            return Err(format!("empty net name in --match-group spec: {spec}"));
        }
        if !seen.insert(n.to_string()) {
            return Err(format!("duplicate net '{n}' in --match-group spec: {spec}"));
        }
        nets.push(n.to_string());
    }
    if nets.len() < 2 {
        return Err(format!(
            "--match-group '{name}' requires at least 2 nets: {spec}"
        ));
    }
    let (tol_str, target_opt) = match tol_raw.split_once('@') {
        Some((t, tgt)) => (
            t,
            Some(
                parse_length_um(tgt)
                    .map_err(|e| format!("invalid target length in '{spec}': {e}"))?,
            ),
        ),
        None => (tol_raw, None),
    };
    let tolerance_um =
        parse_length_um(tol_str).map_err(|e| format!("invalid tolerance in '{spec}': {e}"))?;
    Ok(MatchGroupConstraint {
        name: name.to_string(),
        nets,
        tolerance_um,
        target_um: target_opt,
    })
}

/// Computes total copper length (routed segments plus board fixed segments) for `net_id` in um.
pub fn net_total_length_um(board: &Board, routes: &[NetRoute], net_id: usize) -> f64 {
    let routed = find_net_route(routes, net_id).map_or(0.0, route_wirelength);
    let fixed = net_fixed_length_um(board, net_id);
    routed + fixed
}

fn find_net_route(routes: &[NetRoute], net_id: usize) -> Option<&NetRoute> {
    routes
        .get(net_id)
        .filter(|r| r.net_id == net_id)
        .or_else(|| routes.iter().find(|r| r.net_id == net_id))
}

fn net_fixed_length_um(board: &Board, net_id: usize) -> f64 {
    board
        .fixed_segments
        .iter()
        .filter(|s| s.net_id == Some(net_id))
        .map(|s| s.a.dist(s.b))
        .sum()
}

fn net_is_unrouted(board: &Board, routes: &[NetRoute], net_id: usize) -> bool {
    if board.nets[net_id].ignored {
        return true;
    }
    let nr_opt = find_net_route(routes, net_id);
    if nr_opt.is_some_and(|nr| nr.unconnected_pairs > 0) {
        return true;
    }
    if board.nets[net_id].pad_ids.len() < 2 {
        return false;
    }
    if nr_opt.is_none_or(|nr| nr.segments.is_empty()) {
        return build_physical_components(board, net_id).len() > 1;
    }
    false
}

fn resolve_max_length_nets(
    board: &Board,
    ml: &MaxLengthConstraint,
) -> Vec<(String, Option<usize>)> {
    if ml.nets.len() > 1 {
        let joined = ml.nets.join("+");
        if let Some(&nid) = board.net_name_to_id.get(&joined) {
            return vec![(joined, Some(nid))];
        }
    }
    ml.nets
        .iter()
        .map(|name| (name.clone(), board.net_name_to_id.get(name).copied()))
        .collect()
}

struct MaxLengthMeasurement {
    nets: Vec<String>,
    net_ids: Vec<usize>,
    actual_um: f64,
    has_invalid_or_unrouted: bool,
    exceeds_budget: bool,
}

fn measure_max_length(
    board: &Board,
    routes: &[NetRoute],
    ml: &MaxLengthConstraint,
) -> MaxLengthMeasurement {
    let resolved = resolve_max_length_nets(board, ml);
    let mut nets = Vec::with_capacity(resolved.len());
    let mut net_ids = Vec::with_capacity(resolved.len());
    let mut actual_um = 0.0;
    let mut has_invalid_or_unrouted = resolved.is_empty();
    for (name, nid_opt) in resolved {
        nets.push(name);
        match nid_opt {
            Some(nid) => {
                net_ids.push(nid);
                if net_is_unrouted(board, routes, nid) {
                    has_invalid_or_unrouted = true;
                }
                actual_um += net_total_length_um(board, routes, nid);
            }
            None => has_invalid_or_unrouted = true,
        }
    }
    let exceeds_budget = !net_ids.is_empty() && actual_um > ml.max_um + LENGTH_TOL_EPS_UM;
    MaxLengthMeasurement {
        nets,
        net_ids,
        actual_um,
        has_invalid_or_unrouted,
        exceeds_budget,
    }
}

/// Evaluates all max-length and match-group constraints against current routed + fixed copper.
pub fn evaluate_lengths(
    board: &Board,
    routes: &[NetRoute],
    constraints: &LengthConstraints,
) -> LengthReport {
    let mut report = LengthReport::default();
    if constraints.is_empty() {
        return report;
    }

    for ml in &constraints.max_lengths {
        let m = measure_max_length(board, routes, ml);
        let passed = !m.has_invalid_or_unrouted && !m.exceeds_budget;
        if !passed {
            report.violations.push(LengthViolation {
                kind: "max_length",
                name: m.nets.join("+"),
                nets: m.nets.clone(),
                actual_um: m.actual_um,
                limit_um: ml.max_um,
                excess_um: (m.actual_um - ml.max_um).max(0.0),
            });
        }
        report.max_lengths.push(MaxLengthResult {
            nets: m.nets,
            actual_um: m.actual_um,
            max_um: ml.max_um,
            passed,
        });
    }

    for mg in &constraints.match_groups {
        let mut members = Vec::with_capacity(mg.nets.len());
        let mut has_invalid_or_unrouted = false;
        for name in &mg.nets {
            match board.net_name_to_id.get(name) {
                Some(&nid) => {
                    if net_is_unrouted(board, routes, nid) {
                        has_invalid_or_unrouted = true;
                    }
                    members.push(MatchGroupMember {
                        net: name.clone(),
                        length_um: net_total_length_um(board, routes, nid),
                    });
                }
                None => has_invalid_or_unrouted = true,
            }
        }
        if members.is_empty() {
            report.violations.push(LengthViolation {
                kind: "match_group",
                name: mg.name.clone(),
                nets: mg.nets.clone(),
                actual_um: 0.0,
                limit_um: mg.tolerance_um,
                excess_um: 0.0,
            });
            report.match_groups.push(MatchGroupResult {
                name: mg.name.clone(),
                members,
                min_um: 0.0,
                max_um: 0.0,
                target_um: mg.target_um.unwrap_or(0.0),
                skew_um: 0.0,
                tolerance_um: mg.tolerance_um,
                passed: false,
            });
            continue;
        }
        let min_um = members
            .iter()
            .map(|m| m.length_um)
            .fold(f64::INFINITY, f64::min);
        let max_um = members
            .iter()
            .map(|m| m.length_um)
            .fold(f64::NEG_INFINITY, f64::max);
        let skew_um = max_um - min_um;
        let target_um = mg.target_um.unwrap_or(max_um);
        let max_target_dev = match mg.target_um {
            Some(t) => members
                .iter()
                .map(|m| (m.length_um - t).abs())
                .fold(0.0_f64, f64::max),
            None => 0.0,
        };
        let worst_err = skew_um.max(max_target_dev);
        let passed = !has_invalid_or_unrouted && worst_err <= mg.tolerance_um + LENGTH_TOL_EPS_UM;
        if !passed {
            report.violations.push(LengthViolation {
                kind: "match_group",
                name: mg.name.clone(),
                nets: mg.nets.clone(),
                actual_um: worst_err,
                limit_um: mg.tolerance_um,
                excess_um: (worst_err - mg.tolerance_um).max(0.0),
            });
        }
        report.match_groups.push(MatchGroupResult {
            name: mg.name.clone(),
            members,
            min_um,
            max_um,
            target_um,
            skew_um,
            tolerance_um: mg.tolerance_um,
            passed,
        });
    }

    report
}

/// Inserts DRC-gated octilinear trombone meanders into shorter match-group members.
pub fn tune_match_groups(board: &Board, routes: &mut [NetRoute], constraints: &LengthConstraints) {
    tune_match_groups_with_context(board, routes, constraints, None, &|| false);
}

pub(crate) fn tune_match_groups_with_context(
    board: &Board,
    routes: &mut [NetRoute],
    constraints: &LengthConstraints,
    net_components: Option<&[Vec<Vec<Terminal>>]>,
    is_cancelled: &dyn Fn() -> bool,
) {
    if constraints.match_groups.is_empty() || is_cancelled() {
        return;
    }

    let route_idx_by_net: HashMap<usize, usize> = routes
        .iter()
        .enumerate()
        .map(|(i, nr)| (nr.net_id, i))
        .collect();
    let mut index = RouteSpatialIndex::build(board, routes);
    let mut qbuf = Vec::new();
    let mut sbuf = Vec::new();
    let base_drc = check_drc(board, routes).violations.len();
    let mut local_comps: HashMap<usize, Vec<Vec<Terminal>>> = HashMap::new();

    let max_outer = constraints.match_groups.len().min(4);
    for _pass in 0..max_outer {
        if is_cancelled() {
            break;
        }
        // Process highest-target groups first so shared nets settle before shorter peers tune.
        let mut group_order: Vec<(usize, f64)> = constraints
            .match_groups
            .iter()
            .enumerate()
            .map(|(gi, group)| {
                let target = group_target_um(board, routes, constraints, group).unwrap_or(0.0);
                (gi, target)
            })
            .collect();
        group_order.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(Ordering::Equal));

        let mut any_net_changed = false;
        for (gi, _) in group_order {
            if is_cancelled() {
                return;
            }
            let group = &constraints.match_groups[gi];
            let member_nids = active_group_net_ids(board, group);
            if member_nids.is_empty() || (member_nids.len() < 2 && group.target_um.is_none()) {
                continue;
            }
            let Some(target) = group_target_um(board, routes, constraints, group) else {
                continue;
            };
            let stop_eps = tune_stop_eps(group.tolerance_um);

            // Tune nets with the largest deficit first so they get priority on open corridors.
            let mut work: Vec<(usize, usize, f64)> = Vec::new();
            for &nid in &member_nids {
                let Some(&ridx) = route_idx_by_net.get(&nid) else {
                    continue;
                };
                if routes[ridx].segments.is_empty() || routes[ridx].unconnected_pairs > 0 {
                    continue;
                }
                let cur_len = net_total_length_um(board, routes, nid);
                let cap = member_max_length_cap(board, routes, constraints, nid)
                    .map_or(target, |c| target.min(c));
                if cur_len < cap - stop_eps {
                    work.push((nid, ridx, cap - cur_len));
                }
            }
            work.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap_or(Ordering::Equal));

            for (nid, ridx, _) in work {
                if is_cancelled() {
                    return;
                }
                let cap = member_max_length_cap(board, routes, constraints, nid)
                    .map_or(target, |c| target.min(c));
                let comps: &[Vec<Terminal>] = match net_components {
                    Some(all) => &all[nid],
                    None => local_comps
                        .entry(nid)
                        .or_insert_with(|| build_physical_components(board, nid)),
                };
                let base_unconn = count_unconnected_pairs(board, comps, &routes[ridx]);
                let saved = routes[ridx].clone();
                let target_spec = TuneTarget {
                    route_idx: ridx,
                    fixed_len: net_fixed_length_um(board, nid),
                    target_um: cap,
                    tolerance_um: group.tolerance_um,
                };
                let changed = tune_single_net(
                    board,
                    &mut routes[ridx],
                    target_spec,
                    &mut index,
                    &mut qbuf,
                    &mut sbuf,
                    is_cancelled,
                );
                if changed {
                    let new_unconn = count_unconnected_pairs(board, comps, &routes[ridx]);
                    if new_unconn > base_unconn
                        || check_drc(board, routes).violations.len() > base_drc
                    {
                        routes[ridx] = saved;
                        index.update_net(ridx, &routes[ridx]);
                    } else {
                        routes[ridx].unconnected_pairs = new_unconn;
                        any_net_changed = true;
                    }
                }
            }
        }
        if !any_net_changed {
            break;
        }
    }
}

fn active_group_net_ids(board: &Board, group: &MatchGroupConstraint) -> Vec<usize> {
    group
        .nets
        .iter()
        .filter_map(|name| {
            board
                .net_name_to_id
                .get(name)
                .copied()
                .filter(|&nid| !board.nets[nid].ignored)
        })
        .collect()
}

fn group_target_um(
    board: &Board,
    routes: &[NetRoute],
    constraints: &LengthConstraints,
    group: &MatchGroupConstraint,
) -> Option<f64> {
    let member_nids = active_group_net_ids(board, group);
    if member_nids.is_empty() {
        return None;
    }
    let max_len = member_nids
        .iter()
        .map(|&nid| net_total_length_um(board, routes, nid))
        .fold(0.0_f64, f64::max);
    let mut target = match group.target_um {
        Some(t) => {
            if max_len > t && max_len <= t + group.tolerance_um + LENGTH_TOL_EPS_UM {
                max_len.min(t + group.tolerance_um)
            } else {
                t
            }
        }
        None => max_len,
    };
    if group.target_um.is_none() {
        for &nid in &member_nids {
            if let Some(cap) = member_max_length_cap(board, routes, constraints, nid) {
                target = target.min(cap);
            }
        }
    }
    Some(target)
}

#[inline]
fn tune_stop_eps(tolerance_um: f64) -> f64 {
    (tolerance_um * 0.15)
        .min(40.0)
        .max((tolerance_um * 0.25).min(0.5))
}

/// Tightest upper length limit for `net_id` implied by any `--max-length` constraint.
fn member_max_length_cap(
    board: &Board,
    routes: &[NetRoute],
    constraints: &LengthConstraints,
    net_id: usize,
) -> Option<f64> {
    let mut best: Option<f64> = None;
    for ml in &constraints.max_lengths {
        let resolved = resolve_max_length_nets(board, ml);
        if !resolved.iter().any(|&(_, oid)| oid == Some(net_id)) {
            continue;
        }
        let other_sum: f64 = resolved
            .iter()
            .filter_map(|&(_, oid)| oid.filter(|&id| id != net_id))
            .map(|oid| net_total_length_um(board, routes, oid))
            .sum();
        let cap = (ml.max_um - other_sum).max(0.0);
        best = Some(best.map_or(cap, |b| b.min(cap)));
    }
    best
}

/// Added length of a chamfered octilinear bump of height `h` and chamfer leg `c`.
#[inline]
fn delta_for_height(h: f64, c: f64) -> f64 {
    if h <= 2.0 * c + 0.5 {
        2.0 * (SQRT_2 - 1.0) * h
    } else {
        2.0 * h - 4.0 * c * (2.0 - SQRT_2)
    }
}

/// Exact inverse of `delta_for_height`: bump height needed to add `delta` um without overshoot.
#[inline]
fn height_for_delta(delta: f64, c: f64) -> f64 {
    let switch_4pt = 2.0 * (SQRT_2 - 1.0) * (2.0 * c + 0.5);
    let switch_8pt = 4.0 * (SQRT_2 - 1.0) * c + 1.0;
    if delta <= switch_4pt {
        delta / (2.0 * (SQRT_2 - 1.0))
    } else if delta <= switch_8pt {
        2.0 * c + 0.5
    } else {
        0.5 * delta + 2.0 * c * (2.0 - SQRT_2)
    }
}

struct TuneTarget {
    route_idx: usize,
    fixed_len: f64,
    target_um: f64,
    tolerance_um: f64,
}

struct Candidate {
    si: usize,
    seg: RoutedSegment,
    len: f64,
    u: Point,
    n: Point,
    s_min: f64,
    s_max: f64,
    c: f64,
}

struct ClearCtx<'a> {
    board: &'a Board,
    index: &'a RouteSpatialIndex,
    qbuf: &'a mut Vec<usize>,
    sbuf: &'a mut Vec<usize>,
    clr: f64,
    refine_eps: f64,
}

fn tune_single_net(
    board: &Board,
    nr: &mut NetRoute,
    spec: TuneTarget,
    index: &mut RouteSpatialIndex,
    qbuf: &mut Vec<usize>,
    sbuf: &mut Vec<usize>,
    is_cancelled: &dyn Fn() -> bool,
) -> bool {
    let rules = &board.nets[nr.net_id].rules;
    let clr = rules.clearance;
    let stop_eps = tune_stop_eps(spec.tolerance_um);
    let refine_eps = (stop_eps * 0.5).clamp(0.25, 5.0);
    let board_span = (board.bounds.max_x - board.bounds.min_x)
        .max(board.bounds.max_y - board.bounds.min_y)
        .max(2000.0);
    let pref_cap = (7.5 * (rules.width + clr)).max(2000.0);
    let caps = [pref_cap, board_span];

    let mut pref_side = 1.0_f64;
    let mut bumps_added = 0usize;
    let mut any_changed = false;

    for &h_cap in &caps {
        while bumps_added < 128 {
            if is_cancelled() {
                return any_changed;
            }
            let cur_len = spec.fixed_len + route_wirelength(nr);
            let needed = spec.target_um - cur_len;
            if needed <= stop_eps {
                return any_changed;
            }

            let candidates = collect_candidates(board, nr, clr);
            let mut ctx = ClearCtx {
                board,
                index,
                qbuf,
                sbuf,
                clr,
                refine_eps,
            };
            let Some((ci, s0, chosen_side, bump)) = find_best_bump(
                &mut ctx,
                nr,
                &candidates,
                needed,
                h_cap,
                pref_side,
                is_cancelled,
            ) else {
                break;
            };
            apply_bump(nr, &candidates[ci], s0, bump);
            index.update_net(spec.route_idx, nr);
            any_changed = true;
            bumps_added += 1;
            pref_side = -chosen_side;
        }
    }

    any_changed
}

fn collect_candidates(board: &Board, nr: &NetRoute, clr: f64) -> Vec<Candidate> {
    let tapped = find_tapped_segments(board, nr);
    let mut candidates = Vec::new();
    for (si, seg) in nr.segments.iter().enumerate() {
        if tapped[si] || !is_octilinear(seg.a, seg.b) {
            continue;
        }
        let len = seg.a.dist(seg.b);
        let c = 0.5 * (seg.width + clr);
        let span = 6.0 * c;
        let sa = endpoint_standoff(board, nr, seg.layer, seg.a, seg.width, clr);
        let sb = endpoint_standoff(board, nr, seg.layer, seg.b, seg.width, clr);
        if len >= span + sa + sb {
            let u = Point::new((seg.b.x - seg.a.x) / len, (seg.b.y - seg.a.y) / len);
            let n = Point::new(-u.y, u.x);
            candidates.push(Candidate {
                si,
                seg: seg.clone(),
                len,
                u,
                n,
                s_min: sa,
                s_max: len - sb - span,
                c,
            });
        }
    }
    candidates.sort_by(|a, b| b.len.partial_cmp(&a.len).unwrap_or(Ordering::Equal));
    candidates
}

fn find_best_bump(
    ctx: &mut ClearCtx<'_>,
    nr: &NetRoute,
    candidates: &[Candidate],
    needed: f64,
    h_cap: f64,
    pref_side: f64,
    is_cancelled: &dyn Fn() -> bool,
) -> Option<(usize, f64, f64, Vec<RoutedSegment>)> {
    // Fast path: try a single centered exact-length bump on the longest clear segment.
    for (ci, cand) in candidates.iter().enumerate() {
        if is_cancelled() {
            return None;
        }
        let h_req = height_for_delta(needed, cand.c);
        if h_req > h_cap {
            continue;
        }
        let mid_s0 = 0.5 * (cand.s_min + cand.s_max);
        for side in [pref_side, -pref_side] {
            let bump = build_bump_segments(cand, mid_s0, h_req, side);
            if bump_is_clear(ctx, nr, cand.si, &bump) {
                return Some((ci, mid_s0, side, bump));
            }
        }
    }

    // General path: pack DRC-clean bumps along open sub-intervals of eligible segments.
    for (ci, cand) in candidates.iter().enumerate() {
        let step = ((cand.seg.width + ctx.clr) * 0.5).max(50.0);
        let h_hi = height_for_delta(needed, cand.c).min(h_cap);
        let h_min = (cand.seg.width * 0.2)
            .max(15.0)
            .min(h_hi * 0.5)
            .max(MIN_BUMP_H_UM)
            .min(h_hi);
        let min_useful_delta = needed.min(10.0).min(delta_for_height(h_min, cand.c));

        let mut s0 = cand.s_min;
        loop {
            if is_cancelled() {
                return None;
            }
            let mut best_choice: Option<(f64, f64, Vec<RoutedSegment>)> = None;
            for side in [pref_side, -pref_side] {
                if let Some((h, bump)) = find_max_clear_height(ctx, nr, cand, s0, h_min, h_hi, side)
                {
                    let d = delta_for_height(h, cand.c);
                    let better = match best_choice {
                        None => d >= min_useful_delta,
                        Some((best_d, _, _)) => d > best_d + 1.0,
                    };
                    if better {
                        best_choice = Some((d, side, bump));
                    }
                }
            }
            if let Some((_, chosen_side, bump)) = best_choice {
                return Some((ci, s0, chosen_side, bump));
            }
            if s0 >= cand.s_max - 1e-6 {
                break;
            }
            s0 = (s0 + step).min(cand.s_max);
        }
    }
    None
}

fn find_max_clear_height(
    ctx: &mut ClearCtx<'_>,
    nr: &NetRoute,
    cand: &Candidate,
    s0: f64,
    h_min: f64,
    h_hi: f64,
    side: f64,
) -> Option<(f64, Vec<RoutedSegment>)> {
    let bump_hi = build_bump_segments(cand, s0, h_hi, side);
    if bump_is_clear(ctx, nr, cand.si, &bump_hi) {
        return Some((h_hi, bump_hi));
    }
    let bump_lo = build_bump_segments(cand, s0, h_min, side);
    if !bump_is_clear(ctx, nr, cand.si, &bump_lo) {
        return None;
    }

    let mut lo = h_min;
    let mut hi = h_hi;
    let mut best_bump = bump_lo;
    for _ in 0..12 {
        if hi - lo <= ctx.refine_eps {
            break;
        }
        let mid = 0.5 * (lo + hi);
        let bump = build_bump_segments(cand, s0, mid, side);
        if bump_is_clear(ctx, nr, cand.si, &bump) {
            lo = mid;
            best_bump = bump;
        } else {
            hi = mid;
        }
    }
    Some((lo, best_bump))
}

fn build_bump_segments(cand: &Candidate, s0: f64, h: f64, side: f64) -> Vec<RoutedSegment> {
    let c = cand.c;
    let top_w = 2.0 * c;
    let span = 6.0 * c;
    let h = h.max(MIN_BUMP_H_UM);
    let origin = cand.seg.a;
    let pt = |s: f64, y: f64| {
        Point::new(
            origin.x + cand.u.x * s + cand.n.x * (side * y),
            origin.y + cand.u.y * s + cand.n.y * (side * y),
        )
    };
    let pts: Vec<Point> = if h <= 2.0 * c + 0.5 {
        vec![
            pt(s0, 0.0),
            pt(s0 + h, h),
            pt(s0 + span - h, h),
            pt(s0 + span, 0.0),
        ]
    } else {
        vec![
            pt(s0, 0.0),
            pt(s0 + c, c),
            pt(s0 + c, h - c),
            pt(s0 + 2.0 * c, h),
            pt(s0 + 2.0 * c + top_w, h),
            pt(s0 + 3.0 * c + top_w, h - c),
            pt(s0 + 3.0 * c + top_w, c),
            pt(s0 + span, 0.0),
        ]
    };
    pts.windows(2)
        .map(|w| RoutedSegment {
            layer: cand.seg.layer,
            a: w[0],
            b: w[1],
            width: cand.seg.width,
        })
        .collect()
}

fn bump_is_clear(
    ctx: &mut ClearCtx<'_>,
    nr: &NetRoute,
    seg_idx: usize,
    bump: &[RoutedSegment],
) -> bool {
    let board = ctx.board;
    let net_id = nr.net_id;
    let req_self = ctx.clr + 0.5;
    for cand in bump {
        if !segment_clear(board, net_id, cand, ctx.index, ctx.qbuf, ctx.sbuf) {
            return false;
        }
        let half_w = cand.width * 0.5;
        // Self-clearance against non-replaced segments, vias, pads, and fixed copper of this net.
        for (i, other) in nr.segments.iter().enumerate() {
            if i == seg_idx || other.layer != cand.layer {
                continue;
            }
            let gap = segment_to_segment_dist(cand.a, cand.b, other.a, other.b)
                - half_w
                - other.width * 0.5;
            if gap < req_self {
                return false;
            }
        }
        for v in &nr.vias {
            if !v.layers.contains(&cand.layer) {
                continue;
            }
            if point_to_segment_dist(v.pos, cand.a, cand.b) - v.radius - half_w < req_self {
                return false;
            }
        }
        for &pid in &board.nets[net_id].pad_ids {
            if let Some(sh) = board.pads[pid].shape_on_layer(cand.layer)
                && sh.dist_to_capsule(cand.a, cand.b, half_w) < req_self
            {
                return false;
            }
        }
        let qbox = BBox::from_points(&[cand.a, cand.b], half_w + req_self + 50.0);
        board.fixed_seg_index.query(qbox, ctx.sbuf);
        for &sid in ctx.sbuf.iter() {
            let fs = &board.fixed_segments[sid];
            if fs.net_id != Some(net_id) || fs.layer != cand.layer {
                continue;
            }
            let gap = segment_to_segment_dist(cand.a, cand.b, fs.a, fs.b) - half_w - fs.half_w;
            if gap < req_self {
                return false;
            }
        }
        board.fixed_via_index.query(qbox, ctx.sbuf);
        for &vid in ctx.sbuf.iter() {
            let fv = &board.fixed_vias[vid];
            if fv.net_id != Some(net_id) || !fv.layers.contains(&cand.layer) {
                continue;
            }
            if point_to_segment_dist(fv.pos, cand.a, cand.b) - fv.radius - half_w < req_self {
                return false;
            }
        }
    }
    true
}

fn apply_bump(nr: &mut NetRoute, cand: &Candidate, s0: f64, bump: Vec<RoutedSegment>) {
    let orig = &cand.seg;
    let span = 6.0 * cand.c;
    let p_start = Point::new(orig.a.x + cand.u.x * s0, orig.a.y + cand.u.y * s0);
    let p_end = Point::new(
        orig.a.x + cand.u.x * (s0 + span),
        orig.a.y + cand.u.y * (s0 + span),
    );
    nr.segments.swap_remove(cand.si);
    if orig.a.dist(p_start) > 0.5 {
        nr.segments.push(RoutedSegment {
            layer: orig.layer,
            a: orig.a,
            b: p_start,
            width: orig.width,
        });
    }
    nr.segments.extend(bump);
    if p_end.dist(orig.b) > 0.5 {
        nr.segments.push(RoutedSegment {
            layer: orig.layer,
            a: p_end,
            b: orig.b,
            width: orig.width,
        });
    }
}

fn endpoint_standoff(
    board: &Board,
    nr: &NetRoute,
    layer: usize,
    pt: Point,
    width: f64,
    clr: f64,
) -> f64 {
    let mut s = width + clr + 5.0;
    let half_w = width * 0.5;
    for &pid in &board.nets[nr.net_id].pad_ids {
        let pad = &board.pads[pid];
        if pad.on_layer(layer) && pad.center.dist(pt) <= pad.max_radius + half_w + 5.0 {
            s = s.max(pad.max_radius + half_w + clr + 10.0);
        }
    }
    for v in &nr.vias {
        if v.layers.contains(&layer) && v.pos.dist(pt) <= v.radius + half_w + 5.0 {
            s = s.max(v.radius + half_w + clr + 10.0);
        }
    }
    for fv in &board.fixed_vias {
        if fv.net_id == Some(nr.net_id)
            && fv.layers.contains(&layer)
            && fv.pos.dist(pt) <= fv.radius + half_w + 5.0
        {
            s = s.max(fv.radius + half_w + clr + 10.0);
        }
    }
    s
}
