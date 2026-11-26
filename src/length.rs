//! Length and skew constraints (`--max-length`, `--match-group`) with DRC-gated meander tuning.

use std::cmp::Ordering;
use std::collections::{HashMap, HashSet};
use std::f64::consts::SQRT_2;

use crate::board::{Board, NetRoute, RoutedSegment};
use crate::drc::check_drc;
use crate::geometry::{Point, point_to_segment_dist, segment_to_segment_dist};
use crate::optimize::{RouteSpatialIndex, is_octilinear, pt_key, route_wirelength};

/// Maximum total copper length budget across one net or a `+`-chained series of nets.
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

    pub(crate) fn max_length_net_ids(&self, board: &Board) -> HashSet<usize> {
        let mut ids = HashSet::new();
        for ml in &self.max_lengths {
            for name in &ml.nets {
                if let Some(&nid) = board.net_name_to_id.get(name) {
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
            let mut resolved = Vec::with_capacity(ml.nets.len());
            let mut total = 0.0;
            for name in &ml.nets {
                if let Some(&nid) = board.net_name_to_id.get(name) {
                    resolved.push(nid);
                    total += net_total_length_um(board, routes, nid);
                }
            }
            if !resolved.is_empty() && total > ml.max_um + 0.5 {
                ids.extend(resolved);
            }
        }
        ids
    }
}

/// Category of a length or skew constraint violation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LengthViolationKind {
    MaxLength,
    MatchGroup,
}

impl LengthViolationKind {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::MaxLength => "max_length",
            Self::MatchGroup => "match_group",
        }
    }
}

/// A single violated max-length or match-group constraint.
#[derive(Debug, Clone)]
pub struct LengthViolation {
    pub kind: LengthViolationKind,
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
    if !val.is_finite() || val <= 0.0 {
        return Err(format!("length must be positive and finite: {raw}"));
    }
    Ok(val * scale)
}

/// Parses `NET[+NET2...]=LEN` (e.g. `CLK=22mm` or `/A+/B=40mm`).
pub fn parse_max_length_spec(spec: &str) -> Result<MaxLengthConstraint, String> {
    let (lhs, rhs) = spec
        .split_once('=')
        .ok_or_else(|| format!("invalid --max-length spec (expected NET=LEN): {spec}"))?;
    let mut nets = Vec::new();
    for tok in lhs.split('+') {
        let name = tok.trim();
        if name.is_empty() {
            return Err(format!("empty net name in --max-length spec: {spec}"));
        }
        nets.push(name.to_string());
    }
    if nets.is_empty() {
        return Err(format!("no nets in --max-length spec: {spec}"));
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
    let routed = routes
        .get(net_id)
        .filter(|r| r.net_id == net_id)
        .or_else(|| routes.iter().find(|r| r.net_id == net_id))
        .map_or(0.0, route_wirelength);
    let fixed = net_fixed_length_um(board, net_id);
    routed + fixed
}

fn net_fixed_length_um(board: &Board, net_id: usize) -> f64 {
    board
        .fixed_segments
        .iter()
        .filter(|s| s.net_id == Some(net_id))
        .map(|s| s.a.dist(s.b))
        .sum()
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
        let mut actual_um = 0.0;
        let mut found_any = false;
        for name in &ml.nets {
            if let Some(&nid) = board.net_name_to_id.get(name) {
                found_any = true;
                actual_um += net_total_length_um(board, routes, nid);
            }
        }
        let passed = !found_any || actual_um <= ml.max_um + 0.5;
        if !passed {
            report.violations.push(LengthViolation {
                kind: LengthViolationKind::MaxLength,
                name: ml.nets.join("+"),
                nets: ml.nets.clone(),
                actual_um,
                limit_um: ml.max_um,
                excess_um: actual_um - ml.max_um,
            });
        }
        report.max_lengths.push(MaxLengthResult {
            nets: ml.nets.clone(),
            actual_um,
            max_um: ml.max_um,
            passed,
        });
    }

    for mg in &constraints.match_groups {
        let mut members = Vec::with_capacity(mg.nets.len());
        for name in &mg.nets {
            if let Some(&nid) = board.net_name_to_id.get(name) {
                members.push(MatchGroupMember {
                    net: name.clone(),
                    length_um: net_total_length_um(board, routes, nid),
                });
            }
        }
        if members.is_empty() {
            report.match_groups.push(MatchGroupResult {
                name: mg.name.clone(),
                members,
                min_um: 0.0,
                max_um: 0.0,
                target_um: mg.target_um.unwrap_or(0.0),
                skew_um: 0.0,
                tolerance_um: mg.tolerance_um,
                passed: true,
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
        let passed = worst_err <= mg.tolerance_um + 0.5;
        if !passed {
            report.violations.push(LengthViolation {
                kind: LengthViolationKind::MatchGroup,
                name: mg.name.clone(),
                nets: mg.nets.clone(),
                actual_um: worst_err,
                limit_um: mg.tolerance_um,
                excess_um: worst_err - mg.tolerance_um,
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
    if constraints.match_groups.is_empty() {
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

    for group in &constraints.match_groups {
        let mut member_nids = Vec::with_capacity(group.nets.len());
        for name in &group.nets {
            if let Some(&nid) = board.net_name_to_id.get(name)
                && !board.nets[nid].ignored
            {
                member_nids.push(nid);
            }
        }
        if member_nids.len() < 2 && group.target_um.is_none() {
            continue;
        }
        if member_nids.is_empty() {
            continue;
        }

        let lengths: Vec<(usize, f64)> = member_nids
            .iter()
            .map(|&nid| (nid, net_total_length_um(board, routes, nid)))
            .collect();
        let max_len = lengths.iter().map(|&(_, l)| l).fold(0.0_f64, f64::max);

        let mut target = group.target_um.unwrap_or(max_len);
        if group.target_um.is_none() {
            for &nid in &member_nids {
                if let Some(cap) = member_max_length_cap(board, routes, constraints, nid) {
                    target = target.min(cap);
                }
            }
        }

        // Tune nets with the largest deficit first so they get priority on open corridors.
        let mut work: Vec<(usize, usize, f64)> = Vec::new();
        for &(nid, cur_len) in &lengths {
            let Some(&ridx) = route_idx_by_net.get(&nid) else {
                continue;
            };
            if routes[ridx].segments.is_empty() || routes[ridx].unconnected_pairs > 0 {
                continue;
            }
            let cap = member_max_length_cap(board, routes, constraints, nid)
                .map_or(target, |c| target.min(c));
            let needs_tune = cur_len < cap - group.tolerance_um
                || (max_len - cur_len > group.tolerance_um && cur_len + 5.0 < cap);
            if needs_tune && cap > cur_len + 5.0 {
                work.push((nid, ridx, cap - cur_len));
            }
        }
        work.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap_or(Ordering::Equal));

        for (nid, ridx, _) in work {
            let cap = member_max_length_cap(board, routes, constraints, nid)
                .map_or(target, |c| target.min(c));
            let saved = routes[ridx].clone();
            let fixed_len = net_fixed_length_um(board, nid);
            let changed = tune_single_net(
                board,
                &mut routes[ridx],
                ridx,
                fixed_len,
                cap,
                group.tolerance_um,
                &mut index,
                &mut qbuf,
                &mut sbuf,
            );
            if changed && check_drc(board, routes).violations.len() > base_drc {
                routes[ridx] = saved;
                index.update_net(ridx, &routes[ridx]);
            }
        }
    }
}

/// Tightest upper length limit for `net_id` implied by any `--max-length` constraint.
fn member_max_length_cap(
    board: &Board,
    routes: &[NetRoute],
    constraints: &LengthConstraints,
    net_id: usize,
) -> Option<f64> {
    let target_name = &board.nets[net_id].name;
    let mut best: Option<f64> = None;
    for ml in &constraints.max_lengths {
        if !ml.nets.iter().any(|n| n == target_name) {
            continue;
        }
        let mut other_sum = 0.0;
        for n in &ml.nets {
            if n != target_name
                && let Some(&oid) = board.net_name_to_id.get(n)
            {
                other_sum += net_total_length_um(board, routes, oid);
            }
        }
        let cap = (ml.max_um - other_sum).max(0.0);
        best = Some(best.map_or(cap, |b| b.min(cap)));
    }
    best
}

/// Added length of a chamfered octilinear bump of height `h` and chamfer leg `c`.
#[inline]
fn delta_for_height(h: f64, c: f64) -> f64 {
    if h <= 2.0 * c {
        2.0 * (SQRT_2 - 1.0) * h
    } else {
        2.0 * h - 4.0 * c * (2.0 - SQRT_2)
    }
}

/// Exact inverse of `delta_for_height`: bump height needed to add `delta` um.
#[inline]
fn height_for_delta(delta: f64, c: f64) -> f64 {
    let switch = 4.0 * (SQRT_2 - 1.0) * c;
    if delta <= switch {
        delta / (2.0 * (SQRT_2 - 1.0))
    } else {
        0.5 * delta + 2.0 * c * (2.0 - SQRT_2)
    }
}

#[allow(clippy::too_many_arguments)]
fn tune_single_net(
    board: &Board,
    nr: &mut NetRoute,
    route_idx: usize,
    fixed_len: f64,
    target_um: f64,
    tolerance_um: f64,
    index: &mut RouteSpatialIndex,
    qbuf: &mut Vec<usize>,
    sbuf: &mut Vec<usize>,
) -> bool {
    let rules = &board.nets[nr.net_id].rules;
    let clr = rules.clearance;
    let stop_eps = (tolerance_um * 0.15).clamp(5.0, 40.0);
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
            let cur_len = fixed_len + route_wirelength(nr);
            let needed = target_um - cur_len;
            if needed <= stop_eps {
                return any_changed;
            }

            let tapped = find_tapped_segments(board, nr);
            let mut candidates: Vec<(usize, f64, f64, f64, f64, f64, f64)> = Vec::new();
            for (si, seg) in nr.segments.iter().enumerate() {
                if tapped[si] || !is_octilinear(seg.a, seg.b) {
                    continue;
                }
                let len = seg.a.dist(seg.b);
                let c = 0.5 * (seg.width + clr);
                let top_w = seg.width + clr;
                let span = 4.0 * c + top_w;
                let sa = endpoint_standoff(board, nr, seg.layer, seg.a, seg.width, clr);
                let sb = endpoint_standoff(board, nr, seg.layer, seg.b, seg.width, clr);
                if len >= span + sa + sb {
                    candidates.push((si, len, sa, sb, c, top_w, span));
                }
            }
            candidates.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(Ordering::Equal));

            let mut placed = false;

            // Fast path: try a single centered exact-length bump on the longest clear segment.
            for &(si, len, sa, sb, c, top_w, span) in &candidates {
                let h_req = height_for_delta(needed, c);
                if h_req > h_cap {
                    continue;
                }
                let seg = nr.segments[si].clone();
                let u = Point::new((seg.b.x - seg.a.x) / len, (seg.b.y - seg.a.y) / len);
                let n = Point::new(-u.y, u.x);
                let mid_s0 = 0.5 * (sa + (len - sb - span));
                for side in [pref_side, -pref_side] {
                    let bump = build_bump_segments(
                        seg.a, u, n, seg.layer, seg.width, mid_s0, c, top_w, h_req, side,
                    );
                    if bump_is_clear(board, nr, si, &bump, clr, index, qbuf, sbuf) {
                        apply_bump(nr, si, &seg, u, mid_s0, span, bump);
                        index.update_net(route_idx, nr);
                        any_changed = true;
                        bumps_added += 1;
                        pref_side = -side;
                        placed = true;
                        break;
                    }
                }
                if placed {
                    break;
                }
            }
            if placed {
                continue;
            }

            // General path: pack DRC-clean bumps along open sub-intervals of eligible segments.
            let min_useful_delta = needed.min(10.0);
            for &(si, len, sa, sb, c, top_w, span) in &candidates {
                let seg = nr.segments[si].clone();
                let u = Point::new((seg.b.x - seg.a.x) / len, (seg.b.y - seg.a.y) / len);
                let n = Point::new(-u.y, u.x);
                let s_min = sa;
                let s_max = len - sb - span;
                let step = ((seg.width + clr) * 0.5).max(50.0);
                let h_hi = height_for_delta(needed, c).min(h_cap);
                let h_min = (seg.width * 0.2).max(15.0).min(h_hi);

                let mut s0 = s_min;
                while s0 <= s_max + 1e-6 {
                    let clamped_s0 = s0.min(s_max);
                    let mut best_choice: Option<(f64, f64, Vec<RoutedSegment>)> = None;
                    for side in [pref_side, -pref_side] {
                        if let Some((h, bump)) = find_max_clear_height(
                            board, nr, si, &seg, u, n, clamped_s0, c, top_w, h_min, h_hi, side,
                            clr, index, qbuf, sbuf,
                        ) {
                            let d = delta_for_height(h, c);
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
                        apply_bump(nr, si, &seg, u, clamped_s0, span, bump);
                        index.update_net(route_idx, nr);
                        any_changed = true;
                        bumps_added += 1;
                        pref_side = -chosen_side;
                        placed = true;
                        break;
                    }
                    if clamped_s0 >= s_max - 1e-6 {
                        break;
                    }
                    s0 += step;
                }
                if placed {
                    break;
                }
            }
            if !placed {
                break;
            }
        }
    }

    any_changed
}

#[allow(clippy::too_many_arguments)]
fn find_max_clear_height(
    board: &Board,
    nr: &NetRoute,
    seg_idx: usize,
    seg: &RoutedSegment,
    u: Point,
    n: Point,
    s0: f64,
    c: f64,
    top_w: f64,
    h_min: f64,
    h_hi: f64,
    side: f64,
    clr: f64,
    index: &RouteSpatialIndex,
    qbuf: &mut Vec<usize>,
    sbuf: &mut Vec<usize>,
) -> Option<(f64, Vec<RoutedSegment>)> {
    let bump_hi = build_bump_segments(seg.a, u, n, seg.layer, seg.width, s0, c, top_w, h_hi, side);
    if bump_is_clear(board, nr, seg_idx, &bump_hi, clr, index, qbuf, sbuf) {
        return Some((h_hi, bump_hi));
    }
    let bump_lo = build_bump_segments(seg.a, u, n, seg.layer, seg.width, s0, c, top_w, h_min, side);
    if !bump_is_clear(board, nr, seg_idx, &bump_lo, clr, index, qbuf, sbuf) {
        return None;
    }

    let mut lo = h_min;
    let mut hi = h_hi;
    let mut best_bump = bump_lo;
    for _ in 0..12 {
        if hi - lo <= 5.0 {
            break;
        }
        let mid = 0.5 * (lo + hi);
        let cand = build_bump_segments(seg.a, u, n, seg.layer, seg.width, s0, c, top_w, mid, side);
        if bump_is_clear(board, nr, seg_idx, &cand, clr, index, qbuf, sbuf) {
            lo = mid;
            best_bump = cand;
        } else {
            hi = mid;
        }
    }
    Some((lo, best_bump))
}

#[allow(clippy::too_many_arguments)]
fn build_bump_segments(
    origin: Point,
    u: Point,
    n: Point,
    layer: usize,
    width: f64,
    s0: f64,
    c: f64,
    top_w: f64,
    h: f64,
    side: f64,
) -> Vec<RoutedSegment> {
    let pt = |s: f64, y: f64| {
        Point::new(
            origin.x + u.x * s + n.x * (side * y),
            origin.y + u.y * s + n.y * (side * y),
        )
    };
    let span = 4.0 * c + top_w;
    let pts: Vec<Point> = if h <= 2.0 * c + 1e-6 {
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
        .filter(|w| w[0].dist(w[1]) > 0.1)
        .map(|w| RoutedSegment {
            layer,
            a: w[0],
            b: w[1],
            width,
        })
        .collect()
}

#[allow(clippy::too_many_arguments)]
fn bump_is_clear(
    board: &Board,
    nr: &NetRoute,
    seg_idx: usize,
    bump: &[RoutedSegment],
    clr: f64,
    index: &RouteSpatialIndex,
    qbuf: &mut Vec<usize>,
    sbuf: &mut Vec<usize>,
) -> bool {
    let net_id = nr.net_id;
    let req_self = clr + 0.5;
    for cand in bump {
        if !board.check_segment_static_with_buf(
            net_id, cand.layer, cand.a, cand.b, cand.width, 0.5, sbuf,
        ) {
            return false;
        }
        if !index.clears_segment(board, net_id, cand, qbuf) {
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
        for fs in &board.fixed_segments {
            if fs.net_id != Some(net_id) || fs.layer != cand.layer {
                continue;
            }
            let gap = segment_to_segment_dist(cand.a, cand.b, fs.a, fs.b) - half_w - fs.half_w;
            if gap < req_self {
                return false;
            }
        }
        for fv in &board.fixed_vias {
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

fn apply_bump(
    nr: &mut NetRoute,
    seg_idx: usize,
    orig: &RoutedSegment,
    u: Point,
    s0: f64,
    span: f64,
    bump: Vec<RoutedSegment>,
) {
    let p_start = Point::new(orig.a.x + u.x * s0, orig.a.y + u.y * s0);
    let p_end = Point::new(orig.a.x + u.x * (s0 + span), orig.a.y + u.y * (s0 + span));
    nr.segments.swap_remove(seg_idx);
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

/// Identifies segments with an interior T-junction, pad, or via contact.
fn find_tapped_segments(board: &Board, nr: &NetRoute) -> Vec<bool> {
    let mut pts_by_layer: HashMap<usize, Vec<Point>> = HashMap::new();
    for &pid in &board.nets[nr.net_id].pad_ids {
        let p = &board.pads[pid];
        for &(l, _) in &p.layer_shapes {
            pts_by_layer.entry(l).or_default().push(p.center);
        }
    }
    for v in &nr.vias {
        for &l in &v.layers {
            pts_by_layer.entry(l).or_default().push(v.pos);
        }
    }
    for fv in &board.fixed_vias {
        if fv.net_id == Some(nr.net_id) {
            for &l in &fv.layers {
                pts_by_layer.entry(l).or_default().push(fv.pos);
            }
        }
    }
    for fs in &board.fixed_segments {
        if fs.net_id == Some(nr.net_id) {
            let list = pts_by_layer.entry(fs.layer).or_default();
            list.push(fs.a);
            list.push(fs.b);
        }
    }
    for s in &nr.segments {
        let list = pts_by_layer.entry(s.layer).or_default();
        list.push(s.a);
        list.push(s.b);
    }

    let mut tapped = vec![false; nr.segments.len()];
    for (i, s) in nr.segments.iter().enumerate() {
        let Some(pts) = pts_by_layer.get(&s.layer) else {
            continue;
        };
        let ka = pt_key(s.a);
        let kb = pt_key(s.b);
        let tol = s.width * 0.5 + 2.0;
        let min_x = s.a.x.min(s.b.x) - tol;
        let max_x = s.a.x.max(s.b.x) + tol;
        let min_y = s.a.y.min(s.b.y) - tol;
        let max_y = s.a.y.max(s.b.y) + tol;
        for &pt in pts {
            if pt.x < min_x || pt.x > max_x || pt.y < min_y || pt.y > max_y {
                continue;
            }
            let kp = pt_key(pt);
            if kp == ka || kp == kb {
                continue;
            }
            if point_to_segment_dist(pt, s.a, s.b) <= tol {
                tapped[i] = true;
                break;
            }
        }
    }
    tapped
}
