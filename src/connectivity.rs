//! Exact copper connectivity verification and routing quality scoring.

use std::collections::{BTreeMap, HashSet};

use crate::board::{Board, NetRoute};
use crate::drc::check_drc_scoped;
use crate::geometry::{point_to_segment_dist, segment_to_segment_dist};
use crate::length::{LengthConstraints, evaluate_lengths};
use crate::optimize::route_wirelength;
use crate::router::{Terminal, uf_find};

/// Comparable quality metric for choosing the best board across passes and multi-start seeds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RouteScore {
    pub unconnected_pairs: usize,
    pub drc_violations: usize,
    pub length_violations: usize,
    pub length_excess_um: f64,
    pub total_vias: usize,
    pub wirelength_um: f64,
}

impl RouteScore {
    pub const WORST: Self = Self {
        unconnected_pairs: usize::MAX,
        drc_violations: usize::MAX,
        length_violations: usize::MAX,
        length_excess_um: f64::INFINITY,
        total_vias: usize::MAX,
        wirelength_um: f64::INFINITY,
    };

    #[inline]
    pub fn is_perfect(self) -> bool {
        self.unconnected_pairs == 0 && self.drc_violations == 0 && self.length_violations == 0
    }

    /// Orders by unconnected pairs, DRC, length violations, length excess, vias, then wirelength.
    #[inline]
    pub fn is_better_than(self, other: Self) -> bool {
        if self.unconnected_pairs != other.unconnected_pairs {
            return self.unconnected_pairs < other.unconnected_pairs;
        }
        if self.drc_violations != other.drc_violations {
            return self.drc_violations < other.drc_violations;
        }
        if self.length_violations != other.length_violations {
            return self.length_violations < other.length_violations;
        }
        if (self.length_excess_um - other.length_excess_um).abs() > 0.1 {
            return self.length_excess_um < other.length_excess_um;
        }
        if self.total_vias != other.total_vias {
            return self.total_vias < other.total_vias;
        }
        self.wirelength_um + 1.0 < other.wirelength_um
    }
}

/// Evaluates a full board routing including optional max-length and match-group constraints.
pub fn score_routes(
    board: &Board,
    routes: &[NetRoute],
    constraints: &LengthConstraints,
) -> (RouteScore, crate::drc::DrcReport) {
    score_routes_scoped(board, routes, None, constraints)
}

/// Evaluates routing quality scoped to `target_mask` for unconnected pairs and DRC violations.
pub(crate) fn score_routes_scoped(
    board: &Board,
    routes: &[NetRoute],
    target_mask: Option<&[bool]>,
    constraints: &LengthConstraints,
) -> (RouteScore, crate::drc::DrcReport) {
    let drc = check_drc_scoped(board, routes, target_mask);
    let unconnected_pairs: usize = routes
        .iter()
        .filter(|r| target_mask.is_none_or(|m| m.get(r.net_id).copied().unwrap_or(false)))
        .map(|r| r.unconnected_pairs)
        .sum();
    let len_report = evaluate_lengths(board, routes, constraints);
    let length_violations = len_report.violations.len();
    let length_excess_um: f64 = len_report.violations.iter().map(|v| v.excess_um).sum();
    let total_vias: usize = routes.iter().map(|r| r.vias.len()).sum();
    let wirelength_um: f64 = routes.iter().map(route_wirelength).sum();
    let score = RouteScore {
        unconnected_pairs,
        drc_violations: drc.violations.len(),
        length_violations,
        length_excess_um,
        total_vias,
        wirelength_um,
    };
    (score, drc)
}

/// Groups a net's initial physical components into connected clusters given `nr`'s routed wiring.
pub(crate) fn connected_component_clusters(
    board: &Board,
    components: &[Vec<Terminal>],
    nr: &NetRoute,
) -> Vec<Vec<usize>> {
    let k = components.len();
    if k == 0 {
        return Vec::new();
    }
    if k == 1 {
        return vec![vec![0]];
    }
    let ns = nr.segments.len();
    let nv = nr.vias.len();
    let mut uf: Vec<usize> = (0..k + ns + nv).collect();

    let seg_base = k;
    let via_base = k + ns;

    // 1. Segment-to-segment electrical contact on the same layer (handles T-junctions).
    for (i, sa) in nr.segments.iter().enumerate() {
        let ax0 = sa.a.x.min(sa.b.x);
        let ax1 = sa.a.x.max(sa.b.x);
        let ay0 = sa.a.y.min(sa.b.y);
        let ay1 = sa.a.y.max(sa.b.y);
        for (j, sb) in nr.segments.iter().enumerate().skip(i + 1) {
            if sa.layer != sb.layer {
                continue;
            }
            let tol = (sa.width + sb.width) * 0.5 + 2.0;
            if sb.a.x.max(sb.b.x) + tol < ax0
                || sb.a.x.min(sb.b.x) - tol > ax1
                || sb.a.y.max(sb.b.y) + tol < ay0
                || sb.a.y.min(sb.b.y) - tol > ay1
            {
                continue;
            }
            if segment_to_segment_dist(sa.a, sa.b, sb.a, sb.b) <= tol {
                let ra = uf_find(&mut uf, seg_base + i);
                let rb = uf_find(&mut uf, seg_base + j);
                uf[rb] = ra;
            }
        }
    }

    // 2. Segment-to-via electrical contact.
    for (i, s) in nr.segments.iter().enumerate() {
        for (j, v) in nr.vias.iter().enumerate() {
            if !v.layers.contains(&s.layer) {
                continue;
            }
            let tol = v.radius + s.width * 0.5 + 2.0;
            if point_to_segment_dist(v.pos, s.a, s.b) <= tol {
                let ra = uf_find(&mut uf, seg_base + i);
                let rb = uf_find(&mut uf, via_base + j);
                uf[rb] = ra;
            }
        }
    }

    // 3. Via-to-via electrical contact.
    for (i, va) in nr.vias.iter().enumerate() {
        for (j, vb) in nr.vias.iter().enumerate().skip(i + 1) {
            if va.layers.iter().any(|l| vb.layers.contains(l))
                && va.pos.dist(vb.pos) <= va.radius + vb.radius + 2.0
            {
                let ra = uf_find(&mut uf, via_base + i);
                let rb = uf_find(&mut uf, via_base + j);
                uf[rb] = ra;
            }
        }
    }

    // 4. Component-to-segment and component-to-via electrical contact.
    for (ci, comp) in components.iter().enumerate() {
        for t in comp {
            for (i, s) in nr.segments.iter().enumerate() {
                let touches = if let Some(pid) = t.pad_id {
                    board.pads[pid]
                        .shape_on_layer(s.layer)
                        .is_some_and(|sh| sh.dist_to_capsule(s.a, s.b, s.width * 0.5) <= 2.0)
                } else {
                    t.layers.contains(&s.layer)
                        && point_to_segment_dist(t.exact, s.a, s.b) <= s.width * 0.5 + 15.0
                };
                if touches {
                    let ra = uf_find(&mut uf, ci);
                    let rb = uf_find(&mut uf, seg_base + i);
                    uf[rb] = ra;
                }
            }
            for (j, v) in nr.vias.iter().enumerate() {
                let touches = if let Some(pid) = t.pad_id {
                    v.layers.iter().any(|&l| {
                        board.pads[pid]
                            .shape_on_layer(l)
                            .is_some_and(|sh| sh.dist_to_circle(v.pos, v.radius) <= 2.0)
                    })
                } else {
                    v.layers.iter().any(|l| t.layers.contains(l))
                        && t.exact.dist(v.pos) <= v.radius + 15.0
                };
                if touches {
                    let ra = uf_find(&mut uf, ci);
                    let rb = uf_find(&mut uf, via_base + j);
                    uf[rb] = ra;
                }
            }
        }
    }

    let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for ci in 0..k {
        let root = uf_find(&mut uf, ci);
        groups.entry(root).or_default().push(ci);
    }
    groups.into_values().collect()
}

/// Counts how many MST pairs remain disconnected for a net given its current routed copper.
pub(crate) fn count_unconnected_pairs(
    board: &Board,
    components: &[Vec<Terminal>],
    nr: &NetRoute,
) -> usize {
    let clusters = connected_component_clusters(board, components, nr);
    clusters.len().saturating_sub(1)
}

/// Returns the set of component indices already joined together (for verifying post-opt integrity).
pub(crate) fn count_all_unconnected(
    board: &Board,
    net_components: &[Vec<Vec<Terminal>>],
    routes: &mut [NetRoute],
    routable_nets: &[usize],
) -> usize {
    let mut total = 0;
    let routable: HashSet<usize> = routable_nets.iter().copied().collect();
    for nr in routes.iter_mut() {
        if !routable.contains(&nr.net_id) {
            nr.unconnected_pairs = 0;
            continue;
        }
        nr.unconnected_pairs = count_unconnected_pairs(board, &net_components[nr.net_id], nr);
        total += nr.unconnected_pairs;
    }
    total
}
