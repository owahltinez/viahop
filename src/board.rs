//! Resolved physical board representation, spatial index, and exact continuous DRC.

use std::collections::{HashMap, HashSet};

use crate::dsn::{DsnDesign, KeepoutKind, Rules, Side, WireType};
use crate::geometry::{
    BBox, Point, Shape, point_in_polygon, point_to_polygon_edge_dist, segment_to_polygon_edge_dist,
    segment_to_segment_dist, transform_point,
};

/// Optional board-level clearance overrides (units: micrometers).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct BoardOptions {
    pub hole_to_hole_um: Option<f64>,
    pub edge_clearance_um: Option<f64>,
}

impl BoardOptions {
    pub const DEFAULT_HOLE_TO_HOLE_UM: f64 = 200.0;
}

/// Resolved design rules for a specific net (all dimensions in micrometers).
#[derive(Debug, Clone)]
pub struct NetRules {
    pub width: f64,
    pub clearance: f64,
    pub smd_smd: f64,
    pub via_wire: f64,
    pub via_via: f64,
    pub via_smd: f64,
    pub via_padstack: String,
    pub via_radius: f64,
    pub via_drill: f64,
    /// Layers spanned by this net's via padstack.
    pub via_layers: Vec<usize>,
}

/// A resolved physical pad on the board.
#[derive(Debug, Clone)]
pub struct Pad {
    pub id: usize,
    pub refdes: String,
    pub pin_name: String,
    pub center: Point,
    pub net_id: Option<usize>,
    /// Per-layer exact copper shapes `(layer_idx, Shape)`.
    pub layer_shapes: Vec<(usize, Shape)>,
    pub bbox: BBox,
    pub is_smd: bool,
    pub is_through_hole: bool,
    pub max_radius: f64,
}

impl Pad {
    pub fn on_layer(&self, layer: usize) -> bool {
        self.layer_shapes.iter().any(|(l, _)| *l == layer)
    }

    pub fn shape_on_layer(&self, layer: usize) -> Option<&Shape> {
        self.layer_shapes
            .iter()
            .find(|(l, _)| *l == layer)
            .map(|(_, s)| s)
    }
}

#[derive(Debug, Clone)]
pub struct ResolvedKeepout {
    pub kind: KeepoutKind,
    /// True when defined in `(structure ...)` (e.g., board cutout/slot), false for image keepouts.
    pub is_structure: bool,
    /// Empty means all layers; otherwise specific layer indices.
    pub layers: Vec<usize>,
    pub shape: Shape,
    pub bbox: BBox,
}

impl ResolvedKeepout {
    pub fn applies_to_layer(&self, layer: usize) -> bool {
        self.layers.is_empty() || self.layers.contains(&layer)
    }
}

#[derive(Debug, Clone)]
pub struct FixedSegment {
    pub layer: usize,
    pub a: Point,
    pub b: Point,
    pub half_w: f64,
    pub net_id: Option<usize>,
    pub bbox: BBox,
    pub is_synthetic: bool,
    pub wire_group: Option<usize>,
}

/// Exact static obstacle identified when a continuous DRC check fails.
#[derive(Debug, Clone)]
pub struct StaticBlocker {
    pub label: String,
    pub net_b: Option<usize>,
    pub gap_um: f64,
    pub required_um: f64,
}

#[derive(Clone, Copy)]
enum StaticObstacle {
    Boundary,
    Keepout(bool),
    Pad(usize),
    FixedVia(usize),
    FixedWire(usize),
    FixedPlane(usize),
}

#[derive(Clone, Copy)]
struct StaticHit {
    obstacle: StaticObstacle,
    deficit: f64,
    gap_um: f64,
    required_um: f64,
}

#[derive(Debug, Clone)]
pub struct FixedVia {
    pub pos: Point,
    pub radius: f64,
    pub drill: f64,
    pub layers: Vec<usize>,
    pub net_id: Option<usize>,
    pub bbox: BBox,
}

#[derive(Debug, Clone)]
pub struct FixedPlane {
    pub layer: usize,
    pub net_id: Option<usize>,
    pub shape: Shape,
    pub bbox: BBox,
}

#[derive(Debug, Clone)]
pub struct NetInfo {
    pub id: usize,
    pub name: String,
    pub class_name: String,
    pub pad_ids: Vec<usize>,
    pub rules: NetRules,
    pub ignored: bool,
}

/// A routed trace segment produced by the router.
#[derive(Debug, Clone)]
pub struct RoutedSegment {
    pub layer: usize,
    pub a: Point,
    pub b: Point,
    pub width: f64,
}

/// A routed via produced by the router.
#[derive(Debug, Clone)]
pub struct RoutedVia {
    pub pos: Point,
    pub padstack: String,
    pub radius: f64,
    pub drill: f64,
    pub layers: Vec<usize>,
}

/// Complete routed result for a single net.
#[derive(Debug, Clone, Default)]
pub struct NetRoute {
    pub net_id: usize,
    pub segments: Vec<RoutedSegment>,
    pub vias: Vec<RoutedVia>,
    pub unconnected_pairs: usize,
}

/// Uniform spatial hash grid for O(1) bounding-box queries over static obstacles.
#[derive(Debug, Clone)]
pub struct SpatialGrid {
    cell_size: f64,
    origin_x: f64,
    origin_y: f64,
    cols: usize,
    rows: usize,
    buckets: Vec<Vec<usize>>,
}

impl SpatialGrid {
    pub fn new(bounds: BBox, cell_size: f64) -> Self {
        let margin = cell_size * 2.0;
        let origin_x = bounds.min_x - margin;
        let origin_y = bounds.min_y - margin;
        let cols =
            (((bounds.max_x - bounds.min_x + 2.0 * margin) / cell_size).ceil() as usize).max(1);
        let rows =
            (((bounds.max_y - bounds.min_y + 2.0 * margin) / cell_size).ceil() as usize).max(1);
        Self {
            cell_size,
            origin_x,
            origin_y,
            cols,
            rows,
            buckets: vec![Vec::new(); cols * rows],
        }
    }

    pub fn insert(&mut self, id: usize, bbox: BBox) {
        let (x0, y0, x1, y1) = self.cell_range(bbox);
        for cy in y0..=y1 {
            let row = cy * self.cols;
            for cx in x0..=x1 {
                self.buckets[row + cx].push(id);
            }
        }
    }

    pub fn query(&self, bbox: BBox, out: &mut Vec<usize>) {
        out.clear();
        let (x0, y0, x1, y1) = self.cell_range(bbox);
        if x0 == x1 && y0 == y1 {
            out.extend_from_slice(&self.buckets[y0 * self.cols + x0]);
            return;
        }
        for cy in y0..=y1 {
            let row = cy * self.cols;
            for cx in x0..=x1 {
                out.extend_from_slice(&self.buckets[row + cx]);
            }
        }
        out.sort_unstable();
        out.dedup();
    }

    fn cell_range(&self, bbox: BBox) -> (usize, usize, usize, usize) {
        let gx = |x: f64| -> usize {
            let idx = ((x - self.origin_x) / self.cell_size).floor() as isize;
            idx.clamp(0, (self.cols - 1) as isize) as usize
        };
        let gy = |y: f64| -> usize {
            let idx = ((y - self.origin_y) / self.cell_size).floor() as isize;
            idx.clamp(0, (self.rows - 1) as isize) as usize
        };
        (
            gx(bbox.min_x),
            gy(bbox.min_y),
            gx(bbox.max_x),
            gy(bbox.max_y),
        )
    }
}

/// Complete resolved board used by fanout, router, optimizer, and DRC.
#[derive(Debug, Clone)]
pub struct Board {
    pub layer_names: Vec<String>,
    pub signal_layers: Vec<usize>,
    pub layer_name_to_idx: HashMap<String, usize>,
    pub boundary: Vec<Point>,
    pub bounds: BBox,
    pub pads: Vec<Pad>,
    pub keepouts: Vec<ResolvedKeepout>,
    pub fixed_segments: Vec<FixedSegment>,
    pub fixed_vias: Vec<FixedVia>,
    pub planes: Vec<FixedPlane>,
    pub nets: Vec<NetInfo>,
    pub net_name_to_id: HashMap<String, usize>,
    pub via_at_smd: bool,
    pub hole_to_hole_um: f64,
    pub edge_clearance_um: Option<f64>,
    pub max_net_clearance: f64,
    pub pad_index: SpatialGrid,
    pub keepout_index: SpatialGrid,
    pub fixed_seg_index: SpatialGrid,
    pub fixed_via_index: SpatialGrid,
    pub fixed_plane_index: SpatialGrid,
}

impl Board {
    pub fn from_dsn(
        design: &DsnDesign,
        ignored: &HashSet<String>,
        force_via_at_smd: Option<bool>,
    ) -> Result<Self, String> {
        Self::from_dsn_with_options(design, ignored, force_via_at_smd, BoardOptions::default())
    }

    pub fn from_dsn_with_options(
        design: &DsnDesign,
        ignored: &HashSet<String>,
        force_via_at_smd: Option<bool>,
        options: BoardOptions,
    ) -> Result<Self, String> {
        let hole_to_hole_um = options
            .hole_to_hole_um
            .or(design.default_rules.hole_to_hole)
            .unwrap_or(BoardOptions::DEFAULT_HOLE_TO_HOLE_UM);
        let edge_clearance_um = options
            .edge_clearance_um
            .or(design.default_rules.edge_clearance);
        if !hole_to_hole_um.is_finite() || hole_to_hole_um < 0.0 {
            return Err(format!(
                "hole_to_hole_um must be a non-negative finite number, got {hole_to_hole_um}"
            ));
        }
        if let Some(ec) = edge_clearance_um
            && (!ec.is_finite() || ec < 0.0)
        {
            return Err(format!(
                "edge_clearance_um must be a non-negative finite number, got {ec}"
            ));
        }
        if design.layers.is_empty() {
            return Err("DSN contains no copper layers".to_string());
        }
        let num_layers = design.layers.len();
        let layer_names: Vec<String> = design.layers.iter().map(|l| l.name.clone()).collect();
        let mut layer_name_to_idx = HashMap::new();
        let mut signal_layers = Vec::new();
        for (i, l) in design.layers.iter().enumerate() {
            layer_name_to_idx.insert(l.name.clone(), i);
            let has_inner_plane =
                i > 0 && i + 1 < num_layers && design.planes.iter().any(|p| p.layer == l.name);
            if l.is_signal && !has_inner_plane {
                signal_layers.push(i);
            }
        }
        if signal_layers.is_empty() {
            signal_layers.extend(0..num_layers);
        }

        // Net name -> id and net -> class lookup.
        let mut net_name_to_id = HashMap::new();
        for (i, n) in design.nets.iter().enumerate() {
            net_name_to_id.insert(n.name.clone(), i);
        }
        let mut net_to_class: HashMap<&str, &crate::dsn::NetClass> = HashMap::new();
        for cls in &design.classes {
            for net_name in &cls.nets {
                net_to_class.insert(net_name.as_str(), cls);
            }
        }

        // Map (refdes, pin) -> net_id.
        let mut pin_to_net: HashMap<(String, String), usize> = HashMap::new();
        for (net_id, net) in design.nets.iter().enumerate() {
            for (refdes, pin) in &net.pins {
                pin_to_net.insert((refdes.clone(), pin.clone()), net_id);
            }
        }

        // Resolve placements -> pads & image keepouts.
        let mut pads = Vec::new();
        let mut keepouts = Vec::new();
        let mut fixed_segments = Vec::new();
        let mut net_pad_ids: Vec<Vec<usize>> = vec![Vec::new(); design.nets.len()];

        for k in &design.keepouts {
            let layers = resolve_layer_spec(&k.layer, &layer_name_to_idx);
            keepouts.push(ResolvedKeepout {
                kind: k.kind,
                is_structure: true,
                layers,
                bbox: k.shape.bbox(),
                shape: k.shape.clone(),
            });
        }

        for place in &design.placements {
            let Some(img) = design.images.get(&place.component) else {
                continue;
            };
            let flip_x = place.side == Side::Back;

            for k in &img.keepouts {
                let mut layers = resolve_layer_spec(&k.layer, &layer_name_to_idx);
                if flip_x {
                    for l in &mut layers {
                        *l = num_layers - 1 - *l;
                    }
                }
                let shape = k.shape.transform(flip_x, place.rotation_deg, place.pos);
                keepouts.push(ResolvedKeepout {
                    kind: k.kind,
                    is_structure: false,
                    layers,
                    bbox: shape.bbox(),
                    shape,
                });
            }

            for pin in &img.pins {
                let Some(ps) = design.padstacks.get(&pin.padstack) else {
                    continue;
                };
                let center = transform_point(pin.pos, flip_x, place.rotation_deg, place.pos);
                let mut layer_shapes = Vec::new();
                let mut bbox: Option<BBox> = None;

                for ps_shape in &ps.shapes {
                    let mut layers = resolve_layer_spec(&ps_shape.layer, &layer_name_to_idx);
                    if layers.is_empty() {
                        layers.extend(0..num_layers);
                    }
                    // Apply pin-local rotation, translate to pin.pos, then component placement.
                    let local = if pin.rotation_deg.abs() > 1e-9 {
                        ps_shape.shape.transform(false, pin.rotation_deg, pin.pos)
                    } else {
                        ps_shape.shape.transform(false, 0.0, pin.pos)
                    };
                    let world = local.transform(flip_x, place.rotation_deg, place.pos);
                    let sb = world.bbox();
                    bbox = Some(match bbox {
                        Some(b) => b.union(sb),
                        None => sb,
                    });
                    for mut l in layers {
                        if flip_x {
                            l = num_layers - 1 - l;
                        }
                        layer_shapes.push((l, world.clone()));
                    }
                }
                if layer_shapes.is_empty() {
                    continue;
                }
                let unique_layers: HashSet<usize> = layer_shapes.iter().map(|(l, _)| *l).collect();
                let is_smd = unique_layers.len() == 1;
                let is_through_hole = unique_layers.len() >= num_layers.min(2);
                let bbox = bbox.unwrap();
                let max_radius = ((bbox.max_x - bbox.min_x).hypot(bbox.max_y - bbox.min_y)) * 0.5;
                let net_id = pin_to_net
                    .get(&(place.refdes.clone(), pin.pin_name.clone()))
                    .copied();
                let id = pads.len();
                if let Some(nid) = net_id {
                    net_pad_ids[nid].push(id);
                }
                pads.push(Pad {
                    id,
                    refdes: place.refdes.clone(),
                    pin_name: pin.pin_name.clone(),
                    center,
                    net_id,
                    layer_shapes,
                    bbox,
                    is_smd,
                    is_through_hole,
                    max_radius,
                });
            }

            // KiCad NetTie footprints contain an internal copper bridge polygon between pads
            // that is omitted from DSN exports; add a no-net fixed segment at the bridge midpoint
            // so third-party traces and vias honor full copper clearance while the pads stay open.
            let is_net_tie = place.component.to_ascii_lowercase().contains("nettie")
                || place
                    .part_number
                    .as_deref()
                    .is_some_and(|pn| pn.to_ascii_lowercase().contains("nettie"));
            if is_net_tie && img.pins.len() >= 2 {
                let comp_pads: Vec<&Pad> = pads
                    .iter()
                    .rev()
                    .take(img.pins.len())
                    .filter(|p| p.refdes == place.refdes)
                    .collect();
                for pair in comp_pads.windows(2) {
                    let (p1, p2) = (pair[0], pair[1]);
                    if let Some(&(layer, _)) = p1.layer_shapes.first() {
                        let a = Point::new(
                            p1.center.x + (p2.center.x - p1.center.x) * 0.42,
                            p1.center.y + (p2.center.y - p1.center.y) * 0.42,
                        );
                        let b = Point::new(
                            p1.center.x + (p2.center.x - p1.center.x) * 0.58,
                            p1.center.y + (p2.center.y - p1.center.y) * 0.58,
                        );
                        let pad_r = |p: &Pad| {
                            ((p.bbox.max_x - p.bbox.min_x).min(p.bbox.max_y - p.bbox.min_y)) * 0.5
                        };
                        let half_w = pad_r(p1).min(pad_r(p2)).max(120.0);
                        fixed_segments.push(FixedSegment {
                            layer,
                            a,
                            b,
                            half_w,
                            net_id: None,
                            bbox: BBox::from_points(&[a, b], half_w),
                            is_synthetic: true,
                            wire_group: None,
                        });
                    }
                }
            }
        }

        // Resolve fixed/protected existing wires and vias.
        for (wire_idx, w) in design.existing_wires.iter().enumerate() {
            if w.wire_type == WireType::Route {
                continue;
            }
            let Some(&layer) = layer_name_to_idx.get(&w.layer) else {
                continue;
            };
            let net_id = net_name_to_id.get(&w.net).copied();
            let half_w = w.width * 0.5;
            for seg in w.points.windows(2) {
                fixed_segments.push(FixedSegment {
                    layer,
                    a: seg[0],
                    b: seg[1],
                    half_w,
                    net_id,
                    bbox: BBox::from_points(&[seg[0], seg[1]], half_w),
                    is_synthetic: false,
                    wire_group: Some(wire_idx),
                });
            }
        }

        let mut fixed_vias = Vec::new();
        for v in &design.existing_vias {
            if v.wire_type == WireType::Route {
                continue;
            }
            let (radius, drill, layers) =
                resolve_via_padstack(design, &v.padstack, &layer_name_to_idx, num_layers);
            let net_id = net_name_to_id.get(&v.net).copied();
            fixed_vias.push(FixedVia {
                pos: v.pos,
                radius,
                drill,
                layers,
                net_id,
                bbox: BBox::from_point(v.pos, radius),
            });
        }

        // Resolve planes (signal-layer planes are indexed as routing obstacles; inner planes aid via connectivity).
        let mut planes = Vec::new();
        for pl in &design.planes {
            let Some(&layer) = layer_name_to_idx.get(&pl.layer) else {
                continue;
            };
            planes.push(FixedPlane {
                layer,
                net_id: net_name_to_id.get(&pl.net).copied(),
                bbox: pl.shape.bbox(),
                shape: pl.shape.clone(),
            });
        }

        // Resolve net rules and ignored sets.
        let mut nets = Vec::with_capacity(design.nets.len());
        let default_via = design
            .structure_vias
            .first()
            .cloned()
            .unwrap_or_else(|| "Via[0-1]_500:300_um".to_string());

        for (id, dnet) in design.nets.iter().enumerate() {
            let cls = net_to_class.get(dnet.name.as_str()).copied();
            let class_name = cls
                .map(|c| c.name.clone())
                .unwrap_or_else(|| "default".to_string());
            let rules =
                resolve_net_rules(design, cls, &default_via, &layer_name_to_idx, num_layers);
            let is_ignored = ignored.contains(&dnet.name) || ignored.contains(&class_name);
            nets.push(NetInfo {
                id,
                name: dnet.name.clone(),
                class_name,
                pad_ids: std::mem::take(&mut net_pad_ids[id]),
                rules,
                ignored: is_ignored,
            });
        }

        // Determine board boundary (largest outer boundary polygon).
        let boundary = design
            .boundaries
            .iter()
            .max_by(|a, b| {
                polygon_area(a)
                    .partial_cmp(&polygon_area(b))
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .cloned()
            .unwrap_or_default();

        // Compute overall bounding box.
        let mut bounds = if !boundary.is_empty() {
            BBox::from_points(&boundary, 0.0)
        } else if !pads.is_empty() {
            let mut b = pads[0].bbox;
            for p in &pads[1..] {
                b = b.union(p.bbox);
            }
            b.expand(2000.0)
        } else {
            BBox::new(0.0, 0.0, 10000.0, 10000.0)
        };
        for p in &pads {
            bounds = bounds.union(p.bbox);
        }

        // Build spatial indices (1 mm buckets give crisp O(1) neighborhood lookups).
        let cell_size = 1000.0;
        let mut pad_index = SpatialGrid::new(bounds, cell_size);
        for p in &pads {
            pad_index.insert(p.id, p.bbox);
        }
        let mut keepout_index = SpatialGrid::new(bounds, cell_size);
        for (i, k) in keepouts.iter().enumerate() {
            keepout_index.insert(i, k.bbox);
        }
        let mut fixed_seg_index = SpatialGrid::new(bounds, cell_size);
        for (i, s) in fixed_segments.iter().enumerate() {
            fixed_seg_index.insert(i, s.bbox);
        }
        let mut fixed_via_index = SpatialGrid::new(bounds, cell_size);
        for (i, v) in fixed_vias.iter().enumerate() {
            fixed_via_index.insert(i, v.bbox);
        }
        let mut fixed_plane_index = SpatialGrid::new(bounds, cell_size);
        for (i, pl) in planes.iter().enumerate() {
            if signal_layers.contains(&pl.layer) {
                fixed_plane_index.insert(i, pl.bbox);
            }
        }

        let via_at_smd = force_via_at_smd.unwrap_or(design.via_at_smd);
        let max_net_clearance = nets.iter().fold(100.0_f64, |acc, n| {
            let r = &n.rules;
            acc.max(r.clearance)
                .max(r.via_wire)
                .max(r.via_via)
                .max(r.via_smd)
        });

        Ok(Self {
            layer_names,
            signal_layers,
            layer_name_to_idx,
            boundary,
            bounds,
            pads,
            keepouts,
            fixed_segments,
            fixed_vias,
            planes,
            nets,
            net_name_to_id,
            via_at_smd,
            hole_to_hole_um,
            edge_clearance_um,
            max_net_clearance,
            pad_index,
            keepout_index,
            fixed_seg_index,
            fixed_via_index,
            fixed_plane_index,
        })
    }

    pub fn num_layers(&self) -> usize {
        self.layer_names.len()
    }

    /// Minimum center-to-center spacing between vias for `net_id` (copper + drill hole-to-hole).
    pub fn min_via_spacing(&self, net_id: usize) -> f64 {
        let r = &self.nets[net_id].rules;
        (2.0 * r.via_radius + r.via_via).max(r.via_drill + self.hole_to_hole_um + 5.0)
    }

    pub fn is_inside_boundary(&self, p: Point, margin: f64) -> bool {
        if self.boundary.len() < 3 {
            return true;
        }
        if !point_in_polygon(p, &self.boundary) {
            return false;
        }
        if margin <= 0.0 {
            return true;
        }
        point_to_polygon_edge_dist(p, &self.boundary) >= margin - 1e-4
    }

    /// Checks whether a via for `net_id` at `pos` clears all static obstacles.
    pub fn check_via_static(&self, net_id: usize, pos: Point, margin_extra: f64) -> bool {
        let mut buf = Vec::new();
        self.check_via_static_with_buf(net_id, pos, margin_extra, &mut buf)
    }

    /// Allocation-free variant of `check_via_static` reusing `buf` across spatial queries.
    pub fn check_via_static_with_buf(
        &self,
        net_id: usize,
        pos: Point,
        margin_extra: f64,
        buf: &mut Vec<usize>,
    ) -> bool {
        let rules = &self.nets[net_id].rules;
        self.check_routed_via_static_with_buf(
            net_id,
            pos,
            rules.via_radius,
            rules.via_drill,
            &rules.via_layers,
            margin_extra,
            buf,
        )
    }

    /// Checks a via with explicit padstack geometry (`radius`, `drill`, `layers`) against static obstacles.
    #[allow(clippy::too_many_arguments)]
    pub fn check_routed_via_static_with_buf(
        &self,
        net_id: usize,
        pos: Point,
        radius: f64,
        drill: f64,
        layers: &[usize],
        margin_extra: f64,
        buf: &mut Vec<usize>,
    ) -> bool {
        self.scan_via_static::<true>(
            Some(net_id),
            pos,
            radius,
            drill,
            layers,
            margin_extra,
            None,
            true,
            buf,
        )
        .is_none()
    }

    /// Checks whether a trace segment on `layer` for `net_id` clears all static obstacles.
    pub fn check_segment_static(
        &self,
        net_id: usize,
        layer: usize,
        a: Point,
        b: Point,
        width: f64,
        margin_extra: f64,
    ) -> bool {
        let mut buf = Vec::new();
        self.check_segment_static_with_buf(net_id, layer, a, b, width, margin_extra, &mut buf)
    }

    /// Allocation-free variant of `check_segment_static` reusing `buf` across spatial queries.
    #[allow(clippy::too_many_arguments)]
    pub fn check_segment_static_with_buf(
        &self,
        net_id: usize,
        layer: usize,
        a: Point,
        b: Point,
        width: f64,
        margin_extra: f64,
        buf: &mut Vec<usize>,
    ) -> bool {
        self.scan_segment_static::<true>(Some(net_id), layer, a, b, width, margin_extra, None, buf)
            .is_none()
    }

    /// Returns the worst static obstacle blocking a via for `net_id` at `pos`, if any.
    pub fn diagnose_via_static(
        &self,
        net_id: usize,
        pos: Point,
        margin_extra: f64,
    ) -> Option<StaticBlocker> {
        let mut buf = Vec::new();
        self.diagnose_via_static_with_buf(net_id, pos, margin_extra, &mut buf)
    }

    /// Buffer-reusing variant of `diagnose_via_static` (zero allocation when clean).
    pub fn diagnose_via_static_with_buf(
        &self,
        net_id: usize,
        pos: Point,
        margin_extra: f64,
        buf: &mut Vec<usize>,
    ) -> Option<StaticBlocker> {
        let rules = &self.nets[net_id].rules;
        self.diagnose_routed_via_static_with_buf(
            net_id,
            pos,
            rules.via_radius,
            rules.via_drill,
            &rules.via_layers,
            margin_extra,
            buf,
        )
    }

    /// Diagnoses static obstacles blocking a via with explicit padstack geometry.
    #[allow(clippy::too_many_arguments)]
    pub fn diagnose_routed_via_static_with_buf(
        &self,
        net_id: usize,
        pos: Point,
        radius: f64,
        drill: f64,
        layers: &[usize],
        margin_extra: f64,
        buf: &mut Vec<usize>,
    ) -> Option<StaticBlocker> {
        self.scan_via_static::<false>(
            Some(net_id),
            pos,
            radius,
            drill,
            layers,
            margin_extra,
            None,
            true,
            buf,
        )
        .map(|h| self.format_hit(h))
    }

    /// Returns the worst static obstacle blocking a segment on `layer` for `net_id`, if any.
    pub fn diagnose_segment_static(
        &self,
        net_id: usize,
        layer: usize,
        a: Point,
        b: Point,
        width: f64,
        margin_extra: f64,
    ) -> Option<StaticBlocker> {
        let mut buf = Vec::new();
        self.diagnose_segment_static_with_buf(net_id, layer, a, b, width, margin_extra, &mut buf)
    }

    /// Buffer-reusing variant of `diagnose_segment_static` (zero allocation when clean).
    #[allow(clippy::too_many_arguments)]
    pub fn diagnose_segment_static_with_buf(
        &self,
        net_id: usize,
        layer: usize,
        a: Point,
        b: Point,
        width: f64,
        margin_extra: f64,
        buf: &mut Vec<usize>,
    ) -> Option<StaticBlocker> {
        self.scan_segment_static::<false>(Some(net_id), layer, a, b, width, margin_extra, None, buf)
            .map(|h| self.format_hit(h))
    }

    /// Diagnoses static obstacles colliding with pre-existing `fixed_vias[vid]`.
    pub(crate) fn diagnose_fixed_via_static(
        &self,
        vid: usize,
        margin_extra: f64,
        buf: &mut Vec<usize>,
    ) -> Option<StaticBlocker> {
        let fv = &self.fixed_vias[vid];
        self.scan_via_static::<false>(
            fv.net_id,
            fv.pos,
            fv.radius,
            fv.drill,
            &fv.layers,
            margin_extra,
            Some(vid),
            false,
            buf,
        )
        .map(|h| self.format_hit(h))
    }

    /// Diagnoses static obstacles colliding with pre-existing `fixed_segments[sid]`.
    pub(crate) fn diagnose_fixed_segment_static(
        &self,
        sid: usize,
        margin_extra: f64,
        buf: &mut Vec<usize>,
    ) -> Option<StaticBlocker> {
        let fs = &self.fixed_segments[sid];
        self.scan_segment_static::<false>(
            fs.net_id,
            fs.layer,
            fs.a,
            fs.b,
            fs.half_w * 2.0,
            margin_extra,
            Some(sid),
            buf,
        )
        .map(|h| self.format_hit(h))
    }

    #[allow(clippy::too_many_arguments)]
    fn scan_via_static<const FIRST_ONLY: bool>(
        &self,
        net_id: Option<usize>,
        pos: Point,
        r: f64,
        drill: f64,
        via_layers: &[usize],
        margin_extra: f64,
        skip_fixed_via: Option<usize>,
        check_fixed_segs: bool,
        buf: &mut Vec<usize>,
    ) -> Option<StaticHit> {
        let extra_pos = margin_extra.max(0.0);
        let edge_clr = self.edge_clearance_um.unwrap_or(0.0);
        if self.boundary.len() >= 3 {
            let inside = point_in_polygon(pos, &self.boundary);
            if !inside {
                let gap = if FIRST_ONLY {
                    0.0
                } else {
                    -point_to_polygon_edge_dist(pos, &self.boundary) - r
                };
                return Some(StaticHit {
                    obstacle: StaticObstacle::Boundary,
                    deficit: edge_clr + extra_pos - gap,
                    gap_um: gap,
                    required_um: edge_clr + extra_pos,
                });
            }
            let req = r + edge_clr + margin_extra;
            if req > 0.0 {
                let edge_d = point_to_polygon_edge_dist(pos, &self.boundary);
                if edge_d < req - 1e-4 {
                    return Some(StaticHit {
                        obstacle: StaticObstacle::Boundary,
                        deficit: req - edge_d,
                        gap_um: edge_d - r,
                        required_um: edge_clr + extra_pos,
                    });
                }
            }
        }

        let is_fixed = skip_fixed_via.is_some();
        let hole_min = self.hole_to_hole_um;
        let max_clr = self.max_net_clearance + extra_pos;
        let qbox = BBox::from_point(pos, r + max_clr + 50.0);
        let cutout_margin = self.edge_clearance_um.unwrap_or(145.0).max(145.0);
        let mut worst: Option<StaticHit> = None;

        // Keepouts (structure cutouts honor edge clearance for newly routed copper only).
        self.keepout_index
            .query(qbox.expand(cutout_margin + 15.0), buf);
        for &kid in buf.iter() {
            let k = &self.keepouts[kid];
            if k.kind == KeepoutKind::Wire {
                continue;
            }
            if !via_layers.iter().any(|&l| k.applies_to_layer(l)) {
                continue;
            }
            let k_margin = if is_fixed {
                0.0
            } else if k.is_structure && k.kind == KeepoutKind::All {
                cutout_margin
            } else {
                5.0
            };
            let gap = k.shape.dist_to_circle(pos, r);
            let req = k_margin + margin_extra;
            if gap < req - 1e-4 {
                let hit = StaticHit {
                    obstacle: StaticObstacle::Keepout(k.is_structure),
                    deficit: req - gap,
                    gap_um: gap,
                    required_um: k_margin + extra_pos,
                };
                if FIRST_ONLY {
                    return Some(hit);
                }
                record_hit(&mut worst, hit);
            }
        }

        // Other-net pads.
        self.pad_index.query(qbox, buf);
        for &pid in buf.iter() {
            let pad = &self.pads[pid];
            if same_net(net_id, pad.net_id) {
                continue;
            }
            let base_req = self.pair_clearance(net_id, pad.net_id, |nr| {
                if pad.is_smd { nr.via_smd } else { nr.via_via }
            });
            let req = base_req + margin_extra;
            for &(l, ref sh) in &pad.layer_shapes {
                if !via_layers.contains(&l) {
                    continue;
                }
                if is_fixed
                    && net_id.is_none()
                    && pad.net_id.is_none()
                    && sh.dist_to_circle(pos, 0.0) <= 1.0
                {
                    continue;
                }
                let gap = sh.dist_to_circle(pos, r);
                if gap < req - 1e-4 {
                    let hit = StaticHit {
                        obstacle: StaticObstacle::Pad(pid),
                        deficit: req - gap,
                        gap_um: gap,
                        required_um: base_req + extra_pos,
                    };
                    if FIRST_ONLY {
                        return Some(hit);
                    }
                    record_hit(&mut worst, hit);
                }
            }
        }

        // Fixed vias (other-net copper clearance and shared-layer drill hole-to-hole).
        let fv_reach = (r + max_clr).max(drill * 0.5 + hole_min + extra_pos + 10.0) + 50.0;
        self.fixed_via_index
            .query(BBox::from_point(pos, fv_reach), buf);
        for &vid in buf.iter() {
            if let Some(self_vid) = skip_fixed_via
                && vid <= self_vid
            {
                continue;
            }
            let fv = &self.fixed_vias[vid];
            if !fv.layers.iter().any(|l| via_layers.contains(l)) {
                continue;
            }
            let center_dist = pos.dist(fv.pos);
            let hole_gap = center_dist - (drill + fv.drill) * 0.5;
            let hole_req = hole_min + margin_extra;
            if same_net(net_id, fv.net_id) {
                if center_dist >= 5.0 && hole_gap < hole_req - 1e-4 {
                    let hit = StaticHit {
                        obstacle: StaticObstacle::FixedVia(vid),
                        deficit: hole_req - hole_gap,
                        gap_um: hole_gap,
                        required_um: hole_min + extra_pos,
                    };
                    if FIRST_ONLY {
                        return Some(hit);
                    }
                    record_hit(&mut worst, hit);
                }
                continue;
            }
            let base_req = self.pair_clearance(net_id, fv.net_id, |nr| nr.via_via);
            let req = base_req + margin_extra;
            let copper_gap = center_dist - r - fv.radius;
            if copper_gap < req - 1e-4 || hole_gap < hole_req - 1e-4 {
                let c_def = req - copper_gap;
                let h_def = hole_req - hole_gap;
                let (deficit, gap_um, required_um) = if c_def >= h_def {
                    (c_def, copper_gap, base_req + extra_pos)
                } else {
                    (h_def, hole_gap, hole_min + extra_pos)
                };
                let hit = StaticHit {
                    obstacle: StaticObstacle::FixedVia(vid),
                    deficit,
                    gap_um,
                    required_um,
                };
                if FIRST_ONLY {
                    return Some(hit);
                }
                record_hit(&mut worst, hit);
            }
        }

        // Fixed wires.
        if check_fixed_segs {
            self.fixed_seg_index.query(qbox, buf);
            for &sid in buf.iter() {
                let fs = &self.fixed_segments[sid];
                if same_net(net_id, fs.net_id) || !via_layers.contains(&fs.layer) {
                    continue;
                }
                let base_req = self.pair_clearance(net_id, fs.net_id, |nr| nr.via_wire);
                let req = base_req + margin_extra;
                let sh = Shape::Capsule {
                    a: fs.a,
                    b: fs.b,
                    radius: fs.half_w,
                };
                let gap = sh.dist_to_circle(pos, r);
                if gap < req - 1e-4 {
                    let hit = StaticHit {
                        obstacle: StaticObstacle::FixedWire(sid),
                        deficit: req - gap,
                        gap_um: gap,
                        required_um: base_req + extra_pos,
                    };
                    if FIRST_ONLY {
                        return Some(hit);
                    }
                    record_hit(&mut worst, hit);
                }
            }
        }

        // Signal-layer planes (other-net vias spanning the plane layer must clear it).
        self.fixed_plane_index.query(qbox, buf);
        for &pid in buf.iter() {
            let fp = &self.planes[pid];
            if same_net(net_id, fp.net_id) || !via_layers.contains(&fp.layer) {
                continue;
            }
            let base_req = self.pair_clearance(net_id, fp.net_id, |nr| nr.via_wire);
            let req = base_req + margin_extra;
            let gap = fp.shape.dist_to_circle(pos, r);
            if gap < req - 1e-4 {
                let hit = StaticHit {
                    obstacle: StaticObstacle::FixedPlane(pid),
                    deficit: req - gap,
                    gap_um: gap,
                    required_um: base_req + extra_pos,
                };
                if FIRST_ONLY {
                    return Some(hit);
                }
                record_hit(&mut worst, hit);
            }
        }

        worst
    }

    #[allow(clippy::too_many_arguments)]
    fn scan_segment_static<const FIRST_ONLY: bool>(
        &self,
        net_id: Option<usize>,
        layer: usize,
        a: Point,
        b: Point,
        width: f64,
        margin_extra: f64,
        skip_fixed_seg: Option<usize>,
        buf: &mut Vec<usize>,
    ) -> Option<StaticHit> {
        let half_w = width * 0.5;
        let extra_pos = margin_extra.max(0.0);
        let edge_clr = self.edge_clearance_um.unwrap_or(0.0);
        if self.boundary.len() >= 3 {
            let a_in = point_in_polygon(a, &self.boundary);
            let b_in = point_in_polygon(b, &self.boundary);
            if !a_in || !b_in {
                let gap = if FIRST_ONLY {
                    0.0
                } else {
                    let da = point_to_polygon_edge_dist(a, &self.boundary);
                    let db = point_to_polygon_edge_dist(b, &self.boundary);
                    (if a_in { da } else { -da }).min(if b_in { db } else { -db }) - half_w
                };
                return Some(StaticHit {
                    obstacle: StaticObstacle::Boundary,
                    deficit: edge_clr + extra_pos - gap,
                    gap_um: gap,
                    required_um: edge_clr + extra_pos,
                });
            }
            let edge_req = half_w + edge_clr + margin_extra;
            let edge_d = segment_to_polygon_edge_dist(a, b, &self.boundary);
            if edge_d < edge_req - 1e-4 {
                let pad_exit_ok = edge_d >= half_w + margin_extra - 1e-4
                    && net_id.is_some_and(|nid| {
                        self.segment_edge_pad_exit(
                            nid, layer, a, b, half_w, edge_clr, edge_d, edge_req,
                        )
                    });
                if !pad_exit_ok {
                    return Some(StaticHit {
                        obstacle: StaticObstacle::Boundary,
                        deficit: edge_req - edge_d,
                        gap_um: edge_d - half_w,
                        required_um: edge_clr + extra_pos,
                    });
                }
            }
        }

        let is_fixed = skip_fixed_seg.is_some();
        let max_clr = self.max_net_clearance + extra_pos;
        let qbox = BBox::from_points(&[a, b], half_w + max_clr + 50.0);
        let cutout_margin = self.edge_clearance_um.unwrap_or(145.0).max(145.0);
        let mut worst: Option<StaticHit> = None;

        // Keepouts (structure cutouts honor edge clearance for newly routed copper only).
        self.keepout_index
            .query(qbox.expand(cutout_margin + 15.0), buf);
        for &kid in buf.iter() {
            let k = &self.keepouts[kid];
            if k.kind == KeepoutKind::Via || !k.applies_to_layer(layer) {
                continue;
            }
            let k_margin = if is_fixed {
                0.0
            } else if k.is_structure && k.kind == KeepoutKind::All {
                cutout_margin
            } else {
                5.0
            };
            let gap = k.shape.dist_to_capsule(a, b, half_w);
            let req = k_margin + margin_extra;
            if gap < req - 1e-4 {
                let hit = StaticHit {
                    obstacle: StaticObstacle::Keepout(k.is_structure),
                    deficit: req - gap,
                    gap_um: gap,
                    required_um: k_margin + extra_pos,
                };
                if FIRST_ONLY {
                    return Some(hit);
                }
                record_hit(&mut worst, hit);
            }
        }

        // Other-net pads.
        self.pad_index.query(qbox, buf);
        for &pid in buf.iter() {
            let pad = &self.pads[pid];
            if same_net(net_id, pad.net_id) {
                continue;
            }
            let base_req = self.pair_clearance(net_id, pad.net_id, |nr| nr.clearance);
            let req = base_req + margin_extra;
            if let Some(sh) = pad.shape_on_layer(layer) {
                if is_fixed
                    && net_id.is_none()
                    && pad.net_id.is_none()
                    && (sh.dist_to_circle(a, 0.0) <= 1.0 || sh.dist_to_circle(b, 0.0) <= 1.0)
                {
                    continue;
                }
                let gap = sh.dist_to_capsule(a, b, half_w);
                if gap < req - 1e-4 {
                    let hit = StaticHit {
                        obstacle: StaticObstacle::Pad(pid),
                        deficit: req - gap,
                        gap_um: gap,
                        required_um: base_req + extra_pos,
                    };
                    if FIRST_ONLY {
                        return Some(hit);
                    }
                    record_hit(&mut worst, hit);
                }
            }
        }

        // Fixed vias.
        self.fixed_via_index.query(qbox, buf);
        for &vid in buf.iter() {
            let fv = &self.fixed_vias[vid];
            if same_net(net_id, fv.net_id) || !fv.layers.contains(&layer) {
                continue;
            }
            let sh = Shape::Circle {
                center: fv.pos,
                radius: fv.radius,
            };
            if is_fixed
                && net_id.is_none()
                && fv.net_id.is_none()
                && (sh.dist_to_circle(a, 0.0) <= 1.0 || sh.dist_to_circle(b, 0.0) <= 1.0)
            {
                continue;
            }
            let base_req = self.pair_clearance(net_id, fv.net_id, |nr| nr.via_wire);
            let req = base_req + margin_extra;
            let gap = sh.dist_to_capsule(a, b, half_w);
            if gap < req - 1e-4 {
                let hit = StaticHit {
                    obstacle: StaticObstacle::FixedVia(vid),
                    deficit: req - gap,
                    gap_um: gap,
                    required_um: base_req + extra_pos,
                };
                if FIRST_ONLY {
                    return Some(hit);
                }
                record_hit(&mut worst, hit);
            }
        }

        // Fixed wires.
        let self_wg = skip_fixed_seg.and_then(|s| self.fixed_segments[s].wire_group);
        self.fixed_seg_index.query(qbox, buf);
        for &sid in buf.iter() {
            if let Some(self_sid) = skip_fixed_seg {
                if sid <= self_sid {
                    continue;
                }
                let fs = &self.fixed_segments[sid];
                if fs.is_synthetic || (self_wg.is_some() && self_wg == fs.wire_group) {
                    continue;
                }
            }
            let fs = &self.fixed_segments[sid];
            if same_net(net_id, fs.net_id) || fs.layer != layer {
                continue;
            }
            if is_fixed
                && net_id.is_none()
                && fs.net_id.is_none()
                && (a.dist(fs.a) <= 1.0
                    || a.dist(fs.b) <= 1.0
                    || b.dist(fs.a) <= 1.0
                    || b.dist(fs.b) <= 1.0)
            {
                continue;
            }
            let base_req = self.pair_clearance(net_id, fs.net_id, |nr| nr.clearance);
            let req = base_req + margin_extra;
            let gap = segment_to_segment_dist(a, b, fs.a, fs.b) - half_w - fs.half_w;
            if gap < req - 1e-4 {
                let hit = StaticHit {
                    obstacle: StaticObstacle::FixedWire(sid),
                    deficit: req - gap,
                    gap_um: gap,
                    required_um: base_req + extra_pos,
                };
                if FIRST_ONLY {
                    return Some(hit);
                }
                record_hit(&mut worst, hit);
            }
        }

        // Signal-layer planes (other-net segments on the same layer must clear the plane).
        self.fixed_plane_index.query(qbox, buf);
        for &pid in buf.iter() {
            let fp = &self.planes[pid];
            if same_net(net_id, fp.net_id) || fp.layer != layer {
                continue;
            }
            let base_req = self.pair_clearance(net_id, fp.net_id, |nr| nr.clearance);
            let req = base_req + margin_extra;
            let gap = fp.shape.dist_to_capsule(a, b, half_w);
            if gap < req - 1e-4 {
                let hit = StaticHit {
                    obstacle: StaticObstacle::FixedPlane(pid),
                    deficit: req - gap,
                    gap_um: gap,
                    required_um: base_req + extra_pos,
                };
                if FIRST_ONLY {
                    return Some(hit);
                }
                record_hit(&mut worst, hit);
            }
        }

        worst
    }

    fn format_hit(&self, hit: StaticHit) -> StaticBlocker {
        let (label, net_b) = match hit.obstacle {
            StaticObstacle::Boundary => ("board_boundary".to_string(), None),
            StaticObstacle::Keepout(true) => ("keepout (structure)".to_string(), None),
            StaticObstacle::Keepout(false) => ("keepout (image)".to_string(), None),
            StaticObstacle::Pad(pid) => {
                let pad = &self.pads[pid];
                (
                    format!(
                        "pad {}-{} ({})",
                        pad.refdes,
                        pad.pin_name,
                        self.net_label(pad.net_id)
                    ),
                    pad.net_id,
                )
            }
            StaticObstacle::FixedVia(vid) => {
                let net = self.fixed_vias[vid].net_id;
                (format!("fixed_via ({})", self.net_label(net)), net)
            }
            StaticObstacle::FixedWire(sid) => {
                let net = self.fixed_segments[sid].net_id;
                (format!("fixed_wire ({})", self.net_label(net)), net)
            }
            StaticObstacle::FixedPlane(pid) => {
                let net = self.planes[pid].net_id;
                (format!("plane ({})", self.net_label(net)), net)
            }
        };
        StaticBlocker {
            label,
            net_b,
            gap_um: hit.gap_um,
            required_um: hit.required_um,
        }
    }

    fn net_label(&self, net_id: Option<usize>) -> &str {
        net_id.map_or("no-net", |id| self.nets[id].name.as_str())
    }

    fn pair_clearance(
        &self,
        net_a: Option<usize>,
        net_b: Option<usize>,
        field: impl Fn(&NetRules) -> f64,
    ) -> f64 {
        match (net_a, net_b) {
            (Some(a), Some(b)) => field(&self.nets[a].rules).max(field(&self.nets[b].rules)),
            (Some(a), None) => field(&self.nets[a].rules),
            (None, Some(b)) => field(&self.nets[b].rules),
            (None, None) => self.nets.first().map_or(100.0, |n| field(&n.rules)),
        }
    }

    // Allows inward escape from a pad whose own copper sits closer than edge_clr to the boundary.
    #[allow(clippy::too_many_arguments)]
    fn segment_edge_pad_exit(
        &self,
        net_id: usize,
        layer: usize,
        a: Point,
        b: Point,
        half_w: f64,
        edge_clr: f64,
        edge_d: f64,
        edge_req: f64,
    ) -> bool {
        let da = point_to_polygon_edge_dist(a, &self.boundary);
        let db = point_to_polygon_edge_dist(b, &self.boundary);
        let seg_len = a.dist(b);
        for &pid in &self.nets[net_id].pad_ids {
            let pad = &self.pads[pid];
            let Some(sh) = pad.shape_on_layer(layer) else {
                continue;
            };
            let pad_edge_dist = sh.dist_to_polygon_edge(&self.boundary);
            if pad_edge_dist >= edge_clr - 1e-4 || edge_d - half_w < pad_edge_dist - 1e-3 {
                continue;
            }
            let a_in = sh.dist_to_circle(a, 0.0) <= 1.0;
            let b_in = sh.dist_to_circle(b, 0.0) <= 1.0;
            if a_in && b_in {
                return true;
            }
            let (in_pt, out_pt, d_in, d_out) = if a_in {
                (a, b, da, db)
            } else if b_in {
                (b, a, db, da)
            } else {
                continue;
            };
            if d_in > edge_d + 1e-3 || d_out < edge_req - 1e-4 || d_out <= d_in + 1e-3 {
                continue;
            }
            let esc_len = ((edge_req - d_in).max(0.0) * 1.5).max(pad.max_radius + 150.0);
            if seg_len <= esc_len {
                if seg_len <= pad.max_radius + 150.0 || d_out - d_in >= seg_len * 0.5 - 1e-3 {
                    return true;
                }
            } else {
                let t = esc_len / seg_len;
                let mid = Point::new(
                    in_pt.x + t * (out_pt.x - in_pt.x),
                    in_pt.y + t * (out_pt.y - in_pt.y),
                );
                if segment_to_polygon_edge_dist(mid, out_pt, &self.boundary) >= edge_req - 1e-4 {
                    return true;
                }
            }
        }
        false
    }

    pub fn wire_to_other_clearance(&self, net_a: usize, net_b: Option<usize>) -> f64 {
        self.pair_clearance(Some(net_a), net_b, |r| r.clearance)
    }

    pub fn via_to_wire_clearance(&self, net_a: usize, net_b: Option<usize>) -> f64 {
        self.pair_clearance(Some(net_a), net_b, |r| r.via_wire)
    }

    pub fn via_to_via_clearance(&self, net_a: usize, net_b: Option<usize>) -> f64 {
        self.pair_clearance(Some(net_a), net_b, |r| r.via_via)
    }

    pub fn via_to_pad_clearance(&self, net_a: usize, pad: &Pad) -> f64 {
        self.pair_clearance(Some(net_a), pad.net_id, |r| {
            if pad.is_smd { r.via_smd } else { r.via_via }
        })
    }
}

fn same_net(a: Option<usize>, b: Option<usize>) -> bool {
    matches!((a, b), (Some(na), Some(nb)) if na == nb)
}

fn record_hit(slot: &mut Option<StaticHit>, cand: StaticHit) {
    if slot.is_none_or(|best| cand.deficit > best.deficit) {
        *slot = Some(cand);
    }
}

fn resolve_layer_spec(spec: &str, name_to_idx: &HashMap<String, usize>) -> Vec<usize> {
    if spec.eq_ignore_ascii_case("signal") || spec.eq_ignore_ascii_case("pcb") {
        Vec::new()
    } else if let Some(&idx) = name_to_idx.get(spec) {
        vec![idx]
    } else {
        Vec::new()
    }
}

fn resolve_via_padstack(
    design: &DsnDesign,
    name: &str,
    name_to_idx: &HashMap<String, usize>,
    num_layers: usize,
) -> (f64, f64, Vec<usize>) {
    let parsed_drill = parse_via_drill_from_name(name);
    if let Some(ps) = design.padstacks.get(name) {
        let mut max_r = 0.0_f64;
        let mut layers = Vec::new();
        for sh in &ps.shapes {
            if let Shape::Circle { radius, .. } = sh.shape {
                max_r = max_r.max(radius);
            } else {
                let bb = sh.shape.bbox();
                max_r = max_r.max((bb.max_x - bb.min_x).max(bb.max_y - bb.min_y) * 0.5);
            }
            let resolved = resolve_layer_spec(&sh.layer, name_to_idx);
            if resolved.is_empty() {
                layers.extend(0..num_layers);
            } else {
                layers.extend(resolved);
            }
        }
        layers.sort_unstable();
        layers.dedup();
        if layers.is_empty() {
            layers.extend(0..num_layers);
        }
        if max_r > 0.0 {
            let drill = parsed_drill.unwrap_or(max_r * 1.2);
            return (max_r, drill, layers);
        }
    }
    let fallback_r = parse_via_dia_from_name(name).unwrap_or(300.0) * 0.5;
    let drill = parsed_drill.unwrap_or(fallback_r * 1.2);
    (fallback_r, drill, (0..num_layers).collect())
}

fn parse_via_dia_from_name(name: &str) -> Option<f64> {
    let after_us = name.split('_').nth(1)?;
    let dia_str = after_us.split(':').next()?;
    dia_str.parse::<f64>().ok()
}

fn parse_via_drill_from_name(name: &str) -> Option<f64> {
    let after_us = name.split('_').nth(1)?;
    let drill_str = after_us.split(':').nth(1)?;
    drill_str.parse::<f64>().ok()
}

fn resolve_net_rules(
    design: &DsnDesign,
    cls: Option<&crate::dsn::NetClass>,
    default_via: &str,
    name_to_idx: &HashMap<String, usize>,
    num_layers: usize,
) -> NetRules {
    let def: &Rules = &design.default_rules;
    let cr = cls.map(|c| &c.rules);
    let pick = |a: Option<f64>, b: Option<f64>, fallback: f64| a.or(b).unwrap_or(fallback);

    let width = pick(cr.and_then(|r| r.width), def.width, 150.0);
    let clearance = pick(cr.and_then(|r| r.clearance), def.clearance, 150.0);
    let smd_smd = pick(cr.and_then(|r| r.smd_smd), def.smd_smd, clearance);
    let via_wire = pick(cr.and_then(|r| r.via_wire), def.via_wire, clearance);
    let via_via = pick(cr.and_then(|r| r.via_via), def.via_via, clearance);
    let via_smd = pick(cr.and_then(|r| r.via_smd), def.via_smd, clearance);

    let via_padstack = cls
        .and_then(|c| c.via_names.first().cloned())
        .unwrap_or_else(|| default_via.to_string());
    let (via_radius, via_drill, via_layers) =
        resolve_via_padstack(design, &via_padstack, name_to_idx, num_layers);

    NetRules {
        width,
        clearance,
        smd_smd,
        via_wire,
        via_via,
        via_smd,
        via_padstack,
        via_radius,
        via_drill,
        via_layers,
    }
}

fn polygon_area(pts: &[Point]) -> f64 {
    if pts.len() < 3 {
        return 0.0;
    }
    let mut area = 0.0;
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        area += a.x * b.y - b.x * a.y;
    }
    area.abs() * 0.5
}
