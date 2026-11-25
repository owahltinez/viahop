//! Resolved physical board representation, spatial index, and exact continuous DRC.

use std::collections::{HashMap, HashSet};

use crate::dsn::{DsnDesign, KeepoutKind, Rules, Side, WireType};
use crate::geometry::{
    point_in_polygon, point_to_polygon_edge_dist, segment_to_polygon_dist, segment_to_segment_dist,
    transform_point, BBox, Point, Shape,
};

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
        self.layer_shapes.iter().find(|(l, _)| *l == layer).map(|(_, s)| s)
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
        let cols = (((bounds.max_x - bounds.min_x + 2.0 * margin) / cell_size).ceil() as usize).max(1);
        let rows = (((bounds.max_y - bounds.min_y + 2.0 * margin) / cell_size).ceil() as usize).max(1);
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
        (gx(bbox.min_x), gy(bbox.min_y), gx(bbox.max_x), gy(bbox.max_y))
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
    pub nets: Vec<NetInfo>,
    pub net_name_to_id: HashMap<String, usize>,
    pub via_at_smd: bool,
    pub pad_index: SpatialGrid,
    pub keepout_index: SpatialGrid,
    pub fixed_seg_index: SpatialGrid,
    pub fixed_via_index: SpatialGrid,
}

impl Board {
    pub fn from_dsn(design: &DsnDesign, ignored: &HashSet<String>, force_via_at_smd: Option<bool>) -> Result<Self, String> {
        if design.layers.is_empty() {
            return Err("DSN contains no copper layers".to_string());
        }
        let num_layers = design.layers.len();
        let layer_names: Vec<String> = design.layers.iter().map(|l| l.name.clone()).collect();
        let mut layer_name_to_idx = HashMap::new();
        let mut signal_layers = Vec::new();
        for (i, l) in design.layers.iter().enumerate() {
            layer_name_to_idx.insert(l.name.clone(), i);
            let has_inner_plane = i > 0
                && i + 1 < num_layers
                && design.planes.iter().any(|p| p.layer == l.name);
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
                            ((p.bbox.max_x - p.bbox.min_x)
                                .min(p.bbox.max_y - p.bbox.min_y))
                                * 0.5
                        };
                        let half_w = pad_r(p1).min(pad_r(p2)).max(120.0);
                        fixed_segments.push(FixedSegment {
                            layer,
                            a,
                            b,
                            half_w,
                            net_id: None,
                            bbox: BBox::from_points(&[a, b], half_w),
                        });
                    }
                }
            }
        }

        // Resolve fixed/protected existing wires and vias.
        for w in &design.existing_wires {
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

        // Resolve net rules and ignored sets.
        let mut nets = Vec::with_capacity(design.nets.len());
        let default_via = design
            .structure_vias
            .first()
            .cloned()
            .unwrap_or_else(|| "Via[0-1]_500:300_um".to_string());

        for (id, dnet) in design.nets.iter().enumerate() {
            let cls = net_to_class.get(dnet.name.as_str()).copied();
            let class_name = cls.map(|c| c.name.clone()).unwrap_or_else(|| "default".to_string());
            let rules = resolve_net_rules(design, cls, &default_via, &layer_name_to_idx, num_layers);
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

        let via_at_smd = force_via_at_smd.unwrap_or(design.via_at_smd);

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
            nets,
            net_name_to_id,
            via_at_smd,
            pad_index,
            keepout_index,
            fixed_seg_index,
            fixed_via_index,
        })
    }

    pub fn num_layers(&self) -> usize {
        self.layer_names.len()
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
        point_to_polygon_edge_dist(p, &self.boundary) >= margin
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
        let r = rules.via_radius;
        if !self.is_inside_boundary(pos, r + margin_extra) {
            return false;
        }
        let max_clr = rules.via_via.max(rules.via_smd).max(rules.via_wire) + margin_extra;
        let qbox = BBox::from_point(pos, r + max_clr + 50.0);

        // Keepouts (structure cutouts need extra margin for curved Edge.Cuts clearance).
        self.keepout_index.query(qbox.expand(160.0), buf);
        for &kid in buf.iter() {
            let k = &self.keepouts[kid];
            if k.kind == KeepoutKind::Wire {
                continue;
            }
            if !rules.via_layers.iter().any(|&l| k.applies_to_layer(l)) {
                continue;
            }
            let k_margin = if k.is_structure && k.kind == KeepoutKind::All {
                145.0
            } else {
                5.0
            };
            if k.shape.dist_to_circle(pos, r) < k_margin + margin_extra {
                return false;
            }
        }

        // Other-net pads
        self.pad_index.query(qbox, buf);
        for &pid in buf.iter() {
            let pad = &self.pads[pid];
            if pad.net_id == Some(net_id) {
                continue;
            }
            let req = self.via_to_pad_clearance(net_id, pad) + margin_extra;
            for &(l, ref sh) in &pad.layer_shapes {
                if !rules.via_layers.contains(&l) {
                    continue;
                }
                if sh.dist_to_circle(pos, r) < req - 1e-4 {
                    return false;
                }
            }
        }

        // Fixed vias (check both other-net copper clearance and same-net drill hole-to-hole).
        self.fixed_via_index.query(qbox.expand(120.0), buf);
        for &vid in buf.iter() {
            let fv = &self.fixed_vias[vid];
            let center_dist = pos.dist(fv.pos);
            if fv.net_id == Some(net_id) {
                if center_dist >= 5.0 {
                    let min_hole_c2c = (rules.via_drill + fv.drill) * 0.5 + 205.0 + margin_extra;
                    if center_dist < min_hole_c2c - 1e-4 {
                        return false;
                    }
                }
                continue;
            }
            if !fv.layers.iter().any(|l| rules.via_layers.contains(l)) {
                continue;
            }
            let req = self.via_to_via_clearance(net_id, fv.net_id) + margin_extra;
            let hole_c2c = (rules.via_drill + fv.drill) * 0.5 + 205.0 + margin_extra;
            if center_dist - r - fv.radius < req - 1e-4 || center_dist < hole_c2c - 1e-4 {
                return false;
            }
        }

        // Fixed wires
        self.fixed_seg_index.query(qbox, buf);
        for &sid in buf.iter() {
            let fs = &self.fixed_segments[sid];
            if fs.net_id == Some(net_id) || !rules.via_layers.contains(&fs.layer) {
                continue;
            }
            let req = self.via_to_wire_clearance(net_id, fs.net_id) + margin_extra;
            let sh = Shape::Capsule {
                a: fs.a,
                b: fs.b,
                radius: fs.half_w,
            };
            if sh.dist_to_circle(pos, r) < req - 1e-4 {
                return false;
            }
        }

        true
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
        let rules = &self.nets[net_id].rules;
        let half_w = width * 0.5;
        if self.boundary.len() >= 3 {
            if !point_in_polygon(a, &self.boundary) || !point_in_polygon(b, &self.boundary) {
                return false;
            }
            if segment_to_polygon_dist(a, b, &self.boundary) < half_w + margin_extra {
                let da = point_to_polygon_edge_dist(a, &self.boundary);
                let db = point_to_polygon_edge_dist(b, &self.boundary);
                if da < half_w + margin_extra || db < half_w + margin_extra {
                    return false;
                }
            }
        }
        let max_clr = rules.clearance.max(rules.via_wire) + margin_extra;
        let qbox = BBox::from_points(&[a, b], half_w + max_clr + 50.0);

        // Keepouts (structure cutouts carry extra margin for curved Edge.Cuts clearance).
        self.keepout_index.query(qbox.expand(160.0), buf);
        for &kid in buf.iter() {
            let k = &self.keepouts[kid];
            if k.kind == KeepoutKind::Via || !k.applies_to_layer(layer) {
                continue;
            }
            let k_margin = if k.is_structure && k.kind == KeepoutKind::All {
                145.0
            } else {
                5.0
            };
            if k.shape.dist_to_capsule(a, b, half_w) < k_margin + margin_extra {
                return false;
            }
        }

        // Other-net pads
        self.pad_index.query(qbox, buf);
        for &pid in buf.iter() {
            let pad = &self.pads[pid];
            if pad.net_id == Some(net_id) {
                continue;
            }
            let req = self.wire_to_other_clearance(net_id, pad.net_id) + margin_extra;
            if let Some(sh) = pad.shape_on_layer(layer)
                && sh.dist_to_capsule(a, b, half_w) < req - 1e-4 {
                    return false;
                }
        }

        // Fixed vias
        self.fixed_via_index.query(qbox, buf);
        for &vid in buf.iter() {
            let fv = &self.fixed_vias[vid];
            if fv.net_id == Some(net_id) || !fv.layers.contains(&layer) {
                continue;
            }
            let req = self.via_to_wire_clearance(net_id, fv.net_id) + margin_extra;
            let sh = Shape::Circle {
                center: fv.pos,
                radius: fv.radius,
            };
            if sh.dist_to_capsule(a, b, half_w) < req - 1e-4 {
                return false;
            }
        }

        // Fixed wires
        self.fixed_seg_index.query(qbox, buf);
        for &sid in buf.iter() {
            let fs = &self.fixed_segments[sid];
            if fs.net_id == Some(net_id) || fs.layer != layer {
                continue;
            }
            let req = self.wire_to_other_clearance(net_id, fs.net_id) + margin_extra;
            let gap = segment_to_segment_dist(a, b, fs.a, fs.b) - half_w - fs.half_w;
            if gap < req - 1e-4 {
                return false;
            }
        }

        true
    }

    pub fn wire_to_other_clearance(&self, net_a: usize, net_b: Option<usize>) -> f64 {
        let ca = self.nets[net_a].rules.clearance;
        match net_b {
            Some(nb) => ca.max(self.nets[nb].rules.clearance),
            None => ca,
        }
    }

    pub fn via_to_wire_clearance(&self, net_a: usize, net_b: Option<usize>) -> f64 {
        let ca = self.nets[net_a].rules.via_wire;
        match net_b {
            Some(nb) => ca.max(self.nets[nb].rules.via_wire),
            None => ca,
        }
    }

    pub fn via_to_via_clearance(&self, net_a: usize, net_b: Option<usize>) -> f64 {
        let ca = self.nets[net_a].rules.via_via;
        match net_b {
            Some(nb) => ca.max(self.nets[nb].rules.via_via),
            None => ca,
        }
    }

    pub fn via_to_pad_clearance(&self, net_a: usize, pad: &Pad) -> f64 {
        let base = if pad.is_smd {
            self.nets[net_a].rules.via_smd
        } else {
            self.nets[net_a].rules.via_via
        };
        match pad.net_id {
            Some(nb) => {
                let other = if pad.is_smd {
                    self.nets[nb].rules.via_smd
                } else {
                    self.nets[nb].rules.via_via
                };
                base.max(other)
            }
            None => base,
        }
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
