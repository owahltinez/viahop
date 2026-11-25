//! Full Specctra DSN (and optional `.rules`) parser targeting KiCad 10 grammar.

use std::collections::HashMap;

use crate::geometry::{Point, Shape};
use crate::sexp::{self, SExp};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    Front,
    Back,
}

#[derive(Debug, Clone)]
pub struct Layer {
    pub name: String,
    pub is_signal: bool,
    pub index: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KeepoutKind {
    All,
    Via,
    Wire,
}

#[derive(Debug, Clone)]
pub struct PlaneZone {
    pub net: String,
    pub layer: String,
    pub shape: Shape,
}

#[derive(Debug, Clone)]
pub struct Keepout {
    pub kind: KeepoutKind,
    /// `"signal"` means all copper layers; otherwise a specific layer name.
    pub layer: String,
    pub shape: Shape,
}

#[derive(Debug, Clone, Default)]
pub struct Rules {
    pub width: Option<f64>,
    pub clearance: Option<f64>,
    pub smd_smd: Option<f64>,
    pub via_wire: Option<f64>,
    pub via_via: Option<f64>,
    pub via_smd: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct ComponentPlace {
    pub component: String,
    pub refdes: String,
    pub pos: Point,
    pub side: Side,
    pub rotation_deg: f64,
    pub part_number: Option<String>,
}

#[derive(Debug, Clone)]
pub struct ImagePin {
    pub padstack: String,
    pub pin_name: String,
    pub pos: Point,
    pub rotation_deg: f64,
}

#[derive(Debug, Clone)]
pub struct Image {
    pub name: String,
    pub pins: Vec<ImagePin>,
    pub keepouts: Vec<Keepout>,
}

#[derive(Debug, Clone)]
pub struct PadShape {
    pub layer: String,
    pub shape: Shape,
}

#[derive(Debug, Clone)]
pub struct Padstack {
    pub name: String,
    pub shapes: Vec<PadShape>,
    pub attach: bool,
}

#[derive(Debug, Clone)]
pub struct DsnNet {
    pub name: String,
    /// `(refdes, pin_name)` pairs.
    pub pins: Vec<(String, String)>,
}

#[derive(Debug, Clone)]
pub struct NetClass {
    pub name: String,
    pub nets: Vec<String>,
    pub via_names: Vec<String>,
    pub rules: Rules,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WireType {
    Fix,
    Protect,
    Route,
}

#[derive(Debug, Clone)]
pub struct ExistingWire {
    pub layer: String,
    pub width: f64,
    pub points: Vec<Point>,
    pub net: String,
    pub wire_type: WireType,
}

#[derive(Debug, Clone)]
pub struct ExistingVia {
    pub padstack: String,
    pub pos: Point,
    pub net: String,
    pub wire_type: WireType,
}

/// Complete parsed Specctra DSN design, normalized internally to micrometers.
#[derive(Debug, Clone)]
pub struct DsnDesign {
    pub name: String,
    pub host_cad: String,
    pub host_version: String,
    pub resolution_unit: String,
    pub resolution_divisor: f64,
    pub unit: String,
    /// Multiply raw DSN coordinates by this factor to get micrometers.
    pub unit_to_um: f64,
    pub layers: Vec<Layer>,
    pub boundaries: Vec<Vec<Point>>,
    pub planes: Vec<PlaneZone>,
    pub keepouts: Vec<Keepout>,
    pub structure_vias: Vec<String>,
    pub default_rules: Rules,
    pub via_at_smd: bool,
    pub placements: Vec<ComponentPlace>,
    pub images: HashMap<String, Image>,
    pub padstacks: HashMap<String, Padstack>,
    pub nets: Vec<DsnNet>,
    pub classes: Vec<NetClass>,
    pub existing_wires: Vec<ExistingWire>,
    pub existing_vias: Vec<ExistingVia>,
}

impl DsnDesign {
    pub fn parse(input: &str) -> Result<Self, String> {
        let root = sexp::parse_dsn_tolerant(input)?;
        if root.tag() != Some("pcb") {
            return Err(format!("expected top-level (pcb ...), got {:?}", root.tag()));
        }
        let items = root.as_list().ok_or("invalid root")?;
        let name = items.get(1).and_then(SExp::as_atom).unwrap_or("design").to_string();

        let (host_cad, host_version) = parse_parser_section(&root);
        let (resolution_unit, resolution_divisor) = parse_resolution(&root);
        let unit = root
            .find_child("unit")
            .and_then(|u| u.as_list()?.get(1)?.as_atom())
            .unwrap_or(&resolution_unit)
            .to_string();
        let unit_to_um = unit_scale_to_um(&unit);

        let mut layers = Vec::new();
        let mut boundaries = Vec::new();
        let mut planes = Vec::new();
        let mut keepouts = Vec::new();
        let mut structure_vias = Vec::new();
        let mut default_rules = Rules::default();
        let mut via_at_smd = false;

        if let Some(structure) = root.find_child("structure") {
            for (idx, layer_node) in structure.children_with_tag("layer").enumerate() {
                if let Some(layer) = parse_layer(layer_node, idx) {
                    layers.push(layer);
                }
            }
            layers.sort_by_key(|l| l.index);
            for bnd in structure.children_with_tag("boundary") {
                if let Some(poly) = parse_boundary(bnd, unit_to_um) {
                    boundaries.push(poly);
                }
            }
            for pl_node in structure.children_with_tag("plane") {
                if let Some(pl) = parse_plane(pl_node, unit_to_um) {
                    planes.push(pl);
                }
            }
            parse_keepouts_in(structure, unit_to_um, &mut keepouts);
            for via_node in structure.children_with_tag("via") {
                if let Some(list) = via_node.as_list() {
                    for v in &list[1..] {
                        if let Some(name) = v.as_atom() {
                            structure_vias.push(name.to_string());
                        }
                    }
                }
            }
            for rule_node in structure.children_with_tag("rule") {
                parse_rules_into(rule_node, unit_to_um, &mut default_rules);
            }
            if let Some(ctrl) = structure.find_child("control") {
                if let Some(vas) = ctrl.find_child("via_at_smd") {
                    via_at_smd = vas
                        .as_list()
                        .and_then(|l| l.get(1))
                        .and_then(SExp::as_atom)
                        .is_some_and(|v| v.eq_ignore_ascii_case("on"));
                }
            }
        }

        let mut placements = Vec::new();
        if let Some(placement) = root.find_child("placement") {
            for comp in placement.children_with_tag("component") {
                parse_component_places(comp, unit_to_um, &mut placements);
            }
        }

        let mut images = HashMap::new();
        let mut padstacks = HashMap::new();
        if let Some(lib) = root.find_child("library") {
            for img_node in lib.children_with_tag("image") {
                if let Some(img) = parse_image(img_node, unit_to_um) {
                    images.insert(img.name.clone(), img);
                }
            }
            for ps_node in lib.children_with_tag("padstack") {
                if let Some(ps) = parse_padstack(ps_node, unit_to_um) {
                    padstacks.insert(ps.name.clone(), ps);
                }
            }
        }

        let mut nets = Vec::new();
        let mut classes = Vec::new();
        if let Some(network) = root.find_child("network") {
            for net_node in network.children_with_tag("net") {
                if let Some(net) = parse_net(net_node) {
                    nets.push(net);
                }
            }
            for class_node in network.children_with_tag("class") {
                if let Some(cls) = parse_class(class_node, unit_to_um) {
                    classes.push(cls);
                }
            }
        }

        let mut existing_wires = Vec::new();
        let mut existing_vias = Vec::new();
        if let Some(wiring) = root.find_child("wiring") {
            for w_node in wiring.children_with_tag("wire") {
                if let Some(w) = parse_existing_wire(w_node, unit_to_um) {
                    existing_wires.push(w);
                }
            }
            for v_node in wiring.children_with_tag("via") {
                if let Some(v) = parse_existing_via(v_node, unit_to_um) {
                    existing_vias.push(v);
                }
            }
        }

        Ok(Self {
            name,
            host_cad,
            host_version,
            resolution_unit,
            resolution_divisor,
            unit,
            unit_to_um,
            layers,
            boundaries,
            planes,
            keepouts,
            structure_vias,
            default_rules,
            via_at_smd,
            placements,
            images,
            padstacks,
            nets,
            classes,
            existing_wires,
            existing_vias,
        })
    }

    /// Merges an optional Specctra `.rules` file into this design's default and class rules.
    pub fn apply_rules_file(&mut self, rules_src: &str) -> Result<(), String> {
        let exprs = sexp::parse_all(rules_src)?;
        for expr in &exprs {
            self.apply_rules_node(expr);
        }
        Ok(())
    }

    fn apply_rules_node(&mut self, node: &SExp) {
        let Some(tag) = node.tag() else { return };
        match tag {
            "rules" | "structure" | "network" => {
                if let Some(children) = node.as_list() {
                    for c in &children[1..] {
                        self.apply_rules_node(c);
                    }
                }
            }
            "rule" => parse_rules_into(node, self.unit_to_um, &mut self.default_rules),
            "class" => {
                if let Some(cls) = parse_class(node, self.unit_to_um) {
                    if let Some(existing) = self.classes.iter_mut().find(|c| c.name == cls.name) {
                        merge_rules(&mut existing.rules, &cls.rules);
                        if !cls.via_names.is_empty() {
                            existing.via_names = cls.via_names;
                        }
                    } else {
                        self.classes.push(cls);
                    }
                }
            }
            _ => {}
        }
    }
}

pub fn unit_scale_to_um(unit: &str) -> f64 {
    match unit.to_ascii_lowercase().as_str() {
        "um" => 1.0,
        "mm" => 1000.0,
        "mil" => 25.4,
        "inch" | "in" => 25400.0,
        _ => 1.0,
    }
}

fn parse_parser_section(root: &SExp) -> (String, String) {
    let mut cad = "KiCad's Pcbnew".to_string();
    let mut ver = "10.0".to_string();
    if let Some(p) = root.find_child("parser") {
        if let Some(c) = p.find_child("host_cad").and_then(|n| n.as_list()?.get(1)?.as_atom()) {
            cad = c.to_string();
        }
        if let Some(v) = p.find_child("host_version").and_then(|n| n.as_list()?.get(1)?.as_atom()) {
            ver = v.to_string();
        }
    }
    (cad, ver)
}

fn parse_resolution(root: &SExp) -> (String, f64) {
    if let Some(res) = root.find_child("resolution").and_then(SExp::as_list) {
        let unit = res.get(1).and_then(SExp::as_atom).unwrap_or("um").to_string();
        let div = res.get(2).and_then(SExp::as_f64).unwrap_or(10.0);
        return (unit, div);
    }
    ("um".to_string(), 10.0)
}

fn parse_layer(node: &SExp, fallback_idx: usize) -> Option<Layer> {
    let list = node.as_list()?;
    let name = list.get(1)?.as_atom()?.to_string();
    let is_signal = node
        .find_child("type")
        .and_then(|t| t.as_list()?.get(1)?.as_atom())
        .map(|s| !s.eq_ignore_ascii_case("power"))
        .unwrap_or(true);
    let index = node
        .find_child("property")
        .and_then(|p| p.find_child("index"))
        .and_then(|i| i.as_list()?.get(1)?.as_i64())
        .and_then(|i| usize::try_from(i).ok())
        .unwrap_or(fallback_idx);
    Some(Layer {
        name,
        is_signal,
        index,
    })
}

fn parse_boundary(node: &SExp, scale: f64) -> Option<Vec<Point>> {
    if let Some(path) = node.find_child("path").and_then(SExp::as_list) {
        let pts = parse_coord_pairs(&path[3..], scale);
        if pts.len() >= 3 {
            return Some(pts);
        }
    }
    if let Some(rect) = node.find_child("rect").and_then(SExp::as_list) {
        if rect.len() >= 6 {
            let x1 = rect[2].as_f64()? * scale;
            let y1 = rect[3].as_f64()? * scale;
            let x2 = rect[4].as_f64()? * scale;
            let y2 = rect[5].as_f64()? * scale;
            return Some(vec![
                Point::new(x1, y1),
                Point::new(x2, y1),
                Point::new(x2, y2),
                Point::new(x1, y2),
            ]);
        }
    }
    None
}

fn parse_plane(node: &SExp, scale: f64) -> Option<PlaneZone> {
    let list = node.as_list()?;
    let net = list.get(1)?.as_atom()?.to_string();
    let shape_node = list.get(2)?;
    let (layer, shape) = parse_shape_expr(shape_node, scale)?;
    Some(PlaneZone { net, layer, shape })
}

fn parse_keepouts_in(parent: &SExp, scale: f64, out: &mut Vec<Keepout>) {
    let Some(list) = parent.as_list() else { return };
    for child in &list[1..] {
        let kind = match child.tag() {
            Some("keepout") => KeepoutKind::All,
            Some("via_keepout") => KeepoutKind::Via,
            Some("wire_keepout") => KeepoutKind::Wire,
            _ => continue,
        };
        let Some(klist) = child.as_list() else { continue };
        for sub in &klist[1..] {
            if let Some((layer, shape)) = parse_shape_expr(sub, scale) {
                out.push(Keepout { kind, layer, shape });
            }
        }
    }
}

fn parse_shape_expr(node: &SExp, scale: f64) -> Option<(String, Shape)> {
    let list = node.as_list()?;
    let tag = list.first()?.as_atom()?;
    let layer = list.get(1)?.as_atom()?.to_string();
    match tag {
        "circle" => {
            let dia = list.get(2)?.as_f64()? * scale;
            let cx = list.get(3).and_then(SExp::as_f64).unwrap_or(0.0) * scale;
            let cy = list.get(4).and_then(SExp::as_f64).unwrap_or(0.0) * scale;
            Some((
                layer,
                Shape::Circle {
                    center: Point::new(cx, cy),
                    radius: dia * 0.5,
                },
            ))
        }
        "rect" => {
            let x1 = list.get(2)?.as_f64()? * scale;
            let y1 = list.get(3)?.as_f64()? * scale;
            let x2 = list.get(4)?.as_f64()? * scale;
            let y2 = list.get(5)?.as_f64()? * scale;
            Some((layer, Shape::rect(x1, y1, x2, y2)))
        }
        "polygon" => {
            // (polygon <layer> <aperture_width> x1 y1 x2 y2 ...)
            let pts = parse_coord_pairs(list.get(3..)?, scale);
            if pts.len() >= 3 {
                Some((layer, Shape::polygon(pts)))
            } else {
                None
            }
        }
        "path" => {
            let w = list.get(2)?.as_f64()? * scale;
            let pts = parse_coord_pairs(list.get(3..)?, scale);
            if pts.len() >= 2 {
                Some((
                    layer,
                    Shape::Capsule {
                        a: pts[0],
                        b: *pts.last().unwrap(),
                        radius: w * 0.5,
                    },
                ))
            } else if pts.len() == 1 {
                Some((
                    layer,
                    Shape::Circle {
                        center: pts[0],
                        radius: w * 0.5,
                    },
                ))
            } else {
                None
            }
        }
        _ => None,
    }
}

fn parse_rules_into(node: &SExp, scale: f64, rules: &mut Rules) {
    let Some(list) = node.as_list() else { return };
    for child in &list[1..] {
        let Some(clist) = child.as_list() else { continue };
        match child.tag() {
            Some("width") => {
                if let Some(w) = clist.get(1).and_then(SExp::as_f64) {
                    rules.width = Some(w * scale);
                }
            }
            Some("clearance") => {
                let Some(val) = clist.get(1).and_then(SExp::as_f64) else {
                    continue;
                };
                let val = val * scale;
                let ctype = child
                    .find_child("type")
                    .and_then(|t| t.as_list()?.get(1)?.as_atom());
                match ctype {
                    None => rules.clearance = Some(val),
                    Some("smd_smd") => rules.smd_smd = Some(val),
                    Some("via_wire") | Some("wire_via") => rules.via_wire = Some(val),
                    Some("via_via") => rules.via_via = Some(val),
                    Some("via_smd") | Some("smd_via") => rules.via_smd = Some(val),
                    _ => {}
                }
            }
            _ => {}
        }
    }
}

fn merge_rules(dst: &mut Rules, src: &Rules) {
    if src.width.is_some() {
        dst.width = src.width;
    }
    if src.clearance.is_some() {
        dst.clearance = src.clearance;
    }
    if src.smd_smd.is_some() {
        dst.smd_smd = src.smd_smd;
    }
    if src.via_wire.is_some() {
        dst.via_wire = src.via_wire;
    }
    if src.via_via.is_some() {
        dst.via_via = src.via_via;
    }
    if src.via_smd.is_some() {
        dst.via_smd = src.via_smd;
    }
}

fn parse_component_places(comp_node: &SExp, scale: f64, out: &mut Vec<ComponentPlace>) {
    let Some(list) = comp_node.as_list() else { return };
    let Some(component) = list.get(1).and_then(SExp::as_atom) else { return };
    for place in comp_node.children_with_tag("place") {
        let Some(pl) = place.as_list() else { continue };
        let Some(refdes) = pl.get(1).and_then(SExp::as_atom) else { continue };
        let (Some(x), Some(y)) = (
            pl.get(2).and_then(SExp::as_f64),
            pl.get(3).and_then(SExp::as_f64),
        ) else {
            continue;
        };
        let side = match pl.get(4).and_then(SExp::as_atom) {
            Some(s) if s.eq_ignore_ascii_case("back") => Side::Back,
            _ => Side::Front,
        };
        let rotation_deg = pl.get(5).and_then(SExp::as_f64).unwrap_or(0.0);
        let part_number = place
            .find_child("PN")
            .and_then(|pn| pn.as_list()?.get(1)?.as_atom())
            .map(ToString::to_string);
        out.push(ComponentPlace {
            component: component.to_string(),
            refdes: refdes.to_string(),
            pos: Point::new(x * scale, y * scale),
            side,
            rotation_deg,
            part_number,
        });
    }
}

fn parse_image(node: &SExp, scale: f64) -> Option<Image> {
    let list = node.as_list()?;
    let name = list.get(1)?.as_atom()?.to_string();
    let mut pins = Vec::new();
    for pin_node in node.children_with_tag("pin") {
        let Some(pl) = pin_node.as_list() else { continue };
        let Some(padstack) = pl.get(1).and_then(SExp::as_atom) else { continue };
        let (rotation_deg, idx) = if pl.get(2).and_then(SExp::tag) == Some("rotate") {
            let deg = pl[2]
                .as_list()
                .and_then(|r| r.get(1))
                .and_then(SExp::as_f64)
                .unwrap_or(0.0);
            (deg, 3)
        } else {
            (0.0, 2)
        };
        let Some(pin_name) = pl.get(idx).and_then(SExp::as_atom) else { continue };
        let (Some(x), Some(y)) = (
            pl.get(idx + 1).and_then(SExp::as_f64),
            pl.get(idx + 2).and_then(SExp::as_f64),
        ) else {
            continue;
        };
        pins.push(ImagePin {
            padstack: padstack.to_string(),
            pin_name: pin_name.to_string(),
            pos: Point::new(x * scale, y * scale),
            rotation_deg,
        });
    }
    let mut keepouts = Vec::new();
    parse_keepouts_in(node, scale, &mut keepouts);
    Some(Image {
        name,
        pins,
        keepouts,
    })
}

fn parse_padstack(node: &SExp, scale: f64) -> Option<Padstack> {
    let list = node.as_list()?;
    let name = list.get(1)?.as_atom()?.to_string();
    let mut shapes = Vec::new();
    for sh in node.children_with_tag("shape") {
        if let Some(inner) = sh.as_list().and_then(|l| l.get(1)) {
            if let Some((layer, shape)) = parse_shape_expr(inner, scale) {
                shapes.push(PadShape { layer, shape });
            }
        }
    }
    let attach = node
        .find_child("attach")
        .and_then(|a| a.as_list()?.get(1)?.as_atom())
        .is_some_and(|v| v.eq_ignore_ascii_case("on"));
    Some(Padstack {
        name,
        shapes,
        attach,
    })
}

fn parse_net(node: &SExp) -> Option<DsnNet> {
    let list = node.as_list()?;
    let name = list.get(1)?.as_atom()?.to_string();
    let mut pins = Vec::new();
    if let Some(pins_node) = node.find_child("pins").and_then(SExp::as_list) {
        for item in &pins_node[1..] {
            if let Some(tok) = item.as_atom() {
                if let Some((refdes, pin)) = tok.split_once('-') {
                    pins.push((refdes.to_string(), pin.to_string()));
                }
            }
        }
    }
    Some(DsnNet { name, pins })
}

fn parse_class(node: &SExp, scale: f64) -> Option<NetClass> {
    let list = node.as_list()?;
    let name = list.get(1)?.as_atom()?.to_string();
    let mut nets = Vec::new();
    let mut via_names = Vec::new();
    let mut rules = Rules::default();
    for item in &list[2..] {
        if let Some(net_name) = item.as_atom() {
            nets.push(net_name.to_string());
        } else if item.tag() == Some("circuit") {
            for uv in item.children_with_tag("use_via") {
                if let Some(uvl) = uv.as_list() {
                    for v in &uvl[1..] {
                        if let Some(vn) = v.as_atom() {
                            via_names.push(vn.to_string());
                        }
                    }
                }
            }
        } else if item.tag() == Some("rule") {
            parse_rules_into(item, scale, &mut rules);
        }
    }
    Some(NetClass {
        name,
        nets,
        via_names,
        rules,
    })
}

fn parse_wire_type(parent: &SExp) -> WireType {
    match parent
        .find_child("type")
        .and_then(|t| t.as_list()?.get(1)?.as_atom())
    {
        Some(s) if s.eq_ignore_ascii_case("fix") => WireType::Fix,
        Some(s) if s.eq_ignore_ascii_case("protect") => WireType::Protect,
        _ => WireType::Route,
    }
}

fn parse_existing_wire(node: &SExp, scale: f64) -> Option<ExistingWire> {
    let path = node.find_child("path")?.as_list()?;
    let layer = path.get(1)?.as_atom()?.to_string();
    let width = path.get(2)?.as_f64()? * scale;
    let points = parse_coord_pairs(path.get(3..)?, scale);
    if points.len() < 2 {
        return None;
    }
    let net = node
        .find_child("net")
        .and_then(|n| n.as_list()?.get(1)?.as_atom())
        .unwrap_or("")
        .to_string();
    Some(ExistingWire {
        layer,
        width,
        points,
        net,
        wire_type: parse_wire_type(node),
    })
}

fn parse_existing_via(node: &SExp, scale: f64) -> Option<ExistingVia> {
    let list = node.as_list()?;
    let padstack = list.get(1)?.as_atom()?.to_string();
    let x = list.get(2)?.as_f64()? * scale;
    let y = list.get(3)?.as_f64()? * scale;
    let net = node
        .find_child("net")
        .and_then(|n| n.as_list()?.get(1)?.as_atom())
        .unwrap_or("")
        .to_string();
    Some(ExistingVia {
        padstack,
        pos: Point::new(x, y),
        net,
        wire_type: parse_wire_type(node),
    })
}

fn parse_coord_pairs(slice: &[SExp], scale: f64) -> Vec<Point> {
    let mut pts = Vec::with_capacity(slice.len() / 2);
    let mut i = 0;
    while i + 1 < slice.len() {
        if let (Some(x), Some(y)) = (slice[i].as_f64(), slice[i + 1].as_f64()) {
            pts.push(Point::new(x * scale, y * scale));
        }
        i += 2;
    }
    pts
}
