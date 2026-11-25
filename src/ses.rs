//! KiCad-10-compatible Specctra SES session file reader and writer.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::fmt::Write;
use std::path::Path;

use crate::board::{Board, NetRoute, RoutedSegment, RoutedVia};
use crate::dsn::{DsnDesign, Side};
use crate::geometry::{Point, Shape};
use crate::sexp::{self, format_atom, SExp};

/// Serializes routed tracks and vias into a Specctra `.ses` file compatible with
/// KiCad 10's `pcbnew.ImportSpecctraSES`.
pub fn write_ses(design: &DsnDesign, board: &Board, routes: &[NetRoute]) -> String {
    let mut out = String::with_capacity(64 * 1024);
    let session_name = base_stem(&design.name);
    let res_unit = &design.resolution_unit;
    let res_div = design.resolution_divisor;
    // Scale micrometers into SES integer coordinates: um / unit_to_um * resolution_divisor.
    let um_to_ses = res_div / crate::dsn::unit_scale_to_um(res_unit);

    let _ = writeln!(out, "(session {}", format_atom(&session_name));
    let _ = writeln!(out, "  (base_design {})", format_atom(&session_name));

    // Placement block (grouped by component name).
    let _ = writeln!(out, "  (placement");
    let _ = writeln!(out, "    (resolution {} {})", format_atom(res_unit), res_div as i64);
    let mut by_comp: BTreeMap<&str, Vec<&crate::dsn::ComponentPlace>> = BTreeMap::new();
    for p in &design.placements {
        by_comp.entry(p.component.as_str()).or_default().push(p);
    }
    for (comp, places) in by_comp {
        let _ = writeln!(out, "    (component {}", format_atom(comp));
        for p in places {
            let side = match p.side {
                Side::Front => "front",
                Side::Back => "back",
            };
            let _ = writeln!(
                out,
                "      (place {} {} {} {} {})",
                format_atom(&p.refdes),
                scale_i64(p.pos.x, um_to_ses),
                scale_i64(p.pos.y, um_to_ses),
                side,
                format_angle(p.rotation_deg),
            );
        }
        let _ = writeln!(out, "    )");
    }
    let _ = writeln!(out, "  )");

    let _ = writeln!(out, "  (was_is");
    let _ = writeln!(out, "  )");

    let _ = writeln!(out, "  (routes");
    let _ = writeln!(out, "    (resolution {} {})", format_atom(res_unit), res_div as i64);
    let _ = writeln!(out, "    (parser");
    let _ = writeln!(out, "      (host_cad {})", format_atom(&design.host_cad));
    let _ = writeln!(out, "      (host_version {})", format_atom(&design.host_version));
    let _ = writeln!(out, "    )");

    // library_out: emit deduplicated via padstacks used by routed vias (plus default structure vias).
    let mut used_vias: HashSet<String> = HashSet::new();
    for nr in routes {
        for v in &nr.vias {
            used_vias.insert(v.padstack.clone());
        }
    }
    for sv in &design.structure_vias {
        used_vias.insert(sv.clone());
    }
    let mut sorted_vias: Vec<String> = used_vias.into_iter().collect();
    sorted_vias.sort();

    let _ = writeln!(out, "    (library_out");
    for vname in &sorted_vias {
        write_via_padstack(&mut out, design, board, vname, um_to_ses);
    }
    let _ = writeln!(out, "    )");

    // network_out: emit only nets that have newly routed segments or vias.
    let _ = writeln!(out, "    (network_out");
    for nr in routes {
        if nr.segments.is_empty() && nr.vias.is_empty() {
            continue;
        }
        let net_name = &board.nets[nr.net_id].name;
        let _ = writeln!(out, "      (net {}", format_atom(net_name));
        let chains = chain_segments(&nr.segments);
        for (layer, width, pts) in chains {
            let layer_name = &board.layer_names[layer];
            let _ = writeln!(out, "        (wire");
            let _ = writeln!(
                out,
                "          (path {} {}",
                format_atom(layer_name),
                scale_i64(width, um_to_ses)
            );
            for pt in pts {
                let _ = writeln!(
                    out,
                    "            {} {}",
                    scale_i64(pt.x, um_to_ses),
                    scale_i64(pt.y, um_to_ses)
                );
            }
            let _ = writeln!(out, "          )");
            let _ = writeln!(out, "        )");
        }
        for v in &nr.vias {
            let _ = writeln!(
                out,
                "        (via {} {} {}",
                format_atom(&v.padstack),
                scale_i64(v.pos.x, um_to_ses),
                scale_i64(v.pos.y, um_to_ses)
            );
            let _ = writeln!(out, "        )");
        }
        let _ = writeln!(out, "      )");
    }
    let _ = writeln!(out, "    )");
    let _ = writeln!(out, "  )");
    let _ = writeln!(out, ")");
    out
}

fn write_via_padstack(
    out: &mut String,
    design: &DsnDesign,
    board: &Board,
    vname: &str,
    um_to_ses: f64,
) {
    let _ = writeln!(out, "      (padstack {}", format_atom(vname));
    if let Some(ps) = design.padstacks.get(vname) {
        for sh in &ps.shapes {
            if let Shape::Circle { center, radius } = sh.shape {
                let _ = writeln!(out, "        (shape");
                let _ = writeln!(
                    out,
                    "          (circle {} {} {} {})",
                    format_atom(&sh.layer),
                    scale_i64(radius * 2.0, um_to_ses),
                    scale_i64(center.x, um_to_ses),
                    scale_i64(center.y, um_to_ses),
                );
                let _ = writeln!(out, "        )");
            }
        }
        let _ = writeln!(out, "        (attach {})", if ps.attach { "on" } else { "off" });
    } else {
        let dia = 250.0;
        for lname in &board.layer_names {
            let _ = writeln!(out, "        (shape");
            let _ = writeln!(
                out,
                "          (circle {} {} 0 0)",
                format_atom(lname),
                scale_i64(dia, um_to_ses)
            );
            let _ = writeln!(out, "        )");
        }
        let _ = writeln!(out, "        (attach off)");
    }
    let _ = writeln!(out, "      )");
}

/// Joins contiguous segments on the same layer & width into compact polylines in O(N) time.
fn chain_segments(segments: &[RoutedSegment]) -> Vec<(usize, f64, Vec<Point>)> {
    let pt_key = |p: Point| -> (i64, i64) {
        ((p.x * 100.0).round() as i64, (p.y * 100.0).round() as i64)
    };
    let mut adj: HashMap<(usize, i64, i64), Vec<usize>> = HashMap::new();
    for (i, s) in segments.iter().enumerate() {
        let ka = pt_key(s.a);
        let kb = pt_key(s.b);
        adj.entry((s.layer, ka.0, ka.1)).or_default().push(i);
        adj.entry((s.layer, kb.0, kb.1)).or_default().push(i);
    }

    let mut out = Vec::new();
    let mut used = vec![false; segments.len()];
    let find_next = |tip: Point, layer: usize, width: f64, used: &[bool]| -> Option<(usize, Point)> {
        let tk = pt_key(tip);
        let cands = adj.get(&(layer, tk.0, tk.1))?;
        for &j in cands {
            if used[j] || (segments[j].width - width).abs() > 0.1 {
                continue;
            }
            if segments[j].a.dist_sq(tip) < 1e-4 {
                return Some((j, segments[j].b));
            }
            if segments[j].b.dist_sq(tip) < 1e-4 {
                return Some((j, segments[j].a));
            }
        }
        None
    };

    for i in 0..segments.len() {
        if used[i] {
            continue;
        }
        used[i] = true;
        let layer = segments[i].layer;
        let width = segments[i].width;
        let mut pts = vec![segments[i].a, segments[i].b];

        while let Some((j, next_pt)) = find_next(*pts.last().unwrap(), layer, width, &used) {
            used[j] = true;
            pts.push(next_pt);
        }
        let mut prefix = Vec::new();
        let mut head = pts[0];
        while let Some((j, prev_pt)) = find_next(head, layer, width, &used) {
            used[j] = true;
            prefix.push(prev_pt);
            head = prev_pt;
        }
        if !prefix.is_empty() {
            prefix.reverse();
            prefix.extend(pts);
            pts = prefix;
        }
        out.push((layer, width, pts));
    }
    out
}

fn scale_i64(val_um: f64, um_to_ses: f64) -> i64 {
    (val_um * um_to_ses).round() as i64
}

fn format_angle(deg: f64) -> String {
    if (deg - deg.round()).abs() < 1e-6 {
        format!("{}", deg.round() as i64)
    } else {
        format!("{deg:.2}")
    }
}

fn base_stem(raw: &str) -> String {
    Path::new(raw)
        .file_stem()
        .and_then(|s| s.to_str())
        .filter(|s| !s.is_empty())
        .unwrap_or("design")
        .to_string()
}

/// Parses a Specctra `.ses` file into per-net routed segments and vias mapped onto `board`.
pub fn parse_ses(input: &str, board: &Board) -> Result<Vec<NetRoute>, String> {
    let root = sexp::parse(input)?;
    if root.tag() != Some("session") {
        return Err(format!("expected top-level (session ...), got {:?}", root.tag()));
    }
    let mut routes: Vec<NetRoute> = (0..board.nets.len())
        .map(|net_id| NetRoute {
            net_id,
            ..Default::default()
        })
        .collect();

    let Some(routes_node) = root.find_child("routes") else {
        return Ok(routes);
    };

    let (res_unit, res_div) = parse_ses_resolution(routes_node);
    let ses_to_um = crate::dsn::unit_scale_to_um(&res_unit) / res_div.max(1e-9);

    // Optional via padstacks defined in `(library_out ...)`.
    let mut lib_vias: HashMap<String, (f64, Vec<usize>)> = HashMap::new();
    if let Some(lib_out) = routes_node.find_child("library_out") {
        for ps_node in lib_out.children_with_tag("padstack") {
            if let Some((name, radius, layers)) = parse_ses_padstack(ps_node, board, ses_to_um) {
                lib_vias.insert(name, (radius, layers));
            }
        }
    }

    let Some(net_out) = routes_node.find_child("network_out") else {
        return Ok(routes);
    };

    for net_node in net_out.children_with_tag("net") {
        let Some(net_name) = net_node.as_list().and_then(|l| l.get(1)?.as_atom()) else {
            continue;
        };
        let Some(&net_id) = board.net_name_to_id.get(net_name) else {
            continue;
        };
        if board.nets[net_id].ignored {
            continue;
        }
        let nr = &mut routes[net_id];
        let rules = &board.nets[net_id].rules;

        for wire_node in net_node.children_with_tag("wire") {
            let Some(path) = wire_node.find_child("path").and_then(SExp::as_list) else {
                continue;
            };
            let Some(layer_name) = path.get(1).and_then(SExp::as_atom) else {
                continue;
            };
            let Some(&layer) = board.layer_name_to_idx.get(layer_name) else {
                continue;
            };
            let width = path
                .get(2)
                .and_then(SExp::as_f64)
                .map(|w| w * ses_to_um)
                .unwrap_or(rules.width);
            let pts = parse_ses_points(path.get(3..).unwrap_or(&[]), ses_to_um);
            for pair in pts.windows(2) {
                if pair[0].dist(pair[1]) > 0.5 {
                    nr.segments.push(RoutedSegment {
                        layer,
                        a: pair[0],
                        b: pair[1],
                        width,
                    });
                }
            }
        }

        for via_node in net_node.children_with_tag("via") {
            let Some(list) = via_node.as_list() else {
                continue;
            };
            let Some(padstack) = list.get(1).and_then(SExp::as_atom) else {
                continue;
            };
            let (Some(x), Some(y)) = (
                list.get(2).and_then(SExp::as_f64),
                list.get(3).and_then(SExp::as_f64),
            ) else {
                continue;
            };
            let pos = Point::new(x * ses_to_um, y * ses_to_um);
            let (radius, drill, layers) = if padstack == rules.via_padstack {
                (rules.via_radius, rules.via_drill, rules.via_layers.clone())
            } else if let Some((r, ls)) = lib_vias.get(padstack) {
                let drill = parse_via_drill_hint(padstack).unwrap_or(r * 1.2);
                (*r, drill, ls.clone())
            } else {
                (rules.via_radius, rules.via_drill, rules.via_layers.clone())
            };
            nr.vias.push(RoutedVia {
                pos,
                padstack: padstack.to_string(),
                radius,
                drill,
                layers,
            });
        }
    }

    Ok(routes)
}

fn parse_ses_resolution(routes_node: &SExp) -> (String, f64) {
    if let Some(res) = routes_node.find_child("resolution").and_then(SExp::as_list) {
        let unit = res.get(1).and_then(SExp::as_atom).unwrap_or("um").to_string();
        let div = res.get(2).and_then(SExp::as_f64).unwrap_or(10.0);
        return (unit, div);
    }
    ("um".to_string(), 10.0)
}

fn parse_ses_padstack(
    node: &SExp,
    board: &Board,
    ses_to_um: f64,
) -> Option<(String, f64, Vec<usize>)> {
    let name = node.as_list()?.get(1)?.as_atom()?.to_string();
    let mut max_r = 0.0_f64;
    let mut layers = Vec::new();
    for sh in node.children_with_tag("shape") {
        let Some(inner) = sh.as_list().and_then(|l| l.get(1)?.as_list()) else {
            continue;
        };
        if inner.first().and_then(SExp::as_atom) != Some("circle") {
            continue;
        }
        let Some(lname) = inner.get(1).and_then(SExp::as_atom) else {
            continue;
        };
        if let Some(dia) = inner.get(2).and_then(SExp::as_f64) {
            max_r = max_r.max(dia * 0.5 * ses_to_um);
        }
        if let Some(&lidx) = board.layer_name_to_idx.get(lname) {
            layers.push(lidx);
        }
    }
    layers.sort_unstable();
    layers.dedup();
    if layers.is_empty() {
        layers.extend(0..board.num_layers());
    }
    if max_r > 0.0 {
        Some((name, max_r, layers))
    } else {
        None
    }
}

fn parse_ses_points(slice: &[SExp], ses_to_um: f64) -> Vec<Point> {
    let mut pts = Vec::with_capacity(slice.len() / 2);
    let mut i = 0;
    while i + 1 < slice.len() {
        if let (Some(x), Some(y)) = (slice[i].as_f64(), slice[i + 1].as_f64()) {
            pts.push(Point::new(x * ses_to_um, y * ses_to_um));
        }
        i += 2;
    }
    pts
}

fn parse_via_drill_hint(name: &str) -> Option<f64> {
    let after_us = name.split('_').nth(1)?;
    after_us.split(':').nth(1)?.parse::<f64>().ok()
}
