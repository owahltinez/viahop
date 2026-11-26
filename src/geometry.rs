//! Exact 2D computational geometry for PCB clearance, shapes, and transforms.

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn dist_sq(self, other: Self) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        dx * dx + dy * dy
    }

    pub fn dist(self, other: Self) -> f64 {
        self.dist_sq(other).sqrt()
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BBox {
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64,
}

impl BBox {
    pub const fn new(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Self {
        Self {
            min_x,
            min_y,
            max_x,
            max_y,
        }
    }

    pub fn from_point(p: Point, margin: f64) -> Self {
        Self::new(p.x - margin, p.y - margin, p.x + margin, p.y + margin)
    }

    pub fn from_points(pts: &[Point], margin: f64) -> Self {
        let mut min_x = f64::INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        for p in pts {
            min_x = min_x.min(p.x);
            min_y = min_y.min(p.y);
            max_x = max_x.max(p.x);
            max_y = max_y.max(p.y);
        }
        Self::new(
            min_x - margin,
            min_y - margin,
            max_x + margin,
            max_y + margin,
        )
    }

    pub fn expand(self, margin: f64) -> Self {
        Self::new(
            self.min_x - margin,
            self.min_y - margin,
            self.max_x + margin,
            self.max_y + margin,
        )
    }

    pub fn union(self, other: Self) -> Self {
        Self::new(
            self.min_x.min(other.min_x),
            self.min_y.min(other.min_y),
            self.max_x.max(other.max_x),
            self.max_y.max(other.max_y),
        )
    }

    pub fn intersects(self, other: Self) -> bool {
        self.min_x <= other.max_x
            && self.max_x >= other.min_x
            && self.min_y <= other.max_y
            && self.max_y >= other.min_y
    }
}

/// Exact copper or keepout geometry on a single PCB layer (units: micrometers).
#[derive(Debug, Clone)]
pub enum Shape {
    Circle {
        center: Point,
        radius: f64,
    },
    /// Swept line segment (covers Specctra `path` pad shapes and trace segments).
    Capsule {
        a: Point,
        b: Point,
        radius: f64,
    },
    /// Closed polygon (covers `rect`, rotated `rect`, and arbitrary `polygon`).
    Poly {
        verts: Vec<Point>,
        bbox: BBox,
    },
}

impl Shape {
    pub fn rect(x1: f64, y1: f64, x2: f64, y2: f64) -> Self {
        let min_x = x1.min(x2);
        let max_x = x1.max(x2);
        let min_y = y1.min(y2);
        let max_y = y1.max(y2);
        let verts = vec![
            Point::new(min_x, min_y),
            Point::new(max_x, min_y),
            Point::new(max_x, max_y),
            Point::new(min_x, max_y),
        ];
        Self::Poly {
            bbox: BBox::new(min_x, min_y, max_x, max_y),
            verts,
        }
    }

    pub fn polygon(mut verts: Vec<Point>) -> Self {
        if verts.len() > 1 && verts.first() == verts.last() {
            verts.pop();
        }
        let bbox = BBox::from_points(&verts, 0.0);
        Self::Poly { verts, bbox }
    }

    pub fn bbox(&self) -> BBox {
        match self {
            Self::Circle { center, radius } => BBox::from_point(*center, *radius),
            Self::Capsule { a, b, radius } => BBox::from_points(&[*a, *b], *radius),
            Self::Poly { bbox, .. } => *bbox,
        }
    }

    /// Returns the copper-to-copper gap between this shape and a circle.
    /// Negative or zero indicates overlap/touching.
    pub fn dist_to_circle(&self, center: Point, radius: f64) -> f64 {
        match self {
            Self::Circle {
                center: c,
                radius: r,
            } => c.dist(center) - r - radius,
            Self::Capsule { a, b, radius: r } => point_to_segment_dist(center, *a, *b) - r - radius,
            Self::Poly { verts, .. } => point_to_polygon_dist(center, verts) - radius,
        }
    }

    /// Returns the copper-to-copper gap between this shape and a swept segment (capsule).
    pub fn dist_to_capsule(&self, a: Point, b: Point, half_w: f64) -> f64 {
        match self {
            Self::Circle { center, radius } => {
                point_to_segment_dist(*center, a, b) - radius - half_w
            }
            Self::Capsule {
                a: sa,
                b: sb,
                radius,
            } => segment_to_segment_dist(*sa, *sb, a, b) - radius - half_w,
            Self::Poly { verts, .. } => segment_to_polygon_dist(a, b, verts) - half_w,
        }
    }

    /// Minimum distance from this shape's copper to the edges of polygon `verts`.
    pub fn dist_to_polygon_edge(&self, verts: &[Point]) -> f64 {
        match self {
            Self::Circle { center, radius } => point_to_polygon_edge_dist(*center, verts) - radius,
            Self::Capsule { a, b, radius } => segment_to_polygon_edge_dist(*a, *b, verts) - radius,
            Self::Poly { verts: pv, .. } => match pv.len() {
                0 => f64::INFINITY,
                1 => point_to_polygon_edge_dist(pv[0], verts),
                _ => (0..pv.len())
                    .map(|i| segment_to_polygon_edge_dist(pv[i], pv[(i + 1) % pv.len()], verts))
                    .fold(f64::INFINITY, f64::min),
            },
        }
    }

    /// Transforms a shape in component-local coordinates to world coordinates.
    pub fn transform(&self, flip_x: bool, rot_deg: f64, origin: Point) -> Self {
        match self {
            Self::Circle { center, radius } => Self::Circle {
                center: transform_point(*center, flip_x, rot_deg, origin),
                radius: *radius,
            },
            Self::Capsule { a, b, radius } => Self::Capsule {
                a: transform_point(*a, flip_x, rot_deg, origin),
                b: transform_point(*b, flip_x, rot_deg, origin),
                radius: *radius,
            },
            Self::Poly { verts, .. } => {
                let tv: Vec<Point> = verts
                    .iter()
                    .map(|&p| transform_point(p, flip_x, rot_deg, origin))
                    .collect();
                Self::polygon(tv)
            }
        }
    }
}

/// Applies Specctra component placement transform: optional back-side X mirror,
/// then CCW rotation by `rot_deg`, then translation by `origin`.
pub fn transform_point(p: Point, flip_x: bool, rot_deg: f64, origin: Point) -> Point {
    let lx = if flip_x { -p.x } else { p.x };
    let ly = p.y;
    let (s, c) = rot_deg.to_radians().sin_cos();
    // Snap exact right-angle trig to avoid tiny floating-point drift on 0/90/180/270.
    let (s, c) = snap_trig(s, c);
    Point::new(origin.x + lx * c - ly * s, origin.y + lx * s + ly * c)
}

fn snap_trig(s: f64, c: f64) -> (f64, f64) {
    let snap = |v: f64| {
        if v.abs() < 1e-12 {
            0.0
        } else if (v - 1.0).abs() < 1e-12 {
            1.0
        } else if (v + 1.0).abs() < 1e-12 {
            -1.0
        } else {
            v
        }
    };
    (snap(s), snap(c))
}

pub fn point_to_segment_dist_sq(p: Point, a: Point, b: Point) -> f64 {
    let abx = b.x - a.x;
    let aby = b.y - a.y;
    let len_sq = abx * abx + aby * aby;
    if len_sq <= 1e-18 {
        return p.dist_sq(a);
    }
    let t = (((p.x - a.x) * abx + (p.y - a.y) * aby) / len_sq).clamp(0.0, 1.0);
    let proj = Point::new(a.x + t * abx, a.y + t * aby);
    p.dist_sq(proj)
}

pub fn point_to_segment_dist(p: Point, a: Point, b: Point) -> f64 {
    point_to_segment_dist_sq(p, a, b).sqrt()
}

pub fn segments_intersect(a1: Point, a2: Point, b1: Point, b2: Point) -> bool {
    let orient =
        |p: Point, q: Point, r: Point| (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
    let on_seg = |p: Point, q: Point, r: Point| {
        q.x <= p.x.max(r.x) + 1e-9
            && q.x >= p.x.min(r.x) - 1e-9
            && q.y <= p.y.max(r.y) + 1e-9
            && q.y >= p.y.min(r.y) - 1e-9
    };

    let o1 = orient(a1, a2, b1);
    let o2 = orient(a1, a2, b2);
    let o3 = orient(b1, b2, a1);
    let o4 = orient(b1, b2, a2);

    if (o1 > 0.0) != (o2 > 0.0) && (o3 > 0.0) != (o4 > 0.0) {
        return true;
    }
    if o1.abs() < 1e-9 && on_seg(a1, b1, a2) {
        return true;
    }
    if o2.abs() < 1e-9 && on_seg(a1, b2, a2) {
        return true;
    }
    if o3.abs() < 1e-9 && on_seg(b1, a1, b2) {
        return true;
    }
    if o4.abs() < 1e-9 && on_seg(b1, a2, b2) {
        return true;
    }
    false
}

pub fn segment_to_segment_dist(a1: Point, a2: Point, b1: Point, b2: Point) -> f64 {
    if segments_intersect(a1, a2, b1, b2) {
        return 0.0;
    }
    let d1 = point_to_segment_dist_sq(a1, b1, b2);
    let d2 = point_to_segment_dist_sq(a2, b1, b2);
    let d3 = point_to_segment_dist_sq(b1, a1, a2);
    let d4 = point_to_segment_dist_sq(b2, a1, a2);
    d1.min(d2).min(d3).min(d4).sqrt()
}

pub fn point_in_polygon(p: Point, verts: &[Point]) -> bool {
    let n = verts.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let vi = verts[i];
        let vj = verts[j];
        let intersects = ((vi.y > p.y) != (vj.y > p.y))
            && (p.x < (vj.x - vi.x) * (p.y - vi.y) / (vj.y - vi.y) + vi.x);
        if intersects {
            inside = !inside;
        }
        j = i;
    }
    inside
}

pub fn point_to_polygon_dist(p: Point, verts: &[Point]) -> f64 {
    if verts.is_empty() {
        return f64::INFINITY;
    }
    if point_in_polygon(p, verts) {
        return 0.0;
    }
    let n = verts.len();
    let mut min_sq = f64::INFINITY;
    for i in 0..n {
        let a = verts[i];
        let b = verts[(i + 1) % n];
        min_sq = min_sq.min(point_to_segment_dist_sq(p, a, b));
    }
    min_sq.sqrt()
}

/// Minimum distance from point `p` to the boundary edges of polygon `verts`.
pub fn point_to_polygon_edge_dist(p: Point, verts: &[Point]) -> f64 {
    let n = verts.len();
    if n < 2 {
        return f64::INFINITY;
    }
    let mut min_sq = f64::INFINITY;
    for i in 0..n {
        let a = verts[i];
        let b = verts[(i + 1) % n];
        min_sq = min_sq.min(point_to_segment_dist_sq(p, a, b));
    }
    min_sq.sqrt()
}

/// Returns the closest point on the boundary edges of polygon `verts` to `p`.
pub fn closest_point_on_polygon_edge(p: Point, verts: &[Point]) -> Option<Point> {
    let n = verts.len();
    if n < 2 {
        return None;
    }
    let mut best_sq = f64::INFINITY;
    let mut best_pt = verts[0];
    for i in 0..n {
        let a = verts[i];
        let b = verts[(i + 1) % n];
        let abx = b.x - a.x;
        let aby = b.y - a.y;
        let len_sq = abx * abx + aby * aby;
        let proj = if len_sq <= 1e-18 {
            a
        } else {
            let t = (((p.x - a.x) * abx + (p.y - a.y) * aby) / len_sq).clamp(0.0, 1.0);
            Point::new(a.x + t * abx, a.y + t * aby)
        };
        let d_sq = p.dist_sq(proj);
        if d_sq < best_sq {
            best_sq = d_sq;
            best_pt = proj;
        }
    }
    Some(best_pt)
}

/// Minimum distance from segment `(a, b)` to the boundary edges of polygon `verts`.
pub fn segment_to_polygon_edge_dist(a: Point, b: Point, verts: &[Point]) -> f64 {
    let n = verts.len();
    if n < 2 {
        return f64::INFINITY;
    }
    let mut min_d = f64::INFINITY;
    for i in 0..n {
        let e1 = verts[i];
        let e2 = verts[(i + 1) % n];
        let d = segment_to_segment_dist(a, b, e1, e2);
        if d == 0.0 {
            return 0.0;
        }
        min_d = min_d.min(d);
    }
    min_d
}

pub fn segment_to_polygon_dist(a: Point, b: Point, verts: &[Point]) -> f64 {
    match verts.len() {
        0 => f64::INFINITY,
        1 => point_to_segment_dist(verts[0], a, b),
        _ if point_in_polygon(a, verts) || point_in_polygon(b, verts) => 0.0,
        _ => segment_to_polygon_edge_dist(a, b, verts),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn back_side_transform_matches_kicad() {
        // R27: back 180 at (20400, -12200), local pin1=(-510, 0) -> world (19890, -12200)
        let p = transform_point(
            Point::new(-510.0, 0.0),
            true,
            180.0,
            Point::new(20400.0, -12200.0),
        );
        assert!((p.x - 19890.0).abs() < 1e-6);
        assert!((p.y + 12200.0).abs() < 1e-6);

        // R35: back 270 at (30200, -30500), local pin1=(-510, 0) -> world (30200, -31010)
        let p2 = transform_point(
            Point::new(-510.0, 0.0),
            true,
            270.0,
            Point::new(30200.0, -30500.0),
        );
        assert!((p2.x - 30200.0).abs() < 1e-6);
        assert!((p2.y + 31010.0).abs() < 1e-6);
    }

    #[test]
    fn exact_segment_and_polygon_clearances() {
        let r = Shape::rect(0.0, 0.0, 100.0, 100.0);
        assert!((r.dist_to_circle(Point::new(150.0, 50.0), 20.0) - 30.0).abs() < 1e-6);
        assert!(r.dist_to_circle(Point::new(50.0, 50.0), 10.0) < 0.0);
        let d = r.dist_to_capsule(Point::new(130.0, -20.0), Point::new(130.0, 120.0), 10.0);
        assert!((d - 20.0).abs() < 1e-6);
    }
}
