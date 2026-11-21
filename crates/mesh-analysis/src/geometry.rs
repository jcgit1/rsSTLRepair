//! Parry BVH broadphase plus exact `f64` narrowphase for self-intersections,
//! T-junctions, and thin walls.

use mesh_core::{EdgeKey, FaceId, TriangleMesh, VertexId};
use nalgebra::{Point3, Vector3};
use parry3d::bounding_volume::Aabb;
use parry3d::math::Point as ParryPoint;
use parry3d::partitioning::{Bvh, BvhBuildStrategy};
use serde::{Deserialize, Serialize};

const ABS_EPS: f64 = 1e-8;
const LENGTH_EPS: f64 = 1e-7;

/// Two triangles whose interiors cross or overlap in the plane.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct SelfIntersection {
    pub face_a: FaceId,
    pub face_b: FaceId,
}

/// A vertex lying in the interior of an edge that does not list that vertex.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct TJunction {
    pub vertex: VertexId,
    pub edge: EdgeKey,
    pub host_face: FaceId,
}

/// Opposite, nearly parallel faces closer than a print-scale gap.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ThinWall {
    pub face_a: FaceId,
    pub face_b: FaceId,
    pub distance: f64,
}

/// Geometric findings for one mesh. Broadphase pairs come from a `parry3d` BVH.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GeometryScan {
    pub self_intersections: Vec<SelfIntersection>,
    pub t_junctions: Vec<TJunction>,
    pub thin_walls: Vec<ThinWall>,
}

/// Run the Phase 2 geometric scan. Non-finite and degenerate faces are skipped.
#[must_use]
pub fn scan_geometry(mesh: &TriangleMesh) -> GeometryScan {
    let tris = finite_triangles(mesh);
    let bvh = build_bvh(mesh, &tris);
    let mut self_intersections = Vec::new();
    let mut thin_walls = Vec::new();

    if let Some(bvh) = &bvh {
        let mut workspace = parry3d::partitioning::BvhWorkspace::default();
        bvh.traverse_bvtt_single_tree::<false>(&mut workspace, &mut |ia, ib| {
            let (ia, ib) = ordered_pair(ia, ib);
            let Some(ta) = tris.get(ia as usize).copied().flatten() else {
                return;
            };
            let Some(tb) = tris.get(ib as usize).copied().flatten() else {
                return;
            };
            if shares_edge(ta.idx, tb.idx) {
                return;
            }
            if interiors_intersect(ta.pos, tb.pos) {
                self_intersections.push(SelfIntersection {
                    face_a: FaceId(ia),
                    face_b: FaceId(ib),
                });
            } else if let Some(distance) = thin_wall_distance(ta.pos, tb.pos, mesh) {
                thin_walls.push(ThinWall {
                    face_a: FaceId(ia),
                    face_b: FaceId(ib),
                    distance,
                });
            }
        });
    }

    self_intersections.sort_by_key(|s| (s.face_a.0, s.face_b.0));
    self_intersections.dedup();
    thin_walls.sort_by(|a, b| {
        (a.face_a.0, a.face_b.0)
            .cmp(&(b.face_a.0, b.face_b.0))
            .then(a.distance.total_cmp(&b.distance))
    });
    thin_walls.dedup_by_key(|t| (t.face_a.0, t.face_b.0));

    let t_junctions = find_t_junctions(mesh, bvh.as_ref(), &tris);

    GeometryScan {
        self_intersections,
        t_junctions,
        thin_walls,
    }
}

/// Where `face` crosses the plane of `other`, clipped to `face`.
///
/// Endpoints lie on `face`'s boundary. `None` when the triangles miss or are coplanar.
#[must_use]
pub fn boundary_cut(
    mesh: &TriangleMesh,
    face: FaceId,
    other: FaceId,
) -> Option<(Point3<f64>, Point3<f64>)> {
    let a = mesh.face_positions(face)?;
    let b = mesh.face_positions(other)?;
    if !all_finite(&a) || !all_finite(&b) {
        return None;
    }
    let pa = plane_of(a)?;
    let pb = plane_of(b)?;
    let da = distances(pb, a);
    let db = distances(pa, b);
    if !straddles(da) || !straddles(db) {
        return None;
    }
    let dir = pa.n.cross(&pb.n);
    if dir.norm_squared() < 1e-20 {
        return None;
    }
    let (p, q) = plane_cut_segment(a, da, dir)?;
    let (bp, bq) = plane_cut_segment(b, db, dir)?;
    let (s, e) = overlap_segments(p, q, bp, bq, dir)?;
    if (s - e).norm() <= LENGTH_EPS {
        return None;
    }
    if (p - q).norm() <= LENGTH_EPS {
        return None;
    }
    Some((p, q))
}

/// Intersection segment of two triangle interiors, if they cross in a segment long enough to split.
#[must_use]
pub fn crossing_segment(
    mesh: &TriangleMesh,
    face_a: FaceId,
    face_b: FaceId,
) -> Option<(Point3<f64>, Point3<f64>)> {
    let a = mesh.face_positions(face_a)?;
    let b = mesh.face_positions(face_b)?;
    if !all_finite(&a) || !all_finite(&b) {
        return None;
    }
    match crossing_segment_points(a, b) {
        Cross::Segment(p, q) if (p - q).norm() > LENGTH_EPS => Some((p, q)),
        _ => None,
    }
}

#[derive(Clone, Copy)]
struct Tri {
    idx: [u32; 3],
    pos: [Point3<f64>; 3],
}

fn finite_triangles(mesh: &TriangleMesh) -> Vec<Option<Tri>> {
    (0..mesh.faces.len())
        .map(|i| {
            let idx = mesh.faces[i];
            let pos = mesh.face_positions(FaceId(i as u32))?;
            if !all_finite(&pos) || TriangleMesh::triangle_area(pos[0], pos[1], pos[2]) <= 1e-16 {
                return None;
            }
            Some(Tri { idx, pos })
        })
        .collect()
}

fn build_bvh(mesh: &TriangleMesh, tris: &[Option<Tri>]) -> Option<Bvh> {
    let inflate = thin_gap(mesh) as f32 + 1.0e-4;
    let mut leaves = Vec::new();
    for (i, tri) in tris.iter().enumerate() {
        let Some(tri) = tri else { continue };
        let mut min = tri.pos[0];
        let mut max = tri.pos[0];
        for p in &tri.pos[1..] {
            min.x = min.x.min(p.x);
            min.y = min.y.min(p.y);
            min.z = min.z.min(p.z);
            max.x = max.x.max(p.x);
            max.y = max.y.max(p.y);
            max.z = max.z.max(p.z);
        }
        let aabb = Aabb::new(
            ParryPoint::new(
                (min.x as f32) - inflate,
                (min.y as f32) - inflate,
                (min.z as f32) - inflate,
            ),
            ParryPoint::new(
                (max.x as f32) + inflate,
                (max.y as f32) + inflate,
                (max.z as f32) + inflate,
            ),
        );
        leaves.push((i, aabb));
    }
    if leaves.len() < 2 {
        return None;
    }
    Some(Bvh::from_iter(BvhBuildStrategy::Binned, leaves))
}

fn find_t_junctions(
    mesh: &TriangleMesh,
    bvh: Option<&Bvh>,
    tris: &[Option<Tri>],
) -> Vec<TJunction> {
    let Some(bvh) = bvh else {
        return Vec::new();
    };
    let mut found = Vec::new();
    let mut seen = std::collections::BTreeSet::new();
    for (vi, p) in mesh.vertices.iter().enumerate() {
        if !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite() {
            continue;
        }
        let q = ParryPoint::new(p.x as f32, p.y as f32, p.z as f32);
        let pad = 1.0e-4;
        let query = Aabb::new(
            ParryPoint::new(q.x - pad, q.y - pad, q.z - pad),
            ParryPoint::new(q.x + pad, q.y + pad, q.z + pad),
        );
        for face in bvh.intersect_aabb(&query) {
            let Some(tri) = tris.get(face as usize).and_then(|t| t.as_ref()) else {
                continue;
            };
            if tri.idx.contains(&(vi as u32)) {
                continue;
            }
            for (u, v) in [(0, 1), (1, 2), (2, 0)] {
                let a = tri.pos[u];
                let b = tri.pos[v];
                let Some(t) = point_on_segment_interior(*p, a, b) else {
                    continue;
                };
                let edge = EdgeKey::new(tri.idx[u], tri.idx[v]);
                if seen.insert((vi as u32, edge.0, edge.1)) {
                    let _ = t;
                    found.push(TJunction {
                        vertex: VertexId(vi as u32),
                        edge,
                        host_face: FaceId(face),
                    });
                }
            }
        }
    }
    found.sort_by_key(|t| (t.vertex.0, t.edge.0, t.edge.1, t.host_face.0));
    found
}

fn point_on_segment_interior(p: Point3<f64>, a: Point3<f64>, b: Point3<f64>) -> Option<f64> {
    let ab = b - a;
    let len2 = ab.norm_squared();
    if len2 < 1e-20 {
        return None;
    }
    let t = (p - a).dot(&ab) / len2;
    if !(t > 1e-4 && t < 1.0 - 1e-4) {
        return None;
    }
    let closest = a + ab * t;
    let tol = (ABSOLUTE_EDGE_TOL).max(1e-8 * len2.sqrt());
    if (p - closest).norm() <= tol {
        Some(t)
    } else {
        None
    }
}

const ABSOLUTE_EDGE_TOL: f64 = 1e-7;

fn ordered_pair(a: u32, b: u32) -> (u32, u32) {
    if a < b { (a, b) } else { (b, a) }
}

fn shares_edge(a: [u32; 3], b: [u32; 3]) -> bool {
    let mut shared = 0;
    for va in a {
        if b.contains(&va) {
            shared += 1;
        }
    }
    shared >= 2
}

fn all_finite(pts: &[Point3<f64>; 3]) -> bool {
    pts.iter()
        .all(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite())
}

enum Cross {
    None,
    Segment(Point3<f64>, Point3<f64>),
    Coplanar,
}

fn interiors_intersect(a: [Point3<f64>; 3], b: [Point3<f64>; 3]) -> bool {
    match crossing_segment_points(a, b) {
        Cross::Segment(p, q) => {
            if (p - q).norm() <= LENGTH_EPS {
                return false;
            }
            let on_a = segment_on_one_edge(a, p, q);
            let on_b = segment_on_one_edge(b, p, q);
            // A segment that lies on either triangle's boundary is a touch or a
            // T-junction, not an interior crossing.
            !(on_a || on_b)
        }
        Cross::Coplanar => coplanar_interiors(a, b),
        Cross::None => false,
    }
}

fn crossing_segment_points(a: [Point3<f64>; 3], b: [Point3<f64>; 3]) -> Cross {
    let Some(pa) = plane_of(a) else {
        return Cross::None;
    };
    let Some(pb) = plane_of(b) else {
        return Cross::None;
    };
    let da = distances(pb, a);
    let db = distances(pa, b);
    if da.iter().all(|d| d.abs() <= ABS_EPS) && db.iter().all(|d| d.abs() <= ABS_EPS) {
        return Cross::Coplanar;
    }
    if !straddles(db) || !straddles(da) {
        return Cross::None;
    }
    let dir = pa.n.cross(&pb.n);
    if dir.norm_squared() < 1e-20 {
        return Cross::Coplanar;
    }
    let Some((a0, a1)) = plane_cut_segment(a, da, dir) else {
        return Cross::None;
    };
    let Some((b0, b1)) = plane_cut_segment(b, db, dir) else {
        return Cross::None;
    };
    overlap_segments(a0, a1, b0, b1, dir)
        .map(|(p, q)| Cross::Segment(p, q))
        .unwrap_or(Cross::None)
}

#[derive(Clone, Copy)]
struct Plane {
    n: Vector3<f64>,
    d: f64,
}

fn plane_of(tri: [Point3<f64>; 3]) -> Option<Plane> {
    let n = (tri[1] - tri[0]).cross(&(tri[2] - tri[0]));
    let len = n.norm();
    if len < 1e-16 {
        return None;
    }
    let n = n / len;
    Some(Plane {
        d: -n.dot(&tri[0].coords),
        n,
    })
}

fn distances(plane: Plane, tri: [Point3<f64>; 3]) -> [f64; 3] {
    tri.map(|p| plane.n.dot(&p.coords) + plane.d)
}

fn straddles(d: [f64; 3]) -> bool {
    let pos = d.iter().any(|v| *v > ABS_EPS);
    let neg = d.iter().any(|v| *v < -ABS_EPS);
    pos && neg
}

fn plane_cut_segment(
    tri: [Point3<f64>; 3],
    dist: [f64; 3],
    dir: Vector3<f64>,
) -> Option<(Point3<f64>, Point3<f64>)> {
    let mut hits = Vec::with_capacity(2);
    for (i, j) in [(0, 1), (1, 2), (2, 0)] {
        let di = dist[i];
        let dj = dist[j];
        if di.abs() <= ABS_EPS && dj.abs() <= ABS_EPS {
            continue;
        }
        if di.abs() <= ABS_EPS {
            hits.push(tri[i]);
        } else if di * dj < 0.0 {
            let t = di / (di - dj);
            hits.push(tri[i] + (tri[j] - tri[i]) * t);
        }
    }
    if hits.len() < 2 {
        return None;
    }
    hits.sort_by(|p, q| p.coords.dot(&dir).total_cmp(&q.coords.dot(&dir)));
    Some((hits[0], *hits.last().unwrap()))
}

fn overlap_segments(
    a0: Point3<f64>,
    a1: Point3<f64>,
    b0: Point3<f64>,
    b1: Point3<f64>,
    dir: Vector3<f64>,
) -> Option<(Point3<f64>, Point3<f64>)> {
    let proj = |p: Point3<f64>| p.coords.dot(&dir);
    let (mut a0, mut a1) = (a0, a1);
    if proj(a0) > proj(a1) {
        std::mem::swap(&mut a0, &mut a1);
    }
    let (mut b0, mut b1) = (b0, b1);
    if proj(b0) > proj(b1) {
        std::mem::swap(&mut b0, &mut b1);
    }
    let start = if proj(a0) > proj(b0) { a0 } else { b0 };
    let end = if proj(a1) < proj(b1) { a1 } else { b1 };
    if proj(end) - proj(start) > LENGTH_EPS * dir.norm() {
        Some((start, end))
    } else {
        None
    }
}

fn segment_on_one_edge(tri: [Point3<f64>; 3], p: Point3<f64>, q: Point3<f64>) -> bool {
    for (i, j) in [(0, 1), (1, 2), (2, 0)] {
        if point_on_segment_closed(p, tri[i], tri[j]) && point_on_segment_closed(q, tri[i], tri[j])
        {
            return true;
        }
    }
    false
}

fn point_on_segment_closed(p: Point3<f64>, a: Point3<f64>, b: Point3<f64>) -> bool {
    let ab = b - a;
    let len2 = ab.norm_squared();
    if len2 < 1e-20 {
        return (p - a).norm() <= ABS_EPS;
    }
    let t = (p - a).dot(&ab) / len2;
    if !(-1e-5..=1.0 + 1e-5).contains(&t) {
        return false;
    }
    let t = t.clamp(0.0, 1.0);
    (p - (a + ab * t)).norm() <= 1e-6
}

fn coplanar_interiors(a: [Point3<f64>; 3], b: [Point3<f64>; 3]) -> bool {
    let Some(plane) = plane_of(a) else {
        return false;
    };
    for p in b {
        if strictly_inside(p, a, plane.n) {
            return true;
        }
    }
    for p in a {
        if strictly_inside(p, b, plane.n) {
            return true;
        }
    }
    let (axis, u, v) = dominant_axes(plane.n);
    let _ = axis;
    for (i, j) in [(0, 1), (1, 2), (2, 0)] {
        for (k, l) in [(0, 1), (1, 2), (2, 0)] {
            if segments_cross_proper_2d(
                [a[i][u], a[i][v]],
                [a[j][u], a[j][v]],
                [b[k][u], b[k][v]],
                [b[l][u], b[l][v]],
            ) {
                return true;
            }
        }
    }
    false
}

fn strictly_inside(p: Point3<f64>, tri: [Point3<f64>; 3], _normal: Vector3<f64>) -> bool {
    let v0 = tri[1] - tri[0];
    let v1 = tri[2] - tri[0];
    let v2 = p - tri[0];
    let d00 = v0.dot(&v0);
    let d01 = v0.dot(&v1);
    let d11 = v1.dot(&v1);
    let d20 = v2.dot(&v0);
    let d21 = v2.dot(&v1);
    let denom = d00 * d11 - d01 * d01;
    if denom.abs() < 1e-18 {
        return false;
    }
    let v = (d11 * d20 - d01 * d21) / denom;
    let w = (d00 * d21 - d01 * d20) / denom;
    let u = 1.0 - v - w;
    u > 1e-6 && v > 1e-6 && w > 1e-6
}

fn dominant_axes(n: Vector3<f64>) -> (usize, usize, usize) {
    let ax = n.x.abs();
    let ay = n.y.abs();
    let az = n.z.abs();
    if ax >= ay && ax >= az {
        (0, 1, 2)
    } else if ay >= ax && ay >= az {
        (1, 0, 2)
    } else {
        (2, 0, 1)
    }
}

fn segments_cross_proper_2d(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> bool {
    fn orient(p: [f64; 2], q: [f64; 2], r: [f64; 2]) -> f64 {
        (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    }
    let o1 = orient(a, b, c);
    let o2 = orient(a, b, d);
    let o3 = orient(c, d, a);
    let o4 = orient(c, d, b);
    o1 * o2 < -1e-14 && o3 * o4 < -1e-14
}

fn thin_gap(mesh: &TriangleMesh) -> f64 {
    let diag = mesh.bounding_box().map(|b| b.diagonal()).unwrap_or(1.0);
    (diag * 0.01).clamp(1e-4, 0.35)
}

fn thin_wall_distance(
    a: [Point3<f64>; 3],
    b: [Point3<f64>; 3],
    mesh: &TriangleMesh,
) -> Option<f64> {
    let pa = plane_of(a)?;
    let pb = plane_of(b)?;
    if pa.n.dot(&pb.n) > -0.98 {
        return None;
    }
    let gap = thin_gap(mesh);
    let centroid =
        |t: [Point3<f64>; 3]| Point3::from((t[0].coords + t[1].coords + t[2].coords) / 3.0);
    let ca = centroid(a);
    let cb = centroid(b);
    let dist = (pa.n.dot(&cb.coords) + pa.d).abs();
    if dist > gap || dist <= ABS_EPS {
        return None;
    }
    let projected_b = cb - pa.n * (pa.n.dot(&cb.coords) + pa.d);
    let projected_a = ca - pb.n * (pb.n.dot(&ca.coords) + pb.d);
    if strictly_inside(projected_b, a, pa.n) && strictly_inside(projected_a, b, pb.n) {
        Some(dist)
    } else {
        None
    }
}
