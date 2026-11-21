//! Phase 2 repairs: T-junction stitching, crossing splits, and closed-shell union.

use super::{OpOutput, RepairOp};
use mesh_analysis::{DiagnosticReport, boundary_cut, scan_geometry};
use mesh_attributes::{FaceOrigin, compose_origins};
use mesh_core::{EdgeKey, FaceId, MeshError, TriangleMesh, VertexId};
use nalgebra::{Point3, Vector3};
use std::collections::{BTreeMap, VecDeque};

pub(crate) struct StitchTJunctionsOp;

impl RepairOp for StitchTJunctionsOp {
    fn name(&self) -> &'static str {
        "Stitch T-junctions"
    }

    fn apply(&self, mesh: &TriangleMesh, report: &DiagnosticReport) -> Result<OpOutput, MeshError> {
        if report.t_junctions.is_empty() {
            return Ok(unchanged(mesh, "none"));
        }
        let mut current = mesh.clone();
        let mut origins = identity(mesh.faces.len());
        let mut stitched = 0usize;
        for _ in 0..12 {
            let scan = scan_geometry(&current);
            if scan.t_junctions.is_empty() {
                break;
            }
            let edge = scan.t_junctions[0].edge;
            let mut mids: Vec<VertexId> = scan
                .t_junctions
                .iter()
                .filter(|t| t.edge == edge)
                .map(|t| t.vertex)
                .collect();
            mids.sort_by(|&a, &b| {
                edge_param(&current, edge, a).total_cmp(&edge_param(&current, edge, b))
            });
            mids.dedup();
            let (next, step) = split_edge(&current, edge, &mids);
            if next == current {
                break;
            }
            stitched += mids.len();
            origins = compose_origins(&origins, &step);
            current = next;
        }
        Ok(OpOutput {
            mesh: current,
            origins,
            summary: format!(
                "stitched {stitched} T-junction vertex/vertices into their host edges"
            ),
            max_allowed_face_deletion: 0,
            allow_bbox_shrink: false,
        })
    }
}

pub(crate) struct SplitCrossingsOp;

impl RepairOp for SplitCrossingsOp {
    fn name(&self) -> &'static str {
        "Split crossing triangles"
    }

    fn apply(&self, mesh: &TriangleMesh, report: &DiagnosticReport) -> Result<OpOutput, MeshError> {
        if report.self_intersections.is_empty() {
            return Ok(unchanged(mesh, "none"));
        }
        let mut current = mesh.clone();
        let mut origins = identity(mesh.faces.len());
        let mut splits = 0usize;
        let face_cap = mesh.faces.len().saturating_mul(6).max(48);
        for _ in 0..48 {
            if current.faces.len() > face_cap {
                break;
            }
            let scan = scan_geometry(&current);
            let mut progressed = false;
            for hit in scan.self_intersections {
                let Some((next, step)) = split_crossing(&current, hit.face_a, hit.face_b) else {
                    continue;
                };
                if next == current {
                    continue;
                }
                splits += 1;
                origins = compose_origins(&origins, &step);
                current = next;
                progressed = true;
                break;
            }
            if !progressed {
                break;
            }
        }
        Ok(OpOutput {
            mesh: current,
            origins,
            summary: format!("split {splits} crossing triangle pair(s) along their intersection"),
            max_allowed_face_deletion: 0,
            allow_bbox_shrink: false,
        })
    }
}

pub(crate) struct UnionOverlappingShellsOp;

impl RepairOp for UnionOverlappingShellsOp {
    fn name(&self) -> &'static str {
        "Union overlapping closed shells"
    }

    fn apply(&self, mesh: &TriangleMesh, report: &DiagnosticReport) -> Result<OpOutput, MeshError> {
        if report.components.iter().filter(|c| c.is_closed).count() < 2 {
            return Ok(unchanged(mesh, "none"));
        }
        let Some((next, origins, removed)) = union_overlapping(mesh) else {
            return Ok(unchanged(mesh, "none"));
        };
        let after = mesh_analysis::analyze(&next);
        if !after.is_watertight
            || !after.self_intersections.is_empty()
            || after.signed_volume <= 0.0
        {
            return Ok(unchanged(mesh, "none"));
        }
        Ok(OpOutput {
            mesh: next,
            origins,
            summary: format!("removed {removed} face(s) inside an overlapping closed shell"),
            max_allowed_face_deletion: mesh.faces.len(),
            allow_bbox_shrink: true,
        })
    }
}

fn unchanged(mesh: &TriangleMesh, summary: &str) -> OpOutput {
    OpOutput {
        mesh: mesh.clone(),
        origins: identity(mesh.faces.len()),
        summary: summary.to_string(),
        max_allowed_face_deletion: 0,
        allow_bbox_shrink: false,
    }
}

fn identity(n: usize) -> Vec<FaceOrigin> {
    (0..n)
        .map(|i| FaceOrigin::Identity(FaceId(i as u32)))
        .collect()
}

fn edge_param(mesh: &TriangleMesh, edge: EdgeKey, vertex: VertexId) -> f64 {
    let Some(p) = mesh.vertex(vertex) else {
        return 0.0;
    };
    let Some(a) = mesh.vertex(VertexId(edge.0)) else {
        return 0.0;
    };
    let Some(b) = mesh.vertex(VertexId(edge.1)) else {
        return 0.0;
    };
    let ab = b - a;
    let len2 = ab.norm_squared();
    if len2 < 1e-20 {
        0.0
    } else {
        (p - a).dot(&ab) / len2
    }
}

fn split_edge(
    mesh: &TriangleMesh,
    edge: EdgeKey,
    mids: &[VertexId],
) -> (TriangleMesh, Vec<FaceOrigin>) {
    let mut faces = Vec::with_capacity(mesh.faces.len() + mids.len());
    let mut origins = Vec::with_capacity(mesh.faces.len() + mids.len());
    for (fi, &[a, b, c]) in mesh.faces.iter().enumerate() {
        let parent = FaceId(fi as u32);
        let reference = face_normal(mesh, [a, b, c]).unwrap_or(Vector3::z());
        let Some(apex_forward) = edge_apex([a, b, c], edge) else {
            faces.push([a, b, c]);
            origins.push(FaceOrigin::Identity(parent));
            continue;
        };
        let (apex, forward) = apex_forward;
        let mut chain = Vec::with_capacity(mids.len() + 2);
        if forward {
            chain.push(edge.0);
            chain.extend(mids.iter().map(|v| v.0));
            chain.push(edge.1);
        } else {
            chain.push(edge.1);
            chain.extend(mids.iter().rev().map(|v| v.0));
            chain.push(edge.0);
        }
        for pair in chain.windows(2) {
            let tri = with_winding(mesh, [pair[0], pair[1], apex], reference);
            faces.push(tri);
            origins.push(FaceOrigin::Split { parent });
        }
    }
    (
        TriangleMesh::from_parts(mesh.vertices.clone(), faces),
        origins,
    )
}

fn edge_apex(face: [u32; 3], edge: EdgeKey) -> Option<(u32, bool)> {
    for i in 0..3 {
        let u = face[i];
        let v = face[(i + 1) % 3];
        let w = face[(i + 2) % 3];
        if EdgeKey::new(u, v) == edge {
            let forward = u == edge.0 && v == edge.1;
            return Some((w, forward));
        }
    }
    None
}

fn split_crossing(
    mesh: &TriangleMesh,
    face_a: FaceId,
    face_b: FaceId,
) -> Option<(TriangleMesh, Vec<FaceOrigin>)> {
    let (pa, qa) = boundary_cut(mesh, face_a, face_b)?;
    let (pb, qb) = boundary_cut(mesh, face_b, face_a)?;
    let mut vertices = mesh.vertices.clone();
    let pa_id = insert_vertex(&mut vertices, pa);
    let qa_id = insert_vertex(&mut vertices, qa);
    let pb_id = insert_vertex(&mut vertices, pb);
    let qb_id = insert_vertex(&mut vertices, qb);
    let mut edge_points: BTreeMap<EdgeKey, Vec<u32>> = BTreeMap::new();
    record_boundary_point(mesh, face_a, pa, pa_id, &mut edge_points);
    record_boundary_point(mesh, face_a, qa, qa_id, &mut edge_points);
    record_boundary_point(mesh, face_b, pb, pb_id, &mut edge_points);
    record_boundary_point(mesh, face_b, qb, qb_id, &mut edge_points);
    for (edge, mids) in edge_points.iter_mut() {
        mids.sort_by(|a, b| {
            edge_param_verts(&vertices, *edge, VertexId(*a))
                .partial_cmp(&edge_param_verts(&vertices, *edge, VertexId(*b)))
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        mids.dedup();
    }
    let mut faces = Vec::new();
    let mut origins = Vec::new();
    let mut changed = false;
    for (fi, &idx) in mesh.faces.iter().enumerate() {
        let parent = FaceId(fi as u32);
        let reference = face_normal_pts([
            vertices[idx[0] as usize],
            vertices[idx[1] as usize],
            vertices[idx[2] as usize],
        ])
        .unwrap_or(Vector3::z());
        let pieces = if parent == face_a {
            subdivide_triangle(idx, pa, qa, pa_id, qa_id, &vertices, reference)
        } else if parent == face_b {
            subdivide_triangle(idx, pb, qb, pb_id, qb_id, &vertices, reference)
        } else {
            fan_incident_edges(idx, &edge_points, &vertices, reference)
        };
        if pieces.len() != 1 || pieces[0] != idx {
            changed = true;
        }
        if (parent == face_a || parent == face_b) && (pieces.len() == 1 && pieces[0] == idx) {
            return None;
        }
        for tri in pieces {
            origins.push(if tri == idx {
                FaceOrigin::Identity(parent)
            } else {
                FaceOrigin::Split { parent }
            });
            faces.push(tri);
        }
    }
    if !changed {
        return None;
    }
    Some((TriangleMesh::from_parts(vertices, faces), origins))
}

fn face_positions(vertices: &[Point3<f64>], idx: [u32; 3]) -> [Point3<f64>; 3] {
    [
        vertices[idx[0] as usize],
        vertices[idx[1] as usize],
        vertices[idx[2] as usize],
    ]
}

fn record_boundary_point(
    mesh: &TriangleMesh,
    face: FaceId,
    point: Point3<f64>,
    id: u32,
    edges: &mut BTreeMap<EdgeKey, Vec<u32>>,
) {
    let Some(&idx) = mesh.faces.get(face.0 as usize) else {
        return;
    };
    let pos = face_positions(&mesh.vertices, idx);
    if let Some(Located::Edge(a, b)) = locate_on_boundary(idx, pos, point) {
        edges.entry(EdgeKey::new(a, b)).or_default().push(id);
    }
}

fn edge_param_verts(vertices: &[Point3<f64>], edge: EdgeKey, vertex: VertexId) -> f64 {
    let (Some(p), Some(a), Some(b)) = (
        vertices.get(vertex.0 as usize).copied(),
        vertices.get(edge.0 as usize).copied(),
        vertices.get(edge.1 as usize).copied(),
    ) else {
        return 0.0;
    };
    let ab = b - a;
    let len2 = ab.norm_squared();
    if len2 < 1e-20 {
        0.0
    } else {
        (p - a).dot(&ab) / len2
    }
}

fn fan_incident_edges(
    idx: [u32; 3],
    edge_points: &BTreeMap<EdgeKey, Vec<u32>>,
    vertices: &[Point3<f64>],
    reference: Vector3<f64>,
) -> Vec<[u32; 3]> {
    let mut pieces = vec![idx];
    for (edge, mids) in edge_points {
        if mids.is_empty() {
            continue;
        }
        let mut next = Vec::new();
        for piece in pieces {
            if edge_apex(piece, *edge).is_some() {
                next.extend(fan_edge(vertices, piece, *edge, mids, reference));
            } else {
                next.push(piece);
            }
        }
        pieces = next;
    }
    pieces
}

fn fan_edge(
    vertices: &[Point3<f64>],
    idx: [u32; 3],
    edge: EdgeKey,
    mids: &[u32],
    reference: Vector3<f64>,
) -> Vec<[u32; 3]> {
    let Some((apex, forward)) = edge_apex(idx, edge) else {
        return vec![idx];
    };
    let mut chain = Vec::with_capacity(mids.len() + 2);
    if forward {
        chain.push(edge.0);
        chain.extend(mids.iter().copied());
        chain.push(edge.1);
    } else {
        chain.push(edge.1);
        chain.extend(mids.iter().rev().copied());
        chain.push(edge.0);
    }
    chain
        .windows(2)
        .map(|pair| with_winding_pts(vertices, [pair[0], pair[1], apex], reference))
        .collect()
}

fn subdivide_triangle(
    idx: [u32; 3],
    p: Point3<f64>,
    q: Point3<f64>,
    p_id: u32,
    q_id: u32,
    vertices: &[Point3<f64>],
    reference: Vector3<f64>,
) -> Vec<[u32; 3]> {
    let pos = face_positions(vertices, idx);
    let Some(lp) = locate_on_boundary(idx, pos, p) else {
        return vec![idx];
    };
    let Some(lq) = locate_on_boundary(idx, pos, q) else {
        return vec![idx];
    };
    let raw = match (lp, lq) {
        (Located::Vertex(_), Located::Vertex(_)) => vec![idx],
        (Located::Vertex(v), Located::Edge(e0, e1)) => fan_from_vertex(idx, v, e0, e1, q_id),
        (Located::Edge(e0, e1), Located::Vertex(v)) => fan_from_vertex(idx, v, e0, e1, p_id),
        (Located::Edge(a0, a1), Located::Edge(b0, b1)) => {
            fan_from_two_edges(idx, a0, a1, b0, b1, p_id, q_id)
        }
    };
    raw.into_iter()
        .map(|tri| with_winding_pts(vertices, tri, reference))
        .collect()
}

enum Located {
    Vertex(u32),
    Edge(u32, u32),
}

fn locate_on_boundary(idx: [u32; 3], pos: [Point3<f64>; 3], point: Point3<f64>) -> Option<Located> {
    for i in 0..3 {
        if (pos[i] - point).norm() <= 1e-6 {
            return Some(Located::Vertex(idx[i]));
        }
    }
    for (i, j) in [(0, 1), (1, 2), (2, 0)] {
        if point_on_segment(point, pos[i], pos[j]) {
            return Some(Located::Edge(idx[i], idx[j]));
        }
    }
    None
}

fn point_on_segment(p: Point3<f64>, a: Point3<f64>, b: Point3<f64>) -> bool {
    let ab = b - a;
    let len2 = ab.norm_squared();
    if len2 < 1e-20 {
        return false;
    }
    let t = (p - a).dot(&ab) / len2;
    if !(0.0..=1.0).contains(&t) {
        return false;
    }
    (p - (a + ab * t)).norm() <= 1e-5
}

fn fan_from_vertex(idx: [u32; 3], vertex: u32, e0: u32, e1: u32, point: u32) -> Vec<[u32; 3]> {
    let mut ordered = idx;
    while ordered[0] != vertex {
        ordered = [ordered[1], ordered[2], ordered[0]];
    }
    if !(ordered[1] == e0 && ordered[2] == e1) && !(ordered[1] == e1 && ordered[2] == e0) {
        return vec![idx];
    }
    vec![
        [ordered[0], ordered[1], point],
        [ordered[0], point, ordered[2]],
    ]
}

fn fan_from_two_edges(
    idx: [u32; 3],
    a0: u32,
    a1: u32,
    b0: u32,
    b1: u32,
    p_id: u32,
    q_id: u32,
) -> Vec<[u32; 3]> {
    let shared = [a0, a1].into_iter().find(|v| *v == b0 || *v == b1);
    let Some(corner) = shared else {
        return vec![idx];
    };
    let far_a = if a0 == corner { a1 } else { a0 };
    let far_b = if b0 == corner { b1 } else { b0 };
    let mut ordered = idx;
    while ordered[0] != corner {
        ordered = [ordered[1], ordered[2], ordered[0]];
    }
    let (p_on_first, q_on_first) = if ordered[1] == far_a {
        (p_id, q_id)
    } else if ordered[1] == far_b {
        (q_id, p_id)
    } else {
        return vec![idx];
    };
    vec![
        [corner, p_on_first, q_on_first],
        [p_on_first, ordered[1], ordered[2]],
        [p_on_first, ordered[2], q_on_first],
    ]
}

fn insert_vertex(vertices: &mut Vec<Point3<f64>>, point: Point3<f64>) -> u32 {
    for (i, existing) in vertices.iter().enumerate() {
        if (existing - point).norm_squared() <= 1e-12 {
            return i as u32;
        }
    }
    vertices.push(point);
    (vertices.len() - 1) as u32
}

fn face_normal(mesh: &TriangleMesh, idx: [u32; 3]) -> Option<Vector3<f64>> {
    let pos = [
        *mesh.vertices.get(idx[0] as usize)?,
        *mesh.vertices.get(idx[1] as usize)?,
        *mesh.vertices.get(idx[2] as usize)?,
    ];
    face_normal_pts(pos)
}

fn face_normal_pts(pos: [Point3<f64>; 3]) -> Option<Vector3<f64>> {
    let n = (pos[1] - pos[0]).cross(&(pos[2] - pos[0]));
    let len = n.norm();
    if len < 1e-16 { None } else { Some(n / len) }
}

fn with_winding(mesh: &TriangleMesh, tri: [u32; 3], reference: Vector3<f64>) -> [u32; 3] {
    with_winding_pts(&mesh.vertices, tri, reference)
}

fn with_winding_pts(vertices: &[Point3<f64>], tri: [u32; 3], reference: Vector3<f64>) -> [u32; 3] {
    let Some(n) = face_normal_pts([
        vertices[tri[0] as usize],
        vertices[tri[1] as usize],
        vertices[tri[2] as usize],
    ]) else {
        return tri;
    };
    if n.dot(&reference) < 0.0 {
        [tri[0], tri[2], tri[1]]
    } else {
        tri
    }
}

fn union_overlapping(mesh: &TriangleMesh) -> Option<(TriangleMesh, Vec<FaceOrigin>, usize)> {
    let comps = closed_face_components(mesh);
    if comps.len() < 2 {
        return None;
    }
    let mut chosen = None;
    'pairs: for i in 0..comps.len() {
        for j in (i + 1)..comps.len() {
            if component_overlaps(mesh, &comps[i], &comps[j]) {
                chosen = Some((i, j));
                break 'pairs;
            }
        }
    }
    let (i, j) = chosen?;
    let mut current = mesh.clone();
    let mut shell = vec![2u8; current.faces.len()];
    for &fi in &comps[i] {
        shell[fi] = 0;
    }
    for &fi in &comps[j] {
        shell[fi] = 1;
    }
    let mut origins = identity(mesh.faces.len());
    for _ in 0..64 {
        if current.faces.len() > mesh.faces.len().saturating_mul(8).max(64) {
            break;
        }
        let scan = scan_geometry(&current);
        let mut progressed = false;
        for hit in scan.self_intersections {
            let Some((next, step)) = split_crossing(&current, hit.face_a, hit.face_b) else {
                continue;
            };
            if next == current {
                continue;
            }
            shell = remap_shell(&shell, &step);
            origins = compose_origins(&origins, &step);
            current = next;
            progressed = true;
            break;
        }
        if !progressed {
            break;
        }
    }
    let report = mesh_analysis::analyze(&current);
    if let Ok(welded) = (super::WeldVerticesOp { epsilon: 1e-5 }).apply(&current, &report)
        && welded.mesh != current
    {
        shell = remap_shell(&shell, &welded.origins);
        origins = compose_origins(&origins, &welded.origins);
        current = welded.mesh;
    }
    let faces0 = shell_faces(&shell, 0);
    let faces1 = shell_faces(&shell, 1);
    if faces0.is_empty() || faces1.is_empty() {
        return None;
    }
    let mut keep = vec![true; current.faces.len()];
    let mut removed = 0usize;
    for (fi, id) in shell.iter().enumerate() {
        let other: &[usize] = match id {
            0 => &faces1,
            1 => &faces0,
            _ => continue,
        };
        if sample_inside_other(&current, fi, other)? {
            keep[fi] = false;
            removed += 1;
        }
    }
    if removed == 0 {
        return None;
    }
    let mut faces = Vec::new();
    let mut kept_origins = Vec::new();
    for (fi, &idx) in current.faces.iter().enumerate() {
        if !keep[fi] || idx[0] == idx[1] || idx[1] == idx[2] || idx[2] == idx[0] {
            continue;
        }
        faces.push(idx);
        kept_origins.push(origins[fi].clone());
    }
    let (vertices, faces) = compact(&current.vertices, &faces);
    let mut result = TriangleMesh::from_parts(vertices, faces);
    let report = mesh_analysis::analyze(&result);
    if let Ok(stitched) = StitchTJunctionsOp.apply(&result, &report)
        && stitched.mesh != result
    {
        kept_origins = compose_origins(&kept_origins, &stitched.origins);
        result = stitched.mesh;
    }
    Some((result, kept_origins, removed))
}

fn shell_faces(shell: &[u8], id: u8) -> Vec<usize> {
    shell
        .iter()
        .enumerate()
        .filter_map(|(i, s)| (*s == id).then_some(i))
        .collect()
}

fn remap_shell(shell: &[u8], origins: &[FaceOrigin]) -> Vec<u8> {
    origins
        .iter()
        .map(|origin| {
            let parent = match origin {
                FaceOrigin::Identity(id) | FaceOrigin::Split { parent: id } => id.0 as usize,
                FaceOrigin::Synthesized { boundary_sources } => {
                    return boundary_sources
                        .first()
                        .and_then(|id| shell.get(id.0 as usize).copied())
                        .unwrap_or(2);
                }
                FaceOrigin::Spatial { nearest, .. } => nearest.0 as usize,
            };
            shell.get(parent).copied().unwrap_or(2)
        })
        .collect()
}

fn component_overlaps(mesh: &TriangleMesh, a: &[usize], b: &[usize]) -> bool {
    a.iter().any(|&fi| {
        mesh.face_positions(FaceId(fi as u32)).is_some_and(|pos| {
            let c = Point3::from((pos[0].coords + pos[1].coords + pos[2].coords) / 3.0);
            point_inside(mesh, b, c).unwrap_or(false)
        })
    })
}

fn sample_inside_other(mesh: &TriangleMesh, face: usize, other: &[usize]) -> Option<bool> {
    let idx = mesh.faces.get(face)?;
    let normal = face_normal(mesh, *idx)?;
    let pos = mesh.face_positions(FaceId(face as u32))?;
    let centroid = Point3::from((pos[0].coords + pos[1].coords + pos[2].coords) / 3.0);
    let diag = mesh.bounding_box().map(|b| b.diagonal()).unwrap_or(1.0);
    let sample = centroid + normal * (1e-3 * diag.max(1.0));
    point_inside(mesh, other, sample)
}

fn point_inside(mesh: &TriangleMesh, faces: &[usize], origin: Point3<f64>) -> Option<bool> {
    let dirs = [
        Vector3::new(1.0, 0.137, 0.241),
        Vector3::new(0.193, 1.0, 0.317),
        Vector3::new(0.271, 0.159, 1.0),
        Vector3::new(-0.73, 0.41, 0.55),
        Vector3::new(0.37, -0.81, 0.29),
    ];
    for dir in dirs {
        let dir = dir.normalize();
        let mut hits = 0usize;
        let mut ambiguous = false;
        for &fi in faces {
            let Some(tri) = mesh.face_positions(FaceId(fi as u32)) else {
                continue;
            };
            match ray_triangle(origin, dir, tri) {
                RayHit::Hit => hits += 1,
                RayHit::Ambiguous => {
                    ambiguous = true;
                    break;
                }
                RayHit::Miss => {}
            }
        }
        if !ambiguous {
            return Some(hits % 2 == 1);
        }
    }
    None
}

enum RayHit {
    Miss,
    Hit,
    Ambiguous,
}

fn ray_triangle(origin: Point3<f64>, dir: Vector3<f64>, tri: [Point3<f64>; 3]) -> RayHit {
    let e1 = tri[1] - tri[0];
    let e2 = tri[2] - tri[0];
    let pvec = dir.cross(&e2);
    let det = e1.dot(&pvec);
    if det.abs() < 1e-12 {
        return RayHit::Miss;
    }
    let inv = 1.0 / det;
    let tvec = origin - tri[0];
    let u = tvec.dot(&pvec) * inv;
    if !(-1e-8..=1.0 + 1e-8).contains(&u) {
        return RayHit::Miss;
    }
    let qvec = tvec.cross(&e1);
    let v = dir.dot(&qvec) * inv;
    if v < -1e-8 || u + v > 1.0 + 1e-8 {
        return RayHit::Miss;
    }
    let t = e2.dot(&qvec) * inv;
    if t <= 1e-7 {
        return RayHit::Miss;
    }
    let near_edge = u < 1e-5 || v < 1e-5 || (1.0 - u - v) < 1e-5;
    if near_edge {
        RayHit::Ambiguous
    } else {
        RayHit::Hit
    }
}

fn closed_face_components(mesh: &TriangleMesh) -> Vec<Vec<usize>> {
    let incidence = mesh.edge_incidence();
    let mut visited = vec![false; mesh.faces.len()];
    let mut components = Vec::new();
    for start in 0..mesh.faces.len() {
        if visited[start] {
            continue;
        }
        let mut queue = VecDeque::new();
        let mut faces = Vec::new();
        let mut closed = true;
        visited[start] = true;
        queue.push_back(start);
        while let Some(fi) = queue.pop_front() {
            let [a, b, c] = mesh.faces[fi];
            if a as usize >= mesh.vertices.len()
                || b as usize >= mesh.vertices.len()
                || c as usize >= mesh.vertices.len()
            {
                closed = false;
                continue;
            }
            faces.push(fi);
            for (u, v) in [(a, b), (b, c), (c, a)] {
                if u == v {
                    closed = false;
                    continue;
                }
                let Some(uses) = incidence.get(&EdgeKey::new(u, v)) else {
                    closed = false;
                    continue;
                };
                if uses.len() != 2 {
                    closed = false;
                }
                for eu in uses {
                    let nf = eu.face.0 as usize;
                    if !visited[nf] {
                        visited[nf] = true;
                        queue.push_back(nf);
                    }
                }
            }
        }
        if closed && faces.len() >= 4 {
            components.push(faces);
        }
    }
    components
}

fn compact(vertices: &[Point3<f64>], faces: &[[u32; 3]]) -> (Vec<Point3<f64>>, Vec<[u32; 3]>) {
    let mut remap = vec![u32::MAX; vertices.len()];
    let mut new_verts = Vec::new();
    let mut new_faces = Vec::with_capacity(faces.len());
    for &[a, b, c] in faces {
        let mut tri = [0u32; 3];
        for (k, old) in [a, b, c].into_iter().enumerate() {
            if remap[old as usize] == u32::MAX {
                remap[old as usize] = new_verts.len() as u32;
                new_verts.push(vertices[old as usize]);
            }
            tri[k] = remap[old as usize];
        }
        new_faces.push(tri);
    }
    (new_verts, new_faces)
}

#[cfg(test)]
mod tests {
    use super::super::repair_mesh;
    use super::*;
    use mesh_analysis::analyze;
    use mesh_attributes::AttributeStore;
    use mesh_core::RepairProfile;

    fn crossing() -> TriangleMesh {
        TriangleMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
                Point3::new(1.0, 2.0, 0.0),
                Point3::new(0.0, 0.5, -1.0),
                Point3::new(2.0, 0.5, -1.0),
                Point3::new(1.0, 0.5, 1.0),
            ],
            vec![[0, 1, 2], [3, 4, 5]],
        )
    }

    fn tee() -> TriangleMesh {
        TriangleMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, -1.0, 0.0),
            ],
            vec![[0, 1, 2], [3, 4, 0]],
        )
    }

    #[test]
    fn standard_splits_crossings_and_stitches_t_junctions() {
        let attrs = AttributeStore::new_for_mesh(2);
        let repaired =
            repair_mesh(&crossing(), &attrs, RepairProfile::Standard, None).expect("repair");
        assert!(
            repaired.report.after.self_intersections.is_empty(),
            "still intersecting: {:?}",
            repaired.report.kept_steps
        );
        assert!(
            repaired
                .report
                .kept_steps
                .iter()
                .any(|s| s.name.contains("crossing")),
            "steps {:?} rejected {:?}",
            repaired
                .report
                .kept_steps
                .iter()
                .map(|s| format!("{}: {}", s.name, s.detail))
                .collect::<Vec<_>>(),
            repaired
                .report
                .rejected_steps
                .iter()
                .map(|s| format!("{}: {}", s.name, s.detail))
                .collect::<Vec<_>>()
        );

        let conservative =
            repair_mesh(&crossing(), &attrs, RepairProfile::Conservative, None).expect("repair");
        assert!(!conservative.report.after.self_intersections.is_empty());

        let tee_attrs = AttributeStore::new_for_mesh(2);
        let stitched =
            repair_mesh(&tee(), &tee_attrs, RepairProfile::Standard, None).expect("stitch");
        assert!(stitched.report.after.t_junctions.is_empty());
        let kept =
            repair_mesh(&tee(), &tee_attrs, RepairProfile::Conservative, None).expect("keep");
        assert!(!kept.report.after.t_junctions.is_empty());
    }

    #[test]
    fn aggressive_unions_overlapping_boxes() {
        let a = box_mesh(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 2.0));
        let b = box_mesh(Point3::new(1.0, 1.0, 1.0), Point3::new(3.0, 3.0, 3.0));
        let mesh = merge(&a, &b);
        let before = analyze(&mesh);
        assert!(!before.self_intersections.is_empty());
        let attrs = AttributeStore::new_for_mesh(mesh.faces.len());
        let repaired = repair_mesh(&mesh, &attrs, RepairProfile::Aggressive, None).expect("union");
        assert!(
            repaired.report.after.is_watertight,
            "notes: {:?}",
            repaired.report.unresolved_notes
        );
        assert!(
            repaired.report.after.self_intersections.is_empty(),
            "intersections {} notes {:?} steps {:?} rejected {:?}",
            repaired.report.after.self_intersections.len(),
            repaired.report.unresolved_notes,
            repaired
                .report
                .kept_steps
                .iter()
                .map(|s| format!("{}: {}", s.name, s.detail))
                .collect::<Vec<_>>(),
            repaired
                .report
                .rejected_steps
                .iter()
                .map(|s| format!("{}: {}", s.name, s.detail))
                .collect::<Vec<_>>()
        );
        assert_eq!(repaired.report.after.components.len(), 1);
        let volume = repaired.mesh.signed_volume();
        assert!(
            (volume - 15.0).abs() < 0.05,
            "union volume {volume} (steps {:?})",
            repaired
                .report
                .kept_steps
                .iter()
                .map(|s| s.name.as_str())
                .collect::<Vec<_>>()
        );
    }

    fn box_mesh(min: Point3<f64>, max: Point3<f64>) -> TriangleMesh {
        let vertices = vec![
            Point3::new(min.x, min.y, min.z),
            Point3::new(max.x, min.y, min.z),
            Point3::new(max.x, max.y, min.z),
            Point3::new(min.x, max.y, min.z),
            Point3::new(min.x, min.y, max.z),
            Point3::new(max.x, min.y, max.z),
            Point3::new(max.x, max.y, max.z),
            Point3::new(min.x, max.y, max.z),
        ];
        let faces = vec![
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [2, 3, 7],
            [2, 7, 6],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
        ];
        TriangleMesh::from_parts(vertices, faces)
    }

    fn merge(a: &TriangleMesh, b: &TriangleMesh) -> TriangleMesh {
        let mut vertices = a.vertices.clone();
        let base = vertices.len() as u32;
        vertices.extend_from_slice(&b.vertices);
        let mut faces = a.faces.clone();
        for [x, y, z] in &b.faces {
            faces.push([x + base, y + base, z + base]);
        }
        TriangleMesh::from_parts(vertices, faces)
    }
}
