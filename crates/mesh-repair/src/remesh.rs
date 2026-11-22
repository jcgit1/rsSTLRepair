//! Aggressive local remesh. Long edges split and short edges collapse inside a
//! defect patch. Vertices stay on the input surface. A face that no longer lies
//! on one input triangle is [`FaceOrigin::Spatial`], which attribute transfer
//! marks uncertain.

use super::{OpOutput, RepairOp};
use mesh_analysis::DiagnosticReport;
use mesh_attributes::FaceOrigin;
use mesh_core::{EdgeKey, FaceId, MeshError, TriangleMesh};
use nalgebra::Point3;
use std::collections::{BTreeMap, BTreeSet};

const ASPECT_SEED: f64 = 8.0;
const SPLIT_FACTOR: f64 = 4.0 / 3.0;
const COLLAPSE_FACTOR: f64 = 4.0 / 5.0;
const MAX_EDITS: usize = 8;
const MAX_SEED_FACES: usize = 2048;

pub(crate) struct ControlledRemeshOp;

impl RepairOp for ControlledRemeshOp {
    fn name(&self) -> &'static str {
        "Controlled remesh"
    }

    fn apply(&self, mesh: &TriangleMesh, report: &DiagnosticReport) -> Result<OpOutput, MeshError> {
        let mut work = Work::new(mesh, report);
        if work.seeds().len() > MAX_SEED_FACES {
            return Ok(unchanged(mesh, "skipped: defect patch exceeds 2048 faces"));
        }
        let target = median_edge_length(mesh);
        if !target.is_finite() || target <= 1e-15 {
            return Ok(unchanged(mesh, "none"));
        }

        let mut collapsed = 0usize;
        let mut split = 0usize;
        let mut removed = 0usize;
        for _ in 0..MAX_EDITS {
            if work.seeds().len() > MAX_SEED_FACES {
                break;
            }
            if let Some(n) = work.collapse_shortest(target) {
                collapsed += 1;
                removed += n;
                continue;
            }
            if work.faces.len() >= mesh.faces.len().saturating_mul(4).max(16) {
                break;
            }
            if work.split_longest(target) {
                split += 1;
                continue;
            }
            break;
        }

        if collapsed == 0 && split == 0 {
            return Ok(unchanged(mesh, "none"));
        }

        let (out, origins) = work.finish(mesh);
        let spatial = origins
            .iter()
            .filter(|o| matches!(o, FaceOrigin::Spatial { .. }))
            .count();
        Ok(OpOutput {
            mesh: out,
            origins,
            summary: format!(
                "collapsed {collapsed} short edge(s) and split {split} long edge(s); {spatial} face(s) fell back to spatial correspondence"
            ),
            max_allowed_face_deletion: removed,
            allow_bbox_shrink: false,
        })
    }
}

struct Work {
    vertices: Vec<Point3<f64>>,
    faces: Vec<[u32; 3]>,
    on_faces: Vec<BTreeSet<u32>>,
    source: Vec<Option<u32>>,
    defect: BTreeSet<u32>,
}

impl Work {
    fn new(mesh: &TriangleMesh, report: &DiagnosticReport) -> Self {
        let mut on_faces = vec![BTreeSet::new(); mesh.vertices.len()];
        for (fi, face) in mesh.faces.iter().enumerate() {
            for v in face {
                if let Some(set) = on_faces.get_mut(*v as usize) {
                    set.insert(fi as u32);
                }
            }
        }
        let mut defect = BTreeSet::new();
        for hit in &report.self_intersections {
            defect.insert(hit.face_a.0);
            defect.insert(hit.face_b.0);
        }
        let incidence = mesh.edge_incidence();
        for edge in &report.non_manifold_edges {
            if let Some(uses) = incidence.get(&edge.edge) {
                for used in uses {
                    defect.insert(used.face.0);
                }
            }
        }
        Self {
            vertices: mesh.vertices.clone(),
            faces: mesh.faces.clone(),
            on_faces,
            source: (0..mesh.vertices.len()).map(|i| Some(i as u32)).collect(),
            defect,
        }
    }

    fn seeds(&self) -> BTreeSet<usize> {
        let mut seeds = BTreeSet::new();
        for (fi, face) in self.faces.iter().enumerate() {
            if self.face_is_seed(*face) {
                seeds.insert(fi);
            }
        }
        seeds
    }

    fn face_is_seed(&self, face: [u32; 3]) -> bool {
        if self.aspect(face) >= ASPECT_SEED {
            return true;
        }
        let common = self.common_faces(face);
        common.iter().any(|id| self.defect.contains(id))
    }

    fn aspect(&self, face: [u32; 3]) -> f64 {
        let Some([mn, mx]) = self.edge_span(face) else {
            return f64::MAX;
        };
        if mn <= 1e-15 { f64::MAX } else { mx / mn }
    }

    fn edge_span(&self, face: [u32; 3]) -> Option<[f64; 2]> {
        let pts = self.positions(face)?;
        let lengths = [
            (pts[1] - pts[0]).norm(),
            (pts[2] - pts[1]).norm(),
            (pts[0] - pts[2]).norm(),
        ];
        if lengths.iter().any(|l| !l.is_finite()) {
            return None;
        }
        let mn = lengths.into_iter().fold(f64::MAX, f64::min);
        let mx = lengths.into_iter().fold(0.0, f64::max);
        Some([mn, mx])
    }

    fn positions(&self, face: [u32; 3]) -> Option<[Point3<f64>; 3]> {
        Some([
            *self.vertices.get(face[0] as usize)?,
            *self.vertices.get(face[1] as usize)?,
            *self.vertices.get(face[2] as usize)?,
        ])
    }

    fn common_faces(&self, face: [u32; 3]) -> BTreeSet<u32> {
        let Some(first) = self.on_faces.get(face[0] as usize) else {
            return BTreeSet::new();
        };
        let mut common = first.clone();
        for v in [face[1], face[2]] {
            let Some(set) = self.on_faces.get(v as usize) else {
                return BTreeSet::new();
            };
            common = common.intersection(set).copied().collect();
        }
        common
    }

    fn active(&self, seeds: &BTreeSet<usize>) -> BTreeSet<usize> {
        let mut verts = BTreeSet::new();
        for fi in seeds {
            if let Some(face) = self.faces.get(*fi) {
                verts.extend(face.iter().copied());
            }
        }
        let mut active = seeds.clone();
        for (fi, face) in self.faces.iter().enumerate() {
            if face.iter().any(|v| verts.contains(v)) {
                active.insert(fi);
            }
        }
        active
    }

    fn collapse_shortest(&mut self, target: f64) -> Option<usize> {
        let limit = target * COLLAPSE_FACTOR;
        let mut edges = self.candidate_edges();
        edges.retain(|edge| edge.length < limit);
        edges.sort_by(|a, b| a.length.total_cmp(&b.length).then(a.key.cmp(&b.key)));
        for edge in edges {
            if self.collapse(edge.key.0, edge.key.1) {
                return Some(2);
            }
        }
        None
    }

    fn split_longest(&mut self, target: f64) -> bool {
        let limit = target * SPLIT_FACTOR;
        let mut edges = self.candidate_edges();
        edges.retain(|edge| edge.length > limit);
        edges.sort_by(|a, b| b.length.total_cmp(&a.length).then(a.key.cmp(&b.key)));
        for edge in edges {
            if self.split(edge.key.0, edge.key.1) {
                return true;
            }
        }
        false
    }

    fn candidate_edges(&self) -> Vec<CandEdge> {
        let seeds = self.seeds();
        let active = self.active(&seeds);
        let mut grouped: BTreeMap<EdgeKey, Vec<usize>> = BTreeMap::new();
        for (fi, face) in self.faces.iter().enumerate() {
            for (u, v) in [(face[0], face[1]), (face[1], face[2]), (face[2], face[0])] {
                if u != v {
                    grouped.entry(EdgeKey::new(u, v)).or_default().push(fi);
                }
            }
        }
        let mut out = Vec::new();
        for (key, faces) in grouped {
            if faces.len() != 2 || !faces.iter().all(|fi| active.contains(fi)) {
                continue;
            }
            if !faces.iter().any(|fi| seeds.contains(fi)) {
                continue;
            }
            let Some(pa) = self.vertices.get(key.0 as usize).copied() else {
                continue;
            };
            let Some(pb) = self.vertices.get(key.1 as usize).copied() else {
                continue;
            };
            let length = (pb - pa).norm();
            if length.is_finite() && length > 1e-15 {
                out.push(CandEdge { key, length });
            }
        }
        out
    }

    fn collapse(&mut self, a: u32, b: u32) -> bool {
        let incident = self.faces_on_edge(a, b);
        if incident.len() != 2 || !self.link_ok(a, b, &incident) {
            return false;
        }
        let (Some(pa), Some(pb)) = (
            self.vertices.get(a as usize).copied(),
            self.vertices.get(b as usize).copied(),
        ) else {
            return false;
        };
        let mid = Point3::from((pa.coords + pb.coords) * 0.5);
        if !mid.x.is_finite() || !mid.y.is_finite() || !mid.z.is_finite() {
            return false;
        }
        let new_id = self.vertices.len() as u32;
        let mut next = Vec::with_capacity(self.faces.len());
        for (fi, face) in self.faces.iter().enumerate() {
            if incident.contains(&fi) {
                continue;
            }
            let mut rewritten = *face;
            if rewritten.contains(&a) || rewritten.contains(&b) {
                for v in &mut rewritten {
                    if *v == a || *v == b {
                        *v = new_id;
                    }
                }
            }
            if rewritten[0] == rewritten[1]
                || rewritten[1] == rewritten[2]
                || rewritten[0] == rewritten[2]
            {
                return false;
            }
            if !self.rewritten_area_ok(rewritten, new_id, mid) {
                return false;
            }
            next.push(rewritten);
        }
        if has_duplicate_faces(&next) {
            return false;
        }
        let on = intersect_sets(self.on_faces.get(a as usize), self.on_faces.get(b as usize));
        self.vertices.push(mid);
        self.on_faces.push(on);
        self.source.push(None);
        self.faces = next;
        true
    }

    fn split(&mut self, a: u32, b: u32) -> bool {
        let incident = self.faces_on_edge(a, b);
        if incident.len() != 2 {
            return false;
        }
        let (Some(pa), Some(pb)) = (
            self.vertices.get(a as usize).copied(),
            self.vertices.get(b as usize).copied(),
        ) else {
            return false;
        };
        let mid = Point3::from((pa.coords + pb.coords) * 0.5);
        if !mid.x.is_finite() || !mid.y.is_finite() || !mid.z.is_finite() {
            return false;
        }
        let new_id = self.vertices.len() as u32;
        let mut next = Vec::with_capacity(self.faces.len() + 2);
        for (fi, face) in self.faces.iter().enumerate() {
            if !incident.contains(&fi) {
                next.push(*face);
                continue;
            }
            let (first, second) = split_winding(*face, a, b, new_id);
            if !self.rewritten_area_ok(first, new_id, mid)
                || !self.rewritten_area_ok(second, new_id, mid)
            {
                return false;
            }
            next.push(first);
            next.push(second);
        }
        let on = intersect_sets(self.on_faces.get(a as usize), self.on_faces.get(b as usize));
        self.vertices.push(mid);
        self.on_faces.push(on);
        self.source.push(None);
        self.faces = next;
        true
    }

    fn rewritten_area_ok(&self, face: [u32; 3], new_id: u32, mid: Point3<f64>) -> bool {
        let pos = |v: u32| {
            if v == new_id {
                mid
            } else {
                self.vertices.get(v as usize).copied().unwrap_or(mid)
            }
        };
        TriangleMesh::triangle_area(pos(face[0]), pos(face[1]), pos(face[2])) > 1e-12
    }

    fn faces_on_edge(&self, a: u32, b: u32) -> Vec<usize> {
        self.faces
            .iter()
            .enumerate()
            .filter(|(_, face)| face.contains(&a) && face.contains(&b))
            .map(|(fi, _)| fi)
            .collect()
    }

    fn link_ok(&self, a: u32, b: u32, incident: &[usize]) -> bool {
        let mut opp = BTreeSet::new();
        for fi in incident {
            let Some(face) = self.faces.get(*fi) else {
                return false;
            };
            for v in face {
                if *v != a && *v != b {
                    opp.insert(*v);
                }
            }
        }
        let na = self.neighbors(a);
        let nb = self.neighbors(b);
        let mut inter: BTreeSet<u32> = na.intersection(&nb).copied().collect();
        inter.remove(&a);
        inter.remove(&b);
        inter == opp
    }

    fn neighbors(&self, v: u32) -> BTreeSet<u32> {
        let mut set = BTreeSet::new();
        for face in &self.faces {
            if face.contains(&v) {
                for u in face {
                    if *u != v {
                        set.insert(*u);
                    }
                }
            }
        }
        set
    }

    fn finish(&self, original: &TriangleMesh) -> (TriangleMesh, Vec<FaceOrigin>) {
        let mut referenced = vec![false; self.vertices.len()];
        for face in &self.faces {
            for v in face {
                if let Some(flag) = referenced.get_mut(*v as usize) {
                    *flag = true;
                }
            }
        }
        let mut remap = vec![u32::MAX; self.vertices.len()];
        let mut vertices = Vec::new();
        let mut on_faces = Vec::new();
        let mut source = Vec::new();
        for (i, keep) in referenced.iter().enumerate() {
            if *keep {
                remap[i] = vertices.len() as u32;
                vertices.push(self.vertices[i]);
                on_faces.push(self.on_faces[i].clone());
                source.push(self.source[i]);
            }
        }
        let faces: Vec<[u32; 3]> = self
            .faces
            .iter()
            .map(|face| {
                [
                    remap[face[0] as usize],
                    remap[face[1] as usize],
                    remap[face[2] as usize],
                ]
            })
            .collect();
        let identity = identity_keys(original);
        let origins = faces
            .iter()
            .map(|face| classify(*face, &vertices, &source, &on_faces, &identity, original))
            .collect();
        (TriangleMesh::from_parts(vertices, faces), origins)
    }
}

struct CandEdge {
    key: EdgeKey,
    length: f64,
}

fn unchanged(mesh: &TriangleMesh, summary: &str) -> OpOutput {
    OpOutput {
        mesh: mesh.clone(),
        origins: (0..mesh.faces.len())
            .map(|i| FaceOrigin::Identity(FaceId(i as u32)))
            .collect(),
        summary: summary.to_string(),
        max_allowed_face_deletion: 0,
        allow_bbox_shrink: false,
    }
}

fn median_edge_length(mesh: &TriangleMesh) -> f64 {
    let mut lengths = Vec::new();
    for uses in mesh.edge_incidence().values() {
        let Some(first) = uses.first() else {
            continue;
        };
        let (a, b) = (first.directed.0, first.directed.1);
        let (Some(pa), Some(pb)) = (
            mesh.vertices.get(a as usize).copied(),
            mesh.vertices.get(b as usize).copied(),
        ) else {
            continue;
        };
        let length = (pb - pa).norm();
        if length.is_finite() && length > 1e-15 {
            lengths.push(length);
        }
    }
    if lengths.is_empty() {
        return 0.0;
    }
    lengths.sort_by(f64::total_cmp);
    lengths[lengths.len() / 2]
}

fn intersect_sets(a: Option<&BTreeSet<u32>>, b: Option<&BTreeSet<u32>>) -> BTreeSet<u32> {
    match (a, b) {
        (Some(a), Some(b)) => a.intersection(b).copied().collect(),
        _ => BTreeSet::new(),
    }
}

fn split_winding(face: [u32; 3], a: u32, b: u32, mid: u32) -> ([u32; 3], [u32; 3]) {
    let ia = face.iter().position(|v| *v == a).unwrap_or(0);
    let ib = face.iter().position(|v| *v == b).unwrap_or(0);
    let c = face
        .into_iter()
        .find(|v| *v != a && *v != b)
        .unwrap_or(face[0]);
    if ib == (ia + 1) % 3 {
        ([a, mid, c], [mid, b, c])
    } else {
        ([b, mid, c], [mid, a, c])
    }
}

fn has_duplicate_faces(faces: &[[u32; 3]]) -> bool {
    let mut seen = BTreeSet::new();
    for face in faces {
        let mut key = *face;
        key.sort_unstable();
        if !seen.insert(key) {
            return true;
        }
    }
    false
}

fn identity_keys(mesh: &TriangleMesh) -> BTreeMap<[u32; 3], u32> {
    let mut map = BTreeMap::new();
    for (fi, face) in mesh.faces.iter().enumerate() {
        let mut key = *face;
        key.sort_unstable();
        map.entry(key).or_insert(fi as u32);
    }
    map
}

fn classify(
    face: [u32; 3],
    vertices: &[Point3<f64>],
    source: &[Option<u32>],
    on_faces: &[BTreeSet<u32>],
    identity: &BTreeMap<[u32; 3], u32>,
    original: &TriangleMesh,
) -> FaceOrigin {
    let sources = [
        source.get(face[0] as usize).copied().flatten(),
        source.get(face[1] as usize).copied().flatten(),
        source.get(face[2] as usize).copied().flatten(),
    ];
    if let (Some(a), Some(b), Some(c)) = (sources[0], sources[1], sources[2]) {
        let mut key = [a, b, c];
        key.sort_unstable();
        if let Some(id) = identity.get(&key) {
            return FaceOrigin::Identity(FaceId(*id));
        }
    }
    let centroid = output_centroid(face, vertices);
    let common = common_of(face, on_faces);
    if let Some(parent) = nearest_of(&common, centroid, original) {
        return FaceOrigin::Split {
            parent: FaceId(parent),
        };
    }
    let union = union_of(face, on_faces);
    let nearest = nearest_of(&union, centroid, original).unwrap_or(0);
    let distance = centroid
        .and_then(|point| original_face_distance(original, nearest, point))
        .unwrap_or(0.0);
    FaceOrigin::Spatial {
        nearest: FaceId(nearest),
        distance,
    }
}

fn common_of(face: [u32; 3], on_faces: &[BTreeSet<u32>]) -> BTreeSet<u32> {
    let Some(first) = on_faces.get(face[0] as usize) else {
        return BTreeSet::new();
    };
    let mut common = first.clone();
    for v in [face[1], face[2]] {
        let Some(set) = on_faces.get(v as usize) else {
            return BTreeSet::new();
        };
        common = common.intersection(set).copied().collect();
    }
    common
}

fn union_of(face: [u32; 3], on_faces: &[BTreeSet<u32>]) -> BTreeSet<u32> {
    let mut union = BTreeSet::new();
    for v in face {
        if let Some(set) = on_faces.get(v as usize) {
            union.extend(set.iter().copied());
        }
    }
    union
}

fn nearest_of(
    candidates: &BTreeSet<u32>,
    centroid: Option<Point3<f64>>,
    original: &TriangleMesh,
) -> Option<u32> {
    let centroid = centroid?;
    candidates
        .iter()
        .copied()
        .filter_map(|id| {
            let dist = original_face_distance(original, id, centroid)?;
            Some((dist, id))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)))
        .map(|(_, id)| id)
}

fn output_centroid(face: [u32; 3], vertices: &[Point3<f64>]) -> Option<Point3<f64>> {
    let a = vertices.get(face[0] as usize).copied()?;
    let b = vertices.get(face[1] as usize).copied()?;
    let c = vertices.get(face[2] as usize).copied()?;
    Some(Point3::from((a.coords + b.coords + c.coords) / 3.0))
}

fn original_face_distance(mesh: &TriangleMesh, face: u32, point: Point3<f64>) -> Option<f64> {
    let [a, b, c] = mesh.face_positions(FaceId(face))?;
    let dist = point_triangle_distance(point, a, b, c);
    dist.is_finite().then_some(dist)
}

fn point_triangle_distance(p: Point3<f64>, a: Point3<f64>, b: Point3<f64>, c: Point3<f64>) -> f64 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return ap.norm();
    }
    let bp = p - b;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 {
        return bp.norm();
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (a.coords + ab * v - p.coords).norm();
    }
    let cp = p - c;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0.0 && d5 <= d6 {
        return cp.norm();
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (a.coords + ac * w - p.coords).norm();
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (b.coords + (c.coords - b.coords) * w - p.coords).norm();
    }
    let denom = va + vb + vc;
    if denom.abs() <= 1e-18 {
        return ap.norm().min(bp.norm()).min(cp.norm());
    }
    let v = vb / denom;
    let w = vc / denom;
    (a.coords + ab * v + ac * w - p.coords).norm()
}

#[cfg(test)]
mod tests {
    use super::ControlledRemeshOp;
    use crate::{RepairOp, repair_mesh};
    use mesh_analysis::analyze;
    use mesh_attributes::{AttributeStore, Confidence, FaceOrigin};
    use mesh_core::{RepairProfile, TriangleMesh};
    use nalgebra::Point3;

    fn grid(sliver: bool) -> TriangleMesh {
        let mut vertices = Vec::new();
        for y in 0..4 {
            for x in 0..4 {
                vertices.push(Point3::new(x as f64, y as f64, 0.0));
            }
        }
        if sliver {
            vertices[6] = Point3::new(1.05, 1.0, 0.0);
        }
        let mut faces = Vec::new();
        for y in 0..3 {
            for x in 0..3 {
                let v00 = y * 4 + x;
                let v10 = v00 + 1;
                let v01 = v00 + 4;
                let v11 = v00 + 5;
                faces.push([v00, v10, v11]);
                faces.push([v00, v11, v01]);
            }
        }
        TriangleMesh::from_parts(vertices, faces)
    }

    fn min_edge(mesh: &TriangleMesh) -> f64 {
        mesh.edge_incidence()
            .values()
            .filter_map(|uses| {
                let (a, b) = (uses[0].directed.0, uses[0].directed.1);
                let pa = mesh.vertices.get(a as usize).copied()?;
                let pb = mesh.vertices.get(b as usize).copied()?;
                Some((pb - pa).norm())
            })
            .fold(f64::MAX, f64::min)
    }

    #[test]
    fn uniform_grid_is_not_remeshed() {
        let mesh = grid(false);
        let report = analyze(&mesh);
        let out = ControlledRemeshOp.apply(&mesh, &report).expect("op");
        assert_eq!(out.mesh, mesh);
        assert!(
            out.origins
                .iter()
                .all(|o| matches!(o, FaceOrigin::Identity(_)))
        );
    }

    #[test]
    fn sliver_collapse_marks_spatial_faces_and_keeps_distant_identity() {
        let mesh = grid(true);
        let report = analyze(&mesh);
        let out = ControlledRemeshOp.apply(&mesh, &report).expect("op");
        assert!(min_edge(&out.mesh) > 0.2, "short edge remained");
        assert!(
            out.origins
                .iter()
                .any(|o| matches!(o, FaceOrigin::Spatial { .. }))
        );
        assert!(
            out.origins
                .iter()
                .any(|o| matches!(o, FaceOrigin::Identity(_)))
        );
        let spatial = out
            .origins
            .iter()
            .filter(|o| matches!(o, FaceOrigin::Spatial { distance, .. } if *distance >= 0.0))
            .count();
        assert!(spatial > 0);
    }

    #[test]
    fn aggressive_warns_on_spatial_transfer_and_standard_does_not_remesh() {
        let mesh = grid(true);
        let mut attrs = AttributeStore::new_for_mesh(mesh.faces.len());
        let mat = attrs.intern_material("pla");
        for face in &mut attrs.faces {
            face.material_id = Some(mat);
        }

        let standard = repair_mesh(&mesh, &attrs, RepairProfile::Standard, None).expect("std");
        assert!(
            standard
                .report
                .kept_steps
                .iter()
                .all(|step| step.name != "Controlled remesh")
        );
        assert!(min_edge(&standard.mesh) < 0.1);

        let aggressive = repair_mesh(&mesh, &attrs, RepairProfile::Aggressive, None).expect("agg");
        assert!(
            aggressive
                .report
                .kept_steps
                .iter()
                .any(|step| step.name == "Controlled remesh"),
            "rejected: {:?}",
            aggressive.report.rejected_steps
        );
        assert!(min_edge(&aggressive.mesh) > 0.2);
        assert!(
            aggressive
                .attributes
                .confidence
                .contains(&Confidence::Uncertain)
        );
        assert!(
            aggressive
                .attributes
                .confidence
                .contains(&Confidence::Exact)
        );
        assert!(
            aggressive
                .report
                .warnings
                .iter()
                .any(|warning| warning.contains("spatial proximity"))
        );
    }
}
