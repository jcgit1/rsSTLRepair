//! Non-mutating topology and geometry diagnostics for [`TriangleMesh`].

use mesh_core::{BoundingBox, EdgeKey, EdgeUse, FaceId, RepairProfile, TriangleMesh, VertexId};
use nalgebra::{Point3, Vector3};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, VecDeque};

/// Severity of a diagnostic finding.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum Severity {
    Info,
    Warning,
    Error,
}

/// Classification of detected mesh conditions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DefectKind {
    NonFiniteCoordinates,
    OutOfRangeFaceIndices,
    ExactDuplicateVertices,
    DuplicateFaces,
    ZeroAreaTriangles,
    BoundaryEdges,
    NonManifoldEdges,
    WindingConflicts,
    InsideOutShell,
    NonManifoldVertices,
    DisconnectedComponents,
}

/// One structured diagnostic finding with severity and repairability metadata.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DiagnosticFinding {
    pub kind: DefectKind,
    pub severity: Severity,
    pub count: usize,
    pub message: String,
    pub min_profile_to_repair: Option<RepairProfile>,
}

/// Summary of an edge shared by 3 or more triangles.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct NonManifoldEdge {
    pub edge: EdgeKey,
    pub face_count: usize,
}

/// Summary of a connected component (shell).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ComponentInfo {
    pub id: usize,
    pub face_count: usize,
    pub surface_area: f64,
    pub signed_volume: f64,
    pub is_closed: bool,
}

/// A traced chain or closed loop of boundary edges.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BoundaryLoop {
    pub vertices: Vec<VertexId>,
    pub boundary_faces: Vec<FaceId>,
    pub is_closed: bool,
    pub diameter: f64,
    pub planarity_error: f64,
}

/// Complete non-mutating diagnostic report for a [`TriangleMesh`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DiagnosticReport {
    pub vertex_count: usize,
    pub face_count: usize,
    pub non_finite_vertices: Vec<VertexId>,
    pub out_of_range_faces: Vec<FaceId>,
    pub exact_duplicate_vertex_count: usize,
    pub duplicate_faces: Vec<FaceId>,
    pub zero_area_faces: Vec<FaceId>,
    pub manifold_edge_count: usize,
    pub boundary_edges: Vec<EdgeKey>,
    pub non_manifold_edges: Vec<NonManifoldEdge>,
    pub winding_conflict_edges: Vec<EdgeKey>,
    pub non_manifold_vertices: Vec<VertexId>,
    pub components: Vec<ComponentInfo>,
    pub boundary_loops: Vec<BoundaryLoop>,
    pub bounding_box: Option<BoundingBox>,
    pub surface_area: f64,
    pub signed_volume: f64,
    pub is_watertight: bool,
    pub is_valid_open_surface: bool,
    pub findings: Vec<DiagnosticFinding>,
}

impl DiagnosticReport {
    /// True when the mesh has zero actionable defects (other than optional open boundaries).
    #[must_use]
    pub fn has_errors(&self) -> bool {
        self.findings.iter().any(|f| f.severity == Severity::Error)
    }

    /// Format a concise multi-line summary for CLI and reports.
    #[must_use]
    pub fn summary_text(&self) -> String {
        let mut lines = Vec::new();
        lines.push(format!(
            "Vertices: {} | Faces: {} | Components: {}",
            self.vertex_count,
            self.face_count,
            self.components.len()
        ));
        lines.push(format!(
            "Edges: {} manifold, {} boundary, {} non-manifold, {} winding conflicts",
            self.manifold_edge_count,
            self.boundary_edges.len(),
            self.non_manifold_edges.len(),
            self.winding_conflict_edges.len()
        ));
        lines.push(format!(
            "Status: {}",
            if self.is_watertight {
                "Watertight solid"
            } else if self.is_valid_open_surface {
                "Valid open surface (intentional or unclosed boundary)"
            } else {
                "Defects detected"
            }
        ));
        if let Some(bbox) = self.bounding_box {
            let size = bbox.size();
            lines.push(format!(
                "Dimensions: {:.3} x {:.3} x {:.3} (area {:.3}, signed vol {:.3})",
                size.x, size.y, size.z, self.surface_area, self.signed_volume
            ));
        }
        for f in &self.findings {
            let sev = match f.severity {
                Severity::Info => "INFO",
                Severity::Warning => "WARN",
                Severity::Error => "ERR ",
            };
            lines.push(format!("  [{sev}] {}", f.message));
        }
        lines.join("\n")
    }
}

/// Analyze a [`TriangleMesh`] without mutating it.
#[must_use]
pub fn analyze(mesh: &TriangleMesh) -> DiagnosticReport {
    let vertex_count = mesh.vertices.len();
    let face_count = mesh.faces.len();

    let mut non_finite_vertices = Vec::new();
    let mut unique_coords: BTreeSet<(u64, u64, u64)> = BTreeSet::new();
    let mut exact_duplicate_vertex_count = 0usize;

    for (vi, p) in mesh.vertices.iter().enumerate() {
        if !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite() {
            non_finite_vertices.push(VertexId(vi as u32));
        } else {
            let key = (
                canonical_f64_bits(p.x),
                canonical_f64_bits(p.y),
                canonical_f64_bits(p.z),
            );
            if !unique_coords.insert(key) {
                exact_duplicate_vertex_count += 1;
            }
        }
    }

    let mut out_of_range_faces = Vec::new();
    let mut zero_area_faces = Vec::new();
    let mut duplicate_faces = Vec::new();
    let mut seen_triples: BTreeSet<[u32; 3]> = BTreeSet::new();

    for (fi, &[a, b, c]) in mesh.faces.iter().enumerate() {
        let fid = FaceId(fi as u32);
        if a as usize >= vertex_count || b as usize >= vertex_count || c as usize >= vertex_count {
            out_of_range_faces.push(fid);
            continue;
        }
        if a == b || b == c || c == a {
            zero_area_faces.push(fid);
        } else {
            let pa = mesh.vertices[a as usize];
            let pb = mesh.vertices[b as usize];
            let pc = mesh.vertices[c as usize];
            if !pa.x.is_finite()
                || !pa.y.is_finite()
                || !pa.z.is_finite()
                || !pb.x.is_finite()
                || !pb.y.is_finite()
                || !pb.z.is_finite()
                || !pc.x.is_finite()
                || !pc.y.is_finite()
                || !pc.z.is_finite()
                || TriangleMesh::triangle_area(pa, pb, pc) <= 1e-12
            {
                zero_area_faces.push(fid);
            }
        }

        let mut sorted = [a, b, c];
        sorted.sort_unstable();
        if !seen_triples.insert(sorted) {
            duplicate_faces.push(fid);
        }
    }

    let incidence = mesh.edge_incidence();
    let mut manifold_edge_count = 0usize;
    let mut boundary_edges = Vec::new();
    let mut non_manifold_edges = Vec::new();
    let mut winding_conflict_edges = Vec::new();

    for (&edge, uses) in &incidence {
        match uses.len() {
            1 => boundary_edges.push(edge),
            2 => {
                manifold_edge_count += 1;
                if uses[0].directed == uses[1].directed {
                    winding_conflict_edges.push(edge);
                }
            }
            n if n >= 3 => {
                non_manifold_edges.push(NonManifoldEdge {
                    edge,
                    face_count: n,
                });
            }
            _ => {}
        }
    }

    let non_manifold_vertices = find_non_manifold_vertices(mesh, &incidence);
    let components = find_components(mesh, &incidence);
    let boundary_loops = trace_boundary_loops(mesh, &boundary_edges, &incidence);

    let bounding_box = mesh.bounding_box();
    let surface_area = mesh.surface_area();
    let signed_volume = mesh.signed_volume();

    let has_closed_negative_shell = components
        .iter()
        .any(|c| c.is_closed && c.signed_volume < -1e-9);

    let is_watertight = face_count > 0
        && non_finite_vertices.is_empty()
        && out_of_range_faces.is_empty()
        && duplicate_faces.is_empty()
        && zero_area_faces.is_empty()
        && boundary_edges.is_empty()
        && non_manifold_edges.is_empty()
        && winding_conflict_edges.is_empty()
        && non_manifold_vertices.is_empty()
        && !has_closed_negative_shell;

    let is_valid_open_surface = face_count > 0
        && !is_watertight
        && non_finite_vertices.is_empty()
        && out_of_range_faces.is_empty()
        && duplicate_faces.is_empty()
        && zero_area_faces.is_empty()
        && non_manifold_edges.is_empty()
        && winding_conflict_edges.is_empty()
        && non_manifold_vertices.is_empty()
        && !boundary_edges.is_empty();

    let mut findings = Vec::new();
    if !non_finite_vertices.is_empty() {
        findings.push(DiagnosticFinding {
            kind: DefectKind::NonFiniteCoordinates,
            severity: Severity::Error,
            count: non_finite_vertices.len(),
            message: format!(
                "{} vertex/vertices have NaN or infinite coordinates",
                non_finite_vertices.len()
            ),
            min_profile_to_repair: Some(RepairProfile::Conservative),
        });
    }
    if !out_of_range_faces.is_empty() {
        findings.push(DiagnosticFinding {
            kind: DefectKind::OutOfRangeFaceIndices,
            severity: Severity::Error,
            count: out_of_range_faces.len(),
            message: format!(
                "{} face(s) reference out-of-range vertex indices",
                out_of_range_faces.len()
            ),
            min_profile_to_repair: Some(RepairProfile::Conservative),
        });
    }
    if exact_duplicate_vertex_count > 0 {
        findings.push(DiagnosticFinding {
            kind: DefectKind::ExactDuplicateVertices,
            severity: Severity::Warning,
            count: exact_duplicate_vertex_count,
            message: format!(
                "{exact_duplicate_vertex_count} exact duplicate vertex/vertices can be welded"
            ),
            min_profile_to_repair: Some(RepairProfile::Conservative),
        });
    }
    if !duplicate_faces.is_empty() {
        findings.push(DiagnosticFinding {
            kind: DefectKind::DuplicateFaces,
            severity: Severity::Error,
            count: duplicate_faces.len(),
            message: format!("{} duplicate triangle(s)", duplicate_faces.len()),
            min_profile_to_repair: Some(RepairProfile::Conservative),
        });
    }
    if !zero_area_faces.is_empty() {
        findings.push(DiagnosticFinding {
            kind: DefectKind::ZeroAreaTriangles,
            severity: Severity::Error,
            count: zero_area_faces.len(),
            message: format!(
                "{} zero-area or degenerate triangle(s)",
                zero_area_faces.len()
            ),
            min_profile_to_repair: Some(RepairProfile::Conservative),
        });
    }
    if !non_manifold_edges.is_empty() {
        findings.push(DiagnosticFinding {
            kind: DefectKind::NonManifoldEdges,
            severity: Severity::Error,
            count: non_manifold_edges.len(),
            message: format!(
                "{} non-manifold edge(s) shared by 3+ triangles",
                non_manifold_edges.len()
            ),
            min_profile_to_repair: Some(RepairProfile::Standard),
        });
    }
    if !non_manifold_vertices.is_empty() {
        findings.push(DiagnosticFinding {
            kind: DefectKind::NonManifoldVertices,
            severity: Severity::Error,
            count: non_manifold_vertices.len(),
            message: format!(
                "{} non-manifold (bowtie) vertex/vertices",
                non_manifold_vertices.len()
            ),
            min_profile_to_repair: Some(RepairProfile::Standard),
        });
    }
    if !winding_conflict_edges.is_empty() {
        findings.push(DiagnosticFinding {
            kind: DefectKind::WindingConflicts,
            severity: Severity::Error,
            count: winding_conflict_edges.len(),
            message: format!(
                "{} edge(s) have inconsistent neighbor face winding",
                winding_conflict_edges.len()
            ),
            min_profile_to_repair: Some(RepairProfile::Conservative),
        });
    }
    if has_closed_negative_shell {
        findings.push(DiagnosticFinding {
            kind: DefectKind::InsideOutShell,
            severity: Severity::Error,
            count: components
                .iter()
                .filter(|c| c.is_closed && c.signed_volume < -1e-9)
                .count(),
            message: "Closed shell has inverted (negative) signed volume".to_string(),
            min_profile_to_repair: Some(RepairProfile::Conservative),
        });
    }
    if !boundary_edges.is_empty() {
        let sev = if is_valid_open_surface {
            Severity::Info
        } else {
            Severity::Warning
        };
        let msg = if is_valid_open_surface {
            format!(
                "{} open boundary edge(s) across {} loop(s) (valid open surface; Conservative keeps openings)",
                boundary_edges.len(),
                boundary_loops.len()
            )
        } else {
            format!(
                "{} open boundary edge(s) across {} loop(s)",
                boundary_edges.len(),
                boundary_loops.len()
            )
        };
        findings.push(DiagnosticFinding {
            kind: DefectKind::BoundaryEdges,
            severity: sev,
            count: boundary_edges.len(),
            message: msg,
            min_profile_to_repair: Some(RepairProfile::Standard),
        });
    }
    if components.len() > 1 {
        findings.push(DiagnosticFinding {
            kind: DefectKind::DisconnectedComponents,
            severity: Severity::Info,
            count: components.len(),
            message: format!("{} disconnected shell component(s)", components.len()),
            min_profile_to_repair: Some(RepairProfile::Standard),
        });
    }

    DiagnosticReport {
        vertex_count,
        face_count,
        non_finite_vertices,
        out_of_range_faces,
        exact_duplicate_vertex_count,
        duplicate_faces,
        zero_area_faces,
        manifold_edge_count,
        boundary_edges,
        non_manifold_edges,
        winding_conflict_edges,
        non_manifold_vertices,
        components,
        boundary_loops,
        bounding_box,
        surface_area,
        signed_volume,
        is_watertight,
        is_valid_open_surface,
        findings,
    }
}

fn canonical_f64_bits(v: f64) -> u64 {
    if v == 0.0 {
        0.0_f64.to_bits()
    } else {
        v.to_bits()
    }
}

fn find_non_manifold_vertices(
    mesh: &TriangleMesh,
    incidence: &BTreeMap<EdgeKey, Vec<EdgeUse>>,
) -> Vec<VertexId> {
    let n_verts = mesh.vertices.len();
    let mut vertex_faces: Vec<Vec<u32>> = vec![Vec::new(); n_verts];
    for (fi, &[a, b, c]) in mesh.faces.iter().enumerate() {
        if a as usize >= n_verts || b as usize >= n_verts || c as usize >= n_verts {
            continue;
        }
        let f = fi as u32;
        vertex_faces[a as usize].push(f);
        if b != a {
            vertex_faces[b as usize].push(f);
        }
        if c != a && c != b {
            vertex_faces[c as usize].push(f);
        }
    }

    let mut result = Vec::new();
    for (vi, faces) in vertex_faces.iter().enumerate() {
        if faces.len() <= 1 {
            continue;
        }
        let v = vi as u32;
        let face_set: BTreeSet<u32> = faces.iter().copied().collect();
        let mut visited: BTreeSet<u32> = BTreeSet::new();
        let mut queue = VecDeque::new();
        let first = *face_set.iter().next().expect("non-empty");
        visited.insert(first);
        queue.push_back(first);

        while let Some(f) = queue.pop_front() {
            let [a, b, c] = mesh.faces[f as usize];
            let other_verts = [a, b, c].into_iter().filter(|&x| x != v);
            for ov in other_verts {
                if let Some(uses) = incidence.get(&EdgeKey::new(v, ov)) {
                    for u in uses {
                        if face_set.contains(&u.face.0) && visited.insert(u.face.0) {
                            queue.push_back(u.face.0);
                        }
                    }
                }
            }
        }

        if visited.len() != face_set.len() {
            result.push(VertexId(v));
        }
    }
    result
}

fn find_components(
    mesh: &TriangleMesh,
    incidence: &BTreeMap<EdgeKey, Vec<EdgeUse>>,
) -> Vec<ComponentInfo> {
    let n_verts = mesh.vertices.len();
    let n_faces = mesh.faces.len();
    let mut visited = vec![false; n_faces];
    let mut components = Vec::new();

    for start in 0..n_faces {
        if visited[start] {
            continue;
        }
        let [a, b, c] = mesh.faces[start];
        if a as usize >= n_verts || b as usize >= n_verts || c as usize >= n_verts {
            continue;
        }

        let mut queue = VecDeque::new();
        visited[start] = true;
        queue.push_back(start);

        let mut face_count = 0usize;
        let mut surface_area = 0.0;
        let mut signed_volume = 0.0;
        let mut is_closed = true;

        while let Some(fi) = queue.pop_front() {
            let [va, vb, vc] = mesh.faces[fi];
            if va as usize >= n_verts || vb as usize >= n_verts || vc as usize >= n_verts {
                continue;
            }
            face_count += 1;
            let pa = mesh.vertices[va as usize];
            let pb = mesh.vertices[vb as usize];
            let pc = mesh.vertices[vc as usize];
            surface_area += TriangleMesh::triangle_area(pa, pb, pc);
            let vol = pa.coords.dot(&pb.coords.cross(&pc.coords)) / 6.0;
            if vol.is_finite() {
                signed_volume += vol;
            }

            for (u, v) in [(va, vb), (vb, vc), (vc, va)] {
                if u == v {
                    continue;
                }
                if let Some(uses) = incidence.get(&EdgeKey::new(u, v)) {
                    if uses.len() != 2 {
                        is_closed = false;
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
        }

        components.push(ComponentInfo {
            id: components.len(),
            face_count,
            surface_area,
            signed_volume,
            is_closed,
        });
    }

    components
}

fn trace_boundary_loops(
    mesh: &TriangleMesh,
    boundary_edges: &[EdgeKey],
    incidence: &BTreeMap<EdgeKey, Vec<EdgeUse>>,
) -> Vec<BoundaryLoop> {
    if boundary_edges.is_empty() {
        return Vec::new();
    }

    // Build directed boundary half-edges: opposite to the triangle's own directed edge
    // so a hole loop walks in a consistent direction around the opening.
    let mut adj: BTreeMap<u32, Vec<(u32, EdgeKey, FaceId)>> = BTreeMap::new();
    for &ek in boundary_edges {
        if let Some(uses) = incidence.get(&ek)
            && uses.len() == 1
        {
            let u = uses[0].directed.1;
            let v = uses[0].directed.0;
            adj.entry(u).or_default().push((v, ek, uses[0].face));
        }
    }

    let mut used_edges: BTreeSet<EdgeKey> = BTreeSet::new();
    let mut loops = Vec::new();

    for &start_ek in boundary_edges {
        if used_edges.contains(&start_ek) {
            continue;
        }
        let Some(uses) = incidence.get(&start_ek) else {
            continue;
        };
        if uses.len() != 1 {
            continue;
        }

        let start_u = uses[0].directed.1;
        let first_v = uses[0].directed.0;
        let first_face = uses[0].face;
        used_edges.insert(start_ek);

        let mut verts = vec![VertexId(start_u)];
        let mut faces = vec![first_face];
        let mut curr = first_v;
        let mut is_closed = false;

        loop {
            if curr == start_u {
                is_closed = true;
                break;
            }
            verts.push(VertexId(curr));

            let next = adj.get(&curr).and_then(|neighbors| {
                neighbors
                    .iter()
                    .find(|&&(_, ek, _)| !used_edges.contains(&ek))
                    .copied()
            });
            match next {
                Some((nv, ek, fid)) => {
                    used_edges.insert(ek);
                    faces.push(fid);
                    curr = nv;
                }
                None => break,
            }
        }

        let (diameter, planarity_error) = loop_metrics(mesh, &verts);
        loops.push(BoundaryLoop {
            vertices: verts,
            boundary_faces: faces,
            is_closed,
            diameter,
            planarity_error,
        });
    }

    loops
}

fn loop_metrics(mesh: &TriangleMesh, verts: &[VertexId]) -> (f64, f64) {
    let pts: Vec<Point3<f64>> = verts
        .iter()
        .filter_map(|&v| mesh.vertex(v))
        .filter(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite())
        .collect();
    if pts.len() < 3 {
        return (0.0, 0.0);
    }

    let mut max_d2 = 0.0_f64;
    for i in 0..pts.len() {
        for j in (i + 1)..pts.len() {
            let d2 = (pts[i] - pts[j]).norm_squared();
            if d2 > max_d2 {
                max_d2 = d2;
            }
        }
    }
    let diameter = max_d2.sqrt();

    let centroid = pts.iter().fold(Vector3::zeros(), |acc, p| acc + p.coords) / (pts.len() as f64);

    // Newell normal across the polygon loop
    let mut normal = Vector3::<f64>::zeros();
    for i in 0..pts.len() {
        let a = pts[i].coords - centroid;
        let b = pts[(i + 1) % pts.len()].coords - centroid;
        normal.x += (a.y - b.y) * (a.z + b.z);
        normal.y += (a.z - b.z) * (a.x + b.x);
        normal.z += (a.x - b.x) * (a.y + b.y);
    }
    let norm_len = normal.norm();
    let planarity_error = if norm_len > 1e-12 {
        let unit = normal / norm_len;
        pts.iter()
            .map(|p| (p.coords - centroid).dot(&unit).abs())
            .fold(0.0_f64, f64::max)
    } else {
        0.0
    };

    (diameter, planarity_error)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unit_cube() -> TriangleMesh {
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(0.0, 1.0, 1.0),
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

    #[test]
    fn clean_cube_is_watertight() {
        let report = analyze(&unit_cube());
        assert!(report.is_watertight);
        assert!(!report.has_errors());
        assert_eq!(report.boundary_edges.len(), 0);
        assert_eq!(report.non_manifold_edges.len(), 0);
        assert_eq!(report.components.len(), 1);
    }

    #[test]
    fn open_sheet_is_classified_as_valid_open_surface() {
        let mesh = TriangleMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2], [0, 2, 3]],
        );
        let report = analyze(&mesh);
        assert!(!report.is_watertight);
        assert!(report.is_valid_open_surface);
        assert!(!report.has_errors());
        assert_eq!(report.boundary_edges.len(), 4);
        assert_eq!(report.boundary_loops.len(), 1);
        assert!(report.boundary_loops[0].is_closed);
    }

    #[test]
    fn detects_three_face_non_manifold_edge() {
        let mesh = TriangleMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, -1.0, 0.0),
                Point3::new(0.0, 0.0, 1.0),
            ],
            vec![[0, 1, 2], [1, 0, 3], [0, 1, 4]],
        );
        let report = analyze(&mesh);
        assert_eq!(report.non_manifold_edges.len(), 1);
        assert_eq!(report.non_manifold_edges[0].face_count, 3);
    }

    #[test]
    fn detects_bowtie_vertex() {
        let mesh = TriangleMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(-1.0, 0.0, 0.0),
                Point3::new(0.0, -1.0, 0.0),
            ],
            vec![[0, 1, 2], [0, 3, 4]],
        );
        let report = analyze(&mesh);
        assert_eq!(report.non_manifold_vertices, vec![VertexId(0)]);
    }

    #[test]
    fn detects_nan_zero_area_and_duplicates() {
        let mesh = TriangleMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(f64::NAN, 0.0, 0.0),
            ],
            vec![[0, 1, 2], [0, 1, 2], [0, 1, 1], [0, 1, 4]],
        );
        let report = analyze(&mesh);
        assert_eq!(report.non_finite_vertices, vec![VertexId(4)]);
        assert_eq!(report.exact_duplicate_vertex_count, 1);
        assert_eq!(report.duplicate_faces.len(), 1);
        assert_eq!(report.zero_area_faces.len(), 2);
    }
}
