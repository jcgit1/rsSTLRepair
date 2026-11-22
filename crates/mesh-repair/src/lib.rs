//! Conservative, standard, and aggressive mesh repair pipelines with face correspondence.

pub mod breakage;
mod phase2;
mod remesh;

pub use breakage::{BreakKind, BrokenSample, break_file};

use phase2::{SplitCrossingsOp, StitchTJunctionsOp, UnionOverlappingShellsOp};
use remesh::ControlledRemeshOp;

use mesh_analysis::{BoundaryLoop, ComponentInfo, DiagnosticReport, analyze};
use mesh_attributes::{AttributeStore, AttributeTransferSummary, FaceOrigin, compose_origins};
use mesh_core::{BoundingBox, EdgeKey, FaceId, MeshError, RepairProfile, TriangleMesh};
use nalgebra::{Point3, Vector2, Vector3};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeSet, HashMap, VecDeque};
use std::sync::atomic::{AtomicBool, Ordering};

/// Output produced by a single [`RepairOp`] prior to adopt-or-reject evaluation.
#[derive(Debug, Clone)]
pub struct OpOutput {
    pub mesh: TriangleMesh,
    pub origins: Vec<FaceOrigin>,
    pub summary: String,
    pub max_allowed_face_deletion: usize,
    pub allow_bbox_shrink: bool,
}

/// A discrete, composable topology or geometry repair operation.
pub trait RepairOp {
    fn name(&self) -> &'static str;
    fn apply(&self, mesh: &TriangleMesh, report: &DiagnosticReport) -> Result<OpOutput, MeshError>;
}

/// Record of one step evaluated during a repair pipeline run.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StepOutcome {
    pub name: String,
    pub adopted: bool,
    pub detail: String,
    pub faces_before: usize,
    pub faces_after: usize,
}

/// Full outcome of running [`repair_mesh`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RepairReport {
    pub profile: RepairProfile,
    pub before: DiagnosticReport,
    pub after: DiagnosticReport,
    pub kept_steps: Vec<StepOutcome>,
    pub rejected_steps: Vec<StepOutcome>,
    pub attribute_summary: AttributeTransferSummary,
    pub warnings: Vec<String>,
    pub unresolved_notes: Vec<String>,
    pub was_noop: bool,
    pub unsupported_features: Vec<String>,
}

/// Complete output from [`repair_mesh`].
#[derive(Debug, Clone)]
pub struct RepairResult {
    pub mesh: TriangleMesh,
    pub attributes: AttributeStore,
    pub report: RepairReport,
}

/// List of geometry/format features deferred beyond this release.
#[must_use]
pub fn unsupported_features_list() -> Vec<String> {
    vec![
        "Analytic STEP solids (cylinders, NURBS, and non-triangular faces) are not meshed"
            .to_string(),
        "SVG curves, transforms, and holes are not imported; accepted shapes are extruded 1 mm"
            .to_string(),
        "Thin walls are reported and left in place so intentional shells are not deleted"
            .to_string(),
    ]
}

/// Run the repair pipeline for `profile` on `mesh` and `attributes`.
///
/// Clean watertight inputs with no exact duplicate vertices are returned untouched
/// (`was_noop = true`). Every individual repair step is reanalyzed before adoption and
/// rolled back if it worsens non-finite counts, non-manifold edges, or bounding-box drift.
pub fn repair_mesh(
    mesh: &TriangleMesh,
    attributes: &AttributeStore,
    profile: RepairProfile,
    cancel: Option<&AtomicBool>,
) -> Result<RepairResult, MeshError> {
    let before = analyze(mesh);

    if before.is_watertight
        && before.exact_duplicate_vertex_count == 0
        && before.t_junctions.is_empty()
        && !before.has_errors()
    {
        let attr_summary = AttributeTransferSummary {
            exact_count: attributes.faces.len(),
            ..Default::default()
        };
        let mut unresolved_notes = Vec::new();
        if !before.thin_walls.is_empty() {
            unresolved_notes.push(format!(
                "{} thin-wall face pair(s) reported and left unchanged",
                before.thin_walls.len()
            ));
        }
        return Ok(RepairResult {
            mesh: mesh.clone(),
            attributes: attributes.clone(),
            report: RepairReport {
                profile,
                before: before.clone(),
                after: before,
                kept_steps: Vec::new(),
                rejected_steps: Vec::new(),
                attribute_summary: attr_summary,
                warnings: Vec::new(),
                unresolved_notes,
                was_noop: true,
                unsupported_features: unsupported_features_list(),
            },
        });
    }

    let mut current_mesh = mesh.clone();
    let mut current_report = before.clone();
    let mut cumulative_origins: Vec<FaceOrigin> = (0..mesh.faces.len())
        .map(|i| FaceOrigin::Identity(FaceId(i as u32)))
        .collect();

    let mut kept_steps = Vec::new();
    let mut rejected_steps = Vec::new();
    let mut warnings = Vec::new();

    if profile == RepairProfile::Aggressive {
        warnings.push(
            "Aggressive profile enabled: wider hole caps, small-fragment pruning, and local remeshing may alter fine surface detail. Faces that lose exact correspondence are marked uncertain."
                .to_string(),
        );
    }

    let steps = build_pipeline(profile);
    for op in steps {
        if cancel.is_some_and(|c| c.load(Ordering::Relaxed)) {
            return Err(MeshError::Cancelled);
        }

        let faces_before = current_mesh.faces.len();
        let output = op.apply(&current_mesh, &current_report)?;

        // Fast-path: no-op step
        if output.mesh == current_mesh {
            continue;
        }

        let cand_report = analyze(&output.mesh);
        match check_adopt(
            &current_mesh,
            &current_report,
            &output,
            &cand_report,
            profile,
        ) {
            Ok(()) => {
                cumulative_origins = compose_origins(&cumulative_origins, &output.origins);
                let faces_after = output.mesh.faces.len();
                current_mesh = output.mesh;
                current_report = cand_report;
                kept_steps.push(StepOutcome {
                    name: op.name().to_string(),
                    adopted: true,
                    detail: output.summary,
                    faces_before,
                    faces_after,
                });
            }
            Err(reason) => {
                let faces_after = output.mesh.faces.len();
                rejected_steps.push(StepOutcome {
                    name: op.name().to_string(),
                    adopted: false,
                    detail: format!("Rejected ({reason}): {}", output.summary),
                    faces_before,
                    faces_after,
                });
                warnings.push(format!("Step '{}' rejected: {reason}", op.name()));
            }
        }
    }

    let (next_attrs, attribute_summary) = attributes.transfer(&cumulative_origins);
    warnings.extend(attribute_summary.warnings.iter().cloned());

    let mut unresolved_notes = Vec::new();
    if !current_report.non_manifold_edges.is_empty() {
        unresolved_notes.push(format!(
            "{} non-manifold edge(s) remain unresolved (ambiguous topology preserved rather than guessed)",
            current_report.non_manifold_edges.len()
        ));
    }
    if !current_report.non_manifold_vertices.is_empty() {
        unresolved_notes.push(format!(
            "{} non-manifold vertex/vertices remain unresolved",
            current_report.non_manifold_vertices.len()
        ));
    }
    if !current_report.boundary_edges.is_empty() {
        unresolved_notes.push(format!(
            "{} boundary edge(s) across {} loop(s) remain open",
            current_report.boundary_edges.len(),
            current_report.boundary_loops.len()
        ));
    }
    if !current_report.winding_conflict_edges.is_empty() {
        unresolved_notes.push(format!(
            "{} winding conflict edge(s) remain (typically around non-orientable or non-manifold regions)",
            current_report.winding_conflict_edges.len()
        ));
    }
    if !current_report.self_intersections.is_empty() {
        unresolved_notes.push(format!(
            "{} self-intersection(s) remain",
            current_report.self_intersections.len()
        ));
    }
    if !current_report.t_junctions.is_empty() {
        unresolved_notes.push(format!(
            "{} T-junction(s) remain",
            current_report.t_junctions.len()
        ));
    }
    if !current_report.thin_walls.is_empty() {
        unresolved_notes.push(format!(
            "{} thin-wall face pair(s) reported and left in place",
            current_report.thin_walls.len()
        ));
    }

    Ok(RepairResult {
        mesh: current_mesh,
        attributes: next_attrs,
        report: RepairReport {
            profile,
            before,
            after: current_report,
            kept_steps,
            rejected_steps,
            attribute_summary,
            warnings,
            unresolved_notes,
            was_noop: false,
            unsupported_features: unsupported_features_list(),
        },
    })
}

fn check_adopt(
    prev_mesh: &TriangleMesh,
    prev_report: &DiagnosticReport,
    output: &OpOutput,
    cand_report: &DiagnosticReport,
    profile: RepairProfile,
) -> Result<(), String> {
    if cand_report.non_finite_vertices.len() > prev_report.non_finite_vertices.len() {
        return Err("introduced non-finite coordinates".to_string());
    }
    if cand_report.out_of_range_faces.len() > prev_report.out_of_range_faces.len() {
        return Err("introduced out-of-range face indices".to_string());
    }
    if cand_report.non_manifold_edges.len() > prev_report.non_manifold_edges.len() {
        return Err("increased non-manifold edge count".to_string());
    }
    if cand_report.self_intersections.len() > prev_report.self_intersections.len() {
        return Err("increased self-intersections".to_string());
    }
    let deleted = prev_mesh
        .faces
        .len()
        .saturating_sub(output.mesh.faces.len());
    if deleted > output.max_allowed_face_deletion {
        return Err(format!(
            "deleted {deleted} faces, exceeding step cap {}",
            output.max_allowed_face_deletion
        ));
    }

    if !output.allow_bbox_shrink
        && let (Some(b0), Some(b1)) = (prev_report.bounding_box, cand_report.bounding_box)
    {
        let drift = BoundingBox::max_corner_drift(&b0, &b1);
        let tol = match profile {
            RepairProfile::Conservative => 1e-12,
            RepairProfile::Standard => 2e-4,
            RepairProfile::Aggressive => 5e-4,
        };
        if drift > tol {
            return Err(format!(
                "bounding-box drift {drift:.6} exceeded tolerance {tol:.6}"
            ));
        }
    }
    Ok(())
}

fn build_pipeline(profile: RepairProfile) -> Vec<Box<dyn RepairOp>> {
    match profile {
        RepairProfile::Conservative => vec![
            Box::new(CleanInvalidOp),
            Box::new(WeldVerticesOp { epsilon: 0.0 }),
            Box::new(RemoveDegenerateAndDuplicateFacesOp),
            Box::new(FixWindingOp),
        ],
        RepairProfile::Standard => vec![
            Box::new(CleanInvalidOp),
            Box::new(WeldVerticesOp { epsilon: 1e-4 }),
            Box::new(RemoveDegenerateAndDuplicateFacesOp),
            Box::new(SplitBowtieVerticesOp),
            Box::new(FixWindingOp),
            Box::new(StitchTJunctionsOp),
            Box::new(SplitCrossingsOp),
            Box::new(RemoveSimpleFinsOp),
            Box::new(FillSmallHolesOp {
                max_edges: 16,
                max_diameter_ratio: 0.05,
                max_planarity_ratio: 0.05,
            }),
            Box::new(FixWindingOp),
            Box::new(PruneLooseOpenComponentsOp {
                max_faces: Some(3),
                max_area_ratio: None,
            }),
        ],
        RepairProfile::Aggressive => vec![
            Box::new(CleanInvalidOp),
            Box::new(WeldVerticesOp { epsilon: 1e-4 }),
            Box::new(RemoveDegenerateAndDuplicateFacesOp),
            Box::new(SplitBowtieVerticesOp),
            Box::new(FixWindingOp),
            Box::new(StitchTJunctionsOp),
            Box::new(SplitCrossingsOp),
            Box::new(UnionOverlappingShellsOp),
            Box::new(RemoveSimpleFinsOp),
            Box::new(FillSmallHolesOp {
                max_edges: 64,
                max_diameter_ratio: 0.20,
                max_planarity_ratio: 0.15,
            }),
            Box::new(FixWindingOp),
            Box::new(PruneLooseOpenComponentsOp {
                max_faces: None,
                max_area_ratio: Some(0.01),
            }),
            Box::new(ControlledRemeshOp),
        ],
    }
}

struct CleanInvalidOp;

impl RepairOp for CleanInvalidOp {
    fn name(&self) -> &'static str {
        "Remove invalid indices and non-finite geometry"
    }

    fn apply(&self, mesh: &TriangleMesh, report: &DiagnosticReport) -> Result<OpOutput, MeshError> {
        if report.non_finite_vertices.is_empty() && report.out_of_range_faces.is_empty() {
            return Ok(OpOutput {
                mesh: mesh.clone(),
                origins: (0..mesh.faces.len())
                    .map(|i| FaceOrigin::Identity(FaceId(i as u32)))
                    .collect(),
                summary: "none".to_string(),
                max_allowed_face_deletion: 0,
                allow_bbox_shrink: false,
            });
        }

        let n_verts = mesh.vertices.len();
        let valid_vert = |idx: u32| -> bool {
            let Some(p) = mesh.vertices.get(idx as usize) else {
                return false;
            };
            p.x.is_finite() && p.y.is_finite() && p.z.is_finite()
        };

        let mut new_verts = Vec::new();
        let mut remap = vec![u32::MAX; n_verts];
        let mut new_faces = Vec::new();
        let mut origins = Vec::new();
        let mut dropped = 0usize;

        for (fi, &[a, b, c]) in mesh.faces.iter().enumerate() {
            if !valid_vert(a) || !valid_vert(b) || !valid_vert(c) {
                dropped += 1;
                continue;
            }
            let mut tri = [0u32; 3];
            for (k, &old_v) in [a, b, c].iter().enumerate() {
                if remap[old_v as usize] == u32::MAX {
                    remap[old_v as usize] = new_verts.len() as u32;
                    new_verts.push(mesh.vertices[old_v as usize]);
                }
                tri[k] = remap[old_v as usize];
            }
            new_faces.push(tri);
            origins.push(FaceOrigin::Identity(FaceId(fi as u32)));
        }

        Ok(OpOutput {
            mesh: TriangleMesh::from_parts(new_verts, new_faces),
            origins,
            summary: format!("removed {dropped} invalid/non-finite face(s)"),
            max_allowed_face_deletion: dropped,
            allow_bbox_shrink: false,
        })
    }
}

struct WeldVerticesOp {
    epsilon: f64,
}

impl RepairOp for WeldVerticesOp {
    fn name(&self) -> &'static str {
        if self.epsilon == 0.0 {
            "Weld bitwise-identical vertices"
        } else {
            "Weld near-duplicate vertices (1e-4 mm)"
        }
    }

    fn apply(
        &self,
        mesh: &TriangleMesh,
        _report: &DiagnosticReport,
    ) -> Result<OpOutput, MeshError> {
        let n_verts = mesh.vertices.len();
        if n_verts == 0 {
            return Ok(OpOutput {
                mesh: mesh.clone(),
                origins: Vec::new(),
                summary: "none".to_string(),
                max_allowed_face_deletion: 0,
                allow_bbox_shrink: false,
            });
        }

        let mut old_to_new: Vec<u32> = Vec::with_capacity(n_verts);
        let mut new_verts: Vec<Point3<f64>> = Vec::with_capacity(n_verts);

        if self.epsilon <= 0.0 {
            let mut exact_map: HashMap<(u64, u64, u64), u32> = HashMap::with_capacity(n_verts);
            for &p in &mesh.vertices {
                if p.x.is_finite() && p.y.is_finite() && p.z.is_finite() {
                    let key = (canon_bits(p.x), canon_bits(p.y), canon_bits(p.z));
                    if let Some(&id) = exact_map.get(&key) {
                        old_to_new.push(id);
                    } else {
                        let id = new_verts.len() as u32;
                        new_verts.push(Point3::new(
                            if p.x == 0.0 { 0.0 } else { p.x },
                            if p.y == 0.0 { 0.0 } else { p.y },
                            if p.z == 0.0 { 0.0 } else { p.z },
                        ));
                        exact_map.insert(key, id);
                        old_to_new.push(id);
                    }
                } else {
                    let id = new_verts.len() as u32;
                    new_verts.push(p);
                    old_to_new.push(id);
                }
            }
        } else {
            let eps = self.epsilon;
            let eps2 = eps * eps;
            let cell = eps;
            let mut grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::with_capacity(n_verts);

            for &p in &mesh.vertices {
                if !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite() {
                    let id = new_verts.len() as u32;
                    new_verts.push(p);
                    old_to_new.push(id);
                    continue;
                }
                let cx = (p.x / cell).floor() as i64;
                let cy = (p.y / cell).floor() as i64;
                let cz = (p.z / cell).floor() as i64;

                let mut found: Option<u32> = None;
                let mut best_d2 = eps2;
                for dx in -1..=1 {
                    for dy in -1..=1 {
                        for dz in -1..=1 {
                            if let Some(bucket) = grid.get(&(cx + dx, cy + dy, cz + dz)) {
                                for &cand_id in bucket {
                                    let d2 = (new_verts[cand_id as usize] - p).norm_squared();
                                    if d2 <= best_d2 {
                                        best_d2 = d2;
                                        found = Some(cand_id);
                                    }
                                }
                            }
                        }
                    }
                }

                if let Some(id) = found {
                    old_to_new.push(id);
                } else {
                    let id = new_verts.len() as u32;
                    new_verts.push(p);
                    grid.entry((cx, cy, cz)).or_default().push(id);
                    old_to_new.push(id);
                }
            }
        }

        let welded = n_verts.saturating_sub(new_verts.len());
        if welded == 0 {
            return Ok(OpOutput {
                mesh: mesh.clone(),
                origins: (0..mesh.faces.len())
                    .map(|i| FaceOrigin::Identity(FaceId(i as u32)))
                    .collect(),
                summary: "no vertices welded".to_string(),
                max_allowed_face_deletion: 0,
                allow_bbox_shrink: false,
            });
        }

        let mut new_faces = Vec::with_capacity(mesh.faces.len());
        let mut origins = Vec::with_capacity(mesh.faces.len());
        for (fi, &[a, b, c]) in mesh.faces.iter().enumerate() {
            new_faces.push([
                old_to_new[a as usize],
                old_to_new[b as usize],
                old_to_new[c as usize],
            ]);
            origins.push(FaceOrigin::Identity(FaceId(fi as u32)));
        }

        Ok(OpOutput {
            mesh: TriangleMesh::from_parts(new_verts, new_faces),
            origins,
            summary: format!("welded {welded} vertex/vertices"),
            max_allowed_face_deletion: 0,
            allow_bbox_shrink: false,
        })
    }
}

fn canon_bits(v: f64) -> u64 {
    if v == 0.0 {
        0.0_f64.to_bits()
    } else {
        v.to_bits()
    }
}

struct RemoveDegenerateAndDuplicateFacesOp;

impl RepairOp for RemoveDegenerateAndDuplicateFacesOp {
    fn name(&self) -> &'static str {
        "Remove degenerate and duplicate faces"
    }

    fn apply(
        &self,
        mesh: &TriangleMesh,
        _report: &DiagnosticReport,
    ) -> Result<OpOutput, MeshError> {
        let mut seen: BTreeSet<[u32; 3]> = BTreeSet::new();
        let mut new_faces = Vec::with_capacity(mesh.faces.len());
        let mut origins = Vec::with_capacity(mesh.faces.len());
        let mut removed_degen = 0usize;
        let mut removed_dup = 0usize;

        for (fi, &[a, b, c]) in mesh.faces.iter().enumerate() {
            if a == b || b == c || c == a {
                removed_degen += 1;
                continue;
            }
            let Some([pa, pb, pc]) = mesh.face_positions(FaceId(fi as u32)) else {
                removed_degen += 1;
                continue;
            };
            if TriangleMesh::triangle_area(pa, pb, pc) <= 1e-12 {
                removed_degen += 1;
                continue;
            }
            let mut key = [a, b, c];
            key.sort_unstable();
            if !seen.insert(key) {
                removed_dup += 1;
                continue;
            }
            new_faces.push([a, b, c]);
            origins.push(FaceOrigin::Identity(FaceId(fi as u32)));
        }

        let (compacted_verts, compacted_faces) =
            compact_unused_vertices(&mesh.vertices, &new_faces);
        let total_removed = removed_degen + removed_dup;
        Ok(OpOutput {
            mesh: TriangleMesh::from_parts(compacted_verts, compacted_faces),
            origins,
            summary: format!(
                "removed {removed_degen} degenerate and {removed_dup} duplicate face(s)"
            ),
            max_allowed_face_deletion: total_removed,
            allow_bbox_shrink: false,
        })
    }
}

struct FixWindingOp;

impl RepairOp for FixWindingOp {
    fn name(&self) -> &'static str {
        "Harmonize face winding and outward shell orientation"
    }

    fn apply(
        &self,
        mesh: &TriangleMesh,
        _report: &DiagnosticReport,
    ) -> Result<OpOutput, MeshError> {
        let n_faces = mesh.faces.len();
        if n_faces == 0 {
            return Ok(OpOutput {
                mesh: mesh.clone(),
                origins: Vec::new(),
                summary: "none".to_string(),
                max_allowed_face_deletion: 0,
                allow_bbox_shrink: false,
            });
        }

        let mut faces = mesh.faces.clone();
        let incidence = mesh.edge_incidence();

        // Build adjacency across manifold (2-face) edges only so non-manifold edges don't poison orientation.
        let mut adj: Vec<Vec<(usize, EdgeKey)>> = vec![Vec::new(); n_faces];
        for (&ek, uses) in &incidence {
            if uses.len() == 2 {
                let f0 = uses[0].face.0 as usize;
                let f1 = uses[1].face.0 as usize;
                adj[f0].push((f1, ek));
                adj[f1].push((f0, ek));
            }
        }

        let mut visited = vec![false; n_faces];
        let mut flipped_local = 0usize;
        let mut flipped_shells = 0usize;

        for start in 0..n_faces {
            if visited[start] {
                continue;
            }
            visited[start] = true;
            let mut queue = VecDeque::new();
            queue.push_back(start);
            let mut component_faces = vec![start];
            let mut parity: HashMap<usize, bool> = HashMap::new();
            parity.insert(start, false);
            let mut parity_true_count = 0usize;

            while let Some(fi) = queue.pop_front() {
                let p_i = parity[&fi];
                let mut tri_i = faces[fi];
                if p_i {
                    tri_i.swap(1, 2);
                }
                for &(nf, ek) in &adj[fi] {
                    if visited[nf] {
                        continue;
                    }
                    visited[nf] = true;
                    let dir_i = directed_on_face(tri_i, ek);
                    let dir_n = directed_on_face(faces[nf], ek);
                    let flip_n = matches!((dir_i, dir_n), (Some(di), Some(dn)) if di == dn);
                    if flip_n {
                        parity_true_count += 1;
                    }
                    parity.insert(nf, flip_n);
                    component_faces.push(nf);
                    queue.push_back(nf);
                }
            }

            // Flip whichever parity is the minority so 1 inverted face flips 1 face, not N-1.
            let flip_target = parity_true_count <= component_faces.len() / 2;
            for &fi in &component_faces {
                if parity[&fi] == flip_target {
                    faces[fi].swap(1, 2);
                    flipped_local += 1;
                }
            }

            // Check if this component is closed (every edge of every face in it has incidence == 2)
            let is_closed = component_faces.iter().all(|&fi| {
                let [a, b, c] = faces[fi];
                [(a, b), (b, c), (c, a)].into_iter().all(|(u, v)| {
                    incidence
                        .get(&EdgeKey::new(u, v))
                        .is_some_and(|uses| uses.len() == 2)
                })
            });

            if is_closed {
                let mut vol = 0.0;
                for &fi in &component_faces {
                    let [a, b, c] = faces[fi];
                    let pa = mesh.vertices[a as usize];
                    let pb = mesh.vertices[b as usize];
                    let pc = mesh.vertices[c as usize];
                    vol += pa.coords.dot(&pb.coords.cross(&pc.coords)) / 6.0;
                }
                if vol < -1e-9 {
                    for &fi in &component_faces {
                        faces[fi].swap(1, 2);
                    }
                    flipped_shells += 1;
                }
            }
        }

        let origins = (0..n_faces)
            .map(|i| FaceOrigin::Identity(FaceId(i as u32)))
            .collect();

        Ok(OpOutput {
            mesh: TriangleMesh::from_parts(mesh.vertices.clone(), faces),
            origins,
            summary: format!(
                "flipped {flipped_local} local triangle(s) and {flipped_shells} inverted closed shell(s)"
            ),
            max_allowed_face_deletion: 0,
            allow_bbox_shrink: false,
        })
    }
}

fn directed_on_face(tri: [u32; 3], ek: EdgeKey) -> Option<(u32, u32)> {
    for (u, v) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
        if EdgeKey::new(u, v) == ek {
            return Some((u, v));
        }
    }
    None
}

struct SplitBowtieVerticesOp;

impl RepairOp for SplitBowtieVerticesOp {
    fn name(&self) -> &'static str {
        "Split bowtie (non-manifold) vertices"
    }

    fn apply(&self, mesh: &TriangleMesh, report: &DiagnosticReport) -> Result<OpOutput, MeshError> {
        if report.non_manifold_vertices.is_empty() {
            return Ok(OpOutput {
                mesh: mesh.clone(),
                origins: (0..mesh.faces.len())
                    .map(|i| FaceOrigin::Identity(FaceId(i as u32)))
                    .collect(),
                summary: "none".to_string(),
                max_allowed_face_deletion: 0,
                allow_bbox_shrink: false,
            });
        }

        let mut vertices = mesh.vertices.clone();
        let mut faces = mesh.faces.clone();
        let incidence = mesh.edge_incidence();
        let mut split_count = 0usize;

        for &vid in &report.non_manifold_vertices {
            let v = vid.0;
            let incident: Vec<usize> = faces
                .iter()
                .enumerate()
                .filter_map(|(fi, &[a, b, c])| {
                    if a == v || b == v || c == v {
                        Some(fi)
                    } else {
                        None
                    }
                })
                .collect();
            if incident.len() <= 1 {
                continue;
            }
            let inc_set: BTreeSet<usize> = incident.iter().copied().collect();
            let mut visited: BTreeSet<usize> = BTreeSet::new();
            let mut fans: Vec<Vec<usize>> = Vec::new();

            for &start in &incident {
                if !visited.insert(start) {
                    continue;
                }
                let mut fan = vec![start];
                let mut q = VecDeque::new();
                q.push_back(start);
                while let Some(fi) = q.pop_front() {
                    let [a, b, c] = faces[fi];
                    for ov in [a, b, c].into_iter().filter(|&x| x != v) {
                        if let Some(uses) = incidence.get(&EdgeKey::new(v, ov)) {
                            for u in uses {
                                let nf = u.face.0 as usize;
                                if inc_set.contains(&nf) && visited.insert(nf) {
                                    fan.push(nf);
                                    q.push_back(nf);
                                }
                            }
                        }
                    }
                }
                fans.push(fan);
            }

            if fans.len() > 1 {
                let pos = vertices[v as usize];
                for fan in fans.into_iter().skip(1) {
                    let new_v = vertices.len() as u32;
                    vertices.push(pos);
                    for fi in fan {
                        for idx in &mut faces[fi] {
                            if *idx == v {
                                *idx = new_v;
                            }
                        }
                    }
                    split_count += 1;
                }
            }
        }

        let origins = (0..faces.len())
            .map(|i| FaceOrigin::Identity(FaceId(i as u32)))
            .collect();

        Ok(OpOutput {
            mesh: TriangleMesh::from_parts(vertices, faces),
            origins,
            summary: format!("created {split_count} split vertex copy/copies"),
            max_allowed_face_deletion: 0,
            allow_bbox_shrink: false,
        })
    }
}

struct RemoveSimpleFinsOp;

impl RepairOp for RemoveSimpleFinsOp {
    fn name(&self) -> &'static str {
        "Remove unambiguous 3-face non-manifold fins"
    }

    fn apply(&self, mesh: &TriangleMesh, report: &DiagnosticReport) -> Result<OpOutput, MeshError> {
        if report.non_manifold_edges.is_empty() {
            return Ok(OpOutput {
                mesh: mesh.clone(),
                origins: (0..mesh.faces.len())
                    .map(|i| FaceOrigin::Identity(FaceId(i as u32)))
                    .collect(),
                summary: "none".to_string(),
                max_allowed_face_deletion: 0,
                allow_bbox_shrink: false,
            });
        }

        let incidence = mesh.edge_incidence();
        let mut to_drop: BTreeSet<usize> = BTreeSet::new();

        for nm in &report.non_manifold_edges {
            if nm.face_count != 3 {
                continue;
            }
            let Some(uses) = incidence.get(&nm.edge) else {
                continue;
            };
            if uses.len() != 3 {
                continue;
            }
            let areas: Vec<f64> = uses
                .iter()
                .map(|u| {
                    mesh.face_positions(u.face)
                        .map_or(0.0, |[a, b, c]| TriangleMesh::triangle_area(a, b, c))
                })
                .collect();

            // Find a single candidate face to drop whose remaining two have opposite directed edges
            // and whose area is strictly smaller than both remaining faces.
            let mut best_drop: Option<usize> = None;
            for drop_idx in 0..3 {
                let keep_a = (drop_idx + 1) % 3;
                let keep_b = (drop_idx + 2) % 3;
                if uses[keep_a].directed != uses[keep_b].directed
                    && areas[drop_idx] < areas[keep_a]
                    && areas[drop_idx] < areas[keep_b]
                {
                    best_drop = Some(uses[drop_idx].face.0 as usize);
                    break;
                }
            }
            if let Some(fi) = best_drop {
                to_drop.insert(fi);
            }
        }

        if to_drop.is_empty() {
            return Ok(OpOutput {
                mesh: mesh.clone(),
                origins: (0..mesh.faces.len())
                    .map(|i| FaceOrigin::Identity(FaceId(i as u32)))
                    .collect(),
                summary: "no unambiguous 3-face fins".to_string(),
                max_allowed_face_deletion: 0,
                allow_bbox_shrink: false,
            });
        }

        let mut new_faces = Vec::with_capacity(mesh.faces.len() - to_drop.len());
        let mut origins = Vec::with_capacity(mesh.faces.len() - to_drop.len());
        for (fi, &tri) in mesh.faces.iter().enumerate() {
            if !to_drop.contains(&fi) {
                new_faces.push(tri);
                origins.push(FaceOrigin::Identity(FaceId(fi as u32)));
            }
        }

        let (verts, faces) = compact_unused_vertices(&mesh.vertices, &new_faces);
        let dropped = to_drop.len();
        Ok(OpOutput {
            mesh: TriangleMesh::from_parts(verts, faces),
            origins,
            summary: format!("removed {dropped} unambiguous non-manifold fin face(s)"),
            max_allowed_face_deletion: dropped,
            allow_bbox_shrink: true,
        })
    }
}

struct FillSmallHolesOp {
    max_edges: usize,
    max_diameter_ratio: f64,
    max_planarity_ratio: f64,
}

impl RepairOp for FillSmallHolesOp {
    fn name(&self) -> &'static str {
        "Fill small planar boundary holes"
    }

    fn apply(&self, mesh: &TriangleMesh, report: &DiagnosticReport) -> Result<OpOutput, MeshError> {
        let diag = report.bounding_box.map_or(0.0, |b| b.diagonal()).max(1e-12);
        let max_diam = diag * self.max_diameter_ratio;

        let mut new_faces = mesh.faces.clone();
        let mut origins: Vec<FaceOrigin> = (0..mesh.faces.len())
            .map(|i| FaceOrigin::Identity(FaceId(i as u32)))
            .collect();
        let mut filled_loops = 0usize;
        let mut added_tris = 0usize;

        for bloop in &report.boundary_loops {
            if !bloop.is_closed
                || bloop.vertices.len() < 3
                || bloop.vertices.len() > self.max_edges
                || bloop.diameter > max_diam
                || bloop.diameter <= 1e-12
            {
                continue;
            }
            if bloop.planarity_error > bloop.diameter * self.max_planarity_ratio {
                continue;
            }
            // Reject loops with repeated vertices (pinched boundary)
            let uniq: BTreeSet<u32> = bloop.vertices.iter().map(|v| v.0).collect();
            if uniq.len() != bloop.vertices.len() {
                continue;
            }

            let tris = triangulate_boundary_loop(mesh, bloop);
            if tris.is_empty() {
                continue;
            }

            filled_loops += 1;
            added_tris += tris.len();
            for tri in tris {
                new_faces.push(tri);
                origins.push(FaceOrigin::Synthesized {
                    boundary_sources: bloop.boundary_faces.clone(),
                });
            }
        }

        Ok(OpOutput {
            mesh: TriangleMesh::from_parts(mesh.vertices.clone(), new_faces),
            origins,
            summary: format!("filled {filled_loops} hole loop(s) with {added_tris} triangle(s)"),
            max_allowed_face_deletion: 0,
            allow_bbox_shrink: false,
        })
    }
}

fn triangulate_boundary_loop(mesh: &TriangleMesh, bloop: &BoundaryLoop) -> Vec<[u32; 3]> {
    let n = bloop.vertices.len();
    if n == 3 {
        return vec![[
            bloop.vertices[0].0,
            bloop.vertices[1].0,
            bloop.vertices[2].0,
        ]];
    }

    let pts: Vec<Point3<f64>> = bloop
        .vertices
        .iter()
        .filter_map(|&v| mesh.vertex(v))
        .collect();
    if pts.len() != n {
        return Vec::new();
    }

    // Newell normal of the loop in its trace direction (which already matches the cap's outward winding).
    let centroid = pts.iter().fold(Vector3::zeros(), |acc, p| acc + p.coords) / (n as f64);
    let mut normal = Vector3::<f64>::zeros();
    for i in 0..n {
        let a = pts[i].coords - centroid;
        let b = pts[(i + 1) % n].coords - centroid;
        normal.x += (a.y - b.y) * (a.z + b.z);
        normal.y += (a.z - b.z) * (a.x + b.x);
        normal.z += (a.x - b.x) * (a.y + b.y);
    }
    let normal_len = normal.norm();
    if normal_len <= 1e-12 {
        return Vec::new();
    }
    let unit_n = normal / normal_len;

    // Orthonormal 2D basis (u_axis, v_axis) aligned so (u x v) == unit_n
    let ref_axis = if unit_n.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let u_axis = unit_n.cross(&ref_axis).normalize();
    let v_axis = unit_n.cross(&u_axis).normalize();

    let project = |p: Point3<f64>| -> Vector2<f64> {
        let d = p.coords - centroid;
        Vector2::new(d.dot(&u_axis), d.dot(&v_axis))
    };

    let mut poly: Vec<(u32, Point3<f64>, Vector2<f64>)> = bloop
        .vertices
        .iter()
        .zip(pts.iter())
        .map(|(&vid, &p)| (vid.0, p, project(p)))
        .collect();

    // Ensure positive 2D signed area for ear clipping; track if reversed so winding stays aligned with loop.
    let area2: f64 = (0..poly.len())
        .map(|i| {
            let a = poly[i].2;
            let b = poly[(i + 1) % poly.len()].2;
            a.x * b.y - a.y * b.x
        })
        .sum();
    let reversed = area2 < 0.0;
    if reversed {
        poly.reverse();
    }

    let mut out = Vec::with_capacity(n - 2);
    let mut guard = 0usize;
    while poly.len() > 3 && guard < n * n {
        guard += 1;
        let m = poly.len();
        let mut clipped = false;
        for i in 0..m {
            let prev = (i + m - 1) % m;
            let next = (i + 1) % m;
            let a = poly[prev].2;
            let b = poly[i].2;
            let c = poly[next].2;
            if cross2(b - a, c - a) <= 1e-12 {
                continue;
            }
            let any_inside = (0..m)
                .any(|k| k != prev && k != i && k != next && point_in_tri_2d(poly[k].2, a, b, c));
            if any_inside {
                continue;
            }
            let tri = if reversed {
                [poly[prev].0, poly[next].0, poly[i].0]
            } else {
                [poly[prev].0, poly[i].0, poly[next].0]
            };
            if TriangleMesh::triangle_area(poly[prev].1, poly[i].1, poly[next].1) > 1e-12 {
                out.push(tri);
            }
            poly.remove(i);
            clipped = true;
            break;
        }
        if !clipped {
            break;
        }
    }

    if poly.len() == 3 {
        let tri = if reversed {
            [poly[0].0, poly[2].0, poly[1].0]
        } else {
            [poly[0].0, poly[1].0, poly[2].0]
        };
        if TriangleMesh::triangle_area(poly[0].1, poly[1].1, poly[2].1) > 1e-12 {
            out.push(tri);
        }
    }

    // Only return a complete cap if ear clipping finished cleanly.
    if poly.len() > 3 {
        return Vec::new();
    }
    out
}

fn cross2(a: Vector2<f64>, b: Vector2<f64>) -> f64 {
    a.x * b.y - a.y * b.x
}

fn point_in_tri_2d(p: Vector2<f64>, a: Vector2<f64>, b: Vector2<f64>, c: Vector2<f64>) -> bool {
    let c1 = cross2(b - a, p - a);
    let c2 = cross2(c - b, p - b);
    let c3 = cross2(a - c, p - c);
    c1 >= -1e-12 && c2 >= -1e-12 && c3 >= -1e-12
}

struct PruneLooseOpenComponentsOp {
    max_faces: Option<usize>,
    max_area_ratio: Option<f64>,
}

impl RepairOp for PruneLooseOpenComponentsOp {
    fn name(&self) -> &'static str {
        "Prune small open debris components"
    }

    fn apply(&self, mesh: &TriangleMesh, report: &DiagnosticReport) -> Result<OpOutput, MeshError> {
        if report.components.len() <= 1 {
            return Ok(OpOutput {
                mesh: mesh.clone(),
                origins: (0..mesh.faces.len())
                    .map(|i| FaceOrigin::Identity(FaceId(i as u32)))
                    .collect(),
                summary: "none".to_string(),
                max_allowed_face_deletion: 0,
                allow_bbox_shrink: false,
            });
        }

        let (largest_id, largest_area) = report
            .components
            .iter()
            .max_by(|a, b| {
                a.face_count
                    .cmp(&b.face_count)
                    .then_with(|| a.surface_area.total_cmp(&b.surface_area))
            })
            .map_or((0, 0.0), |c| (c.id, c.surface_area));

        let drop_ids: BTreeSet<usize> = report
            .components
            .iter()
            .filter(|c| c.id != largest_id && !c.is_closed)
            .filter(|c| {
                should_prune_component(c, largest_area, self.max_faces, self.max_area_ratio)
            })
            .map(|c| c.id)
            .collect();

        if drop_ids.is_empty() {
            return Ok(OpOutput {
                mesh: mesh.clone(),
                origins: (0..mesh.faces.len())
                    .map(|i| FaceOrigin::Identity(FaceId(i as u32)))
                    .collect(),
                summary: "no qualifying loose open fragments".to_string(),
                max_allowed_face_deletion: 0,
                allow_bbox_shrink: false,
            });
        }

        let face_comp = assign_face_components(mesh);
        let mut new_faces = Vec::with_capacity(mesh.faces.len());
        let mut origins = Vec::with_capacity(mesh.faces.len());
        let mut dropped_faces = 0usize;

        for (fi, &tri) in mesh.faces.iter().enumerate() {
            let cid = face_comp.get(fi).copied().unwrap_or(0);
            if drop_ids.contains(&cid) {
                dropped_faces += 1;
            } else {
                new_faces.push(tri);
                origins.push(FaceOrigin::Identity(FaceId(fi as u32)));
            }
        }

        let (verts, faces) = compact_unused_vertices(&mesh.vertices, &new_faces);
        Ok(OpOutput {
            mesh: TriangleMesh::from_parts(verts, faces),
            origins,
            summary: format!(
                "removed {} loose open component(s) ({dropped_faces} face(s))",
                drop_ids.len()
            ),
            max_allowed_face_deletion: dropped_faces,
            allow_bbox_shrink: true,
        })
    }
}

fn should_prune_component(
    c: &ComponentInfo,
    largest_area: f64,
    max_faces: Option<usize>,
    max_area_ratio: Option<f64>,
) -> bool {
    if let Some(mf) = max_faces
        && c.face_count <= mf
    {
        return true;
    }
    if let Some(ratio) = max_area_ratio
        && largest_area > 0.0
        && c.surface_area < largest_area * ratio
    {
        return true;
    }
    false
}

fn assign_face_components(mesh: &TriangleMesh) -> Vec<usize> {
    let n_faces = mesh.faces.len();
    let incidence = mesh.edge_incidence();
    let mut comp_of = vec![usize::MAX; n_faces];
    let mut cid = 0usize;

    for start in 0..n_faces {
        if comp_of[start] != usize::MAX {
            continue;
        }
        comp_of[start] = cid;
        let mut q = VecDeque::new();
        q.push_back(start);
        while let Some(fi) = q.pop_front() {
            let [a, b, c] = mesh.faces[fi];
            for (u, v) in [(a, b), (b, c), (c, a)] {
                if let Some(uses) = incidence.get(&EdgeKey::new(u, v)) {
                    for eu in uses {
                        let nf = eu.face.0 as usize;
                        if comp_of[nf] == usize::MAX {
                            comp_of[nf] = cid;
                            q.push_back(nf);
                        }
                    }
                }
            }
        }
        cid += 1;
    }
    comp_of
}

fn compact_unused_vertices(
    vertices: &[Point3<f64>],
    faces: &[[u32; 3]],
) -> (Vec<Point3<f64>>, Vec<[u32; 3]>) {
    let mut remap = vec![u32::MAX; vertices.len()];
    let mut new_verts = Vec::with_capacity(vertices.len());
    let mut new_faces = Vec::with_capacity(faces.len());

    for &[a, b, c] in faces {
        let mut tri = [0u32; 3];
        for (k, &old) in [a, b, c].iter().enumerate() {
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
    use super::*;
    use mesh_attributes::Confidence;
    use proptest::prelude::*;

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

    fn cube_with_small_hole() -> TriangleMesh {
        // 100x100x100 block with a 2x2 mm square opening in the center of the top face (z=100).
        // Outer corners: 0..8; inner hole corners: 8..12.
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),       // 0
            Point3::new(100.0, 0.0, 0.0),     // 1
            Point3::new(100.0, 100.0, 0.0),   // 2
            Point3::new(0.0, 100.0, 0.0),     // 3
            Point3::new(0.0, 0.0, 100.0),     // 4
            Point3::new(100.0, 0.0, 100.0),   // 5
            Point3::new(100.0, 100.0, 100.0), // 6
            Point3::new(0.0, 100.0, 100.0),   // 7
            Point3::new(49.0, 49.0, 100.0),   // 8
            Point3::new(51.0, 49.0, 100.0),   // 9
            Point3::new(51.0, 51.0, 100.0),   // 10
            Point3::new(49.0, 51.0, 100.0),   // 11
        ];
        let faces = vec![
            // bottom z=0
            [0, 2, 1],
            [0, 3, 2],
            // front y=0
            [0, 1, 5],
            [0, 5, 4],
            // back y=100
            [2, 3, 7],
            [2, 7, 6],
            // left x=0
            [0, 4, 7],
            [0, 7, 3],
            // right x=100
            [1, 2, 6],
            [1, 6, 5],
            // top frame around the 2x2 hole (outward +Z)
            [4, 5, 9],
            [4, 9, 8],
            [5, 6, 10],
            [5, 10, 9],
            [6, 7, 11],
            [6, 11, 10],
            [7, 4, 8],
            [7, 8, 11],
        ];
        TriangleMesh::from_parts(vertices, faces)
    }

    #[test]
    fn clean_cube_is_noop() {
        let cube = unit_cube();
        let attrs = AttributeStore::new_for_mesh(cube.faces.len());
        let res = repair_mesh(&cube, &attrs, RepairProfile::Standard, None).expect("repair");
        assert!(res.report.was_noop);
        assert_eq!(res.mesh, cube);
        assert_eq!(res.attributes.faces.len(), cube.faces.len());
    }

    #[test]
    fn conservative_leaves_small_hole_open_while_standard_closes_it() {
        let holey = cube_with_small_hole();
        let mut attrs = AttributeStore::new_for_mesh(holey.faces.len());
        let mat = attrs.intern_material("pla_blue");
        for f in &mut attrs.faces {
            f.material_id = Some(mat);
        }

        let cons = repair_mesh(&holey, &attrs, RepairProfile::Conservative, None).expect("cons");
        assert!(!cons.report.after.is_watertight);
        assert_eq!(cons.report.after.boundary_edges.len(), 4);

        let std_res = repair_mesh(&holey, &attrs, RepairProfile::Standard, None).expect("std");
        assert!(std_res.report.after.is_watertight);
        assert_eq!(std_res.report.after.face_count, 20);
        assert_eq!(std_res.report.attribute_summary.uncertain_count, 0);
        assert!(
            std_res
                .attributes
                .faces
                .iter()
                .all(|f| f.material_id == Some(mat))
        );
    }

    #[test]
    fn standard_fixes_inverted_winding_bowtie_and_three_face_fin() {
        // Cube with flipped face 0, plus a 3-face tiny fin on edge (0,1), plus a bowtie triangle
        let mut mesh = unit_cube();
        mesh.faces[0].swap(1, 2);
        // Add a small fin sharing edge (0,1)
        let fin_tip = mesh.vertices.len() as u32;
        mesh.vertices.push(Point3::new(0.5, -0.05, 0.0));
        mesh.faces.push([0, 1, fin_tip]);

        let attrs = AttributeStore::new_for_mesh(mesh.faces.len());
        let res = repair_mesh(&mesh, &attrs, RepairProfile::Standard, None).expect("repair");
        assert!(res.report.after.is_watertight);
        assert_eq!(res.report.after.non_manifold_edges.len(), 0);
        assert_eq!(res.report.after.winding_conflict_edges.len(), 0);

        // Bowtie test: two separate triangles touching only at vertex 0
        let bowtie = TriangleMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(-1.0, 0.0, 0.0),
                Point3::new(0.0, -1.0, 0.0),
            ],
            vec![[0, 1, 2], [0, 3, 4]],
        );
        let b_attrs = AttributeStore::new_for_mesh(bowtie.faces.len());
        let b_res = repair_mesh(&bowtie, &b_attrs, RepairProfile::Standard, None).expect("bowtie");
        assert_eq!(b_res.report.after.non_manifold_vertices.len(), 0);
    }

    #[test]
    fn conflicting_hole_boundary_marks_synthesized_faces_uncertain() {
        let holey = cube_with_small_hole();
        let mut attrs = AttributeStore::new_for_mesh(holey.faces.len());
        let m1 = attrs.intern_material("slot1");
        let m2 = attrs.intern_material("slot2");
        for f in &mut attrs.faces {
            f.material_id = Some(m1);
        }
        // Make one face touching the top hole edge belong to slot2 (face 11 touches edge 9-8)
        attrs.faces[11].material_id = Some(m2);

        let res = repair_mesh(&holey, &attrs, RepairProfile::Standard, None).expect("repair");
        assert!(res.report.after.is_watertight);
        assert_eq!(res.report.attribute_summary.uncertain_count, 2);
        assert!(res.attributes.confidence.contains(&Confidence::Uncertain));
    }

    proptest! {
        #[test]
        fn repair_never_increases_non_manifold_edges(
            flip in any::<bool>(),
            dup in any::<bool>(),
            zero_tri in any::<bool>(),
        ) {
            let mut mesh = unit_cube();
            if flip {
                mesh.faces[0].swap(1, 2);
            }
            if dup {
                mesh.faces.push(mesh.faces[1]);
            }
            if zero_tri {
                mesh.faces.push([0, 0, 1]);
            }
            let before = analyze(&mesh);
            let attrs = AttributeStore::new_for_mesh(mesh.faces.len());
            let res = repair_mesh(&mesh, &attrs, RepairProfile::Conservative, None).expect("repair");
            prop_assert!(res.report.after.non_manifold_edges.len() <= before.non_manifold_edges.len());
            prop_assert!(res.report.after.non_finite_vertices.is_empty());
        }
    }
}
