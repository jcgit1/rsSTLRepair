//! Reimport-honest validation and attribute-preservation verification.

use mesh_analysis::{DiagnosticReport, analyze};
use mesh_attributes::{AttributeStore, Confidence, FaceOrigin};
use mesh_core::{BoundingBox, MeshError, RepairProfile, ResourceLimits, TriangleMesh};
use mesh_formats::{FormatKind, RepairedThreeMfPart, ThreeMfPackage, load_model, save_model};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

/// Result of exporting a mesh to a temporary file, reimporting it, and reanalyzing it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ValidationResult {
    pub passed: bool,
    pub reimported_report: DiagnosticReport,
    pub bbox_drift: Option<f64>,
    pub errors: Vec<String>,
    pub warnings: Vec<String>,
}

/// Parameters for [`verify_roundtrip`].
pub struct RoundtripRequest<'a> {
    pub ext: &'a str,
    pub original_mesh: &'a TriangleMesh,
    pub repaired_mesh: &'a TriangleMesh,
    pub repaired_attrs: &'a AttributeStore,
    pub expected_report: &'a DiagnosticReport,
    pub profile: RepairProfile,
    pub was_noop: bool,
    pub limits: &'a ResourceLimits,
}

/// Export `mesh` + `attributes` to a temporary file of extension `ext` (`stl` or `obj`),
/// reimport it from disk, and verify that topology, bounding box, and watertight claims hold.
pub fn verify_roundtrip(req: RoundtripRequest<'_>) -> Result<ValidationResult, MeshError> {
    let RoundtripRequest {
        ext,
        original_mesh,
        repaired_mesh,
        repaired_attrs,
        expected_report,
        profile,
        was_noop,
        limits,
    } = req;
    let normalized_ext = ext.trim_start_matches('.').to_ascii_lowercase();
    let tmp = tempfile::Builder::new()
        .prefix("rsstlrepair_validate_")
        .suffix(&format!(".{normalized_ext}"))
        .tempfile()?;

    let format = save_model(tmp.path(), repaired_mesh, repaired_attrs)?;
    let reimported = load_model(tmp.path(), limits)?;
    let reimported_report = analyze(&reimported.mesh);

    let mut errors = Vec::new();
    let mut warnings = Vec::new();

    if expected_report.is_watertight && !reimported_report.is_watertight {
        errors.push(
            "mesh analyzed as watertight in memory, but failed watertightness after export and reimport"
                .to_string(),
        );
    }

    if reimported_report.non_finite_vertices.len() > expected_report.non_finite_vertices.len() {
        errors.push("reimported mesh contains new non-finite vertices".to_string());
    }
    if reimported_report.out_of_range_faces.len() > expected_report.out_of_range_faces.len() {
        errors.push("reimported mesh contains out-of-range face indices".to_string());
    }
    if reimported_report.non_manifold_edges.len() > expected_report.non_manifold_edges.len() {
        errors.push("export/reimport introduced new non-manifold edges".to_string());
    }

    let bbox_drift = match (original_mesh.bounding_box(), reimported_report.bounding_box) {
        (Some(b0), Some(b1)) => Some(BoundingBox::max_corner_drift(&b0, &b1)),
        _ => None,
    };

    // Binary STL and glTF/GLB quantize coordinates to f32. ASCII PLY, AMF, and OBJ stay at full precision.
    let drift_tol = match (format, profile) {
        (
            FormatKind::StlBinary | FormatKind::Gltf | FormatKind::Glb,
            RepairProfile::Conservative,
        ) => 1e-4,
        (FormatKind::StlBinary | FormatKind::Gltf | FormatKind::Glb, _) => 5e-4,
        (_, RepairProfile::Conservative) => 1e-9,
        (_, _) => 5e-4,
    };
    if let Some(drift) = bbox_drift
        && profile == RepairProfile::Conservative
        && drift > drift_tol
    {
        errors.push(format!(
            "conservative repair bounding-box drift {drift:.6} exceeds tolerance {drift_tol:.6}"
        ));
    }

    if was_noop && reimported_report.face_count != original_mesh.faces.len() {
        errors.push(format!(
            "clean no-op mesh changed face count on roundtrip ({} -> {})",
            original_mesh.faces.len(),
            reimported_report.face_count
        ));
    }

    let uncertain = repaired_attrs
        .confidence
        .iter()
        .filter(|&&c| c == Confidence::Uncertain)
        .count();
    if uncertain > 0 {
        warnings.push(format!(
            "{uncertain} face(s) carry Uncertain attribute confidence; inspect before printing"
        ));
    }

    Ok(ValidationResult {
        passed: errors.is_empty(),
        reimported_report,
        bbox_drift,
        errors,
        warnings,
    })
}

/// Export a repaired [`ThreeMfPackage`] to a temporary `.3mf`, reopen it, and verify:
/// 1. Unmodified ZIP entries and untouched `.model` files remain byte-for-byte identical.
/// 2. Every part's reimported topology matches expectations and bounding-box drift stays in bounds.
/// 3. Unchanged (`Identity`) triangles keep their exact `paint_color` / material attributes.
pub fn verify_threemf_roundtrip(
    original_pkg: &ThreeMfPackage,
    repaired_parts: &[RepairedThreeMfPart<'_>],
    expected_reports: &[DiagnosticReport],
    profile: RepairProfile,
    limits: &ResourceLimits,
) -> Result<ValidationResult, MeshError> {
    if original_pkg.parts.len() != repaired_parts.len()
        || original_pkg.parts.len() != expected_reports.len()
    {
        return Err(MeshError::ValidationFailed(
            "3MF validation part count mismatch".to_string(),
        ));
    }

    let tmp = tempfile::Builder::new()
        .prefix("rsstlrepair_3mf_validate_")
        .suffix(".3mf")
        .tempfile()?;

    original_pkg.write_repaired_path(tmp.path(), repaired_parts)?;
    let reimported = ThreeMfPackage::load_path(tmp.path(), limits)?;

    let mut errors = Vec::new();
    let mut warnings = Vec::new();

    let modified_files: BTreeSet<&str> = repaired_parts
        .iter()
        .filter(|p| !p.was_noop)
        .map(|p| p.archive_path)
        .collect();

    if reimported.raw_entries.len() != original_pkg.raw_entries.len() {
        errors.push(format!(
            "3MF archive entry count changed ({} -> {})",
            original_pkg.raw_entries.len(),
            reimported.raw_entries.len()
        ));
    } else {
        for (orig_e, new_e) in original_pkg
            .raw_entries
            .iter()
            .zip(reimported.raw_entries.iter())
        {
            if orig_e.name != new_e.name {
                errors.push(format!(
                    "3MF archive entry order/name changed: '{}' -> '{}'",
                    orig_e.name, new_e.name
                ));
                continue;
            }
            if !modified_files.contains(orig_e.name.as_str()) && orig_e.data != new_e.data {
                errors.push(format!(
                    "unmodified 3MF entry '{}' changed bytes during export",
                    orig_e.name
                ));
            }
        }
    }

    if reimported.parts.len() != original_pkg.parts.len() {
        errors.push(format!(
            "3MF part count changed ({} -> {})",
            original_pkg.parts.len(),
            reimported.parts.len()
        ));
    }

    let mut max_drift: Option<f64> = None;
    let mut combined_reimported_faces = 0usize;
    let mut combined_reimported_verts = 0usize;
    let mut last_report = None;

    for (idx, ((orig_part, rep_part), exp_rep)) in original_pkg
        .parts
        .iter()
        .zip(repaired_parts.iter())
        .zip(expected_reports.iter())
        .enumerate()
    {
        let Some(new_part) = reimported.parts.get(idx) else {
            break;
        };
        let new_rep = analyze(&new_part.mesh);
        combined_reimported_faces += new_rep.face_count;
        combined_reimported_verts += new_rep.vertex_count;

        if exp_rep.is_watertight && !new_rep.is_watertight {
            errors.push(format!(
                "3MF object '{}' failed watertightness after reimport",
                orig_part.object_id
            ));
        }
        if new_rep.non_finite_vertices.len() > exp_rep.non_finite_vertices.len() {
            errors.push(format!(
                "3MF object '{}' gained non-finite vertices on reimport",
                orig_part.object_id
            ));
        }
        if new_rep.non_manifold_edges.len() > exp_rep.non_manifold_edges.len() {
            errors.push(format!(
                "3MF object '{}' gained non-manifold edges on reimport",
                orig_part.object_id
            ));
        }

        if let (Some(b0), Some(b1)) = (orig_part.mesh.bounding_box(), new_rep.bounding_box) {
            let d = BoundingBox::max_corner_drift(&b0, &b1);
            max_drift = Some(max_drift.map_or(d, |cur| cur.max(d)));
            if profile == RepairProfile::Conservative && d > 1e-9 {
                errors.push(format!(
                    "3MF object '{}' conservative bbox drift {d:.9} > 1e-9",
                    orig_part.object_id
                ));
            }
        }

        // Verify exact attribute preservation for Identity triangles
        let mut paint_mismatches = 0usize;
        for (fi, origin) in rep_part.attributes.origins.iter().enumerate() {
            if let FaceOrigin::Identity(old_id) = origin
                && let (Some(orig_fa), Some(new_fa)) = (
                    orig_part.attributes.faces.get(old_id.0 as usize),
                    new_part.attributes.faces.get(fi),
                )
            {
                let same = orig_fa.paint == new_fa.paint
                    && orig_fa.paint_supports == new_fa.paint_supports
                    && orig_fa.paint_seam == new_fa.paint_seam
                    && orig_fa.paint_fuzzy_skin == new_fa.paint_fuzzy_skin
                    && orig_fa.threemf_material == new_fa.threemf_material;
                if !same {
                    paint_mismatches += 1;
                }
            }
        }
        if paint_mismatches > 0 {
            errors.push(format!(
                "3MF object '{}' lost exact attributes on {paint_mismatches} unchanged face(s)",
                orig_part.object_id
            ));
        }

        let uncertain = rep_part
            .attributes
            .confidence
            .iter()
            .filter(|&&c| c == Confidence::Uncertain)
            .count();
        if uncertain > 0 {
            warnings.push(format!(
                "Object '{}': {uncertain} synthesized face(s) carry Uncertain color/material confidence",
                orig_part.object_id
            ));
        }

        last_report = Some(new_rep);
    }

    let summary_report = if reimported.parts.len() == 1 {
        last_report.unwrap_or_else(|| analyze(&TriangleMesh::new()))
    } else {
        let mut merged = analyze(&TriangleMesh::new());
        merged.vertex_count = combined_reimported_verts;
        merged.face_count = combined_reimported_faces;
        merged.is_watertight = reimported
            .parts
            .iter()
            .all(|p| analyze(&p.mesh).is_watertight);
        merged
    };

    Ok(ValidationResult {
        passed: errors.is_empty(),
        reimported_report: summary_report,
        bbox_drift: max_drift,
        errors,
        warnings,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_repair::repair_mesh;
    use nalgebra::Point3;

    fn cube_with_small_hole() -> TriangleMesh {
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(100.0, 0.0, 0.0),
            Point3::new(100.0, 100.0, 0.0),
            Point3::new(0.0, 100.0, 0.0),
            Point3::new(0.0, 0.0, 100.0),
            Point3::new(100.0, 0.0, 100.0),
            Point3::new(100.0, 100.0, 100.0),
            Point3::new(0.0, 100.0, 100.0),
            Point3::new(49.0, 49.0, 100.0),
            Point3::new(51.0, 49.0, 100.0),
            Point3::new(51.0, 51.0, 100.0),
            Point3::new(49.0, 51.0, 100.0),
        ];
        let faces = vec![
            [0, 2, 1],
            [0, 3, 2],
            [0, 1, 5],
            [0, 5, 4],
            [2, 3, 7],
            [2, 7, 6],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
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
    fn roundtrip_passes_for_clean_and_hole_repaired_cubes() {
        let holey = cube_with_small_hole();

        let attrs = AttributeStore::new_for_mesh(holey.faces.len());
        let repaired = repair_mesh(&holey, &attrs, RepairProfile::Standard, None).expect("repair");
        assert!(repaired.report.after.is_watertight);

        let val = verify_roundtrip(RoundtripRequest {
            ext: "stl",
            original_mesh: &holey,
            repaired_mesh: &repaired.mesh,
            repaired_attrs: &repaired.attributes,
            expected_report: &repaired.report.after,
            profile: RepairProfile::Standard,
            was_noop: repaired.report.was_noop,
            limits: &ResourceLimits::default(),
        })
        .expect("roundtrip");
        assert!(val.passed, "errors: {:?}", val.errors);
        assert!(val.reimported_report.is_watertight);
    }
}
