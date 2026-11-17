//! Reimport-honest validation and attribute-preservation verification.

use mesh_analysis::{DiagnosticReport, analyze};
use mesh_attributes::{AttributeStore, Confidence};
use mesh_core::{BoundingBox, MeshError, RepairProfile, ResourceLimits, TriangleMesh};
use mesh_formats::{FormatKind, load_model, save_model};
use serde::{Deserialize, Serialize};

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

    // Binary STL quantizes coordinates to f32 (~1e-6 * magnitude), while OBJ writes f64.
    let drift_tol = match (format, profile) {
        (FormatKind::StlBinary, RepairProfile::Conservative) => 1e-4,
        (FormatKind::StlBinary, _) => 5e-4,
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
