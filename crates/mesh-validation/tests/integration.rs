use mesh_analysis::analyze;
use mesh_core::{RepairProfile, ResourceLimits};
use mesh_formats::load_model;
use mesh_repair::repair_mesh;
use mesh_validation::{RoundtripRequest, verify_roundtrip};
use std::path::PathBuf;

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("crates")
        .parent()
        .expect("repo root")
        .to_path_buf()
}

#[test]
fn valid_fixture_cube_is_watertight_and_noop() {
    let path = repo_root().join("tests/fixtures/valid/cube.stl");
    let limits = ResourceLimits::default();
    let loaded = load_model(&path, &limits).expect("load cube fixture");
    let diag = analyze(&loaded.mesh);
    assert!(diag.is_watertight);
    assert_eq!(diag.face_count, 12);

    let repaired = repair_mesh(
        &loaded.mesh,
        &loaded.attributes,
        RepairProfile::Conservative,
        None,
    )
    .expect("repair");
    assert!(repaired.report.was_noop);
}

#[test]
fn corrupted_fixture_hole_stays_open_in_conservative_and_closes_in_standard() {
    let path = repo_root().join("tests/fixtures/corrupted/broken_hole.stl");
    let limits = ResourceLimits::default();
    let loaded = load_model(&path, &limits).expect("load broken fixture");
    let diag = analyze(&loaded.mesh);
    assert!(!diag.is_watertight);
    assert_eq!(diag.boundary_edges.len(), 4);

    let cons = repair_mesh(
        &loaded.mesh,
        &loaded.attributes,
        RepairProfile::Conservative,
        None,
    )
    .expect("conservative");
    assert!(!cons.report.after.is_watertight);

    let std_res = repair_mesh(
        &loaded.mesh,
        &loaded.attributes,
        RepairProfile::Standard,
        None,
    )
    .expect("standard");
    assert!(std_res.report.after.is_watertight);
    assert_eq!(std_res.report.after.face_count, 20);

    let val = verify_roundtrip(RoundtripRequest {
        ext: "stl",
        original_mesh: &loaded.mesh,
        repaired_mesh: &std_res.mesh,
        repaired_attrs: &std_res.attributes,
        expected_report: &std_res.report.after,
        profile: RepairProfile::Standard,
        was_noop: std_res.report.was_noop,
        limits: &limits,
    })
    .expect("roundtrip");
    assert!(val.passed, "errors: {:?}", val.errors);
    assert!(val.reimported_report.is_watertight);
}
