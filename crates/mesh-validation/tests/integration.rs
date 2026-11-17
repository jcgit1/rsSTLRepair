use mesh_analysis::analyze;
use mesh_core::{RepairProfile, ResourceLimits};
use mesh_formats::{RepairedThreeMfPart, ThreeMfKind, ThreeMfPackage, load_model};
use mesh_repair::repair_mesh;
use mesh_validation::{RoundtripRequest, verify_roundtrip, verify_threemf_roundtrip};
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
    .expect("std");
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

#[test]
fn bambu_orca_two_object_3mf_repairs_broken_part_and_preserves_paint_and_metadata() {
    let path = repo_root().join("tests/fixtures/bambu-orca/painted_two_object.3mf");
    let limits = ResourceLimits::default();
    let pkg = ThreeMfPackage::load_path(&path, &limits).expect("load 3mf fixture");
    assert_eq!(pkg.kind, ThreeMfKind::SlicerProject);
    assert_eq!(pkg.parts.len(), 2);
    assert_eq!(pkg.filament_palette.len(), 4);

    let res0 = repair_mesh(
        &pkg.parts[0].mesh,
        &pkg.parts[0].attributes,
        RepairProfile::Standard,
        None,
    )
    .expect("repair part 0");
    let res1 = repair_mesh(
        &pkg.parts[1].mesh,
        &pkg.parts[1].attributes,
        RepairProfile::Standard,
        None,
    )
    .expect("repair part 1");

    assert!(!res0.report.was_noop);
    assert!(res0.report.after.is_watertight);
    assert!(res1.report.was_noop);

    let refs = [
        RepairedThreeMfPart {
            archive_path: &pkg.parts[0].archive_path,
            object_id: &pkg.parts[0].object_id,
            was_noop: res0.report.was_noop,
            mesh: &res0.mesh,
            attributes: &res0.attributes,
        },
        RepairedThreeMfPart {
            archive_path: &pkg.parts[1].archive_path,
            object_id: &pkg.parts[1].object_id,
            was_noop: res1.report.was_noop,
            mesh: &res1.mesh,
            attributes: &res1.attributes,
        },
    ];
    let expected = [res0.report.after.clone(), res1.report.after.clone()];

    let val = verify_threemf_roundtrip(&pkg, &refs, &expected, RepairProfile::Standard, &limits)
        .expect("3mf roundtrip");
    assert!(val.passed, "errors: {:?}", val.errors);
    assert!(val.reimported_report.is_watertight);
    assert_eq!(val.reimported_report.face_count, 32);
}

#[test]
fn multicolor_basematerials_3mf_is_noop_and_preserves_colors() {
    let path = repo_root().join("tests/fixtures/multicolor/basematerials_cube.3mf");
    let limits = ResourceLimits::default();
    let pkg = ThreeMfPackage::load_path(&path, &limits).expect("load basematerials fixture");
    assert_eq!(pkg.kind, ThreeMfKind::GeometryModel);
    assert_eq!(pkg.parts.len(), 1);
    assert!(pkg.parts[0].attributes.display_color_for_face(0).is_some());
    assert!(pkg.parts[0].attributes.display_color_for_face(2).is_some());
    assert_ne!(
        pkg.parts[0].attributes.display_color_for_face(0),
        pkg.parts[0].attributes.display_color_for_face(2)
    );

    let res = repair_mesh(
        &pkg.parts[0].mesh,
        &pkg.parts[0].attributes,
        RepairProfile::Conservative,
        None,
    )
    .expect("repair");
    assert!(res.report.was_noop);
}
