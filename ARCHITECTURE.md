# Architecture

## Design Goals

1. **Engine/UI Separation:** All geometry, topology analysis, attribute correspondence, file I/O, and repair logic live in library crates with zero dependency on `egui`, `eframe`, `wgpu`, or `axum`.
2. **Non-Manifold Kernel First:** Broken 3D printing meshes cannot be loaded into a strict half-edge structure without discarding or guessing topology. `mesh-core` uses an indexed `TriangleMesh` (`Vec<Point3<f64>>`, `Vec<[u32; 3]>`) paired with a deterministic edge-incidence map (`BTreeMap<EdgeKey, Vec<EdgeUse>>`) allowing 0, 1, 2, or many incident faces per edge.
3. **Face Correspondence as a Contract:** Every repair step returns `Vec<FaceOrigin>` (`Identity`, `Split`, `Synthesized`, `Spatial`). Sequential steps compose back to original input faces via `compose_origins`. Categorical attributes (OBJ materials/groups and future Bambu/Orca `paint_color`) live in `mesh-attributes::AttributeStore` and are never interpolated as continuous RGB values.
4. **Reimport-Honest Validation:** `mesh-validation::verify_roundtrip` writes repaired output to a temporary file, reloads it through `mesh-formats`, and re-runs `mesh-analysis::analyze` before declaring watertightness or writing the final output.

## Workspace Crate Graph

- `crates/mesh-core`: `TriangleMesh`, `VertexId`, `FaceId`, `EdgeKey`, `RepairProfile`, `BoundingBox`, `ResourceLimits`, `MeshError`.
- `crates/mesh-attributes`: `AttributeStore`, `FaceOrigin`, `Confidence`, `PaintAssignment` (TriangleSelector nibble decoder), `compose_origins`.
- `crates/mesh-analysis`: non-mutating `analyze(&TriangleMesh) -> DiagnosticReport`, including self-intersections, T-junctions, and thin walls.
- `crates/mesh-formats`: STL, OBJ, PLY, AMF, glTF/GLB, and package-preserving 3MF. `.3mf` export goes through `ThreeMfPackage`, not `save_model`.
- `crates/mesh-repair`: `RepairOp` trait, `repair_mesh` pipeline, adopt-or-reject policy, cancellation support.
- `crates/mesh-validation`: `verify_roundtrip` post-repair export/reimport checks.
- `crates/mesh-cli`: `mesh-repair` CLI (`analyze`, `repair`).
- `crates/mesh-desktop`: `mesh-repair-gui` (`eframe` + `wgpu` 3D viewports, headless-tested `AppViewModel`).
- `crates/mesh-web`: localhost Axum service `mesh-repair-web`. Geometry crates do not depend on Axum.

## Planned 3MF Package Preservation Strategy (Milestone 3)

1. Read the `.3mf` ZIP archive without flattening `3D/Objects/object_*.model` or `Metadata/*`.
2. Classify as geometry-only, slicer project (`project_settings.config`, `model_settings.config`), or sliced G-code package (refuse geometry rewrite on sliced packages).
3. Copy all unrelated ZIP entries unchanged; rewrite only modified model XML resources.
4. Preserve `paint_color` / `mmu_segmentation` hex strings verbatim for `FaceOrigin::Identity`, copy parent state for `Split`, inherit for `Synthesized` only when all boundary faces agree, and flag `Confidence::Uncertain` on conflicts.
