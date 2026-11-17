# Roadmap

## Completed in Current Pass
- **Milestone 1 — Research & Workspace Foundation:** Crate layout, non-manifold indexed kernel, license audit, TriangleSelector attribute store, STL/OBJ loaders, non-mutating diagnostics, CI workflow.
- **Milestone 2 (STL/OBJ) — Phase 1 Repairs & Validation:** Conservative, Standard, and Aggressive profiles; vertex welding; degenerate/duplicate removal; winding harmonization; bowtie splitting; unambiguous 3-face fin removal; small planar hole filling; loose open component pruning; adopt-or-reject; reimport verification.
- **Milestone 4 (Initial Desktop GUI):** `eframe`/`wgpu` desktop window with drag-and-drop, shared orbit/pan/zoom Before/After viewports, defect overlays, background repair with cancellation, and validated export.

## Next Milestones
- **Milestone 3 — Color-Safe 3MF Package Editing:** ZIP/XML package preservation layer, Bambu Studio / OrcaSlicer `paint_color` & `model_settings.config` / `project_settings.config` roundtrip, ZIP bomb and XML hardening, slicer re-open validation.
- **Phase 2 Geometry Repairs:** `parry3d`-backed self-intersection detection, overlapping shell union, T-junction stitching, thin-wall detection, controlled remeshing with spatial transfer warnings.
- **Milestone 5 — Reliability & Additional Formats:** AMF, PLY, GLB/glTF adapters, large-mesh benchmarks, parser fuzzing.
- **Milestone 6 — Free Public Web Service:** Axum API in `crates/mesh-web`, resource quotas, isolated worker execution, automatic temp-file cleanup.
