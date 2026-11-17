# Roadmap

## Completed
- **Milestone 1 — Research & Workspace Foundation:** Crate layout, non-manifold indexed kernel, license audit, TriangleSelector attribute store, STL/OBJ loaders, non-mutating diagnostics, CI workflow.
- **Milestone 2 — Phase 1 Repairs & Validation:** Conservative, Standard, and Aggressive profiles; vertex welding; degenerate/duplicate removal; winding harmonization; bowtie splitting; unambiguous 3-face fin removal; small planar hole filling; loose open component pruning; adopt-or-reject; reimport verification.
- **Milestone 3 — Color-Safe 3MF Package Editing:** Hardened ZIP/XML package preservation layer, Bambu Studio / OrcaSlicer `3D/Objects/*.model` and `Metadata/*.config` support, byte-for-byte preservation of unmodified entries and unchanged `paint_color` attributes, boundary-agreement attribute transfer with `Uncertain` warnings, and 3MF reimport validation.
- **Milestone 4 (Initial Desktop GUI):** `eframe`/`wgpu` desktop window with drag-and-drop, shared orbit/pan/zoom Before/After viewports, filament/material color rendering, defect overlays, per-object 3MF inspector, background repair with cancellation, and validated export.

## Next Milestones
- **Phase 2 Geometry Repairs:** `parry3d`-backed self-intersection detection, overlapping shell union, T-junction stitching, thin-wall detection, controlled remeshing with spatial transfer warnings, and subdivided `paint_color` split-tree replay.
- **Milestone 5 — Reliability & Additional Formats:** AMF, PLY, GLB/glTF adapters, large-mesh benchmarks, parser fuzzing, and slicer compatibility checks.
- **Milestone 6 — Free Public Web Service:** Axum API in `crates/mesh-web`, resource quotas, isolated worker execution, automatic temp-file cleanup.
