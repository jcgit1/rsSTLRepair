# Milestone 1 + Phase 1 Repairs & Desktop GUI Report

## Summary

Established the `rsSTLRepair` Cargo workspace (`MIT OR Apache-2.0`), implemented the non-manifold indexed `TriangleMesh` kernel, face-correspondence `AttributeStore`, non-mutating topology diagnostics, STL/OBJ loaders and writers, Conservative/Standard/Aggressive Phase 1 repair pipelines, reimport-honest validation, the `mesh-repair` CLI, and the `mesh-repair-gui` Linux desktop application.

## What Works Today

- **Diagnostics (`mesh-analysis`):** Accurately detects non-finite coordinates, out-of-range indices, duplicate vertices and faces, zero-area triangles, boundary loops (with diameter and planarity metrics), 3+ face non-manifold edges, bowtie vertices, winding conflicts, and inverted closed shells. Distinguishes valid open surfaces from corrupt topology.
- **Formats (`mesh-formats`):** Binary and ASCII STL load, binary STL export, OBJ load/export with `g`/`o` groups and `usemtl` categorical material IDs, pre-allocation triangle caps, and explicit refusal of `.3mf` files.
- **Repairs (`mesh-repair`):**
  - Clean watertight models execute as exact no-ops.
  - **Conservative:** Removes invalid/non-finite faces, welds bitwise-identical vertices, removes degenerate and duplicate faces, harmonizes local winding (flipping minority parity), and flips inverted closed shells outward. Leaves open holes and disconnected components untouched.
  - **Standard:** Adds `1e-4` mm near-duplicate welding, bowtie vertex splitting, unambiguous 3-face fin removal, small planar hole capping (`<= 16` edges, `<= 5%` bbox diagonal), and loose open debris pruning (`<= 3` faces).
  - **Aggressive:** Expands hole capping (`<= 64` edges, `<= 20%` diagonal) and prunes open components under `1%` of the largest component's area.
  - Every step runs behind adopt-or-reject checks and returns a composed `FaceOrigin` map.
- **Validation (`mesh-validation`):** Exports repaired models to a temporary file, reimports, and reanalyzes before reporting watertightness or writing final outputs.
- **Desktop GUI (`mesh-desktop`):** Verified on CachyOS Wayland with `eframe`/`wgpu`. Provides drag-and-drop and file picker loading, explicit `.3mf` refusal banner, synchronized dual Before/After 3D viewports, amber/red/magenta defect overlays, profile selector, dimensions lock readout, off-thread repair with cancel, and overwrite-protected export.

## What Is Deferred (Not Claimed Complete)

This list is the Milestone 1 snapshot. 3MF package editing shipped in 0.2.0. Phase 2 geometry, PLY/AMF/glTF, and the localhost repair service shipped in 0.3.0. Controlled remeshing, STEP, and SVG are still open.

- Color-safe `.3mf` package editing and Bambu Studio / OrcaSlicer `paint_color` roundtrip (Milestone 3).
- Self-intersection detection/resolution, T-junction stitching, thin-wall analysis, and controlled remeshing (Phase 2).
- AMF, PLY, GLB/glTF, STEP, and SVG adapters (Milestone 5).
- Public Axum web service in `crates/mesh-web` (Milestone 6).

## Proposed Next Steps

1. Implement Milestone 3 (`mesh-formats::threemf`): ZIP entry-preserving parser/writer, Bambu/Orca `3D/Objects/object_*.model` and `Metadata/*.config` handling, full `paint_color` attribute preservation/transfer validation, and slicer re-open fixture tests.
2. Add Phase 2 `parry3d` self-intersection diagnostics to `mesh-analysis`.
