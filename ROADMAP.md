# Roadmap

## Completed
- **Milestone 1 — Research & Workspace Foundation:** Crate layout, non-manifold indexed kernel, license audit, TriangleSelector attribute store, STL/OBJ loaders, non-mutating diagnostics, CI workflow.
- **Milestone 2 — Phase 1 Repairs & Validation:** Conservative, Standard, and Aggressive profiles; vertex welding; degenerate/duplicate removal; winding harmonization; bowtie splitting; unambiguous 3-face fin removal; small planar hole filling; loose open component pruning; adopt-or-reject; reimport verification.
- **Milestone 3 — Color-Safe 3MF Package Editing:** Hardened ZIP/XML package preservation layer, Bambu Studio / OrcaSlicer `3D/Objects/*.model` and `Metadata/*.config` support, byte-for-byte preservation of unmodified entries and unchanged `paint_color` attributes, boundary-agreement attribute transfer with `Uncertain` warnings, and 3MF reimport validation.
- **Milestone 4 (Initial Desktop GUI):** `eframe`/`wgpu` desktop window with drag-and-drop, shared orbit/pan/zoom Before/After viewports, filament/material color rendering, defect overlays, per-object 3MF inspector, background repair with cancellation, and validated export.
- **Phase 2 Geometry Repairs (0.3.0):** `parry3d` self-intersection detection, T-junction stitching, crossing splits, closed-shell union, thin-wall reporting, and uniform `paint_color` split-tree replay. Thin walls are reported and left in place.
- **Controlled remesh:** The Aggressive profile splits long edges and collapses short edges inside a defect patch. Faces that no longer lie on one input triangle use spatial transfer and are marked uncertain.
- **Milestone 5 — Additional Formats (0.3.0):** ASCII/binary PLY, millimeter AMF, and one-mesh glTF/GLB, plus parser fuzz on random bytes. Large-mesh benchmarks and extra slicer compatibility checks are still open.
- **SVG and tessellated STEP:** Simple SVG contours import as a 1 mm extrusion. STEP import reads triangular `POLY_LOOP` faces and refuses analytic solids. Both are import-only.
- **User guide:** [docs/USER-GUIDE.md](docs/USER-GUIDE.md) states what each profile fixes, what is reported and left alone, and which files are refused.
- **Milestone 6 — Local Repair Service (0.3.0):** Axum API in `crates/mesh-web` (`mesh-repair-web` on `127.0.0.1:8787`), upload and concurrency quotas, blocking repair, and temp-file cleanup. This is a localhost service, not a deployed public host.

## Still Open
- Large-mesh benchmarks and broader slicer compatibility checks.
