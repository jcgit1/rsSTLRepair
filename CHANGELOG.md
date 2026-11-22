# Changelog

## [Unreleased]

### Added
- Aggressive controlled remesh of skinny defect patches. Long edges split and short edges collapse on the input surface. Faces that no longer lie on one input triangle use spatial attribute transfer and are marked uncertain.
- Import-only SVG extrusion of simple closed shapes, and import-only STEP reading of triangular `POLY_LOOP` faces. Curves, transforms, holes, and analytic STEP solids are refused.
- Publish-facing user guide (`docs/USER-GUIDE.md`) describing what each profile fixes and what is left unchanged.

## [0.3.0] - 2026-11-20

### Added
- Phase 2 geometry: `parry3d` broadphase self-intersection, T-junction, and thin-wall scans; Standard stitching and crossing splits; Aggressive union of overlapping closed shells. Thin walls are reported and not deleted.
- Uniform TriangleSelector paint trees replay as a fresh leaf after a geometric split. Mixed or truncated trees are cleared and marked `Uncertain`.
- PLY (ASCII write, ASCII and binary little-endian read), millimeter AMF, and one-mesh glTF/GLB loaders and writers. Categorical `material_id` is preserved. Parser fuzz rejects panics on random bytes.
- Local Axum service `mesh-repair-web` (`GET /health`, `POST /api/repair`) with a 64 MiB upload cap, two concurrent repairs, blocking execution, and reimport validation before the response.

## [0.2.0] - 2026-11-17

### Added
- Package-preserving 3MF reader and surgical XML writer (`mesh-formats::threemf`) supporting standard 3MF materials and Bambu Studio / OrcaSlicer `3D/Objects/*.model`, `Metadata/*.config`, `paint_color`, `mmu_segmentation`, `paint_supports`, `paint_seam`, and `paint_fuzzy_skin`.
- Hardened ZIP/XML checks for 3MF: path traversal rejection, decompressed payload byte cap (ZIP bomb protection), entry-count cap, XML nesting depth limit, `DOCTYPE` rejection, and refusal of sliced `.gcode.3mf` packages.
- `verify_threemf_roundtrip` in `mesh-validation` verifying untouched ZIP entry byte-parity, reimported topology, and unchanged triangle attribute preservation.
- CLI and desktop GUI support for `.3mf` packages, including filament color swatches, multicolor rendering, per-object selection, and refusal of lossy `.3mf` -> `.stl`/`.obj` export.

## [0.1.0] - 2026-11-17

### Added
- Permissive (`MIT OR Apache-2.0`) 9-crate Rust workspace.
- Non-manifold indexed `TriangleMesh` kernel (`mesh-core`) and `AttributeStore` with `FaceOrigin` correspondence and TriangleSelector leaf decoding (`mesh-attributes`).
- Non-mutating diagnostics (`mesh-analysis`) distinguishing valid open surfaces from defects.
- STL (ASCII/binary) and OBJ (groups/`usemtl`) loaders and writers with resource caps (`mesh-formats`).
- Conservative, Standard, and Aggressive Phase 1 repair pipeline with adopt-or-reject policy (`mesh-repair`) and reimport validation (`mesh-validation`).
- `mesh-repair` CLI (`analyze`, `repair`) and `mesh-repair-gui` Linux `eframe`/`wgpu` desktop application (`mesh-desktop`).
