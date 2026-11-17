# Changelog

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
