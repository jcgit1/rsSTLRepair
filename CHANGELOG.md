# Changelog

## [0.1.0] - 2026-11-17

### Added
- Permissive (`MIT OR Apache-2.0`) 9-crate Rust workspace.
- Non-manifold indexed `TriangleMesh` kernel (`mesh-core`) and `AttributeStore` with `FaceOrigin` correspondence and TriangleSelector leaf decoding (`mesh-attributes`).
- Non-mutating diagnostics (`mesh-analysis`) distinguishing valid open surfaces from defects.
- STL (ASCII/binary) and OBJ (groups/`usemtl`) loaders and writers with resource caps and `.3mf` refusal guard (`mesh-formats`).
- Conservative, Standard, and Aggressive Phase 1 repair pipeline with adopt-or-reject policy (`mesh-repair`) and reimport validation (`mesh-validation`).
- `mesh-repair` CLI (`analyze`, `repair`) and `mesh-repair-gui` Linux `eframe`/`wgpu` desktop application (`mesh-desktop`).
