# AGENTS.md — Continuation Guide for Coding Agents

## Repository Orientation

- Workspace root: `/home/jcain/repos/rsSTLRepair` (Rust edition 2024, `MIT OR Apache-2.0`).
- **Core invariant:** Never couple geometry/repair crates (`mesh-core`, `mesh-attributes`, `mesh-analysis`, `mesh-repair`, `mesh-formats`, `mesh-validation`) to `egui`, `eframe`, `wgpu`, or `axum`.
- **Kernel:** `mesh_core::TriangleMesh` is an indexed triangle soup (`f64`) with on-demand `BTreeMap<EdgeKey, Vec<EdgeUse>>` incidence. Do not replace the input representation with a strict half-edge structure.
- **Attribute contract:** Any new `RepairOp` must produce a `Vec<FaceOrigin>` mapping every output face back to its input source(s). Never interpolate categorical filament/material IDs as RGB.
- **3MF contract:** `mesh_formats::threemf::ThreeMfPackage` preserves all unrelated ZIP entries and unmodified `.model` files byte-for-byte, and splices only repaired `<object><mesh>` elements. Sliced `.gcode.3mf` packages and lossy `.3mf -> .stl/.obj` exports are refused.

## Verification Commands

Always run before concluding changes:

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
```

## Next Slice (Phase 2 Geometry & Reliability)

1. Integrate `parry3d` (`Apache-2.0`) in `mesh-analysis` / `mesh-repair` for BVH-accelerated self-intersection detection and reporting.
2. Add split-tree replay (`PaintAssignment::is_subdivided_tree`) for triangles subdivided during mesh repair.
3. Expand format adapters (`PLY`, `AMF`, `GLB/glTF`) and fuzz targets for parsers.
