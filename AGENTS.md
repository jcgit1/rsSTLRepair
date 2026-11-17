# AGENTS.md — Continuation Guide for Coding Agents

## Repository Orientation

- Workspace root: `/home/jcain/repos/rsSTLRepair` (Rust edition 2024, `MIT OR Apache-2.0`).
- **Core invariant:** Never couple geometry/repair crates (`mesh-core`, `mesh-attributes`, `mesh-analysis`, `mesh-repair`, `mesh-formats`, `mesh-validation`) to `egui`, `eframe`, `wgpu`, or `axum`.
- **Kernel:** `mesh_core::TriangleMesh` is an indexed triangle soup (`f64`) with on-demand `BTreeMap<EdgeKey, Vec<EdgeUse>>` incidence. Do not replace the input representation with a strict half-edge structure.
- **Attribute contract:** Any new `RepairOp` must produce a `Vec<FaceOrigin>` mapping every output face back to its input source(s). Never interpolate categorical filament/material IDs as RGB.
- **3MF guard:** Until Milestone 3 is implemented with package-preserving ZIP/XML rewriting, `.3mf` inputs and exports must remain explicitly refused rather than silently flattened.

## Verification Commands

Always run before concluding changes:

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
```

## Next Slice (Milestone 3)

Implement `mesh_formats::threemf` using a ZIP entry-preserving layer (`zip` + `quick-xml`) so untouched package entries (`Metadata/project_settings.config`, `Metadata/model_settings.config`, thumbnails, unrelated objects) are copied verbatim while only repaired `3D/**/*.model` meshes and `paint_color` attributes are updated.
