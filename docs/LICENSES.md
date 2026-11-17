# Dependency and Ecosystem License Review

This project is dual-licensed under **MIT OR Apache-2.0**. Core crates avoid C++ FFI and copyleft/noncommercial dependencies so the engine can be freely redistributed and later compiled for server or `wasm32` targets.

## Reference Projects Evaluated (Not Vendored)

| Project | License | Decision & Rationale |
|---|---|---|
| **Krateian/Sutura** | PolyForm Noncommercial 1.0.0 (`v0.2.0+`); Apache-2.0 (`v0.1.x`) | **Not vendored.** Noncommercial terms block public service hosting and commercial redistribution. Adopted only the high-level acceptance concept (adopt-or-reject and reimport validation). |
| **OrcaSlicer** | AGPL-3.0 | **Not vendored / no code copied.** Studied documented Linux CGAL Fix Model and `--repair` CLI behavior only. |
| **CGAL** | GPL-3.0-or-later (polygon mesh processing) | **Not linked.** Incompatible with `MIT OR Apache-2.0`. |
| **PyMeshLab / VCG** | GPL-3.0 | **Not linked.** Behavior reference only. |
| **Rabbit1015/blender-3mf-repair** | GPL-3.0-or-later | **Not copied.** Observed `paint_color` handling notes only. |
| **monomyth/mmu-remapper** | MIT | Reference for Bambu/Orca `paint_color` and Prusa `mmu_segmentation` layout. |
| **elalish/manifold** | Apache-2.0 | Permissive C++ option; deferred (not in Phase 1, as remeshing destroys triangle correspondence). |
| **dimforge/parry3d** | Apache-2.0 | Planned Phase 2 pure-Rust option for self-intersection detection. |
| **lib3mf-core / threemf** | BSD-2-Clause / 0BSD | Evaluated for Milestone 3; both rebuild models from parsed objects and would drop unknown Bambu XML, so Milestone 3 will use a package-preserving ZIP/XML layer. |

## Workspace Crate Dependencies

All direct workspace dependencies in `Cargo.toml` (`nalgebra`, `thiserror`, `serde`, `serde_json`, `rayon`, `clap`, `tracing`, `tracing-subscriber`, `eframe`, `egui`, `wgpu`, `bytemuck`, `rfd`, `tempfile`, `proptest`) use permissive licenses (`MIT`, `Apache-2.0`, `MIT OR Apache-2.0`, or `BSD-3-Clause`).
