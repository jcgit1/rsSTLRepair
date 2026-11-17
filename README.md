# rsSTLRepair

Open-source, Linux-native 3D model diagnostics and repair utility in Rust, designed with strict face-correspondence tracking for future color-preserving 3MF repairs.

## Current Capabilities (Milestone 1 + Phase 1 Repairs & Desktop GUI)

- **Formats supported:** Binary and ASCII STL, Wavefront OBJ (including `g`/`o` groups and `usemtl` material assignments). `.3mf` files are explicitly refused in this release so painting and slicer metadata cannot be silently stripped.
- **Non-mutating diagnostics:** Non-finite coordinates, out-of-range face indices, exact duplicate vertices, duplicate and zero-area triangles, boundary edges and loops, non-manifold edges (3+ faces), non-manifold (bowtie) vertices, winding conflicts, inverted closed shells, and connected components. Valid open surfaces are distinguished from broken geometry.
- **Three repair profiles (Conservative default):**
  - **Conservative:** Invalid/non-finite cleanup, bitwise-identical vertex welding, duplicate/zero-area face removal, local winding harmonization, and outward shell orientation. Never fills holes or removes components.
  - **Standard:** Adds `1e-4` mm near-duplicate welding, bowtie vertex splitting, unambiguous 3-face fin removal, small planar hole filling (`<= 16` edges, `<= 5%` bounding-box diagonal), and tiny open debris pruning (`<= 3` faces).
  - **Aggressive:** Widens hole filling (`<= 64` edges, `<= 20%` diagonal) and prunes open components under `1%` of the largest component's area.
- **Adopt-or-reject & reimport verification:** Each step is reanalyzed before adoption; exported files are written to a temporary file, reimported, and reanalyzed before declaring success. Original files are never overwritten.
- **Interfaces:**
  - CLI (`mesh-repair`): `analyze` and `repair` subcommands with `--json` output.
  - Linux Desktop GUI (`mesh-repair-gui`): `eframe`/`wgpu` dual synchronized Before/After 3D viewports, defect overlays, drag-and-drop, background repair with cancellation, and validated export.

## Build and Run

```bash
cargo test --workspace
cargo run -p mesh-cli -- analyze tests/fixtures/corrupted/broken_hole.stl
cargo run -p mesh-cli -- repair tests/fixtures/corrupted/broken_hole.stl --profile standard -o /tmp/repaired.stl
cargo run -p mesh-desktop -- tests/fixtures/corrupted/broken_hole.stl
```

## License

Dual-licensed under [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at your option.
