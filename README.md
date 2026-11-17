# rsSTLRepair

Open-source, Linux-native 3D model diagnostics and repair utility in Rust, featuring package-preserving multicolor `.3mf` repairs for Bambu Studio and OrcaSlicer alongside STL and OBJ support.

## Current Capabilities (Milestones 1–3 + Desktop GUI)

- **Formats supported:**
  - **3MF (`.3mf`):** Package-preserving ZIP/XML reader and surgical writer supporting standard 3MF (`<basematerials>`, `<m:colorgroup>`, `pid`/`p1`) and Bambu Studio / OrcaSlicer / PrusaSlicer projects (`3D/Objects/*.model`, `Metadata/project_settings.config`, `Metadata/model_settings.config`, `paint_color`, `slic3rpe:mmu_segmentation`, `paint_supports`, `paint_seam`, `paint_fuzzy_skin`). Sliced `.gcode.3mf` packages are explicitly refused rather than invalidated.
  - **STL (`.stl`):** Binary and ASCII import; binary export.
  - **OBJ (`.obj`):** Import and export with `g`/`o` groups and `usemtl` categorical material IDs.
- **Non-mutating diagnostics:** Non-finite coordinates, out-of-range face indices, exact duplicate vertices, duplicate and zero-area triangles, boundary edges and loops, non-manifold edges (3+ faces), non-manifold (bowtie) vertices, winding conflicts, inverted closed shells, and connected components.
- **Three repair profiles (Conservative default):**
  - **Conservative:** Invalid/non-finite cleanup, bitwise-identical vertex welding, duplicate/zero-area face removal, local winding harmonization, and outward shell orientation. Never fills holes or removes components.
  - **Standard:** Adds `1e-4` mm near-duplicate welding, bowtie vertex splitting, unambiguous 3-face fin removal, small planar hole filling (`<= 16` edges, `<= 5%` bounding-box diagonal), and tiny open debris pruning (`<= 3` faces).
  - **Aggressive:** Widens hole filling (`<= 64` edges, `<= 20%` diagonal) and prunes open components under `1%` of the largest component's area.
- **Color & package preservation:** Each 3MF object is repaired independently; clean objects and unrelated ZIP entries (`Metadata/*`, thumbnails, relationships) are preserved byte-for-byte. Unchanged triangles retain original `paint_color` / material attributes verbatim, hole caps inherit only when boundary neighbors unanimously agree, and conflicts are flagged as `Uncertain`.
- **Reimport-honest validation:** Exported files (`.3mf`, `.stl`, `.obj`) are written to a temporary file, reopened, reanalyzed, and checked for attribute preservation before saving. Original files are never overwritten.
- **Interfaces:**
  - CLI (`mesh-repair`): `analyze` and `repair` subcommands with `--json` output.
  - Linux Desktop GUI (`mesh-repair-gui`): `eframe`/`wgpu` dual synchronized Before/After 3D viewports, filament/material shading, defect overlays, per-object 3MF inspector, drag-and-drop, background repair with cancellation, and validated export.

## Build and Run

```bash
cargo test --workspace
cargo run -p mesh-cli -- analyze tests/fixtures/bambu-orca/painted_two_object.3mf
cargo run -p mesh-cli -- repair tests/fixtures/bambu-orca/painted_two_object.3mf --profile standard -o /tmp/repaired.3mf
cargo run -p mesh-desktop -- tests/fixtures/bambu-orca/painted_two_object.3mf
```

## License

Dual-licensed under [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at your option.
