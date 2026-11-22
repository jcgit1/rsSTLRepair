# rsSTLRepair

Open-source, Linux-native 3D model diagnostics and repair utility in Rust, featuring package-preserving multicolor `.3mf` repairs for Bambu Studio and OrcaSlicer alongside STL, OBJ, PLY, AMF, glTF, simple SVG, and tessellated STEP.

What a repair will and will not change is written for publication in [docs/USER-GUIDE.md](docs/USER-GUIDE.md).

## Current Capabilities (0.3.0)

- **Formats supported:**
  - **3MF (`.3mf`):** Package-preserving ZIP/XML reader and surgical writer supporting standard 3MF (`<basematerials>`, `<m:colorgroup>`, `pid`/`p1`) and Bambu Studio / OrcaSlicer / PrusaSlicer projects (`3D/Objects/*.model`, `Metadata/project_settings.config`, `Metadata/model_settings.config`, `paint_color`, `slic3rpe:mmu_segmentation`, `paint_supports`, `paint_seam`, `paint_fuzzy_skin`). Sliced `.gcode.3mf` packages are explicitly refused rather than invalidated.
  - **STL (`.stl`):** Binary and ASCII import; binary export.
  - **OBJ (`.obj`):** Import and export with `g`/`o` groups and `usemtl` categorical material IDs.
  - **PLY (`.ply`):** ASCII export. ASCII and binary little-endian import for `x,y,z` vertices and triangle faces, with optional `material_id`.
  - **AMF (`.amf`):** Millimeter import and export. `DOCTYPE` is rejected. Volume `materialid` is a categorical face id.
  - **glTF / GLB (`.gltf`, `.glb`):** One mesh, one primitive, `POSITION` plus `UNSIGNED_INT` indices. `.gltf` embeds its buffer. Face material ids live in `primitive.extras.material_ids`.
  - **SVG (`.svg`):** Import only. Simple closed shapes are extruded to a 1 mm solid. User units are millimeters. Curves, transforms, and holes are refused.
  - **STEP (`.step`, `.stp`):** Import only. Triangular `POLY_LOOP` faces are read. Analytic CAD solids are refused.
- **Non-mutating diagnostics:** Non-finite coordinates, out-of-range face indices, exact duplicate vertices, duplicate and zero-area triangles, boundary edges and loops, non-manifold edges (3+ faces), non-manifold (bowtie) vertices, winding conflicts, inverted closed shells, connected components, self-intersections, T-junctions, and thin walls.
- **Three repair profiles (Conservative default):**
  - **Conservative:** Invalid/non-finite cleanup, bitwise-identical vertex welding, duplicate/zero-area face removal, local winding harmonization, and outward shell orientation. Never fills holes, stitches, splits crossings, or removes components.
  - **Standard:** Adds `1e-4` mm near-duplicate welding, bowtie vertex splitting, T-junction stitching, crossing splits, unambiguous 3-face fin removal, small planar hole filling (`<= 16` edges, `<= 5%` bounding-box diagonal), and tiny open debris pruning (`<= 3` faces).
  - **Aggressive:** Adds union of overlapping closed shells, widens hole filling (`<= 64` edges, `<= 20%` diagonal), prunes open components under `1%` of the largest component's area, and locally remeshes skinny defect patches. A face that no longer lies on one input triangle is transferred by spatial proximity and marked uncertain.
- **Color & package preservation:** Each 3MF object is repaired independently; clean objects and unrelated ZIP entries (`Metadata/*`, thumbnails, relationships) are preserved byte-for-byte. Unchanged triangles retain original `paint_color` / material attributes verbatim. A geometric split replays a uniform paint tree as a new leaf and clears a mixed tree as `Uncertain`. Hole caps inherit only when boundary neighbors unanimously agree.
- **Reimport-honest validation:** Exported files are written to a temporary file, reopened, reanalyzed, and checked for attribute preservation before saving. Original files are never overwritten. glTF/GLB use the same drift tolerance as binary STL because their buffers are `f32`.
- **Interfaces:**
  - CLI (`mesh-repair`): `analyze` and `repair` subcommands with `--json` output.
  - Linux Desktop GUI (`mesh-repair-gui`): `eframe`/`wgpu` dual synchronized Before/After 3D viewports, filament/material shading, defect overlays, per-object 3MF inspector, drag-and-drop, background repair with cancellation, and validated export.
  - Local web service (`mesh-repair-web`): `GET /health` and `POST /api/repair` on `127.0.0.1:8787`. Send the file bytes with `X-Filename`. 3MF, SVG, and STEP stay on the CLI and GUI.

## Build and Run

```bash
cargo test --workspace
cargo run -p mesh-cli -- analyze tests/fixtures/bambu-orca/painted_two_object.3mf
cargo run -p mesh-cli -- repair tests/fixtures/bambu-orca/painted_two_object.3mf --profile standard -o /tmp/repaired.3mf
cargo run -p mesh-desktop -- tests/fixtures/bambu-orca/painted_two_object.3mf
cargo run -p mesh-web --bin mesh-repair-web
```

## License

Dual-licensed under [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at your option.
