# Testing

## Running Checks

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
```

## Coverage Summary

- **`mesh-core`:** canonical `EdgeKey`, unit cube signed volume, surface area, bounding box.
- **`mesh-attributes`:** TriangleSelector nibble decoding (`4`, `8`, `0C`, `1C`, `2C`, split-tree detection), sRGB filament palette resolution, boundary inheritance vs conflict -> `Uncertain`.
- **`mesh-analysis`:** clean watertight cube, valid open surface distinction, 3-face non-manifold edge, bowtie vertex, NaN vertex, zero-area and duplicate faces.
- **`mesh-formats`:** binary/ASCII STL cube parity, truncated binary STL rejection, excessive header triangle count rejection before allocation, OBJ `g`/`usemtl` preservation, Bambu/Orca Production-extension 3MF parsing and surgical XML splicing, ZIP path-traversal / decompression-bomb / XML `DOCTYPE` rejection, and sliced `.gcode.3mf` refusal.
- **`mesh-repair`:** clean mesh no-op, Conservative vs Standard small-hole behavior, inverted winding + 3-face fin + bowtie repair, conflicting boundary attributes marked `Uncertain`, `proptest` non-regression on non-manifold edges and non-finite coordinates.
- **`mesh-validation`:** export-to-temp, reimport, and reanalysis roundtrip verification for STL (`cube.stl`, `broken_hole.stl`) and 3MF (`painted_two_object.3mf`, `basematerials_cube.3mf`) including byte-for-byte verification of untouched archive entries and `Identity` triangle paint attributes.
- **`mesh-cli`:** CLI repair integration and refusal to overwrite the input path.
- **`mesh-desktop`:** headless `AppViewModel` tests (Conservative default, painted 3MF load/repair/export, lossy 3MF->STL export refusal, overwrite guard) plus Wayland/wgpu smoke launch.
