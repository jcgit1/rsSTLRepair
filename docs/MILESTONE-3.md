# Milestone 3 Report — Color-Safe 3MF Package Editing

## Summary

Implemented package-preserving `.3mf` import, per-object geometry repair, surgical XML mesh splicing, attribute preservation, reimport validation, and multicolor visualization across `mesh-attributes`, `mesh-formats`, `mesh-validation`, `mesh-cli`, and `mesh-desktop`.

## What Works Today

- **Hardened 3MF Package Reader (`mesh-formats::threemf`):**
  - Enforces max file bytes, max archive entries (`4096`), strict path-traversal rejection, bounded decompression (`max_decompressed_bytes` enforced on actual stream reads), XML depth cap (`128`), and `DOCTYPE` rejection.
  - Classifies packages into `GeometryModel` vs `SlicerProject` and explicitly refuses sliced `.gcode.3mf` packages.
  - Follows `_rels/.rels` and Production-extension `<component p:path="...">` links (such as Bambu Studio / OrcaSlicer `3D/Objects/object_*.model`), parses `Metadata/project_settings.config` filament colors, `Metadata/model_settings.config` default extruders, and standard 3MF `<basematerials>` / `<m:colorgroup>`.
- **Surgical 3MF Writer:**
  - Copies all unrelated ZIP entries (`Metadata/*`, thumbnails, `_rels/.rels`, `[Content_Types].xml`) and untouched `.model` XML files byte-for-byte.
  - Rewrites only repaired `<object><mesh>` elements while retaining `<build>`, `<components>`, transforms, and surrounding XML verbatim.
  - Preserves `paint_color`, `slic3rpe:mmu_segmentation`, `paint_supports`, `paint_seam`, `paint_fuzzy_skin`, and `pid`/`p1`/`p2`/`p3` verbatim on unchanged triangles; inherits uniform attributes on synthesized hole caps when boundary neighbors agree; flags conflicts as `Confidence::Uncertain`.
- **Validation & Interfaces:**
  - `verify_threemf_roundtrip` reopens exported `.3mf` archives, verifies unmodified entry byte-parity, reanalyzes topology, and checks unchanged triangle attribute equality.
  - CLI (`mesh-repair analyze` / `repair`) and Desktop GUI (`mesh-repair-gui`) support `.3mf` drag-and-drop, filament swatches, multicolor rendering, per-object inspection, and `.3mf` export (while refusing lossy `.3mf -> .stl/.obj` export).

## Known Limits / Deferred to Phase 2

- Triangles carrying subdivided brush split trees (`is_subdivided_tree == true`) keep their raw attribute verbatim when unchanged (`Identity`), and are marked `Uncertain` if neighboring a newly synthesized hole cap.
- Self-intersections, T-junctions, controlled remeshing, and additional formats (`PLY`, `AMF`, `glTF`) remain deferred to Phase 2 / Milestone 5.
