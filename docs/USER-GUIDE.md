# User Guide

rsSTLRepair checks a triangle mesh and repairs the defects it can fix without guessing. It is a repair tool for 3D printing, not a CAD modeler. The default profile is **Conservative**, which is the right choice for a painted 3MF project.

A repair is kept only when the mesh is not worse afterward. The original file is never overwritten. Export writes a new path, reopens that file, and checks it before the save counts.

## How to run it

```bash
mesh-repair analyze model.3mf
mesh-repair repair model.3mf --profile standard -o repaired.3mf
mesh-repair-gui model.3mf
mesh-repair-web
```

The desktop window shows the file before and after repair, defect edges, and filament or material colors. The local service listens on `127.0.0.1:8787` only. It is not a public host.

`POST /api/repair` takes the file bytes and an `X-Filename` header. Optional query `profile` is `conservative`, `standard`, or `aggressive`. The service accepts `.stl`, `.obj`, `.ply`, `.amf`, `.gltf`, and `.glb`, up to 64 MiB, with two repairs at a time. `.3mf`, `.svg`, `.step`, and `.stp` stay on the CLI and the desktop app so a package or an import-only format is not flattened into the response.

## What you can open

| File | What is kept | What this version does not do |
|---|---|---|
| `.3mf` | Object hierarchy, transforms, `Metadata/*`, thumbnails, relationships, and unchanged model XML. `paint_color`, `mmu_segmentation`, `paint_supports`, `paint_seam`, `paint_fuzzy_skin`, and `pid`/`p1` stay on triangles that were not rebuilt. | Sliced `.gcode.3mf` packages are refused. Export to `.stl` or `.obj` is refused, because that would drop paint and project data. |
| `.stl` | Triangle geometry. Binary and ASCII both open. Export is binary STL. | STL has no color or object names to preserve. |
| `.obj` | `g` / `o` groups and `usemtl` material names, as categories. | `.mtl` files, textures, and UV coordinates are not loaded. The report says so when an `mtllib` line is present. |
| `.ply` | `x,y,z` vertices, triangle faces, and an optional integer `material_id`. ASCII and binary little-endian both open. Export is ASCII. | Other vertex properties, such as per-vertex color, are rejected. |
| `.amf` | Millimeter meshes. A volume `materialid` is kept as a category. | Any unit other than millimeter is refused. A `DOCTYPE` is refused. |
| `.gltf`, `.glb` | One mesh, one primitive, `POSITION` plus triangle indices. Face material ids in `primitive.extras.material_ids`. A `.gltf` buffer must be embedded. | External buffers, scenes with several meshes, UVs, and texture images are not loaded. Coordinates are stored as 32-bit floats, so a roundtrip can move a vertex by a fraction of a micron. |
| `.svg` | Simple closed shapes, extruded into a solid 1 mm thick. The numbers in the shape are millimeters. Supported shapes are `rect`, `polygon`, a closed `polyline`, `circle`, `ellipse` (32 sides), and a `path` of `M`, `L`, `H`, `V`, and `Z` only. | `viewBox` and document size are not applied. Curves, `transform` attributes, holes, and more than one contour in a single path are refused. SVG is import-only. Export a mesh format. |
| `.step`, `.stp` | Tessellated triangles stored as `POLY_LOOP` faces of three `CARTESIAN_POINT`s. | Analytic solids (cylinders, spheres, NURBS, and any face that is not already a triangle) are refused. STEP is import-only. Export a mesh format. |

CLI and desktop loads also refuse a file over 256 MiB, or a mesh over 10 million vertices or triangles.

## What each profile fixes

Choose the narrowest profile that clears the defects you care about.

**Conservative**

- Drops coordinates that are not finite, and faces that index missing vertices.
- Welds vertices that are bitwise identical.
- Removes zero-area triangles and exact duplicate faces.
- Makes neighboring triangles agree on winding, and turns a closed shell outward.

It does not fill holes, stitch cracks, split crossings, delete pieces, or remesh. A clean watertight mesh is returned unchanged.

**Standard**, in addition:

- Welds vertices closer than 0.0001 mm.
- Splits a bowtie vertex, where two fans touch only at a point.
- Stitches a T-junction by splitting the edge a vertex already lies on.
- Splits triangles that cross each other.
- Removes an unambiguous fin (exactly three faces on one edge).
- Fills a small planar hole: a closed loop of at most 16 edges, no wider than 5% of the model's diagonal, and flat within 5% of that loop's own diameter.
- Deletes loose open scraps of 3 faces or fewer.

**Aggressive**, in addition:

- Unions two closed shells that overlap, when that union comes back as one watertight solid.
- Fills a larger planar hole: at most 64 edges, no wider than 20% of the diagonal, flat within 15% of the loop diameter.
- Deletes open components whose area is under 1% of the largest component.
- Remeshes a local patch of skinny or still-defective triangles. Edges much longer than the typical edge are split. Edges much shorter are collapsed. The patch stops at 2048 seed faces. A uniform mesh is not remeshed.

Aggressive prints a warning that detail and paint fidelity can change.

## What is reported and left alone

These show up in the analysis and in the repair notes. They are not a failed repair. The tool is refusing to invent a shape.

- **Thin walls.** Faces that face each other across a gap smaller than a print-scale tolerance stay in the file. Deleting them would remove intentional shells, such as a hollow part.
- **Openings that are large, bent, or pinched.** A hole past the profile's edge count, diameter, or flatness limit stays open. A loop that repeats a vertex is not capped. An intentional window is not closed by Conservative, and Standard only closes the small planar ones.
- **Non-manifold edges that are not a simple fin.** An edge with three or more faces is removed only in the unambiguous three-face fin case. Anything more tangled is preserved and counted in the notes.
- **Crossings that splitting or the shell union did not clear.** Remaining self-intersections are counted. Open surfaces are not boolean-cut into each other.
- **Non-orientable regions.** Winding that still conflicts after the local fix is counted and left as-is.
- **A clean watertight model.** If the only finding is thin walls, or there is no defect, the file is not rewritten.

A step that would add non-manifold edges, add self-intersections, or move the bounding box past the profile tolerance is rolled back. The report names the rejected step.

## Paint, materials, and 3MF projects

Colors and filament slots are categories. They are copied or left unknown. They are never blended into a new RGB color.

- A triangle that was not rebuilt keeps its original paint string byte for byte.
- A triangle split from one parent keeps that parent's paint when the paint is a single color. A mixed or truncated paint tree is cleared and marked **uncertain**.
- A new hole cap takes the neighbors' material or paint only when every neighbor agrees. A conflict is marked uncertain.
- After an Aggressive remesh, a face that no longer lies on a single input triangle takes the nearest input face's category and is marked uncertain. The report counts those faces.

On a 3MF project, each object is repaired on its own. An object that was already clean is copied through unchanged. Unrelated ZIP entries stay byte for byte.

## How to read the result

- **No-op.** The input was already acceptable for that profile. Nothing was written differently.
- **Warnings.** A step was rejected, paint was uncertain, or Aggressive remesh used a spatial match. Inspect those faces before printing a painted part.
- **Unresolved notes.** Counts of holes, non-manifold edges, crossings, T-junctions, and thin walls that are still present. That list is the contract for this file.
- **Reimport check.** Export fails closed when the written file does not load back to the same topology within the format's tolerance.

If a painted region is marked uncertain, open the repaired 3MF in the slicer and look at that region before sending it to the printer. The geometry may be fine while the filament assignment is not.
