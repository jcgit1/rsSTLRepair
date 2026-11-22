//! Safe mesh importers and exporters (STL, OBJ, and package-preserving 3MF).

pub mod amf;
pub mod gltf;
pub mod ply;
pub mod step;
pub mod svg;
pub mod threemf;

use mesh_attributes::AttributeStore;
use mesh_core::{FaceId, MeshError, ResourceLimits, TriangleMesh};
use nalgebra::Point3;
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Read, Write};
use std::path::Path;
pub use threemf::{
    RepairedThreeMfPart, ThreeMfKind, ThreeMfMeshPart, ThreeMfPackage, transform_point,
};

/// Recognized mesh file formats.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FormatKind {
    StlBinary,
    StlAscii,
    Obj,
    ThreeMf,
    Ply,
    Amf,
    Gltf,
    Glb,
    Svg,
    Step,
}

/// Loaded mesh paired with any categorical attributes, optional 3MF package, and import warnings.
#[derive(Debug, Clone, PartialEq)]
pub struct LoadedModel {
    pub mesh: TriangleMesh,
    pub attributes: AttributeStore,
    pub source_format: FormatKind,
    pub warnings: Vec<String>,
    pub threemf: Option<ThreeMfPackage>,
}

/// Load a model from disk according to its extension, enforcing [`ResourceLimits`].
pub fn load_model(path: &Path, limits: &ResourceLimits) -> Result<LoadedModel, MeshError> {
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .map(str::to_ascii_lowercase)
        .unwrap_or_default();

    match ext.as_str() {
        "stl" => load_stl_path(path, limits),
        "obj" => load_obj_path(path, limits),
        "3mf" => load_threemf_path(path, limits),
        "ply" => load_ply_path(path, limits),
        "amf" => load_amf_path(path, limits),
        "gltf" => load_gltf_path(path, limits, FormatKind::Gltf),
        "glb" => load_gltf_path(path, limits, FormatKind::Glb),
        "svg" => load_svg_path(path, limits),
        "step" | "stp" => load_step_path(path, limits),
        other => Err(MeshError::UnsupportedFormat(format!(
            "unsupported file extension '.{other}'; supported formats are .stl, .obj, .3mf, .ply, .amf, .gltf, .glb, .svg, .step, and .stp"
        ))),
    }
}

/// Load a `.3mf` package via [`ThreeMfPackage::load_path`].
pub fn load_threemf_path(path: &Path, limits: &ResourceLimits) -> Result<LoadedModel, MeshError> {
    let pkg = ThreeMfPackage::load_path(path, limits)?;
    let (mesh, attributes) = combine_threemf_preview(&pkg);
    let warnings = pkg.warnings.clone();
    Ok(LoadedModel {
        mesh,
        attributes,
        source_format: FormatKind::ThreeMf,
        warnings,
        threemf: Some(pkg),
    })
}

/// Build a combined preview [`TriangleMesh`] and [`AttributeStore`] from a [`ThreeMfPackage`].
///
/// For single-object packages with identity transforms, this returns the exact object mesh
/// and attributes. For multi-object packages, display transforms are applied to the preview
/// copy only; actual repair and export operate per-part on [`ThreeMfPackage::parts`].
#[must_use]
pub fn combine_threemf_preview(pkg: &ThreeMfPackage) -> (TriangleMesh, AttributeStore) {
    if pkg.parts.len() == 1 {
        let p = &pkg.parts[0];
        let is_ident = p.display_transform
            == [
                1.0, 0.0, 0.0, 0.0, //
                0.0, 1.0, 0.0, 0.0, //
                0.0, 0.0, 1.0, 0.0, //
                0.0, 0.0, 0.0, 1.0,
            ];
        if is_ident {
            return (p.mesh.clone(), p.attributes.clone());
        }
    }

    let mut verts = Vec::new();
    let mut faces = Vec::new();
    let mut attrs = AttributeStore {
        filament_palette: pkg.filament_palette.clone(),
        ..Default::default()
    };

    for part in &pkg.parts {
        let base_v = verts.len() as u32;
        for &v in &part.mesh.vertices {
            verts.push(transform_point(&part.display_transform, v));
        }
        for (fi, &[a, b, c]) in part.mesh.faces.iter().enumerate() {
            faces.push([base_v + a, base_v + b, base_v + c]);
            let mut fa = part.attributes.faces.get(fi).cloned().unwrap_or_default();
            if fa.paint.is_none()
                && fa.threemf_material.is_none()
                && let Some(ext) = part.attributes.default_extruder
            {
                fa.paint = Some(mesh_attributes::PaintAssignment {
                    attr_name: "paint_color".to_string(),
                    raw_attribute: String::new(),
                    decoded_leaf_state: Some(ext),
                    is_subdivided_tree: false,
                });
            }
            attrs.faces.push(fa);
            attrs
                .origins
                .push(mesh_attributes::FaceOrigin::Identity(FaceId(
                    (faces.len() - 1) as u32,
                )));
            attrs.confidence.push(
                part.attributes
                    .confidence
                    .get(fi)
                    .copied()
                    .unwrap_or(mesh_attributes::Confidence::Exact),
            );
        }
        for (k, v) in &part.attributes.threemf_colors {
            attrs.threemf_colors.entry(k.clone()).or_insert(*v);
        }
    }

    (TriangleMesh::from_parts(verts, faces), attrs)
}

/// Save a mesh and its categorical attributes to a new path according to extension.
///
/// For `.3mf` packages, use [`ThreeMfPackage::write_repaired_path`] so unrelated entries and
/// object hierarchy are preserved instead of flattened.
pub fn save_model(
    path: &Path,
    mesh: &TriangleMesh,
    attributes: &AttributeStore,
) -> Result<FormatKind, MeshError> {
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .map(str::to_ascii_lowercase)
        .unwrap_or_default();

    match ext.as_str() {
        "stl" => {
            write_stl_binary_path(path, mesh)?;
            Ok(FormatKind::StlBinary)
        }
        "obj" => {
            write_obj_path(path, mesh, attributes)?;
            Ok(FormatKind::Obj)
        }
        "ply" => {
            write_ply_path(path, mesh, attributes)?;
            Ok(FormatKind::Ply)
        }
        "amf" => {
            write_amf_path(path, mesh, attributes)?;
            Ok(FormatKind::Amf)
        }
        "gltf" => {
            write_gltf_path(path, mesh, attributes)?;
            Ok(FormatKind::Gltf)
        }
        "glb" => {
            write_glb_path(path, mesh, attributes)?;
            Ok(FormatKind::Glb)
        }
        "svg" => Err(MeshError::UnsupportedFormat(
            "SVG is import-only. Export the repaired mesh as .stl, .obj, .ply, .amf, .gltf, .glb, or .3mf."
                .to_string(),
        )),
        "step" | "stp" => Err(MeshError::UnsupportedFormat(
            "STEP is import-only. Export the repaired mesh as .stl, .obj, .ply, .amf, .gltf, .glb, or .3mf."
                .to_string(),
        )),
        "3mf" => Err(MeshError::UnsupportedFormat(
            "3MF export must use ThreeMfPackage::write_repaired_path to preserve package entries and painting."
                .to_string(),
        )),
        other => Err(MeshError::UnsupportedFormat(format!(
            "unsupported export extension '.{other}'; supported formats are .stl, .obj, .ply, .amf, .gltf, and .glb"
        ))),
    }
}

fn read_limited(path: &Path, limits: &ResourceLimits) -> Result<Vec<u8>, MeshError> {
    let meta = std::fs::metadata(path)?;
    if meta.len() > limits.max_file_bytes {
        return Err(MeshError::FileTooLarge {
            size: meta.len(),
            limit: limits.max_file_bytes,
        });
    }
    let mut file = File::open(path)?;
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes)?;
    if bytes.len() as u64 > limits.max_file_bytes {
        return Err(MeshError::FileTooLarge {
            size: bytes.len() as u64,
            limit: limits.max_file_bytes,
        });
    }
    Ok(bytes)
}

fn loaded_simple(
    mesh: TriangleMesh,
    attributes: AttributeStore,
    source_format: FormatKind,
) -> LoadedModel {
    LoadedModel {
        mesh,
        attributes,
        source_format,
        warnings: Vec::new(),
        threemf: None,
    }
}

/// Load a PLY file from `path`.
pub fn load_ply_path(path: &Path, limits: &ResourceLimits) -> Result<LoadedModel, MeshError> {
    let bytes = read_limited(path, limits)?;
    let (mesh, attributes) = ply::load_ply_slice(&bytes, limits)?;
    Ok(loaded_simple(mesh, attributes, FormatKind::Ply))
}

/// Load an AMF file from `path`.
pub fn load_amf_path(path: &Path, limits: &ResourceLimits) -> Result<LoadedModel, MeshError> {
    let bytes = read_limited(path, limits)?;
    let (mesh, attributes) = amf::load_amf_slice(&bytes, limits)?;
    Ok(loaded_simple(mesh, attributes, FormatKind::Amf))
}

/// Load an SVG file from `path` and extrude each simple contour.
pub fn load_svg_path(path: &Path, limits: &ResourceLimits) -> Result<LoadedModel, MeshError> {
    let bytes = read_limited(path, limits)?;
    let (mesh, attributes) = svg::load_svg_slice(&bytes, limits)?;
    Ok(loaded_simple(mesh, attributes, FormatKind::Svg))
}

/// Load tessellated triangular faces from a STEP file at `path`.
pub fn load_step_path(path: &Path, limits: &ResourceLimits) -> Result<LoadedModel, MeshError> {
    let bytes = read_limited(path, limits)?;
    let (mesh, attributes) = step::load_step_slice(&bytes, limits)?;
    Ok(loaded_simple(mesh, attributes, FormatKind::Step))
}

/// Load a `.gltf` or `.glb` file from `path`.
pub fn load_gltf_path(
    path: &Path,
    limits: &ResourceLimits,
    kind: FormatKind,
) -> Result<LoadedModel, MeshError> {
    let bytes = read_limited(path, limits)?;
    let (mesh, attributes) = gltf::load_gltf_slice(&bytes, limits)?;
    Ok(loaded_simple(mesh, attributes, kind))
}

fn write_ply_path(
    path: &Path,
    mesh: &TriangleMesh,
    attributes: &AttributeStore,
) -> Result<(), MeshError> {
    let file = File::create(path)?;
    ply::write_ply(BufWriter::new(file), mesh, attributes)
}

fn write_amf_path(
    path: &Path,
    mesh: &TriangleMesh,
    attributes: &AttributeStore,
) -> Result<(), MeshError> {
    let file = File::create(path)?;
    amf::write_amf(BufWriter::new(file), mesh, attributes)
}

fn write_gltf_path(
    path: &Path,
    mesh: &TriangleMesh,
    attributes: &AttributeStore,
) -> Result<(), MeshError> {
    let file = File::create(path)?;
    gltf::write_gltf(BufWriter::new(file), mesh, attributes)
}

fn write_glb_path(
    path: &Path,
    mesh: &TriangleMesh,
    attributes: &AttributeStore,
) -> Result<(), MeshError> {
    let file = File::create(path)?;
    gltf::write_glb(BufWriter::new(file), mesh, attributes)
}

/// Load an STL file (auto-detecting binary vs ASCII) from `path`.
pub fn load_stl_path(path: &Path, limits: &ResourceLimits) -> Result<LoadedModel, MeshError> {
    let meta = std::fs::metadata(path)?;
    if meta.len() > limits.max_file_bytes {
        return Err(MeshError::FileTooLarge {
            size: meta.len(),
            limit: limits.max_file_bytes,
        });
    }
    let file = File::open(path)?;
    load_stl_bytes_or_stream(BufReader::new(file), meta.len(), limits)
}

/// Load an STL from an in-memory byte slice.
pub fn load_stl_slice(bytes: &[u8], limits: &ResourceLimits) -> Result<LoadedModel, MeshError> {
    let len = bytes.len() as u64;
    if len > limits.max_file_bytes {
        return Err(MeshError::FileTooLarge {
            size: len,
            limit: limits.max_file_bytes,
        });
    }
    load_stl_bytes_or_stream(std::io::Cursor::new(bytes), len, limits)
}

fn load_stl_bytes_or_stream<R: BufRead + Read>(
    mut reader: R,
    total_len: u64,
    limits: &ResourceLimits,
) -> Result<LoadedModel, MeshError> {
    let mut header = [0u8; 84];
    let n = reader.read(&mut header)?;
    if n == 0 {
        return Err(MeshError::InvalidFormat("empty STL file".to_string()));
    }

    let is_exact_binary = if n == 84 {
        let tri_count = u32::from_le_bytes([header[80], header[81], header[82], header[83]]) as u64;
        84_u64.saturating_add(tri_count.saturating_mul(50)) == total_len
    } else {
        false
    };

    let starts_with_solid = std::str::from_utf8(&header[..n.min(64)])
        .map(|s| s.trim_start().to_ascii_lowercase().starts_with("solid"))
        .unwrap_or(false);

    if is_exact_binary || (!starts_with_solid && n >= 84) {
        let tri_count =
            u32::from_le_bytes([header[80], header[81], header[82], header[83]]) as usize;
        if tri_count > limits.max_triangles {
            return Err(MeshError::TooManyTriangles {
                count: tri_count,
                limit: limits.max_triangles,
            });
        }
        let expected = 84_u64.saturating_add((tri_count as u64).saturating_mul(50));
        if total_len < expected {
            return Err(MeshError::InvalidFormat(format!(
                "truncated binary STL: expected {expected} bytes for {tri_count} triangles, got {total_len}"
            )));
        }
        parse_binary_stl(reader, tri_count, limits)
    } else if starts_with_solid {
        let mut buf = header[..n].to_vec();
        reader.take(limits.max_file_bytes).read_to_end(&mut buf)?;
        parse_ascii_stl(&buf, limits)
    } else {
        Err(MeshError::InvalidFormat(
            "file is neither valid binary nor ASCII STL".to_string(),
        ))
    }
}

fn parse_binary_stl<R: Read>(
    mut reader: R,
    tri_count: usize,
    limits: &ResourceLimits,
) -> Result<LoadedModel, MeshError> {
    let mut builder = IndexedStlBuilder::new(tri_count);
    let mut rec = [0u8; 50];

    for idx in 0..tri_count {
        reader.read_exact(&mut rec).map_err(|e| {
            if e.kind() == std::io::ErrorKind::UnexpectedEof {
                MeshError::InvalidFormat(format!(
                    "truncated binary STL at triangle {idx} of {tri_count}"
                ))
            } else {
                MeshError::Io(e)
            }
        })?;

        let v0 = read_f32_point(&rec[12..24]);
        let v1 = read_f32_point(&rec[24..36]);
        let v2 = read_f32_point(&rec[36..48]);
        builder.push_triangle(v0, v1, v2, limits)?;
    }

    Ok(LoadedModel {
        attributes: AttributeStore::new_for_mesh(builder.faces.len()),
        mesh: TriangleMesh::from_parts(builder.vertices, builder.faces),
        source_format: FormatKind::StlBinary,
        warnings: Vec::new(),
        threemf: None,
    })
}

fn parse_ascii_stl(bytes: &[u8], limits: &ResourceLimits) -> Result<LoadedModel, MeshError> {
    let text = std::str::from_utf8(bytes)
        .map_err(|_| MeshError::InvalidFormat("ASCII STL contains invalid UTF-8".to_string()))?;

    let mut builder = IndexedStlBuilder::new(256);
    let mut tri_verts: Vec<Point3<f64>> = Vec::with_capacity(3);
    let mut saw_endsolid = false;

    for raw_line in text.lines() {
        let line = raw_line.trim();
        if line.is_empty() {
            continue;
        }
        let lower = line.to_ascii_lowercase();
        if lower.starts_with("endsolid") {
            saw_endsolid = true;
            continue;
        }
        if lower.starts_with("vertex ") || lower.starts_with("vertex\t") {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() != 4 {
                return Err(MeshError::InvalidFormat(format!(
                    "malformed ASCII STL vertex line: '{line}'"
                )));
            }
            let x: f64 = parts[1].parse().map_err(|_| {
                MeshError::InvalidFormat(format!("invalid float in STL vertex: '{line}'"))
            })?;
            let y: f64 = parts[2].parse().map_err(|_| {
                MeshError::InvalidFormat(format!("invalid float in STL vertex: '{line}'"))
            })?;
            let z: f64 = parts[3].parse().map_err(|_| {
                MeshError::InvalidFormat(format!("invalid float in STL vertex: '{line}'"))
            })?;
            tri_verts.push(Point3::new(x, y, z));
            if tri_verts.len() == 3 {
                builder.push_triangle(tri_verts[0], tri_verts[1], tri_verts[2], limits)?;
                tri_verts.clear();
            }
        }
    }

    if !tri_verts.is_empty() {
        return Err(MeshError::InvalidFormat(
            "incomplete triangle in ASCII STL".to_string(),
        ));
    }
    if !saw_endsolid && builder.faces.is_empty() {
        return Err(MeshError::InvalidFormat(
            "ASCII STL contains no triangles or endsolid marker".to_string(),
        ));
    }

    Ok(LoadedModel {
        attributes: AttributeStore::new_for_mesh(builder.faces.len()),
        mesh: TriangleMesh::from_parts(builder.vertices, builder.faces),
        source_format: FormatKind::StlAscii,
        warnings: Vec::new(),
        threemf: None,
    })
}

struct IndexedStlBuilder {
    vertices: Vec<Point3<f64>>,
    faces: Vec<[u32; 3]>,
    dedup: HashMap<(u64, u64, u64), u32>,
}

impl IndexedStlBuilder {
    fn new(tri_hint: usize) -> Self {
        Self {
            vertices: Vec::with_capacity(tri_hint / 2),
            faces: Vec::with_capacity(tri_hint),
            dedup: HashMap::with_capacity(tri_hint / 2),
        }
    }

    fn intern_vertex(&mut self, p: Point3<f64>, limits: &ResourceLimits) -> Result<u32, MeshError> {
        if p.x.is_finite() && p.y.is_finite() && p.z.is_finite() {
            let key = (
                canonical_bits(p.x),
                canonical_bits(p.y),
                canonical_bits(p.z),
            );
            if let Some(&idx) = self.dedup.get(&key) {
                return Ok(idx);
            }
            if self.vertices.len() >= limits.max_vertices {
                return Err(MeshError::TooManyVertices {
                    count: self.vertices.len() + 1,
                    limit: limits.max_vertices,
                });
            }
            let idx = self.vertices.len() as u32;
            self.vertices.push(Point3::new(
                if p.x == 0.0 { 0.0 } else { p.x },
                if p.y == 0.0 { 0.0 } else { p.y },
                if p.z == 0.0 { 0.0 } else { p.z },
            ));
            self.dedup.insert(key, idx);
            Ok(idx)
        } else {
            // Keep non-finite vertices distinct so diagnostics surface every occurrence.
            if self.vertices.len() >= limits.max_vertices {
                return Err(MeshError::TooManyVertices {
                    count: self.vertices.len() + 1,
                    limit: limits.max_vertices,
                });
            }
            let idx = self.vertices.len() as u32;
            self.vertices.push(p);
            Ok(idx)
        }
    }

    fn push_triangle(
        &mut self,
        a: Point3<f64>,
        b: Point3<f64>,
        c: Point3<f64>,
        limits: &ResourceLimits,
    ) -> Result<(), MeshError> {
        if self.faces.len() >= limits.max_triangles {
            return Err(MeshError::TooManyTriangles {
                count: self.faces.len() + 1,
                limit: limits.max_triangles,
            });
        }
        let ia = self.intern_vertex(a, limits)?;
        let ib = self.intern_vertex(b, limits)?;
        let ic = self.intern_vertex(c, limits)?;
        self.faces.push([ia, ib, ic]);
        Ok(())
    }
}

fn canonical_bits(v: f64) -> u64 {
    if v == 0.0 {
        0.0_f64.to_bits()
    } else {
        v.to_bits()
    }
}

fn read_f32_point(bytes: &[u8]) -> Point3<f64> {
    let x = f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]) as f64;
    let y = f32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]) as f64;
    let z = f32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]) as f64;
    Point3::new(x, y, z)
}

/// Write a [`TriangleMesh`] as a binary STL file.
pub fn write_stl_binary_path(path: &Path, mesh: &TriangleMesh) -> Result<(), MeshError> {
    let file = File::create(path)?;
    write_stl_binary(BufWriter::new(file), mesh)
}

/// Write a [`TriangleMesh`] as binary STL into `writer`.
pub fn write_stl_binary<W: Write>(mut writer: W, mesh: &TriangleMesh) -> Result<(), MeshError> {
    let mut header = [0u8; 80];
    let tag = b"rsSTLRepair binary STL";
    header[..tag.len()].copy_from_slice(tag);
    writer.write_all(&header)?;

    let count = u32::try_from(mesh.faces.len()).map_err(|_| {
        MeshError::InvalidFormat("mesh face count exceeds binary STL u32 limit".to_string())
    })?;
    writer.write_all(&count.to_le_bytes())?;

    for fi in 0..mesh.faces.len() {
        let [a, b, c] = mesh.face_positions(FaceId(fi as u32)).ok_or_else(|| {
            MeshError::InvalidFormat(format!("face {fi} references out-of-range vertex"))
        })?;
        let ab = b - a;
        let ac = c - a;
        let n = ab.cross(&ac);
        let len = n.norm();
        let normal = if len.is_finite() && len > 0.0 {
            n / len
        } else {
            nalgebra::Vector3::zeros()
        };

        write_f32_vec(&mut writer, normal.x, normal.y, normal.z)?;
        write_f32_vec(&mut writer, a.x, a.y, a.z)?;
        write_f32_vec(&mut writer, b.x, b.y, b.z)?;
        write_f32_vec(&mut writer, c.x, c.y, c.z)?;
        writer.write_all(&0u16.to_le_bytes())?;
    }
    writer.flush()?;
    Ok(())
}

fn write_f32_vec<W: Write>(writer: &mut W, x: f64, y: f64, z: f64) -> Result<(), MeshError> {
    writer.write_all(&(x as f32).to_le_bytes())?;
    writer.write_all(&(y as f32).to_le_bytes())?;
    writer.write_all(&(z as f32).to_le_bytes())?;
    Ok(())
}

/// Load a Wavefront OBJ file, capturing `g`/`o` groups and `usemtl` materials as categorical face attributes.
pub fn load_obj_path(path: &Path, limits: &ResourceLimits) -> Result<LoadedModel, MeshError> {
    let meta = std::fs::metadata(path)?;
    if meta.len() > limits.max_file_bytes {
        return Err(MeshError::FileTooLarge {
            size: meta.len(),
            limit: limits.max_file_bytes,
        });
    }
    let file = File::open(path)?;
    load_obj_reader(BufReader::new(file), limits)
}

/// Load a Wavefront OBJ from a buffered reader.
pub fn load_obj_reader<R: BufRead>(
    reader: R,
    limits: &ResourceLimits,
) -> Result<LoadedModel, MeshError> {
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    let mut attrs = AttributeStore::default();
    let mut current_group: Option<u32> = None;
    let mut current_material: Option<u32> = None;
    let mut face_attrs = Vec::new();
    let mut warnings = Vec::new();
    let mut saw_mtllib = false;

    for raw_line in reader.lines() {
        let line = raw_line?;
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }
        let mut parts = trimmed.split_whitespace();
        let Some(cmd) = parts.next() else {
            continue;
        };
        match cmd {
            "v" => {
                if vertices.len() >= limits.max_vertices {
                    return Err(MeshError::TooManyVertices {
                        count: vertices.len() + 1,
                        limit: limits.max_vertices,
                    });
                }
                let coords: Vec<&str> = parts.collect();
                if coords.len() < 3 {
                    return Err(MeshError::InvalidFormat(format!(
                        "malformed OBJ vertex line: '{trimmed}'"
                    )));
                }
                let x = parse_f64(coords[0], trimmed)?;
                let y = parse_f64(coords[1], trimmed)?;
                let z = parse_f64(coords[2], trimmed)?;
                vertices.push(Point3::new(x, y, z));
            }
            "g" | "o" => {
                let name = parts.collect::<Vec<_>>().join(" ");
                current_group = if name.is_empty() {
                    None
                } else {
                    Some(attrs.intern_group(&name))
                };
            }
            "usemtl" => {
                let name = parts.collect::<Vec<_>>().join(" ");
                current_material = if name.is_empty() {
                    None
                } else {
                    Some(attrs.intern_material(&name))
                };
            }
            "mtllib" => {
                if !saw_mtllib {
                    saw_mtllib = true;
                    warnings.push(
                        "OBJ mtllib/textures are not loaded in this release; categorical usemtl names are preserved."
                            .to_string(),
                    );
                }
            }
            "f" => {
                let mut indices = Vec::new();
                for token in parts {
                    let v_str = token.split('/').next().unwrap_or("");
                    let raw_idx: isize = v_str.parse().map_err(|_| {
                        MeshError::InvalidFormat(format!("invalid OBJ face index: '{token}'"))
                    })?;
                    let idx = if raw_idx > 0 {
                        (raw_idx - 1) as usize
                    } else if raw_idx < 0 {
                        let resolved = vertices.len() as isize + raw_idx;
                        if resolved < 0 {
                            return Err(MeshError::InvalidFormat(format!(
                                "negative OBJ index out of bounds: '{token}'"
                            )));
                        }
                        resolved as usize
                    } else {
                        return Err(MeshError::InvalidFormat(
                            "OBJ vertex index 0 is invalid".to_string(),
                        ));
                    };
                    let idx_u32 = u32::try_from(idx).map_err(|_| MeshError::TooManyVertices {
                        count: idx,
                        limit: u32::MAX as usize,
                    })?;
                    indices.push(idx_u32);
                }
                if indices.len() < 3 {
                    return Err(MeshError::InvalidFormat(format!(
                        "OBJ face has fewer than 3 vertices: '{trimmed}'"
                    )));
                }
                for i in 1..(indices.len() - 1) {
                    if faces.len() >= limits.max_triangles {
                        return Err(MeshError::TooManyTriangles {
                            count: faces.len() + 1,
                            limit: limits.max_triangles,
                        });
                    }
                    faces.push([indices[0], indices[i], indices[i + 1]]);
                    face_attrs.push(mesh_attributes::FaceAttributes {
                        material_id: current_material,
                        group_id: current_group,
                        ..Default::default()
                    });
                }
            }
            _ => {}
        }
    }

    attrs.origins = (0..faces.len())
        .map(|i| mesh_attributes::FaceOrigin::Identity(FaceId(i as u32)))
        .collect();
    attrs.confidence = vec![mesh_attributes::Confidence::Exact; faces.len()];
    attrs.faces = face_attrs;

    Ok(LoadedModel {
        mesh: TriangleMesh::from_parts(vertices, faces),
        attributes: attrs,
        source_format: FormatKind::Obj,
        warnings,
        threemf: None,
    })
}

fn parse_f64(token: &str, line: &str) -> Result<f64, MeshError> {
    token
        .parse::<f64>()
        .map_err(|_| MeshError::InvalidFormat(format!("invalid float '{token}' in '{line}'")))
}

/// Write a [`TriangleMesh`] and its categorical group/material assignments as Wavefront OBJ.
pub fn write_obj_path(
    path: &Path,
    mesh: &TriangleMesh,
    attributes: &AttributeStore,
) -> Result<(), MeshError> {
    let file = File::create(path)?;
    write_obj(BufWriter::new(file), mesh, attributes)
}

/// Write a [`TriangleMesh`] as Wavefront OBJ into `writer`.
pub fn write_obj<W: Write>(
    mut writer: W,
    mesh: &TriangleMesh,
    attributes: &AttributeStore,
) -> Result<(), MeshError> {
    writeln!(writer, "# rsSTLRepair OBJ export")?;
    for v in &mesh.vertices {
        if !v.x.is_finite() || !v.y.is_finite() || !v.z.is_finite() {
            return Err(MeshError::InvalidFormat(
                "cannot export OBJ containing non-finite vertex coordinates".to_string(),
            ));
        }
        writeln!(writer, "v {} {} {}", v.x, v.y, v.z)?;
    }

    let mut active_group: Option<u32> = None;
    let mut active_material: Option<u32> = None;
    let n_verts = mesh.vertices.len();

    for (fi, &[a, b, c]) in mesh.faces.iter().enumerate() {
        if a as usize >= n_verts || b as usize >= n_verts || c as usize >= n_verts {
            return Err(MeshError::InvalidFormat(format!(
                "face {fi} references out-of-range vertex index"
            )));
        }
        if let Some(fa) = attributes.faces.get(fi) {
            if fa.group_id != active_group {
                active_group = fa.group_id;
                if let Some(gid) = active_group
                    && let Some(name) = attributes.group_names.get(gid as usize)
                {
                    writeln!(writer, "g {name}")?;
                }
            }
            if fa.material_id != active_material {
                active_material = fa.material_id;
                if let Some(mid) = active_material
                    && let Some(name) = attributes.material_names.get(mid as usize)
                {
                    writeln!(writer, "usemtl {name}")?;
                }
            }
        }
        writeln!(writer, "f {} {} {}", a + 1, b + 1, c + 1)?;
    }
    writer.flush()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_analysis::analyze;

    fn unit_cube() -> TriangleMesh {
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(0.0, 1.0, 1.0),
        ];
        let faces = vec![
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [2, 3, 7],
            [2, 7, 6],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
        ];
        TriangleMesh::from_parts(vertices, faces)
    }

    #[test]
    fn binary_and_ascii_stl_roundtrip_same_cube() {
        let cube = unit_cube();
        let mut bin = Vec::new();
        write_stl_binary(&mut bin, &cube).expect("write binary STL");

        let loaded_bin = load_stl_slice(&bin, &ResourceLimits::default()).expect("read binary STL");
        assert_eq!(loaded_bin.source_format, FormatKind::StlBinary);
        let rep_bin = analyze(&loaded_bin.mesh);
        assert!(rep_bin.is_watertight);
        assert_eq!(rep_bin.face_count, 12);

        let mut ascii = String::from("solid cube\n");
        for fi in 0..cube.faces.len() {
            let [a, b, c] = cube.face_positions(FaceId(fi as u32)).expect("face");
            ascii.push_str("  facet normal 0 0 1\n    outer loop\n");
            for p in [a, b, c] {
                ascii.push_str(&format!("      vertex {} {} {}\n", p.x, p.y, p.z));
            }
            ascii.push_str("    endloop\n  endfacet\n");
        }
        ascii.push_str("endsolid cube\n");

        let loaded_ascii =
            load_stl_slice(ascii.as_bytes(), &ResourceLimits::default()).expect("read ascii STL");
        assert_eq!(loaded_ascii.source_format, FormatKind::StlAscii);
        let rep_ascii = analyze(&loaded_ascii.mesh);
        assert!(rep_ascii.is_watertight);
        assert_eq!(rep_ascii.vertex_count, rep_bin.vertex_count);
        assert_eq!(rep_ascii.face_count, rep_bin.face_count);
    }

    #[test]
    fn rejects_truncated_binary_stl_and_excessive_triangle_header() {
        let mut bogus = vec![0u8; 84];
        bogus[80..84].copy_from_slice(&(u32::MAX).to_le_bytes());
        let err = load_stl_slice(&bogus, &ResourceLimits::default()).expect_err("must reject");
        assert!(matches!(err, MeshError::TooManyTriangles { .. }));

        let mut truncated = vec![0u8; 90];
        truncated[80..84].copy_from_slice(&2u32.to_le_bytes());
        let err = load_stl_slice(&truncated, &ResourceLimits::default()).expect_err("must reject");
        assert!(matches!(err, MeshError::InvalidFormat(_)));
    }

    #[test]
    fn obj_preserves_groups_materials_and_duplicate_vertices() {
        let obj = "\
v 0 0 0
v 1 0 0
v 0 1 0
v 0 0 0
g top
usemtl red
f 1 2 3
";
        let loaded =
            load_obj_reader(std::io::Cursor::new(obj), &ResourceLimits::default()).expect("obj");
        assert_eq!(loaded.source_format, FormatKind::Obj);
        assert_eq!(loaded.attributes.material_names, vec!["red"]);
        assert_eq!(loaded.attributes.group_names, vec!["top"]);
        assert_eq!(analyze(&loaded.mesh).exact_duplicate_vertex_count, 1);

        let mut out = Vec::new();
        write_obj(&mut out, &loaded.mesh, &loaded.attributes).expect("write obj");
        let text = String::from_utf8(out).expect("utf8");
        assert!(text.contains("g top"));
        assert!(text.contains("usemtl red"));
    }

    #[test]
    fn refuses_sliced_or_traversal_3mf_and_preserves_bambu_paint() {
        // Test ZIP path traversal rejection
        let mut buf = std::io::Cursor::new(Vec::new());
        {
            let mut zw = zip::ZipWriter::new(&mut buf);
            let opts = zip::write::SimpleFileOptions::default();
            zw.start_file("../evil.model", opts).expect("start");
            std::io::Write::write_all(&mut zw, b"bad").expect("write");
            zw.finish().expect("finish");
        }
        let err = ThreeMfPackage::load_slice(&buf.into_inner(), &ResourceLimits::default())
            .expect_err("must reject traversal");
        assert!(matches!(err, MeshError::InvalidFormat(_)));

        // Test sliced G-code 3MF rejection
        let mut gbuf = std::io::Cursor::new(Vec::new());
        {
            let mut zw = zip::ZipWriter::new(&mut gbuf);
            let opts = zip::write::SimpleFileOptions::default();
            zw.start_file("Metadata/plate_1.gcode", opts)
                .expect("start");
            std::io::Write::write_all(&mut zw, b"G28\n").expect("write");
            zw.finish().expect("finish");
        }
        let gerr = ThreeMfPackage::load_slice(&gbuf.into_inner(), &ResourceLimits::default())
            .expect_err("must reject sliced gcode 3mf");
        assert!(matches!(gerr, MeshError::UnsupportedFormat(_)));
    }
}

#[cfg(test)]
mod extra_formats {
    use super::*;
    use mesh_attributes::AttributeStore;
    use mesh_core::ResourceLimits;
    use nalgebra::Point3;
    use proptest::prelude::*;

    fn tri() -> (TriangleMesh, AttributeStore) {
        let mesh = TriangleMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2]],
        );
        let mut attrs = AttributeStore::new_for_mesh(1);
        attrs.faces[0].material_id = Some(7);
        (mesh, attrs)
    }

    #[test]
    fn ply_ascii_roundtrip_keeps_material() {
        let (mesh, attrs) = tri();
        let mut buf = Vec::new();
        ply::write_ply(&mut buf, &mesh, &attrs).expect("write");
        let (loaded, loaded_attrs) =
            ply::load_ply_slice(&buf, &ResourceLimits::default()).expect("load");
        assert_eq!(loaded.faces, mesh.faces);
        assert!((loaded.vertices[1].x - 1.0).abs() < 1e-12);
        assert_eq!(loaded_attrs.faces[0].material_id, Some(7));
    }

    #[test]
    fn ply_binary_little_endian_reads_triangle() {
        let bytes = b"ply\nformat binary_little_endian 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n";
        let mut buf = bytes.to_vec();
        for v in [[0.0f32, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 2.0, 0.0]] {
            for c in v {
                buf.extend_from_slice(&c.to_le_bytes());
            }
        }
        buf.push(3);
        for i in [0u32, 1, 2] {
            buf.extend_from_slice(&i.to_le_bytes());
        }
        let (mesh, _) = ply::load_ply_slice(&buf, &ResourceLimits::default()).expect("binary");
        assert_eq!(mesh.faces, vec![[0, 1, 2]]);
        assert!((mesh.vertices[2].y - 2.0).abs() < 1e-6);
    }

    #[test]
    fn amf_roundtrip_keeps_material_and_rejects_doctype() {
        let (mesh, attrs) = tri();
        let mut buf = Vec::new();
        amf::write_amf(&mut buf, &mesh, &attrs).expect("write");
        let (loaded, loaded_attrs) =
            amf::load_amf_slice(&buf, &ResourceLimits::default()).expect("load");
        assert_eq!(loaded.faces, mesh.faces);
        assert_eq!(loaded_attrs.faces[0].material_id, Some(7));
        let hostile = br#"<?xml version="1.0"?><!DOCTYPE amf [<!ENTITY x "y">]><amf unit="millimeter"></amf>"#;
        assert!(amf::load_amf_slice(hostile, &ResourceLimits::default()).is_err());
    }

    #[test]
    fn gltf_and_glb_roundtrip_material_ids() {
        let (mesh, attrs) = tri();
        let mut gltf = Vec::new();
        gltf::write_gltf(&mut gltf, &mesh, &attrs).expect("gltf");
        let (loaded, loaded_attrs) =
            gltf::load_gltf_slice(&gltf, &ResourceLimits::default()).expect("load gltf");
        assert_eq!(loaded.faces, mesh.faces);
        assert!((loaded.vertices[0].x).abs() < 1e-5);
        assert_eq!(loaded_attrs.faces[0].material_id, Some(7));
        let mut glb = Vec::new();
        gltf::write_glb(&mut glb, &mesh, &attrs).expect("glb");
        let (loaded, _) =
            gltf::load_gltf_slice(&glb, &ResourceLimits::default()).expect("load glb");
        assert_eq!(loaded.faces.len(), 1);
    }

    fn tight() -> ResourceLimits {
        ResourceLimits {
            max_file_bytes: 4096,
            max_triangles: 32,
            max_vertices: 32,
            ..ResourceLimits::default()
        }
    }

    proptest! {
        #[test]
        fn random_bytes_do_not_panic(bytes in proptest::collection::vec(any::<u8>(), 0..512)) {
            let limits = tight();
            let _ = ply::load_ply_slice(&bytes, &limits);
            let _ = amf::load_amf_slice(&bytes, &limits);
            let _ = gltf::load_gltf_slice(&bytes, &limits);
            let _ = svg::load_svg_slice(&bytes, &limits);
            let _ = step::load_step_slice(&bytes, &limits);
            let _ = load_stl_slice(&bytes, &limits);
        }
    }
}
