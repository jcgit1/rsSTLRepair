//! ASCII and binary little-endian PLY with a fixed, known schema.
//!
//! Vertex properties must be `x`, `y`, `z` as `float` or `double`, in that order.
//! Faces are a `vertex_indices` list plus an optional integer `material_id`.
//! Any other property is rejected before the body is allocated.

use mesh_attributes::AttributeStore;
use mesh_core::{MeshError, ResourceLimits, TriangleMesh};
use nalgebra::Point3;
use std::io::Write;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Scalar {
    F32,
    F64,
    I32,
    U8,
    U16,
    U32,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum PlyEncoding {
    Ascii,
    BinaryLe,
}

struct Layout {
    encoding: PlyEncoding,
    vertex_count: usize,
    face_count: usize,
    vertex_type: Scalar,
    index_type: Scalar,
    count_type: Scalar,
    has_material: bool,
}

/// Load a PLY document from memory.
pub fn load_ply_slice(
    bytes: &[u8],
    limits: &ResourceLimits,
) -> Result<(TriangleMesh, AttributeStore), MeshError> {
    if bytes.len() as u64 > limits.max_file_bytes {
        return Err(MeshError::FileTooLarge {
            size: bytes.len() as u64,
            limit: limits.max_file_bytes,
        });
    }
    let header_end = find_header_end(bytes)?;
    let header = std::str::from_utf8(&bytes[..header_end])
        .map_err(|_| MeshError::InvalidFormat("PLY header is not UTF-8".to_string()))?;
    let layout = parse_header(header)?;
    check_counts(layout.vertex_count, layout.face_count, limits)?;
    let body = &bytes[header_end..];
    match layout.encoding {
        PlyEncoding::Ascii => parse_ascii(body, &layout),
        PlyEncoding::BinaryLe => parse_binary(body, &layout),
    }
}

/// Write an ASCII PLY document with full `f64` vertex precision.
pub fn write_ply<W: Write>(
    mut writer: W,
    mesh: &TriangleMesh,
    attributes: &AttributeStore,
) -> Result<(), MeshError> {
    let has_material = attributes.faces.iter().any(|f| f.material_id.is_some());
    writeln!(writer, "ply")?;
    writeln!(writer, "format ascii 1.0")?;
    writeln!(writer, "element vertex {}", mesh.vertices.len())?;
    writeln!(writer, "property double x")?;
    writeln!(writer, "property double y")?;
    writeln!(writer, "property double z")?;
    writeln!(writer, "element face {}", mesh.faces.len())?;
    writeln!(writer, "property list uchar int vertex_indices")?;
    if has_material {
        writeln!(writer, "property int material_id")?;
    }
    writeln!(writer, "end_header")?;
    for v in &mesh.vertices {
        writeln!(writer, "{:.17} {:.17} {:.17}", v.x, v.y, v.z)?;
    }
    for (i, face) in mesh.faces.iter().enumerate() {
        if has_material {
            let id = attributes
                .faces
                .get(i)
                .and_then(|f| f.material_id)
                .unwrap_or(0);
            writeln!(writer, "3 {} {} {} {id}", face[0], face[1], face[2])?;
        } else {
            writeln!(writer, "3 {} {} {} ", face[0], face[1], face[2])?;
        }
    }
    Ok(())
}

fn find_header_end(bytes: &[u8]) -> Result<usize, MeshError> {
    const MARK: &[u8] = b"end_header";
    let limit = bytes.len().min(1_048_576);
    let window = &bytes[..limit];
    let Some(pos) = window.windows(MARK.len()).position(|w| w == MARK) else {
        return Err(MeshError::InvalidFormat(
            "PLY header missing end_header".to_string(),
        ));
    };
    let mut end = pos + MARK.len();
    if bytes.get(end) == Some(&b'\r') {
        end += 1;
    }
    if bytes.get(end) == Some(&b'\n') {
        end += 1;
    } else {
        return Err(MeshError::InvalidFormat(
            "PLY end_header must end with a newline".to_string(),
        ));
    }
    Ok(end)
}

fn parse_header(header: &str) -> Result<Layout, MeshError> {
    let mut lines = header.lines();
    let magic = lines
        .next()
        .ok_or_else(|| MeshError::InvalidFormat("empty PLY".to_string()))?
        .trim();
    if magic != "ply" {
        return Err(MeshError::InvalidFormat("PLY magic missing".to_string()));
    }
    let mut encoding = None;
    let mut element = None;
    let mut vertex_count = None;
    let mut face_count = None;
    let mut vertex_props = Vec::new();
    let mut face_list = None;
    let mut has_material = false;
    for line in lines {
        let line = line.trim();
        if line.is_empty() || line.starts_with("comment") || line.starts_with("obj_info") {
            continue;
        }
        if line == "end_header" {
            break;
        }
        let mut parts = line.split_whitespace();
        let key = parts.next().unwrap_or("");
        match key {
            "format" => {
                let fmt = parts.next().unwrap_or("");
                encoding = Some(match fmt {
                    "ascii" => PlyEncoding::Ascii,
                    "binary_little_endian" => PlyEncoding::BinaryLe,
                    other => {
                        return Err(MeshError::UnsupportedFormat(format!(
                            "PLY format '{other}' is not supported"
                        )));
                    }
                });
            }
            "element" => {
                let name = parts.next().unwrap_or("");
                let count = parts
                    .next()
                    .ok_or_else(|| {
                        MeshError::InvalidFormat("PLY element missing count".to_string())
                    })?
                    .parse::<usize>()
                    .map_err(|_| {
                        MeshError::InvalidFormat("PLY element count is not an integer".to_string())
                    })?;
                element = Some(name.to_string());
                match name {
                    "vertex" => vertex_count = Some(count),
                    "face" => face_count = Some(count),
                    other => {
                        return Err(MeshError::UnsupportedFormat(format!(
                            "PLY element '{other}' is not supported"
                        )));
                    }
                }
            }
            "property" => {
                let which = element.as_deref().unwrap_or("");
                if which == "vertex" {
                    let ty = parts.next().unwrap_or("");
                    let name = parts.next().unwrap_or("");
                    vertex_props.push((name.to_string(), scalar(ty)?));
                } else if which == "face" {
                    let ty = parts.next().unwrap_or("");
                    if ty == "list" {
                        if face_list.is_some() {
                            return Err(MeshError::UnsupportedFormat(
                                "PLY face has more than one list property".to_string(),
                            ));
                        }
                        let count_ty = scalar(parts.next().unwrap_or(""))?;
                        let index_ty = scalar(parts.next().unwrap_or(""))?;
                        let name = parts.next().unwrap_or("");
                        if name != "vertex_indices" && name != "vertex_index" {
                            return Err(MeshError::UnsupportedFormat(format!(
                                "PLY face list '{name}' is not vertex indices"
                            )));
                        }
                        face_list = Some((count_ty, index_ty));
                    } else {
                        let name = parts.next().unwrap_or("");
                        if name != "material_id" {
                            return Err(MeshError::UnsupportedFormat(format!(
                                "PLY face property '{name}' is not supported"
                            )));
                        }
                        let _ = scalar(ty)?;
                        has_material = true;
                    }
                } else {
                    return Err(MeshError::UnsupportedFormat(
                        "PLY property outside vertex or face".to_string(),
                    ));
                }
            }
            other => {
                return Err(MeshError::UnsupportedFormat(format!(
                    "PLY header keyword '{other}' is not supported"
                )));
            }
        }
    }
    let encoding =
        encoding.ok_or_else(|| MeshError::InvalidFormat("PLY format line missing".to_string()))?;
    let vertex_count = vertex_count
        .ok_or_else(|| MeshError::InvalidFormat("PLY vertex element missing".to_string()))?;
    let face_count = face_count
        .ok_or_else(|| MeshError::InvalidFormat("PLY face element missing".to_string()))?;
    if vertex_props.len() != 3
        || vertex_props[0].0 != "x"
        || vertex_props[1].0 != "y"
        || vertex_props[2].0 != "z"
    {
        return Err(MeshError::UnsupportedFormat(
            "PLY vertices must be exactly properties x, y, z".to_string(),
        ));
    }
    if vertex_props.iter().any(|(_, ty)| *ty != vertex_props[0].1) {
        return Err(MeshError::UnsupportedFormat(
            "PLY x, y, z must share one numeric type".to_string(),
        ));
    }
    if !matches!(vertex_props[0].1, Scalar::F32 | Scalar::F64) {
        return Err(MeshError::UnsupportedFormat(
            "PLY x, y, z must be float or double".to_string(),
        ));
    }
    let (count_type, index_type) = face_list
        .ok_or_else(|| MeshError::InvalidFormat("PLY face list property missing".to_string()))?;
    Ok(Layout {
        encoding,
        vertex_count,
        face_count,
        vertex_type: vertex_props[0].1,
        index_type,
        count_type,
        has_material,
    })
}

fn scalar(name: &str) -> Result<Scalar, MeshError> {
    Ok(match name {
        "float" | "float32" => Scalar::F32,
        "double" | "float64" => Scalar::F64,
        "int" | "int32" => Scalar::I32,
        "uchar" | "uint8" => Scalar::U8,
        "ushort" | "uint16" => Scalar::U16,
        "uint" | "uint32" => Scalar::U32,
        other => {
            return Err(MeshError::UnsupportedFormat(format!(
                "PLY type '{other}' is not supported"
            )));
        }
    })
}

fn check_counts(verts: usize, faces: usize, limits: &ResourceLimits) -> Result<(), MeshError> {
    if verts > limits.max_vertices {
        return Err(MeshError::TooManyVertices {
            count: verts,
            limit: limits.max_vertices,
        });
    }
    if faces > limits.max_triangles {
        return Err(MeshError::TooManyTriangles {
            count: faces,
            limit: limits.max_triangles,
        });
    }
    Ok(())
}

fn parse_ascii(body: &[u8], layout: &Layout) -> Result<(TriangleMesh, AttributeStore), MeshError> {
    let text = std::str::from_utf8(body)
        .map_err(|_| MeshError::InvalidFormat("PLY ASCII body is not UTF-8".to_string()))?;
    let mut lines = text.lines().filter(|l| !l.trim().is_empty());
    let mut vertices = Vec::with_capacity(layout.vertex_count);
    for _ in 0..layout.vertex_count {
        let line = lines
            .next()
            .ok_or_else(|| MeshError::InvalidFormat("PLY ended inside vertices".to_string()))?;
        let mut nums = line.split_whitespace();
        let x = parse_f64(nums.next())?;
        let y = parse_f64(nums.next())?;
        let z = parse_f64(nums.next())?;
        if nums.next().is_some() {
            return Err(MeshError::InvalidFormat(
                "PLY vertex has extra values".to_string(),
            ));
        }
        vertices.push(Point3::new(x, y, z));
    }
    let mut faces = Vec::with_capacity(layout.face_count);
    let mut attrs = AttributeStore::new_for_mesh(layout.face_count);
    for i in 0..layout.face_count {
        let line = lines
            .next()
            .ok_or_else(|| MeshError::InvalidFormat("PLY ended inside faces".to_string()))?;
        let mut nums = line.split_whitespace();
        let n = nums
            .next()
            .ok_or_else(|| MeshError::InvalidFormat("PLY face is empty".to_string()))?
            .parse::<usize>()
            .map_err(|_| {
                MeshError::InvalidFormat("PLY face count is not an integer".to_string())
            })?;
        if n != 3 {
            return Err(MeshError::UnsupportedFormat(
                "PLY faces must be triangles".to_string(),
            ));
        }
        let a = parse_index(nums.next(), layout.vertex_count)?;
        let b = parse_index(nums.next(), layout.vertex_count)?;
        let c = parse_index(nums.next(), layout.vertex_count)?;
        faces.push([a, b, c]);
        if layout.has_material {
            let id = nums
                .next()
                .ok_or_else(|| MeshError::InvalidFormat("PLY material_id missing".to_string()))?
                .parse::<i64>()
                .map_err(|_| {
                    MeshError::InvalidFormat("PLY material_id is not an integer".to_string())
                })?;
            if id < 0 {
                return Err(MeshError::InvalidFormat(
                    "PLY material_id is negative".to_string(),
                ));
            }
            attrs.faces[i].material_id = Some(id as u32);
        }
        if nums.next().is_some() {
            return Err(MeshError::InvalidFormat(
                "PLY face has extra values".to_string(),
            ));
        }
    }
    Ok((TriangleMesh::from_parts(vertices, faces), attrs))
}

fn parse_binary(body: &[u8], layout: &Layout) -> Result<(TriangleMesh, AttributeStore), MeshError> {
    let mut cursor = 0usize;
    let mut vertices = Vec::with_capacity(layout.vertex_count);
    for _ in 0..layout.vertex_count {
        let x = read_f64(body, &mut cursor, layout.vertex_type)?;
        let y = read_f64(body, &mut cursor, layout.vertex_type)?;
        let z = read_f64(body, &mut cursor, layout.vertex_type)?;
        vertices.push(Point3::new(x, y, z));
    }
    let mut faces = Vec::with_capacity(layout.face_count);
    let mut attrs = AttributeStore::new_for_mesh(layout.face_count);
    for i in 0..layout.face_count {
        let n = read_uint(body, &mut cursor, layout.count_type)?;
        if n != 3 {
            return Err(MeshError::UnsupportedFormat(
                "PLY faces must be triangles".to_string(),
            ));
        }
        let a = read_index(body, &mut cursor, layout.index_type, layout.vertex_count)?;
        let b = read_index(body, &mut cursor, layout.index_type, layout.vertex_count)?;
        let c = read_index(body, &mut cursor, layout.index_type, layout.vertex_count)?;
        faces.push([a, b, c]);
        if layout.has_material {
            let id = read_uint(body, &mut cursor, Scalar::I32)?;
            attrs.faces[i].material_id = Some(id);
        }
    }
    Ok((TriangleMesh::from_parts(vertices, faces), attrs))
}

fn parse_f64(token: Option<&str>) -> Result<f64, MeshError> {
    let token = token.ok_or_else(|| MeshError::InvalidFormat("PLY vertex is short".to_string()))?;
    token
        .parse::<f64>()
        .map_err(|_| MeshError::InvalidFormat(format!("PLY value '{token}' is not a number")))
}

fn parse_index(token: Option<&str>, verts: usize) -> Result<u32, MeshError> {
    let token =
        token.ok_or_else(|| MeshError::InvalidFormat("PLY face index missing".to_string()))?;
    let value = token
        .parse::<i64>()
        .map_err(|_| MeshError::InvalidFormat("PLY face index is not an integer".to_string()))?;
    if value < 0 || value as usize >= verts {
        return Err(MeshError::InvalidFormat(
            "PLY face index is out of range".to_string(),
        ));
    }
    Ok(value as u32)
}

fn read_f64(body: &[u8], cursor: &mut usize, ty: Scalar) -> Result<f64, MeshError> {
    Ok(match ty {
        Scalar::F32 => f32::from_le_bytes(read_n(body, cursor, 4)?.try_into().unwrap()) as f64,
        Scalar::F64 => f64::from_le_bytes(read_n(body, cursor, 8)?.try_into().unwrap()),
        _ => {
            return Err(MeshError::InvalidFormat(
                "PLY vertex type is not floating point".to_string(),
            ));
        }
    })
}

fn read_uint(body: &[u8], cursor: &mut usize, ty: Scalar) -> Result<u32, MeshError> {
    let value = match ty {
        Scalar::U8 => {
            let b = read_n(body, cursor, 1)?;
            b[0] as u64
        }
        Scalar::U16 => u16::from_le_bytes(read_n(body, cursor, 2)?.try_into().unwrap()) as u64,
        Scalar::U32 => u32::from_le_bytes(read_n(body, cursor, 4)?.try_into().unwrap()) as u64,
        Scalar::I32 => i32::from_le_bytes(read_n(body, cursor, 4)?.try_into().unwrap()) as u64,
        _ => {
            return Err(MeshError::UnsupportedFormat(
                "PLY integer width is not supported".to_string(),
            ));
        }
    };
    u32::try_from(value)
        .map_err(|_| MeshError::InvalidFormat("PLY integer does not fit u32".to_string()))
}

fn read_index(body: &[u8], cursor: &mut usize, ty: Scalar, verts: usize) -> Result<u32, MeshError> {
    let value = read_uint(body, cursor, ty)?;
    if value as usize >= verts {
        return Err(MeshError::InvalidFormat(
            "PLY face index is out of range".to_string(),
        ));
    }
    Ok(value)
}

fn read_n(body: &[u8], cursor: &mut usize, n: usize) -> Result<Vec<u8>, MeshError> {
    if body.len().saturating_sub(*cursor) < n {
        return Err(MeshError::InvalidFormat(
            "PLY binary body ended early".to_string(),
        ));
    }
    let out = body[*cursor..*cursor + n].to_vec();
    *cursor += n;
    Ok(out)
}
