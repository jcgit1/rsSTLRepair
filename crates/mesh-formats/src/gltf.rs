//! One-mesh glTF 2.0 and GLB reader and writer.
//!
//! The primitive must be `POSITION` float `VEC3` plus `UNSIGNED_INT` indices.
//! `.gltf` buffers must be embedded base64. `.glb` is a JSON chunk plus a BIN chunk.
//! Face material ids live in `primitive.extras.material_ids`. This is not a general
//! glTF scene loader.

use mesh_attributes::AttributeStore;
use mesh_core::{MeshError, ResourceLimits, TriangleMesh};
use nalgebra::Point3;
use serde_json::{Value, json};
use std::io::Write;

/// Load a `.gltf` JSON document or a `.glb` binary from memory.
pub fn load_gltf_slice(
    bytes: &[u8],
    limits: &ResourceLimits,
) -> Result<(TriangleMesh, AttributeStore), MeshError> {
    if bytes.len() as u64 > limits.max_file_bytes {
        return Err(MeshError::FileTooLarge {
            size: bytes.len() as u64,
            limit: limits.max_file_bytes,
        });
    }
    if bytes.starts_with(b"glTF") {
        let (json, bin) = split_glb(bytes)?;
        decode_document(&json, &bin, limits)
    } else {
        let json: Value = serde_json::from_slice(bytes)
            .map_err(|err| MeshError::InvalidFormat(format!("glTF JSON error: {err}")))?;
        let bin = embedded_buffer(&json)?;
        decode_document(&json, &bin, limits)
    }
}

/// Write a `.gltf` document with one embedded base64 buffer.
pub fn write_gltf<W: Write>(
    mut writer: W,
    mesh: &TriangleMesh,
    attributes: &AttributeStore,
) -> Result<(), MeshError> {
    let (json, bin) = document(mesh, attributes, true)?;
    let _ = bin;
    writer.write_all(
        serde_json::to_vec_pretty(&json)
            .map_err(|err| MeshError::InvalidFormat(format!("glTF encode error: {err}")))?
            .as_slice(),
    )?;
    Ok(())
}

/// Write a `.glb` binary (JSON chunk + BIN chunk).
pub fn write_glb<W: Write>(
    mut writer: W,
    mesh: &TriangleMesh,
    attributes: &AttributeStore,
) -> Result<(), MeshError> {
    let (json, bin) = document(mesh, attributes, false)?;
    let json_bytes = serde_json::to_vec(&json)
        .map_err(|err| MeshError::InvalidFormat(format!("glTF encode error: {err}")))?;
    let json_pad = pad4(json_bytes.len());
    let bin_pad = pad4(bin.len());
    let total = 12 + 8 + json_bytes.len() + json_pad + 8 + bin.len() + bin_pad;
    writer.write_all(b"glTF")?;
    writer.write_all(&2u32.to_le_bytes())?;
    writer.write_all(&(total as u32).to_le_bytes())?;
    writer.write_all(&((json_bytes.len() + json_pad) as u32).to_le_bytes())?;
    writer.write_all(&0x4E4F534Au32.to_le_bytes())?;
    writer.write_all(&json_bytes)?;
    writer.write_all(&vec![b' '; json_pad])?;
    writer.write_all(&((bin.len() + bin_pad) as u32).to_le_bytes())?;
    writer.write_all(&0x004E4942u32.to_le_bytes())?;
    writer.write_all(&bin)?;
    writer.write_all(&vec![0u8; bin_pad])?;
    Ok(())
}

fn document(
    mesh: &TriangleMesh,
    attributes: &AttributeStore,
    embed: bool,
) -> Result<(Value, Vec<u8>), MeshError> {
    let mut bin = Vec::with_capacity(mesh.vertices.len() * 12 + mesh.faces.len() * 12);
    for v in &mesh.vertices {
        bin.extend_from_slice(&(v.x as f32).to_le_bytes());
        bin.extend_from_slice(&(v.y as f32).to_le_bytes());
        bin.extend_from_slice(&(v.z as f32).to_le_bytes());
    }
    let index_offset = bin.len();
    for face in &mesh.faces {
        for index in face {
            bin.extend_from_slice(&index.to_le_bytes());
        }
    }
    let materials: Vec<Value> = (0..mesh.faces.len())
        .map(
            |i| match attributes.faces.get(i).and_then(|f| f.material_id) {
                Some(id) => Value::from(id),
                None => Value::Null,
            },
        )
        .collect();
    let has_materials = materials.iter().any(|v| !v.is_null());
    let buffer = if embed {
        json!({
            "uri": format!("data:application/octet-stream;base64,{}", b64_encode(&bin)),
            "byteLength": bin.len()
        })
    } else {
        json!({ "byteLength": bin.len() })
    };
    let mut primitive = json!({
        "attributes": { "POSITION": 0 },
        "indices": 1,
        "mode": 4
    });
    if has_materials {
        primitive["extras"] = json!({ "material_ids": materials });
    }
    let json = json!({
        "asset": { "version": "2.0" },
        "buffers": [buffer],
        "bufferViews": [
            { "buffer": 0, "byteOffset": 0, "byteLength": index_offset, "target": 34962 },
            { "buffer": 0, "byteOffset": index_offset, "byteLength": bin.len() - index_offset, "target": 34963 }
        ],
        "accessors": [
            {
                "bufferView": 0,
                "componentType": 5126,
                "count": mesh.vertices.len(),
                "type": "VEC3"
            },
            {
                "bufferView": 1,
                "componentType": 5125,
                "count": mesh.faces.len() * 3,
                "type": "SCALAR"
            }
        ],
        "meshes": [{ "primitives": [primitive] }],
        "nodes": [{ "mesh": 0 }],
        "scenes": [{ "nodes": [0] }],
        "scene": 0
    });
    Ok((json, bin))
}

fn split_glb(bytes: &[u8]) -> Result<(Value, Vec<u8>), MeshError> {
    if bytes.len() < 20 || &bytes[0..4] != b"glTF" {
        return Err(MeshError::InvalidFormat("GLB magic missing".to_string()));
    }
    let version = u32::from_le_bytes(chunk(bytes, 4)?);
    if version != 2 {
        return Err(MeshError::UnsupportedFormat(format!(
            "GLB version {version} is not supported"
        )));
    }
    let length = u32::from_le_bytes(chunk(bytes, 8)?) as usize;
    if length > bytes.len() {
        return Err(MeshError::InvalidFormat(
            "GLB length exceeds the buffer".to_string(),
        ));
    }
    let json_len = u32::from_le_bytes(chunk(bytes, 12)?) as usize;
    let json_type = u32::from_le_bytes(chunk(bytes, 16)?);
    if json_type != 0x4E4F534A {
        return Err(MeshError::InvalidFormat(
            "GLB first chunk is not JSON".to_string(),
        ));
    }
    let json_start: usize = 20;
    let json_end = json_start
        .checked_add(json_len)
        .ok_or_else(|| MeshError::InvalidFormat("GLB JSON chunk overflows".to_string()))?;
    if json_end > bytes.len() {
        return Err(MeshError::InvalidFormat(
            "GLB JSON chunk exceeds the buffer".to_string(),
        ));
    }
    let json: Value = serde_json::from_slice(&bytes[json_start..json_end])
        .map_err(|err| MeshError::InvalidFormat(format!("GLB JSON error: {err}")))?;
    if json_end + 8 > bytes.len() {
        return Ok((json, Vec::new()));
    }
    let bin_len = u32::from_le_bytes(chunk(bytes, json_end)?) as usize;
    let bin_type = u32::from_le_bytes(chunk(bytes, json_end + 4)?);
    if bin_type != 0x004E4942 {
        return Err(MeshError::InvalidFormat(
            "GLB second chunk is not BIN".to_string(),
        ));
    }
    let bin_start = json_end + 8;
    let bin_end = bin_start
        .checked_add(bin_len)
        .ok_or_else(|| MeshError::InvalidFormat("GLB BIN chunk overflows".to_string()))?;
    if bin_end > bytes.len() {
        return Err(MeshError::InvalidFormat(
            "GLB BIN chunk exceeds the buffer".to_string(),
        ));
    }
    Ok((json, bytes[bin_start..bin_end].to_vec()))
}

fn chunk(bytes: &[u8], at: usize) -> Result<[u8; 4], MeshError> {
    let end = at
        .checked_add(4)
        .ok_or_else(|| MeshError::InvalidFormat("GLB header is short".to_string()))?;
    if end > bytes.len() {
        return Err(MeshError::InvalidFormat("GLB header is short".to_string()));
    }
    let mut out = [0u8; 4];
    out.copy_from_slice(&bytes[at..end]);
    Ok(out)
}

fn embedded_buffer(json: &Value) -> Result<Vec<u8>, MeshError> {
    let buffers = json
        .get("buffers")
        .and_then(|v| v.as_array())
        .ok_or_else(|| MeshError::InvalidFormat("glTF buffers array missing".to_string()))?;
    if buffers.len() != 1 {
        return Err(MeshError::UnsupportedFormat(
            "glTF must contain exactly one buffer".to_string(),
        ));
    }
    let uri = buffers[0]
        .get("uri")
        .and_then(|v| v.as_str())
        .ok_or_else(|| {
            MeshError::UnsupportedFormat("glTF buffer must be embedded base64".to_string())
        })?;
    let Some(data) = uri.strip_prefix("data:") else {
        return Err(MeshError::UnsupportedFormat(
            "glTF external buffer URIs are not loaded".to_string(),
        ));
    };
    let Some((_, b64)) = data.split_once(";base64,") else {
        return Err(MeshError::InvalidFormat(
            "glTF data URI is not base64".to_string(),
        ));
    };
    b64_decode(b64)
}

fn decode_document(
    json: &Value,
    bin: &[u8],
    limits: &ResourceLimits,
) -> Result<(TriangleMesh, AttributeStore), MeshError> {
    let version = json
        .pointer("/asset/version")
        .and_then(|v| v.as_str())
        .unwrap_or("");
    if version != "2.0" {
        return Err(MeshError::UnsupportedFormat(
            "glTF asset version must be 2.0".to_string(),
        ));
    }
    let meshes = json
        .get("meshes")
        .and_then(|v| v.as_array())
        .ok_or_else(|| MeshError::InvalidFormat("glTF meshes array missing".to_string()))?;
    if meshes.len() != 1 {
        return Err(MeshError::UnsupportedFormat(
            "glTF must contain exactly one mesh".to_string(),
        ));
    }
    let primitives = meshes[0]
        .get("primitives")
        .and_then(|v| v.as_array())
        .ok_or_else(|| MeshError::InvalidFormat("glTF primitives missing".to_string()))?;
    if primitives.len() != 1 {
        return Err(MeshError::UnsupportedFormat(
            "glTF mesh must contain exactly one primitive".to_string(),
        ));
    }
    let primitive = &primitives[0];
    let position = primitive
        .pointer("/attributes/POSITION")
        .and_then(|v| v.as_u64())
        .ok_or_else(|| MeshError::InvalidFormat("glTF POSITION accessor missing".to_string()))?
        as usize;
    let indices = primitive
        .get("indices")
        .and_then(|v| v.as_u64())
        .ok_or_else(|| MeshError::InvalidFormat("glTF indices accessor missing".to_string()))?
        as usize;
    let accessors = json
        .get("accessors")
        .and_then(|v| v.as_array())
        .ok_or_else(|| MeshError::InvalidFormat("glTF accessors missing".to_string()))?;
    let views = json
        .get("bufferViews")
        .and_then(|v| v.as_array())
        .ok_or_else(|| MeshError::InvalidFormat("glTF bufferViews missing".to_string()))?;
    let (pos_off, pos_count) = accessor_span(accessors, views, position, "VEC3", 5126)?;
    let (idx_off, idx_count) = accessor_span(accessors, views, indices, "SCALAR", 5125)?;
    if pos_count > limits.max_vertices {
        return Err(MeshError::TooManyVertices {
            count: pos_count,
            limit: limits.max_vertices,
        });
    }
    if idx_count % 3 != 0 {
        return Err(MeshError::InvalidFormat(
            "glTF index count is not a multiple of 3".to_string(),
        ));
    }
    let face_count = idx_count / 3;
    if face_count > limits.max_triangles {
        return Err(MeshError::TooManyTriangles {
            count: face_count,
            limit: limits.max_triangles,
        });
    }
    let mut vertices = Vec::with_capacity(pos_count);
    for i in 0..pos_count {
        let at = pos_off + i * 12;
        let x = read_f32(bin, at)? as f64;
        let y = read_f32(bin, at + 4)? as f64;
        let z = read_f32(bin, at + 8)? as f64;
        vertices.push(Point3::new(x, y, z));
    }
    let mut faces = Vec::with_capacity(face_count);
    for i in 0..face_count {
        let at = idx_off + i * 12;
        let a = read_u32(bin, at)?;
        let b = read_u32(bin, at + 4)?;
        let c = read_u32(bin, at + 8)?;
        if [a, b, c].iter().any(|v| *v as usize >= vertices.len()) {
            return Err(MeshError::InvalidFormat(
                "glTF index is out of range".to_string(),
            ));
        }
        faces.push([a, b, c]);
    }
    let mut attrs = AttributeStore::new_for_mesh(faces.len());
    if let Some(ids) = primitive
        .pointer("/extras/material_ids")
        .and_then(|v| v.as_array())
    {
        for (face, id) in attrs.faces.iter_mut().zip(ids) {
            if let Some(n) = id.as_u64() {
                face.material_id = Some(u32::try_from(n).unwrap_or(u32::MAX));
            }
        }
    }
    Ok((TriangleMesh::from_parts(vertices, faces), attrs))
}

fn accessor_span(
    accessors: &[Value],
    views: &[Value],
    index: usize,
    ty: &str,
    component: u64,
) -> Result<(usize, usize), MeshError> {
    let accessor = accessors.get(index).ok_or_else(|| {
        MeshError::InvalidFormat("glTF accessor index is out of range".to_string())
    })?;
    if accessor.get("type").and_then(|v| v.as_str()) != Some(ty)
        || accessor.get("componentType").and_then(|v| v.as_u64()) != Some(component)
    {
        return Err(MeshError::UnsupportedFormat(format!(
            "glTF accessor {index} must be {ty} component {component}"
        )));
    }
    let count = accessor
        .get("count")
        .and_then(|v| v.as_u64())
        .ok_or_else(|| MeshError::InvalidFormat("glTF accessor count missing".to_string()))?
        as usize;
    let view_index = accessor
        .get("bufferView")
        .and_then(|v| v.as_u64())
        .ok_or_else(|| MeshError::InvalidFormat("glTF accessor bufferView missing".to_string()))?
        as usize;
    let view = views.get(view_index).ok_or_else(|| {
        MeshError::InvalidFormat("glTF bufferView index is out of range".to_string())
    })?;
    if view.get("buffer").and_then(|v| v.as_u64()) != Some(0) {
        return Err(MeshError::UnsupportedFormat(
            "glTF bufferView must use buffer 0".to_string(),
        ));
    }
    let view_off = view.get("byteOffset").and_then(|v| v.as_u64()).unwrap_or(0) as usize;
    let acc_off = accessor
        .get("byteOffset")
        .and_then(|v| v.as_u64())
        .unwrap_or(0) as usize;
    Ok((view_off + acc_off, count))
}

fn read_f32(bin: &[u8], at: usize) -> Result<f32, MeshError> {
    let end = at
        .checked_add(4)
        .ok_or_else(|| MeshError::InvalidFormat("glTF buffer ended early".to_string()))?;
    if end > bin.len() {
        return Err(MeshError::InvalidFormat(
            "glTF buffer ended early".to_string(),
        ));
    }
    Ok(f32::from_le_bytes(bin[at..end].try_into().unwrap()))
}

fn read_u32(bin: &[u8], at: usize) -> Result<u32, MeshError> {
    let end = at
        .checked_add(4)
        .ok_or_else(|| MeshError::InvalidFormat("glTF buffer ended early".to_string()))?;
    if end > bin.len() {
        return Err(MeshError::InvalidFormat(
            "glTF buffer ended early".to_string(),
        ));
    }
    Ok(u32::from_le_bytes(bin[at..end].try_into().unwrap()))
}

fn pad4(len: usize) -> usize {
    (4 - (len % 4)) % 4
}

fn b64_encode(bytes: &[u8]) -> String {
    const T: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::new();
    let mut i = 0;
    while i + 3 <= bytes.len() {
        let n = ((bytes[i] as u32) << 16) | ((bytes[i + 1] as u32) << 8) | bytes[i + 2] as u32;
        out.push(T[((n >> 18) & 63) as usize] as char);
        out.push(T[((n >> 12) & 63) as usize] as char);
        out.push(T[((n >> 6) & 63) as usize] as char);
        out.push(T[(n & 63) as usize] as char);
        i += 3;
    }
    if i < bytes.len() {
        let rem = bytes.len() - i;
        let mut n = (bytes[i] as u32) << 16;
        if rem == 2 {
            n |= (bytes[i + 1] as u32) << 8;
        }
        out.push(T[((n >> 18) & 63) as usize] as char);
        out.push(T[((n >> 12) & 63) as usize] as char);
        if rem == 2 {
            out.push(T[((n >> 6) & 63) as usize] as char);
            out.push('=');
        } else {
            out.push('=');
            out.push('=');
        }
    }
    out
}

fn b64_decode(text: &str) -> Result<Vec<u8>, MeshError> {
    fn val(c: u8) -> Result<u8, MeshError> {
        Ok(match c {
            b'A'..=b'Z' => c - b'A',
            b'a'..=b'z' => c - b'a' + 26,
            b'0'..=b'9' => c - b'0' + 52,
            b'+' => 62,
            b'/' => 63,
            _ => {
                return Err(MeshError::InvalidFormat(
                    "glTF base64 is invalid".to_string(),
                ));
            }
        })
    }
    let bytes = text
        .as_bytes()
        .iter()
        .copied()
        .filter(|c| !c.is_ascii_whitespace())
        .collect::<Vec<_>>();
    if bytes.len() % 4 != 0 {
        return Err(MeshError::InvalidFormat(
            "glTF base64 length is invalid".to_string(),
        ));
    }
    let mut out = Vec::with_capacity(bytes.len() / 4 * 3);
    for chunk in bytes.chunks(4) {
        let pad = chunk.iter().rev().take_while(|c| **c == b'=').count();
        if pad > 2 {
            return Err(MeshError::InvalidFormat(
                "glTF base64 padding is invalid".to_string(),
            ));
        }
        let a = val(chunk[0])?;
        let b = val(chunk[1])?;
        let c = if pad < 2 { val(chunk[2])? } else { 0 };
        let d = if pad < 1 { val(chunk[3])? } else { 0 };
        let n = ((a as u32) << 18) | ((b as u32) << 12) | ((c as u32) << 6) | d as u32;
        out.push((n >> 16) as u8);
        if pad < 2 {
            out.push((n >> 8) as u8);
        }
        if pad < 1 {
            out.push(n as u8);
        }
    }
    Ok(out)
}
