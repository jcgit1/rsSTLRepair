//! Tessellated STEP import.
//!
//! A file is accepted when its DATA section contains `CARTESIAN_POINT` entities
//! and `POLY_LOOP` entities of exactly three of those points. Analytic faces
//! (cylinders, NURBS, and any loop that is not already a triangle) are refused.
//! There is no STEP writer.

use mesh_attributes::AttributeStore;
use mesh_core::{MeshError, ResourceLimits, TriangleMesh};
use nalgebra::Point3;
use std::collections::BTreeMap;

/// Load triangular `POLY_LOOP` faces from an ISO-10303-21 STEP file.
pub fn load_step_slice(
    bytes: &[u8],
    limits: &ResourceLimits,
) -> Result<(TriangleMesh, AttributeStore), MeshError> {
    if bytes.len() as u64 > limits.max_file_bytes {
        return Err(MeshError::FileTooLarge {
            size: bytes.len() as u64,
            limit: limits.max_file_bytes,
        });
    }
    let text = std::str::from_utf8(bytes)
        .map_err(|_| MeshError::InvalidFormat("STEP file is not UTF-8".to_string()))?;
    let trimmed = text.trim_start();
    if !trimmed.to_ascii_uppercase().starts_with("ISO-10303-21") {
        return Err(MeshError::InvalidFormat(
            "STEP file does not start with ISO-10303-21".to_string(),
        ));
    }
    let data = data_section(text)?;
    let entities = split_entities(&data)?;
    let mut points: BTreeMap<u64, Point3<f64>> = BTreeMap::new();
    let mut loops = Vec::new();
    for (id, name, body) in &entities {
        match name.as_str() {
            "CARTESIAN_POINT" => {
                points.insert(*id, cartesian_point(body)?);
            }
            "POLY_LOOP" => loops.push(poly_loop(body)?),
            _ => {}
        }
    }
    let mut triangles = Vec::with_capacity(loops.len());
    for ids in loops {
        if ids.len() != 3 {
            return Err(MeshError::UnsupportedFormat(
                "STEP file has a POLY_LOOP that is not a triangle. Analytic solids and non-triangular faces are not meshed.".to_string(),
            ));
        }
        triangles.push(ids);
    }
    if triangles.is_empty() {
        return Err(MeshError::UnsupportedFormat(
            "STEP file has no triangular POLY_LOOP faces. Analytic solids (cylinders, NURBS, and other non-triangular faces) are not meshed.".to_string(),
        ));
    }
    if triangles.len() > limits.max_triangles {
        return Err(MeshError::TooManyTriangles {
            count: triangles.len(),
            limit: limits.max_triangles,
        });
    }
    let mut index_of: BTreeMap<u64, u32> = BTreeMap::new();
    let mut vertices = Vec::new();
    let mut faces = Vec::with_capacity(triangles.len());
    for tri in triangles {
        let mut face = [0u32; 3];
        for (slot, id) in tri.iter().enumerate() {
            let point = points.get(id).ok_or_else(|| {
                MeshError::InvalidFormat(format!("STEP POLY_LOOP references missing point #{id}"))
            })?;
            let idx = if let Some(existing) = index_of.get(id) {
                *existing
            } else {
                if vertices.len() >= limits.max_vertices {
                    return Err(MeshError::TooManyVertices {
                        count: vertices.len() + 1,
                        limit: limits.max_vertices,
                    });
                }
                let fresh = vertices.len() as u32;
                vertices.push(*point);
                index_of.insert(*id, fresh);
                fresh
            };
            face[slot] = idx;
        }
        faces.push(face);
    }
    let face_count = faces.len();
    Ok((
        TriangleMesh::from_parts(vertices, faces),
        AttributeStore::new_for_mesh(face_count),
    ))
}

fn data_section(text: &str) -> Result<String, MeshError> {
    let upper = text.to_ascii_uppercase();
    let Some(start) = upper.find("DATA;") else {
        return Err(MeshError::InvalidFormat(
            "STEP file has no DATA section".to_string(),
        ));
    };
    let rest = &text[start + "DATA;".len()..];
    let rest_upper = rest.to_ascii_uppercase();
    let Some(end) = rest_upper.find("ENDSEC;") else {
        return Err(MeshError::InvalidFormat(
            "STEP DATA section is not closed".to_string(),
        ));
    };
    Ok(strip_comments(&rest[..end]))
}

fn strip_comments(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let bytes = text.as_bytes();
    let mut i = 0usize;
    while i < bytes.len() {
        if bytes[i] == b'/' && i + 1 < bytes.len() && bytes[i + 1] == b'*' {
            i += 2;
            while i + 1 < bytes.len() && !(bytes[i] == b'*' && bytes[i + 1] == b'/') {
                i += 1;
            }
            i = (i + 2).min(bytes.len());
            continue;
        }
        out.push(bytes[i] as char);
        i += 1;
    }
    out
}

fn split_entities(data: &str) -> Result<Vec<(u64, String, String)>, MeshError> {
    let bytes = data.as_bytes();
    let mut i = 0usize;
    let mut entities = Vec::new();
    while i < bytes.len() {
        if bytes[i] != b'#' {
            i += 1;
            continue;
        }
        i += 1;
        let id_start = i;
        while i < bytes.len() && bytes[i].is_ascii_digit() {
            i += 1;
        }
        if i == id_start {
            continue;
        }
        let id: u64 = data[id_start..i]
            .parse()
            .map_err(|_| MeshError::InvalidFormat("STEP entity id is too large".to_string()))?;
        while i < bytes.len() && bytes[i].is_ascii_whitespace() {
            i += 1;
        }
        if i >= bytes.len() || bytes[i] != b'=' {
            continue;
        }
        i += 1;
        while i < bytes.len() && bytes[i].is_ascii_whitespace() {
            i += 1;
        }
        let name_start = i;
        while i < bytes.len() && (bytes[i].is_ascii_alphanumeric() || bytes[i] == b'_') {
            i += 1;
        }
        if i == name_start {
            return Err(MeshError::InvalidFormat(
                "STEP entity is missing a type name".to_string(),
            ));
        }
        let name = data[name_start..i].to_ascii_uppercase();
        while i < bytes.len() && bytes[i].is_ascii_whitespace() {
            i += 1;
        }
        if i >= bytes.len() || bytes[i] != b'(' {
            return Err(MeshError::InvalidFormat(format!(
                "STEP entity {name} is missing a parameter list"
            )));
        }
        let body_start = i + 1;
        let mut depth = 1i32;
        let mut in_string = false;
        i += 1;
        while i < bytes.len() && depth > 0 {
            let c = bytes[i];
            if in_string {
                if c == b'\'' {
                    if i + 1 < bytes.len() && bytes[i + 1] == b'\'' {
                        i += 2;
                        continue;
                    }
                    in_string = false;
                }
                i += 1;
                continue;
            }
            match c {
                b'\'' => in_string = true,
                b'(' => depth += 1,
                b')' => depth -= 1,
                _ => {}
            }
            i += 1;
        }
        if depth != 0 {
            return Err(MeshError::InvalidFormat(
                "STEP entity parameter list is not closed".to_string(),
            ));
        }
        let body = data[body_start..i - 1].to_string();
        entities.push((id, name, body));
    }
    Ok(entities)
}

fn cartesian_point(body: &str) -> Result<Point3<f64>, MeshError> {
    let bytes = body.as_bytes();
    let mut last = None;
    let mut i = 0usize;
    while i < bytes.len() {
        if bytes[i] == b'(' {
            let start = i + 1;
            let mut depth = 1i32;
            i += 1;
            while i < bytes.len() && depth > 0 {
                match bytes[i] {
                    b'(' => depth += 1,
                    b')' => depth -= 1,
                    _ => {}
                }
                i += 1;
            }
            if depth == 0 {
                last = Some(&body[start..i - 1]);
            }
            continue;
        }
        i += 1;
    }
    let Some(list) = last else {
        return Err(MeshError::InvalidFormat(
            "STEP CARTESIAN_POINT has no coordinates".to_string(),
        ));
    };
    let nums = numbers(list)?;
    if nums.len() < 3 {
        return Err(MeshError::InvalidFormat(
            "STEP CARTESIAN_POINT needs three coordinates".to_string(),
        ));
    }
    Ok(Point3::new(nums[0], nums[1], nums[2]))
}

fn poly_loop(body: &str) -> Result<Vec<u64>, MeshError> {
    let mut ids = Vec::new();
    let bytes = body.as_bytes();
    let mut i = 0usize;
    while i < bytes.len() {
        if bytes[i] == b'#' {
            i += 1;
            let start = i;
            while i < bytes.len() && bytes[i].is_ascii_digit() {
                i += 1;
            }
            if i > start {
                ids.push(body[start..i].parse::<u64>().map_err(|_| {
                    MeshError::InvalidFormat("STEP point reference is too large".to_string())
                })?);
            }
            continue;
        }
        i += 1;
    }
    Ok(ids)
}

fn numbers(text: &str) -> Result<Vec<f64>, MeshError> {
    let mut out = Vec::new();
    let bytes = text.as_bytes();
    let mut i = 0usize;
    while i < bytes.len() {
        if bytes[i] == b'+' || bytes[i] == b'-' || bytes[i].is_ascii_digit() || bytes[i] == b'.' {
            let start = i;
            if bytes[i] == b'+' || bytes[i] == b'-' {
                i += 1;
            }
            while i < bytes.len() && (bytes[i].is_ascii_digit() || bytes[i] == b'.') {
                i += 1;
            }
            if i < bytes.len() && (bytes[i] == b'e' || bytes[i] == b'E') {
                i += 1;
                if i < bytes.len() && (bytes[i] == b'+' || bytes[i] == b'-') {
                    i += 1;
                }
                while i < bytes.len() && bytes[i].is_ascii_digit() {
                    i += 1;
                }
            }
            let value = text[start..i].parse::<f64>().map_err(|_| {
                MeshError::InvalidFormat("STEP coordinate is not a number".to_string())
            })?;
            if !value.is_finite() {
                return Err(MeshError::InvalidFormat(
                    "STEP coordinate is not finite".to_string(),
                ));
            }
            out.push(value);
            continue;
        }
        i += 1;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn triangular_poly_loop_loads() {
        let step = b"ISO-10303-21;HEADER;ENDSEC;DATA;\
#1=CARTESIAN_POINT('',(0.,0.,0.));\
#2=CARTESIAN_POINT('',(1.,0.,0.));\
#3=CARTESIAN_POINT('',(0.,1.,0.));\
#4=POLY_LOOP('',(#1,#2,#3));\
ENDSEC;END-ISO-10303-21;";
        let (mesh, _) = load_step_slice(step, &ResourceLimits::default()).expect("step");
        assert_eq!(mesh.faces.len(), 1);
        assert_eq!(mesh.vertices.len(), 3);
    }

    #[test]
    fn analytic_solid_is_refused() {
        let step = b"ISO-10303-21;HEADER;ENDSEC;DATA;\
#1=CYLINDRICAL_SURFACE('',#2,5.);\
ENDSEC;END-ISO-10303-21;";
        let err = load_step_slice(step, &ResourceLimits::default()).expect_err("analytic");
        assert!(err.to_string().contains("Analytic"));
    }

    #[test]
    fn non_triangle_poly_loop_is_refused() {
        let step = b"ISO-10303-21;HEADER;ENDSEC;DATA;\
#1=CARTESIAN_POINT('',(0.,0.,0.));\
#2=CARTESIAN_POINT('',(1.,0.,0.));\
#3=CARTESIAN_POINT('',(1.,1.,0.));\
#4=CARTESIAN_POINT('',(0.,1.,0.));\
#5=POLY_LOOP('',(#1,#2,#3,#4));\
ENDSEC;END-ISO-10303-21;";
        let err = load_step_slice(step, &ResourceLimits::default()).expect_err("quad");
        assert!(err.to_string().contains("not a triangle"));
    }
}
