//! SVG import for simple filled shapes, extruded into a printable solid.
//!
//! User units are millimeters. Each contour becomes a slab 1 mm thick.
//! Curves, transforms, and holes are refused. There is no SVG writer.

use mesh_attributes::AttributeStore;
use mesh_core::{MeshError, ResourceLimits, TriangleMesh};
use nalgebra::Point3;

const THICKNESS_MM: f64 = 1.0;
const CURVE_STEPS: usize = 32;
const MAX_DEPTH: usize = 32;

/// Load an SVG document from memory and extrude each simple contour.
pub fn load_svg_slice(
    bytes: &[u8],
    limits: &ResourceLimits,
) -> Result<(TriangleMesh, AttributeStore), MeshError> {
    if bytes.len() as u64 > limits.max_file_bytes {
        return Err(MeshError::FileTooLarge {
            size: bytes.len() as u64,
            limit: limits.max_file_bytes,
        });
    }
    let contours = parse_contours(bytes)?;
    if contours.is_empty() {
        return Err(MeshError::InvalidFormat(
            "SVG document has no closed contour to extrude".to_string(),
        ));
    }
    let mut vertices = Vec::new();
    let mut faces = Vec::new();
    for contour in contours {
        let (v, f) = extrude_contour(&contour, limits, vertices.len(), faces.len())?;
        vertices.extend(v);
        faces.extend(f);
    }
    let face_count = faces.len();
    Ok((
        TriangleMesh::from_parts(vertices, faces),
        AttributeStore::new_for_mesh(face_count),
    ))
}

fn parse_contours(bytes: &[u8]) -> Result<Vec<Vec<[f64; 2]>>, MeshError> {
    let mut reader = quick_xml::Reader::from_reader(bytes);
    reader.config_mut().trim_text(true);
    reader.config_mut().expand_empty_elements = true;
    let mut buf = Vec::new();
    let mut depth = 0usize;
    let mut saw_svg = false;
    let mut contours = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(quick_xml::events::Event::Start(event)) => {
                depth += 1;
                if depth > MAX_DEPTH {
                    return Err(MeshError::InvalidFormat(
                        "SVG nesting exceeds the safety cap".to_string(),
                    ));
                }
                let name = local_name(event.name().as_ref())?;
                if name == "svg" {
                    saw_svg = true;
                }
                if attr_raw(&event, b"transform")?.is_some() {
                    return Err(MeshError::UnsupportedFormat(
                        "SVG transform attributes are not applied".to_string(),
                    ));
                }
                if let Some(contour) = shape_contour(&name, &event)? {
                    contours.push(contour);
                }
            }
            Ok(quick_xml::events::Event::Empty(event)) => {
                let name = local_name(event.name().as_ref())?;
                if attr_raw(&event, b"transform")?.is_some() {
                    return Err(MeshError::UnsupportedFormat(
                        "SVG transform attributes are not applied".to_string(),
                    ));
                }
                if let Some(contour) = shape_contour(&name, &event)? {
                    contours.push(contour);
                }
            }
            Ok(quick_xml::events::Event::End(_)) => depth = depth.saturating_sub(1),
            Ok(quick_xml::events::Event::DocType(_)) => {
                return Err(MeshError::InvalidFormat(
                    "SVG documents must not contain a DOCTYPE".to_string(),
                ));
            }
            Ok(quick_xml::events::Event::GeneralRef(_)) => {
                return Err(MeshError::InvalidFormat(
                    "SVG documents must not contain entity references".to_string(),
                ));
            }
            Ok(
                quick_xml::events::Event::Text(_)
                | quick_xml::events::Event::CData(_)
                | quick_xml::events::Event::Comment(_)
                | quick_xml::events::Event::Decl(_)
                | quick_xml::events::Event::PI(_),
            ) => {}
            Ok(quick_xml::events::Event::Eof) => break,
            Err(err) => {
                return Err(MeshError::InvalidFormat(format!("SVG XML error: {err}")));
            }
        }
        buf.clear();
    }
    if !saw_svg {
        return Err(MeshError::InvalidFormat(
            "SVG root element missing".to_string(),
        ));
    }
    Ok(contours)
}

fn shape_contour(
    name: &str,
    event: &quick_xml::events::BytesStart<'_>,
) -> Result<Option<Vec<[f64; 2]>>, MeshError> {
    let contour = match name {
        "polygon" => Some(parse_points(&required_attr(event, b"points")?)?),
        "polyline" => {
            let points = parse_points(&required_attr(event, b"points")?)?;
            if !closed(&points) {
                return Err(MeshError::UnsupportedFormat(
                    "SVG polyline must already be closed".to_string(),
                ));
            }
            Some(points)
        }
        "rect" => Some(rect_points(event)?),
        "circle" => Some(ellipse_points(event, true)?),
        "ellipse" => Some(ellipse_points(event, false)?),
        "path" => Some(parse_path(&required_attr(event, b"d")?)?),
        _ => None,
    };
    contour.map(clean_ring).transpose()
}

fn rect_points(event: &quick_xml::events::BytesStart<'_>) -> Result<Vec<[f64; 2]>, MeshError> {
    let x = optional_f64(event, b"x")?.unwrap_or(0.0);
    let y = optional_f64(event, b"y")?.unwrap_or(0.0);
    let w = required_f64(event, b"width")?;
    let h = required_f64(event, b"height")?;
    if w <= 0.0 || h <= 0.0 {
        return Err(MeshError::InvalidFormat(
            "SVG rect width and height must be positive".to_string(),
        ));
    }
    Ok(vec![[x, y], [x + w, y], [x + w, y + h], [x, y + h]])
}

fn ellipse_points(
    event: &quick_xml::events::BytesStart<'_>,
    circle: bool,
) -> Result<Vec<[f64; 2]>, MeshError> {
    let cx = optional_f64(event, b"cx")?.unwrap_or(0.0);
    let cy = optional_f64(event, b"cy")?.unwrap_or(0.0);
    let (rx, ry) = if circle {
        let r = required_f64(event, b"r")?;
        (r, r)
    } else {
        (required_f64(event, b"rx")?, required_f64(event, b"ry")?)
    };
    if rx <= 0.0 || ry <= 0.0 {
        return Err(MeshError::InvalidFormat(
            "SVG circle or ellipse radii must be positive".to_string(),
        ));
    }
    let mut points = Vec::with_capacity(CURVE_STEPS);
    for i in 0..CURVE_STEPS {
        let t = (i as f64) * std::f64::consts::TAU / (CURVE_STEPS as f64);
        points.push([cx + rx * t.cos(), cy + ry * t.sin()]);
    }
    Ok(points)
}

fn parse_path(d: &str) -> Result<Vec<[f64; 2]>, MeshError> {
    let tokens = path_tokens(d)?;
    let mut i = 0usize;
    let mut points = Vec::new();
    let mut cursor = [0.0, 0.0];
    let mut start = [0.0, 0.0];
    let mut closed = false;
    let mut command = None::<char>;
    while i < tokens.len() {
        if let Token::Command(cmd) = tokens[i] {
            command = Some(cmd);
            i += 1;
        }
        let Some(cmd) = command else {
            return Err(MeshError::InvalidFormat(
                "SVG path is missing a command".to_string(),
            ));
        };
        match cmd {
            'Z' | 'z' => {
                if points.is_empty() {
                    return Err(MeshError::InvalidFormat(
                        "SVG path closes before any point".to_string(),
                    ));
                }
                closed = true;
                cursor = start;
                command = None;
            }
            'M' | 'm' | 'L' | 'l' | 'H' | 'h' | 'V' | 'v' => {
                let relative = cmd.is_ascii_lowercase();
                let next = match cmd.to_ascii_uppercase() {
                    'H' => {
                        let x = take_number(&tokens, &mut i)?;
                        let x = if relative { cursor[0] + x } else { x };
                        [x, cursor[1]]
                    }
                    'V' => {
                        let y = take_number(&tokens, &mut i)?;
                        let y = if relative { cursor[1] + y } else { y };
                        [cursor[0], y]
                    }
                    _ => {
                        let x = take_number(&tokens, &mut i)?;
                        let y = take_number(&tokens, &mut i)?;
                        if relative {
                            [cursor[0] + x, cursor[1] + y]
                        } else {
                            [x, y]
                        }
                    }
                };
                if cmd.eq_ignore_ascii_case(&'M') && !points.is_empty() {
                    return Err(MeshError::UnsupportedFormat(
                        "SVG paths with holes or more than one contour are not imported"
                            .to_string(),
                    ));
                }
                if points.is_empty() {
                    start = next;
                }
                points.push(next);
                cursor = next;
                if cmd.eq_ignore_ascii_case(&'M') {
                    command = Some(if relative { 'l' } else { 'L' });
                }
            }
            _ => {
                return Err(MeshError::UnsupportedFormat(
                    "SVG path curves are not imported; use straight M, L, H, V, and Z commands"
                        .to_string(),
                ));
            }
        }
    }
    if !closed {
        return Err(MeshError::UnsupportedFormat(
            "SVG path must be closed with Z".to_string(),
        ));
    }
    Ok(points)
}

#[derive(Clone, Copy)]
enum Token {
    Command(char),
    Number(f64),
}

fn path_tokens(d: &str) -> Result<Vec<Token>, MeshError> {
    let bytes = d.as_bytes();
    let mut i = 0usize;
    let mut tokens = Vec::new();
    while i < bytes.len() {
        let c = bytes[i] as char;
        if c.is_whitespace() || c == ',' {
            i += 1;
            continue;
        }
        if c.is_ascii_alphabetic() {
            tokens.push(Token::Command(c));
            i += 1;
            continue;
        }
        let start = i;
        if bytes[i] == b'+' || bytes[i] == b'-' {
            i += 1;
        }
        let mut saw_digit = false;
        while i < bytes.len() && bytes[i].is_ascii_digit() {
            saw_digit = true;
            i += 1;
        }
        if i < bytes.len() && bytes[i] == b'.' {
            i += 1;
            while i < bytes.len() && bytes[i].is_ascii_digit() {
                saw_digit = true;
                i += 1;
            }
        }
        if i < bytes.len() && (bytes[i] == b'e' || bytes[i] == b'E') {
            let exp = i;
            i += 1;
            if i < bytes.len() && (bytes[i] == b'+' || bytes[i] == b'-') {
                i += 1;
            }
            let exp_digits = i;
            while i < bytes.len() && bytes[i].is_ascii_digit() {
                i += 1;
            }
            if i == exp_digits {
                i = exp;
            }
        }
        if !saw_digit || i == start {
            return Err(MeshError::InvalidFormat(format!(
                "SVG path has an invalid number near '{}'",
                &d[start..]
            )));
        }
        let number = d[start..i]
            .parse::<f64>()
            .map_err(|_| MeshError::InvalidFormat("SVG path number is not finite".to_string()))?;
        if !number.is_finite() {
            return Err(MeshError::InvalidFormat(
                "SVG path number is not finite".to_string(),
            ));
        }
        tokens.push(Token::Number(number));
    }
    Ok(tokens)
}

fn take_number(tokens: &[Token], index: &mut usize) -> Result<f64, MeshError> {
    match tokens.get(*index) {
        Some(Token::Number(value)) => {
            *index += 1;
            Ok(*value)
        }
        _ => Err(MeshError::InvalidFormat(
            "SVG path is missing a coordinate".to_string(),
        )),
    }
}

fn parse_points(text: &str) -> Result<Vec<[f64; 2]>, MeshError> {
    let mut values = Vec::new();
    for part in text.split(|c: char| c.is_whitespace() || c == ',') {
        if part.is_empty() {
            continue;
        }
        let value = part
            .parse::<f64>()
            .map_err(|_| MeshError::InvalidFormat(format!("SVG point '{part}' is not a number")))?;
        if !value.is_finite() {
            return Err(MeshError::InvalidFormat(
                "SVG point is not finite".to_string(),
            ));
        }
        values.push(value);
    }
    if values.len() < 6 || !values.len().is_multiple_of(2) {
        return Err(MeshError::InvalidFormat(
            "SVG points must be pairs and form at least a triangle".to_string(),
        ));
    }
    Ok(values.chunks(2).map(|p| [p[0], p[1]]).collect())
}

fn clean_ring(mut points: Vec<[f64; 2]>) -> Result<Vec<[f64; 2]>, MeshError> {
    if closed(&points) {
        points.pop();
    }
    let mut unique = Vec::new();
    for point in points {
        if unique.last().is_none_or(|prev: &[f64; 2]| {
            (prev[0] - point[0]).abs() > 1e-9 || (prev[1] - point[1]).abs() > 1e-9
        }) {
            unique.push(point);
        }
    }
    if unique.len() >= 2 {
        let first = unique[0];
        let last = *unique.last().expect("len checked");
        if (first[0] - last[0]).abs() <= 1e-9 && (first[1] - last[1]).abs() <= 1e-9 {
            unique.pop();
        }
    }
    if unique.len() < 3 {
        return Err(MeshError::InvalidFormat(
            "SVG contour needs at least three distinct points".to_string(),
        ));
    }
    if signed_area(&unique).abs() <= 1e-12 {
        return Err(MeshError::InvalidFormat(
            "SVG contour has no area".to_string(),
        ));
    }
    if signed_area(&unique) < 0.0 {
        unique.reverse();
    }
    Ok(unique)
}

fn closed(points: &[[f64; 2]]) -> bool {
    if points.len() < 2 {
        return false;
    }
    let a = points[0];
    let b = points[points.len() - 1];
    (a[0] - b[0]).abs() <= 1e-9 && (a[1] - b[1]).abs() <= 1e-9
}

type ExtrudedPlate = (Vec<Point3<f64>>, Vec<[u32; 3]>);

fn extrude_contour(
    poly: &[[f64; 2]],
    limits: &ResourceLimits,
    vertex_base: usize,
    face_base: usize,
) -> Result<ExtrudedPlate, MeshError> {
    let top = ear_clip(poly)?;
    let n = poly.len();
    let added_vertices = n * 2;
    let added_faces = top.len() * 2 + n * 2;
    if vertex_base + added_vertices > limits.max_vertices {
        return Err(MeshError::TooManyVertices {
            count: vertex_base + added_vertices,
            limit: limits.max_vertices,
        });
    }
    if face_base + added_faces > limits.max_triangles {
        return Err(MeshError::TooManyTriangles {
            count: face_base + added_faces,
            limit: limits.max_triangles,
        });
    }
    let mut vertices = Vec::with_capacity(added_vertices);
    for p in poly {
        vertices.push(Point3::new(p[0], p[1], 0.0));
    }
    for p in poly {
        vertices.push(Point3::new(p[0], p[1], THICKNESS_MM));
    }
    let mut faces = Vec::with_capacity(added_faces);
    let n_u = n as u32;
    for tri in &top {
        faces.push([n_u + tri[0], n_u + tri[1], n_u + tri[2]]);
        faces.push([tri[0], tri[2], tri[1]]);
    }
    for i in 0..n {
        let j = ((i + 1) % n) as u32;
        let i = i as u32;
        faces.push([i, j, j + n_u]);
        faces.push([i, j + n_u, i + n_u]);
    }
    Ok((vertices, faces))
}

fn ear_clip(poly: &[[f64; 2]]) -> Result<Vec<[u32; 3]>, MeshError> {
    let mut left: Vec<u32> = (0..poly.len() as u32).collect();
    let mut tris = Vec::with_capacity(poly.len().saturating_sub(2));
    let mut guard = 0usize;
    while left.len() > 3 && guard < poly.len() * poly.len() {
        guard += 1;
        let m = left.len();
        let mut clipped = false;
        for i in 0..m {
            let prev = left[(i + m - 1) % m];
            let curr = left[i];
            let next = left[(i + 1) % m];
            if !is_ear(poly, &left, prev, curr, next) {
                continue;
            }
            tris.push([prev, curr, next]);
            left.remove(i);
            clipped = true;
            break;
        }
        if !clipped {
            return Err(MeshError::UnsupportedFormat(
                "SVG contour is self-intersecting or not a simple polygon".to_string(),
            ));
        }
    }
    if left.len() == 3 {
        tris.push([left[0], left[1], left[2]]);
    }
    Ok(tris)
}

fn is_ear(poly: &[[f64; 2]], left: &[u32], prev: u32, curr: u32, next: u32) -> bool {
    let a = poly[prev as usize];
    let b = poly[curr as usize];
    let c = poly[next as usize];
    if cross(a, b, c) <= 1e-12 {
        return false;
    }
    for id in left {
        if *id == prev || *id == curr || *id == next {
            continue;
        }
        if point_in_tri(poly[*id as usize], a, b, c) {
            return false;
        }
    }
    true
}

fn point_in_tri(p: [f64; 2], a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> bool {
    let c1 = cross(a, b, p);
    let c2 = cross(b, c, p);
    let c3 = cross(c, a, p);
    c1 >= -1e-12 && c2 >= -1e-12 && c3 >= -1e-12
}

fn cross(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

fn signed_area(poly: &[[f64; 2]]) -> f64 {
    let mut area = 0.0;
    for i in 0..poly.len() {
        let p = poly[i];
        let q = poly[(i + 1) % poly.len()];
        area += p[0] * q[1] - q[0] * p[1];
    }
    area * 0.5
}

fn local_name(raw: &[u8]) -> Result<String, MeshError> {
    let text = std::str::from_utf8(raw)
        .map_err(|_| MeshError::InvalidFormat("SVG tag is not UTF-8".to_string()))?;
    Ok(text.rsplit(':').next().unwrap_or(text).to_ascii_lowercase())
}

fn attr_raw(
    event: &quick_xml::events::BytesStart<'_>,
    key: &[u8],
) -> Result<Option<String>, MeshError> {
    for attr in event.attributes() {
        let attr =
            attr.map_err(|err| MeshError::InvalidFormat(format!("SVG attribute error: {err}")))?;
        if attr.key.as_ref() == key || attr.key.local_name().as_ref() == key {
            let value = attr
                .normalized_value(quick_xml::XmlVersion::Implicit1_0)
                .map_err(|_| MeshError::InvalidFormat("SVG attribute is not text".to_string()))?;
            let text = value.into_owned();
            if text.trim().is_empty() {
                return Ok(None);
            }
            return Ok(Some(text));
        }
    }
    Ok(None)
}

fn required_attr(
    event: &quick_xml::events::BytesStart<'_>,
    key: &[u8],
) -> Result<String, MeshError> {
    attr_raw(event, key)?.ok_or_else(|| {
        MeshError::InvalidFormat(format!(
            "SVG element is missing {}",
            String::from_utf8_lossy(key)
        ))
    })
}

fn optional_f64(
    event: &quick_xml::events::BytesStart<'_>,
    key: &[u8],
) -> Result<Option<f64>, MeshError> {
    attr_raw(event, key)?
        .map(|text| parse_f64(&text))
        .transpose()
}

fn required_f64(event: &quick_xml::events::BytesStart<'_>, key: &[u8]) -> Result<f64, MeshError> {
    parse_f64(&required_attr(event, key)?)
}

fn parse_f64(text: &str) -> Result<f64, MeshError> {
    let value = text
        .trim()
        .parse::<f64>()
        .map_err(|_| MeshError::InvalidFormat(format!("SVG value '{text}' is not a number")))?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(MeshError::InvalidFormat(
            "SVG value is not finite".to_string(),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rect_extrudes_to_unit_volume_plate() {
        let svg = br#"<svg xmlns="http://www.w3.org/2000/svg"><rect width="10" height="5"/></svg>"#;
        let (mesh, _) = load_svg_slice(svg, &ResourceLimits::default()).expect("svg");
        assert_eq!(mesh.faces.len(), 12);
        assert!((mesh.signed_volume() - 50.0).abs() < 1e-8);
    }

    #[test]
    fn curves_transforms_and_doctype_are_refused() {
        let limits = ResourceLimits::default();
        let curve = br#"<svg><path d="M0,0 C1,1 2,1 3,0 Z"/></svg>"#;
        let err = load_svg_slice(curve, &limits).expect_err("curve");
        assert!(err.to_string().contains("curves"));
        let transformed = br#"<svg><rect width="1" height="1" transform="translate(2)"/></svg>"#;
        assert!(load_svg_slice(transformed, &limits).is_err());
        let hostile = br#"<!DOCTYPE svg [<!ENTITY x "x">]><svg/>"#;
        assert!(load_svg_slice(hostile, &limits).is_err());
    }
}
