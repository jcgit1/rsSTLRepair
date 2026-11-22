//! AMF reader and writer. Only `unit="millimeter"` is accepted.
//!
//! `<!DOCTYPE>` is rejected. Nesting is capped so a hostile document cannot
//! recurse without bound. Volumes may carry `materialid`, stored as a categorical
//! face material id.

use mesh_attributes::AttributeStore;
use mesh_core::{MeshError, ResourceLimits, TriangleMesh};
use nalgebra::Point3;
use quick_xml::Reader;
use quick_xml::events::Event;
use std::io::Write;

const MAX_DEPTH: usize = 16;

/// Load an AMF document from memory.
pub fn load_amf_slice(
    bytes: &[u8],
    limits: &ResourceLimits,
) -> Result<(TriangleMesh, AttributeStore), MeshError> {
    if bytes.len() as u64 > limits.max_file_bytes {
        return Err(MeshError::FileTooLarge {
            size: bytes.len() as u64,
            limit: limits.max_file_bytes,
        });
    }
    let mut reader = Reader::from_reader(bytes);
    reader.config_mut().trim_text(true);
    reader.config_mut().expand_empty_elements = true;
    let mut buf = Vec::new();
    let mut depth = 0usize;
    let mut saw_amf = false;
    let mut unit_ok = false;
    let mut vertices = Vec::new();
    let mut faces = Vec::new();
    let mut materials = Vec::new();
    let mut stack: Vec<String> = Vec::new();
    let mut text = String::new();
    let mut coord = [None, None, None];
    let mut tri = [None, None, None];
    let mut volume_material: Option<u32> = None;

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(e)) => {
                depth += 1;
                if depth > MAX_DEPTH {
                    return Err(MeshError::InvalidFormat(
                        "AMF nesting exceeds the safety cap".to_string(),
                    ));
                }
                let name = local_name(e.name().as_ref())?;
                if name == "amf" {
                    saw_amf = true;
                    unit_ok = attr_value(&e, b"unit")?.as_deref() == Some("millimeter");
                }
                if name == "volume" {
                    volume_material = attr_value(&e, b"materialid")?
                        .map(|v| parse_u32(&v))
                        .transpose()?;
                }
                if name == "coordinates" {
                    coord = [None, None, None];
                }
                if name == "triangle" {
                    tri = [None, None, None];
                }
                stack.push(name);
                text.clear();
            }
            Ok(Event::End(e)) => {
                let name = local_name(e.name().as_ref())?;
                match name.as_str() {
                    "x" | "y" | "z" => {
                        let value = parse_f64(text.trim())?;
                        let slot = match name.as_str() {
                            "x" => 0,
                            "y" => 1,
                            _ => 2,
                        };
                        coord[slot] = Some(value);
                    }
                    "v1" | "v2" | "v3" => {
                        let value = parse_u32(text.trim())?;
                        let slot = match name.as_str() {
                            "v1" => 0,
                            "v2" => 1,
                            _ => 2,
                        };
                        tri[slot] = Some(value);
                    }
                    "vertex" => {
                        let [Some(x), Some(y), Some(z)] = coord else {
                            return Err(MeshError::InvalidFormat(
                                "AMF vertex is missing a coordinate".to_string(),
                            ));
                        };
                        if vertices.len() >= limits.max_vertices {
                            return Err(MeshError::TooManyVertices {
                                count: vertices.len() + 1,
                                limit: limits.max_vertices,
                            });
                        }
                        vertices.push(Point3::new(x, y, z));
                        coord = [None, None, None];
                    }
                    "triangle" => {
                        let [Some(a), Some(b), Some(c)] = tri else {
                            return Err(MeshError::InvalidFormat(
                                "AMF triangle is missing a vertex index".to_string(),
                            ));
                        };
                        if faces.len() >= limits.max_triangles {
                            return Err(MeshError::TooManyTriangles {
                                count: faces.len() + 1,
                                limit: limits.max_triangles,
                            });
                        }
                        if [a, b, c].iter().any(|i| *i as usize >= vertices.len()) {
                            return Err(MeshError::InvalidFormat(
                                "AMF triangle index is out of range".to_string(),
                            ));
                        }
                        faces.push([a, b, c]);
                        materials.push(volume_material);
                        tri = [None, None, None];
                    }
                    "volume" => volume_material = None,
                    _ => {}
                }
                stack.pop();
                text.clear();
                depth = depth.saturating_sub(1);
            }
            Ok(Event::Text(t)) => {
                text = t
                    .xml_content(quick_xml::XmlVersion::Implicit1_0)
                    .map_err(|_| {
                        MeshError::InvalidFormat("AMF text could not be decoded".to_string())
                    })?
                    .into_owned();
            }
            Ok(Event::CData(t)) => {
                text = String::from_utf8_lossy(&t).into_owned();
            }
            Ok(Event::Empty(_)) => {}
            Ok(Event::DocType(_)) => {
                return Err(MeshError::InvalidFormat(
                    "AMF documents must not contain a DOCTYPE".to_string(),
                ));
            }
            Ok(Event::Comment(_) | Event::Decl(_) | Event::PI(_)) => {}
            Ok(Event::GeneralRef(_)) => {
                return Err(MeshError::InvalidFormat(
                    "AMF documents must not contain entity references".to_string(),
                ));
            }
            Ok(Event::Eof) => break,
            Err(err) => {
                return Err(MeshError::InvalidFormat(format!("AMF XML error: {err}")));
            }
        }
        buf.clear();
    }
    if !saw_amf {
        return Err(MeshError::InvalidFormat(
            "AMF root element missing".to_string(),
        ));
    }
    if !unit_ok {
        return Err(MeshError::UnsupportedFormat(
            "AMF unit must be millimeter".to_string(),
        ));
    }
    let mut attrs = AttributeStore::new_for_mesh(faces.len());
    for (face, material) in attrs.faces.iter_mut().zip(materials) {
        face.material_id = material;
    }
    Ok((TriangleMesh::from_parts(vertices, faces), attrs))
}

/// Write an AMF document in millimeters.
pub fn write_amf<W: Write>(
    mut writer: W,
    mesh: &TriangleMesh,
    attributes: &AttributeStore,
) -> Result<(), MeshError> {
    writeln!(writer, r#"<?xml version="1.0" encoding="UTF-8"?>"#)?;
    writeln!(writer, r#"<amf unit="millimeter">"#)?;
    writeln!(writer, r#" <object id="1">"#)?;
    writeln!(writer, "  <mesh>")?;
    writeln!(writer, "   <vertices>")?;
    for v in &mesh.vertices {
        writeln!(writer, "    <vertex><coordinates>")?;
        writeln!(
            writer,
            "     <x>{:.17}</x><y>{:.17}</y><z>{:.17}</z>",
            v.x, v.y, v.z
        )?;
        writeln!(writer, "    </coordinates></vertex>")?;
    }
    writeln!(writer, "   </vertices>")?;
    let mut current: Option<Option<u32>> = None;
    let mut open = false;
    for (i, face) in mesh.faces.iter().enumerate() {
        let material = attributes.faces.get(i).and_then(|f| f.material_id);
        if current != Some(material) {
            if open {
                writeln!(writer, "   </volume>")?;
            }
            match material {
                Some(id) => writeln!(writer, r#"   <volume materialid="{id}">"#)?,
                None => writeln!(writer, "   <volume>")?,
            }
            current = Some(material);
            open = true;
        }
        writeln!(
            writer,
            "    <triangle><v1>{}</v1><v2>{}</v2><v3>{}</v3></triangle>",
            face[0], face[1], face[2]
        )?;
    }
    if open {
        writeln!(writer, "   </volume>")?;
    }
    writeln!(writer, "  </mesh>")?;
    writeln!(writer, " </object>")?;
    writeln!(writer, "</amf>")?;
    Ok(())
}

fn local_name(raw: &[u8]) -> Result<String, MeshError> {
    let text = std::str::from_utf8(raw)
        .map_err(|_| MeshError::InvalidFormat("AMF tag is not UTF-8".to_string()))?;
    Ok(text.rsplit(':').next().unwrap_or(text).to_ascii_lowercase())
}

fn attr_value(
    event: &quick_xml::events::BytesStart<'_>,
    key: &[u8],
) -> Result<Option<String>, MeshError> {
    for attr in event.attributes() {
        let attr =
            attr.map_err(|err| MeshError::InvalidFormat(format!("AMF attribute error: {err}")))?;
        if attr.key.as_ref() == key || attr.key.local_name().as_ref() == key {
            let value = attr
                .normalized_value(quick_xml::XmlVersion::Implicit1_0)
                .map_err(|_| MeshError::InvalidFormat("AMF attribute is not text".to_string()))?;
            return Ok(Some(value.into_owned()));
        }
    }
    Ok(None)
}

fn parse_f64(text: &str) -> Result<f64, MeshError> {
    text.parse::<f64>()
        .map_err(|_| MeshError::InvalidFormat(format!("AMF value '{text}' is not a number")))
}

fn parse_u32(text: &str) -> Result<u32, MeshError> {
    text.parse::<u32>()
        .map_err(|_| MeshError::InvalidFormat(format!("AMF value '{text}' is not an index")))
}
