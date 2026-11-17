//! Hardened, package-preserving 3MF reader and surgical XML writer.
//!
//! Treats `.3mf` files as Open Packaging Conventions (OPC) ZIP archives. Untouched
//! metadata, settings, thumbnails, relationships, and unmodified object XML files
//! are preserved byte-for-byte; only modified `<object><mesh>` elements are rewritten.

use mesh_attributes::{
    AttributeStore, FaceAttributes, PaintAssignment, ThreeMfTriangleMaterial, parse_srgb_hex,
};
use mesh_core::{MeshError, ResourceLimits, TriangleMesh};
use nalgebra::{Matrix4, Point3};
use quick_xml::events::{BytesEnd, BytesStart, Event};
use quick_xml::{Reader, Writer};
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::fs::File;
use std::io::{BufReader, BufWriter, Cursor, Read, Write};
use std::path::Path;
use zip::write::SimpleFileOptions;
use zip::{CompressionMethod, ZipArchive, ZipWriter};

const MAX_ARCHIVE_ENTRIES: usize = 4096;
const MAX_XML_DEPTH: usize = 128;

/// High-level classification of a `.3mf` package.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThreeMfKind {
    /// Standard 3MF geometry/materials package (`3D/3dmodel.model`).
    GeometryModel,
    /// Bambu Studio / OrcaSlicer / PrusaSlicer project with settings or split `3D/Objects/*.model`.
    SlicerProject,
}

/// Raw entry preserved from the original ZIP container.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ArchiveEntry {
    pub name: String,
    pub compression: CompressionMethod,
    pub data: Vec<u8>,
}

/// One `<object>` containing a `<mesh>` inside a 3MF `.model` file.
#[derive(Debug, Clone, PartialEq)]
pub struct ThreeMfMeshPart {
    pub archive_path: String,
    pub object_id: String,
    pub object_name: Option<String>,
    pub mesh: TriangleMesh,
    pub attributes: AttributeStore,
    /// 4x4 column-major affine transform for preview display only (never baked into XML vertices).
    pub display_transform: [f64; 16],
}

/// Repaired geometry/attributes supplied for one [`ThreeMfMeshPart`] on export.
pub struct RepairedThreeMfPart<'a> {
    pub archive_path: &'a str,
    pub object_id: &'a str,
    pub was_noop: bool,
    pub mesh: &'a TriangleMesh,
    pub attributes: &'a AttributeStore,
}

/// Complete 3MF package loaded with raw ZIP entries intact for loss-minimizing export.
#[derive(Debug, Clone, PartialEq)]
pub struct ThreeMfPackage {
    pub kind: ThreeMfKind,
    pub root_model_path: String,
    pub unit: String,
    pub raw_entries: Vec<ArchiveEntry>,
    pub parts: Vec<ThreeMfMeshPart>,
    pub filament_palette: Vec<[f32; 3]>,
    pub warnings: Vec<String>,
}

impl ThreeMfPackage {
    /// Load and validate a `.3mf` package from `path`, enforcing [`ResourceLimits`].
    pub fn load_path(path: &Path, limits: &ResourceLimits) -> Result<Self, MeshError> {
        let meta = std::fs::metadata(path)?;
        if meta.len() > limits.max_file_bytes {
            return Err(MeshError::FileTooLarge {
                size: meta.len(),
                limit: limits.max_file_bytes,
            });
        }
        let file = File::open(path)?;
        Self::load_reader(BufReader::new(file), limits)
    }

    /// Load and validate a `.3mf` package from an in-memory byte slice.
    pub fn load_slice(bytes: &[u8], limits: &ResourceLimits) -> Result<Self, MeshError> {
        if bytes.len() as u64 > limits.max_file_bytes {
            return Err(MeshError::FileTooLarge {
                size: bytes.len() as u64,
                limit: limits.max_file_bytes,
            });
        }
        Self::load_reader(Cursor::new(bytes), limits)
    }

    /// Load and validate a `.3mf` package from any seekable reader.
    pub fn load_reader<R: Read + std::io::Seek>(
        reader: R,
        limits: &ResourceLimits,
    ) -> Result<Self, MeshError> {
        let mut archive = ZipArchive::new(reader)
            .map_err(|e| MeshError::InvalidFormat(format!("invalid 3MF ZIP archive: {e}")))?;

        if archive.len() > MAX_ARCHIVE_ENTRIES {
            return Err(MeshError::InvalidFormat(format!(
                "3MF archive has {} entries, exceeding limit {MAX_ARCHIVE_ENTRIES}",
                archive.len()
            )));
        }

        let mut raw_entries = Vec::with_capacity(archive.len());
        let mut total_decompressed: u64 = 0;

        for idx in 0..archive.len() {
            let mut entry = archive.by_index(idx).map_err(|e| {
                MeshError::InvalidFormat(format!("failed reading 3MF ZIP entry {idx}: {e}"))
            })?;
            if entry.is_dir() {
                continue;
            }
            let raw_name = entry.name().map_err(|e| {
                MeshError::InvalidFormat(format!("invalid UTF-8 3MF entry name: {e}"))
            })?;
            let name = sanitize_archive_path(&raw_name)?;

            if name.to_ascii_lowercase().ends_with(".gcode") {
                return Err(MeshError::UnsupportedFormat(
                    "3MF package contains sliced G-code; export an unsliced project or model 3MF before repairing geometry."
                        .to_string(),
                ));
            }

            let remaining = limits
                .max_decompressed_bytes
                .saturating_sub(total_decompressed);
            let mut data = Vec::new();
            let take_cap = remaining.saturating_add(1);
            (&mut entry).take(take_cap).read_to_end(&mut data)?;
            if data.len() as u64 > remaining {
                return Err(MeshError::InvalidFormat(format!(
                    "3MF decompressed payload exceeds {} byte limit (ZIP bomb protection)",
                    limits.max_decompressed_bytes
                )));
            }
            total_decompressed = total_decompressed.saturating_add(data.len() as u64);

            raw_entries.push(ArchiveEntry {
                name,
                compression: entry.compression(),
                data,
            });
        }

        let entry_map: BTreeMap<String, usize> = raw_entries
            .iter()
            .enumerate()
            .map(|(i, e)| (e.name.clone(), i))
            .collect();

        let root_model_path = find_root_model_path(&entry_map, &raw_entries)?;
        let kind = classify_package(&raw_entries);
        let filament_palette = extract_filament_palette(&entry_map, &raw_entries);
        let default_extruders = extract_default_extruders(&entry_map, &raw_entries);

        let mut warnings = Vec::new();
        let mut total_verts = 0usize;
        let mut total_tris = 0usize;
        let mut parts = Vec::new();
        let mut model_unit = "millimeter".to_string();

        // Discover model files starting at root_model_path and following Production-extension p:path refs.
        let mut visited_models: BTreeSet<String> = BTreeSet::new();
        let mut model_queue: VecDeque<(String, BTreeMap<String, [f64; 16]>)> =
            VecDeque::from([(root_model_path.clone(), BTreeMap::new())]);

        // Also include any unreferenced 3D/Objects/*.model files deterministically if not reached via components.
        let mut all_object_models: Vec<String> = raw_entries
            .iter()
            .filter(|e| {
                let lower = e.name.to_ascii_lowercase();
                lower.starts_with("3d/objects/") && lower.ends_with(".model")
            })
            .map(|e| e.name.clone())
            .collect();
        all_object_models.sort();

        while let Some((model_path, inherited_xforms)) = model_queue.pop_front() {
            if !visited_models.insert(model_path.clone()) {
                continue;
            }
            let Some(&entry_idx) = entry_map.get(&model_path) else {
                return Err(MeshError::InvalidFormat(format!(
                    "referenced 3MF model '{model_path}' is missing from archive"
                )));
            };

            let parsed = parse_model_xml(
                &model_path,
                &raw_entries[entry_idx].data,
                &filament_palette,
                &default_extruders,
                &inherited_xforms,
                limits,
                &mut total_verts,
                &mut total_tris,
            )?;

            if model_path == root_model_path {
                model_unit = parsed.unit;
                if model_unit != "millimeter" {
                    warnings.push(format!(
                        "3MF model unit is '{}' (repair tolerances operate in native model units).",
                        model_unit
                    ));
                }
            }

            for (sub_path, sub_xforms) in parsed.referenced_submodels {
                if !visited_models.contains(&sub_path) {
                    model_queue.push_back((sub_path, sub_xforms));
                }
            }
            parts.extend(parsed.parts);
        }

        for extra_model in all_object_models {
            if visited_models.insert(extra_model.clone())
                && let Some(&entry_idx) = entry_map.get(&extra_model)
            {
                let parsed = parse_model_xml(
                    &extra_model,
                    &raw_entries[entry_idx].data,
                    &filament_palette,
                    &default_extruders,
                    &BTreeMap::new(),
                    limits,
                    &mut total_verts,
                    &mut total_tris,
                )?;
                parts.extend(parsed.parts);
            }
        }

        if parts.is_empty() {
            return Err(MeshError::InvalidFormat(
                "3MF package contains no triangle mesh objects".to_string(),
            ));
        }

        Ok(Self {
            kind,
            root_model_path,
            unit: model_unit,
            raw_entries,
            parts,
            filament_palette,
            warnings,
        })
    }

    /// Write a repaired 3MF package to `path`, copying unmodified entries verbatim and
    /// splicing only repaired `<object><mesh>` elements into their `.model` XML files.
    pub fn write_repaired_path(
        &self,
        path: &Path,
        repaired_parts: &[RepairedThreeMfPart<'_>],
    ) -> Result<(), MeshError> {
        let file = File::create(path)?;
        self.write_repaired_writer(BufWriter::new(file), repaired_parts)
    }

    /// Write a repaired 3MF package into `writer`.
    pub fn write_repaired_writer<W: Write + std::io::Seek>(
        &self,
        writer: W,
        repaired_parts: &[RepairedThreeMfPart<'_>],
    ) -> Result<(), MeshError> {
        let mut by_file: BTreeMap<&str, BTreeMap<&str, &RepairedThreeMfPart<'_>>> = BTreeMap::new();
        for part in repaired_parts {
            if !part.was_noop {
                by_file
                    .entry(part.archive_path)
                    .or_default()
                    .insert(part.object_id, part);
            }
        }

        let mut zip = ZipWriter::new(writer);
        for entry in &self.raw_entries {
            let method = match entry.compression {
                CompressionMethod::Stored => CompressionMethod::Stored,
                _ => CompressionMethod::Deflated,
            };
            let options = SimpleFileOptions::default().compression_method(method);
            zip.start_file(&entry.name, options).map_err(|e| {
                MeshError::InvalidFormat(format!("failed creating ZIP entry '{}': {e}", entry.name))
            })?;

            if let Some(modified_objs) = by_file.get(entry.name.as_str()) {
                let spliced = splice_model_xml(&entry.data, modified_objs)?;
                zip.write_all(&spliced)?;
            } else {
                zip.write_all(&entry.data)?;
            }
        }
        zip.finish()
            .map_err(|e| MeshError::InvalidFormat(format!("failed finalizing 3MF ZIP: {e}")))?;
        Ok(())
    }
}

fn sanitize_archive_path(raw: &str) -> Result<String, MeshError> {
    if raw.is_empty() || raw.contains('\0') {
        return Err(MeshError::InvalidFormat(
            "3MF archive contains empty or NUL-byte entry name".to_string(),
        ));
    }
    if raw.starts_with('/') || raw.starts_with('\\') {
        return Err(MeshError::InvalidFormat(format!(
            "unsafe absolute path in 3MF archive: '{raw}'"
        )));
    }
    if raw.as_bytes().get(1) == Some(&b':') {
        return Err(MeshError::InvalidFormat(format!(
            "unsafe drive-prefixed path in 3MF archive: '{raw}'"
        )));
    }
    let normalized = raw.replace('\\', "/");
    for seg in normalized.split('/') {
        if seg == ".." {
            return Err(MeshError::InvalidFormat(format!(
                "path traversal segment in 3MF archive: '{raw}'"
            )));
        }
    }
    Ok(normalized)
}

fn normalize_opc_ref(raw: &str) -> Result<String, MeshError> {
    let stripped = raw.trim().trim_start_matches('/');
    sanitize_archive_path(stripped)
}

fn find_root_model_path(
    entry_map: &BTreeMap<String, usize>,
    entries: &[ArchiveEntry],
) -> Result<String, MeshError> {
    if let Some(&idx) = entry_map.get("_rels/.rels") {
        let mut reader = Reader::from_reader(Cursor::new(&entries[idx].data));
        let mut buf = Vec::new();
        loop {
            match reader.read_event_into(&mut buf) {
                Ok(Event::Empty(e)) | Ok(Event::Start(e))
                    if local_name(e.name().as_ref()) == b"Relationship" =>
                {
                    let mut rel_type = None;
                    let mut target = None;
                    for attr in e.attributes().flatten() {
                        match local_name(attr.key.as_ref()) {
                            b"Type" => {
                                rel_type = attr
                                    .normalized_value(quick_xml::XmlVersion::Implicit1_0)
                                    .ok()
                                    .map(|s| s.into_owned());
                            }
                            b"Target" => {
                                target = attr
                                    .normalized_value(quick_xml::XmlVersion::Implicit1_0)
                                    .ok()
                                    .map(|s| s.into_owned());
                            }
                            _ => {}
                        }
                    }
                    if let (Some(ty), Some(tgt)) = (rel_type, target)
                        && ty.ends_with("/3dmodel")
                    {
                        return normalize_opc_ref(&tgt);
                    }
                }
                Ok(Event::DocType(_)) => {
                    return Err(MeshError::InvalidFormat(
                        "unexpected DOCTYPE declaration in 3MF relationships".to_string(),
                    ));
                }
                Ok(Event::Eof) => break,
                Err(e) => {
                    return Err(MeshError::InvalidFormat(format!(
                        "invalid _rels/.rels XML: {e}"
                    )));
                }
                _ => {}
            }
            buf.clear();
        }
    }

    if entry_map.contains_key("3D/3dmodel.model") {
        Ok("3D/3dmodel.model".to_string())
    } else {
        Err(MeshError::InvalidFormat(
            "3MF archive missing _rels/.rels 3dmodel relationship and 3D/3dmodel.model".to_string(),
        ))
    }
}

fn classify_package(entries: &[ArchiveEntry]) -> ThreeMfKind {
    let is_slicer = entries.iter().any(|e| {
        let lower = e.name.to_ascii_lowercase();
        lower == "metadata/project_settings.config"
            || lower == "metadata/model_settings.config"
            || lower == "metadata/slic3r_pe_model.config"
            || (lower.starts_with("3d/objects/") && lower.ends_with(".model"))
    });
    if is_slicer {
        ThreeMfKind::SlicerProject
    } else {
        ThreeMfKind::GeometryModel
    }
}

fn extract_filament_palette(
    entry_map: &BTreeMap<String, usize>,
    entries: &[ArchiveEntry],
) -> Vec<[f32; 3]> {
    let Some(&idx) = entry_map.get("Metadata/project_settings.config") else {
        return Vec::new();
    };
    let Ok(val) = serde_json::from_slice::<serde_json::Value>(&entries[idx].data) else {
        return Vec::new();
    };
    val.get("filament_colour")
        .and_then(|v| v.as_array())
        .map(|arr| {
            arr.iter()
                .filter_map(|item| item.as_str().and_then(parse_srgb_hex))
                .collect()
        })
        .unwrap_or_default()
}

fn extract_default_extruders(
    entry_map: &BTreeMap<String, usize>,
    entries: &[ArchiveEntry],
) -> BTreeMap<String, u32> {
    let mut map = BTreeMap::new();
    for path in [
        "Metadata/model_settings.config",
        "Metadata/Slic3r_PE_model.config",
    ] {
        let Some(&idx) = entry_map.get(path) else {
            continue;
        };
        let mut reader = Reader::from_reader(Cursor::new(&entries[idx].data));
        let mut buf = Vec::new();
        let mut current_obj_id: Option<String> = None;
        let mut in_part = false;
        loop {
            match reader.read_event_into(&mut buf) {
                Ok(Event::Start(e)) => match local_name(e.name().as_ref()) {
                    b"object" => {
                        current_obj_id = attr_str(&e, b"id");
                        in_part = false;
                    }
                    b"part" => in_part = true,
                    b"metadata" if !in_part => {
                        if let Some(oid) = &current_obj_id
                            && attr_str(&e, b"key").as_deref() == Some("extruder")
                            && let Some(val) = attr_str(&e, b"value")
                            && let Ok(ext) = val.parse::<u32>()
                        {
                            map.entry(oid.clone()).or_insert(ext);
                        }
                    }
                    _ => {}
                },
                Ok(Event::Empty(e)) => {
                    if !in_part
                        && local_name(e.name().as_ref()) == b"metadata"
                        && let Some(oid) = &current_obj_id
                        && attr_str(&e, b"key").as_deref() == Some("extruder")
                        && let Some(val) = attr_str(&e, b"value")
                        && let Ok(ext) = val.parse::<u32>()
                    {
                        map.entry(oid.clone()).or_insert(ext);
                    }
                }
                Ok(Event::End(e)) => match local_name(e.name().as_ref()) {
                    b"object" => current_obj_id = None,
                    b"part" => in_part = false,
                    _ => {}
                },
                Ok(Event::Eof) | Err(_) => break,
                _ => {}
            }
            buf.clear();
        }
    }
    map
}

struct ParsedModelXml {
    unit: String,
    parts: Vec<ThreeMfMeshPart>,
    referenced_submodels: Vec<(String, BTreeMap<String, [f64; 16]>)>,
}

#[allow(clippy::too_many_arguments)]
fn parse_model_xml(
    archive_path: &str,
    xml_bytes: &[u8],
    filament_palette: &[[f32; 3]],
    default_extruders: &BTreeMap<String, u32>,
    inherited_xforms: &BTreeMap<String, [f64; 16]>,
    limits: &ResourceLimits,
    total_verts: &mut usize,
    total_tris: &mut usize,
) -> Result<ParsedModelXml, MeshError> {
    let mut reader = Reader::from_reader(Cursor::new(xml_bytes));
    let mut buf = Vec::new();
    let mut depth = 0usize;

    let mut unit = "millimeter".to_string();
    let mut colors: BTreeMap<String, [f32; 3]> = BTreeMap::new();
    let mut active_resource_id: Option<String> = None;
    let mut active_resource_idx: usize = 0;

    struct RawObj {
        id: String,
        name: Option<String>,
        obj_pid: Option<String>,
        obj_pindex: Option<String>,
        vertices: Vec<Point3<f64>>,
        faces: Vec<[u32; 3]>,
        face_attrs: Vec<FaceAttributes>,
        has_mesh: bool,
        components: Vec<(Option<String>, String, [f64; 16])>,
    }

    let mut objects: Vec<RawObj> = Vec::new();
    let mut cur_obj: Option<RawObj> = None;
    let mut in_mesh = false;
    let mut in_vertices = false;
    let mut in_triangles = false;
    let mut build_xforms: BTreeMap<String, [f64; 16]> = BTreeMap::new();

    loop {
        let ev = reader.read_event_into(&mut buf).map_err(|e| {
            MeshError::InvalidFormat(format!("invalid XML in '{archive_path}': {e}"))
        })?;
        match ev {
            Event::DocType(_) => {
                return Err(MeshError::InvalidFormat(format!(
                    "DOCTYPE declaration rejected in '{archive_path}' (XML hardening)"
                )));
            }
            Event::Start(e) => {
                depth += 1;
                if depth > MAX_XML_DEPTH {
                    return Err(MeshError::InvalidFormat(format!(
                        "XML nesting depth exceeds {MAX_XML_DEPTH} in '{archive_path}'"
                    )));
                }
                match local_name(e.name().as_ref()) {
                    b"model" => {
                        if let Some(u) = attr_str(&e, b"unit") {
                            unit = u;
                        }
                    }
                    b"basematerials" | b"colorgroup" => {
                        active_resource_id = attr_str(&e, b"id");
                        active_resource_idx = 0;
                    }
                    b"base" | b"color" => {
                        if let Some(pid) = &active_resource_id {
                            let hex =
                                attr_str(&e, b"displaycolor").or_else(|| attr_str(&e, b"color"));
                            if let Some(h) = hex
                                && let Some(rgb) = parse_srgb_hex(&h)
                            {
                                colors.insert(format!("{pid}:{active_resource_idx}"), rgb);
                            }
                            active_resource_idx += 1;
                        }
                    }
                    b"object" => {
                        let id = attr_str(&e, b"id").ok_or_else(|| {
                            MeshError::InvalidFormat(format!(
                                "<object> missing id attribute in '{archive_path}'"
                            ))
                        })?;
                        cur_obj = Some(RawObj {
                            id,
                            name: attr_str(&e, b"name"),
                            obj_pid: attr_str(&e, b"pid"),
                            obj_pindex: attr_str(&e, b"pindex"),
                            vertices: Vec::new(),
                            faces: Vec::new(),
                            face_attrs: Vec::new(),
                            has_mesh: false,
                            components: Vec::new(),
                        });
                    }
                    b"mesh" if cur_obj.is_some() => {
                        if let Some(obj) = &mut cur_obj {
                            obj.has_mesh = true;
                        }
                        in_mesh = true;
                    }
                    b"vertices" if in_mesh => in_vertices = true,
                    b"triangles" if in_mesh => in_triangles = true,
                    b"component" if cur_obj.is_some() => {
                        if let Some(obj) = &mut cur_obj
                            && let Some(oid) = attr_str(&e, b"objectid")
                        {
                            let p_path = attr_str(&e, b"path")
                                .map(|p| normalize_opc_ref(&p))
                                .transpose()?;
                            let xf = attr_str(&e, b"transform")
                                .and_then(|s| parse_3mf_transform(&s))
                                .unwrap_or_else(identity_mat4);
                            obj.components.push((p_path, oid, xf));
                        }
                    }
                    b"item" => {
                        if let Some(oid) = attr_str(&e, b"objectid") {
                            let xf = attr_str(&e, b"transform")
                                .and_then(|s| parse_3mf_transform(&s))
                                .unwrap_or_else(identity_mat4);
                            build_xforms.entry(oid).or_insert(xf);
                        }
                    }
                    _ => {}
                }
            }
            Event::Empty(e) => match local_name(e.name().as_ref()) {
                b"base" | b"color" => {
                    if let Some(pid) = &active_resource_id {
                        let hex = attr_str(&e, b"displaycolor").or_else(|| attr_str(&e, b"color"));
                        if let Some(h) = hex
                            && let Some(rgb) = parse_srgb_hex(&h)
                        {
                            colors.insert(format!("{pid}:{active_resource_idx}"), rgb);
                        }
                        active_resource_idx += 1;
                    }
                }
                b"vertex" if in_vertices => {
                    if let Some(obj) = &mut cur_obj {
                        *total_verts += 1;
                        if *total_verts > limits.max_vertices {
                            return Err(MeshError::TooManyVertices {
                                count: *total_verts,
                                limit: limits.max_vertices,
                            });
                        }
                        let x = parse_xml_f64(&e, b"x", archive_path)?;
                        let y = parse_xml_f64(&e, b"y", archive_path)?;
                        let z = parse_xml_f64(&e, b"z", archive_path)?;
                        obj.vertices.push(Point3::new(x, y, z));
                    }
                }
                b"triangle" if in_triangles => {
                    if let Some(obj) = &mut cur_obj {
                        *total_tris += 1;
                        if *total_tris > limits.max_triangles {
                            return Err(MeshError::TooManyTriangles {
                                count: *total_tris,
                                limit: limits.max_triangles,
                            });
                        }
                        let v1 = parse_xml_u32(&e, b"v1", archive_path)?;
                        let v2 = parse_xml_u32(&e, b"v2", archive_path)?;
                        let v3 = parse_xml_u32(&e, b"v3", archive_path)?;
                        obj.faces.push([v1, v2, v3]);
                        obj.face_attrs.push(extract_triangle_attrs(&e)?);
                    }
                }
                b"component" if cur_obj.is_some() => {
                    if let Some(obj) = &mut cur_obj
                        && let Some(oid) = attr_str(&e, b"objectid")
                    {
                        let p_path = attr_str(&e, b"path")
                            .map(|p| normalize_opc_ref(&p))
                            .transpose()?;
                        let xf = attr_str(&e, b"transform")
                            .and_then(|s| parse_3mf_transform(&s))
                            .unwrap_or_else(identity_mat4);
                        obj.components.push((p_path, oid, xf));
                    }
                }
                b"item" => {
                    if let Some(oid) = attr_str(&e, b"objectid") {
                        let xf = attr_str(&e, b"transform")
                            .and_then(|s| parse_3mf_transform(&s))
                            .unwrap_or_else(identity_mat4);
                        build_xforms.entry(oid).or_insert(xf);
                    }
                }
                _ => {}
            },
            Event::End(e) => {
                depth = depth.saturating_sub(1);
                match local_name(e.name().as_ref()) {
                    b"basematerials" | b"colorgroup" => active_resource_id = None,
                    b"vertices" => in_vertices = false,
                    b"triangles" => in_triangles = false,
                    b"mesh" => {
                        in_mesh = false;
                        in_vertices = false;
                        in_triangles = false;
                    }
                    b"object" => {
                        if let Some(obj) = cur_obj.take() {
                            objects.push(obj);
                        }
                    }
                    _ => {}
                }
            }
            Event::Eof => break,
            _ => {}
        }
        buf.clear();
    }

    let mut referenced_submodels: Vec<(String, BTreeMap<String, [f64; 16]>)> = Vec::new();
    let mut submodel_idx: BTreeMap<String, usize> = BTreeMap::new();
    let mut local_xforms = build_xforms.clone();
    for (oid, xf) in inherited_xforms {
        local_xforms.entry(oid.clone()).or_insert(*xf);
    }

    // Propagate parent component transforms to external sub-models and local component objects.
    for obj in &objects {
        let parent_xf = local_xforms
            .get(&obj.id)
            .copied()
            .unwrap_or_else(identity_mat4);
        for (p_path, child_id, comp_xf) in &obj.components {
            let combined = mul_mat4(&parent_xf, comp_xf);
            if let Some(sub) = p_path {
                let idx = *submodel_idx.entry(sub.clone()).or_insert_with(|| {
                    let i = referenced_submodels.len();
                    referenced_submodels.push((sub.clone(), BTreeMap::new()));
                    i
                });
                referenced_submodels[idx]
                    .1
                    .entry(child_id.clone())
                    .or_insert(combined);
            } else {
                local_xforms.entry(child_id.clone()).or_insert(combined);
            }
        }
    }

    let mut parts = Vec::new();
    for obj in objects {
        if !obj.has_mesh {
            continue;
        }
        let face_count = obj.faces.len();
        let mut attrs = AttributeStore::new_for_mesh(face_count);
        attrs.filament_palette = filament_palette.to_vec();
        attrs.default_extruder = default_extruders.get(&obj.id).copied();
        attrs.threemf_colors = colors.clone();
        if let Some(name) = &obj.name {
            let gid = attrs.intern_group(name);
            for fa in &mut attrs.faces {
                fa.group_id = Some(gid);
            }
        }
        for (dst, src) in attrs.faces.iter_mut().zip(obj.face_attrs) {
            let gid = dst.group_id;
            *dst = src;
            dst.group_id = gid;
            if dst.threemf_material.is_none()
                && let (Some(pid), Some(pidx)) = (&obj.obj_pid, &obj.obj_pindex)
                && let Some(&rgb) = colors.get(&format!("{pid}:{pidx}"))
            {
                attrs
                    .threemf_colors
                    .entry(format!("__obj_default:{}", obj.id))
                    .or_insert(rgb);
            }
        }

        let display_transform = local_xforms
            .get(&obj.id)
            .copied()
            .unwrap_or_else(identity_mat4);

        parts.push(ThreeMfMeshPart {
            archive_path: archive_path.to_string(),
            object_id: obj.id,
            object_name: obj.name,
            mesh: TriangleMesh::from_parts(obj.vertices, obj.faces),
            attributes: attrs,
            display_transform,
        });
    }

    Ok(ParsedModelXml {
        unit,
        parts,
        referenced_submodels,
    })
}

fn extract_triangle_attrs(e: &BytesStart<'_>) -> Result<FaceAttributes, MeshError> {
    let mut paint: Option<PaintAssignment> = None;
    let mut paint_supports: Option<PaintAssignment> = None;
    let mut paint_seam: Option<PaintAssignment> = None;
    let mut paint_fuzzy_skin: Option<PaintAssignment> = None;
    let mut pid: Option<String> = None;
    let mut p1: Option<String> = None;
    let mut p2: Option<String> = None;
    let mut p3: Option<String> = None;

    for attr in e.attributes().flatten() {
        let full_key = std::str::from_utf8(attr.key.as_ref()).unwrap_or("");
        let val = attr
            .normalized_value(quick_xml::XmlVersion::Implicit1_0)
            .map_err(|err| MeshError::InvalidFormat(format!("invalid XML attribute: {err}")))?
            .into_owned();
        match full_key {
            "paint_color" => paint = Some(PaintAssignment::from_key_hex("paint_color", val)),
            k if k.ends_with(":mmu_segmentation") || k == "mmu_segmentation" => {
                if paint.is_none() {
                    paint = Some(PaintAssignment::from_key_hex(k, val));
                }
            }
            "paint_supports" => {
                paint_supports = Some(PaintAssignment::from_key_hex("paint_supports", val));
            }
            "paint_seam" => {
                paint_seam = Some(PaintAssignment::from_key_hex("paint_seam", val));
            }
            "paint_fuzzy_skin" => {
                paint_fuzzy_skin = Some(PaintAssignment::from_key_hex("paint_fuzzy_skin", val));
            }
            "pid" => pid = Some(val),
            "p1" => p1 = Some(val),
            "p2" => p2 = Some(val),
            "p3" => p3 = Some(val),
            _ => {}
        }
    }

    let threemf_material = match (pid, p1) {
        (Some(pid), Some(p1)) => Some(ThreeMfTriangleMaterial { pid, p1, p2, p3 }),
        _ => None,
    };

    Ok(FaceAttributes {
        material_id: None,
        group_id: None,
        paint,
        paint_supports,
        paint_seam,
        paint_fuzzy_skin,
        threemf_material,
    })
}

fn splice_model_xml(
    original_xml: &[u8],
    modified_objs: &BTreeMap<&str, &RepairedThreeMfPart<'_>>,
) -> Result<Vec<u8>, MeshError> {
    let mut reader = Reader::from_reader(Cursor::new(original_xml));
    let mut writer = Writer::new(Vec::with_capacity(original_xml.len()));
    let mut buf = Vec::new();

    let mut current_obj_id: Option<String> = None;
    let mut skipping_mesh_depth: usize = 0;

    loop {
        let ev = reader
            .read_event_into(&mut buf)
            .map_err(|e| MeshError::InvalidFormat(format!("XML splice read error: {e}")))?;
        match ev {
            Event::Eof => break,
            Event::Start(ref e) if skipping_mesh_depth > 0 => {
                let _ = e;
                skipping_mesh_depth += 1;
            }
            Event::Empty(_) | Event::Text(_) | Event::Comment(_) if skipping_mesh_depth > 0 => {}
            Event::End(_) if skipping_mesh_depth > 0 => {
                skipping_mesh_depth -= 1;
            }
            Event::Start(ref e) if local_name(e.name().as_ref()) == b"object" => {
                current_obj_id = attr_str(e, b"id");
                writer
                    .write_event(Event::Start(e.borrow()))
                    .map_err(xml_write_err)?;
            }
            Event::Start(ref e)
                if local_name(e.name().as_ref()) == b"mesh"
                    && current_obj_id
                        .as_deref()
                        .is_some_and(|id| modified_objs.contains_key(id)) =>
            {
                let oid = current_obj_id.as_deref().expect("checked");
                let part = modified_objs[oid];
                write_mesh_element(&mut writer, part.mesh, part.attributes)?;
                skipping_mesh_depth = 1;
            }
            Event::End(ref e) if local_name(e.name().as_ref()) == b"object" => {
                current_obj_id = None;
                writer
                    .write_event(Event::End(e.borrow()))
                    .map_err(xml_write_err)?;
            }
            other => {
                writer.write_event(other).map_err(xml_write_err)?;
            }
        }
        buf.clear();
    }

    Ok(writer.into_inner())
}

fn write_mesh_element(
    writer: &mut Writer<Vec<u8>>,
    mesh: &TriangleMesh,
    attrs: &AttributeStore,
) -> Result<(), MeshError> {
    writer
        .write_event(Event::Start(BytesStart::new("mesh")))
        .map_err(xml_write_err)?;

    writer
        .write_event(Event::Start(BytesStart::new("vertices")))
        .map_err(xml_write_err)?;
    for v in &mesh.vertices {
        if !v.x.is_finite() || !v.y.is_finite() || !v.z.is_finite() {
            return Err(MeshError::InvalidFormat(
                "cannot export 3MF vertex with non-finite coordinates".to_string(),
            ));
        }
        let mut elem = BytesStart::new("vertex");
        let xs = format_f64_clean(v.x);
        let ys = format_f64_clean(v.y);
        let zs = format_f64_clean(v.z);
        elem.push_attribute(("x", xs.as_str()));
        elem.push_attribute(("y", ys.as_str()));
        elem.push_attribute(("z", zs.as_str()));
        writer
            .write_event(Event::Empty(elem))
            .map_err(xml_write_err)?;
    }
    writer
        .write_event(Event::End(BytesEnd::new("vertices")))
        .map_err(xml_write_err)?;

    writer
        .write_event(Event::Start(BytesStart::new("triangles")))
        .map_err(xml_write_err)?;
    let n_verts = mesh.vertices.len();
    for (fi, &[a, b, c]) in mesh.faces.iter().enumerate() {
        if a as usize >= n_verts || b as usize >= n_verts || c as usize >= n_verts {
            return Err(MeshError::InvalidFormat(format!(
                "face {fi} references out-of-range vertex index"
            )));
        }
        let mut elem = BytesStart::new("triangle");
        let s1 = a.to_string();
        let s2 = b.to_string();
        let s3 = c.to_string();
        elem.push_attribute(("v1", s1.as_str()));
        elem.push_attribute(("v2", s2.as_str()));
        elem.push_attribute(("v3", s3.as_str()));

        if let Some(fa) = attrs.faces.get(fi) {
            if let Some(tm) = &fa.threemf_material {
                elem.push_attribute(("pid", tm.pid.as_str()));
                elem.push_attribute(("p1", tm.p1.as_str()));
                if let Some(p2) = &tm.p2 {
                    elem.push_attribute(("p2", p2.as_str()));
                }
                if let Some(p3) = &tm.p3 {
                    elem.push_attribute(("p3", p3.as_str()));
                }
            }
            if let Some(p) = &fa.paint {
                elem.push_attribute((p.attr_name.as_str(), p.raw_attribute.as_str()));
            }
            if let Some(p) = &fa.paint_supports {
                elem.push_attribute((p.attr_name.as_str(), p.raw_attribute.as_str()));
            }
            if let Some(p) = &fa.paint_seam {
                elem.push_attribute((p.attr_name.as_str(), p.raw_attribute.as_str()));
            }
            if let Some(p) = &fa.paint_fuzzy_skin {
                elem.push_attribute((p.attr_name.as_str(), p.raw_attribute.as_str()));
            }
        }

        writer
            .write_event(Event::Empty(elem))
            .map_err(xml_write_err)?;
    }
    writer
        .write_event(Event::End(BytesEnd::new("triangles")))
        .map_err(xml_write_err)?;

    writer
        .write_event(Event::End(BytesEnd::new("mesh")))
        .map_err(xml_write_err)?;
    Ok(())
}

fn format_f64_clean(v: f64) -> String {
    if v == 0.0 {
        "0".to_string()
    } else {
        format!("{v}")
    }
}

fn xml_write_err(err: impl std::fmt::Display) -> MeshError {
    MeshError::InvalidFormat(format!("XML splice write error: {err}"))
}

fn local_name(raw: &[u8]) -> &[u8] {
    match raw.iter().rposition(|&b| b == b':') {
        Some(pos) => &raw[pos + 1..],
        None => raw,
    }
}

fn attr_str(e: &BytesStart<'_>, target_local: &[u8]) -> Option<String> {
    e.attributes().flatten().find_map(|attr| {
        (local_name(attr.key.as_ref()) == target_local)
            .then(|| {
                attr.normalized_value(quick_xml::XmlVersion::Implicit1_0)
                    .ok()
                    .map(|s| s.into_owned())
            })
            .flatten()
    })
}

fn parse_xml_f64(e: &BytesStart<'_>, key: &[u8], file: &str) -> Result<f64, MeshError> {
    let s = attr_str(e, key).ok_or_else(|| {
        MeshError::InvalidFormat(format!(
            "missing coordinate '{}' in '{file}'",
            String::from_utf8_lossy(key)
        ))
    })?;
    s.trim()
        .parse::<f64>()
        .map_err(|_| MeshError::InvalidFormat(format!("invalid float '{s}' in '{file}'")))
}

fn parse_xml_u32(e: &BytesStart<'_>, key: &[u8], file: &str) -> Result<u32, MeshError> {
    let s = attr_str(e, key).ok_or_else(|| {
        MeshError::InvalidFormat(format!(
            "missing triangle index '{}' in '{file}'",
            String::from_utf8_lossy(key)
        ))
    })?;
    s.trim()
        .parse::<u32>()
        .map_err(|_| MeshError::InvalidFormat(format!("invalid integer '{s}' in '{file}'")))
}

fn identity_mat4() -> [f64; 16] {
    [
        1.0, 0.0, 0.0, 0.0, //
        0.0, 1.0, 0.0, 0.0, //
        0.0, 0.0, 1.0, 0.0, //
        0.0, 0.0, 0.0, 1.0,
    ]
}

fn parse_3mf_transform(s: &str) -> Option<[f64; 16]> {
    let vals: Vec<f64> = s
        .split_whitespace()
        .filter_map(|t| t.parse::<f64>().ok())
        .collect();
    if vals.len() != 12 {
        return None;
    }
    // 3MF 3x4 affine row-major rows (m00 m01 m02, m10 m11 m12, m20 m21 m22, m30 m31 m32)
    // stored column-major for Matrix4 multiplication.
    let m = Matrix4::new(
        vals[0], vals[3], vals[6], vals[9], //
        vals[1], vals[4], vals[7], vals[10], //
        vals[2], vals[5], vals[8], vals[11], //
        0.0, 0.0, 0.0, 1.0,
    );
    let mut out = [0.0; 16];
    out.copy_from_slice(m.as_slice());
    Some(out)
}

fn mul_mat4(a: &[f64; 16], b: &[f64; 16]) -> [f64; 16] {
    let ma = Matrix4::from_column_slice(a);
    let mb = Matrix4::from_column_slice(b);
    let res = ma * mb;
    let mut out = [0.0; 16];
    out.copy_from_slice(res.as_slice());
    out
}

/// Apply a column-major 4x4 display transform to a point (used by preview viewers only).
#[must_use]
pub fn transform_point(m: &[f64; 16], p: Point3<f64>) -> Point3<f64> {
    let mat = Matrix4::from_column_slice(m);
    mat.transform_point(&p)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_attributes::FaceOrigin;

    fn build_sample_bambu_3mf(hole_in_obj1: bool) -> Vec<u8> {
        let rels = br#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Target="/3D/3dmodel.model" Id="rel-1" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel" />
</Relationships>"#;

        let root_model = br#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06">
  <metadata name="Application">BambuStudio-01.09.03.50</metadata>
  <resources>
    <object id="1" type="model">
      <components>
        <component p:path="/3D/Objects/object_1.model" objectid="1" transform="1 0 0 0 1 0 0 0 1 0 0 0" />
      </components>
    </object>
    <object id="2" type="model">
      <components>
        <component p:path="/3D/Objects/object_2.model" objectid="2" transform="1 0 0 0 1 0 0 0 1 150 0 0" />
      </components>
    </object>
  </resources>
  <build>
    <item objectid="1" transform="1 0 0 0 1 0 0 0 1 0 0 0" />
    <item objectid="2" transform="1 0 0 0 1 0 0 0 1 0 0 0" />
  </build>
</model>"#;

        // Object 1: 100x100x100 block with optional 2x2 hole in top face and paint_color codes
        let mut obj1_tris = String::from(
            r#"<triangle v1="0" v2="2" v3="1" paint_color="4" />
        <triangle v1="0" v2="3" v3="2" paint_color="8" />
        <triangle v1="0" v2="1" v3="5" paint_color="0C" />
        <triangle v1="0" v2="5" v3="4" paint_color="0C" />
        <triangle v1="2" v2="3" v3="7" paint_color="1C" />
        <triangle v1="2" v2="7" v3="6" paint_color="1C" />
        <triangle v1="0" v2="4" v3="7" paint_color="4" />
        <triangle v1="0" v2="7" v3="3" paint_color="4" />
        <triangle v1="1" v2="2" v3="6" paint_color="4" />
        <triangle v1="1" v2="6" v3="5" paint_color="4" />
        <triangle v1="4" v2="5" v3="9" paint_color="0C" />
        <triangle v1="4" v2="9" v3="8" paint_color="0C" />
        <triangle v1="5" v2="6" v3="10" paint_color="0C" />
        <triangle v1="5" v2="10" v3="9" paint_color="0C" />
        <triangle v1="6" v2="7" v3="11" paint_color="0C" />
        <triangle v1="6" v2="11" v3="10" paint_color="0C" />
        <triangle v1="7" v2="4" v3="8" paint_color="0C" />
        <triangle v1="7" v2="8" v3="11" paint_color="0C" />"#,
        );
        if !hole_in_obj1 {
            obj1_tris.push_str(
                r#"
        <triangle v1="8" v2="9" v3="10" paint_color="0C" />
        <triangle v1="8" v2="10" v3="11" paint_color="0C" />"#,
            );
        }

        let obj1_xml = format!(
            r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
  <resources>
    <object id="1" name="PaintedBlock" type="model">
      <mesh>
        <vertices>
          <vertex x="0" y="0" z="0" /><vertex x="100" y="0" z="0" />
          <vertex x="100" y="100" z="0" /><vertex x="0" y="100" z="0" />
          <vertex x="0" y="0" z="100" /><vertex x="100" y="0" z="100" />
          <vertex x="100" y="100" z="100" /><vertex x="0" y="100" z="100" />
          <vertex x="49" y="49" z="100" /><vertex x="51" y="49" z="100" />
          <vertex x="51" y="51" z="100" /><vertex x="49" y="51" z="100" />
        </vertices>
        <triangles>
          {obj1_tris}
        </triangles>
      </mesh>
    </object>
  </resources>
</model>"#
        );

        let obj2_xml = br#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
  <resources>
    <object id="2" name="CleanCube" type="model">
      <mesh>
        <vertices>
          <vertex x="0" y="0" z="0" /><vertex x="10" y="0" z="0" />
          <vertex x="10" y="10" z="0" /><vertex x="0" y="10" z="0" />
          <vertex x="0" y="0" z="10" /><vertex x="10" y="0" z="10" />
          <vertex x="10" y="10" z="10" /><vertex x="0" y="10" z="10" />
        </vertices>
        <triangles>
          <triangle v1="0" v2="2" v3="1" paint_color="8" />
          <triangle v1="0" v2="3" v3="2" paint_color="8" />
          <triangle v1="4" v2="5" v3="6" paint_color="8" />
          <triangle v1="4" v2="6" v3="7" paint_color="8" />
          <triangle v1="0" v2="1" v3="5" paint_color="8" />
          <triangle v1="0" v2="5" v3="4" paint_color="8" />
          <triangle v1="2" v2="3" v3="7" paint_color="8" />
          <triangle v1="2" v2="7" v3="6" paint_color="8" />
          <triangle v1="0" v2="4" v3="7" paint_color="8" />
          <triangle v1="0" v2="7" v3="3" paint_color="8" />
          <triangle v1="1" v2="2" v3="6" paint_color="8" />
          <triangle v1="1" v2="6" v3="5" paint_color="8" />
        </triangles>
      </mesh>
    </object>
  </resources>
</model>"#;

        let project_cfg = br##"{"filament_colour":["#8080FF","#000000","#FFFFFF","#808080"]}"##;
        let model_cfg = br#"<?xml version="1.0" encoding="UTF-8"?>
<config>
  <object id="1"><metadata key="extruder" value="1" /></object>
  <object id="2"><metadata key="extruder" value="2" /></object>
</config>"#;

        let mut buf = Cursor::new(Vec::new());
        {
            let mut zw = ZipWriter::new(&mut buf);
            let opts = SimpleFileOptions::default().compression_method(CompressionMethod::Deflated);
            for (name, bytes) in [
                ("_rels/.rels", rels.as_slice()),
                ("3D/3dmodel.model", root_model.as_slice()),
                ("3D/Objects/object_1.model", obj1_xml.as_bytes()),
                ("3D/Objects/object_2.model", obj2_xml.as_slice()),
                ("Metadata/project_settings.config", project_cfg.as_slice()),
                ("Metadata/model_settings.config", model_cfg.as_slice()),
                ("Metadata/top_1.png", b"\x89PNG\r\n\x1a\nfake"),
            ] {
                zw.start_file(name, opts).expect("start");
                zw.write_all(bytes).expect("write");
            }
            zw.finish().expect("finish");
        }
        buf.into_inner()
    }

    #[test]
    fn loads_bambu_slicer_project_and_preserves_unmodified_entries() {
        let bytes = build_sample_bambu_3mf(true);
        let pkg = ThreeMfPackage::load_slice(&bytes, &ResourceLimits::default()).expect("load 3mf");
        assert_eq!(pkg.kind, ThreeMfKind::SlicerProject);
        assert_eq!(pkg.parts.len(), 2);
        assert_eq!(pkg.filament_palette.len(), 4);
        assert_eq!(pkg.parts[0].object_name.as_deref(), Some("PaintedBlock"));
        assert_eq!(pkg.parts[0].attributes.default_extruder, Some(1));
        assert_eq!(
            pkg.parts[0].attributes.faces[2]
                .paint
                .as_ref()
                .and_then(|p| p.decoded_leaf_state),
            Some(3)
        );

        // Simulate repairing only object 1 (closing its hole) while leaving object 2 as no-op.
        let mut repaired_mesh = pkg.parts[0].mesh.clone();
        repaired_mesh.faces.push([8, 9, 10]);
        repaired_mesh.faces.push([8, 10, 11]);
        let mut origins: Vec<FaceOrigin> = (0..pkg.parts[0].mesh.faces.len())
            .map(|i| FaceOrigin::Identity(mesh_core::FaceId(i as u32)))
            .collect();
        origins.push(FaceOrigin::Synthesized {
            boundary_sources: vec![
                mesh_core::FaceId(10),
                mesh_core::FaceId(12),
                mesh_core::FaceId(14),
                mesh_core::FaceId(16),
            ],
        });
        origins.push(FaceOrigin::Synthesized {
            boundary_sources: vec![
                mesh_core::FaceId(10),
                mesh_core::FaceId(12),
                mesh_core::FaceId(14),
                mesh_core::FaceId(16),
            ],
        });
        let (repaired_attrs, summary) = pkg.parts[0].attributes.transfer(&origins);
        assert_eq!(summary.uncertain_count, 0);

        let mut out_zip = Cursor::new(Vec::new());
        pkg.write_repaired_writer(
            &mut out_zip,
            &[
                RepairedThreeMfPart {
                    archive_path: &pkg.parts[0].archive_path,
                    object_id: &pkg.parts[0].object_id,
                    was_noop: false,
                    mesh: &repaired_mesh,
                    attributes: &repaired_attrs,
                },
                RepairedThreeMfPart {
                    archive_path: &pkg.parts[1].archive_path,
                    object_id: &pkg.parts[1].object_id,
                    was_noop: true,
                    mesh: &pkg.parts[1].mesh,
                    attributes: &pkg.parts[1].attributes,
                },
            ],
        )
        .expect("write repaired 3mf");

        let reloaded =
            ThreeMfPackage::load_slice(&out_zip.into_inner(), &ResourceLimits::default())
                .expect("reload 3mf");
        assert_eq!(reloaded.parts[0].mesh.faces.len(), 20);
        // Unchanged triangles keep exact paint_color strings, new cap triangles inherited "0C".
        assert_eq!(
            reloaded.parts[0].attributes.faces[0]
                .paint
                .as_ref()
                .map(|p| p.raw_attribute.as_str()),
            Some("4")
        );
        assert_eq!(
            reloaded.parts[0].attributes.faces[18]
                .paint
                .as_ref()
                .map(|p| p.raw_attribute.as_str()),
            Some("0C")
        );

        // Verify unmodified entries (object_2.model, Metadata/*) remained byte-for-byte identical.
        for name in [
            "3D/3dmodel.model",
            "3D/Objects/object_2.model",
            "Metadata/project_settings.config",
            "Metadata/model_settings.config",
            "Metadata/top_1.png",
        ] {
            let orig = pkg.raw_entries.iter().find(|e| e.name == name).unwrap();
            let next = reloaded
                .raw_entries
                .iter()
                .find(|e| e.name == name)
                .unwrap();
            assert_eq!(
                orig.data, next.data,
                "entry {name} must stay byte-identical"
            );
        }
    }

    #[test]
    fn enforces_decompression_bomb_and_doctype_hardening() {
        let bytes = build_sample_bambu_3mf(false);
        let tight = ResourceLimits {
            max_decompressed_bytes: 128,
            ..Default::default()
        };
        let err = ThreeMfPackage::load_slice(&bytes, &tight).expect_err("must reject zip bomb");
        assert!(matches!(err, MeshError::InvalidFormat(_)));

        let mut dbuf = Cursor::new(Vec::new());
        {
            let mut zw = ZipWriter::new(&mut dbuf);
            let opts = SimpleFileOptions::default();
            zw.start_file("3D/3dmodel.model", opts).expect("start");
            zw.write_all(b"<!DOCTYPE model [<!ENTITY x 'y'>]><model/>")
                .expect("write");
            zw.finish().expect("finish");
        }
        let derr = ThreeMfPackage::load_slice(&dbuf.into_inner(), &ResourceLimits::default())
            .expect_err("must reject DOCTYPE");
        assert!(matches!(derr, MeshError::InvalidFormat(_)));
    }
}
