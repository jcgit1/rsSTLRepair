//! Headless-testable state and worker orchestration for the desktop application.

use crate::camera::OrbitCamera;
use mesh_analysis::{DiagnosticReport, analyze};
use mesh_attributes::{AttributeStore, AttributeTransferSummary};
use mesh_core::{MeshError, RepairProfile, ResourceLimits, TriangleMesh};
use mesh_formats::{
    FormatKind, RepairedThreeMfPart, ThreeMfPackage, combine_threemf_preview, load_model,
    save_model,
};
use mesh_repair::{RepairReport, RepairResult, repair_mesh, unsupported_features_list};
use mesh_validation::{
    RoundtripRequest, ValidationResult, verify_roundtrip, verify_threemf_roundtrip,
};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, TryRecvError};

#[derive(Debug)]
pub enum WorkerOutput {
    Single(Result<Box<RepairResult>, MeshError>),
    ThreeMf(Result<Box<ThreeMfRepairBundle>, MeshError>),
}

#[derive(Debug)]
pub struct ThreeMfRepairBundle {
    pub part_results: Vec<RepairResult>,
    pub combined_mesh: TriangleMesh,
    pub combined_attrs: AttributeStore,
    pub summary_result: RepairResult,
}

/// Application state decoupled from `egui`/`eframe` so workflow logic can be unit tested.
pub struct AppViewModel {
    pub limits: ResourceLimits,
    pub source_path: Option<PathBuf>,
    pub source_format: Option<FormatKind>,
    pub threemf_pkg: Option<ThreeMfPackage>,
    pub threemf_part_reports: Vec<DiagnosticReport>,
    pub threemf_part_results: Vec<RepairResult>,
    pub selected_part: Option<usize>,
    pub before_mesh: Option<TriangleMesh>,
    pub before_attrs: Option<AttributeStore>,
    pub before_report: Option<DiagnosticReport>,
    pub after_mesh: Option<TriangleMesh>,
    pub after_attrs: Option<AttributeStore>,
    pub repair_result: Option<RepairResult>,
    pub validation: Option<ValidationResult>,
    pub profile: RepairProfile,
    pub dimensions_locked: bool,
    pub show_defects: bool,
    pub show_wireframe: bool,
    pub show_grid: bool,
    pub camera: OrbitCamera,
    pub banner_error: Option<String>,
    pub status_line: String,
    pub before_revision: u64,
    pub after_revision: u64,
    pub repair_in_flight: bool,
    cancel_flag: Option<Arc<AtomicBool>>,
    worker_rx: Option<Receiver<WorkerOutput>>,
}

impl Default for AppViewModel {
    fn default() -> Self {
        Self::new()
    }
}

impl AppViewModel {
    #[must_use]
    pub fn new() -> Self {
        Self {
            limits: ResourceLimits::default(),
            source_path: None,
            source_format: None,
            threemf_pkg: None,
            threemf_part_reports: Vec::new(),
            threemf_part_results: Vec::new(),
            selected_part: None,
            before_mesh: None,
            before_attrs: None,
            before_report: None,
            after_mesh: None,
            after_attrs: None,
            repair_result: None,
            validation: None,
            profile: RepairProfile::Conservative,
            dimensions_locked: true,
            show_defects: true,
            show_wireframe: false,
            show_grid: true,
            camera: OrbitCamera::default(),
            banner_error: None,
            status_line: "Drop a .3mf, .stl, or .obj file, or click Open.".to_string(),
            before_revision: 0,
            after_revision: 0,
            repair_in_flight: false,
            cancel_flag: None,
            worker_rx: None,
        }
    }

    /// Load a mesh or 3MF package from disk. Refuses sliced `.gcode.3mf` packages.
    pub fn open_path(&mut self, path: &Path) {
        self.banner_error = None;

        match load_model(path, &self.limits) {
            Ok(loaded) => {
                let report = analyze(&loaded.mesh);
                if let Some(bbox) = report.bounding_box {
                    self.camera.fit_to_bbox(&bbox);
                }
                self.source_path = Some(path.to_path_buf());
                self.source_format = Some(loaded.source_format);
                self.threemf_part_reports = loaded
                    .threemf
                    .as_ref()
                    .map(|pkg| pkg.parts.iter().map(|p| analyze(&p.mesh)).collect())
                    .unwrap_or_default();
                self.threemf_pkg = loaded.threemf;
                self.threemf_part_results.clear();
                self.selected_part = None;
                self.before_mesh = Some(loaded.mesh);
                self.before_attrs = Some(loaded.attributes);
                self.before_report = Some(report);
                self.after_mesh = None;
                self.after_attrs = None;
                self.repair_result = None;
                self.validation = None;
                self.before_revision += 1;
                self.after_revision += 1;
                self.status_line = match &self.threemf_pkg {
                    Some(pkg) => format!(
                        "Loaded 3MF ({:?}, {} object(s)) from {}",
                        pkg.kind,
                        pkg.parts.len(),
                        path.display()
                    ),
                    None => format!("Loaded {}", path.display()),
                };
                if !loaded.warnings.is_empty() {
                    self.banner_error = Some(loaded.warnings.join(" "));
                }
            }
            Err(err) => {
                self.banner_error = Some(format!("Failed to load {}: {err}", path.display()));
                self.status_line = "Load failed.".to_string();
            }
        }
    }

    /// Re-run non-mutating topology diagnostics on the loaded input mesh (and any 3MF parts).
    pub fn analyze_current(&mut self) {
        if let Some(pkg) = &self.threemf_pkg {
            self.threemf_part_reports = pkg.parts.iter().map(|p| analyze(&p.mesh)).collect();
        }
        let Some(mesh) = &self.before_mesh else {
            return;
        };
        let report = analyze(mesh);
        let status = if report.is_watertight {
            "Analysis: watertight solid."
        } else if report.is_valid_open_surface {
            "Analysis: valid open surface."
        } else {
            "Analysis: mesh defects detected."
        };
        self.before_report = Some(report);
        self.status_line = status.to_string();
    }

    /// Spawn a background repair job using the active [`RepairProfile`].
    pub fn start_repair(&mut self) {
        if self.repair_in_flight {
            return;
        }
        let profile = self.profile;
        let cancel = Arc::new(AtomicBool::new(false));
        let cancel_worker = Arc::clone(&cancel);
        let (tx, rx) = mpsc::channel();

        if let Some(pkg) = self.threemf_pkg.clone() {
            self.banner_error = None;
            self.repair_in_flight = true;
            self.cancel_flag = Some(cancel);
            self.worker_rx = Some(rx);
            self.status_line = format!(
                "Running {} repair on {} 3MF object(s)...",
                profile.label(),
                pkg.parts.len()
            );

            std::thread::spawn(move || {
                let res = repair_threemf_bundle(&pkg, profile, &cancel_worker).map(Box::new);
                let _ = tx.send(WorkerOutput::ThreeMf(res));
            });
            return;
        }

        let (Some(mesh), Some(attrs)) = (self.before_mesh.clone(), self.before_attrs.clone())
        else {
            return;
        };

        self.banner_error = None;
        self.repair_in_flight = true;
        self.cancel_flag = Some(cancel);
        self.worker_rx = Some(rx);
        self.status_line = format!("Running {} repair...", profile.label());

        std::thread::spawn(move || {
            let result = repair_mesh(&mesh, &attrs, profile, Some(&cancel_worker)).map(Box::new);
            let _ = tx.send(WorkerOutput::Single(result));
        });
    }

    /// Request cancellation of an in-flight repair job.
    pub fn cancel_repair(&mut self) {
        if let Some(flag) = &self.cancel_flag {
            flag.store(true, Ordering::Relaxed);
            self.status_line = "Cancelling repair...".to_string();
        }
    }

    /// Poll background worker completion; returns `true` if UI state changed.
    pub fn poll_worker(&mut self) -> bool {
        let Some(rx) = &self.worker_rx else {
            return false;
        };
        match rx.try_recv() {
            Ok(WorkerOutput::Single(outcome)) => {
                self.repair_in_flight = false;
                self.cancel_flag = None;
                self.worker_rx = None;
                match outcome {
                    Ok(res) => {
                        let res = *res;
                        self.after_mesh = Some(res.mesh.clone());
                        self.after_attrs = Some(res.attributes.clone());
                        self.status_line = if res.report.was_noop {
                            "Clean mesh: left unmodified (no-op).".to_string()
                        } else if res.report.after.is_watertight {
                            format!(
                                "{} repair finished: watertight.",
                                res.report.profile.label()
                            )
                        } else {
                            format!(
                                "{} repair finished with remaining open/unresolved items.",
                                res.report.profile.label()
                            )
                        };
                        self.repair_result = Some(res);
                        self.validation = None;
                        self.after_revision += 1;
                    }
                    Err(MeshError::Cancelled) => {
                        self.status_line = "Repair cancelled.".to_string();
                    }
                    Err(err) => {
                        self.banner_error = Some(format!("Repair error: {err}"));
                        self.status_line = "Repair failed.".to_string();
                    }
                }
                true
            }
            Ok(WorkerOutput::ThreeMf(outcome)) => {
                self.repair_in_flight = false;
                self.cancel_flag = None;
                self.worker_rx = None;
                match outcome {
                    Ok(bundle) => {
                        let bundle = *bundle;
                        self.threemf_part_results = bundle.part_results;
                        self.after_mesh = Some(bundle.combined_mesh);
                        self.after_attrs = Some(bundle.combined_attrs);
                        let res = bundle.summary_result;
                        self.status_line = if res.report.was_noop {
                            "3MF project clean: all objects preserved unchanged.".to_string()
                        } else if res.report.after.is_watertight {
                            format!(
                                "{} 3MF repair finished: all objects watertight.",
                                res.report.profile.label()
                            )
                        } else {
                            format!(
                                "{} 3MF repair finished with remaining open/unresolved items.",
                                res.report.profile.label()
                            )
                        };
                        self.repair_result = Some(res);
                        self.validation = None;
                        self.after_revision += 1;
                    }
                    Err(MeshError::Cancelled) => {
                        self.status_line = "Repair cancelled.".to_string();
                    }
                    Err(err) => {
                        self.banner_error = Some(format!("3MF repair error: {err}"));
                        self.status_line = "Repair failed.".to_string();
                    }
                }
                true
            }
            Err(TryRecvError::Empty) => false,
            Err(TryRecvError::Disconnected) => {
                self.repair_in_flight = false;
                self.cancel_flag = None;
                self.worker_rx = None;
                self.banner_error = Some("Repair worker exited unexpectedly.".to_string());
                true
            }
        }
    }

    /// Return the mesh/attrs/report currently selected for display in Before viewport.
    #[must_use]
    pub fn active_before_view(
        &self,
    ) -> (
        Option<&TriangleMesh>,
        Option<&AttributeStore>,
        Option<&DiagnosticReport>,
    ) {
        if let (Some(idx), Some(pkg)) = (self.selected_part, &self.threemf_pkg)
            && let Some(part) = pkg.parts.get(idx)
        {
            return (
                Some(&part.mesh),
                Some(&part.attributes),
                self.threemf_part_reports.get(idx),
            );
        }
        (
            self.before_mesh.as_ref(),
            self.before_attrs.as_ref(),
            self.before_report.as_ref(),
        )
    }

    /// Return the mesh/attrs/report currently selected for display in After viewport.
    #[must_use]
    pub fn active_after_view(
        &self,
    ) -> (
        Option<&TriangleMesh>,
        Option<&AttributeStore>,
        Option<&DiagnosticReport>,
    ) {
        if let Some(idx) = self.selected_part
            && let Some(res) = self.threemf_part_results.get(idx)
        {
            return (
                Some(&res.mesh),
                Some(&res.attributes),
                Some(&res.report.after),
            );
        }
        (
            self.after_mesh.as_ref(),
            self.after_attrs.as_ref(),
            self.repair_result.as_ref().map(|r| &r.report.after),
        )
    }

    /// Default sibling export path `<stem>_repaired.<ext>` that never matches the source path.
    #[must_use]
    pub fn suggested_export_path(&self) -> Option<PathBuf> {
        let src = self.source_path.as_ref()?;
        let parent = src.parent().unwrap_or_else(|| Path::new("."));
        let stem = src.file_stem().and_then(|s| s.to_str()).unwrap_or("model");
        let ext = src.extension().and_then(|s| s.to_str()).unwrap_or("stl");
        Some(parent.join(format!("{stem}_repaired.{ext}")))
    }

    /// Validate roundtrip export and write the repaired mesh/3MF package to `target`.
    ///
    /// Refuses to overwrite the original source path or strip a 3MF package to STL/OBJ.
    pub fn export_to_path(&mut self, target: &Path) -> Result<ValidationResult, String> {
        self.banner_error = None;
        if let Some(src) = &self.source_path
            && paths_match(src, target)
        {
            let msg = "Refusing to overwrite the original input file. Choose a new export path."
                .to_string();
            self.banner_error = Some(msg.clone());
            return Err(msg);
        }

        if let Some(pkg) = &self.threemf_pkg {
            let ext = target
                .extension()
                .and_then(|s| s.to_str())
                .map(str::to_ascii_lowercase)
                .unwrap_or_default();
            if ext != "3mf" {
                let msg = "Refusing to export a 3MF package to STL/OBJ (would strip painting and metadata). Choose a .3mf path.".to_string();
                self.banner_error = Some(msg.clone());
                return Err(msg);
            }
            if self.threemf_part_results.len() != pkg.parts.len() {
                return Err("No repaired 3MF package available to export.".to_string());
            }

            let repaired_refs: Vec<RepairedThreeMfPart<'_>> = pkg
                .parts
                .iter()
                .zip(self.threemf_part_results.iter())
                .map(|(orig, res)| RepairedThreeMfPart {
                    archive_path: &orig.archive_path,
                    object_id: &orig.object_id,
                    was_noop: res.report.was_noop,
                    mesh: &res.mesh,
                    attributes: &res.attributes,
                })
                .collect();
            let expected_reports: Vec<DiagnosticReport> = self
                .threemf_part_results
                .iter()
                .map(|r| r.report.after.clone())
                .collect();

            let validation = verify_threemf_roundtrip(
                pkg,
                &repaired_refs,
                &expected_reports,
                self.profile,
                &self.limits,
            )
            .map_err(|e| {
                let msg = format!("3MF validation failed before export: {e}");
                self.banner_error = Some(msg.clone());
                msg
            })?;

            if !validation.passed {
                let msg = format!(
                    "Export blocked by 3MF reimport validation: {}",
                    validation.errors.join("; ")
                );
                self.banner_error = Some(msg.clone());
                return Err(msg);
            }

            pkg.write_repaired_path(target, &repaired_refs)
                .map_err(|e| {
                    let msg = format!("Failed to write {}: {e}", target.display());
                    self.banner_error = Some(msg.clone());
                    msg
                })?;

            self.validation = Some(validation.clone());
            self.status_line = format!("Exported validated 3MF package to {}", target.display());
            return Ok(validation);
        }

        let (Some(orig), Some(repaired), Some(attrs), Some(res)) = (
            &self.before_mesh,
            &self.after_mesh,
            &self.after_attrs,
            &self.repair_result,
        ) else {
            return Err("No repaired mesh available to export.".to_string());
        };

        let ext = target.extension().and_then(|s| s.to_str()).unwrap_or("stl");

        let validation = verify_roundtrip(RoundtripRequest {
            ext,
            original_mesh: orig,
            repaired_mesh: repaired,
            repaired_attrs: attrs,
            expected_report: &res.report.after,
            profile: res.report.profile,
            was_noop: res.report.was_noop,
            limits: &self.limits,
        })
        .map_err(|e| {
            let msg = format!("Validation failed before export: {e}");
            self.banner_error = Some(msg.clone());
            msg
        })?;

        if !validation.passed {
            let msg = format!(
                "Export blocked by reimport validation: {}",
                validation.errors.join("; ")
            );
            self.banner_error = Some(msg.clone());
            return Err(msg);
        }

        save_model(target, repaired, attrs).map_err(|e| {
            let msg = format!("Failed to write {}: {e}", target.display());
            self.banner_error = Some(msg.clone());
            msg
        })?;

        self.validation = Some(validation.clone());
        self.status_line = format!("Exported validated copy to {}", target.display());
        Ok(validation)
    }
}

fn repair_threemf_bundle(
    pkg: &ThreeMfPackage,
    profile: RepairProfile,
    cancel: &AtomicBool,
) -> Result<ThreeMfRepairBundle, MeshError> {
    let mut part_results = Vec::with_capacity(pkg.parts.len());
    for part in &pkg.parts {
        let res = repair_mesh(&part.mesh, &part.attributes, profile, Some(cancel))?;
        part_results.push(res);
    }

    let mut preview_pkg = pkg.clone();
    for (dst, res) in preview_pkg.parts.iter_mut().zip(part_results.iter()) {
        dst.mesh = res.mesh.clone();
        dst.attributes = res.attributes.clone();
    }
    let (combined_mesh, combined_attrs) = combine_threemf_preview(&preview_pkg);

    let (before_preview, _) = combine_threemf_preview(pkg);
    let before = analyze(&before_preview);
    let mut after = analyze(&combined_mesh);
    after.is_watertight = part_results.iter().all(|r| r.report.after.is_watertight);

    let mut kept_steps = Vec::new();
    let mut rejected_steps = Vec::new();
    let mut warnings = Vec::new();
    let mut unresolved_notes = Vec::new();
    let mut attr_sum = AttributeTransferSummary::default();
    let mut all_noop = true;

    for (part, res) in pkg.parts.iter().zip(part_results.iter()) {
        let label = part.object_name.as_deref().unwrap_or(&part.object_id);
        if !res.report.was_noop {
            all_noop = false;
        }
        for mut s in res.report.kept_steps.clone() {
            s.name = format!("[{label}] {}", s.name);
            kept_steps.push(s);
        }
        for mut s in res.report.rejected_steps.clone() {
            s.name = format!("[{label}] {}", s.name);
            rejected_steps.push(s);
        }
        for w in &res.report.warnings {
            warnings.push(format!("[{label}] {w}"));
        }
        for n in &res.report.unresolved_notes {
            unresolved_notes.push(format!("[{label}] {n}"));
        }
        attr_sum.exact_count += res.report.attribute_summary.exact_count;
        attr_sum.inherited_count += res.report.attribute_summary.inherited_count;
        attr_sum.uncertain_count += res.report.attribute_summary.uncertain_count;
    }

    let summary_result = RepairResult {
        mesh: combined_mesh.clone(),
        attributes: combined_attrs.clone(),
        report: RepairReport {
            profile,
            before,
            after,
            kept_steps,
            rejected_steps,
            attribute_summary: attr_sum,
            warnings,
            unresolved_notes,
            was_noop: all_noop,
            unsupported_features: unsupported_features_list(),
        },
    };

    Ok(ThreeMfRepairBundle {
        part_results,
        combined_mesh,
        combined_attrs,
        summary_result,
    })
}

fn paths_match(a: &Path, b: &Path) -> bool {
    if a == b {
        return true;
    }
    match (std::fs::canonicalize(a), std::fs::canonicalize(b)) {
        (Ok(ca), Ok(cb)) => ca == cb,
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_formats::write_stl_binary_path;
    use nalgebra::Point3;
    use std::io::{Cursor, Write};
    use std::time::Duration;
    use zip::write::SimpleFileOptions;
    use zip::{CompressionMethod, ZipWriter};

    fn small_hole_cube() -> TriangleMesh {
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(100.0, 0.0, 0.0),
            Point3::new(100.0, 100.0, 0.0),
            Point3::new(0.0, 100.0, 0.0),
            Point3::new(0.0, 0.0, 100.0),
            Point3::new(100.0, 0.0, 100.0),
            Point3::new(100.0, 100.0, 100.0),
            Point3::new(0.0, 100.0, 100.0),
            Point3::new(49.0, 49.0, 100.0),
            Point3::new(51.0, 49.0, 100.0),
            Point3::new(51.0, 51.0, 100.0),
            Point3::new(49.0, 51.0, 100.0),
        ];
        let faces = vec![
            [0, 2, 1],
            [0, 3, 2],
            [0, 1, 5],
            [0, 5, 4],
            [2, 3, 7],
            [2, 7, 6],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
            [4, 5, 9],
            [4, 9, 8],
            [5, 6, 10],
            [5, 10, 9],
            [6, 7, 11],
            [6, 11, 10],
            [7, 4, 8],
            [7, 8, 11],
        ];
        TriangleMesh::from_parts(vertices, faces)
    }

    fn write_sample_painted_3mf(path: &Path) {
        let rels = br#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Target="/3D/3dmodel.model" Id="rel-1" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel" />
</Relationships>"#;
        let model = br#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
  <resources>
    <object id="1" name="PaintedPart" type="model">
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
          <triangle v1="0" v2="2" v3="1" paint_color="4" />
          <triangle v1="0" v2="3" v3="2" paint_color="4" />
          <triangle v1="0" v2="1" v3="5" paint_color="0C" />
          <triangle v1="0" v2="5" v3="4" paint_color="0C" />
          <triangle v1="2" v2="3" v3="7" paint_color="0C" />
          <triangle v1="2" v2="7" v3="6" paint_color="0C" />
          <triangle v1="0" v2="4" v3="7" paint_color="0C" />
          <triangle v1="0" v2="7" v3="3" paint_color="0C" />
          <triangle v1="1" v2="2" v3="6" paint_color="0C" />
          <triangle v1="1" v2="6" v3="5" paint_color="0C" />
          <triangle v1="4" v2="5" v3="9" paint_color="1C" />
          <triangle v1="4" v2="9" v3="8" paint_color="1C" />
          <triangle v1="5" v2="6" v3="10" paint_color="1C" />
          <triangle v1="5" v2="10" v3="9" paint_color="1C" />
          <triangle v1="6" v2="7" v3="11" paint_color="1C" />
          <triangle v1="6" v2="11" v3="10" paint_color="1C" />
          <triangle v1="7" v2="4" v3="8" paint_color="1C" />
          <triangle v1="7" v2="8" v3="11" paint_color="1C" />
        </triangles>
      </mesh>
    </object>
  </resources>
  <build><item objectid="1" /></build>
</model>"#;
        let cfg = br##"{"filament_colour":["#8080FF","#000000","#FFFFFF","#E06040"]}"##;
        let mut buf = Cursor::new(Vec::new());
        {
            let mut zw = ZipWriter::new(&mut buf);
            let opts = SimpleFileOptions::default().compression_method(CompressionMethod::Deflated);
            zw.start_file("_rels/.rels", opts).unwrap();
            zw.write_all(rels).unwrap();
            zw.start_file("3D/3dmodel.model", opts).unwrap();
            zw.write_all(model).unwrap();
            zw.start_file("Metadata/project_settings.config", opts)
                .unwrap();
            zw.write_all(cfg).unwrap();
            zw.finish().unwrap();
        }
        std::fs::write(path, buf.into_inner()).unwrap();
    }

    #[test]
    fn defaults_to_conservative_and_loads_painted_3mf() {
        let dir = tempfile::tempdir().expect("tempdir");
        let src = dir.path().join("painted.3mf");
        let dst = dir.path().join("painted_repaired.3mf");
        write_sample_painted_3mf(&src);

        let mut vm = AppViewModel::new();
        assert_eq!(vm.profile, RepairProfile::Conservative);
        assert!(vm.dimensions_locked);

        vm.open_path(&src);
        assert!(vm.threemf_pkg.is_some());
        assert_eq!(vm.suggested_export_path().as_deref(), Some(dst.as_path()));

        vm.profile = RepairProfile::Standard;
        vm.start_repair();
        for _ in 0..100 {
            if vm.poll_worker() && !vm.repair_in_flight {
                break;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        assert!(!vm.repair_in_flight);
        assert!(
            vm.repair_result
                .as_ref()
                .is_some_and(|r| r.report.after.is_watertight)
        );
        // Must refuse exporting a 3MF package to STL
        assert!(vm.export_to_path(&dir.path().join("lossy.stl")).is_err());
        let val = vm.export_to_path(&dst).expect("export 3mf");
        assert!(val.passed);
        assert!(dst.exists());
    }

    #[test]
    fn view_model_repairs_and_refuses_source_overwrite() {
        let dir = tempfile::tempdir().expect("tempdir");
        let src = dir.path().join("part.stl");
        let dst = dir.path().join("part_repaired.stl");
        write_stl_binary_path(&src, &small_hole_cube()).expect("write");

        let mut vm = AppViewModel::new();
        vm.open_path(&src);
        assert!(vm.before_report.as_ref().is_some_and(|r| !r.is_watertight));
        assert_eq!(vm.suggested_export_path().as_deref(), Some(dst.as_path()));

        vm.profile = RepairProfile::Standard;
        vm.start_repair();
        for _ in 0..100 {
            if vm.poll_worker() && !vm.repair_in_flight {
                break;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        assert!(!vm.repair_in_flight);
        assert!(
            vm.repair_result
                .as_ref()
                .is_some_and(|r| r.report.after.is_watertight)
        );

        assert!(vm.export_to_path(&src).is_err());
        let val = vm.export_to_path(&dst).expect("export");
        assert!(val.passed);
        assert!(dst.exists());
    }
}
