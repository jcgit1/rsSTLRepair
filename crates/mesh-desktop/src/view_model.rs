//! Headless-testable state and worker orchestration for the desktop application.

use crate::camera::OrbitCamera;
use mesh_analysis::{DiagnosticReport, analyze};
use mesh_attributes::AttributeStore;
use mesh_core::{MeshError, RepairProfile, ResourceLimits, TriangleMesh};
use mesh_formats::{FormatKind, load_model, save_model};
use mesh_repair::{RepairResult, repair_mesh};
use mesh_validation::{RoundtripRequest, ValidationResult, verify_roundtrip};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, TryRecvError};

#[derive(Debug)]
pub enum WorkerOutput {
    Completed(Result<RepairResult, MeshError>),
}

/// Application state decoupled from `egui`/`eframe` so workflow logic can be unit tested.
pub struct AppViewModel {
    pub limits: ResourceLimits,
    pub source_path: Option<PathBuf>,
    pub source_format: Option<FormatKind>,
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
            status_line: "Drop an .stl or .obj file, or click Open.".to_string(),
            before_revision: 0,
            after_revision: 0,
            repair_in_flight: false,
            cancel_flag: None,
            worker_rx: None,
        }
    }

    /// Load a mesh file from disk. Refuses `.3mf` explicitly instead of flattening or stripping data.
    pub fn open_path(&mut self, path: &Path) {
        self.banner_error = None;
        if path
            .extension()
            .and_then(|s| s.to_str())
            .is_some_and(|ext| ext.eq_ignore_ascii_case("3mf"))
        {
            self.banner_error = Some(
                "3MF refused: color-safe 3MF package editing is scheduled for Milestone 3. \
                 This build refuses .3mf files instead of silently stripping painting or slicer metadata."
                    .to_string(),
            );
            self.status_line = "Refused .3mf input (paint-safe 3MF not yet enabled).".to_string();
            return;
        }

        match load_model(path, &self.limits) {
            Ok(loaded) => {
                let report = analyze(&loaded.mesh);
                if let Some(bbox) = report.bounding_box {
                    self.camera.fit_to_bbox(&bbox);
                }
                self.source_path = Some(path.to_path_buf());
                self.source_format = Some(loaded.source_format);
                self.before_mesh = Some(loaded.mesh);
                self.before_attrs = Some(loaded.attributes);
                self.before_report = Some(report);
                self.after_mesh = None;
                self.after_attrs = None;
                self.repair_result = None;
                self.validation = None;
                self.before_revision += 1;
                self.after_revision += 1;
                self.status_line = format!("Loaded {}", path.display());
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

    /// Re-run non-mutating topology diagnostics on the loaded input mesh.
    pub fn analyze_current(&mut self) {
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
        let (Some(mesh), Some(attrs)) = (self.before_mesh.clone(), self.before_attrs.clone())
        else {
            return;
        };
        let profile = self.profile;
        let cancel = Arc::new(AtomicBool::new(false));
        let cancel_worker = Arc::clone(&cancel);
        let (tx, rx) = mpsc::channel();

        self.banner_error = None;
        self.repair_in_flight = true;
        self.cancel_flag = Some(cancel);
        self.worker_rx = Some(rx);
        self.status_line = format!("Running {} repair...", profile.label());

        std::thread::spawn(move || {
            let result = repair_mesh(&mesh, &attrs, profile, Some(&cancel_worker));
            let _ = tx.send(WorkerOutput::Completed(result));
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
            Ok(WorkerOutput::Completed(outcome)) => {
                self.repair_in_flight = false;
                self.cancel_flag = None;
                self.worker_rx = None;
                match outcome {
                    Ok(res) => {
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

    /// Default sibling export path `<stem>_repaired.<ext>` that never matches the source path.
    #[must_use]
    pub fn suggested_export_path(&self) -> Option<PathBuf> {
        let src = self.source_path.as_ref()?;
        let parent = src.parent().unwrap_or_else(|| Path::new("."));
        let stem = src.file_stem().and_then(|s| s.to_str()).unwrap_or("model");
        let ext = src.extension().and_then(|s| s.to_str()).unwrap_or("stl");
        Some(parent.join(format!("{stem}_repaired.{ext}")))
    }

    /// Validate roundtrip export and write the repaired mesh to `target`.
    ///
    /// Refuses to overwrite the original source path.
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
    use std::time::Duration;

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

    #[test]
    fn defaults_to_conservative_and_refuses_3mf() {
        let mut vm = AppViewModel::new();
        assert_eq!(vm.profile, RepairProfile::Conservative);
        assert!(vm.dimensions_locked);

        vm.open_path(Path::new("multicolor.3mf"));
        assert!(vm.before_mesh.is_none());
        assert!(
            vm.banner_error
                .as_deref()
                .is_some_and(|e| e.contains("Milestone 3"))
        );
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
