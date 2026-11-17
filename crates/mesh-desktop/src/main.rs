mod camera;
mod view_model;
mod viewport_wgpu;

use eframe::egui::{self, Color32, CornerRadius, FontFamily, Frame, RichText, Stroke};
use mesh_core::RepairProfile;
use std::path::PathBuf;
use std::sync::Arc;
use tracing_subscriber::EnvFilter;
use view_model::AppViewModel;
use viewport_wgpu::{
    CpuViewportGeometry, ViewportGpuResources, ViewportSlot, make_viewport_callback,
};

const BG_WINDOW: Color32 = Color32::from_rgb(25, 28, 33);
const BG_PANEL: Color32 = Color32::from_rgb(32, 36, 43);
const BG_VIEWPORT: Color32 = Color32::from_rgb(20, 23, 28);
const BG_WIDGET: Color32 = Color32::from_rgb(42, 47, 56);
const BG_WIDGET_HOVER: Color32 = Color32::from_rgb(52, 58, 69);
const BORDER: Color32 = Color32::from_rgb(52, 58, 68);
const ACCENT: Color32 = Color32::from_rgb(59, 116, 217);
const TEXT_PRIMARY: Color32 = Color32::from_rgb(225, 229, 235);
const TEXT_MUTED: Color32 = Color32::from_rgb(148, 156, 168);
const COL_OK: Color32 = Color32::from_rgb(82, 178, 116);
const COL_WARN: Color32 = Color32::from_rgb(229, 169, 59);
const COL_ERR: Color32 = Color32::from_rgb(224, 82, 82);

fn main() -> eframe::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .with_writer(std::io::stderr)
        .init();

    let initial_file = std::env::args_os().nth(1).map(PathBuf::from);

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("rsSTLRepair")
            .with_inner_size([1360.0, 840.0])
            .with_min_inner_size([960.0, 600.0])
            .with_drag_and_drop(true),
        depth_buffer: 32,
        stencil_buffer: 0,
        multisampling: 0,
        ..Default::default()
    };

    eframe::run_native(
        "rsSTLRepair",
        options,
        Box::new(move |cc| {
            apply_industrial_style(&cc.egui_ctx);
            if let Some(render_state) = &cc.wgpu_render_state {
                let resources =
                    ViewportGpuResources::new(&render_state.device, render_state.target_format);
                render_state
                    .renderer
                    .write()
                    .callback_resources
                    .insert(resources);
            }
            let mut app = DesktopApp::new();
            if let Some(path) = initial_file {
                app.vm.open_path(&path);
            }
            Ok(Box::new(app))
        }),
    )
}

fn apply_industrial_style(ctx: &egui::Context) {
    let mut visuals = egui::Visuals::dark();
    visuals.window_fill = BG_WINDOW;
    visuals.panel_fill = BG_PANEL;
    visuals.faint_bg_color = BG_WINDOW;
    visuals.extreme_bg_color = BG_VIEWPORT;
    visuals.window_stroke = Stroke::new(1.0, BORDER);
    visuals.widgets.noninteractive.bg_fill = BG_PANEL;
    visuals.widgets.noninteractive.bg_stroke = Stroke::new(1.0, BORDER);
    visuals.widgets.noninteractive.fg_stroke = Stroke::new(1.0, TEXT_PRIMARY);
    visuals.widgets.noninteractive.corner_radius = CornerRadius::same(2);

    visuals.widgets.inactive.bg_fill = BG_WIDGET;
    visuals.widgets.inactive.weak_bg_fill = BG_WIDGET;
    visuals.widgets.inactive.bg_stroke = Stroke::new(1.0, BORDER);
    visuals.widgets.inactive.fg_stroke = Stroke::new(1.0, TEXT_PRIMARY);
    visuals.widgets.inactive.corner_radius = CornerRadius::same(2);

    visuals.widgets.hovered.bg_fill = BG_WIDGET_HOVER;
    visuals.widgets.hovered.weak_bg_fill = BG_WIDGET_HOVER;
    visuals.widgets.hovered.bg_stroke = Stroke::new(1.0, ACCENT);
    visuals.widgets.hovered.fg_stroke = Stroke::new(1.0, TEXT_PRIMARY);
    visuals.widgets.hovered.corner_radius = CornerRadius::same(2);

    visuals.widgets.active.bg_fill = ACCENT;
    visuals.widgets.active.weak_bg_fill = ACCENT;
    visuals.widgets.active.bg_stroke = Stroke::new(1.0, ACCENT);
    visuals.widgets.active.fg_stroke = Stroke::new(1.0, Color32::WHITE);
    visuals.widgets.active.corner_radius = CornerRadius::same(2);

    visuals.selection.bg_fill = ACCENT;
    visuals.selection.stroke = Stroke::new(1.0, Color32::WHITE);
    ctx.set_theme(egui::Theme::Dark);
    ctx.set_visuals_of(egui::Theme::Dark, visuals);
    ctx.style_mut_of(egui::Theme::Dark, |style| {
        style.spacing.item_spacing = egui::vec2(8.0, 6.0);
        style.spacing.button_padding = egui::vec2(10.0, 5.0);
    });
}

struct DesktopApp {
    vm: AppViewModel,
    cached_before: Option<Arc<CpuViewportGeometry>>,
    cached_after: Option<Arc<CpuViewportGeometry>>,
}

impl DesktopApp {
    fn new() -> Self {
        Self {
            vm: AppViewModel::new(),
            cached_before: None,
            cached_after: None,
        }
    }

    fn ensure_before_geometry(&mut self) -> Option<Arc<CpuViewportGeometry>> {
        let mesh = self.vm.before_mesh.as_ref()?;
        let key = (
            self.vm.before_revision,
            self.vm.show_defects,
            self.vm.show_wireframe,
            self.vm.show_grid,
        );
        if self
            .cached_before
            .as_ref()
            .is_some_and(|c| c.revision_key == key)
        {
            return self.cached_before.clone();
        }
        let built = Arc::new(CpuViewportGeometry::build(
            mesh,
            self.vm.before_attrs.as_ref(),
            self.vm.before_report.as_ref(),
            self.vm.show_defects,
            self.vm.show_wireframe,
            self.vm.show_grid,
            key,
        ));
        self.cached_before = Some(built.clone());
        Some(built)
    }

    fn ensure_after_geometry(&mut self) -> Option<Arc<CpuViewportGeometry>> {
        let mesh = self.vm.after_mesh.as_ref()?;
        let key = (
            self.vm.after_revision,
            self.vm.show_defects,
            self.vm.show_wireframe,
            self.vm.show_grid,
        );
        if self
            .cached_after
            .as_ref()
            .is_some_and(|c| c.revision_key == key)
        {
            return self.cached_after.clone();
        }
        let report = self.vm.repair_result.as_ref().map(|r| &r.report.after);
        let built = Arc::new(CpuViewportGeometry::build(
            mesh,
            self.vm.after_attrs.as_ref(),
            report,
            self.vm.show_defects,
            self.vm.show_wireframe,
            self.vm.show_grid,
            key,
        ));
        self.cached_after = Some(built.clone());
        Some(built)
    }

    fn handle_dropped_files(&mut self, ctx: &egui::Context) {
        let dropped: Vec<PathBuf> = ctx.input(|i| {
            i.raw
                .dropped_files
                .iter()
                .map(|f| f.path().to_path_buf())
                .collect()
        });
        if let Some(first) = dropped.first() {
            self.vm.open_path(first);
        }
    }

    fn draw_toolbar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            if ui.button("Open...").clicked()
                && let Some(path) = rfd::FileDialog::new()
                    .add_filter("Supported Meshes", &["stl", "obj"])
                    .add_filter("All Files", &["*"])
                    .pick_file()
            {
                self.vm.open_path(&path);
            }

            if let Some(path) = &self.vm.source_path {
                let name = path.file_name().and_then(|s| s.to_str()).unwrap_or("model");
                ui.label(RichText::new(name).strong());
            } else {
                ui.label(RichText::new("No model loaded").color(TEXT_MUTED));
            }

            ui.separator();

            ui.label(RichText::new("Profile:").color(TEXT_MUTED));
            ui.add_enabled_ui(!self.vm.repair_in_flight, |ui| {
                for prof in [
                    RepairProfile::Conservative,
                    RepairProfile::Standard,
                    RepairProfile::Aggressive,
                ] {
                    ui.selectable_value(&mut self.vm.profile, prof, prof.label());
                }
            });

            ui.separator();

            let has_mesh = self.vm.before_mesh.is_some();
            if ui
                .add_enabled(
                    has_mesh && !self.vm.repair_in_flight,
                    egui::Button::new("Analyze"),
                )
                .clicked()
            {
                self.vm.analyze_current();
            }

            if self.vm.repair_in_flight {
                if ui.button("Cancel").clicked() {
                    self.vm.cancel_repair();
                }
                ui.spinner();
            } else if ui
                .add_enabled(
                    has_mesh,
                    egui::Button::new(RichText::new("Repair").color(Color32::WHITE)).fill(ACCENT),
                )
                .clicked()
            {
                self.vm.start_repair();
            }

            let can_export = self.vm.after_mesh.is_some() && !self.vm.repair_in_flight;
            if ui
                .add_enabled(can_export, egui::Button::new("Export..."))
                .clicked()
            {
                let mut dialog = rfd::FileDialog::new()
                    .add_filter("STL Binary", &["stl"])
                    .add_filter("Wavefront OBJ", &["obj"]);
                if let Some(suggested) = self.vm.suggested_export_path() {
                    if let Some(parent) = suggested.parent() {
                        dialog = dialog.set_directory(parent);
                    }
                    if let Some(fname) = suggested.file_name().and_then(|s| s.to_str()) {
                        dialog = dialog.set_file_name(fname);
                    }
                }
                if let Some(target) = dialog.save_file() {
                    let _ = self.vm.export_to_path(&target);
                }
            }

            ui.separator();

            ui.checkbox(&mut self.vm.show_defects, "Defects");
            ui.checkbox(&mut self.vm.show_wireframe, "Wireframe");
            ui.checkbox(&mut self.vm.show_grid, "Grid");
            if ui
                .add_enabled(has_mesh, egui::Button::new("Reset View"))
                .clicked()
                && let Some(bbox) = self.vm.before_report.as_ref().and_then(|r| r.bounding_box)
            {
                self.vm.camera.fit_to_bbox(&bbox);
            }
        });
    }

    fn draw_viewports(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        let before_geom = self.ensure_before_geometry();
        let after_geom = self.ensure_after_geometry();

        let avail = ui.available_size();
        let gap = 6.0;
        let each_w = ((avail.x - gap) * 0.5).max(160.0);
        let each_h = avail.y.max(200.0);

        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = gap;
            self.draw_single_viewport(
                ui,
                ctx,
                ViewportSlot::Before,
                "BEFORE",
                egui::vec2(each_w, each_h),
                before_geom,
            );
            self.draw_single_viewport(
                ui,
                ctx,
                ViewportSlot::After,
                "AFTER",
                egui::vec2(each_w, each_h),
                after_geom,
            );
        });
    }

    fn draw_single_viewport(
        &mut self,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
        slot: ViewportSlot,
        title: &str,
        size: egui::Vec2,
        geometry: Option<Arc<CpuViewportGeometry>>,
    ) {
        let (rect, response) = ui.allocate_exact_size(size, egui::Sense::click_and_drag());

        if response.dragged_by(egui::PointerButton::Primary) {
            let shift = ctx.input(|i| i.modifiers.shift);
            if shift {
                self.vm.camera.pan(
                    response.drag_delta().x,
                    response.drag_delta().y,
                    rect.height(),
                );
            } else {
                self.vm
                    .camera
                    .orbit(response.drag_delta().x, response.drag_delta().y);
            }
        } else if response.dragged_by(egui::PointerButton::Middle)
            || response.dragged_by(egui::PointerButton::Secondary)
        {
            self.vm.camera.pan(
                response.drag_delta().x,
                response.drag_delta().y,
                rect.height(),
            );
        }

        if response.hovered() {
            let scroll = ctx.input(|i| i.smooth_scroll_delta.y);
            if scroll.abs() > 0.01 {
                let factor = (1.0 - scroll * 0.0015).clamp(0.65, 1.35);
                self.vm.camera.zoom(factor);
            }
        }

        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, CornerRadius::same(2), BG_VIEWPORT);

        let aspect = (rect.width() / rect.height().max(1.0)).max(0.05);
        let (view_proj, eye) = self.vm.camera.view_proj(aspect);
        if geometry.is_some() {
            painter.add(make_viewport_callback(rect, slot, view_proj, eye, geometry));
        }

        painter.rect_stroke(
            rect,
            CornerRadius::same(2),
            Stroke::new(1.0, BORDER),
            egui::StrokeKind::Inside,
        );

        let subtitle = match slot {
            ViewportSlot::Before => match &self.vm.before_report {
                Some(r) if r.is_watertight => "WATERTIGHT".to_string(),
                Some(r) if r.is_valid_open_surface => {
                    format!("OPEN SURFACE ({} boundary)", r.boundary_edges.len())
                }
                Some(r) => format!(
                    "{} boundary | {} non-manifold | {} winding",
                    r.boundary_edges.len(),
                    r.non_manifold_edges.len(),
                    r.winding_conflict_edges.len()
                ),
                None => "Drop .stl or .obj to inspect".to_string(),
            },
            ViewportSlot::After => match &self.vm.repair_result {
                Some(res) if res.report.was_noop => "NO-OP (already clean)".to_string(),
                Some(res) if res.report.after.is_watertight => "WATERTIGHT".to_string(),
                Some(res) => format!(
                    "{} boundary | {} non-manifold | {} winding",
                    res.report.after.boundary_edges.len(),
                    res.report.after.non_manifold_edges.len(),
                    res.report.after.winding_conflict_edges.len()
                ),
                None if self.vm.before_mesh.is_some() => "Click Repair to preview".to_string(),
                None => "Waiting for input".to_string(),
            },
        };

        let header_pos = rect.min + egui::vec2(10.0, 8.0);
        painter.text(
            header_pos,
            egui::Align2::LEFT_TOP,
            format!("{title}  |  {subtitle}"),
            egui::FontId::new(12.0, FontFamily::Monospace),
            TEXT_PRIMARY,
        );

        if slot == ViewportSlot::Before && self.vm.show_defects && self.vm.before_mesh.is_some() {
            let legend_pos = egui::pos2(rect.min.x + 10.0, rect.max.y - 10.0);
            painter.text(
                legend_pos,
                egui::Align2::LEFT_BOTTOM,
                "Amber: Boundary   Red: Non-manifold   Magenta: Winding",
                egui::FontId::new(11.0, FontFamily::Monospace),
                TEXT_MUTED,
            );
        }
    }

    fn draw_inspector(&self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.label(RichText::new("MODEL METRICS").strong());
            ui.separator();

            if let Some(before) = &self.vm.before_report {
                if let Some(bbox) = before.bounding_box {
                    let s = bbox.size();
                    ui.label(
                        RichText::new(format!("{:.3} x {:.3} x {:.3} mm", s.x, s.y, s.z))
                            .font(egui::FontId::new(13.0, FontFamily::Monospace)),
                    );
                }
                ui.label(
                    RichText::new(if self.vm.dimensions_locked {
                        "Dimensions lock: ON (1:1 scale preserved)"
                    } else {
                        "Dimensions lock: OFF"
                    })
                    .small()
                    .color(TEXT_MUTED),
                );
                ui.add_space(6.0);

                let after = self.vm.repair_result.as_ref().map(|r| &r.report.after);
                egui::Grid::new("metrics_grid")
                    .num_columns(3)
                    .spacing([14.0, 4.0])
                    .striped(true)
                    .show(ui, |ui| {
                        ui.label(RichText::new("Metric").color(TEXT_MUTED));
                        ui.label(RichText::new("Before").color(TEXT_MUTED));
                        ui.label(RichText::new("After").color(TEXT_MUTED));
                        ui.end_row();

                        metric_row(
                            ui,
                            "Vertices",
                            before.vertex_count.to_string(),
                            after.map_or("-".into(), |a| a.vertex_count.to_string()),
                        );
                        metric_row(
                            ui,
                            "Faces",
                            before.face_count.to_string(),
                            after.map_or("-".into(), |a| a.face_count.to_string()),
                        );
                        metric_row(
                            ui,
                            "Boundary",
                            before.boundary_edges.len().to_string(),
                            after.map_or("-".into(), |a| a.boundary_edges.len().to_string()),
                        );
                        metric_row(
                            ui,
                            "Non-manifold E",
                            before.non_manifold_edges.len().to_string(),
                            after.map_or("-".into(), |a| a.non_manifold_edges.len().to_string()),
                        );
                        metric_row(
                            ui,
                            "Non-manifold V",
                            before.non_manifold_vertices.len().to_string(),
                            after.map_or("-".into(), |a| a.non_manifold_vertices.len().to_string()),
                        );
                        metric_row(
                            ui,
                            "Winding",
                            before.winding_conflict_edges.len().to_string(),
                            after.map_or("-".into(), |a| {
                                a.winding_conflict_edges.len().to_string()
                            }),
                        );
                        metric_row(
                            ui,
                            "Components",
                            before.components.len().to_string(),
                            after.map_or("-".into(), |a| a.components.len().to_string()),
                        );
                    });
            } else {
                ui.label(RichText::new("Load an STL or OBJ model to view diagnostics.").color(TEXT_MUTED));
            }

            ui.add_space(10.0);
            ui.label(RichText::new("DIAGNOSTICS & STEPS").strong());
            ui.separator();

            if let Some(res) = &self.vm.repair_result {
                let (status_text, status_col) = if res.report.after.is_watertight {
                    ("Watertight solid", COL_OK)
                } else if res.report.after.is_valid_open_surface {
                    ("Valid open surface", COL_WARN)
                } else {
                    ("Unresolved defects remain", COL_ERR)
                };
                ui.label(RichText::new(status_text).strong().color(status_col));

                if res.report.was_noop {
                    ui.label("Input was already clean; geometry left untouched.");
                } else {
                    for step in &res.report.kept_steps {
                        ui.label(
                            RichText::new(format!(
                                "KEPT: {} ({} -> {}) — {}",
                                step.name, step.faces_before, step.faces_after, step.detail
                            ))
                            .small(),
                        );
                    }
                    for step in &res.report.rejected_steps {
                        ui.label(
                            RichText::new(format!("REJECTED: {} — {}", step.name, step.detail))
                                .small()
                                .color(COL_WARN),
                        );
                    }
                }

                for note in &res.report.unresolved_notes {
                    ui.label(RichText::new(format!("NOTE: {note}")).small().color(COL_WARN));
                }
                for warn in &res.report.warnings {
                    ui.label(RichText::new(format!("WARN: {warn}")).small().color(COL_WARN));
                }

                let attr = &res.report.attribute_summary;
                ui.add_space(4.0);
                ui.label(
                    RichText::new(format!(
                        "Attributes: {} exact, {} inherited, {} uncertain",
                        attr.exact_count, attr.inherited_count, attr.uncertain_count
                    ))
                    .small()
                    .font(egui::FontId::new(11.5, FontFamily::Monospace)),
                );
            } else if let Some(before) = &self.vm.before_report {
                if before.findings.is_empty() {
                    ui.label(RichText::new("No geometry or topology defects detected.").color(COL_OK));
                } else {
                    for f in &before.findings {
                        let col = match f.severity {
                            mesh_analysis::Severity::Info => TEXT_MUTED,
                            mesh_analysis::Severity::Warning => COL_WARN,
                            mesh_analysis::Severity::Error => COL_ERR,
                        };
                        ui.label(RichText::new(&f.message).small().color(col));
                    }
                }
            }

            if let Some(val) = &self.vm.validation {
                ui.add_space(8.0);
                ui.label(RichText::new("REIMPORT VALIDATION").strong());
                ui.separator();
                let col = if val.passed { COL_OK } else { COL_ERR };
                ui.label(
                    RichText::new(if val.passed {
                        "Export verified by reimport."
                    } else {
                        "Reimport validation failed."
                    })
                    .color(col),
                );
                if let Some(drift) = val.bbox_drift {
                    ui.label(
                        RichText::new(format!("BBox drift: {drift:.6} mm"))
                            .small()
                            .font(egui::FontId::new(11.5, FontFamily::Monospace)),
                    );
                }
            }

            ui.add_space(12.0);
            ui.label(RichText::new("SUPPORTED SCOPE").strong());
            ui.separator();
            ui.label(
                RichText::new(
                    "Current: STL (ASCII/binary), OBJ (groups/usemtl), Phase 1 repairs.\n\
                     Deferred: paint-safe 3MF (Milestone 3), self-intersections, remeshing, STEP/PLY/glTF.",
                )
                .small()
                .color(TEXT_MUTED),
            );
        });
    }
}

fn metric_row(ui: &mut egui::Ui, label: &str, before: String, after: String) {
    ui.label(label);
    ui.label(RichText::new(before).font(egui::FontId::new(12.0, FontFamily::Monospace)));
    ui.label(RichText::new(after).font(egui::FontId::new(12.0, FontFamily::Monospace)));
    ui.end_row();
}

impl eframe::App for DesktopApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.handle_dropped_files(&ctx);
        if self.vm.poll_worker() || self.vm.repair_in_flight {
            ctx.request_repaint();
        }

        egui::Panel::top("toolbar")
            .frame(
                Frame::new()
                    .fill(BG_PANEL)
                    .inner_margin(egui::Margin::symmetric(10, 8))
                    .stroke(Stroke::new(1.0, BORDER)),
            )
            .show(ui, |ui| {
                self.draw_toolbar(ui);
                if let Some(err) = &self.vm.banner_error {
                    ui.add_space(4.0);
                    Frame::new()
                        .fill(Color32::from_rgb(64, 32, 32))
                        .stroke(Stroke::new(1.0, COL_ERR))
                        .inner_margin(egui::Margin::symmetric(8, 5))
                        .corner_radius(CornerRadius::same(2))
                        .show(ui, |ui| {
                            ui.label(RichText::new(err).color(Color32::from_rgb(245, 208, 208)));
                        });
                }
            });

        egui::Panel::bottom("status_bar")
            .frame(
                Frame::new()
                    .fill(BG_PANEL)
                    .inner_margin(egui::Margin::symmetric(10, 5))
                    .stroke(Stroke::new(1.0, BORDER)),
            )
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new(&self.vm.status_line)
                            .small()
                            .font(egui::FontId::new(11.5, FontFamily::Monospace)),
                    );
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(
                            RichText::new("Drag: Orbit | Shift/Middle/Right: Pan | Scroll: Zoom")
                                .small()
                                .color(TEXT_MUTED),
                        );
                    });
                });
            });

        egui::Panel::right("inspector")
            .default_size(340.0)
            .min_size(290.0)
            .max_size(440.0)
            .frame(
                Frame::new()
                    .fill(BG_PANEL)
                    .inner_margin(egui::Margin::same(10))
                    .stroke(Stroke::new(1.0, BORDER)),
            )
            .show(ui, |ui| {
                self.draw_inspector(ui);
            });

        egui::CentralPanel::default()
            .frame(
                Frame::new()
                    .fill(BG_WINDOW)
                    .inner_margin(egui::Margin::same(8)),
            )
            .show(ui, |ui| {
                self.draw_viewports(ui, &ctx);
            });

        if ctx.input(|i| !i.raw.hovered_files.is_empty()) {
            let rect = ctx.content_rect();
            let painter = ctx.layer_painter(egui::LayerId::new(
                egui::Order::Foreground,
                egui::Id::unique("drop_overlay"),
            ));
            painter.rect_filled(
                rect,
                CornerRadius::ZERO,
                Color32::from_rgba_unmultiplied(20, 24, 32, 180),
            );
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "Drop .stl or .obj file to load",
                egui::FontId::proportional(20.0),
                TEXT_PRIMARY,
            );
        }
    }
}
