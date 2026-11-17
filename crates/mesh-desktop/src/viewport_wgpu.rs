//! wgpu 3D mesh and defect-overlay renderer embedded via `eframe::egui_wgpu::CallbackTrait`.

use bytemuck::{Pod, Zeroable};
use eframe::egui;
use eframe::egui_wgpu::{self, CallbackResources, CallbackTrait, ScreenDescriptor, wgpu};
use mesh_analysis::DiagnosticReport;
use mesh_attributes::{AttributeStore, Confidence};
use mesh_core::{EdgeKey, FaceId, TriangleMesh};
use nalgebra::Matrix4;
use std::collections::BTreeSet;
use std::sync::Arc;
use wgpu::util::DeviceExt;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ViewportSlot {
    Before,
    After,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct TriVertex {
    pos: [f32; 3],
    normal: [f32; 3],
    color: [f32; 3],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct LineVertex {
    pos: [f32; 3],
    color: [f32; 3],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct CameraUniform {
    view_proj: [[f32; 4]; 4],
    eye: [f32; 4],
}

/// CPU-side vertex buffers prepared only when mesh or overlay toggles change.
pub struct CpuViewportGeometry {
    pub revision_key: (u64, bool, bool, bool),
    tri_vertices: Vec<TriVertex>,
    line_vertices: Vec<LineVertex>,
}

impl CpuViewportGeometry {
    #[must_use]
    pub fn build(
        mesh: &TriangleMesh,
        attrs: Option<&AttributeStore>,
        report: Option<&DiagnosticReport>,
        show_defects: bool,
        show_wireframe: bool,
        show_grid: bool,
        revision_key: (u64, bool, bool, bool),
    ) -> Self {
        let mut tri_vertices = Vec::with_capacity(mesh.faces.len() * 3);
        let mut line_vertices = Vec::new();

        let uncertain_faces: BTreeSet<usize> = attrs
            .map(|a| {
                a.confidence
                    .iter()
                    .enumerate()
                    .filter_map(|(i, c)| (*c == Confidence::Uncertain).then_some(i))
                    .collect()
            })
            .unwrap_or_default();

        for (fi, _) in mesh.faces.iter().enumerate() {
            let Some([a, b, c]) = mesh.face_positions(FaceId(fi as u32)) else {
                continue;
            };
            if !is_finite_pt(&a) || !is_finite_pt(&b) || !is_finite_pt(&c) {
                continue;
            }
            let ab = b - a;
            let ac = c - a;
            let n = ab.cross(&ac);
            let len = n.norm();
            let normal = if len > 1e-12 && len.is_finite() {
                let u = n / len;
                [u.x as f32, u.y as f32, u.z as f32]
            } else {
                [0.0, 0.0, 1.0]
            };

            let color = if uncertain_faces.contains(&fi) {
                [0.88, 0.72, 0.30]
            } else if let Some(mat_id) = attrs.and_then(|st| st.faces.get(fi)?.material_id) {
                categorical_material_color(mat_id)
            } else {
                [0.72, 0.75, 0.79]
            };

            for p in [a, b, c] {
                tri_vertices.push(TriVertex {
                    pos: [p.x as f32, p.y as f32, p.z as f32],
                    normal,
                    color,
                });
            }

            if show_wireframe {
                let wc = [0.22, 0.25, 0.30];
                let pa = [a.x as f32, a.y as f32, a.z as f32];
                let pb = [b.x as f32, b.y as f32, b.z as f32];
                let pc = [c.x as f32, c.y as f32, c.z as f32];
                line_vertices.push(LineVertex { pos: pa, color: wc });
                line_vertices.push(LineVertex { pos: pb, color: wc });
                line_vertices.push(LineVertex { pos: pb, color: wc });
                line_vertices.push(LineVertex { pos: pc, color: wc });
                line_vertices.push(LineVertex { pos: pc, color: wc });
                line_vertices.push(LineVertex { pos: pa, color: wc });
            }
        }

        if show_defects && let Some(rep) = report {
            let push_edge = |out: &mut Vec<LineVertex>, ek: EdgeKey, color: [f32; 3]| {
                let (v0, v1) = ek.vertices();
                if let (Some(p0), Some(p1)) = (mesh.vertex(v0), mesh.vertex(v1))
                    && is_finite_pt(&p0)
                    && is_finite_pt(&p1)
                {
                    out.push(LineVertex {
                        pos: [p0.x as f32, p0.y as f32, p0.z as f32],
                        color,
                    });
                    out.push(LineVertex {
                        pos: [p1.x as f32, p1.y as f32, p1.z as f32],
                        color,
                    });
                }
            };

            for &ek in &rep.boundary_edges {
                push_edge(&mut line_vertices, ek, [0.95, 0.67, 0.22]); // amber
            }
            for nm in &rep.non_manifold_edges {
                push_edge(&mut line_vertices, nm.edge, [0.92, 0.27, 0.27]); // red
            }
            for &ek in &rep.winding_conflict_edges {
                push_edge(&mut line_vertices, ek, [0.85, 0.38, 0.78]); // magenta
            }
        }

        if show_grid && let Some(bbox) = mesh.bounding_box() {
            append_floor_grid(&mut line_vertices, bbox);
        }

        Self {
            revision_key,
            tri_vertices,
            line_vertices,
        }
    }
}

fn is_finite_pt(p: &nalgebra::Point3<f64>) -> bool {
    p.x.is_finite() && p.y.is_finite() && p.z.is_finite()
}

fn categorical_material_color(id: u32) -> [f32; 3] {
    const PALETTE: [[f32; 3]; 8] = [
        [0.68, 0.74, 0.80],
        [0.44, 0.66, 0.86],
        [0.82, 0.58, 0.40],
        [0.54, 0.75, 0.56],
        [0.78, 0.50, 0.54],
        [0.69, 0.60, 0.82],
        [0.78, 0.74, 0.48],
        [0.48, 0.74, 0.74],
    ];
    PALETTE[(id as usize) % PALETTE.len()]
}

fn append_floor_grid(out: &mut Vec<LineVertex>, bbox: mesh_core::BoundingBox) {
    let center = bbox.center();
    let span = (bbox.diagonal() as f32 * 0.75).max(10.0);
    let step = nice_step(span / 5.0);
    let half = (span / step).ceil() * step;
    let z = bbox.min.z as f32;
    let cx = (center.x as f32 / step).round() * step;
    let cy = (center.y as f32 / step).round() * step;
    let grid_col = [0.20, 0.23, 0.28];

    let steps = (half / step).round() as i32;
    for i in -steps..=steps {
        let off = i as f32 * step;
        out.push(LineVertex {
            pos: [cx - half, cy + off, z],
            color: grid_col,
        });
        out.push(LineVertex {
            pos: [cx + half, cy + off, z],
            color: grid_col,
        });
        out.push(LineVertex {
            pos: [cx + off, cy - half, z],
            color: grid_col,
        });
        out.push(LineVertex {
            pos: [cx + off, cy + half, z],
            color: grid_col,
        });
    }
}

fn nice_step(raw: f32) -> f32 {
    let p = 10.0_f32.powf(raw.max(0.01).log10().floor());
    let f = raw / p;
    if f < 1.5 {
        p
    } else if f < 3.5 {
        2.0 * p
    } else if f < 7.5 {
        5.0 * p
    } else {
        10.0 * p
    }
}

struct SlotBuffers {
    revision_key: (u64, bool, bool, bool),
    tri_buf: Option<wgpu::Buffer>,
    tri_count: u32,
    line_buf: Option<wgpu::Buffer>,
    line_count: u32,
    uniform_buf: wgpu::Buffer,
    uniform_bind_group: wgpu::BindGroup,
}

/// Shared GPU pipelines and per-viewport vertex buffers stored in `egui_wgpu::CallbackResources`.
pub struct ViewportGpuResources {
    tri_pipeline: wgpu::RenderPipeline,
    line_pipeline: wgpu::RenderPipeline,
    bind_group_layout: wgpu::BindGroupLayout,
    before: Option<SlotBuffers>,
    after: Option<SlotBuffers>,
}

impl ViewportGpuResources {
    pub fn new(device: &wgpu::Device, target_format: wgpu::TextureFormat) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("mesh_viewport_shader"),
            source: wgpu::ShaderSource::Wgsl(SHADER_WGSL.into()),
        });

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("mesh_camera_bgl"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("mesh_pipeline_layout"),
            bind_group_layouts: &[Some(&bind_group_layout)],
            immediate_size: 0,
        });

        let tri_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("mesh_tri_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_tri"),
                buffers: &[Some(wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<TriVertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x3],
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::LessEqual),
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_tri"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: target_format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            multiview_mask: None,
            cache: None,
        });

        let line_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("mesh_line_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_line"),
                buffers: &[Some(wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<LineVertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3],
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: Some(false),
                depth_compare: Some(wgpu::CompareFunction::LessEqual),
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_line"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: target_format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            multiview_mask: None,
            cache: None,
        });

        Self {
            tri_pipeline,
            line_pipeline,
            bind_group_layout,
            before: None,
            after: None,
        }
    }

    fn slot_mut(&mut self, slot: ViewportSlot) -> &mut Option<SlotBuffers> {
        match slot {
            ViewportSlot::Before => &mut self.before,
            ViewportSlot::After => &mut self.after,
        }
    }

    fn slot(&self, slot: ViewportSlot) -> Option<&SlotBuffers> {
        match slot {
            ViewportSlot::Before => self.before.as_ref(),
            ViewportSlot::After => self.after.as_ref(),
        }
    }
}

/// Paint callback submitted each frame for one of the two synchronized viewports.
pub struct ViewportCallback {
    pub slot: ViewportSlot,
    pub view_proj: Matrix4<f32>,
    pub eye: [f32; 3],
    pub geometry: Option<Arc<CpuViewportGeometry>>,
}

impl CallbackTrait for ViewportCallback {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _screen_descriptor: &ScreenDescriptor,
        _egui_encoder: &mut wgpu::CommandEncoder,
        callback_resources: &mut CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        let Some(res) = callback_resources.get_mut::<ViewportGpuResources>() else {
            return Vec::new();
        };

        let Some(geom) = &self.geometry else {
            *res.slot_mut(self.slot) = None;
            return Vec::new();
        };

        let uniform = CameraUniform {
            view_proj: *self.view_proj.as_ref(),
            eye: [self.eye[0], self.eye[1], self.eye[2], 1.0],
        };

        let bgl = res.bind_group_layout.clone();
        let slot_entry = res.slot_mut(self.slot);

        let needs_upload = slot_entry
            .as_ref()
            .is_none_or(|b| b.revision_key != geom.revision_key);

        if needs_upload {
            let tri_buf = (!geom.tri_vertices.is_empty()).then(|| {
                device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("mesh_tri_vbuf"),
                    contents: bytemuck::cast_slice(&geom.tri_vertices),
                    usage: wgpu::BufferUsages::VERTEX,
                })
            });
            let line_buf = (!geom.line_vertices.is_empty()).then(|| {
                device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("mesh_line_vbuf"),
                    contents: bytemuck::cast_slice(&geom.line_vertices),
                    usage: wgpu::BufferUsages::VERTEX,
                })
            });

            let uniform_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("mesh_camera_ubuf"),
                contents: bytemuck::bytes_of(&uniform),
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            });
            let uniform_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("mesh_camera_bg"),
                layout: &bgl,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: uniform_buf.as_entire_binding(),
                }],
            });

            *slot_entry = Some(SlotBuffers {
                revision_key: geom.revision_key,
                tri_buf,
                tri_count: geom.tri_vertices.len() as u32,
                line_buf,
                line_count: geom.line_vertices.len() as u32,
                uniform_buf,
                uniform_bind_group,
            });
        } else if let Some(existing) = slot_entry.as_ref() {
            queue.write_buffer(&existing.uniform_buf, 0, bytemuck::bytes_of(&uniform));
        }

        Vec::new()
    }

    fn paint(
        &self,
        _info: egui::PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        callback_resources: &CallbackResources,
    ) {
        let Some(res) = callback_resources.get::<ViewportGpuResources>() else {
            return;
        };
        let Some(buffers) = res.slot(self.slot) else {
            return;
        };

        if let Some(tri_buf) = &buffers.tri_buf
            && buffers.tri_count > 0
        {
            render_pass.set_pipeline(&res.tri_pipeline);
            render_pass.set_bind_group(0, &buffers.uniform_bind_group, &[]);
            render_pass.set_vertex_buffer(0, tri_buf.slice(..));
            render_pass.draw(0..buffers.tri_count, 0..1);
        }

        if let Some(line_buf) = &buffers.line_buf
            && buffers.line_count > 0
        {
            render_pass.set_pipeline(&res.line_pipeline);
            render_pass.set_bind_group(0, &buffers.uniform_bind_group, &[]);
            render_pass.set_vertex_buffer(0, line_buf.slice(..));
            render_pass.draw(0..buffers.line_count, 0..1);
        }
    }
}

/// Build an `epaint::PaintCallback` for one viewport rect.
pub fn make_viewport_callback(
    rect: egui::Rect,
    slot: ViewportSlot,
    view_proj: Matrix4<f32>,
    eye: [f32; 3],
    geometry: Option<Arc<CpuViewportGeometry>>,
) -> egui::PaintCallback {
    egui_wgpu::Callback::new_paint_callback(
        rect,
        ViewportCallback {
            slot,
            view_proj,
            eye,
            geometry,
        },
    )
}

const SHADER_WGSL: &str = r#"
struct CameraUniform {
    view_proj: mat4x4<f32>,
    eye: vec4<f32>,
};

@group(0) @binding(0)
var<uniform> cam: CameraUniform;

struct TriIn {
    @location(0) pos: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec3<f32>,
};

struct TriOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) world_pos: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec3<f32>,
};

@vertex
fn vs_tri(in: TriIn) -> TriOut {
    var out: TriOut;
    out.clip_pos = cam.view_proj * vec4<f32>(in.pos, 1.0);
    out.world_pos = in.pos;
    out.normal = in.normal;
    out.color = in.color;
    return out;
}

@fragment
fn fs_tri(in: TriOut, @builtin(front_facing) front: bool) -> @location(0) vec4<f32> {
    let n = select(-normalize(in.normal), normalize(in.normal), front);
    let view_dir = normalize(cam.eye.xyz - in.world_pos);
    let key_dir = normalize(vec3<f32>(0.55, 0.45, 0.85));
    let fill_dir = normalize(vec3<f32>(-0.70, -0.35, 0.35));

    let key = max(dot(n, key_dir), 0.0);
    let fill = max(dot(n, fill_dir), 0.0) * 0.35;
    let ambient = 0.24;

    let half_vec = normalize(key_dir + view_dir);
    let spec = pow(max(dot(n, half_vec), 0.0), 28.0) * 0.14;

    // Subtle cool-coral tint on backfaces so inverted triangles stand out in CAD shading.
    let base = select(in.color * vec3<f32>(0.96, 0.62, 0.58), in.color, front);
    let lit = base * (ambient + key * 0.72 + fill) + vec3<f32>(spec);
    return vec4<f32>(lit, 1.0);
}

struct LineIn {
    @location(0) pos: vec3<f32>,
    @location(1) color: vec3<f32>,
};

struct LineOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) color: vec3<f32>,
};

@vertex
fn vs_line(in: LineIn) -> LineOut {
    var out: LineOut;
    var cp = cam.view_proj * vec4<f32>(in.pos, 1.0);
    cp.z = cp.z - 0.0004 * cp.w;
    out.clip_pos = cp;
    out.color = in.color;
    return out;
}

@fragment
fn fs_line(in: LineOut) -> @location(0) vec4<f32> {
    return vec4<f32>(in.color, 1.0);
}
"#;
