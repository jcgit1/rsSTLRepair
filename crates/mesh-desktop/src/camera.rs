//! Shared Z-up CAD orbit/pan/zoom camera for synchronized Before/After viewports.

use mesh_core::BoundingBox;
use nalgebra::{Matrix4, Point3, Vector3};

/// Z-up perspective orbit camera shared across both viewports.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OrbitCamera {
    pub target: Point3<f32>,
    pub distance: f32,
    pub yaw: f32,
    pub pitch: f32,
    pub fov_y_rad: f32,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            target: Point3::origin(),
            distance: 150.0,
            yaw: 0.72,
            pitch: 0.52,
            fov_y_rad: 42.0_f32.to_radians(),
        }
    }
}

impl OrbitCamera {
    /// Reset framing to fit a mesh bounding box cleanly inside the viewport.
    pub fn fit_to_bbox(&mut self, bbox: &BoundingBox) {
        let c = bbox.center();
        self.target = Point3::new(c.x as f32, c.y as f32, c.z as f32);
        let diag = (bbox.diagonal() as f32).max(1.0);
        self.distance = (diag * 1.45).max(2.0);
        self.yaw = 0.72;
        self.pitch = 0.52;
    }

    /// Orbit around `target` from pointer drag delta in logical points.
    pub fn orbit(&mut self, dx: f32, dy: f32) {
        self.yaw -= dx * 0.008;
        self.pitch = (self.pitch + dy * 0.008).clamp(-1.52, 1.52);
    }

    /// Pan parallel to the view plane from pointer drag delta.
    pub fn pan(&mut self, dx: f32, dy: f32, viewport_height: f32) {
        let (forward, right, up) = self.basis();
        let _ = forward;
        let world_per_px =
            (2.0 * self.distance * (self.fov_y_rad * 0.5).tan()) / viewport_height.max(1.0);
        self.target -= right * (dx * world_per_px);
        self.target += up * (dy * world_per_px);
    }

    /// Multiplicative dolly zoom, clamped to safe near/far limits.
    pub fn zoom(&mut self, factor: f32) {
        self.distance = (self.distance * factor).clamp(0.5, 50_000.0);
    }

    /// Compute camera eye position in world space (Z-up).
    #[must_use]
    pub fn eye(&self) -> Point3<f32> {
        let cp = self.pitch.cos();
        let offset = Vector3::new(
            self.distance * cp * self.yaw.cos(),
            self.distance * cp * self.yaw.sin(),
            self.distance * self.pitch.sin(),
        );
        self.target + offset
    }

    fn basis(&self) -> (Vector3<f32>, Vector3<f32>, Vector3<f32>) {
        let eye = self.eye();
        let forward = (self.target - eye).normalize();
        let world_up = Vector3::z();
        let right = forward.cross(&world_up).normalize();
        let up = right.cross(&forward).normalize();
        (forward, right, up)
    }

    /// Build a wgpu-compatible (0..1 depth NDC) perspective view-projection matrix and eye vector.
    #[must_use]
    pub fn view_proj(&self, aspect: f32) -> (Matrix4<f32>, [f32; 3]) {
        let eye = self.eye();
        let view = Matrix4::look_at_rh(&eye, &self.target, &Vector3::z());
        let near = (self.distance * 0.005).max(0.05);
        let far = (self.distance * 20.0).max(500.0);
        let proj = perspective_wgpu(aspect.max(0.05), self.fov_y_rad, near, far);
        (proj * view, [eye.x, eye.y, eye.z])
    }
}

fn perspective_wgpu(aspect: f32, fov_y: f32, near: f32, far: f32) -> Matrix4<f32> {
    let f = 1.0 / (fov_y * 0.5).tan();
    let nf = 1.0 / (near - far);
    Matrix4::new(
        f / aspect,
        0.0,
        0.0,
        0.0,
        0.0,
        f,
        0.0,
        0.0,
        0.0,
        0.0,
        far * nf,
        near * far * nf,
        0.0,
        0.0,
        -1.0,
        0.0,
    )
}
