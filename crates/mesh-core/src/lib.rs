//! Core non-manifold indexed triangle mesh kernel, IDs, repair profiles, and limits.

use nalgebra::{Point3, Vector3};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use thiserror::Error;

/// Strongly-typed vertex index into [`TriangleMesh::vertices`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct VertexId(pub u32);

/// Strongly-typed face index into [`TriangleMesh::faces`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct FaceId(pub u32);

/// Undirected edge key with canonical sorted vertex indices `(min, max)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct EdgeKey(pub u32, pub u32);

impl EdgeKey {
    /// Construct a canonical undirected edge key from two vertex indices.
    #[must_use]
    pub fn new(a: u32, b: u32) -> Self {
        if a <= b { Self(a, b) } else { Self(b, a) }
    }

    /// Return the two vertex IDs in canonical order.
    #[must_use]
    pub fn vertices(self) -> (VertexId, VertexId) {
        (VertexId(self.0), VertexId(self.1))
    }
}

/// One directed occurrence of an edge within a triangle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EdgeUse {
    pub face: FaceId,
    /// Directed vertices `(u, v)` in the triangle's winding order.
    pub directed: (u32, u32),
}

/// Repair aggressiveness profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum RepairProfile {
    /// Only low-risk corrections; prioritize shape and color/material preservation.
    #[default]
    Conservative,
    /// Automatically repair common defects, including small holes and simple non-manifold edges.
    Standard,
    /// Wider hole caps, loose-fragment removal, and local remeshing. Remeshed faces can lose exact paint correspondence.
    Aggressive,
}

impl RepairProfile {
    /// Human-readable label for CLI and UI display.
    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            Self::Conservative => "Conservative",
            Self::Standard => "Standard",
            Self::Aggressive => "Aggressive",
        }
    }
}

/// Hard safety caps to bound memory and reject hostile or malformed inputs early.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ResourceLimits {
    pub max_file_bytes: u64,
    pub max_triangles: usize,
    pub max_vertices: usize,
    pub max_decompressed_bytes: u64,
}

impl Default for ResourceLimits {
    fn default() -> Self {
        Self {
            max_file_bytes: 256 * 1024 * 1024,
            max_triangles: 10_000_000,
            max_vertices: 10_000_000,
            max_decompressed_bytes: 512 * 1024 * 1024,
        }
    }
}

/// Axis-aligned bounding box in `f64` coordinates (typically millimeters).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BoundingBox {
    pub min: Point3<f64>,
    pub max: Point3<f64>,
}

impl BoundingBox {
    /// Compute the bounding box across finite points. Returns `None` if no finite points exist.
    #[must_use]
    pub fn from_finite_points<'a>(
        points: impl IntoIterator<Item = &'a Point3<f64>>,
    ) -> Option<Self> {
        let mut iter = points
            .into_iter()
            .filter(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite());
        let first = *iter.next()?;
        let mut min = first;
        let mut max = first;
        for p in iter {
            min.x = min.x.min(p.x);
            min.y = min.y.min(p.y);
            min.z = min.z.min(p.z);
            max.x = max.x.max(p.x);
            max.y = max.y.max(p.y);
            max.z = max.z.max(p.z);
        }
        Some(Self { min, max })
    }

    /// Extent vector `(max - min)`.
    #[must_use]
    pub fn size(&self) -> Vector3<f64> {
        self.max - self.min
    }

    /// Euclidean length of the bounding-box diagonal.
    #[must_use]
    pub fn diagonal(&self) -> f64 {
        self.size().norm()
    }

    /// Center point of the bounding box.
    #[must_use]
    pub fn center(&self) -> Point3<f64> {
        Point3::from((self.min.coords + self.max.coords) * 0.5)
    }

    /// Maximum coordinate drift between two bounding boxes across min and max corners.
    #[must_use]
    pub fn max_corner_drift(&self, other: &Self) -> f64 {
        let d_min = (self.min - other.min).abs();
        let d_max = (self.max - other.max).abs();
        d_min
            .x
            .max(d_min.y)
            .max(d_min.z)
            .max(d_max.x)
            .max(d_max.y)
            .max(d_max.z)
    }
}

/// Errors returned across mesh loading, analysis, repair, and validation.
#[derive(Debug, Error)]
pub enum MeshError {
    #[error("file exceeds maximum allowed size ({size} bytes > {limit} bytes)")]
    FileTooLarge { size: u64, limit: u64 },

    #[error("mesh exceeds maximum triangle count ({count} > {limit})")]
    TooManyTriangles { count: usize, limit: usize },

    #[error("mesh exceeds maximum vertex count ({count} > {limit})")]
    TooManyVertices { count: usize, limit: usize },

    #[error("invalid mesh file: {0}")]
    InvalidFormat(String),

    #[error("unsupported format: {0}")]
    UnsupportedFormat(String),

    #[error("operation cancelled")]
    Cancelled,

    #[error("validation failed: {0}")]
    ValidationFailed(String),

    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
}

/// Non-manifold-capable indexed triangle mesh in `f64` precision.
///
/// Unlike a classical half-edge structure, this kernel allows edges with
/// 0, 1, 2, or many incident triangles, duplicate faces, bowtie vertices,
/// and non-finite coordinates so broken meshes can be represented faithfully
/// for diagnosis and conservative repair.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct TriangleMesh {
    pub vertices: Vec<Point3<f64>>,
    pub faces: Vec<[u32; 3]>,
}

impl TriangleMesh {
    /// Create an empty triangle mesh.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Construct a mesh from raw vertex and face arrays.
    #[must_use]
    pub fn from_parts(vertices: Vec<Point3<f64>>, faces: Vec<[u32; 3]>) -> Self {
        Self { vertices, faces }
    }

    /// Number of vertices.
    #[must_use]
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Number of faces.
    #[must_use]
    pub fn face_count(&self) -> usize {
        self.faces.len()
    }

    /// Fetch a vertex position by [`VertexId`].
    #[must_use]
    pub fn vertex(&self, id: VertexId) -> Option<Point3<f64>> {
        self.vertices.get(id.0 as usize).copied()
    }

    /// Fetch vertex indices for a face.
    #[must_use]
    pub fn face(&self, id: FaceId) -> Option<[VertexId; 3]> {
        self.faces
            .get(id.0 as usize)
            .map(|&[a, b, c]| [VertexId(a), VertexId(b), VertexId(c)])
    }

    /// Fetch the 3 vertex positions of a face if all indices are in bounds.
    #[must_use]
    pub fn face_positions(&self, id: FaceId) -> Option<[Point3<f64>; 3]> {
        let &[a, b, c] = self.faces.get(id.0 as usize)?;
        let va = *self.vertices.get(a as usize)?;
        let vb = *self.vertices.get(b as usize)?;
        let vc = *self.vertices.get(c as usize)?;
        Some([va, vb, vc])
    }

    /// Compute the bounding box over finite vertices.
    #[must_use]
    pub fn bounding_box(&self) -> Option<BoundingBox> {
        BoundingBox::from_finite_points(self.vertices.iter())
    }

    /// Build a deterministic map from each undirected edge to all incident directed face edges.
    ///
    /// Faces with out-of-range vertex indices are skipped so this never panics on corrupted meshes.
    #[must_use]
    pub fn edge_incidence(&self) -> BTreeMap<EdgeKey, Vec<EdgeUse>> {
        let mut map: BTreeMap<EdgeKey, Vec<EdgeUse>> = BTreeMap::new();
        let n_verts = self.vertices.len();
        for (fi, &[a, b, c]) in self.faces.iter().enumerate() {
            if a as usize >= n_verts || b as usize >= n_verts || c as usize >= n_verts {
                continue;
            }
            let face = FaceId(fi as u32);
            for (u, v) in [(a, b), (b, c), (c, a)] {
                if u == v {
                    continue;
                }
                map.entry(EdgeKey::new(u, v)).or_default().push(EdgeUse {
                    face,
                    directed: (u, v),
                });
            }
        }
        map
    }

    /// Area of a single triangle in `f64`. Returns `0.0` if non-finite.
    #[must_use]
    pub fn triangle_area(a: Point3<f64>, b: Point3<f64>, c: Point3<f64>) -> f64 {
        let ab = b - a;
        let ac = c - a;
        let area = 0.5 * ab.cross(&ac).norm();
        if area.is_finite() { area } else { 0.0 }
    }

    /// Total surface area across all valid faces.
    #[must_use]
    pub fn surface_area(&self) -> f64 {
        (0..self.faces.len())
            .filter_map(|i| self.face_positions(FaceId(i as u32)))
            .map(|[a, b, c]| Self::triangle_area(a, b, c))
            .sum()
    }

    /// Signed volume enclosed by the mesh (positive for outward normals on a closed solid).
    #[must_use]
    pub fn signed_volume(&self) -> f64 {
        let mut sum = 0.0;
        for i in 0..self.faces.len() {
            if let Some([a, b, c]) = self.face_positions(FaceId(i as u32)) {
                let contrib = a.coords.dot(&b.coords.cross(&c.coords)) / 6.0;
                if contrib.is_finite() {
                    sum += contrib;
                }
            }
        }
        sum
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn edge_key_is_canonical() {
        assert_eq!(EdgeKey::new(5, 2), EdgeKey::new(2, 5));
        assert_eq!(EdgeKey::new(5, 2).vertices(), (VertexId(2), VertexId(5)));
    }

    #[test]
    fn unit_cube_volume_and_area() {
        let mesh = unit_cube();
        assert!((mesh.signed_volume() - 1.0).abs() < 1e-12);
        assert!((mesh.surface_area() - 6.0).abs() < 1e-12);
        let bbox = mesh.bounding_box().expect("cube bbox");
        assert_eq!(bbox.min, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(bbox.max, Point3::new(1.0, 1.0, 1.0));
    }

    fn unit_cube() -> TriangleMesh {
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(0.0, 1.0, 1.0),
        ];
        let faces = vec![
            [0, 2, 1],
            [0, 3, 2], // bottom z=0
            [4, 5, 6],
            [4, 6, 7], // top z=1
            [0, 1, 5],
            [0, 5, 4], // front y=0
            [2, 3, 7],
            [2, 7, 6], // back y=1
            [0, 4, 7],
            [0, 7, 3], // left x=0
            [1, 2, 6],
            [1, 6, 5], // right x=1
        ];
        TriangleMesh::from_parts(vertices, faces)
    }
}
