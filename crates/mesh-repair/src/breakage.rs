//! Synthetic broken samples for exercising the defects this release leaves in place.
//!
//! Each sample is small enough to load in the desktop app and specific enough that
//! Standard and Aggressive repair do not clear the defect it was built to show.

use mesh_core::TriangleMesh;
use nalgebra::Point3;

/// One way the desktop app can break a file for a repair test.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BreakKind {
    /// Opposite faces closer than the print-scale gap.
    ThinWalls,
    /// Openings that fail the hole cap: large, bent, pinched, or over 64 edges.
    UncappedHoles,
    /// An edge with four incident faces, which is not an unambiguous fin.
    AmbiguousNonManifold,
    /// Coplanar overlap. Splitting needs a crossing line, and the shell union needs two closed solids.
    PersistentCrossing,
    /// A closed solid whose only problem is a skinny triangle.
    SkinnyWatertight,
    /// A STEP file with an analytic surface and no triangular face.
    AnalyticStep,
    /// An SVG file with a curve, a transform, and a second contour.
    SvgCurvesTransformsHoles,
}

impl BreakKind {
    pub const ALL: [Self; 7] = [
        Self::ThinWalls,
        Self::UncappedHoles,
        Self::AmbiguousNonManifold,
        Self::PersistentCrossing,
        Self::SkinnyWatertight,
        Self::AnalyticStep,
        Self::SvgCurvesTransformsHoles,
    ];

    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            Self::ThinWalls => "Thin walls",
            Self::UncappedHoles => "Large, bent, pinched, or oversized holes",
            Self::AmbiguousNonManifold => "Non-manifold edge (not a 3-face fin)",
            Self::PersistentCrossing => "Crossing a split or union will not clear",
            Self::SkinnyWatertight => "Watertight solid with only skinny triangles",
            Self::AnalyticStep => "Analytic STEP solid",
            Self::SvgCurvesTransformsHoles => "SVG curves, transforms, and holes",
        }
    }
}

/// A mesh the app can repair, or file bytes the importers refuse.
#[derive(Debug, Clone, PartialEq)]
pub enum BrokenSample {
    Mesh {
        mesh: TriangleMesh,
        note: String,
    },
    File {
        file_name: &'static str,
        bytes: Vec<u8>,
        note: String,
    },
}

/// Build the broken sample for `kind`.
#[must_use]
pub fn break_file(kind: BreakKind) -> BrokenSample {
    match kind {
        BreakKind::ThinWalls => BrokenSample::Mesh {
            mesh: thin_box(),
            note: "Thin walls are reported and left in place.".to_string(),
        },
        BreakKind::UncappedHoles => BrokenSample::Mesh {
            mesh: uncapped_holes(),
            note: "These openings are large, bent, pinched, or past the 64-edge cap, so hole fill leaves them open.".to_string(),
        },
        BreakKind::AmbiguousNonManifold => BrokenSample::Mesh {
            mesh: four_face_edge(),
            note: "This edge has four faces. Only an unambiguous three-face fin is removed.".to_string(),
        },
        BreakKind::PersistentCrossing => BrokenSample::Mesh {
            mesh: coplanar_overlap(),
            note: "These triangles overlap in one plane. Splitting needs a proper crossing, and the union only runs on two closed shells.".to_string(),
        },
        BreakKind::SkinnyWatertight => BrokenSample::Mesh {
            mesh: skinny_solid(),
            note: "The solid is watertight. Aggressive remesh only edits defect seeds, so skinny triangles on a clean solid stay put.".to_string(),
        },
        BreakKind::AnalyticStep => BrokenSample::File {
            file_name: "rsstlrepair-analytic.step",
            bytes: ANALYTIC_STEP.as_bytes().to_vec(),
            note: "Analytic STEP solids are refused. This file has a cylinder and no triangular face.".to_string(),
        },
        BreakKind::SvgCurvesTransformsHoles => BrokenSample::File {
            file_name: "rsstlrepair-curves.svg",
            bytes: SVG_CURVES_TRANSFORM_HOLE.as_bytes().to_vec(),
            note: "SVG curves, transforms, and holes are refused.".to_string(),
        },
    }
}

const ANALYTIC_STEP: &str = "\
ISO-10303-21;
HEADER;
ENDSEC;
DATA;
#1=AXIS2_PLACEMENT_3D('',#2,#3,#4);
#2=CARTESIAN_POINT('',(0.,0.,0.));
#3=DIRECTION('',(0.,0.,1.));
#4=DIRECTION('',(1.,0.,0.));
#5=CYLINDRICAL_SURFACE('',#1,5.);
ENDSEC;
END-ISO-10303-21;
";

const SVG_CURVES_TRANSFORM_HOLE: &str = r#"<svg xmlns="http://www.w3.org/2000/svg">
  <path d="M0,0 C10,0 10,10 0,10 Z"/>
  <path d="M0,0 L20,0 L20,20 L0,20 Z M4,4 L16,4 L16,16 L4,16 Z"/>
  <rect width="4" height="4" transform="translate(1,1)"/>
</svg>
"#;

struct Builder {
    vertices: Vec<Point3<f64>>,
    faces: Vec<[u32; 3]>,
}

impl Builder {
    fn new() -> Self {
        Self {
            vertices: Vec::new(),
            faces: Vec::new(),
        }
    }

    fn v(&mut self, x: f64, y: f64, z: f64) -> u32 {
        let id = self.vertices.len() as u32;
        self.vertices.push(Point3::new(x, y, z));
        id
    }

    fn f(&mut self, a: u32, b: u32, c: u32) {
        self.faces.push([a, b, c]);
    }

    fn finish(self) -> TriangleMesh {
        TriangleMesh::from_parts(self.vertices, self.faces)
    }
}

fn scaled_box(dx: f64, dy: f64, dz: f64) -> TriangleMesh {
    let mut b = Builder::new();
    let p = [
        b.v(0.0, 0.0, 0.0),
        b.v(dx, 0.0, 0.0),
        b.v(dx, dy, 0.0),
        b.v(0.0, dy, 0.0),
        b.v(0.0, 0.0, dz),
        b.v(dx, 0.0, dz),
        b.v(dx, dy, dz),
        b.v(0.0, dy, dz),
    ];
    let faces = [
        [0, 2, 1],
        [0, 3, 2],
        [4, 5, 6],
        [4, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
        [2, 3, 7],
        [2, 7, 6],
        [0, 4, 7],
        [0, 7, 3],
        [1, 2, 6],
        [1, 6, 5],
    ];
    for face in faces {
        b.f(p[face[0]], p[face[1]], p[face[2]]);
    }
    b.finish()
}

fn thin_box() -> TriangleMesh {
    // 40 mm square, 0.12 mm thick. The print-scale gap caps at 0.35 mm.
    scaled_box(40.0, 40.0, 0.12)
}

fn skinny_solid() -> TriangleMesh {
    // Edge AD is about 0.54 mm and AB is 10 mm, so the incident faces are past the aspect-8 seed.
    // The tetrahedron is still a closed solid with no other defect.
    let mut b = Builder::new();
    let a = b.v(0.0, 0.0, 0.0);
    let btm = b.v(10.0, 0.0, 0.0);
    let c = b.v(0.0, 10.0, 0.0);
    let d = b.v(0.2, 0.0, 0.5);
    b.f(a, c, btm);
    b.f(a, btm, d);
    b.f(btm, c, d);
    b.f(c, a, d);
    b.finish()
}

fn four_face_edge() -> TriangleMesh {
    let mut b = Builder::new();
    let a = b.v(0.0, 0.0, 0.0);
    let top = b.v(0.0, 0.0, 10.0);
    let east = b.v(4.0, 0.0, 5.0);
    let north = b.v(0.0, 4.0, 5.0);
    let west = b.v(-4.0, 0.0, 5.0);
    let south = b.v(0.0, -4.0, 5.0);
    b.f(a, top, east);
    b.f(a, top, north);
    b.f(a, top, west);
    b.f(a, top, south);
    b.finish()
}

fn coplanar_overlap() -> TriangleMesh {
    let mut b = Builder::new();
    let a0 = b.v(0.0, 0.0, 0.0);
    let a1 = b.v(4.0, 0.0, 0.0);
    let a2 = b.v(0.0, 4.0, 0.0);
    let b0 = b.v(1.0, 1.0, 0.0);
    let b1 = b.v(5.0, 1.0, 0.0);
    let b2 = b.v(1.0, 5.0, 0.0);
    b.f(a0, a1, a2);
    b.f(b0, b1, b2);
    // Two extra faces join the overlapping triangles into one component
    // without giving the overlap a shared edge the splitter can cut.
    b.f(a0, a1, b1);
    b.f(a1, b0, b1);
    b.finish()
}

fn uncapped_holes() -> TriangleMesh {
    let mut b = Builder::new();
    let o0 = b.v(0.0, 0.0, 0.0);
    let o1 = b.v(100.0, 0.0, 0.0);
    let o2 = b.v(100.0, 100.0, 0.0);
    let o3 = b.v(0.0, 100.0, 0.0);
    let i0 = b.v(10.0, 10.0, 0.0);
    let i1 = b.v(90.0, 10.0, 0.0);
    let i2 = b.v(90.0, 90.0, 0.0);
    let i3 = b.v(10.0, 90.0, 0.0);
    // Square frame. The inner loop is an 80 mm hole, wider than 20% of the model.
    b.f(o0, o1, i1);
    b.f(o0, i1, i0);
    b.f(o1, o2, i2);
    b.f(o1, i2, i1);
    b.f(o2, o3, i3);
    b.f(o2, i3, i2);
    b.f(o3, o0, i0);
    b.f(o3, i0, i3);

    let oversized = add_annulus(&mut b, 150.0, 50.0, 6.0, 10.0, 65, None);
    bridge(&mut b, o1, o2, oversized[0], oversized[1]);

    let bent = add_saddle(&mut b, 150.0, -40.0);
    bridge(&mut b, o0, o1, bent[0], bent[1]);

    let (pinch_b, pinch_x) = add_pinched_loop(&mut b, 40.0, 140.0);
    bridge(&mut b, o2, o3, pinch_x, pinch_b);
    b.finish()
}

fn add_annulus(
    b: &mut Builder,
    cx: f64,
    cy: f64,
    inner_r: f64,
    outer_r: f64,
    n: usize,
    lift: Option<(usize, f64)>,
) -> Vec<u32> {
    let mut inner = Vec::with_capacity(n);
    let mut outer = Vec::with_capacity(n);
    for i in 0..n {
        let t = (i as f64) * std::f64::consts::TAU / (n as f64);
        let (s, c) = (t.sin(), t.cos());
        let z = lift
            .filter(|(index, _)| *index == i)
            .map(|(_, height)| height)
            .unwrap_or(0.0);
        inner.push(b.v(cx + inner_r * c, cy + inner_r * s, z));
        outer.push(b.v(cx + outer_r * c, cy + outer_r * s, 0.0));
    }
    for i in 0..n {
        let j = (i + 1) % n;
        b.f(outer[i], outer[j], inner[j]);
        b.f(outer[i], inner[j], inner[i]);
    }
    outer
}

fn add_saddle(b: &mut Builder, ox: f64, oy: f64) -> Vec<u32> {
    let outer = [
        b.v(ox, oy, 0.0),
        b.v(ox + 10.0, oy, 0.0),
        b.v(ox + 10.0, oy + 10.0, 0.0),
        b.v(ox, oy + 10.0, 0.0),
    ];
    // Opposite corners sit on opposite sides of the outer plane, so the
    // opening is bent past the 15% planarity cap while staying small.
    let inner = [
        b.v(ox + 2.0, oy + 2.0, 3.0),
        b.v(ox + 8.0, oy + 2.0, -3.0),
        b.v(ox + 8.0, oy + 8.0, 3.0),
        b.v(ox + 2.0, oy + 8.0, -3.0),
    ];
    for i in 0..4 {
        let j = (i + 1) % 4;
        b.f(outer[i], outer[j], inner[j]);
        b.f(outer[i], inner[j], inner[i]);
    }
    outer.to_vec()
}

fn bridge(b: &mut Builder, frame_a: u32, frame_b: u32, ring_a: u32, ring_b: u32) {
    b.f(frame_a, frame_b, ring_a);
    b.f(frame_b, ring_b, ring_a);
}

/// A boundary that visits one vertex twice, plus a spur vertex used to join a larger mesh.
fn add_pinched_loop(b: &mut Builder, ox: f64, oy: f64) -> (u32, u32) {
    let a = b.v(ox, oy, 0.0);
    let mid = b.v(ox + 2.0, oy, 0.0);
    let c = b.v(ox + 3.0, oy + 1.0, 0.0);
    let d = b.v(ox + 2.0, oy + 2.0, 0.0);
    let e = b.v(ox, oy + 1.0, 0.0);
    let hub = b.v(ox + 1.2, oy + 0.8, 0.0);
    let spur = b.v(ox + 1.2, oy + 0.8, 6.0);
    b.f(mid, a, hub);
    b.f(c, mid, hub);
    b.f(d, c, hub);
    b.f(mid, d, hub);
    b.f(e, mid, hub);
    b.f(a, e, hub);
    b.f(mid, hub, spur);
    (mid, spur)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::repair_mesh;
    use mesh_analysis::analyze;
    use mesh_attributes::AttributeStore;
    use mesh_core::RepairProfile;

    fn repaired(mesh: &TriangleMesh) -> crate::RepairResult {
        let attrs = AttributeStore::new_for_mesh(mesh.faces.len());
        repair_mesh(mesh, &attrs, RepairProfile::Aggressive, None).expect("repair")
    }

    #[test]
    fn thin_wall_is_reported_and_kept() {
        let mesh = thin_box();
        let before = analyze(&mesh);
        assert!(!before.thin_walls.is_empty());
        let result = repaired(&mesh);
        assert!(result.report.was_noop);
        assert!(!result.report.after.thin_walls.is_empty());
    }

    #[test]
    fn uncapped_holes_cover_each_refusal_and_stay_open() {
        let mesh = uncapped_holes();
        let before = analyze(&mesh);
        let diag = before.bounding_box.unwrap().diagonal();
        let mut large = false;
        let mut bent = false;
        let mut pinched = false;
        let mut oversized = false;
        for loop_ in &before.boundary_loops {
            if !loop_.is_closed {
                continue;
            }
            let repeated = {
                let mut ids: Vec<u32> = loop_.vertices.iter().map(|v| v.0).collect();
                let n = ids.len();
                ids.sort_unstable();
                ids.dedup();
                ids.len() != n
            };
            if repeated {
                pinched = true;
            }
            if loop_.vertices.len() > 64 {
                oversized = true;
            }
            if loop_.diameter > diag * 0.20 {
                large = true;
            }
            if loop_.vertices.len() <= 64
                && loop_.diameter <= diag * 0.20
                && loop_.planarity_error > loop_.diameter * 0.15
            {
                bent = true;
            }
        }
        assert!(large, "missing a hole wider than 20% of the model");
        assert!(bent, "missing a bent hole inside the diameter cap");
        assert!(pinched, "missing a pinched hole");
        assert!(oversized, "missing a hole past 64 edges");
        let after = repaired(&mesh);
        assert!(!after.report.after.boundary_edges.is_empty());
        assert!(
            after
                .report
                .after
                .boundary_loops
                .iter()
                .any(|loop_| loop_.vertices.len() > 64)
        );
    }

    #[test]
    fn four_face_edge_is_not_removed() {
        let mesh = four_face_edge();
        let before = analyze(&mesh);
        assert!(
            before
                .non_manifold_edges
                .iter()
                .any(|edge| edge.face_count >= 4)
        );
        let after = repaired(&mesh);
        assert!(
            after
                .report
                .after
                .non_manifold_edges
                .iter()
                .any(|edge| edge.face_count >= 4)
        );
    }

    #[test]
    fn coplanar_crossing_survives_aggressive_repair() {
        let mesh = coplanar_overlap();
        assert!(!analyze(&mesh).self_intersections.is_empty());
        let after = repaired(&mesh);
        assert!(!after.report.after.self_intersections.is_empty());
    }

    #[test]
    fn skinny_watertight_solid_is_left_unchanged() {
        let mesh = skinny_solid();
        let before = analyze(&mesh);
        assert!(
            before.is_watertight,
            "skinny sample should be a closed solid"
        );
        assert!(min_aspect(&mesh) >= 8.0);
        let after = repaired(&mesh);
        assert!(after.report.was_noop);
        assert!(
            after
                .report
                .kept_steps
                .iter()
                .all(|step| step.name != "Controlled remesh")
        );
        assert_eq!(after.mesh.faces.len(), mesh.faces.len());
    }

    #[test]
    fn refused_files_carry_the_unsupported_constructs() {
        let step = break_file(BreakKind::AnalyticStep);
        let BrokenSample::File { bytes, .. } = step else {
            panic!("step sample should be a file");
        };
        let text = String::from_utf8(bytes).unwrap();
        assert!(text.contains("CYLINDRICAL_SURFACE"));
        assert!(!text.contains("POLY_LOOP"));

        let svg = break_file(BreakKind::SvgCurvesTransformsHoles);
        let BrokenSample::File { bytes, .. } = svg else {
            panic!("svg sample should be a file");
        };
        let text = String::from_utf8(bytes).unwrap();
        assert!(text.contains('C'));
        assert!(text.contains("transform="));
        assert!(text.matches(" M").count() >= 1 || text.contains(" M4"));
    }

    fn min_aspect(mesh: &TriangleMesh) -> f64 {
        mesh.faces
            .iter()
            .map(|face| {
                let pts = [
                    mesh.vertices[face[0] as usize],
                    mesh.vertices[face[1] as usize],
                    mesh.vertices[face[2] as usize],
                ];
                let lengths = [
                    (pts[1] - pts[0]).norm(),
                    (pts[2] - pts[1]).norm(),
                    (pts[0] - pts[2]).norm(),
                ];
                let short = lengths.into_iter().fold(f64::MAX, f64::min);
                let long = lengths.into_iter().fold(0.0, f64::max);
                long / short
            })
            .fold(0.0, f64::max)
    }
}
