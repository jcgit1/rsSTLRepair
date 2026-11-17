//! Face correspondence, confidence tracking, and categorical/paint attribute preservation.

use mesh_core::FaceId;
use serde::{Deserialize, Serialize};

/// Provenance of a face produced by a repair operation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum FaceOrigin {
    /// Carried over unchanged from input face `FaceId`.
    Identity(FaceId),
    /// Created by splitting or subdividing input face `parent`.
    Split { parent: FaceId },
    /// Synthesized (for example, a hole-filling cap) bounded by `boundary_sources`.
    Synthesized { boundary_sources: Vec<FaceId> },
    /// Mapped by spatial proximity in Aggressive mode.
    Spatial { nearest: FaceId, distance: f64 },
}

/// Confidence that a face's categorical attributes match the user's intent.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Confidence {
    /// Exact byte-for-byte preservation from an unmodified source face.
    Exact,
    /// Inherited unambiguously (from a split parent or unanimous boundary faces).
    Inherited,
    /// Conflicting boundary colors/materials or spatial fallback; user should inspect.
    Uncertain,
}

/// Bambu Studio / OrcaSlicer / PrusaSlicer per-triangle painting assignment.
///
/// `paint_color` (and `slic3rpe:mmu_segmentation`) uses a TriangleSelector
/// nibble bitstream encoded as hexadecimal read from right to left:
/// - low 2 bits == 0: leaf node, state = high 2 bits (`s2 < 3`), or `3 + next_nibble`
///   when `s2 == 3` (escape marker, e.g. `"0C"` -> state 3, `"1C"` -> state 4)
/// - low 2 bits != 0: subdivided triangle split tree
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct PaintAssignment {
    /// Raw hex string from the 3MF attribute, kept verbatim for unchanged faces.
    pub raw_attribute: String,
    /// Decoded leaf filament state (`0` = default extruder, `s >= 1` = filament slot `s - 1`),
    /// or `None` if this face is a subdivided split tree or invalid hex.
    pub decoded_leaf_state: Option<u32>,
    /// True when the raw bitstream describes a multi-color split tree.
    pub is_subdivided_tree: bool,
}

impl PaintAssignment {
    /// Decode a Bambu/Orca `paint_color` or Prusa `mmu_segmentation` hex string.
    #[must_use]
    pub fn from_hex(raw: impl Into<String>) -> Self {
        let raw_attribute = raw.into();
        let trimmed = raw_attribute.trim();
        let mut nibbles = trimmed
            .chars()
            .rev()
            .map(|c| c.to_digit(16))
            .collect::<Option<Vec<u32>>>();

        let (decoded_leaf_state, is_subdivided_tree) = match nibbles.as_mut() {
            Some(tokens) if !tokens.is_empty() => {
                let first = tokens[0];
                let split_sides = first & 0b11;
                if split_sides != 0 {
                    (None, true)
                } else {
                    let s2 = first >> 2;
                    let state = if s2 == 3 {
                        if let Some(&esc) = tokens.get(1) {
                            if esc == 14 {
                                match (tokens.get(2), tokens.get(3)) {
                                    (Some(&lo), Some(&hi)) => Some(17 + lo + (hi << 4)),
                                    _ => None,
                                }
                            } else {
                                Some(3 + esc)
                            }
                        } else {
                            None
                        }
                    } else {
                        Some(s2)
                    };
                    (state, false)
                }
            }
            _ => (None, false),
        };

        Self {
            raw_attribute,
            decoded_leaf_state,
            is_subdivided_tree,
        }
    }
}

/// Categorical and paint attributes attached to a single face.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct FaceAttributes {
    pub material_id: Option<u32>,
    pub group_id: Option<u32>,
    pub paint: Option<PaintAssignment>,
}

/// Summary of attribute preservation across a topology-changing repair.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct AttributeTransferSummary {
    pub exact_count: usize,
    pub inherited_count: usize,
    pub uncertain_count: usize,
    pub warnings: Vec<String>,
}

/// Parallel attribute store indexed by [`FaceId`].
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AttributeStore {
    pub material_names: Vec<String>,
    pub group_names: Vec<String>,
    pub faces: Vec<FaceAttributes>,
    pub origins: Vec<FaceOrigin>,
    pub confidence: Vec<Confidence>,
}

impl AttributeStore {
    /// Create a default geometry-only attribute store for `face_count` faces.
    #[must_use]
    pub fn new_for_mesh(face_count: usize) -> Self {
        Self {
            material_names: Vec::new(),
            group_names: Vec::new(),
            faces: vec![FaceAttributes::default(); face_count],
            origins: (0..face_count)
                .map(|i| FaceOrigin::Identity(FaceId(i as u32)))
                .collect(),
            confidence: vec![Confidence::Exact; face_count],
        }
    }

    /// Intern a material name and return its categorical ID.
    pub fn intern_material(&mut self, name: &str) -> u32 {
        if let Some(pos) = self.material_names.iter().position(|n| n == name) {
            return pos as u32;
        }
        let id = self.material_names.len() as u32;
        self.material_names.push(name.to_string());
        id
    }

    /// Intern a group/object name and return its categorical ID.
    pub fn intern_group(&mut self, name: &str) -> u32 {
        if let Some(pos) = self.group_names.iter().position(|n| n == name) {
            return pos as u32;
        }
        let id = self.group_names.len() as u32;
        self.group_names.push(name.to_string());
        id
    }

    /// Transfer categorical attributes across a repair operation using its face-origin map.
    ///
    /// Categorical filament/material IDs are never interpolated or blended as RGB.
    /// Synthesized faces inherit an attribute only when all boundary sources agree;
    /// otherwise they are marked [`Confidence::Uncertain`] and a warning is recorded.
    #[must_use]
    pub fn transfer(&self, new_origins: &[FaceOrigin]) -> (Self, AttributeTransferSummary) {
        let mut next = Self {
            material_names: self.material_names.clone(),
            group_names: self.group_names.clone(),
            faces: Vec::with_capacity(new_origins.len()),
            origins: new_origins.to_vec(),
            confidence: Vec::with_capacity(new_origins.len()),
        };
        let mut summary = AttributeTransferSummary::default();
        let mut uncertain_synth = 0usize;
        let mut spatial_count = 0usize;

        for origin in new_origins {
            match origin {
                FaceOrigin::Identity(old) => {
                    let attrs = self.faces.get(old.0 as usize).cloned().unwrap_or_default();
                    let conf = self
                        .confidence
                        .get(old.0 as usize)
                        .copied()
                        .unwrap_or(Confidence::Exact);
                    next.faces.push(attrs);
                    next.confidence.push(conf);
                    match conf {
                        Confidence::Exact => summary.exact_count += 1,
                        Confidence::Inherited => summary.inherited_count += 1,
                        Confidence::Uncertain => summary.uncertain_count += 1,
                    }
                }
                FaceOrigin::Split { parent } => {
                    let attrs = self
                        .faces
                        .get(parent.0 as usize)
                        .cloned()
                        .unwrap_or_default();
                    next.faces.push(attrs);
                    next.confidence.push(Confidence::Inherited);
                    summary.inherited_count += 1;
                }
                FaceOrigin::Synthesized { boundary_sources } => {
                    let (attrs, unanimous) = self.resolve_boundary(boundary_sources);
                    next.faces.push(attrs);
                    if unanimous {
                        next.confidence.push(Confidence::Inherited);
                        summary.inherited_count += 1;
                    } else {
                        next.confidence.push(Confidence::Uncertain);
                        summary.uncertain_count += 1;
                        uncertain_synth += 1;
                    }
                }
                FaceOrigin::Spatial { nearest, .. } => {
                    let attrs = self
                        .faces
                        .get(nearest.0 as usize)
                        .cloned()
                        .unwrap_or_default();
                    next.faces.push(attrs);
                    next.confidence.push(Confidence::Uncertain);
                    summary.uncertain_count += 1;
                    spatial_count += 1;
                }
            }
        }

        if uncertain_synth > 0 {
            summary.warnings.push(format!(
                "{uncertain_synth} synthesized face(s) border conflicting materials or paint states and were marked Uncertain"
            ));
        }
        if spatial_count > 0 {
            summary.warnings.push(format!(
                "{spatial_count} face(s) used spatial proximity mapping and were marked Uncertain"
            ));
        }

        (next, summary)
    }

    fn resolve_boundary(&self, sources: &[FaceId]) -> (FaceAttributes, bool) {
        if sources.is_empty() {
            return (FaceAttributes::default(), true);
        }
        let samples: Vec<&FaceAttributes> = sources
            .iter()
            .filter_map(|id| self.faces.get(id.0 as usize))
            .collect();
        if samples.is_empty() {
            return (FaceAttributes::default(), true);
        }

        let first = samples[0];
        let material_agrees = samples.iter().all(|s| s.material_id == first.material_id);
        let group_agrees = samples.iter().all(|s| s.group_id == first.group_id);

        let paint_has_split = samples
            .iter()
            .any(|s| s.paint.as_ref().is_some_and(|p| p.is_subdivided_tree));
        let paint_agrees = !paint_has_split
            && samples.iter().all(|s| {
                s.paint.as_ref().and_then(|p| p.decoded_leaf_state)
                    == first.paint.as_ref().and_then(|p| p.decoded_leaf_state)
            });

        let attrs = FaceAttributes {
            material_id: if material_agrees {
                first.material_id
            } else {
                None
            },
            group_id: if group_agrees { first.group_id } else { None },
            paint: if paint_agrees {
                first.paint.clone()
            } else {
                None
            },
        };
        let unanimous = material_agrees && group_agrees && paint_agrees;
        (attrs, unanimous)
    }
}

/// Compose two sequential face-origin maps `(step1_origins, step2_origins)` back to input faces.
///
/// Each element in `step2_origins` refers to faces produced by step 1, and is resolved
/// back to original input `FaceId`s where possible.
#[must_use]
pub fn compose_origins(step1: &[FaceOrigin], step2: &[FaceOrigin]) -> Vec<FaceOrigin> {
    step2
        .iter()
        .map(|o2| match o2 {
            FaceOrigin::Identity(f1) => step1
                .get(f1.0 as usize)
                .cloned()
                .unwrap_or(FaceOrigin::Identity(*f1)),
            FaceOrigin::Split { parent } => match step1.get(parent.0 as usize) {
                Some(FaceOrigin::Identity(orig)) => FaceOrigin::Split { parent: *orig },
                Some(FaceOrigin::Split { parent: orig }) => FaceOrigin::Split { parent: *orig },
                Some(other) => other.clone(),
                None => FaceOrigin::Split { parent: *parent },
            },
            FaceOrigin::Synthesized { boundary_sources } => {
                let mut mapped = Vec::new();
                for s in boundary_sources {
                    match step1.get(s.0 as usize) {
                        Some(FaceOrigin::Identity(orig)) => mapped.push(*orig),
                        Some(FaceOrigin::Split { parent }) => mapped.push(*parent),
                        Some(FaceOrigin::Synthesized {
                            boundary_sources: inner,
                        }) => {
                            mapped.extend(inner.iter().copied());
                        }
                        Some(FaceOrigin::Spatial { nearest, .. }) => mapped.push(*nearest),
                        None => mapped.push(*s),
                    }
                }
                FaceOrigin::Synthesized {
                    boundary_sources: mapped,
                }
            }
            FaceOrigin::Spatial { nearest, distance } => match step1.get(nearest.0 as usize) {
                Some(FaceOrigin::Identity(orig)) => FaceOrigin::Spatial {
                    nearest: *orig,
                    distance: *distance,
                },
                Some(FaceOrigin::Split { parent }) => FaceOrigin::Spatial {
                    nearest: *parent,
                    distance: *distance,
                },
                _ => FaceOrigin::Spatial {
                    nearest: *nearest,
                    distance: *distance,
                },
            },
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_bambu_leaf_paint_codes() {
        assert_eq!(PaintAssignment::from_hex("4").decoded_leaf_state, Some(1));
        assert_eq!(PaintAssignment::from_hex("8").decoded_leaf_state, Some(2));
        assert_eq!(PaintAssignment::from_hex("0C").decoded_leaf_state, Some(3));
        assert_eq!(PaintAssignment::from_hex("1C").decoded_leaf_state, Some(4));
        assert_eq!(PaintAssignment::from_hex("2C").decoded_leaf_state, Some(5));
        let split = PaintAssignment::from_hex("149");
        assert!(split.is_subdivided_tree);
        assert_eq!(split.decoded_leaf_state, None);
    }

    #[test]
    fn boundary_transfer_detects_material_conflict() {
        let mut store = AttributeStore::new_for_mesh(2);
        let red = store.intern_material("red");
        let blue = store.intern_material("blue");
        store.faces[0].material_id = Some(red);
        store.faces[1].material_id = Some(blue);

        let origins = vec![
            FaceOrigin::Identity(FaceId(0)),
            FaceOrigin::Synthesized {
                boundary_sources: vec![FaceId(0), FaceId(1)],
            },
        ];
        let (next, summary) = store.transfer(&origins);
        assert_eq!(summary.exact_count, 1);
        assert_eq!(summary.uncertain_count, 1);
        assert_eq!(next.confidence[0], Confidence::Exact);
        assert_eq!(next.confidence[1], Confidence::Uncertain);
        assert_eq!(next.faces[1].material_id, None);
    }

    #[test]
    fn boundary_transfer_inherits_unanimous_paint() {
        let mut store = AttributeStore::new_for_mesh(2);
        let paint = PaintAssignment::from_hex("0C");
        store.faces[0].paint = Some(paint.clone());
        store.faces[1].paint = Some(paint.clone());

        let origins = vec![FaceOrigin::Synthesized {
            boundary_sources: vec![FaceId(0), FaceId(1)],
        }];
        let (next, summary) = store.transfer(&origins);
        assert_eq!(summary.inherited_count, 1);
        assert_eq!(summary.uncertain_count, 0);
        assert_eq!(next.confidence[0], Confidence::Inherited);
        assert_eq!(next.faces[0].paint, Some(paint));
    }
}
