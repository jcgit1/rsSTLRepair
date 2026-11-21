//! Face correspondence, confidence tracking, and categorical/paint attribute preservation.

use mesh_core::FaceId;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

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
/// `paint_color` (and `slic3rpe:mmu_segmentation`, `paint_supports`, `paint_seam`,
/// `paint_fuzzy_skin`) uses a TriangleSelector nibble bitstream encoded as hexadecimal
/// read from right to left:
/// - low 2 bits == 0: leaf node, state = high 2 bits (`s2 < 3`), or `3 + next_nibble`
///   when `s2 == 3` (escape marker, e.g. `"0C"` -> state 3, `"1C"` -> state 4)
/// - low 2 bits != 0: subdivided triangle split tree
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct PaintAssignment {
    /// Original XML attribute key (e.g. `"paint_color"` or `"slic3rpe:mmu_segmentation"`).
    pub attr_name: String,
    /// Raw hex string from the 3MF attribute, kept verbatim for unchanged faces.
    pub raw_attribute: String,
    /// Decoded leaf filament state (`0` = default extruder, `s >= 1` = filament slot `s - 1`),
    /// or `None` if this face is a subdivided split tree or invalid hex.
    pub decoded_leaf_state: Option<u32>,
    /// True when the raw bitstream describes a multi-color split tree.
    pub is_subdivided_tree: bool,
}

impl PaintAssignment {
    /// Decode a Bambu/Orca `paint_color` hex string using `"paint_color"` as the key.
    #[must_use]
    pub fn from_hex(raw: impl Into<String>) -> Self {
        Self::from_key_hex("paint_color", raw)
    }

    /// Decode a TriangleSelector hex string while preserving its XML attribute key.
    #[must_use]
    pub fn from_key_hex(attr_name: impl Into<String>, raw: impl Into<String>) -> Self {
        let attr_name = attr_name.into();
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
            attr_name,
            raw_attribute,
            decoded_leaf_state,
            is_subdivided_tree,
        }
    }

    /// Encode one unsplit TriangleSelector leaf. State `0` is the default extruder.
    #[must_use]
    pub fn leaf(attr_name: impl Into<String>, state: u32) -> Self {
        let nibbles = encode_leaf_nibbles(state);
        Self::from_key_hex(attr_name, hex_from_bitstream(&nibbles))
    }

    /// Leaf state shared by every leaf of this assignment.
    ///
    /// A subdivided tree returns that state only when every child leaf agrees.
    /// Mixed trees and truncated bitstreams return `None` so repair can mark the
    /// face [`Confidence::Uncertain`] instead of copying the parent hex onto a fragment.
    #[must_use]
    pub fn uniform_leaf_state(&self) -> Option<u32> {
        if !self.is_subdivided_tree {
            return self.decoded_leaf_state;
        }
        let tokens = hex_nibbles(&self.raw_attribute)?;
        let mut index = 0;
        let state = walk_uniform(&tokens, &mut index)?;
        if index == tokens.len() {
            Some(state)
        } else {
            None
        }
    }
}

fn hex_nibbles(raw: &str) -> Option<Vec<u32>> {
    let trimmed = raw.trim();
    if trimmed.is_empty() {
        return None;
    }
    trimmed.chars().rev().map(|c| c.to_digit(16)).collect()
}

fn hex_from_bitstream(nibbles: &[u32]) -> String {
    nibbles
        .iter()
        .rev()
        .map(|n| {
            char::from_digit(*n & 0xF, 16)
                .unwrap_or('0')
                .to_ascii_uppercase()
        })
        .collect()
}

fn encode_leaf_nibbles(mut state: u32) -> Vec<u32> {
    if state < 3 {
        return vec![state << 2];
    }
    let mut extra = Vec::new();
    state -= 3;
    while state >= 15 {
        extra.push(0xF);
        state -= 15;
    }
    extra.push(state);
    let mut nibbles = vec![0xC];
    nibbles.extend(extra);
    nibbles
}

/// Depth-first walk. Children follow the parent nibble, matching Bambu/Orca/Prusa
/// serialization (children are written from the last index down to zero).
fn walk_uniform(tokens: &[u32], index: &mut usize) -> Option<u32> {
    let code = *tokens.get(*index)?;
    *index += 1;
    let splits = code & 0b11;
    if splits == 0 {
        if (code & 0b1100) == 0b1100 {
            let mut next = *tokens.get(*index)?;
            *index += 1;
            let mut escapes = 0u32;
            while next == 0xF {
                escapes += 1;
                next = *tokens.get(*index)?;
                *index += 1;
            }
            Some(next + 15 * escapes + 3)
        } else {
            Some(code >> 2)
        }
    } else {
        let children = splits + 1;
        let mut agreed: Option<u32> = None;
        for _ in 0..children {
            let child = walk_uniform(tokens, index)?;
            match agreed {
                None => agreed = Some(child),
                Some(prev) if prev == child => {}
                Some(_) => return None,
            }
        }
        agreed
    }
}

/// Standard 3MF per-triangle material/property tuple (`pid`, `p1`, optional `p2`, `p3`).
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ThreeMfTriangleMaterial {
    pub pid: String,
    pub p1: String,
    pub p2: Option<String>,
    pub p3: Option<String>,
}

impl ThreeMfTriangleMaterial {
    /// True when all triangle corners reference the same property index (safe to inherit onto caps).
    #[must_use]
    pub fn is_uniform(&self) -> bool {
        let p2_ok = self.p2.as_ref().is_none_or(|p| p == &self.p1);
        let p3_ok = self.p3.as_ref().is_none_or(|p| p == &self.p1);
        p2_ok && p3_ok
    }
}

/// Categorical and paint attributes attached to a single face.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct FaceAttributes {
    pub material_id: Option<u32>,
    pub group_id: Option<u32>,
    pub paint: Option<PaintAssignment>,
    pub paint_supports: Option<PaintAssignment>,
    pub paint_seam: Option<PaintAssignment>,
    pub paint_fuzzy_skin: Option<PaintAssignment>,
    pub threemf_material: Option<ThreeMfTriangleMaterial>,
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
    /// Resolved sRGB filament palette (`0..1` per channel) from 3MF project settings.
    pub filament_palette: Vec<[f32; 3]>,
    /// Default 1-based extruder slot for this object/part (`None` if unspecified).
    pub default_extruder: Option<u32>,
    /// Resolved display colors keyed by `"pid:pindex"` (from `<basematerials>` / `<m:colorgroup>`).
    pub threemf_colors: BTreeMap<String, [f32; 3]>,
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
            filament_palette: Vec::new(),
            default_extruder: None,
            threemf_colors: BTreeMap::new(),
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

    /// Resolve a display color `[0..1; 3]` for face `fi` if filament palette or 3MF colors exist.
    #[must_use]
    pub fn display_color_for_face(&self, fi: usize) -> Option<[f32; 3]> {
        let fa = self.faces.get(fi)?;
        if let Some(paint) = &fa.paint {
            if let Some(state) = paint.decoded_leaf_state {
                let slot_idx = if state == 0 {
                    self.default_extruder.and_then(|e| e.checked_sub(1))
                } else {
                    Some(state - 1)
                };
                if let Some(idx) = slot_idx
                    && let Some(&rgb) = self.filament_palette.get(idx as usize)
                {
                    return Some(rgb);
                }
            } else if paint.is_subdivided_tree {
                return Some([0.82, 0.68, 0.42]);
            }
        }
        if let Some(tm) = &fa.threemf_material {
            let key = format!("{}:{}", tm.pid, tm.p1);
            if let Some(&rgb) = self.threemf_colors.get(&key) {
                return Some(rgb);
            }
        }
        if let Some(ext) = self.default_extruder
            && let Some(idx) = ext.checked_sub(1)
            && let Some(&rgb) = self.filament_palette.get(idx as usize)
        {
            return Some(rgb);
        }
        None
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
            filament_palette: self.filament_palette.clone(),
            default_extruder: self.default_extruder,
            threemf_colors: self.threemf_colors.clone(),
            faces: Vec::with_capacity(new_origins.len()),
            origins: new_origins.to_vec(),
            confidence: Vec::with_capacity(new_origins.len()),
        };
        let mut summary = AttributeTransferSummary::default();
        let mut uncertain_synth = 0usize;
        let mut uncertain_splits = 0usize;
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
                    let (attrs, replayed) = replay_split_attributes(attrs);
                    next.faces.push(attrs);
                    if replayed {
                        next.confidence.push(Confidence::Inherited);
                        summary.inherited_count += 1;
                    } else {
                        next.confidence.push(Confidence::Uncertain);
                        summary.uncertain_count += 1;
                        uncertain_splits += 1;
                    }
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
        if uncertain_splits > 0 {
            summary.warnings.push(format!(
                "{uncertain_splits} split face(s) came from a mixed or truncated paint tree and were marked Uncertain"
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

        let (paint, paint_agrees) = resolve_paint_channel(samples.iter().map(|s| s.paint.as_ref()));
        let (paint_supports, supports_agrees) =
            resolve_paint_channel(samples.iter().map(|s| s.paint_supports.as_ref()));
        let (paint_seam, seam_agrees) =
            resolve_paint_channel(samples.iter().map(|s| s.paint_seam.as_ref()));
        let (paint_fuzzy_skin, fuzzy_agrees) =
            resolve_paint_channel(samples.iter().map(|s| s.paint_fuzzy_skin.as_ref()));

        let tm_agrees = match &first.threemf_material {
            None => samples.iter().all(|s| s.threemf_material.is_none()),
            Some(tm0) => {
                tm0.is_uniform()
                    && samples.iter().all(|s| {
                        s.threemf_material.as_ref().is_some_and(|tm| {
                            tm.is_uniform() && tm.pid == tm0.pid && tm.p1 == tm0.p1
                        })
                    })
            }
        };

        let attrs = FaceAttributes {
            material_id: if material_agrees {
                first.material_id
            } else {
                None
            },
            group_id: if group_agrees { first.group_id } else { None },
            paint,
            paint_supports,
            paint_seam,
            paint_fuzzy_skin,
            threemf_material: if tm_agrees {
                first.threemf_material.clone().map(|mut tm| {
                    tm.p2 = None;
                    tm.p3 = None;
                    tm
                })
            } else {
                None
            },
        };
        let unanimous = material_agrees
            && group_agrees
            && paint_agrees
            && supports_agrees
            && seam_agrees
            && fuzzy_agrees
            && tm_agrees;
        (attrs, unanimous)
    }
}

fn replay_split_attributes(mut attrs: FaceAttributes) -> (FaceAttributes, bool) {
    let mut ok = true;
    attrs.paint = replay_channel(attrs.paint, &mut ok);
    attrs.paint_supports = replay_channel(attrs.paint_supports, &mut ok);
    attrs.paint_seam = replay_channel(attrs.paint_seam, &mut ok);
    attrs.paint_fuzzy_skin = replay_channel(attrs.paint_fuzzy_skin, &mut ok);
    (attrs, ok)
}

fn replay_channel(paint: Option<PaintAssignment>, ok: &mut bool) -> Option<PaintAssignment> {
    let paint = paint?;
    match paint.uniform_leaf_state() {
        Some(state) => Some(PaintAssignment::leaf(paint.attr_name, state)),
        None => {
            *ok = false;
            None
        }
    }
}

fn resolve_paint_channel<'a>(
    mut iter: impl Iterator<Item = Option<&'a PaintAssignment>>,
) -> (Option<PaintAssignment>, bool) {
    let Some(first) = iter.next() else {
        return (None, true);
    };
    let rest: Vec<Option<&'a PaintAssignment>> = iter.collect();
    let has_split = first.is_some_and(|p| p.is_subdivided_tree)
        || rest
            .iter()
            .any(|opt| opt.is_some_and(|p| p.is_subdivided_tree));
    if has_split {
        return (None, false);
    }
    let first_state = first.and_then(|p| p.decoded_leaf_state);
    let agrees = rest
        .iter()
        .all(|opt| opt.and_then(|p| p.decoded_leaf_state) == first_state);
    if agrees {
        (first.cloned(), true)
    } else {
        (None, false)
    }
}

/// Parse `#RRGGBB` or `#RRGGBBAA` hex color into `[r, g, b]` floats in `0.0..=1.0`.
#[must_use]
pub fn parse_srgb_hex(hex: &str) -> Option<[f32; 3]> {
    let h = hex.trim().strip_prefix('#').unwrap_or(hex.trim());
    if h.len() != 6 && h.len() != 8 {
        return None;
    }
    let r = u8::from_str_radix(&h[0..2], 16).ok()?;
    let g = u8::from_str_radix(&h[2..4], 16).ok()?;
    let b = u8::from_str_radix(&h[4..6], 16).ok()?;
    Some([r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0])
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

    #[test]
    fn parses_srgb_palette_and_resolves_paint_color() {
        let mut store = AttributeStore::new_for_mesh(2);
        store.filament_palette = vec![
            parse_srgb_hex("#FF0000FF").expect("red"),
            parse_srgb_hex("#00FF00").expect("green"),
        ];
        store.default_extruder = Some(1);
        store.faces[0].paint = Some(PaintAssignment::from_hex("8")); // slot 2 (index 1)

        assert_eq!(store.display_color_for_face(0), Some([0.0, 1.0, 0.0]));
        assert_eq!(store.display_color_for_face(1), Some([1.0, 0.0, 0.0]));
    }

    #[test]
    fn split_replays_uniform_leaf_and_rejects_mixed_tree() {
        let leaf = PaintAssignment::leaf("paint_color", 1);
        assert_eq!(leaf.raw_attribute, "4");
        assert_eq!(leaf.uniform_leaf_state(), Some(1));
        assert_eq!(PaintAssignment::leaf("paint_color", 3).raw_attribute, "0C");
        assert_eq!(PaintAssignment::leaf("paint_color", 4).raw_attribute, "1C");

        // One split side, two children, both leaf state 1. Bitstream nibbles: 1, 4, 4.
        let uniform = PaintAssignment::from_hex("441");
        assert!(uniform.is_subdivided_tree);
        assert_eq!(uniform.uniform_leaf_state(), Some(1));

        let mixed = PaintAssignment::from_hex("841");
        assert!(mixed.is_subdivided_tree);
        assert_eq!(mixed.uniform_leaf_state(), None);

        let mut store = AttributeStore::new_for_mesh(2);
        store.faces[0].paint = Some(uniform);
        store.faces[1].paint = Some(mixed);
        let (next, summary) = store.transfer(&[
            FaceOrigin::Split { parent: FaceId(0) },
            FaceOrigin::Split { parent: FaceId(1) },
        ]);
        assert_eq!(
            next.faces[0]
                .paint
                .as_ref()
                .map(|p| p.raw_attribute.as_str()),
            Some("4")
        );
        assert_eq!(next.confidence[0], Confidence::Inherited);
        assert!(next.faces[1].paint.is_none());
        assert_eq!(next.confidence[1], Confidence::Uncertain);
        assert_eq!(summary.uncertain_count, 1);
        assert_eq!(summary.warnings.len(), 1);
    }
}
