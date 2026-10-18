// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Pad-entry post-processing.
//!
//! A band ends in a terminal segment that is a tangent from its pad's
//! apex dot to the wrap circle of the band's last gear. When the pad is
//! polygonal, that tangent can cross the pad boundary at a shallow
//! angle, leaving a sliver between the trace edge and the pad edge.
//!
//! A termseg's direction is fully determined by which side of the last
//! gear the band wraps, so the only local repair is topological: undo
//! the terminal cane, redraw it wrapping the same gear in the opposite
//! sense, and re-finish the termseg. This pass detects shallow entries
//! and crowded junctions (three or more band ends sharing a pad dot in
//! near-parallel directions) and attempts that flip where it helps.

use std::collections::BTreeMap;

use geo::{Line, Point};

use crate::{
    board::{edit::BoardDataEdit, AccessMesadata},
    drawing::{
        band::{BandTermsegIndex, BandUid},
        Cane,
        dot::{DotIndex, FixedDotIndex, LooseDotIndex},
        gear::GearIndex,
        primitive::{
            GetCore, GetJoints, GetWeight, LoneLooseSegRef, LooseBendRef, SeqLooseSegRef,
        },
        seg::{LooseSegIndex, SeqLooseSegIndex},
    },
    geometry::{
        edit::{ApplyGeometryEdit, Edit},
        GetSetPos,
    },
    graph::MakeRef,
    layout::{poly::MakePolygon, Layout, LayoutEdit},
    math::intersect_line_and_ray,
    router::draw::Draw,
};

use super::Autorouter;

/// Minimum separation, in radians, between the entry directions of band
/// ends sharing one pad dot before the junction is considered crowded.
const JUNCTION_MIN_ANGLE: f64 = std::f64::consts::FRAC_PI_6;

/// Outcome of the pad-entry post-processing pass.
#[derive(Debug, Default, Clone, Copy)]
pub struct PadEntryStats {
    /// Band ends terminating at a polygonal pad.
    pub polygonal_ends: usize,
    /// Ends entering their pad below the minimum angle.
    pub violations: usize,
    /// Ends whose entry angle was improved.
    pub repairs: usize,
    /// Junctions (three or more ends at one pad dot) with entry
    /// directions separated by less than [`JUNCTION_MIN_ANGLE`].
    pub crowded_junctions: usize,
    /// Junctions whose minimum entry separation was improved.
    pub junction_repairs: usize,
}

/// How a band end's termseg reaches its pad.
#[derive(Clone, Copy)]
enum Approach {
    /// A seq seg: its loose joint is the far joint of the last bend, so
    /// the entry can be flipped by re-wrapping the last gear in the
    /// opposite sense.
    Seq {
        termseg: SeqLooseSegIndex,
        joint: LooseDotIndex,
    },
    /// A lone seg running pad-to-pad: its entry direction is fixed by
    /// the board placement.
    Lone { other: FixedDotIndex },
}

/// One band end terminating at a fixed dot.
#[derive(Clone, Copy)]
struct BandEnd {
    pad: FixedDotIndex,
    /// The pad at the band's other end; identifies the etched path in
    /// the board's band registry.
    other_pad: FixedDotIndex,
    approach: Approach,
}

/// Runs the pad-entry post-processing pass over all bands on the
/// board, recording repairs into `layout_edit`/`board_data_edit`.
///
/// `min_angle` is the minimum angle, in radians, at which a trace may
/// cross its pad's boundary; shallower crossings count as violations.
pub fn improve_pad_entries<M: AccessMesadata>(
    autorouter: &mut Autorouter<M>,
    layout_edit: &mut LayoutEdit,
    board_data_edit: &mut BoardDataEdit,
    principal_layer: usize,
    min_angle: f64,
) -> PadEntryStats {
    let mut stats = PadEntryStats::default();
    let mut ends_by_pad: BTreeMap<FixedDotIndex, Vec<BandEnd>> = BTreeMap::new();

    let bands: Vec<BandUid> = autorouter.board.bands().collect();
    for band in bands {
        for end in band_ends(autorouter.board.layout(), band) {
            ends_by_pad.entry(end.pad).or_default().push(end);
        }
    }

    for end in ends_by_pad.values().flatten() {
        let Some(angle) = pad_entry_angle(autorouter.board.layout(), end.pad, end.approach)
        else {
            continue;
        };
        stats.polygonal_ends += 1;

        if angle < min_angle {
            stats.violations += 1;
            let end = *end;

            if flip_band_end(
                autorouter,
                layout_edit,
                board_data_edit,
                principal_layer,
                end,
                |layout, new_termseg| {
                    termseg_entry_angle(layout, end.pad, new_termseg)
                        .is_some_and(|new_angle| new_angle > angle)
                },
            ) {
                stats.repairs += 1;
            }
        }
    }

    for (pad, ends) in &ends_by_pad {
        if ends.len() < 3 {
            continue;
        }

        let Some(min_gap) = min_entry_separation(autorouter.board.layout(), *pad, ends)
        else {
            continue;
        };

        if min_gap >= JUNCTION_MIN_ANGLE {
            continue;
        }
        stats.crowded_junctions += 1;

        let pad_pos = dot_pos(autorouter.board.layout(), (*pad).into());
        // The approach positions of all other ends are unaffected by a
        // flip, so they can be captured up front.
        let others: Vec<Vec<Point>> = (0..ends.len())
            .map(|i| {
                (0..ends.len())
                    .filter(|j| *j != i)
                    .map(|j| approach_pos(autorouter.board.layout(), ends[j].approach))
                    .collect()
            })
            .collect();

        for (i, end) in ends.iter().enumerate() {
            let others = &others[i];
            let end = *end;

            let flipped = flip_band_end(
                autorouter,
                layout_edit,
                board_data_edit,
                principal_layer,
                end,
                |layout, new_termseg| {
                    let Some(new_approach) =
                        termseg_approach_pos(layout, end.pad, new_termseg)
                    else {
                        return false;
                    };
                    let new_dir = pad_pos - new_approach;
                    let new_gap = others
                        .iter()
                        .map(|other| min_angle_between(pad_pos - *other, new_dir))
                        .fold(f64::INFINITY, f64::min);
                    new_gap > min_gap
                },
            );

            if flipped {
                stats.junction_repairs += 1;
                break;
            }
        }
    }

    stats
}

/// Decomposes a band into its two ends. A seq seg yields one end (its
/// fixed joint); a lone seg yields two (both joints are fixed).
fn band_ends(layout: &Layout<impl AccessMesadata>, band: BandUid) -> Vec<BandEnd> {
    let drawing = layout.drawing();
    let mut ends: Vec<(FixedDotIndex, Approach)> = vec![];

    for termseg in [band[false], band[true]] {
        match termseg {
            LooseSegIndex::Seq(seg) => {
                let (dot, loose) = SeqLooseSegRef::new(seg, drawing).joints();
                if let DotIndex::Fixed(pad) = dot {
                    ends.push((
                        pad,
                        Approach::Seq {
                            termseg: seg,
                            joint: loose,
                        },
                    ));
                }
            }
            LooseSegIndex::Lone(seg) => {
                let (from, to) = LoneLooseSegRef::new(seg, drawing).joints();
                ends.push((from, Approach::Lone { other: to }));
                ends.push((to, Approach::Lone { other: from }));
            }
        }
    }

    if ends.len() != 2 {
        return vec![];
    }

    let other_pad = ends[1].0;
    let this_pad = ends[0].0;
    ends.iter()
        .enumerate()
        .map(|(i, (pad, approach))| {
            let approach = *approach;
            BandEnd {
                pad: *pad,
                other_pad: if i == 0 { other_pad } else { this_pad },
                approach,
            }
        })
        .collect()
}

fn dot_pos(layout: &Layout<impl AccessMesadata>, dot: DotIndex) -> Point {
    layout.drawing().geometry().dot_weight(dot).pos()
}

fn approach_pos(layout: &Layout<impl AccessMesadata>, approach: Approach) -> Point {
    match approach {
        Approach::Seq { joint, .. } => dot_pos(layout, joint.into()),
        Approach::Lone { other, .. } => dot_pos(layout, other.into()),
    }
}

/// The approach point of a freshly drawn termseg as seen from `pad`.
fn termseg_approach_pos(
    layout: &Layout<impl AccessMesadata>,
    pad: FixedDotIndex,
    termseg: BandTermsegIndex,
) -> Option<Point> {
    let drawing = layout.drawing();

    match termseg {
        LooseSegIndex::Seq(seg) => {
            let (_, loose) = SeqLooseSegRef::new(seg, drawing).joints();
            Some(dot_pos(layout, loose.into()))
        }
        LooseSegIndex::Lone(seg) => {
            let (from, to) = LoneLooseSegRef::new(seg, drawing).joints();
            let other = if from == pad { to } else { from };
            Some(dot_pos(layout, other.into()))
        }
    }
}

/// The angle, in radians, at which a band end crosses its pad's polygon
/// boundary, folded into `[0, π/2]`. `None` for pads without a polygonal
/// boundary (bare circular pads, which every entry meets radially) and
/// for entries that never cross the boundary.
fn pad_entry_angle(
    layout: &Layout<impl AccessMesadata>,
    pad: FixedDotIndex,
    approach: Approach,
) -> Option<f64> {
    pad_entry_angle_from(layout, pad, approach_pos(layout, approach))
}

fn termseg_entry_angle(
    layout: &Layout<impl AccessMesadata>,
    pad: FixedDotIndex,
    termseg: BandTermsegIndex,
) -> Option<f64> {
    pad_entry_angle_from(layout, pad, termseg_approach_pos(layout, pad, termseg)?)
}

fn pad_entry_angle_from(
    layout: &Layout<impl AccessMesadata>,
    pad: FixedDotIndex,
    from: Point,
) -> Option<f64> {
    let poly = layout.primitive_poly(pad.into())?;
    let polygon = poly.ref_(layout).shape();
    let pad_pos = dot_pos(layout, pad.into());
    let ray = Line::new(from, pad_pos);

    let mut crossing: Option<(f64, Line)> = None;
    for edge in polygon.exterior().lines() {
        if let Some(pt) = intersect_line_and_ray(&edge, &ray) {
            let delta = pt - from;
            let distance = sq_norm(delta);

            if crossing.is_none_or(|(best, _)| distance < best) {
                crossing = Some((distance, edge));
            }
        }
    }
    let (_, edge) = crossing?;

    let trace = pad_pos - from;
    let edge_dir = edge.end_point() - edge.start_point();
    let cos = (dot(trace, edge_dir).abs() / (sq_norm(trace) * sq_norm(edge_dir)).sqrt())
        .clamp(0.0, 1.0);

    Some(cos.acos())
}

/// The smallest angle between the entry directions of any two band ends
/// at `pad`, in `[0, π]`.
fn min_entry_separation(
    layout: &Layout<impl AccessMesadata>,
    pad: FixedDotIndex,
    ends: &[BandEnd],
) -> Option<f64> {
    let pad_pos = dot_pos(layout, pad.into());

    let directions: Vec<Point> = ends
        .iter()
        .map(|end| pad_pos - approach_pos(layout, end.approach))
        .collect();

    let mut min_gap = f64::INFINITY;
    for (i, dir1) in directions.iter().enumerate() {
        for dir2 in &directions[i + 1..] {
            min_gap = min_gap.min(min_angle_between(*dir1, *dir2));
        }
    }

    min_gap.is_finite().then_some(min_gap)
}

/// The angle between two direction vectors, in `[0, π]`.
fn min_angle_between(dir1: Point, dir2: Point) -> f64 {
    let cos = (dot(dir1, dir2) / (sq_norm(dir1) * sq_norm(dir2)).sqrt()).clamp(-1.0, 1.0);
    cos.acos()
}

/// Flips a band end's pad entry to the last gear's alternate tangent:
/// removes the termseg and the terminal cane, redraws the cane wrapping
/// the same gear in the opposite sense, and finishes a new termseg into
/// the pad. The new geometry is drawn with full infringement and
/// collision checks; the whole attempt is reverted when any step fails
/// or `accept` rejects the result.
///
/// Returns `true` if the end was redrawn and kept.
fn flip_band_end<M: AccessMesadata>(
    autorouter: &mut Autorouter<M>,
    layout_edit: &mut LayoutEdit,
    board_data_edit: &mut BoardDataEdit,
    principal_layer: usize,
    end: BandEnd,
    accept: impl Fn(&Layout<M>, BandTermsegIndex) -> bool,
) -> bool {
    let Approach::Seq { termseg, joint } = end.approach else {
        return false;
    };

    let layout = autorouter.board.layout_mut();

    // The cane whose bend is the termseg's last gear: `joint` is the
    // bend's far joint, so `Cane::from_dot` resolves the cane behind it.
    let cane = Cane::from_dot(joint, layout.drawing());
    let bend_ref = LooseBendRef::new(cane.bend, layout.drawing());
    let joints = bend_ref.joints();

    // `add_cane` places the seg-side joint at `joints.0` for a
    // counterclockwise wrap.
    let sense = if joints.0 == cane.dot {
        crate::math::RotationSense::Clockwise
    } else {
        crate::math::RotationSense::Counterclockwise
    };
    let gear = if let Some(inner) = bend_ref.inner() {
        GearIndex::LooseBend(inner)
    } else {
        GearIndex::FixedDot(bend_ref.core())
    };
    let width = SeqLooseSegRef::new(termseg, layout.drawing())
        .weight()
        .0
        .width;
    let head_tip = layout.drawing().cane_head(joint);

    let mut scratch = LayoutEdit::new();

    layout.remove_termseg(&mut scratch, LooseSegIndex::Seq(termseg));
    let Some(head) = layout.undo_cane(&mut scratch, head_tip) else {
        layout.apply(&scratch.reverse());
        return false;
    };

    let new_head = match gear {
        GearIndex::FixedDot(dot) => layout.cane_around_dot(&mut scratch, head, dot, sense, width),
        GearIndex::LooseBend(inner) => {
            layout.cane_around_bend(&mut scratch, head, inner.into(), sense, width)
        }
        GearIndex::FixedBend(inner) => {
            layout.cane_around_bend(&mut scratch, head, inner.into(), sense, width)
        }
    };
    let Ok((new_head, _)) = new_head else {
        layout.apply(&scratch.reverse());
        return false;
    };

    let Ok(new_termseg) = layout.finish_in_dot(&mut scratch, new_head.into(), end.pad, width)
    else {
        layout.apply(&scratch.reverse());
        return false;
    };

    if !accept(layout, new_termseg) {
        layout.apply(&scratch.reverse());
        return false;
    }

    layout_edit.merge(scratch);

    // The termseg indices embedded in the band's uid and in the
    // ratsnest's recorded ratline termseg are now stale; rebind both to
    // the redrawn band end.
    let new_band = layout
        .drawing()
        .find_loose_band_uid(new_termseg.into())
        .expect("the redrawn band should resolve its uid");
    autorouter.board.update_band_uid(
        board_data_edit,
        end.pad,
        end.other_pad,
        new_band,
    );
    autorouter
        .ratsnests
        .on_principal_layer_mut(principal_layer)
        .rename_band_termseg(LooseSegIndex::Seq(termseg), new_termseg);

    true
}

fn dot(v1: Point, v2: Point) -> f64 {
    v1.x() * v2.x() + v1.y() * v2.y()
}

fn sq_norm(v: Point) -> f64 {
    dot(v, v)
}
