// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Corridor carving: same-net fixed geometry that physically reserves a
//! critical ratline's demand corridor before ordinary routing fills in
//! around it.
//!
//! A carve is a chain of fixed dots and segs along the probed demand
//! path, on the ratline's net and layer(s). Because fixed geometry on
//! the same net is *connectable* — never a blocker — the carve costs its
//! own ratline nothing, while foreign nets must keep clearance from it:
//! the corridor is reserved. And because the geometry is real copper,
//! the reservation survives reconfiguration and also accrues
//! obstacle-penalty pressure against foreign bands that do try to wrap
//! it — a self-reinforcing reservation.
//!
//! Carves deliberately stop short of the endpoint pads: the corridor's
//! interior is what needs defending (pad-adjacent space is escape
//! stubs' job), and keeping carve dots out of pad wrap chains means a
//! dead carve is almost never load-bearing, so removal is clean.

use std::collections::BTreeMap;

use geo::Point;
use petgraph::visit::Walker;

use crate::{
    autorouter::{ratline::RatlineUid, Autorouter},
    board::{
        edit::{BoardDataEdit, BoardEdit},
        AccessMesadata,
    },
    drawing::{
        bend::BendIndex,
        dot::{DotIndex, FixedDotIndex, FixedDotWeight, GeneralDotWeight},
        gear::{GearIndex, WalkOutwards},
        graph::{GetMaybeNet, MakePrimitiveRef},
        primitive::GetLimbs,
        seg::{FixedSegWeight, GeneralSegWeight, SegIndex},
        Drawing,
    },
    geometry::edit::{ApplyGeometryEdit, Edit},
    graph::{GetIndex, MakeRef},
    layout::LayoutEdit,
    math::Circle,
    metrics::{incr, METRICS},
};

/// Lays a same-net fixed dot/seg chain along `path` (a demand polyline
/// of `(center, layer)` waypoints, as produced by a channel probe).
///
/// The probe path is densified first: interpolated carve positions at
/// roughly `width` spacing, since a probe band with few bends returns
/// only its endpoints and an undensified carve would have no interior
/// to reserve. Positions within `2 * width` of either endpoint pad are
/// skipped — the carve reserves the corridor's interior while pad
/// wrap chains stay untouched, so a dead carve is almost never
/// load-bearing and removal stays clean.
///
/// Returns the [`BoardEdit`] recording what was placed — the caller
/// keeps it for later selective removal. Returns `None` when nothing
/// could be placed (e.g. a degenerate or fully pad-adjacent path).
///
/// Placement failures on individual primitives are tolerated — a seg
/// that cannot be placed sits over foreign fixed geometry, which needs
/// no defending anyway — but are counted in `carve_segs_failed`.
pub fn carve_corridor(
    autorouter: &mut Autorouter<impl AccessMesadata>,
    ratline: RatlineUid,
    path: &[(Point, usize)],
    width: f64,
) -> Option<BoardEdit> {
    if path.len() < 2 {
        return None;
    }

    let maybe_net = {
        let (origin, _) = ratline.ref_(autorouter).endpoint_dots();
        origin
            .primitive_ref(autorouter.board().layout().drawing())
            .maybe_net()
    };

    // Densify the demand path: walk each window, emitting interpolated
    // positions at `2 * width` spacing (including each window's
    // endpoint, which carries the polyline's corners). Two band widths
    // is the coarsest spacing that still forms a continuous keep-out
    // wall — foreign bands must keep clearance off every carve prim —
    // and matters: every carve dot and seg lands in every navmesh the
    // router builds afterwards, so carving at band-width granularity
    // would strangle routing on large boards.
    let spacing = 2.0 * width;
    let mut positions: Vec<(Point, usize)> = Vec::new();
    for window in path.windows(2) {
        let (a, layer_a) = window[0];
        let (b, layer_b) = window[1];
        if layer_a != layer_b {
            // Layer transition within the demand path: the carve chain
            // breaks (the probe's via substitute is not carved); the
            // next run's first position is still emitted below by the
            // following window.
            continue;
        }
        let delta = b - a;
        let len = (delta.x() * delta.x() + delta.y() * delta.y()).sqrt();
        let intervals = (len / spacing).ceil().max(1.0) as usize;
        for k in 1..=intervals {
            positions.push((a + delta * (k as f64 / intervals as f64), layer_a));
        }
    }

    let layout = autorouter.board.layout_mut();
    let mut scratch = LayoutEdit::new();
    let mut placed_any = false;

    // No carve within two band widths of an endpoint pad: keeps carve
    // dots out of pad wrap chains.
    let end_margin = 2.0 * width;
    let endpoints = (path[0].0, path[path.len() - 1].0);
    let near_end = |pos: Point| {
        let d0 = pos - endpoints.0;
        let d1 = pos - endpoints.1;
        d0.x() * d0.x() + d0.y() * d0.y() < end_margin * end_margin
            || d1.x() * d1.x() + d1.y() * d1.y() < end_margin * end_margin
    };

    let mut prev_dot: Option<(FixedDotIndex, usize)> = None;
    for (pos, layer) in positions {
        let dot = (!near_end(pos))
            .then(|| {
                layout.add_fixed_dot(
                    &mut scratch,
                    FixedDotWeight(GeneralDotWeight {
                        circle: Circle { pos, r: width },
                        layer,
                        maybe_net,
                    }),
                )
            })
            .and_then(Result::ok);

        if let (Some((prev, prev_layer)), Some(dot)) = (prev_dot, dot) {
            if prev_layer == layer {
                if layout
                    .add_fixed_seg(
                        &mut scratch,
                        prev,
                        dot,
                        FixedSegWeight(GeneralSegWeight {
                            width,
                            layer,
                            maybe_net,
                        }),
                    )
                    .is_ok()
                {
                    incr(&METRICS.carve_segs_placed);
                } else {
                    incr(&METRICS.carve_segs_failed);
                }
            }
        }

        prev_dot = dot.map(|dot| (dot, layer));
        placed_any |= dot.is_some();
    }

    if !placed_any {
        layout.apply(&scratch.reverse());
        return None;
    }

    Some(BoardEdit::new_from_edits(BoardDataEdit::new(), scratch))
}

/// Whether any primitive `edit` created is load-bearing for a band the
/// edit did not create — a created dot with outward wraps or foreign
/// seg/bend limbs, or a created bend with outward wraps (limbs the edit
/// itself created, like its own chain segs, don't count).
///
/// Shared by carve removal and the multilayer reconfigurator's
/// fanout-via removal: geometry another band has wrapped cannot be
/// removed without invalidating that band.
pub fn edit_is_wrapped<CW, Cel, R>(drawing: &Drawing<CW, Cel, R>, edit: &LayoutEdit) -> bool
where
    CW: Clone,
    Cel: Copy,
    R: crate::drawing::rules::AccessRules,
{
    let created_segs: std::collections::BTreeSet<SegIndex> = edit.created_segs().collect();
    let created_bends: std::collections::BTreeSet<BendIndex> = edit.created_bends().collect();
    let geometry = drawing.geometry();

    edit.created_dots().any(|dot| {
        let DotIndex::Fixed(dot) = dot else {
            return false;
        };
        // A removed-but-restorable seg or bend joined to this dot is
        // invisible to the live-geometry checks below, yet a pending
        // reversal replays the join — the dot is still load-bearing
        // and removing it would orphan that restoration.
        if geometry.has_pending_joiners(dot.index()) {
            incr(&METRICS.pending_joiners_held);
            return true;
        }
        let dot_ref = drawing.primitive_ref(dot);
        let mut outwards = dot_ref.outwards();
        outwards.walk_next(drawing).is_some()
            || dot_ref.segs().iter().any(|seg| !created_segs.contains(seg))
            || dot_ref
                .bends()
                .iter()
                .any(|bend| !created_bends.contains(bend))
    }) || edit.created_bends().any(|bend| {
        if geometry.has_pending_joiners(bend.index()) {
            incr(&METRICS.pending_joiners_held);
            return true;
        }
        let mut outwards = GearIndex::from(bend).ref_(drawing).outwards();
        outwards.walk_next(drawing).is_some()
    })
}

/// Removes every carve edit that no foreign band depends on, returning
/// the removal's edit record. Carves that have become load-bearing
/// (`edit_is_wrapped`) are kept — they are same-net copper and remain
/// electrically valid — and counted in `carves_kept_wrapped`.
pub fn remove_dead_carves(
    autorouter: &mut Autorouter<impl AccessMesadata>,
    edits: &mut BTreeMap<RatlineUid, BoardEdit>,
) -> BoardEdit {
    let mut recorder = BoardEdit::new();

    edits.retain(|_, edit| {
        if edit_is_wrapped(autorouter.board().layout().drawing(), &edit.layout_edit) {
            incr(&METRICS.carves_kept_wrapped);
            return true;
        }

        let reverse = edit.reverse();
        autorouter.board.apply_edit(&reverse);
        recorder.merge(edit.clone());
        recorder.merge(reverse);
        incr(&METRICS.carves_removed);
        false
    });

    recorder
}

#[cfg(test)]
mod tests {
    use std::{fs::File, io::BufReader};

    use geo::point;
    use specctra_core::mesadata::SpecctraMesadata;

    use super::*;
    use crate::{
        board::edit::BoardEdit, drawing::seg::FixedSegWeight, geometry::edit::ApplyGeometryEdit,
        specctra::design::SpecctraDesign,
    };

    fn autorouter_0603() -> Autorouter<SpecctraMesadata> {
        let design_file = File::open("tests/unilayer/0603_breakout/0603_breakout.dsn").unwrap();
        let design = SpecctraDesign::load(BufReader::new(design_file)).unwrap();
        Autorouter::new(design.make_board(&mut BoardEdit::new())).unwrap()
    }

    /// Regression test for the stale-endpoint class of geometry
    /// corruption: a band seg joined to a foreign fixed dot — e.g. a
    /// fanout via's — is removed while its removal is still
    /// reversible. The graph shows no join (the seg is tombstoned),
    /// but a pending reversal replays it, so `edit_is_wrapped` must
    /// treat the dot as load-bearing and keep it. Without the pending
    /// joiner tracking, the removal went through and the eventual
    /// backtrace replayed a join onto a dead endpoint.
    #[test]
    fn wrapped_check_sees_pending_joiners() {
        let mut autorouter = autorouter_0603();
        let layout = autorouter.board.layout_mut();

        let dot_w = |x: f64| {
            FixedDotWeight(GeneralDotWeight {
                circle: Circle {
                    pos: point!(x: x, y: -9000.0),
                    r: 100.0,
                },
                layer: 0,
                maybe_net: Some(90),
            })
        };

        // The removable edit — stands in for a fanout via's creation
        // edit; only its fixed dot matters for the check.
        let mut via_edit = LayoutEdit::new();
        let via_dot = layout.add_fixed_dot_infringably(&mut via_edit, dot_w(0.0));

        // A foreign band: its own endpoint dot and a seg joined to
        // the removable dot.
        let mut band_edit = LayoutEdit::new();
        let band_dot = layout.add_fixed_dot_infringably(&mut band_edit, dot_w(800.0));
        layout
            .add_fixed_seg(
                &mut band_edit,
                band_dot,
                via_dot,
                FixedSegWeight(GeneralSegWeight {
                    width: 100.0,
                    layer: 0,
                    maybe_net: Some(90),
                }),
            )
            .unwrap();

        // Live join visible: the dot is already load-bearing.
        assert!(edit_is_wrapped(layout.drawing(), &via_edit));

        // Rip the band but keep its edit reversible: the seg is
        // tombstoned and its join vanishes from the graph, yet the
        // recorded edit still expects the endpoint when replayed.
        layout.apply(&band_edit.reverse());
        assert!(edit_is_wrapped(layout.drawing(), &via_edit));

        // With the dot kept alive, restoring the band rejoins the seg
        // cleanly — before the fix this either corrupted the slot or
        // tripped the endpoint liveness assertion.
        layout.apply(&band_edit);
        assert!(edit_is_wrapped(layout.drawing(), &via_edit));
    }
}
