// SPDX-FileCopyrightText: 2026 Topola contributors
//
// SPDX-License-Identifier: MIT

//! The [`AnnealPlan`] → routing-machinery handoff.
//!
//! The plan is advisory: everything it produces is validated against
//! real clearance before it is committed. Dominant layers become a
//! `layer_map`; quantized multi-layer runs pre-place a via at each
//! run boundary and decompose the ratline into continuations —
//! exactly the state `weave_ratline` produces mid-route, committed
//! before anterouting instead; the contestedness order, congestion
//! seed, and corridors follow. Any boundary whose via cannot be
//! placed degrades that split to a plain layer assignment rather than
//! failing the run.

use std::collections::{BTreeMap, BTreeSet};

use geo::Point;
use petgraph::stable_graph::NodeIndex;
use specctra_core::mesadata::AccessMesadata;

use crate::{
    autorouter::{
        anneal::AnnealPlan, carve::carve_corridor,
        multilayer_autoroute::MultilayerAutorouteOptions, ratline::RatlineUid,
        ratsnest::RatvertexNodeIndex, Autorouter,
    },
    board::edit::BoardEdit,
    drawing::{
        dot::FixedDotIndex,
        graph::{GetMaybeNet, MakePrimitiveRef, PrimitiveIndex},
    },
    geometry::{edit::Edit, GetLayer},
    graph::MakeRef,
    layout::via::ViaWeight,
    math::Circle,
};

/// One planned layer hop, committed as a weave-shaped record so the
/// stepper's woven-anchoring and unweave machinery treats it exactly
/// like a mid-route weave.
pub struct PlannedWeave {
    pub parent: RatlineUid,
    pub continuation: RatlineUid,
    pub via_vertex: NodeIndex<usize>,
    /// Every dot the placed via owns, one per spanned layer —
    /// `adopt_planned_weaves` picks the one on the layer the parent
    /// actually terminates on, which may differ from `from_layer`
    /// when the anteroute fell back to a preferred layer.
    pub via_dots: Vec<FixedDotIndex>,
    pub via_dot_source: FixedDotIndex,
    pub via_dot_target: FixedDotIndex,
    pub target_layer: usize,
    pub destination_dot: FixedDotIndex,
    /// The via-placement edit, merged under `continuation`'s
    /// anteroute edits — it owns the via, as in `weave_ratline`.
    pub via_edit: BoardEdit,
}

/// Everything `apply` commits or prepares for the caller to consume.
pub struct HandoffResult {
    /// Layer assignments for every ratline including continuations —
    /// fed to `MultilayerPreconfigurer::new_from_layer_map`.
    pub layer_map: BTreeMap<RatlineUid, usize>,
    /// Planned splits as weave records for the stepper.
    pub weaves: Vec<PlannedWeave>,
    /// Contestedness order with continuations spliced after their
    /// parents.
    pub order: Vec<RatlineUid>,
    /// Every edit committed during handoff (vias + carves), merged
    /// into the stepper's `anteroute_edit` for history fidelity.
    pub edit: BoardEdit,
}

/// Applies `plan` to the board and ratline set: places split vias,
/// adds continuation ratlines to `autorouter.ratsnests` and
/// `ratlines`, seeds congestion, carves the top contested corridors.
/// Board edits land in `result.edit` for the caller to merge into the
/// command's history.
pub fn apply(
    autorouter: &mut Autorouter<impl AccessMesadata>,
    ratlines: &mut BTreeSet<RatlineUid>,
    plan: AnnealPlan,
    options: &MultilayerAutorouteOptions,
) -> HandoffResult {
    let mut result = HandoffResult {
        layer_map: plan.layer_map.clone(),
        weaves: Vec::new(),
        order: Vec::new(),
        edit: BoardEdit::new(),
    };

    // Order bookkeeping: continuation chains splice in after their
    // parent when the contestedness order is emitted below.
    let mut continuations_of: BTreeMap<RatlineUid, Vec<RatlineUid>> = BTreeMap::new();

    for split in &plan.splits {
        result.layer_map.insert(split.parent, split.runs[0].0);
        if let Some(work) = place_split(
            autorouter,
            ratlines,
            split,
            options.planar.router.routed_band_width,
        ) {
            result.layer_map.extend(work.layer_map);
            result.weaves.extend(work.weaves);
            result.edit.merge(work.edit);
            continuations_of.insert(split.parent, work.chain);
        }
        // On `None` the split degraded inside `place_split` — board
        // and ratsnest are already reverted; the parent keeps
        // run[0]'s layer as a plain assignment.
    }

    // Congestion seed: annealer near-miss counts become the
    // negotiated-congestion record's starting state. Written as
    // absolute counts rather than via `seed_congestion`'s increments —
    // the map persists on the `Autorouter` across undo/replay, and a
    // second replay would otherwise double-seed and take different
    // routing decisions than the first.
    for (primitive, count) in &plan.congestion {
        if matches!(
            primitive,
            PrimitiveIndex::FixedDot(_)
                | PrimitiveIndex::FixedSeg(_)
                | PrimitiveIndex::FixedBend(_)
        ) {
            autorouter.congestion_seed.insert(*primitive, *count);
        }
    }

    // Corridor carving: only the most contested *unsplit* ratlines —
    // a split parent's corridor spans layers and its vias already
    // pin the corridor, so carving it would double-constrain.
    let split_parents: BTreeSet<RatlineUid> = continuations_of.keys().copied().collect();
    for ratline in plan
        .order
        .iter()
        .take(options.anneal.map_or(0, |a| a.carve_top_k))
    {
        if split_parents.contains(ratline) {
            continue;
        }
        let Some(corridor) = plan.corridors.get(ratline) else {
            continue;
        };
        if let Some(edit) = carve_corridor(
            autorouter,
            *ratline,
            corridor,
            options.planar.router.routed_band_width,
        ) {
            result.edit.merge(edit);
        }
    }

    result.order = plan
        .order
        .iter()
        .flat_map(|ratline| {
            std::iter::once(*ratline)
                .chain(continuations_of.get(ratline).into_iter().flatten().copied())
        })
        .collect();
    result
}

/// One boundary's via-site candidates: the site itself plus a
/// deterministic fan of perpendicular/along-track offsets scaled to
/// the corridor width — same "try a few spots" shape as
/// `weave_ratline`'s source-biased candidates.
fn via_site_candidates(site: Point, along: Point, width: f64) -> Vec<Point> {
    let span = along.x().hypot(along.y()).max(1.0);
    let unit = Point::new(along.x() / span, along.y() / span);
    let perp = Point::new(-unit.y(), unit.x());
    let step = (2.0 * width).max(1.0);
    let mut candidates = vec![site];
    for scale in [1.0, -1.0, 2.0, -2.0] {
        candidates.push(site + perp * (scale * step));
    }
    for scale in [1.0, -1.0] {
        candidates.push(site + unit * (scale * step));
    }
    candidates
}

/// `place_split`'s pending output — merged into the `HandoffResult`
/// only when every boundary committed.
struct SplitWork {
    layer_map: BTreeMap<RatlineUid, usize>,
    weaves: Vec<PlannedWeave>,
    chain: Vec<RatlineUid>,
    edit: BoardEdit,
}

/// Places a split's boundary vias and continuation ratlines. On any
/// failure reverses the split's partial work — continuations out of
/// the ratsnest, committed boundary edits reversed — and returns
/// `None`, leaving the parent a plain single-layer assignment. The
/// reversal is not merged into the result edit: pre-configuration
/// edits have no undo stream to stay faithful to (contrast the
/// weave path, which merges both directions because it runs inside
/// the live command history).
fn place_split(
    autorouter: &mut Autorouter<impl AccessMesadata>,
    ratlines: &mut BTreeSet<RatlineUid>,
    split: &super::AnnealSplit,
    band_width: f64,
) -> Option<SplitWork> {
    let parent = split.parent;
    let destination_vertex = parent.ref_(autorouter).endpoint_indices().1;
    let destination_dot = parent.ref_(autorouter).endpoint_dots().1;
    let maybe_net = parent
        .ref_(autorouter)
        .endpoint_dots()
        .0
        .primitive_ref(autorouter.board().layout().drawing())
        .maybe_net();

    let mut work = SplitWork {
        layer_map: BTreeMap::new(),
        weaves: Vec::new(),
        chain: Vec::new(),
        edit: BoardEdit::new(),
    };
    // (vertex, edge) of each added continuation, for rollback.
    let mut added: Vec<(NodeIndex<usize>, petgraph::stable_graph::EdgeIndex<usize>)> = Vec::new();
    let mut previous = parent;

    for boundary in 0..split.via_sites.len() {
        let from_layer = split.runs[boundary].0;
        let to_layer = split.runs[boundary + 1].0;
        let site = split.via_sites[boundary];
        // Track direction at the boundary for the offset fan.
        let along = {
            let run = &split.runs[boundary].1;
            let next = &split.runs[boundary + 1].1;
            let a = run.last().copied().unwrap_or(site);
            let b = next.first().copied().unwrap_or(site);
            b - a
        };

        let mut placed: Option<(Point, Vec<FixedDotIndex>)> = None;
        let mut boundary_edit = BoardEdit::new();
        'sites: for pos in via_site_candidates(site, along, band_width) {
            if let Ok((_via, dots)) = autorouter.board.add_via(
                &mut boundary_edit,
                ViaWeight {
                    from_layer: from_layer.min(to_layer),
                    to_layer: from_layer.max(to_layer),
                    circle: Circle {
                        pos,
                        r: autorouter.board().mesadata().default_via_radius(),
                    },
                    maybe_net,
                },
                None,
            ) {
                placed = Some((pos, dots));
                break 'sites;
            }
        }
        let Some((via_pos, via_dots)) = placed else {
            rollback(
                autorouter,
                ratlines,
                parent,
                &added,
                &boundary_edit,
                &work.edit,
            );
            return None;
        };

        let dot_on = |layer: usize| {
            via_dots.iter().copied().find(|dot| {
                dot.primitive_ref(autorouter.board().layout().drawing())
                    .layer()
                    == layer
            })
        };
        let (Some(via_dot_source), Some(via_dot_target)) = (dot_on(from_layer), dot_on(to_layer))
        else {
            rollback(
                autorouter,
                ratlines,
                parent,
                &added,
                &boundary_edit,
                &work.edit,
            );
            return None;
        };

        // The continuation: via's far-side dot → the original
        // destination, exactly `weave_ratline`'s decomposition.
        let (via_vertex, continuation_edge) = autorouter
            .ratsnests
            .on_principal_layer_mut(parent.principal_layer)
            .add_continuation(
                RatvertexNodeIndex::FixedDot(via_dot_target),
                via_pos,
                destination_vertex,
            );
        let continuation = RatlineUid {
            principal_layer: parent.principal_layer,
            index: continuation_edge,
        };
        ratlines.insert(continuation);
        added.push((via_vertex, continuation_edge));
        work.chain.push(continuation);

        // The previous run's ratline retargets its destination
        // endpoint at the via's near-side dot — committed later by
        // `adopt_planned_weaves` as a `remap_terminating_dots` over
        // the ordinary anteroute result, exactly like
        // `weave_ratline`, so an unweave can restore the pre-split
        // terminating dot. The continuation's via-side endpoint
        // terminates at itself — `new_from_layer_map`'s scan finds
        // it because the dot already lies on `to_layer`.

        work.layer_map.insert(continuation, to_layer);
        work.edit.merge(boundary_edit.clone());
        work.weaves.push(PlannedWeave {
            parent: previous,
            continuation,
            via_vertex,
            via_dots: via_dots.clone(),
            via_dot_source,
            via_dot_target,
            target_layer: to_layer,
            destination_dot,
            via_edit: boundary_edit,
        });
        previous = continuation;
    }

    Some(work)
}

/// Undoes `place_split`'s partial work: removes the added
/// continuations (all on the parent's principal-layer ratsnest) and
/// reverses the committed boundary edits plus the failed attempt's.
fn rollback(
    autorouter: &mut Autorouter<impl AccessMesadata>,
    ratlines: &mut BTreeSet<RatlineUid>,
    parent: RatlineUid,
    added: &[(NodeIndex<usize>, petgraph::stable_graph::EdgeIndex<usize>)],
    boundary_edit: &BoardEdit,
    committed_edit: &BoardEdit,
) {
    for (vertex, edge) in added.iter().rev() {
        autorouter
            .ratsnests
            .on_principal_layer_mut(parent.principal_layer)
            .remove_continuation(*vertex, *edge);
        ratlines.remove(&RatlineUid {
            principal_layer: parent.principal_layer,
            index: *edge,
        });
    }
    // Reverse in chronological reverse order: the failed boundary
    // attempt landed after every committed boundary edit.
    autorouter.board.apply_edit(&boundary_edit.reverse());
    autorouter.board.apply_edit(&committed_edit.reverse());
}
