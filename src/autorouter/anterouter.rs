// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::collections::BTreeMap;

use derive_getters::Getters;
use geo::{point, Distance, Euclidean, Point};
use petgraph::graph::NodeIndex;
use rstar::{Envelope, RTreeObject, AABB};
use serde::{Deserialize, Serialize};
use specctra_core::mesadata::AccessMesadata;
use thiserror::Error;

use crate::{
    autorouter::{
        compass_direction::{CardinalDirection, CompassDirection, OrdinalDirection},
        ratline::RatlineUid,
        Autorouter,
    },
    board::edit::BoardEdit,
    drawing::{
        dot::FixedDotIndex,
        graph::{GetMaybeNet, MakePrimitiveRef},
        primitive::MakePrimitiveShape,
        seg::{FixedSegWeight, GeneralSegWeight},
    },
    geometry::{shape::AccessShape, GenericNode, GetLayer},
    graph::{GenericIndex, MakeRef},
    layout::{poly::MakePolygon, via::ViaWeight},
    math::Circle,
};

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct AnterouterOptions {
    pub fanout_clearance: f64,
    /// Extra outward rings retried when the innermost fanout-via ring
    /// is blocked. Each ring grows the search bbox by a via footprint.
    /// 0 keeps the innermost-ring-only behavior.
    #[serde(default)]
    pub fanout_rings: usize,
}

/// An endpoint's fanout via or seg could not be placed — the
/// ratline's anteroute is incomplete. Callers either fall back to
/// another layer or drop the ratline from the configuration.
#[derive(Clone, Copy, Debug, Error)]
#[error("anteroute fanout placement failed")]
pub struct AnterouteFailure;

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct AnterouterPlan {
    pub layer_map: BTreeMap<RatlineUid, usize>,
    pub static_terminating_dot_map: BTreeMap<(RatlineUid, FixedDotIndex, usize), FixedDotIndex>,
}

#[derive(Getters)]
pub struct Anterouter {
    plan: AnterouterPlan,
}

impl Anterouter {
    pub fn new(plan: AnterouterPlan) -> Self {
        Self { plan }
    }

    pub fn anteroute(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
        recorder: &mut BoardEdit,
        options: &AnterouterOptions,
    ) -> BTreeMap<(RatlineUid, FixedDotIndex), FixedDotIndex> {
        let mut terminating_dot_map = BTreeMap::new();

        // PERF: Unnecessary clone.
        for (ratline, layer) in self.plan.layer_map.clone().iter() {
            self.anteroute_ratline(
                autorouter,
                recorder,
                *ratline,
                *layer,
                options,
                &mut terminating_dot_map,
            )
            .expect("fanout via placement failed");
        }

        terminating_dot_map
    }

    /// Anteroutes one ratline to `layer`: for each endpoint that is not
    /// already statically terminating on the layer, places a fanout via
    /// and a fixed seg off the pad, and records the endpoint's remap to
    /// the via's target-layer dot in `terminating_dot_map`. Used both
    /// by the initial anteroute pass and by multilayer re-layering,
    /// which re-anteroutes only the ratlines whose layer changed.
    ///
    /// Returns `Err` when an endpoint's fanout could not be placed;
    /// whatever the other endpoint placed stays, and its terminating
    /// remap is still recorded.
    pub fn anteroute_ratline(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
        recorder: &mut BoardEdit,
        ratline: RatlineUid,
        layer: usize,
        options: &AnterouterOptions,
        terminating_dot_map: &mut BTreeMap<(RatlineUid, FixedDotIndex), FixedDotIndex>,
    ) -> Result<(), AnterouteFailure> {
        let endpoint_indices = ratline.ref_(autorouter).endpoint_indices();
        let endpoint_dots = ratline.ref_(autorouter).endpoint_dots();
        let mut result = Ok(());

        for endpoint in [0, 1] {
            let endpoint_index = [endpoint_indices.0, endpoint_indices.1][endpoint];
            let endpoint_dot = [endpoint_dots.0, endpoint_dots.1][endpoint];

            if let Some(terminating_dot) =
                self.plan
                    .static_terminating_dot_map
                    .get(&(ratline, endpoint_dot, layer))
            {
                terminating_dot_map.insert((ratline, endpoint_dot), *terminating_dot);
            } else if self
                .anteroute_fanout(
                    autorouter,
                    recorder,
                    endpoint_index,
                    ratline,
                    endpoint_dot,
                    layer,
                    options,
                    terminating_dot_map,
                )
                .is_err()
            {
                result = Err(AnterouteFailure);
            }
        }

        result
    }

    fn anteroute_fanout(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
        recorder: &mut BoardEdit,
        ratvertex: NodeIndex<usize>,
        ratline: RatlineUid,
        source_dot: FixedDotIndex,
        target_layer: usize,
        options: &AnterouterOptions,
        terminating_dot_map: &mut BTreeMap<(RatlineUid, FixedDotIndex), FixedDotIndex>,
    ) -> Result<(), AnterouteFailure> {
        let mut ratline_delta: Point = ratline.ref_(autorouter).line_segment().delta().into();

        if ratvertex == ratline.ref_(autorouter).endpoint_indices().1 {
            ratline_delta = -ratline_delta;
        }

        let small_bbox = if let Some(poly) = autorouter
            .board()
            .layout()
            .primitive_poly(source_dot.into())
        {
            let bbox = poly.ref_(autorouter.board().layout()).shape().envelope();
            AABB::<[f64; 2]>::from_corners(
                [bbox.lower().x(), bbox.lower().y()],
                [bbox.upper().x(), bbox.upper().y()],
            )
        } else {
            autorouter
                .board()
                .layout()
                .drawing()
                .primitive_ref(source_dot)
                .shape()
                .envelope()
        };

        if self
            .anteroute_fanout_on_bbox(
                autorouter,
                recorder,
                ratvertex,
                ratline,
                source_dot,
                small_bbox,
                target_layer,
                CardinalDirection::nearest_from_vector(ratline_delta),
                options,
                terminating_dot_map,
            )
            .is_ok()
        {
            return Ok(());
        }

        let mut large_bbox = small_bbox;

        for enclosing_poly in autorouter.board().layout().polys_enclosing_point(
            source_dot
                .primitive_ref(autorouter.board().layout().drawing())
                .shape()
                .center(),
        ) {
            let enclosing_bbox = enclosing_poly
                .ref_(autorouter.board().layout())
                .shape()
                .envelope();
            large_bbox.merge(&AABB::<[f64; 2]>::from_corners(
                [enclosing_bbox.lower().x(), enclosing_bbox.lower().y()],
                [enclosing_bbox.upper().x(), enclosing_bbox.upper().y()],
            ));
        }

        if self
            .anteroute_fanout_on_bbox(
                autorouter,
                recorder,
                ratvertex,
                ratline,
                source_dot,
                large_bbox,
                target_layer,
                CardinalDirection::nearest_from_vector(ratline_delta),
                options,
                terminating_dot_map,
            )
            .is_ok()
        {
            return Ok(());
        }

        if self
            .anteroute_fanout_on_bbox(
                autorouter,
                recorder,
                ratvertex,
                ratline,
                source_dot,
                small_bbox,
                target_layer,
                OrdinalDirection::nearest_from_vector(ratline_delta),
                options,
                terminating_dot_map,
            )
            .is_ok()
        {
            return Ok(());
        }

        if self
            .anteroute_fanout_on_bbox(
                autorouter,
                recorder,
                ratvertex,
                ratline,
                source_dot,
                large_bbox,
                target_layer,
                OrdinalDirection::nearest_from_vector(ratline_delta),
                options,
                terminating_dot_map,
            )
            .is_ok()
        {
            return Ok(());
        }

        // Ring fallback: every direction sweep so far anchored the via
        // one clearance past the pad's bbox edge. When that innermost
        // ring is fully blocked, retry the sweeps on bboxes grown
        // outward in via-diameter steps — a via a few diameters away
        // from the pad can clear the neighbor that blocked the inner
        // position. This only runs once the inner placement failed, so
        // previously-working fanouts are unaffected.
        let maybe_net = source_dot
            .primitive_ref(autorouter.board().layout().drawing())
            .maybe_net();
        let ring_step = 2.0 * autorouter.board().mesadata().default_via_radius()
            + options.fanout_clearance.max(
                maybe_net
                    .map(|net| {
                        autorouter
                            .board()
                            .layout()
                            .rules()
                            .largest_clearance(Some(net))
                    })
                    .unwrap_or(0.0),
            );
        let grow = |bbox: AABB<[f64; 2]>, distance: f64| {
            AABB::<[f64; 2]>::from_corners(
                [bbox.lower()[0] - distance, bbox.lower()[1] - distance],
                [bbox.upper()[0] + distance, bbox.upper()[1] + distance],
            )
        };

        for ring in 1..=options.fanout_rings {
            let grown = grow(small_bbox, ring_step * ring as f64);
            if self
                .anteroute_fanout_on_bbox(
                    autorouter,
                    recorder,
                    ratvertex,
                    ratline,
                    source_dot,
                    grown,
                    target_layer,
                    CardinalDirection::nearest_from_vector(ratline_delta),
                    options,
                    terminating_dot_map,
                )
                .is_ok()
            {
                return Ok(());
            }
            if self
                .anteroute_fanout_on_bbox(
                    autorouter,
                    recorder,
                    ratvertex,
                    ratline,
                    source_dot,
                    grown,
                    target_layer,
                    OrdinalDirection::nearest_from_vector(ratline_delta),
                    options,
                    terminating_dot_map,
                )
                .is_ok()
            {
                return Ok(());
            }
        }

        Err(AnterouteFailure)
    }

    fn anteroute_fanout_on_bbox(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
        recorder: &mut BoardEdit,
        ratvertex: NodeIndex<usize>,
        ratline: RatlineUid,
        source_dot: FixedDotIndex,
        bbox: AABB<[f64; 2]>,
        target_layer: usize,
        preferred_compass_direction: impl CompassDirection,
        options: &AnterouterOptions,
        terminating_dot_map: &mut BTreeMap<(RatlineUid, FixedDotIndex), FixedDotIndex>,
    ) -> Result<(), AnterouteFailure> {
        if self
            .anteroute_fanout_on_bbox_in_direction(
                autorouter,
                recorder,
                ratvertex,
                ratline,
                source_dot,
                bbox,
                target_layer,
                preferred_compass_direction,
                options,
                terminating_dot_map,
            )
            .is_ok()
        {
            return Ok(());
        }

        let mut counterclockwise_turning_cardinal_direction = preferred_compass_direction;
        let mut clockwise_turning_cardinal_direction = preferred_compass_direction;

        loop {
            counterclockwise_turning_cardinal_direction =
                counterclockwise_turning_cardinal_direction.turn_counterclockwise();

            if self
                .anteroute_fanout_on_bbox_in_direction(
                    autorouter,
                    recorder,
                    ratvertex,
                    ratline,
                    source_dot,
                    bbox,
                    target_layer,
                    counterclockwise_turning_cardinal_direction,
                    options,
                    terminating_dot_map,
                )
                .is_ok()
            {
                return Ok(());
            }

            clockwise_turning_cardinal_direction =
                clockwise_turning_cardinal_direction.turn_clockwise();

            if self
                .anteroute_fanout_on_bbox_in_direction(
                    autorouter,
                    recorder,
                    ratvertex,
                    ratline,
                    source_dot,
                    bbox,
                    target_layer,
                    clockwise_turning_cardinal_direction,
                    options,
                    terminating_dot_map,
                )
                .is_ok()
            {
                return Ok(());
            }

            if counterclockwise_turning_cardinal_direction == preferred_compass_direction
                || clockwise_turning_cardinal_direction == preferred_compass_direction
            {
                return Err(AnterouteFailure);
            }
        }
    }

    fn anteroute_fanout_on_bbox_in_direction(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
        recorder: &mut BoardEdit,
        ratvertex: NodeIndex<usize>,
        ratline: RatlineUid,
        source_dot: FixedDotIndex,
        bbox: AABB<[f64; 2]>,
        target_layer: usize,
        direction: impl Into<Point>,
        options: &AnterouterOptions,
        terminating_dot_map: &mut BTreeMap<(RatlineUid, FixedDotIndex), FixedDotIndex>,
    ) -> Result<(), AnterouteFailure> {
        let (via, dots) = self.place_fanout_via_on_bbox_in_direction(
            autorouter,
            recorder,
            ratvertex,
            ratline,
            source_dot,
            bbox,
            target_layer,
            direction,
            options,
            terminating_dot_map,
        )?;

        let layer = source_dot
            .primitive_ref(autorouter.board().layout().drawing())
            .layer();
        let maybe_net = source_dot
            .primitive_ref(autorouter.board().layout().drawing())
            .maybe_net();
        let fanout_dot = *dots
            .iter()
            .find(|dot| {
                source_dot
                    .primitive_ref(autorouter.board().layout().drawing())
                    .layer()
                    == dot
                        .primitive_ref(autorouter.board().layout().drawing())
                        .layer()
            })
            .unwrap();

        // The design's declared trace width — a fixed width is
        // meaningless across coordinate scales and on
        // fine-pitch boards the stub can never be placed.
        let width = autorouter.board().mesadata().default_trace_width();
        if autorouter
            .board
            .layout_mut()
            .add_fixed_seg(
                &mut recorder.layout_edit,
                source_dot,
                fanout_dot,
                FixedSegWeight(GeneralSegWeight {
                    width,
                    layer,
                    maybe_net,
                }),
            )
            .is_ok()
        {
            Ok(())
        } else {
            autorouter.board.remove_via(recorder, via, dots);
            terminating_dot_map.remove(&(ratline, source_dot));
            Err(AnterouteFailure)
        }
    }

    fn place_fanout_via_on_bbox_in_direction(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
        recorder: &mut BoardEdit,
        _ratvertex: NodeIndex<usize>,
        ratline: RatlineUid,
        source_dot: FixedDotIndex,
        bbox: AABB<[f64; 2]>,
        target_layer: usize,
        direction: impl Into<Point>,
        options: &AnterouterOptions,
        terminating_dot_map: &mut BTreeMap<(RatlineUid, FixedDotIndex), FixedDotIndex>,
    ) -> Result<(GenericIndex<ViaWeight>, Vec<FixedDotIndex>), AnterouteFailure> {
        let source_layer = autorouter
            .board()
            .layout()
            .drawing()
            .primitive_ref(source_dot)
            .layer();
        let pin_maybe_net = autorouter
            .board()
            .layout()
            .drawing()
            .primitive_ref(source_dot)
            .maybe_net();
        let bbox_center = point! {x: bbox.center()[0], y: bbox.center()[1]};
        let center = autorouter
            .board()
            .layout()
            .drawing()
            .primitive_ref(source_dot)
            .shape()
            .center();

        let cardinal_direction_vector = direction.into();

        // The via center must land at least its own radius plus the
        // net's clearance past the pad's bbox edge, or the placement
        // infringes the very pad it fans out — the configured
        // clearance is only a floor for that minimum.
        let via_radius = autorouter.board().mesadata().default_via_radius();
        let fanout_clearance = options.fanout_clearance.max(
            via_radius
                + pin_maybe_net
                    .map(|net| {
                        autorouter
                            .board()
                            .layout()
                            .rules()
                            .largest_clearance(Some(net))
                    })
                    .unwrap_or(0.0),
        );

        let bbox_anchor = point! {
            x: (bbox.upper()[0] - bbox.lower()[0]) / 2.0 * cardinal_direction_vector.x(),
            y: (bbox.upper()[1] - bbox.lower()[1]) / 2.0 * cardinal_direction_vector.y(),
        };
        let bbox_anchor_with_clearance = bbox_anchor
            * (1.0 + fanout_clearance / Euclidean::distance(bbox_anchor, point! {x: 0.0, y: 0.0}));
        let pos = center
            + cardinal_direction_vector
                * (bbox_center + bbox_anchor_with_clearance - center)
                    .dot(cardinal_direction_vector);

        if let Ok((via, dots)) = autorouter.board.add_via(
            recorder,
            ViaWeight {
                from_layer: std::cmp::min(source_layer, target_layer),
                to_layer: std::cmp::max(source_layer, target_layer),
                // The design's declared via size — a fixed radius is
                // meaningless across arbitrary coordinate scales and
                // seals every fanout attempt on fine-pitch boards.
                circle: Circle { pos, r: via_radius },
                maybe_net: pin_maybe_net,
            },
            autorouter
                .board()
                .node_pinname(&GenericNode::Primitive(source_dot.into()))
                .cloned(),
        ) {
            let terminating_dot = dots
                .iter()
                .find(|dot| {
                    target_layer
                        == dot
                            .primitive_ref(autorouter.board().layout().drawing())
                            .layer()
                })
                .unwrap();
            terminating_dot_map.insert((ratline, source_dot), *terminating_dot);
            Ok((via, dots))
        } else {
            Err(AnterouteFailure)
        }
    }
}

#[cfg(test)]
mod tests {
    use std::{collections::BTreeMap, fs::File, io::BufReader};

    use geo::Distance;
    use geo::Euclidean;
    use specctra_core::{
        mesadata::SpecctraMesadata,
        rules::{AccessRules, Conditions},
    };

    use super::*;
    use crate::{
        board::edit::BoardEdit,
        drawing::dot::{FixedDotWeight, GeneralDotWeight},
        specctra::design::SpecctraDesign,
    };

    fn autorouter_lm317() -> Autorouter<SpecctraMesadata> {
        let design_file =
            File::open("tests/multilayer/prerouted_lm317_breakout/unrouted_lm317_breakout.dsn")
                .unwrap();
        let design = SpecctraDesign::load(BufReader::new(design_file)).unwrap();
        Autorouter::new(design.make_board(&mut BoardEdit::new())).unwrap()
    }

    /// A pad whose inner fanout ring is covered by neighboring copper
    /// offset off each direction's centerline — enough to block the
    /// via spot while leaving the radial escape seg a corridor — used
    /// to deadlock on the old innermost-ring-only search. The ring
    /// fallback must place the via one via-diameter further out.
    #[test]
    fn fanout_ring_fallback_places_via_past_blocked_inner_ring() {
        let mut autorouter = autorouter_lm317();
        let options = AnterouterOptions {
            fanout_clearance: 200.0,
            fanout_rings: 3,
        };

        // Pick a ratline whose first endpoint carries a net — a netted
        // pad is the case that matters, and the blockers need a
        // different net to collide with.
        let (ratline, pad, ratvertex) = autorouter
            .ratsnests()
            .on_principal_layer(0)
            .graph()
            .edge_indices()
            .find_map(|index| {
                let uid = RatlineUid {
                    principal_layer: 0,
                    index,
                };
                let pad = uid.ref_(&autorouter).endpoint_dots().0;
                pad.primitive_ref(autorouter.board().layout().drawing())
                    .maybe_net()
                    .map(|_| (uid, pad, uid.ref_(&autorouter).endpoint_indices().0))
            })
            .unwrap();
        let center = pad
            .primitive_ref(autorouter.board().layout().drawing())
            .shape()
            .center();
        let pad_layer = pad
            .primitive_ref(autorouter.board().layout().drawing())
            .layer();
        let pad_net = pad
            .primitive_ref(autorouter.board().layout().drawing())
            .maybe_net();

        // A foreign net for the blocker dots.
        let other_net = autorouter
            .ratsnests()
            .on_principal_layer(0)
            .graph()
            .edge_indices()
            .map(|index| {
                let uid = RatlineUid {
                    principal_layer: 0,
                    index,
                };
                uid.ref_(&autorouter).endpoint_dots()
            })
            .flat_map(|(a, b)| [a, b])
            .find_map(|dot| {
                let net = dot
                    .primitive_ref(autorouter.board().layout().drawing())
                    .maybe_net()?;
                (Some(net) != pad_net).then_some(net)
            });

        let small_bbox = if let Some(poly) = autorouter.board().layout().primitive_poly(pad.into())
        {
            let bbox = poly.ref_(autorouter.board().layout()).shape().envelope();
            AABB::<[f64; 2]>::from_corners(
                [bbox.lower().x(), bbox.lower().y()],
                [bbox.upper().x(), bbox.upper().y()],
            )
        } else {
            pad.primitive_ref(autorouter.board().layout().drawing())
                .shape()
                .envelope()
        };
        let mut large_bbox = small_bbox;
        for enclosing_poly in autorouter.board().layout().polys_enclosing_point(center) {
            let bbox = enclosing_poly
                .ref_(autorouter.board().layout())
                .shape()
                .envelope();
            large_bbox.merge(&AABB::<[f64; 2]>::from_corners(
                [bbox.lower().x(), bbox.lower().y()],
                [bbox.upper().x(), bbox.upper().y()],
            ));
        }

        let via_r = autorouter.board().mesadata().default_via_radius();
        let clearance = options.fanout_clearance.max(
            via_r
                + autorouter
                    .board()
                    .layout()
                    .rules()
                    .largest_clearance(pad_net),
        );
        let seg_halfwidth = autorouter.board().mesadata().default_trace_width() / 2.0;
        let blocker_r = via_r * 0.25;
        // The clearance the collision check actually applies between
        // the pad's net and the blocker net on the pad's layer.
        let pair_clearance = autorouter
            .board()
            .layout()
            .rules()
            .clearance(
                &Conditions {
                    net: pad_net.unwrap(),
                    maybe_region: None,
                    maybe_layer: Some(pad_layer),
                },
                &Conditions {
                    net: other_net.unwrap(),
                    maybe_region: None,
                    maybe_layer: Some(pad_layer),
                },
            )
            .unwrap_or(clearance);
        // Far enough off the centerline that the escape seg clears the
        // blocker, close enough that a via at the inner spot infringes.
        let offset = seg_halfwidth + blocker_r + pair_clearance + via_r * 0.25;
        assert!(
            offset < via_r + blocker_r + pair_clearance,
            "fixture geometry admits no lateral-blocking window: via_r={via_r} seg_hw={seg_halfwidth} pair_clearance={pair_clearance}"
        );

        // Replicates the candidate-position formula so blockers land on
        // every inner-ring landing spot for both candidate bboxes.
        let block = |autorouter: &mut Autorouter<SpecctraMesadata>,
                     recorder: &mut BoardEdit,
                     pos: Point| {
            autorouter.board.layout_mut().add_fixed_dot_infringably(
                &mut recorder.layout_edit,
                FixedDotWeight(GeneralDotWeight {
                    circle: Circle { pos, r: blocker_r },
                    layer: pad_layer,
                    maybe_net: other_net,
                }),
            );
        };

        let dirs = [
            (1.0, 0.0),
            (-1.0, 0.0),
            (0.0, 1.0),
            (0.0, -1.0),
            (1.0, 1.0),
            (1.0, -1.0),
            (-1.0, 1.0),
            (-1.0, -1.0),
        ];
        let mut recorder = BoardEdit::new();
        let mut small_inner_spots = vec![];
        for (bi, bbox) in [small_bbox, large_bbox].iter().enumerate() {
            let half = [
                (bbox.upper()[0] - bbox.lower()[0]) / 2.0,
                (bbox.upper()[1] - bbox.lower()[1]) / 2.0,
            ];
            let bbox_center = point! {x: bbox.center()[0], y: bbox.center()[1]};
            for (dx, dy) in dirs {
                let dir = point! {x: dx, y: dy};
                let anchor = point! {x: half[0] * dx, y: half[1] * dy};
                let anchor_c = anchor
                    * (1.0 + clearance / Euclidean::distance(anchor, point! {x: 0.0, y: 0.0}));
                let pos = center + dir * (bbox_center + anchor_c - center).dot(dir);
                if bi == 0 {
                    small_inner_spots.push(pos);
                }
                let norm = (dx * dx + dy * dy).sqrt();
                let perp = point! {x: -dy / norm, y: dx / norm};
                block(&mut autorouter, &mut recorder, pos + perp * offset);
            }
        }

        let mut anterouter = Anterouter::new(AnterouterPlan {
            layer_map: BTreeMap::from([(ratline, 1)]),
            static_terminating_dot_map: BTreeMap::new(),
        });
        let mut terminating_dot_map = BTreeMap::new();

        // Call the fanout directly so the sibling endpoint's own
        // outcome cannot mask this pad's result.
        anterouter
            .anteroute_fanout(
                &mut autorouter,
                &mut recorder,
                ratvertex,
                ratline,
                pad,
                1,
                &options,
                &mut terminating_dot_map,
            )
            .expect("ring fallback should place the via past the blocked inner ring");

        let via_dot = terminating_dot_map[&(ratline, pad)];
        let via_pos = via_dot
            .primitive_ref(autorouter.board().layout().drawing())
            .shape()
            .center();
        // The inner ring's nearest-direction extent: cardinal spots
        // reach the pad edge + clearance; a fallback-placed via must be
        // strictly beyond that.
        let inner_radius = small_inner_spots[..4]
            .iter()
            .map(|pos| Euclidean::distance(*pos, center))
            .fold(0.0_f64, f64::max);
        assert!(
            Euclidean::distance(via_pos, center) > inner_radius,
            "via should land outside the blocked inner ring"
        );
    }
}
