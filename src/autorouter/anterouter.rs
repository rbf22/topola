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
