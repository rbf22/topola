// SPDX-FileCopyrightText: 2026 Topola contributors
//
// SPDX-License-Identifier: MIT

//! The fixed-geometry obstacle field the annealer repels against.
//!
//! One 2D R-tree per layer holds the bounding envelopes of every fixed
//! primitive and polygon visible on that layer. Repulsion uses the same
//! bounded soft-core potential as curve–curve repulsion — a point
//! *inside* an obstacle's envelope pays a finite penalty, so at high
//! temperature curves pass through pads to explore both sides of a pin
//! cluster, and only late cooling ejects them.

use std::collections::BTreeMap;

use geo::Point;
use rstar::{PointDistance, RTree, RTreeObject, AABB};
use specctra_core::mesadata::AccessMesadata;

use crate::{
    autorouter::Autorouter,
    drawing::{
        graph::{GetMaybeNet, MakePrimitiveRef, PrimitiveIndex},
        primitive::MakePrimitiveShape,
    },
    geometry::shape::AccessShape,
    graph::MakeRef,
    layout::poly::MakePolygon,
};

/// One fixed obstacle's entry in a layer's R-tree.
#[derive(Clone, Copy, Debug)]
struct ObstacleEntry {
    /// The obstacle's XY bounding envelope — repulsion distance is
    /// measured to it, 0 for a point inside.
    envelope: AABB<Point>,
    maybe_net: Option<usize>,
    /// Extra radius folded into the soft-core: repulsion reaches
    /// clearance distance beyond the envelope itself.
    clearance: f64,
    /// The primitive this envelope came from — `None` for polys, which
    /// are compounds and cannot key `congestion`/`obstacle_penalties`.
    primitive: Option<PrimitiveIndex>,
}

impl RTreeObject for ObstacleEntry {
    type Envelope = AABB<Point>;

    fn envelope(&self) -> Self::Envelope {
        self.envelope
    }
}

impl PointDistance for ObstacleEntry {
    fn distance_2(&self, point: &Point) -> f64 {
        self.envelope.distance_2(point)
    }
}

/// Per-layer R-tree of fixed-geometry envelopes with their net and
/// clearance context.
pub struct ObstacleField {
    layers: Vec<RTree<ObstacleEntry>>,
}

impl ObstacleField {
    /// Builds the field from the board's fixed geometry. Before
    /// routing there are no loose primitives, so every primitive on a
    /// layer is a fixed obstacle; the guard stays explicit anyway so
    /// re-annealing mid-route would not treat bands as fixed.
    pub fn new(autorouter: &Autorouter<impl AccessMesadata>) -> Self {
        let drawing = autorouter.board().layout().drawing();
        let layer_count = drawing.layer_count();
        let rules = drawing.rules();

        let mut layers = Vec::with_capacity(layer_count);
        for layer in 0..layer_count {
            let mut entries: Vec<ObstacleEntry> = Vec::new();

            for node in drawing.layer_primitive_nodes(layer) {
                let primitive = node.primitive_ref(drawing);
                let maybe_net = primitive.maybe_net();
                let envelope = primitive.shape().bbox_without_margin();
                let (lo, hi) = (envelope.lower(), envelope.upper());
                entries.push(ObstacleEntry {
                    envelope: AABB::from_corners(
                        Point::new(lo[0], lo[1]),
                        Point::new(hi[0], hi[1]),
                    ),
                    maybe_net,
                    clearance: rules.largest_clearance(maybe_net),
                    primitive: Some(node),
                });
            }

            for poly in autorouter.board().layout().layer_poly_nodes(layer) {
                let polygon = poly.ref_(autorouter.board().layout()).shape();
                let Some(bbox) =
                    geo::algorithm::bounding_rect::BoundingRect::bounding_rect(&polygon)
                else {
                    continue;
                };
                let maybe_net = autorouter
                    .board()
                    .layout()
                    .drawing()
                    .compound_weight(poly.into())
                    .maybe_net();
                entries.push(ObstacleEntry {
                    envelope: AABB::from_corners(
                        Point::new(bbox.min().x, bbox.min().y),
                        Point::new(bbox.max().x, bbox.max().y),
                    ),
                    maybe_net,
                    clearance: rules.largest_clearance(maybe_net),
                    primitive: None,
                });
            }

            layers.push(RTree::bulk_load(entries));
        }

        Self { layers }
    }

    /// Number of layer trees — the `z` quantization bound.
    pub fn layer_count(&self) -> usize {
        self.layers.len()
    }

    /// Soft-core repulsion at `pos` on `layer` for a curve of net
    /// `net`: Σ `σ_eff²/(d² + σ_eff²)` over other-net obstacles, with
    /// `d` the distance to the obstacle's envelope and
    /// `σ_eff = σ + clearance`. Bounded ≈1 inside the envelope —
    /// the pass-through term.
    pub fn soft_repulsion(&self, pos: Point, layer: usize, net: Option<usize>, sigma: f64) -> f64 {
        let Some(tree) = self.layers.get(layer) else {
            return 0.0;
        };

        let mut total = 0.0;
        for entry in tree.locate_within_distance(pos, (sigma * 6.0).powi(2)) {
            if net.is_some() && entry.maybe_net == net {
                continue;
            }
            let sigma_eff = sigma + entry.clearance;
            let d = entry.envelope.distance_2(&pos).sqrt();
            total += sigma_eff * sigma_eff / (d * d + sigma_eff * sigma_eff);
        }
        total
    }

    /// Obstacles within repulsion range of `pos` on `layer` for net
    /// `net`, with their primitive index — for the congestion
    /// accumulator. Polys carry `primitive: None` and are skipped by
    /// the caller when a `PrimitiveIndex` is required.
    pub fn near_obstacles(
        &self,
        pos: Point,
        layer: usize,
        net: Option<usize>,
        sigma: f64,
    ) -> impl Iterator<Item = PrimitiveIndex> + '_ {
        self.layers.get(layer).into_iter().flat_map(move |tree| {
            tree.locate_within_distance(pos, (sigma * 6.0).powi(2))
                .filter_map(move |entry| {
                    (net.is_none() || entry.maybe_net != net)
                        .then_some(entry.primitive)
                        .flatten()
                })
        })
    }

    /// For the freeze-repair pass: if `pos` repels against an
    /// other-net obstacle on `layer`, returns a position pushed just
    /// outside the offending envelope's clearance. `None` when `pos`
    /// is clean.
    pub fn nearest_obstacle_exit(
        &self,
        pos: Point,
        layer: usize,
        net: Option<usize>,
        sigma: f64,
    ) -> Option<Point> {
        let tree = self.layers.get(layer)?;

        let mut nearest: Option<(f64, &AABB<Point>, f64)> = None;
        for entry in tree.locate_within_distance(pos, (sigma * 6.0).powi(2)) {
            if net.is_some() && entry.maybe_net == net {
                continue;
            }
            let sigma_eff = sigma + entry.clearance;
            let d = entry.envelope.distance_2(&pos).sqrt();
            if nearest.is_none_or(|(best, _, _)| d < best) {
                nearest = Some((d, &entry.envelope, sigma_eff));
            }
        }

        let (d, envelope, sigma_eff) = nearest?;
        (d < sigma_eff).then(|| {
            let (lo, hi) = (envelope.lower(), envelope.upper());
            let center = Point::new((lo.x() + hi.x()) / 2.0, (lo.y() + hi.y()) / 2.0);
            let delta = pos - center;
            let len = delta.x().hypot(delta.y());
            let dir = if len > 1e-9 {
                Point::new(delta.x() / len, delta.y() / len)
            } else {
                Point::new(1.0, 0.0)
            };
            // Push along `dir` past the envelope's support function in
            // that direction plus the repulsion core — the position
            // where repulsion is ~0.
            let hx = (hi.x() - lo.x()) / 2.0;
            let hy = (hi.y() - lo.y()) / 2.0;
            let extent = dir.x().abs() * hx + dir.y().abs() * hy;
            center + dir * (extent + sigma_eff)
        })
    }

    /// Counts near-misses of each sampled point against fixed
    /// primitives, accumulated into `counts` — the `plan.congestion`
    /// source feeding `seed_congestion` (fixed primitives only, per
    /// its contract).
    pub fn record_contest(
        &self,
        counts: &mut BTreeMap<PrimitiveIndex, usize>,
        pos: Point,
        layer: usize,
        net: Option<usize>,
        sigma: f64,
    ) {
        for primitive in self.near_obstacles(pos, layer, net, sigma) {
            *counts.entry(primitive).or_insert(0) += 1;
        }
    }
}
