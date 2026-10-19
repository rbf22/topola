// SPDX-FileCopyrightText: 2026 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Continuous 3D demand curves: each ratline is a polyline in
//! `(x, y, z)` space where `x, y` are board-plane coordinates and
//! `z ∈ [0, layer_count−1]` is the fluid layer axis. Endpoint anchors
//! are fixed to the pads' centers and layers; interior control points
//! carry the annealable state.

use geo::Point;
use rstar::AABB;
use specctra_core::mesadata::AccessMesadata;

use crate::{
    autorouter::{ratline::RatlineUid, Autorouter},
    drawing::{
        dot::FixedDotIndex,
        graph::{GetMaybeNet, MakePrimitiveRef},
        primitive::MakePrimitiveShape,
    },
    geometry::{shape::AccessShape, GetLayer},
    graph::MakeRef,
};

/// A point in the continuous routing volume.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Point3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn xy(&self) -> Point {
        Point::new(self.x, self.y)
    }

    /// 2D distance to `other` (the layer axis is handled separately by
    /// the repulsion machinery).
    pub fn dist_xy(&self, other: &Point3) -> f64 {
        (self.x - other.x).hypot(self.y - other.y)
    }

    /// 3D distance with the layer axis scaled by `z_weight`.
    pub fn dist(&self, other: &Point3, z_weight: f64) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = (self.z - other.z) * z_weight;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    pub fn lerp(&self, other: &Point3, t: f64) -> Self {
        Self {
            x: self.x + (other.x - self.x) * t,
            y: self.y + (other.y - self.y) * t,
            z: self.z + (other.z - self.z) * t,
        }
    }

    /// Deviation from the nearest discrete layer, in `[0, 0.5]`.
    pub fn layer_deviation(&self) -> f64 {
        (self.z - self.z.round()).abs()
    }

    /// `cos²(π·deviation)` — 1 on an integer layer, 0 midway between
    /// layers. Weights obstacle repulsion so a curve at fractional `z`
    /// sees no obstacles ("floats between planes").
    pub fn layer_affinity(&self) -> f64 {
        let d = std::f64::consts::PI * self.layer_deviation();
        let c = d.cos();
        c * c
    }

    /// `sin²(π·deviation)` — the planarity potential well: 0 on
    /// integer layers, max midway between them.
    pub fn planarity_violation(&self) -> f64 {
        let s = (std::f64::consts::PI * self.layer_deviation()).sin();
        s * s
    }
}

/// Minimum 2D distance between segments `(a0,a1)` and `(b0,b1)`.
/// Degenerate segments behave as points.
pub fn seg_seg_dist_2d(a0: Point3, a1: Point3, b0: Point3, b1: Point3) -> f64 {
    // Standard closest-point-between-segments, restricted to xy.
    let ux = a1.x - a0.x;
    let uy = a1.y - a0.y;
    let vx = b1.x - b0.x;
    let vy = b1.y - b0.y;
    let wx = a0.x - b0.x;
    let wy = a0.y - b0.y;

    let a = ux * ux + uy * uy;
    let b = ux * vx + uy * vy;
    let c = vx * vx + vy * vy;
    let d = ux * wx + uy * wy;
    let e = vx * wx + vy * wy;

    let denom = a * c - b * b;
    let eps = 1e-12;
    let s = if denom < eps {
        0.0
    } else {
        (b * e - c * d) / denom
    };

    // Clamp s, re-resolve t against it, re-resolve s — the standard
    // segment-segment closest-point clamping sequence.
    let s = s.clamp(0.0, 1.0);
    let t = if c < eps {
        0.0
    } else {
        ((b * s + e) / c).clamp(0.0, 1.0)
    };
    let s = if a < eps {
        0.0
    } else {
        ((b * t - d) / a).clamp(0.0, 1.0)
    };

    let dx = wx + ux * s - vx * t;
    let dy = wy + uy * s - vy * t;
    (dx * dx + dy * dy).sqrt()
}

/// Overlap fraction of two closed z-intervals relative to the shorter
/// one: `1` for full overlap, `0` for disjoint.
pub fn z_overlap(a0: f64, a1: f64, b0: f64, b1: f64) -> f64 {
    let (alo, ahi) = (a0.min(a1), a0.max(a1));
    let (blo, bhi) = (b0.min(b1), b0.max(b1));
    let overlap = (ahi.min(bhi) - alo.max(blo)).max(0.0);
    let shorter = (ahi - alo).min(bhi - blo).max(1e-9);
    (overlap / shorter).min(1.0)
}

/// One ratline's annealable curve. Anchors are constants; `control`
/// holds the `K` interior control points.
#[derive(Clone, Debug)]
pub struct Curve3 {
    pub ratline: RatlineUid,
    pub net: Option<usize>,
    pub start: Point3,
    pub end: Point3,
    pub control: Vec<Point3>,
}

impl Curve3 {
    /// Builds the initial curve for `ratline`: anchors at the endpoint
    /// dots' centers and layers, `control_points` interior points on
    /// the straight (x, y, z) lerp between them.
    pub fn new(
        autorouter: &Autorouter<impl AccessMesadata>,
        ratline: RatlineUid,
        control_points: usize,
    ) -> Self {
        let drawing = autorouter.board().layout().drawing();
        let anchor = |dot: FixedDotIndex| {
            let primitive = dot.primitive_ref(drawing);
            let center = primitive.shape().center();
            Point3::new(center.x(), center.y(), primitive.layer() as f64)
        };
        let (start_dot, end_dot) = ratline.ref_(autorouter).endpoint_dots();
        let (start, end) = (anchor(start_dot), anchor(end_dot));
        let control = (1..=control_points)
            .map(|i| start.lerp(&end, i as f64 / (control_points + 1) as f64))
            .collect();
        let net = start_dot.primitive_ref(drawing).maybe_net();
        Self {
            ratline,
            net,
            start,
            end,
            control,
        }
    }

    /// All polyline vertices in order: `start, control…, end`.
    pub fn points(&self) -> impl Iterator<Item = Point3> + '_ {
        std::iter::once(self.start)
            .chain(self.control.iter().copied())
            .chain(std::iter::once(self.end))
    }

    /// The polyline as a `Vec`.
    pub fn polyline(&self) -> Vec<Point3> {
        self.points().collect()
    }

    /// 3D segments `(a, b)` in order.
    pub fn segments(&self) -> impl Iterator<Item = (Point3, Point3)> + '_ {
        self.points().zip(self.points().skip(1))
    }

    /// The full polyline's 3D AABB.
    pub fn aabb(&self) -> AABB<[f64; 3]> {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for p in self.points() {
            for (k, v) in [p.x, p.y, p.z].iter().enumerate() {
                lo[k] = lo[k].min(*v);
                hi[k] = hi[k].max(*v);
            }
        }
        AABB::from_corners(lo, hi)
    }

    /// Uniformly sampled points along the polyline (per-segment
    /// `per_seg` samples including both endpoints of each segment).
    pub fn sample(&self, per_seg: usize) -> Vec<Point3> {
        let mut out = Vec::with_capacity(self.segments().count() * per_seg.max(1));
        for (a, b) in self.segments() {
            for k in 0..per_seg {
                out.push(a.lerp(&b, k as f64 / per_seg as f64));
            }
        }
        out.push(self.end);
        out
    }
}
