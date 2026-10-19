// SPDX-FileCopyrightText: 2026 Topola contributors
//
// SPDX-License-Identifier: MIT

//! The annealer's energy terms, all built on one bounded soft-core
//! potential. `soft_core(0) = 1` is the pass-through invariant: a
//! curve crossing another pays a *finite* penalty, so high-temperature
//! sweeps accept homotopy changes that a divergent `1/d²` field would
//! make impossible.

use super::{
    curve::{seg_seg_dist_2d, Curve3, Point3},
    obstacles::ObstacleField,
    AnnealOptions,
};

/// Bounded Lorentzian soft core: `σ²/(d²+σ²)`. `f(0)=1`, monotone
/// decreasing, ~`1/d²` tail — short-range repulsion with a finite
/// barrier height instead of a hard core.
pub fn soft_core(d: f64, sigma: f64) -> f64 {
    let s2 = sigma * sigma;
    s2 / (d * d + s2)
}

/// How strongly two segments' z-intervals interact: 1 when
/// overlapping, soft-core decay over half a layer spacing when
/// separated (so adjacent-layer segments interact weakly, layers two
/// apart barely).
fn z_coupling(za0: f64, za1: f64, zb0: f64, zb1: f64) -> f64 {
    let (alo, ahi) = (za0.min(za1), za0.max(za1));
    let (blo, bhi) = (zb0.min(zb1), zb0.max(zb1));
    let gap = (alo.max(blo) - ahi.min(bhi)).max(0.0);
    soft_core(gap, 0.5)
}

/// Context carried across energy evaluations: options-resolved scales.
pub struct EnergyCtx<'a> {
    pub options: &'a AnnealOptions,
    pub sigma: f64,
    pub z_weight: f64,
    /// Repulsion cutoff: contributions beyond this XY distance are
    /// ignored (the tail is ~`σ²/d²`, ~1% at `6σ`).
    pub repulsion_cutoff: f64,
    /// Current obstacle weight after ramping.
    pub w_obstacle: f64,
    /// Current planarity weight after ramping.
    pub w_planar: f64,
}

impl<'a> EnergyCtx<'a> {
    pub fn new(options: &'a AnnealOptions, band_width: f64, ramp: f64) -> Self {
        let sigma = options.sigma(band_width);
        let w_planar = options.w_planar * ramp;
        let w_obstacle = if options.obstacle_weight_ramp {
            options.w_obstacle * (0.1 + 0.9 * ramp)
        } else {
            options.w_obstacle
        };
        Self {
            options,
            sigma,
            z_weight: options.z_scale(band_width),
            repulsion_cutoff: 6.0 * sigma,
            w_obstacle,
            w_planar,
        }
    }

    /// 3D length as a stretch ratio against the straight anchor
    /// distance — dimensionless, so weights transfer across boards.
    pub fn curve_length(&self, c: &Curve3) -> f64 {
        let length: f64 = c.segments().map(|(a, b)| a.dist(&b, self.z_weight)).sum();
        let anchor = c.start.dist(&c.end, self.z_weight).max(1e-9);
        length / anchor
    }

    /// Repulsion between two segments: soft-core over XY distance
    /// times z-interval coupling.
    pub fn pair_repulsion(&self, a0: Point3, a1: Point3, b0: Point3, b1: Point3) -> f64 {
        let dz = z_coupling(a0.z, a1.z, b0.z, b1.z);
        if dz < 0.02 {
            return 0.0;
        }
        let d = seg_seg_dist_2d(a0, a1, b0, b1);
        if d >= self.repulsion_cutoff {
            return 0.0;
        }
        soft_core(d, self.sigma) * dz
    }

    /// Obstacle soft-repulsion: Σ over control points and segment
    /// midpoints, weighted by each sample's layer affinity — at
    /// fractional `z` the curve "floats between planes" and sees no
    /// obstacles.
    pub fn curve_obstacle(&self, c: &Curve3, obstacles: &ObstacleField) -> f64 {
        let mut total = 0.0;
        for (a, b) in c.segments() {
            for p in [a.lerp(&b, 0.25), a.lerp(&b, 0.75)] {
                total += self.point_obstacle(obstacles, p, c.net);
            }
        }
        for p in &c.control {
            total += self.point_obstacle(obstacles, *p, c.net);
        }
        self.w_obstacle * total
    }

    /// One sampled point's obstacle penalty: `soft_repulsion` at the
    /// nearest layer, gated by layer affinity.
    pub fn point_obstacle(&self, obstacles: &ObstacleField, p: Point3, net: Option<usize>) -> f64 {
        let layer =
            p.z.round()
                .clamp(0.0, obstacles.layer_count().saturating_sub(1) as f64) as usize;
        let affinity = p.layer_affinity();
        if affinity < 0.05 {
            return 0.0;
        }
        affinity * obstacles.soft_repulsion(p.xy(), layer, net, self.sigma)
    }

    /// z-tortuosity Σ|Δz| — penalizes oscillation, consolidates layer
    /// hops into single crossings.
    pub fn curve_tortuosity(&self, c: &Curve3) -> f64 {
        c.segments().map(|(a, b)| (a.z - b.z).abs()).sum()
    }

    /// Planarity well Σ sin²(πz) over interior control points.
    pub fn curve_planarity(&self, c: &Curve3) -> f64 {
        self.w_planar
            * c.control
                .iter()
                .map(|p| p.planarity_violation())
                .sum::<f64>()
    }

    /// Total weighted energy of `c`, with repulsion evaluated by the
    /// caller's `repulsion` closure against the current ensemble.
    pub fn total(&self, c: &Curve3, obstacles: &ObstacleField, repulsion: f64) -> f64 {
        self.options.w_length * self.curve_length(c)
            + self.options.w_repulsion * repulsion
            + self.options.w_via * self.curve_tortuosity(c)
            + self.curve_obstacle(c, obstacles)
            + self.curve_planarity(c)
    }
}

#[cfg(test)]
mod tests {
    use super::{super::curve::z_overlap, *};

    #[test]
    fn soft_core_is_bounded_and_short_ranged() {
        assert_eq!(soft_core(0.0, 2.0), 1.0);
        assert!(soft_core(1.0, 2.0) < 1.0);
        assert!(soft_core(0.5, 2.0) > soft_core(1.5, 2.0));
        assert!(soft_core(20.0, 2.0) < 0.02);
    }

    #[test]
    fn z_coupling_decays_over_layers() {
        assert_eq!(z_coupling(0.0, 0.0, 0.0, 0.0), 1.0);
        assert!(z_coupling(0.0, 0.0, 1.0, 1.0) < 0.25);
        assert!(z_coupling(0.0, 0.0, 3.0, 3.0) < 0.03);
    }

    #[test]
    fn seg_dist_matches_point_dist_for_parallel_segments() {
        let a0 = Point3::new(0.0, 0.0, 0.0);
        let a1 = Point3::new(10.0, 0.0, 0.0);
        let b0 = Point3::new(0.0, 3.0, 0.0);
        let b1 = Point3::new(10.0, 3.0, 0.0);
        assert!((seg_seg_dist_2d(a0, a1, b0, b1) - 3.0).abs() < 1e-9);

        let c0 = Point3::new(20.0, 0.0, 0.0);
        let c1 = Point3::new(30.0, 0.0, 0.0);
        assert!((seg_seg_dist_2d(a0, a1, c0, c1) - 10.0).abs() < 1e-9);
    }

    #[test]
    fn z_overlap_fraction() {
        assert_eq!(z_overlap(0.0, 1.0, 0.0, 1.0), 1.0);
        assert_eq!(z_overlap(0.0, 1.0, 2.0, 3.0), 0.0);
        assert!((z_overlap(0.0, 1.0, 0.5, 2.0) - 0.5).abs() < 1e-9);
    }
}
