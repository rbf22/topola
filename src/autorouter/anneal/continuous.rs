// SPDX-FileCopyrightText: 2026 Topola contributors
//
// SPDX-License-Identifier: MIT

//! The paper's formulation: every ratline anneals as one parametric 3D
//! curve under soft-core repulsion. Because pass-throughs cost finite
//! energy, the high-temperature phase genuinely explores homotopy
//! classes — curves find which side of each pin cluster to thread —
//! before cooling quantizes `z` onto discrete layers and consolidates
//! vertical excursions into via sites.

use std::collections::BTreeSet;

use rstar::{RTree, RTreeObject, AABB};
use specctra_core::mesadata::AccessMesadata;

use crate::autorouter::{ratline::RatlineUid, Autorouter};

use super::{
    curve::{Curve3, Point3},
    energy::EnergyCtx,
    obstacles::ObstacleField,
    quantize,
    rng::AnnealRng,
    AnnealOptions, AnnealPlan,
};

/// One curve segment in the broad-phase tree. Entries are rebuilt per
/// sweep; proposals widen their query envelopes by `drift` to see
/// curves that already moved this sweep.
struct SegEntry {
    aabb: AABB<[f64; 3]>,
    curve: usize,
}

impl RTreeObject for SegEntry {
    type Envelope = AABB<[f64; 3]>;

    fn envelope(&self) -> Self::Envelope {
        self.aabb
    }
}

/// Broad-phase slack: covers the largest single-proposal displacement
/// a neighbor could have made since the tree was built.
fn seg_envelope(a: Point3, b: Point3, inflate: f64) -> AABB<[f64; 3]> {
    AABB::from_corners(
        [
            a.x.min(b.x) - inflate,
            a.y.min(b.y) - inflate,
            a.z.min(b.z) - inflate,
        ],
        [
            a.x.max(b.x) + inflate,
            a.y.max(b.y) + inflate,
            a.z.max(b.z) + inflate,
        ],
    )
}

/// Per-curve intrinsic energy components (raw — weights applied at
/// use). Repulsion is not cached: it depends on the whole ensemble, so
/// every proposal evaluates both the current and candidate repulsion
/// live against the same ensemble state.
#[derive(Clone, Copy, Default)]
struct Components {
    length: f64,
    tortuosity: f64,
    obstacle: f64,
    planarity: f64,
}

/// Runs the continuous simulated annealer over `ratlines`.
pub fn anneal(
    autorouter: &Autorouter<impl AccessMesadata>,
    ratlines: &BTreeSet<RatlineUid>,
    options: &AnnealOptions,
    band_width: f64,
) -> AnnealPlan {
    let layer_count = autorouter.board().layout().drawing().layer_count();
    let obstacles = ObstacleField::new(autorouter);
    let mut rng = AnnealRng::new(options.seed);

    let mut curves: Vec<Curve3> = ratlines
        .iter()
        .map(|ratline| Curve3::new(autorouter, *ratline, options.control_points))
        .collect();
    if curves.is_empty() {
        return AnnealPlan::default();
    }

    // Jitter scales: the high-temperature range spans a meaningful
    // fraction of the board; the freeze range is sub-σ.
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for curve in &curves {
        for p in [curve.start, curve.end] {
            lo[0] = lo[0].min(p.x);
            lo[1] = lo[1].min(p.y);
            hi[0] = hi[0].max(p.x);
            hi[1] = hi[1].max(p.y);
        }
    }
    let diag = (hi[0] - lo[0]).hypot(hi[1] - lo[1]);
    let sigma = options.sigma(band_width);
    let jitter_hi = (4.0 * sigma).max(0.1 * diag);
    let jitter_lo = 0.3 * sigma;
    let drift = 2.0 * jitter_hi;

    let nets: Vec<Option<usize>> = curves.iter().map(|c| c.net).collect();

    let intrinsic = |ctx: &EnergyCtx, c: &Curve3, obstacles: &ObstacleField| Components {
        length: ctx.curve_length(c),
        tortuosity: ctx.curve_tortuosity(c),
        obstacle: ctx.curve_obstacle(c, obstacles) / ctx.w_obstacle.max(1e-9),
        planarity: c
            .control
            .iter()
            .map(|p| p.planarity_violation())
            .sum::<f64>(),
    };

    // Curve i's repulsion energy + the neighbor set contributing to
    // it (the crossing observability metric's "before/after" state).
    let repulsion_eval = |ctx: &EnergyCtx,
                          segs: &[(Point3, Point3)],
                          i: usize,
                          curves: &[Curve3],
                          tree: &RTree<SegEntry>|
     -> (f64, BTreeSet<usize>) {
        let mut candidates = BTreeSet::new();
        for &(a, b) in segs {
            for hit in tree.locate_in_envelope_intersecting(&seg_envelope(
                a,
                b,
                ctx.repulsion_cutoff + drift,
            )) {
                if hit.curve != i {
                    candidates.insert(hit.curve);
                }
            }
        }
        let mut total = 0.0;
        let mut neighbors = BTreeSet::new();
        for &j in &candidates {
            if curves[i].net.is_some() && nets[j] == curves[i].net {
                continue;
            }
            let mut pair = 0.0;
            for &(a, b) in segs {
                for (c0, c1) in curves[j].segments() {
                    pair += ctx.pair_repulsion(a, b, c0, c1);
                }
            }
            if pair > 0.01 {
                neighbors.insert(j);
            }
            total += pair;
        }
        (total, neighbors)
    };

    let build_tree = |curves: &[Curve3]| -> RTree<SegEntry> {
        RTree::bulk_load(
            curves
                .iter()
                .enumerate()
                .flat_map(|(i, c)| {
                    c.segments().map(move |(a, b)| SegEntry {
                        aabb: seg_envelope(a, b, 0.0),
                        curve: i,
                    })
                })
                .collect(),
        )
    };

    // Whole-ensemble energy: Σ intrinsic + ½·Σ rep(i) (each pair
    // counted twice). Evaluated per sweep against one tree snapshot.
    let system_energy =
        |ctx: &EnergyCtx, curves: &[Curve3], components: &[Components], tree: &RTree<SegEntry>| {
            let rep_total: f64 = (0..curves.len())
                .map(|i| {
                    let segs: Vec<_> = curves[i].segments().collect();
                    repulsion_eval(ctx, &segs, i, curves, tree).0
                })
                .sum::<f64>()
                * 0.5;
            components
                .iter()
                .map(|c| {
                    options.w_length * c.length
                        + options.w_via * c.tortuosity
                        + ctx.w_obstacle * c.obstacle
                        + ctx.w_planar * c.planarity
                })
                .sum::<f64>()
                + options.w_repulsion * rep_total
        };

    let mut ctx = EnergyCtx::new(options, band_width, 0.0);
    let mut components: Vec<Components> = curves
        .iter()
        .map(|c| intrinsic(&ctx, c, &obstacles))
        .collect();

    let (mut temperature, cooling) = (options.t_start(), options.cooling());
    let mut crossings = 0usize;
    let (mut accepted, mut rejected) = (0usize, 0usize);
    let mut best: Vec<Curve3> = curves.clone();
    // The straight-line ensemble the annealer started from — for the
    // improvement-over-initialization check and the benchmark report.
    let tree = build_tree(&curves);
    let initial_energy = system_energy(&ctx, &curves, &components, &tree);
    let mut best_energy = initial_energy;

    for sweep in 0..options.iterations {
        let ramp = sweep as f64 / (options.iterations.max(2) - 1) as f64;
        ctx = EnergyCtx::new(options, band_width, ramp);
        let tree = build_tree(&curves);
        let sigma_xy = jitter_lo + (jitter_hi - jitter_lo) * (1.0 - ramp);
        let sigma_z = 0.05 + 0.95 * (1.0 - ramp);

        for i in 0..curves.len() {
            let candidate = propose(
                &curves[i],
                &mut rng,
                sigma_xy,
                sigma_z,
                temperature / options.t_start(),
                layer_count,
            );
            let Some(candidate) = candidate else { continue };
            let candidate_curve = Curve3 {
                control: candidate,
                ..curves[i].clone()
            };

            let old_segs: Vec<_> = curves[i].segments().collect();
            let new_segs: Vec<_> = candidate_curve.segments().collect();
            let (rep_old, neigh_old) = repulsion_eval(&ctx, &old_segs, i, &curves, &tree);
            let (rep_new, neigh_new) = repulsion_eval(&ctx, &new_segs, i, &curves, &tree);

            let new_components = intrinsic(&ctx, &candidate_curve, &obstacles);
            let old = components[i];
            let de = options.w_length * (new_components.length - old.length)
                + options.w_via * (new_components.tortuosity - old.tortuosity)
                + ctx.w_obstacle * (new_components.obstacle - old.obstacle)
                + ctx.w_planar * (new_components.planarity - old.planarity)
                + options.w_repulsion * (rep_new - rep_old);

            if rng.accept(de, temperature) {
                if neigh_new != neigh_old {
                    crossings += 1;
                }
                accepted += 1;
                components[i] = new_components;
                curves[i] = candidate_curve;
            } else {
                rejected += 1;
            }
        }

        // Reported energy uses the true pair count: Σ_i rep(i) counts
        // each pair twice.
        let energy = system_energy(&ctx, &curves, &components, &tree);
        if energy < best_energy {
            best_energy = energy;
            best.clone_from_slice(&curves);
        }

        temperature *= cooling;
    }

    crate::metrics::add(&crate::metrics::METRICS.anneal_accepted, accepted);
    crate::metrics::add(&crate::metrics::METRICS.anneal_rejected, rejected);

    let mut plan = quantize::plan(&best, &obstacles, options, sigma, layer_count);
    plan.initial_energy = initial_energy;
    plan.final_energy = best_energy;
    plan.crossings = crossings;
    plan
}

/// One proposal move on `curve`'s interior control points. Returns
/// `None` for a degenerate pick the caller skips (e.g. zero control
/// points).
fn propose(
    curve: &Curve3,
    rng: &mut AnnealRng,
    sigma_xy: f64,
    sigma_z: f64,
    temperature_fraction: f64,
    layer_count: usize,
) -> Option<Vec<Point3>> {
    let k = curve.control.len();
    if k == 0 {
        return None;
    }
    let z_max = (layer_count.saturating_sub(1)).max(1) as f64;
    let clamp_z = |z: f64| z.clamp(0.0, z_max);
    let mut control = curve.control.clone();

    let pick = rng.uniform();
    if pick < 0.35 {
        // jitter_xy: one control point wanders in the board plane.
        let i = rng.below(k);
        control[i].x += rng.gauss() * sigma_xy;
        control[i].y += rng.gauss() * sigma_xy;
    } else if pick < 0.55 {
        // jitter_z: one control point floats between layers.
        let i = rng.below(k);
        control[i].z = clamp_z(control[i].z + rng.gauss() * sigma_z);
    } else if pick < 0.70 {
        // arc_shift_z: a contiguous run of control points hops up or
        // down one or two layers — the via-site formation move.
        let start = rng.below(k);
        let len = 1 + rng.below((k / 2).max(1));
        let dz = if rng.uniform() < 0.5 { -1.0 } else { 1.0 }
            * if rng.uniform() < 0.75 { 1.0 } else { 2.0 };
        for p in control.iter_mut().take(start + len).skip(start) {
            p.z = clamp_z(p.z + dz);
        }
    } else if pick < 0.85 {
        // slide: one control point relaxes toward the midpoint of its
        // neighbors — rubber-band smoothing. Strength follows T.
        let i = rng.below(k);
        let prev = if i == 0 { curve.start } else { control[i - 1] };
        let next = if i + 1 == k {
            curve.end
        } else {
            control[i + 1]
        };
        let mid = prev.lerp(&next, 0.5);
        let lambda = (1.5 * temperature_fraction).clamp(0.05, 1.0);
        control[i] = control[i].lerp(&mid, lambda);
    } else {
        // layer_flip: a contiguous run quantizes onto one random
        // integer layer — direct homotopy exploration in z.
        let start = rng.below(k);
        let len = 1 + rng.below((k / 2).max(1));
        let layer = rng.below(layer_count.max(1)) as f64;
        for p in control.iter_mut().take(start + len).skip(start) {
            p.z = layer;
        }
    }
    Some(control)
}
