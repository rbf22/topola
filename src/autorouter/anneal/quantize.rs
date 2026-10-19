// SPDX-FileCopyrightText: 2026 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Quantization: the annealed curves' fluid `z` collapses onto
//! discrete layers. Runs of constant quantized `z` become single-layer
//! sections; boundaries become via sites; curves are scored by
//! contestedness for the routing order; near-misses accumulate into
//! the congestion seed. A residual-violation repair pass nudges
//! samples still sleeping inside obstacles at freeze — the soft
//! potential permits lingering overlaps, and legal routes cannot use
//! them.

use std::collections::BTreeMap;

use geo::Point;

use super::{
    curve::{Curve3, Point3},
    energy::soft_core,
    obstacles::ObstacleField,
    AnnealOptions, AnnealPlan, AnnealSplit,
};

/// Polyline densification for quantization — via sites, corridor
/// waypoints, and congestion samples all come from this sampling.
const SAMPLES_PER_SEG: usize = 4;

/// One merged run of constant layer over a curve's sample polyline.
struct Run {
    layer: usize,
    /// Sample index range `[lo, hi)` on the curve's samples.
    span: (usize, usize),
    points: Vec<Point>,
}

/// Converts the annealed curves into the routing plan consumed by
/// `handoff`. `final_energy`/`crossings` are stamped by the caller.
pub fn plan(
    curves: &[Curve3],
    obstacles: &ObstacleField,
    options: &AnnealOptions,
    sigma: f64,
    layer_count: usize,
) -> AnnealPlan {
    let mut plan = AnnealPlan::default();
    let contestedness = contestedness_scores(curves, sigma);
    let mut congestion = BTreeMap::new();

    for curve in curves {
        let mut samples = curve.sample(SAMPLES_PER_SEG);
        repair_violations(&mut samples, curve.net, obstacles, sigma, layer_count);
        let runs = quantize_runs(&samples, options.min_run_fraction, layer_count);

        if runs.len() > 1 {
            let via_sites = runs
                .windows(2)
                .map(|window| {
                    let a = *window[0].points.last().unwrap();
                    let b = window[1].points[0];
                    a + (b - a) * 0.5
                })
                .collect();
            plan.splits.push(AnnealSplit {
                parent: curve.ratline,
                runs: runs
                    .iter()
                    .map(|run| (run.layer, run.points.clone()))
                    .collect(),
                via_sites,
            });
        }
        plan.layer_map.insert(curve.ratline, runs[0].layer);

        // Demand corridor: every sample tagged with its merged run's
        // layer — `carve_corridor` breaks chains at transitions.
        let corridor: Vec<(Point, usize)> = samples
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let layer = runs
                    .iter()
                    .find(|run| i >= run.span.0 && i < run.span.1)
                    .map(|run| run.layer)
                    .unwrap_or(runs[0].layer);
                (p.xy(), layer)
            })
            .collect();
        plan.corridors.insert(curve.ratline, corridor);

        // Congestion seed: near-misses on the *quantized* layers — the
        // obstacles this curve will still hug after legalization.
        for (i, p) in samples.iter().enumerate() {
            let layer = runs
                .iter()
                .find(|run| i >= run.span.0 && i < run.span.1)
                .map(|run| run.layer)
                .unwrap_or(runs[0].layer);
            obstacles.record_contest(&mut congestion, p.xy(), layer, curve.net, sigma);
        }
    }

    let mut ranked: Vec<_> = curves.iter().zip(&contestedness).collect();
    ranked.sort_by(|a, b| b.1.total_cmp(a.1));
    plan.order = ranked.into_iter().map(|(curve, _)| curve.ratline).collect();
    plan.congestion = congestion;
    plan
}

/// Curve contestedness: residual repulsion against the rest of the
/// ensemble — the high-E_curves are the contested ones and route
/// first. Uses the same soft-core pairing as the annealer, evaluated
/// on the frozen state.
fn contestedness_scores(curves: &[Curve3], sigma: f64) -> Vec<f64> {
    let mut scores = vec![0.0; curves.len()];
    for i in 0..curves.len() {
        for j in (i + 1)..curves.len() {
            if curves[i].net.is_some() && curves[j].net == curves[i].net {
                continue;
            }
            let mut pair = 0.0;
            for (a0, a1) in curves[i].segments() {
                for (b0, b1) in curves[j].segments() {
                    // Final-state curves are on integer layers — z
                    // coupling is the layer-overlap test.
                    let za = (a0.z + a1.z) * 0.5;
                    let zb = (b0.z + b1.z) * 0.5;
                    if (za - zb).abs() > 0.5 {
                        continue;
                    }
                    let d = super::curve::seg_seg_dist_2d(a0, a1, b0, b1);
                    pair += soft_core(d, sigma);
                }
            }
            scores[i] += pair;
            scores[j] += pair;
        }
    }
    scores
}

/// Nudges samples still repelling obstacles at freeze: at most three
/// pushes per sample toward the nearest exit, anchors exempt (they
/// live inside their own pads).
fn repair_violations(
    samples: &mut [Point3],
    net: Option<usize>,
    obstacles: &ObstacleField,
    sigma: f64,
    layer_count: usize,
) {
    for i in 1..samples.len().saturating_sub(1) {
        for _ in 0..3 {
            let layer = samples[i]
                .z
                .round()
                .clamp(0.0, layer_count.saturating_sub(1) as f64) as usize;
            let repulsion = obstacles.soft_repulsion(samples[i].xy(), layer, net, sigma);
            if repulsion < 0.5 {
                break;
            }
            let Some(exit) = obstacles.nearest_obstacle_exit(samples[i].xy(), layer, net, sigma)
            else {
                break;
            };
            samples[i].x = exit.x();
            samples[i].y = exit.y();
        }
    }
}

/// Run-length encodes each sample's quantized layer, then merges runs
/// shorter than `min_fraction` of the sample count into the larger
/// neighbor — the small-oscillation collapse that keeps one-layer
/// jitters from becoming spurious vias.
fn quantize_runs(samples: &[Point3], min_fraction: f64, layer_count: usize) -> Vec<Run> {
    let layer_of =
        |p: &Point3| p.z.round().clamp(0.0, layer_count.saturating_sub(1) as f64) as usize;
    let min_len = (samples.len() as f64 * min_fraction).ceil().max(1.0) as usize;

    let mut runs: Vec<Run> = Vec::new();
    for (i, p) in samples.iter().enumerate() {
        let layer = layer_of(p);
        match runs.last_mut() {
            Some(run) if run.layer == layer => {
                run.span.1 = i + 1;
                run.points.push(p.xy());
            }
            _ => runs.push(Run {
                layer,
                span: (i, i + 1),
                points: vec![p.xy()],
            }),
        }
    }

    while let Some(shortest) = runs
        .iter()
        .enumerate()
        .filter(|(_, run)| run.span.1 - run.span.0 < min_len)
        .min_by_key(|(_, run)| run.span.1 - run.span.0)
        .map(|(i, _)| i)
    {
        let merge_into = if shortest == 0 {
            1
        } else if shortest + 1 == runs.len() {
            shortest - 1
        } else {
            // Merge into the longer neighbor — a short excursion
            // between two equal-length neighbors joins the earlier.
            let left = runs[shortest - 1].span.1 - runs[shortest - 1].span.0;
            let right = runs[shortest + 1].span.1 - runs[shortest + 1].span.0;
            if left >= right {
                shortest - 1
            } else {
                shortest + 1
            }
        };
        if merge_into < shortest {
            let dropped = runs.remove(shortest);
            let kept = &mut runs[merge_into];
            kept.points.extend(dropped.points);
            kept.span.1 = dropped.span.1;
        } else {
            let dropped = runs.remove(shortest);
            // `merge_into` was `shortest + 1`; removing `shortest`
            // shifts it down one slot.
            let kept = &mut runs[merge_into - 1];
            let mut points = dropped.points;
            points.extend(kept.points.iter().copied());
            kept.points = points;
            kept.span.0 = dropped.span.0;
        }
        // Absorbing a run can unite two same-layer neighbors —
        // coalesce so the merged block stays one run.
        let mut i = 0;
        while i + 1 < runs.len() {
            if runs[i].layer == runs[i + 1].layer {
                let next = runs.remove(i + 1);
                runs[i].points.extend(next.points);
                runs[i].span.1 = next.span.1;
            } else {
                i += 1;
            }
        }
    }

    runs
}

#[cfg(test)]
mod tests {
    use super::*;

    fn samples(layers: &[f64]) -> Vec<Point3> {
        layers
            .iter()
            .enumerate()
            .map(|(i, z)| Point3::new(i as f64 * 10.0, 0.0, *z))
            .collect()
    }

    #[test]
    fn single_run() {
        let runs = quantize_runs(&samples(&[0.1, 0.0, -0.1, 0.2, 0.0]), 0.15, 2);
        assert_eq!(runs.len(), 1);
        assert_eq!(runs[0].layer, 0);
    }

    #[test]
    fn three_runs_yield_two_boundaries() {
        // A long z≈1 excursion mid-curve survives merging: two
        // boundaries → two via sites → two continuations at handoff.
        let profile = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ];
        let runs = quantize_runs(&samples(&profile), 0.15, 2);
        assert_eq!(runs.len(), 3);
        assert_eq!(
            runs.iter().map(|r| r.layer).collect::<Vec<_>>(),
            vec![0, 1, 0]
        );
    }

    #[test]
    fn short_excursion_collapses() {
        // One-layer jitter of two samples inside a 24-sample run is
        // below the merge threshold.
        let mut profile = vec![0.0; 24];
        profile[10] = 1.0;
        profile[11] = 1.0;
        let runs = quantize_runs(&samples(&profile), 0.15, 2);
        assert_eq!(runs.len(), 1);
        assert_eq!(runs[0].layer, 0);
    }

    #[test]
    fn end_short_run_merges_inward() {
        let mut profile = vec![0.0; 24];
        profile[0] = 1.0;
        let runs = quantize_runs(&samples(&profile), 0.15, 2);
        assert_eq!(runs.len(), 1);
    }
}
