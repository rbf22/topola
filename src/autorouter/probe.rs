// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Relaxed-rule channel probes.
//!
//! Each unrouted ratline demands a corridor between its endpoints. To
//! measure that demand we temporarily relax clearances and attempt a
//! route: if the probe succeeds, its band's polyline is the path the
//! connection would take if the geometry allowed it; primitives lying
//! inside the corridor at the real clearance are true blockers. If the
//! probe fails even under relaxed rules, the terminal infringement —
//! recorded by the route stepper — names the last obstacle, whose
//! location marks where the path could not be forced through.
//!
//! Every probe band is reverted before the next ratline is probed and
//! the rules are restored afterwards, so probing leaves the board
//! unchanged.

use std::ops::ControlFlow;

use geo::{Line, Point};

use crate::{
    autorouter::{ratline::RatlineUid, selection::PinSelection, Autorouter},
    board::{
        edit::{BoardDataEdit, BoardEdit},
        AccessMesadata,
    },
    drawing::{
        band::BandTermsegIndex,
        dot::FixedDotIndex,
        graph::{GetMaybeNet, MakePrimitiveRef, PrimitiveIndex},
        loose::{GetPrevNextLoose, LooseIndex},
        primitive::MakePrimitiveShape,
        rules::{AccessRules, Conditions},
        Drawing,
    },
    geometry::{edit::Edit, primitive::PrimitiveShape, shape::AccessShape, GetLayer},
    graph::MakeRef,
    layout::LayoutEdit,
    router::{prenavmesh::PrenavmeshBases, RouteStepper, Router, RouterOptions},
    stepper::Step,
};

/// The outcome of probing one unrouted ratline under relaxed rules.
#[derive(Debug, PartialEq)]
pub struct ChannelProbe {
    /// The ratline's net.
    pub net: Option<usize>,
    /// The ratline's endpoint dots.
    pub endpoints: (FixedDotIndex, FixedDotIndex),
    /// Ordered waypoints of the demand path as `(center, layer)`. Empty
    /// when the probe could not produce a path even under relaxed rules.
    pub path: Vec<(Point, usize)>,
    /// On failure, the terminal `(infringer_ghost, infringee_ghost,
    /// obstacle)`: where the band wanted to be, the blocking primitive's
    /// shape, and the blocking primitive itself.
    pub terminal_blocker: Option<(PrimitiveShape, PrimitiveShape, PrimitiveIndex)>,
}

/// A primitive blocking a probe's demand corridor, with the smallest
/// displacement along the corridor's normal that would clear it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChannelBlocker {
    /// The blocking primitive.
    pub obstacle: PrimitiveIndex,
    /// Smallest displacement clearing the corridor, as `(dx, dy)`.
    pub shift: (f64, f64),
}

/// Projects `(x, y)` onto the unit vector `n` relative to `origin`.
fn project(origin: Point, n: (f64, f64), x: f64, y: f64) -> f64 {
    (x - origin.x()) * n.0 + (y - origin.y()) * n.1
}

/// Interval of a shape's bounding box along the unit vector `dir`,
/// relative to `origin`.
fn extent_along(origin: Point, dir: (f64, f64), shape: &PrimitiveShape) -> (f64, f64) {
    let lo = shape.bbox_without_margin().lower();
    let hi = shape.bbox_without_margin().upper();
    [
        (lo[0], lo[1]),
        (hi[0], lo[1]),
        (hi[0], hi[1]),
        (lo[0], hi[1]),
    ]
    .into_iter()
    .fold(
        (f64::INFINITY, f64::NEG_INFINITY),
        |(lo_s, hi_s), (x, y)| {
            (
                lo_s.min(project(origin, dir, x, y)),
                hi_s.max(project(origin, dir, x, y)),
            )
        },
    )
}

/// The smallest displacement along unit vector `n` that moves `shape`'s
/// bounding box outside `[-half_width, half_width]` around `origin`, on
/// the side the shape's center already occupies. Returns `None` when the
/// box already clears.
fn clearing_shift(
    origin: Point,
    n: (f64, f64),
    half_width: f64,
    center: Point,
    shape: &PrimitiveShape,
) -> Option<(f64, f64)> {
    let (s_min, s_max) = extent_along(origin, n, shape);
    let magnitude = if project(origin, n, center.x(), center.y()) >= 0.0 {
        half_width - s_min
    } else {
        s_max + half_width
    };
    if magnitude <= 0.0 {
        return None;
    }
    let sign = if project(origin, n, center.x(), center.y()) >= 0.0 {
        1.0
    } else {
        -1.0
    };
    Some((n.0 * sign * magnitude, n.1 * sign * magnitude))
}

/// A probe's product: the band's waypoint polyline when it
/// routed, or the terminal blocker when it failed.
type ProbeOutcome = (
    Vec<(Point, usize)>,
    Option<(PrimitiveShape, PrimitiveShape, PrimitiveIndex)>,
);

impl ChannelProbe {
    /// The channel's blockers: primitives lying inside the probe's
    /// demand corridor at real clearance, each with the smallest
    /// displacement that moves it outside. A failed probe reports its
    /// terminal blocker instead, pushed away from where the band wanted
    /// to be. Same-net primitives are connectable, never blockers.
    ///
    /// `drawing` must be the board the probe ran on, with real rules
    /// restored — probing relaxes clearances, so call this after the
    /// relaxation has been reverted.
    pub fn blockers<CW, Cel, R>(
        &self,
        drawing: &Drawing<CW, Cel, R>,
        band_half_width: f64,
    ) -> Vec<ChannelBlocker>
    where
        CW: Clone,
        Cel: Copy,
        R: AccessRules,
    {
        let mut blockers = vec![];
        let mut push = |obstacle: PrimitiveIndex, shift: (f64, f64)| {
            if self.net.is_some() && obstacle.primitive_ref(drawing).maybe_net() == self.net {
                return;
            }
            blockers.push(ChannelBlocker { obstacle, shift });
        };

        if self.path.is_empty() {
            // Relaxed route failed: the terminal infringement names the
            // last obstacle. Push it away from where the band wanted to
            // be, by enough to separate the clearance-inflated ghosts.
            if let Some((infringer, infringee, obstacle)) = self.terminal_blocker {
                let delta = infringee.center() - infringer.center();
                let len = (delta.x() * delta.x() + delta.y() * delta.y()).sqrt();
                if len > 0.0 {
                    let dir = (delta.x() / len, delta.y() / len);
                    let (o_lo, _) = extent_along(infringer.center(), dir, &infringee);
                    let (_, g_hi) = extent_along(infringer.center(), dir, &infringer);
                    let needed = g_hi - o_lo;
                    if needed > 0.0 {
                        push(obstacle, (dir.0 * needed, dir.1 * needed));
                    }
                }
            }
        } else {
            let rules = drawing.rules();
            let probe_net = self.net.unwrap_or(0);
            for window in self.path.windows(2) {
                let (a, layer_a) = window[0];
                let (b, layer_b) = window[1];
                let delta = b - a;
                let len = (delta.x() * delta.x() + delta.y() * delta.y()).sqrt();
                if len == 0.0 || layer_a != layer_b {
                    continue; // Degenerate or a layer transition (via).
                }
                let n = (-delta.y() / len, delta.x() / len);
                let corridor_width = band_half_width * 2.0 + 2.0 * rules.largest_clearance(None);

                for obstacle in drawing.cut(Line::new(a, b), corridor_width, layer_a) {
                    let blocker_net = obstacle.primitive_ref(drawing).maybe_net().unwrap_or(0);
                    let layer = Some(layer_a);
                    let clearance = rules
                        .clearance(
                            &Conditions {
                                net: probe_net,
                                maybe_region: None,
                                maybe_layer: layer,
                            },
                            &Conditions {
                                net: blocker_net,
                                maybe_region: None,
                                maybe_layer: layer,
                            },
                        )
                        .unwrap_or(0.0);
                    let half_width = band_half_width + clearance;
                    let blocker_primitive = obstacle.primitive_ref(drawing);
                    let shape = blocker_primitive.shape();
                    if let Some(shift) = clearing_shift(a, n, half_width, shape.center(), &shape) {
                        push(obstacle, shift);
                    }
                }
            }
        }
        blockers
    }
}

impl<M: AccessMesadata + Clone> Autorouter<M> {
    /// Probes every unrouted ratline with `relax_rules` applied to the
    /// board's rules (typically `override_clearances(0.0)`), returning a
    /// [`ChannelProbe`] per ratline. The board is unchanged afterwards.
    pub fn probe_channels(
        &mut self,
        router_options: RouterOptions,
        relax_rules: impl FnOnce(&mut M),
    ) -> Vec<ChannelProbe> {
        let unrouted = self.unrouted_ratlines();
        self.probe_ratlines(&unrouted, router_options, relax_rules)
    }

    /// Probes only the ratlines covered by `selection`, under relaxed
    /// rules — e.g. to seed congestion for a subset run without
    /// penalizing corridors demanded only by unselected ratlines.
    /// The board is unchanged afterwards.
    pub fn probe_selection(
        &mut self,
        selection: &PinSelection,
        router_options: RouterOptions,
        relax_rules: impl FnOnce(&mut M),
    ) -> Vec<ChannelProbe> {
        let ratlines: Vec<RatlineUid> = self.selected_ratlines(selection, 0).into_iter().collect();
        self.probe_ratlines(&ratlines, router_options, relax_rules)
    }

    /// Probes the given ratlines with `relax_rules` applied to the
    /// board's rules, returning a [`ChannelProbe`] per ratline in the
    /// order given. The board is unchanged afterwards.
    pub fn probe_ratlines(
        &mut self,
        ratlines: &[RatlineUid],
        router_options: RouterOptions,
        relax_rules: impl FnOnce(&mut M),
    ) -> Vec<ChannelProbe> {
        let pairs: Vec<_> = ratlines
            .iter()
            .map(|&uid| {
                let (origin, destination) = uid.ref_(self).endpoint_dots();
                (uid, origin, destination)
            })
            .collect();
        self.probe_ratlines_dots(&pairs, router_options, relax_rules)
    }

    /// Probes the given `(ratline, origin, destination)` triples under
    /// `relax_rules` — for ratlines whose effective endpoints have moved
    /// (e.g. fanout vias), the caller supplies the live terminating dots
    /// rather than the pad dots. The board is unchanged afterwards.
    pub fn probe_ratlines_dots(
        &mut self,
        pairs: &[(RatlineUid, FixedDotIndex, FixedDotIndex)],
        router_options: RouterOptions,
        relax_rules: impl FnOnce(&mut M),
    ) -> Vec<ChannelProbe> {
        let saved_rules = self.board.layout().drawing().rules().clone();
        relax_rules(self.board.layout_mut().rules_mut());

        let mut probes = Vec::with_capacity(pairs.len());
        // The board is reverted between probes, so one set of prenavmesh
        // bases serves every ratline.
        let mut prenavmesh_bases = PrenavmeshBases::default();

        for &(uid, origin, destination) in pairs {
            probes.push(self.probe_one(
                uid,
                origin,
                destination,
                router_options,
                &mut prenavmesh_bases,
            ));
        }

        *self.board.layout_mut().rules_mut() = saved_rules;
        probes
    }

    /// Probes one ratline under the currently installed rules, between
    /// the given endpoint dots. Cross-layer endpoints (needing a via
    /// the planar probe cannot express) yield an empty-path probe.
    fn probe_one(
        &mut self,
        _uid: RatlineUid,
        origin: FixedDotIndex,
        destination: FixedDotIndex,
        router_options: RouterOptions,
        prenavmesh_bases: &mut PrenavmeshBases,
    ) -> ChannelProbe {
        let (net, same_layer) = {
            let drawing = self.board.layout().drawing();
            (
                origin.primitive_ref(drawing).maybe_net(),
                origin.primitive_ref(drawing).layer() == destination.primitive_ref(drawing).layer(),
            )
        };
        if !same_layer {
            return ChannelProbe {
                net,
                endpoints: (origin, destination),
                path: vec![],
                terminal_blocker: None,
            };
        }

        let (path, terminal_blocker) =
            self.probe_ratline(origin, destination, router_options, prenavmesh_bases);
        ChannelProbe {
            net,
            endpoints: (origin, destination),
            path,
            terminal_blocker,
        }
    }

    /// The unrouted ratlines of the principal-layer ratsnest, in edge
    /// order — the work list shared by the sequential and parallel
    /// probe drivers.
    fn unrouted_ratlines(&self) -> Vec<RatlineUid> {
        let ratsnest = self.ratsnests.on_principal_layer(0);
        ratsnest
            .graph()
            .edge_indices()
            .filter(|&index| {
                ratsnest
                    .graph()
                    .edge_weight(index)
                    .is_some_and(|weight| weight.band_termseg.is_none())
            })
            .map(|index| RatlineUid {
                principal_layer: 0,
                index,
            })
            .collect()
    }

    /// Parallel variant of [`Self::probe_channels`]: each worker
    /// clones this autorouter, applies `relax_rules` to its own board,
    /// and probes a disjoint chunk of the unrouted ratlines. `self` is
    /// never mutated — the clone carries the rule relaxation and each
    /// worker reverts every probe band before the next, so chunks can
    /// not interfere. Results are returned in ratline order.
    ///
    /// With `jobs <= 1` this still pays for one clone; callers wanting
    /// no clone should use [`Self::probe_channels`] instead.
    pub fn probe_channels_par(
        &self,
        router_options: RouterOptions,
        relax_rules: &(impl Fn(&mut M) + Sync),
        jobs: usize,
    ) -> Vec<ChannelProbe>
    where
        M: Send + Sync,
    {
        let unrouted = self.unrouted_ratlines();
        self.probe_ratlines_par(&unrouted, router_options, relax_rules, jobs)
    }

    /// Parallel variant of [`Self::probe_selection`].
    pub fn probe_selection_par(
        &self,
        selection: &PinSelection,
        router_options: RouterOptions,
        relax_rules: &(impl Fn(&mut M) + Sync),
        jobs: usize,
    ) -> Vec<ChannelProbe>
    where
        M: Send + Sync,
    {
        let ratlines: Vec<RatlineUid> = self.selected_ratlines(selection, 0).into_iter().collect();
        self.probe_ratlines_par(&ratlines, router_options, relax_rules, jobs)
    }

    /// Parallel variant of [`Self::probe_ratlines`]: the given ratlines
    /// are partitioned across `jobs` cloned workers.
    pub fn probe_ratlines_par(
        &self,
        ratlines: &[RatlineUid],
        router_options: RouterOptions,
        relax_rules: &(impl Fn(&mut M) + Sync),
        jobs: usize,
    ) -> Vec<ChannelProbe>
    where
        M: Send + Sync,
    {
        let chunk_size = ratlines.len().div_ceil(jobs.max(1)).max(1);

        std::thread::scope(|scope| {
            let handles: Vec<_> = ratlines
                .chunks(chunk_size)
                .map(|chunk| {
                    scope.spawn(|| {
                        let mut worker = self.clone();
                        relax_rules(worker.board.layout_mut().rules_mut());
                        let mut prenavmesh_bases = PrenavmeshBases::default();
                        chunk
                            .iter()
                            .map(|&uid| {
                                let (origin, destination) = uid.ref_(&worker).endpoint_dots();
                                worker.probe_one(
                                    uid,
                                    origin,
                                    destination,
                                    router_options,
                                    &mut prenavmesh_bases,
                                )
                            })
                            .collect::<Vec<_>>()
                    })
                })
                .collect();

            handles
                .into_iter()
                .flat_map(|handle| {
                    handle.join().unwrap_or_else(|payload| {
                        let msg = payload
                            .downcast_ref::<&str>()
                            .copied()
                            .or_else(|| payload.downcast_ref::<String>().map(String::as_str));
                        eprintln!("probe worker panicked: {msg:?}");
                        std::panic::resume_unwind(payload)
                    })
                })
                .collect()
        })
    }

    /// Routes one ratline's endpoints with the current (relaxed) rules.
    /// On success returns the band's waypoint polyline and reverts the
    /// band; on failure returns the terminal blocker.
    fn probe_ratline(
        &mut self,
        origin: FixedDotIndex,
        destination: FixedDotIndex,
        router_options: RouterOptions,
        prenavmesh_bases: &mut PrenavmeshBases,
    ) -> ProbeOutcome {
        let Ok(navmesh) =
            prenavmesh_bases.navmesh(self.board.layout(), origin, destination, router_options)
        else {
            log::debug!("probe: navmesh construction failed");
            return (vec![], None);
        };

        // The router borrows the layout mutably, so scope it: `route`
        // is self-contained and survives to be read after the borrow
        // ends and the probe band is reverted.
        let (route, termseg) = {
            let mut router = Router::new(self.board.layout_mut(), router_options);
            let mut route = RouteStepper::new_from_navmesh(
                &mut router,
                LayoutEdit::new(),
                navmesh,
                router_options.routed_band_width,
            );

            let mut termseg = None;
            loop {
                match route.step(&mut router) {
                    Ok(ControlFlow::Continue(())) => {}
                    Ok(ControlFlow::Break(done)) => {
                        termseg = Some(done);
                        break;
                    }
                    Err(err) => {
                        log::debug!("probe: route step error: {err}");
                        break;
                    }
                }
            }
            (route, termseg)
        };

        if termseg.is_none() {
            log::debug!(
                "probe: route failed, obstacles={:?}",
                route.obstacles().len()
            );
        }

        if let Some(termseg) = termseg {
            let path = self.band_waypoints(termseg);
            let (_, navcord, ..) = route.dissolve();
            self.board.apply_edit(
                &BoardEdit::new_from_edits(BoardDataEdit::new(), navcord.recorder).reverse(),
            );
            (path, None)
        } else {
            // The navcord has already stepped back over the partial band.
            let terminal_blocker = route
                .ghosts()
                .first()
                .copied()
                .zip(route.ghosts().get(1).copied())
                .zip(route.obstacles().first().copied())
                .map(|((infringer, infringee), obstacle)| (infringer, infringee, obstacle));
            (vec![], terminal_blocker)
        }
    }

    /// Collects the waypoint polyline of a freshly routed band: each
    /// loose's center and layer, in path order.
    fn band_waypoints(&self, termseg: BandTermsegIndex) -> Vec<(Point, usize)> {
        let drawing = self.board.layout().drawing();
        let mut path = vec![];

        let mut maybe_loose: Option<LooseIndex> = Some(match termseg {
            BandTermsegIndex::Lone(seg) => {
                let primitive = seg.primitive_ref(drawing);
                let layer = primitive.layer();
                match primitive.shape() {
                    PrimitiveShape::Seg(seg) => {
                        path.push((seg.from, layer));
                        path.push((seg.to, layer));
                    }
                    shape => path.push((shape.center(), layer)),
                }
                return path;
            }
            BandTermsegIndex::Seq(first) => first.into(),
        });

        let mut prev = None;
        while let Some(loose) = maybe_loose {
            let primitive = loose.primitive_ref(drawing);
            let layer = primitive.layer();
            let shape = primitive.shape();
            path.push((shape.center(), layer));

            let prev_prev = prev;
            prev = maybe_loose;
            maybe_loose = drawing.loose(loose).next_loose(prev_prev);
        }

        path
    }
}

#[cfg(test)]
mod tests {
    use std::{fs::File, io::BufReader};

    use super::*;
    use crate::{
        board::edit::BoardEdit,
        specctra::{design::SpecctraDesign, mesadata::SpecctraMesadata},
    };

    /// The parallel probe must return exactly what the sequential one
    /// does — same order, same contents — while leaving the board
    /// untouched.
    #[test]
    fn parallel_probe_matches_sequential() {
        let design = || {
            let file = File::open("tests/unilayer/0603_breakout/0603_breakout.dsn").unwrap();
            SpecctraDesign::load(BufReader::new(file)).unwrap()
        };
        let mut sequential_autorouter =
            Autorouter::new(design().make_board(&mut BoardEdit::new())).unwrap();
        let parallel_autorouter =
            Autorouter::new(design().make_board(&mut BoardEdit::new())).unwrap();

        let options = RouterOptions {
            wrap_around_bands: true,
            squeeze_through_under_bends: false,
            routed_band_width: 100.0,
        };
        let sequential =
            sequential_autorouter.probe_channels(options, |rules| rules.override_clearances(0.0));
        let parallel = parallel_autorouter.probe_channels_par(
            options,
            &|rules: &mut SpecctraMesadata| rules.override_clearances(0.0),
            3,
        );
        assert_eq!(sequential, parallel);
    }

    /// `ChannelProbe::blockers` reports corridor blockers at real
    /// clearance, and `seed_congestion` keeps only fixed primitives —
    /// loose indices are recycled as bands come and go, so seeding them
    /// could misattribute the count.
    #[test]
    fn probe_blockers_seed_fixed_congestion_only() {
        let file = File::open("tests/unilayer/0603_breakout/0603_breakout.dsn").unwrap();
        let mut autorouter = Autorouter::new(
            SpecctraDesign::load(BufReader::new(file))
                .unwrap()
                .make_board(&mut BoardEdit::new()),
        )
        .unwrap();

        let options = RouterOptions {
            wrap_around_bands: true,
            squeeze_through_under_bends: false,
            routed_band_width: 100.0,
        };
        let probes = autorouter.probe_channels(options, |rules| rules.override_clearances(0.0));

        let blockers: Vec<PrimitiveIndex> = {
            let drawing = autorouter.board().layout().drawing();
            probes
                .iter()
                .flat_map(|probe| probe.blockers(drawing, 50.0))
                .map(|blocker| blocker.obstacle)
                .collect()
        };
        let fixed_blockers = blockers
            .iter()
            .filter(|index| {
                matches!(
                    index,
                    PrimitiveIndex::FixedDot(_)
                        | PrimitiveIndex::FixedSeg(_)
                        | PrimitiveIndex::FixedBend(_)
                )
            })
            .count();

        autorouter.seed_congestion(blockers);

        assert_eq!(
            autorouter.congestion_seed().values().sum::<usize>(),
            fixed_blockers,
        );
        assert!(autorouter.congestion_seed().keys().all(|index| matches!(
            index,
            PrimitiveIndex::FixedDot(_)
                | PrimitiveIndex::FixedSeg(_)
                | PrimitiveIndex::FixedBend(_)
        )));
    }
}
