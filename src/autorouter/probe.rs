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

use geo::Point;

use crate::{
    autorouter::{ratline::RatlineUid, Autorouter},
    board::{
        edit::{BoardDataEdit, BoardEdit},
        AccessMesadata,
    },
    drawing::{
        band::BandTermsegIndex,
        graph::{GetMaybeNet, MakePrimitiveRef, PrimitiveIndex},
        loose::{GetPrevNextLoose, LooseIndex},
        primitive::MakePrimitiveShape,
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
    pub endpoints: (
        crate::drawing::dot::FixedDotIndex,
        crate::drawing::dot::FixedDotIndex,
    ),
    /// Ordered waypoints of the demand path as `(center, layer)`. Empty
    /// when the probe could not produce a path even under relaxed rules.
    pub path: Vec<(Point, usize)>,
    /// On failure, the terminal `(infringer_ghost, infringee_ghost,
    /// obstacle)`: where the band wanted to be, the blocking primitive's
    /// shape, and the blocking primitive itself.
    pub terminal_blocker: Option<(PrimitiveShape, PrimitiveShape, PrimitiveIndex)>,
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
        let saved_rules = self.board.layout().drawing().rules().clone();
        relax_rules(self.board.layout_mut().rules_mut());

        let unrouted = self.unrouted_ratlines();

        let mut probes = Vec::with_capacity(unrouted.len());
        // The board is reverted between probes, so one set of prenavmesh
        // bases serves every ratline.
        let mut prenavmesh_bases = PrenavmeshBases::default();

        for uid in unrouted {
            probes.push(self.probe_one(uid, router_options, &mut prenavmesh_bases));
        }

        *self.board.layout_mut().rules_mut() = saved_rules;
        probes
    }

    /// Probes one ratline under the currently installed rules.
    /// Cross-layer ratlines (needing a via the planar probe cannot
    /// express) yield an empty-path probe.
    fn probe_one(
        &mut self,
        uid: RatlineUid,
        router_options: RouterOptions,
        prenavmesh_bases: &mut PrenavmeshBases,
    ) -> ChannelProbe {
        let (origin, destination) = uid.ref_(self).endpoint_dots();
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
        let chunk_size = unrouted.len().div_ceil(jobs.max(1)).max(1);

        std::thread::scope(|scope| {
            let handles: Vec<_> = unrouted
                .chunks(chunk_size)
                .map(|chunk| {
                    scope.spawn(|| {
                        let mut worker = self.clone();
                        relax_rules(worker.board.layout_mut().rules_mut());
                        let mut prenavmesh_bases = PrenavmeshBases::default();
                        chunk
                            .iter()
                            .map(|&uid| {
                                worker.probe_one(uid, router_options, &mut prenavmesh_bases)
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
        origin: crate::drawing::dot::FixedDotIndex,
        destination: crate::drawing::dot::FixedDotIndex,
        router_options: RouterOptions,
        prenavmesh_bases: &mut PrenavmeshBases,
    ) -> (
        Vec<(Point, usize)>,
        Option<(PrimitiveShape, PrimitiveShape, PrimitiveIndex)>,
    ) {
        let Ok(navmesh) =
            prenavmesh_bases.navmesh(self.board.layout(), origin, destination, router_options)
        else {
            return (vec![], None);
        };

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
                Err(_) => break,
            }
        }

        // `route` is self-contained; the router borrow ends here so the
        // drawing can be read and the probe band reverted.
        drop(router);

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
}
