// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::{collections::BTreeMap, ops::ControlFlow};

use derive_getters::Getters;
use geo::algorithm::line_measures::{Distance, Euclidean};
use petgraph::data::DataMap;
use serde::{Deserialize, Serialize};

use crate::{
    drawing::{
        band::BandTermsegIndex,
        dot::FixedDotIndex,
        graph::{MakePrimitiveRef, PrimitiveIndex},
        primitive::MakePrimitiveShape,
        rules::AccessRules,
    },
    geometry::{primitive::PrimitiveShape, shape::AccessShape},
    graph::GenericIndex,
    layout::{CompoundEntryLabel, CompoundWeight, Layout, LayoutEdit},
    router::navmesh::BinavnodeNodeIndex,
};

use super::{
    draw::DrawException,
    navcord::{GhostReplay, Navcord, PathCane, ProbeMemoEntry, ProbeVerdict},
    navcorder::{Navcorder, NavcorderException},
    navmesh::{Navmesh, NavmeshError, NavnodeIndex},
    route::RouteStepper,
    thetastar::{PathTracker, ThetastarStrategy},
};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct RouterOptions {
    pub routed_band_width: f64,
    pub wrap_around_bands: bool,
    pub squeeze_through_under_bends: bool,
}

#[derive(Debug)]
pub struct RouterThetastarStrategy<'a, R> {
    pub layout: &'a mut Layout<R>,
    pub navcord: &'a mut Navcord,
    pub destination: FixedDotIndex,
    pub probe_ghosts: Vec<PrimitiveShape>,
    pub probe_obstacles: Vec<PrimitiveIndex>,
    /// Scratch buffer for `reconstruct_path_to`, reused across visits
    /// to avoid a `Vec` allocation per navnode visit.
    pub path_scratch: Vec<NavnodeIndex>,
    /// Extra path costs charged for navigating to (wrapping) a navnode's
    /// primitive (negotiated congestion).
    pub obstacle_penalties: Option<&'a BTreeMap<PrimitiveIndex, f64>>,
}

impl<'a, R> RouterThetastarStrategy<'a, R> {
    pub fn new(layout: &'a mut Layout<R>, navcord: &'a mut Navcord, target: FixedDotIndex) -> Self {
        Self {
            layout,
            navcord,
            destination: target,
            probe_ghosts: vec![],
            probe_obstacles: vec![],
            path_scratch: vec![],
            obstacle_penalties: None,
        }
    }
}

impl<R: AccessRules> ThetastarStrategy<Navmesh, f64, BandTermsegIndex>
    for RouterThetastarStrategy<'_, R>
{
    fn visit_navnode(
        &mut self,
        navmesh: &Navmesh,
        navnode: NavnodeIndex,
        tracker: &PathTracker<Navmesh>,
    ) -> Result<Option<BandTermsegIndex>, ()> {
        crate::metrics::incr(&crate::metrics::METRICS.visit_navnode_calls);
        tracker.reconstruct_path_into(&mut self.path_scratch, navnode);
        let new_path = self.path_scratch.as_slice();

        if navnode == navmesh.destination_navnode() {
            self.layout
                .rework_path(navmesh, self.navcord, &new_path[..new_path.len() - 1])
                .map_err(|_| ())?;

            // The path was explored purely virtually; now that it is
            // being committed, draw its canes into the board for real
            // and then finish the band.
            self.navcord
                .materialize(self.layout, navmesh)
                .map_err(|_| ())?;

            // Set navcord members for consistency. The code would probably work
            // without this, since A* will terminate now anyway.
            self.navcord.maybe_final_termseg = Some(
                self.layout
                    .finish(navmesh, self.navcord, self.destination)
                    .map_err(|_| ())?,
            );
            self.navcord.push_path(navnode);

            Ok(self.navcord.maybe_final_termseg)
        } else {
            self.layout
                .rework_path(navmesh, self.navcord, &new_path[..])
                .map_or(Err(()), |_| Ok(None))
        }
    }

    fn place_probe_to_navnode(
        &mut self,
        navmesh: &Navmesh,
        maybe_initial_parent_navnode: Option<NavnodeIndex>,
        probed_navnode: NavnodeIndex,
    ) -> ControlFlow<Option<f64>> {
        crate::metrics::incr(&crate::metrics::METRICS.probes);

        let memo_key = (maybe_initial_parent_navnode, probed_navnode);

        if let Some(entry) = self
            .navcord
            .probe_memo
            .get(&self.navcord.path_id())
            .and_then(|by_path| by_path.get(&memo_key))
            .cloned()
        {
            crate::metrics::incr(&crate::metrics::METRICS.probes_memoized);
            self.probe_ghosts = entry.ghosts;
            self.probe_obstacles = entry.obstacles;

            return match entry.verdict {
                ProbeVerdict::Drawn { length, replay } => {
                    // Replay the successful probe's virtual step so that
                    // `.step_back(..)` undoes it, exactly as it would
                    // undo a freshly evaluated probe.
                    match replay {
                        GhostReplay::Wrap(cane) => {
                            self.navcord.canes.push(PathCane::Virtual(cane))
                        }
                        GhostReplay::Finish => self.navcord.virtual_finish = true,
                    }
                    self.navcord.push_path(probed_navnode);
                    ControlFlow::Break(Some(length))
                }
                ProbeVerdict::Backtrackable => ControlFlow::Continue(()),
                ProbeVerdict::Terminal => ControlFlow::Break(None),
            };
        }

        let ret = self.place_probe_to_navnode_intern(
            navmesh,
            maybe_initial_parent_navnode,
            probed_navnode,
        );
        let verdict = match ret {
            ControlFlow::Break(Some(length)) => {
                crate::metrics::incr(&crate::metrics::METRICS.probes_los_ok);
                ProbeVerdict::Drawn {
                    length,
                    replay: self.navcord.last_replay().unwrap(),
                }
            }
            ControlFlow::Break(None) => {
                crate::metrics::incr(&crate::metrics::METRICS.probes_failed);
                ProbeVerdict::Terminal
            }
            ControlFlow::Continue(()) => {
                crate::metrics::incr(&crate::metrics::METRICS.probes_backtracked);
                ProbeVerdict::Backtrackable
            }
        };
        self.navcord
            .probe_memo
            .entry(self.navcord.path_id())
            .or_default()
            .insert(
                memo_key,
                ProbeMemoEntry {
                    verdict,
                    ghosts: self.probe_ghosts.clone(),
                    obstacles: self.probe_obstacles.clone(),
                },
            );
        ret
    }

    fn remove_probe(&mut self, _navmesh: &Navmesh) {
        self.navcord.step_back(self.layout);
    }

    fn estimate_cost_to_goal(&mut self, navmesh: &Navmesh, vertex: NavnodeIndex) -> f64 {
        let start_point = PrimitiveIndex::from(navmesh.node_weight(vertex).unwrap().binavnode)
            .primitive_ref(self.layout.drawing())
            .shape()
            .center();
        let end_point = self
            .layout
            .drawing()
            .primitive_ref(self.destination)
            .shape()
            .center();

        Euclidean::distance(&end_point, &start_point)
    }
}

impl<R: AccessRules> RouterThetastarStrategy<'_, R> {
    fn place_probe_to_navnode_intern(
        &mut self,
        navmesh: &Navmesh,
        maybe_initial_parent_navnode: Option<NavnodeIndex>,
        probed_navnode: NavnodeIndex,
    ) -> ControlFlow<Option<f64>> {
        // Probes are speculative steps that are always undone right
        // after, and all steps are evaluated virtually without drawing;
        // the virtual state `.step_to(..)` pushes is undone by the
        // following `.step_back(..)`.
        let result = self
            .navcord
            .step_to(self.layout, navmesh, probed_navnode);

        ControlFlow::Break(match result {
            Ok(probe_length) => Some(
                probe_length
                    + self
                        .obstacle_penalties
                        .and_then(|penalties| {
                            penalties
                                .get(&PrimitiveIndex::from(
                                    navmesh.node_weight(probed_navnode).unwrap().binavnode,
                                ))
                                .copied()
                        })
                        .unwrap_or(0.0),
            ),
            Err(err) => {
                if let NavcorderException::CannotDraw(draw_err) = err {
                    let layout_err = match draw_err {
                        DrawException::NoBitangents(..) => return ControlFlow::Break(None),
                        DrawException::CannotFinishIn(.., layout_err) => layout_err,
                        DrawException::CannotWrapAround(.., layout_err) => layout_err,
                    };

                    let Some((infringer_ghost, infringee_ghost, obstacle)) =
                        layout_err.maybe_ghosts_and_obstacle()
                    else {
                        return ControlFlow::Break(None);
                    };

                    self.probe_ghosts = vec![*infringer_ghost, *infringee_ghost];
                    self.probe_obstacles = vec![obstacle];

                    let Some(initial_parent_navnode) = maybe_initial_parent_navnode else {
                        return ControlFlow::Break(None);
                    };

                    let initial_parent_binavnode = navmesh
                        .node_weight(initial_parent_navnode)
                        .unwrap()
                        .binavnode;

                    let drawing = self.layout.drawing();
                    let compounds = |binavnode: BinavnodeNodeIndex| -> Box<
                        dyn Iterator<Item = (CompoundEntryLabel, GenericIndex<CompoundWeight>)> + '_,
                    > {
                        match binavnode {
                            BinavnodeNodeIndex::FixedDot(dot) => {
                                Box::new(drawing.compounds(dot))
                            }
                            BinavnodeNodeIndex::FixedBend(_bend) => {
                                todo!()
                            }
                            BinavnodeNodeIndex::LooseBend(bend) => {
                                Box::new(drawing.compounds(bend))
                            }
                        }
                    };

                    let from_binavnode = navmesh
                        .node_weight(*self.navcord.path.last().unwrap())
                        .unwrap()
                        .binavnode;

                    if compounds(initial_parent_binavnode).any(|compound1| {
                        compounds(from_binavnode).any(|compound2| compound1 == compound2)
                    }) {
                        // If we have failed to do a line-of-sight draw, but
                        // the current probe source navnode and the initial
                        // parent navnode are on the same compound (TODO:
                        // narrow this down from compound to poly), we continue
                        // backtracking. When ControlFlow::Continue(()) is
                        // returned here, the modified Theta* algorithm instead
                        // of visiting a new navedge recedes the navcord and
                        // with that the current probe source navnode.
                        return ControlFlow::Continue(());
                    }
                }
                None
            }
        })
    }
}

#[derive(Debug, Getters)]
pub struct Router<'a, R: AccessRules> {
    #[getter(skip)]
    layout: &'a mut Layout<R>,
    options: RouterOptions,
}

impl<'a, R: AccessRules> Router<'a, R> {
    pub fn new(layout: &'a mut Layout<R>, options: RouterOptions) -> Self {
        Self { layout, options }
    }

    pub fn route(
        &mut self,
        recorder: LayoutEdit,
        origin: FixedDotIndex,
        destination: FixedDotIndex,
        width: f64,
    ) -> Result<RouteStepper, NavmeshError> {
        RouteStepper::new(self, recorder, origin, destination, width)
    }

    pub fn layout_mut(&mut self) -> &mut Layout<R> {
        self.layout
    }

    pub fn layout(&self) -> &Layout<R> {
        self.layout
    }
}
