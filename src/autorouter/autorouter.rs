// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use derive_getters::Getters;
use geo::Point;
use petgraph::graph::NodeIndex;
use serde::{Deserialize, Serialize};
use spade::InsertionError;
use std::collections::{BTreeMap, BTreeSet};
use thiserror::Error;

use crate::{
    autorouter::{
        critical_autoroute::{CriticalAutorouteExecutionStepper, CriticalAutorouteOptions},
        multilayer_autoroute::MultilayerAutorouteOptions,
        multilayer_preconfigurer::MultilayerAutoroutePreconfigurerInput,
        multilayer_reconfigurator::MultilayerAutorouteReconfigurator,
        planar_preconfigurer::PlanarAutoroutePreconfigurerInput,
        planar_reconfigurator::PlanarAutorouteReconfigurator,
        ratsnests::Ratsnests,
    },
    board::{AccessMesadata, Board},
    drawing::{
        band::BandTermsegIndex,
        graph::{MakePrimitiveRef, PrimitiveIndex},
    },
    geometry::GetLayer,
    graph::MakeRef,
    layout::{via::ViaWeight, LayoutEdit, LayoutException},
    router::{navmesh::NavmeshError, ng, thetastar::ThetastarError, RouterOptions},
    stepper::TimeoutOptions,
    triangulation::GetTrianvertexNodeIndex,
};

use super::{
    measure_length::MeasureLengthExecutionStepper,
    place_via::PlaceViaExecutionStepper,
    pointroute::PointrouteExecutionStepper,
    ratline::RatlineUid,
    ratsnest::RatvertexNodeIndex,
    remove_bands::RemoveBandsExecutionStepper,
    selection::{BandSelection, PinSelection},
};

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub enum PresortBy {
    RatlineIntersectionCountAndLength,
    PairwiseDetours,
    /// Order each SCC's ratlines by endpoint crowding, most-crowded
    /// first: ratlines escaping dense pin regions route while their
    /// corridors are still open.
    RatlineEscapeDensity,
    /// Order each SCC's ratlines by corridor obstruction, most-blocked
    /// first: the straight origin→destination seg is queried against the
    /// R-tree at routing clearances and the number of primitives
    /// infringing it scores how constrained the ratline's corridor is.
    RatlineCorridorCongestion,
}

#[derive(Clone, Copy, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
pub enum ReconfigureBy {
    /// Directed rip-up-and-reroute driven by recorded blocker history
    /// (negotiated congestion), falling back to ordering permutations.
    #[default]
    NegotiatedCongestion,
    /// Ordering permutations only.
    SccPermutations,
}

fn default_congestion_penalty() -> f64 {
    500.0
}

fn default_pad_entry_min_angle() -> f64 {
    std::f64::consts::FRAC_PI_4
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct PlanarAutorouteOptions {
    pub principal_layer: usize,
    pub presort_by: PresortBy,
    pub permutate: bool,
    /// How the reconfigurer chooses the next ratline ordering after a
    /// routing failure.
    #[serde(default)]
    pub reconfigure_by: ReconfigureBy,
    /// Cost penalty applied to paths wrapping a fixed obstacle, per
    /// recorded block, in layout units.
    #[serde(default = "default_congestion_penalty")]
    pub congestion_penalty: f64,
    pub router: RouterOptions,
    pub timeout: TimeoutOptions,
    /// Maximum number of reconfigurations (e.g. ratline order
    /// permutations) before routing finishes early, keeping whatever has
    /// been routed so far. `None` means no limit.
    #[serde(default)]
    pub max_reconfigurations: Option<usize>,
    /// Runs pad-entry post-processing after routing finishes: band ends
    /// crossing a polygonal pad's boundary below
    /// [`PlanarAutorouteOptions::pad_entry_min_angle`] are swung to the
    /// last cane's alternate tangent where possible.
    #[serde(default)]
    pub improve_pad_entry: bool,
    /// Minimum angle, in radians, at which a trace may cross a
    /// polygonal pad's boundary. Used only when
    /// [`PlanarAutorouteOptions::improve_pad_entry`] is set.
    #[serde(default = "default_pad_entry_min_angle")]
    pub pad_entry_min_angle: f64,
    /// Lays escape stubs before routing: endpoint pads surrounded by
    /// other ratline endpoints get a fixed seg through the widest local
    /// gap to a fresh stub dot in open space, and their ratlines are
    /// remapped to terminate at the stub. Stubs are laid
    /// most-crowded-first so interior pads claim escape corridors
    /// before outer pads can wall them in.
    #[serde(default)]
    pub escape_stubs: bool,
    /// Per-attempt cap on router steps spent on a single ratline's
    /// search. Without one, a ratline whose frontier keeps expanding
    /// can consume the entire command budget before later ratlines are
    /// ever attempted; exceeding the cap fails the attempt like a
    /// frontier-exhausted search, letting reconfiguration move on.
    /// `None` leaves attempts unbounded.
    #[serde(default)]
    pub per_ratline_step_limit: Option<usize>,
    /// Adaptive per-attempt cap: the attempt fails once it has consumed
    /// `factor × navmesh nodes` steps, sized to the ratline's own
    /// search space rather than a global constant — sparse nets get a
    /// small budget, dense escapes get room, and a search that cycles
    /// its frontier many times over is almost all rework churn.
    /// Ignored when `per_ratline_step_limit` is set (explicit wins).
    #[serde(default)]
    pub navmesh_step_factor: Option<f64>,
    /// When a ratline fails on bands routed earlier, rip up only the
    /// blocking bands and requeue their ratlines right after the failed
    /// one, instead of backtracing to the first ordering difference and
    /// losing every band routed in between. Used only when
    /// [`ReconfigureBy::NegotiatedCongestion`] is selected.
    #[serde(default)]
    pub surgical_ripup: bool,
    /// Seed for a deterministic within-SCC ratline shuffle applied at
    /// preconfiguration; `0` keeps the deterministic ordering. Parallel
    /// exploration runs give each attempt a different seed so they
    /// traverse different ordering trajectories.
    #[serde(default)]
    pub shuffle_seed: u64,
}

impl PlanarAutorouteOptions {
    /// The per-attempt step cap for a search over `navnodes` navnodes:
    /// the explicit `per_ratline_step_limit` wins; otherwise the cap
    /// scales with the ratline's own search space when
    /// `navmesh_step_factor` is set. `None` leaves attempts unbounded.
    pub fn step_limit_for(&self, navnodes: usize) -> Option<usize> {
        self.per_ratline_step_limit.or_else(|| {
            self.navmesh_step_factor
                .map(|factor| (factor * navnodes as f64) as usize)
        })
    }
}

#[derive(Error, Debug, Clone)]
pub enum AutorouterError {
    #[error("nothing to route")]
    NothingToRoute,
    #[error(transparent)]
    Navmesh(#[from] NavmeshError),
    #[error("routing failed: {0}")]
    Thetastar(#[from] ThetastarError),
    #[error("TopoNavmesh generation failed: {0}")]
    TopoNavmeshGeneration(#[from] ng::NavmeshCalculationError),
    #[error("could not place via")]
    CouldNotPlaceVia(#[from] LayoutException),
    #[error("could not remove band")]
    CouldNotRemoveBand(BandTermsegIndex),
    #[error("need exactly two ratlines")]
    NeedExactlyTwoRatlines,
    #[error("nothing to undo for permutation")]
    NothingToUndoForReconfiguration,
}

/// One recorded blocker attribution: a routing failure of `failed`
/// attributed to `obstacle`; when the obstacle is part of a band routed
/// for another ratline, `blocker` names that ratline.
///
/// Diagnostic-only — written by the negotiated-congestion reconfigurer,
/// drained by frontends for blocker reports. Loose-primitive indices in
/// `obstacle` name the geometry that was live at record time; they are
/// not stable across reconfigurations.
#[derive(Clone, Copy, Debug)]
pub struct BlockerRecord {
    /// Monotonic sequence number of this record.
    pub seq: usize,
    /// Configuration position of the failing ratline.
    pub position: usize,
    /// The ratline whose route failed.
    pub failed: RatlineUid,
    /// The primitive that blocked the route.
    pub obstacle: PrimitiveIndex,
    /// The ratline owning the band the obstacle belongs to, if any.
    pub blocker: Option<RatlineUid>,
}

#[derive(Clone, Getters)]
pub struct Autorouter<M> {
    pub(super) board: Board<M>,
    pub(super) ratsnests: Ratsnests,
    /// Congestion counts pre-seeded before routing — e.g. from relaxed
    /// channel probes (see [`Autorouter::probe_channels`] and
    /// `ChannelProbe::blockers`). The negotiated-congestion reconfigurer
    /// folds these into its live counts, giving repeatedly blocked
    /// primitives a head start on their path penalty instead of learning
    /// them from failures alone.
    pub(super) congestion_seed: BTreeMap<PrimitiveIndex, usize>,
    /// Failure-attribution log — one [`BlockerRecord`] per blocking
    /// obstacle per recorded routing failure.
    pub(super) blocker_log: Vec<BlockerRecord>,
}

impl<M: AccessMesadata> Autorouter<M> {
    pub fn new(board: Board<M>) -> Result<Self, InsertionError> {
        let ratsnests = Ratsnests::new(&board)?;
        Ok(Self {
            board,
            ratsnests,
            congestion_seed: BTreeMap::new(),
            blocker_log: Vec::new(),
        })
    }

    /// Bumps the seeded congestion count of each given primitive. Only
    /// fixed primitives are counted: loose indices are recycled as
    /// bands are undone and redrawn, so a loose key could outlive the
    /// geometry it named and misattribute the count.
    pub fn seed_congestion(&mut self, obstacles: impl IntoIterator<Item = PrimitiveIndex>) {
        for obstacle in obstacles {
            if matches!(
                obstacle,
                PrimitiveIndex::FixedDot(_)
                    | PrimitiveIndex::FixedSeg(_)
                    | PrimitiveIndex::FixedBend(_)
            ) {
                *self.congestion_seed.entry(obstacle).or_insert(0) += 1;
            }
        }
    }

    pub fn board_mut(&mut self) -> &mut Board<M> {
        &mut self.board
    }

    pub fn pointroute(
        &mut self,
        selection: &PinSelection,
        point: Point,
        options: PlanarAutorouteOptions,
    ) -> Result<PointrouteExecutionStepper, AutorouterError> {
        let ratvertex = self
            .find_selected_ratvertex(selection, options.principal_layer)
            .unwrap();
        let origin_dot = match self
            .ratsnests
            .on_principal_layer_mut(options.principal_layer)
            .graph()
            .node_weight(ratvertex)
            .unwrap()
            .node_index()
        {
            RatvertexNodeIndex::FixedDot(dot) => dot,
            RatvertexNodeIndex::Poly(poly) => poly.ref_(self.board.layout()).apex(),
        };

        PointrouteExecutionStepper::new(self, origin_dot, point, options)
    }

    pub fn undo_pointroute(&mut self, band: BandTermsegIndex) -> Result<(), AutorouterError> {
        self.board
            .layout_mut()
            .remove_band(&mut LayoutEdit::new(), band)
            .map_err(|_| AutorouterError::CouldNotRemoveBand(band))
    }

    pub fn multilayer_autoroute(
        &mut self,
        selection: &PinSelection,
        options: MultilayerAutorouteOptions,
    ) -> Result<MultilayerAutorouteReconfigurator, AutorouterError> {
        MultilayerAutorouteReconfigurator::new(
            self,
            MultilayerAutoroutePreconfigurerInput {
                ratlines: self.selected_ratlines(selection, options.planar.principal_layer),
            },
            options,
        )
    }

    pub fn planar_autoroute(
        &mut self,
        selection: &PinSelection,
        options: PlanarAutorouteOptions,
    ) -> Result<PlanarAutorouteReconfigurator, AutorouterError> {
        PlanarAutorouteReconfigurator::new(
            self,
            PlanarAutoroutePreconfigurerInput {
                ratlines: self.selected_planar_ratlines(selection, options.principal_layer),
                terminating_dot_map: BTreeMap::new(),
            },
            options,
        )
    }

    /// Critical-first autorouting: probes the selected ratlines under
    /// relaxed rules, commits the hardest ones first with carved
    /// corridors and rip-up protection, and iteratively re-identifies
    /// newly difficult ratlines on stalls. `options.multilayer`
    /// selects the inner routing machinery; when it is `Some`, the
    /// selection covers every principal layer's ratlines, planar only
    /// the principal layer's same-layer ratlines.
    pub fn critical_autoroute(
        &mut self,
        selection: &PinSelection,
        options: CriticalAutorouteOptions,
    ) -> Result<CriticalAutorouteExecutionStepper, AutorouterError>
    where
        M: Clone,
    {
        let principal_layer = options
            .multilayer
            .map(|options| options.planar.principal_layer)
            .unwrap_or(options.planar.principal_layer);
        let ratlines = if options.multilayer.is_some() {
            self.selected_ratlines(selection, principal_layer)
        } else {
            self.selected_planar_ratlines(selection, principal_layer)
        };
        CriticalAutorouteExecutionStepper::new(self, ratlines, options)
    }

    pub fn topo_autoroute(
        &mut self,
        selection: &PinSelection,
        allowed_edges: BTreeSet<ng::PieEdgeIndex>,
        active_layer: usize,
        width: f64,
        init_navmesh: Option<ng::PieNavmesh>,
    ) -> Result<ng::AutorouteExecutionStepper<M>, AutorouterError>
    where
        M: Clone,
    {
        self.topo_autoroute_ratlines(
            self.selected_ratlines(selection, active_layer)
                .into_iter()
                .collect(),
            allowed_edges,
            active_layer,
            width,
            init_navmesh,
        )
    }

    pub(super) fn topo_autoroute_ratlines(
        &mut self,
        ratlines: Vec<RatlineUid>,
        allowed_edges: BTreeSet<ng::PieEdgeIndex>,
        active_layer: usize,
        width: f64,
        init_navmesh: Option<ng::PieNavmesh>,
    ) -> Result<ng::AutorouteExecutionStepper<M>, AutorouterError>
    where
        M: Clone,
    {
        let navmesh = if let Some(x) = init_navmesh {
            x
        } else {
            ng::calculate_navmesh(&self.board, active_layer)?
        };

        let mut got_any_valid_goals = false;

        use ng::pie::NavmeshIndex;

        let ret = ng::AutorouteExecutionStepper::new(
            self.board.layout(),
            &navmesh,
            self.board
                .bands_by_id()
                .iter()
                .map(|(&k, &v)| (k, v))
                .collect(),
            active_layer,
            allowed_edges,
            ratlines.into_iter().filter_map(|ratline| {
                let (origin, destination) = ratline.ref_(self).endpoint_dots();

                if navmesh
                    .as_ref()
                    .node_data(&NavmeshIndex::Primal(origin))
                    .is_none()
                    || navmesh
                        .as_ref()
                        .node_data(&NavmeshIndex::Primal(destination))
                        .is_none()
                {
                    // e.g. due to wrong active layer
                    return None;
                }

                if self.board.band_between_nodes(origin, destination).is_some() {
                    // already connected
                    return None;
                }

                got_any_valid_goals = true;

                Some(ng::Goal {
                    source: origin,
                    target: destination,
                    width,
                })
            }),
        );

        if !got_any_valid_goals {
            Err(AutorouterError::NothingToRoute)
        } else {
            Ok(ret)
        }
    }

    pub fn place_via(
        &self,
        weight: ViaWeight,
    ) -> Result<PlaceViaExecutionStepper, AutorouterError> {
        PlaceViaExecutionStepper::new(weight)
    }

    pub fn undo_place_via(&mut self, _weight: ViaWeight) {
        todo!();
    }

    pub fn remove_bands(
        &self,
        selection: &BandSelection,
    ) -> Result<RemoveBandsExecutionStepper, AutorouterError> {
        RemoveBandsExecutionStepper::new(selection)
    }

    pub fn undo_remove_bands(&mut self, _selection: &BandSelection) {
        todo!();
    }

    pub fn measure_length(
        &self,
        selection: &BandSelection,
    ) -> Result<MeasureLengthExecutionStepper, AutorouterError> {
        MeasureLengthExecutionStepper::new(selection)
    }

    pub(super) fn selected_ratlines(
        &self,
        selection: &PinSelection,
        principal_layer: usize,
    ) -> BTreeSet<RatlineUid> {
        self.ratsnests()
            .on_principal_layer(principal_layer)
            .graph()
            .edge_indices()
            .filter(|index| {
                let (source, target) = self
                    .ratsnests()
                    .on_principal_layer(principal_layer)
                    .graph()
                    .edge_endpoints(*index)
                    .unwrap();

                let source_ratvertex = self
                    .ratsnests()
                    .on_principal_layer(principal_layer)
                    .graph()
                    .node_weight(source)
                    .unwrap()
                    .node_index();
                let to_ratvertex = self
                    .ratsnests()
                    .on_principal_layer(principal_layer)
                    .graph()
                    .node_weight(target)
                    .unwrap()
                    .node_index();

                selection.contains_node(&self.board, source_ratvertex.into())
                    && selection.contains_node(&self.board, to_ratvertex.into())
            })
            .map(|index| RatlineUid {
                principal_layer,
                index,
            })
            .collect()
    }

    fn selected_planar_ratlines(
        &self,
        selection: &PinSelection,
        layer: usize,
    ) -> BTreeSet<RatlineUid> {
        self.selected_ratlines(selection, layer)
            .into_iter()
            .filter(|ratline| {
                let (endpoint_dot1, endpoint_dot2) = ratline.ref_(self).endpoint_dots();

                endpoint_dot1
                    .primitive_ref(self.board().layout().drawing())
                    .layer()
                    == layer
                    && endpoint_dot2
                        .primitive_ref(self.board().layout().drawing())
                        .layer()
                        == layer
            })
            .collect()
    }

    fn find_selected_ratvertex(
        &self,
        selection: &PinSelection,
        principal_layer: usize,
    ) -> Option<NodeIndex<usize>> {
        self.ratsnests()
            .on_principal_layer(principal_layer)
            .graph()
            .node_indices()
            .find(|ratvertex| {
                selection.contains_node(
                    &self.board,
                    self.ratsnests()
                        .on_principal_layer(principal_layer)
                        .graph()
                        .node_weight(*ratvertex)
                        .unwrap()
                        .node_index()
                        .into(),
                )
            })
    }
}
