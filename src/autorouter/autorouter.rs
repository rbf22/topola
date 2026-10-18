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
        multilayer_autoroute::MultilayerAutorouteOptions,
        multilayer_preconfigurer::MultilayerAutoroutePreconfigurerInput,
        multilayer_reconfigurator::MultilayerAutorouteReconfigurator,
        planar_preconfigurer::PlanarAutoroutePreconfigurerInput,
        planar_reconfigurator::PlanarAutorouteReconfigurator, ratsnests::Ratsnests,
    },
    board::{AccessMesadata, Board},
    drawing::{band::BandTermsegIndex, graph::MakePrimitiveRef},
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

#[derive(Getters)]
pub struct Autorouter<M> {
    pub(super) board: Board<M>,
    pub(super) ratsnests: Ratsnests,
}

impl<M: AccessMesadata> Autorouter<M> {
    pub fn new(board: Board<M>) -> Result<Self, InsertionError> {
        let ratsnests = Ratsnests::new(&board)?;
        Ok(Self { board, ratsnests })
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
