// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::{cmp::Ordering, collections::BTreeMap};

use derive_getters::Getters;
use enum_dispatch::enum_dispatch;
use specctra_core::mesadata::AccessMesadata;

use crate::{
    astar::Astar,
    autorouter::{
        multilayer_autoroute::{MultilayerAutorouteConfiguration, MultilayerAutorouteOptions},
        planar_autoroute::PlanarAutorouteConfigurationStatus,
        planar_preconfigurer::PlanarAutoroutePreconfigurerInput,
        ratline::RatlineUid,
        Autorouter, AutorouterError,
    },
};

#[enum_dispatch]
pub trait MakeNextMultilayerAutorouteConfiguration {
    /// `blockers` are the ratlines whose bands blocked the failed
    /// ratline's last attempt, in original routing order — candidates
    /// for layer rotation alongside the failed ratline itself.
    fn process_planar_result(
        &mut self,
        autorouter: &Autorouter<impl AccessMesadata>,
        planar_result: Result<PlanarAutorouteConfigurationStatus, AutorouterError>,
        blockers: &[RatlineUid],
    );

    fn next_configuration(
        &mut self,
        autorouter: &Autorouter<impl AccessMesadata>,
    ) -> Option<MultilayerAutorouteConfiguration>;
}

#[enum_dispatch(MakeNextMultilayerAutorouteConfiguration)]
pub enum MultilayerAutorouteReconfigurer {
    UniformRandomLayers(IncrementFailedRatlineLayersMultilayerAutorouteReconfigurer),
}

#[derive(Clone, Debug, Getters)]
struct SearchNode {
    configuration: MultilayerAutorouteConfiguration,
}

impl Ord for SearchNode {
    fn cmp(&self, other: &Self) -> Ordering {
        self.configuration.cmp(&other.configuration)
    }
}

impl PartialOrd for SearchNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Eq for SearchNode {}

impl PartialEq for SearchNode {
    fn eq(&self, other: &Self) -> bool {
        self.configuration == other.configuration
    }
}

impl SearchNode {
    pub fn new(configuration: MultilayerAutorouteConfiguration) -> Self {
        Self { configuration }
    }

    pub fn revise_ratline(self, ratline_uid: RatlineUid, layer_count: usize) -> Self {
        let mut new_anterouter_plan = self.configuration.plan.clone();

        *new_anterouter_plan.layer_map.get_mut(&ratline_uid).unwrap() += 1;
        *new_anterouter_plan.layer_map.get_mut(&ratline_uid).unwrap() %= layer_count;

        Self {
            configuration: MultilayerAutorouteConfiguration {
                plan: new_anterouter_plan,
                planar: PlanarAutoroutePreconfigurerInput {
                    ratlines: self.configuration.planar.ratlines.clone(),
                    terminating_dot_map: BTreeMap::new(),
                },
            },
        }
    }
}

pub struct IncrementFailedRatlineLayersMultilayerAutorouteReconfigurer {
    configuration_search: Astar<SearchNode, f64>,
}

impl IncrementFailedRatlineLayersMultilayerAutorouteReconfigurer {
    pub fn new(
        _autorouter: &Autorouter<impl AccessMesadata>,
        preconfiguration: MultilayerAutorouteConfiguration,
        _options: &MultilayerAutorouteOptions,
    ) -> Self {
        Self {
            configuration_search: Astar::new(SearchNode::new(preconfiguration)),
        }
    }
}

impl MakeNextMultilayerAutorouteConfiguration
    for IncrementFailedRatlineLayersMultilayerAutorouteReconfigurer
{
    fn process_planar_result(
        &mut self,
        autorouter: &Autorouter<impl AccessMesadata>,
        planar_result: Result<PlanarAutorouteConfigurationStatus, AutorouterError>,
        blockers: &[RatlineUid],
    ) {
        let Ok(planar_status) = planar_result else {
            return;
        };

        let layer_count = autorouter.board().layout().drawing().layer_count();
        let estimate =
            (planar_status.configuration.ratlines.len() - planar_status.costs.lengths.len()) as f64;

        // The failed ratline rotates first; each blocking ratline is a
        // candidate too — moving a blocker off the contested layer can
        // resolve the conflict without moving the failed ratline.
        for ratline in planar_status
            .configuration
            .ratlines
            .get(planar_status.costs.lengths.len())
            .into_iter()
            .chain(blockers.iter().take(8))
        {
            self.configuration_search.push((
                0.1,
                estimate,
                self.configuration_search
                    .curr_node()
                    .clone()
                    .revise_ratline(*ratline, layer_count),
            ));
        }
    }

    fn next_configuration(
        &mut self,
        _autorouter: &Autorouter<impl AccessMesadata>,
    ) -> Option<MultilayerAutorouteConfiguration> {
        self.configuration_search
            .pop()
            .map(|node| node.configuration().clone())
    }
}
