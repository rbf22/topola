// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::{collections::BTreeMap, ops::ControlFlow};

use derive_getters::{Dissolve, Getters};

use crate::{
    drawing::{
        band::BandTermsegIndex, dot::FixedDotIndex, graph::PrimitiveIndex, rules::AccessRules,
    },
    geometry::primitive::PrimitiveShape,
    layout::LayoutEdit,
    router::{
        navcord::{Navcord, NavcordSearchState},
        navcorder::Navcorder,
        navmesh::{Navmesh, NavmeshError},
        thetastar::{ThetastarError, ThetastarStepper},
        Router, RouterThetastarStrategy,
    },
    stepper::{EstimateProgress, LinearScale, Step},
};

#[derive(Getters, Dissolve)]
pub struct RouteStepper {
    thetastar: ThetastarStepper<Navmesh, f64>,
    navcord: Navcord,
    ghosts: Vec<PrimitiveShape>,
    obstacles: Vec<PrimitiveIndex>,
    #[getter(skip)]
    obstacle_penalties: BTreeMap<PrimitiveIndex, f64>,
    /// Per-attempt cap on `step()` calls. Without one, a single
    /// ratline whose frontier keeps expanding can consume the entire
    /// command budget before any other ratline is attempted.
    #[getter(skip)]
    step_limit: Option<usize>,
    #[getter(skip)]
    steps_taken: usize,
}

impl RouteStepper {
    pub fn new(
        router: &mut Router<impl AccessRules>,
        recorder: LayoutEdit,
        origin: FixedDotIndex,
        destination: FixedDotIndex,
        width: f64,
    ) -> Result<Self, NavmeshError> {
        let navmesh = Navmesh::new(router.layout(), origin, destination, *router.options())?;
        Ok(Self::new_from_navmesh(router, recorder, navmesh, width))
    }

    pub fn new_from_navmesh(
        router: &mut Router<impl AccessRules>,
        recorder: LayoutEdit,
        navmesh: Navmesh,
        width: f64,
    ) -> Self {
        let source = navmesh.origin();
        let source_navnode = navmesh.origin_navnode();
        let target = navmesh.destination();

        let layout = router.layout_mut();
        let mut navcord = layout.start(recorder, source, source_navnode, width);

        let mut strategy = RouterThetastarStrategy::new(layout, &mut navcord, target);
        let thetastar = ThetastarStepper::new(navmesh, source_navnode, &mut strategy);
        let ghosts = vec![];
        let obstacles = vec![];

        Self {
            thetastar,
            navcord,
            ghosts,
            obstacles,
            obstacle_penalties: BTreeMap::new(),
            step_limit: None,
            steps_taken: 0,
        }
    }

    /// Sets extra path costs charged for wrapping each primitive
    /// (negotiated congestion). Penalizing repeatedly blocking
    /// obstacles steers retries toward alternate corridors.
    pub fn set_obstacle_penalties(&mut self, penalties: BTreeMap<PrimitiveIndex, f64>) {
        self.obstacle_penalties = penalties;
    }

    /// Caps how many `step()` calls this attempt may consume before it
    /// is failed like a frontier-exhausted search. `None` (the
    /// default) leaves the attempt unbounded.
    pub fn set_step_limit(&mut self, limit: Option<usize>) {
        self.step_limit = limit;
    }

    /// Installs a search state retained from a dissolved route attempt
    /// on the same ratline, letting this attempt hit the previous
    /// probe/step memo tables instead of re-evaluating them. Only
    /// sound when the board is provably identical — the caller proves
    /// this via the whole-navmesh cache's exact snapshot match.
    pub fn restore_search_state(&mut self, state: NavcordSearchState) {
        self.navcord.restore_search_state(state);
    }
}

impl<R: AccessRules> Step<Router<'_, R>, BandTermsegIndex> for RouteStepper {
    type Error = ThetastarError;

    fn step(
        &mut self,
        router: &mut Router<R>,
    ) -> Result<ControlFlow<BandTermsegIndex>, ThetastarError> {
        let layout = router.layout_mut();

        self.steps_taken += 1;
        if self
            .step_limit
            .is_some_and(|limit| self.steps_taken > limit)
        {
            // Fail exactly like `ThetastarError::NotFound`: unwind the
            // drawn path, then propagate so the caller's failure path
            // (reconfiguration, blocker attribution) runs unchanged.
            crate::metrics::incr(&crate::metrics::METRICS.search_budget_aborts);
            for _ in 1..self.navcord.path.len() {
                self.navcord.step_back(layout);
            }
            return Err(ThetastarError::NotFound);
        }

        let target = self.thetastar.graph().destination();
        let mut strategy = RouterThetastarStrategy::new(layout, &mut self.navcord, target);
        strategy.obstacle_penalties = Some(&self.obstacle_penalties);
        let result = self.thetastar.step(&mut strategy);

        self.ghosts = strategy.probe_ghosts;
        self.obstacles = strategy.probe_obstacles;

        match result {
            Ok(ControlFlow::Continue(..)) => Ok(ControlFlow::Continue(())),
            Ok(ControlFlow::Break((_cost, _path, band))) => Ok(ControlFlow::Break(band)),
            Err(e) => {
                // NOTE(fogti): The 1 instead 0 is because the first element in the path
                // is the source navnode. See also: `NavcordStepper::new`.
                for _ in 1..self.navcord.path.len() {
                    self.navcord.step_back(layout);
                }
                Err(e)
            }
        }
    }
}

impl EstimateProgress for RouteStepper {
    type Value = f64;
    type Subscale = ();

    fn estimate_progress(&self) -> LinearScale<f64> {
        self.thetastar.estimate_progress()
    }
}

#[cfg(test)]
mod tests {
    use std::{fs::File, io::BufReader};

    use super::*;
    use crate::{
        autorouter::{ratline::RatlineUid, Autorouter},
        board::edit::BoardEdit,
        drawing::dot::FixedDotIndex,
        graph::MakeRef,
        router::{navmesh::Navmesh, Router, RouterOptions},
        specctra::{design::SpecctraDesign, mesadata::SpecctraMesadata},
    };

    fn autorouter_0603() -> Autorouter<SpecctraMesadata> {
        let design_file = File::open("tests/unilayer/0603_breakout/0603_breakout.dsn").unwrap();
        let design = SpecctraDesign::load(BufReader::new(design_file)).unwrap();
        Autorouter::new(design.make_board(&mut BoardEdit::new())).unwrap()
    }

    const ROUTER_OPTIONS: RouterOptions = RouterOptions {
        wrap_around_bands: true,
        squeeze_through_under_bends: false,
        routed_band_width: 100.0,
    };

    fn first_ratline_endpoints(
        autorouter: &Autorouter<SpecctraMesadata>,
    ) -> (FixedDotIndex, FixedDotIndex) {
        let ratline = autorouter
            .ratsnests()
            .on_principal_layer(0)
            .graph()
            .edge_indices()
            .map(|index| RatlineUid {
                principal_layer: 0,
                index,
            })
            .next()
            .unwrap();
        ratline.ref_(autorouter).endpoint_dots()
    }

    /// A per-attempt step budget must fail the attempt exactly like
    /// frontier exhaustion: `NotFound`, with the drawn prefix fully
    /// unwound so the caller's failure path sees clean geometry.
    #[test]
    fn step_limit_aborts_like_frontier_exhaustion() {
        let mut autorouter = autorouter_0603();
        let (origin, destination) = first_ratline_endpoints(&autorouter);

        let layout = autorouter.board_mut().layout_mut();
        let navmesh = Navmesh::new(layout, origin, destination, ROUTER_OPTIONS).unwrap();
        let mut router = Router::new(layout, ROUTER_OPTIONS);
        let mut route =
            RouteStepper::new_from_navmesh(&mut router, LayoutEdit::new(), navmesh, 100.0);
        route.set_step_limit(Some(1));

        // Step one runs real work; step two exceeds the budget.
        let _ = route.step(&mut router);
        let result = route.step(&mut router);

        assert!(matches!(result, Err(ThetastarError::NotFound)));
        // The drawn prefix must be unwound back to the source navnode.
        assert_eq!(route.navcord.path.len(), 1);
    }
}
