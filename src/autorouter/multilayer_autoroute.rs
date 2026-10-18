// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::{
    collections::{BTreeMap, BTreeSet},
    ops::ControlFlow,
};

use derive_getters::Getters;
use petgraph::visit::Walker;
use serde::{Deserialize, Serialize};
use specctra_core::mesadata::AccessMesadata;

use crate::{
    autorouter::{
        anterouter::{Anterouter, AnterouterOptions, AnterouterPlan},
        invoker::GetDebugOverlayData,
        planar_autoroute::PlanarAutorouteConfiguration,
        planar_preconfigurer::PlanarAutoroutePreconfigurerInput,
        planar_reconfigurator::{
            PlanarAutorouteReconfigurator, PlanarAutorouteReconfiguratorStatus,
        },
        ratline::RatlineUid,
        Autorouter, AutorouterError, PlanarAutorouteOptions,
    },
    board::edit::BoardEdit,
    drawing::{
        bend::BendIndex,
        dot::DotIndex,
        gear::{GearIndex, WalkOutwards},
        graph::PrimitiveIndex,
        primitive::GetLimbs,
        seg::SegIndex,
    },
    geometry::{edit::Edit, primitive::PrimitiveShape},
    graph::MakeRef,
    router::{navcord::Navcord, navmesh::Navmesh, thetastar::ThetastarStepper},
    stepper::{
        Abort, EstimateProgress, LinearScale, ReconfiguratorStatus, Reconfigure, Step,
        TimeoutOptions,
    },
};

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct MultilayerAutorouteConfiguration {
    pub plan: AnterouterPlan,
    pub planar: PlanarAutoroutePreconfigurerInput,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct MultilayerAutorouteOptions {
    pub anterouter: AnterouterOptions,
    pub planar: PlanarAutorouteOptions,
    pub timeout: TimeoutOptions,
    /// Seed for perturbing the initial layer assignment: when nonzero,
    /// each ratline starts on a deterministic pseudo-random layer
    /// instead of its preferred layer, falling back to the preferred
    /// layer when fanout placement fails. Parallel workers pass
    /// different seeds to explore different layer assignments.
    #[serde(default)]
    pub layer_seed: u64,
}

#[derive(Getters)]
pub struct MultilayerAutorouteExecutionStepper {
    planar: PlanarAutorouteReconfigurator,
    /// Chronological record of every fanout-related edit, merged into
    /// the command's output. Surgical reconfigurations append each
    /// changed ratline's old-edit reversal and new edit here.
    #[getter(skip)]
    anteroute_edit: BoardEdit,
    /// Per-ratline fanout edits, so re-layering can reverse only the
    /// changed ratlines' vias instead of the whole anteroute pass.
    #[getter(skip)]
    anteroute_edits: BTreeMap<RatlineUid, BoardEdit>,
    /// The layer plan the current vias were placed for.
    #[getter(skip)]
    plan: AnterouterPlan,
    #[getter(skip)]
    options: MultilayerAutorouteOptions,
}

impl MultilayerAutorouteExecutionStepper {
    pub fn new(
        autorouter: &mut Autorouter<impl AccessMesadata>,
        configuration: MultilayerAutorouteConfiguration,
        options: MultilayerAutorouteOptions,
    ) -> Result<Self, AutorouterError> {
        let mut anterouter = Anterouter::new(configuration.plan.clone());
        let mut anteroute_edit = BoardEdit::new();
        let mut anteroute_edits = BTreeMap::new();
        let mut terminating_dot_map = BTreeMap::new();

        for (ratline, layer) in configuration.plan.layer_map.clone() {
            let mut ratline_edit = BoardEdit::new();
            if anterouter
                .anteroute_ratline(
                    autorouter,
                    &mut ratline_edit,
                    ratline,
                    layer,
                    &options.anterouter,
                    &mut terminating_dot_map,
                )
                .is_err()
            {
                // Seeded layer assignments may ask for a layer the
                // pads cannot reach. Reverse any partial placement and
                // fall back to the preferred layer.
                autorouter.board.apply_edit(&ratline_edit.reverse());
                anteroute_edit.merge(ratline_edit.clone());
                anteroute_edit.merge(ratline_edit.reverse());
                ratline_edit = BoardEdit::new();
                anterouter
                    .anteroute_ratline(
                        autorouter,
                        &mut ratline_edit,
                        ratline,
                        ratline.ref_(autorouter).preferred_layer(),
                        &options.anterouter,
                        &mut terminating_dot_map,
                    )
                    .expect("fanout via placement failed");
            }
            anteroute_edit.merge(ratline_edit.clone());
            anteroute_edits.insert(ratline, ratline_edit);
        }

        Ok(Self {
            planar: PlanarAutorouteReconfigurator::new(
                autorouter,
                PlanarAutoroutePreconfigurerInput {
                    terminating_dot_map,
                    ratlines: configuration.planar.ratlines,
                },
                options.planar,
            )?,
            anteroute_edit,
            anteroute_edits,
            plan: configuration.plan,
            options: options.clone(),
        })
    }

    /// Finishes routing early, keeping already routed bands. See
    /// [`PlanarAutorouteReconfigurator::finish_early`].
    pub fn finish_early(&mut self, autorouter: &mut Autorouter<impl AccessMesadata>) {
        self.planar.finish_early(autorouter);
    }
}

impl<M: AccessMesadata> Step<Autorouter<M>, Option<BoardEdit>, PlanarAutorouteReconfiguratorStatus>
    for MultilayerAutorouteExecutionStepper
{
    type Error = AutorouterError;

    fn step(
        &mut self,
        autorouter: &mut Autorouter<M>,
    ) -> Result<ControlFlow<Option<BoardEdit>, PlanarAutorouteReconfiguratorStatus>, AutorouterError>
    {
        match self.planar.step(autorouter) {
            Ok(ControlFlow::Break(Some(edit))) => {
                self.anteroute_edit.merge(edit);
                // FIXME: Unnecessary large clone.
                Ok(ControlFlow::Break(Some(self.anteroute_edit.clone())))
            }
            Ok(ControlFlow::Continue(ReconfiguratorStatus::Reconfigured(result))) => Ok(
                ControlFlow::Continue(ReconfiguratorStatus::Reconfigured(result)),
            ),
            x => x,
        }
    }
}

impl<M: AccessMesadata> Abort<Autorouter<M>> for MultilayerAutorouteExecutionStepper {
    fn abort(&mut self, autorouter: &mut Autorouter<M>) {
        self.planar.abort(autorouter)
    }
}

impl<M: AccessMesadata> Reconfigure<Autorouter<M>> for MultilayerAutorouteExecutionStepper {
    type Configuration = MultilayerAutorouteConfiguration;
    type Output = Result<(), AutorouterError>;

    fn reconfigure(
        &mut self,
        autorouter: &mut Autorouter<M>,
        new_configuration: MultilayerAutorouteConfiguration,
    ) -> Result<(), AutorouterError> {
        // Surgical re-layering: only the ratlines whose layer
        // assignment changed need their bands ripped and their fanout
        // vias swapped; every other routed band survives.
        let changed: Vec<RatlineUid> = new_configuration
            .plan
            .layer_map
            .iter()
            .filter(|(ratline, layer)| self.plan.layer_map.get(*ratline) != Some(*layer))
            .map(|(ratline, _)| *ratline)
            .collect();

        if changed.is_empty() {
            self.plan = new_configuration.plan;
            return Ok(());
        }
        crate::metrics::incr(&crate::metrics::METRICS.multilayer_surgical_reconfigures);

        // Ratlines already attempted hold bands that must be ripped and
        // requeued; the rest pick up their new layer when reached.
        let curr = self.planar.curr_ratline_index();
        let attempted: BTreeSet<RatlineUid> = self.planar.configuration().ratlines
            [..curr.min(self.planar.configuration().ratlines.len())]
            .iter()
            .copied()
            .collect();
        let routed_changed: Vec<RatlineUid> = changed
            .iter()
            .copied()
            .filter(|ratline| attempted.contains(ratline))
            .collect();

        // Phase 1: rip the changed ratlines' bands — recorded inside
        // the planar edit stream, resolved under the live
        // terminating-dot map — freeing the fanout via dots. Always
        // run, even with an empty rip-up list, so the in-flight route
        // is dissolved and cannot materialize bends around a via that
        // phase 2 is about to remove.
        self.planar
            .begin_surgical_reconfiguration(autorouter, &routed_changed)?;

        // Phase 2: swap each changed ratline's fanout vias.
        let mut anterouter = Anterouter::new(new_configuration.plan.clone());
        let mut new_entries = BTreeMap::new();
        let mut layer_fixes = vec![];
        // Ratlines whose old fanout vias are removed below. Their
        // terminating-dot map entries must be purged: a retained
        // `FixedDotIndex` points at a freed node slot that a later
        // `add_via` may recycle as a compound — dereferencing it
        // panics in `Geometry::primitive_weight`.
        let mut relayered = vec![];
        for ratline in &changed {
            // The gears the old edit created — and that its reversal
            // therefore removes — must be free of foreign band
            // attachments before removal: a band may bend around a via
            // dot (a wrap) or terminate at it through a seg joint (a
            // limb — lone termsegs wrap nothing). Limbs the same edit
            // itself created, like its fanout seg, don't count.
            // Enumerating the edit's own creations also covers vias the
            // terminating-dot map does not reference.
            let wrapped = self.anteroute_edits.get(ratline).is_some_and(|old_edit| {
                let drawing = autorouter.board.layout().drawing();
                let created_segs: BTreeSet<SegIndex> =
                    old_edit.layout_edit.created_segs().collect();
                let created_bends: BTreeSet<BendIndex> =
                    old_edit.layout_edit.created_bends().collect();

                old_edit.layout_edit.created_dots().any(|dot| {
                    let DotIndex::Fixed(dot) = dot else {
                        return false;
                    };
                    let dot_ref = drawing.primitive_ref(dot);
                    let mut outwards = dot_ref.outwards();
                    outwards.walk_next(drawing).is_some()
                        || dot_ref.segs().iter().any(|seg| !created_segs.contains(seg))
                        || dot_ref
                            .bends()
                            .iter()
                            .any(|bend| !created_bends.contains(bend))
                }) || old_edit.layout_edit.created_bends().any(|bend| {
                    let mut outwards = GearIndex::from(bend).ref_(drawing).outwards();
                    outwards.walk_next(drawing).is_some()
                })
            });

            if wrapped {
                // Keep the old fanout and the old layer: its gears are
                // load-bearing for another band.
                layer_fixes.push((*ratline, self.plan.layer_map[ratline]));
                continue;
            }

            relayered.push(*ratline);

            if let Some(old_edit) = self.anteroute_edits.remove(ratline) {
                autorouter.board.apply_edit(&old_edit.reverse());
                self.anteroute_edit.merge(old_edit.clone());
                self.anteroute_edit.merge(old_edit.reverse());
            }

            let mut ratline_edit = BoardEdit::new();
            let entries_before = new_entries.clone();
            let placed = anterouter.anteroute_ratline(
                autorouter,
                &mut ratline_edit,
                *ratline,
                new_configuration.plan.layer_map[ratline],
                &self.options.anterouter,
                &mut new_entries,
            );

            if placed.is_err() {
                // Re-fanout to the new layer found no room — the board
                // is now crowded where the initial pass worked. A
                // partially placed fanout must be reversed before
                // restoring: an orphan via would remain in this
                // ratline's recorded edit unmapped, and a later swap
                // could remove it while a foreign band wraps it.
                autorouter.board.apply_edit(&ratline_edit.reverse());
                self.anteroute_edit.merge(ratline_edit.clone());
                self.anteroute_edit.merge(ratline_edit.reverse());
                ratline_edit = BoardEdit::new();
                new_entries = entries_before;

                // Restore the old layer's fanout so the ratline keeps
                // terminating dots it can route to. Reversed likewise
                // on partial failure.
                let old_layer = self.plan.layer_map[ratline];
                let entries_before_restore = new_entries.clone();
                if anterouter
                    .anteroute_ratline(
                        autorouter,
                        &mut ratline_edit,
                        *ratline,
                        old_layer,
                        &self.options.anterouter,
                        &mut new_entries,
                    )
                    .is_err()
                {
                    autorouter.board.apply_edit(&ratline_edit.reverse());
                    self.anteroute_edit.merge(ratline_edit.clone());
                    self.anteroute_edit.merge(ratline_edit.reverse());
                    ratline_edit = BoardEdit::new();
                    new_entries = entries_before_restore;
                }
                layer_fixes.push((*ratline, old_layer));
            }

            self.anteroute_edit.merge(ratline_edit.clone());
            self.anteroute_edits.insert(*ratline, ratline_edit);
        }

        // Phase 3: drop the dead dots' remappings, then point live and
        // future configurations at the new terminating dots, then
        // requeue the ripped ratlines right after the currently
        // attempted one.
        self.planar.remove_terminating_dots(&relayered);
        self.planar.remap_terminating_dots(&new_entries);

        {
            let ripped: BTreeSet<RatlineUid> = routed_changed.iter().copied().collect();
            let mut order: Vec<RatlineUid> = self
                .planar
                .configuration()
                .ratlines
                .iter()
                .copied()
                .filter(|ratline| !ripped.contains(ratline))
                .collect();
            let insert_at = self
                .planar
                .configuration()
                .ratlines
                .get(curr)
                .and_then(|attempted| {
                    order
                        .iter()
                        .position(|ratline| *ratline == *attempted)
                        .map(|index| index + 1)
                })
                .unwrap_or(order.len());
            order.splice(insert_at..insert_at, routed_changed.iter().copied());

            self.planar.commit_surgical_reconfiguration(
                autorouter,
                PlanarAutorouteConfiguration {
                    ratlines: order,
                    rip_up: routed_changed,
                    ..self.planar.configuration().clone()
                },
            )?;
        }

        self.plan = new_configuration.plan;
        // Ratlines whose re-fanout failed keep their old layer, so the
        // recorded plan matches the vias actually on the board.
        for (ratline, layer) in layer_fixes {
            self.plan.layer_map.insert(ratline, layer);
        }
        Ok(())
    }
}

impl EstimateProgress for MultilayerAutorouteExecutionStepper {
    type Value = usize;
    type Subscale = LinearScale<f64>;

    fn estimate_progress(&self) -> LinearScale<usize, LinearScale<f64>> {
        self.planar.estimate_progress()
    }
}

impl GetDebugOverlayData for MultilayerAutorouteExecutionStepper {
    fn maybe_thetastar(&self) -> Option<&ThetastarStepper<Navmesh, f64>> {
        self.planar.maybe_thetastar()
    }

    fn maybe_navcord(&self) -> Option<&Navcord> {
        self.planar.maybe_navcord()
    }

    fn ghosts(&self) -> &[PrimitiveShape] {
        self.planar.ghosts()
    }

    fn obstacles(&self) -> &[PrimitiveIndex] {
        self.planar.obstacles()
    }
}

#[cfg(test)]
mod tests {
    use std::{collections::BTreeSet, fs::File, io::BufReader};

    use specctra_core::mesadata::SpecctraMesadata;

    use super::*;
    use crate::{
        autorouter::{
            multilayer_preconfigurer::{
                MultilayerAutoroutePreconfigurerInput, MultilayerPreconfigurer,
            },
            PresortBy, ReconfigureBy,
        },
        drawing::graph::MakePrimitiveRef,
        geometry::GetLayer,
        router::RouterOptions,
        specctra::design::SpecctraDesign,
    };

    fn astable() -> Autorouter<SpecctraMesadata> {
        let design_file =
            File::open("tests/multilayer/astable_multivibrator/astable_multivibrator.dsn").unwrap();
        let design = SpecctraDesign::load(BufReader::new(design_file)).unwrap();
        Autorouter::new(design.make_board(&mut BoardEdit::new())).unwrap()
    }

    /// Moving a ratline to another layer removes its old fanout vias
    /// and places new ones. Every `terminating_dot_map` value must
    /// afterwards resolve to a live dot on the layer the stepper's
    /// recorded plan claims — before the purge fix a dead
    /// `FixedDotIndex` survived in the map until a later `add_via`
    /// recycled its node slot into a compound and `primitive_weight`
    /// panicked.
    #[test]
    fn reconfigure_refreshes_terminating_dot_map() {
        let mut autorouter = astable();
        let layer_count = autorouter.board().layout().drawing().layer_count();

        let ratlines: BTreeSet<RatlineUid> = (0..layer_count)
            .flat_map(|layer| {
                autorouter
                    .ratsnests()
                    .on_principal_layer(layer)
                    .graph()
                    .edge_indices()
                    .take(2)
                    .map(move |index| RatlineUid {
                        principal_layer: layer,
                        index,
                    })
            })
            .take(3)
            .collect();
        let rotated = *ratlines.iter().next().unwrap();

        let options = MultilayerAutorouteOptions {
            anterouter: AnterouterOptions {
                fanout_clearance: 200.0,
            },
            planar: PlanarAutorouteOptions {
                principal_layer: 0,
                presort_by: PresortBy::RatlineIntersectionCountAndLength,
                permutate: true,
                reconfigure_by: ReconfigureBy::default(),
                congestion_penalty: 500.0,
                router: RouterOptions {
                    wrap_around_bands: true,
                    squeeze_through_under_bends: false,
                    routed_band_width: 100.0,
                },
                timeout: TimeoutOptions {
                    initial: 1.0,
                    progress_bonus: 0.005,
                },
                max_reconfigurations: None,
                improve_pad_entry: false,
                pad_entry_min_angle: std::f64::consts::FRAC_PI_4,
                escape_stubs: false,
                surgical_ripup: false,
                shuffle_seed: 0,
            },
            timeout: TimeoutOptions {
                initial: 10.0,
                progress_bonus: 0.0,
            },
            layer_seed: 0,
        };

        let plan = MultilayerPreconfigurer::new(
            &autorouter,
            MultilayerAutoroutePreconfigurerInput {
                ratlines: ratlines.clone(),
            },
        )
        .plan()
        .clone();

        let make_configuration = |plan: &AnterouterPlan| MultilayerAutorouteConfiguration {
            plan: plan.clone(),
            planar: PlanarAutoroutePreconfigurerInput {
                ratlines: ratlines.clone(),
                terminating_dot_map: BTreeMap::new(),
            },
        };

        let mut stepper = MultilayerAutorouteExecutionStepper::new(
            &mut autorouter,
            make_configuration(&plan),
            options,
        )
        .unwrap();

        let assert_map_consistent =
            |stepper: &MultilayerAutorouteExecutionStepper,
             autorouter: &Autorouter<SpecctraMesadata>| {
                let drawing = autorouter.board().layout().drawing();
                for ((ratline, _), dot) in &stepper.planar().configuration().terminating_dot_map {
                    // A stale index panics here once its freed slot has
                    // been recycled into another node variant.
                    let dot_ref = dot.primitive_ref(drawing);
                    assert_eq!(dot_ref.layer(), stepper.plan.layer_map[ratline]);
                }
            };
        assert_map_consistent(&stepper, &autorouter);

        // Cycle the ratline across layers twice: each reconfigure
        // removes the previous fanout via and installs fresh
        // terminating dots, so both the purge and the remap run.
        let mut new_plan = plan;
        for _ in 0..2 {
            *new_plan.layer_map.get_mut(&rotated).unwrap() =
                (new_plan.layer_map[&rotated] + 1) % layer_count;
            stepper
                .reconfigure(&mut autorouter, make_configuration(&new_plan))
                .unwrap();
            assert_map_consistent(&stepper, &autorouter);
            // The swap must have actually run: a relayered ratline
            // without fanout dots would mean the test exercised
            // nothing.
            assert!(
                stepper
                    .planar()
                    .configuration()
                    .terminating_dot_map
                    .keys()
                    .any(|(ratline, _)| *ratline == rotated),
                "rotated ratline has no terminating-dot map entries"
            );
        }
    }
}
