// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::collections::HashMap;

use contracts_try::debug_ensures;
use geo::{
    algorithm::line_measures::{Distance, Euclidean},
    Line, Point,
};
use petgraph::{data::DataMap, visit::Walker};

use crate::{
    drawing::{
        band::BandTermsegIndex,
        bend::BendIndex,
        dot::{DotIndex, FixedDotIndex},
        gear::{GearIndex, WalkOutwards},
        graph::{GetMaybeNet, MakePrimitiveRef, PrimitiveIndex},
        head::{BareHead, CaneHead, GetFace, Head},
        primitive::{GetLimbs, GetWeight, MakePrimitiveShape},
        rules::{AccessRules, Conditions},
        seg::SegIndex,
        AlreadyConnected, Collision, Drawing, DrawingException,
    },
    geometry::{
        primitive::{AccessPrimitiveShape, BendShape, PrimitiveShape, SegShape},
        shape::MeasureLength,
    },
    graph::MakeRef,
    layout::{Layout, LayoutEdit},
    math::{self, Circle, RotationSense},
};

use super::{
    draw::{Draw, DrawException},
    navcorder::NavcorderException,
    navmesh::{BinavnodeNodeIndex, Navmesh, NavnodeIndex},
};

/// The `Navcord` is a data structure that holds the movable non-borrowing data
/// of the currently running routing process.
///
/// Note that this data structure is not a stepper, since steppers always
/// progress linearly, whereas `Navcord` branches out to different states
/// depending on the value of the `to` argument of the `.step_to(...)` method.
///
/// The name "navcord" is a shortening of "navigation cord", by analogy to
/// "navmesh" being a shortening of "navigation mesh".
/// The verdict of a speculative `step_to` probe, for memoization. The
/// probe outcome is a pure function of the board state, the navcord path,
/// the initial parent navnode and the probed navnode, so it can be
/// cached for the lifetime of the navcord (which never outlives the
/// navmesh its navnode indices refer to).
#[derive(Clone, Debug)]
pub enum ProbeVerdict {
    /// The probe was drawn successfully, with the recorded track length
    /// and the virtual step to replay.
    Drawn { length: f64, replay: GhostReplay },
    /// The probe failed and the failure permits continued backtracking.
    Backtrackable,
    /// The probe failed terminally.
    Terminal,
}

/// A memoized probe verdict together with the ghosts/obstacles reported
/// at the time, so that a replayed verdict produces the same observable
/// state as the original probe.
#[derive(Clone, Debug)]
pub struct ProbeMemoEntry {
    pub verdict: ProbeVerdict,
    pub ghosts: Vec<PrimitiveShape>,
    pub obstacles: Vec<PrimitiveIndex>,
}

/// The outcome of `.ghost_step_to(..)`, the non-mutating evaluation of a
/// step.
#[derive(Debug)]
pub enum GhostVerdict {
    /// The step was evaluated without drawing and is reported drawable,
    /// with the recorded track length and the virtual state it pushes.
    Drawn { replay: GhostReplay, length: f64 },
    /// The step was evaluated without drawing and reported to fail
    /// identically to the real draw.
    Failed(NavcorderException),
    /// The step cannot be evaluated without drawing; the real
    /// `.step_to(..)` must be used.
    Unsupported,
}

/// A cane step kept as pure geometry, not drawn into the board.
///
/// Canes are evaluated against the board exactly like real draws but are
/// retained as data: the routed band exists only virtually until the
/// search commits to it and `.materialize(..)` draws the whole path for
/// real. This eliminates almost all geometry insertion, removal and
/// movement costs of path exploration.
#[derive(Clone, Debug)]
pub struct VirtualCane {
    /// The gear the cane was wrapped around.
    gear: GearIndex,
    /// The rotation sense the bend was drawn with.
    sense: RotationSense,
    /// The bitangent guide used: its start is the position the previous
    /// head's face was moved to and its end the position of the cane's
    /// two dots, one of which is the new head's face.
    tangent: Line,
    /// The bend's inner circle, that is, the wraparound's outer circle
    /// inflated by the clearance between them.
    inner_circle: Circle,
}

impl VirtualCane {
    /// The cane's seg shape, spanning between the tangent's endpoints.
    fn seg_shape(&self, width: f64) -> PrimitiveShape {
        PrimitiveShape::Seg(SegShape {
            from: self.tangent.start_point(),
            to: self.tangent.end_point(),
            width,
        })
    }

    /// The cane's bend shape when the head's face sits at `face_pos`.
    /// The face sits at the tangent's end point while this cane is the
    /// head and is moved to the next cane's tangent start point when the
    /// head extends past it.
    fn bend_shape(&self, width: f64, face_pos: Point) -> PrimitiveShape {
        let tangent_end = self.tangent.end_point();
        let (from, to) = match self.sense {
            RotationSense::Counterclockwise => (tangent_end, face_pos),
            RotationSense::Clockwise => (face_pos, tangent_end),
        };
        PrimitiveShape::Bend(BendShape {
            from,
            to,
            inner_circle: self.inner_circle,
            width,
        })
    }
}

/// The virtual state a successfully evaluated step pushes onto the
/// navcord. Kept separately so that memoized verdicts and committed
/// steps can replay it without recomputing the geometry.
#[derive(Clone, Debug)]
pub enum GhostReplay {
    /// A wrap step pushes a virtual cane.
    Wrap(VirtualCane),
    /// A finish step pushes a virtual finish marker.
    Finish,
}

/// One cane step on the navcord's path: either kept purely virtually or
/// materialized into the board.
#[derive(Debug)]
pub(crate) enum PathCane {
    /// The cane exists only as virtual geometry.
    Virtual(VirtualCane),
    /// The cane was drawn into the board by `.materialize(..)`.
    Real(CaneHead),
}

#[derive(Debug)]
pub struct Navcord {
    /// The layout edit which we are currently recording.
    pub recorder: LayoutEdit,
    /// The currently attempted path.
    pub path: Vec<NavnodeIndex>,
    /// The head of the currently routed band as materialized in the
    /// board. While the path's trailing canes are virtual, this stays
    /// the head the materialized prefix ended with.
    pub head: Head,
    /// The cane steps behind `.path`, one per navnode on the path after
    /// the source, up to a possible trailing finish step.
    pub(crate) canes: Vec<PathCane>,
    /// Whether a virtually evaluated finish is on the path. Only
    /// produced by finish probes; the real commit draws through
    /// `.finish(..)` directly.
    pub(crate) virtual_finish: bool,
    /// If the band is finished, stores the termseg that was used to finish it.
    pub maybe_final_termseg: Option<BandTermsegIndex>,
    /// The width of the currently routed band.
    pub width: f64,
    /// The net and layer of the currently routed band, constant over
    /// all of its canes.
    band_net: Option<usize>,
    band_layer: usize,
    /// Memoized probe verdicts keyed by the navcord path state, initial
    /// parent navnode, and probed navnode. Nested so that lookups can
    /// borrow the live path instead of cloning it per probe.
    pub probe_memo: HashMap<
        Box<[NavnodeIndex]>,
        HashMap<(Option<NavnodeIndex>, NavnodeIndex), ProbeMemoEntry>,
    >,
    /// Memoized `.step_to(..)` verdicts keyed by the navcord state —
    /// the path plus the number of leading materialized canes — and
    /// the target navnode. `.rework_path(..)` redraws paths over and
    /// over, so this saves re-evaluating their geometry each time.
    /// Nested so that lookups can borrow the live path instead of
    /// cloning it per step.
    step_memo: HashMap<
        Box<[NavnodeIndex]>,
        HashMap<(usize, NavnodeIndex), Result<(GhostReplay, f64), NavcorderException>>,
    >,
}

impl Navcord {
    /// Create a new navcord.
    pub fn new(
        recorder: LayoutEdit,
        source: FixedDotIndex,
        source_navnode: NavnodeIndex,
        width: f64,
        maybe_net: Option<usize>,
        layer: usize,
    ) -> Navcord {
        Self {
            recorder,
            path: vec![source_navnode],
            head: BareHead { face: source }.into(),
            canes: vec![],
            virtual_finish: false,
            maybe_final_termseg: None,
            width,
            band_net: maybe_net,
            band_layer: layer,
            probe_memo: HashMap::new(),
            step_memo: HashMap::new(),
        }
    }

    /// The virtual canes currently on the path.
    fn virtual_canes(&self) -> impl Iterator<Item = &VirtualCane> + '_ {
        self.canes.iter().filter_map(|cane| match cane {
            PathCane::Virtual(cane) => Some(cane),
            PathCane::Real(..) => None,
        })
    }

    /// The routing band's conditions: its net and layer, constant over
    /// the whole band.
    fn face_conditions(&self) -> Option<Conditions<'static>> {
        self.band_net.map(|net| Conditions {
            net,
            maybe_region: Some("A".into()),
            maybe_layer: Some(self.band_layer),
        })
    }

    /// The circle and rotation sense the head's guides grow from,
    /// replicating `.head_circle(..)` and `.head_sense(..)` for both
    /// real and virtual heads.
    ///
    /// For a cane head the from-circle is a circle around the gear the
    /// last cane is wrapped on; for a bare head a zero-radius circle on
    /// the face. A bend drawn with `sense` yields the opposite head
    /// sense, as in `.head_sense(..)`.
    fn guide_from<CW: Clone, Cel: Copy, R: AccessRules>(
        &self,
        drawing: &Drawing<CW, Cel, R>,
    ) -> (Circle, Option<RotationSense>) {
        let face_conditions = self.face_conditions();
        if let Some(PathCane::Virtual(cane)) = self.canes.last() {
            (
                match cane.gear {
                    GearIndex::FixedDot(dot) => {
                        drawing.dot_circle(dot.into(), self.width, face_conditions.as_ref())
                    }
                    GearIndex::FixedBend(bend) => drawing.bend_circle(
                        bend.into(),
                        self.width,
                        face_conditions.as_ref(),
                    ),
                    GearIndex::LooseBend(bend) => drawing.bend_circle(
                        bend.into(),
                        self.width,
                        face_conditions.as_ref(),
                    ),
                },
                Some(match cane.sense {
                    RotationSense::Counterclockwise => RotationSense::Clockwise,
                    RotationSense::Clockwise => RotationSense::Counterclockwise,
                }),
            )
        } else {
            (
                drawing.head_circle(&self.head, self.width),
                drawing.head_sense(&self.head),
            )
        }
    }

    /// Checks the head's reshaped limbs for infringements exactly as
    /// `.extend_head(..)` does when it moves the face to `face_pos`,
    /// and returns the old bend's arc length after the move, which is
    /// what `.step_to_intern(..)` measures. For a bare head there is
    /// nothing to do.
    fn ghost_extend_head<CW: Clone, Cel: Copy, R: AccessRules>(
        &self,
        drawing: &Drawing<CW, Cel, R>,
        face_pos: Point,
    ) -> Result<f64, DrawingException> {
        match self.canes.last() {
            Some(PathCane::Virtual(cane)) => {
                // A cane head's only limb is the last cane's bend.
                let shape = cane.bend_shape(self.width, face_pos);
                let prev_bend_length = shape.length();
                if let Some(infringement) = drawing.find_infringement_of_shape(
                    shape,
                    self.band_layer,
                    self.band_net,
                    &|_, _| true,
                ) {
                    return Err(infringement.into());
                }
                Ok(prev_bend_length)
            }
            Some(PathCane::Real(..)) | None => {
                let Head::Cane(head) = self.head else {
                    return Ok(0.0);
                };
                let mut prev_bend_length = 0.0;
                for limb in head.face.primitive_ref(drawing).limbs() {
                    let mut shape = limb.primitive_ref(drawing).shape();
                    match &mut shape {
                        PrimitiveShape::Seg(seg_shape) => {
                            let seg: SegIndex = limb.try_into().unwrap();
                            if drawing.geometry().seg_joints(seg).0
                                == DotIndex::Loose(head.face)
                            {
                                seg_shape.from = face_pos;
                            } else {
                                seg_shape.to = face_pos;
                            }
                        }
                        PrimitiveShape::Bend(bend_shape) => {
                            let bend: BendIndex = limb.try_into().unwrap();
                            if drawing.geometry().bend_joints(bend).0
                                == DotIndex::Loose(head.face)
                            {
                                bend_shape.from = face_pos;
                            } else {
                                bend_shape.to = face_pos;
                            }
                        }
                        PrimitiveShape::Dot(..) => unreachable!(),
                    }

                    if limb == PrimitiveIndex::LooseBend(head.cane.bend) {
                        prev_bend_length = shape.length();
                    }

                    if let Some(infringement) = drawing.find_infringement_of_shape(
                        shape,
                        self.band_layer,
                        self.band_net,
                        &|_, _| true,
                    ) {
                        return Err(infringement.into());
                    }
                }
                Ok(prev_bend_length)
            }
        }
    }

    /// The virtual step the last `.step_to(..)` pushed onto the path,
    /// for recording in `.probe_memo`.
    pub fn last_replay(&self) -> Option<GhostReplay> {
        if self.virtual_finish {
            Some(GhostReplay::Finish)
        } else {
            self.canes.last().map(|cane| match cane {
                PathCane::Virtual(cane) => GhostReplay::Wrap(cane.clone()),
                PathCane::Real(..) => unreachable!(),
            })
        }
    }

    /// From the current head `head` wrap a new head around the navnode
    /// `around`, checking for infringements.
    ///
    /// On success also reports whether the cane had to be squeezed through
    /// under bends. A squeezed draw cannot be faithfully reproduced by a
    /// trusted (check-free) replay, because squeezing additionally updates
    /// the bows of neighboring bends.
    fn wrap(
        &mut self,
        layout: &mut Layout<impl AccessRules>,
        navmesh: &Navmesh,
        head: Head,
        around: NavnodeIndex,
    ) -> Result<(CaneHead, bool), NavcorderException> {
        let around_node_weight = navmesh.node_weight(around).unwrap();
        let sense = around_node_weight
            .maybe_sense
            .ok_or(NavcorderException::CannotWrap)?;

        match around_node_weight.binavnode {
            BinavnodeNodeIndex::FixedDot(dot) => {
                layout.cane_around_dot(&mut self.recorder, head, dot, sense, self.width)
            }
            BinavnodeNodeIndex::FixedBend(fixed_bend) => layout.cane_around_bend(
                &mut self.recorder,
                head,
                fixed_bend.into(),
                sense,
                self.width,
            ),
            BinavnodeNodeIndex::LooseBend(loose_bend) => layout.cane_around_bend(
                &mut self.recorder,
                head,
                loose_bend.into(),
                sense,
                self.width,
            ),
        }
        .map_err(NavcorderException::CannotDraw)
    }

    /// Same as `.wrap(..)`, but performs no checks. Must only be used to
    /// replay draws that were already verified in the same state.
    #[allow(dead_code)]
    fn wrap_trusted(
        &mut self,
        layout: &mut Layout<impl AccessRules>,
        navmesh: &Navmesh,
        head: Head,
        around: NavnodeIndex,
    ) -> Result<CaneHead, NavcorderException> {
        let around_node_weight = navmesh.node_weight(around).unwrap();
        let sense = around_node_weight
            .maybe_sense
            .ok_or(NavcorderException::CannotWrap)?;

        match around_node_weight.binavnode {
            BinavnodeNodeIndex::FixedDot(dot) => layout.cane_around_dot_trusted(
                &mut self.recorder,
                head,
                dot,
                sense,
                self.width,
            ),
            BinavnodeNodeIndex::FixedBend(fixed_bend) => layout.cane_around_bend_trusted(
                &mut self.recorder,
                head,
                fixed_bend.into(),
                sense,
                self.width,
            ),
            BinavnodeNodeIndex::LooseBend(loose_bend) => layout.cane_around_bend_trusted(
                &mut self.recorder,
                head,
                loose_bend.into(),
                sense,
                self.width,
            ),
        }
        .map_err(NavcorderException::CannotDraw)
    }

    /// Advance the navcord and the currently routed band by one step to the
    /// navnode `to`.
    ///
    /// Returns the length of the created track.
    #[debug_ensures(ret.is_ok() -> self.path.len() == old(self.path.len() + 1))]
    #[debug_ensures(ret.is_err() -> self.path.len() == old(self.path.len()))]
    pub fn step_to<R: AccessRules>(
        &mut self,
        layout: &mut Layout<R>,
        navmesh: &Navmesh,
        to: NavnodeIndex,
    ) -> Result<f64, NavcorderException> {
        let _timer = crate::metrics::Timer::start(&crate::metrics::METRICS.step_to_nanos);
        let result = self.step_to_intern(layout, navmesh, to);
        match result {
            Ok(_) => crate::metrics::incr(&crate::metrics::METRICS.step_to_ok),
            Err(_) => crate::metrics::incr(&crate::metrics::METRICS.step_to_err),
        }
        result
    }

    fn step_to_intern<R: AccessRules>(
        &mut self,
        layout: &mut Layout<R>,
        navmesh: &Navmesh,
        to: NavnodeIndex,
    ) -> Result<f64, NavcorderException> {
        // The verdict is a pure function of the navcord state — the
        // path and, since materialized canes are real primitives on
        // the board, how many leading canes are materialized — and the
        // target navnode, so it can be memoized.
        let materialized = self
            .canes
            .iter()
            .take_while(|cane| matches!(cane, PathCane::Real(..)))
            .count();
        if let Some(entry) = self
            .step_memo
            .get(self.path.as_slice())
            .and_then(|by_path| by_path.get(&(materialized, to)))
        {
            crate::metrics::incr(&crate::metrics::METRICS.step_memo_hits);
            return match entry.clone() {
                Ok((replay, length)) => {
                    match replay {
                        GhostReplay::Wrap(cane) => {
                            self.canes.push(PathCane::Virtual(cane))
                        }
                        GhostReplay::Finish => self.virtual_finish = true,
                    }
                    self.path.push(to);
                    Ok(length)
                }
                Err(err) => Err(err),
            };
        }

        let verdict = match self.ghost_step_to(layout, navmesh, to) {
            GhostVerdict::Drawn { replay, length } => Ok((replay, length)),
            GhostVerdict::Failed(err) => Err(err),
            GhostVerdict::Unsupported => unreachable!("all steps are ghost-evaluable"),
        };

        // Cap the memo so that on pathological boards it does not grow
        // without bound; beyond the cap verdicts are just re-evaluated.
        if self.step_memo.len() < 2_000_000 {
            self.step_memo
                .entry(self.path.clone().into_boxed_slice())
                .or_default()
                .insert((materialized, to), verdict.clone());
        }

        match verdict {
            Ok((replay, length)) => {
                match replay {
                    GhostReplay::Wrap(cane) => self.canes.push(PathCane::Virtual(cane)),
                    GhostReplay::Finish => self.virtual_finish = true,
                }

                // Now that the new part of the trace has been created, push
                // the navnode `to` onto the currently attempted path to start
                // from it on the next `.step_to(...)` call or retreat from it
                // later using `.step_back(...)`.
                self.path.push(to);
                Ok(length)
            }
            Err(err) => Err(err),
        }
    }

    /// Draws every virtual cane on the path into the board, in path
    /// order, replacing them with their real counterparts.
    ///
    /// This is invoked when the destination navnode is visited and the
    /// path is about to be finished for real. Since every virtual cane
    /// was verified drawable when it was created, the replays are
    /// check-free; a `NoBitangents` error is still propagated like any
    /// other draw failure and leaves the navcord in a consistent
    /// partially materialized state.
    pub fn materialize<R: AccessRules>(
        &mut self,
        layout: &mut Layout<R>,
        navmesh: &Navmesh,
    ) -> Result<(), NavcorderException> {
        for i in 0..self.canes.len() {
            if matches!(self.canes[i], PathCane::Real(..)) {
                continue;
            }
            // Re-checking rather than `.wrap_trusted(..)` keeps the
            // commit honest: a mismatch between the ghost evaluation
            // and the real draw fails the route instead of drawing
            // invalid geometry.
            let (head, _squeezed) =
                self.wrap(layout, navmesh, self.head, self.path[i + 1])?;
            self.head = head.into();
            self.canes[i] = PathCane::Real(head);
            crate::metrics::incr(&crate::metrics::METRICS.step_to_trusted);
        }
        Ok(())
    }

    /// Evaluates `.step_to(..)` without mutating the board, replicating
    /// exactly the checks the real draw performs: the moved head's limb
    /// infringements, the new seg's infringement and collision checks,
    /// and the new bend's connectivity and infringement checks, or the
    /// finish's equivalent checks for a destination step.
    ///
    /// A verdict other than `Unsupported` is identical to the verdict the
    /// real `.step_to(..)` would produce in this state: the
    /// squeeze-through-under-bends fallback in `.insert_cane(..)` performs
    /// the same checks under a strictly stronger filter, so it can never
    /// rescue a failed `.add_cane(..)`.
    pub fn ghost_step_to<R: AccessRules>(
        &self,
        layout: &Layout<R>,
        navmesh: &Navmesh,
        to: NavnodeIndex,
    ) -> GhostVerdict {
        let drawing = layout.drawing();
        let around_node_weight = navmesh.node_weight(to).unwrap();

        if to == navmesh.destination_navnode() {
            let BinavnodeNodeIndex::FixedDot(into) = around_node_weight.binavnode
            else {
                unreachable!();
            };
            return self.ghost_finish(drawing, into);
        }

        let Some(sense) = around_node_weight.maybe_sense else {
            return GhostVerdict::Failed(NavcorderException::CannotWrap);
        };

        let gear: GearIndex = match around_node_weight.binavnode {
            BinavnodeNodeIndex::FixedDot(dot) => dot.into(),
            BinavnodeNodeIndex::FixedBend(bend) => BendIndex::Fixed(bend).into(),
            BinavnodeNodeIndex::LooseBend(bend) => BendIndex::Loose(bend).into(),
        };

        let fail = |err: DrawingException| {
            GhostVerdict::Failed(NavcorderException::CannotDraw(
                DrawException::CannotWrapAround(gear, err),
            ))
        };

        let face_conditions = self.face_conditions();
        let gear_conditions = drawing.conditions(gear.into());
        let (from_circle, from_sense) = self.guide_from(drawing);

        let (to_circle, offset) = match gear {
            GearIndex::FixedDot(dot) => (
                drawing.dot_circle(dot.into(), self.width, face_conditions.as_ref()),
                drawing.clearance(gear_conditions.as_ref(), face_conditions.as_ref()),
            ),
            GearIndex::FixedBend(bend) => (
                drawing.bend_circle(bend.into(), self.width, face_conditions.as_ref()),
                drawing.clearance(face_conditions.as_ref(), gear_conditions.as_ref()),
            ),
            GearIndex::LooseBend(bend) => (
                drawing.bend_circle(bend.into(), self.width, face_conditions.as_ref()),
                drawing.clearance(face_conditions.as_ref(), gear_conditions.as_ref()),
            ),
        };

        let tangent = match math::bitangent(from_circle, from_sense, to_circle, Some(sense))
        {
            Ok(tangent) => tangent,
            Err(err) => {
                return GhostVerdict::Failed(NavcorderException::CannotDraw(err.into()))
            }
        };

        let tangent_start = tangent.start_point();
        let tangent_end = tangent.end_point();

        let layer = self.band_layer;
        let maybe_net = self.band_net;

        // Replicates `.extend_head(..)`: a cane head's face is moved to
        // the tangent's start point and each of its limbs is checked
        // under a tautology filter, exactly as `.move_dot(..)` does. The
        // face dot itself is loose and so its own check is a no-op.
        //
        // The old bend's length with its joint moved is captured here as
        // well, because `.step_to_intern(..)` measures it after the move.
        let prev_bend_length = match self.ghost_extend_head(drawing, tangent_start) {
            Ok(length) => length,
            Err(err) => return fail(err),
        };

        // Replicates `.add_seg(..)`'s checks inside `.add_cane(..)` on the
        // new cane's seg. The two dots that `.add_cane(..)` inserts cannot
        // fail their checks (loose dots never infringe) and cannot affect
        // later checks either, since they share the cane's net.
        let seg_shape = PrimitiveShape::Seg(SegShape {
            from: tangent_start,
            to: tangent_end,
            width: self.width,
        });

        // Limbs of the wraparound's overlapees are exempted as in
        // `.add_cane(..)`, and limbs of the wraparound itself as in
        // `.add_seg(..)`'s check inside it.
        let exempt_limbs = drawing.overlapee_limbs(gear.into());
        let around_limbs = PrimitiveIndex::from(gear)
            .primitive_ref(drawing)
            .limbs();

        if let Some(infringement) = drawing.find_infringement_of_shape(
            seg_shape,
            layer,
            maybe_net,
            &|_, infringee| {
                !exempt_limbs.contains(&infringee) && !around_limbs.contains(&infringee)
            },
        ) {
            return fail(infringement.into());
        }

        let require_different_net = match self.canes.last() {
            Some(PathCane::Virtual(..)) => false,
            _ => matches!(self.head.face(), DotIndex::Fixed(..)),
        };

        if let Some(collision) = drawing.detect_collision_of_shape(
            seg_shape,
            layer,
            maybe_net,
            require_different_net,
        ) {
            return fail(collision.into());
        }

        // The routed band's virtual segs — of the same net as the new
        // seg, so filtered out of the collision query above — can
        // collide with it; this is what prevents the band from
        // self-intersecting. They are checked exactly like real same-net
        // loose seg collidees.
        if !require_different_net {
            for cane in self.virtual_canes() {
                let virtual_seg = cane.seg_shape(self.width);
                if seg_shape.intersects(&virtual_seg) {
                    return fail(Collision(seg_shape, virtual_seg, cane.gear.into()).into());
                }
            }
        }

        // Replicates `.add_loose_bend(..)`'s same-net pre-check: it makes
        // no sense to wrap something around or under one of its
        // connectables.
        if let Some(net) = maybe_net {
            if let Some(around_net) = gear.primitive_ref(drawing).maybe_net() {
                if net == around_net {
                    return fail(AlreadyConnected(net, gear.into()).into());
                }
            }

            let mut outwards = gear.ref_(drawing).outwards();
            let mut outward_gears = vec![];
            while let Some(outward) = outwards.walk_next(drawing) {
                outward_gears.push(outward);
                if let Some(outward_net) = outward.primitive_ref(drawing).maybe_net() {
                    if net == outward_net {
                        return fail(AlreadyConnected(net, outward.into()).into());
                    }
                }
            }

            // A virtual cane's bend wrapping `around` or one of its
            // outwards is itself one of `around`'s outwards, so it is
            // checked exactly like a real one.
            for cane in self.virtual_canes() {
                if cane.gear == gear
                    || outward_gears
                        .iter()
                        .any(|outward| GearIndex::LooseBend(*outward) == cane.gear)
                {
                    return fail(AlreadyConnected(net, cane.gear.into()).into());
                }
            }
        }

        // Replicates the check on the new bend's shape. Its joints are
        // both at the tangent's end point, and its inner circle encloses
        // the wraparound's outer circle by `offset`, same as
        // `.inner_radius(..)` computes it.
        let inner_circle = match gear.primitive_ref(drawing).shape() {
            PrimitiveShape::Dot(dot) => Circle {
                pos: dot.circle.pos,
                r: dot.circle.r + offset,
            },
            PrimitiveShape::Bend(bend) => Circle {
                pos: bend.inner_circle.pos,
                r: bend.inner_circle.r + bend.width + offset,
            },
            PrimitiveShape::Seg(..) => unreachable!(),
        };

        if let Some(infringement) = drawing.find_infringement_of_shape(
            PrimitiveShape::Bend(BendShape {
                from: tangent_end,
                to: tangent_end,
                inner_circle,
                width: self.width,
            }),
            layer,
            maybe_net,
            &|_, infringee| !exempt_limbs.contains(&infringee),
        ) {
            return fail(infringement.into());
        }

        // The returned length is the old bend's arc after the move plus
        // the new cane's length, which is just the fresh seg since the
        // fresh bend is always zero-length.
        GhostVerdict::Drawn {
            replay: GhostReplay::Wrap(VirtualCane {
                gear,
                sense,
                tangent,
                inner_circle,
            }),
            length: prev_bend_length + Euclidean::distance(&tangent_end, &tangent_start),
        }
    }

    /// Evaluates a finish step without mutating the board, replicating
    /// `.finish_in_dot(..)`'s checks: the moved head's limb infringements
    /// and the termseg's infringement and collision checks.
    fn ghost_finish<CW: Clone, Cel: Copy, R: AccessRules>(
        &self,
        drawing: &Drawing<CW, Cel, R>,
        into: FixedDotIndex,
    ) -> GhostVerdict {
        let fail = |err: DrawingException| {
            GhostVerdict::Failed(NavcorderException::CannotDraw(
                DrawException::CannotFinishIn(into, err),
            ))
        };

        // Replicates `.guide_for_head_into_dot(..)`.
        let (from_circle, from_sense) = self.guide_from(drawing);
        let into_pos = drawing.primitive_ref(into).weight().0.circle.pos;
        let bitangent = match math::bitangent(
            from_circle,
            from_sense,
            Circle {
                pos: into_pos,
                r: 0.0,
            },
            None,
        ) {
            Ok(bitangent) => bitangent,
            Err(err) => {
                return GhostVerdict::Failed(NavcorderException::CannotDraw(err.into()))
            }
        };

        let tangent_start = bitangent.start_point();

        // Replicates `.extend_head(..)` on the moved face.
        let prev_bend_length = match self.ghost_extend_head(drawing, tangent_start) {
            Ok(length) => length,
            Err(err) => return fail(err),
        };

        // The termseg's shape: a seq seg from the destination dot to the
        // moved face for a cane head, a lone seg from the face to the
        // destination dot for a bare head.
        let seg_shape = PrimitiveShape::Seg(if self.canes.is_empty() {
            SegShape {
                from: tangent_start,
                to: into_pos,
                width: self.width,
            }
        } else {
            SegShape {
                from: into_pos,
                to: tangent_start,
                width: self.width,
            }
        });

        // Both checks are replicated exactly as `.add_seg(..)` runs
        // them: a tautology-filtered infringement check and a collision
        // check; either way the termseg joins a fixed dot, so same-net
        // collidees — including the band's own segs — are exempted.
        if let Some(infringement) = drawing.find_infringement_of_shape(
            seg_shape,
            self.band_layer,
            self.band_net,
            &|_, _| true,
        ) {
            return fail(infringement.into());
        }

        if let Some(collision) = drawing.detect_collision_of_shape(
            seg_shape,
            self.band_layer,
            self.band_net,
            true,
        ) {
            return fail(collision.into());
        }

        GhostVerdict::Drawn {
            replay: GhostReplay::Finish,
            length: prev_bend_length
                + Euclidean::distance(&bitangent.end_point(), &tangent_start),
        }
    }

    /// Retreat the navcord and the currently routed band by one step.
    #[debug_ensures(self.path.len() == old(self.path.len() - 1))]
    pub fn step_back<R: AccessRules>(
        &mut self,
        layout: &mut Layout<R>,
    ) -> Result<(), NavcorderException> {
        crate::metrics::incr(&crate::metrics::METRICS.step_backs);
        let _timer = crate::metrics::Timer::start(&crate::metrics::METRICS.step_back_nanos);
        self.step_back_intern(layout)
    }

    fn step_back_intern<R: AccessRules>(
        &mut self,
        layout: &mut Layout<R>,
    ) -> Result<(), NavcorderException> {
        if let Some(final_termseg) = self.maybe_final_termseg {
            layout.remove_termseg(&mut self.recorder, final_termseg);
            self.maybe_final_termseg = None;
        } else if self.virtual_finish {
            self.virtual_finish = false;
        } else {
            match self.canes.pop() {
                // A virtual cane leaves nothing on the board to undo.
                Some(PathCane::Virtual(..)) => (),
                Some(PathCane::Real(head)) => {
                    self.head = layout.undo_cane(&mut self.recorder, head).unwrap();
                }
                // "can't unwrap"
                None => return Err(NavcorderException::CannotWrap),
            }
        }

        // Now that the last head of the currently routed band was deleted, pop
        // the last navnode from the currently attempted path so that it is up
        // to date.
        self.path.pop();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::{fs::File, io::BufReader};

    use approx::assert_abs_diff_eq;
    use geo::{point, LineString, Polygon};

    use super::*;
    use crate::{
        autorouter::{ratline::RatlineUid, Autorouter},
        board::edit::BoardEdit,
        drawing::dot::{FixedDotWeight, GeneralDotWeight},
        router::{navcorder::Navcorder, RouterOptions},
        specctra::{design::SpecctraDesign, mesadata::SpecctraMesadata},
    };

    const WIDTH: f64 = 100.0;

    /// A trivial ruleset giving every same-layer pair of primitives a
    /// fixed clearance, for building boards without a `.dsn` file.
    struct TestRules;

    impl AccessRules for TestRules {
        fn clearance(
            &self,
            conditions1: &Conditions<'_>,
            conditions2: &Conditions<'_>,
        ) -> Option<f64> {
            (conditions1.maybe_layer == conditions2.maybe_layer).then_some(500.0)
        }

        fn largest_clearance(&self, _: Option<usize>) -> f64 {
            500.0
        }
    }

    /// Loads the 0603 breakout board and returns an autorouter, a
    /// navmesh and a navcord started on its first layer-0 ratline.
    fn navcord_on_0603() -> (Autorouter<SpecctraMesadata>, Navmesh, Navcord) {
        let design_file = File::open("tests/unilayer/0603_breakout/0603_breakout.dsn").unwrap();
        let design = SpecctraDesign::load(BufReader::new(design_file)).unwrap();
        let mut autorouter = Autorouter::new(design.make_board(&mut BoardEdit::new())).unwrap();

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
        let (origin, destination) = ratline.ref_(&autorouter).endpoint_dots();

        let navmesh = Navmesh::new(
            autorouter.board().layout(),
            origin,
            destination,
            RouterOptions {
                wrap_around_bands: true,
                squeeze_through_under_bends: false,
                routed_band_width: WIDTH,
            },
        )
        .unwrap();

        let navcord = Navcorder::start(
            autorouter.board_mut().layout_mut(),
            LayoutEdit::new(),
            origin,
            navmesh.origin_navnode(),
            WIDTH,
        );
        (autorouter, navmesh, navcord)
    }

    /// Builds a navcord on a minimal synthetic board: a source and a
    /// destination pad of net 1 on the x-axis, with two unrelated pads
    /// of net 2 between them to wrap around. The controlled geometry
    /// lets tests exercise specific failure paths deterministically,
    /// unlike real fixture boards where earlier infringement checks
    /// may mask them.
    fn navcord_on_synthetic(
    ) -> (Layout<TestRules>, Navmesh, Navcord, FixedDotIndex, FixedDotIndex) {
        let mut layout = Layout::new(Drawing::<_, _, TestRules>::new(
            TestRules,
            1,
            Polygon::new(
                LineString::from(vec![
                    (-10000.0, -10000.0),
                    (30000.0, -10000.0),
                    (30000.0, 10000.0),
                    (-10000.0, 10000.0),
                ]),
                vec![],
            ),
            None,
        ));

        let mut recorder = LayoutEdit::new();
        let mut add_dot = |x: f64, y: f64, net: usize| {
            layout.add_fixed_dot_infringably(
                &mut recorder,
                FixedDotWeight(GeneralDotWeight {
                    circle: Circle {
                        pos: point!(x: x, y: y),
                        r: 100.0,
                    },
                    layer: 0,
                    maybe_net: Some(net),
                }),
            )
        };

        let source = add_dot(0.0, 0.0, 1);
        let gear1 = add_dot(6000.0, 0.0, 2);
        let gear2 = add_dot(12000.0, 0.0, 2);
        let destination = add_dot(20000.0, 0.0, 1);

        let navmesh = Navmesh::new(
            &layout,
            source,
            destination,
            RouterOptions {
                wrap_around_bands: true,
                squeeze_through_under_bends: false,
                routed_band_width: WIDTH,
            },
        )
        .unwrap();

        let navcord = Navcorder::start(
            &mut layout,
            LayoutEdit::new(),
            source,
            navmesh.origin_navnode(),
            WIDTH,
        );
        (layout, navmesh, navcord, gear1, gear2)
    }

    /// All navnodes that wrap the given fixed dot, one per rotation
    /// sense.
    fn fixed_dot_navnodes(navmesh: &Navmesh, dot: FixedDotIndex) -> Vec<NavnodeIndex> {
        navmesh
            .graph()
            .node_indices()
            .map(NavnodeIndex)
            .filter(|navnode| {
                navmesh.node_weight(*navnode).unwrap().binavnode
                    == BinavnodeNodeIndex::FixedDot(dot)
            })
            .collect()
    }

    /// Whether the result is the same-net refusal a wrap produces when
    /// the band is already connected to `around` or its outwards.
    fn is_already_connected(result: &Result<f64, NavcorderException>) -> bool {
        matches!(
            result,
            Err(NavcorderException::CannotDraw(
                DrawException::CannotWrapAround(_, DrawingException::AlreadyConnected(..),)
            ))
        )
    }

    /// Finds the first (gear, other_gear) navnode pair that
    /// `.step_to(..)` accepts in order. On return the navcord holds
    /// exactly those two virtual canes.
    fn first_successful_pair(
        navcord: &mut Navcord,
        layout: &mut Layout<impl AccessRules>,
        navmesh: &Navmesh,
        gear: FixedDotIndex,
        other_gear: FixedDotIndex,
    ) -> (NavnodeIndex, NavnodeIndex) {
        for &first in &fixed_dot_navnodes(navmesh, gear) {
            if navcord.step_to(layout, navmesh, first).is_err() {
                continue;
            }
            for &second in &fixed_dot_navnodes(navmesh, other_gear) {
                if navcord.step_to(layout, navmesh, second).is_ok() {
                    return (first, second);
                }
            }
            navcord.step_back(layout).unwrap();
        }
        panic!("no wrappable navnode pair")
    }

    /// Collects the verdicts of rewrapping `gear` while two virtual
    /// canes are on the path, over all its navnodes. Successful rewraps
    /// are undone, so the navcord keeps its two canes.
    fn rewrap_verdicts(
        navcord: &mut Navcord,
        layout: &mut Layout<impl AccessRules>,
        navmesh: &Navmesh,
        gear: FixedDotIndex,
    ) -> Vec<Result<f64, NavcorderException>> {
        fixed_dot_navnodes(navmesh, gear)
            .iter()
            .map(|&navnode| {
                let result = navcord.step_to(layout, navmesh, navnode);
                if result.is_ok() {
                    navcord.step_back(layout).unwrap();
                }
                result
            })
            .collect()
    }

    /// Steps to the first navnode that `.step_to(..)` accepts, and
    /// returns it. Since a failed step leaves the path unchanged, it is
    /// safe to try every navnode in turn.
    fn step_to_any(
        navcord: &mut Navcord,
        layout: &mut Layout<impl AccessRules>,
        navmesh: &Navmesh,
    ) -> NavnodeIndex {
        for to in navmesh.graph().node_indices().map(NavnodeIndex) {
            if to != navmesh.destination_navnode()
                && navcord.step_to(layout, navmesh, to).is_ok()
            {
                return to;
            }
        }
        panic!("no wrappable navnode")
    }

    fn assert_same_point(lhs: Point, rhs: Point) {
        assert_abs_diff_eq!(lhs.x(), rhs.x(), epsilon = 1e-9);
        assert_abs_diff_eq!(lhs.y(), rhs.y(), epsilon = 1e-9);
    }

    fn assert_same_circle(lhs: Circle, rhs: Circle) {
        assert_same_point(lhs.pos, rhs.pos);
        assert_abs_diff_eq!(lhs.r, rhs.r, epsilon = 1e-9);
    }

    fn assert_same_shape(lhs: PrimitiveShape, rhs: PrimitiveShape) {
        match (lhs, rhs) {
            (PrimitiveShape::Dot(lhs), PrimitiveShape::Dot(rhs)) => {
                assert_same_circle(lhs.circle, rhs.circle)
            }
            (PrimitiveShape::Seg(lhs), PrimitiveShape::Seg(rhs)) => {
                assert_same_point(lhs.from, rhs.from);
                assert_same_point(lhs.to, rhs.to);
                assert_abs_diff_eq!(lhs.width, rhs.width, epsilon = 1e-9);
            }
            (PrimitiveShape::Bend(lhs), PrimitiveShape::Bend(rhs)) => {
                assert_same_point(lhs.from, rhs.from);
                assert_same_point(lhs.to, rhs.to);
                assert_same_circle(lhs.inner_circle, rhs.inner_circle);
                assert_abs_diff_eq!(lhs.width, rhs.width, epsilon = 1e-9);
            }
            (lhs, rhs) => panic!("shape mismatch: {lhs:?} vs {rhs:?}"),
        }
    }

    /// A step must be recorded purely virtually: no geometry may be
    /// drawn into the board.
    #[test]
    fn virtual_step_does_not_mutate_board() {
        let (mut autorouter, navmesh, mut navcord) = navcord_on_0603();
        let layout = autorouter.board_mut().layout_mut();
        let node_count = layout.drawing().node_count();

        let _ = step_to_any(&mut navcord, layout, &navmesh);

        assert_eq!(layout.drawing().node_count(), node_count);
        assert_eq!(navcord.path.len(), 2);
        assert_eq!(navcord.canes.len(), 1);
        assert!(matches!(
            navcord.canes.last(),
            Some(PathCane::Virtual(..))
        ));
    }

    /// Materializing a virtual cane must draw exactly the geometry the
    /// virtual evaluation computed.
    #[test]
    fn materialize_draws_virtual_geometry() {
        let (mut autorouter, navmesh, mut navcord) = navcord_on_0603();
        let layout = autorouter.board_mut().layout_mut();

        let _ = step_to_any(&mut navcord, layout, &navmesh);
        let Some(PathCane::Virtual(virtual_cane)) = navcord.canes.last() else {
            unreachable!()
        };
        let virtual_cane = virtual_cane.clone();

        let node_count = layout.drawing().node_count();
        navcord.materialize(layout, &navmesh).unwrap();

        // A cane draws two dots, a seg and a bend.
        assert_eq!(layout.drawing().node_count(), node_count + 4);
        let Some(PathCane::Real(head)) = navcord.canes.last() else {
            panic!("cane should have been materialized")
        };

        let real_seg_shape = PrimitiveIndex::from(head.cane.seg)
            .primitive_ref(layout.drawing())
            .shape();
        let real_bend_shape = PrimitiveIndex::from(head.cane.bend)
            .primitive_ref(layout.drawing())
            .shape();

        assert_same_shape(real_seg_shape, virtual_cane.seg_shape(WIDTH));
        // While the cane is the head its face sits at the tangent's end
        // point, so the bend is zero-length.
        assert_same_shape(
            real_bend_shape,
            virtual_cane.bend_shape(WIDTH, virtual_cane.tangent.end_point()),
        );
    }

    /// A virtual cane of an extended head must materialize with its
    /// bend's face joint moved, exactly as a real draw leaves it.
    #[test]
    fn materialize_extends_previous_bend() {
        let (mut autorouter, navmesh, mut navcord) = navcord_on_0603();
        let layout = autorouter.board_mut().layout_mut();

        let _ = step_to_any(&mut navcord, layout, &navmesh);
        let Some(PathCane::Virtual(first)) = navcord.canes.last() else {
            unreachable!()
        };
        let first = first.clone();

        let _ = step_to_any(&mut navcord, layout, &navmesh);
        let Some(PathCane::Virtual(second)) = navcord.canes.last() else {
            unreachable!()
        };
        let second = second.clone();

        navcord.materialize(layout, &navmesh).unwrap();

        // The first cane's bend must have its face joint at the second
        // cane's tangent start point.
        let Some(PathCane::Real(first_head)) = navcord.canes.first() else {
            unreachable!()
        };
        let real_bend_shape = PrimitiveIndex::from(first_head.cane.bend)
            .primitive_ref(layout.drawing())
            .shape();
        assert_same_shape(
            real_bend_shape,
            first.bend_shape(WIDTH, second.tangent.start_point()),
        );
    }

    /// `.step_back(..)` on a virtual step only pops state; the board is
    /// untouched in both directions.
    #[test]
    fn step_back_undoes_virtual_step() {
        let (mut autorouter, navmesh, mut navcord) = navcord_on_0603();
        let layout = autorouter.board_mut().layout_mut();
        let node_count = layout.drawing().node_count();

        let _ = step_to_any(&mut navcord, layout, &navmesh);
        navcord.step_back(layout).unwrap();

        assert_eq!(navcord.path.len(), 1);
        assert!(navcord.canes.is_empty());
        assert_eq!(layout.drawing().node_count(), node_count);
    }

    /// `.step_back(..)` on a materialized cane removes the real
    /// geometry.
    #[test]
    fn step_back_undoes_materialized_cane() {
        let (mut autorouter, navmesh, mut navcord) = navcord_on_0603();
        let layout = autorouter.board_mut().layout_mut();
        let node_count = layout.drawing().node_count();

        let _ = step_to_any(&mut navcord, layout, &navmesh);
        navcord.materialize(layout, &navmesh).unwrap();
        navcord.step_back(layout).unwrap();

        assert_eq!(navcord.path.len(), 1);
        assert!(navcord.canes.is_empty());
        assert_eq!(layout.drawing().node_count(), node_count);
    }

    /// A finish step evaluated virtually marks the path without drawing
    /// and is undone by `.step_back(..)`.
    #[test]
    fn virtual_finish_is_undone_by_step_back() {
        let (mut autorouter, navmesh, mut navcord) = navcord_on_0603();
        let layout = autorouter.board_mut().layout_mut();
        let node_count = layout.drawing().node_count();

        let result = navcord.step_to(layout, &navmesh, navmesh.destination_navnode());

        if result.is_ok() {
            assert!(navcord.virtual_finish);
            assert!(matches!(
                navcord.last_replay(),
                Some(GhostReplay::Finish)
            ));
            navcord.step_back(layout).unwrap();
            assert!(!navcord.virtual_finish);
            assert_eq!(navcord.path.len(), 1);
        } else {
            // A failed finish may not leave a marker behind.
            assert!(!navcord.virtual_finish);
        }

        assert_eq!(layout.drawing().node_count(), node_count);
    }

    /// Rewrapping a gear that a virtual cane already wraps must be
    /// refused, exactly like the real draw's same-net pre-check on
    /// `around`'s outwards. Some sense combinations fail earlier on seg
    /// collisions — whichever failure mode applies, the rewrap may
    /// never succeed, since it would draw a band through its own bend.
    #[test]
    fn rewrapping_virtual_gear_is_refused() {
        let (mut layout, navmesh, mut navcord, gear1, gear2) = navcord_on_synthetic();

        let gear1_navnodes = fixed_dot_navnodes(&navmesh, gear1);
        assert_eq!(gear1_navnodes.len(), 2);

        let mut verdicts = vec![];
        for &first in &gear1_navnodes {
            if navcord.step_to(&mut layout, &navmesh, first).is_err() {
                continue;
            }
            for &second in &fixed_dot_navnodes(&navmesh, gear2) {
                if navcord.step_to(&mut layout, &navmesh, second).is_err() {
                    continue;
                }
                verdicts.extend(rewrap_verdicts(
                    &mut navcord,
                    &mut layout,
                    &navmesh,
                    gear1,
                ));
                navcord.step_back(&mut layout).unwrap();
            }
            navcord.step_back(&mut layout).unwrap();
        }

        assert!(
            !verdicts.is_empty(),
            "no (gear, other gear) wrap pair succeeded"
        );
        assert!(
            verdicts.iter().all(Result::is_err),
            "rewrapping a gear the band already wraps succeeded: {verdicts:?}"
        );
        assert!(
            verdicts.iter().any(is_already_connected),
            "no rewrap reached the same-net check: {verdicts:?}"
        );
    }

    /// Redrawing the path after backtracking — what `.rework_path(..)`
    /// does on every frontier pop — must restore the same virtual
    /// state, so that later steps are still checked against the
    /// band's virtual bends.
    #[test]
    fn replayed_steps_restore_virtual_state() {
        let (mut layout, navmesh, mut navcord, gear1, gear2) = navcord_on_synthetic();

        let (first, second) =
            first_successful_pair(&mut navcord, &mut layout, &navmesh, gear1, gear2);

        let recorded: Vec<(GearIndex, Point, Point)> = navcord
            .canes
            .iter()
            .map(|cane| match cane {
                PathCane::Virtual(cane) => (
                    cane.gear,
                    cane.tangent.start_point(),
                    cane.tangent.end_point(),
                ),
                PathCane::Real(..) => panic!("expected a virtual cane"),
            })
            .collect();
        assert_eq!(recorded.len(), 2);

        navcord.step_back(&mut layout).unwrap();
        navcord.step_back(&mut layout).unwrap();
        assert!(navcord.canes.is_empty());

        Navcorder::path(&mut layout, &navmesh, &mut navcord, &[first, second]).unwrap();

        let replayed: Vec<(GearIndex, Point, Point)> = navcord
            .canes
            .iter()
            .map(|cane| match cane {
                PathCane::Virtual(cane) => (
                    cane.gear,
                    cane.tangent.start_point(),
                    cane.tangent.end_point(),
                ),
                PathCane::Real(..) => panic!("expected a virtual cane"),
            })
            .collect();
        assert_eq!(replayed, recorded);

        // The restored virtual state is part of the same-net check, so
        // rewrapping the first gear must still be refused.
        let verdicts = rewrap_verdicts(&mut navcord, &mut layout, &navmesh, gear1);
        assert!(
            verdicts.iter().all(Result::is_err),
            "rewrapping a gear the band already wraps succeeded: {verdicts:?}"
        );
        assert!(
            verdicts.iter().any(is_already_connected),
            "no rewrap reached the same-net check: {verdicts:?}"
        );
    }

    /// `generation` must advance on every geometry change — a direct
    /// mutation as well as the `.apply(..)` path used by undo/redo —
    /// so caches can cheaply detect a changed board.
    #[test]
    fn generation_advances_on_mutation_and_apply() {
        use crate::geometry::edit::{ApplyGeometryEdit, Edit};

        let (mut layout, ..) = navcord_on_synthetic();
        let before = layout.drawing().generation();

        let mut recorder = LayoutEdit::new();
        layout.add_fixed_dot_infringably(
            &mut recorder,
            FixedDotWeight(GeneralDotWeight {
                circle: Circle {
                    pos: point!(x: 15000.0, y: 5000.0),
                    r: 100.0,
                },
                layer: 0,
                maybe_net: Some(3),
            }),
        );
        let mutated = layout.drawing().generation();
        assert!(mutated > before);

        layout.apply(&recorder.reverse());
        assert!(layout.drawing().generation() > mutated);
    }
}
