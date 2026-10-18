// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::collections::HashMap;

use contracts_try::debug_ensures;
use geo::algorithm::line_measures::{Distance, Euclidean};
use petgraph::{data::DataMap, visit::Walker};

use crate::{
    drawing::{
        band::BandTermsegIndex,
        bend::BendIndex,
        dot::{DotIndex, FixedDotIndex},
        gear::{GearIndex, WalkOutwards},
        graph::{GetMaybeNet, MakePrimitiveRef, PrimitiveIndex},
        head::{BareHead, CaneHead, GetFace, Head},
        primitive::{GetLimbs, MakePrimitiveShape},
        rules::AccessRules,
        seg::SegIndex,
        AlreadyConnected, DrawingException,
    },
    geometry::{
        primitive::{BendShape, PrimitiveShape, SegShape},
        shape::MeasureLength,
        GetLayer,
    },
    graph::MakeRef,
    layout::{Layout, LayoutEdit},
    math::Circle,
};

use super::{
    draw::{Draw, DrawException},
    navcorder::{Navcorder, NavcorderException},
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
    /// The probe was drawn successfully, with the recorded track length.
    Drawn(f64),
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
/// speculative wrap step.
#[derive(Debug)]
pub enum GhostVerdict {
    /// The wrap was evaluated without drawing and is reported drawable
    /// with the recorded track length.
    Drawn(f64),
    /// The wrap was evaluated without drawing and reported to fail
    /// identically to the real draw.
    Failed(NavcorderException),
    /// The step cannot be evaluated without drawing; the real
    /// `.step_to(..)` must be used.
    Unsupported,
}

#[derive(Debug)]
pub struct Navcord {
    /// The layout edit which we are currently recording.
    pub recorder: LayoutEdit,
    /// The currently attempted path.
    pub path: Vec<NavnodeIndex>,
    /// The head of the currently routed band.
    pub head: Head,
    /// If the band is finished, stores the termseg that was used to finish it.
    pub maybe_final_termseg: Option<BandTermsegIndex>,
    /// The width of the currently routed band.
    pub width: f64,
    /// Memoized probe verdicts keyed by the navcord path state, initial
    /// parent navnode, and probed navnode.
    pub probe_memo:
        HashMap<(Box<[NavnodeIndex]>, Option<NavnodeIndex>, NavnodeIndex), ProbeMemoEntry>,
    /// Lengths of draws that succeeded without squeezing, keyed by the
    /// navcord path state and stepped-to navnode. Such draws can be
    /// replayed without repeating their infringement and collision
    /// checks, because in the same path state they deterministically
    /// produce the same geometry.
    pub drawn_memo: HashMap<Box<[NavnodeIndex]>, HashMap<NavnodeIndex, f64>>,
    /// Whether the last step on `.path` is a virtual (memoized) probe
    /// that drew no geometry. Such steps are undone by simply popping
    /// them in `.step_back(..)`.
    pub virtual_probe: bool,
}

impl Navcord {
    /// Create a new navcord.
    pub fn new(
        recorder: LayoutEdit,
        source: FixedDotIndex,
        source_navnode: NavnodeIndex,
        width: f64,
    ) -> Navcord {
        Self {
            recorder,
            path: vec![source_navnode],
            head: BareHead { face: source }.into(),
            maybe_final_termseg: None,
            width,
            probe_memo: HashMap::new(),
            drawn_memo: HashMap::new(),
            virtual_probe: false,
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
    /// replay draws that were already verified in the same state, that is,
    /// draws recorded in `.drawn_memo`.
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
        let length = if to == navmesh.destination_navnode() {
            let to_binavnode = navmesh.node_weight(to).unwrap().binavnode;
            let BinavnodeNodeIndex::FixedDot(to_dot) = to_binavnode else {
                unreachable!();
            };

            let final_termseg = layout.finish(navmesh, self, to_dot)?;
            self.maybe_final_termseg = Some(final_termseg);

            let final_termseg_length = final_termseg
                .primitive_ref(layout.drawing())
                .shape()
                .length();

            let bend_length = match self.head {
                Head::Cane(old_cane_head) => layout
                    .drawing()
                    .primitive_ref(old_cane_head.cane.bend)
                    .shape()
                    .length(),
                Head::Bare(..) => 0.0,
            };

            final_termseg_length + bend_length

            // NOTE: We don't update the head here because there is currently
            // no head variant that consists only of a seg, and I'm not sure if
            // there should be one.
        } else {
            let old_head = self.head;

            // If this draw succeeded without squeezing earlier in the same
            // path state, it can be replayed without repeating its checks.
            let memoized = self
                .drawn_memo
                .get(self.path.as_slice())
                .and_then(|lengths| lengths.get(&to))
                .copied();

            let (head, squeezed) = match memoized {
                Some(..) => {
                    crate::metrics::incr(&crate::metrics::METRICS.step_to_trusted);
                    (self.wrap_trusted(layout, navmesh, self.head, to)?, false)
                }
                None => self.wrap(layout, navmesh, self.head, to)?,
            };
            self.head = head.into();

            let prev_bend_length = match old_head {
                Head::Cane(old_cane_head) => layout
                    .drawing()
                    .primitive_ref(old_cane_head.cane.bend)
                    .shape()
                    .length(),
                Head::Bare(..) => 0.0,
            };

            let length = prev_bend_length
                // NOTE: the probe's bend length is always 0 here because such is
                // the initial state of a cane (before getting extended, but this
                // is never done for probes). So we could as well only measure the
                // seg's length.
                + self.head.ref_(layout.drawing()).length();

            if memoized.is_none() && !squeezed {
                self.drawn_memo
                    .entry(self.path.clone().into_boxed_slice())
                    .or_default()
                    .insert(to, length);
            }

            length
        };

        // Now that the new part of the trace has been created, push the
        // navnode `to` onto the currently attempted path to start from it
        // on the next `.step_to(...)` call or retreat from it later using
        // `.step_back(...)`.
        self.path.push(to);
        Ok(length)
    }

    /// Evaluates `.step_to(..)` for a wrap step without mutating the
    /// board, replicating exactly the checks the real draw performs: the
    /// moved head's limb infringements, the new seg's infringement and
    /// collision checks, and the new bend's connectivity and infringement
    /// checks.
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
        if to == navmesh.destination_navnode() {
            // Finishing steps are rare enough to not be worth replicating.
            return GhostVerdict::Unsupported;
        }

        let drawing = layout.drawing();
        let around_node_weight = navmesh.node_weight(to).unwrap();

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

        let tangent = match gear {
            GearIndex::FixedDot(dot) => drawing
                .guide_for_head_around_dot(&self.head, dot.into(), sense, self.width),
            GearIndex::FixedBend(bend) => drawing
                .guide_for_head_around_bend(&self.head, bend.into(), sense, self.width),
            GearIndex::LooseBend(bend) => drawing
                .guide_for_head_around_bend(&self.head, bend.into(), sense, self.width),
        };

        let tangent = match tangent {
            Ok(tangent) => tangent,
            Err(err) => {
                return GhostVerdict::Failed(NavcorderException::CannotDraw(err.into()))
            }
        };

        let offset = match gear {
            GearIndex::FixedDot(dot) => drawing
                .offset_for_guide_for_head_around_dot(&self.head, dot.into(), self.width),
            GearIndex::FixedBend(bend) => drawing
                .offset_for_guide_for_head_around_bend(&self.head, bend.into(), self.width),
            GearIndex::LooseBend(bend) => drawing
                .offset_for_guide_for_head_around_bend(&self.head, bend.into(), self.width),
        };

        let tangent_start = tangent.start_point();
        let tangent_end = tangent.end_point();

        let face = self.head.face();
        let face_primitive = face.primitive_ref(drawing);
        let layer = face_primitive.layer();
        let maybe_net = face_primitive.maybe_net();

        // Replicates `.extend_head(..)`: a cane head's face is moved to
        // the tangent's start point and each of its limbs is checked
        // under a tautology filter, exactly as `.move_dot(..)` does. The
        // face dot itself is loose and so its own check is a no-op.
        //
        // The old bend's length with its joint moved is captured here as
        // well, because `.step_to_intern(..)` measures it after the move.
        let mut prev_bend_length = 0.0;
        if let Head::Cane(head) = self.head {
            for limb in head.face.primitive_ref(drawing).limbs() {
                let mut shape = limb.primitive_ref(drawing).shape();
                match &mut shape {
                    PrimitiveShape::Seg(seg_shape) => {
                        let seg: SegIndex = limb.try_into().unwrap();
                        if drawing.geometry().seg_joints(seg).0
                            == DotIndex::Loose(head.face)
                        {
                            seg_shape.from = tangent_start;
                        } else {
                            seg_shape.to = tangent_start;
                        }
                    }
                    PrimitiveShape::Bend(bend_shape) => {
                        let bend: BendIndex = limb.try_into().unwrap();
                        if drawing.geometry().bend_joints(bend).0
                            == DotIndex::Loose(head.face)
                        {
                            bend_shape.from = tangent_start;
                        } else {
                            bend_shape.to = tangent_start;
                        }
                    }
                    PrimitiveShape::Dot(..) => unreachable!(),
                }

                if limb == PrimitiveIndex::LooseBend(head.cane.bend) {
                    prev_bend_length = shape.length();
                }

                if let Some(infringement) = drawing
                    .find_infringement_of_shape(shape, layer, maybe_net, &|_, _| true)
                {
                    return fail(infringement.into());
                }
            }
        }

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

        if let Some(collision) = drawing.detect_collision_of_shape(
            seg_shape,
            layer,
            maybe_net,
            matches!(face, DotIndex::Fixed(..)),
        ) {
            return fail(collision.into());
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
            while let Some(outward) = outwards.walk_next(drawing) {
                if let Some(outward_net) = outward.primitive_ref(drawing).maybe_net() {
                    if net == outward_net {
                        return fail(AlreadyConnected(net, outward.into()).into());
                    }
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
        GhostVerdict::Drawn(
            prev_bend_length + Euclidean::distance(&tangent_end, &tangent_start),
        )
    }

    /// Retreat the navcord and the currently routed band by one step.
    #[debug_ensures(self.path.len() == old(self.path.len() - 1))]
    pub fn step_back<R: AccessRules>(
        &mut self,
        layout: &mut Layout<R>,
    ) -> Result<(), NavcorderException> {
        crate::metrics::incr(&crate::metrics::METRICS.step_backs);
        let _timer = crate::metrics::Timer::start(&crate::metrics::METRICS.step_back_nanos);

        if self.virtual_probe {
            // The last step was a memoized probe that drew no geometry;
            // it leaves no head behind, so it is undone by simply
            // popping it from the path.
            self.path.pop();
            self.virtual_probe = false;
            return Ok(());
        }

        self.step_back_intern(layout)
    }

    fn step_back_intern<R: AccessRules>(
        &mut self,
        layout: &mut Layout<R>,
    ) -> Result<(), NavcorderException> {
        if let Some(final_termseg) = self.maybe_final_termseg {
            layout.remove_termseg(&mut self.recorder, final_termseg);
            self.maybe_final_termseg = None;
        } else {
            if let Head::Cane(head) = self.head {
                self.head = layout.undo_cane(&mut self.recorder, head).unwrap();
            } else {
                // "can't unwrap"
                return Err(NavcorderException::CannotWrap);
            }
        }

        // Now that the last head of the currently routed band was deleted, pop
        // the last navnode from the currently attempted path so that it is up
        // to date.
        self.path.pop();
        Ok(())
    }
}
