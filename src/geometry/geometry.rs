// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use core::marker::PhantomData;
use enum_dispatch::enum_dispatch;
use geo::Point;
use petgraph::{
    stable_graph::StableDiGraph,
    visit::{EdgeRef, NodeIndexable, Walker},
    Direction::{Incoming, Outgoing},
};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

use crate::{
    drawing::{
        bend::{BendWeight, FixedBendWeight, LooseBendWeight},
        dot::{DotWeight, FixedDotWeight, LooseDotWeight},
        graph::PrimitiveWeight,
        primitive::PrimitiveRef,
        seg::{FixedSegWeight, LoneLooseSegWeight, SegWeight, SeqLooseSegWeight},
    },
    geometry::{
        compound::ManageCompounds,
        primitive::{BendShape, DotShape, PrimitiveShape, SegShape},
    },
    graph::{GenericIndex, GetIndex},
    math::Circle,
};

pub trait Retag {
    type Index: Sized + GetIndex + PartialEq + Copy;
    fn retag(&self, index: usize) -> Self::Index;
}

#[enum_dispatch]
pub trait GetLayer {
    fn layer(&self) -> usize;
}

#[enum_dispatch]
pub trait GetSetPos {
    fn pos(&self) -> Point;
    fn set_pos(&mut self, pos: Point);
}

#[enum_dispatch]
pub trait GetWidth {
    fn width(&self) -> f64;
}

#[enum_dispatch]
pub trait GetOffset {
    fn offset(&self) -> f64;
}

#[enum_dispatch]
pub trait SetOffset: GetOffset {
    fn set_offset(&mut self, offset: f64);
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum GeometryLabel<Cel> {
    Joined,
    Outer,
    Core,
    Compound(Cel),
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
pub enum GenericNode<P, C> {
    Primitive(P),
    Compound(C),
    /// A slot freed by a removal that still-pending edits may need to
    /// restore at this exact index. Tombstoning keeps the slot
    /// occupied so `add_node` cannot recycle it while its index is
    /// still referenced by a reversible edit: `update_node` on an
    /// occupied slot swaps the weight in place — keeping the
    /// incumbent's edges — so recycling a pending index would
    /// silently corrupt the graph instead of failing.
    Removed,
}

impl<P: GetIndex, C: GetIndex> GetIndex for GenericNode<P, C> {
    fn index(&self) -> usize {
        match self {
            Self::Primitive(x) => x.index(),
            Self::Compound(x) => x.index(),
            Self::Removed => {
                unreachable!("tombstone nodes carry no index")
            }
        }
    }
}

pub trait AccessDotWeight: GetSetPos + GetWidth + Copy {}
impl<T: GetSetPos + GetWidth + Copy> AccessDotWeight for T {}

pub trait AccessSegWeight: GetWidth + Copy {}
impl<T: GetWidth + Copy> AccessSegWeight for T {}

pub trait AccessBendWeight: SetOffset + GetWidth + Copy {}
impl<T: SetOffset + GetWidth + Copy> AccessBendWeight for T {}

#[derive(Debug)]
pub struct Geometry<PW, DW, SW, BW, CW, Cel, PI, DI, SI, BI> {
    graph: StableDiGraph<GenericNode<PW, CW>, GeometryLabel<Cel>, usize>,
    /// Number of `Removed` tombstone slots: they keep their indices
    /// occupied so a still-pending edit's reversal can restore the
    /// original occupant in place.
    removed_count: usize,
    /// When `TOPOLA_TOMBSTONE_DEBUG` is set, records the call site that
    /// tombstoned each slot so stale-edit failures can name the
    /// operation that removed the node.
    tombstone_origins: std::collections::HashMap<usize, &'static std::panic::Location<'static>>,
    /// Seg/bend nodes removed while still reversible keep their join
    /// endpoints recorded in their edits: a restoration replays the
    /// join, so the endpoint must stay alive until the joiner is
    /// either restored or permanently forgotten. Maps each referenced
    /// endpoint (dot, or a bend's `inner`) to the pending joiner
    /// nodes. Load-bearing checks like `edit_is_wrapped` consult this
    /// so a removal cannot orphan a join a pending reversal expects.
    pending_joiners: std::collections::HashMap<usize, std::collections::BTreeSet<usize>>,
    primitive_weight_marker: PhantomData<PW>,
    dot_weight_marker: PhantomData<DW>,
    seg_weight_marker: PhantomData<SW>,
    bend_weight_marker: PhantomData<BW>,
    compound_weight_marker: PhantomData<CW>,
    index_marker: PhantomData<PI>,
    dot_index_marker: PhantomData<DI>,
    seg_index_marker: PhantomData<SI>,
    bend_index_marker: PhantomData<BI>,
}

impl<PW: Clone, DW, SW, BW, CW: Clone, Cel: Clone, PI, DI, SI, BI> Clone
    for Geometry<PW, DW, SW, BW, CW, Cel, PI, DI, SI, BI>
{
    fn clone(&self) -> Self {
        Self {
            graph: self.graph.clone(),
            removed_count: self.removed_count,
            tombstone_origins: self.tombstone_origins.clone(),
            pending_joiners: self.pending_joiners.clone(),
            ..Self::new()
        }
    }
}

impl<PW, DW, SW, BW, CW, Cel: Clone, PI, DI, SI, BI> Default
    for Geometry<PW, DW, SW, BW, CW, Cel, PI, DI, SI, BI>
{
    fn default() -> Self {
        Self::new()
    }
}

impl<PW, DW, SW, BW, CW, Cel, PI, DI, SI, BI> Geometry<PW, DW, SW, BW, CW, Cel, PI, DI, SI, BI> {
    pub fn new() -> Self {
        Self {
            graph: StableDiGraph::default(),
            removed_count: 0,
            tombstone_origins: std::collections::HashMap::new(),
            pending_joiners: std::collections::HashMap::new(),
            primitive_weight_marker: PhantomData,
            dot_weight_marker: PhantomData,
            seg_weight_marker: PhantomData,
            bend_weight_marker: PhantomData,
            compound_weight_marker: PhantomData,
            index_marker: PhantomData,
            dot_index_marker: PhantomData,
            seg_index_marker: PhantomData,
            bend_index_marker: PhantomData,
        }
    }

    pub fn node_count(&self) -> usize {
        self.graph.node_count() - self.removed_count
    }

    /// Whether `index` still holds a primitive — false when the slot is
    /// vacant or was recycled by a compound node. Memoized verdicts
    /// embed primitive indices recorded under the ambient board state
    /// at the time; merges, rip-ups and bow updates can remove or
    /// replace them, so a dead index means the verdict must be
    /// re-evaluated rather than replayed.
    pub fn has_primitive(&self, index: usize) -> bool {
        matches!(
            self.graph.node_weight(index.into()),
            Some(GenericNode::Primitive(_))
        )
    }

    pub fn primitive_weight(&self, index: usize) -> PW
    where
        PW: Copy,
    {
        match self.graph.node_weight(index.into()) {
            Some(GenericNode::Primitive(weight)) => *weight,
            Some(GenericNode::Compound(_)) => panic!(
                "stale primitive index {index}: slot now holds a compound \
                 (index recycled across a mutation?)"
            ),
            Some(GenericNode::Removed) => {
                let origin = self
                    .tombstone_origins
                    .get(&index)
                    .map_or("unknown".to_string(), |loc| loc.to_string());
                panic!("stale primitive index {index}: tombstoned at {origin}")
            }
            None => panic!("stale primitive index {index}: slot is vacant"),
        }
    }
}

impl<
        PW: GetWidth + TryInto<DW> + TryInto<SW> + TryInto<BW> + Retag<Index = PI> + Copy,
        DW: AccessDotWeight + Into<PW>,
        SW: AccessSegWeight + Into<PW>,
        BW: AccessBendWeight + Into<PW>,
        CW,
        Cel,
        PI: GetIndex + TryInto<DI> + TryInto<SI> + TryInto<BI> + Copy,
        DI: GetIndex + Into<PI> + Copy,
        SI: GetIndex + Into<PI> + Copy,
        BI: GetIndex + Into<PI> + Copy,
    > Geometry<PW, DW, SW, BW, CW, Cel, PI, DI, SI, BI>
{
    /// Names what occupies a graph slot, for diagnosing edit
    /// application failures. Tombstone slots report which call site
    /// removed them when `TOPOLA_TOMBSTONE_DEBUG` is set.
    fn describe_slot(&self, index: usize) -> String {
        let occupant = self.slot_occupant(index);
        match self.tombstone_origins.get(&index) {
            Some(location) => format!("{occupant} (tombstoned at {location})"),
            None => occupant.into(),
        }
    }

    /// Names what occupies a graph slot, for diagnosing edit
    /// application failures.
    fn slot_occupant(&self, index: usize) -> &'static str {
        match self.graph.node_weight(index.into()) {
            None => "vacant",
            Some(GenericNode::Primitive(weight)) => {
                if <PW as TryInto<DW>>::try_into(*weight).is_ok() {
                    "a dot"
                } else if <PW as TryInto<SW>>::try_into(*weight).is_ok() {
                    "a seg"
                } else if <PW as TryInto<BW>>::try_into(*weight).is_ok() {
                    "a bend"
                } else {
                    "an unknown primitive"
                }
            }
            Some(GenericNode::Compound(_)) => "a compound",
            Some(GenericNode::Removed) => "a removal tombstone",
        }
    }

    /// An `*_at_index` creation write requires a vacant or tombstone
    /// slot: `update_node` on a live occupied slot swaps the weight
    /// in place while keeping the incumbent's edges, silently
    /// corrupting the graph when a stale edit index hits a recycled
    /// slot.
    fn assert_vacant_slot(&self, index: usize, what: &str) {
        let ok = matches!(
            self.graph.node_weight(index.into()),
            None | Some(GenericNode::Removed)
        );
        assert!(
            ok,
            "stale edit: creating {what} at slot {index}, which still holds {}",
            self.describe_slot(index),
        );
    }

    /// An `*_at_index` write overwriting a removal tombstone restores
    /// the slot to live use — release it from the tombstone count.
    fn unreserve_slot(&mut self, index: usize) {
        if matches!(
            self.graph.node_weight(index.into()),
            Some(GenericNode::Removed)
        ) {
            self.removed_count -= 1;
            self.tombstone_origins.remove(&index);
        }
    }

    /// Records that removed-but-restorable node `joiner` references
    /// `endpoint`: its pending reversal replays the join, so the
    /// endpoint must stay alive until the joiner is restored or its
    /// removal is forgotten.
    fn record_pending_join(&mut self, endpoint: usize, joiner: usize) {
        crate::metrics::incr(&crate::metrics::METRICS.pending_joins_recorded);
        self.pending_joiners
            .entry(endpoint)
            .or_default()
            .insert(joiner);
    }

    /// `joiner` is live again — its references are ordinary graph
    /// edges once more, not pending restorations.
    fn clear_pending_join(&mut self, endpoint: usize, joiner: usize) {
        if let Some(joiners) = self.pending_joiners.get_mut(&endpoint) {
            joiners.remove(&joiner);
            if joiners.is_empty() {
                self.pending_joiners.remove(&endpoint);
            }
        }
    }

    /// Whether `endpoint` has removed-but-reversible nodes joined to
    /// it. The graph shows no such joins — the joiners are tombstoned
    /// — but their recorded edits replay the join on restoration, so
    /// the endpoint is still load-bearing. Removing it would orphan a
    /// join a pending reversal expects. Entries whose joiner edits are
    /// permanently forgotten leak conservatively: the endpoint reads
    /// as wrapped forever, which keeps geometry alive but cannot
    /// corrupt the graph.
    pub fn has_pending_joiners(&self, endpoint: usize) -> bool {
        self.pending_joiners
            .get(&endpoint)
            .map_or(false, |joiners| !joiners.is_empty())
    }

    /// A `*_at_index` joint endpoint must hold a dot, else
    /// `update_edge` either panics on a vacant slot or joins to a
    /// node of the wrong kind, which later trips `joints()`.
    fn assert_dot_endpoint(&self, index: usize, what: &str, node: usize) {
        let ok = matches!(
            self.graph.node_weight(index.into()),
            Some(GenericNode::Primitive(weight))
                if <PW as TryInto<DW>>::try_into(*weight).is_ok()
        );
        assert!(
            ok,
            "stale edit: {what} endpoint at slot {index} holds {} instead of a dot; \
             node slot {node} holds {}",
            self.describe_slot(index),
            self.describe_slot(node),
        );
    }

    pub fn add_dot<W: AccessDotWeight + Into<PW>>(&mut self, weight: W) -> GenericIndex<W> {
        GenericIndex::<W>::new(
            self.graph
                .add_node(GenericNode::Primitive(weight.into()))
                .index(),
        )
    }

    pub(super) fn add_dot_at_index<W: AccessDotWeight + Into<PW>>(
        &mut self,
        dot: GenericIndex<W>,
        weight: W,
    ) {
        // Legitimate occupants: nothing (creation), a tombstone left
        // by a recorded removal awaiting possible restoration, or
        // another dot (in-place modify). Anything else means a stale
        // edit index is about to swap a live node's weight under its
        // edges.
        let ok = match self.graph.node_weight(dot.index().into()) {
            None | Some(GenericNode::Removed) => true,
            Some(GenericNode::Primitive(occupant)) => {
                <PW as TryInto<DW>>::try_into(*occupant).is_ok()
            }
            Some(GenericNode::Compound(_)) => false,
        };
        assert!(
            ok,
            "stale edit: writing dot at slot {}, which holds {}",
            dot.index(),
            self.describe_slot(dot.index()),
        );
        self.unreserve_slot(dot.index());
        self.graph
            .update_node(dot.index().into(), GenericNode::Primitive(weight.into()));
    }

    pub fn add_seg<W: AccessSegWeight + Into<PW>>(
        &mut self,
        from: DI,
        to: DI,
        weight: W,
    ) -> GenericIndex<W> {
        let seg = GenericIndex::<W>::new(
            self.graph
                .add_node(GenericNode::Primitive(weight.into()))
                .index(),
        );
        self.init_seg_joints(seg, from, to);
        seg
    }

    pub(super) fn add_seg_at_index<W: AccessSegWeight + Into<PW>>(
        &mut self,
        seg: GenericIndex<W>,
        from: DI,
        to: DI,
        weight: W,
    ) {
        self.assert_vacant_slot(seg.index(), "seg");
        self.unreserve_slot(seg.index());
        self.graph
            .update_node(seg.index().into(), GenericNode::Primitive(weight.into()));
        self.init_seg_joints(seg, from, to);
    }

    fn init_seg_joints<W: AccessSegWeight + Into<PW>>(
        &mut self,
        seg: GenericIndex<W>,
        from: DI,
        to: DI,
    ) {
        self.assert_dot_endpoint(from.index(), "seg from", seg.index());
        self.assert_dot_endpoint(to.index(), "seg to", seg.index());
        self.graph.update_edge(
            from.index().into(),
            seg.index().into(),
            GeometryLabel::Joined,
        );
        self.graph
            .update_edge(seg.index().into(), to.index().into(), GeometryLabel::Joined);
        // A restoration makes the joins live edges again.
        self.clear_pending_join(from.index(), seg.index());
        self.clear_pending_join(to.index(), seg.index());
    }

    pub fn is_joined_with<I>(&self, seg: I, node: GenericNode<PI, GenericIndex<CW>>) -> bool
    where
        I: Copy + GetIndex,
        CW: Clone,
        Cel: Copy,
    {
        match node {
            GenericNode::Primitive(prim) => self
                .graph
                .find_edge_undirected(seg.index().into(), prim.index().into())
                .map_or(false, |(eidx, _direction)| {
                    matches!(self.graph.edge_weight(eidx).unwrap(), GeometryLabel::Joined)
                }),
            GenericNode::Removed => false,
            GenericNode::Compound(comp) => self
                .compound_members(comp)
                .any(|(_cel, i)| self.is_joined_with(seg, GenericNode::Primitive(i))),
        }
    }

    pub fn add_bend<W: AccessBendWeight + Into<PW>>(
        &mut self,
        from: DI,
        to: DI,
        core: DI,
        weight: W,
    ) -> GenericIndex<W> {
        let bend = GenericIndex::<W>::new(
            self.graph
                .add_node(GenericNode::Primitive(weight.into()))
                .index(),
        );
        self.init_bend_joints_and_core(bend, from, to, core);
        bend
    }

    pub(super) fn add_bend_at_index<W: AccessBendWeight + Into<PW>>(
        &mut self,
        bend: GenericIndex<W>,
        from: DI,
        to: DI,
        core: DI,
        weight: W,
    ) {
        self.assert_vacant_slot(bend.index(), "bend");
        self.unreserve_slot(bend.index());
        self.graph
            .update_node(bend.index().into(), GenericNode::Primitive(weight.into()));
        self.init_bend_joints_and_core(bend, from, to, core);
    }

    pub(super) fn add_compound_at_index(&mut self, compound: GenericIndex<CW>, weight: CW) {
        self.assert_vacant_slot(compound.index(), "compound");
        self.unreserve_slot(compound.index());
        self.graph
            .update_node(compound.index().into(), GenericNode::Compound(weight));
    }

    fn init_bend_joints_and_core<W: AccessBendWeight + Into<PW>>(
        &mut self,
        bend: GenericIndex<W>,
        from: DI,
        to: DI,
        core: DI,
    ) {
        self.assert_dot_endpoint(from.index(), "bend from", bend.index());
        self.assert_dot_endpoint(to.index(), "bend to", bend.index());
        self.assert_dot_endpoint(core.index(), "bend core", bend.index());
        self.graph.update_edge(
            from.index().into(),
            bend.index().into(),
            GeometryLabel::Joined,
        );
        self.graph.update_edge(
            bend.index().into(),
            to.index().into(),
            GeometryLabel::Joined,
        );
        self.graph.update_edge(
            bend.index().into(),
            core.index().into(),
            GeometryLabel::Core,
        );
        self.clear_pending_join(from.index(), bend.index());
        self.clear_pending_join(to.index(), bend.index());
        self.clear_pending_join(core.index(), bend.index());
    }

    #[track_caller]
    pub fn remove_primitive(&mut self, primitive: PI) {
        let index = primitive.index();
        assert!(
            !matches!(
                self.graph.node_weight(index.into()),
                Some(GenericNode::Removed) | None
            ),
            "stale edit: removing already-removed slot {index} ({})",
            self.describe_slot(index),
        );
        // The removal may be reversed: record this node's join
        // endpoints as pending references so a conflicting removal of
        // an endpoint can be refused while the join can still replay.
        if let Ok(seg) = <PI as TryInto<SI>>::try_into(primitive) {
            let (from, to) = self.seg_joints(seg);
            self.record_pending_join(from.index(), index);
            self.record_pending_join(to.index(), index);
        } else if let Ok(bend) = <PI as TryInto<BI>>::try_into(primitive) {
            let (from, to) = self.bend_joints(bend);
            self.record_pending_join(from.index(), index);
            self.record_pending_join(to.index(), index);
            self.record_pending_join(self.core(bend).index(), index);
            if let Some(inner) = self.inner(bend) {
                self.record_pending_join(inner.index(), index);
            }
        }
        let maybe_removed = self.graph.remove_node(index.into());
        assert!(maybe_removed.is_some());
        // Tombstone the slot: while a recorded removal may still be
        // reversed, the index must not be recycled for a different
        // node — `*_at_index` restorations write back in place.
        self.graph.update_node(index.into(), GenericNode::Removed);
        self.removed_count += 1;
        if tombstone_debug_enabled() {
            self.tombstone_origins
                .insert(index, std::panic::Location::caller());
        }
    }

    pub fn move_dot(&mut self, dot: DI, to: Point) {
        let mut weight = self.dot_weight(dot);
        weight.set_pos(to);
        *self.graph.node_weight_mut(dot.index().into()).unwrap() =
            GenericNode::Primitive(weight.into());
    }

    pub fn shift_bend(&mut self, bend: BI, offset: f64) {
        let mut weight = self.bend_weight(bend);
        weight.set_offset(offset);
        *self.graph.node_weight_mut(bend.index().into()).unwrap() =
            GenericNode::Primitive(weight.into());
    }

    pub fn flip_bend(&mut self, bend: BI) {
        let (from, to) = self.bend_joints(bend);
        let from_edge_weight = self
            .graph
            .remove_edge(
                self.graph
                    .find_edge(from.index().into(), bend.index().into())
                    .unwrap(),
            )
            .unwrap();
        let to_edge_weight = self
            .graph
            .remove_edge(
                self.graph
                    .find_edge(bend.index().into(), to.index().into())
                    .unwrap(),
            )
            .unwrap();
        self.graph
            .update_edge(from.index().into(), bend.index().into(), to_edge_weight);
        self.graph
            .update_edge(bend.index().into(), to.index().into(), from_edge_weight);
    }

    pub fn reattach_bend(&mut self, bend: BI, maybe_new_inner: Option<BI>) {
        if let Some(old_inner_edge) = self
            .graph
            .edges_directed(bend.index().into(), Incoming)
            .find(|edge| matches!(edge.weight(), GeometryLabel::Outer))
        {
            let maybe_removed = self.graph.remove_edge(old_inner_edge.id());
            assert!(maybe_removed.is_some());
        }

        if let Some(new_inner) = maybe_new_inner {
            let ok = matches!(
                self.graph.node_weight(new_inner.index().into()),
                Some(GenericNode::Primitive(weight))
                    if <PW as TryInto<BW>>::try_into(*weight).is_ok()
            );
            assert!(
                ok,
                "stale edit: reattaching bend {} to inner slot {}, which holds {}",
                bend.index(),
                new_inner.index(),
                self.describe_slot(new_inner.index()),
            );
            self.graph.update_edge(
                new_inner.index().into(),
                bend.index().into(),
                GeometryLabel::Outer,
            );
            self.clear_pending_join(new_inner.index(), bend.index());
        }
    }

    pub fn dot_shape(&self, dot: DI) -> PrimitiveShape {
        let weight = self.dot_weight(dot);
        PrimitiveShape::Dot(DotShape {
            circle: Circle {
                pos: weight.pos(),
                r: weight.width() / 2.0,
            },
        })
    }

    pub fn seg_shape(&self, seg: SI) -> PrimitiveShape {
        let (from, to) = self.seg_joints(seg);
        PrimitiveShape::Seg(SegShape {
            from: self.dot_weight(from).pos(),
            to: self.dot_weight(to).pos(),
            width: self.primitive_weight(seg.index().into()).width(),
        })
    }

    pub fn bend_shape(&self, bend: BI) -> PrimitiveShape {
        let (from, to) = self.bend_joints(bend);
        let core_weight = self.core_weight(bend);
        PrimitiveShape::Bend(BendShape {
            from: self.dot_weight(from).pos(),
            to: self.dot_weight(to).pos(),
            inner_circle: Circle {
                pos: core_weight.pos(),
                r: self.inner_radius(bend),
            },
            width: self.primitive_weight(bend.index().into()).width(),
        })
    }

    fn inner_radius(&self, bend: BI) -> f64 {
        let mut r = self.bend_weight(bend).offset();
        let mut rail = bend;

        while let Some(inner) = self.inner(rail) {
            let weight: BW = self.bend_weight(inner);
            r += weight.width() + weight.offset();
            rail = inner;
        }

        self.core_weight(bend).width() / 2.0 + r
    }

    pub fn dot_weight(&self, dot: DI) -> DW {
        let index: usize = dot.index().into();
        self.primitive_weight(index)
            .try_into()
            .unwrap_or_else(|_| panic!("index {index} no longer holds a dot"))
    }

    pub fn seg_weight(&self, seg: SI) -> SW {
        let index: usize = seg.index().into();
        self.primitive_weight(index)
            .try_into()
            .unwrap_or_else(|_| panic!("index {index} no longer holds a seg"))
    }

    pub fn bend_weight(&self, bend: BI) -> BW {
        let index: usize = bend.index().into();
        self.primitive_weight(index)
            .try_into()
            .unwrap_or_else(|_| panic!("index {index} no longer holds a bend"))
    }

    pub fn compound_weight(&self, compound: GenericIndex<CW>) -> &CW {
        let index: usize = compound.index().into();
        match self.graph.node_weight(index.into()) {
            Some(GenericNode::Compound(weight)) => weight,
            Some(GenericNode::Primitive(_)) => {
                panic!("index {index} holds a primitive, not a compound")
            }
            Some(GenericNode::Removed) => {
                panic!("stale compound index {index}: slot is a removal tombstone")
            }
            None => panic!("stale compound index {index}: slot is vacant"),
        }
    }

    fn core_weight(&self, bend: BI) -> DW {
        let bend_index: usize = bend.index().into();
        self.graph
            .edges_directed(bend_index.into(), Outgoing)
            .find(|edge| matches!(edge.weight(), GeometryLabel::Core))
            .map(|edge| {
                self.primitive_weight(edge.target().index())
                    .try_into()
                    .unwrap_or_else(|_| panic!("core of bend {bend_index} no longer holds a dot"))
            })
            .unwrap_or_else(|| panic!("bend {bend_index} has no core edge"))
    }

    pub fn joineds(&self, node: PI) -> impl Iterator<Item = PI> + '_ {
        self.graph
            .neighbors_undirected(node.index().into())
            .filter(move |ni| {
                matches!(
                    self.graph
                        .edge_weight(
                            self.graph
                                .find_edge_undirected(node.index().into(), *ni)
                                .unwrap()
                                .0,
                        )
                        .unwrap(),
                    GeometryLabel::Joined
                )
            })
            .map(|ni| self.primitive_index(ni.index()))
    }

    pub fn joined_segs(&self, dot: DI) -> impl Iterator<Item = SI> + '_ {
        self.joineds(dot.into()).filter_map(|ni| ni.try_into().ok())
    }

    pub fn joined_bends(&self, dot: DI) -> impl Iterator<Item = BI> + '_ {
        self.joineds(dot.into()).filter_map(|ni| ni.try_into().ok())
    }

    fn joints(&self, node: PI) -> (DI, DI) {
        let lhs = self
            .graph
            .edges_directed(node.index().into(), Incoming)
            .find(|edge| matches!(edge.weight(), GeometryLabel::Joined))
            .map(|edge| edge.source());
        let rhs = self
            .graph
            .edges_directed(node.index().into(), Outgoing)
            .find(|edge| matches!(edge.weight(), GeometryLabel::Joined))
            .map(|edge| edge.target());

        (
            lhs.map(|ni| {
                self.primitive_index(ni.index())
                    .try_into()
                    .unwrap_or_else(|_| unreachable!())
            })
            .unwrap(),
            rhs.map(|ni| {
                self.primitive_index(ni.index())
                    .try_into()
                    .unwrap_or_else(|_| unreachable!())
            })
            .unwrap(),
        )
    }

    pub fn seg_joints(&self, seg: SI) -> (DI, DI) {
        self.joints(seg.into())
    }

    pub fn bend_joints(&self, bend: BI) -> (DI, DI) {
        self.joints(bend.into())
    }

    pub fn dot_index_bound(&self) -> usize {
        self.graph.node_bound()
    }

    pub fn seg_index_bound(&self) -> usize {
        self.graph.node_bound()
    }

    pub fn bend_index_bound(&self) -> usize {
        self.graph.node_bound()
    }

    pub fn compound_index_bound(&self) -> usize {
        self.graph.node_bound()
    }
}

impl<PW: Copy + Retag<Index = PI>, DW, SW, BW, CW, Cel, PI, DI, SI, BI>
    Geometry<PW, DW, SW, BW, CW, Cel, PI, DI, SI, BI>
{
    fn primitive_index(&self, index: usize) -> PI {
        self.primitive_weight(index).retag(index)
    }
}

fn tombstone_debug_enabled() -> bool {
    static ENABLED: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ENABLED.get_or_init(|| std::env::var_os("TOPOLA_TOMBSTONE_DEBUG").is_some())
}

pub struct OutwardWalker<BI> {
    frontier: VecDeque<BI>,
}

impl<BI: GetIndex> OutwardWalker<BI> {
    pub fn new(initial_frontier: impl Iterator<Item = BI>) -> Self {
        let mut frontier = VecDeque::new();
        frontier.extend(initial_frontier);

        Self { frontier }
    }
}

impl<
        PW: Copy + Retag<Index = PI>,
        DW,
        SW,
        BW,
        CW,
        Cel,
        PI: TryInto<DI> + TryInto<SI> + TryInto<BI>,
        DI,
        SI,
        BI: Copy + GetIndex,
    > Walker<&Geometry<PW, DW, SW, BW, CW, Cel, PI, DI, SI, BI>> for OutwardWalker<BI>
{
    type Item = BI;

    fn walk_next(
        &mut self,
        geometry: &Geometry<PW, DW, SW, BW, CW, Cel, PI, DI, SI, BI>,
    ) -> Option<Self::Item> {
        let front = self.frontier.pop_front()?;
        self.frontier.extend(geometry.outers(front));

        Some(front)
    }
}

impl<
        PW: Copy + Retag<Index = PI>,
        DW,
        SW,
        BW,
        CW,
        Cel,
        PI: TryInto<DI> + TryInto<SI> + TryInto<BI>,
        DI,
        SI,
        BI: GetIndex,
    > Geometry<PW, DW, SW, BW, CW, Cel, PI, DI, SI, BI>
{
    pub fn all_rails(&self, index: usize) -> impl Iterator<Item = BI> + '_ {
        self.graph
            .edges_directed(index.into(), Incoming)
            .filter(|edge| matches!(edge.weight(), GeometryLabel::Core))
            .map(|edge| {
                self.primitive_index(edge.source().index())
                    .try_into()
                    .unwrap_or_else(|_| unreachable!())
            })
    }

    pub fn core(&self, bend: BI) -> DI {
        self.graph
            .edges_directed(bend.index().into(), Outgoing)
            .find(|edge| matches!(edge.weight(), GeometryLabel::Core))
            .map(|edge| {
                self.primitive_index(edge.target().index())
                    .try_into()
                    .unwrap_or_else(|_| unreachable!())
            })
            .unwrap()
    }

    pub fn inner(&self, bend: BI) -> Option<BI> {
        self.graph
            .edges_directed(bend.index().into(), Incoming)
            .find(|edge| matches!(edge.weight(), GeometryLabel::Outer))
            .map(|edge| {
                self.primitive_index(edge.source().index())
                    .try_into()
                    .unwrap_or_else(|_| unreachable!())
            })
    }

    pub fn outers(&self, bend: BI) -> impl Iterator<Item = BI> + '_ {
        self.graph
            .edges_directed(bend.index().into(), Outgoing)
            .filter(|edge| matches!(edge.weight(), GeometryLabel::Outer))
            .map(|edge| {
                self.primitive_index(edge.target().index())
                    .try_into()
                    .unwrap_or_else(|_| unreachable!())
            })
    }

    pub fn outwards(&self, bend: BI) -> OutwardWalker<BI> {
        OutwardWalker::new(self.outers(bend))
    }
}

impl<PW: Copy + Retag<Index = PI>, DW, SW, BW, CW: Clone, Cel: Copy, PI: Copy, DI, SI, BI>
    ManageCompounds<CW> for Geometry<PW, DW, SW, BW, CW, Cel, PI, DI, SI, BI>
{
    type GeneralIndex = PI;
    type EntryLabel = Cel;

    fn add_compound(&mut self, weight: CW) -> GenericIndex<CW> {
        GenericIndex::<CW>::new(self.graph.add_node(GenericNode::Compound(weight)).index())
    }

    #[track_caller]
    fn remove_compound(&mut self, compound: GenericIndex<CW>) {
        let index = compound.index();
        assert!(
            matches!(
                self.graph.node_weight(index.into()),
                Some(GenericNode::Compound(_))
            ),
            "stale edit: removing compound at slot {index}, which holds no live compound{}",
            if tombstone_debug_enabled() {
                match self.tombstone_origins.get(&index) {
                    Some(location) => format!(" (tombstoned at {location})"),
                    None => String::new(),
                }
            } else {
                String::new()
            },
        );
        let maybe_removed = self.graph.remove_node(index.into());
        assert!(maybe_removed.is_some());
        // See `remove_primitive`: the slot is tombstoned so a pending
        // reversal can restore this compound at its recorded index.
        self.graph.update_node(index.into(), GenericNode::Removed);
        self.removed_count += 1;
        if tombstone_debug_enabled() {
            self.tombstone_origins
                .insert(index, std::panic::Location::caller());
        }
    }

    fn add_to_compound<I>(&mut self, primitive: I, entry_label: Cel, compound: GenericIndex<CW>)
    where
        I: Copy + GetIndex,
    {
        assert!(
            matches!(
                self.graph.node_weight(primitive.index().into()),
                Some(GenericNode::Primitive(_))
            ),
            "stale edit: adding compound member at slot {}, which is not a live primitive",
            primitive.index(),
        );
        self.graph.update_edge(
            primitive.index().into(),
            compound.index().into(),
            GeometryLabel::Compound(entry_label),
        );
    }

    fn compound_weight(&self, compound: GenericIndex<CW>) -> &CW {
        if let GenericNode::Compound(weight) =
            self.graph.node_weight(compound.index().into()).unwrap()
        {
            weight
        } else {
            unreachable!()
        }
    }

    fn compound_members(
        &self,
        compound: GenericIndex<CW>,
    ) -> impl Iterator<Item = (Cel, Self::GeneralIndex)> + '_ {
        self.graph
            .edges_directed(compound.index().into(), Incoming)
            .filter_map(|edge| {
                if let GeometryLabel::Compound(entry_label) = *edge.weight() {
                    Some((entry_label, self.primitive_index(edge.source().index())))
                } else {
                    None
                }
            })
    }

    fn compounds<I>(&self, node: I) -> impl Iterator<Item = (Cel, GenericIndex<CW>)>
    where
        I: Copy + GetIndex,
    {
        self.graph
            .edges_directed(node.index().into(), Outgoing)
            .filter_map(|edge| {
                if let GeometryLabel::Compound(entry_label) = *edge.weight() {
                    Some((entry_label, GenericIndex::new(edge.target().index())))
                } else {
                    None
                }
            })
    }
}
