// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    ops::{Index, IndexMut},
};

use derive_getters::Getters;
use enum_dispatch::enum_dispatch;
use geo::Point;
use spade::{handles::FixedVertexHandle, HasPosition, InsertionError, Point2};

use crate::{
    drawing::{
        bend::FixedBendIndex,
        dot::{DotIndex, FixedDotIndex},
        graph::{GetMaybeNet, MakePrimitiveRef, PrimitiveIndex},
        primitive::{GetCore, GetJoints, MakePrimitiveShape},
        rules::AccessRules,
        seg::{FixedSegIndex, LoneLooseSegIndex, SeqLooseSegIndex},
    },
    geometry::{shape::AccessShape, GetLayer},
    graph::{GenericIndex, GetIndex},
    layout::{CompoundEntryLabel, Layout},
    triangulation::{GetTrianvertexNodeIndex, Triangulation},
};

use super::{navmesh::NavmeshError, RouterOptions};

/// Prenavmesh nodes are the vertices of constrained Delaunay triangulation
/// before it is converted to the navmesh, which is done by multiplying each
/// of the prenavmesh nodes into more nodes, called navnodes.
#[enum_dispatch(GetIndex, MakePrimitive)]
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum PrenavnodeNodeIndex {
    FixedDot(FixedDotIndex),
    FixedBend(FixedBendIndex),
}

impl From<PrenavnodeNodeIndex> for PrimitiveIndex {
    fn from(node: PrenavnodeNodeIndex) -> Self {
        match node {
            PrenavnodeNodeIndex::FixedDot(dot) => PrimitiveIndex::FixedDot(dot),
            PrenavnodeNodeIndex::FixedBend(bend) => PrimitiveIndex::FixedBend(bend),
        }
    }
}

#[derive(Clone, Copy)]
pub struct PrenavmeshWeight {
    pub node: PrenavnodeNodeIndex,
    pub pos: Point,
}

impl GetTrianvertexNodeIndex<PrenavnodeNodeIndex> for PrenavmeshWeight {
    fn node_index(&self) -> PrenavnodeNodeIndex {
        self.node
    }
}

impl HasPosition for PrenavmeshWeight {
    type Scalar = f64;
    fn position(&self) -> Point2<Self::Scalar> {
        Point2::new(self.pos.x(), self.pos.y())
    }
}

impl PrenavmeshWeight {
    pub fn new_from_fixed_dot(layout: &Layout<impl AccessRules>, dot: FixedDotIndex) -> Self {
        Self {
            node: dot.into(),
            pos: dot.primitive_ref(layout.drawing()).shape().center(),
        }
    }

    pub fn new_from_fixed_bend(layout: &Layout<impl AccessRules>, bend: FixedBendIndex) -> Self {
        Self {
            node: bend.into(),
            pos: bend.primitive_ref(layout.drawing()).shape().center(),
        }
    }
}

/// Unordered identity of a constraint edge: both endpoint nodes and
/// their positions. Compared as a set to validate cached prenavmeshes
/// without trusting primitive indices.
pub type ConstraintKey = (
    (PrenavnodeNodeIndex, u64, u64),
    (PrenavnodeNodeIndex, u64, u64),
);

#[derive(Clone, Copy)]
pub struct PrenavmeshConstraint(pub PrenavmeshWeight, pub PrenavmeshWeight);

impl PrenavmeshConstraint {
    pub fn new_from_fixed_dot_pair(
        layout: &Layout<impl AccessRules>,
        from_dot: FixedDotIndex,
        to_dot: FixedDotIndex,
    ) -> Self {
        Self(
            PrenavmeshWeight::new_from_fixed_dot(layout, from_dot),
            PrenavmeshWeight::new_from_fixed_dot(layout, to_dot),
        )
    }

    pub fn new_from_lone_loose_seg(
        layout: &Layout<impl AccessRules>,
        seg: LoneLooseSegIndex,
    ) -> Self {
        let (from_dot, to_dot) = layout.drawing().primitive_ref(seg).joints();
        Self::new_from_fixed_dot_pair(layout, from_dot, to_dot)
    }

    pub fn new_from_seq_loose_seg(
        layout: &Layout<impl AccessRules>,
        seg: SeqLooseSegIndex,
    ) -> Self {
        let (from_joint, to_joint) = layout.drawing().primitive_ref(seg).joints();

        let from_dot = match from_joint {
            DotIndex::Fixed(dot) => dot,
            DotIndex::Loose(dot) => {
                let bend = layout.drawing().primitive_ref(dot).bend();

                layout.drawing().primitive_ref(bend).core()
            }
        };

        let to_bend = layout.drawing().primitive_ref(to_joint).bend();
        let to_dot = layout.drawing().primitive_ref(to_bend).core();
        Self::new_from_fixed_dot_pair(layout, from_dot, to_dot)
    }

    pub fn new_from_fixed_seg(layout: &Layout<impl AccessRules>, seg: FixedSegIndex) -> Self {
        let (from_dot, to_dot) = layout.drawing().primitive_ref(seg).joints();
        Self::new_from_fixed_dot_pair(layout, from_dot, to_dot)
    }

    fn key(&self) -> ConstraintKey {
        let endpoint = |weight: &PrenavmeshWeight| {
            (
                weight.node,
                weight.pos.x().to_bits(),
                weight.pos.y().to_bits(),
            )
        };
        let (a, b) = (endpoint(&self.0), endpoint(&self.1));
        if a <= b { (a, b) } else { (b, a) }
    }
}

#[derive(Clone)]
pub struct PrenavnodeToHandleMap {
    fixed_dot_to_handle: Box<[Option<FixedVertexHandle>]>,
    fixed_bend_to_handle: Box<[Option<FixedVertexHandle>]>,
}

impl PrenavnodeToHandleMap {
    pub fn new(fixed_dot_bound: usize, fixed_bend_bound: usize) -> Self {
        Self {
            fixed_dot_to_handle: vec![None; fixed_dot_bound].into_boxed_slice(),
            fixed_bend_to_handle: vec![None; fixed_bend_bound].into_boxed_slice(),
        }
    }
}

impl Index<PrenavnodeNodeIndex> for PrenavnodeToHandleMap {
    type Output = Option<FixedVertexHandle>;

    fn index(&self, prenavnode: PrenavnodeNodeIndex) -> &Self::Output {
        match prenavnode {
            PrenavnodeNodeIndex::FixedDot(dot) => &self.fixed_dot_to_handle[dot.index()],
            PrenavnodeNodeIndex::FixedBend(bend) => &self.fixed_bend_to_handle[bend.index()],
        }
    }
}

impl IndexMut<PrenavnodeNodeIndex> for PrenavnodeToHandleMap {
    fn index_mut(&mut self, prenavnode: PrenavnodeNodeIndex) -> &mut Self::Output {
        match prenavnode {
            PrenavnodeNodeIndex::FixedDot(dot) => &mut self.fixed_dot_to_handle[dot.index()],
            PrenavnodeNodeIndex::FixedBend(bend) => &mut self.fixed_bend_to_handle[bend.index()],
        }
    }
}

/// The prenavmesh's constrainted triangulation: one vertex per
/// foreign-net fixed dot or bend, plus constraint edges for bands and
/// fixed segs.
pub type PrenavmeshTriangulation =
    Triangulation<PrenavnodeNodeIndex, PrenavnodeToHandleMap, PrenavmeshWeight, ()>;

#[derive(Clone, Getters)]
pub struct Prenavmesh {
    triangulation: PrenavmeshTriangulation,
    constraints: Vec<PrenavmeshConstraint>,
}

impl Prenavmesh {
    pub fn new(
        layout: &Layout<impl AccessRules>,
        origin: FixedDotIndex,
        destination: FixedDotIndex,
        _options: RouterOptions,
    ) -> Result<Self, NavmeshError> {
        crate::metrics::incr(&crate::metrics::METRICS.prenavmesh_builds);
        let _timer = crate::metrics::Timer::start(&crate::metrics::METRICS.prenavmesh_nanos);

        let layer = layout.drawing().primitive_ref(origin).layer();
        let maybe_net = layout.drawing().primitive_ref(origin).maybe_net();

        let triangulation = Self::vertex_triangulation_for_net(layout, maybe_net, layer)?;
        Self::from_vertex_base(layout, origin, destination, triangulation)
    }

    /// Builds the prenavmesh's vertex triangulation, holding every
    /// primitive on `layer` whose net differs from `net`.
    ///
    /// The vertex set does not change while routing: bands only ever
    /// contribute constraint edges, and no fixed primitives appear or
    /// disappear. The result can therefore be cached per `(net, layer)`
    /// and cloned into `.from_vertex_base(..)`, which adds the origin
    /// and destination vertices and the current board's constraints.
    pub fn vertex_triangulation_for_net(
        layout: &Layout<impl AccessRules>,
        maybe_net: Option<usize>,
        layer: usize,
    ) -> Result<PrenavmeshTriangulation, spade::InsertionError> {
        crate::metrics::incr(&crate::metrics::METRICS.prenavmesh_base_builds);
        let _timer =
            crate::metrics::Timer::start(&crate::metrics::METRICS.prenavmesh_base_nanos);

        let mut vertices = vec![];

        for node in layout.drawing().layer_primitive_nodes(layer) {
            let primitive = node.primitive_ref(layout.drawing());

            let Some(primitive_net) = primitive.maybe_net() else {
                continue;
            };

            if Some(primitive_net) != maybe_net {
                match node {
                    PrimitiveIndex::FixedDot(dot) => {
                        layout
                            .drawing()
                            // TODO: Add `.compounds()` method working on `PrimitiveIndex`.
                            .compounds(GenericIndex::<()>::new(dot.index()))
                            .find(|(label, _)| *label == CompoundEntryLabel::Fillet)
                            .is_some();

                        // Do not add prenavnodes for primitives that have been filleted.
                        // For now, we do this by detecting if the primitive overlaps
                        // a fillet.
                        // TODO: This method is simplistic and will obviously result in
                        // false positives in some cases, so in the future, instead of this,
                        // create a fillet compound type and check for compound membership.
                        if Self::is_fixed_dot_filleted(layout, dot) {
                            continue;
                        }

                        vertices.push(PrenavmeshWeight::new_from_fixed_dot(layout, dot));
                    }
                    PrimitiveIndex::FixedBend(bend) => {
                        vertices.push(PrenavmeshWeight::new_from_fixed_bend(layout, bend));
                    }
                    _ => (),
                }
            }
        }

        Triangulation::bulk_load(
            PrenavnodeToHandleMap::new(
                layout.drawing().geometry().dot_index_bound(),
                layout.drawing().geometry().bend_index_bound(),
            ),
            vertices,
        )
    }

    /// Completes a prenavmesh from `triangulation`, typically a cached
    /// per-net vertex base, by inserting the origin and destination
    /// vertices and adding the constraint edges of the board's current
    /// seg primitives.
    pub fn from_vertex_base(
        layout: &Layout<impl AccessRules>,
        origin: FixedDotIndex,
        destination: FixedDotIndex,
        triangulation: PrenavmeshTriangulation,
    ) -> Result<Self, NavmeshError> {
        crate::metrics::incr(&crate::metrics::METRICS.prenavmesh_from_base_builds);
        let _timer =
            crate::metrics::Timer::start(&crate::metrics::METRICS.prenavmesh_from_base_nanos);

        let maybe_net = layout.drawing().primitive_ref(origin).maybe_net();
        let layer = layout.drawing().primitive_ref(origin).layer();

        let mut this = Self {
            triangulation,
            constraints: vec![],
        };
        this.add_endpoints(layout, origin, destination)?;
        this.add_constraints(layout, maybe_net, layer, Some((origin, destination)))?;
        Ok(this)
    }

    /// Builds a prenavmesh holding every constraint for `layer`/`net` but
    /// no endpoint vertices. The result depends only on the board's
    /// committed loose segs and fixed geometry, so it can be cached per
    /// `(net, layer)` and extended incrementally as more bands commit.
    pub fn constrained_base(
        layout: &Layout<impl AccessRules>,
        maybe_net: Option<usize>,
        layer: usize,
        triangulation: PrenavmeshTriangulation,
    ) -> Result<Self, NavmeshError> {
        let mut this = Self {
            triangulation,
            constraints: vec![],
        };
        this.add_constraints(layout, maybe_net, layer, None)?;
        Ok(this)
    }

    /// The loose-seg constraints on `layer` that a fresh constrained
    /// base for `maybe_net` would contain: one per loose seg whose net
    /// differs from `maybe_net`. Comparing this map's keys against a
    /// cached base's tells whether the base is still valid and, if so,
    /// which constraints are missing. Constraints are keyed by endpoint
    /// geometry, not primitive index, so the comparison is unaffected
    /// by index recycling after rip-ups.
    pub fn loose_constraints(
        layout: &Layout<impl AccessRules>,
        maybe_net: Option<usize>,
        layer: usize,
    ) -> BTreeMap<ConstraintKey, PrenavmeshConstraint> {
        layout
            .drawing()
            .layer_primitive_nodes(layer)
            .filter(|node| {
                matches!(
                    node,
                    PrimitiveIndex::LoneLooseSeg(_) | PrimitiveIndex::SeqLooseSeg(_)
                ) && node
                    .primitive_ref(layout.drawing())
                    .maybe_net()
                    .is_some_and(|net| Some(net) != maybe_net)
            })
            .map(|node| {
                let constraint = match node {
                    PrimitiveIndex::LoneLooseSeg(seg) => {
                        PrenavmeshConstraint::new_from_lone_loose_seg(layout, seg)
                    }
                    PrimitiveIndex::SeqLooseSeg(seg) => {
                        PrenavmeshConstraint::new_from_seq_loose_seg(layout, seg)
                    }
                    _ => unreachable!(),
                };
                (constraint.key(), constraint)
            })
            .collect()
    }

    /// Unordered fingerprint of `layer`'s fixed primitives: variant,
    /// node index, and position bits. Fixed geometry is supposed not
    /// to change while routing, but multilayer re-layering places and
    /// removes fanout vias and stub segs mid-run — and freed indices
    /// are recycled, so this is keyed by index plus position rather
    /// than index bounds.
    pub fn fixed_fingerprint(layout: &Layout<impl AccessRules>, layer: usize) -> u64 {
        use std::hash::{Hash, Hasher};

        let (count, sum) = layout
            .drawing()
            .layer_primitive_nodes(layer)
            .filter_map(|node| {
                let tag = match node {
                    PrimitiveIndex::FixedDot(_) => 0u8,
                    PrimitiveIndex::FixedBend(_) => 1,
                    PrimitiveIndex::FixedSeg(_) => 2,
                    _ => return None,
                };
                let primitive = node.primitive_ref(layout.drawing());
                let pos = primitive.shape().center();
                // The net is part of the identity: a via swapped for
                // another net's via at the same index and position is a
                // different primitive as the navmesh sees it.
                Some((
                    tag,
                    node.index(),
                    pos.x().to_bits(),
                    pos.y().to_bits(),
                    primitive.maybe_net(),
                ))
            })
            // Order-independent fold: the rtree yields nodes in
            // traversal order, which shifts as loose primitives come
            // and go, so only commutative accumulators keep the
            // fingerprint stable while the fixed set is unchanged.
            .fold((0u64, 0x9e3779b97f4a7c15u64), |(count, acc), entry| {
                let mut hasher = std::collections::hash_map::DefaultHasher::new();
                entry.hash(&mut hasher);
                (count + 1, acc.wrapping_add(hasher.finish()))
            });
        count.rotate_left(32) ^ sum
    }

    /// Inserts the origin and destination vertices into the prenavmesh.
    /// Applied to a cloned base for every route.
    pub fn add_endpoints(
        &mut self,
        layout: &Layout<impl AccessRules>,
        origin: FixedDotIndex,
        destination: FixedDotIndex,
    ) -> Result<(), InsertionError> {
        self.triangulation
            .add_vertex(PrenavmeshWeight::new_from_fixed_dot(layout, origin))?;
        self.triangulation.add_vertex(PrenavmeshWeight::new_from_fixed_dot(
            layout,
            destination,
        ))?;
        Ok(())
    }

    /// Adds one constraint to an already constrained base. Returns
    /// `Ok(false)` if the new constraint would intersect a stored one: a
    /// fresh build inserts loose constraints before fixed ones and drops
    /// the conflicting fixed seg instead, which cannot be undone
    /// incrementally, so the caller must rebuild the base then.
    pub fn try_add_constraint(
        &mut self,
        constraint: PrenavmeshConstraint,
    ) -> Result<bool, InsertionError> {
        if self
            .triangulation
            .intersects_constraint(&constraint.0, &constraint.1)
        {
            return Ok(false);
        }

        self.add_constraint(constraint)?;
        Ok(true)
    }

    fn add_constraints(
        &mut self,
        layout: &Layout<impl AccessRules>,
        maybe_net: Option<usize>,
        layer: usize,
        endpoints: Option<(FixedDotIndex, FixedDotIndex)>,
    ) -> Result<(), NavmeshError> {
        for node in layout.drawing().layer_primitive_nodes(layer) {
            let primitive = node.primitive_ref(layout.drawing());

            let Some(primitive_net) = primitive.maybe_net() else {
                if let Some((origin, destination)) = endpoints {
                    assert_ne!(node, origin.into());
                    assert_ne!(node, destination.into());
                }
                continue;
            };

            if endpoints.is_some_and(|(origin, destination)| {
                node == origin.into() || node == destination.into()
            }) || Some(primitive_net) != maybe_net
            {
                match node {
                    PrimitiveIndex::LoneLooseSeg(seg) => {
                        self.add_constraint(PrenavmeshConstraint::new_from_lone_loose_seg(
                            layout, seg,
                        ))?;
                    }
                    PrimitiveIndex::SeqLooseSeg(seg) => {
                        self.add_constraint(PrenavmeshConstraint::new_from_seq_loose_seg(
                            layout, seg,
                        ))?;
                    }
                    _ => (),
                }
            }
        }

        for node in layout.drawing().layer_primitive_nodes(layer) {
            let primitive = node.primitive_ref(layout.drawing());

            let Some(primitive_net) = primitive.maybe_net() else {
                continue;
            };

            if endpoints.is_some_and(|(origin, destination)| {
                node == origin.into() || node == destination.into()
            }) || Some(primitive_net) != maybe_net
            {
                // If you have a band that was routed from a polygonal pad,
                // when you will start a new routing some of the constraint
                // edges created from the loose segs of a band will
                // intersect some of the constraint edges created from the
                // fixed segs constituting the pad boundary.
                //
                // Such constraint intersections are erroneous and cause
                // Spade to throw a panic at runtime. So, to prevent this
                // from occuring, we iterate over the layout for the second
                // time, after all the constraint edges from bands have been
                // placed, and only then add constraint edges created from
                // fixed segs that do not cause an intersection.
                match node {
                    PrimitiveIndex::FixedSeg(seg) => {
                        let (from_dot, to_dot) = layout.drawing().primitive_ref(seg).joints();

                        if Self::is_fixed_dot_filleted(layout, from_dot)
                            && Self::is_fixed_dot_filleted(layout, to_dot)
                        {
                            continue;
                        }

                        let constraint = PrenavmeshConstraint::new_from_fixed_seg(layout, seg);

                        if !self
                            .triangulation
                            .intersects_constraint(&constraint.0, &constraint.1)
                        {
                            self.add_constraint(constraint).ok();
                        }
                    }
                    _ => (),
                }
            }
        }

        Ok(())
    }

    fn is_fixed_dot_filleted(layout: &Layout<impl AccessRules>, dot: FixedDotIndex) -> bool {
        layout
            .drawing()
            .compounds(GenericIndex::<()>::new(dot.index()))
            .find(|(label, _)|
            // Fillets fail this test for some reason that I did not investigate, so
            // I added this condition.
                *label == CompoundEntryLabel::Fillet
            // Exclude apices because they may overlap fillets.
                || *label == CompoundEntryLabel::Apex)
            .is_none()
            && layout
                .drawing()
                .overlapees(dot.into())
                .find(|overlapee| {
                    layout
                        .drawing()
                        // TODO: Add `.compounds()` method working on `PrimitiveIndex`.
                        .compounds(GenericIndex::<()>::new(overlapee.2.index()))
                        .find(|(label, _)| *label == CompoundEntryLabel::Fillet)
                        .is_some()
                })
                .is_some()
    }

    fn add_constraint(&mut self, constraint: PrenavmeshConstraint) -> Result<(), InsertionError> {
        self.triangulation
            .add_constraint_edge(constraint.0, constraint.1)?;
        self.constraints.push(constraint);
        Ok(())
    }
}

/// Exact identity of a loose primitive as the navmesh sees it: the map
/// key carries its (possibly recycled) index, and the signature carries
/// everything its references dereference — the weight plus joint and
/// inner-bend linkage. Geometry is implied by the linked dots, which
/// have entries of their own. Edits applied or reverted through
/// `.apply(..)` restore indices and weights exactly, so an equal
/// snapshot proves a cached navmesh's loose-primitive references are
/// still valid — unlike a geometry-only fingerprint, which a rerouted
/// band over the same gears could collide with while its bend indices
/// differ.
#[derive(Debug, PartialEq)]
enum LooseSig {
    Dot(crate::drawing::graph::PrimitiveWeight),
    Seg(crate::drawing::graph::PrimitiveWeight, usize, usize),
    Bend(crate::drawing::graph::PrimitiveWeight, usize, usize, Option<usize>),
}

type LooseSnapshot = BTreeMap<PrimitiveIndex, LooseSig>;

/// Snapshots every loose primitive on `layer`: bands of any net can be
/// wrapped by the navmesh's navnodes, so the snapshot cannot be
/// restricted to one net.
fn loose_snapshot(layout: &Layout<impl AccessRules>, layer: usize) -> LooseSnapshot {
    let drawing = layout.drawing();
    let geometry = drawing.geometry();
    layout
        .drawing()
        .layer_primitive_nodes(layer)
        .filter_map(|node| {
            let sig = match node {
                PrimitiveIndex::LooseDot(_) => {
                    LooseSig::Dot(geometry.primitive_weight(node.index()))
                }
                PrimitiveIndex::LoneLooseSeg(seg) => {
                    let (from, to) = geometry.seg_joints(seg.into());
                    LooseSig::Seg(
                        geometry.primitive_weight(node.index()),
                        from.index(),
                        to.index(),
                    )
                }
                PrimitiveIndex::SeqLooseSeg(seg) => {
                    let (from, to) = geometry.seg_joints(seg.into());
                    LooseSig::Seg(
                        geometry.primitive_weight(node.index()),
                        from.index(),
                        to.index(),
                    )
                }
                PrimitiveIndex::LooseBend(bend) => {
                    let bend_index: crate::drawing::bend::BendIndex = bend.into();
                    let (from, to) = geometry.bend_joints(bend_index);
                    LooseSig::Bend(
                        geometry.primitive_weight(node.index()),
                        from.index(),
                        to.index(),
                        geometry.inner(bend_index).map(|inner| inner.index()),
                    )
                }
                _ => return None,
            };
            Some((node, sig))
        })
        .collect()
}

/// A cached prenavmesh base for one `(net, layer)` pair.
struct PrenavmeshBase {
    /// Vertices for all foreign-net fixed primitives on the layer.
    vertex: PrenavmeshTriangulation,
    /// Fingerprint of the layer's fixed primitives the vertex base was
    /// built from. Fanout vias placed or removed during multilayer
    /// re-layering — and index recycling — change the fingerprint,
    /// forcing a rebuild.
    fingerprint: u64,
    /// Constrained bases — `vertex` plus constraint edges, no endpoint
    /// vertices — each keyed by the exact loose-seg constraint set it
    /// was built from, most-recently-used last. Revisited loose states
    /// (e.g. restored by a backtrace, which re-applies edits with
    /// identical geometry) hit by exact set equality; states that only
    /// add constraints delta-extend a subset entry.
    constrained: Vec<(BTreeSet<ConstraintKey>, Prenavmesh)>,
}

/// A fully built navmesh kept for reuse while the board state it was
/// built against can still be proven identical.
struct CachedNavmesh {
    /// `(net, layer, origin, destination, fixed fingerprint, options)`.
    /// The options are folded in because they affect navnode
    /// generation (`squeeze_through_under_bends` adds navnodes).
    key: (
        Option<usize>,
        usize,
        FixedDotIndex,
        FixedDotIndex,
        u64,
        u64,
        bool,
        bool,
    ),
    /// Every loose primitive's identity at build time. Reuse requires
    /// exact equality — index, weight and linkage alike — which undo
    /// restores bit-for-bit, so backtraces hit legitimately.
    snapshot: LooseSnapshot,
    navmesh: super::navmesh::Navmesh,
}

/// Cache of prenavmesh bases keyed by `(net, layer)`. The vertex
/// triangulation only depends on the board's fixed primitives and is
/// built once per net; constrained bases and whole navmeshes are kept
/// in small MRU lists and reused whenever their exact board-state keys
/// still match.
#[derive(Default)]
pub struct PrenavmeshBases {
    map: HashMap<(Option<usize>, usize), PrenavmeshBase>,
    /// Reusable whole navmeshes, MRU last. Bounded because each navmesh
    /// carries a whole search graph.
    navmeshes: Vec<CachedNavmesh>,
}

/// Bounds the number of constrained bases kept per `(net, layer)`.
const CONSTRAINED_CACHE_CAP: usize = 16;

/// Bounds the number of whole navmeshes kept globally; each carries a
/// search graph proportional to the board.
const NAVMESH_CACHE_CAP: usize = 32;

impl PrenavmeshBases {
    /// Drops every cached base. Only needed after edits that can change
    /// fixed-primitive geometry or nets — never happens while routing.
    pub fn clear(&mut self) {
        self.map.clear();
        self.navmeshes.clear();
    }

    /// Builds a navmesh for the `origin`–`destination` ratline, reusing
    /// a whole cached navmesh when the layer's loose-primitive state is
    /// provably identical, and falling back to cached vertex and
    /// constraint bases otherwise.
    pub fn navmesh(
        &mut self,
        layout: &Layout<impl AccessRules>,
        origin: FixedDotIndex,
        destination: FixedDotIndex,
        options: RouterOptions,
    ) -> Result<super::navmesh::Navmesh, NavmeshError> {
        let origin_primitive = origin.primitive_ref(layout.drawing());
        let (maybe_net, layer) = (origin_primitive.maybe_net(), origin_primitive.layer());
        let snapshot = loose_snapshot(layout, layer);
        let key = (
            maybe_net,
            layer,
            origin,
            destination,
            Prenavmesh::fixed_fingerprint(layout, layer),
            options.routed_band_width.to_bits(),
            options.wrap_around_bands,
            options.squeeze_through_under_bends,
        );

        if let Some(pos) = self
            .navmeshes
            .iter()
            .position(|cached| cached.key == key && cached.snapshot == snapshot)
        {
            crate::metrics::incr(&crate::metrics::METRICS.navmesh_cache_hits);
            let cached = self.navmeshes.remove(pos);
            let navmesh = cached.navmesh.clone();
            self.navmeshes.push(cached);
            return Ok(navmesh);
        }

        let loose_constraints = Prenavmesh::loose_constraints(layout, maybe_net, layer);
        let prenavmesh =
            self.prenavmesh_with(layout, origin, destination, loose_constraints)?;
        let navmesh = super::navmesh::Navmesh::new_from_prenavmesh(
            layout,
            prenavmesh,
            origin,
            destination,
            options,
        )?;

        if self.navmeshes.len() >= NAVMESH_CACHE_CAP {
            self.navmeshes.remove(0);
        }
        self.navmeshes.push(CachedNavmesh {
            key,
            snapshot,
            navmesh: navmesh.clone(),
        });
        Ok(navmesh)
    }

    /// Builds a prenavmesh for the `origin`–`destination` ratline,
    /// reusing cached bases for its `(net, layer)` where possible.
    pub fn prenavmesh(
        &mut self,
        layout: &Layout<impl AccessRules>,
        origin: FixedDotIndex,
        destination: FixedDotIndex,
    ) -> Result<Prenavmesh, NavmeshError> {
        let origin_primitive = origin.primitive_ref(layout.drawing());
        let (maybe_net, layer) = (origin_primitive.maybe_net(), origin_primitive.layer());
        let loose_constraints = Prenavmesh::loose_constraints(layout, maybe_net, layer);
        self.prenavmesh_with(layout, origin, destination, loose_constraints)
    }

    fn prenavmesh_with(
        &mut self,
        layout: &Layout<impl AccessRules>,
        origin: FixedDotIndex,
        destination: FixedDotIndex,
        loose_constraints: BTreeMap<ConstraintKey, PrenavmeshConstraint>,
    ) -> Result<Prenavmesh, NavmeshError> {
        let origin_primitive = origin.primitive_ref(layout.drawing());
        let key = (origin_primitive.maybe_net(), origin_primitive.layer());
        let fingerprint = Prenavmesh::fixed_fingerprint(layout, key.1);

        if !self.map.contains_key(&key) || self.map[&key].fingerprint != fingerprint {
            // Build or rebuild the vertex base: the fingerprint only
            // changes when fixed primitives appear, disappear, or move.
            let vertex = Prenavmesh::vertex_triangulation_for_net(layout, key.0, key.1)?;
            self.map.insert(
                key,
                PrenavmeshBase {
                    vertex,
                    fingerprint,
                    constrained: vec![],
                },
            );
        }
        let base = self.map.get_mut(&key).unwrap();

        let loose_keys: BTreeSet<ConstraintKey> =
            loose_constraints.keys().copied().collect();

        // An entry for exactly this loose state — typically restored by
        // a backtrace — can be reused outright.
        if let Some(pos) = base
            .constrained
            .iter()
            .position(|(keys, _)| *keys == loose_keys)
        {
            crate::metrics::incr(
                &crate::metrics::METRICS.prenavmesh_constrained_base_hits,
            );
            let entry = base.constrained.remove(pos);
            let mut prenavmesh = entry.1.clone();
            base.constrained.push(entry);
            prenavmesh.add_endpoints(layout, origin, destination)?;
            return Ok(prenavmesh);
        }

        // Otherwise delta-extend the largest subset entry: its stored
        // constraints are all still present and only new ones need
        // adding. Rip-ups remove constraints, which spade cannot remove
        // from a triangulation, and a new loose seg may intersect a
        // stored fixed-seg constraint that a fresh build would have
        // dropped — both force a full rebuild.
        let mut constrained = None;
        if let Some((pos, subset_keys)) = base
            .constrained
            .iter()
            .enumerate()
            .filter(|(_, (keys, _))| {
                keys.iter().all(|key| loose_constraints.contains_key(key))
            })
            .max_by_key(|(_, (keys, _))| keys.len())
            .map(|(pos, (keys, _))| (pos, keys.clone()))
        {
            let mut candidate = base.constrained[pos].1.clone();
            let mut ok = true;
            for (key, constraint) in loose_constraints.iter() {
                if subset_keys.contains(key) {
                    continue;
                }
                if !candidate.try_add_constraint(*constraint)? {
                    ok = false;
                    break;
                }
                crate::metrics::incr(
                    &crate::metrics::METRICS.prenavmesh_delta_constraint_adds,
                );
            }
            if ok {
                constrained = Some(candidate);
            }
        }

        let constrained = match constrained {
            Some(constrained) => constrained,
            None => {
                crate::metrics::incr(
                    &crate::metrics::METRICS.prenavmesh_constrained_base_builds,
                );
                let _timer = crate::metrics::Timer::start(
                    &crate::metrics::METRICS.prenavmesh_constrained_base_nanos,
                );
                Prenavmesh::constrained_base(layout, key.0, key.1, base.vertex.clone())?
            }
        };

        if base.constrained.len() >= CONSTRAINED_CACHE_CAP {
            base.constrained.remove(0);
        }
        base.constrained.push((loose_keys, constrained.clone()));

        let mut prenavmesh = constrained;
        prenavmesh.add_endpoints(layout, origin, destination)?;
        Ok(prenavmesh)
    }
}

#[cfg(test)]
mod tests {
    use std::{
        collections::BTreeSet,
        fs::File,
        io::BufReader,
    };

    use geo::point;
    use petgraph::visit::IntoNodeIdentifiers;

    use super::*;
    use crate::{
        autorouter::{ratline::RatlineUid, Autorouter},
        board::edit::BoardEdit,
        drawing::{
            dot::{FixedDotWeight, GeneralDotWeight},
            seg::{GeneralSegWeight, LoneLooseSegWeight},
        },
        geometry::edit::{ApplyGeometryEdit, Edit},
        graph::MakeRef,
        layout::LayoutEdit,
        math::Circle,
        specctra::{design::SpecctraDesign, mesadata::SpecctraMesadata},
    };

    fn autorouter_0603() -> Autorouter<SpecctraMesadata> {
        let design_file =
            File::open("tests/unilayer/0603_breakout/0603_breakout.dsn").unwrap();
        let design = SpecctraDesign::load(BufReader::new(design_file)).unwrap();
        Autorouter::new(design.make_board(&mut BoardEdit::new())).unwrap()
    }

    fn first_ratline(autorouter: &Autorouter<SpecctraMesadata>) -> RatlineUid {
        autorouter
            .ratsnests()
            .on_principal_layer(0)
            .graph()
            .edge_indices()
            .map(|index| RatlineUid {
                principal_layer: 0,
                index,
            })
            .next()
            .unwrap()
    }

    /// A prenavmesh built from a cached per-net vertex base must equal
    /// one built from scratch: same vertex set, same constraint set.
    #[test]
    fn vertex_base_matches_fresh_prenavmesh() {
        let design_file =
            File::open("tests/unilayer/0603_breakout/0603_breakout.dsn").unwrap();
        let design = SpecctraDesign::load(BufReader::new(design_file)).unwrap();
        let autorouter = Autorouter::new(design.make_board(&mut BoardEdit::new())).unwrap();
        let layout = autorouter.board().layout();

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

        let options = RouterOptions {
            wrap_around_bands: true,
            squeeze_through_under_bends: false,
            routed_band_width: 100.0,
        };
        let fresh = Prenavmesh::new(layout, origin, destination, options).unwrap();

        let origin_primitive = origin.primitive_ref(layout.drawing());
        let base = Prenavmesh::vertex_triangulation_for_net(
            layout,
            origin_primitive.maybe_net(),
            origin_primitive.layer(),
        )
        .unwrap();
        let from_base =
            Prenavmesh::from_vertex_base(layout, origin, destination, base).unwrap();

        let nodes = |prenavmesh: &Prenavmesh| {
            prenavmesh
                .triangulation()
                .node_identifiers()
                .collect::<BTreeSet<_>>()
        };
        assert_eq!(nodes(&from_base), nodes(&fresh));

        let constraints = |prenavmesh: &Prenavmesh| {
            prenavmesh
                .constraints()
                .iter()
                .map(|constraint| (constraint.0.node, constraint.1.node))
                .collect::<BTreeSet<_>>()
        };
        assert_eq!(constraints(&from_base), constraints(&fresh));
    }

    /// The fixed-primitive fingerprint keys the vertex bases — it must
    /// stay stable while loose primitives come and go. Loose
    /// insertions shift the rtree traversal order, which is why the
    /// fold is order-independent; a regression here rebuilds the
    /// vertex bases on every call.
    #[test]
    fn fixed_fingerprint_ignores_loose_prims() {
        let mut autorouter = autorouter_0603();
        let layout = autorouter.board_mut().layout_mut();

        let mut dots_edit = LayoutEdit::new();
        let dot = |x: f64| {
            FixedDotWeight(GeneralDotWeight {
                circle: Circle {
                    pos: point!(x: x, y: -8000.0),
                    r: 100.0,
                },
                layer: 0,
                maybe_net: Some(90),
            })
        };
        let a = layout.add_fixed_dot_infringably(&mut dots_edit, dot(0.0));
        let b = layout.add_fixed_dot_infringably(&mut dots_edit, dot(500.0));

        let fingerprint = Prenavmesh::fixed_fingerprint(layout, 0);

        let mut seg_edit = LayoutEdit::new();
        layout
            .add_lone_loose_seg(
                &mut seg_edit,
                a,
                b,
                LoneLooseSegWeight(GeneralSegWeight {
                    width: 100.0,
                    layer: 0,
                    maybe_net: Some(90),
                }),
            )
            .unwrap();
        assert_eq!(Prenavmesh::fixed_fingerprint(layout, 0), fingerprint);

        layout.apply(&seg_edit.reverse());
        assert_eq!(Prenavmesh::fixed_fingerprint(layout, 0), fingerprint);
    }

    /// The fingerprint must notice a via-style swap: a fixed primitive
    /// removed and another placed at the same index and position but
    /// on a different net. Navnode net classification is baked into a
    /// cached navmesh at build time, so a net change must invalidate.
    #[test]
    fn fixed_fingerprint_distinguishes_net_at_same_index() {
        let mut autorouter = autorouter_0603();
        let layout = autorouter.board_mut().layout_mut();

        let mut edit = LayoutEdit::new();
        let dot = layout.add_fixed_dot_infringably(
            &mut edit,
            FixedDotWeight(GeneralDotWeight {
                circle: Circle {
                    pos: point!(x: 0.0, y: -8000.0),
                    r: 100.0,
                },
                layer: 0,
                maybe_net: Some(90),
            }),
        );
        let fingerprint = Prenavmesh::fixed_fingerprint(layout, 0);

        layout.apply(&edit.reverse());
        let swapped = layout.add_fixed_dot_infringably(
            &mut LayoutEdit::new(),
            FixedDotWeight(GeneralDotWeight {
                circle: Circle {
                    pos: point!(x: 0.0, y: -8000.0),
                    r: 100.0,
                },
                layer: 0,
                maybe_net: Some(91),
            }),
        );
        assert_eq!(swapped, dot);
        assert_ne!(Prenavmesh::fixed_fingerprint(layout, 0), fingerprint);
    }

    /// The loose-primitive snapshot gates whole-navmesh reuse — it
    /// must capture insertions exactly and be restored bit-for-bit by
    /// `apply(reverse)`, which is what makes backtrace hits sound.
    #[test]
    fn loose_snapshot_round_trips_through_undo() {
        let mut autorouter = autorouter_0603();
        let layout = autorouter.board_mut().layout_mut();

        let mut dots_edit = LayoutEdit::new();
        let dot = |x: f64| {
            FixedDotWeight(GeneralDotWeight {
                circle: Circle {
                    pos: point!(x: x, y: -8000.0),
                    r: 100.0,
                },
                layer: 0,
                maybe_net: Some(90),
            })
        };
        let a = layout.add_fixed_dot_infringably(&mut dots_edit, dot(0.0));
        let b = layout.add_fixed_dot_infringably(&mut dots_edit, dot(500.0));

        let snapshot = loose_snapshot(layout, 0);

        let mut seg_edit = LayoutEdit::new();
        layout
            .add_lone_loose_seg(
                &mut seg_edit,
                a,
                b,
                LoneLooseSegWeight(GeneralSegWeight {
                    width: 100.0,
                    layer: 0,
                    maybe_net: Some(90),
                }),
            )
            .unwrap();
        assert_ne!(loose_snapshot(layout, 0), snapshot);

        layout.apply(&seg_edit.reverse());
        assert_eq!(loose_snapshot(layout, 0), snapshot);
    }

    /// After loose geometry is added and then undone, a navmesh
    /// rebuild through the cache must yield an identical structure —
    /// equal snapshots make the cached entry hit-eligible, and an
    /// exact rebuild yields equal content either way.
    #[test]
    fn navmesh_cache_restores_identical_navmesh_after_undo() {
        let mut autorouter = autorouter_0603();
        let ratline = first_ratline(&autorouter);
        let (origin, destination) = ratline.ref_(&autorouter).endpoint_dots();
        let options = RouterOptions {
            wrap_around_bands: true,
            squeeze_through_under_bends: false,
            routed_band_width: 100.0,
        };

        // Fixed dots the loose seg will span; they persist for the
        // whole test so the first and final navmeshes see the same
        // fixed geometry.
        let layout = autorouter.board_mut().layout_mut();
        let mut dots_edit = LayoutEdit::new();
        let dot = |x: f64| {
            FixedDotWeight(GeneralDotWeight {
                circle: Circle {
                    pos: point!(x: x, y: -8000.0),
                    r: 100.0,
                },
                layer: 0,
                maybe_net: Some(90),
            })
        };
        let a = layout.add_fixed_dot_infringably(&mut dots_edit, dot(0.0));
        let b = layout.add_fixed_dot_infringably(&mut dots_edit, dot(500.0));

        let mut bases = PrenavmeshBases::default();
        let first = bases.navmesh(layout, origin, destination, options).unwrap();
        let (nodes, edges) = (first.graph().node_count(), first.graph().edge_count());

        // Mutate loose state, forcing a rebuild against different
        // geometry.
        let mut seg_edit = LayoutEdit::new();
        layout
            .add_lone_loose_seg(
                &mut seg_edit,
                a,
                b,
                LoneLooseSegWeight(GeneralSegWeight {
                    width: 100.0,
                    layer: 0,
                    maybe_net: Some(90),
                }),
            )
            .unwrap();
        bases
            .navmesh(layout, origin, destination, options)
            .unwrap();

        layout.apply(&seg_edit.reverse());
        let third = bases.navmesh(layout, origin, destination, options).unwrap();
        assert_eq!(third.graph().node_count(), nodes);
        assert_eq!(third.graph().edge_count(), edges);
    }
}
