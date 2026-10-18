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

/// A cached prenavmesh base for one `(net, layer)` pair.
struct PrenavmeshBase {
    /// Vertices for all foreign-net fixed primitives on the layer.
    vertex: PrenavmeshTriangulation,
    /// `vertex` plus every current constraint edge, without endpoint
    /// vertices. `None` while invalidated.
    constrained: Option<Prenavmesh>,
    /// The loose-seg constraint edges `constrained` was built from.
    loose_keys: BTreeSet<ConstraintKey>,
}

/// Cache of prenavmesh bases keyed by `(net, layer)`. The vertex
/// triangulation only depends on the board's fixed primitives and is
/// built once per net; the constrained base is reused while only new
/// loose-seg constraints appear, and rebuilt after rip-ups (detected as
/// stored constraint edges missing from the board — primitive indices
/// are never trusted since they are recycled).
///
/// Whole navmeshes are deliberately not cached: navnodes wrap loose
/// bends, whose indices are recycled as bands are ripped up and
/// redrawn, and the constraint fingerprint is keyed by core geometry —
/// a rerouted band over the same gears yields the same fingerprint
/// while its bend indices change, so a cached navmesh would hold stale
/// references.
#[derive(Default)]
pub struct PrenavmeshBases {
    map: HashMap<(Option<usize>, usize), PrenavmeshBase>,
}

impl PrenavmeshBases {
    /// Drops every cached base. Only needed after edits that can change
    /// fixed-primitive geometry or nets — never happens while routing.
    pub fn clear(&mut self) {
        self.map.clear();
    }

    /// Builds a navmesh for the `origin`–`destination` ratline, reusing
    /// cached vertex and constraint bases for its `(net, layer)`.
    pub fn navmesh(
        &mut self,
        layout: &Layout<impl AccessRules>,
        origin: FixedDotIndex,
        destination: FixedDotIndex,
        options: RouterOptions,
    ) -> Result<super::navmesh::Navmesh, NavmeshError> {
        let origin_primitive = origin.primitive_ref(layout.drawing());
        let (maybe_net, layer) = (origin_primitive.maybe_net(), origin_primitive.layer());
        let loose_constraints = Prenavmesh::loose_constraints(layout, maybe_net, layer);

        let prenavmesh = self.prenavmesh_with(layout, origin, destination, loose_constraints)?;
        Ok(super::navmesh::Navmesh::new_from_prenavmesh(
            layout,
            prenavmesh,
            origin,
            destination,
            options,
        )?)
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

        if !self.map.contains_key(&key) {
            let vertex = Prenavmesh::vertex_triangulation_for_net(layout, key.0, key.1)?;
            self.map.insert(
                key,
                PrenavmeshBase {
                    vertex,
                    constrained: None,
                    loose_keys: BTreeSet::new(),
                },
            );
        }
        let base = self.map.get_mut(&key).unwrap();

        // Reuse the constrained base while it only misses constraints
        // for newly committed loose segs. Rip-ups remove constraints
        // from the board, which spade cannot remove from a
        // triangulation, and a new loose seg may intersect a stored
        // fixed-seg constraint that a fresh build would have dropped —
        // both force a full rebuild.
        let mut reusable = false;
        if let Some(constrained) = base.constrained.as_mut() {
            if base
                .loose_keys
                .iter()
                .all(|key| loose_constraints.contains_key(key))
            {
                reusable = true;
                for (key, constraint) in loose_constraints.iter() {
                    if base.loose_keys.contains(key) {
                        continue;
                    }
                    if !constrained.try_add_constraint(*constraint)? {
                        reusable = false;
                        break;
                    }
                    crate::metrics::incr(
                        &crate::metrics::METRICS.prenavmesh_delta_constraint_adds,
                    );
                }
            }
        }

        if !reusable {
            crate::metrics::incr(&crate::metrics::METRICS.prenavmesh_constrained_base_builds);
            let _timer = crate::metrics::Timer::start(
                &crate::metrics::METRICS.prenavmesh_constrained_base_nanos,
            );
            base.constrained = Some(Prenavmesh::constrained_base(
                layout,
                key.0,
                key.1,
                base.vertex.clone(),
            )?);
        }
        base.loose_keys = loose_constraints.into_keys().collect();

        let mut prenavmesh = base.constrained.as_ref().unwrap().clone();
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

    use petgraph::visit::IntoNodeIdentifiers;

    use super::*;
    use crate::{
        autorouter::{ratline::RatlineUid, Autorouter},
        board::edit::BoardEdit,
        graph::MakeRef,
        specctra::design::SpecctraDesign,
    };

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
}
