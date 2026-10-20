// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::ops::IndexMut;
use std::{cmp::Ordering, marker::PhantomData};

use geo::algorithm::line_measures::{Distance, Euclidean};
use geo::{point, Point};
use petgraph::visit;
use spade::{
    handles::FixedVertexHandle, ConstrainedDelaunayTriangulation, HasPosition, InsertionError,
};

use crate::graph::GetIndex;

pub trait GetTrianvertexNodeIndex<I> {
    fn node_index(&self) -> I;
}

// No `Debug` derive because `spade::ConstrainedDelaunayTriangulation` does
// not implement `Debug`, though it could (and `spade::DelaunayTriangulation`
// actually does).
#[derive(Clone)]
pub struct Triangulation<I, M, VW: GetTrianvertexNodeIndex<I> + HasPosition, EW: Default> {
    cdt: ConstrainedDelaunayTriangulation<VW, EW>,
    trianvertex_to_handle: M,
    index_marker: PhantomData<I>,
}

impl<
        I: GetIndex,
        M: IndexMut<I, Output = Option<FixedVertexHandle>>,
        VW: GetTrianvertexNodeIndex<I> + HasPosition,
        EW: Default,
    > Triangulation<I, M, VW, EW>
{
    pub fn new(trianvertex_to_handle: M) -> Self {
        Self {
            cdt: <ConstrainedDelaunayTriangulation<VW, EW> as spade::Triangulation>::new(),
            trianvertex_to_handle,
            index_marker: PhantomData,
        }
    }

    /// Builds the triangulation in one pass from `vertices` instead of
    /// inserting them one by one. Much faster for large vertex counts,
    /// since a sequential `.add_vertex(..)` pays a location query per
    /// vertex.
    ///
    /// Bulk loading deduplicates same-position vertices just like
    /// sequential insertion does: node indices whose vertices were
    /// deduplicated are resolved to the shared vertex afterwards.
    pub fn bulk_load(trianvertex_to_handle: M, vertices: Vec<VW>) -> Result<Self, InsertionError>
    where
        VW: Clone,
    {
        let cdt = ConstrainedDelaunayTriangulation::bulk_load_cdt(vertices.clone(), vec![])?;
        let mut this = Self {
            cdt,
            trianvertex_to_handle,
            index_marker: PhantomData,
        };

        for handle in spade::Triangulation::fixed_vertices(&this.cdt) {
            let index = spade::Triangulation::s(&this.cdt)
                .vertex_data(handle)
                .node_index();
            this.trianvertex_to_handle[index] = Some(handle);
        }

        for weight in vertices {
            let slot = &mut this.trianvertex_to_handle[weight.node_index()];
            if slot.is_none() {
                *slot = spade::Triangulation::locate_vertex(&this.cdt, weight.position())
                    .map(|handle| handle.fix());
            }
        }

        Ok(this)
    }

    pub fn add_vertex(&mut self, weight: VW) -> Result<(), InsertionError> {
        let index = weight.node_index();
        self.trianvertex_to_handle[index] =
            Some(spade::Triangulation::insert(&mut self.cdt, weight)?);
        Ok(())
    }

    pub fn add_constraint_edge(&mut self, from: VW, to: VW) -> Result<bool, InsertionError> {
        let from_index = from.node_index();
        let to_index = to.node_index();

        // It is possible for one or both constraint edge endpoint vertices to
        // not exist in the triangulation even after everything has been added.
        // This can happen if the constraint was formed from a band wrapped
        // over a polygonal pad that is the routing origin or destination, since
        // in such situation the vertices of the pad boundary are not added to
        // the triangulation.
        //
        // To prevent this from causing a panic at runtime, we idempotently add
        // the constraint edge endpoint vertices to triangulation before adding
        // the edge itself.
        self.add_vertex(from)?;
        self.add_vertex(to)?;
        Ok(!self
            .cdt
            .try_add_constraint(
                self.trianvertex_to_handle[from_index].unwrap(),
                self.trianvertex_to_handle[to_index].unwrap(),
            )
            .is_empty())
    }

    pub fn exists_constraint(&self, from: I, to: I) -> bool {
        self.cdt.exists_constraint(
            self.trianvertex_to_handle[from].unwrap(),
            self.trianvertex_to_handle[to].unwrap(),
        )
    }

    pub fn intersects_constraint(&self, from: &VW, to: &VW) -> bool {
        // Prefer the handle-based intersection iterator: it walks
        // incident edges directly and never calls `locate`, whose
        // position lookup can walk forever on some triangulation
        // states (lvds2tmds panic — spade's
        // `locate_with_hint_option_core` "Failed to locate position").
        // Handles are absent only for endpoint vertices not yet
        // inserted (e.g. same-net pad-boundary vertices); those fall
        // back to the position-based check.
        if let (Some(from_handle), Some(to_handle)) = (
            self.trianvertex_to_handle[from.node_index()],
            self.trianvertex_to_handle[to.node_index()],
        ) {
            return !self.cdt.can_add_constraint(from_handle, to_handle);
        }

        self.cdt
            .intersects_constraint(from.position(), to.position())
    }

    pub fn find_vertex_at_position(&self, position: Point<<VW as HasPosition>::Scalar>) -> Option<I>
    where
        <VW as HasPosition>::Scalar: geo::CoordNum,
    {
        spade::Triangulation::locate_vertex(
            &self.cdt,
            spade::Point2::new(position.x(), position.y()),
        )
        .map(|handle| self.vertex(handle.fix()))
    }

    pub fn weight(&self, vertex: I) -> &VW {
        spade::Triangulation::s(&self.cdt).vertex_data(self.trianvertex_to_handle[vertex].unwrap())
    }

    pub fn weight_mut(&mut self, vertex: I) -> &mut VW {
        spade::Triangulation::vertex_data_mut(
            &mut self.cdt,
            self.trianvertex_to_handle[vertex].unwrap(),
        )
    }

    fn vertex(&self, handle: FixedVertexHandle) -> I {
        spade::Triangulation::vertex(&self.cdt, handle)
            .as_ref()
            .node_index()
    }

    fn handle(&self, vertex: I) -> FixedVertexHandle {
        self.trianvertex_to_handle[vertex].unwrap()
    }

    pub fn position(&self, vertex: I) -> Point<<VW as HasPosition>::Scalar>
    where
        <VW as HasPosition>::Scalar: geo::CoordNum,
    {
        let position = spade::Triangulation::vertex(&self.cdt, self.handle(vertex)).position();
        point! {x: position.x, y: position.y}
    }
}

impl<
        I: Copy + PartialEq + GetIndex,
        M: IndexMut<I, Output = Option<FixedVertexHandle>>,
        VW: GetTrianvertexNodeIndex<I> + HasPosition,
        EW: Default,
    > visit::GraphBase for Triangulation<I, M, VW, EW>
{
    type NodeId = I;
    type EdgeId = (I, I);
}

#[derive(Debug, Clone, Copy)]
pub struct TriangulationEdgeWeightWrapper<EW> {
    length: f64,
    pub weight: EW,
}

impl<EW> PartialEq for TriangulationEdgeWeightWrapper<EW> {
    fn eq(&self, other: &Self) -> bool {
        self.length.eq(&other.length)
    }
}

impl<EW> PartialOrd for TriangulationEdgeWeightWrapper<EW> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.length.partial_cmp(&other.length)
    }
}

impl<
        I: Copy + PartialEq + GetIndex,
        M: IndexMut<I, Output = Option<FixedVertexHandle>>,
        VW: GetTrianvertexNodeIndex<I> + HasPosition,
        EW: Copy + Default,
    > visit::Data for Triangulation<I, M, VW, EW>
{
    type NodeWeight = VW;
    type EdgeWeight = TriangulationEdgeWeightWrapper<EW>;
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TriangulationEdgeReference<I, EW> {
    from: I,
    to: I,
    weight: TriangulationEdgeWeightWrapper<EW>,
}

impl<I: Copy, EW: Copy> visit::EdgeRef for TriangulationEdgeReference<I, EW> {
    type NodeId = I;
    type EdgeId = (I, I);
    type Weight = TriangulationEdgeWeightWrapper<EW>;

    fn source(&self) -> Self::NodeId {
        self.from
    }

    fn target(&self) -> Self::NodeId {
        self.to
    }

    fn weight(&self) -> &Self::Weight {
        &self.weight
    }

    fn id(&self) -> Self::EdgeId {
        (self.from, self.to)
    }
}

impl<
        'a,
        I: Copy + PartialEq + GetIndex,
        M: IndexMut<I, Output = Option<FixedVertexHandle>>,
        VW: GetTrianvertexNodeIndex<I> + HasPosition,
        EW: Default,
    > visit::IntoNeighbors for &'a Triangulation<I, M, VW, EW>
{
    type Neighbors = Box<dyn Iterator<Item = I> + 'a>;

    fn neighbors(self, vertex: Self::NodeId) -> Self::Neighbors {
        Box::new(
            spade::Triangulation::vertex(&self.cdt, self.handle(vertex))
                .out_edges()
                .map(|handle| self.vertex(handle.to().fix())),
        )
    }
}

impl<
        'a,
        I: Copy + PartialEq + GetIndex,
        M: IndexMut<I, Output = Option<FixedVertexHandle>>,
        VW: GetTrianvertexNodeIndex<I> + HasPosition<Scalar = f64>,
        EW: Copy + Default,
    > visit::IntoEdgeReferences for &'a Triangulation<I, M, VW, EW>
{
    type EdgeRef = TriangulationEdgeReference<I, EW>;
    type EdgeReferences = Box<dyn Iterator<Item = TriangulationEdgeReference<I, EW>> + 'a>;

    fn edge_references(self) -> Self::EdgeReferences {
        Box::new(spade::Triangulation::directed_edges(&self.cdt).map(|edge| {
            let from = self.vertex(edge.from().fix());
            let to = self.vertex(edge.to().fix());

            TriangulationEdgeReference {
                from,
                to,
                weight: TriangulationEdgeWeightWrapper {
                    length: Euclidean::distance(&self.position(from), &self.position(to)),
                    weight: *edge.data(),
                },
            }
        }))
    }
}

impl<
        'a,
        I: Copy + PartialEq + GetIndex,
        M: IndexMut<I, Output = Option<FixedVertexHandle>>,
        VW: GetTrianvertexNodeIndex<I> + HasPosition<Scalar = f64>,
        EW: Copy + Default,
    > visit::IntoEdges for &'a Triangulation<I, M, VW, EW>
{
    type Edges = Box<dyn Iterator<Item = TriangulationEdgeReference<I, EW>> + 'a>;

    fn edges(self, node: Self::NodeId) -> Self::Edges {
        Box::new(
            spade::Triangulation::vertex(&self.cdt, self.handle(node))
                .out_edges()
                .map(|edge| {
                    let from = self.vertex(edge.from().fix());
                    let to = self.vertex(edge.to().fix());

                    TriangulationEdgeReference {
                        from,
                        to,
                        weight: TriangulationEdgeWeightWrapper {
                            length: Euclidean::distance(&self.position(from), &self.position(to)),
                            weight: *edge.data(),
                        },
                    }
                }),
        )
    }
}

impl<
        'a,
        I: Copy + PartialEq + GetIndex,
        M: IndexMut<I, Output = Option<FixedVertexHandle>>,
        VW: GetTrianvertexNodeIndex<I> + HasPosition,
        EW: Default,
    > visit::IntoNodeIdentifiers for &'a Triangulation<I, M, VW, EW>
{
    type NodeIdentifiers = Box<dyn Iterator<Item = I> + 'a>;

    fn node_identifiers(self) -> Self::NodeIdentifiers {
        Box::new(
            spade::Triangulation::fixed_vertices(&self.cdt).map(|vertex| {
                spade::Triangulation::s(&self.cdt)
                    .vertex_data(vertex)
                    .node_index()
            }),
        )
    }
}

#[derive(Debug, PartialEq)]
pub struct TriangulationVertexReference<'a, I, VW> {
    index: I,
    weight: &'a VW,
}

impl<I: Clone, VW> Clone for TriangulationVertexReference<'_, I, VW> {
    fn clone(&self) -> Self {
        Self {
            index: self.index.clone(),
            weight: self.weight,
        }
    }
}

impl<I: Copy, VW> Copy for TriangulationVertexReference<'_, I, VW> {}

impl<I: Copy, VW> visit::NodeRef for TriangulationVertexReference<'_, I, VW> {
    type NodeId = I;
    type Weight = VW;

    fn id(&self) -> Self::NodeId {
        self.index
    }

    fn weight(&self) -> &Self::Weight {
        self.weight
    }
}

impl<
        'a,
        I: Copy + PartialEq + GetIndex,
        M: IndexMut<I, Output = Option<FixedVertexHandle>>,
        VW: GetTrianvertexNodeIndex<I> + HasPosition,
        EW: Copy + Default,
    > visit::IntoNodeReferences for &'a Triangulation<I, M, VW, EW>
{
    type NodeRef = TriangulationVertexReference<'a, I, VW>;
    type NodeReferences = Box<dyn Iterator<Item = TriangulationVertexReference<'a, I, VW>> + 'a>;

    fn node_references(self) -> Self::NodeReferences {
        Box::new(
            spade::Triangulation::fixed_vertices(&self.cdt).map(|vertex| {
                let weight = spade::Triangulation::s(&self.cdt).vertex_data(vertex);
                TriangulationVertexReference {
                    index: weight.node_index(),
                    weight,
                }
            }),
        )
    }
}

impl<
        I: Copy + PartialEq + GetIndex + std::fmt::Debug,
        M: IndexMut<I, Output = Option<FixedVertexHandle>>,
        VW: GetTrianvertexNodeIndex<I> + HasPosition,
        EW: Default,
    > visit::NodeIndexable for &Triangulation<I, M, VW, EW>
{
    fn node_bound(&self) -> usize {
        //self.cdt.num_vertices()
        spade::Triangulation::num_vertices(&self.cdt)
        //FixedVertexHandle::max()
        //Self::new_internal(u32::MAX)
    }

    fn to_index(&self, node: I) -> usize {
        self.trianvertex_to_handle[node].unwrap().index()
    }

    fn from_index(&self, index: usize) -> I {
        spade::Triangulation::s(&self.cdt)
            .vertex_data(FixedVertexHandle::from_index(index))
            .node_index()
    }
}
