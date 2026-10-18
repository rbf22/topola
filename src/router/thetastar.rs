// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT
//
// This file was originally copied from astar.rs of library Petgraph at
// https://github.com/petgraph/petgraph/blob/master/src/algo/astar.rs
//
// Petgraph's original copyright line is
// Copyright (c) 2015

use std::collections::BinaryHeap;
use std::hash::Hash;

use std::ops::{ControlFlow, Sub};

use derive_getters::Getters;
use petgraph::algo::Measure;
use petgraph::visit::{EdgeRef, GraphBase, IntoEdgeReferences, IntoEdges};
use thiserror::Error;

use crate::graph::GetIndex;
use crate::scored::MinScored;
use crate::stepper::{EstimateProgress, LinearScale, Step};

/// Index-addressed per-node slot: node ids are dense `NodeIndex`-style
/// numbers, so a grow-on-demand `Vec` beats a hash map on every lookup.
fn node_slot<'a, T: Copy, N: GetIndex>(slots: &'a [Option<T>], node: N) -> Option<T> {
    slots.get(node.index()).copied().flatten()
}

fn set_node_slot<T: Clone, N: GetIndex>(slots: &mut Vec<Option<T>>, node: N, value: T) {
    let index = node.index();
    if index >= slots.len() {
        slots.resize(index + 1, None);
    }
    slots[index] = Some(value);
}

#[derive(Debug)]
pub struct PathTracker<G>
where
    G: GraphBase,
    G::NodeId: Eq + Ord + Hash + GetIndex,
{
    predecessors: Vec<Option<G::NodeId>>,
}

impl<G> PathTracker<G>
where
    G: GraphBase,
    G::NodeId: Eq + Ord + Hash + GetIndex,
{
    fn new() -> PathTracker<G> {
        PathTracker {
            predecessors: vec![],
        }
    }

    fn predecessor(&self, node: G::NodeId) -> Option<G::NodeId> {
        node_slot(&self.predecessors, node)
    }

    fn set_predecessor(&mut self, node: G::NodeId, previous: G::NodeId) {
        set_node_slot(&mut self.predecessors, node, previous);
    }

    /// Reconstructs the path to `last` into `path`, reusing its
    /// allocation. Called once per navnode visit.
    pub fn reconstruct_path_into(&self, path: &mut Vec<G::NodeId>, last: G::NodeId) {
        crate::metrics::incr(&crate::metrics::METRICS.path_reconstructions);
        path.clear();
        path.push(last);

        let mut current = last;
        while let Some(previous) = self.predecessor(current) {
            path.push(previous);
            current = previous;
        }

        path.reverse();
    }

    pub fn reconstruct_path_to(&self, last: G::NodeId) -> Vec<G::NodeId> {
        let mut path = vec![];
        self.reconstruct_path_into(&mut path, last);
        path
    }
}

pub trait ThetastarStrategy<G, K, R>
where
    G: GraphBase,
    G::NodeId: Eq + Ord + Hash + GetIndex,
    for<'a> &'a G: IntoEdges<NodeId = G::NodeId, EdgeId = G::EdgeId> + MakeEdgeRef,
    K: Measure + Copy,
{
    fn visit_navnode(
        &mut self,
        graph: &G,
        navnode: G::NodeId,
        tracker: &PathTracker<G>,
    ) -> Result<Option<R>, ()>;
    fn place_probe_to_navnode(
        &mut self,
        graph: &G,
        initial_parent_navnode: Option<G::NodeId>,
        probed_navnode: G::NodeId,
    ) -> ControlFlow<Option<K>>;
    fn remove_probe(&mut self, graph: &G);
    fn estimate_cost_to_goal(&mut self, graph: &G, navnode: G::NodeId) -> K;
}

pub trait MakeEdgeRef: IntoEdgeReferences {
    fn edge_ref(&self, edge_id: Self::EdgeId) -> Self::EdgeRef;
}

/// The enum that describes the current state of the algorithm.
#[derive(Clone, Copy, Debug)]
pub enum ThetastarState<N: Copy, E: Copy> {
    /// Visit a new navnode and copy all navedges going out of it.
    Scanning,
    /// Load a new navedge.
    VisitFrontierNavedge(N),
    /// Backtrack by one navnode, then line-of-sight probe to target of the
    /// currently visited navedge. Transition to this state again until a
    /// condition is met, then continue the algorithm by transitioning to the
    /// `ProbeOnNavedge(...)` state.
    BacktrackAndProbeOnLineOfSight(N, E),
    ProbeOnNavedge(N, E),
    /// The probe is in place, retract it and continue to the next navedge.
    Probing(N),
}

/// The pathfinding algorithm Topola uses to find the shortest path to route
/// is Theta* with conditional repeated backtracking. Theta* is just A* with an
/// improvement: every time an navedge is scanned, the algorithm first tries to
/// draw directly to its target navnode from the predecessor of the currently
/// visited navnode. Note that this creates paths with edges that do not all lie
/// on the navmesh.
///
/// Conditional repeated backtracking is our improvement to Theta*: if
/// line-of-sight routing fails, for as long as a condition is met, continue
/// trying to draw from parent of the parent navnode, and so on. This is
/// different from Theta* because in Theta* there is only one backtracking step
/// -- only one attempt to do line-of-sight routing.
#[derive(Getters)]
pub struct ThetastarStepper<G, K>
where
    G: GraphBase,
    G::NodeId: Eq + Ord + Hash + GetIndex,
    for<'a> &'a G: IntoEdges<NodeId = G::NodeId, EdgeId = G::EdgeId> + MakeEdgeRef,
    K: Measure + Copy + Sub<Output = K>,
{
    state: ThetastarState<G::NodeId, G::EdgeId>,
    graph: G,
    /// The priority queue of the navnodes to expand.
    #[getter(skip)]
    frontier: BinaryHeap<MinScored<K, G::NodeId>>,
    /// Also known as the g-scores, or just g. Index-addressed:
    /// `scores[node.index()]` is `Some(g)` once `node` is scored.
    scores: Vec<Option<K>>,
    /// Also known as the f-scores, or just f.
    estimated_costs: Vec<Option<K>>,
    #[getter(skip)]
    path_tracker: PathTracker<G>,
    // FIXME: To work around edge references borrowing from the graph we collect then reiterate over them.
    #[getter(skip)]
    edge_ids: Vec<G::EdgeId>,

    #[getter(skip)]
    progress_estimate_value: K,

    #[getter(skip)]
    progress_estimate_maximum: K,
}

#[derive(Error, Debug, Clone)]
pub enum ThetastarError {
    #[error("A* search found no path")]
    NotFound,
}

impl<G, K> ThetastarStepper<G, K>
where
    G: GraphBase,
    G::NodeId: Eq + Ord + Hash + GetIndex,
    for<'a> &'a G: IntoEdges<NodeId = G::NodeId, EdgeId = G::EdgeId> + MakeEdgeRef,
    K: Measure + Copy + Sub<Output = K>,
{
    pub fn new<R>(
        graph: G,
        start: G::NodeId,
        strategy: &mut impl ThetastarStrategy<G, K, R>,
    ) -> Self {
        let estimated_cost_from_start_to_goal = strategy.estimate_cost_to_goal(&graph, start);

        let mut this = Self {
            state: ThetastarState::Scanning,
            graph,
            frontier: BinaryHeap::new(),
            scores: vec![],
            estimated_costs: vec![],
            path_tracker: PathTracker::<G>::new(),
            edge_ids: Vec::new(),
            progress_estimate_value: K::default(),
            progress_estimate_maximum: estimated_cost_from_start_to_goal,
        };

        set_node_slot(&mut this.scores, start, K::default());
        this.frontier
            .push(MinScored(estimated_cost_from_start_to_goal, start));
        this
    }

    fn push_to_frontier<R>(
        &mut self,
        next: G::NodeId,
        next_score: K,
        predecessor: G::NodeId,
        strategy: &mut impl ThetastarStrategy<G, K, R>,
    ) {
        let cost_to_goal_estimate = strategy.estimate_cost_to_goal(&self.graph, next);

        if cost_to_goal_estimate > self.progress_estimate_maximum {
            self.progress_estimate_maximum = cost_to_goal_estimate;
        }

        if self.progress_estimate_maximum - cost_to_goal_estimate > self.progress_estimate_value {
            self.progress_estimate_value = self.progress_estimate_maximum - cost_to_goal_estimate;
        }

        self.path_tracker.set_predecessor(next, predecessor);
        let next_estimate_score = next_score + cost_to_goal_estimate;
        self.frontier.push(MinScored(next_estimate_score, next));
    }
}

impl<G, K, R, S: ThetastarStrategy<G, K, R>>
    Step<S, (K, Vec<G::NodeId>, R), ThetastarState<G::NodeId, G::EdgeId>> for ThetastarStepper<G, K>
where
    G: GraphBase,
    G::NodeId: Eq + Ord + Hash + GetIndex,
    for<'a> &'a G: IntoEdges<NodeId = G::NodeId, EdgeId = G::EdgeId> + MakeEdgeRef,
    K: Measure + Copy + Sub<Output = K>,
{
    type Error = ThetastarError;

    fn step(
        &mut self,
        strategy: &mut S,
    ) -> Result<
        ControlFlow<(K, Vec<G::NodeId>, R), ThetastarState<G::NodeId, G::EdgeId>>,
        ThetastarError,
    > {
        match self.state {
            ThetastarState::Scanning => {
                let Some(MinScored(estimate_score, navnode)) = self.frontier.pop() else {
                    return Err(ThetastarError::NotFound);
                };

                crate::metrics::incr(&crate::metrics::METRICS.frontier_pops);

                if let Ok(maybe_result) =
                    strategy.visit_navnode(&self.graph, navnode, &self.path_tracker)
                {
                    if let Some(result) = maybe_result {
                        let path = self.path_tracker.reconstruct_path_to(navnode);
                        let cost = node_slot(&self.scores, navnode).unwrap();

                        return Ok(ControlFlow::Break((cost, path, result)));
                    }

                    // If the node has already been visited with an equal or lower
                    // estimated score than now, then we do not need to re-visit it.
                    if node_slot(&self.estimated_costs, navnode)
                        .is_some_and(|score| score <= estimate_score)
                    {
                        crate::metrics::incr(&crate::metrics::METRICS.stale_frontier_pops);
                        return Ok(ControlFlow::Continue(self.state));
                    }
                    set_node_slot(&mut self.estimated_costs, navnode, estimate_score);

                    self.edge_ids.clear();
                    self.edge_ids
                        .extend(self.graph.edges(navnode).map(|edge| edge.id()));

                    self.state = ThetastarState::VisitFrontierNavedge(navnode);
                }
            }
            ThetastarState::VisitFrontierNavedge(visited_navnode) => {
                if let Some(visited_navedge) = self.edge_ids.pop() {
                    self.state = ThetastarState::BacktrackAndProbeOnLineOfSight(
                        visited_navnode,
                        visited_navedge,
                    );
                } else {
                    self.state = ThetastarState::Scanning;
                }
            }
            ThetastarState::BacktrackAndProbeOnLineOfSight(curr_navnode, visited_navedge) => {
                // This lookup can be unwrapped without fear of panic since the node was
                // necessarily scored before adding it to `.visit_next`.
                //let node_score = self.scores[&visited_navnode];
                let initial_source_navnode = (&self.graph).edge_ref(visited_navedge).source();
                let initial_parent_navnode = self.path_tracker.predecessor(initial_source_navnode);
                let probed_navnode = (&self.graph).edge_ref(visited_navedge).target();

                if let Some(parent_navnode) = self.path_tracker.predecessor(curr_navnode) {
                    strategy.visit_navnode(&self.graph, parent_navnode, &self.path_tracker);
                    let parent_score = node_slot(&self.scores, parent_navnode).unwrap();

                    match strategy.place_probe_to_navnode(
                        &self.graph,
                        initial_parent_navnode,
                        probed_navnode,
                    ) {
                        ControlFlow::Continue(()) => {
                            // Transition to self to repeatedly backtrack.
                            self.state = ThetastarState::BacktrackAndProbeOnLineOfSight(
                                parent_navnode,
                                visited_navedge,
                            );
                        }
                        ControlFlow::Break(Some(los_cost)) => {
                            let next = probed_navnode;
                            let next_score = parent_score + los_cost;

                            // No need to add neighbors that we have already reached through a
                            // shorter path than now.
                            if node_slot(&self.scores, next)
                                .is_some_and(|score| score <= next_score)
                            {
                                // We just remove the probe instantly
                                // here instead of doing it in
                                // ThetastarState::Probing or a new
                                // state to avoid complicating.
                                strategy.remove_probe(&self.graph);

                                self.state = ThetastarState::ProbeOnNavedge(
                                    initial_source_navnode,
                                    visited_navedge,
                                );
                                return Ok(ControlFlow::Continue(self.state));
                            }
                            set_node_slot(&mut self.scores, next, next_score);

                            self.push_to_frontier(next, next_score, parent_navnode, strategy);

                            self.state = ThetastarState::Probing(curr_navnode);
                        }
                        ControlFlow::Break(None) => {
                            // Come back to initial navnode if drawing failed
                            // and the backtracking condition is not met.
                            strategy.visit_navnode(
                                &self.graph,
                                initial_source_navnode,
                                &self.path_tracker,
                            );

                            self.state = ThetastarState::ProbeOnNavedge(
                                initial_source_navnode,
                                visited_navedge,
                            );
                        }
                    }
                } else {
                    // Come back from current navnode if drawing from it failed.
                    strategy.visit_navnode(&self.graph, initial_source_navnode, &self.path_tracker);

                    self.state =
                        ThetastarState::ProbeOnNavedge(initial_source_navnode, visited_navedge);
                }
            }
            ThetastarState::ProbeOnNavedge(visited_navnode, visited_navedge) => {
                let visited_score = node_slot(&self.scores, visited_navnode).unwrap();
                let initial_source_navnode = (&self.graph).edge_ref(visited_navedge).source();
                let initial_parent_navnode = self.path_tracker.predecessor(initial_source_navnode);
                let probed_navnode = (&self.graph).edge_ref(visited_navedge).target();

                if let ControlFlow::Break(Some(navedge_cost)) = strategy.place_probe_to_navnode(
                    &self.graph,
                    initial_parent_navnode,
                    probed_navnode,
                ) {
                    let next = probed_navnode;
                    let next_score = visited_score + navedge_cost;

                    // No need to add neighbors that we have already reached through a
                    // shorter path than now.
                    if node_slot(&self.scores, next).is_some_and(|score| score <= next_score) {
                        self.state = ThetastarState::Probing(visited_navnode);
                        return Ok(ControlFlow::Continue(self.state));
                    }
                    set_node_slot(&mut self.scores, next, next_score);

                    self.push_to_frontier(next, next_score, visited_navnode, strategy);

                    self.state = ThetastarState::Probing(visited_navnode);
                } else {
                    self.state = ThetastarState::VisitFrontierNavedge(visited_navnode);
                }
            }
            ThetastarState::Probing(visited_navnode) => {
                strategy.remove_probe(&self.graph);

                self.state = ThetastarState::VisitFrontierNavedge(visited_navnode);
            }
        }

        Ok(ControlFlow::Continue(self.state))
    }
}

impl<G, K> EstimateProgress for ThetastarStepper<G, K>
where
    G: GraphBase,
    G::NodeId: Eq + Ord + Hash + GetIndex,
    for<'a> &'a G: IntoEdges<NodeId = G::NodeId, EdgeId = G::EdgeId> + MakeEdgeRef,
    K: Measure + Copy + Sub<Output = K>,
{
    type Value = K;
    type Subscale = ();

    fn estimate_progress(&self) -> LinearScale<Self::Value> {
        LinearScale::new(
            self.progress_estimate_value,
            self.progress_estimate_maximum,
            (),
        )
    }
}
