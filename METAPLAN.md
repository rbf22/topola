# Metaplan: Making Topola's PCB routing fast and efficient

Goal: reduce autorouting wall-clock time and improve completion rate
(fewer unrouted ratlines) on representative boards.

Baseline metrics (fill in after Phase A):
- `candE.dsn` / example boards: total autoroute time, per-ratline time,
  routed/unrouted count.
- Profile: % time in cane drawing vs. navmesh/prenavmesh construction vs.
  search bookkeeping.

Guiding hypothesis (verified by Phase A, 2026-10-17):
most time is spent in `Navcord::step_to`/`step_back` — i.e. physically
drawing and undrawing cane geometry during Theta* edge probes and
`rework_path` replays.

**Baseline results** (`--features metrics`, release):

| board | time | routed | step_to calls / time | navmesh builds / time |
|---|---|---|---|---|
| 4x4_1206_led_matrix | 62s | 36/36 | 178K / 58.0s (~94%) | 137x / 1.25s (~2%) |
| candE.dsn (120s cap) | timeout | 26/107 | 129K / 99.7s (~83%) | 30x / 12.7s (~10%) |

Key findings:
- **step_to/step_back/rework = ~85–95% of runtime.** Drawing is the cost.
- **~50–58% of probes fail after paying full draw cost** (89K ok vs 89K
  err on 4x4; 64K vs 66K on candE).
- **`visit_navnode` runs ~25–33x per frontier pop** — the backtracking
  loop replays `rework_path` constantly.
- Navmesh construction is cheap on small boards but reaches ~10% on
  candE — Phase B still worthwhile but demoted below C/D.
- Reconfigurations cause repeated navmesh builds (137 builds for 36
  ratlines) — cheap enough to keep.

**Progress log**

| date | change | 4x4 board time | notes |
|---|---|---|---|
| baseline | — | 62s | step_to ~58s, navmesh ~1.2s |
| C3 | probe verdict memoization on `Navcord` | 14s | 29K/105K probes replayed virtually |
| C4a | trusted replay — `drawn_memo` + `*_infringably` draw paths | 3.6s | 65K/71K draws skip all checks |
| C5 | query filter reordering + loose-dot early-out | 3.1s | candidates 14.8M→8.8M, queries 2.3→1.4s |

Post-C5 breakdown on 4x4 (instrumented): step_to ~2.3s (infringe queries
1.38s, mutations ~1.2s), navmesh+prenavmesh ~0.7s (~20%), rest small.
Remaining real probes (~75K) still pay ~3-4 queries each — next lever is
ghost probes (shape-based union query, no mutation) or Phase D laziness.

---

## Phase A — Measurement infrastructure (blocks everything else)

- [x] A1. `metrics` cargo feature on `topola`/`topola-cli` — atomic
      counters + ns timers in `src/metrics.rs`; CLI prints report at end.
- [x] A2. Instrumented: `Prenavmesh::new`, `Navmesh::new`, `step_to`,
      `step_back`, `rework_path` (incl. steps undone/redrawn), frontier
      pops, stale pops, `visit_navnode`, `reconstruct_path_to`, probe
      outcome classes (los_ok/failed/backtracked).
- [x] A3. Baselines captured (table above): 4x4_1206_led_matrix and
      candE.dsn.
- [ ] A4. Optional later: samply flamegraph to break down step_to
      internals (bitangents vs. collision query vs. R-tree insert).

## Phase B — Prenavmesh reuse

- [ ] B1. `probe_channels`: hoist `Prenavmesh::new` out of the per-ratline
      loop. Board is reverted between probes, so one triangulation can be
      reused for all probes via `Navmesh::new_from_prenavmesh`.
- [ ] B2. Sequential autoroute: keep a persistent `Triangulation`; after
      each routed band, incrementally insert its new fixed primitives
      (`Triangulation::add_vertex` / `add_constraint_edge`) instead of a
      full `Prenavmesh::new` rebuild. Invalidate/rebuild only the affected
      navnode region of the navmesh graph.
- [ ] B3. Benchmark: expect navmesh construction to drop out of the profile
      for all but the first ratline.

Risk: incremental correctness (bands add loose bends/segs that later become
fixed obstacles; fillets and unions complicate this). Mitigate with
fallback: rebuild when the delta exceeds a threshold or when incremental
insert hits an unsupported case.

## Phase C — Cheap wins in the search loop

- [ ] C1. Replace `BTreeMap` g-scores in `src/astar.rs` with a dense `Vec`
      (nodes are `NodeIndex`-numbered) or HashMap; same for
      `thetastar.rs` maps (`scores`, `estimated_costs`, `predecessors` →
      `Vec`-indexed-by-navnode where domain allows).
- [ ] C2. Kill per-visit allocations: reuse a buffer in
      `reconstruct_path_to`; compare compounds via iterators instead of
      collecting two `Vec`s in `place_probe_to_navnode`
      (`router.rs:131-158`); reuse `probe_ghosts`/`probe_obstacles` buffers.
- [x] C3. Memoize probe results: cache `step_to` verdicts keyed by
      (navcord-path-state, initial-parent, target navnode) in
      `Navcord::probe_memo`. 62s → 14s on 4x4.
- [x] C4. Reduce `rework_path` churn — done via "trusted replay":
      `Navcord::drawn_memo` records successful non-squeezed draws keyed by
      (path, to); `step_to` replays them via
      `Drawing::add_cane_infringably`/`move_dot_infringably` with no
      queries at all. 14s → 3.6s on 4x4 (91% of draws trusted).
      (`visit_navnode` itself still runs ~25x/pop but each rework is now
      cheap; skipping no-op reworks remains a small TODO.)
- [x] C5. Query filter reordering (`query.rs`): reject loose-dot and
      same-net infringees before geometric evaluation; early-out when
      infringer is a loose dot. Candidates 14.8M→8.8M, query time 2.3→1.4s.
- [ ] C6. `SegShape` trig removal: replaced `rotate_around_point(-90°)`
      with direct `(y, -x)` in `corners`/`rect_intersects_rect`.
- [ ] C7. Ghost probes: evaluate probe feasibility on hypothetical
      shapes (dot/seg/zero-length bend + moved head limbs) via one union
      R-tree query — no speculative insertion/removal, no edit recording.
      Fall back to the real `step_to` on failure (insert_cane squeeze
      path needs real mutation anyway).

## Phase D — Lazy edge validation (Lazy-Theta*)

The structural fix for "every edge probe draws geometry".

- [ ] D1. Give navedges a cheap analytic cost (chord/tangent estimate —
        same quantities `estimate_cost_to_goal` already computes) and run
        Theta* without drawing.
- [ ] D2. Validate lazily: when the search settles on a candidate path,
        draw it for real. On infringement, record which navedge failed
        (obstacle info is already captured via `ghosts`/`obstacles`),
        penalize or remove that edge, and resume/re-run the search.
- [ ] D3. Keep incremental search state across re-searches (retain frontier
        and scores; re-insert only affected frontier entries — D*-lite
        style) so each failed candidate doesn't restart from scratch.
- [ ] D4. Fall back to eager drawing when lazy validation exceeds N
        consecutive failures on one ratline (dense boards may thrash).

Expected effect: draws go from O(edge probes) to O(candidate paths).
Biggest win but largest design change — needs Phase A numbers to justify.

## Phase E — Evaluate the `ng` (PieNavmesh) router

- [ ] E1. Benchmark `topo_autoroute` vs `planar_autoroute` on the same
        boards (it already computes `PieNavmesh` once per board and runs
        multi-goal A* — the architecturally better design).
- [ ] E2. If competitive: fix its hot spots — `evaluate_navmesh` clones
        layout, recorder, and `BTreeMap`s per edge evaluation
        (`eval.rs:91-92`); replace clones with in-place apply/revert.
- [ ] E3. Decide where to invest: legacy planar path (Phase D) vs. ng
        path going forward.

## Phase F — Parallelism (exploration, not per-route)

- [ ] F1. `probe_channels` is embarrassingly parallel: each probe reverts
        the board — run probes on `Board` clones across threads (rayon).
        Measure clone cost vs. probe cost first.
- [ ] F2. Parallel permutation search: evaluate multiple
        ratline orderings (presort variants, shuffles) concurrently on
        board clones; keep the best result. Turns "more exploration" into
        free wall-clock on multicore.
- [ ] F3. Requires: making the stepper interfaces Send-capable or running
        whole `Autorouter` instances per thread (simpler).

## Phase G — Smarter ordering from probe data

- [ ] G1. Feed `probe_channels` results (blockers, corridors, terminal
        blockers) into `PresortBy`: route most-constrained ratlines first,
        or reorder around hard blockers.
- [ ] G2. Use probe corridor overlap to detect ratlines that will conflict
        and schedule them so the second takes the alternate channel.
- [ ] G3. Wire relaxed-probe failure info into `--suggest-shifts` (already
        partially done) and into reconfiguration heuristics.

---

## Suggested execution order (revised after baseline)

1. ~~**A**~~ done — step_to drawing is ~85–95% of cost; navmesh build
   ~2–10%.
2. **C3** — probe memoization; dedupes the retry-probes and re-expansion
   probes. Highest value/effort ratio.
3. **C4** — rework_path/visit_navnode churn reduction.
4. **B2 + B1** — incremental triangulation + probe prenavmesh reuse
   (matters on candE-scale boards: ~10%).
5. **D** — lazy edge validation; the structural fix for draw-per-probe.
6. **E1** — benchmark ng router to decide where D-style effort belongs.
7. **F** — parallel probes/ordering search (needs `Send` boundaries;
   check after C/D change the shape of the hot loop).
8. **G** — probe-data-driven ordering (compounds with everything above).
