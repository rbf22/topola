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
| C6+C7 | trig removal + ghost probes (no speculative mutation) | 1.8s | step_to errors 71K→126; queries still ~200ms |
| C8 | virtual navcord — `VirtualCane` state, materialize at commit | ~1.8s | inserts 265K→2.6K, removes 263K→0, rework churn gone |
| C9 | `step_memo` — replay memoized `.step_to(..)` verdicts | ~1.0s | 77.6K hits (56%), rework redraws ~580ms→19ms |
| B0 | spade `bulk_load_cdt` for prenavmesh vertices | ~0.8s | prenavmesh 501→332ms, construction 567→375ms |
| B2' | per-(net,layer) prenavmesh vertex-base cache | ~0.7s | 133 builds → 12 bases + 133 from-base clones |
| B3 | constrained-base delta cache (geometry `ConstraintKey`s) | ~0.8s | construction 227→~126ms; deltas 144x on 4x4 |
| — | geometry `generation` counter on `GeometryWithRtree` | — | bumps on all mutators + `apply` (undo/redo safe) |
| — | guard: don't reconfigure when stepper already routed all ratlines | — | completed configurations were being reconfigured away before commit |
| B3+ | constrained-base LRU keyed by exact `ConstraintKey` set + whole-navmesh cache validated by exact loose-prim snapshot | ~0.5s | 4x4: bases 12x, constrained 24x (hits 94, deltas 180x), navmesh hit 1x |
| C2 | `path_scratch` for `reconstruct_path_into`; nested memos — no per-call path clone | — | removes ~100K Vec allocs per 4x4 run |
| F2 | `--jobs N`: N threads each load the dsn, route with a different `shuffle_seed`, best invoker kept | — | first full completion flags siblings to finish early |

candE.dsn after C8–B2' (300s cap): construction 197s (~66%),
step_to evals 72.9s, rework 2.7s, draw machinery ~0 — profile is now
dominated by prenavmesh rebuilds (hence B2'/B3) and by the search not
converging (oscillates 9–26/107 — an ordering problem, not raw speed).

candE.dsn after B3+/B4/C2 (300s cap): constrained-base LRU hit
2448/2494 builds (41 full builds, 53 deltas), vertex bases 5x, whole-
navmesh hits 87x; navmesh conversion 73.8s is still the top
construction cost; step_to evals 137s. **Ordering still does not
converge** — oscillates 7–26/107 to the deadline. Caveat for reading
progress: `finish_early` forces `curr_ratline_index = len`, so the
"routed 107/107 (100%)" print after a deadline is an artifact of early
finishing, not real completion — earlier "candE converged" readings
were this artifact.

Whole-navmesh caching — first tried and rejected, then **accepted
behind exact validation**: navnodes retain `LooseBend`/`PrimitiveIndex`
references whose slots are recycled after rip-ups (the `unreachable!`
in `GetWeight`), but `apply()` restores indices and weights bit-for-bit.
The cache therefore keys on `(net, layer, origin, destination, fixed
fingerprint, options)` and validates reuse with an exact
`PrimitiveIndex → LooseSig` snapshot (weight, joints, inner-bend
linkage): an equal snapshot proves every loose reference in the cached
navnodes resolves to the same weight. Backtraces hit legitimately; any
state not bit-identical misses and rebuilds. Bounded to 32 navmeshes.
The `generation` counter detects any board mutation including
`apply`/undo — it is deliberately NOT used as a cache key because every
band commit bumps it.

Cache-invalidation subtlety found by metrics: the fixed-primitive
fingerprint must fold order-independently — `layer_primitive_nodes`
yields rtree traversal order, which shifts as loose prims come and go.
An order-dependent fold flaps on every call and forces vertex-base
rebuilds (87x vs 12x on 4x4); the count⊕sum fold restores stability.

Post-B2' breakdown on 4x4 (instrumented): construction ~227ms (~30%),
step_to eval ~334ms (~45%), thetastar bookkeeping + rest ~200ms.
Unit coverage for the virtual navcord: 8 tests in `router::navcord::tests`
(virtual step, materialize, step_back, finish, replay, AlreadyConnected).

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

- [x] B0. spade `bulk_load_cdt` for the vertex set in
      `Triangulation::bulk_load` — near-linear CDT build instead of a
      locate+insert per vertex. ~35% off prenavmesh construction.
- [x] B2'. Per-(net,layer) vertex-base cache — done differently than
      planned: spade CDTs cannot remove constraints, so instead
      `Prenavmesh::vertex_triangulation_for_net` builds the vertex set
      once per (net, layer) and `PlanarAutorouteExecutionStepper` clones
      it per ratline; `from_vertex_base` adds origin/destination and the
      current constraints. 133 builds → 12 bases on 4x4.
- [x] B1. `probe_channels`: shared `PrenavmeshBases` cache hoisted out of
      the per-ratline loop — probes reuse vertex/constrained bases since
      the board reverts after each probe.
- [x] B3. Constraint-side caching: `PrenavmeshBases` keeps a bounded MRU
      of constrained bases per (net, layer), keyed by the exact
      `ConstraintKey` set (endpoint/core geometry, not
      `PrimitiveIndex`es) — exact-set hits reuse outright, supersets
      delta-extend the largest subset entry, rip-ups and index
      recycling fail the check and rebuild. Self-validating, no explicit
      invalidation hooks needed.
- [x] B4. Whole-navmesh cache in `PrenavmeshBases`, keyed
      (net, layer, origin, destination, fixed fingerprint, options) and
      gated by exact `PrimitiveIndex → LooseSig` snapshot equality —
      `apply()` restores indices/weights bit-for-bit, so equal snapshots
      prove the navnodes' loose-primitive references still resolve.
- [x] B5. Completion guard: `PlanarAutorouteReconfigurator::step` no
      longer reconfigures once the stepper routed every ratline — a
      fully routed configuration used to be backtraced away on the same
      tick instead of committing (the candE "107/107 then 26 kept"
      symptom).

Risk (resolved): incremental vertex insertion turned out unnecessary —
bulk_load + clone amortizes it. Whole-navmesh caching was first
rejected over stale `LooseBend` references, then accepted behind the
exact-snapshot validation in B4 (see progress log).

## Phase C — Cheap wins in the search loop

- [x] C1. `thetastar.rs` maps are now index-addressed `Vec<Option<T>>`
      (`scores`, `estimated_costs`, `predecessors`) via `GetIndex` —
      grow-on-demand, no hashing. `src/astar.rs` keeps `BTreeMap`: its
      keys are `SccSearchNode` permutation states, not index-addressable,
      and the permutation search is cold anyway.
- [x] C2. Kill per-visit allocations: `PathTracker::reconstruct_path_into`
      writes into a `path_scratch` buffer on the strategy; `probe_memo`
      and `step_memo` became nested `path → inner map` so lookups borrow
      the live path (`Box<[N]>: Borrow<[N]>`) instead of cloning it per
      probe/step; same-compound check in `place_probe_to_navnode`
      compares compound iterators without collecting `Vec`s.
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
- [x] C6. `SegShape` trig removal: replaced `rotate_around_point(-90°)`
      with direct `(y, -x)` in `corners`/`rect_intersects_rect`.
- [x] C7. Ghost probes: evaluate probe feasibility on hypothetical
      shapes — `Navcord::ghost_step_to` replicates every check of the
      real draw on shapes only. Real `step_to` errors dropped 71K→126.
- [x] C8. Virtual navcord: the in-flight band lives as `VirtualCane`
      geometry in `navcord.canes`; `materialize` draws it once at
      destination commit via checked `wrap`. Physical mutation during
      search eliminated (removes 263K→0 on 4x4; 10.9M→0 on candE).
- [x] C9. `step_memo`: `.step_to(..)` verdicts memoized on
      (path, materialized-prefix-len, to) — `rework_path` redraws become
      replays. 9.08M hits on candE, rework redraws ~580ms→19ms on 4x4.

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

- [x] E1. Benchmarked via `Command::TopoAutoroute` on the 4x4 board
        (`-c` command file): the ng router did not finish in 2.5+ min
        where the planar path takes ~0.5s — not competitive as-is.
- [ ] E2. If competitive: fix its hot spots — `evaluate_navmesh` clones
        layout, recorder, and `BTreeMap`s per edge evaluation
        (`eval.rs:91-92`); replace clones with in-place apply/revert.
- [x] E3. Decision: keep investing in the legacy planar path. ng needs
        E2-style work before it can even be measured meaningfully;
        revisit only if Phase D stalls.

## Phase F — Parallelism (exploration, not per-route)

- [ ] F1. `probe_channels` is embarrassingly parallel: each probe reverts
        the board — run probes on `Board` clones across threads (rayon).
        Measure clone cost vs. probe cost first.
- [x] F2. Parallel permutation search: `topola --jobs N` spawns N
        threads; each loads the .dsn, builds its own board/invoker, and
        routes with a different `shuffle_seed` (deterministic within-SCC
        ratline shuffle in the preconfigurer). First attempt to route
        everything flags the rest to finish early; the invoker that
        routed the most ratlines is kept for reporting/.ses output.
- [x] F3. Resolved by running whole `Autorouter`/`Invoker` instances per
        thread — no `Send` surgery needed on stepper internals.

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
