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
| C1 | index-addressed thetastar maps (scores/est/predecessors = Vec<Option<T>> via GetIndex); compound-compare without Vec collects | ~0.5s | same search behavior (pops 3941, visits 100745) |
| F2b | `layer_seed` on `MultilayerAutorouteOptions` — parallel workers get pseudo-random initial layer assignments, fanout failures fall back to preferred layer | — | seed 7 routes astable_multivibrator 16/16 |
| G1' | multilayer layer search now rotates **blocker** ratlines too, not only the failed ratline — `record_blockers`' output is surfaced via `PlanarAutorouteReconfigurator::blockers()` and pushed as A* candidates | — | fixes "blocker stays on contested layer" dead-ends |
| — | `fixed_fingerprint` now includes `maybe_net` — a same-index/same-position via swap between nets must invalidate caches | — | found in review; navnode net classification is baked at build time |
| — | cache-invariant tests: fingerprint ignores loose prims, distinguishes same-index net swaps; `loose_snapshot` round-trips through undo; navmesh cache restores identical structure; `new_seeded` determinism/zero-seed/difference | — | `primitive_weight`'s `unreachable` now reports the stale index and slot kind |

candE.dsn is a **4-layer board** (F.Cu, In1.Cu, In2.Cu, B.Cu): all
earlier runs were single-layer, where ~26/107 is a hard ceiling, not
an ordering failure. With `--multilayer --surgical-ripup` the peak
rises to ~38/107 but still oscillates and does not converge within
600s — the outer search rotates the first-failing ratline's layer per
inner attempt (A* over layer assignments), and the inner negotiated-
congestion search keeps permuting. Convergence on candE remains open;
`--jobs N --multilayer` now gives N different layer assignments per
wall-clock budget.

Known latent bug: a nondeterministic `unreachable!()` panic at
`geometry.rs:155` (`primitive_weight` dereferences an index that now
holds a `Compound` node — primitives and compounds share the
StableDiGraph index space, so any index held across a via swap can be
recycled). Seen once on astable_multivibrator at 13/16 after ~3.5 min;
re-runs pass. Reconfiguration timing is wall-clock driven, so the
suite's passing run is luck, not proof. Candidates: indices persisted
in `anteroute_edits`/`terminating_dot_map`/`obstacles` across
re-layering.

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

**Largely obsoleted by the virtual-navcord work** — `ghost_step_to`
already evaluates every probe as pure geometry with no board mutation,
and `materialize` draws only the committed path (D1+D2 effectively
done). What remained was per-probe bookkeeping cost:

- [x] D-memo. Interned navcord path prefixes: probe_memo/step_memo
        keyed by O(1) path ids instead of O(depth) boxed-slice
        hashing/cloning per probe; plus a bbox early-out over virtual
        canes in `ghost_step_to`'s self-intersection scan. candE 50s
        window: within noise (~2%), but strictly less work per probe.
- [x] D3. Retain search state across re-searches — implemented as
        **navcord memo retention** rather than frontier/score reuse:
        probe/step verdicts are pure functions of (path, materialized
        prefix, board state), so a dissolved route's `NavcordSearchState`
        (both memo tables + the path interner they key into) is stored
        in `PrenavmeshBases` keyed by the navmesh key + exact loose
        snapshot. On a whole-navmesh cache hit the exact same navmesh
        object the memo `NavnodeIndex`es refer to comes back, and only
        then is the state handed out — making reuse provably sound.
        Success-path stores are skipped (their snapshot includes the
        new band, unreachable by retries); oversized states (>300k
        entries) are dropped to bound memory; store holds ≤8 entries.
        Channel-probing deliberately stays off this path since its
        relaxed-rule verdicts would poison real routing.
        `TOPOLA_NO_SEARCH_STATE=1` disables retention for A/B checks.
        Measured on candE (55s, `--multilayer --corridor-presort`):
        stored 92, **hits 1** — reconfiguration thrash rarely revisits
        an exact board state (navmesh hits 26/591). Sound and free
        when it fires, but marginal on candE. Remaining upside: finer-
        grained per-entry region invalidation would allow partial reuse
        under changed geometry — significant complexity, not done.
- [x] D3-fix. **Memo staleness bug (latent pre-D3, exposed by
        retention)**: memoized verdicts embed primitive indices (cane
        gears, obstacle refs inside exceptions and probe-obstacle
        lists) that are proven against the *ambient board state at
        record time*. Merges, bow updates and rip-ups can remove those
        primitives while the memo lives — a `step_memo` hit then
        replayed a `Wrap` cane around a vacant bend slot and panicked
        in `Geometry::joints` (deterministic 3s crash in
        `test_astable_multivibrator` with retention on; bisected to
        the D3 commit, but the within-attempt hole predates it —
        retention only widened the replay window).
        Fix: every memo-hit validates that all primitive indices the
        verdict refers to still name live primitives
        (`Geometry::has_primitive`); a dead reference means the verdict
        was recorded under a different ambient state, so the entry is
        evicted and re-evaluated. `NavcordSearchState::retain_live_refs`
        additionally prunes already-dead entries at store time.
        Residual: an entry whose refs all stay live can still replay a
        verdict computed under different surroundings — worst case is
        a wasted attempt, since `materialize` re-checks every cane
        before committing it to the board.
        Verified: `test_astable_multivibrator` passes with retention
        enabled (122s); 46 lib + 28 unilayer + all multilayer tests.

candE profile (50s window, `--multilayer`): step_to 24s of ~50s;
inside it `infringe` 4.75M calls/14.2s with **70M candidates @ ~4.3s**,
`collide` 390k/1.5s, rework_path replays 6M/0.9s. probe_memo hit 19%,
step_memo hit 58%. Remaining levers: fewer *probes* (ordering — Phase
G) and fewer *candidates per query* (R-tree envelope is inflated by
the board's largest net clearance — see `largest_clearance`; a
per-(net-pair) tighter envelope would shrink the candidate set but the
pairwise margin is only known after fetching the candidate). A bbox
precheck inside the candidate loop was tried and reverted — it added
~15ns/candidate and rejected too few to pay.

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

- [x] F1. `probe_channels_par`: probes partitioned across `jobs` workers,
        each on its own `Autorouter` clone with relaxed rules applied
        locally; results merged back in ratline order. `Board`/`Autorouter`
        gained `Clone` for this; worker join panics are surfaced, not
        swallowed. `--suggest-shifts` honors `--jobs` for probing.
        candE (2449 probes): **119s → 29s wall (~4.1×)** on `--jobs 4`
        (~101s CPU). Bit-identical output across reruns of either driver.
        Caveat: probing is *history-sensitive* — reverted bands leave
        allocation-state (slot-vacancy) residue that perturbs later
        searches, so after ~400+ prior probes a marginal relaxed route
        can flip outcome (1/2449 channel diverged: a GND ratline that
        routes only after enough prior probes). Both answers are valid
        samples; sequential output is equally order-sensitive. Also
        surfaced and fixed a pre-existing navmesh panic on
        constraint-edge endpoints absent from the triangulation.
- [x] F2. Parallel permutation search: `topola --jobs N` spawns N
        threads; each loads the .dsn, builds its own board/invoker, and
        routes with a different `shuffle_seed` (deterministic within-SCC
        ratline shuffle in the preconfigurer). First attempt to route
        everything flags the rest to finish early; the invoker that
        routed the most ratlines is kept for reporting/.ses output.
- [x] F3. Resolved by running whole `Autorouter`/`Invoker` instances per
        thread — no `Send` surgery needed on stepper internals.

## Phase G — Smarter ordering from probe data

- [x] G1. `PresortBy::RatlineCorridorCongestion` (`--corridor-presort`):
        presort each SCC's ratlines by corridor obstruction — the count
        of primitives infringing the straight origin→destination seg at
        routing clearances, via the new `infringements_of_shape`
        iterator (`find_infringement_of_shape` now delegates to it).
        Same-net prims and the ratline's own endpoints are excluded by
        the infringement pipeline. Cheaper than running
        `probe_channels`: one R-tree query per ratline at presort time.
        candE `--multilayer --corridor-presort` (50s window): peak
        **38/107 vs 26/107 baseline**, sustained band ~20–37 vs ~12–26;
        still oscillates, doesn't converge. Note: seeded `layer_seed`
        perturbation *hurt* on candE (workers j1–j3 routed 0–22 vs j0's
        38) — preferred layers are near-optimal there.
- [x] G2. Conflict-aware multilayer layer seeding
        (`MultilayerPreconfigurer::new_seeded`): ratlines whose straight
        origin→destination corridors overlap (r-tree-pruned, geo `Line`
        distance < one band width) are spread across layers — the second
        takes a layer not claimed by an already-assigned rival, scanned in
        seed-rotated order so parallel workers still diverge. `layer_seed
        == 0` keeps preferred layers exactly. Replaces the blind
        `splitmix64` shuffle. Deterministic, bounded, unit-tested.
        candE `--jobs 4` result: seeded workers reached 24 vs j0's
        preferred-layer 38 — less harmful than pure random but preferred
        layers still win on this board; seed 0 remains the candE recipe.
        The planar within-SCC piece landed as a conflict-count tiebreak
        under `--corridor-presort`: ratlines whose demand corridors pass
        within one band width of rivals' (same-net pairs excluded — they
        merge) are front-loaded among equally-obstructed ratlines.
        Measured neutral on candE (corridor scores rarely tie; 38/107
        unchanged both ways) — a scheduling signal available when ties
        occur. Reactive blocker precedence remains in
        `PlanarReconfigurer`.
- [x] G3. Relaxed-probe data wired into reconfiguration heuristics.
        `ChannelProbe::blockers()` (moved from `suggest.rs` into the
        library) extracts per-probe obstacle/corridor info;
        `--suggest-shifts` consumes the same path. `--preprobe` runs
        relaxed-clearance probes on the pristine board before routing —
        selection-aware (`probe_selection*` probes only the ratlines the
        run will actually route, so unselected demand doesn't seed
        congestion), parallel under `--jobs`, and identical semantics in
        `run_parallel_jobs` (probe once, seed every worker). Found
        blockers feed `Autorouter::seed_congestion`, which initializes
        `NegotiatedCongestionPlanarAutorouteReconfigurer::congestion` —
        so contested obstacles carry penalties from the very first
        configuration instead of only after negotiated failures.
        Selection-aware probing on candE: 107 probes (not all 2449),
        ~2s, 45 blockers; peak unchanged at 38/107 — mechanism sound,
        neutral on this board.

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

---

## Dependency reduction (completed)

`cargo machete` reported no unused direct dependencies, so reduction
required replacing functionality. Removed from the shipped dependency
set (workspace tree: 171 → 158 unique crates):

- **`peeking_take_while`** — single call in
  `crates/planar-incr-embed/src/utils.rs` replaced by a std `Peekable`
  loop in `rotate_iter`.
- **`ron`** (lib) — the topo-navmesh debug dump in
  `src/autorouter/execution.rs` now uses `serde_json::to_string_pretty`
  (already a workspace dep). `ron` remains only as a
  `planar-incr-embed` dev-dep for insta snapshots.
- **`clap_mangen` + `build.rs`** — deleted; it generated a man page into
  a gitignored `man/` directory that is not shipped anywhere.
- **`env_logger`** — replaced by `crates/topola-cli/src/logger.rs`, a
  ~70-line `log::Log` impl honoring `RUST_LOG` (global level,
  `module=level` directives, longest-prefix wins; default `warn`).
  Drops anstream/env_filter/jiff transitives.
- **`itertools`** — `circular_tuple_windows` in `specctra/design.rs`
  replaced by index arithmetic; `Permutations` in
  `planar_reconfigurer.rs` replaced by a local cloneable
  `SccPermutations` iterator (lexicographic `next_permutation` over
  index vectors; only k=0 and k=n cases were used). Drops `either`.
- **`approx`** — moved to dev-deps; the 4 real-code call sites use new
  `math::abs_diff_eq`/`math::relative_eq` helpers matching the
  crate-default epsilon semantics. Tests keep the `approx` macros.

Retained with rationale: `num-traits` (deep generic bounds throughout
planar-incr-embed — replacing means concretizing to f64), `bimap`
(substantial bidirectional-map usage), `derive-getters`/`enum_dispatch`/
`contracts-try` (macro-driven, many sites), `petgraph`/`spade`/`geo`/
`rstar`/`serde`/`thiserror` (foundational), `clap` (CLI core), all
dev-only test crates (`proptest`/`rstest`/`rstest_reuse`/`insta`).

Caveat: `cargo fmt --check` reports pre-existing diffs in files
untouched by this work — the tree was not fmt-clean before; only
touched files were normalized.

## Stale primitive index panic — diagnosed and fixed

Reproduced on `test_astable_multivibrator` (nondeterministic, ~1 in 3
runs): `primitive_weight` panicked on index 1214 whose slot had been
recycled into a compound. Root cause: multilayer re-layering removed a
ratline's fanout vias (`old_edit.reverse()` in
`MultilayerAutorouteExecutionStepper::reconfigure` phase 2) but, when
re-fanout to the new layer failed **and** restoring the old layer's
fanout also failed, the configuration's `terminating_dot_map` kept the
entry pointing at the dead via dot. `remap_terminating_dots` only ever
extended the map, so the stale `FixedDotIndex` survived until the freed
node slot was recycled by a later `add_via` compound — the next
`ratline_terminating_dots`/`band_between_nodes` deref then hit the
compound and panicked.

Fix: `reconfigure` now tracks the `relayered` ratlines (every changed
ratline that passed the `wrapped` check and had its old edit reversed)
and calls the new `remove_terminating_dots` — plumbed through
`PlanarAutorouteReconfigurator` to both the stepper's live
configuration and the reconfigurer's stored preconfiguration — before
`remap_terminating_dots` applies the fresh entries. Ratlines that kept
their vias (`wrapped` load-bearing gears) keep their entries.

Also: `dot_weight`/`seg_weight`/`bend_weight`/`compound_weight` and the
bend `core_weight` lookup no longer panic bare — they report the
offending index and what the slot actually holds, so any remaining
stale-index path names itself.

Follow-up unit coverage (post-fix):

- `reconfigure_refreshes_terminating_dot_map`
  (`multilayer_autoroute::tests`) — builds a 3-ratline multilayer
  stepper on the astable fixture and cycles one ratline across layers
  twice through `reconfigure`. After each swap it asserts every
  `terminating_dot_map` value resolves to a live dot on the layer the
  recorded plan claims, and that the rotated ratline actually gained
  fanout-dot entries (so the swap path provably ran — not vacuous).
  Deterministic; ~50ms.
- `Permutations<T>` tests (`planar_reconfigurer::tests`) — the
  `itertools` replacement is now generic over `T: Clone` and covered
  for: exhaustive n! enumeration, k=0 single-empty semantics,
  `skip_first` dropping the identity, and clone-forks-iteration (the
  A* expansion clones search nodes wholesale); plus a direct
  `next_permutation` lexicographic/exhaustion test.

## Remaining stale-index hygiene — congestion keys

`PlanarReconfigurer::congestion` used to count every reported
obstacle, including loose band primitives. `obstacle_penalties()`
filtered to fixed primitives, so loose counts were dead weight — but
a *fixed* primitive added after a rip-up (via, fanout dot, escape
stub) can recycle a freed loose slot, resurrecting the stale count as
a misattributed penalty. Loose obstacles are now never inserted; the
filter at penalty derivation is gone. (Loose obstacles still feed
`band_of` → blocker precedence, resolved live at record time.)

## CLI selection fix — `--multilayer` selects all layers

`PinSelection::new_select_layer(board, 0)` was the CLI default for
every command, so `--multilayer` boards only ever selected pins with
a layer-0 pad — pins living solely on other layers were unroutable.
`PinSelection::new_select_all` now backs the `--multilayer` default
(both the single-invoker and `--jobs` paths); planar keeps layer 0.
On candE this is a no-op (107 ratlines either way — every pin-named
endpoint already resolves to a layer-0 dot); on astable it selects
26 pins vs 6. Ratlines whose endpoints lack pin names remain
reachable only via `--nets`.
