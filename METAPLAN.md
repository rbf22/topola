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
- [x] A4. samply flamegraph of candE `--multilayer --corridor-presort`
      (40s window, release+debuginfo build; offline-symbolicated since
      samply leaves offsets unresolved — add 0x100000000 and run
      `atos -o target/release/topola`):

      Post-D3 cost structure on candE — `step_to` is 49% total, but the
      big new bucket is **navmesh per-attempt work at ~31%**
      (`PrenavmeshBases::navmesh_in` 14.5% self / 31.4% total):
      `add_prenavedge_to_repr_as_quadrinavedges` 5.7% rebuilding navmesh
      edges, plus ~6.6% just computing cache keys
      (`fixed_fingerprint` 3.9% + `navmesh_key_and_snapshot` 2.7%).
      Retained search state avoids re-search but not re-build.
      rstar `SelectionIterator::next` is the hottest single frame at
      23.5% self — R-tree traversal inside `overlapee_limbs` (11.4%)
      and infringement queries. `ghost_extend_head` 18% total remains
      the inner draw path. Iterator/collection machinery (FlatMap,
      SmallVec extend, DefaultHasher) ~13% combined — navmesh-build
      bookkeeping. `reconfigure` 10.6%.

      Corroborating `--features metrics` run on candE: `navmesh` builds
      **421× @ 8.7s with only 15 cache hits** — the whole-navmesh
      loose-snapshot match almost never succeeds once attempts mutate
      the board; `search_states` similarly **65 stored, 0 hits** (D3
      retention can't fire without a navmesh hit, since memo keys are
      NavnodeIndexes into that navmesh). `infringe` is the hottest
      single operation: 3.8M calls @ 12.3s, 53.7M candidate checks —
      the same rstar traversal the flamegraph flags. `visit_navnode`/
      `path_reconstruct`/`rework_path` run 4.6M× each — every A*
      expansion pays a navcord rework (cheap per call, 778ms total).

      Fix landed from this data: `Navmesh.graph`/`prenavmesh` are now
      `Arc`-shared so the cache-hit path (and probe-worker clones of
      `Autorouter`, which carry `PrenavmeshBases`) no longer
      deep-copies a graph plus a whole triangulation per handout.

      Next targets implied: (a) ~~`infringe` candidate reduction~~ —
      **tried and reverted**: a per-candidate bbox-distance-vs-clearance
      prune ahead of `inflate`+`intersects` rejected only 43K of 45.2M
      candidates (0.1%); candidates surviving the largest-clearance
      envelope genuinely sit within pairwise clearance, so the cost is
      real geometry work plus rstar traversal, not over-selection.
      (b) ~~navmesh reuse across attempts with a looser key~~ —
      **analyzed and rejected**: `ConstraintKey` deliberately excludes
      loose bends/dots, but `wrap_around_bands`/`squeeze` create
      navnodes whose gears reference loose bends by board index, so the
      exact `LooseSnapshot` is semantically necessary. What landed
      instead: the snapshot is now hash-assisted (see below) and the
      per-build `prenavnode_to_navnodes` `BTreeMap` was replaced with a
      dense `Vec<Option<Vec<..>>>` indexed by prenavnode graph index —
      `Option` preserves the old `contains_key` distinction between
      "not in this triangulation" and "in it with no navnodes".
      (c) ~~memoize `fixed_fingerprint`/`loose_snapshot` per
      `Drawing::generation`~~ — **analyzed and rejected**: the
      generation counter bumps on every band mutation, so it never
      survives between the calls that pay the scan (that is exactly why
      snapshots exist). What landed instead: `LooseSnapshot` carries an
      order-independent `u64` hash folded during construction that
      fast-rejects equality before the `BTreeMap` compare — the map
      compare remains authoritative on hash match, so collisions can
      only cost a wasted compare, never a false hit.
      `loose_snapshot_hash_never_overrides_exact_equality` pins both
      directions of that contract.

## A*-expansion / navcord-materialization coupling — analyzed

The hypothesized big win — decoupling g-score expansion from geometry
materialization so `visit_navnode` stops paying a `rework_path` per
visit (5.6M calls on candE) — turns out to be **already achieved** in
the ways that matter:

- The navcord only ever holds *virtual* canes during search; nothing
  materializes until the destination is popped and committed.
- `rework_path` is incremental: it diffs the current path against the
  target prefix, undoes only the divergent suffix, and redraws only
  the new tail — 5.6M calls, 820ms total on candE (~2% of runtime).
- Redrawn steps are further memoized: `step_memo` replays 5.2M of
  8.6M `step_to` calls on candE without re-running geometry.

The real cost attributed to "every A* expansion" is not rework
overhead but the `infringe` queries inside *genuine* probes — theta*'s
line-of-sight checks are semantically required and each verdict
depends on the exact head state (path prefix + materialized prefix),
so probe verdicts cannot be keyed any looser without changing the
algorithm. What landed instead:

- `ThetastarStepper` now checks `estimated_costs` *before*
  `visit_navnode`, so stale frontier pops (22.3K of 174K pops on
  candE, ~13%) skip path reconstruction and navcord rework entirely.
  Safe because the destination is never stale — finishing returns from
  inside `visit_navnode` before `estimated_costs` is set for it.
- `LooseSnapshot` hash fast-reject and the vector-backed
  `prenavnode_to_navnodes` map described above.

Remaining structural headroom is in `infringe` itself (13.2s of
`step_to`'s 22.1s): the shape predicates are already analytic (SAT
rect-rect, circle distances, `distance_to_point`), so the cost is
R-tree traversal — `SelectionIterator::next` 23.5% self. ~~Batched
candidate fetch~~ — **tried and reverted**: fetching one merged
envelope for a wrap's seg-infringement + collision + bend-infringement
queries drops ~4 traversals to ~1, but forfeits the lazy iterator's
early-exit — ~44% of `step_to`s fail and previously abandoned
unexplored subtrees on the first hit. On candE per-step cost went
2.6µs → 4.3µs (full batch) / 3.4µs (seg lazy + collision/bend batched)
and throughput dropped from 174K to 111K/134K pops in the 40s window.
A tighter envelope than `largest_clearance` inflation is the only
surviving idea here, and the earlier bbox-prune experiment showed the
envelope is already nearly tight.

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
      cheap; stale-pops now skip `visit_navnode` entirely — see the
      "A*-expansion coupling" section.)
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

## Routed-fraction ceiling on candE — selection coverage (fixed)

The 38/107 plateau was **not** a geometric ceiling: `selected_ratlines`
filters each ratline edge through `PinSelection::contains_node`, which
only resolved net membership on `Primitive` nodes. Compound (poly-pad)
ratvertices — most of candE's SMD endpoints — never produced a pin
selector, so their ratlines were silently excluded from every selection,
including `new_select_all`. Every flag-level lever measured the same
plateau for that reason: `--preprobe`, `--apply-shifts`, `--escape-stubs`,
`--multilayer` (90 s) and `--jobs 4` (60 s) all peaked at 38/2449.

Fixes:

- `contains_node` now resolves `maybe_net` on `NodeIndex::Compound` too,
  so net selections (and "all except no nets") cover pad endpoints.
- `--multilayer` selects via `new_select_all_except_nets(board, {})` —
  every net-bearing node on every layer, not just named pins.
- `MultilayerAutorouteExecutionStepper::new` drops ratlines whose fanout
  via cannot be placed on either the assigned or the preferred layer
  instead of `.expect`-panicking; failed attempts restore
  `terminating_dot_map` to a per-attempt snapshot (partial writes would
  otherwise retain dead `FixedDotIndex`es → recycled-compound panics).
- `PlanarAutorouteExecutionStepper::start_routing_ratline` skips
  `UnreachableOrigin`/`UnreachableDestination` ratlines with an empty
  aligned edit slot instead of erroring — previously one sealed endpoint
  aborted the whole command via the reconfigure retry loop.

Measured on candE (`--multilayer --nets GND`, 25 s, debug build):
107→237 ratlines attempted on the GND net alone; 53 of 2449 ratlines
routed at timeout vs the previous 38 best. GND unrouted fell 237→184.

Caveat: expanding the `--multilayer` attempt set to the whole board is
heavier — a full-board run probes far more candidates. Use `--nets` to
scope when memory/time is tight. One flaky `primitive_weight` panic
(stale index recycled to compound) was observed mid-run on astable but
did not reproduce on rerun — the known residual stale-verdict class
(documented under "Stale primitive index panic") likely retains edge
cases; the per-attempt `terminating_dot_map` snapshot above closes one
new instance of it.

## Branch handoff — commit map for upstreaming

43 commits sit ahead of `upstream/develop` on
`specctra-compat-and-cli-progress`. The branch mixes several logically
independent work streams; for upstream review they split most cleanly
as:

1. **Specctra/DSN compatibility + CLI plumbing** — `5c7165a`, `5292cc0`,
   `4607ef2`, `0a840c6`, `02155c1`, `e7c13c0`, `66ee52f`. Multi-via
   `use_via` lists, named-net selection, `--max-seconds`,
   `--max-reconfigurations`, `--suggest-shifts`. Self-contained;
   CLI-only except `0a840c6` (selection semantics).
2. **Micro-optimizations** — `fa5bf9d`, `ca6cb4a`, `0c4c533`,
   `e349b76`, `5d7c44a`, `11ac627`, `e449435`, `ea29d45`, `7724e8e`,
   `87cbaa3`, `5b8a0da`. Build/profile tuning, clearance caching,
   R-tree sizing, allocation cuts, test-profile speedup.
3. **GUI removal** — `7442858`. Drops egui in favour of the CLI;
   upstream-relevant only if the CLI-only direction is accepted.
4. **Negotiated congestion + multilayer scaffolding** — `daef858`,
   `95a4c5f`, `efa40a6`, `6674ab9`, `0d8a26e`, `5fcd64c`, `53bf5bc`,
   `6e1ad6e`. Obstacle penalties, clinch detection, component shifting,
   escape/corridor presorts, surgical rip-up, multilayer layer seeding
   and rotation.
5. **Navcord/navmesh performance + staleness correctness** — `b6de79e`,
   `f7b62c2`, `2663ff6`, `41263bf`, `f4c2df3`, `23e6535`, `14f4099`,
   `0c02cf6`, `5e99ae6`, `86ab47c`, `d441a94`, `7b310e9`, `217d087`,
   `dc7c7e3`, `8672c4e`. Cache infrastructure, retained search state,
   memo liveness guards, preprobe seeding, Arc-shared navmeshes,
   overlapee caching, and the selection/fail-fast fixes.
6. **Formatting normalization** — `a5ddace`. Tree-wide `cargo fmt`;
   will conflict mechanically with upstream — apply last or drop if
   upstream stays unformatted.

Caveats for upstreaming:

- `METAPLAN.md` is a working document — keep it in the fork or strip
  before upstream review.
- `TOPOLA_NO_SEARCH_STATE=1` remains as an A/B lever for the retained
  search-state machinery (documented under "Stale primitive index
  panic").
- Several commits bundle features with their tests; splitting further
  than the groups above requires rebasing interleaved dependencies and
  was judged not worth the risk to a green tree.
- Verification at HEAD: 49 lib tests, 28 unilayer and 3 multilayer
  integration tests all pass; `test_astable_multivibrator` is
  historically flaky (~3 min runtime) and passed on rerun.

## Multilayer layer-rotation starvation — diagnosed and fixed

User observation: moving candE from 2 to 4 copper layers gained almost
nothing (~10%, 48→53 routed on `--nets GND`), suspiciously little for
the added freedom. Investigation results:

**Machinery works.** `.ses` output already carries wires on all four
layers (F.Cu ~9.6K, In1.Cu ~1.1K, In2.Cu ~1.0K, B.Cu ~0.5K segments)
and fanout vias land. `Unreachable*` endpoint skips were ruled out for
the GND plateau — zero skip lines under `RUST_LOG`.

**Bug: the rotation trigger never fired.** The outer multilayer
reconfigurator's `TimeVsProgressAccumulatorTimeout` banks
`progress_bonus` seconds per *newly-reached ratline index*. With the
CLI's `progress_bonus: 0.5`, reaching index ~200 banks ~105 s — the
first layer rotation could not fire inside any realistic run.
`process_planar_result` dutifully pushed rotated configurations into
the A* on every planar reconfiguration; nothing ever popped them.
Zero "multilayer reconfigure" events in a 30 s instrumented run.

**Fix (this commit):**

- Stall-triggered escalation in `MultilayerAutorouteReconfigurator`:
  track the furthest ratline index reached (`costs.lengths.len()` of
  each planar status); after 4 planar reconfigurations without
  advancing it, pop a rotated-layer configuration. Empty A* no longer
  breaks the command — it falls through to normal stepping.
- CLI outer timeout `progress_bonus` 0.5 → 0.05 (budget ≈ 15 s at
  index 200 — the timeout remains as a backstop, not the driver).

Verified: 4 reconfigure events in a 90 s `--nets GND` run, each
relayering a ratline successfully (`1 changed, 1 relayered, 0
re-fanout failed`). Routed count held at 53 in that window — rotation
now engages but gains remain marginal on candE.

**Architectural ceiling (the real answer to "why didn't layers
help"):** `Navmesh::new_from_prenavmesh` asserts
`origin.layer() == destination.layer()` — every ratline routes
*entirely* on a single layer. Multilayer mode assigns a layer per
ratline plus fanout vias at the endpoints; the search space contains
no mid-route vias, so a connection cannot weave between layers to slip
through a corridor that is congested on all layers at different
points. On candE — 2185 prerouted wires and 361 vias filling all four
layers — the residual 2449 ratlines need a complete single-layer
corridor each, and the binding constraint is usually the layer-fixed
pad escape. Per-ratline relayering therefore buys ~10%, not 2×.

True 4-layer freedom requires vias as search edges (a 3D navmesh) —
that is the ng-router problem space, i.e. Phase E2's deferred work,
now with a concrete motivation.

## Mid-route via weaving — implemented (first 3D step)

Rather than rewriting the router core around a fused layered
navmesh, the first increment of true 3D routing is **via weaving**:
on a persistent planar stall, the stalled ratline is decomposed at a
fresh via into two dot-to-dot sub-routes on different layers —
parent → via on its layer, continuation → original destination on
another layer. Both halves reuse the existing planar A* machinery;
the via connects the chain in the ratsnest and in union-find
connectivity.

### Mechanism

- `Ratsnest::add_continuation(vertex, pos, target)` inserts a
  synthetic ratvertex + ratline edge at runtime; the continuation is
  a first-class `RatlineUid` spliced right after its parent in the
  ordering (`insert_ratline_after` in the reconfigurers' stored
  preconfigurations + the live configuration).
- Via sites are sampled **near the source end** of the failed
  corridor (1/8 → 1/2 span plus perpendicular jogs): the parent's
  first half must stay short to survive the congestion that stalled
  it; the continuation takes the long hop on the freer layer.
  `Board::add_via` is the feasibility oracle — it checks point and
  infringement constraints and self-cleans partial placements.
- The parent's destination endpoint is remapped
  (`terminating_dot_map`) at the via's source-layer dot; the
  continuation's entries carry its own destination fanout. A static
  map entry terminates its via-side endpoint on itself.
- Woven ratlines are **anchored** (`woven` set): relayering either
  half would reverse the weave via or rewrite the chaining remap —
  they are excluded from layer-rotation `changed` sets, and
  `adopt_plan_preserving_woven` re-injects their `layer_map`/static
  entries after a stale SearchNode plan snapshot is installed.

### Unweave — broken chains must be reversible

A weave that cannot complete either half would otherwise anchor the
net to a permanently broken chain, consuming the rotation option
that might have saved it. Every weave records a `WeaveRecord`
(parent → continuation, via vertex, via dots, the parent's pre-weave
destination remap). At each stall escalation, `broken_weaves`
checks every live weave for a missing half-band and
`unweave_ratline` rolls them back:

- both half-bands are ripped via the ordinary surgical rip-up,
- the continuation edge leaves the ratsnest, orderings, edit slots,
  and all terminating-dot maps (live configuration + both stored
  preconfigurations),
- the parent's destination remap is restored to its captured prior
  value,
- the via and the continuation's destination fanout **stay on the
  board** as dead-end same-net copper — measured necessity: an
  unrelated band may wrap the via dot as a bend core, and removing
  the dot orphaned such bends (`geometry.rs` `core()` panic on a
  seg of an untouched ratline). The residue is cosmetically untidy
  but electrically harmless.
- `weave_exhausted` marks unweaved parents: site sampling is
  deterministic, so a second weave would repeat the same failure —
  they go straight to rotation. This also bounds total weaves (≤ 1
  per ratline) for memory.

`install_configuration` now tolerates the *attempted* ratline being
absent from the new ordering (the continuation itself was removed)
instead of unwrapping into a panic.

### Pre-existing bugs the weave surfaced (fixed)

- **Fallback preconfiguration stale maps**: the negotiated-congestion
  reconfigurer's `SccPermutations` fallback keeps its own
  `preconfiguration` clone; `remap_terminating_dots`/
  `remove_terminating_dots` never synced it, so fallback emissions
  resurrected terminating dots of already-removed vias — the
  documented "stale primitive index" flake class, now deterministic
  under weave traffic. Both paths now sync the fallback too.
- **Non-SCC ratlines dropped from emitted orderings**: SCC
  permutation output filtered `preconfiguration.ratlines` through
  SCC membership; the synthetic continuation has no SCC and was
  silently omitted. Emitted orderings now retain non-SCC ratlines.

### Results

- `test_astable_multivibrator` passes with the full
  weave → unweave → rotation cycle exercised; 49 lib + 28 unilayer
  + 3 multilayer tests green.
- On candE the mechanism fired end-to-end (via placed, continuation
  enqueued, no stale-index panics); measurable routing gains not yet
  demonstrated — the stalls it fires on are dominated by sealed pad
  escapes, where no mid-route site helps and rotation is the right
  recovery anyway.

### Known limitations / next steps

- Via sites are sampled blind (fixed fractions of the corridor);
  a frontier-aware placement — using the failed route's search tree
  to pick the deepest reachable point — would hit cases where the
  corridor is open early but blocked later.
- The via geometry of unweaved attempts remains as same-net copper;
  a cleanup pass that removes only vias with no band *and* no
  dependent bend cores could tidy the residue.
- True fused-layer navmesh (vias as free-space search edges, Phase
  E2) remains the structurally complete answer; the weave is a
  ratline-level approximation layered on the existing engine.

## Critical-first iterative routing — implemented (opt-in `--critical-first`)

New outer stepper (`CriticalAutorouteExecutionStepper`) that wraps the
planar or multilayer inner router:

1. **Identify**: every selected unresolved ratline is probed on a
   *cloned* autorouter under relaxed clearances
   (`override_clearances(max(largest_clearance, band_width)/4)` —
   clearance 0 produces degenerate navmeshes). Scores combine relaxed
   failure, real-clearance blocker count, demand-path conflicts, and
   endpoint crowding; the top quantile + all relaxed failures form the
   critical set (capped at `max(#ratlines/4, 8)`).
2. **Reserve** (planar only): each critical ratline's demand corridor —
   the probe path, densified to band-width spacing, or the endpoint
   chord when the probe failed — is carved onto the board as same-net
   fixed geometry, invisible to its own net but a clearance obstacle to
   foreign bands. Carves are reverted on abort; dead ones are removed
   at completion unless a foreign band has wrapped them (load-bearing).
3. **Protect** (planar only): critical ratlines are ordered first and
   filtered out of `rip_up` lists in emitted reconfigurations; routed
   protected bands are pinned at the front of emitted orderings in
   *live* order so backtracing can never undo them, and unrouted
   protecteds are urgent-scheduled right after the current ratline. A
   desperation escape revokes protection when every rip candidate is
   protected (monotone shrinking — cannot deadlock).
4. **Re-identify** (planar only): every `reprobe_every`
   reconfigurations the still-unrouted set is re-probed and the
   hardest are promoted (per-round and total caps — uncapped
   promotion starved easy work), carved, protected, and spliced after
   the current position via a surgical reconfiguration commit.

Under `--multilayer` the inner is `MultilayerAutorouteReconfigurator`
but only the probe + congestion seeding apply — ordering, carving, and
protection measurably hurt the fanout plan's own scheduling (see
results below).

### Machinery built to make this possible

- `PlanarAutorouteReconfigurator::new_with_configuration_override` /
  `MultilayerAutorouteReconfigurator::new_with_planar_override`:
  install the critical-first ordering/carves/protected set on the
  freshly built preconfiguration before routing history begins.
- `PlanarAutorouteConfiguration.protected` + reconfigurator
  `apply_protection` transform + `set_protected`/`seed_congestion`
  plumbing into the reconfigurer's stored preconfigurations.
- `probe.rs`: `probe_one`/`probe_ratlines_dots` take explicit endpoint
  dots (terminating-dot-map-aware); parallel and sequential probe
  passes share the worker path.
- `carve.rs`: `carve_corridor` (densification, chord fallback, checked
  `add_fixed_*`), `edit_is_wrapped` (shared load-bearing check,
  refactored out of the multilayer weave-preservation code),
  `remove_dead_carves`.
- Bounded emit loop in `PlanarAutorouteReconfigurator::reconfigure`:
  an ordering differing only in the un-attempted tail is stashed and
  installed after 256 candidates, so a permutation search that never
  produces a backtrace-usable ordering cannot spin inside one `step()`
  (this was an upstream hang: the emit loop drained an effectively
  unbounded SCC-permutation A* in a single call).
- `Invoker::execute_with_timeout` catches step panics and finishes
  early, salvaging routed bands — latent geometry-graph bugs
  (stale node indices, `joints()` unreachable) otherwise lose all
  work on dense boards. The board may still be too corrupt to
  serialize; salvage is best-effort.
- `multilayer_reconfigurer`: `revise_ratline` skips ratlines absent
  from the layer map (continuation/unmapped ratlines — upstream panic).
- `criticality.rs`: scoring + `relaxed_probe_clearance`.

### Results (release, `--features metrics`, 60 s unless noted)

| board / mode | baseline unrouted | critical-first unrouted |
|---|---|---|
| `red_led_heart` planar | 149/176 | **132/176** |
| `red_led_heart` multilayer | 77/176 (30 s) | 77/176 (30 s) |
| `candE.dsn` planar (all nets) | 2365/2403 | **2352/2403** |

Multilayer is at parity: the native plan ordering + probe-derived
congestion seeding, with carving/protection/re-identification all
disabled for it (each was measured harmful — carved corridors starve
via weaving; the forced reorder and protected urgent-scheduling stall
the fanout schedule; re-probing every N reconfigurations is pure
overhead once nothing else consumes its output).

### Pre-existing upstream bugs found while benchmarking

- `PlanarAutorouteReconfigurator::reconfigure` / fallback emit loops
  could drain an unbounded permutation A* inside one `step()`,
  hanging past any timeout (heart baseline, clean HEAD).
- `multilayer_reconfigurer::revise_ratline` panicked on
  continuation/unmapped ratlines (`.unwrap()` on missing layer-map
  entry) — fixed.
- `geometry.rs` `joints()`/`update_edge` panics from stale node
  indices under heavy weave/rip-up traffic (both baseline and
  critical-first hit these on heart at larger budgets) — **root-caused
  and fixed** (see below).

### Geometry-graph corruption — root cause and fix

All typed indices (dots, segs, bends, compounds) share one flat
`StableGraph` node-index namespace, and `*_at_index` restorations use
`update_node`, which replaces the weight **in place while keeping the
occupant's edges**. Two compositional bugs followed:

1. **Slot recycling under reversible edits.** A recorded removal
   freed a slot that fresh allocation (e.g. a weave `add_via`
   compound) recycled; when the removal's reversal replayed, the
   `*_at_index` write swapped a different live node's weight under
   its edges — producing the `maybe_net`/`joints`/`primitive_weight`
   unreachable family. **Fix:** removals tombstone the slot
   (`GenericNode::Removed`) so it stays occupied and can never be
   recycled; `*_at_index` writes assert vacant-or-tombstone and
   `update_node` restores in place. `node_count` excludes tombstones.

2. **Cross-ledger endpoint lifetimes.** A band seg joined to a
   fanout-via dot could be ripped (recorded, reversible) and then the
   multilayer relayer removed the via: `edit_is_wrapped` only saw
   *live* geometry and missed the dead-but-restorable join, and the
   later backtrace replayed the join onto a tombstoned endpoint
   (provenance: `multilayer_autoroute.rs` surgical-relayer
   `old_edit.reverse()` → `backtrace_to_index`). **Fix:** every
   removal records its join endpoints in `Geometry::pending_joiners`;
   `edit_is_wrapped` treats endpoints with pending joiners as
   load-bearing, so the via is kept and the pending band rejoins it
   on restoration — which is also the semantically correct outcome.
   Entries for permanently-forgotten edits leak conservatively (the
   endpoint stays wrapped; cannot corrupt the graph).

Write-site invariants (`assert_vacant_slot`, `assert_dot_endpoint`,
liveness checks on `add_to_compound`/`reattach_bend`/double-removal)
turn any residual staleness into a descriptive failure at the
corrupting write instead of silent type confusion; setting
`TOPOLA_TOMBSTONE_DEBUG` names the call site that tombstoned a slot.

Also fixed alongside: the negotiated-congestion fallback's
duplicate-draining loop could spin for minutes inside one `step()`,
unreachable by `--max-seconds`. It is now bounded per call (4096
pops) and hands the last candidate back — the emit loop's own bound
absorbs duplicate emissions and the A* resumes next call, so the
deadline interrupts the grind and salvages routed bands.

Verification: heart multilayer `--critical-first` 120 s — previously
a deterministic panic — completes clean with valid `.ses` output;
baseline 60/90 s and candE 90 s likewise terminate with output.
95 tests green including the `wrapped_check_sees_pending_joiners`
regression test.
- **Fanout/weave vias used a hardcoded r=100 regardless of the
  declared padstack** (astable/heart/candE all declare 300 µm-radius
  vias). Correcting it unroutes `astable_multivibrator`'s D1-K —
  the fixture's success relied on phantom vias smaller than the real
  padstack. Reverted to preserve baseline behavior; a real fix needs
  fanout placement that searches for legal via sites rather than
  trusting the fake radius.

### Known limitations / next steps

- Critical-first is planar-first: on dense fanout-bound boards the
  probe's value is real but the physical carve currently fights the
  weave. Ideas: carve corridors as *preferred*, not forbidden space
  (soft penalty instead of fixed geometry), or carve only on layers
  the plan assigned.
- `critical_corridor` fixture: baseline oscillates 7–8/9 and never
  converges; critical-first stabilizes at ~6–9/9 depending on run —
  the board's capacity is borderline; better demonstration needs a
  fixture where the critical net *must* go first to be routed at all.
- Progress reporting still uses `curr_ratline_index`, which overstates
  completion when late reconfigurations rip bands back.
- `.cmd` replay determinism under `--critical-first` is wall-clock
  dependent (probe timeouts), like the existing oscillating-board
  tests.

## Performance instrumentation — implemented

The CLI now exposes deterministic benchmarking and per-step telemetry.
A run is a pure function of (board, options, seed, step budget) when
`--deterministic` is set; wall-clock `--max-seconds` remains available
for ordinary runs.

### Flags

| flag | effect |
|---|---|
| `--deterministic` | sets the progress-vs-time timeout accumulator to `{INFINITY, 0}` — reconfiguration becomes purely failure-driven; removes the machine-speed-dependent preemptive trigger |
| `--max-steps N` | machine-independent step budget; the run stops after N top-level `step()` calls |
| `--seed N` | sets both `shuffle_seed` and `layer_seed` |
| `--progress-csv FILE` | per-step time series (throttled: state changes + ~1 s heartbeat) |
| `--blockers-csv FILE` | blocker attribution pairs from negotiated-congestion reconfiguration |

`--jobs` workers get their own `--progress-csv` files suffixed with the
job index; sibling termination on first completion stays wall-clock
dependent, so golden comparisons use single-job deterministic runs.

### Progress CSV columns

`elapsed_s,step,kind,step_ms,position,total,unrouted,unconnected,tombstones,pending_joiners`

- `kind` is the `StepKind` of the step that produced the row:
  `route` (attempt in flight), `routed`, `skipped`, `planar-reconf`,
  `layer-reconf` (multilayer rotation/weave/relayer — distinct from
  inner-planar pass-throughs), `other`.
- `unrouted` counts ratlines without a `band_termseg`; `unconnected`
  counts electrically-open ratlines across principal layers. They
  diverge when a band exists but is a dangling weave half — a real
  failure mode, not a reporting bug (astable deterministic run: 4
  unrouted / 16 unconnected = 12 dangling bands).
- `tombstones` is `geometry.removed_count` — reserved slots awaiting
  possible reversal; `pending_joiners` counts removed-but-reversible
  attachments pinning endpoints.

### Interpreting the signals

- **Plateau in `position`/`unrouted`** = stall; correlate with
  `planar-reconf`/`layer-reconf` rows to see whether the router is
  working or idling inside one step.
- **Sawtooth in `unrouted`** = rip-up churn.
- **`step_nanos_max`** (metrics dump) vs the step-time budget = the
  deadline-lag signal — a single step that dwarfs `--max-seconds` is
  an uninterruptible hot spot (the pre-fix A\* drain showed up here).
- **Emit-loop `candidates`** spikes = the permutation drain — A\*
  re-popping duplicate orderings; bounded per call, resumable.
- **`dedup_truncations`** counts that bound firing; if it fires often
  the emit loop is wasting budget on duplicates.
- **`pending_joiners_held`** counts vias kept alive by pending joins —
  each is a would-be corruption under the pre-fix regime; a large
  value means heavy interleaving of weave/relayer edits with band
  edits.

### Reconfiguration counters (metrics dump)

`reconfiguration: planar N layer M` now reports honestly —
`MultilayerReconfigKind` distinguishes pass-through planar reconfigs,
weaves, and true layer rotations that previously all surfaced as
`Reconfigured(())`. Also emitted: `emit candidates`, `backtraced`
(distance + ratline count), `blockers`, `relayered`,
`refanout-fail`, `kept-wrapped`, `search_states stored/hits`.

### Determinism audit

- Seeds are hand-rolled and deterministic (splitmix64 for
  `layer_seed`, xorshift for `shuffle_seed`); no `rand`/`thread_rng`
  in `src/`.
- All `HashMap`/`HashSet` uses in decision paths are keyed lookups or
  membership sets; emitted configs use `BTreeSet`. No
  iteration-order leaks found.
- The one decision-affecting wall-clock read is
  `TimeVsProgressAccumulatorTimeout::update()`
  (`stepper.rs`); `--deterministic` neutralizes it. Other `Instant`
  reads are the outer deadline (by design) and display-only
  `timeout_progress` values.
- Verified: two identical `--deterministic --max-steps` heart runs
  produce identical step/kind CSV sequences and byte-identical `.ses`.
- `.cmd` replay remains wall-clock dependent for probe timeouts.

### Golden baselines — `bench/golden.tsv`

`scripts/bench.sh` runs a fixture×mode matrix under
`--deterministic --max-steps --seed 0`, captures
unrouted/unconnected/`.ses` SHA, and either compares against
`bench/golden.tsv` (`--check`) or rewrites it (`--write-golden`).
The `.ses` SHA makes trajectory regressions visible even when counts
match. Wall-clock rows are informational only.

Golden rows are seeded from the data already in this document and the
test suite: the critical-first results table, per-fixture test
expectations (astable 16/16, lm317 15/15, optoisolator 20/20), and
fresh deterministic measurements. `CANDE_DSN` env var supplies the
external candE board; missing fixtures are skipped.

Surprises surfaced by the deterministic matrix:

- **Critical-first loses under frozen timeouts**: heart 164 vs 148
  unrouted, candE 2383 vs 2365 — and candE cf costs 84–141 s vs ~4 s
  (carving + probing overhead with no salvageable win). Under
  wall-clock budgets cf looked neutral-to-better; the timeout
  accumulator was doing trajectory-steering work that masked this.
- **heart ml and mlcf produce byte-identical output** (same SHA) —
  the critical-first flag is a no-op through the multilayer path.
- **astable deterministic**: 4 unrouted / 16 unconnected vs 0/0
  wall-clock — frozen timeouts expose how much that fixture's success
  depends on preemptive reconfiguration.

### Profiling hotspots

For a stuck run: `sample <pid> 5` (already pinpointed the A\* drain),
`samply record`, or `cargo flamegraph` — the release profile has
debug info enabled.

## Simulated-annealing global pre-planner — implemented (opt-in `--anneal`)

A global SA pre-planner (`autorouter::anneal`) decides *where and in
what order* the existing router works — layer assignments, route
order, congestion seeds, corridor carves, and (continuous mode)
planned mid-route vias — then hands the plan to the multilayer
machinery. The router itself is untouched: `anneal` lives inside
`MultilayerAutorouteOptions`, so `.cmd` replay, undo/redo, and
`--jobs` all work unchanged. Deterministic by construction —
hand-rolled splitmix64/xorshift (no `rand` dep), iteration-bounded,
no wall-clock reads; congestion seeds are written absolutely so a
replay can't double-seed, and continuation bookkeeping is torn down
at command end (blocker-log `RatlineUid` refs purged).

Two modes behind `--anneal-mode`:

- `continuous` — per-ratline parametric 3D curves `(x,y,z)` under
  Metropolis acceptance with **bounded soft-core repulsion**
  `σ²/(d²+σ²)` (not the paper's divergent `1/d²`). Crossings are
  finite-energy events — curves pass through each other while hot,
  so homotopy changes are ordinary accepted moves, and the
  hard-core limit emerges from the schedule. Obstacle weight ramps
  0.1→1 over cooling; quantized z-run boundaries become planned
  vias handed off as weave-style continuations.
- `discrete` — SA over `(layer_map, order)` with corridor-conflict
  energy; emits the same `AnnealPlan` minus splits. Its `crossings`
  metric counts accepted relayer moves (accept/reject counters are
  continuous-only instrumentation).

### Benchmarks — `--deterministic --max-steps --seed 0`

| board | mode | unrouted | unconnected | vias | wall s | anneal | crossings | splits |
|---|---|---|---|---|---|---|---|---|
| astable (16 rl) | `--multilayer` | 4 | 16 | 6 | 0.2 | — | — | — |
| | `+cont` | 5 | 12 | 28 | 1.2 | 0.9 s | 22 | 5 |
| | `+disc` | 1 | 14 | 17 | 0.2 | 0.002 s | 110 | 0 |
| heart (176 rl) | (base) | 148 | 285 | — | 0.8 | — | — | — |
| | `--multilayer` | 165 | 310 | 32 | 0.4 | — | — | — |
| | `+cont` | 168 | 322 | 304 | 82.5 | 81.8 s | 672 | 74 |
| | `+disc` | 147 | 306 | 123 | 1.8 | 0.02 s | 1133 | 0 |
| candE (2403 rl) | (base) | 2365 | 7472 | 0 | 3.3 | — | — | — |
| | `--multilayer` | 2139 | 6661 | 485 | 127.6 | — | — | — |
| | `+cont` (60 iters) | *run below* | | | | | | |
| | `+disc` | *run below* | | | | | | |

Seed sweep (0–2, direct `--seed`): identical unrouted/unconnected
on both repo boards under both modes — plans converge to the same
family regardless of trajectory noise. `--jobs N` also composes
(3-worker astable discrete lands at the same 1/14).

### Findings

- **Discrete annealing is a small, nearly-free win.** Best
  unrouted count on astable (1 vs 4) and heart (147 vs 148 base /
  165 ml) for ~1–25 ms of pre-planning — ordering + congestion
  seeding + a less-greedy layer map, all cheap.
- **Continuous 3D annealing works mechanically but loses
  end-to-end.** Pass-through happens (22–672 accepted crossings —
  the soft-core does its job) and quantization is clean, yet
  outcomes are *worse* than plain `--multilayer` on both repo
  boards (5 vs 4; 168 vs 165) at 5–200× wall time.
- **Planned splits mostly never land.** `splits=74` on heart but
  only **2 continuations materialized** (ratline total 176→178);
  astable planned 5, placed ~0. On congested boards the annealed
  via sites infringe and `place_split` degrades to a plain layer
  assignment — so continuous mode's headline output (mid-route
  vias) contributes almost nothing; its practical effect is
  layer_map+order+congestion like the discrete mode, just slower
  and (here) worse.
- **Via inflation is real either way**: annealed runs emit 17–123
  vias vs 6–32 for the matching non-annealed mode — the layer_map
  forces fanouts the router wouldn't choose. (Continuous's 304 on
  heart is mostly fanout, not continuations.)
- **The kernel doesn't scale.** Per sweep: a full `system_energy`
  re-eval plus two repulsion evals per proposal; per-sweep cost
  grows superlinearly with curve×neighbor density — 0.9 s at 16
  ratlines, ~82 s at 176, and >22 CPU-min (killed unfinished) at
  2403. The candE golden row is therefore bounded to
  `--anneal-iterations 60`; continuous mode as written is a
  small-board experiment.
- **Verdict**: the pre-planner architecture earned its place, and
  the discrete mode is worth keeping on merit. Continuous mode
  stays behind its flag as a research path — realizing its promise
  needs incremental energy evaluation, split-site feasibility
  screening, and consolidation of z-wiggles, not more iterations.
