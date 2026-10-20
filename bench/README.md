# Benchmark corpus & parameter sweeps

This directory holds the benchmark harness for comparing autorouter
configurations across a curated corpus of real boards. The corpus is a
diverse subset (32 boards) drawn from two large public collections:

- **PCBench** (`tests/external/pcbench/`) — community KiCad designs with
  human-routed references, converted to `.dsn` via KiCad's pcbnew API.
- **Freerouting fixtures** (`tests/external/freerouting/`) — issue-driven
  Specctra `.dsn` files; most ship their own routed `(wiring)` scope,
  vendored as `<slug>_solved.dsn` alongside the emptied problem `.dsn`.

Sources are fetched into `bench/corpora-src/` (gitignored) by
`scripts/fetch_corpora.sh`, indexed into `bench/corpus_index.tsv` by
`scripts/corpus_index.py`, and the selection is recorded in
`bench/corpus.tsv` (manifest: slug, source, layers, ratlines, budget,
class, notes).

## Running a sweep

```sh
# phase 1: every single-flag config x every manifest row x SEEDS
SEEDS=0 scripts/sweep.sh

# phase 2: winner combinations (edit PHASE2_CONFIGS in scripts/sweep.sh)
PHASE=2 SEEDS="0 7" scripts/sweep.sh

# subset / extras
ROW_FILTER=wave scripts/sweep.sh
EXTRA_DSNS="/path/to/candE.dsn" scripts/sweep.sh
JOBS=4 scripts/sweep.sh
```

Every run is deterministic (`--deterministic --max-steps <budget>
--seed <s>`) and writes `<slug>__<config>__s<seed>.{ses,progress.csv,
blockers.csv,log}` plus a `rows/<name>.tsv` result row into
`bench/out/sweep/` (gitignored). Wall-clock figures are informational
only — compare runs by unrouted/unconnected and session hash.

## Reporting

```sh
# rescore all sessions (geometry-invariant metrics + gap to reference)
python3 scripts/score_ses.py --sweep bench/out/sweep > bench/out/sweep/scores.json

# pivot + best-config-per-board + per-class recommendations
python3 scripts/sweep_report.py --scores bench/out/sweep/scores.json
```

`score_ses.py FILE [--ref SOLVED]` works on a single session too.

## Metrics and how to read them

Completeness (from results.tsv): `unrouted`/`unconnected`/`total` —
primary ranking. The `_solved.dsn` references prove most nets *are*
routable, so a persistent unrouted set is a router shortfall, not luck.

Geometry-invariant quality (from scores.json; compared against the
reference's own numbers, never path equality — many valid routings
exist):

| metric | reading |
|---|---|
| `wlen` Δ | total wirelength vs reference — biased by incompleteness |
| `jwlen Nx (n=k)` | median per-net wirelength ratio on nets routed in BOTH — the honest version; <1 means shorter than the reference on those nets |
| `vias` Δ / `vias_per_net` | layer-transition usage; large negative = under-viaing vs reference |
| `wirelength_per_layer` | which layers carry the routing |
| `bends` + `bend_angles` | bend count and angle histogram — humans are ~all 45°/90°; our cane segmentation shows as a large `0-5` (micro-jog) bucket |
| `tortuosity_med/p90` | path length / endpoint span; wandering shows >1.5 |
| `open_ends` / `stub_len` | degree-1 dead ends — proxy for incomplete/abandoned bands |
| `ref_coverage` | fraction of routed nets covered by the reference; `*` marks deltas from a nearly-empty reference |

Failure classification (cross-config consensus):

- net unrouted under some configs → **search/ordering** problem:
  presorts, critical-first, seeds, `--jobs`, annealing.
- net unrouted under *all* configs, recurring `.blockers.csv` entries →
  **structural**: `--suggest-shifts`, escape-stubs, band-width, or a
  genuine corridor deficit.
- fails even under relaxed probing, or panics → **engine/fixture bug**
  (e.g. lvds2tmds' spade `locate` panic in `PrenavmeshBases`).
- reference routed a net via constructs we can't express (planes,
  `(type protect)`) → **fixture artifact**; document, don't chase.

## Validated defaults (phase-2, seeds {0,7}, post-fix binary)

Global default, measured across all 32 boards (mean ratio to each
board's best-known result, seed 0):

```
--multilayer --anneal --anneal-mode discrete     ratio 1.03, best on 17 boards, worst on 0
```

Adding `--preprobe` is a wash globally (ratio 1.03, 16 wins) but wins on
dense boards — treat it as a dense-class variant, not the default.
Everything else is ≥3× off best on average; `--critical-first` and
`--anneal-mode continuous --anneal-carve-top-k 16` are worst-tier and
should stay opt-in only.

Per-class winners (corpus is 1–2 boards per class — provisional):

- **dense** (any size): `… --anneal-mode discrete --preprobe`
- **THT-heavy** (relaymodule, vc4000multirom, avionics_hub):
  `--multilayer --escape-stubs`
- **sparse analog / odd-net**: `--corridor-presort` or plain
  `anneal-mode discrete`
- **hardest large boards** (poncho_fpga, prog_rig, reworkctf,
  smoothieboard, sensors_96boards, tairakb): `--multilayer
  --surgical-ripup` beats anneal on raw unrouted, at a quality cost.

Flags measured net-negative as defaults:

- `--escape-stubs`: eager stubs are *fixed* geometry — un-rippable and
  a permanent foreign-net constraint. +199 unrouted on prog_rig; helps
  only on sparse THT pads. `--lazy-escape-stubs` (stub only on demand,
  in the stepper) is the safe variant — baseline-identical where stubs
  don't pay. See FINDINGS.md I17/F27/F28.
- `--critical-first`: worst-tier mean ratio (7.3×).
- `--navmesh-step-factor 8`: safe, small win (−10 on logicboard); the
  cap never bound on 3/5 measured boards — failures there are genuine,
  not starvation.

Quality/completion trade-off (A7, vs `_solved.dsn` references):
anneal-discrete completes the most ratlines but its joint-net
wirelength is ~1.86× the reference median — the worst of any config.
Completion-first is still the right default (unrouted dominates the
metrics), but expect longer traces; `--critical-first` routes shorter
on the nets it completes.

## Golden rows

`bench/golden.tsv` is the committed regression set — one validated
config per corpus board plus the historical in-repo rows, regenerated
post-fix by `scripts/bench.sh --write-golden` and replayed by
`--check`. The per-board configs are the phase-2 winners documented
above (issue208 substitutes `anneal-mode discrete` for its winning
`continuous` row — 24 min wall is impractical for a regression gate).

Columns: `board, mode, steps, seed, unrouted, unconnected, total,
ses_sha256, wall_s, status` — the regression check compares only
`unrouted`, `unconnected`, `ses_sha256`. The trailing six columns
(`vias, segs, wlen, ref_vias, ref_segs, ref_wlen`) are informational
I6 quality counters from `wire_stats` in `scripts/bench_lib.sh`:
wire-segment count, via placements, and polyline wirelength for our
`.ses` and the adjacent `<stem>_solved.dsn` reference (`-` when
absent). Caveats:

- `wlen`/`ref_wlen` are in each file's raw coordinate units — Topola
  never rescales on load and corpus `(resolution)` labels are
  unreliable (freerouting writes `mil 1000` on mm-scale files), so a
  `wlen`/`ref_wlen` pair is comparable *within* a row, never across
  boards.
- `segs` counts `(wire …)` blocks — our writer emits one block per
  navmesh edge; freerouting writes long polylines. Expect ~3–10×
  inflation here even when the routing is equivalent; it's a
  segmentation metric, not a quality verdict by itself.
- Some references are degenerate comparators: `relaymodule_solved.dsn`
  is via-only (`22|0|0`), several boards have no `_solved` at all.
- For real quality analysis use `scripts/score_ses.py` (per-net
  joint-length ratios, bend histograms, tortuosity) — the golden
  columns are the cheap row-level signal, cross-checked against it.
