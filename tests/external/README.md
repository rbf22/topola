# External board fixtures

Third-party boards used as routing tests/benchmarks. Each directory keeps
the unrouted problem (`*.dsn`) next to its known-good routed solution for
comparison.

## `optoisolator_4n35/` — PCBench (MIT)

- Source: https://github.com/PCBench/PCBench →
  `PCBs/4N35-TTL-Serial-Optoisolator_4N35-TTL-Serial-Optoisolator`
- `optoisolator_4n35_solved.kicad_pcb` — upstream `processed.kicad_pcb`
  (contains the human routing; MIT per its `metadata.json`)
- `optoisolator_4n35.dsn` — Specctra DSN exported after deleting all
  tracks/vias
- `optoisolator_4n35_solved.dsn` — same export with routing kept
- Regenerate:
  ```bash
  PY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
  $PY - <<'EOF'
  import pcbnew
  b = pcbnew.LoadBoard("optoisolator_4n35_solved.kicad_pcb")
  pcbnew.ExportSpecctraDSN(b, "optoisolator_4n35_solved.dsn")
  for t in list(b.GetTracks()):
      b.Remove(t)
  pcbnew.ExportSpecctraDSN(b, "optoisolator_4n35.dsn")
  EOF
  ```
  (`kicad-cli` ≥10 no longer has a `specctra-dsn` export subcommand; the
  pcbnew Python API is the alternative. `b.GetTracks()` misbehaves in some
  KiCad versions — stripping `(segment …)`/`(via …)` lines from the
  `.kicad_pcb` text works too.)

## `fasttest/` — Freerouting fixture

- Source: `Issue313-FastTest.dsn` from the freerouting fixture corpus
  (mirror: https://github.com/dilawar/dsn-parser `dsn-files/`).
  Freerouting fixtures are user-submitted issue reproductions under the
  freerouting repo's GPL — check before distributing.
- `fasttest_solved.dsn` — upstream file unchanged; its `(wiring …)` scope
  is the embedded solution (all `protect` type)
- `fasttest.dsn` — same file with an empty `(wiring)` scope (specctra-core
  requires the scope to exist); this is the routing problem
- Regenerate: replace the `(wiring …)` scope in `fasttest_solved.dsn` with
  `(wiring)`.

## `pcbench/` — PCBench corpus subset (MIT/permissive)

- Source: https://github.com/PCBench/PCBench (`PCBs/`), pinned commit
  `dec3be75cbdef74787625f9043c7391cd473bb64` — fetched by
  `scripts/fetch_corpora.sh` into the gitignored `bench/corpora-src/`.
- 16 boards selected from ~1180 for spread of layer count, net count and
  density — see `bench/corpus.tsv`. Only permissively licensed upstream
  boards (MIT / Apache-2.0 / BSD / CC0 / CC-BY / LGPL) are vendored;
  per-board license/author lives in each upstream `metadata.json`.
- `<slug>.dsn` is the routing problem (tracks stripped from
  `processed.kicad_pcb`), `<slug>_solved.dsn` the human reference
  routing; `.kicad_pcb` originals kept alongside.
- Regenerate: `ROW_FILTER=<slug> scripts/vendor_corpus.sh` (uses
  `scripts/kicad_to_dsn.py` under KiCad's pcbnew python; the board must
  load in the installed KiCad — a few PCBench boards are unloadable in
  KiCad 10 and were excluded during curation).

## `freerouting/` — freerouting fixture corpus subset (GPL)

- Source: `fixtures/` in https://github.com/freerouting/freerouting,
  pinned commit `1be66ad2797395f3a258a7f3b9b31c724a624867` — fetched by
  `scripts/fetch_corpora.sh` (sparse checkout).
- 15 boards selected from ~160 `IssueNNN-*.dsn` reproductions for
  diversity (2/4/6 layers, 24–764 nets, dense + sparse).
- `<slug>.dsn` is the original fixture with its `(wiring …)` scope
  emptied; `<slug>_solved.dsn` keeps the embedded freerouting solution.
- Regenerate: `ROW_FILTER=<slug> scripts/vendor_corpus.sh`
  (`scripts/strip_wiring.py` + `scripts/dsnutil.py` — also normalises
  the pcb name on line 1, renames empty class names, and ensures a
  `(wiring)` scope exists; see quirks below).
- License caveat still applies: fixtures are user submissions under the
  freerouting GPL umbrella — `GPL-3.0-only` in REUSE.toml; don't
  redistribute outside this repo without checking.

## Parser quirks hit while importing (kept for future imports)

- `specctra-core` learns `string_quote`/`space_in_quoted_tokens` from the
  `(parser …)` scope, which comes *after* the pcb name — a quoted pcb name
  containing spaces (e.g. `Issue103-Board-Unrouted.dsn`) fails to parse.
  Workaround: unquote/rename the name on line 1.
- Multi-path `(boundary …)` scopes (panelized boards, e.g.
  `Issue034-Green14SegLED.dsn`) are rejected — single path only.
- `(class` scopes with an empty name (`Issue179-*.dsn`) are rejected.
- `(wiring` is a required scope — strip contents, not the scope itself.
