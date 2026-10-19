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

## Parser quirks hit while importing (kept for future imports)

- `specctra-core` learns `string_quote`/`space_in_quoted_tokens` from the
  `(parser …)` scope, which comes *after* the pcb name — a quoted pcb name
  containing spaces (e.g. `Issue103-Board-Unrouted.dsn`) fails to parse.
  Workaround: unquote/rename the name on line 1.
- Multi-path `(boundary …)` scopes (panelized boards, e.g.
  `Issue034-Green14SegLED.dsn`) are rejected — single path only.
- `(class` scopes with an empty name (`Issue179-*.dsn`) are rejected.
- `(wiring` is a required scope — strip contents, not the scope itself.
