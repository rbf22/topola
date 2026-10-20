#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Topola contributors
#
# SPDX-License-Identifier: MIT

"""Convert a PCBench ``processed.kicad_pcb`` board into a .dsn fixture pair.

Must run under KiCad's bundled python (pcbnew):

    PY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/\
Versions/Current/bin/python3
    $PY scripts/kicad_to_dsn.py <processed.kicad_pcb> <slug> <outdir>

Writes into ``<outdir>``:

- ``<slug>.dsn``               — Specctra problem export (tracks stripped)
- ``<slug>_solved.dsn``        — Specctra export with the human routing kept
- ``<slug>_unrouted.kicad_pcb``— trackless board the problem export came from
- ``<slug>_solved.kicad_pcb``  — copy of the input board (reference)

Track stripping is done on the .kicad_pcb text (balanced-paren removal of
(segment (via (arc scopes)) rather than board.GetTracks(), which drops
items silently on some KiCad versions.
"""

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dsnutil

import pcbnew

RE_TRACK = re.compile(r"\((?:segment|via|arc)\s")


def strip_tracks(kicad_pcb_text: str) -> str:
    out = []
    pos = 0
    for m in RE_TRACK.finditer(kicad_pcb_text):
        end = dsnutil.scope_end(kicad_pcb_text, m.start())
        out.append(kicad_pcb_text[pos : m.start()])
        pos = end
    out.append(kicad_pcb_text[pos:])
    return "".join(out)


def export(board_path: Path, dsn_path: Path, slug: str) -> None:
    board = pcbnew.LoadBoard(str(board_path))
    if board is None:
        raise RuntimeError(f"pcbnew could not load {board_path}")
    pcbnew.ExportSpecctraDSN(board, str(dsn_path))
    text = dsn_path.read_text(errors="replace")
    dsn_path.write_text(dsnutil.normalize(text, slug))


def main() -> None:
    src, slug, outdir = Path(sys.argv[1]), sys.argv[2], Path(sys.argv[3])
    outdir.mkdir(parents=True, exist_ok=True)

    solved_pcb = outdir / f"{slug}_solved.kicad_pcb"
    unrouted_pcb = outdir / f"{slug}_unrouted.kicad_pcb"

    text = src.read_text(errors="replace")
    solved_pcb.write_text(text)
    unrouted_pcb.write_text(strip_tracks(text))

    export(solved_pcb, outdir / f"{slug}_solved.dsn", slug)
    export(unrouted_pcb, outdir / f"{slug}.dsn", slug)
    print(f"{outdir / (slug + '.dsn')} (+ _solved.dsn, .kicad_pcb pair)")


if __name__ == "__main__":
    main()
