#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Topola contributors
#
# SPDX-License-Identifier: MIT

"""Vendor one freerouting .dsn fixture as a problem/solved pair.

Usage:
    scripts/strip_wiring.py <src.dsn> <slug> <outdir>

Writes ``<outdir>/<slug>.dsn`` (wiring scope emptied — the routing
problem) and ``<outdir>/<slug>_solved.dsn`` (the fixture with its
embedded solution kept). Both are normalised for the specctra-core
parser quirks documented in tests/external/README.md.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dsnutil


def main() -> None:
    src, slug, outdir = Path(sys.argv[1]), sys.argv[2], Path(sys.argv[3])
    text = src.read_text(errors="replace")
    problem = dsnutil.write_fixture(text, slug, outdir)
    print(f"{problem} (+ _solved.dsn)")


if __name__ == "__main__":
    main()
