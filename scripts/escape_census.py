#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Topola contributors
#
# SPDX-License-Identifier: MIT

"""Escape-vs-corridor census: for every corpus board, runs topola with
--suggest-shifts and classifies each recorded channel blocker by its
arc-length position along the channel path. Blockers in the first/last
ESCAPE_FRACTION of the path are 'escape' failures (endpoint region);
the rest are 'corridor' failures. Reports per-board counts plus the
channel coverage (unrouted ratlines with/without any blocker).

Usage: escape_census.py [-j N] [-f FILTER] [--analyze-only]
JSONs land in bench/out/escape-census/<slug>.json; analysis reads them.
"""

import concurrent.futures
import json
import math
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BIN = os.path.join(ROOT, "target/release/topola")
OUT = os.path.join(ROOT, "bench/out/escape-census")
MANIFEST = os.path.join(ROOT, "bench/corpus.tsv")
ESCAPE_FRACTION = 0.15


def corpus_boards(row_filter=""):
    boards = []
    with open(MANIFEST) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            slug, source, src, layers, nets, budget = line.split("\t")[:6]
            if row_filter and row_filter not in slug:
                continue
            dsn = os.path.join(ROOT, "tests/external", source, slug, f"{slug}.dsn")
            boards.append((slug, dsn, int(budget)))
    return boards


def run_board(slug, dsn, budget):
    jpath = os.path.join(OUT, f"{slug}.json")
    cmd = [BIN, dsn, "--deterministic", "--max-steps", str(budget),
           "--seed", "0", "--suggest-shifts", jpath,
           "-o", os.path.join(OUT, f"{slug}.ses")]
    r = subprocess.run(cmd, capture_output=True, text=True)
    return slug, r.returncode


def closest_frac(path, pt):
    """Arc-length parameter of pt's closest point on the polyline."""
    total = sum(math.dist(path[i], path[i + 1]) for i in range(len(path) - 1))
    if total == 0:
        return 0.0, math.dist(path[0], pt)
    best_t, best_d, acc = 0.0, float("inf"), 0.0
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        seg = math.dist(a, b)
        if seg == 0:
            continue
        ax, ay = a
        bx, by = b
        px, py = pt
        t = max(0.0, min(1.0, ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / seg**2))
        d = math.dist((ax + t * (bx - ax), ay + t * (by - ay)), pt)
        if d < best_d:
            best_d, best_t = d, (acc + t * seg) / total
        acc += seg
    return best_t, best_d


def analyze(slug):
    jpath = os.path.join(OUT, f"{slug}.json")
    if not os.path.exists(jpath):
        return None
    d = json.load(open(jpath))
    esc = corr = 0
    with_blockers = 0
    for c in d["channels"]:
        if not c["blockers"]:
            continue
        with_blockers += 1
        seen = set()
        for b in c["blockers"]:
            key = b["primitive"]
            if key in seen:
                continue
            seen.add(key)
            t, _ = closest_frac(c["path"], b["center"])
            if t < ESCAPE_FRACTION or t > 1 - ESCAPE_FRACTION:
                esc += 1
            else:
                corr += 1
    return len(d["channels"]), with_blockers, esc, corr


def main():
    jobs = 4
    row_filter = ""
    analyze_only = "--analyze-only" in sys.argv
    args = iter(a for a in sys.argv[1:] if not a.startswith("--"))
    for a in args:
        if a == "-j":
            jobs = int(next(args))
        elif a == "-f":
            row_filter = next(args)

    boards = corpus_boards(row_filter)
    os.makedirs(OUT, exist_ok=True)

    if not analyze_only:
        with concurrent.futures.ThreadPoolExecutor(jobs) as ex:
            futs = [ex.submit(run_board, *b) for b in boards]
            for f in concurrent.futures.as_completed(futs):
                slug, rc = f.result()
                if rc != 0:
                    print(f"{slug}: run failed rc={rc}", file=sys.stderr)

    print("| board | unrouted channels | with blockers | escape | corridor | escape % |")
    print("|---|---|---|---|---|---|")
    te = tc = 0
    for slug, _, _ in boards:
        r = analyze(slug)
        if not r:
            continue
        nch, nb, esc, corr = r
        te += esc
        tc += corr
        pct = f"{100*esc/(esc+corr):.0f}%" if esc + corr else "-"
        print(f"| {slug} | {nch} | {nb} | {esc} | {corr} | {pct} |")
    print(f"| **total** | | | {te} | {tc} | {100*te/(te+tc):.0f}% |" if te + tc else "")


if __name__ == "__main__":
    main()
