#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Topola contributors
#
# SPDX-License-Identifier: MIT

"""Aggregate blocker-census .blockers.csv files into composition tables.

Usage: blocker_report.py [bench/out/blocker-census]

For every *.blockers.csv in the directory, classifies each recorded
obstacle as fixed geometry (FixedDot/FixedSeg/FixedBend: pads, board
features — not displaceable) or routed geometry (LoneLooseSeg/
SeqLooseSeg/LooseBend: bands the router placed — candidates for
rip-up). Joins with results.tsv for unrouted counts. Prints one
markdown table per census mode, sorted by unrouted.
"""

import csv
import glob
import os
import sys
from collections import Counter

FIXED = {"FixedDot", "FixedSeg", "FixedBend"}
ROUTED = {"LoneLooseSeg", "SeqLooseSeg", "LooseBend"}


def obstacle_kind(raw: str) -> str:
    return raw.split("(", 1)[0]


def mode_slug(modes: str) -> str:
    # mirrors `tr ' -' '__'` in blocker_census.sh
    return "census" if not modes.strip() else modes.strip().translate(str.maketrans({" ": "_", "-": "_"}))


def load_results(out_dir: str) -> dict:
    """(slug, mode_slug) -> (unrouted, total) from census results.tsv."""
    unrouted = {}
    path = os.path.join(out_dir, "results.tsv")
    if not os.path.exists(path):
        return unrouted
    with open(path) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            parts = line.rstrip("\n").split("\t")
            if len(parts) < 7:
                continue
            dsn, modes, unr, tot = parts[0], parts[1], parts[4], parts[6]
            slug = os.path.splitext(os.path.basename(dsn))[0]
            unrouted[(slug, mode_slug(modes))] = (unr, tot)
    return unrouted


def main() -> int:
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "bench/out/blocker-census"
    unrouted = load_results(out_dir)

    # group blocker csvs by census mode
    per_mode = {}
    for path in sorted(glob.glob(os.path.join(out_dir, "*.blockers.csv"))):
        stem = os.path.basename(path).replace(".blockers.csv", "")
        slug, _, mode = stem.partition("__")
        per_mode.setdefault(mode, []).append((slug, path))

    for mode in sorted(per_mode):
        print(f"\n### mode: {mode}\n")
        print("| board | unrouted | events | failed nets | blockers | fixed % | routed % | top routed blockers |")
        print("|---|---|---|---|---|---|---|---|---|")
        rows = []
        for slug, path in per_mode[mode]:
            fixed = routed = other = 0
            events, nets = set(), set()
            routed_blockers = Counter()
            with open(path, newline="") as f:
                for row in csv.DictReader(f):
                    kind = obstacle_kind(row["obstacle"])
                    if kind in FIXED:
                        fixed += 1
                    elif kind in ROUTED:
                        routed += 1
                        if row["blocker_net"]:
                            routed_blockers[row["blocker_net"]] += 1
                    else:
                        other += 1
                    events.add((row["position"], row["failed_uid"]))
                    nets.add(row["failed_net"])
            total = fixed + routed + other
            unr, tot = unrouted.get((slug, mode), ("-", "-"))
            rows.append((slug, unr, tot, len(events), len(nets), total, fixed, routed, routed_blockers))

        rows.sort(key=lambda r: -(int(r[1]) if str(r[1]).isdigit() else -1))
        for slug, unr, tot, nev, nnet, total, fixed, routed, rb in rows:
            fpct = f"{100*fixed/total:.0f}%" if total else "-"
            rpct = f"{100*routed/total:.0f}%" if total else "-"
            top = ", ".join(f"{net}×{n}" for net, n in rb.most_common(3)) or "-"
            print(f"| {slug} | {unr}/{tot} | {nev} | {nnet} | {total} | {fpct} | {rpct} | {top} |")
    return 0


if __name__ == "__main__":
    sys.exit(main())
