#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Topola contributors
#
# SPDX-License-Identifier: MIT

"""Pivot bench/out/sweep/results.tsv into a board x config table.

Usage:
    scripts/sweep_report.py [results.tsv] [--scores scores.json]

``--scores`` merges geometry-invariant quality metrics produced by
``scripts/score_ses.py --sweep bench/out/sweep > scores.json`` into the
best-config listing (wirelength/via deltas vs the solved reference,
tortuosity, open ends).

Prints:
  - per-board x config unrouted counts (per-seed cells as "s0/s7")
  - best config per board (min unrouted, then min unconnected, then min wall)
  - per-class recommended default (config winning the most boards)
"""

import argparse
import json
from collections import defaultdict
from pathlib import Path

SHORT = {
    "": "base",
    "--corridor-presort": "corr",
    "--escape-presort": "esc",
    "--surgical-ripup": "surg",
    "--critical-first": "crit",
    "--preprobe": "probe",
    "--escape-stubs": "stubs",
    "--pad-entry": "pad",
    "--multilayer": "ml",
    "--multilayer --anneal --anneal-mode continuous": "ml+annC",
    "--multilayer --anneal --anneal-mode discrete": "ml+annD",
    "--multilayer --critical-first": "ml+crit",
    "--multilayer --surgical-ripup": "ml+surg",
    "--corridor-presort --surgical-ripup": "corr+surg",
    "--critical-first --corridor-presort": "crit+corr",
    "--multilayer --critical-first --surgical-ripup": "ml+crit+surg",
    "--escape-presort --escape-stubs --multilayer": "esc+stubs+ml",
    "--multilayer --anneal --anneal-mode continuous --anneal-carve-top-k 16": "ml+annC+carve",
    "--critical-first --multilayer --anneal --anneal-mode discrete": "crit+ml+annD",
}

CLASS_OF = {}


def short(mode: str) -> str:
    return SHORT.get(mode, mode.replace("--", "").replace(" ", "+")[:12])


def slug_of(dsn: str) -> str:
    return Path(dsn).stem


def load_manifest(root: Path) -> None:
    manifest = root / "bench" / "corpus.tsv"
    if not manifest.is_file():
        return
    for line in manifest.read_text().splitlines()[1:]:
        if not line or line.startswith("#"):
            continue
        parts = line.split("\t")
        if len(parts) >= 7:
            CLASS_OF[parts[0]] = parts[6]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("results", nargs="?", default="bench/out/sweep/results.tsv")
    ap.add_argument(
        "--scores",
        help="JSON from 'score_ses.py --sweep DIR' to merge quality metrics",
    )
    args = ap.parse_args()
    path = Path(args.results)
    root = Path(__file__).resolve().parent.parent
    load_manifest(root)

    scores = {}
    if args.scores:
        for row in json.loads(Path(args.scores).read_text()):
            scores[Path(row["file"]).stem] = row

    rows = []
    for line in path.read_text().splitlines():
        if not line or line.startswith("#"):
            continue
        f = line.split("\t")
        if len(f) < 10:
            continue
        rows.append({
            "dsn": f[0], "mode": f[1], "budget": f[2], "seed": f[3],
            "unrouted": f[4], "unconnected": f[5], "total": f[6],
            "sha": f[7], "wall": f[8], "status": f[9],
        })
    if not rows:
        print("no results")
        return

    boards = sorted({slug_of(r["dsn"]) for r in rows})
    configs = []
    for r in rows:
        c = r["mode"]
        if c not in configs:
            configs.append(c)
    seeds = sorted({r["seed"] for r in rows})

    cell = defaultdict(dict)  # cell[slug][(mode,seed)] = row
    for r in rows:
        cell[slug_of(r["dsn"])][(r["mode"], r["seed"])] = r

    def num(v):
        try:
            return int(v)
        except ValueError:
            return 10**9

    # ---- pivot table (unrouted / total)
    hdr = [f"{c:>10}" for c in (short(m) for m in configs)]
    print(f"{'board':<22}{'tot':>5}{''.join(hdr)}")
    for b in boards:
        total = next((r["total"] for (m, s), r in cell[b].items() if r["total"] != "-"), "-")
        line = f"{b:<22}{total:>5}"
        for m in configs:
            parts = []
            for s in seeds:
                r = cell[b].get((m, s))
                if r is None:
                    parts.append("·")
                elif r["status"] != "ok":
                    parts.append("PANIC"[:6])
                else:
                    parts.append(r["unrouted"])
            line += f"{'/'.join(parts):>10}"
        print(line)

    # ---- best config per board
    print("\nbest config per board (min unrouted, then unconnected, then wall_s):")
    class_win = defaultdict(lambda: defaultdict(int))
    for b in boards:
        best = None
        for m in configs:
            r = cell[b].get((m, seeds[0]))
            if r is None or r["status"] != "ok":
                continue
            try:
                wall = float(r["wall"])
            except (ValueError, TypeError):
                wall = 1e9
            key = (num(r["unrouted"]), num(r["unconnected"]), wall)
            if best is None or key < best[0]:
                best = (key, m, r)
        if best is None:
            print(f"  {b:<22} no ok runs")
            continue
        cls = CLASS_OF.get(b, "?")
        class_win[cls][best[1]] += 1
        line = (f"  {b:<22} {best[1]:<60} unrouted={best[2]['unrouted']} "
                f"unconn={best[2]['unconnected']} wall={best[2]['wall']}s [{cls}]")
        stem = b + best[1].translate(str.maketrans(" -", "__")) + "__s" + seeds[0]
        sc = scores.get(stem)
        if sc:
            d = sc.get("delta", {})
            mt = sc["metrics"]

            def pct(k):
                v = d.get(k)
                return f"{int(v * 100):+d}%" if v is not None else "  -"

            line += f" | wlen{pct('wirelength')}"
            jm, jn = d.get("wlen_joint_med"), d.get("wlen_joint_nets", 0)
            if jm is not None:
                line += f" jwlen{jm:.2f}x(n={jn})"
            line += f" vias{pct('vias')} bends{pct('bends')}"
            if sc.get("ref_coverage", 1) < 0.3:
                line += "*"
            line += f" tort={mt['tortuosity_med']} open={mt['open_ends']}"
        print(line)

    # ---- per-class recommendation
    print("\nrecommended default per class (most board wins at seed %s):" % seeds[0])
    for cls in sorted(class_win):
        winner = max(class_win[cls].items(), key=lambda kv: kv[1])
        print(f"  {cls:<24} {winner[0]}  ({winner[1]} board win(s))")


if __name__ == "__main__":
    main()
