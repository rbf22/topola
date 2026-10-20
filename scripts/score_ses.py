#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Topola contributors
#
# SPDX-License-Identifier: MIT

"""Geometry-invariant scoring for routed Specctra output (.ses/.dsn wiring).

Because many valid routings of one board exist, output is judged on
invariants rather than geometry: wirelength, via usage, layer share,
bend structure, tortuosity, open ends. Run on our .ses and on the
reference ``_solved.dsn`` and the numbers are directly comparable.

Usage:
    score_ses.py FILE                  metrics for one .ses/.dsn
    score_ses.py FILE --ref SOLVED     side-by-side + ratios
    score_ses.py --sweep bench/out/sweep   per config x board vs reference

Net context: wires inherit the enclosing ``(net NAME`` scope; vias
carry their own ``(net`` child. Works for topola .ses and for
freerouting/KiCad wiring scopes alike.
"""

import bisect
import json
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dsnutil

RE_NET_SCOPE = re.compile(r"\(net\s+(\S+)")
RE_WIRE = re.compile(r"\(wire\b")
RE_VIA = re.compile(r"\(via\b")
RE_PATH = re.compile(r"\(path\s+(\S+)\s+(\S+)\s+([^)]*)\)")
RE_NET_REF = re.compile(r"\(net\s+([^()\s]+)\s*\)")


def iter_scopes(text, regex):
    for m in regex.finditer(text):
        yield m, dsnutil.scope_end(text, m.start())


def parse_board(path: Path):
    """Return {net: {wires:[(layer,width,pts)], vias:[(x,y)]}}."""
    text = path.read_text(errors="replace")
    nets = defaultdict(lambda: {"wires": [], "vias": []})

    # wire/via -> net: find the nearest enclosing (net scope. Cheap:
    # record byte ranges of every (net scope once.
    net_ranges = []
    for m, end in iter_scopes(text, RE_NET_SCOPE):
        name = m.group(1).strip('"')
        net_ranges.append((m.start(), end, name))
    net_starts = [r[0] for r in net_ranges]

    def owner(pos):
        # innermost enclosing (net scope: last range that starts before
        # pos; verify pos actually falls inside it (leaf (net "x") refs
        # inside wires span no wires, so the check is required).
        i = bisect.bisect_right(net_starts, pos) - 1
        while i >= 0:
            start, end, name = net_ranges[i]
            if pos < end:
                return name
            i -= 1
        return None

    for m, end in iter_scopes(text, RE_WIRE):
        scope = text[m.start() : end]
        pm = RE_PATH.search(scope)
        if not pm:
            continue
        layer, width, coords = pm.group(1), float(pm.group(2)), pm.group(3)
        vals = [float(v) for v in coords.split()]
        pts = list(zip(vals[0::2], vals[1::2]))
        # two net-association styles: freerouting .dsn puts (net "n")
        # inside the wire scope; .ses net_out scopes enclose the wires.
        nm = RE_NET_REF.search(scope)
        net = (
            nm.group(1).strip('"') if nm else (owner(m.start()) or "?")
        )
        nets[net]["wires"].append((layer, width, pts))

    for m, end in iter_scopes(text, RE_VIA):
        scope = text[m.start() : end]
        vm = re.search(
            r"\(via\s+(?:\"[^\"]*\"|\S+)\s+(-?[\d.]+)\s+(-?[\d.]+)", scope
        )
        if not vm:
            continue
        x, y = float(vm.group(1)), float(vm.group(2))
        nm = RE_NET_REF.search(scope)
        net = nm.group(1).strip('"') if nm else (owner(m.start()) or "?")
        nets[net]["vias"].append((x, y))

    return nets


def metrics(nets):
    total_len = 0.0
    per_layer = defaultdict(float)
    per_net_len = defaultdict(float)
    via_count = sum(len(n["vias"]) for n in nets.values())
    bends = {"0-5": 0, "45": 0, "90": 0, "135": 0, "other": 0}
    open_ends = 0
    stub_len = 0.0
    tort = []

    for net, n in nets.items():
        # group segments by layer; each wire path is a polyline — split
        # into edges for bend detection across consecutive wires.
        net_len = 0.0
        adj = defaultdict(list)  # endpoint -> [(other_end, length, layer)]
        extremes = []
        for layer, width, pts in n["wires"]:
            for i in range(len(pts) - 1):
                a, b = pts[i], pts[i + 1]
                seg = math.hypot(b[0] - a[0], b[1] - a[1])
                net_len += seg
                per_layer[layer] += seg
                adj[a].append((b, seg))
                adj[b].append((a, seg))
            extremes.extend([pts[0], pts[-1]])

        per_net_len[net] += net_len
        total_len += net_len

        # open ends: degree-1 endpoints not at a via position
        via_pts = set(n["vias"])
        for p, links in adj.items():
            if len(links) == 1 and p not in via_pts:
                open_ends += 1
                stub_len += links[0][1]

        # bends at degree-2 vertices on the same layer
        for p, links in adj.items():
            if len(links) != 2 or p in via_pts:
                continue
            (b1, l1), (b2, l2) = links
            v1 = (b1[0] - p[0], b1[1] - p[1])
            v2 = (b2[0] - p[0], b2[1] - p[1])
            m1, m2 = math.hypot(*v1), math.hypot(*v2)
            if m1 == 0 or m2 == 0:
                continue
            cos = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (m1 * m2)))
            deg = math.degrees(math.acos(cos))
            change = 180.0 - deg
            if change < 5:
                bends["0-5"] += 1
            elif abs(change - 45) <= 11:
                bends["45"] += 1
            elif abs(change - 90) <= 11:
                bends["90"] += 1
            elif abs(change - 135) <= 11:
                bends["135"] += 1
            else:
                bends["other"] += 1

        # tortuosity: net path length / span of its endpoints
        if len(extremes) >= 2 and net_len > 0:
            span = max(
                math.hypot(a[0] - b[0], a[1] - b[1])
                for i, a in enumerate(extremes)
                for b in extremes[i + 1 :]
            )
            if span > 0:
                tort.append(net_len / span)

    tort.sort()
    nbends = sum(bends.values())
    return {
        "nets": len(nets),
        "wirelength": round(total_len),
        "wirelength_per_layer": {k: round(v) for k, v in sorted(per_layer.items())},
        "vias": via_count,
        "vias_per_net": round(via_count / max(1, len(nets)), 2),
        "bends": nbends,
        "bend_angles": bends,
        "bend_per_1k_len": round(nbends / max(1e-9, total_len) * 1000, 1),
        "open_ends": open_ends,
        "stub_len": round(stub_len),
        "tortuosity_med": round(tort[len(tort) // 2], 2) if tort else 0,
        "tortuosity_p90": round(tort[int(len(tort) * 0.9)], 2) if tort else 0,
        "per_net_len": {k: round(v) for k, v in per_net_len.items()},
    }


def compare(ours: dict, ref: dict) -> dict:
    out = {"ours": ours, "ref": ref, "delta": {}}
    # deltas are only meaningful when the reference substantially covers
    # the board; many fixtures ship nearly-empty wiring scopes.
    out["ref_coverage"] = (
        round(ref["nets"] / ours["nets"], 2) if ours["nets"] else 0
    )
    for k in ("wirelength", "vias", "bends", "open_ends", "stub_len", "tortuosity_med"):
        o, r = ours[k], ref[k]
        out["delta"][k] = round((o - r) / r, 2) if isinstance(r, (int, float)) and r else None
    # per-net wirelength ratio restricted to nets routed in BOTH — the
    # honest version of the wirelength comparison, immune to incomplete
    # boards (unrouted nets contribute no length on either side).
    joint = [
        o_len / r_len
        for n, o_len in ours["per_net_len"].items()
        if (r_len := ref["per_net_len"].get(n)) and r_len > 0 and o_len > 0
    ]
    joint.sort()
    out["delta"]["wlen_joint_nets"] = len(joint)
    out["delta"]["wlen_joint_med"] = (
        round(joint[len(joint) // 2], 2) if joint else None
    )
    return out


def main() -> None:
    args = sys.argv[1:]

    if args[0] == "--sweep":
        sweep_dir = Path(args[1])
        ref_cache = {}
        rows = []
        for ses in sorted(sweep_dir.glob("*.ses")):
            board = ses.name.split("__")[0]
            if board not in ref_cache:
                ref = next(
                    Path("tests/external").glob(f"*/{board}/{board}_solved.dsn"),
                    None,
                )
                ref_cache[board] = (
                    (str(ref), metrics(parse_board(ref)))
                    if ref and ref.is_file()
                    else (None, None)
                )
            ref_path, ref_m = ref_cache[board]
            ours = metrics(parse_board(ses))
            row = {"file": ses.name, "metrics": ours}
            if ref_m:
                cmp = compare(ours, ref_m)
                row["ref"] = ref_path
                row["delta"] = cmp["delta"]
                row["ref_coverage"] = cmp["ref_coverage"]
            rows.append(row)
        print(json.dumps(rows, indent=1))
        return

    ours = metrics(parse_board(Path(args[0])))
    if "--ref" in args:
        ref = metrics(parse_board(Path(args[args.index("--ref") + 1])))
        print(json.dumps(compare(ours, ref), indent=1))
    else:
        print(json.dumps(ours, indent=1))


if __name__ == "__main__":
    main()
