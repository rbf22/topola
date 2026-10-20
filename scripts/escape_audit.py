#!/usr/bin/env python3
"""Escape-channel audit for a QFN pad field.

For each perimeter pad of a reference footprint, marches outward along
the pad's escape axis (perpendicular to the field edge, away from the
component center) and reports the distance to the first obstructing
copper primitive on each routing layer, plus the number of trace
channels the field's perimeter corridor can carry before that
obstruction.

The deliverable is a lower-bound diagnostic: pins whose escape ray is
blocked within `via_diameter` of the pad edge cannot fan out, and pins
blocked within a couple of trace widths cannot route on that layer at
all — a geometric ceiling independent of the router.
"""

import math
import re
import sys
from dataclasses import dataclass, field

# --- minimal kicad_pcb s-expression reader ---------------------------------

def tokenize(text):
    return re.findall(r'\(|\)|[^\s()"]+|"[^"]*"', text)

def parse(tokens):
    stack = [[]]
    for tok in tokens:
        if tok == "(":
            node = []
            stack[-1].append(node)
            stack.append(node)
        elif tok == ")":
            stack.pop()
        else:
            stack[-1].append(tok.strip('"'))
    return stack[0]

def find_all(node, head):
    return [n for n in node if isinstance(n, list) and n and n[0] == head]

def child(node, head):
    for n in node:
        if isinstance(n, list) and n and n[0] == head:
            return n
    return None

# --- geometry ----------------------------------------------------------------

@dataclass
class Rect:
    cx: float
    cy: float
    w: float
    h: float
    net: str = "?"
    name: str = "?"

    def dist_to_point(self, px, py):
        dx = max(abs(px - self.cx) - self.w / 2, 0.0)
        dy = max(abs(py - self.cy) - self.h / 2, 0.0)
        return math.hypot(dx, dy)

@dataclass
class Disc:
    cx: float
    cy: float
    r: float
    net: str = "?"

    def dist_to_point(self, px, py):
        return max(math.hypot(px - self.cx, py - self.cy) - self.r, 0.0)

@dataclass
class Seg:
    """A routed segment: centerline capsule of half-width `r`."""
    x1: float
    y1: float
    x2: float
    y2: float
    r: float
    net: str = "?"

    def dist_to_point(self, px, py):
        vx, vy = self.x2 - self.x1, self.y2 - self.y1
        L2 = vx * vx + vy * vy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - self.x1) * vx + (py - self.y1) * vy) / L2))
        return max(math.hypot(px - (self.x1 + t * vx), py - (self.y1 + t * vy)) - self.r, 0.0)

    def ray_reach(self, sx, sy, ux, uy, chan_half):
        """Distance along ray until this capsule enters the corridor."""
        # coarse: sample the centerline densely enough for 0.5mm pitch
        vx, vy = self.x2 - self.x1, self.y2 - self.y1
        L = math.hypot(vx, vy)
        n = max(2, int(L / 0.25) + 1)
        best = None
        for i in range(n + 1):
            px, py = self.x1 + vx * i / n, self.y1 + vy * i / n
            rel = (px - sx) * ux + (py - sy) * uy
            lat = abs((px - sx) * uy - (py - sy) * ux)
            if rel > 0 and lat - self.r <= chan_half:
                reach = rel - self.r
                if best is None or reach < best:
                    best = reach
        return best

def rot(px, py, deg):
    r = math.radians(deg)
    return px * math.cos(r) - py * math.sin(r), px * math.sin(r) + py * math.cos(r)

def footprint_pads(fp):
    """Absolute-position pads of a footprint on F.Cu with their nets."""
    at = child(fp, "at")
    fx, fy = float(at[1]), float(at[2])
    frot = float(at[3]) if len(at) > 3 else 0.0
    fside = child(fp, "layer")
    flipped = fside is not None and len(fside) > 1 and "B." in fside[1]
    nets = {}
    for net_node in find_all(fp, "net"):
        nets[net_node[1]] = net_node[2].strip('"') if len(net_node) > 2 else "?"
    out = []
    for pad in find_all(fp, "pad"):
        if len(pad) < 3 or pad[2] not in ("smd", "thru_hole"):
            continue
        layers = child(pad, "layers")
        lnames = layers[1:] if layers else []
        on_front = any(str(l).startswith("F.") or l == "*.Cu" for l in lnames)
        on_back = any(str(l).startswith("B.") or l == "*.Cu" for l in lnames)
        if flipped:
            on_front, on_back = on_back, on_front
        if not (on_front or on_back):
            continue
        pat = child(pad, "at")
        psize = child(pad, "size")
        px, py = float(pat[1]), float(pat[2])
        prot = float(pat[3]) if len(pat) > 3 else 0.0
        w, h = float(psize[1]), float(psize[2])
        if flipped:
            px = -px
        ax, ay = rot(px, py, frot)
        total = (frot + prot) % 180
        if abs(math.sin(math.radians(total))) > 0.5:
            w, h = h, w
        net = child(pad, "net")
        netname = nets.get(net[1], "?") if net else "?"
        out.append(
            dict(
                n=pad[1], cx=fx + ax, cy=fy + ay, w=w, h=h,
                net=netname, front=on_front, back=on_back,
            )
        )
    return out

def main(path, refdes, chan_w, clearance):
    tree = parse(tokenize(open(path).read()))[0]

    pads = []
    for fp in find_all(tree, "footprint"):
        ref = None
        for prop in find_all(fp, "property"):
            if len(prop) > 1 and prop[1] == "Reference":
                ref = prop[2]
        for p in footprint_pads(fp):
            p["ref"] = ref
            pads.append(p)

    # vias and tracks already on the board obstruct identically
    obstacles = []
    for seg in find_all(tree, "segment"):
        net = child(seg, "net")
        netname = net[1] if net else "?"
        at = child(seg, "start")
        en = child(seg, "end")
        w = float(child(seg, "width")[1])
        layer = child(seg, "layer")
        front = layer is not None and str(layer[1]).startswith("F.")
        obstacles.append(
            (front, Seg(float(at[1]), float(at[2]), float(en[1]), float(en[2]), w / 2, netname))
        )
    for via in find_all(tree, "via"):
        at = child(via, "at")
        sz = child(via, "size")
        net = child(via, "net")
        d = Disc(float(at[1]), float(at[2]), float(sz[1]) / 2, net[1] if net else "?")
        obstacles.append((True, d))
        obstacles.append((False, d))

    target = [p for p in pads if p["ref"] == refdes and p["front"]]
    others = [p for p in pads if p["ref"] != refdes and (p["front"] or p["back"])]

    ccx = sum(p["cx"] for p in target) / len(target)
    ccy = sum(p["cy"] for p in target) / len(target)
    print(f"{refdes}: {len(target)} F.Cu pads, field center ({ccx:.2f},{ccy:.2f})")

    blocked_short = 0
    results = []
    for p in target:
        dx, dy = p["cx"] - ccx, p["cy"] - ccy
        # escape axis: dominant component of outward radial direction
        if abs(dx) >= abs(dy):
            ux, uy = (1.0 if dx > 0 else -1.0), 0.0
        else:
            ux, uy = 0.0, (1.0 if dy > 0 else -1.0)
        # start the ray at the pad's outer edge
        edge = max(p["w"], p["h"]) / 2
        sx, sy = p["cx"] + ux * (p["w"] / 2), p["cy"] + uy * (p["h"] / 2)
        first = None
        for front, ob in obstacles + [(True, Rect(o["cx"], o["cy"], o["w"], o["h"], o["net"])) for o in others]:
            if not front:
                continue
            if isinstance(ob, Seg):
                reach = ob.ray_reach(sx, sy, ux, uy, chan_w / 2)
                if reach is not None:
                    reach -= clearance
            else:
                rel = ((ob.cx - sx) * ux + (ob.cy - sy) * uy)
                if rel <= 0:
                    continue
                lat = abs((ob.cx - sx) * uy - (ob.cy - sy) * ux)
                ext = (ob.w if isinstance(ob, Rect) else 2 * ob.r) / 2
                if lat - ext > chan_w / 2:
                    continue
                reach = rel - ext - clearance
            if reach is not None and (first is None or reach < first):
                first = reach
        results.append((p["n"], p["net"], first))
        if first is not None and first < 0.85:
            blocked_short += 1

    print(f"\npad -> first obstruction along escape axis (mm, clearance {clearance}):")
    by_net = {}
    for n, net, d in results:
        by_net.setdefault(net, []).append((n, d))
    for net in sorted(by_net):
        ds = by_net[net]
        dists = ", ".join(f"{n}:{'∞' if d is None else f'{d:.2f}'}" for n, d in ds)
        print(f"  {net:<14} {dists}")

    free = sum(1 for _, _, d in results if d is None or d >= 0.85)
    print(f"\n{free}/{len(results)} pads have ≥0.85mm free escape on F.Cu; "
          f"{blocked_short} blocked inside via-diameter reach")
    missing = len(pads) and len([p for p in pads if p["ref"] == refdes])
    if missing != len(target):
        print(f"({missing - len(target)} pads are backside/flipped, excluded)")

    buckets = {"<0.25 (sealed at pad edge)": 0, "0.25–0.5": 0,
               "0.5–0.85 (no via fits)": 0, "0.85–2": 0, "≥2 / ∞": 0}
    for _, _, d in results:
        if d is None or d >= 2:
            buckets["≥2 / ∞"] += 1
        elif d >= 0.85:
            buckets["0.85–2"] += 1
        elif d >= 0.5:
            buckets["0.5–0.85 (no via fits)"] += 1
        elif d >= 0.25:
            buckets["0.25–0.5"] += 1
        else:
            buckets["<0.25 (sealed at pad edge)"] += 1
    print("\nescape-reach histogram:")
    for k, v in buckets.items():
        print(f"  {k:<30} {v}")

if __name__ == "__main__":
    # trace 0.25 + 2x clearance 0.127 ≈ 0.504mm channel
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "U1",
         float(sys.argv[3]) if len(sys.argv) > 3 else 0.504,
         float(sys.argv[4]) if len(sys.argv) > 4 else 0.127)
