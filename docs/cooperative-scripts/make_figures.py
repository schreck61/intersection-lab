#!/usr/bin/env python3
# Copyright (c) 2026 Jim Schreckengast
# SPDX-License-Identifier: MIT OR Apache-2.0
"""Draw the reference cooperative crossing and its approach zones as SVG, and check
the geometry claims made in the notes (conflict counts, which movements conflict).

Usage: python3 make_figures.py [output-dir]
The default output directory is ../images/cooperative next to this script, where
the notes look for plan-view.svg and approach-zones.svg. Needs Python 3.8 or later
and nothing outside the standard library."""
import itertools
import math
import os
import sys

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "images", "cooperative"))
os.makedirs(OUT, exist_ok=True)

LW = 3.5      # lane width (m)
MED = 2.0     # half median (m)
NIN = 4       # in-box entry lanes per leg: L, T1, T2, T3
HALF = MED + NIN * LW   # leg half-width in the box area = 16 m
STOP = HALF + 2.0       # stop lines 2 m back from the crossing road = 18 m
R_LEFT = STOP + MED + 0.5 * LW   # left-turn radius = 21.75 m
R_SLIP = 25.0                    # slip-lane radius (m)
SLIP_OFF = HALF + 0.5 * LW       # slip lane center offset = 17.75 m
SLIP_START = SLIP_OFF + R_SLIP   # where the slip lane leaves the leg = 42.75 m
EXT = 78.0                       # half extent of the drawing (m)
SC = 6.0                         # px per m
W_PX = 2 * EXT * SC
C_PX = EXT * SC

LEGS = {"N": (0, 1), "E": (1, 0), "S": (0, -1), "W": (-1, 0)}


def px(p):
    return (C_PX + SC * p[0], C_PX - SC * p[1])


def right_of(d):
    return (d[1], -d[0])


def left_of(d):
    return (-d[1], d[0])


def add(a, b, k=1.0):
    return (a[0] + k * b[0], a[1] + k * b[1])


def inbound_dir(leg):
    o = LEGS[leg]
    return (-o[0], -o[1])


def leg_with_outward(v):
    for k, o in LEGS.items():
        if o == v:
            return k
    raise KeyError(v)


def lane_point(leg, lane, dist, outbound=False):
    """Center of a lane at `dist` meters from the center along the leg."""
    o = LEGS[leg]
    d = inbound_dir(leg)
    side = left_of(d) if outbound else right_of(d)
    off = MED + (lane + 0.5) * LW
    return add((o[0] * dist, o[1] * dist), side, off)


# ---------------------------------------------------------------- paths
paths = {}  # name -> list of points (m)
for leg in LEGS:
    d = inbound_dir(leg)
    opp = leg_with_outward(d)
    for i in (1, 2, 3):
        a = lane_point(leg, i, STOP)
        b = lane_point(opp, i, STOP, outbound=True)
        paths[f"{leg}_T{i}"] = [a, b]
    # left: quarter circle, center on the left of the heading
    e0 = lane_point(leg, 0, STOP)
    c = add(e0, left_of(d), R_LEFT)
    exit_leg = leg_with_outward(left_of(d))
    x0 = lane_point(exit_leg, 0, STOP, outbound=True)
    a0 = math.atan2(e0[1] - c[1], e0[0] - c[0])
    a1 = math.atan2(x0[1] - c[1], x0[0] - c[0])
    while a1 < a0:
        a1 += 2 * math.pi
    pts = [(c[0] + R_LEFT * math.cos(a0 + (a1 - a0) * t / 60), c[1] + R_LEFT * math.sin(a0 + (a1 - a0) * t / 60))
           for t in range(61)]
    paths[f"{leg}_L"] = pts
    assert abs(math.dist(pts[-1], x0)) < 1e-6, (leg, pts[-1], x0)

# slip lanes (outside the box)
slips = {}
for leg in LEGS:
    d = inbound_dir(leg)
    s0 = lane_point(leg, NIN, SLIP_START)          # lane 4 where it leaves the leg
    c = add(s0, right_of(d), R_SLIP)
    exit_leg = leg_with_outward(right_of(d))
    x0 = lane_point(exit_leg, NIN, SLIP_START, outbound=True)
    a0 = math.atan2(s0[1] - c[1], s0[0] - c[0])
    a1 = math.atan2(x0[1] - c[1], x0[0] - c[0])
    while a1 > a0:
        a1 -= 2 * math.pi
    pts = [(c[0] + R_SLIP * math.cos(a0 + (a1 - a0) * t / 40), c[1] + R_SLIP * math.sin(a0 + (a1 - a0) * t / 40))
           for t in range(41)]
    assert abs(math.dist(pts[-1], x0)) < 1e-6, (leg, pts[-1], x0)
    slips[leg] = (pts, c)


def seg_intersections(p, q):
    out = []
    for (a, b) in zip(p, p[1:]):
        for (c, d) in zip(q, q[1:]):
            den = (b[0] - a[0]) * (d[1] - c[1]) - (b[1] - a[1]) * (d[0] - c[0])
            if abs(den) < 1e-12:
                continue
            t = ((c[0] - a[0]) * (d[1] - c[1]) - (c[1] - a[1]) * (d[0] - c[0])) / den
            u = ((c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0])) / den
            if 0 <= t <= 1 and 0 <= u <= 1:
                out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
    # de-duplicate points found on shared segment ends
    uniq = []
    for p_ in out:
        if all(math.dist(p_, q_) > 0.2 for q_ in uniq):
            uniq.append(p_)
    return uniq


crossings = []
movement_conflicts = set()
for (na, pa), (nb, pb) in itertools.combinations(paths.items(), 2):
    pts = seg_intersections(pa, pb)
    for p_ in pts:
        crossings.append((na, nb, p_))
    if pts:
        ma = na.split("_")[0] + ("L" if na.endswith("_L") else "T")
        mb = nb.split("_")[0] + ("L" if nb.endswith("_L") else "T")
        movement_conflicts.add(tuple(sorted((ma, mb))))

kinds = {"TT": 0, "LT": 0, "LL": 0}
for na, nb, _ in crossings:
    k = "".join(sorted(("L" if na.endswith("_L") else "T") + ("L" if nb.endswith("_L") else "T")))
    kinds[k] += 1
print(f"crossing conflict points in the box: {len(crossings)}  {kinds}")
# movement names by direction of travel: leg N inbound = southbound, etc.
travel = {"N": "SB", "E": "WB", "S": "NB", "W": "EB"}
mv = sorted({m for pair in movement_conflicts for m in pair})
nice = lambda m: travel[m[0]] + "-" + m[1]
for m in mv:
    nb_ = sorted(nice(b if a == m else a) for a, b in movement_conflicts if m in (a, b))
    print(f"  {nice(m)} conflicts with {', '.join(nb_)}")
adj = {m: set() for m in mv}
for a, b in movement_conflicts:
    adj[a].add(b)
    adj[b].add(a)
tri = [t for t in itertools.combinations(mv, 3) if all(y in adj[x] for x, y in itertools.combinations(t, 2))]
quad = [t for t in itertools.combinations(mv, 4) if all(y in adj[x] for x, y in itertools.combinations(t, 2))]
print(f"  triangles: {len(tri)}: " + "; ".join("{" + ", ".join(nice(m) for m in t) + "}" for t in tri))
print(f"  4-cliques: {len(quad)}")
# opposite lefts?
print("  opposite lefts conflict?", ("NL", "SL") in movement_conflicts, ("EL", "WL") in movement_conflicts)
print(f"  left-turn radius {R_LEFT:.2f} m -> {math.sqrt(2.6*R_LEFT)*3.6:.0f} km/h; slip radius {R_SLIP} m ->"
      f" {math.sqrt(2.6*R_SLIP)*3.6:.0f} km/h; box {2*STOP:.0f} m across")

# ---------------------------------------------------------------- SVG helpers
def poly(points, **kw):
    s = " ".join(f"{x:.1f},{y:.1f}" for x, y in (px(p) for p in points))
    attrs = " ".join(f'{k.replace("_", "-")}="{v}"' for k, v in kw.items())
    return f'<polygon points="{s}" {attrs}/>'


def pline(points, **kw):
    s = " ".join(f"{x:.1f},{y:.1f}" for x, y in (px(p) for p in points))
    attrs = " ".join(f'{k.replace("_", "-")}="{v}"' for k, v in kw.items())
    return f'<polyline points="{s}" fill="none" {attrs}/>'


def line(a, b, **kw):
    (x1, y1), (x2, y2) = px(a), px(b)
    attrs = " ".join(f'{k.replace("_", "-")}="{v}"' for k, v in kw.items())
    return f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" {attrs}/>'


def text(p, s, size=13, anchor="middle", weight="normal", fill="#222", rotate=0):
    x, y = px(p)
    rot = f' transform="rotate({rotate} {x:.1f} {y:.1f})"' if rotate else ""
    return (f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" text-anchor="{anchor}" '
            f'font-weight="{weight}" fill="{fill}" dominant-baseline="middle"{rot}>{s}</text>')


ROAD = "#c9c9c9"
ISLAND = "#dfe8d3"
GRASS = "#eef2e6"
LINE = "#ffffff"
YEL = "#e0b000"
THRU = "#2b6cb0"
LEFT = "#d1631b"
SLIP = "#2f855a"
CONF = "#c53030"

svg = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W_PX:.0f} {W_PX + 120:.0f}" '
       f'font-family="Helvetica, Arial, sans-serif">',
       f'<rect x="0" y="0" width="{W_PX:.0f}" height="{W_PX + 120:.0f}" fill="#ffffff"/>',
       f'<rect x="0" y="0" width="{W_PX:.0f}" height="{W_PX:.0f}" fill="{GRASS}"/>']

# Slip-lane pavement (drawn first, under the legs)
for leg, (pts, c) in slips.items():
    outer = [add(c, (p[0] - c[0], p[1] - c[1]), (R_SLIP + LW / 2) / R_SLIP) for p in pts]
    inner = [add(c, (p[0] - c[0], p[1] - c[1]), (R_SLIP - LW / 2) / R_SLIP) for p in pts]
    svg.append(poly(outer + inner[::-1], fill=ROAD))

# Legs: full width beyond the slip diverge, in-box width inside it
for leg, o in LEGS.items():
    d = inbound_dir(leg)
    r = right_of(d)
    full = MED + (NIN + 1) * LW
    def rect(d0, d1, half):
        a = add((o[0] * d0, o[1] * d0), r, half)
        b = add((o[0] * d1, o[1] * d1), r, half)
        c_ = add((o[0] * d1, o[1] * d1), r, -half)
        e = add((o[0] * d0, o[1] * d0), r, -half)
        return [a, b, c_, e]
    svg.append(poly(rect(SLIP_START, EXT + 1, full), fill=ROAD))
    svg.append(poly(rect(0, SLIP_START + 0.5, HALF), fill=ROAD))

# The box
svg.append(poly([(-STOP, -STOP), (STOP, -STOP), (STOP, STOP), (-STOP, STOP)], fill=ROAD))
svg.append(poly([(-STOP, -STOP), (STOP, -STOP), (STOP, STOP), (-STOP, STOP)], fill="none",
                stroke="#7a7a7a", stroke_width=1.2, stroke_dasharray="6 4"))

# Medians and lane lines
for leg, o in LEGS.items():
    d = inbound_dir(leg)
    r = right_of(d)
    a = (o[0] * STOP, o[1] * STOP)
    b = (o[0] * (EXT + 1), o[1] * (EXT + 1))
    svg.append(poly([add(a, r, MED - 0.3), add(b, r, MED - 0.3), add(b, r, -(MED - 0.3)), add(a, r, -(MED - 0.3))],
                    fill=ISLAND, stroke=YEL, stroke_width=1.5))
    for side in (1, -1):
        for k in range(1, NIN + 1):
            off = MED + k * LW
            start = STOP if k < NIN else SLIP_START  # the outermost line starts at the slip lane
            p0 = (o[0] * start, o[1] * start)
            svg.append(line(add(p0, r, side * off), add(b, r, side * off),
                            stroke=LINE, stroke_width=1.3, stroke_dasharray="8 10"))
    # stop line across the entry lanes
    svg.append(line(add(a, r, MED), add(a, r, HALF), stroke=LINE, stroke_width=3))

# Corner islands with sensor masts
for leg, (pts, c) in slips.items():
    inner = [add(c, (p[0] - c[0], p[1] - c[1]), (R_SLIP - LW / 2 - 0.4) / R_SLIP) for p in pts]
    # island: inner edge of the slip lane closed against the two legs' edges
    corner_x = math.copysign(HALF + 0.4, c[0])
    corner_y = math.copysign(HALF + 0.4, c[1])
    island = inner + [(corner_x, corner_y)]
    svg.append(poly(island, fill=ISLAND, stroke="#9aa88a", stroke_width=1))
    mast = (math.copysign(STOP + 4.5, c[0]), math.copysign(STOP + 4.5, c[1]))
    svg.append(f'<circle cx="{px(mast)[0]:.1f}" cy="{px(mast)[1]:.1f}" r="7" fill="#333"/>')
    svg.append(text(mast, "S", size=10, weight="bold", fill="#fff"))

# Elevated walk/bike ring
ring_r = 31.0
svg.append(f'<circle cx="{C_PX:.1f}" cy="{C_PX:.1f}" r="{ring_r*SC:.1f}" fill="none" stroke="#6b46c1" '
           f'stroke-width="7" stroke-opacity="0.35"/>')
svg.append(f'<circle cx="{C_PX:.1f}" cy="{C_PX:.1f}" r="{ring_r*SC:.1f}" fill="none" stroke="#6b46c1" '
           f'stroke-width="1.5" stroke-dasharray="5 4"/>')

# Paths
for name, pts in paths.items():
    col = LEFT if name.endswith("_L") else THRU
    svg.append(pline(pts, stroke=col, stroke_width=2.2, stroke_opacity=0.85))
for leg, (pts, c) in slips.items():
    svg.append(pline(pts, stroke=SLIP, stroke_width=2.2, stroke_opacity=0.9))
for _, _, p_ in crossings:
    x, y = px(p_)
    svg.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="3.2" fill="{CONF}"/>')

# Lane-use labels at the stop lines and exit-lane labels
for leg, o in LEGS.items():
    d = inbound_dir(leg)
    for i, lab in enumerate(["L", "T", "T", "T", "R"]):
        dist = STOP + 7 if i < NIN else SLIP_START + 8
        p_ = lane_point(leg, i, dist)
        svg.append(text(p_, lab, size=12, weight="bold", fill="#1a1a1a"))
    for i, lab in enumerate(["XL", "XT", "XT", "XT", "XR"]):
        dist = STOP + 7 if i < NIN else SLIP_START + 8
        p_ = lane_point(leg, i, dist, outbound=True)
        svg.append(text(p_, lab, size=10, fill="#444"))
    # arrows of travel on the outer part of the legs
    for outbound in (False, True):
        p_ = lane_point(leg, 2, EXT - 8, outbound=outbound)
        dv = d if not outbound else (-d[0], -d[1])
        tip = add(p_, dv, 4)
        tail = add(p_, dv, -4)
        svg.append(line(tail, tip, stroke="#555", stroke_width=2))
        hl = add(tip, add(dv, right_of(dv), 1), -1.6)
        hr = add(tip, add(dv, left_of(dv), 1), -1.6)
        svg.append(poly([tip, hl, hr], fill="#555"))

# Leg names
for leg, label in [("N", "North leg"), ("S", "South leg"), ("E", "East leg"), ("W", "West leg")]:
    o = LEGS[leg]
    if leg in "NS":
        p_ = (24, o[1] * (EXT - 4))
        svg.append(text(p_, label, size=14, anchor="start", weight="bold"))
    else:
        p_ = (o[0] * (EXT - 4), 25)
        svg.append(text(p_, label, size=14, anchor="end" if leg == "E" else "start", weight="bold"))

# North arrow and scale bar
na = (EXT - 9, EXT - 14)
svg.append(poly([add(na, (0, 5)), add(na, (-2, -1)), add(na, (2, -1))], fill="#222"))
svg.append(text(add(na, (0, 8)), "N", size=13, weight="bold"))
sb0 = (-EXT + 5, -EXT + 6)
svg.append(line(sb0, add(sb0, (20, 0)), stroke="#222", stroke_width=3))
for k in (0, 10, 20):
    svg.append(line(add(sb0, (k, 0)), add(sb0, (k, 1.2)), stroke="#222", stroke_width=2))
svg.append(text(add(sb0, (10, 3)), "20 m", size=12))

# Legend under the plan
ly = W_PX + 22
items = [
    (THRU, "through paths (3 lanes per approach)"),
    (LEFT, f"left-turn paths (radius {R_LEFT:.1f} m, about 27 km/h)"),
    (SLIP, f"slip right turns outside the box (radius {R_SLIP:.0f} m)"),
]
for k, (col, lab) in enumerate(items):
    x = 24 + (k % 2) * 450
    y = ly + (k // 2) * 26
    svg.append(f'<line x1="{x}" y1="{y}" x2="{x+34}" y2="{y}" stroke="{col}" stroke-width="3"/>')
    svg.append(f'<text x="{x+44}" y="{y}" font-size="14" dominant-baseline="middle" fill="#222">{lab}</text>')
x, y = 474, ly + 26
svg.append(f'<circle cx="{x+17}" cy="{y}" r="4" fill="{CONF}"/>')
svg.append(f'<text x="{x+44}" y="{y}" font-size="14" dominant-baseline="middle" fill="#222">'
           f'{len(crossings)} crossing conflict zones, no merges in the box</text>')
y = ly + 52
svg.append(f'<line x1="24" y1="{y}" x2="58" y2="{y}" stroke="#6b46c1" stroke-width="6" stroke-opacity="0.35"/>')
svg.append(f'<text x="68" y="{y}" font-size="14" dominant-baseline="middle" fill="#222">'
           f'elevated walk and bike ring</text>')
svg.append(f'<circle cx="{474+17}" cy="{y}" r="7" fill="#333"/>')
svg.append(f'<text x="{474+17}" y="{y}" font-size="10" font-weight="bold" fill="#fff" text-anchor="middle" '
           f'dominant-baseline="middle">S</text>')
svg.append(f'<text x="{474+44}" y="{y}" font-size="14" dominant-baseline="middle" fill="#222">'
           f'sensor and radio mast (RSU, lidar, radar, camera)</text>')
y = ly + 80
svg.append(f'<text x="24" y="{y}" font-size="13" dominant-baseline="middle" fill="#444">'
           f'L, T, R: entry-lane use at the stop line. XL, XT, XR: receiving lanes, one per movement. '
           f'Dashed square: the {2*STOP:.0f} m box.</text>')
svg.append("</svg>")
with open(os.path.join(OUT, "plan-view.svg"), "w") as f:
    f.write("\n".join(svg))

# ---------------------------------------------------------------- approach zones strip
Z = [  # (start m before stop line, end, label, sub-lines, color)
    (1000, 600, "A  Announce", ["register, receive map,", "get a lane assignment"], "#e3edf7"),
    (600, 200, "B  Sort and coordinate", ["lane changes, platoons,", "provisional slot, speed trim"], "#cfe0f1"),
    (200, 0, "C  Commit", ["plan frozen,", "no lane changes"], "#f6dfc7"),
    (0, -36, "D", [], "#f1c6c6"),
    (-36, -336, "E  Egress", ["own exit lane, merges,", "handoff downstream"], "#dcefd9"),
]
W2, H2 = 1100, 250
x0, x1 = 40, W2 - 40
lo, hi = -336, 1000
TOP, BOT = 58, 132


def X(m):
    return x0 + (hi - m) / (hi - lo) * (x1 - x0)


s = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W2} {H2}" font-family="Helvetica, Arial, sans-serif">',
     f'<rect x="0" y="0" width="{W2}" height="{H2}" fill="#ffffff"/>']
for a, b, lab, sub, col in Z:
    s.append(f'<rect x="{X(a):.1f}" y="{TOP}" width="{X(b)-X(a):.1f}" height="{BOT-TOP}" fill="{col}" '
             f'stroke="#888" stroke-width="1"/>')
    cx = (X(a) + X(b)) / 2
    if lab == "D":
        s.append(f'<text x="{cx:.1f}" y="{(TOP+BOT)/2:.0f}" font-size="13" font-weight="bold" '
                 f'text-anchor="middle" dominant-baseline="middle">D</text>')
        continue
    s.append(f'<text x="{cx:.1f}" y="{TOP+18}" font-size="14" font-weight="bold" text-anchor="middle" '
             f'dominant-baseline="middle" fill="#222">{lab}</text>')
    for k, line_ in enumerate(sub):
        s.append(f'<text x="{cx:.1f}" y="{TOP+38+15*k}" font-size="11.5" text-anchor="middle" '
                 f'dominant-baseline="middle" fill="#333">{line_}</text>')
# travel arrow
s.append(f'<line x1="{x0}" y1="38" x2="{x1-10}" y2="38" stroke="#555" stroke-width="2"/>')
s.append(f'<polygon points="{x1},38 {x1-12},32 {x1-12},44" fill="#555"/>')
s.append(f'<text x="{x0}" y="22" font-size="13" fill="#333">direction of travel (approach at 50 km/h)</text>')
# distance ticks along the bottom edge
for m in [1000, 800, 600, 400, 200, 0, -100, -200, -300]:
    xx = X(m)
    s.append(f'<line x1="{xx:.1f}" y1="{BOT}" x2="{xx:.1f}" y2="{BOT+7}" stroke="#666" stroke-width="1"/>')
    s.append(f'<text x="{xx:.1f}" y="{BOT+18}" font-size="10.5" text-anchor="middle" fill="#666">'
             f'{abs(m):,} m</text>')
# named markers below the ticks, staggered so they never overlap
marks = [
    (1000, "first contact", 0, "start"),
    (200, "commitment line", 0, "middle"),
    (100, "authority point", 1, "middle"),
    (46, "point of no return", 2, "middle"),
    (0, "stop line", 3, "middle"),
]
for m, lab, row, anchor in marks:
    xx = X(m)
    y_lab = BOT + 40 + 16 * row
    s.append(f'<line x1="{xx:.1f}" y1="{BOT+22}" x2="{xx:.1f}" y2="{y_lab-7}" stroke="#222" '
             f'stroke-width="1" stroke-dasharray="2 2"/>')
    s.append(f'<text x="{xx:.1f}" y="{y_lab}" font-size="12" text-anchor="{anchor}" fill="#222">{lab}</text>')
s.append(f'<text x="{x0}" y="{H2-10}" font-size="12" fill="#444">Distances are for a 50 km/h approach. '
         f'They scale with speed and with how much delay zone B must absorb.</text>')
s.append("</svg>")
with open(os.path.join(OUT, "approach-zones.svg"), "w") as f:
    f.write("\n".join(s))
print("wrote", os.listdir(OUT))
