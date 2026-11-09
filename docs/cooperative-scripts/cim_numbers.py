#!/usr/bin/env python3
# Copyright (c) 2026 Jim Schreckengast
# SPDX-License-Identifier: MIT OR Apache-2.0
"""Back-of-envelope numbers for the cooperative-intersection notes
(../cooperative-intersections.md).

Everything here is deliberately simple: one conflict zone, straight crossing,
constant crossing speed. Parameters follow Intersection Lab where it has them
(car 4.5 m long, 3.5 m lanes, 13.9 m/s arterial limit, A_LAT 2.6 m/s^2).
The Monte Carlo runs use fixed seeds, so the output is the same on every run.

Usage: python3 cim_numbers.py > numbers.txt
Needs Python 3.8 or later and nothing outside the standard library.

Where the notes use each printed section:
  1   capacity-optimal crossing speed: section 2.3
  1b  zone occupancy at 40 km/h: section 2.1
  2   batching versus the zipper: section 2.4
  3   signal reference at one crossing point: section 2.2
  4   delay a coordination zone can absorb, and the moving queue: section 2.7
  5   Monte Carlo: section 2.7 and Appendix B
  6   turning speeds: section 2.5
  7   energy per avoided stop: section 10.4
  8   radio load: section 4.10
  9   an early per-zone estimate, not used (section 11 replaces it)
  10  failure policies: section 2.2
  11  whole-intersection bounds: sections 2.6 and 6.9
"""
import math
import random
import statistics as st

L = 4.5        # car length (m), lab Car
W = 3.5        # conflict-zone length along the path for a right-angle crossing (m)
TR = 0.3       # detect + communicate + actuate (s)
DELTA = 0.1    # timing tolerance per vehicle, each side (s)
GAP = 0.6      # CACC time gap, same lane (s)
S_HUMAN = 1900  # HCM base saturation flow (veh/h/ln)


def kmh(v):
    return v * 3.6


def occ(v, length=L):
    return (length + W) / v


def sep(policy, v, a, length=L):
    """Minimum time from one vehicle's nominal entry to the next conflicting vehicle's
    nominal entry at a crossing zone, when they come from different paths."""
    base = occ(v, length) + 2 * DELTA
    if policy == "P0":   # occupancy only (no failure margin)
        return base
    if policy == "P1":   # survives a max-braking stop of the first vehicle (equal brakes)
        return base + TR
    if policy == "P2":   # survives a sudden stop ("brick wall") of the first vehicle
        return base + TR + v / (2 * a)
    raise ValueError(policy)


def hp(v, gap=GAP, length=L):
    """Same-lane follower headway at the zone (s)."""
    return gap + length / v


print("== 1. One crossing zone, streams alternating every vehicle (veh/h through the zone) ==")
speeds = [5.6, 8.3, 11.1, 13.9, 16.7]
for a in [8.0, 6.0, 4.0, 2.0]:
    print(f"  braking a = {a} m/s^2")
    print("    speed   P0 occ-only  P1 brake-safe  P2 stop-safe  same-lane CACC  same-lane human")
    for v in speeds:
        print(
            f"    {kmh(v):4.0f} km/h  {3600/sep('P0', v, a):7.0f}     {3600/sep('P1', v, a):7.0f}"
            f"        {3600/sep('P2', v, a):7.0f}       {3600/hp(v):7.0f}         {S_HUMAN:5d}"
        )
    vstar = math.sqrt(2 * a * (L + W))
    print(f"    P2 capacity-optimal crossing speed v* = sqrt(2a(L+W)) = {vstar:.1f} m/s ({kmh(vstar):.0f} km/h),"
          f" sep {sep('P2', vstar, a):.2f} s -> {3600/sep('P2', vstar, a):.0f} veh/h")

print()
print("== 1b. Separation distances at 40 km/h (11.1 m/s), a = 6 ==")
v, a = 11.1, 6.0
print(f"  occupancy {occ(v):.2f} s; P2 gap after exit {TR + v/(2*a):.2f} s = {(TR + v/(2*a))*v:.1f} m;"
      f" stopping distance {v*v/(2*a):.1f} m")

print()
print("== 2. Batching: k vehicles from one lane, then k from the crossing lane (P2, 40 km/h, a=6) ==")
S2 = sep("P2", 11.1, 6.0)
H = hp(11.1)
print(f"  switch separation S = {S2:.2f} s, same-lane headway h = {H:.2f} s")
for k in [1, 2, 3, 4, 6, 8, 12, 20]:
    thr = k / ((k - 1) * H + S2)
    over = (S2 - H) / ((k - 1) * H + S2)
    cycle = 2 * ((k - 1) * H + S2)
    print(f"  k={k:3d}: {3600*thr:5.0f} veh/h through the zone, overhead {100*over:4.1f}%, micro-cycle {cycle:5.1f} s")
print(f"  k=inf: {3600/H:5.0f} veh/h")
print("  Same with P0 (occupancy only):")
S0 = sep("P0", 11.1, 6.0)
for k in [1, 2, 4, 8]:
    print(f"  k={k:3d}: {3600*k/((k-1)*H + S0):5.0f} veh/h")

print()
print("== 3. Signal reference: one crossing point between two lanes, two-phase ==")
for C in [60, 90, 120]:
    for lost_per_phase in [4.0, 5.0]:
        cap = S_HUMAN * (1 - 2 * lost_per_phase / C)
        print(f"  C={C:3d} s, lost {lost_per_phase} s/phase: {cap:5.0f} veh/h through the point (both lanes)")

print()
print("== 4. How much delay a control zone can absorb without stopping ==")


def absorbable(D, v0, vmin, vx, a_dec=1.5, a_acc=1.5):
    """Max extra time over the nominal profile, staying >= vmin and arriving at the
    stop line at vx. Nominal: cruise at v0, slow to vx at the end."""
    d_n = (v0 * v0 - vx * vx) / (2 * a_dec)
    t_nom = (D - d_n) / v0 + (v0 - vx) / a_dec
    d1 = (v0 * v0 - vmin * vmin) / (2 * a_dec)
    d3 = (vx * vx - vmin * vmin) / (2 * a_acc)
    if d1 + d3 > D:
        return float("nan"), t_nom
    t = (v0 - vmin) / a_dec + (D - d1 - d3) / vmin + (vx - vmin) / a_acc
    return t - t_nom, t_nom


for vmin in [8.3, 5.6, 2.8]:
    row = []
    for D in [150, 300, 500, 800]:
        dmax, _ = absorbable(D, 13.9, vmin, 11.1)
        row.append(f"{D} m: {dmax:5.1f} s")
    print(f"  never below {kmh(vmin):3.0f} km/h -> " + ", ".join(row))
# moving-queue storage
for vmin in [8.3, 5.6]:
    sp_fast = L + GAP * 13.9
    sp_slow = L + GAP * vmin
    print(f"  300 m lane stores {300/sp_slow:4.1f} cars at {kmh(vmin):.0f} km/h vs {300/sp_fast:4.1f} at 50 km/h"
          f" (+{300/sp_slow-300/sp_fast:4.1f})")

print()
print("== 5. Monte Carlo: two crossing lanes, one zone, arrivals with 1.0 s minimum headway ==")


def arrivals(rate_vph, hours, rng, hmin):
    mean = 3600.0 / rate_vph
    t, out = 0.0, []
    while t < hours * 3600:
        t += hmin + rng.expovariate(1.0 / (mean - hmin))
        out.append(t)
    return out


def schedule(streams, policy, S, H, theta=1.5, kmax=12, wmax=20.0):
    """streams: list of two sorted lists of earliest times. Returns list of delays."""
    idx = [0, 0]
    t_last = -1e9
    t_last_s = [-1e9, -1e9]
    last_stream = None
    batch = 0
    delays = []
    n0, n1 = len(streams[0]), len(streams[1])
    while idx[0] < n0 or idx[1] < n1:
        cand = []
        for s in (0, 1):
            if idx[s] < len(streams[s]):
                e = streams[s][idx[s]]
                sep_ = H if last_stream == s else S
                t = max(e, t_last + sep_ if last_stream is not None else e, t_last_s[s] + H)
                cand.append((s, e, t))
        if policy == "FCFS":
            s, e, t = min(cand, key=lambda c: c[1])
        else:  # BATCH: extend the current platoon while followers keep coming
            pick = None
            if last_stream is not None:
                cur = [c for c in cand if c[0] == last_stream]
                oth = [c for c in cand if c[0] != last_stream]
                if cur:
                    s, e, t = cur[0]
                    other_waiting = oth and oth[0][1] <= t
                    other_wait = (t - oth[0][1]) if oth else 0.0
                    if t <= t_last + H + theta and (batch < kmax or not other_waiting) and other_wait < wmax:
                        pick = cur[0]
            if pick is None:
                pick = min(cand, key=lambda c: c[2])
            s, e, t = pick
        batch = batch + 1 if s == last_stream else 1
        delays.append(t - e)
        t_last = t
        t_last_s[s] = t
        last_stream = s
        idx[s] += 1
    return delays


A300_30, _ = absorbable(300, 13.9, 8.3, 11.1)
A300_20, _ = absorbable(300, 13.9, 5.6, 11.1)
A500_20, _ = absorbable(500, 13.9, 5.6, 11.1)
print(f"  absorbable: 300 m >=30 km/h {A300_30:.1f} s; 300 m >=20 km/h {A300_20:.1f} s; 500 m >=20 km/h {A500_20:.1f} s")
H = hp(11.1)
for pol_name, S in [("P0", sep("P0", 11.1, 6.0)), ("P2", sep("P2", 11.1, 6.0))]:
    print(f"  policy {pol_name}: S = {S:.2f} s, H = {H:.2f} s")
    print("    total veh/h  split   sched   mean d  p95 d  p99 d  no-delay  <=A300/30  <=A300/20  <=A500/20")
    for total in [1200, 1800, 2400, 2800, 3000, 3200]:
        for split in [0.5, 0.7]:
            rows = {}
            for policy in ["FCFS", "BATCH"]:
                ds = []
                for seed in range(4):
                    rng = random.Random(1000 * seed + total + int(100 * split))
                    a0 = arrivals(total * split, 2.0, rng, 1.0)
                    a1 = arrivals(total * (1 - split), 2.0, rng, 1.0)
                    d = schedule([a0, a1], policy, S, H)
                    # drop the first 15 minutes as warm-up (approximately, by count)
                    ds.extend(d[len(d) // 8:])
                ds.sort()
                n = len(ds)
                mean = st.fmean(ds)
                p95 = ds[int(0.95 * n)]
                p99 = ds[int(0.99 * n)]
                nod = sum(1 for x in ds if x < 0.05) / n
                f1 = sum(1 for x in ds if x <= A300_30) / n
                f2 = sum(1 for x in ds if x <= A300_20) / n
                f3 = sum(1 for x in ds if x <= A500_20) / n
                flag = "  (unstable)" if mean > 120 else ""
                print(f"    {total:5d}      {split:.1f}    {policy:5s}  {mean:6.1f}  {p95:5.1f}  {p99:5.1f}"
                      f"   {100*nod:5.1f}%    {100*f1:5.1f}%     {100*f2:5.1f}%     {100*f3:5.1f}%{flag}")

print()
print("== 6. Turning speeds at the lab's lateral acceleration (2.6 m/s^2) ==")
for R in [10, 15, 20, 25, 30, 40]:
    v = math.sqrt(2.6 * R)
    print(f"  radius {R:3d} m -> {v:4.1f} m/s ({kmh(v):3.0f} km/h)")

print()
print("== 7. Energy per avoided stop from 50 km/h, lab car (1500 kg, 30% efficiency) ==")
ke = 0.5 * 1500 * 13.9 ** 2
ml = ke / (34_200 * 0.30)
print(f"  kinetic energy {ke/1000:.0f} kJ -> {ml:.1f} mL gasoline to regain it; idling 0.25 mL/s")

print()
print("== 8. Radio load, order of magnitude ==")
for n in [200, 500, 1000]:
    for hz, size in [(10, 350), (10, 500)]:
        mbps = n * hz * size * 8 / 1e6
        print(f"  {n:4d} vehicles x {hz} Hz x {size} B = {mbps:5.1f} Mbit/s")

print()
# An early estimate, kept for reference but not used in the notes. It lets every lane
# have half of each zone's time, ignoring that a lane crosses several conflicting
# streams, so it overstates capacity. Section 11 replaces it.
print("== 9. Reference design, per-zone bound (each crossing zone shared by two lanes) ==")
for v_box in [11.1, 13.9]:
    s = 3600 / hp(v_box)
    for k in [4, 8]:
        S = sep("P2", v_box, 6.0)
        Hh = hp(v_box)
        over = (S - Hh) / ((k - 1) * Hh + S)
        lane = s * (1 - over) / 2
        through = 12 * lane
        left = 4 * lane
        right = 4 * 1500
        print(f"  box {kmh(v_box):.0f} km/h, batches of {k}: lane sat {s:.0f}, overhead {100*over:.0f}% ->"
              f" {lane:.0f} veh/h per crossing lane; 12 through + 4 left lanes = {through+left:.0f}, + slip rights"
              f" {right} = {through+left+right:.0f} veh/h")
# Conventional four-phase signal on the same lanes
for C, gT, gL, lost in [(120, 36, 16, 16), (100, 30, 12, 16)]:
    thr = 12 * S_HUMAN * gT / C + 4 * S_HUMAN * gL / C + 4 * 600
    print(f"  4-phase signal C={C}: through lanes {S_HUMAN*gT/C:.0f}/ln, lefts {S_HUMAN*gL/C:.0f}/ln,"
          f" rights ~600/ln -> {thr:.0f} veh/h")
# Same box, human-like headways
for v_box in [11.1]:
    Hh = 1.9  # s, human saturation headway
    S = sep("P2", v_box, 6.0) + 0.7  # longer reaction for humans
    for k in [4, 8]:
        over = (S - Hh) / ((k - 1) * Hh + S)
        lane = (3600 / Hh) * (1 - over) / 2
        print(f"  same box with human headways (1.9 s), batches of {k}: {lane:.0f} veh/h per crossing lane")

print()
print("== 10. Failure policies at 40 km/h (11.1 m/s), a = 6: PET and zone capacity ==")
v, a = 11.1, 6.0
vs = 25 / 3.6  # survivable secondary impact speed for the harm-limited policy
gaps = {
    "P0 occupancy only": 2 * DELTA,
    "P1 brake-safe (equal brakes)": 2 * DELTA + TR,
    "PH harm-limited (<=25 km/h impact)": 2 * DELTA + TR + (v * v - vs * vs) / (2 * a * v),
    "P2 stop-safe": 2 * DELTA + TR + v / (2 * a),
}
Hh = hp(v)
for name, g in gaps.items():
    S = occ(v) + g
    k1 = 3600 / S
    k4 = 3600 * 4 / (3 * Hh + S)
    k8 = 3600 * 8 / (7 * Hh + S)
    print(f"  {name:36s} PET {g:4.2f} s ({g*v:4.1f} m)  k=1 {k1:5.0f}  k=4 {k4:5.0f}  k=8 {k8:5.0f} veh/h")

print()
print("== 11. Whole intersection, clique bound (L 20% / T 65% / R 15%; 1 left, 3 through, slip right) ==")
pL, pT, pR = 0.20, 0.65, 0.15
qT_per_Q = pT / 3   # per through lane
qL_per_Q = pL
# Conventional dual-ring signal, human drivers
for C, Lt in [(120, 16), (90, 16)]:
    y = 2 * (qT_per_Q / 1900 + qL_per_Q / 1805)
    Q = (1 - Lt / C) / y
    print(f"  conventional signal C={C}: Q = {Q:5.0f} veh/h per approach, {4*Q:6.0f} total")
# AV-aware signal: CACC discharge, flying starts, 2 s lost per phase
hT50 = hp(13.9)
hL = hp(math.sqrt(2.6 * 21.75))
for C in [30, 40, 60]:
    y = 2 * (qT_per_Q * hT50 + qL_per_Q * hL) / 3600
    Q = (1 - 8 / C) / y
    print(f"  AV-aware signal C={C}: Q = {Q:5.0f} per approach, {4*Q:6.0f} total (hT {hT50:.2f} s, hL {hL:.2f} s)")
# Cooperative box: triangles {T,T,L} and {L,L,T}; overhead from 3 switches per micro-cycle
hT40 = hp(11.1)
uT = qT_per_Q * hT40 / 3600
uL = qL_per_Q * hL / 3600
worst = max(2 * uT + uL, 2 * uL + uT)
switch_loss = sep("P2", 11.1, 6.0) - hT40
for Cm in [15, 20, 30]:
    o = 3 * switch_loss / Cm
    Q = (1 - o) / worst
    print(f"  cooperative box, micro-cycle {Cm} s (overhead {100*o:.0f}%): Q = {Q:5.0f} per approach, {4*Q:6.0f} total")
# Per-zone (pairwise) bound for comparison: optimistic
pair = max(2 * uT, uT + uL, 2 * uL)
print(f"  pairwise per-zone bound at 20% overhead (optimistic): {4*0.8/pair:6.0f} total")
