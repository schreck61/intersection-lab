// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Signal-free intersection management for automated, connected vehicles.
//!
//! On the cooperative layout every vehicle is automated and talks to an intersection
//! manager; there are no signals and no roadside sensors. When a vehicle enters the
//! approach it asks for a reservation. The manager answers with a complete trajectory,
//! one acceleration per simulation step from now until the vehicle leaves the map, and
//! the vehicle drives exactly that trajectory. The manager builds each trajectory with
//! the integrator the simulator itself uses, so it knows to rounding error when every
//! vehicle will be in every part of the box, and it grants a trajectory only after
//! checking it against all the trajectories it has already granted. A granted
//! trajectory is never changed.
//!
//! * **Reservation zones.** Each pair of crossing paths has a zone on each path where a
//!   vehicle of the widest class could touch one on the other path
//!   (`Network::reservation_zones`). A vehicle holds a zone from the moment its front
//!   enters it until its rear leaves.
//! * **Safety gap.** A vehicle may enter a zone only some time after the last
//!   conflicting vehicle left it ([`GapPolicy`]). The default, fail-safe policy leaves
//!   the follower enough time to stop short of the zone if the vehicle ahead of it in
//!   time stalls there: two message latencies, the actuation delay and the time to
//!   brake from its own speed.
//! * **Same-lane following.** A trajectory is simulated behind the granted trajectory
//!   of the vehicle ahead in its lane with a constant-time-gap cooperative adaptive
//!   cruise control law, and is rejected if the gap ever drops below a meter.
//! * **Speed profile.** Each vehicle aims for its slot with a decelerate-cruise-
//!   accelerate profile, crossing the stop line at the crossing speed (40 km/h straight
//!   ahead, the curve speed for left turns), so it is never slow inside the box. Long
//!   waits come to rest near the line and leave with a flying start.
//! * **Phases.** Serving conflicting movements alternately, vehicle by vehicle, wastes
//!   the box: every switch costs a clearance gap. The manager serves each movement in
//!   batches, with the ring-and-barrier structure of an actuated signal controller but
//!   no fixed clearance intervals: two rings each serve one through movement and one
//!   left turn of the north-south approaches, then both cross the barrier to the
//!   east-west ones. A batch accepts each vehicle of its movement that can cross the
//!   line soon after the batch's latest one; it ends when none can (gap-out), or when
//!   it has used its share of the cycle (max-out), the share following the measured
//!   flow of each movement. The last phase of a ring before the barrier may run on
//!   while the other ring finishes, since the barrier waits for that ring anyway. The
//!   next batch starts exactly as soon as the reservations allow, zone by zone, rather
//!   than after a yellow and all-red interval; when a queue is waiting, its first
//!   vehicle goes after the last conflicting one rather than into an earlier gap, so
//!   the queue crosses together. Whether left turns lead or lag is chosen from those
//!   clearance times, which are very uneven.
//! * **Waiting.** A vehicle without a trajectory follows the vehicle ahead and waits
//!   far enough before the line to reach the crossing speed by the time it crosses.
//!   An emergency vehicle, and the vehicles ahead of it in its lane, get the earliest
//!   trajectories that fit, whatever the phase.
//! * **Monitors.** Independently of the plans, the simulator measures when each
//!   vehicle actually enters and leaves each zone: the post-encroachment time between
//!   conflicting vehicles, any zone entered while a conflicting vehicle is still in it,
//!   any vehicle entering the box without a reservation, and any difference between
//!   where a vehicle is and where its trajectory says it should be.

use super::network::{LinkId, LinkKind, Network};
use super::vtypes::VKind;
use super::{ControlMode, Reason, Sim, Vehicle, integrate};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, VecDeque};

/// One-way message latency between a vehicle and the manager (s).
pub const LATENCY: f64 = 0.1;
/// Delay between a command and the vehicle's actuators responding (s).
pub const ACTUATION: f64 = 0.3;
/// Impact speed the mitigated policy tolerates (25 km/h).
pub const IMPACT_OK: f64 = 25.0 / 3.6;
/// Gains of the following law on the spacing error and the speed difference.
const K_P: f64 = 0.8;
const K_D: f64 = 1.4;
/// Smallest bumper-to-bumper gap a granted trajectory may leave behind the one ahead (m).
const MIN_PLAN_GAP: f64 = 1.0;
/// Hardest braking a trajectory may use (m/s²); emergency braking stays in reserve.
const PLAN_DECEL: f64 = 3.5;
/// Slowest cruising speed of a vehicle that waits by slowing down early (m/s); longer
/// waits come to rest near the line instead.
const CRAWL: f64 = 2.5;
/// Pad added when a trajectory is moved later to clear a reservation (s).
const PAD: f64 = 0.02;
/// Longest trajectory the manager will build (steps).
const MAX_PLAN_STEPS: usize = 8000;
/// Post-encroachment times longer than this are not interactions (s).
const PET_WINDOW: f64 = 10.0;
/// Width of a post-encroachment-time histogram bin (s).
pub const PET_BIN: f64 = 0.25;
/// Time constant of the measured arrival rates (s).
const RATE_TAU: f64 = 300.0;
/// Shortest maximum phase length (s).
const MIN_MAX_GREEN: f64 = 6.0;
/// How long the served phases and line crossings are kept for display (s).
const HISTORY: f64 = 600.0;

/// How long a vehicle must wait after a conflicting vehicle leaves a zone before it
/// enters it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum GapPolicy {
    /// The follower can always stop short of the zone if the vehicle ahead of it in
    /// time stalls there (P2).
    FailSafe,
    /// The follower can always slow to 25 km/h before reaching a stalled vehicle (PH).
    Mitigated,
    /// Message latency and actuation delay only: assumes no vehicle ever fails (P1).
    Nominal,
}

impl GapPolicy {
    pub const ALL: [GapPolicy; 3] = [GapPolicy::FailSafe, GapPolicy::Mitigated, GapPolicy::Nominal];
    pub fn name(self) -> &'static str {
        match self {
            GapPolicy::FailSafe => "Fail-safe: can stop for a stalled vehicle",
            GapPolicy::Mitigated => "Mitigated: can slow to 25 km/h",
            GapPolicy::Nominal => "Nominal: latency only (no failures)",
        }
    }
    /// One word for labels.
    pub fn label(self) -> &'static str {
        match self {
            GapPolicy::FailSafe => "fail-safe",
            GapPolicy::Mitigated => "mitigated",
            GapPolicy::Nominal => "nominal",
        }
    }
    pub fn short(self) -> &'static str {
        match self {
            GapPolicy::FailSafe => "P2",
            GapPolicy::Mitigated => "PH",
            GapPolicy::Nominal => "P1",
        }
    }
    /// Gap (s) for a vehicle entering at speed `v` with emergency deceleration `brake`.
    pub fn gap(self, v: f64, brake: f64) -> f64 {
        let base = 2.0 * LATENCY + ACTUATION;
        match self {
            GapPolicy::FailSafe => base + v / (2.0 * brake),
            GapPolicy::Mitigated if v > IMPACT_OK => base + (v * v - IMPACT_OK * IMPACT_OK) / (2.0 * brake * v),
            GapPolicy::Mitigated | GapPolicy::Nominal => base,
        }
    }
}

/// In what order the manager gives out slots.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Scheduling {
    /// Each movement in turn, in batches, in the ring-and-barrier order of an actuated
    /// signal (the default).
    Batches,
    /// Each vehicle as soon as it can be scheduled, in the earliest gap that fits: the
    /// streams weave through one another one vehicle at a time.
    FirstCome,
}

impl Scheduling {
    pub const ALL: [Scheduling; 2] = [Scheduling::Batches, Scheduling::FirstCome];
    pub fn name(self) -> &'static str {
        match self {
            Scheduling::Batches => "Batches: each movement in turn",
            Scheduling::FirstCome => "First come, first served: one at a time",
        }
    }
    /// A few words for labels.
    pub fn label(self) -> &'static str {
        match self {
            Scheduling::Batches => "batches",
            Scheduling::FirstCome => "first come, first served",
        }
    }
}

/// Emergency deceleration each class can count on (m/s²).
pub fn brake_decel(kind: VKind) -> f64 {
    match kind {
        VKind::Truck | VKind::Bus => 4.5,
        _ => 6.0,
    }
}

/// Following parameters of automated vehicles: standstill gap (m) and time gap (s).
/// Trucks and buses brake less hard than the cars they may follow, so they keep more.
pub fn follow_params(kind: VKind) -> (f64, f64) {
    match kind {
        VKind::Motorcycle => (1.5, 0.42),
        VKind::Truck | VKind::Bus => (2.0, 0.75),
        _ => (2.0, 0.42),
    }
}

/// Driving parameters of an automated vehicle.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Av {
    pub len: f64,
    /// Acceleration and comfortable deceleration (m/s²).
    pub a: f64,
    pub b: f64,
    pub s0: f64,
    pub tg: f64,
    /// Desired speed (m/s).
    pub v0: f64,
    pub brake: f64,
}

impl Av {
    pub(crate) fn of(v: &Vehicle) -> Av {
        let spec = v.spec();
        let (s0, tg) = follow_params(v.kind);
        Av { len: spec.length, a: spec.accel, b: spec.decel, s0, tg, v0: v.v0, brake: brake_decel(v.kind) }
    }
}

/// Following: constant-time-gap control with the leader's acceleration fed forward
/// near the desired spacing, and kinematic braking when closing in from farther back.
///
/// The braking needed to keep the standstill gap is the larger of matching the leader's
/// speed before the gap is used up and, when the leader is braking, stopping behind the
/// point where it will stop. While that need is small the vehicle may still accelerate;
/// from half its comfortable deceleration on it brakes at least that hard.
pub(crate) fn follow_acc(av: &Av, v: f64, gap: f64, vl: f64, al: f64) -> f64 {
    let e = gap - (av.s0 + av.tg * v);
    let cacc = al.min(av.a) + K_P * e + K_D * (vl - v);
    let room = (gap - av.s0).max(0.1);
    let closing = (v - vl).max(0.0);
    let mut need = closing * closing / (2.0 * room);
    if al < -0.1 {
        let leader_stops = vl * vl / (2.0 * -al);
        need = need.max(v * v / (2.0 * (room + leader_stops)));
    }
    let kin = -need + av.a * (1.0 - need / (0.5 * av.b)).max(0.0);
    cacc.min(kin)
}

// ---------------------------------------------------------------- speed profiles

/// A speed profile of up to four constant-acceleration segments (duration, accel).
#[derive(Clone, Copy, Debug, Default)]
struct Prof {
    seg: [(f64, f64); 4],
    n: usize,
}

impl Prof {
    fn push(&mut self, dur: f64, acc: f64) {
        if dur > 1e-9 && self.n < 4 {
            self.seg[self.n] = (dur, acc);
            self.n += 1;
        }
    }
    /// Speed change over the first `t` seconds (constant speed after the end).
    fn dv(&self, t: f64) -> f64 {
        let mut left = t;
        let mut d = 0.0;
        for &(dur, acc) in &self.seg[..self.n] {
            let x = dur.min(left);
            d += acc * x;
            left -= x;
            if left <= 0.0 {
                break;
            }
        }
        d
    }
}

/// Fastest profile over `d` from speed `v` that crosses the line no faster than `ve`,
/// never exceeding `vtop`. Returns its duration, the profile, and its cruise speed if
/// it crosses the line at exactly `ve` (a decelerate-cruise-accelerate profile).
fn fastest(d: f64, v: f64, ve: f64, vtop: f64, a: f64, b: f64) -> (f64, Prof, Option<f64>) {
    let mut p = Prof::default();
    if d <= 1e-9 {
        return (0.0, p, None);
    }
    if v > ve && (v * v - ve * ve) / (2.0 * b) >= d {
        // Too late to slow down comfortably: brake as hard as needed.
        let acc = -(v * v - ve * ve) / (2.0 * d);
        let t = 2.0 * d / (v + ve);
        p.push(t, acc);
        return (t, p, None);
    }
    if v <= ve && (ve * ve - v * v) / (2.0 * a) >= d {
        let vl = (v * v + 2.0 * a * d).sqrt();
        let t = (vl - v) / a;
        p.push(t, a);
        return (t, p, ((vl - ve).abs() < 1e-9).then_some(ve));
    }
    let vtop = vtop.max(ve).max(v);
    let vp = ((2.0 * a * b * d + b * v * v + a * ve * ve) / (a + b)).sqrt().min(vtop).max(v).max(ve);
    let (t1, d1) = ((vp - v) / a, (vp * vp - v * v) / (2.0 * a));
    let (t3, d3) = ((vp - ve) / b, (vp * vp - ve * ve) / (2.0 * b));
    let d2 = (d - d1 - d3).max(0.0);
    let t2 = if d2 > 0.0 { d2 / vp } else { 0.0 };
    p.push(t1, a);
    p.push(t2, 0.0);
    p.push(t3, -b);
    (t1 + t2 + t3, p, Some(vp))
}

/// Speed `v` to cruise speed `u`, cruise, then `u` to `ve` at the line, over `d`.
/// None if it does not fit.
fn dca(d: f64, v: f64, u: f64, ve: f64, a: f64, b: f64) -> Option<(f64, Prof)> {
    let (t1, d1, a1) = if u < v { ((v - u) / b, (v * v - u * u) / (2.0 * b), -b) } else { ((u - v) / a, (u * u - v * v) / (2.0 * a), a) };
    let (t3, d3, a3) = if ve > u { ((ve - u) / a, (ve * ve - u * u) / (2.0 * a), a) } else { ((u - ve) / b, (u * u - ve * ve) / (2.0 * b), -b) };
    let d2 = d - d1 - d3;
    if d2 < -1e-9 || (d2 > 1e-9 && u < 1e-9) {
        return None;
    }
    let t2 = if d2 > 1e-9 { d2 / u } else { 0.0 };
    let mut p = Prof::default();
    p.push(t1, a1);
    p.push(t2, 0.0);
    p.push(t3, a3);
    Some((t1 + t2 + t3, p))
}

/// Come to rest before the line, wait, and leave so as to cross it `tau` from now:
/// at rest as close to the line as still allows reaching `ve` there (or, with no room
/// for that, entering slower), braking no harder than `b`.
fn stop_profile(d: f64, v: f64, tau: f64, ve: f64, a: f64, b: f64) -> Prof {
    let d3 = ve * ve / (2.0 * a);
    let d_stop = v * v / (2.0 * b);
    let leave = |l: f64| {
        let vl = (2.0 * a * l.max(0.0)).sqrt().min(ve);
        let t_cruise = if vl >= ve { (l - d3).max(0.0) / ve } else { 0.0 };
        (vl / a, t_cruise)
    };
    let time = |r: f64| {
        let t_dec = if v > 1e-9 { 2.0 * r / v } else { 0.0 };
        let (ta, tc) = leave(d - r);
        t_dec + ta + tc
    };
    // Rest point (distance from here): close to the line, but stop sooner if easing
    // all the way there would take longer than the time available.
    let mut r = (d - d3).max(d_stop);
    if v < 0.5 {
        r = d_stop;
    } else if time(r) > tau {
        let (mut lo, mut hi) = (d_stop, r);
        for _ in 0..40 {
            let mid = 0.5 * (lo + hi);
            if time(mid) > tau {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        r = lo;
    }
    let mut p = Prof::default();
    if v > 1e-9 {
        let bb = if r > 1e-9 { (v * v / (2.0 * r)).min(PLAN_DECEL) } else { b };
        p.push(v / bb, -bb);
    }
    let (ta, tc) = leave(d - r);
    p.push((tau - time(r)).max(0.0), 0.0);
    p.push(ta, a);
    p.push(tc, 0.0);
    p
}

/// Speed profile that brings a vehicle `d` meters before the stop line at speed `v` to
/// the line `tau` seconds from now at speed `ve`, or as soon after as it can. Second
/// value: whether it is deliberately slower than the fastest way there.
fn approach_profile(d: f64, v: f64, tau: f64, ve: f64, vtop: f64, a: f64, b: f64) -> (Prof, bool) {
    let (tf, pf, vp) = fastest(d, v, ve, vtop, a, b);
    if tau <= tf + 1e-6 {
        return (pf, false);
    }
    let Some(vp) = vp else {
        return (stop_profile(d, v, tau, ve, a, b), true);
    };
    let solve = |lo: f64, hi: f64| {
        // Cruise speed that takes exactly tau; the duration falls as the speed rises.
        let (mut lo, mut hi) = (lo, hi);
        for _ in 0..40 {
            let mid = 0.5 * (lo + hi);
            match dca(d, v, mid, ve, a, b) {
                Some((t, _)) if t <= tau => hi = mid,
                _ => lo = mid,
            }
            if hi - lo < 1e-7 {
                break;
            }
        }
        dca(d, v, hi, ve, a, b).map(|x| x.1).unwrap_or(pf)
    };
    let floor = CRAWL.min(vp);
    let p = match dca(d, v, floor, ve, a, b) {
        Some((t_floor, _)) if t_floor >= tau => solve(floor, vp),
        Some(_) => stop_profile(d, v, tau, ve, a, b),
        None => {
            // Cannot slow to a crawl and still reach ve: find the slowest cruise that fits.
            let (mut lo, mut hi) = (floor, vp);
            for _ in 0..40 {
                let mid = 0.5 * (lo + hi);
                if dca(d, v, mid, ve, a, b).is_some() {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
            match dca(d, v, hi, ve, a, b) {
                Some((t_lo, _)) if t_lo >= tau => solve(hi, vp),
                _ => stop_profile(d, v, tau, ve, a, b),
            }
        }
    };
    (p, true)
}

/// Time within a step at which a vehicle at `x` with speed `v` and constant
/// acceleration `a` reaches `target` (clamped to the step).
fn cross_time(x: f64, v: f64, a: f64, target: f64, dt: f64) -> f64 {
    let d = target - x;
    if d <= 0.0 {
        return 0.0;
    }
    let disc = v * v + 2.0 * a * d;
    if disc <= 0.0 {
        return dt;
    }
    (2.0 * d / (v + disc.sqrt()).max(1e-12)).clamp(0.0, dt)
}

// ---------------------------------------------------------------- trajectories

/// A zone on a lane's path, in route coordinates (distance from the start of the
/// inbound lane).
#[derive(Clone, Copy, Debug)]
struct ZoneRef {
    cid: usize,
    side: usize,
    x0: f64,
    x1: f64,
}

/// Time a planned vehicle holds one zone.
#[derive(Clone, Copy, Debug)]
pub struct PlanZone {
    pub cid: usize,
    /// 0 if the vehicle is on the conflict's link `a`, 1 if on `b`.
    pub side: usize,
    /// Zone extent in route coordinates.
    pub x0: f64,
    pub x1: f64,
    /// Front enters, rear leaves (s).
    pub t_in: f64,
    pub t_out: f64,
    pub v_in: f64,
    /// Gap this vehicle needs after a conflicting vehicle leaves (s).
    pub need: f64,
}

/// A granted trajectory: the state at every step from `k0` until the vehicle leaves.
#[derive(Clone, Debug)]
pub struct Plan {
    pub k0: u64,
    pub t0: f64,
    /// Route coordinate of the front and speed at steps k0, k0+1, ... (one more entry
    /// than `a`).
    pub x: Vec<f64>,
    pub v: Vec<f64>,
    /// Acceleration applied during each step, and why.
    pub a: Vec<f64>,
    pub why: Vec<Reason>,
    /// When and how fast the front crosses the stop line.
    pub t_line: f64,
    pub v_line: f64,
    pub zones: Vec<PlanZone>,
    pub movement: usize,
    pub lane: LinkId,
}

impl Plan {
    /// Position, speed and acceleration at step `k`, while the vehicle is on the map.
    fn state(&self, k: u64) -> Option<(f64, f64, f64)> {
        let j = usize::try_from(k.checked_sub(self.k0)?).ok()?;
        (j < self.a.len()).then(|| (self.x[j], self.v[j], self.a[j]))
    }
}

/// Everything needed to build one vehicle's trajectory.
struct Req<'a> {
    t: f64,
    k: u64,
    dt: f64,
    x: f64,
    v: f64,
    av: Av,
    movement: usize,
    lane: LinkId,
    /// Route coordinates of the stop line, the end of the box path and the route end.
    l_in: f64,
    l_box: f64,
    l_end: f64,
    /// Speed through the box.
    v_box: f64,
    zones: &'a [ZoneRef],
    leader: Option<(&'a Plan, f64)>,
    policy: GapPolicy,
    /// First vehicle of a phase with others waiting behind it: cross after every
    /// conflicting reservation rather than in a gap before one, so that the vehicles
    /// behind can follow it without waiting for the end of the previous phases.
    opens_batch: bool,
}

/// Simulate a vehicle aiming to cross the stop line at `target`, step by step with the
/// simulator's own integrator, until it leaves the map. None if it would close within
/// a meter of the vehicle ahead or never get anywhere.
fn simulate(r: &Req, target: f64) -> Option<Plan> {
    let (dt, av) = (r.dt, r.av);
    let (mut x, mut v, mut t) = (r.x, r.v, r.t);
    let cap = ((r.l_end - x) / 8.0 / dt) as usize + 64;
    let (mut xs, mut vs) = (Vec::with_capacity(cap), Vec::with_capacity(cap));
    let (mut acs, mut why) = (Vec::with_capacity(cap), Vec::with_capacity(cap));
    xs.push(x);
    vs.push(v);
    // Current profile: shape, speed at its start, time since its start, deliberately slow.
    let mut prof: Option<(Prof, f64, f64, bool)> = None;
    let mut dirty = true;
    let mut t_line = f64::NAN;
    let mut v_line = 0.0;
    let mut zt = vec![(f64::NAN, f64::NAN, 0.0); r.zones.len()];
    let box_reason = if r.v_box < av.v0 - 0.5 { Reason::BoxSpeed } else { Reason::Free };
    let mut k = 0usize;
    while x < r.l_end {
        if k >= MAX_PLAN_STEPS {
            return None;
        }
        let n = r.k + k as u64;
        let lead = r.leader.and_then(|(p, len)| p.state(n).map(|s| (s, len)));
        let fol = match lead {
            Some(((xl, vl, al), len)) => {
                let gap = xl - len - x;
                if gap < MIN_PLAN_GAP {
                    return None;
                }
                Some(follow_acc(&av, v, gap, vl, al))
            }
            None => None,
        };
        let mut reason = Reason::Free;
        let mut acc;
        if x < r.l_in {
            let d = r.l_in - x;
            if prof.is_none() {
                let (p, slow) = approach_profile(d, v, target - t, r.v_box, av.v0, av.a, av.b);
                prof = Some((p, v, 0.0, slow));
                dirty = false;
            }
            let cmd = |p: &(Prof, f64, f64, bool)| ((p.1 + p.0.dv(p.2 + dt)).max(0.0) - v) / dt;
            let mut a_prof = cmd(prof.as_ref().unwrap());
            if dirty && fol.is_none_or(|af| af >= a_prof) {
                // The vehicle ahead held it back; it is free again, so re-aim from here.
                let (p, slow) = approach_profile(d, v, target - t, r.v_box, av.v0, av.a, av.b);
                prof = Some((p, v, 0.0, slow));
                a_prof = cmd(prof.as_ref().unwrap());
                dirty = false;
            }
            acc = a_prof;
            if acc < -0.05 {
                reason = if prof.as_ref().unwrap().3 { Reason::Scheduled } else { box_reason };
            }
            prof.as_mut().unwrap().2 += dt;
        } else if x < r.l_box {
            acc = ((r.v_box - v) / dt).clamp(-av.b, av.a);
        } else {
            acc = av.a * (1.0 - (v / av.v0.max(0.1)).powi(4));
        }
        if let Some(af) = fol
            && af < acc
        {
            acc = af;
            reason = Reason::Following;
            if x < r.l_in {
                dirty = true;
            }
        }
        acc = acc.clamp(-PLAN_DECEL, av.a);
        let (ds, vn) = integrate(v, acc, dt);
        let xn = x + ds;
        if x < r.l_in && xn >= r.l_in {
            let tau = cross_time(x, v, acc, r.l_in, dt);
            t_line = t + tau;
            v_line = v + acc * tau;
        }
        for (z, w) in r.zones.iter().zip(zt.iter_mut()) {
            if w.0.is_nan() && x < z.x0 && xn >= z.x0 {
                let tau = cross_time(x, v, acc, z.x0, dt);
                w.0 = t + tau;
                w.2 = v + acc * tau;
            }
            let xr = z.x1 + av.len;
            if w.1.is_nan() && x < xr && xn >= xr {
                w.1 = t + cross_time(x, v, acc, xr, dt);
            }
        }
        x = xn;
        v = vn;
        t += dt;
        k += 1;
        xs.push(x);
        vs.push(v);
        acs.push(acc);
        why.push(reason);
    }
    if t_line.is_nan() || zt.iter().any(|w| w.0.is_nan() || w.1.is_nan()) {
        return None;
    }
    let zones = r
        .zones
        .iter()
        .zip(&zt)
        .map(|(z, w)| PlanZone {
            cid: z.cid,
            side: z.side,
            x0: z.x0,
            x1: z.x1,
            t_in: w.0,
            t_out: w.1,
            v_in: w.2,
            need: r.policy.gap(w.2, av.brake),
        })
        .collect();
    Some(Plan { k0: r.k, t0: r.t, x: xs, v: vs, a: acs, why, t_line, v_line, zones, movement: r.movement, lane: r.lane })
}

// ---------------------------------------------------------------- manager

/// A reservation of one zone.
#[derive(Clone, Copy, Debug)]
struct Res {
    t_in: f64,
    t_out: f64,
    need: f64,
}

/// Batch state of one movement.
#[derive(Clone, Debug, Default)]
struct MoveState {
    open: bool,
    /// Line-crossing time of the batch's first and latest vehicles.
    first_t: f64,
    frontier: f64,
    granted: usize,
}

/// One ring of the phase sequence: movements served one after another. The barrier
/// splits each ring into the two groups of approaches that face each other (north and
/// south, east and west); both rings serve the same group at a time.
#[derive(Clone, Debug)]
struct Ring {
    /// (group, movement) in service order, grouped.
    seq: Vec<(usize, usize)>,
    pos: usize,
    /// Done with its phases in the current group, waiting for the other ring.
    at_barrier: bool,
}

/// Why a phase ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PhaseEnd {
    /// Still running.
    Running,
    /// No vehicle can reach the line soon enough to continue the batch.
    GapOut,
    /// The batch has run as long as it may.
    MaxOut,
}

/// What serving a phase did.
enum Served {
    Granted,
    Failed(u64),
    Nobody,
}

/// The next vehicle in a lane that has no trajectory yet.
#[derive(Clone, Copy, Debug)]
struct Cand {
    vid: u64,
    idx: usize,
    lane: LinkId,
    movement: usize,
    leader: Option<u64>,
    /// Estimated earliest line crossing that fits the reservations.
    t_est: f64,
    /// Its saturation headway: the line-crossing headway behind a vehicle of its own
    /// length. It continues the batch if it can cross within this (plus the slack) of
    /// the batch's latest crossing. Behind a long truck it cannot, and the phase may
    /// end there: the truck's long tail then passes during the change of phase
    /// instead of holding up the batch.
    h: f64,
}

/// A phase as served: its movement, when its first and last vehicles crossed the stop
/// line, and how many it served.
#[derive(Clone, Copy, Debug)]
pub struct Batch {
    pub movement: usize,
    pub start: f64,
    pub end: f64,
    pub vehicles: usize,
}

/// Running counters for the manager and the safety monitors.
#[derive(Clone, Debug)]
pub struct CoopStats {
    /// Trajectories granted, simulated (including rejected tries) and requests that
    /// found no trajectory in a step.
    pub plans: u64,
    pub sims: u64,
    pub failed: u64,
    /// Phases that served at least one vehicle, phases ended by their maximum length,
    /// and vehicles served out of turn ahead of or as an emergency vehicle.
    pub batches: u64,
    pub maxed: u64,
    pub preempted: u64,
    /// Barrier crossings (cycles are two of these).
    pub barriers: u64,
    /// Zone entries within `PET_WINDOW` of a conflicting vehicle leaving.
    pub interactions: u64,
    /// Smallest post-encroachment time (s), and smallest margin over the required gap.
    pub pet_min: f64,
    pub margin_min: f64,
    pub pet_hist: [u32; 24],
    /// Zones entered while a conflicting vehicle was still inside.
    pub zone_violations: u64,
    /// Vehicles that entered the box without a reservation.
    pub unauthorized: u64,
    /// Steps where a vehicle was not where its trajectory said.
    pub deviations: u64,
    /// Largest difference between a reserved and an actual zone or line time (s).
    pub timing_err_max: f64,
}

impl Default for CoopStats {
    fn default() -> Self {
        CoopStats {
            plans: 0,
            sims: 0,
            failed: 0,
            batches: 0,
            maxed: 0,
            preempted: 0,
            barriers: 0,
            interactions: 0,
            pet_min: f64::INFINITY,
            margin_min: f64::INFINITY,
            pet_hist: [0; 24],
            zone_violations: 0,
            unauthorized: 0,
            deviations: 0,
            timing_err_max: 0.0,
        }
    }
}

/// The intersection manager.
pub struct CoopManager {
    pub policy: GapPolicy,
    /// In what order slots are given out; set it with [`CoopManager::set_scheduling`].
    pub scheduling: Scheduling,
    /// Cycle length the phases share when every movement is saturated (s). Each phase
    /// may run at most its share, in proportion to its measured flow per lane.
    pub max_cycle: f64,
    /// How far behind the batch's latest vehicle (beyond its own headway) a vehicle may
    /// be and still join the batch (s).
    pub slack: f64,
    pub plans: HashMap<u64, Plan>,
    pub stats: CoopStats,
    pub log: Vec<String>,
    /// Phases served in the last `HISTORY` seconds, oldest first.
    pub history: VecDeque<Batch>,
    /// Stop-line crossing time and movement of each vehicle granted a trajectory in the
    /// last `HISTORY` seconds, including crossings still ahead.
    pub crossings: VecDeque<(f64, usize)>,
    /// Reservations per conflict and side.
    res: Vec<[Vec<Res>; 2]>,
    /// Vehicles in each managed lane, front first; the first `n_planned` have
    /// trajectories.
    chains: HashMap<LinkId, (VecDeque<u64>, usize)>,
    moves: Vec<MoveState>,
    rings: Vec<Ring>,
    /// Barrier group being served.
    group: usize,
    /// Clearance time lost per cycle with the chosen ring order (s).
    lost: f64,
    /// Arrivals per movement, exponentially weighted, and the matching weight of time.
    arrivals: Vec<f64>,
    arr_time: f64,
    /// Lanes and saturation headway per lane (cars at the crossing speed) per movement.
    n_lanes: Vec<usize>,
    sat_h: Vec<f64>,
    /// Movements that cross the box (need reservations) and their conflicts.
    managed: Vec<bool>,
    mconf: Vec<Vec<bool>>,
    /// Per managed inbound lane: its box path, zones (route coordinates), the stop line
    /// and box-end coordinates, route length, box speed and holding distance.
    lanes: HashMap<LinkId, LaneInfo>,
    /// Monitor state per conflict side: vehicles inside, last time one left.
    inside: Vec<[u32; 2]>,
    last_out: Vec<[f64; 2]>,
}

#[derive(Clone, Debug)]
struct LaneInfo {
    movement: usize,
    zones: Vec<ZoneRef>,
    l_in: f64,
    l_box: f64,
    l_end: f64,
    v_box: f64,
    /// Speed limit on the approach.
    v_in: f64,
}

impl CoopManager {
    pub fn new(net: &Network) -> CoopManager {
        let n_moves = net.movements.len();
        let mut lanes = HashMap::new();
        let mut chains = HashMap::new();
        let mut managed = vec![false; n_moves];
        for mv in &net.movements {
            for (in_id, rest) in &mv.routes {
                let Some(&conn) = rest.first() else { continue };
                let cl = &net.links[conn];
                if cl.conflicts.is_empty() {
                    continue;
                }
                managed[mv.id] = true;
                let l_in = net.links[*in_id].length();
                let l_box = l_in + cl.length();
                let l_end = l_box + rest[1..].iter().map(|&l| net.links[l].length()).sum::<f64>();
                let zones = cl
                    .conflicts
                    .iter()
                    .map(|&cid| {
                        let c = &net.conflicts[cid];
                        let side = if c.a == conn { 0 } else { 1 };
                        let (s, _, _) = c.side(conn);
                        let ((z0, z1), _) = c.zones(conn);
                        ZoneRef { cid, side, x0: l_in + s - z0, x1: l_in + s + z1 }
                    })
                    .collect();
                let v_box = cl.curve_speed.min(cl.speed_limit);
                let v_in = net.links[*in_id].speed_limit;
                lanes.insert(*in_id, LaneInfo { movement: mv.id, zones, l_in, l_box, l_end, v_box, v_in });
                chains.insert(*in_id, (VecDeque::new(), 0));
            }
        }
        let conns = |m: usize| -> Vec<LinkId> { net.movements[m].routes.iter().filter_map(|r| r.1.first().copied()).collect() };
        let mut mconf = vec![vec![false; n_moves]; n_moves];
        for m in 0..n_moves {
            for n in 0..n_moves {
                if m != n && managed[m] && managed[n] {
                    let cn = conns(n);
                    mconf[m][n] = conns(m).iter().any(|&a| {
                        net.links[a].conflicts.iter().any(|&cid| {
                            let c = &net.conflicts[cid];
                            cn.contains(&c.a) || cn.contains(&c.b)
                        })
                    });
                }
            }
        }
        let rings = Self::rings(net, &managed, &mconf);
        let mut mgr = CoopManager {
            policy: GapPolicy::FailSafe,
            scheduling: Scheduling::Batches,
            max_cycle: 100.0,
            slack: 1.5,
            plans: HashMap::new(),
            stats: CoopStats::default(),
            log: Vec::new(),
            history: VecDeque::new(),
            crossings: VecDeque::new(),
            res: vec![[Vec::new(), Vec::new()]; net.conflicts.len()],
            chains,
            moves: vec![MoveState::default(); n_moves],
            rings,
            group: 0,
            lost: 0.0,
            arrivals: vec![0.0; n_moves],
            arr_time: 0.0,
            n_lanes: vec![0; n_moves],
            sat_h: vec![1.0; n_moves],
            managed,
            mconf,
            lanes,
            inside: vec![[0, 0]; net.conflicts.len()],
            last_out: vec![[f64::NEG_INFINITY; 2]; net.conflicts.len()],
        };
        // The mean over the movement's lanes (the two left-turn lanes have different
        // curve speeds), so the result does not depend on the map's iteration order.
        let mut h_sum = vec![0.0; n_moves];
        for l in mgr.lanes.values() {
            mgr.n_lanes[l.movement] += 1;
            h_sum[l.movement] += (4.5 + 2.0 + 0.42 * l.v_box) / l.v_box;
        }
        for (m, h) in h_sum.into_iter().enumerate() {
            if mgr.n_lanes[m] > 0 {
                mgr.sat_h[m] = h / mgr.n_lanes[m] as f64;
            }
        }
        mgr.order_rings();
        mgr
    }

    /// Count a vehicle arriving for movement `m` (at the edge of the map).
    pub(crate) fn note_arrival(&mut self, m: usize) {
        self.arrivals[m] += 1.0;
    }

    /// Measured arrival rate of movement `m` (veh/h).
    pub fn arrival_rate(&self, m: usize) -> f64 {
        if self.arr_time > 0.0 { self.arrivals[m] / self.arr_time * 3600.0 } else { 0.0 }
    }

    /// Longest the batch of movement `m` may run (s): its share of the cycle, in
    /// proportion to its flow ratio (flow per lane times saturation headway) among the
    /// critical phases, as in Webster's split. Until there is any measured demand every
    /// phase may run the whole cycle less the lost time.
    pub fn max_green(&self, m: usize) -> f64 {
        let y = |m: usize| self.arrivals[m] / self.arr_time.max(1.0) / self.n_lanes[m].max(1) as f64 * self.sat_h[m];
        let crit: f64 = (0..2)
            .map(|g| {
                self.rings
                    .iter()
                    .map(|r| r.seq.iter().filter(|x| x.0 == g).map(|x| y(x.1)).sum::<f64>())
                    .fold(0.0, f64::max)
            })
            .sum();
        let avail = (self.max_cycle - self.lost).max(2.0 * MIN_MAX_GREEN);
        if crit <= 1e-9 {
            return avail;
        }
        (avail * y(m) / crit).clamp(MIN_MAX_GREEN, avail)
    }

    /// Does movement `m` cross the box (need reservations)?
    pub fn managed(&self, m: usize) -> bool {
        self.managed[m]
    }
    /// Do movements `m` and `n` cross?
    pub fn conflicts(&self, m: usize, n: usize) -> bool {
        self.mconf[m][n]
    }
    /// Movements currently accepting vehicles.
    pub fn open_movements(&self) -> Vec<usize> {
        (0..self.moves.len()).filter(|&m| self.moves[m].open).collect()
    }
    /// Distance before the stop line where a vehicle without a slot waits on `lane`: far
    /// enough back to reach the crossing speed by the line once it gets one (m).
    pub fn hold_distance(&self, lane: LinkId, accel: f64) -> Option<f64> {
        self.lanes.get(&lane).map(|l| (l.v_box * l.v_box / (2.0 * accel) + 1.0).clamp(8.0, 90.0))
    }
    /// Reservations currently held (zone-vehicle pairs).
    pub fn reservations(&self) -> usize {
        self.res.iter().map(|r| r[0].len() + r[1].len()).sum()
    }

    pub(crate) fn vehicle_spawned(&mut self, id: u64, lane: LinkId) {
        if let Some(c) = self.chains.get_mut(&lane) {
            c.0.push_back(id);
        }
    }

    pub(crate) fn vehicle_done(&mut self, id: u64, lane: LinkId) {
        self.plans.remove(&id);
        if let Some((q, n)) = self.chains.get_mut(&lane)
            && let Some(pos) = q.iter().position(|&x| x == id)
        {
            q.remove(pos);
            if pos < *n {
                *n -= 1;
            }
        }
    }

    /// The vehicle ahead of each vehicle in its managed lane.
    pub(crate) fn leaders(&self) -> HashMap<u64, u64> {
        let mut out = HashMap::new();
        for (q, _) in self.chains.values() {
            for w in q.iter().collect::<Vec<_>>().windows(2) {
                out.insert(*w[1], *w[0]);
            }
        }
        out
    }

    fn purge(&mut self, t: f64) {
        for r in self.res.iter_mut() {
            for side in r.iter_mut() {
                side.retain(|x| x.t_out + 5.0 > t);
            }
        }
    }

    /// Earliest line-crossing time at or after `t_line` at which zones held during the
    /// given offsets from the line crossing fit around every reservation; with
    /// `after_all`, the earliest after every conflicting reservation.
    fn earliest(&self, zones: &[(usize, usize, f64, f64, f64)], t_line: f64, after_all: bool) -> f64 {
        let mut iv: Vec<(f64, f64)> = Vec::new();
        for &(cid, side, a, b, need) in zones {
            for r in &self.res[cid][1 - side] {
                let lo = r.t_in - r.need - b;
                let hi = r.t_out + need - a;
                if hi > lo {
                    iv.push((lo, hi));
                }
            }
        }
        if after_all {
            return iv.iter().map(|x| x.1).fold(t_line, f64::max);
        }
        iv.sort_by(|x, y| x.0.total_cmp(&y.0));
        let mut t = t_line;
        for (lo, hi) in iv {
            if lo < t && t < hi {
                t = hi;
            }
        }
        t
    }

    /// Does a trajectory fit around every reservation?
    fn fits(&self, p: &Plan) -> bool {
        p.zones.iter().all(|z| {
            self.res[z.cid][1 - z.side].iter().all(|r| z.t_in >= r.t_out + z.need || r.t_in >= z.t_out + r.need)
        })
    }

    /// Build the earliest trajectory for a request that fits the reservation table.
    /// Also returns the number of trajectories simulated.
    fn plan_search(&self, r: &Req) -> (Option<Plan>, u64) {
        let mut target = r.t;
        for n in 1..=12 {
            let Some(p) = simulate(r, target) else { return (None, n) };
            let rel: Vec<_> = p.zones.iter().map(|z| (z.cid, z.side, z.t_in - p.t_line, z.t_out - p.t_line, z.need)).collect();
            let t_ok = self.earliest(&rel, p.t_line, r.opens_batch);
            if t_ok <= p.t_line + 1e-9 {
                if self.fits(&p) {
                    return (Some(p), n);
                }
                target = p.t_line + PAD;
            } else {
                target = target.max(t_ok + PAD);
            }
        }
        (None, 12)
    }

    fn grant(&mut self, c: &Cand, p: Plan) {
        for z in &p.zones {
            self.res[z.cid][z.side].push(Res { t_in: z.t_in, t_out: z.t_out, need: z.need });
        }
        let st = &mut self.moves[c.movement];
        if st.open {
            if st.granted == 0 {
                st.first_t = p.t_line;
                st.frontier = p.t_line;
                self.stats.batches += 1;
            } else {
                st.frontier = st.frontier.max(p.t_line);
            }
            st.granted += 1;
        }
        if let Some((_, n)) = self.chains.get_mut(&c.lane) {
            *n += 1;
        }
        self.crossings.push_back((p.t_line, c.movement));
        self.plans.insert(c.vid, p);
        self.stats.plans += 1;
    }

    /// Vehicles of movement `m` on the map without a trajectory yet.
    fn waiting(&self, m: usize) -> usize {
        self.lanes
            .iter()
            .filter(|(_, l)| l.movement == m)
            .filter_map(|(lane, _)| self.chains.get(lane))
            .map(|(q, n)| q.len() - n)
            .sum()
    }

    /// Minimum line-crossing headway behind another vehicle of the lane at the box
    /// speed.
    fn headway(av: &Av, lead_len: f64, v_box: f64) -> f64 {
        (lead_len + av.s0 + av.tg * v_box) / v_box.max(1.0)
    }

    fn candidates(&self, veh: &[Vehicle], idx: &HashMap<u64, usize>, t: f64, skip: &[u64]) -> Vec<Vec<Cand>> {
        let mut out = vec![Vec::new(); self.moves.len()];
        let mut lanes: Vec<_> = self.chains.iter().collect();
        lanes.sort_by_key(|x| *x.0);
        for (&lane, (q, n)) in lanes {
            let Some(&vid) = q.get(*n) else { continue };
            if skip.contains(&vid) {
                continue;
            }
            let Some(&i) = idx.get(&vid) else { continue };
            let v = &veh[i];
            if v.ri != 0 {
                continue;
            }
            let info = &self.lanes[&lane];
            let av = Av::of(v);
            let leader = if *n > 0 { q.get(*n - 1).copied() } else { None };
            let d = info.l_in - v.s;
            let mut t_lo = t + fastest(d, v.v, info.v_box, av.v0, av.a, av.b).0;
            if let Some(l) = leader.and_then(|l| self.plans.get(&l))
                && let Some(&j) = leader.and_then(|l| idx.get(&l))
            {
                t_lo = t_lo.max(l.t_line + Self::headway(&av, veh[j].len(), info.v_box));
            }
            let zones: Vec<_> = info
                .zones
                .iter()
                .map(|z| {
                    let a = (z.x0 - info.l_in) / info.v_box;
                    let b = (z.x1 + av.len - info.l_in) / info.v_box;
                    (z.cid, z.side, a, b, self.policy.gap(info.v_box, av.brake))
                })
                .collect();
            let t_est = self.earliest(&zones, t_lo, false);
            let h = Self::headway(&av, v.len(), info.v_box);
            out[info.movement].push(Cand {
                vid,
                idx: i,
                lane,
                movement: info.movement,
                leader,
                t_est,
                h,
            });
        }
        for c in out.iter_mut() {
            c.sort_by(|a, b| a.t_est.total_cmp(&b.t_est));
        }
        out
    }

    fn request<'a>(&'a self, veh: &[Vehicle], c: &Cand, t: f64, k: u64, dt: f64) -> Req<'a> {
        let v = &veh[c.idx];
        let info = &self.lanes[&c.lane];
        let leader = c.leader.and_then(|l| {
            let p = self.plans.get(&l)?;
            let len = veh.iter().find(|w| w.id == l)?.len();
            Some((p, len))
        });
        Req {
            t,
            k,
            dt,
            x: v.s,
            v: v.v,
            av: Av::of(v),
            movement: c.movement,
            lane: c.lane,
            l_in: info.l_in,
            l_box: info.l_box,
            l_end: info.l_end,
            v_box: info.v_box,
            zones: &info.zones,
            leader,
            policy: self.policy,
            opens_batch: false,
        }
    }

    /// The two rings: from each pair of facing approaches (P, Q), one ring serves P's
    /// left turn and Q's through movement, the other Q's left and P's through. Phases
    /// running at the same time never cross. Whether lefts lead or lag is chosen later
    /// from the clearance times ([`CoopManager::order_rings`]).
    fn rings(net: &Network, managed: &[bool], mconf: &[Vec<bool>]) -> Vec<Ring> {
        use super::network::{Compass, Turn};
        let mv = |c: Compass, t: Turn| -> Option<usize> {
            let leg = net.leg_index(c)?;
            net.movement_for(leg, t).filter(|&m| managed[m])
        };
        let mut a = Vec::new();
        let mut b = Vec::new();
        for (g, (p, q)) in [(Compass::N, Compass::S), (Compass::E, Compass::W)].into_iter().enumerate() {
            a.extend([mv(p, Turn::L), mv(q, Turn::T)].into_iter().flatten().map(|m| (g, m)));
            b.extend([mv(q, Turn::L), mv(p, Turn::T)].into_iter().flatten().map(|m| (g, m)));
        }
        for &(g, m) in &a {
            for &(h, n) in &b {
                debug_assert!(g != h || !mconf[m][n], "concurrent phases {m} and {n} cross");
            }
        }
        [a, b]
            .into_iter()
            .filter(|seq| !seq.is_empty())
            .map(|seq| {
                let at_barrier = seq[0].0 != 0;
                Ring { seq, pos: 0, at_barrier }
            })
            .collect()
    }

    /// Choose, for each ring and each side of the barrier, whether the left turn leads or
    /// lags the through movement, to lose the least time to clearances per cycle. The
    /// clearances are very uneven: a left turn crosses the opposing through lanes near
    /// the end of its arc, while those through vehicles reach the crossing early, so a
    /// through movement can follow a left only after about 6 s, but a left can follow
    /// the through movement within a fraction of a second.
    fn order_rings(&mut self) {
        let groups: Vec<Vec<Vec<usize>>> = (0..2)
            .map(|g| self.rings.iter().map(|r| r.seq.iter().filter(|x| x.0 == g).map(|x| x.1).collect()).collect())
            .collect();
        let n_rings = self.rings.len();
        // Bit (g * n_rings + r) set: ring r serves group g in reverse order.
        let seqs = |mask: usize| -> Vec<Vec<(usize, usize)>> {
            (0..n_rings)
                .map(|r| {
                    let mut seq = Vec::new();
                    for (g, group) in groups.iter().enumerate() {
                        let mut v = group[r].clone();
                        if mask >> (g * n_rings + r) & 1 == 1 {
                            v.reverse();
                        }
                        seq.extend(v.into_iter().map(|m| (g, m)));
                    }
                    seq
                })
                .collect()
        };
        let mut best: Option<(f64, usize)> = None;
        for mask in 0..(1usize << (2 * n_rings)) {
            let cost = self.cycle_cost(&seqs(mask));
            if best.is_none_or(|b| cost < b.0 - 1e-9) {
                best = Some((cost, mask));
            }
        }
        let Some((cost, mask)) = best else { return };
        self.lost = cost;
        for (r, seq) in seqs(mask).into_iter().enumerate() {
            self.rings[r].seq = seq;
            self.rings[r].pos = 0;
            self.rings[r].at_barrier = self.rings[r].seq.first().is_none_or(|x| x.0 != self.group);
        }
    }

    /// Clearance time lost per cycle (s) when the rings serve their movements in the
    /// given orders: the clearances between consecutive phases of each ring, plus the
    /// longest clearance across each barrier.
    fn cycle_cost(&self, seqs: &[Vec<(usize, usize)>]) -> f64 {
        let d = |m: usize, n: usize| self.intergreen(m, n).unwrap_or(0.0);
        let part = |r: usize, g: usize| -> Vec<usize> { seqs[r].iter().filter(|x| x.0 == g).map(|x| x.1).collect() };
        let mut cost = 0.0;
        for g in 0..2 {
            for r in 0..seqs.len() {
                cost += part(r, g).windows(2).map(|w| d(w[0], w[1])).sum::<f64>();
            }
            let lasts: Vec<usize> = (0..seqs.len()).filter_map(|r| part(r, g).last().copied()).collect();
            let firsts: Vec<usize> = (0..seqs.len()).filter_map(|r| part(r, 1 - g).first().copied()).collect();
            cost += lasts.iter().flat_map(|&m| firsts.iter().map(move |&n| d(m, n))).fold(0.0, f64::max);
        }
        cost
    }

    /// Switch the gap policy for trajectories granted from now on. Trajectories already
    /// granted keep the gaps they were planned with; the phase order stays.
    pub fn set_policy(&mut self, p: GapPolicy) {
        self.policy = p;
        let seqs: Vec<Vec<(usize, usize)>> = self.rings.iter().map(|r| r.seq.clone()).collect();
        self.lost = self.cycle_cost(&seqs);
    }

    /// Switch the scheduling for slots given out from now on. Vehicles keep the
    /// trajectories they already have; batches in progress end.
    pub fn set_scheduling(&mut self, s: Scheduling) {
        if s == Scheduling::FirstCome {
            for r in 0..self.rings.len() {
                if let Some(m) = self.active(r)
                    && self.moves[m].open
                {
                    self.end_phase(r, PhaseEnd::GapOut);
                }
            }
        }
        self.scheduling = s;
    }

    /// Clearance time lost per cycle with the current phase order (s).
    pub fn lost_time(&self) -> f64 {
        self.lost
    }

    /// Side of the barrier being served: 0 for the north-south approaches, 1 for the
    /// east-west ones.
    pub fn group(&self) -> usize {
        self.group
    }

    /// The phases in progress that have served at least one vehicle, as batches so far.
    pub fn open_batches(&self) -> Vec<Batch> {
        (0..self.moves.len())
            .filter(|&m| self.moves[m].open && self.moves[m].granted > 0)
            .map(|m| {
                let st = &self.moves[m];
                Batch { movement: m, start: st.first_t, end: st.frontier, vehicles: st.granted }
            })
            .collect()
    }

    /// Movements of each ring in service order.
    pub fn ring_order(&self) -> Vec<Vec<usize>> {
        self.rings.iter().map(|r| r.seq.iter().map(|x| x.1).collect()).collect()
    }

    /// Clearance time from the last vehicle of movement `m` crossing the stop line to
    /// the earliest a vehicle of `n` may cross it (s), for cars at the crossing speed;
    /// None if they do not cross.
    pub fn intergreen(&self, m: usize, n: usize) -> Option<f64> {
        let mut best: Option<f64> = None;
        for a in self.lanes.values().filter(|l| l.movement == m) {
            for b in self.lanes.values().filter(|l| l.movement == n) {
                for za in &a.zones {
                    for zb in b.zones.iter().filter(|z| z.cid == za.cid && z.side != za.side) {
                        let out = (za.x1 + 4.5 - a.l_in) / a.v_box;
                        let inn = (zb.x0 - b.l_in) / b.v_box;
                        let d = out + self.policy.gap(b.v_box, 6.0) - inn;
                        best = Some(best.map_or(d, |x: f64| x.max(d)));
                    }
                }
            }
        }
        best
    }

    /// Movement each ring is serving (None while it waits at the barrier).
    pub fn ring_phases(&self) -> Vec<Option<usize>> {
        (0..self.rings.len()).map(|r| self.active(r)).collect()
    }

    fn active(&self, r: usize) -> Option<usize> {
        let ring = &self.rings[r];
        (!ring.at_barrier).then(|| ring.seq[ring.pos].1)
    }

    /// Has the phase of the open movement `m` run its course? It ends when no vehicle of
    /// the movement can reach the line in time to continue its batch (none waiting close
    /// enough, and none that could still enter the approach and get there), or when
    /// continuing would make the batch longer than allowed.
    fn phase_end(&self, m: usize, cands: &[Vec<Cand>], t: f64, missed: &[u64]) -> PhaseEnd {
        let st = &self.moves[m];
        if st.granted == 0 {
            return if cands[m].is_empty() { PhaseEnd::GapOut } else { PhaseEnd::Running };
        }
        let limit = self.limit(m, t);
        let mut end = PhaseEnd::GapOut;
        for (lane, l) in self.lanes.iter().filter(|(_, l)| l.movement == m) {
            // With nobody on the lane, a car could still enter it and come straight through.
            let (next, h) = match cands[m].iter().find(|c| c.lane == *lane) {
                Some(c) if missed.contains(&c.vid) => continue,
                Some(c) => (c.t_est, c.h),
                None => (t + l.l_in / l.v_in, (4.5 + 2.0 + 0.42 * l.v_box) / l.v_box),
            };
            let reach = st.frontier + h + self.slack;
            if next <= reach.min(limit) {
                return PhaseEnd::Running;
            }
            if next <= reach {
                end = PhaseEnd::MaxOut;
            }
        }
        end
    }

    /// Latest line crossing the batch of the open movement `m` may have: its maximum
    /// length after its first crossing. The last phase of a ring before the barrier
    /// may run on while another ring still has phases to serve within their own
    /// maximums, since the barrier waits for that ring anyway.
    fn limit(&self, m: usize, t: f64) -> f64 {
        let own = |m: usize| {
            let st = &self.moves[m];
            (if st.granted > 0 { st.first_t } else { t }) + self.max_green(m)
        };
        let mut limit = own(m);
        let Some(r) = self.rings.iter().position(|ring| !ring.at_barrier && ring.seq[ring.pos].1 == m) else { return limit };
        let ring = &self.rings[r];
        if ring.seq[(ring.pos + 1) % ring.seq.len()].0 == self.group {
            return limit;
        }
        for (q, other) in self.rings.iter().enumerate() {
            if q == r || other.at_barrier {
                continue;
            }
            let rest: f64 = other.seq[other.pos + 1..].iter().take_while(|x| x.0 == self.group).map(|x| self.max_green(x.1)).sum();
            limit = limit.max(own(other.seq[other.pos].1) + rest);
        }
        limit
    }

    /// Grant the next vehicle of the open movement `m` that can continue its batch.
    #[allow(clippy::too_many_arguments)]
    fn serve(&mut self, m: usize, cands: &[Vec<Cand>], veh: &[Vehicle], t: f64, k: u64, dt: f64, missed: &mut Vec<u64>) -> Served {
        for c in &cands[m] {
            if missed.contains(&c.vid) {
                continue;
            }
            let st = &self.moves[m];
            let fresh = st.granted == 0;
            // Latest line crossing that still continues the batch.
            let reach = (st.frontier + c.h + self.slack).min(self.limit(m, t));
            if !fresh && c.t_est > reach {
                continue;
            }
            // A lone vehicle may slip into a gap; a queue is served as a batch.
            let opens_batch = fresh && self.waiting(m) > 1;
            let req = Req { opens_batch, ..self.request(veh, c, t, k, dt) };
            let (plan, sims) = self.plan_search(&req);
            self.stats.sims += sims;
            let Some(plan) = plan else {
                self.stats.failed += 1;
                return Served::Failed(c.vid);
            };
            if !fresh && plan.t_line > reach {
                missed.push(c.vid);
                continue;
            }
            let c = *c;
            self.grant(&c, plan);
            return Served::Granted;
        }
        Served::Nobody
    }

    /// The next vehicle to serve out of turn: the first unscheduled vehicle of a lane
    /// with an emergency vehicle in it (the emergency vehicle itself, or one ahead of
    /// it).
    fn urgent(&self, cands: &[Vec<Cand>], veh: &[Vehicle], idx: &HashMap<u64, usize>) -> Option<Cand> {
        cands
            .iter()
            .flatten()
            .find(|c| {
                let (q, n) = &self.chains[&c.lane];
                q.iter().skip(*n).any(|id| idx.get(id).is_some_and(|&i| veh[i].kind == VKind::Emergency))
            })
            .copied()
    }

    fn open(&mut self, m: usize) {
        self.moves[m] = MoveState { open: true, first_t: f64::NAN, frontier: f64::NAN, granted: 0 };
    }

    fn end_phase(&mut self, r: usize, why: PhaseEnd) {
        let ring = &mut self.rings[r];
        let m = ring.seq[ring.pos].1;
        let st = &mut self.moves[m];
        st.open = false;
        if st.granted > 0 {
            self.history.push_back(Batch { movement: m, start: st.first_t, end: st.frontier, vehicles: st.granted });
        }
        if why == PhaseEnd::MaxOut {
            self.stats.maxed += 1;
        }
        ring.pos = (ring.pos + 1) % ring.seq.len();
        if ring.seq[ring.pos].0 != self.group {
            ring.at_barrier = true;
        }
    }

    /// One manager cycle: grant trajectories to the vehicles that can have one now.
    pub(crate) fn step(&mut self, veh: &[Vehicle], t: f64, k: u64, dt: f64) {
        self.purge(t);
        while self.history.front().is_some_and(|b| b.end < t - HISTORY) {
            self.history.pop_front();
        }
        while self.crossings.front().is_some_and(|c| c.0 < t - HISTORY) {
            self.crossings.pop_front();
        }
        let decay = (-dt / RATE_TAU).exp();
        for a in self.arrivals.iter_mut() {
            *a *= decay;
        }
        self.arr_time = self.arr_time * decay + dt;
        let idx: HashMap<u64, usize> = veh.iter().enumerate().map(|(i, v)| (v.id, i)).collect();
        let mut failed: Vec<u64> = Vec::new();
        let mut missed: Vec<u64> = Vec::new();
        let mut crossings = 0;
        for _ in 0..5000 {
            let cands = self.candidates(veh, &idx, t, &failed);
            // An emergency vehicle, and those ahead of it in its lane, get the earliest
            // slots that fit, whatever the phase.
            if let Some(c) = self.urgent(&cands, veh, &idx) {
                let (plan, sims) = self.plan_search(&self.request(veh, &c, t, k, dt));
                self.stats.sims += sims;
                match plan {
                    Some(p) => {
                        self.grant(&c, p);
                        self.stats.preempted += 1;
                    }
                    None => {
                        self.stats.failed += 1;
                        failed.push(c.vid);
                    }
                }
                continue;
            }
            if self.scheduling == Scheduling::FirstCome {
                // Whoever can reach the line first gets the earliest slot that fits.
                let Some(c) = cands.iter().flatten().min_by(|a, b| a.t_est.total_cmp(&b.t_est).then(a.vid.cmp(&b.vid))).copied() else { break };
                let (plan, sims) = self.plan_search(&self.request(veh, &c, t, k, dt));
                self.stats.sims += sims;
                match plan {
                    Some(p) => self.grant(&c, p),
                    None => {
                        self.stats.failed += 1;
                        failed.push(c.vid);
                    }
                }
                continue;
            }
            // Serve the phases in progress.
            let mut progressed = false;
            for r in 0..self.rings.len() {
                let Some(m) = self.active(r) else { continue };
                if !self.moves[m].open {
                    continue;
                }
                match self.serve(m, &cands, veh, t, k, dt, &mut missed) {
                    Served::Granted => progressed = true,
                    Served::Failed(vid) => {
                        failed.push(vid);
                        progressed = true;
                    }
                    Served::Nobody => {}
                }
                if progressed {
                    break;
                }
            }
            if progressed {
                continue;
            }
            let mut changed = false;
            // End the phases no vehicle can continue.
            for r in 0..self.rings.len() {
                if let Some(m) = self.active(r)
                    && self.moves[m].open
                {
                    let why = self.phase_end(m, &cands, t, &missed);
                    if why != PhaseEnd::Running {
                        self.end_phase(r, why);
                        changed = true;
                    }
                }
            }
            // Cross the barrier once every ring is done with this group.
            if self.rings.iter().all(|r| r.at_barrier) {
                if crossings == 2 {
                    break;
                }
                crossings += 1;
                self.stats.barriers += 1;
                self.group = 1 - self.group;
                for ring in self.rings.iter_mut() {
                    ring.at_barrier = ring.seq[ring.pos].0 != self.group;
                }
                changed = true;
            }
            // Start the phases now due.
            for r in 0..self.rings.len() {
                if let Some(m) = self.active(r)
                    && !self.moves[m].open
                {
                    self.open(m);
                    changed = true;
                }
            }
            if !changed {
                break;
            }
        }
    }

    /// Check what actually happened during the step just taken against the plans.
    /// `v_old` holds each vehicle's speed at the start of the step.
    pub(crate) fn observe(&mut self, net: &Network, veh: &[Vehicle], v_old: &[f64], t: f64, k: u64, dt: f64) {
        for (i, v) in veh.iter().enumerate() {
            let off = |ri: usize| v.route[..ri].iter().map(|&l| net.links[l].length()).sum::<f64>();
            let base = off(v.ri);
            let (x, xp) = (base + v.s, base + v.prev_s);
            let vo = v_old[i];
            let Some(p) = self.plans.get(&v.id) else {
                // Entering the box without a reservation.
                if v.ri >= 1 && v.ri - 1 < v.route.len() && self.lanes.contains_key(&v.route[0]) {
                    let l_in = net.links[v.route[0]].length();
                    if xp < l_in && x >= l_in {
                        self.stats.unauthorized += 1;
                        if self.log.len() < 40 {
                            self.log.push(format!("t={t:.1} vehicle #{} entered the box without a reservation", v.id));
                        }
                    }
                }
                continue;
            };
            let j = (k - p.k0) as usize;
            if j + 1 < p.x.len() && ((p.x[j + 1] - x).abs() > 1e-6 || (p.v[j + 1] - v.v).abs() > 1e-9) {
                self.stats.deviations += 1;
                if self.log.len() < 40 {
                    self.log.push(format!(
                        "t={t:.1} vehicle #{} off its trajectory: x {:.4} vs {:.4}, v {:.4} vs {:.4}",
                        v.id, x, p.x[j + 1], v.v, p.v[j + 1]
                    ));
                }
            }
            let l_in = net.links[v.route[0]].length();
            let mut err = 0.0_f64;
            if xp < l_in && x >= l_in {
                err = err.max((t + cross_time(xp, vo, v.a, l_in, dt) - p.t_line).abs());
            }
            let len = v.len();
            let mut events: Vec<(usize, usize, bool, f64, f64)> = Vec::new();
            for z in &p.zones {
                if xp < z.x0 && x >= z.x0 {
                    let ti = t + cross_time(xp, vo, v.a, z.x0, dt);
                    err = err.max((ti - z.t_in).abs());
                    events.push((z.cid, z.side, true, ti, z.need));
                }
                let xr = z.x1 + len;
                if xp < xr && x >= xr {
                    let to = t + cross_time(xp, vo, v.a, xr, dt);
                    err = err.max((to - z.t_out).abs());
                    events.push((z.cid, z.side, false, to, 0.0));
                }
            }
            self.stats.timing_err_max = self.stats.timing_err_max.max(err);
            for (cid, side, entering, time, need) in events {
                if entering {
                    let other = 1 - side;
                    if self.inside[cid][other] > 0 {
                        self.stats.zone_violations += 1;
                        if self.log.len() < 40 {
                            self.log.push(format!("t={t:.1} vehicle #{} entered zone {cid} while it was occupied", v.id));
                        }
                    }
                    let pet = time - self.last_out[cid][other];
                    if pet < PET_WINDOW {
                        let s = &mut self.stats;
                        s.interactions += 1;
                        s.pet_min = s.pet_min.min(pet);
                        s.margin_min = s.margin_min.min(pet - need);
                        let b = ((pet / PET_BIN) as usize).min(s.pet_hist.len() - 1);
                        s.pet_hist[b] += 1;
                    }
                    self.inside[cid][side] += 1;
                } else {
                    self.inside[cid][side] = self.inside[cid][side].saturating_sub(1);
                    self.last_out[cid][side] = self.last_out[cid][side].max(time);
                }
            }
        }
    }
}

// ---------------------------------------------------------------- vehicle control

impl Sim {
    /// Accelerations for the cooperative layout: granted vehicles replay their
    /// trajectories; the rest drive toward their holding point behind the vehicle ahead.
    pub(super) fn coop_accels(&mut self) -> Vec<f64> {
        let Some(coop) = self.coop.as_ref() else { return vec![0.0; self.vehicles.len()] };
        let idx: HashMap<u64, usize> = self.vehicles.iter().enumerate().map(|(i, v)| (v.id, i)).collect();
        let leaders = coop.leaders();
        let mut out: Vec<(f64, Reason)> = Vec::with_capacity(self.vehicles.len());
        for (i, v) in self.vehicles.iter().enumerate() {
            if let Some(p) = coop.plans.get(&v.id) {
                let j = (self.steps - p.k0) as usize;
                if j < p.a.len() {
                    out.push((p.a[j], p.why[j]));
                    continue;
                }
            }
            out.push(self.free_av_acc(i, leaders.get(&v.id).and_then(|l| idx.get(l)).copied(), coop));
        }
        let mut accs = Vec::with_capacity(out.len());
        for (v, (acc, why)) in self.vehicles.iter_mut().zip(out) {
            v.reason = why;
            v.blocked = false;
            v.yield_to = None;
            accs.push(acc);
        }
        accs
    }

    /// An automated vehicle without a trajectory: cruise, follow the vehicle ahead, and
    /// (in a managed lane) stop at the holding point; slow for any curve ahead.
    fn free_av_acc(&self, i: usize, chain_leader: Option<usize>, coop: &CoopManager) -> (f64, Reason) {
        let v = &self.vehicles[i];
        let av = Av::of(v);
        let link = &self.net.links[v.link()];
        let vlim = if link.kind.in_box() { av.v0.min(link.curve_speed) } else { av.v0 };
        let mut acc = av.a * (1.0 - (v.v / vlim.max(0.1)).powi(4));
        let mut why = Reason::Free;
        let lead = match chain_leader {
            Some(j) => {
                let w = &self.vehicles[j];
                let gap = route_x(&self.net, w) - w.len() - route_x(&self.net, v);
                Some((gap, w.v, w.a))
            }
            None => self.leader(i).map(|(gap, vl, j)| (gap, vl, self.vehicles[j].a)),
        };
        if let Some((gap, vl, al)) = lead {
            let a2 = follow_acc(&av, v.v, gap, vl, al);
            if a2 < acc {
                acc = a2;
                why = Reason::Following;
            }
        }
        if v.ri == 0
            && let Some(hold) = coop.hold_distance(v.route[0], av.a)
        {
            let l_in = link.length();
            let mut gap = l_in - hold - v.s;
            if gap < 0.3 {
                gap = l_in - 1.0 - v.s;
            }
            let s_star = 0.5 + v.v * 0.8 + v.v * v.v / (2.0 * (av.a * av.b).sqrt());
            let a2 = av.a * (1.0 - (v.v / av.v0.max(0.1)).powi(4) - (s_star / gap.max(0.05)).powi(2));
            if a2 < acc {
                acc = a2;
                why = Reason::Awaiting;
            }
        }
        // Curves ahead (slip lanes).
        let mut d = link.length() - v.s;
        for &nl in &v.route[v.ri + 1..] {
            let l2 = &self.net.links[nl];
            if l2.kind.in_box() && l2.curve_speed < v.v {
                let a2 = -(v.v * v.v - l2.curve_speed * l2.curve_speed) / (2.0 * (d - 1.0).max(0.5));
                if a2 < acc {
                    acc = a2;
                    why = Reason::Curve;
                }
            }
            d += l2.length();
            if d > 80.0 {
                break;
            }
        }
        (acc.clamp(-6.0, av.a), why)
    }

    /// Run the intersection manager for this step.
    pub(super) fn coop_control(&mut self) {
        if let Some(mut c) = self.coop.take() {
            c.step(&self.vehicles, self.t, self.steps, self.dt);
            self.coop = Some(c);
        }
    }

    /// Switch the gap policy for trajectories granted from now on.
    pub fn set_gap_policy(&mut self, p: GapPolicy) {
        if let Some(c) = self.coop.as_mut() {
            c.set_policy(p);
        }
    }

    /// Switch the order in which the manager gives out slots from now on.
    pub fn set_scheduling(&mut self, s: Scheduling) {
        if let Some(c) = self.coop.as_mut() {
            c.set_scheduling(s);
        }
    }

    pub fn is_cooperative(&self) -> bool {
        self.mode == ControlMode::Cooperative
    }
}

/// Distance of a vehicle's front from the start of its route.
fn route_x(net: &Network, v: &Vehicle) -> f64 {
    v.route[..v.ri].iter().map(|&l| net.links[l].length()).sum::<f64>() + v.s
}

/// True if the link is a managed path through the box.
pub fn is_box_path(net: &Network, l: LinkId) -> bool {
    net.links[l].kind == LinkKind::Connector && !net.links[l].conflicts.is_empty()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Discretize a profile the way the planner does and integrate it.
    fn run(d: f64, v: f64, tau: f64, ve: f64) -> (f64, f64) {
        let (dt, a, b, vtop) = (0.1, 2.0, 2.5, 13.9);
        let (p, _) = approach_profile(d, v, tau, ve, vtop, a, b);
        let (mut x, mut vv, mut t, mut el) = (0.0, v, 0.0, 0.0);
        for _ in 0..20000 {
            let acc = ((v + p.dv(el + dt)).max(0.0) - vv) / dt;
            el += dt;
            let (ds, vn) = integrate(vv, acc, dt);
            if x + ds >= d {
                let tc = cross_time(x, vv, acc, d, dt);
                return (t + tc, vv + acc * tc);
            }
            x += ds;
            vv = vn;
            t += dt;
        }
        (f64::INFINITY, 0.0)
    }

    #[test]
    fn profiles_arrive_on_time_at_speed() {
        // (distance, speed, time available) across the regimes: on time, slowed, crawling,
        // stopping and waiting.
        for &(d, v, tau) in &[
            (200.0, 13.9, 16.0),
            (200.0, 13.9, 20.0),
            (200.0, 13.9, 30.0),
            (200.0, 13.9, 60.0),
            (200.0, 13.9, 150.0),
            (120.0, 8.0, 25.0),
            (60.0, 0.0, 12.0),
            (40.0, 0.0, 9.0),
            (32.0, 0.0, 30.0),
            (90.0, 11.1, 9.0),
        ] {
            let (tf, _, _) = fastest(d, v, 11.11, 13.9, 2.0, 2.5);
            let (t, ve) = run(d, v, tau, 11.11);
            let want = tau.max(tf);
            assert!((t - want).abs() < 0.06, "d={d} v={v} tau={tau}: crossed at {t:.3}, wanted {want:.3}");
            assert!(ve > 11.11 - 0.15 && ve < 11.11 + 0.05, "d={d} v={v} tau={tau}: crossed at {ve:.2} m/s");
        }
    }

    #[test]
    fn short_run_up_enters_slower_but_on_time() {
        // At rest 20 m from the line a car cannot reach 40 km/h: it leaves on time and
        // crosses as fast as the distance allows.
        let (t, ve) = run(20.0, 0.0, 15.0, 11.11);
        assert!((t - 15.0).abs() < 0.06, "crossed at {t:.3}");
        assert!((ve - (2.0f64 * 2.0 * 20.0).sqrt()).abs() < 0.1, "crossed at {ve:.2} m/s");
    }

    #[test]
    fn gap_policies() {
        let v = 40.0 / 3.6;
        let p2 = GapPolicy::FailSafe.gap(v, 6.0);
        let ph = GapPolicy::Mitigated.gap(v, 6.0);
        let p1 = GapPolicy::Nominal.gap(v, 6.0);
        assert!((p1 - 0.5).abs() < 1e-12);
        assert!((p2 - (0.5 + v / 12.0)).abs() < 1e-12);
        assert!(p1 < ph && ph < p2);
        // Below the tolerated impact speed the mitigated policy needs only the latency.
        assert!((GapPolicy::Mitigated.gap(5.0, 6.0) - 0.5).abs() < 1e-12);
        // Trucks brake less hard, so they need more.
        assert!(GapPolicy::FailSafe.gap(v, brake_decel(VKind::Truck)) > p2);
    }

    #[test]
    fn following_law_holds_the_time_gap() {
        // A follower behind a leader that brakes to a stop and pulls away again never
        // closes within a meter and settles at the constant time gap.
        let av = Av { len: 4.5, a: 2.0, b: 2.5, s0: 2.0, tg: 0.42, v0: 13.9, brake: 6.0 };
        let dt = 0.1;
        let (mut xl, mut vl) = (40.0, 11.0);
        let (mut x, mut v) = (0.0, 13.0);
        let mut min_gap = f64::INFINITY;
        for k in 0..1200 {
            let t = k as f64 * dt;
            let al: f64 = if t < 5.0 { 0.0 } else if t < 9.4 { -2.5 } else if t < 20.0 { 0.0 } else { 1.5 };
            let al = if vl + al * dt > 11.1 && t > 20.0 { 0.0 } else { al };
            let gap = xl - 4.5 - x;
            min_gap = min_gap.min(gap);
            let acc = follow_acc(&av, v, gap, vl, al).clamp(-6.0, 2.0);
            let (dsl, vln) = integrate(vl, al, dt);
            let (ds, vn) = integrate(v, acc, dt);
            xl += dsl;
            vl = vln;
            x += ds;
            v = vn;
        }
        let gap = xl - 4.5 - x;
        assert!(min_gap > 1.0, "closed to {min_gap:.2} m");
        assert!((gap - (2.0 + 0.42 * v)).abs() < 0.3, "settled at {gap:.2} m at {v:.2} m/s");
    }
}

