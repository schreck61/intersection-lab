// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Failures at the cooperative crossing: what happens when a vehicle stops dead or
//! brakes to a stop, which the safety gaps are sized for but normal running never
//! shows.
//!
//! * **Failing.** A vehicle is made to fail where it is (`Sim::stall_vehicle`), at a
//!   place the next vehicle of a lane reaches (`Sim::stall_at`, so that another run can
//!   be given the same failure), at random places in the box, or at the worst moment
//!   for crossing traffic (`Sim::stall_at_worst`): when it is about to leave a zone
//!   with a crossing vehicle due right behind it at the smallest gap its policy allows.
//!   It is disabled from then on and stays where it stops until it is cleared away.
//! * **The reaction.** The manager knows every trajectory, so it finds every vehicle a
//!   failure endangers at once, but no vehicle responds sooner than [`REACTION`], which
//!   every gap policy allows for: until then each keeps to the trajectory it has. A
//!   vehicle that can still stop short of the box has its crossing withdrawn and stops
//!   there. One that cannot, with the failed vehicle in a zone on its path, brakes as
//!   hard as it can; one behind a vehicle that left its trajectory follows it until it
//!   rests behind it. Each of these is itself a trajectory, built and checked like any
//!   other, and may endanger further vehicles, so the reaction spreads until every
//!   trajectory fits the others again. Vehicles whose paths stay clear cross as
//!   planned.
//! * **A reaction only holds a vehicle back.** One at rest stays at rest, and one that
//!   can no longer stop short of the box is never faster than the trajectory it had:
//!   the zones ahead of it are its own only from the times that trajectory reaches
//!   them. One that cannot stop behind the vehicle ahead is expected to come to rest
//!   against it, so the zones it will stand in are held from the failure on.
//! * **Contact.** Whether a vehicle stops in time is physics, not planning: with the
//!   fail-safe gap it always can, with the thinner gaps not always. Vehicles that touch
//!   stop where they touch and are disabled too. Nothing of a collision is simulated
//!   beyond that; its closing speed is the measure of it.
//! * **Recovery.** A vehicle at rest holds the zones it stands in, so nothing is
//!   scheduled across them, while the lanes that stay clear are served as usual.
//!   Disabled vehicles are cleared away after `clear_time`. A vehicle that stopped past
//!   the line is granted a way out as soon as its way is clear, ahead of every phase;
//!   it may pass another vehicle at rest slowly if their bodies stay apart. Should
//!   vehicles at rest only block one another once nothing disabled is left, one of them
//!   is towed.

use super::{
    ACTUATION, Av, CoopManager, GapPolicy, LATENCY, PAD, PLAN_DECEL, Plan, PlanKind, PlanZone, Req, ZoneRef,
    brake_decel, cross_time, follow_acc, route_x, simulate,
};
use crate::sim::network::{LinkId, LinkKind, Network};
use crate::sim::rng::Rng;
use crate::sim::vtypes::VKind;
use crate::sim::{Reason, Sim, Vehicle, body_on, integrate, obb_overlap};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, VecDeque};

/// Time from a failure until the brakes of the vehicles it endangers respond: the
/// failed vehicle's report, the manager's alert and the actuation delay (s). Every gap
/// policy allows this much.
pub const REACTION: f64 = 2.0 * LATENCY + ACTUATION;
/// Time a contingency keeps between a vehicle that has left its trajectory and the next
/// one through a zone: the timing tolerance of the two vehicles (s).
const TOLERANCE: f64 = 2.0 * LATENCY;
/// Room a moving vehicle must have before the stop line, beyond its braking distance,
/// for its crossing to be withdrawn there instead of ending in an emergency stop (m).
/// One that stands before the line needs none.
const HALT_ROOM: f64 = 2.0;
/// Where a vehicle whose crossing was withdrawn stops if it is already past its holding
/// point: this far before the line (m).
pub(super) const HALT_GAP: f64 = 1.0;
/// Speed below which a vehicle counts as standing (m/s): braking to rest can leave a
/// rounding error of that order.
const AT_REST: f64 = 1e-9;
/// Longest a contingency may take to bring a vehicle to rest behind another (steps).
const MAX_CONTINGENCY_STEPS: usize = 1200;
/// How long vehicles stopped in the box may block one another, once nothing disabled
/// is left, before one of them is towed (s).
const STUCK_LIMIT: f64 = 15.0;
/// Failures kept in the log.
const INCIDENT_LOG: usize = 50;
/// Room a vehicle leaving the box from rest keeps to the body of a vehicle at rest that
/// it passes (m), and the most it does in the box while it passes one (20 km/h).
const PASS_ROOM: f64 = 0.5;
const PASS_SPEED: f64 = 20.0 / 3.6;
/// How far over its policy's gap a crossing vehicle may follow and still count as
/// following at the smallest gap, when a failure is timed for the worst moment (s).
const WORST_SLACK: f64 = 0.3;

/// How a vehicle fails.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Fault {
    /// It stops where it is, at once, as after a collision or a seized driveline: the
    /// failure the fail-safe and mitigated gaps are sized for.
    Dead,
    /// It loses drive and brakes as hard as it can to a stop: the failure the nominal
    /// gap is sized for.
    Braking,
}

impl Fault {
    pub const ALL: [Fault; 2] = [Fault::Dead, Fault::Braking];
    pub fn name(self) -> &'static str {
        match self {
            Fault::Dead => "Stops dead",
            Fault::Braking => "Brakes hard to a stop",
        }
    }
    /// A few words for labels.
    pub fn label(self) -> &'static str {
        match self {
            Fault::Dead => "stopped dead",
            Fault::Braking => "braked to a stop",
        }
    }
}

/// A vehicle that has failed, or was struck, and stays where it stops until it is
/// cleared away.
#[derive(Clone, Copy, Debug)]
pub struct Wreck {
    pub since: f64,
    pub clear_at: f64,
    pub fault: Fault,
    /// Index of its failure in the log.
    pub incident: u64,
}

/// A place where the next vehicle of a lane to arrive fails: `x` is the route
/// coordinate of its front, measured from the start of the lane.
#[derive(Clone, Copy, Debug)]
pub struct Trap {
    pub lane: LinkId,
    pub x: f64,
    pub fault: Fault,
}

/// Two vehicles touching: when, where, how fast they closed, and who ran into whom.
#[derive(Clone, Copy, Debug)]
pub struct Impact {
    pub t: f64,
    /// Closing speed (m/s).
    pub speed: f64,
    /// Between vehicles on crossing paths; otherwise a vehicle ran into the one ahead
    /// of it in its own lane.
    pub crossing: bool,
    pub striker: u64,
    pub struck: u64,
    /// Where they touched, in the intersection's coordinates.
    pub at: (f64, f64),
}

/// Where a vehicle failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Place {
    Approach,
    Box,
    Exit,
}

impl Place {
    pub fn label(self) -> &'static str {
        match self {
            Place::Approach => "on the approach",
            Place::Box => "in the box",
            Place::Exit => "past the box",
        }
    }
}

/// One failure and what followed from it.
#[derive(Clone, Debug)]
pub struct Incident {
    pub id: u64,
    pub t: f64,
    pub vehicle: u64,
    pub kind: VKind,
    pub fault: Fault,
    pub movement: usize,
    /// Its inbound lane, and how far along its route from the start of that lane its
    /// front was when it failed (m).
    pub lane: LinkId,
    pub x: f64,
    pub place: Place,
    /// Gap policy in force when it happened.
    pub policy: GapPolicy,
    /// Vehicles that braked to an emergency stop, or behind one that did.
    pub stops: u32,
    /// Crossings withdrawn from vehicles that could still stop short of the box.
    pub withdrawn: u32,
    pub impacts: Vec<Impact>,
    /// How far short of the zone it braked for the closest emergency stop came to rest
    /// (m); negative if a vehicle could not stop short of it.
    pub margin: f64,
    /// When the last vehicle that braked for it came to rest.
    pub settled: f64,
    /// When its wrecks were cleared away.
    pub cleared: Option<f64>,
}

impl Incident {
    /// Fastest contact between vehicles on crossing paths, and between vehicles in one
    /// lane (m/s).
    pub fn worst(&self) -> (Option<f64>, Option<f64>) {
        let max = |crossing: bool| {
            self.impacts.iter().filter(|i| i.crossing == crossing).map(|i| i.speed).reduce(f64::max)
        };
        (max(true), max(false))
    }
}

/// How a vehicle that can no longer stop short of the box reacts to a failure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Law {
    /// Brake as hard as it can.
    Stop,
    /// Follow the vehicle ahead in its lane.
    Follow,
}

/// What the manager knows when it reacts to a failure.
struct Alert<'a> {
    net: &'a Network,
    veh: &'a [Vehicle],
    idx: &'a HashMap<u64, usize>,
    t: f64,
    k: u64,
    dt: f64,
    /// Steps of the reaction time.
    steps: u64,
    /// The failure this is part of.
    incident: Option<u64>,
}

/// A trajectory under construction: every step is taken with the simulator's own
/// integrator, so the vehicle replays it exactly.
struct Traj {
    x: Vec<f64>,
    v: Vec<f64>,
    a: Vec<f64>,
    why: Vec<Reason>,
    /// The step at which the vehicle is expected to touch the one ahead of it, if it
    /// cannot stop behind it, and that vehicle's rear then (`Plan::contact`).
    contact: Option<(usize, f64)>,
}

impl Traj {
    fn start(x: f64, v: f64) -> Traj {
        Traj { x: vec![x], v: vec![v], a: Vec::new(), why: Vec::new(), contact: None }
    }
    fn state(&self) -> (f64, f64) {
        (self.x[self.a.len()], self.v[self.a.len()])
    }
    fn push(&mut self, acc: f64, why: Reason, dt: f64) {
        let (x, v) = self.state();
        let (ds, vn) = integrate(v, acc, dt);
        self.x.push(x + ds);
        self.v.push(vn);
        self.a.push(acc);
        self.why.push(why);
    }
    /// Note the first step so far at which the vehicle ahead is in the way: `lead` is
    /// that vehicle's trajectory and length, and `k0` the step this one starts at.
    fn meet(&mut self, k0: u64, lead: Option<(&Plan, f64)>) {
        if self.contact.is_none() {
            self.contact = lead.and_then(|(p, len)| met(p, len, k0, &self.x[..=self.a.len()], 0));
        }
    }
    /// Brake at `b` until at rest.
    fn stop(&mut self, b: f64, dt: f64) {
        self.stop_behind(b, dt, 0, None);
    }
    /// Brake at `b` until at rest, noting where the vehicle ahead is in the way before
    /// that: `lead` is that vehicle's trajectory and length, and `k0` the step this one
    /// starts at.
    fn stop_behind(&mut self, b: f64, dt: f64, k0: u64, lead: Option<(&Plan, f64)>) {
        while self.state().1 > 0.0 {
            if self.contact.is_none() {
                let n = k0 + self.a.len() as u64;
                let rear = lead.and_then(|(p, len)| p.state(n).map(|s| s.0 - len));
                self.contact = rear.filter(|rear| *rear <= self.state().0).map(|rear| (self.a.len(), rear));
            }
            self.push(-b, Reason::Emergency, dt);
        }
    }
}

/// The first step from `from` on at which a vehicle whose front is at `x[j]` at step
/// `k0 + j` has reached the rear of the vehicle ahead, whose trajectory is `lead` and
/// whose length is `len`: that step, and where the rear then is.
fn met(lead: &Plan, len: f64, k0: u64, x: &[f64], from: usize) -> Option<(usize, f64)> {
    (from..x.len()).find_map(|j| {
        let rear = lead.state(k0 + j as u64)?.0 - len;
        (rear <= x[j]).then_some((j, rear))
    })
}

/// When a trajectory that starts at `t0` holds each zone it has not yet left: from its
/// front entering to its rear leaving, forever if it comes to rest inside. `known`
/// gives the entry time and speed into a zone the vehicle is already in. Zones the
/// trajectory never reaches are left out. A vehicle expected to run into the one ahead
/// holds without end every zone it has not left where it then stands: it will stand in
/// one of them.
fn zone_times(
    zones: &[ZoneRef],
    tr: &Traj,
    (t0, dt): (f64, f64),
    (len, brake): (f64, f64),
    policy: GapPolicy,
    known: impl Fn(usize) -> Option<(f64, f64)>,
) -> Vec<PlanZone> {
    let mut out = Vec::new();
    for z in zones {
        let xr = z.x1 + len;
        if tr.x[0] >= xr {
            continue;
        }
        let mut entry = (tr.x[0] >= z.x0).then(|| known(z.cid).unwrap_or((t0, tr.v[0])));
        let mut t_out = f64::INFINITY;
        let mut t = t0;
        for j in 0..tr.a.len() {
            let (x, xn, v, acc) = (tr.x[j], tr.x[j + 1], tr.v[j], tr.a[j]);
            if entry.is_none() && x < z.x0 && xn >= z.x0 {
                let tau = cross_time(x, v, acc, z.x0, dt);
                entry = Some((t + tau, v + acc * tau));
            }
            if x < xr && xn >= xr && tr.contact.is_none_or(|c| xr <= c.1) {
                t_out = t + cross_time(x, v, acc, xr, dt);
                break;
            }
            t += dt;
        }
        if let Some((t_in, v_in)) = entry {
            out.push(PlanZone {
                cid: z.cid,
                side: z.side,
                x0: z.x0,
                x1: z.x1,
                t_in,
                t_out,
                v_in,
                need: policy.gap(v_in, brake),
            });
        }
    }
    out
}

impl CoopManager {
    /// Seed the stream that places random failures, so that runs with the same seed
    /// (the two sides of a comparison among them) get the same failures.
    pub(crate) fn seed_failures(&mut self, seed: u64) {
        self.rng = Rng::new(seed ^ 0x57A1_1ED0);
    }

    /// Is a disabled vehicle on the map, or a vehicle off its crossing because of one?
    pub fn alarm(&self) -> bool {
        self.alarm
    }

    /// Was this vehicle's crossing withdrawn, so that it must stop short of the box?
    pub fn halted(&self, vid: u64) -> bool {
        self.halted.contains(&vid)
    }

    /// Make a vehicle fail when the manager next runs.
    pub(crate) fn stall(&mut self, vid: u64, fault: Fault) {
        self.pending.push((vid, fault, None));
    }

    /// Make the next vehicle of `lane` whose front reaches route coordinate `x` fail
    /// there. False if the lane already has such a place.
    pub fn trap(&mut self, lane: LinkId, x: f64, fault: Fault) -> bool {
        if self.traps.iter().any(|t| t.lane == lane) {
            return false;
        }
        self.traps.push(Trap { lane, x, fault });
        true
    }

    /// Stop line and far edge of the box on a managed lane, as route coordinates.
    pub fn box_span(&self, lane: LinkId) -> Option<(f64, f64)> {
        self.lanes.get(&lane).map(|l| (l.l_in, l.l_box))
    }

    /// Clear every disabled vehicle away when the manager next runs.
    pub fn clear_wrecks(&mut self) {
        for w in self.wrecks.values_mut() {
            w.clear_at = f64::NEG_INFINITY;
        }
    }

    /// Random failures: when one is due, the next vehicle to reach a random spot in the
    /// box, on a random lane, fails there.
    pub(crate) fn arm_random(&mut self, t: f64) {
        if self.stall_rate <= 0.0 {
            self.next_stall = None;
            return;
        }
        let rate = self.stall_rate / 3600.0;
        let due = match self.next_stall {
            Some(due) => due,
            None => {
                let due = t + self.rng.exp(rate);
                self.next_stall = Some(due);
                due
            }
        };
        if t < due {
            return;
        }
        self.next_stall = Some(t + self.rng.exp(rate));
        let mut lanes: Vec<(LinkId, f64, f64)> =
            self.lanes.iter().map(|(lane, l)| (*lane, l.l_in, l.l_box)).collect();
        lanes.sort_by_key(|l| l.0);
        let (lane, l_in, l_box) = lanes[self.rng.below(lanes.len())];
        let x = self.rng.range(l_in, l_box);
        self.trap(lane, x, self.stall_fault);
    }

    /// While armed: the vehicle whose failure right now would leave a crossing vehicle
    /// the least room. It is within a quarter of a second of clearing a zone, and the
    /// next vehicle through that zone follows at its policy's gap, give or take
    /// `WORST_SLACK`. None until there is such a pair.
    fn worst_moment(&self, veh: &[Vehicle], t: f64, dt: f64) -> Option<u64> {
        let mut worst: Option<(f64, u64)> = None;
        for v in veh.iter().filter(|v| !v.disabled) {
            let Some(p) = self.plans.get(&v.id).filter(|p| p.crossing()) else { continue };
            for z in &p.zones {
                let left = z.t_out - t;
                if left <= 0.5 * dt || left > 2.5 * dt {
                    continue;
                }
                let next = self.res[z.cid][1 - z.side]
                    .iter()
                    .filter(|r| r.t_in > z.t_out && self.plans.get(&r.vid).is_some_and(|q| q.crossing()))
                    .min_by(|a, b| a.t_in.total_cmp(&b.t_in));
                let Some(slack) = next.map(|r| r.t_in - z.t_out - r.need) else { continue };
                if slack <= WORST_SLACK && worst.is_none_or(|w| (slack, v.id) < w) {
                    worst = Some((slack, v.id));
                }
            }
        }
        worst.map(|w| w.1)
    }

    /// If armed and this is the moment, make the vehicle whose failure leaves a
    /// crossing vehicle the least room fail.
    pub(crate) fn spring(&mut self, veh: &[Vehicle], t: f64, dt: f64) {
        if let Some(fault) = self.armed
            && let Some(vid) = self.worst_moment(veh, t, dt)
        {
            self.armed = None;
            self.stall(vid, fault);
        }
    }

    /// Vehicles to take off the map now: wrecks whose time is up, and a vehicle towed
    /// out of a standoff.
    pub(crate) fn take_removals(&mut self, t: f64) -> Vec<u64> {
        let mut ids: Vec<u64> =
            self.wrecks.iter().filter(|(_, w)| w.clear_at <= t).map(|(id, _)| *id).collect();
        ids.append(&mut self.remove);
        ids.sort_unstable();
        ids.dedup();
        ids
    }

    /// Vehicles that failed or were struck since the manager last ran, each with its
    /// failure and the incident it belongs to, if that is known.
    pub(crate) fn take_pending(&mut self) -> Vec<(u64, Fault, Option<u64>)> {
        std::mem::take(&mut self.pending)
    }

    /// A vehicle was taken off the map where it stood: it gives up the zones it held.
    pub(crate) fn vehicle_removed(&mut self, net: &Network, v: &Vehicle, t: f64) {
        let lane = v.route[0];
        if let Some(info) = self.lanes.get(&lane) {
            let x = route_x(net, v);
            for z in info.zones.iter().filter(|z| x >= z.x0 && x < z.x1 + v.len()) {
                let n = &mut self.inside[z.cid][z.side];
                *n = n.saturating_sub(1);
                let out = &mut self.last_out[z.cid][z.side];
                *out = out.max(t);
            }
        }
        self.drop_plan(v.id);
        if let Some(w) = self.wrecks.remove(&v.id) {
            self.stats.cleared += 1;
            if let Some(inc) = self.incidents.iter_mut().find(|i| i.id == w.incident) {
                inc.cleared = Some(t);
            }
        }
        self.vehicle_done(v.id, lane);
        self.recount(lane);
    }

    /// Two vehicles have touched: both are wrecks from now on, and part of the failure
    /// that led to it.
    pub(crate) fn impact(&mut self, imp: Impact) {
        let incident = [imp.struck, imp.striker]
            .iter()
            .find_map(|id| self.wrecks.get(id).map(|w| w.incident))
            .or_else(|| {
                self.pending.iter().find(|p| p.0 == imp.struck || p.0 == imp.striker).and_then(|p| p.2)
            })
            .or_else(|| self.incidents.back().map(|i| i.id));
        let st = &mut self.stats;
        if imp.crossing {
            st.crossing_impacts += 1;
            st.crossing_impact_max = st.crossing_impact_max.max(imp.speed);
        } else {
            st.lane_impacts += 1;
            st.lane_impact_max = st.lane_impact_max.max(imp.speed);
        }
        if let Some(inc) = self.incidents.iter_mut().find(|i| Some(i.id) == incident) {
            inc.impacts.push(imp);
        }
        self.pending.push((imp.striker, Fault::Dead, incident));
        self.pending.push((imp.struck, Fault::Dead, incident));
    }

    /// A vehicle has failed, or was struck as part of failure `incident`: it is no
    /// longer the manager's to schedule. On a lane the manager runs, it gets the
    /// trajectory its failure leaves it (at rest, or braking to rest) and holds the
    /// zones that takes, those it rests in without end. `ahead` is the vehicle ahead of
    /// it in its lane, with its length: braking, it gets no farther than that vehicle.
    pub(crate) fn disable(
        &mut self,
        net: &Network,
        v: &Vehicle,
        (fault, incident): (Fault, Option<u64>),
        ahead: Option<(u64, f64)>,
        (t, k, dt): (f64, u64, f64),
    ) {
        let lane = v.route[0];
        let info = self.lanes.get(&lane).cloned();
        let x = route_x(net, v);
        let id = match incident.or_else(|| self.wrecks.get(&v.id).map(|w| w.incident)) {
            Some(id) => id,
            None => {
                let place = match &info {
                    Some(l) if x >= l.l_in && x - v.len() < l.l_box => Place::Box,
                    Some(l) if x >= l.l_in => Place::Exit,
                    None if v.ri > 0 => Place::Exit,
                    _ => Place::Approach,
                };
                let id = self.next_incident;
                self.next_incident += 1;
                self.incidents.push_back(Incident {
                    id,
                    t,
                    vehicle: v.id,
                    kind: v.kind,
                    fault,
                    movement: v.movement,
                    lane,
                    x,
                    place,
                    policy: self.policy,
                    stops: 0,
                    withdrawn: 0,
                    impacts: Vec::new(),
                    margin: f64::INFINITY,
                    settled: t,
                    cleared: None,
                });
                while self.incidents.len() > INCIDENT_LOG {
                    self.incidents.pop_front();
                }
                self.stats.stalls += 1;
                id
            }
        };
        // Everything a failure wrecks is cleared away together.
        let clear_at = t + self.clear_time;
        for w in self.wrecks.values_mut().filter(|w| w.incident == id) {
            w.clear_at = clear_at;
        }
        let since = self.wrecks.get(&v.id).map_or(t, |w| w.since);
        self.wrecks.insert(v.id, Wreck { since, clear_at, fault, incident: id });
        self.halted.remove(&v.id);
        let Some(info) = info else { return };
        let brake = brake_decel(v.kind);
        let mut tr = Traj::start(x, if fault == Fault::Dead { 0.0 } else { v.v });
        let lead = ahead.and_then(|(id, len)| Some((self.plans.get(&id)?, len)));
        tr.stop_behind(brake, dt, k, lead);
        let old = self.plans.get(&v.id);
        let known =
            |cid: usize| old.and_then(|p| p.zones.iter().find(|z| z.cid == cid)).map(|z| (z.t_in, z.v_in));
        let zones = zone_times(&info.zones, &tr, (t, dt), (v.len(), brake), self.policy, known);
        let (t_line, v_line) = old.map_or((f64::NAN, 0.0), |p| (p.t_line, p.v_line));
        if x < info.l_in {
            self.unschedule(t_line, info.movement);
        }
        self.install(
            v.id,
            Plan {
                k0: k,
                t0: t,
                x: tr.x,
                v: tr.v,
                a: tr.a,
                why: tr.why,
                t_line,
                v_line,
                zones,
                movement: info.movement,
                lane,
                kind: PlanKind::Fault,
                rests: true,
                passes: false,
                contact: tr.contact,
            },
        );
    }

    /// Take a crossing that will not happen as planned off the timeline of crossings.
    fn unschedule(&mut self, t_line: f64, movement: usize) {
        if let Some(i) = self.crossings.iter().position(|c| c.0 == t_line && c.1 == movement) {
            self.crossings.remove(i);
        }
    }

    /// The vehicle ahead of `vid` in its lane.
    fn ahead(&self, lane: LinkId, vid: u64) -> Option<u64> {
        let (q, _) = self.chains.get(&lane)?;
        let pos = q.iter().position(|&x| x == vid)?;
        pos.checked_sub(1).and_then(|p| q.get(p).copied())
    }

    /// Is the vehicle driving a trajectory this step (as opposed to finding its own way)?
    pub(crate) fn drives(&self, vid: u64, k: u64) -> bool {
        self.plans.get(&vid).is_some_and(|p| p.rests || k < p.end())
    }

    fn note_margin(&mut self, incident: Option<u64>, margin: f64) {
        self.stats.stop_margin_min = self.stats.stop_margin_min.min(margin);
        if let Some(inc) = self.incidents.iter_mut().find(|i| Some(i.id) == incident) {
            inc.margin = inc.margin.min(margin);
        }
    }

    /// The reaction to a failure. Every vehicle whose trajectory the vehicles in
    /// `changed` now endanger gets a new one: a vehicle that can still stop short of the
    /// box has its crossing withdrawn, and one that cannot brakes to an emergency stop,
    /// or follows the vehicle ahead of it until it rests behind it. A new trajectory can
    /// endanger others in turn, so this goes on until every trajectory fits the others
    /// again. Vehicles whose trajectories still fit keep them and cross as planned.
    ///
    /// Every new trajectory starts with what is left of the old one for the reaction
    /// time: no vehicle responds sooner than that.
    pub(crate) fn respond(
        &mut self,
        net: &Network,
        veh: &[Vehicle],
        changed: &[u64],
        (t, k, dt): (f64, u64, f64),
    ) {
        let idx: HashMap<u64, usize> = veh.iter().enumerate().map(|(i, v)| (v.id, i)).collect();
        let incident = changed.iter().find_map(|id| self.wrecks.get(id).map(|w| w.incident));
        let cx = Alert { net, veh, idx: &idx, t, k, dt, steps: (REACTION / dt).round() as u64, incident };
        let mut queue: VecDeque<u64> = VecDeque::new();
        for &d in changed {
            queue.push_back(d);
            self.follow_on(&cx, d, &mut queue);
        }
        // Each vehicle changes trajectory at most twice (to follow, then to stop), so
        // this ends; the bound is only a safeguard.
        let mut left = 4 * veh.len() + 64;
        while let Some(d) = queue.pop_front() {
            if left == 0 {
                break;
            }
            left -= 1;
            for (vid, x0) in self.endangered(&cx, d) {
                if self.react(&cx, vid, Law::Stop, Some(x0)) {
                    queue.push_back(vid);
                    self.follow_on(&cx, vid, &mut queue);
                }
            }
        }
        self.alarm = true;
    }

    /// The vehicles behind `d` in its lane follow it, one after another, as far back as
    /// trajectories change.
    fn follow_on(&mut self, cx: &Alert, d: u64, queue: &mut VecDeque<u64>) {
        let Some(lane) = cx.idx.get(&d).map(|&i| cx.veh[i].route[0]) else { return };
        let Some((q, _)) = self.chains.get(&lane) else { return };
        let Some(pos) = q.iter().position(|&x| x == d) else { return };
        let behind: Vec<u64> = q.iter().skip(pos + 1).copied().collect();
        for f in behind {
            if !self.react(cx, f, Law::Follow, None) {
                break;
            }
            queue.push_back(f);
        }
    }

    /// Vehicles whose trajectories would take them into a zone while `d`, on its new
    /// trajectory, is in it or has barely left: each with the route coordinate of the
    /// first such zone on its path. Only zones a vehicle enters after the reaction time
    /// count; before that it cannot respond.
    fn endangered(&self, cx: &Alert, d: u64) -> Vec<(u64, f64)> {
        let Some(dp) = self.plans.get(&d) else { return Vec::new() };
        let t_react = cx.t + REACTION;
        let mut out: Vec<(u64, f64)> = Vec::new();
        for (&vid, p) in &self.plans {
            if vid == d || matches!(p.kind, PlanKind::Fault | PlanKind::Withdrawn) {
                continue;
            }
            let first = p
                .zones
                .iter()
                .filter(|vz| vz.t_in > t_react - 1e-9)
                .filter(|vz| {
                    dp.zones.iter().any(|dz| {
                        dz.cid == vz.cid
                            && dz.side != vz.side
                            && vz.t_in < dz.t_out + TOLERANCE
                            && dz.t_in < vz.t_out + TOLERANCE
                    })
                })
                .map(|vz| vz.x0)
                .reduce(f64::min);
            if let Some(x0) = first {
                out.push((vid, x0));
            }
        }
        out.sort_by_key(|x| x.0);
        out
    }

    /// Give `vid` its reaction to a failure. With `hazard`, it reacts to a zone on its
    /// path (at that route coordinate) that it must not enter; otherwise to the vehicle
    /// ahead of it. True if its trajectory changed.
    fn react(&mut self, cx: &Alert, vid: u64, law: Law, hazard: Option<f64>) -> bool {
        let Some(&i) = cx.idx.get(&vid) else { return false };
        let v = &cx.veh[i];
        let Some(old) = self.plans.get(&vid) else { return false };
        if old.kind == PlanKind::Withdrawn {
            return false;
        }
        if v.disabled || matches!(old.kind, PlanKind::Fault | PlanKind::EmergencyStop) {
            // It has failed, or is already braking as hard as it can: its trajectory
            // stays. Note where that leaves it, and where it now meets the vehicle ahead.
            if let Some(x0) = hazard.filter(|_| old.kind == PlanKind::EmergencyStop) {
                let margin = x0 - old.last_x();
                self.note_margin(cx.incident, margin);
            }
            return law == Law::Follow && self.meets(cx, vid);
        }
        if old.rests && cx.k >= old.end() {
            // At rest where an earlier reaction left it: there it stays until it is
            // granted a way out.
            return false;
        }
        let lane = old.lane;
        let Some(info) = self.lanes.get(&lane).cloned() else { return false };
        let av = Av::of(v);
        // The reaction time: it keeps to the trajectory it has.
        let mut tr = Traj::start(route_x(cx.net, v), v.v);
        for n in cx.k..cx.k + cx.steps {
            let j = (n - old.k0) as usize;
            if j < old.a.len() {
                tr.push(old.a[j], old.why[j], cx.dt);
            } else if old.rests {
                tr.push(0.0, Reason::Stranded, cx.dt);
            } else {
                // It leaves the map first.
                return false;
            }
        }
        let (xr, vr) = tr.state();
        let (movement, was_crossing) = (old.movement, old.crossing());
        let (mut t_line, mut v_line) = (old.t_line, old.v_line);
        let (old_last, old_end, old_kind) = (old.last_x(), old.end(), old.kind);
        let known: Vec<(usize, f64, f64)> = old.zones.iter().map(|z| (z.cid, z.t_in, z.v_in)).collect();
        let before_line = tr.x[0] < info.l_in;

        // With room to stop short of the box, its crossing is withdrawn; so is that of a
        // vehicle standing before the line, however close to it.
        if xr + vr * vr / (2.0 * av.brake) + HALT_ROOM <= info.l_in
            || (was_crossing && vr <= AT_REST && xr < info.l_in)
        {
            self.unschedule(t_line, movement);
            self.install(
                vid,
                Plan {
                    k0: cx.k,
                    t0: cx.t,
                    x: tr.x,
                    v: tr.v,
                    a: tr.a,
                    why: tr.why,
                    t_line: f64::NAN,
                    v_line: 0.0,
                    zones: Vec::new(),
                    movement,
                    lane,
                    kind: PlanKind::Withdrawn,
                    rests: false,
                    passes: false,
                    contact: None,
                },
            );
            self.halted.insert(vid);
            self.stats.withdrawn += 1;
            if let Some(inc) = self.incidents.iter_mut().find(|i| Some(i.id) == cx.incident) {
                inc.withdrawn += 1;
            }
            return true;
        }

        // Otherwise it is in the box, or too close to stop short of it.
        let lead = self.ahead(lane, vid).and_then(|l| {
            let p = self.plans.get(&l)?;
            let len = cx.veh[*cx.idx.get(&l)?].len();
            (p.kind != PlanKind::Withdrawn).then_some((p, len))
        });
        // It may meet the vehicle ahead before the reaction time is over.
        tr.meet(cx.k, lead);
        let kind = match (law, lead) {
            (Law::Follow, Some((lp, lead_len))) => {
                let mut rests = false;
                let steps = if tr.contact.is_some() { 0 } else { MAX_CONTINGENCY_STEPS };
                for n in (cx.k + cx.steps..).take(steps) {
                    let (x, vel) = tr.state();
                    if x >= info.l_end {
                        break;
                    }
                    if vel <= AT_REST {
                        // At rest it stays at rest: it is not set in motion to close up.
                        tr.push(-vel / cx.dt, Reason::Stranded, cx.dt);
                        rests = true;
                        break;
                    }
                    let mut acc = if x < info.l_box {
                        ((info.v_box - vel) / cx.dt).clamp(-av.b, av.a)
                    } else {
                        av.a * (1.0 - (vel / av.v0.max(0.1)).powi(4))
                    };
                    let mut why = Reason::Free;
                    // Never faster than the trajectory it had: the zones ahead are its
                    // own only from the times that trajectory reaches them.
                    let j = (n + 1 - old.k0) as usize;
                    let v_old = old.v.get(j).copied().or(old.rests.then_some(0.0));
                    if let Some(cap) = v_old.map(|v_old| (v_old - vel) / cx.dt).filter(|cap| *cap < acc) {
                        acc = cap;
                        why = old.why.get(j - 1).copied().unwrap_or(Reason::Stranded);
                    }
                    if let Some((xl, vl, al)) = lp.state(n) {
                        let gap = xl - lead_len - x;
                        if gap <= 0.0 {
                            // It cannot stop in time. The manager counts on it standing
                            // against the vehicle ahead from here on; it goes on
                            // braking, should that vehicle no longer be there.
                            tr.contact = Some((tr.a.len(), xl - lead_len));
                            break;
                        }
                        if vl <= 0.0 && vel < 0.05 && gap - av.s0 < 0.3 {
                            // Close behind a vehicle at rest: stop here.
                            tr.push(-vel / cx.dt, Reason::Stranded, cx.dt);
                            rests = true;
                            break;
                        }
                        let af = follow_acc(&av, vel, gap, vl, al);
                        if af < acc {
                            acc = af;
                            why = if af < -PLAN_DECEL { Reason::Emergency } else { Reason::Following };
                        }
                    }
                    tr.push(acc.clamp(-av.brake, av.a), why, cx.dt);
                }
                if !rests && tr.state().0 < info.l_end {
                    tr.stop(av.brake, cx.dt);
                }
                PlanKind::StopBehind
            }
            (_, lead) => {
                tr.stop_behind(av.brake, cx.dt, cx.k, lead);
                PlanKind::EmergencyStop
            }
        };
        let rests = tr.state().0 < info.l_end;
        let (last, end) = (tr.state().0, cx.k + tr.a.len() as u64);
        if kind == PlanKind::StopBehind
            && old_kind == kind
            && (old_last - last).abs() < 1e-9
            && old_end == end
        {
            // The vehicle ahead changed nothing for it.
            return false;
        }
        let zones = zone_times(&info.zones, &tr, (cx.t, cx.dt), (av.len, av.brake), self.policy, |cid| {
            known.iter().find(|z| z.0 == cid).map(|z| (z.1, z.2))
        });
        if before_line {
            // It crosses the line later than it was going to, or not at all.
            self.unschedule(t_line, movement);
            (t_line, v_line) = (f64::NAN, 0.0);
            let mut time = cx.t;
            for j in 0..tr.a.len() {
                if tr.x[j] < info.l_in && tr.x[j + 1] >= info.l_in {
                    let tau = cross_time(tr.x[j], tr.v[j], tr.a[j], info.l_in, cx.dt);
                    (t_line, v_line) = (time + tau, tr.v[j] + tr.a[j] * tau);
                    self.crossings.push_back((t_line, movement));
                    break;
                }
                time += cx.dt;
            }
        }
        if let Some(x0) = hazard.filter(|_| kind == PlanKind::EmergencyStop) {
            self.note_margin(cx.incident, x0 - last);
        }
        let settled = cx.t + tr.a.len() as f64 * cx.dt;
        if was_crossing {
            self.stats.emergency_stops += 1;
        }
        if let Some(inc) = self.incidents.iter_mut().find(|i| Some(i.id) == cx.incident) {
            if was_crossing {
                inc.stops += 1;
            }
            if kind == PlanKind::EmergencyStop {
                inc.settled = inc.settled.max(settled);
                self.stats.settle_max = self.stats.settle_max.max(inc.settled - inc.t);
            }
        }
        self.install(
            vid,
            Plan {
                k0: cx.k,
                t0: cx.t,
                x: tr.x,
                v: tr.v,
                a: tr.a,
                why: tr.why,
                t_line,
                v_line,
                zones,
                movement,
                lane,
                kind,
                rests,
                passes: false,
                contact: tr.contact,
            },
        );
        true
    }

    /// A vehicle that has failed, or brakes as hard as it can, keeps its trajectory
    /// whatever the vehicle ahead of it does. Where it is expected to meet that vehicle
    /// may have changed, though, and with it the zones it holds (`Plan::contact`). True
    /// if it has.
    fn meets(&mut self, cx: &Alert, vid: u64) -> bool {
        let Some(old) = self.plans.get(&vid) else { return false };
        let Some(v) = cx.idx.get(&vid).map(|&i| &cx.veh[i]) else { return false };
        let Some(info) = self.lanes.get(&old.lane) else { return false };
        let lead = self.ahead(old.lane, vid).and_then(|l| {
            let p = self.plans.get(&l)?;
            let len = cx.veh[*cx.idx.get(&l)?].len();
            (p.kind != PlanKind::Withdrawn).then_some((p, len))
        });
        let from = (cx.k - old.k0) as usize;
        let contact = lead.and_then(|(p, len)| met(p, len, old.k0, &old.x, from));
        if contact == old.contact {
            return false;
        }
        let tr = Traj { x: old.x.clone(), v: old.v.clone(), a: old.a.clone(), why: old.why.clone(), contact };
        let known = |cid: usize| old.zones.iter().find(|z| z.cid == cid).map(|z| (z.t_in, z.v_in));
        let brake = brake_decel(v.kind);
        let zones = zone_times(&info.zones, &tr, (old.t0, cx.dt), (v.len(), brake), self.policy, known);
        let plan = Plan { zones, contact, ..old.clone() };
        self.install(vid, plan);
        true
    }

    /// Grant a way out to one vehicle that came to rest past the stop line after a
    /// failure, if its way is clear: it waits where it is for a gap in the traffic that
    /// still crosses its path, then drives on.
    ///
    /// A zone on its way that another vehicle at rest holds does not stop it if their
    /// bodies stay `PASS_ROOM` apart all the way: zones are drawn for traffic at speed,
    /// and a vehicle leaving the box from rest may pass closer, at `PASS_SPEED` at most.
    ///
    /// Returns whether a way out was granted, and otherwise a vehicle that waits for
    /// another one at rest that is not disabled.
    fn recover(
        &mut self,
        net: &Network,
        veh: &[Vehicle],
        idx: &HashMap<u64, usize>,
        (t, k, dt): (f64, u64, f64),
    ) -> (bool, Option<u64>) {
        let mut lanes: Vec<LinkId> = self.chains.keys().copied().collect();
        lanes.sort_unstable();
        let mut standoff = None;
        for lane in lanes {
            let (q, n) = &self.chains[&lane];
            let Some(&vid) = q.get(*n) else { continue };
            let Some(&i) = idx.get(&vid) else { continue };
            let v = &veh[i];
            let Some(p) = self.plans.get(&vid) else { continue };
            if v.disabled || p.kind == PlanKind::Fault || !p.rests || k < p.end() {
                continue;
            }
            let info = &self.lanes[&lane];
            let x = p.last_x();
            if x < info.l_in {
                // Before the line it is a candidate like any other.
                continue;
            }
            let av = Av::of(v);
            // Vehicles at rest in zones still on its way: each must leave it room to pass.
            let mut passes = false;
            let mut blocker = None;
            for z in info.zones.iter().filter(|z| x < z.x1 + av.len) {
                for r in self.res[z.cid][1 - z.side].iter().filter(|r| r.t_out.is_infinite()) {
                    passes = true;
                    // Only a vehicle that has come to rest can be passed: one still
                    // braking is not yet where it will stand.
                    let other = idx
                        .get(&r.vid)
                        .map(|&j| &veh[j])
                        .filter(|_| self.plans.get(&r.vid).is_some_and(|hp| k >= hp.end()));
                    if blocker.is_none() && other.is_none_or(|o| !passes_clear(net, v, (x, z.x1 + av.len), o))
                    {
                        blocker = Some(r.vid);
                    }
                }
            }
            if let Some(h) = blocker {
                if !self.wrecks.contains_key(&h) && standoff.is_none() {
                    standoff = Some(vid);
                }
                continue;
            }
            let leader = n
                .checked_sub(1)
                .and_then(|l| q.get(l))
                .and_then(|l| Some((self.plans.get(l)?, veh[*idx.get(l)?].len())));
            let req = Req {
                t,
                k,
                dt,
                x,
                v: 0.0,
                av,
                movement: info.movement,
                lane,
                l_in: info.l_in,
                l_box: info.l_box,
                l_end: info.l_end,
                v_box: if passes { info.v_box.min(PASS_SPEED) } else { info.v_box },
                zones: &info.zones,
                leader,
                policy: self.policy,
                opens_batch: false,
            };
            let mut depart = t;
            let mut found = None;
            for _ in 0..12 {
                let Some(plan) = simulate(&req, depart) else { break };
                // The zones it stands in are its own already, but it may leave them only
                // when no vehicle on its way out of the box is passing it.
                let (ahead, mine): (Vec<&PlanZone>, Vec<&PlanZone>) =
                    plan.zones.iter().partition(|z| z.t_in > t);
                if !mine.iter().all(|z| self.clear_among(z, false)) {
                    break;
                }
                let rel: Vec<_> = ahead
                    .iter()
                    .map(|z| (z.cid, z.side, z.t_in - depart, z.t_out - depart, z.need))
                    .collect();
                let t_ok = self.earliest_among(&rel, depart, false, !passes);
                if !t_ok.is_finite() {
                    break;
                }
                if t_ok > depart + 1e-9 {
                    depart = t_ok + PAD;
                } else if ahead.iter().all(|z| self.clear_among(z, !passes)) {
                    found = Some(plan);
                    break;
                } else {
                    depart += dt;
                }
            }
            if let Some(mut plan) = found {
                plan.passes = passes;
                self.install(vid, plan);
                self.stats.recovered += 1;
                self.stats.plans += 1;
                return (true, None);
            }
        }
        (false, standoff)
    }

    /// After a failure: grant ways out to the vehicles that stopped past the line, and
    /// if they only block one another once nothing disabled is left, tow one of them.
    pub(super) fn clear_box(
        &mut self,
        net: &Network,
        veh: &[Vehicle],
        idx: &HashMap<u64, usize>,
        (t, k, dt): (f64, u64, f64),
    ) {
        let mut granted = false;
        let mut standoff = None;
        for _ in 0..8 {
            let (ok, waiting) = self.recover(net, veh, idx, (t, k, dt));
            standoff = waiting;
            if !ok {
                break;
            }
            granted = true;
        }
        let settled = self.wrecks.is_empty() && !self.plans.values().any(|p| !p.crossing() && k < p.end());
        match standoff {
            Some(vid) if settled && !granted => {
                let since = *self.stuck_since.get_or_insert(t);
                if t - since >= STUCK_LIMIT {
                    self.remove.push(vid);
                    self.stats.towed += 1;
                    self.stuck_since = None;
                }
            }
            _ => self.stuck_since = None,
        }
    }
}

impl Sim {
    /// Failures at the cooperative crossing, dealt with before the manager grants
    /// anything: wrecks whose time is up leave the map, vehicles that failed or were
    /// struck since the last step stop where their failure leaves them, and the manager
    /// reacts for every vehicle they endanger.
    pub(super) fn coop_failures(&mut self, c: &mut CoopManager) {
        let now = (self.t, self.steps, self.dt);
        c.arm_random(self.t);
        c.spring(&self.vehicles, self.t, self.dt);
        let gone = c.take_removals(self.t);
        for id in &gone {
            if let Some(i) = self.vehicles.iter().position(|v| v.id == *id) {
                let v = self.vehicles.swap_remove(i);
                c.vehicle_removed(&self.net, &v, self.t);
            }
        }
        if !gone.is_empty() {
            self.rebuild_occ();
        }
        let pending = c.take_pending();
        if pending.is_empty() {
            return;
        }
        let mut changed: Vec<u64> = Vec::new();
        for (id, fault, incident) in pending {
            let Some(i) = self.vehicles.iter().position(|v| v.id == id) else { continue };
            let fresh = !c.wrecks.contains_key(&id);
            let v = &mut self.vehicles[i];
            if fault == Fault::Dead {
                v.v = 0.0;
                v.a = 0.0;
            }
            v.disabled = true;
            v.reason = Reason::Disabled;
            // The vehicle ahead of it in its lane, and its length.
            let ahead = c
                .ahead(self.vehicles[i].route[0], id)
                .and_then(|l| self.vehicles.iter().find(|w| w.id == l))
                .map(|w| (w.id, w.len()));
            c.disable(&self.net, &self.vehicles[i], (fault, incident), ahead, now);
            if !changed.contains(&id) {
                changed.push(id);
            }
            if fresh {
                self.blind_follower(c, i);
            }
        }
        c.respond(&self.net, &self.vehicles, &changed, now);
    }

    /// The vehicle right behind one that has just failed, if it finds its own way (it
    /// drives no trajectory), does not respond to it for the reaction time either.
    fn blind_follower(&mut self, c: &CoopManager, i: usize) {
        let (lane, x) = (self.vehicles[i].route[0], route_x(&self.net, &self.vehicles[i]));
        let behind = self
            .vehicles
            .iter()
            .enumerate()
            .filter(|(j, w)| *j != i && w.route[0] == lane && !w.disabled)
            .map(|(j, w)| (j, route_x(&self.net, w)))
            .filter(|(_, wx)| *wx < x)
            .max_by(|a, b| a.1.total_cmp(&b.1));
        if let Some((j, _)) = behind
            && !c.drives(self.vehicles[j].id, self.steps)
        {
            self.vehicles[j].blind_until = self.t + REACTION;
        }
    }

    /// Do two vehicles' bodies overlap (by more than the drawn mirrors)?
    fn touching(&self, a: &Vehicle, b: &Vehicle) -> bool {
        let (ba, bb) = (self.body(a), self.body(b));
        ba.iter().any(|p| bb.iter().any(|q| obb_overlap(*p, *q, 0.25)))
    }

    /// Vehicles that ran into one another during this step, looked for only while a
    /// failure is being dealt with. Each pair stops where it touches: nothing of a
    /// collision is simulated beyond that, and its closing speed is the measure of it.
    /// Both are wrecks from then on, and the manager is told.
    pub(in crate::sim) fn coop_contacts(&mut self, c: &mut CoopManager) {
        let t = self.t + self.dt;
        // One behind another in a lane.
        let mut order: Vec<(LinkId, f64, usize)> =
            self.vehicles.iter().enumerate().map(|(i, v)| (v.route[0], route_x(&self.net, v), i)).collect();
        order.sort_by(|a, b| a.0.cmp(&b.0).then(b.1.total_cmp(&a.1)).then(a.2.cmp(&b.2)));
        for w in order.windows(2) {
            let ((lane, _, li), (lane2, _, fi)) = (w[0], w[1]);
            if lane != lane2 {
                continue;
            }
            let (lead, foll) = (&self.vehicles[li], &self.vehicles[fi]);
            if lead.disabled && foll.disabled && lead.v == 0.0 && foll.v == 0.0 {
                continue;
            }
            // The leader's position now, which this pass may have moved back.
            let rear = route_x(&self.net, lead) - lead.len();
            if route_x(&self.net, foll) - rear <= 1e-6 {
                continue;
            }
            let speed = (foll.v - lead.v).max(0.0);
            let (striker, struck) = (foll.id, lead.id);
            set_route_x(&self.net, &mut self.vehicles[fi], rear);
            let at = self.front(&self.vehicles[fi]);
            for i in [li, fi] {
                let v = &mut self.vehicles[i];
                (v.v, v.a, v.disabled) = (0.0, 0.0, true);
            }
            c.impact(Impact { t, speed, crossing: false, striker, struck, at: (at.x, at.y) });
        }
        // On crossing paths, in or next to the box.
        let near: Vec<usize> = (0..self.vehicles.len())
            .filter(|&i| {
                let v = &self.vehicles[i];
                let l = &self.net.links[v.link()];
                l.kind.in_box() || (l.kind != LinkKind::Inbound && v.s < v.len() + 2.0)
            })
            .collect();
        for (n, &i) in near.iter().enumerate() {
            for &j in &near[n + 1..] {
                let (a, b) = (&self.vehicles[i], &self.vehicles[j]);
                if a.route[0] == b.route[0] || (a.disabled && b.disabled && a.v == 0.0 && b.v == 0.0) {
                    continue;
                }
                if !self.touching(a, b) {
                    continue;
                }
                // Back both up along this step's travel to where they first touch.
                let span = |v: &Vehicle| {
                    let x = route_x(&self.net, v);
                    (x - (v.s - v.prev_s), x)
                };
                let ((a0, a1), (b0, b1)) = (span(a), span(b));
                let (mut ta, mut tb) = (a.clone(), b.clone());
                let (mut lo, mut hi) = (0.0, 1.0);
                for _ in 0..8 {
                    let mid = 0.5 * (lo + hi);
                    set_route_x(&self.net, &mut ta, a0 + mid * (a1 - a0));
                    set_route_x(&self.net, &mut tb, b0 + mid * (b1 - b0));
                    if self.touching(&ta, &tb) {
                        hi = mid;
                    } else {
                        lo = mid;
                    }
                }
                let heading = |v: &Vehicle| self.net.links[v.link()].path.tangent(v.s) * v.v;
                let speed = (heading(a) - heading(b)).len();
                let (striker, struck) = if a.v >= b.v { (a.id, b.id) } else { (b.id, a.id) };
                set_route_x(&self.net, &mut self.vehicles[i], a0 + lo * (a1 - a0));
                set_route_x(&self.net, &mut self.vehicles[j], b0 + lo * (b1 - b0));
                let at = self.center(&self.vehicles[i]).lerp(self.center(&self.vehicles[j]), 0.5);
                for k in [i, j] {
                    let v = &mut self.vehicles[k];
                    (v.v, v.a, v.disabled) = (0.0, 0.0, true);
                }
                c.impact(Impact { t, speed, crossing: true, striker, struck, at: (at.x, at.y) });
            }
        }
    }

    /// Make a vehicle of the cooperative crossing fail where it is, when the manager
    /// next runs. False on other layouts, and if there is no such vehicle or it is
    /// already disabled.
    pub fn stall_vehicle(&mut self, id: u64, fault: Fault) -> bool {
        let Some(c) = self.coop.as_mut() else { return false };
        if !self.vehicles.iter().any(|v| v.id == id && !v.disabled) {
            return false;
        }
        c.stall(id, fault);
        true
    }

    /// Make the next vehicle on inbound lane `lane` fail when its front is `x` meters
    /// along its route from the start of that lane. With the same arrivals, another
    /// run can so be given a failure at the same place. False on other layouts, and if
    /// the lane already has such a place.
    pub fn stall_at(&mut self, lane: LinkId, x: f64, fault: Fault) -> bool {
        self.coop.as_mut().is_some_and(|c| c.trap(lane, x, fault))
    }

    /// Where a vehicle is: its inbound lane and how far along its route its front is.
    pub fn place_of(&self, id: u64) -> Option<(LinkId, f64)> {
        self.vehicles.iter().find(|v| v.id == id).map(|v| (v.route[0], route_x(&self.net, v)))
    }

    /// Make a vehicle fail in the box: the one nearest the middle of its path through
    /// it, at once if it is in the box, and otherwise when it gets to the middle. A
    /// vehicle short of the box whose lane already has a place set for a failure is
    /// passed over. Returns the lane and place of the failure.
    pub fn stall_in_box(&mut self, fault: Fault) -> Option<(LinkId, f64)> {
        let c = self.coop.as_ref()?;
        let (v, lane, x, mid) = self
            .vehicles
            .iter()
            .filter(|v| !v.disabled)
            .filter_map(|v| {
                let (l_in, l_box) = c.box_span(v.route[0])?;
                let x = route_x(&self.net, v);
                let set = x < l_in && c.traps.iter().any(|t| t.lane == v.route[0]);
                (x < l_box && !set).then_some((v, v.route[0], x, 0.5 * (l_in + l_box)))
            })
            .min_by(|a, b| (a.2 - a.3).abs().total_cmp(&(b.2 - b.3).abs()).then(a.0.id.cmp(&b.0.id)))?;
        let (l_in, _) = c.box_span(lane)?;
        if x >= l_in {
            let id = v.id;
            self.stall_vehicle(id, fault).then_some((lane, x))
        } else {
            self.stall_at(lane, mid, fault).then_some((lane, mid))
        }
    }

    /// Make a vehicle fail at the worst moment for crossing traffic: the next one about
    /// to leave a zone with a crossing vehicle due right behind it at the smallest gap
    /// its policy allows. False on other layouts.
    pub fn stall_at_worst(&mut self, fault: Fault) -> bool {
        self.coop.as_mut().map(|c| c.armed = Some(fault)).is_some()
    }

    /// Clear every disabled vehicle away when the manager next runs.
    pub fn clear_wrecks(&mut self) {
        if let Some(c) = self.coop.as_mut() {
            c.clear_wrecks();
        }
    }
}

/// Does `v`, driving from route coordinate `from` to `to`, stay `PASS_ROOM` clear of
/// the body of `other` where it stands?
fn passes_clear(net: &Network, v: &Vehicle, (from, to): (f64, f64), other: &Vehicle) -> bool {
    let there = body_on(net, other, 0.0);
    let mut probe = v.clone();
    let mut x = from;
    while x <= to {
        set_route_x(net, &mut probe, x);
        let here = body_on(net, &probe, 0.0);
        if here.iter().any(|p| there.iter().any(|q| obb_overlap(*p, *q, -0.5 * PASS_ROOM))) {
            return false;
        }
        x += 0.5;
    }
    true
}

/// Put a vehicle's front `x` meters from the start of its route, keeping where it was
/// a step ago.
fn set_route_x(net: &Network, v: &mut Vehicle, x: f64) {
    let before = route_x(net, v) - v.s + v.prev_s;
    let (mut ri, mut s) = (0, x);
    while ri + 1 < v.route.len() && s >= net.links[v.route[ri]].length() {
        s -= net.links[v.route[ri]].length();
        ri += 1;
    }
    (v.ri, v.s) = (ri, s);
    v.prev_s = before - (x - s);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::CtrlKind;
    use crate::sim::ControlMode;
    use crate::sim::network::LayoutKind;

    const DT: f64 = 0.1;

    fn zone(cid: usize, x0: f64, x1: f64) -> ZoneRef {
        ZoneRef { cid, side: 0, x0, x1 }
    }

    #[test]
    fn an_emergency_stop_ends_at_rest_after_its_braking_distance() {
        let (v, b) = (40.0 / 3.6, 6.0);
        let mut tr = Traj::start(100.0, v);
        tr.stop(b, DT);
        let (x, v_end) = tr.state();
        assert_eq!(v_end, 0.0);
        assert!((x - 100.0 - v * v / (2.0 * b)).abs() < 1e-9, "stopped after {:.3} m", x - 100.0);
        assert!((tr.a.len() as f64 * DT - v / b).abs() < DT, "took {} steps", tr.a.len());
        assert!(tr.why.iter().all(|&w| w == Reason::Emergency));
        // A vehicle at rest stays where it is.
        let mut still = Traj::start(5.0, 0.0);
        still.stop(b, DT);
        assert_eq!((still.a.len(), still.state()), (0, (5.0, 0.0)));
    }

    #[test]
    fn a_trajectory_holds_the_zones_it_rests_in_without_end() {
        // A 4.5 m car at 10 m/s braking at 5 m/s² from x = 0 rests at x = 10: it clears
        // the first zone, rests in the second and never reaches the third.
        let zones = [zone(1, 1.0, 3.0), zone(2, 8.0, 11.0), zone(3, 20.0, 23.0)];
        let mut tr = Traj::start(0.0, 10.0);
        tr.stop(5.0, DT);
        assert!((tr.state().0 - 10.0).abs() < 1e-9);
        let held = zone_times(&zones, &tr, (50.0, DT), (4.5, 6.0), GapPolicy::FailSafe, |_| None);
        assert_eq!(held.iter().map(|z| z.cid).collect::<Vec<_>>(), [1, 2]);
        // Front at 1 m after 0.1 s; rear past 3 m when the front is at 7.5 m, at 1 s.
        assert!((held[0].t_in - 50.1).abs() < 0.02, "entered at {}", held[0].t_in);
        assert!((held[0].t_out - 51.0).abs() < 0.02, "left at {}", held[0].t_out);
        assert!(held[0].v_in > 9.0 && held[0].need > 1.0);
        assert!(held[1].t_in > held[0].t_in && held[1].t_out.is_infinite());

        // A vehicle that stops dead in a zone has been in it since it entered, and one
        // already past a zone holds nothing of it.
        let dead = Traj::start(9.0, 0.0);
        let held = zone_times(&zones, &dead, (50.0, DT), (4.5, 6.0), GapPolicy::Nominal, |cid| {
            (cid == 2).then_some((49.5, 7.0))
        });
        assert_eq!(held.len(), 1);
        assert_eq!((held[0].cid, held[0].t_in, held[0].v_in), (2, 49.5, 7.0));
        assert!(held[0].t_out.is_infinite());
        let unknown = zone_times(&zones, &dead, (50.0, DT), (4.5, 6.0), GapPolicy::Nominal, |_| None);
        assert_eq!((unknown[0].t_in, unknown[0].v_in), (50.0, 0.0));
    }

    #[test]
    fn a_vehicle_that_cannot_stop_behind_another_holds_the_zones_it_will_stand_in() {
        // A 4.5 m car stands with its front at 49.5 m, so its rear is at 45 m. Another
        // does 10 m/s at 40 m and brakes at 6 m/s²: it needs 8.3 m and has 5 m. It is
        // expected at the first car's rear in the step that takes it there, the seventh;
        // its trajectory brakes on to rest, as it would with nothing in its way.
        let ahead = Plan {
            k0: 0,
            t0: 0.0,
            x: vec![49.5],
            v: vec![0.0],
            a: vec![],
            why: vec![],
            t_line: f64::NAN,
            v_line: 0.0,
            zones: vec![],
            movement: 0,
            lane: 0,
            kind: PlanKind::Fault,
            rests: true,
            passes: false,
            contact: None,
        };
        let mut tr = Traj::start(40.0, 10.0);
        tr.stop_behind(6.0, DT, 0, Some((&ahead, 4.5)));
        assert_eq!(tr.contact, Some((7, 45.0)));
        assert!(tr.x[6] < 45.0 && tr.x[7] > 45.5, "{:.2} m, then {:.2} m", tr.x[6], tr.x[7]);
        let (x, v) = tr.state();
        assert!(v == 0.0 && (x - 40.0 - 100.0 / 12.0).abs() < 1e-9, "at rest at {x:.3} m");
        // With room to stop, nothing is expected.
        let mut far = Traj::start(30.0, 10.0);
        far.stop_behind(6.0, DT, 0, Some((&ahead, 4.5)));
        assert_eq!((far.contact, far.state().1), (None, 0.0));

        // The zones, for a car 4.5 m long. It leaves the first before it gets to the car
        // ahead: its rear is clear of it with its front at 44.75 m. The second it would
        // leave at 45.25 m, in the very step that takes it past the rear of the car
        // ahead, and the third at 47.5 m; the fourth it would only reach braking on. It
        // will stand in the second and third, so those three are held without end. With
        // nothing in its way it leaves the second and third.
        let zones = [zone(1, 36.0, 40.25), zone(2, 38.0, 40.75), zone(3, 41.0, 43.0), zone(4, 47.0, 49.0)];
        let times = |tr: &Traj| {
            let held = zone_times(&zones, tr, (0.0, DT), (4.5, 6.0), GapPolicy::FailSafe, |_| None);
            held.iter().map(|z| (z.cid, z.t_out.is_finite())).collect::<Vec<_>>()
        };
        assert_eq!(times(&tr), [(1, true), (2, false), (3, false), (4, false)]);
        let mut free = Traj::start(40.0, 10.0);
        free.stop(6.0, DT);
        assert_eq!(free.contact, None);
        assert_eq!(times(&free), [(1, true), (2, true), (3, true), (4, false)]);

        // The vehicles behind it are told where it will stand, from that step on.
        let plan = Plan {
            x: tr.x,
            v: tr.v,
            a: tr.a,
            contact: tr.contact,
            kind: PlanKind::EmergencyStop,
            ..ahead.clone()
        };
        assert_eq!(plan.state(6).map(|s| s.0), Some(plan.x[6]));
        assert_eq!((plan.state(7), plan.state(400)), (Some((45.0, 0.0, 0.0)), Some((45.0, 0.0, 0.0))));
        // Looked at again two steps later, the car ahead is still in its way; up to the
        // sixth step it was not.
        assert_eq!(met(&ahead, 4.5, 0, &plan.x, 9), Some((9, 45.0)));
        assert_eq!(met(&ahead, 4.5, 0, &plan.x[..7], 0), None);
    }

    #[test]
    fn a_vehicle_can_be_put_anywhere_along_its_route() {
        let mut sim = Sim::new(LayoutKind::Cooperative, ControlMode::Cooperative, CtrlKind::Fixed, 3);
        sim.run_for(60.0);
        let mut v = sim.vehicles.iter().find(|v| v.ri == 0 && v.route.len() == 3).unwrap().clone();
        let lens: Vec<f64> = v.route.iter().map(|&l| sim.net.links[l].length()).collect();
        let before = route_x(&sim.net, &v) - v.s + v.prev_s;
        for (x, ri) in [(10.0, 0), (lens[0] + 5.0, 1), (lens[0] + lens[1] + 30.0, 2), (lens[0] - 0.5, 0)] {
            set_route_x(&sim.net, &mut v, x);
            assert_eq!(v.ri, ri, "{x} m along");
            assert!((route_x(&sim.net, &v) - x).abs() < 1e-9);
            // Where it was a step ago is unchanged, in the new link's terms.
            assert!((route_x(&sim.net, &v) - v.s + v.prev_s - before).abs() < 1e-9);
        }
        // Passing a vehicle at rest: clear of one on another approach, not of one in
        // the same place.
        let other = sim.vehicles.iter().find(|w| w.leg != v.leg && w.ri == 0).unwrap();
        assert!(passes_clear(&sim.net, &v, (10.0, 60.0), other));
        let twin = v.clone();
        assert!(!passes_clear(&sim.net, &v, (lens[0] - 10.0, lens[0] + 10.0), &twin));
    }

    #[test]
    fn a_withdrawn_crossing_is_braked_for_while_the_vehicle_ahead_goes_unseen() {
        // A vehicle stops dead in the box. Vehicles that can still stop short of the box
        // lose their crossings: each keeps to its trajectory for the reaction time, then
        // brakes. Take the one with the least room to spare, still at speed and not yet
        // braking, and make it blind to the vehicle ahead for another reaction time from
        // the step its trajectory runs out, as `blind_follower` does when that vehicle
        // is struck in that step: it must brake for its line all the same. Half a second
        // more as it was would take it into the box.
        let mut sim = Sim::new(LayoutKind::Cooperative, ControlMode::Cooperative, CtrlKind::Fixed, 0);
        sim.run_for(90.0);
        assert!(sim.stall_in_box(Fault::Dead).is_some());
        for _ in 0..600 {
            if sim.coop.as_ref().unwrap().stats.stalls > 0 {
                break;
            }
            sim.step();
        }
        let c = sim.coop.as_ref().unwrap();
        assert_eq!(c.stats.stalls, 1, "no vehicle failed within a minute");
        let room = |p: &Plan, brake: f64| {
            let (x, v) = (p.last_x(), *p.v.last().unwrap());
            (c.lanes[&p.lane].l_in - x - v * v / (2.0 * brake), v)
        };
        let (id, (spare, v)) = sim
            .vehicles
            .iter()
            .filter_map(|w| Some((w.id, c.plans.get(&w.id)?, brake_decel(w.kind))))
            .filter(|(_, p, _)| p.kind == PlanKind::Withdrawn && p.a.last().is_some_and(|a| *a >= 0.0))
            .map(|(id, p, brake)| (id, room(p, brake)))
            .min_by(|a, b| a.1.0.total_cmp(&b.1.0))
            .expect("a crossing withdrawn from a vehicle that is not braking yet");
        assert!(spare >= HALT_ROOM && spare < 0.45 * v, "{spare:.1} m to spare at {:.0} km/h", v * 3.6);
        let lane = c.plans[&id].lane;
        let l_in = c.lanes[&lane].l_in;
        // Its trajectory runs out with the reaction time. From here it does not respond to
        // the vehicle ahead, as if that one had just been struck.
        let end = c.plans[&id].end();
        while sim.steps < end {
            sim.step();
        }
        assert!(sim.coop.as_ref().unwrap().halted(id));
        let i = sim.vehicles.iter().position(|w| w.id == id).unwrap();
        assert!(sim.vehicles[i].a >= 0.0 && sim.vehicles[i].v > 0.9 * v);
        sim.vehicles[i].blind_until = sim.t + REACTION;
        let mut braked = f64::INFINITY;
        for _ in 0..100 {
            sim.step();
            let Some(w) = sim.vehicles.iter().find(|w| w.id == id) else { break };
            let x = route_x(&sim.net, w);
            assert!(x < l_in, "t={:.1}: {:.1} m past its line", sim.t, x - l_in);
            if sim.t < w.blind_until + 1e-9 {
                braked = braked.min(w.a);
            }
        }
        // It braked hard at once, and stands short of the line.
        assert!(braked < -2.0, "it braked at {braked:.1} m/s² while the vehicle ahead went unseen");
        let w = sim.vehicles.iter().find(|w| w.id == id).expect("still on the map");
        assert!(
            w.v < 0.01 && route_x(&sim.net, w) < l_in - 0.5,
            "{:.1} m short",
            l_in - route_x(&sim.net, w)
        );
        assert_eq!(sim.coop.as_ref().unwrap().stats.unauthorized, 0);
    }

    #[test]
    fn the_reaction_time_is_what_every_gap_allows() {
        assert_eq!(REACTION, 0.5);
        for policy in GapPolicy::ALL {
            for v in [0.0, 5.0, 11.1] {
                assert!(policy.gap(v, 6.0) >= REACTION, "{policy:?} at {v} m/s");
            }
        }
    }
}
