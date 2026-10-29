//! Microscopic intersection simulation.
//!
//! * Longitudinal motion: Intelligent Driver Model (Treiber et al.) with curve-speed
//!   limits derived from path curvature.
//! * Right of way: every pair of paths through the intersection is pre-intersected
//!   into conflict points. Vehicles compare priority (signal indication, stop order,
//!   roundabout circulation) and time windows at each conflict point, accept gaps,
//!   and *commit* once they cannot comfortably stop; committed vehicles always keep
//!   the right of way, which keeps the box collision-free without a global planner.
//! * Control: traffic signals (driven by a pluggable controller), all-way stop,
//!   two-way stop, or roundabout yield.

pub mod demand;
pub mod geom;
pub mod network;
pub mod rng;
pub mod sensors;
pub mod signal;
pub mod stats;
pub mod vtypes;

use crate::control::{self, Controller, CtrlKind, Ctx};
use demand::{DemandConfig, DemandGen, Pending};
use geom::V2;
use network::{LayoutKind, LinkId, LinkKind, Network, Turn};
use rng::Rng;
use sensors::{SensorConfig, SensorView};
use serde::{Deserialize, Serialize};
use signal::{PedSig, Phase, Sig, SignalState};
use stats::{Done, Stats};
use std::collections::{HashMap, VecDeque};
use vtypes::{VKind, VSpec, fuel_ml};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ControlMode {
    Signal,
    AllWayStop,
    TwoWayStop,
    Roundabout,
}

impl ControlMode {
    pub fn name(self) -> &'static str {
        match self {
            ControlMode::Signal => "Traffic signal",
            ControlMode::AllWayStop => "All-way stop",
            ControlMode::TwoWayStop => "Two-way stop (minor street)",
            ControlMode::Roundabout => "Yield (roundabout)",
        }
    }
    pub fn options(kind: LayoutKind) -> Vec<ControlMode> {
        match kind {
            LayoutKind::Roundabout => vec![ControlMode::Roundabout],
            LayoutKind::Town4 => vec![ControlMode::Signal, ControlMode::AllWayStop],
            LayoutKind::MajorMinor4 | LayoutKind::Tee => {
                vec![ControlMode::Signal, ControlMode::TwoWayStop, ControlMode::AllWayStop]
            }
            LayoutKind::Arterial4 => vec![ControlMode::Signal, ControlMode::AllWayStop],
        }
    }
}

#[derive(Clone, Debug)]
pub struct Vehicle {
    pub id: u64,
    pub kind: VKind,
    pub movement: usize,
    pub turn: Turn,
    pub leg: usize,
    pub route: Vec<LinkId>,
    pub ri: usize,
    pub s: f64,
    pub prev_s: f64,
    pub v: f64,
    pub a: f64,
    pub v0: f64,
    pub arrival_t: f64,
    pub spawn_t: f64,
    pub ff_time: f64,
    pub delay: f64,
    pub stopped_time: f64,
    pub stops: u32,
    moving: bool,
    pub fuel_ml: f64,
    pub commits: Vec<usize>,
    yellow_dec: Option<(u64, bool)>,
    pub stop_done: bool,
    pub stop_t: f64,
    pub queue_since: Option<f64>,
    pub connected: bool,
    pub cam_ok: bool,
    pub manual: bool,
    pub color: u32,
    /// Why the vehicle is currently braking (for the UI).
    pub reason: Reason,
    /// Stopped directly behind a vehicle that is itself waiting (cannot arrive soon).
    pub blocked: bool,
    /// Vehicle id this one is currently yielding to (diagnostics / UI).
    pub yield_to: Option<u64>,
    /// Perception-reaction time before pulling away from a standstill (s).
    pub reaction: f64,
    /// Time spent wanting to move while still stopped.
    go_t: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reason {
    Free,
    Following,
    Signal,
    StopSign,
    Yielding,
    Pedestrian,
    Curve,
}

impl Reason {
    pub fn text(self) -> &'static str {
        match self {
            Reason::Free => "free flow",
            Reason::Following => "following",
            Reason::Signal => "stopping for signal",
            Reason::StopSign => "stop sign",
            Reason::Yielding => "yielding right-of-way",
            Reason::Pedestrian => "yielding to pedestrian",
            Reason::Curve => "slowing for turn",
        }
    }
}

impl Vehicle {
    pub fn spec(&self) -> &'static VSpec {
        self.kind.spec()
    }
    pub fn len(&self) -> f64 {
        self.spec().length
    }
    pub fn link(&self) -> LinkId {
        self.route[self.ri]
    }
    pub fn persons(&self) -> f64 {
        self.spec().occupancy
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PedState {
    Waiting,
    Crossing,
    Leaving,
}

#[derive(Clone, Debug)]
pub struct Ped {
    pub id: u64,
    pub cw: usize,
    pub u: f64,
    pub dir: f64,
    pub speed: f64,
    pub state: PedState,
    pub arrive_t: f64,
    pub t_state: f64,
    pub lateral: f64,
    pub color: u32,
    pub anim: f64,
    /// Drawn position at the start of the last step, for smooth rendering between steps.
    pub prev_pos: Option<V2>,
}

/// Per-step right-of-way context of a vehicle.
#[derive(Clone, Copy, Debug, Default)]
struct VStat {
    rank: f64,
    key: f64,
    /// Will stop at its stop line, so it is not a threat downstream.
    excluded: bool,
    hold_line: bool,
    must_stop: bool,
    reason_stop: u8,
}

const LOOK: f64 = 70.0;
const HORIZON: f64 = 140.0;

pub struct Sim {
    pub net: Network,
    pub t: f64,
    pub dt: f64,
    pub vehicles: Vec<Vehicle>,
    pub peds: Vec<Ped>,
    pub occ: Vec<Vec<usize>>,
    pub mode: ControlMode,
    pub signal: SignalState,
    pub controller: Box<dyn Controller>,
    pub ctrl_kind: CtrlKind,
    pub demand: DemandConfig,
    pub demand_gen: DemandGen,
    pub queues: Vec<VecDeque<Pending>>,
    pub sensors: SensorConfig,
    pub view: SensorView,
    pub stats: Stats,
    pub rtor: bool,
    /// Blank-out "NO TURN ON RED" sign lit while pedestrians wait at or cross a
    /// crosswalk the right turn passes.
    pub ntor_peds: bool,
    pub preemption: bool,
    pub max_wait_guard: f64,
    pub preempt_active: bool,
    pub collisions: u64,
    /// Run the (costly) body-overlap safety check every step.
    pub check_safety: bool,
    pub collision_log: Vec<String>,
    pub seed: u64,
    pub last_reward: f64,
    misc: Rng,
    next_id: u64,
    ff_cache: HashMap<(usize, LinkId, usize, i64), f64>,
    vstat: Vec<VStat>,
    arrivals_buf: Vec<(usize, Pending)>,
}

impl Sim {
    pub fn new(kind: LayoutKind, mode: ControlMode, ctrl: CtrlKind, seed: u64) -> Sim {
        let net = Network::build(kind);
        let demand = DemandConfig::default_for(&net);
        Self::with_demand(net, mode, ctrl, demand, seed)
    }

    pub fn with_demand(net: Network, mode: ControlMode, ctrl: CtrlKind, demand: DemandConfig, seed: u64) -> Sim {
        let mode = if ControlMode::options(net.kind).contains(&mode) { mode } else { ControlMode::options(net.kind)[0] };
        let signal = SignalState::new(&net, 0.0);
        let sensors = SensorConfig::default();
        let controller = control::make(ctrl, &net, &demand);
        let n_legs = net.legs.len();
        let n_links = net.links.len();
        let view = SensorView::new(&net);
        Sim {
            demand_gen: DemandGen::new(seed, n_legs),
            queues: vec![VecDeque::new(); n_legs],
            occ: vec![Vec::new(); n_links],
            stats: Stats::new(&net),
            net,
            t: 0.0,
            dt: 0.1,
            vehicles: Vec::new(),
            peds: Vec::new(),
            mode,
            signal,
            controller,
            ctrl_kind: ctrl,
            demand,
            sensors,
            view,
            rtor: true,
            ntor_peds: false,
            preemption: true,
            max_wait_guard: 150.0,
            preempt_active: false,
            collisions: 0,
            check_safety: false,
            collision_log: Vec::new(),
            seed,
            last_reward: 0.0,
            misc: Rng::new(seed ^ 0xA5A5_5A5A),
            next_id: 1,
            ff_cache: HashMap::new(),
            vstat: Vec::new(),
            arrivals_buf: Vec::new(),
        }
    }

    pub fn set_controller(&mut self, ctrl: CtrlKind) {
        self.controller = control::make(ctrl, &self.net, &self.demand);
        self.ctrl_kind = ctrl;
    }

    pub fn signalized(&self) -> bool {
        self.mode == ControlMode::Signal && !self.net.stages.is_empty()
    }

    /// Queue a specific vehicle at an approach (user click).
    pub fn spawn_manual(&mut self, leg: usize, kind: VKind, turn: Turn) -> bool {
        let Some(m) = self.net.movement_for(leg, turn) else { return false };
        let p = Pending {
            kind,
            movement: m,
            arrival_t: self.t,
            speed_factor: 1.0,
            cv_u: self.misc.f64(),
            cam_ok: true,
            manual: true,
        };
        self.queues[leg].push_front(p);
        true
    }

    pub fn spawn_ped(&mut self, cw: usize, from_b: bool) {
        let len = self.net.crosswalks[cw].length;
        let speed = (1.35 + 0.18 * self.misc.normal()).clamp(0.9, 1.8);
        let id = self.next_id;
        self.next_id += 1;
        self.peds.push(Ped {
            id,
            cw,
            u: if from_b { len + 0.9 } else { -0.9 },
            dir: if from_b { -1.0 } else { 1.0 },
            speed,
            state: PedState::Waiting,
            arrive_t: self.t,
            t_state: 0.0,
            lateral: self.misc.range(-0.9, 0.9),
            color: (self.misc.next_u64() & 0xFFFF_FFFF) as u32,
            anim: self.misc.f64() * 6.0,
            prev_pos: None,
        });
        if self.mode == ControlMode::Signal {
            self.signal.cw[cw].called = true;
        }
    }

    pub fn step(&mut self) {
        let dt = self.dt;
        // 1. Demand.
        let mut arr = std::mem::take(&mut self.arrivals_buf);
        arr.clear();
        self.demand_gen.generate(&self.demand, &self.net, self.t, dt, &mut arr);
        for (leg, p) in arr.drain(..) {
            self.stats.generated += 1;
            self.queues[leg].push_back(p);
        }
        self.arrivals_buf = arr;
        let n_cw = self.net.crosswalks.len();
        for (cw, b) in self.demand_gen.peds(&self.demand, n_cw, self.t, dt) {
            self.spawn_ped(cw, b);
        }
        // 2. Spawn at the boundary.
        self.spawn_from_queues();
        self.rebuild_occ();
        // 3. Sense.
        self.sense();
        // 4. Control.
        if self.signalized() {
            self.control_signal();
            self.signal.step(&self.net, dt, self.t + dt);
        }
        // 5. Vehicle behaviour.
        self.compute_vstat();
        self.drive(dt);
        // 6. Pedestrians.
        self.step_peds(dt);
        self.t += dt;
        self.stats.sample(self.t, &self.vehicles, &self.net, &self.queues);
    }

    pub fn run_for(&mut self, seconds: f64) {
        let n = (seconds / self.dt).round() as usize;
        for _ in 0..n {
            self.step();
        }
    }

    fn control_signal(&mut self) {
        let can = self.signal.can_switch();
        let ctx = Ctx {
            net: &self.net,
            view: &self.view,
            sig: &self.signal,
            demand: &self.demand,
            sensors: &self.sensors,
            t: self.t,
            dt: self.dt,
            can_switch: can,
            reward_delay: self.last_reward,
        };
        let wish = self.controller.step(&ctx);
        // Emergency-vehicle preemption overrides the controller.
        self.preempt_active = false;
        if self.preemption {
            if let Some(&(m, eta)) =
                self.view.emergency.iter().min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            {
                if eta < 30.0 {
                    self.preempt_active = true;
                    let serves = |st: usize| self.net.stages[st].stat[m].is_some();
                    if !serves(self.signal.stage) && self.signal.phase == Phase::Green {
                        let target = (0..self.net.stages.len())
                            .find(|&st| self.net.stages[st].stat[m] == Some(network::MStat::G))
                            .or_else(|| (0..self.net.stages.len()).find(|&st| serves(st)));
                        if let Some(j) = target {
                            if self.signal.green_elapsed > 2.0 {
                                self.signal.request(&self.net, j, self.t);
                            }
                        }
                    }
                    return;
                }
            }
        }
        if !can {
            return;
        }
        // Starvation guard for adaptive controllers.
        if self.ctrl_kind.adaptive() && self.max_wait_guard > 0.0 {
            let cur = self.signal.stage;
            let worst = (0..self.net.movements.len())
                .filter(|&m| self.net.stages[cur].stat[m].is_none())
                .map(|m| (m, self.view.moves[m].max_wait))
                .filter(|&(_, w)| w > self.max_wait_guard)
                .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
            if let Some((m, _)) = worst {
                let best = (0..self.net.stages.len())
                    .filter(|&st| self.net.stages[st].stat[m].is_some())
                    .max_by(|&a, &b| {
                        let f = |st: usize| -> f64 {
                            (0..self.net.movements.len())
                                .filter(|&k| self.net.stages[st].stat[k].is_some())
                                .map(|k| self.view.moves[k].queue)
                                .sum()
                        };
                        f(a).partial_cmp(&f(b)).unwrap()
                    });
                if let Some(j) = best {
                    self.signal.request(&self.net, j, self.t);
                    return;
                }
            }
        }
        if let Some(j) = wish {
            self.signal.request(&self.net, j, self.t);
        }
    }

    fn rebuild_occ(&mut self) {
        for o in self.occ.iter_mut() {
            o.clear();
        }
        for (i, v) in self.vehicles.iter().enumerate() {
            self.occ[v.route[v.ri]].push(i);
        }
        let veh = &self.vehicles;
        for o in self.occ.iter_mut() {
            o.sort_by(|&a, &b| veh[b].s.partial_cmp(&veh[a].s).unwrap());
        }
    }

    fn spawn_from_queues(&mut self) {
        for leg in 0..self.queues.len() {
            let Some(p) = self.queues[leg].front() else { continue };
            let mv = &self.net.movements[p.movement];
            let spec = p.kind.spec();
            // Choose the admissible lane with the most room at the entry.
            let mut best: Option<(usize, f64, f64)> = None;
            for (ri, (lane, _)) in mv.routes.iter().enumerate() {
                let (room, lead_v) = self.entry_room(*lane);
                if room > spec.min_gap + 1.0 && best.is_none_or(|b| room > b.1) {
                    best = Some((ri, room, lead_v));
                }
            }
            let Some((ri, room, lead_v)) = best else { continue };
            let p = self.queues[leg].pop_front().unwrap();
            let (lane, tail) = &self.net.movements[p.movement].routes[ri];
            let mut route = vec![*lane];
            route.extend_from_slice(tail);
            let speed = self.net.legs[leg].speed;
            let v0 = speed * spec.speed_factor * p.speed_factor;
            let v_safe = if room < 60.0 { lead_v.min(v0).max(((room - spec.min_gap) * 2.0 * spec.decel).sqrt().min(lead_v + 2.0)) } else { v0 };
            let v = v_safe.min(v0).max(0.0);
            let ff = self.free_flow_time(p.movement, &route, p.kind, v0);
            let id = self.next_id;
            self.next_id += 1;
            let wait_virtual = self.t - p.arrival_t;
            self.vehicles.push(Vehicle {
                id,
                kind: p.kind,
                movement: p.movement,
                turn: self.net.movements[p.movement].turn,
                leg,
                route,
                ri: 0,
                s: 0.0,
                prev_s: 0.0,
                v,
                a: 0.0,
                v0,
                arrival_t: p.arrival_t,
                spawn_t: self.t,
                ff_time: ff,
                delay: wait_virtual,
                stopped_time: wait_virtual,
                stops: 0,
                moving: true,
                fuel_ml: 0.0,
                commits: vec![],
                yellow_dec: None,
                stop_done: false,
                stop_t: 0.0,
                queue_since: None,
                connected: p.cv_u < self.sensors.cv_rate,
                cam_ok: p.cam_ok,
                manual: p.manual,
                color: (self.misc.next_u64() & 0xFFFF_FFFF) as u32,
                reason: Reason::Free,
                blocked: false,
                yield_to: None,
                reaction: if p.kind == VKind::Emergency { 0.3 } else { 0.6 + 0.6 * self.misc.f64() },
                go_t: 0.0,
            });
            let idx = self.vehicles.len() - 1;
            self.occ[self.vehicles[idx].route[0]].push(idx);
        }
    }

    /// Free space at the upstream end of a lane and the speed of its last vehicle.
    fn entry_room(&self, lane: LinkId) -> (f64, f64) {
        let mut room = f64::INFINITY;
        let mut lead_v = 30.0;
        for &j in &self.occ[lane] {
            let w = &self.vehicles[j];
            let rear = w.s - w.len();
            if rear < room {
                room = rear;
                lead_v = w.v;
            }
        }
        (room, lead_v)
    }

    fn free_flow_time(&mut self, m: usize, route: &[LinkId], kind: VKind, v0: f64) -> f64 {
        let key = (m, route[0], kind.index(), (v0 * 10.0).round() as i64);
        if let Some(&t) = self.ff_cache.get(&key) {
            return t;
        }
        let spec = kind.spec();
        let mut t = 0.0;
        let (mut k, mut s, mut v) = (0usize, 0.0, v0);
        let dt = 0.1;
        while k < route.len() {
            let link = &self.net.links[route[k]];
            let vlim = if link.kind.in_box() { link.curve_speed.min(v0) } else { v0 };
            let mut acc = spec.accel * (1.0 - (v / vlim.max(0.1)).powi(4));
            let mut d = link.length() - s;
            for &nl in &route[k + 1..] {
                let l2 = &self.net.links[nl];
                if l2.kind.in_box() && l2.curve_speed < v {
                    acc = acc.min(-(v * v - l2.curve_speed * l2.curve_speed) / (2.0 * (d - 1.0).max(0.5)));
                }
                d += l2.length();
                if d > LOOK {
                    break;
                }
            }
            let acc = acc.max(-spec.decel * 1.5);
            let nv = (v + acc * dt).max(0.5);
            s += 0.5 * (v + nv) * dt;
            v = nv;
            t += dt;
            while k < route.len() && s >= self.net.links[route[k]].length() {
                s -= self.net.links[route[k]].length();
                k += 1;
            }
        }
        self.ff_cache.insert(key, t);
        t
    }

    // ---------------------------------------------------------------- right of way

    fn compute_vstat(&mut self) {
        let mut out = std::mem::take(&mut self.vstat);
        out.clear();
        for v in &self.vehicles {
            out.push(self.vstat_of(v));
        }
        self.vstat = out;
    }

    fn vstat_of(&self, v: &Vehicle) -> VStat {
        let link = &self.net.links[v.link()];
        let on_approach = link.kind == LinkKind::Inbound;
        let mut st = VStat { rank: 2.0, key: self.t + 1e3, ..Default::default() };
        if v.kind == VKind::Emergency {
            st.rank = 5.0;
            return st;
        }
        let d_line = link.length() - v.s;
        match self.mode {
            ControlMode::Signal if self.signalized() => {
                let sig = self.signal.status(&self.net, v.movement);
                let yellow_go = matches!(v.yellow_dec, Some((id, true)) if id == self.signal.change_id);
                if on_approach {
                    match sig {
                        Sig::G | Sig::Rt => st.rank = 2.0,
                        Sig::P => st.rank = 1.0,
                        Sig::Y | Sig::R if yellow_go => st.rank = 2.0,
                        Sig::Y => {
                            st.must_stop = true;
                            st.excluded = true;
                            st.reason_stop = 1;
                        }
                        Sig::R => {
                            if v.turn == Turn::R && self.rtor_ok(v.movement) && v.stop_done && self.t - v.stop_t > 1.0 {
                                st.rank = 0.0;
                                st.hold_line = true;
                            } else {
                                st.must_stop = true;
                                st.excluded = true;
                                st.reason_stop = 1;
                            }
                        }
                    }
                } else if link.kind.in_box() {
                    st.rank = match sig {
                        Sig::G | Sig::Rt => 2.0,
                        Sig::P => 1.0,
                        _ => 3.0,
                    };
                }
                let _ = d_line;
            }
            ControlMode::Signal | ControlMode::AllWayStop => {
                if on_approach {
                    if v.stop_done && self.t - v.stop_t > 0.8 {
                        st.rank = 1.0;
                        st.key = v.stop_t;
                        st.hold_line = true;
                    } else {
                        st.must_stop = true;
                        st.excluded = true;
                        st.reason_stop = 2;
                    }
                } else if link.kind.in_box() {
                    st.rank = 3.0;
                }
            }
            ControlMode::TwoWayStop => {
                let minor = self.net.legs[v.leg].minor;
                if minor {
                    if on_approach {
                        if v.stop_done && self.t - v.stop_t > 0.8 {
                            st.rank = 0.5;
                            st.hold_line = true;
                        } else {
                            st.must_stop = true;
                            st.excluded = true;
                            st.reason_stop = 2;
                        }
                    } else if link.kind.in_box() {
                        st.rank = 3.0;
                    }
                } else {
                    st.rank = if v.turn == Turn::L { 1.0 } else { 2.0 };
                }
            }
            ControlMode::Roundabout => {
                st.rank = if matches!(link.kind, LinkKind::Inbound | LinkKind::Entry) { 1.0 } else { 2.0 };
            }
        }
        st
    }

    /// Right turn on red allowed for movement `m` right now.
    pub fn rtor_ok(&self, m: usize) -> bool {
        self.rtor
            && !self.signal.lpi_held(m)
            && !(self.ntor_peds
                && self.peds.iter().any(|p| p.state != PedState::Leaving && self.net.movements[m].crosswalks.contains(&p.cw)))
    }

    fn route_index(v: &Vehicle, link: LinkId) -> Option<usize> {
        v.route.iter().position(|&l| l == link)
    }

    fn eta(d: f64, v: f64, a: f64, vmax: f64) -> f64 {
        if d <= 0.0 {
            return 0.0;
        }
        let vmax = vmax.max(0.5);
        if v >= vmax {
            return d / v;
        }
        let t1 = (vmax - v) / a;
        let d1 = v * t1 + 0.5 * a * t1 * t1;
        if d <= d1 { (-v + (v * v + 2.0 * a * d).sqrt()) / a } else { t1 + (d - d1) / vmax }
    }

    fn vmax_on(&self, v: &Vehicle, link: LinkId) -> f64 {
        let l = &self.net.links[link];
        if l.kind.in_box() { v.v0.min(l.curve_speed) } else { v.v0 }
    }

    /// Vehicles that will pass point `s_other` on link `other`, with distance of
    /// their front to that point.
    fn threats(&self, other: LinkId, s_other: f64, ext: f64, exclude: usize, out: &mut Vec<(usize, f64)>) {
        out.clear();
        let lo = &self.net.links[other];
        for &j in &self.occ[other] {
            if j == exclude {
                continue;
            }
            let w = &self.vehicles[j];
            let d = s_other - w.s;
            if d + ext + w.len() > 0.0 {
                out.push((j, d));
            }
        }
        // Vehicles whose front has moved on (possibly several links) but whose body
        // still covers the point.
        let mut frontier: Vec<(LinkId, f64)> = lo.succ.iter().map(|&x| (x, lo.length() - s_other)).collect();
        while let Some((l, dist)) = frontier.pop() {
            for &j in &self.occ[l] {
                let w = &self.vehicles[j];
                if j == exclude || out.iter().any(|o| o.0 == j) {
                    continue;
                }
                if let Some(k) = w.route[..w.ri].iter().position(|&x| x == other) {
                    let d = route_dist_static(&self.net, w, k, s_other);
                    if d + ext + w.len() > 0.0 {
                        out.push((j, d));
                    }
                }
            }
            let nl = self.net.links[l].length();
            if dist + nl < 20.0 {
                for &x in &self.net.links[l].succ {
                    frontier.push((x, dist + nl));
                }
            }
        }
        // Upstream links whose vehicles are routed through `other`.
        let mut stack: Vec<(LinkId, f64, u8)> = vec![(other, s_other, 0)];
        while let Some((l, off, depth)) = stack.pop() {
            for &p in &self.net.links[l].pred {
                let lp = self.net.links[p].length();
                for &j in &self.occ[p] {
                    if j == exclude {
                        continue;
                    }
                    let w = &self.vehicles[j];
                    if !w.route[w.ri..].contains(&other) {
                        continue;
                    }
                    let d = (lp - w.s) + off;
                    if d < HORIZON {
                        out.push((j, d));
                    }
                }
                if off + lp < HORIZON && depth < 4 {
                    stack.push((p, off + lp, depth + 1));
                }
            }
        }
    }

    /// Does vehicle `i` have to yield at conflict `cid` (seen from its own route)?
    fn must_yield(&self, i: usize, cid: usize, d_a: f64, buf: &mut Vec<(usize, f64)>) -> Option<usize> {
        let a = &self.vehicles[i];
        let sa = self.vstat[i];
        let c = &self.net.conflicts[cid];
        let my_link = if Self::route_index(a, c.a).is_some() { c.a } else { c.b };
        let (_, other, s_other) = c.side(my_link);
        let ((za0, za1), (zv0, zv1)) = c.zones(my_link);
        let spec = a.spec();
        let vm = self.vmax_on(a, my_link);
        // Merges compare arrival at the merge point (then the follower just follows);
        // crossings compare occupancy of the whole conflict zone.
        let (za0, za1, zv0m, zv1m) = if c.merge { (0.0, 0.0, 0.0, 0.0) } else { (za0, za1, zv0, zv1) };
        let ta_in = Self::eta(d_a - za0, a.v, spec.accel, vm);
        // Conservative: our own clearing assumes sluggish acceleration...
        let ta_out = Self::eta(d_a + za1 + spec.length, a.v, spec.accel * 0.7, vm);
        self.threats(other, s_other, zv1, i, buf);
        for &(j, d_v) in buf.iter() {
            let w = &self.vehicles[j];
            let sw = self.vstat[j];
            // Physically inside the conflict zone always counts as holding it.
            let occupying = d_v - zv0 < 0.3;
            // A vehicle that can no longer stop before the zone has the right of way.
            let cant_stop = w.v * w.v / (2.0 * (d_v - zv0).max(0.1)) > 4.5;
            let committed = w.commits.contains(&cid) || occupying || cant_stop;
            if sw.excluded && !committed {
                continue;
            }
            // A stopped vehicle that is itself waiting (behind a waiting leader, or
            // yielding to someone) is not arriving soon; ignoring it unless it is
            // physically in the zone breaks wait-for cycles.
            let idle = w.v < 0.3 && (w.blocked || w.yield_to.is_some() || w.reason == Reason::Pedestrian);
            if !occupying && idle {
                continue;
            }
            let wspec = w.spec();
            let wm = self.vmax_on(w, other);
            let tv_in = Self::eta(d_v - zv0m, w.v, wspec.accel, wm);
            // ...and theirs assumes they may not speed up (they could be held by a leader),
            // must first react if stopped, and won't clear at all while waiting in the zone.
            let react = if w.v < 0.1 { (w.reaction - w.go_t).max(0.0) } else { 0.0 };
            let tv_out = if occupying && idle {
                f64::INFINITY
            } else {
                react
                    + Self::eta(d_v + zv1m + wspec.length, w.v, wspec.accel, wm)
                        .max((d_v + zv1m + wspec.length).max(0.0) / w.v.max(3.0))
            };
            let w_priority = if committed {
                true
            } else if sw.rank > sa.rank {
                true
            } else if sw.rank < sa.rank {
                false
            } else {
                let (ka, kw) = if sa.key < 1e8 && sw.key < 1e8 && sa.key < self.t + 1e2 && sw.key < self.t + 1e2 {
                    (sa.key, sw.key)
                } else {
                    (self.t + ta_in, self.t + tv_in)
                };
                kw < ka || (kw == ka && w.id < a.id)
            };
            if !w_priority {
                continue;
            }
            let mut margin = if c.merge { 1.2 } else { 0.8 };
            if sa.rank <= 1.0 && sw.rank > sa.rank {
                margin += if c.merge { 0.8 } else { 1.2 };
            }
            if ta_in < tv_out + margin && tv_in < ta_out + margin {
                return Some(j);
            }
        }
        None
    }

    /// A (nearly) stopped vehicle whose body is inside conflict zone `cid`, seen from
    /// vehicle `i`'s side.
    fn stalled_in_zone(&self, i: usize, cid: usize, buf: &mut Vec<(usize, f64)>) -> Option<usize> {
        let a = &self.vehicles[i];
        let c = &self.net.conflicts[cid];
        let my_link = if Self::route_index(a, c.a).is_some() { c.a } else { c.b };
        let (_, other, s_other) = c.side(my_link);
        let (_, (zv0, zv1)) = c.zones(my_link);
        self.threats(other, s_other, zv1, i, buf);
        buf.iter().find(|&&(j, d_v)| {
            let w = &self.vehicles[j];
            w.v < 0.5 && d_v - zv0 < 0.3 && d_v + zv1 + w.len() > 0.0
        }).map(|x| x.0)
    }

    /// Would every later conflict on link `lid` (beyond `s_from`) let us through now?
    fn rest_clear(&self, i: usize, lid: LinkId, s_from: f64, base: f64, buf: &mut Vec<(usize, f64)>) -> bool {
        let v = &self.vehicles[i];
        for &cid in &self.net.links[lid].conflicts {
            let (s_c, _, _) = self.net.conflicts[cid].side(lid);
            if s_c <= s_from + 0.01 || v.commits.contains(&cid) {
                continue;
            }
            if self.must_yield(i, cid, base + s_c, buf).is_some() {
                return false;
            }
        }
        true
    }

    /// Is anyone (not stopping for a signal/sign) expected at conflict `cid` soon?
    fn live_threat(&self, i: usize, cid: usize, buf: &mut Vec<(usize, f64)>) -> bool {
        let a = &self.vehicles[i];
        let c = &self.net.conflicts[cid];
        let my_link = if Self::route_index(a, c.a).is_some() { c.a } else { c.b };
        let (_, other, s_other) = c.side(my_link);
        self.threats(other, s_other, c.zones(my_link).1.1, i, buf);
        buf.iter().any(|&(j, d_v)| {
            let w = &self.vehicles[j];
            !self.vstat[j].excluded && d_v > -1.0 && Self::eta(d_v, w.v, w.spec().accel, w.v0) < 12.0
        })
    }

    fn ped_blocks(&self, i: usize, cw: usize, u_c: f64, d_a: f64, in_box: bool) -> bool {
        let a = &self.vehicles[i];
        let spec = a.spec();
        let vm = a.v0;
        let ta_in = Self::eta(d_a - 2.2, a.v, spec.accel, vm);
        let ta_out = Self::eta(d_a + 2.2 + spec.length, a.v, spec.accel, vm);
        let walk = !self.signalized() || self.signal.cw[cw].state == PedSig::Walk;
        for p in &self.peds {
            if p.cw != cw {
                continue;
            }
            // Unsignalised: a waiting pedestrian asserts right of way to any vehicle
            // that can still stop comfortably.
            let comfortable = !in_box && a.v * a.v / (2.0 * (d_a - 2.6).max(0.1)) < 2.5;
            let active = match p.state {
                PedState::Crossing => true,
                PedState::Waiting if self.signalized() => walk,
                PedState::Waiting => p.t_state > 1.5 && comfortable,
                PedState::Leaving => false,
            };
            if !active {
                continue;
            }
            let x = (u_c - p.u) * p.dir;
            let tp_in = ((x - 2.6) / p.speed).max(0.0);
            let tp_out = (x + 2.6) / p.speed;
            if tp_out < 0.0 {
                continue;
            }
            if ta_in < tp_out + 1.0 && tp_in < ta_out + 1.0 {
                return true;
            }
        }
        false
    }

    // ---------------------------------------------------------------- driving

    fn idm(spec: &VSpec, v: f64, v0: f64, gap: f64, dv: f64) -> f64 {
        let s_star = spec.min_gap + (v * spec.headway + v * dv / (2.0 * (spec.accel * spec.decel).sqrt())).max(0.0);
        spec.accel * (1.0 - (v / v0.max(0.1)).powi(4) - (s_star / gap.max(0.05)).powi(2))
    }

    /// Nearest leader ahead: (gap from own front to leader rear, leader speed).
    fn leader(&self, i: usize) -> Option<(f64, f64, usize)> {
        let v = &self.vehicles[i];
        let link = v.link();
        let mut best: Option<(f64, f64, usize)> = None;
        fn consider(best: &mut Option<(f64, f64, usize)>, gap: f64, vl: f64, j: usize) {
            if best.is_none_or(|b| gap < b.0) {
                *best = Some((gap, vl, j));
            }
        }
        for &j in &self.occ[link] {
            if j == i {
                continue;
            }
            let w = &self.vehicles[j];
            if w.s > v.s || (w.s == v.s && w.id < v.id) {
                consider(&mut best, w.s - w.len() - v.s, w.v, j);
            }
        }
        // Vehicles on diverging sibling paths that still overlap ours laterally.
        if v.ri > 0 {
            self.scan_siblings(i, v.route[v.ri - 1], link, -v.s, &mut best);
        }
        if best.is_some() {
            return best;
        }
        let mut dist = self.net.links[link].length() - v.s;
        let mut prev = link;
        for k in v.ri + 1..v.route.len() {
            let next = v.route[k];
            for &j in &self.occ[next] {
                let w = &self.vehicles[j];
                consider(&mut best, dist + w.s - w.len(), w.v, j);
            }
            self.scan_siblings(i, prev, next, dist, &mut best);
            if best.is_some() {
                return best;
            }
            dist += self.net.links[next].length();
            if dist > 160.0 {
                break;
            }
            prev = next;
        }
        best
    }

    /// Leaders on links that branch from `from` alongside `mine` (diverges), where
    /// `base` is the distance from our front to the start of `mine`.
    fn scan_siblings(&self, i: usize, from: LinkId, mine: LinkId, base: f64, best: &mut Option<(f64, f64, usize)>) {
        let v = &self.vehicles[i];
        for &sib in &self.net.links[from].succ {
            if sib == mine {
                continue;
            }
            for &j in &self.occ[sib] {
                if j == i {
                    continue;
                }
                let w = &self.vehicles[j];
                let rear = w.s - w.len();
                let gap = base + rear;
                if gap < -1.0 || gap > 40.0 {
                    continue;
                }
                let overlaps = rear < 1.0 || {
                    let pr = self.point_behind(w, w.len());
                    let p_mine = self.net.links[mine].path.point(rear);
                    pr.dist(p_mine) < (w.spec().width + v.spec().width) * 0.5 + 0.4
                };
                if overlaps && best.is_none_or(|b| gap < b.0) {
                    *best = Some((gap.max(0.0), w.v, j));
                }
            }
        }
    }

    fn drive(&mut self, dt: f64) {
        // Decide in order of proximity to the conflict area so commitments made by
        // vehicles closer to a conflict are visible to those behind them.
        let mut order: Vec<(f64, usize)> = self
            .vehicles
            .iter()
            .enumerate()
            .map(|(i, v)| {
                let l = &self.net.links[v.link()];
                let k = match l.kind {
                    LinkKind::Inbound => 100.0 + (l.length() - v.s),
                    LinkKind::Outbound => 1000.0 - v.s,
                    _ => -v.s,
                };
                (k, i)
            })
            .collect();
        order.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        let mut buf = Vec::new();
        let mut accs = vec![0.0; self.vehicles.len()];
        for &(_, i) in &order {
            let (acc, reason, commits, stop_done, ydec, (blocked, yield_to), drop) = self.decide(i, &mut buf);
            accs[i] = acc;
            let v = &mut self.vehicles[i];
            v.commits.retain(|c| !drop.contains(c));
            v.reason = reason;
            v.blocked = blocked;
            v.yield_to = yield_to;
            let vid = v.id;
            let mut fresh = vec![];
            for c in commits {
                if !v.commits.contains(&c) {
                    v.commits.push(c);
                    fresh.push(c);
                }
            }
            if let Some(sd) = stop_done {
                if sd && !v.stop_done {
                    v.stop_done = true;
                    v.stop_t = self.t;
                }
            }
            if ydec.is_some() {
                v.yellow_dec = ydec;
            }
            if cfg!(debug_assertions) || std::env::var_os("TS_DEBUG").is_some() {
                for c in fresh {
                    for w in &self.vehicles {
                        if w.id != vid && w.commits.contains(&c) && !w.route.contains(&self.vehicles[i].route[self.vehicles[i].ri.min(self.vehicles[i].route.len()-1)]) && !self.vehicles[i].route.iter().any(|l| w.route[w.ri] == *l) {
                            let me = &self.vehicles[i];
                            let msg = format!(
                                "t={:.1} DOUBLE commit c{} by #{} {} v={:.1} s={:.1} st={:?} | holder #{} {} v={:.1} s={:.1} blocked={} reason={:?}",
                                self.t, c, vid, self.net.movements[me.movement].name, me.v, me.s, self.vstat[i],
                                w.id, self.net.movements[w.movement].name, w.v, w.s, w.blocked, w.reason
                            );
                            if self.collision_log.len() < 30 {
                                self.collision_log.push(msg);
                            }
                        }
                    }
                }
            }
        }
        // Integrate.
        let mut done_idx = Vec::new();
        let mut person_delay = 0.0;
        for i in 0..self.vehicles.len() {
            let mut acc = accs[i];
            let veh = &mut self.vehicles[i];
            let spec = veh.kind.spec();
            // Drivers take a moment to react before pulling away (start-up lost time).
            // While waiting the vehicle keeps re-deciding, so any gap it accepts is judged
            // at the moment it actually moves.
            if veh.v < 0.1 {
                if acc > 0.0 {
                    if veh.go_t < veh.reaction {
                        veh.go_t += dt;
                        acc = 0.0;
                    }
                } else {
                    veh.go_t = 0.0;
                }
            }
            let v_old = veh.v;
            let mut v_new = v_old + acc * dt;
            let ds = if v_new < 0.0 {
                v_new = 0.0;
                if acc < 0.0 { (v_old * v_old / (-2.0 * acc)).min(v_old * dt) } else { 0.0 }
            } else {
                0.5 * (v_old + v_new) * dt
            };
            veh.prev_s = veh.s;
            veh.s += ds;
            veh.v = v_new;
            veh.a = acc;
            // Running delay (lost time relative to free-flow speed at this location).
            let link = &self.net.links[veh.route[veh.ri]];
            let vref = if link.kind.in_box() { veh.v0.min(link.curve_speed) } else { veh.v0 };
            let lost = dt * (1.0 - v_new / vref.max(0.1)).max(0.0);
            veh.delay += lost;
            person_delay += lost * spec.occupancy;
            if v_new < 0.5 {
                veh.stopped_time += dt;
                if veh.moving {
                    veh.stops += 1;
                    veh.moving = false;
                }
            } else if v_new > 3.0 {
                veh.moving = true;
            }
            veh.fuel_ml += fuel_ml(spec, v_new, acc, dt);
            if link.kind == LinkKind::Inbound && v_new < 2.0 && veh.queue_since.is_none() {
                veh.queue_since = Some(self.t);
            }
            while veh.s >= self.net.links[veh.route[veh.ri]].length() {
                let l = self.net.links[veh.route[veh.ri]].length();
                if veh.ri + 1 >= veh.route.len() {
                    done_idx.push(i);
                    break;
                }
                veh.s -= l;
                veh.prev_s -= l;
                veh.ri += 1;
                if self.net.links[veh.route[veh.ri]].kind.in_box() && veh.ri == 1 {
                    veh.queue_since = None;
                }
            }
            // Release conflicts the vehicle has fully cleared.
            if !veh.commits.is_empty() {
                let veh_ref = &*veh;
                let mut keep = Vec::with_capacity(veh_ref.commits.len());
                for &cid in &veh_ref.commits {
                    let c = &self.net.conflicts[cid];
                    let k = Self::route_index(veh_ref, c.a).or_else(|| Self::route_index(veh_ref, c.b));
                    if let Some(k) = k {
                        let s_c = if veh_ref.route[k] == c.a { c.sa } else { c.sb };
                        let d = route_dist_static(&self.net, veh_ref, k, s_c);
                        if d + c.zones(veh_ref.route[k]).0.1 + veh_ref.len() + 0.5 > 0.0 {
                            keep.push(cid);
                        }
                    }
                }
                veh.commits = keep;
            }
        }
        self.last_reward = person_delay;
        // Surrogate safety: a vehicle crossing a crosswalk within ~1.5 s of a walking
        // pedestrian's path (post-encroachment time) counts as a near-miss.
        for v in &self.vehicles {
            let lid = v.route[v.ri];
            for &pci in &self.net.links[lid].ped_conflicts {
                let pc = &self.net.ped_conflicts[pci];
                if !(v.prev_s < pc.s && v.s >= pc.s) {
                    continue;
                }
                let close = self.peds.iter().any(|p| {
                    p.cw == pc.cw && p.state == PedState::Crossing && ((p.u - pc.u).abs() - 1.0).max(0.0) / p.speed < 1.5
                });
                if close {
                    self.stats.ped_near_miss.push(self.t);
                }
            }
        }
        // Remove finished vehicles.
        done_idx.sort_unstable();
        for &i in done_idx.iter().rev() {
            let v = self.vehicles.swap_remove(i);
            let travel = self.t + dt - v.arrival_t;
            let mv = &self.net.movements[v.movement];
            self.stats.record(Done {
                kind: v.kind,
                leg: v.leg,
                turn: mv.turn,
                movement: v.movement,
                delay: (travel - v.ff_time).max(0.0),
                stops: v.stops,
                travel,
                stopped: v.stopped_time,
                fuel_ml: v.fuel_ml,
                co2_g: v.fuel_ml / 1000.0 * v.spec().co2_per_l * 1000.0,
                persons: v.persons(),
                t_exit: self.t + dt,
            });
        }
        self.rebuild_occ();
        if self.check_safety {
            self.check_overlaps();
        }
    }

    /// Returns (accel, reason, new commits, stop-done, yellow decision).
    #[allow(clippy::type_complexity)]
    fn decide(&self, i: usize, buf: &mut Vec<(usize, f64)>) -> (f64, Reason, Vec<usize>, Option<bool>, Option<(u64, bool)>, (bool, Option<u64>), Vec<usize>) {
        let v = &self.vehicles[i];
        let spec = v.spec();
        let st = self.vstat[i];
        let link_id = v.link();
        let link = &self.net.links[link_id];
        let mut reason = Reason::Free;
        let mut v0 = if link.kind.in_box() { v.v0.min(link.curve_speed) } else { v.v0 };
        let on_approach = link.kind == LinkKind::Inbound;
        let d_line = link.length() - v.s;
        let mut commits = Vec::new();
        let mut stop_done = None;
        let mut ydec = None;

        // Emergency vehicles slow down when crossing against the signal.
        if v.kind == VKind::Emergency && on_approach && self.signalized() && d_line < 45.0 {
            let sig = self.signal.status(&self.net, v.movement);
            if !sig.is_green() {
                v0 = v0.min(7.0);
            }
        }
        let mut acc = Self::idm(spec, v.v, v0, 1e9, 0.0);
        let mut blocked = false;
        if let Some((gap, vl, j)) = self.leader(i) {
            let w = &self.vehicles[j];
            blocked = v.v < 0.5
                && gap < 8.0
                && w.v < 0.3
                && (w.blocked || matches!(w.reason, Reason::Yielding | Reason::Pedestrian | Reason::Signal | Reason::StopSign));
            let a2 = Self::idm(spec, v.v, v0, gap, v.v - vl);
            if a2 < acc {
                acc = a2;
                reason = Reason::Following;
            }
            if gap < -0.5 {
                acc = acc.min(-8.0);
            }
        }
        // Curve speeds ahead.
        {
            let mut d = link.length() - v.s;
            for &nl in &v.route[v.ri + 1..] {
                let l2 = &self.net.links[nl];
                if l2.kind.in_box() && l2.curve_speed < v.v {
                    let a2 = -(v.v * v.v - l2.curve_speed * l2.curve_speed) / (2.0 * (d - 1.0).max(0.5));
                    if a2 < acc {
                        acc = a2;
                        reason = Reason::Curve;
                    }
                }
                d += l2.length();
                if d > LOOK {
                    break;
                }
            }
        }
        let stop_at = |gap: f64| Self::idm(spec, v.v, v0, gap + spec.min_gap - 0.6, v.v);
        let can_stop = |gap: f64| v.v * v.v / (2.0 * gap.max(0.05)) < 6.5;

        // Signal / stop sign at the stop line.
        let mut line_hold = false;
        if on_approach && v.kind != VKind::Emergency {
            if self.signalized() && self.mode == ControlMode::Signal {
                let sig = self.signal.status(&self.net, v.movement);
                if sig == Sig::Y && !matches!(v.yellow_dec, Some((id, _)) if id == self.signal.change_id) {
                    let need = v.v * v.v / (2.0 * d_line.max(0.1));
                    let go = need > 3.4 || d_line < 1.0;
                    ydec = Some((self.signal.change_id, go));
                    if go {
                        // Decided this step; treat as permitted.
                    } else {
                        line_hold = true;
                        reason = Reason::Signal;
                    }
                } else if st.must_stop {
                    line_hold = true;
                    reason = Reason::Signal;
                }
                if sig == Sig::R && v.turn == Turn::R && self.rtor_ok(v.movement) && d_line < 2.5 && v.v < 0.3 && !v.stop_done {
                    stop_done = Some(true);
                }
            } else if st.must_stop {
                line_hold = true;
                reason = Reason::StopSign;
                if d_line < 2.5 && v.v < 0.3 {
                    stop_done = Some(true);
                }
            }
        }
        if line_hold {
            let a2 = stop_at(d_line);
            if a2 < acc {
                acc = a2;
            }
            return (acc.clamp(-9.0, spec.accel), reason, commits, stop_done, ydec, (blocked, None), vec![]);
        }

        // Conflict points ahead.
        let mut stop_gap: Option<(f64, Reason)> = None;
        let mut yield_to = None;
        let hold_line_mode = st.hold_line && on_approach;
        let look = LOOK.max(v.v * 5.0 + 20.0);
        // Conflicts we intend to pass this step: (id, distance, zone).
        let mut passing: Vec<(usize, f64, f64, f64)> = Vec::new();
        let mut stale_drop: Vec<usize> = Vec::new();
        let mut k = v.ri;
        let mut base = -v.s; // distance from front to start of route[k]
        'outer: while k < v.route.len() {
            let lid = v.route[k];
            let l = &self.net.links[lid];
            if base > look || l.kind == LinkKind::Outbound {
                break;
            }
            for &cid in &l.conflicts {
                let c = &self.net.conflicts[cid];
                let ((zn, zone), _) = c.zones(lid);
                let (s_c, _, _) = c.side(lid);
                let d = base + s_c;
                if d + zone + spec.length < 0.0 {
                    continue;
                }
                if v.commits.contains(&cid) && !c.merge {
                    // A stopped vehicle's commitment to a zone it hasn't entered is stale:
                    // others may have gone around it, so it must re-check before moving.
                    let stale = v.v < 0.3 && v.a <= 0.05 && d - zn > 0.3;
                    if !stale {
                        // A commitment grants priority, not room: if someone has stalled
                        // inside the zone since, stop short of it while we still can.
                        if d - zn > 0.3 {
                            if let Some(j) = self.stalled_in_zone(i, cid, buf) {
                                let gap = d - zn - 0.8;
                                if can_stop(gap) || v.v < 1.0 {
                                    yield_to = Some(self.vehicles[j].id);
                                    stale_drop.push(cid);
                                    stop_gap = Some((gap, Reason::Yielding));
                                    break 'outer;
                                }
                            }
                        }
                        passing.push((cid, d, zone, zn));
                        continue;
                    }
                    stale_drop.push(cid);
                }
                if d > look {
                    break;
                }
                if let Some(j) = self.must_yield(i, cid, d, buf) {
                    yield_to = Some(self.vehicles[j].id);
                    let gap = if hold_line_mode { d_line } else { d - zn - 0.8 };
                    if can_stop(gap) || v.v < 1.0 {
                        stop_gap = Some((gap, Reason::Yielding));
                        break 'outer;
                    }
                    if !c.merge {
                        // Merges are never locked: they are re-resolved every step.
                        if !c.merge {
                            commits.push(cid);
                        }
                    }
                    passing.push((cid, d, zone, zn));
                } else {
                    let brake = v.v * v.v / (2.0 * 3.0) + 3.0;
                    if d - zn < brake || hold_line_mode {
                        // Permissive movers accept a gap only for the rest of the turn.
                        if st.rank <= 1.0 && !hold_line_mode && !self.rest_clear(i, lid, s_c, base, buf) {
                            let gap = d - zn - 0.8;
                            if can_stop(gap) || v.v < 1.0 {
                                stop_gap = Some((gap, Reason::Yielding));
                                break 'outer;
                            }
                        }
                        // Merges are never locked: they are re-resolved every step.
                        if !c.merge {
                            commits.push(cid);
                        }
                    }
                    passing.push((cid, d, zone, zn));
                }
            }
            for &pci in &l.ped_conflicts {
                let pc = &self.net.ped_conflicts[pci];
                let d = base + pc.s;
                if d + 2.2 + spec.length < 0.0 || d - 2.2 < 0.0 {
                    continue;
                }
                if self.ped_blocks(i, pc.cw, pc.u, d, link.kind.in_box()) {
                    let gap = if hold_line_mode { d_line } else { d - 2.6 };
                    if can_stop(gap) || v.v < 1.0 {
                        if stop_gap.is_none_or(|g| gap < g.0) {
                            stop_gap = Some((gap, Reason::Pedestrian));
                        }
                        break 'outer;
                    }
                }
            }
            base += l.length();
            k += 1;
        }
        let mut drop = stale_drop;
        if let Some((mut gap, r)) = stop_gap {
            if hold_line_mode {
                // Holding at the line: give back commitments made this step.
                commits.clear();
            } else {
                // Don't block the box: if stopping here would leave our body in a
                // conflict zone that live traffic needs (including zones we would only
                // creep into while waiting), stop before it instead.
                let mut ahead = passing.clone();
                let mut base = -v.s;
                for &lid in &v.route[v.ri..] {
                    let l = &self.net.links[lid];
                    if base > gap || l.kind == LinkKind::Outbound {
                        break;
                    }
                    for &cid in &l.conflicts {
                        let c = &self.net.conflicts[cid];
                        let ((zn, zone), _) = c.zones(lid);
                        let d = base + c.side(lid).0;
                        if d - zn < gap && d + zone + spec.length > 0.0 && !ahead.iter().any(|x| x.0 == cid) {
                            ahead.push((cid, d, zone, zn));
                        }
                    }
                    base += l.length();
                }
                ahead.sort_by(|a, b| (a.1 - a.3).partial_cmp(&(b.1 - b.3)).unwrap());
                loop {
                    let mut changed = false;
                    for &(cid, d, zone, zn) in ahead.iter().rev() {
                        if drop.contains(&cid) || !(d - zn < gap && d + zone > gap - spec.length) {
                            continue;
                        }
                        let g2 = d - zn - 0.8;
                        if g2 > -0.3 && g2 < gap && (can_stop(g2) || v.v < 1.0) && self.live_threat(i, cid, buf) {
                            gap = g2;
                            drop.push(cid);
                            changed = true;
                        }
                    }
                    if !changed {
                        break;
                    }
                }
                commits.retain(|c| !drop.contains(c));
            }
            let a2 = stop_at(gap.max(0.0));
            if a2 < acc {
                acc = a2;
                reason = r;
            }
        }
        (acc.clamp(-9.0, spec.accel), reason, commits, stop_done, ydec, (blocked, if stop_gap.is_some() { yield_to } else { None }), drop)
    }

    /// Body as a chain of oriented rectangles (centre, unit axis, half-length,
    /// half-width) that follows the path, so long vehicles bend through turns.
    pub fn body(&self, v: &Vehicle) -> Vec<(V2, V2, f64, f64)> {
        self.body_lagged(v, 0.0)
    }

    /// Body drawn `lag` metres back along the route (see [`Sim::lag`]).
    pub fn body_lagged(&self, v: &Vehicle, lag: f64) -> Vec<(V2, V2, f64, f64)> {
        let len = v.len();
        let n = (len / 4.5).ceil().max(1.0) as usize;
        let seg = len / n as f64;
        let mut out = Vec::with_capacity(n);
        let mut f = self.point_behind(v, lag);
        for k in 1..=n {
            let r = self.point_behind(v, lag + seg * k as f64);
            out.push((f.lerp(r, 0.5), (f - r).norm(), f.dist(r) * 0.5, v.spec().width * 0.5));
            f = r;
        }
        out
    }

    fn check_overlaps(&mut self) {
        // Safety metric: separating-axis test between vehicle bodies in or next to the
        // intersection on different links (small tolerance for drawn mirrors etc.).
        let near: Vec<usize> = (0..self.vehicles.len())
            .filter(|&i| {
                let v = &self.vehicles[i];
                let l = &self.net.links[v.link()];
                l.kind.in_box() || (l.kind != LinkKind::Inbound && v.s < v.len() + 2.0)
            })
            .collect();
        let bodies: Vec<_> = near.iter().map(|&i| self.body(&self.vehicles[i])).collect();
        for x in 0..near.len() {
            for y in x + 1..near.len() {
                let (vi, vj) = (&self.vehicles[near[x]], &self.vehicles[near[y]]);
                if vi.link() == vj.link() || vi.route.contains(&vj.link()) && vj.route.contains(&vi.link()) {
                    continue;
                }
                if bodies[x].iter().any(|a| bodies[y].iter().any(|b| obb_overlap(*a, *b, 0.25))) {
                    self.collisions += 1;
                    if self.collision_log.len() < 30 {
                        let d = |v: &Vehicle| {
                            format!(
                                "#{} {} {} v={:.1} s={:.1}/{:.1} commits={:?} reason={:?} sig={:?}",
                                v.id,
                                self.net.movements[v.movement].name,
                                v.kind.name(),
                                v.v,
                                v.s,
                                self.net.links[v.link()].length(),
                                v.commits,
                                v.reason,
                                if self.signalized() { Some(self.signal.status(&self.net, v.movement)) } else { None }
                            )
                        };
                        let msg = format!("t={:.1} | {} | {}", self.t, d(vi), d(vj));
                        self.collision_log.push(msg);
                    }
                }
            }
        }
    }

    /// Point at distance `back` behind the front of the vehicle, following its route.
    pub fn point_behind(&self, v: &Vehicle, back: f64) -> V2 {
        let mut s = v.s - back;
        let mut k = v.ri;
        while s < 0.0 && k > 0 {
            k -= 1;
            s += self.net.links[v.route[k]].length();
        }
        self.net.links[v.route[k]].path.point(s)
    }

    pub fn front(&self, v: &Vehicle) -> V2 {
        self.net.links[v.link()].path.point(v.s)
    }

    pub fn center(&self, v: &Vehicle) -> V2 {
        self.point_behind(v, v.len() * 0.5)
    }

    /// How far behind its current position to draw a vehicle so that it moves continuously
    /// between steps: `alpha` is the fraction of the next step that has elapsed in real time.
    pub fn lag(&self, v: &Vehicle, alpha: f64) -> f64 {
        (v.s - v.prev_s).max(0.0) * (1.0 - alpha.clamp(0.0, 1.0))
    }

    /// Pedestrian position interpolated between the last two steps.
    pub fn ped_pos_lerp(&self, p: &Ped, alpha: f64) -> V2 {
        let cur = self.ped_pos(p);
        p.prev_pos.map_or(cur, |prev| prev.lerp(cur, alpha.clamp(0.0, 1.0)))
    }

    // ---------------------------------------------------------------- pedestrians

    fn step_peds(&mut self, dt: f64) {
        let signal = self.signalized();
        let mut finished = Vec::new();
        for pi in 0..self.peds.len() {
            self.peds[pi].prev_pos = Some(self.ped_pos(&self.peds[pi]));
            let (cw, state, t_state) = {
                let p = &self.peds[pi];
                (p.cw, p.state, p.t_state)
            };
            let len = self.net.crosswalks[cw].length;
            match state {
                PedState::Waiting => {
                    let go = if signal {
                        self.signal.cw[cw].state == PedSig::Walk
                    } else {
                        t_state > 1.5 && !self.vehicle_near_cw(cw)
                    };
                    if go {
                        let wait = self.t - self.peds[pi].arrive_t;
                        self.stats.ped_waits.push((self.t, wait));
                        let p = &mut self.peds[pi];
                        p.state = PedState::Crossing;
                        p.t_state = 0.0;
                    }
                }
                PedState::Crossing => {
                    let pos = self.ped_pos(&self.peds[pi]);
                    let dirv = (self.net.crosswalks[cw].b - self.net.crosswalks[cw].a).norm() * self.peds[pi].dir;
                    let blocked = self.vehicle_at(pos + dirv * 1.0, 0.9);
                    let p = &mut self.peds[pi];
                    if !blocked {
                        // Hurry up during flashing DON'T WALK.
                        let hurry = if signal && self.signal.cw[cw].state != PedSig::Walk { 1.25 } else { 1.0 };
                        p.u += p.dir * p.speed * hurry * dt;
                        p.anim += dt * p.speed * 4.0;
                    }
                    if (p.dir > 0.0 && p.u > len + 1.0) || (p.dir < 0.0 && p.u < -1.0) {
                        p.state = PedState::Leaving;
                        p.t_state = 0.0;
                    }
                }
                PedState::Leaving => {
                    if t_state > 2.0 {
                        finished.push(pi);
                    }
                }
            }
            let p = &mut self.peds[pi];
            p.t_state += dt;
            if p.state == PedState::Leaving {
                p.anim += dt * 4.0;
            }
        }
        for &i in finished.iter().rev() {
            self.peds.swap_remove(i);
        }
    }

    pub fn ped_pos(&self, p: &Ped) -> V2 {
        let cw = &self.net.crosswalks[p.cw];
        let dir = (cw.b - cw.a).norm();
        let mut u = p.u;
        let mut lat = p.lateral;
        if p.state == PedState::Leaving {
            // Walk off along the sidewalk, away from the road.
            let leg = &self.net.legs[cw.leg];
            let out = leg.d * (p.t_state * p.speed);
            return cw.a + dir * u + dir.right() * lat * 0.0 + out;
        }
        if p.state == PedState::Waiting {
            u = if p.dir > 0.0 { -0.9 } else { cw.length + 0.9 };
            lat *= 1.2;
        }
        cw.a + dir * u + dir.right() * lat
    }

    /// A moving vehicle body within `r` of point `p` (pedestrians step around stopped ones).
    fn vehicle_at(&self, p: V2, r: f64) -> bool {
        self.vehicles.iter().any(|v| {
            if v.v < 0.8 {
                return false;
            }
            let l = &self.net.links[v.link()];
            if !(l.kind.in_box() || (l.kind == LinkKind::Inbound && l.length() - v.s < 8.0)) {
                return false;
            }
            let f = self.front(v);
            let b = self.point_behind(v, v.len());
            dist_point_seg(p, f, b) < v.spec().width / 2.0 + r
        })
    }

    fn vehicle_near_cw(&self, cw: usize) -> bool {
        for pc in self.net.ped_conflicts.iter().filter(|pc| pc.cw == cw) {
            for &j in &self.occ[pc.link] {
                let w = &self.vehicles[j];
                let d = pc.s - w.s;
                // Anything inside the intersection that can't stop, or is on the crosswalk.
                let on_cw = d > -w.len() - 2.0 && d < 2.6 && w.v > 0.5;
                let cant_stop = d > 0.0 && w.v * w.v / (2.0 * (d - 2.6).max(0.1)) > 2.5;
                let going = w.v > 0.5 || w.a > 0.2;
                if on_cw || cant_stop || (d > 0.0 && going) {
                    return true;
                }
            }
            for &p in &self.net.links[pc.link].pred {
                for &j in &self.occ[p] {
                    let w = &self.vehicles[j];
                    let d = self.net.links[p].length() - w.s + pc.s;
                    let cant_stop = w.v * w.v / (2.0 * (d - 2.6).max(0.1)) > 2.5;
                    let leaving = (w.v > 0.5 || w.a > 0.2) && (!w.commits.is_empty() || w.stop_done) && d < 35.0;
                    if w.route[w.ri..].contains(&pc.link) && (cant_stop || leaving) {
                        return true;
                    }
                }
            }
        }
        false
    }

    /// Live statistics on vehicles currently in the system.
    pub fn in_system(&self) -> (usize, usize) {
        (self.vehicles.len(), self.queues.iter().map(|q| q.len()).sum())
    }
}

fn route_dist_static(net: &Network, v: &Vehicle, k: usize, s_on: f64) -> f64 {
    let len = |i: usize| net.links[v.route[i]].length();
    if k == v.ri {
        s_on - v.s
    } else if k > v.ri {
        let mut d = len(v.ri) - v.s;
        for j in v.ri + 1..k {
            d += len(j);
        }
        d + s_on
    } else {
        let mut d = v.s;
        for j in k + 1..v.ri {
            d += len(j);
        }
        -(d + len(k) - s_on)
    }
}

pub fn dist_point_seg(p: V2, a: V2, b: V2) -> f64 {
    let ab = b - a;
    let l2 = ab.dot(ab);
    let t = if l2 < 1e-9 { 0.0 } else { ((p - a).dot(ab) / l2).clamp(0.0, 1.0) };
    p.dist(a + ab * t)
}

/// Separating-axis overlap of two oriented rectangles, each shrunk by `tol`.
pub fn obb_overlap(a: (V2, V2, f64, f64), b: (V2, V2, f64, f64), tol: f64) -> bool {
    let (ca, ua, la, wa) = a;
    let (cb, ub, lb, wb) = b;
    let (la, wa, lb, wb) = (la - tol, wa - tol, lb - tol, wb - tol);
    let d = cb - ca;
    for axis in [ua, ua.right(), ub, ub.right()] {
        let ra = la * ua.dot(axis).abs() + wa * ua.right().dot(axis).abs();
        let rb = lb * ub.dot(axis).abs() + wb * ub.right().dot(axis).abs();
        if d.dot(axis).abs() > ra + rb {
            return false;
        }
    }
    true
}
