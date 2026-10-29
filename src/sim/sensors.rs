//! Sensor models. Controllers never see ground truth: they see a `SensorView`
//! assembled from whichever sensors are enabled.
//!
//! * Inductive loops: stop-bar presence zone + an advance point detector per lane.
//!   Gives presence, actuation pulses, spot speeds and an input-output queue estimate,
//!   but not a vehicle's intended turn in a shared lane.
//! * Intersection camera (video analytics): tracks vehicles within range, sees turn
//!   signals (intent), vehicle class and how long each vehicle has waited; also counts
//!   waiting pedestrians.
//! * Advance radar: long-range position/speed tracking upstream (arrival prediction),
//!   no intent.
//! * Connected vehicles (V2I): equipped vehicles broadcast position, speed, route and
//!   class from any distance.
//! * Emergency preemption receivers (optical/GPS).
//!
//! With `realistic` on, every sensor errs the way field equipment does: cameras lose
//! vehicles in bad weather, at night, at range and behind trucks and buses, and only
//! learn intent from drivers who signal; loops fail stuck-on/stuck-off and miss
//! motorcycles; radar and connected-vehicle reports are noisy (GPS error, latency).
//! The sensor layer runs the usual detector diagnostics (max presence / no activity)
//! and flags lanes it believes are faulty.

use super::network::{LinkKind, Network};
use super::signal::PedSig;
use super::vtypes::VKind;
use super::{PedState, Sim, Vehicle};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

/// Weather / lighting, which mostly affects video detection.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Conditions {
    Clear,
    Night,
    Rain,
    Fog,
    Snow,
}

impl Conditions {
    pub const ALL: [Conditions; 5] = [Conditions::Clear, Conditions::Night, Conditions::Rain, Conditions::Fog, Conditions::Snow];
    pub fn name(self) -> &'static str {
        match self {
            Conditions::Clear => "Clear day",
            Conditions::Night => "Night",
            Conditions::Rain => "Heavy rain",
            Conditions::Fog => "Fog",
            Conditions::Snow => "Snow",
        }
    }
    /// Camera (per-frame detection probability near the camera, usable range factor).
    pub fn camera(self) -> (f64, f64) {
        match self {
            Conditions::Clear => (0.97, 1.0),
            Conditions::Night => (0.90, 0.75),
            Conditions::Rain => (0.85, 0.7),
            Conditions::Fog => (0.65, 0.45),
            Conditions::Snow => (0.75, 0.55),
        }
    }
    /// Radar detection probability (radar is largely weather-proof).
    pub fn radar(self) -> f64 {
        match self {
            Conditions::Rain | Conditions::Snow => 0.93,
            _ => 0.97,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SensorConfig {
    pub loops: bool,
    pub adv_dist: f64,
    pub camera: bool,
    pub cam_range: f64,
    pub radar: bool,
    pub radar_range: f64,
    pub cv: bool,
    pub cv_rate: f64,
    /// Field-realistic sensor errors (otherwise sensors are ideal within range).
    pub realistic: bool,
    pub conditions: Conditions,
    /// Fraction of loop detector stations that are faulty (stuck on or off).
    pub loop_fault: f64,
    /// Fraction of drivers who use their turn signal (camera intent).
    pub signal_use: f64,
    /// Corridor: the upstream intersection shares the vehicles it has released toward
    /// us (experimental: helps little in this simple form, see README).
    pub upstream_link: bool,
}

impl Default for SensorConfig {
    fn default() -> Self {
        SensorConfig {
            loops: true,
            adv_dist: 55.0,
            camera: true,
            cam_range: 80.0,
            radar: true,
            radar_range: 250.0,
            cv: false,
            cv_rate: 0.3,
            realistic: true,
            conditions: Conditions::Clear,
            loop_fault: 0.0,
            signal_use: 0.8,
            upstream_link: false,
        }
    }
}

impl SensorConfig {
    /// Ideal sensors: everything within range is seen exactly.
    pub fn ideal(mut self) -> Self {
        self.realistic = false;
        self
    }
}

/// Deterministic hash to [0, 1).
fn h01(a: u64, b: u64, c: u64) -> f64 {
    let mut x = a.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ b.wrapping_mul(0xC2B2_AE3D_27D4_EB4F) ^ c.wrapping_mul(0x1656_67B1_9E37_79F9);
    x ^= x >> 33;
    x = x.wrapping_mul(0xFF51_AFD7_ED55_8CCD);
    x ^= x >> 33;
    x = x.wrapping_mul(0xC4CE_B9FE_1A85_EC53);
    x ^= x >> 33;
    (x >> 11) as f64 / (1u64 << 53) as f64
}

/// Smooth zero-mean noise with unit variance, correlated over `tau` seconds.
fn noise(a: u64, b: u64, t: f64, tau: f64) -> f64 {
    let k = (t / tau).floor();
    let f = t / tau - k;
    let u = |k: f64| (h01(a, b, k as i64 as u64) * 2.0 - 1.0) * 3f64.sqrt();
    let w = f * f * (3.0 - 2.0 * f);
    u(k) * (1.0 - w) + u(k + 1.0) * w
}

/// Loop station fault for a lane: 0 = working, 1 = stuck off, 2 = stuck on.
fn loop_fault(cfg: &SensorConfig, seed: u64, lane: usize) -> u8 {
    if !cfg.realistic || h01(seed, lane as u64, 0x100F) >= cfg.loop_fault {
        0
    } else if h01(seed, lane as u64, 0x200F) < 0.5 {
        1
    } else {
        2
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    Loop,
    Camera,
    Radar,
    Cv,
}

/// A vehicle as perceived by the sensors, attributed to a movement.
#[derive(Clone, Copy, Debug)]
pub struct Obs {
    pub eta: f64,
    pub dist: f64,
    pub speed: f64,
    pub weight: f64,
    pub persons: f64,
    pub wait: f64,
    pub queued: bool,
    pub kind: Option<VKind>,
}

#[derive(Clone, Debug, Default)]
pub struct MoveObs {
    /// Estimated stopped/slow vehicles (fractional when intent is unknown).
    pub queue: f64,
    /// Person-weighted queue.
    pub pqueue: f64,
    pub vehicles: Vec<Obs>,
    pub max_wait: f64,
    pub stopbar: bool,
    pub last_actuation: f64,
}

#[derive(Clone, Debug, Default)]
pub struct LaneObs {
    pub stopbar: bool,
    pub stopbar_since: Option<f64>,
    pub last_pulse: f64,
    pub count_io: i32,
    pulses: VecDeque<(f64, f64)>,
    /// Detector diagnostics believe this station is faulty (controllers should place
    /// its movements on recall).
    pub fault: bool,
    /// Last time the station reported anything.
    pub last_activity: f64,
    /// Last time a movement using this lane had green (for failed-detector fallback).
    pub last_green: f64,
}

#[derive(Clone, Debug)]
pub struct SensorView {
    pub t: f64,
    pub lanes: Vec<LaneObs>,
    pub moves: Vec<MoveObs>,
    pub out_queue: Vec<f64>,
    pub ped_waiting: Vec<u32>,
    pub ped_max_wait: Vec<f64>,
    pub ped_call: Vec<bool>,
    pub emergency: Vec<(usize, f64)>,
    /// (vehicle id, best source) for visualisation.
    pub detections: Vec<(u64, Source)>,
}

impl SensorView {
    pub fn new(net: &Network) -> Self {
        SensorView {
            t: 0.0,
            lanes: vec![LaneObs::default(); net.links.len()],
            moves: vec![MoveObs::default(); net.movements.len()],
            out_queue: vec![0.0; net.legs.len()],
            ped_waiting: vec![0; net.crosswalks.len()],
            ped_max_wait: vec![0.0; net.crosswalks.len()],
            ped_call: vec![false; net.crosswalks.len()],
            emergency: vec![],
            detections: vec![],
        }
    }

    pub fn movement_demand(&self, m: usize) -> f64 {
        self.moves[m].vehicles.iter().map(|o| o.weight).sum()
    }
}

impl Sim {
    /// Does the intersection camera detect vehicle `v` (distance `d` to the stop line)?
    fn camera_sees(&self, cfg: &SensorConfig, v: &Vehicle, d: f64, lane: usize, t: f64) -> bool {
        if !cfg.realistic {
            return d <= cfg.cam_range && v.cam_ok;
        }
        let (p, range_f) = cfg.conditions.camera();
        let range = cfg.cam_range * range_f;
        if d > range {
            return false;
        }
        // Tall vehicles closer to the camera hide those behind them.
        let occluded = d > 20.0
            && self.occ[lane].iter().any(|&k| {
                let w = &self.vehicles[k];
                w.id != v.id && w.len() > 10.0 && w.s > v.s && w.s - w.len() - v.s < 25.0
            });
        // Detection probability falls with distance; tracking flickers over ~1.5 s.
        let p = if occluded { 0.2 } else { p * (1.0 - 0.25 * d / range) };
        h01(v.id, (t / 1.5) as u64, 0x20) < p
    }

    pub(crate) fn sense(&mut self) {
        let t = self.t;
        let cfg = self.sensors.clone();
        let net = &self.net;
        let mut view = std::mem::replace(&mut self.view, SensorView::new(net));
        view.t = t;
        for m in view.moves.iter_mut() {
            m.queue = 0.0;
            m.pqueue = 0.0;
            m.vehicles.clear();
            m.max_wait = 0.0;
            m.stopbar = false;
        }
        view.detections.clear();
        view.emergency.clear();
        let rich = cfg.camera || cfg.radar || cfg.cv;

        // Historical turning shares per movement for unknown-intent attribution.
        let flows: Vec<f64> = (0..net.movements.len()).map(|m| self.demand.movement_flow(net, m).max(1.0)).collect();

        for leg in &net.legs {
            for &lane in &leg.in_lanes {
                let link = &net.links[lane];
                let len = link.length();
                let lo = &mut view.lanes[lane];
                let moves = &link.lane_moves;
                let total_flow: f64 = moves.iter().map(|&m| flows[m]).sum();
                let s_adv = len - cfg.adv_dist;
                let fault = loop_fault(&cfg, self.seed, lane);
                let mut occupied = false;
                let mut rich_in_zone = 0;
                for &j in &self.occ[lane] {
                    let v = &self.vehicles[j];
                    let d = len - v.s;
                    // Loops miss some vehicles (motorcycles especially) and faulty
                    // stations report nothing.
                    let loop_sees = fault == 0
                        && (!cfg.realistic || h01(v.id, lane as u64, 0x10) >= if v.kind == VKind::Motorcycle { 0.35 } else { 0.01 });
                    if v.s > len - 9.0 && loop_sees {
                        occupied = true;
                    }
                    if cfg.loops && loop_sees && v.prev_s < s_adv && v.s >= s_adv {
                        lo.last_pulse = t;
                        lo.count_io += 1;
                        lo.pulses.push_back((t, v.v.max(1.0)));
                    }
                    let cam = cfg.camera && self.camera_sees(&cfg, v, d, lane, t);
                    let cvd = cfg.cv && v.connected;
                    let rad = cfg.radar && d <= cfg.radar_range && (!cfg.realistic || h01(v.id, (t / 0.5) as u64, 0x30) < cfg.conditions.radar());
                    if v.kind == VKind::Emergency && d < 300.0 {
                        view.emergency.push((v.movement, d / v.v.max(3.0)));
                    }
                    if !(cam || cvd || rad) {
                        continue;
                    }
                    // Measurement error of the best source: radar range/speed noise, or
                    // connected-vehicle GPS error (messages are timestamped, so latency is
                    // compensated by extrapolating with the reported speed).
                    let (d, speed) = if !cfg.realistic || cam {
                        (d, v.v)
                    } else if rad {
                        (d + 1.5 * noise(v.id, 0x50, t, 1.0), (v.v + 0.5 * noise(v.id, 0x51, t, 1.0)).max(0.0))
                    } else {
                        (d + 3.0 * noise(v.id, 0x40, t, 2.0), (v.v + 0.3 * noise(v.id, 0x41, t, 2.0)).max(0.0))
                    };
                    let d = d.max(0.0);
                    if d <= cfg.adv_dist {
                        rich_in_zone += 1;
                    }
                    let src = if cam {
                        super::sensors::Source::Camera
                    } else if cvd {
                        Source::Cv
                    } else {
                        Source::Radar
                    };
                    view.detections.push((v.id, src));
                    // A camera reads intent from turn signals; a driver who doesn't
                    // signal looks like through traffic.
                    let signals = !cfg.realistic || v.turn == super::network::Turn::T || h01(v.id, 0x60, 0) < cfg.signal_use;
                    let cam_move = if signals {
                        Some(v.movement)
                    } else {
                        moves.iter().copied().find(|&m| net.movements[m].turn == super::network::Turn::T)
                    };
                    let intent_known = cvd || (cam && cam_move.is_some()) || moves.len() == 1;
                    let seen_move = if cvd || moves.len() == 1 { v.movement } else { cam_move.unwrap_or(v.movement) };
                    let class_known = cam || cvd;
                    let queued = speed < 2.0 && d < 150.0;
                    let wait = if cam || cvd { v.queue_since.map(|q| t - q).unwrap_or(0.0) } else { 0.0 };
                    let persons = if class_known { v.persons() } else if v.len() > 10.0 { 8.0 } else { 1.4 };
                    let base = Obs {
                        eta: if queued { 0.0 } else { d / speed.max(3.0) },
                        dist: d,
                        speed,
                        weight: 1.0,
                        persons,
                        wait,
                        queued,
                        kind: if class_known { Some(v.kind) } else { None },
                    };
                    let mut push = |m: usize, w: f64| {
                        let mo = &mut view.moves[m];
                        let mut o = base;
                        o.weight = w;
                        if queued {
                            mo.queue += w;
                            mo.pqueue += w * persons;
                            mo.max_wait = mo.max_wait.max(wait);
                        }
                        mo.vehicles.push(o);
                    };
                    if intent_known {
                        push(seen_move, 1.0);
                    } else {
                        for &m in moves {
                            push(m, flows[m] / total_flow);
                        }
                    }
                }
                // Vehicles crossing the stop line leave the loop input-output count.
                for &succ in &link.succ {
                    for &j in &self.occ[succ] {
                        let v = &self.vehicles[j];
                        if v.prev_s < 0.0 && v.s >= 0.0 {
                            lo.count_io = (lo.count_io - 1).max(0);
                            lo.pulses.pop_front();
                        }
                        if v.ri > 0 && v.route[v.ri - 1] == lane && v.s - v.len() < 0.0 && fault == 0 {
                            occupied = true;
                        }
                    }
                }
                if fault == 2 {
                    occupied = true;
                }
                if cfg.loops && cfg.realistic {
                    if occupied || lo.last_pulse >= t - 0.05 {
                        lo.last_activity = t;
                    }
                    // Diagnostics: max presence (stuck on) and no activity while the rest
                    // of the approach is busy (stuck off).
                    // The no-activity timer scales with expected volume: ten expected
                    // headways, between 2 and 15 minutes.
                    let lane_flow: f64 = moves.iter().map(|&m| flows[m] / net.movements[m].routes.len().max(1) as f64).sum();
                    let silent_limit = (10.0 * 3600.0 / lane_flow.max(1.0)).clamp(120.0, 900.0);
                    let stuck_on = lo.stopbar_since.is_some_and(|s0| t - s0 > 180.0);
                    lo.fault = stuck_on || t - lo.last_activity > silent_limit;
                    // Input-output counts drift when vehicles are missed: reset when the
                    // stop bar has been clear and nothing has arrived for a while.
                    if !occupied && t - lo.last_pulse > 15.0 {
                        lo.count_io = 0;
                        lo.pulses.clear();
                    }
                }
                if cfg.loops {
                    if occupied {
                        if lo.stopbar_since.is_none() {
                            lo.stopbar_since = Some(t);
                        }
                        lo.last_pulse = t;
                    } else {
                        lo.stopbar_since = None;
                    }
                    lo.stopbar = occupied;
                } else {
                    lo.stopbar = false;
                    lo.stopbar_since = None;
                }
                while lo.pulses.len() > lo.count_io.max(0) as usize {
                    lo.pulses.pop_front();
                }
                // Loop-only estimates when no richer sensor covers the lane.
                if self.signalized() && moves.iter().any(|&m| self.signal.status(net, m).is_green()) {
                    lo.last_green = t;
                }
                if cfg.loops && !rich {
                    let mut est = (lo.count_io - rich_in_zone).max(0) as f64;
                    let mut occ_wait = lo.stopbar_since.map(|s| t - s).unwrap_or(0.0);
                    if lo.fault {
                        // Failed detector: fall back to historical volume, i.e. the
                        // vehicles expected to have arrived since the lane last had green.
                        let lane_flow: f64 = moves.iter().map(|&m| flows[m] / net.movements[m].routes.len().max(1) as f64).sum();
                        let since = t - lo.last_green;
                        est = (lane_flow / 3600.0 * since).clamp(1.0, 25.0);
                        occ_wait = since;
                    }
                    let pulses: Vec<(f64, f64)> = lo.pulses.iter().copied().collect();
                    for &m in moves {
                        let share = flows[m] / total_flow;
                        let mo = &mut view.moves[m];
                        let mut moving = 0.0;
                        for &(tp, vp) in &pulses {
                            let eta = cfg.adv_dist / vp - (t - tp);
                            if eta > 0.5 {
                                moving += 1.0;
                                mo.vehicles.push(Obs {
                                    eta,
                                    dist: vp * eta,
                                    speed: vp,
                                    weight: share,
                                    persons: 1.4,
                                    wait: 0.0,
                                    queued: false,
                                    kind: None,
                                });
                            }
                        }
                        let q = (est - moving).max(if occupied { 1.0 } else { 0.0 });
                        mo.queue += q * share;
                        mo.pqueue += q * share * 1.4;
                        for _ in 0..(q.round() as usize) {
                            mo.vehicles.push(Obs {
                                eta: 0.0,
                                dist: 0.0,
                                speed: 0.0,
                                weight: share,
                                persons: 1.4,
                                wait: occ_wait,
                                queued: true,
                                kind: None,
                            });
                        }
                        mo.max_wait = mo.max_wait.max(occ_wait);
                    }
                }
                let (sb, lp) = (lo.stopbar, lo.last_pulse);
                for &m in moves {
                    let mo = &mut view.moves[m];
                    mo.stopbar |= sb;
                    mo.last_actuation = mo.last_actuation.max(lp);
                }
            }
        }
        // Corridor: vehicles the upstream intersection is sending, beyond what our own
        // sensors cover (intent unknown: attributed by turning shares).
        if cfg.upstream_link {
            let own = [(cfg.radar, cfg.radar_range), (cfg.camera, cfg.cam_range), (cfg.loops, cfg.adv_dist)]
                .iter()
                .filter(|x| x.0)
                .map(|x| x.1)
                .fold(0.0, f64::max);
            for u in &self.upstream {
                if u.dist <= own {
                    continue;
                }
                let moves: Vec<usize> = (0..net.movements.len()).filter(|&m| net.movements[m].leg == u.leg).collect();
                let total: f64 = moves.iter().map(|&m| flows[m]).sum();
                for &m in &moves {
                    view.moves[m].vehicles.push(Obs {
                        eta: u.dist / u.speed.max(3.0),
                        dist: u.dist,
                        speed: u.speed,
                        weight: flows[m] / total.max(1e-9),
                        persons: u.kind.spec().occupancy,
                        wait: 0.0,
                        queued: false,
                        kind: Some(u.kind),
                    });
                }
            }
        }
        // "No activity" only indicates a fault while the rest of the approach is busy.
        for leg in &net.legs {
            let busy = leg.in_lanes.iter().filter(|&&l| t - view.lanes[l].last_activity < 120.0).count();
            if busy == 0 {
                for &l in &leg.in_lanes {
                    let lo = &mut view.lanes[l];
                    if !lo.stopbar_since.is_some_and(|s0| t - s0 > 180.0) {
                        lo.fault = false;
                        lo.last_activity = lo.last_activity.max(t - 300.0);
                    }
                }
            }
        }
        // Emergency vehicles already inside the intersection keep their preemption.
        for v in &self.vehicles {
            if v.kind == VKind::Emergency && net.links[v.link()].kind.in_box() {
                view.emergency.push((v.movement, 0.0));
            }
        }
        for (li, leg) in net.legs.iter().enumerate() {
            let mut q = 0.0;
            if cfg.camera {
                for &lane in &leg.out_lanes {
                    for &j in &self.occ[lane] {
                        let v = &self.vehicles[j];
                        if v.s < 60.0 && v.v < 3.0 {
                            q += 1.0;
                        }
                    }
                }
            }
            view.out_queue[li] = q;
        }
        for c in view.ped_waiting.iter_mut() {
            *c = 0;
        }
        for c in view.ped_max_wait.iter_mut() {
            *c = 0.0;
        }
        for p in &self.peds {
            if p.state == PedState::Waiting {
                view.ped_waiting[p.cw] += 1;
                view.ped_max_wait[p.cw] = view.ped_max_wait[p.cw].max(t - p.arrive_t);
            }
        }
        for (i, c) in self.signal.cw.iter().enumerate() {
            view.ped_call[i] = c.called || (view.ped_waiting[i] > 0 && c.state == PedSig::Dw);
        }
        let _ = LinkKind::Inbound;
        self.view = view;
    }
}
