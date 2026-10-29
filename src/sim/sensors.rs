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

use super::network::{LinkKind, Network};
use super::signal::PedSig;
use super::vtypes::VKind;
use super::{PedState, Sim};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

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
        }
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
                let mut occupied = false;
                let mut rich_in_zone = 0;
                for &j in &self.occ[lane] {
                    let v = &self.vehicles[j];
                    let d = len - v.s;
                    if v.s > len - 9.0 {
                        occupied = true;
                    }
                    if cfg.loops && v.prev_s < s_adv && v.s >= s_adv {
                        lo.last_pulse = t;
                        lo.count_io += 1;
                        lo.pulses.push_back((t, v.v.max(1.0)));
                    }
                    let cam = cfg.camera && d <= cfg.cam_range && v.cam_ok;
                    let cvd = cfg.cv && v.connected;
                    let rad = cfg.radar && d <= cfg.radar_range;
                    if v.kind == VKind::Emergency && d < 300.0 {
                        view.emergency.push((v.movement, d / v.v.max(3.0)));
                    }
                    if !(cam || cvd || rad) {
                        continue;
                    }
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
                    let intent_known = cam || cvd || moves.len() == 1;
                    let class_known = cam || cvd;
                    let queued = v.v < 2.0 && d < 150.0;
                    let wait = if cam || cvd { v.queue_since.map(|q| t - q).unwrap_or(0.0) } else { 0.0 };
                    let persons = if class_known { v.persons() } else if v.len() > 10.0 { 8.0 } else { 1.4 };
                    let base = Obs {
                        eta: if queued { 0.0 } else { d / v.v.max(3.0) },
                        dist: d,
                        speed: v.v,
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
                        push(v.movement, 1.0);
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
                        if v.ri > 0 && v.route[v.ri - 1] == lane && v.s - v.len() < 0.0 {
                            occupied = true;
                        }
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
                if cfg.loops && !rich {
                    let est = (lo.count_io - rich_in_zone).max(0) as f64;
                    let occ_wait = lo.stopbar_since.map(|s| t - s).unwrap_or(0.0);
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
