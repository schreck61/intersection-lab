// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! A world of one or more intersections that are stepped together and drawn side by
//! side in world coordinates.
//!
//! A corridor chains intersections west to east along the main road, one block
//! (`SPACING`) apart. Vehicles leaving one intersection eastbound or westbound enter the
//! next one's approach; when that approach is full they wait at the end of the exit
//! road, so queues spill back through upstream intersections. Each intersection keeps
//! its own controller and sensors; with `upstream_link` on, it is also told about the
//! vehicles its upstream neighbour is sending (as adaptive corridor systems share
//! their projected outflows).

use super::demand::{DemandConfig, Pending};
use super::geom::{V2, v2};
use super::network::{Compass, IN_LEN, LayoutKind, MStat, Network, OUT_LEN, Turn};
use super::rng::Rng;
use super::{ControlMode, Exit, Sim, UpstreamObs};
use crate::control::CtrlKind;

/// Distance between neighbouring intersections (exit road + approach).
pub const SPACING: f64 = IN_LEN + OUT_LEN;

/// A completed trip through the corridor.
#[derive(Clone, Copy, Debug, Default)]
pub struct Journey {
    pub start: f64,
    pub end: f64,
    pub hops: u32,
    pub stops: u32,
    pub delay: f64,
    pub persons: f64,
    /// Travelled the whole main road: +1 eastbound, -1 westbound, 0 otherwise.
    pub through: i8,
}

/// End-to-end experience on the main road.
#[derive(Clone, Copy, Debug, Default)]
pub struct CorridorSummary {
    pub through_n: usize,
    pub through_travel: f64,
    pub through_stops: f64,
    pub through_delay: f64,
    /// Person-weighted delay over every trip that finished (all origins).
    pub trip_person_delay: f64,
    pub trips: usize,
}

pub struct World {
    pub sims: Vec<Sim>,
    /// Centre of each intersection in world coordinates.
    pub offsets: Vec<V2>,
    pub journeys: Vec<Journey>,
    /// Fixed-time intersections run a coordinated (green wave) plan.
    pub coordinated: bool,
    rng: Rng,
}

impl World {
    pub fn single(sim: Sim) -> World {
        World { sims: vec![sim], offsets: vec![v2(0.0, 0.0)], journeys: vec![], coordinated: false, rng: Rng::new(1) }
    }

    /// `n` intersections of `kind` along the east-west main road. Interior approaches
    /// of the main road are fed by the neighbouring intersection instead of random
    /// arrivals; each intersection draws its other arrivals from its own stream.
    pub fn corridor(n: usize, kind: LayoutKind, mode: ControlMode, ctrl: CtrlKind, demand: &DemandConfig, seed: u64) -> World {
        let n = n.max(1);
        let mut sims = Vec::with_capacity(n);
        for i in 0..n {
            let net = Network::build(kind);
            let mut d = demand.clone();
            d.fed = [false; 4];
            if n > 1 {
                d.fed[super::demand::compass_index(Compass::W)] = i > 0;
                d.fed[super::demand::compass_index(Compass::E)] = i + 1 < n;
            }
            let mut sim = Sim::with_demand(net, mode, ctrl, d, seed + i as u64 * 7919);
            sim.collect_exits = n > 1;
            sims.push(sim);
        }
        World {
            offsets: (0..n).map(|i| v2((i as f64 - (n - 1) as f64 / 2.0) * SPACING, 0.0)).collect(),
            sims,
            journeys: vec![],
            coordinated: false,
            rng: Rng::new(seed ^ 0xC0FF_EE00),
        }
    }

    pub fn is_corridor(&self) -> bool {
        self.sims.len() > 1
    }

    pub fn t(&self) -> f64 {
        self.sims[0].t
    }

    pub fn dt(&self) -> f64 {
        self.sims[0].dt
    }

    pub fn run_for(&mut self, seconds: f64) {
        let n = (seconds / self.dt()).round() as usize;
        for _ in 0..n {
            self.step();
        }
    }

    pub fn step(&mut self) {
        if self.is_corridor() {
            self.link_neighbours();
        }
        for s in &mut self.sims {
            s.step();
        }
        if self.is_corridor() {
            self.hand_over();
        }
    }

    /// Main-road leg (E or W) of intersection `i` and the neighbour it leads to.
    fn neighbour(&self, i: usize, c: Compass) -> Option<usize> {
        match c {
            Compass::E if i + 1 < self.sims.len() => Some(i + 1),
            Compass::W if i > 0 => Some(i - 1),
            _ => None,
        }
    }

    fn opposite(c: Compass) -> Compass {
        match c {
            Compass::E => Compass::W,
            Compass::W => Compass::E,
            Compass::N => Compass::S,
            Compass::S => Compass::N,
        }
    }

    /// Before a step: tell each intersection how much room the next one has for its
    /// exiting traffic, and what the previous one is about to send.
    fn link_neighbours(&mut self) {
        let n = self.sims.len();
        let mut rooms: Vec<Vec<Option<(f64, f64)>>> = self.sims.iter().map(|s| vec![None; s.net.legs.len()]).collect();
        let mut ups: Vec<Vec<UpstreamObs>> = vec![vec![]; n];
        for i in 0..n {
            for c in [Compass::E, Compass::W] {
                let (Some(j), Some(out_leg)) = (self.neighbour(i, c), self.sims[i].net.leg_index(c)) else { continue };
                let down = &self.sims[j];
                let Some(in_leg) = down.net.leg_index(Self::opposite(c)) else { continue };
                // Space on the downstream approach's entry lanes (bays excluded).
                let room = if !down.queues[in_leg].is_empty() {
                    (0.0, 0.0)
                } else {
                    down.net.legs[in_leg]
                        .in_lanes
                        .iter()
                        .filter(|&&l| down.net.links[l].bay_start <= 0.0)
                        .map(|&l| down.entry_room(l))
                        .fold((0.0, 0.0), |a, b| if b.0 > a.0 { (b.0.min(1e3), b.1) } else { a })
                };
                rooms[i][out_leg] = Some(room);
                // Outflow heading our way: vehicles the upstream signal has released (past
                // its stop line), whose arrival is predictable at cruising speed.
                let approach = down.net.links[down.net.legs[in_leg].in_lanes[0]].length();
                let up = &self.sims[i];
                for v in &up.vehicles {
                    let last = *v.route.last().unwrap();
                    if v.ri == 0 || up.net.links[last].leg != out_leg || up.net.links[last].kind != super::network::LinkKind::Outbound {
                        continue;
                    }
                    let mut d = up.net.links[v.link()].length() - v.s;
                    for &l in &v.route[v.ri + 1..] {
                        d += up.net.links[l].length();
                    }
                    ups[j].push(UpstreamObs { leg: in_leg, dist: d + approach, speed: v.v.max(0.85 * v.v0), kind: v.kind });
                }
            }
        }
        for (i, s) in self.sims.iter_mut().enumerate() {
            s.exit_room = std::mem::take(&mut rooms[i]);
            s.upstream = std::mem::take(&mut ups[i]);
        }
    }

    /// After a step: move vehicles that left an intersection toward a neighbour onto
    /// that neighbour's approach; record trips that left the corridor.
    fn hand_over(&mut self) {
        let n = self.sims.len();
        let t = self.t();
        for i in 0..n {
            let exits: Vec<Exit> = std::mem::take(&mut self.sims[i].exits);
            for mut e in exits {
                let c = self.sims[i].net.legs[e.to_leg].compass;
                if e.trip.hops == 1 {
                    let from = self.sims[i].net.legs[e.from_leg].compass;
                    e.trip.from_end = match (i, from) {
                        (0, Compass::W) => 1,
                        (k, Compass::E) if k + 1 == n => -1,
                        _ => 0,
                    };
                }
                match self.neighbour(i, c) {
                    Some(j) => {
                        let down = &self.sims[j];
                        let Some(in_leg) = down.net.leg_index(Self::opposite(c)) else { continue };
                        let shares = down.demand.valid_shares(&down.net, in_leg);
                        let turn = Turn::ALL[self.rng.weighted(&shares)];
                        let Some(m) = down.net.movement_for(in_leg, turn) else { continue };
                        let p = Pending {
                            kind: e.kind,
                            movement: m,
                            arrival_t: t,
                            speed_factor: e.speed_factor,
                            cv_u: if e.connected { 0.0 } else { 1.0 },
                            cam_ok: e.cam_ok,
                            manual: e.manual,
                            trip: e.trip,
                        };
                        let down = &mut self.sims[j];
                        down.queues[in_leg].push_back(p);
                    }
                    None => {
                        let through = match (e.trip.from_end, c) {
                            (1, Compass::E) if i + 1 == n => 1,
                            (-1, Compass::W) if i == 0 => -1,
                            _ => 0,
                        };
                        self.journeys.push(Journey {
                            start: e.trip.start,
                            end: t,
                            hops: e.trip.hops,
                            stops: e.trip.stops,
                            delay: e.trip.delay,
                            persons: e.kind.spec().occupancy,
                            through,
                        });
                    }
                }
            }
        }
    }

    /// Corridor experience of trips that finished after `from_t`.
    pub fn corridor_summary(&self, from_t: f64) -> CorridorSummary {
        let js: Vec<&Journey> = self.journeys.iter().filter(|j| j.end >= from_t).collect();
        let mut s = CorridorSummary { trips: js.len(), ..Default::default() };
        let thr: Vec<&&Journey> = js.iter().filter(|j| j.through != 0).collect();
        s.through_n = thr.len();
        if !thr.is_empty() {
            let k = thr.len() as f64;
            s.through_travel = thr.iter().map(|j| j.end - j.start).sum::<f64>() / k;
            s.through_stops = thr.iter().map(|j| j.stops as f64).sum::<f64>() / k;
            s.through_delay = thr.iter().map(|j| j.delay).sum::<f64>() / k;
        }
        let p: f64 = js.iter().map(|j| j.persons).sum();
        if p > 0.0 {
            s.trip_person_delay = js.iter().map(|j| j.delay * j.persons).sum::<f64>() / p;
        }
        s
    }

    /// Mean "person delay incl. unserved" over the intersections (each intersection's
    /// own measure, so side-street starvation still counts).
    pub fn mean_person_delay_all(&self, from_t: f64) -> f64 {
        let k = self.sims.len() as f64;
        self.sims.iter().map(|s| s.stats.summary(from_t, s.t, &s.vehicles, &s.queues).person_delay_all).sum::<f64>() / k
    }

    /// Turn the fixed-time green wave on or off. A common cycle (the longest Webster
    /// cycle), the main-road through stage first in each cycle, and offsets chosen
    /// greedily to maximise the two-way green band at progression speed.
    pub fn set_coordination(&mut self, on: bool) {
        self.coordinated = on;
        if !on || !self.is_corridor() {
            for s in &mut self.sims {
                s.coordination = None;
                if s.ctrl_kind == CtrlKind::Fixed && s.signalized() {
                    s.set_controller(CtrlKind::Fixed);
                }
            }
            return;
        }
        let mut cycle: f64 = 0.0;
        let mut sync = vec![None; self.sims.len()];
        for (i, s) in self.sims.iter_mut().enumerate() {
            if !s.signalized() {
                continue;
            }
            s.coordination = None;
            s.set_controller(CtrlKind::Fixed);
            if let Some(f) = s.controller.as_fixed() {
                cycle = cycle.max(f.cycle);
            }
            let net = &s.net;
            let mt = |c: Compass| net.leg_index(c).and_then(|l| net.movement_for(l, Turn::T));
            let (e, w) = (mt(Compass::E), mt(Compass::W));
            sync[i] = (0..net.stages.len()).find(|&st| {
                [e, w].iter().flatten().all(|&m| net.stages[st].stat[m] == Some(MStat::G))
            });
        }
        if cycle <= 0.0 {
            return;
        }
        // Main-road green of each intersection at the common cycle.
        let mut green = vec![0.0; self.sims.len()];
        for (i, s) in self.sims.iter_mut().enumerate() {
            if let (Some(st), Some(f)) = (sync[i], s.controller.as_fixed()) {
                f.coordinate_plan(cycle, 0.0, st);
                green[i] = f.greens[0];
            }
        }
        let speed = self.sims[0].net.legs.first().map(|l| l.speed).unwrap_or(13.9) * 0.92;
        let tt = SPACING / speed;
        let overlap = |a: f64, ga: f64, b: f64, gb: f64| -> f64 {
            (-1..=1)
                .map(|k| {
                    let b = b + k as f64 * cycle;
                    ((a + ga).min(b + gb) - a.max(b)).max(0.0)
                })
                .sum::<f64>()
        };
        let mut offsets = vec![0.0; self.sims.len()];
        for i in 1..self.sims.len() {
            let prev = offsets[i - 1];
            let mut best = (f64::NEG_INFINITY, 0.0);
            let mut o = 0.0;
            while o < cycle {
                // Eastbound: released at i-1, arriving tt later at i; westbound: the reverse.
                let eb = overlap((prev + tt).rem_euclid(cycle), green[i - 1], o, green[i]);
                let wb = overlap((o + tt).rem_euclid(cycle), green[i], prev, green[i - 1]);
                if eb + wb > best.0 + 1e-9 {
                    best = (eb + wb, o);
                }
                o += 1.0;
            }
            offsets[i] = best.1;
        }
        for (i, s) in self.sims.iter_mut().enumerate() {
            if let Some(st) = sync[i] {
                s.coordination = Some((cycle, offsets[i], st));
                if s.ctrl_kind == CtrlKind::Fixed {
                    s.set_controller(CtrlKind::Fixed);
                }
            }
        }
    }

    /// Copy the user-adjustable settings (sensors, operations, demand) of intersection
    /// `from` to every other intersection.
    pub fn sync_settings(&mut self, from: usize) {
        let src = &self.sims[from];
        let (sensors, demand) = (src.sensors.clone(), src.demand.clone());
        let (rtor, ntor, lpi, left, pre, guard) =
            (src.rtor, src.ntor_peds, src.signal.lpi, src.left_mode, src.preemption, src.max_wait_guard);
        for (i, s) in self.sims.iter_mut().enumerate() {
            if i == from {
                continue;
            }
            s.sensors = sensors.clone();
            let fed = s.demand.fed;
            s.demand = demand.clone();
            s.demand.fed = fed;
            s.rtor = rtor;
            s.ntor_peds = ntor;
            s.signal.lpi = lpi;
            s.left_mode = left;
            s.preemption = pre;
            s.max_wait_guard = guard;
        }
    }

    /// Copy settings from another world (the comparison twin follows the main world).
    pub fn follow(&mut self, other: &World) {
        for (s, o) in self.sims.iter_mut().zip(&other.sims) {
            s.sensors = o.sensors.clone();
            s.demand = o.demand.clone();
            s.rtor = o.rtor;
            s.ntor_peds = o.ntor_peds;
            s.signal.lpi = o.signal.lpi;
            s.left_mode = o.left_mode;
            s.preemption = o.preemption;
            s.max_wait_guard = o.max_wait_guard;
        }
    }
}
