// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Stage-based signal state machine with MUTCD-style clearance: yellow change and
//! all-red intervals between stages, pedestrian WALK + flashing DON'T WALK, minimum
//! green that grows to fit an active pedestrian interval, and an optional leading
//! pedestrian interval (LPI).

use super::network::{MStat, Network};
use std::collections::VecDeque;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Green,
    Yellow,
    AllRed,
}

/// What a driver sees for one movement.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sig {
    G,
    Rt,
    P,
    Y,
    R,
}

impl Sig {
    pub fn is_green(self) -> bool {
        matches!(self, Sig::G | Sig::Rt | Sig::P)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PedSig {
    Walk,
    Fdw,
    Dw,
}

#[derive(Clone, Debug)]
pub struct CwSignal {
    pub state: PedSig,
    pub timer: f64,
    pub called: bool,
}

#[derive(Clone, Debug)]
pub struct StageSpan {
    pub stage: usize,
    pub start: f64,
    pub end: f64,
}

#[derive(Clone, Debug)]
pub struct SignalState {
    pub stage: usize,
    pub next: usize,
    pub phase: Phase,
    pub t_phase: f64,
    pub green_elapsed: f64,
    pub min_green: f64,
    pub cur_min: f64,
    pub yellow: f64,
    pub all_red: f64,
    pub walk_time: f64,
    pub ped_speed: f64,
    pub cw: Vec<CwSignal>,
    /// Increments at every yellow onset (identifies a yellow for driver decisions).
    pub change_id: u64,
    departing: Vec<bool>,
    pub history: VecDeque<StageSpan>,
    pub switches: u64,
    /// Leading pedestrian interval (s, 0 = off): when a stage starts with WALK, the
    /// vehicle movements that turn across those crosswalks stay red this long.
    pub lpi: f64,
    lpi_hold: Vec<bool>,
    /// Coordinated operation: WALK only starts with the stage (no late walk), so
    /// pedestrian intervals never push a stage past its force-off.
    pub no_late_walk: bool,
}

impl SignalState {
    pub fn new(net: &Network, t: f64) -> Self {
        let first = net.fixed_sequence.first().copied().unwrap_or(0);
        let mut s = SignalState {
            stage: first,
            next: first,
            phase: Phase::Green,
            t_phase: 0.0,
            green_elapsed: 0.0,
            min_green: 6.0,
            cur_min: 6.0,
            yellow: net.yellow,
            all_red: net.all_red,
            walk_time: 7.0,
            ped_speed: 1.1,
            cw: vec![CwSignal { state: PedSig::Dw, timer: 0.0, called: false }; net.crosswalks.len()],
            change_id: 0,
            departing: vec![false; net.movements.len()],
            history: VecDeque::new(),
            switches: 0,
            lpi: 0.0,
            lpi_hold: vec![false; net.movements.len()],
            no_late_walk: false,
        };
        s.history.push_back(StageSpan { stage: first, start: t, end: t });
        s
    }

    pub fn status(&self, net: &Network, m: usize) -> Sig {
        if net.stages.is_empty() {
            return Sig::G;
        }
        let cur = net.stages[self.stage].stat[m];
        let base = match cur {
            Some(MStat::G) => Sig::G,
            Some(MStat::Rt) => Sig::Rt,
            Some(MStat::P) => Sig::P,
            None => Sig::R,
        };
        match self.phase {
            Phase::Green if self.lpi_held(m) => Sig::R,
            Phase::Green => base,
            Phase::Yellow => {
                if self.departing[m] {
                    Sig::Y
                } else {
                    base
                }
            }
            Phase::AllRed => {
                if self.departing[m] {
                    Sig::R
                } else {
                    base
                }
            }
        }
    }

    /// Held red by the leading pedestrian interval (no turn on red either).
    pub fn lpi_held(&self, m: usize) -> bool {
        self.phase == Phase::Green && self.lpi_hold[m] && self.green_elapsed < self.lpi
    }

    pub fn can_switch(&self) -> bool {
        self.phase == Phase::Green && self.green_elapsed >= self.cur_min
    }

    /// Begin the change to stage `j` (ignored while already changing).
    pub fn request(&mut self, net: &Network, j: usize, t: f64) {
        if self.phase != Phase::Green || j == self.stage || j >= net.stages.len() {
            return;
        }
        let a = &net.stages[self.stage].stat;
        let b = &net.stages[j].stat;
        // With LPI, turns that continue into the next stage but cross a crosswalk about
        // to start WALK get a yellow/red arrow so pedestrians can lead.
        let lpi_cw: Vec<bool> =
            (0..self.cw.len()).map(|cw| self.lpi > 0.0 && self.cw[cw].called && net.stages[j].walk.contains(&cw)).collect();
        let mut any = false;
        for m in 0..a.len() {
            let dep = match (a[m], b[m]) {
                (Some(_), None) => true,
                (Some(MStat::G), Some(MStat::P)) => true,
                (Some(_), Some(_)) => net.movements[m].crosswalks.iter().any(|&cw| lpi_cw[cw]),
                _ => false,
            };
            self.departing[m] = dep;
            any |= dep;
        }
        // Pedestrian intervals end with the stage (min green already covered them).
        for c in self.cw.iter_mut() {
            c.state = PedSig::Dw;
        }
        self.next = j;
        self.switches += 1;
        self.change_id += 1;
        if any {
            self.phase = Phase::Yellow;
            self.t_phase = 0.0;
        } else {
            self.start_green(net, t);
        }
    }

    fn start_green(&mut self, net: &Network, t: f64) {
        if let Some(h) = self.history.back_mut() {
            h.end = t;
        }
        let prev_stage = self.stage;
        self.stage = self.next;
        self.phase = Phase::Green;
        self.t_phase = 0.0;
        self.green_elapsed = 0.0;
        self.cur_min = self.min_green;
        let was_departing = std::mem::replace(&mut self.departing, vec![false; net.movements.len()]);
        self.history.push_back(StageSpan { stage: self.stage, start: t, end: t });
        while self.history.len() > 400 {
            self.history.pop_front();
        }
        let prev: Vec<PedSig> = self.cw.iter().map(|c| c.state).collect();
        self.serve_peds(net);
        // LPI applies only to WALK that starts with the stage (never mid-green), and
        // only to movements that start moving now (or were stopped for it) and cross a
        // crosswalk that just started walking.
        for (m, hold) in self.lpi_hold.iter_mut().enumerate() {
            *hold = self.lpi > 0.0
                && net.stages[self.stage].stat[m].is_some()
                && (net.stages[prev_stage].stat[m].is_none() || was_departing[m])
                && net.movements[m].crosswalks.iter().any(|&cw| prev[cw] != PedSig::Walk && self.cw[cw].state == PedSig::Walk);
        }
        if self.lpi_hold.iter().any(|&h| h) {
            self.cur_min = self.cur_min.max(self.lpi + self.min_green);
        }
    }

    /// Start WALK on called crosswalks the current stage can serve.
    fn serve_peds(&mut self, net: &Network) {
        if self.phase != Phase::Green || net.stages.is_empty() {
            return;
        }
        let stage = &net.stages[self.stage];
        for &cw in &stage.walk {
            let c = &mut self.cw[cw];
            if c.called && c.state == PedSig::Dw {
                c.called = false;
                c.state = PedSig::Walk;
                c.timer = self.walk_time;
                let fdw = net.crosswalks[cw].length / self.ped_speed;
                self.cur_min = self.cur_min.max(self.green_elapsed + self.walk_time + fdw);
            }
        }
    }

    /// Stages that can serve a crosswalk.
    pub fn stage_serves_cw(net: &Network, stage: usize, cw: usize) -> bool {
        net.stages[stage].walk.contains(&cw)
    }

    pub fn step(&mut self, net: &Network, dt: f64, t: f64) {
        self.t_phase += dt;
        if let Some(h) = self.history.back_mut() {
            h.end = t;
        }
        match self.phase {
            Phase::Green => {
                self.green_elapsed += dt;
                if !self.no_late_walk {
                    self.serve_peds(net);
                }
            }
            Phase::Yellow => {
                if self.t_phase >= self.yellow {
                    self.phase = Phase::AllRed;
                    self.t_phase = 0.0;
                }
            }
            Phase::AllRed => {
                if self.t_phase >= self.all_red {
                    self.start_green(net, t);
                }
            }
        }
        for (i, c) in self.cw.iter_mut().enumerate() {
            match c.state {
                PedSig::Walk => {
                    c.timer -= dt;
                    if c.timer <= 0.0 {
                        c.state = PedSig::Fdw;
                        c.timer = net.crosswalks[i].length / self.ped_speed;
                    }
                }
                PedSig::Fdw => {
                    c.timer -= dt;
                    if c.timer <= 0.0 {
                        c.state = PedSig::Dw;
                    }
                }
                PedSig::Dw => {}
            }
        }
    }
}
