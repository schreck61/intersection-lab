//! Stage-based signal state machine with MUTCD-style clearance: yellow change and
//! all-red intervals between stages, pedestrian WALK + flashing DON'T WALK, and
//! minimum green that grows to fit an active pedestrian interval.

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
        let mut any = false;
        for m in 0..a.len() {
            let dep = match (a[m], b[m]) {
                (Some(_), None) => true,
                (Some(MStat::G), Some(MStat::P)) => true,
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
        self.stage = self.next;
        self.phase = Phase::Green;
        self.t_phase = 0.0;
        self.green_elapsed = 0.0;
        self.cur_min = self.min_green;
        for d in self.departing.iter_mut() {
            *d = false;
        }
        self.history.push_back(StageSpan { stage: self.stage, start: t, end: t });
        while self.history.len() > 400 {
            self.history.pop_front();
        }
        self.serve_peds(net);
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
                self.serve_peds(net);
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
