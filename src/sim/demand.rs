// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Stochastic demand: per-approach Poisson arrivals modulated by an arrival pattern
//! (random, platooned from an upstream signal, rush-hour peak, random surges),
//! turning shares, vehicle mix and pedestrian arrivals.

use super::network::{Compass, Network, Turn};
use super::rng::Rng;
use super::vtypes::{ALL_KINDS, VKind};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Pattern {
    Random,
    Platoons,
    RushHour,
    Surges,
}

impl Pattern {
    pub const ALL: [Pattern; 4] = [Pattern::Random, Pattern::Platoons, Pattern::RushHour, Pattern::Surges];
    pub fn name(self) -> &'static str {
        match self {
            Pattern::Random => "Random (Poisson)",
            Pattern::Platoons => "Platoons (upstream signal)",
            Pattern::RushHour => "Rush-hour peak",
            Pattern::Surges => "Random surges",
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DemandConfig {
    /// Vehicles per hour per approach, indexed by Compass (N, E, S, W).
    pub rate: [f64; 4],
    /// Turning shares (L, T, R) per approach.
    pub turn: [[f64; 3]; 4],
    /// Vehicle mix weights indexed like `ALL_KINDS`.
    pub mix: [f64; 6],
    /// Pedestrians per hour across all crosswalks.
    pub ped_rate: f64,
    pub pattern: Pattern,
    /// Global demand multiplier.
    pub scale: f64,
    /// Approaches fed by an upstream intersection (corridor): no random arrivals are
    /// generated there, but `rate` still describes their expected flow.
    #[serde(default)]
    pub fed: [bool; 4],
}

impl DemandConfig {
    pub fn default_for(net: &Network) -> Self {
        let mut d = DemandConfig {
            rate: [0.0; 4],
            turn: [[0.15, 0.7, 0.15]; 4],
            mix: [0.62, 0.24, 0.05, 0.03, 0.04, 0.0],
            ped_rate: 60.0,
            pattern: Pattern::Random,
            scale: 1.0,
            fed: [false; 4],
        };
        use super::network::LayoutKind::*;
        let (major, minor) = match net.kind {
            Arterial4 => (700.0, 550.0),
            MajorMinor4 => (750.0, 200.0),
            Town4 => (300.0, 260.0),
            Tee => (650.0, 260.0),
            Roundabout => (420.0, 350.0),
        };
        for leg in &net.legs {
            let ci = compass_index(leg.compass);
            let horizontal = matches!(leg.compass, Compass::E | Compass::W);
            d.rate[ci] = if horizontal { major } else { minor };
        }
        if net.kind == Tee {
            d.turn[compass_index(Compass::E)] = [0.25, 0.75, 0.0];
            d.turn[compass_index(Compass::W)] = [0.0, 0.75, 0.25];
            d.turn[compass_index(Compass::S)] = [0.5, 0.0, 0.5];
        }
        if net.kind == Roundabout {
            d.ped_rate = 0.0;
        }
        d
    }

    /// Hourly flow of one movement (used by fixed-time optimisation and as
    /// "historical" turning counts by controllers).
    pub fn movement_flow(&self, net: &Network, m: usize) -> f64 {
        let mv = &net.movements[m];
        let leg = &net.legs[mv.leg];
        let ci = compass_index(leg.compass);
        let shares = self.valid_shares(net, mv.leg);
        self.rate[ci] * self.scale * shares[mv.turn.index()]
    }

    pub fn valid_shares(&self, net: &Network, leg: usize) -> [f64; 3] {
        let ci = compass_index(net.legs[leg].compass);
        let turns = net.legs[leg].turns();
        let mut s = [0.0; 3];
        for t in Turn::ALL {
            if turns.contains(&t) && net.movement_for(leg, t).is_some() {
                s[t.index()] = self.turn[ci][t.index()].max(0.0);
            }
        }
        let total: f64 = s.iter().sum();
        if total > 0.0 {
            for x in s.iter_mut() {
                *x /= total;
            }
        }
        s
    }
}

pub fn compass_index(c: Compass) -> usize {
    match c {
        Compass::N => 0,
        Compass::E => 1,
        Compass::S => 2,
        Compass::W => 3,
    }
}

/// A vehicle's journey through a corridor (one or more intersections).
#[derive(Clone, Copy, Debug, Default)]
pub struct Trip {
    /// Time the vehicle arrived at the corridor.
    pub start: f64,
    /// Intersections passed so far.
    pub hops: u32,
    pub stops: u32,
    /// Control delay accumulated at previous intersections.
    pub delay: f64,
    /// Entered the corridor at its west (+1) or east (-1) end on the main road.
    pub from_end: i8,
}

/// A generated arrival waiting to enter the network at the upstream boundary.
#[derive(Clone, Debug)]
pub struct Pending {
    pub kind: VKind,
    pub movement: usize,
    pub arrival_t: f64,
    pub speed_factor: f64,
    /// Uniform draw compared against connected-vehicle penetration at spawn.
    pub cv_u: f64,
    pub cam_ok: bool,
    pub manual: bool,
    pub trip: Trip,
}

#[derive(Clone, Debug)]
pub struct DemandGen {
    pub rng: Rng,
    next_t: Vec<f64>,
    next_ped: f64,
    seed: u64,
}

impl DemandGen {
    pub fn new(seed: u64, legs: usize) -> Self {
        DemandGen { rng: Rng::new(seed), next_t: vec![0.0; legs], next_ped: 0.0, seed }
    }

    fn multiplier(&self, pattern: Pattern, leg: usize, t: f64) -> f64 {
        match pattern {
            Pattern::Random => 1.0,
            Pattern::Platoons => {
                let cycle = 90.0;
                let green = 38.0;
                let phase = (t + leg as f64 * 23.0).rem_euclid(cycle);
                if phase < green { (cycle - (cycle - green) * 0.15) / green } else { 0.15 }
            }
            Pattern::RushHour => {
                let x = (t - 900.0) / 420.0;
                0.55 + 0.95 * (-x * x).exp()
            }
            Pattern::Surges => {
                let block = (t / 120.0).floor() as u64;
                let mut r = Rng::new(self.seed ^ (block * 7919 + leg as u64 * 104_729));
                [0.45, 0.8, 1.0, 1.25, 1.7][r.below(5)]
            }
        }
    }

    fn max_multiplier(pattern: Pattern) -> f64 {
        match pattern {
            Pattern::Random => 1.0,
            Pattern::Platoons => (90.0 - 52.0 * 0.15) / 38.0,
            Pattern::RushHour => 1.5,
            Pattern::Surges => 1.7,
        }
    }

    /// Draw arrivals in (t, t+dt] using thinning of a non-homogeneous Poisson process.
    pub fn generate(&mut self, cfg: &DemandConfig, net: &Network, t: f64, dt: f64, out: &mut Vec<(usize, Pending)>) {
        let mmax = Self::max_multiplier(cfg.pattern);
        for li in 0..net.legs.len() {
            let ci = compass_index(net.legs[li].compass);
            let base = if cfg.fed[ci] { 0.0 } else { cfg.rate[ci] * cfg.scale / 3600.0 };
            if base <= 0.0 {
                self.next_t[li] = t + dt;
                continue;
            }
            let lam_max = base * mmax;
            if self.next_t[li] < t {
                self.next_t[li] = t + self.rng.exp(lam_max);
            }
            while self.next_t[li] <= t + dt {
                let at = self.next_t[li];
                self.next_t[li] += self.rng.exp(lam_max);
                let accept = self.rng.f64() < self.multiplier(cfg.pattern, li, at) / mmax;
                // Always consume the same random draws so streams stay aligned.
                let p = self.draw(cfg, net, li, at);
                if accept {
                    out.push((li, p));
                }
            }
        }
    }

    pub fn draw(&mut self, cfg: &DemandConfig, net: &Network, leg: usize, at: f64) -> Pending {
        let shares = cfg.valid_shares(net, leg);
        let ti = self.rng.weighted(&shares);
        let turn = Turn::ALL[ti];
        let movement = net.movement_for(leg, turn).unwrap_or_else(|| {
            net.movements.iter().position(|m| m.leg == leg).expect("leg has movements")
        });
        let mut mix = cfg.mix;
        if mix.iter().sum::<f64>() <= 0.0 {
            mix[0] = 1.0;
        }
        let kind = ALL_KINDS[self.rng.weighted(&mix)];
        let sf = (1.0 + 0.08 * self.rng.normal()).clamp(0.82, 1.18);
        Pending {
            kind,
            movement,
            arrival_t: at,
            speed_factor: sf,
            cv_u: self.rng.f64(),
            cam_ok: self.rng.f64() < 0.985,
            manual: false,
            trip: Trip { start: at, ..Default::default() },
        }
    }

    /// Pedestrian arrivals: returns (crosswalk, from_b_side) pairs.
    pub fn peds(&mut self, cfg: &DemandConfig, n_cw: usize, t: f64, dt: f64) -> Vec<(usize, bool)> {
        let mut out = vec![];
        let rate = cfg.ped_rate * cfg.scale / 3600.0;
        if rate <= 0.0 || n_cw == 0 {
            self.next_ped = t + dt;
            return out;
        }
        if self.next_ped < t {
            self.next_ped = t + self.rng.exp(rate);
        }
        while self.next_ped <= t + dt {
            self.next_ped += self.rng.exp(rate);
            out.push((self.rng.below(n_cw), self.rng.chance(0.5)));
        }
        out
    }
}
