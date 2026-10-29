// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Double DQN with experience replay, Huber loss, Polyak-averaged target network and
//! semi-Markov discounting (decisions are not evenly spaced in time).

use super::nn::{Adam, Grads, Mlp};
use crate::sim::rng::Rng;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

#[derive(Clone, Debug)]
pub struct Transition {
    pub s: Vec<f32>,
    pub a: usize,
    pub r: f32,
    pub s2: Vec<f32>,
    /// Discount to apply to the bootstrap term (gamma^elapsed).
    pub disc: f32,
    /// Actions allowed in `s2` by the safety shield.
    pub mask2: Vec<bool>,
}

pub struct Dqn {
    pub online: Mlp,
    pub target: Mlp,
    adam: Adam,
    replay: Vec<Transition>,
    cap: usize,
    pos: usize,
    pub rng: Rng,
    pub batch: usize,
    pub tau: f32,
    pub updates: u64,
    pub losses: VecDeque<f32>,
    grads: Grads,
}

#[derive(Serialize, Deserialize)]
pub struct SavedModel {
    pub layout: String,
    pub n_in: usize,
    pub n_out: usize,
    pub episodes: u64,
    pub net: Mlp,
}

impl Dqn {
    pub fn new(n_in: usize, n_out: usize, seed: u64) -> Dqn {
        let mut rng = Rng::new(seed);
        let online = Mlp::new(&[n_in, 128, 128, n_out], &mut rng);
        Self::from_net(online, rng)
    }

    pub fn from_net(online: Mlp, rng: Rng) -> Dqn {
        let target = online.clone();
        let adam = Adam::new(&online, 5e-4);
        let grads = Grads::zeros(&online);
        Dqn {
            online,
            target,
            adam,
            replay: Vec::new(),
            cap: 60_000,
            pos: 0,
            rng,
            batch: 64,
            tau: 0.01,
            updates: 0,
            losses: VecDeque::new(),
            grads,
        }
    }

    pub fn q(&self, s: &[f32]) -> Vec<f32> {
        self.online.forward(s)
    }

    pub fn greedy(&self, s: &[f32]) -> usize {
        argmax(&self.q(s))
    }

    pub fn push(&mut self, t: Transition) {
        if self.replay.len() < self.cap {
            self.replay.push(t);
        } else {
            self.replay[self.pos] = t;
            self.pos = (self.pos + 1) % self.cap;
        }
    }

    pub fn set_lr(&mut self, lr: f32) {
        self.adam.lr = lr;
    }

    pub fn replay_len(&self) -> usize {
        self.replay.len()
    }

    pub fn train_step(&mut self) -> Option<f32> {
        if self.replay.len() < 1000 {
            return None;
        }
        self.grads.clear();
        let mut loss = 0.0;
        let n_out = self.online.n_out();
        for _ in 0..self.batch {
            let idx = self.rng.below(self.replay.len());
            let tr = &self.replay[idx];
            let acts = self.online.forward_all(&tr.s);
            let q = acts.last().unwrap();
            let a2 = argmax_masked(&self.online.forward(&tr.s2), &tr.mask2);
            let q2 = self.target.forward(&tr.s2)[a2];
            let y = tr.r + tr.disc * q2;
            let td = q[tr.a] - y;
            // Huber loss gradient.
            let g = td.clamp(-1.0, 1.0);
            loss += if td.abs() < 1.0 { 0.5 * td * td } else { td.abs() - 0.5 };
            let mut gout = vec![0.0f32; n_out];
            gout[tr.a] = g;
            self.online.backward(&acts, &gout, &mut self.grads);
        }
        let scale = 1.0 / self.batch as f32;
        self.adam.apply(&mut self.online, &self.grads, scale);
        self.target.soft_update(&self.online, self.tau);
        self.updates += 1;
        let l = loss * scale;
        self.losses.push_back(l);
        while self.losses.len() > 500 {
            self.losses.pop_front();
        }
        Some(l)
    }
}

/// Argmax over allowed actions (all actions if the mask allows none).
pub fn argmax_masked(v: &[f32], mask: &[bool]) -> usize {
    let mut best: Option<usize> = None;
    for i in 0..v.len() {
        if mask.get(i).copied().unwrap_or(true) && best.is_none_or(|b| v[i] > v[b]) {
            best = Some(i);
        }
    }
    best.unwrap_or_else(|| argmax(v))
}

pub fn argmax(v: &[f32]) -> usize {
    let mut b = 0;
    for i in 1..v.len() {
        if v[i] > v[b] {
            b = i;
        }
    }
    b
}
