// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Deep reinforcement-learning signal controller.
//!
//! State: per movement (queue, arrivals within 10 s and 10-30 s, longest wait, extra
//! person-weight from buses, whether it is currently served), current stage one-hot,
//! time in stage and waiting pedestrians. Action: which stage to run next (choosing
//! the current stage extends it). Reward: negative person-delay accrued until the
//! next decision. Decisions every 2 s once minimum green is satisfied.

use super::maxpressure::MaxPressure;
use super::predictive::Predictive;
use super::{Brain, Controller, Ctx, stat};
use crate::rl::dqn::{Dqn, SavedModel, Transition, argmax_masked};
use crate::sim::network::{MStat, Network};
use crate::sim::rng::Rng;
use crate::sim::signal::Phase;
use std::path::PathBuf;

pub const MAX_CW: usize = 4;
pub const GAMMA_PER_2S: f64 = 0.96;
pub const REWARD_SCALE: f64 = 150.0;

pub struct RlController {
    pub agent: Dqn,
    pub learning: bool,
    pub epsilon: f64,
    pub expert_prob: f64,
    pub expert: Option<Box<dyn Controller>>,
    pub episodes: u64,
    pub loaded_from: Option<String>,
    pub decisions: u64,
    pub reward_sum: f64,
    pub train_every: u32,
    decision_dt: f64,
    next_decision: f64,
    prev: Option<(Vec<f32>, usize, f64)>,
    r_acc: f64,
    last_q: Vec<f32>,
    names: Vec<String>,
    cur: usize,
    expert_wish: Option<(usize, f64)>,
    rng: Rng,
    layout: String,
    since_train: u32,
    last_mask: Vec<bool>,
}

pub fn n_inputs(net: &Network) -> usize {
    net.movements.len() * 6 + net.stages.len() + 1 + MAX_CW
}

/// Where to save this layout's weights (see [`super::weights`]).
pub fn weights_path(layout: &str) -> PathBuf {
    super::weights::save_path(&weights_file(layout))
}

pub fn weights_file(layout: &str) -> String {
    format!("dqn_{layout}.json")
}

impl RlController {
    pub fn new(net: &Network, seed: u64) -> Self {
        let agent = Dqn::new(n_inputs(net), net.stages.len().max(1), seed);
        Self::with_agent(net, agent)
    }

    fn with_agent(net: &Network, agent: Dqn) -> Self {
        RlController {
            agent,
            learning: false,
            epsilon: 0.05,
            expert_prob: 0.0,
            expert: None,
            episodes: 0,
            loaded_from: None,
            decisions: 0,
            reward_sum: 0.0,
            train_every: 1,
            decision_dt: 2.0,
            next_decision: 0.0,
            prev: None,
            r_acc: 0.0,
            last_q: vec![],
            names: net.stages.iter().map(|s| s.name.clone()).collect(),
            cur: 0,
            expert_wish: None,
            rng: Rng::new(0x5EED),
            layout: net.kind.key().to_string(),
            since_train: 0,
            last_mask: vec![],
        }
    }

    pub fn load_or_new(net: &Network) -> Self {
        if let Some((txt, source)) = super::weights::load(&weights_file(net.kind.key())) {
            if let Ok(m) = serde_json::from_str::<SavedModel>(&txt) {
                if m.n_in == n_inputs(net) && m.n_out == net.stages.len() {
                    let mut c = Self::with_agent(net, Dqn::from_net(m.net, Rng::new(7)));
                    c.episodes = m.episodes;
                    c.loaded_from = Some(source);
                    return c;
                }
            }
        }
        let mut c = Self::new(net, 7);
        // An untrained network should learn rather than act randomly forever.
        c.learning = true;
        c.epsilon = 0.2;
        c
    }

    pub fn save(&self, path: &std::path::Path) -> std::io::Result<()> {
        let m = SavedModel {
            layout: self.layout.clone(),
            n_in: self.agent.online.n_in(),
            n_out: self.agent.online.n_out(),
            episodes: self.episodes,
            net: self.agent.online.clone(),
        };
        std::fs::write(path, serde_json::to_string(&m).unwrap())
    }

    pub fn set_expert_predictive(&mut self, net: &Network) {
        self.expert = Some(Box::new(Predictive::new(net)));
    }
    pub fn set_expert_maxpressure(&mut self, net: &Network) {
        self.expert = Some(Box::new(MaxPressure::new(net)));
    }

    /// Reset per-episode bookkeeping (keeps the network and replay memory).
    pub fn new_episode(&mut self, net: &Network) {
        self.prev = None;
        self.r_acc = 0.0;
        self.next_decision = 0.0;
        self.expert_wish = None;
        if self.expert.is_some() {
            self.set_expert_predictive(net);
        }
    }

    /// Safety shield (action masking) around the learned policy, as used when RL
    /// controllers are deployed: never hold past max green, never hold an empty
    /// stage while others wait, never switch to a stage with no demand.
    pub fn shield(ctx: &Ctx) -> Vec<bool> {
        let net = ctx.net;
        let cur = ctx.sig.stage;
        let demand: Vec<f64> = (0..net.stages.len())
            .map(|j| {
                let mut d = 0.0;
                for m in 0..net.movements.len() {
                    if stat(net, j, m).is_some() {
                        let mo = &ctx.view.moves[m];
                        d += mo.queue + mo.vehicles.iter().filter(|o| !o.queued && o.eta < 12.0).map(|o| o.weight).sum::<f64>();
                    }
                }
                d + net.stages[j].walk.iter().map(|&cw| ctx.view.ped_waiting[cw] as f64).sum::<f64>()
            })
            .collect();
        let others_waiting = (0..net.stages.len()).any(|j| j != cur && demand[j] >= 0.5);
        let mut mask: Vec<bool> = demand.iter().map(|&d| d >= 0.5).collect();
        mask[cur] = ctx.sig.green_elapsed < 60.0 && (demand[cur] >= 0.5 || !others_waiting);
        if !mask.iter().any(|&m| m) {
            mask[cur] = true;
        }
        mask
    }

    pub fn features(ctx: &Ctx) -> Vec<f32> {
        let net = ctx.net;
        let mut f = Vec::with_capacity(n_inputs(net));
        let cur = ctx.sig.stage;
        for m in 0..net.movements.len() {
            let mo = &ctx.view.moves[m];
            let mut n10 = 0.0;
            let mut n30 = 0.0;
            for o in &mo.vehicles {
                if o.queued {
                    continue;
                }
                if o.eta < 10.0 {
                    n10 += o.weight;
                } else if o.eta < 30.0 {
                    n30 += o.weight;
                }
            }
            let served = match stat(net, cur, m) {
                Some(MStat::G) | Some(MStat::Rt) => 1.0,
                Some(MStat::P) => 0.5,
                None => 0.0,
            };
            f.push((mo.queue / 8.0).min(3.0) as f32);
            f.push((n10 / 8.0).min(3.0) as f32);
            f.push((n30 / 8.0).min(3.0) as f32);
            f.push((mo.max_wait / 60.0).min(3.0) as f32);
            f.push(((mo.pqueue - 1.4 * mo.queue).max(0.0) / 20.0).min(3.0) as f32);
            f.push(served);
        }
        for s in 0..net.stages.len() {
            f.push(if s == cur { 1.0 } else { 0.0 });
        }
        f.push((ctx.sig.green_elapsed / 60.0).min(2.0) as f32);
        for cw in 0..MAX_CW {
            let n = ctx.view.ped_waiting.get(cw).copied().unwrap_or(0);
            f.push((n as f32 / 4.0).min(1.5));
        }
        f
    }
}

impl Controller for RlController {
    fn name(&self) -> &'static str {
        "Deep RL"
    }

    fn as_rl(&mut self) -> Option<&mut RlController> {
        Some(self)
    }

    fn into_rl(self: Box<Self>) -> Option<Box<RlController>> {
        Some(self)
    }

    fn step(&mut self, ctx: &Ctx) -> Option<usize> {
        self.r_acc -= ctx.reward_delay / REWARD_SCALE;
        if let Some(e) = self.expert.as_mut() {
            if let Some(j) = e.step(ctx) {
                self.expert_wish = Some((j, ctx.t));
            }
        }
        if ctx.sig.phase != Phase::Green || !ctx.can_switch || ctx.t < self.next_decision {
            return None;
        }
        self.next_decision = ctx.t + self.decision_dt;
        self.cur = ctx.sig.stage;
        let s = Self::features(ctx);
        let mask = Self::shield(ctx);
        if let Some((ps, pa, pt)) = self.prev.take() {
            if self.learning {
                let disc = GAMMA_PER_2S.powf((ctx.t - pt) / 2.0) as f32;
                self.agent.push(Transition { s: ps, a: pa, r: self.r_acc as f32, s2: s.clone(), disc, mask2: mask.clone() });
                self.since_train += 1;
                if self.since_train >= self.train_every {
                    self.since_train = 0;
                    self.agent.train_step();
                    self.agent.train_step();
                }
            }
        }
        self.reward_sum += self.r_acc;
        self.r_acc = 0.0;
        self.last_q = self.agent.q(&s);
        let greedy = argmax_masked(&self.last_q, &mask);
        let a = if self.learning && self.expert.is_some() && self.rng.chance(self.expert_prob) {
            match self.expert_wish {
                Some((j, t)) if ctx.t - t < 1.15 => j,
                _ => self.cur,
            }
        } else if self.learning && self.rng.chance(self.epsilon) {
            // Exploration over shield-allowed actions, biased toward holding.
            let allowed: Vec<usize> = (0..mask.len()).filter(|&i| mask[i]).collect();
            if mask[self.cur] && self.rng.chance(0.5) { self.cur } else { allowed[self.rng.below(allowed.len())] }
        } else {
            greedy
        };
        self.decisions += 1;
        self.last_mask = mask;
        self.prev = Some((s, a, ctx.t));
        (a != self.cur).then_some(a)
    }

    fn brain(&self) -> Brain {
        let best = if self.last_q.is_empty() { usize::MAX } else { argmax_masked(&self.last_q, &self.last_mask) };
        let mut notes = vec![match &self.loaded_from {
            Some(p) => format!("pretrained weights: {} episodes ({})", self.episodes, short_path(p)),
            None => "no pretrained weights found: learning from scratch".into(),
        }];
        notes.push(format!(
            "online learning: {}   ε={:.2}   replay={}   updates={}",
            if self.learning { "ON" } else { "off" },
            self.epsilon,
            self.agent.replay_len(),
            self.agent.updates
        ));
        if let Some(l) = self.agent.losses.back() {
            notes.push(format!("TD loss {:.4}", l));
        }
        Brain {
            title: "Q-values (expected future reward per action)".into(),
            bars: self
                .last_q
                .iter()
                .enumerate()
                .map(|(i, q)| {
                    let mut label = if i == self.cur { format!("hold {}", self.names[i]) } else { format!("→ {}", self.names[i]) };
                    if self.last_mask.get(i) == Some(&false) {
                        label.push_str(" (shielded)");
                    }
                    (label, *q as f64, i == best)
                })
                .collect(),
            lower_is_better: false,
            notes,
        }
    }
}

fn short_path(p: &str) -> String {
    p.rsplit('/').next().unwrap_or(p).to_string()
}
