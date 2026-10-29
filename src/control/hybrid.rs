//! Hybrid controller: the predictive (schedule-driven MPC) controller makes every
//! signal decision, while a Double-DQN agent re-tunes it every 10 s from the observed
//! traffic state, choosing its fairness weight, hold/switch bias and maximum green.
//!
//! The learned part never touches the signal directly, so the planner's guarantees
//! (feasible schedules, clearance intervals, max green) are kept. Training first finds
//! the best single tuning for the layout (the "base"); the network is kept only if it
//! beats that base on the evaluation scenarios, otherwise the hybrid runs the base.

use super::predictive::Predictive;
use super::rl::{GAMMA_PER_2S, REWARD_SCALE, RlController};
use super::{Brain, Controller, Ctx};
use crate::rl::dqn::{Dqn, SavedModel, Transition, argmax};
use serde::{Deserialize, Serialize};
use crate::sim::network::Network;
use crate::sim::rng::Rng;
use std::path::PathBuf;

const FAIRNESS: [f64; 3] = [0.0, 0.004, 0.012];
const HOLD_BIAS: [f64; 4] = [-0.05, 0.0, 0.08, 0.2];
const MAX_GREEN: [f64; 4] = [35.0, 45.0, 60.0, 90.0];
pub const N_ACTIONS: usize = 48;
/// The planner's hand-tuned defaults (fairness 0.004, no bias, 60 s max green).
pub const DEFAULT_ACTION: usize = 16 + 4 + 2;
const DECISION_DT: f64 = 10.0;

fn preset(a: usize) -> (f64, f64, f64) {
    (FAIRNESS[a / 16], HOLD_BIAS[(a / 4) % 4], MAX_GREEN[a % 4])
}

pub fn preset_name(a: usize) -> String {
    let (f, b, g) = preset(a);
    format!("fair {f:.3} · bias {b:+.2} · max {g:.0}s")
}

pub fn n_inputs(net: &Network) -> usize {
    super::rl::n_inputs(net)
}

pub fn weights_path(layout: &str) -> PathBuf {
    let file = format!("hybrid_{layout}.json");
    let local = PathBuf::from("weights").join(&file);
    if local.exists() {
        return local;
    }
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("weights").join(file)
}

/// On-disk form: the best static tuning plus, if it earned its place, the network.
#[derive(Serialize, Deserialize)]
pub struct SavedHybrid {
    pub base_action: usize,
    pub base_score: f64,
    pub model: Option<SavedModel>,
}

pub struct Hybrid {
    pub planner: Predictive,
    /// Tuning used when the network has nothing better to say.
    pub base_action: usize,
    /// The network is trained (loaded or being trained) and may choose tunings.
    pub has_model: bool,
    pub agent: Dqn,
    pub learning: bool,
    pub epsilon: f64,
    /// Probability of choosing the default tuning while learning (decays in training).
    pub expert_prob: f64,
    pub episodes: u64,
    pub loaded_from: Option<String>,
    pub decisions: u64,
    /// Pin one tuning (baselines / experiments) instead of asking the agent.
    pub fixed_action: Option<usize>,
    action: usize,
    next_decision: f64,
    prev: Option<(Vec<f32>, usize, f64)>,
    r_acc: f64,
    last_q: Vec<f32>,
    rng: Rng,
    layout: String,
}

impl Hybrid {
    pub fn new(net: &Network, seed: u64) -> Self {
        Self::with_agent(net, Dqn::new(n_inputs(net), N_ACTIONS, seed))
    }

    fn with_agent(net: &Network, agent: Dqn) -> Self {
        Hybrid {
            planner: Predictive::new(net),
            base_action: DEFAULT_ACTION,
            has_model: false,
            agent,
            learning: false,
            epsilon: 0.0,
            expert_prob: 0.0,
            episodes: 0,
            loaded_from: None,
            decisions: 0,
            fixed_action: None,
            action: DEFAULT_ACTION,
            next_decision: 0.0,
            prev: None,
            r_acc: 0.0,
            last_q: vec![],
            rng: Rng::new(0x4B1D),
            layout: net.kind.key().to_string(),
        }
    }

    /// Trained base tuning and network if present; otherwise an untrained agent that
    /// keeps the planner's default tuning (it behaves exactly like the predictive
    /// controller).
    pub fn load_or_new(net: &Network) -> Self {
        let path = weights_path(net.kind.key());
        if let Ok(txt) = std::fs::read_to_string(&path) {
            if let Ok(sv) = serde_json::from_str::<SavedHybrid>(&txt) {
                let mut h = match sv.model {
                    Some(m) if m.n_in == n_inputs(net) && m.n_out == N_ACTIONS => {
                        let mut h = Self::with_agent(net, Dqn::from_net(m.net, Rng::new(11)));
                        h.episodes = m.episodes;
                        h.has_model = true;
                        h
                    }
                    _ => Self::new(net, 11),
                };
                h.base_action = sv.base_action.min(N_ACTIONS - 1);
                h.loaded_from = Some(path.display().to_string());
                return h;
            }
        }
        Self::new(net, 11)
    }

    pub fn save(&self, path: &std::path::Path, base_score: f64, with_model: bool) -> std::io::Result<()> {
        let model = with_model.then(|| SavedModel {
            layout: self.layout.clone(),
            n_in: self.agent.online.n_in(),
            n_out: self.agent.online.n_out(),
            episodes: self.episodes,
            net: self.agent.online.clone(),
        });
        let sv = SavedHybrid { base_action: self.base_action, base_score, model };
        std::fs::write(path, serde_json::to_string(&sv).unwrap())
    }

    pub fn new_episode(&mut self, net: &Network) {
        self.planner = Predictive::new(net);
        self.prev = None;
        self.r_acc = 0.0;
        self.next_decision = 0.0;
        self.action = self.base_action;
    }

    fn apply(&mut self, a: usize) {
        let (f, b, g) = preset(a);
        self.planner.fairness = f;
        self.planner.hold_bias = b;
        self.planner.max_green = g;
        self.action = a;
    }
}

impl Controller for Hybrid {
    fn name(&self) -> &'static str {
        "Hybrid (RL-tuned predictive)"
    }

    fn as_hybrid(&mut self) -> Option<&mut Hybrid> {
        Some(self)
    }

    fn into_hybrid(self: Box<Self>) -> Option<Box<Hybrid>> {
        Some(self)
    }

    fn step(&mut self, ctx: &Ctx) -> Option<usize> {
        self.r_acc -= ctx.reward_delay / REWARD_SCALE;
        if ctx.t >= self.next_decision {
            self.next_decision = ctx.t + DECISION_DT;
            let s = RlController::features(ctx);
            if let Some((ps, pa, pt)) = self.prev.take() {
                if self.learning {
                    let disc = GAMMA_PER_2S.powf((ctx.t - pt) / 2.0) as f32;
                    let mask2 = vec![true; N_ACTIONS];
                    self.agent.push(Transition { s: ps, a: pa, r: self.r_acc as f32, s2: s.clone(), disc, mask2 });
                    for _ in 0..4 {
                        self.agent.train_step();
                    }
                }
            }
            self.r_acc = 0.0;
            self.last_q = self.agent.q(&s);
            let a = if let Some(a) = self.fixed_action {
                a
            } else if self.learning && self.rng.chance(self.expert_prob) {
                self.base_action
            } else if self.learning && self.rng.chance(self.epsilon) {
                // Explore near the current tuning more often than far from it.
                if self.rng.chance(0.5) { self.action } else { self.rng.below(N_ACTIONS) }
            } else if self.has_model || self.learning {
                argmax(&self.last_q)
            } else {
                self.base_action
            };
            self.apply(a);
            self.decisions += 1;
            self.prev = Some((s, a, ctx.t));
        }
        self.planner.step(ctx)
    }

    fn brain(&self) -> Brain {
        let inner = self.planner.brain();
        let mut notes = vec![format!("RL tuning now: {}", preset_name(self.action))];
        notes.push(match (&self.loaded_from, self.has_model) {
            (Some(_), true) => format!("trained network ({} episodes) over base tuning {}", self.episodes, preset_name(self.base_action)),
            (Some(_), false) => format!("no network beat the best static tuning; running {}", preset_name(self.base_action)),
            (None, _) if self.learning => "learning online from scratch".into(),
            (None, _) => "no trained weights: running the planner's default tuning".into(),
        });
        if !self.last_q.is_empty() {
            let mut order: Vec<usize> = (0..N_ACTIONS).collect();
            order.sort_by(|&a, &b| self.last_q[b].partial_cmp(&self.last_q[a]).unwrap());
            let top: Vec<String> = order.iter().take(3).map(|&a| format!("{} ({:.2})", preset_name(a), self.last_q[a])).collect();
            notes.push(format!("best tunings by Q: {}", top.join("; ")));
        }
        notes.extend(inner.notes);
        Brain { title: format!("RL-tuned planner · {}", inner.title), bars: inner.bars, lower_is_better: true, notes }
    }
}
