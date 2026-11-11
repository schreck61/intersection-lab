// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Headless reinforcement-learning training, per signalized layout, on randomized
//! demand (rates, turning shares, arrival pattern, pedestrians) and sensor conditions.
//! Checkpoints are chosen on a validation set; a separate test set, never used for any
//! choice, reports how the chosen weights do.
//!
//! * `dqn`: the Deep RL controller chooses stages; the predictive controller acts as a
//!   decaying "expert" for exploration.
//! * `hybrid`: the agent re-tunes the predictive planner every 10 s; its default
//!   tuning is the decaying expert.
//!
//! usage: train [layout|all] [episodes] [episode-seconds] [dqn|hybrid|both]
//!
//! Environment: `RESUME=1` fine-tunes the saved weights, `SEED=n` changes the random
//! seed, and `HIDDEN=128,128` overrides the hidden layer sizes of a new network.

use std::time::Instant;
use intersection_lab::control::hybrid::{self, Hybrid};
use intersection_lab::control::rl::{self, RlController};
use intersection_lab::control::{Controller, CtrlKind, make};
use intersection_lab::sim::demand::{DemandConfig, Pattern};
use intersection_lab::sim::network::{LayoutKind, Network};
use intersection_lab::sim::rng::Rng;
use intersection_lab::sim::sensors::{Conditions, SensorConfig};
use intersection_lab::sim::{ControlMode, Sim};

/// A checkpoint is accepted only if its mean pedestrian wait is within this many
/// seconds of the predictive controller's on the same scenarios.
const PED_MARGIN: f64 = 5.0;

fn random_demand(net: &Network, rng: &mut Rng) -> DemandConfig {
    let mut d = DemandConfig::default_for(net);
    for r in d.rate.iter_mut() {
        *r *= rng.range(0.35, 1.35);
    }
    for t in d.turn.iter_mut() {
        for x in t.iter_mut() {
            if *x > 0.0 {
                *x = (*x * rng.range(0.5, 1.6)).max(0.02);
            }
        }
    }
    d.pattern = Pattern::ALL[rng.below(4)];
    d.ped_rate = rng.range(0.0, 140.0);
    d.mix[3] = rng.range(0.0, 0.08); // buses
    d
}

/// Demand, sensors and arrival seed for one run.
struct Scenario {
    demand: DemandConfig,
    sensors: SensorConfig,
    seed: u64,
}

/// A fixed scenario set. Every block of eight holds two runs at design demand, one each
/// at 0.8× and 1.2×, two with randomized demand, one in bad weather and one with a
/// quarter of the loop detectors failed.
fn scenario_set(net: &Network, base: u64, n: u64) -> Vec<Scenario> {
    (0..n)
        .map(|k| {
            let mut demand = DemandConfig::default_for(net);
            let mut sensors = SensorConfig::default();
            match k % 8 {
                2 => demand.scale = 0.8,
                3 => demand.scale = 1.2,
                4 | 5 => demand = random_demand(net, &mut Rng::new(base + k)),
                6 => sensors.conditions = Conditions::ALL[1 + ((k / 8 + base / 1000) % 4) as usize],
                7 => sensors.loop_fault = 0.25,
                _ => {}
            }
            Scenario { demand, sensors, seed: base + k }
        })
        .collect()
}

/// Model selection (checkpoints, the hybrid's static tuning).
fn validation_set(net: &Network) -> Vec<Scenario> {
    scenario_set(net, 9000, 16)
}

/// Held out from every choice; reported only.
fn test_set(net: &Network) -> Vec<Scenario> {
    scenario_set(net, 50000, 24)
}

/// The two trainable controllers behind one interface.
enum Agent {
    Dqn(Box<RlController>),
    Hybrid(Box<Hybrid>),
}

impl Agent {
    fn kind(&self) -> CtrlKind {
        match self {
            Agent::Dqn(_) => CtrlKind::DeepRl,
            Agent::Hybrid(_) => CtrlKind::Hybrid,
        }
    }
    fn tag(&self) -> &'static str {
        match self {
            Agent::Dqn(_) => "dqn",
            Agent::Hybrid(_) => "hybrid",
        }
    }
    fn into_controller(self) -> Box<dyn Controller> {
        match self {
            Agent::Dqn(a) => a,
            Agent::Hybrid(a) => a,
        }
    }
    fn from_controller(c: Box<dyn Controller>, kind: CtrlKind) -> Agent {
        match kind {
            CtrlKind::DeepRl => Agent::Dqn(c.into_rl().unwrap()),
            _ => Agent::Hybrid(c.into_hybrid().unwrap()),
        }
    }
    fn set(&mut self, learning: bool, eps: f64, expert: f64) {
        match self {
            Agent::Dqn(a) => (a.learning, a.epsilon, a.expert_prob) = (learning, eps, expert),
            Agent::Hybrid(a) => (a.learning, a.epsilon, a.expert_prob) = (learning, eps, expert),
        }
    }
    fn schedule(&self) -> (bool, f64, f64) {
        match self {
            Agent::Dqn(a) => (a.learning, a.epsilon, a.expert_prob),
            Agent::Hybrid(a) => (a.learning, a.epsilon, a.expert_prob),
        }
    }
    fn episodes(&mut self) -> &mut u64 {
        match self {
            Agent::Dqn(a) => &mut a.episodes,
            Agent::Hybrid(a) => &mut a.episodes,
        }
    }
    fn new_episode(&mut self, net: &Network) {
        match self {
            Agent::Dqn(a) => a.new_episode(net),
            Agent::Hybrid(a) => a.new_episode(net),
        }
    }
    fn save(&self, layout: LayoutKind, score: f64) {
        std::fs::create_dir_all("weights").ok();
        match self {
            Agent::Dqn(a) => a.save(&rl::weights_path(layout.key())).ok(),
            Agent::Hybrid(a) => a.save(&hybrid::weights_path(layout.key()), Some(score), true).ok(),
        };
    }
    fn loss(&self) -> f32 {
        let l = match self {
            Agent::Dqn(a) => a.agent.losses.back(),
            Agent::Hybrid(a) => a.agent.losses.back(),
        };
        l.copied().unwrap_or(0.0)
    }
    fn set_lr(&mut self, lr: f32) {
        match self {
            Agent::Dqn(a) => a.agent.set_lr(lr),
            Agent::Hybrid(a) => a.agent.set_lr(lr),
        }
    }
    /// Run one episode with this agent in control: person delay and pedestrian wait.
    fn run(self, net: &Network, sc: &Scenario, secs: f64) -> ((f64, f64), Agent) {
        let kind = self.kind();
        let mut sim = Sim::with_demand(net.clone(), ControlMode::Signal, kind, sc.demand.clone(), sc.seed);
        sim.sensors = sc.sensors.clone();
        sim.controller = self.into_controller();
        sim.run_for(secs);
        let s = sim.stats.summary(120.0, sim.t, &sim.vehicles, &sim.queues);
        let c = std::mem::replace(&mut sim.controller, make(CtrlKind::Fixed, net, &sim.demand));
        ((s.person_delay_all, s.ped_avg_wait), Agent::from_controller(c, kind))
    }
}

/// Mean person delay and pedestrian wait over a scenario set (greedy policy).
fn evaluate(net: &Network, agent: Agent, set: &[Scenario], secs: f64) -> ((f64, f64), Agent) {
    let mut agent = agent;
    let saved = agent.schedule();
    agent.set(false, 0.0, 0.0);
    let had_model = if let Agent::Hybrid(h) = &mut agent { std::mem::replace(&mut h.has_model, true) } else { false };
    let (mut pd, mut pw) = (0.0, 0.0);
    for sc in set {
        agent.new_episode(net);
        let (r, back) = agent.run(net, sc, secs);
        pd += r.0;
        pw += r.1;
        agent = back;
    }
    agent.set(saved.0, saved.1, saved.2);
    if let Agent::Hybrid(h) = &mut agent {
        h.has_model = had_model;
    }
    let n = set.len() as f64;
    ((pd / n, pw / n), agent)
}

fn baseline(net: &Network, kind: CtrlKind, set: &[Scenario], secs: f64) -> (f64, f64) {
    let (mut pd, mut pw) = (0.0, 0.0);
    for sc in set {
        let mut sim = Sim::with_demand(net.clone(), ControlMode::Signal, kind, sc.demand.clone(), sc.seed);
        sim.sensors = sc.sensors.clone();
        sim.run_for(secs);
        let s = sim.stats.summary(120.0, sim.t, &sim.vehicles, &sim.queues);
        pd += s.person_delay_all;
        pw += s.ped_avg_wait;
    }
    let n = set.len() as f64;
    (pd / n, pw / n)
}

/// A hybrid pinned to one tuning, with no network.
fn static_hybrid(net: &Network, action: usize) -> Agent {
    let mut h = Box::new(Hybrid::new(net, 1));
    h.fixed_action = Some(action);
    Agent::Hybrid(h)
}

fn train(kind: LayoutKind, episodes: u64, secs: f64, which: &str) {
    let net = Network::build(kind);
    let val = validation_set(&net);
    let base_pred = baseline(&net, CtrlKind::Predictive, &val, secs);
    let base_mp = baseline(&net, CtrlKind::MaxPressure, &val, secs);
    let base_fixed = baseline(&net, CtrlKind::Fixed, &val, secs);
    let ped_limit = base_pred.1 + PED_MARGIN;
    println!(
        "[{}] validation baselines (person delay s): fixed {:.1}  max-pressure {:.1}  predictive {:.1} (pedestrian wait {:.1}, limit {:.1})",
        kind.key(),
        base_fixed.0,
        base_mp.0,
        base_pred.0,
        base_pred.1,
        ped_limit
    );
    let hybrid = which == "hybrid";
    let mut base = (f64::INFINITY, hybrid::DEFAULT_ACTION);
    if hybrid {
        // The bar the agent must clear to add anything: the best single tuning.
        let scores: Vec<((f64, f64), usize)> =
            (0..hybrid::N_ACTIONS).map(|a| (evaluate(&net, static_hybrid(&net, a), &val, secs).0, a)).collect();
        base = scores
            .iter()
            .filter(|(s, _)| s.1 <= ped_limit)
            .fold(base, |b, &(s, a)| if s.0 < b.0 { (s.0, a) } else { b });
        println!(
            "[{}:hybrid] best static tuning: {} -> {:.1} s (default tuning {:.1} s)",
            kind.key(),
            hybrid::preset_name(base.1),
            base.0,
            scores[hybrid::DEFAULT_ACTION].0.0
        );
    }
    let mut rng = Rng::new(kind as u64 * 7777 + 1 + resume_salt() + if hybrid { 31 } else { 0 } + seed() * 1000);
    let mut best = f64::INFINITY;
    let resume = std::env::var_os("RESUME").is_some();
    let hidden: Vec<usize> = std::env::var("HIDDEN")
        .ok()
        .map(|h| h.split(',').map(|x| x.trim().parse().expect("HIDDEN: comma-separated layer sizes")).collect())
        .unwrap_or_else(|| (if hybrid { hybrid::HIDDEN } else { rl::HIDDEN }).to_vec());
    let mut agent = match (hybrid, resume) {
        (false, true) => Agent::Dqn(Box::new(RlController::load_or_new(&net))),
        (false, false) => Agent::Dqn(Box::new(RlController::with_hidden(&net, &hidden, 42 + seed()))),
        (true, true) => Agent::Hybrid(Box::new(Hybrid::load_or_new(&net))),
        (true, false) => Agent::Hybrid(Box::new(Hybrid::with_hidden(&net, &hidden, 42 + seed()))),
    };
    if let Agent::Dqn(a) = &mut agent {
        a.set_expert_predictive(&net);
    }
    // Whether the weights file now holds this run's result.
    let mut saved_any = false;
    if let Agent::Hybrid(h) = &mut agent {
        // The base tuning is the exploration expert and the score to beat, also when
        // resuming; save it now so the hybrid falls back to it if no network does better.
        // When no tuning met the pedestrian limit, its score is infinite and isn't saved.
        h.base_action = base.1;
        best = base.0;
        std::fs::create_dir_all("weights").ok();
        h.save(&hybrid::weights_path(kind.key()), Some(base.0), false).ok();
        saved_any = true;
    }
    agent.set(true, 0.0, 0.0);
    if resume {
        agent.set_lr(1.5e-4);
        let (score, back) = evaluate(&net, agent, &val, secs);
        agent = back;
        println!("[{}:{}] resumed from saved weights, validation pd {:.1}", kind.key(), agent.tag(), score.0);
        if score.0 < best && score.1 <= ped_limit {
            best = score.0;
            agent.save(kind, score.0);
            saved_any = true;
        }
    }
    let first_ep = *agent.episodes();
    let t0 = Instant::now();
    for ep in 0..episodes {
        let frac = ep as f64 / episodes as f64;
        let (eps, expert) = if resume {
            ((0.06 * (1.0 - frac)).max(0.02), 0.0)
        } else {
            ((0.25 * (1.0 - frac / 0.7)).max(0.03), (0.6 * (1.0 - frac / 0.45)).max(0.0))
        };
        agent.set(true, eps, expert);
        *agent.episodes() = first_ep + ep + 1;
        // A share of episodes near the layout's design demand, the rest randomized; some
        // in bad weather or with failed loop detectors.
        let demand = if rng.chance(0.35) {
            let mut d = DemandConfig::default_for(&net);
            d.scale = rng.range(0.8, 1.2);
            d
        } else {
            random_demand(&net, &mut rng)
        };
        let mut sensors = SensorConfig::default();
        if rng.chance(0.25) {
            sensors.conditions = Conditions::ALL[1 + rng.below(4)];
        }
        if rng.chance(0.1) {
            sensors.loop_fault = 0.25;
        }
        agent.new_episode(&net);
        let (r, back) = agent.run(&net, &Scenario { demand, sensors, seed: 100 + ep }, secs);
        agent = back;
        if (ep + 1) % 10 == 0 || ep + 1 == episodes {
            let (score, back) = evaluate(&net, agent, &val, secs);
            agent = back;
            let saved = if score.0 < best && score.1 <= ped_limit {
                best = score.0;
                agent.save(kind, score.0);
                saved_any = true;
                " (saved)"
            } else {
                ""
            };
            println!(
                "[{}:{}] ep {:4}  train pd {:6.1}  val pd {:6.1} ped {:5.1}  best {:6.1}  eps {:.2} expert {:.2}  loss {:.4}  {:.0}s{}",
                kind.key(),
                agent.tag(),
                ep + 1,
                r.0,
                score.0,
                score.1,
                best,
                eps,
                expert,
                agent.loss(),
                t0.elapsed().as_secs_f64(),
                saved
            );
        }
    }
    println!(
        "[{}:{}] done. best validation person-delay {:.1} s (fixed {:.1}, max-pressure {:.1}, predictive {:.1})",
        kind.key(),
        agent.tag(),
        best,
        base_fixed.0,
        base_mp.0,
        base_pred.0
    );
    report_test(kind, &net, hybrid, base.1, saved_any, secs);
}

/// Score the saved weights, and the baselines, on the held-out test set.
fn report_test(kind: LayoutKind, net: &Network, hybrid: bool, base_action: usize, saved_any: bool, secs: f64) {
    let test = test_set(net);
    let fmt = |r: (f64, f64)| format!("{:.1} / {:.1}", r.0, r.1);
    let mut line = format!(
        "[{}:{}] TEST person delay / pedestrian wait (s): fixed {}  max-pressure {}  predictive {}",
        kind.key(),
        if hybrid { "hybrid" } else { "dqn" },
        fmt(baseline(net, CtrlKind::Fixed, &test, secs)),
        fmt(baseline(net, CtrlKind::MaxPressure, &test, secs)),
        fmt(baseline(net, CtrlKind::Predictive, &test, secs))
    );
    if hybrid {
        line += &format!("  static {}", fmt(evaluate(net, static_hybrid(net, base_action), &test, secs).0));
    }
    let agent = match (hybrid, saved_any) {
        (_, false) => {
            println!("{line}  agent: no checkpoint met the pedestrian limit, weights not written");
            return;
        }
        (true, true) => {
            let h = Hybrid::load_or_new(net);
            if h.has_model {
                Agent::Hybrid(Box::new(h))
            } else {
                line += "  (no network kept)";
                static_hybrid(net, h.base_action)
            }
        }
        (false, true) => Agent::Dqn(Box::new(RlController::load_or_new(net))),
    };
    println!("{line}  agent {}", fmt(evaluate(net, agent, &test, secs).0));
}

fn resume_salt() -> u64 {
    if std::env::var_os("RESUME").is_some() { 99 } else { 0 }
}

fn seed() -> u64 {
    std::env::var("SEED").ok().and_then(|s| s.parse().ok()).unwrap_or(0)
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let which = args.get(1).map(|s| s.as_str()).unwrap_or("all");
    let episodes: u64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(200);
    let secs: f64 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(900.0);
    let what = args.get(4).cloned().unwrap_or_else(|| "both".into());
    let agents: Vec<&'static str> = match what.as_str() {
        "dqn" => vec!["dqn"],
        "hybrid" => vec!["hybrid"],
        _ => vec!["dqn", "hybrid"],
    };
    let kinds: Vec<LayoutKind> = LayoutKind::ALL
        .iter()
        .copied()
        .filter(|k| k.signalized() && (which == "all" || k.key() == which))
        .collect();
    let hs: Vec<_> = kinds
        .into_iter()
        .flat_map(|k| agents.iter().map(move |&a| std::thread::spawn(move || train(k, episodes, secs, a))))
        .collect();
    for h in hs {
        h.join().unwrap();
    }
}
