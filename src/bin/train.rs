//! Headless reinforcement-learning training, per signalised layout, on randomised
//! demand (rates, turning shares, arrival pattern, pedestrians), keeping the weights
//! that score best on fixed evaluation scenarios.
//!
//! * `dqn`: the Deep RL controller chooses stages; the predictive controller acts as a
//!   decaying "expert" for exploration.
//! * `hybrid`: the agent re-tunes the predictive planner every 10 s; its default
//!   tuning is the decaying expert.
//!
//! usage: train [layout|all] [episodes] [episode-seconds] [dqn|hybrid|both]

use std::time::Instant;
use trafficsim::control::hybrid::{self, Hybrid};
use trafficsim::control::rl::{self, RlController};
use trafficsim::control::{Controller, CtrlKind, make};
use trafficsim::sim::demand::{DemandConfig, Pattern};
use trafficsim::sim::network::{LayoutKind, Network};
use trafficsim::sim::rng::Rng;
use trafficsim::sim::{ControlMode, Sim};

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
    fn save(&self, layout: LayoutKind) {
        std::fs::create_dir_all("weights").ok();
        match self {
            Agent::Dqn(a) => a.save(&rl::weights_path(layout.key())).ok(),
            Agent::Hybrid(a) => a.save(&hybrid::weights_path(layout.key())).ok(),
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
    /// Run one episode with this agent in control.
    fn run(self, net: &Network, d: DemandConfig, seed: u64, secs: f64) -> (f64, Agent) {
        let kind = self.kind();
        let mut sim = Sim::with_demand(net.clone(), ControlMode::Signal, kind, d, seed);
        sim.controller = self.into_controller();
        sim.run_for(secs);
        let pd = sim.stats.summary(120.0, sim.t, &sim.vehicles, &sim.queues).person_delay_all;
        let c = std::mem::replace(&mut sim.controller, make(CtrlKind::Fixed, net, &sim.demand));
        (pd, Agent::from_controller(c, kind))
    }
}

const EVAL_N: u64 = 12;

/// Model-selection scenarios: six at the layout's default demand (different
/// arrival seeds) plus randomised ones, so selection doesn't overfit a few cases.
fn eval_demand(net: &Network, k: u64) -> DemandConfig {
    if k < 6 {
        DemandConfig::default_for(net)
    } else {
        random_demand(net, &mut Rng::new(9000 + k))
    }
}

/// Mean person-delay over fixed evaluation scenarios (greedy policy).
fn evaluate(net: &Network, agent: Agent, secs: f64) -> (f64, Agent) {
    let mut agent = agent;
    let saved = agent.schedule();
    agent.set(false, 0.0, 0.0);
    if let Agent::Hybrid(h) = &mut agent {
        // Greedy evaluation must use the network even before weights are saved.
        h.loaded_from.get_or_insert_with(|| "(training)".into());
    }
    let mut total = 0.0;
    for k in 0..EVAL_N {
        agent.new_episode(net);
        let (pd, back) = agent.run(net, eval_demand(net, k), 9000 + k, secs);
        total += pd;
        agent = back;
    }
    agent.set(saved.0, saved.1, saved.2);
    (total / EVAL_N as f64, agent)
}

fn baseline(net: &Network, kind: CtrlKind, secs: f64) -> f64 {
    let mut total = 0.0;
    for k in 0..EVAL_N {
        let d = eval_demand(net, k);
        let mut sim = Sim::with_demand(net.clone(), ControlMode::Signal, kind, d, 9000 + k);
        sim.run_for(secs);
        total += sim.stats.summary(120.0, sim.t, &sim.vehicles, &sim.queues).person_delay_all;
    }
    total / EVAL_N as f64
}

fn train(kind: LayoutKind, episodes: u64, secs: f64, which: &str) {
    let net = Network::build(kind);
    let base_pred = baseline(&net, CtrlKind::Predictive, secs);
    let base_mp = baseline(&net, CtrlKind::MaxPressure, secs);
    let base_fixed = baseline(&net, CtrlKind::Fixed, secs);
    println!(
        "[{}] eval baselines (person delay s): fixed {:.1}  max-pressure {:.1}  predictive {:.1}",
        kind.key(),
        base_fixed,
        base_mp,
        base_pred
    );
    let hybrid = which == "hybrid";
    if hybrid {
        // The bar the agent must clear to add anything: the best single tuning.
        let scores: Vec<(f64, usize)> = (0..hybrid::N_ACTIONS)
            .map(|a| {
                let mut h = Box::new(Hybrid::new(&net, 1));
                h.fixed_action = Some(a);
                let (s, _) = evaluate(&net, Agent::Hybrid(h), secs);
                (s, a)
            })
            .collect();
        let best_static = scores.iter().copied().fold((f64::INFINITY, 0), |a, b| if b.0 < a.0 { b } else { a });
        println!(
            "[{}:hybrid] best static tuning: {} -> {:.1} s (default tuning {:.1} s)",
            kind.key(),
            hybrid::preset_name(best_static.1),
            best_static.0,
            scores[hybrid::DEFAULT_ACTION].0
        );
    }
    let mut rng = Rng::new(kind as u64 * 7777 + 1 + resume_salt() + if hybrid { 31 } else { 0 });
    let mut best = f64::INFINITY;
    // RESUME=1 fine-tunes the saved weights (lower learning rate, no expert).
    let resume = std::env::var_os("RESUME").is_some();
    let mut agent = match (hybrid, resume) {
        (false, true) => Agent::Dqn(Box::new(RlController::load_or_new(&net))),
        (false, false) => Agent::Dqn(Box::new(RlController::new(&net, 42))),
        (true, true) => Agent::Hybrid(Box::new(Hybrid::load_or_new(&net))),
        (true, false) => Agent::Hybrid(Box::new(Hybrid::new(&net, 42))),
    };
    if let Agent::Dqn(a) = &mut agent {
        a.set_expert_predictive(&net);
    }
    agent.set(true, 0.0, 0.0);
    if resume {
        agent.set_lr(1.5e-4);
        let (score, back) = evaluate(&net, agent, secs);
        agent = back;
        best = score;
        println!("[{}:{}] resumed from saved weights, eval pd {:.1}", kind.key(), agent.tag(), score);
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
        // A share of episodes near the layout's design demand, the rest randomised.
        let d = if rng.chance(0.35) {
            let mut d = DemandConfig::default_for(&net);
            d.scale = rng.range(0.8, 1.2);
            d
        } else {
            random_demand(&net, &mut rng)
        };
        agent.new_episode(&net);
        let (pd, back) = agent.run(&net, d, 100 + ep, secs);
        agent = back;
        if (ep + 1) % 10 == 0 || ep + 1 == episodes {
            let (score, back) = evaluate(&net, agent, secs);
            agent = back;
            let saved = if score < best {
                best = score;
                agent.save(kind);
                " (saved)"
            } else {
                ""
            };
            println!(
                "[{}:{}] ep {:4}  train pd {:6.1}  eval pd {:6.1}  best {:6.1}  eps {:.2} expert {:.2}  loss {:.4}  {:.0}s{}",
                kind.key(),
                agent.tag(),
                ep + 1,
                pd,
                score,
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
        "[{}:{}] done. best eval person-delay {:.1} s (fixed {:.1}, max-pressure {:.1}, predictive {:.1})",
        kind.key(),
        agent.tag(),
        best,
        base_fixed,
        base_mp,
        base_pred
    );
}

fn resume_salt() -> u64 {
    if std::env::var_os("RESUME").is_some() { 99 } else { 0 }
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
