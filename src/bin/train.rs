//! Headless Deep-RL training. Trains one Double-DQN per signalised layout on
//! randomised demand (rates, turning shares, arrival pattern, pedestrians), using the
//! predictive controller as a decaying "expert" for exploration, and keeps the
//! weights that score best on fixed evaluation scenarios.
//!
//! usage: train [layout|all] [episodes] [episode-seconds]

use std::time::Instant;
use trafficsim::control::rl::{RlController, weights_path};
use trafficsim::control::{CtrlKind, make};
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
fn evaluate(net: &Network, rl: Box<RlController>, secs: f64) -> (f64, Box<RlController>) {
    let mut rl = rl;
    let (learning, eps) = (rl.learning, rl.epsilon);
    rl.learning = false;
    rl.epsilon = 0.0;
    let mut total = 0.0;
    let n = EVAL_N;
    for k in 0..n {
        let d = eval_demand(net, k);
        rl.new_episode(net);
        let mut sim = Sim::with_demand(net.clone(), ControlMode::Signal, CtrlKind::DeepRl, d, 9000 + k);
        sim.controller = rl;
        sim.run_for(secs);
        total += sim.stats.summary(120.0, sim.t, &sim.vehicles, &sim.queues).person_delay_all;
        let c = std::mem::replace(&mut sim.controller, make(CtrlKind::Fixed, net, &sim.demand));
        rl = c.into_rl().unwrap();
    }
    rl.learning = learning;
    rl.epsilon = eps;
    (total / n as f64, rl)
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

fn train(kind: LayoutKind, episodes: u64, secs: f64) {
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
    let mut rng = Rng::new(kind as u64 * 7777 + 1 + resume_salt());
    let mut best = f64::INFINITY;
    // RESUME=1 fine-tunes the saved weights (lower learning rate, no expert).
    let resume = std::env::var_os("RESUME").is_some();
    let mut rl = Box::new(if resume { RlController::load_or_new(&net) } else { RlController::new(&net, 42) });
    rl.learning = true;
    rl.set_expert_predictive(&net);
    if resume {
        rl.agent.set_lr(1.5e-4);
        let (score, back) = evaluate(&net, rl, secs);
        rl = back;
        best = score;
        println!("[{}] resumed from saved weights, eval pd {:.1}", kind.key(), score);
    }
    let first_ep = rl.episodes;
    let t0 = Instant::now();
    for ep in 0..episodes {
        let frac = ep as f64 / episodes as f64;
        if resume {
            rl.epsilon = (0.06 * (1.0 - frac)).max(0.02);
            rl.expert_prob = 0.0;
        } else {
            rl.epsilon = (0.25 * (1.0 - frac / 0.7)).max(0.03);
            rl.expert_prob = (0.6 * (1.0 - frac / 0.45)).max(0.0);
        }
        rl.episodes = first_ep + ep + 1;
        // A share of episodes near the layout's design demand, the rest randomised.
        let d = if rng.chance(0.35) {
            let mut d = DemandConfig::default_for(&net);
            d.scale = rng.range(0.8, 1.2);
            d
        } else {
            random_demand(&net, &mut rng)
        };
        rl.new_episode(&net);
        let mut sim = Sim::with_demand(net.clone(), ControlMode::Signal, CtrlKind::DeepRl, d, 100 + ep);
        sim.controller = rl;
        sim.run_for(secs);
        let pd = sim.stats.summary(120.0, sim.t, &sim.vehicles, &sim.queues).person_delay_all;
        let c = std::mem::replace(&mut sim.controller, make(CtrlKind::Fixed, &net, &sim.demand));
        rl = c.into_rl().unwrap();
        if (ep + 1) % 10 == 0 || ep + 1 == episodes {
            let (score, back) = evaluate(&net, rl, secs);
            rl = back;
            let saved = if score < best {
                best = score;
                std::fs::create_dir_all("weights").ok();
                rl.save(&weights_path(kind.key())).ok();
                " (saved)"
            } else {
                ""
            };
            println!(
                "[{}] ep {:4}  train pd {:6.1}  eval pd {:6.1}  best {:6.1}  eps {:.2} expert {:.2}  loss {:.4}  {:.0}s{}",
                kind.key(),
                ep + 1,
                pd,
                score,
                best,
                rl.epsilon,
                rl.expert_prob,
                rl.agent.losses.back().copied().unwrap_or(0.0),
                t0.elapsed().as_secs_f64(),
                saved
            );
        }
    }
    println!(
        "[{}] done. best eval person-delay {:.1} s (fixed {:.1}, max-pressure {:.1}, predictive {:.1})",
        kind.key(),
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
    let kinds: Vec<LayoutKind> = LayoutKind::ALL
        .iter()
        .copied()
        .filter(|k| k.signalized() && (which == "all" || k.key() == which))
        .collect();
    let hs: Vec<_> = kinds.into_iter().map(|k| std::thread::spawn(move || train(k, episodes, secs))).collect();
    for h in hs {
        h.join().unwrap();
    }
}
