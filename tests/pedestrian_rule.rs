// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Regression tests for pedestrian service: Deep RL's safety shield must give the next
//! walk to anyone who has waited longer than `PED_MAX_WAIT`, and the training reward must
//! count pedestrians waiting to cross.

use intersection_lab::control::rl::{PED_MAX_WAIT, RlController};
use intersection_lab::control::{CtrlKind, Ctx};
use intersection_lab::sim::demand::DemandConfig;
use intersection_lab::sim::network::{LayoutKind, Network};
use intersection_lab::sim::{ControlMode, PedState, Sim};

/// Signalized layouts, each with a Deep RL simulation that has run long enough to have
/// traffic on every approach.
fn signalized_sims() -> Vec<Sim> {
    LayoutKind::ALL
        .into_iter()
        .filter(|&k| ControlMode::options(k).contains(&ControlMode::Signal))
        .map(|k| {
            let mut sim = Sim::new(k, ControlMode::Signal, CtrlKind::DeepRl, 5);
            sim.run_for(120.0);
            sim
        })
        .collect()
}

/// The shield's mask for the simulation's current state, with the given longest
/// pedestrian waits per crosswalk.
fn mask_with_waits(sim: &Sim, waits: &[f64]) -> Vec<bool> {
    let mut view = sim.view.clone();
    view.ped_max_wait = waits.to_vec();
    let ctx = Ctx {
        net: &sim.net,
        view: &view,
        sig: &sim.signal,
        demand: &sim.demand,
        sensors: &sim.sensors,
        t: sim.t,
        dt: sim.dt,
        can_switch: true,
        reward_delay: 0.0,
    };
    RlController::shield(&ctx)
}

/// The stages that walk crosswalk `cw`.
fn walking(net: &Network, cw: usize) -> Vec<bool> {
    net.stages.iter().map(|s| s.walk.contains(&cw)).collect()
}

#[test]
fn shield_serves_a_pedestrian_who_waited_too_long() {
    let mut checked = 0;
    for sim in signalized_sims() {
        let net = &sim.net;
        let n = net.crosswalks.len();
        let cur = sim.signal.stage;
        for cw in 0..n {
            let serve = walking(net, cw);
            if net.stages[cur].walk.contains(&cw) || !serve.iter().any(|&s| s) {
                continue;
            }
            let mut waits = vec![0.0; n];
            waits[cw] = PED_MAX_WAIT + 5.0;
            assert_eq!(mask_with_waits(&sim, &waits), serve, "{:?}: crosswalk {cw} waited too long but was not served next", net.kind);
            checked += 1;
        }
    }
    assert!(checked > 0, "no crosswalk was left unserved by the current stage");
}

#[test]
fn shield_leaves_shorter_waits_to_the_controller() {
    for sim in signalized_sims() {
        let n = sim.net.crosswalks.len();
        let normal = mask_with_waits(&sim, &vec![0.0; n]);
        // Waits up to the limit change nothing.
        assert_eq!(mask_with_waits(&sim, &vec![PED_MAX_WAIT; n]), normal, "{:?}", sim.net.kind);
        // Nor does a long wait at a crosswalk the current stage already walks.
        for &cw in &sim.net.stages[sim.signal.stage].walk {
            let mut waits = vec![0.0; n];
            waits[cw] = PED_MAX_WAIT + 30.0;
            assert_eq!(mask_with_waits(&sim, &waits), normal, "{:?}: crosswalk {cw}", sim.net.kind);
        }
    }
}

#[test]
fn shield_serves_the_longest_wait_first() {
    let mut checked = 0;
    for sim in signalized_sims() {
        let net = &sim.net;
        let n = net.crosswalks.len();
        let cur = sim.signal.stage;
        for a in 0..n {
            for b in 0..n {
                let (sa, sb) = (walking(net, a), walking(net, b));
                // Two late crosswalks that no stage walks together, neither walked now.
                if a == b || sa[cur] || sb[cur] || !sa.contains(&true) || !sb.contains(&true) || sa.iter().zip(&sb).any(|(&x, &y)| x && y) {
                    continue;
                }
                let mut waits = vec![0.0; n];
                waits[a] = PED_MAX_WAIT + 20.0;
                waits[b] = PED_MAX_WAIT + 10.0;
                assert_eq!(mask_with_waits(&sim, &waits), sa, "{:?}: crosswalks {a} and {b}", net.kind);
                checked += 1;
            }
        }
    }
    assert!(checked > 0, "no layout has two crosswalks walked in different stages");
}

#[test]
fn reward_counts_waiting_pedestrians() {
    // With no vehicles, the reward is only the time pedestrians spend waiting to cross.
    let net = Network::build(LayoutKind::Arterial4);
    let mut demand = DemandConfig::default_for(&net);
    demand.rate = [0.0; 4];
    demand.ped_rate = 400.0;
    let mut sim = Sim::with_demand(net, ControlMode::Signal, CtrlKind::Fixed, demand, 3);
    let (mut reward, mut waiting) = (0.0, 0.0);
    while sim.t < 900.0 {
        sim.step();
        assert!(sim.vehicles.is_empty() && sim.queues.iter().all(|q| q.is_empty()));
        reward += sim.last_reward;
        waiting += sim.dt * sim.peds.iter().filter(|p| p.state == PedState::Waiting).count() as f64;
    }
    // The reward is taken mid-step, so pedestrians who start or stop waiting in that step
    // can differ by one step from the end-of-step count.
    assert!(waiting > 1000.0, "too little pedestrian waiting to test: {waiting:.0} s");
    assert!((reward - waiting).abs() < 0.02 * waiting, "reward {reward:.0} s, pedestrian waiting {waiting:.0} s");
}
