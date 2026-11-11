// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Traffic intersection simulator: microscopic simulation, sensor models,
//! signal controllers (fixed, actuated, max-pressure, predictive MPC, deep RL).

pub mod control;
pub mod gui;
pub mod rl;
pub mod sim;

use control::CtrlKind;
use sim::demand::DemandConfig;
use sim::network::{LayoutKind, Network};
use sim::sensors::SensorConfig;
use sim::stats::Summary;
use sim::{ControlMode, Sim};

/// Run a headless simulation (see [`scenario`]) for `warmup + duration` seconds and
/// summarize everything that exited after warm-up, with the number of overlaps.
pub fn run_scenario(mut sim: Sim, warmup: f64, duration: f64) -> (Summary, u64) {
    sim.run_for(warmup + duration);
    let s = sim.stats.summary(warmup, sim.t, &sim.vehicles, &sim.queues);
    (s, sim.collisions)
}

/// A fresh headless simulation of a scenario, ready to step.
pub fn scenario(
    kind: LayoutKind,
    mode: ControlMode,
    ctrl: CtrlKind,
    demand: &DemandConfig,
    sensors: &SensorConfig,
    seed: u64,
) -> Sim {
    let net = Network::build(kind);
    let mut sim = Sim::with_demand(net, mode, ctrl, demand.clone(), seed);
    sim.sensors = sensors.clone();
    sim
}
