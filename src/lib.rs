//! Traffic intersection simulator: microscopic simulation, sensor models,
//! signal controllers (fixed, actuated, max-pressure, predictive MPC, deep RL).

pub mod control;
pub mod rl;
pub mod gui;
pub mod sim;

use control::CtrlKind;
use sim::demand::DemandConfig;
use sim::network::{LayoutKind, Network};
use sim::sensors::SensorConfig;
use sim::stats::Summary;
use sim::{ControlMode, Sim};

/// Run one headless scenario and summarise everything that exited after warm-up.
pub fn run_scenario(
    kind: LayoutKind,
    mode: ControlMode,
    ctrl: CtrlKind,
    demand: &DemandConfig,
    sensors: &SensorConfig,
    seed: u64,
    warmup: f64,
    duration: f64,
) -> (Summary, u64) {
    let net = Network::build(kind);
    let mut sim = Sim::with_demand(net, mode, ctrl, demand.clone(), seed);
    sim.sensors = sensors.clone();
    sim.run_for(warmup + duration);
    let s = sim.stats.summary(warmup, sim.t, &sim.vehicles, &sim.queues);
    (s, sim.collisions)
}
