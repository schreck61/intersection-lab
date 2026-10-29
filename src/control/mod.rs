// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Signal controllers. Each one sees only the `SensorView` (plus "historical" demand
//! for fixed-time planning) and asks the signal state machine for a stage.

pub mod actuated;
pub mod fixed;
pub mod hybrid;
pub mod maxpressure;
pub mod predictive;
pub mod rl;
pub mod weights;

use crate::sim::demand::DemandConfig;
use crate::sim::network::{MStat, Network};
use crate::sim::sensors::{SensorConfig, SensorView};
use crate::sim::signal::SignalState;
use serde::{Deserialize, Serialize};

pub struct Ctx<'a> {
    pub net: &'a Network,
    pub view: &'a SensorView,
    pub sig: &'a SignalState,
    pub demand: &'a DemandConfig,
    pub sensors: &'a SensorConfig,
    pub t: f64,
    pub dt: f64,
    /// The signal state machine would honour a switch request now.
    pub can_switch: bool,
    /// Person-weighted delay accumulated during the last step (RL reward signal).
    pub reward_delay: f64,
}

#[derive(Clone, Debug, Default)]
pub struct Brain {
    pub title: String,
    /// (label, value, highlighted)
    pub bars: Vec<(String, f64, bool)>,
    pub lower_is_better: bool,
    pub notes: Vec<String>,
}

pub trait Controller: Send {
    fn name(&self) -> &'static str;
    /// Called every simulation step. Return the stage to switch to (None = hold).
    fn step(&mut self, ctx: &Ctx) -> Option<usize>;
    fn brain(&self) -> Brain {
        Brain::default()
    }
    fn as_rl(&mut self) -> Option<&mut rl::RlController> {
        None
    }
    fn into_rl(self: Box<Self>) -> Option<Box<rl::RlController>> {
        None
    }
    fn as_hybrid(&mut self) -> Option<&mut hybrid::Hybrid> {
        None
    }
    fn as_fixed(&mut self) -> Option<&mut fixed::FixedTime> {
        None
    }
    fn into_hybrid(self: Box<Self>) -> Option<Box<hybrid::Hybrid>> {
        None
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum CtrlKind {
    Fixed,
    Actuated,
    MaxPressure,
    Predictive,
    DeepRl,
    Hybrid,
}

impl CtrlKind {
    pub const ALL: [CtrlKind; 6] =
        [CtrlKind::Fixed, CtrlKind::Actuated, CtrlKind::MaxPressure, CtrlKind::Predictive, CtrlKind::DeepRl, CtrlKind::Hybrid];
    pub fn name(self) -> &'static str {
        match self {
            CtrlKind::Fixed => "Fixed-time (Webster)",
            CtrlKind::Actuated => "Actuated (NEMA gap-out)",
            CtrlKind::MaxPressure => "Max-Pressure",
            CtrlKind::Predictive => "Predictive (schedule-driven MPC)",
            CtrlKind::DeepRl => "AI: Deep RL (Double DQN)",
            CtrlKind::Hybrid => "AI: Hybrid (RL-tuned predictive)",
        }
    }
    pub fn short(self) -> &'static str {
        match self {
            CtrlKind::Fixed => "Fixed",
            CtrlKind::Actuated => "Actuated",
            CtrlKind::MaxPressure => "MaxPressure",
            CtrlKind::Predictive => "Predictive",
            CtrlKind::DeepRl => "DeepRL",
            CtrlKind::Hybrid => "Hybrid",
        }
    }
    pub fn adaptive(self) -> bool {
        matches!(self, CtrlKind::MaxPressure | CtrlKind::Predictive | CtrlKind::DeepRl | CtrlKind::Hybrid)
    }
    pub fn description(self) -> &'static str {
        match self {
            CtrlKind::Fixed => "Pre-timed cycle. Cycle length and splits from Webster's formula using historical demand. Uses no sensors.",
            CtrlKind::Actuated => "Fully actuated: min green, extends on loop actuations within the passage time, gaps out or maxes out, skips stages with no calls. Bus priority extension when a camera/CV sees a bus.",
            CtrlKind::MaxPressure => "Varaiya's throughput-optimal max-pressure: serves the stage with the largest upstream-minus-downstream queue pressure. Needs queue counts (camera).",
            CtrlKind::Predictive => "Rolling-horizon, schedule-driven control in the spirit of SURTRAC: predicts every detected vehicle's arrival, simulates candidate signal schedules 90 s ahead and picks the one minimising person-delay. Benefits most from long-range radar / connected vehicles.",
            CtrlKind::DeepRl => "Double DQN neural network trained by reinforcement learning (reward = negative person delay) on randomised demand. Observes queues, predicted arrivals, waits and pedestrian calls. Can keep learning online.",
            CtrlKind::Hybrid => "The predictive planner makes every signal decision; a reinforcement-learning agent re-tunes it every 10 s (fairness weight, hold/switch bias, max green) from the traffic it sees. Keeps the planner's safety; learns when to deviate from its defaults.",
        }
    }
}

pub fn make(kind: CtrlKind, net: &Network, demand: &DemandConfig) -> Box<dyn Controller> {
    match kind {
        CtrlKind::Fixed => Box::new(fixed::FixedTime::new(net, demand)),
        CtrlKind::Actuated => Box::new(actuated::Actuated::new(net)),
        CtrlKind::MaxPressure => Box::new(maxpressure::MaxPressure::new(net)),
        CtrlKind::Predictive => Box::new(predictive::Predictive::new(net)),
        CtrlKind::DeepRl => Box::new(rl::RlController::load_or_new(net)),
        CtrlKind::Hybrid => Box::new(hybrid::Hybrid::load_or_new(net)),
    }
}

pub fn stat(net: &Network, stage: usize, m: usize) -> Option<MStat> {
    net.stages.get(stage).and_then(|s| s.stat[m])
}

/// Number of inbound lanes that can carry movement `m`.
pub fn lanes_for(net: &Network, m: usize) -> f64 {
    net.movements[m].routes.len().max(1) as f64
}

/// Does switching from stage `a` to stage `b` require yellow + all-red?
pub fn needs_clearance(net: &Network, a: usize, b: usize) -> bool {
    let (sa, sb) = (&net.stages[a].stat, &net.stages[b].stat);
    (0..sa.len()).any(|m| matches!((sa[m], sb[m]), (Some(_), None) | (Some(MStat::G), Some(MStat::P))))
}
