// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Loading trained weights. The app reads weights files from `weights/` in the working
//! directory or from the per-user data folder, so their contents can't be trusted. Each
//! input is tried as both kinds of weights file for the tee intersection, and whatever
//! the loaders accept must survive a training update and a short simulation.

#![no_main]

use intersection_lab::control::hybrid::Hybrid;
use intersection_lab::control::rl::RlController;
use intersection_lab::control::{Controller, CtrlKind};
use intersection_lab::rl::dqn::{Dqn, Transition};
use intersection_lab::sim::network::{LayoutKind, Network};
use intersection_lab::sim::{ControlMode, Sim};
use libfuzzer_sys::fuzz_target;
use std::sync::LazyLock;

static NET: LazyLock<Network> = LazyLock::new(|| Network::build(LayoutKind::Tee));

fuzz_target!(|data: &[u8]| {
    let Ok(txt) = std::str::from_utf8(data) else { return };
    if let Some(mut c) = RlController::from_saved(&NET, txt) {
        train(&mut c.agent);
        run(Box::new(c), CtrlKind::DeepRl);
    }
    if let Some(mut h) = Hybrid::from_saved(&NET, txt) {
        if h.has_model {
            train(&mut h.agent);
        }
        run(Box::new(h), CtrlKind::Hybrid);
    }
});

/// One training update, as online learning makes once its replay memory fills.
fn train(agent: &mut Dqn) {
    let (n_in, n_out) = (agent.online.n_in(), agent.online.n_out());
    let t = Transition { s: vec![1.0; n_in], a: n_out - 1, r: -1.0, s2: vec![0.5; n_in], disc: 0.9, mask2: vec![true; n_out] };
    for _ in 0..1000 {
        agent.push(t.clone());
    }
    agent.train_step();
}

/// Twenty seconds of the tee intersection under `ctrl`, then the explanation the app
/// shows beside the simulation.
fn run(ctrl: Box<dyn Controller>, kind: CtrlKind) {
    let mut sim = Sim::new(LayoutKind::Tee, ControlMode::Signal, CtrlKind::Fixed, 1);
    sim.controller = ctrl;
    sim.ctrl_kind = kind;
    sim.run_for(20.0);
    sim.controller.brain();
}
