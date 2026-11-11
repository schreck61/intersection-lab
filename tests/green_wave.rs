// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The corridor's green wave runs every signal fixed-time. Turning it off gives each
//! signal back the controller it ran before, unless a new one was chosen meanwhile.

use intersection_lab::control::CtrlKind;
use intersection_lab::sim::ControlMode;
use intersection_lab::sim::demand::DemandConfig;
use intersection_lab::sim::network::{LayoutKind, Network};
use intersection_lab::sim::world::World;

fn corridor(n: usize) -> World {
    let kind = LayoutKind::Arterial4;
    let mut w = World::corridor(
        n,
        kind,
        ControlMode::Signal,
        CtrlKind::Predictive,
        &DemandConfig::default_for(&Network::build(kind)),
        1,
    );
    for s in &mut w.sims {
        s.check_safety = true;
    }
    w
}

fn controllers(w: &World) -> Vec<CtrlKind> {
    w.sims.iter().map(|s| s.ctrl_kind).collect()
}

#[test]
fn the_green_wave_gives_back_the_controllers() {
    let mut w = corridor(4);
    w.set_controller(2, CtrlKind::Actuated);
    w.set_controller(3, CtrlKind::Fixed);
    w.set_coordination(true);
    assert_eq!(controllers(&w), [CtrlKind::Fixed; 4]);
    assert!(w.sims.iter().all(|s| s.coordination.is_some()));
    assert_eq!(w.controller_before_wave(2), Some(CtrlKind::Actuated));

    // Chosen while the wave runs: kept when it ends.
    w.set_controller(1, CtrlKind::MaxPressure);
    w.set_controller(2, CtrlKind::Fixed);
    assert_eq!(w.controller_before_wave(2), None);
    w.run_for(30.0);
    w.set_coordination(false);
    assert_eq!(
        controllers(&w),
        [CtrlKind::Predictive, CtrlKind::MaxPressure, CtrlKind::Fixed, CtrlKind::Fixed]
    );
    assert!(w.sims.iter().all(|s| s.coordination.is_none()));
    w.run_for(30.0);
    assert_eq!(w.sims.iter().map(|s| s.collisions).sum::<u64>(), 0);

    // A second wave starts from the controllers running now.
    w.set_coordination(true);
    w.set_coordination(false);
    assert_eq!(
        controllers(&w),
        [CtrlKind::Predictive, CtrlKind::MaxPressure, CtrlKind::Fixed, CtrlKind::Fixed]
    );
}

#[test]
fn a_single_intersection_keeps_its_controller() {
    let mut w = corridor(1);
    w.set_coordination(true);
    assert_eq!(controllers(&w), [CtrlKind::Predictive]);
    assert!(w.sims[0].coordination.is_none());
}
