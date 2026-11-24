// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Emergency vehicles, with and without preemption: one sent straight through at a
//! moment that varies from run to run, and one sent in behind a queue at a red, 30 s after
//! 20 cars for each lane it may use, from the start of the red.
use intersection_lab::control::CtrlKind;
use intersection_lab::sim::network::{LayoutKind, Turn};
use intersection_lab::sim::vtypes::VKind;
use intersection_lab::sim::{ControlMode, Sim};

/// How many of six emergency vehicles got through, and their mean delay (s).
fn mean_delay(pre: bool, queued: bool) -> (usize, f64) {
    let mut total = 0.0;
    let mut n = 0;
    for k in 0..6 {
        let mut sim = Sim::new(LayoutKind::Arterial4, ControlMode::Signal, CtrlKind::Fixed, 300 + k);
        sim.preemption = pre;
        sim.run_for(200.0 + 17.0 * k as f64);
        let leg = (k % 4) as usize;
        if queued {
            let m = sim.net.movement_for(leg, Turn::T).unwrap();
            let green = |sim: &Sim| sim.signal.status(&sim.net, m).is_green();
            while !green(&sim) {
                sim.step();
            }
            while green(&sim) {
                sim.step();
            }
            let lanes = (sim.net.legs[leg].in_lanes.iter())
                .filter(|&&lane| sim.net.links[lane].lane_moves.contains(&m))
                .count();
            for _ in 0..20 * lanes {
                sim.spawn_manual(leg, VKind::Car, Turn::T);
            }
            sim.run_for(30.0);
        }
        sim.spawn_manual(leg, VKind::Emergency, Turn::T);
        sim.run_for(if queued { 300.0 } else { 120.0 });
        for d in sim.stats.done.iter().filter(|d| d.kind == VKind::Emergency) {
            total += d.delay;
            n += 1;
        }
    }
    (n, total / n.max(1) as f64)
}

fn main() {
    for queued in [false, true] {
        for pre in [false, true] {
            let (n, delay) = mean_delay(pre, queued);
            let behind = if queued { "behind a queue, " } else { "" };
            println!("{behind}preemption={pre}: {n} emergency vehicles, avg delay {delay:.1} s");
        }
    }
}
