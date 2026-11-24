// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Gap acceptance: the critical and follow-up headways of the drivers who must yield,
//! measured the way a field study measures them (see `sim::gaps`), next to the Highway
//! Capacity Manual's.
//!
//! ```text
//! cargo run --release --example gaps -- [seeds] [minutes]
//! ```
//!
//! * seeds: the runs on each layout, seeds 1 and up (default 8)
//! * minutes: how long each run lasts (default 60)
//!
//! The runs have the default traffic and nobody crossing, so no driver waits for a
//! pedestrian. The two-way stop runs on the T-intersection and on major/minor, both with
//! four-lane major roads, and the fixed-time signal on every layout with permissive left
//! turns. For each movement, with cars and heavy vehicles (trucks and buses) apart, and
//! the minor street's permissive left turns apart from the major road's, the table gives
//! the drivers followed, their critical headway by maximum likelihood, the mean
//! follow-up headway of those that went in the same gap as the vehicle ahead of them,
//! and the HCM's values for the same movement on a four-lane major road (the HCM gives
//! none for heavy vehicles turning left on a permissive indication). Under each layout
//! it counts the vehicles with the right of way that braked harder than 3 m/s² for a
//! vehicle that judges gaps, and the hardest any of them braked.

use intersection_lab::control::CtrlKind;
use intersection_lab::sim::demand::DemandConfig;
use intersection_lab::sim::gaps::{Choice, GapStudy, critical_headway};
use intersection_lab::sim::network::{LayoutKind, Network, Turn};
use intersection_lab::sim::signal::Sig;
use intersection_lab::sim::{ControlMode, Reason, Sim};
use std::collections::{HashMap, HashSet};

/// A movement group: what it is, which movements belong to it, and the HCM's critical
/// and follow-up headways for cars and for heavy vehicles.
struct Group {
    name: &'static str,
    of: fn(&Network, usize) -> bool,
    hcm: [Option<(f64, f64)>; 2],
}

fn major_left(net: &Network, m: usize) -> bool {
    let mv = &net.movements[m];
    !net.legs[mv.leg].minor && mv.turn == Turn::L
}

fn minor(net: &Network, m: usize, turn: Turn) -> bool {
    let mv = &net.movements[m];
    net.legs[mv.leg].minor && mv.turn == turn
}

/// The groups for a layout under a control mode.
fn groups(kind: LayoutKind, mode: ControlMode) -> Vec<Group> {
    if mode == ControlMode::Signal {
        return vec![
            Group {
                name: "permissive left turn",
                of: |net, m| !minor(net, m, Turn::L) && net.movements[m].turn == Turn::L,
                hcm: [Some((4.5, 2.5)), None],
            },
            Group {
                name: "permissive left turn from the minor street",
                of: |net, m| minor(net, m, Turn::L),
                hcm: [Some((4.5, 2.5)), None],
            },
        ];
    }
    // At a T-intersection the minor street's left turn needs 0.7 s less.
    let left = if kind == LayoutKind::Tee { 6.8 } else { 7.5 };
    vec![
        Group {
            name: "left turn from the major road",
            of: major_left,
            hcm: [Some((4.1, 2.2)), Some((6.1, 3.2))],
        },
        Group {
            name: "right turn from the minor street",
            of: |net, m| minor(net, m, Turn::R),
            hcm: [Some((6.9, 3.3)), Some((8.9, 4.3))],
        },
        Group {
            name: "through the minor street",
            of: |net, m| minor(net, m, Turn::T),
            hcm: [Some((6.5, 4.0)), Some((8.5, 5.0))],
        },
        Group {
            name: "left turn from the minor street",
            of: |net, m| minor(net, m, Turn::L),
            hcm: [Some((left, 3.5)), Some((left + 2.0, 4.5))],
        },
    ]
}

/// Whether movement `m` judges gaps now.
fn judges_gaps(sim: &Sim, m: usize) -> bool {
    let mv = &sim.net.movements[m];
    match sim.mode {
        ControlMode::TwoWayStop => sim.net.legs[mv.leg].minor || mv.turn == Turn::L,
        _ => {
            let sig = sim.signal.status(&sim.net, m);
            sig == Sig::P || (mv.turn == Turn::R && sig == Sig::R)
        }
    }
}

/// What one run gives: the choices, and the vehicles with the right of way that braked
/// harder than 3 m/s² for one that judges gaps, with the hardest braking.
fn run(kind: LayoutKind, mode: ControlMode, seed: u64, minutes: f64) -> (Vec<Choice>, usize, f64) {
    let net = Network::build(kind);
    let demand = DemandConfig::default_for(&net);
    let mut sim = Sim::with_demand(net, mode, CtrlKind::Fixed, demand, seed);
    sim.demand.ped_rate = 0.0;
    let mut study = GapStudy::new();
    let mut hard: HashSet<u64> = HashSet::new();
    let mut hardest: f64 = 0.0;
    while sim.t < minutes * 60.0 {
        sim.step();
        study.observe(&sim);
        let ids: HashMap<u64, usize> = sim.vehicles.iter().enumerate().map(|(i, v)| (v.id, i)).collect();
        for v in &sim.vehicles {
            if v.reason != Reason::Yielding || v.a > -3.0 || judges_gaps(&sim, v.movement) {
                continue;
            }
            if let Some(&j) = v.yield_to.and_then(|id| ids.get(&id))
                && judges_gaps(&sim, sim.vehicles[j].movement)
            {
                hard.insert(v.id);
                hardest = hardest.min(v.a);
            }
        }
    }
    (study.choices, hard.len(), hardest)
}

fn main() {
    let arg = |i: usize| std::env::args().nth(i).and_then(|s| s.parse::<f64>().ok());
    let seeds = arg(1).unwrap_or(8.0) as u64;
    let minutes = arg(2).unwrap_or(60.0);
    let mut cases =
        vec![(LayoutKind::Tee, ControlMode::TwoWayStop), (LayoutKind::MajorMinor4, ControlMode::TwoWayStop)];
    for kind in [LayoutKind::Arterial4, LayoutKind::MajorMinor4, LayoutKind::Town4, LayoutKind::Tee] {
        cases.push((kind, ControlMode::Signal));
    }
    println!("{seeds} runs of {minutes} min on each layout, without pedestrians\n");
    for (kind, mode) in cases {
        let runs: Vec<_> =
            (1..=seeds).map(|seed| std::thread::spawn(move || run(kind, mode, seed, minutes))).collect();
        let runs: Vec<_> = runs.into_iter().map(|h| h.join().unwrap()).collect();
        let net = Network::build(kind);
        let mode_name = if mode == ControlMode::Signal { "fixed-time signal" } else { "two-way stop" };
        println!("{}, {mode_name}", kind.key());
        println!(
            "  {:44} {:>5} {:>6} {:>14} {:>15}",
            "movement", "", "n", "critical (HCM)", "follow-up (HCM)"
        );
        for g in groups(kind, mode) {
            for (k, heavy) in [false, true].into_iter().enumerate() {
                let cs: Vec<&Choice> = (runs.iter())
                    .flat_map(|r| r.0.iter())
                    .filter(|c| (g.of)(&net, c.movement) && c.heavy() == heavy)
                    .collect();
                if cs.is_empty() {
                    continue;
                }
                let fmt = |x: Option<f64>| x.map_or("-".to_string(), |x| format!("{x:.1}"));
                let tc = critical_headway(cs.iter().copied());
                let fu: Vec<f64> = cs.iter().filter_map(|c| c.follow_up).collect();
                let fu = (fu.len() >= 10).then(|| fu.iter().sum::<f64>() / fu.len() as f64);
                let hcm = g.hcm[k];
                println!(
                    "  {:44} {:>5} {:>6} {:>14} {:>15}",
                    if heavy { "" } else { g.name },
                    if heavy { "heavy" } else { "car" },
                    cs.len(),
                    format!("{} ({})", fmt(tc), fmt(hcm.map(|h| h.0))),
                    format!("{} ({})", fmt(fu), fmt(hcm.map(|h| h.1))),
                );
            }
        }
        let hard: usize = runs.iter().map(|r| r.1).sum();
        let hardest = runs.iter().map(|r| r.2).fold(0.0, f64::min).abs();
        println!(
            "  braked harder than 3 m/s² for a vehicle judging gaps: {hard} (hardest {hardest:.1} m/s²)\n"
        );
    }
}
