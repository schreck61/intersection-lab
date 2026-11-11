// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Regression tests: every layout/control combination must run collision-free and
//! serve (nearly) all demand at default volumes.

use intersection_lab::control::CtrlKind;
use intersection_lab::sim::network::LayoutKind;
use intersection_lab::sim::{ControlMode, Sim};

mod common;

#[test]
fn all_scenarios_safe_and_flowing() {
    let mut handles = vec![];
    for kind in LayoutKind::ALL {
        for mode in ControlMode::options(kind) {
            let ctrls: Vec<CtrlKind> =
                if mode == ControlMode::Signal { CtrlKind::ALL.to_vec() } else { vec![CtrlKind::Fixed] };
            for c in ctrls {
                handles.push(common::spawn(move || {
                    let mut sim = Sim::new(kind, mode, c, 77);
                    sim.check_safety = true;
                    sim.run_for(900.0);
                    let s = sim.stats.summary(0.0, sim.t, &sim.vehicles, &sim.queues);
                    let served = sim.stats.done.len() as f64 / sim.stats.generated.max(1) as f64;
                    // Stop control is legitimately over capacity on busy roads (the arterial
                    // carries ~2,500 veh/h, above all-way-stop capacity).
                    let over = mode == ControlMode::TwoWayStop
                        || (mode == ControlMode::AllWayStop && kind == LayoutKind::Arterial4);
                    let need = if over { 0.65 } else { 0.8 };
                    (
                        format!("{} / {} / {}", kind.key(), mode.name(), c.short()),
                        sim.collisions,
                        served,
                        s.avg_delay,
                        need,
                    )
                }));
            }
        }
    }
    let mut failures = vec![];
    for h in handles {
        let (name, coll, served, delay, need) = h.join().unwrap();
        if coll > 0 {
            failures.push(format!("{name}: {coll} overlap ticks"));
        }
        if served < need {
            failures
                .push(format!("{name}: only {:.0}% of arrivals served (delay {delay:.0}s)", served * 100.0));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn identical_arrivals_across_controllers() {
    let generated = |c| {
        let mut sim = Sim::new(LayoutKind::Arterial4, ControlMode::Signal, c, 5);
        sim.run_for(300.0);
        sim.stats.generated
    };
    assert_eq!(generated(CtrlKind::Fixed), generated(CtrlKind::MaxPressure));
}

/// Busy arterials used to lock up now and then: vehicles caught in the box by a phase
/// change, and permissive left-turners waiting for a gap, each stopped across another's
/// path, or across a path whose light was red but turned green while they waited. These
/// seeds all gridlocked before that was fixed.
#[test]
fn no_gridlock_when_oversaturated() {
    use intersection_lab::sim::demand::DemandConfig;
    use intersection_lab::sim::network::Network;
    use std::collections::HashMap;

    let cases = [
        (CtrlKind::Hybrid, 2, 1.25, 2400.0),
        (CtrlKind::Predictive, 11, 1.4, 2400.0),
        (CtrlKind::MaxPressure, 10, 1.4, 2400.0),
        (CtrlKind::MaxPressure, 2, 1.25, 3800.0),
    ];
    let handles: Vec<_> = cases
        .into_iter()
        .map(|(c, seed, scale, until)| {
            common::spawn(move || {
                let net = Network::build(LayoutKind::Arterial4);
                let mut demand = DemandConfig::default_for(&net);
                demand.scale = scale;
                let mut sim = Sim::with_demand(net, ControlMode::Signal, c, demand, seed);
                // How long each vehicle inside the intersection has been standing still.
                let mut still: HashMap<u64, f64> = HashMap::new();
                while sim.t < until {
                    sim.step();
                    for v in &sim.vehicles {
                        let t = still.entry(v.id).or_insert(0.0);
                        *t = if v.v < 0.05 && sim.net.links[v.link()].kind.in_box() {
                            *t + sim.dt
                        } else {
                            0.0
                        };
                        if *t > 300.0 {
                            return Some(format!(
                                "{} seed {seed} ×{scale}: vehicle {} stuck in the box at t={:.0}",
                                c.short(),
                                v.id,
                                sim.t
                            ));
                        }
                    }
                }
                None
            })
        })
        .collect();
    let failures: Vec<String> = handles.into_iter().filter_map(|h| h.join().unwrap()).collect();
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// A left-turner clearing the box and a car going through on the late yellow used to
/// commit to the same crossing in the same instant; the slower one now yields.
/// These seeds all collided before that was fixed.
#[test]
fn no_collision_between_late_yellow_and_clearing_turn() {
    use intersection_lab::sim::demand::DemandConfig;
    use intersection_lab::sim::network::Network;

    let handles: Vec<_> = [(1, 5400.0), (3, 9900.0), (4, 7700.0)]
        .into_iter()
        .map(|(seed, until)| {
            common::spawn(move || {
                let net = Network::build(LayoutKind::Tee);
                let mut demand = DemandConfig::default_for(&net);
                demand.scale = 1.25;
                let mut sim = Sim::with_demand(net, ControlMode::Signal, CtrlKind::Predictive, demand, seed);
                sim.check_safety = true;
                sim.run_for(until);
                (seed, sim.collisions, sim.collision_log.first().cloned())
            })
        })
        .collect();
    let failures: Vec<String> = handles
        .into_iter()
        .map(|h| h.join().unwrap())
        .filter(|r| r.1 > 0)
        .map(|(seed, n, first)| {
            format!("seed {seed}: {n} overlap ticks, first: {}", first.unwrap_or_default())
        })
        .collect();
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
