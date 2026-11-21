// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Regression tests: every layout, control mode and automatic controller must run
//! collision-free and serve (nearly) all demand at default volumes, and a person
//! choosing stages at random must never cut an interval short.

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

/// A person at the controls can slow traffic but never make it unsafe. A "person" who
/// chooses stages at random moments, often during a change or while a minimum green or a
/// pedestrian interval runs, and sometimes the stage that's already green, still gets no
/// overlaps, no green shorter than the minimum, every yellow and all-red in full, and
/// every WALK and flashing DON'T WALK in full.
#[test]
fn a_person_choosing_at_random_never_cuts_an_interval_short() {
    use intersection_lab::sim::rng::Rng;
    use intersection_lab::sim::signal::{PedSig, Phase};
    use intersection_lab::sim::vtypes::VKind;

    let handles: Vec<_> = (LayoutKind::ALL.into_iter().filter(|k| k.signalized()))
        .map(|kind| {
            common::spawn(move || {
                let mut sim = Sim::new(kind, ControlMode::Signal, CtrlKind::Manual, 41);
                sim.check_safety = true;
                // Emergency preemption may end a green early by design: no emergency
                // vehicles. More pedestrians than usual, with a leading interval, keep the
                // pedestrian intervals busy.
                sim.demand.mix[VKind::Emergency.index()] = 0.0;
                sim.demand.ped_rate = 240.0;
                sim.signal.lpi = 2.0;
                let mut rng = Rng::new(9);
                let (n, n_cw, dt) = (sim.net.stages.len(), sim.net.crosswalks.len(), sim.dt);
                let mut problems = vec![];
                // Each crosswalk's state, and when it began.
                let mut prev: Vec<PedSig> = sim.signal.cw.iter().map(|c| c.state).collect();
                let mut since = vec![0.0; n_cw];
                while sim.t < 900.0 {
                    // A choice every 5 s on average.
                    if rng.f64() < 0.02 {
                        let j = rng.below(n);
                        let sig = &sim.signal;
                        sim.controller.as_manual().unwrap().choose(j, sig);
                    }
                    sim.step();
                    for c in 0..n_cw {
                        let now = sim.signal.cw[c].state;
                        if now == prev[c] {
                            continue;
                        }
                        let need = match prev[c] {
                            PedSig::Walk => sim.signal.walk_time,
                            PedSig::Fdw => sim.net.crosswalks[c].length / sim.signal.ped_speed,
                            PedSig::Dw => 0.0,
                        };
                        let held = sim.t - since[c];
                        if (prev[c] == PedSig::Walk && now == PedSig::Dw) || held < need - dt - 1e-6 {
                            problems.push(format!(
                                "crosswalk {c}: {:?} lasted {held:.1} s, then {now:?} at {:.1} s",
                                prev[c], sim.t
                            ));
                        }
                        (prev[c], since[c]) = (now, sim.t);
                    }
                }
                // Every interval but the one still running.
                let spans: Vec<_> = sim.signal.history.iter().collect();
                for h in &spans[..spans.len() - 1] {
                    let len = h.end - h.start;
                    let ok = match h.phase {
                        Phase::Green => len >= sim.signal.min_green - dt - 1e-6,
                        Phase::Yellow => (len - sim.net.yellow).abs() <= dt + 1e-6,
                        Phase::AllRed => (len - sim.net.all_red).abs() <= dt + 1e-6,
                    };
                    if !ok {
                        problems.push(format!(
                            "{:?} of {} lasted {len:.1} s from {:.1} s",
                            h.phase, sim.net.stages[h.stage].name, h.start
                        ));
                    }
                }
                (kind, sim.collisions, sim.signal.switches, problems)
            })
        })
        .collect();
    let mut failures = vec![];
    for h in handles {
        let (kind, collisions, switches, problems) = h.join().unwrap();
        if collisions > 0 {
            failures.push(format!("{}: {collisions} overlap ticks", kind.key()));
        }
        // The person was obeyed: the signal changed stage many times.
        if switches < 20 {
            failures.push(format!("{}: only {switches} stage changes", kind.key()));
        }
        failures.extend(problems.into_iter().map(|p| format!("{}: {p}", kind.key())));
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
