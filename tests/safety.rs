// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Regression tests: every layout, control mode and automatic controller must run
//! collision-free and serve (nearly) all demand at default volumes, no moving vehicle
//! may come within half a meter of anyone on a crosswalk, a person choosing stages at
//! random must never cut an interval short, and emergency preemption may cut a green
//! short but never a clearance interval or a pedestrian's crossing, and holds for an
//! emergency vehicle stopped in a queue until it has crossed.

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

/// Where two paths merge they run close together before they join: on major/minor, a
/// left turn and the through path it joins come within 2.7 m of each other 12 m before
/// the merge point. Merges used to be ordered by arrival at the merge point alone, so a
/// fast vehicle going first there could overtake a slow one alongside it, inside that
/// stretch. Emergency vehicles sent through again and again make that likely: with this
/// seed, one going through on a green touched a truck still clearing the intersection,
/// for 0.4 s, until the order had to hold from where the paths come that close as well.
#[test]
fn no_overlap_where_two_paths_merge() {
    use intersection_lab::sim::network::Turn;
    use intersection_lab::sim::vtypes::VKind;

    let mut sim = Sim::new(LayoutKind::MajorMinor4, ControlMode::Signal, CtrlKind::Fixed, 2);
    sim.check_safety = true;
    sim.demand.ped_rate = 240.0;
    let (mut sent, mut next) = (0, 60.0);
    while sim.t < 1800.0 {
        // Every 90 s, an emergency vehicle through from whichever approach the clock picks.
        if sim.t >= next {
            let leg = (sim.steps as usize / 7) % sim.net.legs.len();
            if sim.net.movement_for(leg, Turn::T).is_some()
                && sim.spawn_manual(leg, VKind::Emergency, Turn::T)
            {
                sent += 1;
            }
            next = sim.t + 90.0;
        }
        sim.step();
    }
    let through = sim.stats.done.iter().filter(|d| d.kind == VKind::Emergency).count();
    assert!(through >= 15 && sent >= 15, "{through} of {sent} emergency vehicles got through");
    assert_eq!(sim.collisions, 0, "overlaps:\n{}", sim.collision_log.join("\n"));
}

/// The single-lane roundabout used to lock up. An entering vehicle could stop where its
/// path already runs close to the ring, behind circulating traffic that had stopped, and
/// the circulating vehicles behind it then waited for it, until each vehicle on the ring
/// waited for another. With an emergency vehicle sent through every 90 s, that happened
/// in about one run in fifty. And a vehicle still on its entry ranked as entering at the
/// next merge around the ring as well, and yielded there to the vehicles entering. A
/// truck, longer than the ring between two entries, then had to wait at its entry until
/// both merges were clear, which in heavy traffic could take many minutes. Before that
/// was fixed, each of these cases left a vehicle standing still for over five minutes:
/// the first with emergency vehicles, the others at 1.35× default demand. Either change
/// alone still leaves one of them stuck.
#[test]
fn no_gridlock_on_the_roundabout() {
    use intersection_lab::sim::demand::DemandConfig;
    use intersection_lab::sim::network::{Network, Turn};
    use intersection_lab::sim::vtypes::VKind;
    use std::collections::HashMap;

    // Seed, demand scale, and whether an emergency vehicle is sent through every 90 s.
    let cases = [(29, 1.0, true), (148, 1.35, false), (155, 1.35, false)];
    let handles: Vec<_> = cases
        .into_iter()
        .map(|(seed, scale, emergency)| {
            common::spawn(move || {
                let net = Network::build(LayoutKind::Roundabout);
                let mut demand = DemandConfig::default_for(&net);
                demand.scale = scale;
                let mut sim = Sim::with_demand(net, ControlMode::Roundabout, CtrlKind::Fixed, demand, seed);
                sim.check_safety = true;
                let (mut sent, mut next) = (0, 60.0);
                // How long each vehicle on the roundabout has been standing still.
                let mut still: HashMap<u64, f64> = HashMap::new();
                while sim.t < 1800.0 {
                    if emergency && sim.t >= next {
                        let leg = (sim.steps as usize / 7) % sim.net.legs.len();
                        if sim.spawn_manual(leg, VKind::Emergency, Turn::T) {
                            sent += 1;
                        }
                        next = sim.t + 90.0;
                    }
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
                                "seed {seed} ×{scale}: vehicle {} stuck on the roundabout at t={:.0}",
                                v.id, sim.t
                            ));
                        }
                    }
                }
                let through = sim.stats.done.iter().filter(|d| d.kind == VKind::Emergency).count();
                if emergency && (through < 15 || sent < 15) {
                    return Some(format!("seed {seed}: {through} of {sent} emergency vehicles got through"));
                }
                (sim.collisions > 0)
                    .then(|| format!("seed {seed} ×{scale}: overlaps:\n{}", sim.collision_log.join("\n")))
            })
        })
        .collect();
    let failures: Vec<String> = handles.into_iter().filter_map(|h| h.join().unwrap()).collect();
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// A truck turning left from the major road at a two-way stop could stop partway through
/// its turn, across the oncoming lanes, and stand there for minutes. It accepted a gap only
/// if the gap lasted for the whole turn, but it committed only to the first lane it
/// crossed, so a vehicle that came into view after it set off, too far away to count
/// before, made it stop short of the second lane while it stood in the first. From there
/// it needed a gap longer than it could see. It also checked only vehicles, and someone
/// already on the crosswalk it was turning into could stop it there just the same. Now it
/// waits for the crosswalk as well, and once it accepts a gap it commits to the whole
/// turn. A committed vehicle also stopped short of a crossing for anyone passing through
/// it, even someone who would be gone well before it got there; it no longer does. The
/// first case needs both of those last two changes. With an emergency vehicle sent through
/// every 90 s, and in the last two cases four times the pedestrians, each of these runs
/// left a truck standing across the oncoming lanes for over four minutes before.
#[test]
fn no_left_turn_stops_across_oncoming_lanes() {
    use intersection_lab::sim::demand::DemandConfig;
    use intersection_lab::sim::network::{Network, Turn};
    use intersection_lab::sim::vtypes::VKind;
    use std::collections::HashMap;

    // Layout, seed, and pedestrians an hour on each crosswalk (60 by default).
    let cases =
        [(LayoutKind::Tee, 11, 60.0), (LayoutKind::MajorMinor4, 14, 240.0), (LayoutKind::Tee, 19, 240.0)];
    let handles: Vec<_> = cases
        .into_iter()
        .map(|(kind, seed, ped_rate)| {
            common::spawn(move || {
                let net = Network::build(kind);
                let demand = DemandConfig::default_for(&net);
                let mut sim = Sim::with_demand(net, ControlMode::TwoWayStop, CtrlKind::Fixed, demand, seed);
                sim.check_safety = true;
                sim.demand.ped_rate = ped_rate;
                let mut next = 60.0;
                // How long each vehicle has been standing still in the box, and how long
                // a left-turner from the major road (east-west on both layouts) has stood
                // with its body where its path crosses the oncoming through lanes.
                let mut still: HashMap<u64, (f64, f64)> = HashMap::new();
                let (mut longest, mut longest_across) = (0.0f64, 0.0f64);
                while sim.t < 1800.0 {
                    if sim.t >= next {
                        let leg = (sim.steps as usize / 7) % sim.net.legs.len();
                        sim.spawn_manual(leg, VKind::Emergency, Turn::T);
                        next = sim.t + 90.0;
                    }
                    sim.step();
                    for v in &sim.vehicles {
                        let l = v.link();
                        let stopped = v.v < 0.05 && sim.net.links[l].kind.in_box();
                        let oncoming = match sim.net.movements[v.movement].name.as_str() {
                            "E_L" => "W_T",
                            "W_L" => "E_T",
                            _ => "",
                        };
                        let across = stopped
                            && sim.net.links[l].conflicts.iter().any(|&cid| {
                                let c = &sim.net.conflicts[cid];
                                let (s_c, other, _) = c.side(l);
                                let ((before, after), _) = c.zones(l);
                                sim.net.links[other]
                                    .movement
                                    .is_some_and(|m| sim.net.movements[m].name == oncoming)
                                    && v.s > s_c - before
                                    && v.s - v.len() < s_c + after
                            });
                        let t = still.entry(v.id).or_insert((0.0, 0.0));
                        t.0 = if stopped { t.0 + sim.dt } else { 0.0 };
                        t.1 = if across { t.1 + sim.dt } else { 0.0 };
                        longest = longest.max(t.0);
                        longest_across = longest_across.max(t.1);
                    }
                }
                if sim.collisions > 0 {
                    return Some(format!(
                        "{} seed {seed}: overlaps:\n{}",
                        kind.key(),
                        sim.collision_log.join("\n")
                    ));
                }
                (longest > 300.0 || longest_across > 60.0).then(|| {
                    format!(
                        "{} seed {seed}: a vehicle stood still in the box for {longest:.0} s, and a left \
                         turn across the oncoming lanes for {longest_across:.0} s",
                        kind.key()
                    )
                })
            })
        })
        .collect();
    let failures: Vec<String> = handles.into_iter().filter_map(|h| h.join().unwrap()).collect();
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

/// Whether vehicle `v`'s body comes within `r` of point `p`.
fn touches(
    sim: &Sim,
    v: &intersection_lab::sim::Vehicle,
    p: intersection_lab::sim::geom::V2,
    r: f64,
) -> bool {
    sim.body(v).iter().any(|&(c, axis, half_len, half_width)| {
        let d = p - c;
        d.dot(axis).abs() < half_len + r && d.cross(axis).abs() < half_width + r
    })
}

/// Each moving vehicle within half a meter of someone on a crosswalk (off the curb, so
/// in the road) right now. Pedestrians step around a stopped vehicle.
fn close_calls(sim: &Sim) -> Vec<String> {
    use intersection_lab::sim::PedState;
    let mut out = vec![];
    for p in sim.peds.iter().filter(|p| p.state == PedState::Crossing) {
        if !(0.0..=sim.net.crosswalks[p.cw].length).contains(&p.u) {
            continue;
        }
        let at = sim.ped_pos(p);
        if let Some(v) = sim.vehicles.iter().find(|v| v.v > 0.0 && touches(sim, v, at, 0.5)) {
            out.push(format!(
                "a {} came within 0.5 m of pedestrian {} at {:.1} s",
                v.kind.name(),
                p.id,
                sim.t
            ));
        }
    }
    out
}

/// At every layout and control mode with crosswalks, and with pedestrians arriving far
/// more often than by default, no moving vehicle comes within half a meter of anyone on
/// a crosswalk. Among the ways that can go wrong: a turning vehicle already on the
/// crosswalk driving on into someone in front of it, a vehicle stopping on a crosswalk
/// to yield to traffic just past it, someone stepping out beside the back of a long
/// vehicle whose front has left the intersection, and a vehicle that could still brake
/// hard driving past someone instead.
#[test]
fn no_vehicle_drives_into_anyone_crossing() {
    let mut handles = vec![];
    for kind in LayoutKind::ALL {
        for mode in ControlMode::options(kind) {
            for seed in [2, 5, 6, 41] {
                handles.push(common::spawn(move || {
                    let mut sim = Sim::new(kind, mode, CtrlKind::Fixed, seed);
                    sim.demand.ped_rate = 240.0;
                    let mut calls = vec![];
                    while sim.t < 1200.0 {
                        sim.step();
                        calls.extend(close_calls(&sim));
                    }
                    let name = format!("{} / {} / seed {seed}", kind.key(), mode.name());
                    (name, sim.net.crosswalks.len(), sim.stats.ped_waits.len(), calls)
                }));
            }
        }
    }
    let mut failures = vec![];
    for h in handles {
        let (name, crosswalks, crossings, calls) = h.join().unwrap();
        // Enough people crossed for the test to mean something.
        if crosswalks > 0 && crossings < 40 {
            failures.push(format!("{name}: only {crossings} people crossed"));
        }
        failures.extend(calls.into_iter().map(|c| format!("{name}: {c}")));
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Emergency preemption is the one request that doesn't wait for `can_switch`. Again and
/// again on every signalized layout, an emergency vehicle is sent toward a red while
/// pedestrians cross on a WALK. Preemption may end the green it finds after 2 s, before
/// its minimum, and end a WALK or flashing DON'T WALK with it: those turn to steady
/// DON'T WALK at once. The yellow and all-red still run in full, the pedestrians already
/// crossing finish, no moving vehicle comes within half a meter of one, and the emergency
/// vehicle gets through.
#[test]
fn preemption_cuts_a_green_short_but_never_a_clearance_or_a_crossing() {
    use intersection_lab::sim::PedState;
    use intersection_lab::sim::network::Turn;
    use intersection_lab::sim::signal::PedSig;
    use intersection_lab::sim::vtypes::VKind;

    let handles: Vec<_> = (LayoutKind::ALL.into_iter().filter(|k| k.signalized()))
        .flat_map(|kind| [23, 24].map(|seed| (kind, seed)))
        .map(|(kind, seed)| {
            common::spawn(move || {
                let mut sim = Sim::new(kind, ControlMode::Signal, CtrlKind::Fixed, seed);
                sim.check_safety = true;
                sim.demand.ped_rate = 240.0;
                let (n_cw, dt) = (sim.net.crosswalks.len(), sim.dt);
                let mut problems = vec![];
                let (mut sent, mut next) = (0, 60.0);
                // Each crosswalk's state, and when it began.
                let mut prev: Vec<PedSig> = sim.signal.cw.iter().map(|c| c.state).collect();
                let mut since = vec![0.0; n_cw];
                // Pedestrians who were crossing when their WALK or flashing DON'T WALK was
                // cut short, each with when.
                let mut cut_off: Vec<(u64, f64)> = vec![];
                while sim.t < 1800.0 {
                    let walking = (0..n_cw).any(|c| {
                        sim.signal.cw[c].state == PedSig::Walk
                            && sim.peds.iter().any(|p| p.cw == c && p.state == PedState::Crossing)
                    });
                    // The last 5 minutes leave everyone time to get through.
                    if sim.t >= next && sim.t < 1500.0 && walking && !sim.preempt_active {
                        let red = (0..sim.net.legs.len()).find(|&leg| {
                            sim.net
                                .movement_for(leg, Turn::T)
                                .is_some_and(|m| !sim.signal.status(&sim.net, m).is_green())
                        });
                        if let Some(leg) = red {
                            assert!(sim.spawn_manual(leg, VKind::Emergency, Turn::T));
                            (sent, next) = (sent + 1, sim.t + 90.0);
                        }
                    }
                    let crossing: Vec<(u64, usize)> = sim
                        .peds
                        .iter()
                        .filter(|p| p.state == PedState::Crossing)
                        .map(|p| (p.id, p.cw))
                        .collect();
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
                        if now == PedSig::Dw && sim.t - since[c] < need - dt - 1e-6 {
                            if !sim.preempt_active {
                                problems.push(format!(
                                    "crosswalk {c}: {:?} cut short without preemption",
                                    prev[c]
                                ));
                            }
                            cut_off.extend(crossing.iter().filter(|p| p.1 == c).map(|p| (p.0, sim.t)));
                        }
                        (prev[c], since[c]) = (now, sim.t);
                    }
                    problems.extend(close_calls(&sim));
                }
                // Everyone cut off finished crossing (the state after crossing is Leaving,
                // and then the pedestrian is gone).
                for &(id, t) in &cut_off {
                    if sim.peds.iter().any(|p| p.id == id && p.state == PedState::Crossing) {
                        problems
                            .push(format!("pedestrian {id}, cut off at {t:.1} s, never finished crossing"));
                    }
                }
                let (short_greens, intervals) = interval_problems(&sim);
                problems.extend(intervals);
                let delays: Vec<f64> =
                    sim.stats.done.iter().filter(|d| d.kind == VKind::Emergency).map(|d| d.delay).collect();
                let name = format!("{} seed {seed}", kind.key());
                (name, sim.collisions, sent, delays, short_greens, cut_off.len(), problems)
            })
        })
        .collect();
    let mut failures = vec![];
    let (mut short_greens, mut cut_off) = (0, 0);
    for h in handles {
        let (key, collisions, sent, delays, greens, cut, problems) = h.join().unwrap();
        eprintln!("{key}: {sent} sent, delays {delays:.1?}, {greens} short greens, {cut} cut off");
        if collisions > 0 {
            failures.push(format!("{key}: {collisions} overlap ticks"));
        }
        if sent < 10 || delays.len() < sent {
            failures.push(format!("{key}: {} of {sent} emergency vehicles got through", delays.len()));
        }
        short_greens += greens;
        cut_off += cut;
        failures.extend(problems.into_iter().map(|p| format!("{key}: {p}")));
    }
    // The cases this test is about came up: greens ended before their minimum, and
    // pedestrians still crossing when their interval ended.
    assert!(short_greens > 0 && cut_off > 0, "{short_greens} short greens, {cut_off} pedestrians cut off");
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// How many of the signal's finished intervals were greens that ended before their
/// minimum, and each that broke what even preemption must respect: a green shorter than
/// 2 s, or a yellow or all-red that didn't run in full.
fn interval_problems(sim: &Sim) -> (usize, Vec<String>) {
    use intersection_lab::sim::signal::Phase;
    let dt = sim.dt;
    let (mut short_greens, mut problems) = (0, vec![]);
    // Every interval but the one still running.
    let spans: Vec<_> = sim.signal.history.iter().collect();
    for h in &spans[..spans.len() - 1] {
        let len = h.end - h.start;
        let ok = match h.phase {
            Phase::Green => {
                short_greens += usize::from(len < sim.signal.min_green - dt - 1e-6);
                len >= 2.0 - 1e-6
            }
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
    (short_greens, problems)
}

/// An emergency vehicle stopped in a queue more than 90 m from the stop line is more than
/// 30 s away by its predicted arrival, which takes its speed as at least 3 m/s. Preemption
/// used to wait until one was due within 30 s, so it let go when the vehicle braked into a
/// queue: the green it had asked for could end while the vehicle still stood in the queue,
/// and the vehicle then waited out a second change before the queue ahead of it moved
/// again. On every approach of every signalized layout with a through movement, under
/// manual control, an emergency vehicle is sent in behind a queue held at a red, and
/// stands still more than 90 m from the stop line. As soon as its green starts, the person
/// at the controls asks for the red again. Preemption holds from the moment the vehicle
/// appears until it has crossed, its green lasts until then, no green ends within 2 s,
/// every yellow and all-red runs in full, nothing overlaps, and it gets through.
#[test]
fn preemption_holds_for_an_emergency_vehicle_stopped_in_a_queue() {
    let handles: Vec<_> = (LayoutKind::ALL.into_iter().filter(|k| k.signalized()))
        .flat_map(|kind| (0..4).map(move |leg| (kind, leg)))
        .map(|(kind, leg)| common::spawn(move || queued_emergency_vehicle(kind, leg)))
        .collect();
    let mut failures = vec![];
    for h in handles {
        let Some((name, delay, problems)) = h.join().unwrap() else { continue };
        eprintln!("{name}: delay {delay:.1?} s");
        failures.extend(problems.into_iter().map(|p| format!("{name}: {p}")));
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Sends an emergency vehicle straight through from approach `leg` of `kind` under manual
/// control, behind a queue held at a red: 20 cars for each lane it may use, which reach
/// about 140 m back, and no other traffic. Once its green starts, chooses the stage that
/// held the queue again. Returns the case's name, the emergency vehicle's delay if it got
/// through, and what went wrong, or `None` if the approach has no through movement.
fn queued_emergency_vehicle(kind: LayoutKind, leg: usize) -> Option<(String, Option<f64>, Vec<String>)> {
    use intersection_lab::sim::network::{LinkKind, Turn};
    use intersection_lab::sim::signal::Phase;
    use intersection_lab::sim::vtypes::VKind;

    let mut sim = Sim::new(kind, ControlMode::Signal, CtrlKind::Manual, 7);
    let m = sim.net.movement_for(leg, Turn::T)?;
    let name = format!("{} leg {leg}", kind.key());
    sim.check_safety = true;
    sim.demand.scale = 0.0;
    let choose = |sim: &mut Sim, j: usize| {
        let sig = &sim.signal;
        sim.controller.as_manual().unwrap().choose(j, sig);
    };
    let green = |sim: &Sim| sim.signal.status(&sim.net, m).is_green();
    let red = (0..sim.net.stages.len()).find(|&st| sim.net.stages[st].stat[m].is_none()).unwrap();
    choose(&mut sim, red);
    while !(sim.signal.stage == red && sim.signal.phase == Phase::Green) {
        sim.step();
    }
    let lanes = (sim.net.legs[leg].in_lanes.iter())
        .filter(|&&lane| sim.net.links[lane].lane_moves.contains(&m))
        .count();
    for _ in 0..20 * lanes {
        assert!(sim.spawn_manual(leg, VKind::Car, Turn::T));
    }
    // Until every car has come to a stop in the queue.
    while sim.vehicles.iter().filter(|v| v.manual && v.v < 0.05).count() < 20 * lanes {
        sim.step();
        if sim.t > 300.0 {
            return Some((name, None, vec!["the queue didn't form in 300 s".into()]));
        }
    }
    assert!(sim.spawn_manual(leg, VKind::Emergency, Turn::T));
    let sent = sim.t;
    let mut problems = vec![];
    let (mut stood_far_back, mut green_since, mut preempted) = (false, None, true);
    let through = |sim: &Sim| sim.stats.done.iter().find(|d| d.kind == VKind::Emergency).map(|d| d.delay);
    while through(&sim).is_none() && sim.t < sent + 300.0 {
        sim.step();
        let ev = sim.vehicles.iter().find(|v| v.kind == VKind::Emergency);
        let Some((link, d, v)) = ev.map(|ev| {
            let link = &sim.net.links[ev.link()];
            (link.kind, link.length() - ev.s, ev.v)
        }) else {
            continue;
        };
        if link == LinkKind::Inbound {
            stood_far_back |= v < 0.05 && d > 90.0;
            if !green(&sim) {
                if let Some(t0) = green_since.take() {
                    problems.push(format!(
                        "its green from {t0:.1} s ended at {:.1} s, {d:.0} m before it reached the stop line",
                        sim.t
                    ));
                }
            } else if green_since.is_none() {
                green_since = Some(sim.t);
                choose(&mut sim, red);
            }
        }
        if preempted && !sim.preempt_active && (link == LinkKind::Inbound || link.in_box()) {
            preempted = false;
            problems.push(format!("preemption let go at {:.1} s, before it had crossed", sim.t));
        }
    }
    let delay = through(&sim);
    if delay.is_none() {
        problems.push("it didn't get through in 300 s".into());
    }
    if !stood_far_back {
        problems.push("it never stood still more than 90 m from the stop line".into());
    }
    if sim.collisions > 0 {
        problems.push(format!("{} overlap ticks", sim.collisions));
    }
    problems.extend(interval_problems(&sim).1);
    Some((name, delay, problems))
}
