// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The signal-free cooperative crossing: its geometry, the intersection manager's phase
//! order, safety under every gap policy, capacity at the design demand, why weaving
//! vehicles through one at a time carries less, determinism, and how it treats light
//! traffic, emergency vehicles and vehicles placed by hand.

use intersection_lab::control::CtrlKind;
use intersection_lab::sim::coop::{CoopManager, GapPolicy, Scheduling};
use intersection_lab::sim::demand::{DemandConfig, Pattern};
use intersection_lab::sim::network::{LayoutKind, Network, Turn, build_cooperative};
use intersection_lab::sim::stats::Summary;
use intersection_lab::sim::vtypes::VKind;
use intersection_lab::sim::{ControlMode, Sim};

/// A crossing with `n_left` left-turn lanes per approach and `total` veh/h spread evenly
/// over the four approaches, with the safety checks on.
fn crossing(n_left: usize, total: f64, cars_only: bool, policy: GapPolicy, seed: u64) -> Sim {
    let net = build_cooperative(n_left);
    let mut demand = DemandConfig::default_for(&net);
    demand.rate = [total / 4.0; 4];
    if cars_only {
        demand.mix = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    }
    let mut sim = Sim::with_demand(net, ControlMode::Cooperative, CtrlKind::Fixed, demand, seed);
    sim.set_gap_policy(policy);
    sim.check_safety = true;
    sim
}

/// Everything the monitors and the overlap check measured that should never happen.
fn safety_failures(name: &str, sim: &Sim) -> Vec<String> {
    let st = &sim.coop.as_ref().expect("an intersection manager").stats;
    let mut f = vec![];
    if sim.collisions > 0 {
        f.push(format!("{name}: {} overlap ticks", sim.collisions));
    }
    if st.zone_violations > 0 {
        f.push(format!("{name}: {} zones entered while occupied", st.zone_violations));
    }
    if st.unauthorized > 0 {
        f.push(format!("{name}: {} box entries without a reservation", st.unauthorized));
    }
    if st.deviations > 0 {
        f.push(format!("{name}: {} steps off the granted trajectory", st.deviations));
    }
    if st.margin_min < -1e-6 {
        f.push(format!("{name}: a crossing came {:.3} s inside its required gap", -st.margin_min));
    }
    if st.timing_err_max > 1e-6 {
        f.push(format!("{name}: zone times off the reservation by {:.2e} s", st.timing_err_max));
    }
    if st.interactions == 0 {
        f.push(format!("{name}: no crossings measured"));
    }
    f
}

fn summary(sim: &Sim, from: f64) -> Summary {
    sim.stats.summary(from, sim.t, &sim.vehicles, &sim.queues)
}

fn queued(sim: &Sim) -> usize {
    sim.queues.iter().map(|q| q.len()).sum()
}

#[test]
fn needs_no_signals_sensors_or_crosswalks() {
    assert_eq!(ControlMode::options(LayoutKind::Cooperative), vec![ControlMode::Cooperative]);
    // Any other control mode asked for is replaced by the manager.
    let mut sim = Sim::new(LayoutKind::Cooperative, ControlMode::Signal, CtrlKind::Predictive, 1);
    assert_eq!(sim.mode, ControlMode::Cooperative);
    assert!(!sim.signalized());
    assert!(sim.net.crosswalks.is_empty());
    assert_eq!(sim.demand.ped_rate, 0.0);
    assert_eq!(sim.demand.rate.iter().sum::<f64>(), 18_000.0, "the default demand is the design demand");
    sim.run_for(120.0);
    assert!(sim.view.detections.is_empty(), "nothing is sensed");
    assert!(sim.coop.as_ref().unwrap().stats.plans > 0);
}

#[test]
fn geometry_has_only_crossings() {
    for (n_left, want) in [(2, (36, 48, 16)), (1, (36, 24, 4))] {
        let net = build_cooperative(n_left);
        let turn = |link: usize| net.movements[net.links[link].movement.expect("a box path")].turn;
        let (mut tt, mut lt, mut ll) = (0, 0, 0);
        for c in &net.conflicts {
            assert!(!c.merge, "{n_left} left: paths merge in the box");
            match (turn(c.a), turn(c.b)) {
                (Turn::T, Turn::T) => tt += 1,
                (Turn::L, Turn::L) => ll += 1,
                (Turn::R, _) | (_, Turn::R) => panic!("{n_left} left: a right turn enters the box"),
                _ => lt += 1,
            }
        }
        assert_eq!((tt, lt, ll), want, "{n_left} left: crossings (through-through, left-through, left-left)");
        let mgr = CoopManager::new(&net);
        let m = |n: &str| net.movement_by_name(n).unwrap();
        for (i, mv) in net.movements.iter().enumerate() {
            assert_eq!(mgr.managed(i), mv.turn != Turn::R, "{}", mv.name);
            let crossed = (0..net.movements.len()).filter(|&o| mgr.conflicts(i, o)).count();
            assert_eq!(
                crossed,
                if mv.turn == Turn::R { 0 } else { 4 },
                "{} crosses {crossed} movements",
                mv.name
            );
        }
        assert!(
            !mgr.conflicts(m("N_L"), m("S_L")) && !mgr.conflicts(m("E_L"), m("W_L")),
            "opposite lefts pass clear"
        );
        // Every lane has its own path through the box and its own receiving lane.
        let mut outs: Vec<usize> =
            net.movements.iter().flat_map(|mv| mv.routes.iter().map(|r| *r.1.last().unwrap())).collect();
        let n = outs.len();
        outs.sort_unstable();
        outs.dedup();
        assert_eq!(outs.len(), n, "{n_left} left: two lanes share a receiving lane");
    }
}

#[test]
fn left_turns_lag_the_through_movements() {
    let net = Network::build(LayoutKind::Cooperative);
    let mgr = CoopManager::new(&net);
    let names: Vec<Vec<&str>> = mgr
        .ring_order()
        .iter()
        .map(|r| r.iter().map(|&m| net.movements[m].name.as_str()).collect())
        .collect();
    assert_eq!(names, [["S_T", "N_L", "W_T", "E_L"], ["N_T", "S_L", "E_T", "W_L"]]);
    // A left turn crosses the opposing through lanes late in its arc, which those
    // vehicles reach early: it can follow them almost at once, but they must wait for it.
    let m = |n: &str| net.movement_by_name(n).unwrap();
    let after_through = mgr.intergreen(m("S_T"), m("N_L")).unwrap();
    let after_left = mgr.intergreen(m("N_L"), m("S_T")).unwrap();
    assert!(after_through < 1.0 && after_left > 4.0, "{after_through:.2} s vs {after_left:.2} s");
    assert!(mgr.lost_time() > 0.0 && mgr.lost_time() < 10.0, "{:.2} s lost per cycle", mgr.lost_time());
}

#[test]
fn every_gap_policy_is_safe_under_heavy_demand() {
    // Each policy, with dual and single left lanes, mixed traffic and cars only, three
    // arrival streams each, at about the capacity of each layout.
    let mut jobs = vec![];
    for policy in GapPolicy::ALL {
        for (n_left, total) in [(2, 18_000.0), (1, 12_000.0)] {
            for cars in [false, true] {
                for seed in 31..34 {
                    jobs.push(std::thread::spawn(move || {
                        let mut sim = crossing(n_left, total, cars, policy, seed);
                        sim.run_for(1200.0);
                        let fleet = if cars { "cars" } else { "mixed" };
                        safety_failures(
                            &format!(
                                "{} / {n_left} left / {fleet} / {total} veh/h / seed {seed}",
                                policy.short()
                            ),
                            &sim,
                        )
                    }));
                }
            }
        }
    }
    let failures: Vec<String> = jobs.into_iter().flat_map(|j| j.join().unwrap()).collect();
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn bursts_heavy_vehicles_and_uneven_demand_are_served_safely() {
    // Each case with the 95th-percentile and longest delay it may cause. The rush-hour
    // peak and the surges exceed the capacity for minutes at a time: they are only
    // checked for safety.
    type Case = (String, DemandConfig, Option<(f64, f64)>);
    let mut cases: Vec<Case> = vec![];
    let net = build_cooperative(2);
    let base = || {
        let mut d = DemandConfig::default_for(&net);
        d.rate = [4_500.0; 4];
        d
    };
    for (pattern, limits) in
        [(Pattern::Platoons, Some((100.0, 300.0))), (Pattern::RushHour, None), (Pattern::Surges, None)]
    {
        let mut d = base();
        d.pattern = pattern;
        cases.push((format!("{pattern:?} arrivals"), d, limits));
    }
    let mut d = base();
    d.rate = [2_000.0; 4];
    d.mix = [0.0, 0.0, 0.6, 0.4, 0.0, 0.0];
    cases.push(("trucks and buses only".into(), d, Some((100.0, 200.0))));
    // Phases of uneven length: each must still serve its queue as one batch.
    let mut d = base();
    d.rate = [6_500.0, 1_500.0, 6_500.0, 1_500.0];
    cases.push(("heavy north-south".into(), d, Some((100.0, 200.0))));
    let mut d = base();
    d.rate = [3_500.0; 4];
    d.turn = [[0.4, 0.45, 0.15]; 4];
    cases.push(("40% left turns".into(), d, Some((100.0, 200.0))));
    let jobs: Vec<_> = cases
        .into_iter()
        .map(|(name, d, limits)| {
            std::thread::spawn(move || {
                let mut sim =
                    Sim::with_demand(build_cooperative(2), ControlMode::Cooperative, CtrlKind::Fixed, d, 41);
                sim.check_safety = true;
                sim.run_for(1200.0);
                let s = summary(&sim, 120.0);
                let mut f = safety_failures(&name, &sim);
                if let Some((p95, max)) = limits {
                    if s.p95_delay > p95 || s.max_delay > max {
                        f.push(format!(
                            "{name}: delay {:.0} s at the 95th percentile, {:.0} s at most",
                            s.p95_delay, s.max_delay
                        ));
                    }
                    if queued(&sim) > 60 {
                        f.push(format!("{name}: {} vehicles still waiting to enter", queued(&sim)));
                    }
                }
                f
            })
        })
        .collect();
    let failures: Vec<String> = jobs.into_iter().flat_map(|j| j.join().unwrap()).collect();
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn switching_policy_or_scheduling_mid_run_stays_safe() {
    let mut sim = crossing(2, 18_000.0, false, GapPolicy::Nominal, 5);
    for p in [GapPolicy::FailSafe, GapPolicy::Mitigated, GapPolicy::Nominal] {
        sim.run_for(150.0);
        sim.set_gap_policy(p);
    }
    sim.run_for(150.0);
    sim.set_scheduling(Scheduling::FirstCome);
    sim.run_for(150.0);
    let batches = sim.coop.as_ref().unwrap().stats.batches;
    sim.set_scheduling(Scheduling::Batches);
    sim.run_for(150.0);
    assert!(
        sim.coop.as_ref().unwrap().stats.batches > batches + 4,
        "batches resume after first come, first served"
    );
    let failures = safety_failures("policy and scheduling switched live", &sim);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn weaving_one_at_a_time_carries_far_less() {
    // First come, first served gives each vehicle the earliest gap that fits, so the
    // streams weave through one another one vehicle at a time. In light traffic that is
    // quicker than waiting for a batch; but every switch between crossing streams costs
    // a full safety gap, which a batch pays once, so it saturates far below the batches.
    let run = |sched: Scheduling, total: f64, seed: u64| {
        std::thread::spawn(move || {
            let mut sim = crossing(2, total, false, GapPolicy::FailSafe, seed);
            sim.set_scheduling(sched);
            sim.run_for(120.0 + 900.0);
            let s = summary(&sim, 120.0);
            let f = safety_failures(&format!("{} at {total} veh/h", sched.label()), &sim);
            (s.avg_delay, s.throughput_vph, queued(&sim), f)
        })
    };
    let jobs = [
        run(Scheduling::FirstCome, 4_000.0, 61),
        run(Scheduling::Batches, 4_000.0, 61),
        run(Scheduling::FirstCome, 14_000.0, 62),
        run(Scheduling::Batches, 14_000.0, 62),
    ];
    let [light_zip, light_batch, heavy_zip, heavy_batch] = jobs.map(|j| j.join().unwrap());
    let failures: Vec<String> =
        [&light_zip.3, &light_batch.3, &heavy_zip.3, &heavy_batch.3].into_iter().flatten().cloned().collect();
    assert!(failures.is_empty(), "{}", failures.join("\n"));
    assert!(
        light_zip.0 < light_batch.0 && light_zip.0 < 3.0,
        "light traffic: {:.1} s weaving, {:.1} s in batches",
        light_zip.0,
        light_batch.0
    );
    assert!(
        heavy_batch.1 > 13_300.0 && heavy_batch.2 < 50 && heavy_batch.0 < 30.0,
        "batches at 14,000 veh/h: served {:.0}, {} waiting, {:.1} s",
        heavy_batch.1,
        heavy_batch.2,
        heavy_batch.0
    );
    assert!(
        heavy_zip.1 < 11_000.0 && heavy_zip.2 > 500,
        "weaving at 14,000 veh/h: served {:.0} veh/h, {} waiting",
        heavy_zip.1,
        heavy_zip.2
    );
}

#[test]
fn dual_left_design_serves_18000_vehicles_per_hour() {
    // Three arrival streams of 20 minutes each at the design demand: everything that
    // arrives is served, nobody waits long, and the monitors see nothing wrong.
    let jobs: Vec<_> = (0..3)
        .map(|seed| {
            std::thread::spawn(move || {
                let mut sim = crossing(2, 18_000.0, false, GapPolicy::FailSafe, 7000 + seed);
                sim.run_for(120.0 + 1200.0);
                let s = summary(&sim, 120.0);
                let mut f = safety_failures(&format!("design demand, seed {seed}"), &sim);
                let q = queued(&sim);
                if s.throughput_vph < 17_300.0 {
                    f.push(format!("seed {seed}: served {:.0} veh/h", s.throughput_vph));
                }
                // A crossing that could not keep up would leave hundreds waiting; a few
                // dozen is a queue caught mid-phase as the run ends.
                if q > 120 {
                    f.push(format!("seed {seed}: {q} vehicles still waiting to enter"));
                }
                if s.avg_delay > 40.0 || s.max_delay > 180.0 {
                    f.push(format!(
                        "seed {seed}: delay {:.1} s on average, {:.0} s at most",
                        s.avg_delay, s.max_delay
                    ));
                }
                f
            })
        })
        .collect();
    let failures: Vec<String> = jobs.into_iter().flat_map(|j| j.join().unwrap()).collect();
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn oversaturated_capacity_is_about_19000_vehicles_per_hour() {
    // Oversaturated, the crossing serves what it can: a little above the design demand.
    let mut sim = crossing(2, 22_000.0, false, GapPolicy::FailSafe, 13);
    sim.run_for(120.0 + 1200.0);
    let s = summary(&sim, 120.0);
    assert!((18_300.0..19_800.0).contains(&s.throughput_vph), "served {:.0} veh/h", s.throughput_vph);
    assert!(queued(&sim) > 500, "the excess demand should queue");
    let failures = safety_failures("oversaturated", &sim);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn single_left_lanes_fall_short_of_the_design_demand() {
    let run = |n_left: usize| {
        std::thread::spawn(move || {
            let mut sim = crossing(n_left, 16_000.0, false, GapPolicy::FailSafe, 3);
            sim.run_for(900.0);
            (summary(&sim, 120.0).throughput_vph, queued(&sim))
        })
    };
    let (dual, single) = (run(2), run(1));
    let ((dual_vph, dual_q), (single_vph, single_q)) = (dual.join().unwrap(), single.join().unwrap());
    assert!(dual_q < 50, "dual lefts left {dual_q} waiting");
    assert!(
        single_q > 300 && single_vph < dual_vph - 1_000.0,
        "single lefts served {single_vph:.0} veh/h, {single_q} waiting"
    );
}

#[test]
fn runs_are_reproducible() {
    // Two runs on their own threads (so their hash maps iterate differently) must agree
    // to the last bit.
    let run = || {
        std::thread::spawn(|| {
            let mut sim = crossing(2, 18_000.0, false, GapPolicy::FailSafe, 11);
            sim.run_for(300.0);
            let c = sim.coop.as_ref().unwrap();
            let delay: f64 = sim.stats.done.iter().map(|d| d.delay).sum();
            let pos: f64 = sim.vehicles.iter().map(|v| v.s + v.v).sum();
            (
                sim.stats.generated,
                sim.stats.done.len(),
                c.stats.plans,
                c.stats.batches,
                delay.to_bits(),
                pos.to_bits(),
                c.stats.pet_min.to_bits(),
            )
        })
    };
    let (a, b) = (run(), run());
    assert_eq!(a.join().unwrap(), b.join().unwrap());
}

#[test]
fn light_traffic_barely_slows() {
    let mut sim = crossing(2, 2_000.0, false, GapPolicy::FailSafe, 21);
    sim.run_for(900.0);
    let s = summary(&sim, 120.0);
    assert!(s.avg_delay < 6.0, "average delay {:.1} s", s.avg_delay);
    assert!(s.max_delay < 40.0, "a vehicle waited {:.0} s", s.max_delay);
    let failures = safety_failures("light traffic", &sim);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn emergency_vehicles_go_first() {
    let mut sim = crossing(2, 14_000.0, false, GapPolicy::FailSafe, 9);
    sim.run_for(300.0);
    let t0 = sim.t;
    for leg in 0..4 {
        assert!(sim.spawn_manual(leg, VKind::Emergency, Turn::L));
    }
    sim.run_for(150.0);
    let others: Vec<f64> = sim
        .stats
        .done
        .iter()
        .filter(|d| d.t_exit > t0 && d.turn == Turn::L && d.kind != VKind::Emergency)
        .map(|d| d.delay)
        .collect();
    let typical = others.iter().sum::<f64>() / others.len().max(1) as f64;
    let em: Vec<f64> =
        sim.stats.done.iter().filter(|d| d.kind == VKind::Emergency).map(|d| d.delay).collect();
    assert_eq!(em.len(), 4, "every emergency vehicle got through");
    assert!(em.iter().all(|&d| d < 15.0), "emergency delays {em:?} (other left turns {typical:.1} s)");
    assert!(sim.coop.as_ref().unwrap().stats.preempted >= 4);
    let failures = safety_failures("emergency", &sim);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn vehicles_placed_by_hand_cross_every_way() {
    let mut sim = crossing(2, 0.0, false, GapPolicy::FailSafe, 1);
    let mut n = 0;
    for leg in 0..4 {
        for turn in Turn::ALL {
            for kind in [VKind::Car, VKind::Truck, VKind::Bus, VKind::Motorcycle] {
                assert!(sim.spawn_manual(leg, kind, turn));
                n += 1;
            }
        }
    }
    sim.run_for(240.0);
    assert_eq!(sim.stats.done.len(), n, "{} of {n} vehicles got through", sim.stats.done.len());
    let failures = safety_failures("by hand", &sim);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
