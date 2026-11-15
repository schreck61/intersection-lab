// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Failures at the signal-free cooperative crossing.
//!
//! Makes one vehicle fail in each of many runs and reports what follows: whether
//! crossing traffic stops short of it, the contacts and their speeds, how many vehicles
//! brake or have their crossings withdrawn, and what the failure costs in delay.
//!
//! ```text
//! cargo run --release --example failures -- [runs] [batch demand] [fcfs demand] [clear s] [part]
//! ```
//!
//! * runs: failures per row of the safety tables (default 200) and a quarter as many
//!   pairs of runs per row of the cost table
//! * batch demand, fcfs demand: total demand in veh/h under each schedule (defaults
//!   16000 and 8000, about 85% and 80% of what each carries)
//! * clear s: how long a disabled vehicle stays before it is cleared away (default 60)
//! * part: `safety`, `cost` or `all` (default)
//!
//! A failure comes either at a random place in the box, on a random lane, to the next
//! vehicle that gets there, or at the worst moment: to a vehicle about to leave a zone
//! with a crossing vehicle due right behind it at the smallest gap its policy allows.

use intersection_lab::control::CtrlKind;
use intersection_lab::sim::coop::{Fault, GapPolicy, Scheduling};
use intersection_lab::sim::demand::DemandConfig;
use intersection_lab::sim::network::{Turn, build_cooperative};
use intersection_lab::sim::rng::Rng;
use intersection_lab::sim::{ControlMode, Sim};

/// Simulated time before the failure, and after it (s).
const WARM: f64 = 180.0;
const AFTER: f64 = 240.0;

#[derive(Clone, Copy)]
struct Case {
    sched: Scheduling,
    policy: GapPolicy,
    fault: Fault,
    worst: bool,
    demand: f64,
    clear: f64,
}

fn crossing(case: &Case, seed: u64) -> Sim {
    let net = build_cooperative(2);
    let mut demand = DemandConfig::default_for(&net);
    demand.rate = [case.demand / 4.0; 4];
    let mut sim = Sim::with_demand(net, ControlMode::Cooperative, CtrlKind::Fixed, demand, seed);
    sim.set_gap_policy(case.policy);
    sim.set_scheduling(case.sched);
    sim.check_safety = true;
    if let Some(c) = sim.coop.as_mut() {
        c.clear_time = case.clear;
    }
    sim
}

/// Make one vehicle fail in the box: at a random place, or at the worst moment.
fn fail(sim: &mut Sim, case: &Case, rng: &mut Rng) {
    if case.worst {
        sim.stall_at_worst(case.fault);
        return;
    }
    let lanes: Vec<usize> = sim
        .net
        .movements
        .iter()
        .filter(|m| m.turn != Turn::R)
        .flat_map(|m| m.routes.iter().map(|r| r.0))
        .collect();
    let lane = lanes[rng.below(lanes.len())];
    let (l_in, l_box) = sim.coop.as_ref().and_then(|c| c.box_span(lane)).expect("a lane through the box");
    sim.stall_at(lane, rng.range(l_in, l_box), case.fault);
}

/// What one failure led to.
#[derive(Default)]
struct Outcome {
    happened: bool,
    /// Fastest contact between crossing vehicles and between vehicles in a lane (m/s).
    crossing: Option<f64>,
    lane: Option<f64>,
    stops: f64,
    withdrawn: f64,
    /// How far short of its zone the closest emergency stop came to rest (m).
    margin: Option<f64>,
    /// Time from the failure to the last emergency stop at rest (s).
    settle: f64,
    towed: u64,
    /// What should never happen, failure or not.
    wrong: u64,
}

fn one_failure(case: Case, seed: u64) -> Outcome {
    let mut sim = crossing(&case, seed);
    let mut rng = Rng::new(seed ^ 0xFA11);
    sim.run_for(WARM + rng.range(0.0, 30.0));
    fail(&mut sim, &case, &mut rng);
    sim.run_for(AFTER);
    let c = sim.coop.as_ref().expect("an intersection manager");
    let st = &c.stats;
    let stuck = sim.vehicles.iter().filter(|v| v.disabled).count() as u64;
    let mut out = Outcome {
        towed: st.towed,
        wrong: st.deviations + st.unauthorized + sim.collisions + stuck,
        ..Default::default()
    };
    if let Some(inc) = c.incidents.front() {
        let (crossing, lane) = inc.worst();
        out = Outcome {
            happened: true,
            crossing,
            lane,
            stops: inc.stops as f64,
            withdrawn: inc.withdrawn as f64,
            margin: inc.margin.is_finite().then_some(inc.margin),
            settle: inc.settled - inc.t,
            ..out
        };
    }
    out
}

fn kmh(v: f64) -> f64 {
    v * 3.6
}

fn safety(runs: u64, batch_q: f64, fcfs_q: f64, clear: f64) {
    println!(
        "One failure per run in the box, {runs} runs per row. Batches at {batch_q:.0} veh/h, first come, first served at {fcfs_q:.0} veh/h."
    );
    println!(
        "Contacts: share of failures with one, and the fastest (km/h). Closest stop: how far short of the"
    );
    println!(
        "zone it braked for the closest emergency stop came to rest, over all runs (m; negative: inside it)."
    );
    for worst in [false, true] {
        for sched in Scheduling::ALL {
            println!(
                "\n=== {}, {} ===",
                if worst { "failure at the worst moment" } else { "failure at a random place and time" },
                sched.label()
            );
            println!(
                "{:<22} {:<4} {:>6} {:>10} {:>8} {:>10} {:>8} {:>7} {:>10} {:>8} {:>7} {:>6} {:>6}",
                "failure",
                "gap",
                "runs",
                "crossing",
                "fastest",
                "same lane",
                "fastest",
                "stops",
                "withdrawn",
                "closest",
                "settle",
                "towed",
                "wrong"
            );
            for fault in Fault::ALL {
                for policy in GapPolicy::ALL {
                    let demand = if sched == Scheduling::Batches { batch_q } else { fcfs_q };
                    let case = Case { sched, policy, fault, worst, demand, clear };
                    let jobs: Vec<_> =
                        (0..runs).map(|k| std::thread::spawn(move || one_failure(case, 4000 + k))).collect();
                    let all: Vec<Outcome> = jobs.into_iter().map(|j| j.join().unwrap()).collect();
                    let res: Vec<&Outcome> = all.iter().filter(|o| o.happened).collect();
                    let n = res.len().max(1) as f64;
                    let share = |f: fn(&Outcome) -> Option<f64>| {
                        100.0 * res.iter().filter(|o| f(o).is_some()).count() as f64 / n
                    };
                    let fastest = |f: fn(&Outcome) -> Option<f64>| {
                        res.iter()
                            .filter_map(|o| f(o))
                            .reduce(f64::max)
                            .map_or("–".to_string(), |v| format!("{:.0}", kmh(v)))
                    };
                    let mean = |f: fn(&Outcome) -> f64| res.iter().map(|o| f(o)).sum::<f64>() / n;
                    let margin = res
                        .iter()
                        .filter_map(|o| o.margin)
                        .reduce(f64::min)
                        .map_or("–".to_string(), |m| format!("{m:.1}"));
                    let settle = res.iter().map(|o| o.settle).fold(0.0, f64::max);
                    println!(
                        "{:<22} {:<4} {:>6} {:>9.1}% {:>8} {:>9.1}% {:>8} {:>7.1} {:>10.1} {:>8} {:>7.1} {:>6} {:>6}",
                        fault.name(),
                        policy.short(),
                        res.len(),
                        share(|o| o.crossing),
                        fastest(|o| o.crossing),
                        share(|o| o.lane),
                        fastest(|o| o.lane),
                        mean(|o| o.stops),
                        mean(|o| o.withdrawn),
                        margin,
                        settle,
                        all.iter().map(|o| o.towed).sum::<u64>(),
                        all.iter().map(|o| o.wrong).sum::<u64>(),
                    );
                }
            }
        }
    }
}

/// The same arrivals with and without one failure, stepped together.
struct Cost {
    /// Vehicles served while the disabled vehicle stood, as a share of those served
    /// without the failure.
    served: f64,
    /// Most vehicles on the map or waiting to enter above the run without the failure.
    backlog: f64,
    /// Time after it was cleared away until the backlog was gone (s); None if it was
    /// still there at the end.
    recovery: Option<f64>,
    /// Average delay added to every vehicle that arrived during the run, those still
    /// on the map or waiting to enter included (s).
    extra_delay: f64,
}

fn people(sim: &Sim) -> f64 {
    let (on_map, waiting) = sim.in_system();
    (on_map + waiting) as f64
}

fn one_cost(case: Case, seed: u64, span: f64) -> Option<Cost> {
    let mut with = crossing(&case, seed);
    let mut without = crossing(&case, seed);
    let mut rng = Rng::new(seed ^ 0xFA11);
    let warm = 300.0 + rng.range(0.0, 30.0);
    with.run_for(warm);
    without.run_for(warm);
    fail(&mut with, &case, &mut rng);
    let (mut t_fail, mut t_clear) = (None, None);
    let (mut served, mut backlog) = ((0usize, 0usize), 0.0_f64);
    let mut recovery = None;
    let mut calm = 0.0;
    let steps = (span / with.dt).round() as usize;
    let mut done = (with.stats.done.len(), without.stats.done.len());
    for _ in 0..steps {
        with.step();
        without.step();
        let c = with.coop.as_ref().expect("an intersection manager");
        if t_fail.is_none() && c.stats.stalls > 0 {
            t_fail = Some(with.t);
            done = (with.stats.done.len(), without.stats.done.len());
        }
        if t_fail.is_some() && t_clear.is_none() && c.wrecks.is_empty() && c.stats.cleared > 0 {
            t_clear = Some(with.t);
            served = (with.stats.done.len() - done.0, without.stats.done.len() - done.1);
        }
        let extra = people(&with) - people(&without);
        backlog = backlog.max(extra);
        if let Some(tc) = t_clear
            && recovery.is_none()
        {
            // Gone once the two runs have stayed within ten vehicles for 20 s.
            calm = if extra <= 10.0 { calm + with.dt } else { 0.0 };
            if calm >= 20.0 {
                recovery = Some(with.t - 20.0 - tc);
            }
        }
    }
    t_clear?;
    let delay = |sim: &Sim| sim.stats.summary(warm, sim.t, &sim.vehicles, &sim.queues);
    Some(Cost {
        served: served.0 as f64 / served.1.max(1) as f64,
        backlog,
        recovery,
        extra_delay: delay(&with).avg_delay_all - delay(&without).avg_delay_all,
    })
}

fn cost(runs: u64, batch_q: f64, fcfs_q: f64, clear: f64) {
    let span = 900.0;
    println!(
        "\nWhat one failure costs: the same arrivals with and without it, {runs} pairs of runs per row, {span:.0} s"
    );
    println!(
        "after the failure, disabled vehicles cleared away after {clear:.0} s. Fail-safe gaps; a vehicle that stops dead"
    );
    println!("at a random place in the box.");
    println!(
        "{:<28} {:>8} {:>16} {:>14} {:>14} {:>12} {:>14}",
        "schedule",
        "demand",
        "served meanwhile",
        "most held up",
        "back to normal",
        "not by then",
        "delay added"
    );
    for (sched, demand) in [
        (Scheduling::Batches, batch_q * 0.75),
        (Scheduling::Batches, batch_q),
        (Scheduling::Batches, 18_000.0),
        (Scheduling::FirstCome, fcfs_q),
    ] {
        let case =
            Case { sched, policy: GapPolicy::FailSafe, fault: Fault::Dead, worst: false, demand, clear };
        let jobs: Vec<_> =
            (0..runs).map(|k| std::thread::spawn(move || one_cost(case, 9000 + k, span))).collect();
        let res: Vec<Cost> = jobs.into_iter().filter_map(|j| j.join().unwrap()).collect();
        let n = res.len().max(1) as f64;
        let mean = |f: fn(&Cost) -> f64| res.iter().map(f).sum::<f64>() / n;
        let back: Vec<f64> = res.iter().filter_map(|c| c.recovery).collect();
        println!(
            "{:<28} {:>8.0} {:>15.0}% {:>10.0} veh {:>12.0} s {:>12} {:>12.1} s",
            sched.label(),
            demand,
            100.0 * mean(|c| c.served),
            mean(|c| c.backlog),
            back.iter().sum::<f64>() / back.len().max(1) as f64,
            res.len() - back.len(),
            mean(|c| c.extra_delay),
        );
    }
}

fn main() {
    let arg = |i: usize| std::env::args().nth(i).unwrap_or_default();
    let runs: u64 = arg(1).parse().unwrap_or(200);
    let batch_q: f64 = arg(2).parse().unwrap_or(16_000.0);
    let fcfs_q: f64 = arg(3).parse().unwrap_or(8_000.0);
    let clear: f64 = arg(4).parse().unwrap_or(60.0);
    let part = arg(5);
    if part != "cost" {
        safety(runs, batch_q, fcfs_q, clear);
    }
    if part != "safety" {
        cost((runs / 4).max(1), batch_q, fcfs_q, clear);
    }
}
