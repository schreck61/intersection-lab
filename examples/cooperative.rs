// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Capacity of the signal-free cooperative crossing.
//!
//! Runs the intersection manager at rising demand and reports what the crossing
//! actually serves, the delay, and what the safety monitors measured.
//!
//! ```text
//! cargo run --release --example cooperative -- [policy] [left lanes] [fleet] [seconds] [seeds] [demands] [arrivals] [scheduling]
//! ```
//!
//! * policy: `p2` (fail-safe, default), `ph` (mitigated) or `p1` (nominal)
//! * left lanes: 2 (default) or 1
//! * fleet: `mix` (the default vehicle mix, default) or `cars`
//! * seconds: measured time per run after a 120 s warm-up (default 900)
//! * seeds: runs per demand level (default 1)
//! * demands: comma-separated total demand levels in veh/h (default 8000 to 20000)
//! * arrivals: `random` (Poisson, default), `platoons`, `rush` or `surges`
//! * scheduling: `batches` (default) or `fcfs` (first come, first served: one vehicle at
//!   a time, each in the earliest gap that fits)

use intersection_lab::control::CtrlKind;
use intersection_lab::sim::coop::{CoopStats, GapPolicy, Scheduling};
use intersection_lab::sim::demand::{DemandConfig, Pattern};
use intersection_lab::sim::network::{Turn, build_cooperative};
use intersection_lab::sim::{ControlMode, Sim};

fn main() {
    let arg = |i: usize| std::env::args().nth(i).unwrap_or_default();
    let policy = match arg(1).as_str() {
        "ph" => GapPolicy::Mitigated,
        "p1" => GapPolicy::Nominal,
        _ => GapPolicy::FailSafe,
    };
    let n_left: usize = arg(2).parse().unwrap_or(2);
    let cars = arg(3) == "cars";
    let secs: f64 = arg(4).parse().unwrap_or(900.0);
    let seeds: u64 = arg(5).parse().unwrap_or(1);
    let pattern = match arg(7).as_str() {
        "platoons" => Pattern::Platoons,
        "rush" => Pattern::RushHour,
        "surges" => Pattern::Surges,
        _ => Pattern::Random,
    };
    let scheduling = if arg(8) == "fcfs" { Scheduling::FirstCome } else { Scheduling::Batches };
    let warm = 120.0;
    println!(
        "policy {} ({}), {}, {} left lane(s), {} fleet, {} arrivals, {secs:.0} s after {warm:.0} s warm-up, {seeds} seed(s)",
        policy.short(),
        policy.name(),
        scheduling.label(),
        n_left,
        if cars { "all-car" } else { "mixed" },
        pattern.name()
    );
    println!(
        "{:>9} {:>9} {:>9} {:>8} {:>8} {:>8} {:>7} {:>8} {:>8} {:>6} {:>6} {:>6}",
        "demand", "arrived", "served", "box/h", "delay", "p95", "maxwait", "queued", "PETmin", "viol", "dev", "overlap"
    );
    let demands: Vec<f64> = match arg(6) {
        a if !a.is_empty() => a.split(',').filter_map(|x| x.parse().ok()).collect(),
        _ => vec![8000.0, 10000.0, 12000.0, 14000.0, 15000.0, 16000.0, 17000.0, 18000.0, 19000.0, 20000.0],
    };
    for q in demands {
        let jobs: Vec<_> = (0..seeds)
            .map(|seed| {
                std::thread::spawn(move || {
                    let net = build_cooperative(n_left);
                    let mut demand = DemandConfig::default_for(&net);
                    demand.rate = [q / 4.0; 4];
                    demand.pattern = pattern;
                    if cars {
                        demand.mix = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0];
                    }
                    let mut sim = Sim::with_demand(net, ControlMode::Cooperative, CtrlKind::Fixed, demand, 7000 + seed);
                    sim.set_gap_policy(policy);
                    sim.set_scheduling(scheduling);
                    sim.check_safety = true;
                    sim.run_for(warm + secs);
                    let s = sim.stats.summary(warm, sim.t, &sim.vehicles, &sim.queues);
                    let boxed = (0..s.by_move.len())
                        .filter(|&m| sim.net.movements[m].turn != Turn::R)
                        .map(|m| s.by_move[m].n as f64)
                        .sum::<f64>()
                        * 3600.0
                        / s.duration.max(1.0);
                    Run {
                        arrived: sim.stats.generated as f64 * 3600.0 / (warm + secs),
                        served: s.throughput_vph,
                        boxed,
                        delay: s.avg_delay,
                        p95: s.p95_delay,
                        max_wait: s.max_delay,
                        queued: sim.queues.iter().map(|x| x.len()).sum::<usize>() as f64,
                        stats: sim.coop.as_ref().unwrap().stats.clone(),
                        overlaps: sim.collisions,
                    }
                })
            })
            .collect();
        let res: Vec<Run> = jobs.into_iter().map(|j| j.join().unwrap()).collect();
        let n = res.len() as f64;
        let avg = |f: fn(&Run) -> f64| res.iter().map(f).sum::<f64>() / n;
        let pet = res.iter().map(|r| r.stats.pet_min).fold(f64::INFINITY, f64::min);
        let viol: u64 = res.iter().map(|r| r.stats.zone_violations + r.stats.unauthorized).sum();
        let dev: u64 = res.iter().map(|r| r.stats.deviations).sum();
        let ov: u64 = res.iter().map(|r| r.overlaps).sum();
        println!(
            "{:>9.0} {:>9.0} {:>9.0} {:>8.0} {:>8.1} {:>8.1} {:>7.0} {:>8.0} {:>8.2} {:>6} {:>6} {:>6}",
            q,
            avg(|r| r.arrived),
            avg(|r| r.served),
            avg(|r| r.boxed),
            avg(|r| r.delay),
            avg(|r| r.p95),
            avg(|r| r.max_wait),
            avg(|r| r.queued),
            pet,
            viol,
            dev,
            ov
        );
    }
}

/// What one run measured (rates in veh/h, times in s).
struct Run {
    arrived: f64,
    served: f64,
    /// Vehicles through the box itself (right turns use the slip lanes).
    boxed: f64,
    delay: f64,
    p95: f64,
    max_wait: f64,
    /// Vehicles still waiting to enter at the end.
    queued: f64,
    stats: CoopStats,
    overlaps: u64,
}
