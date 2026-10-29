// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Headless benchmark: every controller on identical arrival streams.
//!
//! usage: bench [layout|all] [minutes] [seeds] [demand-scale] [pattern]
//!        bench sensors <layout> [minutes] [seeds]   (sensor-package study)

use std::time::Instant;
use intersection_lab::control::CtrlKind;
use intersection_lab::run_scenario;
use intersection_lab::sim::demand::{DemandConfig, Pattern};
use intersection_lab::sim::network::{LayoutKind, Network};
use intersection_lab::sim::sensors::{Conditions, SensorConfig};
use intersection_lab::sim::ControlMode;

/// How much does each sensor package help each sensor-driven controller, with ideal
/// and field-realistic sensors, across weather, and with failed loop detectors?
fn sensor_study(kind: LayoutKind, minutes: f64, seeds: u64) {
    let net = Network::build(kind);
    let demand = DemandConfig::default_for(&net);
    let mut loops = SensorConfig::default();
    loops.camera = false;
    loops.radar = false;
    loops.cv = false;
    let mut cam = loops.clone();
    cam.camera = true;
    let mut rad = cam.clone();
    rad.radar = true;
    let mut cv30 = rad.clone();
    cv30.cv = true;
    cv30.cv_rate = 0.3;
    let mut cv_only = loops.clone();
    cv_only.loops = false;
    cv_only.cv = true;
    cv_only.cv_rate = 1.0;
    let packages: Vec<(String, SensorConfig)> = vec![
        ("loops".into(), loops.clone()),
        ("loops+camera".into(), cam.clone()),
        ("+radar".into(), rad.clone()),
        ("+30% CV".into(), cv30),
        ("100% CV only".into(), cv_only),
        ("+radar IDEAL".into(), rad.clone().ideal()),
    ];
    let title = format!("Sensor packages (realistic errors, clear day; last column ideal sensors)");
    table(kind, &title, &packages, &demand, minutes, seeds);

    let mut conds: Vec<(String, SensorConfig)> = Conditions::ALL
        .iter()
        .map(|&c| {
            let mut s = rad.clone();
            s.conditions = c;
            (c.name().to_string(), s)
        })
        .collect();
    let mut faulty = loops.clone();
    faulty.loop_fault = 0.25;
    conds.push(("loops, 25% faulty".into(), faulty));
    let mut fog_faulty = rad.clone();
    fog_faulty.conditions = Conditions::Fog;
    fog_faulty.loop_fault = 0.25;
    conds.push(("fog + 25% faulty".into(), fog_faulty));
    table(kind, "Weather and detector faults (loops + camera + radar unless noted)", &conds, &demand, minutes, seeds);
}

fn table(kind: LayoutKind, title: &str, packages: &[(String, SensorConfig)], demand: &DemandConfig, minutes: f64, seeds: u64) {
    println!("\n=== {} | {title} | person delay incl. unserved (s), {:.0} min x {} seeds ===", kind.name(), minutes, seeds);
    print!("{:<14}", "controller");
    for (n, _) in packages {
        print!(" {:>17}", n);
    }
    println!();
    for c in [CtrlKind::Actuated, CtrlKind::MaxPressure, CtrlKind::Predictive, CtrlKind::DeepRl, CtrlKind::Hybrid] {
        let hs: Vec<_> = packages
            .iter()
            .map(|(_, s)| {
                let (d, s) = (demand.clone(), s.clone());
                std::thread::spawn(move || {
                    (0..seeds)
                        .map(|k| run_scenario(kind, ControlMode::Signal, c, &d, &s, 1000 + k, 300.0, minutes * 60.0).0.person_delay_all)
                        .sum::<f64>()
                        / seeds as f64
                })
            })
            .collect();
        print!("{:<14}", c.short());
        for h in hs {
            print!(" {:>17.1}", h.join().unwrap());
        }
        println!();
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.get(1).map(|s| s.as_str()) == Some("sensors") {
        let which = args.get(2).map(|s| s.as_str()).unwrap_or("arterial4");
        let minutes: f64 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(20.0);
        let seeds: u64 = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(3);
        for k in LayoutKind::ALL.iter().copied().filter(|k| k.signalized() && (which == "all" || k.key() == which)) {
            sensor_study(k, minutes, seeds);
        }
        return;
    }
    let which = args.get(1).map(|s| s.as_str()).unwrap_or("all");
    let minutes: f64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(20.0);
    let seeds: u64 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(3);
    let scale: f64 = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(1.0);
    let pattern = match args.get(5).map(|s| s.as_str()) {
        Some("platoons") => Pattern::Platoons,
        Some("rush") => Pattern::RushHour,
        Some("surges") => Pattern::Surges,
        _ => Pattern::Random,
    };
    let layouts: Vec<LayoutKind> =
        LayoutKind::ALL.iter().copied().filter(|k| which == "all" || k.key() == which).collect();
    for kind in layouts {
        let net = Network::build(kind);
        let mut demand = DemandConfig::default_for(&net);
        demand.scale = scale;
        demand.pattern = pattern;
        let sensors = SensorConfig::default();
        println!(
            "\n=== {} | {:.0} min x {} seeds | demand x{:.2} | {:?} | {} conflicts ===",
            kind.name(),
            minutes,
            seeds,
            scale,
            pattern,
            net.conflicts.len()
        );
        println!(
            "{:<34} {:>8} {:>8} {:>8} {:>7} {:>8} {:>8} {:>7} {:>6}",
            "control", "delay s", "p95 s", "veh/h", "stops", "persDly*", "pedWait", "unserv", "coll"
        );
        let mut runs: Vec<(String, ControlMode, Option<CtrlKind>)> = vec![];
        for mode in ControlMode::options(kind) {
            if mode == ControlMode::Signal {
                for c in CtrlKind::ALL {
                    runs.push((c.name().to_string(), mode, Some(c)));
                }
            } else {
                runs.push((mode.name().to_string(), mode, None));
            }
        }
        let handles: Vec<_> = runs
            .into_iter()
            .map(|(name, mode, c)| {
                let demand = demand.clone();
                let sensors = sensors.clone();
                std::thread::spawn(move || {
                    let t0 = Instant::now();
                    let mut acc = [0.0f64; 8];
                    for seed in 0..seeds {
                        let (s, coll) = run_scenario(
                            kind,
                            mode,
                            c.unwrap_or(CtrlKind::Fixed),
                            &demand,
                            &sensors,
                            1000 + seed,
                            300.0,
                            minutes * 60.0,
                        );
                        acc[0] += s.avg_delay;
                        acc[1] += s.p95_delay;
                        acc[2] += s.throughput_vph;
                        acc[3] += s.avg_stops;
                        acc[4] += s.person_delay_all;
                        acc[5] += s.ped_avg_wait;
                        acc[6] += s.unserved as f64;
                        acc[7] += coll as f64;
                    }
                    for a in acc.iter_mut() {
                        *a /= seeds as f64;
                    }
                    (name, acc, t0.elapsed().as_secs_f64())
                })
            })
            .collect();
        for h in handles {
            let (name, a, secs) = h.join().unwrap();
            println!(
                "{:<34} {:>8.1} {:>8.1} {:>8.0} {:>7.2} {:>8.1} {:>8.1} {:>7.0} {:>6.0}   ({:.1}s)",
                name, a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], secs
            );
        }
    }
}
