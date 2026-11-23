// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Headless benchmark: every automatic control strategy on identical arrival streams.
//!
//! usage: bench [layout|all] [minutes] [seeds] [demand-scale] [pattern] [--csv FILE]
//!        bench sensors <layout> [minutes] [seeds] [--csv FILE]   (sensor-package study)
//!
//! Every run measures `minutes` after a 5-minute warm-up, and seed k of every strategy
//! is arrival stream 1000 + k, so the strategies meet the same traffic. The output
//! starts with the source the program was built from and the settings, and each table
//! ends with the weights files the Deep RL and Hybrid controllers loaded, with their
//! fingerprints.
//!
//! Each row of a strategy table gives means over the seeds, except in two columns.
//! `vs Pred*` is the mean difference in person delay from the predictive controller on
//! the same seeds, with the half-width of its 95% confidence interval from two seeds
//! up. `coll` totals the overlaps between vehicle bodies that the safety check counted
//! over the seeds, warm-up included; it should always be 0.
//!
//! `--csv FILE` also writes every run's results to FILE, one row per run, each with the
//! source and settings it ran with.

#[path = "shared/record.rs"]
mod record;

use intersection_lab::control::CtrlKind;
use intersection_lab::sim::demand::{DemandConfig, Pattern};
use intersection_lab::sim::network::{LayoutKind, Network};
use intersection_lab::sim::sensors::{Conditions, SensorConfig};
use intersection_lab::sim::stats::Summary;
use intersection_lab::sim::{ControlMode, Sim};
use intersection_lab::{run_scenario, scenario};
use std::io::Write;
use std::time::Instant;

/// The warm-up before each run's measurement starts (s).
const WARMUP: f64 = 300.0;
/// Seed k of every strategy is arrival stream `SEED_BASE + k`.
const SEED_BASE: u64 = 1000;

/// [`run_scenario`] with the safety check on, so that the overlaps it returns were
/// counted. The check is off in a new simulation, and a strategy table's `coll` column
/// could then only be zero.
fn run_checked(mut sim: Sim, warmup: f64, duration: f64) -> (Summary, u64) {
    sim.check_safety = true;
    run_scenario(sim, warmup, duration)
}

/// One run's results: its summary, the overlaps counted if the safety check was on,
/// and the weights its controller loaded if it is a learned one.
struct Run {
    summary: Summary,
    overlaps: Option<u64>,
    weights: Option<String>,
}

/// Run `sim` for the warm-up and then `minutes`, with or without the safety check.
fn run(mut sim: Sim, minutes: f64, check: bool) -> Run {
    let weights = record::weights(sim.controller.as_mut());
    let (summary, overlaps) = if check {
        run_checked(sim, WARMUP, minutes * 60.0)
    } else {
        run_scenario(sim, WARMUP, minutes * 60.0)
    };
    Run { summary, overlaps: check.then_some(overlaps), weights }
}

/// The sensors a run used, as the output and the CSV describe them.
fn describe(s: &SensorConfig) -> String {
    let mut fitted = vec![];
    if s.loops {
        fitted.push("loops".to_string());
    }
    if s.camera {
        fitted.push("camera".into());
    }
    if s.radar {
        fitted.push("radar".into());
    }
    if s.cv {
        fitted.push(format!("{:.0}% connected vehicles", s.cv_rate * 100.0));
    }
    let mut d = format!(
        "{}, {}, {}",
        fitted.join(" + "),
        if s.realistic { "realistic errors" } else { "ideal" },
        s.conditions.name()
    );
    if s.loop_fault > 0.0 {
        d += &format!(", {:.0}% of loop stations faulty", s.loop_fault * 100.0);
    }
    d
}

/// The seeds of a run, for the settings line.
fn seed_range(seeds: u64) -> String {
    match seeds {
        1 => format!("seed {SEED_BASE}"),
        n => format!("seeds {SEED_BASE} to {}", SEED_BASE + n.saturating_sub(1)),
    }
}

/// The weights each learned controller loaded across `runs`, for the line after a table.
/// A file that changed during the benchmark shows as more than one entry.
fn weights_line(runs: &[(&str, &[Run])]) -> Option<String> {
    let found: Vec<String> = runs
        .iter()
        .filter_map(|(name, rs)| {
            let mut seen: Vec<&str> = vec![];
            for w in rs.iter().filter_map(|r| r.weights.as_deref()) {
                if !seen.contains(&w) {
                    seen.push(w);
                }
            }
            (!seen.is_empty()).then(|| format!("{name} {}", seen.join(", then ")))
        })
        .collect();
    (!found.is_empty()).then(|| format!("weights: {}", found.join("; ")))
}

/// A strategy's person delay against the predictive controller's on the same seeds: the
/// mean difference and, from two seeds up, the half-width of its 95% interval.
fn versus(runs: &[Run], base: &[Run]) -> String {
    let diffs: Vec<f64> =
        runs.iter().zip(base).map(|(r, b)| r.summary.person_delay_all - b.summary.person_delay_all).collect();
    record::paired(&diffs)
}

/// The file `--csv` names.
struct Csv {
    out: std::io::BufWriter<std::fs::File>,
    path: String,
}

/// What every CSV row of one table shares.
struct Table<'a> {
    source: &'a str,
    name: &'a str,
    kind: LayoutKind,
    minutes: f64,
    demand: &'a DemandConfig,
}

const CSV_COLUMNS: &str = "source,table,layout,minutes,warmup_min,demand_scale,pattern,case,sensors,control,\
seed,avg_delay_s,p95_delay_s,throughput_veh_h,avg_stops,person_delay_s,ped_wait_s,unserved,overlaps,weights";

/// A CSV field, quoted when it holds a comma, a quote or a line break.
fn field(f: &str) -> String {
    if f.contains([',', '"', '\n', '\r']) { format!("\"{}\"", f.replace('"', "\"\"")) } else { f.to_string() }
}

impl Csv {
    /// Create `path` and write the column names, or stop with the error.
    fn create(path: &str) -> Csv {
        let mut csv = match std::fs::File::create(path) {
            Ok(f) => Csv { out: std::io::BufWriter::new(f), path: path.to_string() },
            Err(e) => fail(path, e),
        };
        csv.line(CSV_COLUMNS);
        csv
    }

    fn line(&mut self, line: &str) {
        if let Err(e) = writeln!(self.out, "{line}") {
            fail(&self.path, e);
        }
    }

    /// One run of `control` on seed `seed`, in case `case` of table `t`.
    fn row(&mut self, t: &Table, case: &str, sensors: &SensorConfig, control: &str, seed: u64, r: &Run) {
        let s = &r.summary;
        let fields = [
            t.source.to_string(),
            t.name.to_string(),
            t.kind.key().to_string(),
            t.minutes.to_string(),
            (WARMUP / 60.0).to_string(),
            t.demand.scale.to_string(),
            format!("{:?}", t.demand.pattern),
            case.to_string(),
            describe(sensors),
            control.to_string(),
            seed.to_string(),
            s.avg_delay.to_string(),
            s.p95_delay.to_string(),
            s.throughput_vph.to_string(),
            s.avg_stops.to_string(),
            s.person_delay_all.to_string(),
            s.ped_avg_wait.to_string(),
            s.unserved.to_string(),
            r.overlaps.map(|o| o.to_string()).unwrap_or_default(),
            r.weights.clone().unwrap_or_default(),
        ];
        let line: Vec<String> = fields.iter().map(|f| field(f)).collect();
        self.line(&line.join(","));
    }

    fn finish(mut self) {
        if let Err(e) = self.out.flush() {
            fail(&self.path, e);
        }
    }
}

fn fail(path: &str, e: std::io::Error) -> ! {
    eprintln!("bench: can't write {path}: {e}");
    std::process::exit(1);
}

/// How much does each sensor package help each sensor-driven controller, with ideal
/// and field-realistic sensors, across weather, and with failed loop detectors?
fn sensor_study(kind: LayoutKind, minutes: f64, seeds: u64, source: &str, csv: &mut Option<Csv>) {
    let net = Network::build(kind);
    let demand = DemandConfig::default_for(&net);
    let loops = SensorConfig { camera: false, radar: false, cv: false, ..Default::default() };
    let cam = SensorConfig { camera: true, ..loops.clone() };
    let rad = SensorConfig { radar: true, ..cam.clone() };
    let cv30 = SensorConfig { cv: true, cv_rate: 0.3, ..rad.clone() };
    let cv_only = SensorConfig { loops: false, cv: true, cv_rate: 1.0, ..loops.clone() };
    let packages: Vec<(String, SensorConfig)> = vec![
        ("loops".into(), loops.clone()),
        ("loops+camera".into(), cam),
        ("+radar".into(), rad.clone()),
        ("+30% CV".into(), cv30),
        ("100% CV only".into(), cv_only),
        ("+radar IDEAL".into(), rad.clone().ideal()),
    ];
    let t = Table { source, name: "sensor packages", kind, minutes, demand: &demand };
    table(
        &t,
        "Sensor packages (realistic errors, clear day; last column ideal sensors)",
        &packages,
        seeds,
        csv,
    );

    let mut conds: Vec<(String, SensorConfig)> = Conditions::ALL
        .iter()
        .map(|&c| (c.name().to_string(), SensorConfig { conditions: c, ..rad.clone() }))
        .collect();
    conds.push(("loops, 25% faulty".into(), SensorConfig { loop_fault: 0.25, ..loops }));
    conds.push((
        "fog + 25% faulty".into(),
        SensorConfig { conditions: Conditions::Fog, loop_fault: 0.25, ..rad },
    ));
    let t = Table { name: "weather and faults", ..t };
    table(&t, "Weather and detector faults (loops + camera + radar unless noted)", &conds, seeds, csv);
}

fn table(t: &Table, title: &str, packages: &[(String, SensorConfig)], seeds: u64, csv: &mut Option<Csv>) {
    println!(
        "\n=== {} | {title} | person delay incl. unserved (s), {:.0} min x {} seeds ===",
        t.kind.name(),
        t.minutes,
        seeds
    );
    print!("{:<14}", "controller");
    for (n, _) in packages {
        print!(" {:>17}", n);
    }
    println!();
    let mut learned: Vec<(&str, Vec<Run>)> = vec![];
    for c in
        [CtrlKind::Actuated, CtrlKind::MaxPressure, CtrlKind::Predictive, CtrlKind::DeepRl, CtrlKind::Hybrid]
    {
        let hs: Vec<_> = packages
            .iter()
            .map(|(_, s)| {
                let (d, s, kind, minutes) = (t.demand.clone(), s.clone(), t.kind, t.minutes);
                std::thread::spawn(move || -> Vec<Run> {
                    (0..seeds)
                        .map(|k| {
                            run(scenario(kind, ControlMode::Signal, c, &d, &s, SEED_BASE + k), minutes, false)
                        })
                        .collect()
                })
            })
            .collect();
        print!("{:<14}", c.short());
        let mut all = vec![];
        for (h, (case, sensors)) in hs.into_iter().zip(packages) {
            let runs = h.join().unwrap();
            let mean = runs.iter().map(|r| r.summary.person_delay_all).sum::<f64>() / seeds as f64;
            print!(" {:>17.1}", mean);
            if let Some(csv) = csv.as_mut() {
                for (k, r) in runs.iter().enumerate() {
                    csv.row(t, case, sensors, c.name(), SEED_BASE + k as u64, r);
                }
            }
            all.extend(runs);
        }
        println!();
        learned.push((c.short(), all));
    }
    let named: Vec<(&str, &[Run])> = learned.iter().map(|(n, rs)| (*n, rs.as_slice())).collect();
    if let Some(line) = weights_line(&named) {
        println!("{line}");
    }
}

/// Every strategy on table `t`'s layout: the table, and its runs in the CSV.
fn strategies(t: &Table, seeds: u64, csv: &mut Option<Csv>) {
    let (kind, minutes, demand) = (t.kind, t.minutes, t.demand);
    let net = Network::build(kind);
    let sensors = SensorConfig::default();
    println!(
        "\n=== {} | {:.0} min x {} seeds | demand x{:.2} | {:?} | {} conflicts ===",
        kind.name(),
        minutes,
        seeds,
        demand.scale,
        demand.pattern,
        net.conflicts.len()
    );
    println!(
        "{:<34} {:>8} {:>8} {:>8} {:>7} {:>8} {:>11} {:>8} {:>7} {:>6}",
        "control", "delay s", "p95 s", "veh/h", "stops", "persDly*", "vs Pred*", "pedWait", "unserv", "coll"
    );
    let mut strategies: Vec<(String, ControlMode, Option<CtrlKind>)> = vec![];
    for mode in ControlMode::options(kind) {
        if mode == ControlMode::Signal {
            for c in CtrlKind::ALL {
                strategies.push((c.name().to_string(), mode, Some(c)));
            }
        } else {
            strategies.push((mode.name().to_string(), mode, None));
        }
    }
    let handles: Vec<_> = strategies
        .into_iter()
        .map(|(name, mode, c)| {
            let demand = demand.clone();
            let sensors = sensors.clone();
            std::thread::spawn(move || {
                let t0 = Instant::now();
                let runs: Vec<Run> = (0..seeds)
                    .map(|k| {
                        let ctrl = c.unwrap_or(CtrlKind::Fixed);
                        run(scenario(kind, mode, ctrl, &demand, &sensors, SEED_BASE + k), minutes, true)
                    })
                    .collect();
                (name, c, runs, t0.elapsed().as_secs_f64())
            })
        })
        .collect();
    let done: Vec<_> = handles.into_iter().map(|h| h.join().unwrap()).collect();
    let base = done.iter().find(|d| d.1 == Some(CtrlKind::Predictive)).map(|d| &d.2);
    for (name, c, runs, secs) in &done {
        let mut acc = [0.0f64; 8];
        for r in runs {
            let s = &r.summary;
            acc[0] += s.avg_delay;
            acc[1] += s.p95_delay;
            acc[2] += s.throughput_vph;
            acc[3] += s.avg_stops;
            acc[4] += s.person_delay_all;
            acc[5] += s.ped_avg_wait;
            acc[6] += s.unserved as f64;
            acc[7] += r.overlaps.unwrap_or(0) as f64;
        }
        // Means over the seeds, except the overlaps: a total, so that one overlap in one
        // run can't round away.
        for a in acc.iter_mut().take(7) {
            *a /= seeds as f64;
        }
        let vs = match base {
            _ if *c == Some(CtrlKind::Predictive) => "(base)".to_string(),
            Some(b) => versus(runs, b),
            None => "-".to_string(),
        };
        println!(
            "{:<34} {:>8.1} {:>8.1} {:>8.0} {:>7.2} {:>8.1} {:>11} {:>8.1} {:>7.0} {:>6.0}   ({:.1}s)",
            name, acc[0], acc[1], acc[2], acc[3], acc[4], vs, acc[5], acc[6], acc[7], secs
        );
        if let Some(csv) = csv.as_mut() {
            for (k, r) in runs.iter().enumerate() {
                csv.row(t, "", &sensors, name, SEED_BASE + k as u64, r);
            }
        }
    }
    let named: Vec<(&str, &[Run])> =
        done.iter().filter_map(|(_, c, rs, _)| c.map(|c| (c.short(), rs.as_slice()))).collect();
    if let Some(line) = weights_line(&named) {
        println!("{line}");
    }
}

fn main() {
    // `--csv FILE` may come anywhere; the other arguments keep their places.
    let mut args: Vec<String> = vec![];
    let mut csv_path = None;
    let mut given = std::env::args().skip(1);
    while let Some(a) = given.next() {
        if a == "--csv" {
            match given.next() {
                Some(p) => csv_path = Some(p),
                None => {
                    eprintln!("bench: --csv needs a file name");
                    std::process::exit(2);
                }
            }
        } else {
            args.push(a);
        }
    }
    let mut csv = csv_path.as_deref().map(Csv::create);
    let source = record::source();
    println!("{source}");
    if args.first().map(|s| s.as_str()) == Some("sensors") {
        let which = args.get(1).map(|s| s.as_str()).unwrap_or("arterial4");
        let minutes: f64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(20.0);
        let seeds: u64 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(3);
        println!("{:.0}-minute warm-up before each run; {}", WARMUP / 60.0, seed_range(seeds));
        for k in
            LayoutKind::ALL.iter().copied().filter(|k| k.signalized() && (which == "all" || k.key() == which))
        {
            sensor_study(k, minutes, seeds, &source, &mut csv);
        }
    } else {
        let which = args.first().map(|s| s.as_str()).unwrap_or("all");
        let minutes: f64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(20.0);
        let seeds: u64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(3);
        let scale: f64 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(1.0);
        let pattern = match args.get(4).map(|s| s.as_str()) {
            Some("platoons") => Pattern::Platoons,
            Some("rush") => Pattern::RushHour,
            Some("surges") => Pattern::Surges,
            _ => Pattern::Random,
        };
        println!(
            "{:.0}-minute warm-up before each run; {}; sensors: {}",
            WARMUP / 60.0,
            seed_range(seeds),
            describe(&SensorConfig::default())
        );
        for kind in LayoutKind::ALL.iter().copied().filter(|k| which == "all" || k.key() == which) {
            let mut demand = DemandConfig::default_for(&Network::build(kind));
            demand.scale = scale;
            demand.pattern = pattern;
            strategies(
                &Table { source: &source, name: "strategies", kind, minutes, demand: &demand },
                seeds,
                &mut csv,
            );
        }
    }
    if let Some(csv) = csv {
        csv.finish();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The T-intersection after a minute, with one vehicle moved on top of another in
    /// its lane.
    fn overlapping() -> Sim {
        let kind = LayoutKind::Tee;
        let demand = DemandConfig::default_for(&Network::build(kind));
        let mut sim =
            scenario(kind, ControlMode::Signal, CtrlKind::Fixed, &demand, &SensorConfig::default(), 1000);
        sim.run_for(60.0);
        let n = sim.vehicles.len();
        let (one, other) = (0..n)
            .flat_map(|i| (i + 1..n).map(move |j| (i, j)))
            .find(|&(i, j)| sim.vehicles[i].link() == sim.vehicles[j].link())
            .expect("two vehicles in one lane");
        sim.vehicles[other].s = sim.vehicles[one].s;
        sim
    }

    #[test]
    fn a_strategy_run_counts_overlaps() {
        let (_, coll) = run_checked(overlapping(), 0.0, 1.0);
        assert!(coll > 0, "two vehicles overlap, and the `coll` column would show 0");
    }

    #[test]
    fn csv_fields_are_quoted_only_when_needed() {
        assert_eq!(field("loops + camera"), "loops + camera");
        assert_eq!(field("loops, 25% faulty"), "\"loops, 25% faulty\"");
        assert_eq!(field("a \"b\""), "\"a \"\"b\"\"\"");
    }

    #[test]
    fn the_paired_column_needs_two_seeds_for_an_interval() {
        let runs = |pd: &[f64]| -> Vec<Run> {
            pd.iter()
                .map(|&p| Run {
                    summary: Summary { person_delay_all: p, ..Default::default() },
                    overlaps: None,
                    weights: None,
                })
                .collect()
        };
        assert_eq!(versus(&runs(&[10.0]), &runs(&[12.5])), "-2.5");
        // Differences 1, 2 and 3: mean 2, half-width 4.303 / sqrt(3) = 2.48.
        assert_eq!(versus(&runs(&[11.0, 12.0, 13.0]), &runs(&[10.0; 3])), "+2.0±2.5");
    }
}
