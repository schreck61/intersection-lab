// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Stress runs: half an hour at a time on every layout and control mode, with an
//! emergency vehicle sent through every 90 s, more pedestrians or more traffic than
//! usual, checked for overlaps and for vehicles stuck in the box.
//!
//! ```text
//! cargo run --release --example stress -- [first seed] [last seed] [layout] [scale] [options]
//! ```
//!
//! * first seed, last seed: the seeds to run, both included (default 1 and 20)
//! * layout: `arterial4`, `majorminor4`, `town4`, `tee`, `roundabout`, `cooperative` or
//!   `all` (default)
//! * scale: demand as a multiple of the default (default 1)
//! * options, comma-separated:
//!   * `ev`: an emergency vehicle sent straight through every 90 s from t = 60 s, from
//!     the leg the step count picks, `(steps / 7) % legs`, or none when that leg has
//!     no through movement
//!   * `peds`: pedestrians four times as often as by default, 240 an hour at 1× (the
//!     roundabout and the cooperative crossing have no crosswalks)
//!   * `both`: each seed runs with each of the other options given both off and on, so
//!     `ev,peds,both` makes four runs of it
//!
//! Each layout runs under each of its control modes, and a signal under each automatic
//! controller. A run lasts 1,800 s with the safety check on, and locks up if a vehicle
//! stands still (below 0.05 m/s) in the box, on a connector or on a roundabout's entry,
//! ring or exit, for over 300 s. For each layout, control mode and controller, the table
//! gives the runs, how many locked up and how many had overlaps, the emergency vehicles
//! that got through of those sent, and the longest any vehicle stood still in the box.
//! Each run that locked up or overlapped is listed after it.
//! `no_overlap_where_two_paths_merge` and `no_gridlock_on_the_roundabout` in
//! `tests/safety.rs` replay runs like these that once overlapped or locked up.

use intersection_lab::control::CtrlKind;
use intersection_lab::sim::demand::DemandConfig;
use intersection_lab::sim::network::{LayoutKind, Network, Turn};
use intersection_lab::sim::vtypes::VKind;
use intersection_lab::sim::{ControlMode, Sim};
use std::collections::HashMap;
use std::sync::atomic::{AtomicUsize, Ordering};

/// How long each run lasts (s).
const SECS: f64 = 1800.0;
/// How long a vehicle may stand still in the box before its run counts as locked up (s).
const LOCK_UP: f64 = 300.0;

/// A layout, one of its control modes, and the controller when it has a signal.
#[derive(Clone, Copy)]
struct Setup {
    kind: LayoutKind,
    mode: ControlMode,
    ctrl: CtrlKind,
}

impl Setup {
    /// The controller, if there is a signal for it to run.
    fn controller(self) -> Option<&'static str> {
        (self.mode == ControlMode::Signal).then(|| self.ctrl.short())
    }
}

/// One run: a setup, a seed, and whether emergency vehicles and extra pedestrians come.
#[derive(Clone, Copy)]
struct Job {
    setup: usize,
    seed: u64,
    ev: bool,
    peds: bool,
}

impl Job {
    /// What this run adds to ordinary traffic, for the list of runs that went wrong.
    fn options(self) -> &'static str {
        match (self.ev, self.peds) {
            (false, false) => "",
            (true, false) => ", emergency vehicles",
            (false, true) => ", more pedestrians",
            (true, true) => ", emergency vehicles, more pedestrians",
        }
    }
}

/// What one run found.
struct Outcome {
    /// Steps on which vehicles overlapped, and the first overlap.
    overlaps: u64,
    first_overlap: String,
    /// Emergency vehicles sent, and how many of them got through.
    sent: usize,
    through: usize,
    /// The longest any vehicle stood still in the box (s).
    longest: f64,
}

fn run(s: Setup, job: Job, scale: f64) -> Outcome {
    let net = Network::build(s.kind);
    let mut demand = DemandConfig::default_for(&net);
    demand.scale = scale;
    let mut sim = Sim::with_demand(net, s.mode, s.ctrl, demand, job.seed);
    sim.check_safety = true;
    if job.peds {
        sim.demand.ped_rate = 240.0;
    }
    let (mut sent, mut next) = (0, 60.0);
    // How long each vehicle in the box has been standing still.
    let mut still: HashMap<u64, f64> = HashMap::new();
    let mut longest: f64 = 0.0;
    while sim.t < SECS {
        if job.ev && sim.t >= next {
            let leg = (sim.steps as usize / 7) % sim.net.legs.len();
            if sim.spawn_manual(leg, VKind::Emergency, Turn::T) {
                sent += 1;
            }
            next = sim.t + 90.0;
        }
        sim.step();
        for v in &sim.vehicles {
            let t = still.entry(v.id).or_insert(0.0);
            *t = if v.v < 0.05 && sim.net.links[v.link()].kind.in_box() { *t + sim.dt } else { 0.0 };
            longest = longest.max(*t);
        }
    }
    Outcome {
        overlaps: sim.collisions,
        first_overlap: sim.collision_log.first().cloned().unwrap_or_default(),
        sent,
        through: sim.stats.done.iter().filter(|d| d.kind == VKind::Emergency).count(),
        longest,
    }
}

/// The totals in one row of the table.
#[derive(Default)]
struct Row {
    runs: usize,
    locked_up: usize,
    overlapped: usize,
    sent: usize,
    through: usize,
    longest: f64,
}

impl Row {
    fn add(&mut self, o: &Outcome) {
        self.runs += 1;
        self.locked_up += usize::from(o.longest > LOCK_UP);
        self.overlapped += usize::from(o.overlaps > 0);
        self.sent += o.sent;
        self.through += o.through;
        self.longest = self.longest.max(o.longest);
    }

    fn cells(&self) -> String {
        let emergency = format!("{}/{}", self.through, self.sent);
        format!(
            "{:>6} {:>10} {:>11} {emergency:>23} {:>18.0} s",
            self.runs, self.locked_up, self.overlapped, self.longest
        )
    }
}

/// Say what is wrong with the command line, show how to use it, and stop.
fn usage(problem: &str) -> ! {
    eprintln!("{problem}");
    eprintln!("usage: stress [first seed] [last seed] [layout|all] [scale] [ev,peds,both]");
    std::process::exit(2);
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let arg = |i: usize| args.get(i).map(String::as_str);
    let seed_arg = |i: usize, default: u64| {
        arg(i).map_or(default, |a| a.parse().unwrap_or_else(|_| usage(&format!("not a seed: {a}"))))
    };
    let (first, last) = (seed_arg(0, 1), seed_arg(1, 20));
    if first > last {
        usage("the first seed comes after the last");
    }
    let layout = arg(2).unwrap_or("all");
    let kinds: Vec<LayoutKind> =
        LayoutKind::ALL.into_iter().filter(|k| layout == "all" || k.key() == layout).collect();
    if kinds.is_empty() {
        usage(&format!("no layout is called {layout}"));
    }
    let scale: f64 =
        arg(3).map_or(1.0, |a| a.parse().unwrap_or_else(|_| usage(&format!("not a demand scale: {a}"))));
    let options: Vec<&str> = arg(4).unwrap_or_default().split(',').filter(|o| !o.is_empty()).collect();
    if let Some(o) = options.iter().find(|o| !["ev", "peds", "both"].contains(o)) {
        usage(&format!("no option is called {o}"));
    }
    let given = |o: &str| options.contains(&o);
    // The settings of an option to run: off and on with `both`.
    let settings = |o: &str| if given(o) && given("both") { vec![false, true] } else { vec![given(o)] };

    let mut setups = vec![];
    for kind in kinds {
        for mode in ControlMode::options(kind) {
            let ctrls: Vec<CtrlKind> =
                if mode == ControlMode::Signal { CtrlKind::ALL.to_vec() } else { vec![CtrlKind::Fixed] };
            setups.extend(ctrls.into_iter().map(|ctrl| Setup { kind, mode, ctrl }));
        }
    }
    let mut jobs = vec![];
    for setup in 0..setups.len() {
        for seed in first..=last {
            for &ev in &settings("ev") {
                for &peds in &settings("peds") {
                    jobs.push(Job { setup, seed, ev, peds });
                }
            }
        }
    }

    let mut extras = String::new();
    for (o, text) in [("ev", "an emergency vehicle every 90 s"), ("peds", "more pedestrians")] {
        if given(o) {
            extras += &format!(", {text}{}", if given("both") { " (off and on)" } else { "" });
        }
    }
    println!(
        "{} runs of {SECS:.0} s with the safety check on: seeds {first} to {last}, {layout}, demand ×{scale}{extras}",
        jobs.len()
    );
    let threads = std::thread::available_parallelism().map_or(4, |n| n.get()).min(jobs.len());
    eprintln!("running on {threads} threads");

    // Each worker takes the next run nobody has started until none are left. Every run
    // depends only on its own settings and seed, so the results don't depend on which
    // thread ran what.
    let next = AtomicUsize::new(0);
    let mut outcomes: Vec<(usize, Outcome)> = std::thread::scope(|scope| {
        let workers: Vec<_> = (0..threads)
            .map(|_| {
                scope.spawn(|| {
                    let mut done = vec![];
                    loop {
                        let i = next.fetch_add(1, Ordering::Relaxed);
                        let Some(&job) = jobs.get(i) else { break };
                        done.push((i, run(setups[job.setup], job, scale)));
                    }
                    done
                })
            })
            .collect();
        workers.into_iter().flat_map(|w| w.join().unwrap()).collect()
    });
    outcomes.sort_by_key(|(i, _)| *i);

    let mut rows: Vec<Row> = setups.iter().map(|_| Row::default()).collect();
    let mut all = Row::default();
    let mut problems = vec![];
    for (job, (_, o)) in jobs.iter().zip(&outcomes) {
        rows[job.setup].add(o);
        all.add(o);
        let s = setups[job.setup];
        let mut found = vec![];
        if o.longest > LOCK_UP {
            found.push(format!("a vehicle stood still in the box for {:.0} s", o.longest));
        }
        if o.overlaps > 0 {
            found.push(format!("{} steps with overlaps, the first at {}", o.overlaps, o.first_overlap));
        }
        if !found.is_empty() {
            let ctrl = s.controller().map(|c| format!(" / {c}")).unwrap_or_default();
            problems.push(format!(
                "  {} / {}{ctrl}, seed {}{}: {}",
                s.kind.key(),
                s.mode.name(),
                job.seed,
                job.options(),
                found.join("; ")
            ));
        }
    }

    println!(
        "{:<12} {:<32} {:<12} {:>6} {:>10} {:>11} {:>23} {:>20}",
        "layout",
        "control mode",
        "controller",
        "runs",
        "locked up",
        "overlapped",
        "emergency through/sent",
        "longest standstill"
    );
    for (s, row) in setups.iter().zip(&rows) {
        println!(
            "{:<12} {:<32} {:<12} {}",
            s.kind.key(),
            s.mode.name(),
            s.controller().unwrap_or("-"),
            row.cells()
        );
    }
    println!("{:<58} {}", "all", all.cells());
    if problems.is_empty() {
        println!("No run locked up or overlapped.");
    } else {
        println!("Runs that locked up or overlapped:");
        for p in problems {
            println!("{p}");
        }
    }
}
