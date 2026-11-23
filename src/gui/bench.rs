// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Benchmark runs behind *Compare controllers* and *Sensor study*: every automatic
//! strategy on the same arrival streams, averaged over a few seeds. The desktop app
//! runs the strategies in parallel on threads; a browser page has a single thread, so
//! there they run one after another, a slice of each frame at a time ([`Sliced`]). The
//! sliced runner is built for native tests too, which check that it gives the threaded
//! runner's results.

use crate::control::CtrlKind;
use crate::sim::coop::{GapPolicy, Scheduling};
use crate::sim::demand::DemandConfig;
use crate::sim::network::LayoutKind;
use crate::sim::sensors::SensorConfig;
use crate::sim::stats::Summary;
use crate::sim::{ControlMode, Sim};

/// How many arrival streams (seeds) each strategy runs on, and how long each run is.
#[derive(Clone, Copy, Debug)]
pub struct Plan {
    pub seeds: u64,
    pub warmup: f64,
    pub duration: f64,
}

/// The plan the app runs.
pub const PLAN: Plan = Plan { seeds: 3, warmup: 300.0, duration: 1200.0 };

/// One strategy to benchmark.
pub struct Job {
    pub label: String,
    pub mode: ControlMode,
    pub ctrl: CtrlKind,
    pub sensors: SensorConfig,
    /// Cooperative crossing: gap policy, scheduling, maximum cycle and batch join window.
    pub coop: Option<(GapPolicy, Scheduling, f64, f64)>,
}

impl Job {
    /// A fresh simulation of this strategy on arrival stream `seed`.
    fn sim(&self, kind: LayoutKind, demand: &DemandConfig, seed: u64) -> Sim {
        let mut sim = crate::scenario(kind, self.mode, self.ctrl, demand, &self.sensors, 500 + seed);
        if let Some((policy, sched, cycle, slack)) = self.coop {
            sim.set_gap_policy(policy);
            sim.set_scheduling(sched);
            if let Some(c) = sim.coop.as_mut() {
                c.max_cycle = cycle;
                c.slack = slack;
            }
        }
        sim
    }

    /// This strategy's row, run straight through: what each of the desktop app's
    /// threads computes.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn row(self, kind: LayoutKind, demand: &DemandConfig, plan: Plan) -> BenchRow {
        let runs = (0..plan.seeds)
            .map(|seed| crate::run_scenario(self.sim(kind, demand, seed), plan.warmup, plan.duration).0)
            .collect();
        BenchRow { label: self.label, summary: average(runs) }
    }
}

pub struct BenchRow {
    pub label: String,
    pub summary: Summary,
}

pub struct Bench {
    pub title: String,
    pub total: usize,
    pub rows: Vec<Option<BenchRow>>,
    #[cfg(not(target_arch = "wasm32"))]
    rx: std::sync::mpsc::Receiver<(usize, BenchRow)>,
    #[cfg(target_arch = "wasm32")]
    queue: Sliced,
}

/// The mean of each run's summary (maximum delay: the worst run).
fn average(runs: Vec<Summary>) -> Summary {
    let k = runs.len() as f64;
    let mut runs = runs.into_iter();
    let mut a = runs.next().expect("at least one run");
    for s in runs {
        a.avg_delay += s.avg_delay;
        a.p95_delay += s.p95_delay;
        a.throughput_vph += s.throughput_vph;
        a.avg_stops += s.avg_stops;
        a.person_delay_all += s.person_delay_all;
        a.co2_per_veh += s.co2_per_veh;
        a.ped_avg_wait += s.ped_avg_wait;
        a.max_delay = a.max_delay.max(s.max_delay);
    }
    a.avg_delay /= k;
    a.p95_delay /= k;
    a.throughput_vph /= k;
    a.avg_stops /= k;
    a.person_delay_all /= k;
    a.co2_per_veh /= k;
    a.ped_avg_wait /= k;
    a
}

#[cfg(not(target_arch = "wasm32"))]
impl Bench {
    pub fn start(title: String, kind: LayoutKind, demand: &DemandConfig, jobs: Vec<Job>) -> Self {
        let (tx, rx) = std::sync::mpsc::channel();
        let total = jobs.len();
        for (i, job) in jobs.into_iter().enumerate() {
            let tx = tx.clone();
            let demand = demand.clone();
            std::thread::spawn(move || {
                tx.send((i, job.row(kind, &demand, PLAN))).ok();
            });
        }
        Bench { title, total, rows: (0..total).map(|_| None).collect(), rx }
    }

    /// Collect the strategies that have finished.
    pub fn poll(&mut self, _ctx: &egui::Context) {
        while let Ok((i, row)) = self.rx.try_recv() {
            self.rows[i] = Some(row);
        }
    }
}

#[cfg(target_arch = "wasm32")]
impl Bench {
    /// Milliseconds of each frame given to the benchmark.
    const SLICE_MS: u128 = 12;

    pub fn start(title: String, kind: LayoutKind, demand: &DemandConfig, jobs: Vec<Job>) -> Self {
        let total = jobs.len();
        let queue = Sliced::new(kind, demand, jobs, PLAN);
        Bench { title, total, rows: (0..total).map(|_| None).collect(), queue }
    }

    /// Advance the benchmark for a slice of this frame.
    pub fn poll(&mut self, ctx: &egui::Context) {
        let t0 = web_time::Instant::now();
        if !self.queue.advance(&mut self.rows, || t0.elapsed().as_millis() < Self::SLICE_MS) {
            ctx.request_repaint();
        }
    }
}

/// The strategies run one after another, a few steps at a time, as a browser page runs
/// them between frames.
#[cfg(any(target_arch = "wasm32", test))]
pub struct Sliced {
    kind: LayoutKind,
    demand: DemandConfig,
    plan: Plan,
    jobs: std::collections::VecDeque<(usize, Job)>,
    current: Option<Run>,
}

/// One seed of one strategy, part way through.
#[cfg(any(target_arch = "wasm32", test))]
struct Run {
    index: usize,
    job: Job,
    seed: u64,
    sim: Sim,
    steps_left: usize,
    /// Summaries of the strategy's earlier seeds.
    done: Vec<Summary>,
}

#[cfg(any(target_arch = "wasm32", test))]
impl Sliced {
    pub fn new(kind: LayoutKind, demand: &DemandConfig, jobs: Vec<Job>, plan: Plan) -> Self {
        let jobs = jobs.into_iter().enumerate().collect();
        Sliced { kind, demand: demand.clone(), plan, jobs, current: None }
    }

    fn run(&self, index: usize, job: Job, seed: u64, done: Vec<Summary>) -> Run {
        let sim = job.sim(self.kind, &self.demand, seed);
        // The same number of steps as `Sim::run_for`, so results match the desktop app.
        let steps_left = ((self.plan.warmup + self.plan.duration) / sim.dt).round() as usize;
        Run { index, job, seed, sim, steps_left, done }
    }

    /// Step the strategies 50 steps at a time for as long as `more` says to, putting each
    /// one that finishes in its place in `rows`. True once every strategy has finished.
    pub fn advance(&mut self, rows: &mut [Option<BenchRow>], mut more: impl FnMut() -> bool) -> bool {
        while more() {
            if self.current.is_none() {
                let Some((index, job)) = self.jobs.pop_front() else { return true };
                self.current = Some(self.run(index, job, 0, Vec::new()));
            }
            let run = self.current.as_mut().expect("a run in progress");
            let n = run.steps_left.min(50);
            for _ in 0..n {
                run.sim.step();
            }
            run.steps_left -= n;
            if run.steps_left > 0 {
                continue;
            }
            let run = self.current.take().expect("a run in progress");
            let mut done = run.done;
            done.push(run.sim.stats.summary(self.plan.warmup, run.sim.t, &run.sim.vehicles, &run.sim.queues));
            if run.seed + 1 < self.plan.seeds {
                self.current = Some(self.run(run.index, run.job, run.seed + 1, done));
            } else {
                rows[run.index] = Some(BenchRow { label: run.job.label, summary: average(done) });
            }
        }
        self.current.is_none() && self.jobs.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::network::Network;

    /// A browser page runs a benchmark a slice of each frame at a time, the desktop app on
    /// threads. However the slices fall, every row comes out the same to the last bit,
    /// in its own place.
    #[test]
    fn slices_give_the_threaded_results() {
        let plan = Plan { seeds: 2, warmup: 30.0, duration: 90.0 };
        let kind = LayoutKind::Tee;
        let demand = DemandConfig::default_for(&Network::build(kind));
        let jobs = || {
            [
                (ControlMode::Signal, CtrlKind::Actuated),
                (ControlMode::Signal, CtrlKind::Hybrid),
                (ControlMode::AllWayStop, CtrlKind::Fixed),
            ]
            .map(|(mode, ctrl)| Job {
                label: format!("{} {}", mode.name(), ctrl.short()),
                mode,
                ctrl,
                sensors: SensorConfig::default(),
                coop: None,
            })
        };
        let threaded: Vec<String> = jobs()
            .into_iter()
            .map(|j| {
                let r = j.row(kind, &demand, plan);
                format!("{} {:?}", r.label, r.summary)
            })
            .collect();
        let mut sliced = Sliced::new(kind, &demand, jobs().into(), plan);
        let mut rows: Vec<Option<BenchRow>> = (0..3).map(|_| None).collect();
        // Slices of 1 to 7 rounds of 50 steps, as uneven as frames.
        let mut frames = 0;
        loop {
            let mut left = 1 + frames % 7;
            let finished = sliced.advance(&mut rows, || {
                left -= 1;
                left + 1 > 0
            });
            frames += 1;
            if finished {
                break;
            }
            assert!(frames < 10_000, "the benchmark never finished");
        }
        let sliced: Vec<String> = rows
            .into_iter()
            .map(|r| {
                let r = r.expect("every row filled");
                format!("{} {:?}", r.label, r.summary)
            })
            .collect();
        assert_eq!(sliced, threaded);
        assert!(frames > 10, "the runs were spread over many frames");
    }
}
