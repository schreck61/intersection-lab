// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Benchmark runs behind *Compare controllers* and *Sensor study*: every strategy on
//! the same arrival streams, averaged over a few seeds. The desktop app runs the
//! strategies in parallel on threads; a browser page has a single thread, so there they
//! run one after another, a slice of each frame at a time.

use crate::control::CtrlKind;
use crate::sim::ControlMode;
use crate::sim::demand::DemandConfig;
use crate::sim::network::LayoutKind;
use crate::sim::sensors::SensorConfig;
use crate::sim::stats::Summary;

/// Arrival streams (seeds) per strategy, and the length of each run.
const SEEDS: u64 = 3;
const WARMUP: f64 = 300.0;
const DURATION: f64 = 1200.0;

/// One strategy to benchmark.
pub struct Job {
    pub label: String,
    pub mode: ControlMode,
    pub ctrl: CtrlKind,
    pub sensors: SensorConfig,
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
    queue: web::Queue,
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
                let runs = (0..SEEDS)
                    .map(|seed| crate::run_scenario(crate::scenario(kind, job.mode, job.ctrl, &demand, &job.sensors, 500 + seed), WARMUP, DURATION).0)
                    .collect();
                tx.send((i, BenchRow { label: job.label, summary: average(runs) })).ok();
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
mod web {
    use super::*;
    use crate::sim::Sim;

    pub struct Queue {
        pub kind: LayoutKind,
        pub demand: DemandConfig,
        pub jobs: std::collections::VecDeque<(usize, Job)>,
        pub current: Option<Run>,
    }

    /// One seed of one strategy, part way through.
    pub struct Run {
        pub index: usize,
        pub job: Job,
        pub seed: u64,
        pub sim: Sim,
        pub steps_left: usize,
        /// Summaries of the strategy's earlier seeds.
        pub done: Vec<Summary>,
    }

    impl Run {
        pub fn new(q: &Queue, index: usize, job: Job, seed: u64, done: Vec<Summary>) -> Self {
            let sim = crate::scenario(q.kind, job.mode, job.ctrl, &q.demand, &job.sensors, 500 + seed);
            // The same number of steps as `Sim::run_for`, so results match the desktop app.
            let steps_left = ((WARMUP + DURATION) / sim.dt).round() as usize;
            Run { index, job, seed, sim, steps_left, done }
        }
    }
}

#[cfg(target_arch = "wasm32")]
impl Bench {
    /// Milliseconds of each frame given to the benchmark.
    const SLICE_MS: u128 = 12;

    pub fn start(title: String, kind: LayoutKind, demand: &DemandConfig, jobs: Vec<Job>) -> Self {
        let total = jobs.len();
        let queue = web::Queue { kind, demand: demand.clone(), jobs: jobs.into_iter().enumerate().collect(), current: None };
        Bench { title, total, rows: (0..total).map(|_| None).collect(), queue }
    }

    /// Advance the benchmark for a slice of this frame.
    pub fn poll(&mut self, ctx: &egui::Context) {
        let t0 = web_time::Instant::now();
        while t0.elapsed().as_millis() < Self::SLICE_MS {
            if self.queue.current.is_none() {
                let Some((index, job)) = self.queue.jobs.pop_front() else { return };
                self.queue.current = Some(web::Run::new(&self.queue, index, job, 0, Vec::new()));
            }
            let run = self.queue.current.as_mut().expect("a run in progress");
            let n = run.steps_left.min(50);
            for _ in 0..n {
                run.sim.step();
            }
            run.steps_left -= n;
            if run.steps_left > 0 {
                continue;
            }
            let run = self.queue.current.take().expect("a run in progress");
            let mut done = run.done;
            done.push(run.sim.stats.summary(WARMUP, run.sim.t, &run.sim.vehicles, &run.sim.queues));
            if run.seed + 1 < SEEDS {
                self.queue.current = Some(web::Run::new(&self.queue, run.index, run.job, run.seed + 1, done));
            } else {
                self.rows[run.index] = Some(BenchRow { label: run.job.label, summary: average(done) });
            }
        }
        ctx.request_repaint();
    }
}
