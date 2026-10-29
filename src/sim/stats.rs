//! Vehicle-experience statistics: HCM control delay, stops, stopped time, fuel/CO2,
//! person delay, level of service, per-approach/movement/class breakdowns and
//! time series for charts.

use super::network::{Network, Turn};
use super::demand::Pending;
use super::vtypes::{ALL_KINDS, VKind};
use super::{Vehicle};
use std::collections::VecDeque;

#[derive(Clone, Debug)]
pub struct Done {
    pub kind: VKind,
    pub leg: usize,
    pub turn: Turn,
    pub movement: usize,
    pub delay: f64,
    pub stops: u32,
    pub travel: f64,
    pub stopped: f64,
    pub fuel_ml: f64,
    pub co2_g: f64,
    pub persons: f64,
    pub t_exit: f64,
}

#[derive(Clone, Debug, Default)]
pub struct Sample {
    pub t: f64,
    pub queue: f64,
    pub in_net: f64,
    pub throughput: f64,
    pub avg_delay: f64,
    pub waiting_entry: f64,
}

#[derive(Clone, Debug, Default)]
pub struct Group {
    pub n: usize,
    pub delay: f64,
    pub stops: f64,
}

impl Group {
    pub fn avg_delay(&self) -> f64 {
        if self.n == 0 { 0.0 } else { self.delay / self.n as f64 }
    }
    pub fn avg_stops(&self) -> f64 {
        if self.n == 0 { 0.0 } else { self.stops / self.n as f64 }
    }
}

#[derive(Clone, Debug, Default)]
pub struct Summary {
    pub n: usize,
    pub duration: f64,
    pub throughput_vph: f64,
    pub avg_delay: f64,
    pub p50_delay: f64,
    pub p95_delay: f64,
    pub max_delay: f64,
    pub avg_stops: f64,
    pub avg_stopped: f64,
    pub person_delay: f64,
    pub avg_travel: f64,
    pub co2_per_veh: f64,
    pub fuel_l_total: f64,
    pub ped_n: usize,
    pub ped_avg_wait: f64,
    pub ped_max_wait: f64,
    pub by_leg: Vec<Group>,
    pub by_move: Vec<Group>,
    pub by_kind: Vec<Group>,
    pub unserved: usize,
    pub unserved_wait: f64,
    /// Person delay over every vehicle that arrived, including those still in the
    /// network or queued upstream (robust to policies that starve an approach).
    pub person_delay_all: f64,
    pub avg_delay_all: f64,
    /// Vehicles passing within ~1.5 s of a walking pedestrian, per hour.
    pub ped_near_miss_ph: f64,
}

pub struct Stats {
    pub done: Vec<Done>,
    pub ped_waits: Vec<(f64, f64)>,
    /// Times of vehicle/pedestrian near-misses (see `Sim::drive`).
    pub ped_near_miss: Vec<f64>,
    pub series: VecDeque<Sample>,
    pub generated: u64,
    pub hist: Vec<u32>,
    next_sample: f64,
    n_legs: usize,
    n_moves: usize,
}

pub const HIST_BIN: f64 = 10.0;
pub const HIST_BINS: usize = 13;

impl Stats {
    pub fn new(net: &Network) -> Self {
        Stats {
            done: Vec::new(),
            ped_waits: Vec::new(),
            ped_near_miss: Vec::new(),
            series: VecDeque::new(),
            generated: 0,
            hist: vec![0; HIST_BINS],
            next_sample: 0.0,
            n_legs: net.legs.len(),
            n_moves: net.movements.len(),
        }
    }

    pub fn record(&mut self, d: Done) {
        let b = ((d.delay / HIST_BIN) as usize).min(HIST_BINS - 1);
        self.hist[b] += 1;
        self.done.push(d);
    }

    pub fn sample(&mut self, t: f64, vehicles: &[Vehicle], net: &Network, queues: &[VecDeque<Pending>]) {
        if t < self.next_sample {
            return;
        }
        self.next_sample = t + 2.0;
        let queue = vehicles
            .iter()
            .filter(|v| v.v < 2.0 && net.links[v.link()].kind == super::network::LinkKind::Inbound)
            .count() as f64;
        let waiting: usize = queues.iter().map(|q| q.len()).sum();
        let window = 300.0_f64.min(t.max(1.0));
        let recent: Vec<&Done> = self.done.iter().rev().take_while(|d| d.t_exit > t - window).collect();
        let throughput = recent.len() as f64 * 3600.0 / window;
        let last60: Vec<&Done> = recent.iter().copied().take_while(|d| d.t_exit > t - 60.0).collect();
        let avg_delay = if last60.is_empty() {
            self.series.back().map(|s| s.avg_delay).unwrap_or(0.0)
        } else {
            last60.iter().map(|d| d.delay).sum::<f64>() / last60.len() as f64
        };
        self.series.push_back(Sample {
            t,
            queue,
            in_net: vehicles.len() as f64,
            throughput,
            avg_delay,
            waiting_entry: waiting as f64,
        });
        while self.series.len() > 900 {
            self.series.pop_front();
        }
    }

    /// Summary of vehicles that exited after `from_t`. `live` lets unfinished vehicles
    /// (still queued) count toward the unserved metrics.
    pub fn summary(&self, from_t: f64, now: f64, live: &[Vehicle], queues: &[VecDeque<Pending>]) -> Summary {
        let ds: Vec<&Done> = self.done.iter().filter(|d| d.t_exit >= from_t).collect();
        let mut s = Summary {
            by_leg: vec![Group::default(); self.n_legs],
            by_move: vec![Group::default(); self.n_moves],
            by_kind: vec![Group::default(); ALL_KINDS.len()],
            duration: (now - from_t).max(1e-6),
            ..Default::default()
        };
        s.n = ds.len();
        if s.n > 0 {
            let n = s.n as f64;
            let mut delays: Vec<f64> = ds.iter().map(|d| d.delay).collect();
            delays.sort_by(|a, b| a.partial_cmp(b).unwrap());
            s.avg_delay = delays.iter().sum::<f64>() / n;
            s.p50_delay = delays[(0.5 * (n - 1.0)) as usize];
            s.p95_delay = delays[(0.95 * (n - 1.0)) as usize];
            s.max_delay = *delays.last().unwrap();
            s.avg_stops = ds.iter().map(|d| d.stops as f64).sum::<f64>() / n;
            s.avg_stopped = ds.iter().map(|d| d.stopped).sum::<f64>() / n;
            s.avg_travel = ds.iter().map(|d| d.travel).sum::<f64>() / n;
            let persons: f64 = ds.iter().map(|d| d.persons).sum();
            s.person_delay = ds.iter().map(|d| d.delay * d.persons).sum::<f64>() / persons.max(1e-9);
            s.co2_per_veh = ds.iter().map(|d| d.co2_g).sum::<f64>() / n;
            s.fuel_l_total = ds.iter().map(|d| d.fuel_ml).sum::<f64>() / 1000.0;
            for d in &ds {
                for g in [&mut s.by_leg[d.leg], &mut s.by_move[d.movement], &mut s.by_kind[d.kind.index()]] {
                    g.n += 1;
                    g.delay += d.delay;
                    g.stops += d.stops as f64;
                }
            }
        }
        s.throughput_vph = s.n as f64 * 3600.0 / s.duration;
        let pw: Vec<f64> = self.ped_waits.iter().filter(|p| p.0 >= from_t).map(|p| p.1).collect();
        s.ped_n = pw.len();
        if !pw.is_empty() {
            s.ped_avg_wait = pw.iter().sum::<f64>() / pw.len() as f64;
            s.ped_max_wait = pw.iter().cloned().fold(0.0, f64::max);
        }
        s.ped_near_miss_ph = self.ped_near_miss.iter().filter(|&&t| t >= from_t).count() as f64 * 3600.0 / s.duration;
        // Vehicles still waiting: the upstream virtual queue and long-delayed vehicles.
        let mut unserved = 0;
        let mut uw = 0.0;
        for q in queues {
            for p in q {
                unserved += 1;
                uw += now - p.arrival_t;
            }
        }
        for v in live {
            if v.arrival_t >= from_t || v.delay > 0.0 {
                if v.delay > 30.0 {
                    unserved += 1;
                    uw += v.delay;
                }
            }
        }
        s.unserved = unserved;
        s.unserved_wait = if unserved > 0 { uw / unserved as f64 } else { 0.0 };
        let (mut pd, mut pp, mut vd, mut vn) = (0.0, 0.0, 0.0, 0.0);
        for d in &ds {
            pd += d.delay * d.persons;
            pp += d.persons;
            vd += d.delay;
            vn += 1.0;
        }
        for v in live {
            let p = v.persons();
            pd += v.delay * p;
            pp += p;
            vd += v.delay;
            vn += 1.0;
        }
        for q in queues {
            for p in q {
                let w = now - p.arrival_t;
                let persons = p.kind.spec().occupancy;
                pd += w * persons;
                pp += persons;
                vd += w;
                vn += 1.0;
            }
        }
        s.person_delay_all = if pp > 0.0 { pd / pp } else { 0.0 };
        s.avg_delay_all = if vn > 0.0 { vd / vn } else { 0.0 };
        s
    }
}

/// HCM level of service from average control delay.
pub fn los(delay: f64, signalized: bool) -> &'static str {
    let t: [f64; 5] = if signalized { [10.0, 20.0, 35.0, 55.0, 80.0] } else { [10.0, 15.0, 25.0, 35.0, 50.0] };
    let l = ["A", "B", "C", "D", "E"];
    for i in 0..5 {
        if delay <= t[i] {
            return l[i];
        }
    }
    "F"
}
