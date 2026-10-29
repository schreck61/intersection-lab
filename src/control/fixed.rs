//! Pre-timed control optimised with Webster's method:
//! C0 = (1.5 L + 5) / (1 - Y), effective green split in proportion to critical flow ratios.

use super::{Brain, Controller, Ctx};
use crate::sim::demand::DemandConfig;
use crate::sim::network::{MStat, Network};
use crate::sim::signal::Phase;

pub struct FixedTime {
    pub seq: Vec<usize>,
    pub greens: Vec<f64>,
    pub cycle: f64,
    last: usize,
    names: Vec<String>,
}

pub const SAT_FLOW: f64 = 1800.0;

impl FixedTime {
    pub fn new(net: &Network, demand: &DemandConfig) -> Self {
        let seq = net.fixed_sequence.clone();
        let (cycle, greens) = webster(net, demand, &seq);
        FixedTime { names: seq.iter().map(|&s| net.stages[s].name.clone()).collect(), seq, greens, cycle, last: 0 }
    }
}

/// Returns (cycle, green per sequence position).
pub fn webster(net: &Network, demand: &DemandConfig, seq: &[usize]) -> (f64, Vec<f64>) {
    if seq.is_empty() {
        return (0.0, vec![]);
    }
    let flows: Vec<f64> = (0..net.movements.len()).map(|m| demand.movement_flow(net, m)).collect();
    // Lane flows: each movement's flow split evenly over the lanes that carry it.
    let mut lane_flow = vec![0.0; net.links.len()];
    for (m, mv) in net.movements.iter().enumerate() {
        let n = mv.routes.len().max(1) as f64;
        for (lane, _) in &mv.routes {
            lane_flow[*lane] += flows[m] / n;
        }
    }
    let protected_somewhere =
        |m: usize| seq.iter().any(|&s| matches!(net.stages[s].stat[m], Some(MStat::G) | Some(MStat::Rt)));
    let mut ys = Vec::new();
    for &s in seq {
        let st = &net.stages[s];
        let mut y: f64 = 0.0;
        for leg in &net.legs {
            for &lane in &leg.in_lanes {
                let moves = &net.links[lane].lane_moves;
                // A shared lane only discharges when every movement in it is served.
                let all_served = moves.iter().all(|&m| st.stat[m].is_some() || flows[m] <= 0.0);
                let served_prot = moves.iter().any(|&m| matches!(st.stat[m], Some(MStat::G) | Some(MStat::Rt)));
                let only_perm = moves.iter().all(|&m| st.stat[m] == Some(MStat::P) && !protected_somewhere(m));
                let has_perm = moves.iter().any(|&m| st.stat[m] == Some(MStat::P) && flows[m] > 0.0);
                if all_served && (served_prot || only_perm) {
                    let sat = if has_perm { SAT_FLOW * 0.7 } else { SAT_FLOW };
                    y = y.max(lane_flow[lane] / sat);
                }
            }
        }
        ys.push(y);
    }
    let lost_per = net.yellow + net.all_red;
    let l = lost_per * seq.len() as f64;
    let ysum: f64 = ys.iter().sum::<f64>().min(0.92);
    let cycle = ((1.5 * l + 5.0) / (1.0 - ysum)).clamp(40.0, 150.0);
    let g_total = cycle - l;
    let yeff: Vec<f64> = ys.iter().map(|y| y.max(0.02)).collect();
    let ytot: f64 = yeff.iter().sum();
    let greens = yeff.iter().map(|y| (g_total * y / ytot).max(7.0)).collect();
    (cycle, greens)
}

impl Controller for FixedTime {
    fn name(&self) -> &'static str {
        "Fixed-time"
    }

    fn step(&mut self, ctx: &Ctx) -> Option<usize> {
        if ctx.sig.phase != Phase::Green || self.seq.is_empty() {
            return None;
        }
        let cur = ctx.sig.stage;
        match self.seq.iter().position(|&s| s == cur) {
            Some(i) => {
                self.last = i;
                if ctx.can_switch && ctx.sig.green_elapsed >= self.greens[i] {
                    return Some(self.seq[(i + 1) % self.seq.len()]);
                }
                None
            }
            // After a preemption, resume the cycle where it left off.
            None => ctx.can_switch.then(|| self.seq[(self.last + 1) % self.seq.len()]),
        }
    }

    fn brain(&self) -> Brain {
        Brain {
            title: format!("Webster plan: cycle {:.0} s", self.cycle),
            bars: self
                .names
                .iter()
                .zip(&self.greens)
                .enumerate()
                .map(|(i, (n, g))| (format!("{n} (s)"), *g, i == self.last))
                .collect(),
            lower_is_better: false,
            notes: vec!["Green times from historical demand; does not react to sensors.".into()],
        }
    }
}
