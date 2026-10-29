//! Schedule-driven rolling-horizon control (inspired by SURTRAC, Xie & Smith).
//!
//! Every second the controller takes each detected vehicle's predicted arrival at the
//! stop line, enumerates candidate schedules ("hold the current stage x seconds,
//! then switch to stage j, then serve stages greedily") over a 90 s horizon, and
//! simulates queue discharge (saturation headway, start-up lost time, clearance
//! intervals) for each. The schedule with the least person-weighted delay wins;
//! only its first decision is executed and the plan is recomputed next second.

use super::{Brain, Controller, Ctx, lanes_for, needs_clearance, stat};
use crate::sim::network::{MStat, Network};
use crate::sim::signal::Phase;
use crate::sim::vtypes::VKind;

pub struct Predictive {
    horizon: f64,
    next_plan: f64,
    max_green: f64,
    names: Vec<String>,
    bars: Vec<(String, f64, bool)>,
    note: String,
}

#[derive(Clone, Copy)]
struct Block {
    stage: usize,
    start: f64,
    end: f64,
}

/// (eta, cost weight, wait so far)
type Arr = (f64, f64, f64);

impl Predictive {
    pub fn new(net: &Network) -> Self {
        Predictive {
            horizon: 90.0,
            next_plan: 0.0,
            max_green: 60.0,
            names: net.stages.iter().map(|s| s.name.clone()).collect(),
            bars: vec![],
            note: String::new(),
        }
    }

    fn headway(st: MStat) -> f64 {
        match st {
            MStat::G => 2.0,
            MStat::Rt => 2.4,
            MStat::P => 4.2,
        }
    }

    fn cost(ctx: &Ctx, blocks: &[Block], arr: &[Vec<Arr>], horizon: f64) -> f64 {
        let net = ctx.net;
        let mut total = 0.0;
        for m in 0..net.movements.len() {
            if arr[m].is_empty() {
                continue;
            }
            let lanes = lanes_for(net, m);
            let mut wins: Vec<(f64, f64, f64)> = Vec::with_capacity(blocks.len());
            for (bi, b) in blocks.iter().enumerate() {
                let Some(st) = stat(net, b.stage, m) else { continue };
                let continuing = if bi == 0 {
                    ctx.sig.green_elapsed > 2.0
                } else {
                    stat(net, blocks[bi - 1].stage, m).is_some() && (blocks[bi - 1].end - b.start).abs() < 1e-6
                };
                let start = b.start + if continuing { 0.0 } else { 2.0 };
                let leaving = blocks.get(bi + 1).is_some_and(|n| stat(net, n.stage, m).is_none());
                let end = b.end + if leaving { 1.5 } else { 0.0 };
                if end > start {
                    wins.push((start, end, Self::headway(st) / lanes));
                }
            }
            let mut wi = 0;
            let mut tnext = 0.0;
            for &(eta, w, wait0) in &arr[m] {
                let mut t = eta.max(tnext);
                let mut served = false;
                while wi < wins.len() {
                    let (s, e, _) = wins[wi];
                    if t < s {
                        t = s;
                    }
                    if t <= e {
                        served = true;
                        break;
                    }
                    wi += 1;
                }
                let dly = if served {
                    tnext = t + wins[wi].2;
                    t - eta
                } else {
                    (horizon - eta).max(0.0) + 25.0
                };
                total += w * (dly + 0.004 * ((wait0 + dly).powi(2) - wait0 * wait0));
            }
        }
        total
    }

    #[allow(clippy::too_many_arguments)]
    fn rollout(
        ctx: &Ctx,
        first: Block,
        j: usize,
        counts: &[Vec<f64>],
        h_eff: &[Vec<f64>],
        horizon: f64,
        min_g: f64,
        blocks: &mut Vec<Block>,
    ) {
        let net = ctx.net;
        let clear = net.yellow + net.all_red;
        let nm = net.movements.len();
        let mut served = vec![0.0; nm];
        let serve = |k: usize, t0: f64, served: &mut Vec<f64>| -> f64 {
            let mut d = min_g;
            for _ in 0..4 {
                let mut need: f64 = 0.0;
                for m in 0..nm {
                    if stat(net, k, m).is_none() {
                        continue;
                    }
                    let c = count_before(&counts[m], t0 + d) - served[m];
                    need = need.max(2.0 + c.max(0.0) * h_eff[k][m]);
                }
                d = need.clamp(min_g, 45.0);
            }
            for m in 0..nm {
                if stat(net, k, m).is_some() {
                    let c = count_before(&counts[m], t0 + d) - served[m];
                    served[m] += c.max(0.0).min((d - 2.0).max(0.0) / h_eff[k][m]);
                }
            }
            d
        };
        blocks.clear();
        blocks.push(first);
        // Credit the first block's service.
        for m in 0..nm {
            if stat(net, first.stage, m).is_some() {
                let c = count_before(&counts[m], first.end);
                served[m] += c.min((first.end - first.start + 2.0) / h_eff[first.stage][m]);
            }
        }
        let mut t = first.end + if needs_clearance(net, first.stage, j) { clear } else { 0.0 };
        let d = serve(j, t, &mut served);
        blocks.push(Block { stage: j, start: t, end: t + d });
        t += d;
        let mut prev = j;
        while t < horizon {
            let mut best = None;
            let mut bv = 0.5;
            for k in 0..net.stages.len() {
                if k == prev {
                    continue;
                }
                let mut v = 0.0;
                for m in 0..nm {
                    if stat(net, k, m).is_some() {
                        v += (count_before(&counts[m], t) - served[m]).max(0.0) / h_eff[k][m];
                    }
                }
                if v > bv {
                    bv = v;
                    best = Some(k);
                }
            }
            let Some(k) = best else { break };
            t += if needs_clearance(net, prev, k) { clear } else { 0.0 };
            let d = serve(k, t, &mut served);
            blocks.push(Block { stage: k, start: t, end: t + d });
            t += d;
            prev = k;
        }
    }
}

fn count_before(etas: &[f64], t: f64) -> f64 {
    etas.partition_point(|&e| e < t) as f64
}

impl Controller for Predictive {
    fn name(&self) -> &'static str {
        "Predictive"
    }

    fn step(&mut self, ctx: &Ctx) -> Option<usize> {
        if ctx.sig.phase != Phase::Green || !ctx.can_switch || ctx.t < self.next_plan {
            return None;
        }
        self.next_plan = ctx.t + 1.0;
        let net = ctx.net;
        let nm = net.movements.len();
        let ns = net.stages.len();
        let cur = ctx.sig.stage;
        let e = ctx.sig.green_elapsed;
        // Predicted arrivals per movement.
        let mut arr: Vec<Vec<Arr>> = vec![vec![]; nm];
        let mut counts: Vec<Vec<f64>> = vec![vec![]; nm];
        for m in 0..nm {
            for o in &ctx.view.moves[m].vehicles {
                let ev = if o.kind == Some(VKind::Emergency) { 40.0 } else { 1.0 };
                arr[m].push((o.eta, o.weight * o.persons * ev, o.wait));
            }
            arr[m].sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap().then(b.2.partial_cmp(&a.2).unwrap()));
            counts[m] = arr[m].iter().map(|a| a.0).collect();
        }
        // Waiting pedestrians: model as queued "vehicles" on a stage's first movement.
        for st in 0..ns {
            for &cw in &net.stages[st].walk {
                let n = ctx.view.ped_waiting[cw];
                if n == 0 {
                    continue;
                }
                if let Some(m) = (0..nm).find(|&m| stat(net, st, m) == Some(MStat::G)) {
                    for _ in 0..n {
                        arr[m].insert(0, (0.0, 1.0, ctx.view.ped_max_wait[cw]));
                    }
                }
            }
        }
        let h_eff: Vec<Vec<f64>> = (0..ns)
            .map(|k| (0..nm).map(|m| stat(net, k, m).map(|s| Self::headway(s) / lanes_for(net, m)).unwrap_or(2.0)).collect())
            .collect();
        let min_g = ctx.sig.min_green;
        let xs = [0.0, 2.0, 4.0, 7.0, 10.0, 15.0, 20.0, 30.0];
        let mut blocks = Vec::new();
        let mut best = (f64::INFINITY, 0.0, cur);
        let mut switch_cost = vec![f64::INFINITY; ns];
        let mut hold_cost = f64::INFINITY;
        for &x in &xs {
            if x > 0.0 && e + x > self.max_green {
                continue;
            }
            for j in 0..ns {
                if j == cur {
                    continue;
                }
                let first = Block { stage: cur, start: 0.0, end: x };
                Self::rollout(ctx, first, j, &counts, &h_eff, self.horizon, min_g, &mut blocks);
                let c = Self::cost(ctx, &blocks, &arr, self.horizon);
                if x == 0.0 {
                    switch_cost[j] = switch_cost[j].min(c);
                } else {
                    hold_cost = hold_cost.min(c);
                }
                if c < best.0 - 1e-6 {
                    best = (c, x, j);
                }
            }
        }
        self.bars = vec![(format!("Hold {}", self.names[cur]), hold_cost, best.1 > 0.0)];
        for j in 0..ns {
            if j != cur {
                self.bars.push((format!("→ {}", self.names[j]), switch_cost[j], best.1 == 0.0 && best.2 == j));
            }
        }
        self.note = if best.1 > 0.0 {
            format!("plan: hold ~{:.0} s more, then {}", best.1, self.names[best.2])
        } else {
            format!("plan: switch now to {}", self.names[best.2])
        };
        (best.1 == 0.0).then_some(best.2)
    }

    fn brain(&self) -> Brain {
        Brain {
            title: "Predicted person-delay over 90 s horizon".into(),
            bars: self.bars.clone(),
            lower_is_better: true,
            notes: vec![self.note.clone()],
        }
    }
}
