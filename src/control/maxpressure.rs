//! Max-pressure control (Varaiya 2013): pressure of a stage = Σ over its movements of
//! saturation-weighted (upstream queue − downstream queue). Serve the max-pressure
//! stage, with a minimum dwell and a switching threshold to account for lost time.

use super::{Brain, Controller, Ctx, lanes_for, stat};
use crate::sim::network::{MStat, Network};
use crate::sim::signal::Phase;

pub struct MaxPressure {
    pressures: Vec<f64>,
    names: Vec<String>,
    cur: usize,
    threshold: f64,
}

impl MaxPressure {
    pub fn new(net: &Network) -> Self {
        MaxPressure {
            pressures: vec![0.0; net.stages.len()],
            names: net.stages.iter().map(|s| s.name.clone()).collect(),
            cur: 0,
            threshold: 3.0,
        }
    }
}

pub fn stage_pressures(ctx: &Ctx) -> Vec<f64> {
    let net = ctx.net;
    let mut p = vec![0.0; net.stages.len()];
    for (j, pj) in p.iter_mut().enumerate() {
        for m in 0..net.movements.len() {
            let w = match stat(net, j, m) {
                Some(MStat::G) | Some(MStat::Rt) => 1.0,
                Some(MStat::P) => 0.45,
                None => continue,
            };
            let mo = &ctx.view.moves[m];
            // Queue plus vehicles arriving within ~8 s.
            let up: f64 = mo.queue + mo.vehicles.iter().filter(|o| !o.queued && o.eta < 8.0).map(|o| o.weight * 0.5).sum::<f64>();
            let down = ctx.view.out_queue[net.movements[m].to_leg] / net.legs[net.movements[m].to_leg].n_out as f64;
            *pj += w * (up / lanes_for(net, m).sqrt() - down).max(0.0) * lanes_for(net, m).sqrt();
        }
        for &cw in &net.stages[j].walk {
            *pj += ctx.view.ped_waiting[cw] as f64 * 1.5;
        }
    }
    p
}

impl Controller for MaxPressure {
    fn name(&self) -> &'static str {
        "Max-Pressure"
    }

    fn step(&mut self, ctx: &Ctx) -> Option<usize> {
        if ctx.sig.phase != Phase::Green {
            return None;
        }
        self.cur = ctx.sig.stage;
        self.pressures = stage_pressures(ctx);
        if !ctx.can_switch {
            return None;
        }
        let (best, pb) = self
            .pressures
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(i, p)| (i, *p))
            .unwrap();
        let pc = self.pressures[self.cur];
        (best != self.cur && pb > pc + self.threshold).then_some(best)
    }

    fn brain(&self) -> Brain {
        Brain {
            title: "Stage pressure (queue upstream − downstream)".into(),
            bars: self.names.iter().zip(&self.pressures).enumerate().map(|(i, (n, p))| (n.clone(), *p, i == self.cur)).collect(),
            lower_is_better: false,
            notes: vec![],
        }
    }
}
