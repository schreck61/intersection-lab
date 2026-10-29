// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Fully actuated control (NEMA-style, single-ring sequence with phase skipping):
//! min green, unit extension per actuation (passage time), gap-out / max-out,
//! calls from stop-bar presence and pedestrian push buttons, and a conditional
//! transit-priority green extension when a camera or connected bus is approaching.
//! Detector stations flagged faulty by the diagnostics are placed on max recall.

use super::{Brain, Controller, Ctx, stat};
use crate::sim::network::{MStat, Network};
use crate::sim::signal::Phase;
use crate::sim::vtypes::VKind;

pub struct Actuated {
    seq: Vec<usize>,
    passage: f64,
    max_green: Vec<f64>,
    gap: f64,
    status: String,
    calls: Vec<bool>,
}

impl Actuated {
    pub fn new(net: &Network) -> Self {
        // The actuated sequence serves every stage in the fixed sequence; extra
        // (split) stages are left out like a standard dual-ring lead-lag plan.
        let seq = net.fixed_sequence.clone();
        let max_green = net
            .stages
            .iter()
            .map(|s| {
                let protected_left = s.stat.iter().enumerate().any(|(m, x)| {
                    *x == Some(MStat::G) && net.movements[m].turn == crate::sim::network::Turn::L
                });
                let through = s.stat.iter().enumerate().any(|(m, x)| {
                    *x == Some(MStat::G) && net.movements[m].turn == crate::sim::network::Turn::T
                });
                if protected_left && !through { 22.0 } else { 50.0 }
            })
            .collect();
        Actuated { seq, passage: 2.5, max_green, gap: 0.0, status: String::new(), calls: vec![] }
    }

    fn has_call(ctx: &Ctx, j: usize) -> bool {
        if !ctx.sensors.loops && !ctx.sensors.camera {
            return true; // detector failure => recall
        }
        let st = &ctx.net.stages[j];
        // Lanes flagged faulty by detector diagnostics are on recall.
        let recall = ctx.net.legs.iter().flat_map(|l| &l.in_lanes).any(|&lane| {
            ctx.view.lanes[lane].fault && ctx.net.links[lane].lane_moves.iter().any(|&m| st.stat[m].is_some())
        });
        recall
            || (0..ctx.net.movements.len()).any(|m| {
                st.stat[m].is_some() && (ctx.view.moves[m].stopbar || ctx.view.moves[m].queue >= 0.5)
            })
            || st.walk.iter().any(|&cw| ctx.view.ped_call[cw])
    }
}

impl Controller for Actuated {
    fn name(&self) -> &'static str {
        "Actuated"
    }

    fn step(&mut self, ctx: &Ctx) -> Option<usize> {
        if ctx.sig.phase != Phase::Green || self.seq.is_empty() {
            return None;
        }
        let cur = ctx.sig.stage;
        self.calls = (0..ctx.net.stages.len()).map(|j| Self::has_call(ctx, j)).collect();
        // Extend only on lanes the current stage fully serves: a shared through/right
        // lane queued at red must not hold a protected-left stage.
        let mut last_act = f64::NEG_INFINITY;
        for leg in &ctx.net.legs {
            for &lane in &leg.in_lanes {
                let moves = &ctx.net.links[lane].lane_moves;
                let all = moves.iter().all(|&m| matches!(stat(ctx.net, cur, m), Some(MStat::G) | Some(MStat::Rt)));
                if all {
                    // A faulty station is on max recall: it always extends.
                    let lp = if ctx.view.lanes[lane].fault { ctx.t } else { ctx.view.lanes[lane].last_pulse };
                    last_act = last_act.max(lp);
                }
            }
        }
        self.gap = if ctx.sensors.loops { ctx.t - last_act } else { 0.0 };
        if !ctx.can_switch {
            self.status = "minimum green".into();
            return None;
        }
        let i = self.seq.iter().position(|&s| s == cur).unwrap_or(0);
        let next = (1..=self.seq.len()).map(|k| self.seq[(i + k) % self.seq.len()]).find(|&j| j != cur && self.calls[j]);
        let Some(next) = next else {
            self.status = "resting (no conflicting calls)".into();
            return None;
        };
        let maxg = self.max_green[cur];
        // Transit priority: hold green for an approaching bus.
        let bus_coming = (0..ctx.net.movements.len()).any(|m| {
            matches!(stat(ctx.net, cur, m), Some(MStat::G) | Some(MStat::Rt))
                && ctx.view.moves[m].vehicles.iter().any(|o| o.kind == Some(VKind::Bus) && o.eta > 0.0 && o.eta < 8.0)
        });
        if bus_coming && ctx.sig.green_elapsed < maxg + 10.0 {
            self.status = "transit priority extension".into();
            return None;
        }
        if ctx.sig.green_elapsed >= maxg {
            self.status = "max-out".into();
            return Some(next);
        }
        if self.gap > self.passage {
            self.status = "gap-out".into();
            return Some(next);
        }
        self.status = "extending".into();
        None
    }

    fn brain(&self) -> Brain {
        Brain {
            title: format!("Actuated: {}", self.status),
            bars: vec![
                ("gap since last actuation (s)".into(), self.gap.min(10.0), self.gap > self.passage),
                ("passage time (s)".into(), self.passage, false),
            ],
            lower_is_better: false,
            notes: vec![format!(
                "stages with calls: {}",
                self.calls.iter().enumerate().filter(|c| *c.1).map(|c| c.0.to_string()).collect::<Vec<_>>().join(", ")
            )],
        }
    }
}
