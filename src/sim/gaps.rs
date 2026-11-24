// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Gap acceptance, measured the way a field study measures it, for the drivers who must
//! yield: at a two-way stop the left turns from the major road and everyone on the minor
//! street, and at a signal the left turns on a permissive indication.
//!
//! Each such driver is followed from the moment it is ready to go: a left turn from the
//! major road when it crosses the stop line, or stops first in line at it, and a vehicle
//! on the minor street when it has stopped at its line. The traffic that ranks above it
//! passes the points where its path crosses or joins theirs, one vehicle after another.
//! The time from its readiness to the first of them is the *lag*, and the time between two
//! of them a *gap*: a headway, front to front. It goes when it crosses its line, from the
//! minor street, or else when it enters its first conflict zone. The lags and gaps that
//! ended before then were rejected. The one it went in, from the last vehicle that passed
//! ahead of it to the first that passed behind it, was accepted. [`critical_headway`] fits
//! the critical headways that best explain those choices.

use super::network::{LinkId, Turn};
use super::signal::Sig;
use super::vtypes::VKind;
use super::{ControlMode, Sim, Vehicle};
use std::collections::{HashMap, HashSet};

/// One driver's choices.
#[derive(Clone, Debug)]
pub struct Choice {
    pub movement: usize,
    pub kind: VKind,
    /// The lags and gaps it let pass (s), in order: the lag first, if it let it pass.
    pub rejected: Vec<f64>,
    /// The lag or gap it went in (s).
    pub accepted: f64,
    /// Whether the one it went in was the lag.
    pub lag: bool,
    /// Its follow-up headway: when it waited right behind the vehicle ahead of it in its
    /// lane and went in the same gap, the time between the two going (s).
    pub follow_up: Option<f64>,
}

impl Choice {
    /// A truck or a bus, which the Highway Capacity Manual counts as a heavy vehicle.
    pub fn heavy(&self) -> bool {
        matches!(self.kind, VKind::Truck | VKind::Bus)
    }
}

/// A driver being followed.
struct Watch {
    movement: usize,
    kind: VKind,
    /// Its rank while it yields: traffic of a higher rank is the traffic it yields to.
    rank: f64,
    /// Its path through the box.
    link: LinkId,
    ready: f64,
    go: Option<f64>,
    /// When its front passed each conflict point on its path.
    passed: HashMap<usize, f64>,
    /// When the vehicle it waited right behind went.
    after: Option<f64>,
}

/// Follows the drivers who must yield through a simulation: call [`GapStudy::observe`]
/// after every step, from the first.
#[derive(Default)]
pub struct GapStudy {
    /// For each path through the box, the vehicles that passed its conflict points: when,
    /// their rank then, and which point.
    passes: HashMap<LinkId, Vec<(f64, f64, usize)>>,
    watch: HashMap<u64, Watch>,
    /// Drivers followed already.
    seen: HashSet<u64>,
    /// Drivers not yet ready that waited right behind one that went, and when it went.
    queued: HashMap<u64, f64>,
    pub choices: Vec<Choice>,
}

impl GapStudy {
    pub fn new() -> Self {
        Self::default()
    }

    /// The rank of vehicle `v` while it must yield, if it must.
    fn yielding_rank(sim: &Sim, v: &Vehicle) -> Option<f64> {
        if v.kind == VKind::Emergency || v.route.len() < 2 {
            return None;
        }
        match sim.mode {
            ControlMode::TwoWayStop if sim.net.legs[v.leg].minor => Some(0.5),
            ControlMode::TwoWayStop => (v.turn == Turn::L).then_some(1.0),
            ControlMode::Signal if sim.signalized() => {
                (v.turn == Turn::L && sim.signal.status(&sim.net, v.movement) == Sig::P).then_some(1.0)
            }
            _ => None,
        }
    }

    pub fn observe(&mut self, sim: &Sim) {
        let (t, dt) = (sim.t, sim.dt);
        // Conflict points passed this step, by the front of each vehicle.
        for v in &sim.vehicles {
            if v.s <= v.prev_s {
                continue;
            }
            let rank = sim.vstat_of(v).rank;
            let mut look = |link: LinkId, from: f64, to: f64| {
                for &cid in &sim.net.links[link].conflicts {
                    let (s_c, other, _) = sim.net.conflicts[cid].side(link);
                    if from < s_c && s_c <= to {
                        let when = t - dt * (to - s_c) / (to - from);
                        self.passes.entry(other).or_default().push((when, rank, cid));
                        if let Some(w) = self.watch.get_mut(&v.id)
                            && w.link == link
                        {
                            w.passed.insert(cid, when);
                        }
                    }
                }
            };
            look(v.link(), v.prev_s, v.s);
            if v.prev_s < 0.0 && v.ri > 0 {
                let p = v.route[v.ri - 1];
                let l = sim.net.links[p].length();
                look(p, v.prev_s + l, v.s + l);
            }
        }
        // When a vehicle's front crossed mark `s` on the link it is on now, if it did so
        // this step.
        let crossed =
            |v: &Vehicle, s: f64| (v.prev_s < s && s <= v.s).then(|| t - dt * (v.s - s) / (v.s - v.prev_s));
        let mut went = Vec::new();
        for (i, v) in sim.vehicles.iter().enumerate() {
            let rank = Self::yielding_rank(sim, v);
            if let Some(w) = self.watch.get_mut(&v.id) {
                if w.go.is_some() {
                    continue;
                }
                let Some(_) = rank else {
                    // Its permissive indication ended before it went: it clears on the
                    // change, not in a gap.
                    self.watch.remove(&v.id);
                    continue;
                };
                let go = if w.rank < 1.0 {
                    if v.ri == 1 { crossed(v, 0.0) } else { None }
                } else if v.link() == w.link {
                    let links = &sim.net.links[w.link];
                    let first = (links.conflicts.iter())
                        .map(|&cid| {
                            let c = &sim.net.conflicts[cid];
                            c.side(w.link).0 - c.zones(w.link).0.0
                        })
                        .fold(f64::INFINITY, f64::min);
                    crossed(v, first)
                } else {
                    None
                };
                if go.is_some() {
                    w.go = go;
                    went.push(i);
                }
                continue;
            }
            let Some(rank) = rank else { continue };
            if self.seen.contains(&v.id) {
                continue;
            }
            let ready = if rank < 1.0 {
                v.stop_done.then_some(v.stop_t)
            } else if v.ri >= 1 {
                Some(crossed(v, 0.0).unwrap_or(t))
            } else {
                let d_line = sim.net.links[v.link()].length() - v.s;
                let first = sim.leader(i).is_none_or(|(gap, _, _)| gap > 8.0);
                (d_line < 5.0 && v.v < 0.3 && first).then_some(t)
            };
            if let Some(ready) = ready {
                self.seen.insert(v.id);
                let after = self.queued.remove(&v.id);
                let w = Watch {
                    movement: v.movement,
                    kind: v.kind,
                    rank,
                    link: v.route[1],
                    ready,
                    go: None,
                    passed: HashMap::new(),
                    after,
                };
                self.watch.insert(v.id, w);
            }
        }
        // Whoever waited right behind each driver that went, in the same lane, may go in
        // the same gap.
        let pos = |x: &Vehicle| x.route[..x.ri].iter().map(|&l| sim.net.links[l].length()).sum::<f64>() + x.s;
        for i in went {
            let v = &sim.vehicles[i];
            let Some(go) = self.watch[&v.id].go else { continue };
            let behind = (sim.vehicles.iter())
                .filter(|x| x.id != v.id && x.route.get(..2) == v.route.get(..2) && pos(x) < pos(v))
                .max_by(|a, b| pos(a).total_cmp(&pos(b)));
            if let Some(x) = behind
                && pos(v) - v.len() - pos(x) < 10.0
                && x.v < 2.0
            {
                match self.watch.get_mut(&x.id) {
                    Some(w) => w.after = Some(go),
                    None if !self.seen.contains(&x.id) => {
                        self.queued.insert(x.id, go);
                    }
                    None => {}
                }
            }
        }
        self.settle(sim);
    }

    /// Record the choices of the drivers that went and whose front has passed every
    /// conflict point on its path, once someone has passed behind them.
    fn settle(&mut self, sim: &Sim) {
        let mut done = Vec::new();
        for (&id, w) in &self.watch {
            let Some(go) = w.go else { continue };
            let points = &sim.net.links[w.link].conflicts;
            if points.iter().any(|cid| !w.passed.contains_key(cid)) {
                continue;
            }
            let Some(all) = self.passes.get(&w.link) else { continue };
            let mut events: Vec<(f64, bool)> = (all.iter())
                .filter(|e| e.1 > w.rank && e.0 > w.ready)
                .map(|&(when, _, cid)| (when, when > w.passed[&cid]))
                .collect();
            let Some(first_behind) = events.iter().filter(|e| e.1).map(|e| e.0).reduce(f64::min) else {
                continue;
            };
            done.push(id);
            events.sort_by(|a, b| a.0.total_cmp(&b.0));
            let last_ahead = events.iter().filter(|e| !e.1).map(|e| e.0).reduce(f64::max);
            // Someone passed ahead of it at one point after someone else passed behind it
            // at another: it went between lanes, not in one gap.
            if last_ahead.is_some_and(|a| a > first_behind) {
                continue;
            }
            let before: Vec<f64> = events.iter().map(|e| e.0).filter(|&x| x <= go).collect();
            let mut rejected = Vec::new();
            let mut prev = w.ready;
            for &x in &before {
                rejected.push(x - prev);
                prev = x;
            }
            let (accepted, lag) = match last_ahead {
                Some(a) => (first_behind - a, false),
                None => (first_behind - w.ready, true),
            };
            // It went in the same gap as the one ahead of it if nobody passed in between.
            let follow_up = w
                .after
                .filter(|&a| !all.iter().any(|e| e.1 > w.rank && e.0 > a && e.0 <= go))
                .map(|a| go - a);
            self.choices.push(Choice {
                movement: w.movement,
                kind: w.kind,
                rejected,
                accepted,
                lag,
                follow_up,
            });
        }
        for id in done {
            self.watch.remove(&id);
        }
    }
}

/// The standard normal distribution function, to within 1.5e-7 (Abramowitz and Stegun,
/// 7.1.26).
fn phi(x: f64) -> f64 {
    let z = x.abs() / std::f64::consts::SQRT_2;
    let k = 1.0 / (1.0 + 0.327_591_1 * z);
    let poly = k
        * (0.254_829_592
            + k * (-0.284_496_736 + k * (1.421_413_741 + k * (-1.453_152_027 + k * 1.061_405_429))));
    let erf = 1.0 - poly * (-z * z).exp();
    0.5 * (1.0 + erf.copysign(x))
}

/// The mean critical headway (s) of the drivers who made `choices`, by maximum likelihood
/// (Troutbeck, 1992), the method the Highway Capacity Manual's field studies used. Each
/// driver's critical headway lies between the longest lag or gap it rejected and the one
/// it accepted, and across drivers critical headways are taken to be log-normal. Drivers
/// who accepted less than they had rejected are left out. `None` with fewer than 10 drivers.
pub fn critical_headway<'a>(choices: impl IntoIterator<Item = &'a Choice>) -> Option<f64> {
    let bounds: Vec<(f64, f64)> = (choices.into_iter())
        .map(|c| (c.rejected.iter().copied().fold(0.0, f64::max), c.accepted))
        .filter(|&(r, a)| a > r)
        .map(|(r, a)| (if r > 0.0 { r.ln() } else { f64::NEG_INFINITY }, a.ln()))
        .collect();
    if bounds.len() < 10 {
        return None;
    }
    let ll = |mu: f64, sigma: f64| -> f64 {
        bounds
            .iter()
            .map(|&(r, a)| {
                let lo = if r.is_finite() { phi((r - mu) / sigma) } else { 0.0 };
                (phi((a - mu) / sigma) - lo).max(1e-300).ln()
            })
            .sum()
    };
    // A coarse grid, then a pattern search from its best point.
    let mut best = (0.0, 0.5, f64::NEG_INFINITY);
    for i in 0..=80 {
        let mu = i as f64 * 0.04;
        for j in 1..=30 {
            let sigma = j as f64 * 0.05;
            let l = ll(mu, sigma);
            if l > best.2 {
                best = (mu, sigma, l);
            }
        }
    }
    let (mut mu, mut sigma, mut l) = best;
    let mut step = 0.02;
    while step > 1e-5 {
        let mut moved = false;
        for (dm, ds) in [(step, 0.0), (-step, 0.0), (0.0, step), (0.0, -step)] {
            let (m2, s2) = (mu + dm, (sigma + ds).max(1e-3));
            let l2 = ll(m2, s2);
            if l2 > l {
                (mu, sigma, l) = (m2, s2, l2);
                moved = true;
            }
        }
        if !moved {
            step /= 2.0;
        }
    }
    Some((mu + sigma * sigma / 2.0).exp())
}
