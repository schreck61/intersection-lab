// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Manual control: a person chooses each stage, with the app's buttons, its keys or a
//! click on an approach. The choice reaches the signal the way every controller's wish
//! does, through `Sim`'s control step and `SignalState::request`, so the minimum green,
//! the pedestrian intervals, the yellow change and all-red clearance and emergency
//! preemption all still apply. A choice waits until the signal can honor it. Without
//! one, the current stage holds: a person gets no max-wait guard.

use super::{Brain, Controller, Ctx};
use crate::sim::network::{MStat, Network, Turn};
use crate::sim::signal::{PedSig, SignalState};

pub struct Manual {
    /// The stage the person chose, until the signal starts changing to it.
    pending: Option<usize>,
    /// The layout's usual order of stages, which `advance` follows.
    seq: Vec<usize>,
    names: Vec<String>,
    /// Each movement's name for people, such as "Southbound Left".
    move_names: Vec<String>,
    /// For each stage, the people it would serve that the sensors report waiting:
    /// vehicle occupants in queues and pedestrians at push buttons.
    waiting: Vec<f64>,
    /// The longest wait the sensors report for a vehicle: (movement, seconds).
    longest: Option<(usize, f64)>,
    /// The stage green at the last step.
    cur: usize,
}

impl Manual {
    pub fn new(net: &Network) -> Self {
        let n = net.stages.len();
        let mut seq: Vec<usize> = net.fixed_sequence.iter().copied().filter(|&j| j < n).collect();
        if seq.is_empty() {
            seq = (0..n).collect();
        }
        Manual {
            pending: None,
            cur: seq.first().copied().unwrap_or(0),
            seq,
            names: net.stages.iter().map(|s| s.name.clone()).collect(),
            move_names: net
                .movements
                .iter()
                .map(|m| format!("{} {}", net.legs[m.leg].compass.bound(), m.turn.name()))
                .collect(),
            waiting: vec![0.0; n],
            longest: None,
        }
    }

    /// The stage the person chose that the signal hasn't started changing to yet.
    pub fn pending(&self, sig: &SignalState) -> Option<usize> {
        // `next` is the green stage, or during a change the stage that comes next. The
        // choice is cleared at the step after the change begins.
        self.pending.filter(|&j| sig.next != j)
    }

    /// Choose stage `j`: the signal changes to it as soon as it can. Choosing the stage
    /// that's green, or the one the signal is already changing to, drops any earlier
    /// choice. Returns false, changing nothing, if there is no stage `j`.
    pub fn choose(&mut self, j: usize, sig: &SignalState) -> bool {
        if j >= self.names.len() {
            return false;
        }
        self.pending = (sig.next != j).then_some(j);
        true
    }

    /// Choose the stage that follows the waiting choice, or the stage that's green or
    /// coming next, in the layout's usual order, wrapping around. From a stage outside
    /// that order, such as a split stage, start the order again. Returns the stage.
    pub fn advance(&mut self, sig: &SignalState) -> Option<usize> {
        let from = self.pending(sig).unwrap_or(sig.next);
        let j = match self.seq.iter().position(|&s| s == from) {
            Some(i) => self.seq[(i + 1) % self.seq.len()],
            None => *self.seq.first()?,
        };
        self.choose(j, sig);
        Some(j)
    }

    /// A sentence for the app: what's green, what's chosen and what the choice waits
    /// for. `preempting` says whether emergency preemption has the signal.
    pub fn status(&self, sig: &SignalState, preempting: bool) -> String {
        let name = |j: usize| self.names.get(j).map_or("?", String::as_str);
        let changing = sig.next != sig.stage;
        match self.pending(sig) {
            Some(j) if preempting => {
                format!("Emergency preemption has the signal; {} waits until it ends.", name(j))
            }
            Some(j) if changing => format!("Next: {}, after the change to {}.", name(j), name(sig.next)),
            Some(j) => {
                // A little slack keeps a whole number of seconds from showing one more.
                let left = (sig.cur_min - sig.green_elapsed - 1e-6).ceil();
                if left <= 0.0 {
                    format!("Next: {}. The change starts at the next step.", name(j))
                } else {
                    let why = if sig.cw.iter().any(|c| c.state != PedSig::Dw) {
                        "the WALK and flashing DON'T WALK end"
                    } else {
                        "the minimum green has run"
                    };
                    format!("Next: {}, in {left} s, when {why}.", name(j))
                }
            }
            None if preempting => "Emergency preemption has the signal.".into(),
            None if changing => format!("Changing to {}.", name(sig.next)),
            None => format!("{} holds until you choose another stage.", name(sig.stage)),
        }
    }

    /// Update the brain's figures from what the sensors report.
    fn measure(&mut self, ctx: &Ctx) {
        let (net, view) = (ctx.net, ctx.view);
        self.cur = ctx.sig.stage;
        self.waiting.clear();
        self.waiting.extend(net.stages.iter().map(|st| {
            let riders: f64 =
                (0..st.stat.len()).filter(|&m| st.stat[m].is_some()).map(|m| view.moves[m].pqueue).sum();
            let walkers: u32 = st.walk.iter().map(|&cw| view.ped_waiting[cw]).sum();
            riders + walkers as f64
        }));
        self.longest = (view.moves.iter().enumerate())
            .map(|(m, o)| (m, o.max_wait))
            .filter(|&(_, w)| w >= 1.0)
            .max_by(|a, b| a.1.total_cmp(&b.1));
    }
}

/// The stage a click on inbound lane `lane` of leg `leg` chooses. The lane's main
/// movement is its only one, or its through movement when the lane is shared. The click
/// chooses a stage that gives that movement a green ball or arrow, taking the first in
/// the layout's usual order, then the lowest numbered; failing that, a stage that gives
/// it a right-turn green, then a permissive one. With `alone`, it chooses among the
/// stages that serve that approach and no other, when one of them serves the movement.
pub fn stage_for(net: &Network, leg: usize, lane: usize, alone: bool) -> Option<usize> {
    let link = *net.legs.get(leg)?.in_lanes.get(lane)?;
    let moves = &net.links[link].lane_moves;
    let &m = moves.iter().find(|&&m| net.movements[m].turn == Turn::T).or(moves.first())?;
    let solo =
        |j: usize| net.stages[j].stat.iter().zip(&net.movements).all(|(s, mv)| s.is_none() || mv.leg == leg);
    let pick = |only_solo: bool| {
        (0..net.stages.len())
            .filter(|&j| !only_solo || solo(j))
            .filter_map(|j| {
                let rank = match net.stages[j].stat[m]? {
                    MStat::G => 0,
                    MStat::Rt => 1,
                    MStat::P => 2,
                };
                let order = net.fixed_sequence.iter().position(|&s| s == j).unwrap_or(usize::MAX);
                Some((rank, order, j))
            })
            .min()
            .map(|(_, _, j)| j)
    };
    if alone { pick(true).or_else(|| pick(false)) } else { pick(false) }
}

impl Controller for Manual {
    fn name(&self) -> &'static str {
        "Manual"
    }

    fn step(&mut self, ctx: &Ctx) -> Option<usize> {
        self.measure(ctx);
        let j = self.pending?;
        if ctx.sig.next == j {
            // The change has begun, or preemption gave `j` green: the choice is done.
            self.pending = None;
            return None;
        }
        ctx.can_switch.then_some(j)
    }

    fn brain(&self) -> Brain {
        let mut notes = vec![
            "Bars: the people each stage would serve that the sensors report waiting, in vehicles and at push buttons."
                .to_string(),
        ];
        if let Some((m, w)) = self.longest {
            notes.push(format!("Longest sensed wait: {w:.0} s, {}.", self.move_names[m]));
        }
        notes.push("A stage holds until you choose another: no max-wait guard steps in.".into());
        Brain {
            title: "Manual control: you choose each stage".into(),
            bars: (self.names.iter().zip(&self.waiting).enumerate())
                .map(|(j, (n, &w))| (format!("{} {n}", j + 1), w, j == self.cur))
                .collect(),
            lower_is_better: false,
            notes,
        }
    }

    fn as_manual(&mut self) -> Option<&mut Manual> {
        Some(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::CtrlKind;
    use crate::sim::network::{Compass, LayoutKind};
    use crate::sim::signal::Phase;
    use crate::sim::vtypes::VKind;
    use crate::sim::{ControlMode, Sim};

    /// Arterial traffic under manual control, without pedestrians or emergency vehicles
    /// unless a test adds them.
    fn sim() -> Sim {
        let mut s = Sim::new(LayoutKind::Arterial4, ControlMode::Signal, CtrlKind::Manual, 7);
        s.demand.ped_rate = 0.0;
        s.demand.mix[VKind::Emergency.index()] = 0.0;
        s
    }

    fn choose(s: &mut Sim, j: usize) -> bool {
        let sig = &s.signal;
        s.controller.as_manual().unwrap().choose(j, sig)
    }

    fn advance(s: &mut Sim) -> Option<usize> {
        let sig = &s.signal;
        s.controller.as_manual().unwrap().advance(sig)
    }

    fn pending(s: &mut Sim) -> Option<usize> {
        let sig = &s.signal;
        s.controller.as_manual().unwrap().pending(sig)
    }

    fn status(s: &mut Sim) -> String {
        let (sig, pre) = (&s.signal, s.preempt_active);
        s.controller.as_manual().unwrap().status(sig, pre)
    }

    /// Step until `f` holds, for at most `secs` seconds. Returns whether it held.
    fn run_until(s: &mut Sim, secs: f64, f: impl Fn(&Sim) -> bool) -> bool {
        for _ in 0..(secs / s.dt).round() as usize {
            if f(s) {
                return true;
            }
            s.step();
        }
        f(s)
    }

    fn green_on(j: usize) -> impl Fn(&Sim) -> bool {
        move |s: &Sim| s.signal.stage == j && s.signal.phase == Phase::Green
    }

    /// One step's rounding is allowed on each interval.
    fn close(x: f64, y: f64) -> bool {
        (x - y).abs() <= 0.1 + 1e-9
    }

    #[test]
    fn a_choice_waits_for_the_minimum_green_then_clears_in_full() {
        let mut s = sim();
        let (a, b) = (s.signal.stage, 2);
        assert_eq!(
            (s.net.stages[a].name.as_str(), s.net.stages[b].name.as_str()),
            ("NS Protected Left", "EW Through")
        );
        assert!(choose(&mut s, b));
        assert_eq!(pending(&mut s), Some(b));
        assert!(run_until(&mut s, 60.0, green_on(b)));
        assert_eq!(pending(&mut s), None);
        let h: Vec<(usize, Phase, f64)> =
            s.signal.history.iter().map(|x| (x.stage, x.phase, x.end - x.start)).collect();
        let kinds: Vec<(usize, Phase)> = h.iter().map(|x| (x.0, x.1)).collect();
        assert_eq!(kinds, [(a, Phase::Green), (a, Phase::Yellow), (a, Phase::AllRed), (b, Phase::Green)]);
        assert!(close(h[0].2, s.signal.min_green), "green {}", h[0].2);
        assert!(close(h[1].2, s.net.yellow) && close(h[2].2, s.net.all_red), "{h:?}");
    }

    /// No max-wait guard: ten minutes of traffic starving on red don't change the stage.
    #[test]
    fn without_a_choice_the_stage_holds() {
        let mut s = sim();
        s.demand.ped_rate = 60.0;
        let a = s.signal.stage;
        s.run_for(600.0);
        assert!(s.view.moves.iter().any(|m| m.max_wait > s.max_wait_guard), "nobody waited past the guard");
        assert_eq!((s.signal.stage, s.signal.phase, s.signal.switches), (a, Phase::Green, 0));
    }

    #[test]
    fn a_choice_can_be_replaced_canceled_or_refused() {
        let mut s = sim();
        let (green, n) = (s.signal.stage, s.net.stages.len());
        assert!(choose(&mut s, 2) && choose(&mut s, 3));
        assert_eq!(pending(&mut s), Some(3), "a new choice replaces the old");
        assert!(!choose(&mut s, n));
        assert_eq!(pending(&mut s), Some(3), "a stage that doesn't exist is refused");
        assert!(choose(&mut s, green));
        assert_eq!(pending(&mut s), None, "choosing the green stage cancels");

        assert!(choose(&mut s, 3));
        assert!(run_until(&mut s, 30.0, |s| s.signal.phase == Phase::Yellow));
        assert_eq!((s.signal.next, pending(&mut s)), (3, None), "the choice is done once the change begins");
        assert!(choose(&mut s, 3));
        assert_eq!(pending(&mut s), None, "the signal is already changing to it");
        assert!(choose(&mut s, green));
        assert_eq!(pending(&mut s), Some(green), "the ending stage can come back after the change");
        assert!(run_until(&mut s, 60.0, green_on(green)));
    }

    #[test]
    fn next_stage_follows_the_usual_order_and_wraps() {
        let mut s = sim();
        let seq = s.net.fixed_sequence.clone();
        assert_eq!(s.signal.stage, seq[0]);
        for &j in &seq[1..] {
            assert_eq!(advance(&mut s), Some(j));
            assert_eq!(pending(&mut s), Some(j));
        }
        // Wrapping round to the green stage cancels the choice.
        assert_eq!(advance(&mut s), Some(seq[0]));
        assert_eq!(pending(&mut s), None);
        // From a split stage, outside the order, Next starts the order again.
        let split = (0..s.net.stages.len()).find(|j| !seq.contains(j)).unwrap();
        assert!(choose(&mut s, split));
        assert!(run_until(&mut s, 60.0, green_on(split)));
        assert_eq!(advance(&mut s), Some(seq[0]));
        assert_eq!(pending(&mut s), Some(seq[0]));
    }

    /// A WALK that starts with the stage, with its leading pedestrian interval, runs in
    /// full before a choice made earlier takes effect.
    #[test]
    fn pedestrian_intervals_hold_a_choice_back() {
        let mut s = sim();
        s.signal.lpi = 3.0;
        let cw = s.net.legs[s.net.leg_index(Compass::E).unwrap()].crosswalk.unwrap();
        assert!(s.net.stages[0].walk.contains(&cw), "NS Through walks the east crosswalk");
        assert!(choose(&mut s, 0));
        assert!(run_until(&mut s, 30.0, |s| s.signal.next == 0 && s.signal.phase != Phase::Green));
        s.spawn_ped(cw, false);
        assert!(choose(&mut s, 2));
        assert!(run_until(&mut s, 30.0, green_on(0)));
        s.step();
        assert_eq!(s.signal.cw[cw].state, PedSig::Walk);
        assert!((0..s.net.movements.len()).any(|m| s.signal.lpi_held(m)), "no leading interval");
        assert!(status(&mut s).ends_with("when the WALK and flashing DON'T WALK end."), "{}", status(&mut s));
        assert!(run_until(&mut s, 60.0, green_on(2)));
        let green = s.signal.history.iter().rev().find(|h| h.stage == 0 && h.phase == Phase::Green).unwrap();
        let fdw = s.net.crosswalks[cw].length / s.signal.ped_speed;
        let need = (s.signal.walk_time + fdw).max(s.signal.lpi + s.signal.min_green);
        assert!(green.end - green.start >= need - 0.1 - 1e-9, "green {} < {need}", green.end - green.start);
    }

    /// An emergency vehicle's preemption outranks a choice; the choice runs once it ends.
    #[test]
    fn preemption_overrides_a_choice_that_then_runs() {
        let mut s = sim();
        s.demand.scale = 0.0;
        let e = s.net.leg_index(Compass::E).unwrap();
        assert!(s.spawn_manual(e, VKind::Emergency, Turn::T));
        assert!(run_until(&mut s, 60.0, |s| s.preempt_active));
        assert!(choose(&mut s, 0));
        assert_eq!(status(&mut s), "Emergency preemption has the signal; NS Through waits until it ends.");
        while s.preempt_active {
            assert_ne!(s.signal.next, 0, "the choice ran during preemption");
            s.step();
            assert!(s.t < 120.0, "preemption never ended");
        }
        let et = s.net.movement_for(e, Turn::T).unwrap();
        assert!(
            s.signal.history.iter().any(|h| s.net.stages[h.stage].stat[et].is_some()),
            "preemption served nothing"
        );
        assert!(run_until(&mut s, 60.0, green_on(0)));
    }

    #[test]
    fn the_status_says_what_the_choice_waits_for() {
        let mut s = sim();
        assert_eq!(status(&mut s), "NS Protected Left holds until you choose another stage.");
        choose(&mut s, 2);
        assert_eq!(status(&mut s), "Next: EW Through, in 6 s, when the minimum green has run.");
        s.run_for(2.5);
        assert_eq!(status(&mut s), "Next: EW Through, in 4 s, when the minimum green has run.");
        s.preempt_active = true;
        assert_eq!(status(&mut s), "Emergency preemption has the signal; EW Through waits until it ends.");
        s.preempt_active = false;
        assert!(run_until(&mut s, 10.0, |s| s.signal.phase == Phase::Yellow));
        assert_eq!(status(&mut s), "Changing to EW Through.");
        choose(&mut s, 0);
        assert_eq!(status(&mut s), "Next: NS Through, after the change to EW Through.");
        choose(&mut s, 2);
        s.preempt_active = true;
        assert_eq!(status(&mut s), "Emergency preemption has the signal.");
        s.preempt_active = false;
        assert!(run_until(&mut s, 10.0, green_on(2)));
        choose(&mut s, 0);
        assert!(run_until(&mut s, 10.0, |s| s.signal.can_switch()));
        assert_eq!(status(&mut s), "Next: NS Through. The change starts at the next step.");
        s.step();
        assert_eq!((s.signal.phase, s.signal.next), (Phase::Yellow, 0));
        assert_eq!(status(&mut s), "Changing to NS Through.");
    }

    #[test]
    fn a_click_chooses_the_stage_that_serves_the_lane() {
        use Compass::*;
        use LayoutKind::*;
        let cases = [
            (Arterial4, N, 0, false, "NS Protected Left"),
            (Arterial4, N, 1, false, "NS Through"),
            (Arterial4, N, 2, false, "NS Through"),
            (Arterial4, N, 0, true, "North Split"),
            (Arterial4, N, 2, true, "North Split"),
            (Arterial4, E, 1, false, "EW Through"),
            (Arterial4, E, 1, true, "East Split"),
            (MajorMinor4, E, 0, false, "EW Protected Left"),
            (MajorMinor4, E, 2, false, "EW Through"),
            (MajorMinor4, S, 0, false, "NS Side Street"),
            (MajorMinor4, S, 0, true, "South Split"),
            (Town4, W, 0, false, "EW (permissive)"),
            (Town4, W, 0, true, "West Split"),
            (Tee, E, 0, false, "East + Protected Left"),
            (Tee, E, 1, false, "Main Street"),
            (Tee, E, 1, true, "East + Protected Left"),
            (Tee, S, 0, false, "Side Street"),
            (Tee, S, 1, false, "Side Street"),
            // No stage serves the west approach alone, so a shift-click falls back.
            (Tee, W, 1, true, "Main Street"),
        ];
        for (kind, c, lane, alone, want) in cases {
            let net = Network::build(kind);
            let leg = net.leg_index(c).unwrap();
            let got = stage_for(&net, leg, lane, alone).map(|j| net.stages[j].name.as_str());
            assert_eq!(got, Some(want), "{kind:?} {c:?} lane {lane}, alone {alone}");
        }
        let net = Network::build(Arterial4);
        assert_eq!(stage_for(&net, 0, 9, false), None);
        assert_eq!(stage_for(&net, 9, 0, false), None);
    }

    #[test]
    fn the_brain_shows_people_waiting_for_each_stage() {
        let mut s = sim();
        s.run_for(120.0);
        let b = s.controller.brain();
        assert_eq!(b.bars.len(), s.net.stages.len());
        assert_eq!(b.bars[0].0, "1 NS Through");
        let lit: Vec<usize> = (0..b.bars.len()).filter(|&j| b.bars[j].2).collect();
        assert_eq!(lit, [s.signal.stage]);
        assert!(b.bars.iter().any(|r| r.1 > 1.0), "{:?}", b.bars);
        assert!(b.notes.iter().any(|n| n.starts_with("Longest sensed wait: ")), "{:?}", b.notes);
        assert!(b.notes.iter().any(|n| n.contains("no max-wait guard")));
    }
}
