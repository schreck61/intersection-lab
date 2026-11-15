// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Failures at the signal-free cooperative crossing: what each gap policy survives when
//! a vehicle stops dead or brakes to a stop, at a random place and at the worst moment
//! for crossing traffic; that vehicles whose paths stay clear keep crossing; that the
//! box is cleared and the crossing recovers, one stopped vehicle letting another pass
//! where it must; what a failure among the stopped vehicles and one at the edge of the
//! map leave behind; that a failure only ever holds vehicles back, and that a vehicle
//! which cannot stop in time is expected where it ends up; what the log of failures
//! keeps; and that failures can be repeated at the same place on another run.

use intersection_lab::control::CtrlKind;
use intersection_lab::sim::coop::{
    CoopStats, Fault, GapPolicy, IMPACT_OK, Incident, Place, PlanKind, REACTION, Scheduling,
};
use intersection_lab::sim::demand::DemandConfig;
use intersection_lab::sim::network::{LayoutKind, LinkId, Turn, build_cooperative};
use intersection_lab::sim::vtypes::VKind;
use intersection_lab::sim::{ControlMode, Reason, Sim, obb_overlap};
use std::collections::{BTreeMap, BTreeSet};

mod common;

/// The crossing at `total` veh/h, with the safety checks on.
fn crossing(total: f64, policy: GapPolicy, sched: Scheduling, seed: u64) -> Sim {
    let net = build_cooperative(2);
    let mut demand = DemandConfig::default_for(&net);
    demand.rate = [total / 4.0; 4];
    let mut sim = Sim::with_demand(net, ControlMode::Cooperative, CtrlKind::Fixed, demand, seed);
    sim.set_gap_policy(policy);
    sim.set_scheduling(sched);
    sim.check_safety = true;
    sim
}

/// Demand each schedule carries with room to spare.
fn demand_for(sched: Scheduling) -> f64 {
    match sched {
        Scheduling::Batches => 16_000.0,
        Scheduling::FirstCome => 8_000.0,
    }
}

/// Inbound lanes whose paths cross the box.
fn box_lanes(sim: &Sim) -> Vec<LinkId> {
    sim.net
        .movements
        .iter()
        .filter(|m| m.turn != Turn::R)
        .flat_map(|m| m.routes.iter().map(|r| r.0))
        .collect()
}

fn stats(sim: &Sim) -> &CoopStats {
    &sim.coop.as_ref().expect("an intersection manager").stats
}

fn first_incident(sim: &Sim) -> Incident {
    sim.coop.as_ref().unwrap().incidents.front().cloned().expect("a failure")
}

/// The vehicles at rest after an emergency stop, each with its lane and how far along
/// its route its front is: lane by lane, the one in front first.
fn stranded(sim: &Sim) -> Vec<(u64, LinkId, f64)> {
    let mut at_rest: Vec<(u64, LinkId, f64)> = sim
        .vehicles
        .iter()
        .filter(|v| v.reason == Reason::Stranded)
        .filter_map(|v| sim.place_of(v.id).map(|(lane, x)| (v.id, lane, x)))
        .collect();
    at_rest.sort_by(|a, b| a.1.cmp(&b.1).then(b.2.total_cmp(&a.2)));
    at_rest
}

/// Vehicles that left their crossings for a failure and brake to rest or stand there:
/// what the app's map counts as stopped for it.
fn stopped(sim: &Sim) -> usize {
    let plans = &sim.coop.as_ref().unwrap().plans;
    plans.values().filter(|p| matches!(p.kind, PlanKind::EmergencyStop | PlanKind::StopBehind)).count()
}

/// Step until `done`, for `seconds` at most. True if it came to that.
fn run_until(sim: &mut Sim, seconds: f64, done: impl Fn(&Sim) -> bool) -> bool {
    for _ in 0..(seconds / sim.dt).round() as usize {
        if done(sim) {
            return true;
        }
        sim.step();
    }
    done(sim)
}

/// A run with random failures: `set` is how many an hour, of which kind, and how long a
/// disabled vehicle stays.
fn with_failures(
    (total, policy, sched, seed): (f64, GapPolicy, Scheduling, u64),
    set: (f64, Fault, f64),
) -> Sim {
    let mut sim = crossing(total, policy, sched, seed);
    let c = sim.coop.as_mut().unwrap();
    (c.stall_rate, c.stall_fault, c.clear_time) = set;
    sim
}

/// No more failures: those under way are dealt with, and two minutes pass.
fn settle(sim: &mut Sim) {
    let c = sim.coop.as_mut().unwrap();
    c.stall_rate = 0.0;
    c.traps.clear();
    sim.run_for(120.0);
}

/// What should never happen, failure or not: a vehicle off the trajectory it holds, in
/// the box without one, overlapping another without that being a recorded contact, or
/// still disabled or at rest in the box long after its failure was cleared away.
fn wrong(name: &str, sim: &Sim) -> Vec<String> {
    let st = stats(sim);
    let mut f = vec![];
    if st.deviations > 0 {
        f.push(format!("{name}: {} steps off a trajectory", st.deviations));
    }
    if st.unauthorized > 0 {
        f.push(format!("{name}: {} box entries without a reservation", st.unauthorized));
    }
    if sim.collisions > 0 {
        f.push(format!("{name}: {} overlap ticks outside any recorded contact", sim.collisions));
    }
    if st.timing_err_max > 1e-6 {
        f.push(format!("{name}: zone times off a trajectory by {:.2e} s", st.timing_err_max));
    }
    let disabled = sim.vehicles.iter().filter(|v| v.disabled).count();
    let stopped = sim.vehicles.iter().filter(|v| v.reason == Reason::Stranded).count();
    if disabled + stopped > 0 {
        f.push(format!("{name}: {disabled} disabled and {stopped} stopped vehicles left behind"));
    }
    if !sim.coop.as_ref().unwrap().wrecks.is_empty() {
        f.push(format!("{name}: wrecks still on the manager's books"));
    }
    f
}

/// One failure at the worst moment for crossing traffic, and four minutes after it.
fn worst_moment(policy: GapPolicy, sched: Scheduling, fault: Fault, seed: u64) -> Sim {
    let mut sim = crossing(demand_for(sched), policy, sched, seed);
    sim.run_for(180.0);
    assert!(sim.stall_at_worst(fault));
    sim.run_for(240.0);
    sim
}

/// The runs of one policy at the worst moment, under both schedules.
fn worst_runs(policy: GapPolicy, fault: Fault, seeds: u64) -> Vec<(String, Sim)> {
    let jobs: Vec<_> = Scheduling::ALL
        .into_iter()
        .flat_map(|sched| (0..seeds).map(move |seed| (sched, seed)))
        .map(|(sched, seed)| {
            common::spawn(move || {
                let name =
                    format!("{} / {} / {} / seed {seed}", policy.short(), sched.label(), fault.label());
                (name, worst_moment(policy, sched, fault, 300 + seed))
            })
        })
        .collect();
    jobs.into_iter().map(|j| j.join().unwrap()).collect()
}

#[test]
fn fail_safe_gaps_stop_crossing_traffic_short_of_a_vehicle_that_stops_dead() {
    // The failure the fail-safe gap is sized for, at the moment that leaves a crossing
    // vehicle the least room: nothing may touch, and in batches every emergency stop
    // ends short of its zone.
    //
    // First come, first served gets within a meter of that. The gap is sized for the
    // speed at which a vehicle enters a zone, and a vehicle still slowing from the
    // approach speed to the crossing speed carries more than that when it has to
    // react: now and then its stop ends a few decimeters inside the zone, clear of the
    // failed vehicle. Seed 4270 is such a run.
    let mut runs = worst_runs(GapPolicy::FailSafe, Fault::Dead, 12);
    runs.push(("P2 / first come, first served / a vehicle still slowing".to_string(), {
        let mut sim = crossing(8_000.0, GapPolicy::FailSafe, Scheduling::FirstCome, 4270);
        sim.run_for(180.0 + intersection_lab::sim::rng::Rng::new(4270 ^ 0xFA11).range(0.0, 30.0));
        assert!(sim.stall_at_worst(Fault::Dead));
        sim.run_for(240.0);
        sim
    }));
    let mut failures = vec![];
    let mut stops = 0;
    for (name, sim) in &runs {
        let (st, inc) = (stats(sim), first_incident(sim));
        failures.extend(wrong(name, sim));
        assert_eq!(st.stalls, 1, "{name}");
        assert_eq!(inc.place, Place::Box, "{name}");
        assert_eq!(inc.fault, Fault::Dead, "{name}");
        if st.crossing_impacts > 0 {
            failures.push(format!("{name}: crossing traffic ran into the failed vehicle"));
        }
        let floor = if name.contains(Scheduling::Batches.label()) { 0.0 } else { -1.0 };
        if st.stop_margin_min < floor {
            failures.push(format!(
                "{name}: an emergency stop ended {:.2} m inside its zone",
                -st.stop_margin_min
            ));
        }
        if st.zone_violations > 0 && st.stop_margin_min >= 0.0 {
            failures.push(format!("{name}: {} zones entered while occupied", st.zone_violations));
        }
        // No vehicle brakes sooner than the reaction time, and from the approach speed
        // an emergency stop takes under three seconds more.
        if st.emergency_stops > 0 && !(REACTION..REACTION + 3.0).contains(&st.settle_max) {
            failures.push(format!("{name}: the emergency stops took {:.1} s", st.settle_max));
        }
        stops += st.emergency_stops;
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
    assert!(stops as usize >= runs.len(), "{stops} emergency stops in {} failures", runs.len());
    // The closest stop comes within a few meters of its zone: the gap has little to spare.
    let closest = runs.iter().map(|r| stats(&r.1).stop_margin_min).fold(f64::INFINITY, f64::min);
    assert!((-1.0..4.0).contains(&closest), "the closest stop was {closest:.2} m short");
    let slowing = stats(&runs.last().unwrap().1).stop_margin_min;
    assert!((-1.0..0.0).contains(&slowing), "the vehicle still slowing stopped {slowing:.2} m short");
}

#[test]
fn thinner_gaps_let_crossing_traffic_reach_a_vehicle_that_stops_dead() {
    // The mitigated gap accepts a contact at no more than 25 km/h; the nominal gap
    // assumes that no vehicle stops dead, and pays for it at close to the crossing speed.
    let jobs = [GapPolicy::Mitigated, GapPolicy::Nominal]
        .map(|policy| std::thread::spawn(move || worst_runs(policy, Fault::Dead, 12)));
    let [mitigated, nominal] = jobs.map(|j| j.join().unwrap());
    let mut failures = vec![];
    let hit = |runs: &[(String, Sim)]| runs.iter().filter(|r| stats(&r.1).crossing_impacts > 0).count();
    let fastest = |runs: &[(String, Sim)]| {
        runs.iter().map(|r| stats(&r.1).crossing_impact_max).fold(0.0, f64::max) * 3.6
    };
    for (name, sim) in mitigated.iter().chain(&nominal) {
        failures.extend(wrong(name, sim));
        assert_eq!(stats(sim).stalls, 1, "{name}");
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
    let (m_hit, n_hit) = (hit(&mitigated), hit(&nominal));
    assert!(
        m_hit * 4 >= mitigated.len(),
        "mitigated gaps: contacts in {m_hit} of {} failures",
        mitigated.len()
    );
    assert!(
        n_hit * 4 >= 3 * nominal.len(),
        "nominal gaps: contacts in {n_hit} of {} failures",
        nominal.len()
    );
    assert!(
        fastest(&mitigated) <= IMPACT_OK * 3.6 + 0.5,
        "mitigated gaps: a contact at {:.1} km/h",
        fastest(&mitigated)
    );
    assert!(
        fastest(&nominal) > IMPACT_OK * 3.6 + 3.0,
        "nominal gaps: the fastest contact was only {:.1} km/h",
        fastest(&nominal)
    );
    // Every contact is in the log of its failure, with the vehicles that touched.
    for (name, sim) in &nominal {
        let inc = first_incident(sim);
        let st = stats(sim);
        assert_eq!(inc.impacts.len() as u64, st.crossing_impacts + st.lane_impacts, "{name}");
        assert!(inc.impacts.iter().all(|i| i.striker != i.struck && i.speed >= 0.0), "{name}");
        if st.crossing_impacts > 0 {
            assert!(inc.margin < 0.0, "{name}: contact, yet the closest stop was {:.2} m short", inc.margin);
            assert!(st.cleared >= 2, "{name}: the vehicles that touched were not both cleared away");
        }
    }
}

#[test]
fn a_vehicle_that_brakes_to_a_stop_is_survived() {
    // The failure the nominal gap is sized for. With the wider gaps nothing touches;
    // with the nominal one a vehicle may end inside its zone, but slowly if at all.
    let jobs = GapPolicy::ALL.map(|policy| std::thread::spawn(move || worst_runs(policy, Fault::Braking, 8)));
    let mut failures = vec![];
    for (policy, runs) in GapPolicy::ALL.into_iter().zip(jobs.map(|j| j.join().unwrap())) {
        for (name, sim) in &runs {
            let st = stats(sim);
            failures.extend(wrong(name, sim));
            assert_eq!(first_incident(sim).fault, Fault::Braking, "{name}");
            let contacts = st.crossing_impacts + st.lane_impacts;
            if policy != GapPolicy::Nominal && contacts > 0 {
                failures.push(format!("{name}: {contacts} contacts"));
            }
            if st.crossing_impact_max > IMPACT_OK {
                failures.push(format!("{name}: a contact at {:.0} km/h", st.crossing_impact_max * 3.6));
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn in_batches_a_failure_rarely_reaches_crossing_traffic() {
    // Mid-batch, the next vehicle to cross the failed one's path is a whole phase away:
    // its crossing is withdrawn and it never needs the gap, even the nominal one. What
    // a dead stop does do is stop the vehicle right behind it, which cannot stop in time.
    let run = |fault: Fault, seed: u64| {
        common::spawn(move || {
            let mut sim = crossing(16_000.0, GapPolicy::Nominal, Scheduling::Batches, 500 + seed);
            sim.run_for(180.0 + seed as f64);
            let lanes = box_lanes(&sim);
            let lane = lanes[(seed as usize * 5) % lanes.len()];
            let (l_in, l_box) = sim.coop.as_ref().unwrap().box_span(lane).unwrap();
            assert!(sim.stall_at(lane, l_in + (0.15 + 0.07 * seed as f64 % 0.7) * (l_box - l_in), fault));
            sim.run_for(240.0);
            (format!("{} / seed {seed}", fault.label()), sim)
        })
    };
    let jobs: Vec<_> = Fault::ALL.into_iter().flat_map(|f| (0..12).map(move |s| run(f, s))).collect();
    let runs: Vec<(String, Sim)> = jobs.into_iter().map(|j| j.join().unwrap()).collect();
    let mut failures = vec![];
    let (mut crossing_hits, mut rear_ends, mut withdrawn) = (0, 0, 0);
    for (name, sim) in &runs {
        let (st, inc) = (stats(sim), first_incident(sim));
        failures.extend(wrong(name, sim));
        assert_eq!(st.stalls, 1, "{name}");
        crossing_hits += st.crossing_impacts;
        withdrawn += st.withdrawn;
        if inc.fault == Fault::Dead {
            rear_ends += u64::from(st.lane_impacts > 0);
        } else if st.lane_impacts > 0 {
            failures.push(format!("{name}: struck from behind"));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
    assert!(crossing_hits <= 1, "{crossing_hits} crossing contacts in {} failures", runs.len());
    assert!(rear_ends >= 6, "a dead stop was struck from behind in only {rear_ends} of 12 failures");
    assert!(withdrawn as usize > 5 * runs.len(), "only {withdrawn} crossings withdrawn");
}

#[test]
fn the_crossing_serves_what_a_wreck_leaves_clear_and_recovers() {
    // The same arrivals with and without one failure, stepped together.
    let run = |seed: u64| {
        common::spawn(move || {
            let make = || crossing(12_000.0, GapPolicy::FailSafe, Scheduling::Batches, 700 + seed);
            let (mut with, mut without) = (make(), make());
            with.run_for(300.0);
            without.run_for(300.0);
            let lanes = box_lanes(&with);
            let lane = lanes[(seed as usize * 3) % lanes.len()];
            let (l_in, l_box) = with.coop.as_ref().unwrap().box_span(lane).unwrap();
            assert!(with.stall_at(lane, 0.5 * (l_in + l_box), Fault::Dead));
            let people = |s: &Sim| {
                let (on_map, waiting) = s.in_system();
                (on_map + waiting) as f64
            };
            let (mut t_fail, mut t_clear) = (None, None);
            let (mut during, mut worst, mut calm, mut back) = ((0, 0), 0.0_f64, 0.0, None);
            let mut done = (0, 0);
            for _ in 0..9000 {
                with.step();
                without.step();
                let c = with.coop.as_ref().unwrap();
                if t_fail.is_none() && c.stats.stalls > 0 {
                    t_fail = Some(with.t);
                    done = (with.stats.done.len(), without.stats.done.len());
                }
                if t_fail.is_some() && t_clear.is_none() && c.wrecks.is_empty() {
                    t_clear = Some(with.t);
                    during = (with.stats.done.len() - done.0, without.stats.done.len() - done.1);
                }
                let extra = people(&with) - people(&without);
                worst = worst.max(extra);
                if t_clear.is_some() && back.is_none() {
                    calm = if extra <= 10.0 { calm + with.dt } else { 0.0 };
                    if calm >= 20.0 {
                        back = Some(with.t);
                    }
                }
            }
            let mut f = wrong(&format!("seed {seed}"), &with);
            let (t_fail, t_clear) = (t_fail.expect("a failure"), t_clear.expect("the wreck cleared"));
            // Cleared a minute after the last vehicle it stopped dead or struck.
            if !(59.0..75.0).contains(&(t_clear - t_fail)) {
                f.push(format!("seed {seed}: cleared {:.0} s after the failure", t_clear - t_fail));
            }
            let share = during.0 as f64 / during.1 as f64;
            if share < 0.6 {
                f.push(format!(
                    "seed {seed}: served {:.0}% of the usual while the wreck stood",
                    share * 100.0
                ));
            }
            if worst < 10.0 {
                f.push(format!("seed {seed}: the failure held up only {worst:.0} vehicles"));
            }
            match back {
                Some(t) if t - t_clear < 600.0 => {}
                _ => {
                    f.push(format!("seed {seed}: not back to normal 10 minutes after the wreck was cleared"))
                }
            }
            let st = stats(&with);
            if st.crossing_impacts > 0 || st.stop_margin_min < 0.0 {
                f.push(format!("seed {seed}: crossing traffic did not stop short"));
            }
            f
        })
    };
    let jobs: Vec<_> = (0..6).map(run).collect();
    let failures: Vec<String> = jobs.into_iter().flat_map(|j| j.join().unwrap()).collect();
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn a_failure_outside_the_box_only_holds_up_its_lane() {
    // On the approach, past the box and on a slip lane no path crosses the failed
    // vehicle: nobody brakes hard for it but the vehicles behind it.
    let net = build_cooperative(2);
    let through = net.movements.iter().find(|m| m.turn == Turn::T).unwrap().routes[0].0;
    let slip = net.movements.iter().find(|m| m.turn == Turn::R).unwrap().routes[0].0;
    let jobs: Vec<_> =
        [(through, 120.0, Place::Approach), (through, f64::NAN, Place::Exit), (slip, 150.0, Place::Approach)]
            .into_iter()
            .map(|(lane, x, place)| {
                common::spawn(move || {
                    let mut sim = crossing(12_000.0, GapPolicy::FailSafe, Scheduling::Batches, 61);
                    sim.run_for(120.0);
                    let x = if x.is_nan() {
                        sim.coop.as_ref().unwrap().box_span(lane).unwrap().1 + 60.0
                    } else {
                        x
                    };
                    assert!(sim.stall_at(lane, x, Fault::Braking));
                    sim.run_for(40.0);
                    let inc = first_incident(&sim);
                    let mut f = vec![];
                    if (inc.lane, inc.place) != (lane, place) || !(x..x + 2.0).contains(&inc.x) {
                        f.push(format!(
                            "failed {:?} on lane {} at {:.1} m, not {place:?} at {x:.1} m",
                            inc.place, inc.lane, inc.x
                        ));
                    }
                    // The lane backs up behind it, and the other lanes keep going.
                    let behind = sim
                        .vehicles
                        .iter()
                        .filter(|v| v.route[0] == lane && !v.disabled && v.v < 0.1)
                        .count();
                    let moving = sim.vehicles.iter().filter(|v| v.route[0] != lane && v.v > 5.0).count();
                    if behind == 0 || moving < 20 {
                        f.push(format!("{place:?}: {behind} stopped behind it, {moving} moving elsewhere"));
                    }
                    sim.run_for(300.0);
                    let st = stats(&sim);
                    if st.crossing_impacts + st.lane_impacts > 0 || st.zone_violations > 0 {
                        f.push(format!("{place:?}: contacts or zones entered while occupied"));
                    }
                    if st.cleared != 1 {
                        f.push(format!("{place:?}: {} wrecks cleared away", st.cleared));
                    }
                    f.extend(wrong(&format!("{place:?} on lane {lane}"), &sim));
                    f
                })
            })
            .collect();
    let failures: Vec<String> = jobs.into_iter().flat_map(|j| j.join().unwrap()).collect();
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn a_failure_can_be_repeated_at_the_same_place_on_another_run() {
    // What the app's comparison does: the vehicle that fails on side A, and on side B
    // the next vehicle to reach the same place.
    let mut a = crossing(14_000.0, GapPolicy::FailSafe, Scheduling::Batches, 9);
    let mut b = crossing(14_000.0, GapPolicy::Nominal, Scheduling::FirstCome, 9);
    a.run_for(150.0);
    b.run_for(150.0);
    let (lane, x) = a.stall_in_box(Fault::Dead).expect("a vehicle on its way into the box");
    assert!(b.stall_at(lane, x, Fault::Dead));
    assert!(!b.stall_at(lane, x + 1.0, Fault::Dead), "one place per lane at a time");
    a.run_for(60.0);
    b.run_for(60.0);
    let (ia, ib) = (first_incident(&a), first_incident(&b));
    assert_eq!((ia.lane, ib.lane), (lane, lane));
    assert_eq!((ia.place, ib.place), (Place::Box, Place::Box));
    // Each stopped within a step's travel of the place.
    assert!(
        (ia.x - x).abs() < 1.5 && (ib.x - x).abs() < 1.5,
        "at {:.1} and {:.1} m, not {x:.1} m",
        ia.x,
        ib.x
    );
    assert_eq!((ia.policy, ib.policy), (GapPolicy::FailSafe, GapPolicy::Nominal));

    // A vehicle chosen by hand fails where it is, and only once.
    let mut sim = crossing(10_000.0, GapPolicy::FailSafe, Scheduling::Batches, 3);
    sim.run_for(100.0);
    let (id, kind) = sim.vehicles.iter().map(|v| (v.id, v.kind)).max_by_key(|v| v.0).unwrap();
    let place = sim.place_of(id).unwrap();
    assert!(sim.stall_vehicle(id, Fault::Dead));
    assert!(!sim.stall_vehicle(u64::MAX, Fault::Dead), "no such vehicle");
    sim.run_for(1.0);
    assert!(!sim.stall_vehicle(id, Fault::Dead), "already disabled");
    let inc = first_incident(&sim);
    assert_eq!((inc.vehicle, inc.kind, inc.lane, inc.place), (id, kind, place.0, Place::Approach));
    let v = sim.vehicles.iter().find(|v| v.id == id).unwrap();
    assert!(v.disabled && v.v == 0.0 && v.reason == Reason::Disabled);
    assert_eq!(sim.place_of(id).unwrap().1, place.1, "a dead stop is where it was");
    // Cleared away at once on request.
    sim.clear_wrecks();
    sim.run_for(0.2);
    assert!(sim.vehicles.iter().all(|v| v.id != id) && stats(&sim).cleared == 1);
}

#[test]
fn random_failures_are_the_same_on_runs_with_the_same_seed() {
    // Both sides of a comparison get failures at the same times and places, whatever
    // their gap policies and schedules; and a run repeats to the last bit.
    let run = |policy: GapPolicy, sched: Scheduling| {
        std::thread::spawn(move || {
            let mut sim = with_failures((demand_for(sched), policy, sched, 17), (60.0, Fault::Dead, 30.0));
            sim.run_for(900.0);
            let c = sim.coop.as_ref().unwrap();
            let places: Vec<(LinkId, i64)> =
                c.incidents.iter().map(|i| (i.lane, i.x.round() as i64)).collect();
            let delay: f64 = sim.stats.done.iter().map(|d| d.delay).sum();
            let pos: f64 = sim.vehicles.iter().map(|v| v.s + v.v).sum();
            let st = &c.stats;
            let print = (
                st.plans,
                st.stalls,
                st.emergency_stops,
                st.withdrawn,
                st.cleared,
                delay.to_bits(),
                pos.to_bits(),
            );
            (places, print, wrong(&format!("{} / {}", policy.short(), sched.label()), &sim), st.stalls)
        })
    };
    let jobs = [
        run(GapPolicy::FailSafe, Scheduling::Batches),
        run(GapPolicy::FailSafe, Scheduling::Batches),
        run(GapPolicy::Nominal, Scheduling::Batches),
    ];
    let [a, again, other] = jobs.map(|j| j.join().unwrap());
    assert!(a.3 >= 8, "{} failures in 15 minutes at 60 an hour", a.3);
    assert_eq!(a.1, again.1, "the same run twice");
    // The places are set at the same times on both runs, and at each the next vehicle
    // to arrive fails: the first failure is the same, and so are most of the others
    // (a lane that one run's wreck blocks gets its failure later, or not at all).
    let same = |x: &(LinkId, i64), y: &(LinkId, i64)| x.0 == y.0 && (x.1 - y.1).abs() <= 2;
    assert!(same(&a.0[0], &other.0[0]), "first failures at {:?} and {:?}", a.0[0], other.0[0]);
    let shared = a.0.iter().filter(|x| other.0.iter().any(|y| same(x, y))).count();
    assert!(shared * 3 >= a.0.len() * 2, "{shared} of {} failures at the same places", a.0.len());
    let failures: Vec<String> = [a.2, again.2, other.2].concat();
    // With one failure a minute a wreck is usually still on the map at the end.
    let failures: Vec<&String> =
        failures.iter().filter(|f| !f.contains("left behind") && !f.contains("books")).collect();
    assert!(failures.is_empty(), "{failures:?}");
}

#[test]
fn many_failures_in_heavy_traffic_leave_nothing_broken() {
    // Twenty minutes at the design demand with a failure every 40 s or so, of both
    // kinds, under each policy and schedule: whatever touches, the books stay straight,
    // and once the failures stop the crossing empties its box.
    let jobs: Vec<_> = GapPolicy::ALL
        .into_iter()
        .flat_map(|p| Scheduling::ALL.map(|s| (p, s)))
        .flat_map(|(p, s)| Fault::ALL.map(|f| (p, s, f)))
        .map(|(policy, sched, fault)| {
            common::spawn(move || {
                let mut sim = with_failures((18_000.0, policy, sched, 23), (90.0, fault, 20.0));
                sim.run_for(1200.0);
                let failed = stats(&sim).stalls;
                settle(&mut sim);
                let name = format!("{} / {} / {}", policy.short(), sched.label(), fault.label());
                let mut f = wrong(&name, &sim);
                if failed < 15 {
                    f.push(format!("{name}: only {failed} failures"));
                }
                if sim.coop.as_ref().unwrap().alarm() {
                    f.push(format!("{name}: still in alarm two minutes after the last failure"));
                }
                if policy == GapPolicy::FailSafe {
                    let st = stats(&sim);
                    if st.crossing_impacts > 0 {
                        f.push(format!("{name}: {} crossing contacts", st.crossing_impacts));
                    }
                }
                f
            })
        })
        .collect();
    let failures: Vec<String> = jobs.into_iter().flat_map(|j| j.join().unwrap()).collect();
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn a_vehicle_stopped_in_the_box_waits_for_one_that_passes_it() {
    // A car turning left brakes to a stop near the end of its arc, and five vehicles,
    // from all four approaches, brake to emergency stops inside the box. Each says that
    // it is stopped and stays where it is until the wreck is cleared away, a minute
    // later. Four are then granted their ways out at once, one of them past the fifth,
    // which stands in a zone on its path with room to pass it slowly. The fifth has to
    // let that one by: its own way out comes seven seconds later.
    let sched = Scheduling::FirstCome;
    let mut sim = crossing(demand_for(sched), GapPolicy::Nominal, sched, 638);
    sim.run_for(340.0);
    let lane = sim.net.movements[sim.net.movement_by_name("S_L").unwrap()].routes[0].0;
    let (l_in, l_box) = sim.coop.as_ref().unwrap().box_span(lane).unwrap();
    assert!(sim.stall_at(lane, l_in + 0.7 * (l_box - l_in), Fault::Braking));
    sim.run_for(15.0);
    let inc = first_incident(&sim);
    assert_eq!((inc.lane, inc.place, inc.fault, inc.stops), (lane, Place::Box, Fault::Braking, 5));
    let stood = stranded(&sim);
    assert_eq!((stood.len(), stopped(&sim)), (5, 5));
    for &(id, lane, x) in &stood {
        let (l_in, l_box) = sim.coop.as_ref().unwrap().box_span(lane).unwrap();
        assert!((l_in..l_box).contains(&x), "#{id} stopped {:.1} m from its stop line", x - l_in);
    }
    let approaches: BTreeSet<usize> =
        sim.vehicles.iter().filter(|v| v.reason == Reason::Stranded).map(|v| v.leg).collect();
    assert_eq!(approaches.len(), 4, "they came from approaches {approaches:?}");
    assert_eq!(Reason::Stranded.text(), "stopped after an emergency, waiting for a way out");

    // Nobody moves while the wreck stands.
    sim.run_for(47.0);
    assert_eq!(stranded(&sim), stood);
    assert_eq!((stats(&sim).recovered, stopped(&sim), sim.coop.as_ref().unwrap().wrecks.len()), (0, 5, 1));

    // It is cleared away a minute after it failed, and in that step four get their ways out.
    assert!(run_until(&mut sim, 5.0, |s| s.coop.as_ref().unwrap().wrecks.is_empty()));
    let t_clear = sim.t;
    let cleared = first_incident(&sim).cleared.expect("the wreck cleared away");
    assert!((cleared - inc.t - 60.0).abs() < 0.2, "cleared {:.1} s after it failed", cleared - inc.t);
    let c = sim.coop.as_ref().unwrap();
    assert_eq!((c.stats.recovered, stopped(&sim)), (4, 1));
    let ids = || stood.iter().map(|s| s.0);
    let passing: Vec<u64> = ids().filter(|id| c.plans[id].passes).collect();
    let waiting: Vec<u64> = ids().filter(|id| c.plans[id].kind == PlanKind::EmergencyStop).collect();
    assert_eq!((passing.len(), waiting.len()), (1, 1), "{passing:?} pass, {waiting:?} wait");
    let (passer, waiter) = (passing[0], waiting[0]);
    let (from, place) = (sim.place_of(passer).unwrap().1, sim.place_of(waiter));

    // The one that passes does so at 20 km/h at most. It comes within a meter of the
    // other and keeps its body more than 40 cm from the other's (the manager allows for
    // half a meter, checked every half meter of the way). The other stays where it is
    // until its own way out is granted.
    let within = |sim: &Sim, room: f64| {
        let body = |id: u64| sim.body(sim.vehicles.iter().find(|v| v.id == id).unwrap());
        body(passer).iter().any(|a| body(waiter).iter().any(|b| obb_overlap(*a, *b, -0.5 * room)))
    };
    let (mut fastest, mut close) = (0.0_f64, false);
    while stats(&sim).recovered < 5 && sim.t < t_clear + 15.0 {
        sim.step();
        assert!(!within(&sim, 0.4), "t={:.1}: within 40 cm of each other", sim.t);
        close |= within(&sim, 1.0);
        assert_eq!(sim.place_of(waiter), place, "t={:.1}: the waiting vehicle moved", sim.t);
        fastest = fastest.max(sim.vehicles.iter().find(|v| v.id == passer).unwrap().v);
    }
    assert!(close, "the passing vehicle never came within a meter of the other");
    assert_eq!((stats(&sim).recovered, stopped(&sim)), (5, 0));
    let waited = sim.t - t_clear;
    assert!((5.0..9.0).contains(&waited), "it waited {waited:.1} s");
    assert!(fastest > 5.0 && fastest <= 20.0 / 3.6 + 1e-9, "it was passed at {:.1} km/h", fastest * 3.6);
    let gone_by = sim.place_of(passer).unwrap().1 - from;
    assert!(gone_by > 25.0, "the passing vehicle was only {gone_by:.1} m on");

    // The box empties without a contact, and nobody has to be towed.
    sim.run_for(20.0);
    let st = stats(&sim);
    assert_eq!((st.crossing_impacts + st.lane_impacts, st.zone_violations, st.towed), (0, 0, 0));
    let failures = wrong("the box after the wreck", &sim);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn a_vehicle_stopped_behind_a_failure_can_fail_in_turn() {
    // Two failures in one lane, as the app's "Stall one in the box" followed by "Stall a
    // vehicle" and a click on a stopped one make them. A car brakes to a stop in the box,
    // and the three vehicles behind it stop behind it without touching. Then the first of
    // the three fails where it stands. The two behind it are at rest already and stay so:
    // they are not counted as stopping again, and nobody else has to brake or loses a
    // crossing. Each wreck is cleared away a minute after its own failure, and the two are
    // routed out once the second is gone.
    let fault = Fault::Braking;
    let mut sim = crossing(12_000.0, GapPolicy::FailSafe, Scheduling::Batches, 6);
    sim.run_for(150.0);
    let (lane, x) = sim.stall_in_box(fault).expect("a vehicle on its way into the box");
    let (l_in, l_box) = sim.coop.as_ref().unwrap().box_span(lane).unwrap();
    assert!((l_in..l_box).contains(&x), "in the box, so it fails at once");
    sim.run_for(13.0);
    let first = first_incident(&sim);
    assert_eq!((first.lane, first.place, first.fault, first.stops), (lane, Place::Box, fault, 3));
    let wreck = sim.place_of(first.vehicle).unwrap().1;
    let behind = stranded(&sim);
    assert_eq!((behind.len(), stopped(&sim)), (3, 3));
    assert!(behind.iter().all(|s| s.1 == lane && (l_in..wreck).contains(&s.2)), "{behind:?}");
    let (stops, withdrawn) = (stats(&sim).emergency_stops, stats(&sim).withdrawn);
    assert_eq!((stops, stats(&sim).lane_impacts), (3, 0));

    let second = behind[0].0;
    assert!(sim.stall_vehicle(second, fault));
    sim.run_for(2.0);
    let c = sim.coop.as_ref().unwrap();
    let inc = c.incidents.back().unwrap();
    assert_eq!((c.stats.stalls, c.incidents.len(), c.wrecks.len()), (2, 2, 2));
    assert_eq!((inc.vehicle, inc.lane, inc.place, inc.fault), (second, lane, Place::Box, fault));
    assert_eq!((inc.stops, inc.withdrawn), (0, 0), "nobody else had to react");
    assert_eq!((c.stats.emergency_stops, c.stats.withdrawn), (stops, withdrawn));
    assert_eq!(stranded(&sim), behind[1..], "the two behind it stay where they are");
    assert_eq!(stopped(&sim), 2);

    // The first wreck goes a minute after the first failure; the second still blocks the lane.
    assert!(run_until(&mut sim, 50.0, |s| s.coop.as_ref().unwrap().wrecks.len() == 1));
    let c = sim.coop.as_ref().unwrap();
    assert!(c.wrecks.contains_key(&second));
    assert_eq!((c.stats.recovered, stranded(&sim)), (0, behind[1..].to_vec()));
    // When that has gone too, the two get their ways out.
    assert!(run_until(&mut sim, 20.0, |s| s.coop.as_ref().unwrap().wrecks.is_empty()));
    let c = sim.coop.as_ref().unwrap();
    assert_eq!((c.stats.cleared, c.stats.recovered, stopped(&sim)), (2, 2, 0));
    for inc in &c.incidents {
        let after = inc.cleared.expect("cleared away") - inc.t;
        assert!((after - 60.0).abs() < 0.2, "failure {} cleared after {after:.1} s", inc.id);
    }
    sim.run_for(30.0);
    assert_eq!(stats(&sim).towed, 0);
    let failures = wrong("two failures in one lane", &sim);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn a_vehicle_that_fails_at_the_edge_of_the_map_rolls_off_it() {
    // A vehicle that loses drive within five meters of the end of its way out is off the
    // map before it can stop: no wreck is left to clear away. The five vehicles behind it
    // that were past their stop line have left their crossings to follow it. Each slows
    // behind the place where it would have come to rest, to 10 km/h at the slowest, and
    // drives off the map without stopping.
    let mut sim = crossing(12_000.0, GapPolicy::FailSafe, Scheduling::Batches, 61);
    sim.run_for(120.0);
    let (lane, rest) = sim.net.movements.iter().find(|m| m.turn == Turn::T).unwrap().routes[0].clone();
    let end: f64 = std::iter::once(lane).chain(rest).map(|l| sim.net.links[l].length()).sum();
    assert!(sim.stall_at(lane, end - 5.0, Fault::Braking));
    assert!(run_until(&mut sim, 30.0, |s| stats(s).stalls == 1));
    let inc = first_incident(&sim);
    assert_eq!((inc.lane, inc.place, inc.fault), (lane, Place::Exit, Fault::Braking));
    assert!((end - 5.0..end).contains(&inc.x), "it failed {:.1} m before the end", end - inc.x);
    let c = sim.coop.as_ref().unwrap();
    let (l_in, _) = c.box_span(lane).unwrap();
    let following = |id: &u64| c.plans.get(id).is_some_and(|p| p.kind == PlanKind::StopBehind);
    let behind: Vec<u64> = sim.vehicles.iter().map(|v| v.id).filter(following).collect();
    assert_eq!((behind.len(), inc.stops, c.wrecks.len()), (5, 5, 1));
    assert!(behind.iter().all(|&id| sim.place_of(id).is_some_and(|p| p.0 == lane && p.1 > l_in)));

    // Within a second it has left the map, and the manager's books with it.
    assert!(run_until(&mut sim, 1.0, |s| s.place_of(inc.vehicle).is_none()), "it is still on the map");
    let c = sim.coop.as_ref().unwrap();
    assert!(c.wrecks.is_empty() && c.alarm(), "no wreck, but vehicles still off their crossings");
    // The vehicles behind it follow it off the map.
    let mut slowest = f64::INFINITY;
    for _ in 0..200 {
        sim.step();
        for v in sim.vehicles.iter().filter(|v| behind.contains(&v.id)) {
            assert!(v.reason != Reason::Stranded, "t={:.1}: #{} came to rest", sim.t, v.id);
            slowest = slowest.min(v.v);
        }
    }
    assert!((2.5..3.0).contains(&slowest), "the slowest of them did {:.1} km/h", slowest * 3.6);
    assert!(behind.iter().all(|&id| sim.place_of(id).is_none()), "one of them is still on the map");
    let c = sim.coop.as_ref().unwrap();
    assert!(!c.alarm(), "still in alarm 20 s after the vehicle left the map");
    assert_eq!((c.stats.cleared, c.stats.recovered, c.stats.lane_impacts), (0, 0, 0));
    assert_eq!(first_incident(&sim).cleared, None, "it was never cleared away");
    let failures = wrong("a failure at the edge of the map", &sim);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn the_log_keeps_the_latest_fifty_failures() {
    // Ten minutes with a failure every six seconds or so, five times what the app
    // offers. The log keeps the latest fifty failures, oldest first, while the counters
    // go on counting them all. Every emergency stop, withdrawn crossing and contact is
    // entered under one failure: summed over every failure the log has held, they are
    // the counters.
    let mut sim =
        with_failures((12_000.0, GapPolicy::FailSafe, Scheduling::Batches, 6), (600.0, Fault::Dead, 10.0));
    let mut held: BTreeMap<u64, Incident> = BTreeMap::new();
    for _ in 0..6000 {
        sim.step();
        held.extend(sim.coop.as_ref().unwrap().incidents.iter().map(|i| (i.id, i.clone())));
    }
    let c = sim.coop.as_ref().unwrap();
    let st = &c.stats;
    assert!(st.stalls >= 60, "{} failures in 10 minutes at 600 an hour", st.stalls);
    assert_eq!(held.len() as u64, st.stalls, "every failure was in the log at first");
    let ids: Vec<u64> = c.incidents.iter().map(|i| i.id).collect();
    assert_eq!(ids, (st.stalls - 50..st.stalls).collect::<Vec<_>>(), "the latest fifty, oldest first");
    assert!(c.incidents.iter().is_sorted_by(|a, b| a.t <= b.t));
    assert!(c.wrecks.values().all(|w| ids.contains(&w.incident)), "a wreck older than the log");
    let total = |of: fn(&Incident) -> u64| held.values().map(of).sum::<u64>();
    assert_eq!(total(|i| u64::from(i.stops)), st.emergency_stops);
    assert_eq!(total(|i| u64::from(i.withdrawn)), st.withdrawn);
    assert_eq!(total(|i| i.impacts.len() as u64), st.crossing_impacts + st.lane_impacts);
    // Each contact is entered under the failure that led to it: one of the two vehicles is
    // the one that failed, or has touched one of its wrecks before.
    for inc in held.values() {
        let mut wrecked = vec![inc.vehicle];
        for hit in &inc.impacts {
            assert!(
                wrecked.contains(&hit.struck) || wrecked.contains(&hit.striker),
                "failure {} holds the contact of #{} and #{}, which it did not wreck",
                inc.id,
                hit.striker,
                hit.struck
            );
            wrecked.extend([hit.struck, hit.striker]);
        }
    }
    // What is left in the log no longer adds up to them.
    let in_log: u64 = c.incidents.iter().map(|i| u64::from(i.stops)).sum();
    assert!(in_log < st.emergency_stops, "{in_log} of {} emergency stops", st.emergency_stops);

    // Once the failures stop, the last wrecks are cleared away and nothing is left behind.
    settle(&mut sim);
    let failures = wrong("after ten minutes of failures", &sim);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn a_failure_holds_vehicles_back_and_never_sets_one_going() {
    // A failure makes vehicles stop: behind the failed vehicle, short of a zone it holds,
    // or short of the box. It never makes one start. A vehicle that stands still once
    // its reaction time is over stays where it is for as long as it is off its crossing,
    // whatever fails ahead of it later: one stopped in the box, and one waiting in the
    // box for its way out to begin. One waiting before its line for a crossing granted
    // for later has that crossing withdrawn instead, and stays short of the box. In these
    // three runs such vehicles were once started up behind a vehicle that had failed,
    // and driven into crossing traffic.
    //
    // Nor does a failure make a vehicle that can no longer stop short of the box get
    // ahead of the trajectory it was granted: the zones ahead of it are its own only
    // from the times that trajectory reaches them.
    let runs = [
        ((12_000.0, GapPolicy::Mitigated, Scheduling::FirstCome, 118), (240.0, Fault::Dead, 30.0), 720.0),
        ((8_000.0, GapPolicy::Nominal, Scheduling::Batches, 127), (600.0, Fault::Braking, 20.0), 400.0),
        ((16_000.0, GapPolicy::FailSafe, Scheduling::FirstCome, 1), (900.0, Fault::Dead, 20.0), 400.0),
    ];
    let jobs = runs.map(|(crossing, set, seconds)| {
        common::spawn(move || {
            let name = format!("{} / {} / seed {}", crossing.1.short(), crossing.2.label(), crossing.3);
            let mut sim = with_failures(crossing, set);
            // Where each vehicle stands that has come to rest off its crossing, and the
            // crossing each vehicle was last granted: when, and where it puts the vehicle
            // step by step.
            let mut standing: BTreeMap<u64, f64> = BTreeMap::new();
            let mut granted: BTreeMap<u64, (f64, Vec<f64>)> = BTreeMap::new();
            // Each vehicle's trajectory a step ago: when it began, and what it was for.
            let mut kinds: BTreeMap<u64, (f64, PlanKind)> = BTreeMap::new();
            let (mut stood, mut held_back, mut moved) = (0, 0, vec![]);
            for _ in 0..(seconds / sim.dt).round() as usize {
                sim.step();
                let coop = sim.coop.as_ref().unwrap();
                let plans = &coop.plans;
                let reacting = |id: &u64| {
                    plans.get(id).filter(|p| matches!(p.kind, PlanKind::EmergencyStop | PlanKind::StopBehind))
                };
                for v in sim.vehicles.iter().filter(|v| !v.disabled) {
                    let (Some(p), Some((_, now))) = (plans.get(&v.id), sim.place_of(v.id)) else { continue };
                    // One that stands before its line when it reacts, with a crossing for
                    // later, is not on an emergency stop: its crossing is withdrawn.
                    let had_crossing = kinds
                        .insert(v.id, (p.t0, p.kind))
                        .is_some_and(|was| was.0 != p.t0 && was.1 == PlanKind::Crossing);
                    let (n, l_in) = ((REACTION / sim.dt).round() as usize, coop.box_span(p.lane).unwrap().0);
                    if had_crossing && reacting(&v.id).is_some() && p.v[n] < 1e-9 && p.x[n] < l_in {
                        moved.push(format!("{name}: t={:.1} #{} not withdrawn at its line", sim.t, v.id));
                    }
                    if p.kind == PlanKind::Crossing {
                        if granted.get(&v.id).is_none_or(|g| g.0 != p.t0) {
                            granted.insert(v.id, (p.t0, p.x.clone()));
                        }
                    } else if reacting(&v.id).is_some()
                        && let Some((t0, xs)) = granted.get(&v.id)
                        && let Some(was) = xs.get(((sim.t - t0) / sim.dt).round() as usize)
                    {
                        held_back += usize::from(now < was - 1.0);
                        if now > was + 1e-6 {
                            moved.push(format!(
                                "{name}: t={:.1} #{} is {:.2} m ahead",
                                sim.t,
                                v.id,
                                now - was
                            ));
                        }
                    }
                }
                standing.retain(|id, x| match (reacting(id), sim.place_of(*id)) {
                    (Some(_), Some((_, now))) if (now - *x).abs() > 1e-9 => {
                        moved.push(format!("{name}: t={:.1} #{id} set going ({:.3} m)", sim.t, now - *x));
                        false
                    }
                    // Granted a way out, struck, or cleared away.
                    (on_plan, place) => on_plan.is_some() && place.is_some(),
                });
                for v in sim.vehicles.iter().filter(|v| v.v < 1e-9 && !v.disabled) {
                    if reacting(&v.id).is_some_and(|p| sim.t >= p.t0 + REACTION - 1e-9)
                        && standing.insert(v.id, sim.place_of(v.id).unwrap().1).is_none()
                    {
                        stood += 1;
                    }
                }
            }
            let (st, policy) = (stats(&sim).clone(), crossing.1);
            if stood < 25 || held_back < 1000 {
                moved.push(format!(
                    "{name}: only {stood} came to rest off their crossings, held back for {held_back} steps"
                ));
            }
            if policy == GapPolicy::FailSafe && st.crossing_impacts > 0 {
                moved.push(format!("{name}: {} crossing contacts", st.crossing_impacts));
            }
            settle(&mut sim);
            moved.extend(wrong(&name, &sim));
            moved
        })
    });
    let failures: Vec<String> = jobs.into_iter().flat_map(|j| j.join().unwrap()).collect();
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn a_vehicle_that_cannot_stop_behind_a_dead_stop_is_expected_where_it_ends_up() {
    // A car stops dead in the box with another four meters behind it at 27 km/h, which
    // cannot stop in time. The manager knows that at once: it expects the second car
    // against the first 0.7 s later, and from the failure on it holds every zone the
    // second car will not have left by then, since it will stand in one of them.
    // Crossing traffic is warned that much sooner than at the contact. The trajectory
    // itself goes on braking to rest, which is what the car would do were the wreck
    // cleared away in the meantime.
    let mut sim = crossing(16_000.0, GapPolicy::Nominal, Scheduling::Batches, 500);
    sim.run_for(180.0);
    let lane = box_lanes(&sim)[0];
    let (l_in, l_box) = sim.coop.as_ref().unwrap().box_span(lane).unwrap();
    assert!(sim.stall_at(lane, l_in + 0.3 * (l_box - l_in), Fault::Dead));
    assert!(run_until(&mut sim, 60.0, |s| stats(s).stalls == 1));
    let inc = first_incident(&sim);
    let failed = sim.vehicles.iter().find(|v| v.id == inc.vehicle).unwrap();
    let rear = inc.x - failed.len();
    let (behind, gap, speed) = sim
        .vehicles
        .iter()
        .filter(|v| !v.disabled)
        .filter_map(|v| {
            sim.place_of(v.id).filter(|p| p.0 == lane && p.1 < rear).map(|p| (v.id, rear - p.1, v.v))
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .expect("a vehicle behind it");
    assert!((3.0..5.0).contains(&gap) && speed > 7.0, "{gap:.1} m behind it at {:.0} km/h", speed * 3.6);
    let plan = &sim.coop.as_ref().unwrap().plans[&behind];
    assert_eq!((plan.kind, plan.rests), (PlanKind::StopBehind, true));
    // It is expected at the failed car's rear, in the step that takes it that far.
    let (step, at) = plan.contact.expect("a contact expected");
    let (past, when) = (plan.x[step] - rear, plan.t0 + step as f64 * sim.dt);
    assert!((at - rear).abs() < 1e-9, "it is expected {:.2} m from the rear", rear - at);
    assert!((0.0..speed * sim.dt).contains(&past), "its trajectory is {past:.2} m past the rear by then");
    assert!((0.5..1.0).contains(&(when - inc.t)), "and {:.1} s after the failure", when - inc.t);
    // The zones it is in by then are held without end, unless it has left them by then;
    // the trajectory brakes on to rest, a few meters farther.
    let entered: Vec<_> = plan.zones.iter().filter(|z| z.t_in <= when).collect();
    assert!(entered.iter().all(|z| z.t_out <= when || z.t_out.is_infinite()));
    let held = entered.iter().filter(|z| z.t_out.is_infinite()).count();
    assert!(held >= 1, "it holds {held} of the {} zones it is in by then", entered.len());
    let (on, rest) = (plan.last_x() - plan.x[step], *plan.v.last().unwrap());
    assert!((2.0..6.0).contains(&on) && rest == 0.0, "it brakes on for {on:.1} m, to {rest:.1} m/s");
    // One of the zones held is one it would leave if it did brake on that far, and
    // every zone it counts as leaving lies behind where it will stand.
    let length = sim.vehicles.iter().find(|v| v.id == behind).unwrap().len();
    let left = entered.iter().filter(|z| z.t_out.is_infinite() && z.x1 + length <= plan.last_x()).count();
    assert!(left >= 1, "braking on, it would leave {left} of the zones held");
    assert!(plan.zones.iter().filter(|z| z.t_out.is_finite()).all(|z| z.x1 + length <= at));
    // The vehicle behind that one is told the same: its trajectory stops short of where
    // the second car will stand, not of where it would stop braking on.
    let from = sim.place_of(behind).unwrap().1;
    let (next, _) = sim
        .vehicles
        .iter()
        .filter_map(|v| sim.place_of(v.id).filter(|p| p.0 == lane && p.1 < from).map(|p| (v.id, p.1)))
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .expect("a third vehicle");
    let its = &sim.coop.as_ref().unwrap().plans[&next];
    assert_eq!((its.kind, its.rests, its.contact), (PlanKind::StopBehind, true, None));
    let room = at - length - its.last_x();
    assert!((0.5..3.0).contains(&room), "the third stops {room:.2} m behind where the second will stand");

    // They touch there and then.
    assert!(run_until(&mut sim, 2.0, |s| stats(s).lane_impacts == 1));
    let hit = first_incident(&sim).impacts[0];
    assert_eq!((hit.striker, hit.struck, hit.crossing), (behind, inc.vehicle, false));
    assert!((hit.t - when).abs() < 1e-6, "they touched at {:.1}, expected at {when:.1}", hit.t);
    assert!(hit.speed > 5.0, "at {:.0} km/h", hit.speed * 3.6);
    let stands = sim.place_of(behind).unwrap().1;
    assert!((stands - rear).abs() < 1e-6, "it stands {:.2} m from the rear", rear - stands);
    // Nothing else touches, and the box is cleared.
    sim.run_for(240.0);
    let st = stats(&sim);
    assert_eq!((st.crossing_impacts, st.lane_impacts, st.cleared), (0, 1, 2));
    let failures = wrong("a dead stop with a vehicle close behind", &sim);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn a_vehicle_expected_against_a_wreck_stops_by_itself_if_the_wreck_is_cleared_first() {
    // The same dead stop, but the wreck is cleared away at once, before the car behind
    // reaches it. That car was expected to come to rest against it. With nothing there
    // to stop it, it brakes on to rest by itself, on the trajectory it was given, and is
    // routed out of the box like any vehicle that stopped there.
    let mut sim = crossing(16_000.0, GapPolicy::Nominal, Scheduling::Batches, 500);
    sim.run_for(180.0);
    let lane = box_lanes(&sim)[0];
    let (l_in, l_box) = sim.coop.as_ref().unwrap().box_span(lane).unwrap();
    assert!(sim.stall_at(lane, l_in + 0.3 * (l_box - l_in), Fault::Dead));
    assert!(run_until(&mut sim, 60.0, |s| stats(s).stalls == 1));
    let wreck = first_incident(&sim).vehicle;
    let c = sim.coop.as_ref().unwrap();
    let (behind, plan) = c.plans.iter().find(|(_, p)| p.contact.is_some()).expect("a contact expected");
    let (behind, at, rest) = (*behind, plan.contact.unwrap().1, plan.last_x());
    assert!(rest - at > 2.0, "it would brake on for {:.1} m", rest - at);
    sim.clear_wrecks();
    sim.step();
    assert!(sim.place_of(wreck).is_none(), "the wreck is still on the map");
    // It comes to rest where its trajectory ends, a few meters past where the wreck stood.
    assert!(run_until(&mut sim, 3.0, |s| s.vehicles.iter().any(|v| v.id == behind && v.v == 0.0)));
    let stands = sim.place_of(behind).unwrap().1;
    assert!((stands - rest).abs() < 1e-6, "it stands {:.2} m from the end of its trajectory", rest - stands);
    let st = stats(&sim);
    assert_eq!((st.lane_impacts + st.crossing_impacts, st.deviations, st.cleared), (0, 0, 1));
    // Its way is clear: it is routed out, and nothing is left behind.
    sim.run_for(60.0);
    assert!(stats(&sim).recovered >= 1 && sim.place_of(behind).is_none());
    let failures = wrong("a wreck cleared away before it is reached", &sim);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn a_vehicle_that_fails_close_behind_a_dead_stop_gets_no_farther_than_it() {
    // The same dead stop, and the car behind it loses drive a step later. Braking as hard
    // as it can, it still reaches the first car: it is expected against it from the step
    // it fails, not a braking distance on.
    let mut sim = crossing(16_000.0, GapPolicy::Nominal, Scheduling::Batches, 500);
    sim.run_for(180.0);
    let lane = box_lanes(&sim)[0];
    let (l_in, l_box) = sim.coop.as_ref().unwrap().box_span(lane).unwrap();
    assert!(sim.stall_at(lane, l_in + 0.3 * (l_box - l_in), Fault::Dead));
    assert!(run_until(&mut sim, 60.0, |s| stats(s).stalls == 1));
    let inc = first_incident(&sim);
    let rear = inc.x - sim.vehicles.iter().find(|v| v.id == inc.vehicle).unwrap().len();
    let (behind, gap, speed) = sim
        .vehicles
        .iter()
        .filter(|v| !v.disabled)
        .filter_map(|v| {
            sim.place_of(v.id).filter(|p| p.0 == lane && p.1 < rear).map(|p| (v.id, rear - p.1, v.v))
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .expect("a vehicle behind it");
    // Too close to stop: from this speed it needs more room than it has.
    assert!(gap < speed * speed / 12.0, "{gap:.1} m behind it at {:.0} km/h", speed * 3.6);
    assert!(sim.stall_vehicle(behind, Fault::Braking));
    sim.step();
    let plan = &sim.coop.as_ref().unwrap().plans[&behind];
    assert_eq!((plan.kind, stats(&sim).stalls), (PlanKind::Fault, 2));
    let (step, at) = plan.contact.expect("a contact expected");
    assert!((at - rear).abs() < 1e-9, "it is expected {:.2} m from the rear", rear - at);
    assert!(plan.last_x() > rear + 0.3, "braking on, it would stop {:.2} m on", plan.last_x() - rear);
    let when = plan.t0 + step as f64 * sim.dt;
    // They touch there and then, and it stands against the first car.
    assert!(run_until(&mut sim, 2.0, |s| stats(s).lane_impacts == 1));
    let hit = first_incident(&sim).impacts[0];
    assert_eq!((hit.striker, hit.struck, hit.crossing), (behind, inc.vehicle, false));
    assert!((hit.t - when).abs() < 1e-6, "they touched at {:.1}, expected at {when:.1}", hit.t);
    let stands = sim.place_of(behind).unwrap().1;
    assert!((stands - rear).abs() < 1e-6, "it stands {:.2} m from the rear", rear - stands);
    sim.run_for(240.0);
    let failures = wrong("two failures, one behind the other", &sim);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn a_vehicle_on_an_emergency_stop_is_expected_against_one_that_then_stops_short() {
    // A vehicle already braking as hard as it can keeps its trajectory whatever the
    // vehicle ahead of it does. When that one then stops short, because it is struck or
    // fails in turn, the first cannot stop behind it any more. The manager says so in
    // that step: the emergency stop is expected to end against the vehicle ahead, and
    // the zones it will stand in are held from then on. A failure every six seconds
    // makes it happen within five minutes.
    let mut sim =
        with_failures((12_000.0, GapPolicy::Nominal, Scheduling::Batches, 8), (600.0, Fault::Dead, 15.0));
    // Emergency stops under way with no contact expected, each with when it began.
    let mut braking: BTreeMap<u64, f64> = BTreeMap::new();
    let mut found = None;
    while found.is_none() && sim.t < 300.0 {
        sim.step();
        let plans = &sim.coop.as_ref().unwrap().plans;
        for (id, p) in plans.iter().filter(|(_, p)| p.kind == PlanKind::EmergencyStop) {
            match p.contact {
                Some(contact) if braking.get(id) == Some(&p.t0) => found = Some((*id, p.t0, contact)),
                Some(_) => {}
                None => _ = braking.insert(*id, p.t0),
            }
        }
    }
    let (id, since, (step, at)) = found.expect("an emergency stop that gained an expected contact");
    let when = since + step as f64 * sim.dt;
    assert!(sim.t > since + 0.2, "it had been braking for {:.1} s", sim.t - since);
    assert!(when > sim.t - sim.dt, "the contact is expected {:.1} s from now", when - sim.t);
    // The vehicle ahead is disabled, and its rear is where the contact is expected.
    let (lane, x) = sim.place_of(id).unwrap();
    let (ahead, disabled, rear) = sim
        .vehicles
        .iter()
        .filter_map(|v| sim.place_of(v.id).filter(|p| p.0 == lane && p.1 > x).map(|p| (v, p.1)))
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(v, x)| (v.id, v.disabled, x - v.len()))
        .expect("a vehicle ahead of it");
    assert!(disabled && (rear - at).abs() < 1e-6, "its rear is {:.2} m from there", rear - at);
    let plan = &sim.coop.as_ref().unwrap().plans[&id];
    assert!(plan.zones.iter().any(|z| z.t_out.is_infinite()), "it holds no zone without end");
    // They touch then and there.
    let struck = |s: &Sim| {
        let log = &s.coop.as_ref().unwrap().incidents;
        log.iter().flat_map(|i| &i.impacts).find(|i| i.striker == id).copied()
    };
    assert!(run_until(&mut sim, 3.0, |s| struck(s).is_some()), "it never reached the vehicle ahead");
    let hit = struck(&sim).unwrap();
    assert_eq!((hit.struck, hit.crossing), (ahead, false));
    assert!((hit.t - when).abs() < 1e-6, "they touched at {:.1}, expected at {when:.1}", hit.t);
    assert!((sim.place_of(id).unwrap().1 - at).abs() < 1e-6);
    // Once the failures stop, nothing is left behind.
    settle(&mut sim);
    let failures = wrong("an emergency stop that ends against the vehicle ahead", &sim);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn fail_safe_gaps_keep_crossing_traffic_apart_through_many_failures() {
    // With the fail-safe gap a vehicle on a crossing path can always stop short of one
    // that stops dead, and that holds for what a failure sets off as well: the vehicle
    // behind it that cannot stop in time, and the ones that stop in the box and later
    // drive out of it. About two hundred dead stops, most far more often than the app
    // offers, and not one contact between vehicles on crossing paths.
    //
    // (In the first two of these runs a crossing vehicle once ran into one that had
    // struck the vehicle ahead of it: in the first run the vehicle right behind the
    // failed one, in the second the one behind that.)
    let runs = [
        ((12_000.0, Scheduling::FirstCome, 3), (300.0, 10.0), 1100.0),
        ((12_000.0, Scheduling::FirstCome, 6), (120.0, 10.0), 700.0),
        ((16_000.0, Scheduling::FirstCome, 1), (900.0, 20.0), 300.0),
        ((18_000.0, Scheduling::Batches, 7), (600.0, 15.0), 600.0),
    ];
    let jobs = runs.map(|((total, sched, seed), (rate, clear), seconds)| {
        common::spawn(move || {
            let name = format!("{} / seed {seed}", sched.label());
            let mut sim =
                with_failures((total, GapPolicy::FailSafe, sched, seed), (rate, Fault::Dead, clear));
            sim.run_for(seconds);
            let st = stats(&sim).clone();
            let mut f = vec![];
            if st.crossing_impacts > 0 {
                f.push(format!("{name}: {} crossing contacts", st.crossing_impacts));
            }
            // What a dead stop does do is stop the vehicle close behind it.
            if st.stalls < 15 || st.lane_impacts < 10 || st.emergency_stops < 30 {
                f.push(format!(
                    "{name}: {} failures, {} struck from behind, {} emergency stops",
                    st.stalls, st.lane_impacts, st.emergency_stops
                ));
            }
            settle(&mut sim);
            f.extend(wrong(&name, &sim));
            f
        })
    });
    let failures: Vec<String> = jobs.into_iter().flat_map(|j| j.join().unwrap()).collect();
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn only_the_cooperative_crossing_has_failures() {
    for kind in LayoutKind::ALL {
        if kind == LayoutKind::Cooperative {
            continue;
        }
        let mut sim = Sim::new(kind, ControlMode::options(kind)[0], CtrlKind::Fixed, 1);
        sim.run_for(60.0);
        let id = sim.vehicles[0].id;
        assert!(!sim.stall_vehicle(id, Fault::Dead), "{kind:?}");
        assert!(!sim.stall_at(sim.vehicles[0].route[0], 10.0, Fault::Dead), "{kind:?}");
        assert!(sim.stall_in_box(Fault::Dead).is_none() && !sim.stall_at_worst(Fault::Dead), "{kind:?}");
        sim.clear_wrecks();
        sim.run_for(10.0);
        assert!(sim.vehicles.iter().all(|v| !v.disabled), "{kind:?}");
    }
    // The two failures, by name.
    assert_eq!(Fault::ALL.map(|f| f.name()), ["Stops dead", "Brakes hard to a stop"]);
    assert_eq!(Fault::ALL.map(|f| f.label()), ["stopped dead", "braked to a stop"]);
    assert_eq!(
        [Place::Approach, Place::Box, Place::Exit].map(|p| p.label()),
        ["on the approach", "in the box", "past the box"]
    );
    let _ = VKind::Car;
}
