// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Gap acceptance against the Highway Capacity Manual (HCM), measured the way a field
//! study measures it (see `sim::gaps`).

use intersection_lab::control::CtrlKind;
use intersection_lab::sim::demand::DemandConfig;
use intersection_lab::sim::gaps::{GapStudy, critical_headway};
use intersection_lab::sim::network::{LayoutKind, Network, Turn};
use intersection_lab::sim::{ControlMode, Sim};

mod common;

/// A car turning left from the major road at a two-way stop accepts about the gaps the
/// HCM's critical headway for the turn, 4.1 s, describes: the model's critical headway,
/// fitted by maximum likelihood to the gaps its drivers let pass and the ones they went
/// in, is within 2 s of it. Before gap acceptance was calibrated it was 7.6 s on the
/// T-intersection and 8.5 s on major/minor. The rest of the difference comes mostly from
/// the time a car takes to cross the near oncoming lane from a standstill.
#[test]
fn major_road_left_turns_accept_gaps_near_the_hcm() {
    const HCM: f64 = 4.1;
    const TOLERANCE: f64 = 2.0;
    let runs = [
        (LayoutKind::Tee, 1),
        (LayoutKind::Tee, 2),
        (LayoutKind::MajorMinor4, 1),
        (LayoutKind::MajorMinor4, 2),
    ];
    let handles: Vec<_> = runs
        .into_iter()
        .map(|(kind, seed)| {
            common::spawn(move || {
                let net = Network::build(kind);
                let demand = DemandConfig::default_for(&net);
                let mut sim = Sim::with_demand(net, ControlMode::TwoWayStop, CtrlKind::Fixed, demand, seed);
                // Nobody crossing, so no driver waits for a pedestrian instead of a gap.
                sim.demand.ped_rate = 0.0;
                let mut study = GapStudy::new();
                while sim.t < 1800.0 {
                    sim.step();
                    study.observe(&sim);
                }
                let net = &sim.net;
                (study.choices.into_iter())
                    .filter(|c| {
                        let m = &net.movements[c.movement];
                        !net.legs[m.leg].minor && m.turn == Turn::L && !c.heavy()
                    })
                    .collect::<Vec<_>>()
            })
        })
        .collect();
    let choices: Vec<_> = handles.into_iter().flat_map(|h| h.join().unwrap()).collect();
    let tc = critical_headway(&choices).expect("too few drivers to fit a critical headway");
    assert!(
        (tc - HCM).abs() <= TOLERANCE,
        "critical headway {tc:.2} s from {} drivers, against the HCM's {HCM} s",
        choices.len()
    );
}
