// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Safety sweep: every layout x control x seed, report in-box overlaps.
use intersection_lab::control::CtrlKind;
use intersection_lab::sim::network::LayoutKind;
use intersection_lab::sim::{ControlMode, Sim};
fn main() {
    let seeds: u64 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(4);
    let scale: f64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(1.0);
    // Optional comma-separated options: "peds" turns on the pedestrian treatments (LPI 4 s +
    // no turn on red with peds, 4x pedestrians); "prot" / "auto" set the left-turn mode.
    let opts = std::env::args().nth(3).unwrap_or_default();
    let peds = opts.split(',').any(|a| a == "peds");
    let left = if opts.split(',').any(|a| a == "prot") {
        intersection_lab::sim::LeftMode::Protected
    } else if opts.split(',').any(|a| a == "auto") {
        intersection_lab::sim::LeftMode::Auto
    } else {
        intersection_lab::sim::LeftMode::Fya
    };
    let mut jobs = vec![];
    for kind in LayoutKind::ALL {
        for mode in ControlMode::options(kind) {
            let ctrls: Vec<CtrlKind> = if mode == ControlMode::Signal { CtrlKind::ALL[..4].to_vec() } else { vec![CtrlKind::Fixed] };
            for c in ctrls {
                jobs.push(std::thread::spawn(move || {
                    let mut total = 0; let mut first = String::new();
                    for seed in 0..seeds {
                        let mut sim = Sim::new(kind, mode, c, 2000 + seed);
                        sim.check_safety = true;
                        sim.demand.scale = scale;
                        sim.left_mode = left;
                        if peds {
                            sim.signal.lpi = 4.0;
                            sim.ntor_peds = true;
                            sim.demand.ped_rate *= 4.0;
                        }
                        sim.run_for(1500.0);
                        total += sim.collisions;
                        if first.is_empty() { if let Some(l) = sim.collision_log.first() { first = l.clone(); } }
                    }
                    (format!("{:<12} {:<28} {:<12}", kind.key(), mode.name(), c.short()), total, first)
                }));
            }
        }
    }
    let mut sum = 0;
    for j in jobs { let (n, t, f) = j.join().unwrap(); sum += t; if t > 0 { println!("{n} {t:>4}  {f}"); } }
    println!("TOTAL overlap ticks: {sum}");
}
