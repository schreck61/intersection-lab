//! Calibration: saturation flow and start-up lost time of a queued through lane.
//! Compares the simulated discharge with the HCM base saturation flow (1,900 pc/h/ln).
use trafficsim::control::CtrlKind;
use trafficsim::sim::demand::DemandConfig;
use trafficsim::sim::network::{LayoutKind, Network, Turn};
use trafficsim::sim::{ControlMode, Sim};
use std::collections::HashMap;

fn main() {
    for (label, mix) in [("cars only", [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]), ("default mix", [0.62, 0.24, 0.05, 0.03, 0.04, 0.0])] {
        let net = Network::build(LayoutKind::Arterial4);
        let mut d = DemandConfig::default_for(&net);
        // Saturate the eastbound through lanes; nothing else.
        d.rate = [0.0, 0.0, 0.0, 3000.0];
        d.turn = [[0.0, 1.0, 0.0]; 4];
        d.mix = mix;
        d.ped_rate = 0.0;
        let mut sim = Sim::with_demand(net, ControlMode::Signal, CtrlKind::Fixed, d, 1);
        let m = sim.net.movement_for(sim.net.leg_index(trafficsim::sim::network::Compass::W).unwrap(), Turn::T).unwrap();
        let mut prev_ri: HashMap<u64, usize> = HashMap::new();
        let mut green_start = None;
        // (lane, position in queue discharge, time since green)
        let mut crossings: Vec<(usize, usize, f64)> = Vec::new();
        let mut count: HashMap<usize, usize> = HashMap::new();
        let mut headways = Vec::new();
        let mut last_cross: HashMap<usize, f64> = HashMap::new();
        let mut first_four = Vec::new();
        for _ in 0..(3600.0 / sim.dt) as usize {
            sim.step();
            let green = sim.signal.status(&sim.net, m).is_green();
            if green && green_start.is_none() {
                green_start = Some(sim.t);
                count.clear();
                last_cross.clear();
            } else if !green {
                green_start = None;
            }
            for v in &sim.vehicles {
                let p = prev_ri.insert(v.id, v.ri).unwrap_or(0);
                if p == 0 && v.ri >= 1 && v.movement == m {
                    if let Some(g) = green_start {
                        let lane = v.route[0];
                        let n = count.entry(lane).or_insert(0);
                        *n += 1;
                        let since = sim.t - g;
                        if *n > 4 {
                            if let Some(&t0) = last_cross.get(&lane) {
                                if v.queue_since.is_some() || v.stops > 0 {
                                    headways.push(sim.t - t0);
                                }
                            }
                        } else if *n == 4 {
                            first_four.push(since);
                        }
                        last_cross.insert(lane, sim.t);
                        crossings.push((lane, *n, since));
                    }
                }
            }
        }
        headways.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let h = headways.iter().sum::<f64>() / headways.len() as f64;
        let med = headways[headways.len() / 2];
        let t4 = first_four.iter().sum::<f64>() / first_four.len() as f64;
        // HCM: start-up lost time = time for first four vehicles - 4 * saturation headway.
        println!(
            "{label:>12}: saturation headway {h:.2} s (median {med:.2}) -> {:.0} veh/h/ln; 4th vehicle at {t4:.1} s, start-up lost time {:.1} s  [n={}]",
            3600.0 / h,
            t4 - 4.0 * h,
            headways.len()
        );
    }
    println!("HCM reference: base saturation flow 1,900 pc/h/ln (headway 1.89 s), start-up lost time ~2 s; heavy vehicles reduce flow ~1-2% per 1% trucks.");
}
