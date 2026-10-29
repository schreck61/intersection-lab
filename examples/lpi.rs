//! Pedestrian safety treatments: leading pedestrian interval (LPI) and a blank-out
//! "NO TURN ON RED" sign lit while pedestrians are present. Reports vehicle/pedestrian
//! near-misses (a vehicle crossing within ~1.5 s of a walking pedestrian) and delay.
use trafficsim::control::CtrlKind;
use trafficsim::sim::network::LayoutKind;
use trafficsim::sim::{ControlMode, Sim};

fn main() {
    let seeds: u64 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(8);
    println!("{:<11} {:<11} {:>6} {:<14}| {:>11} {:>9} {:>12}", "layout", "controller", "peds/h", "treatment", "near-miss/h", "ped wait", "person delay");
    for kind in [LayoutKind::Arterial4, LayoutKind::Town4] {
        for ctrl in [CtrlKind::Fixed, CtrlKind::Predictive] {
            for ped in [60.0, 240.0] {
                for (name, lpi, ntor) in [("none", 0.0, false), ("LPI 4 s", 4.0, false), ("NTOR w/ peds", 0.0, true), ("both", 4.0, true)] {
                    let jobs: Vec<_> = (0..seeds)
                        .map(|seed| {
                            std::thread::spawn(move || {
                                let mut sim = Sim::new(kind, ControlMode::Signal, ctrl, 500 + seed);
                                sim.demand.ped_rate = ped;
                                sim.signal.lpi = lpi;
                                sim.ntor_peds = ntor;
                                sim.run_for(300.0);
                                let t0 = sim.t;
                                sim.run_for(1800.0);
                                let s = sim.stats.summary(t0, sim.t, &sim.vehicles, &sim.queues);
                                (s.ped_near_miss_ph, s.ped_avg_wait, s.person_delay_all)
                            })
                        })
                        .collect();
                    let r: Vec<(f64, f64, f64)> = jobs.into_iter().map(|j| j.join().unwrap()).collect();
                    let n = seeds as f64;
                    let m = |f: fn(&(f64, f64, f64)) -> f64| r.iter().map(f).sum::<f64>() / n;
                    println!("{:<11} {:<11} {:>6} {:<14}| {:>11.1} {:>8.1}s {:>11.1}s", kind.key(), ctrl.short(), ped, name, m(|x| x.0), m(|x| x.1), m(|x| x.2));
                }
            }
        }
    }
}
