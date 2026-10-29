//! Left-turn phasing: permissive flashing yellow arrow vs protected-only vs automatic
//! switching by the volume cross-product, across demand levels.
use trafficsim::control::CtrlKind;
use trafficsim::sim::network::{LayoutKind, Turn};
use trafficsim::sim::{ControlMode, LeftMode, Sim};

fn main() {
    let seeds: u64 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(6);
    println!("{:<11} {:<11} {:>5} {:<30}| {:>12} {:>10} {:>9}", "layout", "controller", "scale", "left turns", "person delay", "left delay", "protected");
    for kind in [LayoutKind::Arterial4, LayoutKind::MajorMinor4] {
        for ctrl in [CtrlKind::Fixed, CtrlKind::Actuated, CtrlKind::Predictive] {
            for scale in [0.7, 1.0, 1.3] {
                for mode in LeftMode::ALL {
                    let jobs: Vec<_> = (0..seeds)
                        .map(|seed| {
                            std::thread::spawn(move || {
                                let mut sim = Sim::new(kind, ControlMode::Signal, ctrl, 900 + seed);
                                sim.demand.scale = scale;
                                sim.left_mode = mode;
                                sim.run_for(300.0);
                                let t0 = sim.t;
                                sim.run_for(1800.0);
                                let s = sim.stats.summary(t0, sim.t, &sim.vehicles, &sim.queues);
                                let lefts: Vec<usize> = (0..sim.net.movements.len()).filter(|&m| sim.net.movements[m].turn == Turn::L).collect();
                                let (ld, ln) = lefts.iter().fold((0.0, 0), |a, &m| (a.0 + s.by_move[m].delay, a.1 + s.by_move[m].n));
                                let prot = sim.left_protected.iter().filter(|&&p| p).count();
                                (s.person_delay_all, ld / ln.max(1) as f64, prot as f64)
                            })
                        })
                        .collect();
                    let r: Vec<(f64, f64, f64)> = jobs.into_iter().map(|j| j.join().unwrap()).collect();
                    let n = seeds as f64;
                    let m = |f: fn(&(f64, f64, f64)) -> f64| r.iter().map(f).sum::<f64>() / n;
                    println!("{:<11} {:<11} {:>5} {:<30}| {:>11.1}s {:>9.1}s {:>9.1}", kind.key(), ctrl.short(), scale, mode.name(), m(|x| x.0), m(|x| x.1), m(|x| x.2));
                }
            }
        }
    }
}
