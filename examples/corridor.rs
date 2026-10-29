//! Corridor study: N intersections along the main road. Compares fixed-time without
//! and with a coordinated green wave, and the adaptive controllers with and without
//! upstream information sharing. Reports end-to-end main-road trips and the mean
//! per-intersection person delay (incl. unserved).
//!
//! usage: corridor [layout] [n] [minutes] [seeds] [demand-scale]
use intersection_lab::control::CtrlKind;
use intersection_lab::sim::demand::DemandConfig;
use intersection_lab::sim::network::{LayoutKind, Network};
use intersection_lab::sim::world::World;
use intersection_lab::sim::ControlMode;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let kind = args.get(1).and_then(|k| LayoutKind::ALL.into_iter().find(|x| x.key() == k)).unwrap_or(LayoutKind::Arterial4);
    let n: usize = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(3);
    let minutes: f64 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(20.0);
    let seeds: u64 = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(4);
    let mut demand = DemandConfig::default_for(&Network::build(kind));
    demand.scale = args.get(5).and_then(|s| s.parse().ok()).unwrap_or(1.0);
    let runs: Vec<(&str, CtrlKind, bool, bool)> = vec![
        ("Fixed (isolated Webster)", CtrlKind::Fixed, false, true),
        ("Fixed (coordinated green wave)", CtrlKind::Fixed, true, true),
        ("Actuated", CtrlKind::Actuated, false, true),
        ("Max-Pressure", CtrlKind::MaxPressure, false, true),
        ("Predictive, no upstream link", CtrlKind::Predictive, false, false),
        ("Predictive + upstream link", CtrlKind::Predictive, false, true),
        ("Hybrid + upstream link", CtrlKind::Hybrid, false, true),
        ("Deep RL + upstream link", CtrlKind::DeepRl, false, true),
    ];
    println!(
        "=== Corridor: {} x {} | {:.0} min x {} seeds ===\n{:<32} {:>10} {:>9} {:>10} {:>10} {:>10} {:>6}",
        n, kind.name(), minutes, seeds, "control", "persDly*", "thru n", "thru time", "thru stops", "thru dly", "coll"
    );
    let handles: Vec<_> = runs
        .into_iter()
        .map(|(name, c, coord, link)| {
            let demand = demand.clone();
            std::thread::spawn(move || {
                let mut acc = [0.0f64; 6];
                for seed in 0..seeds {
                    let mut w = World::corridor(n, kind, ControlMode::Signal, c, &demand, 2000 + seed);
                    for s in &mut w.sims {
                        s.sensors.upstream_link = link;
                        s.check_safety = true;
                    }
                    w.set_coordination(coord);
                    w.run_for(300.0 + minutes * 60.0);
                    let cs = w.corridor_summary(300.0);
                    acc[0] += w.mean_person_delay_all(300.0);
                    acc[1] += cs.through_n as f64;
                    acc[2] += cs.through_travel;
                    acc[3] += cs.through_stops;
                    acc[4] += cs.through_delay;
                    acc[5] += w.sims.iter().map(|s| s.collisions).sum::<u64>() as f64;
                }
                for a in acc.iter_mut().take(5) {
                    *a /= seeds as f64;
                }
                (name, acc)
            })
        })
        .collect();
    for h in handles {
        let (name, a) = h.join().unwrap();
        println!("{:<32} {:>9.1}s {:>9.0} {:>9.1}s {:>10.2} {:>9.1}s {:>6.0}", name, a[0], a[1], a[2], a[3], a[4], a[5]);
    }
}
