//! Regression tests: every layout/control combination must run collision-free and
//! serve (nearly) all demand at default volumes.

use trafficsim::control::CtrlKind;
use trafficsim::sim::network::LayoutKind;
use trafficsim::sim::{ControlMode, Sim};

#[test]
fn all_scenarios_safe_and_flowing() {
    let mut handles = vec![];
    for kind in LayoutKind::ALL {
        for mode in ControlMode::options(kind) {
            let ctrls: Vec<CtrlKind> =
                if mode == ControlMode::Signal { CtrlKind::ALL.to_vec() } else { vec![CtrlKind::Fixed] };
            for c in ctrls {
                handles.push(std::thread::spawn(move || {
                    let mut sim = Sim::new(kind, mode, c, 77);
                    sim.check_safety = true;
                    sim.run_for(900.0);
                    let s = sim.stats.summary(0.0, sim.t, &sim.vehicles, &sim.queues);
                    let served = sim.stats.done.len() as f64 / sim.stats.generated.max(1) as f64;
                    // Stop control is legitimately over capacity on busy major roads.
                    let need = if mode == ControlMode::TwoWayStop { 0.65 } else { 0.8 };
                    (format!("{} / {} / {}", kind.key(), mode.name(), c.short()), sim.collisions, served, s.avg_delay, need)
                }));
            }
        }
    }
    let mut failures = vec![];
    for h in handles {
        let (name, coll, served, delay, need) = h.join().unwrap();
        if coll > 0 {
            failures.push(format!("{name}: {coll} overlap ticks"));
        }
        if served < need {
            failures.push(format!("{name}: only {:.0}% of arrivals served (delay {delay:.0}s)", served * 100.0));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn identical_arrivals_across_controllers() {
    let generated = |c| {
        let mut sim = Sim::new(LayoutKind::Arterial4, ControlMode::Signal, c, 5);
        sim.run_for(300.0);
        sim.stats.generated
    };
    assert_eq!(generated(CtrlKind::Fixed), generated(CtrlKind::MaxPressure));
}
