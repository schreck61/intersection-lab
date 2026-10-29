//! Emergency vehicles: with and without preemption, spawned against a red.
use intersection_lab::control::CtrlKind;
use intersection_lab::sim::network::{LayoutKind, Turn};
use intersection_lab::sim::vtypes::VKind;
use intersection_lab::sim::{ControlMode, Sim};
fn main() {
    for pre in [false, true] {
        let mut total = 0.0;
        let mut n = 0;
        for k in 0..6 {
            let mut sim = Sim::new(LayoutKind::Arterial4, ControlMode::Signal, CtrlKind::Fixed, 300 + k);
            sim.preemption = pre;
            sim.run_for(200.0 + 17.0 * k as f64);
            sim.spawn_manual((k % 4) as usize, VKind::Emergency, Turn::T);
            sim.run_for(120.0);
            for d in sim.stats.done.iter().filter(|d| d.kind == VKind::Emergency) {
                total += d.delay;
                n += 1;
            }
        }
        println!("preemption={pre}: {n} emergency vehicles, avg delay {:.1} s", total / n.max(1) as f64);
    }
}
