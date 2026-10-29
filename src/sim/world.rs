//! A world of one or more intersections that are stepped together and drawn side by
//! side in world coordinates.

use super::Sim;
use super::geom::{V2, v2};

pub struct World {
    pub sims: Vec<Sim>,
    /// Centre of each intersection in world coordinates.
    pub offsets: Vec<V2>,
}

impl World {
    pub fn single(sim: Sim) -> World {
        World { sims: vec![sim], offsets: vec![v2(0.0, 0.0)] }
    }

    pub fn t(&self) -> f64 {
        self.sims[0].t
    }

    pub fn dt(&self) -> f64 {
        self.sims[0].dt
    }

    pub fn step(&mut self) {
        for s in &mut self.sims {
            s.step();
        }
    }

    /// Copy the user-adjustable settings (sensors, operations, demand) of intersection
    /// `from` to every other intersection.
    pub fn sync_settings(&mut self, from: usize) {
        let src = &self.sims[from];
        let (sensors, demand) = (src.sensors.clone(), src.demand.clone());
        let (rtor, ntor, lpi, left, pre, guard) =
            (src.rtor, src.ntor_peds, src.signal.lpi, src.left_mode, src.preemption, src.max_wait_guard);
        for (i, s) in self.sims.iter_mut().enumerate() {
            if i == from {
                continue;
            }
            s.sensors = sensors.clone();
            s.demand = demand.clone();
            s.rtor = rtor;
            s.ntor_peds = ntor;
            s.signal.lpi = lpi;
            s.left_mode = left;
            s.preemption = pre;
            s.max_wait_guard = guard;
        }
    }

    /// Copy settings from another world (the comparison twin follows the main world).
    pub fn follow(&mut self, other: &World) {
        for (s, o) in self.sims.iter_mut().zip(&other.sims) {
            s.sensors = o.sensors.clone();
            s.demand = o.demand.clone();
            s.rtor = o.rtor;
            s.ntor_peds = o.ntor_peds;
            s.signal.lpi = o.signal.lpi;
            s.left_mode = o.left_mode;
            s.preemption = o.preemption;
            s.max_wait_guard = o.max_wait_guard;
        }
    }
}
