//! Vehicle classes with Intelligent Driver Model parameters, occupancy (for
//! person-delay weighting) and a simple power-based fuel/CO2 model.

use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum VKind {
    Car,
    Suv,
    Truck,
    Bus,
    Motorcycle,
    Emergency,
}

pub const ALL_KINDS: [VKind; 6] =
    [VKind::Car, VKind::Suv, VKind::Truck, VKind::Bus, VKind::Motorcycle, VKind::Emergency];

#[derive(Clone, Copy, Debug)]
pub struct VSpec {
    pub name: &'static str,
    pub length: f64,
    pub width: f64,
    /// Desired-speed factor relative to the posted limit.
    pub speed_factor: f64,
    /// IDM maximum acceleration (m/s²).
    pub accel: f64,
    /// IDM comfortable deceleration (m/s²).
    pub decel: f64,
    /// IDM desired time headway (s). Calibrated so a queue of cars discharges at about
    /// the HCM base saturation flow (1,900 veh/h/ln); see `examples/satflow.rs`.
    pub headway: f64,
    /// IDM jam distance (m).
    pub min_gap: f64,
    /// Average persons on board (person-delay weighting).
    pub occupancy: f64,
    /// Mass (kg) for the fuel model.
    pub mass: f64,
    /// Drag area Cd*A (m²).
    pub cda: f64,
    /// Idle fuel rate (mL/s).
    pub idle_fuel: f64,
    /// kg CO2 per litre (gasoline 2.31, diesel 2.68).
    pub co2_per_l: f64,
}

impl VKind {
    pub fn spec(self) -> &'static VSpec {
        match self {
            VKind::Car => &CAR,
            VKind::Suv => &SUV,
            VKind::Truck => &TRUCK,
            VKind::Bus => &BUS,
            VKind::Motorcycle => &MOTO,
            VKind::Emergency => &EMERG,
        }
    }
    pub fn index(self) -> usize {
        ALL_KINDS.iter().position(|&k| k == self).unwrap()
    }
    pub fn name(self) -> &'static str {
        self.spec().name
    }
}

static CAR: VSpec = VSpec {
    name: "Car",
    length: 4.5,
    width: 1.85,
    speed_factor: 1.0,
    accel: 2.0,
    decel: 2.5,
    headway: 1.0,
    min_gap: 2.0,
    occupancy: 1.4,
    mass: 1500.0,
    cda: 0.7,
    idle_fuel: 0.25,
    co2_per_l: 2.31,
};
static SUV: VSpec = VSpec {
    name: "SUV",
    length: 5.1,
    width: 2.0,
    speed_factor: 1.0,
    accel: 1.8,
    decel: 2.5,
    headway: 1.1,
    min_gap: 2.0,
    occupancy: 1.6,
    mass: 2200.0,
    cda: 1.0,
    idle_fuel: 0.33,
    co2_per_l: 2.31,
};
static TRUCK: VSpec = VSpec {
    name: "Truck",
    length: 16.5,
    width: 2.55,
    speed_factor: 0.9,
    accel: 0.8,
    decel: 1.8,
    headway: 1.6,
    min_gap: 3.0,
    occupancy: 1.0,
    mass: 18000.0,
    cda: 5.5,
    idle_fuel: 0.75,
    co2_per_l: 2.68,
};
static BUS: VSpec = VSpec {
    name: "Bus",
    length: 12.2,
    width: 2.55,
    speed_factor: 0.88,
    accel: 1.0,
    decel: 1.8,
    headway: 1.4,
    min_gap: 2.5,
    occupancy: 25.0,
    mass: 13000.0,
    cda: 6.0,
    idle_fuel: 0.7,
    co2_per_l: 2.68,
};
static MOTO: VSpec = VSpec {
    name: "Motorcycle",
    length: 2.2,
    width: 0.85,
    speed_factor: 1.05,
    accel: 3.2,
    decel: 3.0,
    headway: 0.9,
    min_gap: 1.5,
    occupancy: 1.1,
    mass: 300.0,
    cda: 0.5,
    idle_fuel: 0.1,
    co2_per_l: 2.31,
};
static EMERG: VSpec = VSpec {
    name: "Emergency",
    length: 6.5,
    width: 2.2,
    speed_factor: 1.15,
    accel: 2.6,
    decel: 3.5,
    headway: 0.9,
    min_gap: 2.0,
    occupancy: 3.0,
    mass: 4500.0,
    cda: 1.6,
    idle_fuel: 0.45,
    co2_per_l: 2.68,
};

/// Fuel use (mL) over `dt` at speed `v` (m/s) and acceleration `a` (m/s²).
/// Tractive power model: inertia + rolling + aero, ~30% engine efficiency.
pub fn fuel_ml(spec: &VSpec, v: f64, a: f64, dt: f64) -> f64 {
    let p_w = spec.mass * a * v + spec.mass * 9.81 * 0.011 * v + 0.5 * 1.2 * spec.cda * v * v * v;
    let p = p_w.max(0.0);
    // Gasoline ~ 34.2 MJ/L => 34_200 J/mL; efficiency 0.3.
    (spec.idle_fuel + p / (34_200.0 * 0.30)) * dt
}
