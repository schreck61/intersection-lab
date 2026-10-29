# Intersection Lab

A graphically rich, microscopic traffic-intersection simulator in Rust for exploring how
signal control, sensing and AI affect what drivers, transit riders and pedestrians actually
experience.

![Arterial intersection under predictive control](docs/arterial.png)

```bash
cargo run --release                         # the interactive app
cargo run --release --bin bench             # headless controller comparison (all layouts)
cargo run --release --bin bench sensors all # sensor-package study
cargo run --release --bin train             # train the Deep-RL agents (writes weights/)
cargo test --release                        # safety + throughput regression tests
```

## What you can do in the app

* **Intersections:** 4-way arterial with left-turn bays and medians, 4-way major/minor,
  4-way single-lane town intersection, T-junction, single-lane roundabout.
* **Control:** traffic signal (5 controllers, hot-swappable while traffic runs), all-way stop,
  two-way stop, roundabout yield.
* **Demand:** per-approach volume (veh/h) and turning shares, global multiplier, vehicle mix,
  pedestrian volume, and arrival pattern (Poisson, platoons from an upstream signal,
  rush-hour peak, random surges).
* **Spawn by hand:** pick a vehicle type (car, SUV, truck, bus, motorcycle, emergency) and an
  intent (left/through/right), then click an approach road (or the N/E/S/W buttons). Pick
  *Pedestrian* and click a crosswalk corner to add a pedestrian.
* **Sensors:** toggle loops, cameras, advance radar and connected vehicles, and adjust their range or penetration.
  Loop zones light up on actuation; camera tracks get bounding brackets; radar tracks get rings.
* **Inspect:** click any vehicle to see its speed, delay, stops, fuel, which sensor sees it,
  and who it is yielding to (arrow). Optional overlays: conflict points, paths, delay labels,
  night mode, colour by type, delay, turn or speed.
* **Statistics:** HCM control delay and LOS, 95th percentile, throughput, stops, person delay,
  CO₂, pedestrian wait, delay by approach/movement/vehicle class, delay histogram, live charts,
  and a signal timing diagram.
* **Controller brain:** each adaptive controller explains its decision live (stage pressures,
  predicted delay of each candidate schedule, or the neural network's Q-values).
* **Benchmark:** *Compare controllers* and *Sensor study* run every option headless on identical
  arrivals (3 seeds × 20 min) and chart the results.

Keyboard: `Space` pauses. Mouse wheel zooms, drag pans.

## How the simulation works

| Piece | Model |
|---|---|
| Car following | Intelligent Driver Model (Treiber), per-class parameters, driver heterogeneity in desired speed |
| Turning speed | Curvature of each connector path → comfortable lateral acceleration limit |
| Geometry | Lanes, connectors (Bezier/arc), crosswalks built from a compact lane configuration; long vehicles bend along their path |
| Right of way | Every pair of paths is intersected into **conflict points** with geometrically measured zones. Vehicles compare priority (signal indication, stop order, circulating vs entering) and predicted occupancy windows, accept gaps for the *whole* manoeuvre, and **commit** once they can no longer stop comfortably. Commitments go stale when a vehicle stops, merges are re-resolved every step, and wait-for cycles are broken by ignoring vehicles that are themselves waiting. |
| Signals | Stage-based state machine with yellow change and all-red clearance, pedestrian WALK + flashing DON'T WALK (3.5 ft/s clearance), min green that stretches for pedestrians, right turn on red |
| Pedestrians | Push-button calls, walk during compatible stages, vehicles yield; at unsignalised crosswalks they assert right of way to vehicles that can stop comfortably |
| Stats | Control delay = travel time − free-flow time of that vehicle's exact route (computed by a solo run), including time queued upstream of the network |
| Emissions | Tractive-power fuel model (inertia + rolling + aero, idle burn) → CO₂ |
| Safety | Optional separating-axis overlap test between segmented vehicle bodies; `tests/safety.rs` runs every layout × control combination and requires zero overlaps |

Arrivals come from a dedicated random stream, so every controller faces the *same* vehicles,
which keeps comparisons fair.

## Signal controllers

All controllers see only the `SensorView`, never ground truth.

1. **Fixed-time (Webster).** Cycle `C₀ = (1.5L+5)/(1−Y)` and splits from critical lane flow
   ratios of the configured (historical) demand. Uses no sensors.
2. **Actuated (NEMA-style).** Min green, unit extension per actuation (2.5 s passage time),
   gap-out / max-out, phase skipping, recall on detector failure, and a transit-priority
   extension when a camera or connected bus is about to arrive.
3. **Max-Pressure (Varaiya 2013).** Throughput-optimal: serve the stage with the largest
   upstream-minus-downstream queue pressure, with a switching threshold.
4. **Predictive, schedule-driven MPC (in the spirit of CMU's SURTRAC).** Every second, takes
   each detected vehicle's predicted arrival, enumerates "hold *x* s then switch to *j*, then
   greedy" schedules over a 90 s horizon, simulates discharge (saturation headway, start-up
   lost time, clearance), and minimises **person** delay. Buses count by occupancy, emergency
   vehicles heavily, and long waits super-linearly, for fairness.
5. **AI: Deep RL (Double DQN).** A pure-Rust MLP (128-128) trained by reinforcement learning:
   * *State:* per-movement queue, arrivals within 10 s and 10–30 s, longest wait, bus
     presence, and service status; current stage; time in stage; waiting pedestrians.
   * *Action:* the next stage (choosing the current stage extends it).
   * *Reward:* negative person delay accrued until the next decision, with semi-Markov discounting.
   * *Training:* the predictive controller acts as a decaying exploration expert, on randomised
     demand (volumes, turning shares, patterns, pedestrians, buses).

   Pretrained weights ship in `weights/`. In the app you can keep learning online, adjust ε,
   and save weights.

Safety layers sit on top of every controller: emergency-vehicle **preemption**, and a
**max-wait guard** that forces service of any movement waiting longer than *N* s.

## Sensors and why they matter

| Sensor | What controllers get |
|---|---|
| Inductive loops | Stop-bar presence, advance-detector pulses and spot speed, input-output queue estimate. No turn intent in shared lanes. |
| Intersection camera | Tracked vehicles within range, with **intent** (turn signals), class (bus/truck), individual wait times, and waiting pedestrians |
| Advance radar | Long-range position and speed tracks, so arrival times can be predicted 15–20 s ahead |
| Connected vehicles (V2I) | Position, speed, route and class from equipped vehicles at any distance |
| Preemption receiver | Emergency-vehicle approach |

See *Benchmark results* below for measured effects.

## Benchmark results

Default demand, 4 seeds × 20 min after a 5-min warm-up, with all sensors except connected
vehicles (`cargo run --release --bin bench all 20 4`). **Person delay\*** counts every
vehicle that arrived, including those still queued, so a policy can't look good by starving an
approach. Lower is better.

| Person delay\* (s) | Arterial | Major/minor | Town | T-junction |
|---|---|---|---|---|
| Fixed-time (Webster) | 32.8 | 28.4 | 28.6 | 19.2 |
| Actuated (NEMA) | 40.1 | 62.1 | 27.6 | 16.1 |
| **Max-Pressure** | **19.0** | **22.5** | 29.8 | **13.8** |
| **Predictive MPC** | **18.9** | 25.1 | 27.9 | 14.5 |
| Deep RL (shielded DQN) | 23.6 | 28.2 | 63.1 | 14.8 |
| All-way stop | 52.6 | 39.7 | **22.5** | 17.1 |
| Two-way stop | — | 154.9 | — | 45.5 |
| Roundabout | 17.4 | | | |

Zero vehicle-body overlaps in every run.

### Sensor study (`bench sensors all`)

Person delay\* (s) by sensor package:

| Arterial | loops | + camera | + radar | + 30% CV | 100% CV only |
|---|---|---|---|---|---|
| Max-Pressure | 20.0 | 19.9 | 18.8 | 18.8 | 18.8 |
| Predictive MPC | 21.0 | **17.9** | 19.4 | 18.7 | 18.7 |
| Deep RL | 37.7 | 29.5 | 29.1 | 28.4 | 26.1 |

### What the experiments say

* **Adaptive, sensor-driven control pays off where it matters.** On the busy arterial,
  Max-Pressure and the predictive controller cut person delay by about 40% against an optimised
  Webster plan and about 50% against NEMA actuation, and they also halve pedestrian wait.
* **Cameras are the most valuable single upgrade.** Video analytics give exact per-movement
  queues, turn intent in shared lanes, vehicle class (bus priority) and individual wait times.
  Loops cannot see intent or queue length. The camera is what makes the learning controller
  usable (town: 138 s → 66 s; major/minor: 51 → 37 s).
* **Long-range radar and connected vehicles** extend the prediction horizon. They help
  arrival-predicting controllers on platooned or surging traffic. With random arrivals and a
  camera already present the extra gain is small, because the camera already sees most of the
  vehicles that matter in the next green.
* **Right tool for the job:** at low, balanced volumes (town) an all-way stop beats every
  signal, and the roundabout is excellent at moderate volumes. A two-way stop fails once the
  major road is busy (HCM capacity of the minor approach collapses). The simulator reproduces
  these textbook results.
* **Deep RL** learns a real policy: it beats fixed-time and actuated control on the arterial
  and major/minor layouts, and nearly ties the best on the T-junction. But it does not beat the
  model-based controllers, and it is brittle on the shared-lane town layout. The two things that
  made it competitive were the same ones used in practice:
  1. a **safety shield** (action masking: max green, no holding an empty stage, no switching to
     an empty stage);
  2. **model selection on a broad scenario set**, to avoid over-fitting a few seeds.

  Model-based optimisation (max-pressure / rolling-horizon MPC) is the stronger default. RL is
  most promising as a learned value function or residual on top of MPC, trained much longer.

### Emergency preemption

`cargo run --release --example preempt`: emergency-vehicle delay through the arterial drops
from 25.9 s to 4.2 s with preemption.

## Repository layout

```
src/sim/        network geometry, IDM vehicles, right-of-way, signals, sensors, demand, stats
src/control/    fixed, actuated, max-pressure, predictive MPC, deep-RL controllers
src/rl/         MLP + Adam, Double DQN with replay
src/gui/        eframe/egui app: renderer, charts, panels, self-screenshot (TS_SHOT=…)
src/bin/        bench (comparisons, sensor study), train (RL training)
weights/        pretrained DQN weights per layout
tests/          safety + throughput regression tests
examples/       sweep (multi-seed safety sweep), preempt
```

## Notes and limitations

* Lanes are full-length and vehicles choose their lane at entry; there is no lane changing.
* Single isolated intersection. The platoon arrival pattern emulates an upstream signal.
* Stop-controlled intersections at very high demand can, rarely, still produce a brief body
  overlap in unusual pedestrian/vehicle sequences (`cargo run --release --example sweep 3 1.35`).
  Signals and the roundabout are clean across all tested demand levels.
* Training is fast because the simulation runs about 5,000× real time per core:
  `RESUME=1 cargo run --release --bin train all 500` fine-tunes the shipped weights.
