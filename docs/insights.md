# Insights from the simulations

What the experiments in this repository have shown so far, using the final model:
calibrated driving, lane changing and realistic sensor errors.

Delay figures are **person delay**, counting vehicles still queued so that no strategy can
look good by starving an approach. They are averages over 4–8 random seeds, so differences
under about 1–2 s are within noise. The commands that reproduce each result are listed at
the end.

## 1. Signal control: model-based adaptive control is the strong default

| Controller | Arterial | Major/minor | Town | T-junction |
|---|---|---|---|---|
| Fixed-time (Webster) | 30.5 s | 37.9 s | 28.6 s | 21.2 s |
| Actuated (NEMA) | 38.4 s | 54.2 s | 25.5 s | 17.9 s |
| Max-Pressure | 20.1 s | 21.6 s | 29.9 s | 15.8 s |
| Predictive | **16.7 s** | 25.4 s | 23.1 s | 14.0 s |
| Hybrid (RL-tuned predictive) | 17.4 s | **19.8 s** | **22.0 s** | **13.1 s** |

- **Multi-lane layouts:** predictive control and Max-Pressure cut delay 30–45% against a
  well-tuned fixed plan, and cut pedestrian waits by a third to a half.
- **Single-lane town crossing:** the gap is small, and plain actuated control is
  competitive.
- **Actuated on the busier layouts:** it is often worse than fixed-time there, because it
  extends greens for every arriving car without weighing who else is waiting.

## 2. AI is most useful tuning a planner, not replacing it

- **Hybrid:** the predictive controller makes every decision and a neural network
  re-tunes it every 10 s. It was best or near-best on every layout.
- **Where the gain comes from:** mostly a better *fixed* tuning of the predictive
  controller, which a one-off search finds. The adaptive part added a little on two
  layouts (arterial 17.2 → 17.0 s, T-junction 13.5 → 13.0 s) and nothing on the other two.
- **Pure Deep RL:** it beats fixed and actuated control on the big layouts but not the
  model-based controllers, and it is brittle on the town layout (45 s).
- **Hidden cost:** its best result, on the T-junction, came with pedestrian waits twice as
  long as the other controllers'. Optimising one number hid that.

## 3. Sensors: coverage matters more than accuracy

- **Cameras:** with realistic errors they are still the best single upgrade, because they
  see turn intent and queues. On the town layout, predictive control goes from 43.6 s
  with loops only to 20.3 s with a camera.
- **Realistic vs perfect sensors:** within noise. Weather (night, rain, fog, snow) cost
  adaptive control only 0–2 s, because radar backs up the camera.
- **Failed loops:** they matter a great deal when loops are the only sensor. Without
  failure detection, a quarter of the loops failing pushed delay to about 200 s. With
  standard failure detection and a fallback to historical volumes, it degrades gracefully
  to about 40 s.
- **Connected vehicles:** at 30% of vehicles they add little on top of camera and radar.
  With every vehicle connected they could replace the other sensors for adaptive control,
  but not for actuated control, which depends on loops.

## 4. Corridors

- **Green wave:** a coordinated plan is clearly worth it for fixed-time signals. Stops per
  through-trip fell 13% on the arterial and 55% on the major/minor road (3.19 → 1.42).
- **Adaptive control:** it beats the green wave even without coordinating anything
  (3 arterials: 17.8 s vs 26.6 s).
- **Sharing upstream information:** telling each signal what its upstream neighbour just
  released helped about 4% on an uncongested arterial but hurt in congestion and on the
  major/minor road. More lookahead alone isn't enough; the real gains need coordinated
  plans, as in SURTRAC.

## 5. Pedestrian safety

- **The combined treatment:** a leading pedestrian interval plus a "no turn on red while
  pedestrians are present" sign roughly halved near-misses at busy crosswalks
  (2.2 → 1.2 per hour), for 1–5 s more delay.
- **Leading interval alone:** it removes conflicts at the start of green, but most
  near-misses come from right turns on red and from turns later in the green.

## 6. Left turns

- **Permissive with a flashing yellow arrow:** it gives the least delay at normal traffic.
  Protected-only costs 23–76% more on the arterial.
- **Under heavy traffic:** permissive lefts can collapse. Left-turners can't find gaps, the
  bay fills and blocks the through lane (fixed-time on major/minor at 1.3×: 168 s vs 89 s
  protected-only).
- **Automatic switching by volume:** it lands in between.

## 7. Choosing the intersection type

- **Roundabout:** excellent at moderate traffic (16.4 s, comparable to the best signals).
- **Two-way stop:** fails once the major road is busy, because side-street drivers can't
  find gaps.
- **All-way stop:** only competitive at low traffic once drivers take a realistic moment to
  pull away. Even at 0.6× traffic on the town layout it averaged 15.0 s against 10.2 s for
  actuated signals, which reverses an earlier finding.

## 8. Lessons about the model itself

- **Realism changes conclusions:** adding driver reaction time, realistic yellow behaviour
  and real turn bays changed several results. All-way stops lost their low-traffic
  advantage, protected-left capacity had been badly understated, and bay spillback became
  visible.
- **Stress testing finds bugs:** multi-seed runs at high traffic exposed two right-of-way
  bugs that normal traffic never triggered. Overlaps are now zero at 1×, 1.35× and 1.6×
  traffic.
- **Honest baselines matter:** without comparing against the best fixed tuning, the hybrid
  would have looked like a 5–10% AI win when most of that gain was simply better settings.

## Caveats

- **Safety measures:** drivers in the model are law-abiding, so safety is measured through
  near-misses and overlaps, not crashes.
- **Corridors:** they run east–west only.
- **Training data:** the RL agents were trained on simulated traffic only.

## Reproducing the results

```bash
cargo run --release --bin bench all 20 6               # sections 1 and 7
cargo run --release --bin train all 400 900 both       # section 2 (hybrid vs best static tuning)
cargo run --release --bin bench sensors all 20 4       # section 3
cargo run --release --example corridor arterial4 3 20 8
cargo run --release --example corridor majorminor4 4 20 8   # section 4
cargo run --release --example lpi 12                   # section 5
cargo run --release --example fya 6                    # section 6
cargo run --release --bin bench town4 20 6 0.6         # section 7 (low-traffic town)
cargo run --release --example satflow                  # calibration
cargo run --release --example sweep 4 1.35             # safety sweep
```
