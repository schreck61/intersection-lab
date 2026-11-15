# Insights from the simulations

What the experiments in this repository have shown so far, using the final model:
calibrated driving, lane changing and realistic sensor errors.

Delay figures are **person delay**, counting vehicles still queued so that no strategy can
look good by starving an approach. They are averages over 4–8 random seeds, so differences
under about 1–2 s are within noise. The commands that reproduce each result are listed at
the end.

## 1. Signal control: model-based adaptive control is the strong default

| Controller | Arterial | Major/minor | Town | T-intersection |
|---|---|---|---|---|
| Fixed-time (Webster) | 30.5 s | 38.1 s | 28.1 s | 21.2 s |
| Actuated (NEMA) | 37.5 s | 57.5 s | 26.5 s | 18.3 s |
| Max-Pressure | 20.4 s | 23.2 s | 30.2 s | 15.8 s |
| Predictive | 16.9 s | 25.4 s | 21.8 s | 14.8 s |
| Hybrid (RL-tuned predictive) | **16.8 s** | **21.5 s** | **21.6 s** | **13.7 s** |

- **Multi-lane layouts:** predictive control and Max-Pressure cut delay 25–45% against a
  well-tuned fixed plan, and on the four-way layouts cut pedestrian waits by a fifth to a
  half.
- **Single-lane town crossing:** the gap is small, and plain actuated control is
  competitive.
- **Actuated on the busier layouts:** it is often worse than fixed-time there, because it
  extends greens for every arriving car without weighing who else is waiting.

## 2. AI is most useful tuning a planner, not replacing it

- **Hybrid:** the predictive controller makes every decision and a neural network
  re-tunes it every 10 s. It was best or near-best on every layout, and on held-out test
  scenarios (varied demand, weather and failed loops) it beat predictive control on every
  layout, by 0.4 to 1.3 s.
- **Where the gain comes from:** mostly a better *fixed* tuning of the predictive
  controller, which a one-off search finds. On the test set the adaptive part added
  0.2 s on the arterial (17.7 → 17.5 s) and 1.5 s on the town layout (25.8 → 24.3 s), and
  nothing on the other two, where no network beat the fixed tuning.
- **Pure Deep RL:** on the multi-lane layouts it cuts delay by about 30% against
  fixed-time but trails the best controller, the hybrid, by about 4.5 s (arterial 21.3 s,
  major/minor 26.0 s). It has the lowest delay of any controller on the T-intersection
  (12.3 s), and on the single-lane town layout it trails fixed-time and actuated control
  (29.0 s).
- **Network size:** for Deep RL, one hidden layer of 128 units beat two on every layout,
  by 1.8 to 4.5 s on held-out scenarios, and doubling the training didn't close the gap.
  Switching to it cut Deep RL's delay by 10–12 s on the four-way layouts. For the hybrid,
  one and two layers did the same within noise.
- **Hidden cost:** Deep RL's best result, on the T-intersection, used to come with
  pedestrian waits twice as long as the other controllers' (46 s). Optimizing one number
  hid that.
- **Pedestrians need a hard rule:** adding waiting pedestrians to Deep RL's reward, each
  counted like one vehicle occupant, didn't fix it. Pedestrians are a small share of the
  people at an intersection, so the network still traded their waits for vehicle delay,
  and only 2 of 9 runs on the arterial, major/minor and T-intersection layouts met a
  check that pedestrian waits stay within 5 s of predictive control's. Weighting
  pedestrians three times as heavily didn't reliably fix it either. A shield rule that
  gives the next walk to anyone who has waited 40 s did: every run passed, and the
  T-intersection wait fell to 21 s. The hybrid passed without any rule, because the
  planner serves pedestrians itself.
- **The rule helps vehicles too, mostly:** trained the same way with and without it
  (three seeds each, held-out test set), the rule cut person delay by 1.4 s on the
  arterial and major/minor layouts and 2.0 s on the town layout, and cost 1.1 s only on
  the T-intersection. One guess, not tested, is that it forces a regular rotation through
  the stages that the network otherwise has to learn. Against 0.3.0's Deep RL, person
  delay is within 0.6 s on three layouts and 2.1 s lower on major/minor, and pedestrian
  waits are 4 to 26 s shorter. The README's benchmark, which scores one network per
  layout, first suggested the rule cost more; networks trained from different seeds
  differ by 1 to 3 s, so one network per layout can't settle a gap that small.

## 3. Sensors: coverage matters more than accuracy

- **Cameras:** with realistic errors they are still the best single upgrade, because they
  see turn intent and queues. On the town layout, predictive control goes from 47.7 s
  with loops only to 22.5 s with a camera.
- **Realistic vs perfect sensors:** within noise. Weather (night, rain, fog, snow) costs
  adaptive control 0–3 s on the multi-lane layouts, because radar backs up the camera. On
  the single-lane town layout night and snow cost about 6 s.
- **Failed loops:** they matter a great deal when loops are the only sensor. Without
  failure detection, a quarter of the loops failing pushed delay to about 200 s. With
  standard failure detection and a fallback to historical volumes, it degrades gracefully
  to 23–45 s, depending on the layout.
- **Connected vehicles:** at 30% of vehicles they add little on top of camera and radar.
  With every vehicle connected they could replace the other sensors for adaptive control,
  but not for actuated control, which depends on loops.

## 4. Corridors

- **Green wave:** a coordinated plan is clearly worth it for fixed-time signals. Stops per
  through-trip fell 14% on the arterial and 55% on the major/minor road (3.17 → 1.44).
- **Adaptive control:** it beats the green wave even without coordinating anything
  (3 arterials: 17.8 s vs 26.7 s).
- **Sharing upstream information:** telling each signal what its upstream neighbor just
  released made no difference on the arterial at normal traffic (17.8 vs 17.9 s), helped
  only a little at 1.3× (36.4 → 35.5 s), and hurt on the major/minor road
  (15.1 → 16.4 s). More lookahead alone isn't enough; the real gains need coordinated
  plans, as in SURTRAC.

## 5. Pedestrian safety

- **The combined treatment:** a leading pedestrian interval plus a "no turn on red while
  pedestrians are present" sign cut near-misses at busy arterial crosswalks by a half to
  two-thirds (2.3 → 1.2 per hour with fixed-time, 2.3 → 0.8 with predictive control), for
  1–4 s more delay. On the single-lane town layout it made little difference.
- **Leading interval alone:** it helped little (2.3 → 1.8 with fixed-time, none with
  predictive control). Most near-misses come from right turns on red and from turns later
  in the green.

## 6. Left turns

- **Permissive with a flashing yellow arrow:** it gives the least delay at normal traffic.
  Protected-only costs 20–76% more on the arterial. On the major/minor road predictive
  control is the exception: it does better protected-only (23.5 vs 29.4 s).
- **Under heavy traffic:** permissive lefts can collapse. Left-turners can't find gaps, the
  bay fills and blocks the through lane (fixed-time on major/minor at 1.3×: 157 s vs 98 s
  protected-only).
- **Automatic switching by volume:** it lands in between.

## 7. Choosing the intersection type

- **Roundabout:** excellent at moderate traffic (16.2 s, comparable to the best signals).
- **Two-way stop:** fails once the major road is busy, because side-street drivers can't
  find gaps.
- **All-way stop:** only competitive at low traffic once drivers take a realistic moment to
  pull away. Even at 0.6× traffic on the town layout it averaged 15.0 s against 10.5 s for
  actuated signals, which reverses an earlier finding.

## 8. A signal-free crossing for automated vehicles

The cooperative crossing assumes every vehicle is automated and connected. An
intersection manager grants each vehicle an exact trajectory, and pedestrians use
underpasses. Figures are average delay over 3 seeds of 20 minutes, with fail-safe gaps
(a follower can stop if the vehicle ahead stalls in the box) and the default vehicle mix.
The design notes in [cooperative-intersections.md](cooperative-intersections.md) explain
the design and compare these results with their estimates.

- **It carries the design demand:** two left-turn lanes, three through lanes and a slip
  right turn on each approach carry 18,000 veh/h at 29 s average delay and saturate near
  19,000 veh/h, or about 21,700 if every vehicle were a car.
- **Batches, not a zipper:** the capacity comes from serving each movement in short
  batches, in two rings with a barrier like an actuated signal, but with no yellow or
  all-red. Weaving vehicles through one at a time, first come, first served, is better
  only below about 6,000 veh/h and saturates near 10,100 veh/h. Each switch between
  conflicting paths costs about 2.1 s, against 1.0 s for one more following vehicle;
  each path crosses ten other lanes, whose free windows must all line up; and a vehicle
  waiting for its windows holds up its whole lane.
- **The safety rule matters most to the zipper:** gaps that cover only message latency
  and the actuation delay add 25% to first come, first served (12,600 veh/h) but only
  3.5% to batches (19,700), which switch a few times a minute instead of at almost every
  vehicle.
- **Left-turn lanes:** with one per approach the crossing saturates near 13,500 veh/h;
  the second is worth about 5,500 veh/h.
- **Uneven arrivals:** at the design demand (8 seeds), platoons from an upstream signal
  average 34 s against 31 s for random arrivals. Random 2-minute surges of up to 1.7× on
  an approach average 48 s and leave about 160 vehicles waiting.
- **Emergency vehicles:** scheduled first, they waited 7.5–10.4 s at 14,000 veh/h, against
  39 s for other left turns.
- **Safety:** no run had a zone overlap, a zone entered while occupied, a box entry
  without a reservation or a step off a granted trajectory.

None of those runs has a failure in it, and the safety gaps are sized for one. So a
vehicle was made to fail in the box, once per run, 400 runs per case, in batches at
16,000 veh/h and first come, first served at 8,000 veh/h:

- **Each gap survives what it is sized for, with little to spare:** the hardest case is
  a vehicle that stops dead just as it is about to clear a zone, with a crossing vehicle
  due right behind it at the smallest gap allowed. With fail-safe gaps that vehicle
  never reached the failed one, and in batches its closest stop ended 0.6 m short of the
  zone. With mitigated gaps it reached the failed vehicle in 54–85% of failures, at up
  to 22 km/h (the gap allows 25), and with nominal gaps in 89–96%, at up to 35 km/h. A
  vehicle that only braked to a stop, the failure the nominal gap is sized for, was
  reached in 5 of 800 such failures with nominal gaps and never with the wider ones.
- **Batches make most failures harmless to crossing traffic:** at a random place and
  time the next vehicle to cross the failed one's path is usually a whole phase away,
  so its crossing is withdrawn and it stops short of the box. Even with nominal gaps,
  2 of 400 such failures led to a crossing contact, against 11% first come, first
  served (3% with mitigated gaps, none with fail-safe ones). Batching is safer as well
  as faster: it needs its gap only at a change of phase, where the zipper needs it at
  almost every vehicle.
- **No gap protects the vehicle behind:** in a lane cars follow about 0.6 s apart at the
  crossing speed (2 m plus a 0.42 s time gap), which lets them survive the one ahead
  braking, not stopping dead. Mid-batch, a vehicle that stopped dead was struck from
  behind in 86–89% of failures, at up to 38 km/h, whatever the gap. One that braked to a
  stop was struck 3 times in 4,800 failures, at 6 km/h at most, and never in batches.
- **The fail-safe gap has a blind spot:** it is sized for the speed at which a vehicle
  enters a zone, and a vehicle still slowing from the approach speed to the crossing
  speed carries more than that when it has to react. First come, first served, an
  emergency stop ended up to 0.4 m inside its zone in 4 of 400 failures at the worst
  moment, clear of the failed vehicle. None did in batches, where the first vehicle of a
  phase isn't slowing as it enters.
- **A disabled vehicle costs about a sixth of the flow while it stands:** the manager
  closes the zones it holds and serves the lanes that stay open, 83–88% of the usual
  traffic until it is cleared away, 60 s later. The backlog takes longer to clear: about
  3 minutes at 12,000 veh/h and 5 at 16,000 veh/h, and at the design demand of
  18,000 veh/h 53 of 100 runs were not back to normal 13 minutes later.

## 9. Lessons about the model itself

- **Realism changes conclusions:** adding driver reaction time, realistic yellow behavior
  and real turn bays changed several results. All-way stops lost their low-traffic
  advantage, protected-left capacity had been badly understated, and bay spillback became
  visible.
- **Stress testing finds bugs:** multi-seed runs at high traffic exposed right-of-way bugs
  that normal traffic never triggered: overlaps, and gridlocks in which vehicles waiting
  inside the box each blocked another's path. Overlaps are now zero at 1×, 1.35× and 1.6×
  traffic, and 960 long runs (6 controllers × 16 seeds on every signalized layout, at
  1×–1.6× traffic for 5–20 simulated hours) finish with no gridlock and no overlaps.
- **Honest baselines matter:** without comparing against the best fixed tuning, the hybrid
  would have looked like a 5–10% AI win when most of that gain was simply better settings.
- **Held-out testing matters:** the earlier hybrid scored 0.9 s better than predictive
  control on the 12 scenarios used to choose it, and 1.0 s worse in the benchmark. Models
  are now chosen on one scenario set and reported on another that no choice uses.
- **Making things fail finds what normal running hides:** the cooperative crossing's
  monitors read zero through every failure-free run, and so could not show that the
  fail-safe gap falls a few decimeters short for a vehicle that is still slowing down.
  The first failures made on purpose did.

## Caveats

- **Safety measures:** drivers in the model are law-abiding, so safety is measured through
  near-misses and overlaps, not crashes.
- **Corridors:** they run east–west only.
- **Training data:** the RL agents were trained on simulated traffic only.
- **Cooperative crossing:** every vehicle is automated and connected, and human drivers,
  lost messages and a failed manager are not modeled. A vehicle that is made to fail
  stops on its path and reports it at once; vehicles that touch stop where they touch,
  and nothing more of a collision is simulated.

## Reproducing the results

```bash
cargo run --release --bin bench all 20 6               # sections 1 and 7
cargo run --release --bin train all 800 900 dqn        # section 2 (Deep RL)
cargo run --release --bin train all 800 900 hybrid     # section 2 (hybrid vs best static tuning)
cargo run --release --bin bench sensors all 20 4       # section 3
cargo run --release --example corridor arterial4 3 20 8
cargo run --release --example corridor majorminor4 4 20 8   # section 4
cargo run --release --example corridor arterial4 3 20 8 1.3   # section 4 (1.3× traffic)
cargo run --release --example lpi 12                   # section 5
cargo run --release --example fya 6                    # section 6
cargo run --release --bin bench town4 20 6 0.6         # section 7 (low-traffic town)
cargo run --release --example cooperative p2 2 mix 1200 3 14000,16000,17000,18000,19000,20000,22000   # section 8
cargo run --release --example cooperative p2 2 mix 1200 3 2000,4000,6000,8000,10000,12000,14000 random fcfs   # section 8 (first come, first served)
cargo run --release --example cooperative p2 1 mix 1200 3 16000                     # section 8 (one left-turn lane)
cargo run --release --example cooperative p2 2 mix 1200 8 18000 platoons            # section 8 (platoons)
cargo run --release --example failures 400             # section 8 (failures)
cargo run --release --example satflow                  # calibration
cargo run --release --example sweep 4 1.35             # safety sweep
```
