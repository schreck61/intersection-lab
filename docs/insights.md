# Insights from the simulations

What the experiments in this repository have shown so far, using the final model: queue
discharge calibrated against the Highway Capacity Manual, lane changing, and sensors
that make the kinds of errors field equipment makes. Only the discharge is calibrated.
The sensor error rates and the other parameters are plausible values, not measurements
(see the README's [notes and limitations](../README.md#notes-and-limitations)), so the
findings compare strategies rather than predict a particular intersection.

Delay figures are **person delay**, counting vehicles still queued so that no strategy can
look good by starving an approach. They are averages over 4–8 random seeds unless a
section says otherwise. Where a comparison matters it comes from runs paired on the same
seeds or scenarios, with the half-width of the difference's 95% interval (`+1.3 ± 0.4 s`):
a difference smaller than that is within noise. The commands that reproduce each result
are listed at the end.

## 1. Signal control: model-based adaptive control is the strong default

| Controller | Arterial | Major/minor | Town | T-intersection |
|---|---|---|---|---|
| Fixed-time (Webster) | 30.5 s | 37.9 s | 28.1 s | 21.2 s |
| Actuated (NEMA) | 37.5 s | 57.5 s | 26.8 s | 18.3 s |
| Max-Pressure | 20.4 s | 23.2 s | 30.8 s | 15.8 s |
| Predictive | **16.9 s** | 25.4 s | 24.1 s | 14.8 s |
| Hybrid (RL-tuned predictive) | **16.9 s** | **21.5 s** | **22.0 s** | **13.7 s** |

Six seeds of 20 minutes each (`bench all 20 6`). The comparisons below use 30 seeds
(`bench all 20 30`), paired with predictive control.

- **Multi-lane layouts:** predictive control and Max-Pressure cut person delay by 34–42%
  against a well-tuned fixed plan, and pedestrian waits by 30–53%. Max-Pressure trails
  predictive control by 2.5 ± 0.7 s on the arterial and matches it on major/minor
  (−0.3 ± 1.5 s).
- **Single-lane town crossing:** the gaps are smaller, but predictive control still
  leads. Fixed-time trails it by 5.2 ± 2.8 s, actuated control by 7.0 ± 3.5 s and
  Max-Pressure by 9.3 ± 2.7 s.
- **Actuated on the busier layouts:** it is often worse than fixed-time there, because it
  extends greens for every arriving car without weighing who else is waiting.

## 2. AI is most useful tuning a planner, not replacing it

- **Hybrid:** the predictive controller makes every decision and a neural network
  re-tunes it every 10 s. Over 30 seeds at design demand it is ahead of predictive
  control by more than the noise only on major/minor (3.2 ± 1.6 s); on the arterial
  (+0.6 ± 0.7 s), town (+0.2 ± 2.4 s) and T-intersection (−0.6 ± 0.7 s) layouts the two
  are within noise. On held-out test scenarios (varied demand, weather and failed loops)
  its person delay is lower on three layouts, by 0.5 ± 0.9 s on the arterial,
  1.2 ± 1.8 s on major/minor and 1.3 ± 2.8 s on the town layout, and 0.2 ± 0.8 s higher
  on the T-intersection: no difference there is larger than its interval.
- **Where the gain comes from:** mostly a better *fixed* tuning of the predictive
  controller, which a one-off search finds. That tuning is all the Hybrid runs on
  major/minor, its one reliable gain. On the test set the network changed person delay
  by −0.4 ± 0.7 s on the arterial (17.9 → 17.5 s) and −0.8 ± 3.0 s on the town layout
  (25.1 → 24.3 s), both within noise, and nothing on the other two, where no network
  beat the fixed tuning.
- **Pure Deep RL:** on the multi-lane layouts it cuts delay by about 30% against
  fixed-time (arterial 20.7 s, major/minor 25.3 s over six seeds), but over 30 seeds it
  trails predictive control by 2.7 ± 0.7 s on the arterial and is within noise of it on
  major/minor (+0.7 ± 2.3 s). It is ahead of predictive control only on the
  T-intersection, by 1.6 ± 1.0 s (12.5 s over six seeds, the lowest of any controller),
  and trails it on the single-lane town layout by 4.1 ± 3.3 s.
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
  see turn intent and queues. On the town layout, predictive control goes from 48.8 s
  with loops only to 22.5 s with a camera.
- **Realistic vs perfect sensors:** within noise. Weather (night, rain, fog, snow) costs
  adaptive control at most 2.5 s on the multi-lane layouts, because radar backs up the
  camera. On the single-lane town layout four seeds can't separate the conditions:
  predictive control's person delay ranges from 22.7 s in fog to 30.2 s at night, around
  26.6 s in clear weather.
- **Failed loops:** they matter a great deal when loops are the only sensor. Without
  failure detection, a quarter of the loops failing pushed delay to about 200 s. With
  standard failure detection and a fallback to historical volumes, it degrades gracefully
  to 23–47 s, depending on the layout.
- **Connected vehicles:** at 30% of vehicles they add little on top of camera and radar.
  With every vehicle connected they could replace the other sensors for adaptive control,
  but not for actuated control, which depends on loops.

## 4. Corridors

- **Green wave:** a coordinated plan is clearly worth it for fixed-time signals. Stops per
  through-trip fell 16% on the arterial and 56% on the major/minor road (3.23 → 1.41).
- **Adaptive control:** it beats the green wave even without coordinating anything
  (3 arterials: 17.6 s vs 26.6 s).
- **Sharing upstream information:** telling each signal what its upstream neighbor just
  released changed little on the arterial (17.6 → 18.0 s at normal traffic, 36.1 → 35.1 s
  at 1.3×) and hurt on the major/minor road (15.6 → 16.5 s). More lookahead alone isn't
  enough; the real gains need coordinated plans, as in SURTRAC.

## 5. Pedestrian safety

- **The sign does the work:** with predictive control at busy arterial crosswalks
  (240 pedestrians/h), a "no turn on red while pedestrians are present" sign cut
  near-misses from 2.0 to 0.5 per hour alone and to 0.3 with a leading pedestrian
  interval, for 3–5 s more person delay. Most near-misses came from right turns on red.
- **Leading interval alone:** it helped little (2.0 → 1.7 per hour).
- **Too rare to rank:** with fixed-time signals near-misses were already rare, 0.8 per
  hour untreated, about five in the six simulated hours, and on the single-lane town
  layout they were about as rare. At those counts the treatments showed no clear effect.

## 6. Left turns

- **Permissive with a flashing yellow arrow:** it gives the least delay at normal traffic.
  Protected-only costs 21–67% more on the arterial. On the major/minor road predictive
  control is the exception: it does better protected-only (24.4 vs 28.5 s).
- **Under heavy traffic:** permissive lefts can collapse. Left-turners can't find gaps, the
  bay fills and blocks the through lane (fixed-time on major/minor at 1.3×: 170 s vs 99 s
  protected-only).
- **Automatic switching by volume:** it lands in between, except with predictive control
  on major/minor at 1.3×, where it does worse than either (108.5 s, against 91.0 s
  permissive and 104.8 s protected-only).

## 7. Choosing the intersection type

- **Roundabout:** good at moderate traffic. It carries about 1,540 vehicles an hour,
  more than the single-lane town signals (about 1,120), at a median person delay of
  17.2 s over 30 seeds and a mean of 17.8 s, with 30.5 s on the worst seed. Until
  entering vehicles waited for room on the ring (§9), five seeds gave 35–43 s and one
  74 s, for a mean of 23.4 s.
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
- **So do intervals:** six seeds put the Hybrid first or tied for first on three
  layouts. Paired with predictive control over 30 seeds, only its lead on major/minor is
  larger than the noise. `bench` and `train test` now print each difference with its 95%
  interval.
- **Looking at pedestrians finds what overlap checks miss:** a test of emergency
  preemption while people crossed found vehicles passing within half a meter of
  pedestrians in ordinary traffic too, on every layout with crosswalks. A turning vehicle
  already on a crosswalk drove on into someone in front of it, a vehicle yielding to
  traffic just past a crosswalk didn't check the crosswalk on its way, and pedestrians
  stepped out beside the back of a long vehicle whose front had left the intersection.
  Vehicles now stop for them. Over 30 seeds that changed person delay by 0.6 s at most,
  for any controller on any layout. On six seeds of the town layout it moved predictive
  control from 21.8 to 24.1 s, which shows how far six seeds can swing there.
- **Frequent emergency vehicles find what ordinary traffic doesn't:** 2,560 half-hour
  runs on every layout and control mode, with and without one sent through every 90 s
  and with and without four times the pedestrians, gave one overlap, in a run with both.
  Where two paths merge they run close together for up to 12 m before the merge point,
  and vehicles took turns only by when they would reach that point, so a fast vehicle
  going first could overtake a slow one alongside it there. Now the order has to hold
  from where the paths come close as well. The same runs have no overlaps. Seven of
  them, as before the change, have a vehicle that stands in the box for five to six and
  a half minutes: one turning left from the major road at a two-way stop, waiting for a
  gap in oncoming traffic and pedestrians. Two of the seven have no emergency vehicles.
  Over 30 seeds, no signal controller's person delay moved by more than 0.4 s, the
  all-way stop on major/minor moved most (by 2.6 s), and the roundabout's mean stayed at
  23.4 s while single seeds moved both ways. The same stress also locked up the
  single-lane roundabout, in about 2% of runs, before that change and after it.
- **The roundabout needed a stress test of its own:** the long runs above cover only
  signals. With an emergency vehicle sent through every 90 s, the single-lane roundabout
  gridlocked in 11 of 500 half-hour runs, and without one in 1 of 200. An entering
  vehicle could stop where its path already runs close to the ring, behind circulating
  traffic that had stopped, and the circulating vehicles behind it waited for it, until
  each vehicle on the ring waited for another. Heavier traffic found more: at 1.35× half
  the runs, and at 1.6× nine in ten, had a vehicle standing still for over five minutes
  while traffic moved around it. A vehicle still on its entry ranked as entering at the
  next merge around the ring as well, and gave way there to the vehicles entering, so a
  truck, longer than the ring between two entries, had to wait at its entry until both
  merges were clear. Now an entering vehicle waits at its entry until the ring has room
  for all of it beyond the merge, and ranks as circulating at the merges it will reach on
  the ring. Either change alone still leaves runs stuck; together, no vehicle in those
  runs stands still on the roundabout for more than 68 s with emergency vehicles, or
  129 s at 1.6× traffic, against over 26 minutes before. Over 30 seeds of default
  traffic, the roundabout's person delay fell from 23.4 to 17.8 s and its worst seed's
  from 74 to 30.5 s; five seeds got worse, by 3.8 s at most.
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
cargo run --release --bin bench all 20 30              # sections 1 and 2 (paired differences)
cargo run --release --bin train test all 900 both      # section 2 (held-out test of the shipped weights)
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
cargo run --release --example stress 1 20 all 1 ev,peds,both   # section 9 (emergency vehicles on every layout)
cargo run --release --example stress 1 500 roundabout 1 ev     # section 9 (the roundabout with emergency vehicles)
cargo run --release --example stress 1 200 roundabout 1        # section 9 (the roundabout without them)
cargo run --release --example stress 1 200 roundabout 1.35     # section 9 (the roundabout at 1.35× traffic)
cargo run --release --example stress 1 200 roundabout 1.6      # section 9 (the roundabout at 1.6× traffic)
cargo run --release --example satflow                  # calibration
cargo run --release --example sweep 4 1.35             # safety sweep
```
