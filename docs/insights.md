# Insights from the simulations

What the experiments in this repository have shown so far, using the final model: queue
discharge and gap acceptance calibrated against the Highway Capacity Manual, lane
changing, and sensors that make the kinds of errors field equipment makes. Only those
two are calibrated, and gap acceptance only at two-way stops and on permissive
indications. The sensor error rates and the other parameters are plausible values, not
measurements (see the README's
[notes and limitations](../README.md#notes-and-limitations)), so the findings compare
strategies rather than predict a particular intersection.

Delay figures are **person delay**, counting vehicles still queued so that no strategy can
look good by starving an approach. They are averages over 4–8 random seeds unless a
section says otherwise. Where a comparison matters it comes from runs paired on the same
seeds or scenarios, with the half-width of the difference's 95% interval (`+1.3 ± 0.4 s`):
a difference smaller than that is within noise. The commands that reproduce each result
are listed at the end.

## 1. Signal control: model-based adaptive control is the strong default

| Controller | Arterial | Major/minor | Town | T-intersection |
|---|---|---|---|---|
| Fixed-time (Webster) | 29.9 s | 37.3 s | 22.8 s | 20.6 s |
| Actuated (NEMA) | 36.7 s | 54.1 s | 22.1 s | 17.3 s |
| Max-Pressure | 17.4 s | 20.1 s | 23.4 s | 14.0 s |
| Predictive | 15.3 s | 16.6 s | 18.2 s | **11.9 s** |
| Hybrid (RL-tuned predictive) | **14.7 s** | **15.8 s** | **17.1 s** | 12.4 s |

Six seeds of 20 minutes each (`bench all 20 6`). The comparisons below use 30 seeds
(`bench all 20 30`), paired with predictive control.

- **Multi-lane layouts:** predictive control and Max-Pressure cut person delay by 40–55%
  against a well-tuned fixed plan, and pedestrian waits by 39–55%. Max-Pressure trails
  predictive control by 3.0 ± 0.7 s on the arterial and 2.9 ± 0.9 s on major/minor.
- **Single-lane town crossing:** fixed-time and actuated control come closer, but
  predictive control still leads. Fixed-time trails it by 5.9 ± 1.3 s, actuated control
  by 7.5 ± 2.4 s and Max-Pressure by 4.6 ± 1.2 s.
- **Actuated on the busier layouts:** it is often worse than fixed-time there, because it
  extends greens for every arriving car without weighing who else is waiting.

## 2. AI: tuning a planner works better than replacing it, but adds nothing reliable

- **Hybrid:** the predictive controller makes every decision and a neural network
  re-tunes it every 10 s. Over 30 seeds at design demand it is within noise of
  predictive control on every layout: −0.0 ± 0.4 s on the arterial, −0.2 ± 0.7 s on
  major/minor, −0.0 ± 1.2 s on the town layout and +0.0 ± 0.7 s on the T-intersection.
  On held-out test scenarios (varied demand, weather and failed loops) its person delay
  is higher by 0.1 to 0.4 s on every layout, and no difference there is larger than its
  interval. Before gap acceptance was calibrated (§9) it led predictive control on
  major/minor by 3.7 ± 1.9 s, more than the noise.
- **Where the gain came from:** mostly a better *fixed* tuning of the predictive
  controller, which a one-off search finds. That tuning is all the Hybrid runs on
  major/minor, where no network beat it, and on the test set it is within noise of
  predictive control's own settings on every layout. On the other three the network
  changed person delay by +0.2 ± 0.7 s on the arterial (15.0 → 15.2 s), −1.2 ± 2.8 s on
  the town layout (20.9 → 19.8 s) and +0.7 ± 1.3 s on the T-intersection
  (12.4 → 13.1 s), all within noise.
- **Pure Deep RL:** on the multi-lane layouts it cuts delay by about 40% against
  fixed-time (arterial 17.7 s, major/minor 21.5 s over six seeds), but over 30 seeds it
  trails predictive control by 3.2 ± 0.8 s on the arterial, 5.0 ± 1.8 s on major/minor
  and 4.9 ± 2.9 s on the single-lane town layout. On the T-intersection it is within
  noise of predictive control (−0.6 ± 0.8 s), where before the calibration it led by
  1.5 ± 1.1 s.
- **Retraining on the calibrated model didn't bring the leads back:** both networks were
  retrained with the same settings as before (`train all 800 900 dqn` and `hybrid`,
  seed 0). Paired with the networks trained before the calibration on the same 30
  seeds, neither controller's person delay moved by more than the noise on any layout.
  Two more training seeds (`SEED=1` and `SEED=2`) give the same answer: on major/minor
  their Hybrids trail predictive control by 0.2 ± 0.9 and 0.3 ± 1.0 s, and on the
  T-intersection their Deep RL networks are within noise of it (−0.1 ± 0.7 and
  +0.3 ± 0.8 s). With loops alone, though, the shipped Deep RL networks do much worse
  than the ones they replaced (§3). The training experiments below date from before the
  calibration.
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
  see turn intent and queues. On the town layout, predictive control goes from 22.1 s
  with loops only to 16.6 s with a camera.
- **Realistic vs perfect sensors:** within noise. Weather (night, rain, fog, snow) costs
  adaptive control at most 3.3 s on the multi-lane layouts, because radar backs up the
  camera. On the single-lane town layout predictive control's person delay ranges from
  16.6 s at night to 18.4 s in heavy rain and in snow, against 18.0 s in clear weather.
- **Failed loops:** they matter a great deal when loops are the only sensor. Without
  failure detection, a quarter of the loops failing pushed delay to about 200 s. With
  standard failure detection and a fallback to historical volumes, predictive control
  and the Hybrid degrade gracefully, to 18–23 s depending on the layout.
- **Deep RL can't work from loops alone:** its networks were trained with loops, camera
  and radar, sometimes in bad weather or with a quarter of the loops failed, but never
  with loops alone. With loops alone its person delay reaches 62.8 s on major/minor and
  44.4 s on the town layout, against 21.9 and 19.8 s with all three, and 68.6 and
  70.8 s with a quarter of the loops failed. The Hybrid, whose network only tunes the
  predictive planner, degrades about as predictive control does. Retraining on the
  calibrated model made this worse: over 30 seeds with loops alone, Deep RL's person
  delay went from 32.0 to 69.2 s on major/minor and from 38.4 to 47.7 s on the town
  layout.
- **Connected vehicles:** at 30% of vehicles they add little on top of camera and radar.
  With every vehicle connected they could replace the other sensors for adaptive control,
  but not for actuated control, which depends on loops.

## 4. Corridors

- **Green wave:** a coordinated plan is clearly worth it for fixed-time signals. Stops
  per through-trip fell 14% on the arterial and 56% on the major/minor road
  (3.20 → 1.41).
- **Adaptive control:** it beats the green wave even without coordinating anything (3
  arterials: 15.3 s vs 25.8 s).
- **Sharing upstream information:** telling each signal what its upstream neighbor just
  released didn't help consistently on the arterial (15.3 → 15.8 s at normal traffic,
  25.0 → 24.2 s at 1.3×) and hurt on the major/minor road (13.1 → 14.7 s). More
  lookahead alone isn't enough; the real gains need coordinated plans, as in SURTRAC.

## 5. Pedestrian safety

- **The two treatments work together:** with predictive control at busy arterial
  crosswalks (240 pedestrians/h), a "no turn on red while pedestrians are present" sign
  and a 4 s leading pedestrian interval together cut near-misses from 1.0 to 0.2 per
  hour, for 1.7 s more person delay. Either alone cut them only to 0.8. Untreated, that
  is about six near-misses in the six simulated hours, so the counts are small.
- **Too rare to rank:** with fixed-time signals near-misses were as rare, 1.0 per hour
  untreated, and on the single-lane town layout there were 1.5 to 1.8 per hour. At those
  counts the treatments showed no clear effect.

## 6. Left turns

- **Permissive with a flashing yellow arrow:** it gives the least delay at normal
  traffic, with every controller tested on both layouts. Protected-only costs 28–109%
  more on the arterial and 15–43% more on the major/minor road (17.1 vs 24.5 s with
  predictive control).
- **Under heavy traffic:** permissive lefts can collapse. Left-turners can't find gaps,
  the bay fills and blocks the through lane (fixed-time on major/minor at 1.3×: 163 s vs
  86 s protected-only).
- **Automatic switching by volume:** it lands in between, with every controller at every
  demand tested (fixed-time on major/minor at 1.3×: 121 s).

## 7. Choosing the intersection type

- **Roundabout:** good at moderate traffic. It carries about 1,540 vehicles an hour,
  more than the single-lane town signals (about 1,120), at a median person delay of
  17.2 s over 30 seeds and a mean of 17.8 s, with 30.5 s on the worst seed. Until
  entering vehicles waited for room on the ring (§9), five seeds gave 35–43 s and one
  74 s, for a mean of 23.4 s.
- **Two-way stop:** fails once the major road is busy, because side-street drivers can't
  find gaps: over 30 seeds its person delay is 115.3 s on major/minor, against 16.6 s
  with predictive signals, and 27.5 s on the T-intersection, against 12.3 s.
- **All-way stop:** only competitive at low traffic once drivers take a realistic moment
  to pull away. Even at 0.6× traffic on the town layout it averaged 15.0 s against 9.8 s
  for actuated signals, which reverses an earlier finding.

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
- **Stress testing finds bugs:** multi-seed runs at high traffic exposed right-of-way
  bugs that normal traffic never triggered: overlaps, and gridlocks in which vehicles
  waiting inside the box each blocked another's path. Overlaps are now zero at 1×, 1.35×
  and 1.6× traffic, and 960 long runs (6 controllers × 16 seeds on every signalized
  layout, at 1×–1.6× traffic for 5–20 simulated hours) finished with no gridlock and no
  overlaps before gap acceptance was calibrated. Since then, 960 half-hour runs of the
  same controllers at 1.35× and 1.6× traffic have had none either.
- **Honest baselines matter:** without comparing against the best fixed tuning, the hybrid
  would have looked like a 5–10% AI win when most of that gain was simply better settings.
- **Held-out testing matters:** the earlier hybrid scored 0.9 s better than predictive
  control on the 12 scenarios used to choose it, and 1.0 s worse in the benchmark. Models
  are now chosen on one scenario set and reported on another that no choice uses.
- **So do intervals:** six seeds put the Hybrid first on three layouts. Paired with
  predictive control over 30 seeds, none of those leads is larger than the noise; before
  gap acceptance was calibrated, only its lead on major/minor was. `bench` and
  `train test` now print each difference with its 95% interval.
- **Looking at pedestrians finds what overlap checks miss:** a test of emergency
  preemption while people crossed found vehicles passing within half a meter of
  pedestrians in ordinary traffic too, on every layout with crosswalks. A turning vehicle
  already on a crosswalk drove on into someone in front of it, a vehicle yielding to
  traffic just past a crosswalk didn't check the crosswalk on its way, and pedestrians
  stepped out beside the back of a long vehicle whose front had left the intersection.
  Vehicles now stop for them. Over 30 seeds that changed person delay by 0.6 s at most,
  for any controller on any layout. On six seeds of the town layout it moved predictive
  control from 21.8 to 24.1 s, which shows how far six seeds can swing there.
- **Preemption has to hold for a vehicle stuck in a queue:** the same test had emergency
  vehicles waiting more than a minute. Preemption used to wait until one was due within
  30 s, at its speed taken as at least 3 m/s, which one stopped in a queue more than
  90 m back never is. So it let go as the vehicle braked into a queue, the controller
  could then end the green it had asked for while the vehicle still stood there, and the
  vehicle waited out a second change. Sent in behind 20 cars per lane at a red on the
  arterial (`examples/preempt.rs`), emergency vehicles were delayed 49.7 s on average,
  against 117.6 s without preemption. Now preemption lasts while any emergency vehicle
  is on an approach or in the box, and the delay is 36.8 s. Most of the rest is the
  queue ahead moving off, since drivers here don't pull over. The two longest waits left
  in that test, 76 and 78 s, are on the single-lane town layout, behind drivers turning
  left who wait for gaps in the oncoming traffic that the same green lets through.
  Default traffic has no emergency vehicles, so the benchmarks are unchanged. In the
  2,560 stress runs below, 21,610 of 23,040 emergency vehicles got through, against
  21,599, still with no overlaps and no vehicle standing still in the box for over five
  minutes.
- **Frequent emergency vehicles find what ordinary traffic doesn't:** 2,560 half-hour
  runs on every layout and control mode, with and without one sent through every 90 s
  and with and without four times the pedestrians, gave one overlap, in a run with both.
  Where two paths merge they run close together for up to 12 m before the merge point,
  and vehicles took turns only by when they would reach that point, so a fast vehicle
  going first could overtake a slow one alongside it there. Now the order has to hold
  from where the paths come close as well. The same runs have no overlaps. Seven of
  them, as before the change, had a vehicle that stood still in the box for five to six
  and a half minutes (see *Not every long wait is a bug*, below). Over 30 seeds, no
  signal controller's person delay moved by more than 0.4 s, the all-way stop on
  major/minor moved most (by 2.6 s), and the roundabout's mean stayed at 23.4 s while
  single seeds moved both ways. The same stress also locked up the single-lane
  roundabout, in about 2% of runs, before that change and after it.
- **The roundabout needed a stress test of its own:** the long runs above cover only
  signals. With an emergency vehicle sent through every 90 s, the single-lane roundabout
  gridlocked in 11 of 500 half-hour runs, and without one in 1 of 200. An entering
  vehicle could stop where its path already runs close to the ring, behind circulating
  traffic that had stopped, and the circulating vehicles behind it waited for it, until
  each vehicle on the ring waited for another. Heavier traffic found more: at 1.35× half
  the runs, and at 1.6× nine in ten, had a vehicle standing still for over five minutes
  while traffic moved around it. A vehicle still on its entry ranked as entering at the
  next merge around the ring as well, and yielded there to the vehicles entering, so a
  truck, longer than the ring between two entries, had to wait at its entry until both
  merges were clear. Now an entering vehicle waits at its entry until the ring has room
  for all of it beyond the merge, and ranks as circulating at the merges it will reach on
  the ring. Either change alone still leaves runs stuck; together, no vehicle in those
  runs stands still on the roundabout for more than 68 s with emergency vehicles, or
  129 s at 1.6× traffic, against over 26 minutes before. Over 30 seeds of default
  traffic, the roundabout's person delay fell from 23.4 to 17.8 s and its worst seed's
  from 74 to 30.5 s; five seeds got worse, by 3.8 s at most.
- **Not every long wait is a bug:** the seven vehicles above that stood in the box for
  five minutes or more were all turning left from the major road at a two-way stop. Two
  were trucks stranded across the oncoming lanes. Each had accepted a gap long enough
  for its whole turn but committed only to the first lane it crossed, so a vehicle that
  came into view after it set off made it stop in that lane, where a truck needs a
  longer gap in the far lane than its 140 m view can show. A vehicle that yields as it
  turns now commits to the rest of the turn once it accepts a gap for all of it, and
  before it sets off it also waits for anyone crossing, or about to cross, the street it
  turns into. The other five waited short of the oncoming lanes for a gap the model's
  drivers then accepted: at least 6.8 s before the near lane, 7.6 s before the far one
  and 9.3 s before an oncoming right turn joins their street, plus a reaction time of
  0.6–1.2 s, and about 12 to 14 s in a truck, where the Highway Capacity Manual's
  critical headway for the turn is 4.1 s, or about 6 s for a heavy vehicle. One car
  waited 320 s while 76 vehicles came the other way: 20 of the gaps between them were
  6 s or longer and three over 14 s, but side-street left turns, oncoming right turns
  and pedestrians used up the long ones, and the model let a side-street left turn take
  a gap ahead of a major-road left turn waiting for one, though the Manual ranks the
  major-road turn first. Those waits came from the model's caution, not a deadlock, and
  calibrating gap acceptance (below) removed them: none of the 2,560 runs now has a
  vehicle standing still in the box for over five minutes. The longest standstill,
  241 s, is a truck waiting short of the oncoming lanes to turn left from the major road
  of the T-intersection, in a run with four times the pedestrians. In the 160 runs with a
  two-way stop, no left turn from the major road stands across the oncoming lanes for
  more than 17 s, against 84 s with only the change to its commitments and 305 s before
  that. Heavier traffic still finds long waits of the same kind: at 1.6× traffic, 5 of
  the 40 half-hour runs with a two-way stop (`stress 1 20 all 1.6`) have a bus or truck
  that waits five to ten minutes short of the oncoming lanes to turn left from the major
  road, where it needs a gap of 8 to 9 s, against 28 runs before the calibration. None
  of them is a deadlock.
- **Gap acceptance is now measured and calibrated:** a maximum-likelihood fit to the
  gaps drivers let pass and the ones they take gives their critical headway, the gap
  they are as likely to take as to let pass, as in a field study (`examples/gaps.rs`,
  eight hour-long runs per layout and control mode, without pedestrians). Drivers who
  yielded to traffic with the right of way used to leave 2 s both ways and expect to
  pull away sluggishly, so a car turning left from the major road at a two-way stop
  needed 7.6 s on the T-intersection and 8.5 s on major/minor, where the Manual says
  4.1 s, side-street drivers needed 8.5 to 9.1 s, against 6.5 to 7.5 s, and left turns
  on a permissive indication 6.8 to 8.5 s (10.9 s from major/minor's minor street),
  against 4.5 s. Drivers who judge gaps (at a two-way stop, or turning left on a
  permissive indication or right on red) now leave 0.8 s going ahead of a vehicle with
  the right of way at a crossing and 0.4 s going behind it (2.0 and 0.8 s where paths
  merge), and count on 85% of their acceleration, about what car following gives, and on
  reacting before they move off. The vehicle with the right of way leaves them 0.2 s
  less, so near the limit the two agree on who goes first. Two drivers who both judge
  gaps, such as a side-street driver and a major-road left turn, leave each other the
  plain 0.8 s (1.2 s at a merge) both ways, where they used to leave 2 s. A left turn
  from the major road that is waiting for a gap now goes before the side street, as the
  Manual has it, and once in the oncoming lanes it clears them instead of stopping to
  yield. It now needs 5.5 s on both layouts, side-street drivers 5.9 to 7.5 s and
  permissive left turns 5.7 to 7.5 s (9.7 s from major/minor's minor street); a test
  keeps the major-road left within 2 s of the Manual. The difference left comes mostly
  from the 3.4 s a car takes here to cross the near oncoming lane from a standstill.
  Queued drivers still follow each other into a gap at 1.6 to 3.8 times the Manual's
  follow-up headway, and for the major-road left more slowly than before (5.0 against
  3.8 s on the T-intersection).
- **Calibrating it changed the comparisons:** over 30 seeds the two-way stop's person
  delay fell from 171.8 to 115.3 s on major/minor and from 40.4 to 27.5 s on the
  T-intersection. Every signal controller's fell too, by 0.5 to 9.1 s with the networks
  unchanged, and by at least 4.1 s on the single-lane town layout, where a left turn
  waiting for a gap holds up the lane behind it. Predictive control gained most on
  major/minor (23.2 → 16.6 s), so the Hybrid's lead there, its only one larger than the
  noise, is gone, and so is Deep RL's on the T-intersection. Retraining both networks on
  the calibrated model brought neither lead back (§2). The all-way stop and the
  roundabout, which the calibration leaves alone, moved by 0.7 s at most. In the 48
  hours of `examples/gaps.rs`, a vehicle with the right of way braked harder than
  3 m/s² for one judging gaps 11 times, against 25 before, 22 of them from the first
  fault below.
- **Tighter margins bring out older faults:** at the T-intersection's signals, a driver
  who had yet to decide whether to stop for a yellow counted as one that would stop
  until it decided, so a left turn could set off in front of it, and it then braked
  hard: 22 times in eight hours before, 53 with the tighter margins, and never now that
  such a driver ranks as it is about to decide. And a vehicle slowing in a crossing zone
  for someone on the crosswalk ahead was expected to clear the zone at its usual pace,
  so in one stress run a vehicle went in behind it and the two overlapped. It now counts
  as one that won't clear, as a vehicle waiting for another already did.
- **Making things fail finds what normal running hides:** the cooperative crossing's
  monitors read zero through every failure-free run, and so could not show that the
  fail-safe gap falls a few decimeters short for a vehicle that is still slowing down.
  The first failures made on purpose did.

## Caveats

- **Safety measures:** drivers in the model are law-abiding, so safety is measured through
  near-misses and overlaps, not crashes.
- **Corridors:** they run east–west only.
- **Training data:** the RL agents were trained on simulated traffic only.
- **Gap acceptance:** it is calibrated at two-way stops and on permissive indications,
  but still cautious. A car turning left from the major road at a two-way stop needs
  about 5.5 s, where the Highway Capacity Manual's critical headway is 4.1 s, left turns
  on a permissive indication need 5.7 to 7.5 s (9.7 s from the minor street of
  major/minor), against 4.5 s, and queued drivers follow each other into a gap at 1.6 to
  3.8 times the Manual's follow-up headway, so two-way stops and permissive left turns
  still look somewhat worse than they would in the field (§9). Right turns on red use
  the same margins but aren't measured, and the gaps drivers accept entering the
  roundabout aren't calibrated.
- **Cooperative crossing:** every vehicle is automated and connected, and human drivers,
  lost messages and a failed manager are not modeled. A vehicle that is made to fail
  stops on its path and reports it at once; vehicles that touch stop where they touch,
  and nothing more of a collision is simulated.

## Reproducing the results

```bash
cargo run --release --bin bench all 20 6               # sections 1 and 7
cargo run --release --bin bench all 20 30              # sections 1, 2, 7 and 9 (paired differences)
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
cargo run --release --example preempt                  # section 9 (emergency vehicles behind a queue)
cargo run --release --example stress 1 20 all 1 ev,peds,both   # section 9 (emergency vehicles on every layout)
cargo run --release --example stress 1 500 roundabout 1 ev     # section 9 (the roundabout with emergency vehicles)
cargo run --release --example stress 1 200 roundabout 1        # section 9 (the roundabout without them)
cargo run --release --example stress 1 200 roundabout 1.35     # section 9 (the roundabout at 1.35× traffic)
cargo run --release --example stress 1 200 roundabout 1.6      # section 9 (the roundabout at 1.6× traffic)
cargo run --release --example stress 1 20 all 1.35             # section 9 (every layout at 1.35× traffic)
cargo run --release --example stress 1 20 all 1.6              # section 9 (every layout at 1.6× traffic)
cargo run --release --example satflow                  # calibration
cargo run --release --example gaps                     # calibration (gap acceptance, section 9)
cargo run --release --example sweep 4 1.35             # safety sweep
```
