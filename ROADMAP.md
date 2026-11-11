# Roadmap

This is what Intersection Lab plans to work on through the end of 2027, and what it
doesn't plan to take on. It's a statement of direction, not a promise: this is a
one-maintainer project, and plans can change with what the experiments find, what users
ask for and the time available. The maintainer reviews it at least once a year and
whenever plans change.

Last reviewed: November 2026, version 0.5.1.

## Next

* **Transit signal priority.** Buses get a green extension or an early green, optionally
  only when they're running late, so the benchmarks can weigh what priority saves bus
  riders against what it costs everyone else.
* **Pedestrian scramble and slower walkers.** An exclusive pedestrian phase, with
  pedestrians crossing in every direction at once, to compare with the leading
  pedestrian interval and the no-turn-on-red sign. An option for a share of slower
  walkers, such as older pedestrians and people using mobility aids, will test whether
  clearance times leave anyone in the road.
* **Safety measures for vehicle conflicts.** Time-to-collision and post-encroachment
  time, the standard surrogate safety measures, alongside the overlaps and pedestrian
  near-misses measured today.
* **Bring your own controller.** An example and guide showing how to implement the
  `Controller` trait in your own crate and benchmark it against the built-in controllers
  on identical arrivals.
* **Export results as CSV.** Key results, per-vehicle trips and benchmark tables, as a
  download from the app and from `bench`.

## Later in 2027

* **Coordinated adaptive corridors.** Adaptive signals along a corridor that plan
  together, as SURTRAC does, compared with coordinated-actuated control, the usual setup
  on US arterials. The [corridor study](docs/insights.md#4-corridors) found that telling
  each signal what its upstream neighbor released isn't enough, and that the real gains
  need coordinated plans. This will also test whether tuning the planner with
  reinforcement learning, which helps at a single intersection, helps along a corridor
  too.
* **Mixed traffic and failures at the cooperative crossing.** Human drivers mixed in with
  automated vehicles at different shares, and lost messages, a failed intersection
  manager and a vehicle stalled in the box. These are the crossing's main untested
  assumptions.
* **Save, load and share scenarios.** A scenario's layout, demand, sensors and control
  settings saved to a file and loaded again, and in the browser version a link that
  carries the scenario itself. Scenario files and links are a new kind of untrusted
  input, so they'll get the same typed parsing, checks and fuzzing as weights files, and
  [SECURITY.md](SECURITY.md) and the [assurance case](docs/assurance-case.md) will be
  updated to match.
* **Accessibility.** Work through the known limitations listed in
  [ACCESSIBILITY.md](ACCESSIBILITY.md): keyboard ways to select a vehicle, place sensors
  and choose a crosswalk corner, shortcuts between panels, map views that don't rely on
  color alone, and a light or high-contrast theme.

## If time allows

* **A two-lane roundabout,** to test whether the roundabout's strong results at moderate
  volumes hold at arterial volumes.
* **Bicycles:** cyclists in bike lanes, bike boxes, and right-hook conflicts with turning
  vehicles.
* **Small grids:** corridors that run north–south as well as east–west, then a 2×2 grid
  of intersections.
* **Importing traffic counts:** demand set from turning-movement counts in a CSV file, to
  replay a real intersection's peak hour.

## Not planned

To keep the project focused, small and safe to run, it doesn't plan to take on:

* **Large networks:** city-scale or regional networks, freeways, ramps and interchanges.
  Network simulators such as SUMO, VISSIM and Aimsun cover these.
* **Designing real intersections.** Results are for exploring, teaching and research,
  not a substitute for an engineering study.
* **Connections to the outside world:** real signal controllers, field hardware or live
  data feeds, and network features such as accounts, cloud runs or telemetry. The app
  makes no network connections of its own, and that won't change.
* **Plugins, scripting or loading code at run time.** Your own controller is compiled
  in, as described under *Next*.
* **Driver errors and crashes.** Drivers stay law-abiding, and safety is measured with
  surrogates.
* **Python bindings,** such as a Gymnasium environment. They would add a second
  language, package and supply chain to build, sign and publish.
* **Translations and other countries' rules.** The interface stays in English, and the
  traffic rules follow US practice (the MUTCD), with vehicles driving on the right.
* **3D graphics, phones and tablets.** The app stays a 2D, top-down view on desktop
  computers and in desktop browsers.
* **A stable Rust API.** The crate stays at 0.x, so its API can change between minor
  versions.

## Suggesting a change

To propose something for the roadmap, including something listed as not planned, open
an [issue](https://github.com/schreck61/intersection-lab/issues). Help with anything
under *Next*, *Later in 2027* or *If time allows* is welcome; as
[CONTRIBUTING.md](CONTRIBUTING.md#before-you-start) asks, please open an issue before
writing much code, so the approach can be agreed first.
