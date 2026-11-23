# Accessibility

Intersection Lab is a visual simulator: its main content is an animated map of traffic
and a set of live charts. That makes some of it hard or impossible to use without sight,
and some of it hard to use without a mouse. This page says what works today, what
doesn't, and which results are available without the visual app.

Last reviewed: November 2026, for the browser version and the release after 0.8.0.

## Results available as text

The main results can be produced from the command line as plain-text tables, which work
with screen readers, braille displays and any terminal:

```bash
cargo run --release --bin bench all 20 6        # every automatic strategy on every layout
cargo run --release --bin bench sensors all 20 4 # sensor study
cargo run --release --example corridor           # corridor study
cargo run --release --example lpi                # pedestrian safety treatments
cargo run --release --example cooperative        # signal-free cooperative crossing
cargo run --release --example failures           # failures at the cooperative crossing
```

These commands print the average and 95th-percentile delay, throughput, stops, person
delay, pedestrian waits and near misses, a corridor's end-to-end trips and the safety
counts. The first also gives each strategy's difference from predictive control, with
its 95% interval. Given `--csv FILE`, both `bench` commands also write every run to a
CSV file, which a spreadsheet or a screen reader can step through cell by cell. The
`failures` example prints how often vehicles touched after a failure and how fast, the
emergency stops, the crossings withdrawn and the delay a failure adds. These commands
don't print everything the app shows. Some numbers appear only in the app, such as CO₂
per vehicle, the level of service, the breakdowns by approach, movement and vehicle
class, the charts and a selected vehicle's own figures, and only the `cooperative`
example prints the maximum delay.

The [README](README.md#results) and [docs/insights.md](docs/insights.md) report the
findings as text and tables, and every image in them has a text description.

## What works

* **Keyboard:** Tab and Shift+Tab move through the controls, with a visible focus
  outline. Enter or Space activates the focused control; on a dropdown it opens the list,
  the arrow keys move through it, Enter chooses, and Escape closes it. Escape also closes
  the About and benchmark windows, and takes keyboard focus off a control. With no
  control focused, Space pauses and resumes the simulation, and under manual control the
  number keys choose a stage and N chooses the next one.
* **Zoom:** in the desktop app, Cmd or Ctrl with +, − and 0 enlarge, shrink and reset
  the whole interface. In the browser, the browser's own zoom does the same.
* **Motion:** the simulation can be paused at any time, and its speed set from ×0.5 to
  ×100. Flashing lights (flashing yellow arrows, flashing DON'T WALK, turn signals, the
  light bars of emergency vehicles and the hazard lights of a disabled vehicle) blink
  about twice a second at every simulation speed, below the three-flashes-per-second
  limit for photosensitive seizures. The red ring around a disabled vehicle pulses more
  slowly, a little under once a second. Under manual control, the view of what the
  sensors see fades out a vehicle the sensors lose over half a second rather than
  dropping it, so vehicles detected on and off don't blink.
* **Screen readers, desktop only:** the desktop app exposes its controls to the
  operating system's accessibility interface. On macOS it presents buttons, checkboxes,
  sliders, dropdowns and the text of the results panel, and dropdowns and number fields
  are named by the labels shown beside them. Windows and Linux use the same library but
  haven't been checked.
* **Text:** text in the panels and charts is at least 11 px and meets the 4.5:1 contrast
  minimum against its background.

## Known limitations

* **The map and charts are images only.** A screen reader can't describe the vehicles,
  signals, queues, signal timing diagram, line charts, or the cooperative crossing's
  timeline of crossings and histogram of crossing gaps. The numbers in the results panel
  are readable, and so is the sentence it gives for each failure at the cooperative
  crossing.
* **No screen-reader support in the browser version.** It draws everything on a single
  canvas, so assistive technology finds nothing to read. Use the desktop app or the
  command-line tools instead.
* **Pointer-only tasks:** panning and zooming the map, selecting a vehicle, placing a
  pedestrian at a crosswalk corner and the charts' hover readouts need a pointer. So
  does choosing which vehicle of the cooperative crossing fails; the *Stall one in the
  box* and *Stall one at the worst moment* buttons make one fail from the keyboard, and
  what follows is reported as text in the results panel. Under manual control, giving
  an approach green by clicking it needs a pointer too, but the stage buttons and the
  number keys choose any stage without one, and the panel says in text what the signal
  will do next. Which movements and crosswalks a stage serves is shown when the pointer
  hovers over its button.
* **No shortcuts between panels:** reaching the settings means tabbing through the whole
  top toolbar, about two dozen controls.
* **Color:** the *Color by delay* and *Color by speed* map views use color alone. So do
  the dots on the vehicles of the cooperative crossing, which show whether a vehicle has
  its trajectory, is waiting for one, is disabled or has stopped for a disabled vehicle;
  a disabled vehicle also has a ring and hazard lights. The side-by-side table marks the
  better result in green, though the numbers are always shown.
* **Dark theme only**, and the app doesn't follow the system's light or high-contrast
  settings.
* **Large screens:** the desktop window needs at least 1100 × 700 points, and the
  browser version isn't designed for phones.
* **Fast changes at high speed:** at ×30 to ×100, small signal lights and detector
  markers change color several times a second, and with *random stalls / h* turned up,
  the rings that mark disabled vehicles and contacts at the cooperative crossing can
  come and go as fast.

## Supported environments

* Desktop: macOS 11 or later, Windows 10 and 11, and Linux (glibc 2.35 or later).
* Browser: a current desktop browser; tested in Chromium. Phones and tablets aren't supported.

## Reporting a barrier

If something gets in your way, please
[open an issue](https://github.com/schreck61/intersection-lab/issues/new?template=bug_report.yml)
and choose *Accessibility barrier* as the kind of problem, or email the maintainer at
schreck@gmail.com. Say what you were trying to do, what assistive technology or settings
you use, and what happened. Accessibility reports are treated as bugs.
