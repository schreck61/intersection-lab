# Accessibility

Intersection Lab is a visual simulator: its main content is an animated map of traffic
and a set of live charts. That makes some of it hard or impossible to use without sight,
and some of it hard to use without a mouse. This page says what works today, what
doesn't, and how to get every result without the visual app.

Last reviewed: November 2026, version 0.3.0.

## Every result is available as text

Everything the app measures can be produced from the command line as plain-text tables,
which work with screen readers, braille displays and any terminal:

```bash
cargo run --release --bin bench all 20 6        # every controller on every layout
cargo run --release --bin bench sensors all 20 4 # sensor study
cargo run --release --example corridor           # corridor study
cargo run --release --example lpi                # pedestrian safety treatments
```

The [README](README.md#results) and [docs/insights.md](docs/insights.md) report the
findings as text and tables, and every image in them has a text description.

## What works

* **Keyboard:** Tab and Shift+Tab move through the controls, with a visible focus
  outline, and Enter activates the focused button. Space pauses and resumes the
  simulation from anywhere.
* **Zoom:** in the desktop app, Cmd or Ctrl with +, − and 0 enlarge, shrink and reset
  the whole interface. In the browser, the browser's own zoom does the same.
* **Motion:** the simulation can be paused at any time, and its speed set from ×0.5 to
  ×100. Flashing signals (flashing yellow arrows, flashing DON'T WALK, turn signals) blink
  about twice a second at every simulation speed, below the three-flashes-per-second
  limit for photosensitive seizures.
* **Screen readers, desktop only:** the desktop app exposes its controls to the
  operating system's accessibility interface. On macOS it presents buttons, checkboxes,
  sliders, dropdowns and the text of the results panel. Windows and Linux use the same
  library but haven't been checked.

## Known limitations

* **The map and charts are images only.** A screen reader can't describe the vehicles,
  signals, queues, signal timing diagram or line charts. The numbers in the results panel
  are readable.
* **No screen-reader support in the browser version.** It draws everything on a single
  canvas, so assistive technology finds nothing to read. Use the desktop app or the
  command-line tools instead.
* **Some controls lack names.** On macOS, the dropdowns (for example the intersection and
  controller choices) have no accessible name; their labels are separate text.
* **Mouse-only tasks:** selecting a vehicle, placing sensors and choosing a crosswalk
  corner need a pointer.
* **Keyboard gaps:** there are no shortcuts to jump between panels, so reaching the
  settings means tabbing through the whole top toolbar, about two dozen controls.
  Dropdowns showed no focus outline and didn't open from the keyboard in testing, and
  Escape doesn't close the About window.
* **Color:** the *Color by delay* and *Color by speed* map views use color alone. The
  side-by-side table marks the better result in green, though the numbers are always
  shown.
* **Small, low-contrast text:** some hints and chart labels are small or gray on the
  dark background, below recommended contrast (about 3.8:1 where 4.5:1 is recommended).
* **Dark theme only**, and the app doesn't follow the system's light or high-contrast
  settings.
* **Large screens:** the desktop window needs at least 1100 × 700 points, and the
  browser version isn't designed for phones.
* **Fast changes at high speed:** at ×30 to ×100, small signal lights and detector
  markers change color several times a second.

## Supported environments

* Desktop: macOS 11 or later, Windows 10 and 11, and Linux (glibc 2.35 or later).
* Browser: a current desktop browser; tested in Chromium. Phones and tablets aren't supported.

## Reporting a barrier

If something gets in your way, please
[open an issue](https://github.com/schreck61/intersection-lab/issues/new?template=bug_report.yml)
and choose *Accessibility barrier* as the kind of problem, or email the maintainer at
schreck@gmail.com. Say what you were trying to do, what assistive technology or settings
you use, and what happened. Accessibility reports are treated as bugs.
