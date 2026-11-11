// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The app, driven the way a person drives it: through the controls its accessibility
//! tree names (what a screen reader sees), the keyboard, and the pointer on the map. It
//! runs without a window (egui_kittest), so the tests read what the panels say and what
//! the map paints as text, not how anything looks.
//!
//! While it runs, the simulation advances with real time, so tests that count
//! simulated seconds pause it first and advance it with the Step button.

use egui::accesskit::{Role, Toggled};
use egui::{Event, Modifiers, PointerButton, Pos2, Rect, Shape, Vec2, pos2, vec2};
use egui_kittest::Harness;
use egui_kittest::kittest::{NodeT, Queryable};
use intersection_lab::control::CtrlKind;
use intersection_lab::gui::App;
use intersection_lab::gui::render::Camera;
use intersection_lab::sim::ControlMode;
use intersection_lab::sim::LeftMode;
use intersection_lab::sim::coop::{GapPolicy, Scheduling};
use intersection_lab::sim::demand::{DemandConfig, Pattern};
use intersection_lab::sim::geom::{V2, v2};
use intersection_lab::sim::network::{LayoutKind, Network, Turn};
use intersection_lab::sim::sensors::Conditions;
use intersection_lab::sim::vtypes::{ALL_KINDS, VKind};
use std::time::{Duration, Instant};

type Window = Harness<'static, App>;

/// The app as it opens, in a window of its default size.
fn open() -> Window {
    Harness::builder()
        .with_size([1600.0, 1000.0])
        // The desktop app draws emoji with the system's fonts, which a test doesn't load.
        .allow_missing_glyphs()
        .build_eframe(|cc| App::new(cc))
}

/// Every control has a name a screen reader can say (see ACCESSIBILITY.md).
#[track_caller]
fn assert_named(w: &Window) {
    let unnamed = w.unnamed_widgets();
    assert!(unnamed.is_empty(), "controls without a name: {unnamed:#?}");
}

/// Activate the control named `label` the way a screen reader does, which works whether
/// or not it's scrolled into view, and draw the result.
#[track_caller]
fn press(w: &mut Window, label: &str) {
    w.get_by_label(label).click_accesskit();
    w.run_steps(2);
}

/// Pick `option` from the drop-down list named `list`.
#[track_caller]
fn choose(w: &mut Window, list: &str, option: &str) {
    w.get_by_role_and_label(Role::ComboBox, list).click_accesskit();
    w.run_steps(2);
    w.get_by_role(Role::ListBox).get_by_label(option).click_accesskit();
    w.run_steps(2);
    // Choosing with a screen reader leaves the list open, as it would for a keyboard user
    // until Escape.
    w.key_press(egui::Key::Escape);
    w.run_steps(2);
    assert!(w.query_by_role(Role::ListBox).is_none(), "the {list} list stayed open");
}

/// What the drop-down list named `list` shows as chosen.
#[track_caller]
fn chosen(w: &Window, list: &str) -> String {
    w.get_by_role_and_label(Role::ComboBox, list).value().unwrap_or_default()
}

/// Move the slider named `slider` to `value`.
#[track_caller]
fn slide(w: &mut Window, slider: &str, value: f64) {
    w.get_by_role_and_label(Role::Slider, slider).set_value_accesskit(value);
    w.run_steps(2);
}

/// Whether the checkbox named `label` is ticked.
#[track_caller]
fn ticked(w: &Window, label: &str) -> bool {
    w.get_by_role_and_label(Role::CheckBox, label).accesskit_node().toggled() == Some(Toggled::True)
}

/// Tick or clear the checkbox named `label`.
#[track_caller]
fn tick(w: &mut Window, label: &str, on: bool) {
    if ticked(w, label) != on {
        press(w, label);
    }
    assert_eq!(ticked(w, label), on, "{label}");
}

/// The text of every label in the panels and windows.
fn texts(w: &Window) -> Vec<String> {
    w.query_all_by(|n| n.role() == Role::Label).filter_map(|n| n.value()).collect()
}

/// Whether a label reads exactly `text`.
fn has(w: &Window, text: &str) -> bool {
    texts(w).iter().any(|t| t == text)
}

/// Whether a label contains `text`.
fn shows(w: &Window, text: &str) -> bool {
    texts(w).iter().any(|t| t.contains(text))
}

/// Text the last frame painted, on the map and in the charts.
fn painted(w: &Window) -> Vec<String> {
    fn walk(shape: &Shape, out: &mut Vec<String>) {
        match shape {
            Shape::Vec(shapes) => shapes.iter().for_each(|s| walk(s, out)),
            Shape::Text(t) => out.push(t.galley.text().to_owned()),
            _ => {}
        }
    }
    let mut out = vec![];
    for c in &w.output().shapes {
        walk(&c.shape, &mut out);
    }
    out
}

/// Whether the last frame painted text containing `text`.
fn paints(w: &Window, text: &str) -> bool {
    painted(w).iter().any(|t| t.contains(text))
}

/// The status line at the top left of the map: clock, speed and strategy.
#[track_caller]
fn status(w: &Window) -> String {
    painted(w)
        .into_iter()
        .find(|t| t.len() > 8 && t.as_bytes()[2] == b':' && t.as_bytes()[5] == b':')
        .expect("the map's status line")
}

/// The simulated time on the map's clock, in seconds.
#[track_caller]
fn clock(w: &Window) -> u32 {
    let line = status(w);
    let field = |r: std::ops::Range<usize>| line[r].parse::<u32>().expect("hh:mm:ss");
    field(0..2) * 3600 + field(3..5) * 60 + field(6..8)
}

/// A count from the line under the results ("3 completed · 12 in network · …").
#[track_caller]
fn count(w: &Window, what: &str) -> usize {
    let line = texts(w).into_iter().find(|t| t.contains(" waiting to enter · ")).expect("the counts line");
    let part = line.split(" · ").find(|p| p.ends_with(what)).unwrap_or_else(|| panic!("no {what} in {line}"));
    part.split(' ').next().unwrap().parse().unwrap()
}

#[track_caller]
fn pause(w: &mut Window) {
    press(w, "⏸ Pause");
    assert!(w.query_by_label("▶ Run").is_some(), "the button offers to run again");
    assert!(status(w).contains("⏸ paused"));
}

/// Advance the paused simulation `seconds` seconds with the Step button.
#[track_caller]
fn step(w: &mut Window, seconds: usize) {
    for _ in 0..seconds {
        w.get_by_label("⏭ Step").click_accesskit();
        w.step();
    }
    w.step();
}

/// The map: the only control that's neither labeled nor a container.
#[track_caller]
fn map(w: &Window) -> Rect {
    w.query_all_by_role(Role::Unknown)
        .map(|n| n.rect())
        .max_by(|a, b| a.area().total_cmp(&b.area()))
        .expect("the map")
}

/// Where the map draws point `p` (meters, centered on the first intersection) with the
/// view as Fit view leaves it: the intersection in the middle, scaled so the approaches
/// fit 55 m beyond the farthest stop line (80 m on the cooperative crossing). With the
/// comparison on, A has the left half.
#[track_caller]
fn on_map(w: &Window, net: &Network, p: V2, compare: bool) -> Pos2 {
    let rect = map(w);
    let margin = if net.coop.is_some() { 80.0 } else { 55.0 };
    let extent = net.legs.iter().map(|l| l.stop_dist).fold(0.0, f64::max) + margin;
    let width = if compare { rect.width() * 0.5 } else { rect.width() };
    let scale = (width / (2.0 * extent) as f32).min(rect.height() / (2.0 * extent as f32)).max(0.6);
    let half =
        if compare { Rect::from_min_max(rect.min, pos2(rect.center().x - 1.0, rect.max.y)) } else { rect };
    Camera { center: v2(0.0, 0.0), scale, rect: half }.to_screen(p)
}

/// A click of the pointer at `pos`.
#[track_caller]
fn click_at(w: &mut Window, pos: Pos2) {
    w.event(Event::PointerMoved(pos));
    for pressed in [true, false] {
        w.event(Event::PointerButton {
            pos,
            button: PointerButton::Primary,
            pressed,
            modifiers: Modifiers::NONE,
        });
    }
    w.run_steps(2);
}

/// On an inbound lane of `leg`, `r` meters from the center.
fn approach(net: &Network, leg: usize, r: f64) -> V2 {
    let l = &net.legs[leg];
    l.d * r + l.in_side() * (l.in_width * 0.5)
}

/// Points along every inbound lane where vehicles wait, nearest the stop lines first.
fn queue_spots(net: &Network) -> Vec<V2> {
    let mut spots = vec![];
    for k in 0..14 {
        for leg in &net.legs {
            for &lane in &leg.in_lanes {
                let path = &net.links[lane].path;
                let s = path.length - 2.0 - 3.0 * k as f64;
                if s > 0.0 {
                    spots.push(path.point(s));
                }
            }
        }
    }
    spots
}

/// Click along the queues until a vehicle is selected; whether one was.
#[track_caller]
fn select_a_vehicle(w: &mut Window, net: &Network) -> bool {
    for p in queue_spots(net) {
        let pos = on_map(w, net, p, false);
        click_at(w, pos);
        if has(w, "doing") {
            return true;
        }
    }
    false
}

#[test]
fn opens_with_every_panel_and_every_control_named() {
    let w = open();
    for heading in [
        "Scenario",
        "Demand",
        "Sensors",
        "Operations",
        "View",
        "Vehicle experience",
        "Controller brain",
        "Selected vehicle",
        "Benchmark",
        "Signal timing (last 3 min)",
    ] {
        assert!(has(&w, heading), "no {heading}");
    }
    assert_eq!(chosen(&w, "Intersection"), LayoutKind::Arterial4.name());
    assert_eq!(chosen(&w, "Control"), ControlMode::Signal.name());
    assert_eq!(chosen(&w, "Controller"), CtrlKind::Predictive.name());
    assert!(shows(&w, CtrlKind::Predictive.description()));
    assert!(shows(&w, "Click a vehicle to inspect it."));
    assert!(status(&w).ends_with("Predictive"), "{}", status(&w));
    assert!(paints(&w, "Stage: "), "the signal's stage is on the map");
    assert_named(&w);
}

#[test]
fn space_pauses_and_step_advances_one_second() {
    let mut w = open();
    // With nothing focused, Space pauses and resumes.
    w.key_press(egui::Key::Space);
    w.run_steps(2);
    assert!(w.query_by_label("▶ Run").is_some());
    let t = clock(&w);
    step(&mut w, 3);
    assert_eq!(clock(&w), t + 3);
    // Paused, the clock stands still however many frames pass.
    w.run_steps(20);
    assert_eq!(clock(&w), t + 3);
    w.key_press(egui::Key::Space);
    w.run_steps(2);
    assert!(w.query_by_label("⏸ Pause").is_some());

    // Speeds, and Reset back to the start.
    for speed in ["×0.5", "×2", "×5", "×10", "×30", "×100", "×1"] {
        press(&mut w, speed);
        assert!(status(&w).contains(&format!("{speed}   ")), "{}", status(&w));
    }
    pause(&mut w);
    step(&mut w, 2);
    press(&mut w, "⟲ Reset");
    assert_eq!(clock(&w), 0);
    press(&mut w, "Restart");
    assert_eq!(clock(&w), 0);
}

#[test]
fn every_layout_control_mode_and_controller_runs() {
    let mut w = open();
    pause(&mut w);
    for kind in LayoutKind::ALL {
        choose(&mut w, "Intersection", kind.name());
        assert_eq!(chosen(&w, "Intersection"), kind.name());
        for mode in ControlMode::options(kind) {
            if ControlMode::options(kind).len() > 1 {
                choose(&mut w, "Control", mode.name());
            }
            assert_eq!(chosen(&w, "Control"), mode.name());
            if mode == ControlMode::Signal {
                for c in CtrlKind::ALL {
                    choose(&mut w, "Controller", c.name());
                    assert_eq!(chosen(&w, "Controller"), c.name());
                    assert!(shows(&w, c.description()));
                    step(&mut w, 2);
                    assert!(status(&w).ends_with(c.short()), "{}", status(&w));
                    assert!(has(&w, "Controller brain"));
                }
                // Each change was announced at the bottom of the window.
                assert!(shows(&w, "Controller switched live to"));
            } else {
                assert!(w.query_by_role_and_label(Role::ComboBox, "Controller").is_none());
                step(&mut w, 3);
                let strategy = match mode {
                    ControlMode::Cooperative => {
                        format!("Intersection manager · {} gaps", GapPolicy::FailSafe.label())
                    }
                    _ => mode.name().to_string(),
                };
                assert!(status(&w).ends_with(&strategy), "{}", status(&w));
            }
            assert_eq!(has(&w, "Sensors"), kind != LayoutKind::Cooperative);
            assert_eq!(has(&w, "Intersection manager"), kind == LayoutKind::Cooperative);
            assert_named(&w);
        }
        assert!(shows(&w, "safety overlaps: 0"), "vehicles overlapped on {kind:?}");
    }
}

#[test]
fn the_cooperative_crossing_shows_its_intersection_manager() {
    let mut w = open();
    pause(&mut w);
    choose(&mut w, "Intersection", LayoutKind::Cooperative.name());
    step(&mut w, 30);
    // Settings, then what the manager is doing.
    assert!(
        w.query_by_role_and_label(Role::Slider, "signals along main road").is_none(),
        "it always stands alone"
    );
    assert!(shows(&w, "Stop-line crossings by movement"));
    assert!(shows(&w, "Serving "));
    assert!(shows(&w, "Phases in service order"));
    assert!(shows(&w, "Safety monitors (what vehicles actually did)"));
    assert!(paints(&w, "vehicles scheduled"), "the manager's state is on the map");
    for p in GapPolicy::ALL {
        let before = chosen(&w, "Safety gap");
        choose(&mut w, "Safety gap", p.name());
        assert_eq!(chosen(&w, "Safety gap"), p.name());
        if before != p.name() {
            assert!(shows(&w, &format!("Gap policy switched live to {}", p.label())), "{:?}", texts(&w));
        }
    }
    slide(&mut w, "max cycle s", 60.0);
    slide(&mut w, "batch join window s", 2.5);
    step(&mut w, 20);
    choose(&mut w, "Scheduling", Scheduling::FirstCome.name());
    assert!(shows(&w, &format!("Scheduling switched live to {}", Scheduling::FirstCome.label())));
    step(&mut w, 20);
    assert!(shows(&w, "First come, first served: each vehicle gets the earliest gap that fits"));
    assert!(paints(&w, "First come, first served"));
    // The batch settings don't apply to first come, first served.
    assert!(w.get_by_role_and_label(Role::Slider, "max cycle s").accesskit_node().is_disabled());
    choose(&mut w, "Scheduling", Scheduling::Batches.name());
    step(&mut w, 5);
    assert!(shows(&w, "safety overlaps: 0"));
    assert_named(&w);
}

#[test]
fn comparison_runs_a_second_strategy_on_the_same_arrivals() {
    let mut w = open();
    pause(&mut w);
    tick(&mut w, "Compare side by side", true);
    assert_eq!(chosen(&w, "B control"), ControlMode::Signal.name());
    assert_eq!(chosen(&w, "B controller"), CtrlKind::Fixed.short());
    choose(&mut w, "B controller", CtrlKind::Actuated.name());
    assert_eq!(chosen(&w, "B controller"), CtrlKind::Actuated.short());
    step(&mut w, 10);
    assert!(shows(&w, "Side by side (same arrivals)"));
    assert!(shows(&w, "A: Predictive") && shows(&w, "B: Actuated"));
    assert!(paints(&w, "A · Predictive") && paints(&w, "B · Actuated"), "each half of the map is labeled");
    choose(&mut w, "B control", ControlMode::AllWayStop.name());
    assert!(w.query_by_role_and_label(Role::ComboBox, "B controller").is_none());
    step(&mut w, 5);
    assert!(shows(&w, &format!("B: {}", ControlMode::AllWayStop.name())));
    assert_named(&w);

    // On the cooperative crossing, B differs in its scheduling and gaps.
    choose(&mut w, "Intersection", LayoutKind::Cooperative.name());
    assert!(ticked(&w, "Compare side by side"));
    choose(&mut w, "B scheduling", Scheduling::Batches.name());
    choose(&mut w, "B gap policy", GapPolicy::Nominal.name());
    assert_eq!(chosen(&w, "B gap policy"), format!("{} gaps", GapPolicy::Nominal.label()));
    step(&mut w, 10);
    assert!(shows(&w, &format!("B: Intersection manager · {} gaps", GapPolicy::Nominal.label())));
    choose(&mut w, "B scheduling", Scheduling::FirstCome.name());
    assert!(shows(&w, "B: First come, first served"));
    assert_named(&w);
    tick(&mut w, "Compare side by side", false);
    assert!(!shows(&w, "Side by side (same arrivals)"));
}

#[test]
fn a_corridor_shows_each_signal_and_the_green_wave() {
    let mut w = open();
    pause(&mut w);
    slide(&mut w, "signals along main road", 3.0);
    // The panels start at the middle intersection.
    assert!(shows(&w, "Intersection 2 of 3 (west → east)"));
    assert!(shows(&w, "Main-road trips end to end"));
    // Turning the comparison on restarts both sides, every signal with the panel's
    // controller: turn it on first.
    tick(&mut w, "Compare side by side", true);
    choose(&mut w, "Controller", CtrlKind::Actuated.name());
    press(&mut w, "◀");
    assert!(shows(&w, "Intersection 1 of 3"));
    // Each intersection keeps its own controller, and the panel shows the focused one's.
    assert_eq!(chosen(&w, "Controller"), CtrlKind::Predictive.name());
    press(&mut w, "▶");
    assert_eq!(chosen(&w, "Controller"), CtrlKind::Actuated.name());
    tick(&mut w, "Upstream signal shares its outflow", true);
    step(&mut w, 30);

    tick(&mut w, "Fixed-time green wave", true);
    assert!(shows(&w, "Green wave on: every signal runs fixed-time"));
    assert_eq!(chosen(&w, "Controller"), CtrlKind::Fixed.name());
    step(&mut w, 30);
    tick(&mut w, "Fixed-time green wave", false);
    assert!(shows(&w, "Green wave off: the signals run their previous controllers again"));
    assert_eq!(chosen(&w, "Controller"), CtrlKind::Actuated.name());
    press(&mut w, "◀");
    assert_eq!(chosen(&w, "Controller"), CtrlKind::Predictive.name());
    assert_named(&w);
}

#[test]
fn demand_sensor_and_operations_settings_apply() {
    let mut w = open();
    pause(&mut w);
    // Fixed-time plans are re-optimized when the demand changes.
    choose(&mut w, "Controller", CtrlKind::Fixed.name());
    slide(&mut w, "demand ×", 1.5);
    let rates = DemandConfig::default_for(&Network::build(LayoutKind::Arterial4)).rate;
    assert!(shows(&w, &format!("{:.0} veh/h arriving in all", rates.iter().sum::<f64>() * 1.5)));
    for p in Pattern::ALL {
        choose(&mut w, "Arrivals", p.name());
        assert_eq!(chosen(&w, "Arrivals"), p.name());
        step(&mut w, 2);
    }
    let southbound = w.get_by(|n| {
        n.role() == Role::DisclosureTriangle && n.label().is_some_and(|l| l.starts_with("Southbound"))
    });
    southbound.click_accesskit();
    w.run_steps(2);
    slide(&mut w, "veh/h", 900.0);
    slide(&mut w, &format!("{} share", Turn::L.name()), 0.4);
    assert!(
        w.query_by(|n| n.role() == Role::DisclosureTriangle
            && n.label().is_some_and(|l| l.starts_with("Southbound — 1350 veh/h")))
            .is_some()
    );
    press(&mut w, "Vehicle mix");
    for k in ALL_KINDS {
        slide(&mut w, k.name(), 0.2);
    }
    slide(&mut w, "peds/h", 300.0);
    step(&mut w, 10);

    for label in [
        "Inductive loops (stop-bar + advance)",
        "Intersection cameras (video analytics)",
        "Advance radar (long range)",
        "Field-realistic sensor errors",
    ] {
        tick(&mut w, label, false);
        step(&mut w, 2);
        tick(&mut w, label, true);
    }
    tick(&mut w, "Connected vehicles (V2I)", true);
    slide(&mut w, "CV penetration", 0.6);
    slide(&mut w, "advance loop m", 70.0);
    slide(&mut w, "camera range m", 100.0);
    slide(&mut w, "radar range m", 200.0);
    for c in Conditions::ALL {
        choose(&mut w, "conditions", c.name());
        assert_eq!(chosen(&w, "conditions"), c.name());
        step(&mut w, 2);
    }
    slide(&mut w, "faulty loop stations", 0.3);
    slide(&mut w, "turn-signal use", 0.5);
    step(&mut w, 30);
    tick(&mut w, "Field-realistic sensor errors", false);
    assert!(w.get_by_role_and_label(Role::ComboBox, "conditions").accesskit_node().is_disabled());

    tick(&mut w, "Emergency-vehicle preemption", false);
    tick(&mut w, "Right turn on red", false);
    assert!(
        w.get_by_role_and_label(Role::CheckBox, "…except while pedestrians are present")
            .accesskit_node()
            .is_disabled()
    );
    tick(&mut w, "Right turn on red", true);
    tick(&mut w, "…except while pedestrians are present", true);
    slide(&mut w, "leading ped interval s", 3.0);
    slide(&mut w, "max-wait guard s", 90.0);
    for m in LeftMode::ALL {
        choose(&mut w, "left turns", m.name());
        assert_eq!(chosen(&w, "left turns"), m.name());
        // Lefts that aren't flashing-yellow say which run protected only.
        assert_eq!(shows(&w, "protected-only now: "), m != LeftMode::Fya);
        step(&mut w, 5);
    }
    assert_named(&w);
}

#[test]
fn the_learning_controllers_show_what_they_learned() {
    let mut w = open();
    pause(&mut w);
    choose(&mut w, "Controller", CtrlKind::DeepRl.name());
    assert!(has(&w, "AI (Deep RL)"));
    // The shipped network makes the decisions; it learns only when asked to.
    assert!(shows(&w, "pretrained weights: "));
    assert!(!ticked(&w, "Keep learning online"));
    assert!(w.get_by_role_and_label(Role::Slider, "exploration ε").accesskit_node().is_disabled());
    tick(&mut w, "Keep learning online", true);
    slide(&mut w, "exploration ε", 0.3);
    step(&mut w, 30);
    let line = texts(&w).into_iter().find(|t| t.starts_with("decisions ")).expect("the learning figures");
    assert!(!line.starts_with("decisions 0 "), "{line}");
    assert!(shows(&w, "Q-values (expected future reward per action)"));
    assert!(shows(&w, "online learning: ON"));
    press(&mut w, "⟳ Reload pretrained");
    assert!(!ticked(&w, "Keep learning online"), "the reloaded network isn't learning");

    choose(&mut w, "Controller", CtrlKind::Hybrid.name());
    assert!(has(&w, "AI (RL-tuned predictive)"));
    tick(&mut w, "Keep learning online", true);
    slide(&mut w, "exploration ε", 0.2);
    step(&mut w, 30);
    assert!(texts(&w).iter().any(|t| t.starts_with("tuning decisions ")));
    assert_named(&w);
}

#[test]
fn every_view_overlay_and_coloring_draws() {
    let mut w = open();
    pause(&mut w);
    slide(&mut w, "peds/h", 600.0);
    tick(&mut w, "Connected vehicles (V2I)", true);
    step(&mut w, 40);
    for label in ["sensors", "detections", "conflict points", "paths", "delay labels", "night"] {
        tick(&mut w, label, true);
        w.run_steps(2);
    }
    // The legend names what the overlays draw.
    for item in ["loop detector", "camera track", "radar track", "connected vehicle"] {
        assert!(paints(&w, item), "no {item} in the legend");
    }
    for by in ["delay", "turn", "speed", "type"] {
        press(&mut w, by);
        step(&mut w, 3);
    }
    for label in ["sensors", "detections", "conflict points", "paths", "delay labels", "night"] {
        tick(&mut w, label, false);
    }
    assert!(!paints(&w, "camera track"));
    // The cooperative crossing's view shows its reservation zones instead.
    choose(&mut w, "Intersection", LayoutKind::Cooperative.name());
    tick(&mut w, "reservation zones", true);
    tick(&mut w, "paths", true);
    step(&mut w, 20);
    assert!(paints(&w, "has its trajectory (reservation)"));
    assert!(paints(&w, "pedestrian underpass"));
    // And the roundabout, the one layout left.
    choose(&mut w, "Intersection", LayoutKind::Roundabout.name());
    step(&mut w, 20);
    press(&mut w, "Fit view");
    assert_named(&w);
}

#[test]
fn vehicles_and_pedestrians_can_be_placed_by_hand() {
    let mut w = open();
    pause(&mut w);
    let net = Network::build(LayoutKind::Arterial4);
    // One of every kind, on every approach, each way it can turn.
    let waiting = count(&w, "waiting to enter");
    let mut placed = 0;
    for (k, kind) in ALL_KINDS.into_iter().enumerate() {
        let tool =
            w.get_by(|n| n.role() == Role::Button && n.label().is_some_and(|l| l.ends_with(kind.name())));
        tool.click_accesskit();
        w.run_steps(2);
        press(&mut w, ["↰ Left", "↑ Through", "↱ Right"][k % 3]);
        for leg in &net.legs {
            press(&mut w, &format!("+ {}", leg.compass.letter()));
            placed += 1;
        }
    }
    assert_eq!(count(&w, "waiting to enter"), waiting + placed);
    // The vehicle tool shows what a click places, and a click on an approach places it.
    press(&mut w, &format!("🚗 {}", VKind::Car.name()));
    press(&mut w, "↑ Through");
    let target = on_map(&w, &net, approach(&net, 0, net.legs[0].stop_dist + 25.0), false);
    w.hover_at(target);
    w.run_steps(2);
    assert!(paints(&w, "click an approach: Car Through"));
    click_at(&mut w, target);
    let message = format!(
        "{} {} queued on {} (enters 260 m upstream)",
        VKind::Car.name(),
        Turn::T.name(),
        net.legs[0].compass.bound()
    );
    assert!(shows(&w, &message), "{:?}", texts(&w));
    step(&mut w, 60);
    assert!(count(&w, "in network") > 0);
    // Choosing the tool again puts it away.
    press(&mut w, &format!("🚗 {}", VKind::Car.name()));
    assert!(w.query_by_label("+ N").is_none());

    // A pedestrian, at the corner of a crosswalk.
    press(&mut w, "🚶 Pedestrian");
    let corner = on_map(&w, &net, net.crosswalks[0].a, false);
    w.hover_at(corner);
    w.run_steps(2);
    assert!(paints(&w, "click a crosswalk corner"));
    let peds = count(&w, "peds");
    click_at(&mut w, corner);
    assert_eq!(count(&w, "peds"), peds + 1);
    press(&mut w, "🚶 Pedestrian");
    step(&mut w, 60);
    assert!(shows(&w, "safety overlaps: 0"), "vehicles overlapped");
}

#[test]
fn a_movement_an_approach_lacks_is_refused() {
    let mut w = open();
    pause(&mut w);
    choose(&mut w, "Intersection", LayoutKind::Tee.name());
    let net = Network::build(LayoutKind::Tee);
    press(&mut w, &format!("🚌 {}", VKind::Bus.name()));
    press(&mut w, "↑ Through");
    // The stem of the T has no through movement.
    let stem = net.legs.iter().find(|l| !l.turns().contains(&Turn::T)).expect("the stem");
    let before = count(&w, "waiting to enter");
    press(&mut w, &format!("+ {}", stem.compass.letter()));
    assert!(shows(&w, &format!("{} has no {} movement", stem.compass.bound(), Turn::T.name())));
    assert_eq!(count(&w, "waiting to enter"), before);
}

#[test]
fn layouts_without_crosswalks_say_why_there_are_no_pedestrians() {
    let mut w = open();
    for (kind, why) in [
        (LayoutKind::Roundabout, "No crosswalks here: the roundabout is simulated without pedestrians"),
        (LayoutKind::Cooperative, "No crosswalks here: pedestrians use underground passages"),
    ] {
        choose(&mut w, "Intersection", kind.name());
        let tool = w.get_by_label("🚶 Pedestrian");
        assert!(tool.accesskit_node().is_disabled());
        tool.hover();
        w.run_steps(6);
        assert!(has(&w, why), "{kind:?}: no tooltip {why:?}");
        assert!(w.query_by_role_and_label(Role::Slider, "peds/h").is_none());
    }
}

#[test]
fn clicking_a_vehicle_shows_what_it_is_doing() {
    let mut w = open();
    pause(&mut w);
    step(&mut w, 60);
    let net = Network::build(LayoutKind::Arterial4);
    assert!(select_a_vehicle(&mut w, &net), "no vehicle where they queue");
    for row in [
        "movement",
        "speed",
        "doing",
        "delay so far",
        "stops",
        "stopped time",
        "fuel",
        "occupants",
        "seen by",
    ] {
        assert!(has(&w, row), "no {row}");
    }
    assert!(!has(&w, "reservation"));
    // The vehicle drives on and leaves; the panel says so.
    let mut left = false;
    for _ in 0..30 {
        step(&mut w, 10);
        if shows(&w, "(left the network)") {
            left = true;
            break;
        }
    }
    assert!(left, "the vehicle never left");

    // On the cooperative crossing, a vehicle shows its reservation instead.
    choose(&mut w, "Intersection", LayoutKind::Cooperative.name());
    step(&mut w, 30);
    let coop = Network::build(LayoutKind::Cooperative);
    assert!(select_a_vehicle(&mut w, &coop), "no vehicle on the approaches");
    assert!(has(&w, "reservation"));
    assert!(!has(&w, "seen by"));
}

#[test]
fn the_map_pans_zooms_and_fits() {
    let mut w = open();
    pause(&mut w);
    let net = Network::build(LayoutKind::Arterial4);
    let center = map(&w).center();
    // Drag the map diagonally, then zoom with the wheel and with a pinch.
    let to = center + vec2(250.0, 250.0);
    w.hover_at(center);
    w.step();
    w.drag_at(center);
    w.step();
    w.hover_at(center.lerp(to, 0.5));
    w.step();
    w.hover_at(to);
    w.step();
    w.drop_at(to);
    w.step();
    w.hover_at(center);
    w.event(Event::MouseWheel {
        unit: egui::MouseWheelUnit::Point,
        delta: Vec2::new(0.0, 200.0),
        phase: egui::TouchPhase::Move,
        source: egui::MouseWheelSource::Wheel,
        modifiers: Modifiers::NONE,
    });
    w.event(Event::Zoom(1.5));
    w.run_steps(4);
    // A click where the approach was fitted no longer lands on it...
    press(&mut w, &format!("🚗 {}", VKind::Car.name()));
    let target = on_map(&w, &net, approach(&net, 0, net.legs[0].stop_dist + 25.0), false);
    let before = count(&w, "waiting to enter");
    click_at(&mut w, target);
    assert_eq!(count(&w, "waiting to enter"), before);
    // ...until Fit view puts it back.
    press(&mut w, "Fit view");
    click_at(&mut w, target);
    assert_eq!(count(&w, "waiting to enter"), before + 1);
}

/// Let the in-app benchmark finish, drawing frames as a person would watch it.
#[track_caller]
fn wait_for_benchmark(w: &mut Window) {
    let start = Instant::now();
    while shows(w, "running ") {
        assert!(start.elapsed() < Duration::from_secs(600), "the benchmark didn't finish");
        std::thread::sleep(Duration::from_millis(20));
        w.step();
    }
    w.run_steps(2);
}

#[test]
fn the_benchmark_compares_every_strategy() {
    let mut w = open();
    pause(&mut w);
    // No traffic, so the runs take moments: the test is of the window, not the results.
    slide(&mut w, "demand ×", 0.0);
    press(&mut w, "▶ Compare controllers");
    assert!(
        shows(&w, &format!("Controller comparison — {}", LayoutKind::Arterial4.name()))
            || w.query_by_label(&format!("Controller comparison — {}", LayoutKind::Arterial4.name()))
                .is_some()
    );
    // One run at a time per strategy: the buttons wait until it's done.
    assert!(w.get_by_label("▶ Compare controllers").accesskit_node().is_disabled());
    wait_for_benchmark(&mut w);
    assert!(shows(&w, "Person delay incl. unserved (s) — lower is better"));
    for c in CtrlKind::ALL {
        assert!(has(&w, c.name()), "no row for {}", c.name());
    }
    assert!(has(&w, ControlMode::AllWayStop.name()));
    assert_named(&w);
    press(&mut w, "Hide results window");
    assert!(!shows(&w, "lower is better"));
    press(&mut w, "Show results window");
    assert!(shows(&w, "lower is better"));
    // Escape closes it too.
    w.key_press(egui::Key::Escape);
    w.run_steps(2);
    assert!(!shows(&w, "lower is better"));

    // The cooperative crossing compares its schedules and gap policies.
    choose(&mut w, "Intersection", LayoutKind::Cooperative.name());
    slide(&mut w, "demand ×", 0.0);
    press(&mut w, "▶ Compare schedules");
    wait_for_benchmark(&mut w);
    for p in GapPolicy::ALL {
        assert!(has(&w, &format!("Batches · {} gaps", p.label())));
    }
    assert!(has(&w, &format!("First come, first served · {} gaps", GapPolicy::FailSafe.label())));
    assert!(w.query_by_label("▶ Sensor study").is_none(), "the crossing has no sensors to study");
}

#[test]
fn about_lists_the_version_and_licenses() {
    let mut w = open();
    press(&mut w, "About");
    assert!(has(&w, &format!("Intersection Lab {}", env!("CARGO_PKG_VERSION"))));
    assert!(shows(&w, "Third-party software"));
    assert!(w.query_by_label(env!("CARGO_PKG_REPOSITORY")).is_some());
    assert_named(&w);
    // Escape closes it; so does the button.
    w.key_press(egui::Key::Escape);
    w.run_steps(2);
    assert!(!shows(&w, "Third-party software"));
    press(&mut w, "About");
    press(&mut w, "About");
    assert!(!shows(&w, "Third-party software"));
}
