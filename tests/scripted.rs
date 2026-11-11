// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The app's scripted mode, which sets up the documentation's screenshots: `TS_*`
//! variables describe the scene when the app starts. The app reads them once, as it
//! starts, and tests in one process share one environment, so each scene opens in a
//! process of its own: this test binary again, running only `scene`, which prints what
//! the window shows for the test that started it to check.

use egui::accesskit::{Role, Toggled};
use egui::{Shape, ViewportCommand, ViewportId};
use egui_kittest::Harness;
use egui_kittest::kittest::{NodeT, Queryable};
use intersection_lab::control::CtrlKind;
use intersection_lab::gui::App;
use intersection_lab::gui::shot::write_png;
use intersection_lab::sim::ControlMode;
use intersection_lab::sim::coop::{GapPolicy, Scheduling};
use intersection_lab::sim::network::LayoutKind;
use intersection_lab::sim::sensors::Conditions;
use std::path::Path;
use std::process::Command;
use std::time::{Duration, Instant};

type Window = Harness<'static, App>;

/// Whether the app asked for a screenshot in the last frame.
fn screenshot_requested(w: &Window) -> bool {
    w.output()
        .viewport_output
        .get(&ViewportId::ROOT)
        .is_some_and(|v| v.commands.iter().any(|c| matches!(c, ViewportCommand::Screenshot(_))))
}

/// The scene the `TS_*` variables describe, run for 30 simulated seconds (or, for a
/// screenshot, until the app takes it), then printed: one line per drop-down list,
/// number field, ticked checkbox, label and text painted on the map.
#[test]
#[ignore = "a scene for the tests below, each in a process of its own"]
fn scene() {
    let mut w: Window = Harness::builder()
        .with_size([1600.0, 1000.0])
        // The desktop app draws emoji with the system's fonts, which a test doesn't load.
        .allow_missing_glyphs()
        .build_eframe(|cc| App::new(cc));
    w.run_steps(2);
    let mut shot = false;
    if std::env::var_os("TS_SHOT").is_some() {
        // The clock runs in real time up to the shot's moment, and the app gives the
        // panels a moment to catch up before it asks for the capture.
        let start = Instant::now();
        while !shot && start.elapsed() < Duration::from_secs(30) {
            std::thread::sleep(Duration::from_millis(20));
            w.step();
            shot = screenshot_requested(&w);
        }
    } else {
        w.get_by_label("⏸ Pause").click_accesskit();
        w.run_steps(2);
        for _ in 0..30 {
            w.get_by_label("⏭ Step").click_accesskit();
            w.step();
        }
        w.run_steps(2);
    }
    // The test harness has printed the test's name without ending the line.
    println!();
    println!("shot: {shot}");
    for n in w.query_all_by_role(Role::ComboBox) {
        println!(
            "list {}: {}",
            n.accesskit_node().label().unwrap_or_default(),
            n.value().unwrap_or_default()
        );
    }
    for n in w.query_all_by_role(Role::SpinButton) {
        println!(
            "number {}: {}",
            n.accesskit_node().label().unwrap_or_default(),
            n.value().unwrap_or_default()
        );
    }
    for n in w.query_all_by_role(Role::CheckBox) {
        if n.accesskit_node().toggled() == Some(Toggled::True) {
            println!("ticked {}", n.accesskit_node().label().unwrap_or_default());
        }
    }
    for n in w.query_all_by_role(Role::Label) {
        println!("label {}", n.value().unwrap_or_default());
    }
    fn walk(shape: &Shape) {
        match shape {
            Shape::Vec(shapes) => shapes.iter().for_each(walk),
            Shape::Text(t) => println!("painted {}", t.galley.text()),
            _ => {}
        }
    }
    for c in &w.output().shapes {
        walk(&c.shape);
    }
}

/// What the scene's window showed.
struct Scene(String);

impl Scene {
    fn lines(&self) -> impl Iterator<Item = &str> {
        self.0.lines()
    }
    /// The value of the drop-down list (or number field, `what` "number") `name`.
    fn value(&self, what: &str, name: &str) -> Option<&str> {
        let prefix = format!("{what} {name}: ");
        self.lines().find_map(|l| l.strip_prefix(prefix.as_str()))
    }
    fn list(&self, name: &str) -> &str {
        self.value("list", name).unwrap_or_else(|| panic!("no {name} list:\n{}", self.0))
    }
    fn ticked(&self, name: &str) -> bool {
        self.lines().any(|l| l == format!("ticked {name}"))
    }
    fn shows(&self, text: &str) -> bool {
        self.lines().any(|l| l.starts_with("label ") && l.contains(text))
    }
    fn paints(&self, text: &str) -> bool {
        self.lines().any(|l| l.starts_with("painted ") && l.contains(text))
    }
    /// The status line at the top left of the map.
    fn status(&self) -> &str {
        self.lines()
            .filter_map(|l| l.strip_prefix("painted "))
            .find(|t| t.len() > 8 && t.as_bytes()[2] == b':' && t.as_bytes()[5] == b':')
            .unwrap_or_else(|| panic!("no status line:\n{}", self.0))
    }
}

/// Open the scene `vars` describe, in a process of its own.
fn scene_of(vars: &[(&str, &str)]) -> Scene {
    let mut cmd = Command::new(std::env::current_exe().unwrap());
    cmd.args(["scene", "--exact", "--ignored", "--nocapture", "--test-threads=1"]);
    // Only the variables of this scene.
    for (k, _) in std::env::vars_os() {
        if k.to_string_lossy().starts_with("TS_") {
            cmd.env_remove(k);
        }
    }
    let out = cmd.envs(vars.iter().copied()).output().unwrap();
    let stdout = String::from_utf8_lossy(&out.stdout).into_owned();
    assert!(out.status.success(), "{vars:?}:\n{stdout}\n{}", String::from_utf8_lossy(&out.stderr));
    Scene(stdout)
}

#[test]
fn a_signal_with_every_overlay_in_snow_at_night() {
    let s = scene_of(&[
        ("TS_LAYOUT", "arterial4"),
        ("TS_CTRL", "MaxPressure"),
        ("TS_SPEED", "5"),
        ("TS_ZOOM", "1.4"),
        ("TS_COLOR", "speed"),
        ("TS_NIGHT", "1"),
        ("TS_COND", "snow"),
        ("TS_LOOP_FAULT", "0.2"),
        ("TS_OVERLAYS", "sensors, detections,conflicts,paths,labels"),
        ("TS_SELECT", "1"),
        ("TS_SCROLL", "200"),
        ("TS_SCROLL_LEFT", "100"),
    ]);
    assert_eq!(s.list("Intersection"), LayoutKind::Arterial4.name());
    assert_eq!(s.list("Controller"), CtrlKind::MaxPressure.name());
    assert_eq!(s.list("conditions"), Conditions::Snow.name());
    assert!(s.value("number", "faulty loop stations").is_some_and(|v| v.starts_with("0.2")), "{}", s.0);
    for overlay in ["sensors", "detections", "conflict points", "paths", "delay labels", "night"] {
        assert!(s.ticked(overlay), "{overlay} is off:\n{}", s.0);
    }
    assert!(s.status().contains("×5"), "{}", s.status());
    // Vehicle 1 is selected, whether it is still on the map or not.
    assert!(!s.shows("Click a vehicle to inspect it."), "{}", s.0);
    assert!(s.shows("(left the network)") || s.lines().any(|l| l == "label doing"), "{}", s.0);
}

#[test]
fn the_cooperative_crossing_with_both_schedules_and_the_about_window() {
    let s = scene_of(&[
        ("TS_LAYOUT", "cooperative"),
        ("TS_POLICY", "p1"),
        ("TS_SCHED", "fcfs"),
        ("TS_TWIN", "ph"),
        ("TS_COLOR", "turn"),
        ("TS_ABOUT", "1"),
    ]);
    assert_eq!(s.list("Safety gap"), GapPolicy::Nominal.name());
    assert_eq!(s.list("Scheduling"), Scheduling::FirstCome.name());
    assert_eq!(s.list("B scheduling"), Scheduling::Batches.label());
    assert_eq!(s.list("B gap policy"), format!("{} gaps", GapPolicy::Mitigated.label()));
    assert!(s.paints("A · First come, first served · nominal gaps"), "{}", s.0);
    assert!(s.paints("B · Intersection manager · mitigated gaps"), "{}", s.0);
    assert!(s.shows("Third-party software"), "the About window is closed:\n{}", s.0);
}

#[test]
fn the_comparison_can_serve_first_come_first_served() {
    let s = scene_of(&[("TS_LAYOUT", "cooperative"), ("TS_TWIN", "fcfs")]);
    assert!(s.paints("A · Intersection manager · fail-safe gaps"), "{}", s.0);
    assert!(s.paints("B · First come, first served · fail-safe gaps"), "{}", s.0);
}

#[test]
fn each_control_mode_can_be_chosen() {
    for (layout, mode, twin, expect, b) in [
        ("majorminor4", "twoway", "allway", ControlMode::TwoWayStop, Some(ControlMode::AllWayStop)),
        ("tee", "allway", "twoway", ControlMode::AllWayStop, Some(ControlMode::TwoWayStop)),
        ("roundabout", "roundabout", "", ControlMode::Roundabout, None),
        ("cooperative", "cooperative", "", ControlMode::Cooperative, None),
        ("town4", "signal", "Hybrid", ControlMode::Signal, None),
    ] {
        let mut vars = vec![("TS_LAYOUT", layout), ("TS_MODE", mode), ("TS_COLOR", "delay")];
        if !twin.is_empty() {
            vars.push(("TS_TWIN", twin));
        }
        let s = scene_of(&vars);
        assert_eq!(s.list("Control"), expect.name(), "{layout}");
        if let Some(b) = b {
            assert_eq!(s.list("B control"), b.name(), "{layout}");
            assert!(s.paints(&format!("B · {}", b.name())), "{}", s.0);
        }
        if twin == "Hybrid" {
            assert_eq!(s.list("B controller"), CtrlKind::Hybrid.short());
        }
    }
}

#[test]
fn a_corridor_on_a_green_wave() {
    let s = scene_of(&[
        ("TS_LAYOUT", "arterial4"),
        ("TS_CORRIDOR", "3"),
        ("TS_WAVE", "1"),
        ("TS_TWIN", "Predictive"),
        ("TS_COLOR", "type"),
    ]);
    assert!(s.shows("Intersection 2 of 3 (west → east)"), "{}", s.0);
    assert!(s.ticked("Fixed-time green wave"));
    assert_eq!(s.list("Controller"), CtrlKind::Fixed.name());
    assert_eq!(s.value("number", "signals along main road"), Some("3"));
}

#[test]
fn a_benchmark_can_start_with_the_app() {
    for (which, title) in [("controllers", "Controller comparison — "), ("sensors", "Sensor study — ")] {
        let s = scene_of(&[("TS_LAYOUT", "tee"), ("TS_BENCH", which)]);
        let title = format!("{title}{}", LayoutKind::Tee.name());
        assert!(s.shows(&title) || s.0.contains(&title), "{which}:\n{}", s.0);
        assert!(s.shows("running "), "{which}:\n{}", s.0);
    }
}

#[test]
fn a_scripted_screenshot_stops_the_clock_at_its_moment() {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("scripted");
    std::fs::create_dir_all(&dir).unwrap();
    let png = dir.join("shot.png");
    let s = scene_of(&[("TS_SHOT", png.to_str().unwrap()), ("TS_SHOT_AFTER", "2"), ("TS_SPEED", "100")]);
    assert!(s.lines().any(|l| l == "shot: true"), "no screenshot:\n{}", s.0);
    assert!(s.status().starts_with("00:00:02 "), "{}", s.status());
}

/// The image in a PNG that `write_png` wrote: width, height and RGBA pixels. Checks
/// every chunk's checksum and the compressed data's.
fn read_png(bytes: &[u8]) -> (usize, usize, Vec<u8>) {
    fn crc32(data: &[u8]) -> u32 {
        let mut c = 0xFFFF_FFFFu32;
        for &b in data {
            c ^= b as u32;
            for _ in 0..8 {
                c = if c & 1 != 0 { 0xEDB8_8320 ^ (c >> 1) } else { c >> 1 };
            }
        }
        !c
    }
    let be = |b: &[u8]| u32::from_be_bytes(b.try_into().unwrap()) as usize;
    assert_eq!(&bytes[..8], b"\x89PNG\r\n\x1a\n");
    let (mut at, mut chunks) = (8, vec![]);
    while at < bytes.len() {
        let len = be(&bytes[at..at + 4]);
        let body = &bytes[at + 4..at + 8 + len];
        assert_eq!(be(&bytes[at + 8 + len..at + 12 + len]) as u32, crc32(body), "chunk checksum");
        chunks.push((&body[..4], &body[4..]));
        at += 12 + len;
    }
    let kinds: Vec<&[u8]> = chunks.iter().map(|c| c.0).collect();
    assert_eq!(kinds, [b"IHDR", b"IDAT", b"IEND"]);
    let ihdr = chunks[0].1;
    let (w, h) = (be(&ihdr[0..4]), be(&ihdr[4..8]));
    assert_eq!(&ihdr[8..], [8, 6, 0, 0, 0], "8-bit RGBA, no interlacing");
    // zlib with stored (uncompressed) blocks.
    let z = chunks[1].1;
    assert_eq!(&z[..2], [0x78, 0x01]);
    let (mut raw, mut at) = (vec![], 2);
    loop {
        let last = z[at] & 1 == 1;
        let len = u16::from_le_bytes([z[at + 1], z[at + 2]]);
        assert_eq!(u16::from_le_bytes([z[at + 3], z[at + 4]]), !len);
        raw.extend_from_slice(&z[at + 5..at + 5 + len as usize]);
        at += 5 + len as usize;
        if last {
            break;
        }
    }
    let (mut a, mut b) = (1u32, 0u32);
    for &x in &raw {
        a = (a + x as u32) % 65_521;
        b = (b + a) % 65_521;
    }
    assert_eq!(be(&z[at..at + 4]) as u32, (b << 16) | a, "zlib checksum");
    // Each row: filter type 0, then the pixels.
    let mut rgba = vec![];
    for row in raw.chunks(w * 4 + 1) {
        assert_eq!(row[0], 0);
        rgba.extend_from_slice(&row[1..]);
    }
    assert_eq!(rgba.len(), w * h * 4);
    (w, h, rgba)
}

#[test]
fn a_screenshot_is_a_valid_png() {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("scripted");
    std::fs::create_dir_all(&dir).unwrap();
    // Small, and big enough to need several compressed blocks.
    for (w, h) in [(3, 2), (300, 200)] {
        let rgba: Vec<u8> = (0..w * h * 4).map(|i| (i * 7 % 251) as u8).collect();
        let path = dir.join(format!("{w}x{h}.png"));
        write_png(path.to_str().unwrap(), w, h, &rgba).unwrap();
        assert_eq!(read_png(&std::fs::read(&path).unwrap()), (w, h, rgba));
    }
    // A path that can't be written is an error, not a panic.
    assert!(write_png(dir.join("missing/shot.png").to_str().unwrap(), 1, 1, &[0; 4]).is_err());
}
