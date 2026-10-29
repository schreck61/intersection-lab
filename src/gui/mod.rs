//! Interactive desktop front-end (eframe/egui).

pub mod charts;
pub mod render;
pub mod shot;

use crate::control::CtrlKind;
use crate::sim::demand::{DemandConfig, Pattern, compass_index};
use crate::sim::geom::v2;
use crate::sim::network::{LayoutKind, Network, Turn};
use crate::sim::sensors::{Conditions, SensorConfig, Source};
use crate::sim::stats::{HIST_BIN, Summary, los};
use crate::sim::vtypes::{ALL_KINDS, VKind};
use crate::sim::world::World;
use crate::sim::{ControlMode, LeftMode, Sim};
use egui::{Color32, RichText, Sense, Ui, vec2};
use render::{Camera, ColorBy, DrawCtx, Overlays, Scene};
use std::sync::mpsc;
use std::time::Instant;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Tool {
    Select,
    Vehicle,
    Pedestrian,
}

struct BenchRow {
    label: String,
    summary: Summary,
}

struct Bench {
    title: String,
    rx: mpsc::Receiver<(usize, BenchRow)>,
    total: usize,
    rows: Vec<Option<BenchRow>>,
}

pub struct App {
    world: World,
    /// Intersection whose statistics and controls the panels show.
    focus: usize,
    /// Side-by-side comparison: the same arrivals under another control strategy.
    twin: Option<World>,
    twin_mode: ControlMode,
    twin_ctrl: CtrlKind,
    twin_summary: Summary,
    scene: Scene,
    layout: LayoutKind,
    mode: ControlMode,
    ctrl: CtrlKind,
    seed: u64,
    paused: bool,
    speed: f32,
    acc: f64,
    last: Instant,
    cam_center: crate::sim::geom::V2,
    scale: f32,
    fitted: bool,
    ov: Overlays,
    selected: Option<u64>,
    tool: Tool,
    spawn_kind: VKind,
    spawn_turn: Turn,
    summary: Summary,
    summary_at: Instant,
    stats_from: f64,
    bench: Option<Bench>,
    show_bench: bool,
    anim_t: f64,
    message: Option<(String, Instant)>,
    shot: Option<(String, f64)>,
}

impl App {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let mut style = (*cc.egui_ctx.global_style()).clone();
        style.visuals = egui::Visuals::dark();
        style.visuals.panel_fill = Color32::from_rgb(28, 30, 36);
        style.visuals.window_fill = Color32::from_rgb(30, 32, 38);
        style.spacing.item_spacing = vec2(6.0, 5.0);
        cc.egui_ctx.set_global_style(style);
        let layout = LayoutKind::Arterial4;
        let ctrl = CtrlKind::Predictive;
        let mut sim = Sim::new(layout, ControlMode::Signal, ctrl, 1);
        sim.check_safety = true;
        let scene = Scene::build(&sim.net);
        App {
            summary: Summary::default(),
            world: World::single(sim),
            focus: 0,
            twin: None,
            twin_mode: ControlMode::Signal,
            twin_ctrl: CtrlKind::Fixed,
            twin_summary: Summary::default(),
            scene,
            layout,
            mode: ControlMode::Signal,
            ctrl,
            seed: 1,
            paused: false,
            speed: 1.0,
            acc: 0.0,
            last: Instant::now(),
            cam_center: v2(0.0, 0.0),
            scale: 5.0,
            fitted: false,
            ov: Overlays::default(),
            selected: None,
            tool: Tool::Select,
            spawn_kind: VKind::Car,
            spawn_turn: Turn::T,
            summary_at: Instant::now(),
            stats_from: 0.0,
            bench: None,
            show_bench: false,
            anim_t: 0.0,
            message: None,
            shot: None,
        }
        .with_env()
    }

    /// Optional overrides for scripted runs: TS_LAYOUT, TS_MODE, TS_CTRL, TS_TWIN (B side:
    /// a controller or allway/twoway), TS_SPEED, TS_ZOOM, TS_COLOR, TS_NIGHT, TS_COND,
    /// TS_LOOP_FAULT, TS_SHOT (png path) + TS_SHOT_AFTER (sim seconds).
    fn with_env(mut self) -> Self {
        let var = |k: &str| std::env::var(k).ok();
        if let Some(l) = var("TS_LAYOUT").and_then(|v| LayoutKind::ALL.into_iter().find(|k| k.key() == v)) {
            self.layout = l;
        }
        if let Some(m) = var("TS_MODE") {
            self.mode = match m.as_str() {
                "allway" => ControlMode::AllWayStop,
                "twoway" => ControlMode::TwoWayStop,
                "roundabout" => ControlMode::Roundabout,
                _ => ControlMode::Signal,
            };
        }
        if let Some(c) = var("TS_CTRL").and_then(|v| CtrlKind::ALL.into_iter().find(|k| k.short() == v)) {
            self.ctrl = c;
        }
        if let Some(b) = var("TS_TWIN") {
            self.twin = Some(World::single(Sim::new(self.layout, ControlMode::Signal, CtrlKind::Fixed, 0)));
            match CtrlKind::ALL.into_iter().find(|k| k.short() == b) {
                Some(c) => self.twin_ctrl = c,
                None => self.twin_mode = if b == "allway" { ControlMode::AllWayStop } else { ControlMode::TwoWayStop },
            }
        }
        self.rebuild(false);
        if let Some(s) = var("TS_SPEED").and_then(|v| v.parse().ok()) {
            self.speed = s;
        }
        if let Some(c) = var("TS_COLOR") {
            self.ov.color_by = match c.as_str() {
                "delay" => ColorBy::Delay,
                "turn" => ColorBy::Movement,
                "speed" => ColorBy::Speed,
                _ => ColorBy::Type,
            };
        }
        self.ov.night = var("TS_NIGHT").is_some();
        if let Some(c) = var("TS_COND").and_then(|v| Conditions::ALL.into_iter().find(|c| format!("{c:?}").eq_ignore_ascii_case(&v))) {
            self.world.sims[self.focus].sensors.conditions = c;
        }
        if let Some(f) = var("TS_LOOP_FAULT").and_then(|v| v.parse().ok()) {
            self.world.sims[self.focus].sensors.loop_fault = f;
        }
        if let Some(p) = var("TS_SHOT") {
            let after = var("TS_SHOT_AFTER").and_then(|v| v.parse().ok()).unwrap_or(120.0);
            self.shot = Some((p, after));
        }
        self
    }

    fn rebuild(&mut self, keep_demand: bool) {
        if !ControlMode::options(self.layout).contains(&self.mode) {
            self.mode = ControlMode::options(self.layout)[0];
        }
        let old = &self.world.sims[self.focus];
        let net = Network::build(self.layout);
        let demand = if keep_demand && old.net.kind == self.layout { old.demand.clone() } else { DemandConfig::default_for(&net) };
        let template = Self::settings_of(old);
        self.world = World::single(self.make_sim(net, self.mode, self.ctrl, demand, &template));
        self.focus = 0;
        self.rebuild_twin();
        if self.scene.kind != self.layout {
            self.scene = Scene::build(&self.world.sims[0].net);
            self.fitted = false;
        }
        self.selected = None;
        self.stats_from = 0.0;
        self.summary = Summary::default();
    }

    /// The comparison world: identical layout, demand, seed and settings, but its own
    /// control strategy.
    fn rebuild_twin(&mut self) {
        if self.twin.is_none() {
            return;
        }
        if !ControlMode::options(self.layout).contains(&self.twin_mode) {
            self.twin_mode = ControlMode::options(self.layout)[0];
        }
        let src = &self.world.sims[0];
        let template = Self::settings_of(src);
        let sim = self.make_sim(Network::build(self.layout), self.twin_mode, self.twin_ctrl, src.demand.clone(), &template);
        self.twin = Some(World::single(sim));
        self.twin_summary = Summary::default();
    }

    fn settings_of(s: &Sim) -> Sim {
        // A throwaway carrier for the settings a rebuild keeps.
        let mut t = Sim::new(LayoutKind::Town4, ControlMode::AllWayStop, CtrlKind::Fixed, 0);
        t.sensors = s.sensors.clone();
        t.rtor = s.rtor;
        t.ntor_peds = s.ntor_peds;
        t.signal.lpi = s.signal.lpi;
        t.left_mode = s.left_mode;
        t.preemption = s.preemption;
        t.max_wait_guard = s.max_wait_guard;
        t
    }

    fn make_sim(&self, net: Network, mode: ControlMode, ctrl: CtrlKind, demand: DemandConfig, settings: &Sim) -> Sim {
        let mut sim = Sim::with_demand(net, mode, ctrl, demand, self.seed);
        sim.sensors = settings.sensors.clone();
        sim.rtor = settings.rtor;
        sim.preemption = settings.preemption;
        sim.max_wait_guard = settings.max_wait_guard;
        sim.signal.lpi = settings.signal.lpi;
        sim.ntor_peds = settings.ntor_peds;
        sim.left_mode = settings.left_mode;
        sim.check_safety = true;
        sim
    }

    /// Propagate settings edited on the focused intersection to the rest of the world
    /// and to the comparison twin.
    fn sync_settings(&mut self) {
        self.world.sync_settings(self.focus);
        if let Some(t) = self.twin.as_mut() {
            t.follow(&self.world);
        }
    }

    fn notify(&mut self, s: impl Into<String>) {
        self.message = Some((s.into(), Instant::now()));
    }

    fn advance(&mut self, ctx: &egui::Context) {
        let now = Instant::now();
        let real = now.duration_since(self.last).as_secs_f64().min(0.1);
        self.last = now;
        self.anim_t += real;
        if self.paused {
            return;
        }
        self.acc += real * self.speed as f64;
        let budget = Instant::now();
        let dt = self.world.dt();
        while self.acc >= dt {
            self.world.step();
            if let Some(t) = self.twin.as_mut() {
                t.step();
            }
            self.acc -= dt;
            if budget.elapsed().as_millis() > 30 {
                self.acc = 0.0;
                break;
            }
        }
        ctx.request_repaint();
    }

    fn refresh_summary(&mut self) {
        if self.summary_at.elapsed().as_millis() > 400 {
            let sim = &self.world.sims[self.focus];
            self.summary = sim.stats.summary(self.stats_from, sim.t, &sim.vehicles, &sim.queues);
            if let Some(tw) = &self.twin {
                let s = &tw.sims[self.focus.min(tw.sims.len() - 1)];
                self.twin_summary = s.stats.summary(self.stats_from, s.t, &s.vehicles, &s.queues);
            }
            self.summary_at = Instant::now();
        }
    }

    // ------------------------------------------------------------- panels

    fn top_bar(&mut self, ui: &mut Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.label(RichText::new("🚦 Intersection Lab").strong().size(16.0));
            ui.separator();
            if ui.button(if self.paused { "▶ Run" } else { "⏸ Pause" }).clicked() {
                self.paused = !self.paused;
            }
            if ui.button("⏭ Step").on_hover_text("Advance 1 s").clicked() {
                for _ in 0..10 {
                    self.world.step();
                    if let Some(t) = self.twin.as_mut() {
                        t.step();
                    }
                }
            }
            ui.label("Speed");
            for s in [0.5f32, 1.0, 2.0, 5.0, 10.0, 30.0, 100.0] {
                if ui.selectable_label(self.speed == s, format!("×{s}")).clicked() {
                    self.speed = s;
                }
            }
            if ui.button("⟲ Reset").clicked() {
                self.rebuild(true);
            }
            ui.separator();
            ui.label("Spawn:");
            for k in ALL_KINDS {
                let icon = match k {
                    VKind::Car => "🚗",
                    VKind::Suv => "🚙",
                    VKind::Truck => "🚚",
                    VKind::Bus => "🚌",
                    VKind::Motorcycle => "🏍",
                    VKind::Emergency => "🚑",
                };
                let sel = self.tool == Tool::Vehicle && self.spawn_kind == k;
                if ui.selectable_label(sel, format!("{icon} {}", k.name())).on_hover_text("Then click an approach road (or use the N/E/S/W buttons)").clicked() {
                    self.tool = if sel { Tool::Select } else { Tool::Vehicle };
                    self.spawn_kind = k;
                }
            }
            if ui.selectable_label(self.tool == Tool::Pedestrian, "🚶 Pedestrian").on_hover_text("Click near a crosswalk corner").clicked() {
                self.tool = if self.tool == Tool::Pedestrian { Tool::Select } else { Tool::Pedestrian };
            }
            ui.separator();
            ui.label("Intent:");
            for t in Turn::ALL {
                let label = match t {
                    Turn::L => "↰ Left",
                    Turn::T => "↑ Through",
                    Turn::R => "↱ Right",
                };
                ui.selectable_value(&mut self.spawn_turn, t, label);
            }
            if self.tool == Tool::Vehicle {
                ui.separator();
                for li in 0..self.world.sims[self.focus].net.legs.len() {
                    let (letter, bound) = {
                        let leg = &self.world.sims[self.focus].net.legs[li];
                        (leg.compass.letter(), leg.compass.bound())
                    };
                    if ui.button(format!("+ {letter}")).on_hover_text(format!("{bound} approach")).clicked() {
                        let ok = self.world.sims[self.focus].spawn_manual(li, self.spawn_kind, self.spawn_turn);
                        if let Some(t) = self.twin.as_mut() {
                            t.sims[self.focus].spawn_manual(li, self.spawn_kind, self.spawn_turn);
                        }
                        if !ok {
                            self.notify(format!("{} has no {} movement", bound, self.spawn_turn.name()));
                        }
                    }
                }
            }
        });
    }

    fn left_panel(&mut self, ui: &mut Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.heading("Scenario");
            let mut layout = self.layout;
            ui.label("Intersection");
            egui::ComboBox::from_id_salt("layout")
                .selected_text(layout.name())
                .width(270.0)
                .show_ui(ui, |ui| {
                    for k in LayoutKind::ALL {
                        ui.selectable_value(&mut layout, k, k.name());
                    }
                });
            if layout != self.layout {
                self.layout = layout;
                self.rebuild(false);
            }
            let mut mode = self.mode;
            ui.label("Control");
            egui::ComboBox::from_id_salt("mode")
                .selected_text(mode.name())
                .width(270.0)
                .show_ui(ui, |ui| {
                    for m in ControlMode::options(self.layout) {
                        ui.selectable_value(&mut mode, m, m.name());
                    }
                });
            if mode != self.mode {
                self.mode = mode;
                self.rebuild(true);
            }
            if self.mode == ControlMode::Signal {
                let mut c = self.ctrl;
                ui.label("Controller");
            egui::ComboBox::from_id_salt("ctrl")
                    .selected_text(c.name())
                    .width(270.0)
                    .show_ui(ui, |ui| {
                        for k in CtrlKind::ALL {
                            ui.selectable_value(&mut c, k, k.name());
                        }
                    });
                if c != self.ctrl {
                    // Hot-swap: traffic keeps flowing, only the brain changes.
                    self.ctrl = c;
                    self.world.sims[self.focus].set_controller(c);
                    self.notify(format!("Controller switched live to {}", c.name()));
                }
                ui.label(RichText::new(self.ctrl.description()).small().color(Color32::from_gray(170)));
            }
            ui.horizontal(|ui| {
                ui.label("Seed");
                ui.add(egui::DragValue::new(&mut self.seed).range(1..=99_999));
                if ui.button("Restart").clicked() {
                    self.rebuild(true);
                }
            });
            let mut compare = self.twin.is_some();
            if ui
                .checkbox(&mut compare, "Compare side by side")
                .on_hover_text("Run a second copy with the same arrivals under another strategy (restarts both)")
                .changed()
            {
                self.twin = if compare { Some(World::single(Sim::new(self.layout, ControlMode::Signal, CtrlKind::Fixed, 0))) } else { None };
                self.rebuild(true);
            }
            if self.twin.is_some() {
                let (mut m, mut c) = (self.twin_mode, self.twin_ctrl);
                ui.horizontal(|ui| {
                    ui.label("B:");
                    egui::ComboBox::from_id_salt("twin_mode").selected_text(m.name()).width(120.0).show_ui(ui, |ui| {
                        for x in ControlMode::options(self.layout) {
                            ui.selectable_value(&mut m, x, x.name());
                        }
                    });
                    if m == ControlMode::Signal {
                        egui::ComboBox::from_id_salt("twin_ctrl").selected_text(c.short()).width(110.0).show_ui(ui, |ui| {
                            for k in CtrlKind::ALL {
                                ui.selectable_value(&mut c, k, k.name());
                            }
                        });
                    }
                });
                if m != self.twin_mode {
                    self.twin_mode = m;
                    self.rebuild(true);
                } else if c != self.twin_ctrl {
                    self.twin_ctrl = c;
                    if let Some(t) = self.twin.as_mut() {
                        for s in &mut t.sims {
                            s.set_controller(c);
                        }
                    }
                }
            }

            ui.separator();
            ui.heading("Demand");
            let mut changed = false;
            let d = &mut self.world.sims[self.focus].demand;
            changed |= ui.add(egui::Slider::new(&mut d.scale, 0.0..=2.0).text("demand ×")).changed();
            let mut pat = d.pattern;
            ui.label("Arrivals");
            egui::ComboBox::from_id_salt("pattern")
                .selected_text(pat.name())
                .show_ui(ui, |ui| {
                    for p in Pattern::ALL {
                        ui.selectable_value(&mut pat, p, p.name());
                    }
                });
            if pat != d.pattern {
                d.pattern = pat;
                changed = true;
            }
            for li in 0..self.world.sims[self.focus].net.legs.len() {
                let sim = &mut self.world.sims[self.focus];
                let leg = &sim.net.legs[li];
                let ci = compass_index(leg.compass);
                let turns = leg.turns();
                let d = &mut sim.demand;
                egui::CollapsingHeader::new(format!("{} — {:.0} veh/h", leg.compass.bound(), d.rate[ci] * d.scale))
                    .id_salt(("leg", li))
                    .show(ui, |ui| {
                        changed |= ui.add(egui::Slider::new(&mut d.rate[ci], 0.0..=1600.0).text("veh/h").step_by(10.0)).changed();
                        for t in Turn::ALL {
                            if turns.contains(&t) {
                                changed |= ui.add(egui::Slider::new(&mut d.turn[ci][t.index()], 0.0..=1.0).text(format!("{} share", t.name()))).changed();
                            }
                        }
                    });
            }
            egui::CollapsingHeader::new("Vehicle mix").show(ui, |ui| {
                for k in ALL_KINDS {
                    changed |= ui.add(egui::Slider::new(&mut self.world.sims[self.focus].demand.mix[k.index()], 0.0..=1.0).text(k.name())).changed();
                }
            });
            if !self.world.sims[self.focus].net.crosswalks.is_empty() {
                changed |= ui.add(egui::Slider::new(&mut self.world.sims[self.focus].demand.ped_rate, 0.0..=600.0).text("peds/h").step_by(5.0)).changed();
            }
            if changed {
                // Fixed-time plans are re-optimised from the new demand (Webster).
                self.sync_settings();
                for w in std::iter::once(&mut self.world).chain(self.twin.as_mut()) {
                    for sim in &mut w.sims {
                        if sim.ctrl_kind == CtrlKind::Fixed && sim.signalized() {
                            sim.set_controller(CtrlKind::Fixed);
                        }
                    }
                }
            }

            ui.separator();
            ui.heading("Sensors");
            let s = &mut self.world.sims[self.focus].sensors;
            ui.checkbox(&mut s.loops, "Inductive loops (stop-bar + advance)");
            ui.add_enabled(s.loops, egui::Slider::new(&mut s.adv_dist, 20.0..=120.0).text("advance loop m"));
            ui.checkbox(&mut s.camera, "Intersection cameras (video analytics)");
            ui.add_enabled(s.camera, egui::Slider::new(&mut s.cam_range, 20.0..=150.0).text("camera range m"));
            ui.checkbox(&mut s.radar, "Advance radar (long range)");
            ui.add_enabled(s.radar, egui::Slider::new(&mut s.radar_range, 80.0..=260.0).text("radar range m"));
            ui.checkbox(&mut s.cv, "Connected vehicles (V2I)");
            ui.add_enabled(s.cv, egui::Slider::new(&mut s.cv_rate, 0.0..=1.0).text("CV penetration"));
            ui.checkbox(&mut s.realistic, "Field-realistic sensor errors")
                .on_hover_text("Cameras miss vehicles (weather, night, range, behind trucks) and only read intent from turn signals; loops fail and miss motorcycles; radar and connected-vehicle data are noisy");
            ui.add_enabled_ui(s.realistic, |ui| {
                egui::ComboBox::from_label("conditions")
                    .selected_text(s.conditions.name())
                    .show_ui(ui, |ui| {
                        for c in Conditions::ALL {
                            ui.selectable_value(&mut s.conditions, c, c.name());
                        }
                    });
                ui.add(egui::Slider::new(&mut s.loop_fault, 0.0..=0.5).text("faulty loop stations"))
                    .on_hover_text("Share of loop stations stuck on or off. Detector diagnostics flag them after a while and put their lanes on recall (red cross).");
                ui.add(egui::Slider::new(&mut s.signal_use, 0.0..=1.0).text("turn-signal use"))
                    .on_hover_text("Drivers who don't signal look like through traffic to the camera");
            });
            ui.label(RichText::new("Controllers only see these sensors — never ground truth. Try turning cameras/radar off and watch the adaptive controllers degrade.").small().color(Color32::from_gray(160)));

            ui.separator();
            ui.heading("Operations");
            ui.checkbox(&mut self.world.sims[self.focus].preemption, "Emergency-vehicle preemption");
            ui.checkbox(&mut self.world.sims[self.focus].rtor, "Right turn on red");
            ui.add_enabled(self.world.sims[self.focus].rtor, egui::Checkbox::new(&mut self.world.sims[self.focus].ntor_peds, "…except while pedestrians are present"))
                .on_hover_text("Blank-out NO TURN ON RED sign lit while someone waits at or crosses a crosswalk the turn passes");
            ui.add(egui::Slider::new(&mut self.world.sims[self.focus].signal.lpi, 0.0..=7.0).step_by(0.5).text("leading ped interval s"))
                .on_hover_text("WALK starts this long before turning traffic that crosses it gets green (0 = off)");
            if self.world.sims[self.focus].signalized() {
                egui::ComboBox::from_label("left turns")
                    .selected_text(self.world.sims[self.focus].left_mode.name())
                    .show_ui(ui, |ui| {
                        for m in LeftMode::ALL {
                            ui.selectable_value(&mut self.world.sims[self.focus].left_mode, m, m.name());
                        }
                    })
                    .response
                    .on_hover_text("Lefts with their own lane: green arrow plus flashing yellow arrow (yield to oncoming), green arrow only, or switch automatically when left x opposing volume is high");
                let prot: Vec<&str> = (0..self.world.sims[self.focus].net.movements.len())
                    .filter(|&m| self.world.sims[self.focus].left_protected[m])
                    .map(|m| self.world.sims[self.focus].net.movements[m].name.as_str())
                    .collect();
                if self.world.sims[self.focus].left_mode != LeftMode::Fya {
                    ui.label(RichText::new(format!("protected-only now: {}", if prot.is_empty() { "none".into() } else { prot.join(", ") })).small().color(Color32::from_gray(160)));
                }
            }
            ui.add(egui::Slider::new(&mut self.world.sims[self.focus].max_wait_guard, 0.0..=300.0).text("max-wait guard s"))
                .on_hover_text("Adaptive controllers are forced to serve any movement waiting longer than this (0 = off)");

            if self.ctrl == CtrlKind::DeepRl && self.mode == ControlMode::Signal {
                ui.separator();
                ui.heading("AI (Deep RL)");
                let layout = self.layout;
                if let Some(rl) = self.world.sims[self.focus].controller.as_rl() {
                    ui.checkbox(&mut rl.learning, "Keep learning online");
                    ui.add_enabled(rl.learning, egui::Slider::new(&mut rl.epsilon, 0.0..=0.5).text("exploration ε"));
                    ui.label(format!("decisions {}  ·  replay {}  ·  updates {}", rl.decisions, rl.agent.replay_len(), rl.agent.updates));
                    let losses: Vec<f32> = rl.agent.losses.iter().copied().collect();
                    if !losses.is_empty() {
                        ui.label("TD loss");
                        charts::sparkline(ui, &losses, 36.0, Color32::from_rgb(230, 120, 200));
                    }
                    let mut saved = None;
                    ui.horizontal(|ui| {
                        if ui.button("💾 Save weights").clicked() {
                            let p = crate::control::rl::weights_path(layout.key());
                            std::fs::create_dir_all(p.parent().unwrap()).ok();
                            saved = Some(rl.save(&p).map(|_| p.display().to_string()));
                        }
                    });
                    if ui.button("⟳ Reload pretrained").clicked() {
                        self.world.sims[self.focus].set_controller(CtrlKind::DeepRl);
                    }
                    if let Some(r) = saved {
                        match r {
                            Ok(p) => self.notify(format!("Saved {p}")),
                            Err(e) => self.notify(format!("Save failed: {e}")),
                        }
                    }
                }
                ui.label(RichText::new("Thorough training: cargo run --release --bin train").small().monospace());
            }
            if self.ctrl == CtrlKind::Hybrid && self.mode == ControlMode::Signal {
                ui.separator();
                ui.heading("AI (RL-tuned predictive)");
                let layout = self.layout;
                let mut saved = None;
                if let Some(h) = self.world.sims[self.focus].controller.as_hybrid() {
                    ui.checkbox(&mut h.learning, "Keep learning online");
                    ui.add_enabled(h.learning, egui::Slider::new(&mut h.epsilon, 0.0..=0.5).text("exploration ε"));
                    ui.label(format!("tuning decisions {}  ·  replay {}  ·  updates {}", h.decisions, h.agent.replay_len(), h.agent.updates));
                    if ui.button("💾 Save weights").clicked() {
                        let p = crate::control::hybrid::weights_path(layout.key());
                        std::fs::create_dir_all(p.parent().unwrap()).ok();
                        saved = Some(h.save(&p).map(|_| p.display().to_string()));
                    }
                }
                if let Some(r) = saved {
                    match r {
                        Ok(p) => self.notify(format!("Saved {p}")),
                        Err(e) => self.notify(format!("Save failed: {e}")),
                    }
                }
                ui.label(RichText::new("Training: cargo run --release --bin train all 400 900 hybrid").small().monospace());
            }

            ui.separator();
            ui.heading("View");
            ui.horizontal_wrapped(|ui| {
                ui.checkbox(&mut self.ov.sensors, "sensors");
                ui.checkbox(&mut self.ov.detections, "detections");
                ui.checkbox(&mut self.ov.conflicts, "conflict points");
                ui.checkbox(&mut self.ov.paths, "paths");
                ui.checkbox(&mut self.ov.labels, "delay labels");
                ui.checkbox(&mut self.ov.night, "night");
            });
            ui.horizontal(|ui| {
                ui.label("Color by");
                ui.selectable_value(&mut self.ov.color_by, ColorBy::Type, "type");
                ui.selectable_value(&mut self.ov.color_by, ColorBy::Delay, "delay");
                ui.selectable_value(&mut self.ov.color_by, ColorBy::Movement, "turn");
                ui.selectable_value(&mut self.ov.color_by, ColorBy::Speed, "speed");
            });
            if ui.button("Fit view").clicked() {
                self.fitted = false;
            }
        });
    }

    fn strategy_name(&self, mode: ControlMode, ctrl: CtrlKind) -> String {
        if mode == ControlMode::Signal && self.layout.signalized() { ctrl.short().to_string() } else { mode.name().to_string() }
    }

    fn kpi(ui: &mut Ui, label: &str, value: String, color: Color32) {
        ui.vertical(|ui| {
            ui.set_min_width(104.0);
            ui.add(egui::Label::new(RichText::new(label).small().color(Color32::from_gray(150))).extend());
            ui.add(egui::Label::new(RichText::new(value).size(17.0).strong().color(color)).extend());
        });
    }

    fn right_panel(&mut self, ui: &mut Ui) {
        self.refresh_summary();
        let summary = self.summary.clone();
        let s = &summary;
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.heading("Vehicle experience");
                if ui.small_button("reset stats").on_hover_text("Measure from now on").clicked() {
                    self.stats_from = self.world.sims[self.focus].t;
                }
            });
            let signalized = self.world.sims[self.focus].signalized();
            let grade = los(s.avg_delay, signalized);
            let gcol = match grade {
                "A" | "B" => Color32::from_rgb(90, 210, 120),
                "C" => Color32::from_rgb(190, 210, 80),
                "D" => Color32::from_rgb(240, 200, 60),
                "E" => Color32::from_rgb(240, 140, 60),
                _ => Color32::from_rgb(235, 70, 60),
            };
            egui::Grid::new("kpis").num_columns(3).spacing(vec2(16.0, 6.0)).show(ui, |ui| {
                Self::kpi(ui, "avg control delay", format!("{:.1} s", s.avg_delay), render::delay_color(s.avg_delay));
                Self::kpi(ui, "95th percentile", format!("{:.0} s", s.p95_delay), render::delay_color(s.p95_delay * 0.6));
                Self::kpi(ui, "level of service", format!("LOS {grade}"), gcol);
                ui.end_row();
                Self::kpi(ui, "throughput", format!("{:.0} veh/h", s.throughput_vph), Color32::WHITE);
                Self::kpi(ui, "stops / vehicle", format!("{:.2}", s.avg_stops), Color32::WHITE);
                Self::kpi(ui, "person delay*", format!("{:.1} s", s.person_delay_all), Color32::WHITE);
                ui.end_row();
                Self::kpi(ui, "max delay", format!("{:.0} s", s.max_delay), Color32::WHITE);
                Self::kpi(ui, "CO₂ / vehicle", format!("{:.0} g", s.co2_per_veh), Color32::WHITE);
                Self::kpi(ui, "ped wait", format!("{:.1} s", s.ped_avg_wait), Color32::WHITE);
                ui.end_row();
            });
            let (n_in, n_q) = self.world.sims[self.focus].in_system();
            ui.label(
                RichText::new(format!(
                    "{} completed · {} in network · {} waiting to enter · {} peds · ped near-misses {:.1}/h · safety overlaps: {}",
                    s.n, n_in, n_q, self.world.sims[self.focus].peds.len(), s.ped_near_miss_ph, self.world.sims[self.focus].collisions
                ))
                .small()
                .color(Color32::from_gray(160)),
            );
            ui.label(RichText::new("* includes vehicles still queued (penalises starving an approach)").small().color(Color32::from_gray(120)));
            if self.twin.is_some() {
                ui.separator();
                ui.label(RichText::new("Side by side (same arrivals)").strong());
                let b = self.twin_summary.clone();
                let a_name = self.strategy_name(self.mode, self.ctrl);
                let b_name = self.strategy_name(self.twin_mode, self.twin_ctrl);
                egui::Grid::new("ab").striped(true).num_columns(3).show(ui, |ui| {
                    ui.label("");
                    ui.label(RichText::new(format!("A: {a_name}")).strong());
                    ui.label(RichText::new(format!("B: {b_name}")).strong());
                    ui.end_row();
                    let rows: [(&str, f64, f64, &str, bool); 7] = [
                        ("person delay*", s.person_delay_all, b.person_delay_all, "s", true),
                        ("avg delay", s.avg_delay, b.avg_delay, "s", true),
                        ("95th pct delay", s.p95_delay, b.p95_delay, "s", true),
                        ("throughput", s.throughput_vph, b.throughput_vph, "veh/h", false),
                        ("stops / veh", s.avg_stops, b.avg_stops, "", true),
                        ("CO₂ / veh", s.co2_per_veh, b.co2_per_veh, "g", true),
                        ("ped wait", s.ped_avg_wait, b.ped_avg_wait, "s", true),
                    ];
                    for (k, a, bv, unit, lower) in rows {
                        let better = |x: f64, y: f64| if lower { x < y * 0.97 } else { x > y * 1.03 };
                        let col = |x: f64, y: f64| if better(x, y) { Color32::from_rgb(110, 220, 130) } else { Color32::from_gray(220) };
                        ui.label(RichText::new(k).color(Color32::from_gray(150)));
                        ui.label(RichText::new(format!("{a:.1} {unit}")).color(col(a, bv)));
                        ui.label(RichText::new(format!("{bv:.1} {unit}")).color(col(bv, a)));
                        ui.end_row();
                    }
                });
            }

            ui.separator();
            let series = &self.world.sims[self.focus].stats.series;
            let xs: Vec<f64> = series.iter().map(|x| x.t).collect();
            charts::line_chart(
                ui,
                "Queues",
                " veh",
                &xs,
                &[
                    ("stopped", series.iter().map(|x| x.queue).collect(), charts::SERIES[1]),
                    ("in network", series.iter().map(|x| x.in_net).collect(), charts::SERIES[0]),
                ],
                110.0,
            );
            charts::line_chart(
                ui,
                "Delay (vehicles finishing in last 60 s)",
                " s",
                &xs,
                &[("avg delay", series.iter().map(|x| x.avg_delay).collect(), charts::SERIES[3])],
                100.0,
            );
            charts::line_chart(
                ui,
                "Throughput (5-min rolling)",
                "",
                &xs,
                &[("veh/h", series.iter().map(|x| x.throughput).collect(), charts::SERIES[2])],
                90.0,
            );
            ui.label(RichText::new("Delay distribution (s)").strong().size(12.5));
            charts::histogram(ui, &self.world.sims[self.focus].stats.hist, HIST_BIN, 80.0);

            ui.separator();
            ui.label(RichText::new("Average delay by approach").strong().size(12.5));
            let rows: Vec<(String, f64, bool)> = self.world.sims[self.focus]
                .net
                .legs
                .iter()
                .enumerate()
                .map(|(i, l)| (format!("{} ({})", l.compass.bound(), s.by_leg.get(i).map(|g| g.n).unwrap_or(0)), s.by_leg.get(i).map(|g| g.avg_delay()).unwrap_or(0.0), false))
                .collect();
            charts::hbars(ui, &rows, " s", true);
            ui.label(RichText::new("Average delay by movement").strong().size(12.5));
            let rows: Vec<(String, f64, bool)> = self.world.sims[self.focus]
                .net
                .movements
                .iter()
                .enumerate()
                .filter(|(i, _)| s.by_move.get(*i).is_some_and(|g| g.n > 0))
                .map(|(i, m)| {
                    let leg = &self.world.sims[self.focus].net.legs[m.leg];
                    (format!("{} {}", leg.compass.bound(), m.turn.name()), s.by_move[i].avg_delay(), false)
                })
                .collect();
            charts::hbars(ui, &rows, " s", true);
            ui.label(RichText::new("By vehicle class").strong().size(12.5));
            egui::Grid::new("bykind").striped(true).num_columns(4).show(ui, |ui| {
                ui.label("class");
                ui.label("n");
                ui.label("delay s");
                ui.label("stops");
                ui.end_row();
                for k in ALL_KINDS {
                    let Some(g) = s.by_kind.get(k.index()) else { continue };
                    if g.n == 0 {
                        continue;
                    }
                    ui.label(k.name());
                    ui.label(g.n.to_string());
                    ui.label(format!("{:.1}", g.avg_delay()));
                    ui.label(format!("{:.2}", g.avg_stops()));
                    ui.end_row();
                }
            });

            if self.world.sims[self.focus].signalized() {
                ui.separator();
                let b = self.world.sims[self.focus].controller.brain();
                ui.heading("Controller brain");
                ui.label(RichText::new(&b.title).strong());
                charts::hbars(ui, &b.bars, "", b.lower_is_better);
                for n in &b.notes {
                    ui.label(RichText::new(n).small().color(Color32::from_gray(170)));
                }
            }

            ui.separator();
            self.selected_panel(ui);

            ui.separator();
            ui.heading("Benchmark");
            ui.label(RichText::new("Runs every control strategy (or sensor package) headless on identical random arrivals with the current demand, 3 seeds × 20 min.").small().color(Color32::from_gray(160)));
            ui.horizontal(|ui| {
                let running = self.bench.as_ref().is_some_and(|b| b.rows.iter().any(|r| r.is_none()));
                if ui.add_enabled(!running, egui::Button::new("▶ Compare controllers")).clicked() {
                    self.start_bench(false);
                }
                if ui.add_enabled(!running && self.world.sims[self.focus].net.kind.signalized(), egui::Button::new("▶ Sensor study")).clicked() {
                    self.start_bench(true);
                }
            });
            if self.bench.is_some() && ui.button(if self.show_bench { "Hide results window" } else { "Show results window" }).clicked() {
                self.show_bench = !self.show_bench;
            }
        });
    }

    fn selected_panel(&mut self, ui: &mut Ui) {
        ui.heading("Selected vehicle");
        let Some(id) = self.selected else {
            ui.label(RichText::new("Click a vehicle to inspect it.").small().color(Color32::from_gray(150)));
            return;
        };
        let Some(v) = self.world.sims[self.focus].vehicles.iter().find(|v| v.id == id) else {
            ui.label("(left the network)");
            return;
        };
        let net = &self.world.sims[self.focus].net;
        let mv = &net.movements[v.movement];
        let src = self.world.sims[self.focus].view.detections.iter().find(|d| d.0 == id).map(|d| match d.1 {
            Source::Camera => "camera",
            Source::Radar => "radar",
            Source::Cv => "connected (V2I)",
            Source::Loop => "loop",
        });
        egui::Grid::new("sel").num_columns(2).show(ui, |ui| {
            let mut row = |k: &str, val: String| {
                ui.label(RichText::new(k).color(Color32::from_gray(150)));
                ui.label(val);
                ui.end_row();
            };
            row("vehicle", format!("#{} {}", v.id, v.kind.name()));
            row("movement", format!("{} {}", net.legs[mv.leg].compass.bound(), mv.turn.name()));
            row("speed", format!("{:.0} km/h (desired {:.0})", v.v * 3.6, v.v0 * 3.6));
            row("doing", v.reason.text().to_string());
            row("delay so far", format!("{:.1} s", v.delay));
            row("stops", v.stops.to_string());
            row("stopped time", format!("{:.1} s", v.stopped_time));
            row("fuel", format!("{:.0} mL", v.fuel_ml));
            row("occupants", format!("{:.1}", v.persons()));
            row("seen by", src.unwrap_or("not detected").to_string());
            if let Some(y) = v.yield_to {
                row("yielding to", format!("#{y}"));
            }
        });
    }

    fn start_bench(&mut self, sensor_study: bool) {
        let kind = self.layout;
        let demand = self.world.sims[self.focus].demand.clone();
        let base = self.world.sims[self.focus].sensors.clone();
        let mut jobs: Vec<(String, ControlMode, CtrlKind, SensorConfig)> = vec![];
        if sensor_study {
            let mut loops = base.clone();
            loops.camera = false;
            loops.radar = false;
            loops.cv = false;
            loops.loops = true;
            let mut cam = loops.clone();
            cam.camera = true;
            let mut rad = cam.clone();
            rad.radar = true;
            let mut cv = rad.clone();
            cv.cv = true;
            cv.cv_rate = 0.3;
            let mut cv_only = loops.clone();
            cv_only.loops = false;
            cv_only.cv = true;
            cv_only.cv_rate = 1.0;
            let packages = [
                ("loops only", loops),
                ("loops + camera", cam),
                ("loops + camera + radar", rad),
                ("all + 30% CV", cv),
                ("100% connected vehicles", cv_only),
            ];
            for c in [CtrlKind::Actuated, CtrlKind::MaxPressure, CtrlKind::Predictive, CtrlKind::DeepRl] {
                for (name, s) in &packages {
                    jobs.push((format!("{} · {}", c.short(), name), ControlMode::Signal, c, s.clone()));
                }
            }
        } else {
            for m in ControlMode::options(kind) {
                if m == ControlMode::Signal {
                    for c in CtrlKind::ALL {
                        jobs.push((c.name().to_string(), m, c, base.clone()));
                    }
                } else {
                    jobs.push((m.name().to_string(), m, CtrlKind::Fixed, base.clone()));
                }
            }
        }
        let (tx, rx) = mpsc::channel();
        let total = jobs.len();
        for (i, (label, mode, ctrl, sensors)) in jobs.into_iter().enumerate() {
            let tx = tx.clone();
            let demand = demand.clone();
            std::thread::spawn(move || {
                let mut acc: Option<Summary> = None;
                let seeds = 3;
                for seed in 0..seeds {
                    let (s, _) = crate::run_scenario(kind, mode, ctrl, &demand, &sensors, 500 + seed, 300.0, 1200.0);
                    acc = Some(match acc {
                        None => s,
                        Some(mut a) => {
                            a.avg_delay += s.avg_delay;
                            a.p95_delay += s.p95_delay;
                            a.throughput_vph += s.throughput_vph;
                            a.avg_stops += s.avg_stops;
                            a.person_delay_all += s.person_delay_all;
                            a.co2_per_veh += s.co2_per_veh;
                            a.ped_avg_wait += s.ped_avg_wait;
                            a.max_delay = a.max_delay.max(s.max_delay);
                            a
                        }
                    });
                }
                let mut s = acc.unwrap();
                let k = seeds as f64;
                s.avg_delay /= k;
                s.p95_delay /= k;
                s.throughput_vph /= k;
                s.avg_stops /= k;
                s.person_delay_all /= k;
                s.co2_per_veh /= k;
                s.ped_avg_wait /= k;
                tx.send((i, BenchRow { label, summary: s })).ok();
            });
        }
        self.bench = Some(Bench {
            title: if sensor_study { format!("Sensor study — {}", kind.name()) } else { format!("Controller comparison — {}", kind.name()) },
            rx,
            total,
            rows: (0..total).map(|_| None).collect(),
        });
        self.show_bench = true;
    }

    fn bench_window(&mut self, ctx: &egui::Context) {
        let Some(b) = self.bench.as_mut() else { return };
        while let Ok((i, row)) = b.rx.try_recv() {
            b.rows[i] = Some(row);
        }
        if !self.show_bench {
            return;
        }
        let mut open = true;
        egui::Window::new(b.title.clone()).open(&mut open).default_width(640.0).show(ctx, |ui| {
            let done = b.rows.iter().filter(|r| r.is_some()).count();
            if done < b.total {
                ui.horizontal(|ui| {
                    ui.spinner();
                    ui.label(format!("running {done}/{}", b.total));
                });
                ctx.request_repaint();
            }
            let rows: Vec<&BenchRow> = b.rows.iter().flatten().collect();
            if rows.is_empty() {
                return;
            }
            ui.label(RichText::new("Person delay incl. unserved (s) — lower is better").strong());
            let bars: Vec<(String, f64, bool)> = rows.iter().map(|r| (r.label.clone(), r.summary.person_delay_all, false)).collect();
            charts::hbars(ui, &bars, " s", true);
            ui.separator();
            egui::ScrollArea::vertical().max_height(300.0).show(ui, |ui| {
                egui::Grid::new("bench").striped(true).num_columns(8).show(ui, |ui| {
                    for h in ["strategy", "delay s", "p95 s", "veh/h", "stops", "pers dly*", "CO₂ g", "ped wait"] {
                        ui.label(RichText::new(h).strong());
                    }
                    ui.end_row();
                    for r in &rows {
                        let s = &r.summary;
                        ui.label(&r.label);
                        ui.label(format!("{:.1}", s.avg_delay));
                        ui.label(format!("{:.0}", s.p95_delay));
                        ui.label(format!("{:.0}", s.throughput_vph));
                        ui.label(format!("{:.2}", s.avg_stops));
                        ui.label(format!("{:.1}", s.person_delay_all));
                        ui.label(format!("{:.0}", s.co2_per_veh));
                        ui.label(format!("{:.1}", s.ped_avg_wait));
                        ui.end_row();
                    }
                });
            });
        });
        if !open {
            self.show_bench = false;
        }
    }

    fn bottom_panel(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            ui.label(RichText::new("Signal timing (last 3 min)").strong().size(12.0));
            if let Some((m, t)) = &self.message {
                if t.elapsed().as_secs() < 5 {
                    ui.separator();
                    ui.label(RichText::new(m).color(Color32::from_rgb(255, 210, 120)));
                }
            }
        });
        charts::timeline(ui, &self.world.sims[self.focus], 180.0);
    }

    // ------------------------------------------------------------- canvas

    fn canvas(&mut self, ui: &mut Ui) {
        let (resp, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
        let rect = resp.rect;
        if !self.fitted {
            let net = &self.world.sims[self.focus].net;
            let extent = net.legs.iter().map(|l| l.stop_dist).fold(0.0, f64::max) + 55.0;
            self.scale = (rect.width().min(rect.height()) / (2.0 * extent as f32)).max(1.0);
            if let Some(z) = std::env::var("TS_ZOOM").ok().and_then(|v| v.parse::<f32>().ok()) {
                self.scale *= z;
            }
            self.cam_center = v2(0.0, 0.0);
            self.fitted = true;
        }
        // Pan and zoom.
        if resp.dragged() && self.tool == Tool::Select {
            let d = resp.drag_delta();
            self.cam_center = self.cam_center - v2((d.x / self.scale) as f64, (d.y / self.scale) as f64);
        }
        if resp.hovered() {
            let (scroll, zoom) = ui.input(|i| (i.smooth_scroll_delta.y, i.zoom_delta()));
            let factor = zoom * (scroll * 0.0025).exp();
            if (factor - 1.0).abs() > 1e-4 {
                if let Some(h) = resp.hover_pos() {
                    let cam = Camera { center: self.cam_center, scale: self.scale, rect };
                    let before = cam.to_world(h);
                    self.scale = (self.scale * factor).clamp(0.6, 40.0);
                    let cam2 = Camera { center: self.cam_center, scale: self.scale, rect };
                    let after = cam2.to_world(h);
                    self.cam_center = self.cam_center + (before - after);
                }
            }
        }
        // Side by side: A on the left half, B on the right, same camera.
        let halves = if self.twin.is_some() {
            let mid = rect.center().x;
            vec![egui::Rect::from_min_max(rect.min, egui::pos2(mid - 1.0, rect.max.y)), egui::Rect::from_min_max(egui::pos2(mid + 1.0, rect.min.y), rect.max)]
        } else {
            vec![rect]
        };
        painter.rect_filled(rect, 0.0, Color32::from_rgb(20, 22, 26));
        let alpha = if self.paused { 1.0 } else { self.acc / self.world.dt() };
        for (k, half) in halves.iter().enumerate() {
            let (world, mode, ctrl) = if k == 0 {
                (&self.world, self.mode, self.ctrl)
            } else {
                (self.twin.as_ref().unwrap(), self.twin_mode, self.twin_ctrl)
            };
            let label = match (self.twin.is_some(), k) {
                (false, _) => self.strategy_name(mode, ctrl),
                (true, 0) => format!("A · {}", self.strategy_name(mode, ctrl)),
                _ => format!("B · {}", self.strategy_name(mode, ctrl)),
            };
            self.paint_world(&painter.with_clip_rect(*half), world, *half, &label, alpha, k == 0);
        }
        if self.twin.is_some() {
            painter.vline(rect.center().x, rect.y_range(), egui::Stroke::new(2.0, Color32::from_gray(15)));
        }
        self.legend(&painter, halves[0]);

        // Clicks: select / spawn (spawns go to both sides so arrivals stay identical).
        if resp.clicked() {
            if let Some(p) = resp.interact_pointer_pos() {
                let k = halves.iter().position(|h| h.contains(p)).unwrap_or(0);
                let cam = Camera { center: self.cam_center, scale: self.scale, rect: halves[k] };
                let w = cam.to_world(p);
                let (i, local) = self.locate(w);
                match self.tool {
                    Tool::Select if k == 0 => {
                        let sim = &self.world.sims[i];
                        self.selected = sim
                            .vehicles
                            .iter()
                            .map(|v| (v.id, sim.center(v).dist(local) - v.len() * 0.5))
                            .filter(|(_, d)| *d < 2.5)
                            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                            .map(|x| x.0);
                        if self.selected.is_some() {
                            self.focus = i;
                        }
                    }
                    Tool::Select => {}
                    Tool::Vehicle => {
                        let net = &self.world.sims[i].net;
                        let leg = (0..net.legs.len())
                            .map(|li| {
                                let l = &net.legs[li];
                                let r = local.dot(l.d);
                                let lat = local.dot(l.in_side());
                                let ok = r > l.stop_dist * 0.6 && lat > -l.out_width - 12.0 && lat < l.in_width + 12.0;
                                (li, ok, (lat - l.in_width * 0.5).abs())
                            })
                            .filter(|x| x.1)
                            .min_by(|a, b| a.2.partial_cmp(&b.2).unwrap())
                            .map(|x| x.0);
                        if let Some(li) = leg {
                            let ok = self.world.sims[i].spawn_manual(li, self.spawn_kind, self.spawn_turn);
                            if let Some(t) = self.twin.as_mut() {
                                t.sims[i].spawn_manual(li, self.spawn_kind, self.spawn_turn);
                            }
                            let bound = self.world.sims[i].net.legs[li].compass.bound();
                            if !ok {
                                self.notify(format!("{} has no {} movement", bound, self.spawn_turn.name()));
                            } else {
                                self.notify(format!("{} {} queued on {} (enters 260 m upstream)", self.spawn_kind.name(), self.spawn_turn.name(), bound));
                            }
                        }
                    }
                    Tool::Pedestrian => {
                        let best = self.world.sims[i]
                            .net
                            .crosswalks
                            .iter()
                            .flat_map(|c| [(c.id, false, c.a.dist(local)), (c.id, true, c.b.dist(local))])
                            .min_by(|a, b| a.2.partial_cmp(&b.2).unwrap());
                        if let Some((cw, b, d)) = best {
                            if d < 12.0 {
                                self.world.sims[i].spawn_ped(cw, b);
                                if let Some(t) = self.twin.as_mut() {
                                    t.sims[i].spawn_ped(cw, b);
                                }
                            }
                        }
                    }
                }
            }
        }
        if self.tool != Tool::Select {
            if let Some(h) = resp.hover_pos() {
                let txt = match self.tool {
                    Tool::Vehicle => format!("click an approach: {} {}", self.spawn_kind.name(), self.spawn_turn.name()),
                    _ => "click a crosswalk corner".to_string(),
                };
                painter.text(h + vec2(14.0, 10.0), egui::Align2::LEFT_TOP, txt, egui::FontId::proportional(12.0), Color32::WHITE);
            }
        }
    }

    /// Intersection nearest to world point `w`, and `w` in its local coordinates.
    fn locate(&self, w: crate::sim::geom::V2) -> (usize, crate::sim::geom::V2) {
        let i = (0..self.world.offsets.len())
            .min_by(|&a, &b| w.dist(self.world.offsets[a]).partial_cmp(&w.dist(self.world.offsets[b])).unwrap())
            .unwrap_or(0);
        (i, w - self.world.offsets[i])
    }

    fn paint_world(&self, painter: &egui::Painter, world: &World, rect: egui::Rect, label: &str, alpha: f64, main: bool) {
        painter.rect_filled(rect, 0.0, render::GRASS);
        let cam_for = |i: usize| Camera { center: self.cam_center - world.offsets[i], scale: self.scale, rect };
        for i in 0..world.sims.len() {
            self.scene.paint(painter, &cam_for(i), self.ov.night);
        }
        for (i, sim) in world.sims.iter().enumerate() {
            let selected = if main && i == self.focus { self.selected } else { None };
            let dc = DrawCtx { sim, cam: cam_for(i), ov: &self.ov, selected, time: self.anim_t, alpha };
            render::paint_dynamic(painter, &dc);
            if self.ov.detections {
                render::paint_detections(painter, &dc);
            }
        }
        let focus = self.focus.min(world.sims.len() - 1);
        render::hud(painter, rect, &world.sims[focus], label, self.speed, self.paused);
    }

    fn legend(&self, painter: &egui::Painter, rect: egui::Rect) {
        let mut items: Vec<(Color32, &str)> = vec![];
        if self.ov.sensors && self.world.sims[self.focus].sensors.loops {
            items.push((Color32::from_rgb(255, 200, 60), "loop detector"));
        }
        if self.ov.detections {
            if self.world.sims[self.focus].sensors.camera {
                items.push((Color32::from_rgb(90, 220, 255), "camera track"));
            }
            if self.world.sims[self.focus].sensors.radar {
                items.push((Color32::from_rgb(190, 110, 255), "radar track"));
            }
            if self.world.sims[self.focus].sensors.cv {
                items.push((Color32::from_rgb(120, 255, 160), "connected vehicle"));
            }
        }
        let mut y = rect.bottom() - 10.0 - items.len() as f32 * 16.0;
        for (c, t) in items {
            let p = egui::pos2(rect.left() + 14.0, y);
            painter.circle_filled(p, 4.5, c);
            painter.text(p + vec2(10.0, 0.0), egui::Align2::LEFT_CENTER, t, egui::FontId::proportional(11.5), Color32::from_gray(230));
            y += 16.0;
        }
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.advance(&ctx);
        if ui.input(|i| i.key_pressed(egui::Key::Space)) {
            self.paused = !self.paused;
        }
        egui::Panel::top("top").show(ui, |ui| self.top_bar(ui));
        egui::Panel::left("left").default_size(300.0).resizable(true).show(ui, |ui| self.left_panel(ui));
        self.sync_settings();
        egui::Panel::right("right").default_size(380.0).resizable(true).show(ui, |ui| self.right_panel(ui));
        egui::Panel::bottom("bottom").show(ui, |ui| self.bottom_panel(ui));
        egui::CentralPanel::default().frame(egui::Frame::NONE).show(ui, |ui| self.canvas(ui));
        self.bench_window(&ctx);
        if let Some((path, after)) = self.shot.clone() {
            if self.world.sims[self.focus].t >= after {
                self.shot = None;
                shot::request(&ctx, path);
            }
        }
    }
}
