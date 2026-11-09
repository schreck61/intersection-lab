// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Small hand-painted charts (line, bars, histograms, signal and crossing timelines) in
//! egui.

use egui::{Align2, Color32, FontId, Pos2, Rect, Sense, Stroke, Ui, pos2, vec2};

pub const SERIES: [Color32; 6] = [
    Color32::from_rgb(90, 170, 255),
    Color32::from_rgb(255, 150, 70),
    Color32::from_rgb(110, 210, 130),
    Color32::from_rgb(230, 100, 160),
    Color32::from_rgb(200, 190, 90),
    Color32::from_rgb(160, 130, 255),
];

const GRID: Color32 = Color32::from_rgb(52, 56, 64);
const AXIS_TXT: Color32 = Color32::from_rgb(150, 156, 168);
/// Size of axis labels, legends and values; 11 px keeps them legible.
const TXT: f32 = 11.0;

fn nice_max(v: f64) -> f64 {
    if v <= 0.0 {
        return 1.0;
    }
    let p = 10f64.powf(v.log10().floor());
    for m in [1.0, 2.0, 2.5, 5.0, 10.0] {
        if v <= m * p {
            return m * p;
        }
    }
    10.0 * p
}

/// Line chart of one or more series sharing the x axis.
pub fn line_chart(ui: &mut Ui, title: &str, unit: &str, xs: &[f64], series: &[(&str, Vec<f64>, Color32)], height: f32) {
    ui.label(egui::RichText::new(title).strong().size(12.5));
    let (rect, resp) = ui.allocate_exact_size(vec2(ui.available_width(), height), Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 4.0, Color32::from_rgb(24, 26, 31));
    if xs.len() < 2 {
        p.text(rect.center(), Align2::CENTER_CENTER, "collecting data…", FontId::proportional(11.0), AXIS_TXT);
        return;
    }
    let plot = Rect::from_min_max(rect.min + vec2(38.0, 6.0), rect.max - vec2(6.0, 17.0));
    let ymax = nice_max(series.iter().flat_map(|s| s.1.iter().cloned()).fold(0.0, f64::max) * 1.05);
    let (x0, x1) = (xs[0], *xs.last().unwrap());
    let tx = |x: f64| plot.left() + ((x - x0) / (x1 - x0).max(1e-9)) as f32 * plot.width();
    let ty = |y: f64| plot.bottom() - (y / ymax) as f32 * plot.height();
    for k in 0..=4 {
        let y = ymax * k as f64 / 4.0;
        let yy = ty(y);
        p.line_segment([pos2(plot.left(), yy), pos2(plot.right(), yy)], Stroke::new(1.0, GRID));
        p.text(pos2(plot.left() - 4.0, yy), Align2::RIGHT_CENTER, fmt_num(y), FontId::proportional(TXT), AXIS_TXT);
    }
    let mins = |x: f64| format!("{:.0}m", x / 60.0);
    p.text(pos2(plot.left(), plot.bottom() + 2.0), Align2::LEFT_TOP, mins(x0), FontId::proportional(TXT), AXIS_TXT);
    p.text(pos2(plot.right(), plot.bottom() + 2.0), Align2::RIGHT_TOP, mins(x1), FontId::proportional(TXT), AXIS_TXT);
    for (_, ys, col) in series {
        let pts: Vec<Pos2> = xs.iter().zip(ys).map(|(x, y)| pos2(tx(*x), ty(*y))).collect();
        p.add(egui::Shape::line(pts, Stroke::new(1.6, *col)));
    }
    // Legend + hover readout.
    let mut lx = plot.left() + 4.0;
    for (name, ys, col) in series {
        let last = ys.last().copied().unwrap_or(0.0);
        let r = p.text(pos2(lx, plot.top()), Align2::LEFT_TOP, format!("● {name} {}{unit}", fmt_num(last)), FontId::proportional(TXT), *col);
        lx = r.right() + 10.0;
    }
    if let Some(h) = resp.hover_pos() && plot.contains(h) {
        let xv = x0 + ((h.x - plot.left()) / plot.width()) as f64 * (x1 - x0);
        let i = xs.partition_point(|&x| x < xv).min(xs.len() - 1);
        p.line_segment([pos2(tx(xs[i]), plot.top()), pos2(tx(xs[i]), plot.bottom())], Stroke::new(1.0, Color32::from_white_alpha(40)));
        let mut txt = format!("t={:.1} min", xs[i] / 60.0);
        for (name, ys, _) in series {
            txt.push_str(&format!("\n{name}: {}{unit}", fmt_num(ys[i])));
        }
        let pos = pos2((tx(xs[i]) + 8.0).min(plot.right() - 90.0), plot.top() + 14.0);
        let g = p.layout_no_wrap(txt, FontId::proportional(TXT), Color32::WHITE);
        p.rect_filled(Rect::from_min_size(pos - vec2(3.0, 2.0), g.size() + vec2(6.0, 4.0)), 3.0, Color32::from_black_alpha(200));
        p.galley(pos, g, Color32::WHITE);
    }
}

pub fn fmt_num(v: f64) -> String {
    if v.abs() >= 100.0 {
        format!("{:.0}", v)
    } else if v.abs() >= 10.0 {
        format!("{:.1}", v)
    } else {
        format!("{:.2}", v)
    }
}

/// Horizontal bars with labels; `highlight` draws an accent bar.
pub fn hbars(ui: &mut Ui, rows: &[(String, f64, bool)], unit: &str, lower_is_better: bool) {
    if rows.is_empty() {
        return;
    }
    let finite: Vec<f64> = rows.iter().map(|r| r.1).filter(|v| v.is_finite()).collect();
    let vmax = finite.iter().cloned().fold(0.0, f64::max).max(1e-9);
    let vmin = finite.iter().cloned().fold(f64::INFINITY, f64::min);
    let neg = vmin < 0.0;
    let row_h = 17.0;
    let (rect, _) = ui.allocate_exact_size(vec2(ui.available_width(), rows.len() as f32 * row_h + 2.0), Sense::hover());
    let p = ui.painter_at(rect);
    let label_w = (rect.width() * 0.42).min(170.0);
    let best = if lower_is_better {
        finite.iter().cloned().fold(f64::INFINITY, f64::min)
    } else {
        finite.iter().cloned().fold(f64::NEG_INFINITY, f64::max)
    };
    for (i, (name, v, hl)) in rows.iter().enumerate() {
        let y = rect.top() + i as f32 * row_h;
        let bar_area = Rect::from_min_max(pos2(rect.left() + label_w, y + 2.0), pos2(rect.right() - 44.0, y + row_h - 2.0));
        p.text(pos2(rect.left(), y + row_h / 2.0), Align2::LEFT_CENTER, name, FontId::proportional(TXT), if *hl { Color32::WHITE } else { AXIS_TXT });
        p.rect_filled(bar_area, 2.0, Color32::from_rgb(30, 33, 39));
        if v.is_finite() {
            let frac = if neg {
                ((v - vmin) / (vmax - vmin).max(1e-9)) as f32
            } else {
                (v / vmax) as f32
            };
            let w = bar_area.width() * frac.clamp(0.0, 1.0);
            let col = if *hl {
                Color32::from_rgb(255, 200, 70)
            } else if (*v - best).abs() < 1e-9 {
                Color32::from_rgb(110, 210, 130)
            } else {
                Color32::from_rgb(90, 140, 220)
            };
            p.rect_filled(Rect::from_min_size(bar_area.min, vec2(w.max(1.0), bar_area.height())), 2.0, col);
            p.text(pos2(rect.right(), y + row_h / 2.0), Align2::RIGHT_CENTER, format!("{}{unit}", fmt_num(*v)), FontId::proportional(TXT), Color32::WHITE);
        } else {
            p.text(pos2(rect.right(), y + row_h / 2.0), Align2::RIGHT_CENTER, "—", FontId::proportional(TXT), AXIS_TXT);
        }
    }
}

pub fn histogram(ui: &mut Ui, bins: &[u32], bin_w: f64, height: f32) {
    let (rect, resp) = ui.allocate_exact_size(vec2(ui.available_width(), height), Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 4.0, Color32::from_rgb(24, 26, 31));
    let total: u32 = bins.iter().sum();
    let max = *bins.iter().max().unwrap_or(&1) as f32;
    let plot = Rect::from_min_max(rect.min + vec2(6.0, 6.0), rect.max - vec2(6.0, 17.0));
    let bw = plot.width() / bins.len() as f32;
    for (i, &b) in bins.iter().enumerate() {
        let h = plot.height() * b as f32 / max.max(1.0);
        let x = plot.left() + i as f32 * bw;
        let col = super::render::delay_color(i as f64 * bin_w + bin_w / 2.0);
        p.rect_filled(Rect::from_min_max(pos2(x + 1.0, plot.bottom() - h), pos2(x + bw - 1.0, plot.bottom())), 2.0, col);
        if i % 2 == 0 {
            let label = if i == bins.len() - 1 { format!("{:.0}+", i as f64 * bin_w) } else { format!("{:.0}", i as f64 * bin_w) };
            p.text(pos2(x + bw / 2.0, plot.bottom() + 2.0), Align2::CENTER_TOP, label, FontId::proportional(TXT), AXIS_TXT);
        }
    }
    if let Some(h) = resp.hover_pos() && plot.contains(h) {
        let i = (((h.x - plot.left()) / bw) as usize).min(bins.len() - 1);
        let pct = if total > 0 { 100.0 * bins[i] as f32 / total as f32 } else { 0.0 };
        p.text(pos2(plot.right(), plot.top()), Align2::RIGHT_TOP, format!("{:.0}–{:.0}s: {} veh ({:.0}%)", i as f64 * bin_w, (i + 1) as f64 * bin_w, bins[i], pct), FontId::proportional(TXT), Color32::WHITE);
    }
}

/// Histogram of post-encroachment times, the closest calls (shortest times) in red.
pub fn pet_histogram(ui: &mut Ui, bins: &[u32], bin_w: f64, height: f32) {
    let (rect, resp) = ui.allocate_exact_size(vec2(ui.available_width(), height), Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 4.0, Color32::from_rgb(24, 26, 31));
    let total: u32 = bins.iter().sum();
    if total == 0 {
        p.text(rect.center(), Align2::CENTER_CENTER, "no crossings yet", FontId::proportional(TXT), AXIS_TXT);
        return;
    }
    // The last bin collects everything longer; it is clipped so the short times, the
    // ones that matter, keep the scale.
    let last = bins.len() - 1;
    let max = bins[..last].iter().copied().max().unwrap_or(1).max(1) as f32;
    let plot = Rect::from_min_max(rect.min + vec2(6.0, 6.0), rect.max - vec2(6.0, 17.0));
    let bw = plot.width() / bins.len() as f32;
    for (i, &b) in bins.iter().enumerate() {
        let h = (plot.height() * b as f32 / max).min(plot.height());
        let x = plot.left() + i as f32 * bw;
        if i == last && b as f32 > max {
            p.text(pos2(x + bw / 2.0, plot.top() + 2.0), Align2::CENTER_TOP, "▲", FontId::proportional(TXT), AXIS_TXT);
        }
        let mid = (i as f64 + 0.5) * bin_w;
        let col = super::render::delay_color(90.0 * (1.0 - (mid / 2.0).min(1.0)));
        p.rect_filled(Rect::from_min_max(pos2(x + 1.0, plot.bottom() - h), pos2(x + bw - 1.0, plot.bottom())), 2.0, col);
        let at = i as f64 * bin_w;
        if (at - at.round()).abs() < 1e-9 {
            p.text(pos2(x, plot.bottom() + 2.0), Align2::CENTER_TOP, format!("{at:.0} s"), FontId::proportional(TXT), AXIS_TXT);
        }
    }
    if let Some(h) = resp.hover_pos() && plot.contains(h) {
        let i = (((h.x - plot.left()) / bw) as usize).min(bins.len() - 1);
        let pct = 100.0 * bins[i] as f32 / total as f32;
        let range = if i + 1 == bins.len() { format!("{:.2} s or more", i as f64 * bin_w) } else { format!("{:.2}–{:.2} s", i as f64 * bin_w, (i + 1) as f64 * bin_w) };
        p.text(pos2(plot.right(), plot.top()), Align2::RIGHT_TOP, format!("{range}: {} ({pct:.1}%)", bins[i]), FontId::proportional(TXT), Color32::WHITE);
    }
}

/// Stop-line crossings at the cooperative crossing: one row per movement through the
/// box, in the managers' service order, with a tick for each vehicle and a bar for each
/// phase from its first crossing to its last, over the last `past` seconds and the
/// `ahead` seconds already scheduled.
pub fn crossing_timeline(ui: &mut Ui, sim: &crate::sim::Sim, past: f64, ahead: f64) {
    let Some(c) = sim.coop.as_ref() else { return };
    let net = &sim.net;
    let rows: Vec<usize> = c.ring_order().concat();
    let row_h = 14.0;
    let (rect, _) = ui.allocate_exact_size(vec2(ui.available_width(), rows.len().max(1) as f32 * row_h + 21.0), Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 4.0, Color32::from_rgb(22, 24, 28));
    let label_w = 150.0;
    let plot = Rect::from_min_max(rect.min + vec2(label_w, 4.0), rect.max - vec2(8.0, 17.0));
    let now = sim.t;
    let (t0, t1) = (now - past, now + ahead);
    let tx = |t: f64| plot.left() + ((t - t0) / (t1 - t0)) as f32 * plot.width();
    let row_of = |m: usize| rows.iter().position(|&x| x == m);
    let color = |m: usize| if net.movements[m].turn == crate::sim::network::Turn::L { SERIES[1] } else { SERIES[0] };
    let open = c.open_movements();
    for (i, &m) in rows.iter().enumerate() {
        let y = plot.top() + i as f32 * row_h;
        let mv = &net.movements[m];
        let name = format!("{} {}", net.legs[mv.leg].compass.bound(), mv.turn.name().to_lowercase());
        let col = if open.contains(&m) { Color32::WHITE } else { AXIS_TXT };
        p.text(pos2(rect.left() + 6.0, y + row_h / 2.0), Align2::LEFT_CENTER, name, FontId::proportional(TXT), col);
        p.line_segment([pos2(plot.left(), y + row_h - 0.5), pos2(plot.right(), y + row_h - 0.5)], Stroke::new(1.0, Color32::from_rgb(34, 37, 43)));
    }
    let mut t = (t1 / 30.0).floor() * 30.0;
    while t >= t0 {
        if t >= 0.0 {
            p.line_segment([pos2(tx(t), plot.top()), pos2(tx(t), plot.bottom())], Stroke::new(1.0, Color32::from_rgb(40, 44, 50)));
            p.text(pos2(tx(t), plot.bottom() + 1.0), Align2::CENTER_TOP, format!("{:.0}:{:02.0}", (t / 60.0).floor(), t % 60.0), FontId::proportional(TXT), AXIS_TXT);
        }
        t -= 30.0;
    }
    let open_batches = c.open_batches();
    for b in c.history.iter().chain(&open_batches) {
        if b.end < t0 || b.start > t1 {
            continue;
        }
        let Some(i) = row_of(b.movement) else { continue };
        let y = plot.top() + i as f32 * row_h;
        let (a, e) = (tx(b.start.max(t0)), tx(b.end.min(t1)));
        p.rect_filled(Rect::from_min_max(pos2(a - 1.0, y + 1.5), pos2(e.max(a) + 1.0, y + row_h - 2.5)), 2.0, color(b.movement).gamma_multiply(0.45));
    }
    for &(t, m) in &c.crossings {
        if t < t0 || t > t1 {
            continue;
        }
        let Some(i) = row_of(m) else { continue };
        let y = plot.top() + i as f32 * row_h;
        // Crossings still ahead are reservations: dimmer.
        let col = if t > now { Color32::from_white_alpha(90) } else { Color32::from_rgb(235, 238, 245) };
        p.line_segment([pos2(tx(t), y + 2.0), pos2(tx(t), y + row_h - 3.0)], Stroke::new(1.0, col));
    }
    let x = tx(now);
    p.line_segment([pos2(x, plot.top()), pos2(x, plot.bottom())], Stroke::new(1.5, super::render::SCHEDULED));
    p.text(pos2(x + 3.0, plot.top()), Align2::LEFT_TOP, "now", FontId::proportional(TXT), super::render::SCHEDULED);
}

/// Signal timing diagram: one row per stage over the last `window` seconds.
pub fn timeline(ui: &mut Ui, sim: &crate::sim::Sim, window: f64) {
    let net = &sim.net;
    let n = net.stages.len().max(1);
    let row_h = 14.0;
    let (rect, _) = ui.allocate_exact_size(vec2(ui.available_width(), n as f32 * row_h + 21.0), Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 4.0, Color32::from_rgb(22, 24, 28));
    if !sim.signalized() {
        p.text(rect.center(), Align2::CENTER_CENTER, format!("{} — no signal timing", sim.mode.name()), FontId::proportional(12.0), AXIS_TXT);
        return;
    }
    let label_w = 150.0;
    let plot = Rect::from_min_max(rect.min + vec2(label_w, 4.0), rect.max - vec2(8.0, 17.0));
    let t1 = sim.t;
    let t0 = (t1 - window).max(0.0);
    let tx = |t: f64| plot.left() + ((t - t0) / window) as f32 * plot.width();
    for (i, st) in net.stages.iter().enumerate() {
        let y = plot.top() + i as f32 * row_h;
        let cur = i == sim.signal.stage;
        p.text(pos2(rect.left() + 6.0, y + row_h / 2.0), Align2::LEFT_CENTER, &st.name, FontId::proportional(TXT), if cur { Color32::WHITE } else { AXIS_TXT });
        p.line_segment([pos2(plot.left(), y + row_h - 0.5), pos2(plot.right(), y + row_h - 0.5)], Stroke::new(1.0, Color32::from_rgb(34, 37, 43)));
    }
    let k = (window / 30.0) as i64;
    for j in 0..=k {
        let t = (t1 / 30.0).floor() * 30.0 - j as f64 * 30.0;
        if t >= t0 {
            p.line_segment([pos2(tx(t), plot.top()), pos2(tx(t), plot.bottom())], Stroke::new(1.0, Color32::from_rgb(40, 44, 50)));
            p.text(pos2(tx(t), plot.bottom() + 1.0), Align2::CENTER_TOP, format!("{:.0}:{:02.0}", (t / 60.0).floor(), t % 60.0), FontId::proportional(TXT), AXIS_TXT);
        }
    }
    let hist: Vec<_> = sim.signal.history.iter().collect();
    for (hi, h) in hist.iter().enumerate() {
        if h.end < t0 {
            continue;
        }
        let y = plot.top() + h.stage as f32 * row_h;
        let a = tx(h.start.max(t0));
        let b = tx(h.end);
        p.rect_filled(Rect::from_min_max(pos2(a, y + 1.5), pos2(b.max(a + 1.0), y + row_h - 2.5)), 2.0, SERIES[h.stage % SERIES.len()]);
        // Clearance gap (yellow + all-red) before the next stage.
        if let Some(nx) = hist.get(hi + 1) && nx.start > h.end + 0.05 {
            let ya = plot.top() + h.stage as f32 * row_h;
            p.rect_filled(Rect::from_min_max(pos2(b, ya + 1.5), pos2(tx(nx.start).max(b + 1.0), ya + row_h - 2.5)), 1.0, Color32::from_rgb(230, 190, 40));
        }
    }
    if sim.signal.phase != crate::sim::signal::Phase::Green {
        let y = plot.top() + sim.signal.stage as f32 * row_h;
        let a = tx(sim.signal.history.back().map(|h| h.end).unwrap_or(t1));
        let col = if sim.signal.phase == crate::sim::signal::Phase::Yellow { Color32::from_rgb(230, 190, 40) } else { Color32::from_rgb(220, 60, 50) };
        p.rect_filled(Rect::from_min_max(pos2(a, y + 1.5), pos2(tx(t1).max(a + 1.0), y + row_h - 2.5)), 1.0, col);
    }
}

/// Tiny sparkline for loss curves etc.
pub fn sparkline(ui: &mut Ui, ys: &[f32], height: f32, col: Color32) {
    let (rect, _) = ui.allocate_exact_size(vec2(ui.available_width(), height), Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 3.0, Color32::from_rgb(24, 26, 31));
    if ys.len() < 2 {
        return;
    }
    let max = ys.iter().cloned().fold(f32::MIN, f32::max).max(1e-6);
    let pts: Vec<Pos2> = ys
        .iter()
        .enumerate()
        .map(|(i, y)| pos2(rect.left() + i as f32 / (ys.len() - 1) as f32 * rect.width(), rect.bottom() - y / max * (rect.height() - 4.0) - 2.0))
        .collect();
    p.add(egui::Shape::line(pts, Stroke::new(1.2, col)));
}
