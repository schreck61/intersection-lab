// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Top-down rendering of the intersection: a cached world-space mesh for the static
//! scene (terrain, roads, markings, buildings, trees) and per-frame shapes for
//! signals, sensors, vehicles and pedestrians.

use crate::sim::geom::{V2, v2};
use crate::sim::network::{LayoutKind, LinkKind, Network, Turn};
use crate::sim::sensors::Source;
use crate::sim::signal::{PedSig, Phase, Sig};
use crate::sim::vtypes::VKind;
use crate::sim::{ControlMode, PedState, Reason, Sim, Vehicle};
use egui::epaint::{Mesh, PathShape, PathStroke};
use egui::{Align2, Color32, FontId, Pos2, Rect, Shape, Stroke, pos2};

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ColorBy {
    Type,
    Delay,
    Movement,
    Speed,
}

#[derive(Clone)]
pub struct Overlays {
    pub sensors: bool,
    pub detections: bool,
    pub conflicts: bool,
    pub paths: bool,
    pub labels: bool,
    pub night: bool,
    pub color_by: ColorBy,
}

impl Default for Overlays {
    fn default() -> Self {
        Overlays {
            sensors: true,
            detections: true,
            conflicts: false,
            paths: false,
            labels: false,
            night: false,
            color_by: ColorBy::Type,
        }
    }
}

#[derive(Clone, Copy)]
pub struct Camera {
    pub center: V2,
    pub scale: f32,
    pub rect: Rect,
}

impl Camera {
    pub fn to_screen(&self, p: V2) -> Pos2 {
        let c = self.rect.center();
        pos2(c.x + (p.x as f32 - self.center.x as f32) * self.scale, c.y + (p.y as f32 - self.center.y as f32) * self.scale)
    }
    pub fn to_world(&self, p: Pos2) -> V2 {
        let c = self.rect.center();
        v2(self.center.x + ((p.x - c.x) / self.scale) as f64, self.center.y + ((p.y - c.y) / self.scale) as f64)
    }
}

// ------------------------------------------------------------------ palette

pub const GRASS: Color32 = Color32::from_rgb(78, 122, 64);
const GRASS2: Color32 = Color32::from_rgb(70, 112, 58);
const ASPHALT: Color32 = Color32::from_rgb(54, 57, 62);
const ASPHALT_LIGHT: Color32 = Color32::from_rgb(62, 65, 71);
const SIDEWALK: Color32 = Color32::from_rgb(170, 168, 160);
const CURB: Color32 = Color32::from_rgb(205, 203, 196);
const WHITE: Color32 = Color32::from_rgb(235, 235, 230);
const YELLOW: Color32 = Color32::from_rgb(236, 190, 50);
const MEDIAN: Color32 = Color32::from_rgb(60, 63, 68);

fn shade(c: Color32, k: f32) -> Color32 {
    let f = |x: u8| ((x as f32 * k).clamp(0.0, 255.0)) as u8;
    Color32::from_rgba_unmultiplied(f(c.r()), f(c.g()), f(c.b()), c.a())
}

fn lerp_color(a: Color32, b: Color32, t: f32) -> Color32 {
    let t = t.clamp(0.0, 1.0);
    let l = |x: u8, y: u8| (x as f32 + (y as f32 - x as f32) * t) as u8;
    Color32::from_rgb(l(a.r(), b.r()), l(a.g(), b.g()), l(a.b(), b.b()))
}

pub fn delay_color(d: f64) -> Color32 {
    let t = (d / 90.0) as f32;
    if t < 0.5 {
        lerp_color(Color32::from_rgb(70, 200, 110), Color32::from_rgb(240, 210, 60), t * 2.0)
    } else {
        lerp_color(Color32::from_rgb(240, 210, 60), Color32::from_rgb(230, 60, 60), (t - 0.5) * 2.0)
    }
}

const CAR_COLORS: [Color32; 10] = [
    Color32::from_rgb(200, 50, 50),
    Color32::from_rgb(40, 90, 180),
    Color32::from_rgb(230, 230, 232),
    Color32::from_rgb(30, 30, 34),
    Color32::from_rgb(140, 145, 152),
    Color32::from_rgb(90, 150, 90),
    Color32::from_rgb(220, 170, 50),
    Color32::from_rgb(110, 60, 140),
    Color32::from_rgb(60, 160, 190),
    Color32::from_rgb(170, 90, 50),
];

// ------------------------------------------------------------------ static scene

/// World-space triangle soup for the static scene.
pub struct Scene {
    pub kind: LayoutKind,
    pos: Vec<V2>,
    col: Vec<Color32>,
    idx: Vec<u32>,
    trees: Vec<(V2, f32, Color32)>,
    buildings: Vec<([V2; 4], Color32, f32)>,
}

impl Scene {
    fn tri(&mut self, a: V2, b: V2, c: V2, col: Color32) {
        let i = self.pos.len() as u32;
        self.pos.extend([a, b, c]);
        self.col.extend([col, col, col]);
        self.idx.extend([i, i + 1, i + 2]);
    }
    fn poly(&mut self, pts: &[V2], col: Color32) {
        for k in 1..pts.len() - 1 {
            self.tri(pts[0], pts[k], pts[k + 1], col);
        }
    }
    fn quad_line(&mut self, a: V2, b: V2, w: f64, col: Color32) {
        let n = (b - a).norm().right() * (w / 2.0);
        self.poly(&[a + n, b + n, b - n, a - n], col);
    }
    fn polyline(&mut self, pts: &[V2], w: f64, col: Color32) {
        for s in pts.windows(2) {
            self.quad_line(s[0], s[1], w, col);
        }
    }
    fn dashed(&mut self, a: V2, b: V2, w: f64, dash: f64, gap: f64, col: Color32) {
        let len = a.dist(b);
        let dir = (b - a).norm();
        let mut s = 0.0;
        while s < len {
            let e = (s + dash).min(len);
            self.quad_line(a + dir * s, a + dir * e, w, col);
            s += dash + gap;
        }
    }
    fn circle(&mut self, c: V2, r: f64, col: Color32) {
        let n = ((r * 3.0) as usize).clamp(12, 64);
        let pts: Vec<V2> = (0..n).map(|i| c + V2::from_angle(i as f64 / n as f64 * std::f64::consts::TAU) * r).collect();
        self.poly(&pts, col);
    }
    fn ring(&mut self, c: V2, r0: f64, r1: f64, col: Color32) {
        let n = ((r1 * 3.0) as usize).clamp(24, 96);
        for i in 0..n {
            let a0 = i as f64 / n as f64 * std::f64::consts::TAU;
            let a1 = (i + 1) as f64 / n as f64 * std::f64::consts::TAU;
            let (p0, p1) = (c + V2::from_angle(a0) * r0, c + V2::from_angle(a1) * r0);
            let (q0, q1) = (c + V2::from_angle(a0) * r1, c + V2::from_angle(a1) * r1);
            self.poly(&[p0, q0, q1, p1], col);
        }
    }
    fn arrow(&mut self, base: V2, fwd: V2, turn: Turn, col: Color32) {
        let side = match turn {
            Turn::L => fwd.left(),
            Turn::R => fwd.right(),
            Turn::T => fwd,
        };
        let stem_end = base + fwd * 2.2;
        self.quad_line(base, stem_end, 0.28, col);
        let (tip, from) = if turn == Turn::T { (base + fwd * 3.6, stem_end) } else { (stem_end + side * 1.4, stem_end) };
        if turn != Turn::T {
            self.quad_line(from, tip - side * 0.3, 0.28, col);
        }
        let dir = (tip - from).norm();
        self.poly(&[tip, tip - dir * 0.9 + dir.right() * 0.55, tip - dir * 0.9 - dir.right() * 0.55], col);
    }

    pub fn build(net: &Network) -> Scene {
        let mut sc = Scene { kind: net.kind, pos: vec![], col: vec![], idx: vec![], trees: vec![], buildings: vec![] };
        let mut rng = crate::sim::rng::Rng::new(net.kind as u64 * 31 + 5);
        let ext = 420.0;
        // Terrain with a little texture.
        sc.poly(&[v2(-ext, -ext), v2(ext, -ext), v2(ext, ext), v2(-ext, ext)], GRASS);
        for _ in 0..500 {
            let c = v2(rng.range(-ext, ext), rng.range(-ext, ext));
            sc.circle(c, rng.range(2.0, 9.0), if rng.chance(0.5) { GRASS2 } else { shade(GRASS, 1.06) });
        }
        if net.kind == LayoutKind::Roundabout {
            sc.build_roundabout(net, &mut rng);
        } else {
            sc.build_signal_roads(net);
        }
        sc.scatter_buildings(net, &mut rng);
        sc
    }

    fn build_signal_roads(&mut self, net: &Network) {
        let far = net.in_len + 30.0;
        let sw = 3.5;
        let rf = 7.0;
        // Sidewalks first (under the road), then asphalt.
        for leg in &net.legs {
            let (d, ins) = (leg.d, leg.in_side());
            let p = |r: f64, lat: f64| d * r + ins * lat;
            self.poly(&[p(0.0, -leg.out_width - sw), p(far, -leg.out_width - sw), p(far, leg.in_width + sw), p(0.0, leg.in_width + sw)], SIDEWALK);
        }
        for leg in &net.legs {
            let (d, ins) = (leg.d, leg.in_side());
            let p = |r: f64, lat: f64| d * r + ins * lat;
            self.poly(&[p(0.0, -leg.out_width), p(far, -leg.out_width), p(far, leg.in_width), p(0.0, leg.in_width)], ASPHALT);
            self.quad_line(p(0.0, -leg.out_width - 0.15), p(far, -leg.out_width - 0.15), 0.3, CURB);
            self.quad_line(p(0.0, leg.in_width + 0.15), p(far, leg.in_width + 0.15), 0.3, CURB);
        }
        // Curb returns between angularly adjacent legs, filled toward the box.
        let n = net.legs.len();
        let mut order: Vec<usize> = (0..n).collect();
        order.sort_by(|&a, &b| net.legs[a].d.angle().partial_cmp(&net.legs[b].d.angle()).unwrap());
        for k in 0..n {
            let x = &net.legs[order[k]];
            let y = &net.legs[order[(k + 1) % n]];
            if x.d.dot(y.d).abs() > 0.5 {
                continue; // opposite legs (T-junction side): straight edge
            }
            // X's out edge and Y's in edge.
            let ex = x.out_side() * x.out_width;
            let ey = y.in_side() * y.in_width;
            let c = ex + ey; // perpendicular legs: edges meet here
            let t1 = c + x.d * rf;
            let t2 = c + y.d * rf;
            let o = c + x.d * rf + y.d * rf;
            let a1 = (t1 - o).angle();
            let mut a2 = (t2 - o).angle();
            while a2 - a1 > std::f64::consts::PI {
                a2 -= std::f64::consts::TAU;
            }
            while a1 - a2 > std::f64::consts::PI {
                a2 += std::f64::consts::TAU;
            }
            let arc: Vec<V2> = (0..=16).map(|i| o + V2::from_angle(a1 + (a2 - a1) * i as f64 / 16.0) * rf).collect();
            // Sidewalk ring around the corner block, then asphalt fan.
            for w in arc.windows(2) {
                let (i0, i1) = (o + (w[0] - o) * ((rf - sw) / rf), o + (w[1] - o) * ((rf - sw) / rf));
                self.poly(&[w[0], w[1], i1, i0], SIDEWALK);
            }
            for w in arc.windows(2) {
                self.tri(c, w[0], w[1], ASPHALT);
            }
            self.polyline(&arc, 0.3, CURB);
        }
        // Centre fill (covers seams between leg rectangles).
        for leg in &net.legs {
            let (d, ins) = (leg.d, leg.in_side());
            let p = |r: f64, lat: f64| d * r + ins * lat;
            self.poly(&[p(0.0, -leg.out_width), p(leg.stop_dist, -leg.out_width), p(leg.stop_dist, leg.in_width), p(0.0, leg.in_width)], ASPHALT);
        }
        // Markings.
        for (li, leg) in net.legs.iter().enumerate() {
            let (d, ins) = (leg.d, leg.in_side());
            let p = |r: f64, lat: f64| d * r + ins * lat;
            let r0 = leg.stop_dist;
            let w = leg.lane_w;
            // Centre double-yellow and median.
            self.quad_line(p(r0, 0.12), p(far, 0.12), 0.12, YELLOW);
            self.quad_line(p(r0, -0.12), p(far, -0.12), 0.12, YELLOW);
            if leg.med_out > 0.1 {
                let m = leg.med_out;
                self.poly(&[p(r0, -0.2), p(far, -0.2), p(far, -m + 0.2), p(r0, -m + 0.2)], MEDIAN);
                self.quad_line(p(r0, -m + 0.12), p(far, -m + 0.12), 0.12, YELLOW);
                let mut r = r0 + 2.0;
                while r < r0 + 60.0 {
                    self.quad_line(p(r, -0.3), p(r + 2.2, -m + 0.3), 0.18, shade(YELLOW, 0.85));
                    r += 4.0;
                }
            }
            // Turn bay: upstream of it the lane is painted median with a taper.
            let bay = leg.in_lanes.first().map(|&l| net.links[l].bay_start).unwrap_or(0.0);
            let rb = if bay > 0.0 { net.in_len - bay } else { far };
            if bay > 0.0 {
                let taper = 18.0;
                self.poly(&[p(rb + taper, 0.2), p(far, 0.2), p(far, w - 0.15), p(rb + taper, w - 0.15)], MEDIAN);
                self.tri(p(rb, 0.2), p(rb + taper, 0.2), p(rb + taper, w - 0.15), MEDIAN);
                self.quad_line(p(rb, 0.2), p(rb + taper, w - 0.15), 0.14, YELLOW);
                self.quad_line(p(rb + taper, w - 0.15), p(far, w - 0.15), 0.12, YELLOW);
                let mut r = rb + taper + 2.0;
                while r < far - 3.0 {
                    self.quad_line(p(r, 0.4), p(r + 2.2, w - 0.4), 0.18, shade(YELLOW, 0.85));
                    r += 4.0;
                }
            }
            // Lane lines (solid near the stop line: no lane changes there).
            for k in 1..leg.lane_moves.len() {
                let lat = k as f64 * w;
                let end = if k == 1 && bay > 0.0 { rb } else { far };
                self.quad_line(p(r0, lat), p(r0 + 25.0, lat), 0.14, WHITE);
                self.dashed(p(r0 + 25.0, lat), p(end, lat), 0.14, 3.0, 9.0, WHITE);
            }
            for k in 1..leg.n_out {
                let lat = -(leg.med_out + k as f64 * w);
                self.dashed(p(r0, lat), p(far, lat), 0.14, 3.0, 9.0, WHITE);
            }
            // Stop line.
            self.quad_line(p(r0 + 0.25, 0.2), p(r0 + 0.25, leg.in_width - 0.1), 0.5, WHITE);
            // Lane arrows.
            for (k, moves) in leg.lane_moves.iter().enumerate() {
                for &r in &[r0 + 7.0, r0 + 34.0] {
                    for &t in moves {
                        self.arrow(p(r + 3.6, (k as f64 + 0.5) * w), -d, t, shade(WHITE, 0.92));
                    }
                }
            }
            // Crosswalk zebra.
            if let Some(cw) = leg.crosswalk {
                let c = &net.crosswalks[cw];
                let dir = (c.b - c.a).norm();
                let mut u = 0.4;
                while u < c.length - 0.3 {
                    let m = c.a + dir * u;
                    self.quad_line(m - d * 1.5, m + d * 1.5, 0.55, WHITE);
                    u += 1.1;
                }
            }
            let _ = li;
        }
    }

    fn build_roundabout(&mut self, net: &Network, _rng: &mut crate::sim::rng::Rng) {
        let far = net.in_len + 30.0;
        let sw = 3.5;
        let c0 = v2(0.0, 0.0);
        let (rr, rw) = (net.ring_r, net.ring_w);
        for leg in &net.legs {
            let (d, ins) = (leg.d, leg.in_side());
            let p = |r: f64, lat: f64| d * r + ins * lat;
            self.poly(&[p(rr, -leg.out_width - sw), p(far, -leg.out_width - sw), p(far, leg.in_width + sw), p(rr, leg.in_width + sw)], SIDEWALK);
        }
        self.circle(c0, rr + rw / 2.0 + 2.0 + sw, SIDEWALK);
        for leg in &net.legs {
            let (d, ins) = (leg.d, leg.in_side());
            let p = |r: f64, lat: f64| d * r + ins * lat;
            self.poly(&[p(rr, -leg.out_width), p(far, -leg.out_width), p(far, leg.in_width), p(rr, leg.in_width)], ASPHALT);
            self.quad_line(p(rr + 6.0, -leg.out_width - 0.15), p(far, -leg.out_width - 0.15), 0.3, CURB);
            self.quad_line(p(rr + 6.0, leg.in_width + 0.15), p(far, leg.in_width + 0.15), 0.3, CURB);
        }
        self.circle(c0, rr + rw / 2.0 + 2.0, ASPHALT);
        // Central island with truck apron.
        let inner = rr - rw / 2.0;
        self.circle(c0, inner, Color32::from_rgb(150, 120, 100));
        self.ring(c0, inner - 0.4, inner, CURB);
        self.circle(c0, inner - 1.8, Color32::from_rgb(88, 140, 70));
        self.circle(c0, 3.5, Color32::from_rgb(110, 160, 80));
        // Splitter islands, lane markings, yield teeth.
        for leg in &net.legs {
            let (d, ins) = (leg.d, leg.in_side());
            let p = |r: f64, lat: f64| d * r + ins * lat;
            let r0 = rr + rw / 2.0 + 1.2;
            self.poly(&[p(r0, 0.0), p(r0 + 14.0, -1.7), p(r0 + 26.0, -0.6), p(r0 + 26.0, 0.6), p(r0 + 14.0, 1.7)], CURB);
            self.poly(&[p(r0 + 1.0, 0.0), p(r0 + 14.0, -1.3), p(r0 + 25.0, -0.35), p(r0 + 25.0, 0.35), p(r0 + 14.0, 1.3)], Color32::from_rgb(98, 146, 76));
            self.quad_line(p(r0 + 26.0, 0.12), p(far, 0.12), 0.12, YELLOW);
            self.quad_line(p(r0 + 26.0, -0.12), p(far, -0.12), 0.12, YELLOW);
            let ry = leg.stop_dist;
            let lat_c = leg.in_width - leg.lane_w / 2.0;
            let mut k = -1.4;
            while k <= 1.41 {
                let base = p(ry + 0.2, lat_c + k);
                self.poly(&[base + ins * 0.35, base - ins * 0.35, base + d * 0.9], WHITE);
                k += 0.95;
            }
            self.arrow(p(ry + 10.0, lat_c), -d, Turn::R, shade(WHITE, 0.9));
        }
        // Ring lane edge dashes.
        let n = 96;
        for i in (0..n).step_by(2) {
            let a0 = i as f64 / n as f64 * std::f64::consts::TAU;
            let a1 = (i as f64 + 0.8) / n as f64 * std::f64::consts::TAU;
            self.quad_line(V2::from_angle(a0) * (rr + rw / 2.0), V2::from_angle(a1) * (rr + rw / 2.0), 0.15, WHITE);
        }
    }

    fn scatter_buildings(&mut self, net: &Network, rng: &mut crate::sim::rng::Rng) {
        // Keep clear of roads (and their sidewalks).
        let road_clear = |p: V2, margin: f64| -> bool {
            if net.kind == LayoutKind::Roundabout && p.len() < net.ring_r + net.ring_w + 12.0 + margin {
                return false;
            }
            for leg in &net.legs {
                let r = p.dot(leg.d);
                let lat = p.dot(leg.in_side());
                if r > -leg.in_width.max(leg.out_width) - 6.0 && lat < leg.in_width + 4.5 + margin && lat > -leg.out_width - 4.5 - margin {
                    return false;
                }
            }
            true
        };
        let roofs = [
            Color32::from_rgb(120, 92, 80),
            Color32::from_rgb(96, 100, 112),
            Color32::from_rgb(140, 130, 118),
            Color32::from_rgb(84, 88, 96),
            Color32::from_rgb(150, 110, 86),
        ];
        let mut placed: Vec<(V2, f64)> = vec![];
        for _ in 0..900 {
            let c = v2(rng.range(-260.0, 260.0), rng.range(-260.0, 260.0));
            let (w, h) = (rng.range(10.0, 28.0), rng.range(10.0, 24.0));
            let rad = (w * w + h * h).sqrt() / 2.0;
            if !road_clear(c, rad + 3.0) || placed.iter().any(|&(q, r)| q.dist(c) < r + rad + 4.0) {
                continue;
            }
            placed.push((c, rad));
            let (hx, hy) = (v2(w / 2.0, 0.0), v2(0.0, h / 2.0));
            let col = roofs[rng.below(roofs.len())];
            self.buildings.push(([c - hx - hy, c + hx - hy, c + hx + hy, c - hx + hy], col, rng.range(0.6, 1.4) as f32));
            if placed.len() > 70 {
                break;
            }
        }
        // Street trees along sidewalks and random trees in yards.
        for leg in &net.legs {
            let mut r = leg.stop_dist + 14.0;
            while r < net.in_len {
                for side in [-1.0, 1.0] {
                    let lat = if side > 0.0 { leg.in_width + 6.5 } else { -leg.out_width - 6.5 };
                    let p = leg.d * r + leg.in_side() * lat;
                    if !placed.iter().any(|&(q, rad)| q.dist(p) < rad + 3.0) {
                        let g = rng.range(0.85, 1.15) as f32;
                        self.trees.push((p, rng.range(2.6, 3.8) as f32, shade(Color32::from_rgb(52, 110, 50), g)));
                    }
                }
                r += rng.range(11.0, 17.0);
            }
        }
        for _ in 0..240 {
            let p = v2(rng.range(-280.0, 280.0), rng.range(-280.0, 280.0));
            if road_clear(p, 2.0) && !placed.iter().any(|&(q, rad)| q.dist(p) < rad + 3.0) {
                let g = rng.range(0.8, 1.2) as f32;
                self.trees.push((p, rng.range(2.0, 4.5) as f32, shade(Color32::from_rgb(48, 100, 46), g)));
            }
        }
    }

    pub fn paint(&self, painter: &egui::Painter, cam: &Camera, night: bool) {
        let tint = if night { 0.42 } else { 1.0 };
        let mut mesh = Mesh::default();
        mesh.reserve_vertices(self.pos.len());
        mesh.reserve_triangles(self.idx.len() / 3);
        for (p, c) in self.pos.iter().zip(&self.col) {
            mesh.colored_vertex(cam.to_screen(*p), if night { shade(*c, tint) } else { *c });
        }
        mesh.indices.extend_from_slice(&self.idx);
        painter.add(Shape::mesh(mesh));
        // Buildings with offset shadows and roof ridge shading.
        for (pts, col, hgt) in &self.buildings {
            let off = v2(1.6, 2.2) * *hgt as f64;
            let sh: Vec<Pos2> = pts.iter().map(|p| cam.to_screen(*p + off)).collect();
            painter.add(Shape::convex_polygon(sh, Color32::from_black_alpha(70), Stroke::NONE));
            let s: Vec<Pos2> = pts.iter().map(|p| cam.to_screen(*p)).collect();
            painter.add(Shape::convex_polygon(s.clone(), shade(*col, tint), Stroke::new(1.0, shade(*col, 0.7 * tint))));
            let c = pts[0].lerp(pts[2], 0.5);
            let inset: Vec<Pos2> = pts.iter().map(|p| cam.to_screen(c.lerp(*p, 0.7))).collect();
            painter.add(Shape::convex_polygon(inset, shade(*col, 1.12 * tint), Stroke::NONE));
        }
        for (p, r, col) in &self.trees {
            let c = cam.to_screen(*p);
            let rs = r * cam.scale;
            painter.circle_filled(c + egui::vec2(rs * 0.35, rs * 0.45), rs, Color32::from_black_alpha(60));
            painter.circle_filled(c, rs, shade(*col, tint));
            painter.circle_filled(c - egui::vec2(rs * 0.3, rs * 0.3), rs * 0.55, shade(*col, 1.25 * tint));
        }
    }
}

// ------------------------------------------------------------------ dynamic layer

fn sig_color(s: Sig) -> Color32 {
    match s {
        Sig::G | Sig::Rt => Color32::from_rgb(60, 230, 110),
        Sig::P => Color32::from_rgb(140, 230, 90),
        Sig::Y => Color32::from_rgb(250, 200, 40),
        Sig::R => Color32::from_rgb(235, 50, 45),
    }
}

fn poly(painter: &egui::Painter, cam: &Camera, pts: &[V2], fill: Color32) {
    painter.add(Shape::convex_polygon(pts.iter().map(|p| cam.to_screen(*p)).collect(), fill, Stroke::NONE));
}

fn rect_pts(c: V2, ax: V2, hl: f64, hw: f64) -> [V2; 4] {
    let n = ax.right();
    [c + ax * hl + n * hw, c + ax * hl - n * hw, c - ax * hl - n * hw, c - ax * hl + n * hw]
}

pub struct DrawCtx<'a> {
    pub sim: &'a Sim,
    pub cam: Camera,
    pub ov: &'a Overlays,
    pub selected: Option<u64>,
    pub time: f64,
    /// Fraction of the next simulation step elapsed in real time (render interpolation).
    pub alpha: f64,
}

impl DrawCtx<'_> {
    pub fn body(&self, v: &Vehicle) -> Vec<(V2, V2, f64, f64)> {
        let off = self.sim.lc_offset(v, self.alpha);
        let mut b = self.sim.body_lagged(v, self.sim.lag(v, self.alpha));
        if off.len() > 1e-6 {
            // Changing lanes: slide across, nosing toward the new lane.
            let k = off.len().min(3.5) / 3.5;
            for seg in b.iter_mut() {
                seg.0 = seg.0 + off;
                seg.1 = (seg.1 - off.norm() * (0.12 * k)).norm();
            }
        }
        b
    }

    pub fn front(&self, v: &Vehicle) -> V2 {
        self.sim.point_behind(v, self.sim.lag(v, self.alpha)) + self.sim.lc_offset(v, self.alpha)
    }

    pub fn center(&self, v: &Vehicle) -> V2 {
        self.sim.point_behind(v, self.sim.lag(v, self.alpha) + v.len() * 0.5) + self.sim.lc_offset(v, self.alpha)
    }
}

pub fn paint_dynamic(painter: &egui::Painter, dc: &DrawCtx) {
    let sim = dc.sim;
    let cam = dc.cam;
    if dc.ov.sensors {
        paint_sensors(painter, dc);
    }
    if dc.ov.paths {
        for l in &sim.net.links {
            if l.kind.in_box() {
                let pts: Vec<Pos2> = l.path.pts.iter().step_by(2).map(|p| cam.to_screen(*p)).collect();
                painter.add(Shape::line(pts, Stroke::new(1.0, Color32::from_rgba_unmultiplied(120, 200, 255, 60))));
            }
        }
    }
    paint_signals(painter, dc);
    // Vehicles: shadows first, then bodies.
    for v in &sim.vehicles {
        for (c, ax, hl, hw) in dc.body(v) {
            let off = v2(0.35, 0.5);
            poly(painter, &cam, &rect_pts(c + off, ax, hl, hw), Color32::from_black_alpha(55));
        }
    }
    for v in &sim.vehicles {
        paint_vehicle(painter, dc, v);
    }
    for p in &sim.peds {
        paint_ped(painter, dc, p);
    }
    if dc.ov.conflicts {
        for c in &sim.net.conflicts {
            let p = sim.net.links[c.a].path.point(c.sa);
            let held = sim.vehicles.iter().any(|v| v.commits.contains(&c.id));
            let col = if held { Color32::from_rgb(255, 90, 60) } else if c.merge { Color32::from_rgb(120, 180, 255) } else { Color32::from_rgb(255, 230, 120) };
            painter.circle_stroke(cam.to_screen(p), (1.0 * cam.scale).max(3.0), Stroke::new(1.5, col));
        }
    }
    if let Some(id) = dc.selected {
        if let Some(v) = sim.vehicles.iter().find(|v| v.id == id) {
            // Remaining route.
            let mut pts = vec![];
            for k in v.ri..v.route.len() {
                let l = &sim.net.links[v.route[k]];
                let s0 = if k == v.ri { v.s } else { 0.0 };
                let mut s = s0;
                while s < l.length() {
                    pts.push(cam.to_screen(l.path.point(s)));
                    s += 2.0;
                }
            }
            if pts.len() > 1 {
                painter.add(Shape::dashed_line(&pts, Stroke::new(2.0, Color32::from_rgb(255, 255, 255)), 6.0, 4.0));
            }
            for (c, ax, hl, hw) in dc.body(v) {
                let pts: Vec<Pos2> = rect_pts(c, ax, hl + 0.6, hw + 0.6).iter().map(|p| cam.to_screen(*p)).collect();
                painter.add(Shape::closed_line(pts, Stroke::new(2.0, Color32::from_rgb(255, 255, 120))));
            }
            if let Some(yid) = v.yield_to {
                if let Some(w) = sim.vehicles.iter().find(|w| w.id == yid) {
                    painter.arrow(cam.to_screen(dc.front(v)), cam.to_screen(dc.center(w)) - cam.to_screen(dc.front(v)), Stroke::new(2.0, Color32::from_rgb(255, 160, 60)));
                }
            }
        }
    }
}

fn paint_sensors(painter: &egui::Painter, dc: &DrawCtx) {
    let sim = dc.sim;
    let cam = dc.cam;
    let cfg = &sim.sensors;
    for leg in &sim.net.legs {
        for &lane in &leg.in_lanes {
            let l = &sim.net.links[lane];
            let len = l.length();
            let ax = l.path.tangent(len);
            let hw = leg.lane_w * 0.38;
            if cfg.loops {
                let lo = &sim.view.lanes[lane];
                // Stop-bar presence loop.
                let c = l.path.point(len - 4.5);
                let on = lo.stopbar;
                let col = if on { Color32::from_rgba_unmultiplied(255, 190, 40, 110) } else { Color32::from_rgba_unmultiplied(255, 190, 40, 18) };
                let pts = rect_pts(c, ax, 4.3, hw);
                poly(painter, &cam, &pts, col);
                painter.add(Shape::closed_line(pts.iter().map(|p| cam.to_screen(*p)).collect(), Stroke::new(1.0, Color32::from_rgba_unmultiplied(255, 200, 60, 150))));
                if lo.fault {
                    // Diagnostics flagged this station: red cross (lane on recall).
                    let red = Stroke::new((0.25 * cam.scale).max(1.5), Color32::from_rgb(240, 60, 50));
                    painter.line_segment([cam.to_screen(pts[0]), cam.to_screen(pts[2])], red);
                    painter.line_segment([cam.to_screen(pts[1]), cam.to_screen(pts[3])], red);
                }
                // Advance point detector (flashes on actuation).
                if cfg.adv_dist < len {
                    let c = l.path.point(len - cfg.adv_dist);
                    let recent = sim.t - lo.last_pulse < 0.6 && !lo.stopbar;
                    let col = if recent { Color32::from_rgba_unmultiplied(255, 220, 80, 170) } else { Color32::from_rgba_unmultiplied(255, 200, 60, 30) };
                    let pts = rect_pts(c, ax, 1.0, hw);
                    poly(painter, &cam, &pts, col);
                    painter.add(Shape::closed_line(pts.iter().map(|p| cam.to_screen(*p)).collect(), Stroke::new(1.0, Color32::from_rgba_unmultiplied(255, 200, 60, 140))));
                }
            }
        }
        let ins = leg.in_side();
        let pole = leg.d * (leg.stop_dist + 1.5) + ins * (leg.in_width + 2.2);
        if cfg.camera {
            // Field-of-view wedge covering the approach (shrinks in poor visibility).
            let range = if cfg.realistic { cfg.cam_range * cfg.conditions.camera().1 } else { cfg.cam_range };
            let far = leg.d * (leg.stop_dist + range);
            let a = far + ins * (leg.in_width + 1.5);
            let b = far - ins * (leg.out_width * 0.2);
            painter.add(Shape::convex_polygon(
                vec![cam.to_screen(pole), cam.to_screen(a), cam.to_screen(b)],
                Color32::from_rgba_unmultiplied(80, 200, 255, 16),
                Stroke::new(1.0, Color32::from_rgba_unmultiplied(80, 200, 255, 60)),
            ));
            let pc = cam.to_screen(pole);
            let s = (0.9 * cam.scale).max(4.0);
            painter.circle_filled(pc, s, Color32::from_rgb(25, 30, 36));
            painter.circle_stroke(pc, s, Stroke::new(1.5, Color32::from_rgb(80, 200, 255)));
            painter.circle_filled(pc, s * 0.4, Color32::from_rgb(140, 230, 255));
        }
        if cfg.radar {
            let rp = leg.d * (leg.stop_dist + 3.0) + ins * (leg.in_width + 3.6);
            let c = cam.to_screen(rp);
            let s = (0.7 * cam.scale).max(3.0);
            painter.add(Shape::convex_polygon(
                vec![c + egui::vec2(0.0, -s), c + egui::vec2(s, 0.0), c + egui::vec2(0.0, s), c + egui::vec2(-s, 0.0)],
                Color32::from_rgb(190, 110, 255),
                Stroke::NONE,
            ));
            // Range marks.
            let r = leg.d * (leg.stop_dist + cfg.radar_range);
            let a = cam.to_screen(r + ins * (leg.in_width + 1.0));
            let b = cam.to_screen(r);
            painter.add(Shape::dashed_line(&[a, b], Stroke::new(1.5, Color32::from_rgba_unmultiplied(190, 110, 255, 140)), 5.0, 4.0));
            // Sweep beam.
            let phase = (dc.time * 0.8).fract() as f32;
            let sweep = leg.d * (leg.stop_dist + cfg.radar_range * phase as f64) + ins * (leg.in_width * 0.5);
            painter.circle_filled(cam.to_screen(sweep), (0.6 * cam.scale).max(2.0), Color32::from_rgba_unmultiplied(190, 110, 255, 90));
        }
    }
}

fn paint_signals(painter: &egui::Painter, dc: &DrawCtx) {
    let sim = dc.sim;
    let cam = dc.cam;
    let net = &sim.net;
    let blink = (dc.time * 2.0).fract() < 0.55;
    match sim.mode {
        ControlMode::Signal if sim.signalized() => {
            for leg in &net.legs {
                for (k, &lane) in leg.in_lanes.iter().enumerate() {
                    let l = &net.links[lane];
                    // Lane head: show the most restrictive relevant indication for the lane.
                    let moves = &l.lane_moves;
                    let sigs: Vec<(Turn, Sig)> = moves.iter().map(|&m| (net.movements[m].turn, sim.signal.status(net, m))).collect();
                    let main = sigs.iter().find(|s| s.0 == Turn::T).or_else(|| sigs.first()).map(|s| s.1).unwrap_or(Sig::R);
                    // A turn-only lane has an arrow head: its permissive phase is the
                    // flashing yellow arrow alone, never a green ball.
                    let turn_only = !sigs.iter().any(|s| s.0 == Turn::T);
                    let base = l.path.point(l.length() + 0.8);
                    let ax = l.path.tangent(l.length());
                    let n = ax.right();
                    let housing = rect_pts(base + ax * 0.0, ax, 0.55, 1.35);
                    let _ = n;
                    poly(painter, &cam, &housing, Color32::from_rgb(22, 24, 26));
                    let lamps = [(Sig::R, -0.9), (Sig::Y, 0.0), (Sig::G, 0.9)];
                    for (s, off) in lamps {
                        let p = base + ax.left() * off;
                        let lit = match (s, main) {
                            (Sig::G, Sig::P) => !turn_only,
                            (Sig::G, Sig::G | Sig::Rt) => true,
                            (a, b) => a == b,
                        };
                        let col = if lit { sig_color(main) } else { shade(sig_color(s), 0.25) };
                        let pc = cam.to_screen(p);
                        let r = (0.36 * cam.scale).max(1.5);
                        if lit {
                            painter.circle_filled(pc, r * 2.4, Color32::from_rgba_unmultiplied(col.r(), col.g(), col.b(), 40));
                        }
                        painter.circle_filled(pc, r, col);
                    }
                    // Left-turn arrow indication when the lane has a turn with its own state.
                    let tturn = sigs.iter().find(|s| s.0 != Turn::T && (sigs.len() == 1 || s.1 != main));
                    if let Some(&(t, s)) = tturn {
                        let p = base - ax * 1.4;
                        let col = if s == Sig::P { if blink { Color32::from_rgb(250, 200, 40) } else { shade(Color32::from_rgb(250, 200, 40), 0.3) } } else { sig_color(s) };
                        let side = if t == Turn::L { ax.left() } else if t == Turn::R { ax.right() } else { ax };
                        let a = cam.to_screen(p - side * 0.6);
                        let b = cam.to_screen(p + side * 0.6);
                        painter.arrow(a, b - a, Stroke::new((0.25 * cam.scale).max(1.2), col));
                    }
                    // Stop-line glow.
                    let a = l.path.point(l.length()) + ax.right() * (leg.lane_w * 0.45);
                    let b = l.path.point(l.length()) - ax.right() * (leg.lane_w * 0.45);
                    let col = sig_color(main);
                    painter.line_segment([cam.to_screen(a), cam.to_screen(b)], Stroke::new((0.25 * cam.scale).max(1.0), Color32::from_rgba_unmultiplied(col.r(), col.g(), col.b(), 140)));
                    let _ = k;
                }
            }
            // Pedestrian heads at crosswalk ends.
            for (i, cw) in net.crosswalks.iter().enumerate() {
                let st = sim.signal.cw[i].state;
                let col = match st {
                    PedSig::Walk => Color32::from_rgb(235, 245, 255),
                    PedSig::Fdw => if blink { Color32::from_rgb(255, 140, 40) } else { Color32::from_rgb(70, 40, 20) },
                    PedSig::Dw => Color32::from_rgb(230, 110, 30),
                };
                let dir = (cw.b - cw.a).norm();
                for (p, s) in [(cw.a, -1.0), (cw.b, 1.0)] {
                    let q = cam.to_screen(p + dir * (s * 1.6));
                    let r = (0.55 * cam.scale).max(2.5);
                    painter.rect_filled(Rect::from_center_size(q, egui::vec2(r * 2.2, r * 2.2)), 2.0, Color32::from_rgb(20, 20, 22));
                    painter.circle_filled(q, r * 0.8, col);
                }
                if sim.signal.cw[i].called && st == PedSig::Dw {
                    let q = cam.to_screen(cw.a.lerp(cw.b, 0.5));
                    painter.circle_stroke(q, (1.2 * cam.scale).max(4.0), Stroke::new(1.0, Color32::from_rgba_unmultiplied(255, 255, 255, 80)));
                }
            }
        }
        ControlMode::AllWayStop | ControlMode::TwoWayStop | ControlMode::Signal => {
            for leg in &net.legs {
                let stop = sim.mode == ControlMode::AllWayStop || leg.minor;
                if !stop {
                    continue;
                }
                let p = leg.d * (leg.stop_dist + 1.5) + leg.in_side() * (leg.in_width + 1.6);
                let c = cam.to_screen(p);
                let r = (0.9 * cam.scale).max(4.0);
                let pts: Vec<Pos2> = (0..8)
                    .map(|i| {
                        let a = (i as f32 + 0.5) / 8.0 * std::f32::consts::TAU;
                        c + egui::vec2(a.cos(), a.sin()) * r
                    })
                    .collect();
                painter.add(Shape::convex_polygon(pts, Color32::from_rgb(200, 30, 30), Stroke::new(1.0, Color32::WHITE)));
                if r > 6.0 {
                    painter.text(c, Align2::CENTER_CENTER, "STOP", FontId::proportional(r * 0.55), Color32::WHITE);
                }
            }
        }
        ControlMode::Roundabout => {}
    }
}

fn base_color(v: &Vehicle, by: ColorBy) -> Color32 {
    match by {
        ColorBy::Type => match v.kind {
            VKind::Bus => Color32::from_rgb(235, 175, 30),
            VKind::Truck => Color32::from_rgb(205, 208, 214),
            VKind::Emergency => Color32::from_rgb(245, 245, 245),
            VKind::Motorcycle => CAR_COLORS[(v.color % 4) as usize + 3],
            _ => CAR_COLORS[(v.color % CAR_COLORS.len() as u32) as usize],
        },
        ColorBy::Delay => delay_color(v.delay),
        ColorBy::Movement => match v.turn {
            Turn::L => Color32::from_rgb(90, 150, 255),
            Turn::T => Color32::from_rgb(200, 200, 205),
            Turn::R => Color32::from_rgb(255, 150, 70),
        },
        ColorBy::Speed => lerp_color(Color32::from_rgb(220, 60, 60), Color32::from_rgb(70, 210, 120), (v.v / v.v0.max(1.0)) as f32),
    }
}

fn paint_vehicle(painter: &egui::Painter, dc: &DrawCtx, v: &Vehicle) {
    let sim = dc.sim;
    let cam = dc.cam;
    let spec = v.spec();
    let segs = dc.body(v);
    let col = base_color(v, dc.ov.color_by);
    let tint = if dc.ov.night { 0.55 } else { 1.0 };
    let body = shade(col, tint);
    let (fc, fax, fhl, hw) = segs[0];
    let front = fc + fax * fhl;
    let blink = (dc.time * 2.2 + (v.id % 7) as f64 * 0.1).fract() < 0.5;
    let simple = cam.scale < 2.2;
    match v.kind {
        VKind::Truck | VKind::Bus => {
            for (k, &(c, ax, hl, hw)) in segs.iter().enumerate() {
                let pts = rect_pts(c, ax, hl + 0.05, hw);
                let segcol = if v.kind == VKind::Truck && k == 0 && dc.ov.color_by == ColorBy::Type {
                    shade(CAR_COLORS[(v.color % CAR_COLORS.len() as u32) as usize], tint)
                } else {
                    body
                };
                poly(painter, &cam, &pts, segcol);
                if !simple {
                    if v.kind == VKind::Bus {
                        // Window band along both sides and roof units.
                        for side in [1.0, -1.0] {
                            let a = c + ax.right() * (hw * 0.78 * side);
                            poly(painter, &cam, &rect_pts(a, ax, hl * 0.92, hw * 0.16), shade(Color32::from_rgb(40, 60, 80), tint));
                        }
                        if k == 1 || segs.len() == 1 {
                            poly(painter, &cam, &rect_pts(c, ax, 0.9, hw * 0.5), shade(Color32::from_rgb(220, 220, 220), tint));
                        }
                    } else if k > 0 || v.kind == VKind::Truck {
                        let pts: Vec<Pos2> = rect_pts(c, ax, hl, hw).iter().map(|p| cam.to_screen(*p)).collect();
                        painter.add(Shape::closed_line(pts, Stroke::new(0.8, shade(body, 0.7))));
                    }
                }
            }
            if !simple {
                // Windshield.
                let ws = fc + fax * (fhl - 0.55);
                poly(painter, &cam, &rect_pts(ws, fax, 0.28, hw * 0.85), shade(Color32::from_rgb(40, 55, 70), tint));
            }
        }
        _ => {
            let (c, ax, hl, hw) = segs[0];
            let pts = rect_pts(c, ax, hl, hw);
            poly(painter, &cam, &pts, body);
            if v.kind == VKind::Motorcycle {
                painter.circle_filled(cam.to_screen(c - ax * 0.1), (0.35 * cam.scale).max(1.5), Color32::from_rgb(25, 25, 25));
            } else if !simple {
                let glass = shade(Color32::from_rgb(35, 48, 62), tint);
                // Windshield, roof, rear window.
                poly(painter, &cam, &[c + ax * (hl * 0.55) + ax.right() * (hw * 0.82), c + ax * (hl * 0.55) - ax.right() * (hw * 0.82), c + ax * (hl * 0.2) - ax.right() * (hw * 0.72), c + ax * (hl * 0.2) + ax.right() * (hw * 0.72)], glass);
                poly(painter, &cam, &rect_pts(c - ax * (hl * 0.12), ax, hl * 0.3, hw * 0.72), shade(body, 0.88));
                poly(painter, &cam, &rect_pts(c - ax * (hl * 0.58), ax, hl * 0.13, hw * 0.7), glass);
                if v.kind == VKind::Emergency {
                    let on = blink;
                    let l1 = if on { Color32::from_rgb(255, 40, 40) } else { Color32::from_rgb(40, 80, 255) };
                    let l2 = if on { Color32::from_rgb(40, 80, 255) } else { Color32::from_rgb(255, 40, 40) };
                    poly(painter, &cam, &rect_pts(c - ax * 0.2 + ax.right() * (hw * 0.4), ax, 0.25, hw * 0.35), l1);
                    poly(painter, &cam, &rect_pts(c - ax * 0.2 - ax.right() * (hw * 0.4), ax, 0.25, hw * 0.35), l2);
                    painter.circle_filled(cam.to_screen(c), 4.0 * cam.scale, Color32::from_rgba_unmultiplied(l1.r(), l1.g(), l1.b(), 22));
                    poly(painter, &cam, &rect_pts(c + ax * (hl * 0.05), ax, 0.3, hw * 0.9), Color32::from_rgb(210, 40, 40));
                }
            }
        }
    }
    if simple {
        return;
    }
    let (lc, lax, lhl, lhw) = *segs.last().unwrap();
    let rear = lc - lax * lhl;
    // Brake lights.
    let braking = v.a < -0.6 || v.v < 0.1;
    let bl = if braking { Color32::from_rgb(255, 40, 30) } else { Color32::from_rgb(120, 20, 20) };
    for side in [1.0, -1.0] {
        let p = rear + lax * 0.12 + lax.right() * (lhw * 0.72 * side);
        let pc = cam.to_screen(p);
        let r = (0.17 * cam.scale).max(1.0);
        if braking {
            painter.circle_filled(pc, r * 3.0, Color32::from_rgba_unmultiplied(255, 30, 20, 45));
        }
        painter.circle_filled(pc, r, bl);
    }
    // Headlights (beams at night).
    for side in [1.0, -1.0] {
        let p = front - fax * 0.1 + fax.right() * (hw * 0.7 * side);
        if dc.ov.night {
            let tip = p + fax * 14.0 + fax.right() * (2.5 * side);
            let tip2 = p + fax * 14.0 - fax.right() * (1.5 * side);
            painter.add(Shape::convex_polygon(vec![cam.to_screen(p), cam.to_screen(tip), cam.to_screen(tip2)], Color32::from_rgba_unmultiplied(255, 240, 180, 26), Stroke::NONE));
        }
        painter.circle_filled(cam.to_screen(p), (0.15 * cam.scale).max(0.8), Color32::from_rgb(255, 250, 215));
    }
    // Turn indicators near and inside the intersection.
    let link = &sim.net.links[v.link()];
    let near = link.kind.in_box() || (link.kind == LinkKind::Inbound && link.length() - v.s < 70.0);
    if near && v.turn != Turn::T && blink {
        let side = if v.turn == Turn::L { -1.0 } else { 1.0 };
        for p in [front - fax * 0.2 + fax.right() * (hw * side), rear + lax * 0.2 + lax.right() * (lhw * side)] {
            painter.circle_filled(cam.to_screen(p), (0.2 * cam.scale).max(1.2), Color32::from_rgb(255, 170, 30));
        }
    }
    if v.manual {
        painter.circle_stroke(cam.to_screen(fc), (1.2 * cam.scale).max(4.0), Stroke::new(1.0, Color32::from_rgba_unmultiplied(255, 255, 255, 90)));
    }
    if dc.ov.labels && cam.scale > 5.0 {
        let txt = format!("{:.0}s", v.delay);
        painter.text(cam.to_screen(lc), Align2::CENTER_CENTER, txt, FontId::proportional(10.0), Color32::WHITE);
    }
    if v.reason == Reason::Yielding && dc.ov.labels {
        painter.circle_filled(cam.to_screen(front + fax * 1.0), 2.5, Color32::from_rgb(255, 160, 60));
    }
    let _ = spec;
}

pub fn paint_detections(painter: &egui::Painter, dc: &DrawCtx) {
    let sim = dc.sim;
    let cam = dc.cam;
    for &(id, src) in &sim.view.detections {
        let Some(v) = sim.vehicles.iter().find(|v| v.id == id) else { continue };
        let segs = dc.body(v);
        let (c, ax, _, hw) = segs[segs.len() / 2];
        let hl = v.len() / 2.0;
        let col = match src {
            Source::Camera => Color32::from_rgb(90, 220, 255),
            Source::Radar => Color32::from_rgb(190, 110, 255),
            Source::Cv => Color32::from_rgb(120, 255, 160),
            Source::Loop => Color32::from_rgb(255, 200, 60),
        };
        match src {
            Source::Camera => {
                // Bounding-box corner brackets.
                let pts = rect_pts(c, ax, hl + 0.5, hw + 0.5);
                for k in 0..4 {
                    let p = pts[k];
                    let a = pts[(k + 1) % 4];
                    let b = pts[(k + 3) % 4];
                    let ps = cam.to_screen(p);
                    let pa = cam.to_screen(p.lerp(a, 0.25));
                    let pb = cam.to_screen(p.lerp(b, 0.25));
                    painter.add(Shape::line(vec![pa, ps, pb], Stroke::new(1.2, col)));
                }
            }
            Source::Radar => {
                painter.circle_stroke(cam.to_screen(c), (hl as f32 * 0.6 * cam.scale).max(4.0), Stroke::new(1.0, Color32::from_rgba_unmultiplied(col.r(), col.g(), col.b(), 140)));
            }
            Source::Cv => {
                let p = cam.to_screen(c);
                for r in [3.0f32, 6.0] {
                    painter.add(Shape::Path(PathShape::line(
                        (0..7).map(|i| {
                            let a = -2.3 + i as f32 * 0.27;
                            p + egui::vec2(a.cos(), a.sin()) * r * (cam.scale / 4.0).max(0.8)
                        }).collect(),
                        PathStroke::new(1.2, col),
                    )));
                }
            }
            Source::Loop => {}
        }
    }
}

fn paint_ped(painter: &egui::Painter, dc: &DrawCtx, p: &crate::sim::Ped) {
    let sim = dc.sim;
    let cam = dc.cam;
    let pos = sim.ped_pos_lerp(p, dc.alpha);
    let cw = &sim.net.crosswalks[p.cw];
    let dir = (cw.b - cw.a).norm() * p.dir;
    let fwd = if p.state == PedState::Leaving { sim.net.legs[cw.leg].d } else { dir };
    let alpha = if p.state == PedState::Leaving { (255.0 * (1.0 - p.t_state / 2.0)).clamp(0.0, 255.0) as u8 } else { 255 };
    let shirts = [
        Color32::from_rgb(220, 80, 80),
        Color32::from_rgb(80, 120, 220),
        Color32::from_rgb(240, 200, 70),
        Color32::from_rgb(90, 180, 110),
        Color32::from_rgb(200, 120, 200),
        Color32::from_rgb(240, 240, 240),
    ];
    let shirt = shirts[(p.color % shirts.len() as u32) as usize];
    let shirt = Color32::from_rgba_unmultiplied(shirt.r(), shirt.g(), shirt.b(), alpha);
    let walking = p.state != PedState::Waiting;
    let swing = if walking { (p.anim * 3.0).sin() * 0.28 } else { 0.0 };
    let side = fwd.right();
    // Legs.
    for (s, sw) in [(1.0, swing), (-1.0, -swing)] {
        let hip = pos + side * (0.12 * s);
        let foot = hip + fwd * sw;
        painter.line_segment([cam.to_screen(hip), cam.to_screen(foot)], Stroke::new((0.12 * cam.scale).max(1.0), Color32::from_rgba_unmultiplied(40, 40, 50, alpha)));
    }
    // Shoulders and head.
    let sh = rect_pts(pos, side, 0.28, 0.14);
    poly(painter, &cam, &sh, shirt);
    painter.circle_filled(cam.to_screen(pos), (0.13 * cam.scale).max(1.0), Color32::from_rgba_unmultiplied(70, 50, 40, alpha));
    if p.state == PedState::Waiting && cam.scale > 3.0 {
        let wait = sim.t - p.arrive_t;
        if wait > 20.0 {
            painter.circle_stroke(cam.to_screen(pos), 0.6 * cam.scale, Stroke::new(1.0, delay_color(wait)));
        }
    }
}

pub fn hud(painter: &egui::Painter, rect: Rect, sim: &Sim, ctrl_name: &str, speed: f32, paused: bool) {
    let t = sim.t;
    let txt = format!(
        "{:02}:{:02}:{:02}   ×{}{}   {}",
        (t / 3600.0) as u32,
        ((t / 60.0) as u32) % 60,
        (t as u32) % 60,
        speed,
        if paused { "  ⏸ paused" } else { "" },
        ctrl_name
    );
    let pos = rect.left_top() + egui::vec2(12.0, 10.0);
    let r = painter.text(pos + egui::vec2(1.0, 1.0), Align2::LEFT_TOP, &txt, FontId::monospace(14.0), Color32::from_black_alpha(160));
    painter.text(pos, Align2::LEFT_TOP, &txt, FontId::monospace(14.0), Color32::WHITE);
    if sim.signalized() {
        let st = &sim.net.stages[sim.signal.stage];
        let phase = match sim.signal.phase {
            Phase::Green => format!("GREEN {:.0}s", sim.signal.green_elapsed),
            Phase::Yellow => "YELLOW".into(),
            Phase::AllRed => "ALL-RED".into(),
        };
        let mut line = format!("Stage: {}  ·  {}", st.name, phase);
        if sim.preempt_active {
            line.push_str("  ·  🚨 EMERGENCY PREEMPTION");
        }
        let p2 = pos + egui::vec2(0.0, r.height() + 4.0);
        painter.text(p2 + egui::vec2(1.0, 1.0), Align2::LEFT_TOP, &line, FontId::proportional(13.0), Color32::from_black_alpha(160));
        painter.text(p2, Align2::LEFT_TOP, &line, FontId::proportional(13.0), Color32::from_rgb(230, 240, 255));
    }
}

pub const ASPHALT_COLOR: Color32 = ASPHALT_LIGHT;
