// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! 2D geometry: vectors, arc-length parametrised polylines, Bezier/arc builders and
//! path-path intersection used to discover conflict points inside the intersection.
//!
//! World coordinates are metres with +x = east and +y = south (screen convention).

use std::ops::{Add, Mul, Neg, Sub};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct V2 {
    pub x: f64,
    pub y: f64,
}

pub const fn v2(x: f64, y: f64) -> V2 {
    V2 { x, y }
}

impl V2 {
    pub fn len(self) -> f64 {
        self.x.hypot(self.y)
    }
    pub fn norm(self) -> V2 {
        let l = self.len();
        if l < 1e-12 { self } else { v2(self.x / l, self.y / l) }
    }
    pub fn dot(self, o: V2) -> f64 {
        self.x * o.x + self.y * o.y
    }
    pub fn cross(self, o: V2) -> f64 {
        self.x * o.y - self.y * o.x
    }
    pub fn dist(self, o: V2) -> f64 {
        (self - o).len()
    }
    /// Perpendicular pointing to the right of this heading (in y-down coordinates).
    pub fn right(self) -> V2 {
        v2(-self.y, self.x)
    }
    pub fn left(self) -> V2 {
        v2(self.y, -self.x)
    }
    pub fn angle(self) -> f64 {
        self.y.atan2(self.x)
    }
    pub fn from_angle(a: f64) -> V2 {
        v2(a.cos(), a.sin())
    }
    pub fn lerp(self, o: V2, t: f64) -> V2 {
        self + (o - self) * t
    }
}

impl Add for V2 {
    type Output = V2;
    fn add(self, o: V2) -> V2 {
        v2(self.x + o.x, self.y + o.y)
    }
}
impl Sub for V2 {
    type Output = V2;
    fn sub(self, o: V2) -> V2 {
        v2(self.x - o.x, self.y - o.y)
    }
}
impl Mul<f64> for V2 {
    type Output = V2;
    fn mul(self, k: f64) -> V2 {
        v2(self.x * k, self.y * k)
    }
}
impl Neg for V2 {
    type Output = V2;
    fn neg(self) -> V2 {
        v2(-self.x, -self.y)
    }
}

/// Uniform sampling step of resampled paths (metres).
pub const DS: f64 = 0.5;

/// Polyline resampled at uniform arc-length steps for O(1) position lookup.
#[derive(Clone, Debug)]
pub struct Path {
    pub pts: Vec<V2>,
    pub length: f64,
    /// Smallest radius of curvature along the path (f64::INFINITY for straight).
    pub min_radius: f64,
}

impl Path {
    pub fn new(input: &[V2]) -> Path {
        assert!(input.len() >= 2);
        let mut cum = vec![0.0];
        for w in input.windows(2) {
            cum.push(cum.last().unwrap() + w[0].dist(w[1]));
        }
        let length = *cum.last().unwrap();
        let n = ((length / DS).ceil() as usize).max(1) + 1;
        let mut pts = Vec::with_capacity(n);
        let mut j = 0;
        for i in 0..n {
            let s = (i as f64 * DS).min(length);
            while j + 1 < cum.len() - 1 && cum[j + 1] < s {
                j += 1;
            }
            let seg = cum[j + 1] - cum[j];
            let t = if seg > 1e-12 { (s - cum[j]) / seg } else { 0.0 };
            pts.push(input[j].lerp(input[j + 1], t.clamp(0.0, 1.0)));
        }
        let mut p = Path { pts, length, min_radius: f64::INFINITY };
        p.min_radius = p.compute_min_radius();
        p
    }

    fn compute_min_radius(&self) -> f64 {
        // Curvature from heading change over a 2 m window to smooth sampling noise.
        let k = 4usize;
        let n = self.pts.len();
        if n < 2 * k + 1 {
            return f64::INFINITY;
        }
        let mut rmin = f64::INFINITY;
        for i in k..n - k {
            let h0 = (self.pts[i] - self.pts[i - k]).angle();
            let h1 = (self.pts[i + k] - self.pts[i]).angle();
            let mut dh = h1 - h0;
            while dh > std::f64::consts::PI {
                dh -= std::f64::consts::TAU;
            }
            while dh < -std::f64::consts::PI {
                dh += std::f64::consts::TAU;
            }
            let arc = k as f64 * DS;
            if dh.abs() > 1e-4 {
                rmin = rmin.min(arc / dh.abs());
            }
        }
        rmin
    }

    fn seg_index(&self, s: f64) -> (usize, f64) {
        let n = self.pts.len();
        let i = ((s / DS) as usize).min(n - 2);
        let s0 = i as f64 * DS;
        let s1 = ((i + 1) as f64 * DS).min(self.length);
        let t = if s1 - s0 > 1e-12 { (s - s0) / (s1 - s0) } else { 0.0 };
        (i, t)
    }

    /// Position at arc length `s`; extrapolates linearly beyond either end.
    pub fn point(&self, s: f64) -> V2 {
        if s <= 0.0 {
            return self.pts[0] + self.tangent(0.0) * s;
        }
        if s >= self.length {
            return *self.pts.last().unwrap() + self.tangent(self.length) * (s - self.length);
        }
        let (i, t) = self.seg_index(s);
        self.pts[i].lerp(self.pts[i + 1], t)
    }

    pub fn tangent(&self, s: f64) -> V2 {
        let (i, _) = self.seg_index(s.clamp(0.0, self.length));
        (self.pts[i + 1] - self.pts[i]).norm()
    }

    pub fn start(&self) -> V2 {
        self.pts[0]
    }
    pub fn end(&self) -> V2 {
        *self.pts.last().unwrap()
    }

    /// Arc length of a sample index.
    fn s_of(&self, i: usize, t: f64) -> f64 {
        let s0 = i as f64 * DS;
        let s1 = ((i + 1) as f64 * DS).min(self.length);
        s0 + (s1 - s0) * t
    }

    /// All crossing points with another path as (s_self, s_other).
    pub fn intersections(&self, o: &Path) -> Vec<(f64, f64)> {
        let mut out: Vec<(f64, f64)> = Vec::new();
        for i in 0..self.pts.len() - 1 {
            let a = self.pts[i];
            let b = self.pts[i + 1];
            for j in 0..o.pts.len() - 1 {
                let c = o.pts[j];
                let d = o.pts[j + 1];
                if let Some((t, u)) = seg_intersect(a, b, c, d) {
                    let sa = self.s_of(i, t);
                    let sb = o.s_of(j, u);
                    if !out.iter().any(|&(x, y)| (x - sa).abs() < 1.5 && (y - sb).abs() < 1.5) {
                        out.push((sa, sb));
                    }
                }
            }
        }
        out
    }

    /// Closest approach between two paths as (s_self, s_other, distance).
    pub fn closest(&self, o: &Path) -> (f64, f64, f64) {
        let mut best = (0.0, 0.0, f64::INFINITY);
        for (i, p) in self.pts.iter().enumerate() {
            for (j, q) in o.pts.iter().enumerate() {
                let d = p.dist(*q);
                if d < best.2 {
                    best = ((i as f64 * DS).min(self.length), (j as f64 * DS).min(o.length), d);
                }
            }
        }
        best
    }
}

pub fn seg_intersect(a: V2, b: V2, c: V2, d: V2) -> Option<(f64, f64)> {
    let r = b - a;
    let s = d - c;
    let den = r.cross(s);
    if den.abs() < 1e-12 {
        return None;
    }
    let t = (c - a).cross(s) / den;
    let u = (c - a).cross(r) / den;
    if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) { Some((t, u)) } else { None }
}

pub fn cubic(p0: V2, p1: V2, p2: V2, p3: V2, n: usize) -> Vec<V2> {
    (0..=n)
        .map(|i| {
            let t = i as f64 / n as f64;
            let u = 1.0 - t;
            p0 * (u * u * u) + p1 * (3.0 * u * u * t) + p2 * (3.0 * u * t * t) + p3 * (t * t * t)
        })
        .collect()
}

/// Smooth connector from `p0` heading `h0` to `p3` heading `h3`.
/// Uses the ray intersection as a corner guide so turns approximate circular arcs.
pub fn connector(p0: V2, h0: V2, p3: V2, h3: V2) -> Vec<V2> {
    let span = p0.dist(p3);
    let den = h0.cross(h3);
    let (k0, k3) = if den.abs() > 0.05 {
        // Solve p0 + h0*a = p3 - h3*b for the corner point.
        let w = p3 - p0;
        let a = w.cross(h3) / den;
        let b = w.cross(h0) / den;
        if a > 0.0 && b > 0.0 { (a * 0.5523, b * 0.5523) } else { (span / 3.0, span / 3.0) }
    } else {
        (span / 3.0, span / 3.0)
    };
    cubic(p0, p0 + h0 * k0, p3 - h3 * k3, p3, 48)
}

/// Circular arc points from angle a0 to a1 (radians, any direction).
pub fn arc(c: V2, r: f64, a0: f64, a1: f64) -> Vec<V2> {
    let n = (((a1 - a0).abs() * r) / 0.75).ceil().max(4.0) as usize;
    (0..=n)
        .map(|i| {
            let a = a0 + (a1 - a0) * i as f64 / n as f64;
            c + V2::from_angle(a) * r
        })
        .collect()
}
