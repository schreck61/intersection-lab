//! Minimal dense neural network (ReLU MLP) with backprop and Adam, no dependencies.

use crate::sim::rng::Rng;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Dense {
    pub n_in: usize,
    pub n_out: usize,
    /// Row-major [n_out][n_in].
    pub w: Vec<f32>,
    pub b: Vec<f32>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Mlp {
    pub layers: Vec<Dense>,
}

impl Mlp {
    pub fn new(sizes: &[usize], rng: &mut Rng) -> Mlp {
        let layers = sizes
            .windows(2)
            .map(|w| {
                let (n_in, n_out) = (w[0], w[1]);
                let scale = (2.0 / n_in as f64).sqrt();
                Dense {
                    n_in,
                    n_out,
                    w: (0..n_in * n_out).map(|_| (rng.normal() * scale) as f32).collect(),
                    b: vec![0.0; n_out],
                }
            })
            .collect();
        Mlp { layers }
    }

    pub fn n_in(&self) -> usize {
        self.layers[0].n_in
    }
    pub fn n_out(&self) -> usize {
        self.layers.last().unwrap().n_out
    }

    /// Forward pass keeping every layer's (post-activation) output.
    pub fn forward_all(&self, x: &[f32]) -> Vec<Vec<f32>> {
        let mut acts = vec![x.to_vec()];
        let nl = self.layers.len();
        for (li, l) in self.layers.iter().enumerate() {
            let inp = acts.last().unwrap();
            let mut out = l.b.clone();
            for o in 0..l.n_out {
                let row = &l.w[o * l.n_in..(o + 1) * l.n_in];
                let mut s = 0.0f32;
                for i in 0..l.n_in {
                    s += row[i] * inp[i];
                }
                out[o] += s;
                if li + 1 < nl && out[o] < 0.0 {
                    out[o] = 0.0;
                }
            }
            acts.push(out);
        }
        acts
    }

    pub fn forward(&self, x: &[f32]) -> Vec<f32> {
        self.forward_all(x).pop().unwrap()
    }

    /// Accumulate gradients for d(loss)/d(output) = `g_out`.
    pub fn backward(&self, acts: &[Vec<f32>], g_out: &[f32], grads: &mut Grads) {
        let mut g = g_out.to_vec();
        for li in (0..self.layers.len()).rev() {
            let l = &self.layers[li];
            let inp = &acts[li];
            let gw = &mut grads.w[li];
            let gb = &mut grads.b[li];
            let mut g_in = vec![0.0f32; l.n_in];
            for o in 0..l.n_out {
                let go = g[o];
                if go == 0.0 {
                    continue;
                }
                gb[o] += go;
                let row = &l.w[o * l.n_in..(o + 1) * l.n_in];
                let grow = &mut gw[o * l.n_in..(o + 1) * l.n_in];
                for i in 0..l.n_in {
                    grow[i] += go * inp[i];
                    g_in[i] += go * row[i];
                }
            }
            if li > 0 {
                // ReLU derivative of the previous layer's output.
                for i in 0..l.n_in {
                    if inp[i] <= 0.0 {
                        g_in[i] = 0.0;
                    }
                }
            }
            g = g_in;
        }
    }

    /// Polyak averaging toward `src`.
    pub fn soft_update(&mut self, src: &Mlp, tau: f32) {
        for (a, b) in self.layers.iter_mut().zip(&src.layers) {
            for (x, y) in a.w.iter_mut().zip(&b.w) {
                *x += tau * (y - *x);
            }
            for (x, y) in a.b.iter_mut().zip(&b.b) {
                *x += tau * (y - *x);
            }
        }
    }
}

pub struct Grads {
    pub w: Vec<Vec<f32>>,
    pub b: Vec<Vec<f32>>,
}

impl Grads {
    pub fn zeros(m: &Mlp) -> Grads {
        Grads {
            w: m.layers.iter().map(|l| vec![0.0; l.w.len()]).collect(),
            b: m.layers.iter().map(|l| vec![0.0; l.b.len()]).collect(),
        }
    }
    pub fn clear(&mut self) {
        for v in self.w.iter_mut().chain(self.b.iter_mut()) {
            v.iter_mut().for_each(|x| *x = 0.0);
        }
    }
}

pub struct Adam {
    m: Grads,
    v: Grads,
    t: i32,
    pub lr: f32,
}

impl Adam {
    pub fn new(net: &Mlp, lr: f32) -> Adam {
        Adam { m: Grads::zeros(net), v: Grads::zeros(net), t: 0, lr }
    }

    pub fn apply(&mut self, net: &mut Mlp, g: &Grads, scale: f32) {
        let (b1, b2, eps) = (0.9f32, 0.999f32, 1e-8f32);
        self.t += 1;
        let c1 = 1.0 - b1.powi(self.t);
        let c2 = 1.0 - b2.powi(self.t);
        for li in 0..net.layers.len() {
            let l = &mut net.layers[li];
            for (params, gs, ms, vs) in [
                (&mut l.w, &g.w[li], &mut self.m.w[li], &mut self.v.w[li]),
                (&mut l.b, &g.b[li], &mut self.m.b[li], &mut self.v.b[li]),
            ] {
                for i in 0..params.len() {
                    let gi = gs[i] * scale;
                    ms[i] = b1 * ms[i] + (1.0 - b1) * gi;
                    vs[i] = b2 * vs[i] + (1.0 - b2) * gi * gi;
                    let mh = ms[i] / c1;
                    let vh = vs[i] / c2;
                    params[i] -= self.lr * mh / (vh.sqrt() + eps);
                }
            }
        }
    }
}
