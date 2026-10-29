//! Small deterministic PRNG (SplitMix64 seeded xoshiro256**) so runs are reproducible
//! across controllers: the same seed produces the same arrival stream.

#[derive(Clone, Debug)]
pub struct Rng {
    s: [u64; 4],
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        let mut z = seed.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut s = [0u64; 4];
        for v in s.iter_mut() {
            z = z.wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut x = z;
            x = (x ^ (x >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            x = (x ^ (x >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            *v = x ^ (x >> 31);
        }
        Rng { s }
    }

    pub fn next_u64(&mut self) -> u64 {
        let result = self.s[1].wrapping_mul(5).rotate_left(7).wrapping_mul(9);
        let t = self.s[1] << 17;
        self.s[2] ^= self.s[0];
        self.s[3] ^= self.s[1];
        self.s[1] ^= self.s[2];
        self.s[0] ^= self.s[3];
        self.s[2] ^= t;
        self.s[3] = self.s[3].rotate_left(45);
        result
    }

    /// Uniform in [0, 1).
    pub fn f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    pub fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.f64()
    }

    pub fn below(&mut self, n: usize) -> usize {
        ((self.f64() * n as f64) as usize).min(n.saturating_sub(1))
    }

    pub fn chance(&mut self, p: f64) -> bool {
        self.f64() < p
    }

    /// Standard normal via Box-Muller.
    pub fn normal(&mut self) -> f64 {
        let u1 = self.f64().max(1e-12);
        let u2 = self.f64();
        (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
    }

    /// Exponential inter-arrival time for a rate (events per second).
    pub fn exp(&mut self, rate: f64) -> f64 {
        if rate <= 0.0 {
            return f64::INFINITY;
        }
        -(1.0 - self.f64()).max(1e-12).ln() / rate
    }

    /// Pick an index from non-negative weights.
    pub fn weighted(&mut self, w: &[f64]) -> usize {
        let total: f64 = w.iter().sum();
        if total <= 0.0 {
            return 0;
        }
        let mut r = self.f64() * total;
        for (i, &x) in w.iter().enumerate() {
            if r < x {
                return i;
            }
            r -= x;
        }
        w.len() - 1
    }
}
