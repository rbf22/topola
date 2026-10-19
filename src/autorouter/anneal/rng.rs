// SPDX-FileCopyrightText: 2026 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Deterministic pseudo-randomness for the annealer.
//!
//! Seeded `xorshift64*` over a `splitmix64` seed, plus a Box–Muller
//! Gaussian — the same hand-rolled convention as `layer_seed` and
//! `shuffle_seed` elsewhere in the crate. No wall clock, no `rand`
//! dependency: a run is a pure function of `AnnealOptions::seed`, which
//! `.cmd` replay and history redo both rely on.

#[derive(Clone, Debug)]
pub struct AnnealRng {
    state: u64,
    gauss_stash: Option<f64>,
}

impl AnnealRng {
    pub fn new(seed: u64) -> Self {
        // splitmix64 finalizer — deterministic, well-distributed even
        // for seed 0 (whose raw xorshift state would otherwise stick).
        let mut x = seed.wrapping_add(0x9e3779b97f4a7c15);
        x = (x ^ (x >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
        x = (x ^ (x >> 27)).wrapping_mul(0x94d049bb133111eb);
        Self {
            state: (x ^ (x >> 31)).max(1),
            gauss_stash: None,
        }
    }

    /// Next u64 in the stream.
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545f4914f6cdd1d)
    }

    /// Uniform f64 in `[0, 1)`.
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// Uniform f64 in `[lo, hi)`.
    pub fn uniform_range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.uniform()
    }

    /// Uniform usize in `[0, n)`.
    pub fn below(&mut self, n: usize) -> usize {
        (self.next_u64() % n.max(1) as u64) as usize
    }

    /// Standard normal variate (Box–Muller; second variate stashed).
    pub fn gauss(&mut self) -> f64 {
        if let Some(stashed) = self.gauss_stash.take() {
            return stashed;
        }
        let u1 = self.uniform().max(f64::MIN_POSITIVE);
        let u2 = self.uniform();
        let r = (-2.0 * u1.ln()).sqrt();
        let (s, c) = (2.0 * std::f64::consts::PI * u2).sin_cos();
        self.gauss_stash = Some(r * s);
        r * c
    }

    /// Metropolis acceptance test for energy delta `de` at
    /// temperature `t`.
    pub fn accept(&mut self, de: f64, t: f64) -> bool {
        de <= 0.0 || self.uniform() < (-de / t).exp()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_same_stream() {
        let mut a = AnnealRng::new(42);
        let mut b = AnnealRng::new(42);
        for _ in 0..1000 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn zero_seed_does_not_stick() {
        let mut rng = AnnealRng::new(0);
        let mut seen = std::collections::BTreeSet::new();
        for _ in 0..64 {
            seen.insert(rng.next_u64());
        }
        assert!(seen.len() > 60);
    }

    #[test]
    fn gauss_is_roughly_normal() {
        let mut rng = AnnealRng::new(7);
        let n = 20000;
        let (mut sum, mut sq) = (0.0, 0.0);
        for _ in 0..n {
            let x = rng.gauss();
            sum += x;
            sq += x * x;
        }
        let mean = sum / n as f64;
        let var = sq / n as f64 - mean * mean;
        assert!(mean.abs() < 0.1);
        assert!((var - 1.0).abs() < 0.1);
    }
}
