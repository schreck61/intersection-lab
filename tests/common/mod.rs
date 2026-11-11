// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Shared by the integration tests that run many simulations at once.

use std::thread::JoinHandle;

/// Run `job` on a thread of its own, except when measuring test coverage, which runs it
/// right away. Code built for coverage counts what it runs in counters that all threads
/// update, so threads running the same code at once slow one another down many times
/// over (cargo-llvm-cov sets `cfg(coverage)`; see CONTRIBUTING.md).
pub fn spawn<T: Send + 'static>(job: impl FnOnce() -> T + Send + 'static) -> JoinHandle<T> {
    if cfg!(coverage) {
        let done = job();
        std::thread::spawn(move || done)
    } else {
        std::thread::spawn(job)
    }
}
