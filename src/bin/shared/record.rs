// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! What `bench` and `train` record about a run, so that a result can be traced to what
//! produced it: the source the program was built from, and the weights file each
//! learned controller loaded. Weights are looked for in the working directory and the
//! per-user data directory before the copies built in (`control::weights`), so the same
//! command can run different networks on different machines.

use intersection_lab::control::Controller;
use intersection_lab::sim::stats::mean_ci95;

/// The crate's version, and the commit of the source tree this program was built from
/// as `git describe` reports it when the program runs: `v0.8.0-3-g1a2b3c4d5e6f` is
/// commit 1a2b3c4d5e6f, three commits after the tag v0.8.0, and `-dirty` means tracked
/// files differ from it. Without git, or outside a checkout, the commit is unknown.
pub fn source() -> String {
    let described = std::process::Command::new("git")
        .args(["-C", env!("CARGO_MANIFEST_DIR")])
        .args(["describe", "--tags", "--long", "--always", "--dirty", "--abbrev=12"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty());
    format!(
        "intersection-lab {} ({})",
        env!("CARGO_PKG_VERSION"),
        described.as_deref().unwrap_or("commit unknown")
    )
}

/// Where a learned controller's weights came from and their fingerprint
/// (`control::weights::fingerprint`), or `None` for a controller that learns nothing.
pub fn weights(c: &mut dyn Controller) -> Option<String> {
    let (from, fingerprint) = if let Some(rl) = c.as_rl() {
        (rl.loaded_from.clone(), rl.loaded_fingerprint)
    } else {
        let h = c.as_hybrid()?;
        (h.loaded_from.clone(), h.loaded_fingerprint)
    };
    Some(match (from, fingerprint) {
        (Some(from), Some(f)) => format!("{from} (fnv1a {f:016x})"),
        _ => "no weights file, untrained".into(),
    })
}

/// A paired comparison: the mean of `diffs`, the differences between two strategies run
/// on the same seeds or scenarios, and from two of them up the half-width of its 95%
/// confidence interval, as `+1.3±0.4`.
pub fn paired(diffs: &[f64]) -> String {
    match mean_ci95(diffs) {
        (m, Some(h)) => format!("{m:+.1}±{h:.1}"),
        (m, None) => format!("{m:+.1}"),
    }
}
