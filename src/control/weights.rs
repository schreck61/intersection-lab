// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Where trained network weights come from and go to.
//!
//! Loading looks in `weights/` under the working directory (development and training),
//! then in the per-user data directory (weights saved from the app), then falls back to
//! the copies built into the binary, so an installed app works anywhere. Saving goes to
//! `weights/` when that directory exists in the working directory, otherwise to the
//! per-user data directory.

use std::path::PathBuf;

/// Weights shipped with the application.
const EMBEDDED: &[(&str, &str)] = &[
    ("dqn_arterial4.json", include_str!("../../weights/dqn_arterial4.json")),
    ("dqn_majorminor4.json", include_str!("../../weights/dqn_majorminor4.json")),
    ("dqn_town4.json", include_str!("../../weights/dqn_town4.json")),
    ("dqn_tee.json", include_str!("../../weights/dqn_tee.json")),
    ("hybrid_arterial4.json", include_str!("../../weights/hybrid_arterial4.json")),
    ("hybrid_majorminor4.json", include_str!("../../weights/hybrid_majorminor4.json")),
    ("hybrid_town4.json", include_str!("../../weights/hybrid_town4.json")),
    ("hybrid_tee.json", include_str!("../../weights/hybrid_tee.json")),
];

/// Per-user data directory for the application.
pub fn user_dir() -> Option<PathBuf> {
    let home = std::env::var_os("HOME").map(PathBuf::from);
    if cfg!(target_os = "macos") {
        home.map(|h| h.join("Library/Application Support/Intersection Lab"))
    } else if cfg!(windows) {
        std::env::var_os("APPDATA").map(|a| PathBuf::from(a).join("Intersection Lab"))
    } else {
        std::env::var_os("XDG_DATA_HOME")
            .map(PathBuf::from)
            .or_else(|| home.map(|h| h.join(".local/share")))
            .map(|d| d.join("intersection-lab"))
    }
}

/// Contents of weights file `file` and where they came from.
pub fn load(file: &str) -> Option<(String, String)> {
    let local = PathBuf::from("weights").join(file);
    let user = user_dir().map(|d| d.join("weights").join(file));
    for p in std::iter::once(local).chain(user) {
        if let Ok(txt) = std::fs::read_to_string(&p) {
            return Some((txt, p.display().to_string()));
        }
    }
    EMBEDDED.iter().find(|e| e.0 == file).map(|e| (e.1.to_string(), format!("built-in {file}")))
}

/// Where to save weights file `file` (the directory may still need creating).
pub fn save_path(file: &str) -> PathBuf {
    let local = PathBuf::from("weights");
    if local.is_dir() {
        return local.join(file);
    }
    user_dir().map(|d| d.join("weights")).unwrap_or(local).join(file)
}
