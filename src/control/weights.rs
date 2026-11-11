// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Where trained network weights come from and go to.
//!
//! Loading looks in `weights/` under the working directory (development and training),
//! then in the per-user data directory (weights saved from the app), then falls back to
//! the copies built into the binary, so an installed app works anywhere. A copy that
//! can't be read or doesn't load is skipped, so a damaged file doesn't hide a good copy
//! further down the list. Saving goes to `weights/` when that directory exists in the
//! working directory, otherwise to the per-user data directory.

use std::path::{Path, PathBuf};

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

/// Weights file `file` as `parse` loads it from the first copy it accepts, and where
/// that copy came from.
pub fn load<T>(file: &str, parse: impl FnMut(&str) -> Option<T>) -> Option<(T, String)> {
    let dirs: Vec<PathBuf> = std::iter::once(PathBuf::from("weights")).chain(user_dir().map(|d| d.join("weights"))).collect();
    load_from(&dirs, file, parse)
}

/// Like [`load`], but looking in `dirs`, in order, before the built-in copy.
pub fn load_from<T>(dirs: &[impl AsRef<Path>], file: &str, mut parse: impl FnMut(&str) -> Option<T>) -> Option<(T, String)> {
    for p in dirs.iter().map(|d| d.as_ref().join(file)) {
        if let Ok(txt) = std::fs::read_to_string(&p)
            && let Some(v) = parse(&txt)
        {
            return Some((v, p.display().to_string()));
        }
    }
    let txt = EMBEDDED.iter().find(|e| e.0 == file)?.1;
    parse(txt).map(|v| (v, format!("built-in {file}")))
}

/// Where to save weights file `file` (the directory may still need creating).
pub fn save_path(file: &str) -> PathBuf {
    let local = PathBuf::from("weights");
    if local.is_dir() {
        return local.join(file);
    }
    user_dir().map(|d| d.join("weights")).unwrap_or(local).join(file)
}
