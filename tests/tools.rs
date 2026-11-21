// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The developer tools in src/bin, run briefly: `bench` compares every automatic
//! strategy and studies the sensor packages, and `train` trains, saves and resumes both
//! learning controllers. Each runs in a scratch directory of its own, with its home
//! directory there too, so nothing reads or writes the repository's weights or the
//! user's.

use intersection_lab::control::CtrlKind;
use intersection_lab::sim::ControlMode;
use intersection_lab::sim::network::LayoutKind;
use intersection_lab::sim::sensors::Conditions;
use std::path::{Path, PathBuf};
use std::process::Command;

/// An empty scratch directory for test `name`.
fn scratch(name: &str) -> PathBuf {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("tools").join(name);
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// Run `exe` with `args` and `vars` in `dir`, with its per-user data directory there
/// too; what it printed.
fn run(exe: &str, dir: &Path, args: &[&str], vars: &[(&str, &str)]) -> String {
    let out = Command::new(exe)
        .args(args)
        .current_dir(dir)
        .env("HOME", dir)
        .env("XDG_DATA_HOME", dir)
        .env("APPDATA", dir)
        .env_remove("RESUME")
        .env_remove("SEED")
        .env_remove("HIDDEN")
        .envs(vars.iter().copied())
        .output()
        .unwrap();
    let stdout = String::from_utf8(out.stdout).unwrap();
    assert!(
        out.status.success(),
        "{exe} {args:?} failed:\n{stdout}\n{}",
        String::from_utf8_lossy(&out.stderr)
    );
    stdout
}

fn bench(dir: &Path, args: &[&str]) -> String {
    run(env!("CARGO_BIN_EXE_bench"), dir, args, &[])
}

/// The rows of a strategy table, each with the overlaps between vehicles that the safety
/// check counted in its runs (`bench` turns the check on for these tables).
fn collisions(table: &str) -> Vec<(String, f64)> {
    table
        .lines()
        .filter(|l| l.ends_with("s)"))
        .map(|l| {
            let fields: Vec<&str> = l.split_whitespace().collect();
            let coll = fields[fields.len() - 2].parse().unwrap();
            (l[..34].trim().to_string(), coll)
        })
        .collect()
}

#[test]
fn bench_compares_every_strategy_on_every_layout() {
    let dir = scratch("bench_all");
    let out = bench(&dir, &["all", "0.2", "1", "0.3", "surges"]);
    for kind in LayoutKind::ALL {
        assert!(
            out.contains(&format!("=== {} | 0 min x 1 seeds | demand x0.30 | Surges", kind.name())),
            "{out}"
        );
    }
    let rows = collisions(&out);
    for c in CtrlKind::ALL {
        assert!(rows.iter().any(|r| r.0 == c.name()), "no row for {}:\n{out}", c.name());
    }
    for m in
        [ControlMode::AllWayStop, ControlMode::TwoWayStop, ControlMode::Roundabout, ControlMode::Cooperative]
    {
        assert!(rows.iter().any(|r| r.0 == m.name()), "no row for {}:\n{out}", m.name());
    }
    for (name, coll) in rows {
        assert_eq!(coll, 0.0, "{name} had collisions:\n{out}");
    }
}

#[test]
fn bench_takes_each_arrival_pattern() {
    let dir = scratch("bench_patterns");
    for (arg, pattern) in [("platoons", "Platoons"), ("rush", "RushHour"), ("random", "Random")] {
        let out = bench(&dir, &["tee", "0.2", "1", "0.5", arg]);
        assert!(out.contains(&format!("| demand x0.50 | {pattern} |")), "{out}");
        assert_eq!(
            collisions(&out).len(),
            CtrlKind::ALL.len() + ControlMode::options(LayoutKind::Tee).len() - 1
        );
    }
}

#[test]
fn bench_studies_the_sensor_packages() {
    let dir = scratch("bench_sensors");
    let out = bench(&dir, &["sensors", "tee", "0.2", "1"]);
    assert!(
        out.contains("Sensor packages (realistic errors, clear day; last column ideal sensors)"),
        "{out}"
    );
    assert!(out.contains("Weather and detector faults"), "{out}");
    for c in Conditions::ALL {
        assert!(out.contains(c.name()), "no column for {}:\n{out}", c.name());
    }
    // One row per sensor-driven controller in each table.
    for c in
        [CtrlKind::Actuated, CtrlKind::MaxPressure, CtrlKind::Predictive, CtrlKind::DeepRl, CtrlKind::Hybrid]
    {
        assert_eq!(out.lines().filter(|l| l.starts_with(&format!("{} ", c.short()))).count(), 2, "{out}");
    }
}

/// Train on the T-intersection: one short episode, small networks.
fn train(dir: &Path, agent: &str, vars: &[(&str, &str)]) -> String {
    run(env!("CARGO_BIN_EXE_train"), dir, &["tee", "1", "150", agent], vars)
}

#[test]
fn train_saves_a_new_deep_rl_network() {
    let dir = scratch("train_dqn");
    let out = train(&dir, "dqn", &[("HIDDEN", "8")]);
    assert!(out.contains("[tee] validation baselines (person delay s): fixed "), "{out}");
    assert!(out.contains("[tee:dqn] ep    1 "), "{out}");
    assert!(out.contains("[tee:dqn] done. best validation person-delay"), "{out}");
    assert!(out.contains("[tee:dqn] TEST person delay / pedestrian wait (s): fixed "), "{out}");
    // Saved only if it kept pedestrian waits near the predictive controller's.
    let saved = dir.join("weights/dqn_tee.json");
    assert_eq!(saved.exists(), out.contains("(saved)"), "{out}");
    if !saved.exists() {
        assert!(out.contains("no checkpoint met the pedestrian limit, weights not written"), "{out}");
    }
}

#[test]
fn train_resumes_from_the_built_in_deep_rl_network() {
    let dir = scratch("train_dqn_resume");
    let out = train(&dir, "dqn", &[("RESUME", "1"), ("SEED", "3")]);
    assert!(out.contains("[tee:dqn] resumed from saved weights, validation pd "), "{out}");
    assert!(out.contains("[tee:dqn] TEST person delay"), "{out}");
}

#[test]
fn train_saves_the_hybrid_tuning() {
    let dir = scratch("train_hybrid");
    let out = train(&dir, "hybrid", &[("HIDDEN", "8")]);
    assert!(out.contains("[tee:hybrid] best static tuning: "), "{out}");
    assert!(out.contains("[tee:hybrid] TEST person delay / pedestrian wait (s): fixed "), "{out}");
    assert!(out.contains("  static "), "{out}");
    // The best fixed tuning is saved before training, as the fallback.
    assert!(dir.join("weights/hybrid_tee.json").exists(), "{out}");
}

#[test]
fn train_stops_when_it_cannot_write_to_the_weights_folder() {
    // `weights` is a file here, so the folder can't be made. `train` used to turn to the
    // per-user data directory then, and write over the weights saved from the app.
    let dir = scratch("train_no_folder");
    std::fs::write(dir.join("weights"), "not a folder").unwrap();
    let per_user = [
        "Library/Application Support/Intersection Lab/weights",
        "Intersection Lab/weights",
        "intersection-lab/weights",
    ];
    for folder in per_user {
        std::fs::create_dir_all(dir.join(folder)).unwrap();
        std::fs::write(dir.join(folder).join("hybrid_tee.json"), "saved from the app").unwrap();
    }
    let out = Command::new(env!("CARGO_BIN_EXE_train"))
        .args(["tee", "1", "150", "hybrid"])
        .current_dir(&dir)
        .env("HOME", &dir)
        .env("XDG_DATA_HOME", &dir)
        .env("APPDATA", &dir)
        .env_remove("RESUME")
        .env_remove("SEED")
        .env("HIDDEN", "8")
        .output()
        .unwrap();
    let said = String::from_utf8_lossy(&out.stderr);
    assert!(!out.status.success(), "train went on without a weights folder:\n{said}");
    assert!(said.contains("train: can't make the folder weights"), "{said}");
    for folder in per_user {
        let kept = std::fs::read_to_string(dir.join(folder).join("hybrid_tee.json")).unwrap();
        assert_eq!(kept, "saved from the app", "{folder}");
    }
}

#[test]
fn train_stops_when_it_cannot_write_a_weights_file() {
    // The folder is there, but a folder stands where the weights file goes.
    let dir = scratch("train_no_file");
    std::fs::create_dir_all(dir.join("weights").join("hybrid_tee.json")).unwrap();
    let out = Command::new(env!("CARGO_BIN_EXE_train"))
        .args(["tee", "1", "150", "hybrid"])
        .current_dir(&dir)
        .env("HOME", &dir)
        .env("XDG_DATA_HOME", &dir)
        .env("APPDATA", &dir)
        .env_remove("RESUME")
        .env_remove("SEED")
        .env("HIDDEN", "8")
        .output()
        .unwrap();
    let said = String::from_utf8_lossy(&out.stderr);
    let printed = String::from_utf8_lossy(&out.stdout);
    assert!(!out.status.success(), "train went on without saving:\n{printed}\n{said}");
    assert!(said.contains("train: can't write weights"), "{said}");
    assert!(!printed.contains("TEST person delay"), "{printed}");
}

#[test]
fn train_stops_at_a_layout_it_does_not_train() {
    // A mistyped layout, and one without signals, train nothing: say so, and leave no
    // empty `weights` folder behind for the app to save into.
    for layout in ["arterial", "roundabout"] {
        let dir = scratch("train_no_layout");
        let out = Command::new(env!("CARGO_BIN_EXE_train"))
            .args([layout, "1", "150", "hybrid"])
            .current_dir(&dir)
            .env("HOME", &dir)
            .env("XDG_DATA_HOME", &dir)
            .env("APPDATA", &dir)
            .output()
            .unwrap();
        let said = String::from_utf8_lossy(&out.stderr);
        assert!(!out.status.success(), "train {layout} reported success");
        assert!(said.contains(&format!("train: no signalized layout is named {layout}")), "{said}");
        assert!(!dir.join("weights").exists(), "train {layout} left a weights folder");
    }
}

#[test]
fn train_resumes_the_built_in_hybrid() {
    let dir = scratch("train_hybrid_resume");
    let out = train(&dir, "hybrid", &[("RESUME", "1")]);
    assert!(out.contains("[tee:hybrid] resumed from saved weights, validation pd "), "{out}");
    assert!(dir.join("weights/hybrid_tee.json").exists(), "{out}");
}
