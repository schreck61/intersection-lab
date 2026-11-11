// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The developer tools in src/bin, run briefly: `bench` compares every strategy and
//! studies the sensor packages, and `train` trains, saves and resumes both learning
//! controllers. Each runs in a scratch directory of its own, with its home directory
//! there too, so nothing reads or writes the repository's weights or the user's.

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

/// The rows of a strategy table, with the number of collisions each run had.
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
fn train_resumes_the_built_in_hybrid() {
    let dir = scratch("train_hybrid_resume");
    let out = train(&dir, "hybrid", &[("RESUME", "1")]);
    assert!(out.contains("[tee:hybrid] resumed from saved weights, validation pd "), "{out}");
    assert!(dir.join("weights/hybrid_tee.json").exists(), "{out}");
}
