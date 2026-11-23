// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The developer tools in src/bin, run briefly: `bench` compares every automatic
//! strategy and studies the sensor packages, and `train` trains, saves and resumes both
//! learning controllers. Each runs in a scratch directory of its own, with its home
//! directory there too, so nothing reads or writes the repository's weights or the
//! user's.

use intersection_lab::control::{CtrlKind, weights};
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

/// The fields of one CSV line, with quoted fields unquoted.
fn csv_fields(line: &str) -> Vec<String> {
    let (mut fields, mut f, mut quoted) = (vec![], String::new(), false);
    let mut chars = line.chars().peekable();
    while let Some(c) = chars.next() {
        match (c, quoted) {
            ('"', true) if chars.peek() == Some(&'"') => {
                f.push('"');
                chars.next();
            }
            ('"', _) => quoted = !quoted,
            (',', false) => fields.push(std::mem::take(&mut f)),
            _ => f.push(c),
        }
    }
    fields.push(f);
    fields
}

/// The built-in copy of weights file `file`, and its fingerprint as `bench` prints it.
fn built_in(file: &str) -> String {
    let txt = std::fs::read_to_string(Path::new("weights").join(file)).unwrap();
    format!("built-in {file} (fnv1a {:016x})", weights::fingerprint(&txt))
}

#[test]
fn bench_records_what_it_ran() {
    let dir = scratch("bench_record");
    let csv = dir.join("runs.csv");
    let out = bench(&dir, &["tee", "0.2", "2", "0.5", "--csv", csv.to_str().unwrap()]);
    let lines: Vec<&str> = out.lines().collect();
    assert!(lines[0].starts_with(&format!("intersection-lab {} (", env!("CARGO_PKG_VERSION"))), "{out}");
    assert_eq!(
        lines[1],
        "5-minute warm-up before each run; seeds 1000 to 1001; sensors: loops + camera + radar, \
         realistic errors, Clear day"
    );
    // Nothing in this directory or its per-user data directory: the built-in copies.
    let used =
        format!("weights: DeepRL {}; Hybrid {}", built_in("dqn_tee.json"), built_in("hybrid_tee.json"));
    assert!(lines.contains(&used.as_str()), "{out}");
    // Each row: the means, and person delay against predictive control's on the same seeds.
    let rows: Vec<(&str, Vec<&str>)> = lines
        .iter()
        .filter(|l| l.ends_with("s)"))
        .map(|l| (l[..34].trim(), l[34..].split_whitespace().collect()))
        .collect();
    for (name, fields) in &rows {
        match *name {
            "Predictive (schedule-driven MPC)" => assert_eq!(fields[5], "(base)"),
            _ => assert!(fields[5].contains('±'), "{name}: {}", fields[5]),
        }
    }
    // The CSV: one row per strategy and seed, whose means are the table's.
    let text = std::fs::read_to_string(&csv).unwrap();
    let table: Vec<Vec<String>> = text.lines().map(csv_fields).collect();
    assert_eq!(table[0][15], "person_delay_s");
    assert_eq!(table.len(), 1 + rows.len() * 2, "{text}");
    for (name, fields) in &rows {
        let runs: Vec<&Vec<String>> = table.iter().filter(|r| r[9] == *name).collect();
        assert_eq!(runs.iter().map(|r| r[10].as_str()).collect::<Vec<_>>(), ["1000", "1001"], "{name}");
        assert!(runs.iter().all(|r| r.len() == 20 && r[0] == lines[0] && r[2] == "tee" && r[5] == "0.5"));
        let pd = runs.iter().map(|r| r[15].parse::<f64>().unwrap()).sum::<f64>() / 2.0;
        assert_eq!(format!("{pd:.1}"), fields[4], "{name}");
        assert_eq!(runs[0][18], "0", "{name}: overlaps");
        let learned = matches!(*name, "AI: Deep RL (Double DQN)" | "AI: Hybrid (RL-tuned predictive)");
        assert_eq!(runs[0][19].starts_with("built-in "), learned, "{name}: {}", runs[0][19]);
    }
}

#[test]
fn bench_says_which_weights_file_each_learned_controller_ran() {
    // A Deep RL file in the working directory comes before the built-in copy.
    let dir = scratch("bench_local_weights");
    std::fs::create_dir_all(dir.join("weights")).unwrap();
    let shipped = std::fs::read_to_string("weights/dqn_tee.json").unwrap();
    let mut v: serde_json::Value = serde_json::from_str(&shipped).unwrap();
    v["episodes"] = (v["episodes"].as_u64().unwrap() + 1).into();
    let local = v.to_string();
    std::fs::write(dir.join("weights").join("dqn_tee.json"), &local).unwrap();
    let out = bench(&dir, &["tee", "0.2", "1"]);
    let path = Path::new("weights").join("dqn_tee.json");
    let used = format!(
        "weights: DeepRL {} (fnv1a {:016x}); Hybrid {}",
        path.display(),
        weights::fingerprint(&local),
        built_in("hybrid_tee.json")
    );
    assert!(out.lines().any(|l| l == used), "{out}");
}

/// A directory can't be written as a file, and `--csv` needs a file name. Either way
/// `bench` stops with the reason before it runs or prints anything.
#[test]
fn bench_stops_when_it_cannot_write_the_csv() {
    let dir = scratch("bench_no_csv");
    for (args, reason, code) in [
        (vec!["tee", "0.2", "1", "--csv", "."], "bench: can't write .: ", 1),
        (vec!["tee", "--csv"], "bench: --csv needs a file name", 2),
    ] {
        let out = Command::new(env!("CARGO_BIN_EXE_bench")).args(&args).current_dir(&dir).output().unwrap();
        assert_eq!(out.status.code(), Some(code), "bench {args:?}");
        let said = String::from_utf8_lossy(&out.stderr);
        assert!(said.starts_with(reason), "bench {args:?}: {said}");
        assert!(out.stdout.is_empty(), "bench {args:?}: {}", String::from_utf8_lossy(&out.stdout));
    }
    assert_eq!(std::fs::read_dir(&dir).unwrap().count(), 0, "bench left a file behind");
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

#[test]
fn train_test_scores_the_saved_weights_and_writes_nothing() {
    let dir = scratch("train_test");
    let out = run(env!("CARGO_BIN_EXE_train"), &dir, &["test", "tee", "150", "hybrid"], &[]);
    assert!(out.starts_with(&format!("intersection-lab {} (", env!("CARGO_PKG_VERSION"))), "{out}");
    for said in [
        "[tee:hybrid] TEST person delay / pedestrian wait (s): fixed ".to_string(),
        format!("[tee:hybrid] weights: {}", built_in("hybrid_tee.json")),
        "[tee:hybrid] TEST person delay against predictive, mean difference with 95% interval over 24 \
         scenarios (s): fixed "
            .to_string(),
        "[tee:hybrid] TEST agent against the static tuning (s): ".to_string(),
    ] {
        assert!(out.contains(&said), "no {said:?} in:\n{out}");
    }
    assert!(!out.contains("[tee:dqn]") && !out.contains(" ep "), "{out}");
    // No weights folder here, and nothing in the per-user data directory either.
    assert_eq!(std::fs::read_dir(&dir).unwrap().count(), 0, "train test wrote files");
}
