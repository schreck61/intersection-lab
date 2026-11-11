// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Weights files are read from disk, where they can be damaged or edited by hand. The
//! loaders must reject a network that doesn't fit its controller, and nothing an
//! accepted network computes may crash the app. What the app and the trainer save must
//! load again, and a copy that doesn't load mustn't hide one that does.

use intersection_lab::control::hybrid::{Hybrid, N_ACTIONS};
use intersection_lab::control::rl::{RlController, n_inputs};
use intersection_lab::control::{Controller, CtrlKind, weights};
use intersection_lab::sim::network::{LayoutKind, Network};
use intersection_lab::sim::{ControlMode, Sim};
use serde_json::{Value, json};
use std::path::PathBuf;

const DQN_TEE: &str = include_str!("../weights/dqn_tee.json");
const HYBRID_TEE: &str = include_str!("../weights/hybrid_tee.json");

/// An empty directory for this test's files.
fn scratch(name: &str) -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join(name);
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// What the app says about where the hybrid's tuning comes from.
fn status(mut h: Hybrid) -> String {
    h.loaded_from = Some("a weights file".into());
    h.brain().notes.join("\n")
}

/// The shipped tee-intersection network with `edit` applied to its layers.
fn edited(edit: impl FnOnce(&mut Vec<Value>)) -> String {
    let mut v: Value = serde_json::from_str(DQN_TEE).unwrap();
    edit(v["net"]["layers"].as_array_mut().unwrap());
    v.to_string()
}

/// A hybrid weights file for the tee intersection whose network has `layers`, declaring
/// the sizes the controller expects.
fn hybrid_file(net: &Network, layers: Value) -> String {
    let model = json!({ "layout": "tee", "n_in": n_inputs(net), "n_out": N_ACTIONS, "episodes": 1, "net": { "layers": layers } });
    json!({ "base_action": 10, "base_score": 0.0, "model": model }).to_string()
}

/// Layers with finite weights whose sums overflow f32: both hidden units become
/// infinite, so every output is inf - inf = NaN.
fn overflowing_layers(n_in: usize, n_out: usize) -> Value {
    json!([
        { "n_in": n_in, "n_out": 2, "w": vec![3e38; 2 * n_in], "b": [3e38, 3e38] },
        { "n_in": 2, "n_out": n_out, "w": ([1.0, -1.0].repeat(n_out)), "b": vec![0.0; n_out] },
    ])
}

/// Twenty seconds of the tee intersection under `ctrl`, then the explanation the app
/// shows beside the simulation.
fn run(ctrl: Box<dyn Controller>, kind: CtrlKind) {
    let mut sim = Sim::new(LayoutKind::Tee, ControlMode::Signal, CtrlKind::Fixed, 1);
    sim.controller = ctrl;
    sim.ctrl_kind = kind;
    sim.run_for(20.0);
    sim.controller.brain();
}

#[test]
fn shipped_weights_load() {
    for kind in [LayoutKind::Arterial4, LayoutKind::MajorMinor4, LayoutKind::Town4, LayoutKind::Tee] {
        let net = Network::build(kind);
        let key = kind.key();
        let dqn = std::fs::read_to_string(format!("weights/dqn_{key}.json")).unwrap();
        assert!(RlController::from_saved(&net, &dqn).is_some(), "dqn_{key}.json was rejected");
        let hybrid = std::fs::read_to_string(format!("weights/hybrid_{key}.json")).unwrap();
        let with_model = serde_json::from_str::<Value>(&hybrid).unwrap()["model"].is_object();
        assert_eq!(Hybrid::from_saved(&net, &hybrid).unwrap().has_model, with_model, "hybrid_{key}.json");
    }
}

#[test]
fn misshapen_networks_are_rejected() {
    let net = Network::build(LayoutKind::Tee);
    // Each file still declares the 45 inputs and 4 outputs the controller expects.
    let cases = [
        ("no layers", edited(|l| l.clear())),
        ("a layer with no inputs", edited(|l| l[0]["n_in"] = json!(0))),
        ("too few weights", edited(|l| _ = l[0]["w"].as_array_mut().unwrap().pop())),
        ("too few biases", edited(|l| _ = l[1]["b"].as_array_mut().unwrap().pop())),
        ("layers that don't connect", edited(|l| {
            l[1]["n_in"] = json!(64);
            l[1]["w"].as_array_mut().unwrap().truncate(64 * 4);
        })),
        ("three outputs", edited(|l| {
            l[1]["n_out"] = json!(3);
            l[1]["w"].as_array_mut().unwrap().truncate(128 * 3);
            l[1]["b"].as_array_mut().unwrap().pop();
        })),
        ("a weight too large for f32", edited(|l| l[0]["w"][0] = json!(1e39))),
    ];
    for (what, txt) in &cases {
        assert!(RlController::from_saved(&net, txt).is_none(), "accepted a network with {what}");
    }
}

#[test]
fn hybrid_without_a_fitting_network_keeps_its_base_tuning() {
    let net = Network::build(LayoutKind::Tee);
    let h = Hybrid::from_saved(&net, &hybrid_file(&net, json!([]))).unwrap();
    assert!(!h.has_model);
    assert_eq!(h.base_action, 10);
    // The app says the network was left out, not that training found none worth keeping.
    let notes = status(h);
    assert!(notes.contains("network doesn't fit"), "{notes}");
    assert!(!notes.contains("no network beat"), "{notes}");

    let no_network = json!({ "base_action": 10, "base_score": 0.0, "model": null }).to_string();
    let notes = status(Hybrid::from_saved(&net, &no_network).unwrap());
    assert!(notes.contains("no network beat the best static tuning"), "{notes}");
}

#[test]
fn hybrid_files_with_a_non_finite_score_load() {
    let net = Network::build(LayoutKind::Tee);
    let dir = scratch("non_finite_score");
    let mut h = Hybrid::new(&net, 1);
    h.base_action = 7;
    // The app knows no score; training has none when no tuning meets its pedestrian limit.
    for (i, score) in [None, Some(f64::NAN), Some(f64::INFINITY), Some(f64::NEG_INFINITY)].into_iter().enumerate() {
        for with_model in [false, true] {
            let path = dir.join(format!("hybrid_{i}_{with_model}.json"));
            h.save(&path, score, with_model).unwrap();
            let saved = std::fs::read_to_string(&path).unwrap();
            let back = Hybrid::from_saved(&net, &saved).unwrap_or_else(|| panic!("saved with score {score:?}, doesn't load: {saved:.80}"));
            assert_eq!((back.base_action, back.has_model), (7, with_model));
        }
    }
    // What version 0.5.1 wrote for such a score.
    let old = json!({ "base_action": 7, "base_score": null, "model": null }).to_string();
    assert_eq!(Hybrid::from_saved(&net, &old).unwrap().base_action, 7);
}

#[test]
fn a_weights_file_that_does_not_load_is_skipped() {
    let net = Network::build(LayoutKind::Tee);
    let load = |dirs: &[PathBuf]| weights::load_from(dirs, "hybrid_tee.json", |txt| Hybrid::from_saved(&net, txt)).unwrap();
    let (bad, good) = (scratch("skip_bad"), scratch("skip_good"));
    std::fs::write(bad.join("hybrid_tee.json"), r#"{"base_action": 3, "base_sc"#).unwrap();
    let mut h = Hybrid::new(&net, 1);
    h.base_action = 5;
    h.save(&good.join("hybrid_tee.json"), None, false).unwrap();

    let (h, source) = load(&[bad.clone(), good.clone()]);
    assert_eq!(h.base_action, 5);
    assert_eq!(source, good.join("hybrid_tee.json").display().to_string());
    // With no copy on disk that loads, the built-in one is used.
    let (h, source) = load(&[bad]);
    assert_eq!(source, "built-in hybrid_tee.json");
    assert_eq!(h.base_action, Hybrid::from_saved(&net, HYBRID_TEE).unwrap().base_action);
}

#[test]
fn networks_that_overflow_do_not_crash() {
    let net = Network::build(LayoutKind::Tee);
    let n = n_inputs(&net);
    let h = Hybrid::from_saved(&net, &hybrid_file(&net, overflowing_layers(n, N_ACTIONS))).unwrap();
    assert!(h.has_model);
    run(Box::new(h), CtrlKind::Hybrid);

    let dqn = json!({ "layout": "tee", "n_in": n, "n_out": 4, "episodes": 1, "net": { "layers": overflowing_layers(n, 4) } });
    let c = RlController::from_saved(&net, &dqn.to_string()).unwrap();
    run(Box::new(c), CtrlKind::DeepRl);
}
