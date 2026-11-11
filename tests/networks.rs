// Copyright (c) 2026 Jim Schreckengast
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The neural network behind the learning controllers: backpropagation agrees with the
//! gradient measured numerically, Adam fits a line, the shape check turns away every
//! malformed network, and Double DQN learns which action pays in which state.

use intersection_lab::rl::dqn::{Dqn, SavedModel, Transition, argmax, argmax_masked};
use intersection_lab::rl::nn::{Adam, Grads, Mlp};
use intersection_lab::sim::rng::Rng;

/// A weighted sum of the network's outputs: its gradient with respect to the outputs
/// is `c`.
fn weighted_output(net: &Mlp, x: &[f32], c: &[f32]) -> f32 {
    net.forward(x).iter().zip(c).map(|(y, c)| y * c).sum()
}

#[test]
fn backpropagation_matches_the_numerical_gradient() {
    let net = Mlp::new(&[4, 6, 5, 3], &mut Rng::new(7));
    assert_eq!((net.n_in(), net.n_out()), (4, 3));
    let x = [0.5, -1.0, 0.25, 2.0];
    let c = [1.0, -0.5, 2.0];
    let mut grads = Grads::zeros(&net);
    net.backward(&net.forward_all(&x), &c, &mut grads);
    // The network is piecewise linear in each parameter, so a central difference is
    // exact up to rounding unless a ReLU switches within the step.
    let h = 1e-2;
    let numerical = |edit: &dyn Fn(&mut Mlp, f32)| {
        let (mut up, mut down) = (net.clone(), net.clone());
        edit(&mut up, h);
        edit(&mut down, -h);
        (weighted_output(&up, &x, &c) - weighted_output(&down, &x, &c)) / (2.0 * h)
    };
    for li in 0..net.layers.len() {
        for (i, &g) in grads.w[li].iter().enumerate() {
            let n = numerical(&|m, d| m.layers[li].w[i] += d);
            assert!(
                (n - g).abs() < 1e-3 * (1.0 + g.abs()),
                "layer {li} weight {i}: backprop {g}, numerical {n}"
            );
        }
        for (i, &g) in grads.b[li].iter().enumerate() {
            let n = numerical(&|m, d| m.layers[li].b[i] += d);
            assert!(
                (n - g).abs() < 1e-3 * (1.0 + g.abs()),
                "layer {li} bias {i}: backprop {g}, numerical {n}"
            );
        }
    }
    assert!(grads.w.iter().flatten().any(|g| *g != 0.0), "some weight has a gradient");

    // Gradients add up over samples until cleared.
    let once = grads.w.clone();
    net.backward(&net.forward_all(&x), &c, &mut grads);
    for (a, b) in grads.w.iter().flatten().zip(once.iter().flatten()) {
        assert_eq!(*a, 2.0 * b);
    }
    grads.clear();
    assert!(grads.w.iter().chain(&grads.b).flatten().all(|g| *g == 0.0));
}

#[test]
fn adam_fits_a_line() {
    // A single layer has no ReLU: it learns y = 2x - 1 exactly.
    let mut net = Mlp::new(&[1, 1], &mut Rng::new(3));
    let mut adam = Adam::new(&net, 0.02);
    let mut grads = Grads::zeros(&net);
    let xs: Vec<f32> = (0..8).map(|k| k as f32 / 4.0 - 1.0).collect();
    for _ in 0..3000 {
        grads.clear();
        for &x in &xs {
            let acts = net.forward_all(&[x]);
            let err = acts[1][0] - (2.0 * x - 1.0);
            net.backward(&acts, &[err], &mut grads);
        }
        adam.apply(&mut net, &grads, 1.0 / xs.len() as f32);
    }
    let l = &net.layers[0];
    assert!(
        (l.w[0] - 2.0).abs() < 0.01 && (l.b[0] + 1.0).abs() < 0.01,
        "learned y = {}x + {}",
        l.w[0],
        l.b[0]
    );
}

/// A change that leaves a network malformed.
type Damage = fn(&mut Mlp);

#[test]
fn the_shape_check_rejects_every_malformed_network() {
    let good = Mlp::new(&[4, 6, 3], &mut Rng::new(1));
    assert!(good.fits(4, 3));
    assert!(!good.fits(5, 3), "wrong number of inputs");
    assert!(!good.fits(4, 2), "wrong number of outputs");
    let broken: [(&str, Damage); 7] = [
        ("a weight missing", |m| {
            m.layers[0].w.pop();
        }),
        ("a bias missing", |m| {
            m.layers[1].b.pop();
        }),
        ("layers that don't chain", |m| m.layers[1].n_in = 7),
        ("a layer with no outputs", |m| m.layers[1].n_out = 0),
        ("a weight that isn't a number", |m| m.layers[0].w[3] = f32::NAN),
        ("an infinite bias", |m| m.layers[1].b[0] = f32::INFINITY),
        ("no layers at all", |m| m.layers.clear()),
    ];
    for (what, edit) in broken {
        let mut m = good.clone();
        edit(&mut m);
        assert!(!m.fits(4, 3), "accepted a network with {what}");
    }
    let mut huge = good.clone();
    huge.layers[0].n_in = usize::MAX;
    assert!(!huge.fits(usize::MAX, 3), "accepted sizes whose product overflows");

    let saved =
        |n_in: usize| SavedModel { layout: "tee".into(), n_in, n_out: 3, episodes: 0, net: good.clone() };
    assert!(saved(4).fits(4, 3));
    assert!(!saved(4).fits(5, 3), "the controller expects other sizes");
    assert!(!saved(5).fits(5, 3), "the file declares sizes its network doesn't have");
}

#[test]
fn the_target_network_moves_part_way() {
    let mut rng = Rng::new(11);
    let src = Mlp::new(&[3, 4, 2], &mut rng);
    let start = Mlp::new(&[3, 4, 2], &mut rng);
    let mut target = start.clone();
    target.soft_update(&src, 0.25);
    for ((t, a), b) in target.layers.iter().zip(&start.layers).zip(&src.layers) {
        for ((t, a), b) in t.w.iter().chain(&t.b).zip(a.w.iter().chain(&a.b)).zip(b.w.iter().chain(&b.b)) {
            assert!((t - (a + 0.25 * (b - a))).abs() < 1e-6);
        }
    }
}

#[test]
fn argmax_respects_the_mask() {
    assert_eq!(argmax(&[0.1, 0.5, 0.3]), 1);
    assert_eq!(argmax(&[2.0, 2.0]), 0, "ties go to the first");
    assert_eq!(argmax_masked(&[0.1, 0.5, 0.3], &[true, false, true]), 2);
    assert_eq!(argmax_masked(&[0.1, 0.5, 0.3], &[false; 3]), 1, "no allowed action: plain argmax");
    assert_eq!(argmax_masked(&[0.1, 0.5, 0.3], &[false]), 1, "actions past the mask are allowed");
}

/// One step of a two-armed bandit with two states: the action that matches the state
/// pays 1, the other nothing, and the episode ends.
fn bandit(state: usize, action: usize) -> Transition {
    let mut s = vec![0.0; 2];
    s[state] = 1.0;
    let r = if action == state { 1.0 } else { 0.0 };
    Transition { s: s.clone(), a: action, r, s2: s, disc: 0.0, mask2: vec![true; 2] }
}

#[test]
fn double_dqn_learns_which_action_pays() {
    let mut dqn = Dqn::new(2, 2, &[8], 5);
    let mut rng = Rng::new(9);
    for k in 0..1200 {
        assert_eq!(dqn.train_step().is_some(), k >= 1000, "training waits for 1,000 transitions");
        dqn.push(bandit(rng.below(2), rng.below(2)));
    }
    assert_eq!(dqn.replay_len(), 1200);
    dqn.set_lr(1e-2);
    for _ in 0..600 {
        dqn.train_step();
    }
    assert_eq!(dqn.updates, 600 + 200);
    assert_eq!(dqn.losses.len(), 500, "the loss history keeps the latest 500");
    for state in 0..2 {
        let mut s = [0.0; 2];
        s[state] = 1.0;
        assert_eq!(dqn.greedy(&s), state);
        let q = dqn.q(&s);
        assert!((q[state] - 1.0).abs() < 0.1 && q[1 - state].abs() < 0.1, "state {state}: Q = {q:?}");
        // The target network trails the online one but has learned the same thing.
        assert_eq!(argmax(&dqn.target.forward(&s)), state);
    }
}

#[test]
fn the_replay_memory_keeps_the_latest_transitions() {
    let mut dqn = Dqn::new(2, 2, &[4], 1);
    for k in 0..60_010 {
        dqn.push(bandit(k % 2, 0));
    }
    assert_eq!(dqn.replay_len(), 60_000);
}
