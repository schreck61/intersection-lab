# Contributing to Intersection Lab

Thanks for your interest in improving Intersection Lab. Bug reports, realism fixes,
new controllers or experiments, and documentation improvements are all welcome.

By taking part, you agree to follow the [Code of Conduct](CODE_OF_CONDUCT.md). To report a
security vulnerability, follow the [security policy](SECURITY.md) instead of opening a
public issue. How decisions are made and who does what is described in
[GOVERNANCE.md](GOVERNANCE.md).

## Before you start

* **Bugs and small fixes:** open a pull request directly, or an issue first if you'd like
  to discuss it.
* **Anything larger:** open an issue describing the change before writing much code. That
  includes new controllers, changes to vehicle, pedestrian or signal behavior, new
  dependencies, and anything that would change the published results. Agreeing on the
  approach first saves rework. Check [ROADMAP.md](ROADMAP.md) too: it lists what's
  planned and what isn't.
* **Simulation realism:** if something behaves unrealistically, say what real traffic
  would do and, where you can, cite a source (for example the *Highway Capacity Manual*,
  the *MUTCD* or a published study).

## Setting up

You need the latest stable [Rust](https://rustup.rs/) (the crate uses the 2024 edition).

```bash
cargo run --release          # the interactive app
cargo test --release         # regression tests
```

The README's [Build from source](README.md#build-from-source) section lists the
benchmark, training and example commands. To work on the browser version, add the
WebAssembly target and use [Trunk](https://trunk-rs.github.io/trunk/):

```bash
rustup target add wasm32-unknown-unknown
trunk serve --release --open
```

## Checks your change must pass

CI runs these on every pull request. Running them locally first saves a round trip:

```bash
cargo build --locked --release --all-targets
cargo test --locked --release
cargo clippy --locked --release --all-targets -- -D warnings
cargo clippy --locked --release --target wasm32-unknown-unknown --bin intersection-lab -- -D warnings
cargo deny --locked check
```

If you add or update a dependency, also regenerate the third-party license list with
`packaging/licenses.sh` (it needs `cargo-about` 0.9.1). CI fails if
[THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md) is out of date.

CI also checks that every commit in the pull request is
[signed off](#sign-off-your-commits).

CI also fuzzes the loading of weights files for a minute, since the app reads them from
disk (see [fuzz/](fuzz/)). If you change how weights are loaded or used, run it locally
too. It needs nightly Rust and
[cargo-fuzz](https://rust-fuzz.github.io/book/cargo-fuzz.html):

```bash
cargo install cargo-fuzz --locked
cp Cargo.lock fuzz/
mkdir -p fuzz/corpus/weights
cargo +nightly fuzz run weights fuzz/corpus/weights fuzz/seeds/weights -- -max_total_time=60
```

An input that crashes is saved under `fuzz/artifacts/weights/`. To reproduce the crash,
pass that file's path in place of the two folders.

## Guidelines

* **Keep runs deterministic.** The same layout, settings and seed must give the same
  result, which the side-by-side view, the benchmarks and training all depend on. Draw
  random numbers from the simulation's own generator (`src/sim/rng.rs`), never from the
  clock or another source.
* **Safety comes first.** The regression tests require zero vehicle overlaps, and most
  arrivals served, on every layout, control mode and controller. A change that
  introduces an overlap isn't ready, however much it helps delay.
* **Add tests with your change.** New functionality, such as a controller, a safety rule,
  a change to the reward or new simulation behavior, comes with automated tests that
  check it, and a bug fix comes with a test that fails without the fix. Put tests that
  use the public API in [tests/](tests/) (see
  [tests/pedestrian_rule.rs](tests/pedestrian_rule.rs) for an example) and narrower ones
  in a `#[cfg(test)]` module next to the code. Use fixed seeds so tests are
  deterministic, and keep them fast enough to run on every pull request. Changes to
  documentation, UI text or styling don't need tests.
* **Report results with your change.** If it changes how traffic behaves, include before
  and after numbers from `cargo run --release --bin bench all 20 6` (and the corridor or
  sensor study where relevant), and update the tables in the README and
  [docs/insights.md](docs/insights.md) to match. Results from a single seed are noisy;
  use several.
* **Retrained weights:** say which command produced them (for example
  `cargo run --release --bin train all 800 900 dqn`) and include the held-out `TEST` lines
  that `train` prints, which compare the new networks with the baselines. Training is
  deterministic for a given seed and code, so others can reproduce them.
* **Few dependencies.** The project builds for desktop and the browser from one codebase
  and implements its own neural network and simulation, so new crates need a good reason.
  They must use a license allowed in [deny.toml](deny.toml).
* **Style:** match the surrounding code; the project doesn't run `rustfmt`, so please
  don't reformat code you aren't changing. New source files start with the same
  copyright and `SPDX-License-Identifier: MIT OR Apache-2.0` header as the others.
* **Spelling:** use American spelling in code, comments, documentation and UI text.
* **Pull requests:** keep each one focused on a single change, explain what it does and
  why, and note anything you couldn't test (for example a platform you don't have).

## Sign off your commits

Every commit must be signed off under the
[Developer Certificate of Origin](https://developercertificate.org/) (DCO) 1.1. Signing
off certifies that you wrote the change, or otherwise have the right to submit it under
the project's licenses. Add the sign-off with `git commit -s`, which ends the commit
message with a line like this:

```
Signed-off-by: Your Name <you@example.com>
```

The name and email must match the commit's author. The *DCO* check fails a pull request
that has a commit without a sign-off. To add sign-offs to commits you've already made,
rebase over them with `--signoff` and force-push. For the last three commits, for
example:

```bash
git rebase --signoff HEAD~3
git push --force-with-lease
```

## License

Unless you explicitly state otherwise, any contribution intentionally submitted for
inclusion in this project by you, as defined in the Apache-2.0 license, shall be dual
licensed under the [MIT license](LICENSE-MIT) and the
[Apache License, Version 2.0](LICENSE-APACHE), without any additional terms or
conditions.
