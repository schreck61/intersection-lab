## What and why

<!-- What does this change, and why? Link any related issue, e.g. "Fixes #12". -->

## Results

<!-- If traffic behavior changes, give before/after numbers, e.g. from
`cargo run --release --bin bench all 20 6`. For retrained weights, give the
train command and its TEST lines. Otherwise, delete this section. -->

## Checklist

- [ ] `cargo test --locked --release` passes (zero vehicle overlaps)
- [ ] `cargo clippy --locked --release --all-targets -- -D warnings` is clean (and for `wasm32-unknown-unknown` if the app changed)
- [ ] New functionality has tests, and a bug fix has a test that fails without it (see [CONTRIBUTING.md](https://github.com/schreck61/intersection-lab/blob/main/CONTRIBUTING.md#guidelines))
- [ ] Runs stay deterministic for a given seed
- [ ] README, docs/insights.md and in-app text are updated where results or behavior changed
- [ ] New dependencies, if any, are explained and `THIRD_PARTY_LICENSES.md` is regenerated
- [ ] I've noted anything I couldn't test (for example a platform I don't have)
- [ ] Every commit is signed off (`git commit -s`) under the [Developer Certificate of Origin](https://developercertificate.org/)

By submitting this pull request, I agree that my contribution is dual licensed under MIT OR Apache-2.0, as described in [CONTRIBUTING.md](https://github.com/schreck61/intersection-lab/blob/main/CONTRIBUTING.md#license).
