# Governance

Intersection Lab is a one-maintainer project. Jim Schreckengast
([schreck61](https://github.com/schreck61)) started it, maintains it and makes the final
decisions about it. This document describes how those decisions are made and who does
what.

## How decisions are made

* **In public.** Proposals and discussion happen in
  [issues](https://github.com/schreck61/intersection-lab/issues) and pull requests.
  Larger changes, such as new controllers, changes to vehicle, pedestrian or signal
  behavior, new dependencies, or anything that would change the published results,
  start with an issue, as [CONTRIBUTING.md](CONTRIBUTING.md#before-you-start) describes.
* **By evidence.** Questions about realism are settled by what real traffic does, citing
  sources such as the *Highway Capacity Manual*, the *MUTCD* or published studies where
  possible, and questions about performance by benchmark results across several seeds.
  A change that lets vehicles collide isn't accepted, however much it helps delay.
* **Through pull requests.** Every change to `main`, including the maintainer's own,
  goes through a pull request whose required checks pass: the tests on Linux, macOS and
  Windows, the browser build, Clippy, the crate package, the license and advisory
  checks, CodeQL, and DCO sign-off. Nobody can push to `main` directly.
* **With reasons.** The maintainer reviews contributions and decides whether to merge
  them, and explains the reasons when declining a change or asking for a different
  approach. If you disagree with a decision, say so in the issue or pull request.
* **Releases** use [Semantic Versioning](https://semver.org/) and are made by the
  maintainer as described in
  [packaging/README.md](packaging/README.md#making-a-release).

## Roles

| Role | Who | Responsibilities |
|---|---|---|
| Maintainer | Jim Schreckengast ([schreck61](https://github.com/schreck61)) | Sets the project's direction; triages issues; reviews and merges pull requests; keeps CI, dependencies and documentation up to date; makes releases and publishes them on GitHub and crates.io; manages the code-signing and publishing settings; responds to vulnerability reports as [SECURITY.md](SECURITY.md) describes; enforces the [Code of Conduct](CODE_OF_CONDUCT.md) |
| Contributor | Anyone who opens an issue or pull request | Follows [CONTRIBUTING.md](CONTRIBUTING.md), signs off every commit under the [Developer Certificate of Origin](https://developercertificate.org/), and follows the Code of Conduct |
| Security reporter | Anyone who finds a vulnerability | Reports it privately as [SECURITY.md](SECURITY.md) describes, not in a public issue |

## Changing this document

If the project gains more maintainers, this document will be updated to say how they
share decisions. Changes to it go through pull requests like any other change.
