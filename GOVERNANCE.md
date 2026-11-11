# Governance

Intersection Lab is a one-maintainer project. Jim Schreckengast
([schreck61](https://github.com/schreck61)) started it, maintains it and makes the final
decisions about it. Chris Miller ([csm0513](https://github.com/csm0513)) is the backup
maintainer, ready to keep the project going if Jim can't (see [Continuity](#continuity)).
This document describes how decisions are made and who does what.

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
  Windows, test coverage of at least 80% of lines, the browser build, rustfmt and
  Clippy, the crate package, the license and advisory checks, CodeQL, and DCO sign-off.
  Nobody can push to `main` directly.
* **With reasons.** The maintainer reviews contributions and decides whether to merge
  them, and explains the reasons when declining a change or asking for a different
  approach. If you disagree with a decision, say so in the issue or pull request.
* **Releases** use [Semantic Versioning](https://semver.org/) and are made by the
  maintainer as described in
  [packaging/README.md](packaging/README.md#making-a-release).

## Roles

| Role | Who | Responsibilities |
|---|---|---|
| Maintainer | Jim Schreckengast ([schreck61](https://github.com/schreck61)) | Sets the project's direction and keeps the [roadmap](ROADMAP.md) current; triages issues; reviews and merges pull requests; keeps CI, dependencies and documentation up to date; makes releases and publishes them on GitHub and crates.io; manages the code-signing and publishing settings; responds to vulnerability reports as [SECURITY.md](SECURITY.md) describes; enforces the [Code of Conduct](CODE_OF_CONDUCT.md) |
| Backup maintainer | Chris Miller ([csm0513](https://github.com/csm0513)) | Keeps the project going if the maintainer can't, as [Continuity](#continuity) describes: answers and closes issues, merges pull requests whose required checks pass, makes releases, and settles the project's long-term future; has no duties otherwise |
| Contributor | Anyone who opens an issue or pull request | Follows [CONTRIBUTING.md](CONTRIBUTING.md), signs off every commit under the [Developer Certificate of Origin](https://developercertificate.org/), and follows the Code of Conduct |
| Security reporter | Anyone who finds a vulnerability | Reports it privately as [SECURITY.md](SECURITY.md) describes, not in a public issue |

## Continuity

If the maintainer dies, is incapacitated or can no longer maintain the project, the
backup maintainer keeps it going. Within a week of learning of it, they answer and close
issues, merge pull requests whose required checks pass, and make releases. They then
settle the project's long-term future: carrying it on, handing it to a new maintainer,
or archiving it.

The backup maintainer already has the access this takes:

* write access to this repository, which covers issues, pull requests and the version
  tags that build and publish releases;
* co-ownership of the [crate on crates.io](https://crates.io/crates/intersection-lab);
* the maintainer's GitHub *successor* designation, under which GitHub can transfer this
  repository to the backup maintainer if the maintainer dies.

No passwords or keys change hands. Every release is built, signed and published by
GitHub Actions rather than on anyone's own computer, and if the code-signing accounts
lapse, the release workflow can be switched to unsigned packages (see
[packaging/README.md](packaging/README.md#making-a-release)). The backup maintainer has
written instructions for each step, and the maintainer and the backup maintainer check
this access once a year.

## Changing this document

If the project gains more maintainers, this document will be updated to say how they
share decisions. Changes to it go through pull requests like any other change.
