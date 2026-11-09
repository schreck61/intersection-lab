# Security policy

## Supported versions

Security fixes go into the latest release and the browser version, which is rebuilt from
`main`. Older releases don't receive fixes; please update to the
[latest release](https://github.com/schreck61/intersection-lab/releases/latest).

## What to expect

Intersection Lab is a simulator that runs on your own computer. What you can expect from
it, and what it doesn't protect against:

* **No network access.** The desktop app makes no network connections. The browser
  version is served over HTTPS from GitHub Pages and makes no requests of its own once
  it has loaded.
* **No extra privileges.** Once installed, the app runs with your ordinary user rights
  and never asks for more.
* **Only its own files.** The app reads trained-weights files: from a `weights` folder in
  the folder it's started from, then from its folder in your user data directory, and
  otherwise from the copies built into it. It writes them only when you save weights.
  The `train` tool writes the weights it trains to a `weights` folder in the folder it's
  run from, `bench` writes no files, and the app's screenshot mode (`TS_SHOT`) writes
  only the image you name.
* **Files are checked before use.** A weights file is parsed into typed data and checked
  against what its controller accepts (layer sizes, connections between layers, finite
  values). A file that fails is rejected rather than run, and CI fuzzes this loader on
  every pull request. A file that passes can still make its controller manage traffic
  badly, so use weights only from sources you trust.
* **No code from data.** The app has no plugins, scripting or other way to run code
  from the files it reads, and its own code is safe Rust with no `unsafe` blocks.
* **Verifiable releases.** Every release file is signed and comes with a
  build-provenance attestation (see [Verifying downloads](#verifying-downloads)).
  Packages from anywhere else aren't covered.

It doesn't protect against someone who can already change files in your user account,
who could replace the app or its weights, and a very large weights file is read whole
into memory before it's checked, so it can make the app slow or run out of memory.
[docs/assurance-case.md](docs/assurance-case.md) explains why these expectations hold.

## Reporting a vulnerability

Please don't report security vulnerabilities in public issues or pull requests.

Report them privately through GitHub instead:
[open a private vulnerability report](https://github.com/schreck61/intersection-lab/security/advisories/new)
(GitHub's guide:
[privately reporting a security vulnerability](https://docs.github.com/en/code-security/security-advisories/guidance-on-reporting-and-writing-information-about-vulnerabilities/privately-reporting-a-security-vulnerability)).
If you can't use GitHub, email the maintainer at schreck@gmail.com.

Helpful details include:

* the version (shown in the app's *About* window) and your platform, or whether it's the
  browser version;
* what an attacker could do, and the steps or files needed to reproduce it;
* any fix or mitigation you'd suggest.

This is a one-maintainer project. The maintainer aims to acknowledge reports within a
week, keep you updated while a fix is prepared, and credit you in the advisory and
release notes unless you'd rather stay anonymous. Please allow time for a fixed release
before disclosing details publicly.

## Scope

In scope:

* the desktop app and the browser version, including how they read and write files such
  as trained weights;
* the release packages and how they're built, signed and published (the GitHub Actions
  workflows, code signing, checksums and build-provenance attestations);
* vulnerable dependencies that actually affect this project. CI already checks
  dependencies against the RustSec advisory database with `cargo deny`, and
  `deny.toml` and `osv-scanner.toml` list the advisories that don't affect it, each
  with the reason. Please report problems in a dependency itself to that project.

Out of scope: unrealistic simulation behavior or results. Those are welcome as ordinary
[issues](https://github.com/schreck61/intersection-lab/issues).

## Verifying downloads

Every release includes `SHA256SUMS.txt` and a signed build-provenance attestation for
each file. To check that a download was built by this repository's release workflow:

```bash
gh attestation verify <downloaded-file> --repo schreck61/intersection-lab
```

Each release also includes the signed attestation as `IntersectionLab-<version>.sigstore.json`;
pass it with `--bundle` to verify without fetching it from GitHub. The release's
`IntersectionLab-<version>.intoto.jsonl` is an identical copy, named the way supply-chain
scanners such as the OpenSSF Scorecard expect provenance to be.

The macOS app is also signed with a Developer ID and notarized, and the Windows
executable and installer are signed with Azure Artifact Signing. See
[packaging/README.md](packaging/README.md) for details.
