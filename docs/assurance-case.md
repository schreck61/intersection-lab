# Security assurance case

This document argues that Intersection Lab meets the security expectations listed in
[SECURITY.md](../SECURITY.md#what-to-expect). It describes the threat model, identifies
the trust boundaries, and explains how the design follows secure design principles and
how the implementation counters common weaknesses. It's updated whenever any of these
change.

## What needs protecting

* **Users' computers.** Running the app, or opening data with it, must not let anyone
  run code, read or change other files, or reach the network.
* **What users download.** A release package must be exactly what this repository's
  release workflow built from its source.
* **The repository and its pipeline.** Only changes that pass the required checks reach
  `main`, and only the release workflows, running for a version tag, can sign packages
  or publish the crate.

## Threat model

The app is a local, single-user simulator. It has no accounts, no network access and no
server side, so the threats that matter are these:

| Threat | Example | Main defenses |
|---|---|---|
| Malicious data file | A crafted weights file, received from someone or placed in a `weights` folder | Typed parsing, checks on shape and values, fuzzing, safe Rust |
| Tampered download | A modified package from a mirror or a look-alike site | Signed build-provenance attestations, checksums, code signing on macOS and Windows, HTTPS |
| Vulnerable or malicious dependency | A crate or GitHub Action with a known flaw, or a hijacked release of one | Few dependencies, locked versions, `cargo deny` with the RustSec database, actions pinned to commit hashes, Dependabot |
| Malicious contribution | A pull request that hides harmful code or attacks CI | Required checks, the maintainer's review of outside contributions, read-only tokens for pull requests from forks, CodeQL for workflows |
| Stolen release credentials | Misuse of signing or publishing access | Short-lived credentials (OIDC, Sigstore, crates.io Trusted Publishing) and secrets limited to version-tag builds |

Out of scope: someone who already controls the user's account or computer, who could
replace the app itself, and the security of the platforms the project relies on (the
operating system or browser, GitHub, crates.io, Sigstore, and Apple's and Microsoft's
signing services).

## Trust boundaries

1. **Files on disk → the app.** Weights files are untrusted input, whichever folder they
   come from: anyone who can write to those folders, or who sends a user a file, decides
   their contents. The desktop app also reads the fonts installed on the system and, on
   Linux, desktop settings such as the mouse-pointer theme, a user's own included, to
   draw its window: code in its dependencies parses them (boundary 6; see *Remaining
   risks*). The executable and the weights built into it are trusted, since the release
   signatures cover them.
2. **The person running it → the app.** Command-line arguments and environment
   variables (the `bench` and `train` settings, and the app's `TS_` settings such as the
   screenshot mode) come from the person running the program, who already has the same
   rights as the program. They're trusted, but still parsed as typed values.
3. **The browser sandbox.** The browser version runs inside the browser's sandbox, with
   no file access and no network requests after loading, and GitHub Pages serves it
   over HTTPS.
4. **Contributors → `main`.** Anyone can open a pull request. Only the maintainer can
   merge one, and only after the required checks pass. Workflows triggered by pull
   requests from forks get a read-only token and no secrets, and no workflow uses
   `pull_request_target`.
5. **CI → signing and publishing.** The macOS signing settings are in the `release`
   environment and crates.io publishing runs in the `crates-io` environment, both
   limited to `v*` tags. Windows signing, build-provenance attestations and crates.io
   publishing use short-lived OIDC credentials, so no long-lived key or token for them is
   stored anywhere.
6. **The project → its dependencies.** Crates come only from crates.io (deny.toml
   rejects any other source), at the versions locked in Cargo.lock, and every GitHub
   Action is pinned to a commit hash.

## Secure design principles

The design follows the principles set out by Saltzer and Schroeder:

* **Economy of mechanism.** No network code, no plugins or scripting, no `unsafe` code,
  and four direct dependencies (eframe, egui, serde and serde_json, plus two small ones
  for the browser build). The simulation and the neural network are the project's own
  code.
* **Fail-safe defaults.** A network that doesn't fit its controller is never run. A
  weights file that doesn't load is skipped for the next copy, ending with the copy
  built into the executable, and a Hybrid file whose network doesn't fit loads without
  the network and runs only its saved base tuning. If no copy loads, Deep RL starts from
  a fresh network and the Hybrid runs the planner's default tuning (see
  [section 8.7 of the architecture document](architecture.md#87-controlweightsrs-loading-and-saving)).
  Every workflow's token is read-only unless a job needs more, and packages are signed
  only on version-tag builds.
* **Complete mediation.** Every weights file goes through the same loader and the same
  check (`SavedModel::fits`, which uses `Mlp::fits`) before it's used, whichever folder
  it came from.
* **Open design.** The source, the workflows and the release process are public, and
  nothing depends on keeping them secret.
* **Separation of privilege.** Changes reach `main` only through pull requests whose
  checks pass; the signing and publishing credentials are available only to workflows
  running for version tags, which only the maintainer can push; and a release reaches
  users only when the maintainer publishes its draft.
* **Least privilege.** Each workflow job gets only the token permissions it needs, and
  checkouts don't keep credentials. The app runs with the user's ordinary rights.
* **Psychological acceptability.** macOS and Windows check the platform signatures
  automatically, and checking a download's provenance takes one command
  (`gh attestation verify`).

## Common weaknesses countered

| Weakness | How it's countered |
|---|---|
| Memory-safety errors such as out-of-bounds reads and writes or use after free (CWE-787, CWE-125, CWE-416) | The project's code is safe Rust with no `unsafe` blocks, so bounds are checked and memory is managed by the compiler. The fuzzer runs with AddressSanitizer. |
| Improper input validation (CWE-20) | Weights files are parsed with serde into typed structures, then checked for layer sizes, connections and finite values. tests/weights_files.rs and the fuzz target in fuzz/ cover the loader. |
| Integer overflow (CWE-190) | Sizes read from files are multiplied with checked arithmetic, and the fuzzer runs with overflow checks. |
| Deserialization of untrusted data (CWE-502) | Files deserialize into plain data structures with serde_json; nothing in a file selects a type or code to run. |
| Uncontrolled resource consumption (CWE-400) | Malformed networks are rejected before they run, and network outputs that overflow to NaN are sorted safely (`total_cmp`). A very large file is still read whole into memory (see *Remaining risks*). |
| Injection: OS commands, SQL, cross-site scripting, path traversal (CWE-78, CWE-89, CWE-79, CWE-22) | Not applicable by construction: the app's own code starts no programs, has no database and generates no HTML, and it builds file paths only from fixed names, the platform's standard directories and paths the person running it supplies. The window libraries it depends on do start a few programs, and nothing from a weights file is part of what they run: the default browser (on Linux, the one the `BROWSER` variable or the desktop's settings name), given the project's fixed address, when the link in the *About* window is clicked, and, on Linux under Wayland when the app draws its own title bar, `gsettings`, `dbus-send` and `fc-match`, found on the `PATH`, to read the desktop's settings and find the title-bar font (`fc-match` is given the font name that `gsettings` reports). |
| Hard-coded or leaked credentials (CWE-798) | The app has no credentials, and the repository holds none: the macOS signing settings are GitHub environment secrets, and everything else signs in with short-lived OIDC credentials. |
| Vulnerable and outdated components (OWASP A06) | `cargo deny` checks every change against the RustSec advisory database and fails on a locked version that has been yanked, Dependabot proposes weekly updates to the dependencies named in `Cargo.toml` and a security update for a locked version that GitHub has an advisory for, when a fixed version exists, and the OpenSSF Scorecard's vulnerability check runs weekly. Advisories that don't apply are recorded with the reason. |
| Untrusted code in the build (CWE-829) | Every GitHub Action is pinned to a commit hash, CodeQL analyzes the workflows, and no workflow runs pull-request code with write access or secrets. |
| Download of code without an integrity check (CWE-494) | Every release package comes with a signed build-provenance attestation and a SHA-256 checksum, the macOS disk image and the app inside it and the Windows executable and installer are also code-signed, and releases are served over HTTPS. |

CodeQL (for Rust and for GitHub Actions) and Clippy, with every warning treated as an
error, also check all code on every pull request.

## Remaining risks

* **One maintainer.** The maintainer's own changes are checked by CI, CodeQL and fuzzing
  rather than by a second reviewer, and the release pipeline trusts the maintainer's
  GitHub account: whoever controlled that account could publish a release.
* **Valid but unhelpful weights.** A weights file that loads can still make its
  controller manage traffic badly. That affects the simulation's results, not the
  computer it runs on.
* **Very large files.** A weights file is read whole into memory before it's checked, so
  an enormous one can make the app slow or exhaust its memory.
* **Installed fonts.** The fonts the desktop app reads (boundary 1) are parsed by code
  in its dependencies that this project neither wrote nor fuzzes: fontique, read-fonts,
  skrifa and harfrust, and on Linux under Wayland also ttf-parser, which is
  unmaintained (see deny.toml). A flaw there could be reached by a font a user was
  persuaded to install. `cargo deny` checks these crates against the RustSec advisory
  database on every change, as it does every dependency, so it catches only flaws that
  have been reported.
* **Platform trust.** The project relies on GitHub, crates.io, Sigstore, Apple and
  Microsoft for hosting, signing and publishing.
