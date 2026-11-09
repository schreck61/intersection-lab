# Security policy

## Supported versions

Security fixes go into the latest release and the browser version, which is rebuilt from
`main`. Older releases don't receive fixes; please update to the
[latest release](https://github.com/schreck61/intersection-lab/releases/latest).

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
