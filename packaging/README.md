# Packaging and releases

Intersection Lab ships as signed, installable packages for every major desktop platform.
GitHub Actions builds them all from a version tag ([`.github/workflows/release.yml`](../.github/workflows/release.yml)).

| Platform | Architectures | Packages | Signing |
|---|---|---|---|
| macOS 11+ | Apple silicon + Intel (one universal app) | `.dmg` | Developer ID, hardened runtime, notarized and stapled |
| Windows 10/11 | x64, ARM64 | `.msi` installer, portable `.zip` | Azure Trusted Signing (executable and MSI) |
| Linux (glibc 2.35+) | x86_64, aarch64 | `.deb`, `.rpm`, `.tar.gz` | Build provenance attestations + SHA-256 checksums |

Every release also carries `SHA256SUMS.txt` and a signed build-provenance attestation
for each file. Users can verify a download with:

```bash
gh attestation verify IntersectionLab-0.1.0-macos-universal.dmg --repo schreck61/intersection-lab
```

## Making a release

1. Update `version` in `Cargo.toml`, run `cargo build` (refreshes `Cargo.lock`), and
   regenerate the attributions if dependencies changed: `packaging/licenses.sh`.
2. Commit, then tag and push:
   ```bash
   git tag v0.1.0
   git push origin v0.1.0
   ```
3. The workflow checks that the tag matches `Cargo.toml`, verifies licenses and
   attributions, builds and signs every package, and creates a **draft** release with
   all files, checksums and attestations. Review it on GitHub and publish.

To test the pipeline without releasing, run the *Release* workflow manually
(Actions → Release → Run workflow): it builds everything and attaches the packages to
the run as artifacts.

Signing is optional per platform: without its secrets a platform's packages are built
unsigned (macOS ad-hoc), so the pipeline works before certificates are set up.

## Signing setup

Add these under the repository's **Settings → Secrets and variables → Actions**.

### macOS (Developer ID + notarization)

| Name | Kind | Value |
|---|---|---|
| `MACOS_CERTIFICATE` | secret | Base64 of your *Developer ID Application* certificate and private key exported as `.p12` |
| `MACOS_CERTIFICATE_PASSWORD` | secret | The password you gave the `.p12` export |
| `APPLE_API_KEY` | secret | Base64 of an App Store Connect API key (`AuthKey_XXXX.p8`) |
| `APPLE_API_KEY_ID` | secret | That key's ID |
| `APPLE_API_ISSUER` | secret | Your App Store Connect issuer ID |

1. In Keychain Access, select the *Developer ID Application: …* certificate (with its
   private key), *File → Export Items…*, save as `.p12` with a password, then
   `base64 -i cert.p12 | pbcopy` and paste into `MACOS_CERTIFICATE`.
2. In App Store Connect, *Users and Access → Integrations → App Store Connect API*,
   create a team key with *Developer* access, download the `.p8` once, then
   `base64 -i AuthKey_XXXX.p8 | pbcopy` into `APPLE_API_KEY`; copy the key ID and issuer ID.

The workflow imports the certificate into a temporary keychain, signs with the hardened
runtime, notarizes the app and the DMG with `notarytool`, staples both, and deletes the
keychain and key afterwards.

### Windows (Azure Trusted Signing)

| Name | Kind | Value |
|---|---|---|
| `AZURE_TENANT_ID` | secret | Microsoft Entra tenant ID |
| `AZURE_CLIENT_ID` | secret | App registration (service principal) client ID |
| `AZURE_CLIENT_SECRET` | secret | That app registration's client secret |
| `AZURE_SIGNING_ENDPOINT` | variable | Your account's region endpoint, e.g. `https://eus.codesigning.azure.net` |
| `AZURE_SIGNING_ACCOUNT` | variable | Trusted Signing account name |
| `AZURE_SIGNING_PROFILE` | variable | Certificate profile name |

1. In the Azure portal, create a *Trusted Signing* account (Microsoft has since renamed
   the service *Artifact Signing*), complete identity validation for yourself, and
   create a *Public Trust* certificate profile.
2. Create an app registration with a client secret and give it the *Trusted Signing
   Certificate Profile Signer* role on the account.
3. Fill in the secrets and variables above.

The workflow signs `intersection-lab.exe` before packaging (so the zip and the MSI both
carry the signed executable) and then signs the MSI, with SHA-256 digests and an RFC 3161
timestamp. If Microsoft retires `azure/trusted-signing-action`, switch the two signing
steps to its successor action; the inputs are the same.

### Linux

Linux packages aren't code-signed by a platform authority; integrity comes from the
checksums and the signed provenance attestations above. If you later publish to an apt or
yum repository, sign that repository's metadata with a GPG key.

## Open-source compliance

* The project is dual licensed **MIT OR Apache-2.0** (`LICENSE-MIT`, `LICENSE-APACHE`,
  `COPYRIGHT`); every source file carries an SPDX header.
* [`THIRD_PARTY_LICENSES.md`](../THIRD_PARTY_LICENSES.md) lists every crate compiled into
  the binaries on any platform, with its license text (from each crate's own license
  files, so copyright lines are preserved), plus the notices of the fonts egui embeds. It
  is generated by [`licenses.sh`](licenses.sh) (cargo-about, configured by
  [`about.toml`](../about.toml) and [`about.hbs`](about.hbs)); CI fails if it is stale.
* [`deny.toml`](../deny.toml) restricts dependencies to permissive licenses compatible
  with the project's license and checks security advisories and crate sources on every
  push.
* Every package carries the license files and the third-party notices:

  | Package | Where |
  |---|---|
  | macOS `.dmg` | `Licenses/` folder in the disk image, and `Contents/Resources/` inside the app |
  | Windows `.msi` / `.zip` | Next to `intersection-lab.exe` (`*.txt`); the installer shows the license |
  | `.deb` | `/usr/share/doc/intersection-lab/` |
  | `.rpm` | `/usr/share/licenses/intersection-lab/` |
  | `.tar.gz` | Top-level folder |

* The app's *About* window shows the copyright, the license and the full third-party
  notices.

## Building packages locally

```bash
packaging/macos/package.sh          # macOS: universal DMG (signs if MACOS_SIGN_IDENTITY is set)
packaging/linux/package.sh          # Linux: .deb, .rpm, .tar.gz (needs cargo-deb, cargo-generate-rpm)
pwsh packaging/windows/package.ps1 -Arch x64   # Windows, after cargo build --release --target …
packaging/licenses.sh               # regenerate THIRD_PARTY_LICENSES.md (needs cargo-about)
packaging/icons/build.sh            # regenerate icons (macOS)
```

Outputs go to `dist/`.
