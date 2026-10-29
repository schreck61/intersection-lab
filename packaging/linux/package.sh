#!/usr/bin/env bash
# Copyright (c) 2026 Jim Schreckengast
# SPDX-License-Identifier: MIT OR Apache-2.0
#
# Build Linux packages for the host architecture: .deb, .rpm and a portable tarball.
# Requires cargo-deb and cargo-generate-rpm. Build on the oldest distribution you want
# to support (the binary needs that glibc or newer).
#
# usage: packaging/linux/package.sh [out-dir]
set -euo pipefail
cd "$(dirname "$0")/../.."
OUT_DIR=${1:-dist}
VERSION=$(sed -n 's/^version = "\(.*\)"/\1/p' Cargo.toml | head -1)
ARCH=$(uname -m)                                 # x86_64 | aarch64
TARGET="$ARCH-unknown-linux-gnu"
mkdir -p "$OUT_DIR"

echo "==> Building $VERSION for $TARGET"
cargo build --release --locked --target "$TARGET" --bin intersection-lab

echo "==> .deb"
cargo deb --no-build --target "$TARGET" --output "$OUT_DIR/"
echo "==> .rpm"
cargo generate-rpm --target "$TARGET" --output "$OUT_DIR/"

echo "==> tarball"
name="IntersectionLab-$VERSION-linux-$ARCH"
stage="target/package-linux/$name"
rm -rf "target/package-linux" && mkdir -p "$stage"
cp "target/$TARGET/release/intersection-lab" "$stage/"
cp packaging/linux/intersection-lab.desktop packaging/linux/intersection-lab-256.png "$stage/"
cp LICENSE-MIT LICENSE-APACHE COPYRIGHT THIRD_PARTY_LICENSES.md README.md "$stage/"
tar -C target/package-linux -czf "$OUT_DIR/$name.tar.gz" "$name"
ls -l "$OUT_DIR"
