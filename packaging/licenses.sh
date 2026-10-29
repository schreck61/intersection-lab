#!/usr/bin/env bash
# Copyright (c) 2026 Jim Schreckengast
# SPDX-License-Identifier: MIT OR Apache-2.0
#
# Regenerate THIRD_PARTY_LICENSES.md: every crate compiled into the binaries (from
# Cargo.lock, all platforms) with its license text, plus the per-font notices of the
# fonts egui embeds. Requires cargo-about. Use --check to fail if the file is stale.
set -euo pipefail
cd "$(dirname "$0")/.."
out=THIRD_PARTY_LICENSES.md
tmp=$(mktemp)
trap 'rm -f "$tmp"' EXIT

cargo about generate --locked packaging/about.hbs -o "$tmp"

# The fonts built into egui carry their own notices beyond the crate's license.
fonts_dir=$(cargo metadata --format-version 1 --locked |
    python3 -c 'import json,sys,os; m=json.load(sys.stdin); p=[p for p in m["packages"] if p["name"]=="epaint_default_fonts"]; print(os.path.join(os.path.dirname(p[0]["manifest_path"]),"fonts") if p else "")')
if [ -n "$fonts_dir" ]; then
    {
        printf '## Bundled fonts\n\n'
        printf 'The user interface embeds fonts from epaint_default_fonts. Their notices:\n\n'
        for f in Hack-Regular.txt egui-icons.txt emoji-icon-font-mit-license.txt; do
            if [ -f "$fonts_dir/$f" ]; then
                printf '### %s\n\n```text\n' "$f"
                cat "$fonts_dir/$f"
                printf '\n```\n\n'
            fi
        done
    } >> "$tmp"
fi

if [ "${1:-}" = "--check" ]; then
    if ! diff -q "$tmp" "$out" > /dev/null; then
        echo "$out is out of date: run packaging/licenses.sh and commit the result" >&2
        exit 1
    fi
    echo "$out is up to date"
else
    cp "$tmp" "$out"
    echo "wrote $out"
fi
