#!/usr/bin/env bash
# Copyright (c) 2026 Jim Schreckengast
# SPDX-License-Identifier: MIT OR Apache-2.0
#
# Build a universal (Apple silicon + Intel) "Intersection Lab.app", sign it, wrap it in
# a DMG and, when credentials are available, notarize and staple both.
#
# Environment (all optional; without them the result is ad-hoc signed and unnotarized):
#   MACOS_SIGN_IDENTITY   e.g. "Developer ID Application: Jane Doe (TEAMID1234)", or its
#                         certificate's SHA-1 hash
#   Notarization with an App Store Connect API key (preferred):
#     APPLE_API_KEY_PATH  path to the AuthKey_XXXX.p8 file
#     APPLE_API_KEY_ID    key ID
#     APPLE_API_ISSUER    issuer ID
#   or with an Apple ID:
#     APPLE_ID, APPLE_TEAM_ID, APPLE_APP_PASSWORD (app-specific password)
#   OUT_DIR               where to write the DMG (default: dist)
set -euo pipefail
cd "$(dirname "$0")/../.."

APP_NAME="Intersection Lab"
BIN=intersection-lab
BUNDLE_ID=io.github.schreck61.intersection-lab
VERSION=$(sed -n 's/^version = "\(.*\)"/\1/p' Cargo.toml | head -1)
OUT_DIR=${OUT_DIR:-dist}
WORK=target/package-macos
APP="$WORK/$APP_NAME.app"
DMG="$OUT_DIR/IntersectionLab-$VERSION-macos-universal.dmg"
export MACOSX_DEPLOYMENT_TARGET=11.0

echo "==> Building $APP_NAME $VERSION (arm64 + x86_64)"
for t in aarch64-apple-darwin x86_64-apple-darwin; do
    rustup target add "$t" > /dev/null
    cargo build --release --locked --target "$t" --bin "$BIN"
done

rm -rf "$WORK" && mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Resources" "$OUT_DIR"
lipo -create -output "$APP/Contents/MacOS/$BIN" \
    "target/aarch64-apple-darwin/release/$BIN" "target/x86_64-apple-darwin/release/$BIN"
lipo -info "$APP/Contents/MacOS/$BIN"
cp packaging/macos/AppIcon.icns "$APP/Contents/Resources/"
cp LICENSE-MIT LICENSE-APACHE COPYRIGHT THIRD_PARTY_LICENSES.md "$APP/Contents/Resources/"
cat > "$APP/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleName</key><string>$APP_NAME</string>
    <key>CFBundleDisplayName</key><string>$APP_NAME</string>
    <key>CFBundleExecutable</key><string>$BIN</string>
    <key>CFBundleIdentifier</key><string>$BUNDLE_ID</string>
    <key>CFBundleVersion</key><string>$VERSION</string>
    <key>CFBundleShortVersionString</key><string>$VERSION</string>
    <key>CFBundlePackageType</key><string>APPL</string>
    <key>CFBundleInfoDictionaryVersion</key><string>6.0</string>
    <key>CFBundleIconFile</key><string>AppIcon</string>
    <key>LSMinimumSystemVersion</key><string>$MACOSX_DEPLOYMENT_TARGET</string>
    <key>LSApplicationCategoryType</key><string>public.app-category.education</string>
    <key>NSHighResolutionCapable</key><true/>
    <key>NSHumanReadableCopyright</key><string>Copyright © 2026 Jim Schreckengast. MIT OR Apache-2.0.</string>
</dict>
</plist>
PLIST
plutil -lint "$APP/Contents/Info.plist"

sign() {
    if [ -n "${MACOS_SIGN_IDENTITY:-}" ]; then
        codesign --force --options runtime --timestamp --sign "$MACOS_SIGN_IDENTITY" "$@"
    else
        codesign --force --sign - "$@"
    fi
}

notarize() {
    local auth out status id
    if [ -n "${APPLE_API_KEY_PATH:-}" ]; then
        auth=(--key "$APPLE_API_KEY_PATH" --key-id "$APPLE_API_KEY_ID" --issuer "$APPLE_API_ISSUER")
    else
        auth=(--apple-id "$APPLE_ID" --team-id "$APPLE_TEAM_ID" --password "$APPLE_APP_PASSWORD")
    fi
    out=$(xcrun notarytool submit "$1" "${auth[@]}" --output-format json --wait --timeout 30m) || true
    echo "$out"
    status=$(plutil -extract status raw - <<< "$out" 2> /dev/null || echo unknown)
    if [ "$status" != "Accepted" ]; then
        # Show Apple's log, which lists each problem it found.
        echo "error: notarization of $1 failed (status: $status)" >&2
        id=$(plutil -extract id raw - <<< "$out" 2> /dev/null) && xcrun notarytool log "$id" "${auth[@]}" >&2
        return 1
    fi
}

can_notarize=false
if [ -n "${MACOS_SIGN_IDENTITY:-}" ] && { [ -n "${APPLE_API_KEY_PATH:-}" ] || [ -n "${APPLE_ID:-}" ]; }; then
    can_notarize=true
fi

echo "==> Signing (${MACOS_SIGN_IDENTITY:-ad-hoc})"
sign "$APP"
codesign --verify --strict --verbose=2 "$APP"

if $can_notarize; then
    # Notarize the app itself so a copy dragged out of the DMG opens offline too.
    echo "==> Notarizing the app"
    ditto -c -k --keepParent "$APP" "$WORK/app.zip"
    notarize "$WORK/app.zip"
    xcrun stapler staple "$APP"
    spctl --assess --type execute --verbose=2 "$APP"
fi

echo "==> Building $DMG"
stage="$WORK/dmg"
mkdir -p "$stage/Licenses"
cp -R "$APP" "$stage/"
ln -s /Applications "$stage/Applications"
cp LICENSE-MIT LICENSE-APACHE COPYRIGHT THIRD_PARTY_LICENSES.md "$stage/Licenses/"
rm -f "$DMG"
hdiutil create -volname "$APP_NAME $VERSION" -srcfolder "$stage" -fs HFS+ -format UDZO -ov "$DMG" > /dev/null
sign "$DMG"

if $can_notarize; then
    echo "==> Notarizing the DMG"
    notarize "$DMG"
    xcrun stapler staple "$DMG"
    spctl --assess --type open --context context:primary-signature --verbose=2 "$DMG"
else
    echo "    (not notarized: set MACOS_SIGN_IDENTITY and Apple credentials to notarize)"
fi
echo "==> $DMG"
