#!/usr/bin/env bash
# Build crates/vardefm-ffi into packaging/apple/build/VardeFM.xcframework
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="$ROOT/packaging/apple/build"
HEADERS="$OUT/headers"
TMP="$OUT/tmp"
XCFRAMEWORK="$OUT/VardeFM.xcframework"
ZIP="$OUT/VardeFM.xcframework.zip"

IPHONEOS_DEPLOYMENT_TARGET="${IPHONEOS_DEPLOYMENT_TARGET:-16.0}"
MACOSX_DEPLOYMENT_TARGET="${MACOSX_DEPLOYMENT_TARGET:-11.0}"
export IPHONEOS_DEPLOYMENT_TARGET MACOSX_DEPLOYMENT_TARGET

if ! command -v cargo >/dev/null || ! command -v rustup >/dev/null; then
  echo "rustup/cargo required" >&2
  exit 1
fi
if ! command -v xcodebuild >/dev/null; then
  echo "xcodebuild required" >&2
  exit 1
fi

TARGETS=(
  aarch64-apple-ios
  aarch64-apple-ios-sim
  x86_64-apple-ios
  aarch64-apple-darwin
  x86_64-apple-darwin
)

rustup target add "${TARGETS[@]}" >/dev/null

rm -rf "$OUT"
mkdir -p "$HEADERS/CVardeFM" "$TMP"

cp "$ROOT/crates/vardefm-ffi/include/CVardeFM/vardefm.h" "$HEADERS/CVardeFM/"
cp "$ROOT/crates/vardefm-ffi/include/CVardeFM/module.modulemap" "$HEADERS/CVardeFM/"

build_one() {
  local target="$1"
  echo "Building $target"
  cargo build --profile dist --locked -p vardefm-ffi --target "$target"
  cp "$ROOT/target/$target/dist/libvardefm.a" "$TMP/libvardefm-$target.a"
}

for t in "${TARGETS[@]}"; do
  build_one "$t"
done

lipo -create \
  "$TMP/libvardefm-aarch64-apple-ios-sim.a" \
  "$TMP/libvardefm-x86_64-apple-ios.a" \
  -output "$TMP/libvardefm-ios-sim.a"

lipo -create \
  "$TMP/libvardefm-aarch64-apple-darwin.a" \
  "$TMP/libvardefm-x86_64-apple-darwin.a" \
  -output "$TMP/libvardefm-macos.a"

rm -rf "$XCFRAMEWORK"
xcodebuild -create-xcframework \
  -library "$TMP/libvardefm-aarch64-apple-ios.a" -headers "$HEADERS" \
  -library "$TMP/libvardefm-ios-sim.a" -headers "$HEADERS" \
  -library "$TMP/libvardefm-macos.a" -headers "$HEADERS" \
  -output "$XCFRAMEWORK"

(
  cd "$OUT"
  rm -f VardeFM.xcframework.zip
  zip -qr VardeFM.xcframework.zip VardeFM.xcframework
)

CHECKSUM="$(swift package compute-checksum "$ZIP")"
echo "$CHECKSUM" > "$OUT/VardeFM.xcframework.zip.checksum"
echo "Built $XCFRAMEWORK"
echo "Zip     $ZIP"
echo "Checksum $CHECKSUM"
