#!/bin/bash
# Build the VardeFM AppImage (x86_64). Run from the repo root:
#   packaging/appimage/build-appimage.sh [output-dir]
# Requires: cargo, appimagetool in PATH.
set -euo pipefail
cd "$(dirname "$0")/../.."
OUT="${1:-dist}"
VERSION="$(cargo metadata --no-deps --format-version 1 --locked | jq -r '.packages[] | select(.name=="vardefm-cli") | .version')"
echo "[####------] 1/3 building vardefm $VERSION (release)"
cargo build --release --locked -p vardefm-cli
echo "[########--] 2/3 assembling AppDir"
rm -rf "$OUT/AppDir" "$OUT"/VardeFM-*.AppImage
mkdir -p "$OUT/AppDir/usr/bin" "$OUT/AppDir/usr/share/doc/vardefm" "$OUT/AppDir/usr/share/man/man1"
cp target/release/vardefm "$OUT/AppDir/usr/bin/"
cp packaging/appimage/vardefm.desktop "$OUT/AppDir/"
cp packaging/appimage/vardefm.svg "$OUT/AppDir/vardefm.svg"
cp packaging/appimage/AppRun "$OUT/AppDir/AppRun"
chmod +x "$OUT/AppDir/AppRun"
cp LICENSE packaging/README.md "$OUT/AppDir/usr/share/doc/vardefm/" 2>/dev/null || cp LICENSE "$OUT/AppDir/usr/share/doc/vardefm/"
cp docs/*.md "$OUT/AppDir/usr/share/doc/vardefm/"
cp packaging/vardefm.1 "$OUT/AppDir/usr/share/man/man1/"
cp packaging/linux/99-vardefm-cm108.rules "$OUT/AppDir/usr/share/doc/vardefm/" 2>/dev/null || true
# AppStream metadata so software centres show it nicely
cat > "$OUT/AppDir/vardefm.appdata.xml" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<component type="desktop-application">
  <id>vardefm</id>
  <metadata_license>CC0-1.0</metadata_license>
  <project_license>GPL-3.0-or-later</project_license>
  <name>VardeFM</name>
  <summary>Open high-speed data modem for FM radios</summary>
  <description><p>Drop-in alternative to VARA FM for Winlink and similar apps.</p></description>
  <launchable type="desktop-id">vardefm.desktop</launchable>
  <url type="homepage">https://github.com/notepid/vardeFM</url>
  <provides><binary>vardefm</binary></provides>
</component>
EOF
echo "[##########] 3/3 running appimagetool"
ARCH=x86_64 appimagetool "$OUT/AppDir" "$OUT/VardeFM-${VERSION}-x86_64.AppImage"
ls -l "$OUT"/VardeFM-*.AppImage
"$OUT"/VardeFM-*.AppImage --version
