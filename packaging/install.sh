#!/bin/sh
# VardeFM one-line installer.
#
#   curl -fsSL https://raw.githubusercontent.com/notepid/vardeFM/master/packaging/install.sh | sh
#   curl -fsSL https://raw.githubusercontent.com/notepid/vardeFM/master/packaging/install.sh | sh -s -- --version 0.3.1 --to ~/.local/bin
#
# Installs the vardefm binary for your OS/arch from GitHub releases.
# Prefers native packages (.deb/.rpm) when run as root on a matching system,
# otherwise installs the tarball binary. Never prints secrets.
set -eu

REPO="notepid/vardeFM"
BIN="vardefm"
VERSION="latest"
TO=""
USE_PKG="auto"   # auto|deb|rpm|tarball

while [ $# -gt 0 ]; do
  case "$1" in
    --version=*) VERSION="${1#--version=}"; shift;;
    --version) VERSION="${2:-latest}"; shift 2;;
    --to=*) TO="${1#--to=}"; shift;;
    --to) TO="${2:-}"; shift 2;;
    --deb) USE_PKG="deb"; shift;;
    --rpm) USE_PKG="rpm"; shift;;
    --tarball) USE_PKG="tarball"; shift;;
    --help|-h) echo "Usage: install.sh [--version X.Y.Z] [--to DIR] [--deb|--rpm|--tarball]"; exit 0;;
    *) die "unknown arg: $1 (see --help)";;
  esac
done

say() { printf '%s\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }
have() { command -v "$1" >/dev/null 2>&1; }

OS="$(uname -s | tr '[:upper:]' '[:lower:]')"
ARCH="$(uname -m)"
case "$OS" in
  linux) OS=linux;;
  darwin) OS=darwin;;
  *) die "unsupported OS: $OS (Linux and macOS only; Windows: use the .zip from GitHub releases)";;
esac
case "$ARCH" in
  x86_64|amd64) PKGARCH=amd64;;
  aarch64|arm64) PKGARCH=arm64;;
  armv7l|armv7|arm) PKGARCH=armv7; [ "$OS" = linux ] || die "unsupported arch: $ARCH";;
  *) die "unsupported arch: $ARCH";;
esac

# Resolve version -> tag + base URL
if [ "$VERSION" = "latest" ]; then
  API="https://api.github.com/repos/$REPO/releases/latest"
  TAG="$(curl -fsSL "$API" | grep -m1 '"tag_name"' | cut -d'"' -f4)"
  [ -n "$TAG" ] || die "could not resolve latest release"
  VER="${TAG#v}"
else
  VER="$VERSION"; TAG="v$VERSION"
fi
BASE="https://github.com/$REPO/releases/download/$TAG"
say "[##--------] resolved $BIN $TAG for $OS/$PKGARCH"

TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT INT TERM

install_tarball() {
  f="vardefm_${VER}_${OS}_${PKGARCH}.tar.gz"
  say "[####------] downloading $f"
  curl -fsSL -o "$TMP/pkg.tgz" "$BASE/$f" || die "download failed: $BASE/$f"
  tar xzf "$TMP/pkg.tgz" -C "$TMP"
  src="$(find "$TMP" -name "$BIN" -type f | head -1)"
  [ -n "$src" ] || die "binary not found in archive"
  if [ -n "$TO" ]; then
    mkdir -p "$TO"; cp "$src" "$TO/$BIN"; chmod +x "$TO/$BIN"
    say "[##########] installed to $TO/$BIN (add to PATH if needed)"
  elif [ "$(id -u)" = 0 ]; then
    cp "$src" /usr/local/bin/; chmod +x /usr/local/bin/$BIN
    # udev rules for CM108 PTT on Linux
    if [ "$OS" = linux ]; then
      r="$(find "$TMP" -name '*.rules' | head -1)"
      if [ -n "$r" ]; then cp "$r" /etc/udev/rules.d/ 2>/dev/null && udevadm control --reload-rules 2>/dev/null || true; fi
    fi
    say "[##########] installed to /usr/local/bin/$BIN"
  else
    mkdir -p "$HOME/.local/bin"; cp "$src" "$HOME/.local/bin/$BIN"; chmod +x "$HOME/.local/bin/$BIN"
    say "[##########] installed to \$HOME/.local/bin/$BIN (add to PATH if needed)"
  fi
  "$HOME/.local/bin/$BIN" --version 2>/dev/null || /usr/local/bin/$BIN --version 2>/dev/null || "$TO/$BIN" --version 2>/dev/null || true
}

install_deb() {
  f="vardefm_${VER}_amd64.deb"
  say "[####------] downloading $f"
  curl -fsSL -o "$TMP/pkg.deb" "$BASE/$f" || die "download failed (no .deb for this arch yet? falling back to tarball)" 
  if have apt-get; then
    say "[######----] installing .deb (needs root)"
    apt-get update && apt-get install -y "$TMP/pkg.deb"
  else
    dpkg -i "$TMP/pkg.deb" || die "dpkg failed; try: sudo apt install -f"
  fi
  say "[##########] installed vardefm $VER from .deb"
}

install_rpm() {
  f="vardefm-${VER}-1.x86_64.rpm"
  say "[####------] downloading $f"
  curl -fsSL -o "$TMP/pkg.rpm" "$BASE/$f" || die "download failed"
  if have dnf; then dnf install -y "$TMP/pkg.rpm";
  elif have yum; then yum install -y "$TMP/pkg.rpm";
  elif have zypper; then zypper --no-confirm install "$TMP/pkg.rpm";
  else rpm -Uvh "$TMP/pkg.rpm"; fi
  say "[##########] installed vardefm $VER from .rpm"
}

# Decide method
if [ "$USE_PKG" = "auto" ]; then
  if [ "$(id -u)" = 0 ] && [ "$OS" = linux ] && [ "$PKGARCH" = "amd64" ]; then
    if have apt-get || have dpkg; then USE_PKG="deb";
    elif have dnf || have yum || have rpm; then USE_PKG="rpm";
    else USE_PKG="tarball"; fi
  else
    USE_PKG="tarball"
  fi
fi

case "$USE_PKG" in
  deb) install_deb;;
  rpm) install_rpm;;
  tarball) install_tarball;;
esac
