# Packaging VardeFM (maintainer notes)

Release artifacts are built by CI; this file shows how to build them locally
and what secrets the AUR auto-update needs.

## Local builds (repo root)

```sh
cargo build --release --locked -p vardefm-cli   # needed first: fills target/release/vardefm

cargo deb --locked -p vardefm-cli               # -> target/debian/vardefm_<ver>_amd64.deb
cd crates/vardefm-cli && cargo generate-rpm -o ../../target/generate-rpm/vardefm.rpm

bash packaging/appimage/build-appimage.sh dist  # -> dist/VardeFM-<ver>-x86_64.AppImage
bash packaging/install.sh --to /tmp/vf-test --tarball   # test the 1-liner
```

Debian metadata lives in `crates/vardefm-cli/Cargo.toml`
(`[package.metadata.deb]`), RPM metadata in `[package.metadata.generate-rpm]`
(needs `name = "vardefm"` to override the crate name). Shared files:
`packaging/systemd/vardefm.service`, `packaging/vardefm.1`,
`packaging/linux/99-vardefm-cm108.rules`.

## 1-line installer

```sh
curl -fsSL https://raw.githubusercontent.com/notepid/vardeFM/master/packaging/install.sh | sh
```

Installs the release tarball binary (or the .deb/.rpm when run as root on a
matching system). Options: `--version X.Y.Z`, `--to DIR`,
`--deb` / `--rpm` / `--tarball`.

## AUR

- `packaging/aur/vardefm/PKGBUILD` — builds from the release source tarball.
- `packaging/aur/vardefm-bin/PKGBUILD` — repacks the `linux_amd64` release tarball.
- `packaging/aur/vardefm-git/PKGBUILD` — builds the latest git `master` (VCS package).

Refresh `.SRCINFO` after touching a `PKGBUILD`:

```sh
(cd packaging/aur/vardefm && makepkg --printsrcinfo > .SRCINFO)
(cd packaging/aur/vardefm-bin && makepkg --printsrcinfo > .SRCINFO)
(cd packaging/aur/vardefm-git && makepkg --printsrcinfo > .SRCINFO)
```

The `packaging.yml` workflow updates `vardefm` and `vardefm-bin` on every `v*`
tag. `vardefm-git` floats with git master, so it needs no per-release update
(only push when the `PKGBUILD` itself changes).

The `packaging.yml` workflow updates both on every `v*` tag. It needs three
repository secrets: `AUR_SSH_PRIVATE_KEY` (the AUR ssh key), `AUR_USERNAME`,
`AUR_EMAIL`. First push of a new package base (`vardefm`, `vardefm-bin`) must
be done once manually over ssh — see below — afterwards CI keeps them updated.

```sh
# one-time manual push, e.g. for vardefm:
git clone ssh://aur@aur.archlinux.org/vardefm.git /tmp/aur-vardefm
cp packaging/aur/vardefm/{PKGBUILD,.SRCINFO} /tmp/aur-vardefm/
cd /tmp/aur-vardefm && git add PKGBUILD .SRCINFO && git commit -m "vardefm 0.3.1-1" && git push
```
