# Contributing to VardeFM

Thank you for wanting to help. Bug reports, test reports from real radios, documentation
and code are all welcome.

Be respectful. Argue about the code and the idea, not about the person.

## Reporting bugs and suggesting features

Use [GitHub issues](https://github.com/notepid/vardeFM/issues). Search for an existing
issue first.

A bug report should give:

- the VardeFM version (`vardefm --version`) and your operating system,
- the mode (narrow or wide), the radio interface and the PTT method,
- the command you ran and what the modem printed,
- a recording of the received audio, if you can make one: a 48 kHz WAV file, zipped.

## Before you start

- **Small fixes:** open a pull request.
- **Larger work:** open an issue first, so we agree on the approach before you spend time
  on it.
- **Protocol changes:** a change to the on-air protocol or to the host (VARA TCP)
  interface is discussed in an issue before any code. The PR updates
  [docs/PROTOCOL.md](docs/PROTOCOL.md) together with the code. Until 1.0, an on-air change
  that older versions cannot talk to means a minor version bump, because both stations
  must then run the same minor version.
- **Public documentation only:** build from public documentation and open-source
  software. Do not contribute anything that comes from reverse engineering, decompiling or
  capturing the traffic of closed software. Do not copy code whose license does not allow
  `MIT OR Apache-2.0`.

## Development setup

The README explains how to [build](README.md#building), how the
[workspace](README.md#workspace) is laid out, how to run two modems without radios
([`sim-pair`](README.md#virtual-radio-pair-sim-pair)) and what the
[tests](README.md#tests) cover.

CI runs these checks, with clippy and the tests on Windows, Linux x86-64, Linux ARM and
macOS. Run them before you push:

```sh
cargo fmt --all --check
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo test --workspace --locked
```

Formatting follows `rustfmt.toml` (120 columns).

CI also builds every release target and checks the licenses of the dependencies. If you
add a dependency, its license must be allowed in `deny.toml` (`cargo deny`) and accepted
in `about.toml` (`cargo about`), or CI fails.

## Making a change

- Fork the repository and branch from `master`.
- One topic per PR. Small PRs are reviewed sooner and merged sooner.
- Add or update tests with the change. A bug fix comes with a test that fails without the
  fix.
- Say in the PR how you tested it: unit tests, `sim-pair`, or real radios.
- If you change the DSP, the error correction or the modem, run the performance gates
  (`level_gate` and `fer_gate`) by hand. CI does not run them. A change that moves
  decoding performance on purpose records new points in the same PR. See
  [Tests](README.md#tests) in the README.
- Write commit messages with a short imperative subject, as in the existing history:
  `Send one BUFFER report per write, not two`. A prefix that names the area is fine:
  `detect_eval: decode each position once per trial`.
- Add a line under `[Unreleased]` in [CHANGELOG.md](CHANGELOG.md) for anything a user
  would notice.

## Working with AI agents

Contributions written with coding agents are welcome. Much of VardeFM is built this way.

**The PR author is responsible for the code, the PR and its quality, however it was
written.**

You do not need to have read every line. You do need to understand the change:

- what it does and why,
- the concepts it rests on, for example how the retry timer or the burst header it touches
  works,
- what it means for the rest of the project: on-air compatibility, the host interface,
  performance, and other open PRs.

You should be able to explain the approach in review and to choose between alternatives.
You may use an agent to answer review comments and to make the fixes, but you stand behind
the answer. "The agent wrote it" does not answer a review question.

Keep the agent's work in scope. Leave out refactors, renames and formatting changes that
the PR does not need.

Say in the PR description whether an agent wrote the code and which model reviewed it, if
any.
This does not decide whether the PR is accepted. It tells the reviewer how to read it.

## Review your own PR first

We encourage you to run an AI code review on your PR before you ask for a human review.
It is not required, and a small fix rarely needs one. On larger changes it finds real bugs
and saves a round of review.

1. Open the PR as a **draft**.
2. Run an AI code review over the full diff, for example `/code-review` in Claude Code or
   the equivalent in your tool.
3. Check each finding against the code. Fix the real ones.
4. Note in the PR what the review found, and what you dismissed and why.
5. Mark the PR ready for review.

The PR description follows the [pull request template](.github/PULL_REQUEST_TEMPLATE.md).
GitHub fills it in when you open a PR on the website. A PR opened with a ready-made
description, for example with `gh pr create --body` or by an agent, does not get it
automatically, so use the same sections and checklist there.

If you run a review, use a top-tier model if you can: Claude Opus, or an equivalent model
from another vendor. Smaller models miss the timing and protocol bugs that matter most in
a modem.

If you did not run one, say so in the PR. That is fine, and it tells the reviewer where
to look harder.

## What to expect from review

- VardeFM has one maintainer. PRs are reviewed one at a time, roughly in the order they
  become ready. Drafts are not reviewed.
- Feedback and requests for changes may come from an AI review that the maintainer ran.
  The maintainer checks the findings before posting them, but they can still be wrong. If
  a finding is wrong, say so and show why.
- Expect more than one round of review.
- A PR can be declined because it does not fit the direction of the project, even when the
  code is good. Opening an issue first avoids most of these cases.
- Once review has started, add commits. Do not rebase or force-push, because the reviewer
  then has to read the whole PR again. Merge `master` into your branch to resolve
  conflicts.
- A PR that records performance points is merged with a merge commit, so that the recorded
  commit stays reachable from `master`.

## License

VardeFM is dual licensed under MIT OR Apache-2.0. Unless you explicitly state otherwise,
any contribution intentionally submitted for inclusion in the work by you, as defined in
the Apache-2.0 license, shall be dual licensed as above, without any additional terms or
conditions.

You must have the right to submit what you contribute. This includes code that an agent
produced for you.
