#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Topola contributors
#
# SPDX-License-Identifier: MIT

# Fetch the benchmark corpora into bench/corpora-src/ (gitignored).
#
#   PCBench      — pinned commit, MIT dataset of real KiCad boards
#   freerouting  — pinned commit, sparse (fixtures/ + tests/), issue-
#                  reproduction .dsn boards; GPL — see
#                  tests/external/README.md
#
# Commits are pinned so corpus contents are stable across fetches.
# Usage: scripts/fetch_corpora.sh
# Override locations with PCBENCH_SRC / FREEROUTING_SRC to point at
# existing clones instead (symlinked into bench/corpora-src/).

set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$ROOT/bench/corpora-src"
mkdir -p "$SRC"

PCBENCH_COMMIT="dec3be75cbdef74787625f9043c7391cd473bb64"
FREEROUTING_COMMIT="1be66ad2797395f3a258a7f3b9b31c724a624867"

fetch_pinned() {
    local url="$1" commit="$2" dest="$3"
    git init -q "$dest"
    git -C "$dest" remote add origin "$url"
    git -C "$dest" fetch -q --depth 1 origin "$commit"
    git -C "$dest" -c advice.detachedHead=false checkout -q FETCH_HEAD
}

if [[ -n "${PCBENCH_SRC:-}" ]]; then
    ln -sfn "$PCBENCH_SRC" "$SRC/PCBench"
    echo "linked PCBench -> $PCBENCH_SRC"
elif [[ ! -d "$SRC/PCBench/.git" ]]; then
    fetch_pinned https://github.com/PCBench/PCBench.git "$PCBENCH_COMMIT" "$SRC/PCBench"
fi

if [[ -n "${FREEROUTING_SRC:-}" ]]; then
    ln -sfn "$FREEROUTING_SRC" "$SRC/freerouting"
    echo "linked freerouting -> $FREEROUTING_SRC"
elif [[ ! -d "$SRC/freerouting/.git" ]]; then
    git init -q "$SRC/freerouting"
    git -C "$SRC/freerouting" remote add origin https://github.com/freerouting/freerouting.git
    git -C "$SRC/freerouting" sparse-checkout set fixtures tests
    git -C "$SRC/freerouting" fetch -q --depth 1 --filter=blob:none origin "$FREEROUTING_COMMIT"
    git -C "$SRC/freerouting" -c advice.detachedHead=false checkout -q FETCH_HEAD
fi

echo "corpora ready under $SRC"
