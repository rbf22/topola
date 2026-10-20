#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Topola contributors
#
# SPDX-License-Identifier: MIT

# Blocker census: one deterministic baseline run per corpus board,
# collecting .blockers.csv (per-failure obstacle attribution) so the
# fixed-vs-routed blocker composition can be tabulated per board.
#
# Usage:
#   scripts/blocker_census.sh            run census + report
#   JOBS=8 scripts/blocker_census.sh     parallel workers
#   ROW_FILTER=wave scripts/...          only rows whose slug matches
#   scripts/blocker_census.sh --report   rebuild the table only
#
# Runs two modes per board by default: baseline ("" — canonical
# unrouted count; only searches that terminate report blockers) and
# "--per-ratline-steps 3000" (caps each search so it terminates and
# attributes blockers — maximal failure-event coverage).
#
# Results land in bench/out/blocker-census/<slug>.* ; the composition
# table is printed by scripts/blocker_report.py.

set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

BIN="$ROOT/target/release/topola"
OUT_DIR="$ROOT/bench/out/blocker-census"
MANIFEST="$ROOT/bench/corpus.tsv"
RESULTS_DIR="$OUT_DIR/rows"
mkdir -p "$OUT_DIR" "$RESULTS_DIR"

source "$ROOT/scripts/bench_lib.sh"

# ----- worker mode: execute one task line, write one result row file
if [[ "${1:-}" == "worker" ]]; then
    IFS='|' read -r dsn modes budget seed name <<<"$2"
    run_row "$dsn" "$modes" "$budget" "$seed" "$name" >"$RESULTS_DIR/$name.tsv"
    exit 0
fi

SEED="${SEED:-0}"
JOBS="${JOBS:-$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)}"
CENSUS_MODES=("" "--per-ratline-steps 3000")

if [[ "${1:-}" == "--report" ]]; then
    python3 "$ROOT/scripts/blocker_report.py" "$OUT_DIR"
    exit 0
fi

ensure_binary

TASKS=()
while IFS=$'\t' read -r slug source src layers nets budget class notes; do
    [[ "$slug" =~ ^#.*$ || -z "$slug" ]] && continue
    [[ -n "${ROW_FILTER:-}" && "$slug" != *"$ROW_FILTER"* ]] && continue
    dsn=$(board_dsn "$source" "$src" "$slug")
    for modes in "${CENSUS_MODES[@]}"; do
        mslug=$([[ -z "$modes" ]] && echo "census" || echo -n "$modes" | tr ' -' '__')
        TASKS+=("$dsn|$modes|$budget|$SEED|${slug}__${mslug}")
    done
done < "$MANIFEST"

((${#TASKS[@]})) || { echo "no tasks matched" >&2; exit 1; }
echo "census: ${#TASKS[@]} runs across $JOBS workers (seed=$SEED)" >&2

printf '%s\n' "${TASKS[@]}" | xargs -P "$JOBS" -I {} "$0" worker {}

# aggregate result rows (same schema as sweep results.tsv)
{
    printf '#board\tmode\tsteps_budget\tseed\tunrouted\tunconnected\ttotal\tses_sha256\twall_s\tstatus\n'
    cat "$RESULTS_DIR"/*.tsv 2>/dev/null | sort | tr '|' '\t'
} >"$OUT_DIR/results.tsv"

python3 "$ROOT/scripts/blocker_report.py" "$OUT_DIR"
