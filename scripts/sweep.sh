#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Topola contributors
#
# SPDX-License-Identifier: MIT

# Parameter sweep over the benchmark corpus (bench/corpus.tsv).
#
# Every run is deterministic: `--deterministic --max-steps N --seed S`,
# so a row is a pure function of (board, config, budget, seed).
#
# Usage:
#   scripts/sweep.sh                run PHASE + SEEDS matrix, append results.tsv
#   PHASE=1 scripts/sweep.sh        single-flag scan only
#   PHASE=2 SEEDS="0 7" ./...       combo phase at two seeds
#   JOBS=8 scripts/sweep.sh         parallel workers (default: hw cores)
#   ROW_FILTER=wave scripts/sweep.sh  only rows whose slug matches
#   scripts/sweep.sh --report       rebuild the pivot table only
#
# Results land in bench/out/sweep/<slug>__<config>__s<seed>.* plus the
# appended bench/out/sweep/results.tsv (same schema as golden.tsv).

set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

BIN="$ROOT/target/release/topola"
OUT_DIR="$ROOT/bench/out/sweep"
MANIFEST="$ROOT/bench/corpus.tsv"
RESULTS_TSV="$OUT_DIR/results.tsv"
TASKS_FILE="$OUT_DIR/tasks.txt"
RESULTS_DIR="$OUT_DIR/rows"
mkdir -p "$OUT_DIR" "$RESULTS_DIR"

source "$ROOT/scripts/bench_lib.sh"

# ----- worker mode: execute one task line, write one result row file
if [[ "${1:-}" == "worker" ]]; then
    IFS='|' read -r dsn modes budget seed name <<<"$2"
    run_row "$dsn" "$modes" "$budget" "$seed" "$name" >"$RESULTS_DIR/$name.tsv"
    exit 0
fi

# ----- config matrix
PHASE1_CONFIGS=(
    ""
    "--corridor-presort"
    "--escape-presort"
    "--surgical-ripup"
    "--critical-first"
    "--preprobe"
    "--escape-stubs"
    "--pad-entry"
    "--multilayer"
    "--multilayer --anneal --anneal-mode continuous"
    "--multilayer --anneal --anneal-mode discrete"
)
# Phase-2 combos, revised after the phase-1 single-flag scan and the
# A4 blocker census: --critical-first was net-negative solo (dropped),
# --multilayer/--anneal-discrete/--preprobe/--escape-stubs were the
# winners; --surgical-ripup is retested now that blocker attribution
# works post-fix. "" is a post-fix baseline control.
PHASE2_CONFIGS=(
    ""
    "--multilayer"
    "--multilayer --anneal --anneal-mode discrete"
    "--multilayer --anneal --anneal-mode continuous --anneal-carve-top-k 16"
    "--multilayer --preprobe"
    "--multilayer --anneal --anneal-mode discrete --preprobe"
    "--multilayer --escape-stubs"
    "--multilayer --surgical-ripup"
    "--multilayer --anneal --anneal-mode discrete --surgical-ripup"
)

PHASE="${PHASE:-all}"
SEEDS="${SEEDS:-0}"
JOBS="${JOBS:-$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)}"

if [[ "${1:-}" == "--report" ]]; then
    python3 "$ROOT/scripts/sweep_report.py" "$RESULTS_TSV"
    exit 0
fi

ensure_binary

config_slug() {
    echo -n "$1" | tr ' -' '__' | tr -d '\n'
}

TASKS=()
while IFS=$'\t' read -r slug source src layers nets budget class notes; do
    [[ "$slug" =~ ^#.*$ || -z "$slug" ]] && continue
    [[ -n "${ROW_FILTER:-}" && "$slug" != *"$ROW_FILTER"* ]] && continue
    dsn=$(board_dsn "$source" "$src" "$slug")
    if [[ "$PHASE" == "1" || "$PHASE" == "all" ]]; then
        for cfg in "${PHASE1_CONFIGS[@]}"; do
            for seed in $SEEDS; do
                TASKS+=("$dsn|$cfg|$budget|$seed|${slug}$(config_slug "$cfg")__s${seed}")
            done
        done
    fi
    if [[ "$PHASE" == "2" || "$PHASE" == "all" ]]; then
        for cfg in "${PHASE2_CONFIGS[@]}"; do
            for seed in $SEEDS; do
                TASKS+=("$dsn|$cfg|$budget|$seed|${slug}$(config_slug "$cfg")__s${seed}")
            done
        done
    fi
done < "$MANIFEST"

# EXTRA_DSNS: space-separated dsn paths appended as local rows at a
# default budget (e.g. candE/modcore outside the repo).
for extra in ${EXTRA_DSNS:-}; do
    [[ -f "$extra" ]] || { echo "EXTRA_DSNS: $extra missing" >&2; continue; }
    exslug=$(slug "$extra")
    [[ -n "${ROW_FILTER:-}" && "$exslug" != *"$ROW_FILTER"* ]] && continue
    for cfg in "${PHASE1_CONFIGS[@]}"; do
        for seed in $SEEDS; do
            TASKS+=("$extra|$cfg|${EXTRA_BUDGET:-100000}|$seed|${exslug}$(config_slug "$cfg")__s${seed}")
        done
    done
done

((${#TASKS[@]})) || { echo "no tasks matched" >&2; exit 1; }
printf '%s\n' "${TASKS[@]}" >"$TASKS_FILE"
echo "sweep: ${#TASKS[@]} runs across $JOBS workers (phase=$PHASE seeds='$SEEDS')" >&2

printf '%-36s %-58s %7s %9s %11s %8s %8s  %s\n' \
    "board" "mode" "steps" "unrouted" "unconnected" "wall_s" "ses_sha" "status" >&2

# NUL-delimited: -I{} hits the 255-byte replsize limit on long
# board-path + flag-string lines; -0 passes each task as one argv.
printf '%s\0' "${TASKS[@]}" | xargs -0 -P "$JOBS" -n 1 "$0" worker

# aggregate
{
    printf '#board\tmode\tsteps_budget\tseed\tunrouted\tunconnected\ttotal\tses_sha256\twall_s\tstatus\n'
    cat "$RESULTS_DIR"/*.tsv 2>/dev/null | sort | tr '|' '\t'
} >"$RESULTS_TSV"
echo "wrote $RESULTS_TSV" >&2

if [[ "${REPORT:-1}" == "1" ]]; then
    python3 "$ROOT/scripts/sweep_report.py" "$RESULTS_TSV"
fi
