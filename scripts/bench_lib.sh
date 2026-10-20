# SPDX-FileCopyrightText: 2026 Topola contributors
#
# SPDX-License-Identifier: MIT

# Shared helpers for scripts/bench.sh and scripts/sweep.sh.
# Source this file; expects ROOT, BIN, OUT_DIR to be set by the caller.

ensure_binary() {
    if [[ ! -x "$BIN" ]]; then
        echo "building topola-cli --release --features metrics ..." >&2
        cargo build --release -p topola-cli --features metrics >&2
    fi
}

sha256_file() {
    if command -v shasum >/dev/null; then
        shasum -a 256 "$1" | cut -d' ' -f1
    else
        sha256sum "$1" | cut -d' ' -f1
    fi
}

slug() {
    basename "$1" .dsn
}

# board dsn resolution: local rows use src as the in-repo path; corpus
# rows use the vendored fixture.
board_dsn() {
    local source="$1" src="$2" slug="$3"
    if [[ "$source" == "local" ]]; then
        echo "$ROOT/$src"
    else
        echo "$ROOT/tests/external/$source/$slug/$slug.dsn"
    fi
}

# Run one deterministic autoroute job.
#   $1 dsn path | $2 mode flags (string) | $3 step budget | $4 seed
#   $5 output name (file stem for .log/.ses/.progress.csv/.blockers.csv)
# Prints a result row to stdout:
#   dsn|modes|budget|seed|unrouted|unconnected|total|ses_sha|wall_s|status
run_row() {
    local dsn="$1" modes="$2" budget="$3" seed="$4" name="$5"
    local csv="$OUT_DIR/$name.progress.csv"
    local blockers="$OUT_DIR/$name.blockers.csv"
    local ses="$OUT_DIR/$name.ses"
    local log="$OUT_DIR/$name.log"

    # shellcheck disable=SC2086
    "$BIN" "$dsn" $modes --deterministic --max-steps "$budget" \
        --seed "$seed" \
        --progress-csv "$csv" --blockers-csv "$blockers" \
        -o "$ses" 2>"$log" || true

    local finished total unrouted unconnected steps wall sha status
    finished=$(grep -o 'finished: [0-9]* ratlines total, [0-9]* unrouted, [0-9]* unconnected' "$log" | tail -1 || true)
    total=$(echo "$finished" | awk '{print $2}')
    unrouted=$(echo "$finished" | awk '{print $5}')
    unconnected=$(echo "$finished" | awk '{print $7}')
    [[ -z "$unrouted" ]] && unrouted="-"
    [[ -z "$unconnected" ]] && unconnected="-"
    [[ -z "$total" ]] && total="-"
    steps=$(awk -F, '$1 ~ /^[0-9]/' "$csv" 2>/dev/null | tail -1 | cut -d, -f2)
    wall=$(awk -F, '$1 ~ /^[0-9]/' "$csv" 2>/dev/null | tail -1 | cut -d, -f1)
    sha=$([[ -f "$ses" ]] && sha256_file "$ses" | cut -c1-12 || echo "-")
    status="ok"
    grep -q "panicked" "$log" && status="PANIC"

    printf '%-36s %-58s %7s %9s %11s %8s %8s  %s\n' \
        "$name" "$modes" "${steps:--}" "$unrouted" "$unconnected" "${wall:--}" "$sha" "$status" >&2
    printf '%s\n' "$dsn|$modes|$budget|$seed|$unrouted|$unconnected|$total|$sha|${wall:-}|$status"
}
