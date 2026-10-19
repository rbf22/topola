#!/usr/bin/env bash
# Deterministic benchmark matrix for the Topola autorouter.
#
# Every row runs `--deterministic --max-steps N --seed S`, so a run is a
# pure function of (board, options, seed, step budget): identical across
# machines and reruns. Wall-clock budgets (--max-seconds) must NOT be
# used for golden rows — they make outcomes machine-dependent.
#
# Usage:
#   scripts/bench.sh                run the matrix, print results
#   scripts/bench.sh --check        run the matrix, diff vs bench/golden.tsv
#   scripts/bench.sh --write-golden write results into bench/golden.tsv
#   ROW_FILTER=heart scripts/bench.sh   run only rows whose dsn matches
#   SEED=7 scripts/bench.sh             per-run --seed (default: none → 0)
#
# Each run also writes bench/out/<name>.progress.csv (the per-step time
# series) and <name>.blockers.csv (failure attributions), plus the
# routed <name>.ses.

set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

BIN="$ROOT/target/release/topola"
GOLDEN="$ROOT/bench/golden.tsv"
OUT_DIR="$ROOT/bench/out"
mkdir -p "$OUT_DIR"

CHECK=0
WRITE_GOLDEN=0
for arg in "$@"; do
    case "$arg" in
        --check) CHECK=1 ;;
        --write-golden) WRITE_GOLDEN=1 ;;
        *) echo "unknown arg: $arg" >&2; exit 2 ;;
    esac
done

if [[ ! -x "$BIN" ]]; then
    echo "building topola-cli --release --features metrics ..." >&2
    cargo build --release -p topola-cli --features metrics >&2
fi

# ROW := <dsn path>|<mode flag string>|<step budget>
ROWS=(
    "tests/unilayer/0603_breakout/0603_breakout.dsn||20000"
    "tests/unilayer/0603_breakout/0603_breakout.dsn|--surgical-ripup|20000"
    "tests/unilayer/4x4_1206_led_matrix_breakout/4x4_1206_led_matrix_breakout.dsn||50000"
    "tests/unilayer/red_led_heart/red_led_heart.dsn||60000"
    "tests/unilayer/red_led_heart/red_led_heart.dsn|--critical-first|60000"
    "tests/unilayer/red_led_heart/red_led_heart.dsn|--multilayer|60000"
    "tests/unilayer/red_led_heart/red_led_heart.dsn|--critical-first --multilayer|60000"
    "tests/unilayer/critical_corridor/critical_corridor.dsn|--critical-first|30000"
    "tests/external/optoisolator_4n35/optoisolator_4n35.dsn||30000"
    "tests/external/fasttest/fasttest.dsn|--multilayer|30000"
    "tests/multilayer/astable_multivibrator/astable_multivibrator.dsn|--multilayer|30000"
    "tests/multilayer/astable_multivibrator/astable_multivibrator.dsn|--multilayer --anneal --anneal-mode continuous|30000"
    "tests/multilayer/astable_multivibrator/astable_multivibrator.dsn|--multilayer --anneal --anneal-mode discrete|30000"
    "tests/multilayer/prerouted_lm317_breakout/unrouted_lm317_breakout.dsn|--multilayer|30000"
)

# External boards outside the repo (set env to include):
[[ -n "${CANDE_DSN:-}" && -f "${CANDE_DSN:-}" ]] && ROWS+=("$CANDE_DSN||100000")
[[ -n "${CANDE_DSN:-}" && -f "${CANDE_DSN:-}" ]] && ROWS+=("$CANDE_DSN|--critical-first|100000")

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

printf '%-42s %-4s %7s %9s %11s %8s %8s  %s\n' \
    "board" "mode" "steps" "unrouted" "unconnected" "wall_s" "ses_sha" "status" >&2

RESULTS=()
FAILURES=0

for row in "${ROWS[@]}"; do
    IFS='|' read -r dsn modes budget <<<"$row"
    name="$(slug "$dsn")$(echo -n "$modes" | tr ' -' '__' | tr -d '\n')"
    [[ -n "${ROW_FILTER:-}" && "$name" != *"$ROW_FILTER"* ]] && continue
    if [[ ! -f "$dsn" ]]; then
        printf '%-42s %-4s %7s %9s %11s %8s %8s  %s\n' \
            "$name" "-" "-" "-" "-" "-" "-" "missing" >&2
        continue
    fi

    csv="$OUT_DIR/$name.progress.csv"
    blockers="$OUT_DIR/$name.blockers.csv"
    ses="$OUT_DIR/$name.ses"
    log="$OUT_DIR/$name.log"

    # shellcheck disable=SC2086
    "$BIN" "$dsn" $modes --deterministic --max-steps "$budget" \
        --seed "${SEED:-0}" \
        --progress-csv "$csv" --blockers-csv "$blockers" \
        -o "$ses" 2>"$log" || true

    finished=$(grep -o 'finished: [0-9]* ratlines total, [0-9]* unrouted, [0-9]* unconnected' "$log" | tail -1 || true)
    # "finished: 16 ratlines total, 4 unrouted, 16 unconnected"
    total=$(echo "$finished" | awk '{print $2}')
    unrouted=$(echo "$finished" | awk '{print $5}')
    unconnected=$(echo "$finished" | awk '{print $7}')
    [[ -z "$unrouted" ]] && unrouted="-"
    [[ -z "$unconnected" ]] && unconnected="-"
    [[ -z "$total" ]] && total="-"
    steps=$(tail -1 "$csv" 2>/dev/null | cut -d, -f2)
    wall=$(tail -1 "$csv" 2>/dev/null | cut -d, -f1)
    sha=$([[ -f "$ses" ]] && sha256_file "$ses" | cut -c1-12 || echo "-")
    status="ok"
    grep -q "panicked" "$log" && status="PANIC"

    printf '%-42s %-4s %7s %9s %11s %8s %8s  %s\n' \
        "$name" "$modes" "${steps:--}" "$unrouted" "$unconnected" "${wall:--}" "$sha" "$status" >&2

    RESULTS+=("$dsn|$modes|$budget|${SEED:-0}|$unrouted|$unconnected|$total|$sha|${wall:-}|$status")
done

if [[ "$WRITE_GOLDEN" == 1 ]]; then
    {
        printf '#board\tmode\tsteps\tseed\tunrouted\tunconnected\ttotal\tses_sha256\twall_s\tstatus\n'
        printf '%s\n' "${RESULTS[@]}" | tr '|' '\t'
    } >"$GOLDEN"
    echo "wrote $GOLDEN" >&2
elif [[ "$CHECK" == 1 && -f "$GOLDEN" ]]; then
    echo "" >&2
    echo "== golden check ==" >&2
    for res in "${RESULTS[@]}"; do
        IFS='|' read -r dsn modes budget seed unrouted unconnected total sha wall status <<<"$res"
        key="$dsn|$modes|$budget|$seed"
        golden_row=$(grep -F -e "$dsn" "$GOLDEN" | grep -F -e "$modes" | awk -F'\t' -v b="$budget" -v s="$seed" '$3==b && $4==s' | head -1)
        if [[ -z "$golden_row" ]]; then
            printf '  ?? %s %s steps=%s seed=%s — no golden row\n' "$(slug "$dsn")" "$modes" "$budget" "$seed" >&2
            continue
        fi
        g_unrouted=$(echo "$golden_row" | cut -f5)
        g_unconn=$(echo "$golden_row" | cut -f6)
        g_sha=$(echo "$golden_row" | cut -f8)
        ok=1
        [[ "$unrouted" == "$g_unrouted" ]] || ok=0
        [[ "$unconnected" == "$g_unconn" ]] || ok=0
        [[ "$sha" == "$g_sha" || "$g_sha" == "-" || "$sha" == "-" ]] || ok=0
        if [[ "$ok" == 1 ]]; then
            printf '  OK %s %s — unrouted=%s unconnected=%s sha=%s\n' \
                "$(slug "$dsn")" "$modes" "$unrouted" "$unconnected" "$sha" >&2
        else
            printf '  FAIL %s %s — unrouted %s vs golden %s; unconnected %s vs %s; sha %s vs %s\n' \
                "$(slug "$dsn")" "$modes" "$unrouted" "$g_unrouted" "$unconnected" "$g_unconn" "$sha" "$g_sha" >&2
            FAILURES=$((FAILURES + 1))
        fi
    done
    [[ "$FAILURES" == 0 ]] || { echo "$FAILURES golden mismatch(es)" >&2; exit 1; }
    echo "all golden rows match" >&2
fi
