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

source "$ROOT/scripts/bench_lib.sh"
ensure_binary

# ROW := <dsn path>|<mode flag string>|<step budget>
ROWS=(
    "tests/unilayer/0603_breakout/0603_breakout.dsn||20000"
    "tests/unilayer/0603_breakout/0603_breakout.dsn|--surgical-ripup|20000"
    "tests/unilayer/4x4_1206_led_matrix_breakout/4x4_1206_led_matrix_breakout.dsn||50000"
    "tests/unilayer/red_led_heart/red_led_heart.dsn||60000"
    "tests/unilayer/red_led_heart/red_led_heart.dsn|--critical-first|60000"
    "tests/unilayer/red_led_heart/red_led_heart.dsn|--multilayer|60000"
    "tests/unilayer/red_led_heart/red_led_heart.dsn|--critical-first --multilayer|60000"
    "tests/unilayer/red_led_heart/red_led_heart.dsn|--multilayer --anneal --anneal-mode continuous|60000"
    "tests/unilayer/red_led_heart/red_led_heart.dsn|--multilayer --anneal --anneal-mode discrete|60000"
    "tests/unilayer/critical_corridor/critical_corridor.dsn|--critical-first|30000"
    "tests/external/optoisolator_4n35/optoisolator_4n35.dsn||30000"
    "tests/external/fasttest/fasttest.dsn|--multilayer|30000"
    "tests/multilayer/astable_multivibrator/astable_multivibrator.dsn|--multilayer|30000"
    "tests/multilayer/astable_multivibrator/astable_multivibrator.dsn|--multilayer --anneal --anneal-mode continuous|30000"
    "tests/multilayer/astable_multivibrator/astable_multivibrator.dsn|--multilayer --anneal --anneal-mode discrete|30000"
    "tests/multilayer/prerouted_lm317_breakout/unrouted_lm317_breakout.dsn|--multilayer|30000"
)

# Corpus rows: one validated config per vendored board, chosen from the
# phase-2 sweep as each board's min-unrouted config at seed 0 (issue208
# uses anneal-discrete instead of its winning anneal-continuous row —
# 24min wall is impractical for a regression matrix).
# Budgets match bench/corpus.tsv. See bench/README.md for the table.
ROWS+=(
    "tests/external/pcbench/arf154/arf154.dsn|--multilayer --anneal --anneal-mode discrete --preprobe|30000"
    "tests/external/pcbench/beer_gauge/beer_gauge.dsn|--multilayer --anneal --anneal-mode discrete --preprobe|40000"
    "tests/external/pcbench/esp12_breakout/esp12_breakout.dsn|--multilayer --anneal --anneal-mode discrete|30000"
    "tests/external/pcbench/lvds2tmds/lvds2tmds.dsn|--multilayer --anneal --anneal-mode discrete|30000"
    "tests/external/pcbench/miniboard_stm32/miniboard_stm32.dsn|--multilayer --anneal --anneal-mode discrete|30000"
    "tests/external/pcbench/modular_bus/modular_bus.dsn|--multilayer --anneal --anneal-mode discrete|30000"
    "tests/external/pcbench/opamp_bench_supply/opamp_bench_supply.dsn|--corridor-presort|30000"
    "tests/external/pcbench/poncho_fpga/poncho_fpga.dsn|--surgical-ripup|60000"
    "tests/external/pcbench/prog_rig/prog_rig.dsn|--surgical-ripup|150000"
    "tests/external/pcbench/reworkctf/reworkctf.dsn|--multilayer --surgical-ripup|50000"
    "tests/external/pcbench/sensors_96boards/sensors_96boards.dsn|--multilayer --surgical-ripup|40000"
    "tests/external/pcbench/sizif128/sizif128.dsn|--corridor-presort|50000"
    "tests/external/pcbench/smartwatch/smartwatch.dsn|--multilayer --anneal --anneal-mode discrete --preprobe|40000"
    "tests/external/pcbench/tinytracker/tinytracker.dsn|--multilayer --anneal --anneal-mode discrete --preprobe|30000"
    "tests/external/pcbench/uprogrammer/uprogrammer.dsn|--multilayer --anneal --anneal-mode discrete --surgical-ripup|30000"
    "tests/external/pcbench/vc4000multirom/vc4000multirom.dsn|--multilayer --escape-stubs|40000"
    "tests/external/freerouting/avionics_hub/avionics_hub.dsn|--multilayer --escape-stubs|60000"
    "tests/external/freerouting/board_unrouted/board_unrouted.dsn|--multilayer --anneal --anneal-mode discrete|110000"
    "tests/external/freerouting/cnh_tester/cnh_tester.dsn|--preprobe|40000"
    "tests/external/freerouting/contribution/contribution.dsn|--multilayer --anneal --anneal-mode discrete|150000"
    "tests/external/freerouting/fq101/fq101.dsn|--multilayer --anneal --anneal-mode discrete --preprobe|60000"
    "tests/external/freerouting/gp8b/gp8b.dsn|--multilayer --anneal --anneal-mode discrete|110000"
    "tests/external/freerouting/issue208/issue208.dsn|--multilayer --anneal --anneal-mode discrete|60000"
    "tests/external/freerouting/issue684/issue684.dsn|--multilayer --anneal --anneal-mode discrete|30000"
    "tests/external/freerouting/j2_reference/j2_reference.dsn|--multilayer --anneal --anneal-mode discrete --preprobe|30000"
    "tests/external/freerouting/linepcb/linepcb.dsn|--multilayer --anneal --anneal-mode discrete|40000"
    "tests/external/freerouting/logicboard/logicboard.dsn|--multilayer --anneal --anneal-mode discrete --preprobe|150000"
    "tests/external/freerouting/myboard/myboard.dsn|--multilayer --anneal --anneal-mode discrete --surgical-ripup|30000"
    "tests/external/freerouting/relaymodule/relaymodule.dsn|--multilayer --escape-stubs|30000"
    "tests/external/freerouting/smoothieboard/smoothieboard.dsn|--multilayer --surgical-ripup|120000"
    "tests/external/freerouting/tairakb/tairakb.dsn|--multilayer --surgical-ripup|30000"
    "tests/external/freerouting/wavefolder/wavefolder.dsn|--multilayer|30000"
)

# External boards outside the repo (set env to include):
[[ -n "${CANDE_DSN:-}" && -f "${CANDE_DSN:-}" ]] && ROWS+=("$CANDE_DSN||100000")
[[ -n "${CANDE_DSN:-}" && -f "${CANDE_DSN:-}" ]] && ROWS+=("$CANDE_DSN|--critical-first|100000")
[[ -n "${CANDE_DSN:-}" && -f "${CANDE_DSN:-}" ]] && ROWS+=("$CANDE_DSN|--multilayer|100000")
[[ -n "${CANDE_DSN:-}" && -f "${CANDE_DSN:-}" ]] && ROWS+=("$CANDE_DSN|--multilayer --anneal --anneal-mode continuous --anneal-iterations 60|100000")
[[ -n "${CANDE_DSN:-}" && -f "${CANDE_DSN:-}" ]] && ROWS+=("$CANDE_DSN|--multilayer --anneal --anneal-mode discrete|100000")

printf '%-36s %-58s %7s %9s %11s %8s %8s  %s\n' \
    "board" "mode" "steps" "unrouted" "unconnected" "wall_s" "ses_sha" "status" >&2

RESULTS=()
FAILURES=0

for row in "${ROWS[@]}"; do
    IFS='|' read -r dsn modes budget <<<"$row"
    name="$(slug "$dsn")$(echo -n "$modes" | tr ' -' '__' | tr -d '\n')"
    [[ -n "${ROW_FILTER:-}" && "$name" != *"$ROW_FILTER"* ]] && continue
    if [[ ! -f "$dsn" ]]; then
        printf '%-36s %-58s %7s %9s %11s %8s %8s  %s\n' \
            "$name" "-" "-" "-" "-" "-" "-" "missing" >&2
        continue
    fi

    RESULTS+=("$(run_row "$dsn" "$modes" "$budget" "${SEED:-0}" "$name")")
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
