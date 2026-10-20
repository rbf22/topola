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

# Route-quality counters (I6): via placements, wire segs, total wire
# length in a .ses or a reference _solved.dsn. A "(via" line counts as
# a placement only when a coordinate follows the via name — the
# structure block's `(via "name" "name")` declaration is excluded.
# wlen is in the file's raw coordinate units — Topola never rescales
# on load and corpus `(resolution)` labels are unreliable (freerouting
# writes `mil 1000` on mm-scale files), so compare wlen only within a
# dsn/.ses pair, never across boards.
# Prints "vias|segs|wlen"; "-|-|" when the file is absent or empty.
wire_stats() {
    local f="$1"
    [[ -s "$f" ]] || { echo "-|-|-"; return; }
    awk '
        {
            has_wire = /\(wire/
            has_path = /\(path +[^ ]+ +[^ ]+ +-?[0-9]/
            if (has_wire) segs++
            if (/\(via +"?[^"() ]+"? +-?[0-9]/) vias++
            if (has_path) {
                # Wire paths only — boundary/keepout/geometry paths
                # carry no `(wire` opener. `counting` tags whether this
                # open path feeds `len` until its closing `)`.
                counting = (in_wire || has_wire)
                have_pt = 0
                if (counting) {
                    n = split($0, t, /[() ]+/)
                    for (i = 1; i <= n; i++) if (t[i] == "path") { p = i; break }
                    # Coordinate pairs end at the first non-numeric
                    # token — trailing `(net ...)(type route)` children
                    # must not pair with the last coordinate.
                    for (i = p + 3; i + 2 <= n; i += 2) {
                        if (t[i] !~ /^-?[0-9.]+$/ || t[i+1] !~ /^-?[0-9.]+$/) break
                        if (have_pt) {
                            dx = t[i] - px; dy = t[i+1] - py
                            len += sqrt(dx*dx + dy*dy)
                        }
                        px = t[i]; py = t[i+1]; have_pt = 1
                    }
                }
                # A `)` on this line closes the path; otherwise the
                # coordinates continue on the next line.
                open_path = ($0 !~ /\)/)
                if (!open_path) { have_pt = 0; counting = 0 }
                in_wire = 0
            } else if (open_path) {
                # Bare coordinate line continuing an open path.
                if (counting) {
                    m = split($0, c, /[ \t()]+/)
                    # Leading separators leave an empty c[1]; anchor
                    # pairing on the first real token.
                    for (i0 = 1; i0 <= m && c[i0] == ""; i0++);
                    for (i = i0; i + 1 <= m; i += 2) {
                        if (c[i] ~ /^-?[0-9.]+$/ && c[i+1] ~ /^-?[0-9.]+$/) {
                            dx = c[i] - px; dy = c[i+1] - py
                            len += sqrt(dx*dx + dy*dy)
                            px = c[i]; py = c[i+1]
                        }
                    }
                }
                if ($0 ~ /\)/) { open_path = 0; have_pt = 0; counting = 0 }
                in_wire = 0
            } else {
                in_wire = has_wire
            }
        }
        END { printf "%d|%d|%.0f\n", vias+0, segs+0, len }
    ' "$f"
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
#   |vias|segs|wlen|ref_vias|ref_segs|ref_wlen
# The last six are the I6 quality columns: Topola's output and the
# hand-routed `<stem>_solved.dsn` next to the input, when present.
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

    local qstats refstats
    qstats=$(wire_stats "$ses")
    refstats=$(wire_stats "${dsn%.dsn}_solved.dsn")

    printf '%-36s %-58s %7s %9s %11s %6s %8s %8s %8s  %s\n' \
        "$name" "$modes" "${steps:--}" "$unrouted" "$unconnected" \
        "$(cut -d'|' -f1 <<<"$qstats")" "$(cut -d'|' -f3 <<<"$qstats")" \
        "${wall:--}" "$sha" "$status" >&2
    printf '%s\n' "$dsn|$modes|$budget|$seed|$unrouted|$unconnected|$total|$sha|${wall:-}|$status|$qstats|$refstats"
}
