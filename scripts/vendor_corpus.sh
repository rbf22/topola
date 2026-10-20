#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Topola contributors
#
# SPDX-License-Identifier: MIT

# Vendor the boards listed in bench/corpus.tsv into tests/external/.
#
#   pcbench rows     -> tests/external/pcbench/<slug>/   (kicad_to_dsn.py)
#   freerouting rows -> tests/external/freerouting/<slug>/ (strip_wiring.py)
#
# Usage:
#   scripts/vendor_corpus.sh            vendor every manifest row
#   ROW_FILTER=wave scripts/vendor_corpus.sh   vendor matching slugs only
#
# Expects the corpora under bench/corpora-src/ (scripts/fetch_corpora.sh).

set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

SRC="$ROOT/bench/corpora-src"
MANIFEST="$ROOT/bench/corpus.tsv"
PY_KICAD="${PY_KICAD:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}"

[[ -d "$SRC" ]] || { echo "corpora not fetched — run scripts/fetch_corpora.sh" >&2; exit 1; }
[[ -x "$PY_KICAD" ]] || { echo "KiCad pcbnew python not found at $PY_KICAD (set PY_KICAD)" >&2; exit 1; }

tail -n +2 "$MANIFEST" | grep -v '^#' | grep -v '^$' | while IFS=$'\t' read -r slug source src layers nets budget class notes; do
    [[ -n "${ROW_FILTER:-}" && "$slug" != *"$ROW_FILTER"* ]] && continue
    src_path="$SRC/$src"
    [[ -f "$src_path" ]] || { echo "MISSING src for $slug: $src_path" >&2; continue; }
    case "$source" in
        pcbench)
            out="$ROOT/tests/external/pcbench/$slug"
            echo "== pcbench $slug"
            "$PY_KICAD" "$ROOT/scripts/kicad_to_dsn.py" "$src_path" "$slug" "$out" \
                || echo "FAILED $slug" >&2
            ;;
        freerouting)
            out="$ROOT/tests/external/freerouting/$slug"
            echo "== freerouting $slug"
            python3 "$ROOT/scripts/strip_wiring.py" "$src_path" "$slug" "$out" \
                || echo "FAILED $slug" >&2
            ;;
        *) echo "unknown source $source for $slug" >&2 ;;
    esac
done
