#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Topola contributors
#
# SPDX-License-Identifier: MIT

"""Index candidate boards in the fetched corpora.

Scans the freerouting fixtures dir (*.dsn) and the PCBench PCBs dir
(metadata.json + final.json per board) and emits a TSV with the fields
used for curation: layer count, net count, pin/pad count, embedded-wire
count, file size.

Usage:
    scripts/corpus_index.py [CORPORA_SRC_DIR] > bench/corpus_index.tsv

CORPORA_SRC_DIR defaults to bench/corpora-src relative to the repo root.
"""

import json
import re
import sys
from pathlib import Path

RE_LAYER = re.compile(r"^\s*\(layer\s+(\S+)", re.M)
RE_NET = re.compile(r"^\s*\(net\s+\S+", re.M)
RE_PIN = re.compile(r"^\s*\(pin\s+\S+", re.M)
RE_WIRE = re.compile(r"^\s*\(wire\b", re.M)


def index_dsn(path: Path) -> dict:
    text = path.read_text(errors="replace")
    return {
        "layers": len(RE_LAYER.findall(text)),
        "nets": len(RE_NET.findall(text)),
        "pins": len(RE_PIN.findall(text)),
        "wires": len(RE_WIRE.findall(text)),
        "size_kb": round(path.stat().st_size / 1024),
    }


def index_freerouting(fixtures: Path):
    for dsn in sorted(fixtures.rglob("*.dsn")):
        row = index_dsn(dsn)
        yield {
            "id": dsn.stem,
            "source": "freerouting",
            "path": dsn,
            **row,
        }


def index_pcbench(pcbs: Path):
    for board_dir in sorted(pcbs.iterdir()):
        meta_file = board_dir / "metadata.json"
        final_file = board_dir / "final.json"
        if not final_file.is_file():
            continue
        try:
            final = json.loads(final_file.read_text(errors="replace"))
        except json.JSONDecodeError:
            continue
        nets = final.get("nets", {})
        pins = sum(len(pads) for pads in nets.values())
        layers = len(final.get("layers", []))
        license_key = ""
        author = ""
        if meta_file.is_file():
            try:
                meta = json.loads(meta_file.read_text(errors="replace"))
                lic = meta.get("licenses")
                if isinstance(lic, dict):
                    license_key = lic.get("key", "")
                elif isinstance(lic, list):
                    license_key = ",".join(
                        item.get("key", "") for item in lic if isinstance(item, dict)
                    )
                author = meta.get("author", "")
            except json.JSONDecodeError:
                pass
        pcb = board_dir / "processed.kicad_pcb"
        yield {
            "id": board_dir.name,
            "source": "pcbench",
            "path": pcb,
            "layers": layers,
            "nets": len(nets),
            "pins": pins,
            "wires": 0,
            "size_kb": round(pcb.stat().st_size / 1024) if pcb.is_file() else 0,
            "license": license_key,
            "author": author,
        }


def main() -> None:
    root = Path(__file__).resolve().parent.parent
    src = Path(sys.argv[1]) if len(sys.argv) > 1 else root / "bench" / "corpora-src"

    fields = ["id", "source", "layers", "nets", "pins", "wires", "size_kb",
              "license", "author", "path"]
    print("\t".join(fields))

    fr = src / "freerouting" / "fixtures"
    if not fr.is_dir():
        fr = src / "freerouting" / "tests"
    if fr.is_dir():
        for row in index_freerouting(fr):
            print("\t".join(str(row.get(f, "")) for f in fields))
    else:
        print(f"warning: no freerouting fixtures dir under {src}", file=sys.stderr)

    pcbs = src / "PCBench" / "PCBs"
    if pcbs.is_dir():
        for row in index_pcbench(pcbs):
            print("\t".join(str(row.get(f, "")) for f in fields))
    else:
        print(f"warning: no PCBench PCBs dir under {src}", file=sys.stderr)


if __name__ == "__main__":
    main()
