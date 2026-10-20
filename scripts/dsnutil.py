#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Topola contributors
#
# SPDX-License-Identifier: MIT

"""Shared Specctra DSN fixture normalisation.

Implements the parser workarounds documented in tests/external/README.md:

- quoted pcb names containing spaces fail to parse -> the pcb name on
  the first line is replaced with a plain slug;
- ``(class ''`` / ``(class ""`` empty class names are rejected -> they
  are renamed to a placeholder;
- ``(wiring`` is a required scope -> an empty one is appended when the
  file lacks it;
- fixtures embedding a routing solution get a problem variant with an
  emptied ``(wiring)`` scope.

Importable (used by kicad_to_dsn.py under the pcbnew interpreter) and
stdlib-only.
"""

import re
from pathlib import Path

RE_PCB_NAME = re.compile(r"\(pcb\s+(\"(?:[^\"\\]|\\.)*\"|'(?:[^']*)'|[^\s()]+)", re.I)
RE_EMPTY_CLASS = re.compile(r"\(class(\s+)(''|\"\")")
RE_WIRING = re.compile(r"\(wiring\b")
RE_PARSER = re.compile(r"\(parser\b")
RE_SPACED_STRING = re.compile(r"\"[^\"]*\s[^\"]*\"")
RE_NET_DECL = re.compile(r"\(net\s+")
RE_TYPE_ATTR = re.compile(r"\(\s*type\s+[^()]*\)")


def scope_end(text: str, start: int) -> int:
    """Index just past the closing paren of the scope opened at ``start``."""
    depth = 0
    i = start
    in_str = False
    while i < len(text):
        c = text[i]
        if in_str:
            if c == "\\":
                i += 1
            elif c == '"':
                in_str = False
        elif c == '"':
            in_str = True
        elif c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                return i + 1
        i += 1
    raise ValueError("unbalanced parens")


def normalize_pcb_name(text: str, slug: str) -> str:
    """Replace the pcb name on line 1 with an unquoted slug."""
    return RE_PCB_NAME.sub(f"(pcb {slug}", text, count=1)


def fix_empty_class_names(text: str) -> str:
    return RE_EMPTY_CLASS.sub(r"(class\1unnamed_class", text)


def ensure_wiring_scope(text: str) -> str:
    if RE_WIRING.search(text):
        return text
    stripped = text.rstrip()
    if not stripped.endswith(")"):
        raise ValueError("dsn does not end with )")
    return stripped[:-1] + "  (wiring)\n)\n"


def empty_wiring_scopes(text: str) -> str:
    """Empty the contents of every ``(wiring …)`` scope."""
    out = []
    pos = 0
    for m in RE_WIRING.finditer(text):
        end = scope_end(text, m.start())
        out.append(text[pos : m.start()])
        out.append("(wiring)")
        pos = end
    if pos == 0:
        return text
    out.append(text[pos:])
    return "".join(out)


def ensure_parser_quoting(text: str) -> str:
    """Files using ``"``-quoted tokens need ``(string_quote ")`` and,
    when the quoted text contains spaces, ``(space_in_quoted_tokens
    on)``. Both are read in order from the ``(parser`` scope, so they
    are injected immediately after ``(parser`` — ahead of ``host_cad``
    entries that already need them."""
    m = RE_PARSER.search(text)
    if not m:
        return text
    inject = ""
    if '"' in text and "string_quote" not in text:
        inject += '\n    (string_quote ")'
    if RE_SPACED_STRING.search(text) and "space_in_quoted_tokens" not in text:
        inject += "\n    (space_in_quoted_tokens on)"
    if not inject:
        return text
    pos = m.end()
    return text[:pos] + inject + text[pos:]


def strip_net_type_attrs(text: str) -> str:
    """Drop ``(type …)`` children of ``(net NAME …)`` declarations —
    specctra-core has no grammar slot for them. Only applies to net
    declarations (scopes containing ``(pins``); wire/layer/rule ``type``
    forms are untouched. NOTE: protect/fix semantics are lost — nets
    marked protected upstream become ordinarily routable."""
    out = []
    pos = 0
    for m in RE_NET_DECL.finditer(text):
        end = scope_end(text, m.start())
        scope = text[m.start() : end]
        if "(pins" not in scope or "(type" not in scope:
            continue
        cleaned = RE_TYPE_ATTR.sub("", scope)
        if cleaned == scope:
            continue
        out.append(text[pos : m.start()])
        out.append(cleaned)
        pos = end
    out.append(text[pos:])
    return "".join(out)


def normalize(text: str, slug: str) -> str:
    text = normalize_pcb_name(text, slug)
    text = fix_empty_class_names(text)
    text = ensure_parser_quoting(text)
    text = strip_net_type_attrs(text)
    return ensure_wiring_scope(text)


def write_fixture(src_text: str, slug: str, outdir: Path, strip_solved: bool = True):
    """Write ``<slug>.dsn`` (problem) and ``<slug>_solved.dsn`` (reference)."""
    outdir.mkdir(parents=True, exist_ok=True)
    normalized = normalize(src_text, slug)
    (outdir / f"{slug}_solved.dsn").write_text(normalized)
    problem = empty_wiring_scopes(normalized)
    (outdir / f"{slug}.dsn").write_text(problem)
    return outdir / f"{slug}.dsn"
