#!/usr/bin/env python3
"""Convert Codex rollout JSONL logs to Typora-friendly Markdown.

Default behavior:
- export conversation messages only (user + assistant)
- keep text + timestamp
- hide detailed tool execution logs unless explicitly requested
- normalize common math delimiters for Typora
- recover common UTF-8/GBK mojibake in Chinese text
- shorten markdown link labels to filename only
"""

from __future__ import annotations

import argparse
import json
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterable, List, Tuple


Record = Dict[str, str]
FENCE_RE = re.compile(r"^`{3,}")
FILE_LINK_RE = re.compile(r"\[([^\]]+)\]\(([^)]+)\)")

# Common mojibake artifacts when UTF-8 Chinese was decoded as GBK/cp936.
MOJIBAKE_TOKENS = (
    "锛",
    "銆",
    "鈥",
    "鍙",
    "鍦",
    "鍚",
    "鏄",
    "鐨",
    "鎴",
    "浣",
    "璇",
    "闂",
    "绗",
    "缁",
    "閲",
    "妯",
    "锟",
    "�",
)

CHINESE_PUNCT = (
    "，",
    "。",
    "？",
    "！",
    "；",
    "：",
    "（",
    "）",
    "、",
    "《",
    "》",
    "“",
    "”",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert Codex rollout JSONL to Typora-friendly Markdown."
    )
    parser.add_argument("input", type=Path, help="Path to rollout-*.jsonl")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output Markdown file path (default: same name with .md)",
    )
    parser.add_argument(
        "--include-developer",
        action="store_true",
        help="Include developer-role messages.",
    )
    parser.add_argument(
        "--include-system",
        action="store_true",
        help="Include system-role messages.",
    )
    parser.add_argument(
        "--include-tools",
        action="store_true",
        help="Include tool call/tool output records.",
    )
    parser.add_argument(
        "--include-reasoning",
        action="store_true",
        help="Include reasoning summaries when present.",
    )
    parser.add_argument(
        "--no-math-normalize",
        action="store_true",
        help="Do not normalize \\(...\\)/\\[...\\] math delimiters.",
    )
    parser.add_argument(
        "--no-mojibake-fix",
        action="store_true",
        help="Disable mojibake recovery heuristic.",
    )
    parser.add_argument(
        "--keep-link-labels",
        action="store_true",
        help="Keep original markdown link labels.",
    )
    return parser.parse_args()


def typora_math_normalize(text: str) -> str:
    """Convert common LaTeX delimiters to Typora-friendly $ / $$ forms."""
    if not text:
        return text

    text = re.sub(
        r"\\\[(.+?)\\\]",
        lambda m: "$$\n" + m.group(1).strip("\n") + "\n$$",
        text,
        flags=re.DOTALL,
    )
    text = re.sub(
        r"\\\((.+?)\\\)",
        lambda m: "$" + m.group(1).replace("\n", " ").strip() + "$",
        text,
        flags=re.DOTALL,
    )
    return text


def looks_like_math_line(line: str) -> bool:
    """Detect standalone equation-like LaTeX lines."""
    if not line:
        return False

    if re.match(r"^\s*([-*+]\s+|\d+\.\s+|#{1,6}\s+)", line):
        return False
    if re.search(r"[A-Za-z]:\\", line):
        return False
    if "$" in line:
        return False

    latex_signal = re.search(
        r"\\(mathbb|mathbf|begin|end|in\b|times\b|sum\b|tau\b|alpha\b|beta\b|lambda\b|cdot\b|leq?\b|geq?\b|min\b|max\b)",
        line,
    )
    equation_shape = any(token in line for token in ("=", "^", "_", "{", "}"))
    return bool(latex_signal and equation_shape)


def wrap_standalone_math_lines(text: str) -> str:
    """Wrap standalone equation-like lines into $$ blocks."""
    if not text:
        return text

    out: List[str] = []
    in_code = False
    for line in text.splitlines():
        stripped = line.strip()
        if FENCE_RE.match(stripped):
            in_code = not in_code
            out.append(line)
            continue

        if in_code:
            out.append(line)
            continue

        if looks_like_math_line(stripped):
            out.append("$$")
            out.append(stripped)
            out.append("$$")
        else:
            out.append(line)

    return "\n".join(out)


def readability_score(text: str) -> float:
    """Simple score used to decide whether recovered text is better."""
    punct_hits = sum(text.count(tok) for tok in CHINESE_PUNCT)
    mojibake_hits = sum(text.count(tok) for tok in MOJIBAKE_TOKENS)
    replacement_hits = text.count("�")
    return punct_hits * 2.0 - mojibake_hits * 2.5 - replacement_hits * 4.0


def recover_mojibake(text: str) -> str:
    """Recover common Chinese mojibake when conversion improves readability."""
    if not text:
        return text

    hint_hits = sum(text.count(tok) for tok in MOJIBAKE_TOKENS[:-1])
    if hint_hits < 2:
        return text

    best = text
    best_score = readability_score(text)

    for codec in ("gb18030", "gbk", "cp936"):
        for enc_err, dec_err in (
            ("strict", "strict"),
            ("ignore", "ignore"),
            ("replace", "replace"),
        ):
            try:
                candidate = text.encode(codec, errors=enc_err).decode(
                    "utf-8", errors=dec_err
                )
            except UnicodeError:
                continue

            if not candidate or candidate == text:
                continue

            candidate_score = readability_score(candidate)
            # Require meaningful improvement to avoid damaging already-correct text.
            if candidate_score > best_score + 2.0:
                best = candidate
                best_score = candidate_score

                # Sometimes one more pass continues to improve mixed-garbled text.
                try:
                    candidate2 = candidate.encode(codec, errors="ignore").decode(
                        "utf-8", errors="ignore"
                    )
                except UnicodeError:
                    candidate2 = candidate
                score2 = readability_score(candidate2)
                if candidate2 and score2 > best_score + 1.0:
                    best = candidate2
                    best_score = score2

    return best


def extract_filename_from_target(target: str) -> str:
    """Extract filename when link target looks like a file path."""
    t = target.strip()
    if t.startswith("<") and t.endswith(">"):
        t = t[1:-1]

    t = t.split("#", 1)[0].split("?", 1)[0]
    t = t.replace("\\", "/").rstrip("/")
    if not t or "/" not in t:
        return ""

    name = t.rsplit("/", 1)[-1]
    name = re.sub(r":\d+(?::\d+)?$", "", name)
    if re.search(r"\.[A-Za-z0-9]{1,10}$", name):
        return name
    return ""


def simplify_link_labels(text: str) -> str:
    """Shorten markdown link labels to filename only (outside code fences)."""
    if not text:
        return text

    def replace_link(match: re.Match[str]) -> str:
        target = match.group(2)
        filename = extract_filename_from_target(target)
        if not filename:
            return match.group(0)
        return f"[{filename}]({target})"

    out: List[str] = []
    in_code = False
    for line in text.splitlines():
        stripped = line.strip()
        if FENCE_RE.match(stripped):
            in_code = not in_code
            out.append(line)
            continue

        if in_code:
            out.append(line)
            continue

        out.append(FILE_LINK_RE.sub(replace_link, line))

    return "\n".join(out)


def parse_message_text(content: object) -> str:
    """Extract text from a message content list."""
    if not isinstance(content, list):
        return ""

    parts: List[str] = []
    for item in content:
        if not isinstance(item, dict):
            continue
        kind = item.get("type", "")
        if kind in {"input_text", "output_text", "text"}:
            value = item.get("text", "")
            if isinstance(value, str):
                parts.append(value)
        elif "text" in item and isinstance(item.get("text"), str):
            parts.append(item["text"])

    return "\n".join(part for part in parts if part)


def maybe_pretty_json(text: str) -> str:
    """Pretty-print JSON text when possible."""
    stripped = text.strip()
    if not stripped:
        return ""
    try:
        obj = json.loads(stripped)
    except Exception:
        return text
    return json.dumps(obj, ensure_ascii=False, indent=2)


def iter_records(
    jsonl_path: Path,
    include_developer: bool,
    include_system: bool,
    include_tools: bool,
    include_reasoning: bool,
    normalize_math: bool,
    fix_mojibake: bool,
    shorten_link_labels: bool,
) -> Iterable[Record]:
    """Iterate normalized transcript records from JSONL."""
    call_map: Dict[str, Tuple[str, str]] = {}

    allowed_roles = {"user", "assistant"}
    if include_developer:
        allowed_roles.add("developer")
    if include_system:
        allowed_roles.add("system")

    with jsonl_path.open("r", encoding="utf-8") as f:
        for line_no, raw_line in enumerate(f, start=1):
            line = raw_line.strip()
            if not line:
                continue

            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                # Keep parse errors visible only when tool logs are requested.
                if include_tools:
                    yield {
                        "kind": "parse_error",
                        "ts": "",
                        "title": f"json parse error at line {line_no}",
                        "body": line,
                    }
                continue

            if obj.get("type") != "response_item":
                continue

            ts = str(obj.get("timestamp", ""))
            payload = obj.get("payload", {})
            if not isinstance(payload, dict):
                continue

            ptype = payload.get("type")

            if ptype == "message":
                role = str(payload.get("role", "unknown"))
                if role not in allowed_roles:
                    continue

                text = parse_message_text(payload.get("content", []))
                if fix_mojibake:
                    text = recover_mojibake(text)
                if normalize_math:
                    text = typora_math_normalize(text)
                    text = wrap_standalone_math_lines(text)
                if shorten_link_labels:
                    text = simplify_link_labels(text)

                if not text.strip():
                    continue

                yield {
                    "kind": "message",
                    "ts": ts,
                    "title": role,
                    "body": text,
                }

            elif ptype == "function_call" and include_tools:
                name = str(payload.get("name", "tool"))
                arguments = str(payload.get("arguments", ""))
                call_id = str(payload.get("call_id", ""))
                call_map[call_id] = (name, arguments)

                if fix_mojibake:
                    arguments = recover_mojibake(arguments)

                yield {
                    "kind": "tool_call",
                    "ts": ts,
                    "title": f"tool call: {name}",
                    "body": maybe_pretty_json(arguments),
                }

            elif ptype == "function_call_output" and include_tools:
                call_id = str(payload.get("call_id", ""))
                output = str(payload.get("output", ""))
                name = call_map.get(call_id, ("tool", ""))[0]

                if fix_mojibake:
                    output = recover_mojibake(output)
                if normalize_math:
                    output = typora_math_normalize(output)
                    output = wrap_standalone_math_lines(output)

                yield {
                    "kind": "tool_output",
                    "ts": ts,
                    "title": f"tool output: {name}",
                    "body": output,
                }

            elif ptype == "reasoning" and include_reasoning:
                summary = payload.get("summary", [])
                lines: List[str] = []
                if isinstance(summary, list):
                    for item in summary:
                        if isinstance(item, dict) and isinstance(item.get("text"), str):
                            lines.append(item["text"])
                text = "\n".join(lines).strip()
                if text:
                    if fix_mojibake:
                        text = recover_mojibake(text)
                    yield {
                        "kind": "reasoning",
                        "ts": ts,
                        "title": "reasoning summary",
                        "body": text,
                    }


def format_timestamp(ts: str) -> str:
    """Normalize ISO timestamp for display."""
    raw = ts.strip()
    if not raw:
        return ""

    try:
        dt = datetime.fromisoformat(raw.replace("Z", "+00:00"))
    except ValueError:
        return raw

    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)

    return dt.astimezone(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")


def render_markdown(source: Path, records: List[Record]) -> str:
    """Render normalized transcript records to Markdown."""
    lines: List[str] = []
    lines.append("# Codex Session Transcript")
    lines.append("")
    lines.append(f"- Source: `{source}`")
    lines.append(
        f"- Generated: `{datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M:%S UTC')}`"
    )
    lines.append(f"- Records: `{len(records)}`")
    lines.append("")
    lines.append("---")
    lines.append("")

    for idx, rec in enumerate(records, start=1):
        kind = rec.get("kind", "message")
        title = rec.get("title", "message")
        body = rec.get("body", "")
        ts = format_timestamp(rec.get("ts", ""))

        lines.append(f"## {idx:04d}. {title}")
        if ts:
            lines.append("")
            lines.append(f"- Timestamp: `{ts}`")

        lines.append("")
        if kind == "tool_call":
            lines.append("```json")
            lines.append(body.rstrip("\n"))
            lines.append("```")
        elif kind in {"tool_output", "parse_error"}:
            lines.append("```text")
            lines.append(body.rstrip("\n"))
            lines.append("```")
        else:
            lines.append(body.rstrip("\n"))

        lines.append("")

    return "\n".join(lines).rstrip() + "\n"


def main() -> int:
    args = parse_args()

    input_path = args.input
    if not input_path.exists():
        raise SystemExit(f"Input file not found: {input_path}")

    output_path = args.output or input_path.with_suffix(".md")

    records = list(
        iter_records(
            jsonl_path=input_path,
            include_developer=args.include_developer,
            include_system=args.include_system,
            include_tools=args.include_tools,
            include_reasoning=args.include_reasoning,
            normalize_math=not args.no_math_normalize,
            fix_mojibake=not args.no_mojibake_fix,
            shorten_link_labels=not args.keep_link_labels,
        )
    )

    markdown = render_markdown(input_path, records)
    output_path.write_text(markdown, encoding="utf-8")

    print(f"Wrote: {output_path}")
    print(f"Records: {len(records)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
