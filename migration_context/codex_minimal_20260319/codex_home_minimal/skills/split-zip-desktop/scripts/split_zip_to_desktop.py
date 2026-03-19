#!/usr/bin/env python3
"""Split selected files into zip archives on Desktop with a per-archive file cap."""

import argparse
import datetime as dt
import sys
import zipfile
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Package selected files from the current working folder into "
            "multiple zip archives and place them in a new Desktop folder."
        )
    )
    parser.add_argument(
        "paths",
        nargs="*",
        help="File or directory paths to include (relative to current folder by default).",
    )
    parser.add_argument(
        "--from-list",
        dest="from_list",
        help="Text file containing one path per line (blank lines and #comments ignored).",
    )
    parser.add_argument(
        "--all-files",
        action="store_true",
        help="Include all files under the current working folder recursively.",
    )
    parser.add_argument(
        "--max-files",
        type=int,
        default=9,
        help="Maximum file count per zip archive (default: 9).",
    )
    parser.add_argument(
        "--base-name",
        default="src",
        help="Base name used for zip files (default: src).",
    )
    parser.add_argument(
        "--output-name",
        default="",
        help="Optional explicit Desktop output folder name.",
    )
    parser.add_argument(
        "--desktop-dir",
        default="",
        help="Optional desktop directory override for testing.",
    )
    return parser.parse_args()


def load_list_file(list_path: Path) -> List[str]:
    if not list_path.exists() or not list_path.is_file():
        raise ValueError(f"List file not found: {list_path}")

    entries: List[str] = []
    for raw_line in list_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        entries.append(line)
    return entries


def assert_under_root(path: Path, root: Path) -> None:
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"Path is outside current working folder: {path}") from exc


def collect_files_from_path(path: Path) -> List[Path]:
    if path.is_file():
        return [path]
    if path.is_dir():
        return sorted([p for p in path.rglob("*") if p.is_file()])
    return []


def resolve_selected_files(args: argparse.Namespace, cwd: Path) -> List[Path]:
    raw_paths: List[str] = list(args.paths)

    if args.from_list:
        list_file = Path(args.from_list)
        if not list_file.is_absolute():
            list_file = (cwd / list_file).resolve()
        raw_paths.extend(load_list_file(list_file))

    if args.all_files:
        raw_paths.append(".")

    if not raw_paths:
        raise ValueError("No input paths provided. Use paths, --from-list, or --all-files.")

    seen = set()
    selected: List[Path] = []

    for raw in raw_paths:
        candidate = Path(raw)
        if not candidate.is_absolute():
            candidate = (cwd / candidate).resolve()
        else:
            candidate = candidate.resolve()

        if not candidate.exists():
            raise ValueError(f"Input path not found: {raw}")

        assert_under_root(candidate, cwd)
        for file_path in collect_files_from_path(candidate):
            assert_under_root(file_path, cwd)
            key = str(file_path)
            if key in seen:
                continue
            seen.add(key)
            selected.append(file_path)

    selected.sort(key=lambda p: p.relative_to(cwd).as_posix())
    if not selected:
        raise ValueError("No files resolved from the provided inputs.")
    return selected


def chunk_files(files: Sequence[Path], max_files: int) -> Iterable[Sequence[Path]]:
    for i in range(0, len(files), max_files):
        yield files[i : i + max_files]


def ensure_unique_folder(path: Path) -> Path:
    if not path.exists():
        return path
    idx = 1
    while True:
        candidate = Path(f"{path}_{idx:02d}")
        if not candidate.exists():
            return candidate
        idx += 1


def create_zip(zip_path: Path, files: Sequence[Path], root: Path) -> int:
    with zipfile.ZipFile(zip_path, mode="w", compression=zipfile.ZIP_DEFLATED) as zf:
        for fp in files:
            arcname = fp.relative_to(root).as_posix()
            zf.write(fp, arcname)

    with zipfile.ZipFile(zip_path, mode="r") as zf:
        count = sum(1 for info in zf.infolist() if not info.is_dir())
    return count


def build_output_folder(args: argparse.Namespace, desktop_dir: Path) -> Path:
    if args.output_name:
        folder_name = args.output_name
    else:
        timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        folder_name = f"{args.base_name}_bundle_{timestamp}"
    return ensure_unique_folder(desktop_dir / folder_name)


def write_index(
    index_path: Path,
    cwd: Path,
    max_files: int,
    zip_records: Sequence[Tuple[str, int, Sequence[Path]]],
) -> None:
    lines: List[str] = []
    lines.append(f"Source root: {cwd}")
    lines.append(f"Max files per zip: {max_files}")
    total_files = sum(rec[1] for rec in zip_records)
    lines.append(f"Total files: {total_files}")
    lines.append("")
    lines.append("Packages:")

    for zip_name, file_count, files in zip_records:
        lines.append(f"- {zip_name} | files={file_count}")
        for fp in files:
            lines.append(f"  - {fp.relative_to(cwd).as_posix()}")

    index_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()

    if args.max_files < 1:
        print("ERROR: --max-files must be >= 1", file=sys.stderr)
        return 2

    cwd = Path.cwd().resolve()

    if args.desktop_dir:
        desktop_dir = Path(args.desktop_dir).expanduser().resolve()
    else:
        desktop_dir = (Path.home() / "Desktop").resolve()
        if not desktop_dir.exists():
            desktop_dir = Path.home().resolve()

    if not desktop_dir.exists():
        print(f"ERROR: Desktop directory not found: {desktop_dir}", file=sys.stderr)
        return 2

    try:
        selected_files = resolve_selected_files(args, cwd)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    output_folder = build_output_folder(args, desktop_dir)
    output_folder.mkdir(parents=True, exist_ok=False)

    chunks = list(chunk_files(selected_files, args.max_files))
    multi_part = len(chunks) > 1

    zip_records: List[Tuple[str, int, Sequence[Path]]] = []
    for idx, chunk in enumerate(chunks, start=1):
        if multi_part:
            zip_name = f"{args.base_name}_{idx:02d}.zip"
        else:
            zip_name = f"{args.base_name}.zip"

        zip_path = output_folder / zip_name
        file_count = create_zip(zip_path, chunk, cwd)
        if file_count > args.max_files:
            print(
                f"ERROR: zip {zip_name} exceeds --max-files (count={file_count})",
                file=sys.stderr,
            )
            return 3
        zip_records.append((zip_name, file_count, chunk))

    write_index(output_folder / "INDEX.txt", cwd, args.max_files, zip_records)

    print(f"OUTPUT_FOLDER={output_folder}")
    for zip_name, file_count, _ in zip_records:
        print(f"{zip_name}\tfiles={file_count}")
    print(f"INDEX_FILE={output_folder / 'INDEX.txt'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
