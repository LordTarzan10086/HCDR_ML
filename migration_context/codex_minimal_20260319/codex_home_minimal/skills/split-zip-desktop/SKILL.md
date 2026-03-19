---
name: split-zip-desktop
description: Split and package specified files from the current working folder into multiple zip archives (max 9 files per zip) and place all archives in a new Desktop folder. Use when users ask for split zip delivery, per-archive file-count limits, src_01/src_02 naming, or Desktop-ready archive bundles.
---

# Split Zip Desktop

## Workflow

1. Collect the exact files or folders to include, relative to the current working folder.
2. Run `scripts/split_zip_to_desktop.py` with explicit paths, `--from-list`, or `--all-files`.
3. Keep `--max-files` at `9` unless the user explicitly asks for another limit.
4. Verify output zip counts from script output or `INDEX.txt`.
5. Return the Desktop output folder path and the zip filenames.

## Command Patterns

```bash
python "$HOME/.codex/skills/split-zip-desktop/scripts/split_zip_to_desktop.py" file1.m file2.csv dirA
```

```bash
python "$HOME/.codex/skills/split-zip-desktop/scripts/split_zip_to_desktop.py" --from-list selected_files.txt
```

```bash
python "$HOME/.codex/skills/split-zip-desktop/scripts/split_zip_to_desktop.py" --all-files --base-name src --max-files 9
```

## Script Behavior

- Include only the specified files. If a specified path is a directory, include files under it recursively.
- Require all selected files to stay under the current working folder.
- Create a new Desktop folder named `<base-name>_bundle_<timestamp>`.
- If selected file count is greater than `max-files`, name archives `src_01.zip`, `src_02.zip`, ...
- If selected file count is less than or equal to `max-files`, create a single archive `src.zip`.
- Write `INDEX.txt` in the Desktop output folder with archive/file mapping.

## Output Contract

The script prints:

- `OUTPUT_FOLDER=<absolute path>`
- One line per archive: `<zip_name>\tfiles=<count>`

Use these lines directly in the final response.

