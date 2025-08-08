#!/usr/bin/env python3
"""
find_deprecated_dc_paths.py
Run from the root of your Isaac Sim checkout:

    python find_deprecated_dc_paths.py

It will scan all premake5.lua files under source/extensions
and report ones that still include the deprecated dynamic-control path.
"""

import os
import re
from pathlib import Path

# ---------------------------------------------------------------------
# Configuration – change ROOT if you run the script from elsewhere
# ---------------------------------------------------------------------
ROOT = Path(".").resolve()                 # repo root (cwd)
EXTS_DIR = ROOT / "source" / "extensions"  # where all extensions live
BAD_PATTERN = re.compile(
    r"source\s*/\s*deprecated\s*/\s*omni\.isaac\.dynamic_control\s*/include",
    re.IGNORECASE,
)

def scan_file(path: Path) -> list[str]:
    """Return list of *matching* lines that contain the deprecated path."""
    matches = []
    try:
        with path.open("r", encoding="utf-8") as f:
            for line_no, line in enumerate(f, 1):
                if BAD_PATTERN.search(line):
                    matches.append(f"  line {line_no}: {line.rstrip()}")
    except UnicodeDecodeError:
        # skip binary or odd-encoded files
        pass
    return matches

def main():
    offenders: dict[Path, list[str]] = {}

    for premake in EXTS_DIR.rglob("premake5.lua"):
        hits = scan_file(premake)
        if hits:
            offenders[premake] = hits

    if not offenders:
        print("✅ No premake5.lua files still reference the deprecated dynamic-control path.")
        return

    print("❌ Found deprecated dynamic-control include paths in the following premake5.lua files:\n")
    for path, lines in offenders.items():
        # extension folder is two levels up from the premake file
        ext_name = path.parent.name
        print(f"Extension: {ext_name}")
        print(f"File     : {path.relative_to(ROOT)}")
        print("\n".join(lines))
        print("-" * 60)

if __name__ == "__main__":
    main()
