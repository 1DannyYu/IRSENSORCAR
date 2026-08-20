#!/usr/bin/env python3
"""Validate the website inventory dataset.

Checks that image files exist, IDs are unique, English content is complete, and asset filenames
follow the `NNN_` naming rule described in `CONVENTIONS.md`.

    uv run python scripts/check_inventory_data.py
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DATA = REPO / "site" / "src" / "data" / "modules.json"
LANGS = ("en",)
# Required localized fields. Wiring tables (`raspberryPiWiring`, `arduinoWiring`, `stm32Wiring`)
# are optional in shape but expected for modules that document MCU hookup; some entries may be
# empty arrays for Pi-only accessories such as the M.2 HAT.
I18N_FIELDS = ("title", "desc", "specs", "codeSnippet", "raspberryPiWiring")
WIRING_FIELDS = ("raspberryPiWiring", "arduinoWiring", "stm32Wiring")
ASSET_NAME = re.compile(r"^\d{3}_[A-Za-z0-9_]+\.(jpg|png)$")


def load_modules() -> list[dict]:
    return json.loads(DATA.read_text())


def main() -> int:
    modules = load_modules()
    problems: list[str] = []

    seen_ids: set[str] = set()
    for m in modules:
        mid = m.get("id", "<missing id>")
        if mid in seen_ids:
            problems.append(f"{mid}: duplicate id")
        seen_ids.add(mid)

        for key in ("number", "name", "category", "tags", "images"):
            if not m.get(key):
                problems.append(f"{mid}: missing shared field {key}")

        for lang in LANGS:
            block = m.get("i18n", {}).get(lang)
            if not block:
                problems.append(f"{mid}: missing {lang} localization block")
                continue
            for field in I18N_FIELDS:
                if field == "raspberryPiWiring":
                    if field not in block or not isinstance(block[field], list) or not block[field]:
                        problems.append(f"{mid}: {lang}.{field} is missing or empty")
                elif not block.get(field):
                    problems.append(f"{mid}: {lang}.{field} is empty")
            for field in WIRING_FIELDS:
                rows = block.get(field)
                if rows is None:
                    continue
                if not isinstance(rows, list):
                    problems.append(f"{mid}: {lang}.{field} must be a list")
                    continue
                for i, row in enumerate(rows):
                    if not isinstance(row, dict) or "pin" not in row or "conn" not in row:
                        problems.append(f"{mid}: {lang}.{field}[{i}] needs pin/conn")

        for rel in m.get("images", []):
            path = REPO / rel
            if not path.exists():
                problems.append(f"{mid}: missing image {rel}")
            elif not ASSET_NAME.match(path.name):
                problems.append(f"{mid}: filename does not match CONVENTIONS.md §3.3 - {path.name}")

    # Reverse check: which inventory photos are not referenced by any module entry.
    used = {REPO / p for m in modules for p in m.get("images", [])}
    orphans = sorted(
        p.name
        for p in (REPO / "assets" / "inventory").iterdir()
        if p.suffix.lower() in (".jpg", ".png") and p not in used
    )

    print(f"Modules: {len(modules)}, referenced images: {len(used)}")
    if orphans:
        print(
            f"\nUnreferenced inventory photos: {len(orphans)} (not an error, but worth reviewing):"
        )
        for name in orphans:
            print(f"  - {name}")

    if problems:
        print(f"\n✗ {len(problems)} problem(s):")
        for p in problems:
            print(f"  {p}")
        return 1

    print("\n✓ Dataset is complete")
    return 0


if __name__ == "__main__":
    sys.exit(main())
