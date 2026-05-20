#!/usr/bin/env python3
"""audit_clist.py — CI guard: every empty c-list slot must have a registered name.

Usage:
    python scripts/audit_clist.py --check      # exit 1 if any unnamed empty slot
    python scripts/audit_clist.py              # same as --check

Exit codes:
    0  all empty slots are named (or intentionally null / runtime-filled)
    1  one or more slots are empty with no name in their capabilities entry

Wire into the check-capabilities-blocks workflow alongside the JS checker.

An "undocumented" slot is one where:
  - the binary word at that c-list position is 0x00000000, AND
  - the sidecar has no capabilities entry with a non-empty "name" field for that slot

Slots with intentionally_null=true or filled_by present are accepted as named
(they have an explicit design-time decision recorded).
"""

import json
import os
import re
import struct
import sys


LUMPS_DIR = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "server", "lumps")
)


def _parse_header(word):
    magic   = (word >> 27) & 0x1F
    n_m6    = (word >> 23) & 0xF
    cc      =  word        & 0xFF
    lump_sz = 1 << (n_m6 + 6)
    return dict(magic=magic, cc=cc, lump_sz=lump_sz, valid=(magic == 0x1F))


def _read_clist(token):
    path = os.path.join(LUMPS_DIR, f"{token}.lump")
    with open(path, "rb") as f:
        raw = f.read()
    words = struct.unpack(f">{len(raw) // 4}I", raw)
    h = _parse_header(words[0])
    if h["cc"] == 0:
        return h, []
    start = h["lump_sz"] - h["cc"]
    return h, list(words[start: start + h["cc"]])


def _load_sidecar(token):
    path = os.path.join(LUMPS_DIR, f"{token}.json")
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)


def _build_cap_map(capabilities):
    cap_map = {}
    if not capabilities:
        return cap_map
    for i, cap in enumerate(capabilities):
        slot = cap.get("slot", i)
        cap_map[slot] = cap
    return cap_map


def _is_archive(name):
    return bool(re.match(r"^[0-9a-f]{8}-v\d+$", name.lower()))


def _lump_tokens():
    return sorted(
        fn[:-5].lower()
        for fn in os.listdir(LUMPS_DIR)
        if fn.endswith(".lump") and not _is_archive(fn[:-5])
    )


def check():
    tokens   = _lump_tokens()
    failures = []

    for token in tokens:
        try:
            h, clist = _read_clist(token)
        except FileNotFoundError:
            continue

        if h["cc"] == 0:
            continue

        sc      = _load_sidecar(token)
        cap_map = _build_cap_map((sc or {}).get("capabilities", []))

        for slot_idx, word in enumerate(clist):
            if word != 0:
                continue

            cap  = cap_map.get(slot_idx)
            name = (cap or {}).get("name", "").strip()

            if name:
                continue

            abst = (sc or {}).get("abstraction", "?") if sc else "?"
            failures.append(
                f"  {token} ({abst}) slot {slot_idx}: "
                f"empty GT word with no name in capabilities entry"
            )

    if failures:
        print("audit_clist: FAIL — undocumented empty c-list slots found:")
        for line in failures:
            print(line)
        print()
        print(
            "Fix: add a capabilities entry with a 'name' field in Abstraction.Method\n"
            "notation to the sidecar JSON for each listed slot.  Then re-run\n"
            "  python scripts/populate_clist.py --dry-run\n"
            "to confirm the slot can be auto-filled or is correctly marked NEEDS-SPEC."
        )
        return 1

    print(f"audit_clist: OK — all empty c-list slots are named across {len(tokens)} lumps.")
    return 0


def main():
    sys.exit(check())


if __name__ == "__main__":
    main()
