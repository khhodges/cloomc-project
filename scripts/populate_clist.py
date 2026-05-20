#!/usr/bin/env python3
"""populate_clist.py — Fill design-time c-list GT words in lump binaries.

Usage:
    python scripts/populate_clist.py --dry-run   # print table, write nothing
    python scripts/populate_clist.py --write      # apply AUTO-FILL slots, update sidecars
    python scripts/populate_clist.py              # defaults to --dry-run

Status codes per slot:
    FILLED       binary word is already non-zero — skipped
    AUTO-FILL    target_ns or abstract_gt present — GT computed and queued
    RUNTIME      filled_by field present — injected at runtime, skipped
    NULL         intentionally_null=true — deliberately zero, skipped
    NEEDS-SPEC   name registered but GT not yet resolvable
    UNNAMED      no capabilities entry at all — auditor will flag this

GT encoding:
    Inform E-GT  create_gt(0, target_ns, {"E":1}, 1)
    Abstract GT  fixed word from capabilities[slot]["abstract_gt"]
"""

import argparse
import json
import os
import struct
import sys

LUMPS_DIR = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "server", "lumps")
)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "server"))
from boot_image import create_gt  # noqa: E402


def _parse_header(word):
    magic  = (word >> 27) & 0x1F
    n_m6   = (word >> 23) & 0xF
    cw     = (word >> 10) & 0x1FFF
    cc     =  word        & 0xFF
    lump_sz = 1 << (n_m6 + 6)
    return dict(magic=magic, cw=cw, cc=cc, lump_sz=lump_sz, valid=(magic == 0x1F))


def _read_lump(token):
    path = os.path.join(LUMPS_DIR, f"{token}.lump")
    with open(path, "rb") as f:
        raw = f.read()
    words = list(struct.unpack(f">{len(raw) // 4}I", raw))
    h = _parse_header(words[0])
    return h, words, raw


def _write_lump(token, words):
    path = os.path.join(LUMPS_DIR, f"{token}.lump")
    with open(path, "wb") as f:
        f.write(struct.pack(f">{len(words)}I", *words))


def _load_sidecar(token):
    path = os.path.join(LUMPS_DIR, f"{token}.json")
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)


def _save_sidecar(token, data):
    path = os.path.join(LUMPS_DIR, f"{token}.json")
    with open(path, "w") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
        f.write("\n")


def _is_archive(name):
    import re
    return bool(re.match(r"^[0-9a-f]{8}-v\d+$", name.lower()))


def _lump_tokens():
    return sorted(
        fn[:-5].lower()
        for fn in os.listdir(LUMPS_DIR)
        if fn.endswith(".lump") and not _is_archive(fn[:-5])
    )


def _build_cap_map(capabilities):
    """Return {slot_index: cap_entry} from a capabilities list.

    Handles both old-style (implicit slot = list index) and new-style
    (explicit slot field) entries.
    """
    cap_map = {}
    if not capabilities:
        return cap_map
    for i, cap in enumerate(capabilities):
        slot = cap.get("slot", i)
        cap_map[slot] = cap
    return cap_map


def _compute_gt(cap):
    """Return the GT word to write for this capability entry, or None."""
    if "target_ns" in cap:
        return create_gt(0, cap["target_ns"], {"E": 1}, 1)
    if "abstract_gt" in cap:
        return int(cap["abstract_gt"], 16)
    return None


def _classify(slot_idx, binary_word, cap):
    """Return (status, gt_word_or_None, name)."""
    name = cap.get("name", "") if cap else ""

    if binary_word != 0:
        return "FILLED", binary_word, name

    if cap is None:
        return "UNNAMED", None, ""

    if cap.get("intentionally_null"):
        return "NULL", None, name

    if "filled_by" in cap:
        return "RUNTIME", None, name

    gt = _compute_gt(cap)
    if gt is not None:
        return "AUTO-FILL", gt, name

    if name:
        return "NEEDS-SPEC", None, name

    return "UNNAMED", None, ""


def run(write_mode=False):
    tokens = _lump_tokens()

    total_autofill = 0
    total_runtime  = 0
    total_null     = 0
    total_filled   = 0
    total_needsspec = 0
    total_unnamed  = 0

    col_tok  = 12
    col_slot =  5
    col_stat = 12
    col_name = 36
    col_gt   = 12

    header = (
        f"{'Token':<{col_tok}} {'Slot':>{col_slot}} {'Status':<{col_stat}} "
        f"{'Pet name':<{col_name}} {'GT word':<{col_gt}}"
    )
    print(header)
    print("-" * len(header))

    for token in tokens:
        try:
            h, words, _ = _read_lump(token)
        except FileNotFoundError:
            continue

        if h["cc"] == 0:
            continue

        sc = _load_sidecar(token)
        cap_list = (sc or {}).get("capabilities", [])
        cap_map  = _build_cap_map(cap_list)

        clist_start = h["lump_sz"] - h["cc"]

        queued_writes = []

        for slot_idx in range(h["cc"]):
            word_pos    = clist_start + slot_idx
            binary_word = words[word_pos]
            cap         = cap_map.get(slot_idx)

            status, gt_val, name = _classify(slot_idx, binary_word, cap)

            gt_str = f"0x{gt_val:08x}" if gt_val is not None else "—"

            print(
                f"{token:<{col_tok}} {slot_idx:>{col_slot}} {status:<{col_stat}} "
                f"{name:<{col_name}} {gt_str:<{col_gt}}"
            )

            if status == "AUTO-FILL":
                total_autofill += 1
                queued_writes.append((word_pos, slot_idx, gt_val, cap))
            elif status == "FILLED":
                total_filled += 1
            elif status == "RUNTIME":
                total_runtime += 1
            elif status == "NULL":
                total_null += 1
            elif status == "NEEDS-SPEC":
                total_needsspec += 1
            elif status == "UNNAMED":
                total_unnamed += 1

        if write_mode and queued_writes and sc is not None:
            for word_pos, slot_idx, gt_val, cap in queued_writes:
                words[word_pos] = gt_val

            _write_lump(token, words)

            cap_map_updated = dict(cap_map)
            for _, slot_idx, gt_val, cap in queued_writes:
                entry = dict(cap)
                entry["gt"] = f"0x{gt_val:08x}"
                cap_map_updated[slot_idx] = entry

            new_caps = [cap_map_updated[s] for s in sorted(cap_map_updated)]
            sc["capabilities"] = new_caps

            pn = sc.setdefault("pet_names", {})
            cr = pn.setdefault("CR", {})
            for _, slot_idx, _, cap in queued_writes:
                name = cap.get("name", "")
                key  = str(slot_idx)
                if name and key not in cr:
                    cr[key] = name

            _save_sidecar(token, sc)

    # ── Mirror guard: capabilities[].name must equal pet_names.CR[slot] ──────
    mirror_errors = []
    for fn in sorted(os.listdir(LUMPS_DIR)):
        if not fn.endswith(".lump"):
            continue
        token = fn[:-5]
        sc = _load_sidecar(token)
        if sc is None:
            continue
        caps = {str(c["slot"]): c for c in sc.get("capabilities", []) if "slot" in c}
        cr = sc.get("pet_names", {}).get("CR", {})
        for slot_key, cap in caps.items():
            cap_name = cap.get("name", "")
            cr_name  = cr.get(slot_key, "")
            if cap_name and cr_name and cap_name != cr_name:
                mirror_errors.append(
                    f"  {token} slot {slot_key}: capabilities.name={cap_name!r}"
                    f" != pet_names.CR[{slot_key!r}]={cr_name!r}"
                )
    if mirror_errors:
        print("MIRROR ERRORS — pet_names.CR must exactly match capabilities[].name:")
        for e in mirror_errors:
            print(e)
        import sys; sys.exit(1)

    print()
    print(
        f"Summary: {total_filled} FILLED  {total_autofill} AUTO-FILL"
        f"  {total_runtime} RUNTIME  {total_null} NULL"
        f"  {total_needsspec} NEEDS-SPEC  {total_unnamed} UNNAMED"
    )
    if write_mode:
        print(f"Written: {total_autofill} slot(s) filled across lump binaries and sidecars.")
    return total_unnamed


def main():
    parser = argparse.ArgumentParser(
        description="Populate design-time c-list GT words in lump binaries."
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--dry-run", action="store_true", default=True,
        help="Print table without writing anything (default)"
    )
    group.add_argument(
        "--write", action="store_true",
        help="Apply AUTO-FILL slots to binaries and update sidecar JSONs"
    )
    args = parser.parse_args()

    write_mode = args.write
    unnamed = run(write_mode=write_mode)
    sys.exit(0)


if __name__ == "__main__":
    main()
