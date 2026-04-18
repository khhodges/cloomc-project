"""Boot image binary generator (Task #217).

Produces a self-contained binary boot image from a saved boot-config.json.

Format
------
Raw little-endian 32-bit memory dump of the namespace memory window:

    bytes = totalNamespaceWords * 4

The image is exactly what the simulator's `memory[]` array should look
like immediately after `_initNamespaceTable()` finishes, so loading it
is a single `memory.set(uint32_words)` on the simulator side. Real
hardware can copy it straight into namespace SRAM with no
post-processing.

The generator deliberately mirrors `simulator.js _initNamespaceTable()`
rather than calling out to the simulator runtime — Python here is the
canonical boot-image producer; the simulator's hardcoded path remains
as a fallback when no image is present.

Layout (all words 32-bit little-endian):

    [0 .. NS_LUMP_SIZE)                    Namespace lump body (header @0)
    [NS_LUMP_SIZE .. +THREAD_LUMP_SIZE)    Thread lump body  (header @0)
    [.. +ABSTR_LUMP_SIZE)                  Boot.Abstr body   (header @0,
                                            c-list at physical end)
    [resident lump bodies at programmer
     -chosen physAddr]
    [NS_TABLE_BASE .. +NS_TABLE_RESERVE)   Namespace table
       (256 entries × 3 words; named slots followed by Step-3 reserved
        empties; remainder zero)
"""
import json
import os
import struct

NS_TABLE_RESERVE = 0x300        # 768 words = 256 entries × 3
NS_ENTRY_WORDS   = 3
MAX_NS_ENTRIES   = 256
SLOT_SIZE        = 0x40         # 64 words

# Hardware-accurate device register limits (matches simulator.js
# DEVICE_REG_LIMITS and hardware/boot_rom.py _MMIO_ENTRIES).
DEVICE_REG_LIMITS = {11: 2, 12: 5, 13: 0, 14: 4}

# NS slot 3 is the Math.Add Outform demonstration slot — a remote token
# fetched on demand. Token must match simulator.js MATH_ADD_TOKEN.
MATH_ADD_OUTFORM_SLOT  = 3
MATH_ADD_OUTFORM_TOKEN = 0xDEAD0003

# Default abstraction catalog — ports simulator.js _getAbstractionCatalog()
# fallback list (used when no abstractionRegistry is wired in). The boot
# image is produced from this canonical list so server and simulator
# agree on what the default boot ROM contains.
DEFAULT_ABSTRACTION_CATALOG = [
    ("Boot.NS",       {"R":0,"W":0,"X":0,"L":0,"S":0,"E":0}, False),
    ("Boot.Thread",   {"R":0,"W":0,"X":0,"L":0,"S":0,"E":0}, False),
    ("Boot.Abstr",    {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, False),
    ("(empty)",       {"R":0,"W":0,"X":0,"L":0,"S":0,"E":0}, False),
    ("Salvation",     {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, False),
    ("Navana",        {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, False),
    ("Mint",          {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, False),
    ("Memory",        {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, False),
    ("Scheduler",     {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, False),
    ("Stack",         {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, True),
    ("DijkstraFlag",  {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, False),
    ("UART",          {"R":0,"W":0,"X":0,"L":1,"S":1,"E":1}, False),
    ("LED",           {"R":0,"W":0,"X":0,"L":1,"S":1,"E":1}, False),
    ("Button",        {"R":0,"W":0,"X":0,"L":1,"S":0,"E":1}, False),
    ("Timer",         {"R":0,"W":0,"X":0,"L":1,"S":1,"E":1}, False),
    ("Display",       {"R":0,"W":0,"X":0,"L":1,"S":1,"E":1}, False),
    ("SlideRule",     {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, True),
    ("Abacus",        {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, True),
    ("Constants",     {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, False),
    ("Loader",        {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, False),
    ("SUCC",          {"R":0,"W":0,"X":1,"L":1,"S":0,"E":1}, False),
    ("PRED",          {"R":0,"W":0,"X":1,"L":1,"S":0,"E":1}, False),
    ("ADD",           {"R":0,"W":0,"X":1,"L":1,"S":0,"E":1}, False),
    ("SUB",           {"R":0,"W":0,"X":1,"L":1,"S":0,"E":1}, False),
    ("MUL",           {"R":0,"W":0,"X":1,"L":1,"S":0,"E":1}, False),
    ("ISZERO",        {"R":0,"W":0,"X":1,"L":1,"S":0,"E":1}, False),
    ("TRUE",          {"R":0,"W":0,"X":0,"L":1,"S":0,"E":0}, False),
    ("FALSE",         {"R":0,"W":0,"X":0,"L":1,"S":0,"E":0}, False),
    ("Family",        {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, False),
    ("Schoolroom",    {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, False),
    ("Friends",       {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, False),
    ("Tunnel",        {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, False),
    ("Negotiate",     {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, False),
    ("Editor",        {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, False),
    ("Assembler",     {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, False),
    ("Debugger",      {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, False),
    ("Deployer",      {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, False),
    ("Browser",       {"R":0,"W":0,"X":0,"L":1,"S":0,"E":1}, False),
    ("Messenger",     {"R":0,"W":0,"X":0,"L":1,"S":0,"E":1}, False),
    ("Photos",        {"R":0,"W":0,"X":0,"L":1,"S":0,"E":1}, False),
    ("Social",        {"R":0,"W":0,"X":0,"L":1,"S":0,"E":1}, False),
    ("Video",         {"R":0,"W":0,"X":0,"L":1,"S":0,"E":1}, False),
    ("Email",         {"R":0,"W":0,"X":0,"L":1,"S":0,"E":1}, False),
    ("PAIR",          {"R":0,"W":0,"X":1,"L":1,"S":0,"E":1}, False),
    ("GC",            {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, False),
    ("Thread",        {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, False),
    ("Circle",        {"R":0,"W":0,"X":0,"L":0,"S":0,"E":1}, False),
]
assert len(DEFAULT_ABSTRACTION_CATALOG) == 47, "catalog drift vs simulator.js"


# ----- bit-packing helpers (mirror simulator.js exactly) ---------------------

def _u32(x):
    return x & 0xFFFFFFFF


def perm_bits(perms):
    bits = 0
    if perms.get("R"): bits |= 1
    if perms.get("W"): bits |= 2
    if perms.get("X"): bits |= 4
    if perms.get("L"): bits |= 8
    if perms.get("S"): bits |= 16
    if perms.get("E"): bits |= 32
    if perms.get("B"): bits |= 64
    return bits & 0x7F


def pack_ns_word1(limit17, b, f, g, chainable, gt_type, clist_count):
    return _u32(
        ((b & 1) << 31)
        | ((f & 1) << 30)
        | ((g & 1) << 29)
        | ((chainable & 1) << 28)
        | ((gt_type & 3) << 26)
        | (((clist_count or 0) & 0x1FF) << 17)
        | (limit17 & 0x1FFFF)
    )


def pack_lump_header(n_minus_6, cw, cc, typ=0):
    return _u32(
        (0x1F            << 27)
        | ((n_minus_6 & 0xF) << 23)
        | ((cw & 0x1FFF)     << 10)
        | ((typ & 0x3)       <<  8)
        | (cc & 0xFF)
    )


def create_gt(gt_seq, slot_id, perms, gt_type):
    p = (perm_bits(perms) << 25) & 0xFFFFFFFF
    t = ((gt_type & 0x3) << 23) & 0xFFFFFFFF
    s = ((gt_seq  & 0x7F) << 16) & 0xFFFFFFFF
    return _u32(p | t | s | (slot_id & 0xFFFF))


def compute_seal(location, limit17):
    """CRC-16/XMODEM over location (4 bytes BE) || limit17 (3 bytes BE).

    Mirrors simulator.js computeSeal() bit-for-bit.
    """
    crc = 0xFFFF
    payload = [
        (location >> 24) & 0xFF,
        (location >> 16) & 0xFF,
        (location >>  8) & 0xFF,
         location        & 0xFF,
        (limit17  >> 16) & 0xFF,
        (limit17  >>  8) & 0xFF,
         limit17         & 0xFF,
    ]
    for byte in payload:
        for i in range(8):
            bit = ((byte >> (7 - i)) & 1) ^ ((crc >> 15) & 1)
            crc = ((crc << 1) & 0xFFFF) ^ (0x1021 if bit else 0)
    return crc & 0xFFFF


def make_version_seals(gt_seq, location, limit17):
    return _u32(((gt_seq & 0x7F) << 25) | (compute_seal(location, limit17) & 0xFFFF))


# ----- main generator --------------------------------------------------------

def _ns_n_minus_6(lump_words):
    """log2(lump_words) - 6, clipped to 0..15 (header field is 4 bits).

    lump sizes are validated to be powers of 2 ≥ 64 elsewhere (Step 1
    validator); this is just the bit-width conversion.
    """
    n = 0
    while (1 << (n + 6)) < lump_words and n < 15:
        n += 1
    return n


def _read_lump_body(lumps_dir, token_hex):
    """Read raw 32-bit LE words from server/lumps/<token>.lump if present."""
    if not token_hex:
        return None
    path = os.path.join(lumps_dir, f"{token_hex}.lump")
    if not os.path.isfile(path):
        return None
    with open(path, "rb") as f:
        raw = f.read()
    n = len(raw) // 4
    return list(struct.unpack(f"<{n}I", raw[: n * 4]))


def _load_catalog_token_map(manifest_path):
    """ns_slot -> token_hex from server/lumps/manifest.json."""
    try:
        with open(manifest_path, "r") as f:
            entries = json.load(f)
    except Exception:
        return {}
    out = {}
    for e in entries if isinstance(entries, list) else []:
        slot = e.get("ns_slot")
        tok  = e.get("token")
        if isinstance(slot, int) and isinstance(tok, str):
            out[slot] = tok
    return out


def generate_boot_image(cfg, lumps_dir):
    """Produce the binary boot image bytes for the given config dict.

    `cfg` must already be Step-1 valid (target board + step1 fields).
    Step 2 / Step 3 are optional. Returns a `bytes` object whose length
    is `step1.totalNamespaceWords * 4`.
    """
    step1 = cfg["step1"]
    total       = int(step1["totalNamespaceWords"])
    ns_size     = int(step1["namespaceLumpWords"])
    thread_size = int(step1["threadLumpWords"])
    abstr_size  = int(step1["abstractionLumpWords"])

    # Memory image (Python ints, packed at the end).
    mem = [0] * total

    ns_table_base = total - NS_TABLE_RESERVE

    # ----- Step 2: per-slot physAddr overrides --------------------------
    step2_lumps = []
    if isinstance(cfg.get("step2"), dict):
        step2_lumps = cfg["step2"].get("lumps") or []
    phys_override = {}
    for e in step2_lumps:
        if (isinstance(e, dict) and e.get("resident")
                and isinstance(e.get("physAddr"), int) and e["physAddr"] > 0):
            phys_override[int(e["nsSlot"])] = int(e["physAddr"])

    catalog = DEFAULT_ABSTRACTION_CATALOG
    slot_sizes = {0: ns_size, 1: thread_size, 2: abstr_size}

    # ----- NS entries ----------------------------------------------------
    clist_gts = []
    running_offset = 0
    locations = {}                              # idx -> location word
    for i, (label, perms, chainable) in enumerate(catalog):
        # Slot 3 (Math.Add) is an Outform — fixed token & gt_type=2.
        if i == MATH_ADD_OUTFORM_SLOT:
            base = ns_table_base + i * NS_ENTRY_WORDS
            mem[base + 0] = MATH_ADD_OUTFORM_TOKEN
            mem[base + 1] = pack_ns_word1(0, 0, 1, 0, 0, 2, 1)   # f=1, gtType=2, clistCount=1
            mem[base + 2] = make_version_seals(0, MATH_ADD_OUTFORM_TOKEN, 0)
            clist_gts.append(create_gt(0, i, {"E":1}, 1))
            continue

        my_size  = slot_sizes.get(i, SLOT_SIZE)
        override = phys_override.get(i)
        if i == 0:
            loc = 0
            running_offset = ns_size
        else:
            if override is not None:
                loc = override
            else:
                loc = running_offset
                running_offset += my_size
        locations[i] = loc

        # Slot 0: limit covers the entire programmer-budgeted namespace
        # (totalNamespaceWords - 1), not a hardcoded memory.length-1 — the
        # NS root must accurately describe the namespace it anchors.
        if i == 0:
            lim17 = (total - 1) & 0x1FFFF
            clist_count = len(catalog)
        elif i in DEVICE_REG_LIMITS:
            lim17 = DEVICE_REG_LIMITS[i]
            clist_count = 0
        else:
            lim17 = (my_size - 1) & 0x1FFFF
            clist_count = 0

        base = ns_table_base + i * NS_ENTRY_WORDS
        mem[base + 0] = loc & 0xFFFFFFFF
        mem[base + 1] = pack_ns_word1(lim17, 0, 0, 0, 1 if chainable else 0,
                                      1, clist_count)
        mem[base + 2] = make_version_seals(0, loc, lim17)
        clist_gts.append(create_gt(0, i, perms, 1))

    ns_count = len(catalog)

    # ----- Step 3: empty NS slots ---------------------------------------
    empty_count = 0
    if isinstance(cfg.get("step3"), dict):
        try:
            empty_count = max(0, int(cfg["step3"].get("emptySlotCount") or 0))
        except (TypeError, ValueError):
            empty_count = 0
    if ns_count + empty_count > MAX_NS_ENTRIES:
        raise ValueError(
            f"Step 3 emptySlotCount={empty_count} would push NS table to "
            f"{ns_count + empty_count} entries; max {MAX_NS_ENTRIES}."
        )
    # Empty entries are already zero (mem is zero-initialised); just
    # advance the conceptual nsCount. We don't write a count word — the
    # simulator scans for non-zero entries.

    # ----- Foundational lump headers -------------------------------------
    # Thread lump (NS slot 1): cw=32, cc=64, typ=2.
    thread_loc = locations[1]
    mem[thread_loc] = pack_lump_header(_ns_n_minus_6(thread_size), 32, 64, 2)

    # Boot.Abstr (NS slot 2): hardware DEMO_CLIST layout — 17 c-list
    # entries at physical end, code window cw=17 starting at word 1.
    NUC_CODE_WORDS  = 17
    DEMO_CLIST_SIZE = 17

    # Hardware device GTs (clist slots 8..16) — match simulator.js
    # HW_DEVICE_SLOTS table.
    rw_perms  = {"R":1,"W":1}
    led_gt    = lambda: create_gt(0, 12, rw_perms, 1)
    hw_slots  = [(12, rw_perms)] * 6 + [
        (11, rw_perms),
        (13, {"R":1}),
        (14, rw_perms),
    ]
    while len(clist_gts) < 17:
        clist_gts.append(0)
    for off, (ns_idx, perms) in enumerate(hw_slots):
        clist_gts[8 + off] = create_gt(0, ns_idx, perms, 1)

    # Slot 3: E-perm to NS slot 16 (SlideRule) — boot-internal.
    clist_gts[3] = create_gt(0, 16, {"E":1}, 1)

    # Memory-manager GT at clist slot 0: covers full namespace memory
    # (NS slot 0). Replaces the legacy NULL "SAVE epilogue" placeholder
    # called out in the task — when a programmer-authored boot image is
    # loaded, this slot must hand the boot code an actual capability for
    # the memory it manages.
    clist_gts[0] = create_gt(0, 0, {"R":1, "W":1}, 1)

    # Truncate to hardware DEMO_CLIST size (slots beyond 16 are
    # simulator-only and not part of the boot ROM image).
    clist_gts = clist_gts[:DEMO_CLIST_SIZE]

    boot_abstr_loc = locations[2]
    lump_size      = abstr_size
    clist_start    = lump_size - DEMO_CLIST_SIZE

    # Word 0: lump header drives both CR14 and CR6 derivation in hardware.
    mem[boot_abstr_loc] = pack_lump_header(
        _ns_n_minus_6(abstr_size), NUC_CODE_WORDS, DEMO_CLIST_SIZE, 0)

    # C-list at physical end of the lump.
    for i, gt in enumerate(clist_gts):
        mem[boot_abstr_loc + clist_start + i] = gt & 0xFFFFFFFF

    # Update NS slot 2 word1: limit = lumpSize - cc - 1 (covers code
    # region only, c-list lives past it). Re-seal accordingly.
    code_region_limit = lump_size - DEMO_CLIST_SIZE - 1
    boot_ns_base = ns_table_base + 2 * NS_ENTRY_WORDS
    mem[boot_ns_base + 1] = pack_ns_word1(
        code_region_limit, 0, 0, 0, 0, 1, DEMO_CLIST_SIZE)
    mem[boot_ns_base + 2] = make_version_seals(0, boot_abstr_loc, code_region_limit)

    # ----- Resident lump bodies (Step 2) --------------------------------
    token_map = _load_catalog_token_map(
        os.path.join(lumps_dir, "manifest.json"))
    for e in step2_lumps:
        if not (isinstance(e, dict) and e.get("resident")):
            continue
        slot = int(e["nsSlot"])
        phys = int(e["physAddr"])
        token = token_map.get(slot)
        body = _read_lump_body(lumps_dir, token)
        if body is None:
            # No on-disk body — leave region zeroed; lazy loader will
            # populate at runtime. Resident reservation still costs the
            # space (NS entry already points here).
            continue
        # Honour the lump's declared size bound (don't write past it).
        size_cap = int(e.get("lumpSize") or len(body))
        n = min(len(body), size_cap, total - phys)
        for i in range(n):
            mem[phys + i] = body[i] & 0xFFFFFFFF

    # ----- Pack ----------------------------------------------------------
    return struct.pack(f"<{total}I", *mem)
