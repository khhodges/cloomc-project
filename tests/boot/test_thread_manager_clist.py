"""Boot-image test: Thread Manager c-list CR12 authority caps (Task #1529).

Two layers of verification:

  1. Constants layer — THREAD_MANAGER_CLIST in hardware/boot_rom.py defines
     two E-perm GTs pointing at NS slots 19 and 21 (CR12_PORT_CAP and
     CR12_MBIT_CAP).  CR13 caps (slots 20, 22) are IRQ-manager territory
     and are intentionally absent.

  2. Generated image layer — generate_boot_image() writes those same two GTs
     into the Thread Manager lump (NS slot 45) c-list tail at the correct
     word offsets.

The two GTs give Thread Manager delegate access to the S-perm authority
objects that govern CHANGE CR12 and CR12 M-bit installation, enabling
cooperative scheduling implementations to switch thread stacks independently.

GT word layout (new dom+perm encoding):
    [31]    b_flag  = 0
    [30:28] perm3   = 0b100  (E-perm; Church domain)
    [27]    dom     = 1      (Church)
    [26]    spare   = 0
    [25]    f_flag  = 0
    [24:23] gt_type = 0b01   (Inform)
    [22:16] gt_seq  = 0
    [15:0]  slot_id = 19 or 21
"""
import os
import struct
import sys

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, ROOT)

from hardware.boot_rom import (  # noqa: E402
    THREAD_MANAGER_CLIST,
    CHURCH_HW_CR12_PORT_SLOT,
    CHURCH_HW_CR12_MBIT_SLOT,
)
from hardware.hw_types import (  # noqa: E402
    GT_TYPE_INFORM,
    PERM_MASK_E,
    gt_encode_perm,
)
from server.boot_image import (  # noqa: E402
    generate_boot_image,
    NS_ENTRY_WORDS,
)

LUMPS_DIR = os.path.join(ROOT, "server", "lumps")

THREAD_MANAGER_NS_SLOT = 45


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

def _decode_gt(word):
    """Decode a 32-bit GT word into its component fields."""
    word = word & 0xFFFFFFFF
    return {
        "b_flag":  (word >> 31) & 0x1,
        "perm3":   (word >> 28) & 0x7,
        "dom":     (word >> 27) & 0x1,
        "f_flag":  (word >> 25) & 0x1,
        "gt_type": (word >> 23) & 0x3,
        "gt_seq":  (word >> 16) & 0x7F,
        "slot_id":  word        & 0xFFFF,
    }


def _expected_e_perm_gt(slot_id):
    """Build the expected E-perm Inform GT word for the given NS slot."""
    dom, perm3 = gt_encode_perm(PERM_MASK_E)
    return (
        (perm3        << 28)
        | (dom        << 27)
        | (GT_TYPE_INFORM << 23)
        | (slot_id & 0xFFFF)
    ) & 0xFFFFFFFF


def _default_cfg():
    return {
        "step1": {
            "totalNamespaceWords": 16384,
            "namespaceLumpWords":     64,
            "threadLumpWords":       256,
        },
    }


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def boot_words():
    """Generate a default boot image and return as a list of 32-bit words."""
    img = generate_boot_image(_default_cfg(), LUMPS_DIR)
    total = 16384
    assert len(img) == total * 4
    return list(struct.unpack(f"<{total}I", img))


# ---------------------------------------------------------------------------
# Part 1 — THREAD_MANAGER_CLIST constant validation
# ---------------------------------------------------------------------------

def test_thread_manager_clist_length():
    """THREAD_MANAGER_CLIST has exactly 2 entries (cc contribution = 2)."""
    assert len(THREAD_MANAGER_CLIST) == 2, (
        f"Expected 2 entries (CR12_PORT_CAP + CR12_MBIT_CAP), "
        f"got {len(THREAD_MANAGER_CLIST)}.  "
        "CR13 caps belong to Scheduler.IRQ only — do not add them here."
    )


@pytest.mark.parametrize("idx,expected_slot,name", [
    (0, CHURCH_HW_CR12_PORT_SLOT, "CR12_PORT_CAP"),
    (1, CHURCH_HW_CR12_MBIT_SLOT, "CR12_MBIT_CAP"),
])
def test_thread_manager_clist_slot_id(idx, expected_slot, name):
    """THREAD_MANAGER_CLIST[idx] references the correct NS slot."""
    gt = _decode_gt(THREAD_MANAGER_CLIST[idx])
    assert gt["slot_id"] == expected_slot, (
        f"THREAD_MANAGER_CLIST[{idx}] ({name}): slot_id={gt['slot_id']}, "
        f"expected {expected_slot} (NS slot for {name}).\n"
        "  Update the make_gt() call in hardware/boot_rom.py."
    )


@pytest.mark.parametrize("idx,name", [
    (0, "CR12_PORT_CAP"),
    (1, "CR12_MBIT_CAP"),
])
def test_thread_manager_clist_e_perm(idx, name):
    """THREAD_MANAGER_CLIST[idx] carries E-perm (Church domain, perm3=0b100)."""
    gt = _decode_gt(THREAD_MANAGER_CLIST[idx])
    assert gt["dom"] == 1, (
        f"THREAD_MANAGER_CLIST[{idx}] ({name}): dom={gt['dom']}, expected 1 (Church).\n"
        "  E-perm requires dom=1; use PERM_MASK_E in the make_gt() call."
    )
    expected_dom, expected_perm3 = gt_encode_perm(PERM_MASK_E)
    assert gt["perm3"] == expected_perm3, (
        f"THREAD_MANAGER_CLIST[{idx}] ({name}): perm3={gt['perm3']:#05b}, "
        f"expected {expected_perm3:#05b} (E-perm = 0b100).\n"
        "  Check PERM_MASK_E is passed to make_gt()."
    )


@pytest.mark.parametrize("idx,name", [
    (0, "CR12_PORT_CAP"),
    (1, "CR12_MBIT_CAP"),
])
def test_thread_manager_clist_inform_type(idx, name):
    """THREAD_MANAGER_CLIST[idx] is an Inform GT (gt_type=0b01)."""
    gt = _decode_gt(THREAD_MANAGER_CLIST[idx])
    assert gt["gt_type"] == GT_TYPE_INFORM, (
        f"THREAD_MANAGER_CLIST[{idx}] ({name}): gt_type={gt['gt_type']}, "
        f"expected {GT_TYPE_INFORM} (Inform).\n"
        "  Pass GT_TYPE_INFORM as the first argument to make_gt()."
    )


@pytest.mark.parametrize("idx,slot_id,name", [
    (0, CHURCH_HW_CR12_PORT_SLOT, "CR12_PORT_CAP"),
    (1, CHURCH_HW_CR12_MBIT_SLOT, "CR12_MBIT_CAP"),
])
def test_thread_manager_clist_raw_word(idx, slot_id, name):
    """THREAD_MANAGER_CLIST[idx] equals the fully-encoded E-perm Inform GT."""
    expected = _expected_e_perm_gt(slot_id)
    actual   = THREAD_MANAGER_CLIST[idx] & 0xFFFFFFFF
    assert actual == expected, (
        f"THREAD_MANAGER_CLIST[{idx}] ({name}): "
        f"0x{actual:08X} != expected 0x{expected:08X}.\n"
        f"  Expected: E-perm Inform GT → NS slot {slot_id}.\n"
        "  Regenerate with make_gt(GT_TYPE_INFORM, PERM_MASK_E, slot_id, 0)."
    )


def test_thread_manager_clist_excludes_cr13():
    """THREAD_MANAGER_CLIST must not contain any GT pointing at CR13 NS slots (20 or 22)."""
    cr13_slots = {20, 22}
    for idx, word in enumerate(THREAD_MANAGER_CLIST):
        gt = _decode_gt(word)
        assert gt["slot_id"] not in cr13_slots, (
            f"THREAD_MANAGER_CLIST[{idx}]: slot_id={gt['slot_id']} is a CR13 authority cap "
            "(NS slot 20=CR13_PORT_CAP or 22=CR13_MBIT_CAP).  "
            "CR13 caps are IRQ-manager territory and must not appear in Thread Manager's c-list."
        )


# ---------------------------------------------------------------------------
# Part 2 — Generated boot image: Thread Manager lump (NS slot 45) c-list
#
# Confirms that generate_boot_image() writes the CR12 authority-cap GTs into
# the Thread Manager lump c-list tail at the correct word offsets.
#
# Layout (64-word lump, cc=4 after Task #1529):
#   Clist indices 0-1: Scheduler E (slot 8) and Memory E (slot 7)  [pre-existing]
#   Clist indices 2-3: CR12_PORT E (slot 19), CR12_MBIT E (slot 21)  [Task #1529]
# ---------------------------------------------------------------------------

def _thread_manager_lump_base(boot_words_list):
    """Return the word offset in boot_words_list where the Thread Manager lump begins."""
    total = len(boot_words_list)
    ns_table_base = None
    for i in range(1, min(2048, total) + 1):
        if boot_words_list[total - i] == 0xB0070563:
            ns_table_base = total - i + 1
            break
    assert ns_table_base is not None, "BOOT_IMAGE_FORMAT_TAG not found in image"
    tm_ns_base = ns_table_base + THREAD_MANAGER_NS_SLOT * NS_ENTRY_WORDS
    return boot_words_list[tm_ns_base]   # word0_location = lump base word address


def _thread_manager_lump_cc(boot_words_list, lump_base):
    """Return the cc field from the Thread Manager lump header at lump_base."""
    hdr = boot_words_list[lump_base]
    return hdr & 0xFF


def _thread_manager_clist_word(boot_words_list, lump_base, cc, idx):
    """Return the GT word at c-list offset idx inside the Thread Manager lump."""
    lump_size = 64   # SLOT_SIZE — the default 64-word allocation
    return boot_words_list[lump_base + lump_size - cc + idx]


def test_thread_manager_lump_cc_is_4(boot_words):
    """Thread Manager lump (NS slot 45) has cc=4 after adding the two CR12 authority-cap GTs."""
    lump_base = _thread_manager_lump_base(boot_words)
    cc = _thread_manager_lump_cc(boot_words, lump_base)
    assert cc == 4, (
        f"Thread Manager lump at word {lump_base}: cc={cc}, expected 4 "
        "(Scheduler E + Memory E + 2 CR12 authority-cap GTs).\n"
        "  Check SERVICE_CLIST_DEFS slot 45 in server/boot_image.py."
    )


@pytest.mark.parametrize("clist_idx,expected_slot,name", [
    (2, CHURCH_HW_CR12_PORT_SLOT, "CR12_PORT_CAP"),
    (3, CHURCH_HW_CR12_MBIT_SLOT, "CR12_MBIT_CAP"),
])
def test_thread_manager_lump_authority_cap_slot_id(boot_words, clist_idx, expected_slot, name):
    """Generated Thread Manager lump c-list[clist_idx] references NS slot expected_slot."""
    lump_base = _thread_manager_lump_base(boot_words)
    cc = _thread_manager_lump_cc(boot_words, lump_base)
    word = _thread_manager_clist_word(boot_words, lump_base, cc, clist_idx)
    gt = _decode_gt(word)
    assert gt["slot_id"] == expected_slot, (
        f"Thread Manager lump c-list[{clist_idx}] ({name}): slot_id={gt['slot_id']}, "
        f"expected {expected_slot}.\n"
        "  Check SERVICE_CLIST_DEFS slot 45 in server/boot_image.py."
    )


@pytest.mark.parametrize("clist_idx,name", [
    (2, "CR12_PORT_CAP"),
    (3, "CR12_MBIT_CAP"),
])
def test_thread_manager_lump_authority_cap_e_perm(boot_words, clist_idx, name):
    """Generated Thread Manager lump c-list[clist_idx] carries E-perm (Church, perm3=0b100)."""
    lump_base = _thread_manager_lump_base(boot_words)
    cc = _thread_manager_lump_cc(boot_words, lump_base)
    word = _thread_manager_clist_word(boot_words, lump_base, cc, clist_idx)
    gt = _decode_gt(word)
    _, expected_perm3 = gt_encode_perm(PERM_MASK_E)
    assert gt["dom"] == 1, (
        f"Thread Manager lump c-list[{clist_idx}] ({name}): dom={gt['dom']}, expected 1 (Church)."
    )
    assert gt["perm3"] == expected_perm3, (
        f"Thread Manager lump c-list[{clist_idx}] ({name}): perm3={gt['perm3']:#05b}, "
        f"expected {expected_perm3:#05b} (E-perm)."
    )


@pytest.mark.parametrize("clist_idx,slot_id,name", [
    (2, CHURCH_HW_CR12_PORT_SLOT, "CR12_PORT_CAP"),
    (3, CHURCH_HW_CR12_MBIT_SLOT, "CR12_MBIT_CAP"),
])
def test_thread_manager_lump_authority_cap_raw_word(boot_words, clist_idx, slot_id, name):
    """Generated Thread Manager lump c-list[clist_idx] has the correct full 32-bit GT word."""
    lump_base = _thread_manager_lump_base(boot_words)
    cc = _thread_manager_lump_cc(boot_words, lump_base)
    actual   = _thread_manager_clist_word(boot_words, lump_base, cc, clist_idx) & 0xFFFFFFFF
    expected = _expected_e_perm_gt(slot_id)
    assert actual == expected, (
        f"Thread Manager lump c-list[{clist_idx}] ({name}): "
        f"0x{actual:08X} != expected 0x{expected:08X}.\n"
        f"  Expected E-perm Inform GT → NS slot {slot_id}."
    )


def test_thread_manager_lump_no_cr13_caps(boot_words):
    """Generated Thread Manager lump c-list must not contain any CR13 authority GT."""
    lump_base = _thread_manager_lump_base(boot_words)
    cc = _thread_manager_lump_cc(boot_words, lump_base)
    cr13_slots = {20, 22}
    for idx in range(cc):
        word = _thread_manager_clist_word(boot_words, lump_base, cc, idx)
        gt = _decode_gt(word)
        assert gt["slot_id"] not in cr13_slots, (
            f"Thread Manager lump c-list[{idx}]: slot_id={gt['slot_id']} is a CR13 cap "
            "(NS slot 20 or 22).  CR13 authority must not appear in Thread Manager's c-list."
        )
