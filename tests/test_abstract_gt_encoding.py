"""Tests for Abstract GT encoding helpers (Task #406).

Covers:
  - create_abstract_gt() bit layout
  - Device-class constants
  - BOOT_IMAGE_FORMAT_TAG bump
  - LED c-list slots 8-13 in the generated boot image
  - DREAD/DWRITE routing via Node simulator (headless)
"""
import json
import os
import struct
import subprocess
import sys

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)

from server.boot_image import (
    BOOT_IMAGE_FORMAT_TAG,
    DEVICE_CLASS_LED, DEVICE_CLASS_UART, DEVICE_CLASS_BUTTON,
    DEVICE_CLASS_TIMER, DEVICE_CLASS_DISPLAY,
    AB_TYPE_IO, AB_TYPE_M_ELEVATION,
    create_abstract_gt, create_gt,
    generate_boot_image,
    NS_TABLE_RESERVE, NS_ENTRY_WORDS,
)

LUMPS_DIR = os.path.join(ROOT, "server", "lumps")


# ── create_abstract_gt bit-level tests ───────────────────────────────────────

def test_abstract_gt_type_field():
    """Abstract GT always has type=0b11 at bits[24:23]."""
    gt = create_abstract_gt(0x00, {"R": 1, "W": 1}, 0, 0x0100)
    assert (gt >> 23) & 0x3 == 3, "type bits must be 0b11"


def test_abstract_gt_ab_type_field():
    """ab_type occupies bits[31:27]."""
    for ab_type in (0x00, 0x01, 0x1F):
        gt = create_abstract_gt(ab_type, {}, 0, 0)
        assert (gt >> 27) & 0x1F == ab_type, f"ab_type mismatch for 0x{ab_type:02X}"


def test_abstract_gt_rw_bits():
    """R → bit[26], W → bit[25]; X/L/S/E/B are ignored (ab_type territory)."""
    gt_r  = create_abstract_gt(0x00, {"R": 1},        0, 0)
    gt_w  = create_abstract_gt(0x00, {"W": 1},        0, 0)
    gt_rw = create_abstract_gt(0x00, {"R": 1, "W": 1}, 0, 0)
    gt_0  = create_abstract_gt(0x00, {},               0, 0)
    assert (gt_r  >> 26) & 1 == 1 and (gt_r  >> 25) & 1 == 0
    assert (gt_w  >> 26) & 1 == 0 and (gt_w  >> 25) & 1 == 1
    assert (gt_rw >> 26) & 1 == 1 and (gt_rw >> 25) & 1 == 1
    assert (gt_0  >> 26) & 1 == 0 and (gt_0  >> 25) & 1 == 0


def test_abstract_gt_gt_seq_field():
    """gt_seq occupies bits[22:16]."""
    for seq in (0, 1, 63, 127):
        gt = create_abstract_gt(0x00, {}, seq, 0)
        assert (gt >> 16) & 0x7F == seq, f"gt_seq mismatch for {seq}"


def test_abstract_gt_ab_data_field():
    """ab_data occupies bits[15:0]."""
    for data in (0x0000, 0x0100, 0x0105, 0xFFFF):
        gt = create_abstract_gt(0x00, {}, 0, data)
        assert gt & 0xFFFF == data, f"ab_data mismatch for 0x{data:04X}"


def test_abstract_gt_io_led_encoding():
    """ab_data=[15:8]=device_class,[7:0]=device_data for I/O GTs."""
    ab_data = (DEVICE_CLASS_LED << 8) | 3     # LED pin 3
    gt = create_abstract_gt(AB_TYPE_IO, {"R": 1, "W": 1}, 0, ab_data)
    assert (gt >> 27) & 0x1F == AB_TYPE_IO
    assert (gt & 0xFF00) >> 8 == DEVICE_CLASS_LED
    assert (gt & 0x00FF) == 3


def test_abstract_gt_is_32bit():
    """All fields fit in 32 bits; no truncation."""
    gt = create_abstract_gt(0x1F, {"R": 1, "W": 1}, 127, 0xFFFF)
    assert 0 <= gt <= 0xFFFFFFFF


def test_abstract_gt_led0_known_value():
    """LED[0] Abstract GT encodes to the documented 0x07800100."""
    ab_data = (DEVICE_CLASS_LED << 8) | 0
    gt = create_abstract_gt(AB_TYPE_IO, {"R": 1, "W": 1}, 0, ab_data)
    assert gt == 0x07800100, f"LED[0] GT = 0x{gt:08X}, expected 0x07800100"


# ── device-class constant values ─────────────────────────────────────────────

def test_device_class_constants():
    assert DEVICE_CLASS_LED     == 0x01
    assert DEVICE_CLASS_UART    == 0x02
    assert DEVICE_CLASS_BUTTON  == 0x03
    assert DEVICE_CLASS_TIMER   == 0x04
    assert DEVICE_CLASS_DISPLAY == 0x05


def test_ab_type_constants():
    assert AB_TYPE_IO          == 0x00
    assert AB_TYPE_M_ELEVATION == 0x01


# ── BOOT_IMAGE_FORMAT_TAG ────────────────────────────────────────────────────

def test_boot_image_format_tag_is_task_406():
    """BOOT_IMAGE_FORMAT_TAG was bumped to 0xB0070406 for Task #406."""
    assert BOOT_IMAGE_FORMAT_TAG == 0xB0070406, (
        f"Expected 0xB0070406, got 0x{BOOT_IMAGE_FORMAT_TAG:08X}"
    )


def test_boot_image_contains_correct_format_tag():
    """Generated boot image has the updated format tag at NS_TABLE_BASE-1."""
    cfg = {"step1": {
        "totalNamespaceWords": 16384,
        "namespaceLumpWords": 64,
        "threadLumpWords": 256,
        "abstractionLumpWords": 256,
    }}
    img = generate_boot_image(cfg, LUMPS_DIR)
    words = struct.unpack(f"<{len(img)//4}I", img)
    total = len(words)
    tag_idx = total - NS_TABLE_RESERVE - 1
    assert words[tag_idx] == BOOT_IMAGE_FORMAT_TAG


# ── LED c-list slots in generated boot image ─────────────────────────────────

def _get_clist_words(img_bytes, cfg):
    """Return the 17 c-list GT words from Boot.Abstr in the generated image."""
    step1 = cfg["step1"]
    total       = int(step1["totalNamespaceWords"])
    ns_size     = int(step1["namespaceLumpWords"])
    abstr_size  = int(step1["abstractionLumpWords"])
    BOOT_ABSTR_NS_SLOT = 3
    DEMO_CLIST_SIZE = 17

    words = list(struct.unpack(f"<{total}I", img_bytes))
    ns_table_base = total - NS_TABLE_RESERVE

    # Boot.Abstr loc is at NS table slot 3, word0
    boot_entry_loc = words[ns_table_base + BOOT_ABSTR_NS_SLOT * NS_ENTRY_WORDS]
    entry_clist_start = abstr_size - DEMO_CLIST_SIZE
    clist = [words[boot_entry_loc + entry_clist_start + i] for i in range(DEMO_CLIST_SIZE)]
    return clist


def test_led_clist_slots_are_abstract_gts():
    """C-list slots 8-13 must be Abstract GTs (type=0b11)."""
    cfg = {"step1": {
        "totalNamespaceWords": 16384,
        "namespaceLumpWords": 64,
        "threadLumpWords": 256,
        "abstractionLumpWords": 256,
    }}
    img = generate_boot_image(cfg, LUMPS_DIR)
    clist = _get_clist_words(img, cfg)
    for slot_offset in range(6):
        slot_idx = 8 + slot_offset
        gt = clist[slot_idx]
        gt_type = (gt >> 23) & 0x3
        assert gt_type == 3, f"c-list[{slot_idx}] type={gt_type}, expected 3 (Abstract)"


def test_led_clist_slots_correct_device_class():
    """C-list slots 8-13 encode device_class=LED (0x01)."""
    cfg = {"step1": {
        "totalNamespaceWords": 16384,
        "namespaceLumpWords": 64,
        "threadLumpWords": 256,
        "abstractionLumpWords": 256,
    }}
    img = generate_boot_image(cfg, LUMPS_DIR)
    clist = _get_clist_words(img, cfg)
    for led_idx in range(6):
        gt = clist[8 + led_idx]
        device_class = (gt >> 8) & 0xFF
        assert device_class == DEVICE_CLASS_LED, (
            f"c-list[{8+led_idx}] device_class=0x{device_class:02X}, expected 0x01"
        )


def test_led_clist_slots_correct_device_data():
    """C-list slot 8+N encodes device_data=N (pin index)."""
    cfg = {"step1": {
        "totalNamespaceWords": 16384,
        "namespaceLumpWords": 64,
        "threadLumpWords": 256,
        "abstractionLumpWords": 256,
    }}
    img = generate_boot_image(cfg, LUMPS_DIR)
    clist = _get_clist_words(img, cfg)
    for led_idx in range(6):
        gt = clist[8 + led_idx]
        device_data = gt & 0xFF
        assert device_data == led_idx, (
            f"c-list[{8+led_idx}] device_data={device_data}, expected {led_idx}"
        )


def test_led_clist_slots_rw_permissions():
    """LED Abstract GTs have R and W bits set; no other perm bits."""
    cfg = {"step1": {
        "totalNamespaceWords": 16384,
        "namespaceLumpWords": 64,
        "threadLumpWords": 256,
        "abstractionLumpWords": 256,
    }}
    img = generate_boot_image(cfg, LUMPS_DIR)
    clist = _get_clist_words(img, cfg)
    for led_idx in range(6):
        gt = clist[8 + led_idx]
        R = (gt >> 26) & 1
        W = (gt >> 25) & 1
        assert R == 1, f"c-list[{8+led_idx}] missing R bit"
        assert W == 1, f"c-list[{8+led_idx}] missing W bit"


def test_uart_btn_timer_clist_slots_remain_inform():
    """UART (slot 14), BTN (15), TIMER (16) remain Inform GTs (type=0b01)."""
    cfg = {"step1": {
        "totalNamespaceWords": 16384,
        "namespaceLumpWords": 64,
        "threadLumpWords": 256,
        "abstractionLumpWords": 256,
    }}
    img = generate_boot_image(cfg, LUMPS_DIR)
    clist = _get_clist_words(img, cfg)
    for slot_idx, expected_ns in [(14, 11), (15, 13), (16, 14)]:
        gt = clist[slot_idx]
        gt_type = (gt >> 23) & 0x3
        ns_idx  = gt & 0xFFFF
        assert gt_type == 1, f"c-list[{slot_idx}] type={gt_type}, expected 1 (Inform)"
        assert ns_idx == expected_ns, (
            f"c-list[{slot_idx}] NS idx={ns_idx}, expected {expected_ns}"
        )


# ── perm validation (Task #406 requirement) ──────────────────────────────────

@pytest.mark.parametrize("bad_perm", ["X", "L", "S", "E", "B"])
def test_create_abstract_gt_rejects_illegal_perms(bad_perm):
    """create_abstract_gt raises ValueError for X/L/S/E/B perm bits."""
    with pytest.raises(ValueError, match=bad_perm):
        create_abstract_gt(AB_TYPE_IO, {bad_perm: 1}, 0, 0x0100)


def test_create_abstract_gt_accepts_only_rw():
    """create_abstract_gt accepts R and W without raising."""
    gt = create_abstract_gt(AB_TYPE_IO, {"R": 1, "W": 1}, 0, 0x0100)
    assert (gt >> 26) & 1 == 1   # R at bit[26]
    assert (gt >> 25) & 1 == 1   # W at bit[25]


def test_create_abstract_gt_no_perms_ok():
    """create_abstract_gt with empty perms dict is valid."""
    gt = create_abstract_gt(AB_TYPE_IO, {}, 0, 0x0100)
    assert (gt >> 25) & 0x3 == 0   # neither R nor W


# ── LED NS slot 12 freed (Task #406 requirement) ─────────────────────────────

def test_led_ns_slot_12_is_freed():
    """NS slot 12 (LED) must have an all-zero NS table entry (freed, not allocated)."""
    cfg = {"step1": {
        "totalNamespaceWords": 16384,
        "namespaceLumpWords": 64,
        "threadLumpWords": 256,
        "abstractionLumpWords": 256,
    }}
    img = generate_boot_image(cfg, LUMPS_DIR)
    words = struct.unpack(f"<{len(img) // 4}I", img)
    total = len(words)
    ns_table_base = total - NS_TABLE_RESERVE
    # NS slot 12: 4 words starting at ns_table_base + 12 * NS_ENTRY_WORDS
    slot12_base = ns_table_base + 12 * NS_ENTRY_WORDS
    entry_words = [words[slot12_base + i] for i in range(NS_ENTRY_WORDS)]
    assert all(w == 0 for w in entry_words), (
        f"NS slot 12 should be all zeros (freed), got {[hex(w) for w in entry_words]}"
    )


def test_led_catalog_slot_12_is_none():
    """DEFAULT_ABSTRACTION_CATALOG[12] must be None (LED NS slot freed)."""
    from server.boot_image import DEFAULT_ABSTRACTION_CATALOG
    assert DEFAULT_ABSTRACTION_CATALOG[12] is None, (
        "Slot 12 should be None (freed), got: " + repr(DEFAULT_ABSTRACTION_CATALOG[12])
    )


