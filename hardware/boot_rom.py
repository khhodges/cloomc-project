from amaranth import *

from .hw_types import *


def crc16_ccitt(word0_gt25, word1, word2, poly=0x1021, init=0xFFFF):
    """CRC-16/CCITT over GT[24:0] (25 bits, MSB first) + word1 (32 bits) + word2 (32 bits).
    Total: 89 bits, poly=0x1021, init=0xFFFF."""
    crc = init
    for bit in range(24, -1, -1):
        top = ((crc >> 15) ^ ((word0_gt25 >> bit) & 1)) & 1
        crc = ((crc << 1) & 0xFFFF) ^ (poly if top else 0)
    for word in (word1, word2):
        for bit in range(31, -1, -1):
            top = ((crc >> 15) ^ ((word >> bit) & 1)) & 1
            crc = ((crc << 1) & 0xFFFF) ^ (poly if top else 0)
    return crc & 0xFFFF


def encode_church(opcode, cond=CondCode.AL, cr_dst=0, cr_src=0, imm=0):
    return ((opcode & 0x1F) << 27) | ((cond & 0xF) << 23) | \
           ((cr_dst & 0xF) << 19) | ((cr_src & 0xF) << 15) | (imm & 0x7FFF)


def make_gt(gt_type=GT_TYPE_NULL, perms=0, slot_id=0, gt_seq=0):
    return (perms << 25) | (gt_type << 23) | (gt_seq << 16) | slot_id


BOOT_PROGRAM = []
if ENABLE_CHANGE_SWITCH:
    BOOT_PROGRAM.append(
        encode_church(ChurchOpcode.CHANGE, CondCode.AL, cr_dst=8, cr_src=8, imm=1))

BOOT_PROGRAM += [
    encode_church(ChurchOpcode.LOAD, CondCode.AL, cr_dst=1, cr_src=6, imm=0),

    encode_church(ChurchOpcode.LOAD, CondCode.AL, cr_dst=2, cr_src=6, imm=1),

    encode_church(ChurchOpcode.TPERM, CondCode.AL, cr_dst=2, imm=TpermPreset.X),

    encode_church(ChurchOpcode.LAMBDA, CondCode.AL, cr_dst=2),

    encode_church(ChurchOpcode.LOAD, CondCode.AL, cr_dst=0, cr_src=6, imm=6),
    encode_church(ChurchOpcode.TPERM, CondCode.AL, cr_dst=0, imm=TpermPreset.E),
    encode_church(ChurchOpcode.CALL, CondCode.AL, cr_dst=0, cr_src=0),

    encode_church(ChurchOpcode.LOAD, CondCode.AL, cr_dst=7, cr_src=6, imm=1),
    encode_church(ChurchOpcode.TPERM, CondCode.AL, cr_dst=7, imm=TpermPreset.X),
    encode_church(ChurchOpcode.LAMBDA, CondCode.AL, cr_dst=7),

    encode_church(ChurchOpcode.RETURN, CondCode.AL, cr_src=5),

    encode_church(ChurchOpcode.SAVE, CondCode.AL, cr_dst=6, cr_src=1, imm=2),
]

while len(BOOT_PROGRAM) < 256:
    BOOT_PROGRAM.append(0x00000000)


def _make_ns_entry(gt_type, perms, slot_id, gt_seq, location, size):
    """Build a 4-word NS entry: [location, word1_rsv, word2_w2, word3_w3].

    word2_w2: limit_offset[20:0] | gt_seq[6:0] | spare[3:0]
    word3_w3: crc[15:0] | g_bit[0] | spare[14:0]
    """
    gt_word0 = make_gt(gt_type, perms, slot_id, gt_seq)
    word0_gt25 = gt_word0 & 0x1FFFFFF
    word1 = location
    limit_offset = size & 0x1FFFFF
    word2 = (gt_seq & 0x7F) << 21 | limit_offset
    crc = crc16_ccitt(word0_gt25, word1, word2)
    word3 = crc & 0xFFFF
    return [word0_gt25, word1, word2, word3]


DEMO_NAMESPACE = []
for _i in range(16):
    _location = NS_TABLE_BASE if _i == 0 else _i * 0x100
    _size = 8
    _gt_seq = 0
    _entry = _make_ns_entry(GT_TYPE_REAL, PERM_MASK_R | PERM_MASK_W, _i, _gt_seq, _location, _size)
    DEMO_NAMESPACE.extend(_entry)


DEMO_CLIST = [
    make_gt(GT_TYPE_REAL, PERM_MASK_R | PERM_MASK_X, 3, 0),
    make_gt(GT_TYPE_REAL, PERM_MASK_X, 4, 0),
    make_gt(GT_TYPE_NULL, 0, 0, 0),
    make_gt(GT_TYPE_REAL, PERM_MASK_E, 2, 0),
    make_gt(GT_TYPE_REAL, PERM_MASK_E, 5, 0),
    make_gt(GT_TYPE_REAL, PERM_MASK_L, 6, 0),
    make_gt(GT_TYPE_REAL, PERM_MASK_E, 4, 0),
    make_gt(GT_TYPE_NULL, 0, 0, 0),
]

while len(DEMO_CLIST) < 64:
    DEMO_CLIST.append(0)


class BootRom(Elaboratable):
    """Instruction ROM for Church Machine boot and demo program.

    Uses Array constants for reliable iCE40 initialization.
    Only non-zero entries are stored; default is 0.
    Registered output maintains 1-cycle read latency matching original BRAM behavior.
    """

    def __init__(self, program=None):
        if program is None:
            program = BOOT_PROGRAM
        self.program = program[:512]
        while len(self.program) < 512:
            self.program.append(0)

        self.addr = Signal(9)
        self.data = Signal(32)

    def elaborate(self, platform):
        m = Module()

        rom_comb = Signal(32)
        with m.Switch(self.addr):
            for i, word in enumerate(self.program):
                if word != 0:
                    with m.Case(i):
                        m.d.comb += rom_comb.eq(word)
            with m.Default():
                m.d.comb += rom_comb.eq(0)

        m.d.sync += self.data.eq(rom_comb)

        return m
