from amaranth import *
from amaranth.lib.data import StructLayout

GT_LAYOUT = StructLayout({
    "slot_id": unsigned(16),
    "gt_seq":  unsigned(7),
    "gt_type": unsigned(2),
    "perms":   unsigned(6),
    "b_flag":  unsigned(1),
})

CAP_REG_LAYOUT = StructLayout({
    "word0_gt":       GT_LAYOUT,
    "word1_location": unsigned(32),
    "word2_w2":       unsigned(32),
    "word3_w3":       unsigned(32),
})

WORD2_LAYOUT = StructLayout({
    "limit_offset": unsigned(21),
    "gt_seq":       unsigned(7),
    "spare":        unsigned(4),
})

WORD3_LAYOUT = StructLayout({
    "crc":   unsigned(16),
    "g_bit": unsigned(1),
    "spare": unsigned(15),
})

LUMP_HEADER_LAYOUT = StructLayout({
    "r":         unsigned(1),
    "c":         unsigned(1),
    "h":         unsigned(1),
    "mw":        unsigned(6),
    "typ":       unsigned(2),
    "cc":        unsigned(8),
    "n_minus_6": unsigned(4),
    "ver":       unsigned(4),
    "magic":     unsigned(5),
})

NS_ENTRY_LAYOUT = StructLayout({
    "word0_location": unsigned(32),
    "word1_location": unsigned(32),
    "word2_w2":       unsigned(32),
    "word3_w3":       unsigned(32),
})

COND_FLAGS_LAYOUT = StructLayout({
    "N": unsigned(1),
    "Z": unsigned(1),
    "C": unsigned(1),
    "V": unsigned(1),
})
