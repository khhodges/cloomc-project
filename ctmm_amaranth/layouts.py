from amaranth import *
from amaranth.lib.data import StructLayout

GT_LAYOUT = StructLayout({
    "offset": unsigned(32),
    "spare":  unsigned(25),
    "g_bit":  unsigned(1),
    "perms":  unsigned(6),
})

CAP_REG_LAYOUT = StructLayout({
    "word0_gt":       GT_LAYOUT,
    "word1_location": unsigned(64),
    "word2_limit":    unsigned(64),
    "word3_seals":    unsigned(64),
})

NS_ENTRY_LAYOUT = StructLayout({
    "word1_location": unsigned(64),
    "word2_limit":    unsigned(64),
    "word3_seals":    unsigned(64),
})

COND_FLAGS_LAYOUT = StructLayout({
    "N": unsigned(1),
    "Z": unsigned(1),
    "C": unsigned(1),
    "V": unsigned(1),
})

EXCL_MONITOR_LAYOUT = StructLayout({
    "state":     unsigned(2),
    "addr":      unsigned(32),
    "thread_id": unsigned(4),
})
