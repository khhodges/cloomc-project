from enum import IntEnum

PERM_R = 0
PERM_W = 1
PERM_X = 2
PERM_L = 3
PERM_S = 4
PERM_E = 5

PERM_MASK_R = 1 << PERM_R
PERM_MASK_W = 1 << PERM_W
PERM_MASK_X = 1 << PERM_X
PERM_MASK_L = 1 << PERM_L
PERM_MASK_S = 1 << PERM_S
PERM_MASK_E = 1 << PERM_E

DATA_PERMS = PERM_MASK_R | PERM_MASK_W | PERM_MASK_X
CAP_PERMS = PERM_MASK_L | PERM_MASK_S | PERM_MASK_E

NUM_CAP_REGS = 16

CR_CLIST = 6
CR_CLOOMC = 7
CR_THREAD = 8
CR_INTERRUPT = 9
CR_DFAULT = 10
CR_NAMESPACE = 15


class CondCode(IntEnum):
    EQ = 0b0000
    NE = 0b0001
    CS = 0b0010
    CC = 0b0011
    MI = 0b0100
    PL = 0b0101
    VS = 0b0110
    VC = 0b0111
    HI = 0b1000
    LS = 0b1001
    GE = 0b1010
    LT = 0b1011
    GT = 0b1100
    LE = 0b1101
    AL = 0b1110
    NV = 0b1111


class ChurchOpcode(IntEnum):
    LOAD   = 0b00001
    SAVE   = 0b00010
    CALL   = 0b00011
    RETURN = 0b00100
    CHANGE = 0b00101
    SWITCH = 0b00110
    TPERM  = 0b00111
    LOADX  = 0b01000
    SAVEX  = 0b01001
    LDM    = 0b01010
    STM    = 0b01011


class TuringOpcode(IntEnum):
    MOV = 0b10000
    ADD = 0b10001
    SUB = 0b10010
    MUL = 0b10011
    DIV = 0b10100
    AND = 0b10101
    ORR = 0b10110
    EOR = 0b10111
    LSL = 0b11000
    LSR = 0b11001
    ASR = 0b11010
    CMP = 0b11011
    TST = 0b11100
    LDI = 0b11101
    B   = 0b11110
    BL  = 0b11111


class TpermPreset(IntEnum):
    CLEAR = 0
    R     = 1
    RW    = 2
    X     = 3
    RX    = 4
    RWX   = 5
    L     = 6
    S     = 7
    E     = 8
    LS    = 9
    LE    = 10
    SE    = 11
    LSE   = 12
    RWXL  = 13
    RSV1  = 14
    RSV2  = 15


TPERM_MASKS = {
    TpermPreset.CLEAR: 0x0000,
    TpermPreset.R:     PERM_MASK_R,
    TpermPreset.RW:    PERM_MASK_R | PERM_MASK_W,
    TpermPreset.X:     PERM_MASK_X,
    TpermPreset.RX:    PERM_MASK_R | PERM_MASK_X,
    TpermPreset.RWX:   PERM_MASK_R | PERM_MASK_W | PERM_MASK_X,
    TpermPreset.L:     PERM_MASK_L,
    TpermPreset.S:     PERM_MASK_S,
    TpermPreset.E:     PERM_MASK_E,
    TpermPreset.LS:    PERM_MASK_L | PERM_MASK_S,
    TpermPreset.LE:    PERM_MASK_L | PERM_MASK_E,
    TpermPreset.SE:    PERM_MASK_S | PERM_MASK_E,
    TpermPreset.LSE:   PERM_MASK_L | PERM_MASK_S | PERM_MASK_E,
    TpermPreset.RWXL:  PERM_MASK_R | PERM_MASK_W | PERM_MASK_X | PERM_MASK_L,
}


class FaultType(IntEnum):
    NONE       = 0x0
    PERM_R     = 0x1
    PERM_W     = 0x2
    PERM_X     = 0x3
    PERM_L     = 0x4
    PERM_S     = 0x5
    PERM_E     = 0x6
    NULL_CAP   = 0x7
    BOUNDS     = 0x8
    MAC        = 0x9
    INVALID_OP = 0xA
    TPERM_RSV  = 0xB
    EXCL_FAIL  = 0xC


class ExclMonitorState(IntEnum):
    IDLE    = 0b00
    ACTIVE  = 0b01
    CLEARED = 0b10


class BootState(IntEnum):
    IDLE       = 0
    FAULT_RST  = 1
    LOAD_NS    = 2
    INIT_THRD  = 3
    LOAD_NUC   = 4
    COMPLETE   = 5
