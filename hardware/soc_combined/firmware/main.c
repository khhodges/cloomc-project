/*
 * hardware/soc_combined/firmware/main.c
 *
 * Bare-metal RISC-V firmware for the combined Sapphire SoC + Church Machine
 * bitstream on the Ti60F225 devkit.
 *
 * On boot:
 *   1. Writes UART_CLOCKDIV=53 (57,600 baud at 25 MHz).
 *   2. Sends greeting over UART.
 *   3. Waits for the CM to complete its boot sequence (polls STATUS register).
 *   4. Triggers CM free-run by asserting push_button LOW for 5 ms.
 *   5. Loops, emitting a CALLHOME JSON line every second.
 *      On fault: pulses boot_start immediately (PP250 recovery), then continues.
 *
 * UART:    Sapphire UART0 at 0xF8010000, 57,600 baud (25 MHz crystal, CLOCKDIV=53).
 *          NOTE: The formula is baudRate = clkFreq / (8 × (CLOCKDIV + 1)).
 *          At 25 MHz, CLOCKDIV=53 → 25_000_000 / (8×54) = 57,870 ≈ 57,600 baud.
 *          CONFIRMED WORKING on /dev/ttyUSB2. Do NOT use 115200.
 *
 * APB3:    Church Machine bridge at APB_SLAVE_0_BASE (0xF8100000).
 *
 * APB3 CM bridge register map:
 *   +0x00 CTRL   W/R  [0]=cm_pb (0=pressed, 1=released; default 1)
 *   +0x04 STATUS RO   [0]=boot_complete [1]=fault_valid [2]=fault_latched
 *   +0x08 NIA    RO   [31:0]=next instruction address
 *   +0x0C FAULT  RO   [4:0]=fault code
 *   +0x10 UID_LO R/W  [31:0]=lower 32 bits of 64-bit device UID
 *   +0x14 UID_HI R/W  [31:0]=upper 32 bits of 64-bit device UID
 *   -- Track 4-C (new bitstream only) --
 *   +0x18 FAULT_GT    RO  [31:0] GT word0 that caused the fault
 *   +0x1C FAULT_INSTR RO  [31:0] instruction word at fault NIA
 *   +0x20 FAULT_CR14  RO  [31:0] CR14 word0 at fault time
 *   +0x24 FAULT_STAGE RO  [3:0]  pipeline stage (0=Decode…7=Data R/W)
 *
 * Per-board UID configuration
 * ===========================
 * BOARD_UID_HI and BOARD_UID_LO are compile-time constants that form a
 * 64-bit device identity written into the APB3 bridge UID registers at
 * boot and echoed in every CALLHOME JSON packet.
 *
 * When programming multiple Ti60 boards for the same IDE server, recompile
 * the firmware with a distinct UID pair for each board so the IDE Dashboard
 * can track them as separate devices.  Any non-zero 64-bit value works;
 * a simple scheme is to increment BOARD_UID_LO by 1 per board while keeping
 * BOARD_UID_HI fixed (e.g. 0xC0FFEE00 as a site-specific prefix).
 *
 * Example for board #2:
 *   make CFLAGS="-DBOARD_UID_HI=0xC0FFEE00 -DBOARD_UID_LO=0x00000002"
 */

#include <stdint.h>

/* ── Per-board compile-time UID ────────────────────────────────────────────── */
#ifndef BOARD_UID_HI
#define BOARD_UID_HI  0xC0FFEE01UL   /* upper 32 bits — change per site/batch */
#endif
#ifndef BOARD_UID_LO
#define BOARD_UID_LO  0x00000001UL   /* lower 32 bits — change per board      */
#endif

/* ── Sapphire UART0 registers ─────────────────────────────────────────────── */
/* Base address from soc.h: SYSTEM_UART_0_IO_CTRL = 0xF8010000 */
#define UART_BASE       0xF8010000UL
#define UART_DATA       (*(volatile uint32_t *)(UART_BASE + 0x00))
#define UART_STATUS     (*(volatile uint32_t *)(UART_BASE + 0x04))
#define UART_CLOCKDIV   (*(volatile uint32_t *)(UART_BASE + 0x08))
/* Ti60F225 devkit: 25 MHz crystal → GPIOL_P_18_PLLIN0.
 * The sapphire.v in this project has NO internal PLL; io_systemClk is passed
 * straight through.  When peri.xml has no <efxpt:pll_info> for PLL_TL0, the
 * FPGA runs on the raw 25 MHz crystal.
 * CLOCKDIV must be written explicitly — hardware resets it to 0x00.
 * Formula: baudRate = clkFreq / (8 × (clockDivider + 1))
 *   25 MHz clock, CLOCKDIV=53  →  25_000_000 / (8×54) = 57,870 ≈ 57,600 baud
 *   50 MHz clock, CLOCKDIV=53  → 115,200 baud  (if PLL_TL0 ×2 is configured)
 * CONFIRMED WORKING: 57,600 baud on /dev/ttyUSB2, full boot + CALLHOME seen. */

/* ── Church Machine APB3 bridge registers ─────────────────────────────────── */
/* IO_APB_SLAVE_0_INPUT = 0xF8100000 per generated soc.h (Sapphire SoC). */
#define CM_APB_BASE     0xF8100000UL
#define CM_CTRL         (*(volatile uint32_t *)(CM_APB_BASE + 0x00))
#define CM_STATUS       (*(volatile uint32_t *)(CM_APB_BASE + 0x04))
#define CM_NIA          (*(volatile uint32_t *)(CM_APB_BASE + 0x08))
#define CM_FAULT        (*(volatile uint32_t *)(CM_APB_BASE + 0x0C))
#define CM_UID_LO       (*(volatile uint32_t *)(CM_APB_BASE + 0x10))
#define CM_UID_HI       (*(volatile uint32_t *)(CM_APB_BASE + 0x14))
/* Track 4-C APB3 telemetry registers — present only with the Track 4-C bitstream.
 * On older bitstreams all four registers read 0x00000000 (harmless). */
#define CM_FAULT_GT     (*(volatile uint32_t *)(CM_APB_BASE + 0x18))
#define CM_FAULT_INSTR  (*(volatile uint32_t *)(CM_APB_BASE + 0x1C))
#define CM_FAULT_CR14   (*(volatile uint32_t *)(CM_APB_BASE + 0x20))
#define CM_FAULT_STAGE  (*(volatile uint32_t *)(CM_APB_BASE + 0x24))

#define CM_STATUS_BOOT_COMPLETE  (1u << 0)
#define CM_STATUS_FAULT_VALID    (1u << 1)
#define CM_STATUS_FAULT_LATCHED  (1u << 2)

#define CM_CTRL_RELEASED  1u   /* push_button idle (active-low, so 1=released) */
#define CM_CTRL_PRESSED   0u   /* push_button asserted */

/* ── Timing ──────────────────────────────────────────────────────────────────
 * Clock: 25 MHz (raw crystal — PLL_TL0 not configured in current peri.xml).
 * If PLL_TL0 is added later (×2 → 50 MHz), change CLK_HZ to 50000000UL.
 *
 * MEASURED on board: delay_loops(CLK_HZ/4) took ~5.75 s, not 1 s.
 * Root cause: volatile uint32_t loop with nop compiles to 6 instructions
 * (load i, addi, store i, nop, compare, bne) on the Sapphire RISC-V in-order
 * pipeline, ≈23 cycles per iteration — not the assumed 4.
 * Calibrated: CLK_HZ / 25 ≈ 1,000,000 iterations × 23 cycles / 25 MHz ≈ 0.92 s.
 */
#define CLK_HZ          25000000UL
#define LOOPS_PER_SECOND (CLK_HZ / 25)

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static void uart_putc(char c)
{
    /*
     * Unconditional write + fixed inter-character delay.
     *
     * Rationale: the Sapphire UART STATUS register layout varies between
     * Efinix IP configurations (bit 0 vs bits[23:16] for TX-ready).  Rather
     * than guess, we write immediately and wait long enough for the transmitter
     * to finish before the next byte is written.
     *
     * 57600 baud, 10 bits/char (8N1): 1 char = 173.6 µs.
     * 3000 NOP iterations ≈ 48000 cycles @ 25 MHz = 1.92 ms — 11× margin.
     */
    UART_DATA = (1u << 8) | (uint32_t)(unsigned char)c;
    for (volatile uint32_t i = 0; i < 3000u; i++) __asm__("nop");
}

static void uart_puts(const char *s)
{
    while (*s)
        uart_putc(*s++);
}

static void uart_puthex32(uint32_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    uart_puts("0x");
    for (int i = 28; i >= 0; i -= 4)
        uart_putc(hex[(v >> i) & 0xF]);
}

/*
 * Emit a 16-char lowercase hex string for a 64-bit value (hi:lo).
 * No "0x" prefix — used inline inside JSON string values.
 */
static void uart_puthex64_raw(uint32_t hi, uint32_t lo)
{
    static const char hex[] = "0123456789abcdef";
    uint32_t words[2] = {hi, lo};
    for (int w = 0; w < 2; w++) {
        uint32_t v = words[w];
        for (int i = 28; i >= 0; i -= 4)
            uart_putc(hex[(v >> i) & 0xF]);
    }
}

static void delay_loops(uint32_t loops)
{
    volatile uint32_t i;
    for (i = 0; i < loops; i++)
        __asm__ volatile("nop");
}

/* ── Firmware version ─────────────────────────────────────────────────────── */
#define FW_MAJOR 1
#define FW_MINOR 2   /* bumped: LOOPS_PER_SECOND calibrated (was /4, now /25) */

/* ── Fault name lookup table ─────────────────────────────────────────────── */
/*
 * Maps CM_FAULT[4:0] codes to human-readable names.
 * Must stay in sync with ChurchSimulator.FAULT_CODES in simulator/simulator.js
 * and _FAULT_NAMES in hardware/soc_combined/callhome_bridge.py.
 */
static const char * const _fault_names[] = {
    /* 0x00 */ "UNKNOWN",
    /* 0x01 */ "PERM_R",        /* 0x02 */ "PERM_W",
    /* 0x03 */ "PERM_X",        /* 0x04 */ "PERM_L",
    /* 0x05 */ "PERM_S",        /* 0x06 */ "PERM_E",
    /* 0x07 */ "NULL_CAP",      /* 0x08 */ "BOUNDS",
    /* 0x09 */ "VERSION",       /* 0x0A */ "SEAL",
    /* 0x0B */ "INVALID_OP",    /* 0x0C */ "TPERM_RSV",
    /* 0x0D */ "DOMAIN_PURITY", /* 0x0E */ "PERM_B",
    /* 0x0F */ "F_BIT",         /* 0x10 */ "STACK_OVERFLOW",
    /* 0x11 */ "ABSENT_OUTFORM",/* 0x12 */ "STACK_CORRUPT",
    /* 0x13 */ "STACK_UNDERFLOW",/* 0x14 */ "UNKNOWN",
    /* 0x15 */ "OUTFORM_CRC",   /* 0x16 */ "OUTFORM_ALLOC",
    /* 0x17 */ "OUTFORM_MINT",  /* 0x18 */ "OUTFORM_HDR",
};
#define FAULT_NAMES_COUNT ((uint32_t)(sizeof(_fault_names) / sizeof(_fault_names[0])))

static const char *fault_code_name(uint32_t code)
{
    if (code < FAULT_NAMES_COUNT)
        return _fault_names[code];
    return "UNKNOWN";
}

/*
 * Emit a machine-parseable CALLHOME JSON line.
 *
 * Format:
 *   CALLHOME:{"board":"Ti60F225","uid":"HHHH...","nia":"0xNNNN",
 *             "boot_ok":1,"boot_reason":R,"fault":F,"fault_code":C,
 *             "fault_name":"NAME","fw_major":1,"fw_minor":1}\r\n
 *
 * Fields:
 *   boot_reason  — 0=cold boot, 2=fault-recovery re-boot (matches simulator)
 *   fault        — 1 if fault_latched sticky bit is set, 0 otherwise
 *   fault_code   — fault code (0..31); valid when fault==1
 *   fault_name   — human-readable code name from _fault_names[]
 */
static void uart_emit_callhome(uint32_t nia, uint32_t status,
                               uint32_t uid_hi, uint32_t uid_lo,
                               uint32_t boot_reason)
{
    uint32_t fault_latched = (status & CM_STATUS_FAULT_LATCHED) ? 1u : 0u;
    uint32_t fault_code    = fault_latched ? (CM_FAULT & 0x1Fu) : 0u;

    uart_puts("CALLHOME:{\"board\":\"Ti60F225\",\"uid\":\"");
    uart_puthex64_raw(uid_hi, uid_lo);
    uart_puts("\",\"nia\":\"");
    uart_puthex32(nia);
    uart_puts("\",\"boot_ok\":1,\"boot_reason\":");
    uart_putc('0' + (char)(boot_reason & 0xFu));
    uart_puts(",\"fault\":");
    uart_putc(fault_latched ? '1' : '0');
    uart_puts(",\"fault_code\":");
    if (fault_code >= 10u)
        uart_putc('0' + (char)(fault_code / 10u));
    uart_putc('0' + (char)(fault_code % 10u));
    uart_puts(",\"fault_name\":\"");
    uart_puts(fault_code_name(fault_code));
    uart_puts("\"");
    if (fault_latched) {
        /* Track 4-C: GT telemetry registers (read 0x00 on old bitstreams — harmless) */
        uart_puts(",\"fault_gt\":\"");
        uart_puthex32(CM_FAULT_GT);
        uart_puts("\",\"fault_instr\":\"");
        uart_puthex32(CM_FAULT_INSTR);
        uart_puts("\",\"fault_cr14\":\"");
        uart_puthex32(CM_FAULT_CR14);
        uart_puts("\",\"fault_stage\":");
        uint32_t stage = CM_FAULT_STAGE & 0xFu;
        if (stage >= 10u)
            uart_putc('0' + (char)(stage / 10u));
        uart_putc('0' + (char)(stage % 10u));
    }
    uart_puts(",\"fw_major\":");
    uart_putc('0' + FW_MAJOR);
    uart_puts(",\"fw_minor\":");
    uart_putc('0' + FW_MINOR);
    uart_puts("}\r\n");
}

/* ── PP250 fault recovery ────────────────────────────────────────────────────
 * Pulse CM_CTRL_PRESSED for 5 ms to assert boot_start.
 *
 * IMPORTANT — the boot ROM ALWAYS re-executes in full:
 *   The PP250 design makes no distinction between cold boot and fault recovery.
 *   CR15 may itself be the cause of the fault; it cannot be trusted or reused.
 *   The namespace may be in a half-written state at the point of fault.
 *   Therefore the full boot ROM sequence runs unconditionally — re-minting
 *   every Golden Token and rebuilding the namespace from the trusted boot image.
 *   boot_complete is only asserted after all capability state is re-established.
 *
 * Why it is still "fast":
 *   The speed advantage over a cold boot is purely that the FPGA bitstream does
 *   NOT need to be reloaded from SPI flash (~100–500 ms).  The BRAM already
 *   holds the boot image from the initial power-on configuration.  The boot ROM
 *   itself (~20 CLOOMC instructions at 25 MHz) completes in a few microseconds.
 *
 * We poll for boot_complete up to 10 ms after releasing boot_start.
 * Returns 1 if boot_complete re-asserted within the window, 0 if timed out.
 */
static uint32_t pp250_fault_recovery(void)
{
    uart_puts("[PP250] FAULT_RST — pulsing boot_start\r\n");
    CM_CTRL = CM_CTRL_PRESSED;
    delay_loops(CLK_HZ / 200);    /* 5 ms */
    CM_CTRL = CM_CTRL_RELEASED;
    for (uint32_t t = 0; t < 10; t++) {
        if (CM_STATUS & CM_STATUS_BOOT_COMPLETE) {
            uart_puts("[PP250] boot_complete restored\r\n");
            return 1u;
        }
        delay_loops(CLK_HZ / 1000);   /* 1 ms per poll, 10 ms max */
    }
    uart_puts("[PP250] WARNING: boot_complete not seen after 10 ms\r\n");
    return 0u;
}

/* ── Entry point ──────────────────────────────────────────────────────────── */

void main(void)
{
    /*
     * Step 1 — UART baud rate.
     * The Sapphire SoC UART resets CLOCKDIV to 0x00 on power-up.
     * Without this write the UART runs at clkFreq/8 (3.125 Mbaud at 25 MHz)
     * and produces garbage or silence on any standard terminal.
     * CLOCKDIV=53 → 57,600 baud at 25 MHz.  CONFIRMED WORKING.
     */
    UART_CLOCKDIV = 53;

    /*
     * Step 2 — Greeting (uses compile-time UID constants so APB3 is not
     * touched yet; this line appears even if the APB3 address is wrong).
     */
    uart_puts("CHURCH Ti60 SoC+CM v1.2\r\n");
    uart_puts("UID=");
    uart_puthex64_raw(BOARD_UID_HI, BOARD_UID_LO);
    uart_puts("\r\nCONNECT NOW\r\n");

    /*
     * Step 3 — First APB3 access: ensure CM push_button starts released.
     */
    uart_puts("APB_WRITE_START\r\n");
    CM_CTRL = CM_CTRL_RELEASED;
    uart_puts("APB_WRITE_DONE\r\n");

    uint32_t uid_lo = BOARD_UID_LO;
    uint32_t uid_hi = BOARD_UID_HI;

    /* Wait for CM boot_complete — 3-second timeout (3 × ~0.92 s ≈ 2.75 s) */
    uart_puts("Waiting for CM boot...\r\n");
    for (uint32_t t = 0; t < 3; t++) {
        if (CM_STATUS & CM_STATUS_BOOT_COMPLETE) {
            uart_puts("CM boot_complete: 1\r\n");
            break;
        }
        delay_loops(LOOPS_PER_SECOND);
    }

    /* Check for immediate fault at boot */
    if (CM_STATUS & CM_STATUS_FAULT_LATCHED) {
        uart_puts("CM fault at boot! code=");
        uart_puthex32(CM_FAULT & 0x1F);
        uart_puts(" name=");
        uart_puts(fault_code_name(CM_FAULT & 0x1F));
        uart_puts("\r\n");
    }

    /*
     * Trigger CM free-run by asserting push_button LOW for 5 ms.
     * boot_start only needs a few clock cycles; 5 ms is ample margin.
     */
    uart_puts("Asserting CM free-run kick...\r\n");
    CM_CTRL = CM_CTRL_PRESSED;
    delay_loops(CLK_HZ / 200);    /* 5 ms */
    CM_CTRL = CM_CTRL_RELEASED;
    uart_puts("CM free-run kick released.\r\n");

    /* boot_reason: 0=cold, 2=fault-recovery (matches simulator Tunnel.Register) */
    uint32_t boot_reason = 0u;

    /* Monitor loop: emit CALLHOME every second */
    uart_puts("Monitoring CM NIA (Ctrl+C to stop host terminal):\r\n");
    uint32_t iter = 0;
    for (;;) {
        if ((iter % 20) == 0)
            uart_puts("CHURCH Ti60 SoC+CM v1.2\r\n");
        iter++;

        uart_puts("HB=");
        uart_puthex32(iter - 1);
        uart_puts("\r\n");

        uint32_t nia    = CM_NIA;
        uart_puts("NIA_READ_OK\r\n");
        uint32_t status = CM_STATUS;
        uart_puts("STATUS_READ_OK\r\n");

        uart_puts("NIA=");
        uart_puthex32(nia);

        if (status & CM_STATUS_FAULT_LATCHED) {
            uint32_t fc = CM_FAULT & 0x1Fu;
            uart_puts(" FAULT=");
            uart_puthex32(fc);
            uart_puts(" (");
            uart_puts(fault_code_name(fc));
            uart_puts(")\r\n");

            /* PP250 instant recovery: pulse boot_start, CM re-runs 3 boot
             * instructions in 240 ns, boot_complete re-asserts.            */
            if (pp250_fault_recovery()) {
                boot_reason = 2u;
                /* Re-read NIA and status after recovery */
                nia    = CM_NIA;
                status = CM_STATUS;
            }
        } else {
            uart_puts("\r\n");
        }

        uart_emit_callhome(nia, status, uid_hi, uid_lo, boot_reason);

        delay_loops(LOOPS_PER_SECOND);
    }
}
