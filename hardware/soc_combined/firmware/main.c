/*
 * hardware/soc_combined/firmware/main.c
 *
 * Bare-metal RISC-V firmware for the combined Sapphire SoC + Church Machine
 * bitstream on the Ti60F225 devkit.  Redesigned v2.0.
 *
 * HOW THE CHURCH MACHINE STARTS (from CM Verilog, church_ti60_f225.v)
 * ====================================================================
 * ① FPGA reset deasserts.  boot_start fires after 15 clock cycles (automatic,
 *    no firmware action required).  The CM runs its boot ROM from NIA = 0.
 *
 * ② dbg_boot_complete asserts (<1 ms).  This is a sticky flag that stays HIGH
 *    forever; it is now properly wired to the APB3 STATUS.boot_complete bit.
 *    (Previously wired to cm_led1, the halted-heartbeat, which went LOW when
 *    free-run started — so the firmware never reliably saw boot_complete = 1.)
 *
 * ③ startup_ctr counts ~3 s (75,620,543 cycles @ 25 MHz).  During this time
 *    the CM is halted; LED1 blinks as a heartbeat.
 *
 * ④ CM debug FSM (state 0x00 → 0x01 → 0x02 → 0x03 → ... → 0x06 → 0x07):
 *      Sends boot banner + call-home data over the CM UART (ttyUSB3, 115200 bd).
 *      LED2 turns ON when banner_ever_sent = 1 (during banner send, state 0x02).
 *
 * ⑤ State 0x07: free_run_start = 1, free_run_nia = 0.  CM begins executing
 *    from NIA = 0 (re-runs boot ROM then application).  LED3 heartbeat driven
 *    by the running CM program.  LED1 goes OFF (CM not halted).
 *
 * HOW CM FAULT RECOVERY WORKS
 * ============================
 * Faults: APB3 STATUS[2] (fault_latched) is now wired to dbg_fault_valid pulse.
 *         APB3 FAULT[4:0] carries the fault code.  APB3 NIA shows where.
 *         (Previously all tied to 0.)
 *
 * After a fault the CM debug FSM reports it over ttyUSB3, then halts in
 * state 0x0b, waiting.  To restart free-run execution, the push_button must
 * be held LOW for ≥ 1 second (btn_hold_done fires) → FSM → banner → state 0x07.
 *
 * The firmware holds CM_CTRL = PRESSED for 1.5 s when fault_latched is seen,
 * then releases, then waits 4 s for the CM to re-send its banner and re-enter
 * free-run.  boot_reason is set to 2 (matching simulator Tunnel.Register).
 *
 * NOTE: boot_start can only fire once (boot_triggered latch in the CM).
 * Pulsing push_button briefly does NOT re-trigger boot_start.  The recovery
 * path is purely through the debug FSM's btn_hold_done → state 0x02 → 0x07.
 *
 * UART:    Sapphire UART0 at 0xF8010000, 57,600 baud (25 MHz crystal, CLOCKDIV=53).
 *          Formula: baudRate = clkFreq / (8 × (CLOCKDIV + 1))
 *          25 MHz, CLOCKDIV=53 → 25_000_000 / (8×54) = 57,870 ≈ 57,600 baud.
 *          CONFIRMED WORKING on /dev/ttyUSB2.  Do NOT use 115200.
 *
 * APB3:    Church Machine bridge at APB_SLAVE_0_BASE (0xF8100000).
 *
 * APB3 CM bridge register map:
 *   +0x00 CTRL   W/R  [0]=cm_pb (0=pressed, 1=released; default 1)
 *   +0x04 STATUS RO   [0]=boot_complete [1]=fault_valid [2]=fault_latched
 *   +0x08 NIA    RO   [31:0]=next instruction address  (now live from dbg_nia)
 *   +0x0C FAULT  RO   [4:0]=fault code                (now live from dbg_fault)
 *   +0x10 UID_LO R/W  [31:0]=lower 32 bits of 64-bit device UID
 *   +0x14 UID_HI R/W  [31:0]=upper 32 bits of 64-bit device UID
 */

#include <stdint.h>

/* ── Per-board compile-time UID ────────────────────────────────────────────── */
#ifndef BOARD_UID_HI
#define BOARD_UID_HI  0xC0FFEE01UL
#endif
#ifndef BOARD_UID_LO
#define BOARD_UID_LO  0x00000001UL
#endif

/* ── Sapphire UART0 registers ─────────────────────────────────────────────── */
#define UART_BASE       0xF8010000UL
#define UART_DATA       (*(volatile uint32_t *)(UART_BASE + 0x00))
#define UART_STATUS     (*(volatile uint32_t *)(UART_BASE + 0x04))
#define UART_CLOCKDIV   (*(volatile uint32_t *)(UART_BASE + 0x08))

/* ── Church Machine APB3 bridge registers ─────────────────────────────────── */
#define CM_APB_BASE     0xF8100000UL
#define CM_CTRL         (*(volatile uint32_t *)(CM_APB_BASE + 0x00))
#define CM_STATUS       (*(volatile uint32_t *)(CM_APB_BASE + 0x04))
#define CM_NIA          (*(volatile uint32_t *)(CM_APB_BASE + 0x08))
#define CM_FAULT        (*(volatile uint32_t *)(CM_APB_BASE + 0x0C))
#define CM_UID_LO       (*(volatile uint32_t *)(CM_APB_BASE + 0x10))
#define CM_UID_HI       (*(volatile uint32_t *)(CM_APB_BASE + 0x14))

#define CM_STATUS_BOOT_COMPLETE  (1u << 0)
#define CM_STATUS_FAULT_VALID    (1u << 1)
#define CM_STATUS_FAULT_LATCHED  (1u << 2)

#define CM_CTRL_RELEASED  1u
#define CM_CTRL_PRESSED   0u

/* ── Timing ───────────────────────────────────────────────────────────────────
 * Clock: 25 MHz raw crystal (GPIOL_P_18_PLLIN0, no PLL in peri.xml).
 * CONFIRMED: CLOCKDIV=53 → 57,600 baud; delay calibration = CLK_HZ/25.
 *
 * Measured: volatile-loop + nop = 23 cycles/iter on the Sapphire in-order pipe.
 * CLK_HZ / 25 ≈ 1,000,000 iterations × 23 cycles / 25 MHz ≈ 0.92 s.
 */
#define CLK_HZ          25000000UL
#define LOOPS_PER_SECOND (CLK_HZ / 25)

/* ── Firmware version ─────────────────────────────────────────────────────── */
#define FW_MAJOR 1
#define FW_MINOR 3   /* v1.3: fixed top.v debug port wiring, removed bad kick */

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static void uart_putc(char c)
{
    UART_DATA = (1u << 8) | (uint32_t)(unsigned char)c;
    for (volatile uint32_t i = 0; i < 3000u; i++) __asm__("nop");
}

static void uart_puts(const char *s)
{
    while (*s) uart_putc(*s++);
}

static void uart_puthex32(uint32_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    uart_puts("0x");
    for (int i = 28; i >= 0; i -= 4)
        uart_putc(hex[(v >> i) & 0xF]);
}

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
    for (i = 0; i < loops; i++) __asm__ volatile("nop");
}

/* ── Fault name lookup ────────────────────────────────────────────────────── */
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
#define FAULT_NAMES_COUNT ((uint32_t)(sizeof(_fault_names)/sizeof(_fault_names[0])))

static const char *fault_code_name(uint32_t code)
{
    return (code < FAULT_NAMES_COUNT) ? _fault_names[code] : "UNKNOWN";
}

/* ── CALLHOME JSON emitter ────────────────────────────────────────────────── */
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
    uart_puts("\",\"fw_major\":");
    uart_putc('0' + FW_MAJOR);
    uart_puts(",\"fw_minor\":");
    uart_putc('0' + FW_MINOR);
    uart_puts("}\r\n");
}

/* ── CM fault recovery ────────────────────────────────────────────────────────
 * After a CM fault the debug FSM halts in state 0x0b, waiting.
 * Holding push_button LOW for >= 1 s fires btn_hold_done in the CM Verilog,
 * which transitions the FSM → state 0x02 (re-send banner) → ... → state 0x07
 * (free_run_start, NIA = 0), re-executing from the start of the boot image.
 *
 * This is the ONLY correct recovery path.  A brief 5 ms pulse does nothing
 * (boot_triggered latch prevents boot_start from firing again, and btn_hold_done
 * requires >= 1 s continuous hold).
 *
 * Returns 1 if cm_boot_complete reasserts within the 5-second wait, 0 otherwise.
 */
static uint32_t cm_fault_recovery(void)
{
    uart_puts("[CM] FAULT — holding push_button for 1.5 s (btn_hold_done)\r\n");

    /* Hold push_button LOW for 1.5 s — CM Verilog requires >= 1.0 s */
    CM_CTRL = CM_CTRL_PRESSED;
    delay_loops(LOOPS_PER_SECOND + (LOOPS_PER_SECOND / 2));  /* 1.5 s */
    CM_CTRL = CM_CTRL_RELEASED;

    uart_puts("[CM] Button released — waiting for CM banner + free-run restart\r\n");

    /* Wait up to 5 s for the CM debug FSM to re-send its banner and fire
     * free_run_start (state 0x07), after which dbg_boot_complete re-asserts. */
    for (uint32_t t = 0; t < 5; t++) {
        if (CM_STATUS & CM_STATUS_BOOT_COMPLETE) {
            uart_puts("[CM] boot_complete restored after recovery\r\n");
            return 1u;
        }
        delay_loops(LOOPS_PER_SECOND);
    }
    uart_puts("[CM] WARNING: boot_complete not seen after 5 s recovery wait\r\n");
    return 0u;
}

/* ── Entry point ──────────────────────────────────────────────────────────── */

void main(void)
{
    /* ----------------------------------------------------------------
     * Step 1 — UART baud rate.
     * Sapphire SoC UART resets CLOCKDIV to 0x00 on power-up.
     * Without this write the UART runs at clkFreq/8 (3.125 Mbaud).
     * CLOCKDIV=53 → 57,600 baud at 25 MHz.
     * ---------------------------------------------------------------- */
    UART_CLOCKDIV = 53;

    /* ----------------------------------------------------------------
     * Step 2 — Greeting.
     * ---------------------------------------------------------------- */
    uart_puts("CHURCH Ti60 SoC+CM v1.3\r\n");
    uart_puts("UID=");
    uart_puthex64_raw(BOARD_UID_HI, BOARD_UID_LO);
    uart_puts("\r\nCONNECT NOW\r\n");

    /* ----------------------------------------------------------------
     * Step 3 — Release push_button (keep CM_CTRL = RELEASED).
     *
     * The CM starts itself:
     *   boot_start fires 15 cycles after FPGA reset (automatic).
     *   dbg_boot_complete asserts in < 1 ms (now wired to APB3 STATUS bit 0).
     *   The CM debug FSM counts ~3 s, then fires free_run_start.
     *
     * DO NOT pulse CM_CTRL = PRESSED here.  A brief pulse does nothing
     * useful (boot_triggered prevents boot_start from re-firing; btn_hold_done
     * requires >= 1 s continuous hold).  A long hold (>= 1 s) while the CM
     * is running would PAUSE it — exactly the wrong thing at startup.
     * ---------------------------------------------------------------- */
    CM_CTRL = CM_CTRL_RELEASED;

    const uint32_t uid_lo = BOARD_UID_LO;
    const uint32_t uid_hi = BOARD_UID_HI;

    /* ----------------------------------------------------------------
     * Step 4 — Wait for CM boot_complete.
     *
     * dbg_boot_complete (now correctly wired) asserts within < 1 ms of
     * FPGA reset.  The 3-iteration loop (~ 2.75 s) gives ample margin.
     * ---------------------------------------------------------------- */
    uart_puts("Waiting for CM boot_complete...\r\n");
    uint32_t boot_seen = 0u;
    for (uint32_t t = 0; t < 3; t++) {
        if (CM_STATUS & CM_STATUS_BOOT_COMPLETE) {
            boot_seen = 1u;
            uart_puts("CM boot_complete: 1\r\n");
            break;
        }
        delay_loops(LOOPS_PER_SECOND);
    }
    if (!boot_seen) {
        uart_puts("CM boot_complete: timeout (CM debug FSM may still be starting)\r\n");
    }

    /* ----------------------------------------------------------------
     * Step 5 — Wait for CM to reach free-run (~3 s from FPGA reset).
     *
     * The CM debug FSM takes ~3 s from FPGA reset to fire free_run_start.
     * During this time LED2 turns ON (banner_ever_sent = 1) and LED3 starts
     * blinking (CM program heartbeat).  We wait here so the first CALLHOME
     * packet reflects the running state, not the startup-wait state.
     * ---------------------------------------------------------------- */
    uart_puts("Waiting for CM free-run (~3 s startup counter)...\r\n");
    delay_loops(3u * LOOPS_PER_SECOND);
    uart_puts("CM free-run window passed.\r\n");

    uint32_t boot_reason = 0u;  /* 0 = cold boot */

    /* ----------------------------------------------------------------
     * Step 6 — Monitor loop: emit CALLHOME every second.
     * ---------------------------------------------------------------- */
    uart_puts("Monitoring CM (Ctrl+C to stop host terminal):\r\n");
    uint32_t iter = 0;
    for (;;) {
        if ((iter % 20) == 0)
            uart_puts("CHURCH Ti60 SoC+CM v1.3\r\n");
        iter++;

        uint32_t nia    = CM_NIA;
        uint32_t status = CM_STATUS;

        if (status & CM_STATUS_FAULT_LATCHED) {
            uint32_t fc = CM_FAULT & 0x1Fu;
            uart_puts("NIA=");
            uart_puthex32(nia);
            uart_puts(" FAULT=");
            uart_puthex32(fc);
            uart_puts(" (");
            uart_puts(fault_code_name(fc));
            uart_puts(")\r\n");

            uart_emit_callhome(nia, status, uid_hi, uid_lo, boot_reason);

            /* Attempt recovery: hold push_button >= 1 s → btn_hold_done */
            if (cm_fault_recovery()) {
                boot_reason = 2u;  /* fault-recovery re-run */
                /* Re-read after recovery */
                nia    = CM_NIA;
                status = CM_STATUS;
            }
        } else {
            uart_puts("NIA=");
            uart_puthex32(nia);
            uart_puts("\r\n");
            uart_emit_callhome(nia, status, uid_hi, uid_lo, boot_reason);
        }

        delay_loops(LOOPS_PER_SECOND);
    }
}
