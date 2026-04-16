/*
 * Church Machine — BL702 firmware for Tang Nano 20K IoT
 * =======================================================
 *
 * Role: USB-to-UART bridge between host PC and the Gowin GW2AR-18 FPGA
 *       running the Pure Church Lambda Machine.
 *
 * At power-up the BL702 reads its factory-burned 8-byte eFuse chip ID
 * (unique per die, set by Bouffalo Lab during manufacturing) and transmits
 * the 23-byte Church Machine call-home packet over USB CDC before switching
 * into its normal role as a transparent UART ↔ USB bridge.
 *
 * SDK: bl_mcu_sdk (https://github.com/bouffalolab/bl_mcu_sdk)
 *      Tested branch: master, commit ≥ 2022-09
 *
 * Build: see CMakeLists_bl702.txt in this directory.
 *
 * Pin connections (BL702 ↔ Gowin GW2AR-18 on Tang Nano 20K IoT):
 *   BL702 GPIO14 (UART0 TX) -> FPGA UART RX
 *   BL702 GPIO15 (UART0 RX) <- FPGA UART TX
 *
 * Call-home packet layout (23 bytes):
 *   [0..1]   0xCE 0x11          magic
 *   [2]      0x01               board type: TN20K-IoT
 *   [3]      FW_MAJOR           firmware version major
 *   [4]      FW_MINOR           firmware version minor
 *   [5..8]   build_sig[4]       zero (future use)
 *   [9..16]  uid[8]             8-byte BL702 eFuse die unique ID
 *   [17]     0x00               boot reason: cold
 *   [18]     0x00               last fault code: none
 *   [19..22] 0x00000000         fault NIA: none
 *
 * ACK from host bridge: 2 bytes 0xCE 0x22 (received in bridge loop; not
 * waited on before bridging begins — bridge starts immediately after send).
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "bflb_platform.h"
#include "hal_uart.h"

/*
 * eFuse API
 * ---------
 * bl_mcu_sdk header: drivers/bl702_driver/hal_drv/inc/hal_efuse.h
 *
 * ef_ctrl_get_device_info() reads the BL702's eFuse region that contains the
 * factory-programmed device information, including the 8-byte unique chip ID.
 *
 * The HAL fills an efuse_device_info_type struct; the chipID field is the
 * 8-byte (64-bit) per-die unique identifier that cannot be changed after
 * manufacture.
 */
#include "hal_efuse.h"

/*
 * USB CDC ACM API
 * ---------------
 * bl_mcu_sdk uses a lightweight USB device stack found under:
 *   components/usb_stack/class/cdc/
 *
 * The CDC ACM class must be registered with the USB core before calling
 * usb_dc_init().  We declare the three setup functions below — their
 * implementations live in cdc_acm_init.c (compiled alongside this file).
 *
 * cdc_acm_init()       — registers descriptors, interfaces, endpoints, calls
 *                         usb_dc_init(0) to start enumeration.
 * cdc_acm_connected()  — returns true once the host has opened the COM port
 *                         (DTR set) after enumeration.
 * cdc_acm_write()      — queues bytes on the CDC IN endpoint; returns the
 *                         number of bytes accepted.
 * cdc_acm_read()       — drains bytes from the CDC OUT endpoint.
 */
extern void cdc_acm_init(void);
extern bool cdc_acm_connected(void);
extern uint32_t cdc_acm_write(const uint8_t *buf, uint32_t len);
extern uint32_t cdc_acm_read(uint8_t *buf, uint32_t max_len);

/* ── Constants ────────────────────────────────────────────────────────────── */

#define FW_MAJOR           1
#define FW_MINOR           0
#define BOARD_TYPE_IOT     0x01

#define CALLHOME_MAGIC_0   0xCE
#define CALLHOME_MAGIC_1   0x11
#define CALLHOME_PKT_LEN   23

#define FPGA_UART_ID       UART0_INDEX     /* UART0: GPIO14/15 on TN20K-IoT */
#define FPGA_BAUD          115200

#define USB_ATTACH_TIMEOUT_MS  3000   /* max wait for host to connect */
#define USB_ATTACH_POLL_MS     10

#define BRIDGE_BUF_SIZE    256

/* ── eFuse chip ID ────────────────────────────────────────────────────────── */

static void read_chip_uid(uint8_t uid[8])
{
    efuse_device_info_type dev_info;
    memset(&dev_info, 0, sizeof(dev_info));

    /*
     * ef_ctrl_get_device_info() — HAL wrapper around EF_Ctrl_Read_Device_Info()
     * SDK: drivers/bl702_driver/hal_drv/src/hal_efuse.c
     *
     * On return, dev_info.chipID[0..7] holds the 8-byte factory-burned UID.
     * The value is stable across power cycles and resets.
     */
    ef_ctrl_get_device_info(&dev_info);
    memcpy(uid, dev_info.chipID, 8);
}

/* ── Call-home packet ─────────────────────────────────────────────────────── */

static void send_callhome_packet(const uint8_t uid[8])
{
    uint8_t pkt[CALLHOME_PKT_LEN];

    pkt[0]  = CALLHOME_MAGIC_0;
    pkt[1]  = CALLHOME_MAGIC_1;
    pkt[2]  = BOARD_TYPE_IOT;
    pkt[3]  = FW_MAJOR;
    pkt[4]  = FW_MINOR;
    pkt[5]  = 0x00;                /* build_sig[0] */
    pkt[6]  = 0x00;                /* build_sig[1] */
    pkt[7]  = 0x00;                /* build_sig[2] */
    pkt[8]  = 0x00;                /* build_sig[3] */
    memcpy(&pkt[9], uid, 8);       /* uid[8]: BL702 eFuse die unique ID */
    pkt[17] = 0x00;                /* boot_reason: cold */
    pkt[18] = 0x00;                /* last_fault: none */
    pkt[19] = 0x00;                /* fault_nia[3] MSB */
    pkt[20] = 0x00;
    pkt[21] = 0x00;
    pkt[22] = 0x00;                /* fault_nia[0] LSB */

    /*
     * Wait for the USB host to enumerate the CDC ACM device and open the
     * virtual COM port (DTR asserted).  Without this the packet bytes are
     * discarded before the host driver is ready.
     */
    uint32_t waited = 0;
    while (!cdc_acm_connected() && waited < USB_ATTACH_TIMEOUT_MS) {
        bflb_platform_delay_ms(USB_ATTACH_POLL_MS);
        waited += USB_ATTACH_POLL_MS;
    }

    if (cdc_acm_connected()) {
        cdc_acm_write(pkt, CALLHOME_PKT_LEN);
    }
}

/* ── UART ↔ USB transparent bridge ───────────────────────────────────────── */

static struct device *fpga_uart;
static uint8_t uart_rx_buf[BRIDGE_BUF_SIZE];
static uint8_t usb_rx_buf[BRIDGE_BUF_SIZE];

static void bridge_init(void)
{
    /*
     * Open UART0 on GPIO14 (TX) and GPIO15 (RX).
     * bl_mcu_sdk: hal_uart_init() is called by device_open() when the
     * device is first opened with the uart_param_cfg_t attached via
     * device_control(DEVICE_CTRL_CONFIG, ...).  A simpler path used in SDK
     * examples is uart_register() + device_open():
     */
    uart_register(FPGA_UART_ID, "fpga_uart");
    fpga_uart = device_find("fpga_uart");

    uart_param_cfg_t uart_cfg = {
        .baudrate        = FPGA_BAUD,
        .databits        = UART_DATA_LEN_8,
        .stopbits        = UART_STOP_ONE,
        .parity          = UART_PAR_NONE,
        .bit_order       = UART_MSB_FIRST,
    };
    device_control(fpga_uart, DEVICE_CTRL_CONFIG, &uart_cfg);
    device_open(fpga_uart, DEVICE_OFLAG_RDWR | DEVICE_OFLAG_STREAM_TX | DEVICE_OFLAG_STREAM_RX);
}

static void bridge_task(void)
{
    /* USB CDC OUT → FPGA UART TX */
    uint32_t usb_len = cdc_acm_read(usb_rx_buf, BRIDGE_BUF_SIZE);
    if (usb_len > 0) {
        device_write(fpga_uart, 0, usb_rx_buf, usb_len);
    }

    /* FPGA UART RX → USB CDC IN */
    uint32_t uart_len = device_read(fpga_uart, 0, uart_rx_buf, BRIDGE_BUF_SIZE);
    if (uart_len > 0 && cdc_acm_connected()) {
        cdc_acm_write(uart_rx_buf, uart_len);
    }
}

/* ── Entry point ──────────────────────────────────────────────────────────── */

int main(void)
{
    bflb_platform_init(0);

    /*
     * Read chip UID before USB init — eFuse reads do not depend on USB
     * and complete in microseconds.
     */
    uint8_t uid[8];
    read_chip_uid(uid);

    /*
     * Initialise USB CDC ACM device.
     * cdc_acm_init() (defined in cdc_acm_init.c) registers descriptors,
     * adds the CDC interfaces and endpoints, then calls usb_dc_init(0)
     * to begin USB enumeration.  The host will detect a new serial device.
     */
    cdc_acm_init();

    /* Set up the FPGA UART */
    bridge_init();

    /* Send call-home packet once the host CDC driver is ready */
    send_callhome_packet(uid);

    /* Transparent bridge loop */
    while (1) {
        bridge_task();
    }

    return 0;
}
