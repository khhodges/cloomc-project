/*
 * Church Machine — BL702 USB CDC ACM class registration
 * =======================================================
 *
 * This file contains the USB descriptor tables and CDC ACM class setup that
 * must be registered with the CherryUSB device stack before usb_dc_init()
 * is called.  It is compiled alongside bl702_firmware.c.
 *
 * SDK: bl_mcu_sdk with CONFIG_CHERRYUSB=1, CONFIG_CHERRYUSB_DEVICE_CDC=1.
 * SDK path: components/usb_stack/class/cdc/  (CherryUSB CDC ACM class)
 *
 * The three functions declared extern in bl702_firmware.c are defined here:
 *   cdc_acm_init()        — register descriptors + interfaces, call usb_dc_init()
 *   cdc_acm_connected()   — return true when host has opened the virtual COM port
 *   cdc_acm_write()       — write bytes to CDC IN endpoint
 *   cdc_acm_read()        — read bytes from CDC OUT endpoint
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "bflb_platform.h"
#include "usbd_core.h"
#include "usbd_cdc.h"

/* ── Endpoint addresses ───────────────────────────────────────────────────── */
#define CDC_IN_EP     0x81
#define CDC_OUT_EP    0x01
#define CDC_CMD_EP    0x82

#define CDC_MAX_MPS   64   /* full-speed max packet size */
#define CDC_CMD_MPS   8

/* ── USB descriptor tables ────────────────────────────────────────────────── */

#define USBD_VID      0x349B   /* Bouffalo Lab VID */
#define USBD_PID      0x0007   /* BL702 CDC ACM PID */
#define USBD_BCD      0x0100

/*
 * Descriptor layout produced by USB_CONFIG_DESCRIPTOR_INIT + the two CDC
 * interface descriptors (notification interface + data interface).
 * Total config length = 9 (config) + 9 (IAD) + 9+5+5+4+5+7 (CDC ctrl intf)
 *                     + 9+7+7 (CDC data intf) = 75 bytes.
 */
#define USBD_CONFIG_DESC_SIZE  75

static const uint8_t cdc_descriptor[] = {
    /* Device descriptor */
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01,
                               CDC_MAX_MPS, USBD_VID, USBD_PID,
                               USBD_BCD, 0x01, 0x02, 0x03, 0x01),

    /* Configuration descriptor */
    USB_CONFIG_DESCRIPTOR_INIT(USBD_CONFIG_DESC_SIZE, 0x02,
                               0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),

    /* CDC ACM class: two interfaces (notification + data) */
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_CMD_EP, CDC_IN_EP, CDC_OUT_EP,
                            CDC_MAX_MPS, 0x00),

    /* String descriptors */
    USB_LANGID_INIT(USBD_LANGID_STRING),
    /* iManufacturer = 1 */
    0x12, USB_DESCRIPTOR_TYPE_STRING,
    'B', 0, 'o', 0, 'u', 0, 'f', 0, 'f', 0, 'a', 0, 'l', 0, 'o', 0,
    /* iProduct = 2 */
    0x22, USB_DESCRIPTOR_TYPE_STRING,
    'C', 0, 'h', 0, 'u', 0, 'r', 0, 'c', 0, 'h', 0, ' ', 0,
    'M', 0, 'a', 0, 'c', 0, 'h', 0, 'i', 0, 'n', 0, 'e', 0, ' ', 0, 'I', 0,
    /* iSerialNumber = 3: four hex chars — overwritten at runtime in cdc_acm_init() */
    0x0A, USB_DESCRIPTOR_TYPE_STRING,
    '0', 0, '0', 0, '0', 0, '0', 0,

    /* Terminator */
    0x00,
};

/* ── Connection state ─────────────────────────────────────────────────────── */

static volatile bool _connected = false;

/*
 * CherryUSB CDC ACM line-state callback.
 * The host sets DTR (bit 0 of wValue) when it opens the virtual COM port.
 * We track this to know when it is safe to send the call-home packet.
 */
void usbd_cdc_acm_set_control_line_state(struct usb_setup_packet *setup,
                                          uint8_t intf)
{
    (void)intf;
    _connected = (setup->wValue & 0x01) != 0;  /* bit 0 = DTR */
}

bool cdc_acm_connected(void)
{
    return _connected;
}

/* ── Interface / endpoint objects ─────────────────────────────────────────── */

static struct usbd_interface cdc_cmd_intf;
static struct usbd_interface cdc_data_intf;

static struct usbd_endpoint cdc_in_ep = {
    .ep_addr = CDC_IN_EP,
    .ep_cb   = usbd_cdc_acm_bulk_in,
};

static struct usbd_endpoint cdc_out_ep = {
    .ep_addr = CDC_OUT_EP,
    .ep_cb   = usbd_cdc_acm_bulk_out,
};

/* ── Write / read helpers ─────────────────────────────────────────────────── */

uint32_t cdc_acm_write(const uint8_t *buf, uint32_t len)
{
    return usbd_ep_write(CDC_IN_EP, buf, len, NULL);
}

uint32_t cdc_acm_read(uint8_t *buf, uint32_t max_len)
{
    uint32_t actual = 0;
    usbd_ep_read(CDC_OUT_EP, buf, max_len, &actual);
    return actual;
}

/* ── Initialisation ───────────────────────────────────────────────────────── */

void cdc_acm_init(void)
{
    usbd_desc_register(cdc_descriptor);

    usbd_cdc_add_acm_interface(&cdc_cmd_intf);
    usbd_add_interface(&cdc_cmd_intf);

    usbd_add_interface(&cdc_data_intf);
    usbd_add_endpoint(&cdc_data_intf, &cdc_in_ep);
    usbd_add_endpoint(&cdc_data_intf, &cdc_out_ep);

    usb_dc_init(0);
}
