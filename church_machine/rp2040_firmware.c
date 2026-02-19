/*
 * Church Machine — RP2040 firmware for pico-ice
 *
 * Role: USB-to-UART bridge between host PC and the iCE40UP5K FPGA
 *       running the Pure Church Lambda Machine.
 *
 * Build: Using pico-ice-sdk (pico-sdk + pico-ice board support)
 *   mkdir build && cd build
 *   cmake -DPICO_BOARD=pico_ice ..
 *   make
 *
 * Features:
 *   1. Auto-program FPGA from flash on power-up
 *   2. USB CDC (virtual serial port) <-> FPGA UART bridge
 *   3. Optional: command interpreter for register reads, memory dumps
 *   4. Future: "Hello Mum" button handler with contact initiation
 *
 * Pin connections (RP2040 <-> iCE40UP5K):
 *   RP2040 UART TX -> FPGA pin 27 (UART_RX)
 *   RP2040 UART RX <- FPGA pin 25 (UART_TX)
 *   RP2040 GPIO    -> FPGA CLK (pin 35, 12 MHz)
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "tusb.h"
#include "boards/pico_ice.h"
#include "ice_usb.h"
#include "ice_fpga.h"

#define FPGA_UART       uart0
#define FPGA_BAUD       115200
#define FPGA_TX_PIN     ICE_RP_UART_TX_PIN
#define FPGA_RX_PIN     ICE_RP_UART_RX_PIN

#define USB_BUF_SIZE    256

static uint8_t usb_rx_buf[USB_BUF_SIZE];
static uint8_t fpga_rx_buf[USB_BUF_SIZE];

void bridge_init(void) {
    uart_init(FPGA_UART, FPGA_BAUD);
    gpio_set_function(FPGA_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(FPGA_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(FPGA_UART, false, false);
    uart_set_format(FPGA_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(FPGA_UART, true);
}

void bridge_task(void) {
    if (tud_cdc_connected()) {
        uint32_t usb_count = tud_cdc_read(usb_rx_buf, USB_BUF_SIZE);
        for (uint32_t i = 0; i < usb_count; i++) {
            uart_putc_raw(FPGA_UART, usb_rx_buf[i]);
        }
    }

    uint32_t fpga_count = 0;
    while (uart_is_readable(FPGA_UART) && fpga_count < USB_BUF_SIZE) {
        fpga_rx_buf[fpga_count++] = uart_getc(FPGA_UART);
    }

    if (fpga_count > 0 && tud_cdc_connected()) {
        tud_cdc_write(fpga_rx_buf, fpga_count);
        tud_cdc_write_flush();
    }
}

int main(void) {
    stdio_init_all();

    ice_fpga_init(48);
    ice_fpga_start();

    bridge_init();

    printf("Church Machine [pico-ice] ready\n");

    while (1) {
        tud_task();
        bridge_task();
    }

    return 0;
}
