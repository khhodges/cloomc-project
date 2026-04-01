#!/usr/bin/env python3
"""Configure church_ti60_f225.peri.xml via the Efinity DesignAPI.

Usage (from the Efinity project directory that contains church_ti60_f225.peri.xml):

  PYTHONPATH=$HOME/efinity/2025.2/lib:$HOME/efinity/2025.2/pt/bin \\
  EFXPT_HOME=$HOME/efinity/2025.2/pt \\
    $HOME/efinity/2025.2/bin/python3.11 \\
    <path-to-this-script>/setup_ti60_peri.py

Pin map (confirmed from Ti60F225.db pickle + Ti60F225_kit.isf):
  clk         B8  = GPIOT_P_07_CLK4_P  50 MHz on-board crystal  (GCLK)
  uart_tx     H14 = GPIOR_P_11_CLK8_P  FTDI FT232H → FPGA TX
  uart_rx     M14 = GPIOR_P_02_CDI24   FTDI FT232H → FPGA RX
  push_button A7  = GPIOT_N_06         USER_PB active-low (weak pull-up)
  led0        K14 = USER_LED[0]
  led1        J15 = USER_LED[1]
  led2        H10 = USER_LED[2]
  led3        J14 = USER_LED[3]
"""

import sys
import os

sys.path.insert(0, os.path.join(os.environ.get("EFXPT_HOME", ""), "bin"))

from api_service.design import DesignAPI

PERI_XML = "church_ti60_f225.peri.xml"

design = DesignAPI(is_verbose=True)
design.load(PERI_XML)

for bank in ["1A", "1B", "2A", "2B", "3A", "3B", "4A", "4B"]:
    design.set_device_property(bank, "DYNAMIC_VOLTAGE", "0", "IOBANK")
    design.set_mode_sel_name(bank, f"{bank}_MODE_SEL", bank)
    design.set_device_property(bank, "VOLTAGE", "1.8", "IOBANK")
for bank in ["BL", "BR", "TL", "TR"]:
    design.set_device_property(bank, "DYNAMIC_VOLTAGE", "0", "IOBANK")
    design.set_mode_sel_name(bank, f"{bank}_MODE_SEL", bank)
    design.set_device_property(bank, "VOLTAGE", "3.3", "IOBANK")

design.create_input_gpio("clk")
design.create_output_gpio("uart_tx")
design.create_input_gpio("uart_rx")
design.create_input_gpio("push_button")
design.create_output_gpio("led0")
design.create_output_gpio("led1")
design.create_output_gpio("led2")
design.create_output_gpio("led3")

design.set_property("clk",         "CONN_TYPE",   "GCLK")
design.set_property("push_button", "PULL_OPTION", "WEAK_PULLUP")

design.assign_pkg_pin("clk",         "B8")
design.assign_pkg_pin("uart_tx",     "H14")
design.assign_pkg_pin("uart_rx",     "M14")
design.assign_pkg_pin("push_button", "A7")
design.assign_pkg_pin("led0",        "K14")
design.assign_pkg_pin("led1",        "J15")
design.assign_pkg_pin("led2",        "H10")
design.assign_pkg_pin("led3",        "J14")

design.save()
print(f"SUCCESS — {PERI_XML} written")
