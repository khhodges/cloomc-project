#!/usr/bin/env python3
"""Configure church_ti60_f225.peri.xml via the Efinity DesignAPI.

Usage (from the Efinity project directory that contains church_ti60_f225.peri.xml):

  PYTHONPATH=$HOME/efinity/2025.2/lib:$HOME/efinity/2025.2/pt/bin \\
  EFXPT_HOME=$HOME/efinity/2025.2/pt \\
    $HOME/efinity/2025.2/bin/python3.11 \\
    <path-to-this-script>/setup_ti60_peri.py

Pin map (confirmed from Ti60F225_kit.isf reference designs):
  clk         B2  = 25 MHz on-board crystal  → CLKMUX_T ROUTE0 (Phase A, no PLL)
  uart_tx     H14 = GPIOR_P_11  (external header, not FT4232H)
  uart_rx     G13 = GPIOR_P_12  (external header, plain GPIOR, bank 3A, adj to H14)
  push_button A7  = GPIOT_N_06  USER_PB active-low (weak pull-up)
  led0        K14 = USER_LED[0]
  led1        J15 = USER_LED[1]
  led2        H10 = USER_LED[2]
  led3        J14 = USER_LED[3]

NOTE: The Ti60F225 devkit has NO UART path to the FT4232H.
      The FT4232H is used only for JTAG programming/debug.
      uart_tx/rx are routed to GPIO pins for use with an external
      USB-UART adapter if needed.

Clock: Phase A — B2 25 MHz crystal → CLKMUX_T ROUTE0 (no PLL).
       Efinity 2025.2 does NOT support create_input_clock_gpio for dedicated
       GCLK pins (B2 = GPIOT_P_07_CLK4_P).  The correct approach is to route
       the clock through the CLKMUX_T block's ROUTE0 output.  This is done by
       a post-processing XML edit after DesignAPI save().
       The SDC (ti60_f225.sdc) Phase A constraint (period 40 ns) is active.

Efinity 2025.2 patches required (apply once to your Efinity install):
  See hardware/efinity_2025_2_patches/ for all patch scripts and README.
"""

import sys
import os
import re

sys.path.insert(0, os.path.join(os.environ.get("EFXPT_HOME", ""), "bin"))

from api_service.design import DesignAPI

PERI_XML = "church_ti60_f225.peri.xml"

# ── Create a fresh schema-valid peri.xml via the DesignAPI itself.
design = DesignAPI(is_verbose=True)
if hasattr(design, "create"):
    design.create(PERI_XML, "Ti60F225")
    print(f"  created fresh design → {PERI_XML}")
else:
    _SEED = '''<?xml version="1.0" encoding="UTF-8"?>
<efxpt:design_db name="church_ti60_f225" device_def="Ti60F225"
  version="2025.2.0" db_version="20241001"
  last_change_date="Tue Apr 01 00:00:00 2026"
  xmlns:efxpt="http://www.efinixinc.com/peri_design_db"
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:schemaLocation="http://www.efinixinc.com/peri_design_db peri_design_db.xsd ">
  <efxpt:device_info>
    <efxpt:iobank_info>
      <efxpt:iobank name="1A" iostd="1.8 V LVCMOS" is_dyn_voltage="false" mode_sel_name="1A_MODE_SEL"/>
      <efxpt:iobank name="1B" iostd="1.8 V LVCMOS" is_dyn_voltage="false" mode_sel_name="1B_MODE_SEL"/>
      <efxpt:iobank name="2A" iostd="1.8 V LVCMOS" is_dyn_voltage="false" mode_sel_name="2A_MODE_SEL"/>
      <efxpt:iobank name="2B" iostd="1.8 V LVCMOS" is_dyn_voltage="false" mode_sel_name="2B_MODE_SEL"/>
      <efxpt:iobank name="3A" iostd="1.8 V LVCMOS" is_dyn_voltage="false" mode_sel_name="3A_MODE_SEL"/>
      <efxpt:iobank name="3B" iostd="1.8 V LVCMOS" is_dyn_voltage="false" mode_sel_name="3B_MODE_SEL"/>
      <efxpt:iobank name="4A" iostd="1.8 V LVCMOS" is_dyn_voltage="false" mode_sel_name="4A_MODE_SEL"/>
      <efxpt:iobank name="4B" iostd="1.8 V LVCMOS" is_dyn_voltage="false" mode_sel_name="4B_MODE_SEL"/>
      <efxpt:iobank name="BL" iostd="3.3 V LVCMOS" is_dyn_voltage="false" mode_sel_name="BL_MODE_SEL"/>
      <efxpt:iobank name="BR" iostd="3.3 V LVCMOS" is_dyn_voltage="false" mode_sel_name="BR_MODE_SEL"/>
      <efxpt:iobank name="TL" iostd="3.3 V LVCMOS" is_dyn_voltage="false" mode_sel_name="TL_MODE_SEL"/>
      <efxpt:iobank name="TR" iostd="3.3 V LVCMOS" is_dyn_voltage="false" mode_sel_name="TR_MODE_SEL"/>
    </efxpt:iobank_info>
  </efxpt:device_info>
  <efxpt:gpio_info>
    <efxpt:global_unused_config unused_gpio="register" pull_option="none"
      bus_hold="false" is_dyn_voltage="false" dynamic_config="false"/>
  </efxpt:gpio_info>
</efxpt:design_db>
'''
    with open(PERI_XML, "w", encoding="UTF-8") as _f:
        _f.write(_SEED)
    print(f"  wrote seed peri.xml (fallback) → {PERI_XML}")
    design.load(PERI_XML)

for bank in ["1A", "1B", "2A", "2B", "3A", "3B", "4A", "4B"]:
    design.set_device_property(bank, "DYNAMIC_VOLTAGE", "0", "IOBANK")
    design.set_mode_sel_name(bank, f"{bank}_MODE_SEL", bank)
    design.set_device_property(bank, "VOLTAGE", "1.8", "IOBANK")
for bank in ["BL", "BR", "TL", "TR"]:
    design.set_device_property(bank, "DYNAMIC_VOLTAGE", "0", "IOBANK")
    design.set_mode_sel_name(bank, f"{bank}_MODE_SEL", bank)
    design.set_device_property(bank, "VOLTAGE", "3.3", "IOBANK")

# ── GPIO signals (NOT clk — clock goes through CLKMUX, not gpio_info) ─────────
design.create_output_gpio("uart_tx")
design.create_input_gpio("uart_rx")
design.create_input_gpio("push_button")
design.create_output_gpio("led0")
design.create_output_gpio("led1")
design.create_output_gpio("led2")
design.create_output_gpio("led3")

design.set_property("push_button", "PULL_OPTION", "WEAK_PULLUP")

design.assign_pkg_pin("uart_tx",     "H14")
design.assign_pkg_pin("uart_rx",     "G13")
design.assign_pkg_pin("push_button", "A7")
design.assign_pkg_pin("led0",        "K14")
design.assign_pkg_pin("led1",        "J15")
design.assign_pkg_pin("led2",        "H10")
design.assign_pkg_pin("led3",        "J14")
# ─────────────────────────────────────────────────────────────────────────────

design.save()
print(f"  DesignAPI save done → {PERI_XML}")

# ── Post-process: route clk through CLKMUX_T ROUTE0 ──────────────────────────
# Efinity 2025.2 does not allow dedicated GCLK pins (B2 = GPIOT_P_07_CLK4_P)
# as plain gpio_info entries.  The working solution is to assign the clock
# signal name "clk" to the CLKMUX_T block's ROUTE0 output pin.  The physical
# pin B2 is implicitly the CLKMUX_T source — no assign_pkg_pin needed.
#
# clkmux_rule_core_clock_pin and clkmux_rule_core_clock_static_mux are bypassed
# in the patched Efinity 2025.2 install (see hardware/efinity_2025_2_patches/).
with open(PERI_XML, encoding="UTF-8") as f:
    xml = f.read()

# Find the CLKMUX_T block and set the first ROUTE0 pin name to "clk"
# The schema produces: <efxpt:pin name="" type_name="ROUTE0" .../>
# We want:             <efxpt:pin name="clk" type_name="ROUTE0" .../>
clkmux_start = xml.find('<efxpt:clkmux name="CLKMUX_T"')
if clkmux_start == -1:
    print("WARNING: CLKMUX_T block not found — cannot set clock route")
else:
    clkmux_end = xml.find('</efxpt:clkmux>', clkmux_start) + len('</efxpt:clkmux>')
    section = xml[clkmux_start:clkmux_end]

    # Set clk on ROUTE0 (first occurrence of ROUTE0 in this section)
    section = re.sub(
        r'(<efxpt:pin name=")("  type_name="ROUTE0"|" type_name="ROUTE0")',
        r'\1clk\2',
        section, count=1
    )
    xml = xml[:clkmux_start] + section + xml[clkmux_end:]

    with open(PERI_XML, "w", encoding="UTF-8") as f:
        f.write(xml)
    print(f"  CLKMUX_T ROUTE0 set to 'clk' → {PERI_XML}")

print(f"SUCCESS — {PERI_XML} written")
