"""Generate synthesizable Verilog from the Pure Church Machine Amaranth design."""

import os
import sys
from amaranth.back.verilog import convert
from .core import ChurchCore
from .top import ChurchTop
from .pico_ice import ChurchPicoIce
from .uart_test import UartTestTop


def generate_verilog(output_dir="build"):
    os.makedirs(output_dir, exist_ok=True)

    core = ChurchCore()

    ports = [
        core.imem_addr, core.imem_data, core.imem_valid,
        core.dmem_addr, core.dmem_rd_en, core.dmem_rd_data,
        core.dmem_wr_data, core.dmem_wr_en,
        core.ns_addr, core.ns_rd_en, core.ns_wr_en,
        core.boot_start, core.boot_state, core.boot_complete,
        core.gc_start, core.gc_busy, core.gc_garbage_count,
        core.fault, core.fault_valid,
        core.nia,
    ]

    verilog_text = convert(core, ports=ports)

    output_path = os.path.join(output_dir, "church_core.v")
    with open(output_path, "w") as f:
        f.write(verilog_text)

    print(f"Generated: {output_path}")
    print(f"  File size: {len(verilog_text):,} bytes")
    print(f"  Lines: {verilog_text.count(chr(10)):,}")

    module_count = verilog_text.count("module ")
    print(f"  Verilog modules: {module_count}")

    return output_path


def generate_top_verilog(output_dir="build"):
    os.makedirs(output_dir, exist_ok=True)

    top = ChurchTop(clk_freq=12_000_000, baud=115200)

    ports = [
        top.uart_tx,
        top.led_boot, top.led_run, top.led_fault,
        top.dbg_nia, top.dbg_fault, top.dbg_fault_valid,
        top.dbg_boot_state, top.dbg_boot_complete,
    ]

    verilog_text = convert(top, ports=ports)

    output_path = os.path.join(output_dir, "church_top.v")
    with open(output_path, "w") as f:
        f.write(verilog_text)

    print(f"\nGenerated: {output_path}")
    print(f"  File size: {len(verilog_text):,} bytes")
    print(f"  Lines: {verilog_text.count(chr(10)):,}")

    module_count = verilog_text.count("module ")
    print(f"  Verilog modules: {module_count}")

    return output_path


def generate_pico_ice_verilog(output_dir="build"):
    os.makedirs(output_dir, exist_ok=True)

    top = ChurchPicoIce(clk_freq=12_000_000, baud=115200, sim_mode=False)

    ports = [
        top.uart_tx, top.uart_rx, top.push_button,
        top.led_r, top.led_g, top.led_b,
    ]

    verilog_text = convert(top, ports=ports)

    lines = verilog_text.split('\n')
    patched = []
    in_top_module = False
    rst_removed = False
    skip_next_wire_rst = False
    for line in lines:
        if line.startswith('module top(') and not rst_removed:
            line = line.replace(', rst,', ',')
            in_top_module = True
        if in_top_module and line.strip() == 'input rst;':
            patched.append('  wire rst = 1\'b0;')
            skip_next_wire_rst = True
            rst_removed = True
            in_top_module = False
            continue
        if skip_next_wire_rst and line.strip() == 'wire rst;':
            skip_next_wire_rst = False
            continue
        patched.append(line)
    verilog_text = '\n'.join(patched)

    output_path = os.path.join(output_dir, "church_pico_ice.v")
    with open(output_path, "w") as f:
        f.write(verilog_text)

    print(f"\nGenerated: {output_path}")
    print(f"  File size: {len(verilog_text):,} bytes")
    print(f"  Lines: {verilog_text.count(chr(10)):,}")

    module_count = verilog_text.count("module ")
    print(f"  Verilog modules: {module_count}")

    return output_path


def generate_uart_test_verilog(output_dir="build"):
    os.makedirs(output_dir, exist_ok=True)

    top = UartTestTop(clk_freq=12_000_000, baud=115200)

    ports = [
        top.uart_tx, top.uart_rx, top.push_button,
        top.led_r, top.led_g, top.led_b,
    ]

    verilog_text = convert(top, ports=ports)

    lines = verilog_text.split('\n')
    patched = []
    in_top_module = False
    rst_removed = False
    skip_next_wire_rst = False
    for line in lines:
        if line.startswith('module top(') and not rst_removed:
            line = line.replace(', rst,', ',')
            in_top_module = True
        if in_top_module and line.strip() == 'input rst;':
            patched.append('  wire rst = 1\'b0;')
            skip_next_wire_rst = True
            rst_removed = True
            in_top_module = False
            continue
        if skip_next_wire_rst and line.strip() == 'wire rst;':
            skip_next_wire_rst = False
            continue
        patched.append(line)
    verilog_text = '\n'.join(patched)

    output_path = os.path.join(output_dir, "uart_test.v")
    with open(output_path, "w") as f:
        f.write(verilog_text)

    print(f"\nGenerated: {output_path}")
    print(f"  File size: {len(verilog_text):,} bytes")
    print(f"  Lines: {verilog_text.count(chr(10)):,}")

    module_count = verilog_text.count("module ")
    print(f"  Verilog modules: {module_count}")

    return output_path


if __name__ == "__main__":
    pico_ice = "--pico-ice" in sys.argv
    uart_test = "--uart-test" in sys.argv
    output_dir = "build"
    for arg in sys.argv[1:]:
        if not arg.startswith("--"):
            output_dir = arg

    generate_verilog(output_dir)
    generate_top_verilog(output_dir)

    if pico_ice:
        generate_pico_ice_verilog(output_dir)

    if uart_test:
        generate_uart_test_verilog(output_dir)
