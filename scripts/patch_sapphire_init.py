#!/usr/bin/env python3
"""
scripts/patch_sapphire_init.py

Replace $readmemb calls in sapphire.v with explicit Verilog initial-block
assignments so EFX_MAP can embed firmware bytes in the BRAM INITVAL_
parameters during synthesis.

EFX_MAP on Efinix Titanium ignores $readmemb file references (treats them
as simulation-only) but DOES honour explicit `mem[i] = N;` assignments in
initial blocks.  Without this patch, the system_ramA appears zero-initialised
to EFX_MAP, which then eliminates it entirely via optimize-zero-init-rom,
leaving the CPU bus hardwired to 0 and the firmware never executing.

Usage (run from hardware/soc_combined/):
    python3 ../../scripts/patch_sapphire_init.py \\
        sapphire.v \\
        EfxSapphireSoc.v_toplevel_system_ramA_logic_ram_symbol0.bin \\
        EfxSapphireSoc.v_toplevel_system_ramA_logic_ram_symbol1.bin \\
        EfxSapphireSoc.v_toplevel_system_ramA_logic_ram_symbol2.bin \\
        EfxSapphireSoc.v_toplevel_system_ramA_logic_ram_symbol3.bin

Or via Makefile shortcut:
    make -C hardware/soc_combined patch_sapphire

The script edits sapphire.v in-place.  Re-run make in firmware/ and re-run
this script every time the firmware changes, then re-synthesise.
"""

import sys
import re

SYMBOL_NAMES = [
    "EfxSapphireSoc.v_toplevel_system_ramA_logic_ram_symbol0.bin",
    "EfxSapphireSoc.v_toplevel_system_ramA_logic_ram_symbol1.bin",
    "EfxSapphireSoc.v_toplevel_system_ramA_logic_ram_symbol2.bin",
    "EfxSapphireSoc.v_toplevel_system_ramA_logic_ram_symbol3.bin",
]
RAM_VARS = ["ram_symbol0", "ram_symbol1", "ram_symbol2", "ram_symbol3"]


def read_symbol_bin(path):
    """
    Read a $readmemb binary file.

    Each non-empty, non-comment line is a binary string (e.g. "00010111")
    representing one 8-bit word.  Returns list of int.
    """
    values = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("//") or line.startswith("@"):
                continue
            values.append(int(line, 2))
    return values


def gen_assignments(var_name, values):
    """
    Generate Verilog initial-block assignments for one byte lane.

    Skips entries that are zero — EFX_RAM_5K INITVAL_ defaults to 0,
    so only non-zero bytes need explicit assignment.  Always emits the
    very first entry (address 0) to prevent the whole block being
    optimised away as empty.
    """
    lines = []
    for i, v in enumerate(values):
        if v != 0 or i == 0:
            lines.append(f"        {var_name}[{i}] = 8'h{v:02X};")
    return "\n".join(lines)


def main():
    if len(sys.argv) < 6:
        print(
            "Usage: patch_sapphire_init.py sapphire.v "
            "symbol0.bin symbol1.bin symbol2.bin symbol3.bin",
            file=sys.stderr,
        )
        sys.exit(1)

    sapphire_path = sys.argv[1]
    bin_paths = sys.argv[2:6]

    lanes = []
    for i, path in enumerate(bin_paths):
        print(f"  Reading symbol{i}: {path} ...", end=" ", flush=True)
        v = read_symbol_bin(path)
        nonzero = sum(1 for x in v if x)
        print(f"{len(v)} entries, {nonzero} non-zero")
        lanes.append(v)

    with open(sapphire_path, "r") as f:
        content = f.read()

    original_len = len(content)

    for i in range(4):
        var = RAM_VARS[i]
        fname = SYMBOL_NAMES[i]
        pattern = (
            r'\$readmemb\("'
            + re.escape(fname)
            + r'",'
            + re.escape(var)
            + r"\);"
        )
        replacement = gen_assignments(var, lanes[i])
        new_content, n = re.subn(pattern, replacement, content)
        if n == 0:
            print(
                f"ERROR: $readmemb for symbol{i} not found in {sapphire_path}",
                file=sys.stderr,
            )
            print(
                "  Check that sapphire.v was copied from the Efinix IP and "
                "has not been modified already.",
                file=sys.stderr,
            )
            sys.exit(1)
        content = new_content
        assignments = replacement.count("\n") + 1
        print(f"  Replaced $readmemb symbol{i} → {assignments} initial assignments")

    with open(sapphire_path, "w") as f:
        f.write(content)

    delta = len(content) - original_len
    print(
        f"\nPatched {sapphire_path}  "
        f"(+{delta:,} chars, {len(content):,} total)"
    )
    print(
        "\nNext steps:\n"
        "  1. bash work_syn/run_efx_map.sh   (re-synthesise)\n"
        "  2. bash work_pnr/run_efx_pnr.sh   (place & route)\n"
        "  3. bash work_pgm/run_efx_pgm.sh   (bitstream)\n"
        "  4. sudo ~/oss-cad-suite/bin/openFPGALoader "
        "-b titanium_ti60_f225_jtag -f outflow/church_soc_cm.hex\n"
    )


if __name__ == "__main__":
    main()
