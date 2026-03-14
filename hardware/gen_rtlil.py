import os
import sys
from amaranth import ClockSignal
from amaranth.back.rtlil import convert
from .tang_nano_20k import ChurchTangNano20K


def generate_rtlil(output_dir="build"):
    os.makedirs(output_dir, exist_ok=True)

    top = ChurchTangNano20K(clk_freq=27_000_000, baud=115200, sim_mode=False)

    ports = [
        top.uart_tx, top.uart_rx, top.push_button,
        ClockSignal("sync"),
    ] + [led for i, led in enumerate(top.led) if i != 3]

    rtlil_text = convert(top, ports=ports)

    output_path = os.path.join(output_dir, "church_tang_nano_20k.il")
    with open(output_path, "w") as f:
        f.write(rtlil_text)

    print(f"Generated: {output_path}")
    print(f"  File size: {len(rtlil_text):,} bytes")
    print(f"  Lines: {rtlil_text.count(chr(10)):,}")

    return output_path


if __name__ == "__main__":
    output_dir = "build"
    for arg in sys.argv[1:]:
        if not arg.startswith("--"):
            output_dir = arg

    generate_rtlil(output_dir)
