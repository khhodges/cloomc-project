# Church Machine on Tang Nano 20K

## Hardware Platform

The Tang Nano 20K is an FPGA development board from Sipeed featuring:

- **FPGA**: Gowin GW2AR-LV18QN88C8/I7 (GW2A-18C family)
- **Logic**: ~20,736 LUTs
- **BSRAM**: 41,472 bits (block SRAM)
- **Clock**: 27 MHz crystal oscillator
- **USB**: BL616 USB bridge providing UART communication
- **LEDs**: 6 onboard LEDs (active-low)
- **Buttons**: Push button (active-low, active on S1)
- **HDMI**: HDMI output connector
- **Flash**: 32Mbit SPI flash for bitstream storage

## Church Machine Implementation

### Resource Usage

The Church Machine core fits within the GW2A-18C with room to spare:

- 16 Context Registers (128-bit each) mapped to BSRAM
- 16 Data Registers (32-bit each) in LUT RAM
- Namespace table and c-lists in BSRAM (2048 x 32-bit words)
- Full security pipeline (mLoad/mSave) in combinational logic
- Boot ROM (512 words) for initialization sequence

### Pin Assignments

| Signal | Pin | Description |
|--------|-----|-------------|
| clk | 4 | 27 MHz crystal oscillator |
| uart_tx | 17 | UART transmit (to BL616) |
| uart_rx | 18 | UART receive (from BL616) |
| led[0] | 15 | LED 0 (boot status) |
| led[1] | 16 | LED 1 (run/halted) |
| led[2] | 17 | LED 2 (fault indicator) |
| led[3] | 18 | LED 3 (boot complete) |
| led[4] | 19 | LED 4 (halted) |
| led[5] | 20 | LED 5 (stepping) |
| push_button | 88 | Step button (active-low) |

Pin constraints are defined in `hardware/tang_nano_20k.cst`.

### LED Indicators

| LED | Meaning |
|-----|---------|
| LED 0 | ON during boot, OFF when boot complete |
| LED 1 | ON when running, blinks when halted (1 Hz heartbeat) |
| LED 2 | ON when fault detected |
| LED 3 | ON until boot complete |
| LED 4 | ON when halted |
| LED 5 | ON during single-step |

### UART Protocol

- Baud rate: 115,200
- Data bits: 8
- Stop bits: 1
- No parity
- No flow control

On boot, the UART outputs:
```
CHURCH TN20K v1.0
<NIA as hex>HALT
```

On each button press (single-step):
```
S:<NIA as hex>HALT
```

On fault:
```
S:<NIA as hex>F:<fault code as hex>HALT
```

## Build Toolchain

### Prerequisites

Install the open-source FPGA toolchain:

**Option 1: oss-cad-suite (recommended)**

Download from [https://github.com/YosysHQ/oss-cad-suite-build](https://github.com/YosysHQ/oss-cad-suite-build). Includes yosys, nextpnr-gowin, gowin_pack, and openFPGALoader.

**Option 2: Build from source**

```bash
sudo apt install yosys
# Build nextpnr-gowin from source
# Build gowin_pack from the apicula project
```

### Build Steps

```bash
cd hardware

# Step 1: Generate Verilog from Amaranth HDL
make verilog

# Step 2: Synthesize with Yosys
make synth

# Step 3: Place and route with NextPNR-Gowin
make pnr

# Step 4: Create bitstream with gowin_pack
make pack

# Step 5: Upload to Tang Nano 20K
make prog
```

Or run the full chain:

```bash
make all    # verilog → synth → pnr → pack
make prog   # upload bitstream
```

### Build Targets

| Target | Command | Output |
|--------|---------|--------|
| Verilog generation | `make verilog` | `build/church_tang_nano_20k.v` |
| Synthesis | `make synth` | `build/church_tang_nano_20k.json` |
| Place & Route | `make pnr` | `build/church_tang_nano_20k.fs` |
| Bitstream | `make pack` | `build/church_tang_nano_20k.fs` |
| Upload | `make prog` | Programmed to FPGA |
| Clean | `make clean` | Removes build artifacts |
| Report | `make report` | Shows LUT/DFF/BSRAM usage |

### Verilog Generation

The Amaranth HDL design is converted to Verilog using:

```bash
cd .. && python -m hardware.gen_verilog
```

This generates `build/church_tang_nano_20k.v` containing the complete synthesizable design.

## Hardware Architecture

### Memory Map

```
Address Range    Contents
0x0000-0x01FF    Boot ROM (512 words, instruction memory)
0x0200-0x07FF    General data memory (BSRAM)
0xFD00-0xFDFF    Namespace table (256 entries x 3 words)
0xFE00-0xFEFF    Device I/O (UART, LED, Button, Timer)
```

### Boot Sequence

1. BSRAM initialization writes namespace table and c-list entries
2. Boot delay (15 cycles in simulation, 3 seconds on hardware)
3. Boot sequence initializes CR6, CR14, CR8, CR15
4. Banner message sent via UART
5. Machine enters HALTED state, awaiting button press

### Single-Step Mode

The Tang Nano 20K implementation starts in halted mode. Press the push button to execute one instruction. After each step:

1. The NIA (Next Instruction Address) is sent via UART
2. If a fault occurred, the fault code is also sent
3. The machine returns to HALTED state

This allows interactive debugging via a serial terminal.

## Web IDE Integration

The simulator IDE includes hardware deployment features:

1. **Download .bin** — Generates a binary image for the Tang Nano 20K boot ROM
2. **Deploy to Tang** — Uses WebSerial API to upload via the BL616 USB bridge

### WebSerial Requirements

- Chrome or Edge browser (WebSerial API support)
- Tang Nano 20K connected via USB
- BL616 appears as a serial port

### Binary Format

The hardware binary format encodes assembled instructions for the boot ROM. The `hw_binary.js` module in the simulator handles:

- Instruction encoding to 32-bit words
- Boot ROM header generation
- Namespace initialization data
- UF2 format output for USB upload
