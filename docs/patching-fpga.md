# Patching the Tang Nano 20K FPGA — Complete Guide

This guide covers every step from Amaranth HDL source to running patched code
on a Tang Nano 20K (GW2AR-LV18QN88C8/I7) using the Church Machine IDE.

---

## Prerequisites

### On your local machine (Chromebook / Linux)

1. **OSS CAD Suite** — provides Yosys, nextpnr, gowin_pack, openFPGALoader  
   Download: https://github.com/YosysHQ/oss-cad-suite-build/releases  
   After extracting, activate it:
   ```bash
   source ~/oss-cad-suite/environment
   ```

2. **Python 3 with pyserial** — for the UART bridge script  
   ```bash
   pip3 install pyserial
   ```

3. **Tang Nano 20K** plugged in via USB

4. **Check your USB serial ports:**
   ```bash
   ls /dev/ttyUSB*
   ```
   The FT2232 chip on the Tang Nano creates two ports:
   - Channel A = JTAG (usually `ttyUSB0`) — used by openFPGALoader
   - Channel B = UART (usually `ttyUSB1`) — used by the bridge

---

## Part A: Build the Bitstream

### Step 1 — Generate Verilog (Replit)

This runs in the Replit environment. Amaranth HDL is converted to Verilog:

```bash
python3 -m hardware.gen_verilog --iot build
```

Output:
- `build/church_core_iot.v`
- `build/church_tang_nano_20k_iot.v`

### Step 2 — Synthesize with Yosys (Replit)

Yosys converts the Verilog into a Gowin-specific netlist:

```bash
yosys -p "read_verilog build/church_tang_nano_20k_iot.v; synth_gowin -top top -json build/church_tang_nano_20k_iot.json"
```

Output:
- `build/church_tang_nano_20k_iot.json` (~16 MB netlist)

Check for: `Found and reported 0 problems.` at the end.

### Step 3 — Download files to your local machine

Download these two files from Replit to your local machine:

- `build/church_tang_nano_20k_iot.json` (the synthesized netlist)
- `hardware/tang_nano_20k.cst` (pin constraints)
- `server/local_bridge.py` (bridge script — needed later for patching)

Put them in a working directory on your machine, keeping the folder structure:

```
~/Downloads/IoT/
├── build/
│   └── church_tang_nano_20k_iot.json
├── hardware/
│   └── tang_nano_20k.cst
└── server/
    └── local_bridge.py
```

### Step 4 — Place and Route (local machine)

This step maps the netlist onto the physical FPGA. It needs ~20 GB of RAM
and takes 3–5 minutes.

```bash
cd ~/Downloads/IoT

nextpnr-himbaechel \
    --device GW2AR-LV18QN88C8/I7 \
    --json build/church_tang_nano_20k_iot.json \
    --write build/church_tang_nano_20k_iot_pnr.json \
    -o family=GW2A-18C \
    -o cst=hardware/tang_nano_20k.cst \
    --freq 27
```

Wait for: `Info: Program finished normally.`

Output:
- `build/church_tang_nano_20k_iot_pnr.json`

### Step 5 — Pack the bitstream (local machine)

Converts the placed-and-routed design into a `.fs` bitstream file:

```bash
gowin_pack -d GW2A-18C \
    -o build/church_tang_nano_20k_iot.fs \
    build/church_tang_nano_20k_iot_pnr.json
```

Output:
- `build/church_tang_nano_20k_iot.fs`

### Step 6 — Flash to the Tang Nano 20K (local machine)

Upload the bitstream to the FPGA via JTAG:

```bash
openFPGALoader -b tangnano20k build/church_tang_nano_20k_iot.fs
```

Wait for:
```
Load SRAM: [==================================================] 100.00%
Done
DONE
```

### Step 7 — Verify the LEDs

After flashing, you should see:

| LED | Pin | Meaning | Expected |
|-----|-----|---------|----------|
| led0 | 15 | Boot/Run | Solid ON — boot complete |
| led1 | 16 | Halt | Blinking — core halted (no code loaded) |
| led2 | 19 | Fault | OFF — no fault (good) |
| led3 | 20 | Heartbeat | Blinking ~1 Hz — clock alive |

**One solid + two blinking = success.** The core booted and is waiting for code.

---

## Part B: Connect the Bridge

The bridge is a small Python script that runs on your local machine. It relays
UART traffic between the IDE (in your browser) and the FPGA (plugged into USB).

```
IDE (browser) ──HTTPS──► Bridge (your terminal) ──USB/UART──► Tang Nano 20K
```

### Step 8 — Start the bridge script

In a terminal on your local machine (keep this running):

```bash
cd ~/Downloads/IoT
python3 server/local_bridge.py /dev/ttyUSB1 115200
```

You should see:
```
Church Machine FPGA Bridge (HTTPS)
  Serial : /dev/ttyUSB1 @ 115200 baud
  HTTPS  : https://0.0.0.0:8766
  ChromeOS bridge URL: https://penguin.linux.test:8766
```

If you get `Permission denied` on the serial port:
```bash
sudo chmod 666 /dev/ttyUSB1
```

If pyserial is not installed:
```bash
pip3 install pyserial
```

### Step 9 — Accept the self-signed certificate (one time only)

The bridge uses HTTPS with a self-signed certificate. Chrome needs to trust it.

1. Open a **new Chrome tab**
2. Go to: `https://penguin.linux.test:8766/status`
3. Chrome will show a security warning — **this is normal** (it's your own machine)
4. Click **Advanced**
5. Click **Proceed to penguin.linux.test (unsafe)**
6. You should see: `{"ok": true, "open": true, "port": "/dev/ttyUSB1", "baud": 115200}`

You only need to do this once per browser session.

### Step 10 — Connect the IDE to the bridge

1. **Hard refresh** the IDE page: press **Ctrl + Shift + R**
2. Click the **Bridge** button in the IDE toolbar
3. A popup asks for the bridge URL — the default (`https://penguin.linux.test:8766`) is correct, just press **OK**
4. You should see in the console: `FPGA Bridge: Connected ✓`

---

## Part C: Patch Code to the FPGA

### Step 11 — Write code in the IDE

1. Click on a **CR** (capability register) in the CRs panel — for example, **CR0**
2. Click the **Edit** button to open the code editor
3. Write or modify your Church Machine assembly in the editor

### Step 12 — Patch Simulator (optional test)

Click **Patch** (not Patch FPGA) to test your code in the simulator first.
This assembles the code and writes it into simulator memory. You can step
through it and verify behavior before sending to real hardware.

### Step 13 — Patch FPGA

Click **Patch FPGA**. This does three things:

1. **Assembles** your code and patches the simulator memory
2. **Sends the code** to the FPGA over the bridge (PATCH_LUMP protocol)
3. **Sends a RUN command** to start the core executing from PC=0

You should see a success popup with log output like:
```
CR0  NS[0]  base=0x0100  old cw=6  new cw=6  (max 243)
Simulator patched — 6 words written.
Sending code lump (6 words at 0x0100) to FPGA...
  PATCH_LUMP: addr=0x0100 N=6 CRC=0xABCD — sending 32 bytes...
  Echo OK: addr=0x0100 count=6
FPGA patch confirmed by echo.
Sending RUN command...
  RUN sent — core executing from PC=0.
```

Key things to check:
- **"Echo OK"** = FPGA received and wrote the code
- **"RUN sent"** = core is now executing your code
- If the code triggers a fault, led2 (pin 19) will light up

---

## Troubleshooting

### "Failed to fetch" when connecting Bridge
- Did you accept the self-signed certificate? (Step 9)
- Is the bridge script still running in your terminal?

### "No echo received (0 bytes)"
- Is the bridge connected to the right serial port? Check `ls /dev/ttyUSB*`
- Try `/dev/ttyUSB1` (most common for Channel B UART)
- Restart the bridge after flashing a new bitstream

### "Editor is empty"
- Select a CR first, then click Edit to open the code editor
- Write your assembly code, then click Patch FPGA

### LEDs don't change after patching
- Make sure you see "Echo OK" in the patch log — if not, the code didn't reach the FPGA
- Make sure you see "RUN sent" — without this, the core stays halted
- Check the bridge terminal for errors

### Bridge shows "Serial port not open"
- The serial port may have changed after flashing. Check `ls /dev/ttyUSB*`
- Restart the bridge: `python3 server/local_bridge.py /dev/ttyUSB1 115200`

### Permission denied on serial port
```bash
sudo chmod 666 /dev/ttyUSB1
# Or add yourself to the dialout group (permanent fix):
sudo usermod -aG dialout $USER
# Then log out and back in
```

---

## Quick Reference

| Step | Where | Command / Action |
|------|-------|-----------------|
| 1 | Replit | `python3 -m hardware.gen_verilog --iot build` |
| 2 | Replit | `yosys -p "read_verilog build/church_tang_nano_20k_iot.v; synth_gowin -top top -json build/church_tang_nano_20k_iot.json"` |
| 3 | Local | Download `.json`, `.cst`, and `local_bridge.py` from Replit |
| 4 | Local | `nextpnr-himbaechel --device GW2AR-LV18QN88C8/I7 --json build/church_tang_nano_20k_iot.json --write build/church_tang_nano_20k_iot_pnr.json -o family=GW2A-18C -o cst=hardware/tang_nano_20k.cst --freq 27` |
| 5 | Local | `gowin_pack -d GW2A-18C -o build/church_tang_nano_20k_iot.fs build/church_tang_nano_20k_iot_pnr.json` |
| 6 | Local | `openFPGALoader -b tangnano20k build/church_tang_nano_20k_iot.fs` |
| 7 | Local | Check LEDs: 1 solid + 2 blinking = OK |
| 8 | Local | `python3 server/local_bridge.py /dev/ttyUSB1 115200` |
| 9 | Chrome | Accept cert at `https://penguin.linux.test:8766/status` |
| 10 | IDE | Click Bridge → OK |
| 11 | IDE | Select CR → Edit → write code |
| 12 | IDE | Click Patch (simulator test) |
| 13 | IDE | Click Patch FPGA (sends to hardware + runs) |
