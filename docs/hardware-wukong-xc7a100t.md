# Hardware: QMTECH Wukong Artix-7 XC7A100T

**v1.0 — 2026-05-14**
**CONFIDENTIAL**

---

## Board Overview

The QMTECH Wukong Starter Kit carries an Xilinx Artix-7 **XC7A100T-1FGG676C** FPGA.
It is the most capable board in the Church Machine three-board family and the
only one with on-board Ethernet, making it the natural host for the
**Ethernet abstraction** (NS slot 51, token `00003300`).

| Parameter | Value |
|-----------|-------|
| FPGA | Xilinx Artix-7 XC7A100T-1FGG676C |
| System clock | 100 MHz (50 MHz crystal → MMCM ×2) |
| BRAM | 4,860 Kb (≈ 607 KB total) |
| Namespace allocation | 512 KB |
| Toolchain | Vivado 2020+ / Yosys + nextpnr-xilinx |
| Top-level module | `hardware/wukong_xc7a100t.py` |
| Ethernet driver | `hardware/ethernet_xc7a100t.py` |

---

## Ethernet Abstraction (NS Slot 51)

### Role in the Minimal ROM Image

The XC7A100T minimal ROM image contains exactly three LUMPs:

```
ROM
├── NS slot 5   — Namespace LUMP   (Navana + memory management)
├── NS slot 8   — Thread LUMP      (Scheduler + Dijkstra flag)
└── NS slot 51  — Ethernet LUMP    (this abstraction — Application LUMP)
```

The Ethernet abstraction is the **first thing that executes** after boot.
It immediately gives the Church Machine network reach so the Locator can
fetch every other abstraction from the IDE/Mum Tunnel on demand over
Ethernet, replacing UART as the lazy-load transport.  The ROM image is
self-sufficient: no pre-loaded catalogue of abstractions is required.

### Identity

| Field | Value |
|-------|-------|
| Abstraction name | `Ethernet` |
| NS slot | 51 |
| Token | `00003300` |
| LUMP binary | `server/lumps/00003300.lump` |
| Sidecar | `server/lumps/00003300.json` |
| Source | `simulator/cloomc/ethernet.cloomc` |
| `ns_slot_policy` | `"static"` |

### Permissions

Callers receive **E (enter/call)** permission only.  The abstraction does
not expose R/W/X/L/S capabilities to external callers.

### C-list (Internal)

| Slot | CR | Contents | Permissions |
|------|----|----------|-------------|
| 0 | CR7 | Ethernet device GT (`ETH_DEVICE_GT = 0x07800400`) | hardware-backed MMIO |

The device GT is provisioned at boot by the FPGA NS initialiser.  Its
binary representation is stored in the LUMP c-list region (word 63 of
the 64-word lump) as `0x07800400` (Abstract GT, `device_class = ETH = 4`).

---

## Method Table

| Index | Name | Arguments | Returns | Description |
|-------|------|-----------|---------|-------------|
| 0 | `Send` | DR1 = dataGT (reserved, ignored in MMIO model), DR2 = byteLen | DR1 = byteLen committed | Pre-load frame into TX FIFO via ETH_TX_DATA writes, then call Send(byteLen) to arm the transmitter |
| 1 | `Receive` | — | DR1 = 0 (no frame) or N (byte count of pending frame) | Poll for a pending frame; drain it with N sequential ETH_RX_DATA reads |
| 2 | `Connect` | DR1 = IPv4 address (packed, network byte order), DR2 = port number | DR1 = 0 ok, −1 fault | Register a UDP session endpoint |
| 3 | `Status` | — | DR1 = 0 (link down), 1 (link up), 2 (negotiating) | Query hardware link state (direct 3-state value, not bit-flags) |

Only raw Ethernet frames are handled at this layer.  TCP/IP is a future
abstraction built on top of `Ethernet`.

### ABI Convention — DR1/DR2 not DR0/DR1

All CLOOMC abstractions use DR1 and DR2 as the primary working registers
(not DR0/DR1).  This matches the DREAD/DWRITE instruction encoding and is
consistent with the Tunnel abstraction and every other multi-DR method in
the system.  The original task spec table used DR0/DR1 terminology, which
was a documentation error — the compiled instructions and simulator stubs
have always targeted DR1/DR2.

The `Receive()` method returns a single value (byte count in DR1) rather
than the dual-return `(data GT, byte count)` originally described.  The
dual-return contract is not achievable in the MMIO model because ETH_RX_DATA
(offset 7) is a byte FIFO, not a GT-addressable buffer.  Returning the byte
count is the honest MMIO contract; the caller drains the FIFO with N
sequential DREAD reads.  A true data-GT return path requires a BRAM
capability and belongs in a future DMA-capable variant (Task #1165).

---

## LUMP Header

| Field | Value | Notes |
|-------|-------|-------|
| `magic` | `0x1F` | Always — bits[31:27] |
| `n_minus_6` | `0` | Lump size = 2^(0+6) = 64 words |
| `lump_size` | 64 words (256 bytes) | Minimum LUMP |
| `cw` | 13 | 13 instruction words across 4 methods |
| `typ` | `0b00` | Standard lump |
| `cc` | 1 | One C-list entry (Ethernet device GT) |

Header word encoding: `0xF8003401`

```
bits[31:27] = 11111          (magic = 0x1F)
bits[26:23] = 0000           (n_minus_6 = 0 → 64-word lump)
bits[22:10] = 0000000001101  (cw = 13)
bits[9:8]   = 00             (typ = standard)
bits[7:0]   = 00000001       (cc = 1)
```

---

## LUMP Memory Layout (64 words)

```
Word  0      : header word (0xF8003401)
Words  1– 3  : Send    — LOAD CR7,CR6,#0 | DWRITE DR2,CR7,#2 | RETURN
Words  4– 6  : Receive — LOAD CR7,CR6,#0 | DREAD DR1,CR7,#3  | RETURN
Words  7–10  : Connect — LOAD CR7,CR6,#0 | DWRITE DR1,CR7,#4 | DWRITE DR2,CR7,#5 | RETURN
Words 11–13  : Status  — LOAD CR7,CR6,#0 | DREAD DR1,CR7,#1  | RETURN
Words 14–62  : free space (zeroed)
Word  63     : c-list[0] = Ethernet device GT (0x07800400)
```

---

## MMIO Register Map

The Ethernet device GT maps to the `0x40001000` MMIO block on the
XC7A100T.  All registers are 32-bit word-addressed.

| Address | Reg | Name | Access | Description |
|---------|-----|------|--------|-------------|
| `0x40001000` | 0 | `ETH_CTRL` | R/W | bit[0]=reset, bit[1]=enable, bit[2]=loopback |
| `0x40001004` | 1 | `ETH_STATUS` | R | 0 = link down, 1 = link up, 2 = negotiating |
| `0x40001008` | 2 | `ETH_TX_LEN` | R/W | Byte length to transmit; write triggers TX |
| `0x4000100C` | 3 | `ETH_RX_LEN` | R | Byte length of last received frame |
| `0x40001010` | 4 | `ETH_IP_ADDR` | R/W | Packed IPv4 address (network byte order) |
| `0x40001014` | 5 | `ETH_PORT` | R/W | UDP port number |
| `0x40001018` | 6 | `ETH_TX_DATA` | W | Frame TX FIFO — write bytes sequentially |
| `0x4000101C` | 7 | `ETH_RX_DATA` | R | Frame RX FIFO — read bytes sequentially |

---

## PHY Interface

The board uses an RMII (Reduced Media-Independent Interface) PHY.

### RMII Pin Assignments (XDC)

| Signal | Pin | Direction | Description |
|--------|-----|-----------|-------------|
| `rmii_ref_clk` | W12 | In | 50 MHz reference clock from PHY |
| `rmii_crs_dv` | AA11 | In | Carrier sense / data valid |
| `rmii_rxd[0]` | Y11 | In | Receive data bit 0 |
| `rmii_rxd[1]` | Y12 | In | Receive data bit 1 |
| `rmii_txen` | V11 | Out | Transmit enable |
| `rmii_txd[0]` | V12 | Out | Transmit data bit 0 |
| `rmii_txd[1]` | W11 | Out | Transmit data bit 1 |
| `rmii_nrst` | U10 | Out | PHY reset (active-LOW) |
| `mdio_mdc` | U11 | Out | MDIO management clock |
| `mdio_mdio` | V10 | In/Out | MDIO management data |

---

## Simulator Hardware Stub

When running under the JS simulator (no real XC7A100T attached):

| Method | Stub behaviour |
|--------|---------------|
| `Send` | Accepts DR1=dataGT (ignored) and DR2=byteLen; logs to IDE console; returns DR1=byteLen (all bytes "sent"). |
| `Receive` | Returns DR1 = 0 if no frame waiting; DR1 = N (byte count) if a frame has been queued via `simulateEthernetReceive()`. |
| `Connect` | Validates DR1 (non-zero IPv4) and DR2 (port 1–65535); stores endpoint; returns DR1 = 0. |
| `Status` | Returns DR1 = 1 (link up) in simulation; returns DR1 = 2 (negotiating) when link is not yet established. |

The stub is implemented in `simulator/device_abstractions.js` via
`_bindEthernet()` bound to NS slot 51.  It extends
`_deviceState.ethernet` with: `{ linkUp: true, txBuffer: [], rxBuffer: [], endpoint: null }`.

---

## Three-LUMP Boot Sequence (XC7A100T)

```
Power-on
  │
  ▼
Boot ROM (hardware/boot_rom.py)
  B:02  INIT_THRD  — switch to thread context
  B:03  INIT_ABSTR — load Boot.Abstr capability
  B:04  LOAD_NUC   — CALL Salvation (LED-flash demo)
  │
  ▼
Salvation returns
  │
  ▼
Navana.Init() — boot-time NS setup
  Allocates physical memory for:
    - Scheduler + Stack + DijkstraFlag
    - Ethernet driver working buffers (RX/TX ring, 2 KB each)
  │
  ▼
FPGA NS initialiser provisions Abstract GTs:
    NS[51] ← Ethernet device GT (ETH_DEVICE_GT = 0x07800400)
  │
  ▼
Ethernet abstraction (NS slot 51) begins executing:
  Ethernet.Status() → link=up
  Ethernet.Connect(IDE_IP, IDE_PORT) → endpoint registered
  │
  ▼
Locator now calls Ethernet.Send / Ethernet.Receive
  to fetch all remaining abstractions from the IDE/Mum Tunnel
  on demand (lazy load over Ethernet).
```

---

## Hardware Modules

| Module | File | Description |
|--------|------|-------------|
| `EthernetDeviceGT` | `hardware/ethernet_xc7a100t.py` | Abstract GT record — MMIO register map, link state, RX/TX FIFOs |
| `EthernetRMIIDriver` | `hardware/ethernet_xc7a100t.py` | RMII PHY FSM — 100BASE-TX half-duplex frame handler |
| `ChurchWukongXC7A100T` | `hardware/wukong_xc7a100t.py` | Top-level; instantiates `EthernetDeviceGT` as a submodule |

`EthernetDeviceGT` is instantiated by `ChurchWukongXC7A100T` and wired into
the core's MMIO address decoder at base address `0x40001000`.

---

## Board Profile Summary

| Feature | Status |
|---------|--------|
| UART call-home | Supported (existing) |
| Ethernet lazy-load transport | This document |
| WebSerial deployment | Future task |
| Locator retargeted to Ethernet | Future task |
| Tang Nano 20K Ethernet | Out of scope |
| Ti60 F225 Ethernet | Out of scope |

---

## See Also

- [cloomc-foundation.md](cloomc-foundation.md) — Authoritative architectural overview, 3-LUMP starter kit, board profiles
- [locator.md](locator.md) — Absent-lump fetch protocol (Locator uses Ethernet as transport)
- [plan-lazy-load.md](plan-lazy-load.md) — Lazy-load design: Ethernet replaces UART as the fetch channel
- [boot-rom-layout.md](boot-rom-layout.md) — Boot ROM layout (Salvation, DEMO_NAMESPACE)

---
*Confidential — Kenneth Hamer-Hodges — May 2026*
