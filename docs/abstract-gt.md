# Abstract GTs — Self-Describing Device Capabilities

*Introduced in Task #406.*

## Overview

An **Abstract GT** is a 32-bit capability word whose entire authority is
encoded in the word itself.  It needs no namespace table entry and no physical
lump in memory — the GT _is_ the capability.

Compare with an Inform GT (type=0b01) which names an NS slot:

| Property          | Inform GT    | Abstract GT      |
|-------------------|-------------|-----------------|
| NS table entry    | Required    | **None**        |
| Physical lump     | Required    | **None**        |
| Authority storage | NS entry    | GT word itself  |
| mLoad required    | Yes         | **No**          |
| DREAD/DWRITE path | mLoad → mem | Abstract Manager|

For 6 LEDs:
- **Before (Inform):** 1 NS slot + 1 lump (≥ 64 words) + 6 GTs all pointing at NS slot 12
- **After (Abstract):** 0 NS slots + 0 lump bytes + 6 self-describing GTs

---

## Word Layout

```
 31  30  29  28  27 | 26  25 | 24  23 | 22  ..  16 | 15  ..  0
[   ab_type (5b)   ] [R] [W] [ type ] [  gt_seq  ] [  ab_data  ]
      32 types          I/O   = 0b11     version     16-bit payload
```

| Field     | Bits    | Width | Notes |
|-----------|---------|-------|-------|
| `ab_type` | [31:27] | 5     | Abstract category (32 possible) |
| `R`       | [26]    | 1     | Read permission |
| `W`       | [25]    | 1     | Write permission |
| `type`    | [24:23] | 2     | = `0b11` — identifies Abstract GT |
| `gt_seq`  | [22:16] | 7     | Version/generation counter |
| `ab_data` | [15:0]  | 16    | Device-specific payload |

**X, L, S, E, B are repurposed as `ab_type` bits and must never be treated as
permissions on an Abstract GT.**  `createAbstractGT()` and `create_abstract_gt()`
only accept R and W.

---

## Abstract Type Registry (`ab_type`)

| `ab_type` | Category      | Description |
|-----------|---------------|-------------|
| `0x00`    | **I/O**       | Hardware device pins and registers |
| `0x01`    | **M-Elevation** | Sets CRn(M=1) — the M Abstraction |
| `0x02–0x1F` | Reserved   | Future abstract categories |

---

## I/O Sub-Encoding (`ab_type = 0x00`)

```
ab_data[15:8]  =  device_class   (what kind of device)
ab_data[7:0]   =  device_data    (register or pin index within the device)
```

| `device_class` | Device  | `device_data` |
|----------------|---------|---------------|
| `0x01`         | LED     | Pin index 0–5 |
| `0x02`         | UART    | Register offset: TX=0, STATUS=1, RX=2 |
| `0x03`         | Button  | Always 0 |
| `0x04`         | Timer   | TICKS_LO=0 … CTL=4 |
| `0x05`         | Display | DMA register offset |

### LED example — c-list slots 8–13

```
Slot 8:  Abstract GT  ab_type=I/O  R|W  LED[0]  → 0x07800100
Slot 9:  Abstract GT  ab_type=I/O  R|W  LED[1]  → 0x07800101
Slot 10: Abstract GT  ab_type=I/O  R|W  LED[2]  → 0x07800102
Slot 11: Abstract GT  ab_type=I/O  R|W  LED[3]  → 0x07800103
Slot 12: Abstract GT  ab_type=I/O  R|W  LED[4]  → 0x07800104
Slot 13: Abstract GT  ab_type=I/O  R|W  LED[5]  → 0x07800105
```

Word breakdown for `0x07800100` (LED[0]):
- bits[31:27] = `0b00000` → ab_type = 0x00 (I/O)
- bit[26]     = `1`       → R permission
- bit[25]     = `1`       → W permission
- bits[24:23] = `0b11`    → type = Abstract
- bits[22:16] = `0b0000000` → gt_seq = 0
- bits[15:8]  = `0x01`    → device_class = LED
- bits[7:0]   = `0x00`    → device_data = pin 0

---

## Instruction Routing

### DREAD / DWRITE

When the capability register holds an Abstract GT (type bits = `0b11`), the
Abstract Manager intercepts **before** any mLoad call:

```
DREAD  DR1, CR3, 0    ; CR3 holds Abstract LED[0] GT
  → Abstract Manager → _dispatchAbstractDread
  → Read LED[0] state → DR1 ← (this.ledBits >> 0) & 1
  ; No mLoad, no memory access
```

```
DWRITE DR1, CR3, 0    ; DR1=1 → turn LED[0] ON
  → Abstract Manager → _dispatchAbstractDwrite
  → this.ledBits |= (1 << 0)
  ; No mLoad, no memory access
```

Permission checks happen inside the Abstract Manager (R for DREAD, W for
DWRITE), not through mLoad.

### CALL

CALL on an Abstract GT is `INVALID_OP`.  Abstract GTs carry no lump, so
there is nothing to execute.  Use DREAD/DWRITE for I/O devices.

---

## M Abstraction (`ab_type = 0x01`)

The M Abstraction allows privileged inspection and modification of a 5-word
capability register through the DR window.  Only CR15 can have M=1.  See
`.local/tasks/abstract-gt-encoding.md` for the full M API design.

Hardware implementation of the CR→DR M-window is tracked in Task #432.

---

## Helper Functions

### JavaScript (simulator.js)

```js
// Create an Abstract LED GT for LED pin 2
const ab_data = (ChurchSimulator.DEVICE_CLASS_LED << 8) | 2;
const gt = sim.createAbstractGT(ChurchSimulator.AB_TYPE_IO, {R:1, W:1}, 0, ab_data);

// Parse an Abstract GT word
const info = sim.parseAbstractGT(gt);
// { ab_type: 0, R: 1, W: 1, type: 3, gt_seq: 0, ab_data: 0x0102,
//   device_class: 1, device_data: 2 }
```

### Python (server/boot_image.py)

```python
from server.boot_image import (
    create_abstract_gt, AB_TYPE_IO, DEVICE_CLASS_LED
)

ab_data = (DEVICE_CLASS_LED << 8) | 2   # LED pin 2
gt = create_abstract_gt(AB_TYPE_IO, {"R": 1, "W": 1}, 0, ab_data)
# → 0x07800102
```

---

## Format Tag

`BOOT_IMAGE_FORMAT_TAG = 0xB0070406` — bumped from `0x0396` (Task #396) when
Abstract LED GTs replaced the Inform LED GTs in c-list slots 8–13.  Both
`server/boot_image.py` and `simulator/simulator.js` must agree on this value.

---

## Related Tasks

| Task | Description |
|------|-------------|
| #406 | This task — Abstract GT encoding, LED c-list slots 8–13 |
| #430 | Amaranth HDL core changes (FPGA synthesis) |
| #431 | UART, Button, Timer as Abstract GTs |
| #432 | Full hardware implementation of the CR→DR M-window |
