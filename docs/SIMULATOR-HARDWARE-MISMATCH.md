# Simulator ↔ Hardware Mismatch Report

**Status**: ✅ FIXED — All GT type names corrected to match hardware definitions  
**Date**: March 29, 2026 (fixed March 30, 2026)  
**Files**: `simulator/simulator.js` vs `hardware/hw_types.py`

---

## CRITICAL ISSUE: GT Type Mapping

### Hardware Definition (hw_types.py)
```python
GT_TYPE_NULL     = 0b00  # 0
GT_TYPE_INFORM   = 0b01  # 1
GT_TYPE_OUTFORM  = 0b10  # 2
GT_TYPE_ABSTRACT = 0b11  # 3
```

### Simulator Definition (simulator.js, line ~157)
```javascript
typeName: ['NULL','Real','Abstract','???'][type & 3],
```

Maps to:
- type=0 → 'NULL' ✅ CORRECT
- type=1 → 'Real' ❌ WRONG (should be 'Inform')
- type=2 → 'Abstract' ❌ WRONG (should be 'Outform')
- type=3 → '???' ❌ WRONG (should be 'Abstract')

### Impact

| Type | Hardware | Simulator | Mismatch |
|------|----------|-----------|----------|
| 1 | INFORM | Real | ❌ Naming only |
| 2 | OUTFORM | Abstract | ❌ **LOGIC ERROR** |
| 3 | ABSTRACT | ??? | ❌ **UNDEFINED** |

**Severity**: 🔴 **CRITICAL**
- Type 2 logic is executing as if it's Abstract (wrong behavior)
- Type 3 is treated as unknown (will cause faults or wrong dispatch)
- Boot ROM and tutorials use Abstract GTs (type 3) but simulator treats them as '???'

---

## Secondary Issue: Type Comparisons in Simulator

**Location**: `simulator/simulator.js` lines 1171, 1201, 1580

```javascript
// Line 1171: Checks if type === 1 (Real/Inform)
if (cr7Parsed.type === 1 && cr7Parsed.permissions.X) {
    // ...treats as Inform/Real
}

// Line 1201: Checks if type === 2 (Abstract/Outform in simulator = Abstract)
if (cr1Parsed.type === 2) {
    // ...treats as Abstract (WRONG — this is Outform!)
}

// Line 1580: Checks both types 1 and 2
if (cr7Parsed.type === 1 || cr7Parsed.type === 2) {
    // ...mixed logic
}
```

**Problem**: Logic assumes type 2 is Abstract, but hardware defines type 2 as **Outform**.

---

## What Needs to be Fixed

### File: `simulator/simulator.js`

**Change 1** (line ~157): Update typeName array
```javascript
// OLD:
typeName: ['NULL','Real','Abstract','???'][type & 3],

// NEW:
typeName: ['NULL','Inform','Outform','Abstract'][type & 3],
```

**Change 2** (line ~1171): Update type comparison comments/logic
```javascript
// If checking for Inform (type 1), rename or comment clearly:
if (cr7Parsed.type === 1 && cr7Parsed.permissions.X) {
    // Inform GT with X permission — treat as code reference
```

**Change 3** (line ~1201): Fix Outform/Abstract confusion
```javascript
// OLD:
if (cr1Parsed.type === 2) {
    // ...assumes Abstract

// NEW:
if (cr1Parsed.type === 3) {  // Type 3 is Abstract, not type 2
    // ...Abstract GT logic
}
```

**Change 4** (line ~1580): Verify type checks
```javascript
// OLD:
if (cr7Parsed.type === 1 || cr7Parsed.type === 2) {

// NEW (depends on logic):
if (cr7Parsed.type === 1 || cr7Parsed.type === 3) {  // Inform or Abstract
// OR
if (cr7Parsed.type !== 0) {  // Any non-NULL type
```

---

## Testing to Verify Fix

After fixing, verify:

1. **Boot ROM**: Loads type 3 (Abstract) PassKeys for SWITCH
   - Should NOT show '???' in GT display
   - Should correctly validate Abstract GT gates

2. **Tutorials**: All GT_TYPE_INFORM usage (type 1)
   - Should show 'Inform' in UI, not 'Real'
   - Should behave correctly in CALL/LOAD operations

3. **Device abstractions**: Any type 2 (Outform) usage
   - Should show 'Outform' in UI
   - Should verify it's NOT being treated as Abstract

4. **Simulator console**: `parseGT()` output
   - Should display correct type names for all 4 types

---

## Risk Assessment

| Risk | Severity | Likelihood | Impact |
|------|----------|-----------|--------|
| Boot ROM fails on Abstract GT (type 3) | **CRITICAL** | HIGH | Cannot boot FPGA |
| Tutorial GTs (type 1) misbehave | **HIGH** | MEDIUM | Student confusion, wrong results |
| Outform handling (type 2) broken | **MEDIUM** | MEDIUM | Device I/O may not work |
| UI display shows '???' | **LOW** | CERTAIN | Cosmetic, but confusing |

---

## Root Cause

Simulator was written before complete GT type system was defined. It used placeholder "Real" and "Abstract" without the full 4-type taxonomy. Now that hardware defines all 4 types, simulator is out of sync.

---

## Files Updated

- [x] `simulator/simulator.js` — Fixed typeName array and type comparisons
- [x] `simulator/app.js` — Fixed all typeNames arrays and boot text strings
- [x] `simulator/assembler.js` — Fixed TPERM preset map (LE/SE/LSE) and disassembly names
- [x] `simulator/secure_boot_tutorial.js` — All GT type references updated to Inform
- [x] `simulator/system_abstractions.js` — Type guard and error messages fixed
- [x] `simulator/sliderule_tutorial.js` / `namespace_tutorial.js` — Type table updated
- [x] `hardware/boot_rom.py` — Comment cross-refs updated to Inform
- [x] `verilog/ctmm_pkg.sv` — GT_TYPE_REAL renamed to GT_TYPE_INFORM; type table corrected
- [x] `verilog/ctmm_core.sv` / `ctmm_tb.sv` — GT_TYPE_REAL → GT_TYPE_INFORM
- [x] `docs/CM_LUMP_SPECIFICATION.md` / `isa_encoding.md` — Type descriptions updated

---

## Recommendation

**Priority**: Fix before Week 1 hardware deployment (FPGA needs correct type handling).

**Approach**:
1. Update `simulator/simulator.js` typeName array (1 line)
2. Review and fix any type comparisons (likely 3-5 locations)
3. Re-run boot ROM simulator to verify Abstract GT (type 3) PassKeys work
4. Commit with message: "Fix simulator GT type mapping: type 1=Inform, type 2=Outform, type 3=Abstract"
