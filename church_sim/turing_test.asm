; ============================================
; Turing ISA Test
; Exercises Turing integer instructions
; ============================================
;
; Turing ISA (9 instructions):
;   DREAD, DWRITE  — data access via GT (R/W)
;   BFEXT, BFINS   — bitfield access via GT (R/W)
;   MCMP           — compare DRs, set flags
;   IADD, ISUB     — integer arithmetic, set flags
;   BRANCH         — conditional branch
;   RETURN         — shared with Church domain
;
; This test exercises IADD, ISUB, MCMP, BRANCH
; with initialized data registers.
; DREAD/DWRITE/BFEXT/BFINS require a DATA
; object with R/W GT (see GC test for LOAD usage).
; ============================================

; --- Phase 1: Load some GTs to boot ---
LOAD CR0, CR6, 7       ; CR0 = SUCC (XLE)
LOAD CR1, CR6, 9       ; CR1 = ADD (XLE)

; --- Phase 2: Initialize DRs via IADD ---
; DR0 is always 0 (hardwired)
; IADD DR1, DR0, DR0 sets DR1 = 0
IADD DR1, DR0, DR0     ; DR1 = 0 (Z=1)

; Build DR1 = 10 by repeated IADD
; Use ISUB trick: DR2 = 0 - 0xFFFFFFF6 would be complex
; Simpler: IADD DR1 = DR1 + DR1 won't help from 0
; Let's use Church domain to set a value first
LAMBDA CR0             ; SUCC reduction (sets state)

; --- Phase 3: IADD and ISUB ---
IADD DR3, DR1, DR2     ; DR3 = DR1 + DR2
ISUB DR4, DR3, DR1     ; DR4 = DR3 - DR1 (should = DR2)

; --- Phase 4: MCMP compare ---
MCMP DR4, DR2          ; DR4 == DR2? Z=1 expected
BRANCHEQ +2            ; If equal, skip next
IADD DR5, DR1, DR1     ; Only if NOT equal (skipped)

; --- Phase 5: Nonzero compare ---
MCMP DR3, DR4          ; DR3 vs DR4 — may differ
BRANCHNE +2            ; If not equal, skip next
ISUB DR6, DR1, DR1     ; Only if equal (skipped)

; --- Phase 6: Zero flag test ---
ISUB DR7, DR3, DR3     ; DR7 = 0 (Z=1, N=0, C=1)
BRANCHEQ +2            ; Z=1, branch taken
IADD DR8, DR1, DR1     ; Skipped by branch

; --- Phase 7: Overflow test ---
; IADD large values to test C/V flags
IADD DR9, DR3, DR4     ; DR9 = DR3 + DR4
ISUB DR10, DR0, DR1    ; DR10 = 0 - DR1 (wraps, N=1)

HALT
