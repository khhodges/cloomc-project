; ============================================
; Church Machine GC Test (PP250)
; Pure Church — zero Turing instructions
; Run AFTER boot completes (6 steps)
; ============================================
;
; HOW GC WORKS (bidirectional G-bit):
;   The G-bit is not set by a separate "mark" pass.
;   mLoad toggles G toward the "live" value every
;   time a GT is successfully validated. Entries
;   never accessed retain the old polarity.
;
;   GC is a SAFE ABSTRACTION — an atomic Turing
;   machine hidden inside a Church-callable entry.
;   CALL GC enters the abstraction; the hidden
;   Turing implementation scans, identifies garbage,
;   clears NS entries, frees memory, and flips
;   polarity. RETURN exits automatically.
;
;   No mark phase needed. Normal execution IS the mark.
;
; DESIGN:
;   Load 5 abstractions into CR0-CR4 via LOAD.
;   Each LOAD calls mLoad which marks them live.
;   Then LOAD the GC abstraction and CALL it.
;   GC runs as an atomic Turing abstraction —
;   entered via CALL, exited via RETURN.
;
; EXPECTED GC RESULT:
;   Boot(0) = CR6,CR7  — LIVE (boot c-list + code)
;   Threads(1) = CR8   — LIVE (thread identity)
;   Lambda(2) = CR0    — LIVE (loaded via mLoad)
;   SlideRule(3)        — GARBAGE (never accessed)
;   Abacus(4)           — GARBAGE
;   Constants(5) = CR4  — LIVE (loaded via mLoad)
;   Stack(6) = CR2      — LIVE (loaded via mLoad)
;   SUCC(7) = CR1       — LIVE (loaded via mLoad)
;   PRED(8)             — GARBAGE
;   ADD(9) = CR3        — LIVE (loaded via mLoad)
;   SUB(10)-SND(23)     — GARBAGE
;   GC(24) = CR5        — LIVE (loaded to CALL it)
;
;   Live: 8 entries (Boot, Threads, Lambda,
;         Constants, Stack, SUCC, ADD, GC)
;         + CR15 Namespace root (Boot/slot 0)
;   Swept: 16 entries freed
; ============================================

; --- Phase 1: Load subset into CRs (survivors) ---
; Each LOAD calls mLoad, toggling G to "live"
LOAD CR0, CR6, 2       ; CR0 = Lambda    (E)
LOAD CR1, CR6, 7       ; CR1 = SUCC      (LE)
LOAD CR2, CR6, 6       ; CR2 = Stack     (E)
LOAD CR3, CR6, 9       ; CR3 = ADD       (LE)
LOAD CR4, CR6, 5       ; CR4 = Constants (E)

; --- Phase 2: Verify permissions ---
TPERM CR0, E           ; Lambda has E? PASS
TPERM CR1, LE          ; SUCC has L+E? PASS
TPERM CR2, E           ; Stack has E? PASS
TPERM CR3, LE          ; ADD has L+E? PASS
TPERM CR4, E           ; Constants has E? PASS

; --- Phase 3: Exercise live capabilities ---
LAMBDA CR1             ; Church SUCC reduction
LAMBDA CR3             ; Church ADD reduction

; --- Phase 4: CALL GC safe abstraction ---
; Load GC abstraction GT (slot 24) and CALL it.
; GC is an atomic Turing machine — entered via CALL,
; hidden implementation scans/sweeps, exits via RETURN.
LOAD CR5, CR6, 24      ; CR5 = GC abstraction (E)
TPERM CR5, E           ; Verify E permission
CALL CR5               ; Trigger GC — atomic Turing abstraction

; --- Phase 5: HALT ---
; GC has completed. Namespace Browser shows results.
; 16 entries swept, 8 entries survived.
; Polarity flipped for next cycle.
HALT
