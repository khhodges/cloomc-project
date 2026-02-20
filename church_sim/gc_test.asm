; ============================================
; Church Machine GC Test (PP250)
; Pure Church — zero Turing instructions
; Run AFTER boot completes (6 steps)
; ============================================
;
; DESIGN:
;   Phase 1: Load 5 abstractions into CR0-CR4
;            These become "live" — GC must protect them.
;   Phase 2: Verify permissions with TPERM
;   Phase 3: Exercise LAMBDA reductions on live CRs
;   Phase 4: HALT — then press "Run GC" button
;
; EXPECTED GC RESULT:
;   Boot(0) = CR6,CR7  — LIVE (boot c-list + code)
;   Threads(1) = CR8   — LIVE (thread identity)
;   Lambda(2) = CR0    — LIVE (loaded)
;   SlideRule(3)        — GARBAGE (no CR holds it)
;   Abacus(4)           — GARBAGE
;   Constants(5) = CR4  — LIVE (loaded)
;   Stack(6) = CR2      — LIVE (loaded)
;   SUCC(7) = CR1       — LIVE (loaded)
;   PRED(8)             — GARBAGE
;   ADD(9) = CR3        — LIVE (loaded)
;   SUB(10)             — GARBAGE
;   MUL(11)             — GARBAGE
;   DIV(12)             — GARBAGE
;   POW(13)             — GARBAGE
;   SQRT(14)            — GARBAGE
;   LOG(15)             — GARBAGE
;   EXP(16)             — GARBAGE
;   ISZERO(17)          — GARBAGE
;   LEQ(18)             — GARBAGE
;   TRUE(19)            — GARBAGE
;   FALSE(20)           — GARBAGE
;   PAIR(21)            — GARBAGE
;   FST(22)             — GARBAGE
;   SND(23)             — GARBAGE
;
;   Live: 7 entries (Boot, Threads, Lambda,
;         Constants, Stack, SUCC, ADD)
;         + CR15 Namespace root (Boot/slot 0)
;   Swept: 17 entries freed
; ============================================

; --- Phase 1: Load subset into CRs (survivors) ---
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

; --- Phase 4: HALT — ready for GC ---
; Press "Run GC" to trigger PP250 Mark-Scan-Sweep.
; Namespace Browser will show 17 entries vanish.
HALT
