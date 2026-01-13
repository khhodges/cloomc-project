-- =========================================================================
-- PP250.Instructions.Call: Procedure Entry
-- =========================================================================
-- Implements the CALL instruction for capability-protected procedure entry.

module PP250.Instructions.Call (
    instrCALL
) where

import qualified Data.Map as Map
import PP250.Core.Types

-- | CALL Instruction: Enter a Procedure / Lambda Abstraction
-- Implements capability-protected procedure entry:
--   1. Checks for 'Enter' permission on target capability
--   2. Pushes current context (CR6, CR7, IP) onto the link stack
--   3. Enters target: Target becomes CR6, new code loaded into CR7
-- This is how the PP250 implements safe procedure calls - the caller
-- cannot enter code without explicit Enter permission granted by the callee.
instrCALL :: CPUState -> Int -> Either String CPUState
instrCALL cpu targetIdx = 
    let targetCR = Map.findWithDefault emptyCR targetIdx (c_regs cpu)
        currentCR6 = Map.findWithDefault emptyCR 6 (c_regs cpu)
        currentCR7 = Map.findWithDefault emptyCR 7 (c_regs cpu)
    in 
    if cachedName targetCR == "NULL" then Left "TRAP: Target is NULL"
    else if not (PermEnter `elem` activePerms targetCR) then Left "TRAP: No ENTER Permission"
    else 
        let 
            frame = Frame { savedCR6 = currentCR6, savedCR7 = currentCR7, savedOffset = ip_Offset cpu }
            newCodeName = "Code[" ++ cachedName targetCR ++ "]"
            newCR7 = mkCR newCodeName (Local 0) [PermRead, PermExecute]
            newRegs = Map.insert 6 targetCR (Map.insert 7 newCR7 (c_regs cpu))
        in
        Right cpu { 
            c_regs = newRegs, 
            linkStack = frame : linkStack cpu,
            ip_Offset = 0
        }
