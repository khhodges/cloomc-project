-- =========================================================================
-- PP250.Instructions.Return: Procedure Exit
-- =========================================================================
-- Implements the RETURN instruction for capability-protected procedure return.

module PP250.Instructions.Return (
    instrRETURN
) where

import qualified Data.Map as Map
import PP250.Core.Types

-- | RETURN Instruction: Exit from a Procedure
-- Implements capability-protected procedure return:
--   1. Checks if the link stack has a saved frame
--   2. Pops the top frame from the stack
--   3. Restores CR6, CR7, and IP from the saved frame
-- This completes the CALL/RETURN pair for structured procedure invocation.
instrRETURN :: CPUState -> Either String CPUState
instrRETURN cpu = 
    case linkStack cpu of
        [] -> Left "TRAP: Stack Underflow (Nothing to Return to)"
        (frame:rest) -> 
            let 
                restoredRegs = Map.insert 6 (savedCR6 frame) (Map.insert 7 (savedCR7 frame) (c_regs cpu))
            in
            Right cpu {
                c_regs = restoredRegs,
                linkStack = rest,
                ip_Offset = savedOffset frame
            }
