-- =========================================================================
-- PP250.Instructions.Arithmetic: Math Operations
-- =========================================================================
-- Implements arithmetic instructions for data register manipulation.

module PP250.Instructions.Arithmetic (
    instrEXECUTE_Math
) where

import qualified Data.Map as Map
import PP250.Core.Types

-- | EXECUTE_Math Instruction: Arithmetic Operations
-- Performs math operations on data registers using 2-address format:
-- DR[dest] = DR[dest] <op> DR[src]
-- Supported operations: ADD, SUB, POW (exponentiation)
instrEXECUTE_Math :: CPUState -> String -> Int -> Int -> CPUState
instrEXECUTE_Math cpu op destIdx srcIdx = 
    let vDest = Map.findWithDefault 0 destIdx (d_regs cpu)
        vSrc  = Map.findWithDefault 0 srcIdx (d_regs cpu)
        res = case op of 
            "ADD" -> vDest + vSrc
            "SUB" -> vDest - vSrc
            "POW" -> vDest ^ vSrc
            _     -> vDest
        newDRs = Map.insert destIdx res (d_regs cpu)
    in cpu { d_regs = newDRs }
