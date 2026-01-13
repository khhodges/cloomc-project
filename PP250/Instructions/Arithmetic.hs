-- =========================================================================
-- PP250.Instructions.Arithmetic: Math Operations with ARM-style Flags
-- =========================================================================
-- Implements arithmetic instructions for data register manipulation.
-- Sets ARM-compatible NZCV condition flags after each operation.

module PP250.Instructions.Arithmetic (
    instrEXECUTE_Math,
    computeFlags
) where

import qualified Data.Map as Map
import Data.Word (Word64)
import Data.Bits (testBit)
import PP250.Core.Types

-- | Compute ARM-style NZCV condition flags for a result.
-- Takes the operation type, operand values, and result to compute:
--   N - Negative: bit 63 of result (sign bit for 64-bit signed)
--   Z - Zero: result == 0
--   C - Carry: unsigned overflow for ADD, no borrow for SUB
--   V - Overflow: signed overflow detection
computeFlags :: String -> Word64 -> Word64 -> Word64 -> ConditionFlags
computeFlags op a b result = CondFlags {
    flagN = testBit result 63,                    -- Sign bit (negative)
    flagZ = result == 0,                          -- Zero result
    flagC = case op of
        "ADD" -> result < a                       -- Unsigned overflow on ADD
        "SUB" -> a >= b                           -- No borrow on SUB (carry set)
        _     -> False,
    flagV = case op of
        "ADD" -> signedOverflowAdd a b result     -- Signed overflow on ADD
        "SUB" -> signedOverflowSub a b result     -- Signed overflow on SUB
        _     -> False
}

-- | Detect signed overflow for addition.
-- Overflow occurs when adding two numbers with same sign produces opposite sign.
signedOverflowAdd :: Word64 -> Word64 -> Word64 -> Bool
signedOverflowAdd a b result =
    let signA = testBit a 63
        signB = testBit b 63
        signR = testBit result 63
    in (signA == signB) && (signA /= signR)

-- | Detect signed overflow for subtraction.
-- Overflow occurs when subtracting produces unexpected sign change.
signedOverflowSub :: Word64 -> Word64 -> Word64 -> Bool
signedOverflowSub a b result =
    let signA = testBit a 63
        signB = testBit b 63
        signR = testBit result 63
    in (signA /= signB) && (signB == signR)

-- | EXECUTE_Math Instruction: Arithmetic Operations with Flag Updates
-- Performs math operations on data registers using 2-address format:
-- DR[dest] = DR[dest] <op> DR[src]
-- Supported operations: ADD, SUB, POW (exponentiation)
-- Updates NZCV condition flags based on the result.
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
        newFlags = computeFlags op vDest vSrc res
    in cpu { d_regs = newDRs, condFlags = newFlags }
