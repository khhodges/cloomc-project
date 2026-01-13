-- =========================================================================
-- PP250.Instructions.LoadSave: Capability Load and Save Operations
-- =========================================================================
-- Implements LOAD and SAVE instructions for capability manipulation.

module PP250.Instructions.LoadSave (
    instrLOAD,
    instrSAVE
) where

import qualified Data.Map as Map
import PP250.Core.Types

-- | LOAD Instruction: Load a Capability
-- Loads a new capability into a context register. Requires PermLoad
-- permission on the source register. CR7 (code register) is protected
-- and cannot be loaded directly - use EXECUTE instead.
instrLOAD :: CPUState -> Int -> Int -> Int -> Either String CPUState
instrLOAD cpu d s i = 
    if d==7 then Left "Use EXECUTE to load CR7" else
    let src = Map.findWithDefault emptyCR s (c_regs cpu) in
    if not (PermLoad `elem` activePerms src) 
        then Left "TRAP: No LOAD Perm"
        else Right cpu { c_regs = Map.insert d (mkCR ("Obj_"++show i) (Local i) [PermRead]) (c_regs cpu) }

-- | SAVE Instruction: Save/Bind Data
-- Saves data to a location via a capability. Requires PermSave
-- permission on the destination register.
instrSAVE :: CPUState -> Int -> Int -> Either String String
instrSAVE cpu d s = 
    let dst = Map.findWithDefault emptyCR d (c_regs cpu) in
    if not (PermSave `elem` activePerms dst) 
        then Left "TRAP: No SAVE Perm"
        else Right "SUCCESS: Bound."
