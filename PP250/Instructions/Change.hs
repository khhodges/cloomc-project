-- =========================================================================
-- PP250.Instructions.Change: Process/Thread Context Switch
-- =========================================================================
-- Implements the CHANGE instruction for capability-protected context switching.

module PP250.Instructions.Change (
    instrCHANGE
) where

import qualified Data.Map as Map
import PP250.Core.Types
import PP250.Core.Utils (storeKey, fetchKey)

-- | CHANGE Instruction: Context Switch / Process Switch
-- Saves the current thread's state and restores a different thread
-- specified by the offset into the C-List. This is how the PP250
-- implements multitasking - by swapping complete execution contexts.
-- Returns Either an error message (Left) or the new CPU state (Right).
instrCHANGE :: CPUState -> Int -> IO (Either String CPUState)
instrCHANGE cpu offset = do
    putStrLn $ "\n[OP] CHANGE PROCESS (Offset " ++ show offset ++ ")..."
    let currentName = cachedName (cr8_Thread cpu)
    
    putStrLn $ "   > SAVING: " ++ currentName
    let savedDRs = d_regs cpu
    let savedCRs = Map.map storeKey (c_regs cpu)
    let snapshot = SavedState (ip_Offset cpu) (sr_Status cpu) savedDRs savedCRs (linkStack cpu)
    let newRAM   = Map.insert currentName snapshot (ram_Threads cpu)
    
    let newCap = Map.findWithDefault emptyCR offset (scope_CList cpu)
    let newName = cachedName newCap
    if newName == "NULL" 
        then return $ Left "CHANGE Failed (Target NULL)"
        else do
            let newState = Map.findWithDefault emptyState newName newRAM
            let restoredCRs = Map.map fetchKey (storedKeys newState)
            
            let newCPU = cpu {
                cr8_Thread = newCap, 
                ip_Offset = storedIP newState, 
                sr_Status = storedSR newState,
                d_regs = storedDRs newState, 
                c_regs = restoredCRs, 
                linkStack = storedStack newState,
                ram_Threads = newRAM
            }
            putStrLn $ "   > RESTORED: " ++ newName
            return $ Right newCPU
