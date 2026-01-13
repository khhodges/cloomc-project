-- =========================================================================
-- PP250.Boot.Sequence: System Boot Sequence
-- =========================================================================
-- Implements the 4-step boot sequence that initializes the PP250 system.

module PP250.Boot.Sequence (
    bootStep1_HardwareReset,
    bootStep2_LoadNamespace,
    bootStep3_LoadThread,
    bootStep4_LoadResources
) where

import qualified Data.Map as Map
import PP250.Core.Types
import PP250.Console.HUD (displayHUD)

-- | BOOT STEP 1: Hardware Reset
-- Clears all registers to their null/zero state.
bootStep1_HardwareReset :: IO CPUState
bootStep1_HardwareReset = do
    putStrLn "\n[BOOT STEP 1] HARDWARE RESET"
    putStrLn "   > Power Energized. Clearing All Registers to NULL..."
    let emptyRegs = Map.fromList [(i, emptyCR) | i <- [0..7]]
    let emptyData = Map.fromList [(i, 0) | i <- [0..7]]
    let cpu = CPUState emptyRegs emptyData 0 ["RESET"] [] emptyCR emptyCR Map.empty Map.empty
    displayHUD cpu 
    return cpu

-- | BOOT STEP 2: Load Namespace (CR15)
-- Loads the root namespace capability into CR15.
bootStep2_LoadNamespace :: CPUState -> IO CPUState
bootStep2_LoadNamespace cpu = do
    putStrLn "\n[BOOT STEP 2] LOAD NAMESPACE (CR15)"
    putStrLn "   > Formatting Memory at 4000..."
    let bootNS = mkCR "Boot Namespace" (Local 4000) [PermRead, PermLoad]
    putStrLn "   > LOADING CR15..."
    let newCpu = cpu { cr15_NS = bootNS }
    displayHUD newCpu
    return newCpu

-- | BOOT STEP 3: Load Thread Context (CR8)
-- Loads the initial user/thread identity into CR8.
bootStep3_LoadThread :: CPUState -> IO CPUState
bootStep3_LoadThread cpu = do
    putStrLn "\n[BOOT STEP 3] LOAD THREAD CONTEXT (CR8)"
    putStrLn "   > Fetching 'Kenneth' (Entry 1) -> CR8..."
    let kennethCR = mkCR "Kenneth" (Local 8000) []
    let newCpu = cpu { cr8_Thread = kennethCR }
    displayHUD newCpu
    return newCpu

-- | BOOT STEP 4: Load Code and Tools
-- Loads the initial program code into CR7 and tools into CR6.
bootStep4_LoadResources :: CPUState -> IO CPUState
bootStep4_LoadResources cpu = do
    putStrLn "\n[BOOT STEP 4] LOAD CODE & TOOLS (CR7/CR6)"
    putStrLn "   > Fetching 'Diag Code' -> CR7..."
    let cr7 = mkCR "Diag Code" (Local 16000) [PermRead, PermExecute]
    putStrLn "   > Fetching 'Test Arch' -> CR6..."
    let cr6 = mkCR "Test Arch" (Local 20000) [PermEnter, PermBind]
    
    let newRegs = Map.insert 7 cr7 (Map.insert 6 cr6 (c_regs cpu))
    
    let opCR8   = mkCR "Operator" (Local 90000) []
    let opState = SavedState 999 ["READY"] Map.empty Map.empty []
    let ram     = Map.fromList [("Operator", opState)]
    let queue   = Map.fromList [(1, opCR8)]
    
    let dataRegs = Map.fromList [(0, 10), (1, 5)]
    
    let newCpu = cpu { c_regs = newRegs, d_regs = dataRegs, ram_Threads = ram, scope_CList = queue, ip_Offset = 100 }
    displayHUD newCpu
    return newCpu
