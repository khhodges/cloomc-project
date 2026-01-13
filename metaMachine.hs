-- =========================================================================
-- PP250 META-MACHINE: Main Entry Point
-- =========================================================================
-- Architect: Kenneth James Hamer-Hodges
-- 
-- This is the main entry point for the PP250 Capability-Based Meta-Machine
-- simulator. It orchestrates the boot sequence and enters the interactive
-- console for testing and exploration of the capability-based architecture.
--
-- Module Structure:
--   PP250/Core/Types.hs      - Shared data types (CPUState, ContextRegister, etc.)
--   PP250/Core/Utils.hs      - Utility functions (formatting, key operations)
--   PP250/Instructions/*.hs  - Individual instruction implementations
--   PP250/Console/HUD.hs     - System telemetry display
--   PP250/Console/REPL.hs    - Interactive console
--   PP250/Boot/Sequence.hs   - Boot sequence implementation
-- =========================================================================

module Main where

import PP250.Boot.Sequence
import PP250.Console.REPL (runConsole)

-- | Main entry point: Execute boot sequence then enter interactive console
main :: IO ()
main = do
    putStrLn "--- PP250 BOOT SEQUENCE START ---"
    
    cpu1 <- bootStep1_HardwareReset
    putStrLn ">> Press ENTER for Step 2..." >> getLine
    
    cpu2 <- bootStep2_LoadNamespace cpu1
    putStrLn ">> Press ENTER for Step 3..." >> getLine
    
    cpu3 <- bootStep3_LoadThread cpu2
    putStrLn ">> Press ENTER for Step 4..." >> getLine
    
    cpu4 <- bootStep4_LoadResources cpu3
    putStrLn ">> BOOT COMPLETE. ENTERING CONSOLE..."
    putStrLn ">> Try: ADD 0 1 (Computes DR0 = DR0 + DR1)" >> getLine
    
    runConsole cpu4
