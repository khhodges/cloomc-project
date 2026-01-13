-- =========================================================================
-- PP250.Console.REPL: Interactive Console / Read-Eval-Print Loop
-- =========================================================================
-- Provides the interactive command-line interface to the PP250 simulator.

module PP250.Console.REPL (
    runConsole
) where

import qualified Data.Map as Map
import System.IO (hFlush, stdout)
import Text.Printf (printf)

import PP250.Core.Types
import PP250.Core.Utils
import PP250.Console.HUD (displayHUD, showHelp)
import PP250.Instructions.Arithmetic (instrEXECUTE_Math)
import PP250.Instructions.LoadSave (instrLOAD, instrSAVE)
import PP250.Instructions.Call (instrCALL)
import PP250.Instructions.Return (instrRETURN)
import PP250.Instructions.Change (instrCHANGE)

-- | Main console loop: Read-Eval-Print Loop (REPL)
runConsole :: CPUState -> IO ()
runConsole cpu = do
    putStr ">> CMD (HELP for commands): "
    hFlush stdout
    input <- getLine
    
    case words input of
        ["EXIT"] -> putStrLn "--- SHUTDOWN ---"
        
        ["HELP"] -> showHelp >> runConsole cpu
        
        ["HUD"] -> displayHUD cpu >> runConsole cpu
        
        ["NS"] -> do
            let ns = cr15_NS cpu
            putStrLn "\n+----------------------- CR15 NAMESPACE -------------------------+"
            putStrLn $ "| Name:        " ++ padNoTrunc 49 (cachedName ns) ++ " |"
            putStrLn $ "| Location:    " ++ padNoTrunc 49 (formatLoc (cachedLoc ns)) ++ " |"
            putStrLn $ "| Permissions: " ++ padNoTrunc 49 (permString (activePerms ns)) ++ " |"
            putStrLn $ "| Locked:      " ++ padNoTrunc 49 (show (isLocked ns)) ++ " |"
            putStrLn "+----------------------------------------------------------------+"
            runConsole cpu
        
        ["CLIST"] -> do
            putStrLn "\n+--------------------------- C-LIST (CAPABILITY KEYS) ---------------------------+"
            putStrLn "| IDX  | NAME                      | LOCATION         | PERMS   | LOCKED |"
            putStrLn "+------+---------------------------+------------------+---------+--------+"
            let entries = Map.toList (scope_CList cpu)
            if null entries
                then putStrLn "|                           (empty)                                            |"
                else mapM_ printCListEntry entries
            putStrLn "+---------------------------------------------------------------------------------+"
            runConsole cpu
        
        ["FLAGS"] -> do
            let flags = condFlags cpu
            let n = if flagN flags then "1 (Negative)" else "0"
            let z = if flagZ flags then "1 (Zero)"     else "0"
            let c = if flagC flags then "1 (Carry)"    else "0"
            let v = if flagV flags then "1 (Overflow)" else "0"
            putStrLn "\n+------------------- CONDITION FLAGS (ARM-style NZCV) -------------------+"
            putStrLn $ "| N (Negative): " ++ padNoTrunc 56 n ++ " |"
            putStrLn $ "| Z (Zero):     " ++ padNoTrunc 56 z ++ " |"
            putStrLn $ "| C (Carry):    " ++ padNoTrunc 56 c ++ " |"
            putStrLn $ "| V (Overflow): " ++ padNoTrunc 56 v ++ " |"
            putStrLn "+------------------------------------------------------------------------+"
            runConsole cpu
        
        ("CHANGE":xStr:_) -> case readInt xStr of
            Just x  -> do
                result <- instrCHANGE cpu x
                case result of
                    Right newCpu -> runConsole newCpu
                    Left err     -> putStrLn ("[TRAP] " ++ err) >> runConsole cpu
            Nothing -> putStrLn "[ERROR] Invalid Argument (Expected Integer)" >> runConsole cpu
            
        (op:dStr:sStr:_) | op `elem` ["ADD", "SUB", "POW"] -> 
            case (readInt dStr, readInt sStr) of
                (Just d, Just s) -> do
                    let newCpu = instrEXECUTE_Math cpu op d s
                    let oldVal = Map.findWithDefault 0 d (d_regs cpu)
                    let newVal = Map.findWithDefault 0 d (d_regs newCpu)
                    let flags = condFlags newCpu
                    let flagStr = "[" ++ (if flagN flags then "N" else "-") 
                                      ++ (if flagZ flags then "Z" else "-")
                                      ++ (if flagC flags then "C" else "-")
                                      ++ (if flagV flags then "V" else "-") ++ "]"
                    putStrLn $ "   [OK] DR" ++ show d ++ ": 0x" ++ printf "%X" oldVal ++ " -> 0x" ++ printf "%X" newVal
                    putStrLn $ "        Flags: " ++ flagStr
                    runConsole newCpu
                _                -> putStrLn "[ERROR] Invalid Register Index" >> runConsole cpu

        ("LOAD":dStr:sStr:iStr:_) -> 
            case (readInt dStr, readInt sStr, readInt iStr) of
                (Just d, Just s, Just i) -> case instrLOAD cpu d s i of
                    Right c -> do
                        putStrLn $ "   [OK] Loaded object into CR" ++ show d
                        runConsole c
                    Left e  -> putStrLn ("[TRAP] " ++ e) >> runConsole cpu
                _ -> putStrLn "[ERROR] Invalid Arguments" >> runConsole cpu

        ("SAVE":dStr:sStr:_) ->
            case (readInt dStr, readInt sStr) of
                (Just d, Just s) -> case instrSAVE cpu d s of
                    Right m -> putStrLn ("   [OK] " ++ m) >> runConsole cpu
                    Left e  -> putStrLn ("[TRAP] " ++ e) >> runConsole cpu
                _ -> putStrLn "[ERROR] Invalid Arguments" >> runConsole cpu

        ("CALL":rStr:_) -> case readInt rStr of
            Just r -> case instrCALL cpu r of
                Right newCpu -> do
                    putStrLn $ "   [OK] CALL to CR" ++ show r ++ " - Entered '" ++ cachedName (Map.findWithDefault emptyCR 6 (c_regs newCpu)) ++ "'"
                    putStrLn $ "        Stack depth: " ++ show (length (linkStack newCpu))
                    runConsole newCpu
                Left e -> putStrLn ("[TRAP] " ++ e) >> runConsole cpu
            Nothing -> putStrLn "[ERROR] Invalid Register Index" >> runConsole cpu

        ["RETURN"] -> case instrRETURN cpu of
            Right newCpu -> do
                putStrLn $ "   [OK] RETURN - Restored to '" ++ cachedName (Map.findWithDefault emptyCR 6 (c_regs newCpu)) ++ "'"
                putStrLn $ "        IP restored to: " ++ show (ip_Offset newCpu)
                runConsole newCpu
            Left e -> putStrLn ("[TRAP] " ++ e) >> runConsole cpu
                
        [] -> runConsole cpu
        
        _ -> putStrLn "[ERROR] Unknown Command. Type HELP for available commands." >> runConsole cpu

printCListEntry :: (Int, ContextRegister) -> IO ()
printCListEntry (idx, reg) = do
    let idxStr = padNoTrunc 4 (show idx)
    let nameStr = padNoTrunc 25 (cachedName reg)
    let locStr = padNoTrunc 16 (formatLoc (cachedLoc reg))
    let permStr = take 7 (permString (activePerms reg))
    let lockStr = padNoTrunc 6 (if isLocked reg then "Yes" else "No")
    putStrLn $ "| " ++ idxStr ++ " | " ++ nameStr ++ " | " ++ locStr ++ " | " ++ permStr ++ " | " ++ lockStr ++ " |"
