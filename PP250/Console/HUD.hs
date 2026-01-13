-- =========================================================================
-- PP250.Console.HUD: Head-Up Display / System Telemetry
-- =========================================================================
-- Provides the visual display of CPU state for debugging and monitoring.

module PP250.Console.HUD (
    displayHUD,
    showHelp
) where

import qualified Data.Map as Map
import Text.Printf (printf)
import PP250.Core.Types
import PP250.Core.Utils

-- | Display the complete system telemetry HUD.
-- Shows all context registers (CR0-CR7), system registers (CR8, CR15),
-- instruction pointer, and all 8 data registers in formatted tables.
displayHUD :: CPUState -> IO ()
displayHUD cpu = do
    putStrLn "\n========================== PP250 SYSTEM TELEMETRY =========================="
    putStrLn "|  CONTEXT REGISTERS (WORKING SET)                                         |"
    putStrLn "+-----+---------------------+-----------------------+---------+-------------+"
    putStrLn "| ID  | REGISTER ROLE       | OBJECT NAME           | RWX LSE | LOCATION    |"
    putStrLn "+-----+---------------------+-----------------------+---------+-------------+"
    mapM_ (printCR cpu) [0..7]
    putStrLn "+-----+---------------------+-----------------------+---------+-------------+"
    
    putStrLn "|  SYSTEM STATE                                                             |"
    putStrLn "+-----------------------------+---------------------------------------------+"
    putStrLn $ "| CR15 (NAMESPACE)            | " ++ pad 43 (cachedName (cr15_NS cpu)) ++ " |"
    putStrLn $ "| CR8  (THREAD/USER)          | " ++ pad 43 (cachedName (cr8_Thread cpu)) ++ " |"
    putStrLn $ "| IP   (INSTRUCTION PTR)      | " ++ pad 43 (show (ip_Offset cpu)) ++ " |"
    putStrLn "+-----------------------------+---------------------------------------------+"
    
    putStrLn "|  DATA REGISTERS                                                           |"
    putStrLn "+-------------------+-------------------+-------------------+---------------+"
    let dr n = printf "0x%016X" (Map.findWithDefault 0 n (d_regs cpu)) :: String
    putStrLn $ "| DR0: " ++ padNoTrunc 18 (dr 0) ++ " | DR1: " ++ padNoTrunc 18 (dr 1) ++ " |"
    putStrLn $ "| DR2: " ++ padNoTrunc 18 (dr 2) ++ " | DR3: " ++ padNoTrunc 18 (dr 3) ++ " |"
    putStrLn $ "| DR4: " ++ padNoTrunc 18 (dr 4) ++ " | DR5: " ++ padNoTrunc 18 (dr 5) ++ " |"
    putStrLn $ "| DR6: " ++ padNoTrunc 18 (dr 6) ++ " | DR7: " ++ padNoTrunc 18 (dr 7) ++ " |"
    putStrLn "============================================================================="

printCR :: CPUState -> Int -> IO ()
printCR cpu i = do
    let reg = Map.findWithDefault emptyCR i (c_regs cpu)
    let role = case i of 
            7 -> "NUCLEUS (CODE)     "
            6 -> "C-LIST LCA         "
            _ -> "GENERAL CAPABILITY "
    let name = if cachedName reg == "NULL" 
               then padNoTrunc 25 "NULL" 
               else padNoTrunc 25 (cachedName reg)
    let pStr = if cachedName reg == "NULL" 
               then "--- ---" 
               else take 7 (permString (activePerms reg))
    let locStr = padNoTrunc 11 (formatLoc (cachedLoc reg))
    putStrLn $ "| CR" ++ pad 2 (show i) ++ " | " ++ role ++ " | " ++ name ++ " | " ++ pStr ++ " | " ++ locStr ++ " |"

-- | Display the command reference help screen.
showHelp :: IO ()
showHelp = do
    putStrLn "\n======================= PP250 COMMAND REFERENCE ========================"
    putStrLn "| Command                | Description                                 |"
    putStrLn "+------------------------+---------------------------------------------+"
    putStrLn "| HELP                   | Show this help message                      |"
    putStrLn "| HUD                    | Display the system telemetry panel          |"
    putStrLn "| NS                     | Display namespace capability (CR15)         |"
    putStrLn "| CLIST                  | Display C-List of capability keys           |"
    putStrLn "| ADD  <dest> <src>      | DR[dest] = DR[dest] + DR[src]               |"
    putStrLn "| SUB  <dest> <src>      | DR[dest] = DR[dest] - DR[src]               |"
    putStrLn "| POW  <dest> <src>      | DR[dest] = DR[dest] ^ DR[src]               |"
    putStrLn "| LOAD <dest> <src> <i>  | Load object at index i via CR[src] -> CR[d] |"
    putStrLn "| SAVE <dest> <src>      | Save DR[src] to location via CR[dest]       |"
    putStrLn "| CALL <reg>             | Call procedure in CR[reg] (requires Enter)  |"
    putStrLn "| RETURN                 | Return from procedure (pop stack frame)     |"
    putStrLn "| CHANGE <offset>        | Switch to thread at scope offset            |"
    putStrLn "| EXIT                   | Shutdown the system                         |"
    putStrLn "========================================================================="
