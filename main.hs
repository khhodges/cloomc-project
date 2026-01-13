-- =========================================================================
-- PP250 SIMULATOR: CAPABILITY-BASED META-MACHINE
-- =========================================================================
-- Architect: Kenneth James Hamer-Hodges
-- 
-- This simulator models the PP250, an "Industrial Strength Living Machine"
-- based on Lambda Calculus Hardware. It demonstrates capability-based
-- security where all access is controlled through "Golden Tokens" -
-- cryptographic capability keys that encode location, authority, and identity.
--
-- Key Concepts:
--   - Context Registers (CRs): Hold capability keys that grant access rights
--   - Data Registers (DRs): Hold 64-bit numeric values for computation
--   - Namespace (CR15): The root capability defining the system's scope
--   - Thread Context (CR8): The currently executing user/process identity
--   - C-List: A list of capability keys available to the current context
--
-- Features:
--   1. Safe input parsing to prevent runtime crashes
--   2. Interactive console with HUD telemetry display
--   3. Process switching via CHANGE instruction
--   4. Capability-checked LOAD/SAVE operations
-- =========================================================================

-- Required library imports
import qualified Data.Map as Map    -- Provides efficient key-value storage for registers
import Data.Word (Word64)           -- 64-bit unsigned integers for data registers
import System.IO (hFlush, stdout)   -- Console I/O utilities for interactive prompts
import Text.Printf (printf)         -- Formatted printing for hexadecimal display
import Text.Read (readMaybe)        -- Safe string-to-integer parsing (returns Maybe)

-- =========================================================================
-- 1. ARCHITECTURE & STATE
-- =========================================================================
-- This section defines the core data structures that model the PP250's
-- hardware architecture. The design follows capability-based security
-- principles where every memory access requires a valid capability key.

-- | Location represents where an object resides in the system.
-- Local: An address in local memory (integer offset)
-- Literal: A named constant or external reference (string identifier)
data Location = Local Int | Literal String deriving (Show, Eq)

-- | Permission flags that can be granted by a capability key.
-- These control what operations are allowed on the referenced object:
--   PermRead    - Can read the object's contents
--   PermWrite   - Can modify the object's contents
--   PermExecute - Can execute the object as code
--   PermLoad    - Can use this capability to load other capabilities
--   PermSave    - Can use this capability to store data
--   PermEnter   - Can enter/call into this object's code
--   PermBind    - Can bind new capabilities to this object
data Permission = 
    PermRead | PermWrite | PermExecute | PermLoad | PermSave | PermEnter | PermBind
    deriving (Show, Eq)

-- | ContextRegister represents a "Golden Token" - the fundamental
-- capability key in the PP250 system. Each register contains:
--   cachedLoc   - The memory location this capability points to
--   cachedName  - A human-readable name for the referenced object
--   activePerms - List of permissions granted by this capability
--   isLocked    - Whether the key is locked (protected from modification)
data ContextRegister = ContextReg {
    cachedLoc :: Location, 
    cachedName :: String, 
    activePerms :: [Permission],
    isLocked :: Bool 
} deriving (Show, Eq)

-- | SavedThreadState stores a complete snapshot of a thread's execution
-- state, allowing the system to suspend and resume threads. Contains:
--   storedIP    - Instruction pointer (program counter)
--   storedSR    - Status register flags
--   storedDRs   - Saved data register values
--   storedKeys  - Saved context register capabilities
--   storedStack - Saved call stack frames
data SavedThreadState = SavedState {
    storedIP   :: Int,
    storedSR   :: [String],
    storedDRs  :: Map.Map Int Word64,
    storedKeys :: Map.Map Int ContextRegister,
    storedStack:: [StackFrame]
} deriving (Show)

-- | StackFrame preserves the calling context for nested procedure calls.
-- Saves CR6 (tool/LCA), CR7 (code), and the return offset.
data StackFrame = Frame { 
    savedCR6 :: ContextRegister,   -- Saved Lambda Calculus Abstraction
    savedCR7 :: ContextRegister,   -- Saved code capability
    savedOffset :: Int              -- Return address offset
} deriving (Show)

-- | CPUState is the complete state of the PP250 processor at any moment.
-- This is the central data structure passed through all operations:
--   c_regs      - Working set of 8 context registers (CR0-CR7)
--   d_regs      - 8 data registers for numeric computation (DR0-DR7)
--   ip_Offset   - Current instruction pointer
--   sr_Status   - System status flags
--   linkStack   - Call/return stack for nested procedures
--   cr8_Thread  - CR8: Current thread/user identity capability
--   cr15_NS     - CR15: Root namespace capability
--   ram_Threads - Storage for suspended thread states
--   scope_CList - C-List: Available capability keys for this context
data CPUState = CPUState {
    c_regs     :: Map.Map Int ContextRegister, 
    d_regs     :: Map.Map Int Word64,          
    ip_Offset  :: Int,
    sr_Status  :: [String],
    linkStack  :: [StackFrame],
    cr8_Thread :: ContextRegister,             
    cr15_NS    :: ContextRegister,             
    ram_Threads :: Map.Map String SavedThreadState, 
    scope_CList :: Map.Map Int ContextRegister
} deriving (Show)

-- =========================================================================
-- 2. HUD & TELEMETRY
-- =========================================================================
-- Display functions for the Head-Up Display (HUD) that shows the current
-- system state. These utilities format the CPU state into readable tables.

-- | Pad a string to exactly n characters, truncating if too long.
-- Used for fixed-width table columns where alignment is critical.
pad :: Int -> String -> String
pad n s = if length s >= n then take n s else s ++ replicate (n - length s) ' '

-- | Pad a string to at least n characters WITHOUT truncating.
-- Preserves full content even if it exceeds the minimum width.
-- Use this when data integrity is more important than table alignment.
padNoTrunc :: Int -> String -> String
padNoTrunc n s = s ++ replicate (max 0 (n - length s)) ' '

-- | Convert a list of permissions into a compact display string.
-- Format: "RWX LSEB" where each letter appears if that permission
-- is granted, or "-" if not. Example: "R-X L---" means Read+Execute+Load.
permString :: [Permission] -> String
permString ps = 
    (if PermRead `elem` ps then "R" else "-") ++ 
    (if PermWrite `elem` ps then "W" else "-") ++ 
    (if PermExecute `elem` ps then "X" else "-") ++ " " ++
    (if PermLoad `elem` ps then "L" else "-") ++ 
    (if PermSave `elem` ps then "S" else "-") ++ 
    (if PermEnter `elem` ps then "E" else "-") ++ 
    (if PermBind `elem` ps then "B" else "-")

-- | Format a Location value as a human-readable string.
-- Local addresses show as "Local <n>", literals as "Lit <name>".
formatLoc :: Location -> String
formatLoc (Local n) = "Local " ++ show n
formatLoc (Literal s) = "Lit " ++ s

-- | Display the complete system telemetry HUD.
-- Shows all context registers (CR0-CR7), system registers (CR8, CR15),
-- instruction pointer, and all 8 data registers in formatted tables.
displayHUD :: CPUState -> IO ()
displayHUD cpu = do
    -- Header and context register table
    putStrLn "\n========================== PP250 SYSTEM TELEMETRY =========================="
    putStrLn "|  CONTEXT REGISTERS (WORKING SET)                                         |"
    putStrLn "+-----+---------------------+-----------------------+---------+-------------+"
    putStrLn "| ID  | REGISTER ROLE       | OBJECT NAME           | RWX LSE | LOCATION    |"
    putStrLn "+-----+---------------------+-----------------------+---------+-------------+"
    mapM_ printCR [0..7]  -- Print each of the 8 context registers
    putStrLn "+-----+---------------------+-----------------------+---------+-------------+"
    
    -- System state section: namespace, thread, instruction pointer
    putStrLn "|  SYSTEM STATE                                                             |"
    putStrLn "+-----------------------------+---------------------------------------------+"
    putStrLn $ "| CR15 (NAMESPACE)            | " ++ pad 43 (cachedName (cr15_NS cpu)) ++ " |"
    putStrLn $ "| CR8  (THREAD/USER)          | " ++ pad 43 (cachedName (cr8_Thread cpu)) ++ " |"
    putStrLn $ "| IP   (INSTRUCTION PTR)      | " ++ pad 43 (show (ip_Offset cpu)) ++ " |"
    putStrLn "+-----------------------------+---------------------------------------------+"
    
    -- Data registers section: 8 x 64-bit registers displayed in hex
    putStrLn "|  DATA REGISTERS                                                           |"
    putStrLn "+-------------------+-------------------+-------------------+---------------+"
    let dr n = printf "0x%016X" (Map.findWithDefault 0 n (d_regs cpu)) :: String
    putStrLn $ "| DR0: " ++ padNoTrunc 18 (dr 0) ++ " | DR1: " ++ padNoTrunc 18 (dr 1) ++ " |"
    putStrLn $ "| DR2: " ++ padNoTrunc 18 (dr 2) ++ " | DR3: " ++ padNoTrunc 18 (dr 3) ++ " |"
    putStrLn $ "| DR4: " ++ padNoTrunc 18 (dr 4) ++ " | DR5: " ++ padNoTrunc 18 (dr 5) ++ " |"
    putStrLn $ "| DR6: " ++ padNoTrunc 18 (dr 6) ++ " | DR7: " ++ padNoTrunc 18 (dr 7) ++ " |"
    putStrLn "============================================================================="
  where
    -- Helper to print a single context register row
    printCR i = do
        let reg = Map.findWithDefault emptyCR i (c_regs cpu)
        -- Assign semantic roles to special registers
        let role = case i of 
                7 -> "NUCLEUS (CODE)     "   -- CR7: Currently executing code
                6 -> "C-LIST LCA         "   -- CR6: Lambda Calculus Abstraction
                _ -> "GENERAL CAPABILITY "   -- CR0-CR5: General purpose
        let name = if cachedName reg == "NULL" 
                   then padNoTrunc 25 "NULL" 
                   else padNoTrunc 25 (cachedName reg)
        let pStr = if cachedName reg == "NULL" 
                   then "--- ---" 
                   else take 7 (permString (activePerms reg))
        let locStr = padNoTrunc 11 (formatLoc (cachedLoc reg))
        putStrLn $ "| CR" ++ pad 2 (show i) ++ " | " ++ role ++ " | " ++ name ++ " | " ++ pStr ++ " | " ++ locStr ++ " |"

-- | An empty/null context register used as a default value.
-- Represents the absence of a capability (no access rights).
emptyCR :: ContextRegister
emptyCR = ContextReg (Local 0) "NULL" [] False

-- =========================================================================
-- 3. THE BOOT STEPS
-- =========================================================================
-- The PP250 boot sequence initializes the system in a specific order:
-- 1. Hardware Reset - Clear all registers to null state
-- 2. Load Namespace - Establish the root capability (CR15)
-- 3. Load Thread - Set up the initial user context (CR8)
-- 4. Load Resources - Load code and tools into working registers

-- | An empty saved thread state used as a default.
emptyState :: SavedThreadState
emptyState = SavedState 0 [] Map.empty Map.empty []

-- | Helper to create a new context register with given properties.
-- Convenience function that sets isLocked to False by default.
mkCR :: String -> Location -> [Permission] -> ContextRegister
mkCR name loc perms = ContextReg loc name perms False

-- | BOOT STEP 1: Hardware Reset
-- Clears all registers to their null/zero state. This is the first
-- step when the machine is powered on, establishing a clean slate.
bootStep1_HardwareReset :: IO CPUState
bootStep1_HardwareReset = do
    putStrLn "\n[BOOT STEP 1] HARDWARE RESET"
    putStrLn "   > Power Energized. Clearing All Registers to NULL..."
    -- Initialize all 8 context registers to empty
    let emptyRegs = Map.fromList [(i, emptyCR) | i <- [0..7]]
    -- Initialize all 8 data registers to zero
    let emptyData = Map.fromList [(i, 0) | i <- [0..7]]
    -- Create initial CPU state with everything cleared
    let cpu = CPUState emptyRegs emptyData 0 ["RESET"] [] emptyCR emptyCR Map.empty Map.empty
    displayHUD cpu 
    return cpu

-- | BOOT STEP 2: Load Namespace (CR15)
-- Loads the root namespace capability into CR15. The namespace defines
-- the scope of all objects accessible to this system instance.
bootStep2_LoadNamespace :: CPUState -> IO CPUState
bootStep2_LoadNamespace cpu = do
    putStrLn "\n[BOOT STEP 2] LOAD NAMESPACE (CR15)"
    putStrLn "   > Formatting Memory at 4000..."
    -- Create the boot namespace capability with Read and Load permissions
    let bootNS = mkCR "Boot Namespace" (Local 4000) [PermRead, PermLoad]
    putStrLn "   > LOADING CR15..."
    let newCpu = cpu { cr15_NS = bootNS }
    displayHUD newCpu
    return newCpu

-- | BOOT STEP 3: Load Thread Context (CR8)
-- Loads the initial user/thread identity into CR8. This establishes
-- "who" is running on the system - in this case, user "Kenneth".
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
-- Also sets up the simulation environment with an alternate thread
-- ("Operator") that can be switched to via the CHANGE instruction.
bootStep4_LoadResources :: CPUState -> IO CPUState
bootStep4_LoadResources cpu = do
    putStrLn "\n[BOOT STEP 4] LOAD CODE & TOOLS (CR7/CR6)"
    -- Load diagnostic code into CR7 (the code register)
    putStrLn "   > Fetching 'Diag Code' -> CR7..."
    let cr7 = mkCR "Diag Code" (Local 16000) [PermRead, PermExecute]
    -- Load test architecture tools into CR6 (the LCA register)
    putStrLn "   > Fetching 'Test Arch' -> CR6..."
    let cr6 = mkCR "Test Arch" (Local 20000) [PermEnter, PermBind]
    
    -- Insert the new capabilities into the register file
    let newRegs = Map.insert 7 cr7 (Map.insert 6 cr6 (c_regs cpu))
    
    -- Setup Simulation Environment: Create an alternate thread "Operator"
    -- that can be switched to using the CHANGE instruction
    let opCR8   = mkCR "Operator" (Local 90000) []
    let opState = SavedState 999 ["READY"] Map.empty Map.empty []
    let ram     = Map.fromList [("Operator", opState)]
    -- Add Operator to the C-List at index 1
    let queue   = Map.fromList [(1, opCR8)]
    
    -- Pre-load Data Registers with test values: DR0=10 (0xA), DR1=5
    let dataRegs = Map.fromList [(0, 10), (1, 5)]
    
    let newCpu = cpu { c_regs = newRegs, d_regs = dataRegs, ram_Threads = ram, scope_CList = queue, ip_Offset = 100 }
    displayHUD newCpu
    return newCpu

-- =========================================================================
-- 4. INSTRUCTION SET
-- =========================================================================
-- These functions implement the PP250's instruction set. Each instruction
-- operates on the CPU state and returns a new (modified) state.

-- | Lock a capability key when storing it to memory.
-- Locked keys cannot be modified until unlocked (fetched back).
storeKey :: ContextRegister -> ContextRegister
storeKey reg = if cachedName reg == "NULL" then reg else reg { isLocked = True }

-- | Unlock a capability key when fetching it from memory.
-- This makes the key available for modification again.
fetchKey :: ContextRegister -> ContextRegister
fetchKey storedKey = if cachedName storedKey == "NULL" then storedKey else storedKey { isLocked = False }

-- | CHANGE Instruction: Context Switch / Process Switch
-- Saves the current thread's state and restores a different thread
-- specified by the offset into the C-List. This is how the PP250
-- implements multitasking - by swapping complete execution contexts.
-- Returns Either an error message (Left) or the new CPU state (Right).
instrCHANGE :: CPUState -> Int -> IO (Either String CPUState)
instrCHANGE cpu offset = do
    putStrLn $ "\n[OP] CHANGE PROCESS (Offset " ++ show offset ++ ")..."
    let currentName = cachedName (cr8_Thread cpu)
    
    -- SAVE PHASE: Snapshot current thread's complete state
    putStrLn $ "   > SAVING: " ++ currentName
    let savedDRs = d_regs cpu
    let savedCRs = Map.map storeKey (c_regs cpu)  -- Lock all keys
    let snapshot = SavedState (ip_Offset cpu) (sr_Status cpu) savedDRs savedCRs (linkStack cpu)
    let newRAM   = Map.insert currentName snapshot (ram_Threads cpu)
    
    -- RESTORE PHASE: Load the target thread's state from C-List
    let newCap = Map.findWithDefault emptyCR offset (scope_CList cpu)
    let newName = cachedName newCap
    if newName == "NULL" 
        then return $ Left "CHANGE Failed (Target NULL)"  -- No thread at this offset
        else do
            -- Retrieve the target thread's saved state
            let newState = Map.findWithDefault emptyState newName newRAM
            let restoredCRs = Map.map fetchKey (storedKeys newState)  -- Unlock keys
            
            -- Build the new CPU state with restored thread context
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

-- | EXECUTE_Math Instruction: Arithmetic Operations
-- Performs math operations on data registers using 2-address format:
-- DR[dest] = DR[dest] <op> DR[src]
-- Supported operations: ADD, SUB, POW (exponentiation)
instrEXECUTE_Math :: CPUState -> String -> Int -> Int -> CPUState
instrEXECUTE_Math cpu op destIdx srcIdx = 
    let vDest = Map.findWithDefault 0 destIdx (d_regs cpu)
        vSrc  = Map.findWithDefault 0 srcIdx (d_regs cpu)
        res = case op of 
            "ADD" -> vDest + vSrc   -- Addition
            "SUB" -> vDest - vSrc   -- Subtraction
            "POW" -> vDest ^ vSrc   -- Exponentiation
            _     -> vDest          -- Unknown op: no change
        newDRs = Map.insert destIdx res (d_regs cpu)
    in cpu { d_regs = newDRs }

-- | LOAD Instruction: Load a Capability
-- Loads a new capability into a context register. Requires PermLoad
-- permission on the source register. CR7 (code register) is protected
-- and cannot be loaded directly - use EXECUTE instead.
instrLOAD :: CPUState -> Int -> Int -> Int -> Either String CPUState
instrLOAD cpu d s i = 
    if d==7 then Left "Use EXECUTE to load CR7" else  -- CR7 is protected
    let src = Map.findWithDefault emptyCR s (c_regs cpu) in
    if not (PermLoad `elem` activePerms src) 
        then Left "TRAP: No LOAD Perm"  -- Permission check failed
        else Right cpu { c_regs = Map.insert d (mkCR ("Obj_"++show i) (Local i) [PermRead]) (c_regs cpu) }

-- | SAVE Instruction: Save/Bind Data
-- Saves data to a location via a capability. Requires PermSave
-- permission on the destination register.
instrSAVE :: CPUState -> Int -> Int -> Either String String
instrSAVE cpu d s = 
    let dst = Map.findWithDefault emptyCR d (c_regs cpu) in
    if not (PermSave `elem` activePerms dst) 
        then Left "TRAP: No SAVE Perm"  -- Permission check failed
        else Right "SUCCESS: Bound."

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
            -- 1. Create Stack Frame (Snapshot of current context)
            frame = Frame { savedCR6 = currentCR6, savedCR7 = currentCR7, savedOffset = ip_Offset cpu }
            
            -- 2. Simulate Entry (Target -> CR6)
            -- In a real system, we would fetch CR7 from inside the new CR6. 
            -- Here, we mock a code object to show the context switch.
            newCodeName = "Code[" ++ cachedName targetCR ++ "]"
            newCR7 = mkCR newCodeName (Local 0) [PermRead, PermExecute]
            
            newRegs = Map.insert 6 targetCR (Map.insert 7 newCR7 (c_regs cpu))
        in
        Right cpu { 
            c_regs = newRegs, 
            linkStack = frame : linkStack cpu,  -- Push frame onto stack
            ip_Offset = 0                       -- Reset IP for new procedure
        }

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
                -- Restore Registers from the saved Frame
                restoredRegs = Map.insert 6 (savedCR6 frame) (Map.insert 7 (savedCR7 frame) (c_regs cpu))
            in
            Right cpu {
                c_regs = restoredRegs,
                linkStack = rest,              -- Pop frame from stack
                ip_Offset = savedOffset frame  -- Restore saved IP
            }

-- =========================================================================
-- 5. CONSOLE (CRASH-PROOF)
-- =========================================================================
-- The interactive console provides a command-line interface to the PP250.
-- All user input is validated before processing to prevent crashes.

-- | Safe integer parser that returns Nothing for invalid input
-- instead of throwing an exception. Wraps readMaybe for clarity.
readInt :: String -> Maybe Int
readInt s = readMaybe s

-- | Display the command reference help screen.
-- Lists all available commands with their syntax and descriptions.
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

-- | Main console loop: Read-Eval-Print Loop (REPL)
-- Prompts for user input, parses the command, executes it,
-- and loops back for the next command. All input is validated.
runConsole :: CPUState -> IO ()
runConsole cpu = do
    putStr ">> CMD (HELP for commands): "
    hFlush stdout  -- Ensure prompt appears before waiting for input
    input <- getLine
    
    -- Pattern match on the parsed words of input
    case words input of
        -- EXIT: Terminate the console loop
        ["EXIT"] -> putStrLn "--- SHUTDOWN ---"
        
        -- HELP: Display command reference
        ["HELP"] -> showHelp >> runConsole cpu
        
        -- HUD: Display full system telemetry
        ["HUD"] -> displayHUD cpu >> runConsole cpu
        
        -- NS: Display CR15 namespace capability details
        ["NS"] -> do
            let ns = cr15_NS cpu
            putStrLn "\n+----------------------- CR15 NAMESPACE -------------------------+"
            putStrLn $ "| Name:        " ++ padNoTrunc 49 (cachedName ns) ++ " |"
            putStrLn $ "| Location:    " ++ padNoTrunc 49 (formatLoc (cachedLoc ns)) ++ " |"
            putStrLn $ "| Permissions: " ++ padNoTrunc 49 (permString (activePerms ns)) ++ " |"
            putStrLn $ "| Locked:      " ++ padNoTrunc 49 (show (isLocked ns)) ++ " |"
            putStrLn "+----------------------------------------------------------------+"
            runConsole cpu
        
        -- CLIST: Display all capability keys in the C-List
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
          where
            -- Helper to print a single C-List entry row
            printCListEntry (idx, reg) = do
                let idxStr = padNoTrunc 4 (show idx)
                let nameStr = padNoTrunc 25 (cachedName reg)
                let locStr = padNoTrunc 16 (formatLoc (cachedLoc reg))
                let permStr = take 7 (permString (activePerms reg))
                let lockStr = padNoTrunc 6 (if isLocked reg then "Yes" else "No")
                putStrLn $ "| " ++ idxStr ++ " | " ++ nameStr ++ " | " ++ locStr ++ " | " ++ permStr ++ " | " ++ lockStr ++ " |"
        
        -- CHANGE: Context switch to a different thread
        ("CHANGE":xStr:_) -> case readInt xStr of
            Just x  -> do
                result <- instrCHANGE cpu x
                case result of
                    Right newCpu -> runConsole newCpu  -- Success: continue with new context
                    Left err     -> putStrLn ("[TRAP] " ++ err) >> runConsole cpu  -- Failed: stay in current context
            Nothing -> putStrLn "[ERROR] Invalid Argument (Expected Integer)" >> runConsole cpu
            
        -- MATH Operations: ADD, SUB, POW with 2 register arguments
        (op:dStr:sStr:_) | op `elem` ["ADD", "SUB", "POW"] -> 
            case (readInt dStr, readInt sStr) of
                (Just d, Just s) -> do
                    let newCpu = instrEXECUTE_Math cpu op d s
                    -- Show the before and after values for feedback
                    let oldVal = Map.findWithDefault 0 d (d_regs cpu)
                    let newVal = Map.findWithDefault 0 d (d_regs newCpu)
                    putStrLn $ "   [OK] DR" ++ show d ++ ": 0x" ++ printf "%X" oldVal ++ " -> 0x" ++ printf "%X" newVal
                    runConsole newCpu
                _                -> putStrLn "[ERROR] Invalid Register Index" >> runConsole cpu

        -- LOAD: Load capability from source register to destination
        ("LOAD":dStr:sStr:iStr:_) -> 
            case (readInt dStr, readInt sStr, readInt iStr) of
                (Just d, Just s, Just i) -> case instrLOAD cpu d s i of
                    Right c -> do
                        putStrLn $ "   [OK] Loaded object into CR" ++ show d
                        runConsole c
                    Left e  -> putStrLn ("[TRAP] " ++ e) >> runConsole cpu
                _ -> putStrLn "[ERROR] Invalid Arguments" >> runConsole cpu

        -- SAVE: Save data via destination capability
        ("SAVE":dStr:sStr:_) ->
            case (readInt dStr, readInt sStr) of
                (Just d, Just s) -> case instrSAVE cpu d s of
                    Right m -> putStrLn ("   [OK] " ++ m) >> runConsole cpu
                    Left e  -> putStrLn ("[TRAP] " ++ e) >> runConsole cpu
                _ -> putStrLn "[ERROR] Invalid Arguments" >> runConsole cpu

        -- CALL: Enter a procedure via capability in specified register
        ("CALL":rStr:_) -> case readInt rStr of
            Just r -> case instrCALL cpu r of
                Right newCpu -> do
                    putStrLn $ "   [OK] CALL to CR" ++ show r ++ " - Entered '" ++ cachedName (Map.findWithDefault emptyCR 6 (c_regs newCpu)) ++ "'"
                    putStrLn $ "        Stack depth: " ++ show (length (linkStack newCpu))
                    runConsole newCpu
                Left e -> putStrLn ("[TRAP] " ++ e) >> runConsole cpu
            Nothing -> putStrLn "[ERROR] Invalid Register Index" >> runConsole cpu

        -- RETURN: Exit from current procedure, restore caller's context
        ["RETURN"] -> case instrRETURN cpu of
            Right newCpu -> do
                putStrLn $ "   [OK] RETURN - Restored to '" ++ cachedName (Map.findWithDefault emptyCR 6 (c_regs newCpu)) ++ "'"
                putStrLn $ "        IP restored to: " ++ show (ip_Offset newCpu)
                runConsole newCpu
            Left e -> putStrLn ("[TRAP] " ++ e) >> runConsole cpu
                
        -- Empty input: Just re-prompt (no error)
        [] -> runConsole cpu
        
        -- Unknown command: Show error and continue
        _ -> putStrLn "[ERROR] Unknown Command. Type HELP for available commands." >> runConsole cpu

-- =========================================================================
-- 6. MAIN ENTRY POINT
-- =========================================================================
-- The main function orchestrates the complete boot sequence and then
-- enters the interactive console loop.

main :: IO ()
main = do
    putStrLn "--- PP250 BOOT SEQUENCE START ---"
    
    -- Execute the 4-step boot sequence, with user prompts between steps
    cpu1 <- bootStep1_HardwareReset
    putStrLn ">> Press ENTER for Step 2..." >> getLine
    
    cpu2 <- bootStep2_LoadNamespace cpu1
    putStrLn ">> Press ENTER for Step 3..." >> getLine
    
    cpu3 <- bootStep3_LoadThread cpu2
    putStrLn ">> Press ENTER for Step 4..." >> getLine
    
    cpu4 <- bootStep4_LoadResources cpu3
    putStrLn ">> BOOT COMPLETE. ENTERING CONSOLE..."
    putStrLn ">> Try: ADD 0 1 (Computes DR0 = DR0 + DR1)" >> getLine
    
    -- Enter the main console loop
    runConsole cpu4
