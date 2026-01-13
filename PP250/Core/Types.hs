-- =========================================================================
-- PP250.Core.Types: Shared Data Types for the PP250 Meta-Machine
-- =========================================================================
-- This module defines all core data structures used throughout the PP250
-- simulator. These types model the capability-based architecture where
-- all access is controlled through "Golden Tokens" (capability keys).

module PP250.Core.Types (
    Location(..),
    Permission(..),
    ContextRegister(..),
    ConditionFlags(..),
    SavedThreadState(..),
    StackFrame(..),
    CPUState(..),
    emptyCR,
    emptyState,
    emptyFlags,
    mkCR
) where

import qualified Data.Map as Map
import Data.Word (Word64)

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

-- | ConditionFlags represents ARM-style NZCV condition flags.
-- These flags are set by arithmetic/logic operations and used for
-- conditional execution. This matches the standard ARM architecture:
--   flagN - Negative: Set when result is negative (sign bit = 1)
--   flagZ - Zero: Set when result is zero
--   flagC - Carry: Set on unsigned overflow, or shift carry-out
--   flagV - Overflow: Set on signed overflow (two's complement)
data ConditionFlags = CondFlags {
    flagN :: Bool,  -- Negative flag (bit 31 of result)
    flagZ :: Bool,  -- Zero flag (result == 0)
    flagC :: Bool,  -- Carry flag (unsigned overflow)
    flagV :: Bool   -- Overflow flag (signed overflow)
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
    savedCR6 :: ContextRegister,
    savedCR7 :: ContextRegister,
    savedOffset :: Int
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
    condFlags  :: ConditionFlags,              -- ARM-style NZCV condition flags
    linkStack  :: [StackFrame],
    cr8_Thread :: ContextRegister,             
    cr15_NS    :: ContextRegister,             
    ram_Threads :: Map.Map String SavedThreadState, 
    scope_CList :: Map.Map Int ContextRegister
} deriving (Show)

-- | An empty/null context register used as a default value.
-- Represents the absence of a capability (no access rights).
emptyCR :: ContextRegister
emptyCR = ContextReg (Local 0) "NULL" [] False

-- | An empty saved thread state used as a default.
emptyState :: SavedThreadState
emptyState = SavedState 0 [] Map.empty Map.empty []

-- | Empty/cleared condition flags (all False).
-- Used during hardware reset and initialization.
emptyFlags :: ConditionFlags
emptyFlags = CondFlags False False False False

-- | Helper to create a new context register with given properties.
-- Convenience function that sets isLocked to False by default.
mkCR :: String -> Location -> [Permission] -> ContextRegister
mkCR name loc perms = ContextReg loc name perms False
