-- =========================================================================
-- PP250.Core.Utils: Utility Functions for the PP250 Meta-Machine
-- =========================================================================
-- This module provides shared utility functions used across the simulator,
-- including string formatting, permission display, and key locking.

module PP250.Core.Utils (
    pad,
    padNoTrunc,
    permString,
    formatLoc,
    storeKey,
    fetchKey,
    readInt
) where

import PP250.Core.Types
import Text.Read (readMaybe)

-- | Pad a string to exactly n characters, truncating if too long.
-- Used for fixed-width table columns where alignment is critical.
pad :: Int -> String -> String
pad n s = if length s >= n then take n s else s ++ replicate (n - length s) ' '

-- | Pad a string to at least n characters WITHOUT truncating.
-- Preserves full content even if it exceeds the minimum width.
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
formatLoc :: Location -> String
formatLoc (Local n) = "Local " ++ show n
formatLoc (Literal s) = "Lit " ++ s

-- | Lock a capability key when storing it to memory.
-- Locked keys cannot be modified until unlocked (fetched back).
storeKey :: ContextRegister -> ContextRegister
storeKey reg = if cachedName reg == "NULL" then reg else reg { isLocked = True }

-- | Unlock a capability key when fetching it from memory.
-- This makes the key available for modification again.
fetchKey :: ContextRegister -> ContextRegister
fetchKey storedKey = if cachedName storedKey == "NULL" then storedKey else storedKey { isLocked = False }

-- | Safe integer parser that returns Nothing for invalid input
-- instead of throwing an exception.
readInt :: String -> Maybe Int
readInt s = readMaybe s
