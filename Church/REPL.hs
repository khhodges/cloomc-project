module Church.REPL (
    runChurchREPL
) where

import qualified Data.Map as Map
import Data.Char (toUpper)
import System.IO (hFlush, stdout)
import Control.Exception (catch, IOException)
import Church.Types
import Church.Primitives
import Church.Machine
import Church.Abstractions

banner :: IO ()
banner = do
    putStrLn ""
    putStrLn "╔══════════════════════════════════════════════════════════════════╗"
    putStrLn "║          CHURCH COMPUTER — Pure Lambda Machine REPL            ║"
    putStrLn "║                                                                ║"
    putStrLn "║  Six instructions: LOAD  SAVE  CALL  RETURN  LAMBDA  TPERM    ║"
    putStrLn "║  Zero Turing instructions: No ADD, MOV, CMP, or BRANCH        ║"
    putStrLn "║  All computation via Church-encoded lambda calculus            ║"
    putStrLn "║                                                                ║"
    putStrLn "║  Type HELP for commands         Type EXIT to quit              ║"
    putStrLn "╚══════════════════════════════════════════════════════════════════╝"
    putStrLn ""

helpText :: IO ()
helpText = do
    putStrLn ""
    putStrLn "+======================== CHURCH COMPUTER COMMANDS ========================+"
    putStrLn "|                                                                          |"
    putStrLn "|  SYMBOLIC MATH (Lovelace style — one operation per line):                 |"
    putStrLn "|    3 + 5           sqrt(16)       -- arithmetic & functions               |"
    putStrLn "|    6 * 7           log(100)       -- operators: + - * / % ^               |"
    putStrLn "|    2 ^ 10          succ(n)        -- functions: sqrt log exp pow succ pred|"
    putStrLn "|                                                                          |"
    putStrLn "|  VARIABLES (Ada Lovelace style):                                         |"
    putStrLn "|    let x = 3 + 5                    -- store result as 'x'               |"
    putStrLn "|    let y = x * 2                    -- use variables in expressions      |"
    putStrLn "|    let z = sqrt(x)                  -- functions work too                |"
    putStrLn "|    VARS                             -- show all variables                 |"
    putStrLn "|    CLEAR                            -- clear all variables                |"
    putStrLn "|                                                                          |"
    putStrLn "|  CALL SYNTAX (explicit):                                                 |"
    putStrLn "|    Call(SlideRule.ADD, 3, 5)         -- explicit abstraction call         |"
    putStrLn "|    Call(Lambda.MUL, x, ANS)          -- ANS = last result                |"
    putStrLn "|                                                                          |"
    putStrLn "|  RAW INSTRUCTIONS (low-level):                                           |"
    putStrLn "|    LOAD  CR1 SlideRule           -- Load GT into context register         |"
    putStrLn "|    TPERM CR1 E                   -- Verify permission                     |"
    putStrLn "|    CALL  CR1                     -- Enter abstraction scope               |"
    putStrLn "|    RETURN                        -- Return from scope                     |"
    putStrLn "|                                                                          |"
    putStrLn "|  INSPECTION:                                                             |"
    putStrLn "|    NS                            -- Show namespace (all named GTs)        |"
    putStrLn "|    CLIST SlideRule               -- Show abstraction's C-List             |"
    putStrLn "|    REGS                          -- Show context & data registers         |"
    putStrLn "|    TRACE                         -- Show instruction trace                |"
    putStrLn "|                                                                          |"
    putStrLn "|  TURING TEST:                                                            |"
    putStrLn "|    ADD 3 5                       -- Attempt Turing instruction -> FAULT   |"
    putStrLn "|    MOV DR0 DR1                   -- Attempt register transfer -> FAULT    |"
    putStrLn "|    B 100                         -- Attempt branch -> FAULT               |"
    putStrLn "|                                                                          |"
    putStrLn "|  PROGRAMS:                                                               |"
    putStrLn "|    RUN Church/bernoulli.church   -- run a program file                   |"
    putStrLn "|                                                                          |"
    putStrLn "|  HELP, EXIT                                                              |"
    putStrLn "+--------------------------------------------------------------------------+"
    putStrLn ""

type VarStore = Map.Map String Integer

runChurchREPL :: IO ()
runChurchREPL = do
    banner
    let ms = buildChurchMachine
    let vars = Map.empty :: VarStore
    loop ms vars

loop :: MachineState -> VarStore -> IO ()
loop ms vars = do
    putStr "Church> "
    hFlush stdout
    input <- getLine
    let tokens = words input
    let upperTokens = map (map toUpper) tokens
    case upperTokens of
        ["EXIT"]   -> putStrLn "--- CHURCH COMPUTER SHUTDOWN ---"
        ["QUIT"]   -> putStrLn "--- CHURCH COMPUTER SHUTDOWN ---"
        ["HELP"]   -> helpText >> loop ms vars
        ["NS"]     -> showNamespace ms >> loop ms vars
        ["REGS"]   -> showRegisters ms >> loop ms vars
        ["TRACE"]  -> showTrace ms >> loop ms vars
        ["VARS"]   -> showVars vars >> loop ms vars
        ["CLEAR"]  -> do
            putStrLn "  [OK] Variables cleared"
            loop ms (Map.empty :: VarStore)
        ("RUN":_) -> doRunFile ms vars (unwords (drop 1 tokens))
        []         -> loop ms vars

        ["CLIST", _] -> showCList ms (tokens !! 1) >> loop ms vars

        ("LOAD":rest) -> doLoad ms vars rest tokens
        ("TPERM":rest) -> doTperm ms vars rest
        ("CALL":rest) -> doCallRaw ms vars rest
        ["RETURN"] -> doReturn ms vars

        _ | isLetSyntax input -> doLetBinding ms vars input
          | isTuringAttempt upperTokens -> turingFault >> loop ms vars
          | isCallSyntax input -> doSymbolicCall ms vars input
          | isMathExpr input -> doMathExpr ms vars input Nothing
          | otherwise -> do
                putStrLn "[ERROR] Unknown command. Type HELP for available commands."
                loop ms vars

turingFault :: IO ()
turingFault = do
    putStrLn ""
    putStrLn "  +==============================================================+"
    putStrLn "  |  FAULT: Turing-domain instruction rejected                   |"
    putStrLn "  |                                                              |"
    putStrLn "  |  This is a Pure Church Computer.                             |"
    putStrLn "  |  No ADD, SUB, MUL, MOV, CMP, AND, OR, B, BL, LDR, STR      |"
    putStrLn "  |  instructions exist in this architecture.                    |"
    putStrLn "  |                                                              |"
    putStrLn "  |  Use: Call(SlideRule.ADD, 3, 5) for arithmetic               |"
    putStrLn "  |  Use: Call(Lambda.SUCC, 7) for Church successor              |"
    putStrLn "  +==============================================================+"
    putStrLn ""

isCallSyntax :: String -> Bool
isCallSyntax s = isPrefixOf' "CALL(" (map toUpper (dropWhile (== ' ') s))

isPrefixOf' :: String -> String -> Bool
isPrefixOf' [] _ = True
isPrefixOf' _ [] = False
isPrefixOf' (a:as) (b:bs) = a == b && isPrefixOf' as bs

isTuringAttempt :: [String] -> Bool
isTuringAttempt [] = False
isTuringAttempt (cmd:_) = cmd `elem`
    [ "ADD", "SUB", "MUL", "DIV", "MOV", "MVN", "NEG"
    , "AND", "ORR", "EOR", "BIC", "NOT"
    , "LSL", "LSR", "ASR", "ROR"
    , "CMP", "CMN", "TST", "TEQ"
    , "B", "BL", "BX", "BEQ", "BNE"
    , "LDR", "STR", "LDM", "STM"
    , "PUSH", "POP", "SWI", "SVC"
    ]

doRunFile :: MachineState -> VarStore -> String -> IO ()
doRunFile ms vars filename = do
    let fn = strip filename
    result <- (Right <$> readFile fn) `catch` (\e -> return (Left (show (e :: IOException))))
    case result of
        Left err -> do
            putStrLn $ "[ERROR] Cannot read file: " ++ err
            loop ms vars
        Right contents -> do
            putStrLn $ "  -- Running: " ++ fn ++ " --"
            putStrLn ""
            let cmds = filter isExecutable (lines contents)
            runCommands ms vars cmds
  where
    isExecutable line =
        let stripped = dropWhile (== ' ') line
        in not (null stripped) && not (isPrefixOf' "--" stripped)

runCommands :: MachineState -> VarStore -> [String] -> IO ()
runCommands ms vars [] = loop ms vars
runCommands ms vars (cmd:rest) = do
    let tokens = words cmd
    let upperTokens = map (map toUpper) tokens
    case upperTokens of
        ["EXIT"]   -> putStrLn "--- CHURCH COMPUTER SHUTDOWN ---"
        ["QUIT"]   -> putStrLn "--- CHURCH COMPUTER SHUTDOWN ---"
        ["VARS"]   -> showVars vars >> runCommands ms vars rest
        ["REGS"]   -> showRegisters ms >> runCommands ms vars rest
        ["NS"]     -> showNamespace ms >> runCommands ms vars rest
        [] -> runCommands ms vars rest
        _ | isLetSyntax cmd -> doLetBindingBatch ms vars cmd rest
          | isCallSyntax cmd -> doCallBatch ms vars cmd rest
          | isMathExpr cmd -> doMathBatch ms vars cmd rest Nothing
          | isTuringAttempt upperTokens -> turingFault >> runCommands ms vars rest
          | otherwise -> do
                putStrLn $ "[ERROR] Unknown command: " ++ cmd
                runCommands ms vars rest

doLetBindingBatch :: MachineState -> VarStore -> String -> [String] -> IO ()
doLetBindingBatch ms vars input rest = do
    let stripped = dropWhile (== ' ') input
    let afterLet = drop 4 stripped
    case break (== '=') afterLet of
        (_, []) -> do
            putStrLn "[ERROR] Invalid let syntax."
            runCommands ms vars rest
        (varPart, _:callPart) -> do
            let varName = strip varPart
            let callStr = strip callPart
            let parsed = case parseCallSyntax callStr of
                    Just r  -> Just r
                    Nothing -> parseMathExpr callStr
            case parsed of
                Nothing -> do
                    putStrLn $ "[ERROR] Invalid expression in: " ++ input
                    runCommands ms vars rest
                Just (absName_, method, argTokens) ->
                    case resolveArgs vars ms argTokens of
                        Left badVar -> do
                            putStrLn $ "[ERROR] Unknown variable: " ++ badVar ++ " in: " ++ input
                            runCommands ms vars rest
                        Right args -> do
                            putStrLn $ "  -- " ++ strip input ++ "  =>  Call(" ++ absName_ ++ "." ++ method ++ concatMap (\a -> ", " ++ show a) args ++ ") --"
                            let ms0 = ms { msTrace = [] }
                            result <- executeViaInstructions ms0 absName_ method args
                            case result of
                                Left fault -> do
                                    putStrLn $ "  [FAULT] " ++ show fault
                                    runCommands ms vars rest
                                Right ms' -> do
                                    let resultVal = Map.findWithDefault 0 0 (msDataRegs ms')
                                    putStrLn $ "  " ++ varName ++ " = " ++ show resultVal
                                    let vars' = Map.insert varName resultVal vars
                                    runCommands ms' vars' rest

doMathBatch :: MachineState -> VarStore -> String -> [String] -> Maybe String -> IO ()
doMathBatch ms vars input rest maybeVar =
    case parseMathExpr input of
        Nothing -> do
            putStrLn $ "[ERROR] Invalid expression: " ++ input
            runCommands ms vars rest
        Just (absName_, method, argTokens) ->
            case resolveArgs vars ms argTokens of
                Left badVar -> do
                    putStrLn $ "[ERROR] Unknown variable: " ++ badVar ++ " in: " ++ input
                    runCommands ms vars rest
                Right args -> do
                    putStrLn $ "  -- " ++ strip input ++ "  =>  Call(" ++ absName_ ++ "." ++ method ++ concatMap (\a -> ", " ++ show a) args ++ ") --"
                    let ms0 = ms { msTrace = [] }
                    result <- executeViaInstructions ms0 absName_ method args
                    case result of
                        Left fault -> do
                            putStrLn $ "  [FAULT] " ++ show fault
                            runCommands ms vars rest
                        Right ms' -> do
                            let resultVal = Map.findWithDefault 0 0 (msDataRegs ms')
                            case maybeVar of
                                Just vn -> do
                                    putStrLn $ "  " ++ vn ++ " = " ++ show resultVal
                                    runCommands ms' (Map.insert vn resultVal vars) rest
                                Nothing -> do
                                    putStrLn $ "  Result: " ++ show resultVal
                                    runCommands ms' vars rest

doCallBatch :: MachineState -> VarStore -> String -> [String] -> IO ()
doCallBatch ms vars input rest = do
    case parseCallSyntax input of
        Nothing -> do
            putStrLn $ "[ERROR] Invalid Call: " ++ input
            runCommands ms vars rest
        Just (absName_, method, argTokens) ->
            case resolveArgs vars ms argTokens of
                Left badVar -> do
                    putStrLn $ "[ERROR] Unknown variable: " ++ badVar ++ " in: " ++ input
                    runCommands ms vars rest
                Right args -> do
                    putStrLn $ "  -- Executing: Call(" ++ absName_ ++ "." ++ method ++ concatMap (\a -> ", " ++ show a) args ++ ") --"
                    let ms0 = ms { msTrace = [] }
                    result <- executeViaInstructions ms0 absName_ method args
                    case result of
                        Left fault -> do
                            putStrLn $ "  [FAULT] " ++ show fault
                            runCommands ms vars rest
                        Right ms' -> do
                            let resultVal = Map.findWithDefault 0 0 (msDataRegs ms')
                            putStrLn $ "  Result: " ++ show resultVal
                            runCommands ms' vars rest

isLetSyntax :: String -> Bool
isLetSyntax s =
    let upper = map toUpper (dropWhile (== ' ') s)
    in isPrefixOf' "LET " upper

doLetBinding :: MachineState -> VarStore -> String -> IO ()
doLetBinding ms vars input = do
    let stripped = dropWhile (== ' ') input
    let afterLet = drop 4 stripped
    case break (== '=') afterLet of
        (_, []) -> do
            putStrLn "[ERROR] Invalid let syntax. Use: let x = 3 + 5"
            loop ms vars
        (varPart, _:callPart) -> do
            let varName = strip varPart
            let callStr = strip callPart
            if isCallSyntax callStr
                then doSymbolicCallWithVar ms vars callStr (Just varName)
                else if isMathExpr callStr
                    then doMathExpr ms vars callStr (Just varName)
                    else do
                        putStrLn "[ERROR] Invalid expression. Use: let x = 3 + 5  or  let x = Call(SlideRule.ADD, 3, 5)"
                        loop ms vars

doMathExpr :: MachineState -> VarStore -> String -> Maybe String -> IO ()
doMathExpr ms vars input maybeVar =
    case parseMathExpr input of
        Nothing -> do
            putStrLn "[ERROR] Invalid math expression."
            loop ms vars
        Just (absName_, method, argTokens) ->
            case resolveArgs vars ms argTokens of
                Left badVar -> do
                    putStrLn $ "[ERROR] Unknown variable: " ++ badVar
                    loop ms vars
                Right args -> do
                    putStrLn ""
                    putStrLn $ "  -- " ++ strip input ++ "  =>  Call(" ++ absName_ ++ "." ++ method ++ concatMap (\a -> ", " ++ show a) args ++ ") --"
                    putStrLn ""
                    let ms0 = ms { msTrace = [] }
                    result <- executeViaInstructions ms0 absName_ method args
                    case result of
                        Left fault -> do
                            putStrLn $ "  [FAULT] " ++ show fault
                            putStrLn ""
                            loop ms vars
                        Right ms' -> do
                            showTrace ms'
                            let resultVal = Map.findWithDefault 0 0 (msDataRegs ms')
                            putStrLn ""
                            case maybeVar of
                                Just vn -> do
                                    putStrLn $ "  " ++ vn ++ " = " ++ show resultVal
                                    let vars' = Map.insert vn resultVal vars
                                    putStrLn ""
                                    loop ms' vars'
                                Nothing -> do
                                    putStrLn $ "  Result: " ++ show resultVal
                                    putStrLn ""
                                    loop ms' vars

doSymbolicCall :: MachineState -> VarStore -> String -> IO ()
doSymbolicCall ms vars input = doSymbolicCallWithVar ms vars input Nothing

doSymbolicCallWithVar :: MachineState -> VarStore -> String -> Maybe String -> IO ()
doSymbolicCallWithVar ms vars input maybeVar = do
    case parseCallSyntax input of
        Nothing -> do
            putStrLn "[ERROR] Invalid Call syntax. Use: Call(SlideRule.ADD, 3, 5)"
            loop ms vars
        Just (absName_, method, argTokens) ->
            case resolveArgs vars ms argTokens of
                Left badVar -> do
                    putStrLn $ "[ERROR] Unknown variable: " ++ badVar
                    loop ms vars
                Right args -> do
                    putStrLn ""
                    putStrLn $ "  -- Executing: Call(" ++ absName_ ++ "." ++ method ++ concatMap (\a -> ", " ++ show a) args ++ ") --"
                    putStrLn ""
                    let ms0 = ms { msTrace = [] }
                    result <- executeViaInstructions ms0 absName_ method args
                    case result of
                        Left fault -> do
                            putStrLn $ "  [FAULT] " ++ show fault
                            putStrLn ""
                            loop ms vars
                        Right ms' -> do
                            showTrace ms'
                            let resultVal = Map.findWithDefault 0 0 (msDataRegs ms')
                            putStrLn ""
                            case maybeVar of
                                Just vn -> do
                                    putStrLn $ "  " ++ vn ++ " = " ++ show resultVal
                                    let vars' = Map.insert vn resultVal vars
                                    putStrLn ""
                                    loop ms' vars'
                                Nothing -> do
                                    putStrLn $ "  Result: " ++ show resultVal
                                    putStrLn ""
                                    loop ms' vars

resolveArgs :: VarStore -> MachineState -> [String] -> Either String [Integer]
resolveArgs vars ms tokens = mapM (resolveArg vars ms) tokens

resolveArg :: VarStore -> MachineState -> String -> Either String Integer
resolveArg vars ms token =
    let stripped = strip token
        upper = map toUpper stripped
    in if upper == "ANS"
       then Right (Map.findWithDefault 0 0 (msDataRegs ms))
       else case Map.lookup stripped vars of
           Just v  -> Right v
           Nothing -> case readInteger stripped of
               Just n  -> Right n
               Nothing -> Left stripped

showVars :: VarStore -> IO ()
showVars vars = do
    putStrLn ""
    if Map.null vars
        then putStrLn "  (no variables defined)"
        else do
            putStrLn "+=================== VARIABLES ===========================+"
            mapM_ (\(k, v) -> putStrLn $ "  " ++ k ++ " = " ++ show v) (Map.toList vars)
            putStrLn "+=========================================================+"
    putStrLn ""

executeViaInstructions :: MachineState -> String -> String -> [Integer] -> IO (Either Fault MachineState)
executeViaInstructions ms absName_ method args = do
    r1 <- churchLOAD ms 1 absName_
    case r1 of
        Left f -> return $ Left f
        Right ms1 -> do
            r2 <- churchTPERM ms1 1 [E]
            case r2 of
                Left f -> return $ Left f
                Right ms2 -> do
                    let drMap = case args of
                            []    -> Map.insert 0 0 (msDataRegs ms2)
                            [a]   -> Map.insert 1 a (Map.insert 0 0 (msDataRegs ms2))
                            [a,b] -> Map.insert 2 b (Map.insert 1 a (Map.insert 0 0 (msDataRegs ms2)))
                            _     -> foldr (\(i,v) m -> Map.insert i v m) (Map.insert 0 0 (msDataRegs ms2)) (zip [1..] args)
                    let ms3 = ms2 { msDataRegs = drMap }
                    r3 <- churchCALL ms3 1
                    case r3 of
                        Left f -> return $ Left f
                        Right ms4 ->
                            case Map.lookup absName_ (msAbstractions ms) of
                                Nothing -> return $ Left (ScopeFault ("No abstraction: " ++ absName_))
                                Just abs_ -> do
                                    let cl = absCList abs_
                                    case findMethodSlot cl method of
                                        Nothing -> return $ Left (NullFault ("Method not found in C-List: " ++ method))
                                        Just (slot, _) -> do
                                            r4 <- churchLOAD_CLIST ms4 2 cl slot
                                            case r4 of
                                                Left f -> return $ Left f
                                                Right ms5 -> do
                                                    r5 <- churchTPERM ms5 2 [X]
                                                    case r5 of
                                                        Left f -> return $ Left f
                                                        Right ms6 -> do
                                                            let a = Map.findWithDefault 0 1 (msDataRegs ms6)
                                                            let b = Map.findWithDefault 0 2 (msDataRegs ms6)
                                                            let (result, opName, arity) = resolveChurchOp absName_ method slot a b
                                                            let argStr = if arity == 1 then show a ++ " -> " ++ show result
                                                                         else show a ++ ", " ++ show b ++ " -> " ++ show result
                                                            let lambdaTrace = TraceLambda (absName_ ++ "." ++ opName) argStr
                                                            let ms7 = ms6 { msDataRegs = Map.insert 0 result (msDataRegs ms6)
                                                                           , msTrace = lambdaTrace : msTrace ms6 }
                                                            r6 <- churchRETURN ms7
                                                            case r6 of
                                                                Left f -> return $ Left f
                                                                Right ms8 -> return $ Right ms8

resolveChurchOp :: String -> String -> Int -> Integer -> Integer -> (Integer, String, Int)
resolveChurchOp "Lambda" _ slot a b = case slot of
    3  -> (churchSucc a, "SUCC", 1)
    4  -> (churchPred a, "PRED", 1)
    5  -> (churchAdd a b, "ADD", 2)
    6  -> (churchMul a b, "MUL", 2)
    12 -> (if churchIsZero a then 1 else 0, "IS_ZERO", 1)
    13 -> (churchSub a b, "SUB", 2)
    14 -> (churchDiv a b, "DIV", 2)
    15 -> (churchPow a b, "POW", 2)
    _  -> (0, "UNKNOWN", 0)
resolveChurchOp "SlideRule" _ slot a b = case slot of
    2  -> (churchAdd a b, "ADD", 2)
    3  -> (churchSub a b, "SUB", 2)
    4  -> (churchMul a b, "MUL", 2)
    5  -> (churchDiv a b, "DIV", 2)
    6  -> (churchMod a b, "MOD", 2)
    7  -> (churchLog a, "LOG", 1)
    8  -> (churchExp a b, "EXP", 2)
    9  -> (churchSqrt a, "SQRT", 1)
    10 -> (churchPow a b, "POW", 2)
    _  -> (0, "UNKNOWN", 0)
resolveChurchOp _ _ _ a _ = (a, "UNKNOWN", 1)

findMethodSlot :: CList -> String -> Maybe (Int, GoldenToken)
findMethodSlot cl method =
    let entries = Map.toList (clSlots cl)
        matches = filter (\(_, gt) -> matchMethod (gtName gt) method) entries
    in case matches of
        ((slot, gt):_) -> Just (slot, gt)
        [] -> Nothing

matchMethod :: String -> String -> Bool
matchMethod gtName_ method =
    let upperGT = map toUpper gtName_
        upperMethod = map toUpper method
    in upperGT == upperMethod
       || upperGT == ("SR_" ++ upperMethod)

parseMathExpr :: String -> Maybe (String, String, [String])
parseMathExpr input =
    let stripped = strip input
    in case parseMathFunc stripped of
        Just result -> Just result
        Nothing     -> parseMathInfix stripped

parseMathFunc :: String -> Maybe (String, String, [String])
parseMathFunc s =
    let lower = map toLower (takeWhile (\c -> c /= '(' && c /= ' ') s)
        rest = dropWhile (\c -> c /= '(') s
    in if null rest || null lower then Nothing
       else if last rest /= ')' then Nothing
       else let inside = drop 1 (init rest)
                args = map strip (splitOn ',' inside)
            in if any null args then Nothing
               else case lower of
                "sqrt"  -> Just ("SlideRule", "SQRT", args)
                "log"   -> Just ("SlideRule", "LOG", args)
                "exp"   -> Just ("SlideRule", "EXP", args)
                "pow"   -> Just ("Lambda", "POW", args)
                "succ"  -> Just ("Lambda", "SUCC", args)
                "pred"  -> Just ("Lambda", "PRED", args)
                "mod"   -> Just ("SlideRule", "MOD", args)
                _       -> Nothing

parseMathInfix :: String -> Maybe (String, String, [String])
parseMathInfix s =
    case findInfixOp s of
        Just (left, op, right) ->
            let l = strip left
                r = strip right
            in if null l || null r then Nothing
               else case op of
                   '+' -> Just ("SlideRule", "ADD", [l, r])
                   '-' -> Just ("SlideRule", "SUB", [l, r])
                   '*' -> Just ("Lambda", "MUL", [l, r])
                   '/' -> Just ("SlideRule", "DIV", [l, r])
                   '%' -> Just ("SlideRule", "MOD", [l, r])
                   '^' -> Just ("Lambda", "POW", [l, r])
                   _   -> Nothing
        Nothing -> Nothing

findInfixOp :: String -> Maybe (String, Char, String)
findInfixOp s = tryOps ['+', '-', '*', '/', '%', '^']
  where
    tryOps [] = Nothing
    tryOps (op:ops) =
        case splitInfix op s of
            Just (l, r) | not (null (strip l)) && not (null (strip r)) -> Just (l, op, r)
            _ -> tryOps ops
    splitInfix op str =
        case break (== op) str of
            (_, [])    -> Nothing
            (left, _:right) -> Just (left, right)

isMathExpr :: String -> Bool
isMathExpr s = case parseMathExpr (strip s) of
    Just _  -> True
    Nothing -> False

toLower :: Char -> Char
toLower c
    | c >= 'A' && c <= 'Z' = toEnum (fromEnum c + 32)
    | otherwise = c

parseCallSyntax :: String -> Maybe (String, String, [String])
parseCallSyntax input = do
    let stripped = dropWhile (== ' ') input
    let upper = map toUpper stripped
    if not (isPrefixOf' "CALL(" upper)
        then Nothing
        else do
            let inside = drop 5 stripped
            let inside' = if not (null inside) && last inside == ')'
                          then init inside
                          else inside
            let parts = splitOn ',' inside'
            case parts of
                [] -> Nothing
                (target:argStrs) -> do
                    let trimmed = strip target
                    case splitOn '.' trimmed of
                        [absN, meth] -> Just (strip absN, strip meth, map strip argStrs)
                        [absN]       -> Just (strip absN, "ACCESS", map strip argStrs)
                        _            -> Nothing

splitOn :: Char -> String -> [String]
splitOn _ [] = []
splitOn c s = let (w, rest) = break (== c) s
              in w : case rest of
                        []     -> []
                        (_:rs) -> splitOn c rs

strip :: String -> String
strip = reverse . dropWhile (== ' ') . reverse . dropWhile (== ' ')

readInteger :: String -> Maybe Integer
readInteger s = case reads (strip s) of
    [(n, "")] -> Just n
    _         -> Nothing

myMapMaybe :: (a -> Maybe b) -> [a] -> [b]
myMapMaybe _ [] = []
myMapMaybe f (x:xs) = case f x of
    Just y  -> y : myMapMaybe f xs
    Nothing -> myMapMaybe f xs

doLoad :: MachineState -> VarStore -> [String] -> [String] -> IO ()
doLoad ms vars upperRest origTokens = do
    let (crStr, name) = case upperRest of
            ("CR":cr:_) -> (cr, unwords (drop 3 origTokens))
            (cr:_)      -> (cr, unwords (drop 2 origTokens))
            _              -> ("", "")
    case readInteger crStr of
        Nothing -> putStrLn "[ERROR] Invalid register number" >> loop ms vars
        Just cr -> do
            let ms0 = ms { msTrace = [] }
            result <- churchLOAD ms0 (fromIntegral cr) (strip name)
            case result of
                Left fault -> do
                    putStrLn $ "  [FAULT] " ++ show fault
                    loop ms vars
                Right ms' -> do
                    showTrace ms'
                    putStrLn $ "  [OK] CR" ++ show cr ++ " <- " ++ strip name
                    loop ms' vars

doTperm :: MachineState -> VarStore -> [String] -> IO ()
doTperm ms vars rest = do
    let (crStr, permStrs) = case rest of
            ("CR":cr:ps) -> (cr, ps)
            (cr:ps)      -> (cr, ps)
            _            -> ("", [])
    case readInteger crStr of
        Nothing -> putStrLn "[ERROR] Invalid register number" >> loop ms vars
        Just cr -> do
            let perms = concatMap parsePerm permStrs
            let ms0 = ms { msTrace = [] }
            result <- churchTPERM ms0 (fromIntegral cr) perms
            case result of
                Left fault -> do
                    putStrLn $ "  [FAULT] " ++ show fault
                    loop ms vars
                Right ms' -> do
                    showTrace ms'
                    putStrLn "  [OK] Permission check passed"
                    loop ms' vars

parsePerm :: String -> [Permission]
parsePerm s = concatMap charToPerm (map toUpper s)
    where
        charToPerm 'R' = [R]
        charToPerm 'W' = [W]
        charToPerm 'X' = [X]
        charToPerm 'L' = [L]
        charToPerm 'S' = [S]
        charToPerm 'E' = [E]
        charToPerm _   = []

doCallRaw :: MachineState -> VarStore -> [String] -> IO ()
doCallRaw ms vars rest = do
    let crStr = case rest of
            ("CR":cr:_) -> cr
            (cr:_)      -> cr
            _           -> ""
    case readInteger crStr of
        Nothing -> putStrLn "[ERROR] Invalid register number" >> loop ms vars
        Just cr -> do
            let ms0 = ms { msTrace = [] }
            result <- churchCALL ms0 (fromIntegral cr)
            case result of
                Left fault -> do
                    putStrLn $ "  [FAULT] " ++ show fault
                    loop ms vars
                Right ms' -> do
                    showTrace ms'
                    putStrLn "  [OK] Entered abstraction scope"
                    loop ms' vars

doReturn :: MachineState -> VarStore -> IO ()
doReturn ms vars = do
    let ms0 = ms { msTrace = [] }
    result <- churchRETURN ms0
    case result of
        Left fault -> do
            putStrLn $ "  [FAULT] " ++ show fault
            loop ms vars
        Right ms' -> do
            showTrace ms'
            putStrLn "  [OK] Returned"
            loop ms' vars

showNamespace :: MachineState -> IO ()
showNamespace ms = do
    putStrLn ""
    putStrLn "+=================== NAMESPACE (Golden Tokens) ===================+"
    putStrLn "| NAME              | PERMS  | INDEX | VER | DOMAIN  |"
    putStrLn "+-------------------+--------+-------+-----+---------+"
    let entries = Map.toList (nsEntries (msNamespace ms))
    mapM_ showNSEntry entries
    putStrLn "+-------------------+--------+-------+-----+---------+"
    putStrLn ""

showNSEntry :: (String, GoldenToken) -> IO ()
showNSEntry (name, gt) = do
    let nameStr = take 17 (name ++ replicate 17 ' ')
    let permStr = concatMap (\p -> if p `elem` gtPerms gt then show p else "-") [L, S, E]
    let idxStr  = take 5 (show (gtIndex gt) ++ replicate 5 ' ')
    let verStr  = take 3 (show (gtVersion gt) ++ replicate 3 ' ')
    putStrLn $ "| " ++ nameStr ++ " | " ++ permStr ++ "    | " ++ idxStr ++ " | " ++ verStr ++ " | Church  |"

showRegisters :: MachineState -> IO ()
showRegisters ms = do
    putStrLn ""
    putStrLn "+===================== CONTEXT REGISTERS =====================+"
    let crs = Map.toList (msContextRegs ms)
    if null crs
        then putStrLn "|  (all empty)                                               |"
        else mapM_ (\(i, gt) -> putStrLn $ "|  CR" ++ show i ++ ": " ++ gtName gt ++ " " ++ show (gtPerms gt)) crs
    putStrLn "+==================== DATA REGISTERS =========================+"
    let drs = Map.toList (msDataRegs ms)
    mapM_ (\(i, v) -> if v /= 0 then putStrLn $ "|  DR" ++ show i ++ ": " ++ show v else return ()) drs
    putStrLn "+==============================================================+"
    putStrLn ""

showCList :: MachineState -> String -> IO ()
showCList ms name = do
    case Map.lookup name (msAbstractions ms) of
        Nothing -> putStrLn $ "[ERROR] Abstraction not found: " ++ name
        Just abs_ -> do
            putStrLn ""
            putStrLn $ "+=============== C-LIST: " ++ absName abs_ ++ " ===============+"
            putStrLn "| SLOT | NAME              | PERMS  |"
            putStrLn "+------+-------------------+--------+"
            let entries = Map.toList (clSlots (absCList abs_))
            mapM_ showCLEntry entries
            putStrLn "+------+-------------------+--------+"
            putStrLn ""

showCLEntry :: (Int, GoldenToken) -> IO ()
showCLEntry (slot, gt) = do
    let slotStr = take 4 (show slot ++ replicate 4 ' ')
    let nameStr = take 17 (gtName gt ++ replicate 17 ' ')
    let permStr = concatMap (\p -> if p `elem` gtPerms gt then show p else "-") [L, S, E, X]
    putStrLn $ "| " ++ slotStr ++ " | " ++ nameStr ++ " | " ++ permStr ++ "   |"
