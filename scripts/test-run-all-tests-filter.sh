#!/usr/bin/env bash
# test-run-all-tests-filter.sh — unit-tests for the --group + suite-name
# filtering/deduplication logic in run-all-tests.sh.
#
# Exercises the argument-parsing and suite-selection paths in isolation
# (no real suites are launched).
#
# Usage:
#   bash scripts/test-run-all-tests-filter.sh
#
# Exits 0 when all assertions pass, non-zero otherwise.

set -uo pipefail

PASS=0
FAIL=0

_assert_eq() {
    local label="$1"
    local expected="$2"
    local actual="$3"
    if [ "$expected" = "$actual" ]; then
        echo "  PASS: $label"
        PASS=$((PASS + 1))
    else
        echo "  FAIL: $label"
        echo "        expected: $expected"
        echo "        actual:   $actual"
        FAIL=$((FAIL + 1))
    fi
}

# ---------------------------------------------------------------------------
# Minimal replica of the run-all-tests.sh filter machinery
# ---------------------------------------------------------------------------
# Enough suite names to exercise all paths; commands are no-ops.
_ALL_SUITE_NAMES=("alpha" "bravo" "charlie" "delta" "echo" "foxtrot")

declare -A _ALL_GROUPS
_ALL_GROUPS["first"]="alpha bravo"
_ALL_GROUPS["second"]="charlie delta"
_ALL_GROUPS["overlap"]="alpha charlie"

# Runs the filter logic given raw argv and sets _RESULT (space-separated).
_run_filter() {
    local _group=""
    local _explicit=()

    local _args=("$@")
    local _i=0
    while [ $_i -lt ${#_args[@]} ]; do
        local _arg="${_args[$_i]}"
        case "$_arg" in
            --group)
                _i=$((_i + 1))
                _group="${_args[$_i]}"
                ;;
            --group=*)
                _group="${_arg#--group=}"
                ;;
            *)
                _explicit+=("$_arg")
                ;;
        esac
        _i=$((_i + 1))
    done

    local _requested=("${_explicit[@]+"${_explicit[@]}"}")

    if [ -n "$_group" ]; then
        if [ -z "${_ALL_GROUPS[$_group]+set}" ]; then
            _RESULT="ERROR:unknown-group:$_group"
            return
        fi
        read -r -a _gs <<< "${_ALL_GROUPS[$_group]}"
        _requested+=("${_gs[@]}")
        unset _gs
    fi

    local _selected=()
    if [ "${#_requested[@]}" -eq 0 ]; then
        _selected=("${_ALL_SUITE_NAMES[@]}")
    else
        for _name in "${_ALL_SUITE_NAMES[@]}"; do
            for _req in "${_requested[@]}"; do
                if [ "$_req" = "$_name" ]; then
                    _selected+=("$_name")
                    break
                fi
            done
        done
    fi

    _RESULT="${_selected[*]+"${_selected[*]}"}"
}

# ---------------------------------------------------------------------------
echo "── Group-only mode ─────────────────────────────────────────────────"

_run_filter --group first
_assert_eq "--group first → alpha bravo" "alpha bravo" "$_RESULT"

_run_filter --group second
_assert_eq "--group second → charlie delta" "charlie delta" "$_RESULT"

# ---------------------------------------------------------------------------
echo "── Explicit-only mode ──────────────────────────────────────────────"

_run_filter echo foxtrot
_assert_eq "echo foxtrot → echo foxtrot" "echo foxtrot" "$_RESULT"

_run_filter alpha
_assert_eq "alpha → alpha" "alpha" "$_RESULT"

# ---------------------------------------------------------------------------
echo "── Combined: --group + extra suite ─────────────────────────────────"

_run_filter --group first echo
_assert_eq "--group first echo → alpha bravo echo" "alpha bravo echo" "$_RESULT"

_run_filter echo --group first
_assert_eq "echo --group first → alpha bravo echo" "alpha bravo echo" "$_RESULT"

_run_filter --group second foxtrot alpha
_assert_eq "--group second foxtrot alpha → alpha charlie delta foxtrot" \
           "alpha charlie delta foxtrot" "$_RESULT"

# ---------------------------------------------------------------------------
echo "── Deduplication: suite in group AND explicit ───────────────────────"

_run_filter --group first alpha
_assert_eq "--group first alpha (alpha also in group) → alpha bravo" \
           "alpha bravo" "$_RESULT"

_run_filter --group first alpha bravo
_assert_eq "--group first alpha bravo (both in group) → alpha bravo" \
           "alpha bravo" "$_RESULT"

_run_filter --group overlap alpha charlie
_assert_eq "--group overlap alpha charlie (full overlap) → alpha charlie" \
           "alpha charlie" "$_RESULT"

# ---------------------------------------------------------------------------
echo "── Declaration order preserved ─────────────────────────────────────"

_run_filter foxtrot alpha charlie
_assert_eq "foxtrot alpha charlie → declaration order alpha charlie foxtrot" \
           "alpha charlie foxtrot" "$_RESULT"

_run_filter --group second alpha
_assert_eq "--group second alpha → declaration order alpha charlie delta" \
           "alpha charlie delta" "$_RESULT"

# ---------------------------------------------------------------------------
echo "── No arguments → all suites ───────────────────────────────────────"

_run_filter
_assert_eq "no args → all suites" \
           "alpha bravo charlie delta echo foxtrot" "$_RESULT"

# ---------------------------------------------------------------------------
echo "── Unknown group name → error token ────────────────────────────────"

_run_filter --group nosuchgroup
_assert_eq "--group nosuchgroup → error token" \
           "ERROR:unknown-group:nosuchgroup" "$_RESULT"

# ---------------------------------------------------------------------------
echo ""
echo "Results: $PASS passed, $FAIL failed"

if [ "$FAIL" -gt 0 ]; then
    exit 1
fi
