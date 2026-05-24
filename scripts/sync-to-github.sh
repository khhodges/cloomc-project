#!/bin/bash
# sync-to-github.sh — Push current HEAD to khhodges/cloomc-project on GitHub.
# Called automatically by scripts/post-merge.sh after every Replit task merge.
# Requires the GITHUB_PAT secret to be set in Replit Secrets (no expiry, repo scope).

set -euo pipefail

REPO="khhodges/cloomc-project"
REMOTE_NAME="github-sync"
REMOTE_URL="https://x-access-token:${GITHUB_PAT}@github.com/${REPO}.git"

if [ -z "${GITHUB_PAT:-}" ]; then
    echo "sync-to-github: GITHUB_PAT secret is not set — skipping GitHub sync."
    echo "  Set a classic GitHub PAT with 'repo' scope and no expiry in Replit Secrets."
    exit 0
fi

# Add or update the github-sync remote (safe to re-run)
if git remote get-url "$REMOTE_NAME" &>/dev/null; then
    git remote set-url "$REMOTE_NAME" "$REMOTE_URL"
else
    git remote add "$REMOTE_NAME" "$REMOTE_URL"
fi

# Disable LFS for this remote so we only push regular git objects.
# LFS binaries are large; code sync doesn't need them.
git config "remote.${REMOTE_NAME}.lfsurl" "https://github.com/${REPO}.git/info/lfs" 2>/dev/null || true
git config "lfs.${REMOTE_URL}/info/lfs.locksverify" "false" 2>/dev/null || true

BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "main")
HEAD_SHA=$(git rev-parse --short HEAD 2>/dev/null || echo "unknown")

echo "sync-to-github: pushing ${BRANCH} (${HEAD_SHA}) → github.com/${REPO} ..."

# Push with LFS skipped — env var + pre-push hook bypass
GIT_LFS_SKIP_PUSH=1 \
GIT_TRACE=0 \
    git -c "lfs.${REMOTE_URL}.locksverify=false" \
        push "$REMOTE_NAME" "HEAD:refs/heads/${BRANCH}" --force 2>&1

echo "sync-to-github: push succeeded."
