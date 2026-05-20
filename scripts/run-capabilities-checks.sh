#!/usr/bin/env bash
set -e
node scripts/check-capabilities-blocks.js
python scripts/audit_clist.py --check
