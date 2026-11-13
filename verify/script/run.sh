#!/bin/bash
# =============================================================================
# run.sh - D2D HiPi (OpenHappy) compile/run wrapper
#
# Version: v3.1 | Date: 2026-10-31
#
# Notes:
#   This project runs on open-source Verilator directly (the Makefile auto-detects the
#   tool). This script only exports the project environment variables and calls make.
#
# Usage:
#   ./script/run.sh compile smoke_1wr1rd
#   ./script/run.sh run     smoke_1wr1rd               # defaults: LOOPBACK=phys, TRAIN_BYPASS=1
#   ./script/run.sh run     smoke_1wr1rd phys 1        # explicit LOOPBACK=phys TRAIN_BYPASS=1
#   ./script/run.sh regress                            # three-level regression (TRAIN_BYPASS=1)
#
# TRAIN_BYPASS defaults to 1 (training bypassed) so a bare invocation still passes;
# see the Makefile comments and docs/UserGuide.md 3.5.1 (the RX training FSM is a
# three-state shell stub).
# =============================================================================
set -u

# This script lives in <repo-root>/verify/script/: parent is verify/, grandparent is the root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DV="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJ="$(cd "$DV/.." && pwd)"

export PROJ_HOME="$PROJ"
export D2D_HIPI_DV_HOME="$DV"
export HIPI_NEXT_HOME="$PROJ/code"
export HIPINEXT_HOME="$HIPI_NEXT_HOME"
export D2D_SIM_HOME="$PROJ/sim"

ACTION=${1:-run}
CASE=${2:-smoke_1wr1rd}
LOOPBACK=${3:-phys}
TRAIN_BYPASS=${4:-1}   # default 1 (bypass): a bare invocation still passes; see the Makefile comments and docs/UserGuide.md 3.5.1

echo "[run] PROJ=$PROJ"
echo "[run] action=$ACTION case=$CASE loopback=$LOOPBACK train_bypass=$TRAIN_BYPASS"

cd "$DV" || exit 1
if [ "$ACTION" = "regress" ]; then
  make regress DUMP=1
else
  make "$ACTION" TEST="$CASE" DUMP=1 LOOPBACK="$LOOPBACK" TRAIN_BYPASS="$TRAIN_BYPASS"
fi