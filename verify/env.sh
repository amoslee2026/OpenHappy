#!/bin/bash
# ===================================================================
# env.sh - D2D HiPi (OpenHappy) verification environment variables (bash)
#
# Version: v2.2 | Date: 2026-10-31
#
# READ THIS FIRST
#   This is the bash environment setup script.
#   Each server / account may keep the checkout in a different place. If so,
#   point PROJ_HOME below at the absolute path that holds this repository, then
#   source this file.
#
# Usage (bash / zsh):
#   source <repo-root>/verify/env.sh
#
#     or from ~/.bashrc:
#   source /path/to/OpenHappy/verify/env.sh
#
# Notes:
#   - This file only sets environment variables; it never changes the working
#     directory, so it can be sourced from anywhere.
#   - By default the repository root is derived from this script's location
#     (the parent of verify/ = <repo-root>), which is correct for almost every
#     checkout. Set PROJ_HOME manually only when the tree is moved/copied, or
#     when mount points differ.
#
# Repository layout:
#   <repo-root>/
#     |- code/     RTL (design/ + common/ + hardip/)
#     |- verify/   verification environment (TB / VIP / env / seq / test / filelist / script)
#     |- sim/      simulation workspace and results
#     `- docs/     documentation
#
# Tooling: Verilator compiles/simulates (self-built under /project/tools/verilator-*),
#   GTKWave views waveforms. This script sets project variables only, not tool paths.
# ===================================================================

# ---- Repository root: honour an explicit PROJ_HOME, otherwise derive it ----
# Manual mode: uncomment the next line and put your absolute path there:
# export PROJ_HOME=/absolute/path/to/OpenHappy

if [ -z "$PROJ_HOME" ]; then
  # Derived: this script lives in <repo-root>/verify/, so its parent is the root
  PROJ_HOME="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  export PROJ_HOME
fi

# ---- Root of this verification environment ----
export D2D_HIPI_DV_HOME="$PROJ_HOME/verify"

# ---- RTL root (code/ holds design/ + common/ + hardip/) ----
export HIPINEXT_HOME="$PROJ_HOME/code"

# ---- Compatibility alias ----
export HIPI_NEXT_HOME="$HIPINEXT_HOME"

# ---- Simulation workspace ----
export D2D_SIM_HOME="$PROJ_HOME/sim"

# ---- Echo the result (optional, for confirmation) ----
echo "[env.sh] PROJ_HOME        = $PROJ_HOME"
echo "[env.sh] D2D_HIPI_DV_HOME = $D2D_HIPI_DV_HOME"
echo "[env.sh] HIPINEXT_HOME    = $HIPINEXT_HOME"
echo "[env.sh] HIPI_NEXT_HOME   = $HIPI_NEXT_HOME"
echo "[env.sh] D2D_SIM_HOME     = $D2D_SIM_HOME"