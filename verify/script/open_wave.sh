#!/bin/bash
# ===================================================================
# open_wave.sh - open a D2D HiPi (OpenHappy) waveform in GTKWave (VCD)
#
# Version: v1.1 | Date: 2026-10-31
#
# Usage:
#   bash script/open_wave.sh             # default smoke_1wr1rd
#   bash script/open_wave.sh <TEST>
#
# Notes:
#   Waveforms are VCD files produced by Verilator and open directly in GTKWave.
#   Prerequisite: run `make run TEST=<TEST> DUMP=1` first to generate <TEST>.vcd.
# ===================================================================

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DV="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJ="$(cd "$DV/.." && pwd)"

TEST=${1:-smoke_1wr1rd}
SIM="$PROJ/sim"
VCD="$SIM/$TEST/$TEST.vcd"
GTK="$SIM/$TEST/$TEST.gtkw"

if [ ! -f "$VCD" ]; then
  echo "missing waveform: $VCD   (run make run TEST=$TEST DUMP=1 first)"
  exit 1
fi

if [ -f "$GTK" ]; then
  # A save file exists: load it so the key signal layout (credit gating / AXI / link FSM) comes up directly
  echo "[open_wave] GTKWave opening $VCD + save $GTK"
  gtkwave "$VCD" "$GTK"
else
  gtkwave "$VCD"
fi