# ===================================================================
# D2D HiPi loopback verification Makefile (OpenHappy)
#   Simulator: Verilator (open source); waveform: VCD + GTKWave
#
# Version: v3.1 | Date: 2026-10-31
#
# NOTES
#   Tool chain (all open source):
#     - compiler/simulator: Verilator (/project/tools/verilator-*/bin/verilator)
#     - waveform format    : VCD (tb top $dumpfile/$dumpvars, compiled with --trace-vcd)
#     - waveform viewer    : GTKWave (reads VCD)
#   Key points:
#     - Nested filelists MUST be pulled in with UPPER-CASE -F so that Verilator
#       expands them recursively and resolves paths that reference environment
#       variables ($HIPINEXT_HOME / $D2D_HIPI_DV_HOME ...).
#     - Environment variables are set by the bash script env.sh (see its
#       "read this first" block).
#
# REPOSITORY LAYOUT
#   <repo-root>/
#     |- code/     RTL (design/ + common/ + hardip/)
#     |- verify/   verification environment (this directory: TB / VIP / env / seq / test / filelist / script)
#     |- sim/      simulation workspace and results (compile.log / run.log / VCD / obj_dir)
#     `- docs/     documentation
#
# USAGE
#   source <repo-root>/verify/env.sh       # set environment variables (optional: this Makefile derives them)
#   make compile TEST=smoke_1wr1rd         # compile only (Verilator --binary)
#   make run     TEST=smoke_1wr1rd         # compile + run
#   make smoke_1wr1rd                      # equivalent to run
#   make waves   TEST=smoke_1wr1rd         # open the VCD in GTKWave
#   make clean
#
# MAIN VARIABLES
#   VERILATOR    Verilator executable (auto-detected by default)
#   TEST         test name (default smoke_1wr1rd)
#   SEED         random seed (+test_seed=; Verilator does not expand random constraints, echo/extension only)
#   TIMEOUT      simulation timeout in ns (default 2_000_000 = 2 ms)
#   DUMP         1 = dump VCD; 0 = faster (default 0)
#   LOOPBACK     loopback level: phys / paif / cpif (default phys)
#   TRAIN_BYPASS 1 = skip PHY training (TRAIN_MODE=3, link up at power-on)
#
# Note: the RTL root HIPINEXT_HOME is derived from this directory (../code) and can be
#       overridden by the environment variable. All simulation output lands in
#       <repo-root>/sim/<TEST>/; nothing temporary is written under verify/.
# ===================================================================

DV_HOME := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))

# Run recipe shells with bash (they use bash-only syntax such as set -o pipefail)
SHELL := /bin/bash
export DV_HOME
export D2D_HIPI_DV_HOME := $(DV_HOME)

# Repository root: <repo-root>/verify -> <repo-root>
PROJ_HOME := $(abspath $(DV_HOME)/..)
export PROJ_HOME

# RTL root: <repo-root>/code (self-contained)
HIPI_NEXT_HOME ?= $(PROJ_HOME)/code
export HIPI_NEXT_HOME
export HIPINEXT_HOME := $(HIPI_NEXT_HOME)

# Simulation workspace: <repo-root>/sim
D2D_SIM_HOME ?= $(PROJ_HOME)/sim
export D2D_SIM_HOME

# ===================================================================
# EDA tool: Verilator (version available on this server)
#   Probe order: environment variable VERILATOR -> /project/tools/verilator/bin/verilator
#                -> any verilator on PATH. Fails if none is found.
# ===================================================================
VERILATOR ?= $(firstword $(shell \
    if command -v verilator >/dev/null 2>&1; then echo verilator; \
    elif [ -x /project/tools/verilator/bin/verilator ]; then echo /project/tools/verilator/bin/verilator; \
    else echo ""; fi))

ifeq ($(strip $(VERILATOR)),)
$(error Verilator not found. Set VERILATOR=<path> or put it on PATH (this server: /project/tools/verilator/bin/verilator))
endif

TEST      ?= smoke_1wr1rd
SEED      ?= 1
TIMEOUT   ?= 2000000
# DUMP: 1 = dump VCD for GTKWave; 0 = no dump (default: faster, no disk pressure).
#   A full-hierarchy Verilator dump is large (GB scale); enable it only when waveforms are needed.
DUMP      ?= 0
# Loopback level: phys (port-level physical loopback, default) / paif (protocol-layer PAIF loopback)
#                 / cpif (link-layer CPIF loopback)
LOOPBACK  ?= phys
VALID_LOOPBACKS := phys paif cpif

# Parse-time check: reject unsupported LOOPBACK values (the axi level needs two dies facing each
# other via prcl 0x500 bit1 remote_loop and is not covered by this TB)
ifeq ($(filter $(LOOPBACK),$(VALID_LOOPBACKS)),)
$(error Unsupported LOOPBACK=$(LOOPBACK); supported: $(VALID_LOOPBACKS) (axi = prcl 0x500 bit1 remote_loop, needs a two-die TB, not included here))
endif

# TRAIN_BYPASS: 1 = skip PHY training (logic_phy 0x0000 TRAIN_MODE=3, link up at power-on);
#               0 = full automatic training flow
TRAIN_BYPASS ?= 1

# Default TRAIN_BYPASS=1 because the RX training FSM is a three-state shell stub
# (see docs/UserGuide.md 3.5.1): TRAIN_MODE=0 stalls in training under phys loopback, the link
# never comes up and the case fails. Defaulting to 1 keeps a bare `make run` passing.
# XINIT / XASGN: 1 = --x-initial unique / --x-assign unique (approximate 4-state random X->0/1,
#   closer to this RTL, whose async FIFOs depend on reset values); 0 = fast (X->0, faster).
XINIT ?= 1
XASGN ?= 1

TEST_UPPER = $(shell echo $(TEST) | tr a-z A-Z)

SIM_DIR   := $(D2D_SIM_HOME)/$(TEST)
OBJ_DIR   := $(SIM_DIR)/obj_dir
TOP       := d2d_hipi_tb_top
SIMV      := $(OBJ_DIR)/V$(TOP)
COMP_LOG  := $(SIM_DIR)/compile.log
VERI_OUT  := $(SIM_DIR)/verilator.out   # Verilator compile stdout/stderr
RUN_LOG   := $(SIM_DIR)/run.log
SIM_OUT   := $(SIM_DIR)/sim.out
WAVE      := $(SIM_DIR)/$(TEST).vcd     # GTKWave waveform (VCD)

# ------------------------------------------------------------------
# Verilator compile/generate options
#   --binary  : build an executable simulator directly (auto main, runs initial/$display/$finish)
#   --timing  : support #delays and @ event waits (this TB relies on timing heavily)
#   -F        : read the top filelist and expand nested filelists, resolving env vars (essential)
#   -Wno-*    : suppress warning noise (this RTL/TB is not lint-clean by construction)
#   -D        : inject compile-time macros
# ------------------------------------------------------------------
VERI_FLAGS = --binary --timing \
             --top-module $(TOP) \
             --Mdir $(OBJ_DIR) \
             --prefix V$(TOP) \
             $(if $(filter 1,$(DUMP)),--trace-vcd,) \
             -F $(DV_HOME)/filelist/d2d_hipi_tb.f \
             -DTB_TEST_$(TEST_UPPER) \
             -DTIME_SCALE_UNIT=1ns \
             -DTIME_SCALE_PREC=1ps \
             $(if $(filter 1,$(TRAIN_BYPASS)),-DD2D_TB_BYPASS_TRAIN,) \
             $(if $(filter 1,$(DUMP)),-DDUMP_VCD,) \
             -Wno-fatal \
             -Wno-WIDTH -Wno-UNOPTFLAT -Wno-LATCH -Wno-MULTIDRIVEN \
             -Wno-CASEINCOMPLETE -Wno-CASEX -Wno-CASEWITHX \
             -Wno-CMPCONST -Wno-SELRANGE -Wno-UNUSEDSIGNAL \
             -Wno-UNUSEDPARAM -Wno-PINMISSING -Wno-PINCONNECTEMPTY \
             -Wno-IMPLICIT -Wno-UNSIGNED -Wno-VARHIDDEN \
             -Wno-TIMESCALEMOD -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
             -Wno-UNDRIVEN -Wno-UNOPTFLAT -Wno-SYNCASYNCNET \
             -Wno-UNUSED \
             --x-assign $(if $(filter 1,$(XASGN)),unique,fast) \
             --x-initial $(if $(filter 1,$(XINIT)),unique,fast)

# Runtime plusargs (+name=value)
RUN_FLAGS = +test_seed=$(SEED) +sim_timeout=$(TIMEOUT) +LOOPBACK_MODE=$(LOOPBACK) \
            +TRAIN_MODE=$(if $(filter 1,$(TRAIN_BYPASS)),3,0)
ifeq ($(DUMP),1)
RUN_FLAGS += +vcd_name=$(TEST).vcd
endif

# Registered cases (append here when adding a case)
TESTS := smoke_1wr1rd

.PHONY: all compile run regress clean waves logs help $(TESTS)

all: run

help:
	@echo "D2D HiPi (OpenHappy) DV Makefile  (simulator = Verilator)"
	@echo "  repo root: $(PROJ_HOME)"
	@echo "  RTL root : $(HIPINEXT_HOME)"
	@echo "  sim area : $(D2D_SIM_HOME)"
	@echo "  Verilator: $(VERILATOR)"
	@echo
	@echo "  make run TEST=<case>      compile + run"
	@echo "  make compile TEST=<case>  compile only"
	@echo "  make waves TEST=<case>    open the VCD in GTKWave"
	@echo "  LOOPBACK=phys|paif|cpif  loopback level (default phys; cpif = link-layer CPIF loopback)"
	@echo "  TRAIN_BYPASS=1           skip PHY training (TRAIN_MODE=3, up at power-on; default 1 here)"
	@echo "  DUMP=0                   no VCD (faster)"
	@echo "  make logs TEST=<case>     print the log directory"
	@echo "  make regress              full regression (case x {$(VALID_LOOPBACKS)})"
	@echo "  cases: $(TESTS)"
	@echo "  env vars: source <repo-root>/verify/env.sh first, or rely on this Makefile's derivation"

# ------------------------------------------------------------------
# Compile: Verilator --binary (produces sim/<TEST>/obj_dir/V<TOP>)
#   Live in the terminal (tee verilator.out); the log is also copied to compile.log
# ------------------------------------------------------------------
compile:
	@mkdir -p $(SIM_DIR)
	@echo "=============================================================="
	@echo "[MAKE] compile TEST=$(TEST)  (Verilator)"
	@echo "[MAKE]   RTL root : $(HIPINEXT_HOME)"
	@echo "[MAKE]   sim area : $(SIM_DIR)"
	@echo "[MAKE]   filelist : $(DV_HOME)/filelist/d2d_hipi_tb.f  (-F recursive)"
	@echo "=============================================================="
	@rm -rf $(OBJ_DIR)
	@set -o pipefail; \
	 cd $(SIM_DIR) && $(VERILATOR) $(VERI_FLAGS) </dev/null 2>&1 | tee $(VERI_OUT); \
	 rc=$$?; \
	 cp -f $(VERI_OUT) $(COMP_LOG) 2>/dev/null || true; \
	 if [ $$rc -ne 0 ] || [ ! -x $(SIMV) ]; then \
	   echo "=============================================================="; \
	   echo "[MAKE] COMPILE_FAIL (rc=$$rc)"; \
	   echo "[MAKE]   full log: $(COMP_LOG)"; \
	   echo "[MAKE]             $(VERI_OUT)"; \
	   echo "[MAKE]   error summary:"; \
	   grep -E "%Error|Error:" $(COMP_LOG) | tail -40 || true; \
	   echo "=============================================================="; \
	   exit 1; \
	 fi; \
	 echo "=============================================================="; \
	 echo "[MAKE] compile OK -> $(SIMV)"; \
	 echo "[MAKE]   Error/Fatal: $$(grep -cE '%Error' $(COMP_LOG) || true)"; \
	 echo "=============================================================="

# ------------------------------------------------------------------
# Run: execute the Verilator-built simulator
#   Run from the sim directory so the VCD lands in SIM_DIR
# ------------------------------------------------------------------
run: compile
	@echo "=============================================================="
	@echo "[MAKE] run TEST=$(TEST) SEED=$(SEED) TIMEOUT=$(TIMEOUT)ns LOOPBACK=$(LOOPBACK)"
	@echo "[MAKE]   log     : $(SIM_OUT)"
	@echo "=============================================================="
	@set -o pipefail; \
	 cd $(SIM_DIR) && timeout 3600 ./obj_dir/V$(TOP) $(RUN_FLAGS) </dev/null 2>&1 | tee $(SIM_OUT); \
	 rc=$$?; \
	 cp -f $(SIM_OUT) $(RUN_LOG) 2>/dev/null || true; \
	 echo "=============================================================="; \
	 echo "[MAKE] run rc=$$rc"; \
	 echo "[MAKE] stdout: $(SIM_OUT)"; \
	 echo "[MAKE] runlog: $(RUN_LOG)"; \
	 echo "--------- result summary ---------"; \
	 grep -E "TEST RESULT|BASE].*PASS|BASE].*FAIL|SCB.*FAIL|TIMEOUT|ERROR" $(SIM_OUT) | tail -40 || true; \
	 echo "=============================================================="; \
	 exit $$rc

regress:
	@for t in $(TESTS); do \
	  for lb in $(VALID_LOOPBACKS); do \
	    echo "============ REGRESS $$t LOOPBACK=$$lb ============"; \
	    $(MAKE) run TEST=$$t LOOPBACK=$$lb || echo "CASE FAIL: $$t/$$lb"; \
	  done; \
	done
	@echo "==================== REGRESS DONE ===================="

logs:
	@echo "=============================================================="
	@echo " repo root : $(PROJ_HOME)"
	@echo " sim area  : $(D2D_SIM_HOME)"
	@echo " log dir   : $(SIM_DIR)"
	@echo "=============================================================="
	@ls -la $(SIM_DIR)/compile.log $(SIM_DIR)/verilator.out $(SIM_DIR)/run.log $(SIM_DIR)/sim.out 2>/dev/null || true
	@echo
	@echo "  compile.log  : Verilator compile log"
	@echo "  verilator.out: compile stdout+stderr"
	@echo "  run.log      : simulation log"
	@echo "  sim.out      : simulation stdout+stderr"
	@echo "  <TEST>.vcd   : waveform (open with GTKWave)"

# ------------------------------------------------------------------
# Waveform: GTKWave (reads VCD)
#   script/open_wave.sh is the preferred entry; this is the equivalent built-in
# ------------------------------------------------------------------
waves:
	@test -f $(SIM_DIR)/$(TEST).vcd || \
	  (echo "no VCD: $(SIM_DIR)/$(TEST).vcd (run make run TEST=$(TEST) DUMP=1 first)"; exit 1)
	@echo "[WAVES] gtkwave opening $(SIM_DIR)/$(TEST).vcd"
	@gtkwave $(SIM_DIR)/$(TEST).vcd &

clean:
	rm -rf $(SIM_DIR) 2>/dev/null || true
	rm -rf *.daidir obj_dir ucli.key novas.* verdiLog 2>/dev/null || true
	@echo "[MAKE] clean done ($(SIM_DIR))"

cleanall:
	rm -rf $(D2D_SIM_HOME)/* 2>/dev/null || true
	rm -rf *.daidir obj_dir ucli.key novas.* verdiLog 2>/dev/null || true
	@echo "[MAKE] cleanall done ($(D2D_SIM_HOME))"

# Each case is directly usable as a make target
$(TESTS):
	@$(MAKE) run TEST=$@