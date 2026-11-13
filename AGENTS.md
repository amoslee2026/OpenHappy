---
title: OpenHappy Contributor & Agent Manual
type: composite
purpose: user-manual
audience: llm
direction: both
status: approved
version: 1.0.0
author: OpenHappy Team
section_meta: "@meta"
---

# AGENTS.md — OpenHappy contributor & agent manual

Operating manual for anyone (human or agent) changing this repository. Read it before editing RTL,
the verification environment, or the documentation.

## 1. What this repository is

OpenHappy is a Die-to-Die (D2D) chiplet interconnect controller: a digital subsystem implementing
the Chinese national standard **GB/T 46280—2025 "Specification for chiplet interconnection
interface"** (parts 1-4; part 5, 2.5D package, is out of scope).

- Top-level DUT: `d2d_hipi_dig_top` (protocol layer `axi4_symm_prcl` + link layer `link_layer` +
  logic PHY `logic_phy` + controller CSR).
- Shipped configuration: symmetric AXI4 1TX1RX, one physical lane (lane0, 17 IO x 64 bit),
  PAIF 512 bit, CPIF 1024 bit, AXI data 512 bit / ID 10 bit / address 64 bit.
- No analog PHY and no PDK models are included. `code/design/rtl_v/top/d2d_ana_top_stub.sv`
  (450 pins) and `phy_model_stub.sv` (242 pins) exist only so an analog-PHY integration can be
  elaborated; their bodies are empty and they are not part of the shipped TB filelist.
- Repository is **not** under version control at the time of writing; treat files as unrecoverable
  without a manual backup.

## 2. Layout

| Path | Role |
|---|---|
| `code/design/rtl_v/` | RTL (`top/`, `pcsr/`, `protocol_layer/`, `link_layer/`, `logic_phy/`) |
| `code/design/filelist/` | 8 filelists; `d2d_hipi_dig_top.f` is the RTL entry |
| `code/common/` | common library; `xl_std_cell/` holds the `generic_*` wrappers that replace PDK std cells |
| `code/hardip/sram/` | `sram_1rw_128b.v`, hand-written behavioral SRAM used by `replay_buffer` |
| `verify/` | TB, VIPs, scoreboard, sequences, test cases, filelists, scripts |
| `verify/tb/d2d_hipi_tb_top.sv` | top: clocks, interfaces, DUT instance, port-level loopback, VCD, timeout |
| `verify/test/` | test cases; `d2d_hipi_test_base.sv` drives reset + APB bring-up + checks |
| `sim/<TEST>/` | generated: `obj_dir/`, `compile.log`, `verilator.out`, `run.log`, `sim.out`, optional `.vcd` |
| `docs/` | authoritative documentation (English) |
| `docs/architecture/` | block-diagram sources (`d2d_hipi_arch.drawio`, `.svg`, `.png`, edit link) |
| `docs/assets/snapshot/` | reference waveform screenshots |
| `temp/` | scratch; symlink to `/tmp/happy` (deleted material goes to `/tmp/happy/deleted/`) |

## 3. Invariants — do not break

1. `code/design/rtl_v/top/hipi_top_defines.vh` is the single compile-configuration source. It
   decides the compiled shape (symmetric AXI4, single lane) and therefore every filelist
   expansion. Never edit it to "enable" an unshipped shape (asymmetric AXI, AXI5, CXS, extra lanes,
   MCU subsystem): that RTL is not in this tree and enabling the macro breaks the build.
2. `code/design/rtl_v/logic_phy/apb_csr_top_rf.v` is generated from the register spreadsheet
   (reg-gen). Never hand-edit; regenerate instead.
3. Nested filelists MUST be pulled in with upper-case `-F` so Verilator expands them and resolves
   `$HIPINEXT_HOME` / `$D2D_HIPI_DV_HOME`. Lower-case `-f` silently breaks these paths.
4. The TB has its own macro namespace (`verify/tb/d2d_hipi_tb_defines.vh`). Never reuse RTL macros
   on the TB side; `hipi_top_undefines.vh` must stay between the RTL block and the TB block.
5. No PDK / vendor models: std cells are `code/common/xl_std_cell/generic_*`, the SRAM is the
   hand-written `sram_1rw_128b.v`. Do not add vendor simulation models.
6. `d2d_hipi_dig_top` is the only DUT in the shipped TB; loopback is wired explicitly at `tb_top`
   (main-band `tx_lane0_dataN -> rx_lane0_dataN`, sideband `sb_txd -> sb_rxd`). Do not add a
   `LOOPBACK` macro dependency to the TB.
7. `TRAIN_BYPASS` defaults to 1 because the RX training FSM is a three-state shell stub: with
   `TRAIN_MODE=0` training stalls and the link never reaches `S_NORMAL`. Do not flip the default
   to 0 unless a working training FSM lands.

## 4. Build and run

```bash
source verify/env.sh                 # optional; the Makefile derives the same paths
cd verify
make run TEST=smoke_1wr1rd           # compile + simulate; expect "TEST RESULT: PASS"
make compile TEST=smoke_1wr1rd       # compile only
make regress                         # smoke_1wr1rd x {phys, paif, cpif}, TRAIN_BYPASS=1
make run TEST=smoke_1wr1rd DUMP=1    # adds sim/smoke_1wr1rd/smoke_1wr1rd.vcd
make waves TEST=smoke_1wr1rd         # GTKWave
make clean                           # removes sim/<TEST>
```

Tool requirements: Verilator >= 5.000 (`--binary` and `--timing` are mandatory; 4.x cannot build
this flow), GNU Make, a C++14 compiler. GTKWave is optional (waveform viewing only).

Measured on the current tree: Verilator 5.050 builds and runs `smoke_1wr1rd` in ~87 s wall clock
(compile + run), reporting `TEST RESULT: PASS  (checks=0, errors=0)`.

Icarus Verilog is **not** supported: the TB uses `constraint`/`inside`/`break` and class-based
`virtual interface` members, and the filelists rely on `-F` plus environment-variable expansion.

## 5. Debug workflow

1. Read `sim/<TEST>/compile.log` (or `verilator.out`) for `%Error` lines; the Makefile prints the
   last 40 on failure.
2. Read `sim/<TEST>/sim.out` for the phase trace: `[P0]`..`[P4]` APB bring-up, `[SCB]` scoreboard
   verdicts, `[BASE]` case verdict, then `TEST RESULT: PASS|FAIL`.
3. Re-run with `DUMP=1` and inspect `sim/<TEST>/<TEST>.vcd` in GTKWave; reference waves live in
   `docs/assets/snapshot/`.
4. Isolate layers with `LOOPBACK=paif` (protocol layer only, skips link + PHY) or
   `LOOPBACK=cpif` (skips PHY). `LOOPBACK=axi` is rejected at parse time: it needs two dies.

Known non-fatal warning: `SBC_LANE_ENABLE` reads back `0xF` although `0x1` was written (hard-coded
in the generated register file). The TB downgrades it to a warning; it does not affect PASS.

## 6. Tests

- PASS requires `tb_error_cnt == 0`: APB write/read-back match during bring-up, PHY/link FSMs
  reaching ACTIVE / `S_NORMAL`, scoreboard agreement for forwarded requests, input requests and
  responses, and a clean `wait_all_done`.
- Adding a case: create `verify/test/d2d_hipi_test_<name>.sv` with the same construction as
  `smoke_1wr1rd`, register the name in `TESTS` in `verify/Makefile`, and include it in
  `filelist/d2d_hipi_tb.f` before `d2d_hipi_tb_top.sv`.
- A change is not verified until `make run TEST=smoke_1wr1rd` passes; interface, register or
  loopback changes also require `make regress`.

## 7. Documentation rules

- All documentation is **English**: `README.md`, `AGENTS.md`, everything under `docs/`.
- Literal tool output that is Chinese (TB `$display` strings, log keywords) MUST be quoted
  verbatim in docs, because users grep those strings; surrounding explanation is English.
- `docs/` is authoritative. Changing interface, register, command or behaviour claims requires
  updating the affected document in the same change.
- Documents under `docs/` carry YAML frontmatter (`title`, `type`, `purpose`, `audience`,
  `direction`, `status`, `section_meta`).
- Block diagrams: source in `docs/architecture/` (drawio + exported svg/png); screenshots in
  `docs/assets/snapshot/` with the `<case>_<object>_<purpose>.png` naming convention.

## 8. Change hygiene

- `sim/` is generated and git-ignored; never hand-edit or commit build artifacts, VCDs or logs.
- Never delete files: move them to `temp/deleted/<date>-<topic>/` (`temp/` resolves to `/tmp/happy`).
- Keep scope: this tree ships one shape (symmetric AXI4, single lane). New shapes need their own
  filelists and macros, not local edits to the shipped defines.
- RTL and TB comments are inherited Chinese; new documentation is English. Do not mass-translate
  source comments as part of an unrelated change.
- License: **PolyForm Free Trial 1.0.0** — `LICENSE` is one document in two parts. Part I must stay
  the verbatim upstream license text (no edits, no re-title, no reorder; only its Part-I heading and
  the Part-II Additional Terms may change); Part II holds the OpenHappy Additional Terms. `NOTICE`
  carries the copyright line, the commercial-license path and the third-party/tooling statements.
  Source-available, **not** open source: free use covers evaluation for less than 32 consecutive
  calendar days; distribution is not granted; commercial or production use needs a separate written
  license from OpenHappy Team. Never describe the project as "open source" in docs or comments (say
  "source-available"). Do not edit or re-title the Part I text: its §Patent License grants the patent
  license to users and its §Patent Defense clause ends that grant if
  a user asserts patent infringement — both must survive any future licensing change.
- Part II of `LICENSE` is a condition of the grant and prevails over Part I on conflict. Its
  §1 limits the licensed material to the **source code** (`code/`, `verify/` plus filelists, macros,
  scripts, build files) and the **design documents** (`docs/` and the root documents); the standard,
  protocol/interface/register/pin definitions, ideas and methods, user designs and simulation/
  synthesis/fabrication outputs are outside the license. §2 withdraws the Changes-and-New-Works grant
  unless OpenHappy Team authorizes modification in writing, §3 permits path/tool configuration, and §4
  vests IP in authorized modifications and derivatives of the licensed material in OpenHappy Team.
  Keep `LICENSE` and `NOTICE` mutually consistent when anything licensing-related changes.
- Copyright hygiene: source files carry no internal company copyright/confidentiality headers and no
  vendor (PDK cell, SRAM macro, PHY) names; provenance notes say "厂商" / "vendor" instead of naming
  the vendor or part number. Exception: the generated register files (`logic_phy/apb_csr_top_rf.v`, `link_layer/link_layer_rf.v`,
  `link_layer/link_layer_apb_reg_sync.v`) carry a neutral "generated, do not edit" note; re-running
  reg-gen restores the generator's own header, so re-apply the neutralization after any regeneration
  and never hand-edit their generated body.
- Provenance: documentation states that the RTL was developed with **BabelPro**, the closed-source
  professional edition of the open-source **Babel** project. Keep the caveat that BabelPro is not
  distributed here and is not required to build, simulate or integrate the project.
- No synthesis, timing or power flow exists in this repository. Do not claim synthesis, timing
  closure or area/power numbers.
