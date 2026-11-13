---
title: D2D HiPi Integration Guide
type: narrative
purpose: guide
audience: both
direction: output
status: approved
version: 1.0.0
author: OpenHappy Team
related_docs:
  - UserGuide.md
  - architecture/d2d_hipi_arch.drawio
section_meta: "@meta"
---

# D2D HiPi Integration Guide

How to put the OpenHappy Die-to-Die controller into an SoC: which ports to connect, how to clock
and reset it, how to bring the link up over APB, how to compile the RTL, and what to check before
signing off.

Scope of this document: integration of the digital subsystem `d2d_hipi_dig_top` as shipped in
`code/`. Verification of the standalone RTL (testbench, cases, PASS criteria, waveforms) is covered
in [UserGuide.md](./UserGuide.md).

## Quick navigation

| I need to... | Section |
|---|---|
| See what the delivered block contains and what I must supply | 2 |
| Understand the block in one picture | 3 |
| Connect clocks | 4.1 |
| Connect resets and get the release order right | 4.2 |
| Connect the configuration bus | 4.3 |
| Connect the AXI4 user interfaces | 4.4 |
| Connect the die-to-die lane and sideband pins | 4.5, 4.6 |
| Connect interrupts and status | 4.7 |
| Know the APB address map and the exact bring-up writes | 5 |
| Compile the RTL (filelists, macros, cell models) | 6 |
| Replace the empty analog-PHY shell with a real EPHY | 7 |
| Meet the physical lane requirements | 8 |
| Bring up and debug a new instance | 9 |
| Verify the integrated instance | 10 |
| Know what is not finished yet | 11 |
| Run a sign-off checklist | 12 |

## 1. Scope and assumptions

- Standard: **GB/T 46280—2025**, parts 1-4 (general, protocol layer, data link layer, 2D-package
  physical layer). Part 5 (2.5D package) is not implemented.
- No conformance assessment or certification against GB/T 46280 has been performed. This document
  describes the implemented subset, including project extensions that are not part of the standard.
- Shipped shape: **symmetric AXI4, 1 TX + 1 RX lane (lane0)**, PAIF 512 bit, CPIF 1024 bit,
  AXI4 data 512 bit / ID 10 bit / address 64 bit, 32:1 serialization, 17 data pins per direction.
- Digital only. The analog EPHY, package, and PCB/interposer are the integrator's responsibility
  (see section 7).
- **License**: source-available under **PolyForm Free Trial 1.0.0** (`LICENSE`, Part I), subject to
  the Additional Terms in `LICENSE` Part II. Their scope is limited to the **source code and design
  documents** in this repository: the standard, the protocol/interface/register/pin definitions it
  describes, your own designs and RTL, and any simulation, synthesis or fabrication outputs are
  outside the license.
  Evaluation is free for less than 32 consecutive calendar days; distribution is not granted;
  **modifying the RTL (including local bug fixes or RTL/testbench changes) requires prior written
  authorization**, and IP in authorized modifications or derivatives of the licensed material vests in
  OpenHappy Team. Any commercial or production integration needs a separate written license from
  OpenHappy Team.
- No synthesis, DFT, or timing-closure flow ships with this repository; constraints and
  implementation results are the integrator's to produce.

## 2. What you get, what you must supply

Delivered:

| Item | Location |
|---|---|
| RTL (Verilog-2001 style, `.v` / `.vh`) | `code/design/rtl_v/` |
| Filelists (compile entry and sub-filelists) | `code/design/filelist/` |
| Common library and PDK cell wrappers (`generic_*`) | `code/common/` |
| SRAM behavioral model | `code/hardip/sram/sram_1rw_128b.v` |
| Analog PHY port shells for elaboration | `code/design/rtl_v/top/d2d_ana_top_stub.sv`, `phy_model_stub.sv` |
| Self-checking simulation environment | `verify/` |

You must supply:

- APB3 master (or a bridge from your CPU subsystem) for the configuration space.
- AXI4 masters/slaves on the user-side interfaces, matching the widths in section 4.4.
- Clock sources: main-band TX clock, recovered RX clock, APB clock, AXI clock, sideband clocks
  (see 4.1).
- A real analog EPHY with the `HIPI_2D_D2D_PHY_TOP` port list (see 7), or a behavioral model of it
  for simulation.
- Clock/reset sequencing logic that respects the order in 4.2.

## 3. Integration at a glance

```text
        local SoC                                        peer die
  ┌───────────────────┐                          ┌────────────────────┐
  │ AXI4 master(s)    │                          │ AXI4 slave/master  │
  └─────────┬─────────┘                          └─────────┬──────────┘
            │ s0_axi (slave port of DUT)                     │ m0_axi
  ┌─────────▼───────────────────────────────┐      ┌────────▼───────────┐
  │ d2d_hipi_dig_top                        │      │ d2d_hipi_dig_top   │
  │  protocol layer -> link layer -> logic  │      │ (mirror instance)  │
  │  PHY   (APB CSR behind pclk)            │      │                    │
  └───┬───────────────┬─────────────────┬───┘      └───┬────────────┬───┘
      │ tx/rx lane0   │ sideband        │ APB          │ lane0      │ sideband
      │ 17 x 64 bit   │ sb_txd/rxd ...  │ 20-bit addr  │            │
  ┌───▼───────────────▼─────────────────▼───┐      ┌───▼────────────▼───┐
  │ analog EPHY (HIPI_2D_D2D_PHY_TOP)       │      │ analog EPHY        │
  └─────────────────────────────────────────┘      └────────────────────┘
```

The block is symmetric: `s0_axi` is a full 5-channel AXI4 **slave** (requests enter the link here),
`m0_axi` is a full 5-channel AXI4 **master** (requests arriving from the peer leave here). Both are
always present in the shipped single-lane configuration.

## 4. Port interface

`d2d_hipi_dig_top` has 400 ports in the shipped build (165 inputs, 218 outputs, 17 inouts),
counted after preprocessing with `hipi_top_defines.vh` (`PRCL_AXI` defined, single lane, no MCU
subsystem). Inactive `ifdef` branches (CXS protocol layer, extra lanes, MCU/AHB) add ports that do
not exist here; take the number from your own elaboration. The groups that matter for integration
are below; pin-level details are in `code/design/rtl_v/top/d2d_hipi_dig_top.v`.

### 4.1 Clocks

| Signal | Dir | Nominal | Role |
|---|---|---|---|
| `pclk` | in | 1 GHz | APB configuration clock; also feeds logic PHY APB and MCU clock (single domain) |
| `mb_clk` | in | 1 GHz | Main-band TX clock, supplied by the analog PHY |
| `phy_rx_clk0` | in | 1 GHz | lane0 RX recovered clock, supplied by the analog PHY |
| `aclk` | in | 1 GHz | AXI4 user-side clock (protocol layer, `PRCL_AXI` build) |
| `sb_tx_clk_in` | in | 800 MHz | Sideband TX clock source, supplied by the analog PHY |
| `sb_tx_clk` | **out** | 800 MHz | Sideband forwarded (gated) clock to the analog PHY; toggles only while a packet is active |
| `sb_rx_clk` | in | 800 MHz | Sideband RX clock recovered by the analog PHY |
| `pll_lock` | in | - | Analog PHY PLL lock indication |

`pclk`, `mb_clk`, `aclk` and `phy_rx_clk0` are asynchronous to each other; all domain crossings are
inside the RTL. `pll_lock` is the only clock-adjacent input that carries no functional data; drive it
to a defined level. `cxs_clk` / `cxs_rstn` are **not** ports of this build — they are declared only in
the non-`PRCL_AXI` (CXS) branch of the port list.

### 4.2 Resets and release order

All resets are asynchronous and active low.

| Signal | Domain | Notes |
|---|---|---|
| `presetn` | APB / configuration | must be released first |
| `mb_rst_n` | main-band datapath | released after `presetn` |
| `sb_rst_n` | sideband datapath | released together with `mb_rst_n` |
| `mcu_rst_n` | MCU / AHB configuration | synchronous to `pclk`; kept for the MCU path even though the MCU subsystem is not built |
| `aresetn` | AXI4 user side (protocol layer) | must be released **last**, after PHY and link are up |

Release order:

```text
presetn -> mb_rst_n / sb_rst_n / mcu_rst_n -> PHY boot and link training -> aresetn (last)
```

Releasing `aresetn` early breaks protocol-layer bring-up: the layer probes PHY/link state after
reset and will not recover without a full re-reset.

Note that the link-layer and CXS resets inside the controller are software-controlled through the
controller CSR (`pcsr`), while the logic-PHY resets above are hard top-level inputs. Firmware must
coordinate both sides (the sequence in section 5 does this).

### 4.3 APB configuration port

| Signal | Width | Notes |
|---|---|---|
| `hipi_apb_paddr` | 20 | bit[16] selects the CSR domain: 0 = logic PHY, 1 = controller |
| `hipi_apb_pwrite` | 1 | write enable |
| `hipi_apb_psel` | 1 | select |
| `hipi_apb_penable` | 1 | transfer enable |
| `hipi_apb_pwdata` | 32 | write data |
| `hipi_apb_prdata` | 32 | read data |
| `hipi_apb_pready` | 1 | transfer complete |
| `hipi_apb_pslverr` | 1 | error response (unmapped address, write to read-only field) |
| `hipi_apb_pprot` | 3 | protection |

APB3, configured in the `pclk` domain. Read back every write during
bring-up: most configuration errors show up as a readback mismatch, not as a bus error.

Note: the header comment of `code/design/rtl_v/top/d2d_hipi_dig_top.v` (design note 3) still says
the top level has no external APB port and that configuration enters through the internal MCU path.
That note predates the exported APB port. The port table above is authoritative: the top level
exposes `hipi_apb_*` directly, with `hipi_apb_paddr[16]` selecting the logic-PHY or controller CSR
domain.

### 4.4 AXI4 user interfaces

Both interfaces are full AXI4 with 5 channels. Widths are identical on both:

| Field | Width |
|---|---|
| `id` | 10 bit |
| `addr` | 64 bit |
| `len` | 8 bit |
| `size` | 3 bit |
| `burst` | 2 bit |
| `cache` / `prot` / `qos` / `region` / `user` | 4 / 3 / 4 / 4 / 4 bit |
| `data` | 512 bit |
| `strb` | 64 bit |
| `resp` | 2 bit |

`s0_axi_*` (DUT is slave): AW/AR/W are inputs, B/R are outputs — connect your local masters here.
`m0_axi_*` (DUT is master): AW/AR/W are outputs, B/R are inputs — connect the local slaves that
must be reachable from the peer die. Both sides use `aclk`.

Integration rules:

- Do not park `s0_axi_awvalid`/`arvalid`/`wvalid` high while `aresetn` is low.
- Keep `awid`/`arid` within 10 bit; IDs are carried end-to-end, and responses may return out of
  order (one ID = one ordered stream).
- `awlen`/`arlen` are 8 bit, so a burst is 1-256 beats (`awlen` = 0-255); `size` encodes beats up to
  64 B (`0b110`).

### 4.5 Main-band lane interface (die-to-die pins)

| Group | Dir | Width | Count |
|---|---|---|---|
| `tx_lane0_data0` .. `tx_lane0_data16` | out | 64 | 17 |
| `rx_lane0_data0` .. `rx_lane0_data16` | in | 64 | 17 |

16 pins carry data and 1 is a redundant pin. In 32:1 mode the upper 32 bits of each 64-bit value
are zero. These pins connect to the analog EPHY, not to the package directly.

There are **no** separate lane clock ports: the lane TX clock is `mb_clk` and the lane RX clock is
`phy_rx_clk0`, both supplied by the analog PHY (see §4.1).

### 4.6 Sideband

| Signal | Dir | Notes |
|---|---|---|
| `sb_txd` | out | serial sideband TX data, synchronous to `sb_tx_clk` |
| `sb_rxd` | in | serial sideband RX data, synchronous to `sb_rx_clk` |
| `sb_txr` | out | sideband TX redundant channel |
| `sb_rxr` | in | sideband RX redundant channel |
| `sb_tx_clk` | out | forwarded (gated) sideband TX clock |

The sideband carries link-state and event traffic; it must be connected even if the application
never uses in-band control messages, because link bring-up and error handling depend on it.

### 4.7 Interrupts, status and DFX

| Signal | Dir | Notes |
|---|---|---|
| `irq_ext` | out | aggregated logic-PHY interrupt, active high |
| `hipi_ctrl_intr` | out | aggregated controller interrupt (protocol layer / link layer) |
| `dfx_stat_o[31:0]` | out | DFX status vector |

The two interrupt lines are intentionally **not** OR-ed inside the block: wire them to separate
interrupt inputs (or separate bits of your interrupt controller) so firmware can distinguish
PHY-level from protocol/link-level events. Per-source masks exist in the CSR (PHY interrupt masks
at `0x0_0010` / `0x0_0014`, controller mask in the `pcsr` segment).

### 4.8 Remaining ports

| Group | Count | Integration note |
|---|---|---|
| `udi_*` | 5 | user-defined interface bridge: `udi_tx_link0_lane0_data_i`, `udi_tx_link0_lane0_data_vld_i`, `udi_tx_link0_ready_o`, `udi_rx_link0_lane0_data_o`, `udi_rx_link0_lane0_data_vld_o` (`CPIF_DATA_WIDTH` wide). Leave unconnected if unused |
| `lane0_RX[0:16]_OFFSET_CTRL_DATA` | 17 | `inout` analog-PHY lane offset-control data. Leave unconnected if unused |
| `lp_ctrl_*` | 4 | low-power handshake; tie to the inactive level if unused |
| `ahbs_*` | - | AHB slave into the logic-PHY MCU subsystem; not built in this configuration (`MCU_SUBSYS` undefined), leave unconnected |

## 5. Configuration space and bring-up

### 5.1 APB address map

```text
hipi_apb_paddr[19:0]
  +- paddr[16]=0 -> logic PHY CSR (base 0x0_0000)
  |    +- 0x0_0000 .. 0x0_00FF  common control / training (train_mode ...)
  |    +- 0x0_0400 .. 0x0_04FF  PHY interrupt / status
  |    +- 0x0_0800 .. 0x0_08FF  sideband (SBC)
  |    +- 0x0_0C00 ..           further PHY blocks
  +- paddr[16]=1 -> controller CSR (base 0x1_0000)
       +- 0x1_0000 .. 0x1_03FF  pcsr  (common CSR: reset, clock enables, masks)
       +- 0x1_0400 .. 0x1_07FF  prcl  (protocol layer)
       +- 0x1_0C00 .. 0x1_0FFF  link  (link layer)
```

### 5.2 Bring-up sequence

Register offsets below are relative to their segment base (`pcsr` `0x1_0000`, `prcl` `0x1_0400`,
`link` `0x1_0C00`, logic PHY `0x0_0000`); add the base to get the full APB address.

`TRAIN_MODE=3` (training bypass) is mandatory for the current RTL drop: the RX training FSM is a
three-state shell stub, so the automatic flow (`TRAIN_MODE=0`) stalls in training and the link never
reaches `S_NORMAL`.

| Step | Address | Value | Meaning |
|---|---|---|---|
| P0 | - | - | release `presetn`, `mb_rst_n`, `sb_rst_n`, `mcu_rst_n`; hold `aresetn` low |
| P1 | pcsr `0x000` | `0x3` | release main-band / sideband resets from software |
| P1 | pcsr `0x004` | `0x11` | protocol-layer TX/RX clock enables (`tx_clken[0]`, `rx_clken[0]`) |
| P2 | phy `0x000` | `0x3` | `train_mode=3` (BYPASS) |
| P2 | phy `0x004` | `0x10` | `PHY_MODE=0` (32:1), `lane_num=1` |
| P2 | phy `0x008` | `0x1` | PRBS high-32 clock gate enable |
| P2 | phy `0x41C` | `0x7f` | training/calibration timeout step enable |
| P2 | phy `0x810` .. | - | sideband enable / masks |
| P2 | phy `0x000` | retrain event | trigger the first training pass |
| P3 | link `0x000` | `0x1C` | `PHY_MODE` = NRZ / 32:1 |
| P3 | link `0x00C` | `0x0B` | `MODE_REG` = WORK + PAIF |
| P3 | link `0x010` | `0x05` | `TX_RST_N` / `RX_RST_N` = 1 (link out of reset) |
| P3 | link `0x014` | per level | `LOOPBACK_EN` (bit16) for CPIF loopback; 0 for normal operation |
| P3 | link `0x018` | `0x0002_0207` | `RETRY_CFG` reset default |
| P3 | link `0x004` | retrain event | trigger link retrain, then wait for `S_NORMAL` |
| P4a | - | - | release `aresetn` (protocol layer, last) |
| P4b | prcl `0x500` | `0x0` | `hipi_sel=0` (HIPI Next engine) |
| P4b | prcl `0x428` | `0x1` | request timeout monitoring enable |
| P4b | prcl `0x414` | `0x1` | **credit gating enable - write this last** |

Read back each write; a mismatch means the write did not take effect. `SBC_LANE_ENABLE`
(`phy 0x824`) reads back `0xF` even when `0x1` is written: known hard-coded value in the generated
register file, not a bus fault.

### 5.3 Completion criteria

| Check | Target | Recommended timeout |
|---|---|---|
| logic PHY TX FSM | `ACTIVE` (`tx_state=8`) | 2 ms |
| logic PHY RX FSM | `ACTIVE` (`rx_state=2`) | 2 ms |
| link TX / RX FSM | `S_NORMAL` (`link 0x034` bits[7:4] = 4) | 2 ms |
| link `phy_ready` | `link 0x034` bit0 = 1 | - |

Only start AXI traffic after both the PHY and the link report ready. If `S_NORMAL` is never
reached, check `aresetn` timing, then the P2/P3 writes, then the sideband connection.

## 6. RTL integration

### 6.1 Filelist order

Compile through the filelists; do not hand-assemble a file list from the RTL tree, because the
macro windows are opened and closed by specific `.vh` files in a fixed order.

| Order | File | Effect |
|---|---|---|
| 1 | `code/design/filelist/d2d_hipi_dig_top.f` | activates `hipi_top_defines.vh`, then `hipi_ctrl.f` (pcsr, AXI protocol layer, link layer, controller top), `common.f`, `logic_phy.f`, and finally `d2d_hipi_dig_top.v` |
| 2 | `code/design/rtl_v/top/hipi_top_undefines.vh` | closes the top macro window - required before any TB or wrapper code |
| 3 | `code/design/filelist/hardip.f` | `sram_1rw_128b.v` for `replay_buffer` |

Requirements:

- Nested filelists must be included with upper-case `-F` (Verilator) so that environment-variable
  paths (`$HIPINEXT_HOME`, `$D2D_HIPI_DV_HOME`) are resolved and sub-filelists expanded.
- `HIPINEXT_HOME` must point at `code/`. With another simulator, expand the filelists once into a
  flat list, preserving order and the `.vh` positions.
- The RTL uses no `` `include ``; macro files are compiled directly as filelist entries.
- Adapting tool paths, expanding the shipped filelists into your own build script, or passing the
  existing make/script arguments is configuration (permitted by `LICENSE` Part II §3).
  Editing the shipped filelists, RTL, testbench or `hipi_top_defines.vh` is a modification and needs
  prior written authorization from OpenHappy Team.

### 6.2 Macro configuration

`code/design/rtl_v/top/hipi_top_defines.vh` is the single configuration source and defines this
release: `PRCL_AXI` on, symmetric AXI4 (`AXI_ASYM_MODE` off), iodie / cmptdie modes off, single
lane (`PRCL_TX_LANE_1/2/3`, `PRCL_RX_LANE_1/2/3`, `PHY_LANE_1/2/3` off), PAIF 512 bit,
`CPIF_DATA_WIDTH_EQ_1024` on, `MCU_SUBSYS` off.

**Do not edit it.** The RTL for the other shapes is not in this tree, so enabling those macros makes
the build either fail or produce a design nobody has verified. A different shape (extra lanes,
asymmetric direction, CXS, MCU subsystem) requires a separate release with its own defines and
filelists.

The 64:1 IO mode is not available: `io_data_width_mode_strap` is tied low inside the top level and
is not a top-level port.

### 6.3 PDK cells and SRAM

- PDK standard cells were replaced by hand-written `generic_*` wrappers in
  `code/common/xl_std_cell/` (13 files: buffers, inverters, clock gates, muxes, flops). Map them to
  your library cells (or let synthesis map them) rather than instantiating PDK cells directly.
- `link_layer/replay_buffer` uses `sram_1rw_128b.v`, a behavioral model (8 x 128 bit x 32 deep).
  For implementation, replace it with the corresponding foundry SRAM macro; the interface is a
  single-port synchronous SRAM.

## 7. Analog PHY boundary

The digital block expects an analog EPHY named `HIPI_2D_D2D_PHY_TOP` with a 450-pin port list
(`code/design/rtl_v/top/d2d_ana_top_stub.sv`). That stub has an empty body; a second shell
(`phy_model_stub.sv`, 242 pins, real-number bump ports) exists for waveform-level model integration.
Neither is used by the shipped testbench.

To integrate a real PHY:

1. Obtain the analog EPHY library/model with the same module name and port table.
2. Replace the stub in your top-level filelist (the stub is not referenced by
   `d2d_hipi_dig_top.f`, so only your wrapper changes).
3. Connect `tx_lane0_data*` / `sb_txd` / `sb_txr` / `sb_tx_clk` from the digital block to the EPHY
   TX side, and `rx_lane0_data*` / `sb_rxd` / `sb_rxr` from the EPHY RX side back in.
4. Supply `mb_clk`, `phy_rx_clk0`, `sb_tx_clk_in`, `sb_rx_clk`, `pll_lock` from the EPHY, and
   route `sb_tx_clk` back to it.
5. Re-run the P0-P4 bring-up: PHY and link state are readable over APB exactly as in simulation.

## 8. Physical lane requirements

Guidance derived from GB/T 46280.4—2025 and from the implementation. The figures below are
engineering guidance, not measured or certified limits:

- **Data distribution is LSB-first.** `tx_io_mapper` / `rx_io_mapper` include an MSB/LSB flip option
  to compensate for reversed PCB/interposer routing. Decide the flip once per board and keep the
  configuration symmetric on both dies.
- **Lane skew** is compensated in the link layer (`lane_dskew`). Keep intra-lane skew within the
  EPHY's specified training capability; per-step training registers exist
  (`logic_phy/train/`: offset, Vref, PI, DCDL, data check) but the training FSM is not functional in
  this drop, so bring-up on silicon will need external calibration or a new RTL drop.
- **Sideband clock is forwarded and gated**: `sb_tx_clk` only toggles while a packet is active.
  Treat it as a gated clock in the EPHY and in timing constraints.
- **Serialization is 32:1** at 1 GHz main-band rate. Check that EPHY, package and interposer support
  the resulting per-pin rate together with the 17-pin (16 data + 1 redundant) bundle.

## 9. Bring-up and debug of a new instance

1. Power-on: hold all resets low, start clocks (`pclk` first if your sequencing requires it).
2. Release `presetn`, then `mb_rst_n` / `sb_rst_n` / `mcu_rst_n`.
3. Apply the P1-P3 writes from section 5.2 over APB; read every register back.
4. Poll the link status register (`link 0x034`) until bits[7:4] = 4 (`S_NORMAL`).
5. Release `aresetn`, then apply P4b (`hipi_sel`, `timeout_en`, `credit_en` last).
6. Send one AXI write and one AXI read from the local side and check for the expected B/R response
   from the peer side.
7. If nothing arrives, isolate the layer:
   - `prcl 0x500` bit0 = 1 (`p_paif_loop`, protocol-layer local loopback) exercises the protocol
     layer alone: an AXI request issued on `s0_axi` must come back out of `m0_axi`.
   - `link 0x014` bit16 = 1 (CPIF loopback) additionally exercises the link layer.
   - Clearing both restores normal die-to-die operation.
8. Check `irq_ext` and `hipi_ctrl_intr` and read the PHY/link status and mask registers before
   assuming a data-path bug.

The same three levels exist in simulation as `LOOPBACK=paif`, `LOOPBACK=cpif` and `LOOPBACK=phys`;
use them to compare silicon behaviour against the reference environment.

## 10. Verifying the integrated instance

Minimum acceptance before handing the instance to software:

1. Compile the integrated top with the filelists of section 6.1 (plus your own top-level wrapper).
2. Run `verify/` unchanged against your wrapper (wrap `d2d_hipi_dig_top`, keep the port names) or
   instantiate the shipped TB around it: `make run TEST=smoke_1wr1rd` must report
   `TEST RESULT: PASS`, and `make regress` must pass all three loopback levels.
3. Confirm reset sequencing in your own wrapper: `aresetn` released only after link `S_NORMAL`.
4. Confirm the analog PHY replacement still exposes `pll_lock` / recovered clocks with the same
   polarity and clock-gating behaviour as the model.

For reference, the shipped tree currently passes `smoke_1wr1rd` at `phys`, `paif` and `cpif` with
Verilator 5.050.

## 11. Known limitations and open items

| Area | State | Impact on integration |
|---|---|---|
| RX training FSM | three-state shell stub; `TRAIN_MODE=3` (bypass) required | no automatic per-step training; silicon bring-up needs external calibration or a new RTL drop |
| Physical lanes | lane0 only | bandwidth limited to one lane; multi-lane shapes need a new release |
| Analog PHY | stubs only (`d2d_ana_top_stub.sv`, `phy_model_stub.sv`) | no functional link simulation without a real EPHY model |
| `remote_loop` (bond-layer loopback, `prcl 0x500` bit1) | present in RTL, requires two dies | cannot be exercised with a single die; the shipped TB rejects `LOOPBACK=axi` |
| `SBC_LANE_ENABLE` | reads back `0xF` regardless of write | sideband lane enable is effectively fixed; firmware must ignore the field |
| MCU subsystem / AHB slave | `MCU_SUBSYS` undefined | `ahbs_*` ports are inert; configuration goes through the top-level APB port |
| Synthesis, DFT, timing | no flow in this repository | constraints, scan insertion and timing closure are yours |
| 2.5D package (standard part 5) | not implemented | 2D package only |

## 12. Integration checklist

- [ ] Clock plan: `pclk`, `mb_clk`, `aclk`, `phy_rx_clk0` (1 GHz), `sb_tx_clk_in`, `sb_rx_clk`
      (800 MHz) sourced and constrained; `pll_lock` tied to a defined level.
- [ ] Reset plan: `presetn` first, `mb_rst_n` / `sb_rst_n` / `mcu_rst_n` next, `aresetn` last after
      link `S_NORMAL`.
- [ ] APB3 master connected, bit[16] domain decode implemented, readback checking in firmware.
- [ ] `s0_axi` connected to local masters, `m0_axi` connected to local slaves, both on `aclk`.
- [ ] 17 TX and 17 RX lane pins plus lane clocks routed to the EPHY with the chosen MSB/LSB policy.
- [ ] Sideband `sb_txd` / `sb_rxd` / `sb_txr` / `sb_rxr` / `sb_tx_clk` connected.
- [ ] `irq_ext` and `hipi_ctrl_intr` routed to separate interrupt sources; masks programmed.
- [ ] RTL compiled through `d2d_hipi_dig_top.f` + `hipi_top_undefines.vh` + `hardip.f`, `-F`
      semantics preserved, `HIPINEXT_HOME` pointing at `code/`.
- [ ] `hipi_top_defines.vh` unchanged; desired shape matches the shipped symmetric single-lane build.
- [ ] `generic_*` cells mapped to the target library; `sram_1rw_128b` replaced by the foundry macro.
- [ ] Analog EPHY dropped in with the `HIPI_2D_D2D_PHY_TOP` port list; stubs removed from the
      production filelist.
- [ ] P0-P4b bring-up executed with readback; PHY `ACTIVE` and link `S_NORMAL` observed.
- [ ] One AXI write and one read complete across the link; loopback levels used for fault isolation.
- [ ] `make run TEST=smoke_1wr1rd` and `make regress` still pass against the integrated wrapper.
