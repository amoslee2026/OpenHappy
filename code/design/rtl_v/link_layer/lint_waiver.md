---
title: link_layer RTL Lint Waiver List
type: list
purpose: checklist
audience: llm
direction: both
status: approved
version: 0.1.0
author: OpenHappy Team
section_meta: "@meta"
---

# link_layer RTL lint waiver list

> Version: 0.1.0
> Date: 2026-08-07
> Maintainer: OpenHappy Team
> Purpose: record lint (SpyGlass / Verilator and similar) waiver items for the link_layer RTL, for use
>        during functional verification and synthesis review. Every waiver states module / warning /
>        root cause (design intent) / basis (MAS section).

| ID | Registered | Module | File | Warning class | Object | Rationale (design intent) | Basis |
|------|----------|------|------|----------|----------|----------------------|------|
| LW-001 | 2026-08-07 | crc9_generate | crc9_generate.v | UNUSED (input port unused, e.g. SpyGlass W391 / Verilator UNUSEDSIGNAL) | `mb_clk` | Pure combinational module (no always, no registers); clk/rst are kept as ports to match the parent instantiation convention and are not consumed inside | crc9_generate_mas.md §2.1 / §6.1 |
| LW-002 | 2026-08-07 | crc9_generate | crc9_generate.v | UNUSED (input port unused, e.g. SpyGlass W391 / Verilator UNUSEDSIGNAL) | `mb_rst_n` | Same as above: combinational only, reset has no target; respects the parent link_tx synchronous-reset constraint | crc9_generate_mas.md §2.1 / §6.1 |
| LW-003 | 2026-08-07 | crc16_generate | crc16_generate.v | UNUSED (input port unused, e.g. SpyGlass W391 / Verilator UNUSEDSIGNAL) | `mb_clk` | Pure combinational module (no always, no registers); clk/rst kept as ports to match the parent instantiation convention, not consumed inside | crc16_generate_mas.md §2.1 / §6.1 |
| LW-004 | 2026-08-07 | crc16_generate | crc16_generate.v | UNUSED (input port unused, e.g. SpyGlass W391 / Verilator UNUSEDSIGNAL) | `mb_rst_n` | Same as above: combinational only, reset has no target; respects the parent link_tx synchronous-reset constraint | crc16_generate_mas.md §2.1 / §6.1 |
| LW-005 | 2026-08-08 | crc9_check | crc9_check.v | UNUSED (input port unused) | `mb_clk` | Pure combinational module (no always, no registers); clk/rst kept as ports to match the parent link_rx instantiation convention, not consumed inside | crc9_check_mas.md §2.1 |
| LW-006 | 2026-08-08 | crc9_check | crc9_check.v | UNUSED (input port unused) | `mb_rst_n` | Same as above: combinational only, reset has no target | crc9_check_mas.md §2.1 |
| LW-007 | 2026-08-08 | crc16_check | crc16_check.v | UNUSED (input port unused) | `mb_clk` | Pure combinational module (no always, no registers); clk/rst kept as ports, not consumed inside | crc16_check_mas.md §2.1 |
| LW-008 | 2026-08-08 | crc16_check | crc16_check.v | UNUSED (input port unused) | `mb_rst_n` | Same as above: combinational only, reset has no target | crc16_check_mas.md §2.1 |
| LW-009 | 2026-08-08 | replay_buffer / credit_fifo | replay_buffer.v / credit_fifo.v | Width padding (e.g. iverilog "Port addr expects 8 bits, got 6") | SRAM `addr` port | SRAM physical depth is fixed at 256 (8-bit address); the effective depth is set by `REPLAY_BUF_DEPTH` / `CREDIT_FIFO_DEPTH` (default 64 -> 6-bit address); zero-padding the high bits is intended | replay_buffer_mas.md §2.1 / credit_fifo_mas.md §6.1 |
| LW-010 | 2026-08-08 | lane_dskew | lane_dskew.v | UNUSED (input port unused) | `lane1_valid_i` (and lane2/3_valid_i) | MAS §2.1 defines per-lane valid inputs, but valid_o follows lane0_valid_i (all lanes already aligned), so lane1~3 valid are not consumed; ports are kept per MAS | lane_dskew_mas.md §2.1 |
| LW-011 | 2026-08-08 | link_tx_lcmsm / link_rx_lcmsm | link_tx_lcmsm.v / link_rx_lcmsm.v | UNUSED (input port unused) | `work_mode_i` / `evt_result_i` | Required by the MAS §2.1 port table, but neither the MAS §4/§5 pseudocode nor the fsm.json transitions consume them (evt_result is not judged pass/fail; the work_mode function is carried by bridge_en_i) | link_tx_lcmsm_mas.md / link_rx_lcmsm_mas.md §2.1 |
| LW-012 | 2026-08-08 | apb_csr | apb_csr.v | UNUSED (some bits unused) | `pwdata` high bits (RESERVED bits) | Writes to reserved register bits are ignored without error and read back as 0, consistent with the MAS register attribute definition | apb_csr_mas.md §2.2 |
| LW-013 | 2026-08-08 | link_layer | link_layer.v | UNUSED / dangling | `tx_dskew_pattern` (lcmsm dskew_pattern_o), multi-link status outputs | (1) link_tx has no dskew_pat input port (undefined in link_tx_mas v1.0), so lcmsm dskew_pattern_o is left dangling; (2) DFx/interrupt aggregation for asymmetric multi-link is not specified, only link0 is aggregated | link_layer_spec.md (L2) §3.2 / link_tx_mas.md §3.1 |
| LW-005 | 2026-08-08 | link_tx | link_tx.v | UNUSED (some bits unused, Verilator UNUSEDSIGNAL) | `flit_beats_w[11:5]` | Intermediate beat-count value (2048>>total_eff_log2); only the low 5 bits are significant (max 16 beats), the high bits are constant 0 by intended truncation | link_tx_mas.md §4.7 / L1 §4.2.5 |
| LW-006 | 2026-08-08 | link_tx | link_tx.v | UNUSED (some bits unused, Verilator UNUSEDSIGNAL) | `back_first_w[11:5]` | Same as above: intermediate first-beat index of the rear 128 B; only the low 5 bits are significant, high bits constant 0 by intended truncation | link_tx_mas.md §4.7 / L1 §4.2.5 |
| LW-007 | 2026-08-08 | replay_buffer | replay_buffer.v | UNUSED (some bits unused, Verilator UNUSEDSIGNAL) | `rptr_i[6]` | The flag bit of the N-bit pointer only disambiguates empty/full and never enters the SRAM address (the address uses the low N-1 bits); unused by design per the MAS pointer definition | replay_buffer_mas.md §2.1 / §5.1 |
| LW-008 | 2026-08-08 | link_tx_lcmsm / link_rx_lcmsm | link_tx_lcmsm.v / link_rx_lcmsm.v | UNOPTFLAT (Verilator combinational-loop warning) | `state_ctl[18:0]` / `state_ctl[20:0]` | state_ctl uses the three-stage FSM style (following the phy-side example): high bits reference lower bits in a priority-merge chain, not a real combinational loop; lint_off/on pragmas are placed in the source | lcmsm MAS §4 transition tables |
| LW-009 | 2026-08-11 | link_layer | link_layer.v | UNUSEDSIGNAL (Verilator -Wall) | `paif_clk` / `paif_rstn` | Reserved top-level ports (hipi_ctrl_top compatibility), currently unused | L2 §2.1 |
| LW-011 | 2026-08-11 | link_layer | link_layer.v | UNUSEDSIGNAL (some bits) | `csr_loopback_src_mb_sync[15:4]` | RX0_SRC[3:0] is consumed by the RX0 loopback source-select mux (after the 2026-10-10 fix, see link_layer.v RX0 mux); [7:4]/[11:8]/[15:12] are RX1/2/3_SRC used as RX1/2/3 mux case keys and are only consumed when the matching `RX_LINKx_EXIST` expands -- in the current tree RX_LINK1/2/3 are disabled, so [15:4] is not consumed | L1 §5.3 LOOPBACK_CFG_REG |
| LW-012 | 2026-08-11 | link_layer | link_layer.v | PINCONNECTEMPTY | `cap_lane0~3_o` / `align_cnt_o` | DFx observation / reserved outputs, intentionally left dangling at top level (2026-10-08 W287b: `rx_evt_lane_on/off_o` were removed with the port deletion, link_event_process MAS v1.5) | lane_dskew MAS §2.1 / link_rx spec §3.1 |
| LW-013 | 2026-08-11 | link_tx_lcmsm / link_rx_lcmsm | link_*_lcmsm.v | UNUSEDSIGNAL (some bits) | `state_ctl[15]` / `state_ctl[10]` | Unused bits of the three-stage state_ctl style, explicitly tied to 0 to avoid undriven | lcmsm MAS §4.4 |
| LW-014 | 2026-08-11 | link_rx | link_rx.v | UNUSEDSIGNAL | `csr_loopback_src_i` / `arq_done_i` | Source selection happens in the top-level mux (not consumed in this module); arq_done is consumed by rx_lcmsm (pass-through observation port) | link_rx spec §3.1 |
| LW-015 | 2026-08-11 | link_rx | link_rx.v | UNUSEDSIGNAL (some bits) | `dskew_data[2047:1536,1023:512]` | With 2 lanes and s2p_mode=0 the upper 512 bits of each lane are always 0; the cell extraction uses only the lower half, unused by design | L1 §4.3.3.2 |
| LW-016 | 2026-08-11 | link_event_process | link_event_process.v | UNUSEDSIGNAL (some bits) | `csr_lp_cmd_i[5,3:1]` / `evt_status_q[7:4]` | Only bit0/bit4 of lp_cmd have defined opcodes (the rest are pending L1 §6); EventStatus returns only the low 4 bits to lp_sw_sts | event MAS §4.3 |
| LW-017 | 2026-08-11 | apb_csr | apb_csr.v | UNUSEDSIGNAL (some bits) | `pwdata_q[31:18]` / `init_rise[2]` | Reserved register bits have no write decode; init_rise[2]=CONFIG_DONE is driven as a level after the change, bypassing edge detection | apb_csr MAS §2.2 |
| LW-018 | 2026-08-11 | lane_dskew | lane_dskew.v | UNUSEDSIGNAL | `lane1_valid_i` | valid follows lane0 (all lanes share it after alignment); lane1_valid is declared but not consumed | lane_dskew MAS §2.1 |
| LW-019 | 2026-08-11 | crc9_check | crc9_check.v | UNUSEDSIGNAL (some bits) | `flit_i[271:270]` | Tail Rsv bits in compatibility mode do not participate in CRC9 checking | crc9_check MAS §5.1 |
| LW-020 | 2026-08-11 | sync_cell_norst | sync_cell_norst.v | UNUSEDPARAM | `size` | Template-reserved parameter (content of the user-supplied original template), currently unused | - |
