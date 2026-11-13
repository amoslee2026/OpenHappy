// =============================================================================
// module name：apb_csr_top
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/mas/apb_csr_top_mas.md
// =============================================================================
// 功能概述: logic_phy 集中式 CSR 的 APB 唯一接入点：0x000~0xCFF 地址段寄存器物理实现，
//   具名配置直送各消费方 + 7 类中断 sticky 聚合 + 状态回流 2flop 同步。
//   非对称 4T1R（LANE_NUM=4，4 TX + 1 RX）：寄存器本体由生成物 apb_csr_top_rf 承载
//   （reg-gen flow, flow/register_flow/logic_phy_reg_flow, 413 主寄存器, 同源 UVM
//   RAL model）；本模块为 wrapper：APB 前端/pslverr/读mux override + 2flop 同步 +
//   sticky 中断聚合 + int mask 镜像（0x00C/0x900 双入口写同一存储）+ global_rst
//   自恢复 + per-lane 输出打包 + lane2 data_path 影子 alias 寄存器（0x400/0x404/
//   0x440/0x444/0x448 共享段双写）。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-09-16  cdc-resolver  CDC 违例修复（Ac_unsync01/02：sbc_tx）：sbc_tx 手动注入触发
//                          sbc_tx_ctrl_tx_manual_trigger_wp 漏转 toggle（09-13 修复遗漏），
//                          补入 toggle 载体（与 09-13 同范式），sbc_tx 侧 2FF 同步 + 同步
//                          输出打拍 XOR 边沿恢复 1 拍脉冲（sbc_tx.v 已适配）
//  2026-09-13  cdc-resolver  CDC 违例修复（SPYGLASS Ac_unsync01/02 ×117；MAS v3.1）：8 条 *_wp 写脉冲输出改 toggle 载体（seed_load_tx/rx、clear_tx/rx、force_mode_switch、step_trigger、sbc_rx_soft_rst、sbc_rx_manual_read），目的域 2FF 同步 + 同步输出打拍边沿恢复 1 拍脉冲（下游 leaf 已适配）
//  2026-08-07  OpenHappy Team  初始版本：基于 apb_csr_top MAS v2.7 生成
//  2026-08-11  OpenHappy Team  SBC_LANE_ENABLE 复位值参数化（SBC_LANE_EN_RST，
//                       由 logic_phy 按 LP_ lane 宏派生传入；MAS v2.8）
//  2026-08-11  OpenHappy Team  1-lane 配置（LANE_NUM=1）下 RX_ASYNC_FIFO_STATUS lane1
//                       寄存器（0x0C04）sticky 位越界读修复
//  2026-08-28  OpenHappy Team  非对称 4T1R（LANE_NUM=4）RTL 落地（MAS v3.0）
//  2026-09-10  OpenHappy Team  reg-gen 重构：寄存器本体改由 apb_csr_top_rf（generate,
//                       flow/register_flow/logic_phy_reg_flow, excel 427 寄存器）
//                       承载 + UVM RAL model 同源生成（export/sv/ral_logic_phy_
//                       reg_model.sv）；本模块保留 APB 前端/pslverr/2flop 同步/
//                       sticky 中断聚合/int mask 镜像/global_rst 自恢复/per-lane
//                       输出打包（reg-gen skill §5 架构）。读回契约与原 RTL 逐位
//                       一致（4 处 prdata override 见设计说明 2）；影子写保留
//                       lane2 data_path/rx + seed_tx i0~2（0x400/0x404/
//                       0x440/0x444/0x448 共享段双写, 驱 lane2 输出）, 移除
//                       lane3 offset i0~2（0x700/0x704/0x708）死 alias（无读
//                       无输出, 行为不可见）
//  2026-09-11  OpenHappy Team  MDE-CHAIN-RERUN-01-271-IMPL（77/82 实接）:
//                       ①族 a lane0 per-IO 训练参数 64（offset/vref/pi/dcdl）→ 同名
//                       wrapper 输出端口 → logic_phy→lane_top→rx_chain→rx_training→
//                       train RX{i}_*_PARAM（gen_mech 扩展, lanes1-3 189 待 TRAIN-MAS-01）;
//                       ②族 b WO 清位 8 单驱化（清位源归 rf int_clear/intc_clear/
//                       sbc_irq_flag wp/wd, 删自持 int_clear_wen/sbc_irq_clear_pulse）;
//                       ③族 c #1 global_rst 写源采样 rf 副本/#3 train_param_src_o/
//                       #4 datacheck_cfg_fail_threshold_o/#7 eq_param_ctle_tap_o/#8
//                       cdc_fifo_rst 消费; #2 train_mode(0x400)/#5/#6 reorder 待
//                       designer <1:1>, 族 d int_mask×2 HOLD（详见设计说明 7）。
//  2026-10-21  OpenHappy Team  APB3 读完成拍修正（issue #18）：prdata 由 access 拍寄存改为组合直出，与 pready 同拍有效（ARM IHI 0024C）；删 prdata_reg
// =============================================================================

`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// 设计说明:
//
//   1. 寄存器本体：apb_csr_top_rf（DO NOT edit manually, reg-gen flow 生成）。
//      扁平 12-bit 绝对地址映射（413 主寄存器 = top 12 / per-lane 359（4 lane
//      按原 RTL 解码展开: lane0 64 / lane1 101 / lane2 96 / lane3 98, 影子冲突
//      45 条目除外）/ per-link 共享 19 / 训练状态 3 / sideband 10 / intc 5 /
//      mcu 1 / RX FIFO 状态 4）。rf 总线 reg_wr/reg_rd 由 addr_valid 门控
//      （0x1000+ 地址 12-bit 截断不触达 per-lane 存储）。
//   2. 读回契约：apb_prdata = rf.reg_rdata + 4 处 override（wrapper 自持状态）：
//        0x000[4]   global_rst_n —— rf 副本无自恢复, override 用 wrapper 自恢复值
//        0x00C/0x010 int mask   —— 双入口镜像, override 用 wrapper 镜像寄存器
//                                   （0x900/0x904 经 rf RO 输入与 wrapper 同源）
//        0x820[1:0] SBC_IRQ_FLAG —— 原设计读返 SBC_DISPATCH_STATUS 镜像
//      W1S 脉冲字段读回恒 0（pulse 仅 1 cycle, APB 读 access 最早在写 access 后
//      2 cycle, 原 RTL 读回同样不可见 pulse 态, 行为一致）。
//   3. 影子 alias（原 RTL 解码 idx=paddr[11:9]/blk_off=paddr[8:0] 推得）:
//      a) lane2 data_path_cfg/rx (0x400/0x404) 与 SEED_TX i0~2 (0x440/0x444/0x448)
//         与共享段重叠 —— 读返共享段, 写双写（共享块 + per-lane 块）, wrapper 自持
//         alias 寄存器驱 lane2 输出（与原版 per-lane 写解码行为一致）;
//      b) lane3 OFFSET i0~2 (0x700/0x704/0x708) 与 status 段重叠 —— 死 alias
//         （无读无输出）, 省略;
//      c) lane0 低半（DPC/DPC_RX/SEED/FREQ/MSB, 0x000~0x0C8）低于 per_lane_hit
//         下限且与 top 段重叠 —— 不可达, 输出 = 复位值常量。
//   4. sticky/中断：INT_STATUS（0x014/0x908 镜像）/ SBC 状态 sticky / RX FIFO
//      ovf·unf sticky / CDC FIFO ovf·unf sticky 由 wrapper 自持（set 源 = 2flop
//      同步回流, clear = 对应地址写 1, 置位优先于清除）; rf 中这些字段建模为
//      RO（wrapper 驱动 rf 输入）, 读回 = sticky 值。
//   5. 中断聚合：7 类 = lane_top_irq_i（train done/fail/step done/timeout,
//      跨 4 lane OR）+ sideband_irq_i（sbc err/buf ovf）+ CDC FIFO ovf|unf
//      （内部 sticky, set 源 TODO tie 0）。
//   6. CDC：状态回流 2flop 同步（源域 → apb_pclk, per-bit 独立）; 配置输出
//      apb_pclk 域寄存器直出, 跨域同步由消费侧完成（与原 MAS §3.5 实现一致）。
//   7. 未连接说明（MDE-CHAIN-RERUN-01 连接演进, 2026-09-11）:
//      - 零盲接协议复核（RTL-FIX）确认 271 未接输出均有因（旧 RTL 亦无消费/自持镜像/
//        MAS 未落地路由）→ 移交 admin 裁决（admin scope (iii): 实接+hook_min 清零）;
//      - 271-IMPL 实施后：**已实接 80 = 族 a lane0 64 + 族 b 8 + 族 c #1/#2/#3/#4/#7/#8 6
//        + 族 d 2（详见版本历史 2026-09-11）; u_rf 未接输出余 = 191 = lanes1-3 per-IO 训练
//        参数 189（TRAIN-MAS-01, 4T1R 仅 TX 无 rx 落点, 位恒 0 tie 在 logic_phy lane_top 层）
//        + 族 c #5/#6（reorder_cfg_max_train_cycles/skew_comp_depth, 零盲接留待; 与 rx_chain
//        实际 tie0 reorder_delay_rx[96:0]/order_rx[3:0]/valid 无 1:1, 待 designer <1:1>**,
//        到后独立一笔接）;
//      - D3b 4 输入（apb_csr_top test_mode/csr_bridge_mode_i/csr_inj_region_i 与
//        link_rx_512_lane1 csr_s2p_mode_i）为 W240 输入类（非 u_rf 输出）: 无 1:1 安全
//        来源（designer §4.2 核证 + spec 意图性保留端口）→ 不盲接不落 waiver, 报告列项
//        （flow/d2d_hipi_dig_top/archive/mde_271_d3b_links_report_2026-09-11.md）。
//      未映射清单（271 端口逐项）: flow/d2d_hipi_dig_top/archive/
//      mde_rerun_01_rtlf_unmapped_2026-09-11.md（随 wt/mde-rerun-01-rtlf 提交）。
//   8. 机械展开段（wire 声明 / u_rf 例化 / per-lane 打包）由
//      flow/register_flow/logic_phy_reg_flow/gen_mech.py 生成（勿手改）。
// -----------------------------------------------------------------------------

module apb_csr_top #(
    parameter LANE_NUM = 4,   // 非对称 4T1R baseline：4 TX lane + 1 RX lane（bit0）
    parameter IO_NUM   = 16,  // per-link 逻辑 IO 数
    parameter SBC_LANE_EN_RST = 4'b1111  // SBC_LANE_ENABLE.lane_enable 复位值（logic_phy 按 LP_ 宏派生传入）
) (
    // 时钟与复位
    input  wire         apb_pclk,
    input  wire         apb_preset_n,

    // APB slave 接口（mcu_subsys.apb_master_o）
    input  wire         apb_psel,
    input  wire         apb_penable,
    input  wire         apb_pwrite,
    input  wire [15:0]  apb_paddr,
    input  wire [31:0]  apb_pwdata,
    output wire [31:0]  apb_prdata,
    output wire         apb_pslverr,
    output wire         apb_pready,

    // bring-up strap：IO_DATA_WIDTH_MODE[0] 的启动值（apb_csr_top_mas.md:75）
    //   apb_pclk 域；复位释放首沿锁存进 0x004[0]，之后启动后只读（运行时写 → pslverr）
    input  wire         io_data_width_mode_strap,

    // 中断回流（per-lane 4 bit × LANE_NUM，4T1R 下 lane0~3 均有效）
    input  wire [15:0]  lane_top_irq_i,
    input  wire [3:0]   sideband_irq_i,

    // SBC 状态回流（sb_tx_clk 域源）
    input  wire [2:0]   sbc_tx_buf_level_i,
    input  wire         sbc_tx_busy_i,
    input  wire         sbc_tx_err_i,
    input  wire [3:0]   sbc_rx_buf_level_i,
    input  wire         sbc_rx_busy_i,
    input  wire         sbc_rx_parity_err_i,
    input  wire         sbc_rx_gap_err_i,
    input  wire         sbc_rx_buf_ovf_i,
    input  wire [15:0]  sbc_rx_buf_rdata_i,
    input  wire         sbc_pkt_arrival_flag_i,
    input  wire         sbc_err_flag_i,
    input  wire         sbc_illegal_opcode_i,
    input  wire         sbc_illegal_lid_i,

    // DFX / FIFO 状态回流（per-lane 8 bit × LANE_NUM(4) = 32 bit）
    input  wire [15:0]  aligned_rx_i,
    input  wire [31:0]  rx_fifo_status_i,

    // 具名配置输出 → intc（同域无 CDC）
    output wire [6:0]   int_status_o,
    output wire [6:0]   int_mask_mcu_o,
    output wire [6:0]   int_mask_ext_o,

    // 具名配置输出 → lane_top（apb_pclk 域寄存器直出，使用侧做 CDC）
    output wire         io_data_width_mode_o,
    output wire [3:0]   reversal_en_tx_o,
    output wire [3:0]   reversal_en_rx_o,
    output wire [63:0]  repair_map_tx_o,
    output wire [63:0]  repair_map_rx_o,
    output wire [67:0]  msb_lsb_flip_en_tx_o,
    output wire [67:0]  msb_lsb_flip_en_rx_o,
    output wire [1471:0] prbs_seed_tx_o,
    output wire [1471:0] prbs_seed_rx_o,
    output wire [15:0]  flip_sel_tx_o,
    output wire [15:0]  flip_sel_rx_o,
    output wire [3:0]   scramble_en_o,
    output wire [3:0]   scramble_en_tx_o,
    output wire         seed_load_rx_o,
    output wire         clear_rx_o,
    output wire [3:0]   seed_load_tx_o,
    output wire [3:0]   clear_tx_o,
    output wire [3:0]   reorder_delay_step_o,
    output wire [1:0]   train_mode_o,
    output wire         force_mode_switch_o,
    output wire [6:0]   bypass_step_o,
    output wire [6:0]   timeout_step_en_o,
    output wire [15:0]  timeout_threshold_o,
    output wire         data_check_pattern_sel_o,
    output wire [63:0]  vref_pattern_tx_o,
    output wire [63:0]  pi_pattern_tx_o,
    output wire [63:0]  dcdl_pattern_tx_o,
    output wire [63:0]  reorder_pattern_tx_o,
    output wire [63:0]  datacheck_pattern_tx_o,
    output wire         step_trigger_o,
    output wire [6:0]   step_force_end_o,
    output wire [2:0]   step_cnt_o,

    // cfg broadcast → mcu_subsys
    output wire [31:0]  cfg_mcu_o,

    // 具名配置/控制输出 → sideband（apb_pclk 域直出，使用侧做 CDC）
    output wire         sbc_tx_enable_o,
    output wire         sbc_tx_manual_trigger_o,
    output wire [15:0]  sbc_tx_pkt_inject_o,
    output wire         sbc_rx_enable_o,
    output wire         sbc_rx_manual_read_o,
    output wire         sbc_rx_soft_rst_o,
    output wire [7:0]   sbc_rx_buf_depth_o,
    output wire [15:0]  sbc_lid_filter_o,
    output wire [7:0]   sbc_opcode_filter_o,
    output wire [7:0]   sbc_err_mask_o,
    output wire         sbc_pkt_arrival_mask_o,
    output wire         sbc_err_irq_mask_o,
    output wire [1:0]   sbc_irq_clear_o,
    output wire [3:0]   sbc_lane_enable_o,
    // 族 a lane0 per-IO 训练参数 → lane_top→rx_chain→rx_training→train（MDE-CHAIN-RERUN-01-271-IMPL; lanes1-3==TRAIN-MAS-01 扩展点）
    output wire [15:0]  offset_param_l0_i0_offset,
    output wire [15:0]  vref_param_l0_i0_vref,
    output wire [19:0]  pi_param_l0_i0_pi,
    output wire [11:0]  dcdl_param_l0_i0_dcdl,
    output wire [15:0]  offset_param_l0_i1_offset,
    output wire [15:0]  vref_param_l0_i1_vref,
    output wire [19:0]  pi_param_l0_i1_pi,
    output wire [11:0]  dcdl_param_l0_i1_dcdl,
    output wire [15:0]  offset_param_l0_i2_offset,
    output wire [15:0]  vref_param_l0_i2_vref,
    output wire [19:0]  pi_param_l0_i2_pi,
    output wire [11:0]  dcdl_param_l0_i2_dcdl,
    output wire [15:0]  offset_param_l0_i3_offset,
    output wire [15:0]  vref_param_l0_i3_vref,
    output wire [19:0]  pi_param_l0_i3_pi,
    output wire [11:0]  dcdl_param_l0_i3_dcdl,
    output wire [15:0]  offset_param_l0_i4_offset,
    output wire [15:0]  vref_param_l0_i4_vref,
    output wire [19:0]  pi_param_l0_i4_pi,
    output wire [11:0]  dcdl_param_l0_i4_dcdl,
    output wire [15:0]  offset_param_l0_i5_offset,
    output wire [15:0]  vref_param_l0_i5_vref,
    output wire [19:0]  pi_param_l0_i5_pi,
    output wire [11:0]  dcdl_param_l0_i5_dcdl,
    output wire [15:0]  offset_param_l0_i6_offset,
    output wire [15:0]  vref_param_l0_i6_vref,
    output wire [19:0]  pi_param_l0_i6_pi,
    output wire [11:0]  dcdl_param_l0_i6_dcdl,
    output wire [15:0]  offset_param_l0_i7_offset,
    output wire [15:0]  vref_param_l0_i7_vref,
    output wire [19:0]  pi_param_l0_i7_pi,
    output wire [11:0]  dcdl_param_l0_i7_dcdl,
    output wire [15:0]  offset_param_l0_i8_offset,
    output wire [15:0]  vref_param_l0_i8_vref,
    output wire [19:0]  pi_param_l0_i8_pi,
    output wire [11:0]  dcdl_param_l0_i8_dcdl,
    output wire [15:0]  offset_param_l0_i9_offset,
    output wire [15:0]  vref_param_l0_i9_vref,
    output wire [19:0]  pi_param_l0_i9_pi,
    output wire [11:0]  dcdl_param_l0_i9_dcdl,
    output wire [15:0]  offset_param_l0_i10_offset,
    output wire [15:0]  vref_param_l0_i10_vref,
    output wire [19:0]  pi_param_l0_i10_pi,
    output wire [11:0]  dcdl_param_l0_i10_dcdl,
    output wire [15:0]  offset_param_l0_i11_offset,
    output wire [15:0]  vref_param_l0_i11_vref,
    output wire [19:0]  pi_param_l0_i11_pi,
    output wire [11:0]  dcdl_param_l0_i11_dcdl,
    output wire [15:0]  offset_param_l0_i12_offset,
    output wire [15:0]  vref_param_l0_i12_vref,
    output wire [19:0]  pi_param_l0_i12_pi,
    output wire [11:0]  dcdl_param_l0_i12_dcdl,
    output wire [15:0]  offset_param_l0_i13_offset,
    output wire [15:0]  vref_param_l0_i13_vref,
    output wire [19:0]  pi_param_l0_i13_pi,
    output wire [11:0]  dcdl_param_l0_i13_dcdl,
    output wire [15:0]  offset_param_l0_i14_offset,
    output wire [15:0]  vref_param_l0_i14_vref,
    output wire [19:0]  pi_param_l0_i14_pi,
    output wire [11:0]  dcdl_param_l0_i14_dcdl,
    output wire [15:0]  offset_param_l0_i15_offset,
    output wire [15:0]  vref_param_l0_i15_vref,
    output wire [19:0]  pi_param_l0_i15_pi,
    output wire [11:0]  dcdl_param_l0_i15_dcdl,
    // i16（RX16 冗余 IO）per-IO 训练参数（WP1 P0-A 2026-09-22 新增；VREF/PI 为 per-lane 不展开 i16）
    output wire [15:0]  offset_param_l0_i16_offset,
    output wire [11:0]  dcdl_param_l0_i16_dcdl,
    // 族 a2 lane1/2/3 per-IO 训练参数 → lane_top→rx_chain→rx_training→train（MDE-CHAIN-1008 接口贯通 2026-10-12; 位序同 lane0: offset/dcdl i16 在尾, vref/pi 到 i15）
    output wire [15:0]  offset_param_l1_i16_offset,
    output wire [11:0]  dcdl_param_l1_i16_dcdl,
    output wire [15:0]  offset_param_l1_i0_offset,
    output wire [15:0]  offset_param_l1_i1_offset,
    output wire [15:0]  offset_param_l1_i2_offset,
    output wire [15:0]  offset_param_l1_i3_offset,
    output wire [15:0]  offset_param_l1_i4_offset,
    output wire [15:0]  offset_param_l1_i5_offset,
    output wire [15:0]  offset_param_l1_i6_offset,
    output wire [15:0]  offset_param_l1_i7_offset,
    output wire [15:0]  offset_param_l1_i8_offset,
    output wire [15:0]  offset_param_l1_i9_offset,
    output wire [15:0]  offset_param_l1_i10_offset,
    output wire [15:0]  offset_param_l1_i11_offset,
    output wire [15:0]  offset_param_l1_i12_offset,
    output wire [15:0]  offset_param_l1_i13_offset,
    output wire [15:0]  offset_param_l1_i14_offset,
    output wire [15:0]  offset_param_l1_i15_offset,
    output wire [15:0]  vref_param_l1_i0_vref,
    output wire [15:0]  vref_param_l1_i1_vref,
    output wire [15:0]  vref_param_l1_i2_vref,
    output wire [15:0]  vref_param_l1_i3_vref,
    output wire [15:0]  vref_param_l1_i4_vref,
    output wire [15:0]  vref_param_l1_i5_vref,
    output wire [15:0]  vref_param_l1_i6_vref,
    output wire [15:0]  vref_param_l1_i7_vref,
    output wire [15:0]  vref_param_l1_i8_vref,
    output wire [15:0]  vref_param_l1_i9_vref,
    output wire [15:0]  vref_param_l1_i10_vref,
    output wire [15:0]  vref_param_l1_i11_vref,
    output wire [15:0]  vref_param_l1_i12_vref,
    output wire [15:0]  vref_param_l1_i13_vref,
    output wire [15:0]  vref_param_l1_i14_vref,
    output wire [15:0]  vref_param_l1_i15_vref,
    output wire [19:0]  pi_param_l1_i0_pi,
    output wire [19:0]  pi_param_l1_i1_pi,
    output wire [19:0]  pi_param_l1_i2_pi,
    output wire [19:0]  pi_param_l1_i3_pi,
    output wire [19:0]  pi_param_l1_i4_pi,
    output wire [19:0]  pi_param_l1_i5_pi,
    output wire [19:0]  pi_param_l1_i6_pi,
    output wire [19:0]  pi_param_l1_i7_pi,
    output wire [19:0]  pi_param_l1_i8_pi,
    output wire [19:0]  pi_param_l1_i9_pi,
    output wire [19:0]  pi_param_l1_i10_pi,
    output wire [19:0]  pi_param_l1_i11_pi,
    output wire [19:0]  pi_param_l1_i12_pi,
    output wire [19:0]  pi_param_l1_i13_pi,
    output wire [19:0]  pi_param_l1_i14_pi,
    output wire [19:0]  pi_param_l1_i15_pi,
    output wire [11:0]  dcdl_param_l1_i0_dcdl,
    output wire [11:0]  dcdl_param_l1_i1_dcdl,
    output wire [11:0]  dcdl_param_l1_i2_dcdl,
    output wire [11:0]  dcdl_param_l1_i3_dcdl,
    output wire [11:0]  dcdl_param_l1_i4_dcdl,
    output wire [11:0]  dcdl_param_l1_i5_dcdl,
    output wire [11:0]  dcdl_param_l1_i6_dcdl,
    output wire [11:0]  dcdl_param_l1_i7_dcdl,
    output wire [11:0]  dcdl_param_l1_i8_dcdl,
    output wire [11:0]  dcdl_param_l1_i9_dcdl,
    output wire [11:0]  dcdl_param_l1_i10_dcdl,
    output wire [11:0]  dcdl_param_l1_i11_dcdl,
    output wire [11:0]  dcdl_param_l1_i12_dcdl,
    output wire [11:0]  dcdl_param_l1_i13_dcdl,
    output wire [11:0]  dcdl_param_l1_i14_dcdl,
    output wire [11:0]  dcdl_param_l1_i15_dcdl,
    output wire [15:0]  offset_param_l2_i16_offset,
    output wire [11:0]  dcdl_param_l2_i16_dcdl,
    output wire [15:0]  offset_param_l2_i0_offset,
    output wire [15:0]  offset_param_l2_i1_offset,
    output wire [15:0]  offset_param_l2_i2_offset,
    output wire [15:0]  offset_param_l2_i3_offset,
    output wire [15:0]  offset_param_l2_i4_offset,
    output wire [15:0]  offset_param_l2_i5_offset,
    output wire [15:0]  offset_param_l2_i6_offset,
    output wire [15:0]  offset_param_l2_i7_offset,
    output wire [15:0]  offset_param_l2_i8_offset,
    output wire [15:0]  offset_param_l2_i9_offset,
    output wire [15:0]  offset_param_l2_i10_offset,
    output wire [15:0]  offset_param_l2_i11_offset,
    output wire [15:0]  offset_param_l2_i12_offset,
    output wire [15:0]  offset_param_l2_i13_offset,
    output wire [15:0]  offset_param_l2_i14_offset,
    output wire [15:0]  offset_param_l2_i15_offset,
    output wire [15:0]  vref_param_l2_i0_vref,
    output wire [15:0]  vref_param_l2_i1_vref,
    output wire [15:0]  vref_param_l2_i2_vref,
    output wire [15:0]  vref_param_l2_i3_vref,
    output wire [15:0]  vref_param_l2_i4_vref,
    output wire [15:0]  vref_param_l2_i5_vref,
    output wire [15:0]  vref_param_l2_i6_vref,
    output wire [15:0]  vref_param_l2_i7_vref,
    output wire [15:0]  vref_param_l2_i8_vref,
    output wire [15:0]  vref_param_l2_i9_vref,
    output wire [15:0]  vref_param_l2_i10_vref,
    output wire [15:0]  vref_param_l2_i11_vref,
    output wire [15:0]  vref_param_l2_i12_vref,
    output wire [15:0]  vref_param_l2_i13_vref,
    output wire [15:0]  vref_param_l2_i14_vref,
    output wire [15:0]  vref_param_l2_i15_vref,
    output wire [19:0]  pi_param_l2_i0_pi,
    output wire [19:0]  pi_param_l2_i1_pi,
    output wire [19:0]  pi_param_l2_i2_pi,
    output wire [19:0]  pi_param_l2_i3_pi,
    output wire [19:0]  pi_param_l2_i4_pi,
    output wire [19:0]  pi_param_l2_i5_pi,
    output wire [19:0]  pi_param_l2_i6_pi,
    output wire [19:0]  pi_param_l2_i7_pi,
    output wire [19:0]  pi_param_l2_i8_pi,
    output wire [19:0]  pi_param_l2_i9_pi,
    output wire [19:0]  pi_param_l2_i10_pi,
    output wire [19:0]  pi_param_l2_i11_pi,
    output wire [19:0]  pi_param_l2_i12_pi,
    output wire [19:0]  pi_param_l2_i13_pi,
    output wire [19:0]  pi_param_l2_i14_pi,
    output wire [19:0]  pi_param_l2_i15_pi,
    output wire [11:0]  dcdl_param_l2_i0_dcdl,
    output wire [11:0]  dcdl_param_l2_i1_dcdl,
    output wire [11:0]  dcdl_param_l2_i2_dcdl,
    output wire [11:0]  dcdl_param_l2_i3_dcdl,
    output wire [11:0]  dcdl_param_l2_i4_dcdl,
    output wire [11:0]  dcdl_param_l2_i5_dcdl,
    output wire [11:0]  dcdl_param_l2_i6_dcdl,
    output wire [11:0]  dcdl_param_l2_i7_dcdl,
    output wire [11:0]  dcdl_param_l2_i8_dcdl,
    output wire [11:0]  dcdl_param_l2_i9_dcdl,
    output wire [11:0]  dcdl_param_l2_i10_dcdl,
    output wire [11:0]  dcdl_param_l2_i11_dcdl,
    output wire [11:0]  dcdl_param_l2_i12_dcdl,
    output wire [11:0]  dcdl_param_l2_i13_dcdl,
    output wire [11:0]  dcdl_param_l2_i14_dcdl,
    output wire [11:0]  dcdl_param_l2_i15_dcdl,
    output wire [15:0]  offset_param_l3_i16_offset,
    output wire [11:0]  dcdl_param_l3_i16_dcdl,
    output wire [15:0]  offset_param_l3_i0_offset,
    output wire [15:0]  offset_param_l3_i1_offset,
    output wire [15:0]  offset_param_l3_i2_offset,
    output wire [15:0]  offset_param_l3_i3_offset,
    output wire [15:0]  offset_param_l3_i4_offset,
    output wire [15:0]  offset_param_l3_i5_offset,
    output wire [15:0]  offset_param_l3_i6_offset,
    output wire [15:0]  offset_param_l3_i7_offset,
    output wire [15:0]  offset_param_l3_i8_offset,
    output wire [15:0]  offset_param_l3_i9_offset,
    output wire [15:0]  offset_param_l3_i10_offset,
    output wire [15:0]  offset_param_l3_i11_offset,
    output wire [15:0]  offset_param_l3_i12_offset,
    output wire [15:0]  offset_param_l3_i13_offset,
    output wire [15:0]  offset_param_l3_i14_offset,
    output wire [15:0]  offset_param_l3_i15_offset,
    output wire [15:0]  vref_param_l3_i0_vref,
    output wire [15:0]  vref_param_l3_i1_vref,
    output wire [15:0]  vref_param_l3_i2_vref,
    output wire [15:0]  vref_param_l3_i3_vref,
    output wire [15:0]  vref_param_l3_i4_vref,
    output wire [15:0]  vref_param_l3_i5_vref,
    output wire [15:0]  vref_param_l3_i6_vref,
    output wire [15:0]  vref_param_l3_i7_vref,
    output wire [15:0]  vref_param_l3_i8_vref,
    output wire [15:0]  vref_param_l3_i9_vref,
    output wire [15:0]  vref_param_l3_i10_vref,
    output wire [15:0]  vref_param_l3_i11_vref,
    output wire [15:0]  vref_param_l3_i12_vref,
    output wire [15:0]  vref_param_l3_i13_vref,
    output wire [15:0]  vref_param_l3_i14_vref,
    output wire [15:0]  vref_param_l3_i15_vref,
    output wire [19:0]  pi_param_l3_i0_pi,
    output wire [19:0]  pi_param_l3_i1_pi,
    output wire [19:0]  pi_param_l3_i2_pi,
    output wire [19:0]  pi_param_l3_i3_pi,
    output wire [19:0]  pi_param_l3_i4_pi,
    output wire [19:0]  pi_param_l3_i5_pi,
    output wire [19:0]  pi_param_l3_i6_pi,
    output wire [19:0]  pi_param_l3_i7_pi,
    output wire [19:0]  pi_param_l3_i8_pi,
    output wire [19:0]  pi_param_l3_i9_pi,
    output wire [19:0]  pi_param_l3_i10_pi,
    output wire [19:0]  pi_param_l3_i11_pi,
    output wire [19:0]  pi_param_l3_i12_pi,
    output wire [19:0]  pi_param_l3_i13_pi,
    output wire [19:0]  pi_param_l3_i14_pi,
    output wire [19:0]  pi_param_l3_i15_pi,
    output wire [11:0]  dcdl_param_l3_i0_dcdl,
    output wire [11:0]  dcdl_param_l3_i1_dcdl,
    output wire [11:0]  dcdl_param_l3_i2_dcdl,
    output wire [11:0]  dcdl_param_l3_i3_dcdl,
    output wire [11:0]  dcdl_param_l3_i4_dcdl,
    output wire [11:0]  dcdl_param_l3_i5_dcdl,
    output wire [11:0]  dcdl_param_l3_i6_dcdl,
    output wire [11:0]  dcdl_param_l3_i7_dcdl,
    output wire [11:0]  dcdl_param_l3_i8_dcdl,
    output wire [11:0]  dcdl_param_l3_i9_dcdl,
    output wire [11:0]  dcdl_param_l3_i10_dcdl,
    output wire [11:0]  dcdl_param_l3_i11_dcdl,
    output wire [11:0]  dcdl_param_l3_i12_dcdl,
    output wire [11:0]  dcdl_param_l3_i13_dcdl,
    output wire [11:0]  dcdl_param_l3_i14_dcdl,
    output wire [11:0]  dcdl_param_l3_i15_dcdl,
    // 族 c #3/#4/#7 → lane_top→rx_chain→rx_training→train（hook_min: TRAIN_PARAM_SRC_i / FAIL_THRESHOLD_i / CTLE_TAP_i）
    output wire         train_param_src_o,
    output wire [7:0]   datacheck_cfg_fail_threshold_o,
    output wire [3:0]   eq_param_ctle_tap_o,
    // ------------------------------------------------------------------
    // 0xD00+lane*0xC0 模拟 PHY 控制寄存器段（intf logicphy_analogphy_intf v10.3, P0-B）
    //   输出: per-lane packed WRITE_IN 总线（35*32b, 位序=模型端口序: [31:0]=TRX_REG1）
    //         + per-lane TRX_EN（模型 TRX CLK SYNC 组）
    //   输入: READ_OUT / CONNECT_DIG / ERR_EDGE（RO 组合直读, 直连 rf RO 输入）
    // ------------------------------------------------------------------
    output wire [1423:0] phy_ctrl_writein_l0_o,
    output wire [1423:0] phy_ctrl_writein_l1_o,
    output wire [1423:0] phy_ctrl_writein_l2_o,
    output wire [1423:0] phy_ctrl_writein_l3_o,
    output wire [7:0]    phy_trx_en_l0_o,     // [0]tx_en_async [1]tx_en_sync [2]tx_en_track [3]tx_rstb_sync
    output wire [7:0]    phy_trx_en_l1_o,     //                      [4]rx_en_async [5]rx_en_sync [6]rx_en_track [7]rx_rstb_sync
    output wire [7:0]    phy_trx_en_l2_o,
    output wire [7:0]    phy_trx_en_l3_o,
    input  wire [31:0]   phy_rx_clk_eng_readout_l0_i,
    input  wire [31:0]   phy_rx_clk_eng_readout_l1_i,
    input  wire [31:0]   phy_rx_clk_eng_readout_l2_i,
    input  wire [31:0]   phy_rx_clk_eng_readout_l3_i,
    input  wire [31:0]   phy_tx_clk_eng_readout_l0_i,
    input  wire [31:0]   phy_tx_clk_eng_readout_l1_i,
    input  wire [31:0]   phy_tx_clk_eng_readout_l2_i,
    input  wire [31:0]   phy_tx_clk_eng_readout_l3_i,
    input  wire          phy_connect_dig_l0_i,
    input  wire          phy_connect_dig_l1_i,
    input  wire          phy_connect_dig_l2_i,
    input  wire          phy_connect_dig_l3_i,
    input  wire [67:0]   phy_err_edge_l0_i,   // [4n+3:4n]=DOUTn_ERR_EDGE (n=0..15), [67:64]=DOUTRFU_ERR_EDGE
    input  wire [67:0]   phy_err_edge_l1_i,
    input  wire [67:0]   phy_err_edge_l2_i,
    input  wire [67:0]   phy_err_edge_l3_i,
    // SBC 边带冗余（0x828 SBC_REDUNDANCY, intf v10.3 P1: MUX 切换, 主路数据隔离）
    output wire          sbc_txr_en_o,
    output wire          sbc_rxr_sel_o
);

    // ==========================================================================
    // APB 访问控制（与原 RTL 一致）
    // ==========================================================================
    wire        apb_wr_en;
    wire        apb_rd_en;
    wire        addr_aligned;
    reg         addr_valid;
    reg [31:0]  prdata_mux;
    reg         pslverr_cond;
    reg         per_lane_off_valid;

    // rf 总线（addr_valid 门控: 0x1000+ 地址 12-bit 截断不触达 per-lane 存储）
    //   0x004 IO_DATA_WIDTH_MODE 为 RWS（bring-up strap 锁存，启动后只读）：rf 侧该字段
    //   无写路径（MODE 为 rf 输出端口），运行时写必须连带屏蔽 rf 写使能，否则 rf 内部
    //   io_data_width_mode_mode 会被 reg_wdata[0] 改写（双驱 + 语义越权）。
    wire        rws_block_wr = apb_pwrite && (apb_paddr == 16'h0004);
    wire        rf_reg_wr  = apb_wr_en & addr_valid & ~rws_block_wr;
    wire        rf_reg_rd  = apb_rd_en & addr_valid;
    wire [31:0] rf_reg_rdata;

    // per-lane 解码辅助（块基址 0x100 + lane×0x200，块内偏移 = apb_paddr[8:0]）
    wire        per_lane_hit = (apb_paddr >= 16'h100) && (apb_paddr <= 16'h8FF);
    wire [2:0]  per_lane_idx = apb_paddr[11:9];
    wire [8:0]  blk_off      = apb_paddr[8:0];
    wire        lane_valid   = (per_lane_idx < LANE_NUM[2:0]);

    assign apb_wr_en  = apb_psel & apb_penable & apb_pwrite;
    assign apb_rd_en  = apb_psel & apb_penable & ~apb_pwrite;
    assign apb_pready = 1'b1;

    assign addr_aligned = (apb_paddr[1:0] == 2'b00);

    // 块内偏移映射检查（与原 RTL 一致; 各基址低 6 bit 恒 0, 直接用 blk_off[5:2]）
    always @(*) begin
        per_lane_off_valid = 1'b0;
        if (blk_off == 9'h000 || blk_off == 9'h004 ||
            blk_off == 9'h0C0 || blk_off == 9'h0C4 || blk_off == 9'h0C8) begin
            per_lane_off_valid = 1'b1;
        end
        else if ((blk_off >= 9'h040 && blk_off < 9'h080) ||
                 (blk_off >= 9'h080 && blk_off < 9'h0C0) ||
                 (blk_off >= 9'h100 && blk_off < 9'h140) ||
                 (blk_off >= 9'h140 && blk_off < 9'h180) ||
                 (blk_off >= 9'h180 && blk_off < 9'h1C0) ||
                 (blk_off >= 9'h1C0)) begin
            per_lane_off_valid = 1'b1;
        end
    end

    // 地址映射检查：绝对地址段 + per-lane 整块（与原 RTL 一致; 绝对段 case 优先）
    always @(*) begin
        addr_valid = 1'b0;
        case (apb_paddr)
            16'h0000, 16'h0004, 16'h0008, 16'h000C,
            16'h0010, 16'h0014, 16'h0018, 16'h001C,
            16'h0020, 16'h0024, 16'h0028, 16'h00F0,
            16'h0400, 16'h0404, 16'h0408, 16'h040C,
            16'h0410, 16'h0414, 16'h0418, 16'h041C,
            16'h0420, 16'h0424, 16'h0428, 16'h042C,
            16'h0430, 16'h0434, 16'h0438, 16'h043C,
            16'h0440, 16'h0444, 16'h0448,
            16'h0700, 16'h0704, 16'h0708,
            16'h0800, 16'h0804, 16'h0808, 16'h080C,
            16'h0810, 16'h0814, 16'h0818, 16'h081C,
            16'h0820, 16'h0824,
            16'h0900, 16'h0904, 16'h0908, 16'h090C, 16'h0910,
            16'h0A00,
            16'h0C00, 16'h0C04, 16'h0C08, 16'h0C0C: begin
                addr_valid = 1'b1;
            end
            default: begin
                if (per_lane_hit && lane_valid && per_lane_off_valid) begin
                    addr_valid = 1'b1;
                end
            end
        endcase
    end

    // ==========================================================================
    // wrapper 自持状态（sticky / 镜像 / 自恢复 / 同步, 与原 RTL 一致）
    // ==========================================================================
    // int mask 镜像（0x00C/0x900、0x010/0x904 双入口写同一存储; 复位全屏蔽）
    reg [6:0]   int_mask_mcu_reg;
    reg [6:0]   int_mask_ext_reg;
    reg int_mask_audit_fl;
    // INT_STATUS sticky（set = int_raw, clear = INT_CLEAR 写 1, 置位优先）
    reg [6:0]   int_status_reg;
    // CDC FIFO ovf/unf sticky（17 bit; set 源 TODO tie 0, clear = 写 1）
    reg [16:0]  cdc_fifo_ovf_reg;
    reg [16:0]  cdc_fifo_unf_reg;
    // SBC 状态 sticky（set = 同步回流, clear = 对应地址写 1, 置位优先）
    reg         sbc_tx_err_sticky;
    reg         sbc_rx_parity_err_sticky;
    reg         sbc_rx_gap_err_sticky;
    reg         sbc_rx_buf_ovf_sticky;
    // RX 异步 FIFO ovf/unf sticky（per-lane 4 bit, set = 同步回流, clear = 写 1）
    reg [3:0]   fifo_ovf_sticky;
    reg [3:0]   fifo_unf_sticky;
    // TOP_CTRL.global_rst_n 自恢复（写 0 持续 1 cycle 后自动恢复 1; rf 副本无此行为,
    // 读回 0x000[4] 用本寄存器 override）
    reg         global_rst_n_reg;
    // SBC_RX_BUF 读触发 rx_manual_read 脉冲（读 0x810 弹出; 与 rf W1S 写脉冲 OR）
    reg         sbc_rx_rd_pulse;
    // lane2 data_path 影子 alias 寄存器（0x400/0x404/0x440/0x444/0x448 与共享段双写:
    // 读返共享段（rf 主寄存器）, 写同时更新本 alias（原 RTL per-lane 写解码行为）,
    // 驱 lane2 TX/RX 数据通路输出, 见设计说明 3）
    reg         l2_reversal_en_tx;
    reg         l2_reversal_en_rx;
    reg [15:0]  l2_repair_map_tx;
    reg         l2_scramble_en_tx;
    reg [15:0]  l2_repair_map_rx;
    reg         l2_scramble_en_rx;
    reg [22:0]  l2_seed_tx_i0;
    reg [22:0]  l2_seed_tx_i1;
    reg [22:0]  l2_seed_tx_i2;

    // 状态回流 2 flop 同步器（源域 → apb_pclk，per-bit 独立; 与原 RTL 一致）
    reg [15:0]  aligned_rx_sync1;
    reg [15:0]  aligned_rx_sync2;
    reg [31:0]  rx_fifo_status_sync1;
    reg [31:0]  rx_fifo_status_sync2;
    reg [15:0]  lane_irq_sync1;
    reg [15:0]  lane_irq_sync2;
    reg [3:0]   sideband_irq_sync1;
    reg [3:0]   sideband_irq_sync2;
    reg [4:0]   sbc_tx_stat_sync1;
    reg [4:0]   sbc_tx_stat_sync2;
    reg [7:0]   sbc_rx_stat_sync1;
    reg [7:0]   sbc_rx_stat_sync2;
    reg [15:0]  sbc_rx_rdata_sync1;
    reg [15:0]  sbc_rx_rdata_sync2;
    reg [3:0]   sbc_disp_sync1;
    reg [3:0]   sbc_disp_sync2;

    // 中断聚合
    wire [6:0]  int_raw;
    // INT_STATUS 清位源（族 b: 改 rf int_clear/intc_clear wp/wd 单驱, 原自持 int_clear_wen 已删）

    // IO_DATA_WIDTH_MODE[0] 本体 = bring-up strap 锁存值（apb_csr_top_mas.md:75/:187
    //   「bring-up strap 锁存…启动后只读，运行时写触发 pslverr」）。
    //   实现：rf 侧该字段 Access=RO（MODE 为 rf 输入端口，无写路径），本体唯一写者
    //   是本 wrapper 的复位后一次性采样 —— 复位期 flop 归 0（异步复位值须常量，不可
    //   接 strap 信号），复位释放后第 1 个 apb_pclk 沿把静态 strap 采进来，此后永不
    //   更新。满足 bring-up 语义：strap 为上电静态引脚，复位释放首拍采样即等于上电
    //   稳定值；且早于任何 APB 访问（APB master 同由该复位释放驱动），读到的一直是
    //   锁存值；运行期写 0x004 由 pslverr_cond 报错且寄存器不更新（RWS）。
    //   派生只读字段联动：analog_ser_ratio([15:14]) = 0→2'b00 / 1→2'b01；
    //   PRBS_LFSR_CLK_GATE[0](0x008) = ~mode。
    // 注意: 须在 u_rf 例化之前声明 —— 例化端口列表引用派生 wire, 若声明在例化后,
    //       VCS 按隐式 1-bit net 解析, 连接 2-bit rf 端口时高位悬空 Z（PCWM-W）。
    reg         io_data_width_mode_reg;     // strap 锁存本体（bring-up，启动后只读）
    reg         io_data_width_mode_latched; // 一次性采样标志
    wire        w_io_mode_rf;               // rf 侧 MODE 输出（写路径被 rws_block_wr 挡死 + 复位 0 ⇒ 恒 0）
    // w_io_mode_rf 参与 OR 仅为其被消费（消 W528 set-but-not-read）：rf 侧 0x004 写路径已由
    //   rws_block_wr 屏蔽、复位值 0 ⇒ 恒 0 ⇒ reg | 0 = reg，strap 锁存语义不变。
    wire        w_io_mode       = io_data_width_mode_reg | w_io_mode_rf;
    wire [1:0]  w_ser_ratio     = w_io_mode ? 2'b01 : 2'b00;
    wire        w_prbs_clk_gate = ~w_io_mode;

    integer ii;
    integer fi;
    integer ci;
    // ==========================================================================
    // 生成物 wiring: apb_csr_top_rf 端口 -> wrapper 信号（gen_mech.py 展开, 勿手改）
    // ==========================================================================
    wire[ 1:0] w_train_mode;
    wire[ 6:0] w_step_force_end;
    wire[ 2:0] w_step_cnt;
    wire        w_pattern_sel;
    wire[ 6:0] w_bypass_step;
    wire[ 6:0] w_timeout_step_en;
    wire[15:0] w_timeout_threshold;
    wire[31:0] w_tp_vref_lo;
    wire[31:0] w_tp_vref_hi;
    wire[31:0] w_tp_pi_lo;
    wire[31:0] w_tp_pi_hi;
    wire[31:0] w_tp_dcdl_lo;
    wire[31:0] w_tp_dcdl_hi;
    wire[31:0] w_tp_reorder_lo;
    wire[31:0] w_tp_reorder_hi;
    wire[31:0] w_tp_dc_lo;
    wire[31:0] w_tp_dc_hi;
    wire[ 3:0] w_delay_step;
    wire        w_force_mode_switch;
    wire        w_seed_load_rx;
    wire        w_clear_rx;
    wire        w_seed_load_tx;
    wire        w_clear_tx;
    wire        w_step_trigger;
    wire        w_sbc_tx_trig;
    wire        w_sbc_rx_manual_read;
    wire        w_sbc_rx_soft_rst;
    wire        w_sbc_tx_enable;
    wire[15:0] w_sbc_tx_pkt_inject;
    wire        w_sbc_rx_enable;
    wire[ 7:0] w_sbc_rx_buf_depth;
    wire[15:0] w_sbc_lid_filter;
    wire[ 7:0] w_sbc_opcode_filter;
    wire[ 7:0] w_sbc_err_mask;
    wire        w_sbc_pkt_arrival_mask;
    wire        w_sbc_err_irq_mask;
    wire[ 3:0] w_sbc_lane_enable;
    wire        w_mcu_soft_rst_n;
    wire[ 3:0] w_mcu_boot_addr;
    wire        w_mcu_itcm_wr_en;
    wire        w_dpc_l1_rev_tx;
    wire        w_dpc_l1_rev_rx;
    wire[15:0] w_dpc_l1_repair_tx;
    wire        w_dpc_l1_scram_tx;
    wire[15:0] w_dpcr_l1_repair_rx;
    wire        w_dpcr_l1_scram_rx;
    wire[22:0] w_seed_tx_l1_i0;
    wire[22:0] w_seed_tx_l1_i1;
    wire[22:0] w_seed_tx_l1_i2;
    wire[22:0] w_seed_tx_l1_i3;
    wire[22:0] w_seed_tx_l1_i4;
    wire[22:0] w_seed_tx_l1_i5;
    wire[22:0] w_seed_tx_l1_i6;
    wire[22:0] w_seed_tx_l1_i7;
    wire[22:0] w_seed_tx_l1_i8;
    wire[22:0] w_seed_tx_l1_i9;
    wire[22:0] w_seed_tx_l1_i10;
    wire[22:0] w_seed_tx_l1_i11;
    wire[22:0] w_seed_tx_l1_i12;
    wire[22:0] w_seed_tx_l1_i13;
    wire[22:0] w_seed_tx_l1_i14;
    wire[22:0] w_seed_tx_l1_i15;
    wire[22:0] w_seed_rx_l1_i0;
    wire[22:0] w_seed_rx_l1_i1;
    wire[22:0] w_seed_rx_l1_i2;
    wire[22:0] w_seed_rx_l1_i3;
    wire[22:0] w_seed_rx_l1_i4;
    wire[22:0] w_seed_rx_l1_i5;
    wire[22:0] w_seed_rx_l1_i6;
    wire[22:0] w_seed_rx_l1_i7;
    wire[22:0] w_seed_rx_l1_i8;
    wire[22:0] w_seed_rx_l1_i9;
    wire[22:0] w_seed_rx_l1_i10;
    wire[22:0] w_seed_rx_l1_i11;
    wire[22:0] w_seed_rx_l1_i12;
    wire[22:0] w_seed_rx_l1_i13;
    wire[22:0] w_seed_rx_l1_i14;
    wire[22:0] w_seed_rx_l1_i15;
    wire[ 3:0] w_flip_tx_l1;
    wire[ 3:0] w_flip_rx_l1;
    wire[16:0] w_msb_tx_l1;
    wire[16:0] w_msb_rx_l1;
    wire[22:0] w_seed_tx_l2_i3;
    wire[22:0] w_seed_tx_l2_i4;
    wire[22:0] w_seed_tx_l2_i5;
    wire[22:0] w_seed_tx_l2_i6;
    wire[22:0] w_seed_tx_l2_i7;
    wire[22:0] w_seed_tx_l2_i8;
    wire[22:0] w_seed_tx_l2_i9;
    wire[22:0] w_seed_tx_l2_i10;
    wire[22:0] w_seed_tx_l2_i11;
    wire[22:0] w_seed_tx_l2_i12;
    wire[22:0] w_seed_tx_l2_i13;
    wire[22:0] w_seed_tx_l2_i14;
    wire[22:0] w_seed_tx_l2_i15;
    wire[22:0] w_seed_rx_l2_i0;
    wire[22:0] w_seed_rx_l2_i1;
    wire[22:0] w_seed_rx_l2_i2;
    wire[22:0] w_seed_rx_l2_i3;
    wire[22:0] w_seed_rx_l2_i4;
    wire[22:0] w_seed_rx_l2_i5;
    wire[22:0] w_seed_rx_l2_i6;
    wire[22:0] w_seed_rx_l2_i7;
    wire[22:0] w_seed_rx_l2_i8;
    wire[22:0] w_seed_rx_l2_i9;
    wire[22:0] w_seed_rx_l2_i10;
    wire[22:0] w_seed_rx_l2_i11;
    wire[22:0] w_seed_rx_l2_i12;
    wire[22:0] w_seed_rx_l2_i13;
    wire[22:0] w_seed_rx_l2_i14;
    wire[22:0] w_seed_rx_l2_i15;
    wire[ 3:0] w_flip_tx_l2;
    wire[ 3:0] w_flip_rx_l2;
    wire[16:0] w_msb_tx_l2;
    wire[16:0] w_msb_rx_l2;
    wire        w_dpc_l3_rev_tx;
    wire        w_dpc_l3_rev_rx;
    wire[15:0] w_dpc_l3_repair_tx;
    wire        w_dpc_l3_scram_tx;
    wire[15:0] w_dpcr_l3_repair_rx;
    wire        w_dpcr_l3_scram_rx;
    wire[22:0] w_seed_tx_l3_i0;
    wire[22:0] w_seed_tx_l3_i1;
    wire[22:0] w_seed_tx_l3_i2;
    wire[22:0] w_seed_tx_l3_i3;
    wire[22:0] w_seed_tx_l3_i4;
    wire[22:0] w_seed_tx_l3_i5;
    wire[22:0] w_seed_tx_l3_i6;
    wire[22:0] w_seed_tx_l3_i7;
    wire[22:0] w_seed_tx_l3_i8;
    wire[22:0] w_seed_tx_l3_i9;
    wire[22:0] w_seed_tx_l3_i10;
    wire[22:0] w_seed_tx_l3_i11;
    wire[22:0] w_seed_tx_l3_i12;
    wire[22:0] w_seed_tx_l3_i13;
    wire[22:0] w_seed_tx_l3_i14;
    wire[22:0] w_seed_tx_l3_i15;
    wire[22:0] w_seed_rx_l3_i0;
    wire[22:0] w_seed_rx_l3_i1;
    wire[22:0] w_seed_rx_l3_i2;
    wire[22:0] w_seed_rx_l3_i3;
    wire[22:0] w_seed_rx_l3_i4;
    wire[22:0] w_seed_rx_l3_i5;
    wire[22:0] w_seed_rx_l3_i6;
    wire[22:0] w_seed_rx_l3_i7;
    wire[22:0] w_seed_rx_l3_i8;
    wire[22:0] w_seed_rx_l3_i9;
    wire[22:0] w_seed_rx_l3_i10;
    wire[22:0] w_seed_rx_l3_i11;
    wire[22:0] w_seed_rx_l3_i12;
    wire[22:0] w_seed_rx_l3_i13;
    wire[22:0] w_seed_rx_l3_i14;
    wire[22:0] w_seed_rx_l3_i15;
    wire[ 3:0] w_flip_tx_l3;
    wire[ 3:0] w_flip_rx_l3;
    wire[16:0] w_msb_tx_l3;
    wire[16:0] w_msb_rx_l3;
    wire[15:0] w_offset_param_l0_i0;
    wire[15:0] w_vref_param_l0_i0;
    wire[19:0] w_pi_param_l0_i0;
    wire[11:0] w_dcdl_param_l0_i0;
    wire[15:0] w_offset_param_l0_i1;
    wire[15:0] w_vref_param_l0_i1;
    wire[19:0] w_pi_param_l0_i1;
    wire[11:0] w_dcdl_param_l0_i1;
    wire[15:0] w_offset_param_l0_i2;
    wire[15:0] w_vref_param_l0_i2;
    wire[19:0] w_pi_param_l0_i2;
    wire[11:0] w_dcdl_param_l0_i2;
    wire[15:0] w_offset_param_l0_i3;
    wire[15:0] w_vref_param_l0_i3;
    wire[19:0] w_pi_param_l0_i3;
    wire[11:0] w_dcdl_param_l0_i3;
    wire[15:0] w_offset_param_l0_i4;
    wire[15:0] w_vref_param_l0_i4;
    wire[19:0] w_pi_param_l0_i4;
    wire[11:0] w_dcdl_param_l0_i4;
    wire[15:0] w_offset_param_l0_i5;
    wire[15:0] w_vref_param_l0_i5;
    wire[19:0] w_pi_param_l0_i5;
    wire[11:0] w_dcdl_param_l0_i5;
    wire[15:0] w_offset_param_l0_i6;
    wire[15:0] w_vref_param_l0_i6;
    wire[19:0] w_pi_param_l0_i6;
    wire[11:0] w_dcdl_param_l0_i6;
    wire[15:0] w_offset_param_l0_i7;
    wire[15:0] w_vref_param_l0_i7;
    wire[19:0] w_pi_param_l0_i7;
    wire[11:0] w_dcdl_param_l0_i7;
    wire[15:0] w_offset_param_l0_i8;
    wire[15:0] w_vref_param_l0_i8;
    wire[19:0] w_pi_param_l0_i8;
    wire[11:0] w_dcdl_param_l0_i8;
    wire[15:0] w_offset_param_l0_i9;
    wire[15:0] w_vref_param_l0_i9;
    wire[19:0] w_pi_param_l0_i9;
    wire[11:0] w_dcdl_param_l0_i9;
    wire[15:0] w_offset_param_l0_i10;
    wire[15:0] w_vref_param_l0_i10;
    wire[19:0] w_pi_param_l0_i10;
    wire[11:0] w_dcdl_param_l0_i10;
    wire[15:0] w_offset_param_l0_i11;
    wire[15:0] w_vref_param_l0_i11;
    wire[19:0] w_pi_param_l0_i11;
    wire[11:0] w_dcdl_param_l0_i11;
    wire[15:0] w_offset_param_l0_i12;
    wire[15:0] w_vref_param_l0_i12;
    wire[19:0] w_pi_param_l0_i12;
    wire[11:0] w_dcdl_param_l0_i12;
    wire[15:0] w_offset_param_l0_i13;
    wire[15:0] w_vref_param_l0_i13;
    wire[19:0] w_pi_param_l0_i13;
    wire[11:0] w_dcdl_param_l0_i13;
    wire[15:0] w_offset_param_l0_i14;
    wire[15:0] w_vref_param_l0_i14;
    wire[19:0] w_pi_param_l0_i14;
    wire[11:0] w_dcdl_param_l0_i14;
    wire[15:0] w_offset_param_l0_i15;
    wire[15:0] w_vref_param_l0_i15;
    wire[19:0] w_pi_param_l0_i15;
    wire[11:0] w_dcdl_param_l0_i15;
    wire[15:0] w_offset_param_l0_i16;
    wire[11:0] w_dcdl_param_l0_i16;
    // MDE-CHAIN-1008 lane1 train 内部 wire（2026-10-12 接口贯通）
    wire[15:0] w_offset_param_l1_i0;
    wire[15:0] w_offset_param_l1_i1;
    wire[15:0] w_offset_param_l1_i2;
    wire[15:0] w_offset_param_l1_i3;
    wire[15:0] w_offset_param_l1_i4;
    wire[15:0] w_offset_param_l1_i5;
    wire[15:0] w_offset_param_l1_i6;
    wire[15:0] w_offset_param_l1_i7;
    wire[15:0] w_offset_param_l1_i8;
    wire[15:0] w_offset_param_l1_i9;
    wire[15:0] w_offset_param_l1_i10;
    wire[15:0] w_offset_param_l1_i11;
    wire[15:0] w_offset_param_l1_i12;
    wire[15:0] w_offset_param_l1_i13;
    wire[15:0] w_offset_param_l1_i14;
    wire[15:0] w_offset_param_l1_i15;
    wire[15:0] w_offset_param_l1_i16;
    wire[19:0] w_pi_param_l1_i0;
    wire[19:0] w_pi_param_l1_i1;
    wire[19:0] w_pi_param_l1_i2;
    wire[19:0] w_pi_param_l1_i3;
    wire[19:0] w_pi_param_l1_i4;
    wire[19:0] w_pi_param_l1_i5;
    wire[19:0] w_pi_param_l1_i6;
    wire[19:0] w_pi_param_l1_i7;
    wire[19:0] w_pi_param_l1_i8;
    wire[19:0] w_pi_param_l1_i9;
    wire[19:0] w_pi_param_l1_i10;
    wire[19:0] w_pi_param_l1_i11;
    wire[19:0] w_pi_param_l1_i12;
    wire[19:0] w_pi_param_l1_i13;
    wire[19:0] w_pi_param_l1_i14;
    wire[19:0] w_pi_param_l1_i15;
    wire[15:0] w_vref_param_l1_i0;
    wire[15:0] w_vref_param_l1_i1;
    wire[15:0] w_vref_param_l1_i2;
    wire[15:0] w_vref_param_l1_i3;
    wire[15:0] w_vref_param_l1_i4;
    wire[15:0] w_vref_param_l1_i5;
    wire[15:0] w_vref_param_l1_i6;
    wire[15:0] w_vref_param_l1_i7;
    wire[15:0] w_vref_param_l1_i8;
    wire[15:0] w_vref_param_l1_i9;
    wire[15:0] w_vref_param_l1_i10;
    wire[15:0] w_vref_param_l1_i11;
    wire[15:0] w_vref_param_l1_i12;
    wire[15:0] w_vref_param_l1_i13;
    wire[15:0] w_vref_param_l1_i14;
    wire[15:0] w_vref_param_l1_i15;
    wire[11:0] w_dcdl_param_l1_i0;
    wire[11:0] w_dcdl_param_l1_i1;
    wire[11:0] w_dcdl_param_l1_i2;
    wire[11:0] w_dcdl_param_l1_i3;
    wire[11:0] w_dcdl_param_l1_i4;
    wire[11:0] w_dcdl_param_l1_i5;
    wire[11:0] w_dcdl_param_l1_i6;
    wire[11:0] w_dcdl_param_l1_i7;
    wire[11:0] w_dcdl_param_l1_i8;
    wire[11:0] w_dcdl_param_l1_i9;
    wire[11:0] w_dcdl_param_l1_i10;
    wire[11:0] w_dcdl_param_l1_i11;
    wire[11:0] w_dcdl_param_l1_i12;
    wire[11:0] w_dcdl_param_l1_i13;
    wire[11:0] w_dcdl_param_l1_i14;
    wire[11:0] w_dcdl_param_l1_i15;
    wire[11:0] w_dcdl_param_l1_i16;
    // MDE-CHAIN-1008 lane2 train 内部 wire（2026-10-12 接口贯通）
    wire[15:0] w_offset_param_l2_i0;
    wire[15:0] w_offset_param_l2_i1;
    wire[15:0] w_offset_param_l2_i2;
    wire[15:0] w_offset_param_l2_i3;
    wire[15:0] w_offset_param_l2_i4;
    wire[15:0] w_offset_param_l2_i5;
    wire[15:0] w_offset_param_l2_i6;
    wire[15:0] w_offset_param_l2_i7;
    wire[15:0] w_offset_param_l2_i8;
    wire[15:0] w_offset_param_l2_i9;
    wire[15:0] w_offset_param_l2_i10;
    wire[15:0] w_offset_param_l2_i11;
    wire[15:0] w_offset_param_l2_i12;
    wire[15:0] w_offset_param_l2_i13;
    wire[15:0] w_offset_param_l2_i14;
    wire[15:0] w_offset_param_l2_i15;
    wire[15:0] w_offset_param_l2_i16;
    wire[19:0] w_pi_param_l2_i0;
    wire[19:0] w_pi_param_l2_i1;
    wire[19:0] w_pi_param_l2_i2;
    wire[19:0] w_pi_param_l2_i3;
    wire[19:0] w_pi_param_l2_i4;
    wire[19:0] w_pi_param_l2_i5;
    wire[19:0] w_pi_param_l2_i6;
    wire[19:0] w_pi_param_l2_i7;
    wire[19:0] w_pi_param_l2_i8;
    wire[19:0] w_pi_param_l2_i9;
    wire[19:0] w_pi_param_l2_i10;
    wire[19:0] w_pi_param_l2_i11;
    wire[19:0] w_pi_param_l2_i12;
    wire[19:0] w_pi_param_l2_i13;
    wire[19:0] w_pi_param_l2_i14;
    wire[19:0] w_pi_param_l2_i15;
    wire[15:0] w_vref_param_l2_i0;
    wire[15:0] w_vref_param_l2_i1;
    wire[15:0] w_vref_param_l2_i2;
    wire[15:0] w_vref_param_l2_i3;
    wire[15:0] w_vref_param_l2_i4;
    wire[15:0] w_vref_param_l2_i5;
    wire[15:0] w_vref_param_l2_i6;
    wire[15:0] w_vref_param_l2_i7;
    wire[15:0] w_vref_param_l2_i8;
    wire[15:0] w_vref_param_l2_i9;
    wire[15:0] w_vref_param_l2_i10;
    wire[15:0] w_vref_param_l2_i11;
    wire[15:0] w_vref_param_l2_i12;
    wire[15:0] w_vref_param_l2_i13;
    wire[15:0] w_vref_param_l2_i14;
    wire[15:0] w_vref_param_l2_i15;
    wire[11:0] w_dcdl_param_l2_i0;
    wire[11:0] w_dcdl_param_l2_i1;
    wire[11:0] w_dcdl_param_l2_i2;
    wire[11:0] w_dcdl_param_l2_i3;
    wire[11:0] w_dcdl_param_l2_i4;
    wire[11:0] w_dcdl_param_l2_i5;
    wire[11:0] w_dcdl_param_l2_i6;
    wire[11:0] w_dcdl_param_l2_i7;
    wire[11:0] w_dcdl_param_l2_i8;
    wire[11:0] w_dcdl_param_l2_i9;
    wire[11:0] w_dcdl_param_l2_i10;
    wire[11:0] w_dcdl_param_l2_i11;
    wire[11:0] w_dcdl_param_l2_i12;
    wire[11:0] w_dcdl_param_l2_i13;
    wire[11:0] w_dcdl_param_l2_i14;
    wire[11:0] w_dcdl_param_l2_i15;
    wire[11:0] w_dcdl_param_l2_i16;
    // MDE-CHAIN-1008 lane3 train 内部 wire（2026-10-12 接口贯通）
    wire[15:0] w_offset_param_l3_i0;
    wire[15:0] w_offset_param_l3_i1;
    wire[15:0] w_offset_param_l3_i2;
    wire[15:0] w_offset_param_l3_i3;
    wire[15:0] w_offset_param_l3_i4;
    wire[15:0] w_offset_param_l3_i5;
    wire[15:0] w_offset_param_l3_i6;
    wire[15:0] w_offset_param_l3_i7;
    wire[15:0] w_offset_param_l3_i8;
    wire[15:0] w_offset_param_l3_i9;
    wire[15:0] w_offset_param_l3_i10;
    wire[15:0] w_offset_param_l3_i11;
    wire[15:0] w_offset_param_l3_i12;
    wire[15:0] w_offset_param_l3_i13;
    wire[15:0] w_offset_param_l3_i14;
    wire[15:0] w_offset_param_l3_i15;
    wire[15:0] w_offset_param_l3_i16;
    wire[19:0] w_pi_param_l3_i0;
    wire[19:0] w_pi_param_l3_i1;
    wire[19:0] w_pi_param_l3_i2;
    wire[19:0] w_pi_param_l3_i3;
    wire[19:0] w_pi_param_l3_i4;
    wire[19:0] w_pi_param_l3_i5;
    wire[19:0] w_pi_param_l3_i6;
    wire[19:0] w_pi_param_l3_i7;
    wire[19:0] w_pi_param_l3_i8;
    wire[19:0] w_pi_param_l3_i9;
    wire[19:0] w_pi_param_l3_i10;
    wire[19:0] w_pi_param_l3_i11;
    wire[19:0] w_pi_param_l3_i12;
    wire[19:0] w_pi_param_l3_i13;
    wire[19:0] w_pi_param_l3_i14;
    wire[19:0] w_pi_param_l3_i15;
    wire[15:0] w_vref_param_l3_i0;
    wire[15:0] w_vref_param_l3_i1;
    wire[15:0] w_vref_param_l3_i2;
    wire[15:0] w_vref_param_l3_i3;
    wire[15:0] w_vref_param_l3_i4;
    wire[15:0] w_vref_param_l3_i5;
    wire[15:0] w_vref_param_l3_i6;
    wire[15:0] w_vref_param_l3_i7;
    wire[15:0] w_vref_param_l3_i8;
    wire[15:0] w_vref_param_l3_i9;
    wire[15:0] w_vref_param_l3_i10;
    wire[15:0] w_vref_param_l3_i11;
    wire[15:0] w_vref_param_l3_i12;
    wire[15:0] w_vref_param_l3_i13;
    wire[15:0] w_vref_param_l3_i14;
    wire[15:0] w_vref_param_l3_i15;
    wire[11:0] w_dcdl_param_l3_i0;
    wire[11:0] w_dcdl_param_l3_i1;
    wire[11:0] w_dcdl_param_l3_i2;
    wire[11:0] w_dcdl_param_l3_i3;
    wire[11:0] w_dcdl_param_l3_i4;
    wire[11:0] w_dcdl_param_l3_i5;
    wire[11:0] w_dcdl_param_l3_i6;
    wire[11:0] w_dcdl_param_l3_i7;
    wire[11:0] w_dcdl_param_l3_i8;
    wire[11:0] w_dcdl_param_l3_i9;
    wire[11:0] w_dcdl_param_l3_i10;
    wire[11:0] w_dcdl_param_l3_i11;
    wire[11:0] w_dcdl_param_l3_i12;
    wire[11:0] w_dcdl_param_l3_i13;
    wire[11:0] w_dcdl_param_l3_i14;
    wire[11:0] w_dcdl_param_l3_i15;
    wire[11:0] w_dcdl_param_l3_i16;
    wire        w_int_clear_wp;
    wire[ 6:0] w_int_clear_wd;
    wire        w_intc_clear_wp;
    wire[ 6:0] w_intc_clear_wd;
    wire        w_sbc_pkt_arr_flag_wp;
    wire        w_sbc_pkt_arr_flag_wd;
    wire        w_sbc_err_flag_wp;
    wire        w_sbc_err_flag_wd;
    wire        w_top_ctrl_global_rst_n;
    wire        w_cdc_fifo_rst_wp;
    wire        w_train_param_src_o;
    wire[ 7:0] w_datacheck_fail_threshold_o;
    wire[ 3:0] w_eq_param_ctle_tap_o;
    wire[ 1:0] w_train_mode_mode;
    wire[ 6:0] w_int_mask_mcu_mask_mcu;
    wire[ 6:0] w_int_mask_ext_mask_ext;
    wire[ 7:0] w_phy_trx_en_l0;
    wire[ 7:0] w_phy_trx_en_l1;
    wire[ 7:0] w_phy_trx_en_l2;
    wire[ 7:0] w_phy_trx_en_l3;
    wire        w_sbc_txr_en;
    wire        w_sbc_rxr_sel;
    wire[31:0] w_phy_trx_reg1_l0;
    wire[31:0] w_phy_trx_reg2_l0;
    wire[31:0] w_phy_trx_test_reg1_l0;
    wire[31:0] w_phy_trx_test_reg2_l0;
    wire[31:0] w_phy_rx_clk_eng_reg1_l0;
    wire[31:0] w_phy_rx_clk_eng_reg2_l0;
    wire[31:0] w_phy_rx_clk_eng_reg3_l0;
    wire[31:0] w_phy_rx_clk_eng_reg4_l0;
    wire[31:0] w_phy_rx_clk_reg1_l0;
    wire[31:0] w_phy_rx_clk_reg2_l0;
    wire[31:0] w_phy_rx_clk_reg3_l0;
    wire[31:0] w_phy_rx_clk_reg4_l0;
    wire[31:0] w_phy_rx_data_reg1_l0;
    wire[31:0] w_phy_rx_data_reg2_l0;
    wire[31:0] w_phy_rx_data_reg3_l0;
    wire[31:0] w_phy_rx_ldo_reg1_l0;
    wire[31:0] w_phy_rx_ldo_reg2_l0;
    wire[31:0] w_phy_tx_clk_eng_reg1_l0;
    wire[31:0] w_phy_tx_clk_eng_reg2_l0;
    wire[31:0] w_phy_tx_clk_eng_reg3_l0;
    wire[31:0] w_phy_tx_clk_eng_reg4_l0;
    wire[31:0] w_phy_tx_clk_reg1_l0;
    wire[31:0] w_phy_tx_clk_reg2_l0;
    wire[31:0] w_phy_tx_clk_reg3_l0;
    wire[31:0] w_phy_tx_clk_reg4_l0;
    wire[31:0] w_phy_tx_clk_reg5_l0;
    wire[31:0] w_phy_tx_data_reg1_l0;
    wire[31:0] w_phy_tx_data_reg2_l0;
    wire[31:0] w_phy_tx_data_reg3_l0;
    wire[31:0] w_phy_tx_data_reg4_l0;
    wire[31:0] w_phy_tx_data_reg5_l0;
    wire[31:0] w_phy_tx_data_reg6_l0;
    wire[31:0] w_phy_tx_data_reg7_l0;
    wire[31:0] w_phy_tx_ldo_reg1_l0;
    wire[31:0] w_phy_tx_ldo_reg2_l0;
    wire[31:0] w_phy_trx_reg1_l1;
    wire[31:0] w_phy_trx_reg2_l1;
    wire[31:0] w_phy_trx_test_reg1_l1;
    wire[31:0] w_phy_trx_test_reg2_l1;
    wire[31:0] w_phy_rx_clk_eng_reg1_l1;
    wire[31:0] w_phy_rx_clk_eng_reg2_l1;
    wire[31:0] w_phy_rx_clk_eng_reg3_l1;
    wire[31:0] w_phy_rx_clk_eng_reg4_l1;
    wire[31:0] w_phy_rx_clk_reg1_l1;
    wire[31:0] w_phy_rx_clk_reg2_l1;
    wire[31:0] w_phy_rx_clk_reg3_l1;
    wire[31:0] w_phy_rx_clk_reg4_l1;
    wire[31:0] w_phy_rx_data_reg1_l1;
    wire[31:0] w_phy_rx_data_reg2_l1;
    wire[31:0] w_phy_rx_data_reg3_l1;
    wire[31:0] w_phy_rx_ldo_reg1_l1;
    wire[31:0] w_phy_rx_ldo_reg2_l1;
    wire[31:0] w_phy_tx_clk_eng_reg1_l1;
    wire[31:0] w_phy_tx_clk_eng_reg2_l1;
    wire[31:0] w_phy_tx_clk_eng_reg3_l1;
    wire[31:0] w_phy_tx_clk_eng_reg4_l1;
    wire[31:0] w_phy_tx_clk_reg1_l1;
    wire[31:0] w_phy_tx_clk_reg2_l1;
    wire[31:0] w_phy_tx_clk_reg3_l1;
    wire[31:0] w_phy_tx_clk_reg4_l1;
    wire[31:0] w_phy_tx_clk_reg5_l1;
    wire[31:0] w_phy_tx_data_reg1_l1;
    wire[31:0] w_phy_tx_data_reg2_l1;
    wire[31:0] w_phy_tx_data_reg3_l1;
    wire[31:0] w_phy_tx_data_reg4_l1;
    wire[31:0] w_phy_tx_data_reg5_l1;
    wire[31:0] w_phy_tx_data_reg6_l1;
    wire[31:0] w_phy_tx_data_reg7_l1;
    wire[31:0] w_phy_tx_ldo_reg1_l1;
    wire[31:0] w_phy_tx_ldo_reg2_l1;
    wire[31:0] w_phy_trx_reg1_l2;
    wire[31:0] w_phy_trx_reg2_l2;
    wire[31:0] w_phy_trx_test_reg1_l2;
    wire[31:0] w_phy_trx_test_reg2_l2;
    wire[31:0] w_phy_rx_clk_eng_reg1_l2;
    wire[31:0] w_phy_rx_clk_eng_reg2_l2;
    wire[31:0] w_phy_rx_clk_eng_reg3_l2;
    wire[31:0] w_phy_rx_clk_eng_reg4_l2;
    wire[31:0] w_phy_rx_clk_reg1_l2;
    wire[31:0] w_phy_rx_clk_reg2_l2;
    wire[31:0] w_phy_rx_clk_reg3_l2;
    wire[31:0] w_phy_rx_clk_reg4_l2;
    wire[31:0] w_phy_rx_data_reg1_l2;
    wire[31:0] w_phy_rx_data_reg2_l2;
    wire[31:0] w_phy_rx_data_reg3_l2;
    wire[31:0] w_phy_rx_ldo_reg1_l2;
    wire[31:0] w_phy_rx_ldo_reg2_l2;
    wire[31:0] w_phy_tx_clk_eng_reg1_l2;
    wire[31:0] w_phy_tx_clk_eng_reg2_l2;
    wire[31:0] w_phy_tx_clk_eng_reg3_l2;
    wire[31:0] w_phy_tx_clk_eng_reg4_l2;
    wire[31:0] w_phy_tx_clk_reg1_l2;
    wire[31:0] w_phy_tx_clk_reg2_l2;
    wire[31:0] w_phy_tx_clk_reg3_l2;
    wire[31:0] w_phy_tx_clk_reg4_l2;
    wire[31:0] w_phy_tx_clk_reg5_l2;
    wire[31:0] w_phy_tx_data_reg1_l2;
    wire[31:0] w_phy_tx_data_reg2_l2;
    wire[31:0] w_phy_tx_data_reg3_l2;
    wire[31:0] w_phy_tx_data_reg4_l2;
    wire[31:0] w_phy_tx_data_reg5_l2;
    wire[31:0] w_phy_tx_data_reg6_l2;
    wire[31:0] w_phy_tx_data_reg7_l2;
    wire[31:0] w_phy_tx_ldo_reg1_l2;
    wire[31:0] w_phy_tx_ldo_reg2_l2;
    wire[31:0] w_phy_trx_reg1_l3;
    wire[31:0] w_phy_trx_reg2_l3;
    wire[31:0] w_phy_trx_test_reg1_l3;
    wire[31:0] w_phy_trx_test_reg2_l3;
    wire[31:0] w_phy_rx_clk_eng_reg1_l3;
    wire[31:0] w_phy_rx_clk_eng_reg2_l3;
    wire[31:0] w_phy_rx_clk_eng_reg3_l3;
    wire[31:0] w_phy_rx_clk_eng_reg4_l3;
    wire[31:0] w_phy_rx_clk_reg1_l3;
    wire[31:0] w_phy_rx_clk_reg2_l3;
    wire[31:0] w_phy_rx_clk_reg3_l3;
    wire[31:0] w_phy_rx_clk_reg4_l3;
    wire[31:0] w_phy_rx_data_reg1_l3;
    wire[31:0] w_phy_rx_data_reg2_l3;
    wire[31:0] w_phy_rx_data_reg3_l3;
    wire[31:0] w_phy_rx_ldo_reg1_l3;
    wire[31:0] w_phy_rx_ldo_reg2_l3;
    wire[31:0] w_phy_tx_clk_eng_reg1_l3;
    wire[31:0] w_phy_tx_clk_eng_reg2_l3;
    wire[31:0] w_phy_tx_clk_eng_reg3_l3;
    wire[31:0] w_phy_tx_clk_eng_reg4_l3;
    wire[31:0] w_phy_tx_clk_reg1_l3;
    wire[31:0] w_phy_tx_clk_reg2_l3;
    wire[31:0] w_phy_tx_clk_reg3_l3;
    wire[31:0] w_phy_tx_clk_reg4_l3;
    wire[31:0] w_phy_tx_clk_reg5_l3;
    wire[31:0] w_phy_tx_data_reg1_l3;
    wire[31:0] w_phy_tx_data_reg2_l3;
    wire[31:0] w_phy_tx_data_reg3_l3;
    wire[31:0] w_phy_tx_data_reg4_l3;
    wire[31:0] w_phy_tx_data_reg5_l3;
    wire[31:0] w_phy_tx_data_reg6_l3;
    wire[31:0] w_phy_tx_data_reg7_l3;
    wire[31:0] w_phy_tx_ldo_reg1_l3;
    wire[31:0] w_phy_tx_ldo_reg2_l3;

    apb_csr_top_rf u_rf (
        .clk                  (apb_pclk               ),
        .rstn                 (apb_preset_n           ),
        .reg_addr             (apb_paddr[11:0]        ),
        .reg_rd               (rf_reg_rd              ),
        .reg_wr               (rf_reg_wr              ),
        .reg_wdata            (apb_pwdata             ),
        .reg_rdata            (rf_reg_rdata           ),
        .io_data_width_mode_mode  (w_io_mode_rf),   // rf 输出不消费；本体由 strap 锁存驱动
        .top_ctrl_train_mode                                   (w_train_mode),
        .train_step_ctrl_step_force_end                        (w_step_force_end),
        .train_step_ctrl_step_cnt                              (w_step_cnt),
        .datacheck_cfg_pattern_sel                             (w_pattern_sel),
        .bypass_cfg_bypass_step                                (w_bypass_step),
        .timeout_en_timeout_step_en                            (w_timeout_step_en),
        .timeout_threshold_timeout_threshold                   (w_timeout_threshold),
        .train_pattern_vref_lo_pattern                         (w_tp_vref_lo),
        .train_pattern_vref_hi_pattern                         (w_tp_vref_hi),
        .train_pattern_pi_lo_pattern                           (w_tp_pi_lo),
        .train_pattern_pi_hi_pattern                           (w_tp_pi_hi),
        .train_pattern_dcdl_lo_pattern                         (w_tp_dcdl_lo),
        .train_pattern_dcdl_hi_pattern                         (w_tp_dcdl_hi),
        .train_pattern_reorder_lo_pattern                      (w_tp_reorder_lo),
        .train_pattern_reorder_hi_pattern                      (w_tp_reorder_hi),
        .train_pattern_dc_lo_pattern                           (w_tp_dc_lo),
        .train_pattern_dc_hi_pattern                           (w_tp_dc_hi),
        .reorder_cfg_delay_step                                (w_delay_step),
        .top_ctrl_force_mode_switch_wp                         (w_force_mode_switch),
        .debug_cfg_seed_load_rx_wp                             (w_seed_load_rx),
        .debug_cfg_clear_rx_wp                                 (w_clear_rx),
        .debug_cfg_seed_load_tx_wp                             (w_seed_load_tx),
        .debug_cfg_clear_tx_wp                                 (w_clear_tx),
        .train_step_ctrl_step_trigger_wp                       (w_step_trigger),
        .sbc_tx_ctrl_tx_manual_trigger_wp                      (w_sbc_tx_trig),
        .sbc_rx_ctrl_rx_manual_read_wp                         (w_sbc_rx_manual_read),
        .sbc_rx_ctrl_rx_soft_rst_wp                            (w_sbc_rx_soft_rst),
        .sbc_tx_ctrl_tx_enable                                 (w_sbc_tx_enable),
        .sbc_tx_ctrl_tx_pkt_inject                             (w_sbc_tx_pkt_inject),
        .sbc_rx_ctrl_rx_enable                                 (w_sbc_rx_enable),
        .sbc_rx_ctrl_rx_buf_depth                              (w_sbc_rx_buf_depth),
        .sbc_dispatch_cfg_lid_filter                           (w_sbc_lid_filter),
        .sbc_dispatch_cfg_opcode_filter                        (w_sbc_opcode_filter),
        .sbc_dispatch_cfg_err_mask                             (w_sbc_err_mask),
        .sbc_irq_mask_pkt_arrival_mask                         (w_sbc_pkt_arrival_mask),
        .sbc_irq_mask_err_mask                                 (w_sbc_err_irq_mask),
        .sbc_lane_enable_lane_enable                           (w_sbc_lane_enable),
        .mcu_ctrl_mcu_soft_rst_n                               (w_mcu_soft_rst_n),
        .mcu_ctrl_boot_addr                                    (w_mcu_boot_addr),
        .mcu_ctrl_itcm_wr_en                                   (w_mcu_itcm_wr_en),
        .data_path_cfg_l1_reversal_en_tx                       (w_dpc_l1_rev_tx),
        .data_path_cfg_l1_reversal_en_rx                       (w_dpc_l1_rev_rx),
        .data_path_cfg_l1_repair_map_tx                        (w_dpc_l1_repair_tx),
        .data_path_cfg_l1_scramble_en_tx                       (w_dpc_l1_scram_tx),
        .data_path_cfg_rx_l1_repair_map_rx                     (w_dpcr_l1_repair_rx),
        .data_path_cfg_rx_l1_scramble_en_rx                    (w_dpcr_l1_scram_rx),
        .prbs_seed_tx_l1_i0_seed                               (w_seed_tx_l1_i0),
        .prbs_seed_tx_l1_i1_seed                               (w_seed_tx_l1_i1),
        .prbs_seed_tx_l1_i2_seed                               (w_seed_tx_l1_i2),
        .prbs_seed_tx_l1_i3_seed                               (w_seed_tx_l1_i3),
        .prbs_seed_tx_l1_i4_seed                               (w_seed_tx_l1_i4),
        .prbs_seed_tx_l1_i5_seed                               (w_seed_tx_l1_i5),
        .prbs_seed_tx_l1_i6_seed                               (w_seed_tx_l1_i6),
        .prbs_seed_tx_l1_i7_seed                               (w_seed_tx_l1_i7),
        .prbs_seed_tx_l1_i8_seed                               (w_seed_tx_l1_i8),
        .prbs_seed_tx_l1_i9_seed                               (w_seed_tx_l1_i9),
        .prbs_seed_tx_l1_i10_seed                              (w_seed_tx_l1_i10),
        .prbs_seed_tx_l1_i11_seed                              (w_seed_tx_l1_i11),
        .prbs_seed_tx_l1_i12_seed                              (w_seed_tx_l1_i12),
        .prbs_seed_tx_l1_i13_seed                              (w_seed_tx_l1_i13),
        .prbs_seed_tx_l1_i14_seed                              (w_seed_tx_l1_i14),
        .prbs_seed_tx_l1_i15_seed                              (w_seed_tx_l1_i15),
        .prbs_seed_rx_l1_i0_seed                               (w_seed_rx_l1_i0),
        .prbs_seed_rx_l1_i1_seed                               (w_seed_rx_l1_i1),
        .prbs_seed_rx_l1_i2_seed                               (w_seed_rx_l1_i2),
        .prbs_seed_rx_l1_i3_seed                               (w_seed_rx_l1_i3),
        .prbs_seed_rx_l1_i4_seed                               (w_seed_rx_l1_i4),
        .prbs_seed_rx_l1_i5_seed                               (w_seed_rx_l1_i5),
        .prbs_seed_rx_l1_i6_seed                               (w_seed_rx_l1_i6),
        .prbs_seed_rx_l1_i7_seed                               (w_seed_rx_l1_i7),
        .prbs_seed_rx_l1_i8_seed                               (w_seed_rx_l1_i8),
        .prbs_seed_rx_l1_i9_seed                               (w_seed_rx_l1_i9),
        .prbs_seed_rx_l1_i10_seed                              (w_seed_rx_l1_i10),
        .prbs_seed_rx_l1_i11_seed                              (w_seed_rx_l1_i11),
        .prbs_seed_rx_l1_i12_seed                              (w_seed_rx_l1_i12),
        .prbs_seed_rx_l1_i13_seed                              (w_seed_rx_l1_i13),
        .prbs_seed_rx_l1_i14_seed                              (w_seed_rx_l1_i14),
        .prbs_seed_rx_l1_i15_seed                              (w_seed_rx_l1_i15),
        .freq_flip_cfg_l1_flip_sel_tx                          (w_flip_tx_l1),
        .freq_flip_cfg_l1_flip_sel_rx                          (w_flip_rx_l1),
        .msb_lsb_cfg_tx_l1_flip_en_tx                          (w_msb_tx_l1),
        .msb_lsb_cfg_rx_l1_flip_en_rx                          (w_msb_rx_l1),
        .prbs_seed_tx_l2_i3_seed                               (w_seed_tx_l2_i3),
        .prbs_seed_tx_l2_i4_seed                               (w_seed_tx_l2_i4),
        .prbs_seed_tx_l2_i5_seed                               (w_seed_tx_l2_i5),
        .prbs_seed_tx_l2_i6_seed                               (w_seed_tx_l2_i6),
        .prbs_seed_tx_l2_i7_seed                               (w_seed_tx_l2_i7),
        .prbs_seed_tx_l2_i8_seed                               (w_seed_tx_l2_i8),
        .prbs_seed_tx_l2_i9_seed                               (w_seed_tx_l2_i9),
        .prbs_seed_tx_l2_i10_seed                              (w_seed_tx_l2_i10),
        .prbs_seed_tx_l2_i11_seed                              (w_seed_tx_l2_i11),
        .prbs_seed_tx_l2_i12_seed                              (w_seed_tx_l2_i12),
        .prbs_seed_tx_l2_i13_seed                              (w_seed_tx_l2_i13),
        .prbs_seed_tx_l2_i14_seed                              (w_seed_tx_l2_i14),
        .prbs_seed_tx_l2_i15_seed                              (w_seed_tx_l2_i15),
        .prbs_seed_rx_l2_i0_seed                               (w_seed_rx_l2_i0),
        .prbs_seed_rx_l2_i1_seed                               (w_seed_rx_l2_i1),
        .prbs_seed_rx_l2_i2_seed                               (w_seed_rx_l2_i2),
        .prbs_seed_rx_l2_i3_seed                               (w_seed_rx_l2_i3),
        .prbs_seed_rx_l2_i4_seed                               (w_seed_rx_l2_i4),
        .prbs_seed_rx_l2_i5_seed                               (w_seed_rx_l2_i5),
        .prbs_seed_rx_l2_i6_seed                               (w_seed_rx_l2_i6),
        .prbs_seed_rx_l2_i7_seed                               (w_seed_rx_l2_i7),
        .prbs_seed_rx_l2_i8_seed                               (w_seed_rx_l2_i8),
        .prbs_seed_rx_l2_i9_seed                               (w_seed_rx_l2_i9),
        .prbs_seed_rx_l2_i10_seed                              (w_seed_rx_l2_i10),
        .prbs_seed_rx_l2_i11_seed                              (w_seed_rx_l2_i11),
        .prbs_seed_rx_l2_i12_seed                              (w_seed_rx_l2_i12),
        .prbs_seed_rx_l2_i13_seed                              (w_seed_rx_l2_i13),
        .prbs_seed_rx_l2_i14_seed                              (w_seed_rx_l2_i14),
        .prbs_seed_rx_l2_i15_seed                              (w_seed_rx_l2_i15),
        .freq_flip_cfg_l2_flip_sel_tx                          (w_flip_tx_l2),
        .freq_flip_cfg_l2_flip_sel_rx                          (w_flip_rx_l2),
        .msb_lsb_cfg_tx_l2_flip_en_tx                          (w_msb_tx_l2),
        .msb_lsb_cfg_rx_l2_flip_en_rx                          (w_msb_rx_l2),
        .data_path_cfg_l3_reversal_en_tx                       (w_dpc_l3_rev_tx),
        .data_path_cfg_l3_reversal_en_rx                       (w_dpc_l3_rev_rx),
        .data_path_cfg_l3_repair_map_tx                        (w_dpc_l3_repair_tx),
        .data_path_cfg_l3_scramble_en_tx                       (w_dpc_l3_scram_tx),
        .data_path_cfg_rx_l3_repair_map_rx                     (w_dpcr_l3_repair_rx),
        .data_path_cfg_rx_l3_scramble_en_rx                    (w_dpcr_l3_scram_rx),
        .prbs_seed_tx_l3_i0_seed                               (w_seed_tx_l3_i0),
        .prbs_seed_tx_l3_i1_seed                               (w_seed_tx_l3_i1),
        .prbs_seed_tx_l3_i2_seed                               (w_seed_tx_l3_i2),
        .prbs_seed_tx_l3_i3_seed                               (w_seed_tx_l3_i3),
        .prbs_seed_tx_l3_i4_seed                               (w_seed_tx_l3_i4),
        .prbs_seed_tx_l3_i5_seed                               (w_seed_tx_l3_i5),
        .prbs_seed_tx_l3_i6_seed                               (w_seed_tx_l3_i6),
        .prbs_seed_tx_l3_i7_seed                               (w_seed_tx_l3_i7),
        .prbs_seed_tx_l3_i8_seed                               (w_seed_tx_l3_i8),
        .prbs_seed_tx_l3_i9_seed                               (w_seed_tx_l3_i9),
        .prbs_seed_tx_l3_i10_seed                              (w_seed_tx_l3_i10),
        .prbs_seed_tx_l3_i11_seed                              (w_seed_tx_l3_i11),
        .prbs_seed_tx_l3_i12_seed                              (w_seed_tx_l3_i12),
        .prbs_seed_tx_l3_i13_seed                              (w_seed_tx_l3_i13),
        .prbs_seed_tx_l3_i14_seed                              (w_seed_tx_l3_i14),
        .prbs_seed_tx_l3_i15_seed                              (w_seed_tx_l3_i15),
        .prbs_seed_rx_l3_i0_seed                               (w_seed_rx_l3_i0),
        .prbs_seed_rx_l3_i1_seed                               (w_seed_rx_l3_i1),
        .prbs_seed_rx_l3_i2_seed                               (w_seed_rx_l3_i2),
        .prbs_seed_rx_l3_i3_seed                               (w_seed_rx_l3_i3),
        .prbs_seed_rx_l3_i4_seed                               (w_seed_rx_l3_i4),
        .prbs_seed_rx_l3_i5_seed                               (w_seed_rx_l3_i5),
        .prbs_seed_rx_l3_i6_seed                               (w_seed_rx_l3_i6),
        .prbs_seed_rx_l3_i7_seed                               (w_seed_rx_l3_i7),
        .prbs_seed_rx_l3_i8_seed                               (w_seed_rx_l3_i8),
        .prbs_seed_rx_l3_i9_seed                               (w_seed_rx_l3_i9),
        .prbs_seed_rx_l3_i10_seed                              (w_seed_rx_l3_i10),
        .prbs_seed_rx_l3_i11_seed                              (w_seed_rx_l3_i11),
        .prbs_seed_rx_l3_i12_seed                              (w_seed_rx_l3_i12),
        .prbs_seed_rx_l3_i13_seed                              (w_seed_rx_l3_i13),
        .prbs_seed_rx_l3_i14_seed                              (w_seed_rx_l3_i14),
        .prbs_seed_rx_l3_i15_seed                              (w_seed_rx_l3_i15),
        .freq_flip_cfg_l3_flip_sel_tx                          (w_flip_tx_l3),
        .freq_flip_cfg_l3_flip_sel_rx                          (w_flip_rx_l3),
        .msb_lsb_cfg_tx_l3_flip_en_tx                          (w_msb_tx_l3),
        .msb_lsb_cfg_rx_l3_flip_en_rx                          (w_msb_rx_l3),
        .offset_param_l0_i0_offset                             (w_offset_param_l0_i0),
        .vref_param_l0_i0_vref                                 (w_vref_param_l0_i0),
        .pi_param_l0_i0_pi                                     (w_pi_param_l0_i0),
        .dcdl_param_l0_i0_dcdl                                 (w_dcdl_param_l0_i0),
        .offset_param_l0_i1_offset                             (w_offset_param_l0_i1),
        .vref_param_l0_i1_vref                                 (w_vref_param_l0_i1),
        .pi_param_l0_i1_pi                                     (w_pi_param_l0_i1),
        .dcdl_param_l0_i1_dcdl                                 (w_dcdl_param_l0_i1),
        .offset_param_l0_i2_offset                             (w_offset_param_l0_i2),
        .vref_param_l0_i2_vref                                 (w_vref_param_l0_i2),
        .pi_param_l0_i2_pi                                     (w_pi_param_l0_i2),
        .dcdl_param_l0_i2_dcdl                                 (w_dcdl_param_l0_i2),
        .offset_param_l0_i3_offset                             (w_offset_param_l0_i3),
        .vref_param_l0_i3_vref                                 (w_vref_param_l0_i3),
        .pi_param_l0_i3_pi                                     (w_pi_param_l0_i3),
        .dcdl_param_l0_i3_dcdl                                 (w_dcdl_param_l0_i3),
        .offset_param_l0_i4_offset                             (w_offset_param_l0_i4),
        .vref_param_l0_i4_vref                                 (w_vref_param_l0_i4),
        .pi_param_l0_i4_pi                                     (w_pi_param_l0_i4),
        .dcdl_param_l0_i4_dcdl                                 (w_dcdl_param_l0_i4),
        .offset_param_l0_i5_offset                             (w_offset_param_l0_i5),
        .vref_param_l0_i5_vref                                 (w_vref_param_l0_i5),
        .pi_param_l0_i5_pi                                     (w_pi_param_l0_i5),
        .dcdl_param_l0_i5_dcdl                                 (w_dcdl_param_l0_i5),
        .offset_param_l0_i6_offset                             (w_offset_param_l0_i6),
        .vref_param_l0_i6_vref                                 (w_vref_param_l0_i6),
        .pi_param_l0_i6_pi                                     (w_pi_param_l0_i6),
        .dcdl_param_l0_i6_dcdl                                 (w_dcdl_param_l0_i6),
        .offset_param_l0_i7_offset                             (w_offset_param_l0_i7),
        .vref_param_l0_i7_vref                                 (w_vref_param_l0_i7),
        .pi_param_l0_i7_pi                                     (w_pi_param_l0_i7),
        .dcdl_param_l0_i7_dcdl                                 (w_dcdl_param_l0_i7),
        .offset_param_l0_i8_offset                             (w_offset_param_l0_i8),
        .vref_param_l0_i8_vref                                 (w_vref_param_l0_i8),
        .pi_param_l0_i8_pi                                     (w_pi_param_l0_i8),
        .dcdl_param_l0_i8_dcdl                                 (w_dcdl_param_l0_i8),
        .offset_param_l0_i9_offset                             (w_offset_param_l0_i9),
        .vref_param_l0_i9_vref                                 (w_vref_param_l0_i9),
        .pi_param_l0_i9_pi                                     (w_pi_param_l0_i9),
        .dcdl_param_l0_i9_dcdl                                 (w_dcdl_param_l0_i9),
        .offset_param_l0_i10_offset                            (w_offset_param_l0_i10),
        .vref_param_l0_i10_vref                                (w_vref_param_l0_i10),
        .pi_param_l0_i10_pi                                    (w_pi_param_l0_i10),
        .dcdl_param_l0_i10_dcdl                                (w_dcdl_param_l0_i10),
        .offset_param_l0_i11_offset                            (w_offset_param_l0_i11),
        .vref_param_l0_i11_vref                                (w_vref_param_l0_i11),
        .pi_param_l0_i11_pi                                    (w_pi_param_l0_i11),
        .dcdl_param_l0_i11_dcdl                                (w_dcdl_param_l0_i11),
        .offset_param_l0_i12_offset                            (w_offset_param_l0_i12),
        .vref_param_l0_i12_vref                                (w_vref_param_l0_i12),
        .pi_param_l0_i12_pi                                    (w_pi_param_l0_i12),
        .dcdl_param_l0_i12_dcdl                                (w_dcdl_param_l0_i12),
        .offset_param_l0_i13_offset                            (w_offset_param_l0_i13),
        .vref_param_l0_i13_vref                                (w_vref_param_l0_i13),
        .pi_param_l0_i13_pi                                    (w_pi_param_l0_i13),
        .dcdl_param_l0_i13_dcdl                                (w_dcdl_param_l0_i13),
        .offset_param_l0_i14_offset                            (w_offset_param_l0_i14),
        .vref_param_l0_i14_vref                                (w_vref_param_l0_i14),
        .pi_param_l0_i14_pi                                    (w_pi_param_l0_i14),
        .dcdl_param_l0_i14_dcdl                                (w_dcdl_param_l0_i14),
        .offset_param_l0_i15_offset                            (w_offset_param_l0_i15),
        .vref_param_l0_i15_vref                                (w_vref_param_l0_i15),
        .pi_param_l0_i15_pi                                    (w_pi_param_l0_i15),
        .dcdl_param_l0_i15_dcdl                                (w_dcdl_param_l0_i15),
        .offset_param_l0_i16_offset                            (w_offset_param_l0_i16),
        .dcdl_param_l0_i16_dcdl                                (w_dcdl_param_l0_i16),
        // MDE-CHAIN-1008 lane1 train 参数贯通 (2026-10-12)
        .offset_param_l1_i16_offset                             (w_offset_param_l1_i16),
        .dcdl_param_l1_i16_dcdl                             (w_dcdl_param_l1_i16),
        .offset_param_l1_i0_offset                             (w_offset_param_l1_i0),
        .offset_param_l1_i1_offset                             (w_offset_param_l1_i1),
        .offset_param_l1_i2_offset                             (w_offset_param_l1_i2),
        .offset_param_l1_i3_offset                             (w_offset_param_l1_i3),
        .offset_param_l1_i4_offset                             (w_offset_param_l1_i4),
        .offset_param_l1_i5_offset                             (w_offset_param_l1_i5),
        .offset_param_l1_i6_offset                             (w_offset_param_l1_i6),
        .offset_param_l1_i7_offset                             (w_offset_param_l1_i7),
        .offset_param_l1_i8_offset                             (w_offset_param_l1_i8),
        .offset_param_l1_i9_offset                             (w_offset_param_l1_i9),
        .offset_param_l1_i10_offset                             (w_offset_param_l1_i10),
        .offset_param_l1_i11_offset                             (w_offset_param_l1_i11),
        .offset_param_l1_i12_offset                             (w_offset_param_l1_i12),
        .offset_param_l1_i13_offset                             (w_offset_param_l1_i13),
        .offset_param_l1_i14_offset                             (w_offset_param_l1_i14),
        .offset_param_l1_i15_offset                             (w_offset_param_l1_i15),
        .vref_param_l1_i0_vref                             (w_vref_param_l1_i0),
        .vref_param_l1_i1_vref                             (w_vref_param_l1_i1),
        .vref_param_l1_i2_vref                             (w_vref_param_l1_i2),
        .vref_param_l1_i3_vref                             (w_vref_param_l1_i3),
        .vref_param_l1_i4_vref                             (w_vref_param_l1_i4),
        .vref_param_l1_i5_vref                             (w_vref_param_l1_i5),
        .vref_param_l1_i6_vref                             (w_vref_param_l1_i6),
        .vref_param_l1_i7_vref                             (w_vref_param_l1_i7),
        .vref_param_l1_i8_vref                             (w_vref_param_l1_i8),
        .vref_param_l1_i9_vref                             (w_vref_param_l1_i9),
        .vref_param_l1_i10_vref                             (w_vref_param_l1_i10),
        .vref_param_l1_i11_vref                             (w_vref_param_l1_i11),
        .vref_param_l1_i12_vref                             (w_vref_param_l1_i12),
        .vref_param_l1_i13_vref                             (w_vref_param_l1_i13),
        .vref_param_l1_i14_vref                             (w_vref_param_l1_i14),
        .vref_param_l1_i15_vref                             (w_vref_param_l1_i15),
        .pi_param_l1_i0_pi                             (w_pi_param_l1_i0),
        .pi_param_l1_i1_pi                             (w_pi_param_l1_i1),
        .pi_param_l1_i2_pi                             (w_pi_param_l1_i2),
        .pi_param_l1_i3_pi                             (w_pi_param_l1_i3),
        .pi_param_l1_i4_pi                             (w_pi_param_l1_i4),
        .pi_param_l1_i5_pi                             (w_pi_param_l1_i5),
        .pi_param_l1_i6_pi                             (w_pi_param_l1_i6),
        .pi_param_l1_i7_pi                             (w_pi_param_l1_i7),
        .pi_param_l1_i8_pi                             (w_pi_param_l1_i8),
        .pi_param_l1_i9_pi                             (w_pi_param_l1_i9),
        .pi_param_l1_i10_pi                             (w_pi_param_l1_i10),
        .pi_param_l1_i11_pi                             (w_pi_param_l1_i11),
        .pi_param_l1_i12_pi                             (w_pi_param_l1_i12),
        .pi_param_l1_i13_pi                             (w_pi_param_l1_i13),
        .pi_param_l1_i14_pi                             (w_pi_param_l1_i14),
        .pi_param_l1_i15_pi                             (w_pi_param_l1_i15),
        .dcdl_param_l1_i0_dcdl                             (w_dcdl_param_l1_i0),
        .dcdl_param_l1_i1_dcdl                             (w_dcdl_param_l1_i1),
        .dcdl_param_l1_i2_dcdl                             (w_dcdl_param_l1_i2),
        .dcdl_param_l1_i3_dcdl                             (w_dcdl_param_l1_i3),
        .dcdl_param_l1_i4_dcdl                             (w_dcdl_param_l1_i4),
        .dcdl_param_l1_i5_dcdl                             (w_dcdl_param_l1_i5),
        .dcdl_param_l1_i6_dcdl                             (w_dcdl_param_l1_i6),
        .dcdl_param_l1_i7_dcdl                             (w_dcdl_param_l1_i7),
        .dcdl_param_l1_i8_dcdl                             (w_dcdl_param_l1_i8),
        .dcdl_param_l1_i9_dcdl                             (w_dcdl_param_l1_i9),
        .dcdl_param_l1_i10_dcdl                             (w_dcdl_param_l1_i10),
        .dcdl_param_l1_i11_dcdl                             (w_dcdl_param_l1_i11),
        .dcdl_param_l1_i12_dcdl                             (w_dcdl_param_l1_i12),
        .dcdl_param_l1_i13_dcdl                             (w_dcdl_param_l1_i13),
        .dcdl_param_l1_i14_dcdl                             (w_dcdl_param_l1_i14),
        .dcdl_param_l1_i15_dcdl                             (w_dcdl_param_l1_i15),

        // MDE-CHAIN-1008 lane2 train 参数贯通 (2026-10-12)
        .offset_param_l2_i16_offset                             (w_offset_param_l2_i16),
        .dcdl_param_l2_i16_dcdl                             (w_dcdl_param_l2_i16),
        .offset_param_l2_i0_offset                             (w_offset_param_l2_i0),
        .offset_param_l2_i1_offset                             (w_offset_param_l2_i1),
        .offset_param_l2_i2_offset                             (w_offset_param_l2_i2),
        .offset_param_l2_i3_offset                             (w_offset_param_l2_i3),
        .offset_param_l2_i4_offset                             (w_offset_param_l2_i4),
        .offset_param_l2_i5_offset                             (w_offset_param_l2_i5),
        .offset_param_l2_i6_offset                             (w_offset_param_l2_i6),
        .offset_param_l2_i7_offset                             (w_offset_param_l2_i7),
        .offset_param_l2_i8_offset                             (w_offset_param_l2_i8),
        .offset_param_l2_i9_offset                             (w_offset_param_l2_i9),
        .offset_param_l2_i10_offset                             (w_offset_param_l2_i10),
        .offset_param_l2_i11_offset                             (w_offset_param_l2_i11),
        .offset_param_l2_i12_offset                             (w_offset_param_l2_i12),
        .offset_param_l2_i13_offset                             (w_offset_param_l2_i13),
        .offset_param_l2_i14_offset                             (w_offset_param_l2_i14),
        .offset_param_l2_i15_offset                             (w_offset_param_l2_i15),
        .vref_param_l2_i0_vref                             (w_vref_param_l2_i0),
        .vref_param_l2_i1_vref                             (w_vref_param_l2_i1),
        .vref_param_l2_i2_vref                             (w_vref_param_l2_i2),
        .vref_param_l2_i3_vref                             (w_vref_param_l2_i3),
        .vref_param_l2_i4_vref                             (w_vref_param_l2_i4),
        .vref_param_l2_i5_vref                             (w_vref_param_l2_i5),
        .vref_param_l2_i6_vref                             (w_vref_param_l2_i6),
        .vref_param_l2_i7_vref                             (w_vref_param_l2_i7),
        .vref_param_l2_i8_vref                             (w_vref_param_l2_i8),
        .vref_param_l2_i9_vref                             (w_vref_param_l2_i9),
        .vref_param_l2_i10_vref                             (w_vref_param_l2_i10),
        .vref_param_l2_i11_vref                             (w_vref_param_l2_i11),
        .vref_param_l2_i12_vref                             (w_vref_param_l2_i12),
        .vref_param_l2_i13_vref                             (w_vref_param_l2_i13),
        .vref_param_l2_i14_vref                             (w_vref_param_l2_i14),
        .vref_param_l2_i15_vref                             (w_vref_param_l2_i15),
        .pi_param_l2_i0_pi                             (w_pi_param_l2_i0),
        .pi_param_l2_i1_pi                             (w_pi_param_l2_i1),
        .pi_param_l2_i2_pi                             (w_pi_param_l2_i2),
        .pi_param_l2_i3_pi                             (w_pi_param_l2_i3),
        .pi_param_l2_i4_pi                             (w_pi_param_l2_i4),
        .pi_param_l2_i5_pi                             (w_pi_param_l2_i5),
        .pi_param_l2_i6_pi                             (w_pi_param_l2_i6),
        .pi_param_l2_i7_pi                             (w_pi_param_l2_i7),
        .pi_param_l2_i8_pi                             (w_pi_param_l2_i8),
        .pi_param_l2_i9_pi                             (w_pi_param_l2_i9),
        .pi_param_l2_i10_pi                             (w_pi_param_l2_i10),
        .pi_param_l2_i11_pi                             (w_pi_param_l2_i11),
        .pi_param_l2_i12_pi                             (w_pi_param_l2_i12),
        .pi_param_l2_i13_pi                             (w_pi_param_l2_i13),
        .pi_param_l2_i14_pi                             (w_pi_param_l2_i14),
        .pi_param_l2_i15_pi                             (w_pi_param_l2_i15),
        .dcdl_param_l2_i0_dcdl                             (w_dcdl_param_l2_i0),
        .dcdl_param_l2_i1_dcdl                             (w_dcdl_param_l2_i1),
        .dcdl_param_l2_i2_dcdl                             (w_dcdl_param_l2_i2),
        .dcdl_param_l2_i3_dcdl                             (w_dcdl_param_l2_i3),
        .dcdl_param_l2_i4_dcdl                             (w_dcdl_param_l2_i4),
        .dcdl_param_l2_i5_dcdl                             (w_dcdl_param_l2_i5),
        .dcdl_param_l2_i6_dcdl                             (w_dcdl_param_l2_i6),
        .dcdl_param_l2_i7_dcdl                             (w_dcdl_param_l2_i7),
        .dcdl_param_l2_i8_dcdl                             (w_dcdl_param_l2_i8),
        .dcdl_param_l2_i9_dcdl                             (w_dcdl_param_l2_i9),
        .dcdl_param_l2_i10_dcdl                             (w_dcdl_param_l2_i10),
        .dcdl_param_l2_i11_dcdl                             (w_dcdl_param_l2_i11),
        .dcdl_param_l2_i12_dcdl                             (w_dcdl_param_l2_i12),
        .dcdl_param_l2_i13_dcdl                             (w_dcdl_param_l2_i13),
        .dcdl_param_l2_i14_dcdl                             (w_dcdl_param_l2_i14),
        .dcdl_param_l2_i15_dcdl                             (w_dcdl_param_l2_i15),

        // MDE-CHAIN-1008 lane3 train 参数贯通 (2026-10-12)
        .offset_param_l3_i16_offset                             (w_offset_param_l3_i16),
        .dcdl_param_l3_i16_dcdl                             (w_dcdl_param_l3_i16),
        .offset_param_l3_i0_offset                             (w_offset_param_l3_i0),
        .offset_param_l3_i1_offset                             (w_offset_param_l3_i1),
        .offset_param_l3_i2_offset                             (w_offset_param_l3_i2),
        .offset_param_l3_i3_offset                             (w_offset_param_l3_i3),
        .offset_param_l3_i4_offset                             (w_offset_param_l3_i4),
        .offset_param_l3_i5_offset                             (w_offset_param_l3_i5),
        .offset_param_l3_i6_offset                             (w_offset_param_l3_i6),
        .offset_param_l3_i7_offset                             (w_offset_param_l3_i7),
        .offset_param_l3_i8_offset                             (w_offset_param_l3_i8),
        .offset_param_l3_i9_offset                             (w_offset_param_l3_i9),
        .offset_param_l3_i10_offset                             (w_offset_param_l3_i10),
        .offset_param_l3_i11_offset                             (w_offset_param_l3_i11),
        .offset_param_l3_i12_offset                             (w_offset_param_l3_i12),
        .offset_param_l3_i13_offset                             (w_offset_param_l3_i13),
        .offset_param_l3_i14_offset                             (w_offset_param_l3_i14),
        .offset_param_l3_i15_offset                             (w_offset_param_l3_i15),
        .vref_param_l3_i0_vref                             (w_vref_param_l3_i0),
        .vref_param_l3_i1_vref                             (w_vref_param_l3_i1),
        .vref_param_l3_i2_vref                             (w_vref_param_l3_i2),
        .vref_param_l3_i3_vref                             (w_vref_param_l3_i3),
        .vref_param_l3_i4_vref                             (w_vref_param_l3_i4),
        .vref_param_l3_i5_vref                             (w_vref_param_l3_i5),
        .vref_param_l3_i6_vref                             (w_vref_param_l3_i6),
        .vref_param_l3_i7_vref                             (w_vref_param_l3_i7),
        .vref_param_l3_i8_vref                             (w_vref_param_l3_i8),
        .vref_param_l3_i9_vref                             (w_vref_param_l3_i9),
        .vref_param_l3_i10_vref                             (w_vref_param_l3_i10),
        .vref_param_l3_i11_vref                             (w_vref_param_l3_i11),
        .vref_param_l3_i12_vref                             (w_vref_param_l3_i12),
        .vref_param_l3_i13_vref                             (w_vref_param_l3_i13),
        .vref_param_l3_i14_vref                             (w_vref_param_l3_i14),
        .vref_param_l3_i15_vref                             (w_vref_param_l3_i15),
        .pi_param_l3_i0_pi                             (w_pi_param_l3_i0),
        .pi_param_l3_i1_pi                             (w_pi_param_l3_i1),
        .pi_param_l3_i2_pi                             (w_pi_param_l3_i2),
        .pi_param_l3_i3_pi                             (w_pi_param_l3_i3),
        .pi_param_l3_i4_pi                             (w_pi_param_l3_i4),
        .pi_param_l3_i5_pi                             (w_pi_param_l3_i5),
        .pi_param_l3_i6_pi                             (w_pi_param_l3_i6),
        .pi_param_l3_i7_pi                             (w_pi_param_l3_i7),
        .pi_param_l3_i8_pi                             (w_pi_param_l3_i8),
        .pi_param_l3_i9_pi                             (w_pi_param_l3_i9),
        .pi_param_l3_i10_pi                             (w_pi_param_l3_i10),
        .pi_param_l3_i11_pi                             (w_pi_param_l3_i11),
        .pi_param_l3_i12_pi                             (w_pi_param_l3_i12),
        .pi_param_l3_i13_pi                             (w_pi_param_l3_i13),
        .pi_param_l3_i14_pi                             (w_pi_param_l3_i14),
        .pi_param_l3_i15_pi                             (w_pi_param_l3_i15),
        .dcdl_param_l3_i0_dcdl                             (w_dcdl_param_l3_i0),
        .dcdl_param_l3_i1_dcdl                             (w_dcdl_param_l3_i1),
        .dcdl_param_l3_i2_dcdl                             (w_dcdl_param_l3_i2),
        .dcdl_param_l3_i3_dcdl                             (w_dcdl_param_l3_i3),
        .dcdl_param_l3_i4_dcdl                             (w_dcdl_param_l3_i4),
        .dcdl_param_l3_i5_dcdl                             (w_dcdl_param_l3_i5),
        .dcdl_param_l3_i6_dcdl                             (w_dcdl_param_l3_i6),
        .dcdl_param_l3_i7_dcdl                             (w_dcdl_param_l3_i7),
        .dcdl_param_l3_i8_dcdl                             (w_dcdl_param_l3_i8),
        .dcdl_param_l3_i9_dcdl                             (w_dcdl_param_l3_i9),
        .dcdl_param_l3_i10_dcdl                             (w_dcdl_param_l3_i10),
        .dcdl_param_l3_i11_dcdl                             (w_dcdl_param_l3_i11),
        .dcdl_param_l3_i12_dcdl                             (w_dcdl_param_l3_i12),
        .dcdl_param_l3_i13_dcdl                             (w_dcdl_param_l3_i13),
        .dcdl_param_l3_i14_dcdl                             (w_dcdl_param_l3_i14),
        .dcdl_param_l3_i15_dcdl                             (w_dcdl_param_l3_i15),

        .int_clear_clear_wp                                    (w_int_clear_wp),
        .int_clear_clear_wd                                    (w_int_clear_wd),
        .intc_clear_clear_wp                                   (w_intc_clear_wp),
        .intc_clear_clear_wd                                   (w_intc_clear_wd),
        .sbc_irq_flag_pkt_arrival_flag_wp                      (w_sbc_pkt_arr_flag_wp),
        .sbc_irq_flag_pkt_arrival_flag_wd                      (w_sbc_pkt_arr_flag_wd),
        .sbc_irq_flag_err_flag_wp                              (w_sbc_err_flag_wp),
        .sbc_irq_flag_err_flag_wd                              (w_sbc_err_flag_wd),
        .top_ctrl_global_rst_n                                 (w_top_ctrl_global_rst_n),
        .cdc_fifo_rst_fifo_rst_wp                              (w_cdc_fifo_rst_wp),
        .train_param_src_src                                   (w_train_param_src_o),
        .datacheck_cfg_fail_threshold                          (w_datacheck_fail_threshold_o),
        .eq_param_ctle_tap                                     (w_eq_param_ctle_tap_o),
        .train_mode_mode                                       (w_train_mode_mode),
        .int_mask_mcu_mask_mcu                                 (w_int_mask_mcu_mask_mcu),
        .int_mask_ext_mask_ext                                 (w_int_mask_ext_mask_ext),
        .phy_trx_en_ctrl_l0_trx_en                             (w_phy_trx_en_l0),
        .phy_trx_en_ctrl_l1_trx_en                             (w_phy_trx_en_l1),
        .phy_trx_en_ctrl_l2_trx_en                             (w_phy_trx_en_l2),
        .phy_trx_en_ctrl_l3_trx_en                             (w_phy_trx_en_l3),
        .sbc_redundancy_sb_txr_en                              (w_sbc_txr_en),
        .sbc_redundancy_sb_rxr_sel                             (w_sbc_rxr_sel),
        .phy_trx_reg1_l0_reg                                   (w_phy_trx_reg1_l0),
        .phy_trx_reg2_l0_reg                                   (w_phy_trx_reg2_l0),
        .phy_trx_test_reg1_l0_reg                              (w_phy_trx_test_reg1_l0),
        .phy_trx_test_reg2_l0_reg                              (w_phy_trx_test_reg2_l0),
        .phy_rx_clk_eng_reg1_l0_reg                            (w_phy_rx_clk_eng_reg1_l0),
        .phy_rx_clk_eng_reg2_l0_reg                            (w_phy_rx_clk_eng_reg2_l0),
        .phy_rx_clk_eng_reg3_l0_reg                            (w_phy_rx_clk_eng_reg3_l0),
        .phy_rx_clk_eng_reg4_l0_reg                            (w_phy_rx_clk_eng_reg4_l0),
        .phy_rx_clk_reg1_l0_reg                                (w_phy_rx_clk_reg1_l0),
        .phy_rx_clk_reg2_l0_reg                                (w_phy_rx_clk_reg2_l0),
        .phy_rx_clk_reg3_l0_reg                                (w_phy_rx_clk_reg3_l0),
        .phy_rx_clk_reg4_l0_reg                                (w_phy_rx_clk_reg4_l0),
        .phy_rx_data_reg1_l0_reg                               (w_phy_rx_data_reg1_l0),
        .phy_rx_data_reg2_l0_reg                               (w_phy_rx_data_reg2_l0),
        .phy_rx_data_reg3_l0_reg                               (w_phy_rx_data_reg3_l0),
        .phy_rx_ldo_reg1_l0_reg                                (w_phy_rx_ldo_reg1_l0),
        .phy_rx_ldo_reg2_l0_reg                                (w_phy_rx_ldo_reg2_l0),
        .phy_tx_clk_eng_reg1_l0_reg                            (w_phy_tx_clk_eng_reg1_l0),
        .phy_tx_clk_eng_reg2_l0_reg                            (w_phy_tx_clk_eng_reg2_l0),
        .phy_tx_clk_eng_reg3_l0_reg                            (w_phy_tx_clk_eng_reg3_l0),
        .phy_tx_clk_eng_reg4_l0_reg                            (w_phy_tx_clk_eng_reg4_l0),
        .phy_tx_clk_reg1_l0_reg                                (w_phy_tx_clk_reg1_l0),
        .phy_tx_clk_reg2_l0_reg                                (w_phy_tx_clk_reg2_l0),
        .phy_tx_clk_reg3_l0_reg                                (w_phy_tx_clk_reg3_l0),
        .phy_tx_clk_reg4_l0_reg                                (w_phy_tx_clk_reg4_l0),
        .phy_tx_clk_reg5_l0_reg                                (w_phy_tx_clk_reg5_l0),
        .phy_tx_data_reg1_l0_reg                               (w_phy_tx_data_reg1_l0),
        .phy_tx_data_reg2_l0_reg                               (w_phy_tx_data_reg2_l0),
        .phy_tx_data_reg3_l0_reg                               (w_phy_tx_data_reg3_l0),
        .phy_tx_data_reg4_l0_reg                               (w_phy_tx_data_reg4_l0),
        .phy_tx_data_reg5_l0_reg                               (w_phy_tx_data_reg5_l0),
        .phy_tx_data_reg6_l0_reg                               (w_phy_tx_data_reg6_l0),
        .phy_tx_data_reg7_l0_reg                               (w_phy_tx_data_reg7_l0),
        .phy_tx_ldo_reg1_l0_reg                                (w_phy_tx_ldo_reg1_l0),
        .phy_tx_ldo_reg2_l0_reg                                (w_phy_tx_ldo_reg2_l0),
        .phy_trx_reg1_l1_reg                                   (w_phy_trx_reg1_l1),
        .phy_trx_reg2_l1_reg                                   (w_phy_trx_reg2_l1),
        .phy_trx_test_reg1_l1_reg                              (w_phy_trx_test_reg1_l1),
        .phy_trx_test_reg2_l1_reg                              (w_phy_trx_test_reg2_l1),
        .phy_rx_clk_eng_reg1_l1_reg                            (w_phy_rx_clk_eng_reg1_l1),
        .phy_rx_clk_eng_reg2_l1_reg                            (w_phy_rx_clk_eng_reg2_l1),
        .phy_rx_clk_eng_reg3_l1_reg                            (w_phy_rx_clk_eng_reg3_l1),
        .phy_rx_clk_eng_reg4_l1_reg                            (w_phy_rx_clk_eng_reg4_l1),
        .phy_rx_clk_reg1_l1_reg                                (w_phy_rx_clk_reg1_l1),
        .phy_rx_clk_reg2_l1_reg                                (w_phy_rx_clk_reg2_l1),
        .phy_rx_clk_reg3_l1_reg                                (w_phy_rx_clk_reg3_l1),
        .phy_rx_clk_reg4_l1_reg                                (w_phy_rx_clk_reg4_l1),
        .phy_rx_data_reg1_l1_reg                               (w_phy_rx_data_reg1_l1),
        .phy_rx_data_reg2_l1_reg                               (w_phy_rx_data_reg2_l1),
        .phy_rx_data_reg3_l1_reg                               (w_phy_rx_data_reg3_l1),
        .phy_rx_ldo_reg1_l1_reg                                (w_phy_rx_ldo_reg1_l1),
        .phy_rx_ldo_reg2_l1_reg                                (w_phy_rx_ldo_reg2_l1),
        .phy_tx_clk_eng_reg1_l1_reg                            (w_phy_tx_clk_eng_reg1_l1),
        .phy_tx_clk_eng_reg2_l1_reg                            (w_phy_tx_clk_eng_reg2_l1),
        .phy_tx_clk_eng_reg3_l1_reg                            (w_phy_tx_clk_eng_reg3_l1),
        .phy_tx_clk_eng_reg4_l1_reg                            (w_phy_tx_clk_eng_reg4_l1),
        .phy_tx_clk_reg1_l1_reg                                (w_phy_tx_clk_reg1_l1),
        .phy_tx_clk_reg2_l1_reg                                (w_phy_tx_clk_reg2_l1),
        .phy_tx_clk_reg3_l1_reg                                (w_phy_tx_clk_reg3_l1),
        .phy_tx_clk_reg4_l1_reg                                (w_phy_tx_clk_reg4_l1),
        .phy_tx_clk_reg5_l1_reg                                (w_phy_tx_clk_reg5_l1),
        .phy_tx_data_reg1_l1_reg                               (w_phy_tx_data_reg1_l1),
        .phy_tx_data_reg2_l1_reg                               (w_phy_tx_data_reg2_l1),
        .phy_tx_data_reg3_l1_reg                               (w_phy_tx_data_reg3_l1),
        .phy_tx_data_reg4_l1_reg                               (w_phy_tx_data_reg4_l1),
        .phy_tx_data_reg5_l1_reg                               (w_phy_tx_data_reg5_l1),
        .phy_tx_data_reg6_l1_reg                               (w_phy_tx_data_reg6_l1),
        .phy_tx_data_reg7_l1_reg                               (w_phy_tx_data_reg7_l1),
        .phy_tx_ldo_reg1_l1_reg                                (w_phy_tx_ldo_reg1_l1),
        .phy_tx_ldo_reg2_l1_reg                                (w_phy_tx_ldo_reg2_l1),
        .phy_trx_reg1_l2_reg                                   (w_phy_trx_reg1_l2),
        .phy_trx_reg2_l2_reg                                   (w_phy_trx_reg2_l2),
        .phy_trx_test_reg1_l2_reg                              (w_phy_trx_test_reg1_l2),
        .phy_trx_test_reg2_l2_reg                              (w_phy_trx_test_reg2_l2),
        .phy_rx_clk_eng_reg1_l2_reg                            (w_phy_rx_clk_eng_reg1_l2),
        .phy_rx_clk_eng_reg2_l2_reg                            (w_phy_rx_clk_eng_reg2_l2),
        .phy_rx_clk_eng_reg3_l2_reg                            (w_phy_rx_clk_eng_reg3_l2),
        .phy_rx_clk_eng_reg4_l2_reg                            (w_phy_rx_clk_eng_reg4_l2),
        .phy_rx_clk_reg1_l2_reg                                (w_phy_rx_clk_reg1_l2),
        .phy_rx_clk_reg2_l2_reg                                (w_phy_rx_clk_reg2_l2),
        .phy_rx_clk_reg3_l2_reg                                (w_phy_rx_clk_reg3_l2),
        .phy_rx_clk_reg4_l2_reg                                (w_phy_rx_clk_reg4_l2),
        .phy_rx_data_reg1_l2_reg                               (w_phy_rx_data_reg1_l2),
        .phy_rx_data_reg2_l2_reg                               (w_phy_rx_data_reg2_l2),
        .phy_rx_data_reg3_l2_reg                               (w_phy_rx_data_reg3_l2),
        .phy_rx_ldo_reg1_l2_reg                                (w_phy_rx_ldo_reg1_l2),
        .phy_rx_ldo_reg2_l2_reg                                (w_phy_rx_ldo_reg2_l2),
        .phy_tx_clk_eng_reg1_l2_reg                            (w_phy_tx_clk_eng_reg1_l2),
        .phy_tx_clk_eng_reg2_l2_reg                            (w_phy_tx_clk_eng_reg2_l2),
        .phy_tx_clk_eng_reg3_l2_reg                            (w_phy_tx_clk_eng_reg3_l2),
        .phy_tx_clk_eng_reg4_l2_reg                            (w_phy_tx_clk_eng_reg4_l2),
        .phy_tx_clk_reg1_l2_reg                                (w_phy_tx_clk_reg1_l2),
        .phy_tx_clk_reg2_l2_reg                                (w_phy_tx_clk_reg2_l2),
        .phy_tx_clk_reg3_l2_reg                                (w_phy_tx_clk_reg3_l2),
        .phy_tx_clk_reg4_l2_reg                                (w_phy_tx_clk_reg4_l2),
        .phy_tx_clk_reg5_l2_reg                                (w_phy_tx_clk_reg5_l2),
        .phy_tx_data_reg1_l2_reg                               (w_phy_tx_data_reg1_l2),
        .phy_tx_data_reg2_l2_reg                               (w_phy_tx_data_reg2_l2),
        .phy_tx_data_reg3_l2_reg                               (w_phy_tx_data_reg3_l2),
        .phy_tx_data_reg4_l2_reg                               (w_phy_tx_data_reg4_l2),
        .phy_tx_data_reg5_l2_reg                               (w_phy_tx_data_reg5_l2),
        .phy_tx_data_reg6_l2_reg                               (w_phy_tx_data_reg6_l2),
        .phy_tx_data_reg7_l2_reg                               (w_phy_tx_data_reg7_l2),
        .phy_tx_ldo_reg1_l2_reg                                (w_phy_tx_ldo_reg1_l2),
        .phy_tx_ldo_reg2_l2_reg                                (w_phy_tx_ldo_reg2_l2),
        .phy_trx_reg1_l3_reg                                   (w_phy_trx_reg1_l3),
        .phy_trx_reg2_l3_reg                                   (w_phy_trx_reg2_l3),
        .phy_trx_test_reg1_l3_reg                              (w_phy_trx_test_reg1_l3),
        .phy_trx_test_reg2_l3_reg                              (w_phy_trx_test_reg2_l3),
        .phy_rx_clk_eng_reg1_l3_reg                            (w_phy_rx_clk_eng_reg1_l3),
        .phy_rx_clk_eng_reg2_l3_reg                            (w_phy_rx_clk_eng_reg2_l3),
        .phy_rx_clk_eng_reg3_l3_reg                            (w_phy_rx_clk_eng_reg3_l3),
        .phy_rx_clk_eng_reg4_l3_reg                            (w_phy_rx_clk_eng_reg4_l3),
        .phy_rx_clk_reg1_l3_reg                                (w_phy_rx_clk_reg1_l3),
        .phy_rx_clk_reg2_l3_reg                                (w_phy_rx_clk_reg2_l3),
        .phy_rx_clk_reg3_l3_reg                                (w_phy_rx_clk_reg3_l3),
        .phy_rx_clk_reg4_l3_reg                                (w_phy_rx_clk_reg4_l3),
        .phy_rx_data_reg1_l3_reg                               (w_phy_rx_data_reg1_l3),
        .phy_rx_data_reg2_l3_reg                               (w_phy_rx_data_reg2_l3),
        .phy_rx_data_reg3_l3_reg                               (w_phy_rx_data_reg3_l3),
        .phy_rx_ldo_reg1_l3_reg                                (w_phy_rx_ldo_reg1_l3),
        .phy_rx_ldo_reg2_l3_reg                                (w_phy_rx_ldo_reg2_l3),
        .phy_tx_clk_eng_reg1_l3_reg                            (w_phy_tx_clk_eng_reg1_l3),
        .phy_tx_clk_eng_reg2_l3_reg                            (w_phy_tx_clk_eng_reg2_l3),
        .phy_tx_clk_eng_reg3_l3_reg                            (w_phy_tx_clk_eng_reg3_l3),
        .phy_tx_clk_eng_reg4_l3_reg                            (w_phy_tx_clk_eng_reg4_l3),
        .phy_tx_clk_reg1_l3_reg                                (w_phy_tx_clk_reg1_l3),
        .phy_tx_clk_reg2_l3_reg                                (w_phy_tx_clk_reg2_l3),
        .phy_tx_clk_reg3_l3_reg                                (w_phy_tx_clk_reg3_l3),
        .phy_tx_clk_reg4_l3_reg                                (w_phy_tx_clk_reg4_l3),
        .phy_tx_clk_reg5_l3_reg                                (w_phy_tx_clk_reg5_l3),
        .phy_tx_data_reg1_l3_reg                               (w_phy_tx_data_reg1_l3),
        .phy_tx_data_reg2_l3_reg                               (w_phy_tx_data_reg2_l3),
        .phy_tx_data_reg3_l3_reg                               (w_phy_tx_data_reg3_l3),
        .phy_tx_data_reg4_l3_reg                               (w_phy_tx_data_reg4_l3),
        .phy_tx_data_reg5_l3_reg                               (w_phy_tx_data_reg5_l3),
        .phy_tx_data_reg6_l3_reg                               (w_phy_tx_data_reg6_l3),
        .phy_tx_data_reg7_l3_reg                               (w_phy_tx_data_reg7_l3),
        .phy_tx_ldo_reg1_l3_reg                                (w_phy_tx_ldo_reg1_l3),
        .phy_tx_ldo_reg2_l3_reg                                (w_phy_tx_ldo_reg2_l3),
        .io_data_width_mode_lane_num                           (LANE_NUM[3:0]),
        .io_data_width_mode_analog_ser_ratio                   (w_ser_ratio),
        .prbs_lfsr_clk_gate_clk_gate_en                        (w_prbs_clk_gate),
        .int_status_status                                     (int_status_reg),
        .int_raw_status_raw                                    (int_raw),
        .cdc_fifo_ovf_ovf                                      (cdc_fifo_ovf_reg),
        .cdc_fifo_unf_unf                                      (cdc_fifo_unf_reg),
        .intc_mask_mcu_mask_mcu                                (int_mask_mcu_reg),
        .intc_mask_ext_mask_ext                                (int_mask_ext_reg),
        .intc_status_status                                    (int_status_reg),
        .intc_raw_raw                                          (int_raw),
        .train_status_step_state                               (4'b0),
        .train_status_train_done                               (int_status_reg[0]),
        .train_status_train_fail                               (int_status_reg[1]),
        .train_status_step_done                                (int_status_reg[2]),
        .train_status_train_timeout                            (int_status_reg[3]),
        .datacheck_fail_count_fail_count                       (8'h0),
        .aligned_rx_status_aligned                             (aligned_rx_sync2),
        .sbc_tx_status_tx_buf_level                            (sbc_tx_stat_sync2[2:0]),
        .sbc_tx_status_tx_busy                                 (sbc_tx_stat_sync2[3]),
        .sbc_tx_status_tx_err                                  (sbc_tx_err_sticky),
        .sbc_rx_status_rx_buf_level                            (sbc_rx_stat_sync2[3:0]),
        .sbc_rx_status_rx_busy                                 (sbc_rx_stat_sync2[4]),
        .sbc_rx_status_rx_parity_err                           (sbc_rx_parity_err_sticky),
        .sbc_rx_status_rx_gap_err                              (sbc_rx_gap_err_sticky),
        .sbc_rx_status_rx_buf_ovf                              (sbc_rx_buf_ovf_sticky),
        .sbc_rx_buf_rx_buf                                     (sbc_rx_rdata_sync2),
        .sbc_dispatch_status_pkt_arrival_flag                  (sbc_disp_sync2[0]),
        .sbc_dispatch_status_err_flag                          (sbc_disp_sync2[1]),
        .sbc_dispatch_status_illegal_opcode                    (sbc_disp_sync2[2]),
        .sbc_dispatch_status_illegal_lid                       (sbc_disp_sync2[3]),
        .rx_fifo_status_l0_level                               (rx_fifo_status_sync2[0*8 +: 4]),
        .rx_fifo_status_l0_full                                (rx_fifo_status_sync2[0*8+4]),
        .rx_fifo_status_l0_empty                               (rx_fifo_status_sync2[0*8+5]),
        .rx_fifo_status_l0_ovf                                 (fifo_ovf_sticky[0]),
        .rx_fifo_status_l0_unf                                 (fifo_unf_sticky[0]),
        .rx_fifo_status_l1_level                               (rx_fifo_status_sync2[1*8 +: 4]),
        .rx_fifo_status_l1_full                                (rx_fifo_status_sync2[1*8+4]),
        .rx_fifo_status_l1_empty                               (rx_fifo_status_sync2[1*8+5]),
        .rx_fifo_status_l1_ovf                                 (fifo_ovf_sticky[1]),
        .rx_fifo_status_l1_unf                                 (fifo_unf_sticky[1]),
        .rx_fifo_status_l2_level                               (rx_fifo_status_sync2[2*8 +: 4]),
        .rx_fifo_status_l2_full                                (rx_fifo_status_sync2[2*8+4]),
        .rx_fifo_status_l2_empty                               (rx_fifo_status_sync2[2*8+5]),
        .rx_fifo_status_l2_ovf                                 (fifo_ovf_sticky[2]),
        .rx_fifo_status_l2_unf                                 (fifo_unf_sticky[2]),
        .rx_fifo_status_l3_level                               (rx_fifo_status_sync2[3*8 +: 4]),
        .rx_fifo_status_l3_full                                (rx_fifo_status_sync2[3*8+4]),
        .rx_fifo_status_l3_empty                               (rx_fifo_status_sync2[3*8+5]),
        .rx_fifo_status_l3_ovf                                 (fifo_ovf_sticky[3]),
        .rx_fifo_status_l3_unf                                 (fifo_unf_sticky[3]),
        .phy_rx_clk_eng_readout_l0_readout                     (phy_rx_clk_eng_readout_l0_i),
        .phy_tx_clk_eng_readout_l0_readout                     (phy_tx_clk_eng_readout_l0_i),
        .phy_connect_dig_l0_connect_dig                        (phy_connect_dig_l0_i),
        .phy_err_edge_status_l0_lo_err_edge                    (phy_err_edge_l0_i[31:0]),
        .phy_err_edge_status_l0_hi_err_edge                    (phy_err_edge_l0_i[63:32]),
        .phy_err_edge_status_l0_rfu_err_edge                   (phy_err_edge_l0_i[67:64]),
        .phy_rx_clk_eng_readout_l1_readout                     (phy_rx_clk_eng_readout_l1_i),
        .phy_tx_clk_eng_readout_l1_readout                     (phy_tx_clk_eng_readout_l1_i),
        .phy_connect_dig_l1_connect_dig                        (phy_connect_dig_l1_i),
        .phy_err_edge_status_l1_lo_err_edge                    (phy_err_edge_l1_i[31:0]),
        .phy_err_edge_status_l1_hi_err_edge                    (phy_err_edge_l1_i[63:32]),
        .phy_err_edge_status_l1_rfu_err_edge                   (phy_err_edge_l1_i[67:64]),
        .phy_rx_clk_eng_readout_l2_readout                     (phy_rx_clk_eng_readout_l2_i),
        .phy_tx_clk_eng_readout_l2_readout                     (phy_tx_clk_eng_readout_l2_i),
        .phy_connect_dig_l2_connect_dig                        (phy_connect_dig_l2_i),
        .phy_err_edge_status_l2_lo_err_edge                    (phy_err_edge_l2_i[31:0]),
        .phy_err_edge_status_l2_hi_err_edge                    (phy_err_edge_l2_i[63:32]),
        .phy_err_edge_status_l2_rfu_err_edge                   (phy_err_edge_l2_i[67:64]),
        .phy_rx_clk_eng_readout_l3_readout                     (phy_rx_clk_eng_readout_l3_i),
        .phy_tx_clk_eng_readout_l3_readout                     (phy_tx_clk_eng_readout_l3_i),
        .phy_connect_dig_l3_connect_dig                        (phy_connect_dig_l3_i),
        .phy_err_edge_status_l3_lo_err_edge                    (phy_err_edge_l3_i[31:0]),
        .phy_err_edge_status_l3_hi_err_edge                    (phy_err_edge_l3_i[63:32]),
        .phy_err_edge_status_l3_rfu_err_edge                   (phy_err_edge_l3_i[67:64])
    );

    // ==========================================================================
    // per-lane 输出打包（lane0 复位值常量 / lane1·lane3 rf / lane2 DPC+SEEDTX i0~2
    // 为共享段双写 alias 寄存器, 其余 rf; 位序 lane0 在 LSB, 与原版一致）
    // ==========================================================================
    assign reversal_en_tx_o[0] = 1'b0;
    assign reversal_en_rx_o[0] = 1'b0;
    assign repair_map_tx_o[0*16 +: 16] = 16'h0;
    assign scramble_en_tx_o[0] = 1'b1;
    assign repair_map_rx_o[0*16 +: 16] = 16'h0;
    assign scramble_en_o[0] = 1'b1;
    assign flip_sel_tx_o[0*4 +: 4] = 4'h0;
    assign flip_sel_rx_o[0*4 +: 4] = 4'h0;
    assign msb_lsb_flip_en_tx_o[0*17 +: 17] = 17'h0;
    assign msb_lsb_flip_en_rx_o[0*17 +: 17] = 17'h0;
    assign prbs_seed_tx_o[0*368 + 0*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_rx_o[0*368 + 0*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_tx_o[0*368 + 1*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_rx_o[0*368 + 1*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_tx_o[0*368 + 2*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_rx_o[0*368 + 2*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_tx_o[0*368 + 3*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_rx_o[0*368 + 3*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_tx_o[0*368 + 4*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_rx_o[0*368 + 4*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_tx_o[0*368 + 5*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_rx_o[0*368 + 5*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_tx_o[0*368 + 6*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_rx_o[0*368 + 6*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_tx_o[0*368 + 7*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_rx_o[0*368 + 7*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_tx_o[0*368 + 8*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_rx_o[0*368 + 8*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_tx_o[0*368 + 9*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_rx_o[0*368 + 9*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_tx_o[0*368 + 10*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_rx_o[0*368 + 10*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_tx_o[0*368 + 11*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_rx_o[0*368 + 11*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_tx_o[0*368 + 12*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_rx_o[0*368 + 12*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_tx_o[0*368 + 13*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_rx_o[0*368 + 13*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_tx_o[0*368 + 14*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_rx_o[0*368 + 14*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_tx_o[0*368 + 15*23 +: 23] = 23'h7FFFFF;
    assign prbs_seed_rx_o[0*368 + 15*23 +: 23] = 23'h7FFFFF;
    assign reversal_en_tx_o[1] = w_dpc_l1_rev_tx;
    assign reversal_en_rx_o[1] = w_dpc_l1_rev_rx;
    assign repair_map_tx_o[1*16 +: 16] = w_dpc_l1_repair_tx;
    assign scramble_en_tx_o[1] = w_dpc_l1_scram_tx;
    assign repair_map_rx_o[1*16 +: 16] = w_dpcr_l1_repair_rx;
    assign scramble_en_o[1] = w_dpcr_l1_scram_rx;
    assign flip_sel_tx_o[1*4 +: 4] = w_flip_tx_l1;
    assign flip_sel_rx_o[1*4 +: 4] = w_flip_rx_l1;
    assign msb_lsb_flip_en_tx_o[1*17 +: 17] = w_msb_tx_l1;
    assign msb_lsb_flip_en_rx_o[1*17 +: 17] = w_msb_rx_l1;
    assign prbs_seed_tx_o[1*368 + 0*23 +: 23] = w_seed_tx_l1_i0;
    assign prbs_seed_rx_o[1*368 + 0*23 +: 23] = w_seed_rx_l1_i0;
    assign prbs_seed_tx_o[1*368 + 1*23 +: 23] = w_seed_tx_l1_i1;
    assign prbs_seed_rx_o[1*368 + 1*23 +: 23] = w_seed_rx_l1_i1;
    assign prbs_seed_tx_o[1*368 + 2*23 +: 23] = w_seed_tx_l1_i2;
    assign prbs_seed_rx_o[1*368 + 2*23 +: 23] = w_seed_rx_l1_i2;
    assign prbs_seed_tx_o[1*368 + 3*23 +: 23] = w_seed_tx_l1_i3;
    assign prbs_seed_rx_o[1*368 + 3*23 +: 23] = w_seed_rx_l1_i3;
    assign prbs_seed_tx_o[1*368 + 4*23 +: 23] = w_seed_tx_l1_i4;
    assign prbs_seed_rx_o[1*368 + 4*23 +: 23] = w_seed_rx_l1_i4;
    assign prbs_seed_tx_o[1*368 + 5*23 +: 23] = w_seed_tx_l1_i5;
    assign prbs_seed_rx_o[1*368 + 5*23 +: 23] = w_seed_rx_l1_i5;
    assign prbs_seed_tx_o[1*368 + 6*23 +: 23] = w_seed_tx_l1_i6;
    assign prbs_seed_rx_o[1*368 + 6*23 +: 23] = w_seed_rx_l1_i6;
    assign prbs_seed_tx_o[1*368 + 7*23 +: 23] = w_seed_tx_l1_i7;
    assign prbs_seed_rx_o[1*368 + 7*23 +: 23] = w_seed_rx_l1_i7;
    assign prbs_seed_tx_o[1*368 + 8*23 +: 23] = w_seed_tx_l1_i8;
    assign prbs_seed_rx_o[1*368 + 8*23 +: 23] = w_seed_rx_l1_i8;
    assign prbs_seed_tx_o[1*368 + 9*23 +: 23] = w_seed_tx_l1_i9;
    assign prbs_seed_rx_o[1*368 + 9*23 +: 23] = w_seed_rx_l1_i9;
    assign prbs_seed_tx_o[1*368 + 10*23 +: 23] = w_seed_tx_l1_i10;
    assign prbs_seed_rx_o[1*368 + 10*23 +: 23] = w_seed_rx_l1_i10;
    assign prbs_seed_tx_o[1*368 + 11*23 +: 23] = w_seed_tx_l1_i11;
    assign prbs_seed_rx_o[1*368 + 11*23 +: 23] = w_seed_rx_l1_i11;
    assign prbs_seed_tx_o[1*368 + 12*23 +: 23] = w_seed_tx_l1_i12;
    assign prbs_seed_rx_o[1*368 + 12*23 +: 23] = w_seed_rx_l1_i12;
    assign prbs_seed_tx_o[1*368 + 13*23 +: 23] = w_seed_tx_l1_i13;
    assign prbs_seed_rx_o[1*368 + 13*23 +: 23] = w_seed_rx_l1_i13;
    assign prbs_seed_tx_o[1*368 + 14*23 +: 23] = w_seed_tx_l1_i14;
    assign prbs_seed_rx_o[1*368 + 14*23 +: 23] = w_seed_rx_l1_i14;
    assign prbs_seed_tx_o[1*368 + 15*23 +: 23] = w_seed_tx_l1_i15;
    assign prbs_seed_rx_o[1*368 + 15*23 +: 23] = w_seed_rx_l1_i15;
    assign reversal_en_tx_o[2] = l2_reversal_en_tx;
    assign reversal_en_rx_o[2] = l2_reversal_en_rx;
    assign repair_map_tx_o[2*16 +: 16] = l2_repair_map_tx;
    assign scramble_en_tx_o[2] = l2_scramble_en_tx;
    assign repair_map_rx_o[2*16 +: 16] = l2_repair_map_rx;
    assign scramble_en_o[2] = l2_scramble_en_rx;
    assign flip_sel_tx_o[2*4 +: 4] = w_flip_tx_l2;
    assign flip_sel_rx_o[2*4 +: 4] = w_flip_rx_l2;
    assign msb_lsb_flip_en_tx_o[2*17 +: 17] = w_msb_tx_l2;
    assign msb_lsb_flip_en_rx_o[2*17 +: 17] = w_msb_rx_l2;
    assign prbs_seed_tx_o[2*368 + 0*23 +: 23] = l2_seed_tx_i0;
    assign prbs_seed_tx_o[2*368 + 1*23 +: 23] = l2_seed_tx_i1;
    assign prbs_seed_tx_o[2*368 + 2*23 +: 23] = l2_seed_tx_i2;
    assign prbs_seed_tx_o[2*368 + 3*23 +: 23] = w_seed_tx_l2_i3;
    assign prbs_seed_tx_o[2*368 + 4*23 +: 23] = w_seed_tx_l2_i4;
    assign prbs_seed_tx_o[2*368 + 5*23 +: 23] = w_seed_tx_l2_i5;
    assign prbs_seed_tx_o[2*368 + 6*23 +: 23] = w_seed_tx_l2_i6;
    assign prbs_seed_tx_o[2*368 + 7*23 +: 23] = w_seed_tx_l2_i7;
    assign prbs_seed_tx_o[2*368 + 8*23 +: 23] = w_seed_tx_l2_i8;
    assign prbs_seed_tx_o[2*368 + 9*23 +: 23] = w_seed_tx_l2_i9;
    assign prbs_seed_tx_o[2*368 + 10*23 +: 23] = w_seed_tx_l2_i10;
    assign prbs_seed_tx_o[2*368 + 11*23 +: 23] = w_seed_tx_l2_i11;
    assign prbs_seed_tx_o[2*368 + 12*23 +: 23] = w_seed_tx_l2_i12;
    assign prbs_seed_tx_o[2*368 + 13*23 +: 23] = w_seed_tx_l2_i13;
    assign prbs_seed_tx_o[2*368 + 14*23 +: 23] = w_seed_tx_l2_i14;
    assign prbs_seed_tx_o[2*368 + 15*23 +: 23] = w_seed_tx_l2_i15;
    assign prbs_seed_rx_o[2*368 + 0*23 +: 23] = w_seed_rx_l2_i0;
    assign prbs_seed_rx_o[2*368 + 1*23 +: 23] = w_seed_rx_l2_i1;
    assign prbs_seed_rx_o[2*368 + 2*23 +: 23] = w_seed_rx_l2_i2;
    assign prbs_seed_rx_o[2*368 + 3*23 +: 23] = w_seed_rx_l2_i3;
    assign prbs_seed_rx_o[2*368 + 4*23 +: 23] = w_seed_rx_l2_i4;
    assign prbs_seed_rx_o[2*368 + 5*23 +: 23] = w_seed_rx_l2_i5;
    assign prbs_seed_rx_o[2*368 + 6*23 +: 23] = w_seed_rx_l2_i6;
    assign prbs_seed_rx_o[2*368 + 7*23 +: 23] = w_seed_rx_l2_i7;
    assign prbs_seed_rx_o[2*368 + 8*23 +: 23] = w_seed_rx_l2_i8;
    assign prbs_seed_rx_o[2*368 + 9*23 +: 23] = w_seed_rx_l2_i9;
    assign prbs_seed_rx_o[2*368 + 10*23 +: 23] = w_seed_rx_l2_i10;
    assign prbs_seed_rx_o[2*368 + 11*23 +: 23] = w_seed_rx_l2_i11;
    assign prbs_seed_rx_o[2*368 + 12*23 +: 23] = w_seed_rx_l2_i12;
    assign prbs_seed_rx_o[2*368 + 13*23 +: 23] = w_seed_rx_l2_i13;
    assign prbs_seed_rx_o[2*368 + 14*23 +: 23] = w_seed_rx_l2_i14;
    assign prbs_seed_rx_o[2*368 + 15*23 +: 23] = w_seed_rx_l2_i15;
    assign reversal_en_tx_o[3] = w_dpc_l3_rev_tx;
    assign reversal_en_rx_o[3] = w_dpc_l3_rev_rx;
    assign repair_map_tx_o[3*16 +: 16] = w_dpc_l3_repair_tx;
    assign scramble_en_tx_o[3] = w_dpc_l3_scram_tx;
    assign repair_map_rx_o[3*16 +: 16] = w_dpcr_l3_repair_rx;
    assign scramble_en_o[3] = w_dpcr_l3_scram_rx;
    assign flip_sel_tx_o[3*4 +: 4] = w_flip_tx_l3;
    assign flip_sel_rx_o[3*4 +: 4] = w_flip_rx_l3;
    assign msb_lsb_flip_en_tx_o[3*17 +: 17] = w_msb_tx_l3;
    assign msb_lsb_flip_en_rx_o[3*17 +: 17] = w_msb_rx_l3;
    assign prbs_seed_tx_o[3*368 + 0*23 +: 23] = w_seed_tx_l3_i0;
    assign prbs_seed_rx_o[3*368 + 0*23 +: 23] = w_seed_rx_l3_i0;
    assign prbs_seed_tx_o[3*368 + 1*23 +: 23] = w_seed_tx_l3_i1;
    assign prbs_seed_rx_o[3*368 + 1*23 +: 23] = w_seed_rx_l3_i1;
    assign prbs_seed_tx_o[3*368 + 2*23 +: 23] = w_seed_tx_l3_i2;
    assign prbs_seed_rx_o[3*368 + 2*23 +: 23] = w_seed_rx_l3_i2;
    assign prbs_seed_tx_o[3*368 + 3*23 +: 23] = w_seed_tx_l3_i3;
    assign prbs_seed_rx_o[3*368 + 3*23 +: 23] = w_seed_rx_l3_i3;
    assign prbs_seed_tx_o[3*368 + 4*23 +: 23] = w_seed_tx_l3_i4;
    assign prbs_seed_rx_o[3*368 + 4*23 +: 23] = w_seed_rx_l3_i4;
    assign prbs_seed_tx_o[3*368 + 5*23 +: 23] = w_seed_tx_l3_i5;
    assign prbs_seed_rx_o[3*368 + 5*23 +: 23] = w_seed_rx_l3_i5;
    assign prbs_seed_tx_o[3*368 + 6*23 +: 23] = w_seed_tx_l3_i6;
    assign prbs_seed_rx_o[3*368 + 6*23 +: 23] = w_seed_rx_l3_i6;
    assign prbs_seed_tx_o[3*368 + 7*23 +: 23] = w_seed_tx_l3_i7;
    assign prbs_seed_rx_o[3*368 + 7*23 +: 23] = w_seed_rx_l3_i7;
    assign prbs_seed_tx_o[3*368 + 8*23 +: 23] = w_seed_tx_l3_i8;
    assign prbs_seed_rx_o[3*368 + 8*23 +: 23] = w_seed_rx_l3_i8;
    assign prbs_seed_tx_o[3*368 + 9*23 +: 23] = w_seed_tx_l3_i9;
    assign prbs_seed_rx_o[3*368 + 9*23 +: 23] = w_seed_rx_l3_i9;
    assign prbs_seed_tx_o[3*368 + 10*23 +: 23] = w_seed_tx_l3_i10;
    assign prbs_seed_rx_o[3*368 + 10*23 +: 23] = w_seed_rx_l3_i10;
    assign prbs_seed_tx_o[3*368 + 11*23 +: 23] = w_seed_tx_l3_i11;
    assign prbs_seed_rx_o[3*368 + 11*23 +: 23] = w_seed_rx_l3_i11;
    assign prbs_seed_tx_o[3*368 + 12*23 +: 23] = w_seed_tx_l3_i12;
    assign prbs_seed_rx_o[3*368 + 12*23 +: 23] = w_seed_rx_l3_i12;
    assign prbs_seed_tx_o[3*368 + 13*23 +: 23] = w_seed_tx_l3_i13;
    assign prbs_seed_rx_o[3*368 + 13*23 +: 23] = w_seed_rx_l3_i13;
    assign prbs_seed_tx_o[3*368 + 14*23 +: 23] = w_seed_tx_l3_i14;
    assign prbs_seed_rx_o[3*368 + 14*23 +: 23] = w_seed_rx_l3_i14;
    assign prbs_seed_tx_o[3*368 + 15*23 +: 23] = w_seed_tx_l3_i15;
    assign prbs_seed_rx_o[3*368 + 15*23 +: 23] = w_seed_rx_l3_i15;
    assign phy_ctrl_writein_l0_o[0*32 +: 32] = w_phy_trx_reg1_l0;
    assign phy_ctrl_writein_l0_o[1*32 +: 32] = w_phy_trx_reg2_l0;
    assign phy_ctrl_writein_l0_o[2*32 +: 32] = w_phy_trx_test_reg1_l0;
    assign phy_ctrl_writein_l0_o[3*32 +: 32] = w_phy_trx_test_reg2_l0;
    assign phy_ctrl_writein_l0_o[4*32 +: 32] = w_phy_rx_clk_eng_reg1_l0;
    assign phy_ctrl_writein_l0_o[5*32 +: 32] = w_phy_rx_clk_eng_reg2_l0;
    assign phy_ctrl_writein_l0_o[6*32 +: 32] = w_phy_rx_clk_eng_reg3_l0;
    assign phy_ctrl_writein_l0_o[7*32 +: 32] = w_phy_rx_clk_eng_reg4_l0;
    assign phy_ctrl_writein_l0_o[8*32 +: 32] = w_phy_rx_clk_reg1_l0;
    assign phy_ctrl_writein_l0_o[9*32 +: 32] = w_phy_rx_clk_reg2_l0;
    assign phy_ctrl_writein_l0_o[10*32 +: 32] = w_phy_rx_clk_reg3_l0;
    assign phy_ctrl_writein_l0_o[11*32 +: 32] = w_phy_rx_clk_reg4_l0;
    assign phy_ctrl_writein_l0_o[12*32 +: 32] = w_phy_rx_data_reg1_l0;
    assign phy_ctrl_writein_l0_o[13*32 +: 32] = w_phy_rx_data_reg2_l0;
    assign phy_ctrl_writein_l0_o[14*32 +: 32] = w_phy_rx_data_reg3_l0;
    assign phy_ctrl_writein_l0_o[15*32 +: 32] = w_phy_rx_ldo_reg1_l0;
    assign phy_ctrl_writein_l0_o[16*32 +: 32] = w_phy_rx_ldo_reg2_l0;
    assign phy_ctrl_writein_l0_o[17*32 +: 32] = w_phy_tx_clk_eng_reg1_l0;
    assign phy_ctrl_writein_l0_o[18*32 +: 32] = w_phy_tx_clk_eng_reg2_l0;
    assign phy_ctrl_writein_l0_o[19*32 +: 32] = w_phy_tx_clk_eng_reg3_l0;
    assign phy_ctrl_writein_l0_o[20*32 +: 32] = w_phy_tx_clk_eng_reg4_l0;
    assign phy_ctrl_writein_l0_o[21*32 +: 32] = w_phy_tx_clk_reg1_l0;
    assign phy_ctrl_writein_l0_o[22*32 +: 32] = w_phy_tx_clk_reg2_l0;
    assign phy_ctrl_writein_l0_o[23*32 +: 32] = w_phy_tx_clk_reg3_l0;
    assign phy_ctrl_writein_l0_o[24*32 +: 32] = w_phy_tx_clk_reg4_l0;
    assign phy_ctrl_writein_l0_o[25*32 +: 32] = w_phy_tx_clk_reg5_l0;
    assign phy_ctrl_writein_l0_o[26*32 +: 32] = w_phy_tx_data_reg1_l0;
    assign phy_ctrl_writein_l0_o[27*32 +: 32] = w_phy_tx_data_reg2_l0;
    assign phy_ctrl_writein_l0_o[28*32 +: 32] = w_phy_tx_data_reg3_l0;
    assign phy_ctrl_writein_l0_o[29*32 +: 32] = w_phy_tx_data_reg4_l0;
    assign phy_ctrl_writein_l0_o[30*32 +: 32] = w_phy_tx_data_reg5_l0;
    assign phy_ctrl_writein_l0_o[31*32 +: 32] = w_phy_tx_data_reg6_l0;
    assign phy_ctrl_writein_l0_o[32*32 +: 32] = w_phy_tx_data_reg7_l0;
    assign phy_ctrl_writein_l0_o[33*32 +: 32] = w_phy_tx_ldo_reg1_l0;
    assign phy_ctrl_writein_l0_o[34*32 +: 32] = w_phy_tx_ldo_reg2_l0;
    assign phy_ctrl_writein_l1_o[0*32 +: 32] = w_phy_trx_reg1_l1;
    assign phy_ctrl_writein_l1_o[1*32 +: 32] = w_phy_trx_reg2_l1;
    assign phy_ctrl_writein_l1_o[2*32 +: 32] = w_phy_trx_test_reg1_l1;
    assign phy_ctrl_writein_l1_o[3*32 +: 32] = w_phy_trx_test_reg2_l1;
    assign phy_ctrl_writein_l1_o[4*32 +: 32] = w_phy_rx_clk_eng_reg1_l1;
    assign phy_ctrl_writein_l1_o[5*32 +: 32] = w_phy_rx_clk_eng_reg2_l1;
    assign phy_ctrl_writein_l1_o[6*32 +: 32] = w_phy_rx_clk_eng_reg3_l1;
    assign phy_ctrl_writein_l1_o[7*32 +: 32] = w_phy_rx_clk_eng_reg4_l1;
    assign phy_ctrl_writein_l1_o[8*32 +: 32] = w_phy_rx_clk_reg1_l1;
    assign phy_ctrl_writein_l1_o[9*32 +: 32] = w_phy_rx_clk_reg2_l1;
    assign phy_ctrl_writein_l1_o[10*32 +: 32] = w_phy_rx_clk_reg3_l1;
    assign phy_ctrl_writein_l1_o[11*32 +: 32] = w_phy_rx_clk_reg4_l1;
    assign phy_ctrl_writein_l1_o[12*32 +: 32] = w_phy_rx_data_reg1_l1;
    assign phy_ctrl_writein_l1_o[13*32 +: 32] = w_phy_rx_data_reg2_l1;
    assign phy_ctrl_writein_l1_o[14*32 +: 32] = w_phy_rx_data_reg3_l1;
    assign phy_ctrl_writein_l1_o[15*32 +: 32] = w_phy_rx_ldo_reg1_l1;
    assign phy_ctrl_writein_l1_o[16*32 +: 32] = w_phy_rx_ldo_reg2_l1;
    assign phy_ctrl_writein_l1_o[17*32 +: 32] = w_phy_tx_clk_eng_reg1_l1;
    assign phy_ctrl_writein_l1_o[18*32 +: 32] = w_phy_tx_clk_eng_reg2_l1;
    assign phy_ctrl_writein_l1_o[19*32 +: 32] = w_phy_tx_clk_eng_reg3_l1;
    assign phy_ctrl_writein_l1_o[20*32 +: 32] = w_phy_tx_clk_eng_reg4_l1;
    assign phy_ctrl_writein_l1_o[21*32 +: 32] = w_phy_tx_clk_reg1_l1;
    assign phy_ctrl_writein_l1_o[22*32 +: 32] = w_phy_tx_clk_reg2_l1;
    assign phy_ctrl_writein_l1_o[23*32 +: 32] = w_phy_tx_clk_reg3_l1;
    assign phy_ctrl_writein_l1_o[24*32 +: 32] = w_phy_tx_clk_reg4_l1;
    assign phy_ctrl_writein_l1_o[25*32 +: 32] = w_phy_tx_clk_reg5_l1;
    assign phy_ctrl_writein_l1_o[26*32 +: 32] = w_phy_tx_data_reg1_l1;
    assign phy_ctrl_writein_l1_o[27*32 +: 32] = w_phy_tx_data_reg2_l1;
    assign phy_ctrl_writein_l1_o[28*32 +: 32] = w_phy_tx_data_reg3_l1;
    assign phy_ctrl_writein_l1_o[29*32 +: 32] = w_phy_tx_data_reg4_l1;
    assign phy_ctrl_writein_l1_o[30*32 +: 32] = w_phy_tx_data_reg5_l1;
    assign phy_ctrl_writein_l1_o[31*32 +: 32] = w_phy_tx_data_reg6_l1;
    assign phy_ctrl_writein_l1_o[32*32 +: 32] = w_phy_tx_data_reg7_l1;
    assign phy_ctrl_writein_l1_o[33*32 +: 32] = w_phy_tx_ldo_reg1_l1;
    assign phy_ctrl_writein_l1_o[34*32 +: 32] = w_phy_tx_ldo_reg2_l1;
    assign phy_ctrl_writein_l2_o[0*32 +: 32] = w_phy_trx_reg1_l2;
    assign phy_ctrl_writein_l2_o[1*32 +: 32] = w_phy_trx_reg2_l2;
    assign phy_ctrl_writein_l2_o[2*32 +: 32] = w_phy_trx_test_reg1_l2;
    assign phy_ctrl_writein_l2_o[3*32 +: 32] = w_phy_trx_test_reg2_l2;
    assign phy_ctrl_writein_l2_o[4*32 +: 32] = w_phy_rx_clk_eng_reg1_l2;
    assign phy_ctrl_writein_l2_o[5*32 +: 32] = w_phy_rx_clk_eng_reg2_l2;
    assign phy_ctrl_writein_l2_o[6*32 +: 32] = w_phy_rx_clk_eng_reg3_l2;
    assign phy_ctrl_writein_l2_o[7*32 +: 32] = w_phy_rx_clk_eng_reg4_l2;
    assign phy_ctrl_writein_l2_o[8*32 +: 32] = w_phy_rx_clk_reg1_l2;
    assign phy_ctrl_writein_l2_o[9*32 +: 32] = w_phy_rx_clk_reg2_l2;
    assign phy_ctrl_writein_l2_o[10*32 +: 32] = w_phy_rx_clk_reg3_l2;
    assign phy_ctrl_writein_l2_o[11*32 +: 32] = w_phy_rx_clk_reg4_l2;
    assign phy_ctrl_writein_l2_o[12*32 +: 32] = w_phy_rx_data_reg1_l2;
    assign phy_ctrl_writein_l2_o[13*32 +: 32] = w_phy_rx_data_reg2_l2;
    assign phy_ctrl_writein_l2_o[14*32 +: 32] = w_phy_rx_data_reg3_l2;
    assign phy_ctrl_writein_l2_o[15*32 +: 32] = w_phy_rx_ldo_reg1_l2;
    assign phy_ctrl_writein_l2_o[16*32 +: 32] = w_phy_rx_ldo_reg2_l2;
    assign phy_ctrl_writein_l2_o[17*32 +: 32] = w_phy_tx_clk_eng_reg1_l2;
    assign phy_ctrl_writein_l2_o[18*32 +: 32] = w_phy_tx_clk_eng_reg2_l2;
    assign phy_ctrl_writein_l2_o[19*32 +: 32] = w_phy_tx_clk_eng_reg3_l2;
    assign phy_ctrl_writein_l2_o[20*32 +: 32] = w_phy_tx_clk_eng_reg4_l2;
    assign phy_ctrl_writein_l2_o[21*32 +: 32] = w_phy_tx_clk_reg1_l2;
    assign phy_ctrl_writein_l2_o[22*32 +: 32] = w_phy_tx_clk_reg2_l2;
    assign phy_ctrl_writein_l2_o[23*32 +: 32] = w_phy_tx_clk_reg3_l2;
    assign phy_ctrl_writein_l2_o[24*32 +: 32] = w_phy_tx_clk_reg4_l2;
    assign phy_ctrl_writein_l2_o[25*32 +: 32] = w_phy_tx_clk_reg5_l2;
    assign phy_ctrl_writein_l2_o[26*32 +: 32] = w_phy_tx_data_reg1_l2;
    assign phy_ctrl_writein_l2_o[27*32 +: 32] = w_phy_tx_data_reg2_l2;
    assign phy_ctrl_writein_l2_o[28*32 +: 32] = w_phy_tx_data_reg3_l2;
    assign phy_ctrl_writein_l2_o[29*32 +: 32] = w_phy_tx_data_reg4_l2;
    assign phy_ctrl_writein_l2_o[30*32 +: 32] = w_phy_tx_data_reg5_l2;
    assign phy_ctrl_writein_l2_o[31*32 +: 32] = w_phy_tx_data_reg6_l2;
    assign phy_ctrl_writein_l2_o[32*32 +: 32] = w_phy_tx_data_reg7_l2;
    assign phy_ctrl_writein_l2_o[33*32 +: 32] = w_phy_tx_ldo_reg1_l2;
    assign phy_ctrl_writein_l2_o[34*32 +: 32] = w_phy_tx_ldo_reg2_l2;
    assign phy_ctrl_writein_l3_o[0*32 +: 32] = w_phy_trx_reg1_l3;
    assign phy_ctrl_writein_l3_o[1*32 +: 32] = w_phy_trx_reg2_l3;
    assign phy_ctrl_writein_l3_o[2*32 +: 32] = w_phy_trx_test_reg1_l3;
    assign phy_ctrl_writein_l3_o[3*32 +: 32] = w_phy_trx_test_reg2_l3;
    assign phy_ctrl_writein_l3_o[4*32 +: 32] = w_phy_rx_clk_eng_reg1_l3;
    assign phy_ctrl_writein_l3_o[5*32 +: 32] = w_phy_rx_clk_eng_reg2_l3;
    assign phy_ctrl_writein_l3_o[6*32 +: 32] = w_phy_rx_clk_eng_reg3_l3;
    assign phy_ctrl_writein_l3_o[7*32 +: 32] = w_phy_rx_clk_eng_reg4_l3;
    assign phy_ctrl_writein_l3_o[8*32 +: 32] = w_phy_rx_clk_reg1_l3;
    assign phy_ctrl_writein_l3_o[9*32 +: 32] = w_phy_rx_clk_reg2_l3;
    assign phy_ctrl_writein_l3_o[10*32 +: 32] = w_phy_rx_clk_reg3_l3;
    assign phy_ctrl_writein_l3_o[11*32 +: 32] = w_phy_rx_clk_reg4_l3;
    assign phy_ctrl_writein_l3_o[12*32 +: 32] = w_phy_rx_data_reg1_l3;
    assign phy_ctrl_writein_l3_o[13*32 +: 32] = w_phy_rx_data_reg2_l3;
    assign phy_ctrl_writein_l3_o[14*32 +: 32] = w_phy_rx_data_reg3_l3;
    assign phy_ctrl_writein_l3_o[15*32 +: 32] = w_phy_rx_ldo_reg1_l3;
    assign phy_ctrl_writein_l3_o[16*32 +: 32] = w_phy_rx_ldo_reg2_l3;
    assign phy_ctrl_writein_l3_o[17*32 +: 32] = w_phy_tx_clk_eng_reg1_l3;
    assign phy_ctrl_writein_l3_o[18*32 +: 32] = w_phy_tx_clk_eng_reg2_l3;
    assign phy_ctrl_writein_l3_o[19*32 +: 32] = w_phy_tx_clk_eng_reg3_l3;
    assign phy_ctrl_writein_l3_o[20*32 +: 32] = w_phy_tx_clk_eng_reg4_l3;
    assign phy_ctrl_writein_l3_o[21*32 +: 32] = w_phy_tx_clk_reg1_l3;
    assign phy_ctrl_writein_l3_o[22*32 +: 32] = w_phy_tx_clk_reg2_l3;
    assign phy_ctrl_writein_l3_o[23*32 +: 32] = w_phy_tx_clk_reg3_l3;
    assign phy_ctrl_writein_l3_o[24*32 +: 32] = w_phy_tx_clk_reg4_l3;
    assign phy_ctrl_writein_l3_o[25*32 +: 32] = w_phy_tx_clk_reg5_l3;
    assign phy_ctrl_writein_l3_o[26*32 +: 32] = w_phy_tx_data_reg1_l3;
    assign phy_ctrl_writein_l3_o[27*32 +: 32] = w_phy_tx_data_reg2_l3;
    assign phy_ctrl_writein_l3_o[28*32 +: 32] = w_phy_tx_data_reg3_l3;
    assign phy_ctrl_writein_l3_o[29*32 +: 32] = w_phy_tx_data_reg4_l3;
    assign phy_ctrl_writein_l3_o[30*32 +: 32] = w_phy_tx_data_reg5_l3;
    assign phy_ctrl_writein_l3_o[31*32 +: 32] = w_phy_tx_data_reg6_l3;
    assign phy_ctrl_writein_l3_o[32*32 +: 32] = w_phy_tx_data_reg7_l3;
    assign phy_ctrl_writein_l3_o[33*32 +: 32] = w_phy_tx_ldo_reg1_l3;
    assign phy_ctrl_writein_l3_o[34*32 +: 32] = w_phy_tx_ldo_reg2_l3;

    // ==========================================================================
    // 族 a lane0 per-IO 训练参数输出（MDE-CHAIN-RERUN-01-271-IMPL, gen_mech 生成）
    // ==========================================================================
    assign offset_param_l0_i0_offset = w_offset_param_l0_i0;
    assign vref_param_l0_i0_vref = w_vref_param_l0_i0;
    assign pi_param_l0_i0_pi = w_pi_param_l0_i0;
    assign dcdl_param_l0_i0_dcdl = w_dcdl_param_l0_i0;
    assign offset_param_l0_i1_offset = w_offset_param_l0_i1;
    assign vref_param_l0_i1_vref = w_vref_param_l0_i1;
    assign pi_param_l0_i1_pi = w_pi_param_l0_i1;
    assign dcdl_param_l0_i1_dcdl = w_dcdl_param_l0_i1;
    assign offset_param_l0_i2_offset = w_offset_param_l0_i2;
    assign vref_param_l0_i2_vref = w_vref_param_l0_i2;
    assign pi_param_l0_i2_pi = w_pi_param_l0_i2;
    assign dcdl_param_l0_i2_dcdl = w_dcdl_param_l0_i2;
    assign offset_param_l0_i3_offset = w_offset_param_l0_i3;
    assign vref_param_l0_i3_vref = w_vref_param_l0_i3;
    assign pi_param_l0_i3_pi = w_pi_param_l0_i3;
    assign dcdl_param_l0_i3_dcdl = w_dcdl_param_l0_i3;
    assign offset_param_l0_i4_offset = w_offset_param_l0_i4;
    assign vref_param_l0_i4_vref = w_vref_param_l0_i4;
    assign pi_param_l0_i4_pi = w_pi_param_l0_i4;
    assign dcdl_param_l0_i4_dcdl = w_dcdl_param_l0_i4;
    assign offset_param_l0_i5_offset = w_offset_param_l0_i5;
    assign vref_param_l0_i5_vref = w_vref_param_l0_i5;
    assign pi_param_l0_i5_pi = w_pi_param_l0_i5;
    assign dcdl_param_l0_i5_dcdl = w_dcdl_param_l0_i5;
    assign offset_param_l0_i6_offset = w_offset_param_l0_i6;
    assign vref_param_l0_i6_vref = w_vref_param_l0_i6;
    assign pi_param_l0_i6_pi = w_pi_param_l0_i6;
    assign dcdl_param_l0_i6_dcdl = w_dcdl_param_l0_i6;
    assign offset_param_l0_i7_offset = w_offset_param_l0_i7;
    assign vref_param_l0_i7_vref = w_vref_param_l0_i7;
    assign pi_param_l0_i7_pi = w_pi_param_l0_i7;
    assign dcdl_param_l0_i7_dcdl = w_dcdl_param_l0_i7;
    assign offset_param_l0_i8_offset = w_offset_param_l0_i8;
    assign vref_param_l0_i8_vref = w_vref_param_l0_i8;
    assign pi_param_l0_i8_pi = w_pi_param_l0_i8;
    assign dcdl_param_l0_i8_dcdl = w_dcdl_param_l0_i8;
    assign offset_param_l0_i9_offset = w_offset_param_l0_i9;
    assign vref_param_l0_i9_vref = w_vref_param_l0_i9;
    assign pi_param_l0_i9_pi = w_pi_param_l0_i9;
    assign dcdl_param_l0_i9_dcdl = w_dcdl_param_l0_i9;
    assign offset_param_l0_i10_offset = w_offset_param_l0_i10;
    assign vref_param_l0_i10_vref = w_vref_param_l0_i10;
    assign pi_param_l0_i10_pi = w_pi_param_l0_i10;
    assign dcdl_param_l0_i10_dcdl = w_dcdl_param_l0_i10;
    assign offset_param_l0_i11_offset = w_offset_param_l0_i11;
    assign vref_param_l0_i11_vref = w_vref_param_l0_i11;
    assign pi_param_l0_i11_pi = w_pi_param_l0_i11;
    assign dcdl_param_l0_i11_dcdl = w_dcdl_param_l0_i11;
    assign offset_param_l0_i12_offset = w_offset_param_l0_i12;
    assign vref_param_l0_i12_vref = w_vref_param_l0_i12;
    assign pi_param_l0_i12_pi = w_pi_param_l0_i12;
    assign dcdl_param_l0_i12_dcdl = w_dcdl_param_l0_i12;
    assign offset_param_l0_i13_offset = w_offset_param_l0_i13;
    assign vref_param_l0_i13_vref = w_vref_param_l0_i13;
    assign pi_param_l0_i13_pi = w_pi_param_l0_i13;
    assign dcdl_param_l0_i13_dcdl = w_dcdl_param_l0_i13;
    assign offset_param_l0_i14_offset = w_offset_param_l0_i14;
    assign vref_param_l0_i14_vref = w_vref_param_l0_i14;
    assign pi_param_l0_i14_pi = w_pi_param_l0_i14;
    assign dcdl_param_l0_i14_dcdl = w_dcdl_param_l0_i14;
    assign offset_param_l0_i15_offset = w_offset_param_l0_i15;
    assign vref_param_l0_i15_vref = w_vref_param_l0_i15;
    assign pi_param_l0_i15_pi = w_pi_param_l0_i15;
    assign dcdl_param_l0_i15_dcdl = w_dcdl_param_l0_i15;
    assign offset_param_l0_i16_offset = w_offset_param_l0_i16;
    assign dcdl_param_l0_i16_dcdl = w_dcdl_param_l0_i16;
    // MDE-CHAIN-1008 lane1 train 参数 assign (2026-10-12)
    assign dcdl_param_l1_i0_dcdl = w_dcdl_param_l1_i0;
    assign dcdl_param_l1_i1_dcdl = w_dcdl_param_l1_i1;
    assign dcdl_param_l1_i2_dcdl = w_dcdl_param_l1_i2;
    assign dcdl_param_l1_i3_dcdl = w_dcdl_param_l1_i3;
    assign dcdl_param_l1_i4_dcdl = w_dcdl_param_l1_i4;
    assign dcdl_param_l1_i5_dcdl = w_dcdl_param_l1_i5;
    assign dcdl_param_l1_i6_dcdl = w_dcdl_param_l1_i6;
    assign dcdl_param_l1_i7_dcdl = w_dcdl_param_l1_i7;
    assign dcdl_param_l1_i8_dcdl = w_dcdl_param_l1_i8;
    assign dcdl_param_l1_i9_dcdl = w_dcdl_param_l1_i9;
    assign dcdl_param_l1_i10_dcdl = w_dcdl_param_l1_i10;
    assign dcdl_param_l1_i11_dcdl = w_dcdl_param_l1_i11;
    assign dcdl_param_l1_i12_dcdl = w_dcdl_param_l1_i12;
    assign dcdl_param_l1_i13_dcdl = w_dcdl_param_l1_i13;
    assign dcdl_param_l1_i14_dcdl = w_dcdl_param_l1_i14;
    assign dcdl_param_l1_i15_dcdl = w_dcdl_param_l1_i15;
    assign dcdl_param_l1_i16_dcdl = w_dcdl_param_l1_i16;
    assign offset_param_l1_i0_offset = w_offset_param_l1_i0;
    assign offset_param_l1_i1_offset = w_offset_param_l1_i1;
    assign offset_param_l1_i2_offset = w_offset_param_l1_i2;
    assign offset_param_l1_i3_offset = w_offset_param_l1_i3;
    assign offset_param_l1_i4_offset = w_offset_param_l1_i4;
    assign offset_param_l1_i5_offset = w_offset_param_l1_i5;
    assign offset_param_l1_i6_offset = w_offset_param_l1_i6;
    assign offset_param_l1_i7_offset = w_offset_param_l1_i7;
    assign offset_param_l1_i8_offset = w_offset_param_l1_i8;
    assign offset_param_l1_i9_offset = w_offset_param_l1_i9;
    assign offset_param_l1_i10_offset = w_offset_param_l1_i10;
    assign offset_param_l1_i11_offset = w_offset_param_l1_i11;
    assign offset_param_l1_i12_offset = w_offset_param_l1_i12;
    assign offset_param_l1_i13_offset = w_offset_param_l1_i13;
    assign offset_param_l1_i14_offset = w_offset_param_l1_i14;
    assign offset_param_l1_i15_offset = w_offset_param_l1_i15;
    assign offset_param_l1_i16_offset = w_offset_param_l1_i16;
    assign pi_param_l1_i0_pi = w_pi_param_l1_i0;
    assign pi_param_l1_i1_pi = w_pi_param_l1_i1;
    assign pi_param_l1_i2_pi = w_pi_param_l1_i2;
    assign pi_param_l1_i3_pi = w_pi_param_l1_i3;
    assign pi_param_l1_i4_pi = w_pi_param_l1_i4;
    assign pi_param_l1_i5_pi = w_pi_param_l1_i5;
    assign pi_param_l1_i6_pi = w_pi_param_l1_i6;
    assign pi_param_l1_i7_pi = w_pi_param_l1_i7;
    assign pi_param_l1_i8_pi = w_pi_param_l1_i8;
    assign pi_param_l1_i9_pi = w_pi_param_l1_i9;
    assign pi_param_l1_i10_pi = w_pi_param_l1_i10;
    assign pi_param_l1_i11_pi = w_pi_param_l1_i11;
    assign pi_param_l1_i12_pi = w_pi_param_l1_i12;
    assign pi_param_l1_i13_pi = w_pi_param_l1_i13;
    assign pi_param_l1_i14_pi = w_pi_param_l1_i14;
    assign pi_param_l1_i15_pi = w_pi_param_l1_i15;
    assign vref_param_l1_i0_vref = w_vref_param_l1_i0;
    assign vref_param_l1_i1_vref = w_vref_param_l1_i1;
    assign vref_param_l1_i2_vref = w_vref_param_l1_i2;
    assign vref_param_l1_i3_vref = w_vref_param_l1_i3;
    assign vref_param_l1_i4_vref = w_vref_param_l1_i4;
    assign vref_param_l1_i5_vref = w_vref_param_l1_i5;
    assign vref_param_l1_i6_vref = w_vref_param_l1_i6;
    assign vref_param_l1_i7_vref = w_vref_param_l1_i7;
    assign vref_param_l1_i8_vref = w_vref_param_l1_i8;
    assign vref_param_l1_i9_vref = w_vref_param_l1_i9;
    assign vref_param_l1_i10_vref = w_vref_param_l1_i10;
    assign vref_param_l1_i11_vref = w_vref_param_l1_i11;
    assign vref_param_l1_i12_vref = w_vref_param_l1_i12;
    assign vref_param_l1_i13_vref = w_vref_param_l1_i13;
    assign vref_param_l1_i14_vref = w_vref_param_l1_i14;
    assign vref_param_l1_i15_vref = w_vref_param_l1_i15;

    // MDE-CHAIN-1008 lane2 train 参数 assign (2026-10-12)
    assign dcdl_param_l2_i0_dcdl = w_dcdl_param_l2_i0;
    assign dcdl_param_l2_i1_dcdl = w_dcdl_param_l2_i1;
    assign dcdl_param_l2_i2_dcdl = w_dcdl_param_l2_i2;
    assign dcdl_param_l2_i3_dcdl = w_dcdl_param_l2_i3;
    assign dcdl_param_l2_i4_dcdl = w_dcdl_param_l2_i4;
    assign dcdl_param_l2_i5_dcdl = w_dcdl_param_l2_i5;
    assign dcdl_param_l2_i6_dcdl = w_dcdl_param_l2_i6;
    assign dcdl_param_l2_i7_dcdl = w_dcdl_param_l2_i7;
    assign dcdl_param_l2_i8_dcdl = w_dcdl_param_l2_i8;
    assign dcdl_param_l2_i9_dcdl = w_dcdl_param_l2_i9;
    assign dcdl_param_l2_i10_dcdl = w_dcdl_param_l2_i10;
    assign dcdl_param_l2_i11_dcdl = w_dcdl_param_l2_i11;
    assign dcdl_param_l2_i12_dcdl = w_dcdl_param_l2_i12;
    assign dcdl_param_l2_i13_dcdl = w_dcdl_param_l2_i13;
    assign dcdl_param_l2_i14_dcdl = w_dcdl_param_l2_i14;
    assign dcdl_param_l2_i15_dcdl = w_dcdl_param_l2_i15;
    assign dcdl_param_l2_i16_dcdl = w_dcdl_param_l2_i16;
    assign offset_param_l2_i0_offset = w_offset_param_l2_i0;
    assign offset_param_l2_i1_offset = w_offset_param_l2_i1;
    assign offset_param_l2_i2_offset = w_offset_param_l2_i2;
    assign offset_param_l2_i3_offset = w_offset_param_l2_i3;
    assign offset_param_l2_i4_offset = w_offset_param_l2_i4;
    assign offset_param_l2_i5_offset = w_offset_param_l2_i5;
    assign offset_param_l2_i6_offset = w_offset_param_l2_i6;
    assign offset_param_l2_i7_offset = w_offset_param_l2_i7;
    assign offset_param_l2_i8_offset = w_offset_param_l2_i8;
    assign offset_param_l2_i9_offset = w_offset_param_l2_i9;
    assign offset_param_l2_i10_offset = w_offset_param_l2_i10;
    assign offset_param_l2_i11_offset = w_offset_param_l2_i11;
    assign offset_param_l2_i12_offset = w_offset_param_l2_i12;
    assign offset_param_l2_i13_offset = w_offset_param_l2_i13;
    assign offset_param_l2_i14_offset = w_offset_param_l2_i14;
    assign offset_param_l2_i15_offset = w_offset_param_l2_i15;
    assign offset_param_l2_i16_offset = w_offset_param_l2_i16;
    assign pi_param_l2_i0_pi = w_pi_param_l2_i0;
    assign pi_param_l2_i1_pi = w_pi_param_l2_i1;
    assign pi_param_l2_i2_pi = w_pi_param_l2_i2;
    assign pi_param_l2_i3_pi = w_pi_param_l2_i3;
    assign pi_param_l2_i4_pi = w_pi_param_l2_i4;
    assign pi_param_l2_i5_pi = w_pi_param_l2_i5;
    assign pi_param_l2_i6_pi = w_pi_param_l2_i6;
    assign pi_param_l2_i7_pi = w_pi_param_l2_i7;
    assign pi_param_l2_i8_pi = w_pi_param_l2_i8;
    assign pi_param_l2_i9_pi = w_pi_param_l2_i9;
    assign pi_param_l2_i10_pi = w_pi_param_l2_i10;
    assign pi_param_l2_i11_pi = w_pi_param_l2_i11;
    assign pi_param_l2_i12_pi = w_pi_param_l2_i12;
    assign pi_param_l2_i13_pi = w_pi_param_l2_i13;
    assign pi_param_l2_i14_pi = w_pi_param_l2_i14;
    assign pi_param_l2_i15_pi = w_pi_param_l2_i15;
    assign vref_param_l2_i0_vref = w_vref_param_l2_i0;
    assign vref_param_l2_i1_vref = w_vref_param_l2_i1;
    assign vref_param_l2_i2_vref = w_vref_param_l2_i2;
    assign vref_param_l2_i3_vref = w_vref_param_l2_i3;
    assign vref_param_l2_i4_vref = w_vref_param_l2_i4;
    assign vref_param_l2_i5_vref = w_vref_param_l2_i5;
    assign vref_param_l2_i6_vref = w_vref_param_l2_i6;
    assign vref_param_l2_i7_vref = w_vref_param_l2_i7;
    assign vref_param_l2_i8_vref = w_vref_param_l2_i8;
    assign vref_param_l2_i9_vref = w_vref_param_l2_i9;
    assign vref_param_l2_i10_vref = w_vref_param_l2_i10;
    assign vref_param_l2_i11_vref = w_vref_param_l2_i11;
    assign vref_param_l2_i12_vref = w_vref_param_l2_i12;
    assign vref_param_l2_i13_vref = w_vref_param_l2_i13;
    assign vref_param_l2_i14_vref = w_vref_param_l2_i14;
    assign vref_param_l2_i15_vref = w_vref_param_l2_i15;

    // MDE-CHAIN-1008 lane3 train 参数 assign (2026-10-12)
    assign dcdl_param_l3_i0_dcdl = w_dcdl_param_l3_i0;
    assign dcdl_param_l3_i1_dcdl = w_dcdl_param_l3_i1;
    assign dcdl_param_l3_i2_dcdl = w_dcdl_param_l3_i2;
    assign dcdl_param_l3_i3_dcdl = w_dcdl_param_l3_i3;
    assign dcdl_param_l3_i4_dcdl = w_dcdl_param_l3_i4;
    assign dcdl_param_l3_i5_dcdl = w_dcdl_param_l3_i5;
    assign dcdl_param_l3_i6_dcdl = w_dcdl_param_l3_i6;
    assign dcdl_param_l3_i7_dcdl = w_dcdl_param_l3_i7;
    assign dcdl_param_l3_i8_dcdl = w_dcdl_param_l3_i8;
    assign dcdl_param_l3_i9_dcdl = w_dcdl_param_l3_i9;
    assign dcdl_param_l3_i10_dcdl = w_dcdl_param_l3_i10;
    assign dcdl_param_l3_i11_dcdl = w_dcdl_param_l3_i11;
    assign dcdl_param_l3_i12_dcdl = w_dcdl_param_l3_i12;
    assign dcdl_param_l3_i13_dcdl = w_dcdl_param_l3_i13;
    assign dcdl_param_l3_i14_dcdl = w_dcdl_param_l3_i14;
    assign dcdl_param_l3_i15_dcdl = w_dcdl_param_l3_i15;
    assign dcdl_param_l3_i16_dcdl = w_dcdl_param_l3_i16;
    assign offset_param_l3_i0_offset = w_offset_param_l3_i0;
    assign offset_param_l3_i1_offset = w_offset_param_l3_i1;
    assign offset_param_l3_i2_offset = w_offset_param_l3_i2;
    assign offset_param_l3_i3_offset = w_offset_param_l3_i3;
    assign offset_param_l3_i4_offset = w_offset_param_l3_i4;
    assign offset_param_l3_i5_offset = w_offset_param_l3_i5;
    assign offset_param_l3_i6_offset = w_offset_param_l3_i6;
    assign offset_param_l3_i7_offset = w_offset_param_l3_i7;
    assign offset_param_l3_i8_offset = w_offset_param_l3_i8;
    assign offset_param_l3_i9_offset = w_offset_param_l3_i9;
    assign offset_param_l3_i10_offset = w_offset_param_l3_i10;
    assign offset_param_l3_i11_offset = w_offset_param_l3_i11;
    assign offset_param_l3_i12_offset = w_offset_param_l3_i12;
    assign offset_param_l3_i13_offset = w_offset_param_l3_i13;
    assign offset_param_l3_i14_offset = w_offset_param_l3_i14;
    assign offset_param_l3_i15_offset = w_offset_param_l3_i15;
    assign offset_param_l3_i16_offset = w_offset_param_l3_i16;
    assign pi_param_l3_i0_pi = w_pi_param_l3_i0;
    assign pi_param_l3_i1_pi = w_pi_param_l3_i1;
    assign pi_param_l3_i2_pi = w_pi_param_l3_i2;
    assign pi_param_l3_i3_pi = w_pi_param_l3_i3;
    assign pi_param_l3_i4_pi = w_pi_param_l3_i4;
    assign pi_param_l3_i5_pi = w_pi_param_l3_i5;
    assign pi_param_l3_i6_pi = w_pi_param_l3_i6;
    assign pi_param_l3_i7_pi = w_pi_param_l3_i7;
    assign pi_param_l3_i8_pi = w_pi_param_l3_i8;
    assign pi_param_l3_i9_pi = w_pi_param_l3_i9;
    assign pi_param_l3_i10_pi = w_pi_param_l3_i10;
    assign pi_param_l3_i11_pi = w_pi_param_l3_i11;
    assign pi_param_l3_i12_pi = w_pi_param_l3_i12;
    assign pi_param_l3_i13_pi = w_pi_param_l3_i13;
    assign pi_param_l3_i14_pi = w_pi_param_l3_i14;
    assign pi_param_l3_i15_pi = w_pi_param_l3_i15;
    assign vref_param_l3_i0_vref = w_vref_param_l3_i0;
    assign vref_param_l3_i1_vref = w_vref_param_l3_i1;
    assign vref_param_l3_i2_vref = w_vref_param_l3_i2;
    assign vref_param_l3_i3_vref = w_vref_param_l3_i3;
    assign vref_param_l3_i4_vref = w_vref_param_l3_i4;
    assign vref_param_l3_i5_vref = w_vref_param_l3_i5;
    assign vref_param_l3_i6_vref = w_vref_param_l3_i6;
    assign vref_param_l3_i7_vref = w_vref_param_l3_i7;
    assign vref_param_l3_i8_vref = w_vref_param_l3_i8;
    assign vref_param_l3_i9_vref = w_vref_param_l3_i9;
    assign vref_param_l3_i10_vref = w_vref_param_l3_i10;
    assign vref_param_l3_i11_vref = w_vref_param_l3_i11;
    assign vref_param_l3_i12_vref = w_vref_param_l3_i12;
    assign vref_param_l3_i13_vref = w_vref_param_l3_i13;
    assign vref_param_l3_i14_vref = w_vref_param_l3_i14;
    assign vref_param_l3_i15_vref = w_vref_param_l3_i15;

    assign train_param_src_o = w_train_param_src_o;
    assign datacheck_cfg_fail_threshold_o = w_datacheck_fail_threshold_o;
    assign eq_param_ctle_tap_o = w_eq_param_ctle_tap_o;
    assign phy_trx_en_l0_o = w_phy_trx_en_l0;
    assign phy_trx_en_l1_o = w_phy_trx_en_l1;
    assign phy_trx_en_l2_o = w_phy_trx_en_l2;
    assign phy_trx_en_l3_o = w_phy_trx_en_l3;
    assign sbc_txr_en_o = w_sbc_txr_en;
    assign sbc_rxr_sel_o = w_sbc_rxr_sel;

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================


    // 7 类中断源 OR 聚合（同步后 apb_pclk 域，跨 4 lane; 与原 RTL 一致）
    assign int_raw[0] = lane_irq_sync2[0]  | lane_irq_sync2[4]  |
                        lane_irq_sync2[8]  | lane_irq_sync2[12];
    assign int_raw[1] = lane_irq_sync2[1]  | lane_irq_sync2[5]  |
                        lane_irq_sync2[9]  | lane_irq_sync2[13];
    assign int_raw[2] = lane_irq_sync2[2]  | lane_irq_sync2[6]  |
                        lane_irq_sync2[10] | lane_irq_sync2[14];
    assign int_raw[3] = lane_irq_sync2[3]  | lane_irq_sync2[7]  |
                        lane_irq_sync2[11] | lane_irq_sync2[15];
    assign int_raw[4] = sideband_irq_sync2[0];
    assign int_raw[5] = sideband_irq_sync2[1];
    assign int_raw[6] = (|cdc_fifo_ovf_reg) | (|cdc_fifo_unf_reg);

    // 读数据 mux：rf.reg_rdata + 5 处 override（wrapper 自持状态, 设计说明 2）
    //   0x004 整体 override：MODE[0] 本体在 wrapper（strap 锁存），lane_num / analog_ser_ratio
    //   亦由 wrapper 派生（LANE_NUM / w_ser_ratio），不复用 rf 侧 RO 端口。
    // --------------------------------------------------------------------------
    always @(*) begin
        case (apb_paddr)
            16'h0000: prdata_mux = {rf_reg_rdata[31:5], global_rst_n_reg,
                                    rf_reg_rdata[3:0]};
            16'h0004: prdata_mux = {16'h0, w_ser_ratio, 6'h0, LANE_NUM[3:0],
                                    3'h0, w_io_mode};
            16'h000C: prdata_mux = {24'b0, int_mask_audit_fl, int_mask_mcu_reg};
            16'h0010: prdata_mux = {24'b0, int_mask_audit_fl, int_mask_ext_reg};
            16'h0820: prdata_mux = {rf_reg_rdata[31:2],
                                    sbc_disp_sync2[1], sbc_disp_sync2[0]};
            default:  prdata_mux = rf_reg_rdata;
        endcase
    end

    // 读数据组合直出（ACCESS 拍有效，与 pready 同拍；读 2 cycle = SETUP + ACCESS，无 wait state）
    //   非 ACCESS 拍强制 0：避免 SETUP 拍地址切换串扰到 PRDATA（与 top/hipi_ctrl_top.v 同族口径）
    assign apb_prdata = apb_rd_en ? prdata_mux : 32'h0;

    // pslverr（与原 RTL 一致: 非 4B 对齐 / 越界 / RO 镜像写 0x708/0x818）
    always @(*) begin
        pslverr_cond = 1'b0;
        if (!addr_aligned || !addr_valid) begin
            pslverr_cond = 1'b1;
        end
        else if (apb_pwrite &&
                 ((apb_paddr == 16'h0708) || (apb_paddr == 16'h0818))) begin
            pslverr_cond = 1'b1;
        end
        else if (apb_pwrite && (apb_paddr == 16'h0004)) begin
            // RWS: IO_DATA_WIDTH_MODE 为 bring-up strap 锁存值（启动后只读）→ 运行时写报错且不更新
            pslverr_cond = 1'b1;
        end
    end
    assign apb_pslverr = pslverr_cond & apb_psel & apb_penable;

    // ==========================================================================
    // 时序逻辑（wrapper 自持状态, 与原 RTL 逐块一致）
    // ==========================================================================

    // bring-up strap 一次性采样（复位期归 0；释放后首沿锁存 strap，之后永不更新）
    //   每 always 块只赋 1 个信号（rtl-coding §6.1）
    always @(posedge apb_pclk or negedge apb_preset_n) begin
        if (!apb_preset_n) begin
            io_data_width_mode_latched <= 1'b0;
        end
        else begin
            io_data_width_mode_latched <= 1'b1;   // 复位释放后恒 1（一次性窗口）
        end
    end

    always @(posedge apb_pclk or negedge apb_preset_n) begin
        if (!apb_preset_n) begin
            io_data_width_mode_reg <= 1'b0;
        end
        else if (!io_data_width_mode_latched) begin
            io_data_width_mode_reg <= io_data_width_mode_strap;   // 复位释放首沿锁存
        end
        // else: 保持（bring-up 锁存，启动后只读 —— RWS）
    end

    // int mask 镜像：0x00C/0x900、0x010/0x904 双入口写同一寄存器（复位全屏蔽）
    always @(posedge apb_pclk or negedge apb_preset_n) begin
        if (!apb_preset_n) begin
            int_mask_mcu_reg <= 7'h7F;
            int_mask_ext_reg <= 7'h7F;
        end
        else begin
            if (apb_wr_en && ((apb_paddr == 16'h000C) || (apb_paddr == 16'h0900))) begin
                int_mask_mcu_reg <= apb_pwdata[6:0];
            end
            if (apb_wr_en && ((apb_paddr == 16'h0010) || (apb_paddr == 16'h0904))) begin
                int_mask_ext_reg <= apb_pwdata[6:0];
            end
        end
    end

    // 族 d（§4.1, MDE-CHAIN-RERUN-01-271-IMPL）：int_mask 一致性审计——rf 单入口存储
    // （0x00C 写）vs wrapper 双入口权威镜像（0x00C+0x900）；不一致置 sticky 可读位
    // （读回 0x000C/0x0010 bit7, 原 reserved=0; 供验证交叉核对 0x900 写透语义）
    always @(posedge apb_pclk or negedge apb_preset_n) begin
        if (!apb_preset_n) begin
            int_mask_audit_fl <= 1'b0;
        end
        else if (w_int_mask_mcu_mask_mcu != int_mask_mcu_reg ||
                 w_int_mask_ext_mask_ext != int_mask_ext_reg) begin
            int_mask_audit_fl <= 1'b1;
        end
    end

    // TOP_CTRL.global_rst_n：写 0 持续 1 cycle 后自动恢复 1（rf 副本无自恢复,
    // 读回 0x000[4] 用本寄存器 override; train_mode/force_mode_switch 已在 rf）
    // 族 c #1（MDE-CHAIN-RERUN-01-271-IMPL）：写源采样 rf 副本 top_ctrl_global_rst_n
    // （wr_0x000_d1 延迟一拍与 rf 存储更新同拍）, 自恢复/读回语义保持（单权威自恢复）
    reg  wr_0x000_d1;
    always @(posedge apb_pclk or negedge apb_preset_n) begin
        if (!apb_preset_n) begin
            wr_0x000_d1 <= 1'b0;
        end
        else begin
            wr_0x000_d1 <= apb_wr_en && (apb_paddr == 16'h0000);
        end
    end
    always @(posedge apb_pclk or negedge apb_preset_n) begin
        if (!apb_preset_n) begin
            global_rst_n_reg <= 1'b1;
        end
        else if (wr_0x000_d1) begin
            global_rst_n_reg <= w_top_ctrl_global_rst_n;
        end
        else if (!global_rst_n_reg) begin
            global_rst_n_reg <= 1'b1;
        end
    end

    // CDC_FIFO_OVF / UNF sticky（17 bit, 写 1 清; set 源 TODO tie 0, 与原 RTL 一致）
    // 族 c #8（MDE-CHAIN-RERUN-01-271-IMPL）：cdc_fifo_rst 脉冲显式消费=清 ovf/unf
    // status（FIFO 复位清状态, 语义合理; 审计计数替代方案见 271-IMPL 回报）
    always @(posedge apb_pclk or negedge apb_preset_n) begin
        if (!apb_preset_n) begin
            cdc_fifo_ovf_reg <= 17'h0;
            cdc_fifo_unf_reg <= 17'h0;
        end
        else begin
            for (ci = 0; ci < 17; ci = ci + 1) begin
                if (apb_wr_en && apb_paddr == 16'h0020 && apb_pwdata[ci]) begin
                    cdc_fifo_ovf_reg[ci] <= 1'b0;
                end
                if (apb_wr_en && apb_paddr == 16'h0024 && apb_pwdata[ci]) begin
                    cdc_fifo_unf_reg[ci] <= 1'b0;
                end
                if (w_cdc_fifo_rst_wp) begin
                    cdc_fifo_ovf_reg[ci] <= 1'b0;
                    cdc_fifo_unf_reg[ci] <= 1'b0;
                end
            end
        end
    end

    // INT_STATUS sticky（set = int_raw, clear = INT_CLEAR 写 1, 置位优先; 原 RTL 一致）
    // 族 b（MDE-CHAIN-RERUN-01-271-IMPL）：清位源改 rf int_clear/intc_clear wp/wd 单驱
    always @(posedge apb_pclk or negedge apb_preset_n) begin
        if (!apb_preset_n) begin
            int_status_reg <= 7'h00;
        end
        else begin
            for (ii = 0; ii < 7; ii = ii + 1) begin
                if ((w_int_clear_wp && w_int_clear_wd[ii]) ||
                    (w_intc_clear_wp && w_intc_clear_wd[ii])) begin
                    int_status_reg[ii] <= int_raw[ii];
                end
                else if (int_raw[ii]) begin
                    int_status_reg[ii] <= 1'b1;
                end
            end
        end
    end

    // lane2 data_path 影子 alias（0x400/0x404/0x440/0x444/0x448 双写; 与原 RTL
    // per-lane 写解码一致: idx=2 < LANE_NUM 时 per-lane 块同步更新, 见设计说明 3）
    wire l2_wr_en = apb_wr_en & (LANE_NUM > 2);

    // 族 c #2（MDE-CHAIN-RERUN-01-271-IMPL）：0x400 TRAIN_MODE.mode read-through 影子
    // （lane2 影子 alias 共享段区域, 采样于 0x400 双写拍; 供 TRAIN-MAS-01 消费;
    //  与 train_mode_o(0x000 TOP_CTRL) 独立）
    reg [1:0] l2_train_mode_shd;
    always @(posedge apb_pclk or negedge apb_preset_n) begin
        if (!apb_preset_n) begin
            l2_train_mode_shd <= 2'b00;
        end
        else if (l2_wr_en && apb_paddr == 16'h0400) begin
            l2_train_mode_shd <= w_train_mode_mode;
        end
    end

    always @(posedge apb_pclk or negedge apb_preset_n) begin
        if (!apb_preset_n) begin
            l2_reversal_en_tx <= 1'b0;
            l2_reversal_en_rx <= 1'b0;
            l2_repair_map_tx  <= 16'b0;
            l2_scramble_en_tx <= 1'b1;
            l2_repair_map_rx  <= 16'b0;
            l2_scramble_en_rx <= 1'b1;
            l2_seed_tx_i0     <= 23'h7FFFFF;
            l2_seed_tx_i1     <= 23'h7FFFFF;
            l2_seed_tx_i2     <= 23'h7FFFFF;
        end
        else begin
            if (l2_wr_en && apb_paddr == 16'h0400) begin
                l2_reversal_en_tx <= apb_pwdata[0];
                l2_reversal_en_rx <= apb_pwdata[1];
                l2_repair_map_tx  <= apb_pwdata[17:2];
                l2_scramble_en_tx <= apb_pwdata[18];
            end
            if (l2_wr_en && apb_paddr == 16'h0404) begin
                l2_repair_map_rx  <= apb_pwdata[15:0];
                l2_scramble_en_rx <= apb_pwdata[16];
            end
            if (l2_wr_en && apb_paddr == 16'h0440) l2_seed_tx_i0 <= apb_pwdata[22:0];
            if (l2_wr_en && apb_paddr == 16'h0444) l2_seed_tx_i1 <= apb_pwdata[22:0];
            if (l2_wr_en && apb_paddr == 16'h0448) l2_seed_tx_i2 <= apb_pwdata[22:0];
        end
    end

    // SBC_IRQ_FLAG 清位（族 b 单驱化：清位源归 rf sbc_irq_flag wp/wd 同拍 W1C;
    // 旧自持 sbc_irq_clear_pulse 重复解码已删, 时序/置位优先语义保持）
    //（原自持产生块删除, 见 MDE-CHAIN-RERUN-01-271-IMPL 族 b）

    // SBC_RX_BUF 读触发 rx_manual_read 脉冲（读 0x810 后硬件弹出接收缓冲头, 原 RTL 一致;
    // 与 rf W1S 写脉冲（写 0x808[1]）在输出侧 OR）
    always @(posedge apb_pclk or negedge apb_preset_n) begin
        if (!apb_preset_n) begin
            sbc_rx_rd_pulse <= 1'b0;
        end
        else begin
            sbc_rx_rd_pulse <= 1'b0;
            if (apb_rd_en && apb_paddr == 16'h0810) begin
                sbc_rx_rd_pulse <= 1'b1;
            end
        end
    end

    // SBC 状态 sticky（set = 同步回流, clear = 对应地址写 1, 置位优先; 原 RTL 一致）
    always @(posedge apb_pclk or negedge apb_preset_n) begin
        if (!apb_preset_n) begin
            sbc_tx_err_sticky        <= 1'b0;
            sbc_rx_parity_err_sticky <= 1'b0;
            sbc_rx_gap_err_sticky    <= 1'b0;
            sbc_rx_buf_ovf_sticky    <= 1'b0;
        end
        else begin
            // SBC_TX_STATUS.tx_err（0x804[4]）
            if (sbc_tx_stat_sync2[4]) begin
                sbc_tx_err_sticky <= 1'b1;
            end
            else if (apb_wr_en && apb_paddr == 16'h0804 && apb_pwdata[4]) begin
                sbc_tx_err_sticky <= 1'b0;
            end
            // SBC_RX_STATUS.rx_parity_err（0x80C[5]）
            if (sbc_rx_stat_sync2[5]) begin
                sbc_rx_parity_err_sticky <= 1'b1;
            end
            else if (apb_wr_en && apb_paddr == 16'h080C && apb_pwdata[5]) begin
                sbc_rx_parity_err_sticky <= 1'b0;
            end
            // SBC_RX_STATUS.rx_gap_err（0x80C[6]）
            if (sbc_rx_stat_sync2[6]) begin
                sbc_rx_gap_err_sticky <= 1'b1;
            end
            else if (apb_wr_en && apb_paddr == 16'h080C && apb_pwdata[6]) begin
                sbc_rx_gap_err_sticky <= 1'b0;
            end
            // SBC_RX_STATUS.rx_buf_ovf（0x80C[7]）
            if (sbc_rx_stat_sync2[7]) begin
                sbc_rx_buf_ovf_sticky <= 1'b1;
            end
            else if (apb_wr_en && apb_paddr == 16'h080C && apb_pwdata[7]) begin
                sbc_rx_buf_ovf_sticky <= 1'b0;
            end
        end
    end

    // RX 异步 FIFO ovf/unf sticky（per-lane, set = 同步回流, clear = 写 1, 置位优先;
    // 原 RTL 一致; 4 lane 固定宽度 —— rx_fifo_status_i 端口固定 32 bit）
    always @(posedge apb_pclk or negedge apb_preset_n) begin
        if (!apb_preset_n) begin
            fifo_ovf_sticky <= 4'b0;
            fifo_unf_sticky <= 4'b0;
        end
        else begin
            for (fi = 0; fi < 4; fi = fi + 1) begin
                // per-lane 地址 = 0xC00 + lane×4：bit[3:2] 为 lane 号
                if (rx_fifo_status_sync2[fi*8+6]) begin
                    fifo_ovf_sticky[fi] <= 1'b1;
                end
                else if (apb_wr_en && apb_paddr[15:4] == 12'h0C0 &&
                         apb_paddr[3:2] == fi && apb_pwdata[6]) begin
                    fifo_ovf_sticky[fi] <= 1'b0;
                end
                if (rx_fifo_status_sync2[fi*8+7]) begin
                    fifo_unf_sticky[fi] <= 1'b1;
                end
                else if (apb_wr_en && apb_paddr[15:4] == 12'h0C0 &&
                         apb_paddr[3:2] == fi && apb_pwdata[7]) begin
                    fifo_unf_sticky[fi] <= 1'b0;
                end
            end
        end
    end

    // 状态回流 2 flop 同步器（源域寄存器输出 → apb_pclk, per-bit 独立; 原 RTL 一致）
    always @(posedge apb_pclk or negedge apb_preset_n) begin
        if (!apb_preset_n) begin
            aligned_rx_sync1      <= 16'b0;
            aligned_rx_sync2      <= 16'b0;
            rx_fifo_status_sync1  <= 32'b0;
            rx_fifo_status_sync2  <= 32'b0;
            lane_irq_sync1        <= 16'b0;
            lane_irq_sync2        <= 16'b0;
            sideband_irq_sync1    <= 4'b0;
            sideband_irq_sync2    <= 4'b0;
            sbc_tx_stat_sync1     <= 5'b0;
            sbc_tx_stat_sync2     <= 5'b0;
            sbc_rx_stat_sync1     <= 8'b0;
            sbc_rx_stat_sync2     <= 8'b0;
            sbc_rx_rdata_sync1    <= 16'b0;
            sbc_rx_rdata_sync2    <= 16'b0;
            sbc_disp_sync1        <= 4'b0;
            sbc_disp_sync2        <= 4'b0;
        end
        else begin
            aligned_rx_sync1      <= aligned_rx_i;
            aligned_rx_sync2      <= aligned_rx_sync1;
            rx_fifo_status_sync1  <= rx_fifo_status_i;
            rx_fifo_status_sync2  <= rx_fifo_status_sync1;
            lane_irq_sync1        <= lane_top_irq_i;
            lane_irq_sync2        <= lane_irq_sync1;
            sideband_irq_sync1    <= sideband_irq_i;
            sideband_irq_sync2    <= sideband_irq_sync1;
            sbc_tx_stat_sync1     <= {sbc_tx_err_i, sbc_tx_busy_i, sbc_tx_buf_level_i};
            sbc_tx_stat_sync2     <= sbc_tx_stat_sync1;
            sbc_rx_stat_sync1     <= {sbc_rx_buf_ovf_i, sbc_rx_gap_err_i,
                                      sbc_rx_parity_err_i, sbc_rx_busy_i,
                                      sbc_rx_buf_level_i};
            sbc_rx_stat_sync2     <= sbc_rx_stat_sync1;
            sbc_rx_rdata_sync1    <= sbc_rx_buf_rdata_i;
            sbc_rx_rdata_sync2    <= sbc_rx_rdata_sync1;
            sbc_disp_sync1        <= {sbc_illegal_lid_i, sbc_illegal_opcode_i,
                                      sbc_err_flag_i, sbc_pkt_arrival_flag_i};
            sbc_disp_sync2        <= sbc_disp_sync1;
        end
    end

    // ==========================================================================
    // CDC toggle 请求信号（cdc-resolver 2026-09-13）：软件写脉冲 *_wp 是 1 拍 pclk
    // 脉冲，需横跨到 mb_clk(2GHz)/phy_rx_clk(2GHz)/sb_tx_clk(800MHz) 目的域。
    // *_wp 单拍脉冲可能窄于目的域周期整拍漏采，故统一转成 toggle（电平翻转，源域
    // 寄存），目的域 2FF 同步 + XOR 边沿检测恢复 1 拍脉冲。
    // 注：声明必须在使用(assign)之前（SpyGlass STX_VE_606 要求先声明后使用）。
    // ==========================================================================
    reg         seed_load_rx_tog;
    reg         clear_rx_tog;
    reg         seed_load_tx_tog;
    reg         clear_tx_tog;
    reg         force_mode_switch_tog;
    reg         step_trigger_tog;
    reg         sbc_rx_manual_read_tog;
    reg         sbc_rx_soft_rst_tog;
    reg         sbc_tx_manual_trigger_tog;   // 2026-09-16 补：sbc_tx 手动注入触发（09-13 遗漏）

    // ==========================================================================
    // 输出驱动（具名信号直送, apb_pclk 域寄存器直出; 跨域同步由消费侧完成）
    // ==========================================================================

    // → intc（同域无 CDC; wrapper 镜像/sticky）
    assign int_status_o   = int_status_reg;
    assign int_mask_mcu_o = int_mask_mcu_reg;
    assign int_mask_ext_o = int_mask_ext_reg;

    // → lane_top（rf 本体 + W1S 脉冲）
    assign io_data_width_mode_o  = w_io_mode;
    // CDC 修复：*_wp 为 1 拍 pclk 脉冲，直跨到 mb_clk / phy_rx_clk / sb_tx_clk 目的域
    // 会亚稳态/漏采。输出改走 toggle（目的域 2FF 同步 + XOR 恢复 1 拍脉冲），见下方 toggle 段。
    assign seed_load_rx_o        = seed_load_rx_tog;
    assign clear_rx_o            = clear_rx_tog;
    assign seed_load_tx_o        = {LANE_NUM{seed_load_tx_tog}}; // 广播 4 lane
    assign clear_tx_o            = {LANE_NUM{clear_tx_tog}};     // 广播 4 lane
    assign reorder_delay_step_o  = w_delay_step;

    // → lane_top 训练控制 / pattern
    assign train_mode_o              = w_train_mode;
    assign force_mode_switch_o       = force_mode_switch_tog;
    assign bypass_step_o             = w_bypass_step;
    assign timeout_step_en_o         = w_timeout_step_en;
    assign timeout_threshold_o       = w_timeout_threshold;
    assign data_check_pattern_sel_o  = w_pattern_sel;
    assign vref_pattern_tx_o         = {w_tp_vref_hi, w_tp_vref_lo};
    assign pi_pattern_tx_o           = {w_tp_pi_hi, w_tp_pi_lo};
    assign dcdl_pattern_tx_o         = {w_tp_dcdl_hi, w_tp_dcdl_lo};
    assign reorder_pattern_tx_o      = {w_tp_reorder_hi, w_tp_reorder_lo};
    assign datacheck_pattern_tx_o    = {w_tp_dc_hi, w_tp_dc_lo};
    assign step_trigger_o            = step_trigger_tog;
    assign step_force_end_o          = w_step_force_end;
    assign step_cnt_o                = w_step_cnt;

    // → mcu_subsys cfg broadcast（rf MCU_CTRL 零扩展, 与原版位序一致）
    assign cfg_mcu_o = {23'b0, w_mcu_itcm_wr_en,
                        w_mcu_boot_addr, 3'b0, w_mcu_soft_rst_n};

    // → sideband（rf 本体 + W1S 脉冲; rx_manual_read = 写脉冲 | 读 0x810 脉冲）
    assign sbc_tx_enable_o         = w_sbc_tx_enable;
    assign sbc_tx_manual_trigger_o = sbc_tx_manual_trigger_tog;  // 2026-09-16: wp → toggle（09-13 遗漏补）
    assign sbc_tx_pkt_inject_o     = w_sbc_tx_pkt_inject;
    assign sbc_rx_enable_o         = w_sbc_rx_enable;
    assign sbc_rx_manual_read_o    = sbc_rx_manual_read_tog;  // (w_manual_read | rd_pulse) → toggle
    assign sbc_rx_soft_rst_o       = sbc_rx_soft_rst_tog;
    assign sbc_rx_buf_depth_o      = w_sbc_rx_buf_depth;
    assign sbc_lid_filter_o        = w_sbc_lid_filter;
    assign sbc_opcode_filter_o     = w_sbc_opcode_filter;
    assign sbc_err_mask_o          = w_sbc_err_mask;
    assign sbc_pkt_arrival_mask_o  = w_sbc_pkt_arrival_mask;
    assign sbc_err_irq_mask_o      = w_sbc_err_irq_mask;
    assign sbc_irq_clear_o         = {w_sbc_err_flag_wp & w_sbc_err_flag_wd,
                                      w_sbc_pkt_arr_flag_wp & w_sbc_pkt_arr_flag_wd};
    assign sbc_lane_enable_o       = w_sbc_lane_enable;

    // ==========================================================================
    // CDC 意图（cdc-resolver 2026-09-13）：软件写脉冲 *_wp 是 1 拍 pclk 脉冲，
    // 需要横跨到 mb_clk（2GHz）/ phy_rx_clk（2GHz）/ sb_tx_clk（800MHz）。
    // *_wp 单拍脉冲可能窄于目的域周期整拍漏采，故在此统一转成 **toggle 请求**
    // （电平翻转，源域寄存），目的域 2FF 同步 + XOR 边沿检测恢复 1 拍脉冲。
    // 信号载体语义变化：pulse → toggle（下游 leaf 模块已同步适配）。
    // ==========================================================================
    always @(posedge apb_pclk or negedge apb_preset_n) begin
        if (!apb_preset_n) begin
            seed_load_tx_tog      <= 1'b0;
            seed_load_rx_tog      <= 1'b0;
            clear_tx_tog          <= 1'b0;
            clear_rx_tog          <= 1'b0;
            force_mode_switch_tog <= 1'b0;
            step_trigger_tog      <= 1'b0;
            sbc_rx_soft_rst_tog   <= 1'b0;
            sbc_rx_manual_read_tog<= 1'b0;
            sbc_tx_manual_trigger_tog <= 1'b0;
        end
        else begin
            if (w_seed_load_tx)       seed_load_tx_tog       <= ~seed_load_tx_tog;
            if (w_seed_load_rx)       seed_load_rx_tog       <= ~seed_load_rx_tog;
            if (w_clear_tx)           clear_tx_tog           <= ~clear_tx_tog;
            if (w_clear_rx)           clear_rx_tog           <= ~clear_rx_tog;
            if (w_force_mode_switch)  force_mode_switch_tog  <= ~force_mode_switch_tog;
            if (w_step_trigger)       step_trigger_tog       <= ~step_trigger_tog;
            if (w_sbc_rx_soft_rst)    sbc_rx_soft_rst_tog    <= ~sbc_rx_soft_rst_tog;
            if (w_sbc_rx_manual_read | sbc_rx_rd_pulse)
                                      sbc_rx_manual_read_tog <= ~sbc_rx_manual_read_tog;
            if (w_sbc_tx_trig)        sbc_tx_manual_trigger_tog <= ~sbc_tx_manual_trigger_tog;
        end
    end

endmodule
