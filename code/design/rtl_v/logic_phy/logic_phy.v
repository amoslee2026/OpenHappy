// =============================================================================
// module name：logic_phy
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/logic_phy_spec.md
// =============================================================================
// 功能概述: HiPi 逻辑物理层 per-link 顶层（纯例化连线，无逻辑）：lane_top × LANE_NUM +
//   sideband + apb_csr_top + intc + mcu_subsys（black-box stub）
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-07  OpenHappy Team  初始版本：基于 logic_phy_spec v4.8 集成生成
//  2026-08-10  OpenHappy Team  MB_PIPE_DEPTH 拆分 TX/RX 透传（TX_MB_PIPE_DEPTH /
//                       RX_MB_PIPE_DEPTH），默认值 2 → 1
//  2026-10-12  OpenHappy Team  TX_MB_PIPE_DEPTH / RX_MB_PIPE_DEPTH 默认 1 → 0（同步 spec：
//                       边界 mb_pipe 直通 bypass，逻辑级数最短）
//  2026-08-11  OpenHappy Team  LANE_NUM 默认值改 top 宏派生（LP_LANE_NUM），per-lane
//                       TX/RX 例化使能经 lane_top TX_EN/RX_EN 传入（设计说明 15）
//  2026-08-11  OpenHappy Team  sideband 方向掩码 SB_TX_MASK/SB_RX_MASK +
//                       SBC_LANE_ENABLE 复位值宏派生（设计说明 16）
//  2026-08-13  OpenHappy Team  TX/RX 使能改每 lane 每方向离散宏（LP_TX_LANE{i}_EN /
//                       LP_RX_LANE{i}_EN）：lane_top TX_EN/RX_EN、sideband 掩码、
//                       aligned AND 聚合全部按离散掩码判定，支持每 lane TX/RX
//                       独立开关（可非连续），不再依赖连续 count（设计说明 15）
//  2026-08-11  OpenHappy Team  train_param_to_analog 315→323 bit/lane（CDR_OFFSET_CTRL
//                       8 bit 并入 offset 段，对齐 logicphy_analogphy_intf v1.1）
//  2026-08-11  OpenHappy Team  顶端口按 interface 原文逐 lane 展开（`LP_LANE{1,2,3}_EN
//                       条件编译），内部保留同名 packed 向量供 generate 索引；
//                       CPIF 控制 5 信号（tx/rx mode、rst_n、lpc）随
//                       link_logicphy_intf v1.4 方向修正为输入（原为常数输出）
//  2026-08-14  OpenHappy Team  新增集中式复位同步模块 logic_phy_crg：mb_rst_n/sb_rst_n
//                       各时钟域 async assert + sync deassert 统一分发（假设 6 收敛）。
//                       lane_top mb_rst_n→mb_rstn_sync、rx_rst_n→rx_rstn_sync[]；
//                       sideband sb_rst_n→sb_tx_rstn_sync。CRG test 输入当前单点 tie
//  2026-08-24  OpenHappy Team  MCU/APB 复位域收敛进 logic_phy_crg：新增 u_sync_apb/u_sync_mcu
//                       （async 源 mcu_rst_n），apb_preset_n 改接 apb_rstn_sync，
//                       u_mcu_subsys.mcu_rst_n 改接 mcu_rstn_sync（假设 5 修订）
//  2026-08-28  OpenHappy Team  非对称 4T1R 改造（4 TX + 1 RX，与 logic_phy_defines v0.5 对齐）：
//                        lane_top × LANE_NUM 由 gen_lane generate 循环改为 4 个显式实例
//                        u_lane_top_lane0/1/2/3，TX_EN/RX_EN 直接用常量（lane0: 1/1，
//                        lane1~3: 1/0）；TX 侧 per-lane 配置（prbs_seed/repair_map/
//                        msb_lsb_flip_en/flip_sel/reversal/scramble/seed_load/clear）
//                        ×4 slice 分发，RX 侧单 lane 配置仅 lane0 消费；SB_LANE_NUM 2→4
//                        （sideband/apb_csr_top 并行扩 4 lane，训练握手 LANE_NUM==4==
//                        SB_LANE_NUM 直连）；状态回流 ×4（lane_top_irq_i 8→16 bit，
//                        aligned_agg 聚合遍历 4 lane）；去除已知 2-lane 限制（假设 3/4 更新）
//  2026-09-18  OpenHappy Team  无变更（当日曾误将本文件掩码语句改为宏拼接形式排查
//                        STX_VE_800/491，真因在 sbc_dispatcher/sbc_cpif_event 的
//                        RX_EN_LANE_NUM popcount 1-bit 链加溢出，已恢复本文件原状）
//  2026-10-14  OpenHappy Team  OFFSET_CTRL_DATA 三态落地（MDE-CHAIN-D2DHIPITOP-1008-FM170-TRISTATE）：dig 侧
//                       lane0_RX0~16_OFFSET_CTRL_DATA 由恒驱动 reg 改为 WRITE 作 OE 的三态驱动
//                       （WRITE ? DATA_i : 10'bz），消除 dig 恒强 reg 与 PHY inout 双 active 驱动
//                       （170 multidriven → DC 优化 DFF0X → FM FAIL 根因）；数据源拆包 *_DATA_i
//                       中间 net，接口类型不变（inout 保留，读回路径可用）
//  2026-10-20  OpenHappy Team 1T1R 对称（单 lane）编译修复，支撑 AXI_ASYM_MODE 未定义形态：
//                       (a) SB_TX_MASK/SB_RX_MASK/SB_LANE_EN_RST 改从 4-bit 全宽
//                       LP_TX/RX_EN_MASK4 切片（原从已截断到 [LANE_NUM-1:0] 的
//                       LP_*_EN_BITS 再切 [SB_LANE_NUM-1:0] 越界 → 常量 X →
//                       sbc_dispatcher RX_EN_LANE_NUM popcount 非 elaboration 常量
//                       → VCS Error-[V2KGEUV]）；新增 LP_LANE_EN_MASK4 中间量。
//                       LANE_NUM==4（4T1R/4R1T 基线）时结果与旧实现完全等价。
//                       (b) lane1~3 的 lane_top 例化补 `ifdef LP_LANE{1,2,3}_EN 守卫，
//                       与端口声明侧条件编译对齐（单 lane 下 LANE_NUM=1 时不例化）。
//  2026-10-21  OpenHappy Team  RX 异步 FIFO 状态回流落地（issue #13 方案 C）：lane_top 新增
//                       rx_async_fifo_status[7:0] 输出 → per-lane 聚合
//                       rx_fifo_status_agg[31:0] → apb_csr_top.rx_fifo_status_i，
//                       取消原 32'h0 常量 tie；ovf/unf 本阶段恒 0（假设 12 修订）
// ========================================================================

`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// 设计说明:
//
// 集成假设 / 与 spec、interface 偏差说明:
//   1. 顶层 per-lane 端口按 interface 原文逐 lane 展开（spec-gen §7.2 扁平化）；
//      lane{i} 端口块以 `ifdef LP_LANE{1,2,3}_EN 条件编译（宏由 logic_phy_defines.vh
//      派生；Verilog-2005 端口数不可参数化，端口有无只能按宏判断）。模块内部保留
//      packed 向量（与原 packed 端口同名：phy_rx_clk / cpif_tx_lane_data /
//      cpif_rx_lane_data(+_vld) / tx_analog_pad / rx_analog_pad /
//      train_param_to_analog）供 generate 循环索引，顶端口 ↔ packed 映射集中于
//      "组合逻辑"节首，bit 顺序 lane0 / IO0 在 LSB：
//        cpif_tx_lane{i}_data     ↔ cpif_tx_lane_data[i*1024 +: 1024]
//        tx_lane{i}_data{j}[63:0] ↔ tx_analog_pad[(i*17+j)*64 +: 64]
//        rx_lane{i}_data{j}[63:0] ↔ rx_analog_pad[(i*17+j)*64 +: 64]
//        lane{i}_* 训练参数       ↔ train_param_to_analog[i*323 +: 323]
//      非对称配置（如 2TX+1RX）下，存在 lane 的未例化方向端口照常声明：输出由
//      lane_top 禁能方向 tie-off 驱动 0，输入悬空不消费
//   2. irq_lane_o 聚合：lane_top 实际输出 per-lane 8 bit {rx[3:0], tx[3:0]}；
//      本层对每 lane 两个 nibble 按 bit OR 成 4 bit，拼 {lane1[3:0], lane0[3:0]}
//      → apb_csr_top.lane_top_irq_i[7:0]。NOTE: 该 OR 本应在 lane_top 层完成
//      （lane_top_spec §5.6 定义 irq_lane_o 为 4 bit），待 spec 裁决后下沉，
//      本层 OR 为临时措施
//   3. sideband 硬件固定 lane 数 = SB_LANE_NUM=4（2026-08-28 起软硬件一致，训练握手
//      直连）。SB_TX_MASK/SB_RX_MASK 按例化 lane 集截断至 SB_LANE_NUM；LANE_NUM==
//      SB_LANE_NUM（4T1R 下 =4）时训练握手直连，余分支（LANE_NUM<SB_LANE_NUM 的
//      pad 截断 / LANE_NUM>SB_LANE_NUM 的 trunc）保留以兼容运行时参数化，编译期
//      未选中。sideband 侧仍以 phy_rx_clk0/phy_rx_clk1 标量时钟端点适配（lane0/lane1
//      时钟），RX 单 lane（lane0）消费
//   4. apb_csr_top lane 相关端口扩 4 lane（lane_top_irq_i[15:0] / aligned_rx_i[15:0] /
//      rx_fifo_status_i[31:0] / sbc_lane_enable_o[3:0]）：aligned_rx 为各 RX lane
//      per-bit AND 聚合（4T1R 下仅 lane0 RX 参与）；rx_fifo_status 由各 lane_top 的
//      rx_async_fifo_status[7:0] 按 [lane*8 +: 8] 拼装（4 lane × 8 bit = 32 bit）；
//      lane_top_irq_i 拼 {lane3..lane0}[3:0]×4；sbc_lane_enable 位宽随 sideband 扩 4
//   5. apb/mcu 复位域由 logic_phy_crg 集中同步释放：apb_pclk 与 mcu_clk 同源同步，
//      mcu_rst_n async assert + sync deassert 至 apb_pclk 域（apb_rstn_sync）与
//      mcu_clk 域（mcu_rstn_sync），apb_preset_n 即取 apb_rstn_sync（不再接 mcu_rst_n）
//   6. 复位同步集中到 logic_phy_crg（集中式复位同步，spec §5.2"各时钟域内部各自
//      同步"实现收敛）：mb_rst_n → mb_clk 域 mb_rstn_sync / 每 lane phy_rx_clk{i}
//      域 rx_rstn_sync[]；sb_rst_n → sb_tx_clk_in 域 sb_tx_rstn_sync / sb_rx_clk
//      负沿域 sb_rx_rstn_sync。lane_top 的 mb_rst_n/mb 域、rx_rst_n/rx 域复位改接
//      CRG 输出；sideband sb_rst_n 改接 sb_tx_rstn_sync；mcu/APB 域复位集中于 CRG
//      （u_sync_apb → apb_rstn_sync，u_sync_mcu → mcu_rstn_sync，async 源 mcu_rst_n）。
//      CRG 的 test_mode/test_resetn_in 当前单点 tie（logic_phy 无 DFT 顶端口，叶模块
//      原 hardcode），待 DFT 连线时接顶
//   7. train_param_to_analog packed 323 bit/lane，字段布局 {offset[322:108],
//      vref[138:123], pi[122:102], dcdl[101:0]}（intf v10.3 P0-A：380 bit 新布局）
//      （归 offset_train）；段内布局由 rx_training 显式打包（4 训练单元输出为
//      interface 具名信号，大写原文，见 rx_training.v 设计说明），本层边界按同一
//      布局拆包到 lane{i}_* 端口。lane{i}_RX{j}_OFFSET_CTRL_DATA 按 interface
//      原文声明 inout，当前仅驱动写路径（stub 常 0），analog 读回驱动 / logic 侧
//      三态控制待 MAS 补齐（intf v10.3: 10 bit）；RX16_L_CDR 反馈已删（intf v10.3 ⑤）
//   8. pll_lock 输入暂悬空（spec §5.5 其使用归 sideband，sideband 当前无对应
//      端口，待其补齐后接入）
//   9. cpif_tx_lane{i}_data_vld 输入未消费（lane_top 无 vld 端口，TX 数据通路
//      valid-only 语义下 vld 未入 RTL，待 lane_top_spec 补充）；
//      cpif_rx_lane{i}_data_vld 输出 tie 1'b1（rx_async_fifo empty/valid 未引出，
//      待 rx_chain 补路由后改接）
//  10. CPIF 控制信号方向随 link_logicphy_intf v1.4（A=link_layer 视角 output
//      = link→logic）：cpif_tx_mode / cpif_tx_rst_n / cpif_tx_lpc / cpif_rx_mode /
//      cpif_rx_rst_n 为输入，当前 RTL 未消费（复位暂以 mb_rst_n 广播，见假设 6；
//      mode/lpc 预留）；cpif_tx_ready / cpif_rx_ready 为输出，暂 tie 1'b1
//      （spec.json 无 ready 来源路由，是否由训练完成状态驱动待裁决）
//  11. io_data_width_mode_strap 为新增顶层输入（bring-up strap 锁存源，4 份
//      interface 文件均未列出，待补入 top_logic_phy_intf.md）
//  12. apb_csr_top.reorder_delay_step_o 接悬空 wire cfg_reorder_delay_step（spec.json
//      无路由，lane_top 侧为过时引用，rx_chain 内部已 tie 0）；rx_fifo_status_i 已由
//      lane_top.rx_async_fifo_status 供源（issue #13 方案 C）：level/full/empty 有效，
//      ovf/unf 本阶段恒 0（rx_async_fifo 的 overflow/underflow 语义未就绪——underflow
//      由 empty 电平触发、复位后恒置位；且二者 clear 在 rx_chain 侧 tie 0，RW1C 清完
//      即被重置），待上游补 clear 路由后再放开
//  13. spec.json 中 cfg_lane_top_o → lane_top.cfg_i[32] 连接为过时引用
//      （v4.1 已改具名信号直送），本层不实现
//  14. mcu_subsys 为 black-box stub：其 APB master 0（→ apb_csr_top）psel 常 0，
//      CSR 暂不可访问；对外 apb_*（协议层+链路层 CSR）同理常 0，待责任人实现
//  15. lane 配置宏派生（2026-08-11，TX/RX 独立化 2026-08-13）：LANE_NUM 默认值 =
//      `LP_LANE_NUM（= 物理 lane 槽位数，由 logic_phy_defines.vh 从 top PHY_LANE_*
//      宏派生，hipi_top_defines.vh 须先于本层 defines 编译）；per-lane TX/RX 例化
//      使能 = 每 lane 每方向离散宏 LP_TX_LANE{i}_EN / LP_RX_LANE{i}_EN（可独立、可非
//      连续）；2026-08-28（4T1R）起 lane_top ×4 改为直接显式例化，TX_EN/RX_EN 直接用
//      常量（lane0: 1/1，lane1~3: 1/0，与离散宏掩码 LP_TX_EN_MASK4=4'b1111 /
//      LP_RX_EN_MASK4=4'b0001 一致）；掩码仍驱动 sideband 方向掩码 / aligned AND
//      聚合（RX 未例化 lane 不参与；本层按 LP_RX_EN_BITS 掩码）。CSR 运行时有效
//      lane 使能（实际工作 lane）归 apb_csr_top / sideband，后续阶段补齐
//  16. CSR 运行时 lane 使能与编译期例化的衔接（2026-08-11）：
//      SBC_LANE_ENABLE.lane_enable 保持单 bitmap 软件契约（不区分方向），
//      sideband 内部按方向派生有效使能 = lane_enable & SB_TX_MASK/SB_RX_MASK
//      （掩码截断到 SB_LANE_NUM=4 硬件边界，未例化 lane 的软件置位无害）；
//      SBC_LANE_ENABLE 复位值 = SB_LANE_EN_RST（= 例化 lane 集，TX or RX 使能逐 lane
//      离散派生并截断到 SB 边界，复位后使能集 = 例化 lane 集）；aligned_agg 维持编译期
//      掩码不叠加运行时使能（运行时收窄 RX lane 时其 aligned sticky 保持，无聚合挂死风险）
// -----------------------------------------------------------------------------

module logic_phy #(
    parameter LANE_NUM         = `LP_LANE_NUM, // lane 槽位数，合法 1~4，默认 = 物理 lane 数（PHY_LANE_* 派生）；override 须与宏配置一致
    parameter TX_MB_PIPE_DEPTH = 0, // 透传 lane_top TX 侧：analog 边界 pipeline 深度 0~4
    parameter RX_MB_PIPE_DEPTH = 0  // 透传 lane_top RX 侧：analog 边界 pipeline 深度 0~4
) (
    // -------------------------------------------------------------------------
    // 时钟与复位
    // -------------------------------------------------------------------------
    input  wire                 mb_clk,                         // TX 主带时钟（analog PHY 提供，per-link 共享）
    input  wire                 phy_rx_clk0,                    // lane0 RX 恢复时钟（analog PHY 提供）
`ifdef LP_RX_LANE1_EN
    input  wire                 phy_rx_clk1,                    // lane1 RX 恢复时钟
`endif
`ifdef LP_RX_LANE2_EN
    input  wire                 phy_rx_clk2,                    // lane2 RX 恢复时钟
`endif
`ifdef LP_RX_LANE3_EN
    input  wire                 phy_rx_clk3,                    // lane3 RX 恢复时钟
`endif
    input  wire                 sb_tx_clk_in,                   // 边带发送时钟源（常跑，analog PHY 提供）
    output wire                 sb_tx_clk,                      // 边带发送随路时钟（sideband 门控输出 → analog PHY）
    input  wire                 sb_rx_clk,                      // 边带接收时钟（对端随路，analog PHY 提供）
    input  wire                 mcu_clk,                        // MCU 工作时钟（top 提供）
    input  wire                 apb_pclk,                       // APB 配置时钟（与 mcu_clk 同源同步，top 提供）
    input  wire                 mb_rst_n,                       // 主带全局异步复位，低有效（top 提供）
    input  wire                 sb_rst_n,                       // 边带全局异步复位，低有效（top 提供）
    input  wire                 mcu_rst_n,                      // MCU 异步复位，低有效（top 提供，CRG 内同步至 mcu_clk 域）
    input  wire                 presetn,                        // APB 异步复位，低有效（top 提供，CRG 内同步至 apb_pclk 域）

    // -------------------------------------------------------------------------
    // CPIF 接口（与 link_layer，link_logicphy_intf v1.4 原文逐 lane 展开；
    // lane{i} 端口块按 `LP_LANE{1,2,3}_EN 条件编译，见头部假设 1）
    // -------------------------------------------------------------------------
    // 发送数据（link_layer → logic_phy；data_vld 当前未消费，见头部假设 9）
    input  wire [1023:0]        cpif_tx_lane0_data,             // lane0 发送并行数据，LSB 优先
    input  wire                 cpif_tx_lane0_data_vld,         // 发送数据有效指示
`ifdef LP_TX_LANE1_EN
    input  wire [1023:0]        cpif_tx_lane1_data,             // lane1 发送并行数据，LSB 优先
    input  wire                 cpif_tx_lane1_data_vld,         // 发送数据有效指示
`endif
`ifdef LP_TX_LANE2_EN
    input  wire [1023:0]        cpif_tx_lane2_data,             // lane2 发送并行数据，LSB 优先
    input  wire                 cpif_tx_lane2_data_vld,         // 发送数据有效指示
`endif
`ifdef LP_TX_LANE3_EN
    input  wire [1023:0]        cpif_tx_lane3_data,             // lane3 发送并行数据，LSB 优先
    input  wire                 cpif_tx_lane3_data_vld,         // 发送数据有效指示
`endif

    // 接收数据（logic_phy → link_layer；data_vld 暂 tie 1，见头部假设 9）
    output wire [1023:0]        cpif_rx_lane0_data,             // lane0 接收并行数据（CDC 后），LSB 优先
    output wire                 cpif_rx_lane0_data_vld,         // 接收数据有效指示
`ifdef LP_RX_LANE1_EN
    output wire [1023:0]        cpif_rx_lane1_data,             // lane1 接收并行数据（CDC 后），LSB 优先
    output wire                 cpif_rx_lane1_data_vld,         // 接收数据有效指示
`endif
`ifdef LP_RX_LANE2_EN
    output wire [1023:0]        cpif_rx_lane2_data,             // lane2 接收并行数据（CDC 后），LSB 优先
    output wire                 cpif_rx_lane2_data_vld,         // 接收数据有效指示
`endif
`ifdef LP_RX_LANE3_EN
    output wire [1023:0]        cpif_rx_lane3_data,             // lane3 接收并行数据（CDC 后），LSB 优先
    output wire                 cpif_rx_lane3_data_vld,         // 接收数据有效指示
`endif

    // 控制（所有 lane 共享；mode/rst_n/lpc = link→logic 输入，ready = logic→link 输出）
    input  wire [3:0]           cpif_tx_mode,                   // 发送模式，1110=NRZ（当前未消费，见头部假设 10）
    input  wire                 cpif_tx_rst_n,                  // 发送端复位，低有效（当前未消费）
    input  wire                 cpif_tx_lpc,                    // 动态低功耗控制（CPIF→analog PHY 直连：fanout 4 lane TX_LPC_DATA_IN，intf v10.3）
    output wire                 cpif_tx_ready,                  // 发送器就绪（暂 tie 1，见头部假设 10）
    input  wire [3:0]           cpif_rx_mode,                   // 接收模式（当前未消费）
    input  wire                 cpif_rx_rst_n,                  // 接收端复位，低有效（当前未消费）
    output wire                 cpif_rx_ready,                  // 接收器就绪（暂 tie 1，见头部假设 10）

    // 事件接口（所有 lane 共享，↔ sideband 透传）
    input  wire                 cpif_tx_event_enable,
    input  wire                 cpif_tx_event_req,
    input  wire [15:0]          cpif_tx_event_cmd,
    output wire                 cpif_tx_event_ack,
    output wire [7:0]           cpif_tx_event_status,
    input  wire                 cpif_rx_event_enable,
    output wire                 cpif_rx_event_req,
    output wire [15:0]          cpif_rx_event_cmd,
    input  wire                 cpif_rx_event_ack,
    input  wire [7:0]           cpif_rx_event_status,

    // -------------------------------------------------------------------------
    // 主带数据 / 训练校准（与 analog PHY，logicphy_analogphy_intf v1.1 原文逐 lane
    // 展开；非对称配置下存在 lane 的未例化方向端口照常声明——输出 tie 0、输入
    // 悬空不消费，见头部假设 1）
    // -------------------------------------------------------------------------
    // --- lane0 TX 数据（logic → analog，17 IO × 64 bit，IO0 在 packed LSB）---
    output wire [63:0]          tx_lane0_data0,                 // TX0 并行数据（LSB 优先，SDR；32:1 模式高 32 bit 接 0）
    output wire [63:0]          tx_lane0_data1,
    output wire [63:0]          tx_lane0_data2,
    output wire [63:0]          tx_lane0_data3,
    output wire [63:0]          tx_lane0_data4,
    output wire [63:0]          tx_lane0_data5,
    output wire [63:0]          tx_lane0_data6,
    output wire [63:0]          tx_lane0_data7,
    output wire [63:0]          tx_lane0_data8,
    output wire [63:0]          tx_lane0_data9,
    output wire [63:0]          tx_lane0_data10,
    output wire [63:0]          tx_lane0_data11,
    output wire [63:0]          tx_lane0_data12,
    output wire [63:0]          tx_lane0_data13,
    output wire [63:0]          tx_lane0_data14,
    output wire [63:0]          tx_lane0_data15,
    output wire [63:0]          tx_lane0_data16,                // TX16 并行数据（冗余 IO）
    // --- lane0 RX 数据（analog → logic）---
    input  wire [63:0]          rx_lane0_data0,                 // RX0 并行数据（LSB 优先，SDR；32:1 模式有效 32 bit）
    input  wire [63:0]          rx_lane0_data1,
    input  wire [63:0]          rx_lane0_data2,
    input  wire [63:0]          rx_lane0_data3,
    input  wire [63:0]          rx_lane0_data4,
    input  wire [63:0]          rx_lane0_data5,
    input  wire [63:0]          rx_lane0_data6,
    input  wire [63:0]          rx_lane0_data7,
    input  wire [63:0]          rx_lane0_data8,
    input  wire [63:0]          rx_lane0_data9,
    input  wire [63:0]          rx_lane0_data10,
    input  wire [63:0]          rx_lane0_data11,
    input  wire [63:0]          rx_lane0_data12,
    input  wire [63:0]          rx_lane0_data13,
    input  wire [63:0]          rx_lane0_data14,
    input  wire [63:0]          rx_lane0_data15,
    input  wire [63:0]          rx_lane0_data16,                // RX16 并行数据（冗余 IO）
    // --- lane0 训练校准参数（logic → analog；intf v10.3 P0-A 重构：OFFSET_DATA 10b / VREF MAIN+ERR_EDGE /
    //     PI 3×7 GRAY / DCDL 17×6；CDR_OFFSET_CTRL·MAIN_DCDL_CDR·RX16_L_CDR·RX_PI_GX* 删除，intf v10.4）---
    output wire                 lane0_RX_OFFSET_CALIBRATION_ON, // offset 训练使能
    output wire                 lane0_OFFSET_CTRL_CLK,          // offset 训练时钟（intf v10.3: 匹配模型，无 RX_ 前缀）
    output wire                 lane0_OFFSET_RESET_N,           // offset 训练复位（已同步撤离到 OFFSET_CTRL_CLK 域）
    output wire                 lane0_RX0_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane0_RX0_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane0_RX0_OFFSET_CTRL_DATA,    // inout 原文：当前仅驱动写路径；intf v10.3: 8→10 bit
    output wire                 lane0_RX1_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane0_RX1_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane0_RX1_OFFSET_CTRL_DATA,
    output wire                 lane0_RX2_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane0_RX2_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane0_RX2_OFFSET_CTRL_DATA,
    output wire                 lane0_RX3_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane0_RX3_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane0_RX3_OFFSET_CTRL_DATA,
    output wire                 lane0_RX4_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane0_RX4_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane0_RX4_OFFSET_CTRL_DATA,
    output wire                 lane0_RX5_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane0_RX5_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane0_RX5_OFFSET_CTRL_DATA,
    output wire                 lane0_RX6_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane0_RX6_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane0_RX6_OFFSET_CTRL_DATA,
    output wire                 lane0_RX7_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane0_RX7_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane0_RX7_OFFSET_CTRL_DATA,
    output wire                 lane0_RX8_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane0_RX8_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane0_RX8_OFFSET_CTRL_DATA,
    output wire                 lane0_RX9_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane0_RX9_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane0_RX9_OFFSET_CTRL_DATA,
    output wire                 lane0_RX10_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane0_RX10_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane0_RX10_OFFSET_CTRL_DATA,
    output wire                 lane0_RX11_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane0_RX11_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane0_RX11_OFFSET_CTRL_DATA,
    output wire                 lane0_RX12_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane0_RX12_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane0_RX12_OFFSET_CTRL_DATA,
    output wire                 lane0_RX13_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane0_RX13_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane0_RX13_OFFSET_CTRL_DATA,
    output wire                 lane0_RX14_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane0_RX14_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane0_RX14_OFFSET_CTRL_DATA,
    output wire                 lane0_RX15_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane0_RX15_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane0_RX15_OFFSET_CTRL_DATA,
    output wire                 lane0_RX16_OFFSET_CTRL_WRITE,    // RX16 offset 读写（冗余 IO）
    output wire [2:0]           lane0_RX16_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane0_RX16_OFFSET_CTRL_DATA,    // 冗余 IO
    output wire [7:0]           lane0_RX_VREF_CTRL_MAIN,        // VREF 训练参数（MAIN，intf v10.3 改名）
    output wire [7:0]           lane0_RX_VREF_CTRL_ERR_EDGE,    // VREF 训练参数（ERR_EDGE，intf v10.3 新增）
    output wire [6:0]           lane0_PH_I_GRAY_PI,             // PI 校准 I 路 7b GRAY（intf v10.3，替代 RX_PI_GXI/GYI）
    output wire [6:0]           lane0_PH_Q_GRAY_PI,             // PI 校准 Q 路 7b GRAY
    output wire [6:0]           lane0_PH_045_GRAY_PI,           // PI 校准 45° 路 7b GRAY（intf v10.3 新增）
    output wire [5:0]           lane0_RX0_DCDL_CTRL,
    output wire [5:0]           lane0_RX1_DCDL_CTRL,
    output wire [5:0]           lane0_RX2_DCDL_CTRL,
    output wire [5:0]           lane0_RX3_DCDL_CTRL,
    output wire [5:0]           lane0_RX4_DCDL_CTRL,
    output wire [5:0]           lane0_RX5_DCDL_CTRL,
    output wire [5:0]           lane0_RX6_DCDL_CTRL,
    output wire [5:0]           lane0_RX7_DCDL_CTRL,
    output wire [5:0]           lane0_RX8_DCDL_CTRL,
    output wire [5:0]           lane0_RX9_DCDL_CTRL,
    output wire [5:0]           lane0_RX10_DCDL_CTRL,
    output wire [5:0]           lane0_RX11_DCDL_CTRL,
    output wire [5:0]           lane0_RX12_DCDL_CTRL,
    output wire [5:0]           lane0_RX13_DCDL_CTRL,
    output wire [5:0]           lane0_RX14_DCDL_CTRL,
    output wire [5:0]           lane0_RX15_DCDL_CTRL,
    output wire [5:0]           lane0_RX16_DCDL_CTRL,    // RX16 DCDL 训练参数（冗余 IO；intf v10.3: 改名 + 4→6 bit）,
`ifdef LP_TX_LANE1_EN
    // --- lane1 TX 数据（logic → analog，17 IO × 64 bit，IO0 在 packed LSB）---
    output wire [63:0]          tx_lane1_data0,                 // TX0 并行数据（LSB 优先，SDR；32:1 模式高 32 bit 接 0）
    output wire [63:0]          tx_lane1_data1,
    output wire [63:0]          tx_lane1_data2,
    output wire [63:0]          tx_lane1_data3,
    output wire [63:0]          tx_lane1_data4,
    output wire [63:0]          tx_lane1_data5,
    output wire [63:0]          tx_lane1_data6,
    output wire [63:0]          tx_lane1_data7,
    output wire [63:0]          tx_lane1_data8,
    output wire [63:0]          tx_lane1_data9,
    output wire [63:0]          tx_lane1_data10,
    output wire [63:0]          tx_lane1_data11,
    output wire [63:0]          tx_lane1_data12,
    output wire [63:0]          tx_lane1_data13,
    output wire [63:0]          tx_lane1_data14,
    output wire [63:0]          tx_lane1_data15,
    output wire [63:0]          tx_lane1_data16,                // TX16 并行数据（冗余 IO）
`endif
`ifdef LP_RX_LANE1_EN
    // --- lane1 RX 数据（analog → logic）---
    input  wire [63:0]          rx_lane1_data0,                 // RX0 并行数据（LSB 优先，SDR；32:1 模式有效 32 bit）
    input  wire [63:0]          rx_lane1_data1,
    input  wire [63:0]          rx_lane1_data2,
    input  wire [63:0]          rx_lane1_data3,
    input  wire [63:0]          rx_lane1_data4,
    input  wire [63:0]          rx_lane1_data5,
    input  wire [63:0]          rx_lane1_data6,
    input  wire [63:0]          rx_lane1_data7,
    input  wire [63:0]          rx_lane1_data8,
    input  wire [63:0]          rx_lane1_data9,
    input  wire [63:0]          rx_lane1_data10,
    input  wire [63:0]          rx_lane1_data11,
    input  wire [63:0]          rx_lane1_data12,
    input  wire [63:0]          rx_lane1_data13,
    input  wire [63:0]          rx_lane1_data14,
    input  wire [63:0]          rx_lane1_data15,
    input  wire [63:0]          rx_lane1_data16,                // RX16 并行数据（冗余 IO）
    // --- lane1 训练校准参数（logic → analog；intf v10.3 P0-A 重构：OFFSET_DATA 10b / VREF MAIN+ERR_EDGE /
    //     PI 3×7 GRAY / DCDL 17×6；CDR_OFFSET_CTRL·MAIN_DCDL_CDR·RX16_L_CDR·RX_PI_GX* 删除，intf v10.4）---
    output wire                 lane1_RX_OFFSET_CALIBRATION_ON, // offset 训练使能
    output wire                 lane1_OFFSET_CTRL_CLK,          // offset 训练时钟（intf v10.3: 匹配模型，无 RX_ 前缀）
    output wire                 lane1_OFFSET_RESET_N,           // offset 训练复位（已同步撤离到 OFFSET_CTRL_CLK 域）
    output wire                 lane1_RX0_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane1_RX0_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane1_RX0_OFFSET_CTRL_DATA,    // inout 原文：当前仅驱动写路径；intf v10.3: 8→10 bit
    output wire                 lane1_RX1_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane1_RX1_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane1_RX1_OFFSET_CTRL_DATA,
    output wire                 lane1_RX2_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane1_RX2_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane1_RX2_OFFSET_CTRL_DATA,
    output wire                 lane1_RX3_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane1_RX3_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane1_RX3_OFFSET_CTRL_DATA,
    output wire                 lane1_RX4_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane1_RX4_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane1_RX4_OFFSET_CTRL_DATA,
    output wire                 lane1_RX5_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane1_RX5_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane1_RX5_OFFSET_CTRL_DATA,
    output wire                 lane1_RX6_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane1_RX6_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane1_RX6_OFFSET_CTRL_DATA,
    output wire                 lane1_RX7_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane1_RX7_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane1_RX7_OFFSET_CTRL_DATA,
    output wire                 lane1_RX8_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane1_RX8_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane1_RX8_OFFSET_CTRL_DATA,
    output wire                 lane1_RX9_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane1_RX9_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane1_RX9_OFFSET_CTRL_DATA,
    output wire                 lane1_RX10_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane1_RX10_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane1_RX10_OFFSET_CTRL_DATA,
    output wire                 lane1_RX11_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane1_RX11_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane1_RX11_OFFSET_CTRL_DATA,
    output wire                 lane1_RX12_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane1_RX12_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane1_RX12_OFFSET_CTRL_DATA,
    output wire                 lane1_RX13_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane1_RX13_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane1_RX13_OFFSET_CTRL_DATA,
    output wire                 lane1_RX14_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane1_RX14_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane1_RX14_OFFSET_CTRL_DATA,
    output wire                 lane1_RX15_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane1_RX15_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane1_RX15_OFFSET_CTRL_DATA,
    output wire                 lane1_RX16_OFFSET_CTRL_WRITE,    // RX16 offset 读写（冗余 IO）
    output wire [2:0]           lane1_RX16_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane1_RX16_OFFSET_CTRL_DATA,    // 冗余 IO
    output wire [7:0]           lane1_RX_VREF_CTRL_MAIN,        // VREF 训练参数（MAIN，intf v10.3 改名）
    output wire [7:0]           lane1_RX_VREF_CTRL_ERR_EDGE,    // VREF 训练参数（ERR_EDGE，intf v10.3 新增）
    output wire [6:0]           lane1_PH_I_GRAY_PI,             // PI 校准 I 路 7b GRAY（intf v10.3，替代 RX_PI_GXI/GYI）
    output wire [6:0]           lane1_PH_Q_GRAY_PI,             // PI 校准 Q 路 7b GRAY
    output wire [6:0]           lane1_PH_045_GRAY_PI,           // PI 校准 45° 路 7b GRAY（intf v10.3 新增）
    output wire [5:0]           lane1_RX0_DCDL_CTRL,
    output wire [5:0]           lane1_RX1_DCDL_CTRL,
    output wire [5:0]           lane1_RX2_DCDL_CTRL,
    output wire [5:0]           lane1_RX3_DCDL_CTRL,
    output wire [5:0]           lane1_RX4_DCDL_CTRL,
    output wire [5:0]           lane1_RX5_DCDL_CTRL,
    output wire [5:0]           lane1_RX6_DCDL_CTRL,
    output wire [5:0]           lane1_RX7_DCDL_CTRL,
    output wire [5:0]           lane1_RX8_DCDL_CTRL,
    output wire [5:0]           lane1_RX9_DCDL_CTRL,
    output wire [5:0]           lane1_RX10_DCDL_CTRL,
    output wire [5:0]           lane1_RX11_DCDL_CTRL,
    output wire [5:0]           lane1_RX12_DCDL_CTRL,
    output wire [5:0]           lane1_RX13_DCDL_CTRL,
    output wire [5:0]           lane1_RX14_DCDL_CTRL,
    output wire [5:0]           lane1_RX15_DCDL_CTRL,
    output wire [5:0]           lane1_RX16_DCDL_CTRL,    // RX16 DCDL 训练参数（冗余 IO；intf v10.3: 改名 + 4→6 bit）,
`endif
`ifdef LP_TX_LANE2_EN
    // --- lane2 TX 数据（logic → analog，17 IO × 64 bit，IO0 在 packed LSB）---
    output wire [63:0]          tx_lane2_data0,                 // TX0 并行数据（LSB 优先，SDR；32:1 模式高 32 bit 接 0）
    output wire [63:0]          tx_lane2_data1,
    output wire [63:0]          tx_lane2_data2,
    output wire [63:0]          tx_lane2_data3,
    output wire [63:0]          tx_lane2_data4,
    output wire [63:0]          tx_lane2_data5,
    output wire [63:0]          tx_lane2_data6,
    output wire [63:0]          tx_lane2_data7,
    output wire [63:0]          tx_lane2_data8,
    output wire [63:0]          tx_lane2_data9,
    output wire [63:0]          tx_lane2_data10,
    output wire [63:0]          tx_lane2_data11,
    output wire [63:0]          tx_lane2_data12,
    output wire [63:0]          tx_lane2_data13,
    output wire [63:0]          tx_lane2_data14,
    output wire [63:0]          tx_lane2_data15,
    output wire [63:0]          tx_lane2_data16,                // TX16 并行数据（冗余 IO）
`endif
`ifdef LP_RX_LANE2_EN
    // --- lane2 RX 数据（analog → logic）---
    input  wire [63:0]          rx_lane2_data0,                 // RX0 并行数据（LSB 优先，SDR；32:1 模式有效 32 bit）
    input  wire [63:0]          rx_lane2_data1,
    input  wire [63:0]          rx_lane2_data2,
    input  wire [63:0]          rx_lane2_data3,
    input  wire [63:0]          rx_lane2_data4,
    input  wire [63:0]          rx_lane2_data5,
    input  wire [63:0]          rx_lane2_data6,
    input  wire [63:0]          rx_lane2_data7,
    input  wire [63:0]          rx_lane2_data8,
    input  wire [63:0]          rx_lane2_data9,
    input  wire [63:0]          rx_lane2_data10,
    input  wire [63:0]          rx_lane2_data11,
    input  wire [63:0]          rx_lane2_data12,
    input  wire [63:0]          rx_lane2_data13,
    input  wire [63:0]          rx_lane2_data14,
    input  wire [63:0]          rx_lane2_data15,
    input  wire [63:0]          rx_lane2_data16,                // RX16 并行数据（冗余 IO）
    // --- lane2 训练校准参数（logic → analog；intf v10.3 P0-A 重构：OFFSET_DATA 10b / VREF MAIN+ERR_EDGE /
    //     PI 3×7 GRAY / DCDL 17×6；CDR_OFFSET_CTRL·MAIN_DCDL_CDR·RX16_L_CDR·RX_PI_GX* 删除，intf v10.4）---
    output wire                 lane2_RX_OFFSET_CALIBRATION_ON, // offset 训练使能
    output wire                 lane2_OFFSET_CTRL_CLK,          // offset 训练时钟（intf v10.3: 匹配模型，无 RX_ 前缀）
    output wire                 lane2_OFFSET_RESET_N,           // offset 训练复位（已同步撤离到 OFFSET_CTRL_CLK 域）
    output wire                 lane2_RX0_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane2_RX0_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane2_RX0_OFFSET_CTRL_DATA,    // inout 原文：当前仅驱动写路径；intf v10.3: 8→10 bit
    output wire                 lane2_RX1_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane2_RX1_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane2_RX1_OFFSET_CTRL_DATA,
    output wire                 lane2_RX2_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane2_RX2_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane2_RX2_OFFSET_CTRL_DATA,
    output wire                 lane2_RX3_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane2_RX3_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane2_RX3_OFFSET_CTRL_DATA,
    output wire                 lane2_RX4_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane2_RX4_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane2_RX4_OFFSET_CTRL_DATA,
    output wire                 lane2_RX5_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane2_RX5_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane2_RX5_OFFSET_CTRL_DATA,
    output wire                 lane2_RX6_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane2_RX6_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane2_RX6_OFFSET_CTRL_DATA,
    output wire                 lane2_RX7_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane2_RX7_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane2_RX7_OFFSET_CTRL_DATA,
    output wire                 lane2_RX8_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane2_RX8_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane2_RX8_OFFSET_CTRL_DATA,
    output wire                 lane2_RX9_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane2_RX9_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane2_RX9_OFFSET_CTRL_DATA,
    output wire                 lane2_RX10_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane2_RX10_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane2_RX10_OFFSET_CTRL_DATA,
    output wire                 lane2_RX11_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane2_RX11_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane2_RX11_OFFSET_CTRL_DATA,
    output wire                 lane2_RX12_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane2_RX12_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane2_RX12_OFFSET_CTRL_DATA,
    output wire                 lane2_RX13_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane2_RX13_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane2_RX13_OFFSET_CTRL_DATA,
    output wire                 lane2_RX14_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane2_RX14_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane2_RX14_OFFSET_CTRL_DATA,
    output wire                 lane2_RX15_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane2_RX15_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane2_RX15_OFFSET_CTRL_DATA,
    output wire                 lane2_RX16_OFFSET_CTRL_WRITE,    // RX16 offset 读写（冗余 IO）
    output wire [2:0]           lane2_RX16_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane2_RX16_OFFSET_CTRL_DATA,    // 冗余 IO
    output wire [7:0]           lane2_RX_VREF_CTRL_MAIN,        // VREF 训练参数（MAIN，intf v10.3 改名）
    output wire [7:0]           lane2_RX_VREF_CTRL_ERR_EDGE,    // VREF 训练参数（ERR_EDGE，intf v10.3 新增）
    output wire [6:0]           lane2_PH_I_GRAY_PI,             // PI 校准 I 路 7b GRAY（intf v10.3，替代 RX_PI_GXI/GYI）
    output wire [6:0]           lane2_PH_Q_GRAY_PI,             // PI 校准 Q 路 7b GRAY
    output wire [6:0]           lane2_PH_045_GRAY_PI,           // PI 校准 45° 路 7b GRAY（intf v10.3 新增）
    output wire [5:0]           lane2_RX0_DCDL_CTRL,
    output wire [5:0]           lane2_RX1_DCDL_CTRL,
    output wire [5:0]           lane2_RX2_DCDL_CTRL,
    output wire [5:0]           lane2_RX3_DCDL_CTRL,
    output wire [5:0]           lane2_RX4_DCDL_CTRL,
    output wire [5:0]           lane2_RX5_DCDL_CTRL,
    output wire [5:0]           lane2_RX6_DCDL_CTRL,
    output wire [5:0]           lane2_RX7_DCDL_CTRL,
    output wire [5:0]           lane2_RX8_DCDL_CTRL,
    output wire [5:0]           lane2_RX9_DCDL_CTRL,
    output wire [5:0]           lane2_RX10_DCDL_CTRL,
    output wire [5:0]           lane2_RX11_DCDL_CTRL,
    output wire [5:0]           lane2_RX12_DCDL_CTRL,
    output wire [5:0]           lane2_RX13_DCDL_CTRL,
    output wire [5:0]           lane2_RX14_DCDL_CTRL,
    output wire [5:0]           lane2_RX15_DCDL_CTRL,
    output wire [5:0]           lane2_RX16_DCDL_CTRL,    // RX16 DCDL 训练参数（冗余 IO；intf v10.3: 改名 + 4→6 bit）,
`endif
`ifdef LP_TX_LANE3_EN
    // --- lane3 TX 数据（logic → analog，17 IO × 64 bit，IO0 在 packed LSB）---
    output wire [63:0]          tx_lane3_data0,                 // TX0 并行数据（LSB 优先，SDR；32:1 模式高 32 bit 接 0）
    output wire [63:0]          tx_lane3_data1,
    output wire [63:0]          tx_lane3_data2,
    output wire [63:0]          tx_lane3_data3,
    output wire [63:0]          tx_lane3_data4,
    output wire [63:0]          tx_lane3_data5,
    output wire [63:0]          tx_lane3_data6,
    output wire [63:0]          tx_lane3_data7,
    output wire [63:0]          tx_lane3_data8,
    output wire [63:0]          tx_lane3_data9,
    output wire [63:0]          tx_lane3_data10,
    output wire [63:0]          tx_lane3_data11,
    output wire [63:0]          tx_lane3_data12,
    output wire [63:0]          tx_lane3_data13,
    output wire [63:0]          tx_lane3_data14,
    output wire [63:0]          tx_lane3_data15,
    output wire [63:0]          tx_lane3_data16,                // TX16 并行数据（冗余 IO）
`endif
`ifdef LP_RX_LANE3_EN
    // --- lane3 RX 数据（analog → logic）---
    input  wire [63:0]          rx_lane3_data0,                 // RX0 并行数据（LSB 优先，SDR；32:1 模式有效 32 bit）
    input  wire [63:0]          rx_lane3_data1,
    input  wire [63:0]          rx_lane3_data2,
    input  wire [63:0]          rx_lane3_data3,
    input  wire [63:0]          rx_lane3_data4,
    input  wire [63:0]          rx_lane3_data5,
    input  wire [63:0]          rx_lane3_data6,
    input  wire [63:0]          rx_lane3_data7,
    input  wire [63:0]          rx_lane3_data8,
    input  wire [63:0]          rx_lane3_data9,
    input  wire [63:0]          rx_lane3_data10,
    input  wire [63:0]          rx_lane3_data11,
    input  wire [63:0]          rx_lane3_data12,
    input  wire [63:0]          rx_lane3_data13,
    input  wire [63:0]          rx_lane3_data14,
    input  wire [63:0]          rx_lane3_data15,
    input  wire [63:0]          rx_lane3_data16,                // RX16 并行数据（冗余 IO）
    // --- lane3 训练校准参数（logic → analog；intf v10.3 P0-A 重构：OFFSET_DATA 10b / VREF MAIN+ERR_EDGE /
    //     PI 3×7 GRAY / DCDL 17×6；CDR_OFFSET_CTRL·MAIN_DCDL_CDR·RX16_L_CDR·RX_PI_GX* 删除，intf v10.4）---
    output wire                 lane3_RX_OFFSET_CALIBRATION_ON, // offset 训练使能
    output wire                 lane3_OFFSET_CTRL_CLK,          // offset 训练时钟（intf v10.3: 匹配模型，无 RX_ 前缀）
    output wire                 lane3_OFFSET_RESET_N,           // offset 训练复位（已同步撤离到 OFFSET_CTRL_CLK 域）
    output wire                 lane3_RX0_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane3_RX0_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane3_RX0_OFFSET_CTRL_DATA,    // inout 原文：当前仅驱动写路径；intf v10.3: 8→10 bit
    output wire                 lane3_RX1_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane3_RX1_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane3_RX1_OFFSET_CTRL_DATA,
    output wire                 lane3_RX2_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane3_RX2_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane3_RX2_OFFSET_CTRL_DATA,
    output wire                 lane3_RX3_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane3_RX3_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane3_RX3_OFFSET_CTRL_DATA,
    output wire                 lane3_RX4_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane3_RX4_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane3_RX4_OFFSET_CTRL_DATA,
    output wire                 lane3_RX5_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane3_RX5_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane3_RX5_OFFSET_CTRL_DATA,
    output wire                 lane3_RX6_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane3_RX6_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane3_RX6_OFFSET_CTRL_DATA,
    output wire                 lane3_RX7_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane3_RX7_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane3_RX7_OFFSET_CTRL_DATA,
    output wire                 lane3_RX8_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane3_RX8_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane3_RX8_OFFSET_CTRL_DATA,
    output wire                 lane3_RX9_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane3_RX9_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane3_RX9_OFFSET_CTRL_DATA,
    output wire                 lane3_RX10_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane3_RX10_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane3_RX10_OFFSET_CTRL_DATA,
    output wire                 lane3_RX11_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane3_RX11_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane3_RX11_OFFSET_CTRL_DATA,
    output wire                 lane3_RX12_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane3_RX12_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane3_RX12_OFFSET_CTRL_DATA,
    output wire                 lane3_RX13_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane3_RX13_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane3_RX13_OFFSET_CTRL_DATA,
    output wire                 lane3_RX14_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane3_RX14_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane3_RX14_OFFSET_CTRL_DATA,
    output wire                 lane3_RX15_OFFSET_CTRL_WRITE,
    output wire [2:0]           lane3_RX15_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane3_RX15_OFFSET_CTRL_DATA,
    output wire                 lane3_RX16_OFFSET_CTRL_WRITE,    // RX16 offset 读写（冗余 IO）
    output wire [2:0]           lane3_RX16_OFFSET_CTRL_ADDR,
    inout  wire [9:0]           lane3_RX16_OFFSET_CTRL_DATA,    // 冗余 IO
    output wire [7:0]           lane3_RX_VREF_CTRL_MAIN,        // VREF 训练参数（MAIN，intf v10.3 改名）
    output wire [7:0]           lane3_RX_VREF_CTRL_ERR_EDGE,    // VREF 训练参数（ERR_EDGE，intf v10.3 新增）
    output wire [6:0]           lane3_PH_I_GRAY_PI,             // PI 校准 I 路 7b GRAY（intf v10.3，替代 RX_PI_GXI/GYI）
    output wire [6:0]           lane3_PH_Q_GRAY_PI,             // PI 校准 Q 路 7b GRAY
    output wire [6:0]           lane3_PH_045_GRAY_PI,           // PI 校准 45° 路 7b GRAY（intf v10.3 新增）
    output wire [5:0]           lane3_RX0_DCDL_CTRL,
    output wire [5:0]           lane3_RX1_DCDL_CTRL,
    output wire [5:0]           lane3_RX2_DCDL_CTRL,
    output wire [5:0]           lane3_RX3_DCDL_CTRL,
    output wire [5:0]           lane3_RX4_DCDL_CTRL,
    output wire [5:0]           lane3_RX5_DCDL_CTRL,
    output wire [5:0]           lane3_RX6_DCDL_CTRL,
    output wire [5:0]           lane3_RX7_DCDL_CTRL,
    output wire [5:0]           lane3_RX8_DCDL_CTRL,
    output wire [5:0]           lane3_RX9_DCDL_CTRL,
    output wire [5:0]           lane3_RX10_DCDL_CTRL,
    output wire [5:0]           lane3_RX11_DCDL_CTRL,
    output wire [5:0]           lane3_RX12_DCDL_CTRL,
    output wire [5:0]           lane3_RX13_DCDL_CTRL,
    output wire [5:0]           lane3_RX14_DCDL_CTRL,
    output wire [5:0]           lane3_RX15_DCDL_CTRL,
    output wire [5:0]           lane3_RX16_DCDL_CTRL,    // RX16 DCDL 训练参数（冗余 IO；intf v10.3: 改名 + 4→6 bit）,
`endif

    // SBC 边带通道（经 analog PHY 中转至对端）
    output wire                 sb_txd,                         // 边带发送串行数据（同步 sb_tx_clk）
    input  wire                 sb_rxd,                         // 边带接收串行数据（同步 sb_rx_clk）
    output wire                 sb_txr,                         // 边带发送数据冗余（intf v10.3 P1，芯片级扩展；sbc_txr_en=1 时承载 SBC 流、sb_txd 数据隔离）
    input  wire                 sb_rxr,                         // 边带接收数据冗余（sbc_rxr_sel=1 时进解调主路、sb_rxd 忽略）

    // 状态 / 反馈
    input  wire                 pll_lock,                       // 模拟 PHY PLL 锁定（暂悬空，见头部假设 8）

    // -------------------------------------------------------------------------
`ifdef MCU_SUBSYS
    // AHB slave 接口（top 提供，见 top_logic_phy_intf v1.2 → mcu_subsys）
    // -------------------------------------------------------------------------
    input  wire                 ahbs_hsel,
    input  wire [31:0]          ahbs_haddr,
    input  wire [2:0]           ahbs_hburst,
    input  wire [2:0]           ahbs_hsize,
    input  wire [3:0]           ahbs_hprot,
    input  wire [31:0]          ahbs_hwdata,
    input  wire                 ahbs_hwrite,
    input  wire [1:0]           ahbs_htrans,
    input  wire                 ahbs_hready,
    output wire [31:0]          ahbs_hrdata,
    output wire [1:0]           ahbs_hresp,
    output wire                 ahbs_hready_resp,
`else
    // APB slave 接口（top 提供；MCU_SUBSYS 关闭：无 AHB/MCU，外部 APB 直接取代
    // mcu_subsys 输出的两路 APB，本模块内 MUX 按 paddr[16] 分域）
    //   paddr[16]=0: phy CSR（apb_csr_top，0x000~0xFFF）
    //   paddr[16]=1: 协议层+链路层 CSR（apb_pl_* 出口，paddr[15:0] 直通）
    // -------------------------------------------------------------------------
    input  wire                 hipi_apb_psel,
    input  wire                 hipi_apb_penable,
    input  wire                 hipi_apb_pwrite,
    input  wire [19:0]          hipi_apb_paddr,          // [16] = 域选择（0=phy CSR / 1=ctrl CSR）
    input  wire [31:0]          hipi_apb_pwdata,
    output wire [31:0]          hipi_apb_prdata,
    output wire                 hipi_apb_pready,
    output wire                 hipi_apb_pslverr,
`endif

    // -------------------------------------------------------------------------
    // APB 配置接口（logic_phy → 协议层 + 链路层 CSR 合并空间，
    // 见 link_logicphy_intf v1.4 §APB 配置接口；mcu_subsys APB master 1 直出）
    // -------------------------------------------------------------------------
    output wire                 apb_psel,
    output wire                 apb_penable,
    output wire                 apb_pwrite,
    output wire [19:0]          apb_paddr,
    output wire [31:0]          apb_pwdata,
    input  wire [31:0]          apb_prdata,
    input  wire                 apb_pready,
    input  wire                 apb_pslverr,

    // -------------------------------------------------------------------------
    // bring-up strap（新增端口，见头部假设 11）
    // -------------------------------------------------------------------------
    input  wire                 io_data_width_mode_strap,       // 0=32:1 模式 / 1=64:1 模式

    // -------------------------------------------------------------------------
    // 中断（上报外部 host，见 top_logic_phy_intf v1.2）
    // -------------------------------------------------------------------------
    output wire [31:0]          lane0_TRX_REG1_WRITE_IN,
    output wire [31:0]          lane0_TRX_REG2_WRITE_IN,
    output wire [31:0]          lane0_TRX_TEST_REG1_WRITE_IN,
    output wire [31:0]          lane0_TRX_TEST_REG2_WRITE_IN,
    output wire [31:0]          lane0_RX_CLK_ENG_REG1_WRITE_IN,
    output wire [31:0]          lane0_RX_CLK_ENG_REG2_WRITE_IN,
    output wire [31:0]          lane0_RX_CLK_ENG_REG3_WRITE_IN,
    output wire [31:0]          lane0_RX_CLK_ENG_REG4_WRITE_IN,
    output wire [31:0]          lane0_RX_CLK_REG1_WRITE_IN,
    output wire [31:0]          lane0_RX_CLK_REG2_WRITE_IN,
    output wire [31:0]          lane0_RX_CLK_REG3_WRITE_IN,
    output wire [31:0]          lane0_RX_CLK_REG4_WRITE_IN,
    output wire [31:0]          lane0_RX_DATA_REG1_WRITE_IN,
    output wire [31:0]          lane0_RX_DATA_REG2_WRITE_IN,
    output wire [31:0]          lane0_RX_DATA_REG3_WRITE_IN,
    output wire [31:0]          lane0_RX_LDO_REG1_WRITE_IN,
    output wire [31:0]          lane0_RX_LDO_REG2_WRITE_IN,
    output wire [31:0]          lane0_TX_CLK_ENG_REG1_WRITE_IN,
    output wire [31:0]          lane0_TX_CLK_ENG_REG2_WRITE_IN,
    output wire [31:0]          lane0_TX_CLK_ENG_REG3_WRITE_IN,
    output wire [31:0]          lane0_TX_CLK_ENG_REG4_WRITE_IN,
    output wire [31:0]          lane0_TX_CLK_REG1_WRITE_IN,
    output wire [31:0]          lane0_TX_CLK_REG2_WRITE_IN,
    output wire [31:0]          lane0_TX_CLK_REG3_WRITE_IN,
    output wire [31:0]          lane0_TX_CLK_REG4_WRITE_IN,
    output wire [31:0]          lane0_TX_CLK_REG5_WRITE_IN,
    output wire [31:0]          lane0_TX_DATA_REG1_WRITE_IN,
    output wire [31:0]          lane0_TX_DATA_REG2_WRITE_IN,
    output wire [31:0]          lane0_TX_DATA_REG3_WRITE_IN,
    output wire [31:0]          lane0_TX_DATA_REG4_WRITE_IN,
    output wire [31:0]          lane0_TX_DATA_REG5_WRITE_IN,
    output wire [31:0]          lane0_TX_DATA_REG6_WRITE_IN,
    output wire [31:0]          lane0_TX_DATA_REG7_WRITE_IN,
    output wire [31:0]          lane0_TX_LDO_REG1_WRITE_IN,
    output wire [31:0]          lane0_TX_LDO_REG2_WRITE_IN,
    output wire [31:0]          lane1_TRX_REG1_WRITE_IN,
    output wire [31:0]          lane1_TRX_REG2_WRITE_IN,
    output wire [31:0]          lane1_TRX_TEST_REG1_WRITE_IN,
    output wire [31:0]          lane1_TRX_TEST_REG2_WRITE_IN,
`ifdef LP_RX_LANE1_EN
    output wire [31:0]          lane1_RX_CLK_ENG_REG1_WRITE_IN,
    output wire [31:0]          lane1_RX_CLK_ENG_REG2_WRITE_IN,
    output wire [31:0]          lane1_RX_CLK_ENG_REG3_WRITE_IN,
    output wire [31:0]          lane1_RX_CLK_ENG_REG4_WRITE_IN,
    output wire [31:0]          lane1_RX_CLK_REG1_WRITE_IN,
    output wire [31:0]          lane1_RX_CLK_REG2_WRITE_IN,
    output wire [31:0]          lane1_RX_CLK_REG3_WRITE_IN,
    output wire [31:0]          lane1_RX_CLK_REG4_WRITE_IN,
    output wire [31:0]          lane1_RX_DATA_REG1_WRITE_IN,
    output wire [31:0]          lane1_RX_DATA_REG2_WRITE_IN,
    output wire [31:0]          lane1_RX_DATA_REG3_WRITE_IN,
    output wire [31:0]          lane1_RX_LDO_REG1_WRITE_IN,
    output wire [31:0]          lane1_RX_LDO_REG2_WRITE_IN,
`endif
`ifdef LP_TX_LANE1_EN
    output wire [31:0]          lane1_TX_CLK_ENG_REG1_WRITE_IN,
    output wire [31:0]          lane1_TX_CLK_ENG_REG2_WRITE_IN,
    output wire [31:0]          lane1_TX_CLK_ENG_REG3_WRITE_IN,
    output wire [31:0]          lane1_TX_CLK_ENG_REG4_WRITE_IN,
    output wire [31:0]          lane1_TX_CLK_REG1_WRITE_IN,
    output wire [31:0]          lane1_TX_CLK_REG2_WRITE_IN,
    output wire [31:0]          lane1_TX_CLK_REG3_WRITE_IN,
    output wire [31:0]          lane1_TX_CLK_REG4_WRITE_IN,
    output wire [31:0]          lane1_TX_CLK_REG5_WRITE_IN,
    output wire [31:0]          lane1_TX_DATA_REG1_WRITE_IN,
    output wire [31:0]          lane1_TX_DATA_REG2_WRITE_IN,
    output wire [31:0]          lane1_TX_DATA_REG3_WRITE_IN,
    output wire [31:0]          lane1_TX_DATA_REG4_WRITE_IN,
    output wire [31:0]          lane1_TX_DATA_REG5_WRITE_IN,
    output wire [31:0]          lane1_TX_DATA_REG6_WRITE_IN,
    output wire [31:0]          lane1_TX_DATA_REG7_WRITE_IN,
    output wire [31:0]          lane1_TX_LDO_REG1_WRITE_IN,
    output wire [31:0]          lane1_TX_LDO_REG2_WRITE_IN,
`endif
    output wire [31:0]          lane2_TRX_REG1_WRITE_IN,
    output wire [31:0]          lane2_TRX_REG2_WRITE_IN,
    output wire [31:0]          lane2_TRX_TEST_REG1_WRITE_IN,
    output wire [31:0]          lane2_TRX_TEST_REG2_WRITE_IN,
`ifdef LP_RX_LANE2_EN
    output wire [31:0]          lane2_RX_CLK_ENG_REG1_WRITE_IN,
    output wire [31:0]          lane2_RX_CLK_ENG_REG2_WRITE_IN,
    output wire [31:0]          lane2_RX_CLK_ENG_REG3_WRITE_IN,
    output wire [31:0]          lane2_RX_CLK_ENG_REG4_WRITE_IN,
    output wire [31:0]          lane2_RX_CLK_REG1_WRITE_IN,
    output wire [31:0]          lane2_RX_CLK_REG2_WRITE_IN,
    output wire [31:0]          lane2_RX_CLK_REG3_WRITE_IN,
    output wire [31:0]          lane2_RX_CLK_REG4_WRITE_IN,
    output wire [31:0]          lane2_RX_DATA_REG1_WRITE_IN,
    output wire [31:0]          lane2_RX_DATA_REG2_WRITE_IN,
    output wire [31:0]          lane2_RX_DATA_REG3_WRITE_IN,
    output wire [31:0]          lane2_RX_LDO_REG1_WRITE_IN,
    output wire [31:0]          lane2_RX_LDO_REG2_WRITE_IN,
`endif
`ifdef LP_TX_LANE2_EN
    output wire [31:0]          lane2_TX_CLK_ENG_REG1_WRITE_IN,
    output wire [31:0]          lane2_TX_CLK_ENG_REG2_WRITE_IN,
    output wire [31:0]          lane2_TX_CLK_ENG_REG3_WRITE_IN,
    output wire [31:0]          lane2_TX_CLK_ENG_REG4_WRITE_IN,
    output wire [31:0]          lane2_TX_CLK_REG1_WRITE_IN,
    output wire [31:0]          lane2_TX_CLK_REG2_WRITE_IN,
    output wire [31:0]          lane2_TX_CLK_REG3_WRITE_IN,
    output wire [31:0]          lane2_TX_CLK_REG4_WRITE_IN,
    output wire [31:0]          lane2_TX_CLK_REG5_WRITE_IN,
    output wire [31:0]          lane2_TX_DATA_REG1_WRITE_IN,
    output wire [31:0]          lane2_TX_DATA_REG2_WRITE_IN,
    output wire [31:0]          lane2_TX_DATA_REG3_WRITE_IN,
    output wire [31:0]          lane2_TX_DATA_REG4_WRITE_IN,
    output wire [31:0]          lane2_TX_DATA_REG5_WRITE_IN,
    output wire [31:0]          lane2_TX_DATA_REG6_WRITE_IN,
    output wire [31:0]          lane2_TX_DATA_REG7_WRITE_IN,
    output wire [31:0]          lane2_TX_LDO_REG1_WRITE_IN,
    output wire [31:0]          lane2_TX_LDO_REG2_WRITE_IN,
`endif
    output wire [31:0]          lane3_TRX_REG1_WRITE_IN,
    output wire [31:0]          lane3_TRX_REG2_WRITE_IN,
    output wire [31:0]          lane3_TRX_TEST_REG1_WRITE_IN,
    output wire [31:0]          lane3_TRX_TEST_REG2_WRITE_IN,
`ifdef LP_RX_LANE3_EN
    output wire [31:0]          lane3_RX_CLK_ENG_REG1_WRITE_IN,
    output wire [31:0]          lane3_RX_CLK_ENG_REG2_WRITE_IN,
    output wire [31:0]          lane3_RX_CLK_ENG_REG3_WRITE_IN,
    output wire [31:0]          lane3_RX_CLK_ENG_REG4_WRITE_IN,
    output wire [31:0]          lane3_RX_CLK_REG1_WRITE_IN,
    output wire [31:0]          lane3_RX_CLK_REG2_WRITE_IN,
    output wire [31:0]          lane3_RX_CLK_REG3_WRITE_IN,
    output wire [31:0]          lane3_RX_CLK_REG4_WRITE_IN,
    output wire [31:0]          lane3_RX_DATA_REG1_WRITE_IN,
    output wire [31:0]          lane3_RX_DATA_REG2_WRITE_IN,
    output wire [31:0]          lane3_RX_DATA_REG3_WRITE_IN,
    output wire [31:0]          lane3_RX_LDO_REG1_WRITE_IN,
    output wire [31:0]          lane3_RX_LDO_REG2_WRITE_IN,
`endif
`ifdef LP_TX_LANE3_EN
    output wire [31:0]          lane3_TX_CLK_ENG_REG1_WRITE_IN,
    output wire [31:0]          lane3_TX_CLK_ENG_REG2_WRITE_IN,
    output wire [31:0]          lane3_TX_CLK_ENG_REG3_WRITE_IN,
    output wire [31:0]          lane3_TX_CLK_ENG_REG4_WRITE_IN,
    output wire [31:0]          lane3_TX_CLK_REG1_WRITE_IN,
    output wire [31:0]          lane3_TX_CLK_REG2_WRITE_IN,
    output wire [31:0]          lane3_TX_CLK_REG3_WRITE_IN,
    output wire [31:0]          lane3_TX_CLK_REG4_WRITE_IN,
    output wire [31:0]          lane3_TX_CLK_REG5_WRITE_IN,
    output wire [31:0]          lane3_TX_DATA_REG1_WRITE_IN,
    output wire [31:0]          lane3_TX_DATA_REG2_WRITE_IN,
    output wire [31:0]          lane3_TX_DATA_REG3_WRITE_IN,
    output wire [31:0]          lane3_TX_DATA_REG4_WRITE_IN,
    output wire [31:0]          lane3_TX_DATA_REG5_WRITE_IN,
    output wire [31:0]          lane3_TX_DATA_REG6_WRITE_IN,
    output wire [31:0]          lane3_TX_DATA_REG7_WRITE_IN,
    output wire [31:0]          lane3_TX_LDO_REG1_WRITE_IN,
    output wire [31:0]          lane3_TX_LDO_REG2_WRITE_IN,
`endif
    output wire                 lane0_TX_EN_ASYNC,
    output wire                 lane0_TX_EN_SYNC,
    output wire                 lane0_TX_EN_TRACK,
    output wire                 lane0_TX_RSTB_SYNC,
    output wire                 lane0_RX_CLK_EN_ASYNC,
    output wire                 lane0_RX_CLK_EN_SYNC,
    output wire                 lane0_RX_CLK_EN_TRACK,
    output wire                 lane0_RX_RSTB_SYNC,
    output wire                 lane1_TX_EN_ASYNC,
    output wire                 lane1_TX_EN_SYNC,
    output wire                 lane1_TX_EN_TRACK,
    output wire                 lane1_TX_RSTB_SYNC,
    output wire                 lane1_RX_CLK_EN_ASYNC,
    output wire                 lane1_RX_CLK_EN_SYNC,
    output wire                 lane1_RX_CLK_EN_TRACK,
    output wire                 lane1_RX_RSTB_SYNC,
    output wire                 lane2_TX_EN_ASYNC,
    output wire                 lane2_TX_EN_SYNC,
    output wire                 lane2_TX_EN_TRACK,
    output wire                 lane2_TX_RSTB_SYNC,
    output wire                 lane2_RX_CLK_EN_ASYNC,
    output wire                 lane2_RX_CLK_EN_SYNC,
    output wire                 lane2_RX_CLK_EN_TRACK,
    output wire                 lane2_RX_RSTB_SYNC,
    output wire                 lane3_TX_EN_ASYNC,
    output wire                 lane3_TX_EN_SYNC,
    output wire                 lane3_TX_EN_TRACK,
    output wire                 lane3_TX_RSTB_SYNC,
    output wire                 lane3_RX_CLK_EN_ASYNC,
    output wire                 lane3_RX_CLK_EN_SYNC,
    output wire                 lane3_RX_CLK_EN_TRACK,
    output wire                 lane3_RX_RSTB_SYNC,
    input  wire [31:0]          lane0_RX_CLK_ENG_REG_READ_OUT,
    input  wire [31:0]          lane0_TX_CLK_ENG_REG_READ_OUT,
    input  wire                 lane0_CONNECT_DIG_OUT,
    input  wire [31:0]          lane1_RX_CLK_ENG_REG_READ_OUT,
    input  wire [31:0]          lane1_TX_CLK_ENG_REG_READ_OUT,
    input  wire                 lane1_CONNECT_DIG_OUT,
    input  wire [31:0]          lane2_RX_CLK_ENG_REG_READ_OUT,
    input  wire [31:0]          lane2_TX_CLK_ENG_REG_READ_OUT,
    input  wire                 lane2_CONNECT_DIG_OUT,
    input  wire [31:0]          lane3_RX_CLK_ENG_REG_READ_OUT,
    input  wire [31:0]          lane3_TX_CLK_ENG_REG_READ_OUT,
    input  wire                 lane3_CONNECT_DIG_OUT,
    input  wire [3:0]           lane0_DOUT0_ERR_EDGE,
    input  wire [3:0]           lane0_DOUT1_ERR_EDGE,
    input  wire [3:0]           lane0_DOUT2_ERR_EDGE,
    input  wire [3:0]           lane0_DOUT3_ERR_EDGE,
    input  wire [3:0]           lane0_DOUT4_ERR_EDGE,
    input  wire [3:0]           lane0_DOUT5_ERR_EDGE,
    input  wire [3:0]           lane0_DOUT6_ERR_EDGE,
    input  wire [3:0]           lane0_DOUT7_ERR_EDGE,
    input  wire [3:0]           lane0_DOUT8_ERR_EDGE,
    input  wire [3:0]           lane0_DOUT9_ERR_EDGE,
    input  wire [3:0]           lane0_DOUT10_ERR_EDGE,
    input  wire [3:0]           lane0_DOUT11_ERR_EDGE,
    input  wire [3:0]           lane0_DOUT12_ERR_EDGE,
    input  wire [3:0]           lane0_DOUT13_ERR_EDGE,
    input  wire [3:0]           lane0_DOUT14_ERR_EDGE,
    input  wire [3:0]           lane0_DOUT15_ERR_EDGE,
    input  wire [3:0]           lane0_DOUTRFU_ERR_EDGE,
    input  wire [3:0]           lane1_DOUT0_ERR_EDGE,
    input  wire [3:0]           lane1_DOUT1_ERR_EDGE,
    input  wire [3:0]           lane1_DOUT2_ERR_EDGE,
    input  wire [3:0]           lane1_DOUT3_ERR_EDGE,
    input  wire [3:0]           lane1_DOUT4_ERR_EDGE,
    input  wire [3:0]           lane1_DOUT5_ERR_EDGE,
    input  wire [3:0]           lane1_DOUT6_ERR_EDGE,
    input  wire [3:0]           lane1_DOUT7_ERR_EDGE,
    input  wire [3:0]           lane1_DOUT8_ERR_EDGE,
    input  wire [3:0]           lane1_DOUT9_ERR_EDGE,
    input  wire [3:0]           lane1_DOUT10_ERR_EDGE,
    input  wire [3:0]           lane1_DOUT11_ERR_EDGE,
    input  wire [3:0]           lane1_DOUT12_ERR_EDGE,
    input  wire [3:0]           lane1_DOUT13_ERR_EDGE,
    input  wire [3:0]           lane1_DOUT14_ERR_EDGE,
    input  wire [3:0]           lane1_DOUT15_ERR_EDGE,
    input  wire [3:0]           lane1_DOUTRFU_ERR_EDGE,
    input  wire [3:0]           lane2_DOUT0_ERR_EDGE,
    input  wire [3:0]           lane2_DOUT1_ERR_EDGE,
    input  wire [3:0]           lane2_DOUT2_ERR_EDGE,
    input  wire [3:0]           lane2_DOUT3_ERR_EDGE,
    input  wire [3:0]           lane2_DOUT4_ERR_EDGE,
    input  wire [3:0]           lane2_DOUT5_ERR_EDGE,
    input  wire [3:0]           lane2_DOUT6_ERR_EDGE,
    input  wire [3:0]           lane2_DOUT7_ERR_EDGE,
    input  wire [3:0]           lane2_DOUT8_ERR_EDGE,
    input  wire [3:0]           lane2_DOUT9_ERR_EDGE,
    input  wire [3:0]           lane2_DOUT10_ERR_EDGE,
    input  wire [3:0]           lane2_DOUT11_ERR_EDGE,
    input  wire [3:0]           lane2_DOUT12_ERR_EDGE,
    input  wire [3:0]           lane2_DOUT13_ERR_EDGE,
    input  wire [3:0]           lane2_DOUT14_ERR_EDGE,
    input  wire [3:0]           lane2_DOUT15_ERR_EDGE,
    input  wire [3:0]           lane2_DOUTRFU_ERR_EDGE,
    input  wire [3:0]           lane3_DOUT0_ERR_EDGE,
    input  wire [3:0]           lane3_DOUT1_ERR_EDGE,
    input  wire [3:0]           lane3_DOUT2_ERR_EDGE,
    input  wire [3:0]           lane3_DOUT3_ERR_EDGE,
    input  wire [3:0]           lane3_DOUT4_ERR_EDGE,
    input  wire [3:0]           lane3_DOUT5_ERR_EDGE,
    input  wire [3:0]           lane3_DOUT6_ERR_EDGE,
    input  wire [3:0]           lane3_DOUT7_ERR_EDGE,
    input  wire [3:0]           lane3_DOUT8_ERR_EDGE,
    input  wire [3:0]           lane3_DOUT9_ERR_EDGE,
    input  wire [3:0]           lane3_DOUT10_ERR_EDGE,
    input  wire [3:0]           lane3_DOUT11_ERR_EDGE,
    input  wire [3:0]           lane3_DOUT12_ERR_EDGE,
    input  wire [3:0]           lane3_DOUT13_ERR_EDGE,
    input  wire [3:0]           lane3_DOUT14_ERR_EDGE,
    input  wire [3:0]           lane3_DOUT15_ERR_EDGE,
    input  wire [3:0]           lane3_DOUTRFU_ERR_EDGE,
    output wire                 lane0_TX_LPC_DATA_IN,
    input  wire                 lane0_RX_LPC_DATA_OUT,
    output wire                 lane1_TX_LPC_DATA_IN,
`ifdef LP_RX_LANE1_EN
    input  wire                 lane1_RX_LPC_DATA_OUT,  // RX 使能时存在（WP2 门控）
`endif
    output wire                 lane2_TX_LPC_DATA_IN,
`ifdef LP_RX_LANE2_EN
    input  wire                 lane2_RX_LPC_DATA_OUT,  // RX 使能时存在（WP2 门控）
`endif
    output wire                 lane3_TX_LPC_DATA_IN,
`ifdef LP_RX_LANE3_EN
    input  wire                 lane3_RX_LPC_DATA_OUT,  // RX 使能时存在（WP2 门控）
`endif
    // -------------------------------------------------------------------------
    // LPC RX 归约（0=低功耗，任一 lane 进低功耗即上报；intf v10.3 P0-B）
    // -------------------------------------------------------------------------
    output wire                 cpif_rx_lpc,                // 4 lane RX_LPC_DATA_OUT AND 归约（CPIF 侧新增，link 层暂预留消费）

    output wire                     irq_ext                 // 聚合中断，高有效
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    localparam CPIF_DATA_WIDTH  = 1024;  // 单 lane CPIF 硬件位宽（16 IO × 64 bit）
    localparam PHY_PAD_WIDTH    = 1088;  // 单 lane analog pad 位宽（17 IO × 64 bit）
    localparam TRAIN_PARAM_W    = 380;   // 单 lane 训练参数位宽（WP1 P0-A: 3+238+16+21+102）
    localparam IO_NUM           = 16;    // per-lane 逻辑 IO 数
    localparam SB_LANE_NUM      = 4;     // sideband 硬件 lane 数（2026-08-28 起 =LANE_NUM 直连，见头部假设 3）

    // 编译期 lane 每方向使能配置（logic_phy_defines.vh 由 top PHY_LANE_* 派生每方向
    // 离散宏，设计说明 15；支持每 lane TX/RX 独立、可非连续）：
    // LP_TX_EN_MASK4 / LP_RX_EN_MASK4 为 4-bit 离散掩码（bit i = lane i 该方向使能），
    // 经 ifdef-OR 逐位拼接，任意 lane 可单独开关；再截断到 LANE_NUM 槽位。
    localparam [3:0] LP_TX_EN_MASK4 = 4'b0001   // lane0 恒 TX
        `ifdef LP_TX_LANE1_EN
            | 4'b0010
        `endif
        `ifdef LP_TX_LANE2_EN
            | 4'b0100
        `endif
        `ifdef LP_TX_LANE3_EN
            | 4'b1000
        `endif;
    localparam [3:0] LP_RX_EN_MASK4 = 4'b0001   // lane0 恒 RX
        `ifdef LP_RX_LANE1_EN
            | 4'b0010
        `endif
        `ifdef LP_RX_LANE2_EN
            | 4'b0100
        `endif
        `ifdef LP_RX_LANE3_EN
            | 4'b1000
        `endif;
    localparam [LANE_NUM-1:0] LP_TX_EN_BITS   = LP_TX_EN_MASK4[LANE_NUM-1:0];
    localparam [LANE_NUM-1:0] LP_RX_EN_BITS   = LP_RX_EN_MASK4[LANE_NUM-1:0];
    localparam [LANE_NUM-1:0] LP_LANE_EN_BITS = LP_TX_EN_BITS | LP_RX_EN_BITS;  // lane 槽位使能集 = TX or RX
    // lane 槽位使能集（4-bit 全宽，未例化 lane 高位补 0）——sideband 掩码取用
    // ★ 修复（2026-10-20）：必须从 4-bit 全宽 LP_*_MASK4 派生，不能从已被截断到
    //   [LANE_NUM-1:0] 的 LP_TX_EN_BITS/LP_RX_EN_BITS 切片。LANE_NUM < SB_LANE_NUM
    //   （如 1T1R 单 lane：LANE_NUM=1、SB_LANE_NUM=4）时，LP_*_EN_BITS 仅 [0:0]，
    //   再切 [SB_LANE_NUM-1:0]=[3:0] 越界 → 常量变 X → sbc_dispatcher 的
    //   RX_EN_LANE_NUM（popcount）不再是 elaboration 常数 → VCS
    //   Error-[V2KGEUV] Unknown or bad value for genvar。全宽切片语义等价
    //   （LANE_NUM==4 时结果与旧实现完全一致）。
    localparam [3:0] LP_LANE_EN_MASK4 = LP_TX_EN_MASK4 | LP_RX_EN_MASK4;

    // sideband 方向掩码（截断到 SB_LANE_NUM=4 硬件边界，设计说明 16）：
    // bit i=1 表示 lane i 该方向已例化（离散派生）；sideband 内部有效使能 =
    // SBC_LANE_ENABLE.lane_enable & 方向掩码（软件对未例化 lane 置位无害）
    localparam [SB_LANE_NUM-1:0] SB_TX_MASK   = LP_TX_EN_MASK4[SB_LANE_NUM-1:0];
    localparam [SB_LANE_NUM-1:0] SB_RX_MASK   = LP_RX_EN_MASK4[SB_LANE_NUM-1:0];
    // SBC_LANE_ENABLE 复位值 = 例化 lane 集（TX or RX 使能，截断到 SB 边界）
    localparam [SB_LANE_NUM-1:0] SB_LANE_EN_RST = LP_LANE_EN_MASK4[SB_LANE_NUM-1:0];

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================

    // --------------------------------------------------------------------------
    // 顶端口 ↔ 内部 packed 向量（头部假设 1；generate 循环需向量形式索引，
    // 与原 packed 端口同名，内部逻辑不感知顶端口展开形态）
    // --------------------------------------------------------------------------
    wire [LANE_NUM-1:0]                 phy_rx_clk;
    wire [LANE_NUM*CPIF_DATA_WIDTH-1:0] cpif_tx_lane_data;
    wire [LANE_NUM*CPIF_DATA_WIDTH-1:0] cpif_rx_lane_data;
    wire [LANE_NUM-1:0]                 cpif_rx_lane_data_vld;
    wire [LANE_NUM*PHY_PAD_WIDTH-1:0]   tx_analog_pad;
    wire [LANE_NUM*PHY_PAD_WIDTH-1:0]   rx_analog_pad;
    wire [LANE_NUM*TRAIN_PARAM_W-1:0]   train_param_to_analog;
    // OFFSET_CTRL_DATA 三态驱动源（MDE-CHAIN-D2DHIPITOP-1008-FM170-TRISTATE, owner ba8af74f）：
    //   lane0_RX{n}_OFFSET_CTRL_DATA 顶口为 inout（读回路径保留），dig 侧改为三态驱动：
    //   WRITE 作 OE → lane{n}_RX{j}_OFFSET_CTRL_DATA = WRITE ? DATA_i : 10'bz。
    //   DATA 数据源仍来自 train_param_to_analog 解包（下述 unpack assign 解到 *_DATA_i），
    //   顶口三态赋值在 2F 出口（后续 17×10 逐口 assign；接口类型不变，无双向接口变更）。
    wire [9:0] lane0_RX16_OFFSET_CTRL_DATA_i;
    wire [9:0] lane0_RX15_OFFSET_CTRL_DATA_i;
    wire [9:0] lane0_RX14_OFFSET_CTRL_DATA_i;
    wire [9:0] lane0_RX13_OFFSET_CTRL_DATA_i;
    wire [9:0] lane0_RX12_OFFSET_CTRL_DATA_i;
    wire [9:0] lane0_RX11_OFFSET_CTRL_DATA_i;
    wire [9:0] lane0_RX10_OFFSET_CTRL_DATA_i;
    wire [9:0] lane0_RX9_OFFSET_CTRL_DATA_i;
    wire [9:0] lane0_RX8_OFFSET_CTRL_DATA_i;
    wire [9:0] lane0_RX7_OFFSET_CTRL_DATA_i;
    wire [9:0] lane0_RX6_OFFSET_CTRL_DATA_i;
    wire [9:0] lane0_RX5_OFFSET_CTRL_DATA_i;
    wire [9:0] lane0_RX4_OFFSET_CTRL_DATA_i;
    wire [9:0] lane0_RX3_OFFSET_CTRL_DATA_i;
    wire [9:0] lane0_RX2_OFFSET_CTRL_DATA_i;
    wire [9:0] lane0_RX1_OFFSET_CTRL_DATA_i;
    wire [9:0] lane0_RX0_OFFSET_CTRL_DATA_i;

    // --------------------------------------------------------------------------
    // APB 域复位：apb_pclk 与 mcu_clk 同源同步，apb_preset_n 取 CRG 的 apb_rstn_sync
    // = mcu_rst_n 经 apb_pclk 同步释放（见头部假设 5）
    // --------------------------------------------------------------------------
    wire                            apb_preset_n;
    wire                            apb_rstn_sync;              // CRG apb_pclk 域同步复位
    wire                            mcu_rstn_sync;              // CRG mcu_clk 域同步复位

    // --------------------------------------------------------------------------
    // logic_phy_crg 集中式复位同步输出（async assert + sync deassert，见头部假设 6）
    //   mb_rst_n  → mb_clk 域 / 每 lane phy_rx_clk{i} 域；sb_rst_n → sb_tx_clk_in /
    //   sb_rx_clk 负沿域；mcu_rst_n → apb_pclk 域（apb_rstn_sync）/ mcu_clk 域（mcu_rstn_sync）
    // --------------------------------------------------------------------------
    wire                            mb_rstn_sync;                   // mb_clk 域同步复位
    wire [LANE_NUM-1:0]             rx_rstn_sync;                   // 每 lane phy_rx_clk{i} 域同步复位
    wire                            sb_tx_rstn_sync;                // sb_tx_clk_in 域同步复位
    wire                            sb_rx_rstn_sync;                // sb_rx_clk 负沿域同步复位

    // --------------------------------------------------------------------------
    // lane_top ↔ sideband 训练握手（per-lane 向量）
    //   lt_* 为 lane_top 侧视角 [LANE_NUM-1:0]；sb_* 为 sideband 侧视角
    //   [SB_LANE_NUM-1:0]（=4），两者间由 gen_sb_adapt 映射
    // --------------------------------------------------------------------------
    // TX 侧（mb_clk 域端点）：lane→sb 8 条，sb→lane 9 条
    wire [LANE_NUM-1:0]             lt_tx_train_req;            // sb→lane
    wire [LANE_NUM-1:0]             lt_tx_retrain_req;          // sb→lane
    wire [LANE_NUM-1:0]             lt_tx_train_complete_ack;   // lane→sb（level，ACTIVE 保持高）
    wire [LANE_NUM-1:0]             lt_tx_offset_req;           // lane→sb（7 步 req）
    wire [LANE_NUM-1:0]             lt_tx_vref_req;
    wire [LANE_NUM-1:0]             lt_tx_eq_req;
    wire [LANE_NUM-1:0]             lt_tx_pi_req;
    wire [LANE_NUM-1:0]             lt_tx_dcdl_req;
    wire [LANE_NUM-1:0]             lt_tx_reorder_req;
    wire [LANE_NUM-1:0]             lt_tx_data_check_req;
    wire [LANE_NUM-1:0]             lt_tx_offset_complete;      // sb→lane（7 步 complete）
    wire [LANE_NUM-1:0]             lt_tx_vref_complete;
    wire [LANE_NUM-1:0]             lt_tx_eq_complete;
    wire [LANE_NUM-1:0]             lt_tx_pi_complete;
    wire [LANE_NUM-1:0]             lt_tx_dcdl_complete;
    wire [LANE_NUM-1:0]             lt_tx_reorder_complete;
    wire [LANE_NUM-1:0]             lt_tx_data_check_complete;
    // RX 侧（phy_rx_clk{i} 域端点）：lane→sb 8 条，sb→lane 9 条
    wire [LANE_NUM-1:0]             lt_rx_train_req;            // lane→sb（RX 主动 train_req）
    wire [LANE_NUM-1:0]             lt_rx_retrain_req;          // sb→lane
    wire [LANE_NUM-1:0]             lt_rx_train_complete_ack;   // sb→lane
    wire [LANE_NUM-1:0]             lt_rx_offset_req;           // sb→lane（7 步 req）
    wire [LANE_NUM-1:0]             lt_rx_vref_req;
    wire [LANE_NUM-1:0]             lt_rx_eq_req;
    wire [LANE_NUM-1:0]             lt_rx_pi_req;
    wire [LANE_NUM-1:0]             lt_rx_dcdl_req;
    wire [LANE_NUM-1:0]             lt_rx_reorder_req;
    wire [LANE_NUM-1:0]             lt_rx_data_check_req;
    wire [LANE_NUM-1:0]             lt_rx_offset_complete;      // lane→sb（7 步 complete）
    wire [LANE_NUM-1:0]             lt_rx_vref_complete;
    wire [LANE_NUM-1:0]             lt_rx_eq_complete;
    wire [LANE_NUM-1:0]             lt_rx_pi_complete;
    wire [LANE_NUM-1:0]             lt_rx_dcdl_complete;
    wire [LANE_NUM-1:0]             lt_rx_reorder_complete;
    wire [LANE_NUM-1:0]             lt_rx_data_check_complete;

    // sideband 侧训练握手（SB_LANE_NUM=4）
    wire [SB_LANE_NUM-1:0]          sb_tx_train_req;
    wire [SB_LANE_NUM-1:0]          sb_tx_retrain_req;
    wire [SB_LANE_NUM-1:0]          sb_tx_train_complete_ack;
    wire [SB_LANE_NUM-1:0]          sb_tx_offset_req;
    wire [SB_LANE_NUM-1:0]          sb_tx_vref_req;
    wire [SB_LANE_NUM-1:0]          sb_tx_eq_req;
    wire [SB_LANE_NUM-1:0]          sb_tx_pi_req;
    wire [SB_LANE_NUM-1:0]          sb_tx_dcdl_req;
    wire [SB_LANE_NUM-1:0]          sb_tx_reorder_req;
    wire [SB_LANE_NUM-1:0]          sb_tx_data_check_req;
    wire [SB_LANE_NUM-1:0]          sb_tx_offset_complete;
    wire [SB_LANE_NUM-1:0]          sb_tx_vref_complete;
    wire [SB_LANE_NUM-1:0]          sb_tx_eq_complete;
    wire [SB_LANE_NUM-1:0]          sb_tx_pi_complete;
    wire [SB_LANE_NUM-1:0]          sb_tx_dcdl_complete;
    wire [SB_LANE_NUM-1:0]          sb_tx_reorder_complete;
    wire [SB_LANE_NUM-1:0]          sb_tx_data_check_complete;
    wire [SB_LANE_NUM-1:0]          sb_rx_train_req;
    wire [SB_LANE_NUM-1:0]          sb_rx_retrain_req;
    wire [SB_LANE_NUM-1:0]          sb_rx_train_complete_ack;
    wire [SB_LANE_NUM-1:0]          sb_rx_offset_req;
    wire [SB_LANE_NUM-1:0]          sb_rx_vref_req;
    wire [SB_LANE_NUM-1:0]          sb_rx_eq_req;
    wire [SB_LANE_NUM-1:0]          sb_rx_pi_req;
    wire [SB_LANE_NUM-1:0]          sb_rx_dcdl_req;
    wire [SB_LANE_NUM-1:0]          sb_rx_reorder_req;
    wire [SB_LANE_NUM-1:0]          sb_rx_data_check_req;
    wire [SB_LANE_NUM-1:0]          sb_rx_offset_complete;
    wire [SB_LANE_NUM-1:0]          sb_rx_vref_complete;
    wire [SB_LANE_NUM-1:0]          sb_rx_eq_complete;
    wire [SB_LANE_NUM-1:0]          sb_rx_pi_complete;
    wire [SB_LANE_NUM-1:0]          sb_rx_dcdl_complete;
    wire [SB_LANE_NUM-1:0]          sb_rx_reorder_complete;
    wire [SB_LANE_NUM-1:0]          sb_rx_data_check_complete;

    // sideband 标量时钟适配（phy_rx_clk0/1 标量端口）
    wire                            sb_phy_rx_clk0;
    wire                            sb_phy_rx_clk1;

    // --------------------------------------------------------------------------
    // lane_top 状态回流（中断 / PRBS 对齐 / RX 异步 FIFO 状态）
    // --------------------------------------------------------------------------
    wire [LANE_NUM*8-1:0]           lane_irq_raw;       // per-lane {rx[3:0], tx[3:0]}
    wire [LANE_NUM*4-1:0]           lane_irq_or4;       // per-lane 两 nibble 按 bit OR（头部假设 2）
    wire [15:0]                     apb_lane_irq;       // {lane3..lane0}[3:0]×4 → apb_csr_top
    wire [LANE_NUM*IO_NUM-1:0]      lane_aligned;       // per-lane 16 bit 对齐指示
    reg  [IO_NUM-1:0]               aligned_agg;        // 各 RX lane 按位 AND 聚合（头部假设 4）
    wire [LANE_NUM*8-1:0]           lane_fifo_status;   // per-lane 8 bit RX FIFO 状态
    wire [31:0]                     rx_fifo_status_agg; // 4 lane × 8 bit → apb_csr_top

    // --------------------------------------------------------------------------
    // apb_csr_top 具名配置输出 → lane_top（广播各 lane 实例）
    // --------------------------------------------------------------------------
    wire                            cfg_io_data_width_mode;
    wire [3:0]                      cfg_reversal_en_tx;
    wire [3:0]                      cfg_reversal_en_rx;
    wire [63:0]                     cfg_repair_map_tx;
    wire [63:0]                     cfg_repair_map_rx;
    wire [67:0]                     cfg_msb_lsb_flip_en_tx;
    wire [67:0]                     cfg_msb_lsb_flip_en_rx;
    wire [1471:0]                   cfg_prbs_seed_tx;       // 4 × 16 × 23 bit packed
    wire [1471:0]                   cfg_prbs_seed_rx;       // 4 × 16 × 23 bit packed（4T1R 仅 lane0 消费）
    wire [15:0]                     cfg_flip_sel_tx;
    wire [15:0]                     cfg_flip_sel_rx;
    wire [3:0]                      cfg_scramble_en;
    wire [3:0]                      cfg_scramble_en_tx;
    wire                            cfg_seed_load_rx;
    wire                            cfg_clear_rx;
    wire [3:0]                      cfg_seed_load_tx;
    wire [3:0]                      cfg_clear_tx;
    wire [1:0]                      cfg_train_mode;
    wire                            cfg_force_mode_switch;
    wire [6:0]                      cfg_bypass_step;
    wire [6:0]                      cfg_timeout_step_en;
    wire [15:0]                     cfg_timeout_threshold;
    wire [3:0]                      cfg_reorder_delay_step; // 悬空：spec.json 无路由，仅接住 apb_csr_top 输出
    wire                            cfg_data_check_pattern_sel;
    wire [63:0]                     cfg_vref_pattern_tx;
    wire [63:0]                     cfg_pi_pattern_tx;
    wire [63:0]                     cfg_dcdl_pattern_tx;
    wire [63:0]                     cfg_reorder_pattern_tx;
    wire [63:0]                     cfg_datacheck_pattern_tx;
    wire                            cfg_step_trigger;
    wire [6:0]                      cfg_step_force_end;
    wire [2:0]                      cfg_step_cnt;

    // --------------------------------------------------------------------------
    // apb_csr_top 具名配置输出 → sideband
    // --------------------------------------------------------------------------
    wire                            cfg_sbc_tx_enable;
    wire                            cfg_sbc_tx_manual_trigger;
    wire [15:0]                     cfg_sbc_tx_pkt_inject;
    wire                            cfg_sbc_rx_enable;
    wire                            cfg_sbc_rx_manual_read;
    wire                            cfg_sbc_rx_soft_rst;
    wire [7:0]                      cfg_sbc_rx_buf_depth;
    wire [15:0]                     cfg_sbc_lid_filter;
    wire [7:0]                      cfg_sbc_opcode_filter;
    wire [7:0]                      cfg_sbc_err_mask;
    wire                            cfg_sbc_pkt_arrival_mask;
    wire                            cfg_sbc_err_irq_mask;
    wire [1:0]                      cfg_sbc_irq_clear;
    wire [3:0]                      cfg_sbc_lane_enable;
    // 0xD00 段 模拟 PHY 控制寄存器（intf v10.3 P0-B）+ SBC 边带冗余（P1）
    wire [1423:0]                   cfg_phy_ctrl_writein_l0;   // 35*32b [31:0]=TRX_REG1...
    wire [1423:0]                   cfg_phy_ctrl_writein_l1;
    wire [1423:0]                   cfg_phy_ctrl_writein_l2;
    wire [1423:0]                   cfg_phy_ctrl_writein_l3;
    wire [7:0]                      cfg_phy_trx_en_l0;        // [0]TX_EN_ASYNC...[7]RX_RSTB_SYNC
    wire [7:0]                      cfg_phy_trx_en_l1;
    wire [7:0]                      cfg_phy_trx_en_l2;
    wire [7:0]                      cfg_phy_trx_en_l3;
    wire [67:0]                     phy_err_edge_l0_i;        // [67:64]=DOUTRFU, [4n+3:4n]=DOUTn
    wire [67:0]                     phy_err_edge_l1_i;
    wire [67:0]                     phy_err_edge_l2_i;
    wire [67:0]                     phy_err_edge_l3_i;
    wire                            cfg_sbc_txr_en;
    wire                            cfg_sbc_rxr_sel;

    // --------------------------------------------------------------------------
    // sideband → apb_csr_top 状态回流
    // --------------------------------------------------------------------------
    wire [2:0]                      sb_tx_buf_level;
    wire                            sb_tx_busy;
    wire                            sb_tx_err;
    wire [3:0]                      sb_rx_buf_level;
    wire                            sb_rx_busy;
    wire                            sb_rx_parity_err;
    wire                            sb_rx_gap_err;
    wire                            sb_rx_buf_ovf;
    wire [15:0]                     sb_rx_buf_rdata;
    wire                            sb_pkt_arrival_flag;
    wire                            sb_err_flag;
    wire                            sb_illegal_opcode;
    wire                            sb_illegal_lid;

    // sideband 中断（→ apb_csr_top.sideband_irq_i[1:0]，[3:2] tie 0）
    wire                            sb_irq_pkt_arrival;
    wire                            sb_irq_err;

    // --------------------------------------------------------------------------
    // apb_csr_top → intc 具名信号（同域无 CDC）
    // --------------------------------------------------------------------------
    wire [6:0]                      int_status;
    wire [6:0]                      int_mask_mcu;
    wire [6:0]                      int_mask_ext;

    // --------------------------------------------------------------------------
    // mcu_subsys 互连
    // --------------------------------------------------------------------------
    wire [31:0]                     cfg_mcu;        // apb_csr_top → mcu_subsys cfg broadcast
    wire                            irq_mcu;        // intc → mcu_subsys 聚合中断
`ifdef MCU_SUBSYS
    // mcu_subsys APB master 0 → apb_csr_top APB slave（logic_phy 内部 CSR）
    wire                            mcu_apb_psel;
    wire                            mcu_apb_penable;
    wire                            mcu_apb_pwrite;
    wire [15:0]                     mcu_apb_paddr;
    wire [31:0]                     mcu_apb_pwdata;
    wire [31:0]                     mcu_apb_prdata;
    wire                            mcu_apb_pready;
    wire                            mcu_apb_pslverr;
`endif
    // 族 a lane0 打包总线（16 IO × 位宽, i{n}→[n*W +: W] LSB）→ u_lane_top_lane0
    wire [17*16-1:0] cfg_offset_param_l0;
    wire [16*16-1:0] cfg_vref_param_l0;
    wire [16*20-1:0] cfg_pi_param_l0;
    wire [17*12-1:0] cfg_dcdl_param_l0;
    wire            cfg_train_param_src;
    wire [7:0]      cfg_fail_threshold;
    wire [3:0]      cfg_eq_ctle_tap;
    // 族 a lane1/2/3 打包总线（MDE-CHAIN-1008 接口贯通 2026-10-12; 位序同 lane0）
    wire [17*16-1:0] cfg_offset_param_l1;
    wire [16*16-1:0] cfg_vref_param_l1;
    wire [16*20-1:0] cfg_pi_param_l1;
    wire [17*12-1:0] cfg_dcdl_param_l1;
    wire [17*16-1:0] cfg_offset_param_l2;
    wire [16*16-1:0] cfg_vref_param_l2;
    wire [16*20-1:0] cfg_pi_param_l2;
    wire [17*12-1:0] cfg_dcdl_param_l2;
    wire [17*16-1:0] cfg_offset_param_l3;
    wire [16*16-1:0] cfg_vref_param_l3;
    wire [16*20-1:0] cfg_pi_param_l3;
    wire [17*12-1:0] cfg_dcdl_param_l3;

    integer ai;  // aligned 聚合循环变量

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================

    assign apb_preset_n = apb_rstn_sync;

    // --------------------------------------------------------------------------
    // APB 配置路由（phy_apb_* = u_apb_csr_top 的 APB slave 输入，两模式共用）
    //   MCU_SUBSYS on : mcu_subsys APB master 0 直连（现状）
    //   MCU_SUBSYS off: hipi_apb_* 外部 APB MUX 按 paddr[16] 分域（phy / ctrl）
    // --------------------------------------------------------------------------
    wire                phy_apb_psel;
    wire                phy_apb_penable;
    wire                phy_apb_pwrite;
    wire [15:0]         phy_apb_paddr;
    wire [31:0]         phy_apb_pwdata;
    wire [31:0]         phy_apb_prdata;
    wire                phy_apb_pready;
    wire                phy_apb_pslverr;
`ifdef MCU_SUBSYS
    assign phy_apb_psel     = mcu_apb_psel;
    assign phy_apb_penable  = mcu_apb_penable;
    assign phy_apb_pwrite   = mcu_apb_pwrite;
    assign phy_apb_paddr    = mcu_apb_paddr;
    assign phy_apb_pwdata   = mcu_apb_pwdata;
    assign mcu_apb_prdata   = phy_apb_prdata;
    assign mcu_apb_pready   = phy_apb_pready;
    assign mcu_apb_pslverr  = phy_apb_pslverr;
`else
    wire phy_apb_sel  = hipi_apb_psel & ~hipi_apb_paddr[16];   // phy CSR 域
    wire ctrl_apb_sel = hipi_apb_psel &  hipi_apb_paddr[16];   // ctrl CSR 域
    assign phy_apb_psel    = phy_apb_sel;
    assign phy_apb_penable = hipi_apb_penable;
    assign phy_apb_pwrite  = hipi_apb_pwrite;
    assign phy_apb_paddr   = hipi_apb_paddr[15:0];
    assign phy_apb_pwdata  = hipi_apb_pwdata;
    // ctrl 域：驱动既有 apb_pl_* 出口（下游 dig → hipi_ctrl_top，方向两模式不变）
    assign apb_psel    = ctrl_apb_sel;
    assign apb_penable = hipi_apb_penable;
    assign apb_pwrite  = hipi_apb_pwrite;
    assign apb_paddr   = hipi_apb_paddr;
    assign apb_pwdata  = hipi_apb_pwdata;
    // 读回 MUX（APB 协议同一事务至多选中一个域）
    assign hipi_apb_prdata  = ctrl_apb_sel ? apb_prdata  : phy_apb_prdata;
    assign hipi_apb_pready  = ctrl_apb_sel ? apb_pready  : phy_apb_pready;
    assign hipi_apb_pslverr = ctrl_apb_sel ? apb_pslverr : phy_apb_pslverr;
`endif

    // --------------------------------------------------------------------------
    // 顶端口 ↔ 内部 packed 向量映射（bit 布局见头部假设 1；lane0 / IO0 在 LSB）
    //   不映射入向量的输入端口（当前未消费，见头部假设 7/9/10）：
    //     cpif_tx_lane{i}_data_vld / lane{i}_RX16_L_CDR /
    //     cpif_tx_mode / cpif_tx_rst_n / cpif_tx_lpc / cpif_rx_mode / cpif_rx_rst_n
    // --------------------------------------------------------------------------
    // --- lane0（恒存在） ---
    assign phy_rx_clk[0] = phy_rx_clk0;
    assign cpif_tx_lane_data[0*CPIF_DATA_WIDTH +: CPIF_DATA_WIDTH] = cpif_tx_lane0_data;
    // vld=0 时 data 必须全 0（spec §8.4 RXDataVld 约束）
    assign cpif_rx_lane0_data     = cpif_rx_lane_data_vld[0]
                                    ? cpif_rx_lane_data[0*CPIF_DATA_WIDTH +: CPIF_DATA_WIDTH]
                                    : {CPIF_DATA_WIDTH{1'b0}};
    assign cpif_rx_lane0_data_vld = cpif_rx_lane_data_vld[0];
    assign tx_lane0_data0 = tx_analog_pad[(0*17+ 0)*64 +: 64];
    assign tx_lane0_data1 = tx_analog_pad[(0*17+ 1)*64 +: 64];
    assign tx_lane0_data2 = tx_analog_pad[(0*17+ 2)*64 +: 64];
    assign tx_lane0_data3 = tx_analog_pad[(0*17+ 3)*64 +: 64];
    assign tx_lane0_data4 = tx_analog_pad[(0*17+ 4)*64 +: 64];
    assign tx_lane0_data5 = tx_analog_pad[(0*17+ 5)*64 +: 64];
    assign tx_lane0_data6 = tx_analog_pad[(0*17+ 6)*64 +: 64];
    assign tx_lane0_data7 = tx_analog_pad[(0*17+ 7)*64 +: 64];
    assign tx_lane0_data8 = tx_analog_pad[(0*17+ 8)*64 +: 64];
    assign tx_lane0_data9 = tx_analog_pad[(0*17+ 9)*64 +: 64];
    assign tx_lane0_data10 = tx_analog_pad[(0*17+10)*64 +: 64];
    assign tx_lane0_data11 = tx_analog_pad[(0*17+11)*64 +: 64];
    assign tx_lane0_data12 = tx_analog_pad[(0*17+12)*64 +: 64];
    assign tx_lane0_data13 = tx_analog_pad[(0*17+13)*64 +: 64];
    assign tx_lane0_data14 = tx_analog_pad[(0*17+14)*64 +: 64];
    assign tx_lane0_data15 = tx_analog_pad[(0*17+15)*64 +: 64];
    assign tx_lane0_data16 = tx_analog_pad[(0*17+16)*64 +: 64];
    assign rx_analog_pad[(0*17+ 0)*64 +: 64] = rx_lane0_data0;
    assign rx_analog_pad[(0*17+ 1)*64 +: 64] = rx_lane0_data1;
    assign rx_analog_pad[(0*17+ 2)*64 +: 64] = rx_lane0_data2;
    assign rx_analog_pad[(0*17+ 3)*64 +: 64] = rx_lane0_data3;
    assign rx_analog_pad[(0*17+ 4)*64 +: 64] = rx_lane0_data4;
    assign rx_analog_pad[(0*17+ 5)*64 +: 64] = rx_lane0_data5;
    assign rx_analog_pad[(0*17+ 6)*64 +: 64] = rx_lane0_data6;
    assign rx_analog_pad[(0*17+ 7)*64 +: 64] = rx_lane0_data7;
    assign rx_analog_pad[(0*17+ 8)*64 +: 64] = rx_lane0_data8;
    assign rx_analog_pad[(0*17+ 9)*64 +: 64] = rx_lane0_data9;
    assign rx_analog_pad[(0*17+10)*64 +: 64] = rx_lane0_data10;
    assign rx_analog_pad[(0*17+11)*64 +: 64] = rx_lane0_data11;
    assign rx_analog_pad[(0*17+12)*64 +: 64] = rx_lane0_data12;
    assign rx_analog_pad[(0*17+13)*64 +: 64] = rx_lane0_data13;
    assign rx_analog_pad[(0*17+14)*64 +: 64] = rx_lane0_data14;
    assign rx_analog_pad[(0*17+15)*64 +: 64] = rx_lane0_data15;
    assign rx_analog_pad[(0*17+16)*64 +: 64] = rx_lane0_data16;
        assign {
        // offset 段 [379:139]（3 + 17×14）
        lane0_RX_OFFSET_CALIBRATION_ON, lane0_OFFSET_CTRL_CLK, lane0_OFFSET_RESET_N,
        lane0_RX16_OFFSET_CTRL_WRITE, lane0_RX16_OFFSET_CTRL_ADDR, lane0_RX16_OFFSET_CTRL_DATA_i,
        lane0_RX15_OFFSET_CTRL_WRITE, lane0_RX15_OFFSET_CTRL_ADDR, lane0_RX15_OFFSET_CTRL_DATA_i,
        lane0_RX14_OFFSET_CTRL_WRITE, lane0_RX14_OFFSET_CTRL_ADDR, lane0_RX14_OFFSET_CTRL_DATA_i,
        lane0_RX13_OFFSET_CTRL_WRITE, lane0_RX13_OFFSET_CTRL_ADDR, lane0_RX13_OFFSET_CTRL_DATA_i,
        lane0_RX12_OFFSET_CTRL_WRITE, lane0_RX12_OFFSET_CTRL_ADDR, lane0_RX12_OFFSET_CTRL_DATA_i,
        lane0_RX11_OFFSET_CTRL_WRITE, lane0_RX11_OFFSET_CTRL_ADDR, lane0_RX11_OFFSET_CTRL_DATA_i,
        lane0_RX10_OFFSET_CTRL_WRITE, lane0_RX10_OFFSET_CTRL_ADDR, lane0_RX10_OFFSET_CTRL_DATA_i,
        lane0_RX9_OFFSET_CTRL_WRITE, lane0_RX9_OFFSET_CTRL_ADDR, lane0_RX9_OFFSET_CTRL_DATA_i,
        lane0_RX8_OFFSET_CTRL_WRITE, lane0_RX8_OFFSET_CTRL_ADDR, lane0_RX8_OFFSET_CTRL_DATA_i,
        lane0_RX7_OFFSET_CTRL_WRITE, lane0_RX7_OFFSET_CTRL_ADDR, lane0_RX7_OFFSET_CTRL_DATA_i,
        lane0_RX6_OFFSET_CTRL_WRITE, lane0_RX6_OFFSET_CTRL_ADDR, lane0_RX6_OFFSET_CTRL_DATA_i,
        lane0_RX5_OFFSET_CTRL_WRITE, lane0_RX5_OFFSET_CTRL_ADDR, lane0_RX5_OFFSET_CTRL_DATA_i,
        lane0_RX4_OFFSET_CTRL_WRITE, lane0_RX4_OFFSET_CTRL_ADDR, lane0_RX4_OFFSET_CTRL_DATA_i,
        lane0_RX3_OFFSET_CTRL_WRITE, lane0_RX3_OFFSET_CTRL_ADDR, lane0_RX3_OFFSET_CTRL_DATA_i,
        lane0_RX2_OFFSET_CTRL_WRITE, lane0_RX2_OFFSET_CTRL_ADDR, lane0_RX2_OFFSET_CTRL_DATA_i,
        lane0_RX1_OFFSET_CTRL_WRITE, lane0_RX1_OFFSET_CTRL_ADDR, lane0_RX1_OFFSET_CTRL_DATA_i,
        lane0_RX0_OFFSET_CTRL_WRITE, lane0_RX0_OFFSET_CTRL_ADDR, lane0_RX0_OFFSET_CTRL_DATA_i,
        // vref 段 [138:123]（16）
        lane0_RX_VREF_CTRL_MAIN, lane0_RX_VREF_CTRL_ERR_EDGE,
        // pi 段 [122:102]（3×7 GRAY）
        lane0_PH_I_GRAY_PI, lane0_PH_Q_GRAY_PI, lane0_PH_045_GRAY_PI,
        // dcdl 段 [101:0]（17×6）
        lane0_RX16_DCDL_CTRL,
        lane0_RX15_DCDL_CTRL,
        lane0_RX14_DCDL_CTRL,
        lane0_RX13_DCDL_CTRL,
        lane0_RX12_DCDL_CTRL,
        lane0_RX11_DCDL_CTRL,
        lane0_RX10_DCDL_CTRL,
        lane0_RX9_DCDL_CTRL,
        lane0_RX8_DCDL_CTRL,
        lane0_RX7_DCDL_CTRL,
        lane0_RX6_DCDL_CTRL,
        lane0_RX5_DCDL_CTRL,
        lane0_RX4_DCDL_CTRL,
        lane0_RX3_DCDL_CTRL,
        lane0_RX2_DCDL_CTRL,
        lane0_RX1_DCDL_CTRL,
        lane0_RX0_DCDL_CTRL
    } = train_param_to_analog[0*TRAIN_PARAM_W +: TRAIN_PARAM_W];
    // ==========================================================================
    // OFFSET_CTRL_DATA 三态驱动（MDE-CHAIN-D2DHIPITOP-1008-FM170-TRISTATE, owner ba8af74f）
    // 接口类型不变（inout 保留，无双向接口变更）；dig 侧写期(write=1)驱动数据，
    // 非写期置 10'bz（高阻）放行 PHY 读回 → 消除 lane{i}_RX{j}_OFFSET_CTRL_DATA 双 active
    // 驱动（dig 恒 output reg + PHY inout 双驱动 = 170 multidriven FM FAIL 根因）。
    // WRITE 作 OE，数据源 *_DATA_i 来自 train_param_to_analog 解包（见上）。
    // ==========================================================================
    assign lane0_RX0_OFFSET_CTRL_DATA  = lane0_RX0_OFFSET_CTRL_WRITE  ? lane0_RX0_OFFSET_CTRL_DATA_i  : 10'bz;
    assign lane0_RX1_OFFSET_CTRL_DATA  = lane0_RX1_OFFSET_CTRL_WRITE  ? lane0_RX1_OFFSET_CTRL_DATA_i  : 10'bz;
    assign lane0_RX2_OFFSET_CTRL_DATA  = lane0_RX2_OFFSET_CTRL_WRITE  ? lane0_RX2_OFFSET_CTRL_DATA_i  : 10'bz;
    assign lane0_RX3_OFFSET_CTRL_DATA  = lane0_RX3_OFFSET_CTRL_WRITE  ? lane0_RX3_OFFSET_CTRL_DATA_i  : 10'bz;
    assign lane0_RX4_OFFSET_CTRL_DATA  = lane0_RX4_OFFSET_CTRL_WRITE  ? lane0_RX4_OFFSET_CTRL_DATA_i  : 10'bz;
    assign lane0_RX5_OFFSET_CTRL_DATA  = lane0_RX5_OFFSET_CTRL_WRITE  ? lane0_RX5_OFFSET_CTRL_DATA_i  : 10'bz;
    assign lane0_RX6_OFFSET_CTRL_DATA  = lane0_RX6_OFFSET_CTRL_WRITE  ? lane0_RX6_OFFSET_CTRL_DATA_i  : 10'bz;
    assign lane0_RX7_OFFSET_CTRL_DATA  = lane0_RX7_OFFSET_CTRL_WRITE  ? lane0_RX7_OFFSET_CTRL_DATA_i  : 10'bz;
    assign lane0_RX8_OFFSET_CTRL_DATA  = lane0_RX8_OFFSET_CTRL_WRITE  ? lane0_RX8_OFFSET_CTRL_DATA_i  : 10'bz;
    assign lane0_RX9_OFFSET_CTRL_DATA  = lane0_RX9_OFFSET_CTRL_WRITE  ? lane0_RX9_OFFSET_CTRL_DATA_i  : 10'bz;
    assign lane0_RX10_OFFSET_CTRL_DATA = lane0_RX10_OFFSET_CTRL_WRITE ? lane0_RX10_OFFSET_CTRL_DATA_i : 10'bz;
    assign lane0_RX11_OFFSET_CTRL_DATA = lane0_RX11_OFFSET_CTRL_WRITE ? lane0_RX11_OFFSET_CTRL_DATA_i : 10'bz;
    assign lane0_RX12_OFFSET_CTRL_DATA = lane0_RX12_OFFSET_CTRL_WRITE ? lane0_RX12_OFFSET_CTRL_DATA_i : 10'bz;
    assign lane0_RX13_OFFSET_CTRL_DATA = lane0_RX13_OFFSET_CTRL_WRITE ? lane0_RX13_OFFSET_CTRL_DATA_i : 10'bz;
    assign lane0_RX14_OFFSET_CTRL_DATA = lane0_RX14_OFFSET_CTRL_WRITE ? lane0_RX14_OFFSET_CTRL_DATA_i : 10'bz;
    assign lane0_RX15_OFFSET_CTRL_DATA = lane0_RX15_OFFSET_CTRL_WRITE ? lane0_RX15_OFFSET_CTRL_DATA_i : 10'bz;
    assign lane0_RX16_OFFSET_CTRL_DATA = lane0_RX16_OFFSET_CTRL_WRITE ? lane0_RX16_OFFSET_CTRL_DATA_i : 10'bz;
`ifdef LP_TX_LANE1_EN
    // --- lane1 ---
    assign cpif_tx_lane_data[1*CPIF_DATA_WIDTH +: CPIF_DATA_WIDTH] = cpif_tx_lane1_data;
    assign tx_lane1_data0 = tx_analog_pad[(1*17+ 0)*64 +: 64];
    assign tx_lane1_data1 = tx_analog_pad[(1*17+ 1)*64 +: 64];
    assign tx_lane1_data2 = tx_analog_pad[(1*17+ 2)*64 +: 64];
    assign tx_lane1_data3 = tx_analog_pad[(1*17+ 3)*64 +: 64];
    assign tx_lane1_data4 = tx_analog_pad[(1*17+ 4)*64 +: 64];
    assign tx_lane1_data5 = tx_analog_pad[(1*17+ 5)*64 +: 64];
    assign tx_lane1_data6 = tx_analog_pad[(1*17+ 6)*64 +: 64];
    assign tx_lane1_data7 = tx_analog_pad[(1*17+ 7)*64 +: 64];
    assign tx_lane1_data8 = tx_analog_pad[(1*17+ 8)*64 +: 64];
    assign tx_lane1_data9 = tx_analog_pad[(1*17+ 9)*64 +: 64];
    assign tx_lane1_data10 = tx_analog_pad[(1*17+10)*64 +: 64];
    assign tx_lane1_data11 = tx_analog_pad[(1*17+11)*64 +: 64];
    assign tx_lane1_data12 = tx_analog_pad[(1*17+12)*64 +: 64];
    assign tx_lane1_data13 = tx_analog_pad[(1*17+13)*64 +: 64];
    assign tx_lane1_data14 = tx_analog_pad[(1*17+14)*64 +: 64];
    assign tx_lane1_data15 = tx_analog_pad[(1*17+15)*64 +: 64];
    assign tx_lane1_data16 = tx_analog_pad[(1*17+16)*64 +: 64];
`endif
`ifdef LP_RX_LANE1_EN
    assign phy_rx_clk[1] = phy_rx_clk1;
    assign cpif_rx_lane1_data     = cpif_rx_lane_data_vld[1]
                                    ? cpif_rx_lane_data[1*CPIF_DATA_WIDTH +: CPIF_DATA_WIDTH]
                                    : {CPIF_DATA_WIDTH{1'b0}};
    assign cpif_rx_lane1_data_vld = cpif_rx_lane_data_vld[1];
    assign rx_analog_pad[(1*17+ 0)*64 +: 64] = rx_lane1_data0;
    assign rx_analog_pad[(1*17+ 1)*64 +: 64] = rx_lane1_data1;
    assign rx_analog_pad[(1*17+ 2)*64 +: 64] = rx_lane1_data2;
    assign rx_analog_pad[(1*17+ 3)*64 +: 64] = rx_lane1_data3;
    assign rx_analog_pad[(1*17+ 4)*64 +: 64] = rx_lane1_data4;
    assign rx_analog_pad[(1*17+ 5)*64 +: 64] = rx_lane1_data5;
    assign rx_analog_pad[(1*17+ 6)*64 +: 64] = rx_lane1_data6;
    assign rx_analog_pad[(1*17+ 7)*64 +: 64] = rx_lane1_data7;
    assign rx_analog_pad[(1*17+ 8)*64 +: 64] = rx_lane1_data8;
    assign rx_analog_pad[(1*17+ 9)*64 +: 64] = rx_lane1_data9;
    assign rx_analog_pad[(1*17+10)*64 +: 64] = rx_lane1_data10;
    assign rx_analog_pad[(1*17+11)*64 +: 64] = rx_lane1_data11;
    assign rx_analog_pad[(1*17+12)*64 +: 64] = rx_lane1_data12;
    assign rx_analog_pad[(1*17+13)*64 +: 64] = rx_lane1_data13;
    assign rx_analog_pad[(1*17+14)*64 +: 64] = rx_lane1_data14;
    assign rx_analog_pad[(1*17+15)*64 +: 64] = rx_lane1_data15;
    assign rx_analog_pad[(1*17+16)*64 +: 64] = rx_lane1_data16;
        assign {
        // offset 段 [379:139]（3 + 17×14）
        lane1_RX_OFFSET_CALIBRATION_ON, lane1_OFFSET_CTRL_CLK, lane1_OFFSET_RESET_N,
        lane1_RX16_OFFSET_CTRL_WRITE, lane1_RX16_OFFSET_CTRL_ADDR, lane1_RX16_OFFSET_CTRL_DATA,
        lane1_RX15_OFFSET_CTRL_WRITE, lane1_RX15_OFFSET_CTRL_ADDR, lane1_RX15_OFFSET_CTRL_DATA,
        lane1_RX14_OFFSET_CTRL_WRITE, lane1_RX14_OFFSET_CTRL_ADDR, lane1_RX14_OFFSET_CTRL_DATA,
        lane1_RX13_OFFSET_CTRL_WRITE, lane1_RX13_OFFSET_CTRL_ADDR, lane1_RX13_OFFSET_CTRL_DATA,
        lane1_RX12_OFFSET_CTRL_WRITE, lane1_RX12_OFFSET_CTRL_ADDR, lane1_RX12_OFFSET_CTRL_DATA,
        lane1_RX11_OFFSET_CTRL_WRITE, lane1_RX11_OFFSET_CTRL_ADDR, lane1_RX11_OFFSET_CTRL_DATA,
        lane1_RX10_OFFSET_CTRL_WRITE, lane1_RX10_OFFSET_CTRL_ADDR, lane1_RX10_OFFSET_CTRL_DATA,
        lane1_RX9_OFFSET_CTRL_WRITE, lane1_RX9_OFFSET_CTRL_ADDR, lane1_RX9_OFFSET_CTRL_DATA,
        lane1_RX8_OFFSET_CTRL_WRITE, lane1_RX8_OFFSET_CTRL_ADDR, lane1_RX8_OFFSET_CTRL_DATA,
        lane1_RX7_OFFSET_CTRL_WRITE, lane1_RX7_OFFSET_CTRL_ADDR, lane1_RX7_OFFSET_CTRL_DATA,
        lane1_RX6_OFFSET_CTRL_WRITE, lane1_RX6_OFFSET_CTRL_ADDR, lane1_RX6_OFFSET_CTRL_DATA,
        lane1_RX5_OFFSET_CTRL_WRITE, lane1_RX5_OFFSET_CTRL_ADDR, lane1_RX5_OFFSET_CTRL_DATA,
        lane1_RX4_OFFSET_CTRL_WRITE, lane1_RX4_OFFSET_CTRL_ADDR, lane1_RX4_OFFSET_CTRL_DATA,
        lane1_RX3_OFFSET_CTRL_WRITE, lane1_RX3_OFFSET_CTRL_ADDR, lane1_RX3_OFFSET_CTRL_DATA,
        lane1_RX2_OFFSET_CTRL_WRITE, lane1_RX2_OFFSET_CTRL_ADDR, lane1_RX2_OFFSET_CTRL_DATA,
        lane1_RX1_OFFSET_CTRL_WRITE, lane1_RX1_OFFSET_CTRL_ADDR, lane1_RX1_OFFSET_CTRL_DATA,
        lane1_RX0_OFFSET_CTRL_WRITE, lane1_RX0_OFFSET_CTRL_ADDR, lane1_RX0_OFFSET_CTRL_DATA,
        // vref 段 [138:123]（16）
        lane1_RX_VREF_CTRL_MAIN, lane1_RX_VREF_CTRL_ERR_EDGE,
        // pi 段 [122:102]（3×7 GRAY）
        lane1_PH_I_GRAY_PI, lane1_PH_Q_GRAY_PI, lane1_PH_045_GRAY_PI,
        // dcdl 段 [101:0]（17×6）
        lane1_RX16_DCDL_CTRL,
        lane1_RX15_DCDL_CTRL,
        lane1_RX14_DCDL_CTRL,
        lane1_RX13_DCDL_CTRL,
        lane1_RX12_DCDL_CTRL,
        lane1_RX11_DCDL_CTRL,
        lane1_RX10_DCDL_CTRL,
        lane1_RX9_DCDL_CTRL,
        lane1_RX8_DCDL_CTRL,
        lane1_RX7_DCDL_CTRL,
        lane1_RX6_DCDL_CTRL,
        lane1_RX5_DCDL_CTRL,
        lane1_RX4_DCDL_CTRL,
        lane1_RX3_DCDL_CTRL,
        lane1_RX2_DCDL_CTRL,
        lane1_RX1_DCDL_CTRL,
        lane1_RX0_DCDL_CTRL
    } = train_param_to_analog[1*TRAIN_PARAM_W +: TRAIN_PARAM_W];
`else
`ifdef LP_LANE1_EN
    assign phy_rx_clk[1] = 1'b0;   // 4T1R：lane1 仅 TX，无 RX 时钟 → tie0
    assign rx_analog_pad[1*PHY_PAD_WIDTH +: PHY_PAD_WIDTH] = {PHY_PAD_WIDTH{1'b0}};  // 4T1R：lane1 RX 未驱动 → tie0（W287a）
`endif  // LP_LANE1_EN
`endif
`ifdef LP_TX_LANE2_EN
    // --- lane2 ---
    assign cpif_tx_lane_data[2*CPIF_DATA_WIDTH +: CPIF_DATA_WIDTH] = cpif_tx_lane2_data;
    assign tx_lane2_data0 = tx_analog_pad[(2*17+ 0)*64 +: 64];
    assign tx_lane2_data1 = tx_analog_pad[(2*17+ 1)*64 +: 64];
    assign tx_lane2_data2 = tx_analog_pad[(2*17+ 2)*64 +: 64];
    assign tx_lane2_data3 = tx_analog_pad[(2*17+ 3)*64 +: 64];
    assign tx_lane2_data4 = tx_analog_pad[(2*17+ 4)*64 +: 64];
    assign tx_lane2_data5 = tx_analog_pad[(2*17+ 5)*64 +: 64];
    assign tx_lane2_data6 = tx_analog_pad[(2*17+ 6)*64 +: 64];
    assign tx_lane2_data7 = tx_analog_pad[(2*17+ 7)*64 +: 64];
    assign tx_lane2_data8 = tx_analog_pad[(2*17+ 8)*64 +: 64];
    assign tx_lane2_data9 = tx_analog_pad[(2*17+ 9)*64 +: 64];
    assign tx_lane2_data10 = tx_analog_pad[(2*17+10)*64 +: 64];
    assign tx_lane2_data11 = tx_analog_pad[(2*17+11)*64 +: 64];
    assign tx_lane2_data12 = tx_analog_pad[(2*17+12)*64 +: 64];
    assign tx_lane2_data13 = tx_analog_pad[(2*17+13)*64 +: 64];
    assign tx_lane2_data14 = tx_analog_pad[(2*17+14)*64 +: 64];
    assign tx_lane2_data15 = tx_analog_pad[(2*17+15)*64 +: 64];
    assign tx_lane2_data16 = tx_analog_pad[(2*17+16)*64 +: 64];
`endif
`ifdef LP_RX_LANE2_EN
    assign phy_rx_clk[2] = phy_rx_clk2;
    assign cpif_rx_lane2_data     = cpif_rx_lane_data_vld[2]
                                    ? cpif_rx_lane_data[2*CPIF_DATA_WIDTH +: CPIF_DATA_WIDTH]
                                    : {CPIF_DATA_WIDTH{1'b0}};
    assign cpif_rx_lane2_data_vld = cpif_rx_lane_data_vld[2];
    assign rx_analog_pad[(2*17+ 0)*64 +: 64] = rx_lane2_data0;
    assign rx_analog_pad[(2*17+ 1)*64 +: 64] = rx_lane2_data1;
    assign rx_analog_pad[(2*17+ 2)*64 +: 64] = rx_lane2_data2;
    assign rx_analog_pad[(2*17+ 3)*64 +: 64] = rx_lane2_data3;
    assign rx_analog_pad[(2*17+ 4)*64 +: 64] = rx_lane2_data4;
    assign rx_analog_pad[(2*17+ 5)*64 +: 64] = rx_lane2_data5;
    assign rx_analog_pad[(2*17+ 6)*64 +: 64] = rx_lane2_data6;
    assign rx_analog_pad[(2*17+ 7)*64 +: 64] = rx_lane2_data7;
    assign rx_analog_pad[(2*17+ 8)*64 +: 64] = rx_lane2_data8;
    assign rx_analog_pad[(2*17+ 9)*64 +: 64] = rx_lane2_data9;
    assign rx_analog_pad[(2*17+10)*64 +: 64] = rx_lane2_data10;
    assign rx_analog_pad[(2*17+11)*64 +: 64] = rx_lane2_data11;
    assign rx_analog_pad[(2*17+12)*64 +: 64] = rx_lane2_data12;
    assign rx_analog_pad[(2*17+13)*64 +: 64] = rx_lane2_data13;
    assign rx_analog_pad[(2*17+14)*64 +: 64] = rx_lane2_data14;
    assign rx_analog_pad[(2*17+15)*64 +: 64] = rx_lane2_data15;
    assign rx_analog_pad[(2*17+16)*64 +: 64] = rx_lane2_data16;
        assign {
        // offset 段 [379:139]（3 + 17×14）
        lane2_RX_OFFSET_CALIBRATION_ON, lane2_OFFSET_CTRL_CLK, lane2_OFFSET_RESET_N,
        lane2_RX16_OFFSET_CTRL_WRITE, lane2_RX16_OFFSET_CTRL_ADDR, lane2_RX16_OFFSET_CTRL_DATA,
        lane2_RX15_OFFSET_CTRL_WRITE, lane2_RX15_OFFSET_CTRL_ADDR, lane2_RX15_OFFSET_CTRL_DATA,
        lane2_RX14_OFFSET_CTRL_WRITE, lane2_RX14_OFFSET_CTRL_ADDR, lane2_RX14_OFFSET_CTRL_DATA,
        lane2_RX13_OFFSET_CTRL_WRITE, lane2_RX13_OFFSET_CTRL_ADDR, lane2_RX13_OFFSET_CTRL_DATA,
        lane2_RX12_OFFSET_CTRL_WRITE, lane2_RX12_OFFSET_CTRL_ADDR, lane2_RX12_OFFSET_CTRL_DATA,
        lane2_RX11_OFFSET_CTRL_WRITE, lane2_RX11_OFFSET_CTRL_ADDR, lane2_RX11_OFFSET_CTRL_DATA,
        lane2_RX10_OFFSET_CTRL_WRITE, lane2_RX10_OFFSET_CTRL_ADDR, lane2_RX10_OFFSET_CTRL_DATA,
        lane2_RX9_OFFSET_CTRL_WRITE, lane2_RX9_OFFSET_CTRL_ADDR, lane2_RX9_OFFSET_CTRL_DATA,
        lane2_RX8_OFFSET_CTRL_WRITE, lane2_RX8_OFFSET_CTRL_ADDR, lane2_RX8_OFFSET_CTRL_DATA,
        lane2_RX7_OFFSET_CTRL_WRITE, lane2_RX7_OFFSET_CTRL_ADDR, lane2_RX7_OFFSET_CTRL_DATA,
        lane2_RX6_OFFSET_CTRL_WRITE, lane2_RX6_OFFSET_CTRL_ADDR, lane2_RX6_OFFSET_CTRL_DATA,
        lane2_RX5_OFFSET_CTRL_WRITE, lane2_RX5_OFFSET_CTRL_ADDR, lane2_RX5_OFFSET_CTRL_DATA,
        lane2_RX4_OFFSET_CTRL_WRITE, lane2_RX4_OFFSET_CTRL_ADDR, lane2_RX4_OFFSET_CTRL_DATA,
        lane2_RX3_OFFSET_CTRL_WRITE, lane2_RX3_OFFSET_CTRL_ADDR, lane2_RX3_OFFSET_CTRL_DATA,
        lane2_RX2_OFFSET_CTRL_WRITE, lane2_RX2_OFFSET_CTRL_ADDR, lane2_RX2_OFFSET_CTRL_DATA,
        lane2_RX1_OFFSET_CTRL_WRITE, lane2_RX1_OFFSET_CTRL_ADDR, lane2_RX1_OFFSET_CTRL_DATA,
        lane2_RX0_OFFSET_CTRL_WRITE, lane2_RX0_OFFSET_CTRL_ADDR, lane2_RX0_OFFSET_CTRL_DATA,
        // vref 段 [138:123]（16）
        lane2_RX_VREF_CTRL_MAIN, lane2_RX_VREF_CTRL_ERR_EDGE,
        // pi 段 [122:102]（3×7 GRAY）
        lane2_PH_I_GRAY_PI, lane2_PH_Q_GRAY_PI, lane2_PH_045_GRAY_PI,
        // dcdl 段 [101:0]（17×6）
        lane2_RX16_DCDL_CTRL,
        lane2_RX15_DCDL_CTRL,
        lane2_RX14_DCDL_CTRL,
        lane2_RX13_DCDL_CTRL,
        lane2_RX12_DCDL_CTRL,
        lane2_RX11_DCDL_CTRL,
        lane2_RX10_DCDL_CTRL,
        lane2_RX9_DCDL_CTRL,
        lane2_RX8_DCDL_CTRL,
        lane2_RX7_DCDL_CTRL,
        lane2_RX6_DCDL_CTRL,
        lane2_RX5_DCDL_CTRL,
        lane2_RX4_DCDL_CTRL,
        lane2_RX3_DCDL_CTRL,
        lane2_RX2_DCDL_CTRL,
        lane2_RX1_DCDL_CTRL,
        lane2_RX0_DCDL_CTRL
    } = train_param_to_analog[2*TRAIN_PARAM_W +: TRAIN_PARAM_W];
`else
`ifdef LP_LANE2_EN
    assign phy_rx_clk[2] = 1'b0;   // 4T1R：lane2 仅 TX，无 RX 时钟 → tie0
    assign rx_analog_pad[2*PHY_PAD_WIDTH +: PHY_PAD_WIDTH] = {PHY_PAD_WIDTH{1'b0}};  // 4T1R：lane2 RX 未驱动 → tie0（W287a）
`endif  // LP_LANE2_EN
`endif
`ifdef LP_TX_LANE3_EN
    // --- lane3 ---
    assign cpif_tx_lane_data[3*CPIF_DATA_WIDTH +: CPIF_DATA_WIDTH] = cpif_tx_lane3_data;
    assign tx_lane3_data0 = tx_analog_pad[(3*17+ 0)*64 +: 64];
    assign tx_lane3_data1 = tx_analog_pad[(3*17+ 1)*64 +: 64];
    assign tx_lane3_data2 = tx_analog_pad[(3*17+ 2)*64 +: 64];
    assign tx_lane3_data3 = tx_analog_pad[(3*17+ 3)*64 +: 64];
    assign tx_lane3_data4 = tx_analog_pad[(3*17+ 4)*64 +: 64];
    assign tx_lane3_data5 = tx_analog_pad[(3*17+ 5)*64 +: 64];
    assign tx_lane3_data6 = tx_analog_pad[(3*17+ 6)*64 +: 64];
    assign tx_lane3_data7 = tx_analog_pad[(3*17+ 7)*64 +: 64];
    assign tx_lane3_data8 = tx_analog_pad[(3*17+ 8)*64 +: 64];
    assign tx_lane3_data9 = tx_analog_pad[(3*17+ 9)*64 +: 64];
    assign tx_lane3_data10 = tx_analog_pad[(3*17+10)*64 +: 64];
    assign tx_lane3_data11 = tx_analog_pad[(3*17+11)*64 +: 64];
    assign tx_lane3_data12 = tx_analog_pad[(3*17+12)*64 +: 64];
    assign tx_lane3_data13 = tx_analog_pad[(3*17+13)*64 +: 64];
    assign tx_lane3_data14 = tx_analog_pad[(3*17+14)*64 +: 64];
    assign tx_lane3_data15 = tx_analog_pad[(3*17+15)*64 +: 64];
    assign tx_lane3_data16 = tx_analog_pad[(3*17+16)*64 +: 64];
`endif
`ifdef LP_RX_LANE3_EN
    assign phy_rx_clk[3] = phy_rx_clk3;
    assign cpif_rx_lane3_data     = cpif_rx_lane_data_vld[3]
                                    ? cpif_rx_lane_data[3*CPIF_DATA_WIDTH +: CPIF_DATA_WIDTH]
                                    : {CPIF_DATA_WIDTH{1'b0}};
    assign cpif_rx_lane3_data_vld = cpif_rx_lane_data_vld[3];
    assign rx_analog_pad[(3*17+ 0)*64 +: 64] = rx_lane3_data0;
    assign rx_analog_pad[(3*17+ 1)*64 +: 64] = rx_lane3_data1;
    assign rx_analog_pad[(3*17+ 2)*64 +: 64] = rx_lane3_data2;
    assign rx_analog_pad[(3*17+ 3)*64 +: 64] = rx_lane3_data3;
    assign rx_analog_pad[(3*17+ 4)*64 +: 64] = rx_lane3_data4;
    assign rx_analog_pad[(3*17+ 5)*64 +: 64] = rx_lane3_data5;
    assign rx_analog_pad[(3*17+ 6)*64 +: 64] = rx_lane3_data6;
    assign rx_analog_pad[(3*17+ 7)*64 +: 64] = rx_lane3_data7;
    assign rx_analog_pad[(3*17+ 8)*64 +: 64] = rx_lane3_data8;
    assign rx_analog_pad[(3*17+ 9)*64 +: 64] = rx_lane3_data9;
    assign rx_analog_pad[(3*17+10)*64 +: 64] = rx_lane3_data10;
    assign rx_analog_pad[(3*17+11)*64 +: 64] = rx_lane3_data11;
    assign rx_analog_pad[(3*17+12)*64 +: 64] = rx_lane3_data12;
    assign rx_analog_pad[(3*17+13)*64 +: 64] = rx_lane3_data13;
    assign rx_analog_pad[(3*17+14)*64 +: 64] = rx_lane3_data14;
    assign rx_analog_pad[(3*17+15)*64 +: 64] = rx_lane3_data15;
    assign rx_analog_pad[(3*17+16)*64 +: 64] = rx_lane3_data16;
        assign {
        // offset 段 [379:139]（3 + 17×14）
        lane3_RX_OFFSET_CALIBRATION_ON, lane3_OFFSET_CTRL_CLK, lane3_OFFSET_RESET_N,
        lane3_RX16_OFFSET_CTRL_WRITE, lane3_RX16_OFFSET_CTRL_ADDR, lane3_RX16_OFFSET_CTRL_DATA,
        lane3_RX15_OFFSET_CTRL_WRITE, lane3_RX15_OFFSET_CTRL_ADDR, lane3_RX15_OFFSET_CTRL_DATA,
        lane3_RX14_OFFSET_CTRL_WRITE, lane3_RX14_OFFSET_CTRL_ADDR, lane3_RX14_OFFSET_CTRL_DATA,
        lane3_RX13_OFFSET_CTRL_WRITE, lane3_RX13_OFFSET_CTRL_ADDR, lane3_RX13_OFFSET_CTRL_DATA,
        lane3_RX12_OFFSET_CTRL_WRITE, lane3_RX12_OFFSET_CTRL_ADDR, lane3_RX12_OFFSET_CTRL_DATA,
        lane3_RX11_OFFSET_CTRL_WRITE, lane3_RX11_OFFSET_CTRL_ADDR, lane3_RX11_OFFSET_CTRL_DATA,
        lane3_RX10_OFFSET_CTRL_WRITE, lane3_RX10_OFFSET_CTRL_ADDR, lane3_RX10_OFFSET_CTRL_DATA,
        lane3_RX9_OFFSET_CTRL_WRITE, lane3_RX9_OFFSET_CTRL_ADDR, lane3_RX9_OFFSET_CTRL_DATA,
        lane3_RX8_OFFSET_CTRL_WRITE, lane3_RX8_OFFSET_CTRL_ADDR, lane3_RX8_OFFSET_CTRL_DATA,
        lane3_RX7_OFFSET_CTRL_WRITE, lane3_RX7_OFFSET_CTRL_ADDR, lane3_RX7_OFFSET_CTRL_DATA,
        lane3_RX6_OFFSET_CTRL_WRITE, lane3_RX6_OFFSET_CTRL_ADDR, lane3_RX6_OFFSET_CTRL_DATA,
        lane3_RX5_OFFSET_CTRL_WRITE, lane3_RX5_OFFSET_CTRL_ADDR, lane3_RX5_OFFSET_CTRL_DATA,
        lane3_RX4_OFFSET_CTRL_WRITE, lane3_RX4_OFFSET_CTRL_ADDR, lane3_RX4_OFFSET_CTRL_DATA,
        lane3_RX3_OFFSET_CTRL_WRITE, lane3_RX3_OFFSET_CTRL_ADDR, lane3_RX3_OFFSET_CTRL_DATA,
        lane3_RX2_OFFSET_CTRL_WRITE, lane3_RX2_OFFSET_CTRL_ADDR, lane3_RX2_OFFSET_CTRL_DATA,
        lane3_RX1_OFFSET_CTRL_WRITE, lane3_RX1_OFFSET_CTRL_ADDR, lane3_RX1_OFFSET_CTRL_DATA,
        lane3_RX0_OFFSET_CTRL_WRITE, lane3_RX0_OFFSET_CTRL_ADDR, lane3_RX0_OFFSET_CTRL_DATA,
        // vref 段 [138:123]（16）
        lane3_RX_VREF_CTRL_MAIN, lane3_RX_VREF_CTRL_ERR_EDGE,
        // pi 段 [122:102]（3×7 GRAY）
        lane3_PH_I_GRAY_PI, lane3_PH_Q_GRAY_PI, lane3_PH_045_GRAY_PI,
        // dcdl 段 [101:0]（17×6）
        lane3_RX16_DCDL_CTRL,
        lane3_RX15_DCDL_CTRL,
        lane3_RX14_DCDL_CTRL,
        lane3_RX13_DCDL_CTRL,
        lane3_RX12_DCDL_CTRL,
        lane3_RX11_DCDL_CTRL,
        lane3_RX10_DCDL_CTRL,
        lane3_RX9_DCDL_CTRL,
        lane3_RX8_DCDL_CTRL,
        lane3_RX7_DCDL_CTRL,
        lane3_RX6_DCDL_CTRL,
        lane3_RX5_DCDL_CTRL,
        lane3_RX4_DCDL_CTRL,
        lane3_RX3_DCDL_CTRL,
        lane3_RX2_DCDL_CTRL,
        lane3_RX1_DCDL_CTRL,
        lane3_RX0_DCDL_CTRL
    } = train_param_to_analog[3*TRAIN_PARAM_W +: TRAIN_PARAM_W];

    // ==========================================================================
`else
`ifdef LP_LANE3_EN
    assign phy_rx_clk[3] = 1'b0;   // 4T1R：lane3 仅 TX，无 RX 时钟 → tie0
    assign rx_analog_pad[3*PHY_PAD_WIDTH +: PHY_PAD_WIDTH] = {PHY_PAD_WIDTH{1'b0}};  // 4T1R：lane3 RX 未驱动 → tie0（W287a）
`endif  // LP_LANE3_EN
`endif
    // 模拟 PHY 控制寄存器组拆包 + LPC 直连（intf logicphy_analogphy_intf v10.3, P0-B）
    // ==========================================================================
    assign phy_err_edge_l0_i = {lane0_DOUT15_ERR_EDGE, lane0_DOUT14_ERR_EDGE, lane0_DOUT13_ERR_EDGE, lane0_DOUT12_ERR_EDGE, lane0_DOUT11_ERR_EDGE, lane0_DOUT10_ERR_EDGE, lane0_DOUT9_ERR_EDGE, lane0_DOUT8_ERR_EDGE, lane0_DOUT7_ERR_EDGE, lane0_DOUT6_ERR_EDGE, lane0_DOUT5_ERR_EDGE, lane0_DOUT4_ERR_EDGE, lane0_DOUT3_ERR_EDGE, lane0_DOUT2_ERR_EDGE, lane0_DOUT1_ERR_EDGE, lane0_DOUT0_ERR_EDGE, lane0_DOUTRFU_ERR_EDGE};
    assign phy_err_edge_l1_i = {lane1_DOUT15_ERR_EDGE, lane1_DOUT14_ERR_EDGE, lane1_DOUT13_ERR_EDGE, lane1_DOUT12_ERR_EDGE, lane1_DOUT11_ERR_EDGE, lane1_DOUT10_ERR_EDGE, lane1_DOUT9_ERR_EDGE, lane1_DOUT8_ERR_EDGE, lane1_DOUT7_ERR_EDGE, lane1_DOUT6_ERR_EDGE, lane1_DOUT5_ERR_EDGE, lane1_DOUT4_ERR_EDGE, lane1_DOUT3_ERR_EDGE, lane1_DOUT2_ERR_EDGE, lane1_DOUT1_ERR_EDGE, lane1_DOUT0_ERR_EDGE, lane1_DOUTRFU_ERR_EDGE};
    assign phy_err_edge_l2_i = {lane2_DOUT15_ERR_EDGE, lane2_DOUT14_ERR_EDGE, lane2_DOUT13_ERR_EDGE, lane2_DOUT12_ERR_EDGE, lane2_DOUT11_ERR_EDGE, lane2_DOUT10_ERR_EDGE, lane2_DOUT9_ERR_EDGE, lane2_DOUT8_ERR_EDGE, lane2_DOUT7_ERR_EDGE, lane2_DOUT6_ERR_EDGE, lane2_DOUT5_ERR_EDGE, lane2_DOUT4_ERR_EDGE, lane2_DOUT3_ERR_EDGE, lane2_DOUT2_ERR_EDGE, lane2_DOUT1_ERR_EDGE, lane2_DOUT0_ERR_EDGE, lane2_DOUTRFU_ERR_EDGE};
    assign phy_err_edge_l3_i = {lane3_DOUT15_ERR_EDGE, lane3_DOUT14_ERR_EDGE, lane3_DOUT13_ERR_EDGE, lane3_DOUT12_ERR_EDGE, lane3_DOUT11_ERR_EDGE, lane3_DOUT10_ERR_EDGE, lane3_DOUT9_ERR_EDGE, lane3_DOUT8_ERR_EDGE, lane3_DOUT7_ERR_EDGE, lane3_DOUT6_ERR_EDGE, lane3_DOUT5_ERR_EDGE, lane3_DOUT4_ERR_EDGE, lane3_DOUT3_ERR_EDGE, lane3_DOUT2_ERR_EDGE, lane3_DOUT1_ERR_EDGE, lane3_DOUT0_ERR_EDGE, lane3_DOUTRFU_ERR_EDGE};
    assign lane0_TRX_REG1_WRITE_IN = cfg_phy_ctrl_writein_l0[0*32 +: 32];
    assign lane0_TRX_REG2_WRITE_IN = cfg_phy_ctrl_writein_l0[1*32 +: 32];
    assign lane0_TRX_TEST_REG1_WRITE_IN = cfg_phy_ctrl_writein_l0[2*32 +: 32];
    assign lane0_TRX_TEST_REG2_WRITE_IN = cfg_phy_ctrl_writein_l0[3*32 +: 32];
    assign lane0_RX_CLK_ENG_REG1_WRITE_IN = cfg_phy_ctrl_writein_l0[4*32 +: 32];
    assign lane0_RX_CLK_ENG_REG2_WRITE_IN = cfg_phy_ctrl_writein_l0[5*32 +: 32];
    assign lane0_RX_CLK_ENG_REG3_WRITE_IN = cfg_phy_ctrl_writein_l0[6*32 +: 32];
    assign lane0_RX_CLK_ENG_REG4_WRITE_IN = cfg_phy_ctrl_writein_l0[7*32 +: 32];
    assign lane0_RX_CLK_REG1_WRITE_IN = cfg_phy_ctrl_writein_l0[8*32 +: 32];
    assign lane0_RX_CLK_REG2_WRITE_IN = cfg_phy_ctrl_writein_l0[9*32 +: 32];
    assign lane0_RX_CLK_REG3_WRITE_IN = cfg_phy_ctrl_writein_l0[10*32 +: 32];
    assign lane0_RX_CLK_REG4_WRITE_IN = cfg_phy_ctrl_writein_l0[11*32 +: 32];
    assign lane0_RX_DATA_REG1_WRITE_IN = cfg_phy_ctrl_writein_l0[12*32 +: 32];
    assign lane0_RX_DATA_REG2_WRITE_IN = cfg_phy_ctrl_writein_l0[13*32 +: 32];
    assign lane0_RX_DATA_REG3_WRITE_IN = cfg_phy_ctrl_writein_l0[14*32 +: 32];
    assign lane0_RX_LDO_REG1_WRITE_IN = cfg_phy_ctrl_writein_l0[15*32 +: 32];
    assign lane0_RX_LDO_REG2_WRITE_IN = cfg_phy_ctrl_writein_l0[16*32 +: 32];
    assign lane0_TX_CLK_ENG_REG1_WRITE_IN = cfg_phy_ctrl_writein_l0[17*32 +: 32];
    assign lane0_TX_CLK_ENG_REG2_WRITE_IN = cfg_phy_ctrl_writein_l0[18*32 +: 32];
    assign lane0_TX_CLK_ENG_REG3_WRITE_IN = cfg_phy_ctrl_writein_l0[19*32 +: 32];
    assign lane0_TX_CLK_ENG_REG4_WRITE_IN = cfg_phy_ctrl_writein_l0[20*32 +: 32];
    assign lane0_TX_CLK_REG1_WRITE_IN = cfg_phy_ctrl_writein_l0[21*32 +: 32];
    assign lane0_TX_CLK_REG2_WRITE_IN = cfg_phy_ctrl_writein_l0[22*32 +: 32];
    assign lane0_TX_CLK_REG3_WRITE_IN = cfg_phy_ctrl_writein_l0[23*32 +: 32];
    assign lane0_TX_CLK_REG4_WRITE_IN = cfg_phy_ctrl_writein_l0[24*32 +: 32];
    assign lane0_TX_CLK_REG5_WRITE_IN = cfg_phy_ctrl_writein_l0[25*32 +: 32];
    assign lane0_TX_DATA_REG1_WRITE_IN = cfg_phy_ctrl_writein_l0[26*32 +: 32];
    assign lane0_TX_DATA_REG2_WRITE_IN = cfg_phy_ctrl_writein_l0[27*32 +: 32];
    assign lane0_TX_DATA_REG3_WRITE_IN = cfg_phy_ctrl_writein_l0[28*32 +: 32];
    assign lane0_TX_DATA_REG4_WRITE_IN = cfg_phy_ctrl_writein_l0[29*32 +: 32];
    assign lane0_TX_DATA_REG5_WRITE_IN = cfg_phy_ctrl_writein_l0[30*32 +: 32];
    assign lane0_TX_DATA_REG6_WRITE_IN = cfg_phy_ctrl_writein_l0[31*32 +: 32];
    assign lane0_TX_DATA_REG7_WRITE_IN = cfg_phy_ctrl_writein_l0[32*32 +: 32];
    assign lane0_TX_LDO_REG1_WRITE_IN = cfg_phy_ctrl_writein_l0[33*32 +: 32];
    assign lane0_TX_LDO_REG2_WRITE_IN = cfg_phy_ctrl_writein_l0[34*32 +: 32];
    assign lane1_TRX_REG1_WRITE_IN = cfg_phy_ctrl_writein_l1[0*32 +: 32];
    assign lane1_TRX_REG2_WRITE_IN = cfg_phy_ctrl_writein_l1[1*32 +: 32];
    assign lane1_TRX_TEST_REG1_WRITE_IN = cfg_phy_ctrl_writein_l1[2*32 +: 32];
    assign lane1_TRX_TEST_REG2_WRITE_IN = cfg_phy_ctrl_writein_l1[3*32 +: 32];
`ifdef LP_RX_LANE1_EN
    assign lane1_RX_CLK_ENG_REG1_WRITE_IN = cfg_phy_ctrl_writein_l1[4*32 +: 32];
    assign lane1_RX_CLK_ENG_REG2_WRITE_IN = cfg_phy_ctrl_writein_l1[5*32 +: 32];
    assign lane1_RX_CLK_ENG_REG3_WRITE_IN = cfg_phy_ctrl_writein_l1[6*32 +: 32];
    assign lane1_RX_CLK_ENG_REG4_WRITE_IN = cfg_phy_ctrl_writein_l1[7*32 +: 32];
    assign lane1_RX_CLK_REG1_WRITE_IN = cfg_phy_ctrl_writein_l1[8*32 +: 32];
    assign lane1_RX_CLK_REG2_WRITE_IN = cfg_phy_ctrl_writein_l1[9*32 +: 32];
    assign lane1_RX_CLK_REG3_WRITE_IN = cfg_phy_ctrl_writein_l1[10*32 +: 32];
    assign lane1_RX_CLK_REG4_WRITE_IN = cfg_phy_ctrl_writein_l1[11*32 +: 32];
    assign lane1_RX_DATA_REG1_WRITE_IN = cfg_phy_ctrl_writein_l1[12*32 +: 32];
    assign lane1_RX_DATA_REG2_WRITE_IN = cfg_phy_ctrl_writein_l1[13*32 +: 32];
    assign lane1_RX_DATA_REG3_WRITE_IN = cfg_phy_ctrl_writein_l1[14*32 +: 32];
    assign lane1_RX_LDO_REG1_WRITE_IN = cfg_phy_ctrl_writein_l1[15*32 +: 32];
    assign lane1_RX_LDO_REG2_WRITE_IN = cfg_phy_ctrl_writein_l1[16*32 +: 32];
`endif
`ifdef LP_TX_LANE1_EN
    assign lane1_TX_CLK_ENG_REG1_WRITE_IN = cfg_phy_ctrl_writein_l1[17*32 +: 32];
    assign lane1_TX_CLK_ENG_REG2_WRITE_IN = cfg_phy_ctrl_writein_l1[18*32 +: 32];
    assign lane1_TX_CLK_ENG_REG3_WRITE_IN = cfg_phy_ctrl_writein_l1[19*32 +: 32];
    assign lane1_TX_CLK_ENG_REG4_WRITE_IN = cfg_phy_ctrl_writein_l1[20*32 +: 32];
    assign lane1_TX_CLK_REG1_WRITE_IN = cfg_phy_ctrl_writein_l1[21*32 +: 32];
    assign lane1_TX_CLK_REG2_WRITE_IN = cfg_phy_ctrl_writein_l1[22*32 +: 32];
    assign lane1_TX_CLK_REG3_WRITE_IN = cfg_phy_ctrl_writein_l1[23*32 +: 32];
    assign lane1_TX_CLK_REG4_WRITE_IN = cfg_phy_ctrl_writein_l1[24*32 +: 32];
    assign lane1_TX_CLK_REG5_WRITE_IN = cfg_phy_ctrl_writein_l1[25*32 +: 32];
    assign lane1_TX_DATA_REG1_WRITE_IN = cfg_phy_ctrl_writein_l1[26*32 +: 32];
    assign lane1_TX_DATA_REG2_WRITE_IN = cfg_phy_ctrl_writein_l1[27*32 +: 32];
    assign lane1_TX_DATA_REG3_WRITE_IN = cfg_phy_ctrl_writein_l1[28*32 +: 32];
    assign lane1_TX_DATA_REG4_WRITE_IN = cfg_phy_ctrl_writein_l1[29*32 +: 32];
    assign lane1_TX_DATA_REG5_WRITE_IN = cfg_phy_ctrl_writein_l1[30*32 +: 32];
    assign lane1_TX_DATA_REG6_WRITE_IN = cfg_phy_ctrl_writein_l1[31*32 +: 32];
    assign lane1_TX_DATA_REG7_WRITE_IN = cfg_phy_ctrl_writein_l1[32*32 +: 32];
    assign lane1_TX_LDO_REG1_WRITE_IN = cfg_phy_ctrl_writein_l1[33*32 +: 32];
    assign lane1_TX_LDO_REG2_WRITE_IN = cfg_phy_ctrl_writein_l1[34*32 +: 32];
`endif
    assign lane2_TRX_REG1_WRITE_IN = cfg_phy_ctrl_writein_l2[0*32 +: 32];
    assign lane2_TRX_REG2_WRITE_IN = cfg_phy_ctrl_writein_l2[1*32 +: 32];
    assign lane2_TRX_TEST_REG1_WRITE_IN = cfg_phy_ctrl_writein_l2[2*32 +: 32];
    assign lane2_TRX_TEST_REG2_WRITE_IN = cfg_phy_ctrl_writein_l2[3*32 +: 32];
`ifdef LP_RX_LANE2_EN
    assign lane2_RX_CLK_ENG_REG1_WRITE_IN = cfg_phy_ctrl_writein_l2[4*32 +: 32];
    assign lane2_RX_CLK_ENG_REG2_WRITE_IN = cfg_phy_ctrl_writein_l2[5*32 +: 32];
    assign lane2_RX_CLK_ENG_REG3_WRITE_IN = cfg_phy_ctrl_writein_l2[6*32 +: 32];
    assign lane2_RX_CLK_ENG_REG4_WRITE_IN = cfg_phy_ctrl_writein_l2[7*32 +: 32];
    assign lane2_RX_CLK_REG1_WRITE_IN = cfg_phy_ctrl_writein_l2[8*32 +: 32];
    assign lane2_RX_CLK_REG2_WRITE_IN = cfg_phy_ctrl_writein_l2[9*32 +: 32];
    assign lane2_RX_CLK_REG3_WRITE_IN = cfg_phy_ctrl_writein_l2[10*32 +: 32];
    assign lane2_RX_CLK_REG4_WRITE_IN = cfg_phy_ctrl_writein_l2[11*32 +: 32];
    assign lane2_RX_DATA_REG1_WRITE_IN = cfg_phy_ctrl_writein_l2[12*32 +: 32];
    assign lane2_RX_DATA_REG2_WRITE_IN = cfg_phy_ctrl_writein_l2[13*32 +: 32];
    assign lane2_RX_DATA_REG3_WRITE_IN = cfg_phy_ctrl_writein_l2[14*32 +: 32];
    assign lane2_RX_LDO_REG1_WRITE_IN = cfg_phy_ctrl_writein_l2[15*32 +: 32];
    assign lane2_RX_LDO_REG2_WRITE_IN = cfg_phy_ctrl_writein_l2[16*32 +: 32];
`endif
`ifdef LP_TX_LANE2_EN
    assign lane2_TX_CLK_ENG_REG1_WRITE_IN = cfg_phy_ctrl_writein_l2[17*32 +: 32];
    assign lane2_TX_CLK_ENG_REG2_WRITE_IN = cfg_phy_ctrl_writein_l2[18*32 +: 32];
    assign lane2_TX_CLK_ENG_REG3_WRITE_IN = cfg_phy_ctrl_writein_l2[19*32 +: 32];
    assign lane2_TX_CLK_ENG_REG4_WRITE_IN = cfg_phy_ctrl_writein_l2[20*32 +: 32];
    assign lane2_TX_CLK_REG1_WRITE_IN = cfg_phy_ctrl_writein_l2[21*32 +: 32];
    assign lane2_TX_CLK_REG2_WRITE_IN = cfg_phy_ctrl_writein_l2[22*32 +: 32];
    assign lane2_TX_CLK_REG3_WRITE_IN = cfg_phy_ctrl_writein_l2[23*32 +: 32];
    assign lane2_TX_CLK_REG4_WRITE_IN = cfg_phy_ctrl_writein_l2[24*32 +: 32];
    assign lane2_TX_CLK_REG5_WRITE_IN = cfg_phy_ctrl_writein_l2[25*32 +: 32];
    assign lane2_TX_DATA_REG1_WRITE_IN = cfg_phy_ctrl_writein_l2[26*32 +: 32];
    assign lane2_TX_DATA_REG2_WRITE_IN = cfg_phy_ctrl_writein_l2[27*32 +: 32];
    assign lane2_TX_DATA_REG3_WRITE_IN = cfg_phy_ctrl_writein_l2[28*32 +: 32];
    assign lane2_TX_DATA_REG4_WRITE_IN = cfg_phy_ctrl_writein_l2[29*32 +: 32];
    assign lane2_TX_DATA_REG5_WRITE_IN = cfg_phy_ctrl_writein_l2[30*32 +: 32];
    assign lane2_TX_DATA_REG6_WRITE_IN = cfg_phy_ctrl_writein_l2[31*32 +: 32];
    assign lane2_TX_DATA_REG7_WRITE_IN = cfg_phy_ctrl_writein_l2[32*32 +: 32];
    assign lane2_TX_LDO_REG1_WRITE_IN = cfg_phy_ctrl_writein_l2[33*32 +: 32];
    assign lane2_TX_LDO_REG2_WRITE_IN = cfg_phy_ctrl_writein_l2[34*32 +: 32];
`endif
    assign lane3_TRX_REG1_WRITE_IN = cfg_phy_ctrl_writein_l3[0*32 +: 32];
    assign lane3_TRX_REG2_WRITE_IN = cfg_phy_ctrl_writein_l3[1*32 +: 32];
    assign lane3_TRX_TEST_REG1_WRITE_IN = cfg_phy_ctrl_writein_l3[2*32 +: 32];
    assign lane3_TRX_TEST_REG2_WRITE_IN = cfg_phy_ctrl_writein_l3[3*32 +: 32];
`ifdef LP_RX_LANE3_EN
    assign lane3_RX_CLK_ENG_REG1_WRITE_IN = cfg_phy_ctrl_writein_l3[4*32 +: 32];
    assign lane3_RX_CLK_ENG_REG2_WRITE_IN = cfg_phy_ctrl_writein_l3[5*32 +: 32];
    assign lane3_RX_CLK_ENG_REG3_WRITE_IN = cfg_phy_ctrl_writein_l3[6*32 +: 32];
    assign lane3_RX_CLK_ENG_REG4_WRITE_IN = cfg_phy_ctrl_writein_l3[7*32 +: 32];
    assign lane3_RX_CLK_REG1_WRITE_IN = cfg_phy_ctrl_writein_l3[8*32 +: 32];
    assign lane3_RX_CLK_REG2_WRITE_IN = cfg_phy_ctrl_writein_l3[9*32 +: 32];
    assign lane3_RX_CLK_REG3_WRITE_IN = cfg_phy_ctrl_writein_l3[10*32 +: 32];
    assign lane3_RX_CLK_REG4_WRITE_IN = cfg_phy_ctrl_writein_l3[11*32 +: 32];
    assign lane3_RX_DATA_REG1_WRITE_IN = cfg_phy_ctrl_writein_l3[12*32 +: 32];
    assign lane3_RX_DATA_REG2_WRITE_IN = cfg_phy_ctrl_writein_l3[13*32 +: 32];
    assign lane3_RX_DATA_REG3_WRITE_IN = cfg_phy_ctrl_writein_l3[14*32 +: 32];
    assign lane3_RX_LDO_REG1_WRITE_IN = cfg_phy_ctrl_writein_l3[15*32 +: 32];
    assign lane3_RX_LDO_REG2_WRITE_IN = cfg_phy_ctrl_writein_l3[16*32 +: 32];
`endif
`ifdef LP_TX_LANE3_EN
    assign lane3_TX_CLK_ENG_REG1_WRITE_IN = cfg_phy_ctrl_writein_l3[17*32 +: 32];
    assign lane3_TX_CLK_ENG_REG2_WRITE_IN = cfg_phy_ctrl_writein_l3[18*32 +: 32];
    assign lane3_TX_CLK_ENG_REG3_WRITE_IN = cfg_phy_ctrl_writein_l3[19*32 +: 32];
    assign lane3_TX_CLK_ENG_REG4_WRITE_IN = cfg_phy_ctrl_writein_l3[20*32 +: 32];
    assign lane3_TX_CLK_REG1_WRITE_IN = cfg_phy_ctrl_writein_l3[21*32 +: 32];
    assign lane3_TX_CLK_REG2_WRITE_IN = cfg_phy_ctrl_writein_l3[22*32 +: 32];
    assign lane3_TX_CLK_REG3_WRITE_IN = cfg_phy_ctrl_writein_l3[23*32 +: 32];
    assign lane3_TX_CLK_REG4_WRITE_IN = cfg_phy_ctrl_writein_l3[24*32 +: 32];
    assign lane3_TX_CLK_REG5_WRITE_IN = cfg_phy_ctrl_writein_l3[25*32 +: 32];
    assign lane3_TX_DATA_REG1_WRITE_IN = cfg_phy_ctrl_writein_l3[26*32 +: 32];
    assign lane3_TX_DATA_REG2_WRITE_IN = cfg_phy_ctrl_writein_l3[27*32 +: 32];
    assign lane3_TX_DATA_REG3_WRITE_IN = cfg_phy_ctrl_writein_l3[28*32 +: 32];
    assign lane3_TX_DATA_REG4_WRITE_IN = cfg_phy_ctrl_writein_l3[29*32 +: 32];
    assign lane3_TX_DATA_REG5_WRITE_IN = cfg_phy_ctrl_writein_l3[30*32 +: 32];
    assign lane3_TX_DATA_REG6_WRITE_IN = cfg_phy_ctrl_writein_l3[31*32 +: 32];
    assign lane3_TX_DATA_REG7_WRITE_IN = cfg_phy_ctrl_writein_l3[32*32 +: 32];
    assign lane3_TX_LDO_REG1_WRITE_IN = cfg_phy_ctrl_writein_l3[33*32 +: 32];
    assign lane3_TX_LDO_REG2_WRITE_IN = cfg_phy_ctrl_writein_l3[34*32 +: 32];
`endif
    assign lane0_TX_EN_ASYNC = cfg_phy_trx_en_l0[0];
    assign lane0_TX_EN_SYNC = cfg_phy_trx_en_l0[1];
    assign lane0_TX_EN_TRACK = cfg_phy_trx_en_l0[2];
    assign lane0_TX_RSTB_SYNC = cfg_phy_trx_en_l0[3];
    assign lane0_RX_CLK_EN_ASYNC = cfg_phy_trx_en_l0[4];
    assign lane0_RX_CLK_EN_SYNC = cfg_phy_trx_en_l0[5];
    assign lane0_RX_CLK_EN_TRACK = cfg_phy_trx_en_l0[6];
    assign lane0_RX_RSTB_SYNC = cfg_phy_trx_en_l0[7];
    assign lane1_TX_EN_ASYNC = cfg_phy_trx_en_l1[0];
    assign lane1_TX_EN_SYNC = cfg_phy_trx_en_l1[1];
    assign lane1_TX_EN_TRACK = cfg_phy_trx_en_l1[2];
    assign lane1_TX_RSTB_SYNC = cfg_phy_trx_en_l1[3];
    assign lane1_RX_CLK_EN_ASYNC = cfg_phy_trx_en_l1[4];
    assign lane1_RX_CLK_EN_SYNC = cfg_phy_trx_en_l1[5];
    assign lane1_RX_CLK_EN_TRACK = cfg_phy_trx_en_l1[6];
    assign lane1_RX_RSTB_SYNC = cfg_phy_trx_en_l1[7];
    assign lane2_TX_EN_ASYNC = cfg_phy_trx_en_l2[0];
    assign lane2_TX_EN_SYNC = cfg_phy_trx_en_l2[1];
    assign lane2_TX_EN_TRACK = cfg_phy_trx_en_l2[2];
    assign lane2_TX_RSTB_SYNC = cfg_phy_trx_en_l2[3];
    assign lane2_RX_CLK_EN_ASYNC = cfg_phy_trx_en_l2[4];
    assign lane2_RX_CLK_EN_SYNC = cfg_phy_trx_en_l2[5];
    assign lane2_RX_CLK_EN_TRACK = cfg_phy_trx_en_l2[6];
    assign lane2_RX_RSTB_SYNC = cfg_phy_trx_en_l2[7];
    assign lane3_TX_EN_ASYNC = cfg_phy_trx_en_l3[0];
    assign lane3_TX_EN_SYNC = cfg_phy_trx_en_l3[1];
    assign lane3_TX_EN_TRACK = cfg_phy_trx_en_l3[2];
    assign lane3_TX_RSTB_SYNC = cfg_phy_trx_en_l3[3];
    assign lane3_RX_CLK_EN_ASYNC = cfg_phy_trx_en_l3[4];
    assign lane3_RX_CLK_EN_SYNC = cfg_phy_trx_en_l3[5];
    assign lane3_RX_CLK_EN_TRACK = cfg_phy_trx_en_l3[6];
    assign lane3_RX_RSTB_SYNC = cfg_phy_trx_en_l3[7];
    // LPC 直连（intf v10.3 P0-B：LPC 与 CPIF 相关信号直连，不经 CSR）
    // cpif_tx_lpc（link→logic，所有 lane 共享）→ 4 lane TX_LPC_DATA_IN（logic→analog）
    assign lane0_TX_LPC_DATA_IN = cpif_tx_lpc;
    assign lane1_TX_LPC_DATA_IN = cpif_tx_lpc;
    assign lane2_TX_LPC_DATA_IN = cpif_tx_lpc;
    assign lane3_TX_LPC_DATA_IN = cpif_tx_lpc;
    // RX LPC：0=低功耗，任一 lane 进低功耗即上报 → 4 lane AND
    // WP2：lane1~3 RX_LPC_DATA_OUT 按 LP_RX_LANE{n}_EN 门控——RX 未使能的 lane
    // 恒 1 不参与 AND（全程无该信号），使能时并入 AND 归约
`ifdef LP_RX_LANE1_EN
    wire lane1_rx_lpc_and = lane1_RX_LPC_DATA_OUT;
`else
    wire lane1_rx_lpc_and = 1'b1;   // lane1 RX 未使能：常量 1
`endif
`ifdef LP_RX_LANE2_EN
    wire lane2_rx_lpc_and = lane2_RX_LPC_DATA_OUT;
`else
    wire lane2_rx_lpc_and = 1'b1;   // lane2 RX 未使能：常量 1
`endif
`ifdef LP_RX_LANE3_EN
    wire lane3_rx_lpc_and = lane3_RX_LPC_DATA_OUT;
`else
    wire lane3_rx_lpc_and = 1'b1;   // lane3 RX 未使能：常量 1
`endif
    assign cpif_rx_lpc = lane0_RX_LPC_DATA_OUT & lane1_rx_lpc_and
                       & lane2_rx_lpc_and & lane3_rx_lpc_and;

    // --------------------------------------------------------------------------
    // CPIF ready 静态驱动（见头部假设 10；mode/rst_n/lpc 为 link→logic 输入，当前未消费）
    // --------------------------------------------------------------------------
    // TODO: ready 来源待裁决（是否由训练完成 / ACTIVE 状态驱动），暂常就绪
    assign cpif_tx_ready  = 1'b1;
    assign cpif_rx_ready  = 1'b1;

    // --------------------------------------------------------------------------
    // CPIF RX data_vld —— 按 spec 实现（原 TODO「暂常有效」已修复）
    //
    //   spec 依据：
    //     spec/link_layer/link_layer_spec.md:722
    //     spec/link_layer/submods/link_rx/link_rx_mas.md:138
    //       「cpif_rx_lane{i}_data_vld：RX 每 lane 数据有效指示（不可靠，需
    //         flit_boundary_detect 过滤）；vld=0 时对应 lane data 必须为全 0」
    //
    //   原实现 assign cpif_rx_lane_data_vld = {LANE_NUM{1'b1}}（恒有效）导致：
    //     rx_async_fifo 空窗期仍宣告"数据有效" → link_rx 抓到陈旧/无效数据
    //     → flit_boundary_detect 失锁 → CRC 连续报错 → NACK 风暴 → 重传数据
    //     仍被误判 → R/B flit 永远解不出（AW/AR/W 因连续发送 FIFO 不空而侥幸通过）。
    //
    //   改为 per-lane ~empty：rx_async_fifo 空 → vld=0（配合下方数据全 0 门控，
    //   满足 spec「vld=0 时 data 必须全 0」）。
    //   lane_fifo_status[gi*8 +: 8] = {2'b00, empty, full, 4'h0}（见 lane_top.v:382），
    //   故 bit5 = empty。
    // --------------------------------------------------------------------------
    genvar gv_cpif_vld;
    generate
        for (gv_cpif_vld = 0; gv_cpif_vld < LANE_NUM; gv_cpif_vld = gv_cpif_vld + 1) begin : gen_rx_cpif_vld
            assign cpif_rx_lane_data_vld[gv_cpif_vld] = ~lane_fifo_status[gv_cpif_vld*8 + 5];
        end
    endgenerate

    // --------------------------------------------------------------------------
    // lane 中断 nibble OR（见头部假设 2：该 OR 本应归 lane_top，待 spec 裁决下沉）
    // --------------------------------------------------------------------------
    genvar gi;
    generate
        for (gi = 0; gi < LANE_NUM; gi = gi + 1) begin : gen_lane_irq_or
            assign lane_irq_or4[gi*4 +: 4] = lane_irq_raw[gi*8+4 +: 4]   // rx[3:0]
                                           | lane_irq_raw[gi*8   +: 4];  // tx[3:0]
        end
    endgenerate

    // apb_csr_top.lane_top_irq_i[15:0] = {lane3..lane0}[3:0]×4（2026-08-28 起扩 4 lane，
    // 4T1R 下 LANE_NUM=4 全接入；LANE_NUM<4 余位补 0：高低 nibble 均左对齐到高 lane）
    generate
        if (LANE_NUM >= 4) begin : gen_irq_map4
            assign apb_lane_irq = {lane_irq_or4[15:12], lane_irq_or4[11:8],
                                   lane_irq_or4[7:4],   lane_irq_or4[3:0]};
        end
        else if (LANE_NUM == 3) begin : gen_irq_map3
            assign apb_lane_irq = {4'b0000, lane_irq_or4[11:8],
                                   lane_irq_or4[7:4], lane_irq_or4[3:0]};
        end
        else if (LANE_NUM == 2) begin : gen_irq_map2
            assign apb_lane_irq = {8'b0000_0000, lane_irq_or4[7:4], lane_irq_or4[3:0]};
        end
        else begin : gen_irq_map1
            assign apb_lane_irq = {12'b0000_0000_0000, lane_irq_or4[3:0]};
        end
    endgenerate

    // --------------------------------------------------------------------------
    // PRBS 对齐状态聚合：各 lane 按位 AND（全 lane 全 IO 对齐 = 链路对齐，
    // 聚合语义待 spec 明确，见头部假设 4）
    // --------------------------------------------------------------------------
    always @(*) begin
        aligned_agg = {IO_NUM{1'b1}};
        for (ai = 0; ai < LANE_NUM; ai = ai + 1) begin
            if (LP_RX_EN_BITS[ai]) begin
                // RX 未例化 lane（lane_top tie 0）不参与 AND 聚合
                aligned_agg = aligned_agg & lane_aligned[ai*IO_NUM +: IO_NUM];
            end
        end
    end

    // --------------------------------------------------------------------------
    // RX 异步 FIFO 状态按 lane 拼装：lanes < LANE_NUM 取 per-lane slice，
    // 未例化 lane 补 8'h20（空 FIFO 默认态：empty[5]=1，见 lane_top tie-off）
    // --------------------------------------------------------------------------
    genvar fi2;
    generate
        for (fi2 = 0; fi2 < 4; fi2 = fi2 + 1) begin : gen_rx_fifo_status_pad
            if (fi2 < LANE_NUM) begin : gen_lane_status
                assign rx_fifo_status_agg[fi2*8 +: 8] = lane_fifo_status[fi2*8 +: 8];
            end
            else begin : gen_pad_status
                assign rx_fifo_status_agg[fi2*8 +: 8] = 8'h20;
            end
        end
    endgenerate

    // --------------------------------------------------------------------------
    // sideband 标量时钟适配：lane0 恒接 phy_rx_clk0；LANE_NUM<2 时 phy_rx_clk1
    // 复接 lane0 时钟（sideband lane1 输入已 tie 0，时钟仅作 CDC 端点不消费数据）
    // --------------------------------------------------------------------------
    assign sb_phy_rx_clk0 = phy_rx_clk[0];
    generate
        if (LANE_NUM >= 2) begin : gen_sb_clk2
            assign sb_phy_rx_clk1 = phy_rx_clk[1];
        end
        else begin : gen_sb_clk1
            // LANE_NUM==1：复接 lane0 时钟（占位，sideband lane1 不消费数据）
            assign sb_phy_rx_clk1 = phy_rx_clk[0];
        end
    endgenerate

    // --------------------------------------------------------------------------
    // lane_top ↔ sideband 训练握手位宽适配（sideband lane 数 = SB_LANE_NUM=4，头部假设 3；
    // 4T1R 下 LANE_NUM==SB_LANE_NUM 直连，pad/trunc 分支保留兼容运行时参数化）
    // --------------------------------------------------------------------------
    generate
        if (LANE_NUM == SB_LANE_NUM) begin : gen_sb_direct
            // baseline：LANE_NUM==2 直连
            assign lt_tx_train_req              = sb_tx_train_req;
            assign lt_tx_retrain_req            = sb_tx_retrain_req;
            assign sb_tx_train_complete_ack     = lt_tx_train_complete_ack;
            assign sb_tx_offset_req             = lt_tx_offset_req;
            assign sb_tx_vref_req               = lt_tx_vref_req;
            assign sb_tx_eq_req                 = lt_tx_eq_req;
            assign sb_tx_pi_req                 = lt_tx_pi_req;
            assign sb_tx_dcdl_req               = lt_tx_dcdl_req;
            assign sb_tx_reorder_req            = lt_tx_reorder_req;
            assign sb_tx_data_check_req         = lt_tx_data_check_req;
            assign lt_tx_offset_complete        = sb_tx_offset_complete;
            assign lt_tx_vref_complete          = sb_tx_vref_complete;
            assign lt_tx_eq_complete            = sb_tx_eq_complete;
            assign lt_tx_pi_complete            = sb_tx_pi_complete;
            assign lt_tx_dcdl_complete          = sb_tx_dcdl_complete;
            assign lt_tx_reorder_complete       = sb_tx_reorder_complete;
            assign lt_tx_data_check_complete    = sb_tx_data_check_complete;
            assign sb_rx_train_req              = lt_rx_train_req;
            assign lt_rx_retrain_req            = sb_rx_retrain_req;
            assign lt_rx_train_complete_ack     = sb_rx_train_complete_ack;
            assign lt_rx_offset_req             = sb_rx_offset_req;
            assign lt_rx_vref_req               = sb_rx_vref_req;
            assign lt_rx_eq_req                 = sb_rx_eq_req;
            assign lt_rx_pi_req                 = sb_rx_pi_req;
            assign lt_rx_dcdl_req               = sb_rx_dcdl_req;
            assign lt_rx_reorder_req            = sb_rx_reorder_req;
            assign lt_rx_data_check_req         = sb_rx_data_check_req;
            assign sb_rx_offset_complete        = lt_rx_offset_complete;
            assign sb_rx_vref_complete          = lt_rx_vref_complete;
            assign sb_rx_eq_complete            = lt_rx_eq_complete;
            assign sb_rx_pi_complete            = lt_rx_pi_complete;
            assign sb_rx_dcdl_complete          = lt_rx_dcdl_complete;
            assign sb_rx_reorder_complete       = lt_rx_reorder_complete;
            assign sb_rx_data_check_complete    = lt_rx_data_check_complete;
        end
        else if (LANE_NUM < SB_LANE_NUM) begin : gen_sb_pad
            // LANE_NUM==1：sideband lane1 输入 tie 0，输出仅 lane0 消费
            assign lt_tx_train_req              = sb_tx_train_req[0];
            assign lt_tx_retrain_req            = sb_tx_retrain_req[0];
            assign sb_tx_train_complete_ack     = {{(SB_LANE_NUM-1){1'b0}}, lt_tx_train_complete_ack};
            assign sb_tx_offset_req             = {{(SB_LANE_NUM-1){1'b0}}, lt_tx_offset_req};
            assign sb_tx_vref_req               = {{(SB_LANE_NUM-1){1'b0}}, lt_tx_vref_req};
            assign sb_tx_eq_req                 = {{(SB_LANE_NUM-1){1'b0}}, lt_tx_eq_req};
            assign sb_tx_pi_req                 = {{(SB_LANE_NUM-1){1'b0}}, lt_tx_pi_req};
            assign sb_tx_dcdl_req               = {{(SB_LANE_NUM-1){1'b0}}, lt_tx_dcdl_req};
            assign sb_tx_reorder_req            = {{(SB_LANE_NUM-1){1'b0}}, lt_tx_reorder_req};
            assign sb_tx_data_check_req         = {{(SB_LANE_NUM-1){1'b0}}, lt_tx_data_check_req};
            assign lt_tx_offset_complete        = sb_tx_offset_complete[0];
            assign lt_tx_vref_complete          = sb_tx_vref_complete[0];
            assign lt_tx_eq_complete            = sb_tx_eq_complete[0];
            assign lt_tx_pi_complete            = sb_tx_pi_complete[0];
            assign lt_tx_dcdl_complete          = sb_tx_dcdl_complete[0];
            assign lt_tx_reorder_complete       = sb_tx_reorder_complete[0];
            assign lt_tx_data_check_complete    = sb_tx_data_check_complete[0];
            assign sb_rx_train_req              = {{(SB_LANE_NUM-1){1'b0}}, lt_rx_train_req};
            assign lt_rx_retrain_req            = sb_rx_retrain_req[0];
            assign lt_rx_train_complete_ack     = sb_rx_train_complete_ack[0];
            assign lt_rx_offset_req             = sb_rx_offset_req[0];
            assign lt_rx_vref_req               = sb_rx_vref_req[0];
            assign lt_rx_eq_req                 = sb_rx_eq_req[0];
            assign lt_rx_pi_req                 = sb_rx_pi_req[0];
            assign lt_rx_dcdl_req               = sb_rx_dcdl_req[0];
            assign lt_rx_reorder_req            = sb_rx_reorder_req[0];
            assign lt_rx_data_check_req         = sb_rx_data_check_req[0];
            assign sb_rx_offset_complete        = {{(SB_LANE_NUM-1){1'b0}}, lt_rx_offset_complete};
            assign sb_rx_vref_complete          = {{(SB_LANE_NUM-1){1'b0}}, lt_rx_vref_complete};
            assign sb_rx_eq_complete            = {{(SB_LANE_NUM-1){1'b0}}, lt_rx_eq_complete};
            assign sb_rx_pi_complete            = {{(SB_LANE_NUM-1){1'b0}}, lt_rx_pi_complete};
            assign sb_rx_dcdl_complete          = {{(SB_LANE_NUM-1){1'b0}}, lt_rx_dcdl_complete};
            assign sb_rx_reorder_complete       = {{(SB_LANE_NUM-1){1'b0}}, lt_rx_reorder_complete};
            assign sb_rx_data_check_complete    = {{(SB_LANE_NUM-1){1'b0}}, lt_rx_data_check_complete};
        end
        else begin : gen_sb_trunc
            // LANE_NUM>2：sideband 仅支持 lane0/1；lane2/3 的 sb→lane 输入 tie 0，
            // lane→sb 输出悬空（训练不可用，仅数据通路），属已知硬件限制
            assign lt_tx_train_req              = {{(LANE_NUM-SB_LANE_NUM){1'b0}}, sb_tx_train_req};
            assign lt_tx_retrain_req            = {{(LANE_NUM-SB_LANE_NUM){1'b0}}, sb_tx_retrain_req};
            assign sb_tx_train_complete_ack     = lt_tx_train_complete_ack[SB_LANE_NUM-1:0];
            assign sb_tx_offset_req             = lt_tx_offset_req[SB_LANE_NUM-1:0];
            assign sb_tx_vref_req               = lt_tx_vref_req[SB_LANE_NUM-1:0];
            assign sb_tx_eq_req                 = lt_tx_eq_req[SB_LANE_NUM-1:0];
            assign sb_tx_pi_req                 = lt_tx_pi_req[SB_LANE_NUM-1:0];
            assign sb_tx_dcdl_req               = lt_tx_dcdl_req[SB_LANE_NUM-1:0];
            assign sb_tx_reorder_req            = lt_tx_reorder_req[SB_LANE_NUM-1:0];
            assign sb_tx_data_check_req         = lt_tx_data_check_req[SB_LANE_NUM-1:0];
            assign lt_tx_offset_complete        = {{(LANE_NUM-SB_LANE_NUM){1'b0}}, sb_tx_offset_complete};
            assign lt_tx_vref_complete          = {{(LANE_NUM-SB_LANE_NUM){1'b0}}, sb_tx_vref_complete};
            assign lt_tx_eq_complete            = {{(LANE_NUM-SB_LANE_NUM){1'b0}}, sb_tx_eq_complete};
            assign lt_tx_pi_complete            = {{(LANE_NUM-SB_LANE_NUM){1'b0}}, sb_tx_pi_complete};
            assign lt_tx_dcdl_complete          = {{(LANE_NUM-SB_LANE_NUM){1'b0}}, sb_tx_dcdl_complete};
            assign lt_tx_reorder_complete       = {{(LANE_NUM-SB_LANE_NUM){1'b0}}, sb_tx_reorder_complete};
            assign lt_tx_data_check_complete    = {{(LANE_NUM-SB_LANE_NUM){1'b0}}, sb_tx_data_check_complete};
            assign sb_rx_train_req              = lt_rx_train_req[SB_LANE_NUM-1:0];
            assign lt_rx_retrain_req            = {{(LANE_NUM-SB_LANE_NUM){1'b0}}, sb_rx_retrain_req};
            assign lt_rx_train_complete_ack     = {{(LANE_NUM-SB_LANE_NUM){1'b0}}, sb_rx_train_complete_ack};
            assign lt_rx_offset_req             = {{(LANE_NUM-SB_LANE_NUM){1'b0}}, sb_rx_offset_req};
            assign lt_rx_vref_req               = {{(LANE_NUM-SB_LANE_NUM){1'b0}}, sb_rx_vref_req};
            assign lt_rx_eq_req                 = {{(LANE_NUM-SB_LANE_NUM){1'b0}}, sb_rx_eq_req};
            assign lt_rx_pi_req                 = {{(LANE_NUM-SB_LANE_NUM){1'b0}}, sb_rx_pi_req};
            assign lt_rx_dcdl_req               = {{(LANE_NUM-SB_LANE_NUM){1'b0}}, sb_rx_dcdl_req};
            assign lt_rx_reorder_req            = {{(LANE_NUM-SB_LANE_NUM){1'b0}}, sb_rx_reorder_req};
            assign lt_rx_data_check_req         = {{(LANE_NUM-SB_LANE_NUM){1'b0}}, sb_rx_data_check_req};
            assign sb_rx_offset_complete        = lt_rx_offset_complete[SB_LANE_NUM-1:0];
            assign sb_rx_vref_complete          = lt_rx_vref_complete[SB_LANE_NUM-1:0];
            assign sb_rx_eq_complete            = lt_rx_eq_complete[SB_LANE_NUM-1:0];
            assign sb_rx_pi_complete            = lt_rx_pi_complete[SB_LANE_NUM-1:0];
            assign sb_rx_dcdl_complete          = lt_rx_dcdl_complete[SB_LANE_NUM-1:0];
            assign sb_rx_reorder_complete       = lt_rx_reorder_complete[SB_LANE_NUM-1:0];
            assign sb_rx_data_check_complete    = lt_rx_data_check_complete[SB_LANE_NUM-1:0];
        end
    endgenerate

    // ==========================================================================
    // 子模块实例化
    // ==========================================================================

    // --------------------------------------------------------------------------
    // logic_phy_crg 集中式复位同步：mb_rst_n / sb_rst_n 各时钟域 async assert +
    // sync deassert 后统一分发（见头部假设 6）。DFT test_mode 未接顶（当前叶模块
    // 均 hardcode，logic_phy 无 DFT 顶端口），CRG 内单点 tie 保一致，留待 DFT 连线。
    // --------------------------------------------------------------------------
    logic_phy_crg #(
        .LANE_NUM   (LANE_NUM),
        .RX_EN_BITS (LP_RX_EN_BITS)   // per-lane RX 使能（LP_RX_EN_MASK4 派生，4T1R=4'b0001 仅 lane0）
    ) u_logic_phy_crg (
        // clock
        .mb_clk           (mb_clk),
        .phy_rx_clk       (phy_rx_clk),
        .sb_tx_clk_in     (sb_tx_clk_in),
        .sb_rx_clk        (sb_rx_clk),
        .apb_pclk         (apb_pclk),
        .mcu_clk          (mcu_clk),
        // async hard reset
        .mb_rst_n         (mb_rst_n),
        .sb_rst_n         (sb_rst_n),
        .mcu_rst_n        (mcu_rst_n),
        .presetn          (presetn),
        // DFT（单点 tie；待 DFT 连线时改顶端口）
        .test_mode        (1'b0),
        .test_resetn_in   (1'b1),
        // synced reset output
        .mb_rstn_sync     (mb_rstn_sync),
        .rx_rstn_sync     (rx_rstn_sync),
        .sb_tx_rstn_sync  (sb_tx_rstn_sync),
        .sb_rx_rstn_sync  (sb_rx_rstn_sync),
        .apb_rstn_sync    (apb_rstn_sync),
        .mcu_rstn_sync    (mcu_rstn_sync)
    );

    // --------------------------------------------------------------------------
    // lane_top × 4（2026-08-28 由 gen_lane generate 循环改为直接显式例化，非对称 4T1R：
    //   lane0 双向 TX+RX（TX_EN=1/RX_EN=1），lane1~3 仅 TX（TX_EN=1/RX_EN=0）
    //   TX 侧 per-lane 配置（prbs_seed/repair_map/msb_lsb_flip_en/flip_sel/reversal/
    //     scramble/seed_load/clear）按 lane slice 分发（cfg_*_tx[N*W +: W]）；
    //   RX 侧单 lane 配置仅 lane0 消费（lane1~3 RX_EN=0 由 lane_top tie-off 不消费）；
    //   per-link 共享 pattern / train_mode / step 等仍广播）
    // --------------------------------------------------------------------------

    lane_top #(
        .DATA_WIDTH        (CPIF_DATA_WIDTH),
        .IO_NUM            (IO_NUM),
        .PHY_IO_NUM        (17),
        .IO_WIDTH          (64),
        .TX_MB_PIPE_DEPTH  (TX_MB_PIPE_DEPTH),
        .RX_MB_PIPE_DEPTH  (RX_MB_PIPE_DEPTH),
        .TRAIN_PARAM_WIDTH (TRAIN_PARAM_W),
        .TX_EN             (1),             // lane0: TX 使能
        .RX_EN             (1)              // lane0: RX 使能
    ) u_lane_top_lane0 (
        .mb_clk                      (mb_clk),
        .mb_rst_n                    (mb_rstn_sync),   // mb_clk 域同步复位（logic_phy_crg）
        .phy_rx_clk                  (phy_rx_clk[0]),
        .rx_rst_n                    (rx_rstn_sync[0]), // phy_rx_clk0 域同步复位（logic_phy_crg）
        // 主带数据通道
        .tx_lane_data                (cpif_tx_lane_data[0*CPIF_DATA_WIDTH +: CPIF_DATA_WIDTH]),
        .tx_analog_pad               (tx_analog_pad[0*PHY_PAD_WIDTH +: PHY_PAD_WIDTH]),
        .rx_analog_pad               (rx_analog_pad[0*PHY_PAD_WIDTH +: PHY_PAD_WIDTH]),
        .rx_lane_data                (cpif_rx_lane_data[0*CPIF_DATA_WIDTH +: CPIF_DATA_WIDTH]),
        // PRBS 对齐状态 → apb_csr_top DFX
        .aligned_o                   (lane_aligned[0*IO_NUM +: IO_NUM]),
        // RX 异步 FIFO 状态 → apb_csr_top RX_ASYNC_FIFO_STATUS（issue #13 方案 C）
        .rx_async_fifo_status        (lane_fifo_status[0*8 +: 8]),
        // 训练参数下发 → analog PHY
        .train_param_to_analog       (train_param_to_analog[0*TRAIN_PARAM_W +: TRAIN_PARAM_W]),
        // TX 训练启动 / 重训练 / 完成应答
        .train_req_i                 (lt_tx_train_req[0]),
        .retrain_req_i               (lt_tx_retrain_req[0]),
        .train_complete_ack_o        (lt_tx_train_complete_ack[0]),
        // TX per-step 训练请求（lane→sideband）
        .train_offset_req_o          (lt_tx_offset_req[0]),
        .train_vref_req_o            (lt_tx_vref_req[0]),
        .train_eq_req_o              (lt_tx_eq_req[0]),
        .train_pi_req_o              (lt_tx_pi_req[0]),
        .train_dcDL_req_o            (lt_tx_dcdl_req[0]),
        .train_reorder_req_o         (lt_tx_reorder_req[0]),
        .train_data_check_req_o      (lt_tx_data_check_req[0]),
        // TX per-step 训练完成（sideband→lane）
        .train_offset_complete_i     (lt_tx_offset_complete[0]),
        .train_vref_complete_i       (lt_tx_vref_complete[0]),
        .train_eq_complete_i         (lt_tx_eq_complete[0]),
        .train_pi_complete_i         (lt_tx_pi_complete[0]),
        .train_dcDL_complete_i       (lt_tx_dcdl_complete[0]),
        .train_reorder_complete_i    (lt_tx_reorder_complete[0]),
        .train_data_check_complete_i (lt_tx_data_check_complete[0]),
        // RX 训练启动 / 重训练 / 完成应答
        .train_req_o                 (lt_rx_train_req[0]),
        .rx_retrain_req_i            (lt_rx_retrain_req[0]),
        .train_complete_ack_i        (lt_rx_train_complete_ack[0]),
        // RX per-step 训练请求（sideband→lane）
        .train_offset_req_i          (lt_rx_offset_req[0]),
        .train_vref_req_i            (lt_rx_vref_req[0]),
        .train_eq_req_i              (lt_rx_eq_req[0]),
        .train_pi_req_i              (lt_rx_pi_req[0]),
        .train_dcDL_req_i            (lt_rx_dcdl_req[0]),
        .train_reorder_req_i         (lt_rx_reorder_req[0]),
        .train_data_check_req_i      (lt_rx_data_check_req[0]),
        // RX per-step 训练完成（lane→sideband）
        .train_offset_complete_o     (lt_rx_offset_complete[0]),
        .train_vref_complete_o       (lt_rx_vref_complete[0]),
        .train_eq_complete_o         (lt_rx_eq_complete[0]),
        .train_pi_complete_o         (lt_rx_pi_complete[0]),
        .train_dcDL_complete_o       (lt_rx_dcdl_complete[0]),
        .train_reorder_complete_o    (lt_rx_reorder_complete[0]),
        .train_data_check_complete_o (lt_rx_data_check_complete[0]),
        // APB CSR 具名配置（tx_chain 路径，per-lane slice）
        .io_data_width_mode          (cfg_io_data_width_mode),
        .prbs_seed_tx                (cfg_prbs_seed_tx[0*368 +: 368]),
        .flip_sel_tx                 (cfg_flip_sel_tx[0*4 +: 4]),
        .reversal_en_tx              (cfg_reversal_en_tx[0]),
        .repair_map_tx               (cfg_repair_map_tx[0*16 +: 16]),
        .msb_lsb_flip_en_tx          (cfg_msb_lsb_flip_en_tx[0*17 +: 17]),
        .vref_pattern_tx             (cfg_vref_pattern_tx),
        .pi_pattern_tx               (cfg_pi_pattern_tx),
        .dcdl_pattern_tx             (cfg_dcdl_pattern_tx),
        .reorder_pattern_tx          (cfg_reorder_pattern_tx),
        .datacheck_pattern_tx        (cfg_datacheck_pattern_tx),
        .data_check_pattern_sel      (cfg_data_check_pattern_sel),
        .scramble_en_tx              (cfg_scramble_en_tx[0]),
        .seed_load_tx                (cfg_seed_load_tx[0]),
        .clear_tx                    (cfg_clear_tx[0]),
        // APB CSR 具名配置（rx_chain 路径，per-lane slice → lane0；4T1R 唯一 RX lane）
        .msb_lsb_flip_en_rx          (cfg_msb_lsb_flip_en_rx[0*17 +: 17]),
        .reversal_en_rx              (cfg_reversal_en_rx[0]),
        .repair_map_rx               (cfg_repair_map_rx[0*16 +: 16]),
        .prbs_seed_rx                (cfg_prbs_seed_rx[0*368 +: 368]),
        .flip_sel_rx                 (cfg_flip_sel_rx[0*4 +: 4]),
        .scramble_en                 (cfg_scramble_en[0]),
        .seed_load_i                 (cfg_seed_load_rx),
        .clear                       (cfg_clear_rx),
        // APB CSR 具名配置（训练 FSM 路径，广播）
        .train_mode                  (cfg_train_mode),
        .force_mode_switch           (cfg_force_mode_switch),
        .bypass_step                 (cfg_bypass_step),
        .timeout_step_en             (cfg_timeout_step_en),
        .timeout_threshold           (cfg_timeout_threshold),
        .step_trigger                (cfg_step_trigger),
        .step_force_end              (cfg_step_force_end),
        .step_cnt                    (cfg_step_cnt),
        // 中断（per-lane 8 bit {rx[3:0], tx[3:0]}，本层 OR 成 4 bit）
        .irq_lane_o                  (lane_irq_raw[0*8 +: 8]),
                .offset_param                 (cfg_offset_param_l0),
                .vref_param                   (cfg_vref_param_l0),
                .pi_param                     (cfg_pi_param_l0),
                .dcdl_param                   (cfg_dcdl_param_l0),
                .train_param_src_i            (cfg_train_param_src),
                .fail_threshold_i             (cfg_fail_threshold),
                .ctle_tap_i                   (cfg_eq_ctle_tap)
    );

    `ifdef LP_LANE1_EN
    lane_top #(
        .DATA_WIDTH        (CPIF_DATA_WIDTH),
        .IO_NUM            (IO_NUM),
        .PHY_IO_NUM        (17),
        .IO_WIDTH          (64),
        .TX_MB_PIPE_DEPTH  (TX_MB_PIPE_DEPTH),
        .RX_MB_PIPE_DEPTH  (RX_MB_PIPE_DEPTH),
        .TRAIN_PARAM_WIDTH (TRAIN_PARAM_W),
        .TX_EN             (1),             // lane1: TX 使能
        .RX_EN             (0)              // lane1: RX 禁能（4T1R 单 RX）
    ) u_lane_top_lane1 (
        .mb_clk                      (mb_clk),
        .mb_rst_n                    (mb_rstn_sync),
        .phy_rx_clk                  (phy_rx_clk[1]),
        .rx_rst_n                    (rx_rstn_sync[1]),
        // 主带数据通道
        .tx_lane_data                (cpif_tx_lane_data[1*CPIF_DATA_WIDTH +: CPIF_DATA_WIDTH]),
        .tx_analog_pad               (tx_analog_pad[1*PHY_PAD_WIDTH +: PHY_PAD_WIDTH]),
        .rx_analog_pad               (rx_analog_pad[1*PHY_PAD_WIDTH +: PHY_PAD_WIDTH]),
        .rx_lane_data                (cpif_rx_lane_data[1*CPIF_DATA_WIDTH +: CPIF_DATA_WIDTH]),
        // PRBS 对齐状态 → apb_csr_top DFX（RX_EN=0 tie 0，聚合处剔除）
        .aligned_o                   (lane_aligned[1*IO_NUM +: IO_NUM]),
        // RX 异步 FIFO 状态 → apb_csr_top RX_ASYNC_FIFO_STATUS（RX 未例化则 lane_top tie 8'h20）
        .rx_async_fifo_status        (lane_fifo_status[1*8 +: 8]),
        // 训练参数下发 → analog PHY（RX_EN=0 tie 0）
        .train_param_to_analog       (train_param_to_analog[1*TRAIN_PARAM_W +: TRAIN_PARAM_W]),
        // TX 训练启动 / 重训练 / 完成应答
        .train_req_i                 (lt_tx_train_req[1]),
        .retrain_req_i               (lt_tx_retrain_req[1]),
        .train_complete_ack_o        (lt_tx_train_complete_ack[1]),
        // TX per-step 训练请求（lane→sideband）
        .train_offset_req_o          (lt_tx_offset_req[1]),
        .train_vref_req_o            (lt_tx_vref_req[1]),
        .train_eq_req_o              (lt_tx_eq_req[1]),
        .train_pi_req_o              (lt_tx_pi_req[1]),
        .train_dcDL_req_o            (lt_tx_dcdl_req[1]),
        .train_reorder_req_o         (lt_tx_reorder_req[1]),
        .train_data_check_req_o      (lt_tx_data_check_req[1]),
        // TX per-step 训练完成（sideband→lane）
        .train_offset_complete_i     (lt_tx_offset_complete[1]),
        .train_vref_complete_i       (lt_tx_vref_complete[1]),
        .train_eq_complete_i         (lt_tx_eq_complete[1]),
        .train_pi_complete_i         (lt_tx_pi_complete[1]),
        .train_dcDL_complete_i       (lt_tx_dcdl_complete[1]),
        .train_reorder_complete_i    (lt_tx_reorder_complete[1]),
        .train_data_check_complete_i (lt_tx_data_check_complete[1]),
        // RX 训练启动 / 重训练 / 完成应答（RX_EN=0，输出 tie 0 / 输入不消费）
        .train_req_o                 (lt_rx_train_req[1]),
        .rx_retrain_req_i            (lt_rx_retrain_req[1]),
        .train_complete_ack_i        (lt_rx_train_complete_ack[1]),
        // RX per-step 训练请求（sideband→lane，RX_EN=0 不消费）
        .train_offset_req_i          (lt_rx_offset_req[1]),
        .train_vref_req_i            (lt_rx_vref_req[1]),
        .train_eq_req_i              (lt_rx_eq_req[1]),
        .train_pi_req_i              (lt_rx_pi_req[1]),
        .train_dcDL_req_i            (lt_rx_dcdl_req[1]),
        .train_reorder_req_i         (lt_rx_reorder_req[1]),
        .train_data_check_req_i      (lt_rx_data_check_req[1]),
        // RX per-step 训练完成（lane→sideband，RX_EN=0 tie 0）
        .train_offset_complete_o     (lt_rx_offset_complete[1]),
        .train_vref_complete_o       (lt_rx_vref_complete[1]),
        .train_eq_complete_o         (lt_rx_eq_complete[1]),
        .train_pi_complete_o         (lt_rx_pi_complete[1]),
        .train_dcDL_complete_o       (lt_rx_dcdl_complete[1]),
        .train_reorder_complete_o    (lt_rx_reorder_complete[1]),
        .train_data_check_complete_o (lt_rx_data_check_complete[1]),
        // APB CSR 具名配置（tx_chain 路径，per-lane slice）
        .io_data_width_mode          (cfg_io_data_width_mode),
        .prbs_seed_tx                (cfg_prbs_seed_tx[1*368 +: 368]),
        .flip_sel_tx                 (cfg_flip_sel_tx[1*4 +: 4]),
        .reversal_en_tx              (cfg_reversal_en_tx[1]),
        .repair_map_tx               (cfg_repair_map_tx[1*16 +: 16]),
        .msb_lsb_flip_en_tx          (cfg_msb_lsb_flip_en_tx[1*17 +: 17]),
        .vref_pattern_tx             (cfg_vref_pattern_tx),
        .pi_pattern_tx               (cfg_pi_pattern_tx),
        .dcdl_pattern_tx             (cfg_dcdl_pattern_tx),
        .reorder_pattern_tx          (cfg_reorder_pattern_tx),
        .datacheck_pattern_tx        (cfg_datacheck_pattern_tx),
        .data_check_pattern_sel      (cfg_data_check_pattern_sel),
        .scramble_en_tx              (cfg_scramble_en_tx[1]),
        .seed_load_tx                (cfg_seed_load_tx[1]),
        .clear_tx                    (cfg_clear_tx[1]),
        // APB CSR 具名配置（rx_chain 路径，RX_EN=0 不消费，per-lane slice → lane1）
        .msb_lsb_flip_en_rx          (cfg_msb_lsb_flip_en_rx[1*17 +: 17]),
        .reversal_en_rx              (cfg_reversal_en_rx[1]),
        .repair_map_rx               (cfg_repair_map_rx[1*16 +: 16]),
        .prbs_seed_rx                (cfg_prbs_seed_rx[1*368 +: 368]),
        .flip_sel_rx                 (cfg_flip_sel_rx[1*4 +: 4]),
        .scramble_en                 (cfg_scramble_en[1]),
        .seed_load_i                 (cfg_seed_load_rx),
        .clear                       (cfg_clear_rx),
        // APB CSR 具名配置（训练 FSM 路径，广播）
        .train_mode                  (cfg_train_mode),
        .force_mode_switch           (cfg_force_mode_switch),
        .bypass_step                 (cfg_bypass_step),
        .timeout_step_en             (cfg_timeout_step_en),
        .timeout_threshold           (cfg_timeout_threshold),
        .step_trigger                (cfg_step_trigger),
        .step_force_end              (cfg_step_force_end),
        .step_cnt                    (cfg_step_cnt),
        // 中断（per-lane 8 bit {rx[3:0], tx[3:0]}，本层 OR 成 4 bit）
        .irq_lane_o                  (lane_irq_raw[1*8 +: 8]),
                // 族 a lanes  1 per-IO 训练参数：位恒 0（4T1R 仅 TX，TRAIN-MAS-01 启用后实接）
                .offset_param                 (cfg_offset_param_l1),
                .vref_param                   (cfg_vref_param_l1),
                .pi_param                     (cfg_pi_param_l1),
                .dcdl_param                   (cfg_dcdl_param_l1),
                .train_param_src_i            (cfg_train_param_src),
                .fail_threshold_i             (cfg_fail_threshold),
                .ctle_tap_i                   (cfg_eq_ctle_tap)
    );
    `endif  // LP_LANE1_EN

    `ifdef LP_LANE2_EN
    lane_top #(
        .DATA_WIDTH        (CPIF_DATA_WIDTH),
        .IO_NUM            (IO_NUM),
        .PHY_IO_NUM        (17),
        .IO_WIDTH          (64),
        .TX_MB_PIPE_DEPTH  (TX_MB_PIPE_DEPTH),
        .RX_MB_PIPE_DEPTH  (RX_MB_PIPE_DEPTH),
        .TRAIN_PARAM_WIDTH (TRAIN_PARAM_W),
        .TX_EN             (1),             // lane2: TX 使能
        .RX_EN             (0)              // lane2: RX 禁能（4T1R 单 RX）
    ) u_lane_top_lane2 (
        .mb_clk                      (mb_clk),
        .mb_rst_n                    (mb_rstn_sync),
        .phy_rx_clk                  (phy_rx_clk[2]),
        .rx_rst_n                    (rx_rstn_sync[2]),
        // 主带数据通道
        .tx_lane_data                (cpif_tx_lane_data[2*CPIF_DATA_WIDTH +: CPIF_DATA_WIDTH]),
        .tx_analog_pad               (tx_analog_pad[2*PHY_PAD_WIDTH +: PHY_PAD_WIDTH]),
        .rx_analog_pad               (rx_analog_pad[2*PHY_PAD_WIDTH +: PHY_PAD_WIDTH]),
        .rx_lane_data                (cpif_rx_lane_data[2*CPIF_DATA_WIDTH +: CPIF_DATA_WIDTH]),
        // PRBS 对齐状态 → apb_csr_top DFX（RX_EN=0 tie 0，聚合处剔除）
        .aligned_o                   (lane_aligned[2*IO_NUM +: IO_NUM]),
        // RX 异步 FIFO 状态 → apb_csr_top RX_ASYNC_FIFO_STATUS（RX 未例化则 lane_top tie 8'h20）
        .rx_async_fifo_status        (lane_fifo_status[2*8 +: 8]),
        // 训练参数下发 → analog PHY（RX_EN=0 tie 0）
        .train_param_to_analog       (train_param_to_analog[2*TRAIN_PARAM_W +: TRAIN_PARAM_W]),
        // TX 训练启动 / 重训练 / 完成应答
        .train_req_i                 (lt_tx_train_req[2]),
        .retrain_req_i               (lt_tx_retrain_req[2]),
        .train_complete_ack_o        (lt_tx_train_complete_ack[2]),
        // TX per-step 训练请求（lane→sideband）
        .train_offset_req_o          (lt_tx_offset_req[2]),
        .train_vref_req_o            (lt_tx_vref_req[2]),
        .train_eq_req_o              (lt_tx_eq_req[2]),
        .train_pi_req_o              (lt_tx_pi_req[2]),
        .train_dcDL_req_o            (lt_tx_dcdl_req[2]),
        .train_reorder_req_o         (lt_tx_reorder_req[2]),
        .train_data_check_req_o      (lt_tx_data_check_req[2]),
        // TX per-step 训练完成（sideband→lane）
        .train_offset_complete_i     (lt_tx_offset_complete[2]),
        .train_vref_complete_i       (lt_tx_vref_complete[2]),
        .train_eq_complete_i         (lt_tx_eq_complete[2]),
        .train_pi_complete_i         (lt_tx_pi_complete[2]),
        .train_dcDL_complete_i       (lt_tx_dcdl_complete[2]),
        .train_reorder_complete_i    (lt_tx_reorder_complete[2]),
        .train_data_check_complete_i (lt_tx_data_check_complete[2]),
        // RX 训练启动 / 重训练 / 完成应答（RX_EN=0，输出 tie 0 / 输入不消费）
        .train_req_o                 (lt_rx_train_req[2]),
        .rx_retrain_req_i            (lt_rx_retrain_req[2]),
        .train_complete_ack_i        (lt_rx_train_complete_ack[2]),
        // RX per-step 训练请求（sideband→lane，RX_EN=0 不消费）
        .train_offset_req_i          (lt_rx_offset_req[2]),
        .train_vref_req_i            (lt_rx_vref_req[2]),
        .train_eq_req_i              (lt_rx_eq_req[2]),
        .train_pi_req_i              (lt_rx_pi_req[2]),
        .train_dcDL_req_i            (lt_rx_dcdl_req[2]),
        .train_reorder_req_i         (lt_rx_reorder_req[2]),
        .train_data_check_req_i      (lt_rx_data_check_req[2]),
        // RX per-step 训练完成（lane→sideband，RX_EN=0 tie 0）
        .train_offset_complete_o     (lt_rx_offset_complete[2]),
        .train_vref_complete_o       (lt_rx_vref_complete[2]),
        .train_eq_complete_o         (lt_rx_eq_complete[2]),
        .train_pi_complete_o         (lt_rx_pi_complete[2]),
        .train_dcDL_complete_o       (lt_rx_dcdl_complete[2]),
        .train_reorder_complete_o    (lt_rx_reorder_complete[2]),
        .train_data_check_complete_o (lt_rx_data_check_complete[2]),
        // APB CSR 具名配置（tx_chain 路径，per-lane slice）
        .io_data_width_mode          (cfg_io_data_width_mode),
        .prbs_seed_tx                (cfg_prbs_seed_tx[2*368 +: 368]),
        .flip_sel_tx                 (cfg_flip_sel_tx[2*4 +: 4]),
        .reversal_en_tx              (cfg_reversal_en_tx[2]),
        .repair_map_tx               (cfg_repair_map_tx[2*16 +: 16]),
        .msb_lsb_flip_en_tx          (cfg_msb_lsb_flip_en_tx[2*17 +: 17]),
        .vref_pattern_tx             (cfg_vref_pattern_tx),
        .pi_pattern_tx               (cfg_pi_pattern_tx),
        .dcdl_pattern_tx             (cfg_dcdl_pattern_tx),
        .reorder_pattern_tx          (cfg_reorder_pattern_tx),
        .datacheck_pattern_tx        (cfg_datacheck_pattern_tx),
        .data_check_pattern_sel      (cfg_data_check_pattern_sel),
        .scramble_en_tx              (cfg_scramble_en_tx[2]),
        .seed_load_tx                (cfg_seed_load_tx[2]),
        .clear_tx                    (cfg_clear_tx[2]),
        // APB CSR 具名配置（rx_chain 路径，RX_EN=0 不消费，per-lane slice → lane2）
        .msb_lsb_flip_en_rx          (cfg_msb_lsb_flip_en_rx[2*17 +: 17]),
        .reversal_en_rx              (cfg_reversal_en_rx[2]),
        .repair_map_rx               (cfg_repair_map_rx[2*16 +: 16]),
        .prbs_seed_rx                (cfg_prbs_seed_rx[2*368 +: 368]),
        .flip_sel_rx                 (cfg_flip_sel_rx[2*4 +: 4]),
        .scramble_en                 (cfg_scramble_en[2]),
        .seed_load_i                 (cfg_seed_load_rx),
        .clear                       (cfg_clear_rx),
        // APB CSR 具名配置（训练 FSM 路径，广播）
        .train_mode                  (cfg_train_mode),
        .force_mode_switch           (cfg_force_mode_switch),
        .bypass_step                 (cfg_bypass_step),
        .timeout_step_en             (cfg_timeout_step_en),
        .timeout_threshold           (cfg_timeout_threshold),
        .step_trigger                (cfg_step_trigger),
        .step_force_end              (cfg_step_force_end),
        .step_cnt                    (cfg_step_cnt),
        // 中断（per-lane 8 bit {rx[3:0], tx[3:0]}，本层 OR 成 4 bit）
        .irq_lane_o                  (lane_irq_raw[2*8 +: 8]),
                // 族 a lanes  2 per-IO 训练参数：位恒 0（4T1R 仅 TX，TRAIN-MAS-01 启用后实接）
                .offset_param                 (cfg_offset_param_l2),
                .vref_param                   (cfg_vref_param_l2),
                .pi_param                     (cfg_pi_param_l2),
                .dcdl_param                   (cfg_dcdl_param_l2),
                .train_param_src_i            (cfg_train_param_src),
                .fail_threshold_i             (cfg_fail_threshold),
                .ctle_tap_i                   (cfg_eq_ctle_tap)
    );
    `endif  // LP_LANE2_EN

    `ifdef LP_LANE3_EN
    lane_top #(
        .DATA_WIDTH        (CPIF_DATA_WIDTH),
        .IO_NUM            (IO_NUM),
        .PHY_IO_NUM        (17),
        .IO_WIDTH          (64),
        .TX_MB_PIPE_DEPTH  (TX_MB_PIPE_DEPTH),
        .RX_MB_PIPE_DEPTH  (RX_MB_PIPE_DEPTH),
        .TRAIN_PARAM_WIDTH (TRAIN_PARAM_W),
        .TX_EN             (1),             // lane3: TX 使能
        .RX_EN             (0)              // lane3: RX 禁能（4T1R 单 RX）
    ) u_lane_top_lane3 (
        .mb_clk                      (mb_clk),
        .mb_rst_n                    (mb_rstn_sync),
        .phy_rx_clk                  (phy_rx_clk[3]),
        .rx_rst_n                    (rx_rstn_sync[3]),
        // 主带数据通道
        .tx_lane_data                (cpif_tx_lane_data[3*CPIF_DATA_WIDTH +: CPIF_DATA_WIDTH]),
        .tx_analog_pad               (tx_analog_pad[3*PHY_PAD_WIDTH +: PHY_PAD_WIDTH]),
        .rx_analog_pad               (rx_analog_pad[3*PHY_PAD_WIDTH +: PHY_PAD_WIDTH]),
        .rx_lane_data                (cpif_rx_lane_data[3*CPIF_DATA_WIDTH +: CPIF_DATA_WIDTH]),
        // PRBS 对齐状态 → apb_csr_top DFX（RX_EN=0 tie 0，聚合处剔除）
        .aligned_o                   (lane_aligned[3*IO_NUM +: IO_NUM]),
        // RX 异步 FIFO 状态 → apb_csr_top RX_ASYNC_FIFO_STATUS（RX 未例化则 lane_top tie 8'h20）
        .rx_async_fifo_status        (lane_fifo_status[3*8 +: 8]),
        // 训练参数下发 → analog PHY（RX_EN=0 tie 0）
        .train_param_to_analog       (train_param_to_analog[3*TRAIN_PARAM_W +: TRAIN_PARAM_W]),
        // TX 训练启动 / 重训练 / 完成应答
        .train_req_i                 (lt_tx_train_req[3]),
        .retrain_req_i               (lt_tx_retrain_req[3]),
        .train_complete_ack_o        (lt_tx_train_complete_ack[3]),
        // TX per-step 训练请求（lane→sideband）
        .train_offset_req_o          (lt_tx_offset_req[3]),
        .train_vref_req_o            (lt_tx_vref_req[3]),
        .train_eq_req_o              (lt_tx_eq_req[3]),
        .train_pi_req_o              (lt_tx_pi_req[3]),
        .train_dcDL_req_o            (lt_tx_dcdl_req[3]),
        .train_reorder_req_o         (lt_tx_reorder_req[3]),
        .train_data_check_req_o      (lt_tx_data_check_req[3]),
        // TX per-step 训练完成（sideband→lane）
        .train_offset_complete_i     (lt_tx_offset_complete[3]),
        .train_vref_complete_i       (lt_tx_vref_complete[3]),
        .train_eq_complete_i         (lt_tx_eq_complete[3]),
        .train_pi_complete_i         (lt_tx_pi_complete[3]),
        .train_dcDL_complete_i       (lt_tx_dcdl_complete[3]),
        .train_reorder_complete_i    (lt_tx_reorder_complete[3]),
        .train_data_check_complete_i (lt_tx_data_check_complete[3]),
        // RX 训练启动 / 重训练 / 完成应答（RX_EN=0，输出 tie 0 / 输入不消费）
        .train_req_o                 (lt_rx_train_req[3]),
        .rx_retrain_req_i            (lt_rx_retrain_req[3]),
        .train_complete_ack_i        (lt_rx_train_complete_ack[3]),
        // RX per-step 训练请求（sideband→lane，RX_EN=0 不消费）
        .train_offset_req_i          (lt_rx_offset_req[3]),
        .train_vref_req_i            (lt_rx_vref_req[3]),
        .train_eq_req_i              (lt_rx_eq_req[3]),
        .train_pi_req_i              (lt_rx_pi_req[3]),
        .train_dcDL_req_i            (lt_rx_dcdl_req[3]),
        .train_reorder_req_i         (lt_rx_reorder_req[3]),
        .train_data_check_req_i      (lt_rx_data_check_req[3]),
        // RX per-step 训练完成（lane→sideband，RX_EN=0 tie 0）
        .train_offset_complete_o     (lt_rx_offset_complete[3]),
        .train_vref_complete_o       (lt_rx_vref_complete[3]),
        .train_eq_complete_o         (lt_rx_eq_complete[3]),
        .train_pi_complete_o         (lt_rx_pi_complete[3]),
        .train_dcDL_complete_o       (lt_rx_dcdl_complete[3]),
        .train_reorder_complete_o    (lt_rx_reorder_complete[3]),
        .train_data_check_complete_o (lt_rx_data_check_complete[3]),
        // APB CSR 具名配置（tx_chain 路径，per-lane slice）
        .io_data_width_mode          (cfg_io_data_width_mode),
        .prbs_seed_tx                (cfg_prbs_seed_tx[3*368 +: 368]),
        .flip_sel_tx                 (cfg_flip_sel_tx[3*4 +: 4]),
        .reversal_en_tx              (cfg_reversal_en_tx[3]),
        .repair_map_tx               (cfg_repair_map_tx[3*16 +: 16]),
        .msb_lsb_flip_en_tx          (cfg_msb_lsb_flip_en_tx[3*17 +: 17]),
        .vref_pattern_tx             (cfg_vref_pattern_tx),
        .pi_pattern_tx               (cfg_pi_pattern_tx),
        .dcdl_pattern_tx             (cfg_dcdl_pattern_tx),
        .reorder_pattern_tx          (cfg_reorder_pattern_tx),
        .datacheck_pattern_tx        (cfg_datacheck_pattern_tx),
        .data_check_pattern_sel      (cfg_data_check_pattern_sel),
        .scramble_en_tx              (cfg_scramble_en_tx[3]),
        .seed_load_tx                (cfg_seed_load_tx[3]),
        .clear_tx                    (cfg_clear_tx[3]),
        // APB CSR 具名配置（rx_chain 路径，RX_EN=0 不消费，per-lane slice → lane3）
        .msb_lsb_flip_en_rx          (cfg_msb_lsb_flip_en_rx[3*17 +: 17]),
        .reversal_en_rx              (cfg_reversal_en_rx[3]),
        .repair_map_rx               (cfg_repair_map_rx[3*16 +: 16]),
        .prbs_seed_rx                (cfg_prbs_seed_rx[3*368 +: 368]),
        .flip_sel_rx                 (cfg_flip_sel_rx[3*4 +: 4]),
        .scramble_en                 (cfg_scramble_en[3]),
        .seed_load_i                 (cfg_seed_load_rx),
        .clear                       (cfg_clear_rx),
        // APB CSR 具名配置（训练 FSM 路径，广播）
        .train_mode                  (cfg_train_mode),
        .force_mode_switch           (cfg_force_mode_switch),
        .bypass_step                 (cfg_bypass_step),
        .timeout_step_en             (cfg_timeout_step_en),
        .timeout_threshold           (cfg_timeout_threshold),
        .step_trigger                (cfg_step_trigger),
        .step_force_end              (cfg_step_force_end),
        .step_cnt                    (cfg_step_cnt),
        // 中断（per-lane 8 bit {rx[3:0], tx[3:0]}，本层 OR 成 4 bit）
        .irq_lane_o                  (lane_irq_raw[3*8 +: 8]),
                // 族 a lanes  3 per-IO 训练参数：位恒 0（4T1R 仅 TX，TRAIN-MAS-01 启用后实接）
                .offset_param                 (cfg_offset_param_l3),
                .vref_param                   (cfg_vref_param_l3),
                .pi_param                     (cfg_pi_param_l3),
                .dcdl_param                   (cfg_dcdl_param_l3),
                .train_param_src_i            (cfg_train_param_src),
                .fail_threshold_i             (cfg_fail_threshold),
                .ctle_tap_i                   (cfg_eq_ctle_tap)
    );
    `endif  // LP_LANE3_EN

    // --------------------------------------------------------------------------
    // sideband：SBC 边带收发 + 报文分发 + CPIF event 解码（per-link 唯一，
    // lane 数 = SB_LANE_NUM=4；训练握手 LANE_NUM==SB_LANE_NUM 直连，
    // 时钟经 sb_phy_rx_clk0/1 适配；RX 单 lane（lane0）由 RX_LANE_MASK 限定）
    // --------------------------------------------------------------------------
    sideband #(
        .LANE_NUM     (SB_LANE_NUM),
        .TX_LANE_MASK (SB_TX_MASK),
        .RX_LANE_MASK (SB_RX_MASK)
    ) u_sideband (
        .sb_tx_clk_in           (sb_tx_clk_in),
        .sb_tx_clk              (sb_tx_clk),
        .sb_rx_clk              (sb_rx_clk),
        .sb_rst_n               (sb_tx_rstn_sync),   // sb_tx_clk_in 域同步复位（logic_phy_crg）
        .sb_rx_rst_n            (sb_rx_rstn_sync),   // sb_rx_clk 负沿域同步复位（logic_phy_crg）
        .mb_rstn_sync           (mb_rstn_sync),      // mb_clk 域同步复位（logic_phy_crg）
        .rx_rstn_sync           (rx_rstn_sync),      // phy_rx_clk{i} 域同步复位（logic_phy_crg）
        .mb_clk                 (mb_clk),
        .phy_rx_clk0            (sb_phy_rx_clk0),
        .phy_rx_clk1            (sb_phy_rx_clk1),
        // 边带串行通道 ↔ analog PHY
        .sb_txd_o               (sb_txd),
        .sb_rxd_i               (sb_rxd),
        .sb_txr_o               (sb_txr),
        .sb_rxr_i               (sb_rxr),
        .sb_txr_en_i            (cfg_sbc_txr_en),
        .sb_rxr_sel_i           (cfg_sbc_rxr_sel),
        // CPIF 事件接口 ↔ 顶端口透传
        .cpif_tx_event_enable_i (cpif_tx_event_enable),
        .cpif_tx_event_req_i    (cpif_tx_event_req),
        .cpif_tx_event_cmd_i    (cpif_tx_event_cmd),
        .cpif_tx_event_ack_o    (cpif_tx_event_ack),
        .cpif_tx_event_status_o (cpif_tx_event_status),
        .cpif_rx_event_enable_i (cpif_rx_event_enable),
        .cpif_rx_event_req_o    (cpif_rx_event_req),
        .cpif_rx_event_cmd_o    (cpif_rx_event_cmd),
        .cpif_rx_event_ack_i    (cpif_rx_event_ack),
        .cpif_rx_event_status_i (cpif_rx_event_status),
        // TX 训练 req/complete（mb_clk 域端点）
        .train_req_o            (sb_tx_train_req),
        .retrain_req_o          (sb_tx_retrain_req),
        .train_complete_ack_i   (sb_tx_train_complete_ack),
        .offset_req_i           (sb_tx_offset_req),
        .vref_req_i             (sb_tx_vref_req),
        .eq_req_i               (sb_tx_eq_req),
        .pi_req_i               (sb_tx_pi_req),
        .dcDL_req_i             (sb_tx_dcdl_req),
        .reorder_req_i          (sb_tx_reorder_req),
        .data_check_req_i       (sb_tx_data_check_req),
        .offset_complete_o      (sb_tx_offset_complete),
        .vref_complete_o        (sb_tx_vref_complete),
        .eq_complete_o          (sb_tx_eq_complete),
        .pi_complete_o          (sb_tx_pi_complete),
        .dcDL_complete_o        (sb_tx_dcdl_complete),
        .reorder_complete_o     (sb_tx_reorder_complete),
        .data_check_complete_o  (sb_tx_data_check_complete),
        // RX 训练 req/complete（phy_rx_clk{i} 域端点）
        .train_req_i            (sb_rx_train_req),
        .rx_retrain_req_o       (sb_rx_retrain_req),
        .train_complete_ack_o   (sb_rx_train_complete_ack),
        .offset_req_o           (sb_rx_offset_req),
        .vref_req_o             (sb_rx_vref_req),
        .eq_req_o               (sb_rx_eq_req),
        .pi_req_o               (sb_rx_pi_req),
        .dcDL_req_o             (sb_rx_dcdl_req),
        .reorder_req_o          (sb_rx_reorder_req),
        .data_check_req_o       (sb_rx_data_check_req),
        .offset_complete_i      (sb_rx_offset_complete),
        .vref_complete_i        (sb_rx_vref_complete),
        .eq_complete_i          (sb_rx_eq_complete),
        .pi_complete_i          (sb_rx_pi_complete),
        .dcDL_complete_i        (sb_rx_dcdl_complete),
        .reorder_complete_i     (sb_rx_reorder_complete),
        .data_check_complete_i  (sb_rx_data_check_complete),
        // APB CSR 具名配置（CDC 在消费侧完成）
        .sbc_tx_enable_i        (cfg_sbc_tx_enable),
        .sbc_tx_manual_trigger_i(cfg_sbc_tx_manual_trigger),
        .sbc_tx_pkt_inject_i    (cfg_sbc_tx_pkt_inject),
        .sbc_rx_enable_i        (cfg_sbc_rx_enable),
        .sbc_rx_manual_read_i   (cfg_sbc_rx_manual_read),
        .sbc_rx_soft_rst_i      (cfg_sbc_rx_soft_rst),
        .sbc_rx_buf_depth_i     (cfg_sbc_rx_buf_depth),
        .sbc_lid_filter_i       (cfg_sbc_lid_filter),
        .sbc_opcode_filter_i    (cfg_sbc_opcode_filter),
        .sbc_err_mask_i         (cfg_sbc_err_mask),
        .sbc_pkt_arrival_mask_i (cfg_sbc_pkt_arrival_mask),
        .sbc_err_irq_mask_i     (cfg_sbc_err_irq_mask),
        .sbc_irq_clear_i        (cfg_sbc_irq_clear),
        .sbc_lane_enable_i      (cfg_sbc_lane_enable),
        // 状态回流 → apb_csr_top
        .sbc_tx_buf_level_o     (sb_tx_buf_level),
        .sbc_tx_busy_o          (sb_tx_busy),
        .sbc_tx_err_o           (sb_tx_err),
        .sbc_rx_buf_level_o     (sb_rx_buf_level),
        .sbc_rx_busy_o          (sb_rx_busy),
        .sbc_rx_parity_err_o    (sb_rx_parity_err),
        .sbc_rx_gap_err_o       (sb_rx_gap_err),
        .sbc_rx_buf_ovf_o       (sb_rx_buf_ovf),
        .sbc_rx_buf_rdata_o     (sb_rx_buf_rdata),
        .sbc_pkt_arrival_flag_o (sb_pkt_arrival_flag),
        .sbc_err_flag_o         (sb_err_flag),
        .sbc_illegal_opcode_o   (sb_illegal_opcode),
        .sbc_illegal_lid_o      (sb_illegal_lid),
        // 中断 → apb_csr_top.sideband_irq_i[1:0]
        .irq_sbc_pkt_arrival_o  (sb_irq_pkt_arrival),
        .irq_sbc_err_o          (sb_irq_err)
    );

    // 族 a lane0 per-IO 训练参数（MDE-CHAIN-RERUN-01-271-IMPL; apb_csr_top 64 同名输出）
    wire [15:0] offset_param_l0_i0_offset;
    wire [15:0] vref_param_l0_i0_vref;
    wire [19:0] pi_param_l0_i0_pi;
    wire [11:0] dcdl_param_l0_i0_dcdl;
    wire [15:0] offset_param_l0_i1_offset;
    wire [15:0] vref_param_l0_i1_vref;
    wire [19:0] pi_param_l0_i1_pi;
    wire [11:0] dcdl_param_l0_i1_dcdl;
    wire [15:0] offset_param_l0_i2_offset;
    wire [15:0] vref_param_l0_i2_vref;
    wire [19:0] pi_param_l0_i2_pi;
    wire [11:0] dcdl_param_l0_i2_dcdl;
    wire [15:0] offset_param_l0_i3_offset;
    wire [15:0] vref_param_l0_i3_vref;
    wire [19:0] pi_param_l0_i3_pi;
    wire [11:0] dcdl_param_l0_i3_dcdl;
    wire [15:0] offset_param_l0_i4_offset;
    wire [15:0] vref_param_l0_i4_vref;
    wire [19:0] pi_param_l0_i4_pi;
    wire [11:0] dcdl_param_l0_i4_dcdl;
    wire [15:0] offset_param_l0_i5_offset;
    wire [15:0] vref_param_l0_i5_vref;
    wire [19:0] pi_param_l0_i5_pi;
    wire [11:0] dcdl_param_l0_i5_dcdl;
    wire [15:0] offset_param_l0_i6_offset;
    wire [15:0] vref_param_l0_i6_vref;
    wire [19:0] pi_param_l0_i6_pi;
    wire [11:0] dcdl_param_l0_i6_dcdl;
    wire [15:0] offset_param_l0_i7_offset;
    wire [15:0] vref_param_l0_i7_vref;
    wire [19:0] pi_param_l0_i7_pi;
    wire [11:0] dcdl_param_l0_i7_dcdl;
    wire [15:0] offset_param_l0_i8_offset;
    wire [15:0] vref_param_l0_i8_vref;
    wire [19:0] pi_param_l0_i8_pi;
    wire [11:0] dcdl_param_l0_i8_dcdl;
    wire [15:0] offset_param_l0_i9_offset;
    wire [15:0] vref_param_l0_i9_vref;
    wire [19:0] pi_param_l0_i9_pi;
    wire [11:0] dcdl_param_l0_i9_dcdl;
    wire [15:0] offset_param_l0_i10_offset;
    wire [15:0] vref_param_l0_i10_vref;
    wire [19:0] pi_param_l0_i10_pi;
    wire [11:0] dcdl_param_l0_i10_dcdl;
    wire [15:0] offset_param_l0_i11_offset;
    wire [15:0] vref_param_l0_i11_vref;
    wire [19:0] pi_param_l0_i11_pi;
    wire [11:0] dcdl_param_l0_i11_dcdl;
    wire [15:0] offset_param_l0_i12_offset;
    wire [15:0] vref_param_l0_i12_vref;
    wire [19:0] pi_param_l0_i12_pi;
    wire [11:0] dcdl_param_l0_i12_dcdl;
    wire [15:0] offset_param_l0_i13_offset;
    wire [15:0] vref_param_l0_i13_vref;
    wire [19:0] pi_param_l0_i13_pi;
    wire [11:0] dcdl_param_l0_i13_dcdl;
    wire [15:0] offset_param_l0_i14_offset;
    wire [15:0] vref_param_l0_i14_vref;
    wire [19:0] pi_param_l0_i14_pi;
    wire [11:0] dcdl_param_l0_i14_dcdl;
    wire [15:0] offset_param_l0_i15_offset;
    wire [15:0] vref_param_l0_i15_vref;
    wire [19:0] pi_param_l0_i15_pi;
    wire [11:0] dcdl_param_l0_i15_dcdl;
    // i16（RX16 冗余 IO）per-IO 训练参数（WP1 P0-A 2026-09-22；apb_csr_top 0x0E8/0x0EC）
    wire [15:0] offset_param_l0_i16_offset;
    wire [11:0] dcdl_param_l0_i16_dcdl;
    // MDE-CHAIN-1008 lane1 train 参数 wire（2026-10-12 接口贯通）
    wire [15:0] offset_param_l1_i0_offset;
    wire [15:0] offset_param_l1_i1_offset;
    wire [15:0] offset_param_l1_i2_offset;
    wire [15:0] offset_param_l1_i3_offset;
    wire [15:0] offset_param_l1_i4_offset;
    wire [15:0] offset_param_l1_i5_offset;
    wire [15:0] offset_param_l1_i6_offset;
    wire [15:0] offset_param_l1_i7_offset;
    wire [15:0] offset_param_l1_i8_offset;
    wire [15:0] offset_param_l1_i9_offset;
    wire [15:0] offset_param_l1_i10_offset;
    wire [15:0] offset_param_l1_i11_offset;
    wire [15:0] offset_param_l1_i12_offset;
    wire [15:0] offset_param_l1_i13_offset;
    wire [15:0] offset_param_l1_i14_offset;
    wire [15:0] offset_param_l1_i15_offset;
    wire [15:0] offset_param_l1_i16_offset;
    wire [19:0] pi_param_l1_i0_pi;
    wire [19:0] pi_param_l1_i1_pi;
    wire [19:0] pi_param_l1_i2_pi;
    wire [19:0] pi_param_l1_i3_pi;
    wire [19:0] pi_param_l1_i4_pi;
    wire [19:0] pi_param_l1_i5_pi;
    wire [19:0] pi_param_l1_i6_pi;
    wire [19:0] pi_param_l1_i7_pi;
    wire [19:0] pi_param_l1_i8_pi;
    wire [19:0] pi_param_l1_i9_pi;
    wire [19:0] pi_param_l1_i10_pi;
    wire [19:0] pi_param_l1_i11_pi;
    wire [19:0] pi_param_l1_i12_pi;
    wire [19:0] pi_param_l1_i13_pi;
    wire [19:0] pi_param_l1_i14_pi;
    wire [19:0] pi_param_l1_i15_pi;
    wire [15:0] vref_param_l1_i0_vref;
    wire [15:0] vref_param_l1_i1_vref;
    wire [15:0] vref_param_l1_i2_vref;
    wire [15:0] vref_param_l1_i3_vref;
    wire [15:0] vref_param_l1_i4_vref;
    wire [15:0] vref_param_l1_i5_vref;
    wire [15:0] vref_param_l1_i6_vref;
    wire [15:0] vref_param_l1_i7_vref;
    wire [15:0] vref_param_l1_i8_vref;
    wire [15:0] vref_param_l1_i9_vref;
    wire [15:0] vref_param_l1_i10_vref;
    wire [15:0] vref_param_l1_i11_vref;
    wire [15:0] vref_param_l1_i12_vref;
    wire [15:0] vref_param_l1_i13_vref;
    wire [15:0] vref_param_l1_i14_vref;
    wire [15:0] vref_param_l1_i15_vref;
    wire [11:0] dcdl_param_l1_i0_dcdl;
    wire [11:0] dcdl_param_l1_i1_dcdl;
    wire [11:0] dcdl_param_l1_i2_dcdl;
    wire [11:0] dcdl_param_l1_i3_dcdl;
    wire [11:0] dcdl_param_l1_i4_dcdl;
    wire [11:0] dcdl_param_l1_i5_dcdl;
    wire [11:0] dcdl_param_l1_i6_dcdl;
    wire [11:0] dcdl_param_l1_i7_dcdl;
    wire [11:0] dcdl_param_l1_i8_dcdl;
    wire [11:0] dcdl_param_l1_i9_dcdl;
    wire [11:0] dcdl_param_l1_i10_dcdl;
    wire [11:0] dcdl_param_l1_i11_dcdl;
    wire [11:0] dcdl_param_l1_i12_dcdl;
    wire [11:0] dcdl_param_l1_i13_dcdl;
    wire [11:0] dcdl_param_l1_i14_dcdl;
    wire [11:0] dcdl_param_l1_i15_dcdl;
    wire [11:0] dcdl_param_l1_i16_dcdl;
    // MDE-CHAIN-1008 lane2 train 参数 wire（2026-10-12 接口贯通）
    wire [15:0] offset_param_l2_i0_offset;
    wire [15:0] offset_param_l2_i1_offset;
    wire [15:0] offset_param_l2_i2_offset;
    wire [15:0] offset_param_l2_i3_offset;
    wire [15:0] offset_param_l2_i4_offset;
    wire [15:0] offset_param_l2_i5_offset;
    wire [15:0] offset_param_l2_i6_offset;
    wire [15:0] offset_param_l2_i7_offset;
    wire [15:0] offset_param_l2_i8_offset;
    wire [15:0] offset_param_l2_i9_offset;
    wire [15:0] offset_param_l2_i10_offset;
    wire [15:0] offset_param_l2_i11_offset;
    wire [15:0] offset_param_l2_i12_offset;
    wire [15:0] offset_param_l2_i13_offset;
    wire [15:0] offset_param_l2_i14_offset;
    wire [15:0] offset_param_l2_i15_offset;
    wire [15:0] offset_param_l2_i16_offset;
    wire [19:0] pi_param_l2_i0_pi;
    wire [19:0] pi_param_l2_i1_pi;
    wire [19:0] pi_param_l2_i2_pi;
    wire [19:0] pi_param_l2_i3_pi;
    wire [19:0] pi_param_l2_i4_pi;
    wire [19:0] pi_param_l2_i5_pi;
    wire [19:0] pi_param_l2_i6_pi;
    wire [19:0] pi_param_l2_i7_pi;
    wire [19:0] pi_param_l2_i8_pi;
    wire [19:0] pi_param_l2_i9_pi;
    wire [19:0] pi_param_l2_i10_pi;
    wire [19:0] pi_param_l2_i11_pi;
    wire [19:0] pi_param_l2_i12_pi;
    wire [19:0] pi_param_l2_i13_pi;
    wire [19:0] pi_param_l2_i14_pi;
    wire [19:0] pi_param_l2_i15_pi;
    wire [15:0] vref_param_l2_i0_vref;
    wire [15:0] vref_param_l2_i1_vref;
    wire [15:0] vref_param_l2_i2_vref;
    wire [15:0] vref_param_l2_i3_vref;
    wire [15:0] vref_param_l2_i4_vref;
    wire [15:0] vref_param_l2_i5_vref;
    wire [15:0] vref_param_l2_i6_vref;
    wire [15:0] vref_param_l2_i7_vref;
    wire [15:0] vref_param_l2_i8_vref;
    wire [15:0] vref_param_l2_i9_vref;
    wire [15:0] vref_param_l2_i10_vref;
    wire [15:0] vref_param_l2_i11_vref;
    wire [15:0] vref_param_l2_i12_vref;
    wire [15:0] vref_param_l2_i13_vref;
    wire [15:0] vref_param_l2_i14_vref;
    wire [15:0] vref_param_l2_i15_vref;
    wire [11:0] dcdl_param_l2_i0_dcdl;
    wire [11:0] dcdl_param_l2_i1_dcdl;
    wire [11:0] dcdl_param_l2_i2_dcdl;
    wire [11:0] dcdl_param_l2_i3_dcdl;
    wire [11:0] dcdl_param_l2_i4_dcdl;
    wire [11:0] dcdl_param_l2_i5_dcdl;
    wire [11:0] dcdl_param_l2_i6_dcdl;
    wire [11:0] dcdl_param_l2_i7_dcdl;
    wire [11:0] dcdl_param_l2_i8_dcdl;
    wire [11:0] dcdl_param_l2_i9_dcdl;
    wire [11:0] dcdl_param_l2_i10_dcdl;
    wire [11:0] dcdl_param_l2_i11_dcdl;
    wire [11:0] dcdl_param_l2_i12_dcdl;
    wire [11:0] dcdl_param_l2_i13_dcdl;
    wire [11:0] dcdl_param_l2_i14_dcdl;
    wire [11:0] dcdl_param_l2_i15_dcdl;
    wire [11:0] dcdl_param_l2_i16_dcdl;
    // MDE-CHAIN-1008 lane3 train 参数 wire（2026-10-12 接口贯通）
    wire [15:0] offset_param_l3_i0_offset;
    wire [15:0] offset_param_l3_i1_offset;
    wire [15:0] offset_param_l3_i2_offset;
    wire [15:0] offset_param_l3_i3_offset;
    wire [15:0] offset_param_l3_i4_offset;
    wire [15:0] offset_param_l3_i5_offset;
    wire [15:0] offset_param_l3_i6_offset;
    wire [15:0] offset_param_l3_i7_offset;
    wire [15:0] offset_param_l3_i8_offset;
    wire [15:0] offset_param_l3_i9_offset;
    wire [15:0] offset_param_l3_i10_offset;
    wire [15:0] offset_param_l3_i11_offset;
    wire [15:0] offset_param_l3_i12_offset;
    wire [15:0] offset_param_l3_i13_offset;
    wire [15:0] offset_param_l3_i14_offset;
    wire [15:0] offset_param_l3_i15_offset;
    wire [15:0] offset_param_l3_i16_offset;
    wire [19:0] pi_param_l3_i0_pi;
    wire [19:0] pi_param_l3_i1_pi;
    wire [19:0] pi_param_l3_i2_pi;
    wire [19:0] pi_param_l3_i3_pi;
    wire [19:0] pi_param_l3_i4_pi;
    wire [19:0] pi_param_l3_i5_pi;
    wire [19:0] pi_param_l3_i6_pi;
    wire [19:0] pi_param_l3_i7_pi;
    wire [19:0] pi_param_l3_i8_pi;
    wire [19:0] pi_param_l3_i9_pi;
    wire [19:0] pi_param_l3_i10_pi;
    wire [19:0] pi_param_l3_i11_pi;
    wire [19:0] pi_param_l3_i12_pi;
    wire [19:0] pi_param_l3_i13_pi;
    wire [19:0] pi_param_l3_i14_pi;
    wire [19:0] pi_param_l3_i15_pi;
    wire [15:0] vref_param_l3_i0_vref;
    wire [15:0] vref_param_l3_i1_vref;
    wire [15:0] vref_param_l3_i2_vref;
    wire [15:0] vref_param_l3_i3_vref;
    wire [15:0] vref_param_l3_i4_vref;
    wire [15:0] vref_param_l3_i5_vref;
    wire [15:0] vref_param_l3_i6_vref;
    wire [15:0] vref_param_l3_i7_vref;
    wire [15:0] vref_param_l3_i8_vref;
    wire [15:0] vref_param_l3_i9_vref;
    wire [15:0] vref_param_l3_i10_vref;
    wire [15:0] vref_param_l3_i11_vref;
    wire [15:0] vref_param_l3_i12_vref;
    wire [15:0] vref_param_l3_i13_vref;
    wire [15:0] vref_param_l3_i14_vref;
    wire [15:0] vref_param_l3_i15_vref;
    wire [11:0] dcdl_param_l3_i0_dcdl;
    wire [11:0] dcdl_param_l3_i1_dcdl;
    wire [11:0] dcdl_param_l3_i2_dcdl;
    wire [11:0] dcdl_param_l3_i3_dcdl;
    wire [11:0] dcdl_param_l3_i4_dcdl;
    wire [11:0] dcdl_param_l3_i5_dcdl;
    wire [11:0] dcdl_param_l3_i6_dcdl;
    wire [11:0] dcdl_param_l3_i7_dcdl;
    wire [11:0] dcdl_param_l3_i8_dcdl;
    wire [11:0] dcdl_param_l3_i9_dcdl;
    wire [11:0] dcdl_param_l3_i10_dcdl;
    wire [11:0] dcdl_param_l3_i11_dcdl;
    wire [11:0] dcdl_param_l3_i12_dcdl;
    wire [11:0] dcdl_param_l3_i13_dcdl;
    wire [11:0] dcdl_param_l3_i14_dcdl;
    wire [11:0] dcdl_param_l3_i15_dcdl;
    wire [11:0] dcdl_param_l3_i16_dcdl;
    // 族 c #3/#4/#7 单点输出（MDE-271-IMPL; apb_csr_top 同名输出）
    wire            train_param_src_o;
    wire [7:0]      datacheck_cfg_fail_threshold_o;
    wire [3:0]      eq_param_ctle_tap_o;

    // 族 c #3/#4/#7 同名输出直连（train_param_src_o / datacheck_cfg_fail_threshold_o / eq_param_ctle_tap_o）
    assign cfg_train_param_src           = train_param_src_o;
    assign cfg_fail_threshold            = datacheck_cfg_fail_threshold_o;
    assign cfg_eq_ctle_tap               = eq_param_ctle_tap_o;

    // 族 a lane0 per-IO 打包（i{io} → [{io}*W +: W]）
    assign cfg_offset_param_l0[ 0*16 +: 16] = offset_param_l0_i0_offset;
    assign cfg_offset_param_l0[ 1*16 +: 16] = offset_param_l0_i1_offset;
    assign cfg_offset_param_l0[ 2*16 +: 16] = offset_param_l0_i2_offset;
    assign cfg_offset_param_l0[ 3*16 +: 16] = offset_param_l0_i3_offset;
    assign cfg_offset_param_l0[ 4*16 +: 16] = offset_param_l0_i4_offset;
    assign cfg_offset_param_l0[ 5*16 +: 16] = offset_param_l0_i5_offset;
    assign cfg_offset_param_l0[ 6*16 +: 16] = offset_param_l0_i6_offset;
    assign cfg_offset_param_l0[ 7*16 +: 16] = offset_param_l0_i7_offset;
    assign cfg_offset_param_l0[ 8*16 +: 16] = offset_param_l0_i8_offset;
    assign cfg_offset_param_l0[ 9*16 +: 16] = offset_param_l0_i9_offset;
    assign cfg_offset_param_l0[10*16 +: 16] = offset_param_l0_i10_offset;
    assign cfg_offset_param_l0[11*16 +: 16] = offset_param_l0_i11_offset;
    assign cfg_offset_param_l0[12*16 +: 16] = offset_param_l0_i12_offset;
    assign cfg_offset_param_l0[13*16 +: 16] = offset_param_l0_i13_offset;
    assign cfg_offset_param_l0[14*16 +: 16] = offset_param_l0_i14_offset;
    assign cfg_offset_param_l0[15*16 +: 16] = offset_param_l0_i15_offset;
    assign cfg_vref_param_l0[ 0*16 +: 16] = vref_param_l0_i0_vref;
    assign cfg_vref_param_l0[ 1*16 +: 16] = vref_param_l0_i1_vref;
    assign cfg_vref_param_l0[ 2*16 +: 16] = vref_param_l0_i2_vref;
    assign cfg_vref_param_l0[ 3*16 +: 16] = vref_param_l0_i3_vref;
    assign cfg_vref_param_l0[ 4*16 +: 16] = vref_param_l0_i4_vref;
    assign cfg_vref_param_l0[ 5*16 +: 16] = vref_param_l0_i5_vref;
    assign cfg_vref_param_l0[ 6*16 +: 16] = vref_param_l0_i6_vref;
    assign cfg_vref_param_l0[ 7*16 +: 16] = vref_param_l0_i7_vref;
    assign cfg_vref_param_l0[ 8*16 +: 16] = vref_param_l0_i8_vref;
    assign cfg_vref_param_l0[ 9*16 +: 16] = vref_param_l0_i9_vref;
    assign cfg_vref_param_l0[10*16 +: 16] = vref_param_l0_i10_vref;
    assign cfg_vref_param_l0[11*16 +: 16] = vref_param_l0_i11_vref;
    assign cfg_vref_param_l0[12*16 +: 16] = vref_param_l0_i12_vref;
    assign cfg_vref_param_l0[13*16 +: 16] = vref_param_l0_i13_vref;
    assign cfg_vref_param_l0[14*16 +: 16] = vref_param_l0_i14_vref;
    assign cfg_vref_param_l0[15*16 +: 16] = vref_param_l0_i15_vref;
    assign cfg_pi_param_l0[ 0*20 +: 20] = pi_param_l0_i0_pi;
    assign cfg_pi_param_l0[ 1*20 +: 20] = pi_param_l0_i1_pi;
    assign cfg_pi_param_l0[ 2*20 +: 20] = pi_param_l0_i2_pi;
    assign cfg_pi_param_l0[ 3*20 +: 20] = pi_param_l0_i3_pi;
    assign cfg_pi_param_l0[ 4*20 +: 20] = pi_param_l0_i4_pi;
    assign cfg_pi_param_l0[ 5*20 +: 20] = pi_param_l0_i5_pi;
    assign cfg_pi_param_l0[ 6*20 +: 20] = pi_param_l0_i6_pi;
    assign cfg_pi_param_l0[ 7*20 +: 20] = pi_param_l0_i7_pi;
    assign cfg_pi_param_l0[ 8*20 +: 20] = pi_param_l0_i8_pi;
    assign cfg_pi_param_l0[ 9*20 +: 20] = pi_param_l0_i9_pi;
    assign cfg_pi_param_l0[10*20 +: 20] = pi_param_l0_i10_pi;
    assign cfg_pi_param_l0[11*20 +: 20] = pi_param_l0_i11_pi;
    assign cfg_pi_param_l0[12*20 +: 20] = pi_param_l0_i12_pi;
    assign cfg_pi_param_l0[13*20 +: 20] = pi_param_l0_i13_pi;
    assign cfg_pi_param_l0[14*20 +: 20] = pi_param_l0_i14_pi;
    assign cfg_pi_param_l0[15*20 +: 20] = pi_param_l0_i15_pi;
    assign cfg_dcdl_param_l0[ 0*12 +: 12] = dcdl_param_l0_i0_dcdl;
    assign cfg_dcdl_param_l0[ 1*12 +: 12] = dcdl_param_l0_i1_dcdl;
    assign cfg_dcdl_param_l0[ 2*12 +: 12] = dcdl_param_l0_i2_dcdl;
    assign cfg_dcdl_param_l0[ 3*12 +: 12] = dcdl_param_l0_i3_dcdl;
    assign cfg_dcdl_param_l0[ 4*12 +: 12] = dcdl_param_l0_i4_dcdl;
    assign cfg_dcdl_param_l0[ 5*12 +: 12] = dcdl_param_l0_i5_dcdl;
    assign cfg_dcdl_param_l0[ 6*12 +: 12] = dcdl_param_l0_i6_dcdl;
    assign cfg_dcdl_param_l0[ 7*12 +: 12] = dcdl_param_l0_i7_dcdl;
    assign cfg_dcdl_param_l0[ 8*12 +: 12] = dcdl_param_l0_i8_dcdl;
    assign cfg_dcdl_param_l0[ 9*12 +: 12] = dcdl_param_l0_i9_dcdl;
    assign cfg_dcdl_param_l0[10*12 +: 12] = dcdl_param_l0_i10_dcdl;
    assign cfg_dcdl_param_l0[11*12 +: 12] = dcdl_param_l0_i11_dcdl;
    assign cfg_dcdl_param_l0[12*12 +: 12] = dcdl_param_l0_i12_dcdl;
    assign cfg_dcdl_param_l0[13*12 +: 12] = dcdl_param_l0_i13_dcdl;
    assign cfg_dcdl_param_l0[14*12 +: 12] = dcdl_param_l0_i14_dcdl;
    assign cfg_dcdl_param_l0[15*12 +: 12] = dcdl_param_l0_i15_dcdl;
    // i16（RX16 冗余 IO）—— WP1 P0-A 新增
    assign cfg_offset_param_l0[16*16 +: 16] = offset_param_l0_i16_offset;
    assign cfg_dcdl_param_l0[16*12 +: 12] = dcdl_param_l0_i16_dcdl;
    // MDE-CHAIN-1008 lane1 打包 (2026-10-12)
    assign cfg_offset_param_l1[ 0*16 +: 16] = offset_param_l1_i0_offset;
    assign cfg_offset_param_l1[ 1*16 +: 16] = offset_param_l1_i1_offset;
    assign cfg_offset_param_l1[ 2*16 +: 16] = offset_param_l1_i2_offset;
    assign cfg_offset_param_l1[ 3*16 +: 16] = offset_param_l1_i3_offset;
    assign cfg_offset_param_l1[ 4*16 +: 16] = offset_param_l1_i4_offset;
    assign cfg_offset_param_l1[ 5*16 +: 16] = offset_param_l1_i5_offset;
    assign cfg_offset_param_l1[ 6*16 +: 16] = offset_param_l1_i6_offset;
    assign cfg_offset_param_l1[ 7*16 +: 16] = offset_param_l1_i7_offset;
    assign cfg_offset_param_l1[ 8*16 +: 16] = offset_param_l1_i8_offset;
    assign cfg_offset_param_l1[ 9*16 +: 16] = offset_param_l1_i9_offset;
    assign cfg_offset_param_l1[10*16 +: 16] = offset_param_l1_i10_offset;
    assign cfg_offset_param_l1[11*16 +: 16] = offset_param_l1_i11_offset;
    assign cfg_offset_param_l1[12*16 +: 16] = offset_param_l1_i12_offset;
    assign cfg_offset_param_l1[13*16 +: 16] = offset_param_l1_i13_offset;
    assign cfg_offset_param_l1[14*16 +: 16] = offset_param_l1_i14_offset;
    assign cfg_offset_param_l1[15*16 +: 16] = offset_param_l1_i15_offset;
    assign cfg_offset_param_l1[16*16 +: 16] = offset_param_l1_i16_offset;
    assign cfg_vref_param_l1[ 0*16 +: 16] = vref_param_l1_i0_vref;
    assign cfg_vref_param_l1[ 1*16 +: 16] = vref_param_l1_i1_vref;
    assign cfg_vref_param_l1[ 2*16 +: 16] = vref_param_l1_i2_vref;
    assign cfg_vref_param_l1[ 3*16 +: 16] = vref_param_l1_i3_vref;
    assign cfg_vref_param_l1[ 4*16 +: 16] = vref_param_l1_i4_vref;
    assign cfg_vref_param_l1[ 5*16 +: 16] = vref_param_l1_i5_vref;
    assign cfg_vref_param_l1[ 6*16 +: 16] = vref_param_l1_i6_vref;
    assign cfg_vref_param_l1[ 7*16 +: 16] = vref_param_l1_i7_vref;
    assign cfg_vref_param_l1[ 8*16 +: 16] = vref_param_l1_i8_vref;
    assign cfg_vref_param_l1[ 9*16 +: 16] = vref_param_l1_i9_vref;
    assign cfg_vref_param_l1[10*16 +: 16] = vref_param_l1_i10_vref;
    assign cfg_vref_param_l1[11*16 +: 16] = vref_param_l1_i11_vref;
    assign cfg_vref_param_l1[12*16 +: 16] = vref_param_l1_i12_vref;
    assign cfg_vref_param_l1[13*16 +: 16] = vref_param_l1_i13_vref;
    assign cfg_vref_param_l1[14*16 +: 16] = vref_param_l1_i14_vref;
    assign cfg_vref_param_l1[15*16 +: 16] = vref_param_l1_i15_vref;
    assign cfg_pi_param_l1[ 0*20 +: 20] = pi_param_l1_i0_pi;
    assign cfg_pi_param_l1[ 1*20 +: 20] = pi_param_l1_i1_pi;
    assign cfg_pi_param_l1[ 2*20 +: 20] = pi_param_l1_i2_pi;
    assign cfg_pi_param_l1[ 3*20 +: 20] = pi_param_l1_i3_pi;
    assign cfg_pi_param_l1[ 4*20 +: 20] = pi_param_l1_i4_pi;
    assign cfg_pi_param_l1[ 5*20 +: 20] = pi_param_l1_i5_pi;
    assign cfg_pi_param_l1[ 6*20 +: 20] = pi_param_l1_i6_pi;
    assign cfg_pi_param_l1[ 7*20 +: 20] = pi_param_l1_i7_pi;
    assign cfg_pi_param_l1[ 8*20 +: 20] = pi_param_l1_i8_pi;
    assign cfg_pi_param_l1[ 9*20 +: 20] = pi_param_l1_i9_pi;
    assign cfg_pi_param_l1[10*20 +: 20] = pi_param_l1_i10_pi;
    assign cfg_pi_param_l1[11*20 +: 20] = pi_param_l1_i11_pi;
    assign cfg_pi_param_l1[12*20 +: 20] = pi_param_l1_i12_pi;
    assign cfg_pi_param_l1[13*20 +: 20] = pi_param_l1_i13_pi;
    assign cfg_pi_param_l1[14*20 +: 20] = pi_param_l1_i14_pi;
    assign cfg_pi_param_l1[15*20 +: 20] = pi_param_l1_i15_pi;
    assign cfg_dcdl_param_l1[ 0*12 +: 12] = dcdl_param_l1_i0_dcdl;
    assign cfg_dcdl_param_l1[ 1*12 +: 12] = dcdl_param_l1_i1_dcdl;
    assign cfg_dcdl_param_l1[ 2*12 +: 12] = dcdl_param_l1_i2_dcdl;
    assign cfg_dcdl_param_l1[ 3*12 +: 12] = dcdl_param_l1_i3_dcdl;
    assign cfg_dcdl_param_l1[ 4*12 +: 12] = dcdl_param_l1_i4_dcdl;
    assign cfg_dcdl_param_l1[ 5*12 +: 12] = dcdl_param_l1_i5_dcdl;
    assign cfg_dcdl_param_l1[ 6*12 +: 12] = dcdl_param_l1_i6_dcdl;
    assign cfg_dcdl_param_l1[ 7*12 +: 12] = dcdl_param_l1_i7_dcdl;
    assign cfg_dcdl_param_l1[ 8*12 +: 12] = dcdl_param_l1_i8_dcdl;
    assign cfg_dcdl_param_l1[ 9*12 +: 12] = dcdl_param_l1_i9_dcdl;
    assign cfg_dcdl_param_l1[10*12 +: 12] = dcdl_param_l1_i10_dcdl;
    assign cfg_dcdl_param_l1[11*12 +: 12] = dcdl_param_l1_i11_dcdl;
    assign cfg_dcdl_param_l1[12*12 +: 12] = dcdl_param_l1_i12_dcdl;
    assign cfg_dcdl_param_l1[13*12 +: 12] = dcdl_param_l1_i13_dcdl;
    assign cfg_dcdl_param_l1[14*12 +: 12] = dcdl_param_l1_i14_dcdl;
    assign cfg_dcdl_param_l1[15*12 +: 12] = dcdl_param_l1_i15_dcdl;
    assign cfg_dcdl_param_l1[16*12 +: 12] = dcdl_param_l1_i16_dcdl;
    // MDE-CHAIN-1008 lane2 打包 (2026-10-12)
    assign cfg_offset_param_l2[ 0*16 +: 16] = offset_param_l2_i0_offset;
    assign cfg_offset_param_l2[ 1*16 +: 16] = offset_param_l2_i1_offset;
    assign cfg_offset_param_l2[ 2*16 +: 16] = offset_param_l2_i2_offset;
    assign cfg_offset_param_l2[ 3*16 +: 16] = offset_param_l2_i3_offset;
    assign cfg_offset_param_l2[ 4*16 +: 16] = offset_param_l2_i4_offset;
    assign cfg_offset_param_l2[ 5*16 +: 16] = offset_param_l2_i5_offset;
    assign cfg_offset_param_l2[ 6*16 +: 16] = offset_param_l2_i6_offset;
    assign cfg_offset_param_l2[ 7*16 +: 16] = offset_param_l2_i7_offset;
    assign cfg_offset_param_l2[ 8*16 +: 16] = offset_param_l2_i8_offset;
    assign cfg_offset_param_l2[ 9*16 +: 16] = offset_param_l2_i9_offset;
    assign cfg_offset_param_l2[10*16 +: 16] = offset_param_l2_i10_offset;
    assign cfg_offset_param_l2[11*16 +: 16] = offset_param_l2_i11_offset;
    assign cfg_offset_param_l2[12*16 +: 16] = offset_param_l2_i12_offset;
    assign cfg_offset_param_l2[13*16 +: 16] = offset_param_l2_i13_offset;
    assign cfg_offset_param_l2[14*16 +: 16] = offset_param_l2_i14_offset;
    assign cfg_offset_param_l2[15*16 +: 16] = offset_param_l2_i15_offset;
    assign cfg_offset_param_l2[16*16 +: 16] = offset_param_l2_i16_offset;
    assign cfg_vref_param_l2[ 0*16 +: 16] = vref_param_l2_i0_vref;
    assign cfg_vref_param_l2[ 1*16 +: 16] = vref_param_l2_i1_vref;
    assign cfg_vref_param_l2[ 2*16 +: 16] = vref_param_l2_i2_vref;
    assign cfg_vref_param_l2[ 3*16 +: 16] = vref_param_l2_i3_vref;
    assign cfg_vref_param_l2[ 4*16 +: 16] = vref_param_l2_i4_vref;
    assign cfg_vref_param_l2[ 5*16 +: 16] = vref_param_l2_i5_vref;
    assign cfg_vref_param_l2[ 6*16 +: 16] = vref_param_l2_i6_vref;
    assign cfg_vref_param_l2[ 7*16 +: 16] = vref_param_l2_i7_vref;
    assign cfg_vref_param_l2[ 8*16 +: 16] = vref_param_l2_i8_vref;
    assign cfg_vref_param_l2[ 9*16 +: 16] = vref_param_l2_i9_vref;
    assign cfg_vref_param_l2[10*16 +: 16] = vref_param_l2_i10_vref;
    assign cfg_vref_param_l2[11*16 +: 16] = vref_param_l2_i11_vref;
    assign cfg_vref_param_l2[12*16 +: 16] = vref_param_l2_i12_vref;
    assign cfg_vref_param_l2[13*16 +: 16] = vref_param_l2_i13_vref;
    assign cfg_vref_param_l2[14*16 +: 16] = vref_param_l2_i14_vref;
    assign cfg_vref_param_l2[15*16 +: 16] = vref_param_l2_i15_vref;
    assign cfg_pi_param_l2[ 0*20 +: 20] = pi_param_l2_i0_pi;
    assign cfg_pi_param_l2[ 1*20 +: 20] = pi_param_l2_i1_pi;
    assign cfg_pi_param_l2[ 2*20 +: 20] = pi_param_l2_i2_pi;
    assign cfg_pi_param_l2[ 3*20 +: 20] = pi_param_l2_i3_pi;
    assign cfg_pi_param_l2[ 4*20 +: 20] = pi_param_l2_i4_pi;
    assign cfg_pi_param_l2[ 5*20 +: 20] = pi_param_l2_i5_pi;
    assign cfg_pi_param_l2[ 6*20 +: 20] = pi_param_l2_i6_pi;
    assign cfg_pi_param_l2[ 7*20 +: 20] = pi_param_l2_i7_pi;
    assign cfg_pi_param_l2[ 8*20 +: 20] = pi_param_l2_i8_pi;
    assign cfg_pi_param_l2[ 9*20 +: 20] = pi_param_l2_i9_pi;
    assign cfg_pi_param_l2[10*20 +: 20] = pi_param_l2_i10_pi;
    assign cfg_pi_param_l2[11*20 +: 20] = pi_param_l2_i11_pi;
    assign cfg_pi_param_l2[12*20 +: 20] = pi_param_l2_i12_pi;
    assign cfg_pi_param_l2[13*20 +: 20] = pi_param_l2_i13_pi;
    assign cfg_pi_param_l2[14*20 +: 20] = pi_param_l2_i14_pi;
    assign cfg_pi_param_l2[15*20 +: 20] = pi_param_l2_i15_pi;
    assign cfg_dcdl_param_l2[ 0*12 +: 12] = dcdl_param_l2_i0_dcdl;
    assign cfg_dcdl_param_l2[ 1*12 +: 12] = dcdl_param_l2_i1_dcdl;
    assign cfg_dcdl_param_l2[ 2*12 +: 12] = dcdl_param_l2_i2_dcdl;
    assign cfg_dcdl_param_l2[ 3*12 +: 12] = dcdl_param_l2_i3_dcdl;
    assign cfg_dcdl_param_l2[ 4*12 +: 12] = dcdl_param_l2_i4_dcdl;
    assign cfg_dcdl_param_l2[ 5*12 +: 12] = dcdl_param_l2_i5_dcdl;
    assign cfg_dcdl_param_l2[ 6*12 +: 12] = dcdl_param_l2_i6_dcdl;
    assign cfg_dcdl_param_l2[ 7*12 +: 12] = dcdl_param_l2_i7_dcdl;
    assign cfg_dcdl_param_l2[ 8*12 +: 12] = dcdl_param_l2_i8_dcdl;
    assign cfg_dcdl_param_l2[ 9*12 +: 12] = dcdl_param_l2_i9_dcdl;
    assign cfg_dcdl_param_l2[10*12 +: 12] = dcdl_param_l2_i10_dcdl;
    assign cfg_dcdl_param_l2[11*12 +: 12] = dcdl_param_l2_i11_dcdl;
    assign cfg_dcdl_param_l2[12*12 +: 12] = dcdl_param_l2_i12_dcdl;
    assign cfg_dcdl_param_l2[13*12 +: 12] = dcdl_param_l2_i13_dcdl;
    assign cfg_dcdl_param_l2[14*12 +: 12] = dcdl_param_l2_i14_dcdl;
    assign cfg_dcdl_param_l2[15*12 +: 12] = dcdl_param_l2_i15_dcdl;
    assign cfg_dcdl_param_l2[16*12 +: 12] = dcdl_param_l2_i16_dcdl;
    // MDE-CHAIN-1008 lane3 打包 (2026-10-12)
    assign cfg_offset_param_l3[ 0*16 +: 16] = offset_param_l3_i0_offset;
    assign cfg_offset_param_l3[ 1*16 +: 16] = offset_param_l3_i1_offset;
    assign cfg_offset_param_l3[ 2*16 +: 16] = offset_param_l3_i2_offset;
    assign cfg_offset_param_l3[ 3*16 +: 16] = offset_param_l3_i3_offset;
    assign cfg_offset_param_l3[ 4*16 +: 16] = offset_param_l3_i4_offset;
    assign cfg_offset_param_l3[ 5*16 +: 16] = offset_param_l3_i5_offset;
    assign cfg_offset_param_l3[ 6*16 +: 16] = offset_param_l3_i6_offset;
    assign cfg_offset_param_l3[ 7*16 +: 16] = offset_param_l3_i7_offset;
    assign cfg_offset_param_l3[ 8*16 +: 16] = offset_param_l3_i8_offset;
    assign cfg_offset_param_l3[ 9*16 +: 16] = offset_param_l3_i9_offset;
    assign cfg_offset_param_l3[10*16 +: 16] = offset_param_l3_i10_offset;
    assign cfg_offset_param_l3[11*16 +: 16] = offset_param_l3_i11_offset;
    assign cfg_offset_param_l3[12*16 +: 16] = offset_param_l3_i12_offset;
    assign cfg_offset_param_l3[13*16 +: 16] = offset_param_l3_i13_offset;
    assign cfg_offset_param_l3[14*16 +: 16] = offset_param_l3_i14_offset;
    assign cfg_offset_param_l3[15*16 +: 16] = offset_param_l3_i15_offset;
    assign cfg_offset_param_l3[16*16 +: 16] = offset_param_l3_i16_offset;
    assign cfg_vref_param_l3[ 0*16 +: 16] = vref_param_l3_i0_vref;
    assign cfg_vref_param_l3[ 1*16 +: 16] = vref_param_l3_i1_vref;
    assign cfg_vref_param_l3[ 2*16 +: 16] = vref_param_l3_i2_vref;
    assign cfg_vref_param_l3[ 3*16 +: 16] = vref_param_l3_i3_vref;
    assign cfg_vref_param_l3[ 4*16 +: 16] = vref_param_l3_i4_vref;
    assign cfg_vref_param_l3[ 5*16 +: 16] = vref_param_l3_i5_vref;
    assign cfg_vref_param_l3[ 6*16 +: 16] = vref_param_l3_i6_vref;
    assign cfg_vref_param_l3[ 7*16 +: 16] = vref_param_l3_i7_vref;
    assign cfg_vref_param_l3[ 8*16 +: 16] = vref_param_l3_i8_vref;
    assign cfg_vref_param_l3[ 9*16 +: 16] = vref_param_l3_i9_vref;
    assign cfg_vref_param_l3[10*16 +: 16] = vref_param_l3_i10_vref;
    assign cfg_vref_param_l3[11*16 +: 16] = vref_param_l3_i11_vref;
    assign cfg_vref_param_l3[12*16 +: 16] = vref_param_l3_i12_vref;
    assign cfg_vref_param_l3[13*16 +: 16] = vref_param_l3_i13_vref;
    assign cfg_vref_param_l3[14*16 +: 16] = vref_param_l3_i14_vref;
    assign cfg_vref_param_l3[15*16 +: 16] = vref_param_l3_i15_vref;
    assign cfg_pi_param_l3[ 0*20 +: 20] = pi_param_l3_i0_pi;
    assign cfg_pi_param_l3[ 1*20 +: 20] = pi_param_l3_i1_pi;
    assign cfg_pi_param_l3[ 2*20 +: 20] = pi_param_l3_i2_pi;
    assign cfg_pi_param_l3[ 3*20 +: 20] = pi_param_l3_i3_pi;
    assign cfg_pi_param_l3[ 4*20 +: 20] = pi_param_l3_i4_pi;
    assign cfg_pi_param_l3[ 5*20 +: 20] = pi_param_l3_i5_pi;
    assign cfg_pi_param_l3[ 6*20 +: 20] = pi_param_l3_i6_pi;
    assign cfg_pi_param_l3[ 7*20 +: 20] = pi_param_l3_i7_pi;
    assign cfg_pi_param_l3[ 8*20 +: 20] = pi_param_l3_i8_pi;
    assign cfg_pi_param_l3[ 9*20 +: 20] = pi_param_l3_i9_pi;
    assign cfg_pi_param_l3[10*20 +: 20] = pi_param_l3_i10_pi;
    assign cfg_pi_param_l3[11*20 +: 20] = pi_param_l3_i11_pi;
    assign cfg_pi_param_l3[12*20 +: 20] = pi_param_l3_i12_pi;
    assign cfg_pi_param_l3[13*20 +: 20] = pi_param_l3_i13_pi;
    assign cfg_pi_param_l3[14*20 +: 20] = pi_param_l3_i14_pi;
    assign cfg_pi_param_l3[15*20 +: 20] = pi_param_l3_i15_pi;
    assign cfg_dcdl_param_l3[ 0*12 +: 12] = dcdl_param_l3_i0_dcdl;
    assign cfg_dcdl_param_l3[ 1*12 +: 12] = dcdl_param_l3_i1_dcdl;
    assign cfg_dcdl_param_l3[ 2*12 +: 12] = dcdl_param_l3_i2_dcdl;
    assign cfg_dcdl_param_l3[ 3*12 +: 12] = dcdl_param_l3_i3_dcdl;
    assign cfg_dcdl_param_l3[ 4*12 +: 12] = dcdl_param_l3_i4_dcdl;
    assign cfg_dcdl_param_l3[ 5*12 +: 12] = dcdl_param_l3_i5_dcdl;
    assign cfg_dcdl_param_l3[ 6*12 +: 12] = dcdl_param_l3_i6_dcdl;
    assign cfg_dcdl_param_l3[ 7*12 +: 12] = dcdl_param_l3_i7_dcdl;
    assign cfg_dcdl_param_l3[ 8*12 +: 12] = dcdl_param_l3_i8_dcdl;
    assign cfg_dcdl_param_l3[ 9*12 +: 12] = dcdl_param_l3_i9_dcdl;
    assign cfg_dcdl_param_l3[10*12 +: 12] = dcdl_param_l3_i10_dcdl;
    assign cfg_dcdl_param_l3[11*12 +: 12] = dcdl_param_l3_i11_dcdl;
    assign cfg_dcdl_param_l3[12*12 +: 12] = dcdl_param_l3_i12_dcdl;
    assign cfg_dcdl_param_l3[13*12 +: 12] = dcdl_param_l3_i13_dcdl;
    assign cfg_dcdl_param_l3[14*12 +: 12] = dcdl_param_l3_i14_dcdl;
    assign cfg_dcdl_param_l3[15*12 +: 12] = dcdl_param_l3_i15_dcdl;
    assign cfg_dcdl_param_l3[16*12 +: 12] = dcdl_param_l3_i16_dcdl;

    // --------------------------------------------------------------------------
    // apb_csr_top：集中式 CSR（APB 唯一接入点；具名配置直送 lane_top / sideband /
    // intc / mcu_subsys，跨域同步由各消费侧完成）
    // --------------------------------------------------------------------------
    apb_csr_top #(
        .LANE_NUM        (LANE_NUM),
        .IO_NUM          (IO_NUM),
        .SBC_LANE_EN_RST (SB_LANE_EN_RST)
    ) u_apb_csr_top (
        .apb_pclk               (apb_pclk),
        .apb_preset_n           (apb_preset_n),
        // APB slave ← phy_apb_*（MCU on: mcu master0 / off: hipi_apb MUX，见 APB 配置路由段）
        .apb_psel               (phy_apb_psel),
        .apb_penable            (phy_apb_penable),
        .apb_pwrite             (phy_apb_pwrite),
        .apb_paddr              (phy_apb_paddr),
        .apb_pwdata             (phy_apb_pwdata),
        .apb_prdata             (phy_apb_prdata),
        .apb_pslverr            (phy_apb_pslverr),
        .apb_pready             (phy_apb_pready),
        // bring-up strap
        .io_data_width_mode_strap (io_data_width_mode_strap),
        // 中断回流（sideband_irq_i[3:2] 无源，tie 0）
        .lane_top_irq_i         (apb_lane_irq),
        .sideband_irq_i         ({2'b00, sb_irq_err, sb_irq_pkt_arrival}),
        // SBC 状态回流
        .sbc_tx_buf_level_i     (sb_tx_buf_level),
        .sbc_tx_busy_i          (sb_tx_busy),
        .sbc_tx_err_i           (sb_tx_err),
        .sbc_rx_buf_level_i     (sb_rx_buf_level),
        .sbc_rx_busy_i          (sb_rx_busy),
        .sbc_rx_parity_err_i    (sb_rx_parity_err),
        .sbc_rx_gap_err_i       (sb_rx_gap_err),
        .sbc_rx_buf_ovf_i       (sb_rx_buf_ovf),
        .sbc_rx_buf_rdata_i     (sb_rx_buf_rdata),
        .sbc_pkt_arrival_flag_i (sb_pkt_arrival_flag),
        .sbc_err_flag_i         (sb_err_flag),
        .sbc_illegal_opcode_i   (sb_illegal_opcode),
        .sbc_illegal_lid_i      (sb_illegal_lid),
        // DFX / FIFO 状态回流（per-lane 8 bit × 4，见头部假设 12；ovf/unf 本阶段恒 0）
        .aligned_rx_i           (aligned_agg),
        .rx_fifo_status_i       (rx_fifo_status_agg),
        // 具名配置输出 → intc
        .int_status_o           (int_status),
        .int_mask_mcu_o         (int_mask_mcu),
        .int_mask_ext_o         (int_mask_ext),
        // 具名配置输出 → lane_top（广播）
        .io_data_width_mode_o   (cfg_io_data_width_mode),
        .reversal_en_tx_o       (cfg_reversal_en_tx),
        .reversal_en_rx_o       (cfg_reversal_en_rx),
        .repair_map_tx_o        (cfg_repair_map_tx),
        .repair_map_rx_o        (cfg_repair_map_rx),
        .msb_lsb_flip_en_tx_o   (cfg_msb_lsb_flip_en_tx),
        .msb_lsb_flip_en_rx_o   (cfg_msb_lsb_flip_en_rx),
        .prbs_seed_tx_o         (cfg_prbs_seed_tx),
        .prbs_seed_rx_o         (cfg_prbs_seed_rx),
        .flip_sel_tx_o          (cfg_flip_sel_tx),
        .flip_sel_rx_o          (cfg_flip_sel_rx),
        .scramble_en_o          (cfg_scramble_en),
        .scramble_en_tx_o       (cfg_scramble_en_tx),
        .seed_load_rx_o         (cfg_seed_load_rx),
        .clear_rx_o             (cfg_clear_rx),
        .seed_load_tx_o         (cfg_seed_load_tx),
        .clear_tx_o             (cfg_clear_tx),
        .reorder_delay_step_o   (cfg_reorder_delay_step),
        .train_mode_o           (cfg_train_mode),
        .force_mode_switch_o    (cfg_force_mode_switch),
        .bypass_step_o          (cfg_bypass_step),
        .timeout_step_en_o      (cfg_timeout_step_en),
        .timeout_threshold_o    (cfg_timeout_threshold),
        .data_check_pattern_sel_o (cfg_data_check_pattern_sel),
        .vref_pattern_tx_o      (cfg_vref_pattern_tx),
        .pi_pattern_tx_o        (cfg_pi_pattern_tx),
        .dcdl_pattern_tx_o      (cfg_dcdl_pattern_tx),
        .reorder_pattern_tx_o   (cfg_reorder_pattern_tx),
        .datacheck_pattern_tx_o (cfg_datacheck_pattern_tx),
        .step_trigger_o         (cfg_step_trigger),
        .step_force_end_o       (cfg_step_force_end),
        .step_cnt_o             (cfg_step_cnt),
        // cfg broadcast → mcu_subsys
        .cfg_mcu_o              (cfg_mcu),
        // 具名配置输出 → sideband
        .sbc_tx_enable_o        (cfg_sbc_tx_enable),
        .sbc_tx_manual_trigger_o(cfg_sbc_tx_manual_trigger),
        .sbc_tx_pkt_inject_o    (cfg_sbc_tx_pkt_inject),
        .sbc_rx_enable_o        (cfg_sbc_rx_enable),
        .sbc_rx_manual_read_o   (cfg_sbc_rx_manual_read),
        .sbc_rx_soft_rst_o      (cfg_sbc_rx_soft_rst),
        .sbc_rx_buf_depth_o     (cfg_sbc_rx_buf_depth),
        .sbc_lid_filter_o       (cfg_sbc_lid_filter),
        .sbc_opcode_filter_o    (cfg_sbc_opcode_filter),
        .sbc_err_mask_o         (cfg_sbc_err_mask),
        .sbc_pkt_arrival_mask_o (cfg_sbc_pkt_arrival_mask),
        .sbc_err_irq_mask_o     (cfg_sbc_err_irq_mask),
        .sbc_irq_clear_o        (cfg_sbc_irq_clear),
        .sbc_lane_enable_o      (cfg_sbc_lane_enable),
        .sbc_txr_en_o             (cfg_sbc_txr_en),
        .sbc_rxr_sel_o            (cfg_sbc_rxr_sel),
        .phy_ctrl_writein_l0_o  (cfg_phy_ctrl_writein_l0),
        .phy_trx_en_l0_o        (cfg_phy_trx_en_l0),
        .phy_ctrl_writein_l1_o  (cfg_phy_ctrl_writein_l1),
        .phy_trx_en_l1_o        (cfg_phy_trx_en_l1),
        .phy_ctrl_writein_l2_o  (cfg_phy_ctrl_writein_l2),
        .phy_trx_en_l2_o        (cfg_phy_trx_en_l2),
        .phy_ctrl_writein_l3_o  (cfg_phy_ctrl_writein_l3),
        .phy_trx_en_l3_o        (cfg_phy_trx_en_l3),
        .phy_rx_clk_eng_readout_l0_i (lane0_RX_CLK_ENG_REG_READ_OUT),
        .phy_tx_clk_eng_readout_l0_i (lane0_TX_CLK_ENG_REG_READ_OUT),
        .phy_connect_dig_l0_i    (lane0_CONNECT_DIG_OUT),
        .phy_err_edge_l0_i       (phy_err_edge_l0_i),
        .phy_rx_clk_eng_readout_l1_i (lane1_RX_CLK_ENG_REG_READ_OUT),
        .phy_tx_clk_eng_readout_l1_i (lane1_TX_CLK_ENG_REG_READ_OUT),
        .phy_connect_dig_l1_i    (lane1_CONNECT_DIG_OUT),
        .phy_err_edge_l1_i       (phy_err_edge_l1_i),
        .phy_rx_clk_eng_readout_l2_i (lane2_RX_CLK_ENG_REG_READ_OUT),
        .phy_tx_clk_eng_readout_l2_i (lane2_TX_CLK_ENG_REG_READ_OUT),
        .phy_connect_dig_l2_i    (lane2_CONNECT_DIG_OUT),
        .phy_err_edge_l2_i       (phy_err_edge_l2_i),
        .phy_rx_clk_eng_readout_l3_i (lane3_RX_CLK_ENG_REG_READ_OUT),
        .phy_tx_clk_eng_readout_l3_i (lane3_TX_CLK_ENG_REG_READ_OUT),
        .phy_connect_dig_l3_i    (lane3_CONNECT_DIG_OUT),
        .phy_err_edge_l3_i       (phy_err_edge_l3_i),
        .offset_param_l0_i0_offset        (offset_param_l0_i0_offset),
        .vref_param_l0_i0_vref          (vref_param_l0_i0_vref),
        .pi_param_l0_i0_pi              (pi_param_l0_i0_pi),
        .dcdl_param_l0_i0_dcdl          (dcdl_param_l0_i0_dcdl),
        .offset_param_l0_i1_offset        (offset_param_l0_i1_offset),
        .vref_param_l0_i1_vref          (vref_param_l0_i1_vref),
        .pi_param_l0_i1_pi              (pi_param_l0_i1_pi),
        .dcdl_param_l0_i1_dcdl          (dcdl_param_l0_i1_dcdl),
        .offset_param_l0_i2_offset        (offset_param_l0_i2_offset),
        .vref_param_l0_i2_vref          (vref_param_l0_i2_vref),
        .pi_param_l0_i2_pi              (pi_param_l0_i2_pi),
        .dcdl_param_l0_i2_dcdl          (dcdl_param_l0_i2_dcdl),
        .offset_param_l0_i3_offset        (offset_param_l0_i3_offset),
        .vref_param_l0_i3_vref          (vref_param_l0_i3_vref),
        .pi_param_l0_i3_pi              (pi_param_l0_i3_pi),
        .dcdl_param_l0_i3_dcdl          (dcdl_param_l0_i3_dcdl),
        .offset_param_l0_i4_offset        (offset_param_l0_i4_offset),
        .vref_param_l0_i4_vref          (vref_param_l0_i4_vref),
        .pi_param_l0_i4_pi              (pi_param_l0_i4_pi),
        .dcdl_param_l0_i4_dcdl          (dcdl_param_l0_i4_dcdl),
        .offset_param_l0_i5_offset        (offset_param_l0_i5_offset),
        .vref_param_l0_i5_vref          (vref_param_l0_i5_vref),
        .pi_param_l0_i5_pi              (pi_param_l0_i5_pi),
        .dcdl_param_l0_i5_dcdl          (dcdl_param_l0_i5_dcdl),
        .offset_param_l0_i6_offset        (offset_param_l0_i6_offset),
        .vref_param_l0_i6_vref          (vref_param_l0_i6_vref),
        .pi_param_l0_i6_pi              (pi_param_l0_i6_pi),
        .dcdl_param_l0_i6_dcdl          (dcdl_param_l0_i6_dcdl),
        .offset_param_l0_i7_offset        (offset_param_l0_i7_offset),
        .vref_param_l0_i7_vref          (vref_param_l0_i7_vref),
        .pi_param_l0_i7_pi              (pi_param_l0_i7_pi),
        .dcdl_param_l0_i7_dcdl          (dcdl_param_l0_i7_dcdl),
        .offset_param_l0_i8_offset        (offset_param_l0_i8_offset),
        .vref_param_l0_i8_vref          (vref_param_l0_i8_vref),
        .pi_param_l0_i8_pi              (pi_param_l0_i8_pi),
        .dcdl_param_l0_i8_dcdl          (dcdl_param_l0_i8_dcdl),
        .offset_param_l0_i9_offset        (offset_param_l0_i9_offset),
        .vref_param_l0_i9_vref          (vref_param_l0_i9_vref),
        .pi_param_l0_i9_pi              (pi_param_l0_i9_pi),
        .dcdl_param_l0_i9_dcdl          (dcdl_param_l0_i9_dcdl),
        .offset_param_l0_i10_offset        (offset_param_l0_i10_offset),
        .vref_param_l0_i10_vref          (vref_param_l0_i10_vref),
        .pi_param_l0_i10_pi              (pi_param_l0_i10_pi),
        .dcdl_param_l0_i10_dcdl          (dcdl_param_l0_i10_dcdl),
        .offset_param_l0_i11_offset        (offset_param_l0_i11_offset),
        .vref_param_l0_i11_vref          (vref_param_l0_i11_vref),
        .pi_param_l0_i11_pi              (pi_param_l0_i11_pi),
        .dcdl_param_l0_i11_dcdl          (dcdl_param_l0_i11_dcdl),
        .offset_param_l0_i12_offset        (offset_param_l0_i12_offset),
        .vref_param_l0_i12_vref          (vref_param_l0_i12_vref),
        .pi_param_l0_i12_pi              (pi_param_l0_i12_pi),
        .dcdl_param_l0_i12_dcdl          (dcdl_param_l0_i12_dcdl),
        .offset_param_l0_i13_offset        (offset_param_l0_i13_offset),
        .vref_param_l0_i13_vref          (vref_param_l0_i13_vref),
        .pi_param_l0_i13_pi              (pi_param_l0_i13_pi),
        .dcdl_param_l0_i13_dcdl          (dcdl_param_l0_i13_dcdl),
        .offset_param_l0_i14_offset        (offset_param_l0_i14_offset),
        .vref_param_l0_i14_vref          (vref_param_l0_i14_vref),
        .pi_param_l0_i14_pi              (pi_param_l0_i14_pi),
        .dcdl_param_l0_i14_dcdl          (dcdl_param_l0_i14_dcdl),
        .offset_param_l0_i15_offset        (offset_param_l0_i15_offset),
        .vref_param_l0_i15_vref          (vref_param_l0_i15_vref),
        .pi_param_l0_i15_pi              (pi_param_l0_i15_pi),
        .dcdl_param_l0_i15_dcdl          (dcdl_param_l0_i15_dcdl),
        .offset_param_l0_i16_offset      (offset_param_l0_i16_offset),
        .dcdl_param_l0_i16_dcdl          (dcdl_param_l0_i16_dcdl),
        // MDE-CHAIN-1008 lane1 train 参数贯通 (2026-10-12)
        .offset_param_l1_i0_offset            (offset_param_l1_i0_offset),
        .offset_param_l1_i1_offset            (offset_param_l1_i1_offset),
        .offset_param_l1_i2_offset            (offset_param_l1_i2_offset),
        .offset_param_l1_i3_offset            (offset_param_l1_i3_offset),
        .offset_param_l1_i4_offset            (offset_param_l1_i4_offset),
        .offset_param_l1_i5_offset            (offset_param_l1_i5_offset),
        .offset_param_l1_i6_offset            (offset_param_l1_i6_offset),
        .offset_param_l1_i7_offset            (offset_param_l1_i7_offset),
        .offset_param_l1_i8_offset            (offset_param_l1_i8_offset),
        .offset_param_l1_i9_offset            (offset_param_l1_i9_offset),
        .offset_param_l1_i10_offset            (offset_param_l1_i10_offset),
        .offset_param_l1_i11_offset            (offset_param_l1_i11_offset),
        .offset_param_l1_i12_offset            (offset_param_l1_i12_offset),
        .offset_param_l1_i13_offset            (offset_param_l1_i13_offset),
        .offset_param_l1_i14_offset            (offset_param_l1_i14_offset),
        .offset_param_l1_i15_offset            (offset_param_l1_i15_offset),
        .offset_param_l1_i16_offset            (offset_param_l1_i16_offset),
        .vref_param_l1_i0_vref            (vref_param_l1_i0_vref),
        .vref_param_l1_i1_vref            (vref_param_l1_i1_vref),
        .vref_param_l1_i2_vref            (vref_param_l1_i2_vref),
        .vref_param_l1_i3_vref            (vref_param_l1_i3_vref),
        .vref_param_l1_i4_vref            (vref_param_l1_i4_vref),
        .vref_param_l1_i5_vref            (vref_param_l1_i5_vref),
        .vref_param_l1_i6_vref            (vref_param_l1_i6_vref),
        .vref_param_l1_i7_vref            (vref_param_l1_i7_vref),
        .vref_param_l1_i8_vref            (vref_param_l1_i8_vref),
        .vref_param_l1_i9_vref            (vref_param_l1_i9_vref),
        .vref_param_l1_i10_vref            (vref_param_l1_i10_vref),
        .vref_param_l1_i11_vref            (vref_param_l1_i11_vref),
        .vref_param_l1_i12_vref            (vref_param_l1_i12_vref),
        .vref_param_l1_i13_vref            (vref_param_l1_i13_vref),
        .vref_param_l1_i14_vref            (vref_param_l1_i14_vref),
        .vref_param_l1_i15_vref            (vref_param_l1_i15_vref),
        .pi_param_l1_i0_pi            (pi_param_l1_i0_pi),
        .pi_param_l1_i1_pi            (pi_param_l1_i1_pi),
        .pi_param_l1_i2_pi            (pi_param_l1_i2_pi),
        .pi_param_l1_i3_pi            (pi_param_l1_i3_pi),
        .pi_param_l1_i4_pi            (pi_param_l1_i4_pi),
        .pi_param_l1_i5_pi            (pi_param_l1_i5_pi),
        .pi_param_l1_i6_pi            (pi_param_l1_i6_pi),
        .pi_param_l1_i7_pi            (pi_param_l1_i7_pi),
        .pi_param_l1_i8_pi            (pi_param_l1_i8_pi),
        .pi_param_l1_i9_pi            (pi_param_l1_i9_pi),
        .pi_param_l1_i10_pi            (pi_param_l1_i10_pi),
        .pi_param_l1_i11_pi            (pi_param_l1_i11_pi),
        .pi_param_l1_i12_pi            (pi_param_l1_i12_pi),
        .pi_param_l1_i13_pi            (pi_param_l1_i13_pi),
        .pi_param_l1_i14_pi            (pi_param_l1_i14_pi),
        .pi_param_l1_i15_pi            (pi_param_l1_i15_pi),
        .dcdl_param_l1_i0_dcdl            (dcdl_param_l1_i0_dcdl),
        .dcdl_param_l1_i1_dcdl            (dcdl_param_l1_i1_dcdl),
        .dcdl_param_l1_i2_dcdl            (dcdl_param_l1_i2_dcdl),
        .dcdl_param_l1_i3_dcdl            (dcdl_param_l1_i3_dcdl),
        .dcdl_param_l1_i4_dcdl            (dcdl_param_l1_i4_dcdl),
        .dcdl_param_l1_i5_dcdl            (dcdl_param_l1_i5_dcdl),
        .dcdl_param_l1_i6_dcdl            (dcdl_param_l1_i6_dcdl),
        .dcdl_param_l1_i7_dcdl            (dcdl_param_l1_i7_dcdl),
        .dcdl_param_l1_i8_dcdl            (dcdl_param_l1_i8_dcdl),
        .dcdl_param_l1_i9_dcdl            (dcdl_param_l1_i9_dcdl),
        .dcdl_param_l1_i10_dcdl            (dcdl_param_l1_i10_dcdl),
        .dcdl_param_l1_i11_dcdl            (dcdl_param_l1_i11_dcdl),
        .dcdl_param_l1_i12_dcdl            (dcdl_param_l1_i12_dcdl),
        .dcdl_param_l1_i13_dcdl            (dcdl_param_l1_i13_dcdl),
        .dcdl_param_l1_i14_dcdl            (dcdl_param_l1_i14_dcdl),
        .dcdl_param_l1_i15_dcdl            (dcdl_param_l1_i15_dcdl),
        .dcdl_param_l1_i16_dcdl            (dcdl_param_l1_i16_dcdl),
        // MDE-CHAIN-1008 lane2 train 参数贯通 (2026-10-12)
        .offset_param_l2_i0_offset            (offset_param_l2_i0_offset),
        .offset_param_l2_i1_offset            (offset_param_l2_i1_offset),
        .offset_param_l2_i2_offset            (offset_param_l2_i2_offset),
        .offset_param_l2_i3_offset            (offset_param_l2_i3_offset),
        .offset_param_l2_i4_offset            (offset_param_l2_i4_offset),
        .offset_param_l2_i5_offset            (offset_param_l2_i5_offset),
        .offset_param_l2_i6_offset            (offset_param_l2_i6_offset),
        .offset_param_l2_i7_offset            (offset_param_l2_i7_offset),
        .offset_param_l2_i8_offset            (offset_param_l2_i8_offset),
        .offset_param_l2_i9_offset            (offset_param_l2_i9_offset),
        .offset_param_l2_i10_offset            (offset_param_l2_i10_offset),
        .offset_param_l2_i11_offset            (offset_param_l2_i11_offset),
        .offset_param_l2_i12_offset            (offset_param_l2_i12_offset),
        .offset_param_l2_i13_offset            (offset_param_l2_i13_offset),
        .offset_param_l2_i14_offset            (offset_param_l2_i14_offset),
        .offset_param_l2_i15_offset            (offset_param_l2_i15_offset),
        .offset_param_l2_i16_offset            (offset_param_l2_i16_offset),
        .vref_param_l2_i0_vref            (vref_param_l2_i0_vref),
        .vref_param_l2_i1_vref            (vref_param_l2_i1_vref),
        .vref_param_l2_i2_vref            (vref_param_l2_i2_vref),
        .vref_param_l2_i3_vref            (vref_param_l2_i3_vref),
        .vref_param_l2_i4_vref            (vref_param_l2_i4_vref),
        .vref_param_l2_i5_vref            (vref_param_l2_i5_vref),
        .vref_param_l2_i6_vref            (vref_param_l2_i6_vref),
        .vref_param_l2_i7_vref            (vref_param_l2_i7_vref),
        .vref_param_l2_i8_vref            (vref_param_l2_i8_vref),
        .vref_param_l2_i9_vref            (vref_param_l2_i9_vref),
        .vref_param_l2_i10_vref            (vref_param_l2_i10_vref),
        .vref_param_l2_i11_vref            (vref_param_l2_i11_vref),
        .vref_param_l2_i12_vref            (vref_param_l2_i12_vref),
        .vref_param_l2_i13_vref            (vref_param_l2_i13_vref),
        .vref_param_l2_i14_vref            (vref_param_l2_i14_vref),
        .vref_param_l2_i15_vref            (vref_param_l2_i15_vref),
        .pi_param_l2_i0_pi            (pi_param_l2_i0_pi),
        .pi_param_l2_i1_pi            (pi_param_l2_i1_pi),
        .pi_param_l2_i2_pi            (pi_param_l2_i2_pi),
        .pi_param_l2_i3_pi            (pi_param_l2_i3_pi),
        .pi_param_l2_i4_pi            (pi_param_l2_i4_pi),
        .pi_param_l2_i5_pi            (pi_param_l2_i5_pi),
        .pi_param_l2_i6_pi            (pi_param_l2_i6_pi),
        .pi_param_l2_i7_pi            (pi_param_l2_i7_pi),
        .pi_param_l2_i8_pi            (pi_param_l2_i8_pi),
        .pi_param_l2_i9_pi            (pi_param_l2_i9_pi),
        .pi_param_l2_i10_pi            (pi_param_l2_i10_pi),
        .pi_param_l2_i11_pi            (pi_param_l2_i11_pi),
        .pi_param_l2_i12_pi            (pi_param_l2_i12_pi),
        .pi_param_l2_i13_pi            (pi_param_l2_i13_pi),
        .pi_param_l2_i14_pi            (pi_param_l2_i14_pi),
        .pi_param_l2_i15_pi            (pi_param_l2_i15_pi),
        .dcdl_param_l2_i0_dcdl            (dcdl_param_l2_i0_dcdl),
        .dcdl_param_l2_i1_dcdl            (dcdl_param_l2_i1_dcdl),
        .dcdl_param_l2_i2_dcdl            (dcdl_param_l2_i2_dcdl),
        .dcdl_param_l2_i3_dcdl            (dcdl_param_l2_i3_dcdl),
        .dcdl_param_l2_i4_dcdl            (dcdl_param_l2_i4_dcdl),
        .dcdl_param_l2_i5_dcdl            (dcdl_param_l2_i5_dcdl),
        .dcdl_param_l2_i6_dcdl            (dcdl_param_l2_i6_dcdl),
        .dcdl_param_l2_i7_dcdl            (dcdl_param_l2_i7_dcdl),
        .dcdl_param_l2_i8_dcdl            (dcdl_param_l2_i8_dcdl),
        .dcdl_param_l2_i9_dcdl            (dcdl_param_l2_i9_dcdl),
        .dcdl_param_l2_i10_dcdl            (dcdl_param_l2_i10_dcdl),
        .dcdl_param_l2_i11_dcdl            (dcdl_param_l2_i11_dcdl),
        .dcdl_param_l2_i12_dcdl            (dcdl_param_l2_i12_dcdl),
        .dcdl_param_l2_i13_dcdl            (dcdl_param_l2_i13_dcdl),
        .dcdl_param_l2_i14_dcdl            (dcdl_param_l2_i14_dcdl),
        .dcdl_param_l2_i15_dcdl            (dcdl_param_l2_i15_dcdl),
        .dcdl_param_l2_i16_dcdl            (dcdl_param_l2_i16_dcdl),
        // MDE-CHAIN-1008 lane3 train 参数贯通 (2026-10-12)
        .offset_param_l3_i0_offset            (offset_param_l3_i0_offset),
        .offset_param_l3_i1_offset            (offset_param_l3_i1_offset),
        .offset_param_l3_i2_offset            (offset_param_l3_i2_offset),
        .offset_param_l3_i3_offset            (offset_param_l3_i3_offset),
        .offset_param_l3_i4_offset            (offset_param_l3_i4_offset),
        .offset_param_l3_i5_offset            (offset_param_l3_i5_offset),
        .offset_param_l3_i6_offset            (offset_param_l3_i6_offset),
        .offset_param_l3_i7_offset            (offset_param_l3_i7_offset),
        .offset_param_l3_i8_offset            (offset_param_l3_i8_offset),
        .offset_param_l3_i9_offset            (offset_param_l3_i9_offset),
        .offset_param_l3_i10_offset            (offset_param_l3_i10_offset),
        .offset_param_l3_i11_offset            (offset_param_l3_i11_offset),
        .offset_param_l3_i12_offset            (offset_param_l3_i12_offset),
        .offset_param_l3_i13_offset            (offset_param_l3_i13_offset),
        .offset_param_l3_i14_offset            (offset_param_l3_i14_offset),
        .offset_param_l3_i15_offset            (offset_param_l3_i15_offset),
        .offset_param_l3_i16_offset            (offset_param_l3_i16_offset),
        .vref_param_l3_i0_vref            (vref_param_l3_i0_vref),
        .vref_param_l3_i1_vref            (vref_param_l3_i1_vref),
        .vref_param_l3_i2_vref            (vref_param_l3_i2_vref),
        .vref_param_l3_i3_vref            (vref_param_l3_i3_vref),
        .vref_param_l3_i4_vref            (vref_param_l3_i4_vref),
        .vref_param_l3_i5_vref            (vref_param_l3_i5_vref),
        .vref_param_l3_i6_vref            (vref_param_l3_i6_vref),
        .vref_param_l3_i7_vref            (vref_param_l3_i7_vref),
        .vref_param_l3_i8_vref            (vref_param_l3_i8_vref),
        .vref_param_l3_i9_vref            (vref_param_l3_i9_vref),
        .vref_param_l3_i10_vref            (vref_param_l3_i10_vref),
        .vref_param_l3_i11_vref            (vref_param_l3_i11_vref),
        .vref_param_l3_i12_vref            (vref_param_l3_i12_vref),
        .vref_param_l3_i13_vref            (vref_param_l3_i13_vref),
        .vref_param_l3_i14_vref            (vref_param_l3_i14_vref),
        .vref_param_l3_i15_vref            (vref_param_l3_i15_vref),
        .pi_param_l3_i0_pi            (pi_param_l3_i0_pi),
        .pi_param_l3_i1_pi            (pi_param_l3_i1_pi),
        .pi_param_l3_i2_pi            (pi_param_l3_i2_pi),
        .pi_param_l3_i3_pi            (pi_param_l3_i3_pi),
        .pi_param_l3_i4_pi            (pi_param_l3_i4_pi),
        .pi_param_l3_i5_pi            (pi_param_l3_i5_pi),
        .pi_param_l3_i6_pi            (pi_param_l3_i6_pi),
        .pi_param_l3_i7_pi            (pi_param_l3_i7_pi),
        .pi_param_l3_i8_pi            (pi_param_l3_i8_pi),
        .pi_param_l3_i9_pi            (pi_param_l3_i9_pi),
        .pi_param_l3_i10_pi            (pi_param_l3_i10_pi),
        .pi_param_l3_i11_pi            (pi_param_l3_i11_pi),
        .pi_param_l3_i12_pi            (pi_param_l3_i12_pi),
        .pi_param_l3_i13_pi            (pi_param_l3_i13_pi),
        .pi_param_l3_i14_pi            (pi_param_l3_i14_pi),
        .pi_param_l3_i15_pi            (pi_param_l3_i15_pi),
        .dcdl_param_l3_i0_dcdl            (dcdl_param_l3_i0_dcdl),
        .dcdl_param_l3_i1_dcdl            (dcdl_param_l3_i1_dcdl),
        .dcdl_param_l3_i2_dcdl            (dcdl_param_l3_i2_dcdl),
        .dcdl_param_l3_i3_dcdl            (dcdl_param_l3_i3_dcdl),
        .dcdl_param_l3_i4_dcdl            (dcdl_param_l3_i4_dcdl),
        .dcdl_param_l3_i5_dcdl            (dcdl_param_l3_i5_dcdl),
        .dcdl_param_l3_i6_dcdl            (dcdl_param_l3_i6_dcdl),
        .dcdl_param_l3_i7_dcdl            (dcdl_param_l3_i7_dcdl),
        .dcdl_param_l3_i8_dcdl            (dcdl_param_l3_i8_dcdl),
        .dcdl_param_l3_i9_dcdl            (dcdl_param_l3_i9_dcdl),
        .dcdl_param_l3_i10_dcdl            (dcdl_param_l3_i10_dcdl),
        .dcdl_param_l3_i11_dcdl            (dcdl_param_l3_i11_dcdl),
        .dcdl_param_l3_i12_dcdl            (dcdl_param_l3_i12_dcdl),
        .dcdl_param_l3_i13_dcdl            (dcdl_param_l3_i13_dcdl),
        .dcdl_param_l3_i14_dcdl            (dcdl_param_l3_i14_dcdl),
        .dcdl_param_l3_i15_dcdl            (dcdl_param_l3_i15_dcdl),
        .dcdl_param_l3_i16_dcdl            (dcdl_param_l3_i16_dcdl),
        .train_param_src_o               (train_param_src_o),
        .datacheck_cfg_fail_threshold_o  (datacheck_cfg_fail_threshold_o),
        .eq_param_ctle_tap_o             (eq_param_ctle_tap_o)
    );

    // --------------------------------------------------------------------------
    // intc：mask 过滤 + OR 聚合（中断源寄存器归 apb_csr_top，同域无 CDC）
    // --------------------------------------------------------------------------
    intc u_intc (
        .apb_pclk       (apb_pclk),
        .apb_preset_n   (apb_preset_n),
        .int_status_i   (int_status),
        .int_mask_mcu_i (int_mask_mcu),
        .int_mask_ext_i (int_mask_ext),
        .irq_mcu_o      (irq_mcu),
        .irq_ext        (irq_ext)
    );

    // --------------------------------------------------------------------------
    // mcu_subsys：MCU 子系统 black-box stub（功能归 MCU 子系统责任人；
    // APB master 0 → apb_csr_top，APB master 1 → 顶端口协议层+链路层 CSR）
    // --------------------------------------------------------------------------
`ifdef MCU_SUBSYS
    mcu_subsys u_mcu_subsys (
        .mcu_clk            (mcu_clk),
        .mcu_rst_n          (mcu_rstn_sync),
        .apb_pclk           (apb_pclk),
        .apb_preset_n       (apb_preset_n),
        // AHB slave ← top
        .ahbs_hsel          (ahbs_hsel),
        .ahbs_haddr         (ahbs_haddr),
        .ahbs_hburst        (ahbs_hburst),
        .ahbs_hsize         (ahbs_hsize),
        .ahbs_hprot         (ahbs_hprot),
        .ahbs_hwdata        (ahbs_hwdata),
        .ahbs_hwrite        (ahbs_hwrite),
        .ahbs_htrans        (ahbs_htrans),
        .ahbs_hready        (ahbs_hready),
        .ahbs_hrdata        (ahbs_hrdata),
        .ahbs_hresp         (ahbs_hresp),
        .ahbs_hready_resp   (ahbs_hready_resp),
        // cfg broadcast / 聚合中断
        .cfg_i              (cfg_mcu),
        .irq_i              (irq_mcu),
        // APB master 0 → apb_csr_top
        .apb_csr_psel       (mcu_apb_psel),
        .apb_csr_penable    (mcu_apb_penable),
        .apb_csr_pwrite     (mcu_apb_pwrite),
        .apb_csr_paddr      (mcu_apb_paddr),
        .apb_csr_pwdata     (mcu_apb_pwdata),
        .apb_csr_prdata     (mcu_apb_prdata),
        .apb_csr_pready     (mcu_apb_pready),
        .apb_csr_pslverr    (mcu_apb_pslverr),
        // APB master 1 → 顶端口（协议层+链路层 CSR 合并空间）
        .apb_pl_psel        (apb_psel),
        .apb_pl_penable     (apb_penable),
        .apb_pl_pwrite      (apb_pwrite),
        .apb_pl_paddr       (apb_paddr),
        .apb_pl_pwdata      (apb_pwdata),
        .apb_pl_prdata      (apb_prdata),
        .apb_pl_pready      (apb_pready),
        .apb_pl_pslverr     (apb_pslverr)
    );
`endif

endmodule

