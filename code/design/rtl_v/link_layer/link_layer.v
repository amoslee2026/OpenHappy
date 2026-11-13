// =============================================================================
// module name：link_layer
// author: OpenHappy Team
// MAS：spec/link_layer/submods/link_layer/link_layer_spec.md（L2 顶层封装 v1.0）
// 功能概述: 链路层物理 RTL 顶层封装。例化 6 类子模块并完成互连：
//           link_tx ×TX_LINK_NUM / link_rx ×RX_LINK_NUM（数据通路，mb_clk）、
//           link_tx_lcmsm / link_rx_lcmsm（本地控制状态机，sb_clk）、
//           link_event_process（事件处理，sb_clk）、apb_csr（集中式 CSR，
//           apb_clk）。本模块不承载子模块内部功能，仅负责：
//           ① 子模块互连（PAIF/CPIF/UDI/NACK/ACK/Loopback/控制信号对接）；
//           ② 跨时钟域同步（mb_clk 2GHz / sb_clk 800MHz / apb_clk 1GHz
//              三域两两互异步：电平/准静态配置 2flop 同步，脉冲 toggle+
//              边沿检测同步，均位于接收侧）；
//           ③ 对外引脚聚合；④ DFx / 中断聚合至 apb_csr。
// 编码约束（本模块强制）:
//   1. always 块内 if 判断（不含 reset）≤ 2 个；
//   2. for 循环仅用于 generate 例化；
//   3. wire 声明集中在模块头部信号声明区，与 assign 赋值分离；
//   4. mb_clk 域寄存器同步复位（posedge only）；sb_clk / apb_clk 域寄存器
//      异步复位低有效（posedge or negedge）。
// 偏离说明（详见版本历史与评审记录）:
//   ① L2 spec §3.1 的 LINKx_EXIST 拆分为 TX_LINKx_EXIST / RX_LINKx_EXIST
//      （不对称结构 TX/RX link 数独立）；
//   ② 多 bit 状态/计数回读采用 2flop 逐 bit 同步（非 L2 §3.3 的 Gray/req-ack），
//      统计类计数器读值可能瞬时撕裂、自清零语义由 apb_csr 本地实现；
//   ③ link_tx_lcmsm.dskew_pattern_o 悬空（link_tx 无 dskew_pat 输入端口，
//      link_tx_mas v1.0 未定义该信号）；
//   ④ lcmsm soft_rst_i 接 1'b0（apb_csr blueprint 无 soft_rst_o 端口）。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   <日期>  <修订人>  <变更概述>
//  2026-08-08  OpenHappy Team  全量重写（对齐 L2 spec v1.0 评审稿）：替换原空壳
//                          （LL_* 宏体系），按 L2 §2 端口 / §3 例化互连 /
//                          §3.3 CDC 规则实现 6 类子模块集成
//  2026-08-11  OpenHappy Team  CPIF/UDI link1~3 补 lane2/3 端口声明（LINK_LANE3）；
//                          UDI link1~3 对齐 CPIF（lane0/1/2/3 齐全）；
//                          移除 link4 端口+死线+宏（link 仅支持 0~3）。
//  2026-08-12  OpenHappy Team  lint 修复：rx1_* 反向通道网声明提前至 u_link_tx1 首次
//                          使用之前（消除 W110 端口位宽不匹配 + WRN_40 重复声明）
//  2026-08-17  OpenHappy Team  补齐 link2/link3 例化（u_link_tx2/rx2、u_link_tx3/rx3 +
//                          rx2_*/rx3_* 反向通道网 + 各自 loopback 源选择 mux），
//                          link 0~3 全量例化支持最多 4 link（TX/RX_LINKx_EXIST 守卫）
//  2026-09-11  OpenHappy Team  初始化流程补齐（G-01/G-03/G-04）：STATUS_REG.PHY_READY
//                          （cpif tx/rx ready 相与经 2flop 同步到 apb_csr）；
//                          RX_READY_IRQ / LANE_ON_DONE_IRQ 两路 sb→apb 脉冲同步
//                          （link_event_process 中断源 → apb_csr IRQ 聚合）
//  2026-10-08  OpenHappy Team  LL-20260930-N4-v3（F1a 接线）：顶层新 wire
//                          tx_lcsm_active_nxt（u_link_tx_lcmsm.link_active_nxt_o →
//                          u_link_tx0..3.link_active_nxt_i，4 实例各接）；状态机
//                          迁移方程 / 协议拍序不变（零功能变化）。
//  2026-10-23  OpenHappy Team  LL-20261023-CPIFRXLPC：新增输入 cpif_rx_lpc_i（CPIF RX
//                          LPC 状态，logic_phy 4 lane 组合 AND 归约的异步组合电平）→
//                          1 个 sync_cell_norst 2flop 同步到 apb_clk → apb_csr
//                          .status_reg_rx_lpc_i → STATUS_REG.RX_LPC（0x0034 bit9，RO，
//                          复位名义 1）观测回读；纯观测（无 FSM / 中断 / 数据通路用途），
//                          +3 FF，零行为变化（L1 v1.17 / L2 v1.9 / apb_csr MAS v1.14）
// =============================================================================

`timescale 1ns/1ps

`include "link_layer_defines.vh"

module link_layer (
    // =========================================================================
    // 时钟与复位（L2 §2.1）
    // =========================================================================
    input  wire                         mb_clk,                 // 数据通路主时钟 2 GHz
    input  wire                         sb_clk,                 // 控制/事件通路时钟 800 MHz
    input  wire                         apb_clk,                // APB 配置通路时钟 1 GHz
    input  wire                         mb_rst_n,               // 数据通路同步复位，低有效
    input  wire                         sb_rst_n,               // 控制通路异步复位，低有效
    input  wire                         apb_rst_n,              // APB 域异步复位，低有效
    input  wire                       paif_clk,                 // 预留端口（hipi_ctrl_top 兼容），当前不使用
    input  wire                       paif_rstn,                // 预留端口（hipi_ctrl_top 兼容），当前不使用

    // =========================================================================
    // PAIF TX / RX 数据接口（L2 §2.2，mb_clk）
    // =========================================================================
    input  wire [`PAIF_WIDTH-1:0]       paif_tx_link0_data_i,
    input  wire                         paif_tx_link0_valid_i,
    output wire                         paif_tx_link0_ready_o,
    output wire [`PAIF_WIDTH-1:0]       paif_rx_link0_data_o,
    output wire                         paif_rx_link0_valid_o,
    input  wire                         paif_rx_link0_ready_i,
`ifdef TX_LINK1_EXIST
    input  wire [`PAIF_WIDTH-1:0]       paif_tx_link1_data_i,
    input  wire                         paif_tx_link1_valid_i,
    output wire                         paif_tx_link1_ready_o,
`endif
`ifdef RX_LINK1_EXIST
    output wire [`PAIF_WIDTH-1:0]       paif_rx_link1_data_o,
    output wire                         paif_rx_link1_valid_o,
    input  wire                         paif_rx_link1_ready_i,
`endif
`ifdef TX_LINK2_EXIST
    input  wire [`PAIF_WIDTH-1:0]       paif_tx_link2_data_i,
    input  wire                         paif_tx_link2_valid_i,
    output wire                         paif_tx_link2_ready_o,
`endif
`ifdef RX_LINK2_EXIST
    output wire [`PAIF_WIDTH-1:0]       paif_rx_link2_data_o,
    output wire                         paif_rx_link2_valid_o,
    input  wire                         paif_rx_link2_ready_i,
`endif
`ifdef TX_LINK3_EXIST
    input  wire [`PAIF_WIDTH-1:0]       paif_tx_link3_data_i,
    input  wire                         paif_tx_link3_valid_i,
    output wire                         paif_tx_link3_ready_o,
`endif
`ifdef RX_LINK3_EXIST
    output wire [`PAIF_WIDTH-1:0]       paif_rx_link3_data_o,
    output wire                         paif_rx_link3_valid_o,
    input  wire                         paif_rx_link3_ready_i,
`endif

    // =========================================================================
    // CPIF TX / RX 数据接口（L2 §2.3，mb_clk；per-lane 1024 bit，TX ready 共享）
    // lane 声明规则：lane0 永远；`ifdef LINK_LANE1 → lane1；`ifdef LINK_LANE3 → lane2/3
    // （所有 link 共享同一 LANE_MACRO_MODE，故用全局 LINK_LANE1/LINK_LANE3）
    // =========================================================================
    output wire [1023:0]                cpif_tx_link0_lane0_data_o,
    output wire                         cpif_tx_link0_lane0_data_vld_o,
`ifdef LINK_LANE1
    output wire [1023:0]                cpif_tx_link0_lane1_data_o,
    output wire                         cpif_tx_link0_lane1_data_vld_o,
`endif
`ifdef LINK_LANE3
    output wire [1023:0]                cpif_tx_link0_lane2_data_o,
    output wire                         cpif_tx_link0_lane2_data_vld_o,
    output wire [1023:0]                cpif_tx_link0_lane3_data_o,
    output wire                         cpif_tx_link0_lane3_data_vld_o,
`endif
    input  wire                         cpif_tx_link0_ready_i,
    input  wire                         cpif_rx_link0_ready_i,
    input  wire [1023:0]                cpif_rx_link0_lane0_data_i,
    input  wire                         cpif_rx_link0_lane0_data_vld_i,
`ifdef LINK_LANE1
    input  wire [1023:0]                cpif_rx_link0_lane1_data_i,
    input  wire                         cpif_rx_link0_lane1_data_vld_i,
`endif
`ifdef LINK_LANE3
    input  wire [1023:0]                cpif_rx_link0_lane2_data_i,
    input  wire                         cpif_rx_link0_lane2_data_vld_i,
    input  wire [1023:0]                cpif_rx_link0_lane3_data_i,
    input  wire                         cpif_rx_link0_lane3_data_vld_i,
`endif
`ifdef TX_LINK1_EXIST
    output wire [1023:0]                cpif_tx_link1_lane0_data_o,
    output wire                         cpif_tx_link1_lane0_data_vld_o,
`ifdef LINK_LANE1
    output wire [1023:0]                cpif_tx_link1_lane1_data_o,
    output wire                         cpif_tx_link1_lane1_data_vld_o,
`endif
`ifdef LINK_LANE3
    output wire [1023:0]                cpif_tx_link1_lane2_data_o,
    output wire                         cpif_tx_link1_lane2_data_vld_o,
    output wire [1023:0]                cpif_tx_link1_lane3_data_o,
    output wire                         cpif_tx_link1_lane3_data_vld_o,
`endif
    input  wire                         cpif_tx_link1_ready_i,
`endif
`ifdef RX_LINK1_EXIST
    input  wire [1023:0]                cpif_rx_link1_lane0_data_i,
    input  wire                         cpif_rx_link1_lane0_data_vld_i,
`ifdef LINK_LANE1
    input  wire [1023:0]                cpif_rx_link1_lane1_data_i,
    input  wire                         cpif_rx_link1_lane1_data_vld_i,
`endif
`ifdef LINK_LANE3
    input  wire [1023:0]                cpif_rx_link1_lane2_data_i,
    input  wire                         cpif_rx_link1_lane2_data_vld_i,
    input  wire [1023:0]                cpif_rx_link1_lane3_data_i,
    input  wire                         cpif_rx_link1_lane3_data_vld_i,
`endif
`endif
`ifdef TX_LINK2_EXIST
    output wire [1023:0]                cpif_tx_link2_lane0_data_o,
    output wire                         cpif_tx_link2_lane0_data_vld_o,
`ifdef LINK_LANE1
    output wire [1023:0]                cpif_tx_link2_lane1_data_o,
    output wire                         cpif_tx_link2_lane1_data_vld_o,
`endif
`ifdef LINK_LANE3
    output wire [1023:0]                cpif_tx_link2_lane2_data_o,
    output wire                         cpif_tx_link2_lane2_data_vld_o,
    output wire [1023:0]                cpif_tx_link2_lane3_data_o,
    output wire                         cpif_tx_link2_lane3_data_vld_o,
`endif
    input  wire                         cpif_tx_link2_ready_i,
`endif
`ifdef RX_LINK2_EXIST
    input  wire [1023:0]                cpif_rx_link2_lane0_data_i,
    input  wire                         cpif_rx_link2_lane0_data_vld_i,
`ifdef LINK_LANE1
    input  wire [1023:0]                cpif_rx_link2_lane1_data_i,
    input  wire                         cpif_rx_link2_lane1_data_vld_i,
`endif
`ifdef LINK_LANE3
    input  wire [1023:0]                cpif_rx_link2_lane2_data_i,
    input  wire                         cpif_rx_link2_lane2_data_vld_i,
    input  wire [1023:0]                cpif_rx_link2_lane3_data_i,
    input  wire                         cpif_rx_link2_lane3_data_vld_i,
`endif
`endif
`ifdef TX_LINK3_EXIST
    output wire [1023:0]                cpif_tx_link3_lane0_data_o,
    output wire                         cpif_tx_link3_lane0_data_vld_o,
`ifdef LINK_LANE1
    output wire [1023:0]                cpif_tx_link3_lane1_data_o,
    output wire                         cpif_tx_link3_lane1_data_vld_o,
`endif
`ifdef LINK_LANE3
    output wire [1023:0]                cpif_tx_link3_lane2_data_o,
    output wire                         cpif_tx_link3_lane2_data_vld_o,
    output wire [1023:0]                cpif_tx_link3_lane3_data_o,
    output wire                         cpif_tx_link3_lane3_data_vld_o,
`endif
    input  wire                         cpif_tx_link3_ready_i,
`endif
`ifdef RX_LINK3_EXIST
    input  wire [1023:0]                cpif_rx_link3_lane0_data_i,
    input  wire                         cpif_rx_link3_lane0_data_vld_i,
`ifdef LINK_LANE1
    input  wire [1023:0]                cpif_rx_link3_lane1_data_i,
    input  wire                         cpif_rx_link3_lane1_data_vld_i,
`endif
`ifdef LINK_LANE3
    input  wire [1023:0]                cpif_rx_link3_lane2_data_i,
    input  wire                         cpif_rx_link3_lane2_data_vld_i,
    input  wire [1023:0]                cpif_rx_link3_lane3_data_i,
    input  wire                         cpif_rx_link3_lane3_data_vld_i,
`endif
`endif

    // =========================================================================
    // CPIF TX / RX 控制接口（L2 §2.3.2）：输出 5 路为 apb_clk 寄存器直驱、不经跨域；
    //   输入 1 路 cpif_rx_lpc_i 为异步组合电平（非 mb_clk/apb_clk 同步域，v1.9），
    //   经 sync_cell_norst 2flop 同步后仅作 STATUS_REG.RX_LPC 的 RO 观测
    // =========================================================================
    output wire [3:0]                   cpif_tx_mode_o,
    output wire                         cpif_tx_rst_n_o,
    output wire                         cpif_tx_lpc_o,
    output wire [3:0]                   cpif_rx_mode_o,
    output wire                         cpif_rx_rst_n_o,
    input  wire                         cpif_rx_lpc_i,          // CPIF RX LPC 状态（logic_phy 4 lane 组合 AND 归约，异步组合电平、非 mb_clk/apb_clk 同步域，L2 v1.9 §2.3.2）

    // =========================================================================
    // UDI TX / RX 数据接口（L2 §2.4，mb_clk，per-lane 1024 bit）
    // =========================================================================
    input  wire [1023:0]                udi_tx_link0_lane0_data_i,
    input  wire                         udi_tx_link0_lane0_data_vld_i,
    output wire                         udi_tx_link0_ready_o,
    output wire [1023:0]                udi_rx_link0_lane0_data_o,
    output wire                         udi_rx_link0_lane0_data_vld_o,
`ifdef LINK_LANE1
    input  wire [1023:0]                udi_tx_link0_lane1_data_i,
    input  wire                         udi_tx_link0_lane1_data_vld_i,
    output wire [1023:0]                udi_rx_link0_lane1_data_o,
    output wire                         udi_rx_link0_lane1_data_vld_o,
`endif
`ifdef LINK_LANE3
    input  wire [1023:0]                udi_tx_link0_lane2_data_i,
    input  wire                         udi_tx_link0_lane2_data_vld_i,
    input  wire [1023:0]                udi_tx_link0_lane3_data_i,
    input  wire                         udi_tx_link0_lane3_data_vld_i,
    output wire [1023:0]                udi_rx_link0_lane2_data_o,
    output wire                         udi_rx_link0_lane2_data_vld_o,
    output wire [1023:0]                udi_rx_link0_lane3_data_o,
    output wire                         udi_rx_link0_lane3_data_vld_o,
`endif
`ifdef TX_LINK1_EXIST
    input  wire [1023:0]                udi_tx_link1_lane0_data_i,
    input  wire                         udi_tx_link1_lane0_data_vld_i,
    output wire                         udi_tx_link1_ready_o,
`ifdef LINK_LANE1
    input  wire [1023:0]                udi_tx_link1_lane1_data_i,
    input  wire                         udi_tx_link1_lane1_data_vld_i,
`endif
`ifdef LINK_LANE3
    input  wire [1023:0]                udi_tx_link1_lane2_data_i,
    input  wire                         udi_tx_link1_lane2_data_vld_i,
    input  wire [1023:0]                udi_tx_link1_lane3_data_i,
    input  wire                         udi_tx_link1_lane3_data_vld_i,
`endif
`endif
`ifdef RX_LINK1_EXIST
    output wire [1023:0]                udi_rx_link1_lane0_data_o,
    output wire                         udi_rx_link1_lane0_data_vld_o,
`ifdef LINK_LANE1
    output wire [1023:0]                udi_rx_link1_lane1_data_o,
    output wire                         udi_rx_link1_lane1_data_vld_o,
`endif
`ifdef LINK_LANE3
    output wire [1023:0]                udi_rx_link1_lane2_data_o,
    output wire                         udi_rx_link1_lane2_data_vld_o,
    output wire [1023:0]                udi_rx_link1_lane3_data_o,
    output wire                         udi_rx_link1_lane3_data_vld_o,
`endif
`endif
`ifdef TX_LINK2_EXIST
    input  wire [1023:0]                udi_tx_link2_lane0_data_i,
    input  wire                         udi_tx_link2_lane0_data_vld_i,
    output wire                         udi_tx_link2_ready_o,
`ifdef LINK_LANE1
    input  wire [1023:0]                udi_tx_link2_lane1_data_i,
    input  wire                         udi_tx_link2_lane1_data_vld_i,
`endif
`ifdef LINK_LANE3
    input  wire [1023:0]                udi_tx_link2_lane2_data_i,
    input  wire                         udi_tx_link2_lane2_data_vld_i,
    input  wire [1023:0]                udi_tx_link2_lane3_data_i,
    input  wire                         udi_tx_link2_lane3_data_vld_i,
`endif
`endif
`ifdef RX_LINK2_EXIST
    output wire [1023:0]                udi_rx_link2_lane0_data_o,
    output wire                         udi_rx_link2_lane0_data_vld_o,
`ifdef LINK_LANE1
    output wire [1023:0]                udi_rx_link2_lane1_data_o,
    output wire                         udi_rx_link2_lane1_data_vld_o,
`endif
`ifdef LINK_LANE3
    output wire [1023:0]                udi_rx_link2_lane2_data_o,
    output wire                         udi_rx_link2_lane2_data_vld_o,
    output wire [1023:0]                udi_rx_link2_lane3_data_o,
    output wire                         udi_rx_link2_lane3_data_vld_o,
`endif
`endif
`ifdef TX_LINK3_EXIST
    input  wire [1023:0]                udi_tx_link3_lane0_data_i,
    input  wire                         udi_tx_link3_lane0_data_vld_i,
    output wire                         udi_tx_link3_ready_o,
`ifdef LINK_LANE1
    input  wire [1023:0]                udi_tx_link3_lane1_data_i,
    input  wire                         udi_tx_link3_lane1_data_vld_i,
`endif
`ifdef LINK_LANE3
    input  wire [1023:0]                udi_tx_link3_lane2_data_i,
    input  wire                         udi_tx_link3_lane2_data_vld_i,
    input  wire [1023:0]                udi_tx_link3_lane3_data_i,
    input  wire                         udi_tx_link3_lane3_data_vld_i,
`endif
`endif
`ifdef RX_LINK3_EXIST
    output wire [1023:0]                udi_rx_link3_lane0_data_o,
    output wire                         udi_rx_link3_lane0_data_vld_o,
`ifdef LINK_LANE1
    output wire [1023:0]                udi_rx_link3_lane1_data_o,
    output wire                         udi_rx_link3_lane1_data_vld_o,
`endif
`ifdef LINK_LANE3
    output wire [1023:0]                udi_rx_link3_lane2_data_o,
    output wire                         udi_rx_link3_lane2_data_vld_o,
    output wire [1023:0]                udi_rx_link3_lane3_data_o,
    output wire                         udi_rx_link3_lane3_data_vld_o,
`endif
`endif

    // =========================================================================
    // PAIF 控制接口（L2 §2.5，sb_clk）
    // =========================================================================
    input  wire [3:0]                   lp_sw_cmd_i,
    input  wire                         lp_sw_req_i,
    output wire                         lp_sw_ack_o,
    output wire [3:0]                   lp_sw_sts_o,
    input  wire                         bridge_en_i,              // public CSR bridge_en（异步输入，顶层同步分发）

    // =========================================================================
    // 事件接口（L2 §2.6，sb_clk）
    // =========================================================================
    output wire                         cpif_tx_event_enable_o,
    output wire                         cpif_tx_event_req_o,
    output wire [15:0]                  cpif_tx_event_cmd_o,
    input  wire                         cpif_tx_event_ack_i,
    input  wire [7:0]                   cpif_tx_event_status_i,
    output wire                         cpif_rx_event_enable_o,
    input  wire                         cpif_rx_event_req_i,
    input  wire [15:0]                  cpif_rx_event_cmd_i,
    output wire                         cpif_rx_event_ack_o,
    output wire [7:0]                   cpif_rx_event_status_o,

    // =========================================================================
    // APB 总线接口（L2 §2.7，apb_clk；pclk/presetn 即 apb_clk/apb_rst_n）
    // =========================================================================
    input  wire [15:0]                  paddr_i,
    input  wire                         psel_i,
    input  wire                         penable_i,
    input  wire                         pwrite_i,
    input  wire [31:0]                  pwdata_i,
    output wire                         pready_o,
    output wire [31:0]                  prdata_o,
    output wire                         pslverr_o,

    // =========================================================================
    // DFx / 中断接口（L2 §2.8，apb_clk）
    // =========================================================================
    output wire                         irq_o,
    output wire [31:0]                  dfx_stat_o
);

    // =========================================================================
    // 内部信号声明（全部 wire / reg 集中本区，wire 与 assign 分离）
    // =========================================================================

    // ---- apb_csr 配置输出（apb_clk 域原始信号） --------------------------------
    wire        csr_cfg_done_apb;            // INIT_CFG_REG.CONFIG_DONE 置位沿脉冲
    wire        csr_perf_mode_apb;           // 性能模式（work_mode 同值）
    wire        s2p_mode_apb;            // PHY_MODE.s2p_mode
    wire        protocol_mode_apb;       // MODE_REG.PROTOCOL_MODE
    wire        csr_udi_ready_active_en_apb;   // MODE_REG.UDI_READY_ACTIVE_EN（apb 域）
    wire [1:0]  csr_bw_mode_apb;             // BW_CFG_REG.bw_mode
    wire        csr_loopback_en_apb;         // LOOPBACK_CFG_REG.LOOPBACK_EN
    wire [15:0] csr_loopback_src_apb;        // LOOPBACK_CFG_REG.RX0~3_SRC（4×4 bit）
    wire [15:0] csr_ack_route_src_en_apb;    // ACK_ROUTE_CFG_REG.TX0~3_SRC_EN（4×4 bit）
    wire        csr_retry_en_apb;            // RETRY_CFG_REG.RETRY_EN
    wire        csr_credit_en_apb;           // RETRY_CFG_REG.CREDIT_EN
    wire        csr_crc_en_apb;              // RETRY_CFG_REG.CRC_EN
    wire        csr_ecc_en_apb;              // RETRY_CFG_REG.ECC_EN
    wire [1:0]  csr_nack_mode_apb;           // RETRY_CFG_REG.NACK_MODE
    // ---- 动态时钟门控寄存器输出（apb_clk 域，CLK_CFG_REG/CLK_CFG_TX_REG/CLK_CFG_RX_REG，reset 全 0）----
    wire        csr_clk_cfg_sram_dyn_en_apb;  // CLK_CFG_REG.SRAM_DYN_EN（bit2）：SRAM 动态时钟开关
    wire        csr_clk_cfg_tx0_dyn_apb;      // CLK_CFG_TX_REG.TX0_DYN（bit0）：link_tx[0] 每路掩码
    wire        csr_clk_cfg_tx1_dyn_apb;      // CLK_CFG_TX_REG.TX1_DYN（bit1）
    wire        csr_clk_cfg_tx2_dyn_apb;      // CLK_CFG_TX_REG.TX2_DYN（bit2）
    wire        csr_clk_cfg_tx3_dyn_apb;      // CLK_CFG_TX_REG.TX3_DYN（bit3）
    wire        csr_clk_cfg_rx0_dyn_apb;      // CLK_CFG_RX_REG.RX0_DYN（bit0）：link_rx[0] 每路掩码
    wire        csr_clk_cfg_rx1_dyn_apb;      // CLK_CFG_RX_REG.RX1_DYN（bit1）
    wire        csr_clk_cfg_rx2_dyn_apb;      // CLK_CFG_RX_REG.RX2_DYN（bit2）
    wire        csr_clk_cfg_rx3_dyn_apb;      // CLK_CFG_RX_REG.RX3_DYN（bit3）
`ifdef TX_LINK1_EXIST
    wire        inj_en_link1_apb;
    wire        inj_fb_link1_apb;
`endif
`ifdef TX_LINK2_EXIST
    wire        inj_en_link2_apb;
    wire        inj_fb_link2_apb;
`endif
`ifdef TX_LINK3_EXIST
    wire        inj_en_link3_apb;
    wire        inj_fb_link3_apb;
`endif
    wire        csr_tx_flit_clr_apb;
    wire        csr_rx_flit_clr_apb;
    wire        csr_crc_err_clr_apb;
`ifdef TX_LINK1_EXIST
    wire        csr_tx1_cnt_clr_apb;
`endif
`ifdef TX_LINK2_EXIST
    wire        csr_tx2_cnt_clr_apb;
`endif
`ifdef TX_LINK3_EXIST
    wire        csr_tx3_cnt_clr_apb;
`endif
    wire [15:0] drift_period_apb;
    wire [15:0] retry_timeout_val_apb;
    wire [7:0]  retrain_timeout_thr_apb;
    wire [7:0]  retrain_nack_thr_apb;
    wire        retrain_timeout_en_apb;
    wire        retrain_nack_en_apb;
    wire        init_phy_init_apb;       // INIT_CFG_REG.PHY_INIT 置位沿脉冲（独立于 CONFIG_DONE）
    wire        init_retrain_apb;        // INIT_CFG_REG.RETRAIN 置位沿脉冲（软件触发训练，→ mb_clk 两 lcmsm）
    wire        init_lane_on_apb;        // 置位沿脉冲
    wire        init_lane_off_apb;
    wire        init_rx_ready_apb;
    wire [5:0]  lp_cmd_apb;              // LP_CFG_REG 6 路置位沿脉冲

    // =========================================================================
    // 跨时钟域同步声明（全部 sync_cell_norst 例化（无复位 2 级打拍），接收侧；多 bit 逐 bit 例化）
    // 域划分：数据通路 mb_clk（link_tx / link_rx / 两 lcmsm）；
    //        事件/控制 sb_clk（link_event_process + PAIF 控制）；
    //        配置 apb_clk（apb_csr）
    // =========================================================================

    wire        csr_perf_mode_mb_sync;
    wire        s2p_mode_mb_sync;
    wire        protocol_mode_mb_sync;
    wire        csr_udi_ready_active_en_mb_sync;
    wire        csr_loopback_en_mb_sync;
    wire [15:0] csr_loopback_src_mb_sync;
    wire [15:0] csr_ack_route_src_en_mb_sync;
    wire [1:0]  csr_bw_mode_mb_sync;
    wire [1:0]  csr_nack_mode_mb_sync;
    wire        retry_en_mb_sync;
    wire        credit_en_mb_sync;
    wire        crc_en_mb_sync;
    wire        ecc_en_mb_sync;
    wire        retrain_toen_mb_sync;
    wire        retrain_nacken_mb_sync;
    wire        bridge_mb_sync;
    wire [15:0] drift_period_mb_sync;
    wire [15:0] retry_toval_mb_sync;
    wire [7:0]  retrain_tothr_mb_sync;
    wire [7:0]  retrain_nackthr_mb_sync;
    wire        csr_comp_mode_mb;            // = ~csr_perf_mode_mb_sync
    assign csr_comp_mode_mb = ~csr_perf_mode_mb_sync;
    wire        inj_en_link0_mb_sync;
    wire        inj_fb_link0_mb_sync;
`ifdef TX_LINK1_EXIST
    wire        inj_en_link1_mb_sync;
    wire        inj_fb_link1_mb_sync;
`endif
`ifdef TX_LINK2_EXIST
    wire        inj_en_link2_mb_sync;
    wire        inj_fb_link2_mb_sync;
`endif
`ifdef TX_LINK3_EXIST
    wire        inj_en_link3_mb_sync;
    wire        inj_fb_link3_mb_sync;
`endif
    // ---- 动态时钟门控：CLK_CFG/CLK_CFG_TX/CLK_CFG_RX 寄存器同步（apb_clk → mb_clk）----
    // CLK_CFG_REG.SRAM_DYN_EN（bit2）+ CLK_CFG_TX_REG[n]/CLK_CFG_RX_REG[n] 每路独立开关（TX/RX 总开关已删）
    // reset 全 0 = always-on（各门控使能恒 1 → 门控时钟=mb_clk，验收基线零回归）
    wire        csr_clk_cfg_sram_dyn_en_mb_sync;
    wire        csr_clk_cfg_tx0_dyn_mb_sync;
    wire        csr_clk_cfg_tx1_dyn_mb_sync;
    wire        csr_clk_cfg_tx2_dyn_mb_sync;
    wire        csr_clk_cfg_tx3_dyn_mb_sync;
    wire        csr_clk_cfg_rx0_dyn_mb_sync;
    wire        csr_clk_cfg_rx1_dyn_mb_sync;
    wire        csr_clk_cfg_rx2_dyn_mb_sync;
    wire        csr_clk_cfg_rx3_dyn_mb_sync;

    // ---- apb_clk → mb_clk：脉冲（源域 toggle + sync_cell_norst + 边沿检测） --------------
    wire        csr_cfg_done_mb;         // mb 域电平（→ 两 lcmsm，采电平不做沿检测）

    // ---- apb_clk → sb_clk：寄存器通路事件触发脉冲（event_process） ------------------
    reg         csr_init_phy_init_tog_apb;
    reg         csr_init_lane_on_tog_apb;
    reg         csr_init_lane_off_tog_apb;
    reg         csr_init_rx_ready_tog_apb;
    reg  [5:0]  csr_lp_cmd_tog_apb;
    wire        init_phy_init_sb_sync;
    wire        init_lane_on_sb_sync;
    wire        init_lane_off_sb_sync;
    wire        init_rx_ready_sb_sync;
    wire [5:0]  lp_cmd_sb_sync;
    reg         init_phy_init_sb_d;
    reg         init_lane_on_sb_d;
    reg         init_lane_off_sb_d;
    reg         init_rx_ready_sb_d;
    reg  [5:0]  lp_cmd_sb_d;
    wire        csr_init_phy_init_sb_pos;
    wire        csr_init_lane_on_sb_pos;
    wire        csr_init_lane_off_sb_pos;
    wire        csr_init_rx_ready_sb_pos;
    wire [5:0]  csr_lp_cmd_sb_pos;

    // ---- sb_clk → mb_clk：lcmsm 输入（event_process 脉冲 + PAIF 控制电平） ----------
    wire        evt_done_mb_sync;
    wire        rx_evt_lp_enter_mb_sync;
    wire        rx_evt_lp_exit_mb_sync;
    reg         evt_done_mb_d;
    reg         rx_evt_lp_enter_mb_d;
    reg         rx_evt_lp_exit_mb_d;
    wire        evt_done_mb_pos;             // mb 域 1 cycle 脉冲
    wire        rx_evt_lp_enter_mb_pos;
    wire        rx_evt_lp_exit_mb_pos;
    wire        lp_sw_req_mb_sync;       // PAIF 控制电平（顶层引脚 sb 域）
    wire [3:0]  lp_sw_cmd_mb_sync;
    // CDC 修复(Ac_conv02 46FD)：PAIF LP 命令单次捕获（见 sb→mb 段代码注释）
    reg         lp_sw_req_rise_d;       // lp_sw_req_mb_sync 1 拍延迟（上升沿检测）
    wire        lp_sw_req_rise;         // lp_sw_req 上升沿（mb 域 1 拍）
    reg  [3:0]  lp_sw_cmd_cap;          // 单次采样捕获的命令（req 上升沿装载）
    reg         lp_sw_cmd_vld;          // 捕获有效（req 高期间为 1）

    // ---- mb_clk → sb_clk：lcmsm 事件请求（→ event_process） ------------------------
    reg         tx_evt_req_tog_mb;       // mb 域 toggle（req 为脉冲，cmd 为随同电平）
    reg         rx_evt_req_tog_mb;
    wire        tx_evt_req_sb_sync;
    wire        rx_evt_req_sb_sync;
    reg         tx_evt_req_sb_d;
    reg         rx_evt_req_sb_d;
    wire        tx_lcsm_evt_req_sb_pos;      // sb 域 1 cycle 脉冲
    wire        rx_lcsm_evt_req_sb_pos;
    wire [15:0] tx_lcsm_evt_cmd_sb;      // cmd 为电平（req 前置稳定），sync_cell_norst 逐 bit
    wire [15:0] rx_lcsm_evt_cmd_sb;

    // ---- apb_clk → mb_clk：脉冲同步（init_retrain，toggle + sync_cell_norst + 边沿检测） ----
    reg         init_retrain_tog_apb;
    wire        init_retrain_mb_sync;
    reg         init_retrain_mb_d;
    wire        init_retrain_mb_pos;      // mb 域 1 cycle 脉冲

    // ---- apb_clk → mb_clk：RC 读后清零脉冲同步（toggle + sync_cell_norst + 边沿检测） ----
    reg         tx_flit_clr_tog_apb;
    reg         rx_flit_clr_tog_apb;
    reg         crc_err_clr_tog_apb;
`ifdef TX_LINK1_EXIST
    reg         tx1_cnt_clr_tog_apb;
`endif
`ifdef TX_LINK2_EXIST
    reg         tx2_cnt_clr_tog_apb;
`endif
`ifdef TX_LINK3_EXIST
    reg         tx3_cnt_clr_tog_apb;
`endif
    wire        tx_flit_clr_mb_sync;
    wire        rx_flit_clr_mb_sync;
    wire        crc_err_clr_mb_sync;
`ifdef TX_LINK1_EXIST
    wire        tx1_cnt_clr_mb_sync;
`endif
`ifdef TX_LINK2_EXIST
    wire        tx2_cnt_clr_mb_sync;
`endif
`ifdef TX_LINK3_EXIST
    wire        tx3_cnt_clr_mb_sync;
`endif
    reg         tx_flit_clr_mb_d;
    reg         rx_flit_clr_mb_d;
    reg         crc_err_clr_mb_d;
`ifdef TX_LINK1_EXIST
    reg         tx1_cnt_clr_mb_d;
`endif
`ifdef TX_LINK2_EXIST
    reg         tx2_cnt_clr_mb_d;
`endif
`ifdef TX_LINK3_EXIST
    reg         tx3_cnt_clr_mb_d;
`endif
    wire        tx_flit_clr_mb_pos;
    wire        rx_flit_clr_mb_pos;
    wire        crc_err_clr_mb_pos;
`ifdef TX_LINK1_EXIST
    wire        tx1_cnt_clr_mb_pos;
`endif
`ifdef TX_LINK2_EXIST
    wire        tx2_cnt_clr_mb_pos;
`endif
`ifdef TX_LINK3_EXIST
    wire        tx3_cnt_clr_mb_pos;
`endif

    // ---- mb_clk → apb_clk：状态/计数回读（sync_cell_norst 逐 bit） -------------------------
    wire        replay_buf_full_apb_sync;
    wire        retrain_toflag_apb_sync;
    wire        retrain_nackflag_apb_sync;
    wire [15:0] tx_flit_cnt_apb_sync;
    wire [7:0]  timeout_cnt_apb_sync;
    wire [15:0] rx_flit_cnt_apb_sync;
    wire [15:0] crc_err_cnt_apb_sync;
    wire        irq_retry_to_apb_sync;
    wire        irq_retry_nack_apb_sync;
    wire [3:0]  link_state_tx_apb_sync;  // lcmsm 状态（mb 域）→ apb
    wire [3:0]  link_state_rx_apb_sync;
    wire [1:0]  lp_state_tx_apb_sync;
    wire [1:0]  lp_state_rx_apb_sync;
    wire        phy_ready_mb;            // cpif_tx_ready & cpif_rx_ready（mb_clk 域相与，电气初始化完成，G-01）
    wire        phy_ready_apb_sync;      // phy_ready_mb 经 2flop 同步到 apb_clk（→ STATUS_REG.PHY_READY）
    assign phy_ready_mb = cpif_tx_link0_ready_i & cpif_rx_link0_ready_i;
    wire        cpif_rx_lpc_apb_sync;    // cpif_rx_lpc_i 经 2flop 同步到 apb_clk（→ STATUS_REG.RX_LPC）
    // ---- 额外 TX link（1~3）状态/计数回读（mb → apb 逐拍同步）---------
`ifdef TX_LINK1_EXIST
    wire        tx1_replay_buf_full_apb_sync;
    wire        tx1_retrain_toflag_apb_sync;
    wire        tx1_retrain_nackflag_apb_sync;
    wire        tx1_irq_retry_to_apb_sync;
    wire        tx1_irq_retry_nack_apb_sync;
    wire [15:0] tx1_flit_cnt_apb_sync;
    wire [7:0]  tx1_timeout_cnt_apb_sync;
`endif
`ifdef TX_LINK2_EXIST
    wire        tx2_replay_buf_full_apb_sync;
    wire        tx2_retrain_toflag_apb_sync;
    wire        tx2_retrain_nackflag_apb_sync;
    wire        tx2_irq_retry_to_apb_sync;
    wire        tx2_irq_retry_nack_apb_sync;
    wire [15:0] tx2_flit_cnt_apb_sync;
    wire [7:0]  tx2_timeout_cnt_apb_sync;
`endif
`ifdef TX_LINK3_EXIST
    wire        tx3_replay_buf_full_apb_sync;
    wire        tx3_retrain_toflag_apb_sync;
    wire        tx3_retrain_nackflag_apb_sync;
    wire        tx3_irq_retry_to_apb_sync;
    wire        tx3_irq_retry_nack_apb_sync;
    wire [15:0] tx3_flit_cnt_apb_sync;
    wire [7:0]  tx3_timeout_cnt_apb_sync;
`endif
    // 脉冲 mb→apb：toggle + sync_cell_norst + 边沿检测
    reg         credit_ovf_tog_mb;
    reg         seq_mis_tog_mb;
    reg         train_fail_tog_mb;
    wire        credit_ovf_apb_sync;
    wire        seq_mis_apb_sync;
    wire        train_fail_apb_sync;
    reg         credit_ovf_apb_d;
    reg         seq_mis_apb_d;
    reg         train_fail_apb_d;
    wire        credit_ovf_apb_pos;
    wire        seq_mis_apb_pos;
    wire        train_fail_apb_pos;

    // ---- sb_clk → apb_clk：lp_transition 脉冲（event_process 中断） -----------------
    reg         lp_trans_tog_sb;
    wire        lp_trans_apb_sync;
    reg         lp_trans_apb_d;
    wire        lp_trans_apb_pos;
    // ---- sb_clk → apb_clk：初始化流程中断脉冲（G-03/G-04，event_process 中断） ----
    reg         irq_rx_ready_tog_sb;    // 源域 toggle（TX 侧收对端 RX READY 0x7F）
    wire        irq_rx_ready_apb_sync;
    reg         irq_rx_ready_apb_d;
    wire        irq_rx_ready_apb_pos;   // apb 域 1 cycle 脉冲（→ IRQ_STATUS_REG.RX_READY_IRQ）
    reg         irq_lane_on_done_tog_sb;// 源域 toggle（TX 侧收 LANE ON 完成 0x40 返回）
    wire        irq_lane_on_done_apb_sync;
    reg         irq_lane_on_done_apb_d;
    wire        irq_lane_on_done_apb_pos;// apb 域 1 cycle 脉冲（→ IRQ_STATUS_REG.LANE_ON_DONE_IRQ）

    // ---- 跨域直连接口 ---------------------------------------------------------------
    wire        tx0_deskew_done;         // link_tx0 → tx_lcmsm（mb 域直连）

    // ---- link_tx0 / link_rx0 实例互连（mb_clk） -----------------------------------
    wire [1023:0] tx0_cpif_lane0_data;
    wire          tx0_cpif_lane0_vld;
    // ---- link_rx0 loopback 源选择 mux 输出（RX0_SRC = csr_loopback_src_mb_sync[3:0]）----
    reg  [1023:0] lb_rx0_lane0_data;     // loopback 源选择 mux 输出（always case）
    reg           lb_rx0_lane0_vld;
`ifdef LINK_LANE1
    reg  [1023:0] lb_rx0_lane1_data;
    reg           lb_rx0_lane1_vld;
`endif
    wire [7:0]    rx0_nack_seq;
    wire          rx0_nack_valid;
    wire [7:0]    rx0_ack_seq;
    wire          rx0_ack_valid;
    wire [1:0]    rx0_dlp_linknum;
    wire [7:0]    rx0_dlv_ack_seq;
    wire          rx0_dlv_ack_valid;
    wire [7:0]    rx0_lcl_nack_seq;
    wire          rx0_lcl_nack_valid;
    wire [1:0]    rx0_lcl_nack_type;
    // ---- link_ack_route 输出 net（每 TX 一份，routeN → link_txN）----
    wire [7:0]    route0_dlv_ack_seq;
    wire          route0_dlv_ack_valid;
    wire [7:0]    route0_lcl_nack_seq;
    wire          route0_lcl_nack_valid;
    wire [1:0]    route0_lcl_nack_type;
    wire [1:0]    route0_sel_idx;
    wire [7:0]    route0_ack_seq;
    wire          route0_ack_valid;
    wire [7:0]    route0_nack_seq;
    wire          route0_nack_valid;
    wire [1:0]    route0_nack_type;
`ifdef TX_LINK1_EXIST
    wire [7:0]    route1_dlv_ack_seq;
    wire          route1_dlv_ack_valid;
    wire [7:0]    route1_lcl_nack_seq;
    wire          route1_lcl_nack_valid;
    wire [1:0]    route1_lcl_nack_type;
    wire [1:0]    route1_sel_idx;
    wire [7:0]    route1_ack_seq;
    wire          route1_ack_valid;
    wire [7:0]    route1_nack_seq;
    wire          route1_nack_valid;
    wire [1:0]    route1_nack_type;
`endif
`ifdef TX_LINK2_EXIST
    wire [7:0]    route2_dlv_ack_seq;
    wire          route2_dlv_ack_valid;
    wire [7:0]    route2_lcl_nack_seq;
    wire          route2_lcl_nack_valid;
    wire [1:0]    route2_lcl_nack_type;
    wire [1:0]    route2_sel_idx;
    wire [7:0]    route2_ack_seq;
    wire          route2_ack_valid;
    wire [7:0]    route2_nack_seq;
    wire          route2_nack_valid;
    wire [1:0]    route2_nack_type;
`endif
`ifdef TX_LINK3_EXIST
    wire [7:0]    route3_dlv_ack_seq;
    wire          route3_dlv_ack_valid;
    wire [7:0]    route3_lcl_nack_seq;
    wire          route3_lcl_nack_valid;
    wire [1:0]    route3_lcl_nack_type;
    wire [1:0]    route3_sel_idx;
    wire [7:0]    route3_ack_seq;
    wire          route3_ack_valid;
    wire [7:0]    route3_nack_seq;
    wire          route3_nack_valid;
    wire [1:0]    route3_nack_type;
`endif
    wire [255:0]  rx0_arq_req_flit;      // link_rx0 → link_tx0 ARQ(Request) payload 中继
    wire          rx0_arq_req_valid;
    wire [15:0]   rx0_arq_req_rcv_ptr;   // link_rx0 → link_tx0 远端 ARQ(Request) 指示
    wire          rx0_arq_req_rcv_valid;
    wire [1:0]    nack_type_mb;          // NACK_MODE 映射：01→2'b11(Select) / 其他→2'b10(Standard)
    wire          tx0_replay_buf_full;
    wire          tx0_retrain_to_flag;
    wire          tx0_retrain_nack_flag;
    wire          tx0_irq_retry_to;
    wire          tx0_irq_retry_nack;
    wire [1:0]    tx0_retrain_req;
    wire [15:0]   tx0_flit_cnt;
    wire [7:0]    tx0_timeout_cnt;
`ifdef TX_LINK1_EXIST
    wire          tx1_deskew_done;          // link_tx1 → tx_lcmsm（mb 域直连，与 tx0 相与）
    wire [1:0]    tx1_retrain_req;
    wire          tx1_replay_buf_full;
    wire          tx1_retrain_to_flag;
    wire          tx1_retrain_nack_flag;
    wire          tx1_irq_retry_to;
    wire          tx1_irq_retry_nack;
    wire [15:0]   tx1_flit_cnt;
    wire [7:0]    tx1_timeout_cnt;
`endif
`ifdef TX_LINK2_EXIST
    wire          tx2_deskew_done;
    wire [1:0]    tx2_retrain_req;
    wire          tx2_replay_buf_full;
    wire          tx2_retrain_to_flag;
    wire          tx2_retrain_nack_flag;
    wire          tx2_irq_retry_to;
    wire          tx2_irq_retry_nack;
    wire [15:0]   tx2_flit_cnt;
    wire [7:0]    tx2_timeout_cnt;
`endif
`ifdef TX_LINK3_EXIST
    wire          tx3_deskew_done;
    wire [1:0]    tx3_retrain_req;
    wire          tx3_replay_buf_full;
    wire          tx3_retrain_to_flag;
    wire          tx3_retrain_nack_flag;
    wire          tx3_irq_retry_to;
    wire          tx3_irq_retry_nack;
    wire [15:0]   tx3_flit_cnt;
    wire [7:0]    tx3_timeout_cnt;
`endif
    wire          rx0_train_done;          // link_rx0 → link_rx_lcmsm（功能，mb 域直连）
    wire          rx0_train_fail;          // link_rx0 → train_fail 脉冲 → irq_dskew_fail（功能）
    wire          rx0_credit_ovf;
    wire          rx0_seq_mis;
    wire [15:0]   rx0_flit_cnt;
    wire [15:0]   rx0_crc_err_cnt;

    // ---- 共享 lcmsm 聚合：deskew_done 取与（所有 link 对齐才跳转），retrain_req 取或 -------------
    wire          tx_lcsm_deskew_done;
    wire          tx_lcsm_retrain_to_req;
    wire          tx_lcsm_retrain_nack_req;
`ifdef LINK_LANE1
    wire [1023:0] tx0_cpif_lane1_data;
    wire          tx0_cpif_lane1_vld;
`endif
`ifdef LINK_LANE3
    wire [1023:0] tx0_cpif_lane2_data;
    wire          tx0_cpif_lane2_vld;
    wire [1023:0] tx0_cpif_lane3_data;
    wire          tx0_cpif_lane3_vld;
`endif

    // ---- 多 link 互连（非对称结构，ifdef 展开） -------------------------------------
`ifdef TX_LINK1_EXIST
    wire [1023:0] tx1_cpif_lane0_data;
    wire          tx1_cpif_lane0_vld;
`ifdef LINK_LANE1
    wire [1023:0] tx1_cpif_lane1_data;
    wire          tx1_cpif_lane1_vld;
`endif
`endif
`ifdef TX_LINK2_EXIST
    wire [1023:0] tx2_cpif_lane0_data;
    wire          tx2_cpif_lane0_vld;
`ifdef LINK_LANE1
    wire [1023:0] tx2_cpif_lane1_data;
    wire          tx2_cpif_lane1_vld;
`endif
`endif
`ifdef TX_LINK3_EXIST
    wire [1023:0] tx3_cpif_lane0_data;
    wire          tx3_cpif_lane0_vld;
`ifdef LINK_LANE1
    wire [1023:0] tx3_cpif_lane1_data;
    wire          tx3_cpif_lane1_vld;
`endif
`endif
`ifdef RX_LINK1_EXIST
    reg  [1023:0] lb_rx1_lane0_data;     // loopback 源选择 mux 输出（always case）
    reg           lb_rx1_lane0_vld;
`ifdef LINK_LANE1
    reg  [1023:0] lb_rx1_lane1_data;
    reg           lb_rx1_lane1_vld;
`endif
`endif
`ifdef RX_LINK2_EXIST
    reg  [1023:0] lb_rx2_lane0_data;
    reg           lb_rx2_lane0_vld;
`ifdef LINK_LANE1
    reg  [1023:0] lb_rx2_lane1_data;
    reg           lb_rx2_lane1_vld;
`endif
`endif
`ifdef RX_LINK3_EXIST
    reg  [1023:0] lb_rx3_lane0_data;
    reg           lb_rx3_lane0_vld;
`ifdef LINK_LANE1
    reg  [1023:0] lb_rx3_lane1_data;
    reg           lb_rx3_lane1_vld;
`endif
`endif

    // ---- lcmsm / event_process 互连（sb_clk） --------------------------------------
    wire [3:0]  tx_lcsm_state;
    wire        tx_lcsm_active;   // link_tx_lcsm.link_active_o（S_NORMAL 使能，mb_clk 域直连）
    wire        tx_lcsm_active_nxt; // link_tx_lcsm.link_active_nxt_o（S_NORMAL 次态位，F1a link_tx link_active_cp D 源，mb_clk 域直连）
    wire [1:0]  tx_lp_state;
    wire        tx_lcsm_evt_req;
    wire [15:0] tx_lcsm_evt_cmd;
    wire [3:0]  rx_lcsm_state;
    wire        rx_path_en;
    wire        rx_train_start;
    wire [1:0]  rx_lp_state;
    wire        rx_lcsm_evt_req;
    wire [15:0] rx_lcsm_evt_cmd;
    wire        evt_done;
    wire        rx_evt_lp_enter;
    wire        rx_evt_lp_exit;
    wire        irq_lp_trans_sb;
    wire        irq_rx_ready_sb;          // event_process → sb 域中断脉冲（TX 侧收 0x7F，G-03）
    wire        irq_lane_on_done_sb;      // event_process → sb 域中断脉冲（TX 侧收 0x40 完成，G-04）


    // =========================================================================
    // apb_csr 例化（apb_clk 域；集中式 CSR，L2 §5）
    // =========================================================================
    apb_csr u_apb_csr (
        .apb_clk                    (apb_clk),
        .apb_rst_n                  (apb_rst_n),
        .psel_i                     (psel_i),
        .penable_i                  (penable_i),
        .pwrite_i                   (pwrite_i),
        .paddr_i                    (paddr_i),
        .pwdata_i                   (pwdata_i),
        .pready_o                   (pready_o),
        .prdata_o                   (prdata_o),
        .pslverr_o                  (pslverr_o),
        .csr_cfg_done_o(csr_cfg_done_apb),
        .csr_perf_mode_o(csr_perf_mode_apb),
        .csr_s2p_mode_o             (s2p_mode_apb),
        .csr_protocol_mode_o        (protocol_mode_apb),
        .csr_udi_ready_active_en_o (csr_udi_ready_active_en_apb),
        .csr_bw_mode_o(csr_bw_mode_apb),
        .csr_loopback_en_o(csr_loopback_en_apb),
        .csr_loopback_src_o(csr_loopback_src_apb),
        .csr_ack_route_src_en_o(csr_ack_route_src_en_apb),
        .csr_retry_en_o(csr_retry_en_apb),
        .csr_credit_en_o(csr_credit_en_apb),
        .csr_crc_en_o(csr_crc_en_apb),
        .csr_ecc_en_o(csr_ecc_en_apb),
        .csr_nack_mode_o(csr_nack_mode_apb),
        .csr_clk_cfg_sram_dyn_en_o   (csr_clk_cfg_sram_dyn_en_apb),
        .csr_clk_cfg_tx0_dyn_o       (csr_clk_cfg_tx0_dyn_apb),
        .csr_clk_cfg_tx1_dyn_o       (csr_clk_cfg_tx1_dyn_apb),
        .csr_clk_cfg_tx2_dyn_o       (csr_clk_cfg_tx2_dyn_apb),
        .csr_clk_cfg_tx3_dyn_o       (csr_clk_cfg_tx3_dyn_apb),
        .csr_clk_cfg_rx0_dyn_o       (csr_clk_cfg_rx0_dyn_apb),
        .csr_clk_cfg_rx1_dyn_o       (csr_clk_cfg_rx1_dyn_apb),
        .csr_clk_cfg_rx2_dyn_o       (csr_clk_cfg_rx2_dyn_apb),
        .csr_clk_cfg_rx3_dyn_o       (csr_clk_cfg_rx3_dyn_apb),
        .csr_inj_en_link0_o         (inj_en_link0_apb),
        .csr_inj_fb_link0_o         (inj_fb_link0_apb),
`ifdef TX_LINK1_EXIST
        .csr_inj_en_link1_o         (inj_en_link1_apb),
        .csr_inj_fb_link1_o         (inj_fb_link1_apb),
`endif
`ifdef TX_LINK2_EXIST
        .csr_inj_en_link2_o         (inj_en_link2_apb),
        .csr_inj_fb_link2_o         (inj_fb_link2_apb),
`endif
`ifdef TX_LINK3_EXIST
        .csr_inj_en_link3_o         (inj_en_link3_apb),
        .csr_inj_fb_link3_o         (inj_fb_link3_apb),
`endif
        .csr_tx_flit_clr_o          (csr_tx_flit_clr_apb),
        .csr_rx_flit_clr_o          (csr_rx_flit_clr_apb),
        .csr_crc_err_clr_o          (csr_crc_err_clr_apb),
`ifdef TX_LINK1_EXIST
        .csr_tx1_cnt_clr_o          (csr_tx1_cnt_clr_apb),
`endif
`ifdef TX_LINK2_EXIST
        .csr_tx2_cnt_clr_o          (csr_tx2_cnt_clr_apb),
`endif
`ifdef TX_LINK3_EXIST
        .csr_tx3_cnt_clr_o          (csr_tx3_cnt_clr_apb),
`endif
        .csr_drift_period_o         (drift_period_apb),
        .csr_retry_timeout_val_o    (retry_timeout_val_apb),
        .csr_retrain_timeout_thr_o  (retrain_timeout_thr_apb),
        .csr_retrain_nack_thr_o     (retrain_nack_thr_apb),
        .csr_retrain_timeout_en_o   (retrain_timeout_en_apb),
        .csr_retrain_nack_en_o      (retrain_nack_en_apb),
        .csr_init_phy_init_o(init_phy_init_apb),
        .csr_init_retrain_o(init_retrain_apb),
        .csr_init_lane_on_o(init_lane_on_apb),
        .csr_init_lane_off_o(init_lane_off_apb),
        .csr_init_rx_ready_o(init_rx_ready_apb),
        .csr_lp_cmd_o(lp_cmd_apb),
        .cpif_tx_mode_o             (cpif_tx_mode_o),
        .cpif_rx_mode_o             (cpif_rx_mode_o),
        .cpif_tx_rst_n_o            (cpif_tx_rst_n_o),
        .cpif_tx_lpc_o              (cpif_tx_lpc_o),
        .cpif_rx_rst_n_o            (cpif_rx_rst_n_o),
        .link_fsm_state_tx_i        (link_state_tx_apb_sync),
        .bridge_en_i                (bridge_en_i),
        .phy_ready_i                (phy_ready_apb_sync),
        .status_reg_rx_lpc_i        (cpif_rx_lpc_apb_sync),
        .link_fsm_state_rx_i        (link_state_rx_apb_sync),
        .lp_state_tx_i              (lp_state_tx_apb_sync),
        .lp_state_rx_i              (lp_state_rx_apb_sync),
        .retrain_timeout_flag_i     (retrain_toflag_apb_sync),
        .retrain_nack_flag_i        (retrain_nackflag_apb_sync),
        .replay_buf_full_i          (replay_buf_full_apb_sync),
        .tx_flit_cnt_i              (tx_flit_cnt_apb_sync),
        .rx_flit_cnt_i              (rx_flit_cnt_apb_sync),
        .crc_err_cnt_i              (crc_err_cnt_apb_sync),
        .timeout_cnt_i              (timeout_cnt_apb_sync),
        .irq_credit_overflow_i      (credit_ovf_apb_pos),
        .irq_retry_timeout_i        (irq_retry_to_apb_sync),
        .irq_retry_nack_i           (irq_retry_nack_apb_sync),
        .irq_lp_transition_i        (lp_trans_apb_pos),
        .irq_seq_mismatch_i         (seq_mis_apb_pos),
        .irq_dskew_fail_i           (train_fail_apb_pos),
        .irq_rx_ready_i             (irq_rx_ready_apb_pos),
        .irq_lane_on_done_i         (irq_lane_on_done_apb_pos),
`ifdef TX_LINK1_EXIST
        .tx1_replay_buf_full_i      (tx1_replay_buf_full_apb_sync),
        .tx1_retrain_timeout_flag_i (tx1_retrain_toflag_apb_sync),
        .tx1_retrain_nack_flag_i    (tx1_retrain_nackflag_apb_sync),
        .tx1_irq_retry_timeout_i    (tx1_irq_retry_to_apb_sync),
        .tx1_irq_retry_nack_i       (tx1_irq_retry_nack_apb_sync),
        .tx1_flit_cnt_i             (tx1_flit_cnt_apb_sync),
        .tx1_timeout_cnt_i          (tx1_timeout_cnt_apb_sync),
`endif
`ifdef TX_LINK2_EXIST
        .tx2_replay_buf_full_i      (tx2_replay_buf_full_apb_sync),
        .tx2_retrain_timeout_flag_i (tx2_retrain_toflag_apb_sync),
        .tx2_retrain_nack_flag_i    (tx2_retrain_nackflag_apb_sync),
        .tx2_irq_retry_timeout_i    (tx2_irq_retry_to_apb_sync),
        .tx2_irq_retry_nack_i       (tx2_irq_retry_nack_apb_sync),
        .tx2_flit_cnt_i             (tx2_flit_cnt_apb_sync),
        .tx2_timeout_cnt_i          (tx2_timeout_cnt_apb_sync),
`endif
`ifdef TX_LINK3_EXIST
        .tx3_replay_buf_full_i      (tx3_replay_buf_full_apb_sync),
        .tx3_retrain_timeout_flag_i (tx3_retrain_toflag_apb_sync),
        .tx3_retrain_nack_flag_i    (tx3_retrain_nackflag_apb_sync),
        .tx3_irq_retry_timeout_i    (tx3_irq_retry_to_apb_sync),
        .tx3_irq_retry_nack_i       (tx3_irq_retry_nack_apb_sync),
        .tx3_flit_cnt_i             (tx3_flit_cnt_apb_sync),
        .tx3_timeout_cnt_i          (tx3_timeout_cnt_apb_sync),
`endif
        .irq_o                      (irq_o),
        .dfx_stat_o                 (dfx_stat_o)
    );

    // =========================================================================
    // apb_clk → mb_clk：配置电平同步（sync_cell_norst 逐 bit，接收侧）
    // 注：sync_cell_norst 无复位，配置默认值由 apb_csr 寄存器复位值经 2 拍传播获得
    // =========================================================================

    sync_cell_norst u_sync_perf_mode      (.clk(mb_clk), .in(csr_perf_mode_apb),      .syncout(csr_perf_mode_mb_sync));
    sync_cell_norst u_sync_s2p_mode       (.clk(mb_clk), .in(s2p_mode_apb),       .syncout(s2p_mode_mb_sync));
    sync_cell_norst u_sync_protocol_mode    (.clk(mb_clk), .in(protocol_mode_apb),  .syncout(protocol_mode_mb_sync));
    sync_cell_norst u_sync_udi_ready_active_en (.clk(mb_clk), .in(csr_udi_ready_active_en_apb), .syncout(csr_udi_ready_active_en_mb_sync));
    sync_cell_norst u_sync_loopback_en    (.clk(mb_clk), .in(csr_loopback_en_apb),    .syncout(csr_loopback_en_mb_sync));
    sync_cell_norst u_sync_nack_retry_en  (.clk(mb_clk), .in(csr_retry_en_apb),       .syncout(retry_en_mb_sync));
    sync_cell_norst u_sync_credit_en      (.clk(mb_clk), .in(csr_credit_en_apb),      .syncout(credit_en_mb_sync));
    sync_cell_norst u_sync_crc_en         (.clk(mb_clk), .in(csr_crc_en_apb),         .syncout(crc_en_mb_sync));
    sync_cell_norst u_sync_ecc_en         (.clk(mb_clk), .in(csr_ecc_en_apb),         .syncout(ecc_en_mb_sync));
    sync_cell_norst u_sync_inj_en_link0  (.clk(mb_clk), .in(inj_en_link0_apb),       .syncout(inj_en_link0_mb_sync));
    sync_cell_norst u_sync_inj_fb_link0  (.clk(mb_clk), .in(inj_fb_link0_apb),       .syncout(inj_fb_link0_mb_sync));
`ifdef TX_LINK1_EXIST
    sync_cell_norst u_sync_inj_en_link1  (.clk(mb_clk), .in(inj_en_link1_apb),       .syncout(inj_en_link1_mb_sync));
    sync_cell_norst u_sync_inj_fb_link1  (.clk(mb_clk), .in(inj_fb_link1_apb),       .syncout(inj_fb_link1_mb_sync));
`endif
`ifdef TX_LINK2_EXIST
    sync_cell_norst u_sync_inj_en_link2  (.clk(mb_clk), .in(inj_en_link2_apb),       .syncout(inj_en_link2_mb_sync));
    sync_cell_norst u_sync_inj_fb_link2  (.clk(mb_clk), .in(inj_fb_link2_apb),       .syncout(inj_fb_link2_mb_sync));
`endif
`ifdef TX_LINK3_EXIST
    sync_cell_norst u_sync_inj_en_link3  (.clk(mb_clk), .in(inj_en_link3_apb),       .syncout(inj_en_link3_mb_sync));
    sync_cell_norst u_sync_inj_fb_link3  (.clk(mb_clk), .in(inj_fb_link3_apb),       .syncout(inj_fb_link3_mb_sync));
`endif
    sync_cell_norst u_sync_retrain_toen   (.clk(mb_clk), .in(retrain_timeout_en_apb), .syncout(retrain_toen_mb_sync));
    sync_cell_norst u_sync_retrain_nacken (.clk(mb_clk), .in(retrain_nack_en_apb),    .syncout(retrain_nacken_mb_sync));
    sync_cell_norst u_sync_bridge         (.clk(mb_clk), .in(bridge_en_i),            .syncout(bridge_mb_sync));

    // mb_rst_n 数据通路复位同步（sync_cell_norst 2 级打拍进 mb_clk 域）

    sync_cell_norst #(.width(16)) u_loopback_src (.clk(mb_clk), .in(csr_loopback_src_apb),     .syncout(csr_loopback_src_mb_sync));
    sync_cell_norst #(.width(16)) u_ack_route_src (.clk(mb_clk), .in(csr_ack_route_src_en_apb), .syncout(csr_ack_route_src_en_mb_sync));

    sync_cell_norst #(.width(16)) u_drift_period (.clk(mb_clk), .in(drift_period_apb),         .syncout(drift_period_mb_sync));
    sync_cell_norst #(.width(16)) u_retry_toval  (.clk(mb_clk), .in(retry_timeout_val_apb),    .syncout(retry_toval_mb_sync));
    sync_cell_norst #(.width(2))  u_bw_mode      (.clk(mb_clk), .in(csr_bw_mode_apb),          .syncout(csr_bw_mode_mb_sync));
    sync_cell_norst #(.width(2))  u_nack_mode    (.clk(mb_clk), .in(csr_nack_mode_apb),        .syncout(csr_nack_mode_mb_sync));
    sync_cell_norst #(.width(8))  u_retrain_tothr   (.clk(mb_clk), .in(retrain_timeout_thr_apb), .syncout(retrain_tothr_mb_sync));
    sync_cell_norst #(.width(8))  u_retrain_nackthr (.clk(mb_clk), .in(retrain_nack_thr_apb),    .syncout(retrain_nackthr_mb_sync));

    // ---- CLK_CFG 时钟控制寄存器同步（apb_clk → mb_clk，准静态配置，sync_cell_norst 逐 bit 同款）----
    // 每路开关（0x9C/0xA0）+ SRAM 开关（0x98 bit2）；总开关已删；reset 全 0 = always-on
    sync_cell_norst u_sync_clk_cfg_sram_dyn_en(.clk(mb_clk), .in(csr_clk_cfg_sram_dyn_en_apb), .syncout(csr_clk_cfg_sram_dyn_en_mb_sync));
    sync_cell_norst u_sync_clk_cfg_tx0_dyn    (.clk(mb_clk), .in(csr_clk_cfg_tx0_dyn_apb),    .syncout(csr_clk_cfg_tx0_dyn_mb_sync));
    sync_cell_norst u_sync_clk_cfg_tx1_dyn    (.clk(mb_clk), .in(csr_clk_cfg_tx1_dyn_apb),    .syncout(csr_clk_cfg_tx1_dyn_mb_sync));
    sync_cell_norst u_sync_clk_cfg_tx2_dyn    (.clk(mb_clk), .in(csr_clk_cfg_tx2_dyn_apb),    .syncout(csr_clk_cfg_tx2_dyn_mb_sync));
    sync_cell_norst u_sync_clk_cfg_tx3_dyn    (.clk(mb_clk), .in(csr_clk_cfg_tx3_dyn_apb),    .syncout(csr_clk_cfg_tx3_dyn_mb_sync));
    sync_cell_norst u_sync_clk_cfg_rx0_dyn    (.clk(mb_clk), .in(csr_clk_cfg_rx0_dyn_apb),    .syncout(csr_clk_cfg_rx0_dyn_mb_sync));
    sync_cell_norst u_sync_clk_cfg_rx1_dyn    (.clk(mb_clk), .in(csr_clk_cfg_rx1_dyn_apb),    .syncout(csr_clk_cfg_rx1_dyn_mb_sync));
    sync_cell_norst u_sync_clk_cfg_rx2_dyn    (.clk(mb_clk), .in(csr_clk_cfg_rx2_dyn_apb),    .syncout(csr_clk_cfg_rx2_dyn_mb_sync));
    sync_cell_norst u_sync_clk_cfg_rx3_dyn    (.clk(mb_clk), .in(csr_clk_cfg_rx3_dyn_apb),    .syncout(csr_clk_cfg_rx3_dyn_mb_sync));

    // =========================================================================
    // apb_clk → mb_clk：脉冲同步（源域 toggle + sync_cell_norst + 边沿检测）
    // =========================================================================
    // CONFIG_DONE 电平直采（sync_cell_norst 2 拍，无沿检测）
    sync_cell_norst u_sync_cfg_done (.clk(mb_clk), .in(csr_cfg_done_apb), .syncout(csr_cfg_done_mb));

    // INIT_CFG_REG.RETRAIN 软件训练触发（apb_clk → mb_clk 脉冲同步）
    always @(posedge apb_clk or negedge apb_rst_n) begin
        if (!apb_rst_n)
            init_retrain_tog_apb <= 1'b0;
        else if (init_retrain_apb)
            init_retrain_tog_apb <= ~init_retrain_tog_apb;
    end

    sync_cell_norst u_sync_init_retrain (.clk(mb_clk), .in(init_retrain_tog_apb), .syncout(init_retrain_mb_sync));

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            init_retrain_mb_d <= 1'b0;
        else
            init_retrain_mb_d <= init_retrain_mb_sync;
    end

    assign init_retrain_mb_pos = init_retrain_mb_sync ^ init_retrain_mb_d;

    // =========================================================================
    // RC 读后清零脉冲（apb_clk → mb_clk，toggle + sync_cell_norst + 边沿检测）
    // =========================================================================
    always @(posedge apb_clk or negedge apb_rst_n) begin
        if (!apb_rst_n) begin
            tx_flit_clr_tog_apb   <= 1'b0;
            rx_flit_clr_tog_apb   <= 1'b0;
            crc_err_clr_tog_apb   <= 1'b0;
`ifdef TX_LINK1_EXIST
            tx1_cnt_clr_tog_apb   <= 1'b0;
`endif
`ifdef TX_LINK2_EXIST
            tx2_cnt_clr_tog_apb   <= 1'b0;
`endif
`ifdef TX_LINK3_EXIST
            tx3_cnt_clr_tog_apb   <= 1'b0;
`endif
        end
        else begin
            tx_flit_clr_tog_apb   <= tx_flit_clr_tog_apb ^ csr_tx_flit_clr_apb;
            rx_flit_clr_tog_apb   <= rx_flit_clr_tog_apb ^ csr_rx_flit_clr_apb;
            crc_err_clr_tog_apb   <= crc_err_clr_tog_apb ^ csr_crc_err_clr_apb;
`ifdef TX_LINK1_EXIST
            tx1_cnt_clr_tog_apb   <= tx1_cnt_clr_tog_apb ^ csr_tx1_cnt_clr_apb;
`endif
`ifdef TX_LINK2_EXIST
            tx2_cnt_clr_tog_apb   <= tx2_cnt_clr_tog_apb ^ csr_tx2_cnt_clr_apb;
`endif
`ifdef TX_LINK3_EXIST
            tx3_cnt_clr_tog_apb   <= tx3_cnt_clr_tog_apb ^ csr_tx3_cnt_clr_apb;
`endif
        end
    end

    sync_cell_norst u_sync_tx_flit_clr   (.clk(mb_clk), .in(tx_flit_clr_tog_apb),   .syncout(tx_flit_clr_mb_sync));
    sync_cell_norst u_sync_rx_flit_clr   (.clk(mb_clk), .in(rx_flit_clr_tog_apb),   .syncout(rx_flit_clr_mb_sync));
    sync_cell_norst u_sync_crc_err_clr   (.clk(mb_clk), .in(crc_err_clr_tog_apb),   .syncout(crc_err_clr_mb_sync));
`ifdef TX_LINK1_EXIST
    sync_cell_norst u_sync_tx1_cnt_clr   (.clk(mb_clk), .in(tx1_cnt_clr_tog_apb),   .syncout(tx1_cnt_clr_mb_sync));
`endif
`ifdef TX_LINK2_EXIST
    sync_cell_norst u_sync_tx2_cnt_clr   (.clk(mb_clk), .in(tx2_cnt_clr_tog_apb),   .syncout(tx2_cnt_clr_mb_sync));
`endif
`ifdef TX_LINK3_EXIST
    sync_cell_norst u_sync_tx3_cnt_clr   (.clk(mb_clk), .in(tx3_cnt_clr_tog_apb),   .syncout(tx3_cnt_clr_mb_sync));
`endif

    always @(posedge mb_clk) begin
        if (!mb_rst_n) begin
            tx_flit_clr_mb_d   <= 1'b0;
            rx_flit_clr_mb_d   <= 1'b0;
            crc_err_clr_mb_d   <= 1'b0;
`ifdef TX_LINK1_EXIST
            tx1_cnt_clr_mb_d   <= 1'b0;
`endif
`ifdef TX_LINK2_EXIST
            tx2_cnt_clr_mb_d   <= 1'b0;
`endif
`ifdef TX_LINK3_EXIST
            tx3_cnt_clr_mb_d   <= 1'b0;
`endif
        end
        else begin
            tx_flit_clr_mb_d   <= tx_flit_clr_mb_sync;
            rx_flit_clr_mb_d   <= rx_flit_clr_mb_sync;
            crc_err_clr_mb_d   <= crc_err_clr_mb_sync;
`ifdef TX_LINK1_EXIST
            tx1_cnt_clr_mb_d   <= tx1_cnt_clr_mb_sync;
`endif
`ifdef TX_LINK2_EXIST
            tx2_cnt_clr_mb_d   <= tx2_cnt_clr_mb_sync;
`endif
`ifdef TX_LINK3_EXIST
            tx3_cnt_clr_mb_d   <= tx3_cnt_clr_mb_sync;
`endif
        end
    end

    assign tx_flit_clr_mb_pos = tx_flit_clr_mb_sync ^ tx_flit_clr_mb_d;
    assign rx_flit_clr_mb_pos = rx_flit_clr_mb_sync ^ rx_flit_clr_mb_d;
    assign crc_err_clr_mb_pos = crc_err_clr_mb_sync ^ crc_err_clr_mb_d;
`ifdef TX_LINK1_EXIST
    assign tx1_cnt_clr_mb_pos = tx1_cnt_clr_mb_sync ^ tx1_cnt_clr_mb_d;
`endif
`ifdef TX_LINK2_EXIST
    assign tx2_cnt_clr_mb_pos = tx2_cnt_clr_mb_sync ^ tx2_cnt_clr_mb_d;
`endif
`ifdef TX_LINK3_EXIST
    assign tx3_cnt_clr_mb_pos = tx3_cnt_clr_mb_sync ^ tx3_cnt_clr_mb_d;
`endif

    // =========================================================================
    // apb_clk → sb_clk：寄存器通路事件触发脉冲（→ event_process，toggle + 边沿检测）
    // =========================================================================
    always @(posedge apb_clk or negedge apb_rst_n) begin
        if (!apb_rst_n) begin
            csr_init_phy_init_tog_apb <= 1'b0;
            csr_init_lane_on_tog_apb  <= 1'b0;
            csr_init_lane_off_tog_apb <= 1'b0;
            csr_init_rx_ready_tog_apb <= 1'b0;
            csr_lp_cmd_tog_apb        <= 6'b0;
        end
        else begin
            csr_init_phy_init_tog_apb <= csr_init_phy_init_tog_apb ^ init_phy_init_apb;
            csr_init_lane_on_tog_apb  <= csr_init_lane_on_tog_apb  ^ init_lane_on_apb;
            csr_init_lane_off_tog_apb <= csr_init_lane_off_tog_apb ^ init_lane_off_apb;
            csr_init_rx_ready_tog_apb <= csr_init_rx_ready_tog_apb ^ init_rx_ready_apb;
            csr_lp_cmd_tog_apb        <= csr_lp_cmd_tog_apb        ^ lp_cmd_apb;
        end
    end

    sync_cell_norst u_sync_init_phy_init   (.clk(sb_clk), .in(csr_init_phy_init_tog_apb), .syncout(init_phy_init_sb_sync));
    sync_cell_norst u_sync_init_lane_on    (.clk(sb_clk), .in(csr_init_lane_on_tog_apb),  .syncout(init_lane_on_sb_sync));
    sync_cell_norst u_sync_init_lane_off   (.clk(sb_clk), .in(csr_init_lane_off_tog_apb), .syncout(init_lane_off_sb_sync));
    sync_cell_norst u_sync_init_rx_ready   (.clk(sb_clk), .in(csr_init_rx_ready_tog_apb), .syncout(init_rx_ready_sb_sync));

    sync_cell_norst #(.width(6)) u_lp_cmd (.clk(sb_clk), .in(csr_lp_cmd_tog_apb), .syncout(lp_cmd_sb_sync));

    always @(posedge sb_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            init_phy_init_sb_d <= 1'b0;
            init_lane_on_sb_d  <= 1'b0;
            init_lane_off_sb_d <= 1'b0;
            init_rx_ready_sb_d <= 1'b0;
            lp_cmd_sb_d        <= 6'b0;
        end
        else begin
            init_phy_init_sb_d <= init_phy_init_sb_sync;
            init_lane_on_sb_d  <= init_lane_on_sb_sync;
            init_lane_off_sb_d <= init_lane_off_sb_sync;
            init_rx_ready_sb_d <= init_rx_ready_sb_sync;
            lp_cmd_sb_d        <= lp_cmd_sb_sync;
        end
    end

    assign csr_init_phy_init_sb_pos = init_phy_init_sb_sync ^ init_phy_init_sb_d;
    assign csr_init_lane_on_sb_pos  = init_lane_on_sb_sync  ^ init_lane_on_sb_d;
    assign csr_init_lane_off_sb_pos = init_lane_off_sb_sync ^ init_lane_off_sb_d;
    assign csr_init_rx_ready_sb_pos = init_rx_ready_sb_sync ^ init_rx_ready_sb_d;
    assign csr_lp_cmd_sb_pos        = lp_cmd_sb_sync        ^ lp_cmd_sb_d;

    // =========================================================================
    // sb_clk → mb_clk：lcmsm 输入同步（event_process 脉冲 + PAIF 控制电平）
    // =========================================================================
    sync_cell_norst u_sync_evt_done      (.clk(mb_clk), .in(evt_done),           .syncout(evt_done_mb_sync));
    sync_cell_norst u_sync_rx_lp_enter   (.clk(mb_clk), .in(rx_evt_lp_enter),    .syncout(rx_evt_lp_enter_mb_sync));
    sync_cell_norst u_sync_rx_lp_exit    (.clk(mb_clk), .in(rx_evt_lp_exit),     .syncout(rx_evt_lp_exit_mb_sync));
    sync_cell_norst u_sync_lp_sw_req     (.clk(mb_clk), .in(lp_sw_req_i),        .syncout(lp_sw_req_mb_sync));

    sync_cell_norst #(.width(4)) u_lp_sw_cmd (.clk(mb_clk), .in(lp_sw_cmd_i), .syncout(lp_sw_cmd_mb_sync));

    always @(posedge mb_clk) begin
        if (!mb_rst_n) begin
            evt_done_mb_d       <= 1'b0;
            rx_evt_lp_enter_mb_d <= 1'b0;
            rx_evt_lp_exit_mb_d  <= 1'b0;
        end
        else begin
            evt_done_mb_d       <= evt_done_mb_sync;
            rx_evt_lp_enter_mb_d <= rx_evt_lp_enter_mb_sync;
            rx_evt_lp_exit_mb_d  <= rx_evt_lp_exit_mb_sync;
        end
    end

    assign evt_done_mb_pos       = evt_done_mb_sync       ^ evt_done_mb_d;
    assign rx_evt_lp_enter_mb_pos = rx_evt_lp_enter_mb_sync ^ rx_evt_lp_enter_mb_d;
    assign rx_evt_lp_exit_mb_pos  = rx_evt_lp_exit_mb_sync  ^ rx_evt_lp_exit_mb_d;

    // =========================================================================
    // sb_clk → mb_clk：PAIF LP 命令单次捕获（CDC 修复 Ac_conv02 46FD）
    //   u_lp_sw_cmd 为 per-bit 2FF 同步（4 个独立同步器），若把逐位同步总线直接送
    //   lcmsm 组合比较，会在同一周期与 evt_done / rx_evt_lp_* / lp_sw_req 的同步
    //   输出汇聚到 st_low_power_nxt，SpyGlass 判 gray encoding FAILED（多比特撕裂
    //   + req/cmd 关联丢失两重风险）。
    //   依赖 PAIF 一步电平握手（link_event_process：lp_sw_ack_o = req & hold，PL 在
    //   req 高期间保持 cmd 稳定至 ack）：在 lp_sw_req 上升沿把已稳定 ≥2 mb 周期的
    //   命令总线单次采样进 lp_sw_cmd_cap，并在 req 高期间保持 → lcmsm 只看到
    //   "单个已寄存的值"，彻底消除撕裂与关联丢失；req 撤除后清 vld，重新武装。
    // =========================================================================
    assign lp_sw_req_rise = lp_sw_req_mb_sync & ~lp_sw_req_rise_d;

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            lp_sw_req_rise_d <= 1'b0;
        else
            lp_sw_req_rise_d <= lp_sw_req_mb_sync;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n) begin
            lp_sw_cmd_cap <= 4'b0;
            lp_sw_cmd_vld <= 1'b0;
        end
        else if (lp_sw_req_rise) begin
            lp_sw_cmd_cap <= lp_sw_cmd_mb_sync;   // 单次采样（req 高期间总线已稳定）
            lp_sw_cmd_vld <= 1'b1;
        end
        else if (!lp_sw_req_mb_sync) begin
            lp_sw_cmd_vld <= 1'b0;                // req 撤除后清除，重新武装
        end
    end

    // =========================================================================
    // mb_clk → sb_clk：lcmsm 事件请求（→ event_process；req 脉冲 toggle，cmd 电平逐 bit）
    // =========================================================================
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            tx_evt_req_tog_mb <= 1'b0;
        else if (tx_lcsm_evt_req)
            tx_evt_req_tog_mb <= ~tx_evt_req_tog_mb;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            rx_evt_req_tog_mb <= 1'b0;
        else if (rx_lcsm_evt_req)
            rx_evt_req_tog_mb <= ~rx_evt_req_tog_mb;
    end

    sync_cell_norst u_sync_tx_evt_req (.clk(sb_clk), .in(tx_evt_req_tog_mb), .syncout(tx_evt_req_sb_sync));
    sync_cell_norst u_sync_rx_evt_req (.clk(sb_clk), .in(rx_evt_req_tog_mb), .syncout(rx_evt_req_sb_sync));

    sync_cell_norst #(.width(16)) u_tx_evt_cmd (.clk(sb_clk), .in(tx_lcsm_evt_cmd), .syncout(tx_lcsm_evt_cmd_sb));
    sync_cell_norst #(.width(16)) u_rx_evt_cmd (.clk(sb_clk), .in(rx_lcsm_evt_cmd), .syncout(rx_lcsm_evt_cmd_sb));

    always @(posedge sb_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            tx_evt_req_sb_d <= 1'b0;
            rx_evt_req_sb_d <= 1'b0;
        end
        else begin
            tx_evt_req_sb_d <= tx_evt_req_sb_sync;
            rx_evt_req_sb_d <= rx_evt_req_sb_sync;
        end
    end

    assign tx_lcsm_evt_req_sb_pos = tx_evt_req_sb_sync ^ tx_evt_req_sb_d;
    assign rx_lcsm_evt_req_sb_pos = rx_evt_req_sb_sync ^ rx_evt_req_sb_d;

    // =========================================================================
    // mb_clk → apb_clk：状态/计数回读（sync_cell_norst 逐 bit；统计计数允许瞬时撕裂，L1 §2.3）
    // =========================================================================
    sync_cell_norst u_sync_replay_buf_full        (.clk(apb_clk), .in(tx0_replay_buf_full),          .syncout(replay_buf_full_apb_sync));
    sync_cell_norst u_sync_retrain_toflag (.clk(apb_clk), .in(tx0_retrain_to_flag),  .syncout(retrain_toflag_apb_sync));
    sync_cell_norst u_sync_retrain_nackflag (.clk(apb_clk), .in(tx0_retrain_nack_flag), .syncout(retrain_nackflag_apb_sync));
    sync_cell_norst u_sync_irq_retry_to   (.clk(apb_clk), .in(tx0_irq_retry_to),     .syncout(irq_retry_to_apb_sync));
    sync_cell_norst u_sync_irq_retry_nack (.clk(apb_clk), .in(tx0_irq_retry_nack),   .syncout(irq_retry_nack_apb_sync));

    sync_cell_norst #(.width(8))  u_timeout_cnt     (.clk(apb_clk), .in(tx0_timeout_cnt),      .syncout(timeout_cnt_apb_sync));
    sync_cell_norst #(.width(16)) u_tx_flit_cnt     (.clk(apb_clk), .in(tx0_flit_cnt),         .syncout(tx_flit_cnt_apb_sync));
    sync_cell_norst #(.width(16)) u_rx_flit_cnt     (.clk(apb_clk), .in(rx0_flit_cnt),         .syncout(rx_flit_cnt_apb_sync));
    sync_cell_norst #(.width(16)) u_crc_err_cnt     (.clk(apb_clk), .in(rx0_crc_err_cnt),      .syncout(crc_err_cnt_apb_sync));
    sync_cell_norst #(.width(4))  u_link_state_tx   (.clk(apb_clk), .in(tx_lcsm_state),        .syncout(link_state_tx_apb_sync));
    sync_cell_norst #(.width(4))  u_link_state_rx   (.clk(apb_clk), .in(rx_lcsm_state),        .syncout(link_state_rx_apb_sync));
    sync_cell_norst #(.width(2))  u_lp_state_tx     (.clk(apb_clk), .in(tx_lp_state),          .syncout(lp_state_tx_apb_sync));
    sync_cell_norst #(.width(2))  u_lp_state_rx     (.clk(apb_clk), .in(rx_lp_state),          .syncout(lp_state_rx_apb_sync));
    sync_cell_norst u_sync_phy_ready ( .clk(apb_clk), .in(phy_ready_mb), .syncout(phy_ready_apb_sync));  // G-01
    sync_cell_norst u_sync_cpif_rx_lpc ( .clk(apb_clk), .in(cpif_rx_lpc_i), .syncout(cpif_rx_lpc_apb_sync));  // v1.9

    // 脉冲 mb→apb：toggle + sync_cell_norst + 边沿检测
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            credit_ovf_tog_mb <= 1'b0;
        else if (rx0_credit_ovf)
            credit_ovf_tog_mb <= ~credit_ovf_tog_mb;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            seq_mis_tog_mb <= 1'b0;
        else if (rx0_seq_mis)
            seq_mis_tog_mb <= ~seq_mis_tog_mb;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            train_fail_tog_mb <= 1'b0;
        else if (rx0_train_fail)
            train_fail_tog_mb <= ~train_fail_tog_mb;
    end

    sync_cell_norst u_sync_credit_ovf (.clk(apb_clk), .in(credit_ovf_tog_mb), .syncout(credit_ovf_apb_sync));
    sync_cell_norst u_sync_seq_mis    (.clk(apb_clk), .in(seq_mis_tog_mb),    .syncout(seq_mis_apb_sync));
    sync_cell_norst u_sync_train_fail (.clk(apb_clk), .in(train_fail_tog_mb), .syncout(train_fail_apb_sync));

    always @(posedge apb_clk or negedge apb_rst_n) begin
        if (!apb_rst_n) begin
            credit_ovf_apb_d <= 1'b0;
            seq_mis_apb_d    <= 1'b0;
            train_fail_apb_d <= 1'b0;
        end
        else begin
            credit_ovf_apb_d <= credit_ovf_apb_sync;
            seq_mis_apb_d    <= seq_mis_apb_sync;
            train_fail_apb_d <= train_fail_apb_sync;
        end
    end

    assign credit_ovf_apb_pos = credit_ovf_apb_sync ^ credit_ovf_apb_d;
    assign seq_mis_apb_pos    = seq_mis_apb_sync    ^ seq_mis_apb_d;
    assign train_fail_apb_pos = train_fail_apb_sync ^ train_fail_apb_d;

    // 额外 TX link（1~3）状态 / IRQ / 计数 mb → apb 逐拍同步
`ifdef TX_LINK1_EXIST
    sync_cell_norst u_sync_tx1_replay_buf_full   (.clk(apb_clk), .in(tx1_replay_buf_full),   .syncout(tx1_replay_buf_full_apb_sync));
    sync_cell_norst u_sync_tx1_retrain_toflag    (.clk(apb_clk), .in(tx1_retrain_to_flag),   .syncout(tx1_retrain_toflag_apb_sync));
    sync_cell_norst u_sync_tx1_retrain_nackflag  (.clk(apb_clk), .in(tx1_retrain_nack_flag), .syncout(tx1_retrain_nackflag_apb_sync));
    sync_cell_norst u_sync_tx1_irq_retry_to      (.clk(apb_clk), .in(tx1_irq_retry_to),      .syncout(tx1_irq_retry_to_apb_sync));
    sync_cell_norst u_sync_tx1_irq_retry_nack    (.clk(apb_clk), .in(tx1_irq_retry_nack),    .syncout(tx1_irq_retry_nack_apb_sync));
    sync_cell_norst #(.width(16)) u_sync_tx1_flit_cnt  (.clk(apb_clk), .in(tx1_flit_cnt),  .syncout(tx1_flit_cnt_apb_sync));
    sync_cell_norst #(.width(8))  u_sync_tx1_timeout_cnt (.clk(apb_clk), .in(tx1_timeout_cnt), .syncout(tx1_timeout_cnt_apb_sync));
`endif
`ifdef TX_LINK2_EXIST
    sync_cell_norst u_sync_tx2_replay_buf_full   (.clk(apb_clk), .in(tx2_replay_buf_full),   .syncout(tx2_replay_buf_full_apb_sync));
    sync_cell_norst u_sync_tx2_retrain_toflag    (.clk(apb_clk), .in(tx2_retrain_to_flag),   .syncout(tx2_retrain_toflag_apb_sync));
    sync_cell_norst u_sync_tx2_retrain_nackflag  (.clk(apb_clk), .in(tx2_retrain_nack_flag), .syncout(tx2_retrain_nackflag_apb_sync));
    sync_cell_norst u_sync_tx2_irq_retry_to      (.clk(apb_clk), .in(tx2_irq_retry_to),      .syncout(tx2_irq_retry_to_apb_sync));
    sync_cell_norst u_sync_tx2_irq_retry_nack    (.clk(apb_clk), .in(tx2_irq_retry_nack),    .syncout(tx2_irq_retry_nack_apb_sync));
    sync_cell_norst #(.width(16)) u_sync_tx2_flit_cnt  (.clk(apb_clk), .in(tx2_flit_cnt),  .syncout(tx2_flit_cnt_apb_sync));
    sync_cell_norst #(.width(8))  u_sync_tx2_timeout_cnt (.clk(apb_clk), .in(tx2_timeout_cnt), .syncout(tx2_timeout_cnt_apb_sync));
`endif
`ifdef TX_LINK3_EXIST
    sync_cell_norst u_sync_tx3_replay_buf_full   (.clk(apb_clk), .in(tx3_replay_buf_full),   .syncout(tx3_replay_buf_full_apb_sync));
    sync_cell_norst u_sync_tx3_retrain_toflag    (.clk(apb_clk), .in(tx3_retrain_to_flag),   .syncout(tx3_retrain_toflag_apb_sync));
    sync_cell_norst u_sync_tx3_retrain_nackflag  (.clk(apb_clk), .in(tx3_retrain_nack_flag), .syncout(tx3_retrain_nackflag_apb_sync));
    sync_cell_norst u_sync_tx3_irq_retry_to      (.clk(apb_clk), .in(tx3_irq_retry_to),      .syncout(tx3_irq_retry_to_apb_sync));
    sync_cell_norst u_sync_tx3_irq_retry_nack    (.clk(apb_clk), .in(tx3_irq_retry_nack),    .syncout(tx3_irq_retry_nack_apb_sync));
    sync_cell_norst #(.width(16)) u_sync_tx3_flit_cnt  (.clk(apb_clk), .in(tx3_flit_cnt),  .syncout(tx3_flit_cnt_apb_sync));
    sync_cell_norst #(.width(8))  u_sync_tx3_timeout_cnt (.clk(apb_clk), .in(tx3_timeout_cnt), .syncout(tx3_timeout_cnt_apb_sync));
`endif

    // =========================================================================
    // sb_clk → apb_clk：lp_transition 脉冲（event_process 中断，toggle + 边沿检测）
    // =========================================================================
    always @(posedge sb_clk or negedge sb_rst_n) begin
        if (!sb_rst_n)
            lp_trans_tog_sb <= 1'b0;
        else if (irq_lp_trans_sb)
            lp_trans_tog_sb <= ~lp_trans_tog_sb;
    end

    sync_cell_norst u_sync_lp_trans (.clk(apb_clk), .in(lp_trans_tog_sb), .syncout(lp_trans_apb_sync));

    always @(posedge apb_clk or negedge apb_rst_n) begin
        if (!apb_rst_n)
            lp_trans_apb_d <= 1'b0;
        else
            lp_trans_apb_d <= lp_trans_apb_sync;
    end

    assign lp_trans_apb_pos = lp_trans_apb_sync ^ lp_trans_apb_d;
    // =========================================================================
    // sb_clk → apb_clk：初始化流程中断脉冲（G-03/G-04，event_process 中断，
    // toggle + sync_cell_norst + 边沿检测，与 lp_transition 同步路径一致）
    // =========================================================================
    always @(posedge sb_clk or negedge sb_rst_n) begin
        if (!sb_rst_n)
            irq_rx_ready_tog_sb <= 1'b0;
        else if (irq_rx_ready_sb)
            irq_rx_ready_tog_sb <= ~irq_rx_ready_tog_sb;
    end

    sync_cell_norst u_sync_irq_rx_ready (.clk(apb_clk), .in(irq_rx_ready_tog_sb), .syncout(irq_rx_ready_apb_sync));

    always @(posedge apb_clk or negedge apb_rst_n) begin
        if (!apb_rst_n)
            irq_rx_ready_apb_d <= 1'b0;
        else
            irq_rx_ready_apb_d <= irq_rx_ready_apb_sync;
    end

    assign irq_rx_ready_apb_pos = irq_rx_ready_apb_sync ^ irq_rx_ready_apb_d;

    always @(posedge sb_clk or negedge sb_rst_n) begin
        if (!sb_rst_n)
            irq_lane_on_done_tog_sb <= 1'b0;
        else if (irq_lane_on_done_sb)
            irq_lane_on_done_tog_sb <= ~irq_lane_on_done_tog_sb;
    end

    sync_cell_norst u_sync_irq_lane_on_done (.clk(apb_clk), .in(irq_lane_on_done_tog_sb), .syncout(irq_lane_on_done_apb_sync));

    always @(posedge apb_clk or negedge apb_rst_n) begin
        if (!apb_rst_n)
            irq_lane_on_done_apb_d <= 1'b0;
        else
            irq_lane_on_done_apb_d <= irq_lane_on_done_apb_sync;
    end

    assign irq_lane_on_done_apb_pos = irq_lane_on_done_apb_sync ^ irq_lane_on_done_apb_d;

    assign nack_type_mb = (csr_nack_mode_mb_sync == 2'b01) ? 2'b11 : 2'b10;

    // =========================================================================
    // link_tx0 例化（link0 TX，永远例化）
    // =========================================================================
    link_tx u_link_tx0 (
        .mb_clk                    (mb_clk),
        .mb_rst_n                  (mb_rst_n),
        .clk_cfg_tx_n_dyn_i        (csr_clk_cfg_tx0_dyn_mb_sync),
        .sram_dyn_en_i             (csr_clk_cfg_sram_dyn_en_mb_sync),
        .paif_tx_data_i            (paif_tx_link0_data_i),
        .paif_tx_valid_i           (paif_tx_link0_valid_i),
        .paif_tx_ready_o           (paif_tx_link0_ready_o),
        .csr_perf_mode_i           (csr_perf_mode_mb_sync),
        .csr_bridge_mode_i         (bridge_mb_sync),
        .csr_s2p_mode_i            (s2p_mode_mb_sync),
        .csr_bw_mode_i             (csr_bw_mode_mb_sync),
        .csr_protocol_mode_i       (protocol_mode_mb_sync),
        .csr_udi_ready_active_en_i (csr_udi_ready_active_en_mb_sync),
        .csr_retry_en_i            (retry_en_mb_sync),
        .csr_credit_en_i           (credit_en_mb_sync),
        .csr_crc_en_i              (crc_en_mb_sync),
        .csr_ecc_en_i              (ecc_en_mb_sync),
        .csr_inj_en_i              (inj_en_link0_mb_sync),
        .csr_inj_fb_i              (inj_fb_link0_mb_sync),
        .tx_flit_clr_i            (tx_flit_clr_mb_pos),
        .csr_drift_period_i        (drift_period_mb_sync),
        .csr_retry_timeout_val_i   (retry_toval_mb_sync),
        .csr_retrain_timeout_thr_i (retrain_tothr_mb_sync),
        .csr_retrain_nack_thr_i    (retrain_nackthr_mb_sync),
        .csr_retrain_timeout_en_i  (retrain_toen_mb_sync),
        .csr_retrain_nack_en_i     (retrain_nacken_mb_sync),
        .ack_seq_i                 (route0_ack_seq),
        .ack_valid_i               (route0_ack_valid),
        .nack_seq_i                (route0_nack_seq),
        .nack_valid_i              (route0_nack_valid),
        .nack_type_i               (nack_type_mb),
        .dlv_ack_seq_i             (route0_dlv_ack_seq),
        .dlv_ack_valid_i           (route0_dlv_ack_valid),
        .lcl_nack_seq_i            (route0_lcl_nack_seq),
        .lcl_nack_valid_i          (route0_lcl_nack_valid),
        .lcl_nack_type_i           (route0_lcl_nack_type),
        .sel_idx_i                 (route0_sel_idx),
        .udi_tx_lane0_data_i       (udi_tx_link0_lane0_data_i),
        .udi_tx_lane0_data_vld_i   (udi_tx_link0_lane0_data_vld_i),
`ifdef LINK_LANE1
        .udi_tx_lane1_data_i       (udi_tx_link0_lane1_data_i),
        .udi_tx_lane1_data_vld_i   (udi_tx_link0_lane1_data_vld_i),
`endif
`ifdef LINK_LANE3
        .udi_tx_lane2_data_i       (udi_tx_link0_lane2_data_i),
        .udi_tx_lane2_data_vld_i   (udi_tx_link0_lane2_data_vld_i),
        .udi_tx_lane3_data_i       (udi_tx_link0_lane3_data_i),
        .udi_tx_lane3_data_vld_i   (udi_tx_link0_lane3_data_vld_i),
`endif
        .udi_tx_ready_o            (udi_tx_link0_ready_o),
        .link_state_i               (tx_lcsm_state),
        .link_active_i             (tx_lcsm_active),
        .link_active_nxt_i         (tx_lcsm_active_nxt),
        .deskew_done_o               (tx0_deskew_done),
        .cpif_tx_lane0_data_o      (tx0_cpif_lane0_data),
        .cpif_tx_lane0_data_vld_o  (tx0_cpif_lane0_vld),
`ifdef LINK_LANE1
        .cpif_tx_lane1_data_o      (tx0_cpif_lane1_data),
        .cpif_tx_lane1_data_vld_o  (tx0_cpif_lane1_vld),
`endif
`ifdef LINK_LANE3
        .cpif_tx_lane2_data_o      (tx0_cpif_lane2_data),
        .cpif_tx_lane2_data_vld_o  (tx0_cpif_lane2_vld),
        .cpif_tx_lane3_data_o      (tx0_cpif_lane3_data),
        .cpif_tx_lane3_data_vld_o  (tx0_cpif_lane3_vld),
`endif
        .cpif_tx_ready_i           (cpif_tx_link0_ready_i),
        .csr_replay_buf_full_o             (tx0_replay_buf_full),
        .retrain_timeout_flag_o    (tx0_retrain_to_flag),
        .retrain_nack_flag_o       (tx0_retrain_nack_flag),
        .irq_retry_timeout_o       (tx0_irq_retry_to),
        .irq_retry_nack_o          (tx0_irq_retry_nack),
        .retrain_req_o             (tx0_retrain_req),
        .tx_flit_cnt_o             (tx0_flit_cnt),
        .timeout_cnt_o             (tx0_timeout_cnt),
        .arq_req_flit_i            (rx0_arq_req_flit),
        .arq_req_valid_i           (rx0_arq_req_valid),
        .arq_req_rcv_ptr_i         (rx0_arq_req_rcv_ptr),
        .arq_req_rcv_valid_i       (rx0_arq_req_rcv_valid),
        .link_number_i             (3'd0)
    );

    // =========================================================================
    // link_rx0 例化（link0 RX，永远例化）
    // loopback 源选择 mux：按 LOOPBACK_CFG_REG.RX0_SRC 索引已例化的 link_tx 出口
    // （2026-10-10 loopback 修复：RX0 增与 RX1/2/3 同构的 4:1 mux；spec §5.3
    //  RX0_SRC 任何配置下有效；RX0_SRC=0 恒选 tx0 = 现状自环零变化）
    // =========================================================================
    always @(*) begin
        case (csr_loopback_src_mb_sync[3:0])
            4'd0: begin
                lb_rx0_lane0_data = tx0_cpif_lane0_data;
                lb_rx0_lane0_vld  = tx0_cpif_lane0_vld;
            end
`ifdef TX_LINK1_EXIST
            4'd1: begin
                lb_rx0_lane0_data = tx1_cpif_lane0_data;
                lb_rx0_lane0_vld  = tx1_cpif_lane0_vld;
            end
`endif
`ifdef TX_LINK2_EXIST
            4'd2: begin
                lb_rx0_lane0_data = tx2_cpif_lane0_data;
                lb_rx0_lane0_vld  = tx2_cpif_lane0_vld;
            end
`endif
`ifdef TX_LINK3_EXIST
            4'd3: begin
                lb_rx0_lane0_data = tx3_cpif_lane0_data;
                lb_rx0_lane0_vld  = tx3_cpif_lane0_vld;
            end
`endif
            default: begin
                lb_rx0_lane0_data = tx0_cpif_lane0_data;
                lb_rx0_lane0_vld  = tx0_cpif_lane0_vld;
            end
        endcase
    end
`ifdef LINK_LANE1
    always @(*) begin
        case (csr_loopback_src_mb_sync[3:0])
            4'd0: begin
                lb_rx0_lane1_data = tx0_cpif_lane1_data;
                lb_rx0_lane1_vld  = tx0_cpif_lane1_vld;
            end
`ifdef TX_LINK1_EXIST
            4'd1: begin
                lb_rx0_lane1_data = tx1_cpif_lane1_data;
                lb_rx0_lane1_vld  = tx1_cpif_lane1_vld;
            end
`endif
`ifdef TX_LINK2_EXIST
            4'd2: begin
                lb_rx0_lane1_data = tx2_cpif_lane1_data;
                lb_rx0_lane1_vld  = tx2_cpif_lane1_vld;
            end
`endif
`ifdef TX_LINK3_EXIST
            4'd3: begin
                lb_rx0_lane1_data = tx3_cpif_lane1_data;
                lb_rx0_lane1_vld  = tx3_cpif_lane1_vld;
            end
`endif
            default: begin
                lb_rx0_lane1_data = tx0_cpif_lane1_data;
                lb_rx0_lane1_vld  = tx0_cpif_lane1_vld;
            end
        endcase
    end
`endif
    link_rx u_link_rx0 (
        .mb_clk                    (mb_clk),
        .mb_rst_n                  (mb_rst_n),
        .clk_cfg_rx_n_dyn_i        (csr_clk_cfg_rx0_dyn_mb_sync),
        .cpif_rx_lane0_data_i      (cpif_rx_link0_lane0_data_i),
        .cpif_rx_lane0_data_vld_i  (cpif_rx_link0_lane0_data_vld_i),
`ifdef LINK_LANE1
        .cpif_rx_lane1_data_i      (cpif_rx_link0_lane1_data_i),
        .cpif_rx_lane1_data_vld_i  (cpif_rx_link0_lane1_data_vld_i),
`endif
`ifdef LINK_LANE3
        .cpif_rx_lane2_data_i      (cpif_rx_link0_lane2_data_i),
        .cpif_rx_lane2_data_vld_i  (cpif_rx_link0_lane2_data_vld_i),
        .cpif_rx_lane3_data_i      (cpif_rx_link0_lane3_data_i),
        .cpif_rx_lane3_data_vld_i  (cpif_rx_link0_lane3_data_vld_i),
`endif
        .loopback_tx_lane0_data_i      (lb_rx0_lane0_data),
        .loopback_tx_lane0_data_vld_i  (lb_rx0_lane0_vld),
`ifdef LINK_LANE1
        .loopback_tx_lane1_data_i      (lb_rx0_lane1_data),
        .loopback_tx_lane1_data_vld_i  (lb_rx0_lane1_vld),
`endif
`ifdef LINK_LANE3
        .loopback_tx_lane2_data_i      (tx0_cpif_lane2_data),
        .loopback_tx_lane2_data_vld_i  (tx0_cpif_lane2_vld),
        .loopback_tx_lane3_data_i      (tx0_cpif_lane3_data),
        .loopback_tx_lane3_data_vld_i  (tx0_cpif_lane3_vld),
`endif
        .csr_s2p_mode_i            (s2p_mode_mb_sync),
        .csr_bw_mode_i             (csr_bw_mode_mb_sync),
        .csr_protocol_mode_i       (protocol_mode_mb_sync),
        .csr_loopback_en_i         (csr_loopback_en_mb_sync),
        .csr_perf_mode_i               (csr_perf_mode_mb_sync),
        .csr_comp_mode_i               (csr_comp_mode_mb),
        .csr_bridge_mode_i             (bridge_mb_sync),
        .csr_crc_en_i                  (crc_en_mb_sync),
        .csr_ecc_en_i                  (ecc_en_mb_sync),
        .csr_credit_en_i               (credit_en_mb_sync),
        .csr_nack_mode_i               (csr_nack_mode_mb_sync),
        .csr_retry_en_i                (retry_en_mb_sync),
        .train_start_i             (rx_train_start),
        .rx_path_en_i              (rx_path_en),
        .paif_rx_data_o            (paif_rx_link0_data_o),
        .paif_rx_valid_o           (paif_rx_link0_valid_o),
        .paif_rx_ready_i           (paif_rx_link0_ready_i),
        .udi_rx_lane0_data_o       (udi_rx_link0_lane0_data_o),
        .udi_rx_lane0_data_vld_o   (udi_rx_link0_lane0_data_vld_o),
`ifdef LINK_LANE1
        .udi_rx_lane1_data_o       (udi_rx_link0_lane1_data_o),
        .udi_rx_lane1_data_vld_o   (udi_rx_link0_lane1_data_vld_o),
`endif
`ifdef LINK_LANE3
        .udi_rx_lane2_data_o       (udi_rx_link0_lane2_data_o),
        .udi_rx_lane2_data_vld_o   (udi_rx_link0_lane2_data_vld_o),
        .udi_rx_lane3_data_o       (udi_rx_link0_lane3_data_o),
        .udi_rx_lane3_data_vld_o   (udi_rx_link0_lane3_data_vld_o),
`endif
        .nack_seq_o                (rx0_nack_seq),
        .nack_valid_o              (rx0_nack_valid),
        .ack_seq_o                 (rx0_ack_seq),
        .ack_valid_o               (rx0_ack_valid),
        .dlp_linknum_o             (rx0_dlp_linknum),
        .dlv_ack_seq_o             (rx0_dlv_ack_seq),
        .dlv_ack_valid_o           (rx0_dlv_ack_valid),
        .lcl_nack_seq_o            (rx0_lcl_nack_seq),
        .lcl_nack_valid_o          (rx0_lcl_nack_valid),
        .lcl_nack_type_o           (rx0_lcl_nack_type),
        .train_done_o              (rx0_train_done),
        .train_fail_o              (rx0_train_fail),
        .credit_overflow_o         (rx0_credit_ovf),
        .seq_mismatch_o            (rx0_seq_mis),
        .rx_flit_cnt_o             (rx0_flit_cnt),
        .crc_err_cnt_o             (rx0_crc_err_cnt),
        .rx_flit_clr_i             (rx_flit_clr_mb_pos),
        .crc_err_clr_i             (crc_err_clr_mb_pos),
        .arq_req_flit_o            (rx0_arq_req_flit),
        .arq_req_valid_o           (rx0_arq_req_valid),
        .arq_req_rcv_ptr_o         (rx0_arq_req_rcv_ptr),
        .arq_req_rcv_valid_o       (rx0_arq_req_rcv_valid)
    );

    // =========================================================================
    // 多 link 例化（非对称结构；对称默认 (1,1) 下不展开）
    // NACK/ACK 反向通道按同序号 link 配对（TX/RX 同侧均存在时），
    // 仅 TX 存在的 link 反向通道输入 tie 0，仅 RX 存在的 link 反向通道输出悬空；
    // 状态回读仅 link0 聚合进 apb_csr（其余 link 状态端口悬空，DFx 扩展待定义）
    // =========================================================================
// RX1 反向通道网声明（提前至首次例化使用之前，避免先使用后声明触发隐式 1bit 网）
`ifdef RX_LINK1_EXIST
    wire [7:0]    rx1_nack_seq;
    wire          rx1_nack_valid;
    wire [7:0]    rx1_ack_seq;
    wire          rx1_ack_valid;
    wire [1:0]    rx1_dlp_linknum;
    wire [7:0]    rx1_dlv_ack_seq;
    wire          rx1_dlv_ack_valid;
    wire [7:0]    rx1_lcl_nack_seq;
    wire          rx1_lcl_nack_valid;
    wire [1:0]    rx1_lcl_nack_type;
    wire [255:0]  rx1_arq_req_flit;      // link_rx1 → link_tx1 ARQ(Request) payload 中继
    wire          rx1_arq_req_valid;
    wire [15:0]   rx1_arq_req_rcv_ptr;   // link_rx1 → link_tx1 远端 ARQ(Request) 指示
    wire          rx1_arq_req_rcv_valid;
`endif
`ifdef TX_LINK1_EXIST
    link_tx u_link_tx1 (
        .mb_clk                    (mb_clk),
        .mb_rst_n                  (mb_rst_n),
        .clk_cfg_tx_n_dyn_i        (csr_clk_cfg_tx1_dyn_mb_sync),
        .sram_dyn_en_i             (csr_clk_cfg_sram_dyn_en_mb_sync),
        .paif_tx_data_i            (paif_tx_link1_data_i),
        .paif_tx_valid_i           (paif_tx_link1_valid_i),
        .paif_tx_ready_o           (paif_tx_link1_ready_o),
        .csr_perf_mode_i           (csr_perf_mode_mb_sync),
        .csr_bridge_mode_i         (bridge_mb_sync),
        .csr_s2p_mode_i            (s2p_mode_mb_sync),
        .csr_bw_mode_i             (csr_bw_mode_mb_sync),
        .csr_protocol_mode_i       (protocol_mode_mb_sync),
        .csr_udi_ready_active_en_i (csr_udi_ready_active_en_mb_sync),
        .csr_retry_en_i            (retry_en_mb_sync),
        .csr_credit_en_i           (credit_en_mb_sync),
        .csr_crc_en_i              (crc_en_mb_sync),
        .csr_ecc_en_i              (ecc_en_mb_sync),
        .csr_inj_en_i              (inj_en_link1_mb_sync),
        .csr_inj_fb_i              (inj_fb_link1_mb_sync),
        .tx_flit_clr_i            (tx1_cnt_clr_mb_pos),
        .csr_drift_period_i        (drift_period_mb_sync),
        .csr_retry_timeout_val_i   (retry_toval_mb_sync),
        .csr_retrain_timeout_thr_i (retrain_tothr_mb_sync),
        .csr_retrain_nack_thr_i    (retrain_nackthr_mb_sync),
        .csr_retrain_timeout_en_i  (retrain_toen_mb_sync),
        .csr_retrain_nack_en_i     (retrain_nacken_mb_sync),
        // 反向 ACK/NACK 通道经 link_ack_route（route1）——本 TX 例化即存在（TX_LINK1_EXIST）
        .ack_seq_i                 (route1_ack_seq),
        .ack_valid_i               (route1_ack_valid),
        .nack_seq_i                (route1_nack_seq),
        .nack_valid_i              (route1_nack_valid),
        .nack_type_i               (nack_type_mb),
        .dlv_ack_seq_i             (route1_dlv_ack_seq),
        .dlv_ack_valid_i           (route1_dlv_ack_valid),
        .lcl_nack_seq_i            (route1_lcl_nack_seq),
        .lcl_nack_valid_i          (route1_lcl_nack_valid),
        .lcl_nack_type_i           (route1_lcl_nack_type),
        .sel_idx_i                 (route1_sel_idx),
`ifdef RX_LINK1_EXIST
        .arq_req_flit_i            (rx1_arq_req_flit),
        .arq_req_valid_i           (rx1_arq_req_valid),
        .arq_req_rcv_ptr_i         (rx1_arq_req_rcv_ptr),
        .arq_req_rcv_valid_i       (rx1_arq_req_rcv_valid),
`else
        .arq_req_flit_i            (256'd0),
        .arq_req_valid_i           (1'b0),
        .arq_req_rcv_ptr_i         (16'd0),
        .arq_req_rcv_valid_i       (1'b0),
`endif
        .udi_tx_lane0_data_i       (udi_tx_link1_lane0_data_i),
        .udi_tx_lane0_data_vld_i   (udi_tx_link1_lane0_data_vld_i),
`ifdef LINK_LANE1
        .udi_tx_lane1_data_i       (udi_tx_link1_lane1_data_i),
        .udi_tx_lane1_data_vld_i   (udi_tx_link1_lane1_data_vld_i),
`endif
        .udi_tx_ready_o            (udi_tx_link1_ready_o),
        .link_state_i               (tx_lcsm_state),
        .link_active_i             (tx_lcsm_active),
        .link_active_nxt_i         (tx_lcsm_active_nxt),
        .deskew_done_o               (tx1_deskew_done),
        .cpif_tx_lane0_data_o      (tx1_cpif_lane0_data),
        .cpif_tx_lane0_data_vld_o  (tx1_cpif_lane0_vld),
`ifdef LINK_LANE1
        .cpif_tx_lane1_data_o      (tx1_cpif_lane1_data),
        .cpif_tx_lane1_data_vld_o  (tx1_cpif_lane1_vld),
`endif
        .cpif_tx_ready_i           (cpif_tx_link1_ready_i),
        .csr_replay_buf_full_o     (tx1_replay_buf_full),
        .retrain_timeout_flag_o    (tx1_retrain_to_flag),
        .retrain_nack_flag_o       (tx1_retrain_nack_flag),
        .retrain_req_o             (tx1_retrain_req),
        .irq_retry_timeout_o       (tx1_irq_retry_to),
        .irq_retry_nack_o          (tx1_irq_retry_nack),
        .tx_flit_cnt_o             (tx1_flit_cnt),
        .timeout_cnt_o             (tx1_timeout_cnt),
        .link_number_i             (3'd1)
    );
    assign cpif_tx_link1_lane0_data_o     = tx1_cpif_lane0_data;
    assign cpif_tx_link1_lane0_data_vld_o = tx1_cpif_lane0_vld;
`ifdef LINK_LANE1
    assign cpif_tx_link1_lane1_data_o     = tx1_cpif_lane1_data;
    assign cpif_tx_link1_lane1_data_vld_o = tx1_cpif_lane1_vld;
`endif
`endif

`ifdef RX_LINK1_EXIST
    // loopback 源选择 mux：按 LOOPBACK_CFG_REG.RX1_SRC 索引已例化的 link_tx 出口
    always @(*) begin
        case (csr_loopback_src_mb_sync[7:4])
            4'd0: begin
                lb_rx1_lane0_data = tx0_cpif_lane0_data;
                lb_rx1_lane0_vld  = tx0_cpif_lane0_vld;
            end
`ifdef TX_LINK1_EXIST
            4'd1: begin
                lb_rx1_lane0_data = tx1_cpif_lane0_data;
                lb_rx1_lane0_vld  = tx1_cpif_lane0_vld;
            end
`endif
            default: begin
                lb_rx1_lane0_data = tx0_cpif_lane0_data;
                lb_rx1_lane0_vld  = tx0_cpif_lane0_vld;
            end
        endcase
    end
`ifdef LINK_LANE1
    always @(*) begin
        case (csr_loopback_src_mb_sync[7:4])
            4'd0: begin
                lb_rx1_lane1_data = tx0_cpif_lane1_data;
                lb_rx1_lane1_vld  = tx0_cpif_lane1_vld;
            end
`ifdef TX_LINK1_EXIST
            4'd1: begin
                lb_rx1_lane1_data = tx1_cpif_lane1_data;
                lb_rx1_lane1_vld  = tx1_cpif_lane1_vld;
            end
`endif
            default: begin
                lb_rx1_lane1_data = tx0_cpif_lane1_data;
                lb_rx1_lane1_vld  = tx0_cpif_lane1_vld;
            end
        endcase
    end
`endif
    link_rx u_link_rx1 (
        .mb_clk                    (mb_clk),
        .mb_rst_n                  (mb_rst_n),
        .clk_cfg_rx_n_dyn_i        (csr_clk_cfg_rx1_dyn_mb_sync),
        .cpif_rx_lane0_data_i      (cpif_rx_link1_lane0_data_i),
        .cpif_rx_lane0_data_vld_i  (cpif_rx_link1_lane0_data_vld_i),
`ifdef LINK_LANE1
        .cpif_rx_lane1_data_i      (cpif_rx_link1_lane1_data_i),
        .cpif_rx_lane1_data_vld_i  (cpif_rx_link1_lane1_data_vld_i),
`endif
        .loopback_tx_lane0_data_i      (lb_rx1_lane0_data),
        .loopback_tx_lane0_data_vld_i  (lb_rx1_lane0_vld),
`ifdef LINK_LANE1
        .loopback_tx_lane1_data_i      (lb_rx1_lane1_data),
        .loopback_tx_lane1_data_vld_i  (lb_rx1_lane1_vld),
`endif
        .csr_s2p_mode_i            (s2p_mode_mb_sync),
        .csr_bw_mode_i             (csr_bw_mode_mb_sync),
        .csr_protocol_mode_i       (protocol_mode_mb_sync),
        .csr_loopback_en_i         (csr_loopback_en_mb_sync),
        .csr_perf_mode_i               (csr_perf_mode_mb_sync),
        .csr_comp_mode_i               (csr_comp_mode_mb),
        .csr_bridge_mode_i             (bridge_mb_sync),
        .csr_crc_en_i                  (crc_en_mb_sync),
        .csr_ecc_en_i                  (ecc_en_mb_sync),
        .csr_credit_en_i               (credit_en_mb_sync),
        .csr_nack_mode_i               (csr_nack_mode_mb_sync),
        .csr_retry_en_i                (retry_en_mb_sync),
        .train_start_i             (rx_train_start),
        .rx_path_en_i              (rx_path_en),
        .paif_rx_data_o            (paif_rx_link1_data_o),
        .paif_rx_valid_o           (paif_rx_link1_valid_o),
        .paif_rx_ready_i           (paif_rx_link1_ready_i),
        .udi_rx_lane0_data_o       (udi_rx_link1_lane0_data_o),
        .udi_rx_lane0_data_vld_o   (udi_rx_link1_lane0_data_vld_o),
`ifdef LINK_LANE1
        .udi_rx_lane1_data_o       (udi_rx_link1_lane1_data_o),
        .udi_rx_lane1_data_vld_o   (udi_rx_link1_lane1_data_vld_o),
`endif
        .nack_seq_o                (rx1_nack_seq),
        .nack_valid_o              (rx1_nack_valid),
        .ack_seq_o                 (rx1_ack_seq),
        .ack_valid_o               (rx1_ack_valid),
        .dlp_linknum_o             (rx1_dlp_linknum),
        .dlv_ack_seq_o             (rx1_dlv_ack_seq),
        .dlv_ack_valid_o           (rx1_dlv_ack_valid),
        .lcl_nack_seq_o            (rx1_lcl_nack_seq),
        .lcl_nack_valid_o          (rx1_lcl_nack_valid),
        .lcl_nack_type_o           (rx1_lcl_nack_type),
        .train_done_o              (),
        .train_fail_o              (),
        .credit_overflow_o         (),
        .seq_mismatch_o            (),
        .rx_flit_cnt_o             (),
        .crc_err_cnt_o             (),
        .rx_flit_clr_i             (1'b0),
        .crc_err_clr_i             (1'b0),
        .arq_req_flit_o            (rx1_arq_req_flit),
        .arq_req_valid_o           (rx1_arq_req_valid),
        .arq_req_rcv_ptr_o         (rx1_arq_req_rcv_ptr),
        .arq_req_rcv_valid_o       (rx1_arq_req_rcv_valid)
    );
`endif

// =========================================================================
// link_tx2 例化（link2 TX，TX_LINK2_EXIST）
// =========================================================================
// RX2 反向通道网声明（提前至首次例化使用之前，避免先使用后声明触发隐式 1bit 网）
`ifdef RX_LINK2_EXIST
    wire [7:0]    rx2_nack_seq;
    wire          rx2_nack_valid;
    wire [7:0]    rx2_ack_seq;
    wire          rx2_ack_valid;
    wire [1:0]    rx2_dlp_linknum;
    wire [7:0]    rx2_dlv_ack_seq;
    wire          rx2_dlv_ack_valid;
    wire [7:0]    rx2_lcl_nack_seq;
    wire          rx2_lcl_nack_valid;
    wire [1:0]    rx2_lcl_nack_type;
    wire [255:0]  rx2_arq_req_flit;      // link_rx2 → link_tx2 ARQ(Request) payload 中继
    wire          rx2_arq_req_valid;
    wire [15:0]   rx2_arq_req_rcv_ptr;   // link_rx2 → link_tx2 远端 ARQ(Request) 指示
    wire          rx2_arq_req_rcv_valid;
`endif
`ifdef TX_LINK2_EXIST
    link_tx u_link_tx2 (
        .mb_clk                    (mb_clk),
        .mb_rst_n                  (mb_rst_n),
        .clk_cfg_tx_n_dyn_i        (csr_clk_cfg_tx2_dyn_mb_sync),
        .sram_dyn_en_i             (csr_clk_cfg_sram_dyn_en_mb_sync),
        .paif_tx_data_i            (paif_tx_link2_data_i),
        .paif_tx_valid_i           (paif_tx_link2_valid_i),
        .paif_tx_ready_o           (paif_tx_link2_ready_o),
        .csr_perf_mode_i           (csr_perf_mode_mb_sync),
        .csr_bridge_mode_i         (bridge_mb_sync),
        .csr_s2p_mode_i            (s2p_mode_mb_sync),
        .csr_bw_mode_i             (csr_bw_mode_mb_sync),
        .csr_protocol_mode_i       (protocol_mode_mb_sync),
        .csr_udi_ready_active_en_i (csr_udi_ready_active_en_mb_sync),
        .csr_retry_en_i            (retry_en_mb_sync),
        .csr_credit_en_i           (credit_en_mb_sync),
        .csr_crc_en_i              (crc_en_mb_sync),
        .csr_ecc_en_i              (ecc_en_mb_sync),
        .csr_inj_en_i              (inj_en_link2_mb_sync),
        .csr_inj_fb_i              (inj_fb_link2_mb_sync),
        .tx_flit_clr_i            (tx2_cnt_clr_mb_pos),
        .csr_drift_period_i        (drift_period_mb_sync),
        .csr_retry_timeout_val_i   (retry_toval_mb_sync),
        .csr_retrain_timeout_thr_i (retrain_tothr_mb_sync),
        .csr_retrain_nack_thr_i    (retrain_nackthr_mb_sync),
        .csr_retrain_timeout_en_i  (retrain_toen_mb_sync),
        .csr_retrain_nack_en_i     (retrain_nacken_mb_sync),
        // 反向 ACK/NACK 通道经 link_ack_route（route2）——本 TX 例化即存在（TX_LINK2_EXIST）
        .ack_seq_i                 (route2_ack_seq),
        .ack_valid_i               (route2_ack_valid),
        .nack_seq_i                (route2_nack_seq),
        .nack_valid_i              (route2_nack_valid),
        .dlv_ack_seq_i             (route2_dlv_ack_seq),
        .dlv_ack_valid_i           (route2_dlv_ack_valid),
        .lcl_nack_seq_i            (route2_lcl_nack_seq),
        .lcl_nack_valid_i          (route2_lcl_nack_valid),
        .lcl_nack_type_i           (route2_lcl_nack_type),
        .sel_idx_i                 (route2_sel_idx),
`ifdef RX_LINK2_EXIST
        .arq_req_flit_i            (rx2_arq_req_flit),
        .arq_req_valid_i           (rx2_arq_req_valid),
        .arq_req_rcv_ptr_i         (rx2_arq_req_rcv_ptr),
        .arq_req_rcv_valid_i       (rx2_arq_req_rcv_valid),
`else
        .arq_req_flit_i            (256'd0),
        .arq_req_valid_i           (1'b0),
        .arq_req_rcv_ptr_i         (16'd0),
        .arq_req_rcv_valid_i       (1'b0),
`endif
        .nack_type_i               (nack_type_mb),
        .udi_tx_lane0_data_i       (udi_tx_link2_lane0_data_i),
        .udi_tx_lane0_data_vld_i   (udi_tx_link2_lane0_data_vld_i),
`ifdef LINK_LANE1
        .udi_tx_lane1_data_i       (udi_tx_link2_lane1_data_i),
        .udi_tx_lane1_data_vld_i   (udi_tx_link2_lane1_data_vld_i),
`endif
        .udi_tx_ready_o            (udi_tx_link2_ready_o),
        .link_state_i               (tx_lcsm_state),
        .link_active_i             (tx_lcsm_active),
        .link_active_nxt_i         (tx_lcsm_active_nxt),
        .deskew_done_o               (tx2_deskew_done),
        .cpif_tx_lane0_data_o      (tx2_cpif_lane0_data),
        .cpif_tx_lane0_data_vld_o  (tx2_cpif_lane0_vld),
`ifdef LINK_LANE1
        .cpif_tx_lane1_data_o      (tx2_cpif_lane1_data),
        .cpif_tx_lane1_data_vld_o  (tx2_cpif_lane1_vld),
`endif
        .cpif_tx_ready_i           (cpif_tx_link2_ready_i),
        .csr_replay_buf_full_o     (tx2_replay_buf_full),
        .retrain_timeout_flag_o    (tx2_retrain_to_flag),
        .retrain_nack_flag_o       (tx2_retrain_nack_flag),
        .retrain_req_o             (tx2_retrain_req),
        .irq_retry_timeout_o       (tx2_irq_retry_to),
        .irq_retry_nack_o          (tx2_irq_retry_nack),
        .tx_flit_cnt_o             (tx2_flit_cnt),
        .timeout_cnt_o             (tx2_timeout_cnt),
        .link_number_i             (3'd2)
    );
    assign cpif_tx_link2_lane0_data_o     = tx2_cpif_lane0_data;
    assign cpif_tx_link2_lane0_data_vld_o = tx2_cpif_lane0_vld;
`ifdef LINK_LANE1
    assign cpif_tx_link2_lane1_data_o     = tx2_cpif_lane1_data;
    assign cpif_tx_link2_lane1_data_vld_o = tx2_cpif_lane1_vld;
`endif
`endif

`ifdef RX_LINK2_EXIST
    // loopback 源选择 mux：按 LOOPBACK_CFG_REG.RX2_SRC 索引已例化的 link_tx 出口
    always @(*) begin
        case (csr_loopback_src_mb_sync[11:8])
            4'd0: begin
                lb_rx2_lane0_data = tx0_cpif_lane0_data;
                lb_rx2_lane0_vld  = tx0_cpif_lane0_vld;
            end
`ifdef TX_LINK1_EXIST
            4'd1: begin
                lb_rx2_lane0_data = tx1_cpif_lane0_data;
                lb_rx2_lane0_vld  = tx1_cpif_lane0_vld;
            end
`endif
`ifdef TX_LINK2_EXIST
            4'd2: begin
                lb_rx2_lane0_data = tx2_cpif_lane0_data;
                lb_rx2_lane0_vld  = tx2_cpif_lane0_vld;
            end
`endif
            default: begin
                lb_rx2_lane0_data = tx0_cpif_lane0_data;
                lb_rx2_lane0_vld  = tx0_cpif_lane0_vld;
            end
        endcase
    end
`ifdef LINK_LANE1
    always @(*) begin
        case (csr_loopback_src_mb_sync[11:8])
            4'd0: begin
                lb_rx2_lane1_data = tx0_cpif_lane1_data;
                lb_rx2_lane1_vld  = tx0_cpif_lane1_vld;
            end
`ifdef TX_LINK1_EXIST
            4'd1: begin
                lb_rx2_lane1_data = tx1_cpif_lane1_data;
                lb_rx2_lane1_vld  = tx1_cpif_lane1_vld;
            end
`endif
`ifdef TX_LINK2_EXIST
            4'd2: begin
                lb_rx2_lane1_data = tx2_cpif_lane1_data;
                lb_rx2_lane1_vld  = tx2_cpif_lane1_vld;
            end
`endif
            default: begin
                lb_rx2_lane1_data = tx0_cpif_lane1_data;
                lb_rx2_lane1_vld  = tx0_cpif_lane1_vld;
            end
        endcase
    end
`endif
    link_rx u_link_rx2 (
        .mb_clk                    (mb_clk),
        .mb_rst_n                  (mb_rst_n),
        .clk_cfg_rx_n_dyn_i        (csr_clk_cfg_rx2_dyn_mb_sync),
        .cpif_rx_lane0_data_i      (cpif_rx_link2_lane0_data_i),
        .cpif_rx_lane0_data_vld_i  (cpif_rx_link2_lane0_data_vld_i),
`ifdef LINK_LANE1
        .cpif_rx_lane1_data_i      (cpif_rx_link2_lane1_data_i),
        .cpif_rx_lane1_data_vld_i  (cpif_rx_link2_lane1_data_vld_i),
`endif
        .loopback_tx_lane0_data_i      (lb_rx2_lane0_data),
        .loopback_tx_lane0_data_vld_i  (lb_rx2_lane0_vld),
`ifdef LINK_LANE1
        .loopback_tx_lane1_data_i      (lb_rx2_lane1_data),
        .loopback_tx_lane1_data_vld_i  (lb_rx2_lane1_vld),
`endif
        .csr_s2p_mode_i            (s2p_mode_mb_sync),
        .csr_bw_mode_i             (csr_bw_mode_mb_sync),
        .csr_protocol_mode_i       (protocol_mode_mb_sync),
        .csr_loopback_en_i         (csr_loopback_en_mb_sync),
        .csr_perf_mode_i               (csr_perf_mode_mb_sync),
        .csr_comp_mode_i               (csr_comp_mode_mb),
        .csr_bridge_mode_i             (bridge_mb_sync),
        .csr_crc_en_i                  (crc_en_mb_sync),
        .csr_ecc_en_i                  (ecc_en_mb_sync),
        .csr_credit_en_i               (credit_en_mb_sync),
        .csr_nack_mode_i               (csr_nack_mode_mb_sync),
        .csr_retry_en_i                (retry_en_mb_sync),
        .train_start_i             (rx_train_start),
        .rx_path_en_i              (rx_path_en),
        .paif_rx_data_o            (paif_rx_link2_data_o),
        .paif_rx_valid_o           (paif_rx_link2_valid_o),
        .paif_rx_ready_i           (paif_rx_link2_ready_i),
        .udi_rx_lane0_data_o       (udi_rx_link2_lane0_data_o),
        .udi_rx_lane0_data_vld_o   (udi_rx_link2_lane0_data_vld_o),
`ifdef LINK_LANE1
        .udi_rx_lane1_data_o       (udi_rx_link2_lane1_data_o),
        .udi_rx_lane1_data_vld_o   (udi_rx_link2_lane1_data_vld_o),
`endif
        .nack_seq_o                (rx2_nack_seq),
        .nack_valid_o              (rx2_nack_valid),
        .ack_seq_o                 (rx2_ack_seq),
        .ack_valid_o               (rx2_ack_valid),
        .dlp_linknum_o             (rx2_dlp_linknum),
        .dlv_ack_seq_o             (rx2_dlv_ack_seq),
        .dlv_ack_valid_o           (rx2_dlv_ack_valid),
        .lcl_nack_seq_o            (rx2_lcl_nack_seq),
        .lcl_nack_valid_o          (rx2_lcl_nack_valid),
        .lcl_nack_type_o           (rx2_lcl_nack_type),
        .train_done_o              (),
        .train_fail_o              (),
        .credit_overflow_o         (),
        .seq_mismatch_o            (),
        .rx_flit_cnt_o             (),
        .crc_err_cnt_o             (),
        .rx_flit_clr_i             (1'b0),
        .crc_err_clr_i             (1'b0),
        .arq_req_flit_o            (rx2_arq_req_flit),
        .arq_req_valid_o           (rx2_arq_req_valid),
        .arq_req_rcv_ptr_o         (rx2_arq_req_rcv_ptr),
        .arq_req_rcv_valid_o       (rx2_arq_req_rcv_valid)
    );
`endif

    // =========================================================================
    // link_tx3 例化（link3 TX，TX_LINK3_EXIST）
    // =========================================================================
    // RX3 反向通道网声明（提前至首次例化使用之前，避免先使用后声明触发隐式 1bit 网）
`ifdef RX_LINK3_EXIST
    wire [7:0]    rx3_nack_seq;
    wire          rx3_nack_valid;
    wire [7:0]    rx3_ack_seq;
    wire          rx3_ack_valid;
    wire [1:0]    rx3_dlp_linknum;
    wire [7:0]    rx3_dlv_ack_seq;
    wire          rx3_dlv_ack_valid;
    wire [7:0]    rx3_lcl_nack_seq;
    wire          rx3_lcl_nack_valid;
    wire [1:0]    rx3_lcl_nack_type;
    wire [255:0]  rx3_arq_req_flit;      // link_rx3 → link_tx3 ARQ(Request) payload 中继
    wire          rx3_arq_req_valid;
    wire [15:0]   rx3_arq_req_rcv_ptr;   // link_rx3 → link_tx3 远端 ARQ(Request) 指示
    wire          rx3_arq_req_rcv_valid;
`endif
`ifdef TX_LINK3_EXIST
    link_tx u_link_tx3 (
        .mb_clk                    (mb_clk),
        .mb_rst_n                  (mb_rst_n),
        .clk_cfg_tx_n_dyn_i        (csr_clk_cfg_tx3_dyn_mb_sync),
        .sram_dyn_en_i             (csr_clk_cfg_sram_dyn_en_mb_sync),
        .paif_tx_data_i            (paif_tx_link3_data_i),
        .paif_tx_valid_i           (paif_tx_link3_valid_i),
        .paif_tx_ready_o           (paif_tx_link3_ready_o),
        .csr_perf_mode_i           (csr_perf_mode_mb_sync),
        .csr_bridge_mode_i         (bridge_mb_sync),
        .csr_s2p_mode_i            (s2p_mode_mb_sync),
        .csr_bw_mode_i             (csr_bw_mode_mb_sync),
        .csr_protocol_mode_i       (protocol_mode_mb_sync),
        .csr_udi_ready_active_en_i (csr_udi_ready_active_en_mb_sync),
        .csr_retry_en_i            (retry_en_mb_sync),
        .csr_credit_en_i           (credit_en_mb_sync),
        .csr_crc_en_i              (crc_en_mb_sync),
        .csr_ecc_en_i              (ecc_en_mb_sync),
        .csr_inj_en_i              (inj_en_link3_mb_sync),
        .csr_inj_fb_i              (inj_fb_link3_mb_sync),
        .tx_flit_clr_i            (tx3_cnt_clr_mb_pos),
        .csr_drift_period_i        (drift_period_mb_sync),
        .csr_retry_timeout_val_i   (retry_toval_mb_sync),
        .csr_retrain_timeout_thr_i (retrain_tothr_mb_sync),
        .csr_retrain_nack_thr_i    (retrain_nackthr_mb_sync),
        .csr_retrain_timeout_en_i  (retrain_toen_mb_sync),
        .csr_retrain_nack_en_i     (retrain_nacken_mb_sync),
        // 反向 ACK/NACK 通道经 link_ack_route（route3）——本 TX 例化即存在（TX_LINK3_EXIST）
        .ack_seq_i                 (route3_ack_seq),
        .ack_valid_i               (route3_ack_valid),
        .nack_seq_i                (route3_nack_seq),
        .nack_valid_i              (route3_nack_valid),
        .dlv_ack_seq_i             (route3_dlv_ack_seq),
        .dlv_ack_valid_i           (route3_dlv_ack_valid),
        .lcl_nack_seq_i            (route3_lcl_nack_seq),
        .lcl_nack_valid_i          (route3_lcl_nack_valid),
        .lcl_nack_type_i           (route3_lcl_nack_type),
        .sel_idx_i                 (route3_sel_idx),
`ifdef RX_LINK3_EXIST
        .arq_req_flit_i            (rx3_arq_req_flit),
        .arq_req_valid_i           (rx3_arq_req_valid),
        .arq_req_rcv_ptr_i         (rx3_arq_req_rcv_ptr),
        .arq_req_rcv_valid_i       (rx3_arq_req_rcv_valid),
`else
        .arq_req_flit_i            (256'd0),
        .arq_req_valid_i           (1'b0),
        .arq_req_rcv_ptr_i         (16'd0),
        .arq_req_rcv_valid_i       (1'b0),
`endif
        .nack_type_i               (nack_type_mb),
        .udi_tx_lane0_data_i       (udi_tx_link3_lane0_data_i),
        .udi_tx_lane0_data_vld_i   (udi_tx_link3_lane0_data_vld_i),
`ifdef LINK_LANE1
        .udi_tx_lane1_data_i       (udi_tx_link3_lane1_data_i),
        .udi_tx_lane1_data_vld_i   (udi_tx_link3_lane1_data_vld_i),
`endif
        .udi_tx_ready_o            (udi_tx_link3_ready_o),
        .link_state_i               (tx_lcsm_state),
        .link_active_i             (tx_lcsm_active),
        .link_active_nxt_i         (tx_lcsm_active_nxt),
        .deskew_done_o               (tx3_deskew_done),
        .cpif_tx_lane0_data_o      (tx3_cpif_lane0_data),
        .cpif_tx_lane0_data_vld_o  (tx3_cpif_lane0_vld),
`ifdef LINK_LANE1
        .cpif_tx_lane1_data_o      (tx3_cpif_lane1_data),
        .cpif_tx_lane1_data_vld_o  (tx3_cpif_lane1_vld),
`endif
        .cpif_tx_ready_i           (cpif_tx_link3_ready_i),
        .csr_replay_buf_full_o     (tx3_replay_buf_full),
        .retrain_timeout_flag_o    (tx3_retrain_to_flag),
        .retrain_nack_flag_o       (tx3_retrain_nack_flag),
        .retrain_req_o             (tx3_retrain_req),
        .irq_retry_timeout_o       (tx3_irq_retry_to),
        .irq_retry_nack_o          (tx3_irq_retry_nack),
        .tx_flit_cnt_o             (tx3_flit_cnt),
        .timeout_cnt_o             (tx3_timeout_cnt),
        .link_number_i             (3'd3)
    );
    assign cpif_tx_link3_lane0_data_o     = tx3_cpif_lane0_data;
    assign cpif_tx_link3_lane0_data_vld_o = tx3_cpif_lane0_vld;
`ifdef LINK_LANE1
    assign cpif_tx_link3_lane1_data_o     = tx3_cpif_lane1_data;
    assign cpif_tx_link3_lane1_data_vld_o = tx3_cpif_lane1_vld;
`endif
`endif

`ifdef RX_LINK3_EXIST
    // loopback 源选择 mux：按 LOOPBACK_CFG_REG.RX3_SRC 索引已例化的 link_tx 出口
    always @(*) begin
        case (csr_loopback_src_mb_sync[15:12])
            4'd0: begin
                lb_rx3_lane0_data = tx0_cpif_lane0_data;
                lb_rx3_lane0_vld  = tx0_cpif_lane0_vld;
            end
`ifdef TX_LINK1_EXIST
            4'd1: begin
                lb_rx3_lane0_data = tx1_cpif_lane0_data;
                lb_rx3_lane0_vld  = tx1_cpif_lane0_vld;
            end
`endif
`ifdef TX_LINK2_EXIST
            4'd2: begin
                lb_rx3_lane0_data = tx2_cpif_lane0_data;
                lb_rx3_lane0_vld  = tx2_cpif_lane0_vld;
            end
`endif
`ifdef TX_LINK3_EXIST
            4'd3: begin
                lb_rx3_lane0_data = tx3_cpif_lane0_data;
                lb_rx3_lane0_vld  = tx3_cpif_lane0_vld;
            end
`endif
            default: begin
                lb_rx3_lane0_data = tx0_cpif_lane0_data;
                lb_rx3_lane0_vld  = tx0_cpif_lane0_vld;
            end
        endcase
    end
`ifdef LINK_LANE1
    always @(*) begin
        case (csr_loopback_src_mb_sync[15:12])
            4'd0: begin
                lb_rx3_lane1_data = tx0_cpif_lane1_data;
                lb_rx3_lane1_vld  = tx0_cpif_lane1_vld;
            end
`ifdef TX_LINK1_EXIST
            4'd1: begin
                lb_rx3_lane1_data = tx1_cpif_lane1_data;
                lb_rx3_lane1_vld  = tx1_cpif_lane1_vld;
            end
`endif
`ifdef TX_LINK2_EXIST
            4'd2: begin
                lb_rx3_lane1_data = tx2_cpif_lane1_data;
                lb_rx3_lane1_vld  = tx2_cpif_lane1_vld;
            end
`endif
`ifdef TX_LINK3_EXIST
            4'd3: begin
                lb_rx3_lane1_data = tx3_cpif_lane1_data;
                lb_rx3_lane1_vld  = tx3_cpif_lane1_vld;
            end
`endif
            default: begin
                lb_rx3_lane1_data = tx0_cpif_lane1_data;
                lb_rx3_lane1_vld  = tx0_cpif_lane1_vld;
            end
        endcase
    end
`endif
    link_rx u_link_rx3 (
        .mb_clk                    (mb_clk),
        .mb_rst_n                  (mb_rst_n),
        .clk_cfg_rx_n_dyn_i        (csr_clk_cfg_rx3_dyn_mb_sync),
        .cpif_rx_lane0_data_i      (cpif_rx_link3_lane0_data_i),
        .cpif_rx_lane0_data_vld_i  (cpif_rx_link3_lane0_data_vld_i),
`ifdef LINK_LANE1
        .cpif_rx_lane1_data_i      (cpif_rx_link3_lane1_data_i),
        .cpif_rx_lane1_data_vld_i  (cpif_rx_link3_lane1_data_vld_i),
`endif
        .loopback_tx_lane0_data_i      (lb_rx3_lane0_data),
        .loopback_tx_lane0_data_vld_i  (lb_rx3_lane0_vld),
`ifdef LINK_LANE1
        .loopback_tx_lane1_data_i      (lb_rx3_lane1_data),
        .loopback_tx_lane1_data_vld_i  (lb_rx3_lane1_vld),
`endif
        .csr_s2p_mode_i            (s2p_mode_mb_sync),
        .csr_bw_mode_i             (csr_bw_mode_mb_sync),
        .csr_protocol_mode_i       (protocol_mode_mb_sync),
        .csr_loopback_en_i         (csr_loopback_en_mb_sync),
        .csr_perf_mode_i               (csr_perf_mode_mb_sync),
        .csr_comp_mode_i               (csr_comp_mode_mb),
        .csr_bridge_mode_i             (bridge_mb_sync),
        .csr_crc_en_i                  (crc_en_mb_sync),
        .csr_ecc_en_i                  (ecc_en_mb_sync),
        .csr_credit_en_i               (credit_en_mb_sync),
        .csr_nack_mode_i               (csr_nack_mode_mb_sync),
        .csr_retry_en_i                (retry_en_mb_sync),
        .train_start_i             (rx_train_start),
        .rx_path_en_i              (rx_path_en),
        .paif_rx_data_o            (paif_rx_link3_data_o),
        .paif_rx_valid_o           (paif_rx_link3_valid_o),
        .paif_rx_ready_i           (paif_rx_link3_ready_i),
        .udi_rx_lane0_data_o       (udi_rx_link3_lane0_data_o),
        .udi_rx_lane0_data_vld_o   (udi_rx_link3_lane0_data_vld_o),
`ifdef LINK_LANE1
        .udi_rx_lane1_data_o       (udi_rx_link3_lane1_data_o),
        .udi_rx_lane1_data_vld_o   (udi_rx_link3_lane1_data_vld_o),
`endif
        .nack_seq_o                (rx3_nack_seq),
        .nack_valid_o              (rx3_nack_valid),
        .ack_seq_o                 (rx3_ack_seq),
        .ack_valid_o               (rx3_ack_valid),
        .dlp_linknum_o             (rx3_dlp_linknum),
        .dlv_ack_seq_o             (rx3_dlv_ack_seq),
        .dlv_ack_valid_o           (rx3_dlv_ack_valid),
        .lcl_nack_seq_o            (rx3_lcl_nack_seq),
        .lcl_nack_valid_o          (rx3_lcl_nack_valid),
        .lcl_nack_type_o           (rx3_lcl_nack_type),
        .train_done_o              (),
        .train_fail_o              (),
        .credit_overflow_o         (),
        .seq_mismatch_o            (),
        .rx_flit_cnt_o             (),
        .crc_err_cnt_o             (),
        .rx_flit_clr_i             (1'b0),
        .crc_err_clr_i             (1'b0),
        .arq_req_flit_o            (rx3_arq_req_flit),
        .arq_req_valid_o           (rx3_arq_req_valid),
        .arq_req_rcv_ptr_o         (rx3_arq_req_rcv_ptr),
        .arq_req_rcv_valid_o       (rx3_arq_req_rcv_valid)
    );
`endif

    // ---- 共享 lcmsm 聚合驱动：deskew_done 全 link 与，retrain_req 全 link 或 ----
    assign tx_lcsm_deskew_done       = tx0_deskew_done
`ifdef TX_LINK1_EXIST
                                       & tx1_deskew_done
`endif
`ifdef TX_LINK2_EXIST
                                       & tx2_deskew_done
`endif
`ifdef TX_LINK3_EXIST
                                       & tx3_deskew_done
`endif
                                       ;
    assign tx_lcsm_retrain_to_req    = tx0_retrain_req[0]
`ifdef TX_LINK1_EXIST
                                       | tx1_retrain_req[0]
`endif
`ifdef TX_LINK2_EXIST
                                       | tx2_retrain_req[0]
`endif
`ifdef TX_LINK3_EXIST
                                       | tx3_retrain_req[0]
`endif
                                       ;
    assign tx_lcsm_retrain_nack_req  = tx0_retrain_req[1]
`ifdef TX_LINK1_EXIST
                                       | tx1_retrain_req[1]
`endif
`ifdef TX_LINK2_EXIST
                                       | tx2_retrain_req[1]
`endif
`ifdef TX_LINK3_EXIST
                                       | tx3_retrain_req[1]
`endif
                                       ;

    // =========================================================================
    // link_ack_route 例化（mb_clk，每 TX 一份；输入所有 link_rx 反向通道，输出到对应 link_tx）
    //   生成通道 round-robin 仲裁 / 解析通道 CSR 无关、按 DLP link_num 命中路由；
    //   RX1/2/3 输入按 RX_LINKx_EXIST 条件连接
    // =========================================================================
    link_ack_route u_link_ack_route0 (
        .mb_clk                (mb_clk),
        .mb_rst_n              (mb_rst_n),
        .csr_ack_route_src_en_i(csr_ack_route_src_en_mb_sync[3:0]),
        .link_number_i       (2'd0),
        // 生成通道输入（all RX）
        .dlv_ack_seq0_i        (rx0_dlv_ack_seq),
        .dlv_ack_valid0_i      (rx0_dlv_ack_valid),
        .lcl_nack_seq0_i       (rx0_lcl_nack_seq),
        .lcl_nack_valid0_i     (rx0_lcl_nack_valid),
        .lcl_nack_type0_i      (rx0_lcl_nack_type),
`ifdef RX_LINK1_EXIST
        .dlv_ack_seq1_i        (rx1_dlv_ack_seq),
        .dlv_ack_valid1_i      (rx1_dlv_ack_valid),
        .lcl_nack_seq1_i       (rx1_lcl_nack_seq),
        .lcl_nack_valid1_i     (rx1_lcl_nack_valid),
        .lcl_nack_type1_i      (rx1_lcl_nack_type),
`else
        .dlv_ack_seq1_i        (8'd0),
        .dlv_ack_valid1_i      (1'b0),
        .lcl_nack_seq1_i       (8'd0),
        .lcl_nack_valid1_i     (1'b0),
        .lcl_nack_type1_i      (2'b10),
`endif
`ifdef RX_LINK2_EXIST
        .dlv_ack_seq2_i        (rx2_dlv_ack_seq),
        .dlv_ack_valid2_i      (rx2_dlv_ack_valid),
        .lcl_nack_seq2_i       (rx2_lcl_nack_seq),
        .lcl_nack_valid2_i     (rx2_lcl_nack_valid),
        .lcl_nack_type2_i      (rx2_lcl_nack_type),
`else
        .dlv_ack_seq2_i        (8'd0),
        .dlv_ack_valid2_i      (1'b0),
        .lcl_nack_seq2_i       (8'd0),
        .lcl_nack_valid2_i     (1'b0),
        .lcl_nack_type2_i      (2'b10),
`endif
`ifdef RX_LINK3_EXIST
        .dlv_ack_seq3_i        (rx3_dlv_ack_seq),
        .dlv_ack_valid3_i      (rx3_dlv_ack_valid),
        .lcl_nack_seq3_i       (rx3_lcl_nack_seq),
        .lcl_nack_valid3_i     (rx3_lcl_nack_valid),
        .lcl_nack_type3_i      (rx3_lcl_nack_type),
`else
        .dlv_ack_seq3_i        (8'd0),
        .dlv_ack_valid3_i      (1'b0),
        .lcl_nack_seq3_i       (8'd0),
        .lcl_nack_valid3_i     (1'b0),
        .lcl_nack_type3_i      (2'b10),
`endif
        // 解析通道输入（all RX）
        .ack_seq0_i            (rx0_ack_seq),
        .ack_valid0_i          (rx0_ack_valid),
        .nack_seq0_i           (rx0_nack_seq),
        .nack_valid0_i         (rx0_nack_valid),
        .dlp_linknum0_i        (rx0_dlp_linknum),
`ifdef RX_LINK1_EXIST
        .ack_seq1_i            (rx1_ack_seq),
        .ack_valid1_i          (rx1_ack_valid),
        .nack_seq1_i           (rx1_nack_seq),
        .nack_valid1_i         (rx1_nack_valid),
        .dlp_linknum1_i        (rx1_dlp_linknum),
`else
        .ack_seq1_i            (8'd0),
        .ack_valid1_i          (1'b0),
        .nack_seq1_i           (8'd0),
        .nack_valid1_i         (1'b0),
        .dlp_linknum1_i        (2'd0),
`endif
`ifdef RX_LINK2_EXIST
        .ack_seq2_i            (rx2_ack_seq),
        .ack_valid2_i          (rx2_ack_valid),
        .nack_seq2_i           (rx2_nack_seq),
        .nack_valid2_i         (rx2_nack_valid),
        .dlp_linknum2_i        (rx2_dlp_linknum),
`else
        .ack_seq2_i            (8'd0),
        .ack_valid2_i          (1'b0),
        .nack_seq2_i           (8'd0),
        .nack_valid2_i         (1'b0),
        .dlp_linknum2_i        (2'd0),
`endif
`ifdef RX_LINK3_EXIST
        .ack_seq3_i            (rx3_ack_seq),
        .ack_valid3_i          (rx3_ack_valid),
        .nack_seq3_i           (rx3_nack_seq),
        .nack_valid3_i         (rx3_nack_valid),
        .dlp_linknum3_i        (rx3_dlp_linknum),
`else
        .ack_seq3_i            (8'd0),
        .ack_valid3_i          (1'b0),
        .nack_seq3_i           (8'd0),
        .nack_valid3_i         (1'b0),
        .dlp_linknum3_i        (2'd0),
`endif
        // 生成输出 → link_tx0
        .dlv_ack_seq_o         (route0_dlv_ack_seq),
        .dlv_ack_valid_o       (route0_dlv_ack_valid),
        .lcl_nack_seq_o        (route0_lcl_nack_seq),
        .lcl_nack_valid_o      (route0_lcl_nack_valid),
        .lcl_nack_type_o       (route0_lcl_nack_type),
        .sel_idx_o             (route0_sel_idx),
        // 解析输出 → link_tx0
        .ack_seq_o             (route0_ack_seq),
        .ack_valid_o           (route0_ack_valid),
        .nack_seq_o            (route0_nack_seq),
        .nack_valid_o          (route0_nack_valid)
    );

`ifdef TX_LINK1_EXIST
    link_ack_route u_link_ack_route1 (
        .mb_clk                (mb_clk),
        .mb_rst_n              (mb_rst_n),
        .csr_ack_route_src_en_i(csr_ack_route_src_en_mb_sync[7:4]),
        .link_number_i       (2'd1),
        // 生成通道输入（all RX）
        .dlv_ack_seq0_i        (rx0_dlv_ack_seq),
        .dlv_ack_valid0_i      (rx0_dlv_ack_valid),
        .lcl_nack_seq0_i       (rx0_lcl_nack_seq),
        .lcl_nack_valid0_i     (rx0_lcl_nack_valid),
        .lcl_nack_type0_i      (rx0_lcl_nack_type),
`ifdef RX_LINK1_EXIST
        .dlv_ack_seq1_i        (rx1_dlv_ack_seq),
        .dlv_ack_valid1_i      (rx1_dlv_ack_valid),
        .lcl_nack_seq1_i       (rx1_lcl_nack_seq),
        .lcl_nack_valid1_i     (rx1_lcl_nack_valid),
        .lcl_nack_type1_i      (rx1_lcl_nack_type),
`else
        .dlv_ack_seq1_i        (8'd0),
        .dlv_ack_valid1_i      (1'b0),
        .lcl_nack_seq1_i       (8'd0),
        .lcl_nack_valid1_i     (1'b0),
        .lcl_nack_type1_i      (2'b10),
`endif
`ifdef RX_LINK2_EXIST
        .dlv_ack_seq2_i        (rx2_dlv_ack_seq),
        .dlv_ack_valid2_i      (rx2_dlv_ack_valid),
        .lcl_nack_seq2_i       (rx2_lcl_nack_seq),
        .lcl_nack_valid2_i     (rx2_lcl_nack_valid),
        .lcl_nack_type2_i      (rx2_lcl_nack_type),
`else
        .dlv_ack_seq2_i        (8'd0),
        .dlv_ack_valid2_i      (1'b0),
        .lcl_nack_seq2_i       (8'd0),
        .lcl_nack_valid2_i     (1'b0),
        .lcl_nack_type2_i      (2'b10),
`endif
`ifdef RX_LINK3_EXIST
        .dlv_ack_seq3_i        (rx3_dlv_ack_seq),
        .dlv_ack_valid3_i      (rx3_dlv_ack_valid),
        .lcl_nack_seq3_i       (rx3_lcl_nack_seq),
        .lcl_nack_valid3_i     (rx3_lcl_nack_valid),
        .lcl_nack_type3_i      (rx3_lcl_nack_type),
`else
        .dlv_ack_seq3_i        (8'd0),
        .dlv_ack_valid3_i      (1'b0),
        .lcl_nack_seq3_i       (8'd0),
        .lcl_nack_valid3_i     (1'b0),
        .lcl_nack_type3_i      (2'b10),
`endif
        // 解析通道输入（all RX）
        .ack_seq0_i            (rx0_ack_seq),
        .ack_valid0_i          (rx0_ack_valid),
        .nack_seq0_i           (rx0_nack_seq),
        .nack_valid0_i         (rx0_nack_valid),
        .dlp_linknum0_i        (rx0_dlp_linknum),
`ifdef RX_LINK1_EXIST
        .ack_seq1_i            (rx1_ack_seq),
        .ack_valid1_i          (rx1_ack_valid),
        .nack_seq1_i           (rx1_nack_seq),
        .nack_valid1_i         (rx1_nack_valid),
        .dlp_linknum1_i        (rx1_dlp_linknum),
`else
        .ack_seq1_i            (8'd0),
        .ack_valid1_i          (1'b0),
        .nack_seq1_i           (8'd0),
        .nack_valid1_i         (1'b0),
        .dlp_linknum1_i        (2'd0),
`endif
`ifdef RX_LINK2_EXIST
        .ack_seq2_i            (rx2_ack_seq),
        .ack_valid2_i          (rx2_ack_valid),
        .nack_seq2_i           (rx2_nack_seq),
        .nack_valid2_i         (rx2_nack_valid),
        .dlp_linknum2_i        (rx2_dlp_linknum),
`else
        .ack_seq2_i            (8'd0),
        .ack_valid2_i          (1'b0),
        .nack_seq2_i           (8'd0),
        .nack_valid2_i         (1'b0),
        .dlp_linknum2_i        (2'd0),
`endif
`ifdef RX_LINK3_EXIST
        .ack_seq3_i            (rx3_ack_seq),
        .ack_valid3_i          (rx3_ack_valid),
        .nack_seq3_i           (rx3_nack_seq),
        .nack_valid3_i         (rx3_nack_valid),
        .dlp_linknum3_i        (rx3_dlp_linknum),
`else
        .ack_seq3_i            (8'd0),
        .ack_valid3_i          (1'b0),
        .nack_seq3_i           (8'd0),
        .nack_valid3_i         (1'b0),
        .dlp_linknum3_i        (2'd0),
`endif
        // 生成输出 → link_tx1
        .dlv_ack_seq_o         (route1_dlv_ack_seq),
        .dlv_ack_valid_o       (route1_dlv_ack_valid),
        .lcl_nack_seq_o        (route1_lcl_nack_seq),
        .lcl_nack_valid_o      (route1_lcl_nack_valid),
        .lcl_nack_type_o       (route1_lcl_nack_type),
        .sel_idx_o             (route1_sel_idx),
        // 解析输出 → link_tx1
        .ack_seq_o             (route1_ack_seq),
        .ack_valid_o           (route1_ack_valid),
        .nack_seq_o            (route1_nack_seq),
        .nack_valid_o          (route1_nack_valid)
    );
`endif

`ifdef TX_LINK2_EXIST
    link_ack_route u_link_ack_route2 (
        .mb_clk                (mb_clk),
        .mb_rst_n              (mb_rst_n),
        .csr_ack_route_src_en_i(csr_ack_route_src_en_mb_sync[11:8]),
        .link_number_i       (2'd2),
        // 生成通道输入（all RX）
        .dlv_ack_seq0_i        (rx0_dlv_ack_seq),
        .dlv_ack_valid0_i      (rx0_dlv_ack_valid),
        .lcl_nack_seq0_i       (rx0_lcl_nack_seq),
        .lcl_nack_valid0_i     (rx0_lcl_nack_valid),
        .lcl_nack_type0_i      (rx0_lcl_nack_type),
`ifdef RX_LINK1_EXIST
        .dlv_ack_seq1_i        (rx1_dlv_ack_seq),
        .dlv_ack_valid1_i      (rx1_dlv_ack_valid),
        .lcl_nack_seq1_i       (rx1_lcl_nack_seq),
        .lcl_nack_valid1_i     (rx1_lcl_nack_valid),
        .lcl_nack_type1_i      (rx1_lcl_nack_type),
`else
        .dlv_ack_seq1_i        (8'd0),
        .dlv_ack_valid1_i      (1'b0),
        .lcl_nack_seq1_i       (8'd0),
        .lcl_nack_valid1_i     (1'b0),
        .lcl_nack_type1_i      (2'b10),
`endif
`ifdef RX_LINK2_EXIST
        .dlv_ack_seq2_i        (rx2_dlv_ack_seq),
        .dlv_ack_valid2_i      (rx2_dlv_ack_valid),
        .lcl_nack_seq2_i       (rx2_lcl_nack_seq),
        .lcl_nack_valid2_i     (rx2_lcl_nack_valid),
        .lcl_nack_type2_i      (rx2_lcl_nack_type),
`else
        .dlv_ack_seq2_i        (8'd0),
        .dlv_ack_valid2_i      (1'b0),
        .lcl_nack_seq2_i       (8'd0),
        .lcl_nack_valid2_i     (1'b0),
        .lcl_nack_type2_i      (2'b10),
`endif
`ifdef RX_LINK3_EXIST
        .dlv_ack_seq3_i        (rx3_dlv_ack_seq),
        .dlv_ack_valid3_i      (rx3_dlv_ack_valid),
        .lcl_nack_seq3_i       (rx3_lcl_nack_seq),
        .lcl_nack_valid3_i     (rx3_lcl_nack_valid),
        .lcl_nack_type3_i      (rx3_lcl_nack_type),
`else
        .dlv_ack_seq3_i        (8'd0),
        .dlv_ack_valid3_i      (1'b0),
        .lcl_nack_seq3_i       (8'd0),
        .lcl_nack_valid3_i     (1'b0),
        .lcl_nack_type3_i      (2'b10),
`endif
        // 解析通道输入（all RX）
        .ack_seq0_i            (rx0_ack_seq),
        .ack_valid0_i          (rx0_ack_valid),
        .nack_seq0_i           (rx0_nack_seq),
        .nack_valid0_i         (rx0_nack_valid),
        .dlp_linknum0_i        (rx0_dlp_linknum),
`ifdef RX_LINK1_EXIST
        .ack_seq1_i            (rx1_ack_seq),
        .ack_valid1_i          (rx1_ack_valid),
        .nack_seq1_i           (rx1_nack_seq),
        .nack_valid1_i         (rx1_nack_valid),
        .dlp_linknum1_i        (rx1_dlp_linknum),
`else
        .ack_seq1_i            (8'd0),
        .ack_valid1_i          (1'b0),
        .nack_seq1_i           (8'd0),
        .nack_valid1_i         (1'b0),
        .dlp_linknum1_i        (2'd0),
`endif
`ifdef RX_LINK2_EXIST
        .ack_seq2_i            (rx2_ack_seq),
        .ack_valid2_i          (rx2_ack_valid),
        .nack_seq2_i           (rx2_nack_seq),
        .nack_valid2_i         (rx2_nack_valid),
        .dlp_linknum2_i        (rx2_dlp_linknum),
`else
        .ack_seq2_i            (8'd0),
        .ack_valid2_i          (1'b0),
        .nack_seq2_i           (8'd0),
        .nack_valid2_i         (1'b0),
        .dlp_linknum2_i        (2'd0),
`endif
`ifdef RX_LINK3_EXIST
        .ack_seq3_i            (rx3_ack_seq),
        .ack_valid3_i          (rx3_ack_valid),
        .nack_seq3_i           (rx3_nack_seq),
        .nack_valid3_i         (rx3_nack_valid),
        .dlp_linknum3_i        (rx3_dlp_linknum),
`else
        .ack_seq3_i            (8'd0),
        .ack_valid3_i          (1'b0),
        .nack_seq3_i           (8'd0),
        .nack_valid3_i         (1'b0),
        .dlp_linknum3_i        (2'd0),
`endif
        // 生成输出 → link_tx2
        .dlv_ack_seq_o         (route2_dlv_ack_seq),
        .dlv_ack_valid_o       (route2_dlv_ack_valid),
        .lcl_nack_seq_o        (route2_lcl_nack_seq),
        .lcl_nack_valid_o      (route2_lcl_nack_valid),
        .lcl_nack_type_o       (route2_lcl_nack_type),
        .sel_idx_o             (route2_sel_idx),
        // 解析输出 → link_tx2
        .ack_seq_o             (route2_ack_seq),
        .ack_valid_o           (route2_ack_valid),
        .nack_seq_o            (route2_nack_seq),
        .nack_valid_o          (route2_nack_valid)
    );
`endif

`ifdef TX_LINK3_EXIST
    link_ack_route u_link_ack_route3 (
        .mb_clk                (mb_clk),
        .mb_rst_n              (mb_rst_n),
        .csr_ack_route_src_en_i(csr_ack_route_src_en_mb_sync[15:12]),
        .link_number_i       (2'd3),
        // 生成通道输入（all RX）
        .dlv_ack_seq0_i        (rx0_dlv_ack_seq),
        .dlv_ack_valid0_i      (rx0_dlv_ack_valid),
        .lcl_nack_seq0_i       (rx0_lcl_nack_seq),
        .lcl_nack_valid0_i     (rx0_lcl_nack_valid),
        .lcl_nack_type0_i      (rx0_lcl_nack_type),
`ifdef RX_LINK1_EXIST
        .dlv_ack_seq1_i        (rx1_dlv_ack_seq),
        .dlv_ack_valid1_i      (rx1_dlv_ack_valid),
        .lcl_nack_seq1_i       (rx1_lcl_nack_seq),
        .lcl_nack_valid1_i     (rx1_lcl_nack_valid),
        .lcl_nack_type1_i      (rx1_lcl_nack_type),
`else
        .dlv_ack_seq1_i        (8'd0),
        .dlv_ack_valid1_i      (1'b0),
        .lcl_nack_seq1_i       (8'd0),
        .lcl_nack_valid1_i     (1'b0),
        .lcl_nack_type1_i      (2'b10),
`endif
`ifdef RX_LINK2_EXIST
        .dlv_ack_seq2_i        (rx2_dlv_ack_seq),
        .dlv_ack_valid2_i      (rx2_dlv_ack_valid),
        .lcl_nack_seq2_i       (rx2_lcl_nack_seq),
        .lcl_nack_valid2_i     (rx2_lcl_nack_valid),
        .lcl_nack_type2_i      (rx2_lcl_nack_type),
`else
        .dlv_ack_seq2_i        (8'd0),
        .dlv_ack_valid2_i      (1'b0),
        .lcl_nack_seq2_i       (8'd0),
        .lcl_nack_valid2_i     (1'b0),
        .lcl_nack_type2_i      (2'b10),
`endif
`ifdef RX_LINK3_EXIST
        .dlv_ack_seq3_i        (rx3_dlv_ack_seq),
        .dlv_ack_valid3_i      (rx3_dlv_ack_valid),
        .lcl_nack_seq3_i       (rx3_lcl_nack_seq),
        .lcl_nack_valid3_i     (rx3_lcl_nack_valid),
        .lcl_nack_type3_i      (rx3_lcl_nack_type),
`else
        .dlv_ack_seq3_i        (8'd0),
        .dlv_ack_valid3_i      (1'b0),
        .lcl_nack_seq3_i       (8'd0),
        .lcl_nack_valid3_i     (1'b0),
        .lcl_nack_type3_i      (2'b10),
`endif
        // 解析通道输入（all RX）
        .ack_seq0_i            (rx0_ack_seq),
        .ack_valid0_i          (rx0_ack_valid),
        .nack_seq0_i           (rx0_nack_seq),
        .nack_valid0_i         (rx0_nack_valid),
        .dlp_linknum0_i        (rx0_dlp_linknum),
`ifdef RX_LINK1_EXIST
        .ack_seq1_i            (rx1_ack_seq),
        .ack_valid1_i          (rx1_ack_valid),
        .nack_seq1_i           (rx1_nack_seq),
        .nack_valid1_i         (rx1_nack_valid),
        .dlp_linknum1_i        (rx1_dlp_linknum),
`else
        .ack_seq1_i            (8'd0),
        .ack_valid1_i          (1'b0),
        .nack_seq1_i           (8'd0),
        .nack_valid1_i         (1'b0),
        .dlp_linknum1_i        (2'd0),
`endif
`ifdef RX_LINK2_EXIST
        .ack_seq2_i            (rx2_ack_seq),
        .ack_valid2_i          (rx2_ack_valid),
        .nack_seq2_i           (rx2_nack_seq),
        .nack_valid2_i         (rx2_nack_valid),
        .dlp_linknum2_i        (rx2_dlp_linknum),
`else
        .ack_seq2_i            (8'd0),
        .ack_valid2_i          (1'b0),
        .nack_seq2_i           (8'd0),
        .nack_valid2_i         (1'b0),
        .dlp_linknum2_i        (2'd0),
`endif
`ifdef RX_LINK3_EXIST
        .ack_seq3_i            (rx3_ack_seq),
        .ack_valid3_i          (rx3_ack_valid),
        .nack_seq3_i           (rx3_nack_seq),
        .nack_valid3_i         (rx3_nack_valid),
        .dlp_linknum3_i        (rx3_dlp_linknum),
`else
        .ack_seq3_i            (8'd0),
        .ack_valid3_i          (1'b0),
        .nack_seq3_i           (8'd0),
        .nack_valid3_i         (1'b0),
        .dlp_linknum3_i        (2'd0),
`endif
        // 生成输出 → link_tx3
        .dlv_ack_seq_o         (route3_dlv_ack_seq),
        .dlv_ack_valid_o       (route3_dlv_ack_valid),
        .lcl_nack_seq_o        (route3_lcl_nack_seq),
        .lcl_nack_valid_o      (route3_lcl_nack_valid),
        .lcl_nack_type_o       (route3_lcl_nack_type),
        .sel_idx_o             (route3_sel_idx),
        // 解析输出 → link_tx3
        .ack_seq_o             (route3_ack_seq),
        .ack_valid_o           (route3_ack_valid),
        .nack_seq_o            (route3_nack_seq),
        .nack_valid_o          (route3_nack_valid)
    );
`endif

    // =========================================================================
    // link_tx_lcmsm 例化（sb_clk，单实例管理所有 TX link）
    // 端口已精简：无 soft_rst_i / work_mode_i / evt_result_i / tx_path_en_o / tx_flush_o / dskew_pattern_o
    // =========================================================================
    link_tx_lcmsm u_link_tx_lcmsm (
        .mb_clk                (mb_clk),
        .mb_rst_n              (mb_rst_n),
        .csr_cfg_done_i        (csr_cfg_done_mb),
        .tx_ready_i            (cpif_tx_link0_ready_i),
        .deskew_done_i         (tx_lcsm_deskew_done),
        .evt_done_i            (evt_done_mb_pos),
        .lp_sw_req_i           (lp_sw_cmd_vld),
        .lp_sw_cmd_i           (lp_sw_cmd_cap),
        .retrain_timeout_req_i (tx_lcsm_retrain_to_req),
        .retrain_nack_req_i    (tx_lcsm_retrain_nack_req),
        .csr_init_retrain_i    (init_retrain_mb_pos),
        .rx_evt_lp_enter_i     (rx_evt_lp_enter_mb_pos),
        .rx_evt_lp_exit_i      (rx_evt_lp_exit_mb_pos),
        .link_state_o          (tx_lcsm_state),
        .link_active_o         (tx_lcsm_active),
        .link_active_nxt_o     (tx_lcsm_active_nxt),   // F1a（LL-20260930-N4-v3）：S_NORMAL 次态位（link_tx ×4 link_active_cp D 源）
        .lp_state_o            (tx_lp_state),
        .link_evt_req_o        (tx_lcsm_evt_req),
        .link_evt_cmd_o        (tx_lcsm_evt_cmd)
    );

    // =========================================================================
    // link_rx_lcmsm 例化（sb_clk，单实例管理所有 RX link）
    // =========================================================================
    link_rx_lcmsm u_link_rx_lcmsm (
        .mb_clk                (mb_clk),
        .mb_rst_n              (mb_rst_n),
        .csr_cfg_done_i        (csr_cfg_done_mb),
        .rx_ready_i            (cpif_rx_link0_ready_i),
        .evt_done_i            (evt_done_mb_pos),
        .lp_sw_req_i           (lp_sw_cmd_vld),
        .lp_sw_cmd_i           (lp_sw_cmd_cap),
        .retrain_timeout_req_i (tx_lcsm_retrain_to_req),
        .retrain_nack_req_i    (tx_lcsm_retrain_nack_req),
        .rx_evt_lp_sync_i      (rx_evt_lp_enter_mb_pos),
        .rx_evt_lp_exit_i      (rx_evt_lp_exit_mb_pos),
        .lane_dskew_done_i     (rx0_train_done),
        .link_state_o          (rx_lcsm_state),
        .rx_path_en_o          (rx_path_en),
        .train_start_o         (rx_train_start),
        .lp_state_o            (rx_lp_state),
        .link_evt_req_o        (rx_lcsm_evt_req),
        .link_evt_cmd_o        (rx_lcsm_evt_cmd)
    );

    // =========================================================================
    // link_event_process 例化（sb_clk，事件接口直连顶层引脚）
    // =========================================================================
    link_event_process u_link_event_process (
        .sb_clk                 (sb_clk),
        .sb_rst_n               (sb_rst_n),
        .lp_sw_cmd_i            (lp_sw_cmd_i),
        .lp_sw_req_i            (lp_sw_req_i),
        .lp_sw_ack_o            (lp_sw_ack_o),
        .lp_sw_sts_o            (lp_sw_sts_o),
        .lcsm_tx_evt_req_i      (tx_lcsm_evt_req_sb_pos),
        .lcsm_tx_evt_cmd_i      (tx_lcsm_evt_cmd_sb),
        .lcsm_rx_evt_req_i      (rx_lcsm_evt_req_sb_pos),
        .lcsm_rx_evt_cmd_i      (rx_lcsm_evt_cmd_sb),
        .csr_init_phy_init_i       (csr_init_phy_init_sb_pos),
        .csr_init_lane_on_i         (csr_init_lane_on_sb_pos),
        .csr_init_lane_off_i        (csr_init_lane_off_sb_pos),
        .csr_init_rx_ready_i        (csr_init_rx_ready_sb_pos),
        .csr_lp_cmd_i               (csr_lp_cmd_sb_pos),
        .cpif_tx_event_enable_o (cpif_tx_event_enable_o),
        .cpif_tx_event_req_o    (cpif_tx_event_req_o),
        .cpif_tx_event_cmd_o    (cpif_tx_event_cmd_o),
        .cpif_tx_event_ack_i    (cpif_tx_event_ack_i),
        .cpif_tx_event_status_i (cpif_tx_event_status_i),
        .cpif_rx_event_enable_o (cpif_rx_event_enable_o),
        .cpif_rx_event_req_i    (cpif_rx_event_req_i),
        .cpif_rx_event_cmd_i    (cpif_rx_event_cmd_i),
        .cpif_rx_event_ack_o    (cpif_rx_event_ack_o),
        .cpif_rx_event_status_o (cpif_rx_event_status_o),
        .evt_done_o             (evt_done),
        .rx_evt_lp_enter_o      (rx_evt_lp_enter),
        .rx_evt_lp_exit_o       (rx_evt_lp_exit),
        .irq_lp_transition_o    (irq_lp_trans_sb),
        .irq_rx_ready_o         (irq_rx_ready_sb),
        .irq_lane_on_done_o     (irq_lane_on_done_sb)
    );

    // =========================================================================
    // link0 CPIF TX 输出驱动
    // =========================================================================
    assign cpif_tx_link0_lane0_data_o     = tx0_cpif_lane0_data;
    assign cpif_tx_link0_lane0_data_vld_o = tx0_cpif_lane0_vld;
`ifdef LINK_LANE1
    assign cpif_tx_link0_lane1_data_o     = tx0_cpif_lane1_data;
    assign cpif_tx_link0_lane1_data_vld_o = tx0_cpif_lane1_vld;
`endif
`ifdef LINK_LANE3
    assign cpif_tx_link0_lane2_data_o     = tx0_cpif_lane2_data;
    assign cpif_tx_link0_lane2_data_vld_o = tx0_cpif_lane2_vld;
    assign cpif_tx_link0_lane3_data_o     = tx0_cpif_lane3_data;
    assign cpif_tx_link0_lane3_data_vld_o = tx0_cpif_lane3_vld;
`endif

endmodule
