// ===================================================================
// d2d_hipi_tb_defines.vh — D2D HiPi 验证环境 TB 独立配置宏
//
// 版本：v1.0 | 日期：2026-10-19 | 作者：OpenHappy Team
//
// 用途：TB 侧（tb_pkg / tb_top / vip / env / seq）不直接引用 RTL 编译期宏，
//       统一在本文件定义，取值为 RTL 当前 1T1R 对称配置（axi4_symm_prcl）的实际值。
//
// ★ CRITICAL：本文件必须与 RTL 配置保持同步
//   本环境 DUT = d2d_hipi_dig_top，配置由 hipi_top_defines.vh 决定：
//     PRCL_AXI 定义 / AXI_ASYM_MODE 未定义（→ axi4_symm_prcl 对称 1T1R）
//     MASTER_DIE + SLAVE_DIE 定义（非 iodie）
//     物理层 PHY_LANE_1/2/3 全注释（→ 仅物理 lane0，TX+RX 双向）
//     PRCL_TX_LANE_* / PRCL_RX_LANE_* 全注释（→ 协议层单 lane0）
//     CPIF_DATA_WIDTH_EQ_1024 定义（CPIF = 1024）
//     AXI_DATA_WIDTH = 512（PAIF 512 + 未定义 EQ_256）
//   改动 RTL 配置后须同步本文件，否则 TB 与 DUT 位宽/端口失配。
// ===================================================================

`ifndef D2D_HIPI_TB_DEFINES_VH
`define D2D_HIPI_TB_DEFINES_VH

// ================================================================
// 1. AXI4 用户侧接口参数（对齐 hipi_ctrl_top / dig_top s0_axi、m0_axi 端口）
//    来源：d2d_hipi_dig_top.v AXI 端口声明（hipi_top_defines.vh AXI_DATA_WIDTH=512）
// ================================================================
`define D2D_TB_AXI_ID_WIDTH      10
`define D2D_TB_AXI_ADDR_WIDTH    64
`define D2D_TB_AXI_DATA_WIDTH    512
`define D2D_TB_AXI_STRB_WIDTH    64     // DATA/8
`define D2D_TB_AXI_LEN_WIDTH     8
`define D2D_TB_AXI_SIZE_WIDTH    3
`define D2D_TB_AXI_BURST_WIDTH   2
`define D2D_TB_AXI_CACHE_WIDTH   4
`define D2D_TB_AXI_PROT_WIDTH    3
`define D2D_TB_AXI_QOS_WIDTH     4
`define D2D_TB_AXI_REGION_WIDTH  4
`define D2D_TB_AXI_USER_WIDTH    4

// ================================================================
// 2. 主带（main band）物理接口参数
//    - 单物理 lane（PHY_LANE_* 全注释 → 仅 lane0）
//    - 每 lane 17 个 IO（16 active + 1 redundant RFU），每 IO 64 bit（硬件固定）
//    - io_data_width_mode=0（32:1）时每 IO 仅低 32 bit 有效 → 有效带宽 512 bit
// ================================================================
`define D2D_TB_MAIN_LANE_NUM     1      // 物理 lane 数（1T1R 对称：仅 lane0）
`define D2D_TB_MAIN_IO_NUM       17     // 每 lane IO 数（含 1 冗余 IO）
`define D2D_TB_MAIN_IO_WIDTH     64     // 每 IO 位宽（硬件固定）

// ================================================================
// 3. 时钟周期（单位 ns；timescale = 1ns/1ps）
//    来源：spec/d2d_hipi_dig_top_spec.md §3.1.1（2026-10-12 用户决策：数字主频统一 1 GHz）
//      pclk = mb_clk = phy_rx_clk0 = aclk = 1 GHz（1.0 ns）
//      sb_tx_clk_in = sb_rx_clk = 800 MHz（1.25 ns）
// ================================================================
`define D2D_TB_PCLK_PERIOD       1.0
`define D2D_TB_MB_CLK_PERIOD     1.0
`define D2D_TB_PHY_RX_CLK_PERIOD 1.0
`define D2D_TB_ACLK_PERIOD       1.0
`define D2D_TB_SB_CLK_PERIOD     1.25

// 半周期（always #(半周期) 翻转）；避免在 always 里做实数除法
`define D2D_TB_PCLK_HALF         0.5
`define D2D_TB_MB_CLK_HALF       0.5
`define D2D_TB_PHY_RX_CLK_HALF   0.5
`define D2D_TB_ACLK_HALF         0.5
`define D2D_TB_SB_CLK_HALF       0.625

// ================================================================
// 4. APB 参数（DUT hipi_apb_* 从端口）
//    hipi_apb_paddr[19:0]：[16] 为域选择（logic_phy 内 MUX）
//      0 = phy CSR（apb_csr_top，0x000~0xFFF）
//      1 = ctrl CSR（hipi_ctrl_top，paddr[15:0] 直通）
//    ctrl 域内 16bit 地址分段（hipi_ctrl_top 注释）：
//      paddr[11:10] = 00 → pcsr（0x000~0x3FF）
//                     01/10 → 协议层（0x400~0xBFF）
//                     11 → link layer（0xC00~0xFFF）
// ================================================================
`define D2D_TB_APB_ADDR_WIDTH    20
`define D2D_TB_APB_DATA_WIDTH    32
`define D2D_TB_APB_DOMAIN_PHY    1'b0
`define D2D_TB_APB_DOMAIN_CTRL   1'b1
`define D2D_TB_APB_DOMAIN_BIT    16

// ctrl 域内各段基址（16bit 地址空间内）
`define D2D_TB_SEG_PCSR_BASE     16'h0000
`define D2D_TB_SEG_PRCL_BASE     16'h0400
`define D2D_TB_SEG_LINK_BASE     16'h0C00

// ================================================================
// 5. 超时（时钟周期数）/ 调试开关
// ================================================================
`define D2D_TB_AXI_RESP_TIMEOUT  2000    // AXI 单笔请求等 BVALID/RVALID 的超时拍数
`define D2D_TB_AXI_HANDSHAKE_TIMEOUT 2000 // AXI 通道握手（AWREADY/WREADY/ARREADY）超时拍数
`define D2D_TB_APB_TIMEOUT       512     // APB pready 超时拍数
`define D2D_TB_LINK_POLL_CYCLES  200000  // 等链路 S_NORMAL 的轮询上限（mb_clk 拍）

`endif  // D2D_HIPI_TB_DEFINES_VH

// ===================================================================
// APB 初始化配置常量（对应 flow/ip_cfg/init/hipi_ctrl_top_init_cfg.md §2）
//
//   pcsr 0x0004 clk_en 布局：bit[7:0]=tx_clken[7:0] / bit[15:8]=rx_clken[7:0]
//   文档 4T1R baseline 写 0x00F1（tx_clken[3:0] + rx_clken[0]）。
//   本环境 1T1R 对称（单物理 lane0，TX+RX 双向）：
//     hipi_ctrl_top.v axi4_symm_prcl .clken = tx_clken[0]（唯一消费者），
//     故只需 tx_clken[0] + rx_clken[0] → 0x0011。
// ===================================================================
`define D2D_TB_PCSR_CLK_EN      32'h0000_0011   // 1T1R: tx_clken[0] + rx_clken[0]
`define D2D_TB_PCSR_CLK_EN_4T1R 32'h0000_00F1   // 4T1R 文档 baseline（参考）
