// ===================================================================
// d2d_hipi_tb.f - full filelist for the single-DUT (d2d_hipi_dig_top) loopback TB
//
// Version: v1.3 | Date: 2026-10-31
// v1.3: moved to the OpenHappy root layout (code/ + verify/ + sim/ + docs/)
// v1.2: ported into the self-contained project (code/ + verify/ + sim/)
// v1.1: re-layered (RTL / hard IP / TB) with explicit dependencies and ordering;
//       hipi_top_undefines.vh added after the RTL block and before the TB block so the
//       TB side starts from a clean macro window (all TB macros live in d2d_hipi_tb_defines.vh)
//
// -------------------------------------------------------------------
// Environment variables
// -------------------------------------------------------------------
//   HIPINEXT_HOME     = <repo-root>/code     RTL root (derived by the Makefile, overridable)
//   D2D_HIPI_DV_HOME  = <repo-root>/verify   root of this verification environment
//   D2D_SIM_HOME      = <repo-root>/sim      simulation workspace / results
//
// -------------------------------------------------------------------
// Topology (DUT = d2d_hipi_dig_top, single die, digital only)
// -------------------------------------------------------------------
//   tb_top
//     |- d2d_hipi_dig_top  u_dut                 <- the only DUT (digital top)
//     |     |- s0_axi_*  <- axi_master_vip        (DUT acts as AXI slave)
//     |     `- m0_axi_*  -> axi_slave_vip         (DUT acts as AXI master)
//     |     main-band loopback: u_dut.tx_lane0_dataN -> u_dut.rx_lane0_dataN (17 x 64 bit)
//     |     sideband loopback : u_dut.sb_txd -> u_dut.sb_rxd / sb_txr -> sb_rxr
//     |- hipi_apb_*      <- apb_master (apb_if)
//     `- clocks/resets: pclk / mb_clk / phy_rx_clk0 / aclk / sb_clk + 5 resets
//
//   Note: this TB does NOT depend on a LOOPBACK macro - loopback is wired explicitly at
//   tb_top level, and the DUT is the pure-digital d2d_hipi_dig_top (not the 2 x DIG
//   die-vs-die form of d2d_hipi_top).
//
// -------------------------------------------------------------------
// Compile order (strict)
// -------------------------------------------------------------------
//   1) RTL       d2d_hipi_dig_top.f   activates hipi_top_defines.vh itself
//                                    (pcsr -> axi4_prcl -> cxs_prcl -> link_layer ->
//                                     hipi_ctrl_top / common / logic_phy / dig_top)
//   2) Close     hipi_top_undefines.vh  the only cleanup point (top macros die with the RTL block)
//   3) Hard IP   hardip.f (hand-written RTL SRAM model, link_layer replay_buffer)
//                std cells are already RTL (common/xl_std_cell/), no PDK model needed
//   4) TB        defines -> pkg -> if -> vip -> env -> seq -> test -> tb_top
//
//   Constraints:
//     - The RTL block has no `include (macro files are compiled directly by filelist);
//       link_layer.f carries its own +incdir+ and the modules `include "link_layer_defines.vh".
//     - (3) MUST come before (4): the TB-side axi4_if width parameters come from the TB defines
//       but are isolated from the RTL macro window, which is exactly why (2) must close it.
//     - Order inside (4) is sensitive: pkg typedefs (axi_txn) are used by env/seq/test;
//       axi4_if / apb_if must precede the classes that use them; tb_top must be last.
// ===================================================================

// ===================================================================
// (1) RTL: complete D2D digital set (activates the top defines itself)
//    - entry: d2d_hipi_dig_top.f
//    - contains: hipi_top_defines.vh -> hipi_ctrl.f -> common.f -> logic_phy.f -> dig_top.v
//    - shape: 1TX1RX symmetric (AXI_ASYM_MODE undefined -> axi4_symm_prcl),
//             single physical lane (PHY_LANE_1/2/3 all commented out -> lane0 only)
// ===================================================================
-F $HIPINEXT_HOME/design/filelist/d2d_hipi_dig_top.f

// ===================================================================
// (2) Close top macros (single cleanup point; gone once the RTL block ends, leaving the TB
//     in a clean macro window)
// ===================================================================
$HIPINEXT_HOME/design/rtl_v/top/hipi_top_undefines.vh

// ===================================================================
// (3) Hard IP / PDK STD CELL (required for elaboration)
//    - hardip.f   : sram_1rw_128b.v (used by replay_buffer, hand-written RTL, no vendor dependency)
// ===================================================================
-F $HIPINEXT_HOME/design/filelist/hardip.f

// ===================================================================
// ④ TB（顺序敏感）
// ===================================================================

// ---- ④-1 宏与类型（最先）----
$D2D_HIPI_DV_HOME/tb/d2d_hipi_tb_defines.vh      // TB 时钟/超时/位宽/打印宏（RTL 宏不复用）
$D2D_HIPI_DV_HOME/tb/d2d_hipi_tb_pkg.sv          // package：axi_txn 等公共类型

// ---- ④-2 接口（先于使用 virtual interface 的 class）----
$D2D_HIPI_DV_HOME/tb/d2d_hipi_tb_ctrl_if.sv      // 时钟之外的复位/控制（test 驱动）
$D2D_HIPI_DV_HOME/vip/axi/axi4_if.sv             // AXI4 全通道（s0/m0 各一实例）
$D2D_HIPI_DV_HOME/vip/apb/apb_if.sv              // APB3 配置口（20bit 地址）

// ---- ④-3 VIP 与检查器 ----
$D2D_HIPI_DV_HOME/vip/apb/apb_master.sv          // APB master（域封装 apb_write_ctrl/phy）
$D2D_HIPI_DV_HOME/env/d2d_hipi_scoreboard.sv     // scoreboard（mst_exp/mst_act/slv_act 比对）
$D2D_HIPI_DV_HOME/vip/axi/axi_master_vip.sv      // AXI master（驱动 s0_axi，超时检测）
$D2D_HIPI_DV_HOME/vip/axi/axi_slave_vip.sv       // AXI slave （响应 m0_axi，自动回 B/R）
$D2D_HIPI_DV_HOME/vip/axi/axi_monitor.sv         // AXI 采样（s0 请求 / m0 请求 / s0 回程响应）

// ---- ④-4 测试层（base 先于派生）----
$D2D_HIPI_DV_HOME/test/d2d_hipi_test_base.sv     // 复位时序 + APB 配置 + VIP 装配
$D2D_HIPI_DV_HOME/seq/d2d_hipi_seq_lib.sv        // 序列库（单写/单读/并发写读）
$D2D_HIPI_DV_HOME/test/d2d_hipi_test_smoke_1wr1rd.sv  // 用例：并发 1 写 + 1 读

// ---- ④-5 顶层（最后）----
$D2D_HIPI_DV_HOME/tb/d2d_hipi_tb_top.sv          // 时钟/接口/DUT 例化/环回/FSDB/超时
