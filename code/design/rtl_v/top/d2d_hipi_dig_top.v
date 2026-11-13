// =============================================================================
// module name：d2d_hipi_dig_top
// author: OpenHappy Team
// MAS/spec：logic_phy_spec.md / hipi_ctrl_top（集成顶层，MAS 待编写）
// 功能概述: D2D HiPi 数字顶层。例化 hipi_ctrl_top（AXI/CXS 协议层 + 链路层 + pcsr，
//           原样不修改，协议层按宏分支选择）与 logic_phy（逻辑物理层）：CPIF 数据/控制/
//           事件内部直连，MCU APB master 内部直连 controller CSR 空间。
//   AXI 用户侧接口四态镜像（AXI_CMPT_DIE → cmptdie 4R1T / AXI_IO_DIE → iodie 4T1R /
//   其他 → 旧 axi4_asymm_prcl），4T1R/4R1T 形态均由宏选择；D2D lane 方向由
//   hipi_top_defines(.vh/_stmr.vh) 的 D2D_PHY_TX/RX_LANE*_EN 控制（4T1R: lane1~3 仅 TX；
//   4R1T: lane1~3 仅 RX）。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   <日期>  <修订人>  <变更概述>
//  2026-08-11  OpenHappy Team  初始版本：logic_phy + hipi_ctrl_top 集成（4 项集成决策
//                       见设计说明 2/3/4/5）
//  2026-08-11  OpenHappy Team  同步 hipi_ctrl_top v0.7：PRCL_CXS 宏移除，用户侧接口
//                       改 PRCL_AXI/`else CXS 二分支（AXI 端口原样透传）；CPIF
//                       两侧等宽 1024 直连（设计说明 2）；APB 选段 [13:12]→[11:10]
//  2026-08-19  OpenHappy Team  mcu_clk 并入 pclk：删除顶层独立 mcu_clk 输入端口，
//                       logic_phy.mcu_clk 与 apb_pclk 均由 pclk 直供（MCU/AHB
//                       配置通路归 pclk 域，详见 d2d_hipi_dig_top_spec.md v0.2）
//  2026-09-01  OpenHappy Team  同步 hipi_ctrl_top v0.18（AXI/iodie 更新）：AXI 用户侧接口改
//                       三态镜像（AXI_IO_DIE / MASTER_DIE / SLAVE_DIE + 裸宏 MST/SLV_x_LANE_*）；
//                       废弃 AXI_MASTER_DIE_MODE/AXI_SLV_*；AXI 数据位宽改用 AXI_DATA_WIDTH；
//                       支持 iodie 模式（s0+m0+m1~3 AR/R，随 SLV_TX_LANE_n）
//  2026-09-18  OpenHappy Team  四态镜像（方法 A）：AXI 用户侧接口与 hipi_ctrl_top 例化区增加
//                       AXI_CMPT_DIE 分支（cmptdie_axi4_prcl，4R1T：s0+m0 lane0 对称骨干 +
//                       s1~3 AR/R 随 MST_RX_LANE_n），分支优先级 AXI_CMPT_DIE > AXI_IO_DIE >
//                       裸宏 asymm；hipi_ctrl_top 例化区新增 cpif_rx_link1~3 lane0 数据连接
//                       （随 PRCL_RX_LANE_1/2/3，4R1T 下使能，4T1R 下编译移除）。
//                       d2d_hipi_dig_top_stmr.v 为本文件纯复制（仅 module 改名为
//                       d2d_hipi_dig_top_stmr），供 2-die top 例化两个 module；4R1T standalone
//                       形态用 hipi_top_defines_stmr.vh + d2d_hipi_dig_top_stmr.f 编译。
// =============================================================================
// 设计说明：
//   1. 宏体系：以 hipi_top_defines.vh 为唯一宏源（d2d_hipi_dig_top.f 编译顺序：
//      -f hipi_ctrl.f（内含 top defines → pcsr → axi4_asymm_prcl.f →
//      cxs_prcl.f → link_layer.f → hipi_ctrl_top → top undefines）→
//      -f common.f → hipi_top_defines.vh 重新激活 → -f logic_phy.f（内含
//      LP defines → logic_phy RTL → LP undefines）→ 本文件 →
//      hipi_top_undefines.vh）。
//      本文件编译时 LP_* 宏已失效（logic_phy.f 尾部自清），lane 存在合宏
//      D2D_LANE{1,2,3}_EN 由 hipi_top_defines.vh §4b 从 PHY_LANE_* 单源派生
//      （语义同 logic_phy_defines.vh 的 LP_LANE{i}_EN），本文件不再内联自派生/自清。
//      controller 侧端口守卫用 PRCL_TX_LANE_*/PRCL_RX_LANE_*（与 hipi_ctrl_top.v 端口
//      条件一致，控制 ctrl link 组数）；logic_phy 侧物理 lane 用 D2D_LANE{1,2,3}_EN
//      （与 logic_phy.v 的 LP_LANE{i}_EN 端口条件等价）。
//      例外：AXI 用户侧接口按 hipi_ctrl_top.v 四态镜像（AXI_CMPT_DIE / AXI_IO_DIE /
//      MASTER_DIE / SLAVE_DIE），分支优先级 AXI_CMPT_DIE > AXI_IO_DIE > asymm。
//      分支与 lane 守卫宏与 hipi_ctrl_top.v 端口一致：cmptdie（AXI_CMPT_DIE，4R1T）仅
//      s0+m0（lane0 对称骨干）+ s1~3 AR/R（随 MST_RX_LANE_n），无 m1~3、无 s1~3 AW/W/B；
//      iodie（AXI_IO_DIE，4T1R）仅 s0+m0（lane0 对称骨干）+ m1~3 AR/R（随 SLV_TX_LANE_n），
//      无 s1~3/m1~3 AW/W/B；master（s*_axi）lane1~3 随 MST_TX/RX_LANE_*；slave（m*_axi）
//      lane1~3 随 SLV_RX/TX_LANE_*。
//      数据位宽用 AXI_DATA_WIDTH（与 hipi_ctrl_top 一致，2026-09-01 OpenHappy Team 同步 hipi_ctrl_top v0.18）。
//      端口行内注释中"见头部假设 N"指向 logic_phy.v 文件头，"§"指向各自主人口。
//   2. CPIF 位宽映射（决策④）：两侧位宽均由 hipi_top_defines.vh 的
//      CPIF_DATA_WIDTH_EQ_1024 控制。默认 1024：link 侧 = logic_phy 侧 = 1024，
//      直连（32:1 运行时模式 link 侧 s2p_mode=512，高 512 恒 0，logic_phy 侧
//      不消费）；`else（link 侧 512）：TX 零扩展 512→1024，RX 截断低 512。
//      64:1 模式（io_data_width_mode=1）本集成不可用，io_data_width_mode_strap
//      在本文件内 tie 1'b0，不引顶端口。
//   3. APB 配置通路（决策①）：logic_phy mcu_subsys APB master（paddr[19:0]）
//      内部直连 hipi_ctrl_top APB slave（paddr[15:0]，[11:10] 选段 pcsr/协议层/
//      link），本顶层无对外 APB 端口；软件经 AHB slave → MCU 配置全部 CSR。
//      软件约束：MCU 侧 protolink 地址映射 paddr[19:16] 必须恒 0（否则越段）。
//   4. 复位（决策②）：5 个独立顶端口 presetn / cxs_rstn / mb_rst_n / sb_rst_n /
//      mcu_rst_n。注意语义不同源：hipi_ctrl_top 内部 link/cxs 复位由 pcsr 软件
//      控制（mb_rstn_pcsr/sb_rstn_pcsr，不导出），logic_phy 三个复位为外部
//      硬复位，软件需自行协调两侧复位释放时序。
//   5. 中断（决策③）：irq_ext（logic_phy 聚合）与 hipi_ctrl_intr（ctrl 聚合）
//      两根独立输出，不做 OR。
//   6. CPIF 其余连接：cpif_tx_ready（logic_phy 输出，暂 tie 1）扇出至各 link
//      ready_i；cpif_rx_ready 无对端，悬空。非对称 lane 配置（如 1T2R/2T1R）
//      下方向不匹配端口：logic_phy 侧 TX 输入 tie 0 / RX 输出悬空。
//   7. hipi_ctrl_top.v 为 OpenHappy Team 所有，本文件仅例化不修改；lp_ctrl / UDI /
//      AXI（PRCL_AXI）或 CXS（`else）/ dfx_stat_o 等对外接口原样透传至顶端口。
// =============================================================================

`timescale 1ns/1ps
`default_nettype none

module d2d_hipi_dig_top (
    // -------------------------------------------------------------------------
    // 时钟与复位
    // -------------------------------------------------------------------------
    input  wire                 pclk,                           // APB 配置时钟（→ ctrl.pclk + logic_phy.apb_pclk + logic_phy.mcu_clk）
    input  wire                 mb_clk,                         // TX 主带时钟（analog PHY 提供，两实例共享）
    input  wire                 phy_rx_clk0,                    // lane0 RX 恢复时钟（analog PHY 提供）
`ifdef D2D_PHY_RX_LANE1_EN
    input  wire                 phy_rx_clk1,                    // lane1 RX 恢复时钟
`endif
`ifdef D2D_PHY_RX_LANE2_EN
    input  wire                 phy_rx_clk2,                    // lane2 RX 恢复时钟
`endif
`ifdef D2D_PHY_RX_LANE3_EN
    input  wire                 phy_rx_clk3,                    // lane3 RX 恢复时钟
`endif
    input  wire                 sb_tx_clk_in,                   // 边带发送时钟源（常跑，analog PHY 提供）
    output wire                 sb_tx_clk,                      // 边带发送随路时钟（sideband 门控输出 → analog PHY）
    input  wire                 sb_rx_clk,                      // 边带接收时钟（对端随路，analog PHY 提供）
`ifdef PRCL_AXI
    input  wire                 aclk,                           // AXI4 用户侧接口时钟
    input  wire                 aresetn,                        // AXI4 用户侧接口复位（async）
`else
    input  wire                 cxs_clk,                        // CXS 协议层时钟
    input  wire                 cxs_rstn,                       // CXS 域复位
`endif
    input  wire                 presetn,                        // APB 复位（async，→ ctrl.presetn）
    input  wire                 mb_rst_n,                       // 主带全局异步复位，低有效（→ logic_phy）
    input  wire                 sb_rst_n,                       // 边带全局异步复位，低有效（→ logic_phy）
    input  wire                 mcu_rst_n,                      // MCU 异步复位，低有效，已同步 pclk（mcu_clk≡pclk，logic_phy apb/mcu 同域）

    // -------------------------------------------------------------------------
`ifdef MCU_SUBSYS
    // AHB slave 接口（→ logic_phy mcu_subsys，见 top_logic_phy_intf v1.2）
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
    // APB slave 接口（→ logic_phy hipi_apb MUX；MCU_SUBSYS 关闭：无 AHB 路径）
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
    // Low-power interface（→ ctrl，参考 cxs 协议层）
    // -------------------------------------------------------------------------
    input  wire [3:0]           lp_ctrl_cmd,
    input  wire                 lp_ctrl_req,
    output wire [3:0]           lp_ctrl_sts,
    output wire                 lp_ctrl_ack,
`ifdef PRCL_AXI
    // ==========================================================================
    //  AXI 用户侧接口 (对外, 按 hipi_ctrl_top 三态镜像: AXI_IO_DIE / MASTER_DIE / SLAVE_DIE)
    //  ── 非 iodie 模式（`ifndef AXI_IO_DIE）：旧 axi4_asymm_prcl，MASTER_DIE/SLAVE_DIE 决定类型。
    // MASTER_DIE → Master Die 用户侧 AXI4 Slave → s*_axi（AW/W/B 属 TX lane
    //        AR/R 属 RX lane；lane1~3 随 MST_TX/RX_LANE_n）；
    // SLAVE_DIE  → Slave Die 用户侧 AXI4 Master → m*_axi（AW/W/B 属 RX lane
    //        AR/R 属 TX lane；lane1~3 随 SLV_RX/TX_LANE_n）；
    //      两方向同模块并存，各按自身宏条件编译（与 hipi_ctrl_top 一致）。
    //  ── iodie 模式（`ifdef AXI_IO_DIE）：iodie_axi4_prcl，s0(完整 slave)+m0(完整 master)
    //       lane0 对称骨干恒在 + lane1~3 仅 TX R 通道（m1~3 AR/R，随 SLV_TX_LANE_n）；
    //       无 s1~3、无 m1~3 AW/W/B（slv RX 扩展 lane）。
    //  数据位宽取 AXI_DATA_WIDTH（PRCL_AXI 分支：PAIF 1024 → 1024；否则
    //    AXI_DATA_WIDTH_EQ_256 → 256；否则默认 512）。与 hipi_ctrl_top 端口一致。
    // ==========================================================================
`ifdef AXI_CMPT_DIE
    // ==========================================================================
    //  AXI 用户侧接口 — cmptdie 模式（cmptdie_axi4_prcl，4R1T）
    //  lane0 对称骨干：s0_axi（完整 5 通道 slave，AW/AR/W 输入、R/B 输出）+
    //    m0_axi（完整 5 通道 master，AW/AR/W 输出、R/B 输入）恒在；
    //  lane1~3 仅 RX AR 通道：s1~3_axi（仅 AR 输入 / R 输出，随 MST_RX_LANE_n）；
    //  无 m1~3、无 s1~3 AW/W/B（cmptdie 无 TX 扩展 lane，PRCL_TX_LANE_* 下禁止定义）。
    // ==========================================================================
    // ---- s0（lane0 对称骨干 slave，用户侧 AXI4 Slave）----
    // AW 通道（input）
    input  wire [9                                : 0] s0_axi_awid                  ,
    input  wire [63                               : 0] s0_axi_awaddr                ,
    input  wire [7                                : 0] s0_axi_awlen                 ,
    input  wire [2                                : 0] s0_axi_awsize                ,
    input  wire [1                                : 0] s0_axi_awburst               ,
    input  wire                                             s0_axi_awlock           ,
    input  wire [3                                : 0] s0_axi_awcache               ,
    input  wire [2                                : 0] s0_axi_awprot                ,
    input  wire [3                                : 0] s0_axi_awqos                 ,
    input  wire [3                                : 0] s0_axi_awregion              ,
    input  wire [3                                : 0] s0_axi_awuser                ,
    input  wire                                             s0_axi_awvalid          ,
    output wire                                             s0_axi_awready          ,
    // AR 通道（input）
    input  wire [9                                : 0] s0_axi_arid                  ,
    input  wire [63                               : 0] s0_axi_araddr                ,
    input  wire [7                                : 0] s0_axi_arlen                 ,
    input  wire [2                                : 0] s0_axi_arsize                ,
    input  wire [1                                : 0] s0_axi_arburst               ,
    input  wire                                             s0_axi_arlock           ,
    input  wire [3                                : 0] s0_axi_arcache               ,
    input  wire [2                                : 0] s0_axi_arprot                ,
    input  wire [3                                : 0] s0_axi_arqos                 ,
    input  wire [3                                : 0] s0_axi_arregion              ,
    input  wire [3                                : 0] s0_axi_aruser                ,
    input  wire                                             s0_axi_arvalid          ,
    output wire                                             s0_axi_arready          ,
    // W 通道（input）
    input  wire [`AXI_DATA_WIDTH-1                : 0] s0_axi_wdata                 ,
    input  wire [`AXI_DATA_WIDTH/8-1              : 0] s0_axi_wstrb                 ,
    input  wire [3                                : 0] s0_axi_wuser                 ,
    input  wire                                             s0_axi_wlast            ,
    input  wire                                             s0_axi_wvalid           ,
    output wire                                             s0_axi_wready           ,
    // R 通道（output）
    output wire [9                                : 0] s0_axi_rid                   ,
    output wire [`AXI_DATA_WIDTH-1                : 0] s0_axi_rdata                 ,
    output wire [1                                : 0] s0_axi_rresp                 ,
    output wire [3                                : 0] s0_axi_ruser                 ,
    output wire                                             s0_axi_rlast            ,
    output wire                                             s0_axi_rvalid           ,
    input  wire                                             s0_axi_rready           ,
    // B 通道（output）
    output wire [9                                : 0] s0_axi_bid                   ,
    output wire [1                                : 0] s0_axi_bresp                 ,
    output wire [3                                : 0] s0_axi_buser                 ,
    output wire                                             s0_axi_bvalid           ,
    input  wire                                             s0_axi_bready           ,
    // ---- s1~3（lane1~3 RX 扩展仅 AR 通道，随 MST_RX_LANE_n）----
`ifdef MST_RX_LANE_1
    // AR 通道（input）
    input  wire [9                                : 0] s1_axi_arid                   ,
    input  wire [63                               : 0] s1_axi_araddr                 ,
    input  wire [7                                : 0] s1_axi_arlen                  ,
    input  wire [2                                : 0] s1_axi_arsize                 ,
    input  wire [1                                : 0] s1_axi_arburst                ,
    input  wire                                        s1_axi_arlock                 ,
    input  wire [3                                : 0] s1_axi_arcache                ,
    input  wire [2                                : 0] s1_axi_arprot                 ,
    input  wire [3                                : 0] s1_axi_arqos                  ,
    input  wire [3                                : 0] s1_axi_arregion               ,
    input  wire [3                                : 0] s1_axi_aruser                 ,
    input  wire                                        s1_axi_arvalid                ,
    output wire                                        s1_axi_arready                ,
    // R 通道（output）
    output wire [9                                : 0] s1_axi_rid                    ,
    output wire [`AXI_DATA_WIDTH-1                : 0] s1_axi_rdata                  ,
    output wire [1                                : 0] s1_axi_rresp                  ,
    output wire [3                                : 0] s1_axi_ruser                  ,
    output wire                                        s1_axi_rlast                  ,
    output wire                                        s1_axi_rvalid                 ,
    input  wire                                        s1_axi_rready                 ,

`endif
`ifdef MST_RX_LANE_2
    // AR 通道（input）
    input  wire [9                                : 0] s2_axi_arid                   ,
    input  wire [63                               : 0] s2_axi_araddr                 ,
    input  wire [7                                : 0] s2_axi_arlen                  ,
    input  wire [2                                : 0] s2_axi_arsize                 ,
    input  wire [1                                : 0] s2_axi_arburst                ,
    input  wire                                        s2_axi_arlock                 ,
    input  wire [3                                : 0] s2_axi_arcache                ,
    input  wire [2                                : 0] s2_axi_arprot                 ,
    input  wire [3                                : 0] s2_axi_arqos                  ,
    input  wire [3                                : 0] s2_axi_arregion               ,
    input  wire [3                                : 0] s2_axi_aruser                 ,
    input  wire                                        s2_axi_arvalid                ,
    output wire                                        s2_axi_arready                ,
    // R 通道（output）
    output wire [9                                : 0] s2_axi_rid                    ,
    output wire [`AXI_DATA_WIDTH-1                : 0] s2_axi_rdata                  ,
    output wire [1                                : 0] s2_axi_rresp                  ,
    output wire [3                                : 0] s2_axi_ruser                  ,
    output wire                                        s2_axi_rlast                  ,
    output wire                                        s2_axi_rvalid                 ,
    input  wire                                        s2_axi_rready                 ,

`endif
`ifdef MST_RX_LANE_3
    // AR 通道（input）
    input  wire [9                                : 0] s3_axi_arid                   ,
    input  wire [63                               : 0] s3_axi_araddr                 ,
    input  wire [7                                : 0] s3_axi_arlen                  ,
    input  wire [2                                : 0] s3_axi_arsize                 ,
    input  wire [1                                : 0] s3_axi_arburst                ,
    input  wire                                        s3_axi_arlock                 ,
    input  wire [3                                : 0] s3_axi_arcache                ,
    input  wire [2                                : 0] s3_axi_arprot                 ,
    input  wire [3                                : 0] s3_axi_arqos                  ,
    input  wire [3                                : 0] s3_axi_arregion               ,
    input  wire [3                                : 0] s3_axi_aruser                 ,
    input  wire                                        s3_axi_arvalid                ,
    output wire                                        s3_axi_arready                ,
    // R 通道（output）
    output wire [9                                : 0] s3_axi_rid                    ,
    output wire [`AXI_DATA_WIDTH-1                : 0] s3_axi_rdata                  ,
    output wire [1                                : 0] s3_axi_rresp                  ,
    output wire [3                                : 0] s3_axi_ruser                  ,
    output wire                                        s3_axi_rlast                  ,
    output wire                                        s3_axi_rvalid                 ,
    input  wire                                        s3_axi_rready                 ,

`endif
    // ---- m0（lane0 对称骨干 master，用户侧 AXI4 Master）----
    // AW 通道（output）
    output wire [9                                : 0] m0_axi_awid                  ,
    output wire [63                               : 0] m0_axi_awaddr                ,
    output wire [7                                : 0] m0_axi_awlen                 ,
    output wire [2                                : 0] m0_axi_awsize                ,
    output wire [1                                : 0] m0_axi_awburst               ,
    output wire                                             m0_axi_awlock           ,
    output wire [3                                : 0] m0_axi_awcache               ,
    output wire [2                                : 0] m0_axi_awprot                ,
    output wire [3                                : 0] m0_axi_awqos                 ,
    output wire [3                                : 0] m0_axi_awregion              ,
    output wire [3                                : 0] m0_axi_awuser                ,
    output wire                                             m0_axi_awvalid          ,
    input  wire                                             m0_axi_awready          ,
    // AR 通道（output）
    output wire [9                                : 0] m0_axi_arid                  ,
    output wire [63                               : 0] m0_axi_araddr                ,
    output wire [7                                : 0] m0_axi_arlen                 ,
    output wire [2                                : 0] m0_axi_arsize                ,
    output wire [1                                : 0] m0_axi_arburst               ,
    output wire                                             m0_axi_arlock           ,
    output wire [3                                : 0] m0_axi_arcache               ,
    output wire [2                                : 0] m0_axi_arprot                ,
    output wire [3                                : 0] m0_axi_arqos                 ,
    output wire [3                                : 0] m0_axi_arregion              ,
    output wire [3                                : 0] m0_axi_aruser                ,
    output wire                                             m0_axi_arvalid          ,
    input  wire                                             m0_axi_arready          ,
    // W 通道（output）
    output wire [`AXI_DATA_WIDTH-1                : 0] m0_axi_wdata                 ,
    output wire [`AXI_DATA_WIDTH/8-1              : 0] m0_axi_wstrb                 ,
    output wire [3                                : 0] m0_axi_wuser                 ,
    output wire                                             m0_axi_wlast            ,
    output wire                                             m0_axi_wvalid           ,
    input  wire                                             m0_axi_wready           ,
    // R 通道（input）
    input  wire [9                                : 0] m0_axi_rid                   ,
    input  wire [`AXI_DATA_WIDTH-1                : 0] m0_axi_rdata                 ,
    input  wire [1                                : 0] m0_axi_rresp                 ,
    input  wire [3                                : 0] m0_axi_ruser                 ,
    input  wire                                             m0_axi_rlast            ,
    input  wire                                             m0_axi_rvalid           ,
    output wire                                             m0_axi_rready           ,
    // B 通道（input）
    input  wire [9                                : 0] m0_axi_bid                   ,
    input  wire [1                                : 0] m0_axi_bresp                 ,
    input  wire [3                                : 0] m0_axi_buser                 ,
    input  wire                                             m0_axi_bvalid           ,
    output wire                                             m0_axi_bready           ,
`elsif AXI_IO_DIE
    // ==========================================================================
    //  AXI 用户侧接口 — iodie 模式（iodie_axi4_prcl）
    //  lane0 对称骨干：s0_axi（完整 5 通道 slave，AW/AR/W 输入、R/B 输出）+
    //    m0_axi（完整 5 通道 master，AW/AR/W 输出、R/B 输入）恒在；
    //  lane1~3 仅 TX R 通道：m1~3_axi（仅 AR 输出 / R 输入，随 SLV_TX_LANE_n）；
    //  无 s1~3、无 m1~3 AW/W/B（iodie 无 slv RX 扩展 lane，PRCL_RX_LANE_* 下禁止定义）。
    // ==========================================================================
    // ---- s0（lane0 对称骨干 slave，用户侧 AXI4 Slave）----
    // AW 通道（input）
    input  wire [9                                : 0] s0_axi_awid                  ,
    input  wire [63                               : 0] s0_axi_awaddr                ,
    input  wire [7                                : 0] s0_axi_awlen                 ,
    input  wire [2                                : 0] s0_axi_awsize                ,
    input  wire [1                                : 0] s0_axi_awburst               ,
    input  wire                                             s0_axi_awlock           ,
    input  wire [3                                : 0] s0_axi_awcache               ,
    input  wire [2                                : 0] s0_axi_awprot                ,
    input  wire [3                                : 0] s0_axi_awqos                 ,
    input  wire [3                                : 0] s0_axi_awregion              ,
    input  wire [3                                : 0] s0_axi_awuser                ,
    input  wire                                             s0_axi_awvalid          ,
    output wire                                             s0_axi_awready          ,
    // AR 通道（input）
    input  wire [9                                : 0] s0_axi_arid                  ,
    input  wire [63                               : 0] s0_axi_araddr                ,
    input  wire [7                                : 0] s0_axi_arlen                 ,
    input  wire [2                                : 0] s0_axi_arsize                ,
    input  wire [1                                : 0] s0_axi_arburst               ,
    input  wire                                             s0_axi_arlock           ,
    input  wire [3                                : 0] s0_axi_arcache               ,
    input  wire [2                                : 0] s0_axi_arprot                ,
    input  wire [3                                : 0] s0_axi_arqos                 ,
    input  wire [3                                : 0] s0_axi_arregion              ,
    input  wire [3                                : 0] s0_axi_aruser                ,
    input  wire                                             s0_axi_arvalid          ,
    output wire                                             s0_axi_arready          ,
    // W 通道（input）
    input  wire [`AXI_DATA_WIDTH-1                : 0] s0_axi_wdata                 ,
    input  wire [`AXI_DATA_WIDTH/8-1              : 0] s0_axi_wstrb                 ,
    input  wire [3                                : 0] s0_axi_wuser                 ,
    input  wire                                             s0_axi_wlast            ,
    input  wire                                             s0_axi_wvalid           ,
    output wire                                             s0_axi_wready           ,
    // R 通道（output）
    output wire [9                                : 0] s0_axi_rid                   ,
    output wire [`AXI_DATA_WIDTH-1                : 0] s0_axi_rdata                 ,
    output wire [1                                : 0] s0_axi_rresp                 ,
    output wire [3                                : 0] s0_axi_ruser                 ,
    output wire                                             s0_axi_rlast            ,
    output wire                                             s0_axi_rvalid           ,
    input  wire                                             s0_axi_rready           ,
    // B 通道（output）
    output wire [9                                : 0] s0_axi_bid                   ,
    output wire [1                                : 0] s0_axi_bresp                 ,
    output wire [3                                : 0] s0_axi_buser                 ,
    output wire                                             s0_axi_bvalid           ,
    input  wire                                             s0_axi_bready           ,
    // ---- m0（lane0 对称骨干 master，用户侧 AXI4 Master）----
    // AW 通道（output）
    output wire [9                                : 0] m0_axi_awid                  ,
    output wire [63                               : 0] m0_axi_awaddr                ,
    output wire [7                                : 0] m0_axi_awlen                 ,
    output wire [2                                : 0] m0_axi_awsize                ,
    output wire [1                                : 0] m0_axi_awburst               ,
    output wire                                             m0_axi_awlock           ,
    output wire [3                                : 0] m0_axi_awcache               ,
    output wire [2                                : 0] m0_axi_awprot                ,
    output wire [3                                : 0] m0_axi_awqos                 ,
    output wire [3                                : 0] m0_axi_awregion              ,
    output wire [3                                : 0] m0_axi_awuser                ,
    output wire                                             m0_axi_awvalid          ,
    input  wire                                             m0_axi_awready          ,
    // AR 通道（output）
    output wire [9                                : 0] m0_axi_arid                  ,
    output wire [63                               : 0] m0_axi_araddr                ,
    output wire [7                                : 0] m0_axi_arlen                 ,
    output wire [2                                : 0] m0_axi_arsize                ,
    output wire [1                                : 0] m0_axi_arburst               ,
    output wire                                             m0_axi_arlock           ,
    output wire [3                                : 0] m0_axi_arcache               ,
    output wire [2                                : 0] m0_axi_arprot                ,
    output wire [3                                : 0] m0_axi_arqos                 ,
    output wire [3                                : 0] m0_axi_arregion              ,
    output wire [3                                : 0] m0_axi_aruser                ,
    output wire                                             m0_axi_arvalid          ,
    input  wire                                             m0_axi_arready          ,
    // W 通道（output）
    output wire [`AXI_DATA_WIDTH-1                : 0] m0_axi_wdata                 ,
    output wire [`AXI_DATA_WIDTH/8-1              : 0] m0_axi_wstrb                 ,
    output wire [3                                : 0] m0_axi_wuser                 ,
    output wire                                             m0_axi_wlast            ,
    output wire                                             m0_axi_wvalid           ,
    input  wire                                             m0_axi_wready           ,
    // R 通道（input）
    input  wire [9                                : 0] m0_axi_rid                   ,
    input  wire [`AXI_DATA_WIDTH-1                : 0] m0_axi_rdata                 ,
    input  wire [1                                : 0] m0_axi_rresp                 ,
    input  wire [3                                : 0] m0_axi_ruser                 ,
    input  wire                                             m0_axi_rlast            ,
    input  wire                                             m0_axi_rvalid           ,
    output wire                                             m0_axi_rready           ,
    // B 通道（input）
    input  wire [9                                : 0] m0_axi_bid                   ,
    input  wire [1                                : 0] m0_axi_bresp                 ,
    input  wire [3                                : 0] m0_axi_buser                 ,
    input  wire                                             m0_axi_bvalid           ,
    output wire                                             m0_axi_bready           ,
    // ---- m1~3（lane1~3 仅 TX R 通道，随 SLV_TX_LANE_n）----
    // AR 通道（output）/ R 通道（input）
`ifdef SLV_TX_LANE_1
    output wire [9                                : 0] m1_axi_arid                  ,
    output wire [63                               : 0] m1_axi_araddr                ,
    output wire [7                                : 0] m1_axi_arlen                 ,
    output wire [2                                : 0] m1_axi_arsize                ,
    output wire [1                                : 0] m1_axi_arburst               ,
    output wire                                             m1_axi_arlock           ,
    output wire [3                                : 0] m1_axi_arcache               ,
    output wire [2                                : 0] m1_axi_arprot                ,
    output wire [3                                : 0] m1_axi_arqos                 ,
    output wire [3                                : 0] m1_axi_arregion              ,
    output wire [3                                : 0] m1_axi_aruser                ,
    output wire                                             m1_axi_arvalid          ,
    input  wire                                             m1_axi_arready          ,
    input  wire [9                                : 0] m1_axi_rid                   ,
    input  wire [`AXI_DATA_WIDTH-1                : 0] m1_axi_rdata                 ,
    input  wire [1                                : 0] m1_axi_rresp                 ,
    input  wire [3                                : 0] m1_axi_ruser                 ,
    input  wire                                             m1_axi_rlast            ,
    input  wire                                             m1_axi_rvalid           ,
    output wire                                             m1_axi_rready           ,
`endif
`ifdef SLV_TX_LANE_2
    output wire [9                                : 0] m2_axi_arid                  ,
    output wire [63                               : 0] m2_axi_araddr                ,
    output wire [7                                : 0] m2_axi_arlen                 ,
    output wire [2                                : 0] m2_axi_arsize                ,
    output wire [1                                : 0] m2_axi_arburst               ,
    output wire                                             m2_axi_arlock           ,
    output wire [3                                : 0] m2_axi_arcache               ,
    output wire [2                                : 0] m2_axi_arprot                ,
    output wire [3                                : 0] m2_axi_arqos                 ,
    output wire [3                                : 0] m2_axi_arregion              ,
    output wire [3                                : 0] m2_axi_aruser                ,
    output wire                                             m2_axi_arvalid          ,
    input  wire                                             m2_axi_arready          ,
    input  wire [9                                : 0] m2_axi_rid                   ,
    input  wire [`AXI_DATA_WIDTH-1                : 0] m2_axi_rdata                 ,
    input  wire [1                                : 0] m2_axi_rresp                 ,
    input  wire [3                                : 0] m2_axi_ruser                 ,
    input  wire                                             m2_axi_rlast            ,
    input  wire                                             m2_axi_rvalid           ,
    output wire                                             m2_axi_rready           ,
`endif
`ifdef SLV_TX_LANE_3
    output wire [9                                : 0] m3_axi_arid                  ,
    output wire [63                               : 0] m3_axi_araddr                ,
    output wire [7                                : 0] m3_axi_arlen                 ,
    output wire [2                                : 0] m3_axi_arsize                ,
    output wire [1                                : 0] m3_axi_arburst               ,
    output wire                                             m3_axi_arlock           ,
    output wire [3                                : 0] m3_axi_arcache               ,
    output wire [2                                : 0] m3_axi_arprot                ,
    output wire [3                                : 0] m3_axi_arqos                 ,
    output wire [3                                : 0] m3_axi_arregion              ,
    output wire [3                                : 0] m3_axi_aruser                ,
    output wire                                             m3_axi_arvalid          ,
    input  wire                                             m3_axi_arready          ,
    input  wire [9                                : 0] m3_axi_rid                   ,
    input  wire [`AXI_DATA_WIDTH-1                : 0] m3_axi_rdata                 ,
    input  wire [1                                : 0] m3_axi_rresp                 ,
    input  wire [3                                : 0] m3_axi_ruser                 ,
    input  wire                                             m3_axi_rlast            ,
    input  wire                                             m3_axi_rvalid           ,
    output wire                                             m3_axi_rready           ,
`endif
`else
`ifdef AXI_ASYM_MODE
`ifdef MASTER_DIE
    // ---- req 方向（Master Die，用户侧 AXI4 Slave）----
    // AW 通道（input，s0 恒在；s1~3 随 MST_TX_LANE_n）
    input  wire [9                                : 0] s0_axi_awid                  ,
    input  wire [63                               : 0] s0_axi_awaddr                ,
    input  wire [7                                : 0] s0_axi_awlen                 ,
    input  wire [2                                : 0] s0_axi_awsize                ,
    input  wire [1                                : 0] s0_axi_awburst               ,
    input  wire                                             s0_axi_awlock           ,
    input  wire [3                                : 0] s0_axi_awcache               ,
    input  wire [2                                : 0] s0_axi_awprot                ,
    input  wire [3                                : 0] s0_axi_awqos                 ,
    input  wire [3                                : 0] s0_axi_awregion              ,
    input  wire [3                                : 0] s0_axi_awuser                ,
    input  wire                                             s0_axi_awvalid          ,
    output wire                                             s0_axi_awready          ,
`ifdef MST_TX_LANE_1
    input  wire [9                                : 0] s1_axi_awid                  ,
    input  wire [63                               : 0] s1_axi_awaddr                ,
    input  wire [7                                : 0] s1_axi_awlen                 ,
    input  wire [2                                : 0] s1_axi_awsize                ,
    input  wire [1                                : 0] s1_axi_awburst               ,
    input  wire                                             s1_axi_awlock           ,
    input  wire [3                                : 0] s1_axi_awcache               ,
    input  wire [2                                : 0] s1_axi_awprot                ,
    input  wire [3                                : 0] s1_axi_awqos                 ,
    input  wire [3                                : 0] s1_axi_awregion              ,
    input  wire [3                                : 0] s1_axi_awuser                ,
    input  wire                                             s1_axi_awvalid          ,
    output wire                                             s1_axi_awready          ,
`endif
`ifdef MST_TX_LANE_2
    input  wire [9                                : 0] s2_axi_awid                  ,
    input  wire [63                               : 0] s2_axi_awaddr                ,
    input  wire [7                                : 0] s2_axi_awlen                 ,
    input  wire [2                                : 0] s2_axi_awsize                ,
    input  wire [1                                : 0] s2_axi_awburst               ,
    input  wire                                             s2_axi_awlock           ,
    input  wire [3                                : 0] s2_axi_awcache               ,
    input  wire [2                                : 0] s2_axi_awprot                ,
    input  wire [3                                : 0] s2_axi_awqos                 ,
    input  wire [3                                : 0] s2_axi_awregion              ,
    input  wire [3                                : 0] s2_axi_awuser                ,
    input  wire                                             s2_axi_awvalid          ,
    output wire                                             s2_axi_awready          ,
`endif
`ifdef MST_TX_LANE_3
    input  wire [9                                : 0] s3_axi_awid                  ,
    input  wire [63                               : 0] s3_axi_awaddr                ,
    input  wire [7                                : 0] s3_axi_awlen                 ,
    input  wire [2                                : 0] s3_axi_awsize                ,
    input  wire [1                                : 0] s3_axi_awburst               ,
    input  wire                                             s3_axi_awlock           ,
    input  wire [3                                : 0] s3_axi_awcache               ,
    input  wire [2                                : 0] s3_axi_awprot                ,
    input  wire [3                                : 0] s3_axi_awqos                 ,
    input  wire [3                                : 0] s3_axi_awregion              ,
    input  wire [3                                : 0] s3_axi_awuser                ,
    input  wire                                             s3_axi_awvalid          ,
    output wire                                             s3_axi_awready          ,
`endif
    // AR 通道（input，s0 恒在；s1~3 随 MST_RX_LANE_n）
    input  wire [9                                : 0] s0_axi_arid                  ,
    input  wire [63                               : 0] s0_axi_araddr                ,
    input  wire [7                                : 0] s0_axi_arlen                 ,
    input  wire [2                                : 0] s0_axi_arsize                ,
    input  wire [1                                : 0] s0_axi_arburst               ,
    input  wire                                             s0_axi_arlock           ,
    input  wire [3                                : 0] s0_axi_arcache               ,
    input  wire [2                                : 0] s0_axi_arprot                ,
    input  wire [3                                : 0] s0_axi_arqos                 ,
    input  wire [3                                : 0] s0_axi_arregion              ,
    input  wire [3                                : 0] s0_axi_aruser                ,
    input  wire                                             s0_axi_arvalid          ,
    output wire                                             s0_axi_arready          ,
`ifdef MST_RX_LANE_1
    input  wire [9                                : 0] s1_axi_arid                  ,
    input  wire [63                               : 0] s1_axi_araddr                ,
    input  wire [7                                : 0] s1_axi_arlen                 ,
    input  wire [2                                : 0] s1_axi_arsize                ,
    input  wire [1                                : 0] s1_axi_arburst               ,
    input  wire                                             s1_axi_arlock           ,
    input  wire [3                                : 0] s1_axi_arcache               ,
    input  wire [2                                : 0] s1_axi_arprot                ,
    input  wire [3                                : 0] s1_axi_arqos                 ,
    input  wire [3                                : 0] s1_axi_arregion              ,
    input  wire [3                                : 0] s1_axi_aruser                ,
    input  wire                                             s1_axi_arvalid          ,
    output wire                                             s1_axi_arready          ,
`endif
`ifdef MST_RX_LANE_2
    input  wire [9                                : 0] s2_axi_arid                  ,
    input  wire [63                               : 0] s2_axi_araddr                ,
    input  wire [7                                : 0] s2_axi_arlen                 ,
    input  wire [2                                : 0] s2_axi_arsize                ,
    input  wire [1                                : 0] s2_axi_arburst               ,
    input  wire                                             s2_axi_arlock           ,
    input  wire [3                                : 0] s2_axi_arcache               ,
    input  wire [2                                : 0] s2_axi_arprot                ,
    input  wire [3                                : 0] s2_axi_arqos                 ,
    input  wire [3                                : 0] s2_axi_arregion              ,
    input  wire [3                                : 0] s2_axi_aruser                ,
    input  wire                                             s2_axi_arvalid          ,
    output wire                                             s2_axi_arready          ,
`endif
`ifdef MST_RX_LANE_3
    input  wire [9                                : 0] s3_axi_arid                  ,
    input  wire [63                               : 0] s3_axi_araddr                ,
    input  wire [7                                : 0] s3_axi_arlen                 ,
    input  wire [2                                : 0] s3_axi_arsize                ,
    input  wire [1                                : 0] s3_axi_arburst               ,
    input  wire                                             s3_axi_arlock           ,
    input  wire [3                                : 0] s3_axi_arcache               ,
    input  wire [2                                : 0] s3_axi_arprot                ,
    input  wire [3                                : 0] s3_axi_arqos                 ,
    input  wire [3                                : 0] s3_axi_arregion              ,
    input  wire [3                                : 0] s3_axi_aruser                ,
    input  wire                                             s3_axi_arvalid          ,
    output wire                                             s3_axi_arready          ,
`endif
    // W 通道（input，s0 恒在；s1~3 随 MST_TX_LANE_n）
    input  wire [`AXI_DATA_WIDTH-1                : 0] s0_axi_wdata                 ,
    input  wire [`AXI_DATA_WIDTH/8-1              : 0] s0_axi_wstrb                 ,
    input  wire [3                                : 0] s0_axi_wuser                 ,
    input  wire                                             s0_axi_wlast            ,
    input  wire                                             s0_axi_wvalid           ,
    output wire                                             s0_axi_wready           ,
`ifdef MST_TX_LANE_1
    input  wire [`AXI_DATA_WIDTH-1                : 0] s1_axi_wdata                 ,
    input  wire [`AXI_DATA_WIDTH/8-1              : 0] s1_axi_wstrb                 ,
    input  wire [3                                : 0] s1_axi_wuser                 ,
    input  wire                                             s1_axi_wlast            ,
    input  wire                                             s1_axi_wvalid           ,
    output wire                                             s1_axi_wready           ,
`endif
`ifdef MST_TX_LANE_2
    input  wire [`AXI_DATA_WIDTH-1                : 0] s2_axi_wdata                 ,
    input  wire [`AXI_DATA_WIDTH/8-1              : 0] s2_axi_wstrb                 ,
    input  wire [3                                : 0] s2_axi_wuser                 ,
    input  wire                                             s2_axi_wlast            ,
    input  wire                                             s2_axi_wvalid           ,
    output wire                                             s2_axi_wready           ,
`endif
`ifdef MST_TX_LANE_3
    input  wire [`AXI_DATA_WIDTH-1                : 0] s3_axi_wdata                 ,
    input  wire [`AXI_DATA_WIDTH/8-1              : 0] s3_axi_wstrb                 ,
    input  wire [3                                : 0] s3_axi_wuser                 ,
    input  wire                                             s3_axi_wlast            ,
    input  wire                                             s3_axi_wvalid           ,
    output wire                                             s3_axi_wready           ,
`endif
    // R 通道（output，s0 恒在；s1~3 随 MST_RX_LANE_n）
    output wire [9                                : 0] s0_axi_rid                   ,
    output wire [`AXI_DATA_WIDTH-1                : 0] s0_axi_rdata                 ,
    output wire [1                                : 0] s0_axi_rresp                 ,
    output wire [3                                : 0] s0_axi_ruser                 ,
    output wire                                             s0_axi_rlast            ,
    output wire                                             s0_axi_rvalid           ,
    input  wire                                             s0_axi_rready           ,
`ifdef MST_RX_LANE_1
    output wire [9                                : 0] s1_axi_rid                   ,
    output wire [`AXI_DATA_WIDTH-1                : 0] s1_axi_rdata                 ,
    output wire [1                                : 0] s1_axi_rresp                 ,
    output wire [3                                : 0] s1_axi_ruser                 ,
    output wire                                             s1_axi_rlast            ,
    output wire                                             s1_axi_rvalid           ,
    input  wire                                             s1_axi_rready           ,
`endif
`ifdef MST_RX_LANE_2
    output wire [9                                : 0] s2_axi_rid                   ,
    output wire [`AXI_DATA_WIDTH-1                : 0] s2_axi_rdata                 ,
    output wire [1                                : 0] s2_axi_rresp                 ,
    output wire [3                                : 0] s2_axi_ruser                 ,
    output wire                                             s2_axi_rlast            ,
    output wire                                             s2_axi_rvalid           ,
    input  wire                                             s2_axi_rready           ,
`endif
`ifdef MST_RX_LANE_3
    output wire [9                                : 0] s3_axi_rid                   ,
    output wire [`AXI_DATA_WIDTH-1                : 0] s3_axi_rdata                 ,
    output wire [1                                : 0] s3_axi_rresp                 ,
    output wire [3                                : 0] s3_axi_ruser                 ,
    output wire                                             s3_axi_rlast            ,
    output wire                                             s3_axi_rvalid           ,
    input  wire                                             s3_axi_rready           ,
`endif
    // B 通道（output，s0 恒在；s1~3 随 MST_TX_LANE_n）
    output wire [9                                : 0] s0_axi_bid                   ,
    output wire [1                                : 0] s0_axi_bresp                 ,
    output wire [3                                : 0] s0_axi_buser                 ,
    output wire                                             s0_axi_bvalid           ,
    input  wire                                             s0_axi_bready           ,
`ifdef MST_TX_LANE_1
    output wire [9                                : 0] s1_axi_bid                   ,
    output wire [1                                : 0] s1_axi_bresp                 ,
    output wire [3                                : 0] s1_axi_buser                 ,
    output wire                                             s1_axi_bvalid           ,
    input  wire                                             s1_axi_bready           ,
`endif
`ifdef MST_TX_LANE_2
    output wire [9                                : 0] s2_axi_bid                   ,
    output wire [1                                : 0] s2_axi_bresp                 ,
    output wire [3                                : 0] s2_axi_buser                 ,
    output wire                                             s2_axi_bvalid           ,
    input  wire                                             s2_axi_bready           ,
`endif
`ifdef MST_TX_LANE_3
    output wire [9                                : 0] s3_axi_bid                   ,
    output wire [1                                : 0] s3_axi_bresp                 ,
    output wire [3                                : 0] s3_axi_buser                 ,
    output wire                                             s3_axi_bvalid           ,
    input  wire                                             s3_axi_bready           ,
`endif
`endif
`ifdef SLAVE_DIE
    // ---- resp 方向（Slave Die，用户侧 AXI4 Master）----
    // AW 通道（output，m0 恒在；m1~3 随 SLV_RX_LANE_n）
    output wire [9                                : 0] m0_axi_awid                  ,
    output wire [63                               : 0] m0_axi_awaddr                ,
    output wire [7                                : 0] m0_axi_awlen                 ,
    output wire [2                                : 0] m0_axi_awsize                ,
    output wire [1                                : 0] m0_axi_awburst               ,
    output wire                                             m0_axi_awlock           ,
    output wire [3                                : 0] m0_axi_awcache               ,
    output wire [2                                : 0] m0_axi_awprot                ,
    output wire [3                                : 0] m0_axi_awqos                 ,
    output wire [3                                : 0] m0_axi_awregion              ,
    output wire [3                                : 0] m0_axi_awuser                ,
    output wire                                             m0_axi_awvalid          ,
    input  wire                                             m0_axi_awready          ,
`ifdef SLV_RX_LANE_1
    output wire [9                                : 0] m1_axi_awid                  ,
    output wire [63                               : 0] m1_axi_awaddr                ,
    output wire [7                                : 0] m1_axi_awlen                 ,
    output wire [2                                : 0] m1_axi_awsize                ,
    output wire [1                                : 0] m1_axi_awburst               ,
    output wire                                             m1_axi_awlock           ,
    output wire [3                                : 0] m1_axi_awcache               ,
    output wire [2                                : 0] m1_axi_awprot                ,
    output wire [3                                : 0] m1_axi_awqos                 ,
    output wire [3                                : 0] m1_axi_awregion              ,
    output wire [3                                : 0] m1_axi_awuser                ,
    output wire                                             m1_axi_awvalid          ,
    input  wire                                             m1_axi_awready          ,
`endif
`ifdef SLV_RX_LANE_2
    output wire [9                                : 0] m2_axi_awid                  ,
    output wire [63                               : 0] m2_axi_awaddr                ,
    output wire [7                                : 0] m2_axi_awlen                 ,
    output wire [2                                : 0] m2_axi_awsize                ,
    output wire [1                                : 0] m2_axi_awburst               ,
    output wire                                             m2_axi_awlock           ,
    output wire [3                                : 0] m2_axi_awcache               ,
    output wire [2                                : 0] m2_axi_awprot                ,
    output wire [3                                : 0] m2_axi_awqos                 ,
    output wire [3                                : 0] m2_axi_awregion              ,
    output wire [3                                : 0] m2_axi_awuser                ,
    output wire                                             m2_axi_awvalid          ,
    input  wire                                             m2_axi_awready          ,
`endif
`ifdef SLV_RX_LANE_3
    output wire [9                                : 0] m3_axi_awid                  ,
    output wire [63                               : 0] m3_axi_awaddr                ,
    output wire [7                                : 0] m3_axi_awlen                 ,
    output wire [2                                : 0] m3_axi_awsize                ,
    output wire [1                                : 0] m3_axi_awburst               ,
    output wire                                             m3_axi_awlock           ,
    output wire [3                                : 0] m3_axi_awcache               ,
    output wire [2                                : 0] m3_axi_awprot                ,
    output wire [3                                : 0] m3_axi_awqos                 ,
    output wire [3                                : 0] m3_axi_awregion              ,
    output wire [3                                : 0] m3_axi_awuser                ,
    output wire                                             m3_axi_awvalid          ,
    input  wire                                             m3_axi_awready          ,
`endif
    // AR 通道（output，m0 恒在；m1~3 随 SLV_TX_LANE_n）
    output wire [9                                : 0] m0_axi_arid                  ,
    output wire [63                               : 0] m0_axi_araddr                ,
    output wire [7                                : 0] m0_axi_arlen                 ,
    output wire [2                                : 0] m0_axi_arsize                ,
    output wire [1                                : 0] m0_axi_arburst               ,
    output wire                                             m0_axi_arlock           ,
    output wire [3                                : 0] m0_axi_arcache               ,
    output wire [2                                : 0] m0_axi_arprot                ,
    output wire [3                                : 0] m0_axi_arqos                 ,
    output wire [3                                : 0] m0_axi_arregion              ,
    output wire [3                                : 0] m0_axi_aruser                ,
    output wire                                             m0_axi_arvalid          ,
    input  wire                                             m0_axi_arready          ,
`ifdef SLV_TX_LANE_1
    output wire [9                                : 0] m1_axi_arid                  ,
    output wire [63                               : 0] m1_axi_araddr                ,
    output wire [7                                : 0] m1_axi_arlen                 ,
    output wire [2                                : 0] m1_axi_arsize                ,
    output wire [1                                : 0] m1_axi_arburst               ,
    output wire                                             m1_axi_arlock           ,
    output wire [3                                : 0] m1_axi_arcache               ,
    output wire [2                                : 0] m1_axi_arprot                ,
    output wire [3                                : 0] m1_axi_arqos                 ,
    output wire [3                                : 0] m1_axi_arregion              ,
    output wire [3                                : 0] m1_axi_aruser                ,
    output wire                                             m1_axi_arvalid          ,
    input  wire                                             m1_axi_arready          ,
`endif
`ifdef SLV_TX_LANE_2
    output wire [9                                : 0] m2_axi_arid                  ,
    output wire [63                               : 0] m2_axi_araddr                ,
    output wire [7                                : 0] m2_axi_arlen                 ,
    output wire [2                                : 0] m2_axi_arsize                ,
    output wire [1                                : 0] m2_axi_arburst               ,
    output wire                                             m2_axi_arlock           ,
    output wire [3                                : 0] m2_axi_arcache               ,
    output wire [2                                : 0] m2_axi_arprot                ,
    output wire [3                                : 0] m2_axi_arqos                 ,
    output wire [3                                : 0] m2_axi_arregion              ,
    output wire [3                                : 0] m2_axi_aruser                ,
    output wire                                             m2_axi_arvalid          ,
    input  wire                                             m2_axi_arready          ,
`endif
`ifdef SLV_TX_LANE_3
    output wire [9                                : 0] m3_axi_arid                  ,
    output wire [63                               : 0] m3_axi_araddr                ,
    output wire [7                                : 0] m3_axi_arlen                 ,
    output wire [2                                : 0] m3_axi_arsize                ,
    output wire [1                                : 0] m3_axi_arburst               ,
    output wire                                             m3_axi_arlock           ,
    output wire [3                                : 0] m3_axi_arcache               ,
    output wire [2                                : 0] m3_axi_arprot                ,
    output wire [3                                : 0] m3_axi_arqos                 ,
    output wire [3                                : 0] m3_axi_arregion              ,
    output wire [3                                : 0] m3_axi_aruser                ,
    output wire                                             m3_axi_arvalid          ,
    input  wire                                             m3_axi_arready          ,
`endif
    // W 通道（output，m0 恒在；m1~3 随 SLV_RX_LANE_n）
    output wire [`AXI_DATA_WIDTH-1                : 0] m0_axi_wdata                 ,
    output wire [`AXI_DATA_WIDTH/8-1              : 0] m0_axi_wstrb                 ,
    output wire [3                                : 0] m0_axi_wuser                 ,
    output wire                                             m0_axi_wlast            ,
    output wire                                             m0_axi_wvalid           ,
    input  wire                                             m0_axi_wready           ,
`ifdef SLV_RX_LANE_1
    output wire [`AXI_DATA_WIDTH-1                : 0] m1_axi_wdata                 ,
    output wire [`AXI_DATA_WIDTH/8-1              : 0] m1_axi_wstrb                 ,
    output wire [3                                : 0] m1_axi_wuser                 ,
    output wire                                             m1_axi_wlast            ,
    output wire                                             m1_axi_wvalid           ,
    input  wire                                             m1_axi_wready           ,
`endif
`ifdef SLV_RX_LANE_2
    output wire [`AXI_DATA_WIDTH-1                : 0] m2_axi_wdata                 ,
    output wire [`AXI_DATA_WIDTH/8-1              : 0] m2_axi_wstrb                 ,
    output wire [3                                : 0] m2_axi_wuser                 ,
    output wire                                             m2_axi_wlast            ,
    output wire                                             m2_axi_wvalid           ,
    input  wire                                             m2_axi_wready           ,
`endif
`ifdef SLV_RX_LANE_3
    output wire [`AXI_DATA_WIDTH-1                : 0] m3_axi_wdata                 ,
    output wire [`AXI_DATA_WIDTH/8-1              : 0] m3_axi_wstrb                 ,
    output wire [3                                : 0] m3_axi_wuser                 ,
    output wire                                             m3_axi_wlast            ,
    output wire                                             m3_axi_wvalid           ,
    input  wire                                             m3_axi_wready           ,
`endif
    // R 通道（input，m0 恒在；m1~3 随 SLV_TX_LANE_n）
    input  wire [9                                : 0] m0_axi_rid                   ,
    input  wire [`AXI_DATA_WIDTH-1                : 0] m0_axi_rdata                 ,
    input  wire [1                                : 0] m0_axi_rresp                 ,
    input  wire [3                                : 0] m0_axi_ruser                 ,
    input  wire                                             m0_axi_rlast            ,
    input  wire                                             m0_axi_rvalid           ,
    output wire                                             m0_axi_rready           ,
`ifdef SLV_TX_LANE_1
    input  wire [9                                : 0] m1_axi_rid                   ,
    input  wire [`AXI_DATA_WIDTH-1                : 0] m1_axi_rdata                 ,
    input  wire [1                                : 0] m1_axi_rresp                 ,
    input  wire [3                                : 0] m1_axi_ruser                 ,
    input  wire                                             m1_axi_rlast            ,
    input  wire                                             m1_axi_rvalid           ,
    output wire                                             m1_axi_rready           ,
`endif
`ifdef SLV_TX_LANE_2
    input  wire [9                                : 0] m2_axi_rid                   ,
    input  wire [`AXI_DATA_WIDTH-1                : 0] m2_axi_rdata                 ,
    input  wire [1                                : 0] m2_axi_rresp                 ,
    input  wire [3                                : 0] m2_axi_ruser                 ,
    input  wire                                             m2_axi_rlast            ,
    input  wire                                             m2_axi_rvalid           ,
    output wire                                             m2_axi_rready           ,
`endif
`ifdef SLV_TX_LANE_3
    input  wire [9                                : 0] m3_axi_rid                   ,
    input  wire [`AXI_DATA_WIDTH-1                : 0] m3_axi_rdata                 ,
    input  wire [1                                : 0] m3_axi_rresp                 ,
    input  wire [3                                : 0] m3_axi_ruser                 ,
    input  wire                                             m3_axi_rlast            ,
    input  wire                                             m3_axi_rvalid           ,
    output wire                                             m3_axi_rready           ,
`endif
    // B 通道（input，m0 恒在；m1~3 随 SLV_RX_LANE_n）
    input  wire [9                                : 0] m0_axi_bid                   ,
    input  wire [1                                : 0] m0_axi_bresp                 ,
    input  wire [3                                : 0] m0_axi_buser                 ,
    input  wire                                             m0_axi_bvalid           ,
    output wire                                             m0_axi_bready           ,
`ifdef SLV_RX_LANE_1
    input  wire [9                                : 0] m1_axi_bid                   ,
    input  wire [1                                : 0] m1_axi_bresp                 ,
    input  wire [3                                : 0] m1_axi_buser                 ,
    input  wire                                             m1_axi_bvalid           ,
    output wire                                             m1_axi_bready           ,
`endif
`ifdef SLV_RX_LANE_2
    input  wire [9                                : 0] m2_axi_bid                   ,
    input  wire [1                                : 0] m2_axi_bresp                 ,
    input  wire [3                                : 0] m2_axi_buser                 ,
    input  wire                                             m2_axi_bvalid           ,
    output wire                                             m2_axi_bready           ,
`endif
`ifdef SLV_RX_LANE_3
    input  wire [9                                : 0] m3_axi_bid                   ,
    input  wire [1                                : 0] m3_axi_bresp                 ,
    input  wire [3                                : 0] m3_axi_buser                 ,
    input  wire                                             m3_axi_bvalid           ,
    output wire                                             m3_axi_bready           ,
`endif
`endif
`else
    // ==========================================================================
    //  AXI 对称模式（AXI_ASYM_MODE 未定义时，1T1R 对称）：全双工单组 s_axi + m_axi
    //  方向与 asymm 一致：s 输入（AXI 用户侧 slave）/ m 输出（AXI 用户侧 master）；
    //  名称沿用 s0_axi_*/m0_axi_*（与 v2 顶层对齐）；随 PRCL_TX_LANE_1/2/3 展开多组
    //  ==========================================================================
    // ---- s (slave，AXI 用户侧输入，方向与 asymm 的 s0 一致) ----
    // AW/AR/W 通道（input）
    input  wire [9                                : 0] s0_axi_awid                   ,
    input  wire [63                               : 0] s0_axi_awaddr                 ,
    input  wire [7                                : 0] s0_axi_awlen                  ,
    input  wire [2                                : 0] s0_axi_awsize                 ,
    input  wire [1                                : 0] s0_axi_awburst                ,
    input  wire                                        s0_axi_awlock                 ,
    input  wire [3                                : 0] s0_axi_awcache                ,
    input  wire [2                                : 0] s0_axi_awprot                 ,
    input  wire [3                                : 0] s0_axi_awqos                  ,
    input  wire [3                                : 0] s0_axi_awregion               ,
    input  wire [3                                : 0] s0_axi_awuser                 ,
    input  wire                                        s0_axi_awvalid                ,
    output wire                                        s0_axi_awready                ,
    input  wire [9                                : 0] s0_axi_arid                   ,
    input  wire [63                               : 0] s0_axi_araddr                 ,
    input  wire [7                                : 0] s0_axi_arlen                  ,
    input  wire [2                                : 0] s0_axi_arsize                 ,
    input  wire [1                                : 0] s0_axi_arburst                ,
    input  wire                                        s0_axi_arlock                 ,
    input  wire [3                                : 0] s0_axi_arcache                ,
    input  wire [2                                : 0] s0_axi_arprot                 ,
    input  wire [3                                : 0] s0_axi_arqos                  ,
    input  wire [3                                : 0] s0_axi_arregion               ,
    input  wire [3                                : 0] s0_axi_aruser                 ,
    input  wire                                        s0_axi_arvalid                ,
    output wire                                        s0_axi_arready                ,
    input  wire [`AXI_DATA_WIDTH-1                : 0] s0_axi_wdata                  ,
    input  wire [`AXI_DATA_WIDTH/8-1              : 0] s0_axi_wstrb                  ,
    input  wire [3                                : 0] s0_axi_wuser                  ,
    input  wire                                        s0_axi_wlast                  ,
    input  wire                                        s0_axi_wvalid                 ,
    output wire                                        s0_axi_wready                 ,
    // R/B 通道（output）
    output wire [9                                : 0] s0_axi_rid                   ,
    output wire [`AXI_DATA_WIDTH-1                : 0] s0_axi_rdata                  ,
    output wire [1                                : 0] s0_axi_rresp                  ,
    output wire [3                                : 0] s0_axi_ruser                  ,
    output wire                                        s0_axi_rlast                  ,
    output wire                                        s0_axi_rvalid                 ,
    input  wire                                        s0_axi_rready                 ,
    output wire [9                                : 0] s0_axi_bid                   ,
    output wire [1                                : 0] s0_axi_bresp                  ,
    output wire [3                                : 0] s0_axi_buser                  ,
    output wire                                        s0_axi_bvalid                 ,
    input  wire                                        s0_axi_bready                 ,
    // ---- m (master，AXI 用户侧输出，方向与 asymm 的 m0 一致) ----
    // AW/AR/W 通道（output）
    output wire [9                                : 0] m0_axi_awid                   ,
    output wire [63                               : 0] m0_axi_awaddr                 ,
    output wire [7                                : 0] m0_axi_awlen                  ,
    output wire [2                                : 0] m0_axi_awsize                 ,
    output wire [1                                : 0] m0_axi_awburst                ,
    output wire                                        m0_axi_awlock                 ,
    output wire [3                                : 0] m0_axi_awcache                ,
    output wire [2                                : 0] m0_axi_awprot                 ,
    output wire [3                                : 0] m0_axi_awqos                  ,
    output wire [3                                : 0] m0_axi_awregion               ,
    output wire [3                                : 0] m0_axi_awuser                 ,
    output wire                                        m0_axi_awvalid                ,
    input  wire                                        m0_axi_awready                ,
    output wire [9                                : 0] m0_axi_arid                   ,
    output wire [63                               : 0] m0_axi_araddr                 ,
    output wire [7                                : 0] m0_axi_arlen                  ,
    output wire [2                                : 0] m0_axi_arsize                 ,
    output wire [1                                : 0] m0_axi_arburst                ,
    output wire                                        m0_axi_arlock                 ,
    output wire [3                                : 0] m0_axi_arcache                ,
    output wire [2                                : 0] m0_axi_arprot                 ,
    output wire [3                                : 0] m0_axi_arqos                  ,
    output wire [3                                : 0] m0_axi_arregion               ,
    output wire [3                                : 0] m0_axi_aruser                 ,
    output wire                                        m0_axi_arvalid                ,
    input  wire                                        m0_axi_arready                ,
    output wire [`AXI_DATA_WIDTH-1                : 0] m0_axi_wdata                  ,
    output wire [`AXI_DATA_WIDTH/8-1              : 0] m0_axi_wstrb                  ,
    output wire [3                                : 0] m0_axi_wuser                  ,
    output wire                                        m0_axi_wlast                  ,
    output wire                                        m0_axi_wvalid                 ,
    input  wire                                        m0_axi_wready                 ,
    // R/B 通道（input）
    input  wire [9                                : 0] m0_axi_rid                   ,
    input  wire [`AXI_DATA_WIDTH-1                : 0] m0_axi_rdata                  ,
    input  wire [1                                : 0] m0_axi_rresp                  ,
    input  wire [3                                : 0] m0_axi_ruser                  ,
    input  wire                                        m0_axi_rlast                  ,
    input  wire                                        m0_axi_rvalid                 ,
    output wire                                        m0_axi_rready                 ,
    input  wire [9                                : 0] m0_axi_bid                   ,
    input  wire [1                                : 0] m0_axi_bresp                  ,
    input  wire [3                                : 0] m0_axi_buser                  ,
    input  wire                                        m0_axi_bvalid                 ,
    output wire                                        m0_axi_bready                 ,
`endif
`endif
`else
    // ==========================================================================
    //  CXS interface (对外, 按 top TX/RX lane 宏分支)
    //  TX lane → cxs_prcl{n}_rx_* (cxs top 的 CXS RX 方向, 上游→协议层)
    // ==========================================================================
    input  wire [`CXSPRCL_DATAFLITWIDTH-1         : 0] cxs_prcl0_rx_data             ,
    input  wire [`CXSPRCL_CNTL_WIDTH-1            : 0] cxs_prcl0_rx_cntl             ,
    input  wire                                        cxs_prcl0_rx_valid            ,
    input  wire [1                                : 0] cxs_prcl0_rx_prcltype         ,
    input  wire [`CXSPRCL_SRCID_WIDTH-1           : 0] cxs_prcl0_rx_srcid            ,
    input  wire [`CXSPRCL_TGTID_WIDTH-1           : 0] cxs_prcl0_rx_tgtid            ,
`ifdef CXSPRCL_LAST
    input  wire                                        cxs_prcl0_rx_last             ,
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
    input  wire [((`CXSPRCL_DATAFLITWIDTH+7)/8)-1 : 0] cxs_prcl0_rx_chk_data         ,
    input  wire [((`CXSPRCL_CNTL_WIDTH+7)/8)-1    : 0] cxs_prcl0_rx_chk_cntl         ,
    input  wire                                        cxs_prcl0_rx_chk_prcltype     ,
    input  wire                                        cxs_prcl0_rx_chk_srcid        ,
    input  wire                                        cxs_prcl0_rx_chk_tgtid        ,
    input  wire                                        cxs_prcl0_rx_chk_last         ,
    input  wire                                        cxs_prcl0_rx_chk_valid        ,
`endif
    input  wire                                        cxs_prcl0_rx_crdrtn           ,
    output wire                                        cxs_prcl0_rx_crdgnt           ,
    input  wire                                        cxs_prcl0_rx_activereq        ,
    output wire                                        cxs_prcl0_rx_activeack        ,
    output wire                                        cxs_prcl0_rx_deacthint        ,
`ifdef PRCL_TX_LANE_1
    input  wire [`CXSPRCL_DATAFLITWIDTH-1         : 0] cxs_prcl1_rx_data             ,
    input  wire [`CXSPRCL_CNTL_WIDTH-1            : 0] cxs_prcl1_rx_cntl             ,
    input  wire                                        cxs_prcl1_rx_valid            ,
    input  wire [1                                : 0] cxs_prcl1_rx_prcltype         ,
    input  wire [`CXSPRCL_SRCID_WIDTH-1           : 0] cxs_prcl1_rx_srcid            ,
    input  wire [`CXSPRCL_TGTID_WIDTH-1           : 0] cxs_prcl1_rx_tgtid            ,
`ifdef CXSPRCL_LAST
    input  wire                                        cxs_prcl1_rx_last             ,
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
    input  wire [((`CXSPRCL_DATAFLITWIDTH+7)/8)-1 : 0] cxs_prcl1_rx_chk_data         ,
    input  wire [((`CXSPRCL_CNTL_WIDTH+7)/8)-1    : 0] cxs_prcl1_rx_chk_cntl         ,
    input  wire                                        cxs_prcl1_rx_chk_prcltype     ,
    input  wire                                        cxs_prcl1_rx_chk_srcid        ,
    input  wire                                        cxs_prcl1_rx_chk_tgtid        ,
    input  wire                                        cxs_prcl1_rx_chk_last         ,
    input  wire                                        cxs_prcl1_rx_chk_valid        ,
`endif
    input  wire                                        cxs_prcl1_rx_crdrtn           ,
    output wire                                        cxs_prcl1_rx_crdgnt           ,
    input  wire                                        cxs_prcl1_rx_activereq        ,
    output wire                                        cxs_prcl1_rx_activeack        ,
    output wire                                        cxs_prcl1_rx_deacthint        ,
`endif
`ifdef PRCL_TX_LANE_2
    input  wire [`CXSPRCL_DATAFLITWIDTH-1         : 0] cxs_prcl2_rx_data             ,
    input  wire [`CXSPRCL_CNTL_WIDTH-1            : 0] cxs_prcl2_rx_cntl             ,
    input  wire                                        cxs_prcl2_rx_valid            ,
    input  wire [1                                : 0] cxs_prcl2_rx_prcltype         ,
    input  wire [`CXSPRCL_SRCID_WIDTH-1           : 0] cxs_prcl2_rx_srcid            ,
    input  wire [`CXSPRCL_TGTID_WIDTH-1           : 0] cxs_prcl2_rx_tgtid            ,
`ifdef CXSPRCL_LAST
    input  wire                                        cxs_prcl2_rx_last             ,
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
    input  wire [((`CXSPRCL_DATAFLITWIDTH+7)/8)-1 : 0] cxs_prcl2_rx_chk_data         ,
    input  wire [((`CXSPRCL_CNTL_WIDTH+7)/8)-1    : 0] cxs_prcl2_rx_chk_cntl         ,
    input  wire                                        cxs_prcl2_rx_chk_prcltype     ,
    input  wire                                        cxs_prcl2_rx_chk_srcid        ,
    input  wire                                        cxs_prcl2_rx_chk_tgtid        ,
    input  wire                                        cxs_prcl2_rx_chk_last         ,
    input  wire                                        cxs_prcl2_rx_chk_valid        ,
`endif
    input  wire                                        cxs_prcl2_rx_crdrtn           ,
    output wire                                        cxs_prcl2_rx_crdgnt           ,
    input  wire                                        cxs_prcl2_rx_activereq        ,
    output wire                                        cxs_prcl2_rx_activeack        ,
    output wire                                        cxs_prcl2_rx_deacthint        ,
`endif
`ifdef PRCL_TX_LANE_3
    input  wire [`CXSPRCL_DATAFLITWIDTH-1         : 0] cxs_prcl3_rx_data             ,
    input  wire [`CXSPRCL_CNTL_WIDTH-1            : 0] cxs_prcl3_rx_cntl             ,
    input  wire                                        cxs_prcl3_rx_valid            ,
    input  wire [1                                : 0] cxs_prcl3_rx_prcltype         ,
    input  wire [`CXSPRCL_SRCID_WIDTH-1           : 0] cxs_prcl3_rx_srcid            ,
    input  wire [`CXSPRCL_TGTID_WIDTH-1           : 0] cxs_prcl3_rx_tgtid            ,
`ifdef CXSPRCL_LAST
    input  wire                                        cxs_prcl3_rx_last             ,
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
    input  wire [((`CXSPRCL_DATAFLITWIDTH+7)/8)-1 : 0] cxs_prcl3_rx_chk_data         ,
    input  wire [((`CXSPRCL_CNTL_WIDTH+7)/8)-1    : 0] cxs_prcl3_rx_chk_cntl         ,
    input  wire                                        cxs_prcl3_rx_chk_prcltype     ,
    input  wire                                        cxs_prcl3_rx_chk_srcid        ,
    input  wire                                        cxs_prcl3_rx_chk_tgtid        ,
    input  wire                                        cxs_prcl3_rx_chk_last         ,
    input  wire                                        cxs_prcl3_rx_chk_valid        ,
`endif
    input  wire                                        cxs_prcl3_rx_crdrtn           ,
    output wire                                        cxs_prcl3_rx_crdgnt           ,
    input  wire                                        cxs_prcl3_rx_activereq        ,
    output wire                                        cxs_prcl3_rx_activeack        ,
    output wire                                        cxs_prcl3_rx_deacthint        ,
`endif

    // ==========================================================================
    //  CXS TX 方向 (对外, RX lane → cxs_prcl{n}_tx_*, 协议层→上游)
    // ==========================================================================
    output wire [`CXSPRCL_DATAFLITWIDTH-1         : 0] cxs_prcl0_tx_data             ,
    output wire [`CXSPRCL_CNTL_WIDTH-1            : 0] cxs_prcl0_tx_cntl             ,
    output wire                                        cxs_prcl0_tx_valid            ,
    output wire [1                                : 0] cxs_prcl0_tx_prcltype         ,
    output wire [`CXSPRCL_SRCID_WIDTH-1           : 0] cxs_prcl0_tx_srcid            ,
    output wire [`CXSPRCL_TGTID_WIDTH-1           : 0] cxs_prcl0_tx_tgtid            ,
`ifdef CXSPRCL_LAST
    output wire                                        cxs_prcl0_tx_last             ,
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
    output wire [((`CXSPRCL_DATAFLITWIDTH+7)/8)-1 : 0] cxs_prcl0_tx_chk_data         ,
    output wire [((`CXSPRCL_CNTL_WIDTH+7)/8)-1    : 0] cxs_prcl0_tx_chk_cntl         ,
    output wire                                        cxs_prcl0_tx_chk_prcltype     ,
    output wire                                        cxs_prcl0_tx_chk_srcid        ,
    output wire                                        cxs_prcl0_tx_chk_tgtid        ,
    output wire                                        cxs_prcl0_tx_chk_last         ,
    output wire                                        cxs_prcl0_tx_chk_valid        ,
`endif
    output wire                                        cxs_prcl0_tx_crdrtn           ,
    input  wire                                        cxs_prcl0_tx_crdgnt           ,
    output wire                                        cxs_prcl0_tx_activereq        ,
    input  wire                                        cxs_prcl0_tx_activeack        ,
    input  wire                                        cxs_prcl0_tx_deacthint        ,
`ifdef PRCL_RX_LANE_1
    output wire [`CXSPRCL_DATAFLITWIDTH-1         : 0] cxs_prcl1_tx_data             ,
    output wire [`CXSPRCL_CNTL_WIDTH-1            : 0] cxs_prcl1_tx_cntl             ,
    output wire                                        cxs_prcl1_tx_valid            ,
    output wire [1                                : 0] cxs_prcl1_tx_prcltype         ,
    output wire [`CXSPRCL_SRCID_WIDTH-1           : 0] cxs_prcl1_tx_srcid            ,
    output wire [`CXSPRCL_TGTID_WIDTH-1           : 0] cxs_prcl1_tx_tgtid            ,
`ifdef CXSPRCL_LAST
    output wire                                        cxs_prcl1_tx_last             ,
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
    output wire [((`CXSPRCL_DATAFLITWIDTH+7)/8)-1 : 0] cxs_prcl1_tx_chk_data         ,
    output wire [((`CXSPRCL_CNTL_WIDTH+7)/8)-1    : 0] cxs_prcl1_tx_chk_cntl         ,
    output wire                                        cxs_prcl1_tx_chk_prcltype     ,
    output wire                                        cxs_prcl1_tx_chk_srcid        ,
    output wire                                        cxs_prcl1_tx_chk_tgtid        ,
    output wire                                        cxs_prcl1_tx_chk_last         ,
    output wire                                        cxs_prcl1_tx_chk_valid        ,
`endif
    output wire                                        cxs_prcl1_tx_crdrtn           ,
    input  wire                                        cxs_prcl1_tx_crdgnt           ,
    output wire                                        cxs_prcl1_tx_activereq        ,
    input  wire                                        cxs_prcl1_tx_activeack        ,
    input  wire                                        cxs_prcl1_tx_deacthint        ,
`endif
`ifdef PRCL_RX_LANE_2
    output wire [`CXSPRCL_DATAFLITWIDTH-1         : 0] cxs_prcl2_tx_data             ,
    output wire [`CXSPRCL_CNTL_WIDTH-1            : 0] cxs_prcl2_tx_cntl             ,
    output wire                                        cxs_prcl2_tx_valid            ,
    output wire [1                                : 0] cxs_prcl2_tx_prcltype         ,
    output wire [`CXSPRCL_SRCID_WIDTH-1           : 0] cxs_prcl2_tx_srcid            ,
    output wire [`CXSPRCL_TGTID_WIDTH-1           : 0] cxs_prcl2_tx_tgtid            ,
`ifdef CXSPRCL_LAST
    output wire                                        cxs_prcl2_tx_last             ,
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
    output wire [((`CXSPRCL_DATAFLITWIDTH+7)/8)-1 : 0] cxs_prcl2_tx_chk_data         ,
    output wire [((`CXSPRCL_CNTL_WIDTH+7)/8)-1    : 0] cxs_prcl2_tx_chk_cntl         ,
    output wire                                        cxs_prcl2_tx_chk_prcltype     ,
    output wire                                        cxs_prcl2_tx_chk_srcid        ,
    output wire                                        cxs_prcl2_tx_chk_tgtid        ,
    output wire                                        cxs_prcl2_tx_chk_last         ,
    output wire                                        cxs_prcl2_tx_chk_valid        ,
`endif
    output wire                                        cxs_prcl2_tx_crdrtn           ,
    input  wire                                        cxs_prcl2_tx_crdgnt           ,
    output wire                                        cxs_prcl2_tx_activereq        ,
    input  wire                                        cxs_prcl2_tx_activeack        ,
    input  wire                                        cxs_prcl2_tx_deacthint        ,
`endif
`ifdef PRCL_RX_LANE_3
    output wire [`CXSPRCL_DATAFLITWIDTH-1         : 0] cxs_prcl3_tx_data             ,
    output wire [`CXSPRCL_CNTL_WIDTH-1            : 0] cxs_prcl3_tx_cntl             ,
    output wire                                        cxs_prcl3_tx_valid            ,
    output wire [1                                : 0] cxs_prcl3_tx_prcltype         ,
    output wire [`CXSPRCL_SRCID_WIDTH-1           : 0] cxs_prcl3_tx_srcid            ,
    output wire [`CXSPRCL_TGTID_WIDTH-1           : 0] cxs_prcl3_tx_tgtid            ,
`ifdef CXSPRCL_LAST
    output wire                                        cxs_prcl3_tx_last             ,
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
    output wire [((`CXSPRCL_DATAFLITWIDTH+7)/8)-1 : 0] cxs_prcl3_tx_chk_data         ,
    output wire [((`CXSPRCL_CNTL_WIDTH+7)/8)-1    : 0] cxs_prcl3_tx_chk_cntl         ,
    output wire                                        cxs_prcl3_tx_chk_prcltype     ,
    output wire                                        cxs_prcl3_tx_chk_srcid        ,
    output wire                                        cxs_prcl3_tx_chk_tgtid        ,
    output wire                                        cxs_prcl3_tx_chk_last         ,
    output wire                                        cxs_prcl3_tx_chk_valid        ,
`endif
    output wire                                        cxs_prcl3_tx_crdrtn           ,
    input  wire                                        cxs_prcl3_tx_crdgnt           ,
    output wire                                        cxs_prcl3_tx_activereq        ,
    input  wire                                        cxs_prcl3_tx_activeack        ,
    input  wire                                        cxs_prcl3_tx_deacthint        ,
`endif
`endif

    // --- UDI TX / RX data interface (link_layer §2.4, per-lane CPIF_DATA_WIDTH, 按 link × lane 分支) ---
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link0_lane0_data_i     ,
    input  wire                                        udi_tx_link0_lane0_data_vld_i ,
`ifdef LL_LINK_LANE1
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link0_lane1_data_i     ,
    input  wire                                        udi_tx_link0_lane1_data_vld_i ,
`endif
`ifdef LL_LINK_LANE3
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link0_lane2_data_i     ,
    input  wire                                        udi_tx_link0_lane2_data_vld_i ,
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link0_lane3_data_i     ,
    input  wire                                        udi_tx_link0_lane3_data_vld_i ,
`endif
    output wire                                        udi_tx_link0_ready_o          ,
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link0_lane0_data_o     ,
    output wire                                        udi_rx_link0_lane0_data_vld_o ,
`ifdef LL_LINK_LANE1
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link0_lane1_data_o     ,
    output wire                                        udi_rx_link0_lane1_data_vld_o ,
`endif
`ifdef LL_LINK_LANE3
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link0_lane2_data_o     ,
    output wire                                        udi_rx_link0_lane2_data_vld_o ,
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link0_lane3_data_o     ,
    output wire                                        udi_rx_link0_lane3_data_vld_o ,
`endif
`ifdef PRCL_TX_LANE_1
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link1_lane0_data_i     ,
    input  wire                                        udi_tx_link1_lane0_data_vld_i ,
`ifdef LL_LINK_LANE1
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link1_lane1_data_i     ,
    input  wire                                        udi_tx_link1_lane1_data_vld_i ,
`endif
`ifdef LL_LINK_LANE3
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link1_lane2_data_i     ,
    input  wire                                        udi_tx_link1_lane2_data_vld_i ,
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link1_lane3_data_i     ,
    input  wire                                        udi_tx_link1_lane3_data_vld_i ,
`endif
    output wire                                        udi_tx_link1_ready_o          ,
`endif
`ifdef PRCL_TX_LANE_2
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link2_lane0_data_i     ,
    input  wire                                        udi_tx_link2_lane0_data_vld_i ,
`ifdef LL_LINK_LANE1
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link2_lane1_data_i     ,
    input  wire                                        udi_tx_link2_lane1_data_vld_i ,
`endif
`ifdef LL_LINK_LANE3
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link2_lane2_data_i     ,
    input  wire                                        udi_tx_link2_lane2_data_vld_i ,
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link2_lane3_data_i     ,
    input  wire                                        udi_tx_link2_lane3_data_vld_i ,
`endif
    output wire                                        udi_tx_link2_ready_o          ,
`endif
`ifdef PRCL_TX_LANE_3
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link3_lane0_data_i     ,
    input  wire                                        udi_tx_link3_lane0_data_vld_i ,
`ifdef LL_LINK_LANE1
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link3_lane1_data_i     ,
    input  wire                                        udi_tx_link3_lane1_data_vld_i ,
`endif
`ifdef LL_LINK_LANE3
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link3_lane2_data_i     ,
    input  wire                                        udi_tx_link3_lane2_data_vld_i ,
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link3_lane3_data_i     ,
    input  wire                                        udi_tx_link3_lane3_data_vld_i ,
`endif
    output wire                                        udi_tx_link3_ready_o          ,
`endif
`ifdef PRCL_RX_LANE_1
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link1_lane0_data_o     ,
    output wire                                        udi_rx_link1_lane0_data_vld_o ,
`ifdef LL_LINK_LANE1
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link1_lane1_data_o     ,
    output wire                                        udi_rx_link1_lane1_data_vld_o ,
`endif
`ifdef LL_LINK_LANE3
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link1_lane2_data_o     ,
    output wire                                        udi_rx_link1_lane2_data_vld_o ,
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link1_lane3_data_o     ,
    output wire                                        udi_rx_link1_lane3_data_vld_o ,
`endif
`endif
`ifdef PRCL_RX_LANE_2
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link2_lane0_data_o     ,
    output wire                                        udi_rx_link2_lane0_data_vld_o ,
`ifdef LL_LINK_LANE1
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link2_lane1_data_o     ,
    output wire                                        udi_rx_link2_lane1_data_vld_o ,
`endif
`ifdef LL_LINK_LANE3
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link2_lane2_data_o     ,
    output wire                                        udi_rx_link2_lane2_data_vld_o ,
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link2_lane3_data_o     ,
    output wire                                        udi_rx_link2_lane3_data_vld_o ,
`endif
`endif
`ifdef PRCL_RX_LANE_3
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link3_lane0_data_o     ,
    output wire                                        udi_rx_link3_lane0_data_vld_o ,
`ifdef LL_LINK_LANE1
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link3_lane1_data_o     ,
    output wire                                        udi_rx_link3_lane1_data_vld_o ,
`endif
`ifdef LL_LINK_LANE3
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link3_lane2_data_o     ,
    output wire                                        udi_rx_link3_lane2_data_vld_o ,
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link3_lane3_data_o     ,
    output wire                                        udi_rx_link3_lane3_data_vld_o ,
`endif
`endif

    // -------------------------------------------------------------------------
    // 主带数据 / 训练校准（→ analog PHY，logicphy_analogphy_intf v1.1 原文
    // 逐 lane 展开，经 logic_phy 透传；注释中头部假设编号指向 logic_phy.v）
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
    // --- lane0 训练校准参数（logic → analog；OFFSET_CTRL_DATA 为 inout 原文，
    //     当前仅驱动写路径，读回待 MAS；RX16_L_CDR 反馈未消费，见头部假设 7）---
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
    output wire [5:0]           lane0_RX16_DCDL_CTRL,   // RX16 DCDL 训练参数（冗余 IO；intf v10.3: 改名 + 4→6 bit）
`ifdef D2D_PHY_TX_LANE1_EN
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
`ifdef D2D_PHY_RX_LANE1_EN
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
    output wire [5:0]           lane1_RX16_DCDL_CTRL,   // RX16 DCDL 训练参数（冗余 IO；intf v10.3: 改名 + 4→6 bit）
`endif
`ifdef D2D_PHY_TX_LANE2_EN
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
`ifdef D2D_PHY_RX_LANE2_EN
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
    output wire [5:0]           lane2_RX16_DCDL_CTRL,   // RX16 DCDL 训练参数（冗余 IO；intf v10.3: 改名 + 4→6 bit）
`endif
`ifdef D2D_PHY_TX_LANE3_EN
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
`ifdef D2D_PHY_RX_LANE3_EN
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
    output wire [5:0]           lane3_RX16_DCDL_CTRL,   // RX16 DCDL 训练参数（冗余 IO；intf v10.3: 改名 + 4→6 bit）
`endif

    // SBC 边带通道（经 analog PHY 中转至对端）+ PLL 状态
    output wire                 sb_txd,                         // 边带发送串行数据（同步 sb_tx_clk）
    input  wire                 sb_rxd,                         // 边带接收串行数据（同步 sb_rx_clk）
    input  wire                 pll_lock,                       // 模拟 PHY PLL 锁定（暂悬空，见头部假设 8）

    // -------------------------------------------------------------------------
    // DFx status（ctrl link_layer）
    // -------------------------------------------------------------------------
    output wire [31:0]          dfx_stat_o,

    // -------------------------------------------------------------------------
    // 中断（两路独立聚合输出，决策③）
    // -------------------------------------------------------------------------
    output wire                 irq_ext,                        // logic_phy 聚合中断，高有效
    output wire sb_txr,                                      // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire sb_rxr,                                      // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_TRX_REG1_WRITE_IN,                     // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_TRX_REG2_WRITE_IN,                     // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_TRX_TEST_REG1_WRITE_IN,                // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_TRX_TEST_REG2_WRITE_IN,                // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_RX_CLK_ENG_REG1_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_RX_CLK_ENG_REG2_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_RX_CLK_ENG_REG3_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_RX_CLK_ENG_REG4_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_RX_CLK_REG1_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_RX_CLK_REG2_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_RX_CLK_REG3_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_RX_CLK_REG4_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_RX_DATA_REG1_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_RX_DATA_REG2_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_RX_DATA_REG3_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_RX_LDO_REG1_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_RX_LDO_REG2_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_TX_CLK_ENG_REG1_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_TX_CLK_ENG_REG2_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_TX_CLK_ENG_REG3_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_TX_CLK_ENG_REG4_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_TX_CLK_REG1_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_TX_CLK_REG2_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_TX_CLK_REG3_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_TX_CLK_REG4_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_TX_CLK_REG5_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_TX_DATA_REG1_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_TX_DATA_REG2_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_TX_DATA_REG3_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_TX_DATA_REG4_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_TX_DATA_REG5_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_TX_DATA_REG6_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_TX_DATA_REG7_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_TX_LDO_REG1_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane0_TX_LDO_REG2_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_TRX_REG1_WRITE_IN,                     // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_TRX_REG2_WRITE_IN,                     // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_TRX_TEST_REG1_WRITE_IN,                // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_TRX_TEST_REG2_WRITE_IN,                // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
`ifdef D2D_PHY_RX_LANE1_EN
    output wire [31:0] lane1_RX_CLK_ENG_REG1_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_RX_CLK_ENG_REG2_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_RX_CLK_ENG_REG3_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_RX_CLK_ENG_REG4_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_RX_CLK_REG1_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_RX_CLK_REG2_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_RX_CLK_REG3_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_RX_CLK_REG4_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_RX_DATA_REG1_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_RX_DATA_REG2_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_RX_DATA_REG3_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_RX_LDO_REG1_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_RX_LDO_REG2_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
`endif
`ifdef D2D_PHY_TX_LANE1_EN
    output wire [31:0] lane1_TX_CLK_ENG_REG1_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_TX_CLK_ENG_REG2_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_TX_CLK_ENG_REG3_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_TX_CLK_ENG_REG4_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_TX_CLK_REG1_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_TX_CLK_REG2_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_TX_CLK_REG3_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_TX_CLK_REG4_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_TX_CLK_REG5_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_TX_DATA_REG1_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_TX_DATA_REG2_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_TX_DATA_REG3_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_TX_DATA_REG4_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_TX_DATA_REG5_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_TX_DATA_REG6_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_TX_DATA_REG7_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_TX_LDO_REG1_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane1_TX_LDO_REG2_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
`endif
    output wire [31:0] lane2_TRX_REG1_WRITE_IN,                     // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_TRX_REG2_WRITE_IN,                     // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_TRX_TEST_REG1_WRITE_IN,                // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_TRX_TEST_REG2_WRITE_IN,                // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
`ifdef D2D_PHY_RX_LANE2_EN
    output wire [31:0] lane2_RX_CLK_ENG_REG1_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_RX_CLK_ENG_REG2_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_RX_CLK_ENG_REG3_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_RX_CLK_ENG_REG4_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_RX_CLK_REG1_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_RX_CLK_REG2_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_RX_CLK_REG3_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_RX_CLK_REG4_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_RX_DATA_REG1_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_RX_DATA_REG2_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_RX_DATA_REG3_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_RX_LDO_REG1_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_RX_LDO_REG2_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
`endif
`ifdef D2D_PHY_TX_LANE2_EN
    output wire [31:0] lane2_TX_CLK_ENG_REG1_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_TX_CLK_ENG_REG2_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_TX_CLK_ENG_REG3_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_TX_CLK_ENG_REG4_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_TX_CLK_REG1_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_TX_CLK_REG2_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_TX_CLK_REG3_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_TX_CLK_REG4_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_TX_CLK_REG5_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_TX_DATA_REG1_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_TX_DATA_REG2_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_TX_DATA_REG3_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_TX_DATA_REG4_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_TX_DATA_REG5_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_TX_DATA_REG6_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_TX_DATA_REG7_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_TX_LDO_REG1_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane2_TX_LDO_REG2_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
`endif
    output wire [31:0] lane3_TRX_REG1_WRITE_IN,                     // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_TRX_REG2_WRITE_IN,                     // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_TRX_TEST_REG1_WRITE_IN,                // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_TRX_TEST_REG2_WRITE_IN,                // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
`ifdef D2D_PHY_RX_LANE3_EN
    output wire [31:0] lane3_RX_CLK_ENG_REG1_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_RX_CLK_ENG_REG2_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_RX_CLK_ENG_REG3_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_RX_CLK_ENG_REG4_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_RX_CLK_REG1_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_RX_CLK_REG2_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_RX_CLK_REG3_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_RX_CLK_REG4_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_RX_DATA_REG1_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_RX_DATA_REG2_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_RX_DATA_REG3_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_RX_LDO_REG1_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_RX_LDO_REG2_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
`endif
`ifdef D2D_PHY_TX_LANE3_EN
    output wire [31:0] lane3_TX_CLK_ENG_REG1_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_TX_CLK_ENG_REG2_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_TX_CLK_ENG_REG3_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_TX_CLK_ENG_REG4_WRITE_IN,              // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_TX_CLK_REG1_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_TX_CLK_REG2_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_TX_CLK_REG3_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_TX_CLK_REG4_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_TX_CLK_REG5_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_TX_DATA_REG1_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_TX_DATA_REG2_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_TX_DATA_REG3_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_TX_DATA_REG4_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_TX_DATA_REG5_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_TX_DATA_REG6_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_TX_DATA_REG7_WRITE_IN,                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_TX_LDO_REG1_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire [31:0] lane3_TX_LDO_REG2_WRITE_IN,                  // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
`endif
    output wire lane0_TX_EN_ASYNC,                           // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane0_TX_EN_SYNC,                            // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane0_TX_EN_TRACK,                           // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane0_TX_RSTB_SYNC,                          // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane0_RX_CLK_EN_ASYNC,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane0_RX_CLK_EN_SYNC,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane0_RX_CLK_EN_TRACK,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane0_RX_RSTB_SYNC,                          // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane1_TX_EN_ASYNC,                           // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane1_TX_EN_SYNC,                            // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane1_TX_EN_TRACK,                           // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane1_TX_RSTB_SYNC,                          // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane1_RX_CLK_EN_ASYNC,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane1_RX_CLK_EN_SYNC,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane1_RX_CLK_EN_TRACK,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane1_RX_RSTB_SYNC,                          // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane2_TX_EN_ASYNC,                           // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane2_TX_EN_SYNC,                            // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane2_TX_EN_TRACK,                           // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane2_TX_RSTB_SYNC,                          // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane2_RX_CLK_EN_ASYNC,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane2_RX_CLK_EN_SYNC,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane2_RX_CLK_EN_TRACK,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane2_RX_RSTB_SYNC,                          // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane3_TX_EN_ASYNC,                           // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane3_TX_EN_SYNC,                            // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane3_TX_EN_TRACK,                           // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane3_TX_RSTB_SYNC,                          // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane3_RX_CLK_EN_ASYNC,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane3_RX_CLK_EN_SYNC,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane3_RX_CLK_EN_TRACK,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane3_RX_RSTB_SYNC,                          // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [31:0] lane0_RX_CLK_ENG_REG_READ_OUT,               // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [31:0] lane0_TX_CLK_ENG_REG_READ_OUT,               // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire lane0_CONNECT_DIG_OUT,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [31:0] lane1_RX_CLK_ENG_REG_READ_OUT,               // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [31:0] lane1_TX_CLK_ENG_REG_READ_OUT,               // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire lane1_CONNECT_DIG_OUT,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [31:0] lane2_RX_CLK_ENG_REG_READ_OUT,               // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [31:0] lane2_TX_CLK_ENG_REG_READ_OUT,               // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire lane2_CONNECT_DIG_OUT,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [31:0] lane3_RX_CLK_ENG_REG_READ_OUT,               // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [31:0] lane3_TX_CLK_ENG_REG_READ_OUT,               // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire lane3_CONNECT_DIG_OUT,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane0_DOUT0_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane0_DOUT1_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane0_DOUT2_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane0_DOUT3_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane0_DOUT4_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane0_DOUT5_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane0_DOUT6_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane0_DOUT7_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane0_DOUT8_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane0_DOUT9_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane0_DOUT10_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane0_DOUT11_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane0_DOUT12_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane0_DOUT13_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane0_DOUT14_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane0_DOUT15_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane0_DOUTRFU_ERR_EDGE,                      // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane1_DOUT0_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane1_DOUT1_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane1_DOUT2_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane1_DOUT3_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane1_DOUT4_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane1_DOUT5_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane1_DOUT6_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane1_DOUT7_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane1_DOUT8_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane1_DOUT9_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane1_DOUT10_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane1_DOUT11_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane1_DOUT12_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane1_DOUT13_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane1_DOUT14_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane1_DOUT15_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane1_DOUTRFU_ERR_EDGE,                      // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane2_DOUT0_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane2_DOUT1_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane2_DOUT2_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane2_DOUT3_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane2_DOUT4_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane2_DOUT5_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane2_DOUT6_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane2_DOUT7_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane2_DOUT8_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane2_DOUT9_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane2_DOUT10_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane2_DOUT11_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane2_DOUT12_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane2_DOUT13_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane2_DOUT14_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane2_DOUT15_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane2_DOUTRFU_ERR_EDGE,                      // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane3_DOUT0_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane3_DOUT1_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane3_DOUT2_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane3_DOUT3_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane3_DOUT4_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane3_DOUT5_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane3_DOUT6_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane3_DOUT7_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane3_DOUT8_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane3_DOUT9_ERR_EDGE,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane3_DOUT10_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane3_DOUT11_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane3_DOUT12_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane3_DOUT13_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane3_DOUT14_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane3_DOUT15_ERR_EDGE,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire [3:0] lane3_DOUTRFU_ERR_EDGE,                      // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane0_TX_LPC_DATA_IN,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    input wire lane0_RX_LPC_DATA_OUT,                       // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire lane1_TX_LPC_DATA_IN,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
`ifdef D2D_PHY_RX_LANE1_EN
    input wire lane1_RX_LPC_DATA_OUT,
`endif
    output wire lane2_TX_LPC_DATA_IN,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
`ifdef D2D_PHY_RX_LANE2_EN
    input wire lane2_RX_LPC_DATA_OUT,
`endif
    output wire lane3_TX_LPC_DATA_IN,                        // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
`ifdef D2D_PHY_RX_LANE3_EN
    input wire lane3_RX_LPC_DATA_OUT,
`endif
    output wire cpif_rx_lpc,                                 // WP2: intf v10.3 P0-B（1:1 对齐 logic_phy）
    output wire                 hipi_ctrl_intr                  // ctrl 聚合中断（cxs_prcl | link_irq）
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    localparam CPIF_LL_WIDTH = `CPIF_DATA_WIDTH;  // link_layer 侧 CPIF 数据位宽（top defines 宏）
    localparam CPIF_LP_WIDTH = 1024;              // logic_phy 侧 CPIF 数据位宽（32:1 模式仅低 512 有效）

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    // --- CPIF 数据通路（ctrl ↔ logic_phy；位宽映射见"组合逻辑"段）---
    //     命名约定：ll_ 前缀 = link_layer 侧（`CPIF_DATA_WIDTH）；lp_ 前缀 = logic_phy 侧（1024 bit）
    wire [CPIF_LL_WIDTH-1:0]    ll_cpif_tx_lane0_data;      // ctrl 输出（link 侧 512）
    wire                        ll_cpif_tx_lane0_data_vld;
    wire [CPIF_LP_WIDTH-1:0]    lp_cpif_tx_lane0_data;      // 零扩展后 → logic_phy 输入
`ifdef D2D_PHY_TX_LANE1_EN
    wire [CPIF_LL_WIDTH-1:0]    ll_cpif_tx_lane1_data;      // link 源（LL_LINK_LANE1 未定义 → 恒 0）
    wire                        ll_cpif_tx_lane1_data_vld;
    wire [CPIF_LP_WIDTH-1:0]    lp_cpif_tx_lane1_data;      // → logic_phy 输入
`endif
`ifdef D2D_PHY_TX_LANE2_EN
    wire [CPIF_LL_WIDTH-1:0]    ll_cpif_tx_lane2_data;      // link 源（LL_LINK_LANE3 未定义 → 恒 0）
    wire                        ll_cpif_tx_lane2_data_vld;
    wire [CPIF_LP_WIDTH-1:0]    lp_cpif_tx_lane2_data;      // → logic_phy 输入
`endif
`ifdef D2D_PHY_TX_LANE3_EN
    wire [CPIF_LL_WIDTH-1:0]    ll_cpif_tx_lane3_data;      // link 源（LL_LINK_LANE3 未定义 → 恒 0）
    wire                        ll_cpif_tx_lane3_data_vld;
    wire [CPIF_LP_WIDTH-1:0]    lp_cpif_tx_lane3_data;      // → logic_phy 输入
`endif
    wire [CPIF_LP_WIDTH-1:0]    lp_cpif_rx_lane0_data;      // logic_phy 全宽输出
    wire                        lp_cpif_rx_lane0_data_vld;
    wire [CPIF_LL_WIDTH-1:0]    ll_cpif_rx_lane0_data;      // 截断低 512 → ctrl 输入
`ifdef D2D_PHY_RX_LANE1_EN
    wire [CPIF_LP_WIDTH-1:0]    lp_cpif_rx_lane1_data;      // logic_phy 输出
    wire                        lp_cpif_rx_lane1_data_vld;
    wire [CPIF_LL_WIDTH-1:0]    ll_cpif_rx_lane1_data;
`endif
`ifdef D2D_PHY_RX_LANE2_EN
    wire [CPIF_LP_WIDTH-1:0]    lp_cpif_rx_lane2_data;      // logic_phy 输出
    wire                        lp_cpif_rx_lane2_data_vld;
    wire [CPIF_LL_WIDTH-1:0]    ll_cpif_rx_lane2_data;
`endif
`ifdef D2D_PHY_RX_LANE3_EN
    wire [CPIF_LP_WIDTH-1:0]    lp_cpif_rx_lane3_data;      // logic_phy 输出
    wire                        lp_cpif_rx_lane3_data_vld;
    wire [CPIF_LL_WIDTH-1:0]    ll_cpif_rx_lane3_data;
`endif

    // --- CPIF 控制 / ready（全 lane 共享）---
    wire [3:0]                  ll_cpif_tx_mode;
    wire                        ll_cpif_tx_rst_n;
    wire                        ll_cpif_tx_lpc;
    wire [3:0]                  ll_cpif_rx_mode;
    wire                        ll_cpif_rx_rst_n;
    wire                        lp_cpif_tx_ready;           // logic_phy → 各 link ready_i 扇出
    wire                        lp_cpif_rx_ready;           // ctrl 侧无对端，悬空

    // --- CPIF 事件接口（全 lane 共享，ctrl ↔ logic_phy sideband 透传）---
    wire                        ev_tx_enable;
    wire                        ev_tx_req;
    wire [15:0]                 ev_tx_cmd;
    wire                        ev_tx_ack;
    wire [7:0]                  ev_tx_status;
    wire                        ev_rx_enable;
    wire                        ev_rx_req;
    wire [15:0]                 ev_rx_cmd;
    wire                        ev_rx_ack;
    wire [7:0]                  ev_rx_status;

    // --- MCU APB master（logic_phy）→ ctrl APB slave（决策①，paddr 截断 [15:0]）---
    wire                        mcu_apb_psel;
    wire                        mcu_apb_penable;
    wire                        mcu_apb_pwrite;
    wire [19:0]                 mcu_apb_paddr;              // [19:16] 不用（软件约束恒 0）
    wire [31:0]                 mcu_apb_pwdata;
    wire [31:0]                 mcu_apb_prdata;
    wire                        mcu_apb_pready;
    wire                        mcu_apb_pslverr;

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================
    // --------------------------------------------------------------------------
    // CPIF 位宽映射（决策④）：默认等宽直连；`else（link 侧 512）TX 零扩展 / RX 截断
    //     lane0/lane1：由 link 侧驱动（LL_LINK_LANE1 已使能）；
    //     lane2/lane3：见下方独立段（4T1R 物理存在、无 link 源 → 恒 0）。
    // --------------------------------------------------------------------------
`ifdef CPIF_DATA_WIDTH_EQ_1024
    // 等宽（1024 = 1024）：直连；32:1 运行时模式 link 侧 s2p_mode=512 高 512 恒 0
    assign lp_cpif_tx_lane0_data = ll_cpif_tx_lane0_data;
    assign ll_cpif_rx_lane0_data = lp_cpif_rx_lane0_data;
`else
    // TX：link 侧仅驱动低 512 bit，高 512 bit 补 0（32:1 模式高 512 不消费）
    assign lp_cpif_tx_lane0_data = {{(CPIF_LP_WIDTH-CPIF_LL_WIDTH){1'b0}}, ll_cpif_tx_lane0_data};
    // RX：logic_phy 全宽输出截断低 512 bit 送 link，高 512 bit 丢弃（32:1 模式无效位）
    assign ll_cpif_rx_lane0_data = lp_cpif_rx_lane0_data[CPIF_LL_WIDTH-1:0];
`endif

    // --------------------------------------------------------------------------
    // lane1 CPIF 数据（4T1R 非对称，OpenHappy Team）——与 lane2/3 同模式对齐（2026-09-01 修复）
    //   物理层 lane1（PHY_LANE_1 使能）与 ctrl/link 数据 lane1（LL_LINK_LANE1）解耦：
    //   lane1 物理存在但 LL_LINK_LANE1 未定义时无 link 数据源 → tie 0（idle），仍接 logic_phy。
    //   LL_LINK_LANE1 使能后走真实 link 源（等宽直连 / 512 零扩展+截断，与 lane0 同款）。
    //   所有 net 均有显式 wire 声明 + assign 驱动（default_nettype=NONE）。
    // --------------------------------------------------------------------------
`ifdef D2D_PHY_TX_LANE1_EN
`ifdef PRCL_TX_LANE_1
`ifdef CPIF_DATA_WIDTH_EQ_1024
    assign lp_cpif_tx_lane1_data = ll_cpif_tx_lane1_data;                    // 等宽直连
`else
    assign lp_cpif_tx_lane1_data = {{(CPIF_LP_WIDTH-CPIF_LL_WIDTH){1'b0}}, ll_cpif_tx_lane1_data};
`endif
`else
    // lane1 无 link 源（PRCL_TX_LANE_1 未定义），tie 0（idle）：TX 送 idle
    assign ll_cpif_tx_lane1_data     = {CPIF_LL_WIDTH{1'b0}};
    assign ll_cpif_tx_lane1_data_vld = 1'b0;
    assign lp_cpif_tx_lane1_data     = {CPIF_LP_WIDTH{1'b0}};
`endif
`endif
`ifdef D2D_PHY_RX_LANE1_EN
`ifdef PRCL_RX_LANE_1
`ifdef CPIF_DATA_WIDTH_EQ_1024
    assign ll_cpif_rx_lane1_data = lp_cpif_rx_lane1_data;                    // 等宽直连
`else
    assign ll_cpif_rx_lane1_data = lp_cpif_rx_lane1_data[CPIF_LL_WIDTH-1:0]; // 截断低 512
`endif
`else
    // lane1 无 link 源，tie 0（idle）：RX 不消费
    assign lp_cpif_rx_lane1_data     = {CPIF_LP_WIDTH{1'b0}};
    assign lp_cpif_rx_lane1_data_vld = 1'b0;
    assign ll_cpif_rx_lane1_data     = {CPIF_LL_WIDTH{1'b0}};
`endif
`endif

    // --------------------------------------------------------------------------
    // lane2/lane3 CPIF 数据（4T1R 非对称，OpenHappy Team）

    // --------------------------------------------------------------------------
    // lane2/lane3 CPIF 数据（4T1R 非对称，OpenHappy Team）
    //   物理层 4 TX lane（PHY_LANE_2/3 已使能）与 ctrl/link 层 2 数据 lane
    //   （LL_LINK_LANE1 定义、LL_LINK_LANE3 未定义）解耦——lane2/3 物理存在但无 link 数据源。
    //   ★ TX：lp/ll 数据由 PHY_LANE_2/3 决定声明，ll 源在 LL_LINK_LANE3 未定义时恒 0（idle），
    //     仍接 logic_phy 输入，保证 4 物理 TX lane 端口全部连通；
    //     LL_LINK_LANE3 使能后走真实 link 源（`ifdef LL_LINK_LANE3 else tie 0 结构已预留）。
    //   ★ RX：物理 RX lane2/3 使能时（D2D_PHY_RX_LANE2/3_EN）lp_rx 由 logic_phy 输出驱动
    //     （同门控），此处不得再恒 0 tie-off（否则 st 4RX1TX 下多驱动，2026-09-18 修复）；
    //     link 层无 RX lane2/3 数据时（PRCL_RX_LANE_2/3 未定义）ll 侧兜底恒 0。
    //   所有 net 均有显式 wire 声明 + 驱动源（assign 或 logic_phy 输出，default_nettype=NONE）。
    // --------------------------------------------------------------------------
`ifdef D2D_PHY_TX_LANE2_EN
`ifdef PRCL_TX_LANE_2
    assign lp_cpif_tx_lane2_data = {{(CPIF_LP_WIDTH-CPIF_LL_WIDTH){1'b0}}, ll_cpif_tx_lane2_data};
`else
    assign ll_cpif_tx_lane2_data     = {CPIF_LL_WIDTH{1'b0}};   // lane2 无 link 源，恒 0（idle）
    assign ll_cpif_tx_lane2_data_vld = 1'b0;
    assign lp_cpif_tx_lane2_data     = {CPIF_LP_WIDTH{1'b0}};
`endif
`endif
`ifdef D2D_PHY_RX_LANE2_EN
    // lp_cpif_rx_lane2_* 由 logic_phy 输出驱动（同门控）——无 tie-off（防多驱动）
`ifdef PRCL_RX_LANE_2
    assign ll_cpif_rx_lane2_data     = lp_cpif_rx_lane2_data[CPIF_LL_WIDTH-1:0];
`else
    assign ll_cpif_rx_lane2_data     = {CPIF_LL_WIDTH{1'b0}};
`endif
`endif
`ifdef D2D_PHY_TX_LANE3_EN
`ifdef PRCL_TX_LANE_3
    assign lp_cpif_tx_lane3_data = {{(CPIF_LP_WIDTH-CPIF_LL_WIDTH){1'b0}}, ll_cpif_tx_lane3_data};
`else
    assign ll_cpif_tx_lane3_data     = {CPIF_LL_WIDTH{1'b0}};   // lane3 无 link 源，恒 0（idle）
    assign ll_cpif_tx_lane3_data_vld = 1'b0;
    assign lp_cpif_tx_lane3_data     = {CPIF_LP_WIDTH{1'b0}};
`endif
`endif
`ifdef D2D_PHY_RX_LANE3_EN
    // lp_cpif_rx_lane3_* 由 logic_phy 输出驱动（同门控）——无 tie-off（防多驱动）
`ifdef PRCL_RX_LANE_3
    assign ll_cpif_rx_lane3_data     = lp_cpif_rx_lane3_data[CPIF_LL_WIDTH-1:0];
`else
    assign ll_cpif_rx_lane3_data     = {CPIF_LL_WIDTH{1'b0}};
`endif
`endif

// verilator lint_off UNUSEDSIGNAL
wire [3:0] unused_ok_apb_paddr_hi  = mcu_apb_paddr[19:16];    // APB 高位有意不用（决策①）
wire       unused_ok_cpif_rx_ready = lp_cpif_rx_ready;        // ctrl 侧无 cpif_rx_ready 对端
`ifndef CPIF_DATA_WIDTH_EQ_1024
wire [CPIF_LP_WIDTH-CPIF_LL_WIDTH-1:0] unused_ok_rx0_hi =
    lp_cpif_rx_lane0_data[CPIF_LP_WIDTH-1:CPIF_LL_WIDTH];     // RX 高 512 有意截断
`ifdef D2D_PHY_RX_LANE1_EN
wire [CPIF_LP_WIDTH-CPIF_LL_WIDTH-1:0] unused_ok_rx1_hi =
    lp_cpif_rx_lane1_data[CPIF_LP_WIDTH-1:CPIF_LL_WIDTH];
`endif
`ifdef D2D_PHY_RX_LANE2_EN
wire [CPIF_LP_WIDTH-CPIF_LL_WIDTH-1:0] unused_ok_rx2_hi =
    lp_cpif_rx_lane2_data[CPIF_LP_WIDTH-1:CPIF_LL_WIDTH];
`endif
`ifdef D2D_PHY_RX_LANE3_EN
wire [CPIF_LP_WIDTH-CPIF_LL_WIDTH-1:0] unused_ok_rx3_hi =
    lp_cpif_rx_lane3_data[CPIF_LP_WIDTH-1:CPIF_LL_WIDTH];
`endif
`endif
// verilator lint_on UNUSEDSIGNAL

    // ==========================================================================
    // 子模块实例化
    // ==========================================================================
    // --------------------------------------------------------------------------
    // u_hipi_ctrl_top（OpenHappy Team，原样例化不修改）
    // --------------------------------------------------------------------------
    hipi_ctrl_top u_hipi_ctrl_top (
        // 时钟与复位
        .pclk                       (pclk),
        .mb_clk                     (mb_clk),
        .sb_clk                     (sb_tx_clk_in),
        .presetn                    (presetn),
`ifdef PRCL_AXI
        .aclk                       (aclk),
        .aresetn                    (aresetn),
`else
        .cxs_clk                    (cxs_clk),
        .cxs_rstn                   (cxs_rstn),
`endif
        // APB slave ← logic_phy MCU master（paddr 截断 [15:0]，决策①）
        .psel                       (mcu_apb_psel),
        .penable                    (mcu_apb_penable),
        .paddr                      (mcu_apb_paddr[15:0]),
        .pwrite                     (mcu_apb_pwrite),
        .pwdata                     (mcu_apb_pwdata),
        .prdata                     (mcu_apb_prdata),
        .pready                     (mcu_apb_pready),
        .pslverr                    (mcu_apb_pslverr),
        // Low-power
        .lp_ctrl_cmd                (lp_ctrl_cmd),
        .lp_ctrl_req                (lp_ctrl_req),
        .lp_ctrl_sts                (lp_ctrl_sts),
        .lp_ctrl_ack                (lp_ctrl_ack),
        // 中断
        .hipi_ctrl_intr             (hipi_ctrl_intr),
`ifdef PRCL_AXI
`ifdef AXI_CMPT_DIE
        // ---- cmptdie 模式（cmptdie_axi4_prcl，4R1T）：s0+m0 恒在，s1~3 AR/R 随 MST_RX_LANE_n ----
        .s0_axi_awid                                (s0_axi_awid),
        .s0_axi_awaddr                              (s0_axi_awaddr),
        .s0_axi_awlen                               (s0_axi_awlen),
        .s0_axi_awsize                              (s0_axi_awsize),
        .s0_axi_awburst                             (s0_axi_awburst),
        .s0_axi_awlock                              (s0_axi_awlock),
        .s0_axi_awcache                             (s0_axi_awcache),
        .s0_axi_awprot                              (s0_axi_awprot),
        .s0_axi_awqos                               (s0_axi_awqos),
        .s0_axi_awregion                            (s0_axi_awregion),
        .s0_axi_awuser                              (s0_axi_awuser),
        .s0_axi_awvalid                             (s0_axi_awvalid),
        .s0_axi_awready                             (s0_axi_awready),
        .s0_axi_arid                                (s0_axi_arid),
        .s0_axi_araddr                              (s0_axi_araddr),
        .s0_axi_arlen                               (s0_axi_arlen),
        .s0_axi_arsize                              (s0_axi_arsize),
        .s0_axi_arburst                             (s0_axi_arburst),
        .s0_axi_arlock                              (s0_axi_arlock),
        .s0_axi_arcache                             (s0_axi_arcache),
        .s0_axi_arprot                              (s0_axi_arprot),
        .s0_axi_arqos                               (s0_axi_arqos),
        .s0_axi_arregion                            (s0_axi_arregion),
        .s0_axi_aruser                              (s0_axi_aruser),
        .s0_axi_arvalid                             (s0_axi_arvalid),
        .s0_axi_arready                             (s0_axi_arready),
        .s0_axi_wdata                               (s0_axi_wdata),
        .s0_axi_wstrb                               (s0_axi_wstrb),
        .s0_axi_wuser                               (s0_axi_wuser),
        .s0_axi_wlast                               (s0_axi_wlast),
        .s0_axi_wvalid                              (s0_axi_wvalid),
        .s0_axi_wready                              (s0_axi_wready),
        .s0_axi_rid                                 (s0_axi_rid),
        .s0_axi_rdata                               (s0_axi_rdata),
        .s0_axi_rresp                               (s0_axi_rresp),
        .s0_axi_ruser                               (s0_axi_ruser),
        .s0_axi_rlast                               (s0_axi_rlast),
        .s0_axi_rvalid                              (s0_axi_rvalid),
        .s0_axi_rready                              (s0_axi_rready),
        .s0_axi_bid                                 (s0_axi_bid),
        .s0_axi_bresp                               (s0_axi_bresp),
        .s0_axi_buser                               (s0_axi_buser),
        .s0_axi_bvalid                              (s0_axi_bvalid),
        .s0_axi_bready                              (s0_axi_bready),
        .m0_axi_awid                                (m0_axi_awid),
        .m0_axi_awaddr                              (m0_axi_awaddr),
        .m0_axi_awlen                               (m0_axi_awlen),
        .m0_axi_awsize                              (m0_axi_awsize),
        .m0_axi_awburst                             (m0_axi_awburst),
        .m0_axi_awlock                              (m0_axi_awlock),
        .m0_axi_awcache                             (m0_axi_awcache),
        .m0_axi_awprot                              (m0_axi_awprot),
        .m0_axi_awqos                               (m0_axi_awqos),
        .m0_axi_awregion                            (m0_axi_awregion),
        .m0_axi_awuser                              (m0_axi_awuser),
        .m0_axi_awvalid                             (m0_axi_awvalid),
        .m0_axi_awready                             (m0_axi_awready),
        .m0_axi_arid                                (m0_axi_arid),
        .m0_axi_araddr                              (m0_axi_araddr),
        .m0_axi_arlen                               (m0_axi_arlen),
        .m0_axi_arsize                              (m0_axi_arsize),
        .m0_axi_arburst                             (m0_axi_arburst),
        .m0_axi_arlock                              (m0_axi_arlock),
        .m0_axi_arcache                             (m0_axi_arcache),
        .m0_axi_arprot                              (m0_axi_arprot),
        .m0_axi_arqos                               (m0_axi_arqos),
        .m0_axi_arregion                            (m0_axi_arregion),
        .m0_axi_aruser                              (m0_axi_aruser),
        .m0_axi_arvalid                             (m0_axi_arvalid),
        .m0_axi_arready                             (m0_axi_arready),
        .m0_axi_wdata                               (m0_axi_wdata),
        .m0_axi_wstrb                               (m0_axi_wstrb),
        .m0_axi_wuser                               (m0_axi_wuser),
        .m0_axi_wlast                               (m0_axi_wlast),
        .m0_axi_wvalid                              (m0_axi_wvalid),
        .m0_axi_wready                              (m0_axi_wready),
        .m0_axi_rid                                 (m0_axi_rid),
        .m0_axi_rdata                               (m0_axi_rdata),
        .m0_axi_rresp                               (m0_axi_rresp),
        .m0_axi_ruser                               (m0_axi_ruser),
        .m0_axi_rlast                               (m0_axi_rlast),
        .m0_axi_rvalid                              (m0_axi_rvalid),
        .m0_axi_rready                              (m0_axi_rready),
        .m0_axi_bid                                 (m0_axi_bid),
        .m0_axi_bresp                               (m0_axi_bresp),
        .m0_axi_buser                               (m0_axi_buser),
        .m0_axi_bvalid                              (m0_axi_bvalid),
        .m0_axi_bready                              (m0_axi_bready),
        // ---- lane1~3：仅 RX AR 通道（随 MST_RX_LANE_n）----
`ifdef MST_RX_LANE_1
        .s1_axi_arid                                (s1_axi_arid),
        .s1_axi_araddr                              (s1_axi_araddr),
        .s1_axi_arlen                               (s1_axi_arlen),
        .s1_axi_arsize                              (s1_axi_arsize),
        .s1_axi_arburst                             (s1_axi_arburst),
        .s1_axi_arlock                              (s1_axi_arlock),
        .s1_axi_arcache                             (s1_axi_arcache),
        .s1_axi_arprot                              (s1_axi_arprot),
        .s1_axi_arqos                               (s1_axi_arqos),
        .s1_axi_arregion                            (s1_axi_arregion),
        .s1_axi_aruser                              (s1_axi_aruser),
        .s1_axi_arvalid                             (s1_axi_arvalid),
        .s1_axi_arready                             (s1_axi_arready),
        .s1_axi_rid                                 (s1_axi_rid),
        .s1_axi_rdata                               (s1_axi_rdata),
        .s1_axi_rresp                               (s1_axi_rresp),
        .s1_axi_ruser                               (s1_axi_ruser),
        .s1_axi_rlast                               (s1_axi_rlast),
        .s1_axi_rvalid                              (s1_axi_rvalid),
        .s1_axi_rready                              (s1_axi_rready),

`endif
`ifdef MST_RX_LANE_2
        .s2_axi_arid                                (s2_axi_arid),
        .s2_axi_araddr                              (s2_axi_araddr),
        .s2_axi_arlen                               (s2_axi_arlen),
        .s2_axi_arsize                              (s2_axi_arsize),
        .s2_axi_arburst                             (s2_axi_arburst),
        .s2_axi_arlock                              (s2_axi_arlock),
        .s2_axi_arcache                             (s2_axi_arcache),
        .s2_axi_arprot                              (s2_axi_arprot),
        .s2_axi_arqos                               (s2_axi_arqos),
        .s2_axi_arregion                            (s2_axi_arregion),
        .s2_axi_aruser                              (s2_axi_aruser),
        .s2_axi_arvalid                             (s2_axi_arvalid),
        .s2_axi_arready                             (s2_axi_arready),
        .s2_axi_rid                                 (s2_axi_rid),
        .s2_axi_rdata                               (s2_axi_rdata),
        .s2_axi_rresp                               (s2_axi_rresp),
        .s2_axi_ruser                               (s2_axi_ruser),
        .s2_axi_rlast                               (s2_axi_rlast),
        .s2_axi_rvalid                              (s2_axi_rvalid),
        .s2_axi_rready                              (s2_axi_rready),

`endif
`ifdef MST_RX_LANE_3
        .s3_axi_arid                                (s3_axi_arid),
        .s3_axi_araddr                              (s3_axi_araddr),
        .s3_axi_arlen                               (s3_axi_arlen),
        .s3_axi_arsize                              (s3_axi_arsize),
        .s3_axi_arburst                             (s3_axi_arburst),
        .s3_axi_arlock                              (s3_axi_arlock),
        .s3_axi_arcache                             (s3_axi_arcache),
        .s3_axi_arprot                              (s3_axi_arprot),
        .s3_axi_arqos                               (s3_axi_arqos),
        .s3_axi_arregion                            (s3_axi_arregion),
        .s3_axi_aruser                              (s3_axi_aruser),
        .s3_axi_arvalid                             (s3_axi_arvalid),
        .s3_axi_arready                             (s3_axi_arready),
        .s3_axi_rid                                 (s3_axi_rid),
        .s3_axi_rdata                               (s3_axi_rdata),
        .s3_axi_rresp                               (s3_axi_rresp),
        .s3_axi_ruser                               (s3_axi_ruser),
        .s3_axi_rlast                               (s3_axi_rlast),
        .s3_axi_rvalid                              (s3_axi_rvalid),
        .s3_axi_rready                              (s3_axi_rready),

`endif
`elsif AXI_IO_DIE
        .s0_axi_awid                                (s0_axi_awid),
        .s0_axi_awaddr                              (s0_axi_awaddr),
        .s0_axi_awlen                               (s0_axi_awlen),
        .s0_axi_awsize                              (s0_axi_awsize),
        .s0_axi_awburst                             (s0_axi_awburst),
        .s0_axi_awlock                              (s0_axi_awlock),
        .s0_axi_awcache                             (s0_axi_awcache),
        .s0_axi_awprot                              (s0_axi_awprot),
        .s0_axi_awqos                               (s0_axi_awqos),
        .s0_axi_awregion                            (s0_axi_awregion),
        .s0_axi_awuser                              (s0_axi_awuser),
        .s0_axi_awvalid                             (s0_axi_awvalid),
        .s0_axi_awready                             (s0_axi_awready),
        .s0_axi_arid                                (s0_axi_arid),
        .s0_axi_araddr                              (s0_axi_araddr),
        .s0_axi_arlen                               (s0_axi_arlen),
        .s0_axi_arsize                              (s0_axi_arsize),
        .s0_axi_arburst                             (s0_axi_arburst),
        .s0_axi_arlock                              (s0_axi_arlock),
        .s0_axi_arcache                             (s0_axi_arcache),
        .s0_axi_arprot                              (s0_axi_arprot),
        .s0_axi_arqos                               (s0_axi_arqos),
        .s0_axi_arregion                            (s0_axi_arregion),
        .s0_axi_aruser                              (s0_axi_aruser),
        .s0_axi_arvalid                             (s0_axi_arvalid),
        .s0_axi_arready                             (s0_axi_arready),
        .s0_axi_wdata                               (s0_axi_wdata),
        .s0_axi_wstrb                               (s0_axi_wstrb),
        .s0_axi_wuser                               (s0_axi_wuser),
        .s0_axi_wlast                               (s0_axi_wlast),
        .s0_axi_wvalid                              (s0_axi_wvalid),
        .s0_axi_wready                              (s0_axi_wready),
        .s0_axi_rid                                 (s0_axi_rid),
        .s0_axi_rdata                               (s0_axi_rdata),
        .s0_axi_rresp                               (s0_axi_rresp),
        .s0_axi_ruser                               (s0_axi_ruser),
        .s0_axi_rlast                               (s0_axi_rlast),
        .s0_axi_rvalid                              (s0_axi_rvalid),
        .s0_axi_rready                              (s0_axi_rready),
        .s0_axi_bid                                 (s0_axi_bid),
        .s0_axi_bresp                               (s0_axi_bresp),
        .s0_axi_buser                               (s0_axi_buser),
        .s0_axi_bvalid                              (s0_axi_bvalid),
        .s0_axi_bready                              (s0_axi_bready),
        .m0_axi_awid                                (m0_axi_awid),
        .m0_axi_awaddr                              (m0_axi_awaddr),
        .m0_axi_awlen                               (m0_axi_awlen),
        .m0_axi_awsize                              (m0_axi_awsize),
        .m0_axi_awburst                             (m0_axi_awburst),
        .m0_axi_awlock                              (m0_axi_awlock),
        .m0_axi_awcache                             (m0_axi_awcache),
        .m0_axi_awprot                              (m0_axi_awprot),
        .m0_axi_awqos                               (m0_axi_awqos),
        .m0_axi_awregion                            (m0_axi_awregion),
        .m0_axi_awuser                              (m0_axi_awuser),
        .m0_axi_awvalid                             (m0_axi_awvalid),
        .m0_axi_awready                             (m0_axi_awready),
        .m0_axi_arid                                (m0_axi_arid),
        .m0_axi_araddr                              (m0_axi_araddr),
        .m0_axi_arlen                               (m0_axi_arlen),
        .m0_axi_arsize                              (m0_axi_arsize),
        .m0_axi_arburst                             (m0_axi_arburst),
        .m0_axi_arlock                              (m0_axi_arlock),
        .m0_axi_arcache                             (m0_axi_arcache),
        .m0_axi_arprot                              (m0_axi_arprot),
        .m0_axi_arqos                               (m0_axi_arqos),
        .m0_axi_arregion                            (m0_axi_arregion),
        .m0_axi_aruser                              (m0_axi_aruser),
        .m0_axi_arvalid                             (m0_axi_arvalid),
        .m0_axi_arready                             (m0_axi_arready),
        .m0_axi_wdata                               (m0_axi_wdata),
        .m0_axi_wstrb                               (m0_axi_wstrb),
        .m0_axi_wuser                               (m0_axi_wuser),
        .m0_axi_wlast                               (m0_axi_wlast),
        .m0_axi_wvalid                              (m0_axi_wvalid),
        .m0_axi_wready                              (m0_axi_wready),
        .m0_axi_rid                                 (m0_axi_rid),
        .m0_axi_rdata                               (m0_axi_rdata),
        .m0_axi_rresp                               (m0_axi_rresp),
        .m0_axi_ruser                               (m0_axi_ruser),
        .m0_axi_rlast                               (m0_axi_rlast),
        .m0_axi_rvalid                              (m0_axi_rvalid),
        .m0_axi_rready                              (m0_axi_rready),
        .m0_axi_bid                                 (m0_axi_bid),
        .m0_axi_bresp                               (m0_axi_bresp),
        .m0_axi_buser                               (m0_axi_buser),
        .m0_axi_bvalid                              (m0_axi_bvalid),
        .m0_axi_bready                              (m0_axi_bready),
`ifdef SLV_TX_LANE_1
        .m1_axi_arid                                (m1_axi_arid),
        .m1_axi_araddr                              (m1_axi_araddr),
        .m1_axi_arlen                               (m1_axi_arlen),
        .m1_axi_arsize                              (m1_axi_arsize),
        .m1_axi_arburst                             (m1_axi_arburst),
        .m1_axi_arlock                              (m1_axi_arlock),
        .m1_axi_arcache                             (m1_axi_arcache),
        .m1_axi_arprot                              (m1_axi_arprot),
        .m1_axi_arqos                               (m1_axi_arqos),
        .m1_axi_arregion                            (m1_axi_arregion),
        .m1_axi_aruser                              (m1_axi_aruser),
        .m1_axi_arvalid                             (m1_axi_arvalid),
        .m1_axi_arready                             (m1_axi_arready),
        .m1_axi_rid                                 (m1_axi_rid),
        .m1_axi_rdata                               (m1_axi_rdata),
        .m1_axi_rresp                               (m1_axi_rresp),
        .m1_axi_ruser                               (m1_axi_ruser),
        .m1_axi_rlast                               (m1_axi_rlast),
        .m1_axi_rvalid                              (m1_axi_rvalid),
        .m1_axi_rready                              (m1_axi_rready),
`endif
`ifdef SLV_TX_LANE_2
        .m2_axi_arid                                (m2_axi_arid),
        .m2_axi_araddr                              (m2_axi_araddr),
        .m2_axi_arlen                               (m2_axi_arlen),
        .m2_axi_arsize                              (m2_axi_arsize),
        .m2_axi_arburst                             (m2_axi_arburst),
        .m2_axi_arlock                              (m2_axi_arlock),
        .m2_axi_arcache                             (m2_axi_arcache),
        .m2_axi_arprot                              (m2_axi_arprot),
        .m2_axi_arqos                               (m2_axi_arqos),
        .m2_axi_arregion                            (m2_axi_arregion),
        .m2_axi_aruser                              (m2_axi_aruser),
        .m2_axi_arvalid                             (m2_axi_arvalid),
        .m2_axi_arready                             (m2_axi_arready),
        .m2_axi_rid                                 (m2_axi_rid),
        .m2_axi_rdata                               (m2_axi_rdata),
        .m2_axi_rresp                               (m2_axi_rresp),
        .m2_axi_ruser                               (m2_axi_ruser),
        .m2_axi_rlast                               (m2_axi_rlast),
        .m2_axi_rvalid                              (m2_axi_rvalid),
        .m2_axi_rready                              (m2_axi_rready),
`endif
`ifdef SLV_TX_LANE_3
        .m3_axi_arid                                (m3_axi_arid),
        .m3_axi_araddr                              (m3_axi_araddr),
        .m3_axi_arlen                               (m3_axi_arlen),
        .m3_axi_arsize                              (m3_axi_arsize),
        .m3_axi_arburst                             (m3_axi_arburst),
        .m3_axi_arlock                              (m3_axi_arlock),
        .m3_axi_arcache                             (m3_axi_arcache),
        .m3_axi_arprot                              (m3_axi_arprot),
        .m3_axi_arqos                               (m3_axi_arqos),
        .m3_axi_arregion                            (m3_axi_arregion),
        .m3_axi_aruser                              (m3_axi_aruser),
        .m3_axi_arvalid                             (m3_axi_arvalid),
        .m3_axi_arready                             (m3_axi_arready),
        .m3_axi_rid                                 (m3_axi_rid),
        .m3_axi_rdata                               (m3_axi_rdata),
        .m3_axi_rresp                               (m3_axi_rresp),
        .m3_axi_ruser                               (m3_axi_ruser),
        .m3_axi_rlast                               (m3_axi_rlast),
        .m3_axi_rvalid                              (m3_axi_rvalid),
        .m3_axi_rready                              (m3_axi_rready),
`endif
`else
`ifdef AXI_ASYM_MODE
`ifdef MASTER_DIE
        .s0_axi_awid                                (s0_axi_awid),
        .s0_axi_awaddr                              (s0_axi_awaddr),
        .s0_axi_awlen                               (s0_axi_awlen),
        .s0_axi_awsize                              (s0_axi_awsize),
        .s0_axi_awburst                             (s0_axi_awburst),
        .s0_axi_awlock                              (s0_axi_awlock),
        .s0_axi_awcache                             (s0_axi_awcache),
        .s0_axi_awprot                              (s0_axi_awprot),
        .s0_axi_awqos                               (s0_axi_awqos),
        .s0_axi_awregion                            (s0_axi_awregion),
        .s0_axi_awuser                              (s0_axi_awuser),
        .s0_axi_awvalid                             (s0_axi_awvalid),
        .s0_axi_awready                             (s0_axi_awready),
`ifdef MST_TX_LANE_1
        .s1_axi_awid                                (s1_axi_awid),
        .s1_axi_awaddr                              (s1_axi_awaddr),
        .s1_axi_awlen                               (s1_axi_awlen),
        .s1_axi_awsize                              (s1_axi_awsize),
        .s1_axi_awburst                             (s1_axi_awburst),
        .s1_axi_awlock                              (s1_axi_awlock),
        .s1_axi_awcache                             (s1_axi_awcache),
        .s1_axi_awprot                              (s1_axi_awprot),
        .s1_axi_awqos                               (s1_axi_awqos),
        .s1_axi_awregion                            (s1_axi_awregion),
        .s1_axi_awuser                              (s1_axi_awuser),
        .s1_axi_awvalid                             (s1_axi_awvalid),
        .s1_axi_awready                             (s1_axi_awready),
`endif
`ifdef MST_TX_LANE_2
        .s2_axi_awid                                (s2_axi_awid),
        .s2_axi_awaddr                              (s2_axi_awaddr),
        .s2_axi_awlen                               (s2_axi_awlen),
        .s2_axi_awsize                              (s2_axi_awsize),
        .s2_axi_awburst                             (s2_axi_awburst),
        .s2_axi_awlock                              (s2_axi_awlock),
        .s2_axi_awcache                             (s2_axi_awcache),
        .s2_axi_awprot                              (s2_axi_awprot),
        .s2_axi_awqos                               (s2_axi_awqos),
        .s2_axi_awregion                            (s2_axi_awregion),
        .s2_axi_awuser                              (s2_axi_awuser),
        .s2_axi_awvalid                             (s2_axi_awvalid),
        .s2_axi_awready                             (s2_axi_awready),
`endif
`ifdef MST_TX_LANE_3
        .s3_axi_awid                                (s3_axi_awid),
        .s3_axi_awaddr                              (s3_axi_awaddr),
        .s3_axi_awlen                               (s3_axi_awlen),
        .s3_axi_awsize                              (s3_axi_awsize),
        .s3_axi_awburst                             (s3_axi_awburst),
        .s3_axi_awlock                              (s3_axi_awlock),
        .s3_axi_awcache                             (s3_axi_awcache),
        .s3_axi_awprot                              (s3_axi_awprot),
        .s3_axi_awqos                               (s3_axi_awqos),
        .s3_axi_awregion                            (s3_axi_awregion),
        .s3_axi_awuser                              (s3_axi_awuser),
        .s3_axi_awvalid                             (s3_axi_awvalid),
        .s3_axi_awready                             (s3_axi_awready),
`endif
        .s0_axi_arid                                (s0_axi_arid),
        .s0_axi_araddr                              (s0_axi_araddr),
        .s0_axi_arlen                               (s0_axi_arlen),
        .s0_axi_arsize                              (s0_axi_arsize),
        .s0_axi_arburst                             (s0_axi_arburst),
        .s0_axi_arlock                              (s0_axi_arlock),
        .s0_axi_arcache                             (s0_axi_arcache),
        .s0_axi_arprot                              (s0_axi_arprot),
        .s0_axi_arqos                               (s0_axi_arqos),
        .s0_axi_arregion                            (s0_axi_arregion),
        .s0_axi_aruser                              (s0_axi_aruser),
        .s0_axi_arvalid                             (s0_axi_arvalid),
        .s0_axi_arready                             (s0_axi_arready),
`ifdef MST_RX_LANE_1
        .s1_axi_arid                                (s1_axi_arid),
        .s1_axi_araddr                              (s1_axi_araddr),
        .s1_axi_arlen                               (s1_axi_arlen),
        .s1_axi_arsize                              (s1_axi_arsize),
        .s1_axi_arburst                             (s1_axi_arburst),
        .s1_axi_arlock                              (s1_axi_arlock),
        .s1_axi_arcache                             (s1_axi_arcache),
        .s1_axi_arprot                              (s1_axi_arprot),
        .s1_axi_arqos                               (s1_axi_arqos),
        .s1_axi_arregion                            (s1_axi_arregion),
        .s1_axi_aruser                              (s1_axi_aruser),
        .s1_axi_arvalid                             (s1_axi_arvalid),
        .s1_axi_arready                             (s1_axi_arready),
`endif
`ifdef MST_RX_LANE_2
        .s2_axi_arid                                (s2_axi_arid),
        .s2_axi_araddr                              (s2_axi_araddr),
        .s2_axi_arlen                               (s2_axi_arlen),
        .s2_axi_arsize                              (s2_axi_arsize),
        .s2_axi_arburst                             (s2_axi_arburst),
        .s2_axi_arlock                              (s2_axi_arlock),
        .s2_axi_arcache                             (s2_axi_arcache),
        .s2_axi_arprot                              (s2_axi_arprot),
        .s2_axi_arqos                               (s2_axi_arqos),
        .s2_axi_arregion                            (s2_axi_arregion),
        .s2_axi_aruser                              (s2_axi_aruser),
        .s2_axi_arvalid                             (s2_axi_arvalid),
        .s2_axi_arready                             (s2_axi_arready),
`endif
`ifdef MST_RX_LANE_3
        .s3_axi_arid                                (s3_axi_arid),
        .s3_axi_araddr                              (s3_axi_araddr),
        .s3_axi_arlen                               (s3_axi_arlen),
        .s3_axi_arsize                              (s3_axi_arsize),
        .s3_axi_arburst                             (s3_axi_arburst),
        .s3_axi_arlock                              (s3_axi_arlock),
        .s3_axi_arcache                             (s3_axi_arcache),
        .s3_axi_arprot                              (s3_axi_arprot),
        .s3_axi_arqos                               (s3_axi_arqos),
        .s3_axi_arregion                            (s3_axi_arregion),
        .s3_axi_aruser                              (s3_axi_aruser),
        .s3_axi_arvalid                             (s3_axi_arvalid),
        .s3_axi_arready                             (s3_axi_arready),
`endif
        .s0_axi_wdata                               (s0_axi_wdata),
        .s0_axi_wstrb                               (s0_axi_wstrb),
        .s0_axi_wuser                               (s0_axi_wuser),
        .s0_axi_wlast                               (s0_axi_wlast),
        .s0_axi_wvalid                              (s0_axi_wvalid),
        .s0_axi_wready                              (s0_axi_wready),
`ifdef MST_TX_LANE_1
        .s1_axi_wdata                               (s1_axi_wdata),
        .s1_axi_wstrb                               (s1_axi_wstrb),
        .s1_axi_wuser                               (s1_axi_wuser),
        .s1_axi_wlast                               (s1_axi_wlast),
        .s1_axi_wvalid                              (s1_axi_wvalid),
        .s1_axi_wready                              (s1_axi_wready),
`endif
`ifdef MST_TX_LANE_2
        .s2_axi_wdata                               (s2_axi_wdata),
        .s2_axi_wstrb                               (s2_axi_wstrb),
        .s2_axi_wuser                               (s2_axi_wuser),
        .s2_axi_wlast                               (s2_axi_wlast),
        .s2_axi_wvalid                              (s2_axi_wvalid),
        .s2_axi_wready                              (s2_axi_wready),
`endif
`ifdef MST_TX_LANE_3
        .s3_axi_wdata                               (s3_axi_wdata),
        .s3_axi_wstrb                               (s3_axi_wstrb),
        .s3_axi_wuser                               (s3_axi_wuser),
        .s3_axi_wlast                               (s3_axi_wlast),
        .s3_axi_wvalid                              (s3_axi_wvalid),
        .s3_axi_wready                              (s3_axi_wready),
`endif
        .s0_axi_rid                                 (s0_axi_rid),
        .s0_axi_rdata                               (s0_axi_rdata),
        .s0_axi_rresp                               (s0_axi_rresp),
        .s0_axi_ruser                               (s0_axi_ruser),
        .s0_axi_rlast                               (s0_axi_rlast),
        .s0_axi_rvalid                              (s0_axi_rvalid),
        .s0_axi_rready                              (s0_axi_rready),
`ifdef MST_RX_LANE_1
        .s1_axi_rid                                 (s1_axi_rid),
        .s1_axi_rdata                               (s1_axi_rdata),
        .s1_axi_rresp                               (s1_axi_rresp),
        .s1_axi_ruser                               (s1_axi_ruser),
        .s1_axi_rlast                               (s1_axi_rlast),
        .s1_axi_rvalid                              (s1_axi_rvalid),
        .s1_axi_rready                              (s1_axi_rready),
`endif
`ifdef MST_RX_LANE_2
        .s2_axi_rid                                 (s2_axi_rid),
        .s2_axi_rdata                               (s2_axi_rdata),
        .s2_axi_rresp                               (s2_axi_rresp),
        .s2_axi_ruser                               (s2_axi_ruser),
        .s2_axi_rlast                               (s2_axi_rlast),
        .s2_axi_rvalid                              (s2_axi_rvalid),
        .s2_axi_rready                              (s2_axi_rready),
`endif
`ifdef MST_RX_LANE_3
        .s3_axi_rid                                 (s3_axi_rid),
        .s3_axi_rdata                               (s3_axi_rdata),
        .s3_axi_rresp                               (s3_axi_rresp),
        .s3_axi_ruser                               (s3_axi_ruser),
        .s3_axi_rlast                               (s3_axi_rlast),
        .s3_axi_rvalid                              (s3_axi_rvalid),
        .s3_axi_rready                              (s3_axi_rready),
`endif
        .s0_axi_bid                                 (s0_axi_bid),
        .s0_axi_bresp                               (s0_axi_bresp),
        .s0_axi_buser                               (s0_axi_buser),
        .s0_axi_bvalid                              (s0_axi_bvalid),
        .s0_axi_bready                              (s0_axi_bready),
`ifdef MST_TX_LANE_1
        .s1_axi_bid                                 (s1_axi_bid),
        .s1_axi_bresp                               (s1_axi_bresp),
        .s1_axi_buser                               (s1_axi_buser),
        .s1_axi_bvalid                              (s1_axi_bvalid),
        .s1_axi_bready                              (s1_axi_bready),
`endif
`ifdef MST_TX_LANE_2
        .s2_axi_bid                                 (s2_axi_bid),
        .s2_axi_bresp                               (s2_axi_bresp),
        .s2_axi_buser                               (s2_axi_buser),
        .s2_axi_bvalid                              (s2_axi_bvalid),
        .s2_axi_bready                              (s2_axi_bready),
`endif
`ifdef MST_TX_LANE_3
        .s3_axi_bid                                 (s3_axi_bid),
        .s3_axi_bresp                               (s3_axi_bresp),
        .s3_axi_buser                               (s3_axi_buser),
        .s3_axi_bvalid                              (s3_axi_bvalid),
        .s3_axi_bready                              (s3_axi_bready),
`endif
`endif
`ifdef SLAVE_DIE
        .m0_axi_awid                                (m0_axi_awid),
        .m0_axi_awaddr                              (m0_axi_awaddr),
        .m0_axi_awlen                               (m0_axi_awlen),
        .m0_axi_awsize                              (m0_axi_awsize),
        .m0_axi_awburst                             (m0_axi_awburst),
        .m0_axi_awlock                              (m0_axi_awlock),
        .m0_axi_awcache                             (m0_axi_awcache),
        .m0_axi_awprot                              (m0_axi_awprot),
        .m0_axi_awqos                               (m0_axi_awqos),
        .m0_axi_awregion                            (m0_axi_awregion),
        .m0_axi_awuser                              (m0_axi_awuser),
        .m0_axi_awvalid                             (m0_axi_awvalid),
        .m0_axi_awready                             (m0_axi_awready),
`ifdef SLV_RX_LANE_1
        .m1_axi_awid                                (m1_axi_awid),
        .m1_axi_awaddr                              (m1_axi_awaddr),
        .m1_axi_awlen                               (m1_axi_awlen),
        .m1_axi_awsize                              (m1_axi_awsize),
        .m1_axi_awburst                             (m1_axi_awburst),
        .m1_axi_awlock                              (m1_axi_awlock),
        .m1_axi_awcache                             (m1_axi_awcache),
        .m1_axi_awprot                              (m1_axi_awprot),
        .m1_axi_awqos                               (m1_axi_awqos),
        .m1_axi_awregion                            (m1_axi_awregion),
        .m1_axi_awuser                              (m1_axi_awuser),
        .m1_axi_awvalid                             (m1_axi_awvalid),
        .m1_axi_awready                             (m1_axi_awready),
`endif
`ifdef SLV_RX_LANE_2
        .m2_axi_awid                                (m2_axi_awid),
        .m2_axi_awaddr                              (m2_axi_awaddr),
        .m2_axi_awlen                               (m2_axi_awlen),
        .m2_axi_awsize                              (m2_axi_awsize),
        .m2_axi_awburst                             (m2_axi_awburst),
        .m2_axi_awlock                              (m2_axi_awlock),
        .m2_axi_awcache                             (m2_axi_awcache),
        .m2_axi_awprot                              (m2_axi_awprot),
        .m2_axi_awqos                               (m2_axi_awqos),
        .m2_axi_awregion                            (m2_axi_awregion),
        .m2_axi_awuser                              (m2_axi_awuser),
        .m2_axi_awvalid                             (m2_axi_awvalid),
        .m2_axi_awready                             (m2_axi_awready),
`endif
`ifdef SLV_RX_LANE_3
        .m3_axi_awid                                (m3_axi_awid),
        .m3_axi_awaddr                              (m3_axi_awaddr),
        .m3_axi_awlen                               (m3_axi_awlen),
        .m3_axi_awsize                              (m3_axi_awsize),
        .m3_axi_awburst                             (m3_axi_awburst),
        .m3_axi_awlock                              (m3_axi_awlock),
        .m3_axi_awcache                             (m3_axi_awcache),
        .m3_axi_awprot                              (m3_axi_awprot),
        .m3_axi_awqos                               (m3_axi_awqos),
        .m3_axi_awregion                            (m3_axi_awregion),
        .m3_axi_awuser                              (m3_axi_awuser),
        .m3_axi_awvalid                             (m3_axi_awvalid),
        .m3_axi_awready                             (m3_axi_awready),
`endif
        .m0_axi_arid                                (m0_axi_arid),
        .m0_axi_araddr                              (m0_axi_araddr),
        .m0_axi_arlen                               (m0_axi_arlen),
        .m0_axi_arsize                              (m0_axi_arsize),
        .m0_axi_arburst                             (m0_axi_arburst),
        .m0_axi_arlock                              (m0_axi_arlock),
        .m0_axi_arcache                             (m0_axi_arcache),
        .m0_axi_arprot                              (m0_axi_arprot),
        .m0_axi_arqos                               (m0_axi_arqos),
        .m0_axi_arregion                            (m0_axi_arregion),
        .m0_axi_aruser                              (m0_axi_aruser),
        .m0_axi_arvalid                             (m0_axi_arvalid),
        .m0_axi_arready                             (m0_axi_arready),
`ifdef SLV_TX_LANE_1
        .m1_axi_arid                                (m1_axi_arid),
        .m1_axi_araddr                              (m1_axi_araddr),
        .m1_axi_arlen                               (m1_axi_arlen),
        .m1_axi_arsize                              (m1_axi_arsize),
        .m1_axi_arburst                             (m1_axi_arburst),
        .m1_axi_arlock                              (m1_axi_arlock),
        .m1_axi_arcache                             (m1_axi_arcache),
        .m1_axi_arprot                              (m1_axi_arprot),
        .m1_axi_arqos                               (m1_axi_arqos),
        .m1_axi_arregion                            (m1_axi_arregion),
        .m1_axi_aruser                              (m1_axi_aruser),
        .m1_axi_arvalid                             (m1_axi_arvalid),
        .m1_axi_arready                             (m1_axi_arready),
`endif
`ifdef SLV_TX_LANE_2
        .m2_axi_arid                                (m2_axi_arid),
        .m2_axi_araddr                              (m2_axi_araddr),
        .m2_axi_arlen                               (m2_axi_arlen),
        .m2_axi_arsize                              (m2_axi_arsize),
        .m2_axi_arburst                             (m2_axi_arburst),
        .m2_axi_arlock                              (m2_axi_arlock),
        .m2_axi_arcache                             (m2_axi_arcache),
        .m2_axi_arprot                              (m2_axi_arprot),
        .m2_axi_arqos                               (m2_axi_arqos),
        .m2_axi_arregion                            (m2_axi_arregion),
        .m2_axi_aruser                              (m2_axi_aruser),
        .m2_axi_arvalid                             (m2_axi_arvalid),
        .m2_axi_arready                             (m2_axi_arready),
`endif
`ifdef SLV_TX_LANE_3
        .m3_axi_arid                                (m3_axi_arid),
        .m3_axi_araddr                              (m3_axi_araddr),
        .m3_axi_arlen                               (m3_axi_arlen),
        .m3_axi_arsize                              (m3_axi_arsize),
        .m3_axi_arburst                             (m3_axi_arburst),
        .m3_axi_arlock                              (m3_axi_arlock),
        .m3_axi_arcache                             (m3_axi_arcache),
        .m3_axi_arprot                              (m3_axi_arprot),
        .m3_axi_arqos                               (m3_axi_arqos),
        .m3_axi_arregion                            (m3_axi_arregion),
        .m3_axi_aruser                              (m3_axi_aruser),
        .m3_axi_arvalid                             (m3_axi_arvalid),
        .m3_axi_arready                             (m3_axi_arready),
`endif
        .m0_axi_wdata                               (m0_axi_wdata),
        .m0_axi_wstrb                               (m0_axi_wstrb),
        .m0_axi_wuser                               (m0_axi_wuser),
        .m0_axi_wlast                               (m0_axi_wlast),
        .m0_axi_wvalid                              (m0_axi_wvalid),
        .m0_axi_wready                              (m0_axi_wready),
`ifdef SLV_RX_LANE_1
        .m1_axi_wdata                               (m1_axi_wdata),
        .m1_axi_wstrb                               (m1_axi_wstrb),
        .m1_axi_wuser                               (m1_axi_wuser),
        .m1_axi_wlast                               (m1_axi_wlast),
        .m1_axi_wvalid                              (m1_axi_wvalid),
        .m1_axi_wready                              (m1_axi_wready),
`endif
`ifdef SLV_RX_LANE_2
        .m2_axi_wdata                               (m2_axi_wdata),
        .m2_axi_wstrb                               (m2_axi_wstrb),
        .m2_axi_wuser                               (m2_axi_wuser),
        .m2_axi_wlast                               (m2_axi_wlast),
        .m2_axi_wvalid                              (m2_axi_wvalid),
        .m2_axi_wready                              (m2_axi_wready),
`endif
`ifdef SLV_RX_LANE_3
        .m3_axi_wdata                               (m3_axi_wdata),
        .m3_axi_wstrb                               (m3_axi_wstrb),
        .m3_axi_wuser                               (m3_axi_wuser),
        .m3_axi_wlast                               (m3_axi_wlast),
        .m3_axi_wvalid                              (m3_axi_wvalid),
        .m3_axi_wready                              (m3_axi_wready),
`endif
        .m0_axi_rid                                 (m0_axi_rid),
        .m0_axi_rdata                               (m0_axi_rdata),
        .m0_axi_rresp                               (m0_axi_rresp),
        .m0_axi_ruser                               (m0_axi_ruser),
        .m0_axi_rlast                               (m0_axi_rlast),
        .m0_axi_rvalid                              (m0_axi_rvalid),
        .m0_axi_rready                              (m0_axi_rready),
`ifdef SLV_TX_LANE_1
        .m1_axi_rid                                 (m1_axi_rid),
        .m1_axi_rdata                               (m1_axi_rdata),
        .m1_axi_rresp                               (m1_axi_rresp),
        .m1_axi_ruser                               (m1_axi_ruser),
        .m1_axi_rlast                               (m1_axi_rlast),
        .m1_axi_rvalid                              (m1_axi_rvalid),
        .m1_axi_rready                              (m1_axi_rready),
`endif
`ifdef SLV_TX_LANE_2
        .m2_axi_rid                                 (m2_axi_rid),
        .m2_axi_rdata                               (m2_axi_rdata),
        .m2_axi_rresp                               (m2_axi_rresp),
        .m2_axi_ruser                               (m2_axi_ruser),
        .m2_axi_rlast                               (m2_axi_rlast),
        .m2_axi_rvalid                              (m2_axi_rvalid),
        .m2_axi_rready                              (m2_axi_rready),
`endif
`ifdef SLV_TX_LANE_3
        .m3_axi_rid                                 (m3_axi_rid),
        .m3_axi_rdata                               (m3_axi_rdata),
        .m3_axi_rresp                               (m3_axi_rresp),
        .m3_axi_ruser                               (m3_axi_ruser),
        .m3_axi_rlast                               (m3_axi_rlast),
        .m3_axi_rvalid                              (m3_axi_rvalid),
        .m3_axi_rready                              (m3_axi_rready),
`endif
        .m0_axi_bid                                 (m0_axi_bid),
        .m0_axi_bresp                               (m0_axi_bresp),
        .m0_axi_buser                               (m0_axi_buser),
        .m0_axi_bvalid                              (m0_axi_bvalid),
        .m0_axi_bready                              (m0_axi_bready),
`ifdef SLV_RX_LANE_1
        .m1_axi_bid                                 (m1_axi_bid),
        .m1_axi_bresp                               (m1_axi_bresp),
        .m1_axi_buser                               (m1_axi_buser),
        .m1_axi_bvalid                              (m1_axi_bvalid),
        .m1_axi_bready                              (m1_axi_bready),
`endif
`ifdef SLV_RX_LANE_2
        .m2_axi_bid                                 (m2_axi_bid),
        .m2_axi_bresp                               (m2_axi_bresp),
        .m2_axi_buser                               (m2_axi_buser),
        .m2_axi_bvalid                              (m2_axi_bvalid),
        .m2_axi_bready                              (m2_axi_bready),
`endif
`ifdef SLV_RX_LANE_3
        .m3_axi_bid                                 (m3_axi_bid),
        .m3_axi_bresp                               (m3_axi_bresp),
        .m3_axi_buser                               (m3_axi_buser),
        .m3_axi_bvalid                              (m3_axi_bvalid),
        .m3_axi_bready                              (m3_axi_bready),
`endif
`endif
`else
        // ---- 对称 AXI（AXI_ASYM_MODE 未定义）----
        .s0_axi_awid(s0_axi_awid),
        .s0_axi_awaddr(s0_axi_awaddr),
        .s0_axi_awlen(s0_axi_awlen),
        .s0_axi_awsize(s0_axi_awsize),
        .s0_axi_awburst(s0_axi_awburst),
        .s0_axi_awlock(s0_axi_awlock),
        .s0_axi_awcache(s0_axi_awcache),
        .s0_axi_awprot(s0_axi_awprot),
        .s0_axi_awqos(s0_axi_awqos),
        .s0_axi_awregion(s0_axi_awregion),
        .s0_axi_awuser(s0_axi_awuser),
        .s0_axi_awvalid(s0_axi_awvalid),
        .s0_axi_awready(s0_axi_awready),
        .s0_axi_arid(s0_axi_arid),
        .s0_axi_araddr(s0_axi_araddr),
        .s0_axi_arlen(s0_axi_arlen),
        .s0_axi_arsize(s0_axi_arsize),
        .s0_axi_arburst(s0_axi_arburst),
        .s0_axi_arlock(s0_axi_arlock),
        .s0_axi_arcache(s0_axi_arcache),
        .s0_axi_arprot(s0_axi_arprot),
        .s0_axi_arqos(s0_axi_arqos),
        .s0_axi_arregion(s0_axi_arregion),
        .s0_axi_aruser(s0_axi_aruser),
        .s0_axi_arvalid(s0_axi_arvalid),
        .s0_axi_arready(s0_axi_arready),
        .s0_axi_wdata(s0_axi_wdata),
        .s0_axi_wstrb(s0_axi_wstrb),
        .s0_axi_wuser(s0_axi_wuser),
        .s0_axi_wlast(s0_axi_wlast),
        .s0_axi_wvalid(s0_axi_wvalid),
        .s0_axi_wready(s0_axi_wready),
        .s0_axi_rid(s0_axi_rid),
        .s0_axi_rdata(s0_axi_rdata),
        .s0_axi_rresp(s0_axi_rresp),
        .s0_axi_ruser(s0_axi_ruser),
        .s0_axi_rlast(s0_axi_rlast),
        .s0_axi_rvalid(s0_axi_rvalid),
        .s0_axi_rready(s0_axi_rready),
        .s0_axi_bid(s0_axi_bid),
        .s0_axi_bresp(s0_axi_bresp),
        .s0_axi_buser(s0_axi_buser),
        .s0_axi_bvalid(s0_axi_bvalid),
        .s0_axi_bready(s0_axi_bready),
        .m0_axi_awid(m0_axi_awid),
        .m0_axi_awaddr(m0_axi_awaddr),
        .m0_axi_awlen(m0_axi_awlen),
        .m0_axi_awsize(m0_axi_awsize),
        .m0_axi_awburst(m0_axi_awburst),
        .m0_axi_awlock(m0_axi_awlock),
        .m0_axi_awcache(m0_axi_awcache),
        .m0_axi_awprot(m0_axi_awprot),
        .m0_axi_awqos(m0_axi_awqos),
        .m0_axi_awregion(m0_axi_awregion),
        .m0_axi_awuser(m0_axi_awuser),
        .m0_axi_awvalid(m0_axi_awvalid),
        .m0_axi_awready(m0_axi_awready),
        .m0_axi_arid(m0_axi_arid),
        .m0_axi_araddr(m0_axi_araddr),
        .m0_axi_arlen(m0_axi_arlen),
        .m0_axi_arsize(m0_axi_arsize),
        .m0_axi_arburst(m0_axi_arburst),
        .m0_axi_arlock(m0_axi_arlock),
        .m0_axi_arcache(m0_axi_arcache),
        .m0_axi_arprot(m0_axi_arprot),
        .m0_axi_arqos(m0_axi_arqos),
        .m0_axi_arregion(m0_axi_arregion),
        .m0_axi_aruser(m0_axi_aruser),
        .m0_axi_arvalid(m0_axi_arvalid),
        .m0_axi_arready(m0_axi_arready),
        .m0_axi_wdata(m0_axi_wdata),
        .m0_axi_wstrb(m0_axi_wstrb),
        .m0_axi_wuser(m0_axi_wuser),
        .m0_axi_wlast(m0_axi_wlast),
        .m0_axi_wvalid(m0_axi_wvalid),
        .m0_axi_wready(m0_axi_wready),
        .m0_axi_rid(m0_axi_rid),
        .m0_axi_rdata(m0_axi_rdata),
        .m0_axi_rresp(m0_axi_rresp),
        .m0_axi_ruser(m0_axi_ruser),
        .m0_axi_rlast(m0_axi_rlast),
        .m0_axi_rvalid(m0_axi_rvalid),
        .m0_axi_rready(m0_axi_rready),
        .m0_axi_bid(m0_axi_bid),
        .m0_axi_bresp(m0_axi_bresp),
        .m0_axi_buser(m0_axi_buser),
        .m0_axi_bvalid(m0_axi_bvalid),
        .m0_axi_bready(m0_axi_bready),
`endif
`endif
`else
        .cxs_prcl0_rx_data           (cxs_prcl0_rx_data),
        .cxs_prcl0_rx_cntl           (cxs_prcl0_rx_cntl),
        .cxs_prcl0_rx_valid          (cxs_prcl0_rx_valid),
        .cxs_prcl0_rx_prcltype       (cxs_prcl0_rx_prcltype),
        .cxs_prcl0_rx_srcid          (cxs_prcl0_rx_srcid),
        .cxs_prcl0_rx_tgtid          (cxs_prcl0_rx_tgtid),
`ifdef CXSPRCL_LAST
        .cxs_prcl0_rx_last           (cxs_prcl0_rx_last),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl0_rx_chk_data       (cxs_prcl0_rx_chk_data),
        .cxs_prcl0_rx_chk_cntl       (cxs_prcl0_rx_chk_cntl),
        .cxs_prcl0_rx_chk_prcltype   (cxs_prcl0_rx_chk_prcltype),
        .cxs_prcl0_rx_chk_srcid      (cxs_prcl0_rx_chk_srcid),
        .cxs_prcl0_rx_chk_tgtid      (cxs_prcl0_rx_chk_tgtid),
        .cxs_prcl0_rx_chk_last       (cxs_prcl0_rx_chk_last),
        .cxs_prcl0_rx_chk_valid      (cxs_prcl0_rx_chk_valid),
`endif
        .cxs_prcl0_rx_crdrtn         (cxs_prcl0_rx_crdrtn),
        .cxs_prcl0_rx_crdgnt         (cxs_prcl0_rx_crdgnt),
        .cxs_prcl0_rx_activereq      (cxs_prcl0_rx_activereq),
        .cxs_prcl0_rx_activeack      (cxs_prcl0_rx_activeack),
        .cxs_prcl0_rx_deacthint      (cxs_prcl0_rx_deacthint),
`ifdef PRCL_TX_LANE_1
        .cxs_prcl1_rx_data           (cxs_prcl1_rx_data),
        .cxs_prcl1_rx_cntl           (cxs_prcl1_rx_cntl),
        .cxs_prcl1_rx_valid          (cxs_prcl1_rx_valid),
        .cxs_prcl1_rx_prcltype       (cxs_prcl1_rx_prcltype),
        .cxs_prcl1_rx_srcid          (cxs_prcl1_rx_srcid),
        .cxs_prcl1_rx_tgtid          (cxs_prcl1_rx_tgtid),
`ifdef CXSPRCL_LAST
        .cxs_prcl1_rx_last           (cxs_prcl1_rx_last),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl1_rx_chk_data       (cxs_prcl1_rx_chk_data),
        .cxs_prcl1_rx_chk_cntl       (cxs_prcl1_rx_chk_cntl),
        .cxs_prcl1_rx_chk_prcltype   (cxs_prcl1_rx_chk_prcltype),
        .cxs_prcl1_rx_chk_srcid      (cxs_prcl1_rx_chk_srcid),
        .cxs_prcl1_rx_chk_tgtid      (cxs_prcl1_rx_chk_tgtid),
        .cxs_prcl1_rx_chk_last       (cxs_prcl1_rx_chk_last),
        .cxs_prcl1_rx_chk_valid      (cxs_prcl1_rx_chk_valid),
`endif
        .cxs_prcl1_rx_crdrtn         (cxs_prcl1_rx_crdrtn),
        .cxs_prcl1_rx_crdgnt         (cxs_prcl1_rx_crdgnt),
        .cxs_prcl1_rx_activereq      (cxs_prcl1_rx_activereq),
        .cxs_prcl1_rx_activeack      (cxs_prcl1_rx_activeack),
        .cxs_prcl1_rx_deacthint      (cxs_prcl1_rx_deacthint),
`endif
`ifdef PRCL_TX_LANE_2
        .cxs_prcl2_rx_data           (cxs_prcl2_rx_data),
        .cxs_prcl2_rx_cntl           (cxs_prcl2_rx_cntl),
        .cxs_prcl2_rx_valid          (cxs_prcl2_rx_valid),
        .cxs_prcl2_rx_prcltype       (cxs_prcl2_rx_prcltype),
        .cxs_prcl2_rx_srcid          (cxs_prcl2_rx_srcid),
        .cxs_prcl2_rx_tgtid          (cxs_prcl2_rx_tgtid),
`ifdef CXSPRCL_LAST
        .cxs_prcl2_rx_last           (cxs_prcl2_rx_last),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl2_rx_chk_data       (cxs_prcl2_rx_chk_data),
        .cxs_prcl2_rx_chk_cntl       (cxs_prcl2_rx_chk_cntl),
        .cxs_prcl2_rx_chk_prcltype   (cxs_prcl2_rx_chk_prcltype),
        .cxs_prcl2_rx_chk_srcid      (cxs_prcl2_rx_chk_srcid),
        .cxs_prcl2_rx_chk_tgtid      (cxs_prcl2_rx_chk_tgtid),
        .cxs_prcl2_rx_chk_last       (cxs_prcl2_rx_chk_last),
        .cxs_prcl2_rx_chk_valid      (cxs_prcl2_rx_chk_valid),
`endif
        .cxs_prcl2_rx_crdrtn         (cxs_prcl2_rx_crdrtn),
        .cxs_prcl2_rx_crdgnt         (cxs_prcl2_rx_crdgnt),
        .cxs_prcl2_rx_activereq      (cxs_prcl2_rx_activereq),
        .cxs_prcl2_rx_activeack      (cxs_prcl2_rx_activeack),
        .cxs_prcl2_rx_deacthint      (cxs_prcl2_rx_deacthint),
`endif
`ifdef PRCL_TX_LANE_3
        .cxs_prcl3_rx_data           (cxs_prcl3_rx_data),
        .cxs_prcl3_rx_cntl           (cxs_prcl3_rx_cntl),
        .cxs_prcl3_rx_valid          (cxs_prcl3_rx_valid),
        .cxs_prcl3_rx_prcltype       (cxs_prcl3_rx_prcltype),
        .cxs_prcl3_rx_srcid          (cxs_prcl3_rx_srcid),
        .cxs_prcl3_rx_tgtid          (cxs_prcl3_rx_tgtid),
`ifdef CXSPRCL_LAST
        .cxs_prcl3_rx_last           (cxs_prcl3_rx_last),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl3_rx_chk_data       (cxs_prcl3_rx_chk_data),
        .cxs_prcl3_rx_chk_cntl       (cxs_prcl3_rx_chk_cntl),
        .cxs_prcl3_rx_chk_prcltype   (cxs_prcl3_rx_chk_prcltype),
        .cxs_prcl3_rx_chk_srcid      (cxs_prcl3_rx_chk_srcid),
        .cxs_prcl3_rx_chk_tgtid      (cxs_prcl3_rx_chk_tgtid),
        .cxs_prcl3_rx_chk_last       (cxs_prcl3_rx_chk_last),
        .cxs_prcl3_rx_chk_valid      (cxs_prcl3_rx_chk_valid),
`endif
        .cxs_prcl3_rx_crdrtn         (cxs_prcl3_rx_crdrtn),
        .cxs_prcl3_rx_crdgnt         (cxs_prcl3_rx_crdgnt),
        .cxs_prcl3_rx_activereq      (cxs_prcl3_rx_activereq),
        .cxs_prcl3_rx_activeack      (cxs_prcl3_rx_activeack),
        .cxs_prcl3_rx_deacthint      (cxs_prcl3_rx_deacthint),
`endif
        .cxs_prcl0_tx_data           (cxs_prcl0_tx_data),
        .cxs_prcl0_tx_cntl           (cxs_prcl0_tx_cntl),
        .cxs_prcl0_tx_valid          (cxs_prcl0_tx_valid),
        .cxs_prcl0_tx_prcltype       (cxs_prcl0_tx_prcltype),
        .cxs_prcl0_tx_srcid          (cxs_prcl0_tx_srcid),
        .cxs_prcl0_tx_tgtid          (cxs_prcl0_tx_tgtid),
`ifdef CXSPRCL_LAST
        .cxs_prcl0_tx_last           (cxs_prcl0_tx_last),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl0_tx_chk_data       (cxs_prcl0_tx_chk_data),
        .cxs_prcl0_tx_chk_cntl       (cxs_prcl0_tx_chk_cntl),
        .cxs_prcl0_tx_chk_prcltype   (cxs_prcl0_tx_chk_prcltype),
        .cxs_prcl0_tx_chk_srcid      (cxs_prcl0_tx_chk_srcid),
        .cxs_prcl0_tx_chk_tgtid      (cxs_prcl0_tx_chk_tgtid),
        .cxs_prcl0_tx_chk_last       (cxs_prcl0_tx_chk_last),
        .cxs_prcl0_tx_chk_valid      (cxs_prcl0_tx_chk_valid),
`endif
        .cxs_prcl0_tx_crdrtn         (cxs_prcl0_tx_crdrtn),
        .cxs_prcl0_tx_crdgnt         (cxs_prcl0_tx_crdgnt),
        .cxs_prcl0_tx_activereq      (cxs_prcl0_tx_activereq),
        .cxs_prcl0_tx_activeack      (cxs_prcl0_tx_activeack),
        .cxs_prcl0_tx_deacthint      (cxs_prcl0_tx_deacthint),
`ifdef PRCL_RX_LANE_1
        .cxs_prcl1_tx_data           (cxs_prcl1_tx_data),
        .cxs_prcl1_tx_cntl           (cxs_prcl1_tx_cntl),
        .cxs_prcl1_tx_valid          (cxs_prcl1_tx_valid),
        .cxs_prcl1_tx_prcltype       (cxs_prcl1_tx_prcltype),
        .cxs_prcl1_tx_srcid          (cxs_prcl1_tx_srcid),
        .cxs_prcl1_tx_tgtid          (cxs_prcl1_tx_tgtid),
`ifdef CXSPRCL_LAST
        .cxs_prcl1_tx_last           (cxs_prcl1_tx_last),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl1_tx_chk_data       (cxs_prcl1_tx_chk_data),
        .cxs_prcl1_tx_chk_cntl       (cxs_prcl1_tx_chk_cntl),
        .cxs_prcl1_tx_chk_prcltype   (cxs_prcl1_tx_chk_prcltype),
        .cxs_prcl1_tx_chk_srcid      (cxs_prcl1_tx_chk_srcid),
        .cxs_prcl1_tx_chk_tgtid      (cxs_prcl1_tx_chk_tgtid),
        .cxs_prcl1_tx_chk_last       (cxs_prcl1_tx_chk_last),
        .cxs_prcl1_tx_chk_valid      (cxs_prcl1_tx_chk_valid),
`endif
        .cxs_prcl1_tx_crdrtn         (cxs_prcl1_tx_crdrtn),
        .cxs_prcl1_tx_crdgnt         (cxs_prcl1_tx_crdgnt),
        .cxs_prcl1_tx_activereq      (cxs_prcl1_tx_activereq),
        .cxs_prcl1_tx_activeack      (cxs_prcl1_tx_activeack),
        .cxs_prcl1_tx_deacthint      (cxs_prcl1_tx_deacthint),
`endif
`ifdef PRCL_RX_LANE_2
        .cxs_prcl2_tx_data           (cxs_prcl2_tx_data),
        .cxs_prcl2_tx_cntl           (cxs_prcl2_tx_cntl),
        .cxs_prcl2_tx_valid          (cxs_prcl2_tx_valid),
        .cxs_prcl2_tx_prcltype       (cxs_prcl2_tx_prcltype),
        .cxs_prcl2_tx_srcid          (cxs_prcl2_tx_srcid),
        .cxs_prcl2_tx_tgtid          (cxs_prcl2_tx_tgtid),
`ifdef CXSPRCL_LAST
        .cxs_prcl2_tx_last           (cxs_prcl2_tx_last),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl2_tx_chk_data       (cxs_prcl2_tx_chk_data),
        .cxs_prcl2_tx_chk_cntl       (cxs_prcl2_tx_chk_cntl),
        .cxs_prcl2_tx_chk_prcltype   (cxs_prcl2_tx_chk_prcltype),
        .cxs_prcl2_tx_chk_srcid      (cxs_prcl2_tx_chk_srcid),
        .cxs_prcl2_tx_chk_tgtid      (cxs_prcl2_tx_chk_tgtid),
        .cxs_prcl2_tx_chk_last       (cxs_prcl2_tx_chk_last),
        .cxs_prcl2_tx_chk_valid      (cxs_prcl2_tx_chk_valid),
`endif
        .cxs_prcl2_tx_crdrtn         (cxs_prcl2_tx_crdrtn),
        .cxs_prcl2_tx_crdgnt         (cxs_prcl2_tx_crdgnt),
        .cxs_prcl2_tx_activereq      (cxs_prcl2_tx_activereq),
        .cxs_prcl2_tx_activeack      (cxs_prcl2_tx_activeack),
        .cxs_prcl2_tx_deacthint      (cxs_prcl2_tx_deacthint),
`endif
`ifdef PRCL_RX_LANE_3
        .cxs_prcl3_tx_data           (cxs_prcl3_tx_data),
        .cxs_prcl3_tx_cntl           (cxs_prcl3_tx_cntl),
        .cxs_prcl3_tx_valid          (cxs_prcl3_tx_valid),
        .cxs_prcl3_tx_prcltype       (cxs_prcl3_tx_prcltype),
        .cxs_prcl3_tx_srcid          (cxs_prcl3_tx_srcid),
        .cxs_prcl3_tx_tgtid          (cxs_prcl3_tx_tgtid),
`ifdef CXSPRCL_LAST
        .cxs_prcl3_tx_last           (cxs_prcl3_tx_last),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl3_tx_chk_data       (cxs_prcl3_tx_chk_data),
        .cxs_prcl3_tx_chk_cntl       (cxs_prcl3_tx_chk_cntl),
        .cxs_prcl3_tx_chk_prcltype   (cxs_prcl3_tx_chk_prcltype),
        .cxs_prcl3_tx_chk_srcid      (cxs_prcl3_tx_chk_srcid),
        .cxs_prcl3_tx_chk_tgtid      (cxs_prcl3_tx_chk_tgtid),
        .cxs_prcl3_tx_chk_last       (cxs_prcl3_tx_chk_last),
        .cxs_prcl3_tx_chk_valid      (cxs_prcl3_tx_chk_valid),
`endif
        .cxs_prcl3_tx_crdrtn         (cxs_prcl3_tx_crdrtn),
        .cxs_prcl3_tx_crdgnt         (cxs_prcl3_tx_crdgnt),
        .cxs_prcl3_tx_activereq      (cxs_prcl3_tx_activereq),
        .cxs_prcl3_tx_activeack      (cxs_prcl3_tx_activeack),
        .cxs_prcl3_tx_deacthint      (cxs_prcl3_tx_deacthint),
`endif
`endif

        // CPIF 数据（TX：ctrl → logic_phy；RX：logic_phy 低 512 → ctrl）
        // 多 link 映射：logic_phy 物理 lane{N} TX ← hipi_ctrl_top link{N} lane0（PRCL_TX_LANE_{N}）；
        //   link0 lane0 恒在 → lane0；link1/2/3 lane0 随 PRCL_TX_LANE_1/2/3。
        .cpif_tx_link0_lane0_data_o    (ll_cpif_tx_lane0_data),
        .cpif_tx_link0_lane0_data_vld_o(ll_cpif_tx_lane0_data_vld),
        .cpif_tx_link0_ready_i         (lp_cpif_tx_ready),
        .cpif_rx_link0_ready_i         (1'b1),  // logic_phy RX valid-only 无反压，恒 ready
        .cpif_rx_link0_lane0_data_i    (ll_cpif_rx_lane0_data),
        .cpif_rx_link0_lane0_data_vld_i(lp_cpif_rx_lane0_data_vld),
        // link1/2/3 lane0 RX 数据（logic_phy 物理 lane1/2/3 → ctrl，随 PRCL_RX_LANE_1/2/3；
        //   4R1T 下使能，4T1R 下编译移除）
`ifdef PRCL_RX_LANE_1
        .cpif_rx_link1_lane0_data_i    (ll_cpif_rx_lane1_data),
        .cpif_rx_link1_lane0_data_vld_i(lp_cpif_rx_lane1_data_vld),
`endif
`ifdef PRCL_RX_LANE_2
        .cpif_rx_link2_lane0_data_i    (ll_cpif_rx_lane2_data),
        .cpif_rx_link2_lane0_data_vld_i(lp_cpif_rx_lane2_data_vld),
`endif
`ifdef PRCL_RX_LANE_3
        .cpif_rx_link3_lane0_data_i    (ll_cpif_rx_lane3_data),
        .cpif_rx_link3_lane0_data_vld_i(lp_cpif_rx_lane3_data_vld),
`endif
        // link1/2/3 lane0 TX 数据（logic_phy 物理 lane1/2/3 源，随 PRCL_TX_LANE_1/2/3）
`ifdef PRCL_TX_LANE_1
        .cpif_tx_link1_lane0_data_o    (ll_cpif_tx_lane1_data),
        .cpif_tx_link1_lane0_data_vld_o(ll_cpif_tx_lane1_data_vld),
`endif
`ifdef PRCL_TX_LANE_2
        .cpif_tx_link2_lane0_data_o    (ll_cpif_tx_lane2_data),
        .cpif_tx_link2_lane0_data_vld_o(ll_cpif_tx_lane2_data_vld),
`endif
`ifdef PRCL_TX_LANE_3
        .cpif_tx_link3_lane0_data_o    (ll_cpif_tx_lane3_data),
        .cpif_tx_link3_lane0_data_vld_o(ll_cpif_tx_lane3_data_vld),
`endif
        // link1/2/3 CPIF TX ready（PRCL_TX_LANE_1/2/3 下端口存在；接共享 lp_cpif_tx_ready）
`ifdef PRCL_TX_LANE_1
        .cpif_tx_link1_ready_i         (lp_cpif_tx_ready),
`endif
`ifdef PRCL_TX_LANE_2
        .cpif_tx_link2_ready_i         (lp_cpif_tx_ready),
`endif
`ifdef PRCL_TX_LANE_3
        .cpif_tx_link3_ready_i         (lp_cpif_tx_ready),
`endif
        // CPIF 控制（全 link 共享）
        .cpif_tx_mode_o           (ll_cpif_tx_mode),
        .cpif_tx_rst_n_o          (ll_cpif_tx_rst_n),
        .cpif_tx_lpc_o            (ll_cpif_tx_lpc),
        .cpif_rx_mode_o           (ll_cpif_rx_mode),
        .cpif_rx_rst_n_o          (ll_cpif_rx_rst_n),
        // UDI TX / RX data interface（per-lane，透传顶端口）
        .udi_tx_link0_lane0_data_i   (udi_tx_link0_lane0_data_i),
        .udi_tx_link0_lane0_data_vld_i(udi_tx_link0_lane0_data_vld_i),
`ifdef LL_LINK_LANE1
        .udi_tx_link0_lane1_data_i   (udi_tx_link0_lane1_data_i),
        .udi_tx_link0_lane1_data_vld_i(udi_tx_link0_lane1_data_vld_i),
`endif
`ifdef LL_LINK_LANE3
        .udi_tx_link0_lane2_data_i   (udi_tx_link0_lane2_data_i),
        .udi_tx_link0_lane2_data_vld_i(udi_tx_link0_lane2_data_vld_i),
        .udi_tx_link0_lane3_data_i   (udi_tx_link0_lane3_data_i),
        .udi_tx_link0_lane3_data_vld_i(udi_tx_link0_lane3_data_vld_i),
`endif
        .udi_tx_link0_ready_o        (udi_tx_link0_ready_o),
        .udi_rx_link0_lane0_data_o   (udi_rx_link0_lane0_data_o),
        .udi_rx_link0_lane0_data_vld_o(udi_rx_link0_lane0_data_vld_o),
`ifdef LL_LINK_LANE1
        .udi_rx_link0_lane1_data_o   (udi_rx_link0_lane1_data_o),
        .udi_rx_link0_lane1_data_vld_o(udi_rx_link0_lane1_data_vld_o),
`endif
`ifdef LL_LINK_LANE3
        .udi_rx_link0_lane2_data_o   (udi_rx_link0_lane2_data_o),
        .udi_rx_link0_lane2_data_vld_o(udi_rx_link0_lane2_data_vld_o),
        .udi_rx_link0_lane3_data_o   (udi_rx_link0_lane3_data_o),
        .udi_rx_link0_lane3_data_vld_o(udi_rx_link0_lane3_data_vld_o),
`endif
`ifdef PRCL_TX_LANE_1
        .udi_tx_link1_lane0_data_i   (udi_tx_link1_lane0_data_i),
        .udi_tx_link1_lane0_data_vld_i(udi_tx_link1_lane0_data_vld_i),
`ifdef LL_LINK_LANE1
        .udi_tx_link1_lane1_data_i   (udi_tx_link1_lane1_data_i),
        .udi_tx_link1_lane1_data_vld_i(udi_tx_link1_lane1_data_vld_i),
`endif
`ifdef LL_LINK_LANE3
        .udi_tx_link1_lane2_data_i   (udi_tx_link1_lane2_data_i),
        .udi_tx_link1_lane2_data_vld_i(udi_tx_link1_lane2_data_vld_i),
        .udi_tx_link1_lane3_data_i   (udi_tx_link1_lane3_data_i),
        .udi_tx_link1_lane3_data_vld_i(udi_tx_link1_lane3_data_vld_i),
`endif
        .udi_tx_link1_ready_o        (udi_tx_link1_ready_o),
`endif
`ifdef PRCL_TX_LANE_2
        .udi_tx_link2_lane0_data_i   (udi_tx_link2_lane0_data_i),
        .udi_tx_link2_lane0_data_vld_i(udi_tx_link2_lane0_data_vld_i),
`ifdef LL_LINK_LANE1
        .udi_tx_link2_lane1_data_i   (udi_tx_link2_lane1_data_i),
        .udi_tx_link2_lane1_data_vld_i(udi_tx_link2_lane1_data_vld_i),
`endif
`ifdef LL_LINK_LANE3
        .udi_tx_link2_lane2_data_i   (udi_tx_link2_lane2_data_i),
        .udi_tx_link2_lane2_data_vld_i(udi_tx_link2_lane2_data_vld_i),
        .udi_tx_link2_lane3_data_i   (udi_tx_link2_lane3_data_i),
        .udi_tx_link2_lane3_data_vld_i(udi_tx_link2_lane3_data_vld_i),
`endif
        .udi_tx_link2_ready_o        (udi_tx_link2_ready_o),
`endif
`ifdef PRCL_TX_LANE_3
        .udi_tx_link3_lane0_data_i   (udi_tx_link3_lane0_data_i),
        .udi_tx_link3_lane0_data_vld_i(udi_tx_link3_lane0_data_vld_i),
`ifdef LL_LINK_LANE1
        .udi_tx_link3_lane1_data_i   (udi_tx_link3_lane1_data_i),
        .udi_tx_link3_lane1_data_vld_i(udi_tx_link3_lane1_data_vld_i),
`endif
`ifdef LL_LINK_LANE3
        .udi_tx_link3_lane2_data_i   (udi_tx_link3_lane2_data_i),
        .udi_tx_link3_lane2_data_vld_i(udi_tx_link3_lane2_data_vld_i),
        .udi_tx_link3_lane3_data_i   (udi_tx_link3_lane3_data_i),
        .udi_tx_link3_lane3_data_vld_i(udi_tx_link3_lane3_data_vld_i),
`endif
        .udi_tx_link3_ready_o        (udi_tx_link3_ready_o),
`endif
`ifdef PRCL_RX_LANE_1
        .udi_rx_link1_lane0_data_o   (udi_rx_link1_lane0_data_o),
        .udi_rx_link1_lane0_data_vld_o(udi_rx_link1_lane0_data_vld_o),
`ifdef LL_LINK_LANE1
        .udi_rx_link1_lane1_data_o   (udi_rx_link1_lane1_data_o),
        .udi_rx_link1_lane1_data_vld_o(udi_rx_link1_lane1_data_vld_o),
`endif
`ifdef LL_LINK_LANE3
        .udi_rx_link1_lane2_data_o   (udi_rx_link1_lane2_data_o),
        .udi_rx_link1_lane2_data_vld_o(udi_rx_link1_lane2_data_vld_o),
        .udi_rx_link1_lane3_data_o   (udi_rx_link1_lane3_data_o),
        .udi_rx_link1_lane3_data_vld_o(udi_rx_link1_lane3_data_vld_o),
`endif
`endif
`ifdef PRCL_RX_LANE_2
        .udi_rx_link2_lane0_data_o   (udi_rx_link2_lane0_data_o),
        .udi_rx_link2_lane0_data_vld_o(udi_rx_link2_lane0_data_vld_o),
`ifdef LL_LINK_LANE1
        .udi_rx_link2_lane1_data_o   (udi_rx_link2_lane1_data_o),
        .udi_rx_link2_lane1_data_vld_o(udi_rx_link2_lane1_data_vld_o),
`endif
`ifdef LL_LINK_LANE3
        .udi_rx_link2_lane2_data_o   (udi_rx_link2_lane2_data_o),
        .udi_rx_link2_lane2_data_vld_o(udi_rx_link2_lane2_data_vld_o),
        .udi_rx_link2_lane3_data_o   (udi_rx_link2_lane3_data_o),
        .udi_rx_link2_lane3_data_vld_o(udi_rx_link2_lane3_data_vld_o),
`endif
`endif
`ifdef PRCL_RX_LANE_3
        .udi_rx_link3_lane0_data_o   (udi_rx_link3_lane0_data_o),
        .udi_rx_link3_lane0_data_vld_o(udi_rx_link3_lane0_data_vld_o),
`ifdef LL_LINK_LANE1
        .udi_rx_link3_lane1_data_o   (udi_rx_link3_lane1_data_o),
        .udi_rx_link3_lane1_data_vld_o(udi_rx_link3_lane1_data_vld_o),
`endif
`ifdef LL_LINK_LANE3
        .udi_rx_link3_lane2_data_o   (udi_rx_link3_lane2_data_o),
        .udi_rx_link3_lane2_data_vld_o(udi_rx_link3_lane2_data_vld_o),
        .udi_rx_link3_lane3_data_o   (udi_rx_link3_lane3_data_o),
        .udi_rx_link3_lane3_data_vld_o(udi_rx_link3_lane3_data_vld_o),
`endif
`endif
        // Event TX / RX（全 lane 共享）
        .cpif_tx_event_enable_o   (ev_tx_enable),
        .cpif_tx_event_req_o      (ev_tx_req),
        .cpif_tx_event_cmd_o      (ev_tx_cmd),
        .cpif_tx_event_ack_i      (ev_tx_ack),
        .cpif_tx_event_status_i   (ev_tx_status),
        .cpif_rx_event_enable_o   (ev_rx_enable),
        .cpif_rx_event_req_i      (ev_rx_req),
        .cpif_rx_event_cmd_i      (ev_rx_cmd),
        .cpif_rx_event_ack_o      (ev_rx_ack),
        .cpif_rx_event_status_o   (ev_rx_status),
        // DFx
        .dfx_stat_o               (dfx_stat_o)
    );

    // --------------------------------------------------------------------------
    // u_logic_phy
    // --------------------------------------------------------------------------
    logic_phy u_logic_phy (
        // 时钟与复位（apb_pclk 与 mcu_clk 均由 pclk 直供，logic_phy mcu/apb 同域；strap tie 0，决策④）
        .mb_clk                     (mb_clk),
        .phy_rx_clk0                (phy_rx_clk0),
`ifdef D2D_PHY_RX_LANE1_EN
        .phy_rx_clk1                (phy_rx_clk1),
`endif
`ifdef D2D_PHY_RX_LANE2_EN
        .phy_rx_clk2                (phy_rx_clk2),
`endif
`ifdef D2D_PHY_RX_LANE3_EN
        .phy_rx_clk3                (phy_rx_clk3),
`endif
        .sb_tx_clk_in               (sb_tx_clk_in),
        .sb_tx_clk                  (sb_tx_clk),
        .sb_rx_clk                  (sb_rx_clk),
        .mcu_clk                    (pclk),
        .apb_pclk                   (pclk),
        .mb_rst_n                   (mb_rst_n),
        .sb_rst_n                   (sb_rst_n),
        .mcu_rst_n                  (mcu_rst_n),
        .presetn                    (presetn),
        // CPIF 发送数据（link → logic；lp_cpif_tx_lane* = 零扩展后全宽，见组合逻辑段）
        // 逻辑层物理 lane 由 PHY_LANE_* 控制（每 lane 恒 TX+RX），不再用 PRCL_TX_LANE_*/PRCL_RX_LANE_*
        .cpif_tx_lane0_data         (lp_cpif_tx_lane0_data),
        .cpif_tx_lane0_data_vld     (ll_cpif_tx_lane0_data_vld),
`ifdef D2D_PHY_TX_LANE1_EN
        .cpif_tx_lane1_data         (lp_cpif_tx_lane1_data),
        .cpif_tx_lane1_data_vld     (ll_cpif_tx_lane1_data_vld),
`endif
`ifdef D2D_PHY_TX_LANE2_EN
        .cpif_tx_lane2_data         (lp_cpif_tx_lane2_data),
        .cpif_tx_lane2_data_vld     (ll_cpif_tx_lane2_data_vld),
`endif
`ifdef D2D_PHY_TX_LANE3_EN
        .cpif_tx_lane3_data         (lp_cpif_tx_lane3_data),
        .cpif_tx_lane3_data_vld     (ll_cpif_tx_lane3_data_vld),
`endif
        // CPIF 接收数据（logic → link；ctrl 侧接截断后的 ll_cpif_rx_lane*）
        .cpif_rx_lane0_data         (lp_cpif_rx_lane0_data),
        .cpif_rx_lane0_data_vld     (lp_cpif_rx_lane0_data_vld),
`ifdef D2D_PHY_RX_LANE1_EN
        .cpif_rx_lane1_data         (lp_cpif_rx_lane1_data),
        .cpif_rx_lane1_data_vld     (lp_cpif_rx_lane1_data_vld),
`endif
`ifdef D2D_PHY_RX_LANE2_EN
        .cpif_rx_lane2_data         (lp_cpif_rx_lane2_data),
        .cpif_rx_lane2_data_vld     (lp_cpif_rx_lane2_data_vld),
`endif
`ifdef D2D_PHY_RX_LANE3_EN
        .cpif_rx_lane3_data         (lp_cpif_rx_lane3_data),
        .cpif_rx_lane3_data_vld     (lp_cpif_rx_lane3_data_vld),
`endif
        // CPIF 控制（全 lane 共享）
        .cpif_tx_mode               (ll_cpif_tx_mode),
        .cpif_tx_rst_n              (ll_cpif_tx_rst_n),
        .cpif_tx_lpc                (ll_cpif_tx_lpc),
        .cpif_tx_ready              (lp_cpif_tx_ready),
        .cpif_rx_mode               (ll_cpif_rx_mode),
        .cpif_rx_rst_n              (ll_cpif_rx_rst_n),
        .cpif_rx_ready              (lp_cpif_rx_ready),
        // CPIF 事件（全 lane 共享）
        .cpif_tx_event_enable       (ev_tx_enable),
        .cpif_tx_event_req          (ev_tx_req),
        .cpif_tx_event_cmd          (ev_tx_cmd),
        .cpif_tx_event_ack          (ev_tx_ack),
        .cpif_tx_event_status       (ev_tx_status),
        .cpif_rx_event_enable       (ev_rx_enable),
        .cpif_rx_event_req          (ev_rx_req),
        .cpif_rx_event_cmd          (ev_rx_cmd),
        .cpif_rx_event_ack          (ev_rx_ack),
        .cpif_rx_event_status       (ev_rx_status),
        // 主带数据 / 训练校准（→ analog PHY，顶端口直透）
        .tx_lane0_data0              (tx_lane0_data0),
        .tx_lane0_data1              (tx_lane0_data1),
        .tx_lane0_data2              (tx_lane0_data2),
        .tx_lane0_data3              (tx_lane0_data3),
        .tx_lane0_data4              (tx_lane0_data4),
        .tx_lane0_data5              (tx_lane0_data5),
        .tx_lane0_data6              (tx_lane0_data6),
        .tx_lane0_data7              (tx_lane0_data7),
        .tx_lane0_data8              (tx_lane0_data8),
        .tx_lane0_data9              (tx_lane0_data9),
        .tx_lane0_data10             (tx_lane0_data10),
        .tx_lane0_data11             (tx_lane0_data11),
        .tx_lane0_data12             (tx_lane0_data12),
        .tx_lane0_data13             (tx_lane0_data13),
        .tx_lane0_data14             (tx_lane0_data14),
        .tx_lane0_data15             (tx_lane0_data15),
        .tx_lane0_data16             (tx_lane0_data16),
        .rx_lane0_data0              (rx_lane0_data0),
        .rx_lane0_data1              (rx_lane0_data1),
        .rx_lane0_data2              (rx_lane0_data2),
        .rx_lane0_data3              (rx_lane0_data3),
        .rx_lane0_data4              (rx_lane0_data4),
        .rx_lane0_data5              (rx_lane0_data5),
        .rx_lane0_data6              (rx_lane0_data6),
        .rx_lane0_data7              (rx_lane0_data7),
        .rx_lane0_data8              (rx_lane0_data8),
        .rx_lane0_data9              (rx_lane0_data9),
        .rx_lane0_data10             (rx_lane0_data10),
        .rx_lane0_data11             (rx_lane0_data11),
        .rx_lane0_data12             (rx_lane0_data12),
        .rx_lane0_data13             (rx_lane0_data13),
        .rx_lane0_data14             (rx_lane0_data14),
        .rx_lane0_data15             (rx_lane0_data15),
        .rx_lane0_data16             (rx_lane0_data16),
        .lane0_RX_OFFSET_CALIBRATION_ON                 (lane0_RX_OFFSET_CALIBRATION_ON),
        .lane0_OFFSET_CTRL_CLK                          (lane0_OFFSET_CTRL_CLK),
        .lane0_OFFSET_RESET_N                           (lane0_OFFSET_RESET_N),
        .lane0_RX0_OFFSET_CTRL_WRITE                    (lane0_RX0_OFFSET_CTRL_WRITE),
        .lane0_RX0_OFFSET_CTRL_ADDR                     (lane0_RX0_OFFSET_CTRL_ADDR),
        .lane0_RX0_OFFSET_CTRL_DATA                     (lane0_RX0_OFFSET_CTRL_DATA),
        .lane0_RX1_OFFSET_CTRL_WRITE                    (lane0_RX1_OFFSET_CTRL_WRITE),
        .lane0_RX1_OFFSET_CTRL_ADDR                     (lane0_RX1_OFFSET_CTRL_ADDR),
        .lane0_RX1_OFFSET_CTRL_DATA                     (lane0_RX1_OFFSET_CTRL_DATA),
        .lane0_RX2_OFFSET_CTRL_WRITE                    (lane0_RX2_OFFSET_CTRL_WRITE),
        .lane0_RX2_OFFSET_CTRL_ADDR                     (lane0_RX2_OFFSET_CTRL_ADDR),
        .lane0_RX2_OFFSET_CTRL_DATA                     (lane0_RX2_OFFSET_CTRL_DATA),
        .lane0_RX3_OFFSET_CTRL_WRITE                    (lane0_RX3_OFFSET_CTRL_WRITE),
        .lane0_RX3_OFFSET_CTRL_ADDR                     (lane0_RX3_OFFSET_CTRL_ADDR),
        .lane0_RX3_OFFSET_CTRL_DATA                     (lane0_RX3_OFFSET_CTRL_DATA),
        .lane0_RX4_OFFSET_CTRL_WRITE                    (lane0_RX4_OFFSET_CTRL_WRITE),
        .lane0_RX4_OFFSET_CTRL_ADDR                     (lane0_RX4_OFFSET_CTRL_ADDR),
        .lane0_RX4_OFFSET_CTRL_DATA                     (lane0_RX4_OFFSET_CTRL_DATA),
        .lane0_RX5_OFFSET_CTRL_WRITE                    (lane0_RX5_OFFSET_CTRL_WRITE),
        .lane0_RX5_OFFSET_CTRL_ADDR                     (lane0_RX5_OFFSET_CTRL_ADDR),
        .lane0_RX5_OFFSET_CTRL_DATA                     (lane0_RX5_OFFSET_CTRL_DATA),
        .lane0_RX6_OFFSET_CTRL_WRITE                    (lane0_RX6_OFFSET_CTRL_WRITE),
        .lane0_RX6_OFFSET_CTRL_ADDR                     (lane0_RX6_OFFSET_CTRL_ADDR),
        .lane0_RX6_OFFSET_CTRL_DATA                     (lane0_RX6_OFFSET_CTRL_DATA),
        .lane0_RX7_OFFSET_CTRL_WRITE                    (lane0_RX7_OFFSET_CTRL_WRITE),
        .lane0_RX7_OFFSET_CTRL_ADDR                     (lane0_RX7_OFFSET_CTRL_ADDR),
        .lane0_RX7_OFFSET_CTRL_DATA                     (lane0_RX7_OFFSET_CTRL_DATA),
        .lane0_RX8_OFFSET_CTRL_WRITE                    (lane0_RX8_OFFSET_CTRL_WRITE),
        .lane0_RX8_OFFSET_CTRL_ADDR                     (lane0_RX8_OFFSET_CTRL_ADDR),
        .lane0_RX8_OFFSET_CTRL_DATA                     (lane0_RX8_OFFSET_CTRL_DATA),
        .lane0_RX9_OFFSET_CTRL_WRITE                    (lane0_RX9_OFFSET_CTRL_WRITE),
        .lane0_RX9_OFFSET_CTRL_ADDR                     (lane0_RX9_OFFSET_CTRL_ADDR),
        .lane0_RX9_OFFSET_CTRL_DATA                     (lane0_RX9_OFFSET_CTRL_DATA),
        .lane0_RX10_OFFSET_CTRL_WRITE                   (lane0_RX10_OFFSET_CTRL_WRITE),
        .lane0_RX10_OFFSET_CTRL_ADDR                    (lane0_RX10_OFFSET_CTRL_ADDR),
        .lane0_RX10_OFFSET_CTRL_DATA                    (lane0_RX10_OFFSET_CTRL_DATA),
        .lane0_RX11_OFFSET_CTRL_WRITE                   (lane0_RX11_OFFSET_CTRL_WRITE),
        .lane0_RX11_OFFSET_CTRL_ADDR                    (lane0_RX11_OFFSET_CTRL_ADDR),
        .lane0_RX11_OFFSET_CTRL_DATA                    (lane0_RX11_OFFSET_CTRL_DATA),
        .lane0_RX12_OFFSET_CTRL_WRITE                   (lane0_RX12_OFFSET_CTRL_WRITE),
        .lane0_RX12_OFFSET_CTRL_ADDR                    (lane0_RX12_OFFSET_CTRL_ADDR),
        .lane0_RX12_OFFSET_CTRL_DATA                    (lane0_RX12_OFFSET_CTRL_DATA),
        .lane0_RX13_OFFSET_CTRL_WRITE                   (lane0_RX13_OFFSET_CTRL_WRITE),
        .lane0_RX13_OFFSET_CTRL_ADDR                    (lane0_RX13_OFFSET_CTRL_ADDR),
        .lane0_RX13_OFFSET_CTRL_DATA                    (lane0_RX13_OFFSET_CTRL_DATA),
        .lane0_RX14_OFFSET_CTRL_WRITE                   (lane0_RX14_OFFSET_CTRL_WRITE),
        .lane0_RX14_OFFSET_CTRL_ADDR                    (lane0_RX14_OFFSET_CTRL_ADDR),
        .lane0_RX14_OFFSET_CTRL_DATA                    (lane0_RX14_OFFSET_CTRL_DATA),
        .lane0_RX15_OFFSET_CTRL_WRITE                   (lane0_RX15_OFFSET_CTRL_WRITE),
        .lane0_RX15_OFFSET_CTRL_ADDR                    (lane0_RX15_OFFSET_CTRL_ADDR),
        .lane0_RX15_OFFSET_CTRL_DATA                    (lane0_RX15_OFFSET_CTRL_DATA),
        .lane0_RX16_OFFSET_CTRL_WRITE                   (lane0_RX16_OFFSET_CTRL_WRITE),
        .lane0_RX16_OFFSET_CTRL_ADDR                    (lane0_RX16_OFFSET_CTRL_ADDR),
        .lane0_RX16_OFFSET_CTRL_DATA                    (lane0_RX16_OFFSET_CTRL_DATA),
        .lane0_RX_VREF_CTRL_MAIN                        (lane0_RX_VREF_CTRL_MAIN),
        .lane0_RX_VREF_CTRL_ERR_EDGE                    (lane0_RX_VREF_CTRL_ERR_EDGE),
        .lane0_PH_I_GRAY_PI                             (lane0_PH_I_GRAY_PI),
        .lane0_PH_Q_GRAY_PI                             (lane0_PH_Q_GRAY_PI),
        .lane0_PH_045_GRAY_PI                           (lane0_PH_045_GRAY_PI),
        .lane0_RX0_DCDL_CTRL                            (lane0_RX0_DCDL_CTRL),
        .lane0_RX1_DCDL_CTRL                            (lane0_RX1_DCDL_CTRL),
        .lane0_RX2_DCDL_CTRL                            (lane0_RX2_DCDL_CTRL),
        .lane0_RX3_DCDL_CTRL                            (lane0_RX3_DCDL_CTRL),
        .lane0_RX4_DCDL_CTRL                            (lane0_RX4_DCDL_CTRL),
        .lane0_RX5_DCDL_CTRL                            (lane0_RX5_DCDL_CTRL),
        .lane0_RX6_DCDL_CTRL                            (lane0_RX6_DCDL_CTRL),
        .lane0_RX7_DCDL_CTRL                            (lane0_RX7_DCDL_CTRL),
        .lane0_RX8_DCDL_CTRL                            (lane0_RX8_DCDL_CTRL),
        .lane0_RX9_DCDL_CTRL                            (lane0_RX9_DCDL_CTRL),
        .lane0_RX10_DCDL_CTRL                           (lane0_RX10_DCDL_CTRL),
        .lane0_RX11_DCDL_CTRL                           (lane0_RX11_DCDL_CTRL),
        .lane0_RX12_DCDL_CTRL                           (lane0_RX12_DCDL_CTRL),
        .lane0_RX13_DCDL_CTRL                           (lane0_RX13_DCDL_CTRL),
        .lane0_RX14_DCDL_CTRL                           (lane0_RX14_DCDL_CTRL),
        .lane0_RX15_DCDL_CTRL                           (lane0_RX15_DCDL_CTRL),
        .lane0_RX16_DCDL_CTRL                           (lane0_RX16_DCDL_CTRL),
        .sb_txr                                         (sb_txr),  // P0-B
        .sb_rxr                                         (sb_rxr),  // P0-B
        .lane0_TRX_REG1_WRITE_IN                        (lane0_TRX_REG1_WRITE_IN),  // P0-B
        .lane0_TRX_REG2_WRITE_IN                        (lane0_TRX_REG2_WRITE_IN),  // P0-B
        .lane0_TRX_TEST_REG1_WRITE_IN                   (lane0_TRX_TEST_REG1_WRITE_IN),  // P0-B
        .lane0_TRX_TEST_REG2_WRITE_IN                   (lane0_TRX_TEST_REG2_WRITE_IN),  // P0-B
        .lane0_RX_CLK_ENG_REG1_WRITE_IN                 (lane0_RX_CLK_ENG_REG1_WRITE_IN),  // P0-B
        .lane0_RX_CLK_ENG_REG2_WRITE_IN                 (lane0_RX_CLK_ENG_REG2_WRITE_IN),  // P0-B
        .lane0_RX_CLK_ENG_REG3_WRITE_IN                 (lane0_RX_CLK_ENG_REG3_WRITE_IN),  // P0-B
        .lane0_RX_CLK_ENG_REG4_WRITE_IN                 (lane0_RX_CLK_ENG_REG4_WRITE_IN),  // P0-B
        .lane0_RX_CLK_REG1_WRITE_IN                     (lane0_RX_CLK_REG1_WRITE_IN),  // P0-B
        .lane0_RX_CLK_REG2_WRITE_IN                     (lane0_RX_CLK_REG2_WRITE_IN),  // P0-B
        .lane0_RX_CLK_REG3_WRITE_IN                     (lane0_RX_CLK_REG3_WRITE_IN),  // P0-B
        .lane0_RX_CLK_REG4_WRITE_IN                     (lane0_RX_CLK_REG4_WRITE_IN),  // P0-B
        .lane0_RX_DATA_REG1_WRITE_IN                    (lane0_RX_DATA_REG1_WRITE_IN),  // P0-B
        .lane0_RX_DATA_REG2_WRITE_IN                    (lane0_RX_DATA_REG2_WRITE_IN),  // P0-B
        .lane0_RX_DATA_REG3_WRITE_IN                    (lane0_RX_DATA_REG3_WRITE_IN),  // P0-B
        .lane0_RX_LDO_REG1_WRITE_IN                     (lane0_RX_LDO_REG1_WRITE_IN),  // P0-B
        .lane0_RX_LDO_REG2_WRITE_IN                     (lane0_RX_LDO_REG2_WRITE_IN),  // P0-B
        .lane0_TX_CLK_ENG_REG1_WRITE_IN                 (lane0_TX_CLK_ENG_REG1_WRITE_IN),  // P0-B
        .lane0_TX_CLK_ENG_REG2_WRITE_IN                 (lane0_TX_CLK_ENG_REG2_WRITE_IN),  // P0-B
        .lane0_TX_CLK_ENG_REG3_WRITE_IN                 (lane0_TX_CLK_ENG_REG3_WRITE_IN),  // P0-B
        .lane0_TX_CLK_ENG_REG4_WRITE_IN                 (lane0_TX_CLK_ENG_REG4_WRITE_IN),  // P0-B
        .lane0_TX_CLK_REG1_WRITE_IN                     (lane0_TX_CLK_REG1_WRITE_IN),  // P0-B
        .lane0_TX_CLK_REG2_WRITE_IN                     (lane0_TX_CLK_REG2_WRITE_IN),  // P0-B
        .lane0_TX_CLK_REG3_WRITE_IN                     (lane0_TX_CLK_REG3_WRITE_IN),  // P0-B
        .lane0_TX_CLK_REG4_WRITE_IN                     (lane0_TX_CLK_REG4_WRITE_IN),  // P0-B
        .lane0_TX_CLK_REG5_WRITE_IN                     (lane0_TX_CLK_REG5_WRITE_IN),  // P0-B
        .lane0_TX_DATA_REG1_WRITE_IN                    (lane0_TX_DATA_REG1_WRITE_IN),  // P0-B
        .lane0_TX_DATA_REG2_WRITE_IN                    (lane0_TX_DATA_REG2_WRITE_IN),  // P0-B
        .lane0_TX_DATA_REG3_WRITE_IN                    (lane0_TX_DATA_REG3_WRITE_IN),  // P0-B
        .lane0_TX_DATA_REG4_WRITE_IN                    (lane0_TX_DATA_REG4_WRITE_IN),  // P0-B
        .lane0_TX_DATA_REG5_WRITE_IN                    (lane0_TX_DATA_REG5_WRITE_IN),  // P0-B
        .lane0_TX_DATA_REG6_WRITE_IN                    (lane0_TX_DATA_REG6_WRITE_IN),  // P0-B
        .lane0_TX_DATA_REG7_WRITE_IN                    (lane0_TX_DATA_REG7_WRITE_IN),  // P0-B
        .lane0_TX_LDO_REG1_WRITE_IN                     (lane0_TX_LDO_REG1_WRITE_IN),  // P0-B
        .lane0_TX_LDO_REG2_WRITE_IN                     (lane0_TX_LDO_REG2_WRITE_IN),  // P0-B
        .lane1_TRX_REG1_WRITE_IN                        (lane1_TRX_REG1_WRITE_IN),  // P0-B
        .lane1_TRX_REG2_WRITE_IN                        (lane1_TRX_REG2_WRITE_IN),  // P0-B
        .lane1_TRX_TEST_REG1_WRITE_IN                   (lane1_TRX_TEST_REG1_WRITE_IN),  // P0-B
        .lane1_TRX_TEST_REG2_WRITE_IN                   (lane1_TRX_TEST_REG2_WRITE_IN),  // P0-B
`ifdef D2D_PHY_RX_LANE1_EN
        .lane1_RX_CLK_ENG_REG1_WRITE_IN                 (lane1_RX_CLK_ENG_REG1_WRITE_IN),  // P0-B
        .lane1_RX_CLK_ENG_REG2_WRITE_IN                 (lane1_RX_CLK_ENG_REG2_WRITE_IN),  // P0-B
        .lane1_RX_CLK_ENG_REG3_WRITE_IN                 (lane1_RX_CLK_ENG_REG3_WRITE_IN),  // P0-B
        .lane1_RX_CLK_ENG_REG4_WRITE_IN                 (lane1_RX_CLK_ENG_REG4_WRITE_IN),  // P0-B
        .lane1_RX_CLK_REG1_WRITE_IN                     (lane1_RX_CLK_REG1_WRITE_IN),  // P0-B
        .lane1_RX_CLK_REG2_WRITE_IN                     (lane1_RX_CLK_REG2_WRITE_IN),  // P0-B
        .lane1_RX_CLK_REG3_WRITE_IN                     (lane1_RX_CLK_REG3_WRITE_IN),  // P0-B
        .lane1_RX_CLK_REG4_WRITE_IN                     (lane1_RX_CLK_REG4_WRITE_IN),  // P0-B
        .lane1_RX_DATA_REG1_WRITE_IN                    (lane1_RX_DATA_REG1_WRITE_IN),  // P0-B
        .lane1_RX_DATA_REG2_WRITE_IN                    (lane1_RX_DATA_REG2_WRITE_IN),  // P0-B
        .lane1_RX_DATA_REG3_WRITE_IN                    (lane1_RX_DATA_REG3_WRITE_IN),  // P0-B
        .lane1_RX_LDO_REG1_WRITE_IN                     (lane1_RX_LDO_REG1_WRITE_IN),  // P0-B
        .lane1_RX_LDO_REG2_WRITE_IN                     (lane1_RX_LDO_REG2_WRITE_IN),  // P0-B
`endif
`ifdef D2D_PHY_TX_LANE1_EN
        .lane1_TX_CLK_ENG_REG1_WRITE_IN                 (lane1_TX_CLK_ENG_REG1_WRITE_IN),  // P0-B
        .lane1_TX_CLK_ENG_REG2_WRITE_IN                 (lane1_TX_CLK_ENG_REG2_WRITE_IN),  // P0-B
        .lane1_TX_CLK_ENG_REG3_WRITE_IN                 (lane1_TX_CLK_ENG_REG3_WRITE_IN),  // P0-B
        .lane1_TX_CLK_ENG_REG4_WRITE_IN                 (lane1_TX_CLK_ENG_REG4_WRITE_IN),  // P0-B
        .lane1_TX_CLK_REG1_WRITE_IN                     (lane1_TX_CLK_REG1_WRITE_IN),  // P0-B
        .lane1_TX_CLK_REG2_WRITE_IN                     (lane1_TX_CLK_REG2_WRITE_IN),  // P0-B
        .lane1_TX_CLK_REG3_WRITE_IN                     (lane1_TX_CLK_REG3_WRITE_IN),  // P0-B
        .lane1_TX_CLK_REG4_WRITE_IN                     (lane1_TX_CLK_REG4_WRITE_IN),  // P0-B
        .lane1_TX_CLK_REG5_WRITE_IN                     (lane1_TX_CLK_REG5_WRITE_IN),  // P0-B
        .lane1_TX_DATA_REG1_WRITE_IN                    (lane1_TX_DATA_REG1_WRITE_IN),  // P0-B
        .lane1_TX_DATA_REG2_WRITE_IN                    (lane1_TX_DATA_REG2_WRITE_IN),  // P0-B
        .lane1_TX_DATA_REG3_WRITE_IN                    (lane1_TX_DATA_REG3_WRITE_IN),  // P0-B
        .lane1_TX_DATA_REG4_WRITE_IN                    (lane1_TX_DATA_REG4_WRITE_IN),  // P0-B
        .lane1_TX_DATA_REG5_WRITE_IN                    (lane1_TX_DATA_REG5_WRITE_IN),  // P0-B
        .lane1_TX_DATA_REG6_WRITE_IN                    (lane1_TX_DATA_REG6_WRITE_IN),  // P0-B
        .lane1_TX_DATA_REG7_WRITE_IN                    (lane1_TX_DATA_REG7_WRITE_IN),  // P0-B
        .lane1_TX_LDO_REG1_WRITE_IN                     (lane1_TX_LDO_REG1_WRITE_IN),  // P0-B
        .lane1_TX_LDO_REG2_WRITE_IN                     (lane1_TX_LDO_REG2_WRITE_IN),  // P0-B
`endif
        .lane2_TRX_REG1_WRITE_IN                        (lane2_TRX_REG1_WRITE_IN),  // P0-B
        .lane2_TRX_REG2_WRITE_IN                        (lane2_TRX_REG2_WRITE_IN),  // P0-B
        .lane2_TRX_TEST_REG1_WRITE_IN                   (lane2_TRX_TEST_REG1_WRITE_IN),  // P0-B
        .lane2_TRX_TEST_REG2_WRITE_IN                   (lane2_TRX_TEST_REG2_WRITE_IN),  // P0-B
`ifdef D2D_PHY_RX_LANE2_EN
        .lane2_RX_CLK_ENG_REG1_WRITE_IN                 (lane2_RX_CLK_ENG_REG1_WRITE_IN),  // P0-B
        .lane2_RX_CLK_ENG_REG2_WRITE_IN                 (lane2_RX_CLK_ENG_REG2_WRITE_IN),  // P0-B
        .lane2_RX_CLK_ENG_REG3_WRITE_IN                 (lane2_RX_CLK_ENG_REG3_WRITE_IN),  // P0-B
        .lane2_RX_CLK_ENG_REG4_WRITE_IN                 (lane2_RX_CLK_ENG_REG4_WRITE_IN),  // P0-B
        .lane2_RX_CLK_REG1_WRITE_IN                     (lane2_RX_CLK_REG1_WRITE_IN),  // P0-B
        .lane2_RX_CLK_REG2_WRITE_IN                     (lane2_RX_CLK_REG2_WRITE_IN),  // P0-B
        .lane2_RX_CLK_REG3_WRITE_IN                     (lane2_RX_CLK_REG3_WRITE_IN),  // P0-B
        .lane2_RX_CLK_REG4_WRITE_IN                     (lane2_RX_CLK_REG4_WRITE_IN),  // P0-B
        .lane2_RX_DATA_REG1_WRITE_IN                    (lane2_RX_DATA_REG1_WRITE_IN),  // P0-B
        .lane2_RX_DATA_REG2_WRITE_IN                    (lane2_RX_DATA_REG2_WRITE_IN),  // P0-B
        .lane2_RX_DATA_REG3_WRITE_IN                    (lane2_RX_DATA_REG3_WRITE_IN),  // P0-B
        .lane2_RX_LDO_REG1_WRITE_IN                     (lane2_RX_LDO_REG1_WRITE_IN),  // P0-B
        .lane2_RX_LDO_REG2_WRITE_IN                     (lane2_RX_LDO_REG2_WRITE_IN),  // P0-B
`endif
`ifdef D2D_PHY_TX_LANE2_EN
        .lane2_TX_CLK_ENG_REG1_WRITE_IN                 (lane2_TX_CLK_ENG_REG1_WRITE_IN),  // P0-B
        .lane2_TX_CLK_ENG_REG2_WRITE_IN                 (lane2_TX_CLK_ENG_REG2_WRITE_IN),  // P0-B
        .lane2_TX_CLK_ENG_REG3_WRITE_IN                 (lane2_TX_CLK_ENG_REG3_WRITE_IN),  // P0-B
        .lane2_TX_CLK_ENG_REG4_WRITE_IN                 (lane2_TX_CLK_ENG_REG4_WRITE_IN),  // P0-B
        .lane2_TX_CLK_REG1_WRITE_IN                     (lane2_TX_CLK_REG1_WRITE_IN),  // P0-B
        .lane2_TX_CLK_REG2_WRITE_IN                     (lane2_TX_CLK_REG2_WRITE_IN),  // P0-B
        .lane2_TX_CLK_REG3_WRITE_IN                     (lane2_TX_CLK_REG3_WRITE_IN),  // P0-B
        .lane2_TX_CLK_REG4_WRITE_IN                     (lane2_TX_CLK_REG4_WRITE_IN),  // P0-B
        .lane2_TX_CLK_REG5_WRITE_IN                     (lane2_TX_CLK_REG5_WRITE_IN),  // P0-B
        .lane2_TX_DATA_REG1_WRITE_IN                    (lane2_TX_DATA_REG1_WRITE_IN),  // P0-B
        .lane2_TX_DATA_REG2_WRITE_IN                    (lane2_TX_DATA_REG2_WRITE_IN),  // P0-B
        .lane2_TX_DATA_REG3_WRITE_IN                    (lane2_TX_DATA_REG3_WRITE_IN),  // P0-B
        .lane2_TX_DATA_REG4_WRITE_IN                    (lane2_TX_DATA_REG4_WRITE_IN),  // P0-B
        .lane2_TX_DATA_REG5_WRITE_IN                    (lane2_TX_DATA_REG5_WRITE_IN),  // P0-B
        .lane2_TX_DATA_REG6_WRITE_IN                    (lane2_TX_DATA_REG6_WRITE_IN),  // P0-B
        .lane2_TX_DATA_REG7_WRITE_IN                    (lane2_TX_DATA_REG7_WRITE_IN),  // P0-B
        .lane2_TX_LDO_REG1_WRITE_IN                     (lane2_TX_LDO_REG1_WRITE_IN),  // P0-B
        .lane2_TX_LDO_REG2_WRITE_IN                     (lane2_TX_LDO_REG2_WRITE_IN),  // P0-B
`endif
        .lane3_TRX_REG1_WRITE_IN                        (lane3_TRX_REG1_WRITE_IN),  // P0-B
        .lane3_TRX_REG2_WRITE_IN                        (lane3_TRX_REG2_WRITE_IN),  // P0-B
        .lane3_TRX_TEST_REG1_WRITE_IN                   (lane3_TRX_TEST_REG1_WRITE_IN),  // P0-B
        .lane3_TRX_TEST_REG2_WRITE_IN                   (lane3_TRX_TEST_REG2_WRITE_IN),  // P0-B
`ifdef D2D_PHY_RX_LANE3_EN
        .lane3_RX_CLK_ENG_REG1_WRITE_IN                 (lane3_RX_CLK_ENG_REG1_WRITE_IN),  // P0-B
        .lane3_RX_CLK_ENG_REG2_WRITE_IN                 (lane3_RX_CLK_ENG_REG2_WRITE_IN),  // P0-B
        .lane3_RX_CLK_ENG_REG3_WRITE_IN                 (lane3_RX_CLK_ENG_REG3_WRITE_IN),  // P0-B
        .lane3_RX_CLK_ENG_REG4_WRITE_IN                 (lane3_RX_CLK_ENG_REG4_WRITE_IN),  // P0-B
        .lane3_RX_CLK_REG1_WRITE_IN                     (lane3_RX_CLK_REG1_WRITE_IN),  // P0-B
        .lane3_RX_CLK_REG2_WRITE_IN                     (lane3_RX_CLK_REG2_WRITE_IN),  // P0-B
        .lane3_RX_CLK_REG3_WRITE_IN                     (lane3_RX_CLK_REG3_WRITE_IN),  // P0-B
        .lane3_RX_CLK_REG4_WRITE_IN                     (lane3_RX_CLK_REG4_WRITE_IN),  // P0-B
        .lane3_RX_DATA_REG1_WRITE_IN                    (lane3_RX_DATA_REG1_WRITE_IN),  // P0-B
        .lane3_RX_DATA_REG2_WRITE_IN                    (lane3_RX_DATA_REG2_WRITE_IN),  // P0-B
        .lane3_RX_DATA_REG3_WRITE_IN                    (lane3_RX_DATA_REG3_WRITE_IN),  // P0-B
        .lane3_RX_LDO_REG1_WRITE_IN                     (lane3_RX_LDO_REG1_WRITE_IN),  // P0-B
        .lane3_RX_LDO_REG2_WRITE_IN                     (lane3_RX_LDO_REG2_WRITE_IN),  // P0-B
`endif
`ifdef D2D_PHY_TX_LANE3_EN
        .lane3_TX_CLK_ENG_REG1_WRITE_IN                 (lane3_TX_CLK_ENG_REG1_WRITE_IN),  // P0-B
        .lane3_TX_CLK_ENG_REG2_WRITE_IN                 (lane3_TX_CLK_ENG_REG2_WRITE_IN),  // P0-B
        .lane3_TX_CLK_ENG_REG3_WRITE_IN                 (lane3_TX_CLK_ENG_REG3_WRITE_IN),  // P0-B
        .lane3_TX_CLK_ENG_REG4_WRITE_IN                 (lane3_TX_CLK_ENG_REG4_WRITE_IN),  // P0-B
        .lane3_TX_CLK_REG1_WRITE_IN                     (lane3_TX_CLK_REG1_WRITE_IN),  // P0-B
        .lane3_TX_CLK_REG2_WRITE_IN                     (lane3_TX_CLK_REG2_WRITE_IN),  // P0-B
        .lane3_TX_CLK_REG3_WRITE_IN                     (lane3_TX_CLK_REG3_WRITE_IN),  // P0-B
        .lane3_TX_CLK_REG4_WRITE_IN                     (lane3_TX_CLK_REG4_WRITE_IN),  // P0-B
        .lane3_TX_CLK_REG5_WRITE_IN                     (lane3_TX_CLK_REG5_WRITE_IN),  // P0-B
        .lane3_TX_DATA_REG1_WRITE_IN                    (lane3_TX_DATA_REG1_WRITE_IN),  // P0-B
        .lane3_TX_DATA_REG2_WRITE_IN                    (lane3_TX_DATA_REG2_WRITE_IN),  // P0-B
        .lane3_TX_DATA_REG3_WRITE_IN                    (lane3_TX_DATA_REG3_WRITE_IN),  // P0-B
        .lane3_TX_DATA_REG4_WRITE_IN                    (lane3_TX_DATA_REG4_WRITE_IN),  // P0-B
        .lane3_TX_DATA_REG5_WRITE_IN                    (lane3_TX_DATA_REG5_WRITE_IN),  // P0-B
        .lane3_TX_DATA_REG6_WRITE_IN                    (lane3_TX_DATA_REG6_WRITE_IN),  // P0-B
        .lane3_TX_DATA_REG7_WRITE_IN                    (lane3_TX_DATA_REG7_WRITE_IN),  // P0-B
        .lane3_TX_LDO_REG1_WRITE_IN                     (lane3_TX_LDO_REG1_WRITE_IN),  // P0-B
        .lane3_TX_LDO_REG2_WRITE_IN                     (lane3_TX_LDO_REG2_WRITE_IN),  // P0-B
`endif
        .lane0_TX_EN_ASYNC                              (lane0_TX_EN_ASYNC),  // P0-B
        .lane0_TX_EN_SYNC                               (lane0_TX_EN_SYNC),  // P0-B
        .lane0_TX_EN_TRACK                              (lane0_TX_EN_TRACK),  // P0-B
        .lane0_TX_RSTB_SYNC                             (lane0_TX_RSTB_SYNC),  // P0-B
        .lane0_RX_CLK_EN_ASYNC                          (lane0_RX_CLK_EN_ASYNC),  // P0-B
        .lane0_RX_CLK_EN_SYNC                           (lane0_RX_CLK_EN_SYNC),  // P0-B
        .lane0_RX_CLK_EN_TRACK                          (lane0_RX_CLK_EN_TRACK),  // P0-B
        .lane0_RX_RSTB_SYNC                             (lane0_RX_RSTB_SYNC),  // P0-B
        .lane1_TX_EN_ASYNC                              (lane1_TX_EN_ASYNC),  // P0-B
        .lane1_TX_EN_SYNC                               (lane1_TX_EN_SYNC),  // P0-B
        .lane1_TX_EN_TRACK                              (lane1_TX_EN_TRACK),  // P0-B
        .lane1_TX_RSTB_SYNC                             (lane1_TX_RSTB_SYNC),  // P0-B
        .lane1_RX_CLK_EN_ASYNC                          (lane1_RX_CLK_EN_ASYNC),  // P0-B
        .lane1_RX_CLK_EN_SYNC                           (lane1_RX_CLK_EN_SYNC),  // P0-B
        .lane1_RX_CLK_EN_TRACK                          (lane1_RX_CLK_EN_TRACK),  // P0-B
        .lane1_RX_RSTB_SYNC                             (lane1_RX_RSTB_SYNC),  // P0-B
        .lane2_TX_EN_ASYNC                              (lane2_TX_EN_ASYNC),  // P0-B
        .lane2_TX_EN_SYNC                               (lane2_TX_EN_SYNC),  // P0-B
        .lane2_TX_EN_TRACK                              (lane2_TX_EN_TRACK),  // P0-B
        .lane2_TX_RSTB_SYNC                             (lane2_TX_RSTB_SYNC),  // P0-B
        .lane2_RX_CLK_EN_ASYNC                          (lane2_RX_CLK_EN_ASYNC),  // P0-B
        .lane2_RX_CLK_EN_SYNC                           (lane2_RX_CLK_EN_SYNC),  // P0-B
        .lane2_RX_CLK_EN_TRACK                          (lane2_RX_CLK_EN_TRACK),  // P0-B
        .lane2_RX_RSTB_SYNC                             (lane2_RX_RSTB_SYNC),  // P0-B
        .lane3_TX_EN_ASYNC                              (lane3_TX_EN_ASYNC),  // P0-B
        .lane3_TX_EN_SYNC                               (lane3_TX_EN_SYNC),  // P0-B
        .lane3_TX_EN_TRACK                              (lane3_TX_EN_TRACK),  // P0-B
        .lane3_TX_RSTB_SYNC                             (lane3_TX_RSTB_SYNC),  // P0-B
        .lane3_RX_CLK_EN_ASYNC                          (lane3_RX_CLK_EN_ASYNC),  // P0-B
        .lane3_RX_CLK_EN_SYNC                           (lane3_RX_CLK_EN_SYNC),  // P0-B
        .lane3_RX_CLK_EN_TRACK                          (lane3_RX_CLK_EN_TRACK),  // P0-B
        .lane3_RX_RSTB_SYNC                             (lane3_RX_RSTB_SYNC),  // P0-B
        .lane0_RX_CLK_ENG_REG_READ_OUT                  (lane0_RX_CLK_ENG_REG_READ_OUT),  // P0-B
        .lane0_TX_CLK_ENG_REG_READ_OUT                  (lane0_TX_CLK_ENG_REG_READ_OUT),  // P0-B
        .lane0_CONNECT_DIG_OUT                          (lane0_CONNECT_DIG_OUT),  // P0-B
        .lane1_RX_CLK_ENG_REG_READ_OUT                  (lane1_RX_CLK_ENG_REG_READ_OUT),  // P0-B
        .lane1_TX_CLK_ENG_REG_READ_OUT                  (lane1_TX_CLK_ENG_REG_READ_OUT),  // P0-B
        .lane1_CONNECT_DIG_OUT                          (lane1_CONNECT_DIG_OUT),  // P0-B
        .lane2_RX_CLK_ENG_REG_READ_OUT                  (lane2_RX_CLK_ENG_REG_READ_OUT),  // P0-B
        .lane2_TX_CLK_ENG_REG_READ_OUT                  (lane2_TX_CLK_ENG_REG_READ_OUT),  // P0-B
        .lane2_CONNECT_DIG_OUT                          (lane2_CONNECT_DIG_OUT),  // P0-B
        .lane3_RX_CLK_ENG_REG_READ_OUT                  (lane3_RX_CLK_ENG_REG_READ_OUT),  // P0-B
        .lane3_TX_CLK_ENG_REG_READ_OUT                  (lane3_TX_CLK_ENG_REG_READ_OUT),  // P0-B
        .lane3_CONNECT_DIG_OUT                          (lane3_CONNECT_DIG_OUT),  // P0-B
        .lane0_DOUT0_ERR_EDGE                           (lane0_DOUT0_ERR_EDGE),  // P0-B
        .lane0_DOUT1_ERR_EDGE                           (lane0_DOUT1_ERR_EDGE),  // P0-B
        .lane0_DOUT2_ERR_EDGE                           (lane0_DOUT2_ERR_EDGE),  // P0-B
        .lane0_DOUT3_ERR_EDGE                           (lane0_DOUT3_ERR_EDGE),  // P0-B
        .lane0_DOUT4_ERR_EDGE                           (lane0_DOUT4_ERR_EDGE),  // P0-B
        .lane0_DOUT5_ERR_EDGE                           (lane0_DOUT5_ERR_EDGE),  // P0-B
        .lane0_DOUT6_ERR_EDGE                           (lane0_DOUT6_ERR_EDGE),  // P0-B
        .lane0_DOUT7_ERR_EDGE                           (lane0_DOUT7_ERR_EDGE),  // P0-B
        .lane0_DOUT8_ERR_EDGE                           (lane0_DOUT8_ERR_EDGE),  // P0-B
        .lane0_DOUT9_ERR_EDGE                           (lane0_DOUT9_ERR_EDGE),  // P0-B
        .lane0_DOUT10_ERR_EDGE                          (lane0_DOUT10_ERR_EDGE),  // P0-B
        .lane0_DOUT11_ERR_EDGE                          (lane0_DOUT11_ERR_EDGE),  // P0-B
        .lane0_DOUT12_ERR_EDGE                          (lane0_DOUT12_ERR_EDGE),  // P0-B
        .lane0_DOUT13_ERR_EDGE                          (lane0_DOUT13_ERR_EDGE),  // P0-B
        .lane0_DOUT14_ERR_EDGE                          (lane0_DOUT14_ERR_EDGE),  // P0-B
        .lane0_DOUT15_ERR_EDGE                          (lane0_DOUT15_ERR_EDGE),  // P0-B
        .lane0_DOUTRFU_ERR_EDGE                         (lane0_DOUTRFU_ERR_EDGE),  // P0-B
        .lane1_DOUT0_ERR_EDGE                           (lane1_DOUT0_ERR_EDGE),  // P0-B
        .lane1_DOUT1_ERR_EDGE                           (lane1_DOUT1_ERR_EDGE),  // P0-B
        .lane1_DOUT2_ERR_EDGE                           (lane1_DOUT2_ERR_EDGE),  // P0-B
        .lane1_DOUT3_ERR_EDGE                           (lane1_DOUT3_ERR_EDGE),  // P0-B
        .lane1_DOUT4_ERR_EDGE                           (lane1_DOUT4_ERR_EDGE),  // P0-B
        .lane1_DOUT5_ERR_EDGE                           (lane1_DOUT5_ERR_EDGE),  // P0-B
        .lane1_DOUT6_ERR_EDGE                           (lane1_DOUT6_ERR_EDGE),  // P0-B
        .lane1_DOUT7_ERR_EDGE                           (lane1_DOUT7_ERR_EDGE),  // P0-B
        .lane1_DOUT8_ERR_EDGE                           (lane1_DOUT8_ERR_EDGE),  // P0-B
        .lane1_DOUT9_ERR_EDGE                           (lane1_DOUT9_ERR_EDGE),  // P0-B
        .lane1_DOUT10_ERR_EDGE                          (lane1_DOUT10_ERR_EDGE),  // P0-B
        .lane1_DOUT11_ERR_EDGE                          (lane1_DOUT11_ERR_EDGE),  // P0-B
        .lane1_DOUT12_ERR_EDGE                          (lane1_DOUT12_ERR_EDGE),  // P0-B
        .lane1_DOUT13_ERR_EDGE                          (lane1_DOUT13_ERR_EDGE),  // P0-B
        .lane1_DOUT14_ERR_EDGE                          (lane1_DOUT14_ERR_EDGE),  // P0-B
        .lane1_DOUT15_ERR_EDGE                          (lane1_DOUT15_ERR_EDGE),  // P0-B
        .lane1_DOUTRFU_ERR_EDGE                         (lane1_DOUTRFU_ERR_EDGE),  // P0-B
        .lane2_DOUT0_ERR_EDGE                           (lane2_DOUT0_ERR_EDGE),  // P0-B
        .lane2_DOUT1_ERR_EDGE                           (lane2_DOUT1_ERR_EDGE),  // P0-B
        .lane2_DOUT2_ERR_EDGE                           (lane2_DOUT2_ERR_EDGE),  // P0-B
        .lane2_DOUT3_ERR_EDGE                           (lane2_DOUT3_ERR_EDGE),  // P0-B
        .lane2_DOUT4_ERR_EDGE                           (lane2_DOUT4_ERR_EDGE),  // P0-B
        .lane2_DOUT5_ERR_EDGE                           (lane2_DOUT5_ERR_EDGE),  // P0-B
        .lane2_DOUT6_ERR_EDGE                           (lane2_DOUT6_ERR_EDGE),  // P0-B
        .lane2_DOUT7_ERR_EDGE                           (lane2_DOUT7_ERR_EDGE),  // P0-B
        .lane2_DOUT8_ERR_EDGE                           (lane2_DOUT8_ERR_EDGE),  // P0-B
        .lane2_DOUT9_ERR_EDGE                           (lane2_DOUT9_ERR_EDGE),  // P0-B
        .lane2_DOUT10_ERR_EDGE                          (lane2_DOUT10_ERR_EDGE),  // P0-B
        .lane2_DOUT11_ERR_EDGE                          (lane2_DOUT11_ERR_EDGE),  // P0-B
        .lane2_DOUT12_ERR_EDGE                          (lane2_DOUT12_ERR_EDGE),  // P0-B
        .lane2_DOUT13_ERR_EDGE                          (lane2_DOUT13_ERR_EDGE),  // P0-B
        .lane2_DOUT14_ERR_EDGE                          (lane2_DOUT14_ERR_EDGE),  // P0-B
        .lane2_DOUT15_ERR_EDGE                          (lane2_DOUT15_ERR_EDGE),  // P0-B
        .lane2_DOUTRFU_ERR_EDGE                         (lane2_DOUTRFU_ERR_EDGE),  // P0-B
        .lane3_DOUT0_ERR_EDGE                           (lane3_DOUT0_ERR_EDGE),  // P0-B
        .lane3_DOUT1_ERR_EDGE                           (lane3_DOUT1_ERR_EDGE),  // P0-B
        .lane3_DOUT2_ERR_EDGE                           (lane3_DOUT2_ERR_EDGE),  // P0-B
        .lane3_DOUT3_ERR_EDGE                           (lane3_DOUT3_ERR_EDGE),  // P0-B
        .lane3_DOUT4_ERR_EDGE                           (lane3_DOUT4_ERR_EDGE),  // P0-B
        .lane3_DOUT5_ERR_EDGE                           (lane3_DOUT5_ERR_EDGE),  // P0-B
        .lane3_DOUT6_ERR_EDGE                           (lane3_DOUT6_ERR_EDGE),  // P0-B
        .lane3_DOUT7_ERR_EDGE                           (lane3_DOUT7_ERR_EDGE),  // P0-B
        .lane3_DOUT8_ERR_EDGE                           (lane3_DOUT8_ERR_EDGE),  // P0-B
        .lane3_DOUT9_ERR_EDGE                           (lane3_DOUT9_ERR_EDGE),  // P0-B
        .lane3_DOUT10_ERR_EDGE                          (lane3_DOUT10_ERR_EDGE),  // P0-B
        .lane3_DOUT11_ERR_EDGE                          (lane3_DOUT11_ERR_EDGE),  // P0-B
        .lane3_DOUT12_ERR_EDGE                          (lane3_DOUT12_ERR_EDGE),  // P0-B
        .lane3_DOUT13_ERR_EDGE                          (lane3_DOUT13_ERR_EDGE),  // P0-B
        .lane3_DOUT14_ERR_EDGE                          (lane3_DOUT14_ERR_EDGE),  // P0-B
        .lane3_DOUT15_ERR_EDGE                          (lane3_DOUT15_ERR_EDGE),  // P0-B
        .lane3_DOUTRFU_ERR_EDGE                         (lane3_DOUTRFU_ERR_EDGE),  // P0-B
        .lane0_TX_LPC_DATA_IN                           (lane0_TX_LPC_DATA_IN),  // P0-B
        .lane0_RX_LPC_DATA_OUT                          (lane0_RX_LPC_DATA_OUT),  // P0-B
        .lane1_TX_LPC_DATA_IN                           (lane1_TX_LPC_DATA_IN),  // P0-B
`ifdef D2D_PHY_RX_LANE1_EN
        .lane1_RX_LPC_DATA_OUT                          (lane1_RX_LPC_DATA_OUT),  // P0-B
`endif
        .lane2_TX_LPC_DATA_IN                           (lane2_TX_LPC_DATA_IN),  // P0-B
`ifdef D2D_PHY_RX_LANE2_EN
        .lane2_RX_LPC_DATA_OUT                          (lane2_RX_LPC_DATA_OUT),  // P0-B
`endif
        .lane3_TX_LPC_DATA_IN                           (lane3_TX_LPC_DATA_IN),  // P0-B
`ifdef D2D_PHY_RX_LANE3_EN
        .lane3_RX_LPC_DATA_OUT                          (lane3_RX_LPC_DATA_OUT),  // P0-B
`endif
        .cpif_rx_lpc                                    (cpif_rx_lpc),  // P0-B
`ifdef D2D_PHY_TX_LANE1_EN
        .tx_lane1_data0              (tx_lane1_data0),   // 补接（原遗漏 data0，仅 data1~16 已接）
        .tx_lane1_data1              (tx_lane1_data1),
        .tx_lane1_data2              (tx_lane1_data2),
        .tx_lane1_data3              (tx_lane1_data3),
        .tx_lane1_data4              (tx_lane1_data4),
        .tx_lane1_data5              (tx_lane1_data5),
        .tx_lane1_data6              (tx_lane1_data6),
        .tx_lane1_data7              (tx_lane1_data7),
        .tx_lane1_data8              (tx_lane1_data8),
        .tx_lane1_data9              (tx_lane1_data9),
        .tx_lane1_data10             (tx_lane1_data10),
        .tx_lane1_data11             (tx_lane1_data11),
        .tx_lane1_data12             (tx_lane1_data12),
        .tx_lane1_data13             (tx_lane1_data13),
        .tx_lane1_data14             (tx_lane1_data14),
        .tx_lane1_data15             (tx_lane1_data15),
        .tx_lane1_data16             (tx_lane1_data16),
`endif
`ifdef D2D_PHY_RX_LANE1_EN
        .rx_lane1_data0              (rx_lane1_data0),
        .rx_lane1_data1              (rx_lane1_data1),
        .rx_lane1_data2              (rx_lane1_data2),
        .rx_lane1_data3              (rx_lane1_data3),
        .rx_lane1_data4              (rx_lane1_data4),
        .rx_lane1_data5              (rx_lane1_data5),
        .rx_lane1_data6              (rx_lane1_data6),
        .rx_lane1_data7              (rx_lane1_data7),
        .rx_lane1_data8              (rx_lane1_data8),
        .rx_lane1_data9              (rx_lane1_data9),
        .rx_lane1_data10             (rx_lane1_data10),
        .rx_lane1_data11             (rx_lane1_data11),
        .rx_lane1_data12             (rx_lane1_data12),
        .rx_lane1_data13             (rx_lane1_data13),
        .rx_lane1_data14             (rx_lane1_data14),
        .rx_lane1_data15             (rx_lane1_data15),
        .rx_lane1_data16             (rx_lane1_data16),
        .lane1_RX_OFFSET_CALIBRATION_ON                 (lane1_RX_OFFSET_CALIBRATION_ON),
        .lane1_OFFSET_CTRL_CLK                          (lane1_OFFSET_CTRL_CLK),
        .lane1_OFFSET_RESET_N                           (lane1_OFFSET_RESET_N),
        .lane1_RX0_OFFSET_CTRL_WRITE                    (lane1_RX0_OFFSET_CTRL_WRITE),
        .lane1_RX0_OFFSET_CTRL_ADDR                     (lane1_RX0_OFFSET_CTRL_ADDR),
        .lane1_RX0_OFFSET_CTRL_DATA                     (lane1_RX0_OFFSET_CTRL_DATA),
        .lane1_RX1_OFFSET_CTRL_WRITE                    (lane1_RX1_OFFSET_CTRL_WRITE),
        .lane1_RX1_OFFSET_CTRL_ADDR                     (lane1_RX1_OFFSET_CTRL_ADDR),
        .lane1_RX1_OFFSET_CTRL_DATA                     (lane1_RX1_OFFSET_CTRL_DATA),
        .lane1_RX2_OFFSET_CTRL_WRITE                    (lane1_RX2_OFFSET_CTRL_WRITE),
        .lane1_RX2_OFFSET_CTRL_ADDR                     (lane1_RX2_OFFSET_CTRL_ADDR),
        .lane1_RX2_OFFSET_CTRL_DATA                     (lane1_RX2_OFFSET_CTRL_DATA),
        .lane1_RX3_OFFSET_CTRL_WRITE                    (lane1_RX3_OFFSET_CTRL_WRITE),
        .lane1_RX3_OFFSET_CTRL_ADDR                     (lane1_RX3_OFFSET_CTRL_ADDR),
        .lane1_RX3_OFFSET_CTRL_DATA                     (lane1_RX3_OFFSET_CTRL_DATA),
        .lane1_RX4_OFFSET_CTRL_WRITE                    (lane1_RX4_OFFSET_CTRL_WRITE),
        .lane1_RX4_OFFSET_CTRL_ADDR                     (lane1_RX4_OFFSET_CTRL_ADDR),
        .lane1_RX4_OFFSET_CTRL_DATA                     (lane1_RX4_OFFSET_CTRL_DATA),
        .lane1_RX5_OFFSET_CTRL_WRITE                    (lane1_RX5_OFFSET_CTRL_WRITE),
        .lane1_RX5_OFFSET_CTRL_ADDR                     (lane1_RX5_OFFSET_CTRL_ADDR),
        .lane1_RX5_OFFSET_CTRL_DATA                     (lane1_RX5_OFFSET_CTRL_DATA),
        .lane1_RX6_OFFSET_CTRL_WRITE                    (lane1_RX6_OFFSET_CTRL_WRITE),
        .lane1_RX6_OFFSET_CTRL_ADDR                     (lane1_RX6_OFFSET_CTRL_ADDR),
        .lane1_RX6_OFFSET_CTRL_DATA                     (lane1_RX6_OFFSET_CTRL_DATA),
        .lane1_RX7_OFFSET_CTRL_WRITE                    (lane1_RX7_OFFSET_CTRL_WRITE),
        .lane1_RX7_OFFSET_CTRL_ADDR                     (lane1_RX7_OFFSET_CTRL_ADDR),
        .lane1_RX7_OFFSET_CTRL_DATA                     (lane1_RX7_OFFSET_CTRL_DATA),
        .lane1_RX8_OFFSET_CTRL_WRITE                    (lane1_RX8_OFFSET_CTRL_WRITE),
        .lane1_RX8_OFFSET_CTRL_ADDR                     (lane1_RX8_OFFSET_CTRL_ADDR),
        .lane1_RX8_OFFSET_CTRL_DATA                     (lane1_RX8_OFFSET_CTRL_DATA),
        .lane1_RX9_OFFSET_CTRL_WRITE                    (lane1_RX9_OFFSET_CTRL_WRITE),
        .lane1_RX9_OFFSET_CTRL_ADDR                     (lane1_RX9_OFFSET_CTRL_ADDR),
        .lane1_RX9_OFFSET_CTRL_DATA                     (lane1_RX9_OFFSET_CTRL_DATA),
        .lane1_RX10_OFFSET_CTRL_WRITE                   (lane1_RX10_OFFSET_CTRL_WRITE),
        .lane1_RX10_OFFSET_CTRL_ADDR                    (lane1_RX10_OFFSET_CTRL_ADDR),
        .lane1_RX10_OFFSET_CTRL_DATA                    (lane1_RX10_OFFSET_CTRL_DATA),
        .lane1_RX11_OFFSET_CTRL_WRITE                   (lane1_RX11_OFFSET_CTRL_WRITE),
        .lane1_RX11_OFFSET_CTRL_ADDR                    (lane1_RX11_OFFSET_CTRL_ADDR),
        .lane1_RX11_OFFSET_CTRL_DATA                    (lane1_RX11_OFFSET_CTRL_DATA),
        .lane1_RX12_OFFSET_CTRL_WRITE                   (lane1_RX12_OFFSET_CTRL_WRITE),
        .lane1_RX12_OFFSET_CTRL_ADDR                    (lane1_RX12_OFFSET_CTRL_ADDR),
        .lane1_RX12_OFFSET_CTRL_DATA                    (lane1_RX12_OFFSET_CTRL_DATA),
        .lane1_RX13_OFFSET_CTRL_WRITE                   (lane1_RX13_OFFSET_CTRL_WRITE),
        .lane1_RX13_OFFSET_CTRL_ADDR                    (lane1_RX13_OFFSET_CTRL_ADDR),
        .lane1_RX13_OFFSET_CTRL_DATA                    (lane1_RX13_OFFSET_CTRL_DATA),
        .lane1_RX14_OFFSET_CTRL_WRITE                   (lane1_RX14_OFFSET_CTRL_WRITE),
        .lane1_RX14_OFFSET_CTRL_ADDR                    (lane1_RX14_OFFSET_CTRL_ADDR),
        .lane1_RX14_OFFSET_CTRL_DATA                    (lane1_RX14_OFFSET_CTRL_DATA),
        .lane1_RX15_OFFSET_CTRL_WRITE                   (lane1_RX15_OFFSET_CTRL_WRITE),
        .lane1_RX15_OFFSET_CTRL_ADDR                    (lane1_RX15_OFFSET_CTRL_ADDR),
        .lane1_RX15_OFFSET_CTRL_DATA                    (lane1_RX15_OFFSET_CTRL_DATA),
        .lane1_RX16_OFFSET_CTRL_WRITE                   (lane1_RX16_OFFSET_CTRL_WRITE),
        .lane1_RX16_OFFSET_CTRL_ADDR                    (lane1_RX16_OFFSET_CTRL_ADDR),
        .lane1_RX16_OFFSET_CTRL_DATA                    (lane1_RX16_OFFSET_CTRL_DATA),
        .lane1_RX_VREF_CTRL_MAIN                        (lane1_RX_VREF_CTRL_MAIN),
        .lane1_RX_VREF_CTRL_ERR_EDGE                    (lane1_RX_VREF_CTRL_ERR_EDGE),
        .lane1_PH_I_GRAY_PI                             (lane1_PH_I_GRAY_PI),
        .lane1_PH_Q_GRAY_PI                             (lane1_PH_Q_GRAY_PI),
        .lane1_PH_045_GRAY_PI                           (lane1_PH_045_GRAY_PI),
        .lane1_RX0_DCDL_CTRL                            (lane1_RX0_DCDL_CTRL),
        .lane1_RX1_DCDL_CTRL                            (lane1_RX1_DCDL_CTRL),
        .lane1_RX2_DCDL_CTRL                            (lane1_RX2_DCDL_CTRL),
        .lane1_RX3_DCDL_CTRL                            (lane1_RX3_DCDL_CTRL),
        .lane1_RX4_DCDL_CTRL                            (lane1_RX4_DCDL_CTRL),
        .lane1_RX5_DCDL_CTRL                            (lane1_RX5_DCDL_CTRL),
        .lane1_RX6_DCDL_CTRL                            (lane1_RX6_DCDL_CTRL),
        .lane1_RX7_DCDL_CTRL                            (lane1_RX7_DCDL_CTRL),
        .lane1_RX8_DCDL_CTRL                            (lane1_RX8_DCDL_CTRL),
        .lane1_RX9_DCDL_CTRL                            (lane1_RX9_DCDL_CTRL),
        .lane1_RX10_DCDL_CTRL                           (lane1_RX10_DCDL_CTRL),
        .lane1_RX11_DCDL_CTRL                           (lane1_RX11_DCDL_CTRL),
        .lane1_RX12_DCDL_CTRL                           (lane1_RX12_DCDL_CTRL),
        .lane1_RX13_DCDL_CTRL                           (lane1_RX13_DCDL_CTRL),
        .lane1_RX14_DCDL_CTRL                           (lane1_RX14_DCDL_CTRL),
        .lane1_RX15_DCDL_CTRL                           (lane1_RX15_DCDL_CTRL),
        .lane1_RX16_DCDL_CTRL                           (lane1_RX16_DCDL_CTRL),
`endif
`ifdef D2D_PHY_TX_LANE2_EN
        .tx_lane2_data0              (tx_lane2_data0),   // 补接（原遗漏 data0，仅 data1~16 已接）
        .tx_lane2_data1              (tx_lane2_data1),
        .tx_lane2_data2              (tx_lane2_data2),
        .tx_lane2_data3              (tx_lane2_data3),
        .tx_lane2_data4              (tx_lane2_data4),
        .tx_lane2_data5              (tx_lane2_data5),
        .tx_lane2_data6              (tx_lane2_data6),
        .tx_lane2_data7              (tx_lane2_data7),
        .tx_lane2_data8              (tx_lane2_data8),
        .tx_lane2_data9              (tx_lane2_data9),
        .tx_lane2_data10             (tx_lane2_data10),
        .tx_lane2_data11             (tx_lane2_data11),
        .tx_lane2_data12             (tx_lane2_data12),
        .tx_lane2_data13             (tx_lane2_data13),
        .tx_lane2_data14             (tx_lane2_data14),
        .tx_lane2_data15             (tx_lane2_data15),
        .tx_lane2_data16             (tx_lane2_data16),
`endif
`ifdef D2D_PHY_RX_LANE2_EN
        .rx_lane2_data0              (rx_lane2_data0),
        .rx_lane2_data1              (rx_lane2_data1),
        .rx_lane2_data2              (rx_lane2_data2),
        .rx_lane2_data3              (rx_lane2_data3),
        .rx_lane2_data4              (rx_lane2_data4),
        .rx_lane2_data5              (rx_lane2_data5),
        .rx_lane2_data6              (rx_lane2_data6),
        .rx_lane2_data7              (rx_lane2_data7),
        .rx_lane2_data8              (rx_lane2_data8),
        .rx_lane2_data9              (rx_lane2_data9),
        .rx_lane2_data10             (rx_lane2_data10),
        .rx_lane2_data11             (rx_lane2_data11),
        .rx_lane2_data12             (rx_lane2_data12),
        .rx_lane2_data13             (rx_lane2_data13),
        .rx_lane2_data14             (rx_lane2_data14),
        .rx_lane2_data15             (rx_lane2_data15),
        .rx_lane2_data16             (rx_lane2_data16),
        .lane2_RX_OFFSET_CALIBRATION_ON                 (lane2_RX_OFFSET_CALIBRATION_ON),
        .lane2_OFFSET_CTRL_CLK                          (lane2_OFFSET_CTRL_CLK),
        .lane2_OFFSET_RESET_N                           (lane2_OFFSET_RESET_N),
        .lane2_RX0_OFFSET_CTRL_WRITE                    (lane2_RX0_OFFSET_CTRL_WRITE),
        .lane2_RX0_OFFSET_CTRL_ADDR                     (lane2_RX0_OFFSET_CTRL_ADDR),
        .lane2_RX0_OFFSET_CTRL_DATA                     (lane2_RX0_OFFSET_CTRL_DATA),
        .lane2_RX1_OFFSET_CTRL_WRITE                    (lane2_RX1_OFFSET_CTRL_WRITE),
        .lane2_RX1_OFFSET_CTRL_ADDR                     (lane2_RX1_OFFSET_CTRL_ADDR),
        .lane2_RX1_OFFSET_CTRL_DATA                     (lane2_RX1_OFFSET_CTRL_DATA),
        .lane2_RX2_OFFSET_CTRL_WRITE                    (lane2_RX2_OFFSET_CTRL_WRITE),
        .lane2_RX2_OFFSET_CTRL_ADDR                     (lane2_RX2_OFFSET_CTRL_ADDR),
        .lane2_RX2_OFFSET_CTRL_DATA                     (lane2_RX2_OFFSET_CTRL_DATA),
        .lane2_RX3_OFFSET_CTRL_WRITE                    (lane2_RX3_OFFSET_CTRL_WRITE),
        .lane2_RX3_OFFSET_CTRL_ADDR                     (lane2_RX3_OFFSET_CTRL_ADDR),
        .lane2_RX3_OFFSET_CTRL_DATA                     (lane2_RX3_OFFSET_CTRL_DATA),
        .lane2_RX4_OFFSET_CTRL_WRITE                    (lane2_RX4_OFFSET_CTRL_WRITE),
        .lane2_RX4_OFFSET_CTRL_ADDR                     (lane2_RX4_OFFSET_CTRL_ADDR),
        .lane2_RX4_OFFSET_CTRL_DATA                     (lane2_RX4_OFFSET_CTRL_DATA),
        .lane2_RX5_OFFSET_CTRL_WRITE                    (lane2_RX5_OFFSET_CTRL_WRITE),
        .lane2_RX5_OFFSET_CTRL_ADDR                     (lane2_RX5_OFFSET_CTRL_ADDR),
        .lane2_RX5_OFFSET_CTRL_DATA                     (lane2_RX5_OFFSET_CTRL_DATA),
        .lane2_RX6_OFFSET_CTRL_WRITE                    (lane2_RX6_OFFSET_CTRL_WRITE),
        .lane2_RX6_OFFSET_CTRL_ADDR                     (lane2_RX6_OFFSET_CTRL_ADDR),
        .lane2_RX6_OFFSET_CTRL_DATA                     (lane2_RX6_OFFSET_CTRL_DATA),
        .lane2_RX7_OFFSET_CTRL_WRITE                    (lane2_RX7_OFFSET_CTRL_WRITE),
        .lane2_RX7_OFFSET_CTRL_ADDR                     (lane2_RX7_OFFSET_CTRL_ADDR),
        .lane2_RX7_OFFSET_CTRL_DATA                     (lane2_RX7_OFFSET_CTRL_DATA),
        .lane2_RX8_OFFSET_CTRL_WRITE                    (lane2_RX8_OFFSET_CTRL_WRITE),
        .lane2_RX8_OFFSET_CTRL_ADDR                     (lane2_RX8_OFFSET_CTRL_ADDR),
        .lane2_RX8_OFFSET_CTRL_DATA                     (lane2_RX8_OFFSET_CTRL_DATA),
        .lane2_RX9_OFFSET_CTRL_WRITE                    (lane2_RX9_OFFSET_CTRL_WRITE),
        .lane2_RX9_OFFSET_CTRL_ADDR                     (lane2_RX9_OFFSET_CTRL_ADDR),
        .lane2_RX9_OFFSET_CTRL_DATA                     (lane2_RX9_OFFSET_CTRL_DATA),
        .lane2_RX10_OFFSET_CTRL_WRITE                   (lane2_RX10_OFFSET_CTRL_WRITE),
        .lane2_RX10_OFFSET_CTRL_ADDR                    (lane2_RX10_OFFSET_CTRL_ADDR),
        .lane2_RX10_OFFSET_CTRL_DATA                    (lane2_RX10_OFFSET_CTRL_DATA),
        .lane2_RX11_OFFSET_CTRL_WRITE                   (lane2_RX11_OFFSET_CTRL_WRITE),
        .lane2_RX11_OFFSET_CTRL_ADDR                    (lane2_RX11_OFFSET_CTRL_ADDR),
        .lane2_RX11_OFFSET_CTRL_DATA                    (lane2_RX11_OFFSET_CTRL_DATA),
        .lane2_RX12_OFFSET_CTRL_WRITE                   (lane2_RX12_OFFSET_CTRL_WRITE),
        .lane2_RX12_OFFSET_CTRL_ADDR                    (lane2_RX12_OFFSET_CTRL_ADDR),
        .lane2_RX12_OFFSET_CTRL_DATA                    (lane2_RX12_OFFSET_CTRL_DATA),
        .lane2_RX13_OFFSET_CTRL_WRITE                   (lane2_RX13_OFFSET_CTRL_WRITE),
        .lane2_RX13_OFFSET_CTRL_ADDR                    (lane2_RX13_OFFSET_CTRL_ADDR),
        .lane2_RX13_OFFSET_CTRL_DATA                    (lane2_RX13_OFFSET_CTRL_DATA),
        .lane2_RX14_OFFSET_CTRL_WRITE                   (lane2_RX14_OFFSET_CTRL_WRITE),
        .lane2_RX14_OFFSET_CTRL_ADDR                    (lane2_RX14_OFFSET_CTRL_ADDR),
        .lane2_RX14_OFFSET_CTRL_DATA                    (lane2_RX14_OFFSET_CTRL_DATA),
        .lane2_RX15_OFFSET_CTRL_WRITE                   (lane2_RX15_OFFSET_CTRL_WRITE),
        .lane2_RX15_OFFSET_CTRL_ADDR                    (lane2_RX15_OFFSET_CTRL_ADDR),
        .lane2_RX15_OFFSET_CTRL_DATA                    (lane2_RX15_OFFSET_CTRL_DATA),
        .lane2_RX16_OFFSET_CTRL_WRITE                   (lane2_RX16_OFFSET_CTRL_WRITE),
        .lane2_RX16_OFFSET_CTRL_ADDR                    (lane2_RX16_OFFSET_CTRL_ADDR),
        .lane2_RX16_OFFSET_CTRL_DATA                    (lane2_RX16_OFFSET_CTRL_DATA),
        .lane2_RX_VREF_CTRL_MAIN                        (lane2_RX_VREF_CTRL_MAIN),
        .lane2_RX_VREF_CTRL_ERR_EDGE                    (lane2_RX_VREF_CTRL_ERR_EDGE),
        .lane2_PH_I_GRAY_PI                             (lane2_PH_I_GRAY_PI),
        .lane2_PH_Q_GRAY_PI                             (lane2_PH_Q_GRAY_PI),
        .lane2_PH_045_GRAY_PI                           (lane2_PH_045_GRAY_PI),
        .lane2_RX0_DCDL_CTRL                            (lane2_RX0_DCDL_CTRL),
        .lane2_RX1_DCDL_CTRL                            (lane2_RX1_DCDL_CTRL),
        .lane2_RX2_DCDL_CTRL                            (lane2_RX2_DCDL_CTRL),
        .lane2_RX3_DCDL_CTRL                            (lane2_RX3_DCDL_CTRL),
        .lane2_RX4_DCDL_CTRL                            (lane2_RX4_DCDL_CTRL),
        .lane2_RX5_DCDL_CTRL                            (lane2_RX5_DCDL_CTRL),
        .lane2_RX6_DCDL_CTRL                            (lane2_RX6_DCDL_CTRL),
        .lane2_RX7_DCDL_CTRL                            (lane2_RX7_DCDL_CTRL),
        .lane2_RX8_DCDL_CTRL                            (lane2_RX8_DCDL_CTRL),
        .lane2_RX9_DCDL_CTRL                            (lane2_RX9_DCDL_CTRL),
        .lane2_RX10_DCDL_CTRL                           (lane2_RX10_DCDL_CTRL),
        .lane2_RX11_DCDL_CTRL                           (lane2_RX11_DCDL_CTRL),
        .lane2_RX12_DCDL_CTRL                           (lane2_RX12_DCDL_CTRL),
        .lane2_RX13_DCDL_CTRL                           (lane2_RX13_DCDL_CTRL),
        .lane2_RX14_DCDL_CTRL                           (lane2_RX14_DCDL_CTRL),
        .lane2_RX15_DCDL_CTRL                           (lane2_RX15_DCDL_CTRL),
        .lane2_RX16_DCDL_CTRL                           (lane2_RX16_DCDL_CTRL),
`endif
`ifdef D2D_PHY_TX_LANE3_EN
        .tx_lane3_data0              (tx_lane3_data0),   // 补接（原遗漏 data0，仅 data1~16 已接）
        .tx_lane3_data1              (tx_lane3_data1),
        .tx_lane3_data2              (tx_lane3_data2),
        .tx_lane3_data3              (tx_lane3_data3),
        .tx_lane3_data4              (tx_lane3_data4),
        .tx_lane3_data5              (tx_lane3_data5),
        .tx_lane3_data6              (tx_lane3_data6),
        .tx_lane3_data7              (tx_lane3_data7),
        .tx_lane3_data8              (tx_lane3_data8),
        .tx_lane3_data9              (tx_lane3_data9),
        .tx_lane3_data10             (tx_lane3_data10),
        .tx_lane3_data11             (tx_lane3_data11),
        .tx_lane3_data12             (tx_lane3_data12),
        .tx_lane3_data13             (tx_lane3_data13),
        .tx_lane3_data14             (tx_lane3_data14),
        .tx_lane3_data15             (tx_lane3_data15),
        .tx_lane3_data16             (tx_lane3_data16),
`endif
`ifdef D2D_PHY_RX_LANE3_EN
        .rx_lane3_data0              (rx_lane3_data0),
        .rx_lane3_data1              (rx_lane3_data1),
        .rx_lane3_data2              (rx_lane3_data2),
        .rx_lane3_data3              (rx_lane3_data3),
        .rx_lane3_data4              (rx_lane3_data4),
        .rx_lane3_data5              (rx_lane3_data5),
        .rx_lane3_data6              (rx_lane3_data6),
        .rx_lane3_data7              (rx_lane3_data7),
        .rx_lane3_data8              (rx_lane3_data8),
        .rx_lane3_data9              (rx_lane3_data9),
        .rx_lane3_data10             (rx_lane3_data10),
        .rx_lane3_data11             (rx_lane3_data11),
        .rx_lane3_data12             (rx_lane3_data12),
        .rx_lane3_data13             (rx_lane3_data13),
        .rx_lane3_data14             (rx_lane3_data14),
        .rx_lane3_data15             (rx_lane3_data15),
        .rx_lane3_data16             (rx_lane3_data16),
        .lane3_RX_OFFSET_CALIBRATION_ON                 (lane3_RX_OFFSET_CALIBRATION_ON),
        .lane3_OFFSET_CTRL_CLK                          (lane3_OFFSET_CTRL_CLK),
        .lane3_OFFSET_RESET_N                           (lane3_OFFSET_RESET_N),
        .lane3_RX0_OFFSET_CTRL_WRITE                    (lane3_RX0_OFFSET_CTRL_WRITE),
        .lane3_RX0_OFFSET_CTRL_ADDR                     (lane3_RX0_OFFSET_CTRL_ADDR),
        .lane3_RX0_OFFSET_CTRL_DATA                     (lane3_RX0_OFFSET_CTRL_DATA),
        .lane3_RX1_OFFSET_CTRL_WRITE                    (lane3_RX1_OFFSET_CTRL_WRITE),
        .lane3_RX1_OFFSET_CTRL_ADDR                     (lane3_RX1_OFFSET_CTRL_ADDR),
        .lane3_RX1_OFFSET_CTRL_DATA                     (lane3_RX1_OFFSET_CTRL_DATA),
        .lane3_RX2_OFFSET_CTRL_WRITE                    (lane3_RX2_OFFSET_CTRL_WRITE),
        .lane3_RX2_OFFSET_CTRL_ADDR                     (lane3_RX2_OFFSET_CTRL_ADDR),
        .lane3_RX2_OFFSET_CTRL_DATA                     (lane3_RX2_OFFSET_CTRL_DATA),
        .lane3_RX3_OFFSET_CTRL_WRITE                    (lane3_RX3_OFFSET_CTRL_WRITE),
        .lane3_RX3_OFFSET_CTRL_ADDR                     (lane3_RX3_OFFSET_CTRL_ADDR),
        .lane3_RX3_OFFSET_CTRL_DATA                     (lane3_RX3_OFFSET_CTRL_DATA),
        .lane3_RX4_OFFSET_CTRL_WRITE                    (lane3_RX4_OFFSET_CTRL_WRITE),
        .lane3_RX4_OFFSET_CTRL_ADDR                     (lane3_RX4_OFFSET_CTRL_ADDR),
        .lane3_RX4_OFFSET_CTRL_DATA                     (lane3_RX4_OFFSET_CTRL_DATA),
        .lane3_RX5_OFFSET_CTRL_WRITE                    (lane3_RX5_OFFSET_CTRL_WRITE),
        .lane3_RX5_OFFSET_CTRL_ADDR                     (lane3_RX5_OFFSET_CTRL_ADDR),
        .lane3_RX5_OFFSET_CTRL_DATA                     (lane3_RX5_OFFSET_CTRL_DATA),
        .lane3_RX6_OFFSET_CTRL_WRITE                    (lane3_RX6_OFFSET_CTRL_WRITE),
        .lane3_RX6_OFFSET_CTRL_ADDR                     (lane3_RX6_OFFSET_CTRL_ADDR),
        .lane3_RX6_OFFSET_CTRL_DATA                     (lane3_RX6_OFFSET_CTRL_DATA),
        .lane3_RX7_OFFSET_CTRL_WRITE                    (lane3_RX7_OFFSET_CTRL_WRITE),
        .lane3_RX7_OFFSET_CTRL_ADDR                     (lane3_RX7_OFFSET_CTRL_ADDR),
        .lane3_RX7_OFFSET_CTRL_DATA                     (lane3_RX7_OFFSET_CTRL_DATA),
        .lane3_RX8_OFFSET_CTRL_WRITE                    (lane3_RX8_OFFSET_CTRL_WRITE),
        .lane3_RX8_OFFSET_CTRL_ADDR                     (lane3_RX8_OFFSET_CTRL_ADDR),
        .lane3_RX8_OFFSET_CTRL_DATA                     (lane3_RX8_OFFSET_CTRL_DATA),
        .lane3_RX9_OFFSET_CTRL_WRITE                    (lane3_RX9_OFFSET_CTRL_WRITE),
        .lane3_RX9_OFFSET_CTRL_ADDR                     (lane3_RX9_OFFSET_CTRL_ADDR),
        .lane3_RX9_OFFSET_CTRL_DATA                     (lane3_RX9_OFFSET_CTRL_DATA),
        .lane3_RX10_OFFSET_CTRL_WRITE                   (lane3_RX10_OFFSET_CTRL_WRITE),
        .lane3_RX10_OFFSET_CTRL_ADDR                    (lane3_RX10_OFFSET_CTRL_ADDR),
        .lane3_RX10_OFFSET_CTRL_DATA                    (lane3_RX10_OFFSET_CTRL_DATA),
        .lane3_RX11_OFFSET_CTRL_WRITE                   (lane3_RX11_OFFSET_CTRL_WRITE),
        .lane3_RX11_OFFSET_CTRL_ADDR                    (lane3_RX11_OFFSET_CTRL_ADDR),
        .lane3_RX11_OFFSET_CTRL_DATA                    (lane3_RX11_OFFSET_CTRL_DATA),
        .lane3_RX12_OFFSET_CTRL_WRITE                   (lane3_RX12_OFFSET_CTRL_WRITE),
        .lane3_RX12_OFFSET_CTRL_ADDR                    (lane3_RX12_OFFSET_CTRL_ADDR),
        .lane3_RX12_OFFSET_CTRL_DATA                    (lane3_RX12_OFFSET_CTRL_DATA),
        .lane3_RX13_OFFSET_CTRL_WRITE                   (lane3_RX13_OFFSET_CTRL_WRITE),
        .lane3_RX13_OFFSET_CTRL_ADDR                    (lane3_RX13_OFFSET_CTRL_ADDR),
        .lane3_RX13_OFFSET_CTRL_DATA                    (lane3_RX13_OFFSET_CTRL_DATA),
        .lane3_RX14_OFFSET_CTRL_WRITE                   (lane3_RX14_OFFSET_CTRL_WRITE),
        .lane3_RX14_OFFSET_CTRL_ADDR                    (lane3_RX14_OFFSET_CTRL_ADDR),
        .lane3_RX14_OFFSET_CTRL_DATA                    (lane3_RX14_OFFSET_CTRL_DATA),
        .lane3_RX15_OFFSET_CTRL_WRITE                   (lane3_RX15_OFFSET_CTRL_WRITE),
        .lane3_RX15_OFFSET_CTRL_ADDR                    (lane3_RX15_OFFSET_CTRL_ADDR),
        .lane3_RX15_OFFSET_CTRL_DATA                    (lane3_RX15_OFFSET_CTRL_DATA),
        .lane3_RX16_OFFSET_CTRL_WRITE                   (lane3_RX16_OFFSET_CTRL_WRITE),
        .lane3_RX16_OFFSET_CTRL_ADDR                    (lane3_RX16_OFFSET_CTRL_ADDR),
        .lane3_RX16_OFFSET_CTRL_DATA                    (lane3_RX16_OFFSET_CTRL_DATA),
        .lane3_RX_VREF_CTRL_MAIN                        (lane3_RX_VREF_CTRL_MAIN),
        .lane3_RX_VREF_CTRL_ERR_EDGE                    (lane3_RX_VREF_CTRL_ERR_EDGE),
        .lane3_PH_I_GRAY_PI                             (lane3_PH_I_GRAY_PI),
        .lane3_PH_Q_GRAY_PI                             (lane3_PH_Q_GRAY_PI),
        .lane3_PH_045_GRAY_PI                           (lane3_PH_045_GRAY_PI),
        .lane3_RX0_DCDL_CTRL                            (lane3_RX0_DCDL_CTRL),
        .lane3_RX1_DCDL_CTRL                            (lane3_RX1_DCDL_CTRL),
        .lane3_RX2_DCDL_CTRL                            (lane3_RX2_DCDL_CTRL),
        .lane3_RX3_DCDL_CTRL                            (lane3_RX3_DCDL_CTRL),
        .lane3_RX4_DCDL_CTRL                            (lane3_RX4_DCDL_CTRL),
        .lane3_RX5_DCDL_CTRL                            (lane3_RX5_DCDL_CTRL),
        .lane3_RX6_DCDL_CTRL                            (lane3_RX6_DCDL_CTRL),
        .lane3_RX7_DCDL_CTRL                            (lane3_RX7_DCDL_CTRL),
        .lane3_RX8_DCDL_CTRL                            (lane3_RX8_DCDL_CTRL),
        .lane3_RX9_DCDL_CTRL                            (lane3_RX9_DCDL_CTRL),
        .lane3_RX10_DCDL_CTRL                           (lane3_RX10_DCDL_CTRL),
        .lane3_RX11_DCDL_CTRL                           (lane3_RX11_DCDL_CTRL),
        .lane3_RX12_DCDL_CTRL                           (lane3_RX12_DCDL_CTRL),
        .lane3_RX13_DCDL_CTRL                           (lane3_RX13_DCDL_CTRL),
        .lane3_RX14_DCDL_CTRL                           (lane3_RX14_DCDL_CTRL),
        .lane3_RX15_DCDL_CTRL                           (lane3_RX15_DCDL_CTRL),
        .lane3_RX16_DCDL_CTRL                           (lane3_RX16_DCDL_CTRL),
`endif

        // SBC / PLL
        .sb_txd                      (sb_txd),
        .sb_rxd                      (sb_rxd),
        .pll_lock                    (pll_lock),
        // AHB / APB slave（MCU_SUBSYS 二选一）
`ifdef MCU_SUBSYS
        .ahbs_hsel                  (ahbs_hsel),
        .ahbs_haddr                 (ahbs_haddr),
        .ahbs_hburst                (ahbs_hburst),
        .ahbs_hsize                 (ahbs_hsize),
        .ahbs_hprot                 (ahbs_hprot),
        .ahbs_hwdata                (ahbs_hwdata),
        .ahbs_hwrite                (ahbs_hwrite),
        .ahbs_htrans                (ahbs_htrans),
        .ahbs_hready                (ahbs_hready),
        .ahbs_hrdata                (ahbs_hrdata),
        .ahbs_hresp                 (ahbs_hresp),
        .ahbs_hready_resp           (ahbs_hready_resp),
`else
        .hipi_apb_psel              (hipi_apb_psel),
        .hipi_apb_penable           (hipi_apb_penable),
        .hipi_apb_pwrite            (hipi_apb_pwrite),
        .hipi_apb_paddr             (hipi_apb_paddr),
        .hipi_apb_pwdata            (hipi_apb_pwdata),
        .hipi_apb_prdata            (hipi_apb_prdata),
        .hipi_apb_pready            (hipi_apb_pready),
        .hipi_apb_pslverr           (hipi_apb_pslverr),
`endif        // APB master → ctrl APB slave（决策①）
        .apb_psel                   (mcu_apb_psel),
        .apb_penable                (mcu_apb_penable),
        .apb_pwrite                 (mcu_apb_pwrite),
        .apb_paddr                  (mcu_apb_paddr),
        .apb_pwdata                 (mcu_apb_pwdata),
        .apb_prdata                 (mcu_apb_prdata),
        .apb_pready                 (mcu_apb_pready),
        .apb_pslverr                (mcu_apb_pslverr),
        // bring-up strap：tie 0 = 32:1 模式（决策④，64:1 需 link 层 CPIF 1024）
        .io_data_width_mode_strap   (1'b0),
        // 中断
        .irq_ext                    (irq_ext)
    );

endmodule

`default_nettype wire
