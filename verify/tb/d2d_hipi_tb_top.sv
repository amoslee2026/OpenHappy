// ===================================================================
// d2d_hipi_tb_top.sv — D2D HiPi 单 DUT 环回验证环境顶层
//
// 版本：v1.0 | 日期：2026-10-19 | 作者：OpenHappy Team
//
// 拓扑（单 DUT 单组 d2d，主带+边带在 DUT 端口上自环）：
//
//   axi_master_vip ──AW/AR/W──► DUT s0_axi（DUT 内作为 AXI Slave）
//                                    │  协议层 axi4_symm_prcl（1T1R 对称）
//                                    ▼
//                          link_layer / logic_phy（主带 TX）
//                                    │
//                    ★ 主带环回：tx_lane0_dataN → rx_lane0_dataN（TB 直连）
//                                    │
//                          logic_phy（主带 RX）/ link_layer
//                                    ▼
//   axi_slave_vip  ◄──AW/AR/W──  DUT m0_axi（DUT 内作为 AXI Master）
//        │ B/R                          ▲
//        └──────────────────────────────┘
//
// 数据流方向（用户指定）：
//   axi_master_vip → d2d slave → 主带环回 → d2d master → axi_slave_vip
//   响应反方向：axi_slave_vip(B/R) → d2d master → 主带环回 → d2d slave → axi_master_vip
//
// 边带环回：sb_txd → sb_rxd，sb_txr → sb_rxr（TB 直连，串行数据自环）
//
// 时钟（来源 spec/d2d_hipi_dig_top_spec.md §3.1.1，2026-10-12 决策统一 1 GHz）：
//   pclk = mb_clk = phy_rx_clk0 = aclk = 1 GHz（1.0 ns 周期）
//   sb_tx_clk_in = 800 MHz（1.25 ns）；sb_rx_clk 默认同频自由运行（详见 DUT 例化注释）
//
// 复位：由 test（class）通过 ctrl 接口驱动，tb_top 只在 t=0 置 0，不与 test 抢驱动。
//
// 职责划分：
//   tb_top  —— 时钟、接口例化、DUT 例化与环回接线、FSDB、超时、test 选择、收尾
//   test    —— 复位时序、APB 配置、VIP 创建与激励、scoreboard 比对、PASS/FAIL
// ===================================================================

`timescale 1ns/1ps

`ifndef D2D_HIPI_TB_TOP_SV
`define D2D_HIPI_TB_TOP_SV

import d2d_hipi_tb_pkg::*;

module d2d_hipi_tb_top;

  // =================================================================
  // 0. DUT 环回/输出线网（提前声明，供 sb_rx_clk 的 source-sync 形态使用）
  // =================================================================
  wire sb_txd;
  wire sb_txr;
  wire sb_tx_clk;

  // =================================================================
  // 1. 时钟
  // =================================================================
  logic pclk         = 1'b0;
  logic mb_clk       = 1'b0;
  logic phy_rx_clk0  = 1'b0;
  logic aclk         = 1'b0;
  logic sb_clk       = 1'b0;   // = sb_tx_clk_in
  logic sb_rx_clk    = 1'b0;

  // -----------------------------------------------------------------
  // ★ 时钟相位错开（spec d2d_hipi_top_spec §3.4：pclk / mb_clk / aclk
  //   互异步，`set_clock_groups -asynchronous`；TB 里相位完全对齐会掩盖
  //   CDC 问题，也可能让 DUT 内部跨域握手出现"假同拍"竞争）。
  //
  //   本 TB 保留真实频率（pclk=mb_clk=phy_rx_clk0=aclk=1 GHz、
  //   sb=800 MHz），但把各时钟的**起振相位**错开，模拟异步关系：
  //     pclk        : phase 0.000 ns
  //     aclk        : phase 0.230 ns（≈ T/4，避开 0/0.5 的沿）
  //     mb_clk      : phase 0.470 ns（≈ T/2 - 0.03）
  //     phy_rx_clk0 : phase 0.130 ns（独立恢复时钟，随手取一相位）
  //     sb_clk      : phase 0.000 ns（800 MHz，本身即与 1 GHz 异步）
  //     sb_rx_clk   : 同 sb_clk
  //   相位值均落在 1ps 精度网格上，且与 0/0.5/1.0 的采样沿不重合。
  //   如需恢复全对齐（对比实验），编译加 +define+CLK_PHASE_ALIGNED。
  // -----------------------------------------------------------------
  localparam real PCLK_PHASE        = 0.000ns;
`ifdef CLK_PHASE_ALIGNED
  localparam real ACLK_PHASE        = 0.000ns;
  localparam real MB_CLK_PHASE      = 0.000ns;
  localparam real PHY_RX_CLK_PHASE  = 0.000ns;
`else
  localparam real ACLK_PHASE        = 0.230ns;   // 与 pclk 错开 ~T/4
  localparam real MB_CLK_PHASE      = 0.470ns;   // 与 pclk 错开 ~T/2
  localparam real PHY_RX_CLK_PHASE  = 0.130ns;   // RX 恢复域，独立相位
`endif

  // 相位起振：先等对应相位延迟，再进入自由运行
  initial begin #(PCLK_PHASE);       forever #(`D2D_TB_PCLK_HALF)       pclk        = ~pclk;       end
  initial begin #(ACLK_PHASE);       forever #(`D2D_TB_ACLK_HALF)       aclk        = ~aclk;       end
  initial begin #(MB_CLK_PHASE);     forever #(`D2D_TB_MB_CLK_HALF)     mb_clk      = ~mb_clk;     end
  initial begin #(PHY_RX_CLK_PHASE); forever #(`D2D_TB_PHY_RX_CLK_HALF) phy_rx_clk0 = ~phy_rx_clk0; end
  initial begin forever #(`D2D_TB_SB_CLK_HALF) sb_clk = ~sb_clk; end   // 800 MHz

`ifndef SB_RX_FROM_TXCLK
  // 默认：sb_rx_clk 自由运行（800 MHz）。理由见 DUT 例化的 sb_rx_clk 注释。
  initial begin forever #(`D2D_TB_SB_CLK_HALF) sb_rx_clk = ~sb_rx_clk; end
`else
  // 真实 source-sync 形态：对端随路时钟（此处取 DUT 的 sb_tx_clk）。
  // 注意：sb_tx_clk 是门控输出，边带空闲期停低，logic_phy.crg 的 sb_rx 域
  // 复位同步器会因无沿而卡住 → 该模式下必须先等边带开始翻转再撤 sb_rst_n。
  assign sb_rx_clk = sb_tx_clk;
`endif

  // =================================================================
  // 2. 波形 dump（+DUMP_VCD，由 Makefile DUMP=1 注入；GTKWave 读 VCD）
  //    原 VCS 版用 fsdb（$fsdbDumpfile/$fsdbDumpvars/$fsdbDumpMDA），
  //    本服务器改为开源 Verilator + GTKWave，故改用标准 $dumpfile/$dumpvars。
  // =================================================================
`ifdef DUMP_VCD
  initial begin
    string vcd_file;
    if ($value$plusargs("vcd_name=%s", vcd_file))
      $dumpfile(vcd_file);
    else
      $dumpfile("d2d_hipi.vcd");
    $dumpvars(0, d2d_hipi_tb_top);
  end
`endif

  // =================================================================
  // 3. 控制接口（复位 + test_done）
  // =================================================================
  d2d_hipi_tb_ctrl_if ctrl (
    .pclk        (pclk),
    .mb_clk      (mb_clk),
    .phy_rx_clk0 (phy_rx_clk0),
    .aclk        (aclk),
    .sb_clk      (sb_clk)
  );

  // -----------------------------------------------------------------
  // 状态机探针直连（DUT 内部层级 → ctrl 接口，供 test 只读轮询）
  //   层次经波形核对确认：
  //     logic_phy: u_logic_phy / u_lane_top_lane0 / gen_tx_chain / u_tx_chain
  //                / u_tx_training_fsm.cur_state
  //     link:      u_hipi_ctrl_top / u_link_layer / u_link_tx(_rx)_lcmsm.cur_state
  // -----------------------------------------------------------------
  assign ctrl.phy_tx_state = u_dut.u_logic_phy.u_lane_top_lane0.gen_tx_chain.u_tx_chain.u_tx_training_fsm.cur_state;
  assign ctrl.phy_rx_state = u_dut.u_logic_phy.u_lane_top_lane0.gen_rx_chain.u_rx_chain.u_rx_training.u_rx_training_fsm.rx_state;
  assign ctrl.lnk_tx_state = u_dut.u_hipi_ctrl_top.u_link_layer.u_link_tx_lcmsm.cur_state;
  assign ctrl.lnk_tx_bin   = u_dut.u_hipi_ctrl_top.u_link_layer.u_link_tx_lcmsm.link_state_o;
  assign ctrl.lnk_rx_state = u_dut.u_hipi_ctrl_top.u_link_layer.u_link_rx_lcmsm.cur_state;

  initial begin
    ctrl.aresetn   = 1'b0;
    ctrl.presetn   = 1'b0;
    ctrl.mb_rst_n  = 1'b0;
    ctrl.sb_rst_n  = 1'b0;
    ctrl.mcu_rst_n = 1'b0;
    ctrl.test_done = 1'b0;
  end

  // =================================================================
  // 4. AXI / APB 接口例化
  //    s0_axi_if —— DUT slave 口；VIP 当 Master 用（modport master）
  //                 但 interface 本体是双向容器，DUT 直接按信号名连接即可
  //    m0_axi_if —— DUT master 口；VIP 当 Slave 用（modport slave）
  // =================================================================
  axi4_if s0_axi_if (.aclk(aclk), .aresetn(ctrl.aresetn));
  axi4_if m0_axi_if (.aclk(aclk), .aresetn(ctrl.aresetn));

  apb_if #(
    .ADDR_WIDTH (`D2D_TB_APB_ADDR_WIDTH),
    .DATA_WIDTH (`D2D_TB_APB_DATA_WIDTH)
  ) apb_cfg_if (
    .pclk    (pclk),
    .presetn (ctrl.presetn)
  );

  // =================================================================
  // 5. 主带环回线网（17 IO × 64bit，DUT tx_lane0_dataN → rx_lane0_dataN）
  //    必须显式声明 64bit：否则 VCS 按隐式 1-bit wire 处理，
  //    "1-bit expression is connected to 64-bit port"（34 条 warning），
  //    实际只有 bit0 环回、其余 63bit 悬空 → CPIF 数据全错。
  // =================================================================
  wire [63:0] tx_lane0_data0;
  wire [63:0] tx_lane0_data1;
  wire [63:0] tx_lane0_data2;
  wire [63:0] tx_lane0_data3;
  wire [63:0] tx_lane0_data4;
  wire [63:0] tx_lane0_data5;
  wire [63:0] tx_lane0_data6;
  wire [63:0] tx_lane0_data7;
  wire [63:0] tx_lane0_data8;
  wire [63:0] tx_lane0_data9;
  wire [63:0] tx_lane0_data10;
  wire [63:0] tx_lane0_data11;
  wire [63:0] tx_lane0_data12;
  wire [63:0] tx_lane0_data13;
  wire [63:0] tx_lane0_data14;
  wire [63:0] tx_lane0_data15;
  wire [63:0] tx_lane0_data16;

  // =================================================================
  // 6. DUT 例化（主带 + 边带环回在此完成）
  // =================================================================
  // ==========================================================================
  // DUT：d2d_hipi_dig_top
  //   配置 = 协议层 axi4_symm_prcl（1T1R 对称，AXI_ASYM_MODE 未定义）
  //          MASTER_DIE + SLAVE_DIE（非 iodie）/ 物理层仅 lane0（PHY_LANE_* 全关）
  //   主带环回：tx_lane0_data0..16 → rx_lane0_data0..16（逐 IO 直连，DUT 内部环回）
  //   边带环回：sb_txd → sb_rxd，sb_txr → sb_rxr（串行数据自环）
  //   sb_rx_clk：默认 TB 自由运行 800MHz（不用 DUT 门控 sb_tx_clk，避免
  //              空闲期停钟导致 logic_phy.crg 的 sb_rx 域复位同步器卡住）；
  //              +SB_RX_FROM_TXCLK 时改用 DUT sb_tx_clk（真实 source-sync 形态）。
  // ==========================================================================
  d2d_hipi_dig_top u_dut (
        .pclk                                   (pclk),
        .mb_clk                                 (mb_clk),
        .phy_rx_clk0                            (phy_rx_clk0),
        .sb_tx_clk_in                           (sb_clk),
        .sb_tx_clk                              (sb_tx_clk),
        .sb_rx_clk                              (sb_rx_clk),
        .aclk                                   (aclk),
        // 5 复位统一由 ctrl interface（test class 驱动）提供，tb_top 不直驱
        .aresetn                                (ctrl.aresetn),
        .presetn                                (ctrl.presetn),
        .mb_rst_n                               (ctrl.mb_rst_n),
        .sb_rst_n                               (ctrl.sb_rst_n),
        .mcu_rst_n                              (ctrl.mcu_rst_n),
        // APB 配置口（→ apb_master VIP；paddr[16] 选择 phy/ctrl 域）
        .hipi_apb_psel                          (apb_cfg_if.psel),
        .hipi_apb_penable                       (apb_cfg_if.penable),
        .hipi_apb_pwrite                        (apb_cfg_if.pwrite),
        .hipi_apb_paddr                         (apb_cfg_if.paddr),
        .hipi_apb_pwdata                        (apb_cfg_if.pwdata),
        .hipi_apb_prdata                        (apb_cfg_if.prdata),
        .hipi_apb_pready                        (apb_cfg_if.pready),
        .hipi_apb_pslverr                       (apb_cfg_if.pslverr),
        .lp_ctrl_cmd                            (4'h0),
        .lp_ctrl_req                            (1'b0),
        .lp_ctrl_sts                            (),
        .lp_ctrl_ack                            (),
        .s0_axi_awid                            (s0_axi_if.awid),
        .s0_axi_awaddr                          (s0_axi_if.awaddr),
        .s0_axi_awlen                           (s0_axi_if.awlen),
        .s0_axi_awsize                          (s0_axi_if.awsize),
        .s0_axi_awburst                         (s0_axi_if.awburst),
        .s0_axi_awlock                          (s0_axi_if.awlock),
        .s0_axi_awcache                         (s0_axi_if.awcache),
        .s0_axi_awprot                          (s0_axi_if.awprot),
        .s0_axi_awqos                           (s0_axi_if.awqos),
        .s0_axi_awregion                        (s0_axi_if.awregion),
        .s0_axi_awuser                          (s0_axi_if.awuser),
        .s0_axi_awvalid                         (s0_axi_if.awvalid),
        .s0_axi_awready                         (s0_axi_if.awready),
        .s0_axi_arid                            (s0_axi_if.arid),
        .s0_axi_araddr                          (s0_axi_if.araddr),
        .s0_axi_arlen                           (s0_axi_if.arlen),
        .s0_axi_arsize                          (s0_axi_if.arsize),
        .s0_axi_arburst                         (s0_axi_if.arburst),
        .s0_axi_arlock                          (s0_axi_if.arlock),
        .s0_axi_arcache                         (s0_axi_if.arcache),
        .s0_axi_arprot                          (s0_axi_if.arprot),
        .s0_axi_arqos                           (s0_axi_if.arqos),
        .s0_axi_arregion                        (s0_axi_if.arregion),
        .s0_axi_aruser                          (s0_axi_if.aruser),
        .s0_axi_arvalid                         (s0_axi_if.arvalid),
        .s0_axi_arready                         (s0_axi_if.arready),
        .s0_axi_wdata                           (s0_axi_if.wdata),
        .s0_axi_wstrb                           (s0_axi_if.wstrb),
        .s0_axi_wuser                           (s0_axi_if.wuser),
        .s0_axi_wlast                           (s0_axi_if.wlast),
        .s0_axi_wvalid                          (s0_axi_if.wvalid),
        .s0_axi_wready                          (s0_axi_if.wready),
        .s0_axi_rid                             (s0_axi_if.rid),
        .s0_axi_rdata                           (s0_axi_if.rdata),
        .s0_axi_rresp                           (s0_axi_if.rresp),
        .s0_axi_ruser                           (s0_axi_if.ruser),
        .s0_axi_rlast                           (s0_axi_if.rlast),
        .s0_axi_rvalid                          (s0_axi_if.rvalid),
        .s0_axi_rready                          (s0_axi_if.rready),
        .s0_axi_bid                             (s0_axi_if.bid),
        .s0_axi_bresp                           (s0_axi_if.bresp),
        .s0_axi_buser                           (s0_axi_if.buser),
        .s0_axi_bvalid                          (s0_axi_if.bvalid),
        .s0_axi_bready                          (s0_axi_if.bready),
        .m0_axi_awid                            (m0_axi_if.awid),
        .m0_axi_awaddr                          (m0_axi_if.awaddr),
        .m0_axi_awlen                           (m0_axi_if.awlen),
        .m0_axi_awsize                          (m0_axi_if.awsize),
        .m0_axi_awburst                         (m0_axi_if.awburst),
        .m0_axi_awlock                          (m0_axi_if.awlock),
        .m0_axi_awcache                         (m0_axi_if.awcache),
        .m0_axi_awprot                          (m0_axi_if.awprot),
        .m0_axi_awqos                           (m0_axi_if.awqos),
        .m0_axi_awregion                        (m0_axi_if.awregion),
        .m0_axi_awuser                          (m0_axi_if.awuser),
        .m0_axi_awvalid                         (m0_axi_if.awvalid),
        .m0_axi_awready                         (m0_axi_if.awready),
        .m0_axi_arid                            (m0_axi_if.arid),
        .m0_axi_araddr                          (m0_axi_if.araddr),
        .m0_axi_arlen                           (m0_axi_if.arlen),
        .m0_axi_arsize                          (m0_axi_if.arsize),
        .m0_axi_arburst                         (m0_axi_if.arburst),
        .m0_axi_arlock                          (m0_axi_if.arlock),
        .m0_axi_arcache                         (m0_axi_if.arcache),
        .m0_axi_arprot                          (m0_axi_if.arprot),
        .m0_axi_arqos                           (m0_axi_if.arqos),
        .m0_axi_arregion                        (m0_axi_if.arregion),
        .m0_axi_aruser                          (m0_axi_if.aruser),
        .m0_axi_arvalid                         (m0_axi_if.arvalid),
        .m0_axi_arready                         (m0_axi_if.arready),
        .m0_axi_wdata                           (m0_axi_if.wdata),
        .m0_axi_wstrb                           (m0_axi_if.wstrb),
        .m0_axi_wuser                           (m0_axi_if.wuser),
        .m0_axi_wlast                           (m0_axi_if.wlast),
        .m0_axi_wvalid                          (m0_axi_if.wvalid),
        .m0_axi_wready                          (m0_axi_if.wready),
        .m0_axi_rid                             (m0_axi_if.rid),
        .m0_axi_rdata                           (m0_axi_if.rdata),
        .m0_axi_rresp                           (m0_axi_if.rresp),
        .m0_axi_ruser                           (m0_axi_if.ruser),
        .m0_axi_rlast                           (m0_axi_if.rlast),
        .m0_axi_rvalid                          (m0_axi_if.rvalid),
        .m0_axi_rready                          (m0_axi_if.rready),
        .m0_axi_bid                             (m0_axi_if.bid),
        .m0_axi_bresp                           (m0_axi_if.bresp),
        .m0_axi_buser                           (m0_axi_if.buser),
        .m0_axi_bvalid                          (m0_axi_if.bvalid),
        .m0_axi_bready                          (m0_axi_if.bready),
        .udi_tx_link0_lane0_data_i              (1024'h0),
        .udi_tx_link0_lane0_data_vld_i          (1'b0),
        .udi_tx_link0_ready_o                   (),
        .udi_rx_link0_lane0_data_o              (),
        .udi_rx_link0_lane0_data_vld_o          (),
        .tx_lane0_data0                         (tx_lane0_data0),
        .tx_lane0_data1                         (tx_lane0_data1),
        .tx_lane0_data2                         (tx_lane0_data2),
        .tx_lane0_data3                         (tx_lane0_data3),
        .tx_lane0_data4                         (tx_lane0_data4),
        .tx_lane0_data5                         (tx_lane0_data5),
        .tx_lane0_data6                         (tx_lane0_data6),
        .tx_lane0_data7                         (tx_lane0_data7),
        .tx_lane0_data8                         (tx_lane0_data8),
        .tx_lane0_data9                         (tx_lane0_data9),
        .tx_lane0_data10                        (tx_lane0_data10),
        .tx_lane0_data11                        (tx_lane0_data11),
        .tx_lane0_data12                        (tx_lane0_data12),
        .tx_lane0_data13                        (tx_lane0_data13),
        .tx_lane0_data14                        (tx_lane0_data14),
        .tx_lane0_data15                        (tx_lane0_data15),
        .tx_lane0_data16                        (tx_lane0_data16),
        .rx_lane0_data0                         (tx_lane0_data0),
        .rx_lane0_data1                         (tx_lane0_data1),
        .rx_lane0_data2                         (tx_lane0_data2),
        .rx_lane0_data3                         (tx_lane0_data3),
        .rx_lane0_data4                         (tx_lane0_data4),
        .rx_lane0_data5                         (tx_lane0_data5),
        .rx_lane0_data6                         (tx_lane0_data6),
        .rx_lane0_data7                         (tx_lane0_data7),
        .rx_lane0_data8                         (tx_lane0_data8),
        .rx_lane0_data9                         (tx_lane0_data9),
        .rx_lane0_data10                        (tx_lane0_data10),
        .rx_lane0_data11                        (tx_lane0_data11),
        .rx_lane0_data12                        (tx_lane0_data12),
        .rx_lane0_data13                        (tx_lane0_data13),
        .rx_lane0_data14                        (tx_lane0_data14),
        .rx_lane0_data15                        (tx_lane0_data15),
        .rx_lane0_data16                        (tx_lane0_data16),
        .lane0_RX_OFFSET_CALIBRATION_ON         (),
        .lane0_OFFSET_CTRL_CLK                  (),
        .lane0_OFFSET_RESET_N                   (),
        .lane0_RX0_OFFSET_CTRL_WRITE            (),
        .lane0_RX0_OFFSET_CTRL_ADDR             (),
        .lane0_RX0_OFFSET_CTRL_DATA             (),
        .lane0_RX1_OFFSET_CTRL_WRITE            (),
        .lane0_RX1_OFFSET_CTRL_ADDR             (),
        .lane0_RX1_OFFSET_CTRL_DATA             (),
        .lane0_RX2_OFFSET_CTRL_WRITE            (),
        .lane0_RX2_OFFSET_CTRL_ADDR             (),
        .lane0_RX2_OFFSET_CTRL_DATA             (),
        .lane0_RX3_OFFSET_CTRL_WRITE            (),
        .lane0_RX3_OFFSET_CTRL_ADDR             (),
        .lane0_RX3_OFFSET_CTRL_DATA             (),
        .lane0_RX4_OFFSET_CTRL_WRITE            (),
        .lane0_RX4_OFFSET_CTRL_ADDR             (),
        .lane0_RX4_OFFSET_CTRL_DATA             (),
        .lane0_RX5_OFFSET_CTRL_WRITE            (),
        .lane0_RX5_OFFSET_CTRL_ADDR             (),
        .lane0_RX5_OFFSET_CTRL_DATA             (),
        .lane0_RX6_OFFSET_CTRL_WRITE            (),
        .lane0_RX6_OFFSET_CTRL_ADDR             (),
        .lane0_RX6_OFFSET_CTRL_DATA             (),
        .lane0_RX7_OFFSET_CTRL_WRITE            (),
        .lane0_RX7_OFFSET_CTRL_ADDR             (),
        .lane0_RX7_OFFSET_CTRL_DATA             (),
        .lane0_RX8_OFFSET_CTRL_WRITE            (),
        .lane0_RX8_OFFSET_CTRL_ADDR             (),
        .lane0_RX8_OFFSET_CTRL_DATA             (),
        .lane0_RX9_OFFSET_CTRL_WRITE            (),
        .lane0_RX9_OFFSET_CTRL_ADDR             (),
        .lane0_RX9_OFFSET_CTRL_DATA             (),
        .lane0_RX10_OFFSET_CTRL_WRITE           (),
        .lane0_RX10_OFFSET_CTRL_ADDR            (),
        .lane0_RX10_OFFSET_CTRL_DATA            (),
        .lane0_RX11_OFFSET_CTRL_WRITE           (),
        .lane0_RX11_OFFSET_CTRL_ADDR            (),
        .lane0_RX11_OFFSET_CTRL_DATA            (),
        .lane0_RX12_OFFSET_CTRL_WRITE           (),
        .lane0_RX12_OFFSET_CTRL_ADDR            (),
        .lane0_RX12_OFFSET_CTRL_DATA            (),
        .lane0_RX13_OFFSET_CTRL_WRITE           (),
        .lane0_RX13_OFFSET_CTRL_ADDR            (),
        .lane0_RX13_OFFSET_CTRL_DATA            (),
        .lane0_RX14_OFFSET_CTRL_WRITE           (),
        .lane0_RX14_OFFSET_CTRL_ADDR            (),
        .lane0_RX14_OFFSET_CTRL_DATA            (),
        .lane0_RX15_OFFSET_CTRL_WRITE           (),
        .lane0_RX15_OFFSET_CTRL_ADDR            (),
        .lane0_RX15_OFFSET_CTRL_DATA            (),
        .lane0_RX16_OFFSET_CTRL_WRITE           (),
        .lane0_RX16_OFFSET_CTRL_ADDR            (),
        .lane0_RX16_OFFSET_CTRL_DATA            (),
        .lane0_RX_VREF_CTRL_MAIN                (),
        .lane0_RX_VREF_CTRL_ERR_EDGE            (),
        .lane0_PH_I_GRAY_PI                     (),
        .lane0_PH_Q_GRAY_PI                     (),
        .lane0_PH_045_GRAY_PI                   (),
        .lane0_RX0_DCDL_CTRL                    (),
        .lane0_RX1_DCDL_CTRL                    (),
        .lane0_RX2_DCDL_CTRL                    (),
        .lane0_RX3_DCDL_CTRL                    (),
        .lane0_RX4_DCDL_CTRL                    (),
        .lane0_RX5_DCDL_CTRL                    (),
        .lane0_RX6_DCDL_CTRL                    (),
        .lane0_RX7_DCDL_CTRL                    (),
        .lane0_RX8_DCDL_CTRL                    (),
        .lane0_RX9_DCDL_CTRL                    (),
        .lane0_RX10_DCDL_CTRL                   (),
        .lane0_RX11_DCDL_CTRL                   (),
        .lane0_RX12_DCDL_CTRL                   (),
        .lane0_RX13_DCDL_CTRL                   (),
        .lane0_RX14_DCDL_CTRL                   (),
        .lane0_RX15_DCDL_CTRL                   (),
        .lane0_RX16_DCDL_CTRL                   (),
        .sb_txd                                 (sb_txd),
        .sb_rxd                                 (sb_txd),
        .pll_lock                               (1'b1),
        .dfx_stat_o                             (),
        .irq_ext                                (),
        .sb_txr                                 (sb_txr),
        .sb_rxr                                 (sb_txr),
        .lane0_TRX_REG1_WRITE_IN                (),
        .lane0_TRX_REG2_WRITE_IN                (),
        .lane0_TRX_TEST_REG1_WRITE_IN           (),
        .lane0_TRX_TEST_REG2_WRITE_IN           (),
        .lane0_RX_CLK_ENG_REG1_WRITE_IN         (),
        .lane0_RX_CLK_ENG_REG2_WRITE_IN         (),
        .lane0_RX_CLK_ENG_REG3_WRITE_IN         (),
        .lane0_RX_CLK_ENG_REG4_WRITE_IN         (),
        .lane0_RX_CLK_REG1_WRITE_IN             (),
        .lane0_RX_CLK_REG2_WRITE_IN             (),
        .lane0_RX_CLK_REG3_WRITE_IN             (),
        .lane0_RX_CLK_REG4_WRITE_IN             (),
        .lane0_RX_DATA_REG1_WRITE_IN            (),
        .lane0_RX_DATA_REG2_WRITE_IN            (),
        .lane0_RX_DATA_REG3_WRITE_IN            (),
        .lane0_RX_LDO_REG1_WRITE_IN             (),
        .lane0_RX_LDO_REG2_WRITE_IN             (),
        .lane0_TX_CLK_ENG_REG1_WRITE_IN         (),
        .lane0_TX_CLK_ENG_REG2_WRITE_IN         (),
        .lane0_TX_CLK_ENG_REG3_WRITE_IN         (),
        .lane0_TX_CLK_ENG_REG4_WRITE_IN         (),
        .lane0_TX_CLK_REG1_WRITE_IN             (),
        .lane0_TX_CLK_REG2_WRITE_IN             (),
        .lane0_TX_CLK_REG3_WRITE_IN             (),
        .lane0_TX_CLK_REG4_WRITE_IN             (),
        .lane0_TX_CLK_REG5_WRITE_IN             (),
        .lane0_TX_DATA_REG1_WRITE_IN            (),
        .lane0_TX_DATA_REG2_WRITE_IN            (),
        .lane0_TX_DATA_REG3_WRITE_IN            (),
        .lane0_TX_DATA_REG4_WRITE_IN            (),
        .lane0_TX_DATA_REG5_WRITE_IN            (),
        .lane0_TX_DATA_REG6_WRITE_IN            (),
        .lane0_TX_DATA_REG7_WRITE_IN            (),
        .lane0_TX_LDO_REG1_WRITE_IN             (),
        .lane0_TX_LDO_REG2_WRITE_IN             (),
        .lane1_TRX_REG1_WRITE_IN                (),
        .lane1_TRX_REG2_WRITE_IN                (),
        .lane1_TRX_TEST_REG1_WRITE_IN           (),
        .lane1_TRX_TEST_REG2_WRITE_IN           (),
        .lane2_TRX_REG1_WRITE_IN                (),
        .lane2_TRX_REG2_WRITE_IN                (),
        .lane2_TRX_TEST_REG1_WRITE_IN           (),
        .lane2_TRX_TEST_REG2_WRITE_IN           (),
        .lane3_TRX_REG1_WRITE_IN                (),
        .lane3_TRX_REG2_WRITE_IN                (),
        .lane3_TRX_TEST_REG1_WRITE_IN           (),
        .lane3_TRX_TEST_REG2_WRITE_IN           (),
        .lane0_TX_EN_ASYNC                      (),
        .lane0_TX_EN_SYNC                       (),
        .lane0_TX_EN_TRACK                      (),
        .lane0_TX_RSTB_SYNC                     (),
        .lane0_RX_CLK_EN_ASYNC                  (),
        .lane0_RX_CLK_EN_SYNC                   (),
        .lane0_RX_CLK_EN_TRACK                  (),
        .lane0_RX_RSTB_SYNC                     (),
        .lane1_TX_EN_ASYNC                      (),
        .lane1_TX_EN_SYNC                       (),
        .lane1_TX_EN_TRACK                      (),
        .lane1_TX_RSTB_SYNC                     (),
        .lane1_RX_CLK_EN_ASYNC                  (),
        .lane1_RX_CLK_EN_SYNC                   (),
        .lane1_RX_CLK_EN_TRACK                  (),
        .lane1_RX_RSTB_SYNC                     (),
        .lane2_TX_EN_ASYNC                      (),
        .lane2_TX_EN_SYNC                       (),
        .lane2_TX_EN_TRACK                      (),
        .lane2_TX_RSTB_SYNC                     (),
        .lane2_RX_CLK_EN_ASYNC                  (),
        .lane2_RX_CLK_EN_SYNC                   (),
        .lane2_RX_CLK_EN_TRACK                  (),
        .lane2_RX_RSTB_SYNC                     (),
        .lane3_TX_EN_ASYNC                      (),
        .lane3_TX_EN_SYNC                       (),
        .lane3_TX_EN_TRACK                      (),
        .lane3_TX_RSTB_SYNC                     (),
        .lane3_RX_CLK_EN_ASYNC                  (),
        .lane3_RX_CLK_EN_SYNC                   (),
        .lane3_RX_CLK_EN_TRACK                  (),
        .lane3_RX_RSTB_SYNC                     (),
        .lane0_RX_CLK_ENG_REG_READ_OUT          (32'h0),
        .lane0_TX_CLK_ENG_REG_READ_OUT          (32'h0),
        .lane0_CONNECT_DIG_OUT                  (1'b0),
        .lane1_RX_CLK_ENG_REG_READ_OUT          (32'h0),
        .lane1_TX_CLK_ENG_REG_READ_OUT          (32'h0),
        .lane1_CONNECT_DIG_OUT                  (1'b0),
        .lane2_RX_CLK_ENG_REG_READ_OUT          (32'h0),
        .lane2_TX_CLK_ENG_REG_READ_OUT          (32'h0),
        .lane2_CONNECT_DIG_OUT                  (1'b0),
        .lane3_RX_CLK_ENG_REG_READ_OUT          (32'h0),
        .lane3_TX_CLK_ENG_REG_READ_OUT          (32'h0),
        .lane3_CONNECT_DIG_OUT                  (1'b0),
        .lane0_DOUT0_ERR_EDGE                   (4'h0),
        .lane0_DOUT1_ERR_EDGE                   (4'h0),
        .lane0_DOUT2_ERR_EDGE                   (4'h0),
        .lane0_DOUT3_ERR_EDGE                   (4'h0),
        .lane0_DOUT4_ERR_EDGE                   (4'h0),
        .lane0_DOUT5_ERR_EDGE                   (4'h0),
        .lane0_DOUT6_ERR_EDGE                   (4'h0),
        .lane0_DOUT7_ERR_EDGE                   (4'h0),
        .lane0_DOUT8_ERR_EDGE                   (4'h0),
        .lane0_DOUT9_ERR_EDGE                   (4'h0),
        .lane0_DOUT10_ERR_EDGE                  (4'h0),
        .lane0_DOUT11_ERR_EDGE                  (4'h0),
        .lane0_DOUT12_ERR_EDGE                  (4'h0),
        .lane0_DOUT13_ERR_EDGE                  (4'h0),
        .lane0_DOUT14_ERR_EDGE                  (4'h0),
        .lane0_DOUT15_ERR_EDGE                  (4'h0),
        .lane0_DOUTRFU_ERR_EDGE                 (4'h0),
        .lane1_DOUT0_ERR_EDGE                   (4'h0),
        .lane1_DOUT1_ERR_EDGE                   (4'h0),
        .lane1_DOUT2_ERR_EDGE                   (4'h0),
        .lane1_DOUT3_ERR_EDGE                   (4'h0),
        .lane1_DOUT4_ERR_EDGE                   (4'h0),
        .lane1_DOUT5_ERR_EDGE                   (4'h0),
        .lane1_DOUT6_ERR_EDGE                   (4'h0),
        .lane1_DOUT7_ERR_EDGE                   (4'h0),
        .lane1_DOUT8_ERR_EDGE                   (4'h0),
        .lane1_DOUT9_ERR_EDGE                   (4'h0),
        .lane1_DOUT10_ERR_EDGE                  (4'h0),
        .lane1_DOUT11_ERR_EDGE                  (4'h0),
        .lane1_DOUT12_ERR_EDGE                  (4'h0),
        .lane1_DOUT13_ERR_EDGE                  (4'h0),
        .lane1_DOUT14_ERR_EDGE                  (4'h0),
        .lane1_DOUT15_ERR_EDGE                  (4'h0),
        .lane1_DOUTRFU_ERR_EDGE                 (4'h0),
        .lane2_DOUT0_ERR_EDGE                   (4'h0),
        .lane2_DOUT1_ERR_EDGE                   (4'h0),
        .lane2_DOUT2_ERR_EDGE                   (4'h0),
        .lane2_DOUT3_ERR_EDGE                   (4'h0),
        .lane2_DOUT4_ERR_EDGE                   (4'h0),
        .lane2_DOUT5_ERR_EDGE                   (4'h0),
        .lane2_DOUT6_ERR_EDGE                   (4'h0),
        .lane2_DOUT7_ERR_EDGE                   (4'h0),
        .lane2_DOUT8_ERR_EDGE                   (4'h0),
        .lane2_DOUT9_ERR_EDGE                   (4'h0),
        .lane2_DOUT10_ERR_EDGE                  (4'h0),
        .lane2_DOUT11_ERR_EDGE                  (4'h0),
        .lane2_DOUT12_ERR_EDGE                  (4'h0),
        .lane2_DOUT13_ERR_EDGE                  (4'h0),
        .lane2_DOUT14_ERR_EDGE                  (4'h0),
        .lane2_DOUT15_ERR_EDGE                  (4'h0),
        .lane2_DOUTRFU_ERR_EDGE                 (4'h0),
        .lane3_DOUT0_ERR_EDGE                   (4'h0),
        .lane3_DOUT1_ERR_EDGE                   (4'h0),
        .lane3_DOUT2_ERR_EDGE                   (4'h0),
        .lane3_DOUT3_ERR_EDGE                   (4'h0),
        .lane3_DOUT4_ERR_EDGE                   (4'h0),
        .lane3_DOUT5_ERR_EDGE                   (4'h0),
        .lane3_DOUT6_ERR_EDGE                   (4'h0),
        .lane3_DOUT7_ERR_EDGE                   (4'h0),
        .lane3_DOUT8_ERR_EDGE                   (4'h0),
        .lane3_DOUT9_ERR_EDGE                   (4'h0),
        .lane3_DOUT10_ERR_EDGE                  (4'h0),
        .lane3_DOUT11_ERR_EDGE                  (4'h0),
        .lane3_DOUT12_ERR_EDGE                  (4'h0),
        .lane3_DOUT13_ERR_EDGE                  (4'h0),
        .lane3_DOUT14_ERR_EDGE                  (4'h0),
        .lane3_DOUT15_ERR_EDGE                  (4'h0),
        .lane3_DOUTRFU_ERR_EDGE                 (4'h0),
        .lane0_TX_LPC_DATA_IN                   (),
        .lane0_RX_LPC_DATA_OUT                  (1'b0),
        .lane1_TX_LPC_DATA_IN                   (),
        .lane2_TX_LPC_DATA_IN                   (),
        .lane3_TX_LPC_DATA_IN                   (),
        .cpif_rx_lpc                            (),
        .hipi_ctrl_intr                         ()
  );

  // =================================================================
  // 6. 超时保护（test 不置 test_done 时打印超时并结束）
  // =================================================================
  initial begin : timeout_watch
    longint unsigned to_ns;
    if (!$value$plusargs("sim_timeout=%d", to_ns)) to_ns = 2_000_000;  // 默认 2 ms
    #(to_ns * 1ns);
    $display("==========================================================");
    $display("%0t [TB_TOP] TIMEOUT: test did not finish in %0d ns", $time, to_ns);
    $display("==========================================================");
    $finish;
  end

  // =================================================================
  // 7. test 选择（TB_TEST_<FEATURE>，由 Makefile 注入）
  // =================================================================
  initial begin
    $display("==========================================================");
    $display(" D2D HiPi DV — single DUT (dig_top) loopback TB");
    $display("   AXI: s0_axi <- axi_master_vip / m0_axi -> axi_slave_vip");
    $display("   main-band loopback: tx_lane0_dataN -> rx_lane0_dataN");
    $display("   sideband  loopback: sb_txd -> sb_rxd, sb_txr -> sb_rxr");
    $display("==========================================================");
  end

  // test module 通过 interface 端口拿到 TB 侧句柄（避免层次化引用，便于复用）
`ifdef TB_TEST_SMOKE_1WR1RD
  d2d_hipi_test_smoke_1wr1rd u_test (
    .ctrl     (ctrl),
    .s0_axi   (s0_axi_if),
    .m0_axi   (m0_axi_if),
    .apb_cfg  (apb_cfg_if)
  );
`else
  // 缺省（未指定 TB_TEST_*）：直接用 smoke 用例，便于单点调试
  d2d_hipi_test_smoke_1wr1rd u_test (
    .ctrl     (ctrl),
    .s0_axi   (s0_axi_if),
    .m0_axi   (m0_axi_if),
    .apb_cfg  (apb_cfg_if)
  );
`endif

  // =================================================================
  // 8. 收尾：test 置 test_done → 打印 PASS/FAIL 汇总 → $finish
  // =================================================================
  initial begin : end_of_test
    wait (ctrl.test_done === 1'b1);
    repeat (50) @(posedge aclk);
    $display("==========================================================");
    if (tb_error_cnt == 0)
      $display(" TEST RESULT: PASS  (checks=%0d, errors=0)", tb_check_cnt);
    else
      $display(" TEST RESULT: FAIL  (checks=%0d, errors=%0d)", tb_check_cnt, tb_error_cnt);
    $display("==========================================================");
    $finish;
  end

  // =================================================================
  // 8b. 数据面阻塞链路诊断（仅调试用）——统计 TX 发信用 / RX 收信用的事件，而非单点快照
  //   tx_crd_emit : p_crd_gen.m_ready & enable 握手指数 = 本端发出的本地信用次数
  //   rx_crd_got  : u_unpack.awcrd_rp0_valid 上升沿计数 = 收到并解出的远端信用次数
  // =================================================================
`ifndef DUMP_VCD
  reg [15:0] diag_tx_crd_emit = 16'd0;
  reg [15:0] diag_rx_crd_got  = 16'd0;
  reg        diag_prev_awcrd_v = 1'b0;
  always @(posedge mb_clk) begin
    // TX 信用发射握手（p_crd_gen.m_ready 且 enable）
    if (u_dut.u_hipi_ctrl_top.u_axi_symm_prcl.i_aw_crdgen.m_ready &&
        u_dut.u_hipi_ctrl_top.u_axi_symm_prcl.credit_en_int)
      diag_tx_crd_emit <= diag_tx_crd_emit + 16'd1;
    // RX 解出远端信用（上升沿）
    if (u_dut.u_hipi_ctrl_top.u_axi_symm_prcl.u_unpack.awcrd_rp0_valid &&
        ~diag_prev_awcrd_v)
      diag_rx_crd_got <= diag_rx_crd_got + 16'd1;
    diag_prev_awcrd_v <= u_dut.u_hipi_ctrl_top.u_axi_symm_prcl.u_unpack.awcrd_rp0_valid;
  end

  initial begin : stall_diag
    #(100_000 * 1ns);   // 采样点 ~100us：事务(≈2.87us)后、203us 测试超时前
    $display("==========================================================");
    $display(" [STALL-DIAG] credit 闭环事件统计（~100us）:");
    $display("   TX 本地信用发射次数      diag_tx_crd_emit = %0d   (>0 说明信用确实发出)", diag_tx_crd_emit);
    $display("   RX 解出远端信用次数      diag_rx_crd_got  = %0d   (==0 说明信用没回来/没解析)", diag_rx_crd_got);
    $display("   i_aw_crd_ctrl.crd_cnt_r  = %0d  loc_ready=%0d",
             u_dut.u_hipi_ctrl_top.u_axi_symm_prcl.i_aw_crd_ctrl.crd_cnt_r,
             u_dut.u_hipi_ctrl_top.u_axi_symm_prcl.i_aw_crd_ctrl.loc_ready);
    $display("   u_unpack: unpack_rxvalid 累计(unpack 收到 flit)=%0d  busy=%0d",
             u_dut.u_hipi_ctrl_top.u_axi_symm_prcl.u_unpack.paif_rxvalid,
             u_dut.u_hipi_ctrl_top.u_axi_symm_prcl.u_unpack.busy);
    $display("==========================================================");
    #(100_000 * 1ns);
  end
`endif

endmodule

`endif  // D2D_HIPI_TB_TOP_SV
