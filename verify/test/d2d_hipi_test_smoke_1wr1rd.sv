// ===================================================================
// d2d_hipi_test_smoke_1wr1rd.sv — 基础 smoke 用例
//
// 版本：v1.0 | 日期：2026-10-19 | 作者：OpenHappy Team
//
// 序列（用户指定的第一支 seq）：
//   1. 复位释放（presetn → aresetn → mb_rst_n → sb_rst_n/mcu_rst_n）
//   2. APB 基础配置（pcsr / link layer 必要的使能）
//   3. axi_master_vip 同时发出 1 笔写请求 + 写数据，和 1 笔读请求
//   4. axi_slave_vip 收到写请求后自动回 BRESP，收到读请求后自动回 RDATA
//   5. scoreboard 检查：
//        - master 发出的请求 == s0_axi 实采（DUT slave 口原样收到）
//        - s0_axi 实采的请求数 == m0_axi 转发出数（主带环回转发）
//        - 响应 B/R 数量与编码正确（回程成功）
//
// PASS 判据：tb_error_cnt == 0 / mst.err_cnt == 0 / slv.err_cnt == 0
//
// 环境 APB 初始化严格按 flow/ip_cfg/init/readme.md §3 Phase 0~4 执行：
//   Phase 0 硬复位/时钟  （reset_dut，aresetn 保持复位）
//   Phase 1 pcsr         0x000 rstn + 0x004 clk_en（1T1R=0x0011）
//   Phase 2 logic_phy    bring-up 校验 / SBC 使能 / 中断解屏蔽 / 训练超时 / 训练模式
//   Phase 3 link_layer   PHY_MODE→CONFIG_DONE→RX_READY→PHY_INIT→LANE_ON→RETRAIN
//   Phase 4 prcl         释放 aresetn（最后）→ credit_en + timeout_en
//            / scb 全 CLEAN
// ===================================================================

`timescale 1ns/1ps

`ifndef D2D_HIPI_TEST_SMOKE_1WR1RD_SV
`define D2D_HIPI_TEST_SMOKE_1WR1RD_SV

import d2d_hipi_tb_pkg::*;

module d2d_hipi_test_smoke_1wr1rd (
  d2d_hipi_tb_ctrl_if    ctrl,
  axi4_if               s0_axi,
  axi4_if               m0_axi,
  apb_if                apb_cfg
);

  d2d_hipi_test_base base;

  // 激励参数（可通过 plusargs 覆盖，便于回归遍历）
  int unsigned wr_len  = 0;
  int unsigned wr_size = 6;
  int unsigned rd_len  = 0;
  int unsigned rd_size = 6;
  logic [63:0] wr_addr = 'h1000;
  logic [63:0] rd_addr = 'h2000;
  logic [9:0]  wr_id   = 10'h0;
  logic [9:0]  rd_id   = 10'h1;

  initial begin
    // ---- plusargs ----
    void'($value$plusargs("WR_LEN=%d",  wr_len));
    void'($value$plusargs("WR_SIZE=%d", wr_size));
    void'($value$plusargs("RD_LEN=%d",  rd_len));
    void'($value$plusargs("RD_SIZE=%d", rd_size));
    void'($value$plusargs("WR_ADDR=%h", wr_addr));
    void'($value$plusargs("RD_ADDR=%h", rd_addr));

    $display("==========================================================");
    $display(" TEST: smoke_1wr1rd  (1 write + 1 read issued concurrently)");
    $display("   WR: id=0x%0h addr=0x%0h len=%0d size=%0d",
             wr_id, wr_addr, wr_len, wr_size);
    $display("   RD: id=0x%0h addr=0x%0h len=%0d size=%0d",
             rd_id, rd_addr, rd_len, rd_size);
    $display("==========================================================");

    // ---- 环境构建 ----
    base = new(ctrl, s0_axi, m0_axi, apb_cfg);
    base.build();
    base.start_background();

    // ---- 复位 ----
    base.reset_dut();

    // =================================================================
    // APB 初始化序列（严格按 flow/ip_cfg/init/readme.md §3 Phase 0~4）
    //   Phase 0 已完成（reset_dut：presetn→mb/sb→mcu，aresetn 保持复位）
    // =================================================================
    base.apb_basic_config();   // Phase 1  pcsr（rstn + clk_en）
    base.phy_init();           // Phase 2  logic_phy（bring-up/SBC/irq/timeout/train_mode）
    base.link_init();          // Phase 3  link_layer（PHY_MODE→cfg_done→rx_ready→phy_init→lane_on→retrain）
    base.esc_prcl_reset();     // Phase 4a 释放 aresetn（协议层用户侧复位，最后）
    base.prcl_config();        // Phase 4b prcl（credit_en + timeout_en）

    // =================================================================
    // ★ 关键：等 link / logic_phy 状态机全部进入 ACTIVE 后，再发 AXI 请求
    //   依据：link_csr 0x0C34[7:4] (link TX FSM, 4=NORMAL)
    //         phy_csr  0x002C[3:0]/[4] (TX train_done / step_state)
    //         phy_csr  0x0034[15:0]    (RX aligned，0xFFFF == RX FSM ACTIVE)
    //   避免在链路未就绪时发请求导致 AXI 超时误判。
    // =================================================================
    base.wait_all_active();

    // ---- 发起 1 笔写 + 1 笔读 ----
    fork
      d2d_hipi_seq_lib::one_write(base, wr_id, wr_addr[AXI_ADDR_W-1:0],
                                  wr_len[AXI_LEN_W-1:0], wr_size[AXI_SIZE_W-1:0],
                                  2'b01, 'hCAFE_0000);
      d2d_hipi_seq_lib::one_read (base, rd_id, rd_addr[AXI_ADDR_W-1:0],
                                  rd_len[AXI_LEN_W-1:0], rd_size[AXI_SIZE_W-1:0],
                                  2'b01);
    join

    // ---- 等数据通路落定 ----
    base.wait_all_done();

    // ---- 比对 + PASS/FAIL ----
    base.check_pass("smoke_1wr1rd");
  end

endmodule

`endif  // D2D_HIPI_TEST_SMOKE_1WR1RD_SV
