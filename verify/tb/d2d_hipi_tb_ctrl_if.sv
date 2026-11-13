// ===================================================================
// d2d_hipi_tb_ctrl_if.sv — TB 控制接口（时钟句柄 + 复位 + test_done）
//
// 版本：v1.0 | 日期：2026-10-19 | 作者：OpenHappy Team
//
// 用途：tb_top 拥有本 interface，test（class）通过 virtual 句柄：
//   1. 观察各时钟（用于 wait(@posedge ctrl.aclk) 等）；
//   2. 驱动全部复位（复位由 test 控制，tb_top 不主动驱动，避免双驱动竞态）；
//   3. 置 test_done 通知 tb_top 收尾（$finish）。
//
// 复位分组（与 DUT 端口一一对应，见 spec/d2d_hipi_dig_top_spec.md §3.2）：
//   aresetn  —— AXI 用户侧异步复位（→ 协议层 aresetn，aclk 域）
//   presetn  —— APB 配置域复位（→ ctrl pcsr / logic_phy apb，pclk 域）
//   mb_rst_n —— 主带全局复位（→ logic_phy，mb_clk / phy_rx_clk 域）
//   sb_rst_n —— 边带全局复位（→ logic_phy，sb_tx_clk_in / sb_rx_clk 域）
//   mcu_rst_n—— MCU 域复位（→ logic_phy，pclk 同步域）
//
// 复位释放顺序建议（test 的 reset_dut 任务实现）：
//   先 presetn（APB 可配）→ 再 aresetn（AXI）→ 再 mb/sb/mcu（数据面）。
//   实测若数据面先释放、协议层后释放，link 会因 aresetn 未释放而不推进。
// ===================================================================

`ifndef D2D_HIPI_TB_CTRL_IF_SV
`define D2D_HIPI_TB_CTRL_IF_SV

interface d2d_hipi_tb_ctrl_if (
  input logic pclk,
  input logic mb_clk,
  input logic phy_rx_clk0,
  input logic aclk,
  input logic sb_clk
);

  // ---- 复位（test 驱动，tb_top 初始化为 0）----
  logic aresetn;
  logic presetn;
  logic mb_rst_n;
  logic sb_rst_n;
  logic mcu_rst_n;

  // ---- 结束标志（test 置 1 → tb_top 打印汇总并 $finish）----
  logic test_done;

  // =================================================================
  // 状态机探针（tb_top 从 DUT 内部层级直连，test 只读）
  //
  //   logic_phy 侧（lane0，其余 lane 在 1T1R 配置下 TX_EN=0/RX_EN=0）：
  //     phy_tx_state  ← u_dut.u_logic_phy.u_lane_top_lane0
  //                      /gen_tx_chain/u_tx_chain/u_tx_training_fsm.cur_state
  //                     4bit：0=ST_IDLE 1=TRAIN_OFFSET 2=TRAIN_VREF 3=TRAIN_EQ
  //                           4=TRAIN_PI 5=TRAIN_DCDL 6=TRAIN_REORDER
  //                           7=DATA_CHK 8=ACTIVE 9=ST_ERROR
  //     phy_rx_state  ← u_dut.u_logic_phy.u_lane_top_lane0
  //                      /gen_rx_chain/u_rx_chain/u_rx_training/u_rx_training_fsm.rx_state
  //                     3bit：0=ST_IDLE 1=ST_BRINGUP 2=ST_ACTIVE
  //
  //   link 侧：
  //     lnk_tx_state  ← u_dut.u_hipi_ctrl_top.u_link_layer/u_link_tx_lcmsm.cur_state
  //                     6bit one-hot：000001=CONFIG 000010=PHY_INIT 000100=PHY_TRAIN
  //                                   001000=LINK_TRAIN 010000=NORMAL 100000=LP
  //     lnk_tx_bin    ← 同上 link_state_o（4bit 二进制，0/1/2/3/4/5）
  //     lnk_rx_state  ← u_dut.u_hipi_ctrl_top.u_link_layer/u_link_rx_lcmsm.cur_state
  //                     6bit one-hot（同上编码）
  // =================================================================
  logic [3:0] phy_tx_state;
  logic [2:0] phy_rx_state;
  logic [5:0] lnk_tx_state;
  logic [3:0] lnk_tx_bin;
  logic [5:0] lnk_rx_state;

  // 派生 ACTIVE 判据（组合）
  logic phy_tx_active;   // == 4'd8
  logic phy_rx_active;   // == 3'd2
  logic lnk_tx_active;   // cur_state[4]
  logic lnk_rx_active;   // cur_state[4]

  assign phy_tx_active = (phy_tx_state == 4'd8);
  assign phy_rx_active = (phy_rx_state == 3'd2);
  assign lnk_tx_active = lnk_tx_state[4];
  assign lnk_rx_active = lnk_rx_state[4];

  modport tb (
    input  pclk, mb_clk, phy_rx_clk0, aclk, sb_clk,
    input  phy_tx_state, phy_rx_state, lnk_tx_state, lnk_tx_bin, lnk_rx_state,
    input  phy_tx_active, phy_rx_active, lnk_tx_active, lnk_rx_active,
    output aresetn, presetn, mb_rst_n, sb_rst_n, mcu_rst_n, test_done
  );

  // -------------------------------------------------------------------
  // 探针等待任务（在 interface 内实现，test 直接调用）
  //   timeout_ps：超时门限；超时返回 0，正常返回 1
  // -------------------------------------------------------------------
  task automatic wait_phy_active(input time timeout_ps, output bit ok);
    time t0;
    t0 = $time;
    ok = 1'b0;
    forever begin
      @(posedge mb_clk);
      if (phy_tx_active && phy_rx_active) begin
        ok = 1'b1;
        $display("%0t [PROBE] logic_phy ACTIVE ✅  tx_state=%0d rx_state=%0d (耗时 %0t)",
                 $time, phy_tx_state, phy_rx_state, $time - t0);
        break;
      end
      if (($time - t0) > timeout_ps) begin
        $display("%0t [PROBE] logic_phy ACTIVE 超时 ❌  tx_state=%0d rx_state=%0d",
                 $time, phy_tx_state, phy_rx_state);
        break;
      end
    end
  endtask

  task automatic wait_link_active(input time timeout_ps, output bit ok);
    time t0;
    t0 = $time;
    ok = 1'b0;
    forever begin
      @(posedge mb_clk);
      if (lnk_tx_active && lnk_rx_active) begin
        ok = 1'b1;
        $display("%0t [PROBE] link ACTIVE ✅  tx_cur=%b(bin=%0d) rx_cur=%b (耗时 %0t)",
                 $time, lnk_tx_state, lnk_tx_bin, lnk_rx_state, $time - t0);
        break;
      end
      if (($time - t0) > timeout_ps) begin
        $display("%0t [PROBE] link ACTIVE 超时 ❌  tx_cur=%b(bin=%0d) rx_cur=%b",
                 $time, lnk_tx_state, lnk_tx_bin, lnk_rx_state);
        break;
      end
    end
  endtask

endinterface

`endif  // D2D_HIPI_TB_CTRL_IF_SV
