// =============================================================================
// module name：data_checker
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/rx_chain/submods/rx_training/mas/data_checker_mas.md（待编写）
// =============================================================================
// 功能概述: data check 比对单元（DATA_CHK 态 data_i vs prbs_seq_i 比对，连续失败阈值
//   DATACHECK_FAIL_THRESHOLD 默认 3）：端口 stub，功能待 MAS 编写后补充
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-09-11  OpenHappy Team  MDE-CHAIN-RERUN-01-271-IMPL 族 c #4 hook_min：新增
//                       FAIL_THRESHOLD_i 输入 + check_req_i 沿采样影子 fail_threshold_shd
//                       （结构消费占位，供 TRAIN-MAS-01 DATACHECK 判定用）
// =============================================================================

`timescale 1ns/1ps

module data_checker #(
    parameter DATA_WIDTH = 1024         // 16 IO × 64 bit
) (
    input  wire                  phy_rx_clk,
    input  wire                  rx_rst_n,
    input  wire                  check_req_i,            // ← rx_training_fsm.data_check_req_o
    input  wire [DATA_WIDTH-1:0] data_i,                 // ← rx_prbs_descrambler.data_o（DATA_CHK 态 bypass 直通）
    input  wire [DATA_WIDTH-1:0] prbs_seq_i,             // ← rx_prbs_descrambler.prbs_seq_o（expected PRBS pattern）
    input  wire                  data_check_pattern_sel, // DATA_CHK_CFG.pattern_sel（0=PRBS / 1=自定义）
    // 族 c #4（MDE-CHAIN-RERUN-01-271-IMPL）：DATACHECK_FAIL_THRESHOLD hook_min 输入（消费待 TRAIN-MAS-01）
    input  wire [7:0]            FAIL_THRESHOLD_i,
    output wire                  check_done_o,           // → rx_training_fsm.data_check_done_i
    output wire                  check_fail_o            // 比对失败指示（连续失败阈值 DATACHECK_FAIL_THRESHOLD 默认 3）
);

    // ==========================================================================
    // stub（MAS 待编写）: check_done_o/check_fail_o 暂驱 0；
    //   族 c #4 hook_min = check_req_i 沿采样 FAIL_THRESHOLD_i 入影子（结构消费，
    //   供 TRAIN-MAS-01 DATACHECK 连续失败判定用）
    // ==========================================================================
    assign check_done_o = 1'b0;
    assign check_fail_o = 1'b0;

    reg [7:0] fail_threshold_shd;
    always @(posedge phy_rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            fail_threshold_shd <= 8'h3;
        end
        else if (check_req_i) begin
            fail_threshold_shd <= FAIL_THRESHOLD_i;
        end
    end

endmodule
