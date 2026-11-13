// =============================================================================
// module name：tx_mb_pipe
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/tx_chain/submods/tx_data_path/mas/tx_mb_pipe_mas.md
// =============================================================================
// 功能概述: TX 数据通路 stage 5：logic_phy 与 analog_PHY 边界 1088 bit pipeline 缓冲，
//   MB_PIPE_DEPTH 级纯打拍（默认 0，0 = 直通 bypass）
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-06  OpenHappy Team  初始版本：基于 tx_mb_pipe MAS v1.3 生成
// =============================================================================

`timescale 1ns/1ps

module tx_mb_pipe #(
    parameter PHY_IO_NUM    = 17,
    parameter IO_WIDTH      = 64,
    parameter MB_PIPE_DEPTH = 0
) (
    input  wire                              mb_clk,
    input  wire                              mb_rst_n,
    input  wire  [PHY_IO_NUM*IO_WIDTH-1:0]   data_i,
    output wire  [PHY_IO_NUM*IO_WIDTH-1:0]   data_o
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    localparam DATA_TOTAL_WIDTH = PHY_IO_NUM * IO_WIDTH;  // 1088 bit

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    // pipeline 寄存器链：pipe_reg[0..MB_PIPE_DEPTH-1]
    // 注：Verilog-2005 不支持 reg 数组按非常量索引声明，用 packed 1D 实现多级 pipeline
    reg [DATA_TOTAL_WIDTH-1:0] pipe_reg [0:MB_PIPE_DEPTH-1];

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================
    // pipeline 寄存器链生成
    //   - MB_PIPE_DEPTH == 0：bypass（无寄存器，由 gen_pipe_out 选择 data_i）
    //   - MB_PIPE_DEPTH == 1：单级寄存器
    //   - MB_PIPE_DEPTH >= 2：多级寄存器链
    // 注：bypass 路径不在此处驱动 data_o，统一由 gen_pipe_out 处理（避免多驱动冲突）
    // ==========================================================================
    generate
        if (MB_PIPE_DEPTH == 1) begin : gen_single_stage
            always @(posedge mb_clk or negedge mb_rst_n) begin
                if (!mb_rst_n) begin
                    pipe_reg[0] <= {DATA_TOTAL_WIDTH{1'b0}};
                end
                else begin
                    pipe_reg[0] <= data_i;
                end
            end
        end
        else if (MB_PIPE_DEPTH >= 2) begin : gen_multi_stage
            // 第 1 级
            always @(posedge mb_clk or negedge mb_rst_n) begin
                if (!mb_rst_n) begin
                    pipe_reg[0] <= {DATA_TOTAL_WIDTH{1'b0}};
                end
                else begin
                    pipe_reg[0] <= data_i;
                end
            end
            // 第 2 ~ MB_PIPE_DEPTH-1 级
            genvar i;
            for (i = 1; i < MB_PIPE_DEPTH; i = i + 1) begin : gen_pipe_stage
                always @(posedge mb_clk or negedge mb_rst_n) begin
                    if (!mb_rst_n) begin
                        pipe_reg[i] <= {DATA_TOTAL_WIDTH{1'b0}};
                    end
                    else begin
                        pipe_reg[i] <= pipe_reg[i-1];
                    end
                end
            end
        end
        // MB_PIPE_DEPTH == 0：无寄存器，bypass 由 gen_pipe_out 处理
    endgenerate

    // 输出选择（bypass / pipeline 末级）
    generate
        if (MB_PIPE_DEPTH == 0) begin : gen_bypass_out
            assign data_o = data_i;
        end
        else begin : gen_pipe_out
            assign data_o = pipe_reg[MB_PIPE_DEPTH-1];
        end
    endgenerate

endmodule
