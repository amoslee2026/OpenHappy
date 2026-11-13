// =============================================================================
// module name：rx_mb_pipe
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/rx_chain/submods/rx_data_path/mas/rx_mb_pipe_mas.md
// =============================================================================
// 功能概述: RX 数据通路 stage 1:analog_PHY 与 logic_phy 边界 1088 bit pipeline 缓冲，
//   MB_PIPE_DEPTH 级纯打拍（默认 0，0 = 直通 bypass）
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-06  OpenHappy Team  初始版本：基于 rx_mb_pipe MAS v1.3 生成
// =============================================================================

`timescale 1ns/1ps

module rx_mb_pipe #(
    parameter PHY_IO_NUM    = 17,
    parameter IO_WIDTH      = 64,
    parameter MB_PIPE_DEPTH = 0
) (
    input  wire                              phy_rx_clk,
    input  wire                              rx_rst_n,
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
    //   - MB_PIPE_DEPTH == 0：bypass（综合工具会优化为 wire 直通）
    //   - MB_PIPE_DEPTH == 1：单级寄存器
    //   - MB_PIPE_DEPTH >= 2：多级寄存器链
    // 注：generate for 内的 always 必须用独立 always 块描述每级寄存器
    // ==========================================================================
    generate
        if (MB_PIPE_DEPTH == 0) begin : gen_bypass
            // bypass：组合直通（不推荐，时序风险）
            // data_o 在下方输出选择段统一驱动
        end
        else if (MB_PIPE_DEPTH == 1) begin : gen_single_stage
            always @(posedge phy_rx_clk or negedge rx_rst_n) begin
                if (!rx_rst_n) begin
                    pipe_reg[0] <= {DATA_TOTAL_WIDTH{1'b0}};
                end
                else begin
                    pipe_reg[0] <= data_i;
                end
            end
        end
        else begin : gen_multi_stage
            // 第 1 级
            always @(posedge phy_rx_clk or negedge rx_rst_n) begin
                if (!rx_rst_n) begin
                    pipe_reg[0] <= {DATA_TOTAL_WIDTH{1'b0}};
                end
                else begin
                    pipe_reg[0] <= data_i;
                end
            end
            // 第 2 ~ MB_PIPE_DEPTH-1 级
            genvar i;
            for (i = 1; i < MB_PIPE_DEPTH; i = i + 1) begin : gen_pipe_stage
                always @(posedge phy_rx_clk or negedge rx_rst_n) begin
                    if (!rx_rst_n) begin
                        pipe_reg[i] <= {DATA_TOTAL_WIDTH{1'b0}};
                    end
                    else begin
                        pipe_reg[i] <= pipe_reg[i-1];
                    end
                end
            end
        end
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
