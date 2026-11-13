// =============================================================================
// module name：tx_byte_dispatcher
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/tx_chain/submods/tx_data_path/mas/tx_byte_dispatcher_mas.md
// =============================================================================
// 功能概述: TX 数据通路 stage 1：链路层 1024 bit (128 byte) 纯组合分发为 16 logical IO × 64 bit，
//   支持 64:1/32:1 双模式（32:1 高 32 bit 接 0），byte 序 LSB 小端不可配，无寄存器/FSM
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-06  OpenHappy Team  初始版本：基于 tx_byte_dispatcher MAS v1.6 生成
// =============================================================================

`timescale 1ns/1ps

module tx_byte_dispatcher #(
    parameter DATA_WIDTH = 1024,
    parameter IO_NUM     = 16,
    parameter IO_WIDTH   = 64
) (
    input  wire                  mb_clk,    // 时序约束参考，本模块无寄存器
    input  wire                  mb_rst_n,  // 复位由下游吸收，本模块无寄存器
    input  wire  [DATA_WIDTH-1:0] data_i,
    output wire  [IO_NUM*IO_WIDTH-1:0] data_o,
    input  wire                  io_data_width_mode
);

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    // 无内部信号

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================
    // byte[i] = data_i[i*8 +: 8] (i = 0..127, byte0 在 LSB)
    //
    // 64:1 模式：每 IO 8 byte，byte[k] 在最低 byte（LSB 小端）
    //   data_o[k] = {byte[k+112], byte[k+96], ..., byte[k+16], byte[k]}
    //
    // 32:1 模式：每 IO 4 byte，高 32 bit 接 0
    //   data_o[k] = {32'b0, byte[k+48], byte[k+32], byte[k+16], byte[k]}
    //
    // 注：本模块无寄存器，mb_clk / mb_rst_n 仅作时序约束参考端口保留
    // ==========================================================================

    genvar k;
    generate
        for (k = 0; k < IO_NUM; k = k + 1) begin : gen_dispatch
            // 64:1 模式拼接：8 byte 拼成 64 bit
            wire [63:0] word_64;
            assign word_64 = {
                data_i[((k+112)*8) +: 8],   // bit[63:56]
                data_i[((k+96) *8) +: 8],   // bit[55:48]
                data_i[((k+80) *8) +: 8],   // bit[47:40]
                data_i[((k+64) *8) +: 8],   // bit[39:32]
                data_i[((k+48) *8) +: 8],   // bit[31:24]
                data_i[((k+32) *8) +: 8],   // bit[23:16]
                data_i[((k+16) *8) +: 8],   // bit[15:8]
                data_i[( k    *8) +: 8]    // bit[7:0]
            };

            // 32:1 模式拼接：4 byte 拼成 32 bit，高 32 bit 接 0
            wire [63:0] word_32;
            assign word_32 = {
                32'b0,
                data_i[((k+48) *8) +: 8],   // bit[31:24]
                data_i[((k+32) *8) +: 8],   // bit[23:16]
                data_i[((k+16) *8) +: 8],   // bit[15:8]
                data_i[( k    *8) +: 8]    // bit[7:0]
            };

            assign data_o[k*IO_WIDTH +: IO_WIDTH] = io_data_width_mode ? word_64 : word_32;
        end
    endgenerate

endmodule
