// =============================================================================
// module name：tx_msb_lsb
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/tx_chain/submods/tx_data_path/mas/tx_msb_lsb_mas.md
// =============================================================================
// 功能概述: TX 数据通路 stage 4：17 physical IO × 64 bit per-IO bit 序翻转（抵消 PCB 反向走线），
//   纯组合 0 cycle，冗余 IO 使用自身 flip 配置
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-06  OpenHappy Team  初始版本：基于 tx_msb_lsb MAS v1.7 生成
// =============================================================================

`timescale 1ns/1ps

module tx_msb_lsb #(
    parameter PHY_IO_NUM = 17,
    parameter IO_WIDTH   = 64
) (
    input  wire                              mb_clk,    // 时序约束参考，本模块无寄存器
    input  wire                              mb_rst_n,  // 复位由下游吸收，本模块无寄存器
    input  wire  [PHY_IO_NUM*IO_WIDTH-1:0]   data_i,
    output wire  [PHY_IO_NUM*IO_WIDTH-1:0]   data_o,
    input  wire  [PHY_IO_NUM-1:0]            msb_lsb_flip_en_tx
);

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    // 无内部信号（纯组合 wire 重排 + 2:1 mux）

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================
    // per physical IO bit 序翻转：
    //   - msb_lsb_flip_en_tx[p] == 1'b1：bit[i] ↔ bit[63-i]
    //   - msb_lsb_flip_en_tx[p] == 1'b0：原序直传
    // 实现方式：每 IO 一个 64-bit 2:1 mux，输入 1 路为原序，1 路为反序（wire 重排）
    // 选择端为 msb_lsb_flip_en_tx[p]（level 信号直通，不打拍）
    //
    // 注：本模块无寄存器，mb_clk / mb_rst_n 仅作时序约束参考端口保留
    // ==========================================================================
    genvar p, b;
    generate
        for (p = 0; p < PHY_IO_NUM; p = p + 1) begin : gen_bit_reverse
            // 反序 wire 重排：bit[i] ↔ bit[63-i]
            wire [IO_WIDTH-1:0] data_reversed;
            for (b = 0; b < IO_WIDTH; b = b + 1) begin : gen_rev_bit
                assign data_reversed[b] = data_i[p*IO_WIDTH + (IO_WIDTH-1-b)];
            end

            // 2:1 mux 选择原序 / 反序
            assign data_o[p*IO_WIDTH +: IO_WIDTH] =
                msb_lsb_flip_en_tx[p] ? data_reversed : data_i[p*IO_WIDTH +: IO_WIDTH];
        end
    endgenerate

endmodule
