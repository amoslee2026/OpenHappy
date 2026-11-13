// =============================================================================
// module name：tx_io_mapper
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/tx_chain/submods/tx_data_path/mas/tx_io_mapper_mas.md
// =============================================================================
// 功能概述: TX 数据通路 stage 3：16 logical IO → 17 physical IO 的 repair + reversal 合并单一组合查表
//   （SENTINEL 输出 0），纯组合 0 cycle 流水
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-10-13  OpenHappy Team  去输出寄存改纯组合（1 cycle → 0 cycle）：1 GHz 降频后单级查表
//                       组合放充裕，合并入上游 prbs 输出寄存与下游 msb_lsb 的 1 拍；
//                       TX 端到端 2 → 1 cycle；删 mb_clk / mb_rst_n（无寄存器）；
//                       用户决策（arch_prof 审查通过）。兜底：若后续 1 GHz 单拍仍紧，
//                       可恢复输出寄存（+1 cycle）。
//  2026-08-06  OpenHappy Team  初始版本：基于 tx_io_mapper MAS v1.8 生成
// =============================================================================

`timescale 1ns/1ps

module tx_io_mapper #(
    parameter IO_NUM     = 16,
    parameter PHY_IO_NUM = 17,
    parameter IO_WIDTH   = 64
) (
    input  wire  [IO_NUM*IO_WIDTH-1:0]       data_i,         // 16 logical IO × 64 bit
    output wire  [PHY_IO_NUM*IO_WIDTH-1:0]   data_o,         // 17 physical IO × 64 bit
    input  wire                          reversal_en_tx, // IO reversal 使能
    input  wire  [15:0]                  repair_map_tx,  // bit[3:0]=f, bit[4]=valid, bit[5]=d, bit[15:6] 保留
    input  wire                          io_data_width_mode
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    // SENTINEL：src=16 表示该 physical IO 输出 64'b0（失效位或无失效冗余位）
    localparam [4:0] SENTINEL = 5'd16;

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    // repair_map_tx 字段解码
    wire [3:0]  fail_idx;     // 失效 IO 索引 f（0..15）
    wire        fail_valid;   // 1=有失效，0=无失效
    wire        fail_dir;     // 0=高索引方向移位，1=低索引方向移位

    // 每 physical IO 的 lookup_src（0..15 或 SENTINEL）
    wire [4:0]  lookup_src [0:PHY_IO_NUM-1];

    // 组合查找结果（mux 输出）
    wire [IO_WIDTH-1:0] data_o_next [0:PHY_IO_NUM-1];

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // repair_map_tx 字段解码
    //   bit[3:0] = 失效 IO 索引 f（0..15）
    //   bit[4]   = 无失效哨兵 valid（1=有失效，0=无失效）
    //   bit[5]   = 移位方向 d（0=高索引方向移位，1=低索引方向移位）
    //   bit[15:6]= 保留（硬件忽略）
    // --------------------------------------------------------------------------
    assign fail_idx   = repair_map_tx[3:0];
    assign fail_valid = repair_map_tx[4];
    assign fail_dir   = repair_map_tx[5];

    // --------------------------------------------------------------------------
    // L→P lookup_src 译码（reversal + repair 合并到索引层）
    //   每 physical IO p (0..16) 输出一个 5-bit src（0..15 或 SENTINEL=16）
    //   - 无失效（valid=0）:
    //       lookup_src[p]      = reversal_en_tx ? (15 - p) : p   (for p=0..15)
    //       lookup_src[16]     = SENTINEL（冗余 IO 输出 0）
    //   - 有失效 d=0（高索引移位）:
    //       p < f  : lookup_src[p] = reversal_en_tx ? (15 - p) : p
    //       p == f: lookup_src[p] = SENTINEL（失效位输出 0）
    //       p > f  : lookup_src[p] = reversal_en_tx ? (15 - (p-1)) : (p-1)
    //                (移位，冗余 P[16] = L[15])
    //   - 有失效 d=1（低索引移位）:
    //       p < f  : lookup_src[p] = reversal_en_tx ? (15 - (p+1)) : (p+1)
    //                (移位，冗余 P[16] = L[0])
    //       p == f: lookup_src[p] = SENTINEL（失效位输出 0）
    //       p > f  : lookup_src[p] = reversal_en_tx ? (15 - p) : p   (不移位)
    //                注意：p=16 时若 f<16，src=16 仍属不移位分支，但 15-p=-1 越界，
    //                      实际 p=16 时 src 取 0（承载 L[0] 溢出）
    //   注：reversal_en_tx=1 时 src 已是翻转后的 L 索引；mux 选择 L[src] 即完成 reversal
    // --------------------------------------------------------------------------
    genvar p;
    generate
        for (p = 0; p < PHY_IO_NUM; p = p + 1) begin : gen_lookup_src
            // 默认无失效：p=0..15 取 reversal_en_tx ? (15-p) : p；p=16 取 SENTINEL
            if (p == PHY_IO_NUM - 1) begin : gen_redundant_io
                // 冗余 IO (p=16)
                assign lookup_src[p] = fail_valid ?
                                         (fail_dir ? 5'd0 : 5'd15) :  // d=1 承载 L[0]，d=0 承载 L[15]
                                         SENTINEL;                     // 无失效冗余 IO 输出 0
            end
            else begin : gen_active_io
                // active IO (p=0..15)：运行期条件用组合三元表达式（generate if 仅允许常量表达式）
                wire [4:0] p5;
                wire [4:0] f5;
                wire [4:0] src_no_fail;   // 无失效映射
                wire [4:0] src_shift_dn;  // d=0 高索引移位（p>f 时 src = p-1）
                wire [4:0] src_shift_up;  // d=1 低索引移位（p<f 时 src = p+1）

                assign p5          = p[4:0];
                assign f5          = {1'b0, fail_idx};
                assign src_no_fail = reversal_en_tx ? (5'd15 - p5) : p5;
                assign src_shift_dn = reversal_en_tx ? (5'd15 - (p5 - 5'd1)) : (p5 - 5'd1);
                assign src_shift_up = reversal_en_tx ? (5'd15 - (p5 + 5'd1)) : (p5 + 5'd1);

                assign lookup_src[p] = !fail_valid ? src_no_fail :
                                       (p5 == f5)  ? SENTINEL :              // 失效位输出 0
                                       (fail_dir == 1'b0) ?
                                           // d=0（高索引移位）：p<f 不移位，p>f 移位 p-1
                                           ((p5 < f5) ? src_no_fail : src_shift_dn) :
                                           // d=1（低索引移位）：p<f 移位 p+1，p>f 不移位
                                           ((p5 < f5) ? src_shift_up : src_no_fail);
            end
        end
    endgenerate

    // --------------------------------------------------------------------------
    // 17:1 mux 并行选择（每 physical IO 一个）
    //   - src < 16：P[p] = L[src]（L = data_i[src]）
    //   - src == SENTINEL：P[p] = 64'b0
    //   - 32:1 模式：L 高 32 bit 已为 0，mux 自然传递 0，无需额外门控
    // --------------------------------------------------------------------------
    generate
        for (p = 0; p < PHY_IO_NUM; p = p + 1) begin : gen_mux
            reg [IO_WIDTH-1:0] mux_out;
            integer k;
            always @(*) begin
                mux_out = {IO_WIDTH{1'b0}};  // 默认 0（含 SENTINEL 路径）
                for (k = 0; k < IO_NUM; k = k + 1) begin
                    if (lookup_src[p] == k) begin
                        mux_out = data_i[k*IO_WIDTH +: IO_WIDTH];
                    end
                end
                // SENTINEL 情况保持默认 0（lookup_src[p] == 16 时 for 循环无匹配）
            end
            assign data_o_next[p] = mux_out;
        end
    endgenerate

    // ==========================================================================
    // 输出驱动
    //   纯组合直出（1 GHz 降频后单级查表组合放在 prbs 输出寄存 → msb_lsb → pad 的 1 拍内，
    //   去 TX 侧输出寄存：TX 端到端 2 → 1 cycle。无寄存器，无时钟域）。
    // --------------------------------------------------------------------------
    genvar op;
    generate
        for (op = 0; op < PHY_IO_NUM; op = op + 1) begin : gen_output
            assign data_o[op*IO_WIDTH +: IO_WIDTH] = data_o_next[op];
        end
    endgenerate

endmodule
