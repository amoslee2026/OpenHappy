// =============================================================================
// module name：rx_io_mapper
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/rx_chain/submods/rx_data_path/mas/rx_io_mapper_mas.md
// =============================================================================
// 功能概述: RX 数据通路 stage 3：17 physical IO → 16 logical IO 的 repair + reversal 反向组合查表，
//   输出寄存型 1 cycle 流水
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-06  OpenHappy Team  初始版本：基于 rx_io_mapper MAS v1.4 生成
// =============================================================================

`timescale 1ns/1ps

module rx_io_mapper #(
    parameter IO_NUM     = 16,
    parameter PHY_IO_NUM = 17,
    parameter IO_WIDTH   = 64
) (
    input  wire                              phy_rx_clk,
    input  wire                              rx_rst_n,
    input  wire  [PHY_IO_NUM*IO_WIDTH-1:0]   data_i,      // 17 physical IO × 64 bit
    output wire  [IO_NUM*IO_WIDTH-1:0]       data_o,       // 16 logical IO × 64 bit
    input  wire                              reversal_en_rx,  // IO reversal 使能
    input  wire  [15:0]                       repair_map_rx,   // bit[3:0]=f, bit[4]=valid, bit[5]=d, bit[15:6] 保留
    input  wire                              io_data_width_mode
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    // SENTINEL：lookup_p=16 表示该 logical IO 由冗余 IO P[16] 承载
    //   - repair 激活（fail_valid=1）+ d=1：P[16] 承载 L[0]（无 reversal）或对偶溢出位（reversal）
    //   - repair 激活 + d=0：P[16] 承载 L[15]（无 reversal）
    //   - 无失效（fail_valid=0）：P[16] 不被任何 L 选中（lookup_p 表无 l 指向 16）
    //   注：失效 IO 位置 P[f] 不被任何 L 选中——lookup_p 表中无 l 指向 f
    localparam [4:0] SENTINEL = 5'd16;

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    // repair_map_rx 字段解码
    wire [3:0]  fail_idx;     // 失效 IO 索引 f（0..15）
    wire        fail_valid;   // 1=有失效，0=无失效
    wire        fail_dir;     // 0=高索引方向移位（d=0），1=低索引方向移位（d=1）

    // 每 logical IO 的 lookup_p（0..16，指向承载 L[l] 的 physical IO）
    wire [4:0]  lookup_p [0:IO_NUM-1];

    // 组合查找结果（mux 输出）
    wire [IO_WIDTH-1:0] data_o_next [0:IO_NUM-1];

    // 输出寄存器（packed 1D，规避 Verilog-2005 2D reg 限制）
    reg  [IO_NUM*IO_WIDTH-1:0] data_o_reg;

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // repair_map_rx 字段解码
    //   bit[3:0] = 失效 IO 索引 f（0..15）
    //   bit[4]   = 无失效哨兵 valid（1=有失效，0=无失效）
    //   bit[5]   = 移位方向 d（0=高索引方向移位，1=低索引方向移位）
    //   bit[15:6]= 保留（硬件忽略）
    // --------------------------------------------------------------------------
    assign fail_idx   = repair_map_rx[3:0];
    assign fail_valid = repair_map_rx[4];
    assign fail_dir   = repair_map_rx[5];

    // --------------------------------------------------------------------------
    // P→L lookup_p 译码（reversal + repair 合并到索引层）
    //   每 logical IO l (0..15) 输出一个 5-bit p_src（0..15 或 SENTINEL=16）
    //   - 无失效（valid=0）:
    //       reversal_en_rx=0: lookup_p[l] = l
    //       reversal_en_rx=1: lookup_p[l] = 15 - l
    //       SENTINEL 不出现（P[16] 不被任何 L 选中）
    //   - 有失效 d=0（高索引移位），reversal_en_rx=0:
    //       l < f  : lookup_p[l] = l
    //       l >= f : lookup_p[l] = l + 1   （P[f] 跳过，P[16] 承载原 L[15]）
    //   - 有失效 d=0，reversal_en_rx=1:
    //       (15 - l) < f  : lookup_p[l] = 15 - l
    //       (15 - l) >= f : lookup_p[l] = 15 - l + 1   （失效位 f 的 P 不被选中）
    //   - 有失效 d=1（低索引移位），reversal_en_rx=0:
    //       l == 0 : lookup_p[0] = SENTINEL（冗余 IO P[16] 承载原 L[0] 溢出）
    //       1<=l<=f: lookup_p[l] = l - 1
    //       f<l<=15: lookup_p[l] = l    （不移位）
    //   - 有失效 d=1，reversal_en_rx=1（MAS §3.3 "原理同 d=0"，reversal 在索引层合并）：
    //       p' = 15 - l 为承载 L[l] 的"原始 P 索引"（reversal 后）
    //       d=1 移位规则在 P 域应用：
    //         p' > f  : lookup_p[l] = p'        （不移位分支）
    //         p' == f : lookup_p[l] = SENTINEL  （失效位，L[l] 由冗余 IO P[16] 承载）
    //         p' < f  : lookup_p[l] = p' - 1    （移位分支）
    //       边界：p' = 0（即 l=15）+ d=1 移位分支：p'-1 越界，归一到 SENTINEL
    // --------------------------------------------------------------------------
    genvar l;
    generate
        for (l = 0; l < IO_NUM; l = l + 1) begin : gen_lookup_p
            // 运行期条件用组合三元表达式（generate if 仅允许常量表达式）
            wire [4:0] l5;
            wire [4:0] f5;
            wire [4:0] p_rev;         // reversal 域原始 P 索引 p' = 15 - l
            wire [4:0] src_no_fail;   // 无失效映射
            wire [4:0] src_d0;        // 有失效 d=0（高索引移位）
            wire [4:0] src_d1;        // 有失效 d=1（低索引移位）

            assign l5          = l[4:0];
            assign f5          = {1'b0, fail_idx};
            assign p_rev       = 5'd15 - l[4:0];
            assign src_no_fail = reversal_en_rx ? p_rev : l5;

            // d=0：l<f 不移位，l>=f 移位 l+1（reversal 域：p'<f 不移位，p'>=f 移位 p'+1）
            assign src_d0 = !reversal_en_rx ?
                              ((l5 < f5)    ? l5    : l5 + 5'd1) :
                              ((p_rev < f5) ? p_rev : p_rev + 5'd1);

            // d=1：
            //   无 reversal：l==0 → SENTINEL（冗余 IO 承载 L[0] 溢出）；
            //                1<=l<=f → l-1；l>f → 不移位
            //   reversal（MAS §3.3 "原理同 d=0"，d=1 移位规则在 P 域应用）：
            //     p'>f → p'（不移位）；p'==f → SENTINEL（失效位由冗余 IO 承载）；
            //     p'<f → p'-1（移位），边界 p'=0（l=15）越界归一到 SENTINEL
            assign src_d1 = !reversal_en_rx ?
                              ((l5 == 5'd0) ? SENTINEL  :
                               (l5 <= f5)   ? l5 - 5'd1 : l5) :
                              ((p_rev > f5)  ? p_rev :
                               (p_rev == f5) ? SENTINEL :
                               (l5 == 5'd15) ? SENTINEL : p_rev - 5'd1);

            assign lookup_p[l] = !fail_valid ? src_no_fail :
                                 (fail_dir == 1'b0) ? src_d0 : src_d1;
        end
    endgenerate

    // --------------------------------------------------------------------------
    // 17:1 mux 并行选择（每 logical IO 一个）
    //   - p_src <= 15：L[l] = P[p_src]
    //   - p_src == SENTINEL（16）：L[l] 由冗余 IO P[16] 承载（repair 激活时有效）
    //     注：fail_valid=0 时 lookup_p 不会取 SENTINEL，P[16] 不被选中
    //   - 32:1 模式：P 高 32 bit 已为 0，mux 自然传递 0，无需额外门控
    // --------------------------------------------------------------------------
    generate
        for (l = 0; l < IO_NUM; l = l + 1) begin : gen_mux
            reg [IO_WIDTH-1:0] mux_out;
            integer k;
            always @(*) begin
                mux_out = {IO_WIDTH{1'b0}};  // 默认 0（防御性）
                for (k = 0; k < PHY_IO_NUM; k = k + 1) begin
                    if (lookup_p[l] == k) begin
                        mux_out = data_i[k*IO_WIDTH +: IO_WIDTH];
                    end
                end
            end
            assign data_o_next[l] = mux_out;
        end
    endgenerate

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // 输出寄存器（16 logical IO × 64 bit = 1024 bit）
    //   - 无独立输入寄存（v1.2 改造：去掉 u_input_reg）
    //   - data_o_next 组合直达 u_output_reg 采样
    //   - 32:1 模式高 32 bit/IO 由 mux 输入 P 高 32 bit=0 自然保持为 0
    // --------------------------------------------------------------------------
    always @(posedge phy_rx_clk or negedge rx_rst_n) begin
        integer k;
        if (!rx_rst_n) begin
            data_o_reg <= {(IO_NUM*IO_WIDTH){1'b0}};
        end
        else begin
            for (k = 0; k < IO_NUM; k = k + 1) begin
                data_o_reg[k*IO_WIDTH +: IO_WIDTH] <= data_o_next[k];
            end
        end
    end

    // 输出驱动
    assign data_o = data_o_reg;

endmodule
