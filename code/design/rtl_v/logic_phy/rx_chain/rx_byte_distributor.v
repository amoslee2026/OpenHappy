// =============================================================================
// module name：rx_byte_distributor
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/rx_chain/submods/rx_data_path/mas/rx_byte_distributor_mas.md
// =============================================================================
// 功能概述: RX 数据通路 stage 5：16 IO × 64 bit 纯组合重组为 1024 bit 链路层数据，
//   per-IO bit-rotate 对齐 + 正/反序拼接选择，byte 序 LSB 小端（与 TX 侧反向对称）
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-06  OpenHappy Team  初始版本：基于 rx_byte_distributor MAS v1.2 生成
// =============================================================================

`timescale 1ns/1ps

module rx_byte_distributor #(
    parameter IO_NUM      = 16,
    parameter IO_WIDTH    = 64,
    parameter DATA_WIDTH = 1024
) (
    input  wire                              phy_rx_clk,  // 时序约束参考，本模块无寄存器
    input  wire                              rx_rst_n,    // 复位由下游吸收，本模块无寄存器
    input  wire  [IO_NUM*IO_WIDTH-1:0]         data_i,
    output wire  [DATA_WIDTH-1:0]              data_o,
    input  wire                              reorder_trigger,
    output wire                              reorder_done,
    input  wire  [95:0]                       reorder_delay_rx,  // 16 × 6 bit per-IO bit-rotate
    input  wire  [3:0]                        reorder_order_rx,
    input  wire                              reorder_cfg_valid,
    input  wire                              io_data_width_mode
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    localparam BYTES_PER_IO_64 = 8;   // 64:1 模式每 IO 8 byte
    localparam BYTES_PER_IO_32 = 4;   // 32:1 模式每 IO 4 byte

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    // per-IO bit-rotate 后数据
    wire [IO_WIDTH-1:0] data_rotate [0:IO_NUM-1];

    // 拼接顺序选择后数据
    wire [IO_WIDTH-1:0] data_ordered [0:IO_NUM-1];

    // 64:1 模式下重组的 1024 bit 数据（正序 / 反序一致，仅在 wiring 阶段区分）
    wire [DATA_WIDTH-1:0] data_assemble_64;
    // 32:1 模式下重组的 1024 bit 数据（低 512 bit 有效，高 512 bit 接 0）
    wire [DATA_WIDTH-1:0] data_assemble_32;

    // 训练态观测输出（reorder_trigger=1 时直驱 data_o）
    //   MAS §3.4：训练态 data_o 切换为 reorder 观测输出
    //   本模块实现 per-IO 原始未对齐数据观测（contiguous per-IO layout）
    //   data_o_observation[k*64 +: 64] = data_i[k]   for k=0..15
    //   注：观测 layout 与 runtime 不同（runtime 为 byte interleaved LSB 小端）
    //   32:1 模式下高 32 bit/IO 由上游置 0，观测输出自然保持 0
    wire [DATA_WIDTH-1:0] data_o_observation;

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // Per-IO bit-rotate 对齐
    //   每 IO 按 reorder_delay_rx[i*6 +: 6] 循环左移 0~63 bit
    //   实现方式：每 bit 1 个 64:1 mux（输入 = P[i][0..63] 循环左移 0~63 位的 64 种结果）
    //   共 16 × 64 = 1024 个 64:1 mux
    // --------------------------------------------------------------------------
    genvar i;
    generate
        for (i = 0; i < IO_NUM; i = i + 1) begin : gen_bit_rotate
            wire [5:0] rotate_amt;
            assign rotate_amt = reorder_delay_rx[i*6 +: 6];

            // 每 bit 1 个 64:1 mux：输入 = {P[i][rot], P[i][rot+1], ...} 共 64 种结果
            // 即 data_rotate[i][b] = P[i][(b + rotate_amt) % 64]
            // 用 always @(*) + for 循环描述 64:1 mux，综合工具展开为 LUT
            reg [IO_WIDTH-1:0] rotate_out;
            integer b;
            integer shift;
            always @(*) begin
                for (b = 0; b < IO_WIDTH; b = b + 1) begin
                    shift = (b + rotate_amt) % IO_WIDTH;
                    rotate_out[b] = data_i[i*IO_WIDTH + shift];
                end
            end
            assign data_rotate[i] = rotate_out;
        end
    endgenerate

    // --------------------------------------------------------------------------
    // 拼接顺序选择
    //   - reorder_order_rx == 4'b0000（正序）：data_ordered[i] = data_rotate[i]
    //   - reorder_order_rx == 4'b0001（反序）：data_ordered[i] = data_rotate[15-i]
    //   - 保留编码（2~15）：按正序处理（避免综合出 X）
    // --------------------------------------------------------------------------
    generate
        for (i = 0; i < IO_NUM; i = i + 1) begin : gen_order_mux
            assign data_ordered[i] = (reorder_order_rx == 4'b0001) ?
                                     data_rotate[IO_NUM-1-i] :
                                     data_rotate[i];
        end
    endgenerate

    // --------------------------------------------------------------------------
    // 反拼接为 128 byte（LSB 小端，与 TX tx_byte_dispatcher 反向对称）
    //   输出为 flat 1024 bit 一维总线
    //
    //   对称关系（与 TX 完全对称）：
    //     TX: byte[k + j*16] = IO[k][j*8 +: 8]   (k=0..15, j=0..7 for 64:1)
    //     RX: data_o[(k + j*16)*8 +: 8] = P_ordered[k][j*8 +: 8]
    //
    //   即 byte 在 1024 bit 流中的位置 = (k + j*IO_NUM)*8
    //     - j=0: byte[k]      来自 IO[k][7:0]
    //     - j=1: byte[k+16]   来自 IO[k][15:8]
    //     - j=2: byte[k+32]   来自 IO[k][23:16]
    //     - ...
    //     - j=7: byte[k+112]  来自 IO[k][63:56]
    //
    //   64:1 模式：j=0..7, byte 范围 0..127（128 byte = 1024 bit）
    //   32:1 模式：j=0..3, byte 范围 0..63 （64 byte = 512 bit, 高 512 bit 接 0）
    //
    //   注：MAS §5.1 伪码公式写作 (k*8 + j) 与本处 (k + j*16) 不一致，
    //       但 MAS §1 明确"byte 序固定 LSB 小端，与 TX 侧 tx_byte_dispatcher 反向对称"，
    //       故以 §1 对称关系为准；MAS 伪码 typo 待 mas-gen skill 触发时同步修正
    // --------------------------------------------------------------------------
    genvar k, j;
    generate
        // 64:1 模式重组
        for (k = 0; k < IO_NUM; k = k + 1) begin : gen_assemble_64_io
            for (j = 0; j < BYTES_PER_IO_64; j = j + 1) begin : gen_assemble_64_byte
                assign data_assemble_64[(k + j*IO_NUM)*8 +: 8] =
                    data_ordered[k][j*8 +: 8];
            end
        end
        // 32:1 模式重组（低 512 bit）+ 高 512 bit 接 0
        for (k = 0; k < IO_NUM; k = k + 1) begin : gen_assemble_32_io
            for (j = 0; j < BYTES_PER_IO_32; j = j + 1) begin : gen_assemble_32_byte
                assign data_assemble_32[(k + j*IO_NUM)*8 +: 8] =
                    data_ordered[k][j*8 +: 8];
            end
        end
        // 高 512 bit 接 0
        assign data_assemble_32[DATA_WIDTH-1:512] = 512'b0;
    endgenerate

    // --------------------------------------------------------------------------
    // 训练态观测输出 wiring（per-IO 原始未对齐数据，contiguous per-IO layout）
    //   MAS §3.4：reorder_trigger=1（训练态）下 data_o 切换为 reorder 观测输出
    //   本模块实现：观测输出 = per-IO 原始 data_i，按 IO 顺序紧密排列
    //     data_o_observation[k*64 +: 64] = data_i[k*64 +: 64]   for k=0..15
    //   注：观测 layout 与 runtime 不同（runtime 为 byte interleaved LSB 小端）
    //       32:1 模式下高 32 bit/IO 由上游置 0，观测输出自然保持 0
    // --------------------------------------------------------------------------
    assign data_o_observation = data_i;

    // --------------------------------------------------------------------------
    // data_o 模式选择 mux：
    //   - reorder_trigger=1（训练态）：data_o = data_o_observation（per-IO 观测输出）
    //   - reorder_trigger=0（runtime）：
    //       io_data_width_mode=1（64:1）：data_o = data_assemble_64
    //       io_data_width_mode=0（32:1）：data_o = data_assemble_32（高 512 bit 为 0）
    // --------------------------------------------------------------------------
    assign data_o = reorder_trigger ? data_o_observation :
                                  (io_data_width_mode ? data_assemble_64 : data_assemble_32);

    // --------------------------------------------------------------------------
    // reorder_done 输出（纯组合 level 信号）
    //   reorder_done = reorder_cfg_valid（参数有效指示直驱）
    //   训练态（reorder_cfg_valid=0）下 reorder_done=0
    //   训练完成（reorder_cfg_valid=1）后 reorder_done=1
    // --------------------------------------------------------------------------
    // 注：reorder_trigger 作为 data_o 模式选择信号（训练态切为观测输出，见上方 mux）
    //     reorder_trigger 与 reorder_done（=reorder_cfg_valid）解耦：
    //     - 训练态：reorder_trigger=1, reorder_cfg_valid=0 → data_o=观测输出, reorder_done=0
    //     - 训练完成：reorder_trigger=0, reorder_cfg_valid=1 → data_o=runtime 重组, reorder_done=1
    assign reorder_done = reorder_cfg_valid;

endmodule
