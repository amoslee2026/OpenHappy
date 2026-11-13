// =============================================================================
// module name：rx_data_path
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/rx_chain/submods/rx_data_path/rx_data_path_spec.md（spec 级父模块，无 MAS）
// =============================================================================
// 功能概述: RX 数据通路 6 级流水集成父模块（纯例化连线，无逻辑）：
//   mb_pipe → msb_lsb → io_mapper → prbs → byte_distribute → async_fifo（stage 6 CDC 至 mb_clk）
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-07  OpenHappy Team  初始版本：基于 rx_data_path_spec v1.9 集成生成
// =============================================================================

`timescale 1ns/1ps

module rx_data_path #(
    parameter DATA_WIDTH    = 1024,
    parameter IO_NUM        = 16,
    parameter PHY_IO_NUM    = 17,
    parameter IO_WIDTH      = 64,
    parameter MB_PIPE_DEPTH = 0
) (
    // 时钟与复位
    input  wire                            phy_rx_clk,      // per-lane RX 恢复时钟（前 5 stage）
    input  wire                            rx_rst_n,        // RX 域异步复位，低有效
    input  wire                            mb_clk,          // TX 主带时钟（async_fifo 读侧）
    input  wire                            mb_rst_n,        // mb_clk 域异步复位，低有效

    // 数据通道
    input  wire  [PHY_IO_NUM*IO_WIDTH-1:0] rx_analog_pad,   // analog PHY → stage 1
    output wire  [DATA_WIDTH-1:0]          rx_lane_data,    // stage 6 → 链路层 CPIIF

    // PRBS 观测 / 对齐状态（TDM 共享送 rx_training data_checker / fsm，phy_rx_clk 同域）
    output wire  [IO_NUM*IO_WIDTH-1:0]     prbs_seq_o,
    output wire  [IO_NUM-1:0]              aligned_o,

    // reorder 训练接口（↔ rx_training_fsm，phy_rx_clk 同域）
    input  wire                            reorder_trigger,
    output wire                            reorder_done,
    input  wire  [IO_NUM*6-1:0]            reorder_delay_rx,   // 16 × 6 bit per-IO bit-rotate
    input  wire  [3:0]                     reorder_order_rx,
    input  wire                            reorder_cfg_valid,

    // APB CSR 具名配置（apb_csr_top 解码后直送）
    input  wire                            io_data_width_mode,
    input  wire  [PHY_IO_NUM-1:0]          msb_lsb_flip_en_rx,
    input  wire                            reversal_en_rx,
    input  wire  [15:0]                    repair_map_rx,
    input  wire  [IO_NUM*23-1:0]           prbs_seed_rx,       // 16 × 23 bit
    input  wire  [3:0]                     flip_sel_rx,
    input  wire                            scramble_en,
    input  wire                            self_sync_en,
    input  wire                            seed_load_i,
    input  wire                            clear,

    // rx_async_fifo 控制 / 状态（↔ apb_csr_top）
    input  wire                            fifo_soft_reset,
    input  wire                            overflow_clear,
    input  wire                            underflow_clear,
    output wire  [3:0]                     fifo_status         // {overflow, underflow, full, empty}
);

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    // stage 间串行数据连线（上一级 data_o → 下一级 data_i）
    wire [PHY_IO_NUM*IO_WIDTH-1:0] mb_pipe_out_data;      // stage 1 → stage 2（1088 bit）
    wire [PHY_IO_NUM*IO_WIDTH-1:0] msb_lsb_out_data;      // stage 2 → stage 3（1088 bit）
    wire [IO_NUM*IO_WIDTH-1:0]     io_mapper_out_data;    // stage 3 → stage 4（1024 bit）
    wire [IO_NUM*IO_WIDTH-1:0]     descrambler_out_data;  // stage 4 → stage 5（1024 bit）
    wire [DATA_WIDTH-1:0]          distributor_out_data;  // stage 5 → stage 6（1024 bit）

    // ==========================================================================
    // 子模块实例化
    // ==========================================================================

    // --------------------------------------------------------------------------
    // stage 1：rx_mb_pipe（边界 pipeline，MB_PIPE_DEPTH cycle）
    // --------------------------------------------------------------------------
    rx_mb_pipe #(
        .PHY_IO_NUM    (PHY_IO_NUM),
        .IO_WIDTH      (IO_WIDTH),
        .MB_PIPE_DEPTH (MB_PIPE_DEPTH)
    ) u_rx_mb_pipe (
        .phy_rx_clk(phy_rx_clk),
        .rx_rst_n  (rx_rst_n),
        .data_i    (rx_analog_pad),
        .data_o    (mb_pipe_out_data)
    );

    // --------------------------------------------------------------------------
    // stage 2：rx_msb_lsb（纯组合 0 cycle）
    // --------------------------------------------------------------------------
    rx_msb_lsb #(
        .PHY_IO_NUM (PHY_IO_NUM),
        .IO_WIDTH   (IO_WIDTH)
    ) u_rx_msb_lsb (
        .phy_rx_clk        (phy_rx_clk),
        .rx_rst_n          (rx_rst_n),
        .data_i            (mb_pipe_out_data),
        .data_o            (msb_lsb_out_data),
        .msb_lsb_flip_en_rx(msb_lsb_flip_en_rx)
    );

    // --------------------------------------------------------------------------
    // stage 3：rx_io_mapper（输出寄存型 1 cycle）
    // --------------------------------------------------------------------------
    rx_io_mapper #(
        .IO_NUM     (IO_NUM),
        .PHY_IO_NUM (PHY_IO_NUM),
        .IO_WIDTH   (IO_WIDTH)
    ) u_rx_io_mapper (
        .phy_rx_clk        (phy_rx_clk),
        .rx_rst_n          (rx_rst_n),
        .data_i            (msb_lsb_out_data),
        .data_o            (io_mapper_out_data),
        .reversal_en_rx    (reversal_en_rx),
        .repair_map_rx     (repair_map_rx),
        .io_data_width_mode(io_data_width_mode)
    );

    // --------------------------------------------------------------------------
    // stage 4：rx_prbs_descrambler（输出寄存型 1 cycle）
    // --------------------------------------------------------------------------
    rx_prbs_descrambler #(
        .IO_NUM      (IO_NUM),
        .IO_WIDTH    (IO_WIDTH),
        .STATE_WIDTH (23)
    ) u_rx_prbs_descrambler (
        .phy_rx_clk        (phy_rx_clk),
        .rx_rst_n          (rx_rst_n),
        .data_i            (io_mapper_out_data),
        .data_o            (descrambler_out_data),
        .prbs_seq_o        (prbs_seq_o),
        .aligned_o         (aligned_o),
        .prbs_seed_rx      (prbs_seed_rx),
        .flip_sel_rx       (flip_sel_rx),
        .io_data_width_mode(io_data_width_mode),
        .scramble_en       (scramble_en),
        .self_sync_en      (self_sync_en),
        .seed_load_i       (seed_load_i),
        .clear             (clear)
    );

    // --------------------------------------------------------------------------
    // stage 5：rx_byte_distributor（纯组合 0 cycle）
    // --------------------------------------------------------------------------
    rx_byte_distributor #(
        .IO_NUM     (IO_NUM),
        .IO_WIDTH   (IO_WIDTH),
        .DATA_WIDTH (DATA_WIDTH)
    ) u_rx_byte_distributor (
        .phy_rx_clk        (phy_rx_clk),
        .rx_rst_n          (rx_rst_n),
        .data_i            (descrambler_out_data),
        .data_o            (distributor_out_data),
        .reorder_trigger   (reorder_trigger),
        .reorder_done      (reorder_done),
        .reorder_delay_rx  (reorder_delay_rx),
        .reorder_order_rx  (reorder_order_rx),
        .reorder_cfg_valid (reorder_cfg_valid),
        .io_data_width_mode(io_data_width_mode)
    );

    // --------------------------------------------------------------------------
    // stage 6：rx_async_fifo（CDC phy_rx_clk → mb_clk）
    // --------------------------------------------------------------------------
    rx_async_fifo #(
        .FIFO_DEPTH (8),
        .DATA_WIDTH (DATA_WIDTH),
        .PTR_WIDTH  (4)
    ) u_rx_async_fifo (
        .phy_rx_clk     (phy_rx_clk),
        .mb_clk         (mb_clk),
        .rx_rst_n       (rx_rst_n),
        .mb_rst_n       (mb_rst_n),
        .fifo_soft_reset(fifo_soft_reset),
        .overflow_clear (overflow_clear),
        .underflow_clear(underflow_clear),
        .data_i         (distributor_out_data),
        .data_o         (rx_lane_data),
        .fifo_status    (fifo_status)
    );

endmodule
