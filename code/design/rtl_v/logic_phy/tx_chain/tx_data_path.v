// =============================================================================
// module name：tx_data_path
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/tx_chain/submods/tx_data_path/tx_data_path_spec.md（spec 级父模块，无 MAS）
// =============================================================================
// 功能概述: TX 数据通路 5 级流水集成父模块（纯例化连线，无逻辑）：
//   dispatch → prbs → io_mapper → msb_lsb → mb_pipe，5 stage 全 mb_clk 域，无 CDC
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-07  OpenHappy Team  初始版本：基于 tx_data_path_spec v1.11 集成生成
// =============================================================================

`timescale 1ns/1ps

module tx_data_path #(
    parameter DATA_WIDTH    = 1024,
    parameter IO_NUM        = 16,
    parameter PHY_IO_NUM    = 17,
    parameter IO_WIDTH      = 64,
    parameter MB_PIPE_DEPTH = 0
) (
    // 时钟与复位
    input  wire                            mb_clk,
    input  wire                            mb_rst_n,

    // 数据通道
    input  wire  [DATA_WIDTH-1:0]          tx_lane_data,        // 链路层 CPIIF → stage 1
    output wire  [PHY_IO_NUM*IO_WIDTH-1:0] tx_analog_pad,       // stage 5 → analog PHY

    // 训练 pattern 切换请求（tx_training_fsm one-hot 电平，mb_clk 同域）
    input  wire                            vref_req,
    input  wire                            pi_req,
    input  wire                            dcdl_req,
    input  wire                            reorder_req,
    input  wire                            data_check_req,

    // APB CSR 具名配置（apb_csr_top 解码后直送）
    input  wire                            io_data_width_mode,
    input  wire  [IO_NUM*23-1:0]           prbs_seed_tx,        // 16 × 23 bit
    input  wire  [3:0]                     flip_sel_tx,
    input  wire                            reversal_en_tx,
    input  wire  [15:0]                    repair_map_tx,
    input  wire  [PHY_IO_NUM-1:0]          msb_lsb_flip_en_tx,
    input  wire  [IO_WIDTH-1:0]            vref_pattern_tx,
    input  wire  [IO_WIDTH-1:0]            pi_pattern_tx,
    input  wire  [IO_WIDTH-1:0]            dcdl_pattern_tx,
    input  wire  [IO_WIDTH-1:0]            reorder_pattern_tx,
    input  wire  [IO_WIDTH-1:0]            datacheck_pattern_tx,
    input  wire                            data_check_pattern_sel,
    input  wire                            scramble_en_tx,
    input  wire                            seed_load_tx,
    input  wire                            clear_tx
);

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    // stage 间串行数据连线（上一级 data_o → 下一级 data_i）
    wire [IO_NUM*IO_WIDTH-1:0]     dispatcher_out_data;   // stage 1 → stage 2（1024 bit）
    wire [IO_NUM*IO_WIDTH-1:0]     scrambler_out_data;    // stage 2 → stage 3（1024 bit）
    wire [PHY_IO_NUM*IO_WIDTH-1:0] io_mapper_out_data;    // stage 3 → stage 4（1088 bit）
    wire [PHY_IO_NUM*IO_WIDTH-1:0] msb_lsb_out_data;      // stage 4 → stage 5（1088 bit）

    // ==========================================================================
    // 子模块实例化
    // ==========================================================================

    // --------------------------------------------------------------------------
    // stage 1：tx_byte_dispatcher（纯组合 0 cycle）
    // --------------------------------------------------------------------------
    tx_byte_dispatcher #(
        .DATA_WIDTH (DATA_WIDTH),
        .IO_NUM     (IO_NUM),
        .IO_WIDTH   (IO_WIDTH)
    ) u_tx_byte_dispatcher (
        .mb_clk            (mb_clk),
        .mb_rst_n          (mb_rst_n),
        .data_i            (tx_lane_data),
        .data_o            (dispatcher_out_data),
        .io_data_width_mode(io_data_width_mode)
    );

    // --------------------------------------------------------------------------
    // stage 2：tx_prbs_scrambler（输出寄存型 1 cycle）
    // --------------------------------------------------------------------------
    tx_prbs_scrambler #(
        .IO_NUM      (IO_NUM),
        .IO_WIDTH    (IO_WIDTH),
        .STATE_WIDTH (23)
    ) u_tx_prbs_scrambler (
        .mb_clk                (mb_clk),
        .mb_rst_n              (mb_rst_n),
        .data_i                (dispatcher_out_data),
        .data_o                (scrambler_out_data),
        .vref_req              (vref_req),
        .pi_req                (pi_req),
        .dcdl_req              (dcdl_req),
        .reorder_req           (reorder_req),
        .data_check_req        (data_check_req),
        .vref_pattern_tx       (vref_pattern_tx),
        .pi_pattern_tx         (pi_pattern_tx),
        .dcdl_pattern_tx       (dcdl_pattern_tx),
        .reorder_pattern_tx    (reorder_pattern_tx),
        .datacheck_pattern_tx  (datacheck_pattern_tx),
        .data_check_pattern_sel(data_check_pattern_sel),
        .scramble_en_tx        (scramble_en_tx),
        .prbs_seed_tx          (prbs_seed_tx),
        .flip_sel_tx           (flip_sel_tx),
        .io_data_width_mode    (io_data_width_mode),
        .seed_load_i           (seed_load_tx),
        .clear                 (clear_tx)
    );

    // --------------------------------------------------------------------------
    // stage 3：tx_io_mapper（纯组合 0 cycle，1 GHz 降频后去输出寄存）
    // --------------------------------------------------------------------------
    tx_io_mapper #(
        .IO_NUM     (IO_NUM),
        .PHY_IO_NUM (PHY_IO_NUM),
        .IO_WIDTH   (IO_WIDTH)
    ) u_tx_io_mapper (
        .data_i            (scrambler_out_data),
        .data_o            (io_mapper_out_data),
        .reversal_en_tx    (reversal_en_tx),
        .repair_map_tx     (repair_map_tx),
        .io_data_width_mode(io_data_width_mode)
    );

    // --------------------------------------------------------------------------
    // stage 4：tx_msb_lsb（纯组合 0 cycle）
    // --------------------------------------------------------------------------
    tx_msb_lsb #(
        .PHY_IO_NUM (PHY_IO_NUM),
        .IO_WIDTH   (IO_WIDTH)
    ) u_tx_msb_lsb (
        .mb_clk            (mb_clk),
        .mb_rst_n          (mb_rst_n),
        .data_i            (io_mapper_out_data),
        .data_o            (msb_lsb_out_data),
        .msb_lsb_flip_en_tx(msb_lsb_flip_en_tx)
    );

    // --------------------------------------------------------------------------
    // stage 5：tx_mb_pipe（边界 pipeline，MB_PIPE_DEPTH cycle）
    // --------------------------------------------------------------------------
    tx_mb_pipe #(
        .PHY_IO_NUM    (PHY_IO_NUM),
        .IO_WIDTH      (IO_WIDTH),
        .MB_PIPE_DEPTH (MB_PIPE_DEPTH)
    ) u_tx_mb_pipe (
        .mb_clk  (mb_clk),
        .mb_rst_n(mb_rst_n),
        .data_i  (msb_lsb_out_data),
        .data_o  (tx_analog_pad)
    );

endmodule
