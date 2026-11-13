// =============================================================================
// module name：rx_chain
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/rx_chain/rx_chain_spec.md（spec 级父模块，无 MAS）
// =============================================================================
// 功能概述: RX 侧集成父模块（纯例化连线，无逻辑）：rx_data_path 6 stage + rx_training 子系统，
//   数据通路前 5 stage phy_rx_clk 域、stage 6（rx_async_fifo）CDC 至 mb_clk
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-10-21  OpenHappy Team  rx_async_fifo 状态引出（issue #13 方案 C）：新增 fifo_status_o[3:0]
//                       顶层端口（[0]=empty/[1]=full/[2]=underflow/[3]=overflow），
//                       替代原内部 unused_fifo_status 悬空 wire
//  2026-08-11  OpenHappy Team  TRAIN_PARAM_WIDTH 默认 315→323（CDR_OFFSET_CTRL 8 bit 并入
//                       offset_train，对齐 logicphy_analogphy_intf v1.1）
//  2026-08-07  OpenHappy Team  初始版本：基于 rx_chain_spec v2.1 集成生成
// =============================================================================

`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// 设计说明:
//
// 集成假设 / 与 spec 偏差说明:
//   - data_checker 数据源偏差（TODO，待 MAS 阶段修正）：data_checker.data_i 语义应为
//     rx_data_path 内部 rx_prbs_descrambler.data_o（descrambler 后、phy_rx_clk 域），
//     但 rx_data_path 顶层未将该信号引出；本实现暂以 rx_lane_data（rx_async_fifo 输出、
//     mb_clk 域）替代，存在跨时钟域语义偏差，待 rx_data_path 顶层补端口或 MAS 阶段修正
//   - rx_data_path 的 reorder 配置端口（reorder_delay_rx / reorder_order_rx /
//     reorder_cfg_valid）与 rx_async_fifo 控制端口（fifo_soft_reset / overflow_clear /
//     underflow_clear）在 rx_chain_spec.json 中无对应顶层端口（lane_top_spec.json 仅有
//     reorder_delay_step w=4 的过时引用）：本层输入暂 tie 0；状态端口 fifo_status 已引至
//     本层输出 fifo_status_o（issue #13 方案 C，per-lane 8 bit 寄存器布局由 lane_top 拼装）
//   - 训练控制端口命名采用 rx_chain_spec.json v2.1 全名（train_<step>_req_i /
//     train_<step>_complete_o / train_req_o / retrain_req_i / train_complete_ack_i）；
//     lane_top_spec.json 侧 dcDL 端口为 train_dcDL_* 大小写变体，由 lane_top 集成层映射
//   - per-lane RX 复位命名 rx_rst_n（rx_chain_spec 遗留项：命名归 MAS；与 rx_data_path.v
//     已有端口命名保持一致）
// -----------------------------------------------------------------------------

module rx_chain #(
    parameter DATA_WIDTH        = 1024,
    parameter IO_NUM            = 16,
    parameter PHY_IO_NUM        = 17,
    parameter IO_WIDTH          = 64,
    parameter MB_PIPE_DEPTH     = 0,
    parameter TRAIN_PARAM_WIDTH = 380
) (
    // 时钟与复位
    input  wire                            phy_rx_clk,      // per-lane RX 恢复时钟（数据通路前 5 stage + 训练子系统）
    input  wire                            rx_rst_n,        // RX 域异步复位，低有效
    input  wire                            mb_clk,          // TX 主带时钟（rx_async_fifo 读侧）
    input  wire                            mb_rst_n,        // mb_clk 域异步复位，低有效

    // 主带数据通道
    input  wire  [PHY_IO_NUM*IO_WIDTH-1:0] rx_analog_pad,   // analog PHY → rx_data_path stage 1
    output wire  [DATA_WIDTH-1:0]          rx_lane_data,    // rx_data_path stage 6（CDC 后）→ 链路层 CPIIF

    // PRBS 对齐状态（rx_data_path → apb_csr_top DFX；同时内部 fanout 至 rx_training）
    output wire  [IO_NUM-1:0]              aligned_o,

    // RX 异步 FIFO 状态（mb_clk 域呈现；[0]=empty / [1]=full / [2]=underflow / [3]=overflow）
    output wire  [3:0]                     fifo_status_o,

    // APB CSR 具名配置（rx_data_path 路径，apb_csr_top 解码后直送）
    input  wire                            io_data_width_mode,
    input  wire  [PHY_IO_NUM-1:0]          msb_lsb_flip_en_rx,
    input  wire                            reversal_en_rx,
    input  wire  [15:0]                    repair_map_rx,
    input  wire  [IO_NUM*23-1:0]           prbs_seed_rx,    // 16 × 23 bit
    input  wire  [3:0]                     flip_sel_rx,
    input  wire                            scramble_en,
    input  wire                            seed_load_i,
    input  wire                            clear,

    // APB CSR 具名配置（rx_training 路径，apb_csr_top 解码后直送）
    input  wire  [1:0]                     train_mode,
    input  wire                            force_mode_switch,
    input  wire  [6:0]                     bypass_step,
    input  wire  [6:0]                     timeout_step_en,
    input  wire  [15:0]                    timeout_threshold,
    input  wire                            data_check_pattern_sel,
    input  wire                            step_trigger,
    input  wire  [6:0]                     step_force_end,
    input  wire  [2:0]                     step_cnt,

    // sideband-led 训练控制（1 cycle pulse，CDC 在 sideband 侧完成）
    output wire                            train_req_o,
    input  wire                            retrain_req_i,
    input  wire                            train_complete_ack_i,
    input  wire                            train_offset_req_i,
    input  wire                            train_vref_req_i,
    input  wire                            train_eq_req_i,
    input  wire                            train_pi_req_i,
    input  wire                            train_dcdl_req_i,
    input  wire                            train_reorder_req_i,
    input  wire                            train_data_check_req_i,
    output wire                            train_offset_complete_o,
    output wire                            train_vref_complete_o,
    output wire                            train_eq_complete_o,
    output wire                            train_pi_complete_o,
    output wire                            train_dcdl_complete_o,
    output wire                            train_reorder_complete_o,
    output wire                            train_data_check_complete_o,

    // 训练参数下发（rx_training 4 执行单元 fan-in 聚合 → analog PHY）
    output wire  [TRAIN_PARAM_WIDTH-1:0]   train_param_to_analog,

    // 中断（bit[0]=train done / bit[1]=train fail / bit[2]=step done / bit[3]=train timeout）
    output wire  [3:0]                     irq_lane_o,
    // 族 a lane0 per-IO 训练参数（MDE-CHAIN-RERUN-01-271-IMPL; 打包总线; 消费在 rx_training→train）
    input  wire  [PHY_IO_NUM*16-1:0]         offset_param,
    input  wire  [IO_NUM*16-1:0]           vref_param,
    input  wire  [IO_NUM*20-1:0]           pi_param,
    input  wire  [PHY_IO_NUM*12-1:0]         dcdl_param,
    // 族 c hook_min（#3/#4/#7: TRAIN_PARAM_SRC_i / FAIL_THRESHOLD_i / CTLE_TAP_i）
    input  wire                            train_param_src_i,
    input  wire  [7:0]                     fail_threshold_i,
    input  wire  [3:0]                     ctle_tap_i
);

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    // rx_data_path ↔ rx_training 内部互连
    wire                            self_sync_en;       // rx_training → rx_data_path
    wire                            reorder_trigger;    // rx_training → rx_data_path
    wire                            reorder_done;       // rx_data_path → rx_training
    wire [IO_NUM*IO_WIDTH-1:0]      prbs_seq;           // rx_data_path.prbs_seq_o → rx_training.prbs_seq_i

    // ==========================================================================
    // 子模块实例化
    // ==========================================================================

    // --------------------------------------------------------------------------
    // rx_data_path：RX 数据通路 6 stage
    //   reorder 配置 / async_fifo 控制端口 spec.json 未路由，输入暂 tie 0（见文件头偏差说明）
    // --------------------------------------------------------------------------
    rx_data_path #(
        .DATA_WIDTH    (DATA_WIDTH),
        .IO_NUM        (IO_NUM),
        .PHY_IO_NUM    (PHY_IO_NUM),
        .IO_WIDTH      (IO_WIDTH),
        .MB_PIPE_DEPTH (MB_PIPE_DEPTH)
    ) u_rx_data_path (
        .phy_rx_clk        (phy_rx_clk),
        .rx_rst_n          (rx_rst_n),
        .mb_clk            (mb_clk),
        .mb_rst_n          (mb_rst_n),
        .rx_analog_pad     (rx_analog_pad),
        .rx_lane_data      (rx_lane_data),
        .prbs_seq_o        (prbs_seq),
        .aligned_o         (aligned_o),
        .reorder_trigger   (reorder_trigger),
        .reorder_done      (reorder_done),
        .reorder_delay_rx  ({(IO_NUM*6){1'b0}}),
        .reorder_order_rx  (4'b0000),
        .reorder_cfg_valid (1'b0),
        .io_data_width_mode(io_data_width_mode),
        .msb_lsb_flip_en_rx(msb_lsb_flip_en_rx),
        .reversal_en_rx    (reversal_en_rx),
        .repair_map_rx     (repair_map_rx),
        .prbs_seed_rx      (prbs_seed_rx),
        .flip_sel_rx       (flip_sel_rx),
        .scramble_en       (scramble_en),
        .self_sync_en      (self_sync_en),
        .seed_load_i       (seed_load_i),
        .clear             (clear),
        .fifo_soft_reset   (1'b0),
        .overflow_clear    (1'b0),
        .underflow_clear   (1'b0),
        .fifo_status       (fifo_status_o)
    );

    // --------------------------------------------------------------------------
    // rx_training：RX 训练子系统
    //   data_i 暂接 rx_lane_data（async_fifo 输出），与 descrambler.data_o 语义存在
    //   偏差（见文件头 TODO，待 MAS 阶段修正）
    // --------------------------------------------------------------------------
    rx_training #(
        .DATA_WIDTH        (DATA_WIDTH),
        .TRAIN_PARAM_WIDTH (TRAIN_PARAM_WIDTH),
        .IO_NUM            (IO_NUM),
        .PHY_IO_NUM        (PHY_IO_NUM)
    ) u_rx_training (
        .phy_rx_clk                  (phy_rx_clk),
        .rx_rst_n                    (rx_rst_n),
        .self_sync_en                (self_sync_en),
        .aligned_o                   (aligned_o),
        .reorder_trigger             (reorder_trigger),
        .reorder_done                (reorder_done),
        .train_req_o                 (train_req_o),
        .retrain_req_i               (retrain_req_i),
        .train_complete_ack_i        (train_complete_ack_i),
        .train_offset_req_i          (train_offset_req_i),
        .train_vref_req_i            (train_vref_req_i),
        .train_eq_req_i              (train_eq_req_i),
        .train_pi_req_i              (train_pi_req_i),
        .train_dcdl_req_i            (train_dcdl_req_i),
        .train_reorder_req_i         (train_reorder_req_i),
        .train_data_check_req_i      (train_data_check_req_i),
        .train_offset_complete_o     (train_offset_complete_o),
        .train_vref_complete_o       (train_vref_complete_o),
        .train_eq_complete_o         (train_eq_complete_o),
        .train_pi_complete_o         (train_pi_complete_o),
        .train_dcdl_complete_o       (train_dcdl_complete_o),
        .train_reorder_complete_o    (train_reorder_complete_o),
        .train_data_check_complete_o (train_data_check_complete_o),
        .train_mode                  (train_mode),
        .force_mode_switch           (force_mode_switch),
        .bypass_step                 (bypass_step),
        .timeout_step_en             (timeout_step_en),
        .timeout_threshold           (timeout_threshold),
        .data_check_pattern_sel      (data_check_pattern_sel),
        .step_trigger                (step_trigger),
        .step_force_end              (step_force_end),
        .step_cnt                    (step_cnt),
        .data_i                      (rx_lane_data),
        .prbs_seq_i                  (prbs_seq),
        .train_param_o               (train_param_to_analog),
        .offset_param                (offset_param),
        .vref_param                  (vref_param),
        .pi_param                    (pi_param),
        .dcdl_param                  (dcdl_param),
        .train_param_src_i           (train_param_src_i),
        .fail_threshold_i            (fail_threshold_i),
        .ctle_tap_i                  (ctle_tap_i),
        .irq_lane_o                  (irq_lane_o)
    );

endmodule
