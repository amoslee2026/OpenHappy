// =============================================================================
// module name：tx_chain
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/tx_chain/tx_chain_spec.md（spec 级父模块，无 MAS）
// =============================================================================
// 功能概述: TX 侧集成父模块（纯例化连线，无逻辑）：tx_data_path 5 stage + tx_training_fsm，
//   训练 req 双路 fanout 至 tx_data_path pattern mux 与顶层端口（sideband SBC encode）
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-07  OpenHappy Team  初始版本：基于 tx_chain_spec v1.13 集成生成
// =============================================================================

`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// 设计说明:
//
// 集成假设 / 与 spec 偏差说明:
//   - 顶层 complete 端口命名采用 tx_chain_spec.md §5.5 的 train_<step>_complete_i
//     （tx_chain_spec.json internal_connections 中 ports.offset_complete_i 等为其缩略写法，
//     与 tx_training_fsm.v 实际端口 train_<step>_complete_i 对齐，逐信号同名透传）
//   - 顶层 req 端口命名采用 tx_chain_spec.json 的 offset_req_o/vref_req_o/eq_req_o/pi_req_o/
//     dcDL_req_o/reorder_req_o/data_check_req_o（与 tx_training_fsm.v 端口同名；
//     lane_top_spec.json 侧对应端口名为 train_<step>_req_o，由 lane_top 集成层映射）
//   - tx_training_fsm 端口 dcDL_req_o 保留原样大小写（fsm 输出），
//     tx_data_path 侧对应输入端口名为 dcdl_req（全小写，见 tx_data_path.v）
//   - tx_training_fsm.data_check_pattern_sel_i 为 1 bit 且在 fsm 内部声明 unused
//     （spec v1.10 起 pattern_sel 消费方移至 tx_data_path），本层 tie 1'b0
//   - tx_training_fsm DFX 状态输出（train_fsm_state_o/train_step_cnt_o/train_timeout_cnt_o/
//     train_done_o/train_fail_o/step_done_o/train_timeout_o）在 tx_chain_spec.json 中
//     无对应顶层端口，本层接内部 wire 不上顶（TRAIN_FSM_STATE 等只读寄存器路径
//     待 lane_top / apb_csr_top 集成阶段补齐）
// -----------------------------------------------------------------------------

module tx_chain #(
    parameter DATA_WIDTH    = 1024,
    parameter IO_NUM        = 16,
    parameter PHY_IO_NUM    = 17,
    parameter IO_WIDTH      = 64,
    parameter MB_PIPE_DEPTH = 0
) (
    // 时钟与复位
    input  wire                            mb_clk,
    input  wire                            mb_rst_n,

    // 主带数据通道
    input  wire  [DATA_WIDTH-1:0]          tx_lane_data,        // 链路层 CPIIF → tx_data_path
    output wire  [PHY_IO_NUM*IO_WIDTH-1:0] tx_analog_pad,       // tx_data_path → analog PHY

    // 训练启动 / 重训练请求（sideband.sbc_cpif_event 解 CPIF event 产生，mb_clk 同域）
    input  wire                            train_req_i,
    input  wire                            retrain_req_i,

    // per-step 训练完成指示（sideband 解 SBC + 多 lane AND 聚合后回送，mb_clk 同域）
    input  wire                            train_offset_complete_i,
    input  wire                            train_vref_complete_i,
    input  wire                            train_eq_complete_i,
    input  wire                            train_pi_complete_i,
    input  wire                            train_dcdl_complete_i,
    input  wire                            train_reorder_complete_i,
    input  wire                            train_data_check_complete_i,

    // 训练全流程完成应答（ACTIVE entry cycle 拉高 1 cycle，回报 sideband）
    output wire                            train_complete_ack_o,

    // per-step 训练请求（one-hot 互斥 level；透传顶层 → sideband per-step SBC encode；
    // 其中 vref/pi/dcdl/reorder/data_check 5 条同时 fanout 至 tx_data_path pattern mux）
    output wire                            offset_req_o,
    output wire                            vref_req_o,
    output wire                            eq_req_o,
    output wire                            pi_req_o,
    output wire                            dcDL_req_o,
    output wire                            reorder_req_o,
    output wire                            data_check_req_o,

    // APB CSR 具名配置（tx_data_path 路径，apb_csr_top 解码后直送）
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
    input  wire                            clear_tx,

    // APB CSR 具名配置（tx_training_fsm 路径，apb_csr_top 解码后直送）
    input  wire  [1:0]                     train_mode,
    input  wire                            force_mode_switch,
    input  wire  [6:0]                     bypass_step,
    input  wire  [6:0]                     timeout_step_en,
    input  wire  [15:0]                    timeout_threshold,
    input  wire                            step_trigger,
    input  wire  [6:0]                     step_force_end,
    input  wire  [2:0]                     step_cnt,

    // 中断（bit[0]=train done / bit[1]=train fail / bit[2]=step done / bit[3]=train timeout）
    output wire  [3:0]                     irq_lane_o
);

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    // tx_training_fsm DFX 状态输出：spec.json 无对应顶层端口，接内部 wire 不上顶
    // verilator lint_off UNUSED
    wire [3:0]  unused_train_fsm_state;
    wire [2:0]  unused_train_step_cnt;
    wire [15:0] unused_train_timeout_cnt;
    wire        unused_train_done;
    wire        unused_train_fail;
    wire        unused_step_done;
    wire        unused_train_timeout;
    // verilator lint_on UNUSED

    // ==========================================================================
    // 子模块实例化
    // ==========================================================================

    // --------------------------------------------------------------------------
    // tx_data_path：TX 数据通路 5 stage
    //   5 条训练 req（vref/pi/dcdl/reorder/data_check）直接读顶层输出端口
    //   （output wire 内部可读），实现 req 双路 fanout 的第二路
    // --------------------------------------------------------------------------
    tx_data_path #(
        .DATA_WIDTH    (DATA_WIDTH),
        .IO_NUM        (IO_NUM),
        .PHY_IO_NUM    (PHY_IO_NUM),
        .IO_WIDTH      (IO_WIDTH),
        .MB_PIPE_DEPTH (MB_PIPE_DEPTH)
    ) u_tx_data_path (
        .mb_clk                (mb_clk),
        .mb_rst_n              (mb_rst_n),
        .tx_lane_data          (tx_lane_data),
        .tx_analog_pad         (tx_analog_pad),
        .vref_req              (vref_req_o),
        .pi_req                (pi_req_o),
        .dcdl_req              (dcDL_req_o),
        .reorder_req           (reorder_req_o),
        .data_check_req        (data_check_req_o),
        .io_data_width_mode    (io_data_width_mode),
        .prbs_seed_tx          (prbs_seed_tx),
        .flip_sel_tx           (flip_sel_tx),
        .reversal_en_tx        (reversal_en_tx),
        .repair_map_tx         (repair_map_tx),
        .msb_lsb_flip_en_tx    (msb_lsb_flip_en_tx),
        .vref_pattern_tx       (vref_pattern_tx),
        .pi_pattern_tx         (pi_pattern_tx),
        .dcdl_pattern_tx       (dcdl_pattern_tx),
        .reorder_pattern_tx    (reorder_pattern_tx),
        .datacheck_pattern_tx  (datacheck_pattern_tx),
        .data_check_pattern_sel(data_check_pattern_sel),
        .scramble_en_tx        (scramble_en_tx),
        .seed_load_tx          (seed_load_tx),
        .clear_tx              (clear_tx)
    );

    // --------------------------------------------------------------------------
    // tx_training_fsm：TX 训练 step 调度器（单 mb_clk 域）
    //   data_check_pattern_sel_i 在 fsm 内部 unused（pattern mux 归 tx_data_path），tie 1'b0
    // --------------------------------------------------------------------------
    tx_training_fsm u_tx_training_fsm (
        .mb_clk                      (mb_clk),
        .mb_rst_n                    (mb_rst_n),
        .train_req_i                 (train_req_i),
        .retrain_req_i               (retrain_req_i),
        .train_offset_complete_i     (train_offset_complete_i),
        .train_vref_complete_i       (train_vref_complete_i),
        .train_eq_complete_i         (train_eq_complete_i),
        .train_pi_complete_i         (train_pi_complete_i),
        .train_dcdl_complete_i       (train_dcdl_complete_i),
        .train_reorder_complete_i    (train_reorder_complete_i),
        .train_data_check_complete_i (train_data_check_complete_i),
        .train_complete_ack_o        (train_complete_ack_o),
        .train_mode_i                (train_mode),
        .force_mode_switch_i         (force_mode_switch),
        .bypass_step_i               (bypass_step),
        .timeout_step_en_i           (timeout_step_en),
        .timeout_threshold_i         (timeout_threshold),
        .data_check_pattern_sel_i    (1'b0),
        .step_trigger_i              (step_trigger),
        .step_force_end_i            (step_force_end),
        .step_cnt_i                  (step_cnt),
        .offset_req_o                (offset_req_o),
        .vref_req_o                  (vref_req_o),
        .eq_req_o                    (eq_req_o),
        .pi_req_o                    (pi_req_o),
        .dcDL_req_o                  (dcDL_req_o),
        .reorder_req_o               (reorder_req_o),
        .data_check_req_o            (data_check_req_o),
        .train_fsm_state_o           (unused_train_fsm_state),
        .train_step_cnt_o            (unused_train_step_cnt),
        .train_timeout_cnt_o         (unused_train_timeout_cnt),
        .train_done_o                (unused_train_done),
        .train_fail_o                (unused_train_fail),
        .step_done_o                 (unused_step_done),
        .train_timeout_o             (unused_train_timeout),
        .irq_lane_o                  (irq_lane_o)
    );

endmodule
