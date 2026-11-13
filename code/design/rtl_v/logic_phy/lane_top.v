// =============================================================================
// module name：lane_top
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/lane_top_spec.md（spec 级父模块，无 MAS）
// =============================================================================
// 功能概述: 单 lane 集成父模块（纯例化连线，无逻辑）：tx_chain + rx_chain 按
//   TX_EN/RX_EN 参数可选例化，时钟复位广播 + per-step 训练 req/complete 与 CSR 具名配置透传路由
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-07  OpenHappy Team  初始版本：基于 lane_top_spec v1.19 集成生成
//  2026-08-10  OpenHappy Team  MB_PIPE_DEPTH 拆分 TX/RX（TX_MB_PIPE_DEPTH /
//                       RX_MB_PIPE_DEPTH），默认值 2 → 1（与 spec 偏差见设计说明）
//  2026-10-12  OpenHappy Team  TX_MB_PIPE_DEPTH / RX_MB_PIPE_DEPTH 默认 1 → 0（同步 spec，
//                       边界 mb_pipe 直通 bypass：末级寄存器消失，边界长走线时序
//                       由模拟侧/布线吸收，逻辑级数最短）
//  2026-10-21  OpenHappy Team  新增 rx_async_fifo_status[7:0] 输出（issue #13 方案 C）：per-lane
//                       寄存器布局 {unf, ovf, empty, full, level[3:0]}，由 rx_chain
//                       fifo_status_o 显式重排拼接（位序与 rx_async_fifo 相反）；
//                       ovf/unf 本阶段恒 0（见文件头设计说明）
//  2026-08-11  OpenHappy Team  TX_EN/RX_EN 参数化条件例化（对齐 lane_top_spec v1.21
//                       TX-only/RX-only 形态；禁能侧输出 tie-off 见设计说明）
//  2026-08-11  OpenHappy Team  TRAIN_PARAM_WIDTH 默认 315→323（CDR_OFFSET_CTRL 8 bit 并入
//                       offset_train，对齐 logicphy_analogphy_intf v1.1）
// =============================================================================

`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// 设计说明:
//
// 集成假设 / 与 spec 偏差说明:
//   - 命名映射（lane_top 顶端口 ↔ 子模块实际端口，以子模块 .v 为准，本层例化时映射）：
//       train_<step>_req_o        ↔ tx_chain.<step>_req_o（7 条；
//                                   tx_chain 端口 dcDL_req_o 保留原样大小写）
//       train_dcDL_complete_i     ↔ tx_chain.train_dcdl_complete_i（dcdl 全小写）
//       train_dcDL_req_i          ↔ rx_chain.train_dcdl_req_i（dcdl 全小写）
//       train_dcDL_complete_o     ↔ rx_chain.train_dcdl_complete_o（dcdl 全小写）
//       rx_retrain_req_i          ↔ rx_chain.retrain_req_i（RX 侧重训请求，
//                                   lane_top_spec v1.12 拆分 rx_retrain_req_i）
//   - self_sync_en 不进 lane_top 顶端口：lane_top_spec.json internal_connections
//     中 ports.self_sync_en → rx_chain.self_sync_en 为过时引用——rx_chain v2.1 起
//     self_sync_en 为 rx_chain 内部信号（rx_training → rx_data_path 内部连线），
//     无此顶层端口；待 lane_top_spec.json 后续版本清理该引用
//   - irq_lane_o 位宽偏差（TODO，待 spec 裁决）：lane_top_spec.md §5.6 定义
//     irq_lane_o 为 4 bit（TX/RX training_fsm 各 4 bit 按 bit OR 聚合出顶）；
//     本实现按集成任务要求采用 8 bit 拼接 irq_lane_o = {rx_irq[3:0], tx_irq[3:0]}
//     （TX/RX 中断源硬件分离，软件无需经 INT_STATUS 区分触发源），
//     与 spec.md §5.6 不一致，待 spec 修订或回本层改 OR 聚合
//   - rx_chain 未被 lane_top_spec 覆盖的端口：reorder 配置（reorder_delay_rx /
//     reorder_order_rx / reorder_cfg_valid）与 rx_async_fifo 控制端口（fifo_soft_reset /
//     overflow_clear / underflow_clear）由 rx_chain 层输入 tie 0，本层不新增端口
//     （lane_top_spec.json 仅有 reorder_delay_step w=4 的过时引用）；
//     rx_async_fifo 状态则新增本层输出 rx_async_fifo_status[7:0] 引至 logic_phy
//     → apb_csr_top RX_ASYNC_FIFO_STATUS（issue #13 方案 C），布局
//     {unf[7], ovf[6], empty[5], full[4], level[3:0]}——rx_async_fifo.fifo_status 为
//     {overflow, underflow, full, empty}（[0]=empty/[1]=full），与寄存器位序相反，
//     故必须显式拼接；ovf/unf 本阶段恒 0：underflow 由 rx_async_fifo 的 empty 电平
//     触发（复位后 empty 恒 1 即恒置位），且两者的 clear 在 rx_chain 恒 tie 0
//     （RW1C 清完即被重置），本阶段不具备可用语义
//   - per-lane RX 复位命名 rx_rst_n（lane_top_spec §5.2 遗留项：命名归 MAS；
//     与 rx_chain.v 已有端口命名保持一致）
//   - MB_PIPE_DEPTH 拆分 TX/RX（TX_MB_PIPE_DEPTH / RX_MB_PIPE_DEPTH）且默认
//     2 → 1：lane_top_spec §3.1/§6.1 描述为默认 2 级；深度 0 时末级寄存器消失，
//     边界长走线与模拟侧时序失去 pipeline 吸收能力，深度 1 是物理边界最小安全
//     配置（末级寄存器仍落在 logic_phy 边界），待 spec 修订
//   - 2026-10-12 默认值再降为 0（bypass）：用户决策，逻辑级数最短优先，边界走线
//     时序由模拟侧调整；spec 已同步为默认 0（logic_phy_spec / lane_top_spec）
//   - TX_EN/RX_EN 禁能侧 tie-off 语义（2026-08-11，per-lane 方向使能由
//     logic_phy 按 LP_TX_LANE_NUM/LP_RX_LANE_NUM 宏派生传入）：
//       TX_EN=0：tx_analog_pad / train_complete_ack_o / 7 条 train_*_req_o /
//                irq TX nibble 输出 0，TX 侧输入（含 CSR 具名配置）不消费
//       RX_EN=0：rx_lane_data / aligned_o / train_param_to_analog / train_req_o /
//                7 条 train_*_complete_o / irq RX nibble 输出 0，RX 侧输入不消费；
//                aligned_o 输出 0 由 logic_phy 聚合处按 RX 使能掩码剔除（不参与 AND） |
//                rx_async_fifo_status = 8'h20（仅 empty[5]=1 = 复位默认空态，
//                level/full/ovf/unf 全 0）
// -----------------------------------------------------------------------------

module lane_top #(
    parameter DATA_WIDTH        = 1024,
    parameter IO_NUM            = 16,
    parameter PHY_IO_NUM        = 17,
    parameter IO_WIDTH          = 64,
    parameter TX_MB_PIPE_DEPTH  = 0,
    parameter RX_MB_PIPE_DEPTH  = 0,
    parameter TRAIN_PARAM_WIDTH = 380,
    parameter TX_EN             = 1,    // 1=例化 tx_chain；0=TX 侧输出 tie-off（TX-only/RX-only 形态）
    parameter RX_EN             = 1     // 1=例化 rx_chain；0=RX 侧输出 tie-off
) (
    // 时钟与复位
    input  wire                            mb_clk,            // TX 主带时钟（per-link 共享；rx_async_fifo 读侧）
    input  wire                            mb_rst_n,          // mb_clk 域异步复位，低有效
    input  wire                            phy_rx_clk,        // per-lane RX 恢复时钟（spec 中 phy_rx_clk{i}，单 lane 模板去 {i}）
    input  wire                            rx_rst_n,          // RX 域异步复位，低有效

    // 主带数据通道
    input  wire  [DATA_WIDTH-1:0]          tx_lane_data,      // 链路层 CPIF → tx_chain
    output wire  [PHY_IO_NUM*IO_WIDTH-1:0] tx_analog_pad,     // tx_chain → analog PHY
    input  wire  [PHY_IO_NUM*IO_WIDTH-1:0] rx_analog_pad,     // analog PHY → rx_chain
    output wire  [DATA_WIDTH-1:0]          rx_lane_data,      // rx_chain（CDC 后）→ 链路层 CPIF

    // PRBS 对齐状态（rx_chain → apb_csr_top DFX）
    output wire  [IO_NUM-1:0]              aligned_o,

    // RX 异步 FIFO 状态（→ apb_csr_top RX_ASYNC_FIFO_STATUS；位序见文件头设计说明）
    output wire  [7:0]                     rx_async_fifo_status,

    // 训练参数下发（rx_chain 4 执行单元 fan-in 聚合 → analog PHY）
    output wire  [TRAIN_PARAM_WIDTH-1:0]   train_param_to_analog,

    // 训练启动 / 重训练 / 完成应答（TX 方向，mb_clk 同域，CDC 在 sideband）
    input  wire                            train_req_i,
    input  wire                            retrain_req_i,
    output wire                            train_complete_ack_o,

    // per-step 训练请求（TX 方向：tx_chain → sideband per-step SBC encode）
    output wire                            train_offset_req_o,
    output wire                            train_vref_req_o,
    output wire                            train_eq_req_o,
    output wire                            train_pi_req_o,
    output wire                            train_dcDL_req_o,
    output wire                            train_reorder_req_o,
    output wire                            train_data_check_req_o,

    // per-step 训练完成指示（TX 方向：sideband 解 SBC + 多 lane AND 聚合后回送）
    input  wire                            train_offset_complete_i,
    input  wire                            train_vref_complete_i,
    input  wire                            train_eq_complete_i,
    input  wire                            train_pi_complete_i,
    input  wire                            train_dcDL_complete_i,
    input  wire                            train_reorder_complete_i,
    input  wire                            train_data_check_complete_i,

    // 训练启动 / 重训练 / 完成应答（RX 方向，phy_rx_clk 同域，CDC 在 sideband）
    output wire                            train_req_o,
    input  wire                            rx_retrain_req_i,
    input  wire                            train_complete_ack_i,

    // per-step 训练请求（RX 方向：sideband 透传对端 TX 请求 → rx_chain）
    input  wire                            train_offset_req_i,
    input  wire                            train_vref_req_i,
    input  wire                            train_eq_req_i,
    input  wire                            train_pi_req_i,
    input  wire                            train_dcDL_req_i,
    input  wire                            train_reorder_req_i,
    input  wire                            train_data_check_req_i,

    // per-step 训练完成指示（RX 方向：rx_chain → sideband SBC encode 回对端 TX）
    output wire                            train_offset_complete_o,
    output wire                            train_vref_complete_o,
    output wire                            train_eq_complete_o,
    output wire                            train_pi_complete_o,
    output wire                            train_dcDL_complete_o,
    output wire                            train_reorder_complete_o,
    output wire                            train_data_check_complete_o,

    // APB CSR 具名配置（tx_chain 路径，apb_csr_top 解码后直送）
    input  wire                            io_data_width_mode,    // TX/RX 共享字段
    input  wire  [IO_NUM*23-1:0]           prbs_seed_tx,          // 16 × 23 bit
    input  wire  [3:0]                     flip_sel_tx,
    input  wire                            reversal_en_tx,
    input  wire  [15:0]                    repair_map_tx,
    input  wire  [PHY_IO_NUM-1:0]          msb_lsb_flip_en_tx,
    input  wire  [IO_WIDTH-1:0]            vref_pattern_tx,
    input  wire  [IO_WIDTH-1:0]            pi_pattern_tx,
    input  wire  [IO_WIDTH-1:0]            dcdl_pattern_tx,
    input  wire  [IO_WIDTH-1:0]            reorder_pattern_tx,
    input  wire  [IO_WIDTH-1:0]            datacheck_pattern_tx,
    input  wire                            data_check_pattern_sel, // TX/RX 共享字段
    input  wire                            scramble_en_tx,
    input  wire                            seed_load_tx,
    input  wire                            clear_tx,

    // APB CSR 具名配置（rx_chain 路径，apb_csr_top 解码后直送）
    input  wire  [PHY_IO_NUM-1:0]          msb_lsb_flip_en_rx,
    input  wire                            reversal_en_rx,
    input  wire  [15:0]                    repair_map_rx,
    input  wire  [IO_NUM*23-1:0]           prbs_seed_rx,          // 16 × 23 bit
    input  wire  [3:0]                     flip_sel_rx,
    input  wire                            scramble_en,
    input  wire                            seed_load_i,
    input  wire                            clear,

    // APB CSR 具名配置（训练 FSM 路径，fanout 至 tx_chain / rx_chain 两侧）
    input  wire  [1:0]                     train_mode,
    input  wire                            force_mode_switch,
    input  wire  [6:0]                     bypass_step,
    input  wire  [6:0]                     timeout_step_en,
    input  wire  [15:0]                    timeout_threshold,
    input  wire                            step_trigger,
    input  wire  [6:0]                     step_force_end,
    input  wire  [2:0]                     step_cnt,

    // 中断（{rx[3:0], tx[3:0]} 拼接；各 4 bit：bit[0]=train done / bit[1]=train fail /
    // bit[2]=step done / bit[3]=train timeout；位宽与 spec.md §5.6 偏差见文件头说明）
    output wire  [7:0]                     irq_lane_o,
    // 族 a lane0 per-IO 训练参数（MDE-CHAIN-RERUN-01-271-IMPL; 打包总线; lanes1-3 位恒 0 = 4T1R 仅 TX）
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
    // TX / RX 训练 FSM 各自 4 bit 中断，本层拼接出顶（见文件头 irq_lane_o 偏差说明）
    wire [3:0] tx_irq_lane;
    wire [3:0] rx_irq_lane;

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================
    // 中断拼接：高 4 bit = RX，低 4 bit = TX
    assign irq_lane_o = {rx_irq_lane, tx_irq_lane};

    // ==========================================================================
    // 子模块实例化
    // ==========================================================================

    // --------------------------------------------------------------------------
    // tx_chain：TX 侧（tx_data_path 5 stage + tx_training_fsm，单 mb_clk 域）
    //   TX_EN=0 时不例化，TX 侧输出 tie-off（语义见文件头设计说明）
    //   顶端口 train_<step>_req_o / train_<step>_complete_i 与 tx_chain 端口
    //   命名差异在本例化端口映射中完成（dcDL ↔ dcDL_req_o / train_dcdl_complete_i）
    // --------------------------------------------------------------------------
    generate
        if (TX_EN) begin : gen_tx_chain

            tx_chain #(
                .DATA_WIDTH    (DATA_WIDTH),
                .IO_NUM        (IO_NUM),
                .PHY_IO_NUM    (PHY_IO_NUM),
                .IO_WIDTH      (IO_WIDTH),
                .MB_PIPE_DEPTH (TX_MB_PIPE_DEPTH)
            ) u_tx_chain (
                .mb_clk                      (mb_clk),
                .mb_rst_n                    (mb_rst_n),
                .tx_lane_data                (tx_lane_data),
                .tx_analog_pad               (tx_analog_pad),
                .train_req_i                 (train_req_i),
                .retrain_req_i               (retrain_req_i),
                .train_offset_complete_i     (train_offset_complete_i),
                .train_vref_complete_i       (train_vref_complete_i),
                .train_eq_complete_i         (train_eq_complete_i),
                .train_pi_complete_i         (train_pi_complete_i),
                .train_dcdl_complete_i       (train_dcDL_complete_i),
                .train_reorder_complete_i    (train_reorder_complete_i),
                .train_data_check_complete_i (train_data_check_complete_i),
                .train_complete_ack_o        (train_complete_ack_o),
                .offset_req_o                (train_offset_req_o),
                .vref_req_o                  (train_vref_req_o),
                .eq_req_o                    (train_eq_req_o),
                .pi_req_o                    (train_pi_req_o),
                .dcDL_req_o                  (train_dcDL_req_o),
                .reorder_req_o               (train_reorder_req_o),
                .data_check_req_o            (train_data_check_req_o),
                .io_data_width_mode          (io_data_width_mode),
                .prbs_seed_tx                (prbs_seed_tx),
                .flip_sel_tx                 (flip_sel_tx),
                .reversal_en_tx              (reversal_en_tx),
                .repair_map_tx               (repair_map_tx),
                .msb_lsb_flip_en_tx          (msb_lsb_flip_en_tx),
                .vref_pattern_tx             (vref_pattern_tx),
                .pi_pattern_tx               (pi_pattern_tx),
                .dcdl_pattern_tx             (dcdl_pattern_tx),
                .reorder_pattern_tx          (reorder_pattern_tx),
                .datacheck_pattern_tx        (datacheck_pattern_tx),
                .data_check_pattern_sel      (data_check_pattern_sel),
                .scramble_en_tx              (scramble_en_tx),
                .seed_load_tx                (seed_load_tx),
                .clear_tx                    (clear_tx),
                .train_mode                  (train_mode),
                .force_mode_switch           (force_mode_switch),
                .bypass_step                 (bypass_step),
                .timeout_step_en             (timeout_step_en),
                .timeout_threshold           (timeout_threshold),
                .step_trigger                (step_trigger),
                .step_force_end              (step_force_end),
                .step_cnt                    (step_cnt),
                .irq_lane_o                  (tx_irq_lane)
            );

        end
        else begin : gen_tx_tieoff

            assign tx_analog_pad          = {PHY_IO_NUM*IO_WIDTH{1'b0}};
            assign train_complete_ack_o   = 1'b0;
            assign train_offset_req_o     = 1'b0;
            assign train_vref_req_o       = 1'b0;
            assign train_eq_req_o         = 1'b0;
            assign train_pi_req_o         = 1'b0;
            assign train_dcDL_req_o       = 1'b0;
            assign train_reorder_req_o    = 1'b0;
            assign train_data_check_req_o = 1'b0;
            assign tx_irq_lane            = 4'b0000;

        end
    endgenerate

    // --------------------------------------------------------------------------
    // rx_chain：RX 侧（rx_data_path 6 stage + rx_training 子系统）
    //   RX_EN=0 时不例化，RX 侧输出 tie-off（语义见文件头设计说明）
    //   命名映射：rx_retrain_req_i → retrain_req_i；
    //   train_dcDL_req_i / train_dcDL_complete_o ↔ train_dcdl_*（dcdl 全小写）
    //   self_sync_en 为 rx_chain 内部信号，不占本层端口（见文件头说明）
    // --------------------------------------------------------------------------
    generate
        if (RX_EN) begin : gen_rx_chain

            // rx_async_fifo.fifo_status 位序 {overflow, underflow, full, empty}
            // （[0]=empty / [1]=full）与寄存器布局 {unf, ovf, empty, full, level[3:0]}
            // 相反，必须显式重排拼接；ovf/unf 本阶段恒 0（见文件头设计说明）
            wire [3:0] fifo_status_w;

            rx_chain #(
                .DATA_WIDTH        (DATA_WIDTH),
                .IO_NUM            (IO_NUM),
                .PHY_IO_NUM        (PHY_IO_NUM),
                .IO_WIDTH          (IO_WIDTH),
                .MB_PIPE_DEPTH     (RX_MB_PIPE_DEPTH),
                .TRAIN_PARAM_WIDTH (TRAIN_PARAM_WIDTH)
            ) u_rx_chain (
                .phy_rx_clk                  (phy_rx_clk),
                .rx_rst_n                    (rx_rst_n),
                .mb_clk                      (mb_clk),
                .mb_rst_n                    (mb_rst_n),
                .rx_analog_pad               (rx_analog_pad),
                .rx_lane_data                (rx_lane_data),
                .aligned_o                   (aligned_o),
                .io_data_width_mode          (io_data_width_mode),
                .msb_lsb_flip_en_rx          (msb_lsb_flip_en_rx),
                .reversal_en_rx              (reversal_en_rx),
                .repair_map_rx               (repair_map_rx),
                .prbs_seed_rx                (prbs_seed_rx),
                .flip_sel_rx                 (flip_sel_rx),
                .scramble_en                 (scramble_en),
                .seed_load_i                 (seed_load_i),
                .clear                       (clear),
                .train_mode                  (train_mode),
                .force_mode_switch           (force_mode_switch),
                .bypass_step                 (bypass_step),
                .timeout_step_en             (timeout_step_en),
                .timeout_threshold           (timeout_threshold),
                .data_check_pattern_sel      (data_check_pattern_sel),
                .step_trigger                (step_trigger),
                .step_force_end              (step_force_end),
                .step_cnt                    (step_cnt),
                .train_req_o                 (train_req_o),
                .retrain_req_i               (rx_retrain_req_i),
                .train_complete_ack_i        (train_complete_ack_i),
                .train_offset_req_i          (train_offset_req_i),
                .train_vref_req_i            (train_vref_req_i),
                .train_eq_req_i              (train_eq_req_i),
                .train_pi_req_i              (train_pi_req_i),
                .train_dcdl_req_i            (train_dcDL_req_i),
                .train_reorder_req_i         (train_reorder_req_i),
                .train_data_check_req_i      (train_data_check_req_i),
                .train_offset_complete_o     (train_offset_complete_o),
                .train_vref_complete_o       (train_vref_complete_o),
                .train_eq_complete_o         (train_eq_complete_o),
                .train_pi_complete_o         (train_pi_complete_o),
                .train_dcdl_complete_o       (train_dcDL_complete_o),
                .train_reorder_complete_o    (train_reorder_complete_o),
                .train_data_check_complete_o (train_data_check_complete_o),
                .train_param_to_analog       (train_param_to_analog),
                .offset_param                (offset_param),
                .vref_param                  (vref_param),
                .pi_param                    (pi_param),
                .dcdl_param                  (dcdl_param),
                .train_param_src_i           (train_param_src_i),
                .fail_threshold_i            (fail_threshold_i),
                .ctle_tap_i                  (ctle_tap_i),
                .irq_lane_o                  (rx_irq_lane),
                .fifo_status_o               (fifo_status_w)
            );

            assign rx_async_fifo_status = {2'b00, fifo_status_w[0], fifo_status_w[1], 4'h0};

        end
        else begin : gen_rx_tieoff

            assign rx_lane_data                = {DATA_WIDTH{1'b0}};
            assign aligned_o                   = {IO_NUM{1'b0}};
            assign train_param_to_analog       = {TRAIN_PARAM_WIDTH{1'b0}};
            assign train_req_o                 = 1'b0;
            assign train_offset_complete_o     = 1'b0;
            assign train_vref_complete_o       = 1'b0;
            assign train_eq_complete_o         = 1'b0;
            assign train_pi_complete_o         = 1'b0;
            assign train_dcDL_complete_o       = 1'b0;
            assign train_reorder_complete_o    = 1'b0;
            assign train_data_check_complete_o = 1'b0;
            assign rx_irq_lane                 = 4'b0000;
            // RX 未例化：仅保留 empty 复位默认位（[5]=1，复位默认空），余位 0
            assign rx_async_fifo_status        = 8'h20;

        end
    endgenerate

endmodule
