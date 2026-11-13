// =============================================================================
// module name：rx_training
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/rx_chain/submods/rx_training/rx_training_spec.md（spec 级父模块，无 MAS）
// =============================================================================
// 功能概述: RX 训练子系统集成父模块（纯例化连线，无逻辑）：rx_training_fsm +
//   offset/vref/pi/dcdl 4 训练单元 + data_checker，4 单元 interface 具名参数
//   在本层打包聚合为 train_param_o 323 bit 出顶
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-11  OpenHappy Team  4 训练单元输出改 interface 具名端口（大写原文，对齐
//                       logicphy_analogphy_intf v1.1），本层新增打包聚合逻辑
//                       （布局见设计说明）；CDR_OFFSET_CTRL(8) 并入 offset 段
//  2026-08-07  OpenHappy Team  初始版本：基于 rx_training_spec v1.2 集成生成
// =============================================================================

`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// 设计说明:
//
// 集成假设 / 与 spec 偏差说明:
//   - data_i / prbs_seq_i 为 rx_training_spec.json 未列出的顶层输入端口：
//     按 data_checker stub 端口（data_i / prbs_seq_i，1024 bit）引出，
//     数据来自 rx_data_path（prbs_seq_o / descrambler 后数据），见 rx_chain.v 注释
//   - data_check_pattern_sel fanout 至 rx_training_fsm 与 data_checker 两个消费方
//     （spec.json 仅列 FSM 一路，data_checker stub 端口实际需要）
//   - data_checker.check_fail_o 在 spec.json 中无路由，接内部 wire 不上顶
//   - 顶层训练参数端口命名为 train_param_o（rx_training_spec v1.1 N2 决策；
//     rx_chain 层再映射为 train_param_to_analog）
//   - train_param_o 打包布局（380 bit，MSB→LSB；intf v10.3 P0-A 2026-09-22 重构；
//     4 单元输出为 interface 具名信号，本层执行打包，字段拆解归 analog 边界）：
//       offset 段 [379:139]（241）：{RX_OFFSET_CALIBRATION_ON, OFFSET_CTRL_CLK,
//         OFFSET_RESET_N, RX16..RX0 各 {WRITE, ADDR[2:0], DATA[9:0]}}
//         （per-IO 14 bit，RX0 在段 LSB；CDR_OFFSET_CTRL 已删，intf v10.3）
//       vref 段 [138:123]（16）：{RX_VREF_CTRL_MAIN, RX_VREF_CTRL_ERR_EDGE}
//       pi 段 [122:102]（21）：{PH_I_GRAY_PI, PH_Q_GRAY_PI, PH_045_GRAY_PI}（7 bit GRAY）
//       dcdl 段 [101:0]（102）：{DCDL_CTRL_RX16..RX0}（6 bit/IO，RX0 在段 LSB；
//         DCDL_CDR×16 已删，intf v10.4）
//   - 顶层输入 aligned_o 保留 _o 后缀命名（与 rx_data_path.aligned_o 信号身份对齐，
//     依 rx_training_spec §5.3；方向为 input）
// -----------------------------------------------------------------------------

module rx_training #(
    parameter DATA_WIDTH          = 1024,
    parameter TRAIN_PARAM_WIDTH   = 380,   // WP1 P0-A: offset 241(3+238) + vref 16 + pi 21 + dcdl 102
    parameter IO_NUM              = 16,    // per-lane IO 数（族 a per-IO 训练参数面宽）
    parameter PHY_IO_NUM          = 17     // 物理 IO 数（含 RX16 冗余；intf v10.3 per-IO 训练信号 17 路）
) (
    // 时钟与复位
    input  wire                          phy_rx_clk,
    input  wire                          rx_rst_n,

    // PRBS 对齐控制（↔ rx_data_path.rx_prbs_descrambler，phy_rx_clk 同域）
    output wire                          self_sync_en,     // 出顶 → rx_data_path.self_sync_en
    input  wire  [15:0]                  aligned_o,        // 入顶 ← rx_data_path.aligned_o（方向 input，命名依 spec）

    // reorder 训练触发（↔ rx_data_path.rx_byte_distributor，valid-ready，phy_rx_clk 同域）
    output wire                          reorder_trigger,
    input  wire                          reorder_done,

    // sideband-led 训练控制（1 cycle pulse，CDC 在 sideband 侧完成）
    output wire                          train_req_o,
    input  wire                          retrain_req_i,
    input  wire                          train_complete_ack_i,
    input  wire                          train_offset_req_i,
    input  wire                          train_vref_req_i,
    input  wire                          train_eq_req_i,
    input  wire                          train_pi_req_i,
    input  wire                          train_dcdl_req_i,
    input  wire                          train_reorder_req_i,
    input  wire                          train_data_check_req_i,
    output wire                          train_offset_complete_o,
    output wire                          train_vref_complete_o,
    output wire                          train_eq_complete_o,
    output wire                          train_pi_complete_o,
    output wire                          train_dcdl_complete_o,
    output wire                          train_reorder_complete_o,
    output wire                          train_data_check_complete_o,

    // APB CSR 具名配置（apb_csr_top 解码后直送，9 路 → rx_training_fsm）
    input  wire  [1:0]                   train_mode,
    input  wire                          force_mode_switch,
    input  wire  [6:0]                   bypass_step,
    input  wire  [6:0]                   timeout_step_en,
    input  wire  [15:0]                  timeout_threshold,
    input  wire                          data_check_pattern_sel,
    input  wire                          step_trigger,
    input  wire  [6:0]                   step_force_end,
    input  wire  [2:0]                   step_cnt,

    // data_checker 数据通路（spec.json 未列出，按 data_checker stub 端口引出；
    // 数据来自 rx_data_path，phy_rx_clk 同域语义见 rx_chain.v 注释）
    input  wire  [DATA_WIDTH-1:0]        data_i,           // DATA_CHK 态 bypass 直通数据
    input  wire  [DATA_WIDTH-1:0]        prbs_seq_i,       // rx_prbs_descrambler.prbs_seq_o（expected PRBS pattern）

    // 训练参数下发（4 执行单元具名输出在本层打包聚合 → analog PHY，布局见设计说明）
    output wire  [TRAIN_PARAM_WIDTH-1:0] train_param_o,

    // 中断（bit[0]=train done / bit[1]=train fail / bit[2]=step done / bit[3]=train timeout）
    output wire  [3:0]                   irq_lane_o,
    // 族 a lane0 per-IO 训练参数（MDE-CHAIN-RERUN-01-271-IMPL; 打包总线, 16×位宽）
    input  wire  [PHY_IO_NUM*16-1:0]          offset_param,   // 17 × 16 bit per-IO（intf v10.3: RX0~16 含冗余 IO）
    input  wire  [IO_NUM*16-1:0]              vref_param,     // 16 × 16 bit
    input  wire  [IO_NUM*20-1:0]              pi_param,       // 16 × 20 bit
    input  wire  [PHY_IO_NUM*12-1:0]          dcdl_param,     // 17 × 12 bit per-IO（intf v10.3）
    // 族 c hook_min（MDE-CHAIN-RERUN-01-271-IMPL: #3/#4/#7, 消费待 TRAIN-MAS-01）
    input  wire                               train_param_src_i,
    input  wire  [7:0]                        fail_threshold_i,
    input  wire  [3:0]                        ctle_tap_i
);

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    // 4 个训练执行单元参数输出（interface 具名信号，大写原文；本层打包聚合到 train_param_o）
    // offset 段（215 bit）
    wire        RX_OFFSET_CALIBRATION_ON;
    wire        OFFSET_CTRL_CLK;
    wire        OFFSET_RESET_N;
    wire        RX0_OFFSET_CTRL_WRITE;
    wire [2:0]  RX0_OFFSET_CTRL_ADDR;
    wire [9:0]  RX0_OFFSET_CTRL_DATA;
    wire        RX1_OFFSET_CTRL_WRITE;
    wire [2:0]  RX1_OFFSET_CTRL_ADDR;
    wire [9:0]  RX1_OFFSET_CTRL_DATA;
    wire        RX2_OFFSET_CTRL_WRITE;
    wire [2:0]  RX2_OFFSET_CTRL_ADDR;
    wire [9:0]  RX2_OFFSET_CTRL_DATA;
    wire        RX3_OFFSET_CTRL_WRITE;
    wire [2:0]  RX3_OFFSET_CTRL_ADDR;
    wire [9:0]  RX3_OFFSET_CTRL_DATA;
    wire        RX4_OFFSET_CTRL_WRITE;
    wire [2:0]  RX4_OFFSET_CTRL_ADDR;
    wire [9:0]  RX4_OFFSET_CTRL_DATA;
    wire        RX5_OFFSET_CTRL_WRITE;
    wire [2:0]  RX5_OFFSET_CTRL_ADDR;
    wire [9:0]  RX5_OFFSET_CTRL_DATA;
    wire        RX6_OFFSET_CTRL_WRITE;
    wire [2:0]  RX6_OFFSET_CTRL_ADDR;
    wire [9:0]  RX6_OFFSET_CTRL_DATA;
    wire        RX7_OFFSET_CTRL_WRITE;
    wire [2:0]  RX7_OFFSET_CTRL_ADDR;
    wire [9:0]  RX7_OFFSET_CTRL_DATA;
    wire        RX8_OFFSET_CTRL_WRITE;
    wire [2:0]  RX8_OFFSET_CTRL_ADDR;
    wire [9:0]  RX8_OFFSET_CTRL_DATA;
    wire        RX9_OFFSET_CTRL_WRITE;
    wire [2:0]  RX9_OFFSET_CTRL_ADDR;
    wire [9:0]  RX9_OFFSET_CTRL_DATA;
    wire        RX10_OFFSET_CTRL_WRITE;
    wire [2:0]  RX10_OFFSET_CTRL_ADDR;
    wire [9:0]  RX10_OFFSET_CTRL_DATA;
    wire        RX11_OFFSET_CTRL_WRITE;
    wire [2:0]  RX11_OFFSET_CTRL_ADDR;
    wire [9:0]  RX11_OFFSET_CTRL_DATA;
    wire        RX12_OFFSET_CTRL_WRITE;
    wire [2:0]  RX12_OFFSET_CTRL_ADDR;
    wire [9:0]  RX12_OFFSET_CTRL_DATA;
    wire        RX13_OFFSET_CTRL_WRITE;
    wire [2:0]  RX13_OFFSET_CTRL_ADDR;
    wire [9:0]  RX13_OFFSET_CTRL_DATA;
    wire        RX14_OFFSET_CTRL_WRITE;
    wire [2:0]  RX14_OFFSET_CTRL_ADDR;
    wire [9:0]  RX14_OFFSET_CTRL_DATA;
    wire        RX15_OFFSET_CTRL_WRITE;
    wire [2:0]  RX15_OFFSET_CTRL_ADDR;
    wire [9:0]  RX15_OFFSET_CTRL_DATA;
    wire        RX16_OFFSET_CTRL_WRITE;
    wire [2:0]  RX16_OFFSET_CTRL_ADDR;
    wire [9:0]  RX16_OFFSET_CTRL_DATA;
    // vref 段（8 bit）
    wire [7:0]  RX_VREF_CTRL_MAIN;
    wire [7:0]  RX_VREF_CTRL_ERR_EDGE;
    // pi 段（24 bit）
    wire [6:0]  PH_I_GRAY_PI;
    wire [6:0]  PH_Q_GRAY_PI;
    wire [6:0]  PH_045_GRAY_PI;
    // dcdl 段（76 bit）
    wire [5:0]  DCDL_CTRL_RX0;
    wire [5:0]  DCDL_CTRL_RX1;
    wire [5:0]  DCDL_CTRL_RX2;
    wire [5:0]  DCDL_CTRL_RX3;
    wire [5:0]  DCDL_CTRL_RX4;
    wire [5:0]  DCDL_CTRL_RX5;
    wire [5:0]  DCDL_CTRL_RX6;
    wire [5:0]  DCDL_CTRL_RX7;
    wire [5:0]  DCDL_CTRL_RX8;
    wire [5:0]  DCDL_CTRL_RX9;
    wire [5:0]  DCDL_CTRL_RX10;
    wire [5:0]  DCDL_CTRL_RX11;
    wire [5:0]  DCDL_CTRL_RX12;
    wire [5:0]  DCDL_CTRL_RX13;
    wire [5:0]  DCDL_CTRL_RX14;
    wire [5:0]  DCDL_CTRL_RX15;
    wire [5:0]  DCDL_CTRL_RX16;

    // FSM ↔ 执行单元握手
    wire                          offset_train_req;
    wire                          offset_train_done;
    wire                          vref_train_req;
    wire                          vref_train_done;
    wire                          pi_train_req;
    wire                          pi_train_done;
    wire                          dcdl_train_req;
    wire                          dcdl_train_done;
    wire                          data_check_req;
    wire                          data_check_done;

    // data_checker.check_fail_o：spec.json 无路由，接内部 wire 不上顶
    // verilator lint_off UNUSED
    wire                          unused_check_fail;
    // verilator lint_on UNUSED

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================
    // 训练参数打包聚合：{offset 215, vref 8, pi 24, dcdl 76}（MSB→LSB，段内布局见设计说明）
    assign train_param_o = {
        // offset 段 [379:139] = 241 bit（3 + 238；intf v10.3: DATA 8→10，CDR_OFFSET_CTRL 删除）
        RX_OFFSET_CALIBRATION_ON, OFFSET_CTRL_CLK, OFFSET_RESET_N,
        RX16_OFFSET_CTRL_WRITE, RX16_OFFSET_CTRL_ADDR, RX16_OFFSET_CTRL_DATA,
        RX15_OFFSET_CTRL_WRITE, RX15_OFFSET_CTRL_ADDR, RX15_OFFSET_CTRL_DATA,
        RX14_OFFSET_CTRL_WRITE, RX14_OFFSET_CTRL_ADDR, RX14_OFFSET_CTRL_DATA,
        RX13_OFFSET_CTRL_WRITE, RX13_OFFSET_CTRL_ADDR, RX13_OFFSET_CTRL_DATA,
        RX12_OFFSET_CTRL_WRITE, RX12_OFFSET_CTRL_ADDR, RX12_OFFSET_CTRL_DATA,
        RX11_OFFSET_CTRL_WRITE, RX11_OFFSET_CTRL_ADDR, RX11_OFFSET_CTRL_DATA,
        RX10_OFFSET_CTRL_WRITE, RX10_OFFSET_CTRL_ADDR, RX10_OFFSET_CTRL_DATA,
        RX9_OFFSET_CTRL_WRITE,  RX9_OFFSET_CTRL_ADDR,  RX9_OFFSET_CTRL_DATA,
        RX8_OFFSET_CTRL_WRITE,  RX8_OFFSET_CTRL_ADDR,  RX8_OFFSET_CTRL_DATA,
        RX7_OFFSET_CTRL_WRITE,  RX7_OFFSET_CTRL_ADDR,  RX7_OFFSET_CTRL_DATA,
        RX6_OFFSET_CTRL_WRITE,  RX6_OFFSET_CTRL_ADDR,  RX6_OFFSET_CTRL_DATA,
        RX5_OFFSET_CTRL_WRITE,  RX5_OFFSET_CTRL_ADDR,  RX5_OFFSET_CTRL_DATA,
        RX4_OFFSET_CTRL_WRITE,  RX4_OFFSET_CTRL_ADDR,  RX4_OFFSET_CTRL_DATA,
        RX3_OFFSET_CTRL_WRITE,  RX3_OFFSET_CTRL_ADDR,  RX3_OFFSET_CTRL_DATA,
        RX2_OFFSET_CTRL_WRITE,  RX2_OFFSET_CTRL_ADDR,  RX2_OFFSET_CTRL_DATA,
        RX1_OFFSET_CTRL_WRITE,  RX1_OFFSET_CTRL_ADDR,  RX1_OFFSET_CTRL_DATA,
        RX0_OFFSET_CTRL_WRITE,  RX0_OFFSET_CTRL_ADDR,  RX0_OFFSET_CTRL_DATA,
        // vref 段 [138:123] = 16 bit（intf v10.3: MAIN + ERR_EDGE）
        RX_VREF_CTRL_MAIN, RX_VREF_CTRL_ERR_EDGE,
        // pi 段 [122:102] = 21 bit（intf v10.3: 3×7 GRAY，替代 4×6 GXI/GXQ/GYI/GYQ）
        PH_I_GRAY_PI, PH_Q_GRAY_PI, PH_045_GRAY_PI,
        // dcdl 段 [101:0] = 102 bit（intf v10.3: 17×6；v10.4 起 DCDL_CDR×16 删除）
        DCDL_CTRL_RX16, DCDL_CTRL_RX15, DCDL_CTRL_RX14, DCDL_CTRL_RX13,
        DCDL_CTRL_RX12, DCDL_CTRL_RX11, DCDL_CTRL_RX10, DCDL_CTRL_RX9,
        DCDL_CTRL_RX8,  DCDL_CTRL_RX7,  DCDL_CTRL_RX6,  DCDL_CTRL_RX5,
        DCDL_CTRL_RX4,  DCDL_CTRL_RX3,  DCDL_CTRL_RX2,  DCDL_CTRL_RX1,
        DCDL_CTRL_RX0
    };
    // ==========================================================================
    // 子模块实例化
    // ==========================================================================

    // --------------------------------------------------------------------------
    // rx_training_fsm：RX 训练主控 step 调度器
    // --------------------------------------------------------------------------
    rx_training_fsm u_rx_training_fsm (
        .phy_rx_clk                 (phy_rx_clk),
        .rx_rst_n                   (rx_rst_n),
        .self_sync_en               (self_sync_en),
        .aligned_i                  (aligned_o),
        .reorder_trigger            (reorder_trigger),
        .reorder_done               (reorder_done),
        .train_req_o                (train_req_o),
        .retrain_req_i              (retrain_req_i),
        .train_complete_ack_i       (train_complete_ack_i),
        .train_offset_req_i         (train_offset_req_i),
        .train_vref_req_i           (train_vref_req_i),
        .train_eq_req_i             (train_eq_req_i),
        .train_pi_req_i             (train_pi_req_i),
        .train_dcdl_req_i           (train_dcdl_req_i),
        .train_reorder_req_i        (train_reorder_req_i),
        .train_data_check_req_i     (train_data_check_req_i),
        .train_offset_complete_o    (train_offset_complete_o),
        .train_vref_complete_o      (train_vref_complete_o),
        .train_eq_complete_o        (train_eq_complete_o),
        .train_pi_complete_o        (train_pi_complete_o),
        .train_dcdl_complete_o      (train_dcdl_complete_o),
        .train_reorder_complete_o   (train_reorder_complete_o),
        .train_data_check_complete_o(train_data_check_complete_o),
        .train_mode                 (train_mode),
        .force_mode_switch          (force_mode_switch),
        .bypass_step                (bypass_step),
        .timeout_step_en            (timeout_step_en),
        .timeout_threshold          (timeout_threshold),
        .data_check_pattern_sel     (data_check_pattern_sel),
        .step_trigger               (step_trigger),
        .step_force_end             (step_force_end),
        .step_cnt                   (step_cnt),
        .irq_lane_o                 (irq_lane_o),
        .offset_train_req_o         (offset_train_req),
        .offset_train_done_i        (offset_train_done),
        .vref_train_req_o           (vref_train_req),
        .vref_train_done_i          (vref_train_done),
        .pi_train_req_o             (pi_train_req),
        .pi_train_done_i            (pi_train_done),
        .dcdl_train_req_o           (dcdl_train_req),
        .dcdl_train_done_i          (dcdl_train_done),
        .data_check_req_o           (data_check_req),
        .data_check_done_i          (data_check_done)
    );

    // --------------------------------------------------------------------------
    // offset_train：offset 训练执行单元（含 CDR_OFFSET_CTRL）
    // --------------------------------------------------------------------------
    offset_train u_offset_train (
        .phy_rx_clk               (phy_rx_clk),
        .rx_rst_n                 (rx_rst_n),
        .train_req_i              (offset_train_req),
        .train_done_o             (offset_train_done),
        .RX_OFFSET_CALIBRATION_ON (RX_OFFSET_CALIBRATION_ON),
        .OFFSET_CTRL_CLK       (OFFSET_CTRL_CLK),
        .OFFSET_RESET_N        (OFFSET_RESET_N),
        .RX0_OFFSET_CTRL_WRITE    (RX0_OFFSET_CTRL_WRITE),
        .RX0_OFFSET_CTRL_ADDR     (RX0_OFFSET_CTRL_ADDR),
        .RX0_OFFSET_CTRL_DATA     (RX0_OFFSET_CTRL_DATA),
        .RX1_OFFSET_CTRL_WRITE    (RX1_OFFSET_CTRL_WRITE),
        .RX1_OFFSET_CTRL_ADDR     (RX1_OFFSET_CTRL_ADDR),
        .RX1_OFFSET_CTRL_DATA     (RX1_OFFSET_CTRL_DATA),
        .RX2_OFFSET_CTRL_WRITE    (RX2_OFFSET_CTRL_WRITE),
        .RX2_OFFSET_CTRL_ADDR     (RX2_OFFSET_CTRL_ADDR),
        .RX2_OFFSET_CTRL_DATA     (RX2_OFFSET_CTRL_DATA),
        .RX3_OFFSET_CTRL_WRITE    (RX3_OFFSET_CTRL_WRITE),
        .RX3_OFFSET_CTRL_ADDR     (RX3_OFFSET_CTRL_ADDR),
        .RX3_OFFSET_CTRL_DATA     (RX3_OFFSET_CTRL_DATA),
        .RX4_OFFSET_CTRL_WRITE    (RX4_OFFSET_CTRL_WRITE),
        .RX4_OFFSET_CTRL_ADDR     (RX4_OFFSET_CTRL_ADDR),
        .RX4_OFFSET_CTRL_DATA     (RX4_OFFSET_CTRL_DATA),
        .RX5_OFFSET_CTRL_WRITE    (RX5_OFFSET_CTRL_WRITE),
        .RX5_OFFSET_CTRL_ADDR     (RX5_OFFSET_CTRL_ADDR),
        .RX5_OFFSET_CTRL_DATA     (RX5_OFFSET_CTRL_DATA),
        .RX6_OFFSET_CTRL_WRITE    (RX6_OFFSET_CTRL_WRITE),
        .RX6_OFFSET_CTRL_ADDR     (RX6_OFFSET_CTRL_ADDR),
        .RX6_OFFSET_CTRL_DATA     (RX6_OFFSET_CTRL_DATA),
        .RX7_OFFSET_CTRL_WRITE    (RX7_OFFSET_CTRL_WRITE),
        .RX7_OFFSET_CTRL_ADDR     (RX7_OFFSET_CTRL_ADDR),
        .RX7_OFFSET_CTRL_DATA     (RX7_OFFSET_CTRL_DATA),
        .RX8_OFFSET_CTRL_WRITE    (RX8_OFFSET_CTRL_WRITE),
        .RX8_OFFSET_CTRL_ADDR     (RX8_OFFSET_CTRL_ADDR),
        .RX8_OFFSET_CTRL_DATA     (RX8_OFFSET_CTRL_DATA),
        .RX9_OFFSET_CTRL_WRITE    (RX9_OFFSET_CTRL_WRITE),
        .RX9_OFFSET_CTRL_ADDR     (RX9_OFFSET_CTRL_ADDR),
        .RX9_OFFSET_CTRL_DATA     (RX9_OFFSET_CTRL_DATA),
        .RX10_OFFSET_CTRL_WRITE   (RX10_OFFSET_CTRL_WRITE),
        .RX10_OFFSET_CTRL_ADDR    (RX10_OFFSET_CTRL_ADDR),
        .RX10_OFFSET_CTRL_DATA    (RX10_OFFSET_CTRL_DATA),
        .RX11_OFFSET_CTRL_WRITE   (RX11_OFFSET_CTRL_WRITE),
        .RX11_OFFSET_CTRL_ADDR    (RX11_OFFSET_CTRL_ADDR),
        .RX11_OFFSET_CTRL_DATA    (RX11_OFFSET_CTRL_DATA),
        .RX12_OFFSET_CTRL_WRITE   (RX12_OFFSET_CTRL_WRITE),
        .RX12_OFFSET_CTRL_ADDR    (RX12_OFFSET_CTRL_ADDR),
        .RX12_OFFSET_CTRL_DATA    (RX12_OFFSET_CTRL_DATA),
        .RX13_OFFSET_CTRL_WRITE   (RX13_OFFSET_CTRL_WRITE),
        .RX13_OFFSET_CTRL_ADDR    (RX13_OFFSET_CTRL_ADDR),
        .RX13_OFFSET_CTRL_DATA    (RX13_OFFSET_CTRL_DATA),
        .RX14_OFFSET_CTRL_WRITE   (RX14_OFFSET_CTRL_WRITE),
        .RX14_OFFSET_CTRL_ADDR    (RX14_OFFSET_CTRL_ADDR),
        .RX14_OFFSET_CTRL_DATA    (RX14_OFFSET_CTRL_DATA),
        .RX15_OFFSET_CTRL_WRITE   (RX15_OFFSET_CTRL_WRITE),
        .RX15_OFFSET_CTRL_ADDR    (RX15_OFFSET_CTRL_ADDR),
        .RX15_OFFSET_CTRL_DATA    (RX15_OFFSET_CTRL_DATA),
        .RX16_OFFSET_CTRL_WRITE   (RX16_OFFSET_CTRL_WRITE),
        .RX16_OFFSET_CTRL_ADDR    (RX16_OFFSET_CTRL_ADDR),
        .RX16_OFFSET_CTRL_DATA    (RX16_OFFSET_CTRL_DATA),
        .RX0_OFFSET_PARAM ( offset_param[0*16 +: 16] ),
        .RX1_OFFSET_PARAM ( offset_param[1*16 +: 16] ),
        .RX2_OFFSET_PARAM ( offset_param[2*16 +: 16] ),
        .RX3_OFFSET_PARAM ( offset_param[3*16 +: 16] ),
        .RX4_OFFSET_PARAM ( offset_param[4*16 +: 16] ),
        .RX5_OFFSET_PARAM ( offset_param[5*16 +: 16] ),
        .RX6_OFFSET_PARAM ( offset_param[6*16 +: 16] ),
        .RX7_OFFSET_PARAM ( offset_param[7*16 +: 16] ),
        .RX8_OFFSET_PARAM ( offset_param[8*16 +: 16] ),
        .RX9_OFFSET_PARAM ( offset_param[9*16 +: 16] ),
        .RX10_OFFSET_PARAM ( offset_param[10*16 +: 16] ),
        .RX11_OFFSET_PARAM ( offset_param[11*16 +: 16] ),
        .RX12_OFFSET_PARAM ( offset_param[12*16 +: 16] ),
        .RX13_OFFSET_PARAM ( offset_param[13*16 +: 16] ),
        .RX14_OFFSET_PARAM ( offset_param[14*16 +: 16] ),
        .RX15_OFFSET_PARAM ( offset_param[15*16 +: 16] ),
        .RX16_OFFSET_PARAM ( offset_param[16*16 +: 16] )
    );

    // --------------------------------------------------------------------------
    // vref_train：Vref 训练执行单元
    // --------------------------------------------------------------------------
    vref_train u_vref_train (
        .phy_rx_clk   (phy_rx_clk),
        .rx_rst_n     (rx_rst_n),
        .train_req_i  (vref_train_req),
        .train_done_o (vref_train_done),
        .RX_VREF_CTRL_MAIN    (RX_VREF_CTRL_MAIN),
        .RX_VREF_CTRL_ERR_EDGE (RX_VREF_CTRL_ERR_EDGE),
        .RX0_VREF_PARAM ( vref_param[0*16 +: 16] ),
        .RX1_VREF_PARAM ( vref_param[1*16 +: 16] ),
        .RX2_VREF_PARAM ( vref_param[2*16 +: 16] ),
        .RX3_VREF_PARAM ( vref_param[3*16 +: 16] ),
        .RX4_VREF_PARAM ( vref_param[4*16 +: 16] ),
        .RX5_VREF_PARAM ( vref_param[5*16 +: 16] ),
        .RX6_VREF_PARAM ( vref_param[6*16 +: 16] ),
        .RX7_VREF_PARAM ( vref_param[7*16 +: 16] ),
        .RX8_VREF_PARAM ( vref_param[8*16 +: 16] ),
        .RX9_VREF_PARAM ( vref_param[9*16 +: 16] ),
        .RX10_VREF_PARAM ( vref_param[10*16 +: 16] ),
        .RX11_VREF_PARAM ( vref_param[11*16 +: 16] ),
        .RX12_VREF_PARAM ( vref_param[12*16 +: 16] ),
        .RX13_VREF_PARAM ( vref_param[13*16 +: 16] ),
        .RX14_VREF_PARAM ( vref_param[14*16 +: 16] ),
        .RX15_VREF_PARAM ( vref_param[15*16 +: 16] )
    );

    // --------------------------------------------------------------------------
    // pi_train：PI 校准训练执行单元（CDA + I/Q）
    // --------------------------------------------------------------------------
    pi_train u_pi_train (
        .phy_rx_clk   (phy_rx_clk),
        .rx_rst_n     (rx_rst_n),
        .train_req_i  (pi_train_req),
        .train_done_o (pi_train_done),
        .PH_I_GRAY_PI  (PH_I_GRAY_PI),
        .PH_Q_GRAY_PI  (PH_Q_GRAY_PI),
        .PH_045_GRAY_PI (PH_045_GRAY_PI),
        .RX0_PI_PARAM ( pi_param[0*20 +: 20] ),
        .RX1_PI_PARAM ( pi_param[1*20 +: 20] ),
        .RX2_PI_PARAM ( pi_param[2*20 +: 20] ),
        .RX3_PI_PARAM ( pi_param[3*20 +: 20] ),
        .RX4_PI_PARAM ( pi_param[4*20 +: 20] ),
        .RX5_PI_PARAM ( pi_param[5*20 +: 20] ),
        .RX6_PI_PARAM ( pi_param[6*20 +: 20] ),
        .RX7_PI_PARAM ( pi_param[7*20 +: 20] ),
        .RX8_PI_PARAM ( pi_param[8*20 +: 20] ),
        .RX9_PI_PARAM ( pi_param[9*20 +: 20] ),
        .RX10_PI_PARAM ( pi_param[10*20 +: 20] ),
        .RX11_PI_PARAM ( pi_param[11*20 +: 20] ),
        .RX12_PI_PARAM ( pi_param[12*20 +: 20] ),
        .RX13_PI_PARAM ( pi_param[13*20 +: 20] ),
        .RX14_PI_PARAM ( pi_param[14*20 +: 20] ),
        .RX15_PI_PARAM ( pi_param[15*20 +: 20] )
    );

    // --------------------------------------------------------------------------
    // dcdl_train：DCDL（deskew）训练执行单元
    // --------------------------------------------------------------------------
    dcdl_train u_dcdl_train (
        .phy_rx_clk         (phy_rx_clk),
        .rx_rst_n           (rx_rst_n),
        .train_req_i        (dcdl_train_req),
        .train_done_o       (dcdl_train_done),
        .DCDL_CTRL_RX0      (DCDL_CTRL_RX0),
        .DCDL_CTRL_RX1      (DCDL_CTRL_RX1),
        .DCDL_CTRL_RX2      (DCDL_CTRL_RX2),
        .DCDL_CTRL_RX3      (DCDL_CTRL_RX3),
        .DCDL_CTRL_RX4      (DCDL_CTRL_RX4),
        .DCDL_CTRL_RX5      (DCDL_CTRL_RX5),
        .DCDL_CTRL_RX6      (DCDL_CTRL_RX6),
        .DCDL_CTRL_RX7      (DCDL_CTRL_RX7),
        .DCDL_CTRL_RX8      (DCDL_CTRL_RX8),
        .DCDL_CTRL_RX9      (DCDL_CTRL_RX9),
        .DCDL_CTRL_RX10     (DCDL_CTRL_RX10),
        .DCDL_CTRL_RX11     (DCDL_CTRL_RX11),
        .DCDL_CTRL_RX12     (DCDL_CTRL_RX12),
        .DCDL_CTRL_RX13     (DCDL_CTRL_RX13),
        .DCDL_CTRL_RX14     (DCDL_CTRL_RX14),
        .DCDL_CTRL_RX15     (DCDL_CTRL_RX15),
        .DCDL_CTRL_RX16     (DCDL_CTRL_RX16),
        .RX0_DCDL_PARAM ( dcdl_param[0*12 +: 12] ),
        .RX1_DCDL_PARAM ( dcdl_param[1*12 +: 12] ),
        .RX2_DCDL_PARAM ( dcdl_param[2*12 +: 12] ),
        .RX3_DCDL_PARAM ( dcdl_param[3*12 +: 12] ),
        .RX4_DCDL_PARAM ( dcdl_param[4*12 +: 12] ),
        .RX5_DCDL_PARAM ( dcdl_param[5*12 +: 12] ),
        .RX6_DCDL_PARAM ( dcdl_param[6*12 +: 12] ),
        .RX7_DCDL_PARAM ( dcdl_param[7*12 +: 12] ),
        .RX8_DCDL_PARAM ( dcdl_param[8*12 +: 12] ),
        .RX9_DCDL_PARAM ( dcdl_param[9*12 +: 12] ),
        .RX10_DCDL_PARAM ( dcdl_param[10*12 +: 12] ),
        .RX11_DCDL_PARAM ( dcdl_param[11*12 +: 12] ),
        .RX12_DCDL_PARAM ( dcdl_param[12*12 +: 12] ),
        .RX13_DCDL_PARAM ( dcdl_param[13*12 +: 12] ),
        .RX14_DCDL_PARAM ( dcdl_param[14*12 +: 12] ),
        .RX15_DCDL_PARAM ( dcdl_param[15*12 +: 12] ),
        .RX16_DCDL_PARAM ( dcdl_param[16*12 +: 12] )
    );

    // --------------------------------------------------------------------------
    // data_checker：训练后 data check
    // --------------------------------------------------------------------------
    data_checker #(
        .DATA_WIDTH (DATA_WIDTH)
    ) u_data_checker (
        .phy_rx_clk            (phy_rx_clk),
        .rx_rst_n              (rx_rst_n),
        .check_req_i           (data_check_req),
        .data_i                (data_i),
        .prbs_seq_i            (prbs_seq_i),
        .data_check_pattern_sel(data_check_pattern_sel),
        .check_done_o          (data_check_done),
        .check_fail_o          (unused_check_fail),
        .FAIL_THRESHOLD_i    (fail_threshold_i)
    );


    // --------------------------------------------------------------------------
    // 族 c #3/#7 hook_min（MDE-CHAIN-RERUN-01-271-IMPL）：train_req_o 沿采样
    //   train_param_src_i / ctle_tap_i 入影子（结构消费占位, 供 TRAIN-MAS-01 消费）
    // --------------------------------------------------------------------------
    reg         train_param_src_shd;
    reg  [3:0]  ctle_tap_shd;
    always @(posedge phy_rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            train_param_src_shd <= 1'b0;
            ctle_tap_shd        <= 4'h0;
        end
        else if (train_req_o) begin
            train_param_src_shd <= train_param_src_i;
            ctle_tap_shd        <= ctle_tap_i;
        end
    end

endmodule
