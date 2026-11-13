// =============================================================================
// module name：dcdl_train
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/rx_chain/submods/rx_training/mas/dcdl_train_mas.md（待编写）
// =============================================================================
// 功能概述: DCDL（deskew）训练执行单元（国标 RX DESKEW 指令，pattern 可配）：端口 stub，
//   功能待 MAS 编写后补充；与 rx_training_fsm 走 valid-ready req/done 握手（phy_rx_clk 同域）
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-11  OpenHappy Team  输出端口由打包 train_param_o 拆分为 interface 具名信号
//                       （大写原文：DCDL_CTRL_RX0~RX16[5:0]，intf v10.3 4→6 bit；v10.4 起 DCDL_CDR 删除）；打包聚合上移到 rx_training 层
//  2026-08-07  OpenHappy Team  初始版本：端口 stub（MAS 待编写），功能未实现
//  2026-09-11  OpenHappy Team  MDE-CHAIN-RERUN-01-271-IMPL 族 a §1.4 hook：新增 per-IO 训练参数
//                       输入 RX0~15_DCDL_PARAM，train_req_i 沿采样低字节→DCDL_CTRL_RX{i}
//                       （结构消费占位，供 TRAIN-MAS-01 per-IO sweep 消费）
// =============================================================================

`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// 设计说明:
//
// 端口依据:
//   - rx_training_spec.json v1.2 internal_connections（fsm ↔ dcdl_train 握手）
//   - interface/logicphy_analogphy_intf.md v1.1 §训练校准参数（端口名保持 interface
//     大写原文，per-lane 内去 lane{i}_ 前缀）：
//       per-IO DCDL_CTRL_RX0~RX16 各 6 bit（17 × 6 = 102，RX16 为冗余 IO；intf v10.3: 4→6 bit）
//       （intf v10.4: DCDL_CDR×16 已删，CDR 相关输出清零）
//   - 注：interface 另有 lane{i}_RX16_L_CDR[3:0] 为 analog → logic 反馈输入，
//     本 stub 未包含该输入端口，待 MAS 补充
// -----------------------------------------------------------------------------

module dcdl_train (
    input  wire        phy_rx_clk,
    input  wire        rx_rst_n,
    input  wire        train_req_i,             // ← rx_training_fsm.dcdl_train_req_o
    output wire        train_done_o,            // → rx_training_fsm.dcdl_train_done_i

    // 训练参数输出（→ rx_training 打包聚合；信号定义见 logicphy_analogphy_intf v1.1）
    output reg  [5:0]  DCDL_CTRL_RX0,               // RX0 DCDL 训练参数
    output reg  [5:0]  DCDL_CTRL_RX1,
    output reg  [5:0]  DCDL_CTRL_RX2,
    output reg  [5:0]  DCDL_CTRL_RX3,
    output reg  [5:0]  DCDL_CTRL_RX4,
    output reg  [5:0]  DCDL_CTRL_RX5,
    output reg  [5:0]  DCDL_CTRL_RX6,
    output reg  [5:0]  DCDL_CTRL_RX7,
    output reg  [5:0]  DCDL_CTRL_RX8,
    output reg  [5:0]  DCDL_CTRL_RX9,
    output reg  [5:0]  DCDL_CTRL_RX10,
    output reg  [5:0]  DCDL_CTRL_RX11,
    output reg  [5:0]  DCDL_CTRL_RX12,
    output reg  [5:0]  DCDL_CTRL_RX13,
    output reg  [5:0]  DCDL_CTRL_RX14,
    output reg  [5:0]  DCDL_CTRL_RX15,
    output reg  [5:0]  DCDL_CTRL_RX16,              // RX16 DCDL 训练参数（冗余 IO；intf v10.3: 4→6 bit，WP1 起由 RX16_DCDL_PARAM 驱动）

    // 族 a lane0 per-IO DCDL 训练参数（MDE-CHAIN-RERUN-01-271-IMPL; RX0~15）
    input  wire [11:0] RX0_DCDL_PARAM,
    input  wire [11:0] RX1_DCDL_PARAM,
    input  wire [11:0] RX2_DCDL_PARAM,
    input  wire [11:0] RX3_DCDL_PARAM,
    input  wire [11:0] RX4_DCDL_PARAM,
    input  wire [11:0] RX5_DCDL_PARAM,
    input  wire [11:0] RX6_DCDL_PARAM,
    input  wire [11:0] RX7_DCDL_PARAM,
    input  wire [11:0] RX8_DCDL_PARAM,
    input  wire [11:0] RX9_DCDL_PARAM,
    input  wire [11:0] RX10_DCDL_PARAM,
    input  wire [11:0] RX11_DCDL_PARAM,
    input  wire [11:0] RX12_DCDL_PARAM,
    input  wire [11:0] RX13_DCDL_PARAM,
    input  wire [11:0] RX14_DCDL_PARAM,
    input  wire [11:0] RX15_DCDL_PARAM,
    input  wire [11:0] RX16_DCDL_PARAM     // i16（RX16 冗余 IO，WP1 P0-A 2026-09-22）
);

    // ==========================================================================
    // 族 a §1.4 hook（MDE-CHAIN-RERUN-01-271-IMPL）：train_req_i 脉冲拍采样 per-IO
    //   DCDL_PARAM 低字节 → DCDL_CTRL_RX{i}（结构消费占位；per-IO sweep 完整映射
    //   属 TRAIN-MAS-01）
    //   DCDL_CTRL_RX16（冗余）由 RX16_DCDL_PARAM 驱动（WP1 2026-09-22；原 stub 0）
    // ==========================================================================
    assign train_done_o = 1'b0;
            
    always @(posedge phy_rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            DCDL_CTRL_RX0 <= 6'b0;
            DCDL_CTRL_RX1 <= 6'b0;
            DCDL_CTRL_RX2 <= 6'b0;
            DCDL_CTRL_RX3 <= 6'b0;
            DCDL_CTRL_RX4 <= 6'b0;
            DCDL_CTRL_RX5 <= 6'b0;
            DCDL_CTRL_RX6 <= 6'b0;
            DCDL_CTRL_RX7 <= 6'b0;
            DCDL_CTRL_RX8 <= 6'b0;
            DCDL_CTRL_RX9 <= 6'b0;
            DCDL_CTRL_RX10 <= 6'b0;
            DCDL_CTRL_RX11 <= 6'b0;
            DCDL_CTRL_RX12 <= 6'b0;
            DCDL_CTRL_RX13 <= 6'b0;
            DCDL_CTRL_RX14 <= 6'b0;
            DCDL_CTRL_RX15 <= 6'b0;
            DCDL_CTRL_RX16 <= 6'b0;
        end
        else if (train_req_i) begin
            DCDL_CTRL_RX0 <= RX0_DCDL_PARAM[5:0];
            DCDL_CTRL_RX1 <= RX1_DCDL_PARAM[5:0];
            DCDL_CTRL_RX2 <= RX2_DCDL_PARAM[5:0];
            DCDL_CTRL_RX3 <= RX3_DCDL_PARAM[5:0];
            DCDL_CTRL_RX4 <= RX4_DCDL_PARAM[5:0];
            DCDL_CTRL_RX5 <= RX5_DCDL_PARAM[5:0];
            DCDL_CTRL_RX6 <= RX6_DCDL_PARAM[5:0];
            DCDL_CTRL_RX7 <= RX7_DCDL_PARAM[5:0];
            DCDL_CTRL_RX8 <= RX8_DCDL_PARAM[5:0];
            DCDL_CTRL_RX9 <= RX9_DCDL_PARAM[5:0];
            DCDL_CTRL_RX10 <= RX10_DCDL_PARAM[5:0];
            DCDL_CTRL_RX11 <= RX11_DCDL_PARAM[5:0];
            DCDL_CTRL_RX12 <= RX12_DCDL_PARAM[5:0];
            DCDL_CTRL_RX13 <= RX13_DCDL_PARAM[5:0];
            DCDL_CTRL_RX14 <= RX14_DCDL_PARAM[5:0];
            DCDL_CTRL_RX15 <= RX15_DCDL_PARAM[5:0];
            DCDL_CTRL_RX16 <= RX16_DCDL_PARAM[5:0];
        end
    end

endmodule
