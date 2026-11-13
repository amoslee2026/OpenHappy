// =============================================================================
// module name：vref_train
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/rx_chain/submods/rx_training/mas/vref_train_mas.md（待编写）
// =============================================================================
// 功能概述: Vref 训练执行单元（pattern 默认全 1）：端口 stub，功能待 MAS 编写后补充；
//   与 rx_training_fsm 走 valid-ready req/done 握手（phy_rx_clk 同域）
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-11  OpenHappy Team  输出端口由打包 train_param_o 拆分为 interface 具名信号
//                       （大写原文：RX_VREF_CTRL_MAIN/RX_VREF_CTRL_ERR_EDGE[7:0]，intf v10.3）；打包聚合上移到 rx_training 层
//  2026-08-07  OpenHappy Team  初始版本：端口 stub（MAS 待编写），功能未实现
//  2026-09-11  OpenHappy Team  MDE-CHAIN-RERUN-01-271-IMPL 族 a §1.4 hook：新增 per-IO 训练参数
//                       输入 RX0~15_VREF_PARAM + train_req_i 沿采样影子 VREF_PARAM_SHDW
//                       （结构消费占位，供 TRAIN-MAS-01 per-IO sweep 消费）
// =============================================================================

`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// 设计说明:
//
// 端口依据:
//   - rx_training_spec.json v1.2 internal_connections（fsm ↔ vref_train 握手）
//   - interface/logicphy_analogphy_intf.md v1.1 §训练校准参数（端口名保持 interface
//     大写原文，per-lane 内去 lane{i}_ 前缀）：
//       RX_VREF_CTRL_MAIN[7:0] + RX_VREF_CTRL_ERR_EDGE[7:0]（per-lane Vref 训练参数，intf v10.3）
// -----------------------------------------------------------------------------

module vref_train (
    input  wire        phy_rx_clk,
    input  wire        rx_rst_n,
    input  wire        train_req_i,             // ← rx_training_fsm.vref_train_req_o
    output wire        train_done_o,            // → rx_training_fsm.vref_train_done_i

    // 训练参数输出（→ rx_training 打包聚合；信号定义见 logicphy_analogphy_intf v1.1）
    output wire [7:0]  RX_VREF_CTRL_MAIN,          // Vref 训练参数（MAIN，intf v10.3 自 RX_VREF_CTRL 改名）
    output wire [7:0]  RX_VREF_CTRL_ERR_EDGE,    // Vref 训练参数（ERR_EDGE，intf v10.3 新增）

    // 族 a lane0 per-IO VREF 训练参数（MDE-CHAIN-RERUN-01-271-IMPL; RX0~15）
    input  wire [15:0] RX0_VREF_PARAM,
    input  wire [15:0] RX1_VREF_PARAM,
    input  wire [15:0] RX2_VREF_PARAM,
    input  wire [15:0] RX3_VREF_PARAM,
    input  wire [15:0] RX4_VREF_PARAM,
    input  wire [15:0] RX5_VREF_PARAM,
    input  wire [15:0] RX6_VREF_PARAM,
    input  wire [15:0] RX7_VREF_PARAM,
    input  wire [15:0] RX8_VREF_PARAM,
    input  wire [15:0] RX9_VREF_PARAM,
    input  wire [15:0] RX10_VREF_PARAM,
    input  wire [15:0] RX11_VREF_PARAM,
    input  wire [15:0] RX12_VREF_PARAM,
    input  wire [15:0] RX13_VREF_PARAM,
    input  wire [15:0] RX14_VREF_PARAM,
    input  wire [15:0] RX15_VREF_PARAM
);

    // ==========================================================================
    // 族 a §1.4 hook（MDE-CHAIN-RERUN-01-271-IMPL）：train_req_i 脉冲拍采样 per-IO
    //   VREF_PARAM 入影子 VREF_PARAM_SHDW（结构消费占位，供 TRAIN-MAS-01 per-IO sweep）
    //   RX_VREF_CTRL_MAIN/ERR_EDGE 聚合输出保持 stub 0（MAS 待编写；ERR_EDGE 独立源 TODO）
    // ==========================================================================
    assign train_done_o = 1'b0;
    assign RX_VREF_CTRL_MAIN       = 8'b0;
    assign RX_VREF_CTRL_ERR_EDGE = RX_VREF_CTRL_MAIN; // TODO(WP1/TRAIN-MAS): 独立参数源待 OpenHappy Team 确认，暂与 MAIN 同值

    reg [15:0] VREF_PARAM_SHDW [0:15];
    integer vi;
    always @(posedge phy_rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            for (vi = 0; vi < 16; vi = vi + 1) VREF_PARAM_SHDW[vi] <= 16'b0;
        end
        else if (train_req_i) begin
            for (vi = 0; vi < 16; vi = vi + 1) begin
                case (vi)
                    0:  VREF_PARAM_SHDW[vi] <= RX0_VREF_PARAM;
                    1:  VREF_PARAM_SHDW[vi] <= RX1_VREF_PARAM;
                    2:  VREF_PARAM_SHDW[vi] <= RX2_VREF_PARAM;
                    3:  VREF_PARAM_SHDW[vi] <= RX3_VREF_PARAM;
                    4:  VREF_PARAM_SHDW[vi] <= RX4_VREF_PARAM;
                    5:  VREF_PARAM_SHDW[vi] <= RX5_VREF_PARAM;
                    6:  VREF_PARAM_SHDW[vi] <= RX6_VREF_PARAM;
                    7:  VREF_PARAM_SHDW[vi] <= RX7_VREF_PARAM;
                    8:  VREF_PARAM_SHDW[vi] <= RX8_VREF_PARAM;
                    9:  VREF_PARAM_SHDW[vi] <= RX9_VREF_PARAM;
                    10: VREF_PARAM_SHDW[vi] <= RX10_VREF_PARAM;
                    11: VREF_PARAM_SHDW[vi] <= RX11_VREF_PARAM;
                    12: VREF_PARAM_SHDW[vi] <= RX12_VREF_PARAM;
                    13: VREF_PARAM_SHDW[vi] <= RX13_VREF_PARAM;
                    14: VREF_PARAM_SHDW[vi] <= RX14_VREF_PARAM;
                    15: VREF_PARAM_SHDW[vi] <= RX15_VREF_PARAM;
                    default: ;
                endcase
            end
        end
    end

endmodule
