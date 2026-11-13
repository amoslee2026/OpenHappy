// =============================================================================
// module name：pi_train
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/rx_chain/submods/rx_training/mas/pi_train_mas.md（待编写）
// =============================================================================
// 功能概述: PI 校准训练执行单元（CDA + I/Q 合并，国标 RX CDA 指令，pattern 可配）：端口 stub，
//   功能待 MAS 编写后补充；与 rx_training_fsm 走 valid-ready req/done 握手（phy_rx_clk 同域）
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-11  OpenHappy Team  输出端口由打包 train_param_o 拆分为 interface 具名信号
//                       （大写原文：PH_I_GRAY_PI/PH_Q_GRAY_PI/PH_045_GRAY_PI[6:0]）；打包聚合上移到
//                       rx_training 层
//  2026-08-07  OpenHappy Team  初始版本：端口 stub（MAS 待编写），功能未实现
//  2026-09-11  OpenHappy Team  MDE-CHAIN-RERUN-01-271-IMPL 族 a §1.4 hook：新增 per-IO 训练参数
//                       输入 RX0~15_PI_PARAM + train_req_i 沿采样影子 PI_PARAM_SHDW
//                       （结构消费占位，供 TRAIN-MAS-01 per-IO sweep 消费）
// =============================================================================

`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// 设计说明:
//
// 端口依据:
//   - rx_training_spec.json v1.2 internal_connections（fsm ↔ pi_train 握手）
//   - interface/logicphy_analogphy_intf.md v1.1 §训练校准参数（端口名保持 interface
//     大写原文，per-lane 内去 lane{i}_ 前缀）：
//       PH_I_GRAY_PI/PH_Q_GRAY_PI/PH_045_GRAY_PI 各 7 bit GRAY 码（intf v10.3），合计 21 bit
// -----------------------------------------------------------------------------

module pi_train (
    input  wire        phy_rx_clk,
    input  wire        rx_rst_n,
    input  wire        train_req_i,             // ← rx_training_fsm.pi_train_req_o
    output wire        train_done_o,            // → rx_training_fsm.pi_train_done_i

    // 训练参数输出（→ rx_training 打包聚合；信号定义见 logicphy_analogphy_intf v1.1）
    output wire [6:0]  PH_I_GRAY_PI,                // PI 校准 I 路参数（7 bit GRAY 码，intf v10.3 替代 RX_PI_GXI/GYI）
    output wire [6:0]  PH_Q_GRAY_PI,                // PI 校准 Q 路参数（7 bit GRAY 码，intf v10.3 替代 RX_PI_GXQ/GYQ）
    output wire [6:0]  PH_045_GRAY_PI,            // PI 校准 45° 路参数（7 bit GRAY 码，intf v10.3 新增）

    // 族 a lane0 per-IO PI 训练参数（MDE-CHAIN-RERUN-01-271-IMPL; RX0~15）
    input  wire [19:0] RX0_PI_PARAM,
    input  wire [19:0] RX1_PI_PARAM,
    input  wire [19:0] RX2_PI_PARAM,
    input  wire [19:0] RX3_PI_PARAM,
    input  wire [19:0] RX4_PI_PARAM,
    input  wire [19:0] RX5_PI_PARAM,
    input  wire [19:0] RX6_PI_PARAM,
    input  wire [19:0] RX7_PI_PARAM,
    input  wire [19:0] RX8_PI_PARAM,
    input  wire [19:0] RX9_PI_PARAM,
    input  wire [19:0] RX10_PI_PARAM,
    input  wire [19:0] RX11_PI_PARAM,
    input  wire [19:0] RX12_PI_PARAM,
    input  wire [19:0] RX13_PI_PARAM,
    input  wire [19:0] RX14_PI_PARAM,
    input  wire [19:0] RX15_PI_PARAM
);

    // ==========================================================================
    // 族 a §1.4 hook（MDE-CHAIN-RERUN-01-271-IMPL）：train_req_i 脉冲拍采样 per-IO
    //   PI_PARAM 入影子 PI_PARAM_SHDW（结构消费占位，供 TRAIN-MAS-01 per-IO sweep）
    //   聚合 GXI/GXQ/GYI/GYQ 保持 stub 0（MAS 待编写）
    // ==========================================================================
    assign train_done_o = 1'b0;
    // TODO(WP1/TRAIN-MAS): GRAY 编码表待 OpenHappy Team 确认，暂 stub 0（原 4×6b 增益 stub 语义保持）
    assign {PH_045_GRAY_PI, PH_Q_GRAY_PI, PH_I_GRAY_PI} = 21'b0;

    reg [19:0] PI_PARAM_SHDW [0:15];
    integer pi_i;
    always @(posedge phy_rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            for (pi_i = 0; pi_i < 16; pi_i = pi_i + 1) PI_PARAM_SHDW[pi_i] <= 20'b0;
        end
        else if (train_req_i) begin
            for (pi_i = 0; pi_i < 16; pi_i = pi_i + 1) begin
                case (pi_i)
                    0:  PI_PARAM_SHDW[pi_i] <= RX0_PI_PARAM;
                    1:  PI_PARAM_SHDW[pi_i] <= RX1_PI_PARAM;
                    2:  PI_PARAM_SHDW[pi_i] <= RX2_PI_PARAM;
                    3:  PI_PARAM_SHDW[pi_i] <= RX3_PI_PARAM;
                    4:  PI_PARAM_SHDW[pi_i] <= RX4_PI_PARAM;
                    5:  PI_PARAM_SHDW[pi_i] <= RX5_PI_PARAM;
                    6:  PI_PARAM_SHDW[pi_i] <= RX6_PI_PARAM;
                    7:  PI_PARAM_SHDW[pi_i] <= RX7_PI_PARAM;
                    8:  PI_PARAM_SHDW[pi_i] <= RX8_PI_PARAM;
                    9:  PI_PARAM_SHDW[pi_i] <= RX9_PI_PARAM;
                    10: PI_PARAM_SHDW[pi_i] <= RX10_PI_PARAM;
                    11: PI_PARAM_SHDW[pi_i] <= RX11_PI_PARAM;
                    12: PI_PARAM_SHDW[pi_i] <= RX12_PI_PARAM;
                    13: PI_PARAM_SHDW[pi_i] <= RX13_PI_PARAM;
                    14: PI_PARAM_SHDW[pi_i] <= RX14_PI_PARAM;
                    15: PI_PARAM_SHDW[pi_i] <= RX15_PI_PARAM;
                    default: ;
                endcase
            end
        end
    end

endmodule
