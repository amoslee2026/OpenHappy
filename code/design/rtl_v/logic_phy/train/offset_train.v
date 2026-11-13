// =============================================================================
// module name：offset_train
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/rx_chain/submods/rx_training/mas/offset_train_mas.md（待编写）
// =============================================================================
// 功能概述: offset 训练执行单元（per-IO 串行下发参数 sweep）：端口 stub，功能待 MAS 编写后补充；
//   与 rx_training_fsm 走 valid-ready req/done 握手（phy_rx_clk 同域）
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-11  OpenHappy Team  输出端口由打包 train_param_o 拆分为 interface 具名信号
//                       （大写原文，对齐 logicphy_analogphy_intf v1.1 §训练校准参数：
//                       per-lane 全局 3 + 17 IO × WRITE/ADDR/DATA[9:0]，intf v10.3：CDR_OFFSET_CTRL 删除）；
//                       打包聚合上移到 rx_training 层
//  2026-08-07  OpenHappy Team  初始版本：端口 stub（MAS 待编写），功能未实现
// =============================================================================

`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// 设计说明:
//
// 端口依据:
//   - rx_training_spec.json v1.2 internal_connections（fsm ↔ offset_train 握手）
//   - interface/logicphy_analogphy_intf.md v1.1 §训练校准参数（端口名保持 interface
//     大写原文，per-lane 内去 lane{i}_ 前缀）：
//       per-lane 全局：RX_OFFSET_CALIBRATION_ON / RX_OFFSET_CTRL_CLK / RX_OFFSET_RESET_N
//       per-IO（RX0~RX16，RX16 为冗余 IO）：RX{j}_OFFSET_CTRL_WRITE / ADDR[2:0] / DATA[9:0]（intf v10.3: 8→10 bit）
//   - 注：interface 中 per-IO OFFSET_CTRL_DATA 为 inout 8 bit（读回路径），
//     本模块 DATA 端口仅为写侧输出；读回数据输入端口待 MAS 补充
// -----------------------------------------------------------------------------

module offset_train (
    input  wire        phy_rx_clk,
    input  wire        rx_rst_n,
    input  wire        train_req_i,             // ← rx_training_fsm.offset_train_req_o
    output wire        train_done_o,            // → rx_training_fsm.offset_train_done_i

    // 训练参数输出（→ rx_training 打包聚合；信号定义见 logicphy_analogphy_intf v1.1）
    output wire        RX_OFFSET_CALIBRATION_ON,    // offset 训练使能
    output wire        OFFSET_CTRL_CLK,            // offset 训练时钟（intf v10.3: 无 RX_ 前缀，匹配模型 OFFSET_CTRL_CLK）
    output wire        OFFSET_RESET_N,             // offset 训练复位（intf v10.3: 无 RX_ 前缀，匹配模型 OFFSET_RESET_N）

    // per-IO offset 训练参数（RX0~RX16，RX16 为冗余 IO；RX0~15 DATA 由族 a hook 驱动）
    output wire        RX0_OFFSET_CTRL_WRITE,
    output wire [2:0]  RX0_OFFSET_CTRL_ADDR,
    output reg  [9:0]  RX0_OFFSET_CTRL_DATA,
    output wire        RX1_OFFSET_CTRL_WRITE,
    output wire [2:0]  RX1_OFFSET_CTRL_ADDR,
    output reg  [9:0]  RX1_OFFSET_CTRL_DATA,
    output wire        RX2_OFFSET_CTRL_WRITE,
    output wire [2:0]  RX2_OFFSET_CTRL_ADDR,
    output reg  [9:0]  RX2_OFFSET_CTRL_DATA,
    output wire        RX3_OFFSET_CTRL_WRITE,
    output wire [2:0]  RX3_OFFSET_CTRL_ADDR,
    output reg  [9:0]  RX3_OFFSET_CTRL_DATA,
    output wire        RX4_OFFSET_CTRL_WRITE,
    output wire [2:0]  RX4_OFFSET_CTRL_ADDR,
    output reg  [9:0]  RX4_OFFSET_CTRL_DATA,
    output wire        RX5_OFFSET_CTRL_WRITE,
    output wire [2:0]  RX5_OFFSET_CTRL_ADDR,
    output reg  [9:0]  RX5_OFFSET_CTRL_DATA,
    output wire        RX6_OFFSET_CTRL_WRITE,
    output wire [2:0]  RX6_OFFSET_CTRL_ADDR,
    output reg  [9:0]  RX6_OFFSET_CTRL_DATA,
    output wire        RX7_OFFSET_CTRL_WRITE,
    output wire [2:0]  RX7_OFFSET_CTRL_ADDR,
    output reg  [9:0]  RX7_OFFSET_CTRL_DATA,
    output wire        RX8_OFFSET_CTRL_WRITE,
    output wire [2:0]  RX8_OFFSET_CTRL_ADDR,
    output reg  [9:0]  RX8_OFFSET_CTRL_DATA,
    output wire        RX9_OFFSET_CTRL_WRITE,
    output wire [2:0]  RX9_OFFSET_CTRL_ADDR,
    output reg  [9:0]  RX9_OFFSET_CTRL_DATA,
    output wire        RX10_OFFSET_CTRL_WRITE,
    output wire [2:0]  RX10_OFFSET_CTRL_ADDR,
    output reg  [9:0]  RX10_OFFSET_CTRL_DATA,
    output wire        RX11_OFFSET_CTRL_WRITE,
    output wire [2:0]  RX11_OFFSET_CTRL_ADDR,
    output reg  [9:0]  RX11_OFFSET_CTRL_DATA,
    output wire        RX12_OFFSET_CTRL_WRITE,
    output wire [2:0]  RX12_OFFSET_CTRL_ADDR,
    output reg  [9:0]  RX12_OFFSET_CTRL_DATA,
    output wire        RX13_OFFSET_CTRL_WRITE,
    output wire [2:0]  RX13_OFFSET_CTRL_ADDR,
    output reg  [9:0]  RX13_OFFSET_CTRL_DATA,
    output wire        RX14_OFFSET_CTRL_WRITE,
    output wire [2:0]  RX14_OFFSET_CTRL_ADDR,
    output reg  [9:0]  RX14_OFFSET_CTRL_DATA,
    output wire        RX15_OFFSET_CTRL_WRITE,
    output wire [2:0]  RX15_OFFSET_CTRL_ADDR,
    output reg  [9:0]  RX15_OFFSET_CTRL_DATA,
    output wire        RX16_OFFSET_CTRL_WRITE,
    output wire [2:0]  RX16_OFFSET_CTRL_ADDR,
    output reg  [9:0]  RX16_OFFSET_CTRL_DATA,

    // 族 a lane0 per-IO offset 训练参数（MDE-CHAIN-RERUN-01-271-IMPL; RX0~16）
    input  wire [15:0] RX0_OFFSET_PARAM,
    input  wire [15:0] RX1_OFFSET_PARAM,
    input  wire [15:0] RX2_OFFSET_PARAM,
    input  wire [15:0] RX3_OFFSET_PARAM,
    input  wire [15:0] RX4_OFFSET_PARAM,
    input  wire [15:0] RX5_OFFSET_PARAM,
    input  wire [15:0] RX6_OFFSET_PARAM,
    input  wire [15:0] RX7_OFFSET_PARAM,
    input  wire [15:0] RX8_OFFSET_PARAM,
    input  wire [15:0] RX9_OFFSET_PARAM,
    input  wire [15:0] RX10_OFFSET_PARAM,
    input  wire [15:0] RX11_OFFSET_PARAM,
    input  wire [15:0] RX12_OFFSET_PARAM,
    input  wire [15:0] RX13_OFFSET_PARAM,
    input  wire [15:0] RX14_OFFSET_PARAM,
    input  wire [15:0] RX15_OFFSET_PARAM,
    input  wire [15:0] RX16_OFFSET_PARAM   // i16（RX16 冗余 IO，WP1 P0-A 2026-09-22；apb_csr_top 0x0E8）
);

    // ==========================================================================
    // 族 a §1.4 hook（MDE-271-IMPL）：train_req_i 脉冲拍采样 per-IO OFFSET_PARAM
    //   低字节 → OFFSET_CTRL_DATA（结构消费占位；per-IO sweep 完整映射属 TRAIN-MAS-01）
    // 其余输出（WRITE/ADDR/CDR/CAL/RX16）保持 stub 安全常值 0（MAS 待编写）
    // ==========================================================================
    assign train_done_o = 1'b0;
    assign RX_OFFSET_CALIBRATION_ON = 1'b0;
    assign OFFSET_CTRL_CLK          = 1'b0;
    assign OFFSET_RESET_N           = 1'b0;
    assign RX0_OFFSET_CTRL_WRITE  = 1'b0;
    assign RX0_OFFSET_CTRL_ADDR   = 3'b0;
    assign RX1_OFFSET_CTRL_WRITE  = 1'b0;
    assign RX1_OFFSET_CTRL_ADDR   = 3'b0;
    assign RX2_OFFSET_CTRL_WRITE  = 1'b0;
    assign RX2_OFFSET_CTRL_ADDR   = 3'b0;
    assign RX3_OFFSET_CTRL_WRITE  = 1'b0;
    assign RX3_OFFSET_CTRL_ADDR   = 3'b0;
    assign RX4_OFFSET_CTRL_WRITE  = 1'b0;
    assign RX4_OFFSET_CTRL_ADDR   = 3'b0;
    assign RX5_OFFSET_CTRL_WRITE  = 1'b0;
    assign RX5_OFFSET_CTRL_ADDR   = 3'b0;
    assign RX6_OFFSET_CTRL_WRITE  = 1'b0;
    assign RX6_OFFSET_CTRL_ADDR   = 3'b0;
    assign RX7_OFFSET_CTRL_WRITE  = 1'b0;
    assign RX7_OFFSET_CTRL_ADDR   = 3'b0;
    assign RX8_OFFSET_CTRL_WRITE  = 1'b0;
    assign RX8_OFFSET_CTRL_ADDR   = 3'b0;
    assign RX9_OFFSET_CTRL_WRITE  = 1'b0;
    assign RX9_OFFSET_CTRL_ADDR   = 3'b0;
    assign RX10_OFFSET_CTRL_WRITE  = 1'b0;
    assign RX10_OFFSET_CTRL_ADDR   = 3'b0;
    assign RX11_OFFSET_CTRL_WRITE  = 1'b0;
    assign RX11_OFFSET_CTRL_ADDR   = 3'b0;
    assign RX12_OFFSET_CTRL_WRITE  = 1'b0;
    assign RX12_OFFSET_CTRL_ADDR   = 3'b0;
    assign RX13_OFFSET_CTRL_WRITE  = 1'b0;
    assign RX13_OFFSET_CTRL_ADDR   = 3'b0;
    assign RX14_OFFSET_CTRL_WRITE  = 1'b0;
    assign RX14_OFFSET_CTRL_ADDR   = 3'b0;
    assign RX15_OFFSET_CTRL_WRITE  = 1'b0;
    assign RX15_OFFSET_CTRL_ADDR   = 3'b0;
    assign RX16_OFFSET_CTRL_WRITE  = 1'b0;
    assign RX16_OFFSET_CTRL_ADDR   = 3'b0;
    
    always @(posedge phy_rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            RX0_OFFSET_CTRL_DATA <= 10'b0;
            RX1_OFFSET_CTRL_DATA <= 10'b0;
            RX2_OFFSET_CTRL_DATA <= 10'b0;
            RX3_OFFSET_CTRL_DATA <= 10'b0;
            RX4_OFFSET_CTRL_DATA <= 10'b0;
            RX5_OFFSET_CTRL_DATA <= 10'b0;
            RX6_OFFSET_CTRL_DATA <= 10'b0;
            RX7_OFFSET_CTRL_DATA <= 10'b0;
            RX8_OFFSET_CTRL_DATA <= 10'b0;
            RX9_OFFSET_CTRL_DATA <= 10'b0;
            RX10_OFFSET_CTRL_DATA <= 10'b0;
            RX11_OFFSET_CTRL_DATA <= 10'b0;
            RX12_OFFSET_CTRL_DATA <= 10'b0;
            RX13_OFFSET_CTRL_DATA <= 10'b0;
            RX14_OFFSET_CTRL_DATA <= 10'b0;
            RX15_OFFSET_CTRL_DATA <= 10'b0;
            RX16_OFFSET_CTRL_DATA <= 10'b0;
        end
        else if (train_req_i) begin
            RX0_OFFSET_CTRL_DATA <= RX0_OFFSET_PARAM[9:0];
            RX1_OFFSET_CTRL_DATA <= RX1_OFFSET_PARAM[9:0];
            RX2_OFFSET_CTRL_DATA <= RX2_OFFSET_PARAM[9:0];
            RX3_OFFSET_CTRL_DATA <= RX3_OFFSET_PARAM[9:0];
            RX4_OFFSET_CTRL_DATA <= RX4_OFFSET_PARAM[9:0];
            RX5_OFFSET_CTRL_DATA <= RX5_OFFSET_PARAM[9:0];
            RX6_OFFSET_CTRL_DATA <= RX6_OFFSET_PARAM[9:0];
            RX7_OFFSET_CTRL_DATA <= RX7_OFFSET_PARAM[9:0];
            RX8_OFFSET_CTRL_DATA <= RX8_OFFSET_PARAM[9:0];
            RX9_OFFSET_CTRL_DATA <= RX9_OFFSET_PARAM[9:0];
            RX10_OFFSET_CTRL_DATA <= RX10_OFFSET_PARAM[9:0];
            RX11_OFFSET_CTRL_DATA <= RX11_OFFSET_PARAM[9:0];
            RX12_OFFSET_CTRL_DATA <= RX12_OFFSET_PARAM[9:0];
            RX13_OFFSET_CTRL_DATA <= RX13_OFFSET_PARAM[9:0];
            RX14_OFFSET_CTRL_DATA <= RX14_OFFSET_PARAM[9:0];
            RX15_OFFSET_CTRL_DATA <= RX15_OFFSET_PARAM[9:0];
            RX16_OFFSET_CTRL_DATA <= RX16_OFFSET_PARAM[9:0];
        end
    end

endmodule
