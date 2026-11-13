// =============================================================================
// module name：prbs_gen
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/tx_chain/mas/prbs_gen_mas.md
// =============================================================================
// 功能概述: TX 侧 PRBS23 生成 IP（independent LFSR，GB/T 46280.4 多项式 1+x^18+x^23），
//   T^64/T^32 并行推进 + 64 bit 并行输出原语，64:1/32:1 双模位宽联动
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-06  OpenHappy Team  初始版本：基于 prbs_gen MAS v1.2 生成
//  2026-08-18  OpenHappy Team  字内 bit 序修正为 LSB-first（MAS v1.4）：打包
//                       {seq_w_low,seq_high_tmp} → {seq_high_tmp,seq_w_low}
//                       （prbs_seq_o[0]=state[22]=第1个输出，时间最早），对齐协议
//                       接口"LSB 优先；SDR"；修复原 MSB-first/折返打包与数据通路
//                       bit 序相反、导致 RX 侧 self-sync 重建无法收敛的问题
// =============================================================================

`timescale 1ns/1ps

module prbs_gen #(
    parameter STATE_WIDTH  = 23,
    parameter OUTPUT_WIDTH = 64
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire  [STATE_WIDTH-1:0] seed_i,
    input  wire                    seed_load_i,
    input  wire                    prbs_en,
    input  wire                    clear,
    input  wire                    io_data_width_mode,
    output wire  [OUTPUT_WIDTH-1:0] prbs_seq_o,
    output wire  [STATE_WIDTH-1:0]  state_o
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    localparam [STATE_WIDTH-1:0] RESET_STATE = {STATE_WIDTH{1'b1}};

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    reg  [STATE_WIDTH-1:0] state;
    wire [STATE_WIDTH-1:0] state_adv;
    wire [OUTPUT_WIDTH-1:0] seq_w_full;     // T^64 展开结果（全 64 bit）
    wire [31:0]            seq_w_low;      // T^32 展开结果（低 32 bit，第二级门控源）
    wire [STATE_WIDTH-1:0] state_t32;      // T^32(state)，第一级推进结果
    wire [STATE_WIDTH-1:0] state_next;
    wire [OUTPUT_WIDTH-1:0] prbs_seq_w;

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================
    // PRBS23 单步反馈：feedback = state[22] ^ state[17]
    // 单步推进：state_next_1 = {state[21:0], feedback}（向高 bit 移 1 位）
    //
    // T^32 并行展开（基于 23×23 状态转移矩阵 T 的 32 次幂）
    // 输出位 seq_w_low[i] = state 推进 i 步后的最高位（state[22] @ step i）
    // 推进后 state_t32 = state 推进 32 步的结果
    //
    // 实现思路：对每个输出位 seq_w_low[i] (i=0..31)，展开为 state 各 bit 的 XOR
    //   - seq_w_low[0]  = state[22]
    //   - seq_w_low[i]  = state[22-i] ⊕ state[17-i]  (当 22-i >= 0 且 17-i >= 0)
    //   - 高 i 值需递推（state bit 被多次反馈）
    //
    // 用 always @(*) 描述 T^32 单步递推以避免手工展开 23×23 矩阵的复杂度

    // --------------------------------------------------------------------------
    // T^32 展开（第一级）：基于 state 递推 32 步，输出低 32 bit 与推进后 state
    // --------------------------------------------------------------------------
    // 临时变量：用整型 reg 数组描述递推过程，综合工具会展开为 XOR 树
    reg [STATE_WIDTH-1:0] state_tmp_t32;
    reg [31:0]            seq_low_tmp;
    integer               i_t32;

    always @(*) begin
        state_tmp_t32 = state;
        for (i_t32 = 0; i_t32 < 32; i_t32 = i_t32 + 1) begin
            seq_low_tmp[i_t32] = state_tmp_t32[STATE_WIDTH-1];
            // 反馈位 = state[22] ^ state[17]
            state_tmp_t32 = {state_tmp_t32[STATE_WIDTH-2:0],
                            (state_tmp_t32[STATE_WIDTH-1] ^ state_tmp_t32[STATE_WIDTH-6])};
        end
    end

    assign seq_w_low  = seq_low_tmp;
    assign state_t32  = state_tmp_t32;

    // --------------------------------------------------------------------------
    // T^64 展开（第二级）：基于 state_t32 再递推 32 步，输出高 32 bit 与最终 state
    // --------------------------------------------------------------------------
    reg [STATE_WIDTH-1:0] state_tmp_t64;
    reg [31:0]            seq_high_tmp;
    integer               i_t64;

    always @(*) begin
        state_tmp_t64 = state_t32;
        for (i_t64 = 0; i_t64 < 32; i_t64 = i_t64 + 1) begin
            seq_high_tmp[i_t64] = state_tmp_t64[STATE_WIDTH-1];
            state_tmp_t64 = {state_tmp_t64[STATE_WIDTH-2:0],
                            (state_tmp_t64[STATE_WIDTH-1] ^ state_tmp_t64[STATE_WIDTH-6])};
        end
    end

    // 完整 64 bit 输出打包（MAS §3.4 字内 bit 序约定，LSB-first = 协议接口"LSB 优先；SDR"）：
    //   prbs_seq_o[0] = 本拍第 1 个输出（时间最早） = state[22]
    //   prbs_seq_o[63] = 本拍第 64 个输出（时间最晚）
    // T^32 第一级产生时间上最早 32 个输出（seq_w_low[0]=state[22]，seq_w_low[31]=推进 31 步输出）
    // T^32 第二级产生时间上后 32 个输出（seq_high_tmp[0]=state 推进 32 步后的 state[22] 起）
    // 故 LSB-first 拼接：{seq_high_tmp[31:0], seq_w_low[31:0]} —— seq_w_low 时间更早，置低位
    assign seq_w_full = {seq_high_tmp, seq_w_low};
    assign state_adv  = io_data_width_mode ? state_tmp_t64 : state_t32;

    // 输出打包：32:1 模式高 32 bit 置 0
    assign prbs_seq_w = io_data_width_mode ? seq_w_full
                                            : {{32{1'b0}}, seq_w_low};

    // state_next mux：clear > seed_load > prbs_en > 推进 / 保持
    assign state_next = clear        ? {STATE_WIDTH{1'b0}} :
                        seed_load_i  ? seed_i                :
                        prbs_en       ? state_adv             :
                                       state;

    // 输出
    assign prbs_seq_o = prbs_seq_w;
    assign state_o    = state;

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= RESET_STATE;   // 复位全 1（PRBS23 全 0 会卡死）
        end
        else begin
            state <= state_next;
        end
    end

endmodule
