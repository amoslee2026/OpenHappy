// =============================================================================
// module name：prbs_chk
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/rx_chain/mas/prbs_chk_mas.md
// =============================================================================
// 功能概述: RX 侧 PRBS23 检查 + 对齐 IP（self-sync / independent 双模式，GB/T 46280.4
//   多项式 1+x^18+x^23），T^64/T^32 并行推进 + 64 bit 并行输出 + 64 cycle 收敛判定
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-06  OpenHappy Team  初始版本：基于 prbs_chk MAS v1.3 生成
//  2026-08-18  OpenHappy Team  字内 bit 序修正为 LSB-first + self-sync 重建切片修正（MAS v1.5）：
//                       打包 {seq_w_low,seq_high_tmp} → {seq_high_tmp,seq_w_low}
//                       （prbs_seq_o[0]=state[22]=第1个输出，时间最早）；
//                       recon data_i[63:41]/[31:9] → reverse23(data_i[22:0])
//                       （state[22]=data_i[0] 时间最早）；对齐协议接口"LSB 优先；SDR"，
//                       修复原 MSB-first/折返打包 + 反向切片导致 self-sync 永不收敛的问题
//  2026-09-15  OpenHappy Team  RX-SELFSYNC-IMPL-01-R2（B-1 修复）：self-sync match 分支 aligned_next 改
//                        sticky——(sync_cnt==SYNC_CNT_MAX)?1:aligned_reg，cnt==63 置 1 后匹配续拍保持
//                        （原 1 拍脉冲，FSM 晚 1 拍切 independent 时 u_aligned_reg 冻结 0）;
//                        reject（任意 1 拍不匹配）仍清零重计 + aligned 回落 0，independent 分支不变
// =============================================================================

`timescale 1ns/1ps

module prbs_chk #(
    parameter STATE_WIDTH       = 23,
    parameter OUTPUT_WIDTH      = 64,
    parameter SYNC_CHECK_CYCLES = 64
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire  [OUTPUT_WIDTH-1:0] data_i,
    input  wire  [STATE_WIDTH-1:0]  seed_i,
    input  wire                    seed_load_i,
    input  wire                    prbs_en,
    input  wire                    self_sync_en,
    input  wire                    clear,
    input  wire                    io_data_width_mode,
    output wire  [OUTPUT_WIDTH-1:0] prbs_seq_o,
    output wire                    aligned_o,
    output wire  [STATE_WIDTH-1:0]  state_o
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    localparam [STATE_WIDTH-1:0] RESET_STATE = {STATE_WIDTH{1'b1}};
    localparam [5:0] SYNC_CNT_MAX = SYNC_CHECK_CYCLES - 1;  // 6'd63

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    reg  [STATE_WIDTH-1:0]  state;
    reg  [5:0]              sync_cnt;
    reg                     aligned_reg;

    wire [STATE_WIDTH-1:0]  recon;          // self-sync 重建切片
    wire [STATE_WIDTH-1:0] sync_src;       // 推进源（recon / state）
    wire [STATE_WIDTH-1:0] state_adv;      // T^W(sync_src)
    wire [OUTPUT_WIDTH-1:0] seq_w_full;     // T^64 展开
    wire [31:0]            seq_w_low;     // T^32 展开（低 32 bit）
    wire [STATE_WIDTH-1:0] state_t32;     // T^32(state)
    wire [OUTPUT_WIDTH-1:0] prbs_seq_w;
    wire [STATE_WIDTH-1:0]  state_next;
    wire                   sync_match;
    reg  [5:0]             sync_cnt_next;
    reg                    aligned_next;

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // self-sync 重建切片（LSB-first，对齐协议接口"LSB 优先；SDR"）：
    //   state 语义 = "未来 23 个输出 bit 的窗口"，state[22] 时间最早 = data_i[0]（LSB 时间最早）
    //   故重建 state = 取 data_i[22:0]（低端 23 bit）再反序，使 state[22]=data_i[0]、state[0]=data_i[22]
    //   （64:1 与 32:1 均取低 32 bit 有效段头 23 bit；32:1 模式 data_i[31:0] 有效、bit0 仍最早）
    // --------------------------------------------------------------------------
    function automatic [STATE_WIDTH-1:0] reverse23;
        input [STATE_WIDTH-1:0] in;
        integer b;
        begin
            for (b = 0; b < STATE_WIDTH; b = b + 1)
                reverse23[b] = in[STATE_WIDTH-1-b];
        end
    endfunction

    assign recon = reverse23(data_i[STATE_WIDTH-1:0]);

    // --------------------------------------------------------------------------
    // sync_src mux：self-sync 选重建切片 / independent 选自反馈 state
    // --------------------------------------------------------------------------
    assign sync_src = self_sync_en ? recon : state;

    // --------------------------------------------------------------------------
    // T^32 展开（第一级）：基于 sync_src 递推 32 步
    //   - 输出低 32 bit（seq_w_low）与推进后 state（state_t32）
    //   - 注意：self-sync 模式 sync_src = recon（来自 data_i），independent 模式 sync_src = state
    // --------------------------------------------------------------------------
    reg [STATE_WIDTH-1:0] state_tmp_t32;
    reg [31:0]            seq_low_tmp;
    integer               i_t32;

    always @(*) begin
        state_tmp_t32 = sync_src;
        for (i_t32 = 0; i_t32 < 32; i_t32 = i_t32 + 1) begin
            seq_low_tmp[i_t32] = state_tmp_t32[STATE_WIDTH-1];
            state_tmp_t32 = {state_tmp_t32[STATE_WIDTH-2:0],
                            (state_tmp_t32[STATE_WIDTH-1] ^ state_tmp_t32[STATE_WIDTH-6])};
        end
    end

    assign seq_w_low = seq_low_tmp;
    assign state_t32 = state_tmp_t32;

    // --------------------------------------------------------------------------
    // T^64 展开（第二级）：基于 state_t32 再递推 32 步
    //   - 输出高 32 bit 与最终 state_adv
    //   - 32:1 模式第二级输出无效（高 32 bit 置 0），但 state_adv 用 T^32
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
    // T^32 第一级产生时间上最早 32 个输出（seq_w_low），T^32 第二级产生时间上后 32 个（seq_high_tmp）
    // 故 LSB-first 拼接：{seq_high_tmp[31:0], seq_w_low[31:0]} —— seq_w_low 时间更早，置低位
    assign seq_w_full = {seq_high_tmp, seq_w_low};
    assign state_adv  = io_data_width_mode ? state_tmp_t64 : state_t32;

    // 输出打包：32:1 模式高 32 bit 置 0
    assign prbs_seq_w = io_data_width_mode ? seq_w_full
                                            : {{32{1'b0}}, seq_w_low};

    // state_next mux：clear > seed_load > prbs_en > 推进 / 保持
    // 注：self_sync/independent 由 sync_src mux 选择推进源，统一进 state_adv
    assign state_next = clear        ? {STATE_WIDTH{1'b0}} :
                        seed_load_i  ? seed_i                :
                        prbs_en       ? state_adv             :
                                       state;

    // 输出
    assign prbs_seq_o = prbs_seq_w;
    assign state_o    = state;

    // --------------------------------------------------------------------------
    // self-sync 收敛判定
    //   - sync_match: 64:1 全 64 bit 比较 / 32:1 仅低 32 bit 比较
    //   - 计数器：连续 64 cycle 匹配后下一拍拉高 aligned_next，随后匹配续拍保持（sticky，
    //     cnt==63 触发后 u_aligned_reg 锁存 1；B-1: 原 1 拍脉冲致 FSM 晚 1 拍切
    //     independent 时冻结 0）
    //   - 任意 1 cycle 不匹配：计数器清零 + aligned_next=0（reject 清 0）
    //   - independent 模式：计数器冻结，aligned_o 保持上一值
    // --------------------------------------------------------------------------
    assign sync_match = io_data_width_mode ? (prbs_seq_w == data_i)
                                            : (prbs_seq_w[31:0] == data_i[31:0]);

    always @(*) begin
        if (self_sync_en) begin
            if (sync_match) begin
                sync_cnt_next = sync_cnt + 6'd1;
                aligned_next  = (sync_cnt == SYNC_CNT_MAX) ? 1'b1 : aligned_reg;  // sticky：cnt==63 置 1 后匹配续拍保持
            end
            else begin
                sync_cnt_next = 6'd0;
                aligned_next  = 1'b0;  // reject：任意 1 拍不匹配，清零重计 + aligned 回落 0
            end
        end
        else begin
            // independent 模式：计数器冻结，aligned 保持
            sync_cnt_next = sync_cnt;
            aligned_next  = aligned_reg;
        end
    end

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= RESET_STATE;  // 复位全 1
            sync_cnt     <= 6'd0;
            aligned_reg  <= 1'b0;
        end
        else begin
            state        <= state_next;
            sync_cnt     <= sync_cnt_next;
            aligned_reg  <= aligned_next;
        end
    end

    // aligned_o 寄存器输出
    assign aligned_o = aligned_reg;

endmodule
