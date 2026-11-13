// =============================================================================
// module name：tx_training_fsm
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/tx_chain/mas/tx_training_fsm_mas.md
// =============================================================================
// 功能概述: TX 训练步骤调度主控 FSM（10 态，4 种训练模式），per-step req/complete 握手 +
//   超时 / 强制结束检测，SBC 编解码与 CDC 归 sideband，单 mb_clk 域
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-09-13  cdc-resolver  CDC 违例修复（MAS v2.4）：force_mode_switch_i/step_trigger_i 改 toggle 载体，入口 2FF 同步 + 同步输出打拍（t2^t2_d）边沿恢复 mb_clk 1 拍脉冲，FSM 全部使用点改接同步后信号
//  2026-08-07  OpenHappy Team  初始版本：基于 tx_training_fsm MAS v2.1 生成
// =============================================================================

`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// 设计说明:
//
// 与 MAS 偏差说明（端口以 blueprint.json 为权威）:
//   - blueprint 无 int_clear_pulse_i 端口：ERROR 态仅由 mb_rst_n 复位退出（MAS §5.2 伪码
//     中 firmware 写 INT_CLEAR 回 IDLE 的路径无端口承载）
//   - blueprint 无独立 req_pulse 端口：req_o 由 req_hold_reg（level）驱动；req_pulse_reg
//     按 blueprint internals 保留为内部 1-cycle 脉冲寄存器，不驱动端口
//   - 手动模式单步完成后按 fsm.json 转移表顺序推进下一步（MAS §5.3 "回 IDLE" 与
//     fsm.json 冲突，以 fsm.json 为权威）
// -----------------------------------------------------------------------------

module tx_training_fsm (
    // 时钟与复位
    input  wire        mb_clk,
    input  wire        mb_rst_n,

    // 训练启动 / 重训练请求（sideband 解 CPIIF event 产生，mb_clk 域）
    input  wire        train_req_i,
    input  wire        retrain_req_i,

    // per-step 训练完成指示（sideband 回收对端 X_COMPLETE SBC 报文后产生，mb_clk 域）
    input  wire        train_offset_complete_i,
    input  wire        train_vref_complete_i,
    input  wire        train_eq_complete_i,
    input  wire        train_pi_complete_i,
    input  wire        train_dcdl_complete_i,
    input  wire        train_reorder_complete_i,
    input  wire        train_data_check_complete_i,

    // 训练完成 ack（ACTIVE 态 entry cycle 拉高 1 cycle，回报 sideband）
    output wire        train_complete_ack_o,

    // 配置接口（apb_csr_top 具名信号直送）
    input  wire [1:0]  train_mode_i,
    input  wire        force_mode_switch_i,
    input  wire [6:0]  bypass_step_i,
    input  wire [6:0]  timeout_step_en_i,
    input  wire [15:0] timeout_threshold_i,
    input  wire        data_check_pattern_sel_i,
    input  wire        step_trigger_i,
    input  wire [6:0]  step_force_end_i,
    input  wire [2:0]  step_cnt_i,

    // per-step 训练请求（one-hot 互斥，level；双 fanout 至 tx_data_path 与 sideband）
    output wire        offset_req_o,
    output wire        vref_req_o,
    output wire        eq_req_o,
    output wire        pi_req_o,
    output wire        dcDL_req_o,
    output wire        reorder_req_o,
    output wire        data_check_req_o,

    // DFX 状态回流与中断
    output wire [3:0]  train_fsm_state_o,
    output wire [2:0]  train_step_cnt_o,
    output wire [15:0] train_timeout_cnt_o,
    output wire        train_done_o,
    output wire        train_fail_o,
    output wire        step_done_o,
    output wire        train_timeout_o,
    output wire [3:0]  irq_lane_o
);

    // ==========================================================================
    // 参数定义（状态编码按 tx_training_fsm_train_main_fsm.json，binary）
    // ==========================================================================
    localparam ST_IDLE          = 4'd0;
    localparam ST_TRAIN_OFFSET  = 4'd1;
    localparam ST_TRAIN_VREF    = 4'd2;
    localparam ST_TRAIN_EQ      = 4'd3;
    localparam ST_TRAIN_PI      = 4'd4;
    localparam ST_TRAIN_DCDL    = 4'd5;
    localparam ST_TRAIN_REORDER = 4'd6;
    localparam ST_DATA_CHK      = 4'd7;
    localparam ST_ACTIVE        = 4'd8;
    localparam ST_ERROR         = 4'd9;
    // 4'd10 ~ 4'd15 为非法态，default 分支回 IDLE

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================

    // --------------------------------------------------------------------------
    // CDC 同步（pclk 写脉冲 → toggle → mb_clk 2FF → 同步输出打拍边沿检测归 1 拍脉冲）
    //   force_mode_switch_i / step_trigger_i 端口语义已由 “pclk 1 拍脉冲” 改为
    //   “toggle 请求”（apb_csr_top 侧由 *_wp 转 toggle），此处 2FF 同步 + 同步
    //   输出打拍 XOR 恢复 mb_clk 域 1 拍脉冲；fms_t1/step_t1 仅驱动 t2（纯 2FF
    //   同步链），边沿检测在同步输出侧（cdc-resolver 2026-09-13）。
    // --------------------------------------------------------------------------
    reg  fms_t1, fms_t2, fms_t2_d;
    reg  step_t1, step_t2, step_t2_d;
    always @(posedge mb_clk or negedge mb_rst_n) begin
        if (!mb_rst_n) begin
            fms_t1    <= 1'b0;
            fms_t2    <= 1'b0;
            fms_t2_d  <= 1'b0;
            step_t1   <= 1'b0;
            step_t2   <= 1'b0;
            step_t2_d <= 1'b0;
        end
        else begin
            fms_t1    <= force_mode_switch_i;
            fms_t2    <= fms_t1;
            fms_t2_d  <= fms_t2;
            step_t1   <= step_trigger_i;
            step_t2   <= step_t1;
            step_t2_d <= step_t2;
        end
    end
    wire force_mode_switch_i_ckd = fms_t2 ^ fms_t2_d;   // mb_clk 域 1 拍脉冲
    wire step_trigger_i_ckd      = step_t2 ^ step_t2_d;

    // --------------------------------------------------------------------------
    // FSM 状态
    // --------------------------------------------------------------------------
    reg  [3:0]  cur_state;
    reg  [3:0]  next_state;

    // --------------------------------------------------------------------------
    // 训练步骤 / 超时计数 / entry cycle 标志
    // --------------------------------------------------------------------------
    reg  [2:0]  step_cnt_reg;          // 当前训练步骤计数（0..7）
    reg  [15:0] timeout_cnt_reg;       // 超时计数器（mb_clk cycle）
    reg         entry_cycle_reg;       // 进入态第一拍标志，复位 1

    // --------------------------------------------------------------------------
    // req 持续 / 脉冲寄存器（bit0=offset ... bit6=data_check）
    // --------------------------------------------------------------------------
    reg  [6:0]  req_hold_reg;          // active 态持续拉高，complete 或超时拉低
    reg  [6:0]  req_pulse_reg;         // entry cycle 1-cycle 脉冲（内部，无独立端口）

    // --------------------------------------------------------------------------
    // 状态标志寄存器（done/fail/timeout 为 level/sticky，step_done 为 1-cycle 脉冲）
    // --------------------------------------------------------------------------
    reg         train_done_reg;
    reg         train_fail_reg;
    reg         step_done_pulse_reg;
    reg         train_timeout_reg;
    reg         train_complete_ack_reg;

    // --------------------------------------------------------------------------
    // 组合 helper 信号
    // --------------------------------------------------------------------------
    wire [2:0]  current_step;
    reg         train_complete_hit;    // 当前步骤 complete 命中（7 选 1 mux）
    wire        timeout_hit;           // 当前步骤超时命中
    wire        bypass_hit;            // 简化模式当前步骤 bypass 命中
    wire        force_end_hit;         // 当前步骤强制结束命中

    // --------------------------------------------------------------------------
    // data_check_pattern_sel_i：pattern mux 在 tx_data_path 内完成，本模块不使用
    // --------------------------------------------------------------------------
    // verilator lint_off UNUSED
    wire        unused_data_check_pattern_sel;
    // verilator lint_on UNUSED

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================

    assign current_step = step_cnt_reg;

    assign unused_data_check_pattern_sel = data_check_pattern_sel_i;

    // per-step complete 信号选择（按 current_step 选对应 train_<step>_complete_i）
    always @(*) begin
        case (current_step)
            3'd0:    train_complete_hit = train_offset_complete_i;
            3'd1:    train_complete_hit = train_vref_complete_i;
            3'd2:    train_complete_hit = train_eq_complete_i;
            3'd3:    train_complete_hit = train_pi_complete_i;
            3'd4:    train_complete_hit = train_dcdl_complete_i;
            3'd5:    train_complete_hit = train_reorder_complete_i;
            3'd6:    train_complete_hit = train_data_check_complete_i;
            default: train_complete_hit = 1'b0;
        endcase
    end

    // 超时检测：per-step 使能 + 计数达阈值（共享阈值）
    assign timeout_hit  = timeout_step_en_i[current_step]
                        && (timeout_cnt_reg >= timeout_threshold_i);

    // 简化模式 bypass：仅 train_mode==2'b01 生效，entry cycle 检测
    assign bypass_hit   = (train_mode_i == 2'b01) && bypass_step_i[current_step];

    // per-step 强制结束
    assign force_end_hit = step_force_end_i[current_step];

    // --------------------------------------------------------------------------
    // step_cnt → 状态译码（手动模式 IDLE 态按 step_cnt_i 跳对应 TRAIN_* 态）
    // --------------------------------------------------------------------------
    function [3:0] step_to_state;
        input [2:0] step;
        begin
            case (step)
                3'd0:    step_to_state = ST_TRAIN_OFFSET;
                3'd1:    step_to_state = ST_TRAIN_VREF;
                3'd2:    step_to_state = ST_TRAIN_EQ;
                3'd3:    step_to_state = ST_TRAIN_PI;
                3'd4:    step_to_state = ST_TRAIN_DCDL;
                3'd5:    step_to_state = ST_TRAIN_REORDER;
                3'd6:    step_to_state = ST_DATA_CHK;
                default: step_to_state = ST_IDLE;
            endcase
        end
    endfunction

    // --------------------------------------------------------------------------
    // FSM 第二段：下一状态逻辑（组合）
    //   优先级：force_mode_switch > complete > force_end > timeout > bypass
    //   bypass 仅在 entry cycle 检测；complete/force_end/timeout 在 active/wait cycle 检测
    // --------------------------------------------------------------------------
    always @(*) begin
        next_state = cur_state;  // 默认保持
        case (cur_state)
            ST_IDLE: begin
                if (train_mode_i == 2'b11) begin
                    // 上电即通
                    next_state = ST_ACTIVE;
                end
                else if (step_trigger_i_ckd && (train_mode_i == 2'b10)) begin
                    // 手动模式单步触发
                    next_state = step_to_state(step_cnt_i);
                end
                else if (train_req_i) begin
                    // 自动 / 简化模式启动，从 offset 开始
                    next_state = ST_TRAIN_OFFSET;
                end
            end

            ST_TRAIN_OFFSET: begin
                if (force_mode_switch_i_ckd) begin
                    next_state = ST_ERROR;
                end
                else if (entry_cycle_reg) begin
                    if (bypass_hit) begin
                        next_state = ST_TRAIN_VREF;
                    end
                end
                else begin
                    if (train_complete_hit) begin
                        next_state = ST_TRAIN_VREF;
                    end
                    else if (force_end_hit) begin
                        next_state = ST_TRAIN_VREF;
                    end
                    else if (timeout_hit) begin
                        next_state = ST_ERROR;
                    end
                end
            end

            ST_TRAIN_VREF: begin
                if (force_mode_switch_i_ckd) begin
                    next_state = ST_ERROR;
                end
                else if (entry_cycle_reg) begin
                    if (bypass_hit) begin
                        next_state = ST_TRAIN_EQ;
                    end
                end
                else begin
                    if (train_complete_hit) begin
                        next_state = ST_TRAIN_EQ;
                    end
                    else if (force_end_hit) begin
                        next_state = ST_TRAIN_EQ;
                    end
                    else if (timeout_hit) begin
                        next_state = ST_ERROR;
                    end
                end
            end

            ST_TRAIN_EQ: begin
                if (force_mode_switch_i_ckd) begin
                    next_state = ST_ERROR;
                end
                else if (entry_cycle_reg) begin
                    if (bypass_hit) begin
                        next_state = ST_TRAIN_PI;
                    end
                end
                else begin
                    if (train_complete_hit) begin
                        next_state = ST_TRAIN_PI;
                    end
                    else if (force_end_hit) begin
                        next_state = ST_TRAIN_PI;
                    end
                    else if (timeout_hit) begin
                        next_state = ST_ERROR;
                    end
                end
            end

            ST_TRAIN_PI: begin
                if (force_mode_switch_i_ckd) begin
                    next_state = ST_ERROR;
                end
                else if (entry_cycle_reg) begin
                    if (bypass_hit) begin
                        next_state = ST_TRAIN_DCDL;
                    end
                end
                else begin
                    if (train_complete_hit) begin
                        next_state = ST_TRAIN_DCDL;
                    end
                    else if (force_end_hit) begin
                        next_state = ST_TRAIN_DCDL;
                    end
                    else if (timeout_hit) begin
                        next_state = ST_ERROR;
                    end
                end
            end

            ST_TRAIN_DCDL: begin
                if (force_mode_switch_i_ckd) begin
                    next_state = ST_ERROR;
                end
                else if (entry_cycle_reg) begin
                    if (bypass_hit) begin
                        next_state = ST_TRAIN_REORDER;
                    end
                end
                else begin
                    if (train_complete_hit) begin
                        next_state = ST_TRAIN_REORDER;
                    end
                    else if (force_end_hit) begin
                        next_state = ST_TRAIN_REORDER;
                    end
                    else if (timeout_hit) begin
                        next_state = ST_ERROR;
                    end
                end
            end

            ST_TRAIN_REORDER: begin
                if (force_mode_switch_i_ckd) begin
                    next_state = ST_ERROR;
                end
                else if (entry_cycle_reg) begin
                    if (bypass_hit) begin
                        next_state = ST_DATA_CHK;
                    end
                end
                else begin
                    if (train_complete_hit) begin
                        next_state = ST_DATA_CHK;
                    end
                    else if (force_end_hit) begin
                        next_state = ST_DATA_CHK;
                    end
                    else if (timeout_hit) begin
                        next_state = ST_ERROR;
                    end
                end
            end

            ST_DATA_CHK: begin
                if (force_mode_switch_i_ckd) begin
                    next_state = ST_ERROR;
                end
                else if (entry_cycle_reg) begin
                    if (bypass_hit) begin
                        next_state = ST_ACTIVE;
                    end
                end
                else begin
                    if (train_complete_hit) begin
                        next_state = ST_ACTIVE;
                    end
                    else if (force_end_hit) begin
                        next_state = ST_ACTIVE;
                    end
                    else if (timeout_hit) begin
                        next_state = ST_ERROR;
                    end
                end
            end

            ST_ACTIVE: begin
                if (force_mode_switch_i_ckd) begin
                    next_state = ST_ERROR;
                end
                else if (retrain_req_i) begin
                    next_state = ST_IDLE;
                end
            end

            ST_ERROR: begin
                // blueprint 无 int_clear_pulse_i 端口：ERROR 态仅由 mb_rst_n 退出
                next_state = ST_ERROR;
            end

            default: begin
                // 非法态（4'd10~4'd15）自恢复回 IDLE
                next_state = ST_IDLE;
            end
        endcase
    end

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // FSM 第一段：状态寄存器
    // --------------------------------------------------------------------------
    always @(posedge mb_clk or negedge mb_rst_n) begin
        if (!mb_rst_n) begin
            cur_state <= ST_IDLE;
        end
        else begin
            cur_state <= next_state;
        end
    end

    // --------------------------------------------------------------------------
    // FSM 第三段：输出逻辑（时序）
    //   - 脉冲型寄存器（req_pulse / step_done / complete_ack）每拍默认清零
    //   - entry_cycle_reg：状态将跳转时置 1（下一拍为新态 entry cycle）
    //   - timeout_cnt：entry cycle 清零，active/wait cycle 每拍 +1，complete/force_end/
    //     timeout 命中拍保持（下一拍 entry 清零）
    // --------------------------------------------------------------------------
    always @(posedge mb_clk or negedge mb_rst_n) begin
        if (!mb_rst_n) begin
            step_cnt_reg           <= 3'd0;
            timeout_cnt_reg        <= 16'd0;
            entry_cycle_reg        <= 1'b1;
            req_hold_reg           <= 7'b0;
            req_pulse_reg          <= 7'b0;
            train_done_reg         <= 1'b0;
            train_fail_reg         <= 1'b0;
            step_done_pulse_reg    <= 1'b0;
            train_timeout_reg      <= 1'b0;
            train_complete_ack_reg <= 1'b0;
        end
        else begin
            // 脉冲型寄存器默认清零（1-cycle 脉冲语义）
            req_pulse_reg          <= 7'b0;
            step_done_pulse_reg    <= 1'b0;
            train_complete_ack_reg <= 1'b0;

            // 状态跳转后下一拍为 entry cycle
            if (cur_state != next_state) begin
                entry_cycle_reg <= 1'b1;
            end
            else begin
                entry_cycle_reg <= 1'b0;
            end

            case (cur_state)
                ST_IDLE: begin
                    req_hold_reg    <= 7'b0;
                    timeout_cnt_reg <= 16'd0;
                    if (train_mode_i == 2'b11) begin
                        step_cnt_reg <= 3'd0;
                    end
                    else if (step_trigger_i_ckd && (train_mode_i == 2'b10)) begin
                        step_cnt_reg <= step_cnt_i;
                    end
                    else if (train_req_i) begin
                        step_cnt_reg <= 3'd0;
                    end
                end

                ST_TRAIN_OFFSET: begin
                    if (entry_cycle_reg) begin
                        timeout_cnt_reg <= 16'd0;
                        if (bypass_hit) begin
                            // 简化模式 bypass：不发 req，直接跳下一步
                            req_hold_reg <= 7'b0;
                            step_cnt_reg <= 3'd1;
                        end
                        else begin
                            // active 起始：拉高 req + 产生 1-cycle 脉冲
                            req_hold_reg  <= 7'b000_0001;
                            req_pulse_reg <= 7'b000_0001;
                        end
                    end
                    else begin
                        if (train_complete_hit) begin
                            req_hold_reg        <= 7'b0;
                            step_done_pulse_reg <= 1'b1;
                            step_cnt_reg        <= 3'd1;
                        end
                        else if (force_end_hit) begin
                            req_hold_reg <= 7'b0;
                            step_cnt_reg <= 3'd1;
                        end
                        else if (timeout_hit) begin
                            req_hold_reg      <= 7'b0;
                            train_timeout_reg <= 1'b1;
                        end
                        else begin
                            timeout_cnt_reg <= timeout_cnt_reg + 16'd1;
                        end
                    end
                end

                ST_TRAIN_VREF: begin
                    if (entry_cycle_reg) begin
                        timeout_cnt_reg <= 16'd0;
                        if (bypass_hit) begin
                            req_hold_reg <= 7'b0;
                            step_cnt_reg <= 3'd2;
                        end
                        else begin
                            req_hold_reg  <= 7'b000_0010;
                            req_pulse_reg <= 7'b000_0010;
                        end
                    end
                    else begin
                        if (train_complete_hit) begin
                            req_hold_reg        <= 7'b0;
                            step_done_pulse_reg <= 1'b1;
                            step_cnt_reg        <= 3'd2;
                        end
                        else if (force_end_hit) begin
                            req_hold_reg <= 7'b0;
                            step_cnt_reg <= 3'd2;
                        end
                        else if (timeout_hit) begin
                            req_hold_reg      <= 7'b0;
                            train_timeout_reg <= 1'b1;
                        end
                        else begin
                            timeout_cnt_reg <= timeout_cnt_reg + 16'd1;
                        end
                    end
                end

                ST_TRAIN_EQ: begin
                    if (entry_cycle_reg) begin
                        timeout_cnt_reg <= 16'd0;
                        if (bypass_hit) begin
                            req_hold_reg <= 7'b0;
                            step_cnt_reg <= 3'd3;
                        end
                        else begin
                            req_hold_reg  <= 7'b000_0100;
                            req_pulse_reg <= 7'b000_0100;
                        end
                    end
                    else begin
                        if (train_complete_hit) begin
                            req_hold_reg        <= 7'b0;
                            step_done_pulse_reg <= 1'b1;
                            step_cnt_reg        <= 3'd3;
                        end
                        else if (force_end_hit) begin
                            req_hold_reg <= 7'b0;
                            step_cnt_reg <= 3'd3;
                        end
                        else if (timeout_hit) begin
                            req_hold_reg      <= 7'b0;
                            train_timeout_reg <= 1'b1;
                        end
                        else begin
                            timeout_cnt_reg <= timeout_cnt_reg + 16'd1;
                        end
                    end
                end

                ST_TRAIN_PI: begin
                    if (entry_cycle_reg) begin
                        timeout_cnt_reg <= 16'd0;
                        if (bypass_hit) begin
                            req_hold_reg <= 7'b0;
                            step_cnt_reg <= 3'd4;
                        end
                        else begin
                            req_hold_reg  <= 7'b000_1000;
                            req_pulse_reg <= 7'b000_1000;
                        end
                    end
                    else begin
                        if (train_complete_hit) begin
                            req_hold_reg        <= 7'b0;
                            step_done_pulse_reg <= 1'b1;
                            step_cnt_reg        <= 3'd4;
                        end
                        else if (force_end_hit) begin
                            req_hold_reg <= 7'b0;
                            step_cnt_reg <= 3'd4;
                        end
                        else if (timeout_hit) begin
                            req_hold_reg      <= 7'b0;
                            train_timeout_reg <= 1'b1;
                        end
                        else begin
                            timeout_cnt_reg <= timeout_cnt_reg + 16'd1;
                        end
                    end
                end

                ST_TRAIN_DCDL: begin
                    if (entry_cycle_reg) begin
                        timeout_cnt_reg <= 16'd0;
                        if (bypass_hit) begin
                            req_hold_reg <= 7'b0;
                            step_cnt_reg <= 3'd5;
                        end
                        else begin
                            req_hold_reg  <= 7'b001_0000;
                            req_pulse_reg <= 7'b001_0000;
                        end
                    end
                    else begin
                        if (train_complete_hit) begin
                            req_hold_reg        <= 7'b0;
                            step_done_pulse_reg <= 1'b1;
                            step_cnt_reg        <= 3'd5;
                        end
                        else if (force_end_hit) begin
                            req_hold_reg <= 7'b0;
                            step_cnt_reg <= 3'd5;
                        end
                        else if (timeout_hit) begin
                            req_hold_reg      <= 7'b0;
                            train_timeout_reg <= 1'b1;
                        end
                        else begin
                            timeout_cnt_reg <= timeout_cnt_reg + 16'd1;
                        end
                    end
                end

                ST_TRAIN_REORDER: begin
                    if (entry_cycle_reg) begin
                        timeout_cnt_reg <= 16'd0;
                        if (bypass_hit) begin
                            req_hold_reg <= 7'b0;
                            step_cnt_reg <= 3'd6;
                        end
                        else begin
                            req_hold_reg  <= 7'b010_0000;
                            req_pulse_reg <= 7'b010_0000;
                        end
                    end
                    else begin
                        if (train_complete_hit) begin
                            req_hold_reg        <= 7'b0;
                            step_done_pulse_reg <= 1'b1;
                            step_cnt_reg        <= 3'd6;
                        end
                        else if (force_end_hit) begin
                            req_hold_reg <= 7'b0;
                            step_cnt_reg <= 3'd6;
                        end
                        else if (timeout_hit) begin
                            req_hold_reg      <= 7'b0;
                            train_timeout_reg <= 1'b1;
                        end
                        else begin
                            timeout_cnt_reg <= timeout_cnt_reg + 16'd1;
                        end
                    end
                end

                ST_DATA_CHK: begin
                    if (entry_cycle_reg) begin
                        timeout_cnt_reg <= 16'd0;
                        if (bypass_hit) begin
                            req_hold_reg <= 7'b0;
                            step_cnt_reg <= 3'd7;
                        end
                        else begin
                            req_hold_reg  <= 7'b100_0000;
                            req_pulse_reg <= 7'b100_0000;
                        end
                    end
                    else begin
                        if (train_complete_hit) begin
                            req_hold_reg        <= 7'b0;
                            step_done_pulse_reg <= 1'b1;
                            step_cnt_reg        <= 3'd7;
                        end
                        else if (force_end_hit) begin
                            req_hold_reg <= 7'b0;
                            step_cnt_reg <= 3'd7;
                        end
                        else if (timeout_hit) begin
                            req_hold_reg      <= 7'b0;
                            train_timeout_reg <= 1'b1;
                        end
                        else begin
                            timeout_cnt_reg <= timeout_cnt_reg + 16'd1;
                        end
                    end
                end

                ST_ACTIVE: begin
                    req_hold_reg <= 7'b0;
                    if (entry_cycle_reg) begin
                        // 训练完成 ack：ACTIVE 态 entry cycle 拉高 1 cycle
                        train_complete_ack_reg <= 1'b1;
                    end
                    if (retrain_req_i || force_mode_switch_i_ckd) begin
                        train_done_reg <= 1'b0;
                    end
                    else begin
                        train_done_reg <= 1'b1;
                    end
                end

                ST_ERROR: begin
                    req_hold_reg  <= 7'b0;
                    train_fail_reg <= 1'b1;
                    // train_timeout_reg sticky 保持；无 int_clear 端口，仅复位清
                end

                default: begin
                    // 非法态：清 req，状态经 next_state 回 IDLE
                    req_hold_reg <= 7'b0;
                end
            endcase

            // force_mode_switch 全局中止：训练态 / ACTIVE 态跳 ERROR 时强制清 req，
            // 避免 entry cycle 置位 req 后带入 ERROR 态残留 1 拍
            if (force_mode_switch_i_ckd
                && (cur_state != ST_IDLE)
                && (cur_state != ST_ERROR)) begin
                req_hold_reg  <= 7'b0;
                req_pulse_reg <= 7'b0;
            end
        end
    end

    // ==========================================================================
    // 输出 assign
    // ==========================================================================
    assign offset_req_o          = req_hold_reg[0];
    assign vref_req_o            = req_hold_reg[1];
    assign eq_req_o              = req_hold_reg[2];
    assign pi_req_o              = req_hold_reg[3];
    assign dcDL_req_o            = req_hold_reg[4];
    assign reorder_req_o         = req_hold_reg[5];
    assign data_check_req_o      = req_hold_reg[6];

    assign train_fsm_state_o     = cur_state;
    assign train_step_cnt_o      = step_cnt_reg;
    assign train_timeout_cnt_o   = timeout_cnt_reg;
    assign train_done_o          = train_done_reg;
    assign train_fail_o          = train_fail_reg;
    assign step_done_o           = step_done_pulse_reg;
    assign train_timeout_o       = train_timeout_reg;
    assign train_complete_ack_o  = train_complete_ack_reg;

    // bit[0]=train done / bit[1]=train fail / bit[2]=step done / bit[3]=train timeout
    assign irq_lane_o            = {train_timeout_reg,
                                    step_done_pulse_reg,
                                    train_fail_reg,
                                    train_done_reg};

endmodule
