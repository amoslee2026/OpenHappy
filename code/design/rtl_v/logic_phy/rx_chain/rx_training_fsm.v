// =============================================================================
// module name：rx_training_fsm
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/rx_chain/submods/rx_training/mas/rx_training_fsm_mas.md
//   （@6c163bb, issue #1; §1.2/§2 self_sync_en 驱动已实现, per-step 调度待迭代）
// =============================================================================
// 功能概述: RX 训练主控 FSM（纯 step 调度器，单 phy_rx_clk 域）：本期实现 MAS §2
//   最小 bring-up 三态壳（ST_IDLE/ST_BRINGUP/ST_ACTIVE）+ self_sync_en 驱动
//   （issue #1 RTL blocker 闭合, 覆盖 5 条 self-sync coverage TP: TP-003/004/005/006/014）;
//   per-step 训练调度/超时/irq/手动模式 沿用端口 stub，待后续 MAS 修订迭代
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-07  OpenHappy Team  初始版本：端口 stub（MAS 待编写），功能未实现
//  2026-09-15  coder    RX-SELFSYNC-IMPL-01：self_sync_en 由硬编码 0 修为 MAS §2
//                        bring-up 三态壳驱动（ST_IDLE/ST_BRINGUP/ST_ACTIVE; 复位清 0 /
//                        train_mode==2'b11→BRINGUP+en=1 / aligned_i 全 1→ACTIVE+en=0 /
//                        retrain_req_i→重校准+en=1）; 其余输出保持 stub 0
//  2026-09-15  OpenHappy Team  RX-SELFSYNC-IMPL-01-R3（issue #6, B-2 修复）：BRINGUP ③ 出口加
//                        bringup_entry 入口退避——自 ST_IDLE / ST_ACTIVE/retrain 任一路径
//                        进入 ST_BRINGUP 的入口拍不评估 aligned_i==16'hFFFF→ACTIVE 出口
//                        （次拍起正常评估）; 消除 retrain 入口拍消费 stale sticky aligned FFFF
//                        →ACTIVE 回弹→reject 清 aligned→ACTIVE+aligned=0 恒态死锁
//                        （数据无法重对齐）; 单模块无接口变更, prbs_chk 不动
// =============================================================================

`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// 设计说明:
//
// 端口依据:
//   - rx_training_spec.json v1.2 internal_connections（submods.rx_training_fsm 相关条目）
//   - rx_chain_spec.json v2.1 internal_connections（rx_training 顶端口透传）
// -----------------------------------------------------------------------------

module rx_training_fsm (
    input  wire        phy_rx_clk,
    input  wire        rx_rst_n,

    // -------------------------------------------------------------------------
    // PRBS 对齐控制（rx_data_path.rx_prbs_descrambler 交互）
    // -------------------------------------------------------------------------
    output reg         self_sync_en,                // 出顶到 rx_data_path.self_sync_en（注册驱动）
    input  wire [15:0] aligned_i,                   // rx_data_path 顶端口 aligned_o 回送

    // -------------------------------------------------------------------------
    // reorder 训练交互（rx_data_path.rx_byte_distributor）
    // -------------------------------------------------------------------------
    output wire        reorder_trigger,             // reorder 训练触发
    input  wire        reorder_done,                // reorder 训练完成

    // -------------------------------------------------------------------------
    // sideband-led 训练控制（1 cycle pulse，CDC 在 sideband 侧完成）
    // -------------------------------------------------------------------------
    output wire        train_req_o,                 // RX 主动发起训练请求 → sideband
    input  wire        retrain_req_i,               // peer/本地 retrain 触发
    input  wire        train_complete_ack_i,        // 对端 TX 进 ACTIVE 回送 ack
    input  wire        train_offset_req_i,          // IDLE → TRAIN_OFFSET / OFFSET_INIT
    input  wire        train_vref_req_i,            // IDLE → TRAIN_VREF
    input  wire        train_eq_req_i,              // IDLE → TRAIN_EQ
    input  wire        train_pi_req_i,              // IDLE → TRAIN_PI
    input  wire        train_dcdl_req_i,            // IDLE → TRAIN_DCDL
    input  wire        train_reorder_req_i,         // IDLE → TRAIN_REORDER
    input  wire        train_data_check_req_i,      // IDLE → DATA_CHK
    output wire        train_offset_complete_o,     // → sideband 聚合回对端 TX
    output wire        train_vref_complete_o,
    output wire        train_eq_complete_o,
    output wire        train_pi_complete_o,
    output wire        train_dcdl_complete_o,
    output wire        train_reorder_complete_o,
    output wire        train_data_check_complete_o,

    // -------------------------------------------------------------------------
    // APB 具名配置信号（apb_csr_top 解码后直送）
    // -------------------------------------------------------------------------
    input  wire [1:0]  train_mode,                  // TRAIN_MODE.mode[1:0]
    input  wire        force_mode_switch,           // TOP_CTRL.force_mode_switch
    input  wire [6:0]  bypass_step,                 // BYPASS_CFG.bypass_step[6:0]
    input  wire [6:0]  timeout_step_en,             // TIMEOUT_EN.timeout_step_en[6:0]
    input  wire [15:0] timeout_threshold,           // TIMEOUT_THRESHOLD.timeout_threshold[15:0]
    input  wire        data_check_pattern_sel,      // DATA_CHK_CFG.pattern_sel（0=PRBS / 1=自定义）
    input  wire        step_trigger,                // TRAIN_STEP_CTRL.step_trigger（RW1S）
    input  wire [6:0]  step_force_end,              // TRAIN_STEP_CTRL.step_force_end[6:0]
    input  wire [2:0]  step_cnt,                    // TRAIN_STEP_CNT.step_cnt[2:0]

    // -------------------------------------------------------------------------
    // 中断（4 bit level：train done / train fail / step done / train timeout）
    // -------------------------------------------------------------------------
    output wire [3:0]  irq_lane_o,

    // -------------------------------------------------------------------------
    // 训练执行单元握手（valid-ready，phy_rx_clk{i} 同域）
    // -------------------------------------------------------------------------
    output wire        offset_train_req_o,          // → offset_train.train_req_i
    input  wire        offset_train_done_i,         // ← offset_train.train_done_o
    output wire        vref_train_req_o,            // → vref_train.train_req_i
    input  wire        vref_train_done_i,           // ← vref_train.train_done_o
    output wire        pi_train_req_o,              // → pi_train.train_req_i
    input  wire        pi_train_done_i,             // ← pi_train.train_done_o
    output wire        dcdl_train_req_o,            // → dcdl_train.train_req_i
    input  wire        dcdl_train_done_i,           // ← dcdl_train.train_done_o
    output wire        data_check_req_o,            // → data_checker.check_req_i
    input  wire        data_check_done_i            // ← data_checker.check_done_o
);

    // ==========================================================================
    // TODO: 完整 FSM 后续修订实现（MAS 0.1 仅落 self_sync; 其余待 1.0 迭代）
    //   - step 调度 FSM（per-step 训练态转移, ST_TRAIN_* 已预留 localparam）
    //   - per-step req 接收 + complete 产生（1 cycle pulse）
    //   - per-step 超时检测（timeout_step_en / timeout_threshold）
    //   - 手动模式单步触发 / 强制结束（step_trigger / step_force_end / step_cnt）
    //   - irq_lane_o 4 bit 中断产生
    //   - EQ CTLE 档位切换（EQ_PARAM，CTLE 档位参数待补充）
    // ==========================================================================

    // --------------------------------------------------------------------------
    // RX-SELFSYNC-IMPL-01（issue #1 RTL blocker）: MAS §2 最小 bring-up 三态壳
    //   self_sync_en 驱动（注册逻辑, phy_rx_clk 同域; 方案 A=FSM 内部驱动, 决策 7=C-a）:
    //     ① !rx_rst_n                   → self_sync_en<=0（复位清零）
    //     ② train_mode==2'b11（上电即通）→ ST_BRINGUP + self_sync_en<=1（对齐对端 PRBS）
    //     ③ aligned_i[15:0]==16'hFFFF   → self_sync_en<=0 + ST_ACTIVE（切 independent,
    //                                      aligned 在 prbs_chk 锁存保持）
    //     ④ retrain_req_i               → ST_BRINGUP + self_sync_en<=1（重校准/重对齐）
    //   保持时态: BRINGUP 内 aligned 未全 1 期间 self_sync_en 保持 1（持续追踪, TP-004/005 边界）
    //   ⑤ R3（issue #6, B-2）: 进入 ST_BRINGUP（自 ST_IDLE 或 ST_ACTIVE/retrain 任一路径）的
    //      入口拍不评估 ③ 出口（bringup_entry=1 退避 1 拍）——避免 retrain 入口拍消费 stale
    //      sticky aligned FFFF → ACTIVE 回弹 → prbs_chk reject 清 aligned → ACTIVE+aligned=0
    //      恒态死锁; 次拍起正常评估（干净 retrain 出口仅晚 1 拍, 无害）
    //   per-lane（4T1R 仅 lane0 生效; lane1~3 无 RX 恒 0, lane_top_spec v1.10 注）
    // --------------------------------------------------------------------------

    // 三态壳状态编码
    localparam [2:0] ST_IDLE    = 3'd0;   // 空闲（复位/未激活, self_sync_en=0）
    localparam [2:0] ST_BRINGUP = 3'd1;   // 上电即通校准（self_sync_en=1, 对齐对端 PRBS）
    localparam [2:0] ST_ACTIVE  = 3'd2;   // aligned 全 1 保持（self_sync_en=0, independent）
    // per-step 训练态/错误态 localparam 预留（完整 FSM 后续迭代; 本期不引用/不实现转移）:
    localparam [5:0] ST_TRAIN_OFFSET   = 6'd3;
    localparam [5:0] ST_TRAIN_VREF     = 6'd4;
    localparam [5:0] ST_TRAIN_EQ       = 6'd5;
    localparam [5:0] ST_TRAIN_PI       = 6'd6;
    localparam [5:0] ST_TRAIN_DCDL     = 6'd7;
    localparam [5:0] ST_TRAIN_REORDER  = 6'd8;
    localparam [5:0] ST_TRAIN_DATA_CHK = 6'd9;
    localparam [5:0] ST_ERROR          = 6'd10;

    reg [2:0]  rx_state;

    // R3（issue #6, B-2）: BRINGUP 入口退避标志——进入 ST_BRINGUP 当拍置 1、次拍清 0;
    // 为 1 的 BRINGUP 拍不评估 aligned_i==FFFF 出口（见 ⑤）
    reg        bringup_entry;

    always @(posedge phy_rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            rx_state       <= ST_IDLE;
            self_sync_en   <= 1'b0;
            bringup_entry  <= 1'b0;
        end
        else begin
            case (rx_state)
                // ② 上电即通: train_mode==2'b11 进入校准并置 self_sync_en（同时武装入口退避）
                ST_IDLE: begin
                    if (train_mode == 2'b11) begin
                        rx_state       <= ST_BRINGUP;
                        self_sync_en   <= 1'b1;
                        bringup_entry  <= 1'b1;
                    end
                end
                // ③⑤ BRINGUP: 入口拍退避（bringup_entry=1 保持 BRINGUP 不评估出口, 次拍清 0）;
                //             全 1 → ACTIVE（切 independent, en=0）; 否则保持追踪;
                //             retrain 保持/重申 en=1
                ST_BRINGUP: begin
                    if (bringup_entry) begin
                        rx_state       <= ST_BRINGUP;
                        self_sync_en   <= 1'b1;
                        bringup_entry  <= 1'b0;
                    end
                    else if (aligned_i == 16'hFFFF) begin
                        rx_state       <= ST_ACTIVE;
                        self_sync_en   <= 1'b0;
                        bringup_entry  <= 1'b0;
                    end
                    else if (retrain_req_i) begin
                        rx_state       <= ST_BRINGUP;
                        self_sync_en   <= 1'b1;
                        bringup_entry  <= 1'b0;
                    end
                end
                // ACTIVE: aligned 锁存保持; retrain → 重校准（同时武装入口退避）
                ST_ACTIVE: begin
                    if (retrain_req_i) begin
                        rx_state       <= ST_BRINGUP;
                        self_sync_en   <= 1'b1;
                        bringup_entry  <= 1'b1;
                    end
                    else begin
                        bringup_entry  <= 1'b0;
                    end
                end
                default: begin
                    rx_state       <= ST_IDLE;
                    self_sync_en   <= 1'b0;
                    bringup_entry  <= 1'b0;
                end
            endcase
        end
    end

    // --------------------------------------------------------------------------
    // stub（其余输出保持安全常值 0, 待 MAS 1.0 per-step 训练调度补全后替换）:
    //   reorder_trigger / train_req_o / *_complete_o / irq_lane_o / *_train_req_o
    // --------------------------------------------------------------------------
    assign reorder_trigger             = 1'b0;
    assign train_req_o                 = 1'b0;
    assign train_offset_complete_o     = 1'b0;
    assign train_vref_complete_o       = 1'b0;
    assign train_eq_complete_o         = 1'b0;
    assign train_pi_complete_o         = 1'b0;
    assign train_dcdl_complete_o       = 1'b0;
    assign train_reorder_complete_o    = 1'b0;
    assign train_data_check_complete_o = 1'b0;
    assign irq_lane_o                  = 4'b0000;
    assign offset_train_req_o          = 1'b0;
    assign vref_train_req_o            = 1'b0;
    assign pi_train_req_o              = 1'b0;
    assign dcdl_train_req_o            = 1'b0;
    assign data_check_req_o            = 1'b0;

endmodule
