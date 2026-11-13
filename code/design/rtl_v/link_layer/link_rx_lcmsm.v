// =============================================================================
// module name：link_rx_lcmsm
// author: OpenHappy Team
// MAS：spec/link_layer/submods/link_layer/mas/link_rx_lcmsm_mas.md
// 功能概述: RX 侧本地控制状态机（Local Control State Machine）。按 L1 §3 实现
//           RX 侧 link_fsm 6 态推进：CONFIG → PHY_INIT →（需 deskew → LINK_TRAIN
//           / 不需 deskew → NORMAL）→ NORMAL(U0) ⇄ LP(U2)。
//           错误丢弃（CRC 出错后丢弃非 ARQ Flit）由 link_rx 内部 err_discard
//           寄存器自行控制（CRC 失败置位，兼容收 ARQ(ACK) / 性能收正确 seq
//           退出），本模块不参与；
//           rx_path_en_o 为 NORMAL 态指示（= rx_active）。
//           PHY_INIT 直接转 LINK_TRAIN（多 lane 首训，init_train_done=0）或
//           NORMAL（单 lane / 不需 deskew）；首拍拉高 train_start_o 触发
//           link_rx 内部 lane_dskew，监控 lane_dskew_done_i /
//           lane_dskew_fail_i 决定进 NORMAL 或 retrain。
//           PHY_TRAIN 仅重训练回流路径使用：重训练（NORMAL/LP 触发）先进
//           PHY_TRAIN，再经 go_normal_tr（=evt_done&init_train_done）直接转
//           NORMAL；重训练不重新 deskew（首次训练后 init_train_done=1，
//           go_link_train=0，故去掉 PHY_TRAIN→LINK_TRAIN 转移 state_ctl[5]）。
//           事件命令仅生成 RETRAIN cmd（16'h0080，retrain 触发时与进
//           PHY_TRAIN 同时发出）；PHY_INIT / LANE_ON / LANE_OFF / RX_READY
//           cmd 均属寄存器通路（INIT_CFG_REG），不由本模块生成。
//           FSM 为 state_ctl 三段式风格（条件位 → 次态位 → 状态寄存器）。
//           本模块全部逻辑处于 mb_clk（2 GHz）数据通路时钟域。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   <日期>  <修订人>  <变更概述>
//  2026-08-08  OpenHappy Team  初始版本（对齐 MAS v1.0）
//  2026-08-09  OpenHappy Team  重构为 L1 §3 6 态模型：删 S_BW_SWITCH / S_RETRAIN
//                          （并入 PHY_TRAIN）/ S_ERR_DISCARD（改独立标志，重
//                          训练不清除）；新增 S_PHY_TRAIN / init_train_done
//                          首次训练标记；PHY_INIT cmd 移交寄存器通路，本模块
//                          仅生成 RETRAIN cmd；删除 bridge_en_i /
//                          rx_evt_lane_on/off_i（无消费）；lp_sw_cmd 仅低功耗
//  2026-08-11  OpenHappy Team  新增 INIT_CFG_REG.RETRAIN 软件训练触发：S_NORMAL 下
//                          置位发 RETRAIN cmd + 跳 S_PHY_TRAIN；S_PHY_TRAIN 下
//                          置位仅发 cmd（退出条件不变，仍等 evt_done）
//  2026-08-14  OpenHappy Team  错误丢弃（err_discard）控制移交 link_rx 内部，本模块
//                          删除独立 err_discard_o 标志及其 crc_err_i / arq_done_i
//                          输入；状态机恢复纯 link_fsm 推进
//  2026-08-14  OpenHappy Team  PHY_INIT 出口改直接转 LINK_TRAIN（需 deskew）或 NORMAL
//                          （不需 deskew），不再先进 PHY_TRAIN；PHY_TRAIN 保留为
//                          重训练回流状态；init_train_done 改在首次 LINK_TRAIN
//                          完成（lane_dskew_done）置 1
//  2026-08-14  OpenHappy Team  删除软件重训练触发（sw_retrain_req / csr_init_retrain_i）：
//                          sw_retrain 仅控制 TX 侧（lc_tx_lcmsm），RX 侧不处理；
//                          NORMAL → PHY_TRAIN 仅剩硬件 retrain_req（超时/NACK）
//  2026-08-25  OpenHappy Team  删除 PHY_TRAIN→LINK_TRAIN 转移（state_ctl[5]）：重训练
//                          不重新 deskew（init_train_done=1 使 go_link_train=0），
//                          该转移为死代码；PHY_TRAIN 现仅经 go_normal_tr 直转 NORMAL
//  2026-09-16  cdc-resolver CDC 修复(Ac_conv02 46FD 同源)：lp_enter/lp_exit 事件
//                          先寄存成单 bit（lp_enter_evt_r/lp_exit_evt_r）再参与次态
//                          组合，切断多路跨域同步信号同拍汇聚进 st_low_power_nxt；
//                          lp_sw_req_i/lp_sw_cmd_i 改为接收顶层单次捕获后的单个
//                          寄存值，LP 进出相对原判决延后 1 拍（非时序关键路径）
// =============================================================================

`timescale 1ns/1ps

`include "link_layer_defines.vh"

module link_rx_lcmsm (
    // =========================================================================
    // 时钟与复位
    // =========================================================================
    input  wire        mb_clk,                 // 数据通路主时钟 2 GHz
    input  wire        mb_rst_n,               // 数据通路同步复位，低有效

    // =========================================================================
    // 配置接口（apb_csr 解码后具名信号直送，经顶层同步）
    // =========================================================================
    input  wire        csr_cfg_done_i,             // 配置完成脉冲，S_CONFIG 下有效

    // =========================================================================
    // PHY 就绪引脚（Logic PHY → 顶层 sync_cell 同步至 mb_clk）
    // =========================================================================
    input  wire        rx_ready_i,             // cpif_rx_ready（PHY 初始化完成，电平）

    // =========================================================================
    // 事件交互（link_event_process）
    // =========================================================================
    input  wire        evt_done_i,             // 事件完成回报（1 cycle 脉冲）

    // =========================================================================
    // PAIF 低功耗命令（PL→LINK，仅低功耗编码；已由顶层 link_layer 单次捕获）
    //   lp_sw_req_i = lp_sw_cmd_vld（捕获有效，req 高期间为 1）
    //   lp_sw_cmd_i = lp_sw_cmd_cap（req 上升沿单次采样的命令值）
    //   语义与原始 PAIF 请求/命令一致：cmd 只在 req 有效期间被消费。
    // =========================================================================
    input  wire        lp_sw_req_i,            // PAIF 命令请求（捕获后有效电平）
    input  wire [3:0]  lp_sw_cmd_i,            // 0001=LP_ENTER / 0010=LP_EXIT（捕获后值）

    // =========================================================================
    // 重训练请求（link_tx.retry_ctrl，经顶层 mb_clk→mb_clk 同步）
    // =========================================================================
    input  wire        retrain_timeout_req_i,  // 超时重传失败达阈值（1 cycle 脉冲）
    input  wire        retrain_nack_req_i,     // NACK 重传失败达阈值（1 cycle 脉冲）

    // =========================================================================
    // 对端事件通知（link_event_process）
    // =========================================================================
    input  wire        rx_evt_lp_sync_i,        // 对端进入 U2 同步边带指令（1 cycle 脉冲），收到即进 U2
    input  wire        rx_evt_lp_exit_i,       // 对端 LP_EXIT 通知

    // =========================================================================
    // link_rx 状态输入（deskew）
    // =========================================================================
    input  wire        lane_dskew_done_i,      // deskew done（对齐成功或 align_cnt 计满 7 拉高，状态机只接 done）

    // =========================================================================
    // 状态回报 / 下游控制 / 事件请求（仅 RETRAIN cmd）
    // =========================================================================
    output wire [3:0]  link_state_o,           // RX link FSM 当前状态（经 apb_csr 聚合）
    output reg         rx_path_en_o,           // RX 数据通路使能；S_NORMAL 时为 1（= rx_active）
    output reg         train_start_o,          // 触发 lane_dskew 训练（1 cycle 脉冲）
    output reg  [1:0]  lp_state_o,             // 低功耗状态：00=U0 / 10=U2
    output reg         link_evt_req_o,         // RETRAIN 事件请求（1 cycle 脉冲，与 cmd 同拍）
    output reg  [15:0] link_evt_cmd_o          // EventCmd：16'h0080=RETRAIN
);

    // =========================================================================
    // 内部信号定义
    // =========================================================================

    // ---- FSM 状态（one-hot，state_ctl 三段式风格） ------------------------------
    reg  [5:0]  cur_state;                     // one-hot：[0]CONFIG/[1]PHY_INIT/[2]PHY_TRAIN/[3]LINK_TRAIN/[4]NORMAL/[5]LP
    wire [5:0]  next_state;                    // one-hot 次态（state_ctl 归并）
    // verilator lint_off UNOPTFLAT
    // state_ctl 高位引用低位做优先级链（参考风格固有结构，非真实组合环）
    wire [13:0] state_ctl;                     // 状态控制信号（第一段组合）
    wire        st_config;                     // 当前状态位
    wire        st_pl_init;
    wire        st_phy_train;
    wire        st_link_train;
    wire        st_link_active;
    wire        st_low_power;
    wire        st_config_nxt;                 // 各状态次态位（第二段组合）
    wire        st_pl_init_nxt;
    wire        st_phy_train_nxt;
    wire        st_link_train_nxt;
    wire        st_link_active_nxt;
    wire        st_low_power_nxt;
    // verilator lint_on UNOPTFLAT
    wire [2:0]  cur_state_bin;                 // one-hot → 二进制（STATUS_REG 回读用）

    // ---- 训练控制 ------------------------------------------------------------------
    reg         train_entered;                 // 已进入 S_LINK_TRAIN 标志（train_start_o 单拍生成用）
    reg         init_train_done;               // 首次训练标记（复位 0，首次 LINK_TRAIN 完成置 1）

    // ---- CDC 收敛修复（Ac_conv02 46FD）：事件寄存解码 --------------------------
    // lp_enter_req/lp_exit_done 为多路跨域同步信号的组合结果，先寄存成单 bit
    // （lp_enter_evt_r/lp_exit_evt_r）再参与次态，使 st_low_power_nxt 的组合扇入
    // 退化为"单事件 bit + 状态位"。LP 进出延后 1 拍。
    reg         lp_enter_evt_r;                // lp_enter_req 寄存（mb 域 1 bit）
    reg         lp_exit_evt_r;                 // lp_exit_done 寄存（mb 域 1 bit）

    // ---- 转移条件预计算 --------------------------------------------------------------
    wire        phy_train_done;                // RETRAIN cmd 握手完成
    wire        go_link_train;                 // PHY_TRAIN → LINK_TRAIN（首次 train 且多 lane）
    wire        go_normal_tr;                  // PHY_TRAIN → NORMAL（其它所有情况）
    wire        retrain_req;                   // 重训练请求聚合（超时 / NACK 阈值）
    wire        lp_enter_req;                  // U2 进入请求（收到对端同步边带指令）→ 寄存
    wire        lp_exit_done;                  // U2 退出握手完成（本端命令+事件完成或对端通知）→ 寄存
    wire        retrain_fire;                  // RETRAIN cmd 触发（retrain 条件拍）

    // =========================================================================
    // 组合逻辑：转移条件预计算
    // =========================================================================
    assign phy_train_done = evt_done_i;
    // 训练转移：多 lane 时需先走 LINK_TRAIN（deskew）；单 lane 编译期省去
    //   LINK_LANE1 定义 ↔ lane≥2 / 不定义 ↔ 单 lane，`ifdef 编译前选代码段
    `ifdef LINK_LANE1
        assign go_link_train = phy_train_done & ~init_train_done;
        assign go_normal_tr  = phy_train_done & init_train_done;
    `else
        assign go_link_train = 1'b0;
        assign go_normal_tr  = phy_train_done;
    `endif
    assign retrain_req    = retrain_timeout_req_i | retrain_nack_req_i;
    // 进 U2：仅等对端同步边带指令（L1 §4.5.2 两通路均以收到同步边带指令进 U2；
    // 寄存器配置 / PAIF LP_ENTER 意图为上游握手，由 link_event_process 处理，同步信号到达即满足意图）
    assign lp_enter_req   = rx_evt_lp_sync_i;                                              // → lp_enter_evt_r
    assign lp_exit_done   = (lp_sw_req_i & (lp_sw_cmd_i == 4'b0010) & evt_done_i)
                          | rx_evt_lp_exit_i;                                              // → lp_exit_evt_r
    assign retrain_fire   = (st_link_active & retrain_req)
                          | (st_low_power & lp_exit_evt_r);

    // 事件寄存（CDC 收敛修复：跨域输入只经 D 端进寄存，次态组合不再直接见同步输出）
    always @(posedge mb_clk) begin
        if (!mb_rst_n) begin
            lp_enter_evt_r <= 1'b0;
            lp_exit_evt_r  <= 1'b0;
        end else begin
            lp_enter_evt_r <= lp_enter_req;
            lp_exit_evt_r  <= lp_exit_done;
        end
    end

    // =========================================================================
    // FSM（state_ctl 三段式：条件位 → 次态位 → 状态寄存器）
    // =========================================================================

    // --------------------------
    // 第一段：组合逻辑计算状态控制信号
    // --------------------------
    assign st_config      = cur_state[0];
    assign st_pl_init     = cur_state[1];
    assign st_phy_train   = cur_state[2];
    assign st_link_train  = cur_state[3];
    assign st_link_active = cur_state[4];
    assign st_low_power   = cur_state[5];

    assign state_ctl[0]  = st_config & csr_cfg_done_i;                          // CONFIG → PHY_INIT
    assign state_ctl[1]  = st_config & ~csr_cfg_done_i;                         // 保持
    assign state_ctl[2]  = st_pl_init & rx_ready_i & go_link_train;             // PHY_INIT → LINK_TRAIN（需 deskew）
    assign state_ctl[3]  = st_pl_init & rx_ready_i & go_normal_tr;              // PHY_INIT → NORMAL（不需 deskew）
    assign state_ctl[4]  = st_pl_init & ~(|state_ctl[3:2]);                      // 保持
    assign state_ctl[5]  = st_phy_train & go_normal_tr;                         // PHY_TRAIN → NORMAL
    assign state_ctl[6]  = st_phy_train & ~state_ctl[5];                        // 保持
    assign state_ctl[7]  = st_link_train & lane_dskew_done_i;                   // LINK_TRAIN → NORMAL（状态机只接 done；对齐失败仅报中断 F-05，不进状态机）
    assign state_ctl[8]  = st_link_train & ~lane_dskew_done_i;                  // 保持
    assign state_ctl[9]  = st_link_active & retrain_req;                        // NORMAL → PHY_TRAIN（同时发 RETRAIN cmd）
    assign state_ctl[10] = st_link_active & lp_enter_evt_r & ~state_ctl[9];       // NORMAL → LP
    assign state_ctl[11] = st_link_active & ~(|state_ctl[10:9]);                // 保持
    assign state_ctl[12] = st_low_power & lp_exit_evt_r;                         // LP → PHY_TRAIN（同时发 RETRAIN cmd）
    assign state_ctl[13] = st_low_power & ~lp_exit_evt_r;                        // 保持

    // --------------------------
    // 第二段：组合逻辑计算下一状态
    // --------------------------
    assign st_config_nxt      = state_ctl[1] | ~mb_rst_n;
    assign st_pl_init_nxt     = (state_ctl[0] | state_ctl[4]) & mb_rst_n;
    assign st_phy_train_nxt   = (state_ctl[6] | state_ctl[9] | state_ctl[12]) & mb_rst_n;
    assign st_link_train_nxt  = (state_ctl[2] | state_ctl[8]) & mb_rst_n;
    assign st_link_active_nxt = (state_ctl[3] | state_ctl[5] | state_ctl[7] | state_ctl[11]) & mb_rst_n;
    assign st_low_power_nxt   = (state_ctl[10] | state_ctl[13]) & mb_rst_n;

    assign next_state = {st_low_power_nxt, st_link_active_nxt, st_link_train_nxt,
                         st_phy_train_nxt, st_pl_init_nxt, st_config_nxt};

    // --------------------------
    // 第三段：时序逻辑寄存当前状态
    // （code/common 无带复位通用寄存器 IP：data_pipe 无复位、generic_dffs_q 绑
    //  foundry 单元不可仿真，状态寄存器用 always 实现，复位进 S_CONFIG one-hot）
    // --------------------------
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            cur_state <= 6'b000_001;         // S_CONFIG
        else
            cur_state <= next_state;
    end

    // one-hot → 二进制（STATUS_REG.LINK_FSM_STATE 编码，复用 code/common IP）
    onehot_to_binary #(
        .ONEHOT_WIDTH (6)
    ) u_state_onehot2bin (
        .one_hot_in  (cur_state),
        .binary_out  (cur_state_bin)
    );

    // =========================================================================
    // 训练控制 / 事件请求 / 下游控制（时序）
    // =========================================================================

    // init_train_done：复位 0，首次 LINK_TRAIN 完成（lane_dskew_done → 进 NORMAL）置 1
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            init_train_done <= 1'b0;
        else if (st_link_train && lane_dskew_done_i)
            init_train_done <= 1'b1;
    end

    // RETRAIN 事件请求/命令：retrain 触发拍拉高 1 拍，cmd=16'h0080（Type=0）
    always @(posedge mb_clk) begin
        if (!mb_rst_n) begin
            link_evt_req_o <= 1'b0;
        end else begin
            link_evt_req_o <= retrain_fire;
        end
    end

    // EventCmd 寄存（retrain 触发拍装载，独立 always 避免 if 嵌套）
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            link_evt_cmd_o <= 16'b0;
        else if (retrain_fire)
            link_evt_cmd_o <= {1'b0, 7'b0, `EVT_OP_RETRAIN};
    end

    // rx_path_en：S_NORMAL 态指示（= rx_active），与 err_discard 相互独立
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            rx_path_en_o <= 1'b0;
        else if (st_link_active_nxt)
            rx_path_en_o <= 1'b1;
        else if (!st_link_active_nxt && st_link_active)
            rx_path_en_o <= 1'b0;
    end

    // 低功耗状态：S_LP 内为 U2，退出 S_LP 回 U0
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            lp_state_o <= 2'b00;
        else if (st_low_power && lp_exit_evt_r)
            lp_state_o <= 2'b00;               // U0
        else if (st_low_power)
            lp_state_o <= 2'b10;               // U2
    end

    // train_entered：进 S_LINK_TRAIN 前清零，首拍后置 1
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            train_entered <= 1'b0;
        else if (st_phy_train && go_link_train)
            train_entered <= 1'b0;
        else if (st_link_train && !train_entered)
            train_entered <= 1'b1;
    end

    // lane_dskew 训练触发：S_LINK_TRAIN 首拍拉高 1 cycle
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            train_start_o <= 1'b0;
        else
            train_start_o <= st_link_train & ~train_entered;
    end

    // =========================================================================
    // 状态回报（组合，经 apb_csr 聚合进 STATUS_REG.LINK_FSM_STATE）
    // =========================================================================
    assign link_state_o = {1'b0, cur_state_bin};

endmodule
