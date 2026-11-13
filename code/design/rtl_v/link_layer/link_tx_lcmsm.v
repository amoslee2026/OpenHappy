// =============================================================================
// module name：link_tx_lcmsm
// author: OpenHappy Team
// MAS：spec/link_layer/submods/link_layer/mas/link_tx_lcmsm_mas.md
// 功能概述: TX 侧本地控制状态机（Local Control State Machine）。按 L1 §3 实现
//           TX 侧 link_fsm 6 态推进：CONFIG → PHY_INIT → PHY_TRAIN →（首次
//           train 且多 lane → LINK_TRAIN）→ NORMAL(U0) ⇄ LP(U2)。
//           本 FSM 只输出状态信息（link_state_o / lp_state_o），TX 数据通路
//           行为（收发使能 / 训练 Pattern 发送）由 link_tx 按 link_state_i
//           自行派生。事件命令仅生成 RETRAIN cmd（16'h0080，retrain 触发时
//           与进 PHY_TRAIN 同时发出）；PHY_INIT / LANE_ON / LANE_OFF /
//           RX_READY cmd 均属寄存器通路（INIT_CFG_REG），不由本模块生成。
//           init_train_done 标记区分首次训练（PHY_INIT 路径）与重训练：
//           仅首次训练且多 lane 进入 LINK_TRAIN（deskew 驻留 2 拍），重训练
//           PHY_TRAIN 后直接到 NORMAL。
//           FSM 为 state_ctl 三段式风格（条件位 → 次态位 → 状态寄存器）。
//           本模块全部逻辑处于 mb_clk（2 GHz）数据通路时钟域。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   <日期>  <修订人>  <变更概述>
//  2026-08-08  OpenHappy Team  初始版本（对齐 MAS v1.0）
//  2026-08-09  OpenHappy Team  重构为 L1 §3 6 态模型：删 S_BW_SWITCH（不支持动态
//                          切换）/ S_RETRAIN（并入 PHY_TRAIN）；新增
//                          S_PHY_TRAIN / init_train_done 首次训练标记；
//                          PHY_INIT cmd 移交寄存器通路（INIT_CFG_REG），本模块
//                          仅生成 RETRAIN cmd；删除 bridge_en_i /
//                          rx_evt_lane_on/off_i（无消费）；lp_sw_cmd 仅低功耗
//  2026-08-11  OpenHappy Team  新增 INIT_CFG_REG.RETRAIN 软件训练触发：S_NORMAL 下
//                          置位发 RETRAIN cmd + 跳 S_PHY_TRAIN；S_PHY_TRAIN 下
//                          置位仅发 cmd（退出条件不变，仍等 evt_done）
//  2026-09-16  cdc-resolver CDC 修复(Ac_conv02 46FD)：lp_enter/lp_exit 事件
//                          先寄存成单 bit（lp_enter_evt_r/lp_exit_evt_r）再参与
//                          次态组合，切断多路跨域同步信号同拍汇聚进 st_low_power_nxt；
//                          lp_sw_req_i/lp_sw_cmd_i 改为接收顶层单次捕获后的单个
//                          寄存值（PAIF 一步电平握手语义不变），LP 进出相对原判决
//                          延后 1 拍（非时序关键路径）
//  2026-10-08  OpenHappy Team  LL-20260930-N4-v3（F1a）：+1 组合输出口 link_active_nxt_o
//                          （= st_link_active_nxt 次态位，L220 既有 net，0 逻辑）——link_tx
//                          父侧副本寄存器 link_active_cp 的 D 源（真同 D 同沿，cp ≡
//                          cur_state[4] 逐拍恒等含迁移/复位拍；D=Q 直连副本影子禁用）；
//                          状态迁移方程不变。
// =============================================================================

`timescale 1ns/1ps

`include "link_layer_defines.vh"

module link_tx_lcmsm (
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
    input  wire        tx_ready_i,             // cpif_tx_ready（PHY 初始化完成，电平）

    // =========================================================================
    // link_tx deskew done（link_tx.v 发满 2 拍 Pattern 拉高）
    // =========================================================================
    input  wire        deskew_done_i,          // deskew done（1 cycle 脉冲，状态机只接 done）

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
    input  wire        csr_init_retrain_i,     // INIT_CFG_REG.RETRAIN 软件训练触发沿（1 cycle 脉冲，S_NORMAL/S_PHY_TRAIN 下发 cmd）

    // =========================================================================
    // 对端事件通知（link_event_process）
    // =========================================================================
    input  wire        rx_evt_lp_enter_i,      // 对端 LP_ENTER 通知（1 cycle 脉冲）
    input  wire        rx_evt_lp_exit_i,       // 对端 LP_EXIT 通知

    // =========================================================================
    // 状态回报 / 事件请求（仅 RETRAIN cmd）
    // =========================================================================
    output wire [3:0]  link_state_o,           // link FSM 当前状态（经 apb_csr 聚合进 STATUS_REG）
    output wire        link_active_o,          // S_NORMAL 状态位（=cur_state[4]，与 link_state_o 同拍，供 link_tx 直接采，免 onehot2bin+判等）
    output wire        link_active_nxt_o,      // S_NORMAL 次态位（=st_link_active_nxt，0 逻辑；F1a link_tx link_active_cp D 源）
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
    wire [15:0] state_ctl;                     // 状态控制信号（第一段组合）
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
    reg         init_train_done;               // 首次训练标记（复位 0，S_PHY_TRAIN 首次退出置 1）

    // ---- CDC 收敛修复（Ac_conv02 46FD）：事件寄存解码 --------------------------
    // lp_enter_req/lp_exit_done 为多路跨域同步信号的组合结果，先寄存成单 bit
    // （lp_enter_evt_r/lp_exit_evt_r）再参与次态，使 st_low_power_nxt 的组合扇入
    // 退化为"单事件 bit + 状态位"，消除同拍多信号组合汇聚。LP 进出延后 1 拍。
    reg         lp_enter_evt_r;                // lp_enter_req 寄存（mb 域 1 bit）
    reg         lp_exit_evt_r;                 // lp_exit_done 寄存（mb 域 1 bit）

    // ---- 转移条件预计算 --------------------------------------------------------------
    wire        phy_ready_ok;                  // PHY ready（事件完成 + phy_ready 电平）
    wire        phy_train_done;                // RETRAIN cmd 握手完成
    wire        go_link_train;                 // PHY_TRAIN → LINK_TRAIN（首次 train 且多 lane）
    wire        go_normal_tr;                  // PHY_TRAIN → NORMAL（其它所有情况）
    wire        retrain_req;                   // 重训练请求聚合（超时 / NACK 阈值）
    wire        sw_retrain_req;                 // 软件寄存器触发训练（INIT_CFG_REG.RETRAIN）
    wire        lp_enter_req;                  // U2 进入请求（本端命令或对端通知）→ 寄存
    wire        lp_exit_done;                  // U2 退出握手完成（本端命令+事件完成或对端通知）→ 寄存
    wire        retrain_fire;                  // RETRAIN cmd 触发（retrain 条件拍）

    // =========================================================================
    // 组合逻辑：转移条件预计算
    // =========================================================================
    assign phy_ready_ok   = tx_ready_i;                    // PHY ready = cpif_tx_ready 引脚（非事件握手）
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
    assign sw_retrain_req = csr_init_retrain_i;   // 软件训练触发（沿脉冲）
    assign lp_enter_req   = (lp_sw_req_i & (lp_sw_cmd_i == 4'b0001)) | rx_evt_lp_enter_i;  // → lp_enter_evt_r
    assign lp_exit_done   = (lp_sw_req_i & (lp_sw_cmd_i == 4'b0010) & evt_done_i)
                          | rx_evt_lp_exit_i;                                              // → lp_exit_evt_r
    assign retrain_fire   = (st_link_active & retrain_req)
                          | ((st_link_active | st_phy_train) & sw_retrain_req)
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
    assign state_ctl[2]  = st_pl_init & phy_ready_ok;                       // PHY_INIT → PHY_TRAIN
    assign state_ctl[3]  = 1'b0;                                            // 未用（事件超时已去除）
    assign state_ctl[4]  = st_pl_init & ~(|state_ctl[3:2]);                 // 保持
    assign state_ctl[5]  = st_phy_train & go_link_train;                    // PHY_TRAIN → LINK_TRAIN（首次 + 多 lane）
    assign state_ctl[6]  = st_phy_train & go_normal_tr;                     // PHY_TRAIN → NORMAL（其它所有）
    assign state_ctl[7]  = st_phy_train & ~(|state_ctl[6:5]);               // 保持（等 RETRAIN cmd 握手）
    assign state_ctl[8]  = st_link_train & deskew_done_i;                   // LINK_TRAIN → NORMAL（link_tx 发满 2 拍 Pattern 拉高 done）
    assign state_ctl[9]  = st_link_train & ~deskew_done_i;                  // 保持（驻留）
    assign state_ctl[10] = st_link_active & (retrain_req | sw_retrain_req);   // NORMAL → PHY_TRAIN（同时发 RETRAIN cmd）
    assign state_ctl[11] = st_link_active & lp_enter_evt_r & ~(|state_ctl[10:10]);  // NORMAL → LP
    assign state_ctl[12] = st_link_active & ~(|state_ctl[11:10]);                 // 保持
    assign state_ctl[13] = st_low_power & lp_exit_evt_r;                     // LP → PHY_TRAIN（同时发 RETRAIN cmd）
    assign state_ctl[14] = st_low_power & ~lp_exit_evt_r;                    // 保持
    assign state_ctl[15] = 1'b0;                                            // 未用

    // --------------------------
    // 第二段：组合逻辑计算下一状态
    // --------------------------
    assign st_config_nxt      = (state_ctl[1] | state_ctl[3]) | ~mb_rst_n;
    assign st_pl_init_nxt     = (state_ctl[0] | state_ctl[4]) & mb_rst_n;
    assign st_phy_train_nxt   = (state_ctl[2] | state_ctl[7] | state_ctl[10] | state_ctl[13]) & mb_rst_n;
    assign st_link_train_nxt  = (state_ctl[5] | state_ctl[9]) & mb_rst_n;
    assign st_link_active_nxt = (state_ctl[6] | state_ctl[8] | state_ctl[12]) & mb_rst_n;
    assign st_low_power_nxt   = (state_ctl[11] | state_ctl[14]) & mb_rst_n;

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
    // 训练控制 / 事件请求 / 低功耗状态（时序）
    // =========================================================================

    // init_train_done：复位 0，S_PHY_TRAIN 首次退出置 1
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            init_train_done <= 1'b0;
        else if (phy_train_done && st_phy_train)
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

    // 低功耗状态：S_LP 内为 U2，退出 S_LP 回 U0
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            lp_state_o <= 2'b00;
        else if (st_low_power && lp_exit_evt_r)
            lp_state_o <= 2'b00;               // U0
        else if (st_low_power)
            lp_state_o <= 2'b10;               // U2
    end

    // =========================================================================
    // 状态回报（组合，经 apb_csr 聚合进 STATUS_REG.LINK_FSM_STATE）
    // =========================================================================
    assign link_state_o  = {1'b0, cur_state_bin};

    // =========================================================================
    // S_NORMAL 使能（直接采状态寄存器位 cur_state[4]，与 link_state_o 同拍）
    // 将 onehot2bin + link_state_i==4'h4 判等从 link_tx 关键数据/CRC 组合链摘出
    // =========================================================================
    assign link_active_o = st_link_active;

    // =========================================================================
    // S_NORMAL 次态位（F1a，LL-20260930-N4-v3：skill7 寄存器复制——0 逻辑直出 =
    // st_link_active_nxt 次态位，非 D=Q 直连副本影子）
    // link_tx 父侧副本寄存器 link_active_cp 的 D 源（真同 D 同沿：cp ≡ cur_state[4]
    // 逐拍恒等含迁移/复位拍）；状态迁移方程不变，link_active_o 扇出缓冲树由 cp 副本分担
    // =========================================================================
    assign link_active_nxt_o = st_link_active_nxt;

endmodule
