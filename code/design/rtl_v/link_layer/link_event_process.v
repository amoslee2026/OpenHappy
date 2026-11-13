// =============================================================================
// module name：link_event_process
// author: OpenHappy Team
// MAS：spec/link_layer/submods/link_layer/mas/link_event_process_mas.md
// 功能概述: 链路层事件处理单元，实现 CPIF 事件接口双方向管理。
//           TX 事件（LINK→PHY）：汇聚 lcmsm 事件请求（PHY_INIT / RETRAIN，
//           Type=0）、PAIF lp_sw 命令（LP_ENTER / LP_EXIT / LANE_ON /
//           LANE_OFF，Type=1）、寄存器通路触发（INIT_CFG_REG / LP_CFG_REG
//           置位沿，Type=1）三路事件源，仲裁后翻译为 16 bit EventCmd 经
//           cpif_tx_event_* 发往 Logic PHY，等待 ack 后回报 evt_done；
//           RX 事件（PHY→LINK）：接收对端 Type=1 事件，回
//           ack + status=8'h00，解码生成 rx_evt_lp_* 通知脉冲
//           （lane LANE_ON/OFF 通知输出已删——无消费侧，2026-10-08 W287b 正向解决）。同时
//           维护 irq_lp_transition_o。
//           初始化流程新增（G-03/G-04）：RX 事件解码 0x7F（RX READY，TX 侧
//           收到）→ irq_rx_ready_o 中断脉冲；0x40 按方向区分——本侧已发起
//           LANE_ON（lane_on_issued_r）→ 收到 0x40 为完成返回 → irq_lane_on_done_o
//           中断脉冲；未发起 → 仅回 ack（无通知输出）。
// 编码约束（本模块强制）:
//   1. FSM 三段式：时序状态寄存器 + 次态组合逻辑（always @(*)）+ 时序输出；
//   2. 每个 always 块内 if 判断（不含 reset 判断）不超过 2 个，禁止深层
//      嵌套，条件用 && 归并或预计算 wire；
//   3. 组合逻辑只允许 assign 赋值（FSM 次态逻辑除外）；
//   4. for 循环仅用于 generate 实例化。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   <日期>  <修订人>  <变更概述>
//  2026-08-08  OpenHappy Team  初始版本（对齐 MAS v1.0）
//  2026-08-08  OpenHappy Team  去状态机：evt FSM 删除，改 tx_busy 在途标志 +
//                          timeout_cnt 事务化实现（受理→req→ack/超时回报），
//                          RX 事件 rx_accept 单拍 ack + 通知
//  2026-08-11  OpenHappy Team  去除 250 cycle TX 事件超时看门狗，tx_busy 仅由
//                          ack（resp_fire）清零，evt_timeout_o 下游链路同步清除
//  2026-08-17  OpenHappy Team  lp_sw_ack_o 改一步电平握手：由脉冲（lp_ack_fire 单拍）
//                          改为电平跟随 req——event 完成（lp_ack_fire）且 req 高才
//                          set lp_ack_hold_r，req 拉低 clear，lp_sw_ack_o = req & hold；
//                          ack 不再等完整 event 回路作为单拍脉冲，而保持到 req 撤除。
//  2026-09-11  OpenHappy Team  初始化流程补齐（G-03/G-04）：RX 事件新增 0x7F（RX READY）
//                          → irq_rx_ready_o 中断；0x40 按方向区分（lane_on_issued_r：
//                          本侧发起 LANE_ON → 完成返回中断 irq_lane_on_done_o，未发起
//                          → 仅回 ack（lane 通知输出后续删除，2026-10-08 W287b 解决）。
//  2026-09-16  cdc-resolver  CDC 违例修复（Ac_unsync01 33E3：lp_ack_hold_r）：
//                          lp_sw_req_i/lp_sw_cmd_i（顶层 io.sdc 定域 mb_clk）进模块
//                          先 2FF 同步到 sb_clk 再消费（内部全部改接同步后信号），消除
//                          mb→sb 直接采样；PAIF 一步电平握手语义不变（req 保持时段 cmd
//                          稳定，多 bit 同步不撕裂）
// =============================================================================

`timescale 1ns/1ps

`include "link_layer_defines.vh"

module link_event_process (
    // =========================================================================
    // 时钟与复位（sb_clk 800 MHz 控制通路域）
    // =========================================================================
    input  wire        sb_clk,                  // 控制通路时钟 800 MHz
    input  wire        sb_rst_n,                // 控制通路异步复位，低有效

    // =========================================================================
    // PAIF 控制接口（协议层 PL → LINK）
    // =========================================================================
    input  wire [3:0]  lp_sw_cmd_i,             // PAIF 命令：0001=LP_ENTER / 0010=LP_EXIT / 0100=带宽增加 / 1000=带宽减少（mb 域，进模块 2FF 同步）
    input  wire        lp_sw_req_i,             // PAIF 命令请求（PL 保持至 ack；mb 域，进模块 2FF 同步）
    output wire        lp_sw_ack_o,             // PAIF 命令应答（一步电平握手：event 完成且 req 高才拉高，req 拉低即拉低）
    output wire [3:0]  lp_sw_sts_o,             // PAIF 命令状态（EventStatus[3:0]）

    // =========================================================================
    // lcmsm 事件请求接口（link_tx_lcmsm / link_rx_lcmsm）
    // =========================================================================
    input  wire        lcsm_tx_evt_req_i,       // TX lcmsm 事件请求（1 cycle 脉冲）
    input  wire [15:0] lcsm_tx_evt_cmd_i,       // TX lcmsm EventCmd（16'h0044=PHY_INIT / 16'h0080=RETRAIN）
    input  wire        lcsm_rx_evt_req_i,       // RX lcmsm 事件请求（1 cycle 脉冲）
    input  wire [15:0] lcsm_rx_evt_cmd_i,       // RX lcmsm EventCmd（16'h0044=PHY_INIT / 16'h0080=RETRAIN）

    // =========================================================================
    // 寄存器通路触发（apb_csr 置位沿，经顶层同步至 sb_clk）
    // =========================================================================
    input  wire        csr_init_phy_init_i,         // INIT_CFG_REG.CONFIG_DONE 置位沿（发 PHY_INIT cmd，与 lcmsm 状态推进共用）
    input  wire        csr_init_lane_on_i,          // INIT_CFG_REG.LANE_ON 置位沿
    input  wire        csr_init_lane_off_i,         // INIT_CFG_REG.LANE_OFF 置位沿
    input  wire        csr_init_rx_ready_i,         // INIT_CFG_REG.RX_READY 置位沿
    input  wire [5:0]  csr_lp_cmd_i,                // LP_CFG_REG 置位沿：bit0=LP_ENTER / bit4=LP_EXIT（其余 opcode 待 L1 §6 定义）

    // =========================================================================
    // CPIF TX 事件接口（LINK → Logic PHY）
    // =========================================================================
    output wire        cpif_tx_event_enable_o,  // 发送事件接口使能（复位释放后常 1）
    output wire        cpif_tx_event_req_o,     // 发送事件请求（1 cycle 脉冲）
    output wire [15:0] cpif_tx_event_cmd_o,     // 发送事件命令 EventCmd
    input  wire        cpif_tx_event_ack_i,     // 发送事件序列结束指示（PHY→LINK）
    input  wire [7:0]  cpif_tx_event_status_i,  // 发送事件状态 EventStatus（与 ack 同拍有效）

    // =========================================================================
    // CPIF RX 事件接口（Logic PHY → LINK）
    // =========================================================================
    output wire        cpif_rx_event_enable_o,  // 接收事件接口使能（复位释放后常 1）
    input  wire        cpif_rx_event_req_i,     // 接收事件请求（对端发起）
    input  wire [15:0] cpif_rx_event_cmd_i,     // 接收事件命令 EventCmd
    output wire        cpif_rx_event_ack_o,     // 接收事件序列结束指示（LINK→PHY）
    output wire [7:0]  cpif_rx_event_status_o,  // 接收事件状态（固定回 8'h00）

    // =========================================================================
    // 回报两侧 lcmsm / 中断
    // =========================================================================
    output wire        evt_done_o,              // TX 事件完成回报（1 cycle 脉冲，广播两 lcmsm）
    output wire        rx_evt_lp_enter_o,       // 对端 LP_ENTER 事件通知（1 cycle 脉冲）
    output wire        rx_evt_lp_exit_o,        // 对端 LP_EXIT 事件通知（1 cycle 脉冲）
    output wire        irq_lp_transition_o,     // 低功耗进出中断脉冲（L1 §4.9 F-03）
    output wire        irq_rx_ready_o,          // TX 侧收到对端 RX READY（0x7F，Type=1）中断脉冲（G-03，SB 域）
    output wire        irq_lane_on_done_o       // TX 侧收到 LANE ON 完成（0x40 返回方向，Type=1）中断脉冲（G-04，SB 域）
);

    // =========================================================================
    // 编译期参数与本地编码
    // =========================================================================
    // EVT_OP_PHY_INIT / EVT_OP_RETRAIN / EVT_OP_LP_ENTER / EVT_OP_LP_EXIT /
    // EVT_OP_LANE_ON / EVT_OP_LANE_OFF / EVT_OP_RX_READY 来自
    // link_layer_defines.vh（L1 §5.3.X，MAS §2.1/§4.3）

    // EventCmd 16 bit 全编码常量：{Type, 7'b0, opcode}（MAS §4.3）
    localparam [15:0] EVT_CMD_PHY_INIT = {1'b0, 7'b0, `EVT_OP_PHY_INIT};   // 16'h0044 Type=0
    localparam [15:0] EVT_CMD_LP_ENTER = {1'b1, 7'b0, `EVT_OP_LP_ENTER};   // 16'h8084 Type=1
    localparam [15:0] EVT_CMD_LP_EXIT  = {1'b1, 7'b0, `EVT_OP_LP_EXIT};    // 16'h8089 Type=1
    localparam [15:0] EVT_CMD_LANE_ON  = {1'b1, 7'b0, `EVT_OP_LANE_ON};    // 16'h8040 Type=1
    localparam [15:0] EVT_CMD_LANE_OFF = {1'b1, 7'b0, `EVT_OP_LANE_OFF};   // 16'h8041 Type=1
    localparam [15:0] EVT_CMD_RX_READY = {1'b1, 7'b0, `EVT_OP_RX_READY};    // 16'h807F Type=1


    // =========================================================================
    // 信号声明区
    // =========================================================================

    // ---- 事务控制（无状态机：busy 标志） ----------------------------------------
    reg         tx_busy;               // TX 事件在途（受理 → ack 回报）

    // ---- 事件处理寄存器 -----------------------------------------------------------
    reg  [15:0] evt_cmd_q;             // 当前处理中的 EventCmd
    reg         evt_from_lp_q;         // 当前事件来自 lp_sw 标志
    reg  [7:0]  evt_status_q;          // ack 同拍采样的 EventStatus 寄存

    // ---- 输出寄存器 ---------------------------------------------------------------
    reg         cpif_tx_event_req_r;
    reg  [15:0] cpif_tx_event_cmd_r;
    reg         cpif_rx_event_ack_r;
    reg         lp_ack_hold_r;         // lp_sw 应答电平保持：event 完成置 1、req 拉低清 0
    reg  [3:0]  lp_sw_sts_r;
    reg         evt_done_r;
    reg         rx_evt_lp_enter_r;
    reg         rx_evt_lp_exit_r;
    reg         irq_lp_transition_r;
    reg         lane_on_issued_r;       // 本侧已发起 LANE_ON（TX 侧）：csr_init_lane_on_i 置位、收到对端 0x40 完成清零
    reg         irq_rx_ready_r;         // 对端 RX READY（0x7F）中断脉冲（G-03）
    reg         irq_lane_on_done_r;     // 对端 LANE ON 完成（0x40 返回）中断脉冲（G-04）

    // ---- 事务控制条件 wire ---------------------------------------------------------
    wire        rx_accept;             // 对端事件受理（1 拍 ack + 通知）
    wire        tx_accept;             // TX 事件仲裁受理
    wire        resp_fire;             // ack 到达回报

    // ---- 仲裁预计算 ---------------------------------------------------------
    wire        lcsm_req;              // 两侧 lcmsm 事件请求（同拍同 cmd 去重）
    wire [15:0] lcmsm_cmd_sel;         // lcmsm cmd 选择（tx 优先）
    wire        reg_trig;              // 寄存器通路触发（已定义 opcode 的置位沿）
    wire [15:0] reg_evt_cmd;           // 寄存器通路 EventCmd 选择
    wire        lp_sel;                // 本拍仲裁选中 lp_sw 通路
    wire [15:0] tx_evt_cmd_sel;        // 仲裁选中的 EventCmd
    wire [15:0] lp_sw_evt_cmd;         // lp_sw_cmd（同步后）翻译结果

    // ---- 对端事件解码预计算 -------------------------------------------------------
    wire        rx_cmd_lp_enter;       // 对端 cmd 为 LP_ENTER（Type=1）
    wire        rx_cmd_lp_exit;        // 对端 cmd 为 LP_EXIT（Type=1）
    wire        rx_cmd_lane_on;        // 对端 cmd 为 LANE_ON（Type=1）
    wire        rx_cmd_rx_ready;       // 对端 cmd 为 RX_READY（0x7F，Type=1）→ TX 侧收到 ready → G-03 中断

    // ---- 回报 / 中断条件预计算 -----------------------------------------------------
    wire        lp_ack_fire;           // lp_sw 应答条件（ack 回报）
    wire        resp_lp_evt;           // 本端发起的 LP_ENTER / LP_EXIT 完成
    wire        rx_lp_evt;             // 对端 LP_ENTER / LP_EXIT 通知

    // =========================================================================
    // mb_clk(PAIF) → sb_clk：lp_sw_req / lp_sw_cmd 2FF 同步（CDC 修复 Ac_unsync01 33E3）
    //   lp_ctrl_* 芯片引脚在顶层 io.sdc 定域 mb_clk(prcl_clk)（2026-09-13），而本模块
    //   运行于 sb_clk（800MHz）控制域；PAIF 一步电平握手保证 req 高期间 cmd 稳定
    //   （PL 保持至 ack，见 MAS §5.1），此同步不改变握手语义，仅消除 mb→sb 直接采样
    //   （lp_ack_hold_r <= lp_sw_req_i）的亚稳态风险。
    //   注：lp_sw_cmd 为多 bit，但其仅在 lp_sw_req（同步后）有效期间被消费，双域保持
    //   协议使采样时刻总线已稳定 ≥2 sb 周期，不撕裂。
    // =========================================================================
    reg         lp_sw_req_sync1, lp_sw_req_sb;
    reg  [3:0]  lp_sw_cmd_sync1, lp_sw_cmd_sb;
    always @(posedge sb_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            lp_sw_req_sync1 <= 1'b0;
            lp_sw_req_sb    <= 1'b0;
            lp_sw_cmd_sync1 <= 4'b0;
            lp_sw_cmd_sb    <= 4'b0;
        end
        else begin
            lp_sw_req_sync1 <= lp_sw_req_i;
            lp_sw_req_sb    <= lp_sw_req_sync1;
            lp_sw_cmd_sync1 <= lp_sw_cmd_i;
            lp_sw_cmd_sb    <= lp_sw_cmd_sync1;
        end
    end

    // =========================================================================
    // lp_sw 命令翻译 function（MAS §5.1.1）
    //   lp_sw_cmd（sync 后） → Type=1 EventCmd；未定义编码返回 16'b0
    // =========================================================================
    function [15:0] translate_evt;
        input [3:0] lp_cmd;
        begin
            case (lp_cmd)
                4'b0001:  translate_evt = EVT_CMD_LP_ENTER;   // LP_ENTER → 16'h8084
                4'b0010:  translate_evt = EVT_CMD_LP_EXIT;    // LP_EXIT  → 16'h8089
                default:  translate_evt = 16'b0;
            endcase
        end
    endfunction

    // =========================================================================
    // 组合逻辑（仅 assign）
    // =========================================================================

    // 事务控制条件（无状态机；优先级：RX 事件 > lcmsm > lp_sw > 寄存器通路）
    assign rx_accept     = cpif_rx_event_req_i & ~tx_busy;
    assign tx_accept     = ~tx_busy & ~cpif_rx_event_req_i &
                           (lcsm_req || lp_sw_req_sb || reg_trig);
    assign resp_fire     = tx_busy & cpif_tx_event_ack_i;

    // 仲裁预计算（MAS §4.4：RX 事件 > lcmsm > lp_sw > 寄存器通路）
    assign lcsm_req      = lcsm_tx_evt_req_i || lcsm_rx_evt_req_i;
    assign lcmsm_cmd_sel = lcsm_tx_evt_req_i ? lcsm_tx_evt_cmd_i : lcsm_rx_evt_cmd_i;

    // 寄存器通路触发：仅已定义 event opcode 的置位沿受理
    // （csr_lp_cmd_i[1]/[2]/[3]/[5] 的 opcode 待 L1 §6 定义，当前不发起 event，MAS §4.3 注）
    assign reg_trig      = csr_init_phy_init_i || csr_init_lane_on_i || csr_init_lane_off_i ||
                           csr_init_rx_ready_i || csr_lp_cmd_i[0] || csr_lp_cmd_i[4];
    assign reg_evt_cmd   = csr_init_phy_init_i ? EVT_CMD_PHY_INIT :
                           csr_init_lane_on_i  ? EVT_CMD_LANE_ON  :
                           csr_init_lane_off_i ? EVT_CMD_LANE_OFF :
                           csr_init_rx_ready_i ? EVT_CMD_RX_READY :
                           csr_lp_cmd_i[0]     ? EVT_CMD_LP_ENTER :
                                             EVT_CMD_LP_EXIT;

    assign lp_sw_evt_cmd   = translate_evt(lp_sw_cmd_sb);
    assign lp_sel          = lp_sw_req_sb && !lcsm_req;
    assign tx_evt_cmd_sel  = lcsm_req    ? lcmsm_cmd_sel  :
                             lp_sw_req_sb ? lp_sw_evt_cmd  :
                                           reg_evt_cmd;

    // 对端 cmd 解码（仅 Type=1 的 LP / LANE 事件生成通知，其余容错丢弃）
    assign rx_cmd_lp_enter = (cpif_rx_event_cmd_i == EVT_CMD_LP_ENTER);
    assign rx_cmd_lp_exit  = (cpif_rx_event_cmd_i == EVT_CMD_LP_EXIT);
    assign rx_cmd_lane_on  = (cpif_rx_event_cmd_i == EVT_CMD_LANE_ON);
    // 0x7F 现为"对端收到不处理"容错丢弃；G-03 新增：本侧（TX 侧）收到对端 RX READY → 中断
    assign rx_cmd_rx_ready = (cpif_rx_event_cmd_i == EVT_CMD_RX_READY);

    // 回报 / 中断条件
    assign lp_ack_fire  = resp_fire && evt_from_lp_q;
    assign resp_lp_evt  = resp_fire &&
                          ((evt_cmd_q == EVT_CMD_LP_ENTER) || (evt_cmd_q == EVT_CMD_LP_EXIT));
    assign rx_lp_evt    = rx_accept && (rx_cmd_lp_enter || rx_cmd_lp_exit);

    // 事件接口使能（复位释放后常 1，MAS §2.1）
    assign cpif_tx_event_enable_o = 1'b1;
    assign cpif_rx_event_enable_o = 1'b1;

    // RX 事件状态固定回 8'h00（OK，MAS §2.1）
    assign cpif_rx_event_status_o = 8'h00;

    // 输出赋值
    assign cpif_tx_event_req_o   = cpif_tx_event_req_r;
    assign cpif_tx_event_cmd_o   = cpif_tx_event_cmd_r;
    assign cpif_rx_event_ack_o   = cpif_rx_event_ack_r;
    assign lp_sw_sts_o    = lp_sw_sts_r;

    // lp_sw_ack 寄存（cdc-resolver 2026-09-16）：跨域源必须是寄存器输出。原组合
    // `lp_sw_req_sb & lp_ack_hold_r` 为两个同域信号 AND，会触发 Ac_glitch03
    // （2 source 同域 fanin 可在采样沿尖峰），先打 1 拍成 lp_sw_ack_r 再输出，
    // 一步电平握手语义不变（ack 电平延后 1 sb 拍）。
    reg         lp_sw_ack_r;
    always @(posedge sb_clk or negedge sb_rst_n) begin
        if (!sb_rst_n)
            lp_sw_ack_r <= 1'b0;
        else
            lp_sw_ack_r <= lp_sw_req_sb & lp_ack_hold_r;   // 一步电平握手（sb 域，寄存后输出）
    end
    assign lp_sw_ack_o = lp_sw_ack_r;
    assign evt_done_o            = evt_done_r;
    assign rx_evt_lp_enter_o     = rx_evt_lp_enter_r;
    assign rx_evt_lp_exit_o      = rx_evt_lp_exit_r;
    assign irq_lp_transition_o   = irq_lp_transition_r;
    assign irq_rx_ready_o        = irq_rx_ready_r;
    assign irq_lane_on_done_o    = irq_lane_on_done_r;

    // =========================================================================
    // tx_busy：受理置位，ack 回报清零（无状态机）
    // =========================================================================
    always @(posedge sb_clk or negedge sb_rst_n) begin
        if (!sb_rst_n)
            tx_busy <= 1'b0;
        else if (resp_fire)
            tx_busy <= 1'b0;
        else if (tx_accept)
            tx_busy <= 1'b1;
    end

    // =========================================================================
    // evt FSM 第三段：时序输出与数据通路寄存器
    // =========================================================================

    // 当前事件 cmd / 来源寄存（仲裁受理拍加载）
    always @(posedge sb_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            evt_cmd_q     <= 16'b0;
            evt_from_lp_q <= 1'b0;
        end else if (tx_accept) begin
            evt_cmd_q     <= tx_evt_cmd_sel;
            evt_from_lp_q <= lp_sel;
        end
    end

    // EventStatus 寄存（ack 同拍采样，供回报使用）
    always @(posedge sb_clk or negedge sb_rst_n) begin
        if (!sb_rst_n)
            evt_status_q <= 8'b0;
        else if (resp_fire)
            evt_status_q <= cpif_tx_event_status_i;
    end

    // CPIF TX 事件请求（受理后 1 拍拉高 1 cycle）与 EventCmd 输出
    always @(posedge sb_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            cpif_tx_event_req_r <= 1'b0;
        end else begin
            cpif_tx_event_req_r <= tx_accept;
        end
    end

    // EventCmd 寄存（受理拍装载，独立 always 避免 if 嵌套）
    always @(posedge sb_clk or negedge sb_rst_n) begin
        if (!sb_rst_n)
            cpif_tx_event_cmd_r <= 16'b0;
        else if (tx_accept)
            cpif_tx_event_cmd_r <= tx_evt_cmd_sel;
    end

    // 脉冲类输出（默认每拍按条件重载，单拍脉冲）
    //   cpif_rx_event_ack_o ：rx_accept 单拍回 ack
    //   evt_done_o          ：resp_fire 回报
    //   lp_sw_ack_o         ：一步电平握手（见 lp_ack_hold_r，非脉冲）
    //   rx_evt_*_o          ：rx_accept 解码对端 cmd 生成通知脉冲
    //   irq_lp_transition_o ：本端 LP_ENTER/LP_EXIT 完成 或 对端 LP 事件通知
    //   irq_rx_ready_o      ：G-03：本侧（TX 侧）收到对端 RX READY（0x7F）完成通知
    //   irq_lane_on_done_o  ：G-04：本侧（TX 侧）已发起 LANE_ON、收到对端 0x40 返回完成
    // 方向判定（对端 0x40 两义，G-04）：
    //   本侧已发起 LANE_ON（lane_on_issued_r=1）→ 收到 0x40 为"完成返回"→ 中断；
    //   本侧未发起（lane_on_issued_r=0）→ 收到 0x40 为"LANE_ON 请求"→ 仅回 ack
    //   （lane 通知输出已删——无消费侧，2026-10-08 W287b 解决）。
    always @(posedge sb_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            cpif_rx_event_ack_r <= 1'b0;
            evt_done_r          <= 1'b0;
            rx_evt_lp_enter_r   <= 1'b0;
            rx_evt_lp_exit_r    <= 1'b0;
            irq_lp_transition_r <= 1'b0;
            irq_rx_ready_r      <= 1'b0;
            irq_lane_on_done_r  <= 1'b0;
        end else begin
            cpif_rx_event_ack_r <= rx_accept;
            evt_done_r          <= resp_fire;
            rx_evt_lp_enter_r   <= rx_accept && rx_cmd_lp_enter;
            rx_evt_lp_exit_r    <= rx_accept && rx_cmd_lp_exit;
            irq_lp_transition_r <= resp_lp_evt || rx_lp_evt;
            irq_rx_ready_r      <= rx_accept && rx_cmd_rx_ready;
            irq_lane_on_done_r  <= rx_accept && rx_cmd_lane_on && lane_on_issued_r;
        end
    end

    // lane_on_issued：本侧已发起 LANE_ON（TX 侧）标志。csr_init_lane_on_i 置位；
    // 收到对端 0x40（完成返回）清零，再次写 LANE_ON 重新武装。
    always @(posedge sb_clk or negedge sb_rst_n) begin
        if (!sb_rst_n)
            lane_on_issued_r <= 1'b0;
        else if (csr_init_lane_on_i)
            lane_on_issued_r <= 1'b1;
        else if (rx_accept && rx_cmd_lane_on && lane_on_issued_r)
            lane_on_issued_r <= 1'b0;
    end

    // lp_sw 应答电平保持（一步握手）：event 完成（lp_ack_fire）置 1，
    // req 拉低清 0，使 lp_sw_ack_o 跟随 req 电平、且只在 event 完成后才拉高。
    always @(posedge sb_clk or negedge sb_rst_n) begin
        if (!sb_rst_n)
            lp_ack_hold_r <= 1'b0;
        else if (lp_ack_fire)
            lp_ack_hold_r <= lp_sw_req_sb;
        else if (~lp_sw_req_sb)
            lp_ack_hold_r <= 1'b0;
    end

    // lp_sw 命令状态（与 lp_sw_ack_o 同拍；完成回 status[3:0]）
    always @(posedge sb_clk or negedge sb_rst_n) begin
        if (!sb_rst_n)
            lp_sw_sts_r <= 4'b0;
        else if (lp_ack_fire)
            lp_sw_sts_r <= evt_status_q[3:0];
    end

endmodule
