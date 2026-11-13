// =============================================================================
// module name：sbc_rx
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/sideband/mas/sbc_rx_mas.md
// =============================================================================
// 功能概述: SBC 接收器（per-link 唯一）：sb_rxd_i 串行解串 16 bit 报文（negedge 采样 + dp 偶校验）
//   + toggle 邮箱 CDC 至 sb_tx_clk 域 + 双路分发（dispatcher 直通 + 软件观测缓冲）
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-09-13  cdc-resolver  CDC 违例修复（MAS v1.6）：soft_rst_i/manual_read_i 改 toggle 载体，入口 2FF 同步 + 同步输出打拍（t2^t2_d）边沿恢复 sb_tx_clk 1 拍脉冲，全部使用点改接同步后信号
//  2026-08-07  OpenHappy Team  初始版本：基于 sbc_rx MAS v1.3 生成
// =============================================================================

`timescale 1ns/1ps

module sbc_rx #(
    parameter PKT_WIDTH = 16,
    parameter BUF_DEPTH = 8
) (
    // 时钟与复位
    input  wire                 sb_rx_clk,  // 边带接收时钟（对端随路 source-synchronous，最高 500 MHz）
    input  wire                 sb_tx_clk,  // 边带工作时钟（分发域；sideband 顶 ports.sb_tx_clk_in）
    input  wire                 sb_rst_n,   // 边带域异步复位，低有效（sb_tx_clk_in 域，CRG 已同步）
    input  wire                 sb_rx_rst_n,// 边带接收随路复位（sb_rx_clk 负沿域，CRG 已同步）

    // 串行输入
    input  wire                 sb_rxd_i,   // 边带接收串行数据（negedge 采样，LSB first；sb_rxr_sel=1 时被隔离忽略）
    input  wire                 sb_rxr_i,   // 边带接收数据冗余（sb_rxr_sel=1 时进解调主路，intf v10.3 P1）
    input  wire                 sb_rxr_sel_i, // 0x828 SBC_REDUNDANCY[1]（经 sideband，apb_pclk 域 quasi-static）

    // 配置接口（CDC 在 apb_csr_top 侧完成，到达本模块已为 sb_tx_clk 域）
    input  wire                 enable_i,       // 接收使能，高有效（sb_rx_clk 域内再同步）
    input  wire                 manual_read_i,  // 软件读弹出脉冲（1 cycle）
    input  wire                 soft_rst_i,     // 软复位脉冲（1 cycle）
    input  wire [7:0]           buf_depth_i,    // 软件观测缓冲有效深度（1~8，0/>8 钳位 8）

    // 直通分发（sbc_dispatcher，valid-only）
    output reg  [PKT_WIDTH-1:0] pkt_o,
    output reg                  pkt_valid_o,        // 1 cycle 脉冲

    // 错误事件（sbc_dispatcher，分类别 1 cycle 脉冲）
    output wire                 parity_err_event_o,
    output wire                 gap_err_event_o,

    // 状态回流（level，CDC 在 apb_csr_top 侧完成）
    output wire [3:0]           buf_level_o,    // 软件观测缓冲水位 0~8
    output wire                 busy_o,         // 接收忙
    output wire                 parity_err_o,   // 偶校验错 sticky level
    output wire                 gap_err_o,      // 报文间隔错 sticky level
    output wire                 buf_ovf_o,      // 软件观测缓冲溢出 sticky level
    output reg  [PKT_WIDTH-1:0] buf_rdata_o     // 软件观测缓冲头报文（空时输出 0）
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    localparam PTR_WIDTH = 3;   // log2(BUF_DEPTH)

    // FSM 状态编码（binary，见 sbc_rx_recv_fsm.json）
    localparam ST_IDLE     = 2'b00;
    localparam ST_SHIFT    = 2'b01;
    localparam ST_GAP_WAIT = 2'b10;

    // 位宽对齐常量
    localparam [3:0]         BIT_CNT_MAX       = 4'd15;    // 第 16 个 negedge（采样 D15=dp）
    localparam [7:0]         BUF_DEPTH_U8      = BUF_DEPTH[7:0];        // 8'd8
    localparam [PTR_WIDTH:0] BUF_DEPTH_EFF_MAX = BUF_DEPTH[PTR_WIDTH:0]; // 钳位上限 8
    localparam [4:0]         RST_GUARD_CYCLES  = 5'd16;    // soft_rst 后屏蔽窗（2 倍裕量）

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================

    // --------------------------------------------------------------------------
    // sb_rx_clk 域：解串 FSM / 移位采样 / 校验 / 邮箱发送侧
    // --------------------------------------------------------------------------
    reg  [1:0]              cur_state;
    reg  [1:0]              next_state;
    reg  [PKT_WIDTH-1:0]    shift_reg;          // 解串移位寄存器（16 次采样后 [k]=Dk）
    reg  [3:0]              bit_cnt;            // 采样计数 1→15
    reg  [PKT_WIDTH-1:0]    pkt_hold_reg;       // 邮箱报文保持寄存器
    reg                     req_toggle;         // 邮箱请求 toggle（发送侧）
    reg                     parity_err_toggle;  // 校验错事件 toggle
    reg                     gap_err_toggle;     // 间隔错事件 toggle
    reg                     fsm_active_rx;      // 解串 FSM 活动标志（寄sb_rx_clk 域）
    wire [PKT_WIDTH-1:0]    pkt16;              // 第 16 个 negedge 当沿拼接的完整报文
    wire                    parity_ok;          // 偶校验通过（16 bit 全 XOR == 0）
    wire                    mailbox_free;       // 邮箱空（req_toggle == ack_sync）

    // --------------------------------------------------------------------------
    // CDC 同步（sb_tx_clk → sb_rx_clk 方向）
    // --------------------------------------------------------------------------
    wire                    enable_rx;          // enable_i 同步至 sb_rx_clk 域
    wire                    ack_sync;           // ack_toggle 回同步至 sb_rx_clk 域
    wire                    srst_sync2;         // soft_rst toggle 同步至 sb_rx_clk 域
    reg                     srst_sync2_d;       // 边沿检测打拍
    wire                    soft_rst_rx;        // soft_rst 转发脉冲（sb_rx_clk 域）

    // --------------------------------------------------------------------------
    // CDC 同步（sb_rx_clk → sb_tx_clk 方向）
    // --------------------------------------------------------------------------
    wire                    req_sync2;          // req_toggle 同步至 sb_tx_clk 域
    wire [PKT_WIDTH-1:0]    pkt_hold_sync;      // 报文 per-bit 2flop 同步
    wire                    perr_sync2;         // parity_err_toggle 同步
    wire                    gerr_sync2;         // gap_err_toggle 同步
    wire                    fsm_active_sync;    // FSM 活动标志同步至 sb_tx_clk 域
    reg                     req_sync2_d;        // 边沿检测打拍
    reg                     perr_sync2_d;
    reg                     gerr_sync2_d;
    wire                    req_edge;
    wire                    parity_err_edge;
    wire                    gap_err_edge;
    wire                    req_accept;         // req_edge 经 rst_guard 屏蔽后有效
    wire                    parity_err_event;   // 事件脉冲（经 rst_guard 屏蔽）
    wire                    gap_err_event;

    // --------------------------------------------------------------------------
    // sb_tx_clk 域：邮箱接收侧 / 软件观测缓冲 / sticky 标志
    // --------------------------------------------------------------------------
    reg                     ack_toggle;         // 邮箱应答 toggle（接收侧）
    reg                     soft_rst_toggle;    // soft_rst 转发 toggle
    reg  [4:0]              rst_guard_cnt;      // 屏蔽窗计数（16 cycle）
    wire                    rst_guard_active;
    reg  [PKT_WIDTH-1:0]    buf_mem [0:BUF_DEPTH-1];  // 软件观测缓冲寄存器堆
    reg  [PTR_WIDTH:0]      wptr;
    reg  [PTR_WIDTH:0]      rptr;
    wire [PTR_WIDTH:0]      buf_level;
    reg  [PTR_WIDTH:0]      buf_depth_eff;      // buf_depth_i 钳位后有效深度
    wire                    buf_push;           // copy 入队
    wire                    buf_pop;            // manual_read 弹出（空则忽略）
    wire                    buf_ovf_set;        // copy 丢弃（缓冲满）
    reg                     parity_err_flag;    // sticky 标志组
    reg                     gap_err_flag;
    reg                     buf_ovf_flag;
    wire                    mbx_pending;        // 邮箱待取（req 已同步未 ack）

    // --------------------------------------------------------------------------
    // CDC 同步（pclk 写脉冲 → toggle → sb_tx_clk 2FF → 同步输出打拍边沿检测归 1 拍脉冲）
    //   soft_rst_i / manual_read_i 端口语义已由 “pclk 1 拍脉冲” 改为 “toggle 请求”
    //   （apb_csr_top 侧由 *_wp 转 toggle），此处 2FF 同步 + 同步输出打拍 XOR 恢复为
    //   sb_tx_clk 域 1 拍脉冲；soft_rst_t1/manual_read_t1 仅驱动 t2（纯 2FF 同步链），
    //   边沿检测在同步输出侧，消除亚稳态与窄脉冲漏采（cdc-resolver 2026-09-13）。
    //   注：pclk 1GHz > sb_tx_clk 800MHz，单拍脉冲不能直接 2FF 打拍，必须用 toggle。
    // --------------------------------------------------------------------------
    reg                     soft_rst_t1, soft_rst_t2, soft_rst_t2_d;
    reg                     manual_read_t1, manual_read_t2, manual_read_t2_d;
    wire                    soft_rst_i_ckd     = soft_rst_t2     ^ soft_rst_t2_d;
    wire                    manual_read_i_ckd  = manual_read_t2  ^ manual_read_t2_d;
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            soft_rst_t1     <= 1'b0;
            soft_rst_t2     <= 1'b0;
            soft_rst_t2_d   <= 1'b0;
            manual_read_t1  <= 1'b0;
            manual_read_t2  <= 1'b0;
            manual_read_t2_d<= 1'b0;
        end
        else begin
            soft_rst_t1     <= soft_rst_i;
            soft_rst_t2     <= soft_rst_t1;
            soft_rst_t2_d   <= soft_rst_t2;
            manual_read_t1  <= manual_read_i;
            manual_read_t2  <= manual_read_t1;
            manual_read_t2_d<= manual_read_t2;
        end
    end

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // sb_rx_clk 域：第 16 个 negedge 当沿完成处理用组合信号
    // pkt16 = {sb_rxd_i, shift_reg[15:1]}（当沿采样值拼接前 15 bit 移位结果）
    // --------------------------------------------------------------------------
    // SBC 冗余选路（intf v10.3 P1）：sb_rxr_sel=1 → rxr 进解调主路、rxd 忽略（仅边带空闲期可切换）
    wire                        sb_rxd_sel = sb_rxr_sel_i ? sb_rxr_i : sb_rxd_i;

    assign pkt16        = {sb_rxd_sel, shift_reg[PKT_WIDTH-1:1]};
    assign parity_ok    = (^pkt16 == 1'b0);
    assign mailbox_free = (req_toggle == ack_sync);

    // soft_rst 转发边沿检测（sb_rx_clk 域）
    assign soft_rst_rx = srst_sync2 ^ srst_sync2_d;

    // --------------------------------------------------------------------------
    // sb_tx_clk 域：邮箱 req / 错误事件边沿检测 + rst_guard 屏蔽
    // 注：soft_rst 跨域清零存在数 cycle 时差，toggle 由 1 清 0 会在同步链上产生
    // 伪边沿——req 与 parity/gap 事件边沿统一经 rst_guard 屏蔽窗过滤（MAS §5.3）
    // --------------------------------------------------------------------------
    assign req_edge        = req_sync2  ^ req_sync2_d;
    assign parity_err_edge = perr_sync2 ^ perr_sync2_d;
    assign gap_err_edge    = gerr_sync2 ^ gerr_sync2_d;

    assign req_accept        = req_edge        & ~rst_guard_active;
    assign parity_err_event  = parity_err_edge & ~rst_guard_active;
    assign gap_err_event     = gap_err_edge    & ~rst_guard_active;

    assign rst_guard_active = (rst_guard_cnt != 5'd0);

    // --------------------------------------------------------------------------
    // 软件观测缓冲：水位 / 有效深度钳位 / 入队弹出
    // --------------------------------------------------------------------------
    assign buf_level   = wptr - rptr;
    assign buf_push    = req_accept & (buf_level < buf_depth_eff);
    assign buf_pop     = manual_read_i_ckd & (buf_level != {PTR_WIDTH+1{1'b0}});
    assign buf_ovf_set = req_accept & (buf_level >= buf_depth_eff);

    // buf_depth_i 钳位：0 或 >8 钳位为 8（防呆；软件契约 enable_i=0 时修改）
    always @(*) begin
        if ((buf_depth_i == 8'd0) || (buf_depth_i > BUF_DEPTH_U8)) begin
            buf_depth_eff = BUF_DEPTH_EFF_MAX;
        end
        else begin
            buf_depth_eff = buf_depth_i[PTR_WIDTH:0];
        end
    end

    // 缓冲头组合呈现（空时输出 0，防软件读到随机值）
    always @(*) begin
        if (buf_level != {PTR_WIDTH+1{1'b0}}) begin
            buf_rdata_o = buf_mem[rptr[PTR_WIDTH-1:0]];
        end
        else begin
            buf_rdata_o = {PKT_WIDTH{1'b0}};
        end
    end

    // --------------------------------------------------------------------------
    // 状态输出
    // --------------------------------------------------------------------------
    assign mbx_pending        = (req_sync2 != ack_toggle);
    assign busy_o             = fsm_active_sync | mbx_pending
                              | (buf_level != {PTR_WIDTH+1{1'b0}});
    assign buf_level_o        = buf_level;
    assign parity_err_event_o = parity_err_event;
    assign gap_err_event_o    = gap_err_event;
    assign parity_err_o       = parity_err_flag;
    assign gap_err_o          = gap_err_flag;
    assign buf_ovf_o          = buf_ovf_flag;

    // --------------------------------------------------------------------------
    // FSM 下一状态逻辑（sb_rx_clk 域；协议保证报文间隙时钟停低，
    // IDLE / GAP_WAIT 下首个 negedge 即报文 D0 采样点，沿到达即报文开始）
    // --------------------------------------------------------------------------
    always @(*) begin
        next_state = ST_IDLE;
        case (cur_state)
            ST_IDLE: begin
                if (enable_rx) begin
                    next_state = ST_SHIFT;
                end
                else begin
                    next_state = ST_IDLE;
                end
            end
            ST_SHIFT: begin
                if (!enable_rx) begin
                    next_state = ST_IDLE;       // disable 中止（已收部分丢弃）
                end
                else if (bit_cnt == BIT_CNT_MAX) begin
                    next_state = ST_GAP_WAIT;   // 16 bit 收齐
                end
                else begin
                    next_state = ST_SHIFT;
                end
            end
            ST_GAP_WAIT: begin
                if (!enable_rx) begin
                    next_state = ST_IDLE;       // disable 回空闲
                end
                else begin
                    next_state = ST_SHIFT;      // 首个 negedge = 次报文 D0
                end
            end
            default: begin
                next_state = ST_IDLE;           // 非法状态自恢复
            end
        endcase
    end

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // FSM 状态寄存器（negedge sb_rx_clk；转移优先级：复位 > soft_rst 转发 >
    // enable=0 > 正常流转）
    // --------------------------------------------------------------------------
    always @(negedge sb_rx_clk or negedge sb_rx_rst_n) begin
        if (!sb_rx_rst_n) begin
            cur_state <= ST_IDLE;
        end
        else if (soft_rst_rx) begin
            cur_state <= ST_IDLE;
        end
        else begin
            cur_state <= next_state;
        end
    end

    // --------------------------------------------------------------------------
    // 解串数据通路 + 邮箱发送侧（negedge sb_rx_clk）
    // - IDLE / GAP_WAIT：采样 D0 入 shift_reg[15]，bit_cnt 置 1
    // - SHIFT：每 negedge {sb_rxd_i, shift_reg[15:1]} 填入，bit_cnt 自增
    // - 第 16 个 negedge（bit_cnt==15，采样 D15=dp）当沿完成处理：
    //   校验错 → parity_err_toggle 翻转（丢包）；邮箱忙 → gap_err_toggle 翻转
    //   （丢包）；否则装载 pkt_hold_reg 并翻转 req_toggle 发起 CDC 请求
    // - soft_rst_rx：清 FSM 计数 / 邮箱 req_toggle / err toggle（MAS §5.3）
    // - enable_rx=0：中止采样（邮箱 / 标志保持不清）
    // --------------------------------------------------------------------------
    always @(negedge sb_rx_clk or negedge sb_rx_rst_n) begin
        if (!sb_rx_rst_n) begin
            shift_reg         <= {PKT_WIDTH{1'b0}};
            bit_cnt           <= 4'd0;
            pkt_hold_reg      <= {PKT_WIDTH{1'b0}};
            req_toggle        <= 1'b0;
            parity_err_toggle <= 1'b0;
            gap_err_toggle    <= 1'b0;
        end
        else if (soft_rst_rx) begin
            shift_reg         <= {PKT_WIDTH{1'b0}};
            bit_cnt           <= 4'd0;
            req_toggle        <= 1'b0;
            parity_err_toggle <= 1'b0;
            gap_err_toggle    <= 1'b0;
        end
        else if (!enable_rx) begin
            bit_cnt           <= 4'd0;
        end
        else begin
            case (cur_state)
                ST_IDLE,
                ST_GAP_WAIT: begin
                    shift_reg <= {sb_rxd_sel, {PKT_WIDTH-1{1'b0}}};
                    bit_cnt   <= 4'd1;
                end
                ST_SHIFT: begin
                    shift_reg <= pkt16;
                    if (bit_cnt == BIT_CNT_MAX) begin
                        bit_cnt <= 4'd0;
                        if (!parity_ok) begin
                            // 偶校验错：丢包 + 事件（不进邮箱）
                            parity_err_toggle <= ~parity_err_toggle;
                        end
                        else if (!mailbox_free) begin
                            // 邮箱 overrun：报文间隔过近违例，丢包 + 事件
                            gap_err_toggle <= ~gap_err_toggle;
                        end
                        else begin
                            // 校验通过且邮箱空：装载邮箱并发起 CDC 请求
                            pkt_hold_reg <= pkt16;
                            req_toggle   <= ~req_toggle;
                        end
                    end
                    else begin
                        bit_cnt <= bit_cnt + 4'd1;
                    end
                end
                default: begin
                    bit_cnt <= 4'd0;
                end
            endcase
        end
    end

    // --------------------------------------------------------------------------
    // FSM 活动标志（sb_rx_clk 域寄存器输出，供 u_fsm_active_sync 同步）
    // --------------------------------------------------------------------------
    always @(negedge sb_rx_clk or negedge sb_rx_rst_n) begin
        if (!sb_rx_rst_n) begin
            fsm_active_rx <= 1'b0;
        end
        else if (soft_rst_rx || !enable_rx) begin
            fsm_active_rx <= 1'b0;
        end
        else begin
            if (cur_state == ST_SHIFT) begin
                fsm_active_rx <= 1'b1;
            end
            else begin
                fsm_active_rx <= 1'b0;
            end
        end
    end

    // --------------------------------------------------------------------------
    // soft_rst 转发边沿检测打拍（sb_rx_clk 域）
    // --------------------------------------------------------------------------
    always @(negedge sb_rx_clk or negedge sb_rx_rst_n) begin
        if (!sb_rx_rst_n) begin
            srst_sync2_d <= 1'b0;
        end
        else begin
            srst_sync2_d <= srst_sync2;
        end
    end

    // --------------------------------------------------------------------------
    // 边沿检测打拍（sb_tx_clk 域；soft_rst_i_ckd 时清零，配合 rst_guard 吸收伪边沿）
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            req_sync2_d  <= 1'b0;
            perr_sync2_d <= 1'b0;
            gerr_sync2_d <= 1'b0;
        end
        else if (soft_rst_i_ckd) begin
            req_sync2_d  <= 1'b0;
            perr_sync2_d <= 1'b0;
            gerr_sync2_d <= 1'b0;
        end
        else begin
            req_sync2_d  <= req_sync2;
            perr_sync2_d <= perr_sync2;
            gerr_sync2_d <= gerr_sync2;
        end
    end

    // --------------------------------------------------------------------------
    // 邮箱接收侧（sb_tx_clk 域）：req_edge 当拍直通分发 + 回 ack
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            pkt_o       <= {PKT_WIDTH{1'b0}};
            pkt_valid_o <= 1'b0;
            ack_toggle  <= 1'b0;
        end
        else if (soft_rst_i_ckd) begin
            pkt_valid_o <= 1'b0;
            ack_toggle  <= 1'b0;
        end
        else begin
            if (req_accept) begin
                pkt_o       <= pkt_hold_sync;
                pkt_valid_o <= 1'b1;
                ack_toggle  <= req_sync2;
            end
            else begin
                pkt_valid_o <= 1'b0;
            end
        end
    end

    // --------------------------------------------------------------------------
    // 软件观测缓冲寄存器堆写（无复位，数据由指针有效性保护）
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk) begin
        if (buf_push) begin
            buf_mem[wptr[PTR_WIDTH-1:0]] <= pkt_hold_sync;
        end
    end

    // --------------------------------------------------------------------------
    // 写指针（soft_rst_i_ckd flush）
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            wptr <= {PTR_WIDTH+1{1'b0}};
        end
        else if (soft_rst_i_ckd) begin
            wptr <= {PTR_WIDTH+1{1'b0}};
        end
        else if (buf_push) begin
            wptr <= wptr + 1'b1;
        end
    end

    // --------------------------------------------------------------------------
    // 读指针（soft_rst_i_ckd flush；manual_read_i_ckd 空弹忽略，防下溢）
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            rptr <= {PTR_WIDTH+1{1'b0}};
        end
        else if (soft_rst_i_ckd) begin
            rptr <= {PTR_WIDTH+1{1'b0}};
        end
        else if (buf_pop) begin
            rptr <= rptr + 1'b1;
        end
    end

    // --------------------------------------------------------------------------
    // sticky 标志组（事件置位，soft_rst_i_ckd / sb_rst_n 清除）
    // buf_ovf：copy 入队时 level >= buf_depth_eff 置位（copy 丢弃，直通不受影响）
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            parity_err_flag <= 1'b0;
            gap_err_flag    <= 1'b0;
            buf_ovf_flag    <= 1'b0;
        end
        else if (soft_rst_i_ckd) begin
            parity_err_flag <= 1'b0;
            gap_err_flag    <= 1'b0;
            buf_ovf_flag    <= 1'b0;
        end
        else begin
            if (parity_err_event) begin
                parity_err_flag <= 1'b1;
            end
            if (gap_err_event) begin
                gap_err_flag <= 1'b1;
            end
            if (buf_ovf_set) begin
                buf_ovf_flag <= 1'b1;
            end
        end
    end

    // --------------------------------------------------------------------------
    // soft_rst 转发 toggle（sb_tx_clk 域 → u_sync_soft_rst → sb_rx_clk 域）
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            soft_rst_toggle <= 1'b0;
        end
        else if (soft_rst_i_ckd) begin
            soft_rst_toggle <= ~soft_rst_toggle;
        end
    end

    // --------------------------------------------------------------------------
    // rst_guard 屏蔽窗计数（soft_rst_i_ckd 装载 16 cycle，减到 0 停止）
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            rst_guard_cnt <= 5'd0;
        end
        else if (soft_rst_i_ckd) begin
            rst_guard_cnt <= RST_GUARD_CYCLES;
        end
        else if (rst_guard_cnt != 5'd0) begin
            rst_guard_cnt <= rst_guard_cnt - 5'd1;
        end
    end

    // ==========================================================================
    // 子模块实例化
    // ==========================================================================

    // --------------------------------------------------------------------------
    // sb_tx_clk → sb_rx_clk 方向同步器
    // --------------------------------------------------------------------------
    // enable_i 再同步（u_sync_enable）
    sync_cell u_sync_enable (
        .clk_d  (sb_rx_clk),
        .rstn_d (sb_rx_rst_n),
        .data_s (enable_i),
        .data_d (enable_rx)
    );

    // ack_toggle 回同步（u_ack_sync）
    sync_cell u_ack_sync (
        .clk_d  (sb_rx_clk),
        .rstn_d (sb_rx_rst_n),
        .data_s (ack_toggle),
        .data_d (ack_sync)
    );

    // soft_rst toggle 转发同步（u_sync_soft_rst）
    sync_cell u_sync_soft_rst (
        .clk_d  (sb_rx_clk),
        .rstn_d (sb_rx_rst_n),
        .data_s (soft_rst_toggle),
        .data_d (srst_sync2)
    );

    // --------------------------------------------------------------------------
    // sb_rx_clk → sb_tx_clk 方向同步器
    // --------------------------------------------------------------------------
    // req_toggle 同步（u_sync_req）
    sync_cell u_sync_req (
        .clk_d  (sb_tx_clk),
        .rstn_d (sb_rst_n),
        .data_s (req_toggle),
        .data_d (req_sync2)
    );

    // 报文 per-bit 2flop 同步（u_sync_pkt；发送侧保持至 ack 往返 ≈6 cycle，
    // 数据稳定窗口远大于采样点，per-bit 2flop 无 skew 风险）
    genvar gi;
    generate
        for (gi = 0; gi < PKT_WIDTH; gi = gi + 1) begin : gen_sync_pkt
            sync_cell u_sync_pkt (
                .clk_d  (sb_tx_clk),
                .rstn_d (sb_rst_n),
                .data_s (pkt_hold_reg[gi]),
                .data_d (pkt_hold_sync[gi])
            );
        end
    endgenerate

    // parity / gap err toggle 同步（u_sync_err_evt）
    sync_cell u_sync_perr (
        .clk_d  (sb_tx_clk),
        .rstn_d (sb_rst_n),
        .data_s (parity_err_toggle),
        .data_d (perr_sync2)
    );

    sync_cell u_sync_gerr (
        .clk_d  (sb_tx_clk),
        .rstn_d (sb_rst_n),
        .data_s (gap_err_toggle),
        .data_d (gerr_sync2)
    );

    // FSM 活动标志同步（u_fsm_active_sync，busy_o 组合输入）
    sync_cell u_fsm_active_sync (
        .clk_d  (sb_tx_clk),
        .rstn_d (sb_rst_n),
        .data_s (fsm_active_rx),
        .data_d (fsm_active_sync)
    );

endmodule
