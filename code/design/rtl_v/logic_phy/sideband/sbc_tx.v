// =============================================================================
// module name：sbc_tx
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/sideband/mas/sbc_tx_mas.md
// =============================================================================
// 功能概述: SBC 发送器（per-link 唯一）：16 bit 报文并串转换输出（LSB first + dp 偶校验 +
//   8 cycle GAP）+ 随路时钟门控 + 发送缓冲 FIFO + 手动注入通道，单 sb_tx_clk_in 域
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-07  OpenHappy Team  初始版本：基于 sbc_tx MAS v1.2 生成
//  2026-09-16  cdc-resolver  CDC 违例修复（Ac_unsync01/02 336B6/336B7/336B8）：
//                          manual_trigger_i 改 toggle 载体（apb_csr_top 侧
//                          sbc_tx_ctrl_tx_manual_trigger_wp 转 toggle），入口 2FF 同步
//                          + 同步输出打拍（t2^t2_d）边沿恢复 sb_tx_clk_in 1 拍脉冲，
//                          全部使用点改接同步后信号
// =============================================================================

`timescale 1ns/1ps

module sbc_tx #(
    parameter PKT_WIDTH    = 16,
    parameter BUF_DEPTH    = 4,
    parameter SHIFT_CYCLES = 16,
    parameter GAP_CYCLES   = 8
) (
    // 时钟与复位
    input  wire                 sb_tx_clk_in,   // 边带发送工作时钟（常跑时钟源，最高 500 MHz）
    output wire                 sb_tx_clk,      // 边带发送随路时钟（门控输出，间隙停低）
    input  wire                 sb_rst_n,       // 边带 TX 域同步释放复位（logic_phy_crg 产出）

    // 配置接口（CDC 在消费侧完成：manual_trigger 为 toggle 请求，本模块 2FF 同步 + 边沿恢复）
    input  wire                 enable_i,           // 发送使能，高有效
    input  wire                 manual_trigger_i,   // 手动注入触发（toggle 请求，非原始 1 拍脉冲）
    input  wire [PKT_WIDTH-1:0] pkt_inject_i,       // 手动注入报文（quasi-static）

    // 报文输入（sbc_dispatcher，valid-ready 握手）
    input  wire [PKT_WIDTH-1:0] pkt_i,
    input  wire                 pkt_valid_i,
    output wire                 pkt_ready_o,

    // 串行输出（LSB first，IDLE/GAP/ERROR 期间维持低电平）
    output wire                 sb_txd_o,   // SBC 主路（sbc_txr_en=1 时数据隔离保持 0，intf v10.3 P1）
    output wire                 sb_txr_o,   // SBC 数据冗余路（sbc_txr_en=1 时承载 SBC 流）
    input  wire                 sb_txr_en_i, // 0x828 SBC_REDUNDANCY[0]（经 sideband，apb_pclk 域 quasi-static）

    // 状态回流（level，CDC 在 apb_csr_top 侧完成）
    output wire [2:0]           buf_level_o,    // 发送缓冲水位 0~4
    output wire                 busy_o,         // 发送忙（FSM 非 IDLE 或缓冲非空）
    output wire                 err_o           // 发送错误 sticky level
);

    // 内部信号
    reg                         sb_txd_o_int;   // 原串行输出 mux（隔离 mux 之前级，intf v10.3 P1）

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    localparam PTR_WIDTH = 2;   // log2(BUF_DEPTH)

    // FSM 状态编码（binary，见 sbc_tx_send_fsm.json）
    localparam ST_IDLE  = 3'b000;
    localparam ST_LOAD  = 3'b001;
    localparam ST_SHIFT = 3'b010;
    localparam ST_GAP   = 3'b011;
    localparam ST_ERROR = 3'b100;

    // 位宽对齐常量
    localparam [PTR_WIDTH:0] BUF_DEPTH_LVL = BUF_DEPTH[PTR_WIDTH:0];        // 4
    localparam [3:0]         BIT_CNT_MAX   = SHIFT_CYCLES[3:0] - 4'd1;      // 15
    localparam [3:0]         GAP_CNT_MAX   = GAP_CYCLES[3:0] - 4'd1;        // 7

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================

    // --------------------------------------------------------------------------
    // FSM
    // --------------------------------------------------------------------------
    reg  [2:0]              cur_state;
    reg  [2:0]              next_state;

    // --------------------------------------------------------------------------
    // 发送缓冲 FIFO（u_pkt_buf：depth 4 × 16 bit 寄存器堆，enable_i=0 flush）
    // --------------------------------------------------------------------------
    reg  [PKT_WIDTH-1:0]    buf_mem [0:BUF_DEPTH-1];
    reg  [PTR_WIDTH:0]      wptr;
    reg  [PTR_WIDTH:0]      rptr;
    wire [PTR_WIDTH:0]      buf_level;
    wire                    buf_empty;
    wire                    buf_full;
    wire [PKT_WIDTH-1:0]    buf_dout;           // 缓冲头报文（组合可读）
    reg                     buf_wr;
    reg  [PKT_WIDTH-1:0]    buf_din;
    wire                    buf_wr_dispatcher;  // dispatcher 报文写（优先）
    wire                    buf_wr_inject;      // 手动 inject 写（次之）
    wire                    buf_pop;            // LOAD 状态弹缓冲头

    // --------------------------------------------------------------------------
    // 错误检测（u_err_flag）
    // --------------------------------------------------------------------------
    wire                    ovf_detect;         // 满时 push 协议违例（进 ERROR 停机）
    wire                    inject_drop;        // inject 丢弃（err sticky，FSM 不停机）
    reg                     err_flag;

    // --------------------------------------------------------------------------
    // 数据通路（u_shift_reg / u_bit_cnt / u_gap_cnt / u_parity_gen）
    // --------------------------------------------------------------------------
    reg  [PKT_WIDTH-1:0]    shift_reg;
    reg  [3:0]              bit_cnt;
    reg  [3:0]              gap_cnt;
    wire                    dp_bit;             // 偶校验位 dp = ^buf_dout[14:0]

    // --------------------------------------------------------------------------
    // 随路时钟门控（u_clk_gate）
    // --------------------------------------------------------------------------
    wire                    tx_clk_en;

    // --------------------------------------------------------------------------
    // CDC 同步（pclk 写脉冲 → toggle → sb_tx_clk_in 2FF → 同步输出打拍边沿检测归 1 拍脉冲）
    //   manual_trigger_i 端口语义已由 “pclk 1 拍脉冲” 改为 “toggle 请求”（apb_csr_top 侧
    //   sbc_tx_ctrl_tx_manual_trigger_wp 转 toggle），此处 2FF 同步 + 同步输出打拍 XOR 恢复为
    //   sb_tx_clk_in 域 1 拍脉冲；manual_trigger_t1 仅驱动 t2（纯 2FF 同步链），边沿检测在
    //   同步输出侧，消除亚稳态与窄脉冲漏采（cdc-resolver 2026-09-16）。
    //   注：pclk 1GHz > sb_tx_clk_in 最高 500MHz，单拍脉冲不能直接 2FF 打拍，必须用 toggle。
    // --------------------------------------------------------------------------
    reg         manual_trigger_t1, manual_trigger_t2, manual_trigger_t2_d;
    wire        manual_trigger_i_ckd = manual_trigger_t2 ^ manual_trigger_t2_d;
    always @(posedge sb_tx_clk_in or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            manual_trigger_t1   <= 1'b0;
            manual_trigger_t2   <= 1'b0;
            manual_trigger_t2_d <= 1'b0;
        end
        else begin
            manual_trigger_t1   <= manual_trigger_i;
            manual_trigger_t2   <= manual_trigger_t1;
            manual_trigger_t2_d <= manual_trigger_t2;
        end
    end

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================

    // 缓冲水位与空满判定（wptr/rptr 多 1 bit 绕环，level 范围 0~4）
    assign buf_level = wptr - rptr;
    assign buf_empty = (buf_level == {PTR_WIDTH+1{1'b0}});
    assign buf_full  = (buf_level == BUF_DEPTH_LVL);

    // 缓冲头组合呈现（LOAD 期间 dp 计算与 shift_reg 装载用）
    assign buf_dout = buf_mem[rptr[PTR_WIDTH-1:0]];

    // valid-ready 反压：缓冲未满即 ready
    assign pkt_ready_o = ~buf_full;

    // 缓冲写仲裁：dispatcher 报文优先，inject 次之
    assign buf_wr_dispatcher = pkt_valid_i & pkt_ready_o;
    assign buf_wr_inject     = manual_trigger_i_ckd & ~buf_full & ~buf_wr_dispatcher;

    always @(*) begin
        if (buf_wr_dispatcher) begin
            buf_wr  = 1'b1;
            buf_din = pkt_i;
        end
        else if (buf_wr_inject) begin
            buf_wr  = 1'b1;
            buf_din = pkt_inject_i;
        end
        else begin
            buf_wr  = 1'b0;
            buf_din = {PKT_WIDTH{1'b0}};
        end
    end

    // 错误检测拆分（MAS §5.1 M3）：
    // ovf_detect  —— 协议违例类硬错误（valid 在 ready=0 时强推），进 ERROR 停机
    // inject_drop —— 软件误用类（inject 缓冲满 / 与 dispatcher 同 cycle 写冲突），
    //                err sticky + 丢弃 inject，FSM 不停机
    assign ovf_detect  = pkt_valid_i & ~pkt_ready_o;
    assign inject_drop = (manual_trigger_i_ckd & buf_full)
                       | (manual_trigger_i_ckd & buf_wr_dispatcher);

    // LOAD 状态弹缓冲头（buf_dout 组合可读，装载与弹出同沿完成）
    assign buf_pop = (cur_state == ST_LOAD) & ~buf_empty;

    // 偶校验生成：含 dp 的 16 bit 中 1 的个数为偶 → dp 取其余 15 bit XOR 归约
    assign dp_bit = ^buf_dout[PKT_WIDTH-2:0];

    // 随路时钟门控使能：LOAD 期间开（首个输出 posedge = LOAD 末沿，与 D0 跳变对齐），
    // SHIFT 末 cycle（bit_cnt==15）关（末 negedge 后停低），每报文 16 posedge + 16 negedge；
    // tx_clk_en 由 posedge 寄存器组合产生，只在 sb_tx_clk_in 低相位变化，ICG 整形无毛刺
    assign tx_clk_en = (cur_state == ST_LOAD)
                     | ((cur_state == ST_SHIFT) & (bit_cnt != BIT_CNT_MAX));

    // 串行输出 mux（state + shift_reg[0]，Mealy；IDLE/LOAD/GAP/ERROR 维持低电平）
    always @(*) begin
        if (cur_state == ST_SHIFT) begin
            sb_txd_o_int = shift_reg[0];
        end
        else begin
            sb_txd_o_int = 1'b0;
        end
    end

    // SBC 冗余 mux（intf v10.3 P1，芯片级扩展 pad 冗余切换）：
    // sb_txr_en=1 → 冗余路承载 SBC 流、主路 sb_txd 数据隔离（保持 0，随路时钟行为不变）；
    // 软件纪律：仅边带空闲期（无报文在途）可切换
    assign sb_txd_o = sb_txr_en_i ? 1'b0 : sb_txd_o_int;
    assign sb_txr_o = sb_txr_en_i ? sb_txd_o_int : 1'b0;

    // 状态输出
    assign busy_o      = (cur_state != ST_IDLE) | ~buf_empty;
    assign buf_level_o = buf_level;
    assign err_o       = err_flag;

    // --------------------------------------------------------------------------
    // FSM 下一状态逻辑（转移优先级：ovf_detect > enable_i=0 中止 > 正常流转）
    // --------------------------------------------------------------------------
    always @(*) begin
        next_state = ST_IDLE;
        case (cur_state)
            ST_IDLE: begin
                if (ovf_detect) begin
                    next_state = ST_ERROR;
                end
                else if (enable_i && !buf_empty) begin
                    next_state = ST_LOAD;
                end
                else begin
                    next_state = ST_IDLE;
                end
            end
            ST_LOAD: begin
                if (ovf_detect) begin
                    next_state = ST_ERROR;
                end
                else if (!enable_i) begin
                    next_state = ST_IDLE;   // disable 中止（报文丢弃）
                end
                else begin
                    next_state = ST_SHIFT;  // 单 cycle 装载
                end
            end
            ST_SHIFT: begin
                if (ovf_detect) begin
                    next_state = ST_ERROR;
                end
                else if (!enable_i) begin
                    next_state = ST_IDLE;   // disable 中止（报文截断）
                end
                else if (bit_cnt == BIT_CNT_MAX) begin
                    next_state = ST_GAP;    // 16 bit 全部移出
                end
                else begin
                    next_state = ST_SHIFT;
                end
            end
            ST_GAP: begin
                if (ovf_detect) begin
                    next_state = ST_ERROR;
                end
                else if (gap_cnt == GAP_CNT_MAX) begin
                    if (!buf_empty && enable_i) begin
                        next_state = ST_LOAD;   // 背靠背发送
                    end
                    else begin
                        next_state = ST_IDLE;   // 发完回空闲
                    end
                end
                else begin
                    next_state = ST_GAP;
                end
            end
            ST_ERROR: begin
                if (!enable_i) begin
                    next_state = ST_IDLE;   // 软件先 disable 再清错
                end
                else begin
                    next_state = ST_ERROR;
                end
            end
            default: begin
                next_state = ST_IDLE;       // 非法状态自恢复
            end
        endcase
    end

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // FSM 状态寄存器
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk_in or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            cur_state <= ST_IDLE;
        end
        else begin
            cur_state <= next_state;
        end
    end

    // --------------------------------------------------------------------------
    // 发送缓冲寄存器堆写（无复位，数据由指针有效性保护）
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk_in) begin
        if (buf_wr) begin
            buf_mem[wptr[PTR_WIDTH-1:0]] <= buf_din;
        end
    end

    // --------------------------------------------------------------------------
    // 写指针（enable_i=0 时 flush，防 stale 报文）
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk_in or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            wptr <= {PTR_WIDTH+1{1'b0}};
        end
        else if (!enable_i) begin
            wptr <= {PTR_WIDTH+1{1'b0}};
        end
        else if (buf_wr) begin
            wptr <= wptr + 1'b1;
        end
    end

    // --------------------------------------------------------------------------
    // 读指针（enable_i=0 时 flush）
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk_in or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            rptr <= {PTR_WIDTH+1{1'b0}};
        end
        else if (!enable_i) begin
            rptr <= {PTR_WIDTH+1{1'b0}};
        end
        else if (buf_pop) begin
            rptr <= rptr + 1'b1;
        end
    end

    // --------------------------------------------------------------------------
    // 移位寄存器（LOAD 装载 {dp, pkt[14:0]}，SHIFT 每 cycle 向低 bit 移 1 位，
    // bit0 移出 → LSB first）
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk_in or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            shift_reg <= {PKT_WIDTH{1'b0}};
        end
        else if (cur_state == ST_LOAD) begin
            shift_reg <= {dp_bit, buf_dout[PKT_WIDTH-2:0]};
        end
        else if (cur_state == ST_SHIFT) begin
            shift_reg <= {1'b0, shift_reg[PKT_WIDTH-1:1]};
        end
    end

    // --------------------------------------------------------------------------
    // 移位计数器（SHIFT 期间 0→15）
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk_in or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            bit_cnt <= 4'd0;
        end
        else if (!enable_i) begin
            bit_cnt <= 4'd0;
        end
        else if (cur_state == ST_SHIFT) begin
            if (bit_cnt == BIT_CNT_MAX) begin
                bit_cnt <= 4'd0;
            end
            else begin
                bit_cnt <= bit_cnt + 4'd1;
            end
        end
        else begin
            bit_cnt <= 4'd0;
        end
    end

    // --------------------------------------------------------------------------
    // 报文间隔计数器（GAP 期间 0→7，固定 8 cycle 协议下限）
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk_in or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            gap_cnt <= 4'd0;
        end
        else if (!enable_i) begin
            gap_cnt <= 4'd0;
        end
        else if (cur_state == ST_GAP) begin
            if (gap_cnt == GAP_CNT_MAX) begin
                gap_cnt <= 4'd0;
            end
            else begin
                gap_cnt <= gap_cnt + 4'd1;
            end
        end
        else begin
            gap_cnt <= 4'd0;
        end
    end

    // --------------------------------------------------------------------------
    // 错误 sticky 标志（ovf_detect / inject_drop 置位，enable_i=0 清除）
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk_in or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            err_flag <= 1'b0;
        end
        else if (!enable_i) begin
            err_flag <= 1'b0;
        end
        else if (ovf_detect || inject_drop) begin
            err_flag <= 1'b1;
        end
    end

    // ==========================================================================
    // 子模块实例化
    // ==========================================================================

    // --------------------------------------------------------------------------
    // 随路时钟门控单元（u_clk_gate）：工艺库 ICG，低电平透明 latch + AND，
    // 对 sb_tx_clk_in 门控输出 sb_tx_clk，无毛刺；sb_tx_clk 仅驱动输出 pad 方向，
    // 不作为本模块内部任何寄存器时钟
    generic_icg_pos u_clk_gate (
        .CK (sb_tx_clk_in),
        .E  (tx_clk_en),
        .Q  (sb_tx_clk)
    );

endmodule
