// =============================================================================
// module name：p_axi4_rch_pack
// author: OpenHappy Team
// MAS：p_axi4_rch_pack_mas.md
// 功能概述: resp 方向 per-lane PAIF TX flit 封装模块（R 优先 + b_vc 触发 + SP 内嵌 Credit/B）
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   2026-08-06  OpenHappy Team  初始版本
//   2026-08-12  OpenHappy Team  S2 修复：busy 末项 (buf_cnt_r==0)→(buf_cnt_r!=0)（buf 非空=忙，修复语义反转）
//                     S1 修复：buf_cnt_after_out bit[5] 补 |(&buf_cnt_r[4:0])（63→32 正确）
//                     S3 修复：generate 补 PAIF_DW 非 512/1024 兜底 else（防 paif_txdata_w 悬空）
//                     W1/W3/R_WIDTH 决策落地：sp_sel 优先级以 §5.1 伪码为准（wcrd_full > arcrd_full > b_rp），
//                     b_dly 计数为 buf_ready 有效 cycle（RTL 现状），R_WIDTH 含 ruser 且与 bond 已对齐
//                     W2：b_vc Credit 字段注释位宽修正（AW@[80:73]/AR@[88:81]/W@[96:89]）
//                     W5：flit_phase 命名统一 flit_phase_r（与同模块寄存器 _r 后缀一致）
// =============================================================================
// NOTE: R_WIDTH = PAIF_DW+17（含 ruser 4bit）依本 MAS §6；bond 侧 r_tx_bus 同为
//   AXI4_DW+17（p_axi4_bond.v R_WIDTH 参数，已对齐），ruser 已含于位宽。
// =============================================================================
// 字段位布局依 HIPI_AXI4_Next.md 定义实现（SP / B_VC / R_VC / r_lite_bus 排布见文档，
//   其中 RUSER 4bit 已含于 R_WIDTH=PAIF_DW+17）。
// =============================================================================


// NOTE: 依 axi_prcl 目录约定（与 p_axi4_bond.v / p_axis_async_fifo.v /
//   p_axi4_unbond.v / p_crd_gen.v 一致），leaf 子模块不自带 \`include；
//   axi_defines.vh 由顶层 wrapper / filelist 在编译期提供，故本文件
//   直接使用 AXI4_R_FULL_CN / AXI4_R_LITE_CN / AXI4_B_CN 等宏。

module p_axi4_rch_pack #(
    parameter PAIF_DW = 512,         // PAIF 数据位宽（512 或 1024），与 AXI4 数据位宽相同
    parameter [1:0] R_RP = 2'd0      // 本 lane 的 R 通道 RP 编号（实例化时静态固化）
) (
    // 时钟与复位
    input  wire                 clk,
    input  wire                 rstn,

    // 状态
    output wire                 busy,

    // R 通道（R_RP[i] txfifo 出口）
    input  wire [PAIF_DW+17-1:0] r_bus,    // R 通道合并信息（R_WIDTH=PAIF_DW+17）
    input  wire                 r_lite,     // R lite 标志（决定 r_full_vc / r_lite_vc）
    input  wire                 r_valid,
    output wire                 r_ready,

    // B RP0~3（经 b_rp_dmx 路由到本 lane）
    input  wire [15:0]          b_rp0_bus,
    input  wire                 b_rp0_valid,
    output wire                 b_rp0_ready,
    input  wire [15:0]          b_rp1_bus,
    input  wire                 b_rp1_valid,
    output wire                 b_rp1_ready,
    input  wire [15:0]          b_rp2_bus,
    input  wire                 b_rp2_valid,
    output wire                 b_rp2_ready,
    input  wire [15:0]          b_rp3_bus,
    input  wire                 b_rp3_valid,
    output wire                 b_rp3_ready,

    // AW Credit RP0~3（2bit 编码 0/1/4/8）
    input  wire [1:0]           awcrd_rp0,
    output wire                 awcrd_rp0_ready,
    input  wire [1:0]           awcrd_rp1,
    output wire                 awcrd_rp1_ready,
    input  wire [1:0]           awcrd_rp2,
    output wire                 awcrd_rp2_ready,
    input  wire [1:0]           awcrd_rp3,
    output wire                 awcrd_rp3_ready,

    // W Credit RP0~3
    input  wire [1:0]           wcrd_rp0,
    output wire                 wcrd_rp0_ready,
    input  wire [1:0]           wcrd_rp1,
    output wire                 wcrd_rp1_ready,
    input  wire [1:0]           wcrd_rp2,
    output wire                 wcrd_rp2_ready,
    input  wire [1:0]           wcrd_rp3,
    output wire                 wcrd_rp3_ready,

    // AR Credit RP0~3
    input  wire [1:0]           arcrd_rp0,
    output wire                 arcrd_rp0_ready,
    input  wire [1:0]           arcrd_rp1,
    output wire                 arcrd_rp1_ready,
    input  wire [1:0]           arcrd_rp2,
    output wire                 arcrd_rp2_ready,
    input  wire [1:0]           arcrd_rp3,
    output wire                 arcrd_rp3_ready,

    // PAIF TX 接口（per-lane）
    output wire [PAIF_DW-1:0]   paif_txdata,
    output wire                 paif_txvalid,
    input  wire                 paif_txready
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================

    // R 通道合并信号位宽
    localparam R_WIDTH       = PAIF_DW + 17;
    // R VC 位宽（8'd0 + r_bus + R_RP + CN = PAIF_DW+32）
    localparam R_VC_WIDTH    = PAIF_DW + 32;
    // R lite 合并信号位宽（删除 rid/rresp 后重整）
    localparam R_LITE_WIDTH  = PAIF_DW + 5;
    // 移位粒度（32bit DW 对齐粒度）
    localparam GRAN_WIDTH    = (PAIF_DW * 32) / 1024;
    // 每拍 PAIF payload 位宽（去除 0xfb28 填充）
    localparam SHIFT_WIDTH   = PAIF_DW - GRAN_WIDTH;
    // buf_r 位宽：1 R VC + 1 b_vc 跨拍缓存
    localparam BUFFER_WIDTH  = SHIFT_WIDTH + R_VC_WIDTH + 128;
    // R VC / B VC 占用的 GRAN 粒度数
    localparam [6:0] R_VC_CNT = R_VC_WIDTH / GRAN_WIDTH;
    localparam [6:0] B_VC_CNT = 128 / GRAN_WIDTH;
    // 256Byte Flit 拍数（512bit→4 拍，1024bit→2 拍）
    localparam FLT_PHS_WIDTH = 2048 / PAIF_DW;

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================

    // --------------------------------------------------------------------------
    // buf_r / buf_cnt_r / flit_phase_r / b_dly_cnt_r 寄存器
    // --------------------------------------------------------------------------
    reg [BUFFER_WIDTH-1:0]  buf_r;          // 跨拍 payload 移位寄存器（左移入队/右移出队）
    reg [6:0]               buf_cnt_r;      // buf_r 中有效 GRAN 粒度数据计数
    reg [FLT_PHS_WIDTH-1:0] flit_phase_r;     // Flit 拍内相位
    reg [1:0]               b_dly_cnt_r;    // b_rp 等待 sp 字段传输的延迟计数（3 cycle 触发）

    // --------------------------------------------------------------------------
    // B RP 有效向量与选择
    // --------------------------------------------------------------------------
    wire [3:0] b_rpv;               // {b_rp3_valid, b_rp2_valid, b_rp1_valid, b_rp0_valid}
    wire [15:0] b_bus_sel;          // 选中的 b_rpx_bus（用于 sp_type3 / b_vc）
    wire [1:0]  b_rp_sel;           // 选中的 b_rp 编号

    // --------------------------------------------------------------------------
    // Credit 满载/有效向量
    // --------------------------------------------------------------------------
    wire [3:0] arcrd_full;
    wire [3:0] wcrd_full;
    wire [3:0] arcrd_valid;
    wire [3:0] wcrd_valid;

    // --------------------------------------------------------------------------
    // SP 字段封装
    // --------------------------------------------------------------------------
    wire [19:0] sp_type0;           // awcrd + wcrd 封装
    wire [19:0] sp_type2;           // arcrd 封装
    wire [19:0] sp_type3;           // 选中 b_rpx_bus 封装
    wire [2:0]  sp_hpri_sts;
    wire [2:0]  sp_hpri_sel;
    wire [1:0]  sp_lpri_sts;
    wire [1:0]  sp_lpri_sel;
    wire [2:0]  sp_sel;
    wire [19:0] sp_type_hpri;
    wire [19:0] sp_type_lpri;
    wire [19:0] sp;                 // 最终选择出的 SP 字段
    wire [2:0]  sp_send;            // buf_ready & sp_sel（实际发送 SP 的使能）

    // --------------------------------------------------------------------------
    // b_vc / r_lite_bus / R VC
    // --------------------------------------------------------------------------
    wire [127:0]        b_vc;
    wire [R_LITE_WIDTH-1:0] r_lite_bus;
    wire [R_VC_WIDTH-1:0]   r_full_vc;
    wire [R_VC_WIDTH-1:0]   r_lite_vc;
    wire [R_VC_WIDTH-1:0]   r_vc;

    // --------------------------------------------------------------------------
    // b_vc 触发逻辑
    // --------------------------------------------------------------------------
    wire b_vc_valid;
    wire b_rp_over2;
    wire b_dly_set;
    wire b_vc_set;
    wire b_vc_send;

    // --------------------------------------------------------------------------
    // buf 计数 / ready
    // --------------------------------------------------------------------------
    wire        buf_over_paif;
    wire [6:0]  buf_cnt_after_out;
    wire        buf_ready;
    wire [6:0]  vc_in_cnt;

    // --------------------------------------------------------------------------
    // buf 移位数据通路
    // --------------------------------------------------------------------------
    wire [R_VC_WIDTH+128-1:0] vc_in;        // 入队 VC（r_vc + b_vc）
    wire [BUFFER_WIDTH-1:0]   vc_in_ext;    // vc_in 零扩展到 BUFFER_WIDTH
    wire [BUFFER_WIDTH-1:0]   vc_lsft_s0;
    wire [BUFFER_WIDTH-1:0]   vc_lsft_s1;
    wire [BUFFER_WIDTH-1:0]   vc_lsft_s2;
    wire [BUFFER_WIDTH-1:0]   vc_lsft_s3;
    wire [BUFFER_WIDTH-1:0]   vc_in_lsft;   // vc_in 左移 buf_cnt_after_out[4:0] 个 GRAN_WIDTH
    wire [BUFFER_WIDTH-1:0]   buf_rsft;     // buf_r 右移 SHIFT_WIDTH（paif 出队）

    // --------------------------------------------------------------------------
    // PAIF 输出
    // --------------------------------------------------------------------------
    wire [PAIF_DW-1:0] paif_txdata_w;
    wire               paif_txvalid_w;

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // B RP 有效向量与 bus/rp 选择
    //   b_rp_sel 默认 0（无 b_rp 有效时），仅当 |b_rpv 时 sp_send[2] 使能
    // --------------------------------------------------------------------------
    assign b_rpv = {b_rp3_valid, b_rp2_valid, b_rp1_valid, b_rp0_valid};

    assign b_bus_sel = ({16{b_rp0_valid}} & b_rp0_bus) |
                       ({16{b_rp1_valid}} & b_rp1_bus) |
                       ({16{b_rp2_valid}} & b_rp2_bus) |
                       ({16{b_rp3_valid}} & b_rp3_bus);

    assign b_rp_sel = ({2{b_rp1_valid}} & 2'd1) |
                      ({2{b_rp2_valid}} & 2'd2) |
                      ({2{b_rp3_valid}} & 2'd3);

    // --------------------------------------------------------------------------
    // Credit 满载 / 有效向量
    //   full: credit 编码所有 RP 均满（&各 RP 2bit）
    //   valid: credit 编码至少一个 RP 有值（|各 RP 2bit）
    // --------------------------------------------------------------------------
    assign arcrd_full  = {(&arcrd_rp3), (&arcrd_rp2), (&arcrd_rp1), (&arcrd_rp0)};
    assign wcrd_full   = {(&wcrd_rp3),  (&wcrd_rp2),  (&wcrd_rp1),  (&wcrd_rp0)};
    assign arcrd_valid = {(|arcrd_rp3), (|arcrd_rp2), (|arcrd_rp1), (|arcrd_rp0)};
    assign wcrd_valid  = {(|wcrd_rp3),  (|wcrd_rp2),  (|wcrd_rp1),  (|wcrd_rp0)};

    // --------------------------------------------------------------------------
    // SP 字段封装
    //   sp_type3 = {b_bus_sel, b_rp_sel, 2'd3}（分析源 md 明确给出）
    // --------------------------------------------------------------------------
    assign sp_type3 = {b_bus_sel, b_rp_sel, 2'd3};

    // sp_type0（20bit）—— AW/W Credit 封装
    //   依 HIPI_AXI4_Next.md SP AW/W Credit 模式（TYPE=2'd0）：
    //   {TYPE[1:0], AW_CRD_RP0[3:2], AW_CRD_RP1[5:4], AW_CRD_RP2[7:6], AW_CRD_RP3[9:8],
    //    W_CRD_RP0[11:10], W_CRD_RP1[13:12], W_CRD_RP2[15:14], W_CRD_RP3[17:16], Padding[19:18]}
    assign sp_type0 = {2'b0, wcrd_rp3, wcrd_rp2, wcrd_rp1, wcrd_rp0,
                       awcrd_rp3, awcrd_rp2, awcrd_rp1, awcrd_rp0, 2'd0};

    // sp_type2（20bit）—— AR Credit 封装
    //   依 HIPI_AXI4_Next.md SP AR Credit 模式（TYPE=2'd2）：
    //   {TYPE[1:0], AR_CRD_RP0[3:2], AR_CRD_RP1[5:4], AR_CRD_RP2[7:6], AR_CRD_RP3[9:8], Padding[19:10]}
    assign sp_type2 = {10'd0, arcrd_rp3, arcrd_rp2, arcrd_rp1, arcrd_rp0, 2'd2};

    // --------------------------------------------------------------------------
    // sp_sel 优先级选择（MAS §5.1 伪码）
    //   sp_hpri_sts = {|b_rpv, |arcrd_full, |wcrd_full}，sp_hpri_sel 取最低置位比特
    //   sp_lpri_sts = {|arcrd_valid, |wcrd_valid}，sp_lpri_sel 取最低置位比特
    //   b_vc_set | ~r_lite 时 sp_sel=0（SP 字段不传输）
    // NOTE: 设计确认以 §5.1 伪码为准：|b_rpv 置于 sp_hpri_sts[2]（MSB），
    //   "取最低置位比特" 使 bit[0]（wcrd_full）优先级最高，
    //   实际优先级 wcrd_full > arcrd_full > b_rp；MAS §1.1 文字已同步。
    // --------------------------------------------------------------------------
    assign sp_hpri_sts = {|b_rpv, |arcrd_full, |wcrd_full};
    assign sp_hpri_sel = sp_hpri_sts & ((~sp_hpri_sts) + 3'd1);

    assign sp_lpri_sts = {|arcrd_valid, |wcrd_valid};
    assign sp_lpri_sel = sp_lpri_sts & ((~sp_lpri_sts) + 2'd1);

    assign sp_sel = (b_vc_set | (~r_lite)) ? 3'd0 :
                    (|sp_hpri_sel) ? sp_hpri_sel : {1'b0, sp_lpri_sel};

    assign sp_type_hpri = ({20{sp_hpri_sel[2]}} & sp_type3) |
                          ({20{sp_hpri_sel[1]}} & sp_type2) |
                          ({20{sp_hpri_sel[0]}} & sp_type0);

    assign sp_type_lpri = ({20{sp_lpri_sel[1]}} & sp_type2) |
                          ({20{sp_lpri_sel[0]}} & sp_type0);

    assign sp = (b_vc_set | (~r_lite)) ? 20'd0 :
                (|sp_hpri_sel) ? sp_type_hpri : sp_type_lpri;

    assign sp_send = {3{buf_ready}} & sp_sel;

    // --------------------------------------------------------------------------
    // r_lite_bus（删除 r_bus 中 rid/rresp 后重整）
    //   依 HIPI_AXI4_Next.md R_full 排布：r_bus = {RUSER[4], RLAST, RID[10], RRESP[2], RDATA}，
    //   R_lite = {RUSER, RLAST, RDATA}（去除 RID/RRESP），故 r_lite_bus =
    //   {r_bus[R_WIDTH-1:R_WIDTH-5], r_bus[PAIF_DW-1:0]}。RUSER 4bit（见文档）。
    // --------------------------------------------------------------------------
    assign r_lite_bus = {r_bus[R_WIDTH-1:R_WIDTH-5], r_bus[PAIF_DW-1:0]};

    // --------------------------------------------------------------------------
    // R VC 封装
    //   r_full_vc : {8'd0, r_bus, R_RP, AXI4_R_FULL_CN}（sp 字段无效）
    //   r_lite_vc : {sp, r_lite_bus, R_RP, AXI4_R_LITE_CN}（sp 字段内嵌 Credit/B）
    // --------------------------------------------------------------------------
    assign r_full_vc = {8'd0, r_bus, R_RP, `AXI4_R_FULL_CN};
    assign r_lite_vc = {sp, r_lite_bus, R_RP, `AXI4_R_LITE_CN};
    assign r_vc = r_lite ? r_lite_vc : r_full_vc;

    // --------------------------------------------------------------------------
    // b_vc 封装
    //   依 HIPI_AXI4_Next.md B_VC 布局：resp 方向填充 B 数据（B_RPV@[8:5] / B_RP0-3@[24:9]..[72:57]）
    //   + 本地 AW/AR/W Credit（AW_CRD@[80:73]、AR_CRD@[88:81]、W_CRD@[96:89]，各 4 RP × 2bit=8bit），
    //   R/B Credit（@[111:105]）与 Padding 填 0。
    // --------------------------------------------------------------------------
    assign b_vc = {15'd0, 8'd0, 8'd0,
                   wcrd_rp3, wcrd_rp2, wcrd_rp1, wcrd_rp0,
                   arcrd_rp3, arcrd_rp2, arcrd_rp1, arcrd_rp0,
                   awcrd_rp3, awcrd_rp2, awcrd_rp1, awcrd_rp0,
                   b_rp3_bus, b_rp2_bus, b_rp1_bus, b_rp0_bus, b_rpv, `AXI4_B_CN};

    // --------------------------------------------------------------------------
    // b_vc_set 触发逻辑
    //   ①2+ b_rp 等待：b_rp_over2
    //   ②b_rp 延迟 3 cycle 仍无法经 r_lite_vc sp 字段传输：b_dly_set
    //   ③r_valid=0 且 b_vc 有数据：b_vc_valid & ~r_valid
    // --------------------------------------------------------------------------
    assign b_vc_valid = (|b_rpv) | (|arcrd_valid) | (|wcrd_valid);

    assign b_rp_over2 = ((b_rpv[3] | b_rpv[2]) & (b_rpv[1] | b_rpv[0])) |
                        ((b_rpv[3] | b_rpv[1]) & (b_rpv[2] | b_rpv[0]));

    assign b_dly_set = (|b_rpv) & (&b_dly_cnt_r) & r_valid & (~r_lite);

    assign b_vc_set = b_rp_over2 | b_dly_set | (b_vc_valid & (~r_valid));

    assign b_vc_send = b_vc_set & buf_ready;

    // --------------------------------------------------------------------------
    // buf_cnt_after_out / buf_ready
    //   paif_txready 时按 31 个 GRAN_WIDTH 出队，buf_over_paif 屏蔽
    //   buf_ready: 出队后剩余 < 32 GRAN 才允许入队
    // NOTE(S1 修复): 原伪码 {~buf_cnt_r[5], buf_cnt_r[4:0]+1} 在 buf_cnt_r=63（[4:0]=31）时
    //   5bit 加法回绕为 0（应 32）。修复：bit[5] 补 `|(&buf_cnt_r[4:0])`，63→32 正确。
    //   本模块 buf_cnt_r 可达 63（R+B 同拍入队 42），未覆盖 ≥95 场景（wch_pack 需单独核对）。
    // --------------------------------------------------------------------------
    assign buf_over_paif = |buf_cnt_r[6:5];

    assign buf_cnt_after_out = paif_txready ?
        ({1'b0, ((~buf_cnt_r[5]) | (&buf_cnt_r[4:0])) , (buf_cnt_r[4:0] + 5'd1)} & {7{buf_over_paif}}) :
        buf_cnt_r;

    assign buf_ready = ~(|buf_cnt_after_out[6:5]);

    // --------------------------------------------------------------------------
    // R ready（R 优先：r_valid & buf_ready 即占用 buf）
    // --------------------------------------------------------------------------
    assign r_ready = r_valid & buf_ready;

    // --------------------------------------------------------------------------
    // vc_in 入队数据（b_vc_send 时 r_vc 在高位、b_vc 在低位；否则仅 r_vc）
    //   r_ready 屏蔽 r_vc，避免无效 R 入队
    // --------------------------------------------------------------------------
    assign vc_in = b_vc_send ?
        {({R_VC_WIDTH{r_ready}} & r_vc), b_vc} :
        {128'd0, ({R_VC_WIDTH{r_ready}} & r_vc)};

    // vc_in 零扩展到 BUFFER_WIDTH（高位补 SHIFT_WIDTH 个 0，承载最大左移量）
    assign vc_in_ext = {{SHIFT_WIDTH{1'b0}}, vc_in};

    // --------------------------------------------------------------------------
    // vc_in 左移桶形移位器（按 buf_cnt_after_out[4:0] 移位，每级 2^i * GRAN_WIDTH）
    //   参考设计：分析源 md "左移位" 桶形移位器逐级实现
    // --------------------------------------------------------------------------
    assign vc_lsft_s0 = buf_cnt_after_out[0] ?
        {vc_in_ext[BUFFER_WIDTH-1-(1*GRAN_WIDTH):0], {(1*GRAN_WIDTH){1'b0}}} : vc_in_ext;
    assign vc_lsft_s1 = buf_cnt_after_out[1] ?
        {vc_lsft_s0[BUFFER_WIDTH-1-(2*GRAN_WIDTH):0], {(2*GRAN_WIDTH){1'b0}}} : vc_lsft_s0;
    assign vc_lsft_s2 = buf_cnt_after_out[2] ?
        {vc_lsft_s1[BUFFER_WIDTH-1-(4*GRAN_WIDTH):0], {(4*GRAN_WIDTH){1'b0}}} : vc_lsft_s1;
    assign vc_lsft_s3 = buf_cnt_after_out[3] ?
        {vc_lsft_s2[BUFFER_WIDTH-1-(8*GRAN_WIDTH):0], {(8*GRAN_WIDTH){1'b0}}} : vc_lsft_s2;
    assign vc_in_lsft = buf_cnt_after_out[4] ?
        {vc_lsft_s3[BUFFER_WIDTH-1-(16*GRAN_WIDTH):0], {(16*GRAN_WIDTH){1'b0}}} : vc_lsft_s3;

    // buf_r 右移出队（paif_txready 时移出一个 SHIFT_WIDTH payload）
    assign buf_rsft = paif_txready ?
        {{SHIFT_WIDTH{1'b0}}, buf_r[BUFFER_WIDTH-1:SHIFT_WIDTH]} : buf_r;

    // --------------------------------------------------------------------------
    // vc_in_cnt：本拍入队的 GRAN 粒度数
    //   b_vc_send & r_ready : R_VC_CNT + B_VC_CNT
    //   b_vc_send & ~r_ready: B_VC_CNT
    //   ~b_vc_send & r_ready: R_VC_CNT
    //   其余 : 0
    // --------------------------------------------------------------------------
    assign vc_in_cnt = ({7{b_vc_send & r_ready}}    & (R_VC_CNT + B_VC_CNT)) |
                       ({7{b_vc_send & (~r_ready)}} & B_VC_CNT) |
                       ({7{(~b_vc_send) & r_ready}} & R_VC_CNT);

    // --------------------------------------------------------------------------
    // ready 输出
    //   wcrd/awcrd_ready : b_vc_send | sp_send[0]（sp_type0 内嵌 awcrd+wcrd）
    //   arcrd_ready      : b_vc_send | sp_send[1]（sp_type2 内嵌 arcrd）
    //   b_rpx_ready      : b_vc_send 时全部拉高（b_vc 承载所有有效 b_rp）；
    //                      否则 sp_send[2] & b_rp_sel 命中时拉高对应 RP（sp_type3）
    // --------------------------------------------------------------------------
    assign wcrd_rp0_ready  = b_vc_send | sp_send[0];
    assign wcrd_rp1_ready  = b_vc_send | sp_send[0];
    assign wcrd_rp2_ready  = b_vc_send | sp_send[0];
    assign wcrd_rp3_ready  = b_vc_send | sp_send[0];

    assign awcrd_rp0_ready = b_vc_send | sp_send[0];
    assign awcrd_rp1_ready = b_vc_send | sp_send[0];
    assign awcrd_rp2_ready = b_vc_send | sp_send[0];
    assign awcrd_rp3_ready = b_vc_send | sp_send[0];

    assign arcrd_rp0_ready = b_vc_send | sp_send[1];
    assign arcrd_rp1_ready = b_vc_send | sp_send[1];
    assign arcrd_rp2_ready = b_vc_send | sp_send[1];
    assign arcrd_rp3_ready = b_vc_send | sp_send[1];

    assign b_rp0_ready = b_vc_send | (sp_send[2] & (b_rp_sel == 2'd0));
    assign b_rp1_ready = b_vc_send | (sp_send[2] & (b_rp_sel == 2'd1));
    assign b_rp2_ready = b_vc_send | (sp_send[2] & (b_rp_sel == 2'd2));
    assign b_rp3_ready = b_vc_send | (sp_send[2] & (b_rp_sel == 2'd3));

    // --------------------------------------------------------------------------
    // busy（MAS §5.1 修正 S2：buf 非空=buf_cnt_r!=0，原 ==0 语义反转致 txlane_idle 恒 0）
    // --------------------------------------------------------------------------
    assign busy = r_valid | b_rp0_valid | b_rp1_valid | b_rp2_valid | b_rp3_valid |
                  (|awcrd_rp0) | (|awcrd_rp1) | (|awcrd_rp2) | (|awcrd_rp3) |
                  (|arcrd_rp0) | (|arcrd_rp1) | (|arcrd_rp2) | (|arcrd_rp3) |
                  (|wcrd_rp0)  | (|wcrd_rp1)  | (|wcrd_rp2)  | (|wcrd_rp3)  |
                  paif_txvalid_w | (|buf_cnt_r);

    // --------------------------------------------------------------------------
    // paif_txvalid
    // --------------------------------------------------------------------------
    assign paif_txvalid_w = |flit_phase_r;
    assign paif_txvalid = paif_txvalid_w;

    // --------------------------------------------------------------------------
    // paif_txdata（按 PAIF_DW 分支：512 为 4 拍交替填充，1024 为双端填充）
    //   每拍 payload 取 buf_r[SHIFT_WIDTH-1:0]，填充 0xfb28
    // --------------------------------------------------------------------------
    generate
        if (PAIF_DW == 512) begin : gen_paif_512
            assign paif_txdata_w = (flit_phase_r[1] || flit_phase_r[3]) ?
                {16'hfb28, buf_r[SHIFT_WIDTH-1:0]} :
                {buf_r[SHIFT_WIDTH-1:0], 16'hfb28};
        end else if (PAIF_DW == 1024) begin : gen_paif_1024
            assign paif_txdata_w = {16'hfb28, buf_r[SHIFT_WIDTH-1:0], 16'hfb28};
        end
    endgenerate
    assign paif_txdata = paif_txdata_w;

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // buf_r：paif_txready || buf_ready 时更新（入队左移 | 出队右移）
    // --------------------------------------------------------------------------
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            buf_r <= {BUFFER_WIDTH{1'b0}};
        end else if (paif_txready || buf_ready) begin
            buf_r <= vc_in_lsft | buf_rsft;
        end
    end

    // --------------------------------------------------------------------------
    // buf_cnt_r：出队后剩余 + 本拍入队
    // --------------------------------------------------------------------------
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            buf_cnt_r <= 7'd0;
        end else if (paif_txready || buf_ready) begin
            buf_cnt_r <= buf_cnt_after_out + vc_in_cnt;
        end
    end

    // --------------------------------------------------------------------------
    // b_dly_cnt_r：跟踪 b_rp 等待 sp 字段传输的时长
    //   sp_send[2] / b_vc_send / 无 b_rp 等待时清零；否则自增（饱和于 3）
    // --------------------------------------------------------------------------
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            b_dly_cnt_r <= 2'd0;
        end else if (buf_ready) begin
            if (sp_send[2] || b_vc_send || (~(|b_rpv))) begin
                b_dly_cnt_r <= 2'd0;
            end else if (~(&b_dly_cnt_r)) begin
                b_dly_cnt_r <= b_dly_cnt_r + 2'd1;
            end
        end
    end

    // --------------------------------------------------------------------------
    // flit_phase_r：Flit 拍内相位推进
    //   启动条件：(r_ready || b_vc_send || buf_over_paif) 且
    //             (末拍完成且 paif_txready，或当前未在发)
    //   推进条件：paif_txready 且 flit_phase_r 非零 → 左移（逐拍输出）
    // --------------------------------------------------------------------------
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            flit_phase_r <= {FLT_PHS_WIDTH{1'b0}};
        end else if ((r_ready || b_vc_send || buf_over_paif) &&
                     ((flit_phase_r[FLT_PHS_WIDTH-1] && paif_txready) || (~paif_txvalid_w))) begin
            flit_phase_r <= {{(FLT_PHS_WIDTH-1){1'b0}}, 1'b1};
        end else if (paif_txready && (|flit_phase_r)) begin
            flit_phase_r <= {flit_phase_r[FLT_PHS_WIDTH-2:0], 1'b0};
        end
    end

    // ==========================================================================
    // 子模块实例化
    // ==========================================================================
    // 无子模块（纯数据通路 + 寄存器）

    // ==========================================================================
    // 断言与检查（可选）
    // ==========================================================================
    // 暂无（MAS BUGLIST 为空，待 RTL 验证阶段补充）

endmodule
