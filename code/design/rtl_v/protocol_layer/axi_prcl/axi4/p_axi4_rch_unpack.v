// =============================================================================
// module name：p_axi4_rch_unpack
// author: OpenHappy Team
// MAS：p_axi4_rch_unpack_mas.md
// 功能概述: req 方向 per-lane PAIF RX flit 解封装模块（CN 识别 + R/B VC 解析 + SP 字段 Credit 提取）
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   2026-08-06  OpenHappy Team  初始版本
//   2026-08-12  OpenHappy Team  S1 修复：busy 末项 (buf_cnt_r==6'd0)→(|buf_cnt_r)（buf 非空=忙，修复语义反转）；
//                     清理死信号 r_cn_get1、r_bus_comb
// =============================================================================



module p_axi4_rch_unpack #(
    parameter PAIF_DW = 512,
    parameter R_WIDTH = PAIF_DW + 17
) (
    // 时钟与复位
    input  wire                    clk,
    input  wire                    rstn,
    output wire                    busy,

    // R 通道合并信息输出 -> p_axi4_unbond.r_rp[i] (rxfifo 入口)
    output wire [R_WIDTH-1:0]      r_bus,
    output wire                    r_valid,

    // B RP0~3 合并信息输出 -> p_axi4_unbond.b_rp0~3 (经 b_rp_dmx 反向分发)
    output wire [15:0]             b_rp0_bus,
    output wire                    b_rp0_valid,
    output wire [15:0]             b_rp1_bus,
    output wire                    b_rp1_valid,
    output wire [15:0]             b_rp2_bus,
    output wire                    b_rp2_valid,
    output wire [15:0]             b_rp3_bus,
    output wire                    b_rp3_valid,

    // AW Credit RP0~3 输出 -> p_crd_tx_ctrl (OR-merge 后)
    output wire [1:0]              awcrd_rp0,
    output wire                    awcrd_rp0_valid,
    output wire [1:0]              awcrd_rp1,
    output wire                    awcrd_rp1_valid,
    output wire [1:0]              awcrd_rp2,
    output wire                    awcrd_rp2_valid,
    output wire [1:0]              awcrd_rp3,
    output wire                    awcrd_rp3_valid,

    // W Credit RP0~3 输出 -> p_crd_tx_ctrl
    output wire [1:0]              wcrd_rp0,
    output wire                    wcrd_rp0_valid,
    output wire [1:0]              wcrd_rp1,
    output wire                    wcrd_rp1_valid,
    output wire [1:0]              wcrd_rp2,
    output wire                    wcrd_rp2_valid,
    output wire [1:0]              wcrd_rp3,
    output wire                    wcrd_rp3_valid,

    // AR Credit RP0~3 输出 -> p_crd_tx_ctrl
    output wire [1:0]              arcrd_rp0,
    output wire                    arcrd_rp0_valid,
    output wire [1:0]              arcrd_rp1,
    output wire                    arcrd_rp1_valid,
    output wire [1:0]              arcrd_rp2,
    output wire                    arcrd_rp2_valid,
    output wire [1:0]              arcrd_rp3,
    output wire                    arcrd_rp3_valid,

    // PAIF RX 接口 (per-lane)
    input  wire [PAIF_DW-1:0]      paif_rxdata,
    input  wire                    paif_rxvalid,
    output wire                    paif_rxready
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================

    localparam R_VC_WIDTH   = PAIF_DW + 32;          // R VC 位宽
    localparam B_VC_WIDTH   = 128;                   // B VC 位宽
    localparam GRAN_WIDTH   = (PAIF_DW * 32) / 1024; // 移位粒度
    localparam PLD_WIDTH    = PAIF_DW - GRAN_WIDTH;  // 每拍 payload 位宽
    localparam BUFFER_WIDTH = PLD_WIDTH * 2;         // buf_r 位宽 (2 拍 payload 缓存)
    localparam R_LITE_WIDTH = PAIF_DW + 5;           // R_lite 物理通道信息位宽
    localparam [5:0] R_VC_CNT = R_VC_WIDTH / GRAN_WIDTH; // R VC 占用 GRAN 数
    localparam [5:0] B_VC_CNT = B_VC_WIDTH / GRAN_WIDTH; // B VC 占用 GRAN 数

    // r_bus 内部 rid/rresp 比特位置依 HIPI_AXI4_Next.md R_full 排布：
    //   r_bus = {RUSER[4]@[R_WIDTH-1:R_WIDTH-4], RLAST@[R_WIDTH-5],
    //            RID[10]@[AXI4_DW+11:AXI4_DW+2], RRESP[2]@[AXI4_DW+1:AXI4_DW],
    //            RDATA[AXI4_DW]@[AXI4_DW-1:0]}
    //   RUSER 4bit（见文档），R_WIDTH = PAIF_DW + 17。
    localparam RID_LSB   = PAIF_DW + 2;   // rid 起始比特 (r_bus 内)
    localparam RRESP_LSB = PAIF_DW;       // rresp 起始比特 (r_bus 内)

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================

    // --------------------------------------------------------------------------
    // buf_r / buf_cnt_r / paif_phs_r — 移位寄存器与计数
    // --------------------------------------------------------------------------
    reg [BUFFER_WIDTH-1:0] buf_r;     // 缓存跨拍 payload, 按 GRAN_WIDTH 粒度移位拼接
    reg [5:0]              buf_cnt_r; // 当前 buffer 有效数据 GRAN 数
    reg                    paif_phs_r; // PAIF_DW==512 时 2 拍 payload 相位 (仅 512 模式使用)

    // --------------------------------------------------------------------------
    // payload 提取
    // --------------------------------------------------------------------------
    wire [PLD_WIDTH-1:0] payload;

    // --------------------------------------------------------------------------
    // CN 识别
    // --------------------------------------------------------------------------
    wire r_lite_cn_get0, r_lite_cn_get1;
    wire r_full_cn_get0, r_full_cn_get1;
    wire r_cn_get0;
    wire b_cn_get0, b_cn_get1;
    wire padding_get0, padding_get1, padding_get2, padding_get3;
    wire sp_get0, sp_get1;

    // --------------------------------------------------------------------------
    // 处理模式判定
    // --------------------------------------------------------------------------
    wire buf_cnt_ovr_rbvc, buf_cnt_ovr_rvc, buf_cnt_ovr_bvc;
    wire rbvc_proc, rvc_proc, bvc_proc, proc;

    // --------------------------------------------------------------------------
    // 边界与计数
    // --------------------------------------------------------------------------
    wire        pld_1_bound, pld_2_bound;
    wire [5:0]  pld_1_bnd_cnt;
    wire [5:0]  bound_cnt;
    wire [5:0]  vc_cnt;
    wire [5:0]  out_cnt;
    wire [5:0]  buf_cnt_after_out;
    wire        rx_accept;

    // --------------------------------------------------------------------------
    // 移位结果
    // --------------------------------------------------------------------------
    wire [BUFFER_WIDTH-1:0] buf_rsft;
    wire [BUFFER_WIDTH-1:0] payload_lsft;

    // --------------------------------------------------------------------------
    // R 通道提取
    // --------------------------------------------------------------------------
    wire                  r_vc_pos0;
    wire [R_VC_WIDTH-1:0] r_vc;
    wire                  r_is_full, r_is_lite;
    wire                  r_valid_comb;
    wire [R_WIDTH-1:0]    r_bus_w;
    wire [R_LITE_WIDTH-1:0] r_lite_bus_w;
    wire [9:0]            rid_saved_w;
    wire [1:0]            rresp_saved_w;
    wire [R_WIDTH-1:0]    r_bus_lite_recon_w;

    // --------------------------------------------------------------------------
    // B 通道提取
    // --------------------------------------------------------------------------
    wire [B_VC_WIDTH-1:0] b_vc;
    wire [3:0]            b_rpv;
    wire                  bvc_valid;
    wire [15:0]           bvc_b_rp0_bus, bvc_b_rp1_bus, bvc_b_rp2_bus, bvc_b_rp3_bus;
    wire                  bvc_b_rp0_valid, bvc_b_rp1_valid, bvc_b_rp2_valid, bvc_b_rp3_valid;
    wire [1:0]            bvc_awcrd_rp0, bvc_awcrd_rp1, bvc_awcrd_rp2, bvc_awcrd_rp3;
    wire [1:0]            bvc_wcrd_rp0,  bvc_wcrd_rp1,  bvc_wcrd_rp2,  bvc_wcrd_rp3;
    wire [1:0]            bvc_arcrd_rp0, bvc_arcrd_rp1, bvc_arcrd_rp2, bvc_arcrd_rp3;
    wire                  bvc_awcrd_rp0_valid, bvc_awcrd_rp1_valid, bvc_awcrd_rp2_valid, bvc_awcrd_rp3_valid;
    wire                  bvc_wcrd_rp0_valid,  bvc_wcrd_rp1_valid,  bvc_wcrd_rp2_valid,  bvc_wcrd_rp3_valid;
    wire                  bvc_arcrd_rp0_valid, bvc_arcrd_rp1_valid, bvc_arcrd_rp2_valid, bvc_arcrd_rp3_valid;

    // --------------------------------------------------------------------------
    // SP 字段提取 (r_lite_vc 时 sp 有效)
    // --------------------------------------------------------------------------
    wire [19:0]           sp_field;
    wire [1:0]            sp_type;
    wire                  sp_type0, sp_type2, sp_type3;
    wire                  sp_valid;
    wire [1:0]            sp_b_rp_sel;
    wire [15:0]           sp_b_bus;
    wire [1:0]            sp_awcrd_rp0, sp_awcrd_rp1, sp_awcrd_rp2, sp_awcrd_rp3;
    wire [1:0]            sp_wcrd_rp0,  sp_wcrd_rp1,  sp_wcrd_rp2,  sp_wcrd_rp3;
    wire [1:0]            sp_arcrd_rp0, sp_arcrd_rp1, sp_arcrd_rp2, sp_arcrd_rp3;
    wire                  sp_awcrd_rp0_valid, sp_awcrd_rp1_valid, sp_awcrd_rp2_valid, sp_awcrd_rp3_valid;
    wire                  sp_wcrd_rp0_valid,  sp_wcrd_rp1_valid,  sp_wcrd_rp2_valid,  sp_wcrd_rp3_valid;
    wire                  sp_arcrd_rp0_valid, sp_arcrd_rp1_valid, sp_arcrd_rp2_valid, sp_arcrd_rp3_valid;
    wire                  sp_b_rp0_valid, sp_b_rp1_valid, sp_b_rp2_valid, sp_b_rp3_valid;
    wire [15:0]           sp_b_rp0_bus,  sp_b_rp1_bus,  sp_b_rp2_bus,  sp_b_rp3_bus;

    // --------------------------------------------------------------------------
    // OR-merge 输出 (sp 与 b_vc 不会同时有效)
    // --------------------------------------------------------------------------
    wire [15:0]           b_rp0_bus_comb, b_rp1_bus_comb, b_rp2_bus_comb, b_rp3_bus_comb;
    wire                  b_rp0_valid_comb, b_rp1_valid_comb, b_rp2_valid_comb, b_rp3_valid_comb;
    wire [1:0]            awcrd_rp0_comb, awcrd_rp1_comb, awcrd_rp2_comb, awcrd_rp3_comb;
    wire                  awcrd_rp0_valid_comb, awcrd_rp1_valid_comb, awcrd_rp2_valid_comb, awcrd_rp3_valid_comb;
    wire [1:0]            wcrd_rp0_comb,  wcrd_rp1_comb,  wcrd_rp2_comb,  wcrd_rp3_comb;
    wire                  wcrd_rp0_valid_comb,  wcrd_rp1_valid_comb,  wcrd_rp2_valid_comb,  wcrd_rp3_valid_comb;
    wire [1:0]            arcrd_rp0_comb, arcrd_rp1_comb, arcrd_rp2_comb, arcrd_rp3_comb;
    wire                  arcrd_rp0_valid_comb, arcrd_rp1_valid_comb, arcrd_rp2_valid_comb, arcrd_rp3_valid_comb;

    // --------------------------------------------------------------------------
    // 输出寄存器
    // --------------------------------------------------------------------------
    reg [R_WIDTH-1:0]      r_bus_r;
    reg                    r_valid_r;
    reg [9:0]              rid_saved_r;
    reg [1:0]              rresp_saved_r;
    reg [15:0]             b_rp0_bus_r, b_rp1_bus_r, b_rp2_bus_r, b_rp3_bus_r;
    reg                    b_rp0_valid_r, b_rp1_valid_r, b_rp2_valid_r, b_rp3_valid_r;
    reg [1:0]              awcrd_rp0_r, awcrd_rp1_r, awcrd_rp2_r, awcrd_rp3_r;
    reg                    awcrd_rp0_valid_r, awcrd_rp1_valid_r, awcrd_rp2_valid_r, awcrd_rp3_valid_r;
    reg [1:0]              wcrd_rp0_r,  wcrd_rp1_r,  wcrd_rp2_r,  wcrd_rp3_r;
    reg                    wcrd_rp0_valid_r,  wcrd_rp1_valid_r,  wcrd_rp2_valid_r,  wcrd_rp3_valid_r;
    reg [1:0]              arcrd_rp0_r, arcrd_rp1_r, arcrd_rp2_r, arcrd_rp3_r;
    reg                    arcrd_rp0_valid_r, arcrd_rp1_valid_r, arcrd_rp2_valid_r, arcrd_rp3_valid_r;

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // payload 提取 (PAIF_DW==512: 2 拍相位; PAIF_DW==1024: 单拍中间 payload)
    // --------------------------------------------------------------------------
    generate
        if (PAIF_DW == 512) begin : g_p512
            assign payload = paif_phs_r ? paif_rxdata[0 +: PLD_WIDTH]
                                        : paif_rxdata[16 +: PLD_WIDTH];
        end else begin : g_p1024
            // PAIF_DW==1024: 跳过低 16bit (0xfb28 padding), 取中间 PLD_WIDTH payload
            assign payload = paif_rxdata[16 +: PLD_WIDTH];
        end
    endgenerate

    // --------------------------------------------------------------------------
    // CN 识别 (5bit CN 字段位于各 VC 起始处)
    //   位置 0: buf_r[4:0]
    //   位置 1 (B 后接 R): buf_r[B_VC_WIDTH +: 5]
    //   位置 1 (R 后接 B): buf_r[R_VC_WIDTH +: 5]
    //   位置 2 (rbvc 后 padding): buf_r[(R_VC_WIDTH+B_VC_WIDTH) +: 5]
    // --------------------------------------------------------------------------
    // NOTE: MAS §5.1 伪码中 padding_get[3]/sp_get[1] 使用 (R_VC_CNT+B_VC_CNT) 作
    //       比特索引, 与 VC 字段实际比特位置 (R_VC_WIDTH+B_VC_WIDTH) 不一致, 视为
    //       伪码笔误。此处按 VC bit-layout (HIPI_AXI4_Next.md) 修正为比特位置。
    assign r_lite_cn_get0 = (buf_r[4:0]               == `AXI4_R_LITE_CN);
    assign r_lite_cn_get1 = (buf_r[B_VC_WIDTH +: 5]   == `AXI4_R_LITE_CN);
    assign r_full_cn_get0 = (buf_r[4:0]               == `AXI4_R_FULL_CN);
    assign r_full_cn_get1 = (buf_r[B_VC_WIDTH +: 5]   == `AXI4_R_FULL_CN);
    assign r_cn_get0      = r_lite_cn_get0 | r_full_cn_get0;

    assign b_cn_get0      = (buf_r[4:0] == `AXI4_B_CN) && (buf_cnt_r >= B_VC_CNT);
    assign b_cn_get1      = (buf_r[R_VC_WIDTH +: 5] == `AXI4_B_CN) &&
                            (buf_cnt_r >= (R_VC_CNT + B_VC_CNT));

    assign padding_get0   = (buf_r[4:0]                          == 5'd0);
    assign padding_get1   = (buf_r[B_VC_WIDTH +: 5]              == 5'd0);
    assign padding_get2   = (buf_r[R_VC_WIDTH +: 5]              == 5'd0);
    assign padding_get3   = (buf_r[(R_VC_WIDTH+B_VC_WIDTH) +: 5] == 5'd0);

    // sp 字段位于 r_vc 顶端 20bit; sp_get 标识对应 r_vc 位置 sp 是否非零
    assign sp_get0        = (buf_r[R_VC_WIDTH-1 -: 20]                   != 20'd0);
    assign sp_get1        = (buf_r[(R_VC_WIDTH+B_VC_WIDTH)-1 -: 20]      != 20'd0);

    // --------------------------------------------------------------------------
    // 处理模式判定
    //   rbvc_proc: R+B 同拍 (b+R_lite(sp=0) 或 b+R_full)
    //   rvc_proc : R 单独
    //   bvc_proc : B 单独
    // --------------------------------------------------------------------------
    assign buf_cnt_ovr_rbvc = (buf_cnt_r >= (R_VC_CNT + B_VC_CNT));
    assign buf_cnt_ovr_rvc  = (buf_cnt_r >= R_VC_CNT);
    assign buf_cnt_ovr_bvc  = (buf_cnt_r >= B_VC_CNT);

    assign rbvc_proc = buf_cnt_ovr_rbvc &&
        ((b_cn_get0 && (r_full_cn_get1 || (r_lite_cn_get1 && !sp_get1))) ||
         (b_cn_get1 && (r_full_cn_get0 || (r_lite_cn_get0 && !sp_get0))));
    assign rvc_proc  = buf_cnt_ovr_rvc && r_cn_get0;
    assign bvc_proc  = buf_cnt_ovr_bvc && b_cn_get0;
    assign proc      = bvc_proc | rvc_proc | rbvc_proc;

    // --------------------------------------------------------------------------
    // 边界与计数
    //   pld_1_bound: padding 紧随单 vc (B 或头 padding) 时按 payload 边界移位
    //   pld_2_bound: padding 紧随 rbvc/rvc 时按 payload 边界移位
    // --------------------------------------------------------------------------
    assign pld_1_bound   = padding_get0 | (bvc_proc && padding_get1);
    assign pld_2_bound   = (rbvc_proc && padding_get3) | (rvc_proc && padding_get2);
    assign pld_1_bnd_cnt = buf_cnt_r[5] ? {1'b0, (buf_cnt_r[4:0] + 5'd1)}
                                        : buf_cnt_r;
    assign bound_cnt     = ({6{pld_2_bound}} & buf_cnt_r) |
                           ({6{pld_1_bound}} & pld_1_bnd_cnt);
    assign vc_cnt        = ({6{rbvc_proc}}                     & (R_VC_CNT + B_VC_CNT)) |
                           ({6{(!rbvc_proc) && rvc_proc}}      & R_VC_CNT) |
                           ({6{(!rbvc_proc) && (!rvc_proc) && bvc_proc}} & B_VC_CNT);
    assign out_cnt       = (pld_1_bound | pld_2_bound) ? bound_cnt : vc_cnt;
    assign buf_cnt_after_out = buf_cnt_r - out_cnt;

    // --------------------------------------------------------------------------
    // paif_rxready: buf_cnt_after_out[5]=1 表示溢出, 反压上游
    // --------------------------------------------------------------------------
    assign paif_rxready = ~buf_cnt_after_out[5];
    assign rx_accept    = paif_rxready & paif_rxvalid;

    // --------------------------------------------------------------------------
    // buf_r 右移 out_cnt 个 GRAN_WIDTH (6 级桶形移位, 粒度 1/2/4/8/16/32 GRAN)
    // --------------------------------------------------------------------------
    wire [BUFFER_WIDTH-1:0] buf_rsft_s0;
    wire [BUFFER_WIDTH-1:0] buf_rsft_s1;
    wire [BUFFER_WIDTH-1:0] buf_rsft_s2;
    wire [BUFFER_WIDTH-1:0] buf_rsft_s3;
    wire [BUFFER_WIDTH-1:0] buf_rsft_s4;
    wire [BUFFER_WIDTH-1:0] buf_rsft_s5;

    assign buf_rsft_s0 = out_cnt[0] ? {{(1*GRAN_WIDTH){1'b0}},  buf_r[BUFFER_WIDTH-1 : (1*GRAN_WIDTH)]}  : buf_r;
    assign buf_rsft_s1 = out_cnt[1] ? {{(2*GRAN_WIDTH){1'b0}},  buf_rsft_s0[BUFFER_WIDTH-1 : (2*GRAN_WIDTH)]} : buf_rsft_s0;
    assign buf_rsft_s2 = out_cnt[2] ? {{(4*GRAN_WIDTH){1'b0}},  buf_rsft_s1[BUFFER_WIDTH-1 : (4*GRAN_WIDTH)]} : buf_rsft_s1;
    assign buf_rsft_s3 = out_cnt[3] ? {{(8*GRAN_WIDTH){1'b0}},  buf_rsft_s2[BUFFER_WIDTH-1 : (8*GRAN_WIDTH)]} : buf_rsft_s2;
    assign buf_rsft_s4 = out_cnt[4] ? {{(16*GRAN_WIDTH){1'b0}}, buf_rsft_s3[BUFFER_WIDTH-1 : (16*GRAN_WIDTH)]} : buf_rsft_s3;
    assign buf_rsft_s5 = out_cnt[5] ? {{(32*GRAN_WIDTH){1'b0}}, buf_rsft_s4[BUFFER_WIDTH-1 : (32*GRAN_WIDTH)]} : buf_rsft_s4;
    assign buf_rsft    = buf_rsft_s5;

    // --------------------------------------------------------------------------
    // payload 左移 buf_cnt_after_out[4:0] 个 GRAN_WIDTH (5 级桶形移位, 拼入 buf_r)
    // --------------------------------------------------------------------------
    wire [BUFFER_WIDTH-1:0] pld_ext;
    wire [BUFFER_WIDTH-1:0] pld_lsft_s0;
    wire [BUFFER_WIDTH-1:0] pld_lsft_s1;
    wire [BUFFER_WIDTH-1:0] pld_lsft_s2;
    wire [BUFFER_WIDTH-1:0] pld_lsft_s3;
    wire [BUFFER_WIDTH-1:0] pld_lsft_s4;
    wire [4:0]              pld_shift;

    assign pld_ext    = {{(BUFFER_WIDTH-PLD_WIDTH){1'b0}}, payload};
    assign pld_shift  = buf_cnt_after_out[4:0];
    assign pld_lsft_s0 = pld_shift[0] ? {pld_ext[BUFFER_WIDTH-1-(1*GRAN_WIDTH) : 0],  {(1*GRAN_WIDTH){1'b0}}}  : pld_ext;
    assign pld_lsft_s1 = pld_shift[1] ? {pld_lsft_s0[BUFFER_WIDTH-1-(2*GRAN_WIDTH) : 0], {(2*GRAN_WIDTH){1'b0}}} : pld_lsft_s0;
    assign pld_lsft_s2 = pld_shift[2] ? {pld_lsft_s1[BUFFER_WIDTH-1-(4*GRAN_WIDTH) : 0], {(4*GRAN_WIDTH){1'b0}}} : pld_lsft_s1;
    assign pld_lsft_s3 = pld_shift[3] ? {pld_lsft_s2[BUFFER_WIDTH-1-(8*GRAN_WIDTH) : 0], {(8*GRAN_WIDTH){1'b0}}} : pld_lsft_s2;
    assign pld_lsft_s4 = pld_shift[4] ? {pld_lsft_s3[BUFFER_WIDTH-1-(16*GRAN_WIDTH) : 0],{(16*GRAN_WIDTH){1'b0}}} : pld_lsft_s3;
    assign payload_lsft = pld_lsft_s4;

    // --------------------------------------------------------------------------
    // R 通道提取
    //   r_vc 位置 0 (r 在前) 或位置 B_VC_WIDTH (b 在前)
    // --------------------------------------------------------------------------
    assign r_vc_pos0  = r_full_cn_get0 | r_lite_cn_get0;
    assign r_vc       = r_vc_pos0 ? buf_r[0 +: R_VC_WIDTH]
                                  : buf_r[B_VC_WIDTH +: R_VC_WIDTH];
    assign r_is_full  = r_vc_pos0 ? r_full_cn_get0 : r_full_cn_get1;
    assign r_is_lite  = r_vc_pos0 ? r_lite_cn_get0 : r_lite_cn_get1;
    assign r_valid_comb = rvc_proc | rbvc_proc;

    // R_full: r_bus = r_vc[R_WIDTH+6:7] (跳过 CN[4:0] + RP[6:5])
    assign r_bus_w      = r_vc[7 +: R_WIDTH];
    // R_lite: r_lite_bus = r_vc[R_LITE_WIDTH+6:7]
    assign r_lite_bus_w = r_vc[7 +: R_LITE_WIDTH];

    // rid/rresp 提取位置依 HIPI_AXI4_Next.md r_bus 排布（见 localparam 注释）
    assign rid_saved_w   = r_bus_w[(RID_LSB+10)-1 : RID_LSB];
    assign rresp_saved_w = r_bus_w[(RRESP_LSB+2)-1 : RRESP_LSB];

    // r_lite → r_bus 重构：将保存的 rid/rresp 插回真实位置
    //   r_bus = {ruser[4], rlast[1], rid_saved[10], rresp_saved[2], rdata[AXI4_DW]}
    //   r_lite_bus = {ruser[4], rlast[1], rdata[PAIF_DW]}（去除了 rid/rresp）
    assign r_bus_lite_recon_w = {r_lite_bus_w[R_LITE_WIDTH-1 : R_LITE_WIDTH-4],
                                 r_lite_bus_w[R_LITE_WIDTH-5],
                                 rid_saved_r,
                                 rresp_saved_r,
                                 r_lite_bus_w[PAIF_DW-1:0]};


    // --------------------------------------------------------------------------
    // B 通道提取
    //   b_vc 位置 0 (b 在前) 或位置 R_VC_WIDTH (r 在前)
    // --------------------------------------------------------------------------
    assign b_vc     = b_cn_get0 ? buf_r[0 +: B_VC_WIDTH]
                                : buf_r[R_VC_WIDTH +: B_VC_WIDTH];
    assign bvc_valid = bvc_proc | rbvc_proc;

    // B_VC 字段布局 (HIPI_AXI4_Next.md): B_RPV[8:5], B_RP0[24:9], ..., B_RP3[72:57]
    assign b_rpv = b_vc[8:5];

    assign bvc_b_rp0_bus   = b_vc[24 +: 16];
    assign bvc_b_rp1_bus   = b_vc[40 +: 16];
    assign bvc_b_rp2_bus   = b_vc[56 +: 16];
    assign bvc_b_rp3_bus   = b_vc[72 +: 16];
    assign bvc_b_rp0_valid = bvc_valid & b_rpv[0];
    assign bvc_b_rp1_valid = bvc_valid & b_rpv[1];
    assign bvc_b_rp2_valid = bvc_valid & b_rpv[2];
    assign bvc_b_rp3_valid = bvc_valid & b_rpv[3];

    // B_VC 中 AW/W/AR Credit 字段 (R/B Credit 属于 resp 方向, req 方向不输出)
    assign bvc_awcrd_rp0 = b_vc[73 +: 2];
    assign bvc_awcrd_rp1 = b_vc[75 +: 2];
    assign bvc_awcrd_rp2 = b_vc[77 +: 2];
    assign bvc_awcrd_rp3 = b_vc[79 +: 2];
    assign bvc_wcrd_rp0  = b_vc[89 +: 2];
    assign bvc_wcrd_rp1  = b_vc[91 +: 2];
    assign bvc_wcrd_rp2  = b_vc[93 +: 2];
    assign bvc_wcrd_rp3  = b_vc[95 +: 2];
    assign bvc_arcrd_rp0 = b_vc[81 +: 2];
    assign bvc_arcrd_rp1 = b_vc[83 +: 2];
    assign bvc_arcrd_rp2 = b_vc[85 +: 2];
    assign bvc_arcrd_rp3 = b_vc[87 +: 2];

    // Credit valid: b_vc 处理时且对应 credit 值非零 (0 表示无 credit)
    assign bvc_awcrd_rp0_valid = bvc_valid & (|bvc_awcrd_rp0);
    assign bvc_awcrd_rp1_valid = bvc_valid & (|bvc_awcrd_rp1);
    assign bvc_awcrd_rp2_valid = bvc_valid & (|bvc_awcrd_rp2);
    assign bvc_awcrd_rp3_valid = bvc_valid & (|bvc_awcrd_rp3);
    assign bvc_wcrd_rp0_valid  = bvc_valid & (|bvc_wcrd_rp0);
    assign bvc_wcrd_rp1_valid  = bvc_valid & (|bvc_wcrd_rp1);
    assign bvc_wcrd_rp2_valid  = bvc_valid & (|bvc_wcrd_rp2);
    assign bvc_wcrd_rp3_valid  = bvc_valid & (|bvc_wcrd_rp3);
    assign bvc_arcrd_rp0_valid = bvc_valid & (|bvc_arcrd_rp0);
    assign bvc_arcrd_rp1_valid = bvc_valid & (|bvc_arcrd_rp1);
    assign bvc_arcrd_rp2_valid = bvc_valid & (|bvc_arcrd_rp2);
    assign bvc_arcrd_rp3_valid = bvc_valid & (|bvc_arcrd_rp3);

    // --------------------------------------------------------------------------
    // SP 字段提取 (仅 r_lite_vc 且 sp 非零时有效; rbvc 模式 sp 必为 0)
    //   sp_field = r_vc[R_VC_WIDTH-1-:20], sp[1:0]=TYPE
    //   type0: AW+W Credit, type2: AR Credit, type3: B 模式
    // --------------------------------------------------------------------------
    assign sp_field = r_vc[R_VC_WIDTH-1 -: 20];
    assign sp_type  = sp_field[1:0];
    assign sp_type0 = (sp_type == 2'd0);
    assign sp_type2 = (sp_type == 2'd2);
    assign sp_type3 = (sp_type == 2'd3);
    assign sp_valid = r_valid_comb & r_is_lite & (|sp_field);

    // sp_type0: AW_Credit_RP0..3 [9:2], W_Credit_RP0..3 [17:10]
    assign sp_awcrd_rp0 = sp_field[3:2];
    assign sp_awcrd_rp1 = sp_field[5:4];
    assign sp_awcrd_rp2 = sp_field[7:6];
    assign sp_awcrd_rp3 = sp_field[9:8];
    assign sp_wcrd_rp0  = sp_field[11:10];
    assign sp_wcrd_rp1  = sp_field[13:12];
    assign sp_wcrd_rp2  = sp_field[15:14];
    assign sp_wcrd_rp3  = sp_field[17:16];

    // sp_type2: AR_Credit_RP0..3 [9:2]
    assign sp_arcrd_rp0 = sp_field[3:2];
    assign sp_arcrd_rp1 = sp_field[5:4];
    assign sp_arcrd_rp2 = sp_field[7:6];
    assign sp_arcrd_rp3 = sp_field[9:8];

    assign sp_awcrd_rp0_valid = sp_valid & sp_type0 & (|sp_awcrd_rp0);
    assign sp_awcrd_rp1_valid = sp_valid & sp_type0 & (|sp_awcrd_rp1);
    assign sp_awcrd_rp2_valid = sp_valid & sp_type0 & (|sp_awcrd_rp2);
    assign sp_awcrd_rp3_valid = sp_valid & sp_type0 & (|sp_awcrd_rp3);
    assign sp_wcrd_rp0_valid  = sp_valid & sp_type0 & (|sp_wcrd_rp0);
    assign sp_wcrd_rp1_valid  = sp_valid & sp_type0 & (|sp_wcrd_rp1);
    assign sp_wcrd_rp2_valid  = sp_valid & sp_type0 & (|sp_wcrd_rp2);
    assign sp_wcrd_rp3_valid  = sp_valid & sp_type0 & (|sp_wcrd_rp3);
    assign sp_arcrd_rp0_valid = sp_valid & sp_type2 & (|sp_arcrd_rp0);
    assign sp_arcrd_rp1_valid = sp_valid & sp_type2 & (|sp_arcrd_rp1);
    assign sp_arcrd_rp2_valid = sp_valid & sp_type2 & (|sp_arcrd_rp2);
    assign sp_arcrd_rp3_valid = sp_valid & sp_type2 & (|sp_arcrd_rp3);

    // sp_type3: RP[3:2], B[19:4] (16bit)
    assign sp_b_rp_sel = sp_field[3:2];
    assign sp_b_bus    = sp_field[19:4];
    assign sp_b_rp0_valid = sp_valid & sp_type3 & (sp_b_rp_sel == 2'd0);
    assign sp_b_rp1_valid = sp_valid & sp_type3 & (sp_b_rp_sel == 2'd1);
    assign sp_b_rp2_valid = sp_valid & sp_type3 & (sp_b_rp_sel == 2'd2);
    assign sp_b_rp3_valid = sp_valid & sp_type3 & (sp_b_rp_sel == 2'd3);
    assign sp_b_rp0_bus   = sp_b_rp0_valid ? sp_b_bus : 16'd0;
    assign sp_b_rp1_bus   = sp_b_rp1_valid ? sp_b_bus : 16'd0;
    assign sp_b_rp2_bus   = sp_b_rp2_valid ? sp_b_bus : 16'd0;
    assign sp_b_rp3_bus   = sp_b_rp3_valid ? sp_b_bus : 16'd0;

    // --------------------------------------------------------------------------
    // OR-merge 输出 (sp 与 b_vc 互斥, 不会同时有效; 无效源输出 0)
    // --------------------------------------------------------------------------
    assign b_rp0_bus_comb   = sp_b_rp0_bus   | bvc_b_rp0_bus;
    assign b_rp1_bus_comb   = sp_b_rp1_bus   | bvc_b_rp1_bus;
    assign b_rp2_bus_comb   = sp_b_rp2_bus   | bvc_b_rp2_bus;
    assign b_rp3_bus_comb   = sp_b_rp3_bus   | bvc_b_rp3_bus;
    assign b_rp0_valid_comb = sp_b_rp0_valid | bvc_b_rp0_valid;
    assign b_rp1_valid_comb = sp_b_rp1_valid | bvc_b_rp1_valid;
    assign b_rp2_valid_comb = sp_b_rp2_valid | bvc_b_rp2_valid;
    assign b_rp3_valid_comb = sp_b_rp3_valid | bvc_b_rp3_valid;

    assign awcrd_rp0_comb        = sp_awcrd_rp0 | bvc_awcrd_rp0;
    assign awcrd_rp1_comb        = sp_awcrd_rp1 | bvc_awcrd_rp1;
    assign awcrd_rp2_comb        = sp_awcrd_rp2 | bvc_awcrd_rp2;
    assign awcrd_rp3_comb        = sp_awcrd_rp3 | bvc_awcrd_rp3;
    assign awcrd_rp0_valid_comb  = sp_awcrd_rp0_valid | bvc_awcrd_rp0_valid;
    assign awcrd_rp1_valid_comb  = sp_awcrd_rp1_valid | bvc_awcrd_rp1_valid;
    assign awcrd_rp2_valid_comb  = sp_awcrd_rp2_valid | bvc_awcrd_rp2_valid;
    assign awcrd_rp3_valid_comb  = sp_awcrd_rp3_valid | bvc_awcrd_rp3_valid;

    assign wcrd_rp0_comb         = sp_wcrd_rp0  | bvc_wcrd_rp0;
    assign wcrd_rp1_comb         = sp_wcrd_rp1  | bvc_wcrd_rp1;
    assign wcrd_rp2_comb         = sp_wcrd_rp2  | bvc_wcrd_rp2;
    assign wcrd_rp3_comb         = sp_wcrd_rp3  | bvc_wcrd_rp3;
    assign wcrd_rp0_valid_comb   = sp_wcrd_rp0_valid  | bvc_wcrd_rp0_valid;
    assign wcrd_rp1_valid_comb   = sp_wcrd_rp1_valid  | bvc_wcrd_rp1_valid;
    assign wcrd_rp2_valid_comb   = sp_wcrd_rp2_valid  | bvc_wcrd_rp2_valid;
    assign wcrd_rp3_valid_comb   = sp_wcrd_rp3_valid  | bvc_wcrd_rp3_valid;

    assign arcrd_rp0_comb        = sp_arcrd_rp0 | bvc_arcrd_rp0;
    assign arcrd_rp1_comb        = sp_arcrd_rp1 | bvc_arcrd_rp1;
    assign arcrd_rp2_comb        = sp_arcrd_rp2 | bvc_arcrd_rp2;
    assign arcrd_rp3_comb        = sp_arcrd_rp3 | bvc_arcrd_rp3;
    assign arcrd_rp0_valid_comb  = sp_arcrd_rp0_valid | bvc_arcrd_rp0_valid;
    assign arcrd_rp1_valid_comb  = sp_arcrd_rp1_valid | bvc_arcrd_rp1_valid;
    assign arcrd_rp2_valid_comb  = sp_arcrd_rp2_valid | bvc_arcrd_rp2_valid;
    assign arcrd_rp3_valid_comb  = sp_arcrd_rp3_valid | bvc_arcrd_rp3_valid;

    // --------------------------------------------------------------------------
    // busy (组合, OR of all valid + paif_rxvalid + buf 非空标志)
    //   NOTE(S1 修复): 原 analysis md 末项为 (buf_cnt_r == 6'd0)（buf 空=忙，语义反转），
    //   改 (|buf_cnt_r)（buf 非空=忙，与 spec rxlane_idle=~busy 一致）
    // --------------------------------------------------------------------------
    assign busy = r_valid_r | b_rp0_valid_r | b_rp1_valid_r | b_rp2_valid_r | b_rp3_valid_r |
                  awcrd_rp0_valid_r | awcrd_rp1_valid_r | awcrd_rp2_valid_r | awcrd_rp3_valid_r |
                  arcrd_rp0_valid_r | arcrd_rp1_valid_r | arcrd_rp2_valid_r | arcrd_rp3_valid_r |
                  wcrd_rp0_valid_r  | wcrd_rp1_valid_r  | wcrd_rp2_valid_r  | wcrd_rp3_valid_r  |
                  paif_rxvalid | (|buf_cnt_r);

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // paif_phs_r (仅 PAIF_DW==512 使用)
    // --------------------------------------------------------------------------
    generate
        if (PAIF_DW == 512) begin : g_p512_ph
            always @(posedge clk or negedge rstn) begin
                if (!rstn) begin
                    paif_phs_r <= 1'b0;
                end else if (rx_accept) begin
                    paif_phs_r <= ~paif_phs_r;
                end
            end
        end
    endgenerate

    // --------------------------------------------------------------------------
    // buf_r / buf_cnt_r 更新
    //   proc 或 rx_accept 时: buf_r 右移 out_cnt GRAN, 拼入 payload (左移)
    //   buf_cnt_r = buf_cnt_after_out + (rx_accept ? 31 : 0)
    // --------------------------------------------------------------------------
    // buf_r 移位寄存器（proc 或 rx_accept 时更新，每寄存器独立 always 块）
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            buf_r <= {BUFFER_WIDTH{1'b0}};
        end else if (proc || rx_accept) begin
            buf_r <= buf_rsft | ({BUFFER_WIDTH{rx_accept}} & payload_lsft);
        end
    end
    // buf_cnt_r 计数寄存器
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            buf_cnt_r <= 6'd0;
        end else if (proc || rx_accept) begin
            buf_cnt_r <= buf_cnt_after_out + ({6{rx_accept}} & 6'd31);
        end
    end

    // --------------------------------------------------------------------------
    // R 通道输出寄存器 + rid/rresp 保存（每寄存器独立 always 块）
    //   r_full: 输出 r_bus, 保存 rid/rresp
    //   r_lite: 用保存的 rid/rresp 恢复完整 r_bus
    // --------------------------------------------------------------------------
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            r_valid_r <= 1'b0;
        end else begin
            r_valid_r <= r_valid_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            r_bus_r <= {R_WIDTH{1'b0}};
        end else begin
            if (r_valid_comb) begin
                if (r_is_full) begin
                    r_bus_r <= r_bus_w;
                end else begin
                    // r_lite: 用上次保存的 rid/rresp 恢复完整 r_bus
                    r_bus_r <= r_bus_lite_recon_w;
                end
            end
            // r_valid_comb==0 时 r_bus_r 保持 (降低翻转, 等效时钟门控)
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            rid_saved_r <= 10'd0;
        end else begin
            if (r_valid_comb && r_is_full)
                rid_saved_r <= rid_saved_w;
            // r_full 未处理时保持
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            rresp_saved_r <= 2'd0;
        end else begin
            if (r_valid_comb && r_is_full)
                rresp_saved_r <= rresp_saved_w;
            // r_full 未处理时保持
        end
    end

    // --------------------------------------------------------------------------
    // B RP0~3 输出寄存器（每寄存器独立 always 块）
    // --------------------------------------------------------------------------
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            b_rp0_bus_r <= 16'd0;
        end else begin
            b_rp0_bus_r <= b_rp0_bus_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            b_rp0_valid_r <= 1'b0;
        end else begin
            b_rp0_valid_r <= b_rp0_valid_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            b_rp1_bus_r <= 16'd0;
        end else begin
            b_rp1_bus_r <= b_rp1_bus_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            b_rp1_valid_r <= 1'b0;
        end else begin
            b_rp1_valid_r <= b_rp1_valid_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            b_rp2_bus_r <= 16'd0;
        end else begin
            b_rp2_bus_r <= b_rp2_bus_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            b_rp2_valid_r <= 1'b0;
        end else begin
            b_rp2_valid_r <= b_rp2_valid_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            b_rp3_bus_r <= 16'd0;
        end else begin
            b_rp3_bus_r <= b_rp3_bus_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            b_rp3_valid_r <= 1'b0;
        end else begin
            b_rp3_valid_r <= b_rp3_valid_comb;
        end
    end

    // --------------------------------------------------------------------------
    // AW Credit RP0~3 输出寄存器（每寄存器独立 always 块）
    // --------------------------------------------------------------------------
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            awcrd_rp0_r <= 2'd0;
        end else begin
            awcrd_rp0_r <= awcrd_rp0_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            awcrd_rp0_valid_r <= 1'b0;
        end else begin
            awcrd_rp0_valid_r <= awcrd_rp0_valid_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            awcrd_rp1_r <= 2'd0;
        end else begin
            awcrd_rp1_r <= awcrd_rp1_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            awcrd_rp1_valid_r <= 1'b0;
        end else begin
            awcrd_rp1_valid_r <= awcrd_rp1_valid_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            awcrd_rp2_r <= 2'd0;
        end else begin
            awcrd_rp2_r <= awcrd_rp2_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            awcrd_rp2_valid_r <= 1'b0;
        end else begin
            awcrd_rp2_valid_r <= awcrd_rp2_valid_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            awcrd_rp3_r <= 2'd0;
        end else begin
            awcrd_rp3_r <= awcrd_rp3_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            awcrd_rp3_valid_r <= 1'b0;
        end else begin
            awcrd_rp3_valid_r <= awcrd_rp3_valid_comb;
        end
    end

    // --------------------------------------------------------------------------
    // W Credit RP0~3 输出寄存器（每寄存器独立 always 块）
    // --------------------------------------------------------------------------
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            wcrd_rp0_r <= 2'd0;
        end else begin
            wcrd_rp0_r <= wcrd_rp0_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            wcrd_rp0_valid_r <= 1'b0;
        end else begin
            wcrd_rp0_valid_r <= wcrd_rp0_valid_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            wcrd_rp1_r <= 2'd0;
        end else begin
            wcrd_rp1_r <= wcrd_rp1_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            wcrd_rp1_valid_r <= 1'b0;
        end else begin
            wcrd_rp1_valid_r <= wcrd_rp1_valid_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            wcrd_rp2_r <= 2'd0;
        end else begin
            wcrd_rp2_r <= wcrd_rp2_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            wcrd_rp2_valid_r <= 1'b0;
        end else begin
            wcrd_rp2_valid_r <= wcrd_rp2_valid_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            wcrd_rp3_r <= 2'd0;
        end else begin
            wcrd_rp3_r <= wcrd_rp3_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            wcrd_rp3_valid_r <= 1'b0;
        end else begin
            wcrd_rp3_valid_r <= wcrd_rp3_valid_comb;
        end
    end

    // --------------------------------------------------------------------------
    // AR Credit RP0~3 输出寄存器（每寄存器独立 always 块）
    // --------------------------------------------------------------------------
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            arcrd_rp0_r <= 2'd0;
        end else begin
            arcrd_rp0_r <= arcrd_rp0_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            arcrd_rp0_valid_r <= 1'b0;
        end else begin
            arcrd_rp0_valid_r <= arcrd_rp0_valid_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            arcrd_rp1_r <= 2'd0;
        end else begin
            arcrd_rp1_r <= arcrd_rp1_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            arcrd_rp1_valid_r <= 1'b0;
        end else begin
            arcrd_rp1_valid_r <= arcrd_rp1_valid_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            arcrd_rp2_r <= 2'd0;
        end else begin
            arcrd_rp2_r <= arcrd_rp2_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            arcrd_rp2_valid_r <= 1'b0;
        end else begin
            arcrd_rp2_valid_r <= arcrd_rp2_valid_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            arcrd_rp3_r <= 2'd0;
        end else begin
            arcrd_rp3_r <= arcrd_rp3_comb;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            arcrd_rp3_valid_r <= 1'b0;
        end else begin
            arcrd_rp3_valid_r <= arcrd_rp3_valid_comb;
        end
    end

    // ==========================================================================
    // 输出驱动
    // ==========================================================================
    assign r_bus          = r_bus_r;
    assign r_valid        = r_valid_r;

    assign b_rp0_bus      = b_rp0_bus_r;
    assign b_rp0_valid    = b_rp0_valid_r;
    assign b_rp1_bus      = b_rp1_bus_r;
    assign b_rp1_valid    = b_rp1_valid_r;
    assign b_rp2_bus      = b_rp2_bus_r;
    assign b_rp2_valid    = b_rp2_valid_r;
    assign b_rp3_bus      = b_rp3_bus_r;
    assign b_rp3_valid    = b_rp3_valid_r;

    assign awcrd_rp0      = awcrd_rp0_r;
    assign awcrd_rp0_valid= awcrd_rp0_valid_r;
    assign awcrd_rp1      = awcrd_rp1_r;
    assign awcrd_rp1_valid= awcrd_rp1_valid_r;
    assign awcrd_rp2      = awcrd_rp2_r;
    assign awcrd_rp2_valid= awcrd_rp2_valid_r;
    assign awcrd_rp3      = awcrd_rp3_r;
    assign awcrd_rp3_valid= awcrd_rp3_valid_r;

    assign wcrd_rp0       = wcrd_rp0_r;
    assign wcrd_rp0_valid = wcrd_rp0_valid_r;
    assign wcrd_rp1       = wcrd_rp1_r;
    assign wcrd_rp1_valid = wcrd_rp1_valid_r;
    assign wcrd_rp2       = wcrd_rp2_r;
    assign wcrd_rp2_valid = wcrd_rp2_valid_r;
    assign wcrd_rp3       = wcrd_rp3_r;
    assign wcrd_rp3_valid = wcrd_rp3_valid_r;

    assign arcrd_rp0      = arcrd_rp0_r;
    assign arcrd_rp0_valid= arcrd_rp0_valid_r;
    assign arcrd_rp1      = arcrd_rp1_r;
    assign arcrd_rp1_valid= arcrd_rp1_valid_r;
    assign arcrd_rp2      = arcrd_rp2_r;
    assign arcrd_rp2_valid= arcrd_rp2_valid_r;
    assign arcrd_rp3      = arcrd_rp3_r;
    assign arcrd_rp3_valid= arcrd_rp3_valid_r;

    // ==========================================================================
    // 断言与检查 (可选, 仿真期生效)
    // ==========================================================================
    // synthesis translate_off
    // sp 与 b_vc 不可同时处理 (设计约束, 违反即协议异常)
    // r_bus 内部 rid/rresp 位置依 HIPI_AXI4_Next.md R_full 排布（见 localparam 注释）
    // synthesis translate_on

endmodule

