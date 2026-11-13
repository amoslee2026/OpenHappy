//========================================================================
// File: p_axi_loop.v
// 来源: design/rtl_v/block/d2d_ctrl/protocol_layer/
//       protocol_paif_loopback.v 同级 axi_loopback.v（复制至 axi_prcl/common/ 后更名）
// author: OpenHappy Team
// Revision:
//
//========================================================================
// Description: AXI 总线层回环（bond 层）。组合逻辑，AXI IN ↔ protocol tx / protocol rx ↔
//   AXI OUT 的旁路/回灌。sel==1: s1(protocol rx) 回灌 m0(protocol tx)，m1 关闭；
//   sel==0 直通：s0→m0、s1→m1。参数化 AW_W(aw)/AR_W(ar)/W_W(w)/R_W(r)/B_W(b) 与 bond
//   总线位宽对齐（AW/AR 分拆，兼容 AXI5 的 `AW_BUS_W/`AR_BUS_W 不同位宽；AXI4 侧 AW/AR
//   等宽时传同一值）。
//   W/R 各带 lite 位通道（s0/s1_w_lite、m0/m1_w_lite 等），随数据同选/直通。
// 修改记录:
//   2026-08-30  OpenHappy Team  模块 axi_loopback → p_axi_loop（更名，无逻辑改动）
//   2026-08-30  OpenHappy Team  增加 W/R lite 位通道（s0/s1/m0/m1 各 *_w_lite/*_r_lite）
//   2026-09-02  OpenHappy Team  AX_W 分拆为 AW_W / AR_W：AW/AR 通道独立位宽参数，兼容 AXI5
//                       bond（`AW_BUS_W != `AR_BUS_W）；AXI4 侧 AW/AR 等宽传同一值
// ========================================================================
module p_axi_loop  #(
    parameter AW_W = 107,
    parameter AR_W = 107,
    parameter W_W  = 149,
    parameter R_W  = 143,
    parameter B_W  = 14
)(
    input  wire                         sel,

    // slave 0 connect AXI IN
    input  wire  [AW_W        -1 : 0]   s0_aw_data,
    input  wire                         s0_aw_valid,
    output wire                         s0_aw_ready,

    input  wire  [AR_W        -1 : 0]   s0_ar_data,
    input  wire                         s0_ar_valid,
    output wire                         s0_ar_ready,

    input  wire  [W_W         -1 : 0]   s0_w_data,
    input  wire                         s0_w_valid,
    output wire                         s0_w_ready,
    input  wire                         s0_w_lite,     // s0 W lite 位

    input  wire  [R_W         -1 : 0]   s0_r_data,
    input  wire                         s0_r_valid,
    output wire                         s0_r_ready,
    input  wire                         s0_r_lite,     // s0 R lite 位

    input  wire  [B_W         -1 : 0]   s0_b_data,
    input  wire                         s0_b_valid,
    output wire                         s0_b_ready,


    // slave 1 connect protocol rx
    input  wire  [AW_W        -1 : 0]   s1_aw_data,
    input  wire                         s1_aw_valid,
    output wire                         s1_aw_ready,

    input  wire  [AR_W        -1 : 0]   s1_ar_data,
    input  wire                         s1_ar_valid,
    output wire                         s1_ar_ready,

    input  wire  [W_W         -1 : 0]   s1_w_data,
    input  wire                         s1_w_valid,
    output wire                         s1_w_ready,
    input  wire                         s1_w_lite,     // s1 W lite 位

    input  wire  [R_W         -1 : 0]   s1_r_data,
    input  wire                         s1_r_valid,
    output wire                         s1_r_ready,
    input  wire                         s1_r_lite,     // s1 R lite 位

    input  wire  [B_W         -1 : 0]   s1_b_data,
    input  wire                         s1_b_valid,
    output wire                         s1_b_ready,

    // master 0 connect protocol tx
    output wire  [AW_W        -1 : 0]   m0_aw_data,
    output wire                         m0_aw_valid,
    input  wire                         m0_aw_ready,

    output wire  [AR_W        -1 : 0]   m0_ar_data,
    output wire                         m0_ar_valid,
    input  wire                         m0_ar_ready,

    output wire  [W_W         -1 : 0]   m0_w_data,
    output wire                         m0_w_valid,
    input  wire                         m0_w_ready,
    output wire                         m0_w_lite,     // m0 W lite 位（→ TX FIFO lite）

    output wire  [R_W         -1 : 0]   m0_r_data,
    output wire                         m0_r_valid,
    input  wire                         m0_r_ready,
    output wire                         m0_r_lite,     // m0 R lite 位（→ TX FIFO lite）

    output wire  [B_W         -1 : 0]   m0_b_data,
    output wire                         m0_b_valid,
    input  wire                         m0_b_ready,

    // master 1 connect AXI OUT
    output wire  [AW_W        -1 : 0]   m1_aw_data,
    output wire                         m1_aw_valid,
    input  wire                         m1_aw_ready,

    output wire  [AR_W        -1 : 0]   m1_ar_data,
    output wire                         m1_ar_valid,
    input  wire                         m1_ar_ready,

    output wire  [W_W         -1 : 0]   m1_w_data,
    output wire                         m1_w_valid,
    input  wire                         m1_w_ready,
    output wire                         m1_w_lite,     // m1 W lite 位（→ AXI OUT）

    output wire  [R_W         -1 : 0]   m1_r_data,
    output wire                         m1_r_valid,
    input  wire                         m1_r_ready,
    output wire                         m1_r_lite,     // m1 R lite 位（→ AXI OUT）

    output wire  [B_W         -1 : 0]   m1_b_data,
    output wire                         m1_b_valid,
    input  wire                         m1_b_ready

);

//====================================
//  AXI IN -> | s0   m0  | -> protocol_tx
//            |          |
//            |          |
//  AXI OUT<- | m1   s1  | <- protocol rx
//
//  sel == 1:   s1 -> m0
//====================================
    wire  [16   -1 : 0] wdata_plus;
    wire  [16   -1 : 0] rdata_plus;

    assign  wdata_plus = s1_w_data[15 : 0] + 16'd1;
    assign  rdata_plus = s1_r_data[15 : 0] + 16'd1;

    assign  m0_aw_data = sel ? s1_aw_data : s0_aw_data;
    assign  m0_ar_data = sel ? s1_ar_data : s0_ar_data;
    assign  m0_w_data  = sel ? {s1_w_data[W_W -1 : 16] , wdata_plus} : s0_w_data;
    assign  m0_r_data  = sel ? {s1_r_data[R_W -1 : 16] , rdata_plus} : s0_r_data;
    assign  m0_b_data  = sel ? s1_b_data  : s0_b_data;

    // W/R lite 位随数据路径同选（回环时取 s1，直通时取 s0）
    assign  m0_w_lite  = sel ? s1_w_lite  : s0_w_lite;
    assign  m0_r_lite  = sel ? s1_r_lite  : s0_r_lite;

    assign  m0_aw_valid = sel ? s1_aw_valid : s0_aw_valid;
    assign  m0_ar_valid = sel ? s1_ar_valid : s0_ar_valid;
    assign  m0_w_valid  = sel ? s1_w_valid  : s0_w_valid;
    assign  m0_r_valid  = sel ? s1_r_valid  : s0_r_valid;
    assign  m0_b_valid  = sel ? s1_b_valid  : s0_b_valid;

    assign  s0_aw_ready = (!sel) & m0_aw_ready;
    assign  s1_aw_ready = sel & m0_aw_ready | ((!sel) & m1_aw_ready);

    assign  s0_ar_ready = (!sel) & m0_ar_ready;
    assign  s1_ar_ready = sel & m0_ar_ready | ((!sel) & m1_ar_ready);

    assign  s0_w_ready  = (!sel) & m0_w_ready;
    assign  s1_w_ready  = sel & m0_w_ready | ((!sel) & m1_w_ready);

    assign  s0_r_ready  = (!sel) & m0_r_ready;
    assign  s1_r_ready  = sel & m0_r_ready | ((!sel) & m1_r_ready);

    assign  s0_b_ready  = (!sel) & m0_b_ready;
    assign  s1_b_ready  = sel & m0_b_ready | ((!sel) & m1_b_ready);

    assign  m1_aw_data = s1_aw_data;
    assign  m1_ar_data = s1_ar_data;
    assign  m1_w_data  = s1_w_data;
    assign  m1_r_data  = s1_r_data;
    assign  m1_b_data  = s1_b_data;

    // m1 W/R lite 直通（同 m1 w/r 数据）
    assign  m1_w_lite  = s1_w_lite;
    assign  m1_r_lite  = s1_r_lite;

    assign  m1_aw_valid = (!sel) & s1_aw_valid;
    assign  m1_ar_valid = (!sel) & s1_ar_valid;
    assign  m1_w_valid  = (!sel) & s1_w_valid;
    assign  m1_r_valid  = (!sel) & s1_r_valid;
    assign  m1_b_valid  = (!sel) & s1_b_valid;
    

endmodule
