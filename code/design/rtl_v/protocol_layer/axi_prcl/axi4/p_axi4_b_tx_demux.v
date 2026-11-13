// =============================================================================
// module name：p_axi4_b_tx_demux
// author: OpenHappy Team
// MAS：p_axi4_b_tx_demux_mas.md
// 功能概述: resp 方向 B 通道与 AR/AW/W Credit 多 lane 路由（4 RP × 4 lane，纯组合）
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   2026-08-06  OpenHappy Team  初始版本
// =============================================================================
// NOTE: B_WIDTH 取 16 与 rch_pack 的 b_rp_bus / bond 的 b_tx_bus 位宽一致（{buser,bresp,bid}）。
// =============================================================================


module p_axi4_b_tx_demux #(
    parameter B_WIDTH   = 16,
    parameter CRD_WIDTH = 2
) (
    // APB 静态配置
    input  wire [1:0]   b_rp0_dmx,
    input  wire [1:0]   b_rp1_dmx,
    input  wire [1:0]   b_rp2_dmx,
    input  wire [1:0]   b_rp3_dmx,
    input  wire [1:0]   arcrd_dmx,
    input  wire [1:0]   awwcrd_dmx,      // AW + W Credit 共用 dmx（aww = AW+W 合并缩写）

    // B RP 输入（4 组：bus + valid + ready）
    input  wire [B_WIDTH-1:0] s_b_rp0_bus,  input wire s_b_rp0_valid, output wire s_b_rp0_ready,
    input  wire [B_WIDTH-1:0] s_b_rp1_bus,  input wire s_b_rp1_valid, output wire s_b_rp1_ready,
    input  wire [B_WIDTH-1:0] s_b_rp2_bus,  input wire s_b_rp2_valid, output wire s_b_rp2_ready,
    input  wire [B_WIDTH-1:0] s_b_rp3_bus,  input wire s_b_rp3_valid, output wire s_b_rp3_ready,

    // AW Credit 输入（4 组）
    input  wire [CRD_WIDTH-1:0] s_awcrd_rp0,  output wire s_awcrd_rp0_ready,
    input  wire [CRD_WIDTH-1:0] s_awcrd_rp1,  output wire s_awcrd_rp1_ready,
    input  wire [CRD_WIDTH-1:0] s_awcrd_rp2,  output wire s_awcrd_rp2_ready,
    input  wire [CRD_WIDTH-1:0] s_awcrd_rp3,  output wire s_awcrd_rp3_ready,

    // W Credit 输入（4 组）
    input  wire [CRD_WIDTH-1:0] s_wcrd_rp0,   output wire s_wcrd_rp0_ready,
    input  wire [CRD_WIDTH-1:0] s_wcrd_rp1,   output wire s_wcrd_rp1_ready,
    input  wire [CRD_WIDTH-1:0] s_wcrd_rp2,   output wire s_wcrd_rp2_ready,
    input  wire [CRD_WIDTH-1:0] s_wcrd_rp3,   output wire s_wcrd_rp3_ready,

    // AR Credit 输入（4 组）
    input  wire [CRD_WIDTH-1:0] s_arcrd_rp0,  output wire s_arcrd_rp0_ready,
    input  wire [CRD_WIDTH-1:0] s_arcrd_rp1,  output wire s_arcrd_rp1_ready,
    input  wire [CRD_WIDTH-1:0] s_arcrd_rp2,  output wire s_arcrd_rp2_ready,
    input  wire [CRD_WIDTH-1:0] s_arcrd_rp3,  output wire s_arcrd_rp3_ready,

    // tx lane0 输出
    output wire [B_WIDTH-1:0] m0_b_rp0_bus, output wire m0_b_rp0_valid, input wire m0_b_rp0_ready,
    output wire [B_WIDTH-1:0] m0_b_rp1_bus, output wire m0_b_rp1_valid, input wire m0_b_rp1_ready,
    output wire [B_WIDTH-1:0] m0_b_rp2_bus, output wire m0_b_rp2_valid, input wire m0_b_rp2_ready,
    output wire [B_WIDTH-1:0] m0_b_rp3_bus, output wire m0_b_rp3_valid, input wire m0_b_rp3_ready,
    output wire [CRD_WIDTH-1:0] m0_awcrd_rp0, input wire m0_awcrd_rp0_ready,
    output wire [CRD_WIDTH-1:0] m0_awcrd_rp1, input wire m0_awcrd_rp1_ready,
    output wire [CRD_WIDTH-1:0] m0_awcrd_rp2, input wire m0_awcrd_rp2_ready,
    output wire [CRD_WIDTH-1:0] m0_awcrd_rp3, input wire m0_awcrd_rp3_ready,
    output wire [CRD_WIDTH-1:0] m0_wcrd_rp0,  input wire m0_wcrd_rp0_ready,
    output wire [CRD_WIDTH-1:0] m0_wcrd_rp1,  input wire m0_wcrd_rp1_ready,
    output wire [CRD_WIDTH-1:0] m0_wcrd_rp2,  input wire m0_wcrd_rp2_ready,
    output wire [CRD_WIDTH-1:0] m0_wcrd_rp3,  input wire m0_wcrd_rp3_ready,
    output wire [CRD_WIDTH-1:0] m0_arcrd_rp0, input wire m0_arcrd_rp0_ready,
    output wire [CRD_WIDTH-1:0] m0_arcrd_rp1, input wire m0_arcrd_rp1_ready,
    output wire [CRD_WIDTH-1:0] m0_arcrd_rp2, input wire m0_arcrd_rp2_ready,
    output wire [CRD_WIDTH-1:0] m0_arcrd_rp3, input wire m0_arcrd_rp3_ready,

    // tx lane1 输出
    output wire [B_WIDTH-1:0] m1_b_rp0_bus, output wire m1_b_rp0_valid, input wire m1_b_rp0_ready,
    output wire [B_WIDTH-1:0] m1_b_rp1_bus, output wire m1_b_rp1_valid, input wire m1_b_rp1_ready,
    output wire [B_WIDTH-1:0] m1_b_rp2_bus, output wire m1_b_rp2_valid, input wire m1_b_rp2_ready,
    output wire [B_WIDTH-1:0] m1_b_rp3_bus, output wire m1_b_rp3_valid, input wire m1_b_rp3_ready,
    output wire [CRD_WIDTH-1:0] m1_awcrd_rp0, input wire m1_awcrd_rp0_ready,
    output wire [CRD_WIDTH-1:0] m1_awcrd_rp1, input wire m1_awcrd_rp1_ready,
    output wire [CRD_WIDTH-1:0] m1_awcrd_rp2, input wire m1_awcrd_rp2_ready,
    output wire [CRD_WIDTH-1:0] m1_awcrd_rp3, input wire m1_awcrd_rp3_ready,
    output wire [CRD_WIDTH-1:0] m1_wcrd_rp0,  input wire m1_wcrd_rp0_ready,
    output wire [CRD_WIDTH-1:0] m1_wcrd_rp1,  input wire m1_wcrd_rp1_ready,
    output wire [CRD_WIDTH-1:0] m1_wcrd_rp2,  input wire m1_wcrd_rp2_ready,
    output wire [CRD_WIDTH-1:0] m1_wcrd_rp3,  input wire m1_wcrd_rp3_ready,
    output wire [CRD_WIDTH-1:0] m1_arcrd_rp0, input wire m1_arcrd_rp0_ready,
    output wire [CRD_WIDTH-1:0] m1_arcrd_rp1, input wire m1_arcrd_rp1_ready,
    output wire [CRD_WIDTH-1:0] m1_arcrd_rp2, input wire m1_arcrd_rp2_ready,
    output wire [CRD_WIDTH-1:0] m1_arcrd_rp3, input wire m1_arcrd_rp3_ready,

    // tx lane2 输出
    output wire [B_WIDTH-1:0] m2_b_rp0_bus, output wire m2_b_rp0_valid, input wire m2_b_rp0_ready,
    output wire [B_WIDTH-1:0] m2_b_rp1_bus, output wire m2_b_rp1_valid, input wire m2_b_rp1_ready,
    output wire [B_WIDTH-1:0] m2_b_rp2_bus, output wire m2_b_rp2_valid, input wire m2_b_rp2_ready,
    output wire [B_WIDTH-1:0] m2_b_rp3_bus, output wire m2_b_rp3_valid, input wire m2_b_rp3_ready,
    output wire [CRD_WIDTH-1:0] m2_awcrd_rp0, input wire m2_awcrd_rp0_ready,
    output wire [CRD_WIDTH-1:0] m2_awcrd_rp1, input wire m2_awcrd_rp1_ready,
    output wire [CRD_WIDTH-1:0] m2_awcrd_rp2, input wire m2_awcrd_rp2_ready,
    output wire [CRD_WIDTH-1:0] m2_awcrd_rp3, input wire m2_awcrd_rp3_ready,
    output wire [CRD_WIDTH-1:0] m2_wcrd_rp0,  input wire m2_wcrd_rp0_ready,
    output wire [CRD_WIDTH-1:0] m2_wcrd_rp1,  input wire m2_wcrd_rp1_ready,
    output wire [CRD_WIDTH-1:0] m2_wcrd_rp2,  input wire m2_wcrd_rp2_ready,
    output wire [CRD_WIDTH-1:0] m2_wcrd_rp3,  input wire m2_wcrd_rp3_ready,
    output wire [CRD_WIDTH-1:0] m2_arcrd_rp0, input wire m2_arcrd_rp0_ready,
    output wire [CRD_WIDTH-1:0] m2_arcrd_rp1, input wire m2_arcrd_rp1_ready,
    output wire [CRD_WIDTH-1:0] m2_arcrd_rp2, input wire m2_arcrd_rp2_ready,
    output wire [CRD_WIDTH-1:0] m2_arcrd_rp3, input wire m2_arcrd_rp3_ready,

    // tx lane3 输出
    output wire [B_WIDTH-1:0] m3_b_rp0_bus, output wire m3_b_rp0_valid, input wire m3_b_rp0_ready,
    output wire [B_WIDTH-1:0] m3_b_rp1_bus, output wire m3_b_rp1_valid, input wire m3_b_rp1_ready,
    output wire [B_WIDTH-1:0] m3_b_rp2_bus, output wire m3_b_rp2_valid, input wire m3_b_rp2_ready,
    output wire [B_WIDTH-1:0] m3_b_rp3_bus, output wire m3_b_rp3_valid, input wire m3_b_rp3_ready,
    output wire [CRD_WIDTH-1:0] m3_awcrd_rp0, input wire m3_awcrd_rp0_ready,
    output wire [CRD_WIDTH-1:0] m3_awcrd_rp1, input wire m3_awcrd_rp1_ready,
    output wire [CRD_WIDTH-1:0] m3_awcrd_rp2, input wire m3_awcrd_rp2_ready,
    output wire [CRD_WIDTH-1:0] m3_awcrd_rp3, input wire m3_awcrd_rp3_ready,
    output wire [CRD_WIDTH-1:0] m3_wcrd_rp0,  input wire m3_wcrd_rp0_ready,
    output wire [CRD_WIDTH-1:0] m3_wcrd_rp1,  input wire m3_wcrd_rp1_ready,
    output wire [CRD_WIDTH-1:0] m3_wcrd_rp2,  input wire m3_wcrd_rp2_ready,
    output wire [CRD_WIDTH-1:0] m3_wcrd_rp3,  input wire m3_wcrd_rp3_ready,
    output wire [CRD_WIDTH-1:0] m3_arcrd_rp0, input wire m3_arcrd_rp0_ready,
    output wire [CRD_WIDTH-1:0] m3_arcrd_rp1, input wire m3_arcrd_rp1_ready,
    output wire [CRD_WIDTH-1:0] m3_arcrd_rp2, input wire m3_arcrd_rp2_ready,
    output wire [CRD_WIDTH-1:0] m3_arcrd_rp3, input wire m3_arcrd_rp3_ready
);

    // ==========================================================================
    // B RP 路由（每 RP 按各自 dmx 选通到对应 lane；bus 与 valid/ready 同路径，统一位掩码风格）
    // ==========================================================================
    // RP0
    assign m0_b_rp0_bus   = {B_WIDTH{(b_rp0_dmx == 2'd0)}} & s_b_rp0_bus;
    assign m1_b_rp0_bus   = {B_WIDTH{(b_rp0_dmx == 2'd1)}} & s_b_rp0_bus;
    assign m2_b_rp0_bus   = {B_WIDTH{(b_rp0_dmx == 2'd2)}} & s_b_rp0_bus;
    assign m3_b_rp0_bus   = {B_WIDTH{(b_rp0_dmx == 2'd3)}} & s_b_rp0_bus;
    assign m0_b_rp0_valid = (b_rp0_dmx == 2'd0) & s_b_rp0_valid;
    assign m1_b_rp0_valid = (b_rp0_dmx == 2'd1) & s_b_rp0_valid;
    assign m2_b_rp0_valid = (b_rp0_dmx == 2'd2) & s_b_rp0_valid;
    assign m3_b_rp0_valid = (b_rp0_dmx == 2'd3) & s_b_rp0_valid;
    assign s_b_rp0_ready  = ((b_rp0_dmx == 2'd0) & m0_b_rp0_ready) |
                            ((b_rp0_dmx == 2'd1) & m1_b_rp0_ready) |
                            ((b_rp0_dmx == 2'd2) & m2_b_rp0_ready) |
                            ((b_rp0_dmx == 2'd3) & m3_b_rp0_ready);

    // RP1
    assign m0_b_rp1_bus   = {B_WIDTH{(b_rp1_dmx == 2'd0)}} & s_b_rp1_bus;
    assign m1_b_rp1_bus   = {B_WIDTH{(b_rp1_dmx == 2'd1)}} & s_b_rp1_bus;
    assign m2_b_rp1_bus   = {B_WIDTH{(b_rp1_dmx == 2'd2)}} & s_b_rp1_bus;
    assign m3_b_rp1_bus   = {B_WIDTH{(b_rp1_dmx == 2'd3)}} & s_b_rp1_bus;
    assign m0_b_rp1_valid = (b_rp1_dmx == 2'd0) & s_b_rp1_valid;
    assign m1_b_rp1_valid = (b_rp1_dmx == 2'd1) & s_b_rp1_valid;
    assign m2_b_rp1_valid = (b_rp1_dmx == 2'd2) & s_b_rp1_valid;
    assign m3_b_rp1_valid = (b_rp1_dmx == 2'd3) & s_b_rp1_valid;
    assign s_b_rp1_ready  = ((b_rp1_dmx == 2'd0) & m0_b_rp1_ready) |
                            ((b_rp1_dmx == 2'd1) & m1_b_rp1_ready) |
                            ((b_rp1_dmx == 2'd2) & m2_b_rp1_ready) |
                            ((b_rp1_dmx == 2'd3) & m3_b_rp1_ready);

    // RP2
    assign m0_b_rp2_bus   = {B_WIDTH{(b_rp2_dmx == 2'd0)}} & s_b_rp2_bus;
    assign m1_b_rp2_bus   = {B_WIDTH{(b_rp2_dmx == 2'd1)}} & s_b_rp2_bus;
    assign m2_b_rp2_bus   = {B_WIDTH{(b_rp2_dmx == 2'd2)}} & s_b_rp2_bus;
    assign m3_b_rp2_bus   = {B_WIDTH{(b_rp2_dmx == 2'd3)}} & s_b_rp2_bus;
    assign m0_b_rp2_valid = (b_rp2_dmx == 2'd0) & s_b_rp2_valid;
    assign m1_b_rp2_valid = (b_rp2_dmx == 2'd1) & s_b_rp2_valid;
    assign m2_b_rp2_valid = (b_rp2_dmx == 2'd2) & s_b_rp2_valid;
    assign m3_b_rp2_valid = (b_rp2_dmx == 2'd3) & s_b_rp2_valid;
    assign s_b_rp2_ready  = ((b_rp2_dmx == 2'd0) & m0_b_rp2_ready) |
                            ((b_rp2_dmx == 2'd1) & m1_b_rp2_ready) |
                            ((b_rp2_dmx == 2'd2) & m2_b_rp2_ready) |
                            ((b_rp2_dmx == 2'd3) & m3_b_rp2_ready);

    // RP3
    assign m0_b_rp3_bus   = {B_WIDTH{(b_rp3_dmx == 2'd0)}} & s_b_rp3_bus;
    assign m1_b_rp3_bus   = {B_WIDTH{(b_rp3_dmx == 2'd1)}} & s_b_rp3_bus;
    assign m2_b_rp3_bus   = {B_WIDTH{(b_rp3_dmx == 2'd2)}} & s_b_rp3_bus;
    assign m3_b_rp3_bus   = {B_WIDTH{(b_rp3_dmx == 2'd3)}} & s_b_rp3_bus;
    assign m0_b_rp3_valid = (b_rp3_dmx == 2'd0) & s_b_rp3_valid;
    assign m1_b_rp3_valid = (b_rp3_dmx == 2'd1) & s_b_rp3_valid;
    assign m2_b_rp3_valid = (b_rp3_dmx == 2'd2) & s_b_rp3_valid;
    assign m3_b_rp3_valid = (b_rp3_dmx == 2'd3) & s_b_rp3_valid;
    assign s_b_rp3_ready  = ((b_rp3_dmx == 2'd0) & m0_b_rp3_ready) |
                            ((b_rp3_dmx == 2'd1) & m1_b_rp3_ready) |
                            ((b_rp3_dmx == 2'd2) & m2_b_rp3_ready) |
                            ((b_rp3_dmx == 2'd3) & m3_b_rp3_ready);

    // ==========================================================================
    // AW Credit 路由（4 RP 共用 awwcrd_dmx）
    // ==========================================================================
    assign m0_awcrd_rp0 = {CRD_WIDTH{(awwcrd_dmx == 2'd0)}} & s_awcrd_rp0;
    assign m1_awcrd_rp0 = {CRD_WIDTH{(awwcrd_dmx == 2'd1)}} & s_awcrd_rp0;
    assign m2_awcrd_rp0 = {CRD_WIDTH{(awwcrd_dmx == 2'd2)}} & s_awcrd_rp0;
    assign m3_awcrd_rp0 = {CRD_WIDTH{(awwcrd_dmx == 2'd3)}} & s_awcrd_rp0;
    assign s_awcrd_rp0_ready = ((awwcrd_dmx == 2'd0) & m0_awcrd_rp0_ready) |
                               ((awwcrd_dmx == 2'd1) & m1_awcrd_rp0_ready) |
                               ((awwcrd_dmx == 2'd2) & m2_awcrd_rp0_ready) |
                               ((awwcrd_dmx == 2'd3) & m3_awcrd_rp0_ready);

    assign m0_awcrd_rp1 = {CRD_WIDTH{(awwcrd_dmx == 2'd0)}} & s_awcrd_rp1;
    assign m1_awcrd_rp1 = {CRD_WIDTH{(awwcrd_dmx == 2'd1)}} & s_awcrd_rp1;
    assign m2_awcrd_rp1 = {CRD_WIDTH{(awwcrd_dmx == 2'd2)}} & s_awcrd_rp1;
    assign m3_awcrd_rp1 = {CRD_WIDTH{(awwcrd_dmx == 2'd3)}} & s_awcrd_rp1;
    assign s_awcrd_rp1_ready = ((awwcrd_dmx == 2'd0) & m0_awcrd_rp1_ready) |
                               ((awwcrd_dmx == 2'd1) & m1_awcrd_rp1_ready) |
                               ((awwcrd_dmx == 2'd2) & m2_awcrd_rp1_ready) |
                               ((awwcrd_dmx == 2'd3) & m3_awcrd_rp1_ready);

    assign m0_awcrd_rp2 = {CRD_WIDTH{(awwcrd_dmx == 2'd0)}} & s_awcrd_rp2;
    assign m1_awcrd_rp2 = {CRD_WIDTH{(awwcrd_dmx == 2'd1)}} & s_awcrd_rp2;
    assign m2_awcrd_rp2 = {CRD_WIDTH{(awwcrd_dmx == 2'd2)}} & s_awcrd_rp2;
    assign m3_awcrd_rp2 = {CRD_WIDTH{(awwcrd_dmx == 2'd3)}} & s_awcrd_rp2;
    assign s_awcrd_rp2_ready = ((awwcrd_dmx == 2'd0) & m0_awcrd_rp2_ready) |
                               ((awwcrd_dmx == 2'd1) & m1_awcrd_rp2_ready) |
                               ((awwcrd_dmx == 2'd2) & m2_awcrd_rp2_ready) |
                               ((awwcrd_dmx == 2'd3) & m3_awcrd_rp2_ready);

    assign m0_awcrd_rp3 = {CRD_WIDTH{(awwcrd_dmx == 2'd0)}} & s_awcrd_rp3;
    assign m1_awcrd_rp3 = {CRD_WIDTH{(awwcrd_dmx == 2'd1)}} & s_awcrd_rp3;
    assign m2_awcrd_rp3 = {CRD_WIDTH{(awwcrd_dmx == 2'd2)}} & s_awcrd_rp3;
    assign m3_awcrd_rp3 = {CRD_WIDTH{(awwcrd_dmx == 2'd3)}} & s_awcrd_rp3;
    assign s_awcrd_rp3_ready = ((awwcrd_dmx == 2'd0) & m0_awcrd_rp3_ready) |
                               ((awwcrd_dmx == 2'd1) & m1_awcrd_rp3_ready) |
                               ((awwcrd_dmx == 2'd2) & m2_awcrd_rp3_ready) |
                               ((awwcrd_dmx == 2'd3) & m3_awcrd_rp3_ready);

    // ==========================================================================
    // W Credit 路由（4 RP 共用 awwcrd_dmx，与 AW crd 同字段）
    // ==========================================================================
    assign m0_wcrd_rp0 = {CRD_WIDTH{(awwcrd_dmx == 2'd0)}} & s_wcrd_rp0;
    assign m1_wcrd_rp0 = {CRD_WIDTH{(awwcrd_dmx == 2'd1)}} & s_wcrd_rp0;
    assign m2_wcrd_rp0 = {CRD_WIDTH{(awwcrd_dmx == 2'd2)}} & s_wcrd_rp0;
    assign m3_wcrd_rp0 = {CRD_WIDTH{(awwcrd_dmx == 2'd3)}} & s_wcrd_rp0;
    assign s_wcrd_rp0_ready = ((awwcrd_dmx == 2'd0) & m0_wcrd_rp0_ready) |
                              ((awwcrd_dmx == 2'd1) & m1_wcrd_rp0_ready) |
                              ((awwcrd_dmx == 2'd2) & m2_wcrd_rp0_ready) |
                              ((awwcrd_dmx == 2'd3) & m3_wcrd_rp0_ready);

    assign m0_wcrd_rp1 = {CRD_WIDTH{(awwcrd_dmx == 2'd0)}} & s_wcrd_rp1;
    assign m1_wcrd_rp1 = {CRD_WIDTH{(awwcrd_dmx == 2'd1)}} & s_wcrd_rp1;
    assign m2_wcrd_rp1 = {CRD_WIDTH{(awwcrd_dmx == 2'd2)}} & s_wcrd_rp1;
    assign m3_wcrd_rp1 = {CRD_WIDTH{(awwcrd_dmx == 2'd3)}} & s_wcrd_rp1;
    assign s_wcrd_rp1_ready = ((awwcrd_dmx == 2'd0) & m0_wcrd_rp1_ready) |
                              ((awwcrd_dmx == 2'd1) & m1_wcrd_rp1_ready) |
                              ((awwcrd_dmx == 2'd2) & m2_wcrd_rp1_ready) |
                              ((awwcrd_dmx == 2'd3) & m3_wcrd_rp1_ready);

    assign m0_wcrd_rp2 = {CRD_WIDTH{(awwcrd_dmx == 2'd0)}} & s_wcrd_rp2;
    assign m1_wcrd_rp2 = {CRD_WIDTH{(awwcrd_dmx == 2'd1)}} & s_wcrd_rp2;
    assign m2_wcrd_rp2 = {CRD_WIDTH{(awwcrd_dmx == 2'd2)}} & s_wcrd_rp2;
    assign m3_wcrd_rp2 = {CRD_WIDTH{(awwcrd_dmx == 2'd3)}} & s_wcrd_rp2;
    assign s_wcrd_rp2_ready = ((awwcrd_dmx == 2'd0) & m0_wcrd_rp2_ready) |
                              ((awwcrd_dmx == 2'd1) & m1_wcrd_rp2_ready) |
                              ((awwcrd_dmx == 2'd2) & m2_wcrd_rp2_ready) |
                              ((awwcrd_dmx == 2'd3) & m3_wcrd_rp2_ready);

    assign m0_wcrd_rp3 = {CRD_WIDTH{(awwcrd_dmx == 2'd0)}} & s_wcrd_rp3;
    assign m1_wcrd_rp3 = {CRD_WIDTH{(awwcrd_dmx == 2'd1)}} & s_wcrd_rp3;
    assign m2_wcrd_rp3 = {CRD_WIDTH{(awwcrd_dmx == 2'd2)}} & s_wcrd_rp3;
    assign m3_wcrd_rp3 = {CRD_WIDTH{(awwcrd_dmx == 2'd3)}} & s_wcrd_rp3;
    assign s_wcrd_rp3_ready = ((awwcrd_dmx == 2'd0) & m0_wcrd_rp3_ready) |
                              ((awwcrd_dmx == 2'd1) & m1_wcrd_rp3_ready) |
                              ((awwcrd_dmx == 2'd2) & m2_wcrd_rp3_ready) |
                              ((awwcrd_dmx == 2'd3) & m3_wcrd_rp3_ready);

    // ==========================================================================
    // AR Credit 路由（4 RP 共用 arcrd_dmx）
    // ==========================================================================
    assign m0_arcrd_rp0 = {CRD_WIDTH{(arcrd_dmx == 2'd0)}} & s_arcrd_rp0;
    assign m1_arcrd_rp0 = {CRD_WIDTH{(arcrd_dmx == 2'd1)}} & s_arcrd_rp0;
    assign m2_arcrd_rp0 = {CRD_WIDTH{(arcrd_dmx == 2'd2)}} & s_arcrd_rp0;
    assign m3_arcrd_rp0 = {CRD_WIDTH{(arcrd_dmx == 2'd3)}} & s_arcrd_rp0;
    assign s_arcrd_rp0_ready = ((arcrd_dmx == 2'd0) & m0_arcrd_rp0_ready) |
                               ((arcrd_dmx == 2'd1) & m1_arcrd_rp0_ready) |
                               ((arcrd_dmx == 2'd2) & m2_arcrd_rp0_ready) |
                               ((arcrd_dmx == 2'd3) & m3_arcrd_rp0_ready);

    assign m0_arcrd_rp1 = {CRD_WIDTH{(arcrd_dmx == 2'd0)}} & s_arcrd_rp1;
    assign m1_arcrd_rp1 = {CRD_WIDTH{(arcrd_dmx == 2'd1)}} & s_arcrd_rp1;
    assign m2_arcrd_rp1 = {CRD_WIDTH{(arcrd_dmx == 2'd2)}} & s_arcrd_rp1;
    assign m3_arcrd_rp1 = {CRD_WIDTH{(arcrd_dmx == 2'd3)}} & s_arcrd_rp1;
    assign s_arcrd_rp1_ready = ((arcrd_dmx == 2'd0) & m0_arcrd_rp1_ready) |
                               ((arcrd_dmx == 2'd1) & m1_arcrd_rp1_ready) |
                               ((arcrd_dmx == 2'd2) & m2_arcrd_rp1_ready) |
                               ((arcrd_dmx == 2'd3) & m3_arcrd_rp1_ready);

    assign m0_arcrd_rp2 = {CRD_WIDTH{(arcrd_dmx == 2'd0)}} & s_arcrd_rp2;
    assign m1_arcrd_rp2 = {CRD_WIDTH{(arcrd_dmx == 2'd1)}} & s_arcrd_rp2;
    assign m2_arcrd_rp2 = {CRD_WIDTH{(arcrd_dmx == 2'd2)}} & s_arcrd_rp2;
    assign m3_arcrd_rp2 = {CRD_WIDTH{(arcrd_dmx == 2'd3)}} & s_arcrd_rp2;
    assign s_arcrd_rp2_ready = ((arcrd_dmx == 2'd0) & m0_arcrd_rp2_ready) |
                               ((arcrd_dmx == 2'd1) & m1_arcrd_rp2_ready) |
                               ((arcrd_dmx == 2'd2) & m2_arcrd_rp2_ready) |
                               ((arcrd_dmx == 2'd3) & m3_arcrd_rp2_ready);

    assign m0_arcrd_rp3 = {CRD_WIDTH{(arcrd_dmx == 2'd0)}} & s_arcrd_rp3;
    assign m1_arcrd_rp3 = {CRD_WIDTH{(arcrd_dmx == 2'd1)}} & s_arcrd_rp3;
    assign m2_arcrd_rp3 = {CRD_WIDTH{(arcrd_dmx == 2'd2)}} & s_arcrd_rp3;
    assign m3_arcrd_rp3 = {CRD_WIDTH{(arcrd_dmx == 2'd3)}} & s_arcrd_rp3;
    assign s_arcrd_rp3_ready = ((arcrd_dmx == 2'd0) & m0_arcrd_rp3_ready) |
                               ((arcrd_dmx == 2'd1) & m1_arcrd_rp3_ready) |
                               ((arcrd_dmx == 2'd2) & m2_arcrd_rp3_ready) |
                               ((arcrd_dmx == 2'd3) & m3_arcrd_rp3_ready);

endmodule
