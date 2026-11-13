// =============================================================================
// module name：p_axi4_unbond
// author: OpenHappy Team
// MAS：p_axi4_unbond_mas.md
// 功能概述: AXI4 通道解绑定共享模板，将 bond bus 还原为用户侧 AXI4 物理通道信号（纯组合）
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   2026-08-06  OpenHappy Team  初始版本
//   2026-08-11  OpenHappy Team  补全 hipi_sel 国标模式解封装（AW/AR）；修正 W/R lite 注释
//   2026-08-11  OpenHappy Team  BUG-001：HIPI Next 模式（hipi_sel=0）AW/AR 字段错位修复
//                      原实现用单 LHS + {2'b0,bus}/{bus,2'b0} 统一两模式，HIPI Next 模式
//                      末尾补 2'b0 与国标 bus 中间 2 个 pad 错位，导致 lock~user 6 段错位；
//                      改为逐段 hipi_sel 选位（id~prot 8 段选位，qos/region/user 两模式对齐直通）
// =============================================================================
// NOTE: bond bus 位宽与 p_axi4_bond 保持一致（见 bond 模块头部 NOTE）。
// =============================================================================


module p_axi4_unbond #(
    parameter AXI_DW = 512,
    parameter AX_WIDTH = 107,
    parameter W_WIDTH  = AXI_DW + AXI_DW/8 + 5,   // {wuser, wlast, wstrb, wdata}（不含 lite）
    parameter R_WIDTH  = AXI_DW + 17,              // {ruser, rlast, rid, rresp, rdata}（不含 lite）
    parameter B_WIDTH  = 16                         // {buser, bresp, bid}
) (

    // 用户侧 AXI4 Master AW（4 组，resp 方向 active）
    output wire [9:0]  m0_axi_awid,   output wire [63:0] m0_axi_awaddr,
    output wire [7:0]  m0_axi_awlen,  output wire [2:0]  m0_axi_awsize,
    output wire [1:0]  m0_axi_awburst,output wire        m0_axi_awlock,
    output wire [3:0]  m0_axi_awcache,output wire [2:0]  m0_axi_awprot,
    output wire [3:0]  m0_axi_awqos,  output wire [3:0]  m0_axi_awregion,
    output wire [3:0]  m0_axi_awuser, output wire        m0_axi_awvalid,
    input  wire        m0_axi_awready,
    output wire [9:0]  m1_axi_awid,   output wire [63:0] m1_axi_awaddr,
    output wire [7:0]  m1_axi_awlen,  output wire [2:0]  m1_axi_awsize,
    output wire [1:0]  m1_axi_awburst,output wire        m1_axi_awlock,
    output wire [3:0]  m1_axi_awcache,output wire [2:0]  m1_axi_awprot,
    output wire [3:0]  m1_axi_awqos,  output wire [3:0]  m1_axi_awregion,
    output wire [3:0]  m1_axi_awuser, output wire        m1_axi_awvalid,
    input  wire        m1_axi_awready,
    output wire [9:0]  m2_axi_awid,   output wire [63:0] m2_axi_awaddr,
    output wire [7:0]  m2_axi_awlen,  output wire [2:0]  m2_axi_awsize,
    output wire [1:0]  m2_axi_awburst,output wire        m2_axi_awlock,
    output wire [3:0]  m2_axi_awcache,output wire [2:0]  m2_axi_awprot,
    output wire [3:0]  m2_axi_awqos,  output wire [3:0]  m2_axi_awregion,
    output wire [3:0]  m2_axi_awuser, output wire        m2_axi_awvalid,
    input  wire        m2_axi_awready,
    output wire [9:0]  m3_axi_awid,   output wire [63:0] m3_axi_awaddr,
    output wire [7:0]  m3_axi_awlen,  output wire [2:0]  m3_axi_awsize,
    output wire [1:0]  m3_axi_awburst,output wire        m3_axi_awlock,
    output wire [3:0]  m3_axi_awcache,output wire [2:0]  m3_axi_awprot,
    output wire [3:0]  m3_axi_awqos,  output wire [3:0]  m3_axi_awregion,
    output wire [3:0]  m3_axi_awuser, output wire        m3_axi_awvalid,
    input  wire        m3_axi_awready,

    // 用户侧 AXI4 Master AR（4 组，resp 方向 active）
    output wire [9:0]  m0_axi_arid,   output wire [63:0] m0_axi_araddr,
    output wire [7:0]  m0_axi_arlen,  output wire [2:0]  m0_axi_arsize,
    output wire [1:0]  m0_axi_arburst,output wire        m0_axi_arlock,
    output wire [3:0]  m0_axi_arcache,output wire [2:0]  m0_axi_arprot,
    output wire [3:0]  m0_axi_arqos,  output wire [3:0]  m0_axi_arregion,
    output wire [3:0]  m0_axi_aruser, output wire        m0_axi_arvalid,
    input  wire        m0_axi_arready,
    output wire [9:0]  m1_axi_arid,   output wire [63:0] m1_axi_araddr,
    output wire [7:0]  m1_axi_arlen,  output wire [2:0]  m1_axi_arsize,
    output wire [1:0]  m1_axi_arburst,output wire        m1_axi_arlock,
    output wire [3:0]  m1_axi_arcache,output wire [2:0]  m1_axi_arprot,
    output wire [3:0]  m1_axi_arqos,  output wire [3:0]  m1_axi_arregion,
    output wire [3:0]  m1_axi_aruser, output wire        m1_axi_arvalid,
    input  wire        m1_axi_arready,
    output wire [9:0]  m2_axi_arid,   output wire [63:0] m2_axi_araddr,
    output wire [7:0]  m2_axi_arlen,  output wire [2:0]  m2_axi_arsize,
    output wire [1:0]  m2_axi_arburst,output wire        m2_axi_arlock,
    output wire [3:0]  m2_axi_arcache,output wire [2:0]  m2_axi_arprot,
    output wire [3:0]  m2_axi_arqos,  output wire [3:0]  m2_axi_arregion,
    output wire [3:0]  m2_axi_aruser, output wire        m2_axi_arvalid,
    input  wire        m2_axi_arready,
    output wire [9:0]  m3_axi_arid,   output wire [63:0] m3_axi_araddr,
    output wire [7:0]  m3_axi_arlen,  output wire [2:0]  m3_axi_arsize,
    output wire [1:0]  m3_axi_arburst,output wire        m3_axi_arlock,
    output wire [3:0]  m3_axi_arcache,output wire [2:0]  m3_axi_arprot,
    output wire [3:0]  m3_axi_arqos,  output wire [3:0]  m3_axi_arregion,
    output wire [3:0]  m3_axi_aruser, output wire        m3_axi_arvalid,
    input  wire        m3_axi_arready,

    // 用户侧 AXI4 Master W（4 组，resp 方向 active）
    output wire [AXI_DW-1:0]     m0_axi_wdata, output wire [AXI_DW/8-1:0] m0_axi_wstrb,
    output wire [3:0]             m0_axi_wuser, output wire                  m0_axi_wlast,
    output wire                   m0_axi_wvalid,
    input  wire                   m0_axi_wready,
    output wire [AXI_DW-1:0]     m1_axi_wdata, output wire [AXI_DW/8-1:0] m1_axi_wstrb,
    output wire [3:0]             m1_axi_wuser, output wire                  m1_axi_wlast,
    output wire                   m1_axi_wvalid,
    input  wire                   m1_axi_wready,
    output wire [AXI_DW-1:0]     m2_axi_wdata, output wire [AXI_DW/8-1:0] m2_axi_wstrb,
    output wire [3:0]             m2_axi_wuser, output wire                  m2_axi_wlast,
    output wire                   m2_axi_wvalid,
    input  wire                   m2_axi_wready,
    output wire [AXI_DW-1:0]     m3_axi_wdata, output wire [AXI_DW/8-1:0] m3_axi_wstrb,
    output wire [3:0]             m3_axi_wuser, output wire                  m3_axi_wlast,
    output wire                   m3_axi_wvalid,
    input  wire                   m3_axi_wready,

    // 用户侧 AXI4 Slave R（4 组，req 方向 active）
    output wire [9:0]             s0_axi_rid,   output wire [AXI_DW-1:0]   s0_axi_rdata,
    output wire [1:0]             s0_axi_rresp, output wire [3:0]            s0_axi_ruser,
    output wire                   s0_axi_rlast,
    output wire                   s0_axi_rvalid,input  wire                  s0_axi_rready,
    output wire [9:0]             s1_axi_rid,   output wire [AXI_DW-1:0]   s1_axi_rdata,
    output wire [1:0]             s1_axi_rresp, output wire [3:0]            s1_axi_ruser,
    output wire                   s1_axi_rlast,
    output wire                   s1_axi_rvalid,input  wire                  s1_axi_rready,
    output wire [9:0]             s2_axi_rid,   output wire [AXI_DW-1:0]   s2_axi_rdata,
    output wire [1:0]             s2_axi_rresp, output wire [3:0]            s2_axi_ruser,
    output wire                   s2_axi_rlast,
    output wire                   s2_axi_rvalid,input  wire                  s2_axi_rready,
    output wire [9:0]             s3_axi_rid,   output wire [AXI_DW-1:0]   s3_axi_rdata,
    output wire [1:0]             s3_axi_rresp, output wire [3:0]            s3_axi_ruser,
    output wire                   s3_axi_rlast,
    output wire                   s3_axi_rvalid,input  wire                  s3_axi_rready,

    // 用户侧 AXI4 Slave B（4 组，req 方向 active）
    output wire [9:0]             s0_axi_bid,   output wire [1:0]            s0_axi_bresp,
    output wire [3:0]             s0_axi_buser, output wire                  s0_axi_bvalid,
    input  wire                   s0_axi_bready,
    output wire [9:0]             s1_axi_bid,   output wire [1:0]            s1_axi_bresp,
    output wire [3:0]             s1_axi_buser, output wire                  s1_axi_bvalid,
    input  wire                   s1_axi_bready,
    output wire [9:0]             s2_axi_bid,   output wire [1:0]            s2_axi_bresp,
    output wire [3:0]             s2_axi_buser, output wire                  s2_axi_bvalid,
    input  wire                   s2_axi_bready,
    output wire [9:0]             s3_axi_bid,   output wire [1:0]            s3_axi_bresp,
    output wire [3:0]             s3_axi_buser, output wire                  s3_axi_bvalid,
    input  wire                   s3_axi_bready,

    // AW bond bus 输入（4 组）
    input  wire [AX_WIDTH-1:0]    aw0_rx_bus, input wire aw0_rx_valid, output wire aw0_rx_ready,
    input  wire [AX_WIDTH-1:0]    aw1_rx_bus, input wire aw1_rx_valid, output wire aw1_rx_ready,
    input  wire [AX_WIDTH-1:0]    aw2_rx_bus, input wire aw2_rx_valid, output wire aw2_rx_ready,
    input  wire [AX_WIDTH-1:0]    aw3_rx_bus, input wire aw3_rx_valid, output wire aw3_rx_ready,
    // AR bond bus 输入（4 组）
    input  wire [AX_WIDTH-1:0]    ar0_rx_bus, input wire ar0_rx_valid, output wire ar0_rx_ready,
    input  wire [AX_WIDTH-1:0]    ar1_rx_bus, input wire ar1_rx_valid, output wire ar1_rx_ready,
    input  wire [AX_WIDTH-1:0]    ar2_rx_bus, input wire ar2_rx_valid, output wire ar2_rx_ready,
    input  wire [AX_WIDTH-1:0]    ar3_rx_bus, input wire ar3_rx_valid, output wire ar3_rx_ready,
    // W bond bus 输入（4 组）
    input  wire [W_WIDTH-1:0]     w0_rx_bus,  input wire w0_rx_valid,  output wire w0_rx_ready,
    input  wire [W_WIDTH-1:0]     w1_rx_bus,  input wire w1_rx_valid,  output wire w1_rx_ready,
    input  wire [W_WIDTH-1:0]     w2_rx_bus,  input wire w2_rx_valid,  output wire w2_rx_ready,
    input  wire [W_WIDTH-1:0]     w3_rx_bus,  input wire w3_rx_valid,  output wire w3_rx_ready,
    // R bond bus 输入（4 组）
    input  wire [R_WIDTH-1:0]     r0_rx_bus,  input wire r0_rx_valid,  output wire r0_rx_ready,
    input  wire [R_WIDTH-1:0]     r1_rx_bus,  input wire r1_rx_valid,  output wire r1_rx_ready,
    input  wire [R_WIDTH-1:0]     r2_rx_bus,  input wire r2_rx_valid,  output wire r2_rx_ready,
    input  wire [R_WIDTH-1:0]     r3_rx_bus,  input wire r3_rx_valid,  output wire r3_rx_ready,
    // B bond bus 输入（4 组）
    input  wire [B_WIDTH-1:0]     b0_rx_bus,  input wire b0_rx_valid,  output wire b0_rx_ready,
    input  wire [B_WIDTH-1:0]     b1_rx_bus,  input wire b1_rx_valid,  output wire b1_rx_ready,
    input  wire [B_WIDTH-1:0]     b2_rx_bus,  input wire b2_rx_valid,  output wire b2_rx_ready,
    input  wire [B_WIDTH-1:0]     b3_rx_bus,  input wire b3_rx_valid,  output wire b3_rx_ready
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    // AX_WIDTH / W_WIDTH / R_WIDTH / B_WIDTH 已上提到 #() 块（端口列表引用，须为 parameter）

    // lite 为独立打包标志，不进 bus；bus 解封装不含 lite

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================
    // AW/AR 通道解封装（仅 HIPI Next 模式；2026-09-22 删除 hipi_sel 国标模式）
    // bus 107bit 遵循 HIPI_AXI4_Next.md：ID 低位、USER 高位，紧凑无填充：
    //   {user[106:103],region[102:99],qos[98:95],prot[94:92],cache[91:88],
    //    lock[87],burst[86:85],size[84:82],len[81:74],addr[73:10],id[9:0]}
    //   历史 BUG-001（2026-08-11，hipi_sel 双模式时代）：单 LHS+补 2'b0 统一两模式导致 lock~user
    //   6 段错位，曾改逐段选位；hipi_sel 删除后仅保留 HIPI Next 直取分支。
    // 以下 bit 索引：bus[106:0]（低位=id、高位=user）；格式与 bond 模块 aw/ar*_tx_bus 的 hipi_sel=0 分支对应。
    //
    // AW 通道
    // RP0 AW
    assign m0_axi_awid    = aw0_rx_bus[9:0];
    assign m0_axi_awaddr  = aw0_rx_bus[73:10];
    assign m0_axi_awlen   = aw0_rx_bus[81:74];
    assign m0_axi_awsize  = aw0_rx_bus[84:82];
    assign m0_axi_awburst = aw0_rx_bus[86:85];
    assign m0_axi_awlock  = aw0_rx_bus[87];
    assign m0_axi_awcache = aw0_rx_bus[91:88];
    assign m0_axi_awprot  = aw0_rx_bus[94:92];
    assign m0_axi_awqos   = aw0_rx_bus[98:95];   // 两模式对齐
    assign m0_axi_awregion= aw0_rx_bus[102:99];    // 两模式对齐
    assign m0_axi_awuser  = aw0_rx_bus[106:103];    // 两模式对齐
    assign m0_axi_awvalid = aw0_rx_valid;
    assign aw0_rx_ready   = m0_axi_awready;
    // RP1 AW
    assign m1_axi_awid    = aw1_rx_bus[9:0];
    assign m1_axi_awaddr  = aw1_rx_bus[73:10];
    assign m1_axi_awlen   = aw1_rx_bus[81:74];
    assign m1_axi_awsize  = aw1_rx_bus[84:82];
    assign m1_axi_awburst = aw1_rx_bus[86:85];
    assign m1_axi_awlock  = aw1_rx_bus[87];
    assign m1_axi_awcache = aw1_rx_bus[91:88];
    assign m1_axi_awprot  = aw1_rx_bus[94:92];
    assign m1_axi_awqos   = aw1_rx_bus[98:95];
    assign m1_axi_awregion= aw1_rx_bus[102:99];
    assign m1_axi_awuser  = aw1_rx_bus[106:103];
    assign m1_axi_awvalid = aw1_rx_valid;
    assign aw1_rx_ready   = m1_axi_awready;
    // RP2 AW
    assign m2_axi_awid    = aw2_rx_bus[9:0];
    assign m2_axi_awaddr  = aw2_rx_bus[73:10];
    assign m2_axi_awlen   = aw2_rx_bus[81:74];
    assign m2_axi_awsize  = aw2_rx_bus[84:82];
    assign m2_axi_awburst = aw2_rx_bus[86:85];
    assign m2_axi_awlock  = aw2_rx_bus[87];
    assign m2_axi_awcache = aw2_rx_bus[91:88];
    assign m2_axi_awprot  = aw2_rx_bus[94:92];
    assign m2_axi_awqos   = aw2_rx_bus[98:95];
    assign m2_axi_awregion= aw2_rx_bus[102:99];
    assign m2_axi_awuser  = aw2_rx_bus[106:103];
    assign m2_axi_awvalid = aw2_rx_valid;
    assign aw2_rx_ready   = m2_axi_awready;
    // RP3 AW
    assign m3_axi_awid    = aw3_rx_bus[9:0];
    assign m3_axi_awaddr  = aw3_rx_bus[73:10];
    assign m3_axi_awlen   = aw3_rx_bus[81:74];
    assign m3_axi_awsize  = aw3_rx_bus[84:82];
    assign m3_axi_awburst = aw3_rx_bus[86:85];
    assign m3_axi_awlock  = aw3_rx_bus[87];
    assign m3_axi_awcache = aw3_rx_bus[91:88];
    assign m3_axi_awprot  = aw3_rx_bus[94:92];
    assign m3_axi_awqos   = aw3_rx_bus[98:95];
    assign m3_axi_awregion= aw3_rx_bus[102:99];
    assign m3_axi_awuser  = aw3_rx_bus[106:103];
    assign m3_axi_awvalid = aw3_rx_valid;
    assign aw3_rx_ready   = m3_axi_awready;

    // AR 通道（同 AW，HIPI Next 排布）
    // RP0 AR
    assign m0_axi_arid    = ar0_rx_bus[9:0];
    assign m0_axi_araddr  = ar0_rx_bus[73:10];
    assign m0_axi_arlen   = ar0_rx_bus[81:74];
    assign m0_axi_arsize  = ar0_rx_bus[84:82];
    assign m0_axi_arburst = ar0_rx_bus[86:85];
    assign m0_axi_arlock  = ar0_rx_bus[87];
    assign m0_axi_arcache = ar0_rx_bus[91:88];
    assign m0_axi_arprot  = ar0_rx_bus[94:92];
    assign m0_axi_arqos   = ar0_rx_bus[98:95];
    assign m0_axi_arregion= ar0_rx_bus[102:99];
    assign m0_axi_aruser  = ar0_rx_bus[106:103];
    assign m0_axi_arvalid = ar0_rx_valid;
    assign ar0_rx_ready   = m0_axi_arready;
    // RP1 AR
    assign m1_axi_arid    = ar1_rx_bus[9:0];
    assign m1_axi_araddr  = ar1_rx_bus[73:10];
    assign m1_axi_arlen   = ar1_rx_bus[81:74];
    assign m1_axi_arsize  = ar1_rx_bus[84:82];
    assign m1_axi_arburst = ar1_rx_bus[86:85];
    assign m1_axi_arlock  = ar1_rx_bus[87];
    assign m1_axi_arcache = ar1_rx_bus[91:88];
    assign m1_axi_arprot  = ar1_rx_bus[94:92];
    assign m1_axi_arqos   = ar1_rx_bus[98:95];
    assign m1_axi_arregion= ar1_rx_bus[102:99];
    assign m1_axi_aruser  = ar1_rx_bus[106:103];
    assign m1_axi_arvalid = ar1_rx_valid;
    assign ar1_rx_ready   = m1_axi_arready;
    // RP2 AR
    assign m2_axi_arid    = ar2_rx_bus[9:0];
    assign m2_axi_araddr  = ar2_rx_bus[73:10];
    assign m2_axi_arlen   = ar2_rx_bus[81:74];
    assign m2_axi_arsize  = ar2_rx_bus[84:82];
    assign m2_axi_arburst = ar2_rx_bus[86:85];
    assign m2_axi_arlock  = ar2_rx_bus[87];
    assign m2_axi_arcache = ar2_rx_bus[91:88];
    assign m2_axi_arprot  = ar2_rx_bus[94:92];
    assign m2_axi_arqos   = ar2_rx_bus[98:95];
    assign m2_axi_arregion= ar2_rx_bus[102:99];
    assign m2_axi_aruser  = ar2_rx_bus[106:103];
    assign m2_axi_arvalid = ar2_rx_valid;
    assign ar2_rx_ready   = m2_axi_arready;
    // RP3 AR
    assign m3_axi_arid    = ar3_rx_bus[9:0];
    assign m3_axi_araddr  = ar3_rx_bus[73:10];
    assign m3_axi_arlen   = ar3_rx_bus[81:74];
    assign m3_axi_arsize  = ar3_rx_bus[84:82];
    assign m3_axi_arburst = ar3_rx_bus[86:85];
    assign m3_axi_arlock  = ar3_rx_bus[87];
    assign m3_axi_arcache = ar3_rx_bus[91:88];
    assign m3_axi_arprot  = ar3_rx_bus[94:92];
    assign m3_axi_arqos   = ar3_rx_bus[98:95];
    assign m3_axi_arregion= ar3_rx_bus[102:99];
    assign m3_axi_aruser  = ar3_rx_bus[106:103];
    assign m3_axi_arvalid = ar3_rx_valid;
    assign ar3_rx_ready   = m3_axi_arready;

    // W 通道解封装（bus 不含 w_lite，lite 由 bond 独立输出；解封装无需丢弃）
    assign {m0_axi_wuser, m0_axi_wlast, m0_axi_wstrb, m0_axi_wdata} = w0_rx_bus;
    assign m0_axi_wvalid = w0_rx_valid;
    assign w0_rx_ready   = m0_axi_wready;

    assign {m1_axi_wuser, m1_axi_wlast, m1_axi_wstrb, m1_axi_wdata} = w1_rx_bus;
    assign m1_axi_wvalid = w1_rx_valid;
    assign w1_rx_ready   = m1_axi_wready;

    assign {m2_axi_wuser, m2_axi_wlast, m2_axi_wstrb, m2_axi_wdata} = w2_rx_bus;
    assign m2_axi_wvalid = w2_rx_valid;
    assign w2_rx_ready   = m2_axi_wready;

    assign {m3_axi_wuser, m3_axi_wlast, m3_axi_wstrb, m3_axi_wdata} = w3_rx_bus;
    assign m3_axi_wvalid = w3_rx_valid;
    assign w3_rx_ready   = m3_axi_wready;

    // R 通道解封装（bus 不含 r_lite，lite 由 bond 独立输出；解封装无需丢弃）
    assign {s0_axi_ruser, s0_axi_rlast, s0_axi_rid, s0_axi_rresp, s0_axi_rdata} = r0_rx_bus;
    assign s0_axi_rvalid = r0_rx_valid;
    assign r0_rx_ready   = s0_axi_rready;

    assign {s1_axi_ruser, s1_axi_rlast, s1_axi_rid, s1_axi_rresp, s1_axi_rdata} = r1_rx_bus;
    assign s1_axi_rvalid = r1_rx_valid;
    assign r1_rx_ready   = s1_axi_rready;

    assign {s2_axi_ruser, s2_axi_rlast, s2_axi_rid, s2_axi_rresp, s2_axi_rdata} = r2_rx_bus;
    assign s2_axi_rvalid = r2_rx_valid;
    assign r2_rx_ready   = s2_axi_rready;

    assign {s3_axi_ruser, s3_axi_rlast, s3_axi_rid, s3_axi_rresp, s3_axi_rdata} = r3_rx_bus;
    assign s3_axi_rvalid = r3_rx_valid;
    assign r3_rx_ready   = s3_axi_rready;

    // B 通道解封装
    assign {s0_axi_buser, s0_axi_bresp, s0_axi_bid} = b0_rx_bus;
    assign s0_axi_bvalid = b0_rx_valid;
    assign b0_rx_ready   = s0_axi_bready;

    assign {s1_axi_buser, s1_axi_bresp, s1_axi_bid} = b1_rx_bus;
    assign s1_axi_bvalid = b1_rx_valid;
    assign b1_rx_ready   = s1_axi_bready;

    assign {s2_axi_buser, s2_axi_bresp, s2_axi_bid} = b2_rx_bus;
    assign s2_axi_bvalid = b2_rx_valid;
    assign b2_rx_ready   = s2_axi_bready;

    assign {s3_axi_buser, s3_axi_bresp, s3_axi_bid} = b3_rx_bus;
    assign s3_axi_bvalid = b3_rx_valid;
    assign b3_rx_ready   = s3_axi_bready;

endmodule
//   2026-09-22  OpenHappy Team  删除 hipi_sel 国标模式：移除 hipi_sel 端口与 AW/AR 逐段选位 MUX（8 字段×4 lane×2 通道=64 处），
//                      仅保留 HIPI Next 直取分支（与 bond 的 hipi_sel=0 输出格式对应）；W/R/B 通道本无 hipi_sel，不变
