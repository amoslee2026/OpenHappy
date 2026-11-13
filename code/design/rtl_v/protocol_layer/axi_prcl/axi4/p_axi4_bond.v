// =============================================================================
// module name：p_axi4_bond
// author: OpenHappy Team
// MAS：p_axi4_bond_mas.md
// 功能概述: AXI4 通道绑定共享模板，将 AXI4 物理通道按 HIPI Next 协议合并为 bond bus 送 txfifo
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   2026-08-06  OpenHappy Team  初始版本
// =============================================================================
// NOTE: bond bus 位宽按 MAS §5.1 字段拼接与分析源 md(p_axi4_bond.md) 推导：
//   AX_WIDTH = 107 (AW/AR：id+addr+len+size+burst+lock+cache+prot+qos+region+user)
//   W_WIDTH  = AXI_DW + AXI_DW/8 + 5 ({wuser, wlast, wstrb, wdata})
//   R_WIDTH  = AXI_DW + 17 ({ruser, rlast, rid, rresp, rdata})
//   B_WIDTH  = 16 ({buser, bresp, bid})
//   lite 标志（w_lite/r_lite）为独立输出，不进 bus，供 pack 选择 lite/full VC。
//   位宽标注与 MAS §6 / p_axi4_rch_pack_mas.md（R_WIDTH=PAIF_DW+17）已对齐。
// =============================================================================


module p_axi4_bond #(
    parameter AXI_DW = 512,
    parameter AX_WIDTH = 107,
    parameter W_WIDTH  = AXI_DW + AXI_DW/8 + 5,   // {wuser, wlast, wstrb, wdata}（不含 lite）
    parameter R_WIDTH  = AXI_DW + 17,              // {ruser, rlast, rid, rresp, rdata}（不含 lite）
    parameter B_WIDTH  = 16                         // {buser, bresp, bid}
) (
    // 时钟、复位与模式选择
    input  wire        clk,
    input  wire        rstn,
    input  wire        hipi_sel,

    // 用户侧 AXI4 Slave AW（4 组）
    input  wire [9:0]  s0_axi_awid,   input  wire [63:0] s0_axi_awaddr,
    input  wire [7:0]  s0_axi_awlen,  input  wire [2:0]  s0_axi_awsize,
    input  wire [1:0]  s0_axi_awburst,input  wire        s0_axi_awlock,
    input  wire [3:0]  s0_axi_awcache,input  wire [2:0]  s0_axi_awprot,
    input  wire [3:0]  s0_axi_awqos,  input  wire [3:0]  s0_axi_awregion,
    input  wire [3:0]  s0_axi_awuser, input  wire        s0_axi_awvalid,
    output wire        s0_axi_awready,
    input  wire [9:0]  s1_axi_awid,   input  wire [63:0] s1_axi_awaddr,
    input  wire [7:0]  s1_axi_awlen,  input  wire [2:0]  s1_axi_awsize,
    input  wire [1:0]  s1_axi_awburst,input  wire        s1_axi_awlock,
    input  wire [3:0]  s1_axi_awcache,input  wire [2:0]  s1_axi_awprot,
    input  wire [3:0]  s1_axi_awqos,  input  wire [3:0]  s1_axi_awregion,
    input  wire [3:0]  s1_axi_awuser, input  wire        s1_axi_awvalid,
    output wire        s1_axi_awready,
    input  wire [9:0]  s2_axi_awid,   input  wire [63:0] s2_axi_awaddr,
    input  wire [7:0]  s2_axi_awlen,  input  wire [2:0]  s2_axi_awsize,
    input  wire [1:0]  s2_axi_awburst,input  wire        s2_axi_awlock,
    input  wire [3:0]  s2_axi_awcache,input  wire [2:0]  s2_axi_awprot,
    input  wire [3:0]  s2_axi_awqos,  input  wire [3:0]  s2_axi_awregion,
    input  wire [3:0]  s2_axi_awuser, input  wire        s2_axi_awvalid,
    output wire        s2_axi_awready,
    input  wire [9:0]  s3_axi_awid,   input  wire [63:0] s3_axi_awaddr,
    input  wire [7:0]  s3_axi_awlen,  input  wire [2:0]  s3_axi_awsize,
    input  wire [1:0]  s3_axi_awburst,input  wire        s3_axi_awlock,
    input  wire [3:0]  s3_axi_awcache,input  wire [2:0]  s3_axi_awprot,
    input  wire [3:0]  s3_axi_awqos,  input  wire [3:0]  s3_axi_awregion,
    input  wire [3:0]  s3_axi_awuser, input  wire        s3_axi_awvalid,
    output wire        s3_axi_awready,

    // 用户侧 AXI4 Slave AR（4 组）
    input  wire [9:0]  s0_axi_arid,   input  wire [63:0] s0_axi_araddr,
    input  wire [7:0]  s0_axi_arlen,  input  wire [2:0]  s0_axi_arsize,
    input  wire [1:0]  s0_axi_arburst,input  wire        s0_axi_arlock,
    input  wire [3:0]  s0_axi_arcache,input  wire [2:0]  s0_axi_arprot,
    input  wire [3:0]  s0_axi_arqos,  input  wire [3:0]  s0_axi_arregion,
    input  wire [3:0]  s0_axi_aruser, input  wire        s0_axi_arvalid,
    output wire        s0_axi_arready,
    input  wire [9:0]  s1_axi_arid,   input  wire [63:0] s1_axi_araddr,
    input  wire [7:0]  s1_axi_arlen,  input  wire [2:0]  s1_axi_arsize,
    input  wire [1:0]  s1_axi_arburst,input  wire        s1_axi_arlock,
    input  wire [3:0]  s1_axi_arcache,input  wire [2:0]  s1_axi_arprot,
    input  wire [3:0]  s1_axi_arqos,  input  wire [3:0]  s1_axi_arregion,
    input  wire [3:0]  s1_axi_aruser, input  wire        s1_axi_arvalid,
    output wire        s1_axi_arready,
    input  wire [9:0]  s2_axi_arid,   input  wire [63:0] s2_axi_araddr,
    input  wire [7:0]  s2_axi_arlen,  input  wire [2:0]  s2_axi_arsize,
    input  wire [1:0]  s2_axi_arburst,input  wire        s2_axi_arlock,
    input  wire [3:0]  s2_axi_arcache,input  wire [2:0]  s2_axi_arprot,
    input  wire [3:0]  s2_axi_arqos,  input  wire [3:0]  s2_axi_arregion,
    input  wire [3:0]  s2_axi_aruser, input  wire        s2_axi_arvalid,
    output wire        s2_axi_arready,
    input  wire [9:0]  s3_axi_arid,   input  wire [63:0] s3_axi_araddr,
    input  wire [7:0]  s3_axi_arlen,  input  wire [2:0]  s3_axi_arsize,
    input  wire [1:0]  s3_axi_arburst,input  wire        s3_axi_arlock,
    input  wire [3:0]  s3_axi_arcache,input  wire [2:0]  s3_axi_arprot,
    input  wire [3:0]  s3_axi_arqos,  input  wire [3:0]  s3_axi_arregion,
    input  wire [3:0]  s3_axi_aruser, input  wire        s3_axi_arvalid,
    output wire        s3_axi_arready,

    // 用户侧 AXI4 Slave W（4 组）
    input  wire [AXI_DW-1:0]     s0_axi_wdata, input  wire [AXI_DW/8-1:0] s0_axi_wstrb,
    input  wire [3:0]             s0_axi_wuser, input  wire                  s0_axi_wlast,
    input  wire                   s0_axi_wvalid,
    output wire                   s0_axi_wready,
    input  wire [AXI_DW-1:0]     s1_axi_wdata, input  wire [AXI_DW/8-1:0] s1_axi_wstrb,
    input  wire [3:0]             s1_axi_wuser, input  wire                  s1_axi_wlast,
    input  wire                   s1_axi_wvalid,
    output wire                   s1_axi_wready,
    input  wire [AXI_DW-1:0]     s2_axi_wdata, input  wire [AXI_DW/8-1:0] s2_axi_wstrb,
    input  wire [3:0]             s2_axi_wuser, input  wire                  s2_axi_wlast,
    input  wire                   s2_axi_wvalid,
    output wire                   s2_axi_wready,
    input  wire [AXI_DW-1:0]     s3_axi_wdata, input  wire [AXI_DW/8-1:0] s3_axi_wstrb,
    input  wire [3:0]             s3_axi_wuser, input  wire                  s3_axi_wlast,
    input  wire                   s3_axi_wvalid,
    output wire                   s3_axi_wready,

    // 用户侧 AXI4 Master R（4 组）
    input  wire [9:0]             m0_axi_rid,   input  wire [AXI_DW-1:0]   m0_axi_rdata,
    input  wire [1:0]             m0_axi_rresp, input  wire [3:0]            m0_axi_ruser,
    input  wire                   m0_axi_rlast,
    input  wire                   m0_axi_rvalid,output wire                  m0_axi_rready,
    input  wire [9:0]             m1_axi_rid,   input  wire [AXI_DW-1:0]   m1_axi_rdata,
    input  wire [1:0]             m1_axi_rresp, input  wire [3:0]            m1_axi_ruser,
    input  wire                   m1_axi_rlast,
    input  wire                   m1_axi_rvalid,output wire                  m1_axi_rready,
    input  wire [9:0]             m2_axi_rid,   input  wire [AXI_DW-1:0]   m2_axi_rdata,
    input  wire [1:0]             m2_axi_rresp, input  wire [3:0]            m2_axi_ruser,
    input  wire                   m2_axi_rlast,
    input  wire                   m2_axi_rvalid,output wire                  m2_axi_rready,
    input  wire [9:0]             m3_axi_rid,   input  wire [AXI_DW-1:0]   m3_axi_rdata,
    input  wire [1:0]             m3_axi_rresp, input  wire [3:0]            m3_axi_ruser,
    input  wire                   m3_axi_rlast,
    input  wire                   m3_axi_rvalid,output wire                  m3_axi_rready,

    // 用户侧 AXI4 Master B（4 组）
    input  wire [9:0]             m0_axi_bid,   input  wire [1:0]            m0_axi_bresp,
    input  wire [3:0]             m0_axi_buser, input  wire                  m0_axi_bvalid,
    output wire                   m0_axi_bready,
    input  wire [9:0]             m1_axi_bid,   input  wire [1:0]            m1_axi_bresp,
    input  wire [3:0]             m1_axi_buser, input  wire                  m1_axi_bvalid,
    output wire                   m1_axi_bready,
    input  wire [9:0]             m2_axi_bid,   input  wire [1:0]            m2_axi_bresp,
    input  wire [3:0]             m2_axi_buser, input  wire                  m2_axi_bvalid,
    output wire                   m2_axi_bready,
    input  wire [9:0]             m3_axi_bid,   input  wire [1:0]            m3_axi_bresp,
    input  wire [3:0]             m3_axi_buser, input  wire                  m3_axi_bvalid,
    output wire                   m3_axi_bready,

    // AW bond bus 输出（4 组）
    output wire [AX_WIDTH-1:0]    aw0_tx_bus, output wire aw0_tx_valid, input wire aw0_tx_ready,
    output wire [AX_WIDTH-1:0]    aw1_tx_bus, output wire aw1_tx_valid, input wire aw1_tx_ready,
    output wire [AX_WIDTH-1:0]    aw2_tx_bus, output wire aw2_tx_valid, input wire aw2_tx_ready,
    output wire [AX_WIDTH-1:0]    aw3_tx_bus, output wire aw3_tx_valid, input wire aw3_tx_ready,
    // AR bond bus 输出（4 组）
    output wire [AX_WIDTH-1:0]    ar0_tx_bus, output wire ar0_tx_valid, input wire ar0_tx_ready,
    output wire [AX_WIDTH-1:0]    ar1_tx_bus, output wire ar1_tx_valid, input wire ar1_tx_ready,
    output wire [AX_WIDTH-1:0]    ar2_tx_bus, output wire ar2_tx_valid, input wire ar2_tx_ready,
    output wire [AX_WIDTH-1:0]    ar3_tx_bus, output wire ar3_tx_valid, input wire ar3_tx_ready,
    // W bond bus 输出（4 组）
    output wire [W_WIDTH-1:0]     w0_tx_bus,  output wire w0_tx_valid,  input wire w0_tx_ready,
    output wire [W_WIDTH-1:0]     w1_tx_bus,  output wire w1_tx_valid,  input wire w1_tx_ready,
    output wire [W_WIDTH-1:0]     w2_tx_bus,  output wire w2_tx_valid,  input wire w2_tx_ready,
    output wire [W_WIDTH-1:0]     w3_tx_bus,  output wire w3_tx_valid,  input wire w3_tx_ready,
    // R bond bus 输出（4 组）
    output wire [R_WIDTH-1:0]     r0_tx_bus,  output wire r0_tx_valid,  input wire r0_tx_ready,
    output wire [R_WIDTH-1:0]     r1_tx_bus,  output wire r1_tx_valid,  input wire r1_tx_ready,
    output wire [R_WIDTH-1:0]     r2_tx_bus,  output wire r2_tx_valid,  input wire r2_tx_ready,
    output wire [R_WIDTH-1:0]     r3_tx_bus,  output wire r3_tx_valid,  input wire r3_tx_ready,
    // B bond bus 输出（4 组）
    output wire [B_WIDTH-1:0]     b0_tx_bus,  output wire b0_tx_valid,  input wire b0_tx_ready,
    output wire [B_WIDTH-1:0]     b1_tx_bus,  output wire b1_tx_valid,  input wire b1_tx_ready,
    output wire [B_WIDTH-1:0]     b2_tx_bus,  output wire b2_tx_valid,  input wire b2_tx_ready,
    output wire [B_WIDTH-1:0]     b3_tx_bus,  output wire b3_tx_valid,  input wire b3_tx_ready,
    // lite 打包模式标志（独立信号，不进 bus，供 pack 选择 lite/full VC）
    output wire                   w0_lite,    output wire w1_lite,
    output wire                   w2_lite,    output wire w3_lite,
    output wire                   r0_lite,    output wire r1_lite,
    output wire                   r2_lite,    output wire r3_lite
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    // AX_WIDTH / W_WIDTH / R_WIDTH / B_WIDTH 已上提到 #() 块（端口列表引用，须为 parameter）

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    // R 通道 last_* 寄存器（每路独立，用于 r_lite 生成）
    reg        last_valid_r0, last_valid_r1, last_valid_r2, last_valid_r3;
    reg [9:0]  last_rid_r0,   last_rid_r1,   last_rid_r2,   last_rid_r3;
    reg [1:0]  last_rresp_r0, last_rresp_r1, last_rresp_r2, last_rresp_r3;

    wire       r0_hsk, r1_hsk, r2_hsk, r3_hsk;

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================
    // AW 通道（hipi_sel=0 HIPI Next：原宽传输；hipi_sel=1 国标：ID 取低 8bit/LOCK 高位填 0/QOS 高位填 0）
    // hipi_sel=1 时位宽不变：ID 少 2bit、LOCK 多 1bit、QOS 多 1bit，总量守恒
    // tx_bus 位序遵循 HIPI_AXI4_Next.md：ID 低位、USER 高位（107bit：id[9:0]/addr[73:10]/len[81:74]/
    //   size[84:82]/burst[86:85]/lock[87]/cache[91:88]/prot[94:92]/qos[98:95]/region[102:99]/user[106:103]）
    assign s0_axi_awready = aw0_tx_ready;
    assign aw0_tx_valid   = s0_axi_awvalid;
    assign aw0_tx_bus     = hipi_sel ? {s0_axi_awuser, s0_axi_awregion, s0_axi_awqos, 1'b0, s0_axi_awprot,
                                        s0_axi_awcache, s0_axi_awlock, 1'b0, s0_axi_awburst, s0_axi_awsize,
                                        s0_axi_awlen, s0_axi_awaddr, s0_axi_awid[7:0]}
                                     : {s0_axi_awuser, s0_axi_awregion, s0_axi_awqos, s0_axi_awprot,
                                        s0_axi_awcache, s0_axi_awlock, s0_axi_awburst, s0_axi_awsize,
                                        s0_axi_awlen, s0_axi_awaddr, s0_axi_awid};

    assign s1_axi_awready = aw1_tx_ready;
    assign aw1_tx_valid   = s1_axi_awvalid;
    assign aw1_tx_bus     = hipi_sel ? {s1_axi_awuser, s1_axi_awregion, s1_axi_awqos, 1'b0, s1_axi_awprot,
                                        s1_axi_awcache, s1_axi_awlock, 1'b0, s1_axi_awburst, s1_axi_awsize,
                                        s1_axi_awlen, s1_axi_awaddr, s1_axi_awid[7:0]}
                                     : {s1_axi_awuser, s1_axi_awregion, s1_axi_awqos, s1_axi_awprot,
                                        s1_axi_awcache, s1_axi_awlock, s1_axi_awburst, s1_axi_awsize,
                                        s1_axi_awlen, s1_axi_awaddr, s1_axi_awid};

    assign s2_axi_awready = aw2_tx_ready;
    assign aw2_tx_valid   = s2_axi_awvalid;
    assign aw2_tx_bus     = hipi_sel ? {s2_axi_awuser, s2_axi_awregion, s2_axi_awqos, 1'b0, s2_axi_awprot,
                                        s2_axi_awcache, s2_axi_awlock, 1'b0, s2_axi_awburst, s2_axi_awsize,
                                        s2_axi_awlen, s2_axi_awaddr, s2_axi_awid[7:0]}
                                     : {s2_axi_awuser, s2_axi_awregion, s2_axi_awqos, s2_axi_awprot,
                                        s2_axi_awcache, s2_axi_awlock, s2_axi_awburst, s2_axi_awsize,
                                        s2_axi_awlen, s2_axi_awaddr, s2_axi_awid};

    assign s3_axi_awready = aw3_tx_ready;
    assign aw3_tx_valid   = s3_axi_awvalid;
    assign aw3_tx_bus     = hipi_sel ? {s3_axi_awuser, s3_axi_awregion, s3_axi_awqos, 1'b0, s3_axi_awprot,
                                        s3_axi_awcache, s3_axi_awlock, 1'b0, s3_axi_awburst, s3_axi_awsize,
                                        s3_axi_awlen, s3_axi_awaddr, s3_axi_awid[7:0]}
                                     : {s3_axi_awuser, s3_axi_awregion, s3_axi_awqos, s3_axi_awprot,
                                        s3_axi_awcache, s3_axi_awlock, s3_axi_awburst, s3_axi_awsize,
                                        s3_axi_awlen, s3_axi_awaddr, s3_axi_awid};

    // AR 通道（同 AW）
    assign s0_axi_arready = ar0_tx_ready;
    assign ar0_tx_valid   = s0_axi_arvalid;
    assign ar0_tx_bus     = hipi_sel ? {s0_axi_aruser, s0_axi_arregion, s0_axi_arqos, 1'b0, s0_axi_arprot,
                                        s0_axi_arcache, s0_axi_arlock, 1'b0, s0_axi_arburst, s0_axi_arsize,
                                        s0_axi_arlen, s0_axi_araddr, s0_axi_arid[7:0]}
                                     : {s0_axi_aruser, s0_axi_arregion, s0_axi_arqos, s0_axi_arprot,
                                        s0_axi_arcache, s0_axi_arlock, s0_axi_arburst, s0_axi_arsize,
                                        s0_axi_arlen, s0_axi_araddr, s0_axi_arid};

    assign s1_axi_arready = ar1_tx_ready;
    assign ar1_tx_valid   = s1_axi_arvalid;
    assign ar1_tx_bus     = hipi_sel ? {s1_axi_aruser, s1_axi_arregion, s1_axi_arqos, 1'b0, s1_axi_arprot,
                                        s1_axi_arcache, s1_axi_arlock, 1'b0, s1_axi_arburst, s1_axi_arsize,
                                        s1_axi_arlen, s1_axi_araddr, s1_axi_arid[7:0]}
                                     : {s1_axi_aruser, s1_axi_arregion, s1_axi_arqos, s1_axi_arprot,
                                        s1_axi_arcache, s1_axi_arlock, s1_axi_arburst, s1_axi_arsize,
                                        s1_axi_arlen, s1_axi_araddr, s1_axi_arid};

    assign s2_axi_arready = ar2_tx_ready;
    assign ar2_tx_valid   = s2_axi_arvalid;
    assign ar2_tx_bus     = hipi_sel ? {s2_axi_aruser, s2_axi_arregion, s2_axi_arqos, 1'b0, s2_axi_arprot,
                                        s2_axi_arcache, s2_axi_arlock, 1'b0, s2_axi_arburst, s2_axi_arsize,
                                        s2_axi_arlen, s2_axi_araddr, s2_axi_arid[7:0]}
                                     : {s2_axi_aruser, s2_axi_arregion, s2_axi_arqos, s2_axi_arprot,
                                        s2_axi_arcache, s2_axi_arlock, s2_axi_arburst, s2_axi_arsize,
                                        s2_axi_arlen, s2_axi_araddr, s2_axi_arid};

    assign s3_axi_arready = ar3_tx_ready;
    assign ar3_tx_valid   = s3_axi_arvalid;
    assign ar3_tx_bus     = hipi_sel ? {s3_axi_aruser, s3_axi_arregion, s3_axi_arqos, 1'b0, s3_axi_arprot,
                                        s3_axi_arcache, s3_axi_arlock, 1'b0, s3_axi_arburst, s3_axi_arsize,
                                        s3_axi_arlen, s3_axi_araddr, s3_axi_arid[7:0]}
                                     : {s3_axi_aruser, s3_axi_arregion, s3_axi_arqos, s3_axi_arprot,
                                        s3_axi_arcache, s3_axi_arlock, s3_axi_arburst, s3_axi_arsize,
                                        s3_axi_arlen, s3_axi_araddr, s3_axi_arid};

    // W 通道（w_lite = &wstrb，置于最高 bit）
    assign s0_axi_wready = w0_tx_ready;
    assign w0_tx_valid   = s0_axi_wvalid;
    assign w0_lite       = &s0_axi_wstrb;
    assign w0_tx_bus     = {s0_axi_wuser, s0_axi_wlast, s0_axi_wstrb, s0_axi_wdata};

    assign s1_axi_wready = w1_tx_ready;
    assign w1_tx_valid   = s1_axi_wvalid;
    assign w1_lite       = &s1_axi_wstrb;
    assign w1_tx_bus     = {s1_axi_wuser, s1_axi_wlast, s1_axi_wstrb, s1_axi_wdata};

    assign s2_axi_wready = w2_tx_ready;
    assign w2_tx_valid   = s2_axi_wvalid;
    assign w2_lite       = &s2_axi_wstrb;
    assign w2_tx_bus     = {s2_axi_wuser, s2_axi_wlast, s2_axi_wstrb, s2_axi_wdata};

    assign s3_axi_wready = w3_tx_ready;
    assign w3_tx_valid   = s3_axi_wvalid;
    assign w3_lite       = &s3_axi_wstrb;
    assign w3_tx_bus     = {s3_axi_wuser, s3_axi_wlast, s3_axi_wstrb, s3_axi_wdata};

    // R 通道（r_lite = last_valid & id 匹配 & resp 匹配；置于最高 bit）
    assign m0_axi_rready = r0_tx_ready;
    assign r0_tx_valid   = m0_axi_rvalid;
    assign r0_hsk        = m0_axi_rvalid & m0_axi_rready;
    assign r0_lite       = last_valid_r0 & (m0_axi_rid == last_rid_r0) & (m0_axi_rresp == last_rresp_r0);
    assign r0_tx_bus     = hipi_sel ? {2'd0, m0_axi_ruser, m0_axi_rlast, m0_axi_rid[7:0], m0_axi_rresp, m0_axi_rdata} : 
                                      {m0_axi_ruser, m0_axi_rlast, m0_axi_rid, m0_axi_rresp, m0_axi_rdata};

    assign m1_axi_rready = r1_tx_ready;
    assign r1_tx_valid   = m1_axi_rvalid;
    assign r1_hsk        = m1_axi_rvalid & m1_axi_rready;
    assign r1_lite       = last_valid_r1 & (m1_axi_rid == last_rid_r1) & (m1_axi_rresp == last_rresp_r1);
    assign r1_tx_bus     = hipi_sel ? {2'd0, m1_axi_ruser, m1_axi_rlast, m1_axi_rid[7:0], m1_axi_rresp, m1_axi_rdata} :
                                      {m1_axi_ruser, m1_axi_rlast, m1_axi_rid, m1_axi_rresp, m1_axi_rdata};

    assign m2_axi_rready = r2_tx_ready;
    assign r2_tx_valid   = m2_axi_rvalid;
    assign r2_hsk        = m2_axi_rvalid & m2_axi_rready;
    assign r2_lite       = last_valid_r2 & (m2_axi_rid == last_rid_r2) & (m2_axi_rresp == last_rresp_r2);
    assign r2_tx_bus     = hipi_sel ? {2'd0, m2_axi_ruser, m2_axi_rlast, m2_axi_rid[7:0], m2_axi_rresp, m2_axi_rdata} :
                                      {m2_axi_ruser, m2_axi_rlast, m2_axi_rid, m2_axi_rresp, m2_axi_rdata};

    assign m3_axi_rready = r3_tx_ready;
    assign r3_tx_valid   = m3_axi_rvalid;
    assign r3_hsk        = m3_axi_rvalid & m3_axi_rready;
    assign r3_lite       = last_valid_r3 & (m3_axi_rid == last_rid_r3) & (m3_axi_rresp == last_rresp_r3);
    assign r3_tx_bus     = hipi_sel ? {2'd0, m3_axi_ruser, m3_axi_rlast, m3_axi_rid[7:0], m3_axi_rresp, m3_axi_rdata} :
                                      {m3_axi_ruser, m3_axi_rlast, m3_axi_rid, m3_axi_rresp, m3_axi_rdata};

    // B 通道
    assign m0_axi_bready = b0_tx_ready;
    assign b0_tx_valid   = m0_axi_bvalid;
    assign b0_tx_bus     = hipi_sel ? {2'd0, m0_axi_buser, m0_axi_bresp, m0_axi_bid[7:0]} :
                                      {m0_axi_buser, m0_axi_bresp, m0_axi_bid};

    assign m1_axi_bready = b1_tx_ready;
    assign b1_tx_valid   = m1_axi_bvalid;
    assign b1_tx_bus     = hipi_sel ? {2'd0, m1_axi_buser, m1_axi_bresp, m1_axi_bid[7:0]} :
                                      {m1_axi_buser, m1_axi_bresp, m1_axi_bid};

    assign m2_axi_bready = b2_tx_ready;
    assign b2_tx_valid   = m2_axi_bvalid;
    assign b2_tx_bus     = hipi_sel ? {2'd0, m2_axi_buser, m2_axi_bresp, m2_axi_bid[7:0]} :
                                      {m2_axi_buser, m2_axi_bresp, m2_axi_bid};

    assign m3_axi_bready = b3_tx_ready;
    assign b3_tx_valid   = m3_axi_bvalid;
    assign b3_tx_bus     = hipi_sel ? {2'd0, m3_axi_buser, m3_axi_bresp, m3_axi_bid[7:0]} :
                                      {m3_axi_buser, m3_axi_bresp, m3_axi_bid};

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================
    // R 通道 last_* 寄存器（rvalid & rready 时更新；每寄存器独立 always 块）
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            last_valid_r0 <= 1'b0;
        end
        else if (r0_hsk) begin
            last_valid_r0 <= 1'b1;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            last_rid_r0 <= 10'd0;
        end
        else if (r0_hsk) begin
            last_rid_r0 <= m0_axi_rid;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            last_rresp_r0 <= 2'd0;
        end
        else if (r0_hsk) begin
            last_rresp_r0 <= m0_axi_rresp;
        end
    end

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            last_valid_r1 <= 1'b0;
        end
        else if (r1_hsk) begin
            last_valid_r1 <= 1'b1;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            last_rid_r1 <= 10'd0;
        end
        else if (r1_hsk) begin
            last_rid_r1 <= m1_axi_rid;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            last_rresp_r1 <= 2'd0;
        end
        else if (r1_hsk) begin
            last_rresp_r1 <= m1_axi_rresp;
        end
    end

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            last_valid_r2 <= 1'b0;
        end
        else if (r2_hsk) begin
            last_valid_r2 <= 1'b1;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            last_rid_r2 <= 10'd0;
        end
        else if (r2_hsk) begin
            last_rid_r2 <= m2_axi_rid;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            last_rresp_r2 <= 2'd0;
        end
        else if (r2_hsk) begin
            last_rresp_r2 <= m2_axi_rresp;
        end
    end

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            last_valid_r3 <= 1'b0;
        end
        else if (r3_hsk) begin
            last_valid_r3 <= 1'b1;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            last_rid_r3 <= 10'd0;
        end
        else if (r3_hsk) begin
            last_rid_r3 <= m3_axi_rid;
        end
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            last_rresp_r3 <= 2'd0;
        end
        else if (r3_hsk) begin
            last_rresp_r3 <= m3_axi_rresp;
        end
    end

endmodule
