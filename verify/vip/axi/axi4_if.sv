// ===================================================================
// axi4_if.sv — AXI4 接口（D2D HiPi 用户侧 AXI，s0/m0 对称骨架）
//
// 版本：v1.0 | 日期：2026-10-19 | 作者：OpenHappy Team
//
// 用途：DUT（d2d_hipi_dig_top）对称 AXI 用户口的 TB 侧接口容器。
//   1T1R 对称模式下 DUT 有两个 AXI 口：
//     s0_axi —— DUT 内部作为 AXI Slave（AW/AR/W 输入，R/B 输出）
//               ← 接 axi_master_vip（VIP 作为 AXI Master 发请求）
//     m0_axi —— DUT 内部作为 AXI Master（AW/AR/W 输出，R/B 输入）
//               → 接 axi_slave_vip（VIP 作为 AXI Slave 回响应）
//   两个口信号名完全一致（仅前缀不同），故本 interface 一份参数化位宽即可复用两份实例。
//
// 位宽来源：d2d_hipi_tb_defines.vh（与 RTL AXI_DATA_WIDTH=512 等一致）
// modport：
//   master —— VIP 作为 AXI Master（驱动 AW/AR/W，采样 R/B）；接 DUT 的 s0_axi
//   slave  —— VIP 作为 AXI Slave （采样 AW/AR/W，驱动 R/B）；接 DUT 的 m0_axi
//   monitor—— 只读采样（scoreboard 用），无驱动
// ===================================================================

`ifndef AXI4_IF_SV
`define AXI4_IF_SV

interface axi4_if #(
  parameter int ID_W   = `D2D_TB_AXI_ID_WIDTH,
  parameter int ADDR_W = `D2D_TB_AXI_ADDR_WIDTH,
  parameter int DATA_W = `D2D_TB_AXI_DATA_WIDTH,
  parameter int STRB_W = `D2D_TB_AXI_STRB_WIDTH,
  parameter int LEN_W  = `D2D_TB_AXI_LEN_WIDTH,
  parameter int SIZE_W = `D2D_TB_AXI_SIZE_WIDTH,
  parameter int BURST_W= `D2D_TB_AXI_BURST_WIDTH,
  parameter int CACHE_W= `D2D_TB_AXI_CACHE_WIDTH,
  parameter int PROT_W = `D2D_TB_AXI_PROT_WIDTH,
  parameter int QOS_W  = `D2D_TB_AXI_QOS_WIDTH,
  parameter int RGN_W  = `D2D_TB_AXI_REGION_WIDTH,
  parameter int USER_W = `D2D_TB_AXI_USER_WIDTH
)(
  input logic aclk,
  input logic aresetn
);

  // ---- AW 通道 ----
  logic [ID_W-1:0]    awid;
  logic [ADDR_W-1:0]  awaddr;
  logic [LEN_W-1:0]   awlen;
  logic [SIZE_W-1:0]  awsize;
  logic [BURST_W-1:0] awburst;
  logic               awlock;
  logic [CACHE_W-1:0] awcache;
  logic [PROT_W-1:0]  awprot;
  logic [QOS_W-1:0]   awqos;
  logic [RGN_W-1:0]   awregion;
  logic [USER_W-1:0]  awuser;
  logic               awvalid;
  logic               awready;

  // ---- W 通道 ----
  logic [DATA_W-1:0]  wdata;
  logic [STRB_W-1:0]  wstrb;
  logic [USER_W-1:0]  wuser;
  logic               wlast;
  logic               wvalid;
  logic               wready;

  // ---- B 通道 ----
  logic [ID_W-1:0]    bid;
  logic [1:0]         bresp;
  logic [USER_W-1:0]  buser;
  logic               bvalid;
  logic               bready;

  // ---- AR 通道 ----
  logic [ID_W-1:0]    arid;
  logic [ADDR_W-1:0]  araddr;
  logic [LEN_W-1:0]   arlen;
  logic [SIZE_W-1:0]  arsize;
  logic [BURST_W-1:0] arburst;
  logic               arlock;
  logic [CACHE_W-1:0] arcache;
  logic [PROT_W-1:0]  arprot;
  logic [QOS_W-1:0]   arqos;
  logic [RGN_W-1:0]   arregion;
  logic [USER_W-1:0]  aruser;
  logic               arvalid;
  logic               arready;

  // ---- R 通道 ----
  logic [ID_W-1:0]    rid;
  logic [DATA_W-1:0]  rdata;
  logic [1:0]         rresp;
  logic [USER_W-1:0]  ruser;
  logic               rlast;
  logic               rvalid;
  logic               rready;

  // ---------------------------------------------------------------
  // modport：master（VIP 主动发请求）
  // ---------------------------------------------------------------
  modport master (
    input  aclk, aresetn,
    output awid, awaddr, awlen, awsize, awburst, awlock, awcache, awprot,
           awqos, awregion, awuser, awvalid,
    input  awready,
    output wdata, wstrb, wuser, wlast, wvalid,
    input  wready,
    input  bid, bresp, buser, bvalid,
    output bready,
    output arid, araddr, arlen, arsize, arburst, arlock, arcache, arprot,
           arqos, arregion, aruser, arvalid,
    input  arready,
    input  rid, rdata, rresp, ruser, rlast, rvalid,
    output rready
  );

  // ---------------------------------------------------------------
  // modport：slave（VIP 被动回响应）
  // ---------------------------------------------------------------
  modport slave (
    input  aclk, aresetn,
    input  awid, awaddr, awlen, awsize, awburst, awlock, awcache, awprot,
           awqos, awregion, awuser, awvalid,
    output awready,
    input  wdata, wstrb, wuser, wlast, wvalid,
    output wready,
    output bid, bresp, buser, bvalid,
    input  bready,
    input  arid, araddr, arlen, arsize, arburst, arlock, arcache, arprot,
           arqos, arregion, aruser, arvalid,
    output arready,
    output rid, rdata, rresp, ruser, rlast, rvalid,
    input  rready
  );

  // ---------------------------------------------------------------
  // modport：monitor（只读采样，scoreboard 用）
  // ---------------------------------------------------------------
  modport monitor (
    input aclk, aresetn,
    input awid, awaddr, awlen, awsize, awburst, awlock, awcache, awprot,
          awqos, awregion, awuser, awvalid, awready,
    input wdata, wstrb, wuser, wlast, wvalid, wready,
    input bid, bresp, buser, bvalid, bready,
    input arid, araddr, arlen, arsize, arburst, arlock, arcache, arprot,
          arqos, arregion, aruser, arvalid, arready,
    input rid, rdata, rresp, ruser, rlast, rvalid, rready
  );

endinterface

`endif  // AXI4_IF_SV
