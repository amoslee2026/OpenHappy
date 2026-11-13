// ===================================================================
// apb_if.sv — APB3 接口（D2D HiPi hipi_apb_* slave 端口）
//
// 版本：v1.1 | 日期：2026-10-19 | 作者：OpenHappy Team
// 说明：v1.1 从 cxs_prcl/dv/vip/apb/apb_if.sv 复用，按 D2D HiPi 需要调整：
//       ADDR_WIDTH 16 → 20（DUT hipi_apb_paddr[19:0]，bit16 为 phy/ctrl 域选择）。
//       其余（两相协议 SETUP→ACCESS，32bit 数据）不变。
// ===================================================================

`ifndef APB_IF_SV
`define APB_IF_SV

interface apb_if #(
  parameter int ADDR_WIDTH = 20,
  parameter int DATA_WIDTH = 32
)(
  input logic pclk,
  input logic presetn
);

  logic                  psel;
  logic                  penable;
  logic [ADDR_WIDTH-1:0] paddr;
  logic                  pwrite;
  logic [DATA_WIDTH-1:0] pwdata;
  logic [DATA_WIDTH-1:0] prdata;
  logic                  pready;
  logic                  pslverr;

  modport master (
    input  pclk, presetn,
    output psel, penable, paddr, pwrite, pwdata,
    input  prdata, pready, pslverr
  );

  modport slave (
    input  pclk, presetn,
    input  psel, penable, paddr, pwrite, pwdata,
    output prdata, pready, pslverr
  );

endinterface

`endif  // APB_IF_SV
