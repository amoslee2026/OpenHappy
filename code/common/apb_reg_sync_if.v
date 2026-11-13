//
//====================================================================
// File: apb_reg_sync_if
// Last Modify By: OpenHappy Team
// Revision: 0.2
// Date:2026.9.8
//
//=====================================================================
// Description:
//   APB3 -> register-file 接口。由 register_flow 生成模板（link_layer_apb_reg_sync.v）
//   例化。原 `cache_apb_reg_sync_if`（rregs 宏实例）改写为行为 RTL：模块改名为
//   `apb_reg_sync_if`，5 路 rregs（无复位 D 触发器）以 always @(posedge pclk) q<=d
//   实现，行为完全等价。pready 为注册脉冲（1 等待态，兼容 2 拍时序）：
//     reg_rd  = psel & ~pwrite（打拍）
//     reg_wr  = psel & pwrite & penable（打拍）
//     reg_addr/wdata = apb_paddr/pwdata（打拍）
//     apb_pready = 注册脉冲 (psel & penable)（异步复位清零）
//     apb_prdata = reg_rdata（打拍回读）
//=====================================================================
module apb_reg_sync_if #(
    parameter  REG_AW = 16
)(
    input  wire                 pclk,
    input  wire                 preset_n,
    output wire                 rf_clk,
    input  wire                 apb_psel,
    input  wire                 apb_penable,
    input  wire                 apb_pwrite,
    input  wire [REG_AW-1:0]    apb_paddr,
    input  wire [31:0]          apb_pwdata,
    output reg                 apb_pready,
    output reg [31:0]          apb_prdata,

    output reg                  reg_rd,
    output reg                  reg_wr,
    output reg  [REG_AW-1:0]    reg_addr,
    output reg  [31:0]          reg_wdata,
    input  wire [31:0]          reg_rdata,

    input  wire                 test_mode
);

    assign  rf_clk = pclk;
    

    always @(posedge pclk or negedge preset_n)
        if (~preset_n)
         apb_pready<=1'b0;
        else 
         apb_pready<= apb_psel&apb_penable;

    always @(posedge pclk)
      apb_prdata <=reg_rdata;

    always @(posedge pclk)
        reg_addr   <= apb_paddr;

    always @(posedge pclk)
        reg_rd     <= apb_psel & (!apb_pwrite);

    always @(posedge pclk)
        reg_wr     <= apb_psel & apb_pwrite & apb_penable;

    always @(posedge pclk)
        reg_wdata  <= apb_pwdata;

endmodule
