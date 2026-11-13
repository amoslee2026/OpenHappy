// -------------------------------------------------------------------------------
// File   : pcsr.v
// Author : OpenHappy Team
// Date   : 2026-08-07
// -------------------------------------------------------------------------------
// Description:
//   Controller Public CSR (PROTOCOL_PUB) shell. APB3 slave (32bit data, 12bit addr),
//   public segment base 0x0000, 2 RW registers per spec/controller_pub_csr/
//   controller_pub_csr.md:
//     0x00 rstn             : global reset control
//     0x04 clk_en           : AXI channel clock enable (tx/rx each 8 lanes)
//   APB 读写通路：地址解码（public 段 0x000~0x3ff + 2 个寄存器偏移命中）→ 写通路
//   （apb_wr && 命中 → pwdata 写寄存器）/ 读通路（组合 case mux → prdata）；
//   pready / pslverr 组合输出（未映射地址报 pslverr）。控制逻辑（reset gating /
//   clock gating）仍为 TODO，端口从 spec field map 移植。
// -------------------------------------------------------------------------------
// Change Log:
//   2026-08-07  OpenHappy Team  v0.1  Shell generation from controller_pub_csr.md
//   2026-08-07  OpenHappy Team  v0.2  Add sb_rstn output (sideband reset, = rstn_reg[0])
//   2026-08-13  OpenHappy Team  v0.3  Fix assign-to-reg：删除 prdata_reg / pready_reg 冗余寄存器，
//                                prdata / pready 端口直接作寄存器（always 驱动），消除连续赋值驱动 reg
//   2026-08-13  OpenHappy Team  v0.4  补齐 APB 读写通路（仿 cxs_prcl_csr 风格）：地址解码
//                                （0x000~0x3ff 段 + 5 寄存器命中）、写通路（每寄存器独立 always）、
//                                读 mux（组合 case）、pslverr（未映射地址报错）/ pready（组合）
//   2026-08-14  OpenHappy Team  v0.5  删除 loopback 寄存器：protocol_loopback（0x0c）/ link_loopback（0x10）
//                                local/remote/link_loopback 输出端口（loopback 由协议层 CSR 本地控制）
//   2026-08-14  OpenHappy Team  v0.6  删除 credit_en 寄存器（0x08）+ 输出端口（PAIF credit 使能改由
//                                CXS 协议层 CSR 本地控制；AXI 协议层 credit_en 输入由 top tie1）
//   2026-08-25  OpenHappy Team  v0.7  clk_en 寄存器扩到 16bit：tx_clken[7:0]/rx_clken[7:0]
//                                （对 spec/controller_pub_csr 各 8 lane）
//   2026-09-10  OpenHappy Team  v0.8  spec 对齐：spec/controller_pub_csr 表格更新为
//                                rstn 实际使用 2 bit（bit0 mb_rstn / bit1 sb_rstn，v0.2 注释
//                                “sb_rstn = rstn_reg[0]”为笔误，代码一直取 bit1）；本文件逻辑不变
// -------------------------------------------------------------------------------

`default_nettype none

module pcsr
(
    // --- Clock and reset ---
    input  wire                                   pclk              ,
    input  wire                                   presetn           ,

    // --- APB interface (pclk 域) ---
    input  wire                                   psel              ,
    input  wire                                   penable           ,
    input  wire [11:0]                            paddr             ,
    input  wire                                   pwrite            ,
    input  wire [31:0]                            pwdata            ,
    output reg  [31:0]                            prdata            , // 组合读 mux
    output wire                                   pready            ,
    output wire                                   pslverr           ,

    // --- Register outputs (→ 外部逻辑) ---
    output wire                                   mb_rstn           , // rstn.rstn
    output wire                                   sb_rstn           , // rstn.rstn（sideband 复位）
    output wire [7 : 0]                           tx_clken          , // clk_en.tx_clken
    output wire [7 : 0]                           rx_clken           // clk_en.rx_clken
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    // 寄存器地址（spec/controller_pub_csr，基址 0x0000，public 段 0x000~0x3ff）
    localparam ADDR_RSTN              = 12'h000;
    localparam ADDR_CLK_EN            = 12'h004;
    // 寄存器复位值（spec 默认值均为 0x0）
    localparam [31:0] RSTN_RST              = 32'h0;
    localparam [31:0] CLK_EN_RST            = 32'h0;

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    // APB 相位组合信号
    wire apb_access;
    wire apb_wr    ;
    // 地址解码（pcsr public 段 0x000~0x3ff）
    wire is_pcsr_addr      ;
    wire is_rstn_addr      ;
    wire is_clk_en_addr    ;
    wire addr_implemented  ;

    // RW 寄存器
    reg [31:0] rstn_reg             ; // 0x00，bit0 rstn
    reg [31:0] clk_en_reg           ; // 0x04，bit15:8 rx_clken，bit7:0 tx_clken

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================
    // APB 相位解码
    assign apb_access = psel & penable;
    assign apb_wr     = apb_access & pwrite;

    // 地址解码：pcsr public 段 0x000~0x3ff（判断是否访问 pcsr）+ 5 个寄存器偏移命中
    assign is_pcsr_addr        = (paddr[11:10] == 2'b00);
    assign is_rstn_addr        = is_pcsr_addr && (paddr == ADDR_RSTN);
    assign is_clk_en_addr      = is_pcsr_addr && (paddr == ADDR_CLK_EN);
    assign addr_implemented    = is_rstn_addr || is_clk_en_addr;

    // pready / pslverr（组合，仿 cxs_prcl_csr：段外或段内未映射地址报 APB 错误）
    assign pready  = apb_access && addr_implemented;
    assign pslverr = apb_access && !addr_implemented;

    // 寄存器字段 -> 具名输出（位映射与 spec/controller_pub_csr 一致：rstn bit0/1 = mb/sb_rstn）
    assign mb_rstn        = rstn_reg[0];
    assign sb_rstn        = rstn_reg[1]; // sideband 复位（spec rstn bit1）
    assign tx_clken       = clk_en_reg[7:0];
    assign rx_clken       = clk_en_reg[15:8];

    // ==========================================================================
    // 时序逻辑 —— 寄存器写通路（每寄存器独立 always，1 信号/块；
    //                 apb_wr 且命中偏移时写入 pwdata，未命中保持原值）
    // ==========================================================================
    // rstn_reg（0x00，bit0 rstn）
    always @(posedge pclk or negedge presetn) begin
        if (~presetn) begin
            rstn_reg <= RSTN_RST;
        end else if (apb_wr && is_rstn_addr) begin
            rstn_reg <= pwdata;
        end
    end

    // clk_en_reg（0x04，bit15:8 rx_clken / bit7:0 tx_clken）
    always @(posedge pclk or negedge presetn) begin
        if (~presetn) begin
            clk_en_reg <= CLK_EN_RST;
        end else if (apb_wr && is_clk_en_addr) begin
            clk_en_reg <= pwdata;
        end
    end

    // ==========================================================================
    // 组合逻辑 —— 读通路
    // ==========================================================================
    // prdata 读 mux（组合，仿 cxs_prcl_csr `case (addr) → rd_din`；未映射偏移读 0）
    always @(*) begin
        case (paddr[11:0])
            ADDR_RSTN:              prdata = rstn_reg;
            ADDR_CLK_EN:            prdata = clk_en_reg;
            default:                prdata = 32'h0;
        endcase
    end

endmodule

`default_nettype wire
