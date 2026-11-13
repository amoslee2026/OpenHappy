// =============================================================================
// module name：mcu_subsys
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/logic_phy_spec.md（§3.8 MCU 子系统规格；黑盒无 MAS）
// =============================================================================
// 功能概述: MCU 子系统（RISC-V core wrapper，per-link 全局唯一）端口 stub（black-box），
//   仅端口骨架 + 安全常值输出，功能由 MCU 子系统责任人补充
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-07  OpenHappy Team  初始版本：端口 stub（black-box），功能由 MCU 子系统责任人补充
// =============================================================================

`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// 设计说明:
//
// 端口推导依据（logic_phy_spec.json internal_connections + spec.md §3.8/§5.6）:
//   - cfg_i[31:0]            ← apb_csr_top.cfg_mcu_o（APB-broadcast，黑盒长期保留）
//   - irq_i                  ← intc.irq_mcu_o（聚合中断，高有效，level）
//   - ahbs_*                 ↔ top_logic_phy_intf v1.2 AHB slave（firmware 加载 /
//                              各层 CSR 配置 / MCU debug）
//   - apb_csr_*  master      → apb_csr_top APB slave（logic_phy 内部 CSR）
//   - apb_pl_*   master      → 顶端口 apb_*（协议层+链路层 CSR 合并空间，
//                              见 link_logicphy_intf v1.4 §APB 配置接口，paddr 20 bit）
//   - apb_pclk / apb_preset_n：2 路 APB master 均工作于 apb_pclk 域
//                              （与 mcu_clk 同源同步，见 spec §3.2/§3.8）
//
// TODO（责任人补充）:
//   - RISC-V core / DM / TCM / AXI2AHB / AHB Matrix / AHB2APB 全部内部逻辑
//   - JTAG debug 接口（待 MCU 选型确定后补端口，同步修订 top_logic_phy_intf）
//   - AHB 协议类型确认（AHB-Lite / AHB，见 spec §5.6 遗留项）
//   - cfg_i 解码（MCU_CTRL：mcu_soft_rst_n / boot_addr / itcm_wr_en）
//   - irq_i 接入 core 中断控制器
// -----------------------------------------------------------------------------

module mcu_subsys (
    // 时钟与复位
    input  wire         mcu_clk,            // MCU 工作时钟（与外部总线配置时钟同步）
    input  wire         mcu_rst_n,          // MCU 异步复位，低有效（含软复位，已同步至 mcu_clk）
    input  wire         apb_pclk,           // APB 配置时钟（与 mcu_clk 同源同步）
    input  wire         apb_preset_n,       // APB 域复位，低有效（顶层层接 mcu_rst_n）

    // AHB slave 接口（外部 master 经此访问 MCU 内存 / 各层 CSR / debug，见 §3.8）
    input  wire         ahbs_hsel,
    input  wire [31:0]  ahbs_haddr,
    input  wire [2:0]   ahbs_hburst,
    input  wire [2:0]   ahbs_hsize,
    input  wire [3:0]   ahbs_hprot,
    input  wire [31:0]  ahbs_hwdata,
    input  wire         ahbs_hwrite,
    input  wire [1:0]   ahbs_htrans,
    input  wire         ahbs_hready,
    output wire [31:0]  ahbs_hrdata,
    output wire [1:0]   ahbs_hresp,
    output wire         ahbs_hready_resp,

    // cfg broadcast（apb_csr_top.cfg_mcu_o，黑盒内部解码）
    input  wire [31:0]  cfg_i,

    // 聚合中断输入（intc.irq_mcu_o，高有效 level）
    input  wire         irq_i,

    // APB master 0：→ logic_phy 内部 apb_csr_top（物理层 CSR，paddr 16 bit）
    output wire         apb_csr_psel,
    output wire         apb_csr_penable,
    output wire         apb_csr_pwrite,
    output wire [15:0]  apb_csr_paddr,
    output wire [31:0]  apb_csr_pwdata,
    input  wire [31:0]  apb_csr_prdata,
    input  wire         apb_csr_pready,
    input  wire         apb_csr_pslverr,

    // APB master 1：→ 协议层 + 链路层 CSR 合并空间（出顶，paddr 20 bit）
    output wire         apb_pl_psel,
    output wire         apb_pl_penable,
    output wire         apb_pl_pwrite,
    output wire [19:0]  apb_pl_paddr,
    output wire [31:0]  apb_pl_pwdata,
    input  wire [31:0]  apb_pl_prdata,
    input  wire         apb_pl_pready,
    input  wire         apb_pl_pslverr
);

    // ==========================================================================
    // 输出驱动（black-box stub 安全常值，功能待 MCU 子系统责任人实现）
    // ==========================================================================
    // AHB slave 响应：hready_resp 常 1（不挂死总线）、hresp=OKAY、hrdata 常 0
    // TODO: 接 AHB Matrix slave 响应逻辑
    assign ahbs_hrdata      = 32'h0000_0000;
    assign ahbs_hresp       = 2'b00;                // OKAY
    assign ahbs_hready_resp = 1'b1;

    // APB master 0（→ apb_csr_top）：psel/penable 常 0，不发起传输
    // TODO: 接 AHB2APB 桥输出
    assign apb_csr_psel    = 1'b0;
    assign apb_csr_penable = 1'b0;
    assign apb_csr_pwrite  = 1'b0;
    assign apb_csr_paddr   = 16'h0000;
    assign apb_csr_pwdata  = 32'h0000_0000;

    // APB master 1（→ 协议层+链路层 CSR 出顶）：psel/penable 常 0，不发起传输
    // TODO: 接 AHB2APB 桥输出
    assign apb_pl_psel    = 1'b0;
    assign apb_pl_penable = 1'b0;
    assign apb_pl_pwrite  = 1'b0;
    assign apb_pl_paddr   = 20'h0_0000;
    assign apb_pl_pwdata  = 32'h0000_0000;

endmodule
