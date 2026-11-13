// =============================================================================
// module name：crc9_check
// author: OpenHappy Team
// MAS：spec/link_layer/submods/link_rx/mas/crc9_check_mas.md
// 功能概述: 兼容模式下 RX 侧 32 B Flit 的 CRC9 校验单元。对 Flit Header
//           （5 bit）+ Payload（256 bit）共 261 bit 数据按多项式
//           x^9 + x^5 + x^4 + x + 1（0x119，Init=0x1FF，无输入/输出反转，
//           XorOut=0x000）重算 CRC9，与 Flit Tail 携带的 CHK 字段
//           （flit_i[269:261]）比对。校验失败置 err_o 脉冲，由父模块
//           link_rx 触发错误丢弃与 NACK 重传请求并聚合驱动
//           CRC_ERR_CNT_REG.CRC0_ERR_CNT 计数；CRC_EN=0 或非兼容模式时
//           跳过校验直接 pass。本模块为纯组合逻辑，输入数据稳定后
//           pass/err 同拍组合有效，无寄存器延迟；CRC 计算引擎直接例化
//           crc9_generate（与 TX 侧同一并行 XOR 矩阵，禁止重复展开）。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   <日期>  <修订人>  <变更概述>
//  2026-08-08  OpenHappy Team  初始版本（对齐 MAS v1.0）
// =============================================================================

`timescale 1ns/1ps

module crc9_check (
    // 时钟与复位（保留端口，纯组合模块内部不使用）
    input  wire               mb_clk,
    input  wire               mb_rst_n,

    // 配置（apb_csr 具名直送，经 link_rx 透传）
    input  wire               csr_comp_mode_i,     // 兼容模式指示，高有效
    input  wire               csr_crc_en_i,        // CRC 校验使能（RETRY_CFG_REG.CRC_EN）

    // 数据输入（完整 32 B Flit）
    // bit[4:0]=Header / bit[260:5]=Payload / bit[269:261]=CHK / bit[271:270]=Rsv
    input  wire [271:0]       flit_i,
    input  wire               flit_valid_i,    // Flit 有效，校验拍拉高 1 cycle

    // 校验结果输出（与 flit_valid_i 同拍组合，pass_o / err_o 互斥）
    output wire               pass_o,          // 校验通过指示
    output wire               err_o,           // 校验失败脉冲
    output wire [8:0]         crc9_calc_o      // 实际计算的 CRC9（DFx 观测）
);

// =============================================================================
// 信号声明
// =============================================================================
wire [8:0]  crc9_calc_w;    // CRC9 计算结果（Header + Payload 共 261 bit）
wire        err_comb;       // 校验失败组合指示（未含 valid 门控）

// =============================================================================
// CRC9 计算引擎（例化 crc9_generate：poly=9'h119, Init=9'h1FF，
// 并行 XOR 矩阵展开，纯组合 0 cycle；与 TX 侧同款，不重复展开矩阵）
// =============================================================================
crc9_generate u_crc9_generate (
    .mb_clk      (mb_clk),
    .mb_rst_n    (mb_rst_n),
    .crc9_data_i (flit_i[260:0]),
    .crc9_o      (crc9_calc_w)
);

// =============================================================================
// 组合逻辑：CHK 字段比对 + 使能门控 + valid 门控
// -----------------------------------------------------------------------------
// err_comb：仅在兼容模式且 CRC 使能时参与比对；
// CRC_EN=0 或 csr_comp_mode_i=0 时跳过校验，pass_o=flit_valid_i、err_o=0
// （link_layer_spec.md §4.3.3.3 / §5.3）
// =============================================================================
assign err_comb    = csr_crc_en_i & csr_comp_mode_i & (crc9_calc_w != flit_i[269:261]);

assign err_o       = flit_valid_i & err_comb;
assign pass_o      = flit_valid_i & (~err_comb | ~csr_crc_en_i | ~csr_comp_mode_i);
assign crc9_calc_o = crc9_calc_w;

endmodule
