// =============================================================================
// module name：crc16_check
// author: OpenHappy Team
// MAS：spec/link_layer/submods/link_rx/mas/crc16_check_mas.md
// 功能概述: 性能模式下 RX 侧 CRC16 校验单元，单引擎分时复用（128 B 单元
//           粒度）。每个 128 B 单元收齐即校验：unit0（前 128 B）比对单元内
//           携带的 CRC0 字段、unit1（后 128 B）比对同一相对位置携带的 CRC1
//           字段——CRC 字段在单元内相对位置一致（unit_i[1023:1008]，即
//           单元内 Byte 126~127），同一套比对逻辑覆盖两种单元，无需区分
//           单元序号。多项式 x^16 + x^15 + x^2 + 1（0x8005，Init=0xFFFF，
//           无输入/输出反转，XorOut=0x0000）；校验按 1008 bits 计算：重算
//           仅引用 unit_i[1007:0]（Byte 0~125），槽位 [1023:1008] 不被矩阵
//           引用（承载待比对字段）。校验失败置 err_o 脉冲，由父模块 link_rx
//           触发错误丢弃与 NACK 重传请求并聚合驱动 CRC 错误计数；CRC_EN=0
//           或非性能模式时跳过直接 pass。本模块为纯组合判定（v1.10 去 D23）E 拍
//           同拍直通：使能态 pass/err 于组包完成拍（E）同拍组合有效；禁用态
//           pass 同拍直通、err 恒 0；CRC 计算
//           引擎直接例化 crc16_generate（与 TX 侧同一并行 XOR 矩阵，禁止
//           重复展开）。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   <日期>  <修订人>  <变更概述>
//  2026-10-15  OpenHappy Team  LL-20261015-DUALPUSH（user 两路隔离 + 去 D23，MAS v1.10）：本模块为
//                         路 2（重传使能）校验引擎——删 D23 打 1 拍（crc_calc_p/crc_field_p/
//                         flit_valid_p 寄存），改 E 拍同拍组合直通
//                         （crc_eq_w=(crc_calc_w==unit_i[1023:1008])），pass/err 于组包完成拍
//                         （E）同拍回接；mb_clk/mb_rst_n 端口保留（同 crc9_check 先例）。
//  2026-08-08  OpenHappy Team  初始版本（对齐 MAS v1.0）
//  2026-08-08  OpenHappy Team  双引擎改单引擎分时复用（128 B 单元粒度）：
//                          flit_i(2048)+pass0/err0/pass1/err1 改为
//                          unit_i(1024)+pass_o/err_o；CRC 字段单元内
//                          相对位置一致（unit_i[1023:1008]），同一比对逻辑
//  2026-09-09  OpenHappy Team  D23：crc16_generate 输出（crc_calc_w）插 1 级可选
//                          流水寄存（crc_calc_p/crc_field_p/flit_valid_p），
//                          en_ck（CRC_EN&perf）时 pass/err 对齐到 E+1 拍、
//                          CRC_EN=0 仍 E 拍（en_ck 常量折叠分拍）。断 crc XOR
//                          深树主路径，使能态只剩 16bit 比较级。
//  2026-09-23  OpenHappy Team  文档化自包含槽位清零契约（crc_data_w 
//                          内部清零 [1023:1008]，现状已满足）；功能零变化
//  2026-09-23  OpenHappy Team  用户裁决（LL-20260923-CHK1008）：校验口径 1008 bits
//                          ——内嵌 u_crc16_generate 改直喂完整 unit_i（删除内部
//                          清零中间量 crc_data_w = {16'h0000, unit_i[1007:0]}）；
//                          校验码值随口径变化（d=0: 0x7fdf→0xd558，与 TX 同一
//                          新矩阵 → 链路自洽）；字段提取 / D23 打拍 / pass-err
//                          语义零改动；D23 时钟/复位语义文档化（O-2 注释收敛）
// =============================================================================

`timescale 1ns/1ps

module crc16_check (
    // 时钟与复位（v1.10 去 D23 后纯组合直通；端口保留，对齐 crc9_check 先例）
    input  wire               mb_clk,
    input  wire               mb_rst_n,

    // 配置（apb_csr 具名直送，经 link_rx 透传）
    input  wire               csr_perf_mode_i,     // 性能模式指示，高有效
    input  wire               csr_crc_en_i,        // CRC 校验总使能（RETRY_CFG_REG.CRC_EN）

    // 数据输入（1 个 128 B 单元）
    // CRC 字段固定位于 unit_i[1023:1008]（unit0 携带 CRC0 / unit1 携带 CRC1，
    // 折算到单元内同一位置，即单元内 Byte 126~127）；校验按 1008 bits 计算
    // （矩阵仅引用 unit_i[1007:0]，槽位承载待比对字段，不被矩阵引用）
    input  wire [1023:0]      unit_i,
    input  wire               flit_valid_i,    // 单元有效，校验拍拉高 1 cycle

    // 校验结果输出（v1.10 去 D23：使能态 E 拍同拍组合直通；禁用态 pass=E 拍直通，err=0）
    output wire               pass_o,          // 本单元校验通过指示
    output wire               err_o            // 本单元校验失败脉冲
);

// =============================================================================
// 信号声明
// =============================================================================
wire [15:0]   crc_calc_w;    // 计算结果（E 拍组合）
wire          crc_eq_w;      // 16 bit 计算结果与 CRC 字段相等（E 拍同拍 pass 判据）
wire          en_ck;         // CRC 校验使能合取（性能模式 & CRC_EN，静态 CSR）

// =============================================================================
// CRC16 单引擎（例化 crc16_generate：poly=16'h8005, Init=16'hFFFF，
// 并行 XOR 矩阵展开，纯组合 0 cycle；与 TX 侧同款，不重复展开矩阵；
// 1008-bit 口径：直喂完整 unit_i，矩阵仅引用 [1007:0]）
// =============================================================================

crc16_generate u_crc16_generate (
    .crc16_data_i (unit_i),               // 1024-bit 完整单元直喂；矩阵仅引用 [1007:0]（1008-bit 口径）
    .crc16_o      (crc_calc_w)
);

// =============================================================================
// 去 D23（v1.10，LL-20261015-DUALPUSH）：删 D23 打 1 拍（crc_calc_p/crc_field_p/
// flit_valid_p 寄存），E 拍同拍组合直通；mb_clk/mb_rst_n 端口保留（同 crc9_check 先例）。
// =============================================================================

// =============================================================================
// 组合逻辑：使能门控 + E 拍同拍比对 + valid 门控
// =============================================================================
assign en_ck      = csr_crc_en_i & csr_perf_mode_i;
assign crc_eq_w   = (crc_calc_w == unit_i[1023:1008]);

// 使能态（en_ck=1）：E 拍同拍判通；禁用态（en_ck=0）：E 拍直通 pass
assign pass_o     = (en_ck & flit_valid_i & crc_eq_w) | (~en_ck & flit_valid_i);
assign err_o      =  en_ck & flit_valid_i & ~crc_eq_w;

endmodule
