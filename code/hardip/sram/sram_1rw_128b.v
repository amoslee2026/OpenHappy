// =============================================================================
// module name：sram_1rw_128b
// author: OpenHappy Team
// MAS：spec/link_layer/submods/link_tx/mas/replay_buffer_mas.md §6.1（SRAM 物理实现）
// =============================================================================
// 功能概述: 128 bit 宽、深度可配置的 1RW（单口读写）SRAM 行为模型，
//   用于替代 replay_buffer 中原本直接例化的厂商 SRAM 硬宏，
//   使 D2D 源码可作为源码交付。
//
//   端口语义（与原硬宏 _core 视图逐条对齐）：
//     - 时钟上升沿采样 req/we/addr
//     - req=1 && we=1：写 mem[addr] ← wdata；rdata 保持（写不更新输出）
//     - req=1 && we=0：读，rdata ← mem[addr]（1 拍读延迟）
//     - req=0        ：无操作
//
//   与硬宏差异（不影响功能，仅去掉 DFT/BIST/repair 引脚）：
//     - 硬宏 45 个端口中的 RSCOUT/ROP/RSCIN/RSCEN/RSCRST/RSCLK/FISO/TEST1/
//       TEST_RNM/RME/RM/CD/CAPT/PIPEME/TPIPEME/TCLKE/STICKY/SI_*/SE_*/
//       DFTCLKEN/DFTMASK/TADR/TD/TWE/TME/TCLK/BISTE/LS/DS/SD/BC1/BC2 全部删除
//       —— 原例化处这些引脚均 tie 0（DFT/BIST 未使用），行为等价。
//     - 不实现 repair（repair 使能恒 0）、不实现 light-sleep（LS 恒 0）。
//     - 无 specify 块：不引入零延迟时序检查，仿真不需要 +define+functional。
//
//   容量对应关系（replay_buffer 用法）：
//     REPLAY_BUF_DEPTH_16  → 8  片 × DEPTH=32  （每层 1024bit = 8×128）
//     REPLAY_BUF_DEPTH_32  → 16 片 × DEPTH=32
//     REPLAY_BUF_DEPTH_64  → 16 片 × DEPTH=64
//     REPLAY_BUF_DEPTH_128 → 16 片 × DEPTH=128
// =============================================================================
// version history
//   2026-10-26  OpenHappy Team  初始版本：自主实现替代厂商 SRAM 硬宏（D2D 对称版）
// =============================================================================

`timescale 1ns/1ps

module sram_1rw_128b #(
    parameter DEPTH  = 32,          // 存储深度（字数），必须为 2 的幂
    parameter ADDR_W = 5            // 地址位宽 = $clog2(DEPTH)
) (
    input  wire              clk,   // 时钟（上升沿采样）
    input  wire              req,   // 访问请求，高有效
    input  wire              we,    // 1=写 / 0=读
    input  wire [ADDR_W-1:0] addr,  // 地址
    input  wire [127:0]      wdata, // 写数据
    output reg  [127:0]      rdata  // 读数据（1 拍延迟）
);

    // -------------------------------------------------------------------------
    // 存储阵列
    // -------------------------------------------------------------------------
    reg [127:0] mem [0:DEPTH-1];

    // -------------------------------------------------------------------------
    // 读写行为（写优先判定：同拍 we=1 时只写不读，与硬宏一致）
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (req && we) begin
            mem[addr] <= wdata;         // 写：rdata 保持上一次读值
        end
        else if (req && !we) begin
            rdata <= mem[addr];         // 读：下一拍输出
        end
    end

endmodule
