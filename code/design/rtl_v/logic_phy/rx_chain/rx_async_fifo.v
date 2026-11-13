// =============================================================================
// module name：rx_async_fifo
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/rx_chain/submods/rx_data_path/mas/rx_async_fifo_mas.md
// =============================================================================
// 功能概述: RX 数据通路 stage 6：异步 FIFO CDC（phy_rx_clk 写侧 → mb_clk 读侧），
//   depth 4 × 1024 bit，Gray 码指针 + 2flop 同步，overflow/underflow sticky 检测
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-06  OpenHappy Team  初始版本：基于 rx_async_fifo MAS v1.2 生成
// =============================================================================
//   2026-09-07  zhangna  CDC fix (Ac_glitch03 full_sync_mb1): full 组合状态先于
//                     phy_rx_clk 域打一拍成 full_q 寄存器源再跨域同步,消除组合 glitch
//   2026-10-22  OpenHappy Team  CDC fix (Ac_glitch03/Ac_conv04 rx_fifo_status_sync1[5]):
//                     empty 组合状态先于 mb_clk 域打一拍成 empty_q 寄存器源再跨域同步
//                     (与 full_q 反方向对称)，消除 multi-source toggling / gray FAILED
// =============================================================================

`timescale 1ns/1ps

module rx_async_fifo #(
    parameter FIFO_DEPTH = 4,
    parameter DATA_WIDTH = 1024,
    parameter PTR_WIDTH  = 3   // ceil(log2(FIFO_DEPTH)) + 1 = 3
) (
    input  wire                  phy_rx_clk,    // 写侧时钟
    input  wire                  mb_clk,        // 读侧时钟
    input  wire                  rx_rst_n,      // 写侧异步复位
    input  wire                  mb_rst_n,      // 读侧异步复位
    input  wire                  fifo_soft_reset, // 软复位脉冲（高有效）
    input  wire                  overflow_clear,  // overflow RW1C 软清除脉冲（mb_clk 域源，内部 2 flop 同步到 phy_rx_clk 域）
    input  wire                  underflow_clear, // underflow RW1C 软清除脉冲（mb_clk 域本地）
    input  wire  [DATA_WIDTH-1:0] data_i,        // 写侧数据输入（1024 bit）
    output wire  [DATA_WIDTH-1:0] data_o,        // 读侧数据输出（1024 bit）
    output wire  [3:0]           fifo_status    // {overflow, underflow, full, empty}
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    localparam [PTR_WIDTH-1:0] PTR_RESET = {PTR_WIDTH{1'b0}};

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    // 写侧（phy_rx_clk 域）
    reg  [PTR_WIDTH-1:0]  wptr_bin;
    reg  [PTR_WIDTH-1:0]  wptr_gray;
    wire [PTR_WIDTH-1:0]  wptr_bin_next;
    wire [PTR_WIDTH-1:0]  wptr_gray_next;
    wire                  full;
    wire                  write_en;       // 非满 + 非软复位时写使能

    // FIFO 寄存器阵列（4 entry × 1024 bit）
    reg  [DATA_WIDTH-1:0] fifo_reg [0:FIFO_DEPTH-1];

    // overflow sticky bit（phy_rx_clk 域）
    reg                   overflow;

    // 读侧（mb_clk 域）
    reg  [PTR_WIDTH-1:0]  rptr_bin;
    reg  [PTR_WIDTH-1:0]  rptr_gray;
    wire [PTR_WIDTH-1:0]  rptr_bin_next;
    wire [PTR_WIDTH-1:0]  rptr_gray_next;
    wire                  empty;
    reg                   empty_q;        // 2026-10-22 CDC 修复：empty 的 mb_clk 域打拍源
    wire                  read_en;        // 非空 + 非软复位时读使能

    // 读出数据 regout（mb_clk 域）
    reg  [DATA_WIDTH-1:0] data_o_reg;

    // underflow sticky bit（mb_clk 域）
    reg                   underflow;

    // 跨域同步信号
    // 写指针 Gray 码同步到 mb_clk 域
    reg  [PTR_WIDTH-1:0]  wptr_gray_sync1;
    reg  [PTR_WIDTH-1:0]  wptr_gray_sync2;

    // 读指针 Gray 码同步到 phy_rx_clk 域
    reg  [PTR_WIDTH-1:0]  rptr_gray_sync1;
    reg  [PTR_WIDTH-1:0]  rptr_gray_sync2;

    // soft_reset 双域同步
    reg                   soft_reset_rx_sync1;
    reg                   soft_reset_rx_sync2;
    reg                   soft_reset_mb_sync1;
    reg                   soft_reset_mb_sync2;

    // overflow_clear 同步到 phy_rx_clk 域（清除源在 mb_clk 域，apb_csr_top RW1C 解码）
    reg                   overflow_clear_sync1;
    reg                   overflow_clear_sync2;

    // fifo_status 跨域呈现：phy_rx_clk 域产生的 overflow / full 同步到 mb_clk 域
    reg                   overflow_sync_mb1;
    reg                   overflow_sync_mb2;
    reg                   full_sync_mb1;
    reg                   full_sync_mb2;

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // binary → Gray 编码
    //   gray = binary ^ (binary >> 1)
    // --------------------------------------------------------------------------
    assign wptr_bin_next  = wptr_bin + 1'b1;
    assign wptr_gray_next = wptr_bin_next ^ (wptr_bin_next >> 1);

    assign rptr_bin_next  = rptr_bin + 1'b1;
    assign rptr_gray_next = rptr_bin_next ^ (rptr_bin_next >> 1);

    // --------------------------------------------------------------------------
    // full 检测（phy_rx_clk 域，写侧观察）
    //   depth 4 = 2^2，需 3 bit 指针（PTR_WIDTH=3），绕 8 个状态区分 full/empty
    //   full 判定：wptr_gray == {~rptr_gray_sync2[2:1], rptr_gray_sync2[0]}
    //   （高二位取反 + 其余位相等——Gray 码回绕镜像特性，仅翻转最高位会在
    //     wptr=7 才误报 full，depth 4 FIFO 被写穿 3 次）
    //   注：本公式要求 FIFO_DEPTH == 2^(PTR_WIDTH-1)
    // --------------------------------------------------------------------------
    assign full = (wptr_gray == {~rptr_gray_sync2[PTR_WIDTH-1:PTR_WIDTH-2],
                                  rptr_gray_sync2[PTR_WIDTH-3:0]});

    // --------------------------------------------------------------------------
    // empty 检测（mb_clk 域，读侧观察）
    //   empty = (rptr_gray == wptr_gray_sync2)（Gray 码相等即 binary 相等）
    // --------------------------------------------------------------------------
    assign empty = (rptr_gray == wptr_gray_sync2);

    // --------------------------------------------------------------------------
    // 写 / 读使能（水位 + 软复位保护）
    // --------------------------------------------------------------------------
    assign write_en = ~full & ~soft_reset_rx_sync2;
    assign read_en  = ~empty & ~soft_reset_mb_sync2;

    // --------------------------------------------------------------------------
    // fifo_status 输出（mb_clk 域呈现，归 apb_csr_top RX_ASYNC_FIFO_STATUS）
    //   bit[0]=empty / bit[1]=full / bit[2]=underflow / bit[3]=overflow
    //   overflow / full 在 phy_rx_clk 域产生，经 2 flop 同步到 mb_clk 域呈现
    //   （MAS §5.1: {overflow_sync_mb, underflow, full_sync_mb, empty}）
    // --------------------------------------------------------------------------
    assign fifo_status = {overflow_sync_mb2, underflow, full_sync_mb2, empty_q};

    // 2026-09-07 zhangna CDC 修复(Ac_glitch03 full_sync_mb1): full 为组合比较结果
    // (wptr_gray 与 rptr_gray_sync2 组合相等判断),直接驱动 mb_clk 同步器首级会报
    // "Sources from same domain in fanin" 可 glitch。现先于 phy_rx_clk 域打一拍成
    // 寄存器源 full_q 再跨域同步,消除组合 glitch(overflow 本就是寄存器输出,无需改)。
    // 仅使状态呈现滞后 1 拍,不影响 write_en 组合门控(其仍用组合 full)。
    reg full_q;

    // ==========================================================================
    // 写侧时序逻辑（phy_rx_clk 域）
    // ==========================================================================
    always @(posedge phy_rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            wptr_bin  <= PTR_RESET;
            wptr_gray <= PTR_RESET;
        end
        else if (soft_reset_rx_sync2) begin
            // 软复位清空
            wptr_bin  <= PTR_RESET;
            wptr_gray <= PTR_RESET;
        end
        else if (write_en) begin
            // 写指针推进
            wptr_bin  <= wptr_bin_next;
            wptr_gray <= wptr_gray_next;
            // FIFO 寄存器写入（depth 4，按 wptr_bin 低 2 bit 寻址）
            fifo_reg[wptr_bin_next % FIFO_DEPTH] <= data_i;
        end
    end

    // --------------------------------------------------------------------------
    // overflow sticky bit（phy_rx_clk 域，RW1C）
    //   触发：full（满状态下上游持续送数；本模块无写请求输入，以 full 水位为溢出代理）
    //   保持：置 1 后保持，直到软写 1 清除（overflow_clear，mb_clk 域源经 2 flop 同步）
    //   优先级：软复位 > 软清除 > 置位
    // --------------------------------------------------------------------------
    always @(posedge phy_rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            overflow <= 1'b0;
        end
        else if (soft_reset_rx_sync2) begin
            overflow <= 1'b0;
        end
        else if (overflow_clear_sync2) begin
            overflow <= 1'b0;
        end
        else if (full) begin
            overflow <= 1'b1;
        end
    end

    // full 组合状态在 phy_rx_clk 域打一拍成寄存器源(full_q),供 mb_clk 域同步呈现
    always @(posedge phy_rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            full_q <= 1'b0;
        end
        else begin
            full_q <= full;
        end
    end

    // ==========================================================================
    // 读侧时序逻辑（mb_clk 域）
    // ==========================================================================
    always @(posedge mb_clk or negedge mb_rst_n) begin
        if (!mb_rst_n) begin
            rptr_bin   <= PTR_RESET;
            rptr_gray  <= PTR_RESET;
            data_o_reg <= {DATA_WIDTH{1'b0}};
        end
        else if (soft_reset_mb_sync2) begin
            // 软复位清空
            rptr_bin   <= PTR_RESET;
            rptr_gray  <= PTR_RESET;
            data_o_reg <= {DATA_WIDTH{1'b0}};
        end
        else if (read_en) begin
            // 读指针推进 + 读出数据 regout
            rptr_bin   <= rptr_bin_next;
            rptr_gray  <= rptr_gray_next;
            data_o_reg <= fifo_reg[rptr_bin_next % FIFO_DEPTH];
        end
    end

    // 2026-10-22 CDC 修复(Ac_glitch03 rx_fifo_status_sync1[5])：empty 为 mb_clk 域
    // 组合比较结果(rptr_gray 与 wptr_gray_sync2)，直接驱动 apb_clk 域同步器首级会报
    // multi-source toggling。现于 mb_clk 域打一拍成寄存器源 empty_q 再跨域同步，
    // 与 full_q(phy_rx_clk 域，:193-200) 域不同、做法对称。
    // 复位取 1：复位后 rptr_gray=PTR_RESET、wptr_gray_sync2=PTR_RESET ⇒ 组合 empty=1。
    // 仅使状态呈现滞后 1 拍，不影响 read_en 组合门控（其仍用组合 empty，:132）。
    always @(posedge mb_clk or negedge mb_rst_n) begin
        if (!mb_rst_n) begin
            empty_q <= 1'b1;
        end
        else begin
            empty_q <= empty;
        end
    end

    // --------------------------------------------------------------------------
    // underflow sticky bit（mb_clk 域，RW1C）
    //   触发：empty（空状态下下游持续取数；本模块无读请求输入，以 empty 水位为下溢代理）
    //   保持：置 1 后保持，直到软写 1 清除（underflow_clear，mb_clk 域本地）
    //   优先级：软复位 > 软清除 > 置位
    // --------------------------------------------------------------------------
    always @(posedge mb_clk or negedge mb_rst_n) begin
        if (!mb_rst_n) begin
            underflow <= 1'b0;
        end
        else if (soft_reset_mb_sync2) begin
            underflow <= 1'b0;
        end
        else if (underflow_clear) begin
            underflow <= 1'b0;
        end
        else if (empty) begin
            underflow <= 1'b1;
        end
    end

    // ==========================================================================
    // 跨域同步（2 flop synchronizers）
    // ==========================================================================

    // --------------------------------------------------------------------------
    // 写指针 Gray 码同步到 mb_clk 域（mb_clk 域 2 flop 同步器）
    // --------------------------------------------------------------------------
    always @(posedge mb_clk or negedge mb_rst_n) begin
        if (!mb_rst_n) begin
            wptr_gray_sync1 <= PTR_RESET;
            wptr_gray_sync2 <= PTR_RESET;
        end
        else begin
            wptr_gray_sync1 <= wptr_gray;
            wptr_gray_sync2 <= wptr_gray_sync1;
        end
    end

    // --------------------------------------------------------------------------
    // 读指针 Gray 码同步到 phy_rx_clk 域（phy_rx_clk 域 2 flop 同步器）
    // --------------------------------------------------------------------------
    always @(posedge phy_rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            rptr_gray_sync1 <= PTR_RESET;
            rptr_gray_sync2 <= PTR_RESET;
        end
        else begin
            rptr_gray_sync1 <= rptr_gray;
            rptr_gray_sync2 <= rptr_gray_sync1;
        end
    end

    // --------------------------------------------------------------------------
    // soft_reset 双域 2 flop 同步（按 MAS §3.6，假设 fifo_soft_reset 来自 apb_csr_top
    //   可能为 mb_clk 域或独立时钟域，此处双域各 2 flop 同步保险）
    // --------------------------------------------------------------------------
    // phy_rx_clk 域同步
    always @(posedge phy_rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            soft_reset_rx_sync1 <= 1'b0;
            soft_reset_rx_sync2 <= 1'b0;
        end
        else begin
            soft_reset_rx_sync1 <= fifo_soft_reset;
            soft_reset_rx_sync2 <= soft_reset_rx_sync1;
        end
    end

    // mb_clk 域同步
    always @(posedge mb_clk or negedge mb_rst_n) begin
        if (!mb_rst_n) begin
            soft_reset_mb_sync1 <= 1'b0;
            soft_reset_mb_sync2 <= 1'b0;
        end
        else begin
            soft_reset_mb_sync1 <= fifo_soft_reset;
            soft_reset_mb_sync2 <= soft_reset_mb_sync1;
        end
    end

    // --------------------------------------------------------------------------
    // overflow_clear 同步到 phy_rx_clk 域（清除源在 mb_clk 域）
    // --------------------------------------------------------------------------
    always @(posedge phy_rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            overflow_clear_sync1 <= 1'b0;
            overflow_clear_sync2 <= 1'b0;
        end
        else begin
            overflow_clear_sync1 <= overflow_clear;
            overflow_clear_sync2 <= overflow_clear_sync1;
        end
    end

    // --------------------------------------------------------------------------
    // fifo_status 跨域呈现：overflow / full 同步到 mb_clk 域（2 flop）
    // --------------------------------------------------------------------------
    always @(posedge mb_clk or negedge mb_rst_n) begin
        if (!mb_rst_n) begin
            overflow_sync_mb1 <= 1'b0;
            overflow_sync_mb2 <= 1'b0;
            full_sync_mb1     <= 1'b0;
            full_sync_mb2     <= 1'b0;
        end
        else begin
            overflow_sync_mb1 <= overflow;
            overflow_sync_mb2 <= overflow_sync_mb1;
            full_sync_mb1     <= full_q;
            full_sync_mb2     <= full_sync_mb1;
        end
    end

    // ==========================================================================
    // 输出驱动
    // ==========================================================================
    assign data_o = data_o_reg;

endmodule
