// =============================================================================
// module name：p_ovrflow_det
// author: OpenHappy Team
// MAS：p_ovrflow_det_mas.md
// 功能概述: RX FIFO 写侧溢出检测共享模板，(~ready)&valid 时拉中断并经 sync_cell 同步 clr
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   2026-08-06  OpenHappy Team  初始版本
// =============================================================================


module p_ovrflow_det (
    // 工作域（rxfifo 写时钟域）
    input  wire        clk,
    input  wire        rstn,
    input  wire        ready,    // rxfifo 写侧 ready（0=FIFO 满）
    input  wire        valid,    // rxfifo 写侧 valid
    output wire        intr,     // 溢出中断（工作域电平）
    input  wire        clr       // 中断清除（APB 域，经 sync_cell 同步到工作域）
);

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    wire        clr_sync;     // clr 经 2 级 FF 同步到工作域
    reg         intr_r;       // 溢出中断寄存器（电平触发，置位后保持至 clr_sync）

    // ==========================================================================
    // 子模块实例化
    // ==========================================================================
    // clr 从 APB 域（pclk）同步到工作域（clk）
    sync_cell u_clr_sync (
        .clk_d  (clk),
        .rstn_d (rstn),
        .data_s (clr),
        .data_d (clr_sync)
    );

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================
    // 溢出条件优先级高于 clr，确保不漏报
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            intr_r <= 1'b0;
        end
        else if ((~ready) & valid) begin
            intr_r <= 1'b1;
        end
        else if (clr_sync) begin
            intr_r <= 1'b0;
        end
    end

    assign intr = intr_r;

endmodule
