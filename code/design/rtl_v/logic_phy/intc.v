// =============================================================================
// module name：intc
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/mas/intc_mas.md
// =============================================================================
// 功能概述: logic_phy per-link 中断控制器（中断最终聚合原语）：7 类中断源 mask 过滤 + OR 聚合
//   输出 irq_mcu / irq_ext（寄存 1 级），状态/屏蔽/清除存储全部归 apb_csr_top
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-07  OpenHappy Team  初始版本：基于 intc MAS v1.0 生成
// =============================================================================

`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// 设计说明:
//
// 中断 bit 映射（int_status_i[6:0]，apb_csr_top sticky 直送）:
//   bit[0]=lane_train_done / bit[1]=lane_train_fail / bit[2]=lane_step_done
//   bit[3]=lane_train_timeout / bit[4]=sbc_err / bit[5]=sbc_buf_ovf
//   bit[6]=cdc_fifo_err
// -----------------------------------------------------------------------------

module intc (
    // 时钟与复位
    input  wire        apb_pclk,
    input  wire        apb_preset_n,

    // 中断状态与屏蔽配置（apb_csr_top 具名信号直送，同域无 CDC）
    input  wire [6:0]  int_status_i,
    input  wire [6:0]  int_mask_mcu_i,
    input  wire [6:0]  int_mask_ext_i,

    // 聚合中断输出（高有效，寄存 1 级）
    output reg         irq_mcu_o,
    output reg         irq_ext
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    localparam INT_SRC_NUM = 7;  // 中断源类数（hardware-fixed）

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    // 两路 AND-NOT-OR 组合聚合结果
    wire irq_mcu_next;
    wire irq_ext_next;

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================
    // MCU 路聚合：任一未屏蔽中断源置位即向 MCU 上报
    // mask 复位默认全 1（全屏蔽，归 apb_csr_top INT_MASK_MCU 复位值），软件使能后生效
    assign irq_mcu_next = |(int_status_i & ~int_mask_mcu_i);

    // 外部路聚合：与 MCU 路独立屏蔽，同一 status 向量
    assign irq_ext_next = |(int_status_i & ~int_mask_ext_i);

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================
    // 输出寄存 1 级：中断源置位 → irq 输出延迟 1 cycle，撤销同理 1 cycle
    // level 中断语义：int_status_i 未清除且未屏蔽期间 irq 持续有效
    always @(posedge apb_pclk or negedge apb_preset_n) begin
        if (!apb_preset_n) begin
            irq_mcu_o <= 1'b0;
            irq_ext   <= 1'b0;
        end
        else begin
            irq_mcu_o <= irq_mcu_next;
            irq_ext   <= irq_ext_next;
        end
    end

endmodule
