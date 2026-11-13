// =============================================================================
// module name：generic_dffs_q
// 功能概述: 异步置位 DFF（SDN 低有效），D → Q
//           RTL 行为实现，等效通用标准单元（SE/SI 扫描口恒 0）
// =============================================================================
// version history
//   2026-10-26  OpenHappy Team  自主实现：行为模型 → 手写 RTL 实现
// =============================================================================
`timescale 1ns/1ps

module generic_dffs_q(
    input       D                       ,
    input       SDN                     ,
    input       CK                      ,
    output      Q
);

    reg q_r;

    always @(posedge CK or negedge SDN) begin
        if (!SDN) q_r <= 1'b1;
        else      q_r <= D;
    end

    assign Q = q_r;

endmodule
