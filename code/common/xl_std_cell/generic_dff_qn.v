// =============================================================================
// module name：generic_dff_qn
// 功能概述: 异步复位 DFF（RDN 低有效），D → QN（反相输出）
//           RTL 行为实现，等效通用标准单元（SE/SI 扫描口恒 0）
// =============================================================================
// version history
//   2026-10-26  OpenHappy Team  自主实现：行为模型 → 手写 RTL 实现
// =============================================================================
`timescale 1ns/1ps

module generic_dff_qn(
    input       D                       ,
    input       RDN                     ,
    input       CK                      ,
    output      QN
);

    reg q_r;

    always @(posedge CK or negedge RDN) begin
        if (!RDN) q_r <= 1'b0;
        else      q_r <= D;
    end

    assign QN = ~q_r;

endmodule
