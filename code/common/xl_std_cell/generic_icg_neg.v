// =============================================================================
// module name：generic_icg_neg
// 功能概述: 负沿时钟门控单元（ICG），RTL 行为实现，等效通用标准单元
//   结构：锁存(n1, E|TE, hold=~CK) → CK 高相位透明；Q = ~n1 | CK
//   即：CK=1 期间锁存使能，CK=0 期间输出 = ~使能（禁用时恒高）
// =============================================================================
// version history
//   2026-10-26  OpenHappy Team  自主实现：行为模型 → 手写 RTL 实现
// =============================================================================
`timescale 1ns/1ps

module generic_icg_neg(
    input       CK                      ,
    input       E                       ,
    output      Q
);

    reg en_lat;

    // CK 高相位透明（锁存使能），CK 低相位保持 → 负沿门控，禁用时 Q 恒 1
    always @(*) begin
        if (CK) en_lat = E;
    end

    assign Q = CK | ~en_lat;

endmodule
