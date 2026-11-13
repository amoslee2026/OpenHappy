// =============================================================================
// module name：generic_icg_pos
// 功能概述: 正沿时钟门控单元（ICG），RTL 行为实现，等效通用标准单元
//   结构：锁存(n1, E|TE, hold=CK) → CK 低相位透明；Q = n1 & CK
//   即：CK=0 期间锁存使能，CK=1 期间输出跟随 CK → 无毛刺门控时钟
// =============================================================================
// version history
//   2026-10-26  OpenHappy Team  自主实现：行为模型 → 手写 RTL 实现
//                          必须用锁存器建模（不可写 CK&E 纯组合，否则边沿毛刺）
// =============================================================================
`timescale 1ns/1ps

module generic_icg_pos(
    input       CK                      ,
    input       E                       ,
    output      Q
);

    reg en_lat;

    // CK 低相位透明（锁存使能），CK 高相位保持 → 消除使能信号引起的时钟毛刺
    always @(*) begin
        if (!CK) en_lat = E;
    end

    assign Q = CK & en_lat;

endmodule
