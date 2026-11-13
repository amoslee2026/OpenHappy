// =============================================================================
// module name：sync_cell_norst
// author: OpenHappy Team
// 功能概述: 通用无复位 CDC 同步单元（2 级触发器打拍）。链路层跨时钟域电平 /
//           准静态信号同步用（数据通路 mb_clk / 控制 sb_clk / 配置 apb_clk
//           三域两两互异步，L1 §2.3）；配置类信号复位后的默认值由 apb_csr
//           寄存器自身复位值经 2 拍传播获得，无需同步器复位。
//           多 bit 总线按位例化（generate 展开）。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   <日期>  <修订人>  <变更概述>
//  2026-08-10  OpenHappy Team  初始版本（替换 code/common/sync_cell：无复位、
//                          无 foundry 单元依赖，仿真/lint 无需 stub）
// =============================================================================

`timescale 1ns/1ps

module sync_cell_norst (
    syncout,
    in,
    clk
);

parameter width = 1, size = 1;

output [width-1:0] syncout;
input  [width-1:0] in;
input              clk;

reg    [width-1:0] sq_0;
reg    [width-1:0] sq_1;
wire   [width-1:0] syncout;

//input tmx;

always @(posedge clk)
begin
sq_0 <= in;
sq_1 <= sq_0;
end

assign syncout = sq_1;

endmodule
