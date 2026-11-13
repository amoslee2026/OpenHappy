// -------------------------------------------------------------------------------
// File   : rstn_sync_neg.v
// Author : OpenHappy Team
// Date   : 2026-10-21
// -------------------------------------------------------------------------------
// Description:
//   Negative-edge reset synchronizer —— rstn_sync.v 的负沿极性兄弟模块。
//   Async assert / sync deassert: resetn_out drops asynchronously with resetn_in
//   and releases synchronously after RESET_SYNC_STAGE negedge clk edges,
//   removing reset-removal metastability.
//   DFT: when test_mode is asserted, resetn_out bypasses to test_resetn_in.
//
//   极性成对范式（跟随库内既有 wrapper 惯例，而非给 rstn_sync 加 CLK_POL 参数）：
//     generic_icg_pos.v / generic_icg_neg.v
//     generic_dff_q.v   / generic_dff_qn.v
//     rstn_sync.v       / rstn_sync_neg.v   ← 本文件
//   加参数会在 rstn_sync 内部引入极性分支（改变既有实例层级 / lint waiver 路径），且
//   破坏 logic_phy_spec.md:386「本模块为纯单元调用，无独立 spec」不变量；兄弟模块对
//   rstn_sync 的 33 处实例零影响。
//
//   实现 = **真负沿描述**（always @(negedge clk …)）⇒ 综合映射原生负沿单元
//   SNDRNQV*（.CKN 端口），STA 从网表直接取下降沿，**零 SDC 增量**。与签核网表现状
//   一致：d2d_hipi_dig_top_0_Sep21.v 中 SNDRNQV* 42 处、CLKINV/CKBUF 0 处，
//   既有负沿 FF 全部免反相器直连 .CKN。
//
//   【反例·勿复用】不得写成 wire clk_i = ~clk 再喂正沿 FF（或加 CLK_POL 参数二选一）：
//   本设计 flow/d2d_hipi_top/generate_sdc/*.sdc 中 set_propagated_clock = 0 处、
//   set_clock_sense / set_clock_inverters = 0 处 ⇒ 时钟按**理想时钟**传播，理想时钟
//   不会在反相器输出翻转极性 ⇒ PT/DC 仍按原相位正沿查 creation/removal，
//   「复位释放只剩半周期余量」这类违例在 STA 里**不可见**（是漏检，不是修好）。
//   generic_ckinv 现有使用者（glitch_free_mux.v:121-123）只驱动 enable，从未驱动时钟。
//
//   使用场景：logic_phy sb_rx_clk 负沿域（MAS logic_phy_crg_mas.md:51/:63/:74/:115；
//   logic_phy_spec.md:386「本模块为纯单元调用，无独立 spec」）。
// -------------------------------------------------------------------------------
// Change Log:
//   2026-10-21  OpenHappy Team  v1.0  Initial creation（issue #14 R7）
// -------------------------------------------------------------------------------

module rstn_sync_neg (
		  test_mode,
		  test_resetn_in,
		  resetn_in,
		  clk,
		  resetn_out
);

parameter  RESET_SYNC_STAGE = 2;

input   test_mode;
input   test_resetn_in;
input   resetn_in;
input   clk;

output  resetn_out;

wire    resetn_out;
wire    resetn_in_t;

reg  [RESET_SYNC_STAGE-1:0]    resetn_sync_r;

assign resetn_in_t = test_mode ? test_resetn_in : resetn_in;

// 移位链：第 0 级 D 恒 1，逐级串接前级 Q —— 与 rstn_sync.v 的 generic_dff_q 链同构。
//   RESET_SYNC_STAGE=1 时无 "前级"，退化为单级 D 恒 1，故分两支（避免 [-1:0] 非法切片）。
//   单 always 块整体赋值整个向量（每个 always 只赋 1 个信号，rtl-coding §6.1）。
generate
    if(RESET_SYNC_STAGE > 1)begin:rstn_sync_neg_regs
        always @(negedge clk or negedge resetn_in_t)begin
            if(!resetn_in_t)begin
                resetn_sync_r <= {RESET_SYNC_STAGE{1'b0}};
            end
            else begin
                resetn_sync_r <= {resetn_sync_r[RESET_SYNC_STAGE-2:0], 1'b1};
            end
        end
    end else begin:rstn_sync_neg_reg
        always @(negedge clk or negedge resetn_in_t)begin
            if(!resetn_in_t)begin
                resetn_sync_r <= 1'b0;
            end
            else begin
                resetn_sync_r <= 1'b1;
            end
        end
    end
endgenerate

assign resetn_out = test_mode ? test_resetn_in : resetn_sync_r[RESET_SYNC_STAGE-1];

endmodule
