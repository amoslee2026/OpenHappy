// =============================================================================
// module name：p_crd_gen
// author: OpenHappy Team
// MAS：p_crd_gen_mas.md
// 功能概述: Credit 生成共享模板，统计本端可用 Credit 并编码为 2bit 送至对端
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   2026-08-06  OpenHappy Team  初始版本
// =============================================================================
// NOTE: CRD_NUM 默认取 `PRCL_ASYNC_RXFIFO_DEPTH(=32)，与 rxfifo DEPTH 对齐；
//   CRD_WIDTH=$clog2(CRD_NUM+1)。MAS §6 标注 CRD_NUM=16/CRD_WIDTH=5 与 §2.1 不一致，
//   本实现以 §2.1 + defines 宏为准（CRD_NUM=32, CRD_WIDTH=6）。
// =============================================================================


module p_crd_gen #(
    parameter CRD_NUM   = 32,
    parameter CRD_WIDTH = $clog2(CRD_NUM + 1)
) (
    input  wire                   clk,
    input  wire                   rstn,
    input  wire                   enable,
    input  wire  [CRD_WIDTH-1:0]  loc_crd,
    input  wire                   loc_crd_valid,
    output wire  [1:0]            m_crd,
    input  wire                   m_ready
);

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    reg  [CRD_WIDTH-1:0] crd_cnt_r;
    wire                 crd_8, crd_4, crd_1;
    wire [1:0]            m_crd_enc;
    wire [CRD_WIDTH-1:0]  crd_out;
    wire [CRD_WIDTH-1:0]  crd_cnt_next;

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================
    // 可用 Credit 阈值判定（crd_cnt_r >= 8 / >= 4 / >= 1）
    assign crd_8 = (crd_cnt_r >= 8);
    assign crd_4 = crd_8 | (crd_cnt_r >= 4);
    assign crd_1 = (crd_cnt_r >= 1);

    // m_crd 2bit 编码（enable=0 时恒 0）
    assign m_crd_enc = {crd_4, (crd_8 | (crd_1 & (~crd_4)))};
    assign m_crd      = m_crd_enc & {2{enable}};

    // crd_out 解码：m_crd 编码对应的 credit 数量
    //   2'b00→0, 2'b01→1, 2'b10→4, 2'b11→8
    assign crd_out = {{(CRD_WIDTH-4){1'b0}}, (&m_crd_enc), (m_crd_enc[1] & (~m_crd_enc[0])), 1'b0, ((~m_crd_enc[1]) & m_crd_enc[0])};

    // 下一拍 crd_cnt（位宽 CRD_WIDTH 足够：crd_cnt_r+loc_crd ≤ CRD_NUM < 2^CRD_WIDTH，
    //   默认无溢出；crd_out ≤ crd_cnt_r 保证无下溢）
    // ASSERT_INV: crd_cnt_r <= CRD_NUM（上界：crd_cnt_r 复位为 CRD_NUM，loc_crd 累加受 rxfifo 深度
    //            约束，正常工作下不会超 CRD_NUM；异常路径需 SVA 复核）
    // ASSERT_INV: crd_cnt_r >= crd_out（下界：crd_out 由 crd_cnt_r 编码得出，m_ready 时 crd_out
    //            恒 <= crd_cnt_r，无下溢风险）
    assign crd_cnt_next = crd_cnt_r + loc_crd - crd_out;

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            crd_cnt_r <= CRD_NUM[CRD_WIDTH-1:0];   // 复位满额度
        end
        else if (enable) begin
            case ({loc_crd_valid, m_ready})
                2'b11:   crd_cnt_r <= crd_cnt_next;
                2'b10:   crd_cnt_r <= crd_cnt_r + loc_crd;
                2'b01:   crd_cnt_r <= crd_cnt_r - crd_out;
                default: ; // hold
            endcase
        end
    end

endmodule
