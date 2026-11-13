// =============================================================================
// module name：p_crd_tx_ctrl
// author: OpenHappy Team
// MAS：p_crd_tx_ctrl_mas.md
// 功能概述: Credit 发送控制共享模板，接收对端 Credit 并通过 Credit 反压数据发送
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   2026-08-06  OpenHappy Team  初始版本
// =============================================================================


module p_crd_tx_ctrl (
    input  wire        clk,
    input  wire        rstn,
    input  wire        enable,
    input  wire [1:0]  remote_crd,
    input  wire        remote_crd_valid,
    input  wire        s_valid,
    output wire        s_ready,
    output wire        m_valid,
    input  wire        m_ready
);

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    reg  [7:0]   crd_cnt_r;
    wire [7:0]   crd_in;
    wire         m_handshake;
    wire         loc_ready;
    wire [7:0]   crd_cnt_next;

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================
    // remote_crd 2bit 编码解码为 credit 数量：0→0, 1→1, 2→4, 3→8
    assign crd_in = {{4{1'b0}}, (&remote_crd), (remote_crd[1] & (~remote_crd[0])), 1'b0, ((~remote_crd[1]) & remote_crd[0])};

    assign m_handshake = m_valid & m_ready;
    assign loc_ready   = (|crd_cnt_r) | (~enable);

    // 数据通路反压：Credit>0 或 disable 时放行
    assign m_valid = s_valid & loc_ready;
    assign s_ready = m_ready & loc_ready;

    // 下一拍 crd_cnt（位宽 8bit 足够：crd_cnt_r ≤ FIFO 深度 32，默认不溢出）
    assign crd_cnt_next = crd_cnt_r + crd_in - 8'd1;

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            crd_cnt_r <= 8'd0;
        end
        else begin
            case ({remote_crd_valid, (m_handshake & enable)})
                2'b11:   crd_cnt_r <= crd_cnt_next;
                2'b10:   crd_cnt_r <= crd_cnt_r + crd_in;
                2'b01:   crd_cnt_r <= crd_cnt_r - 8'd1;
                default: ; // hold
            endcase
        end
    end

endmodule
