//========================================================================
// File: p_data_byp.v
// 来源: design/rtl_v/block/d2d_ctrl/protocol_layer/
//       d2d_data_byp.v（复制至 axi_prcl/common/ 后更名）
// author: OpenHappy Team
// Revision:
//
//========================================================================
// Description: 就绪旁路数据寄存（Bypass-on-ready data register）：输出 m_data/m_valid 由
//   m_ready 与 s_shake 控制，将写侧数据打一拍保持（当 m 侧未 ready 时保持前一拍数据）。
//   通用 DATA_W 参数化，用于回环模块 m1（输出寄存）侧。端口 rst_n 沿用源模块命名。
// 修改记录:
//   2026-08-30  OpenHappy Team  模块 d2d_data_byp → p_data_byp（更名；端口/逻辑无改动）
// ========================================================================
module p_data_byp #(
    parameter DATA_W = 64
)(
    input  wire                     clk,
    input  wire                     rst_n,

    input  wire  [DATA_W -1 : 0]    s_data,
    input  wire                     s_valid,
    output wire                     s_ready,

    output wire  [DATA_W -1 : 0]    m_data,
    output wire                     m_valid,
    input  wire                     m_ready
);

    reg [DATA_W -1 : 0]  temp_r;
    reg                  rdy_r;
    reg                  vld_r;

    wire   s_shake;

    assign  s_shake = s_ready & s_valid;

    assign  s_ready = rdy_r;

    assign  m_data = rdy_r ? s_data : temp_r;

    assign  m_valid = rdy_r ? s_valid : vld_r;

    always @ (posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            temp_r <= {DATA_W{1'b0}};
        end
        else begin
            if(s_shake)
                temp_r <= s_data;
        end
    end

    always @ (posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            rdy_r <= 1'b0;
        end
        else begin
            if(m_ready)
                rdy_r <= 1'b1;
            else if(s_shake)
                rdy_r <= 1'b0;
        end
    end

    always @ (posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            vld_r <= 1'b0;
        end
        else begin
            if(m_ready)
                vld_r <= 1'b0;
            else if(s_shake)
                vld_r <= 1'b1;
        end
    end

endmodule
