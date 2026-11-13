//========================================================================
// File: p_data_pip.v
// 来源: design/rtl_v/block/d2d_ctrl/protocol_layer/
//       d2d_data_pip.v（复制至 axi_prcl/common/ 后更名）
// author: OpenHappy Team
// Revision:
//
//========================================================================
// Description: 数据流水寄存器（Pipeline data register）：FWFT 风格，写侧 s_shake 打一拍存入
//   temp_r，m 侧直出 temp_r/vld_r（m_ready 反压清 vld_r）。通用 DATA_W 参数化，
//   用于回环模块 s0/s1（输入流水）侧。端口 rst_n 沿用源模块命名。
// 修改记录:
//   2026-08-30  OpenHappy Team  模块 d2d_data_pip → p_data_pip（更名；端口/逻辑无改动）
// ========================================================================
module p_data_pip #(
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
    reg                  vld_r;

    wire    s_shake;
    assign  s_shake = s_ready & s_valid;

    assign  m_data = temp_r;
    assign  m_valid = vld_r;
    assign  s_ready = m_ready | (~vld_r);

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
            vld_r <= 1'b0;
        end
        else begin
            if(s_shake)
                vld_r <= 1'b1;
            else if(m_ready)
                vld_r <= 1'b0;
        end
    end

endmodule
