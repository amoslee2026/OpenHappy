//========================================================================
// File: p_paif_loop.v
// 来源: design/rtl_v/block/d2d_ctrl/protocol_layer/protocol_paif_loopback.v
//       （复制至 axi_prcl/common/ 后二次修改）
// author: OpenHappy Team
// Revision:
//
//========================================================================
// 功能概述: PAIF 层回环（protocol_tx → link tx / link rx → protocol_rx 的旁路/回灌）。
//   本地定宽简化版：paif_data 位宽由 parameter PAIF_DW 约束，删除 strb/fltcnt 辅助信号
//   （valid 统一为 1bit），rst_n 改 rstn，模块名 protocol_paif_loopback → p_paif_loop。
//   sel==1: s0(protocol_tx) -> m1(protocol_rx)  内回环，绕过 Link。
//   数据路径：m0 regout（tx 到 link）+ s0/s1 data_pip + m1 data_byp（rx 输出寄存）。
// 修改记录:
//   2026-08-30  OpenHappy Team  定宽化 PAIF_DW + 删 strb/fltcnt + rstn + 更名 p_paif_loop
// ========================================================================

module p_paif_loop #(
    parameter PAIF_DW = 512
) (
    input  wire                clk,
    input  wire                rstn,

    input  wire                sel,

    // slave 0 connect protocol tx
    input  wire [PAIF_DW-1:0]  s0_paif_data,
    input  wire                s0_paif_valid,
    output wire                s0_paif_ready,

    // slave 1 connect link rx
    input  wire [PAIF_DW-1:0]  s1_paif_data,
    input  wire                s1_paif_valid,
    output wire                s1_paif_ready,

    // master 0 connect link tx
    output wire [PAIF_DW-1:0]  m0_paif_data,
    output wire                m0_paif_valid,
    input  wire                m0_paif_ready,

    // master 1 connect protocol rx
    output wire [PAIF_DW-1:0]  m1_paif_data,
    output wire                m1_paif_valid,
    input  wire                m1_paif_ready
);

//====================================
//  protocol_tx -> | s0   m0  | -> link tx
//                 |          |
//                 |          |
//  protocol rx <- | m1   s1  | <- link rx
//
//  sel == 1:   s0 -> m1
//====================================

assign m0_paif_data  = sel ? {PAIF_DW{1'b0}} : s0_paif_data;
assign m0_paif_valid = sel ? {PAIF_DW{1'b0}} : s0_paif_valid;

assign m1_paif_data  = sel ? s0_paif_data : s1_paif_data;
assign m1_paif_valid = sel ? s0_paif_valid : s1_paif_valid;

assign s0_paif_ready = sel ? m1_paif_ready : m0_paif_ready;
assign s1_paif_ready = sel ? 1'b0 : m1_paif_ready;



    //============  m0 regout =======================//
    /*
    reg  [PAIF_DW-1:0]   m0_data_r;
    reg                  m0_valid_r;
    reg                  m0_valid_rep_r;

    wire    loc_m0_ready;
    assign  loc_m0_ready = m0_paif_ready | (~m0_valid_r);

    wire    loc_m0_valid;
    assign  loc_m0_valid = s0_paif_valid & (!sel);

    wire    loc_m0_shake;
    assign  loc_m0_shake = loc_m0_valid & loc_m0_ready;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            m0_valid_r     <= 1'b0;
            m0_valid_rep_r <= 1'b0;
        end
        else begin
            if (loc_m0_shake) begin
                m0_valid_r     <= s0_paif_valid;
                m0_valid_rep_r <= s0_paif_valid;
            end
            else if (m0_paif_ready) begin
                m0_valid_r     <= 1'b0;
                m0_valid_rep_r <= 1'b0;
            end
        end
    end

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            m0_data_r <= {PAIF_DW{1'b0}};
        end
        else begin
            if (loc_m0_shake) begin
                m0_data_r <= s0_paif_data;
            end
        end
    end

    assign  m0_paif_data  = m0_data_r;
    assign  m0_paif_valid = m0_valid_rep_r;
    */


    //============  s1 data pip  =======================//
    /*
    wire  [PAIF_DW-1:0]   loc_s1_data;
    wire                  loc_s1_valid;
    wire                  loc_s1_ready;

    p_data_pip #(
        .DATA_W         (PAIF_DW)
    ) u_s1_data_pip(
        .clk            (clk    ),
        .rst_n          (rstn   ),

        .s_data         (s1_paif_data),
        .s_valid        (s1_paif_valid),
        .s_ready        (s1_paif_ready),

        .m_data         (loc_s1_data),
        .m_valid        (loc_s1_valid),
        .m_ready        (loc_s1_ready)
    );
    */

    //============  s0 data pip  =======================//
    /*
    wire  [PAIF_DW-1:0]   loc_s0_data;
    wire                  loc_s0_valid;
    wire                  loc_s0_ready;

    p_data_pip #(
        .DATA_W         (PAIF_DW)
    ) u_s0_data_pip(
        .clk            (clk    ),
        .rst_n          (rstn   ),

        .s_data         (s0_paif_data),
        .s_valid        (s0_paif_valid),
        .s_ready        (s0_paif_ready),

        .m_data         (loc_s0_data),
        .m_valid        (loc_s0_valid),
        .m_ready        (loc_s0_ready)
    );
    */


    //============  m1 ready regout =======================//
    /*
    wire  [PAIF_DW-1:0]   loc_m1_data;
    wire                  loc_m1_valid;
    wire                  loc_m1_ready;

    p_data_byp #(
        .DATA_W         (PAIF_DW)
    ) u_m1_data_byp(
        .clk            (clk    ),
        .rst_n          (rstn   ),

        .s_data         (loc_m1_data),
        .s_valid        (loc_m1_valid),
        .s_ready        (loc_m1_ready),

        .m_data         (m1_paif_data),
        .m_valid        (m1_paif_valid),
        .m_ready        (m1_paif_ready)
    );
    */


    //=========== loopback logic ======================//
    /*
    assign  loc_m1_data  = ({PAIF_DW{sel}} & loc_s0_data) | ({PAIF_DW{!sel}} & loc_s1_data);
    assign  loc_m1_valid = (sel & loc_s0_valid) | ((!sel) & loc_s1_valid);

    assign  loc_s0_ready = ((!sel) & loc_m0_ready) | (sel & loc_m1_ready);
    assign  loc_s1_ready = (!sel) & loc_m1_ready;
    */


endmodule
