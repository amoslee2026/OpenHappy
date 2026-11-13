//========================================================================
// File:
// Last Modify By: OpenHappy Team
// Revision:
// Date:
//
//========================================================================
// Description:
//
//========================================================================
module protocol_tx_packet  (
    input  wire                         clk,
    input  wire                         rst_n,
    
    input  wire  [107         -1 : 0]   s_aw_data,
    input  wire                         s_aw_valid,
    output wire                         s_aw_ready,

    input  wire  [107         -1 : 0]   s_ar_data,
    input  wire                         s_ar_valid,
    output wire                         s_ar_ready,

    input  wire  [581         -1 : 0]   s_w_data,
    input  wire                         s_w_valid,
    output wire                         s_w_ready,

    input  wire  [529         -1 : 0]   s_r_data,
    input  wire                         s_r_valid,
    output wire                         s_r_ready,

    input  wire  [16          -1 : 0]   s_b_data,
    input  wire                         s_b_valid,
    output wire                         s_b_ready,

    input  wire  [2           -1 : 0]   s_awcrd,
    output wire                         s_awcrd_ready,
    input  wire  [2           -1 : 0]   s_arcrd,
    output wire                         s_arcrd_ready,
    input  wire  [2           -1 : 0]   s_wcrd,
    output wire                         s_wcrd_ready,
    input  wire  [2           -1 : 0]   s_rcrd,
    output wire                         s_rcrd_ready,
    input  wire  [2           -1 : 0]   s_bcrd,
    output wire                         s_bcrd_ready,

    output wire  [768         -1 : 0]   m_data,
    output wire  [3           -1 : 0]   m_valid,
    input  wire                         m_ready
);
    // Priority : W = R > AW = AR > CRD > B
    
    reg   [768   -1 : 0] pkt;

    wire  [3     -1 : 0] pkt_vld;

    wire  [112   -1 : 0] aw_vc;
    wire  [112   -1 : 0] ar_vc;
    wire  [640   -1 : 0] w_vc;
    wire  [608   -1 : 0] r_vc;
    wire  [24    -1 : 0] b_vc;
    wire  [32    -1 : 0] crd_vc;
    wire                 crd_valid;
    wire                 crd_ready;

    assign s_awcrd_ready = crd_ready;
    assign s_arcrd_ready = crd_ready;
    assign s_wcrd_ready  = crd_ready;
    assign s_rcrd_ready  = crd_ready;
    assign s_bcrd_ready  = crd_ready;

    assign crd_valid = (|(s_awcrd | s_arcrd | s_wcrd | s_rcrd | s_bcrd));

    assign aw_vc  = {s_aw_data, `AXI4_AW_CHANNEL};
    assign ar_vc  = {s_ar_data, `AXI4_AR_CHANNEL};
    assign b_vc   = {3'd0, s_b_data, `AXI4_B_CHANNEL};
    assign crd_vc = {17'd0, s_bcrd, s_rcrd, s_wcrd, s_arcrd, s_awcrd, `AXI4_CREDIT_CHANNEL};

    assign r_vc  = {{4'd0, s_r_data[512+2+10+1 +: 4], s_r_data[512+2+10], s_r_data[512+2 +: 8], s_r_data[512 +: 2], s_r_data[384 +: 128], `AXI4_RDATA_CHANNEL},   // [R转换] 4×152=608 → r_vc（2026-09-22 修复：原误写 ar_vc，s_r_data 529bit 内偏移合法）
                    {4'd0, s_r_data[512+2+10+1 +: 4], s_r_data[512+2+10], s_r_data[512+2 +: 8], s_r_data[512 +: 2], s_r_data[256 +: 128], `AXI4_RDATA_CHANNEL},
                    {4'd0, s_r_data[512+2+10+1 +: 4], s_r_data[512+2+10], s_r_data[512+2 +: 8], s_r_data[512 +: 2], s_r_data[128 +: 128], `AXI4_RDATA_CHANNEL},
                    {4'd0, s_r_data[512+2+10+1 +: 4], s_r_data[512+2+10], s_r_data[512+2 +: 8], s_r_data[512 +: 2], s_r_data[0 +: 128], `AXI4_RDATA_CHANNEL} };

    assign w_vc  = {{6'd0, s_w_data[577 +: 4], s_w_data[576], s_w_data[560 +: 16], s_w_data[384 +: 128], `AXI4_WDATA_CHANNEL},   // [W转换] 4×160=640 → w_vc
                    {6'd0, s_w_data[577 +: 4], s_w_data[576], s_w_data[544 +: 16], s_w_data[256 +: 128], `AXI4_WDATA_CHANNEL},    // 2026-09-22 修复：原误写 aw_vc 且误读 s_r_data（529bit）越界，
                    {6'd0, s_w_data[577 +: 4], s_w_data[576], s_w_data[528 +: 16], s_w_data[128 +: 128], `AXI4_WDATA_CHANNEL},   // 正确源为 s_w_data（581bit bond W：user[580:577]/last[576]/strb64[575:512]/data[511:0]）
                    {6'd0, s_w_data[577 +: 4], s_w_data[576], s_w_data[512 +: 16], s_w_data[0 +: 128], `AXI4_WDATA_CHANNEL} };
    

    wire    s_x_valid;
    wire    s_ax_valid;

    assign  s_x_valid  = s_w_valid | s_r_valid;
    assign  s_ax_valid = s_aw_valid | s_ar_valid;

    wire    no_x;
    wire    no_ax;

    assign  no_x  = !s_x_valid;
    assign  no_ax = !s_ax_valid;

    reg     x_st_r;     // 0 select w; 1 select r
    reg     ax_st_r;    // 0 select aw; 1 select ar

    wire    s_w_sel;
    wire    s_r_sel;
    wire    s_aw_sel;
    wire    s_ar_sel;
    wire    s_crd_sel;
    wire    s_b_sel;

    assign  s_w_sel = s_w_valid & ((!x_st_r) | (!s_r_valid));
    assign  s_r_sel = s_r_valid & (  x_st_r  | (!s_w_valid));

    assign  s_aw_sel = s_aw_valid & ((!ax_st_r) | (!s_ar_valid) | no_x);
    assign  s_ar_sel = s_ar_valid & (  ax_st_r  | (!s_aw_valid) | no_x);

    assign  s_crd_sel = crd_valid & ((!s_w_sel) | (s_w_sel & no_ax));

    assign  s_b_sel = s_b_valid & (no_ax | (s_r_sel & (!crd_valid)) | 
                      (no_x & ((s_ar_valid ^ s_aw_valid) | (!crd_valid))));


    assign  s_w_ready = m_ready & s_w_sel;
    assign  s_r_ready = m_ready & s_r_sel;
    assign  s_aw_ready = m_ready & s_aw_sel;
    assign  s_ar_ready = m_ready & s_ar_sel;
    assign  crd_ready = m_ready & s_crd_sel;
    assign  s_b_ready   = m_ready & s_b_sel;



    wire    s_valid;
    assign  s_valid = s_w_valid | s_r_valid | s_aw_valid | s_ar_valid | crd_valid | s_b_valid;
    
    

    //====================  W/R  select =====================//
    wire    nxt_x_st;

    assign  nxt_x_st = s_w_sel;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            x_st_r <= 1'b0;
        end
        else begin
            if(s_w_ready || s_r_ready) begin
                x_st_r <= nxt_x_st;
            end
        end
    end


    //====================  AW/AR  select =====================//
    wire    nxt_ax_st;

    assign  nxt_ax_st = s_aw_sel ? (s_ar_sel ? ax_st_r : 1'b1) : 1'b0;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            ax_st_r <= 1'b0;
        end
        else begin
            if(s_aw_ready || s_ar_ready) begin
                ax_st_r <= nxt_ax_st;
            end
        end
    end
    

    //====================  valid  =====================//
    assign  pkt_vld[0] = s_valid;
    assign  pkt_vld[1] = s_x_valid;
    assign  pkt_vld[2] = s_x_valid;

    
    //====================  data   =====================//
    always @(*) begin
        if(s_w_sel) begin
            case({s_b_sel , s_crd_sel , s_ar_sel , s_aw_sel})
                4'b0001     :   pkt = {16'd0 , aw_vc , w_vc};
                4'b0010     :   pkt = {16'd0 , ar_vc , w_vc};
                4'b1100     :   pkt = {{(128-24-32){1'b0}} , crd_vc , b_vc , w_vc};
                4'b0100     :   pkt = {{(128-32){1'b0}} , crd_vc , w_vc};
                4'b1000     :   pkt = {{(128-24){1'b0}} , b_vc , w_vc};
                4'b0000     :   pkt = {128'd0 , w_vc};
                default     :   pkt = {128'd0 , w_vc};
            endcase
        end
        else if(s_r_sel) begin
            case({s_b_sel , s_crd_sel , s_ar_sel , s_aw_sel})
                4'b0001     :   pkt = {{(160-112){1'b0}} , aw_vc , r_vc};
                4'b0101     :   pkt = {{(160-112-32){1'b0}} , crd_vc , aw_vc , r_vc};
                4'b1001     :   pkt = {{(160-112-24){1'b0}} , b_vc , aw_vc , r_vc};
                4'b0010     :   pkt = {{(160-112){1'b0}} , ar_vc , r_vc};
                4'b0110     :   pkt = {{(160-112-32){1'b0}} , crd_vc , ar_vc , r_vc};
                4'b1010     :   pkt = {{(160-112-24){1'b0}} , b_vc , ar_vc , r_vc};
                4'b1100     :   pkt = {{(160-24-32){1'b0}} , crd_vc , b_vc , r_vc};
                4'b0100     :   pkt = {{(160-32){1'b0}} , crd_vc , r_vc}; 
                4'b1000     :   pkt = {{(160-24){1'b0}} , b_vc , r_vc};
                4'b0000     :   pkt = {{(160){1'b0}} , r_vc};
                default     :   pkt = {{(160){1'b0}} , r_vc};
            endcase
        end
        else if(s_valid) begin
            case({s_b_sel , s_crd_sel , s_ar_sel , s_aw_sel})
                4'b1011     :   pkt = {512'd0 , 8'd0 , b_vc , ar_vc , aw_vc};
                4'b0111     :   pkt = {512'd0 , crd_vc , ar_vc , aw_vc};
                4'b0011     :   pkt = {512'd0 , 32'd0 , ar_vc , aw_vc};
                4'b0001     :   pkt = {512'd0 , 144'd0 , aw_vc};
                4'b0010     :   pkt = {512'd0 , 144'd0 , ar_vc}; 
                4'b1101     :   pkt = {512'd0 , 88'd0 , crd_vc , b_vc , aw_vc};
                4'b1110     :   pkt = {512'd0 , 88'd0 , crd_vc , b_vc , ar_vc};
                4'b0101     :   pkt = {512'd0 , 112'd0 , crd_vc , aw_vc};
                4'b1001     :   pkt = {512'd0 , 120'd0 , b_vc , aw_vc};
                4'b0110     :   pkt = {512'd0 , 112'd0 , crd_vc , ar_vc};
                4'b1010     :   pkt = {512'd0 , 120'd0 , b_vc , ar_vc};
                4'b1100     :   pkt = {512'd0 , 200'd0 , crd_vc , b_vc};
                4'b1000     :   pkt = {512'd0 , 232'd0 , b_vc};
                4'b0100     :   pkt = {512'd0 , 224'd0 , crd_vc};
                default     :   pkt = {512'd0 , 224'd0 , crd_vc};
            endcase
        end
        else begin
            pkt = 768'd0;
        end
    end

    //====================  error check =====================//

    //reg  err;

    //always @(posedge clk or negedge rst_n) begin
    //    if(!rst_n) begin
    //        err <= 1'b0;
    //    end
    //    else begin
    //        if(s_w_sel) begin
    //            case({s_b_sel , s_crd_sel , s_ar_sel , s_aw_sel})
    //                4'b0001,
    //                4'b0010,
    //                4'b1100,
    //                4'b0100,
    //                4'b1000,
    //                4'b0000     :   err <= 1'b0;
    //                default     :   err <= 1'b1;
    //            endcase
    //        end
    //        else if(s_r_sel) begin
    //            case({s_b_sel , s_crd_sel , s_ar_sel , s_aw_sel})
    //                4'b0001,
    //                4'b0101,
    //                4'b1001,
    //                4'b0010,
    //                4'b0110,
    //                4'b1010,
    //                4'b1100,
    //                4'b0100,
    //                4'b1000,
    //                4'b0000     :   err <= 1'b0;
    //                default     :   err <= 1'b1;
    //            endcase
    //        end
    //        else if(s_valid) begin
    //            case({s_b_sel , s_crd_sel , s_ar_sel , s_aw_sel})
    //                4'b1011,
    //                4'b0111,
    //                4'b0011,
    //                4'b0001,
    //                4'b0010,
    //                4'b1101,
    //                4'b1110,
    //                4'b0101,
    //                4'b1001,
    //                4'b0110,
    //                4'b1010,
    //                4'b1100,
    //                4'b1000,
    //                4'b0100     :   err <= 1'b0;
    //                default     :   err <= 1'b1;
    //            endcase
    //        end
    //    end
    //end



    //====================  Output  =====================//

    assign  m_data  = pkt;

    assign  m_valid = pkt_vld;

endmodule
