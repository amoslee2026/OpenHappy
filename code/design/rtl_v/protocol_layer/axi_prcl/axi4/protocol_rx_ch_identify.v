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

module protocol_rx_ch_identify  (
    input  wire                         clk,
    input  wire                         rst_n,

    output wire  [3           -1 : 0]   pkt_err,
    output wire                         err_intr,
    output wire  [4           -1 : 0]   rx_st,
    input  wire                         err_clr,
    
    input  wire  [512         -1 : 0]   s_paif_data,
    input  wire                         s_paif_valid,
    output wire                         s_paif_ready,

    output wire  [581         -1 : 0]   m_w_data,
    output wire                         m_w_valid,

    output wire  [529         -1 : 0]   m_r_data,
    output wire                         m_r_valid,

    output wire  [107*1       -1 : 0]   m_aw_data,
    output wire  [1           -1 : 0]   m_aw_valid,

    output wire  [107*1       -1 : 0]   m_ar_data,
    output wire  [1           -1 : 0]   m_ar_valid,

    output wire  [16*1        -1 : 0]   m_b_data,
    output wire  [1           -1 : 0]   m_b_valid,

    output wire  [2           -1 : 0]   m_awcrd,
    output wire                         m_awcrd_valid,

    output wire  [2           -1 : 0]   m_arcrd,
    output wire                         m_arcrd_valid,

    output wire  [2           -1 : 0]   m_wcrd,
    output wire                         m_wcrd_valid,

    output wire  [2           -1 : 0]   m_rcrd,
    output wire                         m_rcrd_valid,

    output wire  [2           -1 : 0]   m_bcrd,
    output wire                         m_bcrd_valid
);
    localparam [4  -1 : 0] ST_NULL      = 4'b0001;
    localparam [4  -1 : 0] ST_W         = 4'b0010;
    localparam [4  -1 : 0] ST_R         = 4'b0100;
    localparam [4  -1 : 0] ST_SPKT      = 4'b1000;

    assign err_intr = |pkt_err;

    wire [18   -1 : 0] crd_data;
    wire               crd_valid;

    assign {m_bcrd, m_rcrd, m_wcrd, m_arcrd, m_awcrd} = crd_data[9:0];

    assign m_awcrd_valid = crd_valid;
    assign m_arcrd_valid = crd_valid;
    assign m_wcrd_valid  = crd_valid;
    assign m_rcrd_valid  = crd_valid;
    assign m_bcrd_valid  = crd_valid;

    reg  [768  -1 : 0] temp_r;
    wire [768  -1 : 0] paif_lsft;
    wire [768  -1 : 0] temp_rsft;
    wire [256  -1 : 0] loc_paif_data;
    reg  [2    -1 : 0] cnt_r;
    reg  [2    -1 : 0] nxt_cnt;
    reg  [4    -1 : 0] st_r;
    reg  [4    -1 : 0] nxt_st;
    wire [5    -1 : 0] nxt_cn;
    reg                proc_r;
    wire               proc_nxt;
    wire               paif_first;
    wire               st_trig;
    
    assign  s_paif_ready = 1'b1;

    assign  paif_first = ((cnt_r == 2'd0) & s_paif_valid);

    assign  st_trig = proc_r | paif_first;

    assign  nxt_cn = s_paif_data[4:0];

    assign  proc_nxt = st_trig ? nxt_st[3] : (cnt_r[1] & s_paif_valid);

    //===================  FSM =======================//
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            proc_r <= 1'b0;
        end else begin
            proc_r <= proc_nxt;
        end
    end

    //===================  FSM =======================//
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            st_r <= ST_NULL;
        end else if(st_trig) begin
            st_r <= nxt_st;
        end
    end

    assign  nxt_st[2:0] = s_paif_valid ? {(nxt_cn == `AXI4_RDATA_CHANNEL) , (nxt_cn == `AXI4_WDATA_CHANNEL) , (nxt_cn == 5'd0)} : 3'd1;
    assign  nxt_st[3]   = (nxt_st[2:0] == 3'd0);
    

    //===================  how many 256bits data in temp_r =======================//
    assign  nxt_cnt = proc_r ? {1'b0 , (s_paif_valid & (nxt_cn != 5'd0))} : s_paif_valid ? (cnt_r + 2'd1) : cnt_r;


    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            cnt_r <= 2'd0;
        end else if (proc_r || s_paif_valid) begin
            cnt_r <= nxt_cnt;
        end
    end


    //===================  temp_r =======================//
    assign loc_paif_data = s_paif_valid ? s_paif_data[255:0] : 256'd0;

    assign paif_lsft = st_trig  ?  {512'd0 , loc_paif_data} : 
                       cnt_r[0] ? {256'd0 , loc_paif_data , 256'd0} : {loc_paif_data , 512'd0};

    assign temp_rsft = proc_r ? 768'd0 : temp_r;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            temp_r <= 768'd0;
        end else if (proc_r || s_paif_valid) begin
            temp_r <= temp_rsft | paif_lsft;
        end
    end


    //===================  channel out  =======================//

    wire  s_err;

    assign  m_w_data = { temp_r[5+128+16+1 +: 4],
                         temp_r[5+128+16],
                         temp_r[485+128 +: 16], temp_r[325+128 +: 16], temp_r[165+128 +: 16], temp_r[5+128 +: 16],
                         temp_r[485 +: 128], temp_r[325 +: 128], temp_r[165 +: 128], temp_r[5 +: 128]};

    assign  m_r_data = { temp_r[5+128+2+8+1 +: 4],
                         temp_r[5+128+2+8],
                         {2'd0, temp_r[5+128+2 +: 8]},
                         temp_r[5+128 +: 2],
                         temp_r[(456+5) +: 128], temp_r[(304+5) +: 128], temp_r[(152+5) +: 128], temp_r[5 +: 128]};

    
    assign  m_w_valid = proc_r & st_r[1];
    assign  m_r_valid = proc_r & st_r[2];

    
    wire [256  -1 : 0] pkt_data;
    wire               pkt_vld;

    assign  pkt_vld = proc_r;

    assign  pkt_data = {128'd0 , {128{st_r[1]}} & temp_r[640 +: 128]} | 
                       {96'd0  , {160{st_r[2]}} & temp_r[608 +: 160]} | 
                       ({256{st_r[3]}} & temp_r[0 +: 256]);
    
    
    protocol_rx_pkt_proc  u_pkt_proc(    
        .s_pkt_data     (pkt_data   ),
        .s_pkt_valid    (pkt_vld    ),

        .m_aw_data      (m_aw_data  ),
        .m_aw_valid     (m_aw_valid ),

        .m_ar_data      (m_ar_data  ),
        .m_ar_valid     (m_ar_valid ),

        .m_b_data       (m_b_data[15:2]   ),
        .m_b_valid      (m_b_valid  ),

        .m_crd_data     (crd_data   ),
        .m_crd_valid    (crd_valid  ),

        .pld_err        (s_err      )
    );

    assign m_b_data[1:0] = 2'd0;

    //===================  err =======================//

    wire  r_err;
    wire  w_err;
    wire  err_clr_sync;   // err_clr 两级同步（CSR 域 → 本模块 clk 域）

    reg   s_err_r;
    reg   r_err_r;
    reg   w_err_r;

    // err_clr CDC 两级同步
    sync_cell u_err_clr_sync (.clk_d(clk), .rstn_d(rst_n), .data_s(err_clr), .data_d(err_clr_sync));

    assign  w_err = m_w_valid & ((temp_r[160 +: 5] != `AXI4_WDATA_CHANNEL) | (temp_r[320 +: 5] != `AXI4_WDATA_CHANNEL) | (temp_r[480 +: 5] != `AXI4_WDATA_CHANNEL));

    assign  r_err = m_r_valid & ((temp_r[152 +: 5] != `AXI4_RDATA_CHANNEL) | (temp_r[304 +: 5] != `AXI4_RDATA_CHANNEL) | (temp_r[456 +: 5] != `AXI4_RDATA_CHANNEL));
    

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            s_err_r <= 1'b0;
        end else if (s_err) begin
            s_err_r <= 1'b1;
        end else if (err_clr_sync) begin
            s_err_r <= 1'b0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            w_err_r <= 1'b0;
        end else if (w_err) begin
            w_err_r <= 1'b1;
        end else if (err_clr_sync) begin
            w_err_r <= 1'b0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            r_err_r <= 1'b0;
        end else if (r_err) begin
            r_err_r <= 1'b1;
        end else if (err_clr_sync) begin
            r_err_r <= 1'b0;
        end
    end


    assign  pkt_err = {r_err_r , w_err_r , s_err_r};

    //================ st out =====================//
    assign  rx_st = st_r;



endmodule
