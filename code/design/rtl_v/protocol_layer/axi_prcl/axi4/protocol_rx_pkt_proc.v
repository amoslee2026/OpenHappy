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

module protocol_rx_pkt_proc  (    
    input  wire  [256         -1 : 0]   s_pkt_data,
    input  wire                         s_pkt_valid,

    output wire  [107         -1 : 0]   m_aw_data,
    output wire                         m_aw_valid,

    output wire  [107         -1 : 0]   m_ar_data,
    output wire                         m_ar_valid,

    output wire  [14          -1 : 0]   m_b_data,
    output wire                         m_b_valid,

    output wire  [18          -1 : 0]   m_crd_data,
    output wire                         m_crd_valid,

    output wire                         pld_err
);
    
    wire                aw_get;
    wire  [2    -1 : 0] ar_get;
    wire  [3    -1 : 0] b_get;
    wire  [5    -1 : 0] crd_get;

    wire                aw_vld;
    wire  [2    -1 : 0] ar_vld;
    wire  [3    -1 : 0] b_vld;
    wire  [5    -1 : 0] crd_vld;


    wire                pld_0b;
    wire                pld_32b;
    wire                pld_64b;
    wire                pld_128b;
    wire                pld_160b;
    wire                pld_192b;
    wire                pld_224b;
    wire                pld_256b;

    wire  [8    -1 : 0] pld_flag;

    assign  aw_get = (s_pkt_data[0 +: 5] == `AXI4_AW_CHANNEL);

    assign  ar_get[0] = (s_pkt_data[0 +: 5] == `AXI4_AR_CHANNEL);
    assign  ar_get[1] = (s_pkt_data[112 +: 5] == `AXI4_AR_CHANNEL);

    assign  b_get[0] = (s_pkt_data[0 +: 5] == `AXI4_B_CHANNEL);
    assign  b_get[1] = (s_pkt_data[112 +: 5] == `AXI4_B_CHANNEL);
    assign  b_get[2] = (s_pkt_data[(112+112) +: 5] == `AXI4_B_CHANNEL);

    assign  crd_get[0] = (s_pkt_data[0 +: 5] == `AXI4_CREDIT_CHANNEL);
    assign  crd_get[1] = (s_pkt_data[24 +: 5] == `AXI4_CREDIT_CHANNEL);
    assign  crd_get[2] = (s_pkt_data[112 +: 5] == `AXI4_CREDIT_CHANNEL);
    assign  crd_get[3] = (s_pkt_data[(112+24) +: 5] == `AXI4_CREDIT_CHANNEL);
    assign  crd_get[4] = (s_pkt_data[(112+112) +: 5] == `AXI4_CREDIT_CHANNEL);


    assign  pld_0b   = (s_pkt_data[0 +: 5] == 5'd0);
    assign  pld_32b  = (b_get[0] & (s_pkt_data[24 +: 5] == 5'd0)) | (crd_get[0] & (s_pkt_data[32 +: 5] == 5'd0));
    assign  pld_64b  = b_get[0] & crd_get[1] & (s_pkt_data[(24+32) +: 5] == 5'd0);
    assign  pld_128b = (aw_get | ar_get[0]) & (s_pkt_data[112 +: 5] == 5'd0);
    assign  pld_160b = (aw_get | ar_get[0]) & ((b_get[1] & (s_pkt_data[(112+24) +: 5] == 5'd0)) | (crd_get[2] & (s_pkt_data[(112+32) +: 5] == 5'd0)));
    assign  pld_192b = (aw_get | ar_get[0]) & (b_get[1] & crd_get[3]) & (s_pkt_data[(112+24+32) +: 5] == 5'd0);
    assign  pld_224b = aw_get & ar_get[1] & (s_pkt_data[(112+112) +: 5] == 5'd0);
    assign  pld_256b = aw_get & ar_get[1] & (b_get[2] | crd_get[4]);
    
    assign  pld_flag = {pld_256b , pld_224b , pld_192b , pld_160b ,
                        pld_128b , pld_64b  , pld_32b  , pld_0b};


    assign  aw_vld = aw_get;

    assign  ar_vld[0] = ar_get[0];
    assign  ar_vld[1] = ar_get[1] & (!(|pld_flag[2:0]));

    assign  b_vld[0] = b_get[0];
    assign  b_vld[1] = b_get[1] & (!(|pld_flag[2:0]));
    assign  b_vld[2] = b_get[2] & pld_256b;

    assign  crd_vld[0] = crd_get[0];
    assign  crd_vld[1] = crd_get[1] & pld_64b;
    assign  crd_vld[2] = crd_get[2] & (!(|pld_flag[2:0]));
    assign  crd_vld[3] = crd_get[3] & pld_192b;
    assign  crd_vld[4] = crd_get[4] & pld_256b;


    assign  m_aw_data = s_pkt_data[5 +: 107];

    assign  m_ar_data = (s_pkt_data[5 +: 107] & {107{ar_vld[0]}}) | 
                        (s_pkt_data[(112+5) +: 107] & {107{ar_vld[1]}}) ;

    assign  m_b_data  = (s_pkt_data[5 +: 14] & {14{b_vld[0]}}) | 
                        (s_pkt_data[(112+5) +: 14] & {14{b_vld[1]}}) | 
                        (s_pkt_data[(112+112+5) +: 14] & {14{b_vld[2]}}) ; 

    assign  m_crd_data = (s_pkt_data[5 +: 18] & {18{crd_vld[0]}}) | 
                         (s_pkt_data[(24+5) +: 18] & {18{crd_vld[1]}}) | 
                         (s_pkt_data[(112+5) +: 18] & {18{crd_vld[2]}}) | 
                         (s_pkt_data[(112+24+5) +: 18] & {18{crd_vld[3]}}) | 
                         (s_pkt_data[(112+112+5) +: 18] & {18{crd_vld[4]}});

    
    assign  m_aw_valid = s_pkt_valid & aw_vld;
    assign  m_ar_valid = s_pkt_valid & (|ar_vld);
    assign  m_b_valid  = s_pkt_valid & (|b_vld);
    assign  m_crd_valid  = s_pkt_valid & (|crd_vld);

    assign  pld_err = s_pkt_valid & (pld_flag == 8'd0);

endmodule
