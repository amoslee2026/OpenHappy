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
module p_rx_demux1to2 #(
    parameter DATA_W = 512
) (
    input  wire                         sel,

    input  wire  [DATA_W      -1 : 0]   s_data,
    input  wire                         s_valid,
    output wire                         s_ready,

    output wire  [DATA_W      -1 : 0]   m0_data,
    output wire                         m0_valid,
    input  wire                         m0_ready,

    output wire  [DATA_W      -1 : 0]   m1_data,
    output wire                         m1_valid,
    input  wire                         m1_ready
);
    
    assign m1_data  = sel ? s_data  : {DATA_W{1'b0}};
    assign m1_valid = sel ? s_valid : 1'b0;

    assign m0_data  = sel ? {DATA_W{1'b0}} : s_data;
    assign m0_valid = sel ? 1'b0 : s_valid; 

    assign s_ready = sel ? m1_ready : m0_ready;
    
    
endmodule
