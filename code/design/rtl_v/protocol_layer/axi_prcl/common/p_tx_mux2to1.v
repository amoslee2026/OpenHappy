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
module p_tx_mux2to1 #(
    parameter DATA_W = 512
) (
    input  wire                         sel,

    input  wire  [DATA_W      -1 : 0]   s0_data,
    input  wire                         s0_valid,
    output wire                         s0_ready,

    input  wire  [DATA_W      -1 : 0]   s1_data,
    input  wire                         s1_valid,
    output wire                         s1_ready,

    output wire  [DATA_W      -1 : 0]   m_data,
    output wire                         m_valid,
    input  wire                         m_ready
);
    
    assign m_data  = sel ? s1_data  : s0_data;
    assign m_valid = sel ? s1_valid : s0_valid;

    assign s0_ready = sel ? 1'b0 : m_ready;
    assign s1_ready = sel ? m_ready : 1'b0; 
    
    
endmodule
