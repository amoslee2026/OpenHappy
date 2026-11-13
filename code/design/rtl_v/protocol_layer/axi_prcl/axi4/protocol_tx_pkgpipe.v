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
module protocol_tx_pkgpipe  (
    input  wire                         clk,
    input  wire                         rst_n,
    
    input  wire  [768         -1 : 0]   s_data,
    input  wire  [3           -1 : 0]   s_valid,
    output wire                         s_ready,

    output wire  [512         -1 : 0]   m_data,
    output wire                         m_valid,
    input  wire                         m_ready
);
    
    reg   [768   -1 : 0] buf_r;

    reg   [3     -1 : 0] loc_valid_r;

    wire  loc_ready;

    wire  s_hsk;

    assign loc_ready = (loc_valid_r[2:1] == 2'd0);

    assign s_ready = loc_ready & m_ready;

    assign s_hsk = s_valid[0] & s_ready;

    assign m_data = {256'd0 , buf_r[255:0]};

    assign m_valid = loc_valid_r[0];
    
    

    //==================== buf_r =====================//
    
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            buf_r <= 768'd0;
        end else if (s_hsk) begin
            buf_r <= s_data;
        end else if (m_ready) begin
            buf_r <= buf_r >> 256;
        end
    end


    //====================  loc valid =====================//
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            loc_valid_r <= 3'd0;
        end else if (s_hsk) begin
            loc_valid_r <= s_valid;
        end else if (m_ready) begin
            loc_valid_r <= loc_valid_r >> 1;
        end
    end

endmodule
