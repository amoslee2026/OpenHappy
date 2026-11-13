//
//====================================================================
// File: data_pipe
// Last Modify By: OpenHappy Team
// Revision: 0.1
// Date:2026.1.23
//
//=====================================================================
// Description:
//
//=====================================================================
module data_pipe #(
    parameter  DATA_WIDTH = 32,
    parameter  PIPE_STAGE = 1
)(
    input  wire                     clk,
    input  wire [DATA_WIDTH-1 : 0]  data_in,
    output wire [DATA_WIDTH-1 : 0]  data_out
);
    genvar i;

    reg  [DATA_WIDTH-1 : 0]  pipe [PIPE_STAGE : 0];

    assign  data_out = pipe[PIPE_STAGE];

    generate
        for (i = 0 ; i < (PIPE_STAGE+1) ; i = i + 1) begin:pipe_
            if (i == 0) begin
                always@(*) begin
                    pipe[0] = data_in;
                end
            end
            else begin
                always@(posedge clk) begin
                    pipe[i] <= pipe[i-1];
                end
            end
        end
    endgenerate

endmodule
