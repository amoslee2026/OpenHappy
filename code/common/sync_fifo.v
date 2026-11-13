//======================================================================
// File: async_fifo
// Last Modify By: OpenHappy Team
// Revision: v0
// Date:
//
//======================================================================
// Description:
// async FIFO,depth is 2^n
//
//======================================================================

module sync_fifo
#(
parameter DATA_WIDTH = 32,
parameter DEPTH      = 16
)(
input  wire                  clk,
input  wire                  rst_n,

input  wire                  wen,
input  wire [DATA_WIDTH-1:0] wdata,

input  wire                  ren,
output wire [DATA_WIDTH-1:0] rdata,

output wire                  full,
output wire                  empty,
output wire                  overflow,
output wire                  underflow
);

localparam ADDR_WIDTH = (DEPTH == 1) ? 1 : $clog2(DEPTH);

reg  [ADDR_WIDTH:0]   wptr;
reg  [ADDR_WIDTH:0]   rptr;

wire [ADDR_WIDTH-1:0] waddr;
wire [ADDR_WIDTH-1:0] raddr;

reg  [DATA_WIDTH-1:0] mem[DEPTH-1:0];


assign waddr = wptr[ADDR_WIDTH-1:0];
assign raddr = rptr[ADDR_WIDTH-1:0];

always @(posedge clk) begin
    if(wen == 1'b1) begin
        mem[waddr] <= wdata;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0) begin
        wptr <= {(ADDR_WIDTH+1){1'b0}};
    end
    else begin
        if(wen == 1'b1 && full == 1'b0) begin
            wptr <= wptr + 1'b1;
        end
    end
end

assign rdata = mem[raddr];

always @(posedge clk or negedge rst_n) begin
    if(rst_n == 1'b0) begin
        rptr <= {(ADDR_WIDTH+1){1'b0}};
    end
    else begin
        if(ren == 1'b1 && empty == 1'b0) begin
            rptr <= rptr + 1'b1;
        end
    end
end

assign full      = ((wptr[ADDR_WIDTH] != rptr[ADDR_WIDTH]) && (wptr[ADDR_WIDTH-1:0] == rptr[ADDR_WIDTH-1:0])) ? 1'b1 : 1'b0;
assign empty     = (wptr == rptr) ? 1'b1 : 1'b0;
assign overflow  = (wen == 1'b1 && full == 1'b1) ? 1'b1 : 1'b0;
assign underflow = (ren == 1'b1 && empty == 1'b1) ? 1'b1 : 1'b0;

endmodule
