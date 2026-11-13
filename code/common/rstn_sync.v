// -------------------------------------------------------------------------------
// File   : rstn_sync.v
// Author : OpenHappy Team
// Date   : 2026-08-04
// -------------------------------------------------------------------------------
// Description:
//   Common reset synchronizer. Async assert / sync deassert: resetn_out drops
//   asynchronously with resetn_in and releases synchronously after
//   RESET_SYNC_STAGE clk edges, removing reset-removal metastability.
//   DFT: when test_mode is asserted, resetn_out bypasses to test_resetn_in.
// -------------------------------------------------------------------------------
// Change Log:
//   2026-08-04  OpenHappy Team  v1.0  Initial creation (project common library)
//   2026-08-06  OpenHappy Team  v1.1  Assignment width matching: shift-in '1' 常量补 0 到
//                                RESET_SYNC_STAGE 位，OR 两侧位宽对齐
// -------------------------------------------------------------------------------

module rstn_sync (
		  test_mode,
		  test_resetn_in,
		  resetn_in,
		  clk,
		  resetn_out
);

parameter  RESET_SYNC_STAGE = 2;

input   test_mode;
input   test_resetn_in;
input   resetn_in;
input   clk;

output  resetn_out;

wire    resetn_out;
wire    resetn_sync;
wire    resetn_in_t;

wire [RESET_SYNC_STAGE-1:0]    resetn_sync_array;

assign resetn_in_t = test_mode ? test_resetn_in : resetn_in;

genvar i;
generate
    for(i = 0; i < RESET_SYNC_STAGE; i = i + 1)begin:rstn_sync_regs
        if(i == 0)begin
            generic_dff_q dnt_rstn_sync_reg(
                .D               (1'b1                      ),
                .RDN             (resetn_in_t               ), 
                .CK              (clk                       ),
                .Q               (resetn_sync_array[i]      )
            );
        end else begin
            generic_dff_q dnt_rstn_sync_reg(
                .D               (resetn_sync_array[i - 1]  ),
                .RDN             (resetn_in_t               ),
                .CK              (clk                       ),
                .Q               (resetn_sync_array[i]      )
            );

        end
    end 
endgenerate

assign resetn_sync  = resetn_sync_array[RESET_SYNC_STAGE-1];

assign resetn_out = test_mode ? test_resetn_in : resetn_sync;

endmodule
