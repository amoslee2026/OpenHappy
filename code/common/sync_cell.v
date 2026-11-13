// -------------------------------------------------------------------------------
// File   : sync_cell.v
// Author : OpenHappy Team
// Date   : 2026-08-04
// -------------------------------------------------------------------------------
// Description:
//   Common single-bit CDC synchronizer (multi-stage flip-flop, default 2-stage).
//   Async assert on rstn_d, releases to RST_VALUE to match the source signal's
//   reset value.
//   Per verilog-generator skill: data_s MUST be a register output; multi-bit
//   buses MUST instantiate one sync_cell per bit (e.g. gray-code pointers),
//   never feed combinational logic into data_s.
// -------------------------------------------------------------------------------
// Change Log:
//   2026-08-04  OpenHappy Team  v1.0  Initial creation (project common library)
//   2026-09-28  OpenHappy Team      v2.0  Sync stage configurable: replace fixed 2-stage
//                                FF chain (dnt_sync_regs_1/_2) with a generate
//                                loop keyed on new parameter SYNC_STAGE (default 2),
//                                mirroring rstn_sync.v style; RST_VALUE branch retained.
// -------------------------------------------------------------------------------

module sync_cell #(
    parameter                               RST_VALUE   = 1'b0                      ,
    parameter                               SYNC_STAGE  = 2
)(
    input                                   clk_d               ,
    input                                   rstn_d              ,
    input                                   data_s              ,
    output                                  data_d
);
    wire [SYNC_STAGE-1:0]                   sync_regs           ;

    genvar i;
    generate
        if(RST_VALUE == 1'b0)begin
            for(i = 0; i < SYNC_STAGE; i = i + 1)begin:dnt_sync_regs
                generic_dff_q dnt_sync_regs_ff(
                    .D               (i == 0 ? data_s : sync_regs[i-1]),
                    .RDN             (rstn_d                    ),
                    .CK              (clk_d                     ),
                    .Q               (sync_regs[i]              )
                );
            end
        end else begin//async set
            for(i = 0; i < SYNC_STAGE; i = i + 1)begin:dnt_sync_regs
                generic_dffs_q dnt_sync_regs_ff(
                    .D               (i == 0 ? data_s : sync_regs[i-1]),
                    .SDN             (rstn_d                    ),
                    .CK              (clk_d                     ),
                    .Q               (sync_regs[i]              )
                );
            end
        end
    endgenerate

    assign data_d = sync_regs[SYNC_STAGE-1];

endmodule

