module sync_icg_pos (
    input  wire  RSTN,
    input  wire  CK,
    input  wire  E,
    output wire  Q
);
    wire loc_enable;

    sync_cell  #( .RST_VALUE (1'b1) ) i_clk_en ( .clk_d (CK),  .rstn_d (RSTN),  .data_s (E),  .data_d (loc_enable) );

    generic_icg_pos  i_clkcg ( .CK (CK),  .E (loc_enable),  .Q (Q) );

endmodule
