//======================================================================
// File: glitch_free_mux
// Last Modify By: OpenHappy Team
// Revision: 0.1
// Date:2025.1.2
//
//======================================================================
// Description:
//
//======================================================================
`timescale 1ns/1ps
module glitch_free_mux (
	  input     rstn            ,
	  input     clk0            ,
	  input     clk1            ,
	  input     sel             ,
	  input     test_mode       ,
	  output    clk_sel
);

    wire reset0_n, reset1_n;

    rstn_sync clk0_rstn_sync(
		  .test_mode(test_mode) ,
		  .test_resetn_in(1'b1) ,
		  .resetn_in(rstn)      ,
		  .clk(clk0)            ,
		  .resetn_out(reset0_n)
    );
    //rstn sync for clk1
    rstn_sync clk1_rstn_sync(
		  .test_mode(test_mode) ,
		  .test_resetn_in(1'b1) ,
		  .resetn_in(rstn)      ,
		  .clk(clk1)            ,
		  .resetn_out(reset1_n)
    );

	wire    select_sync2_clk0;
    wire    select_sync2_clk0_inv;
	wire	select_sync2_clk1;

	wire    clk0_en_sync2;
    wire    clk0_en_sync2_inv;
	wire    clk1_en_sync2;
    wire    clk1_en_sync2_inv;

	wire    clk0_en;
	wire	clk1_en;

	reg	    clk0_en_reg;
	reg	    clk1_en_reg;

	wire	gated_clk0;
	wire	gated_clk1;
    //
    sync_cell i_sel_sync2_clk0(
        .clk_d                  (clk0               ),
        .rstn_d                 (reset0_n           ),
        .data_s                 (sel                ),
        .data_d                 (select_sync2_clk0  )
    );
    /*
    sync_cell #(
        .RST_VALUE              (1'b1               )
    ) i_clk1_en_sync2_clk0(
        .clk_d                  (clk0               ),
        .rstn_d                 (reset0_n           ),
        .data_s                 (clk1_en_reg        ),
        .data_d                 (clk1_en_sync2      )
    );
    */
   //output clk0 during reset
    sync_cell i_clk1_en_sync2_clk0(
        .clk_d                  (clk0               ),
        .rstn_d                 (reset0_n           ),
        .data_s                 (clk1_en_reg        ),
        .data_d                 (clk1_en_sync2      )
    );
	

	always @(posedge clk0 or negedge reset0_n) begin
		if (!reset0_n)
			clk0_en_reg <= 1'b1;
		else
			clk0_en_reg <= clk0_en;
	end
    
    //
    sync_cell i_sel_sync2_clk1(
        .clk_d                  (clk1               ),
        .rstn_d                 (reset1_n           ),
        .data_s                 (sel                ),
        .data_d                 (select_sync2_clk1  )
    );
    //
    sync_cell #(
        .RST_VALUE              (1'b1               )
    ) i_clk0_en_sync2_clk1(
        .clk_d                  (clk1               ),
        .rstn_d                 (reset1_n           ),
        .data_s                 (clk0_en_reg        ),
        .data_d                 (clk0_en_sync2      )
    );

	always @(posedge clk1 or negedge reset1_n) begin
		if (!reset1_n)
			clk1_en_reg <= 1'b0;
		else
			clk1_en_reg <= clk1_en;
	end
    
    generic_ckinv dnt_clk_inv0  ( .I(select_sync2_clk0), .ZN(select_sync2_clk0_inv));
    generic_ckinv dnt_clk_inv1  ( .I(clk1_en_sync2), .ZN(clk1_en_sync2_inv));
    generic_ckinv dnt_clk_inv2  ( .I(clk0_en_sync2), .ZN(clk0_en_sync2_inv));
    generic_ckand dnt_clk_and0  ( .A1(select_sync2_clk0_inv), .A2(clk1_en_sync2_inv), .Z(clk0_en));
    generic_ckand dnt_clk_and1  ( .A1(select_sync2_clk1), .A2(clk0_en_sync2_inv), .Z(clk1_en));
	//assign clk0_en = ~select_sync2_clk0 & ~clk1_en_sync2;
	//assign clk1_en =  select_sync2_clk1 & ~clk0_en_sync2;

	generic_icg_pos dnt_gck_clk0 (
        .CK(clk0)                                               ,
        .E(clk0_en)                                             , 
        .Q(gated_clk0)
	);
	generic_icg_pos dnt_gck_clk1 (
        .CK(clk1)                                               ,
        .E(clk1_en)                                             ,
        .Q(gated_clk1)
	);

	generic_ckor dnt_clk_or( .Z(clk_sel), .A1(gated_clk0), .A2(gated_clk1) );

endmodule
