//======================================================================
// File: async_fifo.v
// Last Modify By: OpenHappy Team
// Revision: 1.1
// Date:2025/7/7
//
//======================================================================
// Description:
//
//======================================================================
module async_fifo #(
    parameter                                            FIFO_DEPTH  = 16                ,   
                                                         FIFO_WIDTH  = 768               
)(  
    //write domain
    input                                                wclk                            ,
    input                                                wrstn                           ,
    input                                                wen                             ,
    input       [FIFO_WIDTH-1        : 0]                din                             ,   
    output                                               fifo_full                       ,
    output      [$clog2(FIFO_DEPTH)  : 0]                fifo_wrptr                      ,
    output      [$clog2(FIFO_DEPTH)  : 0]                fifo_rdptr_syn_w                ,
    //read domain
    input                                                rclk                            ,
    input                                                rrstn                           ,
    input                                                ren                             ,
    output reg  [FIFO_WIDTH-1        : 0]                dout                            ,
    output                                               fifo_empty                      ,
    output      [$clog2(FIFO_DEPTH)  : 0]                fifo_rdptr                                                                                   
);
    //pointer
    reg         [$clog2(FIFO_DEPTH)  : 0]                wr_ptr_b        , rd_ptr_b      ;
    wire        [$clog2(FIFO_DEPTH)  : 0]                wr_ptr_b_next   , rd_ptr_b_next ;
    reg         [$clog2(FIFO_DEPTH)  : 0]                wr_ptr_g        , rd_ptr_g      ;
    wire        [$clog2(FIFO_DEPTH)  : 0]                wr_ptr_g_next   , rd_ptr_g_next ;
    reg         [$clog2(FIFO_DEPTH)  : 0]                wr_ptr_g_sync   , rd_ptr_g_sync ;
    wire        [$clog2(FIFO_DEPTH)  : 0]                wr_ptr_g_s_b    , rd_ptr_g_s_b  ;  
    //
    reg         [FIFO_WIDTH-1        : 0]                fifo_ram [FIFO_DEPTH-1:0]       ;
    //reset combine
    wire                                                 wr_rstn                         ;
    wire                                                 w_rstn                          ;
    wire                                                 r_rstn                          ;

    //=====================================================================================
    //reset combine and sync
    assign wr_rstn = wrstn && rrstn;
    //
    rstn_sync i_wrstn_sync(
	    .test_mode                                  (1'b0                               ),
		.test_resetn_in                             (1'b1                               ),
		.resetn_in                                  (wr_rstn                            ),
		.clk                                        (wclk                               ),
		.resetn_out                                 (w_rstn                             )    
    );
    //
    rstn_sync i_rrstn_sync(
	    .test_mode                                  (1'b0                               ),
		.test_resetn_in                             (1'b1                               ),
		.resetn_in                                  (wr_rstn                            ),
		.clk                                        (rclk                               ),
		.resetn_out                                 (r_rstn                             )    
    );
    //=====================================================================================

    //=====================================================================================
    //write domain
    //for fifo write pointer wrapper in capture module
    assign fifo_wrptr = wr_ptr_b;
    //for credit cpmpute in capture module
    assign fifo_rdptr_syn_w = rd_ptr_g_s_b;

    //read domain
    //for fifo read pointer wrapper in axi module
    assign fifo_rdptr = rd_ptr_b;
    //===================================================================================== 

    //=====================================================================================
    //next pointer binary -> gray-code 
    genvar i;
    generate
        for(i=$clog2(FIFO_DEPTH); i>-1; i=i-1)begin:binary2gray
            if(i == $clog2(FIFO_DEPTH))begin
                //wr_ptr
                assign wr_ptr_g_next[i] = wr_ptr_b_next[i];
                //rd_ptr
                assign rd_ptr_g_next[i] = rd_ptr_b_next[i];
            end else begin
                //wr_ptr
                assign wr_ptr_g_next[i] = wr_ptr_b_next[i] ^ wr_ptr_b_next[i+1];
                //rd_ptr
                assign rd_ptr_g_next[i] = rd_ptr_b_next[i] ^ rd_ptr_b_next[i+1];
            end
        end
    endgenerate

    //pointer gray-code -> binary
    generate 
        for(i=$clog2(FIFO_DEPTH); i>-1; i=i-1)begin:gray2binary
            if(i == $clog2(FIFO_DEPTH))begin
                //wr_ptr
                assign wr_ptr_g_s_b[i] = wr_ptr_g_sync[i];
                //rd_ptr
                assign rd_ptr_g_s_b[i] = rd_ptr_g_sync[i];
            end else begin
                //wr_ptr
                assign wr_ptr_g_s_b[i] = wr_ptr_g_sync[i] ^ wr_ptr_g_s_b[i+1];
                //rd_ptr
                assign rd_ptr_g_s_b[i] = rd_ptr_g_sync[i] ^ rd_ptr_g_s_b[i+1];
            end
        end
    endgenerate 
    //=====================================================================================
    //write domain
    //next binary write pointer
    assign wr_ptr_b_next = (wen&&!fifo_full) ? wr_ptr_b+1 : wr_ptr_b;
    
    //binary write pointer update
    always@(posedge wclk, negedge w_rstn)begin
        if(!w_rstn)begin
            wr_ptr_b<={($clog2(FIFO_DEPTH)+1){1'b0}};
        end else begin
            wr_ptr_b<=wr_ptr_b_next;
        end
    end
    
    //gray-code write pointer update
    always@(posedge wclk, negedge w_rstn)begin
        if(!w_rstn)begin
            wr_ptr_g<={($clog2(FIFO_DEPTH)+1){1'b0}};
        end else begin
            wr_ptr_g<=wr_ptr_g_next;
        end
    end
    
    //read pointer sync
    generate
        for(i = 0; i < $clog2(FIFO_DEPTH)+1; i = i + 1)begin:rd_ptr_g_sync2
            sync_cell rd_ptr_sync (
                .clk_d                (wclk                         ),
                .rstn_d               (w_rstn                       ),         
                .data_s               (rd_ptr_g[i]                  ),
                .data_d               (rd_ptr_g_sync[i]             )
            );
        end
    endgenerate
    
    //=====================================================================================
    

    //=====================================================================================
    //read domain
    //next binary read pointer    
    assign rd_ptr_b_next = (ren&&!fifo_empty) ? rd_ptr_b+1 : rd_ptr_b;

    //binary read pointer update
    always@(posedge rclk, negedge r_rstn)begin
        if(!r_rstn)begin
            rd_ptr_b<={($clog2(FIFO_DEPTH)+1){1'b0}};
        end else begin
            rd_ptr_b<=rd_ptr_b_next;
        end
    end
    ////gray-code read pointer update
    always@(posedge rclk, negedge r_rstn)begin
        if(!r_rstn)begin
            rd_ptr_g<={($clog2(FIFO_DEPTH)+1){1'b0}};
        end else begin
            rd_ptr_g<=rd_ptr_g_next;
        end
    end
    //write pointer sync
    generate
        for(i = 0; i < $clog2(FIFO_DEPTH)+1; i = i + 1)begin:wr_ptr_g_sync2
            sync_cell wr_ptr_sync (
                .clk_d                (rclk                         ),
                .rstn_d               (r_rstn                       ),         
                .data_s               (wr_ptr_g[i]                  ),
                .data_d               (wr_ptr_g_sync[i]             )
            );
        end
    endgenerate
    //=====================================================================================
    

    //=====================================================================================
    //fifo data write in
    always@(posedge wclk)begin          
        if(wen&&!fifo_full)begin
            fifo_ram[wr_ptr_b[$clog2(FIFO_DEPTH)-1:0]] <= din; 
        end
    end

    //fifo data read out
    always@(posedge rclk)begin
        if(!fifo_empty)begin
            dout <= fifo_ram[rd_ptr_b[$clog2(FIFO_DEPTH)-1:0]];
        end
    end
    //=====================================================================================


    //=====================================================================================
    //full/empty flag
    assign fifo_full = (wr_ptr_b ^ rd_ptr_g_s_b) == {1'b1,{($clog2(FIFO_DEPTH)){1'b0}}};
        
    assign fifo_empty= (wr_ptr_g_s_b ^ rd_ptr_b) == {1'b0,{($clog2(FIFO_DEPTH)){1'b0}}};
    
    //=====================================================================================
endmodule


