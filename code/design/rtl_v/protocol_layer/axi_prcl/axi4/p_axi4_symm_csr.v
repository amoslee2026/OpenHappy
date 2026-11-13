// =============================================================================
// module name：p_axi4_symm_csr
// author: OpenHappy Team
// MAS：p_axi4_symm_csr_mas.md
// 功能概述: axi4_symm 单条对称链路协议层 APB 寄存器实现中枢（集中式 CSR，
//   aggregated）。APB3 Slave（地址 12bit，数据 32bit），解码 protocol layer 段
//   寄存器（基址 0x400）后以具名信号直送各子模块（credit / timeout / 单 lane
//   ovrflow clr / loopback 等），汇聚单链路中断 raw（单 lane b0/r0/w0/ar0/aw0
//   ovrflow + r0/w0 err/timeout + gb 包处理错误），输出合并中断 intr = (|intr_sts_l)|(|intr_sts_h)。
//   pready / prdata 均为寄存器输出，读写均 2-cycle 无等待；pslverr tie 0。
//   与 p_axi4_asymm_csr 相比，本模块为单条对称链路（无 req/resp 双侧 die 归属）：
//   - 不实例化 demux（0x410 为 ASYMM 双侧通道选择）与 AXI5 专属（0x404/0x418）
//   - lane_idle(0x420) / ots_idle(0x424) 用单链路简化字段
//   - 中断/清零仅保留单 lane x0 位（b0/r0/w0/ar0/aw0，err/timeout 仅 r0/w0）
//   - err/timeout id 仅低半寄存器（w0/r0），高半（0x454/0x45c/0x464/0x46c）不实例化
//   - 额外 0x500 loopback（remote_loop/local_loop，AXI4 SYMM 专属）
//   - 2026-09-23 新增（对齐 axi4_symm_csr.md）：0x600 hipi_sel（格式全局模式 RW）、
//     0x604 gb_status（RO：err_st=pkt_err / proc_st=rx_st，源 protocol_rx_ch_identify）、
//     intr_raw_h[16] gb_intr_raw / intr_clr_h[16] gb_intr_clr（GB 包处理中断，err_intr/err_clr）
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   2026-08-30  OpenHappy Team  初始版本：单链路对称寄存器（对齐 axi4_symm_csr.md）
//   2026-09-23  OpenHappy Team  按 axi4_symm_csr.md 新增：① 0x600 hipi_sel（RW bit0，1=hipi gb / 0=hipi next）
//                       ② GB 包处理通路：输入 gb_intr/pkt_err[2:0]/rx_st[3:0]、输出 gb_err_clr；
//                       intr_raw_h[16]=gb_intr、intr_clr_h[16] W1C→gb_err_clr（3 pclk 展宽，同既有 clr 惯例）、
//                       0x604 gb_status RO={7'd0,pkt_err,rx_st}（源 protocol_rx_ch_identify）
// =============================================================================


module p_axi4_symm_csr (
    // APB 总线（对外）
    input  wire        pclk,
    input  wire        presetn,
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [11:0] paddr,
    input  wire [31:0] pwdata,
    output wire [31:0] prdata,
    output wire        pready,
    output wire        pslverr,
    // 中断合并输出（对外）
    output wire        intr,
    // 配置输出（→ 子模块）
    output wire        credit_en,
    output wire        timeout_en,
    output wire [15:0] timeout_num,
    output wire        w0_timeout_clr, output wire r0_timeout_clr,
    output wire        w0_err_clr,     output wire r0_err_clr,
    // 溢出清零输出（intr_clr_l 0x448，单链路单 lane）
    output wire        r0_ovrflow_clr,
    output wire        b0_ovrflow_clr,
    output wire        aw0_ovrflow_clr, output wire ar0_ovrflow_clr,
    output wire        w0_ovrflow_clr,
    // 回环使能输出（loopback 0x500）
    output wire        remote_loop,
    output wire        local_loop,
    // hipi 格式全局模式（0x600 hipi_sel，2026-09-23 新增）
    output wire        hipi_sel,
    // GB 包处理清除输出（intr_clr_h[16] gb_intr_clr W1C → protocol_rx_ch_identify.err_clr）
    output wire        gb_err_clr,
    // 状态输入（← 子模块）；lane/ots idle 单链路
    input  wire        rx_r_ots_idle, input wire rx_w_ots_idle,
    input  wire        tx_r_ots_idle, input wire tx_w_ots_idle,
    input  wire        tx_lane_idle,  input wire rx_lane_idle,
    input  wire [9:0]  w0_timeout_id,
    input  wire [9:0]  r0_timeout_id,
    input  wire        w0_timeout_intr, input wire r0_timeout_intr,
    input  wire [9:0]  w0_err_id,
    input  wire [9:0]  r0_err_id,
    input  wire        w0_err_intr,   input wire r0_err_intr,
    // 溢出中断输入（intr_raw_l 0x430，单链路单 lane）
    input  wire        r0_ovrflow_intr,
    input  wire        b0_ovrflow_intr,
    input  wire        aw0_ovrflow_intr, input wire ar0_ovrflow_intr,
    input  wire        w0_ovrflow_intr,
    // GB 包处理状态/中断输入（← protocol_rx_ch_identify，2026-09-23 新增）
    input  wire        gb_intr,         // = err_intr = |pkt_err（电平保持至清除）
    input  wire [2:0]  gb_pkt_err,      // = pkt_err {r_err_r, w_err_r, s_err_r}
    input  wire [3:0]  gb_rx_st,        // = rx_st（FSM one-hot）
    // 协议层聚合信息（由顶层 PROTOCOL_LAYER_INFO 宏传入，0x400 RO 锁存回读）
    input  wire [31:0] protocol_layer_info
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    // APB_ADDR_W=12 / APB_DATA_W=32 固定（pclk 域 APB3，MAS/blueprint 未要求参数化）

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    // APB 相位组合信号
    wire apb_setup;
    wire apb_access;
    wire apb_wr;
    assign apb_setup  = psel & ~penable;
    assign apb_access = psel & penable;
    assign apb_wr     = apb_access & pwrite;

    // RO 组合值（无独立存储寄存器）
    // intr_raw_l：bit[16] = b0 ovrflow，bit[12] = r0 ovrflow，bit[8] = w0 ovrflow，
    //              bit[4] = ar0 ovrflow，bit[0] = aw0 ovrflow（仅单 lane x0）
    wire [31:0] intr_raw_l;
    // intr_raw_h：bit[16] = gb，bit[12] = r0 err，bit[8] = w0 err，bit[4] = r0 timeout，bit[0] = w0 timeout
    wire [31:0] intr_raw_h;
    wire [31:0] intr_sts_l;
    wire [31:0] intr_sts_h;

    // RW 寄存器
    reg        credit_en_reg;   // 0x414 credit_en（bit0 RW）
    reg        timeout_en_reg;
    reg [15:0] timeout_num_reg;
    reg [31:0] intr_mask_l_reg;
    reg [31:0] intr_mask_h_reg;
    reg [1:0]  loopback_reg;    // 0x500 loopback（bit1=remote_loop, bit0=local_loop）
    reg        hipi_sel_reg;    // 0x600 hipi_sel（bit0 RW，1=hipi gb / 0=hipi next）

    // prdata / pready 寄存器输出
    reg [31:0] prdata_reg;
    reg        pready_reg;

    // clr 寄存器（自清零脉冲）
    // ovrflow_clr：bit[16]=b0, bit[12]=r0, bit[8]=w0, bit[4]=ar0, bit[0]=aw0（单 lane）
    reg [16:0] ovrflow_clr_reg;
    // timeout_err_clr：bit[16]=gb, bit[12]=r0_err, bit[8]=w0_err, bit[4]=r0_timeout, bit[0]=w0_timeout
    reg [16:0] timeout_err_clr_reg;

    // clr 展宽寄存器：dly1/dly2 逐拍移位，out_r = clr_reg | dly1 | dly2（clr 输出 3 pclk 持续，
    //   保证被下游 sync_cell 2FF 可靠捕获；out_r 为寄存器输出）
    reg [16:0] ovrflow_clr_dly1_reg;
    reg [16:0] ovrflow_clr_dly2_reg;
    reg [16:0] ovrflow_clr_out_r;
    reg [16:0] timeout_err_clr_dly1_reg;
    reg [16:0] timeout_err_clr_dly2_reg;
    reg [16:0] timeout_err_clr_out_r;

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================
    // 中断 raw 聚合（纯组合，直接跟随输入）
    assign intr_raw_l = {15'h0,
                         b0_ovrflow_intr,
                         3'h0,
                         r0_ovrflow_intr,
                         3'h0,
                         w0_ovrflow_intr,
                         3'h0,
                         ar0_ovrflow_intr,
                         3'h0,
                         aw0_ovrflow_intr};
    assign intr_raw_h = {15'h0,
                         gb_intr,
                         3'h0,
                         r0_err_intr,
                         3'h0,
                         w0_err_intr,
                         3'h0,
                         r0_timeout_intr,
                         3'h0,
                         w0_timeout_intr};

    assign intr_sts_l = intr_raw_l & ~intr_mask_l_reg;
    assign intr_sts_h = intr_raw_h & ~intr_mask_h_reg;

    // 中断合并输出（纯组合，送协议层顶层 / CPU）
    assign intr = (|intr_sts_l) | (|intr_sts_h);

    // pslverr tie 0
    assign pslverr = 1'b0;

    // 字段分发（RW 寄存器解码输出 → 具名信号）
    assign timeout_en  = timeout_en_reg;
    assign timeout_num = timeout_num_reg;
    assign credit_en   = credit_en_reg;
    assign remote_loop = loopback_reg[1];
    assign local_loop  = loopback_reg[0];
    assign hipi_sel    = hipi_sel_reg;

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================
    // prdata 寄存器输出（SETUP 拍按 paddr 锁存）
    always @(posedge pclk or negedge presetn) begin
        if (!presetn)
            prdata_reg <= 32'h0;
        else if (apb_setup) begin
            case (paddr[11:2])
                10'h100: prdata_reg <= protocol_layer_info;                            // 0x400 RO
                10'h105: prdata_reg <= {31'h0, credit_en_reg};                         // 0x414 RW
                10'h108: prdata_reg <= {30'h0, rx_lane_idle, tx_lane_idle};            // 0x420 RO
                10'h109: prdata_reg <= {28'h0, rx_r_ots_idle, rx_w_ots_idle,
                                        tx_r_ots_idle, tx_w_ots_idle};                 // 0x424 RO
                10'h10a: prdata_reg <= {31'h0, timeout_en_reg};                        // 0x428 RW
                10'h10b: prdata_reg <= {16'h0, timeout_num_reg};                       // 0x42c RW
                10'h10c: prdata_reg <= intr_raw_l;                                     // 0x430 RO
                10'h10d: prdata_reg <= intr_raw_h;                                     // 0x434 RO
                10'h10e: prdata_reg <= intr_sts_l;                                     // 0x438 RO
                10'h10f: prdata_reg <= intr_sts_h;                                     // 0x43c RO
                10'h110: prdata_reg <= intr_mask_l_reg;                                // 0x440 RW
                10'h111: prdata_reg <= intr_mask_h_reg;                                // 0x444 RW
                // 0x448 / 0x44c intr_clr（W1C，读返回 0）
                10'h114: prdata_reg <= {22'h0, w0_timeout_id};                        // 0x450
                10'h116: prdata_reg <= {22'h0, r0_timeout_id};                        // 0x458
                10'h118: prdata_reg <= {22'h0, w0_err_id};                            // 0x460
                10'h11a: prdata_reg <= {22'h0, r0_err_id};                            // 0x468
                10'h140: prdata_reg <= {30'h0, loopback_reg};                         // 0x500 RW
                10'h180: prdata_reg <= {31'h0, hipi_sel_reg};                        // 0x600 RW
                10'h181: prdata_reg <= {25'h0, gb_pkt_err, gb_rx_st};                // 0x604 RO（err_st[6:4]=pkt_err, proc_st[3:0]=rx_st）
                default: prdata_reg <= 32'h0;                                          // 未映射 / 非法地址返回 0
            endcase
        end
    end
    assign prdata = prdata_reg;

    // pready 寄存器输出（SETUP 触发下一拍置 1，ACCESS 后回 0）
    always @(posedge pclk or negedge presetn) begin
        if (!presetn)
            pready_reg <= 1'b0;
        else if (apb_setup)
            pready_reg <= 1'b1;     // SETUP → 下拍（ACCESS）置 1
        else if (apb_access)
            pready_reg <= 1'b0;     // ACCESS 完成后回 0
    end
    assign pready = pready_reg;

    // RW 寄存器写入（ACCESS 当拍执行；每寄存器独立 always 块）
    always @(posedge pclk or negedge presetn) begin
        if (!presetn)
            credit_en_reg <= 1'b0;
        else if (apb_wr && (paddr[11:2] == 10'h105))   // 0x414
            credit_en_reg <= pwdata[0];
    end
    always @(posedge pclk or negedge presetn) begin
        if (!presetn)
            timeout_en_reg <= 1'b0;
        else if (apb_wr && (paddr[11:2] == 10'h10a))
            timeout_en_reg <= pwdata[0];
    end
    always @(posedge pclk or negedge presetn) begin
        if (!presetn)
            timeout_num_reg <= 16'h100;   // 复位 0x100
        else if (apb_wr && (paddr[11:2] == 10'h10b))
            timeout_num_reg <= pwdata[15:0];
    end
    always @(posedge pclk or negedge presetn) begin
        if (!presetn)
            intr_mask_l_reg <= 32'h0;
        else if (apb_wr && (paddr[11:2] == 10'h110))
            intr_mask_l_reg <= pwdata;
    end
    always @(posedge pclk or negedge presetn) begin
        if (!presetn)
            intr_mask_h_reg <= 32'h0;
        else if (apb_wr && (paddr[11:2] == 10'h111))
            intr_mask_h_reg <= pwdata;
    end
    always @(posedge pclk or negedge presetn) begin
        if (!presetn)
            loopback_reg <= 2'h0;
        else if (apb_wr && (paddr[11:2] == 10'h140))   // 0x500
            loopback_reg <= pwdata[1:0];
    end

    always @(posedge pclk or negedge presetn) begin
        if (!presetn)
            hipi_sel_reg <= 1'b0;
        else if (apb_wr && (paddr[11:2] == 10'h180))   // 0x600
            hipi_sel_reg <= pwdata[0];
    end

    // ovrflow_clr 自清零脉冲（intr_clr_l 0x448 → ovrflow_clr，单 lane）
    // T2(ACCESS) 写入 clr_reg（bit[16]=b0, bit[12]=r0, bit[8]=w0, bit[4]=ar0, bit[0]=aw0），
    // T3 输出 1-cycle 脉冲后自清零
    always @(posedge pclk or negedge presetn) begin
        if (!presetn)
            ovrflow_clr_reg <= 17'h0;
        else if (|ovrflow_clr_reg)
            // 已置位的 bit 自清零（产生 1-cycle 脉冲）
            ovrflow_clr_reg <= 17'h0;
        else if (apb_wr && (paddr[11:2] == 10'h112))   // 0x448
            ovrflow_clr_reg <= {pwdata[16], 3'h0, pwdata[12], 3'h0, pwdata[8], 3'h0,
                                pwdata[4], 3'h0, pwdata[0]};
    end
    // ovrflow_clr 展宽：dly1/dly2 逐拍移位，out_r = clr_reg|dly1|dly2（3 pclk 持续，寄存器输出）
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) ovrflow_clr_dly1_reg <= 17'h0;
        else         ovrflow_clr_dly1_reg <= ovrflow_clr_reg;
    end
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) ovrflow_clr_dly2_reg <= 17'h0;
        else         ovrflow_clr_dly2_reg <= ovrflow_clr_dly1_reg;
    end
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) ovrflow_clr_out_r <= 17'h0;
        else         ovrflow_clr_out_r <= ovrflow_clr_reg | ovrflow_clr_dly1_reg | ovrflow_clr_dly2_reg;
    end
    assign b0_ovrflow_clr   = ovrflow_clr_out_r[16];
    assign r0_ovrflow_clr   = ovrflow_clr_out_r[12];
    assign w0_ovrflow_clr   = ovrflow_clr_out_r[8];
    assign ar0_ovrflow_clr  = ovrflow_clr_out_r[4];
    assign aw0_ovrflow_clr  = ovrflow_clr_out_r[0];

    // timeout_clr / err_clr / gb clr 自清零脉冲（intr_clr_h 0x44c，单 lane）
    // bit[16] = gb, bit[12] = r0 err, bit[8] = w0 err, bit[4] = r0 timeout, bit[0] = w0 timeout
    always @(posedge pclk or negedge presetn) begin
        if (!presetn)
            timeout_err_clr_reg <= 17'h0;
        else if (|timeout_err_clr_reg)
            timeout_err_clr_reg <= 17'h0;
        else if (apb_wr && (paddr[11:2] == 10'h113))   // 0x44c
            timeout_err_clr_reg <= {pwdata[16], 3'h0, pwdata[12], 3'h0, pwdata[8], 3'h0, pwdata[4], 3'h0, pwdata[0]};   // [16:0] = gb/r0err/w0err/r0to/w0to 间隔 3'h0（17bit，lint W164b 修复）
    end
    // timeout_clr/err_clr 展宽：dly1/dly2 逐拍移位，out_r = clr_reg|dly1|dly2（3 pclk 持续，寄存器输出）
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) timeout_err_clr_dly1_reg <= 17'h0;
        else         timeout_err_clr_dly1_reg <= timeout_err_clr_reg;
    end
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) timeout_err_clr_dly2_reg <= 17'h0;
        else         timeout_err_clr_dly2_reg <= timeout_err_clr_dly1_reg;
    end
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) timeout_err_clr_out_r <= 17'h0;
        else         timeout_err_clr_out_r <= timeout_err_clr_reg | timeout_err_clr_dly1_reg | timeout_err_clr_dly2_reg;
    end
    assign w0_timeout_clr = timeout_err_clr_out_r[0];
    assign r0_timeout_clr = timeout_err_clr_out_r[4];
    assign w0_err_clr     = timeout_err_clr_out_r[8];
    assign r0_err_clr     = timeout_err_clr_out_r[12];
    assign gb_err_clr     = timeout_err_clr_out_r[16];   // → protocol_rx_ch_identify.err_clr（模块内 2FF 同步）

endmodule
