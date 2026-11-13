`timescale 1ns/1ps

`include "link_layer_defines.vh"

module link_rx_compat (
    // 时钟与复位
    input  wire                 mb_clk,
    input  wire                 mb_rst_n,

    // CSR（apb_csr 解码具名透传；s2p/bw 接口保留不消费，protocol 父直通）
    input  wire                 csr_perf_mode_i,
    input  wire                 csr_comp_mode_i,
    input  wire                 csr_protocol_mode_i,
    input  wire                 csr_s2p_mode_i,
    input  wire [1:0]           csr_bw_mode_i,
    input  wire                 csr_crc_en_i,
    input  wire                 csr_credit_en_i,
    input  wire [1:0]           csr_nack_mode_i,
    input  wire                 rx_active_i,
    input  wire                 csr_retry_en_i,          // RETRY_CFG_REG.RETRY_EN（门控 ACK/NACK/ARQ 生成）

    // lane0 原始 beat（父 udi_bypass_mux 输出 0T；兼容低 272 bit 有效）
    input  wire [1023:0]        lane0_data_i,
    input  wire                 lane0_valid_i,

    // credit_fifo 回读（父例化）
    input  wire [1023:0]        cf_flit_out_i,
    input  wire                 cf_flit_end_i,
    input  wire                 cf_pop_valid_i,
    input  wire [7:0]           cf_occupancy_i,

    // PAIF / 计数清空
    input  wire                 paif_rx_ready_i,
    input  wire                 rx_flit_clr_i,
    input  wire                 crc_err_clr_i,

    // credit_fifo push / pop（形态侧驱动）
    output wire                 cf_push_o,
    output wire [1023:0]        cf_flit_in_o,
    output wire                 cf_flit_end_o,
    output wire                 cf_pop_o,

    // 反向通道（→ 父 link_tx；兼容模式 NACK 即 ARQ(Request)）
    output wire                 ack_valid_o,
    output wire [7:0]           ack_seq_o,
    output wire                 nack_valid_o,
    output wire [7:0]           nack_seq_o,
    output wire [1:0]           dlp_linknum_o,
    output wire                 lcl_nack_valid_o,
    output wire [7:0]           lcl_nack_seq_o,
    output wire [1:0]           lcl_nack_type_o,

    // ARQ（本形态唯一带 ARQ 口；生成 → 父 link_tx 中继，接收 → 触发 link_tx ARQ(ACK)）
    output wire [255:0]         arq_req_flit_o,
    output wire                 arq_req_valid_o,
    output wire [15:0]          arq_req_rcv_ptr_o,
    output wire                 arq_req_rcv_valid_o,

    // PAIF RX 出口
    output wire [1023:0]        paif_rx_data_o,
    output wire                 paif_rx_valid_o,
    output wire [7:0]           dlv_ack_seq_o,
    output wire                 dlv_ack_o,

    // 状态 / 统计
    output wire                 seq_mismatch_o,
    output wire [31:0]          rx_flit_cnt_o,
    output wire [31:0]          crc_err_cnt_o
);

    // =========================================================================
    // 拼装 + flit_boundary（兼容 32B Flit，1T；Type/Len 字段 bit[3:0] 非全 0 锁定）
    // =========================================================================
    reg  [2:0]      asm_beat_cnt;      // Flit 内拍计数（兼容恒 0）
    reg             in_flit;           // Flit 内标志
    reg  [511:0]    compat_word;       // 兼容 512 bit 捕获（低 272 有效）
    reg             word_vld_d1;       // beat 接受一级延迟（对齐捕获）
    reg  [1023:0]   unit_asm;          // 单元稳定副本（beat 沿捕获保持）
    reg             lane_word_vld;
    wire            beat_nz;
    wire            beat_accept;
    wire            flit_last_beat;
    wire            word_set;
    wire            unit_set;
    wire [1023:0]   unit_next;

    assign beat_nz        = |lane0_data_i[3:0];
    assign beat_accept    = lane0_valid_i & rx_active_i & (in_flit | beat_nz);
    assign flit_last_beat = (asm_beat_cnt == 3'd0);
    assign word_set       = beat_accept;

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            asm_beat_cnt <= 3'd0;
        else if (beat_accept & flit_last_beat)
            asm_beat_cnt <= 3'd0;
        else if (beat_accept)
            asm_beat_cnt <= asm_beat_cnt + 3'd1;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            in_flit <= 1'b0;
        else if (beat_accept & flit_last_beat)
            in_flit <= 1'b0;
        else if (beat_accept)
            in_flit <= 1'b1;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            compat_word <= 512'd0;
        else if (beat_accept)
            compat_word <= lane0_data_i[511:0];
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            word_vld_d1 <= 1'b0;
        else
            word_vld_d1 <= word_set;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            unit_asm <= 1024'd0;
        else if (word_vld_d1)
            unit_asm <= {{(1024-512){1'b0}}, compat_word};
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            lane_word_vld <= 1'b0;
        else
            lane_word_vld <= word_vld_d1;
    end

    assign unit_next = {{(1024-272){1'b0}}, unit_asm[271:0]};
    assign unit_set  = lane_word_vld;

    // =========================================================================
    // stage u1：单元寄存 + 特征解析（兼容 comp_* / is_arq / SUB_TYPE）
    // =========================================================================
    reg  [1023:0]   unit_r1;
    reg             unit_vld_r1;
    reg             unit_idx_r1;
    reg             front_data_r1;
    reg             null_frame_r1;
    reg             null_back_r2;
    wire            null_back_1;
    wire            is_null_1;
    wire            is_data_1;
    wire [7:0]      hdr_seq_1;
    wire [1:0]      dlp_cmd_1;
    wire [7:0]      dlp_seq_1;
    wire [1:0]      dlp_linknum_1;
    wire [3:0]      comp_subtype_1;
    wire            comp_ack_1;
    wire            is_arq_1;
    wire            is_unit1_1;

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            unit_r1 <= 1024'd0;
        else if (unit_set)
            unit_r1 <= unit_next;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            unit_vld_r1 <= 1'b0;
        else
            unit_vld_r1 <= unit_set;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            unit_idx_r1 <= 1'b1;
        else if (unit_set & csr_perf_mode_i)
            unit_idx_r1 <= ~unit_idx_r1;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            front_data_r1 <= 1'b0;
        else if (unit_set & csr_perf_mode_i)
            front_data_r1 <= unit_r1[0];
    end

    // gdx_review 2026-09-15 Item4：NULL 判定改依 Header bit0 + payload 特征——
    //   性能：bit0（type）=0 即控制，性能模式控制 Flit 唯一即 NULL（仅 front 半拍判，
    //         back 经 null_frame_r1→null_back_1 递推）；
    //   兼容：Header bit0（Type）=0 且 Payload bit[3:0]==4'h0（NULL payload 全 0，
    //         与 ARQ 的 SUB_TYPE 0001/0010/0100 区分）。
    assign is_null_1      = csr_perf_mode_i & (~is_unit1_1 & ~unit_r1[0])
                          | (~csr_perf_mode_i & csr_comp_mode_i) & (~unit_r1[0] & (unit_r1[8:5] == 4'h0));
    assign is_data_1      = csr_perf_mode_i & (unit_r1[0] | front_data_r1)
                          | ~csr_perf_mode_i & unit_r1[0];

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            null_frame_r1 <= 1'b0;
        else if (unit_set & csr_perf_mode_i)
            null_frame_r1 <= is_null_1;   // 锁存本单元 NULL 判定，供 back 半拍 null_back_1 递推
    end

    assign null_back_1      = csr_perf_mode_i & is_unit1_1 & null_frame_r1;
    assign hdr_seq_1        = unit_r1[11:4];
    assign dlp_cmd_1        = unit_r1[1:0];
    assign dlp_seq_1        = unit_r1[11:4];
    assign dlp_linknum_1    = unit_r1[3:2];
    assign comp_subtype_1   = unit_r1[8:5];
    assign comp_ack_1       = unit_r1[1];
    assign is_unit1_1       = csr_perf_mode_i & unit_idx_r1;
    assign is_arq_1         = csr_perf_mode_i & (is_unit1_1 & (dlp_cmd_1 != 2'b00))
                              | ~csr_perf_mode_i & (~unit_r1[0] & (comp_subtype_1 != 4'd0));

    // =========================================================================
    // CRC9（内嵌例化，纯组合；本形态内嵌 crc9，无 crc16 asm_unit 交接）
    // =========================================================================
    wire            crc9_pass;
    wire            crc9_err;
    wire [8:0]      crc9_calc_unused;   // crc9 观测输出（DFx）
    wire            crc_pass_1;
    wire            crc_err_1;

    crc9_check u_crc9_check (
        .mb_clk            (mb_clk),
        .mb_rst_n          (mb_rst_n),
        .csr_comp_mode_i   (csr_comp_mode_i),
        .csr_crc_en_i      (csr_crc_en_i),
        .flit_i            (unit_r1[271:0]),
        .flit_valid_i      (unit_vld_r1 & csr_comp_mode_i),
        .pass_o            (crc9_pass),
        .err_o             (crc9_err),
        .crc9_calc_o       (crc9_calc_unused)
    );
    assign crc_pass_1 = crc9_pass;
    assign crc_err_1  = crc9_err;

    // =========================================================================
    // rx_seq_check（兼容收 1 数据 Flit 自增；不因 retrain 清零）
    // =========================================================================
    reg  [7:0]   rx_seq;
    reg  [7:0]   cur_flit_seq;
    wire         seq_chk_en;
    wire         seq_mismatch_1;
    wire         is_retx_unit0;

    assign is_retx_unit0 = 1'b0;
    assign seq_chk_en    = crc_pass_1 & csr_comp_mode_i & is_data_1;
    assign seq_mismatch_1 = seq_chk_en & (hdr_seq_1 != rx_seq);

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            rx_seq <= 8'd0;
        else if (seq_chk_en)
            rx_seq <= hdr_seq_1 + 8'd1;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            cur_flit_seq <= 8'd0;
        else if (unit_vld_r1 & csr_comp_mode_i)
            cur_flit_seq <= hdr_seq_1;
    end

    // =========================================================================
    // ACK / NACK 解析（兼容 Flit CRC9 通过拍：数据 Flit Header ACK bit）
    // =========================================================================
    wire  ack_p_set_1;
    // gdx_review 2026-09-15 Item6：ACK/NACK/ARQ(Request) 生成由 RETRY_EN 门控（crc_en 仅控制 CRC 校验）
    assign ack_p_set_1  = crc_pass_1 & csr_retry_en_i & csr_comp_mode_i & is_data_1 & comp_ack_1;

    // =========================================================================
    // stage u2：校验结果与解析结果寄存
    // =========================================================================
    reg  [1023:0]  unit_r2;
    reg            unit_vld_r2;
    reg            unit_idx_r2;
    reg            pass_r2;
    reg            err_r2;
    reg            null_r2;
    reg            data_r2;
    reg            arq_r2;
    reg            seq_mis_r2;
    reg  [7:0]     seq_r2;
    reg            ack_p_r2;
    reg  [7:0]     ack_seq_r2;
    reg  [1:0]     dlp_linknum_r2;
    reg  [3:0]     comp_subtype_r2;

    always @(posedge mb_clk) begin
        if (!mb_rst_n)      unit_r2 <= 1024'd0;
        else                unit_r2 <= unit_r1;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      unit_vld_r2 <= 1'b0;
        else                unit_vld_r2 <= unit_vld_r1;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      unit_idx_r2 <= 1'b0;
        else                unit_idx_r2 <= unit_idx_r1;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      pass_r2 <= 1'b0;
        else                pass_r2 <= crc_pass_1;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      err_r2 <= 1'b0;
        else                err_r2 <= crc_err_1;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      null_r2 <= 1'b0;
        else                null_r2 <= is_null_1;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      null_back_r2 <= 1'b0;
        else                null_back_r2 <= null_back_1;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      data_r2 <= 1'b0;
        else                data_r2 <= is_data_1;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      arq_r2 <= 1'b0;
        else                arq_r2 <= is_arq_1;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      seq_mis_r2 <= 1'b0;
        else                seq_mis_r2 <= seq_mismatch_1;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      seq_r2 <= 8'd0;
        else                seq_r2 <= hdr_seq_1;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      ack_p_r2 <= 1'b0;
        else                ack_p_r2 <= ack_p_set_1;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      ack_seq_r2 <= 8'd0;
        else                ack_seq_r2 <= dlp_seq_1;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      dlp_linknum_r2 <= 2'd0;
        else                dlp_linknum_r2 <= dlp_linknum_1;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      comp_subtype_r2 <= 4'd0;
        else                comp_subtype_r2 <= comp_subtype_1;
    end

    // =========================================================================
    // nack_initiate / ARQ（解析通道 + 本地生成通道分离）
    // =========================================================================
    wire [7:0] local_nack_seq;
    assign local_nack_seq  = {8{(csr_nack_mode_i == 2'b01)}} & seq_r2 | {8{~(csr_nack_mode_i == 2'b01)}} & rx_seq;
    assign ack_valid_o     = ack_p_r2;
    assign ack_seq_o       = ack_seq_r2;
    assign nack_valid_o    = 1'b0;
    assign nack_seq_o      = 8'd0;
    assign dlp_linknum_o   = dlp_linknum_r2;
    assign lcl_nack_valid_o = err_r2 & csr_retry_en_i & csr_perf_mode_i;
    assign lcl_nack_seq_o   = local_nack_seq;
    assign lcl_nack_type_o  = {2{(csr_nack_mode_i == 2'b01)}} & 2'b11 | {2{~(csr_nack_mode_i == 2'b01)}} & 2'b10;

    // ARQ(Request) 生成：本地 CRC9 失败（兼容）→ Pointer=rx_seq / SUB_TYPE=0001 送父中继
    assign arq_req_flit_o   = {236'd0, {8'd0, rx_seq}, 4'b0001};
    assign arq_req_valid_o  = err_r2 & csr_retry_en_i & csr_comp_mode_i;
    // 远端 ARQ(Request) 接收：CRC 通过 + SUB_TYPE=0001 → 触发 link_tx ARQ(ACK)
    assign arq_req_rcv_ptr_o   = unit_r2[24:9];
    assign arq_req_rcv_valid_o = arq_r2 & (comp_subtype_r2 == 4'b0001) & pass_r2 & csr_comp_mode_i;

    // =========================================================================
    // err_discard_ctrl + credit_fifo push（兼容收 ARQ(ACK) 退出；无 half2）
    // =========================================================================
    reg  err_discard;
    wire err_discard_set;
    wire err_discard_clr;
    wire push_mask;
    wire push_ok;

    assign err_discard_set = err_r2 & ~null_r2;
    assign err_discard_clr = (csr_comp_mode_i & pass_r2 & arq_r2 & (comp_subtype_r2 == 4'b0010));

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            err_discard <= 1'b0;
        else if (err_discard_clr)
            err_discard <= 1'b0;
        else if (err_discard_set)
            err_discard <= 1'b1;
    end

    assign push_mask = err_discard & ~arq_r2;
    assign push_ok   = unit_vld_r2 & pass_r2 & data_r2 & ~null_r2 & ~null_back_r2
                     & rx_active_i & ~push_mask;

    assign cf_push_o     = push_ok;
    assign cf_flit_in_o  = unit_r2;
    assign cf_flit_end_o = 1'b1;

    // =========================================================================
    // paif_rx_if（pop → PAIF 拆分；flit_end 单元末拍握手 → 交付 ACK）
    // =========================================================================
    reg  [1023:0]  paif_shift;
    reg            paif_end_r;
    reg            paif_beat_cnt;
    reg            paif_busy;
    reg  [7:0]     dlv_seq;
    wire           cf_empty;
    wire           paif_last_beat;
    wire           paif_beat_fire;
    wire           dlv_ack_set;

    assign cf_empty        = (cf_occupancy_i == 8'd0);
    `ifdef PAIF_INTF_512
        assign paif_last_beat = paif_beat_cnt;
    `else
        assign paif_last_beat = 1'b1;
    `endif
    assign paif_beat_fire  = paif_busy & paif_rx_valid_o & paif_rx_ready_i;
    assign dlv_ack_set     = paif_beat_fire & paif_last_beat & paif_end_r & csr_credit_en_i;
    assign cf_pop_o        = ~paif_busy & ~cf_empty;

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            paif_busy <= 1'b0;
        else if (cf_pop_valid_i)
            paif_busy <= 1'b1;
        else if (paif_beat_fire & paif_last_beat)
            paif_busy <= 1'b0;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            paif_shift <= 1024'd0;
        else if (cf_pop_valid_i)
            paif_shift <= cf_flit_out_i;
        else if (paif_beat_fire & ~paif_last_beat)
            paif_shift <= {512'd0, paif_shift[1023:512]};
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            paif_end_r <= 1'b0;
        else if (cf_pop_valid_i)
            paif_end_r <= cf_flit_end_i;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            paif_beat_cnt <= 1'b0;
        else if (cf_pop_valid_i)
            paif_beat_cnt <= 1'b0;
        else if (paif_beat_fire & ~paif_last_beat)
            paif_beat_cnt <= 1'b1;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            dlv_seq <= 8'd0;
        else if (dlv_ack_set)
            dlv_seq <= dlv_seq + 8'd1;
    end

    assign dlv_ack_o   = dlv_ack_set;
    assign dlv_ack_seq_o = dlv_seq;
    `ifdef PAIF_INTF_512
        assign paif_rx_data_o = {{512{1'b0}}, paif_shift[511:0]};
    `else
        assign paif_rx_data_o = paif_shift;
    `endif
    assign paif_rx_valid_o = paif_busy;

    // =========================================================================
    // 统计（32 bit 内部计数，父截低位上提）
    // =========================================================================
    reg [31:0] rx_flit_cnt;
    reg [31:0] crc_err_cnt;

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            rx_flit_cnt <= 32'd0;
        else if (rx_flit_clr_i)
            rx_flit_cnt <= 32'd0;
        else if (dlv_ack_set)
            rx_flit_cnt <= rx_flit_cnt + 32'd1;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            crc_err_cnt <= 32'd0;
        else if (crc_err_clr_i)
            crc_err_cnt <= 32'd0;
        else if (err_r2)
            crc_err_cnt <= crc_err_cnt + 32'd1;
    end

    assign seq_mismatch_o = seq_mis_r2;
    assign rx_flit_cnt_o  = rx_flit_cnt;
    assign crc_err_cnt_o  = crc_err_cnt;

endmodule
