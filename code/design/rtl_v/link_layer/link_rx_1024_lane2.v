// =============================================================================
// module name：link_rx_1024_lane2
// author: OpenHappy Team
// MAS：spec/link_layer/submods/link_rx/mas/link_rx_1024_lane2_mas.md（形态③）
// 功能概述: link_rx 父模块按「形态」拆分的形态③（性能 PAIF 1024 × 2 lane）
//           数据通路核心。父 lane_dskew 对齐合并字取单 128B 单元 / 单元特征解析 /
//           rx_seq_check / err_discard_ctrl / paif_rx_if（1024 直通）/ 统计 /
//           ACK-NACK 全链。CRC16 走 asm_unit 交接契约：输出 asm_unit_o /
//           asm_unit_valid_o 交父 crc16_check 单引擎，回接 crc16_pass_i /
//           crc16_err_i，内部不例化 crc16。
// 统一流水（组包→两路 push→credit_fifo→pop→PAIF 一体化，对齐用户两套时序+MAS v1.7/v1.15）：
//           去 D23（LL-20261015-DUALPUSH）：判通拍寄存级（asm_done_p/unit_hold/de/en_ck）整级
//           删除，push 提前至组包完成拍（E）同拍——两路严格隔离、仅入口 OR 汇合：路 1 =
//           桥接或重传 disable（~csr_retry_en_i）判通即当 T（E）直接 push（push_direct =
//           push_gate_nc & ~csr_retry_en_i，锥无矩阵/chk_pass）；路 2 = 非桥接非 UDI 重传
//           使能（csr_retry_en_i=1）E 拍组合同拍 chk_pass 后 push（push_chk = push_gate_nc &
//           chk_pass & csr_retry_en_i）；cf_push_o = push_direct | push_chk（credit_fifo 入口
//           OR）。pop=发送拍=push 次拍，present=pop 同拍组合（credit_fifo FWFT 组合直读
//           flit_out_o）。asm_done 仅用于单元边界分类（flit_end / unit_idx），不引入 push
//           串行延迟；u1/u2 寄存仅保留 ACK/NACK/统计/seq 影子决策，不阻塞 push；paif 无
//           busy 门控。收→push→pop 不插额外流水寄存器。crc 输入为纯数据 unit_comb，控制
//           判定在 crc 之外，仅路 2 的 chk_pass 门控 push_chk（矩阵外浅 AND），不污染 crc 数据位。
// 编码约束：同步复位；always 内 if ≤ 2；组合仅 assign；for 仅 generate。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复)
//  2026-10-15  OpenHappy Team  LL-20261015-DUALPUSH（user 两路隔离 + 去 D23，arch 评审 PASS）：
//                          push 改两路严格隔离、仅入口 OR 汇合——路 1（桥接或重传 disable，
//                          ~csr_retry_en_i）判通即当 T（E）直接 push（push_direct=push_gate_nc&
//                          ~csr_retry_en_i，锥无矩阵/chk_pass）、数据=纯 unit_comb；路 2（非桥接
//                          非 UDI 重传使能）E 拍组合同拍 chk_pass 后 push（push_chk=push_gate_nc&
//                          chk_pass&csr_retry_en_i）、数据=unit_e；cf_push_o=push_direct|push_chk；
//                          cf_flit_in_o={1024{~csr_retry_en_i}}&unit_comb|{1024{csr_retry_en_i}}&unit_e。
//                          去 D23：删 en_ck/unit_hold/asm_done_p/ud/de 与 2 个判定级 always，
//                          特征/影子/统计沿 asm_done（E 拍），push/影子/统计前移 1 拍。
//                          裁决点⑥ 必做门：err_discard_set/half2_set 补 & csr_retry_en_i、
//                          seq_chk_en 补 & csr_retry_en_i 显式化、crc_err_cnt 增量键补 &
//                          csr_retry_en_i（防路 1 僵尸计数）。ECC 单错纠正取源保留（unit_e）。
//                          MAS v1.7 / spec v1.15 已落盘。
//  2026-09-08  OpenHappy Team  统一流水改造：u1/u2 去串行、push=E 组合、pop=E+1、
//                           present=pop 组合（PAIF1024 直通），去 paif_busy。
//  2026-09-09  OpenHappy Team  D23：CRC pipe 集成——en_ck 常量折叠分拍，CRC 使能态
//                           决策对齐 E+1（ud/de 选择），禁用态仍 E；paif_rx_valid
//                           改由占用判定、与 paif_rx_ready 独立（断 in-out）。
//  2026-09-10  OpenHappy Team  对齐 MAS v1.3 §3④/§5.1 判通拍寄存级表述与用户两套时序（arch_prof
//                           F-01~F-05 已 PASS，OpenHappy Team 需求授权）：本形态 push 本就为
//                           判通拍（crc_en=1）/组包完成拍（crc_en=0）组合直出，无二级写接口
//                           打拍，本次仅同步注释与修订历史，无功能改动。
// =============================================================================

`timescale 1ns/1ps

`include "link_layer_defines.vh"

module link_rx_1024_lane2 (
    // 时钟与复位
    input  wire                 mb_clk,
    input  wire                 mb_rst_n,

    // CSR（apb_csr 解码具名透传）
    input  wire                 csr_perf_mode_i,
    input  wire                 csr_comp_mode_i,
    input  wire                 csr_protocol_mode_i,
    input  wire                 csr_s2p_mode_i,
    input  wire [1:0]           csr_bw_mode_i,
    input  wire                 csr_crc_en_i,
    input  wire                 csr_ecc_en_i,
    input  wire                 csr_credit_en_i,
    input  wire [1:0]           csr_nack_mode_i,
    input  wire                 rx_active_i,
    input  wire                 csr_retry_en_i,          // RETRY_CFG_REG.RETRY_EN（门控 ACK/NACK 解析与本地 NACK 生成）

    // 父 lane_dskew 对齐合并字（2 lane 对齐后 2048 bit；低 1024=lane0、高 1024=lane1）
    input  wire [2047:0]        dskew_data_i,
    input  wire                 dskew_valid_i,

    // 父 crc16_check 往返（asm_unit 交接契约）
    input  wire                 crc16_pass_i,
    input  wire                 crc16_err_i,
    // 父 ecc_check 回接（D23 E+1 打拍，判通于组包完成次拍回接；ECC_EN=1 采本回接）
    input  wire                 ecc_pass_i,
    input  wire                 ecc_err_i,
    input  wire [1023:0]        ecc_corr_unit_i,

    // credit_fifo 回读（父例化）
    input  wire [1023:0]        cf_flit_out_i,
    input  wire                 cf_flit_end_i,
    input  wire                 cf_pop_valid_i,
    input  wire [7:0]           cf_occupancy_i,

    // PAIF / 计数清空
    input  wire                 paif_rx_ready_i,
    input  wire                 rx_flit_clr_i,
    input  wire                 crc_err_clr_i,

    // CRC16 交接输出（→ 父 crc16_check.data_i / 校验有效）
    output wire [1023:0]        asm_unit_o,
    output wire                 asm_unit_valid_o,

    // credit_fifo push / pop（形态侧驱动）
    output wire                 cf_push_o,
    output wire [1023:0]        cf_flit_in_o,
    output wire                 cf_flit_end_o,
    output wire                 cf_pop_o,

    // 反向通道（→ 父 link_tx，性能模式无 ARQ）
    output wire                 ack_valid_o,
    output wire [7:0]           ack_seq_o,
    output wire                 nack_valid_o,
    output wire [7:0]           nack_seq_o,
    output wire [1:0]           dlp_linknum_o,
    output wire                 lcl_nack_valid_o,
    output wire [7:0]           lcl_nack_seq_o,
    output wire [1:0]           lcl_nack_type_o,

    // PAIF RX 出口（1024，1 拍 1 单元）
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
    // 拼装（dskew 合并字；s2p 恒 0，2 lane → 2 拍 / Flit）
    // 单 128 B 单元 = {lane1 低 512, lane0 低 512} 同拍合并 1024
    // =========================================================================
    wire [1023:0]   unit_merge;         // 同拍合并 128 B 单元（纯数据）

    // =========================================================================
    // 内部信号声明（统一前置，供 assign 引用：STX_VE_606 声明序）
    // =========================================================================

    // --- 拼装 ---
    reg               in_flit;             // Flit 内标志
    reg  [2:0]        asm_beat_cnt;        // Flit 内拍计数（仅边界分类）
    wire              beat_nz;
    wire              beat_accept;
    wire              flit_last_beat;
    wire [1023:0]     unit_comb;           // 组合单元合并字（纯数据）
    wire              asm_done;            // 组包完成脉冲 E

    // --- CRC16 交接（0 cycle 组合，同拍回接）---
    wire              crc_pass;
    wire              crc_err;
    wire              chk_pass;   // 统一判通判据（ECC_EN=1 用 ecc_pass_i 回接 / =0 沿用 crc_pass）
    wire              chk_err;    // 统一判错判据（ECC_EN=1 用 ecc_err_i 回接 / =0 沿用 crc_err）

    // --- 数据/判通（去 D23，E 拍同拍；两路唯一选择信号 csr_retry_en_i）---
    wire [1023:0]     unit_e;      // 数据选择结果（ECC_EN=1 用 ecc_corr_unit_i 纠正取源 / =0 用 unit_comb）
    wire              push_direct; // 路 1 = 桥接或重传 disable（~csr_retry_en_i）判通直接 push（锥无矩阵/chk_pass）
    wire              push_chk;    // 路 2 = 重传使能且 chk_pass：E 拍组合同拍 push（矩阵外浅 AND）
    wire              push_gate_nc;// 判通公共锥（不含 chk_pass/矩阵；push_gate 前身）

    // --- 特征 / 状态 ---
    reg               unit_idx;            // 单元序号：1=前(unit1)，0=后(unit0)；复位 1
    reg               front_d;             // 前单元 data 位
    reg               prev_null;           // 前单元 null 标志
    reg  [7:0]        rx_seq;
    reg  [7:0]        cur_flit_seq;
    reg               half2_pending;       // 后半 128B CRC 失败待补标
    reg  [7:0]        pending_seq;         // 待补 Flit 的 seq
    reg               err_discard;         // 错误丢弃标志

    wire              is_unit1;
    wire              is_null_c;
    wire              is_data_c;
    wire              is_arq_c;
    wire              null_back_c;
    wire [7:0]        hdr_seq;
    wire [1:0]        dlp_cmd;

    // --- rx_seq_check ---
    wire              is_retx_unit0;
    wire              seq_chk_en;
    wire              seq_mismatch;

    // --- ACK / NACK 解析（决策拍组合）---
    wire              ack_p_set;
    wire              nack_p_set;

    // --- half2 / err_discard ---
    wire              half2_set;
    wire              half2_clr;
    wire              err_discard_set;
    wire              err_discard_clr;

    // --- push（两路隔离：push 拍 = E 拍组包完成拍同拍组合直出，无二级写接口打拍）---
    wire              push_mask;
    wire              push_skip;

    // --- 影子输出（反向通道 / 统计 决策，不阻塞 push）---
    reg               ack_p_q;
    reg  [7:0]        ack_seq_q;
    reg               nack_p_q;
    reg  [7:0]        nack_seq_q;
    reg  [1:0]        dlp_linknum_q;
    reg               lcl_nack_q;
    reg  [7:0]        lcl_nack_seq_q;
    reg  [1:0]        lcl_nack_type_q;
    reg               seq_mismatch_q;

    // --- paif_rx_if（pop=E+1，present=pop 组合，PAIF1024 直通）---
    reg  [7:0]        dlv_seq;
    wire              dlv_ack_set;

    // --- 统计 ---
    reg  [31:0]       rx_flit_cnt;
    reg  [31:0]       crc_err_cnt;

    // =========================================================================
    // 拼装：组合单元合并字 unit_comb + 完成脉冲 asm_done（E）
    //   每 dskew 拍即一 128 B 单元，asm_done=beat_accept（每拍）
    // 注：asm_done 仅用于单元边界分类（flit_end / unit_idx），非 push 串行延迟
    // =========================================================================
    assign unit_merge     = {dskew_data_i[1535:1024], dskew_data_i[511:0]};
    assign beat_nz        = |unit_merge;
    assign beat_accept    = dskew_valid_i & rx_active_i & (in_flit | beat_nz);
    assign flit_last_beat = (asm_beat_cnt == 3'd1);

    assign unit_comb      = unit_merge;
    assign asm_done       = beat_accept;

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

    // =========================================================================
    // CRC16 交接：asm_unit_o = unit_comb（纯数据）→ 父 crc16_check；
    // 回接 crc16_pass_i / crc16_err_i 同拍（0 cycle 组合）。控制判定在 crc 之外，
    // 仅 crc_pass 门控 push，不污染 crc 输入数据位。
    // =========================================================================
    assign asm_unit_o       = unit_comb;
    assign asm_unit_valid_o = asm_done;
    assign crc_pass         = crc16_pass_i;
    assign crc_err          = crc16_err_i;

    // 去 D23（LL-20261015-DUALPUSH，MAS v1.7 §5.1）：删 en_ck/unit_hold/asm_done_p/de 判通拍
    //   寄存级，E 拍同拍组合直通——判断是有效 Flit（push_gate_nc）即当 T push + chk 同拍判通。
    //   数据级 unit_e 保留 ECC 单错纠正取源（HIG-2）；chk_pass/chk_err 均 E 拍同拍回接组合判通。
    assign unit_e       = {1024{ csr_ecc_en_i}} & ecc_corr_unit_i
                        | {1024{~csr_ecc_en_i}} & unit_comb;
    assign chk_pass     = csr_ecc_en_i & ecc_pass_i | ~csr_ecc_en_i & crc_pass;
    assign chk_err      = csr_ecc_en_i & ecc_err_i  | ~csr_ecc_en_i & crc_err;

    // =========================================================================
    // 单元特征组合（E 拍判通，依据 unit_e = {ecc_en}&ecc_corr_unit_i | {~ecc_en}&unit_comb）
    // =========================================================================
    assign is_unit1        = csr_perf_mode_i & unit_idx;
    // gdx_review 2026-09-15 Item4 终裁二：NULL = 控制(type bit0=0) 且 seq=8'hff。仅 front 半
    //   拍判（header 所在单元），back 半拍沿用 prev_null 递推（null_back_c）不判——back [15:0]
    //   为 DLP/CRC 槽非 header（CASE_7 根因：back 半判 NULL 误伤数据后半）。仅依 seq==ff 会与
    //   数据 seq=0xff（tx_seq 0~255 循环可取）撞车，故 + ~ud[0] 控制位锁定。
    assign is_null_c       = csr_perf_mode_i & ~unit_e[0] & (unit_e[11:4] == 8'hff);
    assign is_data_c       = csr_perf_mode_i & (unit_e[0] | front_d) | ~csr_perf_mode_i & unit_e[0];
    assign is_arq_c        = csr_perf_mode_i & (is_unit1 & (dlp_cmd != 2'b00))
                            | ~csr_perf_mode_i & (~unit_e[0] & (unit_e[8:5] != 4'd0));
    assign null_back_c     = csr_perf_mode_i & is_unit1 & prev_null;
    assign hdr_seq         = unit_e[11:4];
    assign dlp_cmd         = unit_e[1:0];

    // =========================================================================
    // rx_seq_check（E 拍组合；rx_seq / cur_flit_seq 同步更新于 asm_done 沿）
    // =========================================================================
    assign is_retx_unit0 = half2_pending & ~is_unit1 & (hdr_seq == pending_seq);
    assign seq_chk_en    = chk_pass & csr_retry_en_i & csr_perf_mode_i & ~is_unit1 & ~is_null_c & is_data_c
                         & ~is_retx_unit0
                         & (~err_discard | err_discard_clr);
    assign seq_mismatch  = seq_chk_en & (hdr_seq != rx_seq);

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            rx_seq <= 8'd0;
        else if (seq_chk_en)
            rx_seq <= hdr_seq + 8'd1;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            cur_flit_seq <= 8'd0;
        else if (asm_done & csr_perf_mode_i & ~is_unit1)
            cur_flit_seq <= hdr_seq;
    end

    // =========================================================================
    // ACK / NACK 解析（决策拍组合，unit1 CRC 通过拍）
    // =========================================================================
    // gdx_review 2026-09-15 Item6：ACK/NACK 解析与本地 NACK 生成由 RETRY_EN 门控
    //   （crc_en 仅控制 CRC 校验本身；retry_en 才控制差错/确认通路）
    assign ack_p_set  = chk_pass & csr_retry_en_i
                      & ( csr_perf_mode_i & (is_unit1 & (dlp_cmd == 2'b01))
                        | ~csr_perf_mode_i & (is_data_c & unit_e[1]) );
    assign nack_p_set = chk_pass & csr_retry_en_i & csr_perf_mode_i & is_unit1 & dlp_cmd[1];

    // 单元序号 / data 位 / null 前溯（每单元决策沿更新）
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            unit_idx <= 1'b1;
        else if (asm_done & csr_perf_mode_i)
            unit_idx <= ~unit_idx;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            front_d <= 1'b0;
        else if (asm_done & csr_perf_mode_i)
            front_d <= unit_e[0];
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            prev_null <= 1'b0;
        else if (asm_done & csr_perf_mode_i)
            prev_null <= is_null_c;
    end

    // =========================================================================
    // half2_pending / pending_seq（E 拍组合 + E 沿更新）
    // =========================================================================
    assign half2_set = chk_err & csr_retry_en_i & ~is_null_c & ~null_back_c & unit_idx & csr_perf_mode_i & ~err_discard;
    assign half2_clr = half2_pending & unit_idx & chk_pass & is_data_c & ~err_discard & (cur_flit_seq == pending_seq);

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            half2_pending <= 1'b0;
        else if (half2_clr)
            half2_pending <= 1'b0;
        else if (half2_set)
            half2_pending <= 1'b1;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            pending_seq <= 8'd0;
        else if (half2_clr)
            pending_seq <= 8'd0;
        else if (half2_set)
            pending_seq <= cur_flit_seq;
    end

    // =========================================================================
    // err_discard_ctrl（E 拍组合 + E 沿更新）
    // =========================================================================
    assign err_discard_set = chk_err & csr_retry_en_i & ~is_null_c & ~null_back_c;
    assign err_discard_clr = (csr_comp_mode_i & chk_pass & is_arq_c & (unit_e[8:5] == 4'b0010))
                           | (csr_perf_mode_i & chk_pass & is_data_c & ~is_null_c & (hdr_seq == rx_seq));

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            err_discard <= 1'b0;
        else if (err_discard_clr)
            err_discard <= 1'b0;
        else if (err_discard_set)
            err_discard <= 1'b1;
    end

    // =========================================================================
    // credit_fifo push（两路隔离，push 拍 = E 拍组包完成拍同拍组合直出，无 push_q 二级打拍）：
    //   路 1（桥接或重传 disable，~csr_retry_en_i）判通即当 T 直接 push，数据=纯 unit_comb；
    //   路 2（重传使能）E 拍同拍 chk_pass 后 push，数据=unit_e；cf_push_o = push_direct | push_chk
    //   （credit_fifo 入口 OR）。push_gate_nc 为判通公共锥（含 data/null/back/push 门控，
    //   不含 chk_pass/任何矩阵——路 1 结构不变量）。
    // =========================================================================
    assign push_mask = err_discard & ~is_arq_c;
    assign push_skip = half2_pending & ~unit_idx & (hdr_seq == pending_seq) & csr_perf_mode_i;
    assign push_gate_nc = asm_done & is_data_c & ~is_null_c & ~null_back_c
                        & rx_active_i & ~push_mask & ~push_skip;
    // 路 1 / 路 2 严格隔离，唯一选择信号 csr_retry_en_i（只进两路 AND / 数据选择 mux，不并入矩阵）：
    assign push_direct = push_gate_nc & ~csr_retry_en_i;
    assign push_chk    = push_gate_nc & chk_pass & csr_retry_en_i;
    assign cf_push_o   = push_direct | push_chk;

    // push 拍组合直出（无二级写接口打拍，对齐 MAS v1.7 §5.1）：
    //   路 1 push 数据 = 纯 unit_comb（浅，无矩阵入锥）；路 2 push 数据 = unit_e
    //   （ECC 单错纠正取源 / CRC 时 unit_comb）。静态 sel 数据 mux，矩阵只在路 2 operand。
    assign cf_flit_in_o  = {1024{~csr_retry_en_i}} & unit_comb
                         | {1024{ csr_retry_en_i}} & unit_e;
    assign cf_flit_end_o = csr_comp_mode_i | unit_idx;

    // =========================================================================
    // 影子输出寄存（反向通道 / 统计 决策，每单元 asm_done 沿采样；不阻塞 push）
    // =========================================================================
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      ack_p_q <= 1'b0;
        else if (asm_done)  ack_p_q <= ack_p_set;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      ack_seq_q <= 8'd0;
        else if (asm_done)  ack_seq_q <= hdr_seq;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      nack_p_q <= 1'b0;
        else if (asm_done)  nack_p_q <= nack_p_set;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      nack_seq_q <= 8'd0;
        else if (asm_done)  nack_seq_q <= hdr_seq;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      dlp_linknum_q <= 2'd0;
        else if (asm_done)  dlp_linknum_q <= unit_e[3:2];
    end
    always @(posedge mb_clk) begin
        // gdx_review 2026-09-15 Item6：本地 NACK 生成由 RETRY_EN 门控
        if (!mb_rst_n)      lcl_nack_q <= 1'b0;
        else if (asm_done)  lcl_nack_q <= chk_err & csr_perf_mode_i & csr_retry_en_i;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      lcl_nack_seq_q <= 8'd0;
        else if (asm_done)  lcl_nack_seq_q <= {8{(csr_nack_mode_i == 2'b01) & is_unit1 }} & cur_flit_seq
                                                                         | {8{(csr_nack_mode_i == 2'b01) & ~is_unit1}} & hdr_seq
                                                                         | {8{~(csr_nack_mode_i == 2'b01)         }} & rx_seq;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      lcl_nack_type_q <= 2'd0;
        else if (asm_done)  lcl_nack_type_q <= {2{(csr_nack_mode_i == 2'b01)}} & 2'b11 | {2{~(csr_nack_mode_i == 2'b01)}} & 2'b10;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      seq_mismatch_q <= 1'b0;
        else if (asm_done)  seq_mismatch_q <= seq_mismatch;
    end

    assign ack_valid_o      = ack_p_q;
    assign ack_seq_o        = ack_seq_q;
    assign nack_valid_o     = nack_p_q;
    assign nack_seq_o       = nack_seq_q;
    assign dlp_linknum_o    = dlp_linknum_q;
    assign lcl_nack_valid_o = lcl_nack_q;
    assign lcl_nack_seq_o   = lcl_nack_seq_q;
    assign lcl_nack_type_o  = lcl_nack_type_q;
    assign seq_mismatch_o   = seq_mismatch_q;

    // =========================================================================
    // paif_rx_if（pop=发送拍=push 次拍（crc_en=1 判通拍次拍 / crc_en=0 组包完成次拍）
    // 与 present 同拍组合；credit_fifo FWFT 组合直读；
    // PAIF1024 每单元 1 拍直送。无 paif_busy 门控）
    // =========================================================================
    // valid 仅由可用/占用判定（FIFO 占用>0），与 paif_rx_ready 无关（断
    // ready→valid 组合转发）；ready 仍作 pop/消费使能照常使用。
    assign cf_pop_o        = paif_rx_ready_i;
    assign paif_rx_valid_o = (cf_occupancy_i != 8'd0);
    assign paif_rx_data_o  = cf_flit_out_i;
    assign dlv_ack_set     = cf_pop_valid_i & cf_flit_end_i & csr_credit_en_i;

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            dlv_seq <= 8'd0;
        else if (dlv_ack_set)
            dlv_seq <= dlv_seq + 8'd1;
    end

    assign dlv_ack_o        = dlv_ack_set;
    assign dlv_ack_seq_o    = dlv_seq;

    // =========================================================================
    // 统计（32 bit 内部计数，父截低位上提）
    // =========================================================================
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
        else if (chk_err & csr_retry_en_i)
            crc_err_cnt <= crc_err_cnt + 32'd1;
    end

    assign rx_flit_cnt_o  = rx_flit_cnt;
    assign crc_err_cnt_o  = crc_err_cnt;

endmodule
