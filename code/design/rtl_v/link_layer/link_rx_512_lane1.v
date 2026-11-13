// =============================================================================
// module name：link_rx_512_lane1
// author: OpenHappy Team
// MAS：spec/link_layer/submods/link_rx/mas/link_rx_512_lane1_mas.md（形态①）
// 功能概述: link_rx 父模块按「形态」拆分的形态①（性能 PAIF 512 × 1 lane）
//           数据通路核心。拼装（s2p=0 低 512 两拍拼 128B）/ 单元特征解析 /
//           rx_seq_check / err_discard_ctrl / paif_rx_if（PAIF512 单管逐 bank
//           读序列，v1.9；end tag = back 首拍锁存 end_tag_r，dlv_ack =
//           commit&back_last&end_tag_r&credit_en）/
//           统计 / ACK-NACK 全链。CRC16 走 asm_unit 交接契约（同②③）：本形态
//           输出 asm_unit_o / asm_unit_valid_o 交父 crc16_check 单引擎，回接
//           crc16_pass_i / crc16_err_i，内部不例化 crc16。
// 统一流水（组包→两路 push→credit_fifo→pop→PAIF 一体化，对齐用户两套时序+MAS v1.14/v1.15）：
//           去 D23（LL-20261015-DUALPUSH）：判通拍寄存级（asm_done_p/unit_hold/unit_decide/
//           en_ck）整级删除，push 提前至组包完成拍（E）同拍——两路严格隔离、仅入口
//           OR 汇合：路 1 = 桥接或重传 disable（~csr_retry_en_i）判通即当 T（E）直接
//           push（push_direct = push_gate_nc & ~csr_retry_en_i，锥无矩阵/chk_pass）；路 2 =
//           非桥接非 UDI 重传使能（csr_retry_en_i=1）E 拍组合同拍 chk_pass 后 push
//           （push_chk = push_gate_nc & chk_pass & csr_retry_en_i）；cf_push_o = push_direct |
//           push_chk（credit_fifo 入口 OR）。pop=发送拍=push 次拍，present=pop 同拍组合
//           （credit_fifo FWFT 组合直读 flit_out_o）。asm_done 仅用于单元边界分类
//           （flit_end / unit_idx），不引入 push 串行延迟；u1/u2 寄存仅保留 ACK/NACK/
//           统计/seq 影子决策，不阻塞 push；paif 无 busy 门控。收→push→pop 不插额外
//           流水寄存器。crc 输入为纯数据 unit_comb，控制判定在 crc 之外，仅路 2
//           的 chk_pass 门控 push_chk（矩阵外浅 AND），不污染 crc 数据位。
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
//                          去 D23：删 en_ck/unit_hold/asm_done_p/unit_data/unit_decide 与 2 个判定级
//                          always，特征/影子/统计沿 asm_done（E 拍），push/影子/统计前移 1 拍。
//                          裁决点⑥ 必做门：err_discard_set/half2_set 补 & csr_retry_en_i、seq_chk_en
//                          补 & csr_retry_en_i 显式化、crc_err_cnt 增量键补 & csr_retry_en_i（防路 1
//                          僵尸计数）、front_seen_set 重指 = cf_push_o & ~unit_idx（路 1 front 入库
//                          亦跟踪）。ECC 单错纠正取源保留（unit_e）。MAS v1.14 / spec v1.15 已落盘。
//  2026-10-15  OpenHappy Team  LL-20261015-PUSHMASK-UNDRIVEN（c9c7 落码误删恢复）：恢复
//                          push_mask 驱动行（= (err_discard & ~err_discard_clr) & ~is_arq_c）——
//                          c9c7 落码误删使 push_mask 悬空 → push_gate/push_ok 被 X 门控 →
//                          数据 push 静止（r5b 全红绿线静止）。HEAD 基线 L510 原文整行恢复。
//  2026-10-13  OpenHappy Team  LL-20261013-EccCrcPipeRm-revert（DC reg2reg=51>36 G1 门禁 FAIL，
//                          G1 唯一回退 = 重新加 pipe，jq9 job 417467）：反向恢复 D23
//                          判通拍寄存级——恢复 en_ck（形态侧）/unit_hold/asm_done_p 声明 +
//                          两个 always + unit_decide；unit_e→unit_data、asm_done→unit_decide
//                          （全读点改回 unit_decide/unit_data）。保留 LL-20261013-c9/c18 的
//                          front_seen 二元组 / back 前提闸 / F3 dlv_ack 重发与 delivered-range
//                          （C4 前臂 ~seq_mismatch 读 unit_data/unit_decide 不变）。spec/MAS
//                          placeholder 反向修订由 designer 负责（D23 E+1 拍语义）。
//  2026-10-13  OpenHappy Team  LL-20261013-EccCrcPipeRm（user timing 放松 reg2reg≤36，去
//                          link_rx 校验后一级 pipe；arch 复审通过）：判通拍寄存级整级
//                          删除——删 en_ck（形态侧）/unit_hold/asm_done_p 寄存 + 两个 always；
//                          unit_data→unit_e（= {ecc_en}&ecc_corr_unit_i | {~ecc_en}&unit_comb，
//                          HIG-2 保留 ECC 纠正取源，cf_flit_in_o/特征/影子读 unit_e）；
//                          unit_decide→asm_done（E 拍判通，全读点改 asm_done）。crc/ecc
//                          校验矩阵併回 E 拍（check 模块同步减 D23）——push/影子/统计时序
//                          平移 -1T（push T3→T2、pop T4→T3）；rx_seq 推进拍随之前移。等价：
//                          bit-identity 逐拍等价（含 X 签名 + ECC 单错纠正实体值）。
//  2026-10-13  OpenHappy Team  LL-20261013-c9/c18 一体化落码（arch 已通过，MAS v1.12 契约同步）：
//                          PUSH 块整块恢复（front_seen 二元组 / back 前提闸 / F3 dlv_ack 重发）
//                          + C4 前臂 ~seq_mismatch（push_gate front 臂，复用既有 8-bit !=，+0 比较器）
//                          + F3 delivered-range wrap-safe 谓词（8-bit 模减 w∈[1,127]，新增 1 个
//                          8-bit magnitude comparator，reg2out 支 ≤10，与 C4 push 锥分锥）。
//                          push_skip / is_retx_unit0 保留精确 ==front_seen_seq（不并入 delivered-range）；
//                          无新增流水寄存器；?: 全部 {N{sel}} AND/OR 展开；credit_fifo.v / defines 零改动。
//  2026-09-08  OpenHappy Team  形态① CRC16 归属改走 asm_unit 交接→父 crc16_check
//                           回接（同形态②③），删除内部 crc16_check 例化。
//  2026-09-08  OpenHappy Team  统一流水改造：u1/u2 去串行、push=E 组合、pop=E+1、
//                           present=pop 组合（PAIF512 拆 2 拍），去 paif_busy。
//  2026-09-09  OpenHappy Team  D23：CRC pipe 集成——en_ck 常量折叠分拍，CRC 使能态
//                           决策对齐 E+1（unit_data/unit_decide 选择），禁用态仍 E；paif_rx_valid
//                           改由占用判定、与 paif_rx_ready 独立（断 in-out）。
//  2026-09-10  OpenHappy Team  D26：破关键路径（MAX 26→≤23）：beat_accept 拆
//                           beat_cont(flop)/beat_start(OR)，asm_done(s2p=0) 改由
//                           in_flit flop 派生（非零 OR 树仅 flit 首拍需要）；credit_fifo
//                           写接口打拍（push/flit_in/flit_end → E+1），断控制链直达
//                           mem 写，bank 写使能改由 flop 驱动。
//  2026-09-10  OpenHappy Team  对齐用户 T1~T4/T1~T2 两套时序与 MAS v1.4 §3④/§5.1（arch_prof
//                           F-01~F-05 已 PASS，OpenHappy Team 需求授权）：撤除 D26 的写接口
//                           二级打拍（push_q/flit_in_q/flit_end_q）。判通拍寄存级 =
//                           crc16 消费侧打 1 拍回接（crc_pass，crc16_check D23 内部流水）
//                           + asm_done_p（判通拍事件）/ unit_hold（组包完成拍 unit_comb
//                           寄存版）+ 特征判据（unit_data/unit_idx 于判通拍组合值）；push/flit_in/
//                           flit_end 于 push 拍（crc_en=1 判通拍 T3 / crc_en=0 组包完成拍
//                           T2）组合直出，不二次打拍。pop=push 次拍（T4/T2），present=pop
//                           同拍组合不变。
//  2026-09-11  OpenHappy Team  v1.5 级数优化（iter_8 A 类专项，对应 MAS v1.5 §3⑤ A-1~A-4 /
//                           analysis link_layer_iter8_timing_A_s2p_struct）：s2p=1 死分支
//                           结构性删除，纯组合不加 pipe——beats_per_flit 折叠常量 BPF_S2P0；
//                           beat_nz 恒低 512 相或（删 1024-bit OR 树与 mux，A-1）；
//                           unit_comb 恒 {当前拍低 512, lane_half} 组包拼接（A-2）；
//                           asm_done 恒 beat_cont & asm_beat_cnt[0]（flop 派生，断 OR 树
//                           →push 直连，消除 26 级穿透 credit_fifo 写，A-3）；lane_half 装载
//  2026-09-11  OpenHappy Team  Standard NACK back 单元错误恢复（BUGFIX 2026-09-11，
//                           iter_10 CASE_8 暴露）：rx_seq 期望序列号推进改 flit 级——
//                           front（unit0）通过不再推进，移至本 Flit 的 back（CRC1）
//                           验收通过拍自增（原 front-advance 使 back 出错时 rx_seq
//                           已越过出错 flit → Standard NACK（lcl_nack_seq=rx_seq）
//                           重传起始跳过出错 flit、坏 back 半永不重传、sb 数据丢失）。
//                           half2_clr 修复：重传后半 seq 判据由 hdr_seq（back 单元
//                           恒 0）改 cur_flit_seq（本 Flit 锁存 seq），对齐 MAS §4.2
//                           「重传后半 seq==pending_seq」；加 ~err_discard 门控（与
//                           push/rx_seq_adv 同判据）。无新增流水寄存器（纯组合判据
//                           调整，不加 pipe）。
//  2026-09-24  OpenHappy Team  LL-20260924-CRIFVO v1.9（对齐 MAS v1.9）：paif_rx_if 改
//                           PAIF512 单管逐 bank 读序列——废除 paif_second/paif_hold/
//                           paif_end（拆拍语义由 credit_fifo 读序列 b0→b1→b2→b3 承担）；
//                           valid = cf_pop_valid_i（仅 empty 控，M4 断组合环）；pop =
//                           commit = paif_rx_ready_i & cf_pop_valid_i；data = cf_flit_out_i
//                           直出；dlv_ack = commit & back_last & end_tag_r & credit_en
//                           （back 首/末拍由 cf_flit_end_i & ~end_back_prev / & end_back_prev
//                           派生，arch 方案 A，prev 更新门 = commit：flit_end toggle、
//                           ~flit_end→0）。端口集 / 协议拍序 / 链路总延迟零变化。
//  2026-09-29  OpenHappy Team  LL-TIMING-N1（DC 时序 N=1）：beat_nz 预 OR 上提父壳——
//                          新增输入口 lane0_data_nz_i（=|(loopback 选通后低 512)，
//                          父壳按分配律 sel&|pipe[511:0] | ~sel&|cpif[511:0] 预计算，
//                          位级等价）；旧内部 |lane0_data_i[511:0] 512 位 OR 树删除
//                          （loopback 选通控制不再骑 512 位数据进 OR 树——CLKICG
//                          sq_1→lane_half ICG 18→≤6）。beat_nz 语义零变化（flit 首拍
//                          非零判定）；MAS 端口集差异已回报 designer。
// =============================================================================

`timescale 1ns/1ps

`include "link_layer_defines.vh"

module link_rx_512_lane1 (
    // 时钟与复位
    input  wire                 mb_clk,
    input  wire                 mb_rst_n,

    // CSR（apb_csr 解码具名透传）
    input  wire                 csr_perf_mode_i,
    input  wire                 csr_comp_mode_i,
    input  wire                 csr_crc_en_i,
    input  wire                 csr_ecc_en_i,
    input  wire                 csr_credit_en_i,
    input  wire [1:0]           csr_nack_mode_i,
    input  wire                 rx_active_i,
    input  wire                 csr_retry_en_i,          // RETRY_CFG_REG.RETRY_EN（门控 ACK/NACK 解析与本地 NACK 生成）
    input  wire                 csr_loopback_en_i,       // LOOPBACK_CFG_REG.LOOPBACK_EN（父既有信号；loopback 形态 F3 禁频）

    // lane0 原始 beat（父 udi_bypass_mux 输出 0T 直通）
    input  wire [1023:0]        lane0_data_i,
    input  wire                 lane0_valid_i,
    input  wire                 lane0_data_nz_i,    // 低 512 非零（父壳预 OR；LL-TIMING-N1——loopback 选通只进 1 bit，不建 512 位 OR 树控制锥）

    // 父 crc16_check 往返（asm_unit 交接契约）
    input  wire                 crc16_pass_i,
    input  wire                 crc16_err_i,
    // 父 ecc_check 回接（D23 E+1 打拍，判通于组包完成次拍=T3 判通拍回接；ECC_EN=1 采本回接）
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
    // 编译期派生常量（本形态 PAIF 512 × 1 lane，s2p 恒 0）
    // =========================================================================
    localparam [2:0] BPF_S2P0 = 3'd4;      // s2p=0 每 Flit 拍数（低 512 两拍成 128B）；v1.5 常量折叠，BPF_S2P1 随 s2p=1 死分支删除

    // =========================================================================
    // 内部信号声明（统一前置，供 assign 引用：STX_VE_606 声明序）
    // =========================================================================

    // --- 拼装（lane0 原始 beat）---
    reg  [511:0]      lane_half;           // 单元低 512 前半（s2p=0 首拍锁存）
    reg  [2:0]        asm_beat_cnt;        // Flit 内拍计数（仅边界分类）
    reg               in_flit;             // Flit 内标志
    wire [2:0]        beats_per_flit;
    wire              beat_nz;
    wire              beat_cont;       // in-flit 续拍（in_flit flop 派生，短路径）
    wire              beat_start;      // flit 首拍启动（非零判定，仅首拍需要）
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
    wire              push_gate_nc;// 判通公共锥（含前端臂，不含 chk_pass/矩阵；push_gate 前身）

    // --- 特征 / 状态 ---
    reg               unit_idx;            // 单元序号：1=前(unit1)，0=后(unit0)；复位 1
    reg               front_d;             // 前单元 data 位
    reg               prev_null;           // 前单元 null 标志
    reg  [7:0]        rx_seq;
    reg  [7:0]        cur_flit_seq;
    reg               half2_pending;       // 后半 128B CRC 失败待补标
    reg  [7:0]        pending_seq;         // 待补 Flit 的 seq
    reg               err_discard;         // 错误丢弃标志
    reg               front_seen_valid;    // BUG-P1 [LL-20261010-R2G1-PUSH] front 已接受半交付存活标（置位=front 被 push）
    reg  [7:0]        front_seen_seq;      // BUG-P1 最近被接受 front 的 seq（复位 8'hFF，不随 back 完成清）

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
    wire              rx_seq_adv;

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
    // BUG-P1/P-2 / F3-c9 front_seen 组合与 delivered-range（handoff §4 P-1/P-2 + c9 Step3 定稿）
    wire              front_seen_set;
    wire              front_seen_bacl_clr;
    wire [7:0]        delivered_d0;
    wire [7:0]        delivered_d1;

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

    // --- paif_rx_if（pop=E+1，present=pop 组合，PAIF512 单管逐 bank 读序列）---
    reg               end_tag_r;       // back 首拍(=flit_end & ~prev)锁存的 flit_end（防 rd 相位回绕失真）
    reg               end_back_prev;   // 上一 commit 认领拍是否 back（arch 方案 A）
    reg  [7:0]        dlv_seq;
    wire              dlv_ack_set;
    wire              back_last;       // = cf_flit_end_i &  end_back_prev（b3 / compat b1）
    wire              cf_commit;       // 握手认领 = paif_rx_ready_i & cf_pop_valid_i
    // BUG-F3 duplicate 重发 ACK（handoff §5.5 + c9 Step3 delivered-range 定稿）
    wire              front_dup_c7;
    wire              back_dup_c7;
    wire              c7_ack_retx;

    // --- 统计 ---
    reg  [31:0]       rx_flit_cnt;
    reg  [31:0]       crc_err_cnt;

    // =========================================================================
    // 拼装：组合单元合并字 unit_comb + 完成脉冲 asm_done（E）
    //   v1.5 级数优化：s2p=1 死分支删除（PAIF512 s2p 恒 0，高 512 don't-care），
    //   纯组合不加 pipe。s2p=0 低 512 两拍拼 128B：unit_comb = {当前拍, 锁存前半}，
    //   asm_done=第 2 拍（flop 派生，断 OR 树→push 直连）
    // 注：asm_done 仅用于单元边界分类（flit_end / unit_idx），非 push 串行延迟；
    //     s2p/bw/protocol 模式口已删（原父透传不消费，2026-09-24 unused 清理）
    // =========================================================================
    // v1.5 级数优化：beats_per_flit 折叠常量 BPF_S2P0（=4），删 s2p 选择（A-4）
    assign beats_per_flit = BPF_S2P0;

    // v1.5 级数优化：beat_nz 恒低 512 相或——删 s2p=1 的 1024-bit OR 树与 mux，高 512
    // don't-care，不建输入逻辑锥（A-1）；LL-TIMING-N1：预 OR 上提父壳（lane0_data_nz_i，
    // 位级等价——OR 对公共选通分配律），本模块内 512 位 OR 树删除
    assign beat_nz        = lane0_data_nz_i;
    assign beat_cont      = lane0_valid_i & rx_active_i & in_flit;   // flop 派生，无 OR 树
    assign beat_start     = lane0_valid_i & rx_active_i & beat_nz;   // 低 512 OR 仅 flit 首拍用
    assign beat_accept    = beat_cont | beat_start;                  // 接受（状态更新用，含 OR）
    assign flit_last_beat = (asm_beat_cnt == (beats_per_flit - 3'd1));

    // 组合合并字（纯数据，不进控制位）：v1.5 恒组包拼接（删 s2p=1 全宽直通 mux，A-2）：
    //   unit_comb[1023:512]=当前拍低 512（128B 高半）、[511:0]=lane_half（低半），
    //   高 512 来自组包、非输入高 512（unit_data/unit_hold 高 512 仅作数据源，A-3）
    assign unit_comb      = {lane0_data_i[511:0], lane_half};
    // 完成脉冲 E：v1.5 恒 s2p=0 分支 asm_done = beat_cont & asm_beat_cnt[0]（flop 派生，
    // 不依赖非零 OR 树——非零判定只在 flit 首拍 in_flit=0 时真正需要，而首拍 cnt[0]=0
    // 不产出 asm_done）。删 s2p=1 分支 beat_accept（断 OR 树→push 直连，消除 26 级穿透
    // credit_fifo 写，A-3/A-4）
    assign asm_done       = beat_cont & asm_beat_cnt[0];

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
            lane_half <= 512'd0;
        // v1.5 级数优化：lane_half 装载条件去 s2p mux，恒 beat_accept & ~asm_beat_cnt[0]（A-4）
        else if (beat_accept & ~asm_beat_cnt[0])
            lane_half <= lane0_data_i[511:0];
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

    // 去 D23（LL-20261015-DUALPUSH，MAS v1.14 §5.1）：删 en_ck/unit_hold/asm_done_p/unit_decide
    //   判通拍寄存级，E 拍同拍组合直通——判断是有效 Flit（push_gate_nc）即当 T push + chk 同拍判通。
    //   数据级 unit_e 保留 ECC 单错纠正取源（HIG-2）；chk_pass/chk_err 均 E 拍同拍回接组合判通。
    assign unit_e       = {1024{ csr_ecc_en_i}} & ecc_corr_unit_i
                        | {1024{~csr_ecc_en_i}} & unit_comb;
    assign chk_pass     = csr_ecc_en_i & ecc_pass_i | ~csr_ecc_en_i & crc_pass;
    assign chk_err      = csr_ecc_en_i & ecc_err_i  | ~csr_ecc_en_i & crc_err;

    // =========================================================================
    // 单元特征组合（E 拍判通，依据 unit_e = {ecc_en}&ecc_corr_unit_i | {~ecc_en}&unit_comb）
    // =========================================================================
    assign is_unit1        = csr_perf_mode_i & unit_idx;
    // gdx_review 2026-09-15 Item4 终裁二（golden=spec/link_layer_spec.md §4.1.1.1/§4.1.3）：
    //   NULL = 控制 Flit(type bit0=0) 且 seq=8'hff。单依 seq==8'hff 不足——tx_seq 0~255 循环，
    //   数据 Flit seq 可合法取到 0xff（spec 只写 0~255，未声明数据不得取 ff），会与 NULL 撞车；
    //   且发送端已改为控制头 type=0（hdr_type_linknum_ctrl），因此收端以
    //   is_null_c = perf & ~type & (seq==8'hff) 双重锁定（控制 + ff）才无歧义。
    //   CASE_7 根因（designer 定位）是 DLP back 半无需判 NULL：back 半 [15:0] 为 DLP/CRC
    //   槽非 header（bit0 非 type、无 seq），本半不判 NULL，由 prev_null 递推 null_back_c
    //   覆盖整 Flit——初版把 ~bit0 落到 back 半才误伤数据后半（OVER-DELIVERY），
    //   与"front 判 NULL"本身无关。
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
    // BUG-P1 [LL-20261010-R2G1-PUSH] 判别器一般化（handoff §4 P-1）：重传 front 判定由
    //   half2_pending 扩为 front_seen 二元组 —— 重复 front 不参与 seq 校验（免 seq_mismatch
    //   误报骗 NACK）。half2 既有支保留；架构 F-3：精确 ==front_seen_seq（< 会误伤在途乱序）。
    assign is_retx_unit0 = csr_perf_mode_i & ~is_unit1 & ~is_null_c & is_data_c
                         & ( (half2_pending & (hdr_seq == pending_seq))
                           | (hdr_seq == front_seen_seq) );
    // 错误丢弃期（err_discard=1）在途后续 Flit（seq > 冻结的 rx_seq）不参与 seq 检查：
    //   rx_seq 冻结于错误点、不因被丢弃单元递进（spec §4.4.2.2 step3/6 只对「通过且
    //   seq==expected_seq 且被接受」的整 Flit 推进——BUGFIX 2026-09-11 起 rx_seq 推进
    //   移至 back 验收拍（rx_seq_adv），front 仅作 seq 校验/计数不再推进）。否则 rx_seq
    //   赶超错误点 → 重传好副本永不满足 hdr_seq==rx_seq（spec step12 恢复无门 → 卡死，
    //   旧 BUG_LOG ②③ 复现）。
    assign seq_chk_en    = chk_pass & csr_retry_en_i & csr_perf_mode_i & ~is_unit1 & ~is_null_c & is_data_c
                         & ~is_retx_unit0
                         & (~err_discard | err_discard_clr);
    assign seq_mismatch  = seq_chk_en & (hdr_seq != rx_seq);

    // =========================================================================
    // rx_seq 期望序列号推进——flit 级（整 Flit 验收后自增，spec §4.4.2.2 step3）
    //   Standard NACK back 单元错误恢复（BUGFIX 2026-09-11，iter_10 CASE_8 暴露）。
    //   原实现在 front 单元 CRC 通过即推进（rx_seq <= hdr_seq+1，seq_chk_en 含
    //   ~is_unit1）：back（CRC1）出错时 rx_seq 已被 front 赶超到 出错 flit+1 →
    //   Standard NACK（lcl_nack_seq_q=rx_seq）重传起始跳过出错 flit，坏 back 半永不
    //   重传（sb BIT-MISMATCH + LEFTOVER）。修复：front 通过不推进，推进时机移后 1 拍
    //   至本 Flit 的 back（CRC1）验收通过（cur_flit_seq 已于 front 拍锁存=本 Flit
    //   hdr_seq，back 单元自身 hdr_seq 恒 0 不可用）——back 出错时 rx_seq 仍等于出错
    //   Flit 的 seq → Standard/Select NACK 均指向出错 Flit、A 从该 seq 重传补齐；
    //   err_discard 冻结（~err_discard 门控）与 half2_pending/push_skip 恢复语义保持。
    //   判据与 push 的 back 接受判据一致（asm_done / data / ~null / ~null_back / 非丢弃期），
    //   数据 Flit 每对 front+back 验收恰一次 +1，无新增流水寄存器（不加 pipe）。
    assign rx_seq_adv = asm_done & chk_pass & csr_perf_mode_i & is_unit1 & is_data_c
                      & ~is_null_c & ~null_back_c & ~err_discard;

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            rx_seq <= 8'd0;
        else if (rx_seq_adv)
            rx_seq <= cur_flit_seq + 8'd1;
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
            unit_idx <= 1'b0;
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
    // 重传后半 pass & data & 本 Flit seq==pending_seq 才清（MAS §4.2：重传仅补写后半、
    //   unit1 校验通过后 push 并清 half2_pending）。back 单元自身 hdr_seq 恒 0（[15:0]
    //   为 DLP/CRC 槽而非 header），须以 cur_flit_seq（本 Flit 于 front 拍锁存的 seq）
    //   判定；~err_discard 门控＝后半确实「被接受」而非处于丢弃期（与 push/rx_seq_adv
    //   同判据）。原 (hdr_seq==pending_seq) 恒 0≠pending_seq → half2_pending 永不清，
    //   seq 回绕后误伤后续同 seq Flit（BUGFIX 2026-09-11，Standard NACK back 恢复配套）。
    assign half2_clr = half2_pending & unit_idx & chk_pass & is_data_c
                     & ~err_discard & (cur_flit_seq == pending_seq);

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
        else if (half2_set)
            pending_seq <= cur_flit_seq;
    end

    // =========================================================================
    // err_discard_ctrl（E 拍组合 + E 沿更新）
    // =========================================================================
    assign err_discard_set = chk_err & csr_retry_en_i & ~is_null_c & ~null_back_c;
    // 退出（清除）仅性能路径（MAS §1.2.1/§4.3，spec §4.4.2.2 step12）：收到正确
    //   seq==rx_seq 重传 Flit（CRC 通过）拉低。必须限定 ~is_unit1（FRONT）：出错 Flit 自身
    //   的 back（CRC1 好、hdr_seq==rx_seq）不得误触发清理，否则「错误丢弃后续 Flit」
    //   （spec step6）被旁路 → 在途 Flit 照常入库导致乱序/重复。
    assign err_discard_clr = (csr_comp_mode_i & chk_pass & is_arq_c & (unit_e[8:5] == 4'b0010))
                           | (csr_perf_mode_i & chk_pass & is_data_c & ~is_null_c
                              & ~is_unit1 & (hdr_seq == rx_seq));

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            err_discard <= 1'b0;
        else if (err_discard_clr)
            err_discard <= 1'b0;
        else if (err_discard_set)
            err_discard <= 1'b1;
    end

    // =========================================================================
    // front_seen 二元组（BUG-P1/P-2：handoff §4 P-1/P-2）+ delivered-range 距离（c9 Step3）：
    //   front_seen_valid = 半交付存活标，front_seen_seq = 最近被接受 front 的 seq。
    //   置位 = front 被实际 push；清除 = seq-tag 配对（该 Flit back 补齐 rx_seq_adv 或
    //   half2_clr）。arch A：set 分支优先于 clr（不短路）；arch C：判别以 valid 为主。
    //   delivered_d0/d1 = rx_seq − hdr_seq/cur_flit_seq 8-bit 模减（wrap-safe 半空间距离，
    //   F3 delivered-range 谓词用，w∈[1,127] 为已交付区间，255→0 跨环绕 immune）。
    // =========================================================================
    assign delivered_d0 = rx_seq - hdr_seq;
    assign delivered_d1 = rx_seq - cur_flit_seq;
    assign front_seen_set      = cf_push_o & ~unit_idx;
    assign front_seen_bacl_clr = (rx_seq_adv | half2_clr) & (cur_flit_seq == front_seen_seq) & front_seen_valid;

    always @(posedge mb_clk) begin
        if (!mb_rst_n)                front_seen_valid <= 1'b0;
        else if (front_seen_set)      front_seen_valid <= 1'b1;
        else if (front_seen_bacl_clr) front_seen_valid <= 1'b0;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)           front_seen_seq <= 8'hFF;
        else if (front_seen_set) front_seen_seq <= hdr_seq;
    end

    // =========================================================================
    // credit_fifo push（两路隔离，push 拍 = E 拍组包完成拍同拍组合直出，无 push_q 二级打拍）：
    //   路 1（桥接或重传 disable，~csr_retry_en_i）判通即当 T 直接 push，数据=纯 unit_comb；
    //   路 2（重传使能）E 拍同拍 chk_pass 后 push，数据=unit_e；cf_push_o = push_direct | push_chk
    //   （credit_fifo 入口 OR）。push_gate_nc 为判通公共锥（含 data/null/back/push 门控与 front/back 臂，
    //   不含 chk_pass/任何矩阵——路 1 结构不变量）。
    // =========================================================================
    // 退出丢弃的那一个重传单元（err_discard_clr=1）与正常一并 push——消除「清理 err_discard
    //   的拍自身仍被旧 err_discard 屏蔽 → 重传前端被丢、后半站错位单独入库」的同拍竞争。
    assign push_mask = (err_discard & ~err_discard_clr) & ~is_arq_c;
    // BUG-P1 [LL-20261010-R2G1-PUSH] push_skip 判别器一般化（handoff §4 P-1）：由「back CRC-error」
    //   扩为 front_seen —— 重复 front（half2 后补 or 已交付 front_seen）一律不 push（spec L933）。
    //   架构 F-3：精确 ==front_seen_seq（< 会误伤在途乱序，不并入 delivered-range）。
    //   c9 恢复判别（LL-20261014-c9c7）：err_discard_clr=1 拍该 front 真该 push（c9 恢复补
    //     被丢弃单元），true dup 恒 hdr_seq!=rx_seq → err_discard_clr=0 → skip 保留（half2 左支
    //     保持 distinct 语义，arch 裁定）。
    assign push_skip = csr_perf_mode_i & ~unit_idx & is_data_c & ~is_null_c & ~null_back_c
                     & ( (half2_pending & (hdr_seq == pending_seq))
                       | (hdr_seq == front_seen_seq) & ~err_discard_clr );
    // BUG-P2 [back 前提闸] + C4 前臂（c9）：push_gate 前后臂——
    //   front 臂（~unit_idx）追加 & ~seq_mismatch（C4：复用既有 8-bit !=，+0 新比较器，
    //     整窗重放旧 seq 的 re-push 被扼制；!= 对 255→0 环绕免疫）；
    //   back 臂（unit_idx）= front 已接受（front_seen_valid & cur_flit_seq==front_seen_seq）才放行
    //     （孤儿 back 抑制，c18/c8-B）。AND/OR only（?: 禁用 → {N{sel}} 折叠）。
    assign push_gate_nc = asm_done & is_data_c & ~is_null_c & ~null_back_c
                        & rx_active_i & ~push_mask & ~push_skip
                        & ( ( {1{~unit_idx}} & ~seq_mismatch )
                          | ( {1{ unit_idx}} & front_seen_valid & (cur_flit_seq == front_seen_seq) ) );
    // 路 1 / 路 2 严格隔离，唯一选择信号 csr_retry_en_i（只进两路 AND / 数据选择 mux，不并入矩阵）：
    assign push_direct = push_gate_nc & ~csr_retry_en_i;
    assign push_chk    = push_gate_nc & chk_pass & csr_retry_en_i;
    assign cf_push_o   = push_direct | push_chk;

    // push 拍组合直出（无二级写接口打拍，对齐 MAS v1.14 §5.1）：
    //   路 1 push 数据 = 纯 unit_comb（浅，无矩阵入锥）；路 2 push 数据 = unit_e
    //   （ECC 单错纠正取源 / CRC 时 unit_comb）。静态 sel 数据 mux，矩阵只在路 2 operand。
    assign cf_flit_in_o  = {1024{~csr_retry_en_i}} & unit_comb
                         | {1024{ csr_retry_en_i}} & unit_e;
    assign cf_flit_end_o = csr_comp_mode_i | unit_idx;

    // =========================================================================
    // 影子输出寄存（反向通道 / 统计 决策，每单元 asm_done 沿采样；不阻塞 push）
    // =========================================================================
    // MAS §5.8/§5.9 + SVA A-RX-02/03：ack/nack/lcl_nack 解析输出须为
    // 1-cycle 脉冲（当下 asm_done 拍有效即置位、无新 asm_done 自动清零），禁止电平保持——
    // 保持电平会把陈旧 ACK 经 link_ack_route 低索引优先级长期遮蔽新到达的
    // 其他 RX ACK（T1-4a C2/C3 返回链路 retire 停置根因）。seq/lnk/type 等
    // 伴随数据仍按 asm_done 采样保持（消费方在 ack_valid 脉冲窗内读取）。
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      ack_p_q <= 1'b0;
        else                ack_p_q <= asm_done & ack_p_set;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      ack_seq_q <= 8'd0;
        else                ack_seq_q <= hdr_seq;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      nack_p_q <= 1'b0;
        else                nack_p_q <= asm_done & nack_p_set;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      nack_seq_q <= 8'd0;
        else                nack_seq_q <= hdr_seq;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      dlp_linknum_q <= 2'd0;
        else                dlp_linknum_q <= unit_e[3:2];
    end
    always @(posedge mb_clk) begin
        // gdx_review 2026-09-15 Item6：本地 NACK 生成由 RETRY_EN 门控
        if (!mb_rst_n)      lcl_nack_q <= 1'b0;
        else                lcl_nack_q <= asm_done & chk_err & csr_perf_mode_i & csr_retry_en_i;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      lcl_nack_seq_q <= 8'd0;
        else                lcl_nack_seq_q <= {8{(csr_nack_mode_i == 2'b01) & is_unit1 }} & cur_flit_seq
                                            | {8{(csr_nack_mode_i == 2'b01) & ~is_unit1}} & hdr_seq
                                            | {8{~(csr_nack_mode_i == 2'b01)         }} & rx_seq;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      lcl_nack_type_q <= 2'd0;
        else                lcl_nack_type_q <= {2{(csr_nack_mode_i == 2'b01)}} & 2'b11 | {2{~(csr_nack_mode_i == 2'b01)}} & 2'b10;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      seq_mismatch_q <= 1'b0;
        else                seq_mismatch_q <= seq_mismatch;
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
    // paif_rx_if（pop=发送拍=push 次拍（crc_en=1 判通拍次拍 T4 / crc_en=0 用户 T2=本形态
    // 组包完成次拍）与 present 同拍组合；credit_fifo FWFT 组合直读 + ren 逐拍——
    // PAIF512 单管：每拍读序列单 bank 512 直出（cf_flit_out_i 低 512 有效），
    // 拆 2-beat 语义由 credit_fifo 读序列（b0→b1→b2→b3）承担；end tag 由 back
    // 首拍(=flit_end & ~prev)锁存 end_tag_r，防 rd 相位回绕失真。无 paif_busy 门控）
    // =========================================================================
    // valid 仅由 credit_fifo empty regout 控（cf_pop_valid_i），v1.9 M4 不含
    // pop_i/ready 派生，断 ready→valid 组合环；握手认领 cf_commit =
    // paif_rx_ready_i & cf_pop_valid_i（读序列逐拍推进）。
    assign cf_pop_o        = cf_commit;
    assign paif_rx_valid_o = cf_pop_valid_i;
    assign paif_rx_data_o  = cf_flit_out_i;              // 组合直出（父截低 512）
    assign cf_commit        = paif_rx_ready_i & cf_pop_valid_i;   // 握手认领
    assign back_last       = cf_flit_end_i &  end_back_prev;
    // BUG-F3 [LL-20261010-R2G1-PUSH] duplicate 重发 ACK（handoff §5.5 + c9 Step3 定稿，spec L933）：
    //   已完整交付 Flit 的重复 front/back 到达 → 仅重发 ACK（不重复 push，P-1/P-2 已抑制）。
    //   c9 Step3 delivered-range wrap-safe 谓词（MAS v1.12 §5.1.2）：
    //   w = rx_seq − hdr_seq/cur_flit_seq 8-bit 模减，已交付-过去 = (w!=0) & ~w[7]（半空间 1..127）；
    //   w==0（当前期望 seq 内）被 != 排除；w[7]==1（落后 ≥128 的“未来”成员）被 MSB 排除；
    //   255→0 跨环绕 immune；front_seen_seq != 8'hFF 兜复位/环绕残余；~front_seen_valid 分野 =
    //   半投递 vs 全投递（MAS §5.1.2）。新增 1 个 8-bit magnitude comparator（模减族，~2-4L，
    //   c7_ack_retx→dlv_ack_seq_o reg2out 支 ≤10，与 C4 push 锥分锥）。
    assign front_dup_c7 = csr_perf_mode_i & ~unit_idx & is_data_c & ~is_null_c & ~null_back_c
                        & (front_seen_seq != 8'hFF) & (delivered_d0 != 8'd0) & ~delivered_d0[7]
                        & ~front_seen_valid;
    assign back_dup_c7  = csr_perf_mode_i &  unit_idx & is_data_c & ~is_null_c & ~null_back_c
                        & (front_seen_seq != 8'hFF) & (delivered_d1 != 8'd0) & ~delivered_d1[7]
                        & ~front_seen_valid;
    // LL-20261014-c9c7：c7_ack_retx 追加 & ~csr_loopback_en_i（loopback 形态 F3 禁频；
    //   csr_loopback_en_i 浅端口末级晚合并入锥，不加重 front_dup_c7/back_dup_c7 magnitude 传播）
    assign c7_ack_retx  = csr_perf_mode_i & csr_credit_en_i & csr_retry_en_i & ~csr_loopback_en_i
                        & (front_dup_c7 | back_dup_c7);
    assign dlv_ack_set  = (cf_commit & back_last & end_tag_r & csr_credit_en_i) | c7_ack_retx;

    // end_back_prev 更新门 = commit（认领拍）：flit_end 时 toggle（first 0→1 / last
    // 1→0 边界清），~flit_end 时→0（front/兼容持续段）；ready=0 冻结无 commit → 保持
    always @(posedge mb_clk) begin
        if (!mb_rst_n)                       end_back_prev <= 1'b0;
        else if (cf_commit & cf_flit_end_i)  end_back_prev <= ~end_back_prev;
        else if (cf_commit)                  end_back_prev <= 1'b0;
    end

    // end_tag_r：每 commit 认领拍锁存 cf_flit_end_i（back 首拍=1 / front 拍=0）
    always @(posedge mb_clk) begin
        if (!mb_rst_n)      end_tag_r <= 1'b0;
        else if (cf_commit) end_tag_r <= cf_flit_end_i;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            dlv_seq <= 8'd0;
        else if (dlv_ack_set)
            dlv_seq <= dlv_seq + 8'd1;
    end

    assign dlv_ack_o        = dlv_ack_set;
    // arch 注意项 B：F3 支携带 front_seen_seq（选项 (a) 强制）+ C4 push 锥与 F3 锥分锥；
    //   AND/OR 数据臂展开（禁裸三目）；dlv_seq 自增保留（dlv_ack_set OR 后同一 always 覆盖）。
    assign dlv_ack_seq_o    = ( {8{c7_ack_retx}} & front_seen_seq ) | ( {8{~c7_ack_retx}} & dlv_seq );

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
