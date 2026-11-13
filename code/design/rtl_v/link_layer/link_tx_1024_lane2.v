// =============================================================================
// module name：link_tx_1024_lane2
// author: OpenHappy Team
// MAS/spec：spec/link_layer/submods/link_tx/mas/link_tx_1024_lane2_mas.md
//           上游 spec：spec/link_layer/link_layer_spec.md §4.2.3.2 / §4.2.3.3 / §4.2.5
// 功能概述: link_tx 内部 TX 位宽转换形态③（PAIF 1024 bit × CPIF 2 lane × s2p_mode=0）
//           等宽流式收发（PAIF 每拍 128B ↔ CPIF 2 lane 每拍并行各 64B 共 128B）+
//           1 个 PAIF 数据缓冲（1T，当拍 PAIF 数据 = 1 个完整 128B 单元）承载 2 lane 分发
//           （送 CRC / 自嵌字段）→ CPIF 2 lane 并行输出 → 入口反压 +
//           钟漂反压（8 拍）；无出口 FIFO；NULL / 重传复用 CRC 数据通道做位宽转换，
//           不 replay push。单 Flit 全延迟 3T（T1 收 front → T2 发 front+收 back →
//           T3 发 back）。
// NOTE: 端口数据位宽按 MAS §2.1 硬编码 1024（本形态 PAIF 恒 1024 × 2 lane 恒 2），
//       不派生自编译期 `PAIF_WIDTH / `LANE_COUNT 宏（当前 link_layer_defines.vh
//       编译期非对称结构 PAIF=512 / 1 lane，与本形态不匹配）；仅 `REPLAY_BUF_PTR
//       宏用于 replay 写指针位宽。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   2026-08-19  OpenHappy Team  初始版本（对应 link_tx_1024_lane2_mas.md v1.0）
//   2026-09-03  OpenHappy Team  设计修正：入口反压改为链式语义——cpif_tx_ready 不再作为直连
//                           入口反压条件，入口反压仅由 link_tx_active=0 / 钟漂反压 /
//                           PAIF 数据缓冲(1T)顶满 驱动；正常 front/back 装载不再与 send_allowed 相与
//                          （出口不可发时 PAIF 数据缓冲可吸收 1 拍 PAIF 数据（1 个完整 128B
//                           front），顶满后反压入口，无数据丢失）；NULL/重传（nv）装载仍与
//                           send_allowed 相与。
//   2026-09-04  OpenHappy Team  设计对齐 golden（spec §4.2.3.3）：出口反压仅由 link_tx_active=0 /
//                           钟漂反压触发，移除 send_allowed 中的 cpif_tx_ready_i 项（其不构成
//                           出口/入口反压，仅作状态机跳转/判断 + UDI 透传）。本形态端口保留
//                           不再被使用，仅标注说明。
//   2026-09-03  OpenHappy Team  ARCH：字段自嵌重构（取代父回喂 unit_ckd 注入，对齐 512 形态）——
//                           header / DLP / CRC 校验码 / NULL 数据 / 重传数据 改为本模块
//                           输入，由本模块在 PAIF 数据管线按拍位自嵌：3 路数据源
//                           （PAIF / NULL / 重传，优先级 重传>NULL>paif）做三路 CRC mux，
//                           经 header/dlpmux 后输出 crc_data_1024_lane2_o 送外部 crc16，
//                           CRC 校验码经 crc_chk_1024_lane2_i 同拍回接本模块，嵌入对应
//                           CRC 槽位（front→CRC0 / back→CRC1）。移除父回喂
//                           unit_ckd_1024_lane2_i 与合并 nv_flit_valid 注入路径；NULL/重传
//                           改传数据+front 标记；PAIF 接收改以父权威 paif_rx_accept_i 推进
//                           （避免 NULL/重传注入期父停收而本模块仍独立收）。
//   2026-09-03  OpenHappy Team  新增 unit_busy_1024_lane2_o（单元缓冲待发/在途标志）输出，
//                         供父模块 link_tx 门控 NULL/重传 front→back 装载节拍，避免在途
//                         单元被覆盖丢失（与 512 单元高半段在途标志同语义，BUG LOG 2026-09-02）。
// =============================================================================

`timescale 1ns/1ps

`include "link_layer_defines.vh"

module link_tx_1024_lane2 (
    // 时钟与复位（统一同步复位，mb_rst_n 低有效）
    input  wire                        mb_clk,
    input  wire                        mb_rst_n,

    // 配置接口（apb_csr 解码直送）
    input  wire                        csr_perf_mode_i,      // 1=性能模式（本模块仅此被父例化）
    input  wire                        csr_protocol_mode_i,  // 1=PAIF 模式；0=UDI 本模块不送数
    input  wire                        csr_s2p_mode_i,       // 本形态恒 0（接口保留，每 lane 高 512 恒 0）
    input  wire [1:0]                  csr_bw_mode_i,        // 本形态不使用（接口保留，恒全带宽）
    input  wire [15:0]                 csr_drift_period_i,   // 钟漂反压周期（cycle）

    // PAIF TX 入口（性能模式 1024bit/拍 = 1 拍收 1 个 128B 单元）
    input  wire [1023:0]               paif_tx_data_i,
    input  wire                        paif_tx_valid_i,
    output wire                        paif_tx_ready_o,      // 入口反压（1=可收 / 0=反压）
    input  wire                        paif_rx_accept_i,     // 父权威 PAIF 接收（含控制停收门）；
                                                              // 本模块的 beat/装载/front/back 均据此推进，
                                                              // 避免 NULL/重传注入期父停收而本模块仍独立收

    // 字段自嵌输入（父模块组合直送，本模块在 PAIF 数据管线按拍位嵌入）
    input  wire [15:0]                 header_1024_lane2_i,   // 前 128B Header 字段（front 拍嵌入 [15:0]；父按源选 exit_hdr）
    input  wire [15:0]                 dlp_1024_lane2_i,      // 后 128B DLP 字段（back 拍嵌入 [15:0]；父 dlp_field）
    input  wire [15:0]                 crc_chk_1024_lane2_i,  // CRC 校验码回接（外部 crc16 同拍组合结果；front→CRC0 / back→CRC1）
    input  wire [11:0]                 ecc_chk_1024_lane2_i,  // ECC 校验码回接（外部 ecc_generate 同拍组合结果；front→ECC0 / back→ECC1）；csr_ecc_en_i=1 时嵌 chk 槽 [1019:1008]（高 4 tie0）
    // NULL / 重传 直接数据输入（三路 CRC mux 源；重传 > NULL > paif）
    input  wire [1023:0]               null_data_1024_lane2_i, // NULL 半单元原始数据（payload，header/DLP 由本模块嵌）
    input  wire                        null_valid_1024_lane2_i,// NULL 半单元有效（front/back 事件；不 replay push，仅装载/CE）
    input  wire                        null_front_1024_lane2_i,// 1=front(CRC0) / 0=back(CRC1)（寄存半态 ~null_half，作 CRC 槽位/分槽；装载火线仍为 valid 事件）
    input  wire                        null_active_1024_lane2_i,// NULL 源选择（寄存 null_active：作 CRC 源/槽位 select，非装载事件）
    input  wire [1023:0]               retry_data_1024_lane2_i,// 重传半单元原始数据（replay pop，header/DLP 由本模块嵌）
    input  wire                        retry_valid_1024_lane2_i,// 重传半单元有效（front/back 事件；不 replay push，仅装载/CE）
    input  wire                        retry_front_1024_lane2_i,// 1=front(CRC0) / 0=back(CRC1)（寄存半态 ~retry_half，作 CRC 槽位/分槽；装载火线仍为 valid 事件）
    input  wire                        retry_active_1024_lane2_i,// 重传源选择（寄存 retry_active：作 CRC 源/槽位 select，非装载事件）
    // 注错 / CRC 使能（本模块在组装级执行；与父 CRC 语义一致）
    input  wire                        csr_crc_en_i,         // CRC 总使能（保留：TX 出口 CRC 槽一律嵌入校验码，不影响槽位）
    input  wire                        csr_ecc_en_i,         // ECC 总使能（RETRY_CFG_REG.ECC_EN，bit21）：chk 槽组装 mux 二选一——=1 嵌 ecc_chk 低 12 / =0 嵌 crc_chk 16bit；与 CRC_EN 互选（同时 ECC 优先）
    input  wire                        csr_inj_en_i,         // INJECT_ERR_LINKn_REG.INJECT_EN：注错使能（电平门控 EN=1 即注错；payload bit0，仅数据/重传不注 NULL）
    input  wire                        csr_inj_fb_i,         // INJECT_ERR_LINKn_REG.FRONT_BACK：0=front(CRC0)/1=back(CRC1)（选命中半单元，单元级 front/back 判定）

    // CRC 组装交接（→ 父模块 crc16_generate；三路 mux 选出的当前源 + header/DLP 嵌入、CRC 槽清 0）
    //   front/back 标记由本模块内部 crc_front_sel 自持（父仅需送 CRC、回嵌校验码，不需 front 标记）
    output wire [1023:0]               crc_data_1024_lane2_o, // 当前要算 CRC 的数据（无 valid，组合按拍采样）

    // replay push 上送（→ 父模块 replay_buffer）
    output wire                        replay_push_wen_1024_lane2_o,  // Flit 收满拍拉高 1 cycle
    output wire [`REPLAY_BUF_PTR-1:0]  replay_push_wptr_1024_lane2_o, // 写指针（数据 Flit push 递增）
    output wire [2047:0]               replay_push_flit_1024_lane2_o, // {后128B原始, 前128B带seq/Header}

    // CPIF TX 2 lane 出口（s2p=0：各 lane 低 512 有效、高 512 恒 0）
    output wire [1023:0]               cpif_tx_lane0_data_1024_lane2_o, // lane0=Byte0~63（单元低半）
    output wire                        cpif_tx_lane0_data_1024_lane2_vld_o,
    output wire [1023:0]               cpif_tx_lane1_data_1024_lane2_o, // lane1=Byte64~127（单元高半）
    output wire                        cpif_tx_lane1_data_1024_lane2_vld_o,
    output wire                        unit_busy_1024_lane2_o, // 单元缓冲待发/在途（1=正在发/待发单元未发完）
    input  wire                        cpif_tx_ready_i,      // 状态机跳转/判断条件（PHY ready）+ UDI 透传
                                                              //   （golden spec §4.2.3.3 / §8.5）：不构成出口/入口反压，
                                                              //   本形态（性能模式）不使用，保留端口仅为接口稳定
    input  wire                        link_tx_active_i      // 出口使能（S_NORMAL 时=1；=0 为出口反压条件）
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    // 依赖 link_layer_defines.vh：仅 `REPLAY_BUF_PTR（=5，含 1 bit flag 位，DEPTH_16 激活态）；
    // 数据位宽按 MAS §2.1 硬编码 1024 / 2048，不派生 `PAIF_WIDTH / `LANE_COUNT。
    localparam DRIFT_BACKOFF_CYCLES = 4'd8;   // 钟漂反压宽度（8 拍，MAS F-05）

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================

    // --------------------------------------------------------------------------
    // PAIF 入口接收 / 出口门控 / 反压条件（组合）
    // --------------------------------------------------------------------------
    wire        paif_rx_accept;      // PAIF 拍被接收（父权威 paif_rx_accept_i）
    wire        send_allowed;        // 出口可发（link active & ~钟漂；cpif_tx_ready 不参与出口反压）
    wire        reconv_en;           // 单元装载有效（正常 front/back 收满 或 NULL/重传注入）
    wire        unit_load;           // 本拍实际装载新单元（reconv & 出口可发）
    wire        unit_buf_not_empty;  // PAIF 数据缓冲有未发完单元且本拍无法发出 → 入口反压（MAS §3.2）
    wire        send_active;         // 正在发送（有未完成单元）
    wire        drift_trigger;       // 钟漂反压触发（周期到且 Flit 已发完）
    wire        backoff_last;        // 钟漂反压末拍
    wire [3:0]  backoff_next;        // 钟漂反压计数递减

    // --------------------------------------------------------------------------
    // 收齐级（PAIF 1024，单拍收齐 1 个 128B 单元）
    // --------------------------------------------------------------------------
    reg         paif_beat_cnt;       // 0=前128B / 1=后128B（DFX 观测点）
    wire        front_proc;          // 前 128B 收满拍（beat0）
    wire        back_proc;           // 后 128B 收满拍（beat1）
    wire        flit_complete;       // 整 Flit 收满（=back_proc，NULL/重传已过滤）
    wire [1023:0] unit_128b;         // 当前 128B 原始单元（PAIF 单拍数据）

    // --------------------------------------------------------------------------
    // PAIF 数据缓冲(1T)（128B 单元，非 FIFO）与三路 CRC mux + 字段自嵌
    // --------------------------------------------------------------------------
    reg  [1023:0] unit_buf_128b;     // PAIF 数据缓冲(1T)（本模块自嵌后完整单元）
    reg         unit_buf_pending;    // 缓冲内单元待发（装载后下一拍发出）
    wire        crc_front_sel;       // 当前拍 front/back：1=front(嵌 header/CRC0) / 0=back(嵌 DLP/CRC1)
    wire        paif_is_front;       // PAIF 源 front 指示（寄存 beat 判位，无 accept/反压；仅装载拍取用）
    wire [1023:0] crc_src_raw;       // 三路源原始数据（重传 > NULL > paif）
    wire [1023:0] crc_src_inj;       // 注错后原始数据（payload 逐位翻，仅数据/重传不注 NULL）
    wire        inj_beat;            // 注错事件拍（数据/重传，非 NULL）
    wire [1023:0] inj_mask;          // 注错掩码：payload bit0 = 单元 bit16（=1024'b1<<16；Header/DLP 占 [15:0]，与 512 同构）
    wire [15:0] field_embed;         // 按 front/back 选择的嵌入字段（front=header_i / back=dlp_i）
    wire [15:0] chk_slot_1024_lane2;       // 出口 chk 槽[1023:1008]（校验码 16bit；ECC=1 嵌 {4'h0,ecc_chk 低 12} / =0 嵌 crc_chk 16bit）
    wire [1023:0] unit_assembled;    // 自嵌后完整单元 {crc 槽, 注入 payload, header/DLP}（组合直出）

    // --------------------------------------------------------------------------
    // replay push 写指针（内部自管理，数据 Flit push 递增）
    // --------------------------------------------------------------------------
    reg  [`REPLAY_BUF_PTR-1:0] replay_wptr_cnt;

    // --------------------------------------------------------------------------
    // 钟漂反压（计数器 + 8 拍 backoff）
    // --------------------------------------------------------------------------
    reg  [15:0] drift_cnt;           // 自由运行周期计数器（不停机，DFX 观测）
    reg  [3:0]  drift_backoff_cnt;   // 反压剩余拍数（0=未反压）
    reg         clock_drift_stall;   // 钟漂反压进行中（=1 且反压出口/入口）

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // PAIF 收齐级：beat 计数 / front-back 收满 / 128B 组装
    //   本形态 PAIF=1024，单拍收齐 1 个 128B 单元，无需跨拍累积。
    //   PAIF 接收以父权威 paif_rx_accept_i 为准（其已含 paif_ready_ctl 的
    //   ~null/~retry/~arq 停收门）。原先本模块用自身 paif_tx_ready_o 独立推导，
    //   NULL/重传注入期父停收而本模块仍独立收 → spur front_proc/back_proc → 与注入
    //   装载冲突丢单元（BUG LOG 2026-09-02，与 512 同批修复）。
    // --------------------------------------------------------------------------
    assign paif_rx_accept = paif_rx_accept_i;
    assign unit_128b = paif_tx_data_i[1023:0];

    assign front_proc = csr_perf_mode_i & paif_rx_accept & (paif_beat_cnt == 1'b0);
    assign back_proc  = csr_perf_mode_i & paif_rx_accept & (paif_beat_cnt == 1'b1);
    // PAIF 源 front 指示：仅由寄存 beat 判位（无 accept/反压/控制），在 PAIF front 装载拍与
    //   front_proc 等值（front_proc=beat==0），back 拍（beat==1）取 0 —— 作 CRC front/back 槽位选择。
    assign paif_is_front = (paif_beat_cnt == 1'b0);
    assign flit_complete = back_proc;   // 后 128B 收满 = 整个 256B Flit 收齐（NULL/重传已过滤）

    // --------------------------------------------------------------------------
    // 入口反压（MAS §3.2 链式语义：两个出口反压条件 link_tx_active=0 / 钟漂反压，
    //   或内部 PAIF 数据缓冲(1T)顶满）。cpif_tx_ready 不再直连入口反压——出口不可发时
    //   PAIF 数据缓冲可吸收 1 拍 PAIF 数据（1 个完整 128B front），顶满（unit_buf_not_empty）
    //   后反压 PAIF 入口（链式传递且无数据丢失）。正常锁步流下缓冲装载后下一拍即发出
    //   （2 lane 并行 1 拍发完整 128B），故不阻塞入口。
    // --------------------------------------------------------------------------
    // 出口反压（golden spec §4.2.3.3）：仅由 link_tx_active=0 或 钟漂反压 触发；
    //   cpif_tx_ready_i（PHY ready 跳转/判断 + UDI 透传）不构成出口反压，不进入发送使能。
    assign send_allowed = link_tx_active_i & ~clock_drift_stall;
    assign unit_buf_not_empty = unit_buf_pending & ~send_allowed;
    assign paif_tx_ready_o = ~( ~link_tx_active_i
                              | clock_drift_stall
                              | unit_buf_not_empty );

    // 单元装载：正常=front/back 收满回喂；NULL/重传=对应 valid（三路源统一装载自嵌单元）。
    // 正常 front/back 装载不依赖出口可发（不与 send_allowed 相与）——当拍 PAIF 数据收满即
    // 存入 PAIF 数据缓冲，供下一拍出口可发时真实发出；出口不可发时 front 单元停在 PAIF
    // 数据缓冲（顶满 → 反压 PAIF 入口），实现"出口不可发 → PAIF 数据缓冲(1T)顶满 →
    // 反压入口"的链式语义且不丢数。
    // NULL/重传注入仍与 send_allowed 相与：反压期不装载，避免覆盖缓冲中待发单元
    //  （且父以 ~unit_busy 门控装载节拍）。
    assign reconv_en  = (front_proc | back_proc) | null_valid_1024_lane2_i | retry_valid_1024_lane2_i;
    assign unit_load  = (front_proc | back_proc)
                      | ((null_valid_1024_lane2_i | retry_valid_1024_lane2_i) & send_allowed);

    // --------------------------------------------------------------------------
    // 三路 CRC mux + 字段自嵌（取代父回喂 unit_ckd 注入）
    //   源选择：重传 > NULL > paif。NULL/重传一旦触发必须发完（父 paif_rx_accept_i
    //   含 ~null/~retry/~arq 停收门，且父以 ~unit_busy 门控装载节拍），其进行期 PAIF
    //   被停收（front/back 不动作），中途 PAIF 数据不插入拆断——故单拍仅一个源有效，
    //   优先级仲裁为语义保证。
    //   按拍位自嵌：front 拍嵌 header_i([15:0])、back 拍嵌 dlp_i，CRC 槽[1023:1008]清 0，
    //   输出 crc_data_1024_lane2_o 送外部 crc16；校验码经 crc_chk_1024_lane2_i 同拍回接，
    //   由 unit_assembled 嵌回 CRC 槽（front→CRC0 / back→CRC1）。
    //   注错（payload 逐位，仅数据/重传不注 NULL）：与父 crc 语义一致，翻转 crc_src_raw
    //   bit0/bit1，payload 取注入后、header/DLP 取字段嵌入——CRC 算在注入后 payload 上。
    // --------------------------------------------------------------------------
    // CRC 数据源/槽位选择仅用已寄存源状态（retry/null_active）与半态（retry/null_front 由父
    //   直连 ~retry_half/~null_half）、PAIF 寄存 beat 判位——不含 conv_accept/link_active，
    //   切断 link_active→CRC select→CRC16 树 的跨模块 28 级关键路径；装载/valid 仍由
    //   retry_valid/null_valid 事件驱动（reconv_en/CE、inj_beat 节拍、front_load/back_load 不变）。
    wire crc_sel_retry_1024l2 = retry_active_1024_lane2_i;
    wire crc_sel_null_1024l2  = ~retry_active_1024_lane2_i & null_active_1024_lane2_i;
    wire crc_sel_data_1024l2  = ~retry_active_1024_lane2_i & ~null_active_1024_lane2_i;
    assign crc_front_sel = crc_sel_retry_1024l2 & retry_front_1024_lane2_i
                          | crc_sel_null_1024l2  & null_front_1024_lane2_i
                          | crc_sel_data_1024l2  & paif_is_front;
    assign crc_src_raw  = {1024{crc_sel_retry_1024l2}} & retry_data_1024_lane2_i
                         | {1024{crc_sel_null_1024l2}}  & null_data_1024_lane2_i
                         | {1024{crc_sel_data_1024l2}}  & unit_128b;
    // 注错（v1.6 per-TX 独立控制，gdx_review Item 1）：每 TX INJECT_EN 电平门控（EN=1 即
    //   注错，无 trigger/臂化）+ FRONT_BACK 选半单元（0=front(CRC0)/1=back(CRC1)）——
    //   对注入前原始数据 crc_src_raw 施加 payload bit0 XOR（inj_mask=1<<16=单元 bit16，与
    //   512 同构；勿改字面 bit0 会破坏 Header/DLP[15:0]），仅数据/重传不注 NULL。装载火线
    //   （load 事件）之外的拍 unit_assembled 不被捕获/槽不装载，select/注入值无关紧要。
    //   FRONT_BACK 选半单元（crc_front_sel: 1=front/0=back），EN=1 电平门控；仅数据/重传不注 NULL。
    //   掩码收敛同 512：payload bit0 = 单元 bit16（Header/DLP 占 [15:0]），勿改字面 bit0（会破坏 Header/DLP）。
    assign inj_mask     = (1024'b1 << 16);
    assign inj_beat     = csr_inj_en_i & (csr_inj_fb_i ^ crc_front_sel) & ~null_active_1024_lane2_i;
    assign crc_src_inj  = {1024{inj_beat}} & (crc_src_raw ^ inj_mask) | {1024{~inj_beat}} & crc_src_raw;
    assign field_embed  = {16{ crc_front_sel}} & header_1024_lane2_i | {16{~crc_front_sel}} & dlp_1024_lane2_i;
    assign crc_data_1024_lane2_o  = {16'd0, crc_src_raw[1007:16], field_embed};
    // 自嵌后完整单元（组合直出，crc_chk / ecc_chk 同拍有效——crc16 / ecc 均纯组合 0 cycle）。
    //   TX 出口 chk 槽[1023:1008]按 csr_ecc_en_i 二选一嵌校验码（ECC_EN=1 嵌 ecc_chk 低 12、
    //   高 4 tie0；=0 嵌 crc16 校验码不依赖 csr_crc_en_i——CRC_EN=0 由收端不查兜底）。
    assign chk_slot_1024_lane2 = {16{csr_ecc_en_i}} & {4'h0, ecc_chk_1024_lane2_i}
                                | {16{~csr_ecc_en_i}} & crc_chk_1024_lane2_i;
    assign unit_assembled = {chk_slot_1024_lane2,
                             crc_src_inj[1007:16], field_embed};

    // --------------------------------------------------------------------------
    // replay push 上送（仅正常数据 Flit 收满产生，NULL/重传不 push）
    //   写数据 {back 原始(本拍 paif), front 带 seq/Header(本拍前装载的 unit_buf)}
    //   写使能 = flit_complete（back 收满拍）
    // --------------------------------------------------------------------------
    assign replay_push_wen_1024_lane2_o  = flit_complete;
    assign replay_push_flit_1024_lane2_o = {paif_tx_data_i[1023:0], unit_buf_128b};
    assign replay_push_wptr_1024_lane2_o = replay_wptr_cnt;

    // --------------------------------------------------------------------------
    // CPIF 2 lane 等宽并行发送（s2p=0，1 个 128B 单元 1 拍内 2 lane 并行发出）
    //   装载拍（unit_load）装载本模块自嵌后完整 128B 单元；
    //   装载后一拍（unit_buf_pending）→ 2 lane 并行发完整单元（各 lane 低 512）。
    //   3T 时序（MAS §3.1）：
    //     T1 收 front 128B（装载，不发）→ T2 CPIF 2 lane 发 front + 收 back 128B（装载）
    //     T3 CPIF 2 lane 发 back（整 Flit 发完）
    // --------------------------------------------------------------------------
    assign send_active = unit_load | unit_buf_pending;
    // 单元"待发/在途"标志 → 父模块，用于 NULL/重传 front→back 装载节拍保持，避免在途
    //   单元被新装载覆盖（与 512 单元高半段在途标志同语义；BUG_LOG 2026-09-02）
    assign unit_busy_1024_lane2_o = unit_buf_pending;
    assign cpif_tx_lane0_data_1024_lane2_vld_o = unit_buf_pending & send_allowed;
    assign cpif_tx_lane1_data_1024_lane2_vld_o = unit_buf_pending & send_allowed;
    assign cpif_tx_lane0_data_1024_lane2_o = {1024{cpif_tx_lane0_data_1024_lane2_vld_o}}
                                            & {512'd0, unit_buf_128b[511:0]    };
    assign cpif_tx_lane1_data_1024_lane2_o = {1024{cpif_tx_lane1_data_1024_lane2_vld_o}}
                                            & {512'd0, unit_buf_128b[1023:512]};

    // --------------------------------------------------------------------------
    // 钟漂反压（F-05：间隔 DRIFT_PERIOD 反压出口 8 拍，须当前 Flit 发完才能反压）
    // --------------------------------------------------------------------------
    assign drift_trigger = (drift_cnt >= csr_drift_period_i)
                         & ~clock_drift_stall
                         & ~send_active;
    assign backoff_last  = clock_drift_stall & (drift_backoff_cnt == 4'd1);
    assign backoff_next  = {4{backoff_last}} & 4'd0 | {4{~backoff_last}} & (drift_backoff_cnt - 4'd1);

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // PAIF beat 计数（0→1 循环，仅 accept 拍推进）
    // --------------------------------------------------------------------------
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            paif_beat_cnt <= 1'b0;
        else if (paif_rx_accept && (paif_beat_cnt == 1'b1))
            paif_beat_cnt <= 1'b0;
        else if (paif_rx_accept)
            paif_beat_cnt <= ~paif_beat_cnt;
    end

    // --------------------------------------------------------------------------
    // PAIF 数据缓冲(1T)：装载拍采样本模块自嵌后完整单元（正常/NULL/重传统一）
    // --------------------------------------------------------------------------
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            unit_buf_128b <= {1024{1'b0}};
        else if (unit_load)
            unit_buf_128b <= unit_assembled;
    end

    // --------------------------------------------------------------------------
    // PAIF 数据缓冲(1T)"待发"标志：装载拍置位，2 lane 并行真实发出后清位
    //   连续流下每拍装载 + 下一拍发出，标志保持 1，不阻塞入口。
    // --------------------------------------------------------------------------
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            unit_buf_pending <= 1'b0;
        else if (unit_load)
            unit_buf_pending <= 1'b1;
        else if (unit_buf_pending & send_allowed)
            unit_buf_pending <= 1'b0;
    end

    // --------------------------------------------------------------------------
    // replay 写指针：每个数据 Flit push（flit_complete）递增 1，与 seq 逐 Flit 递增对齐
    // --------------------------------------------------------------------------
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            replay_wptr_cnt <= {`REPLAY_BUF_PTR{1'b0}};
        else if (flit_complete)
            replay_wptr_cnt <= replay_wptr_cnt + 1'b1;
    end

    // --------------------------------------------------------------------------
    // drift 周期计数器（自由运行，达到周期清零；8 拍反压计入下一周期，不停机）
    // --------------------------------------------------------------------------
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            drift_cnt <= 16'd0;
        else if (drift_cnt >= csr_drift_period_i)
            drift_cnt <= 16'd0;
        else
            drift_cnt <= drift_cnt + 16'd1;
    end

    // --------------------------------------------------------------------------
    // 钟漂反压状态（触发进 8 拍 backoff，反压期间 vld=0、入口反压）
    // --------------------------------------------------------------------------
    always @(posedge mb_clk) begin
        if (!mb_rst_n) begin
            clock_drift_stall   <= 1'b0;
            drift_backoff_cnt   <= 4'd0;
        end
        else if (drift_trigger) begin
            clock_drift_stall   <= 1'b1;
            drift_backoff_cnt   <= DRIFT_BACKOFF_CYCLES;
        end
        else if (clock_drift_stall) begin
            clock_drift_stall   <= ~backoff_last;
            drift_backoff_cnt   <= backoff_next;
        end
    end

    // ==========================================================================
    // 断言与检查（可选）
    // ==========================================================================

endmodule
