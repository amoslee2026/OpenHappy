// =============================================================================
// module name：link_tx_1024_lane1
// author: OpenHappy Team
// MAS/spec：spec/link_layer/submods/link_tx/mas/link_tx_1024_lane1_mas.md
//           上游 spec：spec/link_layer/link_layer_spec.md §4.2.3.2 / §4.2.3.3 / §4.2.5
// 功能概述: link_tx 内部 TX 位宽转换形态②（PAIF 1024 bit × CPIF 1 lane）
//           每拍 128 B 收齐 1 个 128 B 单元；CPIF 1 lane 按 csr_s2p_mode_i
//           输出——s2p_mode=1（1024 全有效，128B/拍）等宽直通单 Flit 3T；
//           s2p_mode=0（低 512 有效，64B/拍）128 B 单元拆 2 拍，单 Flit 5T。
//           1 个 PAIF 数据缓冲（1T，当拍 PAIF 数据 = 1 个完整 128B 单元），经 2 槽
//           send staging 承载 s2p 拆拍分发；无出口 FIFO；送 CRC=单 1024bit 口
//           （无 valid，CRC 纯组合按拍采样）；NULL/重传复用 CRC 数据通道做位宽转换
//           不 replay push；入口反压 + 钟漂反压（8 拍）。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   2026-08-19  OpenHappy Team  初始版本（对应 link_tx_1024_lane1_mas.md v1.0）
//   2026-08-20  OpenHappy Team  修复 send_slot 调度 bug（原 buf_occupy[0]==0 时恒 0 切不到
//                          slot1，back 永不发）；改为每槽独立段位跟踪(seg_remain/seg_second)
//                          + 组合仲裁 front→back 推进，正确实现 3T/5T。
//                          修复装载拍即发陈旧数据：send_pending 不再计入 front/back/nv，
//                          装载拍只装载、下一拍起发（MAS §3.1）。清理死信号。
//   2026-09-03  OpenHappy Team  设计修正：入口反压改为链式语义——cpif_tx_ready 不再作为直连
//                         入口反压条件，入口反压仅由 link_tx_active=0 / 钟漂反压 /
//                         PAIF 数据缓冲(1T)顶满 驱动（存 1T PAIF 数据，出口不可发 →
//                         缓冲顶满 → 反压）。
//   2026-09-04  OpenHappy Team  设计对齐 golden（spec §4.2.3.3）：出口反压仅由 link_tx_active=0 /
//                         钟漂反压触发，移除 send_allowed 中的 cpif_tx_ready_i 项（其不构成
//                         出口/入口反压，仅作状态机跳转/判断 + UDI 透传）。本形态端口保留
//                         不再被使用，仅标注说明。
//   2026-09-03  OpenHappy Team  ARCH：字段自嵌重构（取代父回喂 unit_ckd 注入，对齐 512 形态）——
//                         header / DLP / CRC 校验码 / NULL 数据 / 重传数据 改为本模块
//                         输入，由本模块在 PAIF 数据管线按拍位自嵌：3 路数据源
//                         （PAIF / NULL / 重传，优先级 重传>NULL>paif）做三路 CRC mux，
//                         经 header/dlpmux 后输出 crc_data_1024_lane1_o 送外部 crc16，
//                         CRC 校验码经 crc_chk_1024_lane1_i 同拍回接本模块，嵌入对应
//                         CRC 槽位（front→CRC0 / back→CRC1）。移除父回喂
//                         unit_ckd_1024_lane1_i 与合并 nv_flit_valid 注入路径；NULL/重传
//                         改传数据+front 标记；PAIF 接收改以父权威 paif_rx_accept_i 推进
//                         （避免 NULL/重传注入期父停收而本模块仍独立收）。
//                         front 槽(槽0)/back 槽(槽1)各自装载对应源自嵌单元。
//   2026-09-03  OpenHappy Team  新增 unit_busy_1024_lane1_o（任一槽单元待发/在途标志）输出，
//                         供父模块 link_tx 门控 NULL/重传 front→back 装载节拍，避免在途
//                         单元被覆盖丢失（与 512 单元高半段在途标志同语义，BUG LOG 2026-09-02）。
// =============================================================================

`timescale 1ns/1ps

`include "link_layer_defines.vh"

module link_tx_1024_lane1 (
    // 时钟与复位（统一同步复位，mb_rst_n 低有效）
    input  wire                        mb_clk,
    input  wire                        mb_rst_n,

    // 配置接口（apb_csr 解码直送）
    input  wire                        csr_perf_mode_i,      // 1=性能模式（本模块仅此被父例化）
    input  wire                        csr_protocol_mode_i,  // 1=PAIF 模式；0=UDI 本模块不送数
    input  wire                        csr_s2p_mode_i,       // 1=CPIF 1024 全有效(3T)；0=低512 有效拆2拍(5T)
    input  wire [1:0]                  csr_bw_mode_i,        // 本形态不使用（接口保留，恒全带宽）
    input  wire [15:0]                 csr_drift_period_i,   // 钟漂反压周期（cycle）

    // PAIF TX 入口（性能模式 1024bit/拍 = 1 个 128B 单元）
    input  wire [1023:0]               paif_tx_data_i,
    input  wire                        paif_tx_valid_i,
    output wire                        paif_tx_ready_o,      // 入口反压（1=可收 / 0=反压）
    input  wire                        paif_rx_accept_i,     // 父权威 PAIF 接收（含控制停收门）；
                                                              // 本模块的 beat/装载/front/back 均据此推进，
                                                              // 避免 NULL/重传注入期父停收而本模块仍独立收

    // 字段自嵌输入（父模块组合直送，本模块在 PAIF 数据管线按拍位嵌入）
    input  wire [15:0]                 header_1024_lane1_i,   // 前 128B Header 字段（front 拍嵌入 [15:0]；父按源选 exit_hdr）
    input  wire [15:0]                 dlp_1024_lane1_i,      // 后 128B DLP 字段（back 拍嵌入 [15:0]；父 dlp_field）
    input  wire [15:0]                 crc_chk_1024_lane1_i,  // CRC 校验码回接（外部 crc16 同拍组合结果；front→CRC0 / back→CRC1）
    input  wire [11:0]                 ecc_chk_1024_lane1_i,  // ECC 校验码回接（外部 ecc_generate 同拍组合结果；front→ECC0 / back→ECC1）；csr_ecc_en_i=1 时嵌 chk 槽 [1019:1008]（高 4 tie0）
    // 注错 / CRC / ECC 使能（本模块在组装级执行；与父 CRC 语义一致）
    // NULL / 重传 直接数据输入（三路 CRC mux 源；重传 > NULL > paif）
    input  wire [1023:0]               null_data_1024_lane1_i, // NULL 半单元原始数据（payload，header/DLP 由本模块嵌）
    input  wire                        null_valid_1024_lane1_i,// NULL 半单元有效（front/back 事件；不 replay push，仅装载/CE）
    input  wire                        null_front_1024_lane1_i,// 1=front(CRC0) / 0=back(CRC1)（寄存半态 ~null_half，作 CRC 槽位/分槽；装载火线仍为 valid 事件）
    input  wire                        null_active_1024_lane1_i,// NULL 源选择（寄存 null_active：作 CRC 源/槽位 select，非装载事件）
    input  wire [1023:0]               retry_data_1024_lane1_i,// 重传半单元原始数据（replay pop，header/DLP 由本模块嵌）
    input  wire                        retry_valid_1024_lane1_i,// 重传半单元有效（front/back 事件；不 replay push，仅装载/CE）
    input  wire                        retry_front_1024_lane1_i,// 1=front(CRC0) / 0=back(CRC1)（寄存半态 ~retry_half，作 CRC 槽位/分槽；装载火线仍为 valid 事件）
    input  wire                        retry_active_1024_lane1_i,// 重传源选择（寄存 retry_active：作 CRC 源/槽位 select，非装载事件）
    // 注错 / CRC 使能（本模块在组装级执行；与父 CRC 语义一致）
    input  wire                        csr_crc_en_i,         // CRC 总使能（保留：TX 出口 CRC 槽一律嵌入校验码，不影响槽位）
    input  wire                        csr_ecc_en_i,         // ECC 总使能（RETRY_CFG_REG.ECC_EN，bit21）：chk 槽组装 mux 二选一——=1 嵌 ecc_chk 低 12 / =0 嵌 crc_chk 16bit；与 CRC_EN 互选（同时 ECC 优先）
    input  wire                        csr_inj_en_i,         // INJECT_ERR_LINKn_REG.INJECT_EN：注错使能（电平门控 EN=1 即注错；payload bit0，仅数据/重传不注 NULL）
    input  wire                        csr_inj_fb_i,         // INJECT_ERR_LINKn_REG.FRONT_BACK：0=front(CRC0)/1=back(CRC1)（选命中半单元，单元级 front/back 判定）

    // CRC 组装交接（→ 父模块 crc16_generate；三路 mux 选出的当前源 + header/DLP 嵌入、CRC 槽清 0）
    //   front/back 标记由本模块内部 crc_front_sel 自持（父仅需送 CRC、回嵌校验码，不需 front 标记）
    output wire [1023:0]               crc_data_1024_lane1_o, // 当前要算 CRC 的数据（无 valid，组合按拍采样）

    // replay push 上送（→ 父模块 replay_buffer）
    output wire                        replay_push_wen_1024_lane1_o,     // Flit 收满拍拉高 1 cycle
    output wire [`REPLAY_BUF_PTR-1:0]  replay_push_wptr_1024_lane1_o,    // 写指针（数据 Flit push 递增）
    output wire [2047:0]               replay_push_flit_1024_lane1_o,    // {后128B原始, 前128B带seq/Header}

    // CPIF TX 1 lane 出口（s2p=1 全1024；s2p=0 低512 有效、高512 恒0）
    output wire [1023:0]               cpif_tx_data_1024_lane1_o,
    output wire                        cpif_tx_data_1024_lane1_vld_o,
    output wire                        unit_busy_1024_lane1_o, // 任一槽单元待发/在途（1=正在发/待发单元未发完）
    input  wire                        cpif_tx_ready_i,      // 状态机跳转/判断条件（PHY ready）+ UDI 透传
                                                              //   （golden spec §4.2.3.3 / §8.5）：不构成出口/入口反压，
                                                              //   本形态（性能模式）不使用，保留端口仅为接口稳定
    input  wire                        link_tx_active_i      // 出口使能（S_NORMAL 时=1；=0 为出口反压条件）
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    // 依赖 link_layer_defines.vh：`PAIF_WIDTH（本形态恒 1024）、`REPLAY_BUF_PTR
    localparam DRIFT_BACKOFF_CYCLES = 4'd8;   // 钟漂反压宽度（8 拍，MAS F-05）

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================

    // --------------------------------------------------------------------------
    // PAIF 入口接收 / 出口门控 / 反压条件（组合）
    // --------------------------------------------------------------------------
    wire        paif_rx_accept;      // PAIF 拍被接收（父权威 paif_rx_accept_i）
    wire        send_allowed;        // 出口可发（link active & ~钟漂；cpif_tx_ready 不参与出口反压）
    wire        front_proc;          // 前 128B 收满拍（PAIF beat0）
    wire        back_proc;           // 后 128B 收满拍（PAIF beat1）
    wire        flit_complete;       // 整 Flit 收满（=back_proc，NULL/重传已过滤）
    wire        buf_full_block;      // PAIF 数据缓冲(1T)顶满 → 入口反压
    wire        drift_trigger;       // 钟漂反压触发（周期到且 Flit 已发完）
    wire        backoff_last;        // 钟漂反压末拍
    wire [3:0]  backoff_next;        // 钟漂反压计数递减

    // --------------------------------------------------------------------------
    // PAIF 收齐（每拍 128 B = 1 单元；front beat0 / back beat1）
    // --------------------------------------------------------------------------
    reg  [1:0]  paif_beat_cnt;       // 0=front / 1=back（DFX 观测点）
    wire [1023:0] unit_128b;         // 本拍 PAIF 数据 = 当前 128B 单元

    // --------------------------------------------------------------------------
    // PAIF 数据缓冲(1T)（当拍 PAIF 数据 = 1 个完整 128B 单元），经 2 槽 send staging
    //   承载 s2p 拆拍展开；槽0=前 128B（待发），槽1=后 128B（待发）
    // --------------------------------------------------------------------------
    reg  [1023:0] unit_buf_128b [0:1];   // 2 槽单元缓冲（本模块自嵌后完整单元）
    // 每槽独立段位跟踪：seg_remain=当前单元剩余待发段数（s2p=1→1段；s2p=0→2段）
    //                   seg_second =当前单元下一发段是否第 2 段(高512)（s2p=0 用）
    reg  [1:0]   seg_remain [0:1];       // 各槽剩余待发段数（减到 0 即该槽发完）
    reg          seg_second [0:1];       // 各槽下一发段=第2段(高512)？  s2p=0 用
    // 槽占用（由 seg_remain 组合导出）
    wire         buf_occupy [0:1];       // 各槽占用（有待发单元未发完）
    wire         send_slot;              // 当前发送槽（组合仲裁：0=front 优先 / 1=back）
    wire         front_load;             // front（槽0）装载事件：PAIF front / NULL front / 重传 front
    wire         back_load;              // back（槽1）装载事件：PAIF back / NULL back / 重传 back

    // --------------------------------------------------------------------------
    // 三路 CRC mux + 字段自嵌（取代父回喂 unit_ckd 注入）
    // --------------------------------------------------------------------------
    wire        crc_front_sel;       // 当前拍 front/back：1=front(嵌 header/CRC0) / 0=back(嵌 DLP/CRC1)
    wire        paif_is_front;       // PAIF 源 front 指示（寄存 beat 判位，无 accept/反压；仅装载拍取用）
    wire [1023:0] crc_src_raw;       // 三路源原始数据（重传 > NULL > paif）
    wire [1023:0] crc_src_inj;       // 注错后原始数据（payload 逐位翻，仅数据/重传不注 NULL）
    wire        inj_beat;            // 注错事件拍（数据/重传，非 NULL）
    wire [1023:0] inj_mask;          // 注错掩码：payload bit0 = 单元 bit16（=1024'b1<<16；Header/DLP 占 [15:0]，与 512 同构）
    wire [15:0] field_embed;         // 按 front/back 选择的嵌入字段（front=header_i / back=dlp_i）
    wire [15:0] chk_slot_1024_lane1;       // 出口 chk 槽[1023:1008]（校验码 16bit；ECC=1 嵌 {4'h0,ecc_chk 低 12} / =0 嵌 crc_chk 16bit）
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
    // PAIF 收齐级：每拍 128 B 收齐 1 个单元（front beat0 / back beat1）
    //   PAIF 接收以父权威 paif_rx_accept_i 为准（其已含 paif_ready_ctl 的
    //   ~null/~retry/~arq 停收门）。原先本模块用自身 paif_tx_ready_o 独立推导，
    //   NULL/重传注入期父停收而本模块仍独立收 → spur front_proc/back_proc → 与注入
    //   装载冲突丢单元（BUG LOG 2026-09-02，与 512 同批修复）。
    // --------------------------------------------------------------------------
    assign paif_rx_accept = paif_rx_accept_i;
    assign unit_128b      = paif_tx_data_i[1023:0];

    assign front_proc = csr_perf_mode_i & paif_rx_accept & (paif_beat_cnt == 2'd0);
    assign back_proc  = csr_perf_mode_i & paif_rx_accept & (paif_beat_cnt == 2'd1);
    // PAIF 源 front 指示：仅由寄存 beat 判位（无 accept/反压/控制），在 PAIF front 装载拍与
    //   front_proc 等值（front_proc=beat==0），back 拍（beat==1）取 0 —— 作 CRC front/back 槽位选择。
    assign paif_is_front = (paif_beat_cnt == 2'd0);
    assign flit_complete = back_proc;   // 后 128B 收满 = 整个 256B Flit 收齐

    // --------------------------------------------------------------------------
    // PAIF 数据缓冲(1T)顶满判定：当拍 PAIF 数据 = 1 个完整 128B 单元，经 2 槽 send
    //   staging 承载 s2p 拆拍展开；任一槽占用即视为缓冲忙。
    //   s2p=1（等宽直通）：front/back 各 1 拍发，槽仅在发送拍短暂占用后即释放，
    //     不阻塞连续 PAIF（连续背靠背是 1 发 1 收重叠，见下）。
    //   s2p=0（拆 2 拍）：front 拆 2 拍发、back 拆 2 拍发，且 front 第 1 拍与 back
    //     收齐重叠——front/back 均待发即顶满，故"满"= 两槽都被占用。
    // --------------------------------------------------------------------------
    assign buf_full_block = buf_occupy[0] & buf_occupy[1];

    // 出口反压（golden spec §4.2.3.3）：仅由 link_tx_active=0 或 钟漂反压 触发；
    //   cpif_tx_ready_i（PHY ready 跳转/判断 + UDI 透传）不构成出口反压，不进入发送使能。
    assign send_allowed = link_tx_active_i & ~clock_drift_stall;
    // 入口反压（MAS §3.2 链式语义）：两个出口反压条件 link_tx_active=0 / 钟漂反压，
    //   或内部 PAIF 数据缓冲(1T)顶满。cpif_tx_ready 不再直连入口反压——PAIF 数据缓冲
    //   存 1T PAIF 数据（当拍 = 1 个完整 128B 单元），出口不可发 → 缓冲顶满 → 反压
    //   PAIF 入口（无数据丢失）。
    assign paif_tx_ready_o = ~( ~link_tx_active_i
                              | clock_drift_stall
                              | buf_full_block );

    // --------------------------------------------------------------------------
    // 三路 CRC mux + 字段自嵌（取代父回喂 unit_ckd 注入）
    //   源选择：重传 > NULL > paif。NULL/重传一旦触发必须发完（父 paif_rx_accept_i
    //   含 ~null/~retry/~arq 停收门，且父以 ~unit_busy 门控装载节拍），其进行期 PAIF
    //   被停收（front/back 不动作），中途 PAIF 数据不插入拆断——故单拍仅一个源有效，
    //   优先级仲裁为语义保证。
    //   按拍位自嵌：front 拍嵌 header_i([15:0])、back 拍嵌 dlp_i，CRC 槽[1023:1008]清 0，
    //   输出 crc_data_1024_lane1_o 送外部 crc16；校验码经 crc_chk_1024_lane1_i 同拍回接，
    //   由 unit_assembled 嵌回 CRC 槽（front→CRC0 / back→CRC1）。
    //   注错（payload 逐位，仅数据/重传不注 NULL）：与父 crc 语义一致，翻转 crc_src_raw
    //   bit0/bit1，payload 取注入后、header/DLP 取字段嵌入——CRC 算在注入后 payload 上。
    // --------------------------------------------------------------------------
    // CRC 数据源/槽位选择仅用已寄存源状态（retry/null_active）与半态（retry/null_front 由父
    //   直连 ~retry_half/~null_half）、PAIF 寄存 beat 判位——不含 conv_accept/link_active，
    //   切断 link_active→CRC select→CRC16 树 的跨模块 28 级关键路径；装载/valid 仍由
    //   retry_valid/null_valid 事件驱动（reconv_en/CE、inj_beat 节拍、front_load/back_load 不变）。
    wire crc_sel_retry_1024l1 = retry_active_1024_lane1_i;
    wire crc_sel_null_1024l1  = ~retry_active_1024_lane1_i & null_active_1024_lane1_i;
    wire crc_sel_data_1024l1  = ~retry_active_1024_lane1_i & ~null_active_1024_lane1_i;
    assign crc_front_sel = crc_sel_retry_1024l1 & retry_front_1024_lane1_i
                          | crc_sel_null_1024l1  & null_front_1024_lane1_i
                          | crc_sel_data_1024l1  & paif_is_front;
    assign crc_src_raw  = {1024{crc_sel_retry_1024l1}} & retry_data_1024_lane1_i
                         | {1024{crc_sel_null_1024l1}}  & null_data_1024_lane1_i
                         | {1024{crc_sel_data_1024l1}}  & unit_128b;
    // 注错（v1.6 per-TX 独立控制，gdx_review Item 1）：每 TX INJECT_EN 电平门控（EN=1 即
    //   注错，无 trigger/臂化）+ FRONT_BACK 选半单元（0=front(CRC0)/1=back(CRC1)）——
    //   对注入前原始数据 crc_src_raw 施加 payload bit0 XOR（inj_mask=1<<16=单元 bit16，与
    //   512 同构；勿改字面 bit0 会破坏 Header/DLP[15:0]），仅数据/重传不注 NULL。装载火线
    //   （load 事件）之外的拍 unit_assembled 不被捕获/槽不装载，select/注入值无关紧要。
    //   FRONT_BACK 选半单元（crc_front_sel: 1=front/0=back），EN=1 电平门控；仅数据/重传不注 NULL。
    // 注错（v1.6 per-TX 独立控制，gdx_review Item 1）：每 TX INJECT_EN 电平门控（EN=1 即
    //   注错，无 trigger/臂化）+ FRONT_BACK 选半单元（0=front(CRC0)/1=back(CRC1)）——
    //   对注入前原始数据 crc_src_raw 施加 payload bit0 XOR（inj_mask=1<<16=单元 bit16，与
    //   512 同构；勿改字面 bit0 会破坏 Header/DLP[15:0]），仅数据/重传不注 NULL。装载火线
    //   （load 事件）之外的拍 unit_assembled 不被捕获/槽不装载，select/注入值无关紧要。
    //   FRONT_BACK 选半单元（crc_front_sel: 1=front/0=back），EN=1 电平门控；仅数据/重传不注 NULL。
    //   掩码收敛同 512：payload bit0 = 单元 bit16（Header/DLP 占 [15:0]），勿改字面 bit0（会破坏 Header/DLP）。
    assign inj_mask     = (1024'b1 << 16);
    assign inj_beat     = csr_inj_en_i & (csr_inj_fb_i ^ crc_front_sel) & ~null_active_1024_lane1_i;
    assign crc_src_inj  = {1024{inj_beat}} & (crc_src_raw ^ inj_mask) | {1024{~inj_beat}} & crc_src_raw;
    assign field_embed  = {16{ crc_front_sel}} & header_1024_lane1_i | {16{~crc_front_sel}} & dlp_1024_lane1_i;
    assign crc_data_1024_lane1_o  = {16'd0, crc_src_raw[1007:16], field_embed};
    // 自嵌后完整单元（组合直出，crc_chk / ecc_chk 同拍有效——crc16 / ecc 均纯组合 0 cycle）。
    //   TX 出口 chk 槽[1023:1008]按 csr_ecc_en_i 二选一嵌校验码（ECC_EN=1 嵌 ecc_chk 低 12、
    //   高 4 tie0；=0 嵌 crc16 校验码不依赖 csr_crc_en_i——CRC_EN=0 由收端不查兜底）。
    assign chk_slot_1024_lane1 = {16{csr_ecc_en_i}} & {4'h0, ecc_chk_1024_lane1_i}
                                | {16{~csr_ecc_en_i}} & crc_chk_1024_lane1_i;
    assign unit_assembled = {chk_slot_1024_lane1, crc_src_inj[1007:16], field_embed};

    // --------------------------------------------------------------------------
    // 槽装载事件：front（槽0）<== PAIF front / NULL front / 重传 front；
    //             back（槽1）<== PAIF back / NULL back / 重传 back。
    //   每槽装载对应源自嵌单元（unit_assembled，组合直出——本形态当拍 PAIF 数据即
    //   完整 128B 单元，header/CRC 可同拍自嵌）。
    // --------------------------------------------------------------------------
    assign front_load = front_proc
                      | (null_valid_1024_lane1_i & null_front_1024_lane1_i)
                      | (retry_valid_1024_lane1_i & retry_front_1024_lane1_i);
    assign back_load  = back_proc
                      | (null_valid_1024_lane1_i & ~null_front_1024_lane1_i)
                      | (retry_valid_1024_lane1_i & ~retry_front_1024_lane1_i);

    // --------------------------------------------------------------------------
    // replay push 上送（仅正常数据 Flit 收满产生，NULL/重传不 push）
    //   写数据 {back 原始(本拍 unit_128b), front 带 seq/Header(槽0已装)}
    //   写使能 = flit_complete（back 收满拍）
    // --------------------------------------------------------------------------
    assign replay_push_wen_1024_lane1_o  = flit_complete;
    assign replay_push_flit_1024_lane1_o = {unit_128b, unit_buf_128b[0]};
    assign replay_push_wptr_1024_lane1_o = replay_wptr_cnt;

    // --------------------------------------------------------------------------
    // CPIF 流式发送（s2p=1 等宽直通 1T/单元；s2p=0 拆 2 拍 64B/拍）
    //   3T（s2p=1）：T1 收front→T2 发front+收back→T3 发back
    //   5T（s2p=0）：T1 收front→T2 发front前段+收back→T3 发front后段→T4 发back前段→T5 发back后段
    // ==========================================================================
    wire send_active;
    wire send_issue;   // 本拍真实发出一个数据片段（send_allowed 且当前有单元待发）
    wire send_pending; // 当前有待发片段（= 任一台占用，装载拍不算待发）

    // 待发片段：任一槽占用（其单元未发完）即有待发；s2p=0 每槽跨 2 拍，s2p=1 每槽 1 拍
    // 槽占用由 seg_remain 组合导出（装载拍 seg_remain 尚未置位→不占用→不发，满足 MAS §3.1
    // “装载拍只装载、下一拍起发”，杜绝装载拍读取陈旧缓冲数据）
    assign buf_occupy[0] = (seg_remain[0] != 2'd0);
    assign buf_occupy[1] = (seg_remain[1] != 2'd0);

    // 单元"待发/在途"标志 → 父模块，用于 NULL/重传 front→back 装载节拍保持，避免在途
    //   单元被新装载覆盖（与 512 单元高半段在途标志同语义；BUG_LOG 2026-09-02）
    assign unit_busy_1024_lane1_o = buf_occupy[0] | buf_occupy[1];

    // 发送槽组合仲裁：front(槽0)未发完优先，否则 back(槽1)；两者皆空则槽0（无发送）
    assign send_slot = ~buf_occupy[0] & buf_occupy[1];

    // 待发片段=仍有占用槽（不含 front/back/nv 装载本身：装载拍不算待发）
    assign send_pending = buf_occupy[0] | buf_occupy[1];
    assign send_active  = send_pending;
    assign send_issue   = send_active & send_allowed;

    // CPIF 数据：按当前发送槽 + 段选（s2p=1 全量；s2p=0 按该槽段位 seg_second 前/后 512）
    //   s2p=0 高段须驱动到 [1023:512]（低 512 置 0），与 link_rx s2p=0 lane-split
    //   拼装（奇数拍 [1023:512]）对齐，否则 CRC[1023:1008] 在 RX 读到 0（BUG_LOG 2026-09-02）
    wire [1023:0] cpif_unit_sel = {1024{send_slot}} & unit_buf_128b[1]
                                 | {1024{~send_slot}} & unit_buf_128b[0];
    wire          cpif_seg_sel  = send_slot & seg_second[1]
                                | ~send_slot & seg_second[0];
    wire [1023:0] cpif_data_seg = {1024{csr_s2p_mode_i            }} & cpif_unit_sel
                                | {1024{~csr_s2p_mode_i & cpif_seg_sel  }} & {cpif_unit_sel[1023:512], 512'd0}
                                | {1024{~csr_s2p_mode_i & ~cpif_seg_sel }} & {512'd0, cpif_unit_sel[511:0]};
    assign cpif_tx_data_1024_lane1_o = {1024{send_issue}} & cpif_data_seg;
    assign cpif_tx_data_1024_lane1_vld_o = send_issue;

    // --------------------------------------------------------------------------
    // 钟漂反压（F-05：间隔 DRIFT_PERIOD 反压出口 8 拍，须当前 Flit 发完才能反压）
    // --------------------------------------------------------------------------
    assign drift_trigger = (drift_cnt >= csr_drift_period_i)
                         & ~clock_drift_stall
                         & ~send_pending;
    assign backoff_last  = clock_drift_stall & (drift_backoff_cnt == 4'd1);
    assign backoff_next  = {4{backoff_last}} & 4'd0 | {4{~backoff_last}} & (drift_backoff_cnt - 4'd1);

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // PAIF beat 计数（0=front / 1=back 循环，仅 accept 拍推进）
    // --------------------------------------------------------------------------
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            paif_beat_cnt <= 2'd0;
        else if (paif_rx_accept && (paif_beat_cnt == 2'd1))
            paif_beat_cnt <= 2'd0;
        else if (paif_rx_accept)
            paif_beat_cnt <= paif_beat_cnt + 2'd1;
    end

    // --------------------------------------------------------------------------
    // 2 槽单元缓冲装载（front→槽0，back→槽1；NULL/重传 按 front/back 分槽）
    //   装载本模块自嵌后完整单元（正常/NULL/重传统一，unit_assembled）
    // --------------------------------------------------------------------------
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            unit_buf_128b[0] <= {1024{1'b0}};
        else if (front_load)
            unit_buf_128b[0] <= unit_assembled;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            unit_buf_128b[1] <= {1024{1'b0}};
        else if (back_load)
            unit_buf_128b[1] <= unit_assembled;
    end

    // --------------------------------------------------------------------------
    // 各槽剩余段数跟踪：装载拍载入段数（s2p=1→1 段；s2p=0→2 段）；发出 1 段减 1；
    //   减到 0 即该槽发完 → buf_occupy 组合归 0（槽释放、供仲裁切走）
    // --------------------------------------------------------------------------
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            seg_remain[0] <= 2'd0;
        else if (front_load)
            seg_remain[0] <= {2{csr_s2p_mode_i}} & 2'd1 | {2{~csr_s2p_mode_i}} & 2'd2;
        else if (send_issue && (send_slot == 1'b0))
            seg_remain[0] <= seg_remain[0] - 2'd1;
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            seg_remain[1] <= 2'd0;
        else if (back_load)
            seg_remain[1] <= {2{csr_s2p_mode_i}} & 2'd1 | {2{~csr_s2p_mode_i}} & 2'd2;
        else if (send_issue && (send_slot == 1'b1))
            seg_remain[1] <= seg_remain[1] - 2'd1;
    end

    // --------------------------------------------------------------------------
    // 各槽段位跟踪（s2p=0 每单元 2 段：0=低512，1=高512；每发 1 段翻转）
    //   装载拍复位为第 1 段（低 512）
    // --------------------------------------------------------------------------
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            seg_second[0] <= 1'b0;
        else if (front_load)
            seg_second[0] <= 1'b0;
        else if (send_issue && (send_slot == 1'b0) && ~csr_s2p_mode_i)
            seg_second[0] <= ~seg_second[0];
    end
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            seg_second[1] <= 1'b0;
        else if (back_load)
            seg_second[1] <= 1'b0;
        else if (send_issue && (send_slot == 1'b1) && ~csr_s2p_mode_i)
            seg_second[1] <= ~seg_second[1];
    end

    // --------------------------------------------------------------------------
    // replay 写指针：每个数据 Flit push（flit_complete）递增 1
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
