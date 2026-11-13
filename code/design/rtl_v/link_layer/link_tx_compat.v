// =============================================================================
// module name：link_tx_compat
// author: OpenHappy Team
// MAS/spec：spec/link_layer/submods/link_tx/mas/link_tx_compat_mas.md
//           上游 spec：spec/link_layer/link_layer_spec.md §4.2.3.2 / §4.2.3.3
// 功能概述: link_tx 内部 TX 位宽转换形态④（兼容模式 32 B Flit / CRC9 内嵌），
//           兼容模式（csr_perf_mode_i=0&csr_protocol_mode_i=1）例化。
//           流水线 1T 延迟（L1 §4.2.5）：PAIF T1 收齐 1 个完整 272 bit 兼容 Flit
//           （接收父组装 Header+Payload 261 bit → 内嵌 crc9_generate 算 CHK →
//           拼 11 bit Tail）→ 采样进 1T 槽缓冲 compat_flit_272_r → T2 从 CPIF 发出
//           （低 272 bit，高位恒 0，s2p=0/1 通用）→ 正常数据 Flit 同步 replay push；
//           无出口 FIFO；入口反压（1T 槽满且下游反压 的链式条件 / link 非 active /
//           钟漂反压 8 拍）。
//           gdx_review Item 8：兼容模式 ARQ / NULL 控制 Flit 生成由父 link_tx 迁入本
//           模块（ctrl_flit_gen 兼容部分 + 兼容 NULL），父 link_tx 仅例化透传（数据 /
//           重传 / ARQ 请求 → 本模块；本模块回报控制 Flit 在发/装载事件）。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   <日期>  <修订人>  <变更概述>
//  2026-11-08  OpenHappy Team  初始版本（对应 link_tx_compat_mas.md v1.0）
//  2026-09-03  OpenHappy Team  设计修正：入口反压改为链式语义——cpif_tx_ready 不再作为直连
//                           入口反压条件，改为"1T 槽满 且 下游反压"才反压入口；并新增
//                           1 槽缓冲（compat_flit_272_r / compat_flit_valid_r）实现
//                           流水线 1T 延迟（T1 收齐 → T2 从 CPIF 发出），取代原单拍
//                           组合直出（原实现同拍 T1 即送出，违反 L1 §4.2.5 1T 延迟）。
//  2026-09-04  OpenHappy Team  设计对齐 golden（spec §4.2.3.3）：出口反压仅由 link_tx_active=0 /
//                           钟漂反压触发，can_emit 移除 cpif_tx_ready_i 项；入口反压移除原
//                           "1T 槽满 & 下游反压"中的 cpif_tx_ready_i 项（其不构成入口反压）——
//                           can_emit 不再依赖 cpif_tx_ready 后，1T 槽在出口允许时每拍发出并
//                           腾空，无独立"槽满"累积项，入口反压即 active=0 / 钟漂 直连。
//                           本形态端口保留不再被使用，仅标注。
//  2026-09-14  OpenHappy Team  gdx_review Item 8：兼容模式 ARQ / NULL flit 生成迁入本模块
//                           （控制 Flit 状态机 null_active/arq_active/arq_pending/arq_kind
//                            + ctrl_comp_null / ctrl_arq_flit 生成 + 内部源仲裁），父 link_tx
//                           仅例化透传。入口端口改为区分数据（compat_flit_data/valid）与重传
//                           （retry_flit_data/valid + retry_active）；ARQ 请求输入
//                           （arq_req_flit/valid、arq_req_rcv_ptr/valid、retrans_done、
//                            tx_seq、null_gen_req）内建生成控制 Flit；新增回报
//                           ctrl_busy_compat_o（NULL/ARQ 在发，父反压 PAIF 入口）与
//                           ctrl_event_compat_o（NULL/ARQ 装载事件拍）。replay push 仅
//                           数据 Flit（内部源判定，去 nv_flit_valid_compat_i 外置标记）。
// =============================================================================

`timescale 1ns/1ps

`include "link_layer_defines.vh"

module link_tx_compat (
    // 时钟与复位（统一同步复位，mb_rst_n 低有效）
    input  wire                        mb_clk,
    input  wire                        mb_rst_n,

    // 配置接口（apb_csr 解码直送；仅兼容模式例化，csr_perf_mode_i=0）
    input  wire                        csr_perf_mode_i,      // 0=兼容（本形态被父例化）/ 1=不例化（防御门控）
    input  wire                        csr_protocol_mode_i,  // 1=PAIF 送数 / 0=UDI 父直通，本模块不送数
    input  wire                        csr_s2p_mode_i,       // 兼容 0/1 通用，仅低 272 bit 承载（接口保留）
    input  wire [1:0]                  csr_bw_mode_i,        // 带宽模式（接口保留，本形态不使用）
    input  wire [15:0]                 csr_drift_period_i,   // 钟漂反压触发周期（cycle）

    // PAIF 数据 Flit 入口（父组装 Header+Payload，261 bit，CRC9 未算；仅 DATA 数据 Flit）
    input  wire [260:0]                compat_flit_data_compat_i,  // bit[4:0]=Header / bit[260:5]=Payload
    input  wire                        compat_flit_valid_compat_i, // 1=当前拍数据 Flit 有效
    output wire                        paif_tx_ready_compat_o,     // 入口反压（1=可收 / 0=反压）

    // 兼容重传 Flit 装载（父 replay 读回；Header+Payload 261 bit，CRC9 未算）
    input  wire [271:0]                retry_flit_compat_i,        // 重传 Flit（低 261 bit 有效，Tail 忽略）
    input  wire                        retry_flit_valid_compat_i,  // 1=兼容重传单装载拍（父 retry_compat_event）
    input  wire                        retry_active_compat_i,      // 1=父重传进行中（ARQ 不与重传抢占/源优先级）

    // 兼容 NULL / ARQ 控制 Flit 生成输入（gdx_review Item 8：生成逻辑在本模块内）
    input  wire                        null_gen_req_compat_i,      // 1=本拍请求生成兼容 NULL（父：ACK/NACK 待发等）
    input  wire [255:0]                arq_req_flit_compat_i,      // ARQ(Request) payload（父 link_rx 经同步）
    input  wire                        arq_req_valid_compat_i,     // ARQ(Request) 有效
    input  wire [15:0]                 arq_req_rcv_ptr_compat_i,   // 收到远端 ARQ(Request) Pointer（→ ARQ(ACK)）
    input  wire                        arq_req_rcv_valid_compat_i, // 远端 ARQ(Request) 有效（触发 ARQ(ACK)）
    input  wire                        retrans_done_compat_i,      // 兼容重传序列完成（触发 ARQ(Done)）
    input  wire [7:0]                  tx_seq_compat_i,            // 父 tx_seq（ARQ(Done) Pointer 用）

    // 控制 Flit 在发 / 装载事件回报（→ 父 link_tx）
    output wire                        ctrl_busy_compat_o,         // 1=NULL/ARQ 控制 Flit 在发（父反压 PAIF 入口）
    output wire                        ctrl_event_compat_o,        // 1=NULL/ARQ 控制 Flit 装载事件拍（父 flit_sent/装载）

    // replay push 上送（→ 父模块 replay_buffer，经父转发；仅正常数据 Flit 产生）
    output wire                        replay_push_wen_compat_o,       // 正常数据 Flit 收齐拍拉高 1 cycle
    output wire [`REPLAY_BUF_PTR-1:0]  replay_push_wptr_compat_o,      // 写指针（数据 Flit push 递增）
    output wire [271:0]                replay_push_flit_compat_o,      // 原始 32 B Flit（CHK 未算、Tail.Rsv=0）

    // CPIF TX 1 lane 出口（低 272 bit 承载，高位恒 0）
    output wire [1023:0]               cpif_tx_data_compat_o,
    output wire                        cpif_tx_data_compat_vld_o,
    input  wire                        cpif_tx_ready_i,      // 状态机跳转/判断条件（PHY ready）+ UDI 透传
                                                              //   （golden spec §4.2.3.3 / §8.5）：不构成出口/入口反压，
                                                              //   本形态（兼容模式）不使用，保留端口仅为接口稳定
    input  wire                        link_tx_active_i      // 出口使能（S_NORMAL 时=1；=0 为出口反压条件）
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    // 依赖 link_layer_defines.vh：`REPLAY_BUF_PTR（replay 写指针位宽，含 1 bit flag）
    localparam DRIFT_BACKOFF_CYCLES = 4'd8;   // 钟漂反压宽度（8 拍，MAS F-06）

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================

    // --------------------------------------------------------------------------
    // 入口握手 / 出口门控 / 反压条件（组合）
    // --------------------------------------------------------------------------
    wire        conv_accept;       // 本形态可装载（PAIF 模式 & 出口可发；控制事件门控）
    wire        compat_rx_accept;  // 槽装载（数据/重传/控制事件 & 槽本拍结束为空）
    wire        can_emit;          // 出口本拍可发（link active & ~钟漂）
    wire        emit_held;         // 槽内 Flit 本拍发出（valid & can_emit）＝ T2 出口有效
    wire        slot_free;         // 槽本拍结束时为空（可收新 Flit）
    wire        send_active;       // 正在发送（兼容 Flit 在槽内待发 = 1T 槽非空）
    wire        drift_trigger;     // 钟漂反压触发（周期到且当前 Flit 已发完）
    wire        backoff_last;      // 钟漂反压末拍
    wire [3:0]  backoff_next;      // 钟漂反压计数递减

    // --------------------------------------------------------------------------
    // 1T 槽缓冲（流水线 1T 延迟：T1 收齐 → T2 发出）
    // --------------------------------------------------------------------------
    reg  [271:0] compat_flit_272_r;   // 持有已收完整 272 bit 兼容 Flit（1 槽，非 FIFO）
    reg          compat_flit_valid_r; // 1=槽内有待发 Flit（槽满）

    // --------------------------------------------------------------------------
    // CRC9 内嵌（crc9_generate 纯组合，覆盖 Header+Payload，0 cycle）
    // --------------------------------------------------------------------------
    wire [8:0]  chk;                  // CRC9 计算值（DFX 观测点，注错前原始值）

    // --------------------------------------------------------------------------
    // 完整 272 bit 兼容 Flit 组合拼装（源 261 bit → Header+Payload+CHK+Rsv）
    // --------------------------------------------------------------------------
    wire [260:0] flit_src_261;     // 当前装载源 261 bit（ARQ > 重传 > NULL > 数据）
    wire [271:0] compat_flit_272;  // 当前源完整 272 bit（CHK 内嵌，Tail.Rsv=0）

    // --------------------------------------------------------------------------
    // 兼容 NULL / ARQ 控制 Flit 生成（gdx_review Item 8：由父 link_tx 迁入）
    //   NULL：固定 Pattern Flit（Header={1'b0,2'b01,1'b1,1'b0}，payload 全 0）
    //   ARQ ：父 arq_* 输入生成 Request/ACK/Done 控制 Flit（Header={1'b0,2'b01,1'b0,1'b0}）
    // --------------------------------------------------------------------------
    reg          null_active;       // 兼容 NULL 在发（非抢占）
    reg          arq_active;        // ARQ 控制 Flit 在发（抢占型优先级最高）
    reg          arq_pending;       // 待发 ARQ 请求被阻塞时锁存
    reg  [1:0]   arq_kind;          // 0=RELAY / 1=ACK / 2=DONE
    wire         null_gen_req;      // 兼容 NULL 生成请求（父请求 & 空闲）
    wire         null_compat_event; // 兼容 NULL 单装载事件
    wire         arq_ack_req;       // 收到远端 ARQ(Request) 需应答（触发 ARQ(ACK)）
    wire         arq_req_pend;      // ARQ 请求组合源（Request/ACK/Done），与 arq_pending 供启动判定
    wire         arq_start;         // ARQ 控制 Flit 启动（抢占型）
    wire         arq_compat_event;  // ARQ 控制 Flit 单装载事件
    wire [271:0] ctrl_comp_null;    // 兼容 NULL（272b，Header ACK bit=1）
    wire [271:0] ctrl_arq_flit;     // 兼容 ARQ 控制 Flit（Header+SUB_TYPE+Pointer+Reserved）
    wire [3:0]   arq_subtype;       // ARQ SUB_TYPE：0001=Request / 0010=ACK / 0100=Done
    wire [15:0]  arq_pointer;       // ARQ Pointer：Request=中继 / ACK=rcv_ptr / Done=tx_seq
    wire [255:0] arq_payload;       // ARQ 256-bit Payload（SUB_TYPE[3:0]+Pointer[19:4]+Reserved[255:20]）

    // --------------------------------------------------------------------------
    // 钟漂反压（计数器 + 8 拍 backoff）
    // --------------------------------------------------------------------------
    reg  [15:0] drift_cnt;           // 自由运行周期计数器（不停机，DFX 观测）
    reg  [3:0]  drift_backoff_cnt;   // 反压剩余拍数（0=未反压）
    reg         clock_drift_stall;   // 钟漂反压进行中（=1 且反压出口/入口）

    // --------------------------------------------------------------------------
    // replay push 写指针（内部自管理，正常数据 Flit push 递增）
    // --------------------------------------------------------------------------
    reg  [`REPLAY_BUF_PTR-1:0] replay_wptr_cnt;

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // 本形态可装载（PAIF 模式 & 出口可发；控制事件与入口握手门控，= 父 conv_accept 语义）
    // --------------------------------------------------------------------------
    assign conv_accept = csr_protocol_mode_i & link_tx_active_i & ~clock_drift_stall;

    // --------------------------------------------------------------------------
    // 兼容 NULL 生成（固定 Pattern Flit）
    // --------------------------------------------------------------------------
    assign null_gen_req   = null_gen_req_compat_i & ~null_active & ~csr_perf_mode_i;
    assign null_compat_event = null_active & ~csr_perf_mode_i & conv_accept;
    assign ctrl_comp_null[4:0]    = {1'b0, 2'b01, 1'b1, 1'b0};
    assign ctrl_comp_null[260:5]  = 256'd0;
    assign ctrl_comp_null[271:261]= 11'd0;

    // --------------------------------------------------------------------------
    // ARQ 控制 Flit 生成（抢占型，MAS §6.5 / §6.5.2；与父重传互斥）
    // --------------------------------------------------------------------------
    assign arq_ack_req      = arq_req_rcv_valid_compat_i;
    assign arq_req_pend     = arq_req_valid_compat_i | arq_ack_req | retrans_done_compat_i;
    assign arq_start        = ~arq_active & ~csr_perf_mode_i & ~retry_active_compat_i
                            & (arq_req_pend | arq_pending);
    assign arq_compat_event = arq_active & ~csr_perf_mode_i & conv_accept;
    assign arq_subtype = {4{(arq_kind == 2'b01)}} & 4'b0010
                       | {4{(arq_kind == 2'b10)}} & 4'b0100
                       | {4{~(arq_kind == 2'b01) & ~(arq_kind == 2'b10)}} & 4'b0001;
    assign arq_pointer = {16{(arq_kind == 2'b01)}} & arq_req_rcv_ptr_compat_i[15:0]
                       | {16{(arq_kind == 2'b10)}} & {8'd0, tx_seq_compat_i[7:0]}
                       | {16{~(arq_kind == 2'b01) & ~(arq_kind == 2'b10)}} & arq_req_flit_compat_i[19:4];
    assign arq_payload[3:0]    = arq_subtype[3:0];
    assign arq_payload[19:4]   = arq_pointer[15:0];
    assign arq_payload[255:20] = {236{(arq_kind == 2'b00)}} & arq_req_flit_compat_i[255:20];
    assign ctrl_arq_flit[4:0]    = {1'b0, 2'b01, 1'b0, 1'b0};
    assign ctrl_arq_flit[260:5]  = arq_payload[255:0];
    assign ctrl_arq_flit[271:261]= 11'd0;

    // --------------------------------------------------------------------------
    // 装载源 261 bit 仲裁（优先级 ARQ > 重传 > NULL > 数据；与父 exit_comp_src 同序）
    // --------------------------------------------------------------------------
    wire arq_m_arq   = arq_active;
    wire arq_m_retry = retry_active_compat_i & ~csr_perf_mode_i & ~arq_m_arq;
    wire arq_m_null  = null_active & ~csr_perf_mode_i & ~arq_m_arq & ~arq_m_retry;
    wire arq_m_data  = ~arq_m_arq & ~arq_m_retry & ~arq_m_null;
    assign flit_src_261 = {261{arq_m_arq}}   & ctrl_arq_flit[260:0]
                         | {261{arq_m_retry}} & retry_flit_compat_i[260:0]
                         | {261{arq_m_null}}  & ctrl_comp_null[260:0]
                         | {261{arq_m_data}}  & compat_flit_data_compat_i[260:0];

    // --------------------------------------------------------------------------
    // 内嵌 CRC9 计算（crc9_generate 纯组合，输入 261 bit → 9 bit CHK）
    //   注：CRC9 覆盖当前装载源的 Header+Payload（含控制/重传 Flit，与父原语义一致）
    // --------------------------------------------------------------------------
    // u_crc9.crc9_data_i <= flit_src_261[260:0]
    // u_crc9.crc9_o      => chk[8:0]（组合直出，无寄存器）

    // --------------------------------------------------------------------------
    // 完整 272 bit 兼容 Flit（组合拼装，Tail = {CHK[8:0], Rsv[1:0]=2'b00}）
    // --------------------------------------------------------------------------
    assign compat_flit_272 =
        { flit_src_261[4:0],    // Header 5 bit
          flit_src_261[260:5],  // Payload 256 bit
          chk[8:0],             // Tail CHK（内嵌 crc9 输出）
          2'b00 };              // Tail Rsv

    // --------------------------------------------------------------------------
    // 入口握手：1T 槽缓冲收取（T1 收齐入槽，T2 从 CPIF 发出，满足 L1 §4.2.5 1T 延迟）
    //   严控 4 条件：装载事件（数据/重传/NULL/ARQ）& PAIF 模式 & ~性能(防御) & 槽空
    // --------------------------------------------------------------------------
    assign compat_rx_accept =
        ( compat_flit_valid_compat_i
        | retry_flit_valid_compat_i
        | null_compat_event
        | arq_compat_event )
      & csr_protocol_mode_i
      & ~csr_perf_mode_i
      & slot_free;

    // --------------------------------------------------------------------------
    // 出口可发 / 1T 槽发送（can_emit → emit_held）与 槽空判定
    // --------------------------------------------------------------------------
    assign can_emit  = link_tx_active_i & ~clock_drift_stall;
    assign emit_held = compat_flit_valid_r & can_emit;
    assign slot_free = ~compat_flit_valid_r | emit_held;

    // --------------------------------------------------------------------------
    // 入口反压（L1 §4.2.3.3）：出口反压仅由 link_tx_active=0 / 钟漂反压 触发，二者经
    //   内部 1T 槽（compat_rx_accept 已按 slot_free 门控收取）直接反压 PAIF 入口。
    //   控制 Flit（NULL/ARQ）在发通过 ctrl_busy_compat_o 回报父侧并入 paif_ready_ctl。
    //   PAIF 协议模式为 0 时父 udi_bypass_mux 直通，本模块强制不送数。
    // --------------------------------------------------------------------------
    assign paif_tx_ready_compat_o =
        csr_protocol_mode_i
      & ~( ~link_tx_active_i
         | clock_drift_stall );

    // --------------------------------------------------------------------------
    // CPIF 单 lane 流出（低 272 bit 承载，高位恒 0；vld=0 时 data 全 0）
    // --------------------------------------------------------------------------
    assign cpif_tx_data_compat_vld_o = emit_held;
    assign cpif_tx_data_compat_o = {1024{emit_held}} & {752'd0, compat_flit_272_r};

    // --------------------------------------------------------------------------
    // replay push（仅正常数据 Flit 收齐产生；NULL/ARQ/重传不 push）
    //   push 数据 = 原始 32 B Flit（Header + Payload + Tail，CHK 未算、Rsv=0）；
    //               重传时父模块重新送入本模块内嵌 crc9 重新计算（replay_buffer 不存 CRC）
    // --------------------------------------------------------------------------
    assign replay_push_wen_compat_o  = compat_rx_accept
                                     & ~retry_active_compat_i
                                     & ~null_active
                                     & ~arq_active;
    assign replay_push_flit_compat_o =
        { flit_src_261[4:0],
          flit_src_261[260:5],
          11'd0 };
    assign replay_push_wptr_compat_o = replay_wptr_cnt;

    // --------------------------------------------------------------------------
    // 控制 Flit 回报（→ 父）：在发（反压 PAIF 入口）/ 装载事件拍（flit_sent/装载）
    // --------------------------------------------------------------------------
    assign ctrl_busy_compat_o  = null_active | arq_active;
    assign ctrl_event_compat_o = null_compat_event | arq_compat_event;

    // --------------------------------------------------------------------------
    // 发送使能 / 钟漂反压触发（MAS §5.3：周期到且当前 Flit 已发完才能反压）
    // --------------------------------------------------------------------------
    assign send_active    = compat_flit_valid_r;
    assign drift_trigger  = (drift_cnt >= csr_drift_period_i)
                          & ~clock_drift_stall
                          & ~send_active;
    assign backoff_last   = clock_drift_stall & (drift_backoff_cnt == 4'd1);
    assign backoff_next   = {4{backoff_last}} & 4'd0 | {4{~backoff_last}} & (drift_backoff_cnt - 4'd1);

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // 兼容 NULL 状态机（非抢占：父请求置位，装载事件拍后清位）
    // --------------------------------------------------------------------------
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            null_active <= 1'b0;
        else if (null_gen_req)
            null_active <= 1'b1;
        else if (null_compat_event)
            null_active <= 1'b0;
    end

    // --------------------------------------------------------------------------
    // ARQ 状态机（抢占型：请求锁存 / 启动 / 装载后清位）
    // --------------------------------------------------------------------------
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            arq_pending <= 1'b0;
        else if (arq_pending)
            arq_pending <= ~arq_start;
        else if (arq_req_pend)
            arq_pending <= ~arq_start;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            arq_active <= 1'b0;
        else if (arq_start)
            arq_active <= 1'b1;
        else if (arq_compat_event)
            arq_active <= 1'b0;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            arq_kind <= 2'd0;
        else if (~arq_active & arq_req_pend)
            arq_kind <= {2{ arq_req_valid_compat_i              }} & 2'd0
                      | {2{~arq_req_valid_compat_i & arq_ack_req }} & 2'd1
                      | {2{~arq_req_valid_compat_i & ~arq_ack_req}} & 2'd2;
    end

    // --------------------------------------------------------------------------
    // 1T 槽缓冲 load（流水线 1T 延迟：T1 收齐入槽 → T2 发出）
    // --------------------------------------------------------------------------
    always @(posedge mb_clk) begin
        if (!mb_rst_n) begin
            compat_flit_valid_r <= 1'b0;
            compat_flit_272_r   <= 272'd0;
        end
        else if (compat_rx_accept) begin
            compat_flit_valid_r <= 1'b1;
            compat_flit_272_r   <= compat_flit_272;
        end
        else if (emit_held) begin
            compat_flit_valid_r <= 1'b0;
        end
        // 否则保持不变
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

    // --------------------------------------------------------------------------
    // replay 写指针：每个正常数据 Flit push 递增 1（父模块不自分配，本模块自管理）
    // ==========================================================================
    // NOTE: MAS §5.1 伪码引用 "wptr_from_parent"，但 §2.1 端口表与父例化接口均无
    //       该输入；按父模块"内部自管理写指针"语义（与 sibling 512_lane1 一致）
    //       实现为内部 reg 自增，随数据 Flit push 逐 Flit 递增。
    // =========================================================================
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            replay_wptr_cnt <= {`REPLAY_BUF_PTR{1'b0}};
        else if (replay_push_wen_compat_o)
            replay_wptr_cnt <= replay_wptr_cnt + 1'b1;
    end

    // ==========================================================================
    // 子模块实例化
    // ==========================================================================
    // 内嵌 crc9_generate（纯组合，0 cycle 输出，覆盖当前装载源 Header+Payload 261 bit）
    crc9_generate #() u_crc9 (
        .mb_clk     (mb_clk),
        .mb_rst_n   (mb_rst_n),
        .crc9_data_i(flit_src_261[260:0]),
        .crc9_o     (chk)
    );

endmodule
