//------------------------------------------------------------------------------
// Module:       link_tx_512_lane1
// Author:       OpenHappy Team
// MAS:          spec/link_layer/submods/link_tx/mas/link_tx_512_lane1_mas.md (v0.1)
// Function:     512 PAIF TX lane1 —— 四路独立控制框架（paif / null / retry / udi）
//               1) paif 路：pre-format 4 臂 mux + 单 512 寄存器；accept 权威装载/发送；
//                  入口反压保 1 拍在途；replay push 锚 T2/T4（wen={push_back,push_front}）
//               2) null 路：自生成 4 臂 mux + 单 512 寄存器；start+1 首拍实发，4 拍零间隔
//               3) retry 路：行流式自走（buffer Q ready +1T 行稳定 2 拍）；4 拍/Flit 零间隔；
//                  LO 拍发低 512（back 行 DLP 替换）、HI 拍发 {chk_q, 高 496}
//               4) udi 路：1T pipe 全 1024（csr_protocol_mode_i=0 直通）
//               出口：四路独立门控 OR（结构不变量，vld/data 同源单级；sa 外提：vld = sa & 四路 pending OR）
//               窗口：三路独立门控 OR（udi 无项；{16'h0, hi496, lo512} 1008 口径，chk 槽 tie 0）
//               注错：paif/retry 路 LO 发送拍 1-bit XOR bit16（fb 极性：fb=0 命中 front）
//               钟漂：4 路全空闲且计数达 csr_drift_period_i 触发 8 拍反压（stall 期间 sa=0）
//
// Version History:
//   2026-09-24  OpenHappy Team  0 版本首发（MAS v0.1 四路独立控制框架，1:1 实现）
//   2026-09-24  OpenHappy Team  本树编码约束转换（designer 口径）：复位改同步
//                          （always @(posedge mb_clk) + if (!mb_rst_n)，敏感列表不含异步复位边沿）；
//                          组合 5 个 case mux 转纯 assign AND/OR 链（assign-only）；
//                          retry_send 加 retrans_active_r 门控（Q1 决议：防末 Flit 后
//                          重泵 stale 行，MAS v0.1 §5.1 L420 同步口径）
//   2026-09-24  OpenHappy Team  lint 修正：W116 2:1 链/chk 两选一/null_field 选择位复制到向量宽
//                          （{W{sel}}，1-bit sel 零扩展致每臂仅剩 bit0 的功能缺陷）；
//                          窗口输出口径 {16'h0, 1008 三路 OR}（MAS §3.4/§5.1 同步）
//   2026-09-24  OpenHappy Team  paif_load 改 accept&vld（lint A.1-19：vld 读取扇出；协议等价——
//                          accept 拍父必给 vld=1 且尊重 paif_tx_ready_o）
//   2026-09-25  OpenHappy Team  reg2out ≤6：出口 vld/data sa 外提（纯代数重组，不加
//                          pipeline/锁存）——vld = sa & 四路 pending OR（tx_busy，
//                          单级 AND；旧四 send 各自 &sa 再 OR，sa 门控 4 次）；data
//                          MUX 选位改 sa-less pending 项（retry 路 = retry_pend 新
//                          wire、udi 路 = udi_vld_reg，旧 udi_send_o/retry_send_o
//                          出口项删除；retry_send 保 sa 门控，模块内寄存器时序
//                          不变）；sa=1 同拍 bit-exact，sa=0 按 MAS §3.4 裁决
//                          （四路 send=0 → vld=0/data 门 0，pending 保持零丢失，
//                          udi 1T 在途缓冲 udi_send=0）。模块内时序（clear/win/
//                          tick/push/注错）与窗口/replay/tx_busy 零 diff。
//   2026-09-29  OpenHappy Team  LL-TIMING-N1 reg2reg 2 引擎拆分（retry 臂拆出/父专用引擎）
//                          回退（功能回归：CASE_14/2 重传 chk 槽在 X 过渡拍/重传拍
//                          与基线分叉，d559/d558 替代原窗口 CRC）——恢复原三臂门控 OR
//                          单窗口结构。24 级 reg2reg 控制穿数据链（line_active/
//                          retry_phase→retry_win→retry 臂→12 级 XOR 树→crc_code）
//                          保持原样，ESCALATE 已回报 user（出口加 1 级发送寄存或
//                          放开 sa 数据门控裁决）。
//   2026-09-29  OpenHappy Team  编译期参数单一事实源切换（用户纪律：不维护两套参数文件）：include
//                          link_layer_params.vh → link_layer_defines.vh；模块内纯行索引 localparam
//                          改名 REPLAY_ROW_IDX_W（避让同名宏 REPLAY_BUF_PTR=5 含 flag，值 4 不变；
//                          端口字面 3:0 与全部位宽不变）
//   2026-10-08  OpenHappy Team  LL-20260930-N4-v3（N4 三超标锥）：R2 sa 外提（skill6 晚合并：
//                          臂内 {512{pv&sa}} 3×512 复制点移除，出口 512-bit AND 处与臂 OR
//                          合并一次——分配律恒等，含 X 签名 bit-identity；sa 同宽 {512{sa}}
//                          形态（VCS 2023.12 跨宽 concat 元素截断 bug 规避，本地模型核证
//                          ≡ 旧臂内形态；vld / paif_ready / 3 sends / udi 臂 sa-less 全
//                          不动）+ F-a retry 窗口同式 2×504 扇出切半（retry_win_a/b
//                          严格逻辑复制各驱半区，位序 {hi496, lo512} 不变，null/paif
//                          臂 1008 窗口不动；端口目标上下文本地模型核证 ≡ 旧 1008 窗口
//                          形态）。零功能变化。
//   2026-10-08  OpenHappy Team  LL-20261008-sa-data-9level（skill7 因子空间分区复制 K=4，512 = 4×128 均分，arch v2.1
//                          Option C）：出口 data cpif_tx_data_512_lane1_o 重写——sa 外提复制为 12 因子副本
//                          （sa & <arm_valid>）× 4 组（同式同输入读原 sa/arm net，F-a 同形态），每 net 只驱本组
//                          128-bit 数据门：w_i = 组内 128-bit 3 臂 OR（| 操作数 = 单元素 concat 选位 + &
//                          表达式，& 优先级高于 | 无括号，skill5；同宽 {W{sel}} 复制，纪律①），lo_512 =
//                          {w3,w2,w1,w0} 4 元素裸 RHS concat（paif_out_o 3 元素 RHS concat 先例同语法位置；
//                          VCS 2023.12 | 操作数禁内嵌多元素 concat——SE 约束仅针对 | 操作数位置，不针对裸
//                          RHS）；出口 = udi 项（字面不动，sa-less 全 1024）| {512'b0, lo_512}（2 元素 |
//                          操作数，与现行同形态）；sa 仍在 data 臂（D1），sa=0 → 12 因子副本全 0 → 三路
//                          数据门 0（MAS §3.4 契约字面不变），pending 零丢失（udi 1T 在途缓冲）；sa 分发梯
//                          512-loads → 128-loads × 4（下轮 DC 验收 5/6：reg2out_tx 各族 ≤10 + netlist 因子
//                          副本分离）。零功能变化 / 零新寄存器 / 零 pipeline。
//   2026-10-09  OpenHappy Team  BUG-③/BUG-④（front slot 0 数据窗口拍集 + paif 注错拍恢复；arch 评审通过；
//                          参照 MAS v0.13 §5.1/§8、spec §4.2.3.2，spec 不动）：
//                          ③ L239 paif_chk_smp / L248 paif_win 1-bit 选位 paif_beat_cnt[1]→[0]
//                          （拍集 T3/T4→T2/T4，v0.4 编辑族笔误——T2 窗口=0 致 front 单元 chk 槽 =
//                          CRC(0 窗口) 常数 0xD558/0x0000，front 首送必 CRC 失败，15-case NACK/重传
//                          风暴家族）；④ L249 paif_inj_fire 恢复 0 版本原式 csr_inj_en_i &
//                          ~(csr_inj_fb_i ^ paif_beat_cnt[1]) & paif_win（fb=0 命中 front T2/W1 LO
//                          拍、fb=1 命中 back T4/W3 LO 拍，与端口契约一致；v0.4 曾误 T5/W4、T3/W2
//                          且 front/back 对调）。timing 结构性零变化（1-bit 选位换位；K4 出口段 /
//                          窗口 OR 块 / pre-format / push 锚 / 时序块 / retry 注错全未触）。
//   2026-10-10  OpenHappy Team  LL-20261010-R2G1 B 票（R-B1b：重传窗 X 读家族 c24/c8；arch LLArchG1B 通过）：
//                          replay push 锚 send→collect（paif_load = accept & vld）——spec §4.2.3.2
//                          T2/T4 收齐拍锚定（accept⊆pushed 结构不变量），与出口发送状态解耦；
//                          常态 sa=1 collect 拍 = send 拍（pipe 1 拍流水）同拍零回归（c22 / ③
//                          家族签名不回退）；分叉面 = sa=0 / 子管冻结角（push 不再随 send 丢失）。
//                          两行 assign + 段注释；时序块 / 出口 K4 段 / 窗口 OR / pre-format /
//                          注错（③④ 修复面 L249/L258/L259）/ retrans drain / wptr 递进全未触。
//   2026-10-13  OpenHappy Team  LL-20261013-BWREVERT（user 指令：回退纯带宽改动 §F c10，恢复 2026-10-12
//                          前结构）——带宽缩减发送机制整体移除：csr_bw_mode_i 2-bit 全量透传回退
//                          1-bit 恒 0 截位；出口带宽缩减寄存器 / 出口 W 拍队列 / 切片输出 / 发送拍
//                          门控共因子 *_g / K4 出口第 4 臂及其数据门全部删除，出口回归 3 臂 12 因子
//                          K4 结构；§F 门控改名点（paif_clear / null_clear / null_done_o /
//                          retry_row_done / retrans_tick_o / null 装载 / retry_phase 翻转）恢复
//                          *_send_o / *_chk_smp 原读判据；R-B1b（2026-10-10 replay push 锚）与
//                          BUG-③/④（2026-10-09）保留。
//------------------------------------------------------------------------------
`timescale 1ns/1ps

module link_tx_512_lane1 (
    // ── 时钟/复位 ──
    input  wire                     mb_clk,                    // 模块时钟，mb 域
    input  wire                     mb_rst_n,                  // 低有效同步复位（always @(posedge mb_clk) + if (!mb_rst_n)）
    // ── CSR（父 link_tx 经端口传入） ──
    input  wire                     csr_ecc_en_i,              // 1=chk 取 ecc_chk，0=取 crc_chk
    input  wire                     csr_inj_en_i,              // 注错使能（paif/retry 路）
    input  wire                     csr_inj_fb_i,              // 0=命中 front、1=命中 back
    input  wire                     csr_protocol_mode_i,       // 1=PAIF 四路框架 / 0=UDI 直通
    input  wire       [15:0]        csr_drift_period_i,        // 钟漂反压间隔（拍）
    input  wire                     csr_s2p_mode_i,            // 本形态恒 0（端口契约保留）
    input  wire                     csr_bw_mode_i,             // 本形态恒 0（端口契约保留）
    // ── 父 link_tx 寄存源（512 宽） ──
    input  wire       [511:0]       header_512_lane1_i,        // TX Header（front 装载拍嵌低 16 槽）
    input  wire       [511:0]       dlp_512_lane1_i,           // DLP（back 装载拍嵌低 16 槽；retry back 行替换源）
    // ── PAIF 入口 ──
    input  wire       [511:0]       paif_tx_data_i,            // 源 512-bit 数据流（4 beat × 512/单元）
    input  wire                     paif_tx_vld_i,             // 数据有效（本模块以 accept 权威握手）
    input  wire                     paif_rx_accept_i,          // 父权威接收（装载拍 + push 锚）
    // ── 重传行流（父 flit_sched / replay_buffer 行流读接线） ──
    input  wire                     retrans_start_i,           // 重传起始（脉冲）
    input  wire        [7:0]        retrans_len_i,             // 重传长度（Flit 数，start 拍采样）
    input  wire                     retrans_line_ready_i,      // 行就绪（与 LO 发送拍对齐）
    input  wire       [1023:0]      retrans_line_data_i,       // 行流读数据（Q ready +1T，稳定 2 拍）
    // ── NULL 自生成 / UDI 直通 ──
    input  wire                     null_flit_start_i,         // NULL 起始（父单事件：null_start + 全路空闲）
    input  wire       [1023:0]      udi_data_i,                // UDI TX 数据（每拍无条件寄存 1T）
    input  wire                     udi_vld_i,                 // UDI TX 有效（每拍无条件寄存 1T）
    // ── 父 crc16/ecc 组合回接（对窗口同拍输出） ──
    input  wire        [15:0]       crc_chk_512_lane1_i,       // CRC 模式 chk
    input  wire        [15:0]       ecc_chk_512_lane1_i,       // ECC 模式 chk（16 位全宽回接）
    input  wire                     cpif_tx_ready_i,           // 保留不接（MAS §2.1：无出口 FIFO，不构成反压）
    input  wire                     link_tx_active_i,          // 出口使能（S_NORMAL=1）
    // ── replay push（父 replay_buffer 行流 push 3 口） ──
    output wire      [1:0]          replay_push_wen_o,         // wen[0]=push_front / wen[1]=push_back
    output wire      [1023:0]       replay_push_data_o,        // {paif_tx_data_i, paif_data_reg}（注入前）
    output wire      [3:0]          replay_push_wptr_o,        // replay_wptr_cnt；宽度 REPLAY_ROW_IDX_W=4
                                                               // （MAS §2.1 = $clog2(REPLAY_BUF_DEPTH) 纯行索引；
                                                               //  V2005 端口列表不前向引用 body 参数，取字面 3:0）
    // ── 行流读调度 / 完成 / 入口反压 ──
    output wire                     retrans_tick_o,            // LO 发送拍脉冲（buffer 每 2 拍读 1 行）
    output wire                     retrans_flit_done_o,       // Flit 末拍实发脉冲（back 行 HI 拍）
    output wire                     null_done_o,               // NULL 末拍实发脉冲（第 4 拍）
    output wire                     paif_tx_ready_o,           // 入口反压（保 1 拍在途）
    output wire                     tx_busy_512_lane1_o,       // 任一路 pending（父 ICG / 钟漂判定）
    // ── 出口（父 512→256 适配 lane0） ──
    output wire      [1023:0]       cpif_tx_data_512_lane1_o,  // 四路独立门控 OR（vld=0 时 data 门 0）
    output wire                     cpif_tx_data_512_lane1_vld_o, // 四路 send 门控 OR（全 reg-out）
    // ── 窗口（父 crc16_1024 / ecc_512_gen） ──
    output wire      [1023:0]       crc_data_512_lane1_o       // 三路独立门控 OR（{16'h0, 1008 窗口}，chk 槽 tie 0）
);

    // ════════════════════════════════════════════════════════════════════════
    // 编译期参数（MAS §2.1：经 `include 进入，不入端口）
    // ════════════════════════════════════════════════════════════════════════
    `include "link_layer_defines.vh"               // 编译期参数/宏单一事实源（REPLAY_BUF_DEPTH = 16）
    localparam REPLAY_ROW_IDX_W = $clog2(`REPLAY_BUF_DEPTH); // = 4（纯行索引，不含 flag 位；defines.vh `REPLAY_BUF_PTR 宏 = 5 含 flag，语义不同本模块改名避让；端口字面宽度 3:0 与此一致）

    // ════════════════════════════════════════════════════════════════════════
    // 声明（先声明后使用；顺序 reg 与组合 wire 分置）
    // ════════════════════════════════════════════════════════════════════════
    // ── 顺序信号（同步低复位：always @(posedge mb_clk) + if (!mb_rst_n)；
    //    4 个纯 pipe 数据寄存器无复位——MAS §5.3.1 豁免） ──
    reg  [511:0]                   paif_data_reg;              // paif 在途 512（pre-format；无复位）
    reg                           paif_data_valid;             // paif pending
    reg  [1:0]                     paif_beat_cnt;              // paif 拍索引 0..3（3→0 回绕）
    reg  [15:0]                    paif_chk_q;                 // paif chk 采样（win 拍沿）
    reg  [511:0]                   null_data_reg;              // null 在途 512（自生成；无复位）
    reg                           null_data_valid;             // null pending
    reg  [1:0]                     null_cnt;                   // null 拍索引 0..3
    reg  [15:0]                    null_chk_q;                 // null chk 采样（win 拍沿）
    reg                           retrans_active_r;            // 重传进行中（len 个 Flit 后自停）
    reg  [7:0]                     retrans_len_r;              // 重传 Flit 总数
    reg  [7:0]                     retrans_flit_cnt_r;         // 已消费 Flit 计数
    reg  [15:0]                    retry_chk_q;                // retry chk 采样（LO 发送拍沿）
    reg                           retry_phase;                 // 0=LO 拍 1=HI 拍（每发送拍翻转）
    reg                           retry_row_parity;            // 0=front 行 1=back 行（每行完成翻转）
    reg                           retry_line_valid;            // 行 pending（HI 发送拍清）
    reg  [1023:0]                  udi_beat_reg;               // udi 在途 1024（无复位）
    reg                           udi_vld_reg;                 // udi 有效 1T（无复位）
    reg  [REPLAY_ROW_IDX_W-1:0]    replay_wptr_cnt;            // replay 写指针（T4 push_back 拍递增）
    reg  [15:0]                    drift_cnt;                  // 钟漂计数器（持续计数不归零）
    reg                           clock_drift_stall;           // 钟漂反压锁存（装载 8 拍递减归零释放）
    reg  [3:0]                     drift_backoff_cnt;          // 钟漂 8 拍反压递减
    // ── 组合 mux 信号（assign AND/OR 链；本树编码约束：组合逻辑 assign-only；
    //    选择位均复制到向量宽 {W{sel}}——W116，1-bit sel 零扩展会使每臂仅剩 bit0） ──
    wire [511:0]                   paif_pre;                   // 下一拍 pre-format（4 臂 2:1 链）
    wire [511:0]                   paif_pre_m01;               // 2:1 级：arm0/arm1（sel cnt[0]）
    wire [511:0]                   paif_pre_m23;               // 2:1 级：arm2/arm3（sel cnt[0]）
    wire [511:0]                   null_cur;                   // 当前拍自生成（4 臂 2:1 链）
    wire [511:0]                   null_cur_m01;               // 2:1 级：arm0/arm1（sel cnt[0]）
    wire [511:0]                   null_cur_m23;               // 2:1 级：arm2/arm3（sel cnt[0]）
    wire [511:0]                   null_nxt;                   // 下一拍自生成（4 臂 2:1 链）
    wire [511:0]                   null_nxt_m01;               // 2:1 级：arm0/arm1（sel cnt_next[0]）
    wire [511:0]                   null_nxt_m23;               // 2:1 级：arm2/arm3（sel cnt_next[0]）
    wire [511:0]                   retry_data;                 // retry LO 拍 512（front 原样 / back 替换，2:1）
    // ── 组合信号（assign 驱动，wire 型） ──
    wire [15:0]                    crc_code;                   // chk 16 位全宽两选一（ecc_en）
    wire                           sa;                         // 发送允许 = link_tx_active & ~drift
    wire                           tx_busy;                    // 四路 pending OR
    wire                           drift_trig;                 // 钟漂触发（4 路全空闲 + 达周期）
    // paif 路
    wire                           paif_ready;                 // accept 就绪（~valid | sa）
    wire                           paif_load;                  // accept & vld 装载
    wire [1:0]                     paif_cnt_next;              // 拍索引 next（3→0）
    wire                           paif_clear;                 // 末拍清（T5 沿）
    wire                           paif_chk_smp;               // chk 采样（win 拍沿）
    wire                           paif_send_o;                // paif 路发送
    wire                           paif_win;                   // paif 窗口（win 拍）
    wire                           paif_inj_fire;              // paif 注错（1-bit）
    wire [511:0]                   paif_out_o;                 // paif 路出口 512（注错后）
    // null 路
    wire [1:0]                     null_cnt_next;              // 拍索引 next（3→0）
    wire                           null_send_o;                // null 路发送
    wire                           null_clear;                 // 末拍清（第 4 拍沿）
    wire                           null_chk_smp;               // chk 采样（win 拍沿）
    wire                           null_win;                   // null 窗口（win 拍）
    wire [15:0]                    null_field;                 // null LO 拍低 16（header/dlp 两选一）
    wire [511:0]                   null_out_o;                 // null 路出口 512（不注错）
    // retry 路
    wire                           retry_pend;                 // sa-less 行发送（出口 MUX 选位；retry_send 去 sa 因子）
    wire                           retry_send;                 // 行就绪/行 pending 拍发送（sa 门控；模块内寄存器时序）
    wire                           retry_row_done;             // 行完成（HI 发送拍沿）
    wire                           retry_flit_last;            // Flit 完成（back 行完成 + 计数达 len）
    wire                           retry_chk_smp;              // chk 采样（LO 发送拍沿）
    wire [511:0]                   retry_lo_replaced;          // LO 行 DLP 替换 512（back 行）
    wire [495:0]                   retry_hi496;                // 行高 496（窗口与 HI 拍共用）
    wire                           retry_win_a;                // retry 窗口上 504（LO 发送拍；F-a 同式扇出切半复制 1）
    wire                           retry_win_b;                // retry 窗口下 504（LO 发送拍；F-a 同式扇出切半复制 2）
    wire                           retry_inj_fire;             // retry 注错（1-bit）
    wire [511:0]                   retry_out_o;                // retry 路出口 512（注错后）
    // udi 路（sa 外提：出口选位 = udi_vld_reg，协议门已沉入寄存器侧；udi_send_o 删除）
    wire [1023:0]                  udi_out_o;                  // udi 路出口（全 1024）
    // replay push
    wire                           push_front;                 // T2 拍 push_front
    wire                           push_back;                  // T4 拍 push_back
    // K=4 因子空间分区复制（LL-20261008-sa-data-9level，arch v2.1 Option C；先声明后使用）：
    // 12 因子副本（sa & arm_valid × 4 组，同式同输入）+ 4 组内 128-bit 3 臂 OR + 出口低 512
    wire                           sa_pv0, sa_pv1, sa_pv2, sa_pv3;  // paif 数据门因子副本 g0~g3（sa & paif_data_valid）
    wire                           sa_nv0, sa_nv1, sa_nv2, sa_nv3;  // null 数据门因子副本 g0~g3（sa & null_data_valid）
    wire                           sa_ra0, sa_ra1, sa_ra2, sa_ra3;  // 重传数据门因子副本 g0~g3（sa & retrans_active_r）
    wire [127:0]                   w0, w1, w2, w3;             // 组内 128-bit 3 臂 OR g0~g3（位序 w3 高 → w0 低）
    wire [511:0]                   lo_512;                     // 出口低 512 = {w3,w2,w1,w0} 4 元素裸 RHS concat

    // ════════════════════════════════════════════════════════════════════════
    // 组合逻辑（assign-only；本树编码约束：组合 mux 用 AND/OR 2:1 链，不用行为型组合块；
    //   选择位一律 {W{sel}} 复制到向量宽——W116）
    // ════════════════════════════════════════════════════════════════════════
    // ── 公共：chk 两选一 / 发送允许 / 四路 busy / 钟漂触发（MAS §5.1） ──
    assign crc_code = ({16{~csr_ecc_en_i}} & crc_chk_512_lane1_i)
                    | ({16{ csr_ecc_en_i}} & ecc_chk_512_lane1_i); // 16 位全宽两选一（选择位 {16{sel}} 复制）
    assign sa       = link_tx_active_i & ~clock_drift_stall;   // sa = link_tx_active & ~drift（不锁存）
    assign tx_busy  = paif_data_valid | null_data_valid
                    | retrans_active_r ;          // 四路任一 pending
    assign tx_busy_512_lane1_o = tx_busy;
    assign drift_trig = ~clock_drift_stall
                      & (drift_cnt >= csr_drift_period_i)
                      & ~tx_busy;                              // 全路空闲且计数达周期

    // ── paif 路（MAS §5.1 paif 段；拍表 §3.1） ──
    assign paif_ready    = ~paif_data_valid | sa;              // 保 1 拍在途
    assign paif_load     = paif_rx_accept_i & paif_tx_vld_i;   // accept & vld 装载（vld 读取扇出，A.1-19）
    assign paif_cnt_next = (paif_beat_cnt + 2'd1) & {2{~(&paif_beat_cnt)}}; // 3→0（MAS 三目 → 屏蔽加）
    assign paif_send_o   = paif_data_valid & sa;
    assign paif_clear    = paif_send_o & (~|paif_beat_cnt);                     // 末拍（T5 沿）
    assign paif_chk_smp  = paif_send_o & paif_beat_cnt[0]; // win 拍沿（T2/T4，BUG-③：v0.4 拍集笔误恢复，MAS v0.13 §8）
    // 下一拍 pre-format 4 臂 mux（T1 装载 cnt=0 → T2 发 beat1；HI 臂取当拍窗口 crc_code；
    //   case → 3 级 2:1 AND/OR 链，语义零变化；选择位 {512{sel}} 复制——W116）
    assign paif_pre_m01  = ({512{ paif_beat_cnt[0]}} & {crc_code,              paif_tx_data_i[495:0]})
                          | ({512{~paif_beat_cnt[0]}} & {paif_tx_data_i[511:16], header_512_lane1_i[15:0]});
    assign paif_pre_m23  = ({512{ paif_beat_cnt[0]}} & {crc_code,              paif_tx_data_i[495:0]})
                          | ({512{~paif_beat_cnt[0]}} & {paif_tx_data_i[511:16], dlp_512_lane1_i[15:0]});
    assign paif_pre      = ({512{ paif_beat_cnt[1]}} & paif_pre_m23)
                          | ({512{~paif_beat_cnt[1]}} & paif_pre_m01);
    assign paif_win      = paif_data_valid & paif_beat_cnt[0]; // win 拍（T2/T4，BUG-③：v0.4 拍集笔误恢复，MAS v0.13 §8）
    assign paif_inj_fire = csr_inj_en_i & ~(csr_inj_fb_i ^ paif_beat_cnt[1]) & paif_win; // BUG-④：0 版本原式——fb=0 命中 front（T2/W1 LO 拍）、fb=1 命中 back（T4/W3 LO 拍），MAS v0.13 端口契约（v0.4 编辑族笔误恢复；~(fb^cnt[1]) 括号为功能必需）
    assign paif_out_o    = {paif_data_reg[511:17], paif_data_reg[16]^paif_inj_fire,paif_data_reg[15:0]};                // 1-bit XOR bit16
    assign paif_tx_ready_o = paif_ready;                                                  // 入口反压

    // ── null 路（MAS §5.1 null 段；拍表 §3.2；4 拍自生成零间隔） ──
    assign null_cnt_next = (null_cnt + 2'd1) & {2{~(null_cnt == 2'd3)}};     // 3→0
    assign null_send_o   = null_data_valid & sa;
    assign null_clear    = null_send_o & (null_cnt == 2'd3);                 // 末拍（第 4 拍沿）
    assign null_chk_smp  = null_send_o & ~null_cnt[0]; // win 拍沿
    // 当前拍自生成 4 臂 mux（beat1 header / beat2 chk / beat3 dlp / beat4 chk；case → 2:1 链）
    assign null_cur_m01  = ({512{ null_cnt[0]}} & {crc_code, 496'h0})
                          | ({512{~null_cnt[0]}} & {496'h0, header_512_lane1_i[15:0]});
    assign null_cur_m23  = ({512{ null_cnt[0]}} & {crc_code, 496'h0})
                          | ({512{~null_cnt[0]}} & {496'h0, dlp_512_lane1_i[15:0]});
    assign null_cur      = ({512{ null_cnt[1]}} & null_cur_m23)
                          | ({512{~null_cnt[1]}} & null_cur_m01);
    // 下一拍自生成 4 臂 mux（发送拍沿装载，索引 null_cnt_next；case → 2:1 链）
    assign null_nxt_m01  = ({512{ null_cnt_next[0]}} & {crc_code, 496'h0})
                          | ({512{~null_cnt_next[0]}} & {496'h0, header_512_lane1_i[15:0]});
    assign null_nxt_m23  = ({512{ null_cnt_next[0]}} & {crc_code, 496'h0})
                          | ({512{~null_cnt_next[0]}} & {496'h0, dlp_512_lane1_i[15:0]});
    assign null_nxt      = ({512{ null_cnt_next[1]}} & null_nxt_m23)
                          | ({512{~null_cnt_next[1]}} & null_nxt_m01);
    assign null_win      = null_data_valid & ~null_cnt[0]; // win 拍
    assign null_field    = ({16{~null_cnt[1]}} & header_512_lane1_i[15:0])
                          | ({16{ null_cnt[1]}} & dlp_512_lane1_i[15:0]);     // LO 拍低 16（选择位 {16{sel}}）
    assign null_out_o    = null_data_reg;                                      // null 不注错
    assign null_done_o   = null_send_o & (&null_cnt);                   // 末拍实发

    // ── retry 路（MAS §5.1 retry 段；拍表 §3.3；行流式自走 4 拍/Flit 零间隔） ──
    // 发送：行就绪拍或行 pending 拍，含 retrans_active_r 门控（buffer 侧 line_q_valid 为电平，
    //   末次行读后保持 1；active 门控防末 Flit 后重泵 stale 行，MAS v0.1 §5.1 L420）
    assign retry_pend   = (retry_line_valid | retrans_line_ready_i) & retrans_active_r; // sa-less（出口选位）
    assign retry_send   = retry_pend & sa;   // sa 门控（模块内时序不变）
    assign retry_row_done  = retry_send & (retry_phase == 1'b1);              // 行完成（HI 发送拍沿）
    assign retry_flit_last = retry_row_done & retry_row_parity
                          & (retrans_flit_cnt_r + 8'd1 >= retrans_len_r);     // len 个 Flit 后自停
    assign retry_chk_smp   = retry_send & (retry_phase == 1'b0);              // LO 发送拍沿
    assign retrans_tick_o  = retry_chk_smp;                                   // tick = retry_send & phase==0
    // LO 行 DLP 替换（back 行；front 行原样）
    assign retry_lo_replaced = {retrans_line_data_i[511:16], dlp_512_lane1_i[15:0]};
    assign retry_data        = ({512{ retry_row_parity}} & retry_lo_replaced)
                              | ({512{~retry_row_parity}} & retrans_line_data_i[511:0]); // case → 2:1（{512{sel}}）
    assign retry_hi496       = retrans_line_data_i[1007:512];                 // 行高 496（窗口与 HI 拍共用）
    // 出口 512：LO 拍 = retry_data；HI 拍 = {chk_q, 行高 496}（case → 2:1）
    // F-a（LL-20260930-N4-v3）：窗口同式 2×504 扇出切半（skill7 严格逻辑复制：a/b
    // 读原 retry_phase；retry_win_a 驱上 504、retry_win_b 驱下 504，替代旧 1008 单 net
    // 窗口高扇出缓冲树）
    assign retry_win_a       = retry_pend & ~retry_phase;             // 窗口 = LO 发送拍（上 504）
    assign retry_win_b       = retry_pend & ~retry_phase;             // 窗口 = LO 发送拍（下 504）
    assign retry_inj_fire    = csr_inj_en_i & (~csr_inj_fb_i &~retry_row_parity|csr_inj_fb_i&retry_row_parity) ; // fb=0 命中 front
    assign retry_out_o       =  {512{ retry_phase}} & {retry_chk_q, retrans_line_data_i[1007:512]}
                              | {512{~retry_phase}} & {retry_data[511:17],retry_data[16]^retry_inj_fire,retry_data[15:0]} ;    // 1-bit XOR bit16
    assign retrans_flit_done_o = retry_row_done & retry_row_parity;           // Flit 末拍（back 行 HI 拍）

    // ── udi 路（MAS §5.1 udi 段；1T pipe 全 1024；协议门沉入 udi_vld_reg 寄存侧，
    //    出口选位 = udi_vld_reg，sa 外提到出口外层门） ──
    assign udi_out_o  = udi_beat_reg;

    // ── 出口：K=4 因子空间分区复制（MAS §5.1 出口段；LL-20261008-sa-data-9level，arch v2.1
    //    Option C；替代 R2 出口形态，R2 历史见版本条目） ──
    // vld = 四路 pending OR（tx_busy）单级 AND sa（旧四 send 各自 &sa 再 OR，sa
    // 门控降为 1 次）；data MUX 选位 = sa-less pending 项（retry 路 = retry_pend，
    // 与旧 send 项 1:1 同构）；K=4 因子空间分区复制（skill7 扇出分担）：sa 外提复制为
    // 12 因子副本（sa & <arm_valid>）× 4 组（同式同输入读原 sa/arm net，纪律③，F-a
    // 同形态），每 net 只驱本组 128-bit 数据门——原单 512-loads 分发梯拆为 4 组
    // 128-loads（reg2out_tx ≤10，下轮 DC 验收 5/6）；sa 仍在 data 臂（D1），sa=0 →
    // 12 因子副本全 0 → 三路数据门 0（MAS §3.4 契约字面不变），pending 保持零丢失
    // （udi 1T 在途缓冲）；w_i = 组内 128-bit 3 臂 OR（| 操作数 = 单元素 concat
    // 选位 + & 表达式，与现行出口同形态；& 优先级高于 | 无括号，skill5；同宽
    // {W{sel}} 复制，纪律①）；lo_512 = {w3,w2,w1,w0} 4 元素裸 RHS concat（位序自明：
    // w3 → [511:384] … w0 → [127:0]；paif_out_o 3 元素 RHS concat 先例同语法位置——
    // VCS 2023.12 SE 约束仅针对 | 操作数内嵌多元素 concat，不针对裸 RHS，形态注
    // 见下窗口段）；出口 = udi 项（字面不动，sa-less 全 1024）| {512'b0, lo_512}
    // （2 元素 | 操作数，与现行同形态）；bit-identity（验收 1）：新低 512[b] =
    // sa&pv&paif[b] | sa&nv&null[b] | sa&ra&retry[b] ≡ 旧出口（分配律恒等，0/1/X
    // 全域；in-TB N6 R2 探针同式互证含 X 签名）
    assign sa_pv0 = sa & paif_data_valid;   // g0 paif 数据门因子副本
    assign sa_pv1 = sa & paif_data_valid;   // g1 paif 数据门因子副本
    assign sa_pv2 = sa & paif_data_valid;   // g2 paif 数据门因子副本
    assign sa_pv3 = sa & paif_data_valid;   // g3 paif 数据门因子副本
    assign sa_nv0 = sa & null_data_valid;   // g0 null 数据门因子副本
    assign sa_nv1 = sa & null_data_valid;   // g1 null 数据门因子副本
    assign sa_nv2 = sa & null_data_valid;   // g2 null 数据门因子副本
    assign sa_nv3 = sa & null_data_valid;   // g3 null 数据门因子副本
    assign sa_ra0 = sa & retrans_active_r;  // g0 重传数据门因子副本
    assign sa_ra1 = sa & retrans_active_r;  // g1 重传数据门因子副本
    assign sa_ra2 = sa & retrans_active_r;  // g2 重传数据门因子副本
    assign sa_ra3 = sa & retrans_active_r;  // g3 重传数据门因子副本
    assign w0 = {128{sa_pv0}} & paif_out_o[127:0]   | {128{sa_nv0}} & null_out_o[127:0]   | {128{sa_ra0}} & retry_out_o[127:0];
    assign w1 = {128{sa_pv1}} & paif_out_o[255:128] | {128{sa_nv1}} & null_out_o[255:128] | {128{sa_ra1}} & retry_out_o[255:128];
    assign w2 = {128{sa_pv2}} & paif_out_o[383:256] | {128{sa_nv2}} & null_out_o[383:256] | {128{sa_ra2}} & retry_out_o[383:256];
    assign w3 = {128{sa_pv3}} & paif_out_o[511:384] | {128{sa_nv3}} & null_out_o[511:384] | {128{sa_ra3}} & retry_out_o[511:384];
    assign lo_512 = {w3, w2, w1, w0};
    assign cpif_tx_data_512_lane1_vld_o = sa & tx_busy|udi_vld_reg;
    assign cpif_tx_data_512_lane1_o = {1024{udi_vld_reg}} & udi_out_o | {512'b0, lo_512};
    // debug 断言：四路互斥（父单事件保证；同拍 >1 路 send = 接线错误）
    // `ifdef DEBUG
    //     assert property (@(posedge mb_clk)
    //         $onehot0({paif_data_valid, null_data_valid, retry_pend, udi_vld_reg}));
    // `endif

    // ── 窗口：三路独立门控 OR（udi 无项；MAS §3.4/§5.1 口径：1024 输出 = {16'h0, 1008 窗口}，
    //   1008 = {16'h0, hi496, lo512} 去 16 位 chk 槽；CRC/ECC 引擎按 1008 口径消费）──
    // F-a（LL-20260930-N4-v3）：retry 臂 1008 窗口切 2×504——retry_win_a 驱上组
    //   W[1007:504] = {retry_hi496, retry_data[511:504]}、retry_win_b 驱下组 W[503:0] =
    //   retry_data[503:0]（skill7 严格逻辑复制：a/b 同式同输入、读原 retry_phase、
    //   每窗口位仅驱本半区 504 loads；bit-identity 含 X 签名 ≡ 旧单 net retry_win）；
    //   null/paif 臂 1008 窗口不动（非 critical start，禁无差别复制）。
    //   形态注：VCS 2023.12 不解析「| 操作数内嵌多元素 concat」（括号/裸形均 SE），
    //   半区臂用 {{N{…}}, x} / {x, {N{…}}} 双括号零扩展习语（同 credit_fifo 编码约束
    //   5 先例形态）：上组下 504 零扩、下组上 504 零扩，1008 位 OR 合流；
    //   20 万拍 X-sweep 模型核证 out_old === out_new（fa_final/tb_fa_r2）
    assign crc_data_512_lane1_o = {16'h0,
             ({1008{null_win}}  & {496'h0, 496'h0, null_field})
          |  ({1008{paif_win}}  & {paif_tx_data_i[495:0], paif_data_reg[511:0]})
          |  ({{504{retry_win_a}}, {504{1'b0}}} & {retry_hi496, retry_data[511:504], {504{1'b0}}})
          |  ({504'h0, {504{retry_win_b}}} & {{504{1'b0}}, retry_data[503:0]})};

    // ── replay push（锚 = paif 收齐拍（collect = accept & vld；BUG-RB1b）；行 = 注入前数据；MAS §5.1 push 段） ──
    assign push_front = paif_load & (paif_beat_cnt == 2'd1);                  // T2 收齐拍（collect 锚，spec §4.2.3.2；BUG-RB1b）
    assign push_back  = paif_load & (paif_beat_cnt == 2'd3);                  // T4 收齐拍（collect 锚，spec §4.2.3.2；BUG-RB1b）
    assign replay_push_wen_o  = {push_back, push_front};
    assign replay_push_data_o = {paif_tx_data_i, paif_data_reg};              // 注入前
    assign replay_push_wptr_o = replay_wptr_cnt;

    // ════════════════════════════════════════════════════════════════════════
    // 顺序逻辑（同步低复位：always @(posedge mb_clk) + if (!mb_rst_n)；
    //   1 always 驱 1 寄存器；不含复位 ≤2 if 分支）
    // ════════════════════════════════════════════════════════════════════════
    // ── paif 路 ──
    // 在途数据：纯 pipe 无复位（§5.3.1 豁免）；T1 accept 沿装载下一拍 pre-format
    always @(posedge mb_clk) begin
        if (paif_load)
            paif_data_reg <= paif_pre;
    end
    // pending：load 优先 clear/排空（T4-accept 同拍重载零丢失；MAS §5.1 paif 段）
    // C7 重传 livelock 修复 Fix2（LL-20260925-CRIFVO-C7 §6）：retrans_start_i 拍排空子 PAIF 管
    //   让位重传——retry_enter 落于数据边界拍时，父入口反压停 accept（无 load），拍号卡 1，
    //   旧清 0 条件（paif_clear = send&cnt==0）永不可达，unit_busy_f 恒 1 反压父
    //   retrans_start 恒 0（三环互锁）。父 Fix1 删 ~unit_busy_f 门控后，管不排空则四路 OR
    //   数据选位（:273-277）同拍选中 pv 路（冻结行）+ 重传路 → 数据位级 OR 错。
    //   优先级 paif_load > (paif_clear | retrans_start_i)：互斥（start 要求父 retry_active=1
    //   → paif_ready_ctl=0 → accept=0 → paif_load=0，MAS L224 NULL/重传期间父停收），合并臂
    //   保持 load>clear 不变量。被排空行已同/前拍 push 入 replay（push 锚 :292-294），无丢失；
    //   四路出口 onehot0 不变量保持（start 拍 pv 清、次拍仅 retrans_active_r 一路 pending）。
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            paif_data_valid <= 1'b0;
        else if (paif_load)
            paif_data_valid <= 1'b1;
        else if (paif_clear | retrans_start_i)
            paif_data_valid <= 1'b0;
    end
    // 拍索引：装载拍推进（3→0）；C7 Fix2 retrans_start 拍归零
    //   （与 paif_clear 无关——旧行为 clear 拍拍号不动，保持；互斥论证见 pv 块注释）
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            paif_beat_cnt <= 2'd0;
        else if (paif_load)
            paif_beat_cnt <= paif_cnt_next;
        else if (retrans_start_i)
            paif_beat_cnt <= 2'd0;
    end
    // chk 采样于 win 拍沿（T2/T4（v0.13 恢复 spec 口径，BUG-③）；DFX/一致性，不参与本模块判定）
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            paif_chk_q <= 16'h0;
        else if (paif_chk_smp)
            paif_chk_q <= crc_code;
    end

    // ── null 路 ──
    // 在途数据：纯 pipe 无复位；start 拍装载 beat1，发送拍沿装载下一拍
    always @(posedge mb_clk) begin
        if (null_flit_start_i)
            null_data_reg <= null_cur;
        else if (null_send_o)
            null_data_reg <= null_nxt;
    end
    // pending：start 置 / 末拍清（parent 单事件保证 start 时全路空闲，两条件互斥）
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            null_data_valid <= 1'b0;
        else if (null_flit_start_i)
            null_data_valid <= 1'b1;
        else if (null_clear)
            null_data_valid <= 1'b0;
    end
    // 拍索引：start 归零 / 发送拍推进
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            null_cnt <= 2'd0;
        else if (null_flit_start_i)
            null_cnt <= 2'd0;
        else if (null_send_o)
            null_cnt <= null_cnt_next;
    end
    // chk 采样于 win 拍沿（T1/T3；DFX/一致性）
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            null_chk_q <= 16'h0;
        else if (null_chk_smp)
            null_chk_q <= crc_code;
    end

    // ── retry 路（MAS 码序 start→ready→toggle→行完成，同拍多赋值点 last-write-wins；
    //    start 拍 4 路全空闲（quiescent）：phase/parity/line_valid 恒 0，start 归零项为
    //    同值 no-op 不显式编码，优先级即 行完成清 > ready 置 / Flit 完成清 > start 置） ──
    // 重传进行中：Flit 完成清 / start 置
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            retrans_active_r <= 1'b0;
        else if (retry_flit_last)
            retrans_active_r <= 1'b0;
        else if (retrans_start_i)
            retrans_active_r <= 1'b1;
    end
    // 重传长度：start 拍采样
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            retrans_len_r <= 8'd0;
        else if (retrans_start_i)
            retrans_len_r <= retrans_len_i;
    end
    // 已消费 Flit 计数：每个 back 行完成递增 / start 归零（consis H-2：旧式 flit_cnt 只在自停谓词成立时 +1，len≥2 谓词恒假→永不自停；恢复 MAS §5.1 L407-416 口径）
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            retrans_flit_cnt_r <= 8'd0;
        else if (retry_row_done & retry_row_parity)
            retrans_flit_cnt_r <= retrans_flit_cnt_r + 8'd1;
        else if (retrans_start_i)
            retrans_flit_cnt_r <= 8'd0;
    end
    // chk 采样于 LO 发送拍沿（窗口当拍有效；DFX/一致性）
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            retry_chk_q <= 16'h0;
        else if (retry_chk_smp)
            retry_chk_q <= crc_code;
    end
    // LO/HI 相位：每发送拍翻转（quiescent phase=0）
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            retry_phase <= 1'b0;
        else if (retry_send)
            retry_phase <= ~retry_phase;
    end
    // 行 parity：每行完成（HI 发送拍沿）交替（front 0 → back 1；quiescent parity=0）
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            retry_row_parity <= 1'b0;
        else if (retry_row_done)
            retry_row_parity <= ~retry_row_parity;
    end
    // 行 pending：ready 沿置 / 行完成沿清（行完成 > ready，quiescent no-op 已述）
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            retry_line_valid <= 1'b0;
        else if (retry_row_done)
            retry_line_valid <= 1'b0;
        else if (retrans_line_ready_i)
            retry_line_valid <= 1'b1;
    end

    // ── udi 路（每拍无条件寄存 1T；无复位——vld 随数据，§5.3.1 豁免） ──
    always @(posedge mb_clk) begin
        udi_beat_reg <= udi_data_i;
    end
    always @(posedge mb_clk) begin
        udi_vld_reg <= udi_vld_i&~csr_protocol_mode_i ;
    end

    // ── replay push 写指针（T4 push_back 拍递增） ──
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            replay_wptr_cnt <= {REPLAY_ROW_IDX_W{1'b0}};
        else if (push_back)
            replay_wptr_cnt <= replay_wptr_cnt + 1'b1;
    end

    // ── 钟漂（drift_cnt 持续计数不归零；stall 装载 8 拍反压；装载与递减互斥） ──
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            drift_cnt <= 16'd0;
        else
            drift_cnt <= drift_cnt + 16'd1;
    end
    // stall 置位（drift_trig，要求 ~stall）与释放（backoff 归 1）互斥
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            clock_drift_stall <= 1'b0;
        else if (drift_trig)
            clock_drift_stall <= 1'b1;
        else if (clock_drift_stall & (drift_backoff_cnt == 4'd1))
            clock_drift_stall <= 1'b0;
    end
    // backoff 装载（drift_trig）与递减（stall 期间）互斥
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            drift_backoff_cnt <= 4'd0;
        else if (drift_trig)
            drift_backoff_cnt <= 4'd8;
        else if (clock_drift_stall & (drift_backoff_cnt != 4'd0))
            drift_backoff_cnt <= drift_backoff_cnt - 4'd1;
    end

endmodule
