// =============================================================================
// module name：replay_buffer
// author: OpenHappy Team
// MAS：spec/link_layer/submods/link_tx/mas/replay_buffer_mas.md
// 功能概述: TX 侧协议层重传缓冲区。按 seq 索引存取原始 PAIF 数据（Header +
//           Payload，DLP 未更新、CRC 未计算），供重传时读出并送父模块出口
//           路径共享 dlp_update + CRC 计算。
//           存储形态（编译期 REPLAY_BUF_DEPTH 选择）：
//             <16 深>：8 片 32×128bit SRAM 并联（sram_1rw_128b #(.DEPTH(32))），
//               每层 1024bit=128B；32 层×1024bit = 16 flit×2048bit = 8×32×128
//               = 32 Kbit。每个 flit 存 2 层（同拍双写：link_tx 侧 push_front /
//               push_back 两次 push（同 wptr），replay_buffer 以 d16_wr_last 判
//               front/back 层，首写 {k,0}、次写 {k,1}）；
//               重传读出连续给 cpif 发送（背靠背层读），读延迟 3 拍 = 2 拍背靠背
//               层读 + 1 拍组装寄存成 2048bit 整字。
//             <32/64/128 深>：16 个 128 bit × N SRAM 并联拼成 2048 bit 数据通路，
//               1 cycle 整字读写，读延迟 2 cycle。
//           三套指针：wr_ptr / rd_ptr 外部输入，retire_ptr 内部 ACK 驱动；
//           credit 机制复用 buffer 指针（credit 容量 = REPLAY_BUF_DEPTH）。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   <日期>  <修订人>  <变更概述>
//  2026-08-06  OpenHappy Team  初始版本（对齐 MAS v1.4）
//  2026-08-08  OpenHappy Team  对齐 MAS v1.0：读出路径改为 2 cycle——ren_d 打拍对齐
//                          SRAM 1 cycle 读延迟，flit_out_reg 输出寄存 1T，
//                          下游（dlp_update 数据源 mux）只使用寄存后数据；
//                          flit_out_valid_o = ren_d 再打 1 拍（ren_i 后 2 cycle）
//  2026-08-08  OpenHappy Team  1WR 单口仲裁显式化：read_fire 增加 ~sram_write
//                          （同拍写优先、读让路，父模块 ren_i 保持后下一拍重发）
//  2026-08-12  OpenHappy Team  SRAM 行为模型替换为厂商 SRAM 硬宏直接例化（hardip/sram，
//                          规格 X∈{32,64,128} 由 REPLAY_BUF_DEPTH_*
//                          presence 宏 `ifdef 选择）；test 输入 tie 0、输出悬空。
//  2026-09-18  OpenHappy Team  对齐 MAS v1.2：16 深分支存储由 DFF 寄存器阵列改为
//                          8 片 32×128 SRAM（每层 1024bit=128B，每 flit 存 2 层）
//  2026-10-26  OpenHappy Team   去厂商依赖：厂商 SRAM 硬宏替换为手写 RTL 行为模型 sram_1rw_128b
//                          （hardip/sram/sram_1rw_128b.v，参数化 DEPTH/ADDR_W）。
//                          45 端口 → 6 端口（DFT/BIST/repair 引脚原即 tie 0，
//                          行为零变化）；4 处 ifdef 分支统一按 DEPTH 参数区分。
//                          ——front 层收齐 128B push、back 层收齐再 push（2 次/flit）；
//                          重传读背靠背连续送 cpif，flit_out/valid 延迟 3 拍
//                          （2 拍背靠背层读 + 1 拍组装寄存）。32/64/128 深分支
//                          与 retire/occupancy/full/ack 逻辑不变。
//  2026-09-18  OpenHappy Team  方案 B v1.1（同天追加）：16 深写机由"front/back 半字捕获+
//                          wr_pend 次拍写"简化为"同拍双写"——d16_wr_last（复位 4'hF）+
//                          d16_is_front 判层，删 d16_front_half/d16_back_half/
//                          d16_wr_pend/d16_wr_base；write_fire 在 D16 分支加 d16_is_front
//                          门控（ifdef/ifndef 各 1 驱动）。读路径与 32/64/128 分支逐字不变。
//  2026-09-23  OpenHappy Team  512 形态 reg2out 重构（LL-20260923-TX512REGOUT，MAS v1.8）：重传读改行流式自走——
//                          16 深：父仅给起始事件（retrans_start/rptr_start/len）+ child 泵 tick（retrans_tick）
//                          pin，buffer 自走 T0{K,0}/T2{K,1}/T4{K+1,0}/T6{K+1,1}，line_q（d16_front_q 改名
//                          复用）稳定 2 拍，统一行流出口 retrans_line_data_o/ready；D9 删 16 深 flit_out 2048
//                          字组装、legacy 字出口 tie 0；1WR 写优先仲裁 / d16_rd_pend 健壮性机制不变（arch M2）；
//                          32/64/128 深 512 形态字读自发起（ren 每 flit 1 次自发起）。1024/兼容形态与写侧 /
//                          credit / retire / full / ack / d16 写机（同拍双写）全部不变。
// =============================================================================

`timescale 1ns/1ps

`include "link_layer_defines.vh"

module replay_buffer (
    // 时钟与复位
    input  wire                 mb_clk,           // 2 GHz 数据通路主时钟
    input  wire                 mb_rst_n,         // 同步复位，低有效
    input  wire                 sram_dyn_en_i,    // replay SRAM 动态门控使能（顶层 clk_cfg_sram_dyn_en 分发；1=动态 / 0=恒开）

    // CSR 配置（经父模块 link_tx 送入）
    input  wire                 csr_retry_en_i,   // CTRL_REG.RETRY_EN
    input  wire                 csr_credit_en_i,  // CTRL_REG.CREDIT_EN
    input  wire                 csr_perf_mode_i,  // MODE_REG.WORK_MODE：1=性能 / 0=兼容

    // 写接口（flit_sched → replay_buffer）
    input  wire                 wen_i,            // 写使能，valid-only
    input  wire [`REPLAY_BUF_PTR-1:0]         wptr_i,           // 外部 wr_ptr，N bit 含 1 bit flag
    input  wire [2047:0]        flit_in_i,        // 原始 PAIF 数据（无 DLP 更新无 CRC）

    // 读接口（retry_ctrl → replay_buffer）
    input  wire                 ren_i,            // 重传读使能，valid-only
    input  wire [`REPLAY_BUF_PTR-1:0]         rptr_i,           // 外部 rd_ptr，N bit 含 1 bit flag

    // ACK 接口（retry_ctrl → replay_buffer）
    input  wire                 ack_valid_i,      // ACK 脉冲，valid-only
    input  wire [`REPLAY_BUF_PTR-1:0]         ack_ptr_i,        // ACK 携带 seq，N bit 含 1 bit flag；性能模式有效

    // 行流读接口（512 形态 reg2out 重传读，MAS v1.8 §2.1 L83-88；1024/兼容形态父侧不接）
    input  wire                 retrans_start_i,      // 重传起始事件（单脉冲：泵空闲 + Flit 边界拍；buffer 自此自走窗口 D1）
    input  wire [`REPLAY_BUF_PTR-1:0] retrans_rptr_start_i, // 重传窗口头（8 bit seq 截取低 N bit；pop 序列初值）
    input  wire                 retrans_tick_i,       // child 单元泵 LO advance 拍（每 tick 一行读；经父接线）
    output wire [1023:0]        retrans_line_data_o,  // 行流数据（16 深 line_q 稳定 2 拍 / 32+ 字选行）
    output wire                 retrans_line_ready_o, // 行流就绪（16 深 line_active&line_q_valid / 32+ 字稳）

    // 输出
    // legacy 字口（仅 1024/兼容形态；512 形态重传读走行流 retrans_line_*，无 legacy 字口——v1.13 形态条件化）
`ifndef PAIF_INTF_512
    output wire [2047:0]        flit_out_o,       // 读出的原始 PAIF 数据
    output wire                 flit_out_valid_o, // 读出有效：32/64/128 深 ren_i 后 2 cycle；16 深分支 3 拍
`endif
    output wire                 full_o            // 缓冲区满 = credit 耗尽
);

    // =========================================================================
    // 参数定义（MAS §2.1 / §2.2）
    // =========================================================================
    // N = clog2(REPLAY_BUF_DEPTH) + 1（含 1 bit flag 位）= 宏 REPLAY_BUF_PTR
    //   DEPTH=16  → N=5 bit（bit[4]=flag，bit[3:0]=flit 序号 k；k 低 4 bit 索引层组）
    //   DEPTH=32  → N=6 bit（bit[5]=flag，bit[4:0]=地址）
    //   DEPTH=64  → N=7 bit（bit[6]=flag，bit[5:0]=地址）
    //   DEPTH=128 → N=8 bit（bit[7]=flag，bit[6:0]=地址）
    // 禁用 256——8 bit seq 无法提供 9 bit 指针宽度
    // 16 深分支物理层地址铺平（5bit，对应 8 片 32 深 × 128bit SRAM 的 32 层空间）：
    //   ADR[4:0] = {flit[3:0], fb[0]}，flit k → 层 {2k=front, 2k+1=back}；
    //   flit[4]/flag 位不参与寻址——仅决定层组归属并随 5bit ADR 自然环绕。

    // =========================================================================
    // 内部信号声明（MAS §5.1 / §6.1）
    // 所有 wire / reg 统一定义在 RTL 最前面，wire 与 assign 分离
    // =========================================================================

    // 寄存器（三套指针 + 判满 + pop 输出，32/64/128 与 16 深共用）
    reg  [`REPLAY_BUF_PTR-1:0]    retire_ptr;         // ACK 驱动的释放指针（含 flag 位）
    reg  [`REPLAY_BUF_PTR-1:0]    occupancy;          // = (wptr_i - retire_ptr) mod 2^N = 已用 credit
    reg             full;               // 判满结果
    reg  [2047:0]   flit_out_reg;       // pop 输出寄存器：读出的 2048 bit 整字（32/64/128 深 2 cycle；16 深 3 拍组装）
`ifndef PAIF_INTF_512
    reg             flit_out_valid_reg; // 读出有效寄存器（单脉冲；仅非 512 形态，v1.13 形态条件化）
`endif

    // ---- 内部 ICG 动态时钟门控（sram_en 组合；sram_hold1/2/3 未门控 mb_clk 域自保持）----
    wire            sram_en;            // SRAM 门控使能：~sram_dyn_en_i|wr_fire|rd_fire|sram_hold_2_3T
    wire            ck_sram;            // 内部 ICG 输出 SRAM 门控时钟（CK=mb_clk，相位对齐）
    reg             sram_hold1;         // 读写请求后自保持 1T（覆盖 SRAM 读延迟）
    reg             sram_hold2;         //           自保持 2T（32/64/128 深 2 cycle）
    reg             sram_hold3;         //           自保持 3T（16 深 3 拍）
    wire            sram_hold_2_3T;     // hold1|hold2|hold3
    wire            wr_fire;            // SRAM 写火线（32/64/128=sram_write / 16=d16_sram_we）
    wire            rd_fire;            // SRAM 读火线（32/64/128=read_fire / 16=d16_read_req）

`ifdef REPLAY_BUF_DEPTH_16
    // ---- 16 深分支（8×32×128 SRAM）内部信号（MAS §5.1 / blueprint u_sram16_x0..7）----
    reg  [3:0]      d16_wr_last;        // 本 flit 组最后写入层所属指针（复位 4'hF；sram_write 更新）
    wire            d16_is_front;       // (wptr_i[3:0] != d16_wr_last) → front 层写
`ifdef PAIF_INTF_512
    // ---- 行流式自走读（MAS v1.8 §5.1/§3.3；arch M2/M3/M4；D1：4 拍/flit 连续零 gap；
    //      D9：2048 字组装删除，重传出口 = 统一行流 retrans_line_data_o/ready）----
    reg             line_rd_d;          // 行读后 1 拍（d16_line_rd 打 1 拍——line_q 锁存拍，Q 有效拍末沿捕获，arch S1/M4b）
    reg             line_q_valid;       // 行窗有效（start 清 0、每次行读后 1 拍置 1——首拍 = start+2 = child 首 tick 拍）
    reg  [1023:0]   line_q;             // 行寄存器（d16_front_q 改名复用；buffer 内唯一 1024 行寄存器，稳定 2 拍；D9）
    wire            d16_line_rd;        // 行读发起（同拍组合，arch M2：禁用 edge-registered——会晚 1 拍致首 tick 读旧行）
    wire [4:0]      d16_line_addr;      // 行读层地址：start={K,0}（priming）/ tick={line_pop_seq,~line_parity}（old 值）
    wire [4:0]      d16_wr_adr;         // 写层地址：front={k,0} / back={k,1}（else 支占位 don't-care，arch R3）
`else
    reg             d16_rd_pend;        // 读启动挂起（read_fire 拍写侧占用，等接口释放）
    reg  [3:0]      d16_rd_base;        // 读 flit 序号 k（rptr_i[3:0]，重传期间外部稳定）
    reg  [1:0]      d16_rd_ph;          // 读机相位：00 idle / 01 back 层请求+采样 front / 10 组装输出
    reg  [1023:0]   d16_front_q;        // front 层读出采样（back 层请求拍 SRAM Q=front 数据）
    wire [1:0]      d16_rd_ph_next;     // 读机下一相位（预计算，满足时序块 if≤2）
    wire            d16_rd_advance;     // 读机相位推进（非 idle 或 启动拍，预计算 if≤2）
    wire [4:0]      d16_wr_adr;         // 写层地址：front={k,0} / back={k,1}
    wire [4:0]      d16_rd_adr;         // 读层地址：front={k,0} / back={k,1}
    wire            d16_rd_issue;       // 读侧想启动（read_fire | d16_rd_pend）
    wire            d16_rd_pend_next;   // 读启动挂起下一拍（预计算，满足时序块 if≤2）
    wire            d16_read_req;       // 读侧 SRAM 请求（front req | back req）
`endif
    wire            d16_sram_we;        // SRAM 写使能（= sram_write）
    wire            d16_sram_me;        // SRAM 使能（= d16_sram_we | d16_line_rd / d16_read_req）
    wire [4:0]      d16_sram_adr;       // SRAM 地址（写优先，arch M2：不变）
    wire [1023:0]   d16_sram_d;         // SRAM 写数据（= flit_in_i[1023:0]，同拍双写每拍直接写入）
    wire [1023:0]   d16_rdata;          // 8 片 SRAM Q 拼接（1 层 1024bit 读出）
    genvar          j;
`else
    // 32/64/128 深分支：ren_d 打拍 + SRAM rdata 拼接（MAS §5.1）
    reg             ren_d;              // 读使能打拍（对齐 SRAM 1 cycle 读延迟）
    wire [2047:0]   flit_sram_rdata;    // 16 个 SRAM rdata 拼接的 2048 bit 读数据（1 cycle 读延迟）
`endif

`ifdef PAIF_INTF_512
    // ---- 512 形态行流共享引擎寄存器（MAS v1.8 §5.1 L355-358 / arch M3；各深度共用，ck_sram 域）----
    reg  [3:0]      line_pop_seq;   // pop flit 序号（start 由 retrans_rptr_start_i[3:0] 初始化；back 行读 tick 边沿（old parity=0）+1；末行 rptr 可观测——统计）
    reg             line_parity;    // 行相位：0=front 行 {k,0} / 1=back 行 {k,1}（每 tick 翻转）
    reg  [7:0]      line_flit_cnt;  // 已消费 flit 数（back 行消费 tick 边沿（old parity=1）+1；仅统计——buffer 不自停）
    reg             line_active;    // 重传在途（retrans_start_i 置位；复位清；读活动随 tick 天然停——child 泵自停，arch M4）
    wire            d16_line_tick;  // tick 拍（retrans_tick_i & line_active，预计算）
    wire            d16_seq_inc;    // back 行读 tick 边沿（old parity=0，arch M3，预计算 if≤2）
    wire            d16_cnt_inc;    // back 行消费 tick 边沿（old parity=1，预计算 if≤2）
`endif

    // 使能组合（MAS §5.1，各深度共用）
    wire            credit_active;      // = csr_retry_en_i | csr_credit_en_i
    wire            write_fire;         // 指针推进（含 credit-only 不写 SRAM 场景）
    wire            sram_write;         // 仅 RETRY_EN=1 实际写 SRAM
    `ifdef PAIF_INTF_512
    `ifndef REPLAY_BUF_DEPTH_16
    wire            read_fire;          // 重传读（512 32/64/128 深：SRAM ME/ren_d 链；512 16 深行流自发起无此项）
    `endif
    `else
    wire            read_fire;          // 重传读仅在 RETRY_EN=1 时有意义
    `endif
    wire            ack_fire;           // ACK 推进条件

`ifndef REPLAY_BUF_DEPTH_16
    // SRAM 地址（32/64/128 深分支）
    wire [`REPLAY_BUF_PTR-2:0]    sram_addr;          // 写优先：sram_write ? wptr_i[`REPLAY_BUF_PTR-2:0] : rptr_i[`REPLAY_BUF_PTR-2:0]
    wire [7:0]              sram_addr_pad;      // 零扩展到 SRAM 宏 8 bit 地址口
`endif

`ifndef REPLAY_BUF_DEPTH_16
`ifdef PAIF_INTF_512
    // 32/64/128 深行流自走字读（MAS v1.8 L190；自发起/地址见组合段）
    wire            d16_self_rd_act; // 字读自发起（start | tick 且 old parity=1 = 每 front 行 tick 前 2 拍；并入 read_fire 驱动 SRAM ME/ren_d 链）
    wire [`REPLAY_BUF_PTR-2:0] self_rd_adr; // 字读地址（start=retrans_rptr_start_i / 其后=line_pop_seq 零扩展）
`endif
`endif

    // retire_ptr 下一拍候选
    wire [`REPLAY_BUF_PTR-1:0]    retire_ptr_inc;     // 兼容模式 +1（flag 位环绕翻转）
    wire [`REPLAY_BUF_PTR-1:0]    retire_ptr_next;    // 性能模式 = ack_ptr_i / 兼容模式 = retire_ptr_inc

    // occupancy 下一拍候选
    wire [`REPLAY_BUF_PTR-1:0]    occupancy_dec;      // 兼容模式纯 ACK：-1
    wire [`REPLAY_BUF_PTR-1:0]    occupancy_inc;      // 纯写：+1
    wire [`REPLAY_BUF_PTR-1:0]    occupancy_perf_next;// 性能模式直算 wptr_i - ack_ptr_i
    wire [`REPLAY_BUF_PTR-1:0]    occupancy_comp_next;// 兼容模式：同拍写+ACK 抵消，否则 -1

    // full 判定
    wire            full_match;         // 仅 flag 位不同

    // =========================================================================
    // 组合逻辑 assign（MAS §5.1，指针/判满各深度共用）
    // =========================================================================

    assign credit_active       = csr_retry_en_i | csr_credit_en_i;
`ifndef REPLAY_BUF_DEPTH_16
    assign write_fire          = wen_i & credit_active & ~full;
    assign wr_fire             = sram_write;
    assign rd_fire             = read_fire;
`endif
    assign sram_write          = wen_i & csr_retry_en_i & ~full;
    `ifdef PAIF_INTF_512
    `ifndef REPLAY_BUF_DEPTH_16
        assign read_fire           = ren_i & 
                                    | d16_self_rd_act 
`endif
    // 512 16 深（行流）：d16_line_rd 独立发起（D9），无 read_fire（2026-09-24 unused 清理）
    `else
        assign read_fire           = ren_i ;   // 1WR 单口：写优先，同拍有写时读让路（父模块 ren_i 保持，下一拍重发）
    `endif
    assign ack_fire            = ack_valid_i & credit_active;

`ifndef REPLAY_BUF_DEPTH_16
`ifdef PAIF_INTF_512
    // 32/64/128 深行流自走（MAS v1.8 L190：ren 每 flit 1 次自发起 = start + 每 front 行 tick 前 2 拍；字读保留）
    assign d16_self_rd_act     = retrans_start_i | (line_active & retrans_tick_i & line_parity); // 字读自发起（old parity=1 = front 行 tick 前 2 拍）
    assign self_rd_adr         = retrans_start_i ? retrans_rptr_start_i[`REPLAY_BUF_PTR-2:0]
                                                 : line_pop_seq; // flit 粒度 4bit seq（N-2≥4，隐式零扩展；32+ 深 >16-flit 窗口 wrap——MAS 伪码 4bit line_pop_seq 上限）
    assign sram_addr           = {`REPLAY_BUF_PTR-1{sram_write}} & wptr_i[`REPLAY_BUF_PTR-2:0]
                                | {`REPLAY_BUF_PTR-1{~sram_write}} & {`REPLAY_BUF_PTR-1{d16_self_rd_act}} & self_rd_adr
                                | {`REPLAY_BUF_PTR-1{~sram_write & ~d16_self_rd_act}} & rptr_i[`REPLAY_BUF_PTR-2:0]; // 写 > 自读 > legacy rptr（512 形态父 ren_i=0）
`else
    assign sram_addr           = {`REPLAY_BUF_PTR-1{sram_write}} & wptr_i[`REPLAY_BUF_PTR-2:0]
                                | {`REPLAY_BUF_PTR-1{~sram_write}} & rptr_i[`REPLAY_BUF_PTR-2:0];
`endif
    assign sram_addr_pad       = {{(9-`REPLAY_BUF_PTR){1'b0}}, sram_addr};
`endif

    assign retire_ptr_inc      = retire_ptr + 1'b1;
    assign retire_ptr_next     = {`REPLAY_BUF_PTR{csr_perf_mode_i}} & ack_ptr_i[`REPLAY_BUF_PTR-1:0]
                                | {`REPLAY_BUF_PTR{~csr_perf_mode_i}} & retire_ptr_inc;

    assign occupancy_dec       = occupancy - 1'b1;
    assign occupancy_inc       = occupancy + 1'b1;
    assign occupancy_perf_next = wptr_i[`REPLAY_BUF_PTR-1:0] - ack_ptr_i[`REPLAY_BUF_PTR-1:0];
    assign occupancy_comp_next = {`REPLAY_BUF_PTR{write_fire}} & occupancy
                                | {`REPLAY_BUF_PTR{~write_fire}} & occupancy_dec;

    assign full_match          = (wptr_i[`REPLAY_BUF_PTR-2:0] == retire_ptr[`REPLAY_BUF_PTR-2:0]) &&
                                 (wptr_i[`REPLAY_BUF_PTR-1]  != retire_ptr[`REPLAY_BUF_PTR-1]);
    assign full                = credit_active & full_match;
    assign full_o              = full;                 // 缓冲区满出口（判满结果直出；LL-20260923-TX512REGOUT 补漏——v1.8 重写时丢失的基线 L466 驱动）

`ifdef PAIF_INTF_512
    // 512 形态行流共享引擎 tick 边沿预计算（MAS v1.8 §5.1 伪码 L382-386 / arch M3；if≤2）
    assign d16_line_tick = retrans_tick_i & line_active;   // tick 拍（child 单元泵 LO advance）
    assign d16_seq_inc   = d16_line_tick & ~line_parity;   // back 行读 tick 边沿（old parity=0，arch M3）：K→K+1
    assign d16_cnt_inc   = d16_line_tick & line_parity;    // back 行消费 tick 边沿（old parity=1）：仅统计
`endif

`ifdef REPLAY_BUF_DEPTH_16
    // ---- 16 深分支组合（MAS §5.1 层写 / §3.3 层读时序）----
    // 写（同拍双写，1 次 wen = 1 层，2 次/flit，同 wptr）：link_tx 侧 push_front / push_back
    //   两次 push（same wptr k）先后到达。首写：wptr_i[3:0] != d16_wr_last → d16_is_front=1
    //   （复位 4'hF 保证 flit0 首写判 front）→ 写层 {k,0}=front，D=flit_in_i[1023:0]；
    //   次写：wptr_i[3:0] == d16_wr_last（首次 sram_write 已把 d16_wr_last 更新为 k）
    //   → d16_is_front=0 → 写层 {k,1}=back，D=flit_in_i[1023:0]。D16 分支 write_fire 按
    //   is_front 门控：credit/occupancy 每 flit 只计 1 次（front 写；ifndef 侧公共驱动关闭）。
    // 读（重传，3 拍）：d16_rd_start 拍发 front 层读请求（ADR={k,0}）→ 次拍发
    //   back 层读请求（ADR={k,1}）并采样 front 层读出（d16_front_q）→ 再 1 拍把
    //   {back 层读出, front_q} 组装成 2048bit 整字并打 valid（flit_out_valid_o 单脉冲，
    //   低 1024bit=front、高 1024bit=back，与写回 flit_in_i 字面一致）。ren_i 后 3 拍。
    // 1WR 仲裁：写优先（arch M2：不变）
    assign d16_is_front  = (wptr_i[3:0] != d16_wr_last);
    assign write_fire    = wen_i & credit_active & ~full & d16_is_front;
    assign wr_fire       = d16_sram_we;
`ifdef PAIF_INTF_512
    // ---- 行流式自走读（MAS v1.8 §5.1/§3.3；arch M2/M3/M4；D1：4 拍/flit 连续零 gap）----
    // 父仅给起始事件（retrans_start_i + retrans_rptr_start_i）；buffer 自走（无逐行父 ren/rptr；len 不消费——child 侧停止）。
    // 读节奏 pin 到 child 单元泵 LO advance 拍（retrans_tick_i）：start 时 priming 读 + 每 tick 一行读（每 2 拍 1 行读）。
    // 读发起 = 同拍组合（arch M2：edge-registered 会晚 1 拍致首 tick 读旧行、全行错位）：
    //   d16_line_rd = retrans_start_i | (line_active & retrans_tick_i)
    // 读序（addr 指向 2 拍后被消费的行，arch R2/M3）：T0 {K,0} / T2 {K,1} / T4 {K+1,0} / T6 {K+1,1}
    // 写占口 corner（arch M2）——d16_line_rd 拍若 sram_write 同拍则行读丢失；重传期父反压
    //   PAIF 入口无写（不变量），512 行流读发起不加 gate（非 512 字读机保留 d16_rd_pend 挂起）。
    assign rd_fire        = d16_line_rd;                     // ICG sram_en 读活动源（MAS L116 并入）
    assign d16_line_rd    = retrans_start_i | (line_active & retrans_tick_i);
    assign d16_line_addr  = {5{retrans_start_i}} & {retrans_rptr_start_i[3:0], 1'b0}    // start priming 读 = {K,0}（front 行）
                            | {5{~retrans_start_i}} & {line_pop_seq, ~line_parity};     // tick 读 = 下一行（old 值）
    assign d16_wr_adr     = {5{sram_write}} & {wptr_i[3:0], ~d16_is_front}
                           | {5{~sram_write}} & {d16_line_addr[3:0], 1'b1};            // else 支占位 don't-care（arch R3：读口统一为行流）；[3:0] 显式 5bit 位宽（lint W116/MatchWidthOnAssign：与现隐式截断位型逐位一致）
    assign d16_sram_we    = sram_write;
    assign d16_sram_me    = d16_sram_we | d16_line_rd;
    assign d16_sram_adr   = {5{sram_write}} & d16_wr_adr | {5{~sram_write}} & d16_line_addr; // 写优先（arch M2：不变）
    assign d16_sram_d     = flit_in_i[1023:0];
`else
    assign rd_fire        = d16_read_req;
    assign d16_wr_adr     = {5{sram_write}} & {wptr_i[3:0], ~d16_is_front} | {5{~sram_write}} & {d16_rd_base, 1'b1};
    assign d16_rd_issue   = read_fire | d16_rd_pend;
    assign d16_rd_start   = d16_rd_issue & ~sram_write;
    assign d16_rd_adr     = {5{(d16_rd_ph == 2'b01)}} & {d16_rd_base, 1'b1} | {5{~(d16_rd_ph == 2'b01)}} & {rptr_i[3:0], 1'b0};
    assign d16_read_req   = d16_rd_start | (d16_rd_ph == 2'b01);
    assign d16_sram_we    = sram_write;
    assign d16_sram_me    = d16_sram_we | d16_read_req;
    assign d16_sram_adr   = {5{sram_write}} & d16_wr_adr | {5{~sram_write}} & d16_rd_adr;
    assign d16_sram_d     = flit_in_i[1023:0];
    assign d16_rd_ph_next = {2{(d16_rd_ph == 2'b01)}} & 2'b10
                           | {2{(d16_rd_ph == 2'b10)}} & 2'b00
                           | {2{~(d16_rd_ph == 2'b01) & ~(d16_rd_ph == 2'b10)}} & 2'b01;
    // 读机相位推进：非 idle 或 启动拍（if≤2 预计算）
    assign d16_rd_advance  = (d16_rd_ph != 2'b00) | d16_rd_start;
    // 读启动挂起下一拍：read_fire 拍写侧占用 → 置 1；实际启动拍 → 清 0（if≤2 预计算）
    assign d16_rd_pend_next = (read_fire & ~d16_rd_start)
                             | (~(read_fire & ~d16_rd_start) & ~(d16_rd_start) & d16_rd_pend);
`endif
`endif

    // =========================================================================
    // 内部 ICG（u_icg_sram）：SRAM 门控独立于 TX/RX 路；sram_hold1/2/3 DFF
    //   打未门控 mb_clk 域（读写请求后自保持 2~3 拍，覆盖 SRAM 读延迟；
    //   32/64/128 深 2 cycle、16 深 3 拍）。sram_dyn_en_i 复位 0 → sram_en 恒 1
    //   → ck_sram=mb_clk（bit-exact 零回归）。
    // =========================================================================
    assign sram_en          = ~sram_dyn_en_i | wr_fire | rd_fire | sram_hold_2_3T;
    assign sram_hold_2_3T   = sram_hold1 | sram_hold2 | sram_hold3;

    sync_icg_pos u_icg_sram (
        .RSTN (mb_rst_n),
        .CK   (mb_clk),
        .E    (sram_en),
        .Q    (ck_sram)
    );

    // 读写请求后自保持 2~3 拍（未门控 mb_clk 域，门控关断前仍运转）
    always @(posedge mb_clk) begin
        if (!mb_rst_n) begin
            sram_hold1 <= 1'b0;
            sram_hold2 <= 1'b0;
            sram_hold3 <= 1'b0;
        end else begin
            sram_hold1 <= wr_fire | rd_fire;
            sram_hold2 <= sram_hold1;
            sram_hold3 <= sram_hold2;
        end
    end

    // =========================================================================
    // 存储例化（MAS §5.1 / §6.1）
    //  <16 深>：8 片 32 深 × 128bit SRAM 并联（sram_1rw_128b #(.DEPTH(32), .ADDR_W(5))），
    //           共享 CLK / ME / WE / ADR[4:0]；wdata / rdata 按 128bit 段分发到各 SRAM。
    //           每层 1024bit = 128B；push_front / push_back 两次写同 wptr 分写 front/back 层。
    //  <32/64/128 深>：16 个 128 bit × N SRAM 并联，共享 ADR / ME / WE，
    //           物理深度 N 由 REPLAY_BUF_DEPTH 对应规格决定，`ifdef 在编译期选 DEPTH 参数。
    // =========================================================================
`ifdef REPLAY_BUF_DEPTH_16
    generate
        for (j = 0; j < 8; j = j + 1) begin : u_sram16_x
            sram_1rw_128b #(.DEPTH(32), .ADDR_W(5)) u_sram (
                .clk     (ck_sram),
                .req     (d16_sram_me),
                .we      (d16_sram_we),
                .addr    (d16_sram_adr),
                .wdata   (d16_sram_d[j*128 +: 128]),
                .rdata   (d16_rdata[j*128 +: 128])
            );
        end
    endgenerate
`else
    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : sram_x
`ifdef REPLAY_BUF_DEPTH_32
            sram_1rw_128b #(.DEPTH(32), .ADDR_W(5)) u_sram (
                .clk     (ck_sram),
                .req     (sram_write | read_fire),
                .we      (sram_write),
                .addr    (sram_addr_pad[4:0]),
                .wdata   (flit_in_i[i*128 +: 128]),
                .rdata   (flit_sram_rdata[i*128 +: 128])
            );
`elsif REPLAY_BUF_DEPTH_128
            sram_1rw_128b #(.DEPTH(128), .ADDR_W(7)) u_sram (
                .clk     (ck_sram),
                .req     (sram_write | read_fire),
                .we      (sram_write),
                .addr    (sram_addr_pad[6:0]),
                .wdata   (flit_in_i[i*128 +: 128]),
                .rdata   (flit_sram_rdata[i*128 +: 128])
            );
`else
            // 默认 64 深
            sram_1rw_128b #(.DEPTH(64), .ADDR_W(6)) u_sram (
                .clk     (ck_sram),
                .req     (sram_write | read_fire),
                .we      (sram_write),
                .addr    (sram_addr_pad[5:0]),
                .wdata   (flit_in_i[i*128 +: 128]),
                .rdata   (flit_sram_rdata[i*128 +: 128])
            );
`endif
        end
    endgenerate
`endif

    // =========================================================================
    // 时序逻辑（同步复位，always @(posedge mb_clk)，禁用异步复位）
    // =========================================================================

    // retire_ptr 更新：仅 ack_fire 时推进（兼容 +1 / 性能直置，各深度共用）
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            retire_ptr <= {`REPLAY_BUF_PTR{1'b0}};
        else if (ack_fire)
            retire_ptr <= retire_ptr_next;
    end

    // occupancy 更新（各深度共用）：优先级 性能模式 ACK > 兼容模式 ACK > 写推进
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            occupancy <= {`REPLAY_BUF_PTR{1'b0}};
        else if (ack_fire)
            occupancy <= {`REPLAY_BUF_PTR{csr_perf_mode_i}} & occupancy_perf_next
                        | {`REPLAY_BUF_PTR{~csr_perf_mode_i}} & occupancy_comp_next;
        else if (write_fire)
            occupancy <= occupancy_inc;
    end

`ifdef PAIF_INTF_512
    // ---- 512 形态行流共享引擎（MAS v1.8 §5.1 伪码 L369-386；arch M3；各深度共用，ck_sram 域）----
    // tick 拍行相位翻转 + 序号/统计分离（if≤2：内层 if 预计算为 & 归并 wire d16_seq_inc/d16_cnt_inc）；
    // start 与 tick 互斥（父仅泵空闲 + Flit 边界拍发 start，start 拍 line_active 旧值=0 → tick 支不触发，与 else-if 同果）
    always @(posedge ck_sram) begin
        if (!mb_rst_n) begin
            line_pop_seq  <= 4'd0;
            line_parity   <= 1'b0;
            line_flit_cnt <= 8'd0;
            line_active   <= 1'b0;
        end else if (retrans_start_i) begin
            line_pop_seq  <= retrans_rptr_start_i[3:0];       // 窗口头 K
            line_parity   <= 1'b0;                            // 首行 = front 行
            line_flit_cnt <= 8'd0;
            line_active   <= 1'b1;
        end else if (d16_line_tick) begin
            line_parity   <= ~line_parity;
            line_pop_seq  <= d16_seq_inc ? line_pop_seq + 4'd1 : line_pop_seq;
            line_flit_cnt <= d16_cnt_inc ? line_flit_cnt + 8'd1 : line_flit_cnt;
        end
    end
`endif

`ifndef REPLAY_BUF_DEPTH_16
    // ---- 32/64/128 深：读出通路（ren_i 后 2 cycle，MAS §5.1）----
    // 读使能打拍：对齐 SRAM 1 cycle 读延迟（ck_sram 域）
    always @(posedge ck_sram) begin
        if (!mb_rst_n)
            ren_d <= 1'b0;
        else
            ren_d <= read_fire;
    end

    // SRAM 读出数据寄存 1T：ren_d=1 时 SRAM rdata 有效，当拍采样入 flit_out_reg（ck_sram 域）
    always @(posedge ck_sram) begin
        if (!mb_rst_n)
            flit_out_reg <= 2048'd0;
        else if (ren_d)
            flit_out_reg <= flit_sram_rdata;
    end

`ifndef PAIF_INTF_512
    // flit_out_valid 输出寄存器：ren_i 后 2 cycle（1 cycle SRAM 读 + 1 cycle 寄存器输出；ck_sram 域）
    always @(posedge ck_sram) begin
        if (!mb_rst_n)
            flit_out_valid_reg <= 1'b0;
        else
            flit_out_valid_reg <= ren_d;
    end
`endif
`else
    // ---- 16 深分支：层写机（MAS §5.1，同拍双写：1 次 wen = 1 层）----
    // 写层组最后指针捕获（sram_write 拍）——同拍双写判层寄存器化（复位 4'hF：
    //   保证 flit0 首写判为 front 层 {0,0}；同 k 次写 → back 层 {k,1}；ck_sram 域）
    always @(posedge ck_sram) begin
        if (!mb_rst_n)
            d16_wr_last <= 4'hF;          // ★ 复位 4'hF（首写必判 front 层）
        else if (sram_write)
            d16_wr_last <= wptr_i[3:0];
    end

`ifdef PAIF_INTF_512
    // ---- 行流自走读时序（MAS v1.8 §5.1 伪码 L359-361/L387-389；arch S1/M4b；ck_sram 域）----
    // 行读后 1 拍（d16_line_rd 打 1 拍——Q 有效拍末沿捕获，line_q 锁存拍；arch S1/M4b）
    always @(posedge ck_sram) begin
        if (!mb_rst_n)
            line_rd_d <= 1'b0;
        else
            line_rd_d <= d16_line_rd;
    end

    // 行寄存器锁存：line_rd_d 拍采样 SRAM Q（行窗 = [tick, tick+1] 覆盖 LO+HI 2 拍消费）
    always @(posedge ck_sram) begin
        if (!mb_rst_n)
            line_q <= 1024'd0;
        else if (line_rd_d)
            line_q <= d16_rdata;
    end

    // 行窗有效：start 清 0、每次行读后 1 拍置 1（首拍 = start+2 = child 首 tick 拍；重传期连续；arch M4c）
    always @(posedge ck_sram) begin
        if (!mb_rst_n)
            line_q_valid <= 1'b0;
        else if (retrans_start_i)
            line_q_valid <= 1'b0;
        else if (line_rd_d)
            line_q_valid <= 1'b1;
    end
`else
    // ---- 16 深分支：层读机（ren_i 后 3 拍，MAS §3.1 / §3.3）----
    // 读启动挂起：read_fire 拍若写侧占用（当拍写），等接口释放后启动
    //   （d16_rd_pend_next 组合预计算，时序块仅 2 分支满足 if≤2；ck_sram 域）
    always @(posedge ck_sram) begin
        if (!mb_rst_n)
            d16_rd_pend <= 1'b0;
        else
            d16_rd_pend <= d16_rd_pend_next;
    end

    // 读序号捕获（实际启动拍，重传期间 rptr_i 稳定；ck_sram 域）
    always @(posedge ck_sram) begin
        if (!mb_rst_n)
            d16_rd_base <= 4'd0;
        else if (d16_rd_start)
            d16_rd_base <= rptr_i[3:0];
    end

    // 读机相位：00 idle →(启动) 01 back 层请求 → 10 组装输出 → 00
    //   （d16_rd_ph_next / d16_rd_advance 组合预计算，满足时序块 if≤2；ck_sram 域）
    always @(posedge ck_sram) begin
        if (!mb_rst_n)
            d16_rd_ph <= 2'b00;
        else if (d16_rd_advance)
            d16_rd_ph <= d16_rd_ph_next;
    end

    // front 层读出采样（back 层请求拍 SRAM Q=front 层数据；ck_sram 域）
    always @(posedge ck_sram) begin
        if (!mb_rst_n)
            d16_front_q <= 1024'd0;
        else if (d16_rd_ph == 2'b01)
            d16_front_q <= d16_rdata;
    end

    // 组装整字：{back 层读出[1023:0], front_q[1023:0]}，与写入字面一致；ck_sram 域
    always @(posedge ck_sram) begin
        if (!mb_rst_n)
            flit_out_reg <= 2048'd0;
        else if (d16_rd_ph == 2'b10)
            flit_out_reg <= {d16_rdata[1023:0], d16_front_q[1023:0]};
    end

    // 读出有效：组装拍单脉冲（ren_i 后 3 拍；ck_sram 域）
    always @(posedge ck_sram) begin
        if (!mb_rst_n)
            flit_out_valid_reg <= 1'b0;
        else if (d16_rd_ph == 2'b10)
            flit_out_valid_reg <= 1'b1;
        else
            flit_out_valid_reg <= 1'b0;
    end
`endif
`endif

    // =========================================================================
    // 输出赋值
    // =========================================================================
`ifdef PAIF_INTF_512
    // 统一行流出口（MAS v1.8 L87-88/L190；16 深 line_q / 32+ 字选行）
`ifdef REPLAY_BUF_DEPTH_16
    assign retrans_line_data_o  = line_q;                         // line_q 稳定 2 拍（stall 多拍保持；MAS L396）
    assign retrans_line_ready_o = line_active & line_q_valid;     // IDLE（line_active=0）=0 / 首拍=start+2 / 重传期连续（arch M4c）
`else
    assign retrans_line_data_o  = line_parity ? flit_out_reg[2047:1024] : flit_out_reg[1023:0]; // tick 计数 parity 选行（MAS L190/L399）
    // 字稳（MAS L399）：ren 每 flit 1 次自发起 + 2 拍读延迟 ⇒ 字在整个 tick 窗口 [T2..T5]/[T6..T9]… 恒稳；
    // legacy 字口 valid 为单脉冲（非 512 形态 flit_out_valid_reg），不能作行流 ready（back 行 tick 会死锁）→ 字稳等价 = line_active
    assign retrans_line_ready_o = line_active;
`endif
`endif
`ifndef PAIF_INTF_512
    // legacy 字口输出（仅非 512 形态；512 形态重传读走上方行流 retrans_line_*，无 legacy 字口——v1.13
    // 形态条件化 D9 闭环：512 build flit_out_o/flit_out_valid_o 端口不存在）
    assign flit_out_o       = flit_out_reg;
    assign flit_out_valid_o = flit_out_valid_reg;
`endif

endmodule
