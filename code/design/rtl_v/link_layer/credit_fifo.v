// =============================================================================
// module name：credit_fifo
// author: OpenHappy Team
// golden：spec/link_layer/link_layer_spec.md §4.3.3.4 / §4.4.3（权威）
//         （MAS credit_fifo_mas.md 与该 golden 冲突处以 golden 为准）
// 功能概述: RX 侧 CRC 校验通过 Flit 的缓冲 FIFO。由四片 credit_fifo_bank
//           （u_bank0..3，512 bit × CREDIT_FIFO_DEPTH 寄存器阵列）拼接成
//           4×512 bank 寄存器 FIFO（替代原 SRAM 硬宏）。v1.6（LL-20260924-
//           CRIFVO）本模块收敛为 PAIF512 ×1 lane 专用（M1）：删旧 2×1024 双
//           bank（rd_bank/occ_lo/occ_hi/lo_rd/hi_rd）拼读，PAIF512 单管每拍
//           读序列单 bank 512 直出。拼一个 256 B Flit 的 4 个 512-beat：
//           front（低 1024，unit0）拆 b0+b1、back（高 1024，unit1）拆 b2+b3。
//           性能模式：front→b0/b1、back→b2/b3 均按 Flit seq 索引（front 直接
//           取 flit_in[PTR_W+3:4]，back 复用 front 写时锁存的 cur_seq）；
//           兼容模式（Flit<512 bit，单单元）只写/读 b0/b1，地址为递增计数
//           读 = ren 逐拍选通 + 行寄存器 row_cur 当拍直出（LL-TIMING-N1 FWFT 读重构：
//           bank 读 mux 改址 rd_ptr_next、rd_ptr_adv 拍 row_cur 锁下读序行；PAIF512 每拍
//           1 bank → 512 直出、到 paif 接口无新增 pipe、协议拍序/链路总延迟零变化）；
//           pop_valid_o = v1.6.4 用户终裁：四 bank empty NOR4 任一 bank 非空
//           （~empty_b0 | ~empty_b1 | ~empty_b2 | ~empty_b3，不分
//           mode/接口；is_self 只控 ren/flit_out 512 片选/rd_seq_done，
//           不控 valid；v1.6.3「compat 仅 st=0 数据相位 / pad 相位
//           pv=0 静默 pop」语义作废；M4 不含 pop_i，valid 独立于 ready 断组合环；
//           两形态分述：perf=b0..b3 任一非空 / compat=b0/b1 任一非空
//           （b2/b3 恒空），MAS v1.6.12 §1.1）。
//           每 bank cnt_bX(CNT_W)+empty：cnt_nxt = cnt + 写 - ren，满判
//           cnt==CF_DEPTH 可靠无回绕（M2）；empty = cnt_nxt==0 打拍。入口无反压
//           （无 push_ready 口；满 + 新 push → 丢弃该单元并报 overflow_o）：
//           出口由 paif_rx_ready（link_rx pop）反压，pop 反压冻结读序列零丢失。
//           flit_end 不存入 bank，由读出阶段+模式派生。1T 不变不变量：T0 push
//           → T1 pop；链路总延迟/端口/协议拍序零变化。replay bank mem 无复位
//           （去清零，v1.5，X 泄漏由 ren 仅对 empty=0 bank 拉高保证）。
//           注意：性能模式写/读均按序推进（交付按序），本实现未实现 seq 空洞
//           跳过（CRC 失败未存 front 的边界），覆盖验证的 PAIF 注入 happy path。
// 编码约束（本模块强制):
//   1. 仅时序信号使用 always 语句（always @(posedge mb_clk)，同步复位，
//      禁用异步复位）；
//   2. 每个 always 块内 if 判断（不含 reset 判断）≤ 2 个，禁止 if 嵌套，
//      条件用 && 归并或预计算 wire；
//   3. 组合逻辑只允许 assign 赋值；
//   4. for 循环仅用于 generate 实例化。
//   5. 注: VCS 2023.12 不解析 "复制因子直接作多元素 concat 首元素" 形状
//      ({N{1'b0}, x} 报 SE token ',') —— 零扩一律用嵌套形状 {{N{1'b0}}, x}。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   <日期>  <修订人>  <变更概述>
//  2026-08-08  OpenHappy Team  初始版本（对齐 MAS v1.0）
//  2026-08-08  OpenHappy Team  SRAM 口径修正：改用 1R1W 双口宏 saculs0c4l2p*
//  2026-08-08  OpenHappy Team  entry 改 128 B 单元粒度（FLIT_W=1024 固定）+ flit_end
//  2026-09-04  OpenHappy Team  对齐 golden §4.3.3.4/§4.4.3 重构：单 Flit 条目 2048 bit
//                          /16 SRAM，深度语义即 64 flit；修复逻辑深度 32 flit bug
//  2026-09-07  OpenHappy Team  SRAM 硬宏替换为双 bank 寄存器 FIFO（credit_fifo_bank
//                          bank_lo/bank_hi）；去 SRAM 依赖；FWFT ≈0 cycle pop；
//                          去除 csr_credit_en_i/bypass（RX credit_en 只控 ACK 生
//                          成，上提到 link_rx dlv_ack）；新增 csr_perf_mode_i
//                          路由 front/back 与选指针模型；入口无反压。
//  2026-09-09  OpenHappy Team  D23：双 1024 bank 改 4×512 并行 bank（b0+b1=front、
//                          b2+b3=back），存储 512-beat 粒度；接口/控制/占用/
//                          overflow/flit_end/seq 语义不变。
//  2026-09-24  OpenHappy Team  LL-20260924-CRIFVO v1.6：读通路重构（按 arch_prof
//                          M1~M4 回环收敛）：① 收敛 PAIF512 ×1 lane 专用，删全部
//                          PAIF1024 双 bank 并读/拼（rd_bank/occ_lo/occ_hi/lo_rd/
//                          hi_rd 删除，PAIF1024 拼接归形态②内部）；② 每 bank
//                          cnt_bX(CNT_W) 位宽参数化 CNT_W=clog2(CF_DEPTH)+1
//                          （16/32/64/128→5/6/7/8），满判 cnt==CF_DEPTH 可靠；
//                         ③ 兼容读序列仅 b0→b1 两相位（M3，b2/b3 is_self 门控
//                          csr_perf_mode_i），rd_seq_done(perf=b3/compat=b1)+
//                          rd_seq_next 回绕 0；④ pop_valid_o=true 四 bank empty
//                          regout OR 取反（去 & pop_i，断组合环，握手由消费侧
//                          paif_rx_ready & pop_valid 承担）；bank mem 去复位清零；
//                          读序列逐 bank（PAIF512 单 bank 512 直出）。端口/协议
//                          拍序/链路总延迟零变化。
//  2026-09-25  OpenHappy Team  v1.6.3（CRIFVO C1）：pop_valid_o 公式 A——perf =
//                          active bank（is_self）非空 / compat = 仅 st=0 数据
//                          相位（is_self_b0 & ~empty_b0，pad 相位 st=1 pv=0 由
//                          消费侧 cf_pop 静默消费——修 CASE14 compat bridge 死
//                          锁）；bridge 零改（link_rx_compat 不动）。
//  2026-09-28  OpenHappy Team  LL-LINTBASE-04（round1 lint 基线）：occupancy_o 四 bank
//                          求和改先零扩至 CNT_W+3（occ_sum）再取低 8b 出——修满
//                          配置 4×16=64（7b）档旧直加 CNT_W 截断的 DFx 回绕
//                          （16 档零功能影响：cf_occupancy_i 512 形态 DFx-only
//                          不入 valid 锥、1024 只判 !=0）；cnt_bN_nxt ±1 操作数
//                          零扩至 CNT_W（W116 宽度匹配，功能等价）。
//  2026-09-29  OpenHappy Team  LL-TIMING-N1（DC 时序 N=1，N=0 基线 job 400776）：①
//                          reg2out 16→≤6——读路径 FWFT 重构：bank 读 mux 改址
//                          rd_ptr_next（bX_rd=mem[rd_ptr_next]），新增 row_cur_b0..3
//                          行寄存器（无复位，类 bank mem 存储豁免）于 rd_ptr_adv 拍
//                          锁存下读序行、写命中读行拍（空 FIFO 0→1 首 push，读写同
//                          行）补 flit_in 半——读序首拍 row_cur 恒=mem[rd_ptr]，
//                          T0 push→T1 pop/valid/end 拍序零变化、不加 pipe（flit_out_o
//                          改 is_self 选 row_cur）；② in2reg 19→18 / CLKICG -1——写
//                          阻塞条件预并（hi_kind/lo_wr_block/hi_wr_block，push_i 仅
//                          单 AND 级，旧 raw&~full 两级）+ 写行 one-hot（lo/hi_row_sel，
//                          bank 逐行写 ICG 使能=wr_en&row_sel 单 AND，4bit 等值链移出
//                          bank 关键链）。功能位级等价（见各段注）。
//  2026-10-09  OpenHappy Team  LL-20261009-CFCOMPAT（BUG-002）：compat 读序死锁修复——
//                          L315 读序列推进去 &csr_perf_mode_i 门控（全模式统一，
//                          对齐 MAS v1.6.12 §5.1；perf 行为零变化）+ pop_valid v1.6.3
//                          公式 A 陈旧注释 3 处同步 v1.6.4 口径（L19-23/L117/L477-480，
//                          逻辑零改动）
// =============================================================================

`timescale 1ns/1ps

`include "link_layer_defines.vh"

module credit_fifo (
    // 时钟与复位
    input  wire                          mb_clk,         // 2 GHz 数据通路主时钟
    input  wire                          mb_rst_n,       // 同步复位，低有效

    // CSR 配置（apb_csr 解码 MODE_REG.WORK_MODE 具名直送）
    input  wire                          csr_perf_mode_i,// 1=性能(2048bit Flit) / 0=兼容(单单元)

    // 写接口（link_rx CRC 校验通过拍 → credit_fifo）
    input  wire                          push_i,         // 写入请求，valid（128 B 单元）
    input  wire [1023:0]                 flit_in_i,      // 待写入 128 B 单元（Flit 低/高半）
    input  wire                          flit_end_i,     // 单元为 Flit 末单元标记（不存入 bank）

    // 读接口（paif_rx_if → credit_fifo）
    input  wire                          pop_i,          // 读出请求；paif_rx_ready 时拉高
    output wire [1023:0]                 flit_out_o,     // 读出 128 B 单元（PAIF512 每拍 512 直出）
    output wire                          flit_end_o,     // 读出单元的 flit_end（由 bank+模式派生）
    output wire                          pop_valid_o,    // 读出有效（v1.6.4 用户终裁：NOR4 任一 bank 非空，不分 mode/接口；is_self 不控 valid）

    // 状态输出
    output wire                          overflow_o,     // 溢出脉冲（1 cycle）→ 中断聚合
    output wire [7:0]                    occupancy_o     // 当前 FIFO 占用单元数（DFx；空=0）
);

    // =========================================================================
    // 参数定义：寄存器 FIFO 深度 = credit FIFO 深度（由 CREDIT_FIFO_DEPTH_* 选）
    // =========================================================================
`ifdef CREDIT_FIFO_DEPTH_16
    localparam CF_DEPTH = 16;
`elsif CREDIT_FIFO_DEPTH_32
    localparam CF_DEPTH = 32;
`elsif CREDIT_FIFO_DEPTH_128
    localparam CF_DEPTH = 128;
`else
    localparam CF_DEPTH = 64;                 // 默认 64（与 replay buffer 深度一致）
`endif
    localparam PTR_W = $clog2(CF_DEPTH);      // bank/指针位宽
    localparam CNT_W = PTR_W + 1;             // 每 bank cnt 位宽（16/32/64/128→5/6/7/8，M2）
    localparam ZEXT_W = CNT_W - 1;            // ±1 操作数零扩位数（1bit→CNT_W，W116 宽度匹配）

    // =========================================================================
    // 内部信号声明（wire / reg 统一定义在最前，wire 与 assign 分离）
    // =========================================================================

    // 寄存器（指针/占用计数均在本层；bank 为纯存储阵列）
    reg  [PTR_W-1:0]  wr_ptr;         // 兼容模式写递增计数指针
    reg  [PTR_W-1:0]  rd_ptr;         // 读指针（性能=交付 seq；兼容=递增计数）
    reg  [PTR_W-1:0]  cur_seq;        // 性能模式当前 Flit seq 锁存（front 写时锁存，back 复用）
    reg  [1:0]        rd_seq_state;   // 读序列相位（0=b0、1=b1、2=b2、3=b3，PAIF512 每拍 1 bank）
    reg  [CNT_W-1:0]  cnt_b0;         // u_bank0 目数（front 低 512）
    reg  [CNT_W-1:0]  cnt_b1;         // u_bank1 目数（front 高 512）
    reg  [CNT_W-1:0]  cnt_b2;         // u_bank2 目数（back  低 512）
    reg  [CNT_W-1:0]  cnt_b3;         // u_bank3 目数（back  高 512）
    reg               empty_b0;       // u_bank0 空
    reg               empty_b1;       // u_bank1 空
    reg               empty_b2;       // u_bank2 空
    reg               empty_b3;       // u_bank3 空

    // 行寄存器（LL-TIMING-N1 FWFT 读重构：rd_ptr_adv 拍锁 mem[rd_ptr_next]（bank 改址读
    // bX_rd）；写命中当前行拍补 flit_in 半——命中当前行=0→1 首 push（front）或读序
    // 进行中的 back push（perf 读序冻结态 2、back 单元迟到的当前行补写，b2/b3 pop
    // 拍恒晚于该写沿 ≥1 拍，装载及时）；写命中下读序行且 rd_ptr_adv 同拍（bank 写
    // 与 row_cur 读同沿竞态）取 flit_in——读序首拍恒=mem[rd_ptr]，协议拍序/链路总
    // 延迟零变化、无新增 pipe；无复位：类 bank mem 存储阵列豁免（去清零口径），
    // X 泄漏由 pop 仅对 empty=0 bank 有效 + 写命中仅有效 push 保证）
    reg  [511:0]      row_cur_b0;     // u_bank0 行（当前读序交付行，front 低 512）
    reg  [511:0]      row_cur_b1;     // u_bank1 行（front 高 512）
    reg  [511:0]      row_cur_b2;     // u_bank2 行（back  低 512）
    reg  [511:0]      row_cur_b3;     // u_bank3 行（back  高 512）

    // bank 读出数据（LL-TIMING-N1：bank 读 mux 改址 rd_ptr_next——bX_rd=mem[rd_ptr_next]，
    // 供 row_cur 于 rd_ptr_adv 拍锁存下读序行；4×512：b0/b1=front 低/高、b2/b3=back 低/高）
    wire [511:0]      b0_rd;          // u_bank0 直读（mem[rd_ptr_next]，front 低 512）
    wire [511:0]      b1_rd;          // u_bank1 直读（mem[rd_ptr_next]，front 高 512）
    wire [511:0]      b2_rd;          // u_bank2 直读（mem[rd_ptr_next]，back  低 512）
    wire [511:0]      b3_rd;          // u_bank3 直读（mem[rd_ptr_next]，back  高 512）

    // 写侧组合
    wire [PTR_W-1:0]  lo_wr_addr;     // b0/b1（front）写地址（性能=seq / 兼容=wr_ptr）
    wire [PTR_W-1:0]  hi_wr_addr;     // b2/b3（back）写地址（性能=seq；兼容 不写）
    wire              hi_kind;        // 性能 back 单元（push 拍 perf & end；兼容/perf front=0）
    wire              lo_wr_en;       // 实际写 b0/b1（阻塞则丢弃）
    wire              hi_wr_en;       // 实际写 b2/b3（阻塞则丢弃）
    wire              lo_full;        // front（b0/b1）满
    wire              hi_full;        // back（b2/b3）满
    wire              lo_wr_block;    // front 写阻塞（LL-TIMING-N1 预并：hi_kind | lo_full）
    wire              hi_wr_block;    // back  写阻塞（LL-TIMING-N1 预并：~hi_kind | hi_full）
    wire              overflow_set;   // 溢出脉冲（1 cycle）
    wire              lo_wr_hit_rd;   // front 写命中当前行（lo_wr_en & lo_wr_addr==rd_ptr；
                                      // 仅 0→1 首 push——读序进行中 front 已 pop 空、不可再 push）
    wire              hi_wr_hit_rd;   // back  写命中当前行（hi_wr_en & hi_wr_addr==rd_ptr；
                                      // 0→1 首 push 或读序冻结态 2 的当前行 back 迟到 push）
    wire              lo_wr_hit_next; // front 同拍 push 命中下读序行（rd_ptr_adv & lo_wr_en &
                                      // lo_wr_addr==rd_ptr_next：push 写与 row_cur 读同沿竞态，取 flit_in）
    wire              hi_wr_hit_next; // back  同拍 push 命中下读序行
    wire [CF_DEPTH-1:0] lo_row_sel;   // 写行 one-hot（b0/b1 共用；bank 逐行 ICG 使能预计算）
    wire [CF_DEPTH-1:0] hi_row_sel;   // 写行 one-hot（b2/b3 共用）
    genvar            gk;
    wire              wr_advance;     // 兼容模式 wr_ptr 推进
    wire              front_write;    // 性能模式 front（b0/b1）写命中，锁存 Flit seq
    wire [PTR_W-1:0]  wr_ptr_next;    // 兼容 wr_ptr 下一拍（mod CF_DEPTH）
    wire [PTR_W-1:0]  rd_ptr_next;    // rd_ptr 下一拍（mod CF_DEPTH）
    wire [CNT_W-1:0]  cnt_b0_nxt;     // 每 bank 目数下一拍（+写 -ren）
    wire [CNT_W-1:0]  cnt_b1_nxt;
    wire [CNT_W-1:0]  cnt_b2_nxt;
    wire [CNT_W-1:0]  cnt_b3_nxt;
    wire [CNT_W+2:0]  occ_sum;        // occupancy 四 bank 和（各 cnt 零扩至 CNT_W+3；
                                      //  最大值 4×CF_DEPTH = 2^(CNT_W+1)，无截断）

    // 读序列组合
    wire              is_self_b0;     // 读序列相位命中该 bank（b0）
    wire              is_self_b1;     // （b1）
    wire              is_self_b2;     // （b2；性能相位，兼容止于 b1）
    wire              is_self_b3;     // （b3；性能相位，兼容止于 b1）
    wire              ren_b0;         // b0 读使能（= ~empty & pop_i & is_self）
    wire              ren_b1;         // b1 读使能
    wire              ren_b2;         // b2 读使能（兼容恒 0）
    wire              ren_b3;         // b3 读使能（兼容恒 0）
    wire              rd_seq_adv;     // 任一 bank ren 即推进读序列
    wire              rd_seq_done;    // perf=b3 / compat=b1（读满该 Flit bank 组）
    wire [1:0]        rd_seq_next;    // done 回绕 0，防 bank 轮转悬空
    wire              rd_ptr_adv;     // rd_ptr 推进（= rd_seq_adv & rd_seq_done）

    // =========================================================================
    // 组合逻辑 assign
    // =========================================================================

    // 入口无反压（无 push_ready 口）：满 + 新 push 丢弃该单元并报溢出。
    // LL-TIMING-N1：写阻塞条件预并（lo/hi_wr_block）——push_i 仅经 1 级 AND 到 wr_en
    // （旧 raw&~full 两级）；in2reg/CLKICG push→wr_en 链降 1 级。位级等价：
    //   lo = push & ~(hi_kind | lo_full)（hi_kind = perf&end = 旧 raw 的 ~ 项）
    //   hi = push & ~(~hi_kind | hi_full)
    //   overflow = push & (lo_full & ~hi_kind | hi_full & hi_kind)（= 旧 raw&full 两式）
    assign hi_kind      = csr_perf_mode_i & flit_end_i;               // 性能 back 单元
    assign lo_full      = (cnt_b0 == CF_DEPTH[CNT_W-1:0]) | (cnt_b1 == CF_DEPTH[CNT_W-1:0]);
    assign hi_full      = (cnt_b2 == CF_DEPTH[CNT_W-1:0]) | (cnt_b3 == CF_DEPTH[CNT_W-1:0]);
    assign lo_wr_block  = hi_kind | lo_full;
    assign hi_wr_block  = ~hi_kind | hi_full;
    assign overflow_set = push_i & (lo_full & ~hi_kind | hi_full & hi_kind);
    assign lo_wr_en     = push_i & ~lo_wr_block;                      // 满则丢弃（写使能）
    assign hi_wr_en     = push_i & ~hi_wr_block;

    // 性能模式 front 写命中：锁存该 Flit 的 data seq，供 back 写地址复用
    // （back 单元内 [PTR_W+3:4] 为 DLP 字段，不含 data seq，须沿用 front 的 seq）
    assign front_write  = lo_wr_en & csr_perf_mode_i;

    // 写地址：性能模式 = Flit seq 低位（front 直接取 flit_in，back 复用锁存 seq）；
    // 兼容模式 = 递增计数指针（只写 b0/b1）
    assign lo_wr_addr   = {PTR_W{ csr_perf_mode_i}} & flit_in_i[PTR_W+3:4] | {PTR_W{~csr_perf_mode_i}} & wr_ptr;
    assign hi_wr_addr   = {PTR_W{csr_perf_mode_i}} & cur_seq;

    // 指针推进：兼容 wr_ptr 在 b0/b1 写后推进；性能 wr_ptr 恒不动（seq 索引）
    assign wr_advance   = lo_wr_en & ~csr_perf_mode_i;
    assign wr_ptr_next  = {PTR_W{~(wr_ptr == (CF_DEPTH[PTR_W-1:0] - 1'b1))}} & (wr_ptr + 1'b1);
    assign rd_ptr_next  = {PTR_W{~(rd_ptr == (CF_DEPTH[PTR_W-1:0] - 1'b1))}} & (rd_ptr + 1'b1);
    // LL-TIMING-N1：写命中当前行（lo=0→1 首 push；hi=首 push 或读序冻结态 2 的当前行
    // back 迟到 push——b2/b3 pop 拍恒晚于该写沿 ≥1 拍，装载及时）：row_cur 取该拍 flit_in 半
    //（bank mem 同沿写入，bX_rd 旧值不可取）
    assign lo_wr_hit_rd = lo_wr_en & (lo_wr_addr == rd_ptr);
    assign hi_wr_hit_rd = hi_wr_en & (hi_wr_addr == rd_ptr);
    // LL-TIMING-N1：同拍竞态 push（rd_ptr_adv 拍——当前行末 pop 拍——恰逢 push 写入下读序行
    // rd_ptr_next：bank mem 写与 row_cur 读同沿，row_cur 取 bX_rd 为旧值 → 改取该拍 flit_in
    // 半（push 数据组合可用，位级=本沿写入值）。仅该沿 row_cur 读的行=rd_ptr_next，eq 判据精确；
    // 与 hit_rd 互斥（rd_ptr≠rd_ptr_next、0→1 push 拍无 pop 进行 adv=0）
    assign lo_wr_hit_next = rd_ptr_adv & lo_wr_en & (lo_wr_addr == rd_ptr_next);
    assign hi_wr_hit_next = rd_ptr_adv & hi_wr_en & (hi_wr_addr == rd_ptr_next);
    // LL-TIMING-N1：写行 one-hot（(wr_addr==K) 等值预计算；b0/b1 共用 lo、b2/b3 共用 hi——
    // 4bank 原各算 16 路等值，现 2 组共享；等值链为 wr_addr（寄存器/数据）驱动、不在
    // push→ICG 关键链上，bank 逐行写 ICG 使能 = wr_en & row_sel 单 AND 级）
    generate
        for (gk = 0; gk < CF_DEPTH; gk = gk + 1) begin : g_row_sel
            assign lo_row_sel[gk] = (lo_wr_addr == gk);
            assign hi_row_sel[gk] = (hi_wr_addr == gk);
        end
    endgenerate

    // 每 bank 目数 nxt：cnt + 写 - ren（同拍 push+pop 相抵；CNT_W 无回绕，满判可靠；
    // ±1 操作数零扩至 CNT_W 宽度匹配，嵌套形状 {{ZEXT_W{1'b0}},x}，见编码约束 5）
    assign cnt_b0_nxt   = cnt_b0 + {{ZEXT_W{1'b0}}, lo_wr_en} - {{ZEXT_W{1'b0}}, ren_b0};
    assign cnt_b1_nxt   = cnt_b1 + {{ZEXT_W{1'b0}}, lo_wr_en} - {{ZEXT_W{1'b0}}, ren_b1};
    assign cnt_b2_nxt   = cnt_b2 + {{ZEXT_W{1'b0}}, hi_wr_en} - {{ZEXT_W{1'b0}}, ren_b2};
    assign cnt_b3_nxt   = cnt_b3 + {{ZEXT_W{1'b0}}, hi_wr_en} - {{ZEXT_W{1'b0}}, ren_b3};

    // 每 bank 空标志：cnt_nxt==0 打拍（regout，valid 全 regout，仅由 empty 控）
    always @(posedge mb_clk) begin
        if (!mb_rst_n) begin
            empty_b0 <= 1'b1;
            empty_b1 <= 1'b1;
            empty_b2 <= 1'b1;
            empty_b3 <= 1'b1;
        end else begin
            empty_b0 <= (cnt_b0_nxt == {CNT_W{1'b0}});
            empty_b1 <= (cnt_b1_nxt == {CNT_W{1'b0}});
            empty_b2 <= (cnt_b2_nxt == {CNT_W{1'b0}});
            empty_b3 <= (cnt_b3_nxt == {CNT_W{1'b0}});
        end
    end

    // 读序列 counter：is_self（rd_seq_state==N）逐 bank 选通，ren 逐拍拉高
    // （= ~empty & pop_i & is_self；pop_i=协议层 paif_rx_ready，ready=0 冻结零丢失）
    assign is_self_b0   = ~(|rd_seq_state);
    assign is_self_b1   = ~rd_seq_state[1]&rd_seq_state[0];
    assign is_self_b2   =  rd_seq_state[1]&~rd_seq_state[0];  // 性能相位（兼容止于 b1）
    assign is_self_b3   =  &rd_seq_state;
    assign ren_b0       = ~empty_b0 & pop_i & is_self_b0;             // 用户契约：就这是 fifo 读使能
    assign ren_b1       = ~empty_b1 & pop_i & is_self_b1;
    assign ren_b2       = ~empty_b2 & pop_i & is_self_b2;             // 兼容下恒 0（空 bank 不拉 ren）
    assign ren_b3       = ~empty_b3 & pop_i & is_self_b3;
    assign rd_seq_adv   = ren_b0 | ren_b1 | ren_b2 | ren_b3;          // 任一 bank ren 即推进
    assign rd_seq_done  = csr_perf_mode_i ? is_self_b3 : is_self_b1;  // perf 4 bank / compat 2 bank（M3）
    assign rd_seq_next  = rd_seq_done ? 2'd0 : rd_seq_state + 2'd1;   // 完成回绕，防轮转悬空

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            rd_seq_state <= 2'd0;
        else if (rd_seq_adv)   // BUG-002: 去 &csr_perf_mode_i（读序推进全模式统一，MAS v1.6.12 §5.1；compat b0→b1 收窄由 rd_seq_done=is_self_b1 回绕承担）
            rd_seq_state <= rd_seq_next;                              // b0→…→done→回 0
    end

    assign rd_ptr_adv   = rd_seq_adv & rd_seq_done;                   // 读满该 Flit bank 组后 rd_ptr+1

    // =========================================================================
    // credit_fifo_bank 例化（4×512 并行 bank 寄存器阵列；LL-TIMING-N1：读 mux 改址
    // rd_ptr_next 供 row_cur 锁存下读序行、写行 one-hot wr_row_sel 逐行 ICG）
    // 映射：b0/b1=front（低/高 512）、b2/b3=back（低/高 512）。front/back 写拍
    // 同时写其两块 512 bank（各取 flit_in 低/高半）；读侧按 rd_ptr_next 同地址并行
    // 读 4 bank（row_cur 行寄存器锁存后由 is_self 片选单 bank 512 直出填 1024 口）。
    // =========================================================================
    credit_fifo_bank #(
        .DATA_WIDTH (512),
        .DEPTH      (CF_DEPTH),
        .PTR_W      (PTR_W)
    ) u_bank0 (
        .clk       (mb_clk),
        .rst_n     (mb_rst_n),
        .wr_en     (lo_wr_en),
        .wr_row_sel(lo_row_sel),
        .wr_data   (flit_in_i[511:0]),
        .rd_addr   (rd_ptr_next),
        .rd_data   (b0_rd)
    );

    credit_fifo_bank #(
        .DATA_WIDTH (512),
        .DEPTH      (CF_DEPTH),
        .PTR_W      (PTR_W)
    ) u_bank1 (
        .clk       (mb_clk),
        .rst_n     (mb_rst_n),
        .wr_en     (lo_wr_en),
        .wr_row_sel(lo_row_sel),
        .wr_data   (flit_in_i[1023:512]),
        .rd_addr   (rd_ptr_next),
        .rd_data   (b1_rd)
    );

    credit_fifo_bank #(
        .DATA_WIDTH (512),
        .DEPTH      (CF_DEPTH),
        .PTR_W      (PTR_W)
    ) u_bank2 (
        .clk       (mb_clk),
        .rst_n     (mb_rst_n),
        .wr_en     (hi_wr_en),
        .wr_row_sel(hi_row_sel),
        .wr_data   (flit_in_i[511:0]),
        .rd_addr   (rd_ptr_next),
        .rd_data   (b2_rd)
    );

    credit_fifo_bank #(
        .DATA_WIDTH (512),
        .DEPTH      (CF_DEPTH),
        .PTR_W      (PTR_W)
    ) u_bank3 (
        .clk       (mb_clk),
        .rst_n     (mb_rst_n),
        .wr_en     (hi_wr_en),
        .wr_row_sel(hi_row_sel),
        .wr_data   (flit_in_i[1023:512]),
        .rd_addr   (rd_ptr_next),
        .rd_data   (b3_rd)
    );

    // =========================================================================
    // 时序逻辑（同步复位，always @(posedge mb_clk)，禁用异步复位）
    // 每个 always 块内 if 判断（不含 reset 判断）≤ 2 个
    // =========================================================================

    // 兼容模式写计数指针推进（纯标准 FIFO，去 flush：指针仅读写自然推进）
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            wr_ptr <= {PTR_W{1'b0}};
        else if (wr_advance)
            wr_ptr <= wr_ptr_next;
    end

    // 性能模式 Flit seq 锁存：front 写时锁存，back 写时沿用
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            cur_seq <= {PTR_W{1'b0}};
        else if (front_write)
            cur_seq <= flit_in_i[PTR_W+3:4];
    end

    // 读指针推进（性能=交付 seq / 兼容=计数）；rd_ptr_adv = rd_seq_adv & rd_seq_done
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            rd_ptr <= {PTR_W{1'b0}};
        else if (rd_ptr_adv)
            rd_ptr <= rd_ptr_next;
    end

    // 行寄存器装载（LL-TIMING-N1）：优先取写命中行的 flit_in 半（hit_rd：空 FIFO 0→1 首 push；
    // hit_next：rd_ptr_adv 拍同沿 push 写 rd_ptr_next 行——bank 写与 row_cur 读同沿竞态，
    // bX_rd 为旧值不可取）；否则 rd_ptr_adv 拍取 bank 改址读 bX_rd=mem[rd_ptr_next]（下一读序行）。
    // 读序首拍 row_cur 恒=mem[rd_ptr]（本沿写入后的值）。if≤2、无复位。
    always @(posedge mb_clk) begin
        if (lo_wr_hit_rd | lo_wr_hit_next)
            row_cur_b0 <= flit_in_i[511:0];
        else if (rd_ptr_adv)
            row_cur_b0 <= b0_rd;
    end

    always @(posedge mb_clk) begin
        if (lo_wr_hit_rd | lo_wr_hit_next)
            row_cur_b1 <= flit_in_i[1023:512];
        else if (rd_ptr_adv)
            row_cur_b1 <= b1_rd;
    end

    always @(posedge mb_clk) begin
        if (hi_wr_hit_rd | hi_wr_hit_next)
            row_cur_b2 <= flit_in_i[511:0];
        else if (rd_ptr_adv)
            row_cur_b2 <= b2_rd;
    end

    always @(posedge mb_clk) begin
        if (hi_wr_hit_rd | hi_wr_hit_next)
            row_cur_b3 <= flit_in_i[1023:512];
        else if (rd_ptr_adv)
            row_cur_b3 <= b3_rd;
    end

    // 每 bank 目数（CNT_W 无回绕：max 16 ≤ 2^(CNT_W)-1；同拍 push+pop 相抵）
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            cnt_b0 <= {CNT_W{1'b0}};
        else
            cnt_b0 <= cnt_b0_nxt;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            cnt_b1 <= {CNT_W{1'b0}};
        else
            cnt_b1 <= cnt_b1_nxt;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            cnt_b2 <= {CNT_W{1'b0}};
        else
            cnt_b2 <= cnt_b2_nxt;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            cnt_b3 <= {CNT_W{1'b0}};
        else
            cnt_b3 <= cnt_b3_nxt;
    end

    // =========================================================================
    // 输出赋值（PAIF512 单 bank 512 直出；valid 全 regout（empty/is_self 相位/perf_mode）；flit_end 由阶段+模式派生）
    // =========================================================================
    // v1.6.4 用户终裁：pop_valid_o = ~empty_b0 | ~empty_b1 | ~empty_b2 | ~empty_b3
    //（NOR4 任一 bank 非空，不分 mode/接口；is_self 只控 ren/flit_out 片选/
    // rd_seq_done，不控 valid；v1.6.3 pad 相位静默 pop 语义作废；M4 不含
    // pop_i/ready，断组合环；全 regout：empty_bX 寄存器）——两形态分述
    //（MAS v1.6.12 §1.1）：perf 有效 = b0..b3 任一非空 / compat 有效 = b0/b1
    // 任一非空（b2/b3 compat 不写恒空）
    assign pop_valid_o = ~empty_b0 | ~empty_b1 | ~empty_b2 |~ empty_b3;
    // PAIF512 单管：每拍读序列相位（is_self）选通单 bank 行寄存器 row_cur_bX，数据落在
    // [511:0]（{512{is_self_bX}} & row_cur_bX：512-bit 操作数复制掩码零扩直出），上 512 恒 0
    // （PAIF_INTF_512 下父取 [511:0]，见 link_rx.v:736）。LL-TIMING-N1：行寄存当拍直出
    //（FWFT 语义不变——rd_ptr 变更次拍即有效，读延迟 0 拍，reg2out 由行寄存器起算）
    assign flit_out_o   ={512'h0, {512{is_self_b0}} & row_cur_b0
                        | {512{is_self_b1}} & row_cur_b1
                        | {512{is_self_b2}} & row_cur_b2
                        | {512{is_self_b3}} & row_cur_b3};
    assign flit_end_o   = csr_perf_mode_i ? (is_self_b2 | is_self_b3) : 1'b1;
    assign overflow_o   = overflow_set;
    // DFx occupancy：各 bank 计数先零扩至 CNT_W+3 求和（occ_sum，最大值
    // 4×CF_DEPTH = 2^(CNT_W+1)，无截断），按口宽出低 8b（修旧直加 CNT_W 截断
    // 回绕：满 4×16=64 档需 7b，occupancy_o 8b；嵌套形状见编码约束 5）
    assign occ_sum      = {{3{1'b0}}, cnt_b0} + {{3{1'b0}}, cnt_b1}
                        + {{3{1'b0}}, cnt_b2} + {{3{1'b0}}, cnt_b3};
    assign occupancy_o  = occ_sum[7:0];

endmodule
