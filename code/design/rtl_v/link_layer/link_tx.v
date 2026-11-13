// =============================================================================
// module name：link_tx
// author: OpenHappy Team
// MAS：spec/link_layer/submods/link_tx/link_tx_mas.md（MAS，单文档多节，v1.9）+
//      mas/crc9_generate_mas.md / crc16_generate_mas.md / replay_buffer_mas.md /
//      mas/link_tx_512_lane1_mas.md / link_tx_1024_lane1_mas.md /
//      link_tx_1024_lane2_mas.md / link_tx_compat_mas.md
// 功能概述: TX 侧核心处理级（父模块）。保留父模块全部控制逻辑：flit_sched（PAIF
//           按拍累积 / beat 计数 / seq 分配）、tx_seq_mgr、dlp_update（组合嵌入）、
//           ctrl_flit_gen（NULL / ARQ 自生成 + 注入仲裁）、retry_ctrl（NACK/超时/
//           ARQ 触发 + replay 读侧 + retrain 计数）、timeout_monitor、credit 指针
//           （replay buffer wr/retire 推导）、udi_bypass_mux（含 UDI 入口一级 pipe）、
//           deskew（S_LINK_TRAIN Pattern）、统计、中断上报、csr 透传。
//           PAIF↔CPIF 位宽 / lane / 拆分 / 钟漂反压转换由 4 个形态子模块
//           （link_tx_512_lane1 / link_tx_1024_lane1 / link_tx_1024_lane2 /
//           link_tx_compat，MAS §2 / §4.10~4.13）按编译期参数（PAIF_INTF_512 /
//           LANE_MACRO_MODE）二选一例化承担——父模块例化全部 4 个形态，各自以
//           编译期宏条件包住，编译时仅 1 个被激活（非 generate、非运行时）。
//           父模块把 crc 数据上送、父组装回喂单元、NULL/重传注入、replay push 上送、
//           CPIF lane 输出接到「当前例化」形态的端口（4 个形态的同类交接信号在
//           父模块用宏选择哪个连出去）。
// 编码约束（本模块强制）:
//   1. 仅时序信号使用 always 语句（always @(posedge mb_clk)，同步复位，
//      禁用异步复位）；
//   2. always 内部不允许 if 嵌套 if，最多 2 个判断条件（条件用 & 归并）；
//   3. 组合逻辑只允许 assign 赋值（不使用 always @(*)）；
//   4. for 循环仅用于 generate 实例化；
//   5. wire 与 assign 分离，wire 集中在文件头内部信号区声明。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   <日期>  <修订人>  <变更概述>
//  2026-08-06  OpenHappy Team  初始版本（对齐 L2 v1.4 / L1 v2.26 DLP 组合嵌入 /
//                          L1 v2.28 link_tx_active_i 出口反压）
//  2026-08-07  OpenHappy Team  对齐 L2 v1.5：crc0/crc1 使能合并为单 bit csr_crc_en_i；
//                          lane1/2/3 端口按 `ifdef 条件声明
//  2026-08-08  OpenHappy Team  全量重写（对齐评审中 v1.0 MAS）：paif_cpif_conv / bw_mode
//                          带宽门控 / 端口同步 / 桥接直连
//  2026-08-08  OpenHappy Team  重传读控制补读在途门控 / 同拍写冲突处理
//  2026-08-08  OpenHappy Team  link_tx_active_i → link_state_i（lcmsm 只输出状态）
//  2026-08-12  OpenHappy Team  lint 修复：bridge s2p 直连零扩展（SYNTH_5251）
//  2026-08-14  OpenHappy Team  新增 ctrl_flit_gen 控制 Flit 自生成 + 注入仲裁
//  2026-08-14  OpenHappy Team  RTL 重写骨架：仅保留接口 + 子模块例化
//  2026-08-14  OpenHappy Team  RTL 增量重写（正常发送数据通路 / 重传 / ARQ / UDI pipe /
//                          注错 / deskew，详见 link_tx.v 前身多轮修订）
//  2026-08-21  OpenHappy Team  修复 dlp_cmd_reg/dlp_seq_reg 无清除路径（BUG B-03）
//  2026-11-08  OpenHappy Team  PAIF↔CPIF 转换下沉为 4 个形态子模块（对齐 link_tx_mas
//                          v1.9 §4.10~4.13）：父模块保留全部控制逻辑，删除内联
//                          paif_cpif_conv 发送段（conv_latch/conv_busy/conv_beat_*/
//                          tx_hold/s2p_sel/conv_lane*），改为 4 个形态模块按编译期
//                          宏条件例化（默认 512×1lane → link_tx_512_lane1）。父模块
//                          经宏选择把 crc/回喂单元/NULL重传/replay/CPIF 接到当前
//                          激活形态端口；对外顶层端口不变。
//  2026-09-02  OpenHappy Team  BUGFIX：NULL/重传 Flit 无 CRC（reverse B→A TX PAIF
//                          反压死锁根因①）。ctrl_flit_gen NULL 半单元（ctrl_perf_null）
//                          直接送 unit_ckd_f，未回填 CRC16 → 对端 RX crc16_check 拒绝 →
//                          NACK → retry_active 死锁（replay 缓冲空无可读）→
//                          paif_ready_ctl=0 → paif_tx_ready_o 恒 0 入口堵死。
//                          修复：NULL 半单元经 crc16 引擎回填（null_unit_crc），
//                          与数据/重传同构后注入 unit_ckd_f（BUG LOG 2026-09-02）。
//  2026-09-02  OpenHappy Team  BUGFIX：NULL/重传 front→back 事件 1 拍内连发，与
//                          s2p=0 高半段（含 CRC）发送冲突 → 高半段被覆盖丢失。
//                          新增形态端口 unit_busy_512_lane1_o（高半段在途标志），
//                          null/retry front/back 事件以 ~unit_busy_f 门控，避免
//                          上单元高半段未发完时新装载覆盖（BUG LOG 2026-09-02）。
//  2026-09-11  OpenHappy Team  BUGFIX：重传进入改到 PAIF Flit 边界（retry_trigger 不再直接置
//                          retry_active）——NACK/超时恰逢数据 Flit 中途时，直接切断会令
//                          「前端已上线、后端未收满」的半 Flit 成为孤 front，线上 front/back
//                          奇偶错位，对端 RX 按 unit_idx 递推 front/back 整体翻转 → 重传以
//                          错误相位到达 → 数据错乱（CASE_8/t21 复现链路）。改为：触发拍锁存
//                          retry_pending/init/stop，待 paif_beat_cnt 回 0（整 Flit 收齐的
//                          稳定边界、且在途 NULL/ARQ 排空）才进 retry_active；被排空的 Flit
//                          以完整 F/B 上线并随重传窗口（动态 stop=进入拍 tx_seq）一并重传。
//  2026-09-23  OpenHappy Team  注释修正（LL-20260923-CHK1008）：ecc16_result[11:0] 全宽嵌入
//                          {P, tie0, SEC[9:0]}（bit 11 由恒 0 改整体奇偶位 P；TX 3 形态
//                          原全宽嵌入路径不变，无硬连线放开）；本文件功能零改动
//  2026-09-23  OpenHappy Team  LL-20260923-TX512REGOUT（形态① v1.11 单元泵 / 父 v1.21 重传瘦化，
//                          arch 一/二轮通过）：`PAIF_INTF_512 分支重传改 buffer 行流自走读——
//                          父仅发 retrans_start 单事件（rptr_start/len 经父到 buffer；start/len 到
//                          child），删 retry_flit(2048) 锁存 / retry_flit_loaded / retry_half /
//                          front-back 双事件 / retry_rd_busy（512 形态）；retry_rd_seq 降为影子
//                          （retrans_flit_done+1，停止决策/统计）；新增 len==0 空窗 guard（新
//                          corner，注释标明）；NULL 改 child 自生成（null_flit_start_f 直送
//                          child、null_done_o 清 null_active，删 null_half / front-back 双事件 /
//                          ctrl_perf_null）；exit_hdr 512 删 RETRY 支（重传 front 行含固化
//                          header、back 行 child 内 DLP 替换）；DLP 清除锚点迁 back_proc /
//                          retrans_flit_done / null_done；flit_sent_event 512 = front_proc |
//                          retrans_start | null_start（锚点迁移）；UDI 512 删 udi_pipe_lane0
//                          （raw 入口直送 child、child 1T 无条件寄存，arch M7），cpif lane0 父
//                          mux 退化为 deskew | child 终态；udi_tx_ready_o 不变；buffer 增 6 个
//                          retrans_* 端口接线（ren 512 恒 0）。1024 两形态 / compat /
//                          CRC 1008 口径 / 写侧 / credit / retire / full / ack 全部不变。
//  2026-09-25  OpenHappy Team  BUGFIX（形态① PAIF_INTF_512 NULL front 拍头采样错位一拍，10-CASE 回归全 FAIL 根因）：
//                          NULL child 自生成 beat0 线上数据（null_cur）于起始拍沿装载 header_f，而父 exit_hdr
//                          NULL 支原由寄存 null_active 门控、起始拍次拍才成立 → 线上头 = DATA 支 {tx_seq,
//                          type=1}（0x0001），CRC 窗口（child null_field，发送拍采样）= NULL 支 {8'hff, type=0}
//                          （0x0FFx）→ 线上头与 CRC 计及头错位一拍 → RX front 拍 chkerr + is_null_c（type=0 &
//                          seq=0xff）失判（err_discard / 孤儿 back push / cfifo 冻结 / retrain 级联）。修复：
//                          exit_src NULL 支扩至起始拍（null_active | null_flit_start_f，本文件出口注入区）；
//                          起始拍全路 quiescent（~unit_busy_f）且 null_idle_ok 要求 ~paif_tx_valid_i，
//                          DATA 支 / paif 装载不受影响；DLP back 拍（dlp_f 独立源）不变；child
//                          link_tx_512_lane1 / 1024 两形态 / compat / RX 侧零 diff。
//  2026-09-29  OpenHappy Team  LL-TIMING-N1 reg2reg 2 引擎拆分回退（功能回归：CASE_14/2
//                          重传 chk 槽在 X 过渡拍/重传拍与基线分叉）——恢复原单引擎
//                          结构（crc16_generate/ecc_generate 各 1，crc16_data 三臂
//                          窗口直喂）。24 级 reg2reg 控制穿数据链保持原样，ESCALATE
//                          已回报 user（TX 出口加 1 级发送寄存，或放开 sa 数据门控
//                          /retry 臂控制裁决）。RX 侧（credit_fifo/ecc_check/link_rx
//                          nz 预 OR/row_cur/proto_not）不在本回退范围。
//  2026-10-08  OpenHappy Team  LL-20260930-N4-v3（F1a）：+1 输入口 link_active_nxt_i + per-link
//                          同 D 副本寄存器 link_active_cp（mb_clk 无条件寄存、无复位 X-safe，
//                          D = lcmsm 次态位 st_link_active_nxt，cp ≡ cur_state[4] 逐拍恒等含
//                          迁移/复位拍；D=Q 直连副本影子禁用）；512 形态 child
//                          .link_tx_active_i 改接 link_active_cp（唯一改接线点）；tx_active /
//                          tx_idle / timeout_run / udi_tx_ready_o 仍读原 link_tx_active
//                          （最小改动面，零功能变化）。
//  2026-10-10  OpenHappy Team  LL-20261010-R2G1 B 票（R-B1a：重传窗 X 读家族 c24/c8；arch LLArchG1B 通过）：
//                          retry_stop_seq 动态 stop 改 min_mod256(tx_seq, push_done_seq+1)——新增
//                          push_done_seq（最后完全写 back 单元 push 完成 flit seq，复位 8'hFF=-1；512 形态 =
//                          child wen[1] push_back 收齐拍 +1 递推，与 tx_seq 常态锁步）；常态 d≡0 →
//                          stop=tx_seq 零回归（NACK/超时触发时 push 早已完成，c22/③ 家族签名不回退）；
//                          push 未齐 → stop=push_done_seq+1 窗截断（空窗由既有 retrans_empty_clear
//                          承接，超时/NACK 再触发无 flit 丢失）；1024/兼容形态零改动（单 wen/accept
//                          同拍结构无 X 窗角，retry_read_issue 段不动）；timing：retry_stop_seq D 锥
//                          +4-6 级（≤23 事件锥，非逐拍 critical），push_done_seq 纯 +1 计数非关键。
//                          改动面 = 头注释 + 声明块 + 新 always + 3 wire + retry_stop_seq 动态支
//                          （'ifdef PAIF_INTF_512 包内）。
//  2026-10-12  OpenHappy Team  LL-20261010-G3（G3 arch LLArchG3 通过）：§F c10 带宽缩减
//                          （child link_tx_512_lane1 出口 W 拍队列 + 低段在前切片，v0.14）——
//                          父侧仅 2 处：① 形态① child 例化 .csr_bw_mode_i 1-bit 截位改 2-bit
//                          全量透传（BW_CFG_REG 2-bit 口径，1/2 / 1/4 缩减激活；0 版本本形态
//                          恒 0 截位作废）；② §F 域外（§E 同票）null_idle_ok 载体修复见下。
//  2026-10-12  OpenHappy Team  LL-20261010-G3（G3 arch LLArchG3 通过）：§E c7 loopback 自 ACK 载体
//                          修复——null_idle_ok = proto_is_paif & ~retry_active & ~paif_rx_accept
//                          （~paif_rx_accept = ~paif_tx_valid_i | paif_rx_stall，含 held-valid
//                          valid=1/stall=1 反压角）：NULL 载体在 held-valid 反压期保持活性，
//                          loopback 自 ACK 闭环 pumping（三 corner 等价性 arch 已证）；兼容形态
//                          null_gen_req 保持旧 ~paif_tx_valid_i 读判据（内联旧 null_idle_ok 公式，
//                          held-valid 反压×compat NULL 并发行为差按 arch 技术注②保留语义，
//                          bit 位级零回归）；paif_rx_accept 为既有线（L514）无新增。
// =============================================================================

 // 2026-10-13  OpenHappy Team  动态时钟控制去总开关（arch LL-20261013-dynclk 通过）：删
 //                          input clk_cfg_tx_dyn_en_i（总开关），clken_tx 化简
 //                          ~(dyn_en&n_dyn)|… → ~n_dyn|…（NAND2→INV -1 级）；本路
 //                          clk_cfg_tx_n_dyn_i 语义改「本 TX 独立动态时钟开关」。
`timescale 1ns/1ps

`include "link_layer_defines.vh"

module link_tx (
    // =========================================================================
    // 时钟与复位
    // =========================================================================
    input  wire                       mb_clk,                 // 2 GHz 数据通路主时钟
    input  wire                       mb_rst_n,               // 同步复位，低有效
    input  wire                       clk_cfg_tx_n_dyn_i,     // 本 TX 独立动态时钟开关（CLK_CFG_TX_REG[n]）：1=动态 clken / 0=always-on（总开关已删）
    input  wire                       sram_dyn_en_i,          // replay SRAM 动态门控使能（→ replay_buffer u_icg_sram）

    // =========================================================================
    // 上游 PAIF 接口（协议层 → link_tx）
    // =========================================================================
    input  wire [`PAIF_WIDTH-1:0]     paif_tx_data_i,         // PAIF TX 数据
    input  wire                       paif_tx_valid_i,        // PAIF TX 有效

    // =========================================================================
    // 反压信号（入口反压 / 出口反压统一汇集于此，两侧就绪同拍仲裁）
    // =========================================================================
    output wire                       paif_tx_ready_o,        // 入口反压：PAIF TX ready（1=可收 / 0=反压，反压上游 PAIF 入口）
    input  wire                       cpif_tx_ready_i,        // 出口反压：CPIF TX ready（0=反压 / 1=可收，所有 lane 共享）

    // =========================================================================
    // CSR 配置（apb_csr 解码后经顶层同步器送入，mb_clk 域）
    // =========================================================================
    input  wire                       csr_perf_mode_i,        // MODE_REG.WORK_MODE：1=性能 / 0=兼容
   input  wire                       csr_bridge_mode_i,      // public CSR bridge_en（顶层 bridge_en_i 经同步）；桥接下不 bypass、不直连，TX 走正常 Flit 流水线（link_tx_mas §3.1）
    input  wire                       csr_s2p_mode_i,         // PHY_MODE.s2p_mode：1=1024 全有效 / 0=低 512 有效
    input  wire [1:0]                 csr_bw_mode_i,          // BW_CFG_REG.bw_mode：00=全带宽 / 01=1/2 / 10=1/4
    input  wire                       csr_protocol_mode_i,    // MODE_REG.PROTOCOL_MODE：1=PAIF / 0=UDI
    input  wire                       csr_udi_ready_active_en_i,  // MODE_REG.UDI_READY_ACTIVE_EN：1=UDI ready 跟随 link_tx_active / 0=恒 1
    input  wire                       csr_retry_en_i,         // RETRY_CFG_REG.RETRY_EN
    input  wire                       csr_credit_en_i,        // RETRY_CFG_REG.CREDIT_EN
    input  wire                       csr_crc_en_i,           // RETRY_CFG_REG.CRC_EN（CRC0/CRC1 分时复用同一使能）
    input  wire                       csr_ecc_en_i,           // RETRY_CFG_REG.ECC_EN（性能模式；chk 槽组装与 crc16 互选）
    input  wire                       csr_inj_en_i,           // INJECT_ERR_LINKn_REG.INJECT_EN（per-TX 独立，电平门控 EN=1 即注错）
    input  wire                       csr_inj_fb_i,           // INJECT_ERR_LINKn_REG.FRONT_BACK：0=front(CRC0)/1=back(CRC1)（选半单元 payload bit0 注入）
    input  wire                       tx_flit_clr_i,
    input  wire [15:0]                csr_drift_period_i,     // DRIFT_CFG_REG.DRIFT_PERIOD（钟漂周期）
    input  wire [15:0]                csr_retry_timeout_val_i,// TIMEOUT_REG.RETRY_TIMEOUT
    input  wire [7:0]                 csr_retrain_timeout_thr_i,// RETRAIN_THR_REG.RETRAIN_TIMEOUT_THR
    input  wire [7:0]                 csr_retrain_nack_thr_i, // RETRAIN_THR_REG.RETRAIN_NACK_THR
    input  wire                       csr_retrain_timeout_en_i,// RETRAIN_THR_REG.RETRAIN_TIMEOUT_EN
    input  wire                       csr_retrain_nack_en_i,  // RETRAIN_THR_REG.RETRAIN_NACK_EN

    // =========================================================================
    // 反向通道（RX 侧 link_rx 经同步送入，mb_clk 域）
    // 解析通道（对端 → 本端 retry_ctrl）：ack_* 推进 retire_ptr / nack_* 触发重传
    // =========================================================================
    input  wire [7:0]                 ack_seq_i,              // 解析的 DLP ACK 序列号
    input  wire                       ack_valid_i,            // 解析 ACK 有效脉冲
    input  wire [7:0]                 nack_seq_i,             // 解析的 DLP NACK 序列号
    input  wire                       nack_valid_i,           // 解析 NACK 有效脉冲
    input  wire [1:0]                 nack_type_i,            // 解析 NACK 类型：2'b10=Standard / 2'b11=Select

    // =========================================================================
    // 生成通道（link_rx 本端接收进度 → DLP 嵌入回送对端）
    // =========================================================================
    input  wire [7:0]                 dlv_ack_seq_i,          // 交付 ACK 序列号（flit 送完 PAIF 后生成）
    input  wire                       dlv_ack_valid_i,        // 交付 ACK 有效脉冲
    input  wire [7:0]                 lcl_nack_seq_i,         // 本地 CRC 失败 NACK 序列号
    input  wire                       lcl_nack_valid_i,       // 本地 NACK 有效脉冲
    input  wire [1:0]                 lcl_nack_type_i,        // 本地 NACK 类型：2'b10=Standard / 2'b11=Select
    input  wire [1:0]                 sel_idx_i,              // link_ack_route 选中源 RX 索引（0~3）；嵌入 DLP 时回填 link_num 位（§4.2.6）

    // =========================================================================
    // ARQ 控制 Flit 中继 / 远端请求指示（link_rx 经同步送入，mb_clk 域）
    // =========================================================================
    input  wire [255:0]               arq_req_flit_i,         // link_rx 构造的 ARQ(Request) payload（SUB_TYPE[3:0]+Pointer[19:4]），Header/CHK 由本模块生成
    input  wire                       arq_req_valid_i,        // ARQ(Request) 有效（兼容模式 CRC 失败拍）
    input  wire [15:0]                arq_req_rcv_ptr_i,      // 收到远端 ARQ(Request) 的 Pointer（→ ARQ(ACK) Pointer）
    input  wire                       arq_req_rcv_valid_i,    // 远端 ARQ(Request) 有效（触发 ARQ(ACK) 应答）

    // =========================================================================
    // 链路编号（例化时编码传入：link0→3'd0、link1→3'd1 ...；写入性能模式 Flit
    // Header bit[3:1]，见 spec §4.1.1.1）
    // =========================================================================
    input  wire [2:0]                 link_number_i,          // 本链路编号（3 bit）

    // =========================================================================
    // UDI TX 接口（用户逻辑 → udi_bypass_mux）
    // =========================================================================
    input  wire [1023:0]              udi_tx_lane0_data_i,
    input  wire                       udi_tx_lane0_data_vld_i,
`ifdef LINK_LANE1
    input  wire [1023:0]              udi_tx_lane1_data_i,
    input  wire                       udi_tx_lane1_data_vld_i,
`endif
`ifdef LINK_LANE3
    input  wire [1023:0]              udi_tx_lane2_data_i,
    input  wire                       udi_tx_lane2_data_vld_i,
    input  wire [1023:0]              udi_tx_lane3_data_i,
    input  wire                       udi_tx_lane3_data_vld_i,
`endif
    output wire                       udi_tx_ready_o,         // ~proto_is_paif & (link_tx_active | ~csr_udi_ready_active_en_i)；AND-OR 等价值（rewrite 规则不用 ?:）（PROTOCOL_MODE=0 时生效）

    // =========================================================================
    // link_tx_lcmsm 状态机接口
    // =========================================================================
    input  wire [3:0]                 link_state_i,           // TX link FSM 状态（link_tx_lcmsm.link_state_o 经顶层同步，L1 §3.1 编码）：4'h3=S_LINK_TRAIN / 4'h4=S_NORMAL / 其他=非数据收发
    input  wire                       link_active_i,          // S_NORMAL 使能（link_tx_lcmsm.link_active_o，与 link_state_i 同拍，替代组合判等 link_state_i==4'h4）
    input  wire                       link_active_nxt_i,      // S_NORMAL 次态位（link_tx_lcmsm.link_active_nxt_o，F1a 同 D 副本寄存器源，与 link_active_i 同拍）
    output wire                       deskew_done_o,          // deskew done：训练 Pattern 发满 2 拍拉高 1 cycle（→ link_tx_lcmsm，状态机只接 done）

    // =========================================================================
    // 下游 CPIF TX 接口（link_tx → Logic PHY）
    // =========================================================================
    output wire [1023:0]              cpif_tx_lane0_data_o,
    output wire                       cpif_tx_lane0_data_vld_o,
`ifdef LINK_LANE1
    output wire [1023:0]              cpif_tx_lane1_data_o,
    output wire                       cpif_tx_lane1_data_vld_o,
`endif
`ifdef LINK_LANE3
    output wire [1023:0]              cpif_tx_lane2_data_o,
    output wire                       cpif_tx_lane2_data_vld_o,
    output wire [1023:0]              cpif_tx_lane3_data_o,
    output wire                       cpif_tx_lane3_data_vld_o,
`endif
    // =========================================================================
    // 状态 / 中断上报
    // =========================================================================
    output wire                       csr_replay_buf_full_o,          // replay buffer 满 = credit 耗尽
    output wire                       retrain_timeout_flag_o, // per-link TX 重训练超时 flag
    output wire                       retrain_nack_flag_o,    // per-link TX 重训练 NACK flag
    output wire                       irq_retry_timeout_o,    // per-link TX 重试超时 IRQ
    output wire                       irq_retry_nack_o,       // per-link TX 重试 NACK IRQ
    output wire [1:0]                 retrain_req_o,          // 重训练请求至 link_tx_lcmsm（bit0=超时 / bit1=NACK）
    output wire [15:0]                tx_flit_cnt_o,          // per-link TX 协议层 Flit 总数
    output wire [7:0]                 timeout_cnt_o          // per-link TX 超时重传次数

);

    // =========================================================================
    // 内部信号声明（wire 集中声明，与 assign 分离）
    // =========================================================================

    reg  [1:0]    paif_beat_cnt;          // PAIF 拍计数（性能 1024：0/1 / 512：0..3；兼容 0）
    reg  [7:0]    tx_seq;                 // 协议层 Flit seq 计数器（0~255 循环）
    reg  [15:0]   tx_flit_cnt;            // TX 协议层 Flit 总数
    reg  [1:0]    dlp_cmd_reg;            // 当前搭载 replay_cmd
    reg  [7:0]    dlp_seq_reg;            // 当前搭载 seq
    reg  [1:0]    dlp_linknum_reg;        // 当前搭载所指向的对端 link_num（= 源 RX 索引，route 的 sel_idx_i，嵌入 DLP bit[3:2]）
    reg  [9:0]    dlp_cmdseq_at_null_start; // 512：NULL 实发拍 {cmd,seq} 快照（D2 竞态修复：null_done 仅清未重载的 DLP）

    // ---- 内部 ICG 动态时钟门控（clken_tx 组合；tx_noidleQ/QQ 未门控 mb_clk 域展宽）----
    wire          ck_tx;                  // 内部 ICG 输出门控时钟（CK=mb_clk，相位对齐）
    wire          clken_tx;               // 内部 ICG 使能：~(dyn_en & n_dyn)|tx_active|tx_wake|tx_hold_2T
    wire          tx_active;              // 活动：link_tx_active|retry_active|null_active|arq_active|unit_busy_f
    wire          tx_wake;                // 唤醒：paif_tx_valid_i|arq_req_valid_i|arq_req_rcv_valid_i
    wire          tx_idle;                // 组合判 idle（L1 §4.10.3 (a)(b)(c) 全不拉高）
    wire          tx_hold_2T;             // idle 展宽 2T：~tx_idle|tx_noidleQ|tx_noidleQQ
    wire          arq_active;             // 兼容 NULL/ARQ 在发（ifdef=ctrl_busy_compat / else=0）
    reg           tx_noidleQ;             // 未门控 mb_clk 域展宽 DFF（门控关断前须仍运转）
    reg           tx_noidleQQ;


    // ---- flit_sched：PAIF 累积 / Flit 组装 / beat 计数 ---------------------------------
    wire          paif_rx_stall;          // PAIF 反压（_stall 后缀，控制源：retry/arq/UDI）
    wire          paif_ready_ctl;         // PAIF 控制侧可收（~retry_active & ~arq_active & proto）
    wire          paif_ready_conv;        // 当前例化形态入口可收（形态 paif_tx_ready_*）
    wire          paif_rx_accept;         // 本拍接收 PAIF 数据（valid & 控制可收 & 形态可收）
    wire          is_first_beat;          // 本拍为 Flit 首拍
    wire          is_last_beat;           // 本拍为 Flit 末拍
    `ifdef LINK_TX_COMPAT_MODE
    wire [271:0]  comp_flit_assembled;    // 兼容模式组装 Flit（272 bit，CRC9 未算；兼容形态内嵌算）
    `endif
    wire          flit_complete;          // 完整 Flit 收齐脉冲（数据 Flit，控制侧）
    wire          front_proc;             // 本拍完成前端 128 B 单元（t=CRC0+Header）
    wire          back_proc;              // 本拍完成后端 128 B 单元（DLP+CRC1）
    `ifndef PAIF_INTF_512
    wire          compat_sum;             // 兼容模式整 Flit 收齐（单实现，CRC9；512 形态孤儿：读者在非 512 exit_compat / 兼容形态）
    `endif
    wire [3:0]    hdr_type_linknum;       // Header bit[3:0]：{link_num, type=1}（数据/重传）
    wire [3:0]    hdr_type_linknum_ctrl;  // Header bit[3:0]：{link_num, type=0}（控制/NULL，gdx 2026-09-15）
`ifdef PAIF_INTF_512
    reg  [511:0]  paif_accum_lo;          // 512 bit 单元前半拍累积（unit 高半）
    reg  [2047:0] paif_perf_full;         // 整 Flit 原始 2048 bit 累积（replay 用，无 CRC/DLP）
`else
    reg  [1023:0] perf_front_half_reg;    // 前端 128 B 单元原始数据寄存（replay 组装用）
`endif

    // ---- replay_buffer 接口（数据源来自当前例化形态的 replay_push_*，父模块转发） ----
    wire          replay_buf_wen;
    wire [`REPLAY_BUF_PTR-1:0] replay_buf_wptr;
    wire [2047:0] replay_buf_flit_in;
    wire          replay_buf_ren;
    wire [`REPLAY_BUF_PTR-1:0] replay_buf_rptr;
    wire          replay_buf_ack_valid;
    wire [`REPLAY_BUF_PTR-1:0] replay_buf_ack_ptr;
    wire          replay_buf_full;
    `ifndef PAIF_INTF_512
    wire [2047:0] replay_buf_flit_out;          // legacy flit 读回（1024/兼容形态；512 形态 D9 恒 0 无消费者）
    wire          replay_buf_flit_out_valid;
    `endif

    // ---- dlp_update / CRC 路径 ---------------------------------------------------------
    wire [15:0]   dlp_field;              // 组合嵌入后的 DLP 字段（Byte 128~129）
    wire [1023:0] crc16_data;             // CRC16 引擎输入（各性能形态 crc_data_*_o 上送，已嵌字段且 CRC 槽清 0）
    wire [1023:0] ecc16_data;             // ECC 引擎输入（与 crc16_data 同数据源，纯组合 0 cycle）
    wire [11:0]   ecc16_result;           // ECC 结果（ecc_o[11:0] = {P, tie0, SEC[9:0]}，bit 11 = 整体奇偶位 P，LL-20260923-CHK1008）
    wire [15:0]   crc16_result;           // CRC16 结果
    // ---- 注错（payload bit0/bit1 逐位使能；NULL/ARQ 控制 Flit 不注错，仅数据/重传）-----
    //    注错已下沉至各性能形态模块（crc_src_inj），父侧仅透传 csr_inj_* 至形态。

    // ---- 出口数据源注入（正常 / 重传 / 控制共用 dlp_update+CRC，MAS §4.4 / §5.8）-----
    wire [1:0]    exit_src;               // 2'b00=DATA / 2'b01=RETRY / 2'b10=CTRL
    `ifndef PAIF_INTF_512
    wire          exit_front;             // 单元装载·前端（DATA=front_proc / RETRY=retry front / CTRL=perf NULL front）
    wire          exit_back;              // 单元装载·后端（512 形态锚点已迁 front_proc/back_proc/retrans_*，未用）
    wire          exit_compat;            // 兼容整 Flit 装载（DATA=compat_sum / CTRL=compat NULL）
    `endif
    wire [15:0]   exit_hdr;               // 前端 Header[15:0]：DATA=tx_seq / RETRY=内嵌 / CTRL=seq 8'hff

    // ---- bridge 下 retry/credit 等效关闭（与 link_rx.v:242 对称 gate；bridge=1 → 无重传 / 无 credit 追踪）-----
    wire          retry_en_eff  = csr_retry_en_i  & ~csr_bridge_mode_i;
    wire          credit_en_eff = csr_credit_en_i & ~csr_bridge_mode_i;
    // ---- 出口/装载事件（rtl_coder 误删恢复）-----
    `ifndef PAIF_INTF_512
    wire          conv_accept;             // 当前例化形态可装载（= paif_ready_conv；512 形态调度器不消费）
    `endif
    wire          flit_sent_event;         // 新 Flit 进入事件（赋值在 timeout_monitor 698）
`ifdef PAIF_INTF_512
    // ---- v1.21 重传行流瘦化（`PAIF_INTF_512 形态①；MAS link_tx_mas v1.21 §4.10 / D1：父仅给
    //      起始事件，buffer 自走行读；tick 经父转接 child→buffer）----
    wire [7:0]    retrans_len_c;         // 窗口剩余 flit 数（stop−rd_seq 模 256；retrans_start 拍 = 全窗口）
    reg           retrans_started;       // retrans_start 已发出（窗口期防重复发 start——child 泵首拍未忙，M4b）
    wire          retrans_start;         // 重传起始单事件（retry_active & ~retrans_started & len≠0；C7 Fix1 删泵空闲门控——依据见 :698-705）
    wire          retrans_done;          // 第 len 个 retrans_flit_done（窗口完成）
    wire          retrans_empty_clear;   // len==0 空窗 guard（新 corner：retry_enter 锁存 start==stop → 1 拍清 retry_active）
    wire          retrans_tick;          // child 泵 LO advance tick（→ buffer retrans_tick_i 行读调度，经父转接）
    wire          retrans_flit_done;     // child 每重传 flit 末拍实发脉冲（→ 影子 rd_seq+1 / 统计 / 第 len 个停止）
    wire          retrans_line_ready;    // buffer 行就绪（→ child retrans_line_ready_i）
    wire [1023:0] retrans_line_data;     // buffer 行流数据（→ child retrans_line_data_i；front 行原样 / back 行 child 内 DLP 替换）
    wire          null_done;             // child NULL 4 拍流末拍实发（清 null_active，D2）
    wire          null_flit_start_f;     // NULL 起始单事件（null_start 公式 & 泵空闲，D2）
    // ---- 形态① replay 行流 push 交接（child MAS v0.1 37 口契约；2026-09-24 同步） ----
    wire [1:0]    replay_push_wen_512_f;   // child wen = {push_back(T4), push_front(T2)}：每 flit 2 次 push，同行索引
    wire [3:0]    replay_push_wptr_512_f;  // child wptr 4 位纯行索引（T4++ 回绕 15→0）
    reg           replay_wptr_flag_r;      // 16 回绕 flag（buffer wptr_i[4]；full_match/occupancy 消费；wptr 15→0 翻转）
`endif

    // ---- retry_ctrl / timeout_monitor / ctrl_flit_gen / 出口注入（性能重传 + 兼容 ARQ 握手）-----
    reg           retry_active;           // 重传进行中（反压 PAIF 入口，MAS §6.4 条件 2）
    wire          retry_trigger_perf;     // 性能重传触发（NACK / 超时）
    wire          retry_trigger_comp;     // 兼容重传触发（收到远端 ARQ(Request)）
    wire          retry_trigger;          // retry_trigger_perf | retry_trigger_comp
    wire [7:0]    retry_init_seq;         // 重传起始序列号（perf=NACK seq / 超时 retire / compat=ARQ Pointer）
    wire [7:0]    retry_init_stop;        // 重传终止序列号（perf=tx_seq / compat=tx_seq 写指针）
    `ifndef PAIF_INTF_512
    wire          retry_front_event;      // 性能重传前端装载事件（与 exit_front 合并进装载）
    wire          retry_back_event;       // 性能重传后端装载事件
    wire          retry_compat_event;     // 兼容重传单装载事件（加入 exit_compat，非 front/back）
    wire          retry_read_issue;       // 读请求（replay_buf_ren）
    `endif
    `ifndef PAIF_INTF_512
    wire          retry_done;             // 重传终止（末拍装载后下一读序列号追上 stop；512 形态用 retrans_done）
    `endif
    reg  [7:0]    retry_rd_seq;           // 重传读序列号（replay rptr 的 8 bit seq）
    `ifdef PAIF_INTF_512
    // ---- BUG-RB1a（LL-20261010-R2G1 B 票：重传窗 X 读家族 c24/c8；arch LLArchG1B 通过）----
    //     push_done_seq = 最后完全写（back 单元 push 完成）flit seq；动态 stop = min_mod256(tx_seq,
    //     push_done_seq+1)——子管 push 未齐时窗截断（X 窗消除），常态 d≡0 零回归（NACK/超时触发时
    //     push 早已完成）；空窗由 retrans_empty_clear 承接；1024/兼容 = 单 wen/accept 同拍 d 恒 0
    //     （行为零变化）；stop_clamp_c 于 retry_stop_seq 动态支 512 形态引用。
    reg  [7:0]    push_done_seq;           // 最后完全写入 flit seq（复位 8'hFF=-1，watermark=+1=pushed 数）
    wire [7:0]    push_done_plus1;         // push_done_seq + 8'd1（模 256 水位）
    wire [7:0]    stop_dist;               // tx_seq - (push_done_seq+1) 8-bit 模距（push 落后 ≤ DEPTH≪128，半模内无歧义）
    wire          stop_eq0;                // stop_dist == 0（& | 形态，rtl-coding 禁 ?:）
    wire [7:0]    stop_clamp_c;            // 动态 stop 钳制 = min_mod256(tx_seq, push_done_seq+1)
    `endif
    `ifndef PAIF_INTF_512
    reg           retry_flit_loaded;      // 当前重传 Flit 已捕获（后续切 front/back 处理）
    reg           retry_half;             // 当前重传 Flit 处理半：0=front / 1=back
    reg  [2047:0] retry_flit;             // 捕获的重传 Flit 原始 2048 bit（无 CRC/DLP；512 形态删——
                                          //   legacy flit_out 恒 0 不锁存且无消费者，冗余 pipe，2026-09-24 清理）
    `endif
    reg  [7:0]    retry_cnt;              // 当前重传次数（内部计数，仅阈值比较）
    `ifndef PAIF_INTF_512
    reg           retry_rd_busy;          // 重传读在途（ren 发出至 flit 捕获）
    `endif
    reg  [7:0]    retry_stop_seq;         // 重传终止序列号（重传起 至 前一个=stop 停止）
    reg  [7:0]    retire_seq;             // ACK 推进的 retire 序列号镜像（= 最早未确认 seq）
    reg  [15:0]   timeout_cnt;            // oldest_unacked 计时器
    wire          timeout_run;            // 窗口非空（retire_seq != tx_seq），计时使能
    wire          timeout_event;          // 超时脉冲（送 retry_ctrl）
    reg  [7:0]    retrain_timeout_cnt;
    reg  [7:0]    retrain_nack_cnt;
    reg           retrain_timeout_flag;
    reg           retrain_nack_flag;
    reg           retrain_timeout_flag_d; // flag 打拍（供 irq 沿脉冲）
    reg           retrain_nack_flag_d;
    wire          retrain_timeout_thr;    // 超时重传失败计数达阈值
    wire          retrain_nack_thr;       // NACK 重传失败计数达阈值
    wire          null_idle_ok;           // NULL 通路空闲（可非抢占插入）
    wire          null_start;             // NULL 非抢占插入启动
    `ifndef PAIF_INTF_512
    wire          null_front_event;       // 性能 NULL 前端装载事件
    wire          null_back_event;        // 性能 NULL 后端装载事件
    `endif
    reg           null_active;            // 性能 NULL 正在发送（非抢占：无 PAIF Flit 可搭载 ACK 时）
    `ifndef PAIF_INTF_512
    reg           null_half;              // 性能 NULL 处理半：0=front / 1=back
    wire [2047:0] ctrl_perf_null;         // 性能 NULL（256B，Header seq=8'hff，DLP 承载 ACK/NACK）
    `endif
    wire          ack_advance_event;      // ACK 推进（retry_ctrl 有效 ACK 推进 retire）
`ifdef LINK_TX_COMPAT_MODE
    // 兼容模式 ARQ/NULL flit 生成迁入 link_tx_compat（gdx_review Item 8）：父仅例化透传。
    // 父 → 兼容形态：兼容 NULL 生成请求 / 重传 Flit 装载 / 兼容重传完成（→ ARQ(Done)）
    wire          null_gen_req;           // 兼容 NULL 生成请求（父：ACK/NACK 待发等 → link_tx_compat）
    wire [271:0]  retry_flit_compat;      // 兼容重传 Flit（父 replay 读回 → link_tx_compat）
    wire          retry_flit_valid_compat;// 兼容重传单装载拍（= retry_compat_event）
    wire          retrans_done_compat;    // 兼容重传序列完成（→ ARQ(Done) 生成请求）
    // 兼容形态 → 父：NULL/ARQ 控制在发 / 装载事件
    wire          ctrl_busy_compat;       // 兼容 NULL/ARQ 在发（并入 paif_ready_ctl / retry_enter 反压）
    wire          ctrl_event_compat;      // 兼容 NULL/ARQ 装载事件拍（并入 exit_compat / flit_sent）
`endif

    // ---- udi / bridge ----------------------------------------------------------------
    wire          proto_is_paif;          // 1=PAIF（conv 通路）/ 0=UDI 直通

    // ---- 当前例化形态交接 canonical 线（由下方形态例化区驱动/消费，宏选择） ----
    // 形态 → 父：入口 ready / CRC 上送 / replay push / CPIF lane 输出
    wire          conv_ready_active;      // 当前例化形态入口可收（paif_tx_ready_*）
    wire [1023:0] crc_data_active;        // 当前例化形态 CRC 上送 128 B 单元（已嵌字段且 CRC 槽清 0；兼容形态不挂连置 0）
    wire          replay_push_wen_f;      // 当前例化形态 replay push 写使能（数据 Flit 收满拍）
    wire [`REPLAY_BUF_PTR-1:0] replay_push_wptr_f;  // 当前例化形态 replay push 写指针
    wire [2047:0] replay_push_flit_f;     // 当前例化形态 replay push 原始 Flit（兼容形态低 272 bit 有效）
    wire [1023:0] cpif_paif_lane0_data;   // 当前例化形态 CPIF lane0 数据（PAIF 通路 → udi_bypass_mux）
    wire          cpif_paif_lane0_vld;    // 当前例化形态 CPIF lane0 有效
`ifdef LINK_LANE1
    wire [1023:0] cpif_paif_lane1_data;   // 当前例化形态 CPIF lane1 数据（2 lane 形态）
    wire          cpif_paif_lane1_vld;    // 当前例化形态 CPIF lane1 有效
`endif
    // 父 → 形态：字段自嵌驱动（header/DLP/CRC 校验码 + NULL/重传 数据&front）/ NULL/重传注入
    wire          unit_busy_f;            // 当前例化形态单元待发/在途（已装载未发完，门控 NULL/重传装载）
    wire [260:0]  compat_flit_data_f;     // 兼容模式父组装 Header+Payload（261 bit）→ 兼容形态
    wire          compat_flit_valid_f;    // 兼容模式父组装有效 → 兼容形态
    `ifndef PAIF_INTF_512
    reg  [1023:0] udi_pipe_lane0_data;    // lane0 UDI 入口 pipe 数据（较原始输入延迟 1T）
    reg           udi_pipe_lane0_vld;     // lane0 UDI 入口 pipe 有效
    `endif
`ifdef LINK_LANE1
    reg  [1023:0] udi_pipe_lane1_data;
    reg           udi_pipe_lane1_vld;
`endif
`ifdef LINK_LANE3
    reg  [1023:0] udi_pipe_lane2_data;
    reg           udi_pipe_lane2_vld;
    reg  [1023:0] udi_pipe_lane3_data;
    reg           udi_pipe_lane3_vld;
`endif

    // ---- 状态派生 ----------------------------------------------------------------------
    wire          link_tx_active;         // S_NORMAL（4'h4）：正常收发
    reg           link_active_cp;         // F1a：link_active 同 D 副本寄存器（≤ link_active_nxt_i，mb_clk 无条件寄存，无复位 X-safe）

    // =====================================================================================
    // PAIF 接收 / beat 计数（flit_sched 控制侧；PAIF↔CPIF 拆分与 CPIF 输出由当前例化
    // 形态模块承担，父模块 beat 仅用于控制：seq 分配 / replay 写 / CRC 调度 / 注入仲裁）
    // =====================================================================================
    // 控制侧可收（retry / ARQ 抢占 / UDI 模式时反压 PAIF 入口）
    // NULL 为非抢占控制 Flit（MAS §6.5）：TX 在发 NULL（承载 ACK/NACK）期间不得再
    //   accept PAIF 数据拍，否则下一数据 Flit 的 4 拍在 NULL 输出期间被 front/back 计算
    //   但被 NULL 抢占丢弃 → 单元错位 → 对端 CRC 失败（BUG_LOG 2026-09-02：flit8 过渡
    //   null-vs-data 仲裁竞态修复）。故 paif_ready_ctl 增加 ~null_active 反压。
`ifdef LINK_TX_COMPAT_MODE
   assign paif_ready_ctl = csr_protocol_mode_i & ~retry_active & ~ctrl_busy_compat & ~null_active;
`else
   assign paif_ready_ctl = csr_protocol_mode_i & ~retry_active & ~null_active;
`endif
    // 形态侧可收 = 当前例化形态入口 ready（宏选择，见下方形态例化区）
    // 形态侧可收由「当前例化形态」的入口 ready 驱动（见下方形态例化区，宏选择），
    // 此处定义的 canonical 线 paif_ready_conv 由激活形态实例赋值。
    assign paif_ready_conv = conv_ready_active;
    // 入口反压：控制侧 & 形态侧 均就绪才可收。
    //   RX_FIFO 半字写/读、deep 对齐 golden §4.3.3.4/§4.4.3：对端 RX credit_fifo 逻辑深度
    //   已修正为 64 flit（= replay buffer 深度），read/retire 均为单调封顶的 credit 指针，
    //   故 RX 反压完全由 replay_buf_full（credit 耗尽）及合法的 ready/conv 反压驱动，
    //   不再需要任何硬编码在途上限（原 RX_FLIT_CAP=30 的 band-aid 已于 2026-09-04 撤除）。
    assign paif_rx_stall   = ~paif_ready_ctl | ~paif_ready_conv | replay_buf_full;
    assign paif_rx_accept  = paif_tx_valid_i & ~paif_rx_stall;
    assign paif_tx_ready_o = ~paif_rx_stall;

    // 单元判定（PAIF 位宽 1024 / 512 代码完全隔离，仅由单一 `ifdef PAIF_INTF_512 控制）
    `ifdef PAIF_INTF_512
        // ---------------- 512 bit PAIF 路径（与 1024 完全隔离） ----------------
        assign is_last_beat  = (paif_beat_cnt == 2'd3) | ~csr_perf_mode_i;
        // 512 形态父侧不再拼装 128 B 单元/replay 累积：unit 拼装与 replay push 已下沉至
        //   形态模块内部（paif_data_rg 1T 缓冲 + 自嵌 header/DLP/CRC，自产 replay push）。
        assign front_proc = csr_perf_mode_i & paif_rx_accept & (paif_beat_cnt == 2'd1);
        assign back_proc  = csr_perf_mode_i & paif_rx_accept & (paif_beat_cnt == 2'd3);
        // 512 形态 CRC front/back 选择已下沉至形态模块内部（crc_front_512_lane1_o 自嵌
        //   header/DLP+CRC），父侧不再组装——crc16_sel_front/crc_front_active 仅 1024 系使用。
    `else
        // ---------------- 1024 bit PAIF 路径（与 512 完全隔离） ----------------
        assign is_first_beat = (paif_beat_cnt == 2'd0);
        assign is_last_beat  = (paif_beat_cnt == 2'd1) | ~csr_perf_mode_i;
        always @(posedge ck_tx) begin
            if (!mb_rst_n)
                perf_front_half_reg <= 1024'd0;
            else if (paif_rx_accept && is_first_beat)
                perf_front_half_reg <= paif_tx_data_i;
        end
        assign front_proc = csr_perf_mode_i & paif_rx_accept & is_first_beat;
        assign back_proc  = csr_perf_mode_i & paif_rx_accept & is_last_beat;
        // CRC front/back 选择已下沉至各性能形态模块内部（crc_front_sel），父侧不再组装。
    `endif

    // PAIF 拍计数器（两种位宽共用同一计数器逻辑，is_last_beat 由上方 ifdef 分支决定）
    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            paif_beat_cnt <= 2'd0;
        else if (paif_rx_accept && is_last_beat)
            paif_beat_cnt <= 2'd0;
        else if (paif_rx_accept)
            paif_beat_cnt <= paif_beat_cnt + 2'd1;
    end

    `ifdef LINK_TX_COMPAT_MODE
    assign comp_flit_assembled[4:0]     = {1'b0, 2'b01, 1'b0, 1'b1};
    assign comp_flit_assembled[260:5]   = paif_tx_data_i[255:0];
    assign comp_flit_assembled[271:261] = 11'd0;
    `endif

    assign flit_complete  = paif_rx_accept & is_last_beat;
    `ifndef PAIF_INTF_512
    assign compat_sum     = ~csr_perf_mode_i & flit_complete;
    `endif

    // =========================================================================
    // tx_seq_mgr：seq 分配（PAIF 数据 Flit 每个完整 Flit 递增 1；NULL/ARQ 不占）
    // =========================================================================
    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            tx_seq <= 8'd0;
        else if (flit_complete)
            tx_seq <= tx_seq + 8'd1;
    end

    // =========================================================================
    // dlp_update：DLP 组合嵌入（无采样点无锁存，仅 assign）
    // =========================================================================
    always @(posedge ck_tx) begin
        if (!mb_rst_n) begin
            dlp_cmd_reg      <= 2'b00;
            dlp_seq_reg      <= 8'd0;
            dlp_linknum_reg  <= 2'd0;
        end
        else if (lcl_nack_valid_i) begin
            dlp_cmd_reg      <= lcl_nack_type_i;
            dlp_seq_reg      <= lcl_nack_seq_i;
            dlp_linknum_reg  <= sel_idx_i;
        end
        else if (dlv_ack_valid_i) begin
            dlp_cmd_reg      <= 2'b01;
            dlp_seq_reg      <= dlv_ack_seq_i;
            dlp_linknum_reg  <= sel_idx_i;
        end
        // v1.21 形态①（D2）：DLP 清除锚点迁移——重传/NULL 不经父 exit_back（child 自走），
        //   DLP 实发拍 = 数据 back_proc（T4）/ 重传 back 行（child T4，retrans_flit_done 晚 1 拍
        //   回报）/ NULL back（child 4 拍流末拍，null_done 晚 1 拍回报）；重传/NULL 窗口 PAIF 入口
        //   反压（paif_ready_ctl ~retry_active/~null_active）无 back_proc 交错；1024/兼容形态保持
        //   原 exit_back 同拍语义。2026-09-24 竞态修复：null_done 仅清「未重载」的 DLP——新 DLP
        //   （dlv_ack/nack）载入后、上一 NULL 的 null_done 晚 1-2 拍到达会误清新 DLP（peer ACK
        //   丢失 → 误重传 → 对端重复交付；T13 ack(99) 丢失根因）；判据 = DLP {cmd,seq} 自 NULL
        //   实发拍（null_flit_start_f）以来未变（快照 dlp_cmdseq_at_null_start），变 = 新命令
        //   待下一 NULL 搭载（cmd 或 seq 任一变化均判为重载，防同 seq 跨 cmd 误清）。
        `ifdef PAIF_INTF_512
        else if (back_proc | retrans_flit_done
                | (null_done & ({dlp_cmd_reg, dlp_seq_reg} == dlp_cmdseq_at_null_start))) begin
        `else
         else if (exit_back) begin
        `endif
            dlp_cmd_reg      <= 2'b00;
            dlp_seq_reg      <= 8'd0;
        end
    end

    // v1.21 形态①（2026-09-24 竞态修复配套）：NULL 实发拍 DLP {cmd,seq} 快照
    `ifdef PAIF_INTF_512
    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            dlp_cmdseq_at_null_start <= 10'd0;
        else if (null_flit_start_f)
            dlp_cmdseq_at_null_start <= {dlp_cmd_reg, dlp_seq_reg};
    end
    `endif

    assign dlp_field = {4'b0000, dlp_seq_reg, dlp_linknum_reg, dlp_cmd_reg};

    // Header bit[3:0]：{link_num（本 TX 发送链路编号）, type=1}，由 link_number_i 驱动
    // （修正原先 hdr_type_linknum 无驱动；Header link_num 标识本 TX 发送链路，见 §4.1.1.1）
    // gdx_review 2026-09-15（golden=spec/link_layer_spec.md §4.1.1.1/§4.1.3）：NULL 是控制
    //   Flit（type=0）。原 NULL 与控制共用 hdr_type_linknum（type=1）→ 线上 NULL 看起来像数据；
    //   现控制/NULL 用 type=0 变体（hdr_type_linknum_ctrl），数据/重传保持 type=1。收端 NULL
    //   判据维持 seq==8'hff（designer 终裁，以下详见 gdx_review.md Item4）。
    assign hdr_type_linknum      = {link_number_i, 1'b1};
    assign hdr_type_linknum_ctrl = {link_number_i, 1'b0};

    // =========================================================================
    // retry_ctrl：NACK / 超时 / ARQ(Request) 触发 + replay 读侧驱动 + ACK 退订 + retrain 计数
    // =========================================================================
    // retry 触发须限定在未确认窗口 [retire_seq, tx_seq) 内：
    //  spec §4.4.2.2-8  nack_seq < unack_seq(=retire_seq) → 重复 NACK，丢弃；
    //  nack_seq 亦不得指向未发送 Flit（>= tx_seq）。
    //  否则会重传已 retire(已释放) 的 replay buffer 条目 → rd 冻结、retry_active 卡死。
    assign retry_trigger_perf = ~retry_active & retry_en_eff & proto_is_paif
                                & csr_perf_mode_i
                                & ( (nack_valid_i & (nack_seq_i >= retire_seq)
                                                  & (nack_seq_i <  tx_seq))
                                  | timeout_event );
    assign retry_trigger_comp = ~retry_active & retry_en_eff & proto_is_paif
                                & ~csr_perf_mode_i & arq_req_rcv_valid_i;
    assign retry_trigger      = retry_trigger_perf | retry_trigger_comp;
    assign retry_init_seq  = {8{csr_perf_mode_i}} & ( {8{nack_valid_i       }} & nack_seq_i
                                             | {8{timeout_event      }} & retire_seq
                                             | {8{~nack_valid_i & ~timeout_event}} & 8'd0 )
                          | {8{~csr_perf_mode_i}} & arq_req_rcv_ptr_i[7:0];
    assign retry_init_stop = {8{csr_perf_mode_i}} & ( {8{nack_valid_i & (nack_type_i == 2'b11)}} & (nack_seq_i + 8'd1)
                                             | {8{nack_valid_i & (nack_type_i != 2'b11)}} & tx_seq
                                             | {8{timeout_event & ~nack_valid_i        }} & tx_seq
                                             | {8{~nack_valid_i & ~timeout_event       }} & 8'd0 )
                          | {8{~csr_perf_mode_i}} & tx_seq;
    // =========================================================================
    // retry 进入边界化（BUGFIX 2026-09-11）
    //   retry_active 不再在 retry_trigger（NACK/超时）当拍直接置位：若触发恰逢数据 Flit
    //   中途（前端 beat1 已装载上线、后端 beat2/3 未收满），直接切断 PAIF 会令该 Flit 的
    //   前端成为 孤 front（只发前 128B、无后 128B），线上 front/back 奇偶错位（出现 F,F,B）。
    //   对端 RX 按 unit_idx 奇偶递推 front/back，奇偶翻转后重传流以错误相位到达 → 数据错乱
    //   （iter_10 CASE_8/t21 实证：NACK 已指向出错 Flit、重传窗口正确，但 flit12 孤 front
    //   翻转位相导致重传 flit10 front/back 被 RX 错播为 back/front）。
    //   改为：触发拍只锁存 retry_pending/init/stop，待 paif_beat_cnt 回 0（整 Flit 已收齐的
    //   边界拍、且在途 NULL/ARQ 排空）才置 retry_active——被排空的 Flit 以完整 F/B 上线，
    //   动态 stop（Standard NACK/超时/兼容 = tx_seq）取进入拍最新 tx_seq，把该 Flit 一并纳入
    //   重传窗口（B 侧 err_discard 丢弃窗口内全部 Flit，重传即可补齐，无 flit 丢失）。
    // -------------------------------------------------------------------------
    reg           retry_pending;          // 重传已触发、等待 PAIF Flit 边界进入
    reg  [7:0]    retry_init_lat;         // 重传起始 seq 锁存（触发拍采样；进入拍 nack 已无效）
    reg  [7:0]    retry_stop_lat;         // 重传终止 seq 锁存（固定 stop 型：Select NACK=seq+1）
    reg           retry_stop_dyn;         // 该次 stop 是否“动态 tx_seq”（Standard NACK/超时/兼容）
    wire          retry_enter;            // 边界拍进入重传（retry_pending & 边界 & 非 NULL/ARQ）

`ifdef LINK_TX_COMPAT_MODE
    assign retry_enter = retry_pending & ~null_active & ~ctrl_busy_compat
                       & (paif_beat_cnt == 2'd0);
`else
    assign retry_enter = retry_pending & ~null_active
                       & (paif_beat_cnt == 2'd0);
`endif

    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            retry_pending <= 1'b0;
        else if (retry_trigger)
            retry_pending <= 1'b1;
        else if (retry_enter)
            retry_pending <= 1'b0;
    end

    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            retry_init_lat <= 8'd0;
        else if (retry_trigger)
            retry_init_lat <= retry_init_seq;
    end

    always @(posedge ck_tx) begin
        if (!mb_rst_n) begin
            retry_stop_lat <= 8'd0;
            retry_stop_dyn <= 1'b0;
        end
        else if (retry_trigger) begin
            retry_stop_lat <= retry_init_stop;
            retry_stop_dyn <= ~(csr_perf_mode_i & nack_valid_i & (nack_type_i == 2'b11));
        end
    end
    // 读请求：（1024/兼容形态）有未重传完 Flit 且读不在途、当前 Flit 已送完才发下一条（perf/compat 共用；
    // conv_accept 以当前例化形态入口 ready 为准——形态可装载即发读，避免组合环）
    `ifndef PAIF_INTF_512
    assign retry_read_issue = retry_active & ~retry_rd_busy & ~retry_flit_loaded
                              & (retry_rd_seq != retry_stop_seq) & conv_accept;
    assign retry_front_event = retry_active & csr_perf_mode_i & retry_flit_loaded & ~retry_half & conv_accept & ~unit_busy_f;
    assign retry_back_event  = retry_active & csr_perf_mode_i & retry_flit_loaded & retry_half & conv_accept & ~unit_busy_f;
    assign retry_compat_event= retry_active & ~csr_perf_mode_i & retry_flit_loaded & conv_accept;
    `endif
    `ifndef PAIF_INTF_512
    assign retry_done       = (retry_rd_seq == retry_stop_seq - 8'd1);
    `endif

    `ifdef PAIF_INTF_512
    // ---- v1.21 形态① 行流瘦化（MAS link_tx_mas v1.21 §4.10 / D1；arch M3/M4；buffer v1.8 自走）----
    // 父仅给起始事件（retrans_start 单事件 + 窗口头 rptr（replay_buf_rptr）+ 窗口 len）；buffer
    //   自走行读（tick = child 泵 LO advance，经父转接）；retry_rd_seq 降为影子计数器
    //   （retrans_flit_done+1；停止决策 / 剩余窗口计数 / 统计），不再驱动 buffer ren/rptr 逐读。
    assign retrans_len_c = retry_stop_seq - retry_rd_seq;   // 8 bit 模 256 减法 = 剩余窗口 flit 数（retrans_start 拍 = 全窗口）
    // len==0 空窗 guard（新 corner，现网无显式处理）：retry_enter 边界拍可锁存 start==stop
    //   （窗口内 Flit 全部已被 ACK retire）→ 不发 retrans_start，1 拍后清 retry_active（空窗重传完成）。
    assign retrans_empty_clear = retry_active & (retrans_len_c == 8'd0);
    // retrans_start 单事件（retry_active & Flit 边界 & start≠stop）：Flit 边界由 retry_enter
    //   （paif_beat_cnt==0 边界拍锁存，BUGFIX 2026-09-11）+ retry_active 期 PAIF 入口反压（paif_ready_ctl）
    //   保证；retrans_started 防重复发 start——child 泵 start 脉冲后首拍未忙（retrans_tick 在 start+2），
    //   无本 guard 会重复发 start 致 buffer 重复 priming 读（M4b）。
    // C7 重传 livelock 修复 Fix1（LL-20260925-CRIFVO-C7 §6）：删 ~unit_busy_f 门控——
    //   retry_enter 落于数据边界拍时子管冻结（拍号仅 load 拍前进，cnt 卡 1，paif_clear
    //   =send&cnt==0 永不可达）使 unit_busy_f 反压恒 1，retrans_start 恒 0 → retry_active
    //   永不能自清（三环互锁）。retrans_started（M4b）已担 start 脉冲后重复发防护；
    //   retry_active 期间入口反压杜绝新 load 竞争——空闲门冗余且有害。start 拍子管内
    //   在途 beat 由子 Fix2（retrans_start_i 清 0 臂）同拍排空；子四路出口 onehot0
    //   不变量保持（start 拍 pv 清、次拍仅 retrans_active_r 一路 pending）；被排空行
    //   已 push 入 replay（子 :292-294），无数据丢失。
    assign retrans_start = retry_active & ~retrans_started & (retrans_len_c != 8'd0);
    // 第 len 个 retrans_flit_done：剩余计数 = 1 当拍（retrans_flit_done 后沿 rd_seq+1 → len_c-1）
    assign retrans_done  = retrans_flit_done & (retrans_len_c == 8'd1);
    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            retrans_started <= 1'b0;
        else if (retrans_done | retrans_empty_clear)
            retrans_started <= 1'b0;
        else if (retrans_start)
            retrans_started <= 1'b1;
    end
    `endif

    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            retry_active <= 1'b0;
        `ifdef PAIF_INTF_512
        // v1.21 形态①：第 len 个 retrans_flit_done 或空窗 guard 清（retrans_done / retrans_empty_clear
        //   均含 retry_active，互斥；retry_enter 在 retry_active=0 拍，无同拍冲突）
        else if (retrans_done | retrans_empty_clear)
            retry_active <= 1'b0;
        `else
        else if ((csr_perf_mode_i & retry_back_event  & retry_done)
               | (~csr_perf_mode_i & retry_compat_event & retry_done))
            retry_active <= 1'b0;
        `endif
        else if (retry_enter)
            retry_active <= 1'b1;
    end

    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            retry_rd_seq <= 8'd0;
        else if (retry_enter)
            retry_rd_seq <= retry_init_lat;
        `ifdef PAIF_INTF_512
        // 影子读序列号（v1.21 形态①：每 retrans_flit_done +1，不再驱动 buffer ren/rptr 逐读）
        else if (retrans_flit_done)
            retry_rd_seq <= retry_rd_seq + 8'd1;
        `else
        else if (retry_back_event | retry_compat_event)
            retry_rd_seq <= retry_rd_seq + 8'd1;
        `endif
    end

    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            retry_stop_seq <= 8'd0;
        else if (retry_enter)
            `ifdef PAIF_INTF_512
            retry_stop_seq <= {8{retry_stop_dyn}} & stop_clamp_c | {8{~retry_stop_dyn}} & retry_stop_lat;
            `else
            retry_stop_seq <= {8{retry_stop_dyn}} & tx_seq | {8{~retry_stop_dyn}} & retry_stop_lat;
            `endif
    end

    `ifdef PAIF_INTF_512
    // ---- BUG-RB1a：push 完成水位（事件 = 512 形态 child wen[1] push_back 收齐拍；每 flit 恰 1 次，
    //   R-B1b 后锚定 T4 收齐拍；与 tx_seq（flit_complete，accept 末拍）常态锁步 → d≡0 零回归）----
    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            push_done_seq <= 8'hFF;                                     // 复位 -1（无 flit 完全写入）：watermark=push_done_seq+1=pushed 数
        else if (replay_push_wen_512_f[1])
            push_done_seq <= push_done_seq + 8'd1;                      // 每 back 收齐拍 +1 → 最后完全写入 flit seq（pushed-1）
    end
    // ---- BUG-RB1a：动态 stop 钳制 stop_dist = tx_seq-(push_done_seq+1) 8-bit 模距（push 落后
    //   ≤ DEPTH≪128，半模内无歧义）；d=0 → stop=tx_seq 零回归；d>0 → stop=push_done_seq+1 窗截断
    //   （X 窗消除，空窗由 retrans_empty_clear 承接）；& | 形态（rtl-coding 禁 ?:）----
    assign push_done_plus1  = push_done_seq + 8'd1;
    assign stop_dist        = tx_seq - push_done_plus1;
    assign stop_eq0         = ~(|stop_dist);
    assign stop_clamp_c     = ({8{stop_eq0}} & tx_seq) | ({8{~stop_eq0}} & push_done_plus1);
    `endif
    `ifndef PAIF_INTF_512
    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            retry_flit_loaded <= 1'b0;
        else if (replay_buf_flit_out_valid & retry_active)
            retry_flit_loaded <= 1'b1;
        else if (retry_back_event | retry_compat_event)
            retry_flit_loaded <= 1'b0;
    end

    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            retry_half <= 1'b0;
        else if (retry_front_event)
            retry_half <= 1'b1;
        else if (retry_back_event)
            retry_half <= 1'b0;
    end

    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            retry_rd_busy <= 1'b0;
        else if (retry_read_issue)
            retry_rd_busy <= 1'b1;
        else if (replay_buf_flit_out_valid & retry_active)
            retry_rd_busy <= 1'b0;
    end
    `endif

    `ifndef PAIF_INTF_512
    // retry_flit 捕获 pipe（1024/兼容形态：legacy flit_out 读回锁存；512 形态不编译——
    //   16 深 buffer D9：legacy flit_out_valid 恒 0 永不锁存、512 构建无消费者，2048 位冗余 pipe）
    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            retry_flit <= 2048'd0;
        else if (replay_buf_flit_out_valid & retry_active)
            retry_flit <= replay_buf_flit_out;
    end
    `endif

    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            retry_cnt <= 8'd0;
        else if (retry_enter)
            retry_cnt <= 8'd0;
        `ifdef PAIF_INTF_512
        else if (retrans_flit_done)
            retry_cnt <= retry_cnt + 8'd1;
        `else
        else if (retry_back_event | retry_compat_event)
            retry_cnt <= retry_cnt + 8'd1;
        `endif
    end

    `ifdef PAIF_INTF_512
    // v1.21 形态①：ren 恒 0——buffer 行流自走读（retrans_start_i / retrans_tick_i 驱动），父不再逐读（D1）
    assign replay_buf_ren      = 1'b0;
    `else
    assign replay_buf_ren      = retry_read_issue;
    `endif
    assign replay_buf_rptr     = retry_rd_seq[`REPLAY_BUF_PTR-1:0];
    assign replay_buf_ack_valid = ack_valid_i & csr_perf_mode_i;
    assign ack_advance_event    = ack_valid_i & csr_perf_mode_i;

    // ACK 推进 retire（MAS link_tx_mas.md：ack_seq_i 仅驱动 retire_ptr / oldest_unacked 推进；
    //   在途未 ACK 数 = wr_ptr - retire_ptr，故 retire 不得超出已发送窗口）。
    // BUGFIX 2026-09-02：retire_seq 必须被 tx_seq（已发窗口）封顶，且单调不回退。
    //   当对端（含重传被重复交付时）ACK 出超过本端 tx_seq 的 ack_seq 时，cap 到 tx_seq；
    //   否则 retire_seq > tx_seq 使 retry_ctrl 的重传窗口（retry_init_seq=retire_seq →
    //   retry_stop_seq=tx_seq）退化为空/跨环绕范围，retry_done 永不成立 → retry_active
    //   卡死 → paif_ready_ctl=0 → A 无法发新 Flit（tx_seq 停置、重传风暴）。
    // 2026-09-25 代数重排（bit-identical，不加 pipeline/锁存，控制信号不参与数据臂）：
    //   将原「加法比较」两级串联 clamp（a+1>r ? a+1:r → n1>t ? t:n1）摊平，断开
    //   reg2reg 关键路径（dlp_linknum_q → replay_buffer/retire_ptr，23 级，DC 399921 实测）
    //   上的进位-比较串联。
    //   等价（均 8-bit 环绕语义）：s1=(a>=r)&(a!=8'hFF) ⇔ 旧 s1=(a+1 mod 256 > r)：a=FF ⇒ a+1=0，
    //     0>r≡0 = (FF>=r)&(FF!=FF) ✓；a≠FF ⇒ a+1 不进位，a+1>r ⇔ a>=r ✓。s2：旧 s2=(n1>t)，
    //     s1=1 ⇒ n1=a+1∈1..255，a+1>t ⇔ a>=t；s1=0 ⇒ n1=r，即 r>t ✓。
    //     256^3=16,777,216 组 (a,r,t) 穷举逐组 bit-identical（见 task 回报）。
    //   09-02/09-03 BUGFIX 保护不变：单调不回退（n1≥r，retire 不被陈旧 ACK 拉回）
    //   + 封顶于 tx_seq（n2≤t）；credit 指针（replay_buf_ack_ptr）仍取同一 clamp 结果 ack_retire_cap。
    wire [7:0] ack_sum        = ack_seq_i + 8'd1;                    // a+1（仅数据臂，不参与选择）
    wire       ack_next_sel   = (ack_seq_i >= retire_seq) & (ack_seq_i != 8'hFF); // s1 ≡ 旧 (a+1 > r)
    wire [7:0] ack_retire_next = {8{ack_next_sel}} & ack_sum
                                | {8{~ack_next_sel}} & retire_seq;   // n1 ≡ 旧 (s1 ? a+1 : r)
    wire       ack_cap_sel     = ack_next_sel & (ack_seq_i >= tx_seq) // s2 ≡ 旧 (n1 > t)：s1=1 ⇒ a>=t
                                | ~ack_next_sel & (retire_seq > tx_seq); // s1=0 ⇒ r>t
    wire [7:0] ack_retire_cap  = {8{ack_cap_sel}} & tx_seq
                                | {8{~ack_cap_sel}} & ack_retire_next; // n2 ≡ 旧 (s2 ? t : n1)
    // BUGFIX 2026-09-03（credit/retire 限流）：replay buffer 的 credit 指针（replay_buf_ack_ptr）
    //   必须与 retry 窗口 retire_seq 共用「单调不回退 + 封顶于已发窗口」的保护。
    //   原实现把原始 ack_seq_i 直接送 replay_buffer 作 retire 指针：若对端（含重传被重复交付 /
    //   乱序 / 陈旧 ACK）回给低于当前 retire 的 ack_seq，replay retire_ptr 被拉回 → credit(full)
    //   被错误释放 → 本端超发超过 REPLAY_BUF_DEPTH 限额 → 对端 RX credit_fifo 溢出丢单元。
    //   现改为将单调封顶的 ack_retire_cap 作为 credit 指针，保证 credit 只被正确回补、不回退，
    //   发送端在途未确认数恒 ≤ credit 限额（不超发）。
    assign replay_buf_ack_ptr   = ack_retire_cap[`REPLAY_BUF_PTR-1:0];
    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            retire_seq <= 8'd0;
        else if (ack_advance_event)
            retire_seq <= ack_retire_cap;
    end

    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            retrain_timeout_cnt <= 8'd0;
        else if (ack_valid_i | nack_valid_i)
            retrain_timeout_cnt <= 8'd0;
        else if (timeout_event)
            retrain_timeout_cnt <= retrain_timeout_cnt + 8'd1;
    end
    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            retrain_nack_cnt <= 8'd0;
        else if (ack_valid_i)
            retrain_nack_cnt <= 8'd0;
        else if (nack_valid_i)
            retrain_nack_cnt <= retrain_nack_cnt + 8'd1;
    end
    assign retrain_timeout_thr = (retrain_timeout_cnt >= csr_retrain_timeout_thr_i);
    assign retrain_nack_thr    = (retrain_nack_cnt    >= csr_retrain_nack_thr_i);
    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            retrain_timeout_flag <= 1'b0;
        else if (retrain_timeout_thr & csr_retrain_timeout_en_i)
            retrain_timeout_flag <= 1'b1;
    end
    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            retrain_nack_flag <= 1'b0;
        else if (retrain_nack_thr & csr_retrain_nack_en_i)
            retrain_nack_flag <= 1'b1;
    end
    always @(posedge ck_tx) begin
        if (!mb_rst_n) begin
            retrain_timeout_flag_d <= 1'b0;
            retrain_nack_flag_d    <= 1'b0;
        end
        else begin
            retrain_timeout_flag_d <= retrain_timeout_flag;
            retrain_nack_flag_d    <= retrain_nack_flag;
        end
    end

    // =========================================================================
    // timeout_monitor（MAS §4.5）：oldest_unacked 计时，flit_sent 启动 / ack 重启 / retry 冻结
    // =========================================================================
    `ifdef PAIF_INTF_512
    // v1.21 形态① 锚点迁移（MAS §4.5）：重传/NULL 前端拍事件已删 → retrans_start / null_start
    //   （替换现 exit_front/exit_compat 的重传/NULL front 拍项）；front_proc = T2 = 泵 LO(u0)
    //   advance 锚（front 行上线拍，与现同拍）
    assign flit_sent_event = front_proc | retrans_start | null_start;
    `else
    assign flit_sent_event = exit_front | exit_compat;   // 新 Flit 进入（front/compat 装载，非 back），送 timeout_monitor
    `endif
    // link 非 S_NORMAL（训练/低功耗）时冻结超时重传计时，避免非活跃期空跑计数
    assign timeout_run     = link_active_i & ~retry_active & retry_en_eff & (retire_seq != tx_seq);
    // gdx_review Item 7：timeout_clr_i 寄存器去掉；timeout_cnt 仅保留 +1 与清 0 两状态，
    //   原置 1 处（ack_advance_event / flit_sent_event）改置 0，所有清 0 条件合并到单一 if。
    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            timeout_cnt <= 16'd0;
        else if (retry_trigger | ack_advance_event | (flit_sent_event & ~retry_active))
            timeout_cnt <= 16'd0;
        else if (timeout_run)
            timeout_cnt <= timeout_cnt + 16'd1;
    end
    assign timeout_event = (timeout_cnt >=        csr_retry_timeout_val_i) & timeout_run;

    // =========================================================================
    // ctrl_flit_gen：NULL Flit 自生成（非抢占，MAS §6.5 / §6.5.1）
    // =========================================================================
    assign null_idle_ok = proto_is_paif & ~retry_active & ~paif_rx_accept;
    // gdx_review Item 8：兼容 NULL 生成迁入 link_tx_compat（父仅例化透传），父 null_start 仅性能模式
    assign null_start   = null_idle_ok & ~null_active & (dlp_cmd_reg != 2'b00) & csr_perf_mode_i;
    `ifdef PAIF_INTF_512
    // v1.21 形态①（D2）：NULL 由 child 自生成（null_cnt 4 拍连续、header/dlp 取自 header_f/dlp_f，
    //   child D2）——父仅给起始单事件（null_start 公式 & 泵空闲 ~unit_busy_f——NULL/重传事件须等
    //   当前单元高半段发完，BUG_LOG 2026-09-02 语义保持），null_done_o 末拍清 null_active
    assign null_flit_start_f = null_start & ~unit_busy_f;
    `else
    // NULL/重传 perf 事件须等当前单元高半段（含 CRC）发完（~unit_busy_f），否则 front→back
    // 连拍装载覆盖丢高半段 → 对端 RX CRC 校验失败（BUG_LOG 2026-09-02：NULL/重传前端半丢失修复）
    assign null_front_event  = null_active & csr_perf_mode_i & ~null_half & conv_accept & ~unit_busy_f;
    assign null_back_event   = null_active & csr_perf_mode_i & null_half & conv_accept & ~unit_busy_f;
    assign ctrl_perf_null = {1008'd0, dlp_field,
                             1008'd0, 4'b0000, 8'hff, hdr_type_linknum_ctrl};
    `endif
`ifdef LINK_TX_COMPAT_MODE
    // 兼容 NULL 生成请求（父：ACK/NACK 待发、无 PAIF 数据、非重传 → link_tx_compat 内建生成）
    assign null_gen_req = proto_is_paif & ~retry_active & ~paif_tx_valid_i & (dlp_cmd_reg != 2'b00) & ~csr_perf_mode_i;
`endif

    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            null_active <= 1'b0;
        `ifdef PAIF_INTF_512
        // v1.21 形态①（D2）：child null_done 末拍清（NULL 4 拍连续自走）；null_done /
        //   null_flit_start_f 互斥（null_start 要求 ~null_active，无同拍冲突）
        else if (null_done)
            null_active <= 1'b0;
        else if (null_flit_start_f)
            null_active <= 1'b1;
        `else
        else if (null_start)
            null_active <= 1'b1;
        else if (null_back_event)
            null_active <= 1'b0;
        `endif
    end
    `ifndef PAIF_INTF_512
    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            null_half <= 1'b0;
        else if (null_front_event)
            null_half <= 1'b1;
        else if (null_back_event)
            null_half <= 1'b0;
    end
    `endif

    // =========================================================================
    // 出口数据源注入 mux（正常 / 重传 / NULL / ARQ 共用 dlp_update + CRC，MAS §4.4 / §4.9）
    // =========================================================================
    `ifdef PAIF_INTF_512
    // v1.21 形态①（D4）：重传行流——front 行含固化 header 原样重发、back 行 child 内 DLP 替换 →
    //   RETRY 源删除（重传拍无需 header 回嵌）；NULL child 自生成（header_f 8'hff / dlp_f 自嵌）
    // 2026-09-25 BUGFIX（NULL front 拍头采样错位一拍）：NULL 支须在起始拍 null_flit_start_f 即成立——
    //   child NULL beat0 线上数据于起始拍沿装载 header_f（null_cur），CRC 窗口（null_field）于发送拍
    //   （null_active 寄存翻转次拍）采样；起始拍若仍取 DATA 支 → 线上头 {tx_seq,type=1}（0x0001）与
    //   CRC 计及头 {8'hff,type=0}（0x0FFx）错位一拍 → RX front chkerr + is_null_c 失判。
    //   起始拍安全性：全路 quiescent（~unit_busy_f）且 null_idle_ok 要求 ~paif_tx_valid_i（无 paif 装载）。
    assign exit_src   = {2{null_active | null_flit_start_f}} & 2'b10;
    assign exit_hdr   = {16{(exit_src == 2'b10)}} & {4'b0000, 8'hff, hdr_type_linknum_ctrl}
                       | {16{~(exit_src == 2'b10)}} & {4'b0000, tx_seq, hdr_type_linknum};
    `else
    assign exit_src = {2{retry_active}} & 2'b01 | {2{~retry_active & null_active}} & 2'b10;
    assign exit_front = front_proc | retry_front_event | null_front_event;
    assign exit_back  = back_proc  | retry_back_event  | null_back_event;
    // gdx_review Item 8：兼容控制 Flit（NULL/ARQ）装载事件由兼容形态回报 ctrl_event_compat_o
`ifdef LINK_TX_COMPAT_MODE
    assign exit_compat= compat_sum | retry_compat_event | ctrl_event_compat;
`else
    assign exit_compat= compat_sum | retry_compat_event;
`endif
    assign exit_hdr = {16{(exit_src == 2'b01)}} & retry_flit[15:0]
                     | {16{(exit_src == 2'b10)}} & {4'b0000, 8'hff, hdr_type_linknum_ctrl}
                     | {16{~(exit_src == 2'b01) & ~(exit_src == 2'b10)}} & {4'b0000, tx_seq, hdr_type_linknum};
    `endif

    // 当前例化形态可装载（解耦：直接取形态入口 ready，不依赖装载事件，避免组合环）
    `ifndef PAIF_INTF_512
    assign conv_accept = paif_ready_conv;
    `endif

    // =========================================================================
    // CRC 路径（crc9/crc16 分时按单元拍；CRC9 兼容模式、CRC16 性能模式）
    //   三路 CRC 数据源 mux（PAIF/NULL/重传）+ header/DLP 嵌入 + 注错 已下沉至各性能
    //   形态模块（link_tx_512_lane1 / link_tx_1024_lane1 / link_tx_1024_lane2）——
    //   模块输出 crc_data_*_o（当前要算 CRC、已嵌字段且 CRC 槽清 0 的单拍数据）返父，
    //   父仅送入 crc16_generate；校验码 crc16_result 同拍回接模块 crc_chk_*_i 由其在线
    //   内嵌回 CRC 槽位（front→CRC0 / back→CRC1）。CRC16 纯组合，0 cycle 无环。
    //   故父侧不再做 front_unit/back_unit/null_unit_crc/unit_ckd_f 组装（见形态驱动区）。
    // =========================================================================
    assign crc16_data  = crc_data_active;
    assign ecc16_data   = crc_data_active;   // 同一数据源（MAS §4.4：ecc_data 与 crc16_data 同源）

    // 父组装回喂单元 / 形态驱动（2 系形态交接统一为"字段自嵌"语义）：
    //   各性能形态（512/1024_lane1/1024_lane2）均为字段自嵌重构——父不再回喂
    //   unit_ckd/nv 完整单元，改驱动 header/DLP/CRC 校验码 + NULL/重传 数据&front 标记
    //   给形态模块，由模块自身在 PAIF 管线按拍位自嵌。
    //   header_f = exit_hdr（per-source front header：DATA=tx_seq / RETRY=flit[15:0] /
    //              NULL=8'hff），front 拍由模块嵌入 [15:0]
    //   dlp_f    = dlp_field（back DLP，统一），back 拍由模块嵌入 [15:0]
    //   retry_data_f = replay pop 半单元（retry_half 选 front/back；header/DLP 由模块嵌）
    //   null_data_f  = 0（NULL payload 全 0；header/DLP 由模块按 header_f/dlp_f 嵌入，
    //                 与父侧原 ctrl_perf_null 组装等价）
    wire [15:0]   header_f;
    wire [15:0]   dlp_f;
    `ifndef PAIF_INTF_512
    wire [1023:0] retry_data_f;
    wire          retry_valid_f;
    wire          retry_front_f;
    wire [1023:0] null_data_f;
    wire          null_valid_f;
    wire          null_front_f;
    `endif
    // CRC 数据源选择（寄存源状态，不入装载事件）：与装载脉冲 retry_valid_f/null_valid_f
    //   解耦——select 用已寄存的 retry_active/null_active（短路径，不带 conv_accept/link_active），
    //   装载/valid 拍仍用事件脉冲（reconv_en/CE、inj_beat、front/back 节拍不变）。
    //   front/back 标记同样改用已寄存半态（~retry_half / ~null_half）：front 拍半态=0
    //   （事件亦要求 ~retry_half/~null_half），替换含 conv_accept 的前端脉冲，切断
    //   link_active→CRC 数据源/槽位选择 的关键路径。
    `ifndef PAIF_INTF_512
    wire          retry_sel_f;         // CRC 源选择：1=重传源（=寄存 retry_active）
    wire          null_sel_f;          // CRC 源选择：1=NULL 源（=寄存 null_active）
    `endif
    assign header_f      = exit_hdr;
    assign dlp_f         = dlp_field;
    `ifndef PAIF_INTF_512
    assign retry_data_f  = ({1024{retry_half}} & retry_flit[2047:1024]) | ({1024{~retry_half}} & retry_flit[1023:0]);
    assign retry_valid_f = retry_front_event | retry_back_event;
    assign retry_front_f = ~retry_half;             // front 标记：半态（front=0→1 / back=1→0）
    assign null_data_f   = {1024{1'b0}};
    assign null_valid_f  = null_front_event | null_back_event;
    assign null_front_f  = ~null_half;              // front 标记：半态（front=0→1 / back=1→0）
    assign retry_sel_f   = retry_active;            // 寄存源状态直连（无 conv_accept/~unit_busy gating）
    assign null_sel_f    = null_active;
    `endif
    // gdx_review Item 8：兼容 ARQ/NULL 生成迁入 link_tx_compat，父仅透传数据 Flit /
    //   重传 Flit / ARQ 请求；控制 Flit 装载事件经 ctrl_event_compat_o 回报（并入 flit_sent）。
    // 兼容模式父组装 Header+Payload（仅 DATA 数据 Flit → 兼容形态 compat_flit_data_compat_i）
    `ifdef LINK_TX_COMPAT_MODE
    assign compat_flit_data_f  = comp_flit_assembled[260:0];
    assign compat_flit_valid_f = compat_sum;
    `endif
    `ifndef PAIF_INTF_512
    `ifdef LINK_TX_COMPAT_MODE
    assign retry_flit_compat       = retry_flit[271:0];
    assign retry_flit_valid_compat = retry_compat_event;
    assign retrans_done_compat     = retry_compat_event & retry_done;
    `endif
    `endif

    // =========================================================================
    // 当前例化形态模块例化（编译期宏选择，非 generate，编译时仅 1 个性能形态激活；
    // 兼容形态由 LINK_TX_COMPAT_MODE 单独激活，默认 0 关闭）
    // =========================================================================
`ifdef LINK_TX_COMPAT_MODE
    // ---------------- 兼容模式形态（link_tx_compat，形态④）-----------------
    // 兼容形态内嵌 CRC9（无外送 CRC），paif 入口为父组装 Header+Payload（261 bit）
    link_tx_compat u_link_tx_compat (
        .mb_clk                    (ck_tx),
        .mb_rst_n                  (mb_rst_n),
        .csr_perf_mode_i           (csr_perf_mode_i),
        .csr_protocol_mode_i       (csr_protocol_mode_i),
        .csr_s2p_mode_i            (csr_s2p_mode_i),
        .csr_bw_mode_i             (csr_bw_mode_i),
        .csr_drift_period_i        (csr_drift_period_i),
        .compat_flit_data_compat_i (compat_flit_data_f),
        .compat_flit_valid_compat_i(compat_flit_valid_f),
        .paif_tx_ready_compat_o    (conv_ready_active),
        .retry_flit_compat_i       (retry_flit_compat),
        .retry_flit_valid_compat_i (retry_flit_valid_compat),
        .retry_active_compat_i     (retry_active),
        .null_gen_req_compat_i     (null_gen_req),
        .arq_req_flit_compat_i     (arq_req_flit_i),
        .arq_req_valid_compat_i    (arq_req_valid_i),
        .arq_req_rcv_ptr_compat_i  (arq_req_rcv_ptr_i),
        .arq_req_rcv_valid_compat_i(arq_req_rcv_valid_i),
        .retrans_done_compat_i     (retrans_done_compat),
        .tx_seq_compat_i           (tx_seq),
        .ctrl_busy_compat_o        (ctrl_busy_compat),
        .ctrl_event_compat_o       (ctrl_event_compat),
        .replay_push_wen_compat_o  (replay_push_wen_f),
        .replay_push_wptr_compat_o (replay_push_wptr_f),
        .replay_push_flit_compat_o (replay_push_flit_f[271:0]),
        .cpif_tx_data_compat_o     (cpif_paif_lane0_data),
        .cpif_tx_data_compat_vld_o (cpif_paif_lane0_vld),
        .cpif_tx_ready_i           (cpif_tx_ready_i),
        .link_tx_active_i          (link_tx_active)
    );
    assign crc_data_active   = 1024'd0;       // 兼容形态无外送 CRC
`ifdef LINK_LANE1
    assign cpif_paif_lane1_data = 1024'd0;
    assign cpif_paif_lane1_vld  = 1'b0;
`endif
`else
    `ifdef PAIF_INTF_512
        // ---------------- 形态① link_tx_512_lane1（PAIF 512 × 1 lane）-----------------
        link_tx_512_lane1 u_link_tx_512_lane1 (
            .mb_clk                    (ck_tx),
            .mb_rst_n                  (mb_rst_n),
            .csr_protocol_mode_i       (csr_protocol_mode_i),
            .csr_s2p_mode_i            (csr_s2p_mode_i),
            .csr_bw_mode_i             (csr_bw_mode_i[0]),     // 2→1（child MAS v0.1）：本形态恒 0，取低 1 位
            .csr_drift_period_i        (csr_drift_period_i),
            .paif_tx_data_i            (paif_tx_data_i),
            .paif_tx_vld_i             (paif_tx_valid_i),      // 端口改名（child MAS v0.1：paif_tx_valid_i→paif_tx_vld_i）
            .paif_tx_ready_o           (conv_ready_active),
            .paif_rx_accept_i          (paif_rx_accept),
            // 字段自嵌输入（child MAS v0.1：512 全宽端口、模块取 [15:0]；父源为 16 位
            //   header_f/dlp_f（exit_hdr/dlp_field），连接处零扩展——父侧 512 位寄存源若需
            //   落实为寄存器，属后续父侧任务，见回报不一致清单）
            .header_512_lane1_i        ({496'h0, header_f}),
            .dlp_512_lane1_i           ({496'h0, dlp_f}),
            .crc_chk_512_lane1_i       (crc16_result),
            .ecc_chk_512_lane1_i       ({4'h0, ecc16_result}),  // 12→16（child MAS v0.1）：SECDED P-bit tie-0 移父侧
            // 重传行流（D1：父仅给起始事件 + len；行数据/ready 经父自 buffer 直送；
            //   tick 经父转送 buffer，flit_done 驱动影子 rd_seq/统计/停止）
            // C7-2（LL-20260925-CRIFVO-C7 拍级模型 S4 发现，裁决=候选①）：len 改接组合
            //   retrans_len_c（start 拍 rd_seq 仍=窗口头 → 全窗 len 当拍有效）——旧接线父
            //   （已删 retrans_len 锁存寄存器——lint W528 set-not-read）与子采样同沿（start 拍双沿锁存）→ 子采旧值（首窗 0）：
            //   len≥2 首窗 child flit0 后自停 → 父 retrans_done 无源 → retry_active 恒 1。
            //   恢复子 MAS 端口契约 L123「start 拍采样」口径。
            .retrans_start_i           (retrans_start),
            .retrans_len_i             (retrans_len_c),
            .retrans_line_data_i       (retrans_line_data),
            .retrans_line_ready_i      (retrans_line_ready),
            .retrans_tick_o            (retrans_tick),
            .retrans_flit_done_o       (retrans_flit_done),
            // NULL 自生成（D2：起始单事件直送 child；null_done 末拍回报清 null_active）
            .null_flit_start_i         (null_flit_start_f),
            .null_done_o               (null_done),
            // UDI 直通（D7/M7：raw 入口直送，child 1T 无条件寄存；512 父删 udi_pipe_lane0）
            .udi_data_i                (udi_tx_lane0_data_i),
            .udi_vld_i                 (udi_tx_lane0_data_vld_i),
            .csr_ecc_en_i              (csr_ecc_en_i),
            .csr_inj_en_i              (csr_inj_en_i),
            .csr_inj_fb_i              (csr_inj_fb_i),
            .crc_data_512_lane1_o      (crc_data_active),
            // replay 行流 push（child MAS v0.1：wen 2 位 {push_back,push_front} /
            //   data 1024 行（注入前）/ wptr 4 位纯行索引；wptr 扩展与 wen 合成
            //   buffer 口径见下方转发区）
            .replay_push_wen_o         (replay_push_wen_512_f),
            .replay_push_wptr_o        (replay_push_wptr_512_f),
            .replay_push_data_o        (replay_push_flit_f[1023:0]),
            .cpif_tx_data_512_lane1_o     (cpif_paif_lane0_data),
            .cpif_tx_data_512_lane1_vld_o (cpif_paif_lane0_vld),
            .tx_busy_512_lane1_o          (unit_busy_f),
            .cpif_tx_ready_i           (cpif_tx_ready_i),
            .link_tx_active_i          (link_active_cp)          // F1a（LL-20260930-N4-v3）：改接同 D 副本寄存器（唯一改接线点）
        );
`ifdef LINK_LANE1
        assign cpif_paif_lane1_data = 1024'd0;
        assign cpif_paif_lane1_vld  = 1'b0;
`endif
        // 512 形态（方案 B v1.1）：replay push 数据缩为 1024 bit 半单元，父侧高 1024 bit 补 0；
        //   仅 512 配置驱动（1024 两形态与 compat 态仍整宽驱动 replay_push_flit_f，此处若
        //   无条件 assign 会与 1024 形态整宽输出双驱动）。
        assign replay_push_flit_f[2047:1024] = 1024'd0;
    `else
        `ifdef LINK_LANE1
            // ---------------- 形态③ link_tx_1024_lane2（PAIF 1024 × 2 lane）-----------------
            link_tx_1024_lane2 u_link_tx_1024_lane2 (
                .mb_clk                    (ck_tx),
                .mb_rst_n                  (mb_rst_n),
                .csr_perf_mode_i           (csr_perf_mode_i),
                .csr_protocol_mode_i       (csr_protocol_mode_i),
                .csr_s2p_mode_i            (csr_s2p_mode_i),
                .csr_bw_mode_i             (csr_bw_mode_i),
                .csr_drift_period_i        (csr_drift_period_i),
                .paif_tx_data_i            (paif_tx_data_i),
                .paif_tx_valid_i           (paif_tx_valid_i),
                .paif_tx_ready_o           (conv_ready_active),
                .paif_rx_accept_i          (paif_rx_accept),
                // 字段自嵌输入（1024_lane2 形态：header/DLP/CRC 校验码 + NULL/重传 数据&front）：
                .header_1024_lane2_i       (header_f),
                .dlp_1024_lane2_i          (dlp_f),
                .crc_chk_1024_lane2_i      (crc16_result),

                .ecc_chk_1024_lane2_i      (ecc16_result),
                .null_data_1024_lane2_i    (null_data_f),
                .null_valid_1024_lane2_i   (null_valid_f),
                .null_front_1024_lane2_i   (null_front_f),
                .null_active_1024_lane2_i  (null_sel_f),
                .retry_data_1024_lane2_i   (retry_data_f),
                .retry_valid_1024_lane2_i  (retry_valid_f),
                .retry_front_1024_lane2_i  (retry_front_f),
                .retry_active_1024_lane2_i (retry_sel_f),
                .csr_crc_en_i              (csr_crc_en_i),

                .csr_ecc_en_i              (csr_ecc_en_i),
                .csr_inj_en_i              (csr_inj_en_i),
                .csr_inj_fb_i              (csr_inj_fb_i),
                .crc_data_1024_lane2_o     (crc_data_active),
                .replay_push_wen_1024_lane2_o (replay_push_wen_f),
                .replay_push_wptr_1024_lane2_o(replay_push_wptr_f),
                .replay_push_flit_1024_lane2_o(replay_push_flit_f),
                .cpif_tx_lane0_data_1024_lane2_o (cpif_paif_lane0_data),
                .cpif_tx_lane0_data_1024_lane2_vld_o (cpif_paif_lane0_vld),
                .cpif_tx_lane1_data_1024_lane2_o (cpif_paif_lane1_data),
                .cpif_tx_lane1_data_1024_lane2_vld_o (cpif_paif_lane1_vld),
                .unit_busy_1024_lane2_o    (unit_busy_f),
                .cpif_tx_ready_i           (cpif_tx_ready_i),
                .link_tx_active_i          (link_tx_active)
            );
        `else
            // ---------------- 形态② link_tx_1024_lane1（PAIF 1024 × 1 lane）-----------------
            link_tx_1024_lane1 u_link_tx_1024_lane1 (
                .mb_clk                    (ck_tx),
                .mb_rst_n                  (mb_rst_n),
                .csr_perf_mode_i           (csr_perf_mode_i),
                .csr_protocol_mode_i       (csr_protocol_mode_i),
                .csr_s2p_mode_i            (csr_s2p_mode_i),
                .csr_bw_mode_i             (csr_bw_mode_i),
                .csr_drift_period_i        (csr_drift_period_i),
                .paif_tx_data_i            (paif_tx_data_i),
                .paif_tx_valid_i           (paif_tx_valid_i),
                .paif_tx_ready_o           (conv_ready_active),
                .paif_rx_accept_i          (paif_rx_accept),
                // 字段自嵌输入（1024_lane1 形态：header/DLP/CRC 校验码 + NULL/重传 数据&front）：
                .header_1024_lane1_i       (header_f),
                .dlp_1024_lane1_i          (dlp_f),
                .crc_chk_1024_lane1_i      (crc16_result),
                .ecc_chk_1024_lane1_i      (ecc16_result),
                .null_data_1024_lane1_i    (null_data_f),
                .null_valid_1024_lane1_i   (null_valid_f),
                .null_front_1024_lane1_i   (null_front_f),
                .null_active_1024_lane1_i  (null_sel_f),
                .retry_data_1024_lane1_i   (retry_data_f),
                .retry_valid_1024_lane1_i  (retry_valid_f),
                .retry_front_1024_lane1_i  (retry_front_f),
                .retry_active_1024_lane1_i (retry_sel_f),
                .csr_crc_en_i              (csr_crc_en_i),

                .csr_ecc_en_i              (csr_ecc_en_i),
                .csr_inj_en_i              (csr_inj_en_i),
                .csr_inj_fb_i              (csr_inj_fb_i),
                .crc_data_1024_lane1_o     (crc_data_active),
                .replay_push_wen_1024_lane1_o (replay_push_wen_f),
                .replay_push_wptr_1024_lane1_o(replay_push_wptr_f),
                .replay_push_flit_1024_lane1_o(replay_push_flit_f),
                .cpif_tx_data_1024_lane1_o (cpif_paif_lane0_data),
                .cpif_tx_data_1024_lane1_vld_o (cpif_paif_lane0_vld),
                .unit_busy_1024_lane1_o    (unit_busy_f),
                .cpif_tx_ready_i           (cpif_tx_ready_i),
                .link_tx_active_i          (link_tx_active)
            );
`ifdef LINK_LANE1
            assign cpif_paif_lane1_data = 1024'd0;
            assign cpif_paif_lane1_vld  = 1'b0;
`endif
        `endif
    `endif
`endif

    // replay buffer 写入（数据源来自当前例化形态 replay_push_*，父模块转发）
    `ifdef PAIF_INTF_512
    // 形态①（child MAS v0.1 36 口契约）：child wen 2 位 {push_back(T4), push_front(T2)}——每 flit
    //   2 次 push（同行索引），合成 buffer 单 bit wen（buffer d16 双写机按 wen 顺序判 front/back 层，
    //   replay_buffer.v d16_is_front/d16_wr_last）；child wptr 4 位纯行索引 → buffer 5 位指针扩展
    //   （wptr_i[4] = 16 回绕 flag，replay_buffer.v full_match/occupancy_perf_next 消费）：flag 于
    //   wptr 15→0 翻转（等价旧 child 5 位自由运行 wptr 高位，复位同为 0）
        always @(posedge ck_tx) begin
            if (!mb_rst_n)
                replay_wptr_flag_r <= 1'b0;
            else if (replay_push_wen_512_f[1] & (replay_push_wptr_512_f == 4'd15))
                replay_wptr_flag_r <= ~replay_wptr_flag_r;
        end
        assign replay_buf_wen   = replay_push_wen_512_f[1] | replay_push_wen_512_f[0];
        assign replay_buf_wptr  = {replay_wptr_flag_r, replay_push_wptr_512_f};
    `else
        assign replay_buf_wen   = replay_push_wen_f;
        assign replay_buf_wptr  = replay_push_wptr_f;
    `endif
    assign replay_buf_flit_in = replay_push_flit_f;

    // =========================================================================
    // 状态派生
    // =========================================================================
    // link_tx_active：直接采 FSM 寄存状态位（link_tx_lcmsm.link_active_o），
    // 摘掉 onehot2bin + link_state_i==4'h4 组合判等（关键数据/CRC 链）
    assign link_tx_active = link_active_i;
    // F1a（LL-20260930-N4-v3）：link_active 同 D 副本寄存器——D = lcmsm 次态位
    //（link_active_nxt_i = st_link_active_nxt），mb_clk 无条件寄存（无复位，X-safe：
    //复位期 st_link_active_nxt ≡ 0（& mb_rst_n）→ 复位态确定 0 = cur_state[4] S_CONFIG）；
    //cp ≡ cur_state[4] 逐拍恒等含迁移/复位拍（D = 次态位形态；D=Q 直连副本影子禁用——
    //1→0 迁移拍 + pv/nv/ra pending 将提前 1 拍活跃违反 §3.4 契约）。仅驱 512 形态
    //出口（u_link_tx_512_lane1 .link_tx_active_i 改接线点）；tx_active / tx_idle /
    //timeout_run / udi_tx_ready_o 全部仍读原 link_tx_active（最小改动面）
    always @(posedge mb_clk) begin
        link_active_cp <= link_active_nxt_i;
    end
    // =========================================================================
    // 内部 ICG：u_icg_tx（本模块恰 1 处 sync_icg_pos；CK=mb_clk，Q=ck_tx 模块主时钟）
    //   clken_tx = ~clk_cfg_tx_n_dyn_i | tx_active | tx_wake | tx_hold_2T（TX 总开关已删）
    //   tx_idle 按 L1 §4.10.3 组合定义：（a）link_active_i 拉低且 unit_busy 未置
    //   （b）paif_tx_valid_i 拉低（c）retry/null/arq 在途全 0 且 ARQ 唤醒源不拉高。
    //   本路开关 n_dyn 复位 0 → clken_tx 恒 1 → ck_tx=mb_clk（bit-exact 零回归；复用旧总开关-同时=0 行为）。
    // =========================================================================
`ifdef LINK_TX_COMPAT_MODE
    assign arq_active = ctrl_busy_compat;   // 兼容 NULL/ARQ 控制在发 = arq_active（兼容 ARQ 握手在途）
`else
    assign arq_active = 1'b0;
`endif
    assign tx_active   = link_tx_active | retry_active | null_active | arq_active | unit_busy_f;
    assign tx_wake     = paif_tx_valid_i | arq_req_valid_i | arq_req_rcv_valid_i;
    assign tx_idle     = ~link_tx_active & ~unit_busy_f & ~paif_tx_valid_i
                         & ~retry_active & ~null_active & ~arq_active
                         & ~arq_req_valid_i & ~arq_req_rcv_valid_i;
    assign tx_hold_2T  = ~tx_idle | tx_noidleQ | tx_noidleQQ;
    assign clken_tx    = ~clk_cfg_tx_n_dyn_i | tx_active | tx_wake | tx_hold_2T;

    sync_icg_pos u_icg_tx (
        .RSTN (mb_rst_n),
        .CK   (mb_clk),
        .E    (clken_tx),
        .Q    (ck_tx)
    );

    // 进 idle 判定与展宽须在门控关断前仍运转 → noidle DFF 打未门控 mb_clk 域
    always @(posedge mb_clk) begin
        if (!mb_rst_n) begin
            tx_noidleQ  <= 1'b0;
            tx_noidleQQ <= 1'b0;
        end else begin
            tx_noidleQ  <= ~tx_idle;
            tx_noidleQQ <= tx_noidleQ;
        end
    end


    // ---- deskew（S_LINK_TRAIN，多 lane 训练对齐；仅多 lane 编译期例化）---------
    //   gdx_review Item 3：单 lane（LANE_MACRO_MODE=2'b00、无 LINK_LANE1）下无 deskew
    //   相关逻辑（编译期排除 counter/pattern），对齐 RX 侧 1 lane 直通语义（RX 形态
    //   ①② train_done_o=train_start_i 直通）。单 lane 时 TX FSM 不驻留 S_LINK_TRAIN
    //   （link_tx_lcmsm go_link_train=0），deskew_done_o=in_link_train 恒 0、deskew_act 恒 0。
`ifdef LINK_LANE1
    wire          in_link_train;
    reg  [1:0]    deskew_cnt;
    wire          deskew_act;
    assign in_link_train = (link_state_i == 4'h3);

    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            deskew_cnt <= 2'd0;
        else if (!in_link_train)
            deskew_cnt <= 2'd0;
        else if (deskew_cnt < 2'd2)
            deskew_cnt <= deskew_cnt + 2'd1;
    end

    assign deskew_act    = in_link_train & (deskew_cnt < 2'd2);
    assign deskew_done_o = in_link_train & (deskew_cnt == 2'd2);
`else
    wire          in_link_train;
    wire          deskew_act;
    assign in_link_train = (link_state_i == 4'h3);
    assign deskew_act    = 1'b0;
    assign deskew_done_o = in_link_train;   // 单 lane 直通（FSM 不驻留 S_LINK_TRAIN，恒 0）
`endif

    // =========================================================================
    // udi_bypass_mux / 桥接直连
    // =========================================================================
    // UDI 入口一级 pipe（纯重定时，每拍无条件采样，无 enable）
    // gdx_review Item 5：udi_pipe_lane0_data / udi_pipe_lane0_vld 去掉复位值——纯重定时
    //   pipe 每拍无条件采样，复位后行为不依赖初始值（UDI 模式下 ~proto_is_paif 从本 pipe
    //   直出，首拍前为 X，τ拍后恒为有效重定时数据）。
    `ifndef PAIF_INTF_512
    always @(posedge ck_tx) begin
        udi_pipe_lane0_data <= udi_tx_lane0_data_i;
        udi_pipe_lane0_vld  <= udi_tx_lane0_data_vld_i;
    end
    `endif
`ifdef LINK_LANE1
    always @(posedge ck_tx) begin
        if (!mb_rst_n) begin
            udi_pipe_lane1_data <= 1024'd0;
            udi_pipe_lane1_vld  <= 1'b0;
        end else begin
            udi_pipe_lane1_data <= udi_tx_lane1_data_i;
            udi_pipe_lane1_vld  <= udi_tx_lane1_data_vld_i;
        end
    end
`endif
`ifdef LINK_LANE3
    always @(posedge ck_tx) begin
        if (!mb_rst_n) begin
            udi_pipe_lane2_data <= 1024'd0;
            udi_pipe_lane2_vld  <= 1'b0;
        end else begin
            udi_pipe_lane2_data <= udi_tx_lane2_data_i;
            udi_pipe_lane2_vld  <= udi_tx_lane2_data_vld_i;
        end
    end
    always @(posedge ck_tx) begin
        if (!mb_rst_n) begin
            udi_pipe_lane3_data <= 1024'd0;
            udi_pipe_lane3_vld  <= 1'b0;
        end else begin
            udi_pipe_lane3_data <= udi_tx_lane3_data_i;
            udi_pipe_lane3_vld  <= udi_tx_lane3_data_vld_i;
        end
    end
`endif

   assign proto_is_paif = csr_protocol_mode_i;

    // CPIF 出口：PAIF 通路 = 当前例化形态输出（cpif_paif_*）；UDI 通路 = 入口 pipe；
    // 桥接下不直连——数据走形态正常流水线（csr_bridge_mode_i 不再选择直连通路，link_tx_mas §3.1）
    `ifdef PAIF_INTF_512
    // v1.21 形态①（D7/M7）：协议 mux（PAIF 泵 / UDI 直通）已入 child——cpif_paif_lane0_* 即
    //   lane0 终态（含 UDI beat）；父 mux 退化为 deskew | child
    assign cpif_tx_lane0_data_o     = {1024{deskew_act}} | cpif_paif_lane0_data;
    assign cpif_tx_lane0_data_vld_o = deskew_act | cpif_paif_lane0_vld;
    `else
    assign cpif_tx_lane0_data_o     = {1024{deskew_act}}
                                    | ({1024{proto_is_paif}} & cpif_paif_lane0_data)
                                    | ({1024{~proto_is_paif}} & udi_pipe_lane0_data);
    assign cpif_tx_lane0_data_vld_o = deskew_act | (proto_is_paif & cpif_paif_lane0_vld)
                                    | (~proto_is_paif & udi_pipe_lane0_vld);
    `endif
`ifdef LINK_LANE1
    assign cpif_tx_lane1_data_o     = {1024{deskew_act}}
                                    | ({1024{proto_is_paif}} & cpif_paif_lane1_data)
                                    | ({1024{~proto_is_paif}} & udi_pipe_lane1_data);
    assign cpif_tx_lane1_data_vld_o = deskew_act | (proto_is_paif & cpif_paif_lane1_vld)
                                    | (~proto_is_paif & udi_pipe_lane1_vld);
`endif
`ifdef LINK_LANE3
    assign cpif_tx_lane2_data_o     = {1024{~proto_is_paif}} & udi_pipe_lane2_data;
    assign cpif_tx_lane2_data_vld_o = ~proto_is_paif & udi_pipe_lane2_vld;
    assign cpif_tx_lane3_data_o     = {1024{~proto_is_paif}} & udi_pipe_lane3_data;
    assign cpif_tx_lane3_data_vld_o = ~proto_is_paif & udi_pipe_lane3_vld;
`endif

    assign udi_tx_ready_o = ~proto_is_paif & (link_tx_active | ~csr_udi_ready_active_en_i);   // AND-OR 等价值（rewrite 规则不用 ?:）

    // =========================================================================
    // 统计
    // =========================================================================
    always @(posedge ck_tx) begin
        if (!mb_rst_n)
            tx_flit_cnt <= 16'd0;
        else if (tx_flit_clr_i)
            tx_flit_cnt <= 16'd0;
        // gdx_review Item 6：兼容模式也统计（兼容整 Flit 收齐 compat_sum=~perf&flit_complete，
        //   与性能路径并集即 flit_complete，行为对齐工作模式）
        else if (flit_complete)
            tx_flit_cnt <= tx_flit_cnt + 16'd1;
    end

    // =========================================================================
    // 输出赋值（状态 / 中断上报）
    // =========================================================================
    assign csr_replay_buf_full_o   = replay_buf_full;
    assign retrain_timeout_flag_o  = retrain_timeout_flag;
    assign retrain_nack_flag_o     = retrain_nack_flag;
    assign irq_retry_timeout_o     = retrain_timeout_flag & ~retrain_timeout_flag_d;
    assign irq_retry_nack_o        = retrain_nack_flag & ~retrain_nack_flag_d;
    assign retrain_req_o           = {retrain_nack_flag & ~retrain_nack_flag_d,
                                      retrain_timeout_flag & ~retrain_timeout_flag_d};
    assign tx_flit_cnt_o           = tx_flit_cnt;
    assign timeout_cnt_o           = timeout_cnt[7:0];
    // 动态活动已并入 clken_tx 组合（内部 u_icg_tx）；replay SRAM 读写火线在
    //   replay_buffer u_icg_sram 的 sram_en 组合内自持（sram_hold_2_3T），不再上提。

    // =========================================================================
    // 公共子模块例化：replay_buffer / crc16_generate / crc9_generate（照例化）
    // =========================================================================
    replay_buffer u_replay_buffer (
        .mb_clk            (mb_clk),
        .mb_rst_n          (mb_rst_n),
        .sram_dyn_en_i     (sram_dyn_en_i),
        .csr_retry_en_i    (retry_en_eff),
        .csr_credit_en_i   (credit_en_eff),
        .csr_perf_mode_i   (csr_perf_mode_i),
        .wen_i             (replay_buf_wen),
        .wptr_i            (replay_buf_wptr),
        .flit_in_i         (replay_buf_flit_in),
        .ren_i             (replay_buf_ren),
        .rptr_i            (replay_buf_rptr),
        .ack_valid_i       (replay_buf_ack_valid),
        .ack_ptr_i         (replay_buf_ack_ptr),
        .full_o            (replay_buf_full)
    `ifndef PAIF_INTF_512
        // legacy flit 读回口（1024/兼容形态消费：retry_flit 捕获 / exit_hdr 等；512 形态 D9
        //   恒 0、行流读 retrans_line_* 替代——2026-09-24 冗余清理，与 retry_flit 同处理）
        , .flit_out_o       (replay_buf_flit_out)
        , .flit_out_valid_o (replay_buf_flit_out_valid)
    `endif
    `ifdef PAIF_INTF_512
        // v1.21 512 行流读（MAS replay_buffer v1.8：父仅给起始事件 + 窗口头/len；tick 经父
        //   转接 child→buffer；行数据/ready buffer→child）
        , .retrans_start_i      (retrans_start)
        , .retrans_rptr_start_i (replay_buf_rptr)
        , .retrans_tick_i       (retrans_tick)
        , .retrans_line_data_o  (retrans_line_data)
        , .retrans_line_ready_o (retrans_line_ready)
    `endif
    );
    crc16_generate u_crc16_generate (
        .crc16_data_i (crc16_data),
        .crc16_o      (crc16_result)
    );

    // ECC 引擎（与 crc16 并接同一数据源；纯组合 0 cycle，ecc16_result 供 TX 出口 chk 槽
    // 组装 mux——csr_ecc_en_i=1 嵌 ecc_o 低 12（高 4 tie0）、=0 嵌 crc16）
    ecc_generate u_ecc_generate (
        .ecc_data_i   (ecc16_data),
        .ecc_o        (ecc16_result)
    );

endmodule
