// =============================================================================
// module name: p_axi4_rwch_unpack
// author     : OpenHappy Team
// spec/MAS   : spec/protocol_layer/axi4_symm_prcl/submods/p_axi4_rwch_unpack/
//                p_axi4_rwch_unpack_spec.md / p_axi4_rwch_unpack_mas.md
// 功能概述   : 对称模式 AXI4 协议层 per-lane PAIF RX flit 解封装模块（新架构）。
//   接收 PAIF RX 数据，将扁平 VC 流（各 VC 以头部 CN+RP 自描述、首尾相接、
//   可跨拍、顺序任意）逐拍解析为 AW/AR/W/R/B 五物理通道信息 + AW/W/AR/R/B
//   五类 Credit（各 RP0~3）。架构：byp/pip 两级流水 → 游标行扫描预译码 →
//   每通道独立弹性缓冲（K 态保留 / 优先级排空）→ SP/b_vc 时隙合并输出。
//   设计约束:
//     C1 pad 恒在 payload 高位；最低位为 pad ⇒ 整拍 pad（尾部 pad → 游标归 0）
//     C2 w/r 不同时输出（流内数据 VC 任一时刻单一类型）
//     C3 w/r 的 SP 输出阻塞 b_vc 输出（SP 优先级高，时隙化无冲突）
//     C4 b/credit 必须寄存器输出（valid 未拉高时 bus 归 0）；aw/w/r 为组合输出（[bak201 修订] b/ar/credit 已改组合直出）
//     C5 抛弃聚合 VC 概念（同通道 VC 可相邻）；PRF 不处理
// =============================================================================
// version history（仅记录规格变更、重要BUG修复，不记录 lint 等小问题修改）
//   2026-09-18  OpenHappy Team  新架构重写（bak78 补全）：VC 流游标 + 每通道弹性缓冲，
//                      替换旧"数据VC+gather+PRF"单缓冲架构；含反压锁步
//                      （unmasked req 无环）、扫描行扩槽（低行 gth/last 全驱动）、
//                      K 态保留（跨拍拼合）、优先级排空（C5）、SP/b_vc 时隙
//                      合并（C3）、b/credit 寄存器输出（C4）、w/r 组合输出（C4 修订）→ [bak201 修订] b/ar/credit 组合化
//   2026-09-19  OpenHappy Team  bak79：扫描预译码前推 byp 级（计算数据源 pld_pip→pld_byp；行选择索引
//                      cn_ptr_idx = pld_pip_hsk ? cn_ptr_nxt_b_r : cn_ptr_pip_r = 内容 P 进入 pip 时
//                      将看到的游标）；cn_ptr*sel / *_int_masked / rsft_vc[*] / *_vc_index /
//                      后继游标 / 后继 flag 全部寄存器化（byp→pip 转移拍 pld_byp_hsk 更新），
//                      pip 侧经同名 alias 消费寄存值，下游消费者代码不变；另 *_req 亦寄存器化
//                      （byp_hsk 拍采样 f(P 的前驱) | |P 的 masked_b，与组合值逐拍位等价），
//                      反压入口（pld_pip_rdy → pld_byp_rdy → PAIF RX ready）不再过 OR 级。关键锥由
//                      "31:1 行选 + CN 译码 + 优先链 + 宽窗选择 → 通道缓冲" 变为 "预译码寄存器
//                      → 通道缓冲"。
//   2026-09-20  OpenHappy Team  bak100：① cn_ptr_idx 恒取 cn_ptr_nxt_b_r（删 pld_pip_hsk 2:1 选：
//                      锁步不变式 pip 空 ⇒ cn_ptr_pip_r == cn_ptr_nxt_b_r，采样边上逐位相同，
//                      断开 buf_r→rdy→hsk→idx→预译码 反馈环）；② flag 保持项/req 采样前驱
//                      直接用 last_cn_flag_pip_r（同不变式：pip 空 ⇒ flag_r == _b_r）；
//                      ③ |w/r_int_masked 的 pip 侧 OR 归约寄存器化为 w/r_int_masked_any_pip
//                      （byp_hsk 拍与 _b_r 并行采样，全时点 == |*_int_masked，6 处消费者换 1bit
//                      寄存器直读；8/16bit *_int_masked_pip_r 及 alias 无 pip 消费者可被 DC 裁掉）；
//                      ④ *_vdn 寄存器化（byp 组合 _b：前驱 flag=last_cn_flag_pip_r(I2)、
//                      游标=cn_ptr_nxt_b_r(I1)、masked=|*_int_masked_pip、sel[vc_index]=_b 组合，
//                      byp_hsk 拍采样；与 P 驻留期 pip 组合 *_vdn 逐位相同。cnt 更新锥去掉
//                      字段查找/flag 选/首次加法，只留 cnt_df + vdn_r 单加法；pip dat_vdn 死信号已删）。
//   2026-09-20  OpenHappy Team  bak101：1a w 缓冲变基址 CN 探针（w_buf_r[(wfull_bot_r?WFULL:WLITE)+:5]，
//                      数据依赖地址 ⇒ ~1088bit 宽 MUX）改两个定基址探针 w_bot1_at_full/lite
//                      + 1bit 选（行为等价：选中侧值同，未选侧不消费），w rdy 深锥去宽 MUX；
//                      wfull_bot_r 常数展开进 w_partial/cnt_df D 减/rsft_w 移位/w_valid_int
//                      （减法器/比较器 B 端编译期常数，w_bot_width 死信号已删）；
//                      w/r_buf_rdy 置常 1（结构性重言式：缓冲定宽 = 最坏拼合 ⇒ cnt_df < VC_CNT
//                      三分支皆真；前提 = w/r 输出无回压每拍弹出 + 每拍注入 ≤ PLD_CNT + C2，
//                      见代码内证明注释；aw/ar/b 不适用）；pld_pip_rdy 保留 w/r 项原结构；
//                      2b vc_index 串行优先链 → 并行 first-hit（p=前缀命中/g=首个命中 one-hot/
//                      MASK 编码，值恒等）；2c dat_mask 串行累加 → 并行前缀 OR（|dat_cn_byp[j-1:0]，
//                      原下三角常量表已删）。
//   2026-09-21  OpenHappy Team  bak105~108：CN 解码重构系列（均与 bak104 行为等价，纯数据拍由 valid_b
//                      的 (vld & ~pld_only) 广播项统一清零——idx=31 越界 X / idx=32..37 oh 回绕行 0..5
//                      活数据均不可信，该广播项不可删；窗外槽表值 7'h7f 不撞任何合法 CN）：
//                      bak105：12 路 cn_b 内 ~pld_only + valid 去 exist + pad 放宽 ~bit4；
//                      bak106：门控收敛至 valid_b 广播 + pad 严格 ==5'd0；bak107：valid 去 vld
//                      （byp_hsk⊃vld 不变式，实测 QOR 回归，已回退）；bak108：2a one-hot 行选回退
//                      原式 31:1 直读（本库级数中性，省 oh/列布线面积）+ vld 恢复（= bak106 形式）。
//                      综合：bak104 −0.037/L23 > bak105 −0.039/L25 > bak106 −0.044/L23 > bak107 −0.051/L24
//                      > bak108 −0.042/L23（TNS：104 −19.4 / 108 −21.0 同档）；bak104 仍为实测最优基线，
//                      2a 回退确认级数中性。
//   2026-09-22  OpenHappy Team  bak108r：ar_rp0~3_bus/valid 输出级寄存器化 → 组合直出（去 8 个 ar_rpN_*_r
//                      寄存器，端口改驱 _nxt；busy 同步改用 arN_valid_int）。语义：AR 输出提前 1 拍
//                      （drain 拍即出，原为 drain+1 拍寄存），父模块需接受 AR 提前一拍；
//                      面积 −432 flops（4×107+4），ar 输出变为组合路径（buf 寄存器+比较+mux 到端口）。
//   2026-09-21  OpenHappy Team  lint 清理：SpyGlass W528 定位 28 个死信号删除（16 个 x_int_masked_pip_r
//                      预译码寄存器 + 4 个 per-slot 表寄存器（cn_ptr_wnd_sel/B_ptr_sel/A_ptr_sel/
//                      dat_vdn_sel_b_r）+ 3 个聚合游标表寄存器（exist/gth_ptr/last_sel_b_r）+ 4 条 pip
//                      同名 alias 线（上列 4 表）+ cn_ptr_pip_r（bak100 后零消费）+ cn_ptr_byp（级联：唯一消费者
//                      为 cn_ptr_pip_r）；前一轮已删 19 条死 alias
//                      （16 masked + 3 游标表 pip alias）。byp 组合表（*_sel_b / *_cn_b / vdn / B·A_ptr_int）
//                      均为真实消费者，保活。终轮 lint（0921e）：0 Error / 0 W528；余 3 条已知良性
//                      Warning（W415a×2 = B/A_ptr_int_pip 有意 OR 累加；STARC MatchWidth×1 =
//                      cn_ptr_wnd_last[i]=j genvar 宽度风格）。面积 −~150 flops（DC 原本修剪，QOR 不变）。
//   2026-09-20  OpenHappy Team  bak104：① cn_ptr_wnd_last 旧手写常量表删除（bak103 已改 generate 内
//                      按越界不等式自推导，旧表 512 行 7–30、1024 行 3–7/16–30 共 24/31 行错值，
//                      且与 generate 赋值构成双驱动）；② last_cn_flag aggr 门控 |cn_ptr_nxt_b →
//                      gth_ptr_nz_pip = ~(last_pad｜last_dat) & ｜gth_sel（正常拍支：masked_aggr[last]
//                      =1 前提下与 |cn_ptr_nxt_b 恒等——无数据 VC（valid 门控）/无 pad（尾部填充）/
//                      B·A_ptr_int 类掩码=0，指针 OR 退化为几何 gth_sel），hold 支 {8{|cn_ptr_nxt_b}}
//                      → {8{cn_ptr_idx[5]}}（纯数据拍 |df_ptr = ｜(idx−31) = idx[5]，逐 cursor 值恒等）；
//                      预期 6 个 aggr flag bit（现 L24 头）门控锥 −2~4 级。
//   2026-09-20  OpenHappy Team  bak103：CN 探针寄存器化（x_cn_get_r / wfull_bot_r / wlite_bot_r /
//                      w_data_bot1_r / rfull_bot_r / rlite_bot_r / r_data_bot1_r，next-state 解码：
//                      D = next-state 解码（源头统一：slot 缓冲直解 x_buf_nxt 各槽 CN——K 态不变式
//                      保证 slot0 rsft/lsft 互斥无 OR 冲突；w/r 解码 w/r_buf_nxt 底部/次 CN），
//                      与缓冲同更新拍采样 ⇒ 探针寄存器
//                      与组合版逐拍严格同步（归纳：更新拍后 = 组合(下一状态)；非更新拍缓冲/探针
//                      同冻结；复位基 0）⇒ 全部消费端重连为寄存器，逐拍位恒等、行为零变化
//                      （纯译码锥重定时，FM 可核）；排空锥起点去掉 CN 等值比较 + pop 宽度决策
//                      （b/ar 排空族 L17-22 → ~L12-14，w 自锥 B 族 L23 → ~L14-16）；旧组合探针
//                      wfull_bot/wlite_bot/w_data_bot1/w_bot1_at_*、rfull_bot/rlite_bot/r_data_bot1
//                      已删；slot 组合 x_cn_get 向量亦已删（D 直解 x_buf_nxt 后无消费者）；
//                      slot 探针寄存器 [8:5] 条件编译（1024 寄存器 / 512 always @(*) 置 0）、
//                      paif_phs_r 条件编译（仅 512 存在时钟寄存器；1024 无赋值、无消费者）；
//                      2a one-hot 行选：cn_ptr_oh_b（31 并行等值）+ AND-OR 树替换
//                      cn_ptr_wnd_sel_byp 的 31:1 数据行选（A 族 L24 → ~17–19；常量表行选不变）。
//   2026-09-20  OpenHappy Team  bak102：SP 字段预寄存，w_sp_r/r_sp_r 与 w/r_buf_r 同更新拍采样、
//                      滞后 w/r_buf 一个时钟周期（SP 输出拍 = VC 输出拍 + 1，用户确认的契约松弛）：
//                      VC 在 cycle n 输出（w/r_valid_int(n)=1，弹出于 edge(n+1)）⇒ w_sp_r(n+1)=输出拍底部
//                      SP（pop 前采样，真值）、w_sp_valid_r(n+1)=输出拍 ∧ SP≠0 ⇒ sp_valid 纯寄存器；
//                      sp_field/sp_valid 改从 w/r_sp_r 选 ⇒ b_hold 反压链寄存器→寄存器（断大类 I 28 级
//                      的 w 状态→SP 译码→b_hold 锥）；前置条件：C2（w/r 不同时输出 ⇒ sp 输出拍不冲突）、
//                      pack 序（SP 载荷的 b_vc 在其 w VC 之后 ⇒ b_vc 最早输出拍 = sp 滞后拍 ⇒ b_hold 恰阻塞之）、
//                      parent 接受 SP credit 合并滞后 1 拍；组合 w_valid_int 保留（w 端口 C4）；
//                      旧宽字段组合读 sp_wfull/wlite/rl_field 已删（其读端移入 w_sp_r D 锥）。
//   2026-09-21  OpenHappy Team  bak201：aw 段组合化（改组合读 *_buf 组合视图、删 *_cn_get_r 寄存器族）；对
//                      ar0~3/b 同步执行：CN 探针改组合、rdy 判据统一 *_buf_cnt_r（发射拍不吸收，吞吐 −1 拍）、
//                      输出 *_bus/*_valid 组合直读 *_buf（去 C4 输出级一拍）、_nxt 统一句式
//                      (~X_tx_proc) ? 保持 : drain（aw/ar 无 SP，与 partial 判据等价；b 因 C3 有 SP
//                      阻塞：合法 b 弹出受 ~sp_out_beat 门控、排底 flush 非 b 垃圾不受 SP 阻塞、K 补全
//                      遇 SP 保留缓冲下拍再弹防吞拍）。b 输出 b_rp0~3 & credit 同 bak200 已组合化；
//                      最终 ar/b 端口段内直驱、清 *_nxt 中间信号。
//   2026-09-23  OpenHappy Team（bak201 文件内后续修订，无独立 bak202 版本）：并行清理——
//                      ① 输出级/声明冗余：五类 credit × 4RP 的 40 个 *_crd_*_r（C4 输出级寄存器）
//                      为纯死寄存器（输出早已直连 *_nxt、busy 不引用）→ 删 20 组 always + 40 行声明，
//                      credit 端口改组合直出（C5 仍由 *_nxt 的 valid 门控保证 bus 归 0）；
//                      ② w/r 段 16 条死声明删除（rsft_*/lsft_w_new_*/lsft_w_part_w*/
//                      _buf_cnt_df/_data_bot/_data_bot1_nxt/_data_bot1_r 等，组合化后遗留）；
//                      ③ 重复/过时注释删除（SP type0/1 ×2、bvc 门控 ×5、version history 更新）；
//                      ④ aw/ar/b nxt 统一 (~tx_proc) ? 保持 : drain（aw/ar 无 SP 与 partial 判据等价，
//                      b 因 C3 用 ~b_tx_proc 区分 SP 只挡合法弹出/排底不受阻/ K 补全防吞拍）；
//                      ⑤ aw/ar 端口段内直驱（清 *_nxt）、w_bus 组合直读补不变式注释
//                      （w 全跨拍、补全拍即弹、新 VC 头落上半 ⇒ 无跨 VC OR 污染）。
//   ---------------------------------------------------------------------------
//   2026-10-19  OpenHappy Team  bak202（独立文件副本，架构微调 + 命名统一 + 死声明清理）：
//                      ① 命名统一：byp 级组合统一后缀 _byp（原 _b），pip 级寄存统一后缀
//                      _pip_r（原 _b_r / _nxt_b_r / _r），所有注释同步更新；
//                      ② **行选索引去寄存**：cn_ptr_idx 由「寄存的后继游标 cn_ptr_nxt_b_r」
//                      改为「pip 游标经后继函数组合派生」cn_ptr_idx = cn_ptr_byp；
//                      等价性：设 S=cn_ptr_pip_r（pip 游标）、R=原 cn_ptr_nxt_b_r，两者同在
//                      pld_byp_hsk 拍以 succ() 递推且同复位 0 ⇒ R(t) ≡ succ(S(t))，故
//                      原 idx = R(t) ≡ succ(S(t)) = 新 idx（succ 计算从寄存器搬回组合）；
//                      ③ **表查找索引拆分**：CN 行选 cn_ptr_wnd_sel_byp 用 cn_ptr_idx（byp 内容行）；
//                      B/A_ptr、dat_vdn 常量表改用 cn_ptr_pip_r（pip 内容行，其后继即 byp 游标），
//                      与 ② 的组合后继函数同源一致；
//                      ④ **预译码整体寄存化**：byp 拍译码结果（cn_ptr_wnd_valid / pad·dat·B·A·
//                      aw·ar0~3·b·wfull·wlite·rfull·rlite_cn / pld_only_dat / cn_ptr_wnd_gth_ptr_sel /
//                      cn_ptr_wnd_last_sel）在 pld_byp_hsk 拍整体寄存为 *_pip_r，pip 侧
//                      *_int_masked 由（*_pip_r ∧ valid_pip_r）组合派生（原为 *_b 组合直取）；
//                      ⑤ **req / vdn / vc_index 去寄存器**：删 aw_req_r~b_req_r(8)、*_vdn_b_r(8)、
//                      *_vc_index_b_r(7)、cn_ptr_nxt_b_r、w/r_int_masked_any_b_r(2) 共 26 个寄存器，
//                      改为由寄存器输入（*_int_masked_pip / last_cn_flag_pre_r / *_vc_index_pip）
//                      组合派生 ⇒ 无组合环（输入全为寄存器输出），反压入口仍无 OR 级；
//                      ⑥ **last_cn_flag 双寄存器合单**：原 A=last_cn_flag_nxt_b_r（byp_hsk，byp flag）
//                      + B=last_cn_flag_r（pip_hsk，= A 延迟一拍，pip flag）→ 合并为单个
//                      last_cn_flag_pre_r（pip_hsk 采样 last_cn_flag_pip，pip flag），
//                      其值 ≡ 原 B，故用于 *_pld_in / *_vdn_pip / *_req 三处均位等；
//                      ⑦ **valid 去 pld_byp_vld 门控**：cn_ptr_wnd_valid_byp 只保留
//                      (~dat_mask & ~pld_only_dat)，因唯一消费者 *_pip_r 仅在
//                      pld_byp_hsk(=vld & rdy) 拍采样，采样边 vld 恒 1 ⇒ 门控冗余；
//                      X 安全由 pld_only_dat = idx[5] | (&idx[4:0]) 覆盖 idx≥31 保证；
//                      ⑧ 死声明清理：删 bak79/bak100 遗留的 _b / _b_r 命名族（req/vdn/vc_index/
//                      后继游标/last_cn_flag_nxt）共 46 条零引用零驱动声明，删 last_cn_flag_pip_r；
//                      ⑨ 修正 b_req 缺右括号语法错、补全 40 条未声明信号（pip 预译码寄存器族、
//                      *_vc_index_pip / *_vdn_pip / *_ptr_byp / last_cn_flag_pre_r）；
//                      ⑩ 修正 w/r_int_masked_any_pip、last_cn_flag_pip 的 reg↔wire 声明类型。
//                      回归：VCS 0 Error 0 Warning；SpyGlass lint_rtl 0 Fatal/0 Error。
// =============================================================================

module p_axi4_rwch_unpack #(
    parameter PAIF_DW = 512
) (
    // 时钟与复位
    input  wire                    clk,
    input  wire                    rstn,
    output wire                    busy,

    // AW 通道（本 lane W_RP）
    output wire [106:0]            aw_bus,
    output wire                    aw_valid,

    // AR 通道（4 个 RP）
    output wire [106:0]            ar_rp0_bus,
    output wire                    ar_rp0_valid,
    output wire [106:0]            ar_rp1_bus,
    output wire                    ar_rp1_valid,
    output wire [106:0]            ar_rp2_bus,
    output wire                    ar_rp2_valid,
    output wire [106:0]            ar_rp3_bus,
    output wire                    ar_rp3_valid,

    // W 通道（本 lane W_RP）
    output wire [PAIF_DW+PAIF_DW/8+4:0] w_bus,
    output wire                    w_valid,

    // R 通道（本 lane R_RP）
    output wire [PAIF_DW+16:0]     r_bus,
    output wire                    r_valid,

    // B 通道（4 个 RP）
    output wire [15:0]             b_rp0_bus,
    output wire                    b_rp0_valid,
    output wire [15:0]             b_rp1_bus,
    output wire                    b_rp1_valid,
    output wire [15:0]             b_rp2_bus,
    output wire                    b_rp2_valid,
    output wire [15:0]             b_rp3_bus,
    output wire                    b_rp3_valid,

    // AW Credit RP0~3
    output wire [1:0]              awcrd_rp0,
    output wire                    awcrd_rp0_valid,
    output wire [1:0]              awcrd_rp1,
    output wire                    awcrd_rp1_valid,
    output wire [1:0]              awcrd_rp2,
    output wire                    awcrd_rp2_valid,
    output wire [1:0]              awcrd_rp3,
    output wire                    awcrd_rp3_valid,

    // W Credit RP0~3
    output wire [1:0]              wcrd_rp0,
    output wire                    wcrd_rp0_valid,
    output wire [1:0]              wcrd_rp1,
    output wire                    wcrd_rp1_valid,
    output wire [1:0]              wcrd_rp2,
    output wire                    wcrd_rp2_valid,
    output wire [1:0]              wcrd_rp3,
    output wire                    wcrd_rp3_valid,

    // AR Credit RP0~3
    output wire [1:0]              arcrd_rp0,
    output wire                    arcrd_rp0_valid,
    output wire [1:0]              arcrd_rp1,
    output wire                    arcrd_rp1_valid,
    output wire [1:0]              arcrd_rp2,
    output wire                    arcrd_rp2_valid,
    output wire [1:0]              arcrd_rp3,
    output wire                    arcrd_rp3_valid,

    // R Credit RP0~3
    output wire [1:0]              rcrd_rp0,
    output wire                    rcrd_rp0_valid,
    output wire [1:0]              rcrd_rp1,
    output wire                    rcrd_rp1_valid,
    output wire [1:0]              rcrd_rp2,
    output wire                    rcrd_rp2_valid,
    output wire [1:0]              rcrd_rp3,
    output wire                    rcrd_rp3_valid,

    // B Credit RP0~3
    output wire [1:0]              bcrd_rp0,
    output wire                    bcrd_rp0_valid,
    output wire [1:0]              bcrd_rp1,
    output wire                    bcrd_rp1_valid,
    output wire [1:0]              bcrd_rp2,
    output wire                    bcrd_rp2_valid,
    output wire [1:0]              bcrd_rp3,
    output wire                    bcrd_rp3_valid,

    // PAIF RX 接口（per-lane）
    input  wire [PAIF_DW-1:0]      paif_rxdata,
    input  wire                    paif_rxvalid,
    output wire                    paif_rxready
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    // --- 物理通道合并信号位宽（CN/RP 头之下的数据部分 + 用户字段）---
    localparam AX_WIDTH        = 107;                        // AW/AR 合并信号位宽
    localparam W_WIDTH         = PAIF_DW + PAIF_DW/8 + 5;    // W 合并：WDATA+WSTRB+WLAST+WUSER
    localparam R_WIDTH         = PAIF_DW + 17;               // R full 合并：WDATA+RRESP+RID+RLAST+RUSER
    localparam R_LITE_WIDTH    = PAIF_DW + 5;                // R lite 合并：WDATA+RLAST+RUSER

    // --- VC 位宽（含 7bit CN+RP 头；C5：PRF 区不处理，视为保留位）---
    localparam AX_VC_WIDTH     = 128;                        // AW/AR/B VC
    localparam WFULL_VC_WIDTH  = PAIF_DW + PAIF_DW/8 + 32;   // W full VC（顶 20bit = SP）
    localparam WLITE_VC_WIDTH  = PAIF_DW + 32;               // W lite VC（顶 20bit = SP）
    localparam R_VC_WIDTH      = PAIF_DW + 32;               // R full/lite VC 同宽（顶 20bit：rfull 无 SP/rlite 有 SP）

    // --- 移位粒度 / 单拍 payload（C1：payload 内内容恒为前缀，pad 为后缀）---
    localparam GRAN_WIDTH      = (PAIF_DW*32)/1024;          // 2Byte@512 / 4Byte@1024
    localparam PLD_WIDTH       = PAIF_DW - GRAN_WIDTH;       // 62Byte@512 / 124Byte@1024

    // --- 缓冲位宽 = 一个 VC + 一拍 payload - 一个粒度（最坏拼合占用）---
    localparam AX_BUF_WIDTH    = AX_VC_WIDTH + PLD_WIDTH - GRAN_WIDTH;
    localparam W_BUF_WIDTH     = WFULL_VC_WIDTH + PLD_WIDTH - GRAN_WIDTH;
    localparam R_BUF_WIDTH     = R_VC_WIDTH + PLD_WIDTH - GRAN_WIDTH;

    // --- 以 GRAN 计数的容量 ---
    localparam AX_VC_CNT       = AX_VC_WIDTH   / GRAN_WIDTH;
    localparam WFULL_VC_CNT    = WFULL_VC_WIDTH/ GRAN_WIDTH;
    localparam WLITE_VC_CNT    = WLITE_VC_WIDTH/ GRAN_WIDTH;
    localparam R_VC_CNT        = R_VC_WIDTH    / GRAN_WIDTH;
    localparam PLD_CNT         = PLD_WIDTH     / GRAN_WIDTH;

    localparam AX_BUF_CNT_W    = $clog2(AX_BUF_WIDTH/GRAN_WIDTH);
    localparam DAT_BUF_CNT_W   = $clog2(W_BUF_WIDTH/GRAN_WIDTH);
    // 扫描行槽数 = 向上取整：保证每行恰有一个"越界槽"（gth_ptr/last 全行驱动）
    localparam PLD_CN_NUM      = ((PLD_WIDTH%AX_VC_WIDTH) == 0) ? (PLD_WIDTH/AX_VC_WIDTH) : ((PLD_WIDTH/AX_VC_WIDTH)+1);

    // --- 桶形移位各级粒度（1/2/4/8/16/32 GRAN）---
    localparam S0_GRAN = 1;
    localparam S1_GRAN = 2;
    localparam S2_GRAN = 4;
    localparam S3_GRAN = 8;
    localparam S4_GRAN = 16;
    localparam S5_GRAN = 32;

    genvar i, j;
    integer k;

    // ==========================================================================
    // 内部信号定义（按功能分组，全部声明先于使用）
    // ==========================================================================

    // --------------------------------------------------------------------------
    // 1. paif 相位 / payload / 握手
    // --------------------------------------------------------------------------
    reg  paif_phs_r;                       // 512 双拍相位（2B 开销位置上下交替；[bak103] 仅 512 模式存在：1024 无赋值、无消费者）
    wire [PLD_WIDTH-1:0] payload;          // 当拍 payload（去开销后）
    wire paif_hsk;                         // PAIF RX 握手

    // --------------------------------------------------------------------------
    // 2. byp/pip 两级流水（payload 寄存）
    // --------------------------------------------------------------------------
    wire [PLD_WIDTH-1:0] pld_byp;
    wire pld_byp_vld;
    wire pld_byp_rdy;
    wire [PLD_WIDTH-1:0] pld_pip;
    wire pld_pip_vld;
    wire pld_pip_rdy;
    wire pld_pip_hsk;                      // 流水前进 = 游标/flag/缓冲吸收的统一节拍

    // --------------------------------------------------------------------------
    // 3. 游标 / 扫描预译码表（行 = 游标 GRAN 位置，槽 = 行内 128bit VC 候选位）
    //    [bak79 引入 / bak202 调整] 预译码前推 byp 级：byp 组合（后缀 _byp，源 pld_byp /
    //    行选索引 cn_ptr_idx = cn_ptr_byp）→ pld_byp_hsk 拍整体寄存为 *_pip_r →
    //    pip 侧 *_int_masked 由（*_pip_r ∧ valid_pip_r）组合派生。几何常量表不变。
    // --------------------------------------------------------------------------
    wire pld_only_dat_byp;                   // 纯数据拍（游标 31..37），byp 版（用 cn_ptr_idx）
    wire pld_byp_hsk;                      // byp→pip 转移拍
    wire [5:0] cn_ptr_idx;                 // byp 预译码行选索引 = P 进入 pip 时将看到的游标
    wire [PLD_WIDTH-1:0] rsft_pld_b_s0, rsft_pld_b_s1, rsft_pld_b_s2, rsft_pld_b_s3, rsft_pld_b_s4;
    wire [PLD_WIDTH-1:0] rsft_vc_byp [PLD_CN_NUM-1:0];    // byp 组合：游标对齐后各槽视图
    wire [PLD_WIDTH-1:0] rsft_vc [PLD_CN_NUM-1:0];      // = rsft_vc_pip_r（alias）
    wire [PLD_CN_NUM*7-1:0]   cn_ptr_wnd_b [PLD_CNT-1:0];          // 每槽头部 7bit（CN+RP），源 pld_byp
    wire [PLD_CN_NUM*6-1:0]   cn_ptr_wnd_B_ptr [PLD_CNT-1:0];      // （几何常量）wfull 后继游标
    wire [PLD_CN_NUM*6-1:0]   cn_ptr_wnd_A_ptr [PLD_CNT-1:0];      // （几何常量）lite/r 后继游标
    wire [5:0]                cn_ptr_wnd_gth_ptr [PLD_CNT-1:0];    // （几何常量）行末聚合 VC 后继游标
    wire [$clog2(PLD_CN_NUM)-1:0] cn_ptr_wnd_last [PLD_CNT-1:0];   // （几何常量）行末越界槽号
    wire [PLD_CN_NUM*6-1:0]     cn_ptr_wnd_dat_vdn [PLD_CNT-1:0];  // （几何常量）槽距 payload 尾
    wire [6:0]                cn_ptr_wnd_sel_byp [PLD_CN_NUM-1:0];
    wire [5:0]                cn_ptr_wnd_B_ptr_sel_pip [PLD_CN_NUM-1:0];
    wire [5:0]                cn_ptr_wnd_A_ptr_sel_pip [PLD_CN_NUM-1:0];
    wire [5:0]                 cn_ptr_wnd_dat_vdn_sel_pip [PLD_CN_NUM-1:0];
    wire [5:0]                cn_ptr_wnd_gth_ptr_sel_byp;
    wire [$clog2(PLD_CN_NUM)-1:0] cn_ptr_wnd_last_sel_byp;

    // --------------------------------------------------------------------------
    // 4. 译码 / 数据 VC 掩码 / 有效 / masked（byp 组合 _byp + 寄存 _pip_r）
    // --------------------------------------------------------------------------
    wire [PLD_CN_NUM-1:0] wfull_cn_byp;
    wire [PLD_CN_NUM-1:0] wlite_cn_byp;
    wire [PLD_CN_NUM-1:0] rfull_cn_byp;
    wire [PLD_CN_NUM-1:0] rlite_cn_byp;
    wire [PLD_CN_NUM-1:0] ar0_cn_byp;
    wire [PLD_CN_NUM-1:0] ar1_cn_byp;
    wire [PLD_CN_NUM-1:0] ar2_cn_byp;
    wire [PLD_CN_NUM-1:0] ar3_cn_byp;
    wire [PLD_CN_NUM-1:0] aw_cn_byp;
    wire [PLD_CN_NUM-1:0] b_cn_byp;
    wire [PLD_CN_NUM-1:0] pad_cn_byp;
    // ---- [bak202 死声明区-1] bak79 req 寄存器：bak202 已改组合直出（见 req 节），
    //      本批 8 个零引用零驱动，待清理 ----
    reg  aw_req_r;        // [bak79 遗留/死] 原 req 寄存器
    reg  ar0_req_r;       // [bak79 遗留/死]
    reg  ar1_req_r;       // [bak79 遗留/死]
    reg  ar2_req_r;       // [bak79 遗留/死]
    reg  ar3_req_r;       // [bak79 遗留/死]
    reg  w_req_r;
    reg  r_req_r;
    reg  b_req_r;
    wire [PLD_CN_NUM-1:0] dat_cn_byp;
    wire [PLD_CN_NUM-1:0] A_cn_byp;
    wire [PLD_CN_NUM-1:0] B_cn_byp;
    wire [PLD_CN_NUM-1:0] cn_ptr_wnd_dat_mask_byp;   // [bak101-2c] 并行前缀 OR（见组合逻辑节；原串行累加/下三角常量表已删）
    wire [PLD_CN_NUM-1:0] cn_ptr_wnd_valid_byp;                  // byp 拍可解码位
    wire [PLD_CN_NUM-1:0] pad_int_masked_pip;
    wire [PLD_CN_NUM-1:0] dat_int_masked_pip;
    wire [PLD_CN_NUM-1:0] B_int_masked_pip;
    wire [PLD_CN_NUM-1:0] A_int_masked_pip;
    wire [PLD_CN_NUM-1:0] aw_int_masked_pip;
    wire [PLD_CN_NUM-1:0] ar0_int_masked_pip;
    wire [PLD_CN_NUM-1:0] ar1_int_masked_pip;
    wire [PLD_CN_NUM-1:0] ar2_int_masked_pip;
    wire [PLD_CN_NUM-1:0] ar3_int_masked_pip;
    wire [PLD_CN_NUM-1:0] b_int_masked_pip;
    wire [PLD_CN_NUM-1:0] wfull_int_masked_pip;
    wire [PLD_CN_NUM-1:0] wlite_int_masked_pip;
    wire [PLD_CN_NUM-1:0] rfull_int_masked_pip;
    wire [PLD_CN_NUM-1:0] rlite_int_masked_pip;
    wire [PLD_CN_NUM-1:0] w_int_masked_pip;
    wire [PLD_CN_NUM-1:0] r_int_masked_pip;
    wire last_pad_pip;
    wire last_dat_pip;

    // --------------------------------------------------------------------------
    // 5. 指针后继 / last_cn_flag（byp 组合 _byp；跨拍续传标记）
    // --------------------------------------------------------------------------
    wire [5:0] B_ptr_nxt_b;
    wire [5:0] A_ptr_nxt_b;
    wire [5:0] gth_ptr_nxt_b;
    wire [5:0] df_ptr_nxt_b;
    wire       gth_ptr_nz_pip;                   // [bak104] 行末真跨拍门控（~last_pad_pip/~last_dat_pip & |cn_ptr_wnd_gth_ptr_sel_pip_r；flag 支用，等价 |cn_ptr_byp）
    reg  [5:0] B_ptr_int_pip;
    reg  [5:0] A_ptr_int_pip;
    wire [5:0] cn_ptr_nxt_b;
    wire [5:0] cn_ptr_byp;
    reg  [5:0] cn_ptr_pip_r;
    // [FIX] last_cn_flag_pip 原先重复声明为 reg 与 wire，且实际由 assign 驱动（现 line 1046）→
    //       仅保留唯一 wire 声明。last_cn_flag_nxt 为 bak79 遗留 alias（零引用零驱动，待清理）。
    wire [7:0] last_cn_flag_nxt;                 // [bak202] 死声明（bak79 alias 遗留）
    wire [7:0] last_cn_flag_pip;                 // 组合：pip 内容 flag = f(*_int_masked_pip, last_cn_flag_pre_r)

    // --------------------------------------------------------------------------
    // 6. vc_index（行内首个本通道 VC 槽号）/ vdn（本拍吸收的数据量，GRAN）
    // --------------------------------------------------------------------------
    wire [$clog2(PLD_CN_NUM)-1:0] dat_vc_index;   // = dat_vc_index_pip（alias）
    wire [$clog2(PLD_CN_NUM)-1:0] aw_vc_index;    // = aw_vc_index_pip（alias）
    wire [$clog2(PLD_CN_NUM)-1:0] ar0_vc_index;   // = ar0_vc_index_pip（alias）
    wire [$clog2(PLD_CN_NUM)-1:0] ar1_vc_index;   // = ar1_vc_index_pip（alias）
    wire [$clog2(PLD_CN_NUM)-1:0] ar2_vc_index;   // = ar2_vc_index_pip（alias）
    wire [$clog2(PLD_CN_NUM)-1:0] ar3_vc_index;   // = ar3_vc_index_pip（alias）
    wire [$clog2(PLD_CN_NUM)-1:0] b_vc_index;     // = b_vc_index_pip（alias）
    wire [$clog2(PLD_CN_NUM)-1:0] dat_vc_index_b;   // [bak101-2b] 并行 first-hit（见组合逻辑节）
    wire [$clog2(PLD_CN_NUM)-1:0] aw_vc_index_b;
    wire [$clog2(PLD_CN_NUM)-1:0] ar0_vc_index_b;
    wire [$clog2(PLD_CN_NUM)-1:0] ar1_vc_index_b;
    wire [$clog2(PLD_CN_NUM)-1:0] ar2_vc_index_b;
    wire [$clog2(PLD_CN_NUM)-1:0] ar3_vc_index_b;
    wire [$clog2(PLD_CN_NUM)-1:0] b_vc_index_b;
    // [bak101-2b] vc_index 并行 first-hit 中间向量：p[j] = 低编号槽任一命中；g = 首个命中槽 one-hot（或全 0）
    wire [PLD_CN_NUM-1:0] dat_p, dat_g;
    wire [PLD_CN_NUM-1:0] aw_p,  aw_g;
    wire [PLD_CN_NUM-1:0] ar0_p, ar0_g;
    wire [PLD_CN_NUM-1:0] ar1_p, ar1_g;
    wire [PLD_CN_NUM-1:0] ar2_p, ar2_g;
    wire [PLD_CN_NUM-1:0] ar3_p, ar3_g;
    wire [PLD_CN_NUM-1:0] b_p,   b_g;
    // [FIX] 缺失声明补全（pip 侧并行 first-hit 索引，assign 驱动，原来无任何声明）：
    //   vc_index_pip 位宽同 vc_index（$clog2(PLD_CN_NUM)；4 槽→2bit / 8 槽→3bit）
    wire [$clog2(PLD_CN_NUM)-1:0] dat_vc_index_pip;
    wire [$clog2(PLD_CN_NUM)-1:0] aw_vc_index_pip;
    wire [$clog2(PLD_CN_NUM)-1:0] ar0_vc_index_pip;
    wire [$clog2(PLD_CN_NUM)-1:0] ar1_vc_index_pip;
    wire [$clog2(PLD_CN_NUM)-1:0] ar2_vc_index_pip;
    wire [$clog2(PLD_CN_NUM)-1:0] ar3_vc_index_pip;
    wire [$clog2(PLD_CN_NUM)-1:0] b_vc_index_pip;
    // [FIX] 缺失声明补全（pip 侧后继游标分量，6bit，同 df_ptr_byp/gth_ptr_byp）
    wire [5:0] df_ptr_byp;
    wire [5:0] gth_ptr_byp;
    wire [5:0] B_ptr_byp;
    wire [5:0] A_ptr_byp;

    // 预译码寄存器（byp→pip 转移拍 pld_byp_hsk 更新；否则保持 = 当前 pip 内容的预译码）
    // ---- [bak202 死声明区-2] bak79/bak100 的 _b_r 命名族（已改 _pip 组合派生），
    //      本批 7+1 个零引用零驱动，待清理 ----
    reg [$clog2(PLD_CN_NUM)-1:0] dat_vc_index_b_r;   // [bak202 死] 改 *_vc_index_pip
    reg [$clog2(PLD_CN_NUM)-1:0] aw_vc_index_b_r;    // [bak202 死]
    reg [$clog2(PLD_CN_NUM)-1:0] ar0_vc_index_b_r;   // [bak202 死]
    reg [$clog2(PLD_CN_NUM)-1:0] ar1_vc_index_b_r;   // [bak202 死]
    reg [$clog2(PLD_CN_NUM)-1:0] ar2_vc_index_b_r;   // [bak202 死]
    reg [$clog2(PLD_CN_NUM)-1:0] ar3_vc_index_b_r;   // [bak202 死]
    reg [$clog2(PLD_CN_NUM)-1:0] b_vc_index_b_r;     // [bak202 死]
    reg [PLD_WIDTH-1:0] rsft_vc_pip_r [PLD_CN_NUM-1:0];   // 活：byp_hsk 采样 rsft_vc_byp
    reg [5:0] cn_ptr_nxt_b_r;                        // [bak202 死] 改 cn_ptr_pip_r
    // [FIX] 缺失声明补全：pip 拍寄存的译码/行选结果（均在上方 pld_byp_hsk 寄存器块内被 <= 赋值，
    //       位宽与其 byp 源一致；原有代码完全未声明，靠隐式 net 侥幸编译，全文件仅 _pip_r 这一批缺）
    reg [PLD_CN_NUM-1:0] cn_ptr_wnd_valid_pip_r;      // <= cn_ptr_wnd_valid_byp
    reg                  pld_only_dat_pip_r;          // <= pld_only_dat_byp
    reg [PLD_CN_NUM-1:0] pad_cn_pip_r;                // <= pad_cn_byp
    reg [PLD_CN_NUM-1:0] dat_cn_pip_r;                // <= dat_cn_byp
    reg [PLD_CN_NUM-1:0] B_cn_pip_r;                  // <= B_cn_byp
    reg [PLD_CN_NUM-1:0] A_cn_pip_r;                  // <= A_cn_byp
    reg [PLD_CN_NUM-1:0] aw_cn_pip_r;                 // <= aw_cn_byp
    reg [PLD_CN_NUM-1:0] ar0_cn_pip_r;                // <= ar0_cn_byp
    reg [PLD_CN_NUM-1:0] ar1_cn_pip_r;                // <= ar1_cn_byp
    reg [PLD_CN_NUM-1:0] ar2_cn_pip_r;                // <= ar2_cn_byp
    reg [PLD_CN_NUM-1:0] ar3_cn_pip_r;                // <= ar3_cn_byp
    reg [PLD_CN_NUM-1:0] b_cn_pip_r;                  // <= b_cn_byp
    reg [PLD_CN_NUM-1:0] wfull_cn_pip_r;              // <= wfull_cn_byp
    reg [PLD_CN_NUM-1:0] wlite_cn_pip_r;              // <= wlite_cn_byp
    reg [PLD_CN_NUM-1:0] rfull_cn_pip_r;              // <= rfull_cn_byp
    reg [PLD_CN_NUM-1:0] rlite_cn_pip_r;              // <= rlite_cn_byp
    reg [5:0]            cn_ptr_wnd_gth_ptr_sel_pip_r; // <= cn_ptr_wnd_gth_ptr_sel_byp
    reg [$clog2(PLD_CN_NUM)-1:0] cn_ptr_wnd_last_sel_pip_r; // <= cn_ptr_wnd_last_sel_byp
    // [FIX] w/r_int_masked_any_pip 原误声明为 reg，但实际由 assign 驱动（line 1192/1193）→ 改为 wire
    wire w_int_masked_any_pip;  // [bak100③] w 命中 any 归约（= |w_int_masked_pip；lsft_w_new 使能）
    wire r_int_masked_any_pip;  // [bak100③] r 命中 any 归约（= |r_int_masked_pip；同上）
    // ---- [bak202 死声明区-3] bak100④ 的 vdn _b/_b_r 命名族：
    //      bak202 已改 *_vdn_pip 组合直出，本批 18 个零引用零驱动，待清理 ----
    wire [5:0] dat_vdn_b;                        // [bak202 死]
    wire [AX_BUF_CNT_W-1:0] aw_vdn_b;            // [bak202 死]
    wire [AX_BUF_CNT_W-1:0] ar0_vdn_b;           // [bak202 死]
    wire [AX_BUF_CNT_W-1:0] ar1_vdn_b;           // [bak202 死]
    wire [AX_BUF_CNT_W-1:0] ar2_vdn_b;           // [bak202 死]
    wire [AX_BUF_CNT_W-1:0] ar3_vdn_b;           // [bak202 死]
    wire [AX_BUF_CNT_W-1:0] b_vdn_b;             // [bak202 死]
    wire [5:0] w_vdn_b;                          // [bak202 死]
    wire [5:0] r_vdn_b;                          // [bak202 死]
    reg  [AX_BUF_CNT_W-1:0] aw_vdn_b_r;          // [bak202 死]
    reg  [AX_BUF_CNT_W-1:0] ar0_vdn_b_r;         // [bak202 死]
    reg  [AX_BUF_CNT_W-1:0] ar1_vdn_b_r;         // [bak202 死]
    reg  [AX_BUF_CNT_W-1:0] ar2_vdn_b_r;         // [bak202 死]
    reg  [AX_BUF_CNT_W-1:0] ar3_vdn_b_r;         // [bak202 死]
    reg  [AX_BUF_CNT_W-1:0] b_vdn_b_r;           // [bak202 死]
    reg  [5:0] w_vdn_b_r;                        // [bak202 死]
    reg  [5:0] r_vdn_b_r;                        // [bak202 死]
    wire [AX_BUF_CNT_W-1:0] aw_vdn;    // = aw_vdn_pip（alias）
    wire [AX_BUF_CNT_W-1:0] ar0_vdn;   // = ar0_vdn_pip（alias）
    wire [AX_BUF_CNT_W-1:0] ar1_vdn;   // = ar1_vdn_pip（alias）
    wire [AX_BUF_CNT_W-1:0] ar2_vdn;   // = ar2_vdn_pip（alias）
    wire [AX_BUF_CNT_W-1:0] ar3_vdn;   // = ar3_vdn_pip（alias）
    wire [AX_BUF_CNT_W-1:0] b_vdn;     // = b_vdn_pip（alias）
    wire [5:0] w_vdn;                  // = w_vdn_pip（alias）
    wire [5:0] r_vdn;                  // = r_vdn_pip（alias）
    // [FIX] 缺失声明补全（pip 侧 vdn：dat 为 6bit；aw/ar/b 与 aw_vdn_b 同宽 AX_BUF_CNT_W；w/r 为 6bit）
    wire [5:0] dat_vdn_pip;
    wire [AX_BUF_CNT_W-1:0] aw_vdn_pip;
    wire [AX_BUF_CNT_W-1:0] ar0_vdn_pip;
    wire [AX_BUF_CNT_W-1:0] ar1_vdn_pip;
    wire [AX_BUF_CNT_W-1:0] ar2_vdn_pip;
    wire [AX_BUF_CNT_W-1:0] ar3_vdn_pip;
    wire [AX_BUF_CNT_W-1:0] b_vdn_pip;
    wire [5:0] w_vdn_pip;
    wire [5:0] r_vdn_pip;
    // [FIX] 缺失声明补全（pre 拍 flag 寄存器，always 驱动，8bit 同 last_cn_flag_pip）
    reg  [7:0] last_cn_flag_pre_r;

    // --------------------------------------------------------------------------
    // 7. req/rdy（反压锁步：req = 有内容(无环根)，rdy = 排空后残留 < 阈值）
    // --------------------------------------------------------------------------
    wire aw_req;
    wire ar0_req;
    wire ar1_req;
    wire ar2_req;
    wire ar3_req;
    wire w_req;
    wire r_req;
    wire b_req;
    wire aw_buf_rdy;
    wire ar0_buf_rdy;
    wire ar1_buf_rdy;
    wire ar2_buf_rdy;
    wire ar3_buf_rdy;
    wire w_buf_rdy;
    wire r_buf_rdy;
    wire b_buf_rdy;

    // --------------------------------------------------------------------------
    // 8. aw 缓冲（底部 = 最老 AW；K 态保留 / W0 清零 / D 优先级排空）
    // --------------------------------------------------------------------------
    reg  [AX_BUF_WIDTH-1:0] aw_buf_r;
    reg  [AX_BUF_CNT_W-1:0] aw_buf_cnt_r;
    wire [AX_BUF_WIDTH-1:0] aw_buf_nxt;
    wire [AX_BUF_WIDTH-1:0] aw_buf;
    wire [AX_BUF_CNT_W-1:0] aw_buf_cnt_nxt;
    wire [AX_BUF_CNT_W-1:0] aw_buf_cnt;
    wire aw_buf_hsk;
    wire aw_tx_proc;
    wire [8:0] aw_cn_get;                    // [bak103] 直解 D（x_buf_nxt 各槽 CN 解码）
    wire [AX_BUF_WIDTH-1:0] aw_pld_in;
    wire [AX_BUF_WIDTH-1:0] lsft_aw_s0, lsft_aw_s1, lsft_aw_s2, lsft_aw;

    // --------------------------------------------------------------------------
    // 9. ar0~ar3 缓冲（aw 同构，CN+RP 匹配）
    // --------------------------------------------------------------------------
    reg  [AX_BUF_WIDTH-1:0] ar0_buf_r;
    reg  [AX_BUF_CNT_W-1:0] ar0_buf_cnt_r;
    wire [AX_BUF_WIDTH-1:0] ar0_buf_nxt;
    wire [AX_BUF_WIDTH-1:0] ar0_buf;
    wire [AX_BUF_CNT_W-1:0] ar0_buf_cnt_nxt;
    wire [AX_BUF_CNT_W-1:0] ar0_buf_cnt;
    wire ar0_buf_hsk;
    wire ar0_tx_proc;
    wire [8:0] ar0_cn_get;                 // [bak201] 组合 CN 探针（原 ar0_cn_get_r 寄存器化改回）
    wire [AX_BUF_WIDTH-1:0] ar0_pld_in;
    wire [AX_BUF_WIDTH-1:0] lsft_ar0_s0, lsft_ar0_s1, lsft_ar0_s2, lsft_ar0;

    reg  [AX_BUF_WIDTH-1:0] ar1_buf_r;
    reg  [AX_BUF_CNT_W-1:0] ar1_buf_cnt_r;
    wire [AX_BUF_WIDTH-1:0] ar1_buf_nxt;
    wire [AX_BUF_WIDTH-1:0] ar1_buf;
    wire [AX_BUF_CNT_W-1:0] ar1_buf_cnt_nxt;
    wire [AX_BUF_CNT_W-1:0] ar1_buf_cnt;
    wire ar1_buf_hsk;
    wire ar1_tx_proc;
    wire [8:0] ar1_cn_get;                 // [bak201] 组合 CN 探针（原 ar1_cn_get_r 寄存器化改回）
    wire [AX_BUF_WIDTH-1:0] ar1_pld_in;
    wire [AX_BUF_WIDTH-1:0] lsft_ar1_s0, lsft_ar1_s1, lsft_ar1_s2, lsft_ar1;

    reg  [AX_BUF_WIDTH-1:0] ar2_buf_r;
    reg  [AX_BUF_CNT_W-1:0] ar2_buf_cnt_r;
    wire [AX_BUF_WIDTH-1:0] ar2_buf_nxt;
    wire [AX_BUF_WIDTH-1:0] ar2_buf;
    wire [AX_BUF_CNT_W-1:0] ar2_buf_cnt_nxt;
    wire [AX_BUF_CNT_W-1:0] ar2_buf_cnt;
    wire ar2_buf_hsk;
    wire ar2_tx_proc;
    wire [8:0] ar2_cn_get;                 // [bak201] 组合 CN 探针（原 ar2_cn_get_r 寄存器化改回）
    wire [AX_BUF_WIDTH-1:0] ar2_pld_in;
    wire [AX_BUF_WIDTH-1:0] lsft_ar2_s0, lsft_ar2_s1, lsft_ar2_s2, lsft_ar2;

    reg  [AX_BUF_WIDTH-1:0] ar3_buf_r;
    reg  [AX_BUF_CNT_W-1:0] ar3_buf_cnt_r;
    wire [AX_BUF_WIDTH-1:0] ar3_buf_nxt;
    wire [AX_BUF_WIDTH-1:0] ar3_buf;
    wire [AX_BUF_CNT_W-1:0] ar3_buf_cnt_nxt;
    wire [AX_BUF_CNT_W-1:0] ar3_buf_cnt;
    wire ar3_buf_hsk;
    wire ar3_tx_proc;
    wire [8:0] ar3_cn_get;                 // [bak201] 组合 CN 探针（原 ar3_cn_get_r 寄存器化改回）
    wire [AX_BUF_WIDTH-1:0] ar3_pld_in;
    wire [AX_BUF_WIDTH-1:0] lsft_ar3_s0, lsft_ar3_s1, lsft_ar3_s2, lsft_ar3;

    // --------------------------------------------------------------------------
    // 10. w 缓冲（两半模型，r 同构：下半 = 完整 w VC（宽按 wfull/wlite 选），
    //     上半 = 次 w 段；(ii) 不变式下次 w VC 恒紧邻，g>=1 量化搜索已移除）
    // --------------------------------------------------------------------------
    reg  [W_BUF_WIDTH-1:0] w_buf_r;
    reg  [DAT_BUF_CNT_W-1:0] w_buf_cnt_r;
    wire [W_BUF_WIDTH-1:0] w_buf;
    wire [W_BUF_WIDTH-1:0] w_buf_nxt;
    wire [DAT_BUF_CNT_W-1:0] w_buf_cnt;
    wire [DAT_BUF_CNT_W-1:0] w_buf_cnt_nxt;
    wire w_buf_hsk;
    wire w_tx_proc;
    reg  wfull_bot_r;                            // [bak103] 底部 w VC 是 wfull（next-state 解码，与 w_buf_r 严格同步）
    reg  wlite_bot_r;                            // [bak103] 底部 w VC 是 wlite
    wire wfull_bot_nxt;                          // [bak103] w_buf_nxt 底部 CN 解码（D 专用）
    wire wlite_bot_nxt;
    // [bak102] SP 字段预寄存（w）：w_sp_r/w_sp_valid_r 滞后 w_buf_r 一个时钟周期
    wire [19:0] w_sp_cur;                          // 当前底部 VC 的 SP（定基址读 + 1bit 类型选；输出拍 n 内为真值）
    wire        w_sp_evt;                          // 本拍底部 VC 输出 ∧ SP≠0（同 edge 采样 → 下拍 sp 输出）
    wire [W_BUF_WIDTH-1:0] w_pld_in;             // raw payload 零扩展至缓冲宽（桶形输入）
    wire [W_BUF_WIDTH-1:0] lsft_w_s0, lsft_w_s1, lsft_w_s2, lsft_w_s3, lsft_w_s4, lsft_w_s5, lsft_w;
    wire [W_BUF_WIDTH-1:0] lsft_w_partial;
    wire [W_BUF_WIDTH-1:0] lsft_w_new;
    wire [W_WIDTH-1:0] w_bus_wfull;
    wire [W_WIDTH-1:0] w_bus_wlite;

    // --------------------------------------------------------------------------
    // 11. r 缓冲（w 同构，rfull/rlite 同宽）
    // --------------------------------------------------------------------------
    reg  [R_BUF_WIDTH-1:0] r_buf_r;
    reg  [DAT_BUF_CNT_W-1:0] r_buf_cnt_r;
    wire [R_BUF_WIDTH-1:0] r_buf;
    wire [R_BUF_WIDTH-1:0] r_buf_nxt;
    wire [DAT_BUF_CNT_W-1:0] r_buf_cnt;
    wire [DAT_BUF_CNT_W-1:0] r_buf_cnt_nxt;
    wire r_buf_hsk;
    wire r_tx_proc;
    reg  rfull_bot_r;                           // [bak103] 底部 r VC 是 rfull（next-state 解码，与 r_buf_r 严格同步）
    reg  rlite_bot_r;                           // [bak103] 底部 r VC 是 rlite
    wire rfull_bot_nxt;                         // [bak103] r_buf_nxt 底部 CN 解码（D 专用）
    wire rlite_bot_nxt;
    // [bak102] SP 字段预寄存（r）：r_sp_r/r_sp_valid_r 滞后 r_buf_r 一个时钟周期
    wire [19:0] r_sp_cur;                          // 当前底部 r VC 的 SP（rlite 承载 / rfull 无 SP，类型门控）
    wire        r_sp_evt;                          // 本拍底部 r VC 输出 ∧ SP≠0（同 edge 采样 → 下拍 sp 输出）
    wire [R_BUF_WIDTH-1:0] r_pld_in;            // raw payload 零扩展至缓冲宽（桶形输入）
    wire [R_BUF_WIDTH-1:0] lsft_rd_s0, lsft_rd_s1, lsft_rd_s2, lsft_rd_s3, lsft_rd_s4, lsft_rd_s5, lsft_rd;
    wire [R_BUF_WIDTH-1:0] lsft_rd_partial;
    wire [R_BUF_WIDTH-1:0] lsft_rd_new;
    wire [R_WIDTH-1:0] r_bus_rfull;
    wire [R_LITE_WIDTH-1:0] r_lite_bus;
    wire [R_WIDTH-1:0] r_bus_lite_recon;
    reg  [9:0] rid_saved_r;                      // r_full 输出时保存，供 r_lite 重建
    reg  [1:0] rresp_saved_r;

    // --------------------------------------------------------------------------
    // 12. b 缓冲（aw 同构；弹出受 sp_out_beat 阻塞，C3）
    // --------------------------------------------------------------------------
    reg  [AX_BUF_WIDTH-1:0] b_buf_r;
    reg  [AX_BUF_CNT_W-1:0] b_buf_cnt_r;
    wire [AX_BUF_WIDTH-1:0] b_buf_nxt;
    wire [AX_BUF_WIDTH-1:0] b_buf;
    wire [AX_BUF_CNT_W-1:0] b_buf_cnt_nxt;
    wire [AX_BUF_CNT_W-1:0] b_buf_cnt;
    wire b_buf_hsk;                        // b 吸收握手（b_req & pld_pip_hsk）
    wire b_tx_proc;
    wire [8:0] b_cn_get;                 // [bak201] 组合 CN 探针（原 b_cn_get_r 寄存器化改回）
    wire [AX_BUF_WIDTH-1:0] b_pld_in;
    wire [AX_BUF_WIDTH-1:0] lsft_b_s0, lsft_b_s1, lsft_b_s2, lsft_b;
    wire [AX_VC_WIDTH-1:0] b_vc;                 // 底部 128bit b_vc

    // --------------------------------------------------------------------------
    // 13. SP 字段译码（数据 VC 顶 20bit，TYPE 自描述；r_full 无 SP）
    // --------------------------------------------------------------------------
    wire [19:0] sp_field;
    wire [1:0]  sp_type;
    wire        sp_valid;                        // SP 非零（pack 空 SP 填 0）
    wire        sp_out_beat;                     // SP 输出拍（C3：阻塞 b_vc 输出）
    wire [15:0] sp_b_bus;
    wire [1:0]  sp_b_rp_sel;
    wire [15:0] sp_b_rp0_bus_w;
    wire [15:0] sp_b_rp1_bus_w;
    wire [15:0] sp_b_rp2_bus_w;
    wire [15:0] sp_b_rp3_bus_w;
    wire sp_b_rp0_valid_w;
    wire sp_b_rp1_valid_w;
    wire sp_b_rp2_valid_w;
    wire sp_b_rp3_valid_w;
    wire [1:0]  sp_awcrd_rp0_w;
    wire [1:0]  sp_awcrd_rp1_w;
    wire [1:0]  sp_awcrd_rp2_w;
    wire [1:0]  sp_awcrd_rp3_w;
    wire [1:0]  sp_wcrd_rp0_w;
    wire [1:0]  sp_wcrd_rp1_w;
    wire [1:0]  sp_wcrd_rp2_w;
    wire [1:0]  sp_wcrd_rp3_w;
    wire [1:0]  sp_arcrd_rp0_w;
    wire [1:0]  sp_arcrd_rp1_w;
    wire [1:0]  sp_arcrd_rp2_w;
    wire [1:0]  sp_arcrd_rp3_w;
    wire [1:0]  sp_rcrd_rp0_w;
    wire [1:0]  sp_rcrd_rp1_w;
    wire [1:0]  sp_rcrd_rp2_w;
    wire [1:0]  sp_rcrd_rp3_w;
    wire [1:0]  sp_bcrd_rp0_w;
    wire [1:0]  sp_bcrd_rp1_w;
    wire [1:0]  sp_bcrd_rp2_w;
    wire [1:0]  sp_bcrd_rp3_w;
    wire sp_awcrd_rp0_valid_w;
    wire sp_awcrd_rp1_valid_w;
    wire sp_awcrd_rp2_valid_w;
    wire sp_awcrd_rp3_valid_w;
    wire sp_wcrd_rp0_valid_w;
    wire sp_wcrd_rp1_valid_w;
    wire sp_wcrd_rp2_valid_w;
    wire sp_wcrd_rp3_valid_w;
    wire sp_arcrd_rp0_valid_w;
    wire sp_arcrd_rp1_valid_w;
    wire sp_arcrd_rp2_valid_w;
    wire sp_arcrd_rp3_valid_w;
    wire sp_rcrd_rp0_valid_w;
    wire sp_rcrd_rp1_valid_w;
    wire sp_rcrd_rp2_valid_w;
    wire sp_rcrd_rp3_valid_w;
    wire sp_bcrd_rp0_valid_w;
    wire sp_bcrd_rp1_valid_w;
    wire sp_bcrd_rp2_valid_w;
    wire sp_bcrd_rp3_valid_w;

    // --------------------------------------------------------------------------
    // 14. b_vc 字段译码（128bit：CN|B_RPV|B_RP0..3|AWcrd|ARcrd|Wcrd|Rcrd|Bcrd|pad）
    // --------------------------------------------------------------------------
    wire bvc_valid;                              // = b_out_beat（C3 仲裁后）
    wire [15:0] bvc_b_rp0_bus_w;
    wire [15:0] bvc_b_rp1_bus_w;
    wire [15:0] bvc_b_rp2_bus_w;
    wire [15:0] bvc_b_rp3_bus_w;
    wire bvc_b_rp0_valid_w;
    wire bvc_b_rp1_valid_w;
    wire bvc_b_rp2_valid_w;
    wire bvc_b_rp3_valid_w;
    wire [1:0]  bvc_awcrd_rp0_w;
    wire [1:0]  bvc_awcrd_rp1_w;
    wire [1:0]  bvc_awcrd_rp2_w;
    wire [1:0]  bvc_awcrd_rp3_w;
    wire [1:0]  bvc_wcrd_rp0_w;
    wire [1:0]  bvc_wcrd_rp1_w;
    wire [1:0]  bvc_wcrd_rp2_w;
    wire [1:0]  bvc_wcrd_rp3_w;
    wire [1:0]  bvc_arcrd_rp0_w;
    wire [1:0]  bvc_arcrd_rp1_w;
    wire [1:0]  bvc_arcrd_rp2_w;
    wire [1:0]  bvc_arcrd_rp3_w;
    wire [1:0]  bvc_rcrd_rp0_w;
    wire [1:0]  bvc_rcrd_rp1_w;
    wire [1:0]  bvc_rcrd_rp2_w;
    wire [1:0]  bvc_rcrd_rp3_w;
    wire [1:0]  bvc_bcrd_rp0_w;
    wire [1:0]  bvc_bcrd_rp1_w;
    wire [1:0]  bvc_bcrd_rp2_w;
    wire [1:0]  bvc_bcrd_rp3_w;
    wire bvc_awcrd_rp0_valid_w;
    wire bvc_awcrd_rp1_valid_w;
    wire bvc_awcrd_rp2_valid_w;
    wire bvc_awcrd_rp3_valid_w;
    wire bvc_wcrd_rp0_valid_w;
    wire bvc_wcrd_rp1_valid_w;
    wire bvc_wcrd_rp2_valid_w;
    wire bvc_wcrd_rp3_valid_w;
    wire bvc_arcrd_rp0_valid_w;
    wire bvc_arcrd_rp1_valid_w;
    wire bvc_arcrd_rp2_valid_w;
    wire bvc_arcrd_rp3_valid_w;
    wire bvc_rcrd_rp0_valid_w;
    wire bvc_rcrd_rp1_valid_w;
    wire bvc_rcrd_rp2_valid_w;
    wire bvc_rcrd_rp3_valid_w;
    wire bvc_bcrd_rp0_valid_w;
    wire bvc_bcrd_rp1_valid_w;
    wire bvc_bcrd_rp2_valid_w;
    wire bvc_bcrd_rp3_valid_w;

    // --------------------------------------------------------------------------
    // 15. 输出级合并（SP 时隙 / b_vc 时隙，C3 互斥）
    //     AR/B/credit 组合直出（[bak108r]/[bak201 修订] 去输出级寄存器）；W/R/AW 组合输出
    // --------------------------------------------------------------------------
    wire [15:0] b_rp0_bus_nxt;
    wire b_rp0_valid_nxt;
    wire [15:0] b_rp1_bus_nxt;
    wire b_rp1_valid_nxt;
    wire [15:0] b_rp2_bus_nxt;
    wire b_rp2_valid_nxt;
    wire [15:0] b_rp3_bus_nxt;
    wire b_rp3_valid_nxt;
    wire [1:0]  awcrd_rp0_nxt;
    wire awcrd_rp0_valid_nxt;
    wire [1:0]  awcrd_rp1_nxt;
    wire awcrd_rp1_valid_nxt;
    wire [1:0]  awcrd_rp2_nxt;
    wire awcrd_rp2_valid_nxt;
    wire [1:0]  awcrd_rp3_nxt;
    wire awcrd_rp3_valid_nxt;
    wire [1:0]  wcrd_rp0_nxt;
    wire wcrd_rp0_valid_nxt;
    wire [1:0]  wcrd_rp1_nxt;
    wire wcrd_rp1_valid_nxt;
    wire [1:0]  wcrd_rp2_nxt;
    wire wcrd_rp2_valid_nxt;
    wire [1:0]  wcrd_rp3_nxt;
    wire wcrd_rp3_valid_nxt;
    wire [1:0]  arcrd_rp0_nxt;
    wire arcrd_rp0_valid_nxt;
    wire [1:0]  arcrd_rp1_nxt;
    wire arcrd_rp1_valid_nxt;
    wire [1:0]  arcrd_rp2_nxt;
    wire arcrd_rp2_valid_nxt;
    wire [1:0]  arcrd_rp3_nxt;
    wire arcrd_rp3_valid_nxt;
    wire [1:0]  rcrd_rp0_nxt;
    wire rcrd_rp0_valid_nxt;
    wire [1:0]  rcrd_rp1_nxt;
    wire rcrd_rp1_valid_nxt;
    wire [1:0]  rcrd_rp2_nxt;
    wire rcrd_rp2_valid_nxt;
    wire [1:0]  rcrd_rp3_nxt;
    wire rcrd_rp3_valid_nxt;
    wire [1:0]  bcrd_rp0_nxt;
    wire bcrd_rp0_valid_nxt;
    wire [1:0]  bcrd_rp1_nxt;
    wire bcrd_rp1_valid_nxt;
    wire [1:0]  bcrd_rp2_nxt;
    wire bcrd_rp2_valid_nxt;
    wire [1:0]  bcrd_rp3_nxt;
    wire bcrd_rp3_valid_nxt;


    // --------------------------------------------------------------------------
    // 16. busy（端口由 assign 直接驱动，无内部再声明）
    // --------------------------------------------------------------------------

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // payload 提取（512：2B 开销上下交替；1024：首尾各 2B 固定无效）
    // --------------------------------------------------------------------------
    generate
        if (PAIF_DW == 512) begin : g_p512_sel
            assign payload = paif_phs_r ? paif_rxdata[0 +: PLD_WIDTH]
                                        : paif_rxdata[16 +: PLD_WIDTH];
        end else begin : g_p1024_sel
            assign payload = paif_rxdata[16 +: PLD_WIDTH];
        end
    endgenerate

    assign paif_hsk = paif_rxvalid & paif_rxready;

    // --------------------------------------------------------------------------
    // byp/pip 流水握手与前进条件（反压锁步：unmasked req，无环）
    //   任一"有内容且未就绪"通道即停步；全就绪才前进（各通道同拍吸收）
    // --------------------------------------------------------------------------
    assign pld_pip_hsk = pld_pip_vld & pld_pip_rdy;

    // [bak101] w/r 项保留原结构：w_buf_rdy/r_buf_rdy ≡ 1（结构性重言式，证明见下方 rdy 置常注释）
    //   ⇒ (1 | ~w_req)/(1 | ~r_req) ≡ 1，DC 常量折叠后反压实际仅 aw/ar0~3/b 六通道生效；
    //   保留 w/r 项以固化无回压耦合约束（若 rdy 不再恒 1，此处无需再改）。
    assign pld_pip_rdy = (aw_buf_rdy  | (~aw_req)) &
                         (ar0_buf_rdy | (~ar0_req)) &
                         (ar1_buf_rdy | (~ar1_req)) &
                         (ar2_buf_rdy | (~ar2_req)) &
                         (ar3_buf_rdy | (~ar3_req)) &
                         (w_buf_rdy   | (~w_req))   &
                         (r_buf_rdy   | (~r_req))   &
                         (b_buf_rdy   | (~b_req));

    // --------------------------------------------------------------------------
    // [bak79] 扫描预译码前推 byp 级：
    // [bak79 引入 / bak202 去寄存] 扫描预译码前推 byp 级：
    //   行选索引 cn_ptr_idx = cn_ptr_byp（pip 游标经后继函数组合派生）
    //     —— 物理含义：内容 P（现 byp）的游标。bak202 前为寄存的 cn_ptr_nxt_b_r，
    //        本例改为组合派生：设 S=cn_ptr_pip_r（pip 游标）、succ() 为后继函数，
    //        S(t+1)=succ(S(t))（pld_byp_hsk 拍）、且 succ(0)=0 ⇒ 去掉寄存后的
    //        cn_ptr_index 与旧寄存值逐拍位等价（仅把 succ 计算从寄存器搬回组合）。
    //     游标/flag 仅 pld_pip_hsk 拍前进（锁步）⇒ P 驻留 pip 期间 cn_ptr_idx 不变。
    //   数据源 pld_pip → pld_byp；输出（sel/masked/rsft_vc/vc_index/后继游标/后继 flag）
    //   在 byp→pip 转移拍（pld_byp_hsk）寄存器化（后缀 _pip_r），pip 侧组合消费。
    // --------------------------------------------------------------------------
    assign pld_byp_hsk  = pld_byp_vld & pld_byp_rdy;
    assign cn_ptr_idx   = cn_ptr_byp;
    assign pld_only_dat_byp = (cn_ptr_idx[5] || (&cn_ptr_idx[4:0]));

    // --------------------------------------------------------------------------
    // 游标对齐视图（byp 源，5 级桶形 0..31）：rsft_vc_byp[j] = 游标 + j*128bit 处 VC 候选视图
    // --------------------------------------------------------------------------
    assign rsft_pld_b_s0 = cn_ptr_idx[0] ? {{(S0_GRAN*GRAN_WIDTH){1'b0}}, pld_byp[PLD_WIDTH-1 : (S0_GRAN*GRAN_WIDTH)]} : pld_byp;
    assign rsft_pld_b_s1 = cn_ptr_idx[1] ? {{(S1_GRAN*GRAN_WIDTH){1'b0}}, rsft_pld_b_s0[PLD_WIDTH-1 : (S1_GRAN*GRAN_WIDTH)]} : rsft_pld_b_s0;
    assign rsft_pld_b_s2 = cn_ptr_idx[2] ? {{(S2_GRAN*GRAN_WIDTH){1'b0}}, rsft_pld_b_s1[PLD_WIDTH-1 : (S2_GRAN*GRAN_WIDTH)]} : rsft_pld_b_s1;
    assign rsft_pld_b_s3 = cn_ptr_idx[3] ? {{(S3_GRAN*GRAN_WIDTH){1'b0}}, rsft_pld_b_s2[PLD_WIDTH-1 : (S3_GRAN*GRAN_WIDTH)]} : rsft_pld_b_s2;
    assign rsft_pld_b_s4 = cn_ptr_idx[4] ? {{(S4_GRAN*GRAN_WIDTH){1'b0}}, rsft_pld_b_s3[PLD_WIDTH-1 : (S4_GRAN*GRAN_WIDTH)]} : rsft_pld_b_s3;
    assign rsft_vc_byp[0] = rsft_pld_b_s4;

    generate
        for(i=1 ; i<PLD_CN_NUM ; i=i+1) begin
            assign rsft_vc_byp[i] = rsft_vc_byp[0] >> (i*AX_VC_WIDTH);
        end
    endgenerate

    // --------------------------------------------------------------------------
    // 扫描预译码表（几何常量 + 每槽 7bit 头部）
    //   行 i 覆盖 GRAN 位置 {i, i+8, ...}；每行恰有一个"越界槽"（j_x），
    //   其 gth_ptr/last 必被赋值 → 全行驱动无 X（C1：越界槽后继 = 行末聚合 VC 跨拍）
    // --------------------------------------------------------------------------
    generate
        for(i=0 ; i<PLD_CNT ; i=i+1) begin
            for(j=0 ; j<PLD_CN_NUM ; j=j+1) begin
                if ((i*GRAN_WIDTH + j*AX_VC_WIDTH) < PLD_WIDTH) begin
                    assign cn_ptr_wnd_b[i][j*7 +: 7] = pld_byp[(i*GRAN_WIDTH + j*AX_VC_WIDTH) +: 7];
                    assign cn_ptr_wnd_B_ptr[i][j*6 +: 6]   = (i + j*AX_VC_CNT) + (WFULL_VC_CNT - {1'b0, PLD_CNT});
                    assign cn_ptr_wnd_A_ptr[i][j*6 +: 6]   = (i + j*AX_VC_CNT) + (WLITE_VC_CNT - {1'b0, PLD_CNT});
                    assign cn_ptr_wnd_dat_vdn[i][j*6 +: 6] = ({1'b0, PLD_CNT} - (i + j*AX_VC_CNT));

                    if ((i*GRAN_WIDTH + (j+1)*AX_VC_WIDTH) >= PLD_WIDTH) begin
                        assign cn_ptr_wnd_gth_ptr[i] = (i + j*AX_VC_CNT) + (AX_VC_CNT - {1'b0, PLD_CNT});
                        assign cn_ptr_wnd_last[i] = {1'b0, j};
                    end

                end else begin
                    assign cn_ptr_wnd_b[i][j*7 +: 7] = 7'h7f;
                    assign cn_ptr_wnd_B_ptr[i][j*6 +: 6]   = 6'd0;
                    assign cn_ptr_wnd_A_ptr[i][j*6 +: 6]   = 6'd0;
                    assign cn_ptr_wnd_dat_vdn[i][j*6 +: 6]  = 6'd0;
                end
            end
        end
    endgenerate


    // --------------------------------------------------------------------------
    // 行选（原式 31:1 直读窗口表）：cn_ptr_wnd_sel_byp[i] = cn_ptr_wnd_b[idx[4:0]][i*7 +: 7]
    //   （bak103 曾改 2a one-hot AND-OR 行选，本库实测级数中性——31 输入 OR 映射 ≈ 31:1 MUX
    //   深度；bak108 回退原式：结构更简、省 oh 向量 + 列布线面积）
    //   常量表（B/A_ptr、dat_vdn）同式（输入常量 ⇒ 退化为索引函数，非深段）；
    //   纯数据拍（idx 31..37）：idx=31 越界读（表仅 0..30 行）⇒ sel=X；
    //   idx=32..37 ⇒ idx[4:0] 回绕读行 0..5 ⇒ sel = pld_byp 活数据视图（跨拍续传片段，
    //   非 VC 头，CN 字段不可信）——两种情况 *_cn_byp 解码（含 X）均由 valid_byp 的
    //   (~pld_only_dat_byp) 广播项统一清零（0 ∧ X = 0，无寄存器/输出逃逸；
    //   [bak202] 原式含 pld_byp_vld 门控，因 valid_byp 仅被 _pip_r 在 pld_byp_hsk 拍采样
    //   （采样边 vld 恒 1）⇒ 门控冗余，已删；
    //   对应 bak104 的 exist_sel=pld_only?0:表），位语义恒等；
    //   该广播项不可删（idx=32..37 会误命中真实 CN/pad，idx=31 的 X 需 0 掩蔽）。
    // --------------------------------------------------------------------------


    // 选中行（游标行 = cn_ptr_idx）各槽字段（byp 组合）
    generate
        for(i=0 ; i<PLD_CN_NUM ; i=i+1) begin : g_wnd_sel_oh
            assign cn_ptr_wnd_sel_byp[i]         = cn_ptr_wnd_b[cn_ptr_idx[4:0]][i*7 +: 7];
            assign cn_ptr_wnd_B_ptr_sel_pip[i]   = cn_ptr_wnd_B_ptr[cn_ptr_pip_r[4:0]][i*6 +: 6];   // 常量表，不变
            assign cn_ptr_wnd_A_ptr_sel_pip[i]   = cn_ptr_wnd_A_ptr[cn_ptr_pip_r[4:0]][i*6 +: 6];   // 常量表，不变
            assign cn_ptr_wnd_dat_vdn_sel_pip[i] = cn_ptr_wnd_dat_vdn[cn_ptr_pip_r[4:0]][i*6 +: 6]; // 常量表，不变
        end
    endgenerate

    // 纯数据拍（游标 31..37）：整拍无 VC 头可译码，exist 全 0 屏蔽
    assign cn_ptr_wnd_gth_ptr_sel_byp = cn_ptr_wnd_gth_ptr[cn_ptr_idx[4:0]];
    assign cn_ptr_wnd_last_sel_byp    = cn_ptr_wnd_last[cn_ptr_idx[4:0]];


    // --------------------------------------------------------------------------
    // CN/RP 译码（选中行各槽；越界槽 = 7'h7f 不误命中；byp 有效门控）
    // --------------------------------------------------------------------------
    generate
        for(i=0 ; i<PLD_CN_NUM ; i=i+1) begin
            assign wfull_cn_byp[i] = (cn_ptr_wnd_sel_byp[i][0+:5] == `AXI4_W_FULL_CN);
            assign wlite_cn_byp[i] = (cn_ptr_wnd_sel_byp[i][0+:5] == `AXI4_W_LITE_CN);
            assign rfull_cn_byp[i] = (cn_ptr_wnd_sel_byp[i][0+:5] == `AXI4_R_FULL_CN);
            assign rlite_cn_byp[i] = (cn_ptr_wnd_sel_byp[i][0+:5] == `AXI4_R_LITE_CN);
            assign ar0_cn_byp[i]   = (cn_ptr_wnd_sel_byp[i][0+:5] == `AXI4_AR_CN) & (cn_ptr_wnd_sel_byp[i][5+:2] == 2'd0);
            assign ar1_cn_byp[i]   = (cn_ptr_wnd_sel_byp[i][0+:5] == `AXI4_AR_CN) & (cn_ptr_wnd_sel_byp[i][5+:2] == 2'd1);
            assign ar2_cn_byp[i]   = (cn_ptr_wnd_sel_byp[i][0+:5] == `AXI4_AR_CN) & (cn_ptr_wnd_sel_byp[i][5+:2] == 2'd2);
            assign ar3_cn_byp[i]   = (cn_ptr_wnd_sel_byp[i][0+:5] == `AXI4_AR_CN) & (cn_ptr_wnd_sel_byp[i][5+:2] == 2'd3);
            assign aw_cn_byp[i]    = (cn_ptr_wnd_sel_byp[i][0+:5] == `AXI4_AW_CN);
            assign b_cn_byp[i]     = (cn_ptr_wnd_sel_byp[i][0+:5] == `AXI4_B_CN);
            assign pad_cn_byp[i]   = (cn_ptr_wnd_sel_byp[i][0+:5] == 5'd0);
        end
    endgenerate

    // 数据 VC（wfull = B 底座 / lite|r = A 底座）与聚合 VC（aw/ar/b）分类（byp）
    assign A_cn_byp   = wlite_cn_byp | rfull_cn_byp | rlite_cn_byp;
    assign B_cn_byp   = wfull_cn_byp;
    assign dat_cn_byp = wfull_cn_byp | wlite_cn_byp | rfull_cn_byp | rlite_cn_byp;

    // --------------------------------------------------------------------------
    // 数据 VC 后槽位掩码（数据 VC 体内的 bit 可能形似 CN，屏蔽防误译码）
    // [bak101-2c] 串行累加（下三角常量表 + for 链）→ 并行前缀 OR：
    //   原表 tri[k][j] = (j>k) ⇒ dat_mask[j] = |dat_cn_byp[j-1:0]（j=0 恒 0），值恒等；
    //   深度：串行 ~3–4 级 → 并行 j 输入 OR 树 ~2 级。
    // --------------------------------------------------------------------------
    generate
        for (j = 0; j < PLD_CN_NUM; j = j + 1) begin : g_datmask
            if (j == 0) begin : g_dm0
                assign cn_ptr_wnd_dat_mask_byp[j] = 1'b0;
            end else begin : g_dmj
                assign cn_ptr_wnd_dat_mask_byp[j] = |dat_cn_byp[j-1:0];
            end
        end
    endgenerate

    assign cn_ptr_wnd_valid_byp = (~cn_ptr_wnd_dat_mask_byp) & {PLD_CN_NUM{~pld_only_dat_byp}};

    // --------------------------------------------------------------------------
    // 各类 VC 命中（行内，已掩码；byp）
    // --------------------------------------------------------------------------
    assign pad_int_masked_pip = (cn_ptr_wnd_valid_pip_r & pad_cn_pip_r);
    assign dat_int_masked_pip = (cn_ptr_wnd_valid_pip_r & dat_cn_pip_r);
    assign B_int_masked_pip   = (cn_ptr_wnd_valid_pip_r & B_cn_pip_r);
    assign A_int_masked_pip   = (cn_ptr_wnd_valid_pip_r & A_cn_pip_r);

    assign aw_int_masked_pip    = (cn_ptr_wnd_valid_pip_r & aw_cn_pip_r);
    assign ar0_int_masked_pip   = (cn_ptr_wnd_valid_pip_r & ar0_cn_pip_r);
    assign ar1_int_masked_pip   = (cn_ptr_wnd_valid_pip_r & ar1_cn_pip_r);
    assign ar2_int_masked_pip   = (cn_ptr_wnd_valid_pip_r & ar2_cn_pip_r);
    assign ar3_int_masked_pip   = (cn_ptr_wnd_valid_pip_r & ar3_cn_pip_r);
    assign b_int_masked_pip     = (cn_ptr_wnd_valid_pip_r & b_cn_pip_r);
    assign wfull_int_masked_pip = (cn_ptr_wnd_valid_pip_r & wfull_cn_pip_r);
    assign wlite_int_masked_pip = (cn_ptr_wnd_valid_pip_r & wlite_cn_pip_r);
    assign rfull_int_masked_pip = (cn_ptr_wnd_valid_pip_r & rfull_cn_pip_r);
    assign rlite_int_masked_pip = (cn_ptr_wnd_valid_pip_r & rlite_cn_pip_r);
    assign w_int_masked_pip     = wlite_int_masked_pip | wfull_int_masked_pip;
    assign r_int_masked_pip     = rlite_int_masked_pip | rfull_int_masked_pip;

    assign last_pad_pip = |pad_int_masked_pip;
    assign last_dat_pip = |dat_int_masked_pip;

    // --------------------------------------------------------------------------
    // 游标后继（四路互斥 OR：df 纯数据 / B wfull / A lite|r / gth 行末聚合跨拍）
    //   gth 仅在"无数据 VC 且尾部非 pad 且非纯数据拍"时生效
    // --------------------------------------------------------------------------
    always @(*) begin
        B_ptr_int_pip   = {PLD_CNT{1'b0}};
        A_ptr_int_pip   = {PLD_CNT{1'b0}};

        for(k=0 ; k<PLD_CN_NUM ; k=k+1) begin
            B_ptr_int_pip = (B_int_masked_pip[k] ? cn_ptr_wnd_B_ptr_sel_pip[k] : {PLD_CNT{1'b0}}) | B_ptr_int_pip;
            A_ptr_int_pip = (A_int_masked_pip[k] ? cn_ptr_wnd_A_ptr_sel_pip[k] : {PLD_CNT{1'b0}}) | A_ptr_int_pip;
        end
    end

    assign B_ptr_byp   = B_ptr_int_pip;
    assign A_ptr_byp   = A_ptr_int_pip;
    assign gth_ptr_byp = (last_pad_pip || last_dat_pip || pld_only_dat_pip_r) ? 6'd0 : cn_ptr_wnd_gth_ptr_sel_pip_r;
    assign df_ptr_byp  = cn_ptr_pip_r[5] ? {1'b0, (cn_ptr_pip_r[4:0] + 5'd1)} : 6'd0;  // pld_only_dat_pip_r ? (cn_ptr_pip_r - 6'd31) : 6'd0

    assign cn_ptr_byp = df_ptr_byp | B_ptr_byp | A_ptr_byp | gth_ptr_byp;


    // --------------------------------------------------------------------------
    // last_cn_flag（跨拍续传标记）
    //   aw/ar/b：行末槽为该通道 VC 且后继非 0（真跨拍）；
    //   w/r：行内任一数据 VC（数据 VC 恒跨拍，C2 单一类型）；
    //   纯数据拍：AND 保持（|cn_ptr_byp ≠ 0 表示数据 VC 尚有续传）
    //   [bak79 修正 / bak202 双寄存器合单] 保持项前驱 flag 用 last_cn_flag_pre_r
    //   （= 上一拍 pip 内容的 flag，即 Q 进入边上的 f(Q 的前驱)）：
    //   流式 = last_cn_flag_pip 上一拍采样值（前驱 X 的 flag），pip 空 = flag 保持
    //   （= f(最后吸收内容)）；若错用当拍组合值则为 f(X 的前驱)，差一代：X 内 w 尾 +
    //   r 头相邻（C2 合法）+ Q 纯 r 续拍时 f(Q)[w] 误置 1，R 驻留期 w_req 卡高串污。
    //   [bak202] 原双寄存器结构（A=last_cn_flag_nxt_b_r@byp_hsk 存 byp flag；
    //   B=last_cn_flag_r@pip_hsk = A 延迟一拍 = pip flag）合单为 last_cn_flag_pre_r：
    //   在 pld_pip_hsk 拍直接采样本组合值 last_cn_flag_pip，其值 ≡ 原 B（递推与
    //   复位均相同），故用于 *_pld_in / *_vdn_pip / *_req 三处均逐位等价。
    //   输入（*_int_masked_pip / *_cn_pip_r / *sel_pip_r / cn_ptr_pip_r）全为寄存器输出
    //   ⇒ 无组合环。
    // --------------------------------------------------------------------------
    assign gth_ptr_nz_pip = (~(last_pad_pip | last_dat_pip)) & (|cn_ptr_wnd_gth_ptr_sel_pip_r);
    assign last_cn_flag_pip = pld_only_dat_pip_r ? ({8{cn_ptr_pip_r[5]}} & last_cn_flag_pre_r) :
                              { b_int_masked_pip[cn_ptr_wnd_last_sel_pip_r] & gth_ptr_nz_pip,
                                |r_int_masked_pip,
                                |w_int_masked_pip,
                                ar3_int_masked_pip[cn_ptr_wnd_last_sel_pip_r] & gth_ptr_nz_pip,
                                ar2_int_masked_pip[cn_ptr_wnd_last_sel_pip_r] & gth_ptr_nz_pip,
                                ar1_int_masked_pip[cn_ptr_wnd_last_sel_pip_r] & gth_ptr_nz_pip,
                                ar0_int_masked_pip[cn_ptr_wnd_last_sel_pip_r] & gth_ptr_nz_pip,
                                aw_int_masked_pip[cn_ptr_wnd_last_sel_pip_r] & gth_ptr_nz_pip};

    // --------------------------------------------------------------------------
    // vc_index（行内首个本通道 VC 槽号，供缓冲对齐视图选取；byp 版）
    // [bak101-2b] 串行优先链（级联 MUX，~PLD_CN_NUM 级）→ 并行 first-hit：
    //   p[j] = |m[j-1:0]（低编号槽有命中）；g[j] = m[j] & ~p[j]（j 为首个命中槽 ⇒ g one-hot 或全 0）；
    //   idx[b] = |(g & MASK_b)（MASK_b = 槽号 b 位为 1 的常量掩码）。
    //   与原串行扫描值（最小命中槽号）恒等（含多命中取首、无命中为 0）；
    //   深度：前缀 OR(~2) + AND(1) + 归约(1) ≈ 3–4 级（512）/ 4–5 级（1024）。
    // --------------------------------------------------------------------------
    generate
        for (j = 0; j < PLD_CN_NUM; j = j + 1) begin : g_vcp
            if (j == 0) begin : g_p0
                assign dat_p[j] = 1'b0;
                assign aw_p[j]  = 1'b0;
                assign ar0_p[j] = 1'b0;
                assign ar1_p[j] = 1'b0;
                assign ar2_p[j] = 1'b0;
                assign ar3_p[j] = 1'b0;
                assign b_p[j]   = 1'b0;
            end else begin : g_pj
                assign dat_p[j] = |dat_int_masked_pip[j-1:0];
                assign aw_p[j]  = |aw_int_masked_pip[j-1:0];
                assign ar0_p[j] = |ar0_int_masked_pip[j-1:0];
                assign ar1_p[j] = |ar1_int_masked_pip[j-1:0];
                assign ar2_p[j] = |ar2_int_masked_pip[j-1:0];
                assign ar3_p[j] = |ar3_int_masked_pip[j-1:0];
                assign b_p[j]   = |b_int_masked_pip[j-1:0];
            end
        end
    endgenerate
    generate
        for (j = 0; j < PLD_CN_NUM; j = j + 1) begin : g_vcg
            assign dat_g[j] = dat_int_masked_pip[j] & ~dat_p[j];
            assign aw_g[j]  = aw_int_masked_pip[j]  & ~aw_p[j];
            assign ar0_g[j] = ar0_int_masked_pip[j] & ~ar0_p[j];
            assign ar1_g[j] = ar1_int_masked_pip[j] & ~ar1_p[j];
            assign ar2_g[j] = ar2_int_masked_pip[j] & ~ar2_p[j];
            assign ar3_g[j] = ar3_int_masked_pip[j] & ~ar3_p[j];
            assign b_g[j]   = b_int_masked_pip[j]   & ~b_p[j];
        end
    endgenerate
    generate
        if (PLD_CN_NUM == 4) begin : g_vci4
            // 4 槽 idx 2bit：bit0 = {j=1,3} = 4'b1010；bit1 = {j=2,3} = 4'b1100
            assign dat_vc_index_pip[0] = |(dat_g & 4'b1010);
            assign dat_vc_index_pip[1] = |(dat_g & 4'b1100);
            assign aw_vc_index_pip[0]  = |(aw_g & 4'b1010);
            assign aw_vc_index_pip[1]  = |(aw_g & 4'b1100);
            assign ar0_vc_index_pip[0] = |(ar0_g & 4'b1010);
            assign ar0_vc_index_pip[1] = |(ar0_g & 4'b1100);
            assign ar1_vc_index_pip[0] = |(ar1_g & 4'b1010);
            assign ar1_vc_index_pip[1] = |(ar1_g & 4'b1100);
            assign ar2_vc_index_pip[0] = |(ar2_g & 4'b1010);
            assign ar2_vc_index_pip[1] = |(ar2_g & 4'b1100);
            assign ar3_vc_index_pip[0] = |(ar3_g & 4'b1010);
            assign ar3_vc_index_pip[1] = |(ar3_g & 4'b1100);
            assign b_vc_index_pip[0]   = |(b_g & 4'b1010);
            assign b_vc_index_pip[1]   = |(b_g & 4'b1100);
        end else begin : g_vci8
            // 8 槽 idx 3bit：bit0 = 8'b10101010；bit1 = 8'b11001100；bit2 = 8'b11110000
            assign dat_vc_index_pip[0] = |(dat_g & 8'b10101010);
            assign dat_vc_index_pip[1] = |(dat_g & 8'b11001100);
            assign dat_vc_index_pip[2] = |(dat_g & 8'b11110000);
            assign aw_vc_index_pip[0]  = |(aw_g & 8'b10101010);
            assign aw_vc_index_pip[1]  = |(aw_g & 8'b11001100);
            assign aw_vc_index_pip[2]  = |(aw_g & 8'b11110000);
            assign ar0_vc_index_pip[0] = |(ar0_g & 8'b10101010);
            assign ar0_vc_index_pip[1] = |(ar0_g & 8'b11001100);
            assign ar0_vc_index_pip[2] = |(ar0_g & 8'b11110000);
            assign ar1_vc_index_pip[0] = |(ar1_g & 8'b10101010);
            assign ar1_vc_index_pip[1] = |(ar1_g & 8'b11001100);
            assign ar1_vc_index_pip[2] = |(ar1_g & 8'b11110000);
            assign ar2_vc_index_pip[0] = |(ar2_g & 8'b10101010);
            assign ar2_vc_index_pip[1] = |(ar2_g & 8'b11001100);
            assign ar2_vc_index_pip[2] = |(ar2_g & 8'b11110000);
            assign ar3_vc_index_pip[0] = |(ar3_g & 8'b10101010);
            assign ar3_vc_index_pip[1] = |(ar3_g & 8'b11001100);
            assign ar3_vc_index_pip[2] = |(ar3_g & 8'b11110000);
            assign b_vc_index_pip[0]   = |(b_g & 8'b10101010);
            assign b_vc_index_pip[1]   = |(b_g & 8'b11001100);
            assign b_vc_index_pip[2]   = |(b_g & 8'b11110000);
        end
    endgenerate

    // --------------------------------------------------------------------------
    // vdn pip 侧（组合；[bak202] 原 *_vdn_b_r 寄存器已删）：
    //   操作数（I1/I2 锁步不变式）：前驱 flag = last_cn_flag_pre_r（= f(Q)，
    //   pip 空时保持）；游标行 = cn_ptr_pip_r（= Q 后继 = P 进入游标，即 cn_ptr_byp）；
    //   masked = |*_int_masked_pip（P 的译码，寄存派生）；sel[vc_index] = *_vc_index_pip（P 的行几何）。
    //   全部输入为寄存器输出 ⇒ 纯组合无环；与 bak201 的 * _vdn_b_r 寄存值逐位相同。
    //   w/r 的和 cn_ptr + dat_vdn = 31−8j ≤ 31 无 6bit 溢出。
    // --------------------------------------------------------------------------
    assign dat_vdn_pip = cn_ptr_wnd_dat_vdn_sel_pip[dat_vc_index_pip];
    assign aw_vdn_pip  = last_cn_flag_pre_r[0] ? PLD_CNT : cn_ptr_wnd_dat_vdn_sel_pip[aw_vc_index_pip];
    assign ar0_vdn_pip = last_cn_flag_pre_r[1] ? PLD_CNT : cn_ptr_wnd_dat_vdn_sel_pip[ar0_vc_index_pip];
    assign ar1_vdn_pip = last_cn_flag_pre_r[2] ? PLD_CNT : cn_ptr_wnd_dat_vdn_sel_pip[ar1_vc_index_pip];
    assign ar2_vdn_pip = last_cn_flag_pre_r[3] ? PLD_CNT : cn_ptr_wnd_dat_vdn_sel_pip[ar2_vc_index_pip];
    assign ar3_vdn_pip = last_cn_flag_pre_r[4] ? PLD_CNT : cn_ptr_wnd_dat_vdn_sel_pip[ar3_vc_index_pip];
    assign b_vdn_pip   = last_cn_flag_pre_r[7] ? PLD_CNT : cn_ptr_wnd_dat_vdn_sel_pip[b_vc_index_pip];
    assign w_vdn_pip   = (|w_int_masked_pip) ? dat_vdn_pip : PLD_CNT;
    assign r_vdn_pip   = (|r_int_masked_pip) ? dat_vdn_pip : PLD_CNT;

    // --------------------------------------------------------------------------
    // 预译码寄存器：byp→pip 转移拍（pld_byp_hsk）更新；否则保持
    //   （保持拍 = 当前 pip 内容驻留且游标未前进 ⇒ 预译码仍有效）
    // --------------------------------------------------------------------------
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            for(k=0 ; k<PLD_CN_NUM ; k=k+1) begin
                rsft_vc_pip_r[k]          <= {PLD_WIDTH{1'b0}};
            end

            cn_ptr_pip_r               <= 6'd0;
            cn_ptr_wnd_valid_pip_r     <= {PLD_CN_NUM{1'b0}};
            pld_only_dat_pip_r         <= 1'b0;
            pad_cn_pip_r               <= {PLD_CN_NUM{1'b0}};
            dat_cn_pip_r               <= {PLD_CN_NUM{1'b0}};
            B_cn_pip_r                 <= {PLD_CN_NUM{1'b0}};
            A_cn_pip_r                 <= {PLD_CN_NUM{1'b0}};
            aw_cn_pip_r                <= {PLD_CN_NUM{1'b0}};
            ar0_cn_pip_r               <= {PLD_CN_NUM{1'b0}};
            ar1_cn_pip_r               <= {PLD_CN_NUM{1'b0}};
            ar2_cn_pip_r               <= {PLD_CN_NUM{1'b0}};
            ar3_cn_pip_r               <= {PLD_CN_NUM{1'b0}};
            b_cn_pip_r                 <= {PLD_CN_NUM{1'b0}};
            wfull_cn_pip_r             <= {PLD_CN_NUM{1'b0}};
            wlite_cn_pip_r             <= {PLD_CN_NUM{1'b0}};
            rfull_cn_pip_r             <= {PLD_CN_NUM{1'b0}};
            rlite_cn_pip_r             <= {PLD_CN_NUM{1'b0}};
            cn_ptr_wnd_gth_ptr_sel_pip_r <= 6'd0;
            cn_ptr_wnd_last_sel_pip_r  <= {($clog2(PLD_CN_NUM)){1'b0}};

        end else if (pld_byp_hsk) begin
            for(k=0 ; k<PLD_CN_NUM ; k=k+1) begin
                rsft_vc_pip_r[k]          <= rsft_vc_byp[k];
            end
            
            cn_ptr_pip_r               <= cn_ptr_byp;
            cn_ptr_wnd_valid_pip_r     <= cn_ptr_wnd_valid_byp;
            pld_only_dat_pip_r         <= pld_only_dat_byp;
            pad_cn_pip_r               <= pad_cn_byp;
            dat_cn_pip_r               <= dat_cn_byp;
            B_cn_pip_r                 <= B_cn_byp;
            A_cn_pip_r                 <= A_cn_byp;
            aw_cn_pip_r                <= aw_cn_byp;
            ar0_cn_pip_r               <= ar0_cn_byp;
            ar1_cn_pip_r               <= ar1_cn_byp;
            ar2_cn_pip_r               <= ar2_cn_byp;
            ar3_cn_pip_r               <= ar3_cn_byp;
            b_cn_pip_r                 <= b_cn_byp;
            wfull_cn_pip_r             <= wfull_cn_byp;
            wlite_cn_pip_r             <= wlite_cn_byp;
            rfull_cn_pip_r             <= rfull_cn_byp;
            rlite_cn_pip_r             <= rlite_cn_byp;
            cn_ptr_wnd_gth_ptr_sel_pip_r <= cn_ptr_wnd_gth_ptr_sel_byp;
            cn_ptr_wnd_last_sel_pip_r  <= cn_ptr_wnd_last_sel_byp;
        end
    end

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin 
            last_cn_flag_pre_r <= 8'd0;
        end else if (pld_pip_hsk) begin
            last_cn_flag_pre_r <= last_cn_flag_pip;
        end
    end

    // --------------------------------------------------------------------------
    // pip 侧同名 alias（[bak202] 下游消费者代码不变；rsft_vc 消费寄存值，其余消费 *_pip 组合值）
    // --------------------------------------------------------------------------
    generate
        for(i=0 ; i<PLD_CN_NUM ; i=i+1) begin
            assign rsft_vc[i] = rsft_vc_pip_r[i];
        end
    endgenerate


    assign dat_vc_index = dat_vc_index_pip;
    assign aw_vc_index  = aw_vc_index_pip;
    assign ar0_vc_index = ar0_vc_index_pip;
    assign ar1_vc_index = ar1_vc_index_pip;
    assign ar2_vc_index = ar2_vc_index_pip;
    assign ar3_vc_index = ar3_vc_index_pip;
    assign b_vc_index   = b_vc_index_pip;

    assign w_int_masked_any_pip   = |w_int_masked_pip;
    assign r_int_masked_any_pip   = |r_int_masked_pip;

    // --------------------------------------------------------------------------
    // vdn（本拍落入缓冲的本通道内容量，GRAN）
    //   w/r 两半模型：flag 拍缓冲只放数据 VC 内容（中间聚合 VC 被投影剔除）：
    //     flag ∧ 命中 → cn_ptr_idx + dat_vdn = 31 - 8j（j = 中间聚合 VC 数）
    //     flag ∧ 未命中（尾 pad / 纯数据）→ 整拍 31（尾 pad 多计无害：内容为 0）
    //   aw/ar/b（整拍 raw 或对齐视图，无投影）：flag = 整拍 31；否则 = VC 头距 payload 尾
    // --------------------------------------------------------------------------
    // --------------------------------------------------------------------------
    // vdn alias（[bak202] 消费 *_vdn_pip 组合值；cnt_nxt 消费者代码不变；pip dat_vdn 已删）
    // --------------------------------------------------------------------------
    assign aw_vdn  = aw_vdn_pip;
    assign ar0_vdn = ar0_vdn_pip;
    assign ar1_vdn = ar1_vdn_pip;
    assign ar2_vdn = ar2_vdn_pip;
    assign ar3_vdn = ar3_vdn_pip;
    assign w_vdn   = w_vdn_pip;
    assign r_vdn   = r_vdn_pip;
    assign b_vdn   = b_vdn_pip;

    // --------------------------------------------------------------------------
    // req（本通道有内容/续传，无环根）与 rdy（排空后残留 < 阈值，可吸收）
    // --------------------------------------------------------------------------
    // --------------------------------------------------------------------------
    // [bak79 引入 / bak202 去寄存] req（反压锁步根，无组合环）：
    //   P 驻留 pip 期间的正确 req = f(P 的前驱)[i] | |P 的 masked[i]（flag 语义：
    //   X 驻留期间 flag = f(X 的前驱)；K 态经纯数据拍保持项传递）。
    //   [bak202] 原 aw_req_r~b_req_r 八个寄存器已删，改为组合：
    //     前驱 flag = last_cn_flag_pre_r（pip_hsk 采样，已含前驱 flag 语义）
    //     pip 译码 = |*_int_masked_pip（预译码寄存器派生，即 P 驻留期译码）
    //   两者均为寄存器输出 ⇒ 组合无环，反压入口（pld_pip_rdy → pld_byp_rdy →
    //   PAIF RX ready）仍不过额外 OR 级；与 bak201 的寄存值逐拍位等价。
    // --------------------------------------------------------------------------
    assign aw_req  = last_cn_flag_pre_r[0] | (|aw_int_masked_pip);
    assign ar0_req = last_cn_flag_pre_r[1] | (|ar0_int_masked_pip);
    assign ar1_req = last_cn_flag_pre_r[2] | (|ar1_int_masked_pip);
    assign ar2_req = last_cn_flag_pre_r[3] | (|ar2_int_masked_pip);
    assign ar3_req = last_cn_flag_pre_r[4] | (|ar3_int_masked_pip);
    assign w_req   = last_cn_flag_pre_r[5] | (|w_int_masked_pip);
    assign r_req   = last_cn_flag_pre_r[6] | (|r_int_masked_pip);
    assign b_req   = last_cn_flag_pre_r[7] | (|b_int_masked_pip);

    assign aw_buf_rdy  = (aw_buf_cnt_r < AX_VC_CNT);
    assign ar0_buf_rdy = (ar0_buf_cnt_r < AX_VC_CNT);
    assign ar1_buf_rdy = (ar1_buf_cnt_r < AX_VC_CNT);
    assign ar2_buf_rdy = (ar2_buf_cnt_r < AX_VC_CNT);
    assign ar3_buf_rdy = (ar3_buf_cnt_r < AX_VC_CNT);
    // [bak101] w/r_buf_rdy 结构性恒 1（重言式，置常 1 使 DC 裁掉 rdy 比较锥）：
    //   rdy = (cnt_df < VC_CNT) 三分支皆真：
    //     W0（无次数据 VC）：cnt_df = 0 < VC_CNT ✓
    //     K 态（partial）：cnt_df = cnt_r < 底部宽度 ≤ WFULL_VC_CNT ✓（严格小于）
    //     D 态（有次 VC）：cnt_df = cnt_r − 底部宽度 ≤ CAP − 底部宽度，其中
    //       w: CAP_W = WFULL_VC + PLD_CNT − 1（缓冲定宽 = 最坏拼合）⇒
    //           512: 68−34 = 34 < 38；1024: 67−33 = 34 < 37 ✓
    //       r: CAP_R = R_VC + PLD_CNT − 1 ⇒ 512: 64−34 = 30 < 34；1024: 63−33 = 30 < 33 ✓
    //   （cnt_r ≤ CAP 归纳：K 态补满最坏 = (底部宽度−1) + PLD_CNT = CAP 恰合，
    //    D 态弹+吸后 ≤ CAP−底部宽度+PLD_CNT < CAP）
    //   前提不变式（破坏则常量坍塌，需重新评估）：
    //     ① w/r 输出无回压、每拍无条件弹出（tx_proc = ~partial，C4 修订 owner 决策）；
    //     ② 每拍注入 ≤ PLD_CNT（vdn 公式）；③ 缓冲定宽 = VC + PLD − GRAN；④ C2 数据 VC 单一类型。
    //   aw/ar/b 不适用（D 态残留上界 CAP−AX_VC_CNT = 30 ≥ AX_VC_CNT = 8，可真反压）。
    //   [bak201 修订] 原 w_buf_cnt_df 监测建议已随该信号删除；如需可加 SVA assert (w_buf_cnt < 阈值)。
    assign w_buf_rdy   = 1'b1;
    assign r_buf_rdy   = 1'b1;
    assign b_buf_rdy   = (b_buf_cnt_r < AX_VC_CNT);

    // ==========================================================================
    // 通道弹性缓冲（组合部分）
    //   通用结构：底部 = 最老本通道 VC；
    //     K 态（底部 partial，跨拍拼合）：保留 + shift-in 残段量；
    //     D 态（次同通道 VC 在缓冲内）：右移至其头部；
    //     W0（无次同通道）：清零（底部完整 VC 已输出）。
    //   吸收 = req & pld_pip_hsk（锁步）；shift-in 量 = cnt_df（rdy 阈值
    //   将其钳制在 barrel 档位内，N7 不变量）。
    // ==========================================================================

    // --------------------------------------------------------------------------
    // aw 缓冲
    // --------------------------------------------------------------------------
    assign aw_pld_in = last_cn_flag_pre_r[0] ? {{(AX_BUF_WIDTH-PLD_WIDTH){1'b0}}, pld_pip} :
                                           {{(AX_BUF_WIDTH-PLD_WIDTH){1'b0}}, rsft_vc[aw_vc_index]};

    assign lsft_aw_s0 = aw_buf_cnt_r[0] ? {aw_pld_in[AX_BUF_WIDTH-1-(S0_GRAN*GRAN_WIDTH) : 0], {(S0_GRAN*GRAN_WIDTH){1'b0}}} : aw_pld_in;
    assign lsft_aw_s1 = aw_buf_cnt_r[1] ? {lsft_aw_s0[AX_BUF_WIDTH-1-(S1_GRAN*GRAN_WIDTH) : 0], {(S1_GRAN*GRAN_WIDTH){1'b0}}} : lsft_aw_s0;
    assign lsft_aw_s2 = aw_buf_cnt_r[2] ? {lsft_aw_s1[AX_BUF_WIDTH-1-(S2_GRAN*GRAN_WIDTH) : 0], {(S2_GRAN*GRAN_WIDTH){1'b0}}} : lsft_aw_s1;

    generate
        if(PAIF_DW == 1024) begin : g_lsft_aw_1024
            assign lsft_aw = lsft_aw_s1;
        end else begin : g_lsft_aw_512
            assign lsft_aw = lsft_aw_s2;
        end
    endgenerate

    assign aw_buf = aw_buf_hsk ? (aw_buf_r | lsft_aw) : aw_buf_r;

    assign aw_buf_cnt = aw_buf_hsk ? (aw_buf_cnt_r + aw_vdn) : aw_buf_cnt_r;


    assign aw_tx_proc = (aw_buf_cnt >= AX_VC_CNT) || ((|aw_buf_cnt_r) && (aw_buf_r[0+:5] != `AXI4_AW_CN));
    // 吸收 = 本通道有内容 且 流水前进（aw_buf_rdy 已隐含于 pld_pip_rdy）
    assign aw_buf_hsk = aw_req & pld_pip_hsk;

    assign aw_buf_cnt_nxt = (~aw_tx_proc) ? aw_buf_cnt :
                           (aw_cn_get[8:1] == 8'd0) ? {AX_BUF_CNT_W{1'b0}} :
                           aw_cn_get[1] ? (aw_buf_cnt - AX_VC_CNT) : (aw_buf_cnt - (2*AX_VC_CNT));


    assign aw_buf_nxt = (~aw_tx_proc) ? aw_buf :
                     (aw_cn_get[8:1] == 8'd0) ? {AX_BUF_WIDTH{1'b0}} :
                     aw_cn_get[1] ? (aw_buf >> AX_VC_WIDTH) : (aw_buf >> (2*AX_VC_WIDTH));


    assign aw_cn_get[0] = (aw_buf[0*AX_VC_WIDTH +: 5] == `AXI4_AW_CN);
    assign aw_cn_get[1] = (aw_buf[1*AX_VC_WIDTH +: 5] == `AXI4_AW_CN);
    assign aw_cn_get[2] = (aw_buf[2*AX_VC_WIDTH +: 5] == `AXI4_AW_CN);
    assign aw_cn_get[3] = (aw_buf[3*AX_VC_WIDTH +: 5] == `AXI4_AW_CN);
    assign aw_cn_get[4] = (aw_buf[4*AX_VC_WIDTH +: 5] == `AXI4_AW_CN);

    generate
        if(PAIF_DW == 1024) begin : g_aw_cnget_hi
            assign aw_cn_get[5] = (aw_buf[5*AX_VC_WIDTH +: 5] == `AXI4_AW_CN);
            assign aw_cn_get[6] = (aw_buf[6*AX_VC_WIDTH +: 5] == `AXI4_AW_CN);
            assign aw_cn_get[7] = (aw_buf[7*AX_VC_WIDTH +: 5] == `AXI4_AW_CN);
            assign aw_cn_get[8] = (aw_buf[8*AX_VC_WIDTH +: 5] == `AXI4_AW_CN);
        end else begin : g_aw_cnget_lo
            assign aw_cn_get[5] = 1'b0;
            assign aw_cn_get[6] = 1'b0;
            assign aw_cn_get[7] = 1'b0;
            assign aw_cn_get[8] = 1'b0;
        end
    endgenerate


    assign aw_bus   = aw_buf[7 +: AX_WIDTH];
    assign aw_valid = (aw_buf_cnt >= AX_VC_CNT) & aw_cn_get[0];

    // --------------------------------------------------------------------------
    // ar0 缓冲（aw 同构，CN+RP 匹配）
    // --------------------------------------------------------------------------
    assign ar0_pld_in = last_cn_flag_pre_r[1] ? {{(AX_BUF_WIDTH-PLD_WIDTH){1'b0}}, pld_pip} :
                                            {{(AX_BUF_WIDTH-PLD_WIDTH){1'b0}}, rsft_vc[ar0_vc_index]};

    assign lsft_ar0_s0 = ar0_buf_cnt_r[0] ? {ar0_pld_in[AX_BUF_WIDTH-1-(S0_GRAN*GRAN_WIDTH) : 0], {(S0_GRAN*GRAN_WIDTH){1'b0}}} : ar0_pld_in;
    assign lsft_ar0_s1 = ar0_buf_cnt_r[1] ? {lsft_ar0_s0[AX_BUF_WIDTH-1-(S1_GRAN*GRAN_WIDTH) : 0], {(S1_GRAN*GRAN_WIDTH){1'b0}}} : lsft_ar0_s0;
    assign lsft_ar0_s2 = ar0_buf_cnt_r[2] ? {lsft_ar0_s1[AX_BUF_WIDTH-1-(S2_GRAN*GRAN_WIDTH) : 0], {(S2_GRAN*GRAN_WIDTH){1'b0}}} : lsft_ar0_s1;

    generate
        if(PAIF_DW == 1024) begin : g_lsft_ar0_1024
            assign lsft_ar0 = lsft_ar0_s1;
        end else begin : g_lsft_ar0_512
            assign lsft_ar0 = lsft_ar0_s2;
        end
    endgenerate

    assign ar0_buf = ar0_buf_hsk ? (ar0_buf_r | lsft_ar0) : ar0_buf_r;
    assign ar0_buf_cnt = ar0_buf_hsk ? (ar0_buf_cnt_r + ar0_vdn) : ar0_buf_cnt_r;

    assign ar0_tx_proc = (ar0_buf_cnt >= AX_VC_CNT) || ((|ar0_buf_cnt_r) && (ar0_buf_r[0+:7] != {2'b00, `AXI4_AR_CN}));
    assign ar0_buf_hsk = ar0_req & pld_pip_hsk;

    assign ar0_buf_cnt_nxt = (~ar0_tx_proc) ? ar0_buf_cnt :
                          (ar0_cn_get[8:1] == 8'd0) ? {AX_BUF_CNT_W{1'b0}} :
                          ar0_cn_get[1] ? (ar0_buf_cnt - AX_VC_CNT) : (ar0_buf_cnt - (2*AX_VC_CNT));

    assign ar0_buf_nxt = (~ar0_tx_proc) ? ar0_buf :
                     (ar0_cn_get[8:1] == 8'd0) ? {AX_BUF_WIDTH{1'b0}} :
                     ar0_cn_get[1] ? (ar0_buf >> AX_VC_WIDTH) : (ar0_buf >> (2*AX_VC_WIDTH));

    assign ar0_cn_get[0] = (ar0_buf[0*AX_VC_WIDTH +: 7] == {2'b00, `AXI4_AR_CN});
    assign ar0_cn_get[1] = (ar0_buf[1*AX_VC_WIDTH +: 7] == {2'b00, `AXI4_AR_CN});
    assign ar0_cn_get[2] = (ar0_buf[2*AX_VC_WIDTH +: 7] == {2'b00, `AXI4_AR_CN});
    assign ar0_cn_get[3] = (ar0_buf[3*AX_VC_WIDTH +: 7] == {2'b00, `AXI4_AR_CN});
    assign ar0_cn_get[4] = (ar0_buf[4*AX_VC_WIDTH +: 7] == {2'b00, `AXI4_AR_CN});

    generate
        if(PAIF_DW == 1024) begin : g_ar0_cnget_hi
            assign ar0_cn_get[5] = (ar0_buf[5*AX_VC_WIDTH +: 7] == {2'b00, `AXI4_AR_CN});
            assign ar0_cn_get[6] = (ar0_buf[6*AX_VC_WIDTH +: 7] == {2'b00, `AXI4_AR_CN});
            assign ar0_cn_get[7] = (ar0_buf[7*AX_VC_WIDTH +: 7] == {2'b00, `AXI4_AR_CN});
            assign ar0_cn_get[8] = (ar0_buf[8*AX_VC_WIDTH +: 7] == {2'b00, `AXI4_AR_CN});
        end else begin : g_ar0_cnget_lo
            assign ar0_cn_get[5] = 1'b0;
            assign ar0_cn_get[6] = 1'b0;
            assign ar0_cn_get[7] = 1'b0;
            assign ar0_cn_get[8] = 1'b0;
        end
    endgenerate

    assign ar_rp0_valid = (ar0_buf_cnt >= AX_VC_CNT) & ar0_cn_get[0];
    assign ar_rp0_bus   = ar_rp0_valid ? ar0_buf[7 +: AX_WIDTH] : {AX_WIDTH{1'b0}};

    // --------------------------------------------------------------------------
    // ar1 缓冲
    // --------------------------------------------------------------------------
    assign ar1_pld_in = last_cn_flag_pre_r[2] ? {{(AX_BUF_WIDTH-PLD_WIDTH){1'b0}}, pld_pip} :
                                            {{(AX_BUF_WIDTH-PLD_WIDTH){1'b0}}, rsft_vc[ar1_vc_index]};

    assign lsft_ar1_s0 = ar1_buf_cnt_r[0] ? {ar1_pld_in[AX_BUF_WIDTH-1-(S0_GRAN*GRAN_WIDTH) : 0], {(S0_GRAN*GRAN_WIDTH){1'b0}}} : ar1_pld_in;
    assign lsft_ar1_s1 = ar1_buf_cnt_r[1] ? {lsft_ar1_s0[AX_BUF_WIDTH-1-(S1_GRAN*GRAN_WIDTH) : 0], {(S1_GRAN*GRAN_WIDTH){1'b0}}} : lsft_ar1_s0;
    assign lsft_ar1_s2 = ar1_buf_cnt_r[2] ? {lsft_ar1_s1[AX_BUF_WIDTH-1-(S2_GRAN*GRAN_WIDTH) : 0], {(S2_GRAN*GRAN_WIDTH){1'b0}}} : lsft_ar1_s1;

    generate
        if(PAIF_DW == 1024) begin : g_lsft_ar1_1024
            assign lsft_ar1 = lsft_ar1_s1;
        end else begin : g_lsft_ar1_512
            assign lsft_ar1 = lsft_ar1_s2;
        end
    endgenerate

    assign ar1_buf = ar1_buf_hsk ? (ar1_buf_r | lsft_ar1) : ar1_buf_r;
    assign ar1_buf_cnt = ar1_buf_hsk ? (ar1_buf_cnt_r + ar1_vdn) : ar1_buf_cnt_r;

    assign ar1_tx_proc = (ar1_buf_cnt >= AX_VC_CNT) || ((|ar1_buf_cnt_r) && (ar1_buf_r[0+:7] != {2'b01, `AXI4_AR_CN}));
    assign ar1_buf_hsk = ar1_req & pld_pip_hsk;

    assign ar1_buf_cnt_nxt = (~ar1_tx_proc) ? ar1_buf_cnt :
                          (ar1_cn_get[8:1] == 8'd0) ? {AX_BUF_CNT_W{1'b0}} :
                          ar1_cn_get[1] ? (ar1_buf_cnt - AX_VC_CNT) : (ar1_buf_cnt - (2*AX_VC_CNT));

    assign ar1_buf_nxt = (~ar1_tx_proc) ? ar1_buf :
                     (ar1_cn_get[8:1] == 8'd0) ? {AX_BUF_WIDTH{1'b0}} :
                     ar1_cn_get[1] ? (ar1_buf >> AX_VC_WIDTH) : (ar1_buf >> (2*AX_VC_WIDTH));

    assign ar1_cn_get[0] = (ar1_buf[0*AX_VC_WIDTH +: 7] == {2'b01, `AXI4_AR_CN});
    assign ar1_cn_get[1] = (ar1_buf[1*AX_VC_WIDTH +: 7] == {2'b01, `AXI4_AR_CN});
    assign ar1_cn_get[2] = (ar1_buf[2*AX_VC_WIDTH +: 7] == {2'b01, `AXI4_AR_CN});
    assign ar1_cn_get[3] = (ar1_buf[3*AX_VC_WIDTH +: 7] == {2'b01, `AXI4_AR_CN});
    assign ar1_cn_get[4] = (ar1_buf[4*AX_VC_WIDTH +: 7] == {2'b01, `AXI4_AR_CN});

    generate
        if(PAIF_DW == 1024) begin : g_ar1_cnget_hi
            assign ar1_cn_get[5] = (ar1_buf[5*AX_VC_WIDTH +: 7] == {2'b01, `AXI4_AR_CN});
            assign ar1_cn_get[6] = (ar1_buf[6*AX_VC_WIDTH +: 7] == {2'b01, `AXI4_AR_CN});
            assign ar1_cn_get[7] = (ar1_buf[7*AX_VC_WIDTH +: 7] == {2'b01, `AXI4_AR_CN});
            assign ar1_cn_get[8] = (ar1_buf[8*AX_VC_WIDTH +: 7] == {2'b01, `AXI4_AR_CN});
        end else begin : g_ar1_cnget_lo
            assign ar1_cn_get[5] = 1'b0;
            assign ar1_cn_get[6] = 1'b0;
            assign ar1_cn_get[7] = 1'b0;
            assign ar1_cn_get[8] = 1'b0;
        end
    endgenerate

    assign ar_rp1_valid = (ar1_buf_cnt >= AX_VC_CNT) & ar1_cn_get[0];
    assign ar_rp1_bus   = ar_rp1_valid ? ar1_buf[7 +: AX_WIDTH] : {AX_WIDTH{1'b0}};

    // --------------------------------------------------------------------------
    // ar2 缓冲
    // --------------------------------------------------------------------------
    assign ar2_pld_in = last_cn_flag_pre_r[3] ? {{(AX_BUF_WIDTH-PLD_WIDTH){1'b0}}, pld_pip} :
                                            {{(AX_BUF_WIDTH-PLD_WIDTH){1'b0}}, rsft_vc[ar2_vc_index]};

    assign lsft_ar2_s0 = ar2_buf_cnt_r[0] ? {ar2_pld_in[AX_BUF_WIDTH-1-(S0_GRAN*GRAN_WIDTH) : 0], {(S0_GRAN*GRAN_WIDTH){1'b0}}} : ar2_pld_in;
    assign lsft_ar2_s1 = ar2_buf_cnt_r[1] ? {lsft_ar2_s0[AX_BUF_WIDTH-1-(S1_GRAN*GRAN_WIDTH) : 0], {(S1_GRAN*GRAN_WIDTH){1'b0}}} : lsft_ar2_s0;
    assign lsft_ar2_s2 = ar2_buf_cnt_r[2] ? {lsft_ar2_s1[AX_BUF_WIDTH-1-(S2_GRAN*GRAN_WIDTH) : 0], {(S2_GRAN*GRAN_WIDTH){1'b0}}} : lsft_ar2_s1;

    generate
        if(PAIF_DW == 1024) begin : g_lsft_ar2_1024
            assign lsft_ar2 = lsft_ar2_s1;
        end else begin : g_lsft_ar2_512
            assign lsft_ar2 = lsft_ar2_s2;
        end
    endgenerate

    assign ar2_buf = ar2_buf_hsk ? (ar2_buf_r | lsft_ar2) : ar2_buf_r;
    assign ar2_buf_cnt = ar2_buf_hsk ? (ar2_buf_cnt_r + ar2_vdn) : ar2_buf_cnt_r;

    assign ar2_tx_proc = (ar2_buf_cnt >= AX_VC_CNT) || ((|ar2_buf_cnt_r) && (ar2_buf_r[0+:7] != {2'b10, `AXI4_AR_CN}));
    assign ar2_buf_hsk = ar2_req & pld_pip_hsk;

    assign ar2_buf_cnt_nxt = (~ar2_tx_proc) ? ar2_buf_cnt :
                          (ar2_cn_get[8:1] == 8'd0) ? {AX_BUF_CNT_W{1'b0}} :
                          ar2_cn_get[1] ? (ar2_buf_cnt - AX_VC_CNT) : (ar2_buf_cnt - (2*AX_VC_CNT));

    assign ar2_buf_nxt = (~ar2_tx_proc) ? ar2_buf :
                     (ar2_cn_get[8:1] == 8'd0) ? {AX_BUF_WIDTH{1'b0}} :
                     ar2_cn_get[1] ? (ar2_buf >> AX_VC_WIDTH) : (ar2_buf >> (2*AX_VC_WIDTH));

    assign ar2_cn_get[0] = (ar2_buf[0*AX_VC_WIDTH +: 7] == {2'b10, `AXI4_AR_CN});
    assign ar2_cn_get[1] = (ar2_buf[1*AX_VC_WIDTH +: 7] == {2'b10, `AXI4_AR_CN});
    assign ar2_cn_get[2] = (ar2_buf[2*AX_VC_WIDTH +: 7] == {2'b10, `AXI4_AR_CN});
    assign ar2_cn_get[3] = (ar2_buf[3*AX_VC_WIDTH +: 7] == {2'b10, `AXI4_AR_CN});
    assign ar2_cn_get[4] = (ar2_buf[4*AX_VC_WIDTH +: 7] == {2'b10, `AXI4_AR_CN});

    generate
        if(PAIF_DW == 1024) begin : g_ar2_cnget_hi
            assign ar2_cn_get[5] = (ar2_buf[5*AX_VC_WIDTH +: 7] == {2'b10, `AXI4_AR_CN});
            assign ar2_cn_get[6] = (ar2_buf[6*AX_VC_WIDTH +: 7] == {2'b10, `AXI4_AR_CN});
            assign ar2_cn_get[7] = (ar2_buf[7*AX_VC_WIDTH +: 7] == {2'b10, `AXI4_AR_CN});
            assign ar2_cn_get[8] = (ar2_buf[8*AX_VC_WIDTH +: 7] == {2'b10, `AXI4_AR_CN});
        end else begin : g_ar2_cnget_lo
            assign ar2_cn_get[5] = 1'b0;
            assign ar2_cn_get[6] = 1'b0;
            assign ar2_cn_get[7] = 1'b0;
            assign ar2_cn_get[8] = 1'b0;
        end
    endgenerate

    assign ar_rp2_valid = (ar2_buf_cnt >= AX_VC_CNT) & ar2_cn_get[0];
    assign ar_rp2_bus   = ar_rp2_valid ? ar2_buf[7 +: AX_WIDTH] : {AX_WIDTH{1'b0}};

    // --------------------------------------------------------------------------
    // ar3 缓冲
    // --------------------------------------------------------------------------
    assign ar3_pld_in = last_cn_flag_pre_r[4] ? {{(AX_BUF_WIDTH-PLD_WIDTH){1'b0}}, pld_pip} :
                                            {{(AX_BUF_WIDTH-PLD_WIDTH){1'b0}}, rsft_vc[ar3_vc_index]};

    assign lsft_ar3_s0 = ar3_buf_cnt_r[0] ? {ar3_pld_in[AX_BUF_WIDTH-1-(S0_GRAN*GRAN_WIDTH) : 0], {(S0_GRAN*GRAN_WIDTH){1'b0}}} : ar3_pld_in;
    assign lsft_ar3_s1 = ar3_buf_cnt_r[1] ? {lsft_ar3_s0[AX_BUF_WIDTH-1-(S1_GRAN*GRAN_WIDTH) : 0], {(S1_GRAN*GRAN_WIDTH){1'b0}}} : lsft_ar3_s0;
    assign lsft_ar3_s2 = ar3_buf_cnt_r[2] ? {lsft_ar3_s1[AX_BUF_WIDTH-1-(S2_GRAN*GRAN_WIDTH) : 0], {(S2_GRAN*GRAN_WIDTH){1'b0}}} : lsft_ar3_s1;

    generate
        if(PAIF_DW == 1024) begin : g_lsft_ar3_1024
            assign lsft_ar3 = lsft_ar3_s1;
        end else begin : g_lsft_ar3_512
            assign lsft_ar3 = lsft_ar3_s2;
        end
    endgenerate

    assign ar3_buf = ar3_buf_hsk ? (ar3_buf_r | lsft_ar3) : ar3_buf_r;
    assign ar3_buf_cnt = ar3_buf_hsk ? (ar3_buf_cnt_r + ar3_vdn) : ar3_buf_cnt_r;

    assign ar3_tx_proc = (ar3_buf_cnt >= AX_VC_CNT) || ((|ar3_buf_cnt_r) && (ar3_buf_r[0+:7] != {2'b11, `AXI4_AR_CN}));
    assign ar3_buf_hsk = ar3_req & pld_pip_hsk;

    assign ar3_buf_cnt_nxt = (~ar3_tx_proc) ? ar3_buf_cnt :
                          (ar3_cn_get[8:1] == 8'd0) ? {AX_BUF_CNT_W{1'b0}} :
                          ar3_cn_get[1] ? (ar3_buf_cnt - AX_VC_CNT) : (ar3_buf_cnt - (2*AX_VC_CNT));

    assign ar3_buf_nxt = (~ar3_tx_proc) ? ar3_buf :
                     (ar3_cn_get[8:1] == 8'd0) ? {AX_BUF_WIDTH{1'b0}} :
                     ar3_cn_get[1] ? (ar3_buf >> AX_VC_WIDTH) : (ar3_buf >> (2*AX_VC_WIDTH));

    assign ar3_cn_get[0] = (ar3_buf[0*AX_VC_WIDTH +: 7] == {2'b11, `AXI4_AR_CN});
    assign ar3_cn_get[1] = (ar3_buf[1*AX_VC_WIDTH +: 7] == {2'b11, `AXI4_AR_CN});
    assign ar3_cn_get[2] = (ar3_buf[2*AX_VC_WIDTH +: 7] == {2'b11, `AXI4_AR_CN});
    assign ar3_cn_get[3] = (ar3_buf[3*AX_VC_WIDTH +: 7] == {2'b11, `AXI4_AR_CN});
    assign ar3_cn_get[4] = (ar3_buf[4*AX_VC_WIDTH +: 7] == {2'b11, `AXI4_AR_CN});

    generate
        if(PAIF_DW == 1024) begin : g_ar3_cnget_hi
            assign ar3_cn_get[5] = (ar3_buf[5*AX_VC_WIDTH +: 7] == {2'b11, `AXI4_AR_CN});
            assign ar3_cn_get[6] = (ar3_buf[6*AX_VC_WIDTH +: 7] == {2'b11, `AXI4_AR_CN});
            assign ar3_cn_get[7] = (ar3_buf[7*AX_VC_WIDTH +: 7] == {2'b11, `AXI4_AR_CN});
            assign ar3_cn_get[8] = (ar3_buf[8*AX_VC_WIDTH +: 7] == {2'b11, `AXI4_AR_CN});
        end else begin : g_ar3_cnget_lo
            assign ar3_cn_get[5] = 1'b0;
            assign ar3_cn_get[6] = 1'b0;
            assign ar3_cn_get[7] = 1'b0;
            assign ar3_cn_get[8] = 1'b0;
        end
    endgenerate

    assign ar_rp3_valid = (ar3_buf_cnt >= AX_VC_CNT) & ar3_cn_get[0];
    assign ar_rp3_bus   = ar_rp3_valid ? ar3_buf[7 +: AX_WIDTH] : {AX_WIDTH{1'b0}};

    // --------------------------------------------------------------------------
    // w 缓冲（两半模型，r 同构；桶形 6 级 0..63 GRAN）
    //   (ii) 不变式：w 数据 VC 后继只可能是数据 VC 或尾 pad → 次 w VC 恒紧邻
    //   （偏移 = 底部 w VC 整宽），原 g=1..8 量化搜索已移除
    // --------------------------------------------------------------------------
    // [bak103] w 组合探针（wfull_bot/wlite_bot/w_data_bot1/w_bot1_at_*）已删：
    //   消费端全部重连 next-state 寄存器（wfull_bot_r 等；_nxt 解码定义在 w_buf_nxt 之后）。

    assign w_tx_proc   = ((w_buf_cnt >= WFULL_VC_CNT)&&wfull_bot_r) || ((w_buf_cnt >= WLITE_VC_CNT)&&wlite_bot_r);
    assign w_buf_hsk   = w_req & pld_pip_hsk;

    assign wfull_bot_nxt = (w_buf_nxt[0 +: 5] == `AXI4_W_FULL_CN);
    assign wlite_bot_nxt = (w_buf_nxt[0 +: 5] == `AXI4_W_LITE_CN);

    // 桶形：输入 raw pld_pip 零扩展（flag 拍整拍吸收）
    assign w_pld_in = {{(W_BUF_WIDTH-PLD_WIDTH){1'b0}}, pld_pip};
    assign lsft_w_s0 = w_buf_cnt_r[0] ? {w_pld_in[W_BUF_WIDTH-1-(S0_GRAN*GRAN_WIDTH) : 0], {(S0_GRAN*GRAN_WIDTH){1'b0}}} : w_pld_in;
    assign lsft_w_s1 = w_buf_cnt_r[1] ? {lsft_w_s0[W_BUF_WIDTH-1-(S1_GRAN*GRAN_WIDTH) : 0], {(S1_GRAN*GRAN_WIDTH){1'b0}}} : lsft_w_s0;
    assign lsft_w_s2 = w_buf_cnt_r[2] ? {lsft_w_s1[W_BUF_WIDTH-1-(S2_GRAN*GRAN_WIDTH) : 0], {(S2_GRAN*GRAN_WIDTH){1'b0}}} : lsft_w_s1;
    assign lsft_w_s3 = w_buf_cnt_r[3] ? {lsft_w_s2[W_BUF_WIDTH-1-(S3_GRAN*GRAN_WIDTH) : 0], {(S3_GRAN*GRAN_WIDTH){1'b0}}} : lsft_w_s2;
    assign lsft_w_s4 = w_buf_cnt_r[4] ? {lsft_w_s3[W_BUF_WIDTH-1-(S4_GRAN*GRAN_WIDTH) : 0], {(S4_GRAN*GRAN_WIDTH){1'b0}}} : lsft_w_s3;
    assign lsft_w_s5 = w_buf_cnt_r[5] ? {lsft_w_s4[W_BUF_WIDTH-1-(S5_GRAN*GRAN_WIDTH) : 0], {(S5_GRAN*GRAN_WIDTH){1'b0}}} : lsft_w_s4;

    assign lsft_w_partial   = last_cn_flag_pre_r[5] ? {{(W_BUF_WIDTH-WFULL_VC_WIDTH){1'b0}}, lsft_w_s5[WFULL_VC_WIDTH-1:0]} : {W_BUF_WIDTH{1'b0}};

    assign lsft_w_new       = w_int_masked_any_pip ? last_cn_flag_pre_r[5] ? {rsft_vc[dat_vc_index][0+:W_BUF_WIDTH-WFULL_VC_WIDTH], {WFULL_VC_WIDTH{1'b0}}} : 
                                                                         {{(W_BUF_WIDTH-PLD_WIDTH){1'b0}}, rsft_vc[dat_vc_index]} : {W_BUF_WIDTH{1'b0}};

    assign lsft_w = lsft_w_partial | lsft_w_new;

    assign w_buf  = w_buf_hsk ? (w_buf_r | lsft_w) : w_buf_r;

    assign w_buf_cnt = w_buf_hsk ? (last_cn_flag_pre_r[5] & w_int_masked_any_pip) ? (WFULL_VC_WIDTH + {1'b0, w_vdn}) : (w_buf_cnt_r + {1'b0, w_vdn}) : w_buf_cnt_r;

    assign w_buf_cnt_nxt = w_tx_proc ? (w_buf_hsk & w_int_masked_any_pip) ? {1'b0, w_vdn} : {DAT_BUF_CNT_W{1'b0}} : w_buf_cnt;

    assign w_buf_nxt = w_tx_proc ? (w_buf_hsk & w_int_masked_any_pip) ? {{(W_BUF_WIDTH-PLD_WIDTH){1'b0}}, rsft_vc[dat_vc_index]} : {W_BUF_WIDTH{1'b0}} : w_buf;

    // 底部 w VC 完整且为数据 CN 时输出；wfull/wlite 字段重建（wlite: strb 全 1）
    // [bak201 修订] w_bus 组合直读 w_buf（非寄存）：依赖不变式——w VC(544/608) > payload(496) ⇒ 全跨拍，
    //   完整 VC 仅在"补全拍"（组合 cnt 达标）成形并当拍弹出，缓冲不驻留完整 VC；补全拍底部 =
    //   w_buf_r(首段) | lsft_w_partial(尾段) = 本 VC 本体（拼合），新 VC 头经 lsft_w_new flag=1 分支
    //   落缓冲上半、不在 w_bus 底部读取区 ⇒ 无跨 VC OR 污染。与 aw/ar/b 的"rdy 钳制弹拍不吸"
    //   不同机制，靠不变式(ii) 自洽（若破坏则需回退 w_buf_r）。
    assign w_valid     = w_tx_proc;
    assign w_bus_wfull = w_buf[7 +: W_WIDTH];
    assign w_bus_wlite = {w_buf[7+PAIF_DW+1 +: 4], w_buf[7+PAIF_DW], {(PAIF_DW/8){1'b1}}, w_buf[7 +: PAIF_DW]};
    assign w_bus       = wfull_bot_r ? w_bus_wfull : w_bus_wlite;


    // --------------------------------------------------------------------------
    // [bak102] SP 字段预寄存（w）：一拍滞后采样
    //   VC 在 cycle n 输出（w_valid(n)=1）⇒ 底部 SP 在 n 内为真值（K 补全拍即输出拍：
    //   补全 edge(n) 写入 SP 区，n 内组合逻辑已见真值；K 未完成时 SP 区为垃圾、w_valid=0 门控）。
    //   弹出在 edge(n+1)：同 edge 采样 w_sp_r(n+1)=n 拍底部 SP（pop 前值）⇒ w_sp_r 滞后
    //   w_buf_r 一拍，恰持于输出 VC 的 SP；w_sp_valid_r(n+1)=输出 ∧ 非零 ⇒ sp 输出拍 = n+1。
    //   启用条件与 w_buf_r 一致（w_rdy≡1 ⇒ w_tx_proc⟹w_buf_hsk，底部 SP 变化仅在 pop edge，全覆盖）。
    // --------------------------------------------------------------------------
    assign w_sp_cur = wfull_bot_r ? w_buf[WFULL_VC_WIDTH-1 -: 20]
                                  : w_buf[WLITE_VC_WIDTH-1 -: 20];
    assign w_sp_evt = w_valid & (w_sp_cur != 20'd0);

    // --------------------------------------------------------------------------
    // r 缓冲（w 同构；rfull/rlite 同宽，输出时按类型重建 r_bus）
    //   r_lite 重建：RID/RRESP 取最近一次 r_full 输出时的保存值（协议保证
    //   R_RP 内 r_full 先于其 r_lite 出现）
    // --------------------------------------------------------------------------
    // [bak103] r 组合探针（rfull_bot/rlite_bot/r_data_bot1）已删：消费端重连 next-state 寄存器。

    assign r_tx_proc   = ((r_buf_cnt >= R_VC_CNT)&&(rfull_bot_r | rlite_bot_r));
    assign r_buf_hsk   = r_req & pld_pip_hsk;

    // [BUGFIX 2026] rfull_bot/rlite_bot 底部 CN 解码误用 W 侧常量
    //   （AXI4_W_FULL_CN=19 / AXI4_W_LITE_CN=21），而 pack 侧 r_full_vc/r_lite_vc
    //   底部填的是 AXI4_R_FULL_CN=20 / AXI4_R_LITE_CN=22 —— 恒不匹配
    //   ⇒ rfull_bot_r/rlite_bot_r 恒 0 ⇒ r_tx_proc 恒 0 ⇒ r_valid 恒 0，
    //   R 数据永久滞留 r_buf 内无法弹出（读通路整条死掉）。
    //   改用 R 侧 CN 常量（对标 w 侧 wfull_bot_nxt/wlite_bot_nxt 的正确写法）。
    assign rfull_bot_nxt = (r_buf_nxt[0 +: 5] == `AXI4_R_FULL_CN);
    assign rlite_bot_nxt = (r_buf_nxt[0 +: 5] == `AXI4_R_LITE_CN);


    assign r_pld_in = {{(R_BUF_WIDTH-PLD_WIDTH){1'b0}}, pld_pip};
    assign lsft_rd_s0 = r_buf_cnt_r[0] ? {r_pld_in[R_BUF_WIDTH-1-(S0_GRAN*GRAN_WIDTH) : 0], {(S0_GRAN*GRAN_WIDTH){1'b0}}} : r_pld_in;
    assign lsft_rd_s1 = r_buf_cnt_r[1] ? {lsft_rd_s0[R_BUF_WIDTH-1-(S1_GRAN*GRAN_WIDTH) : 0], {(S1_GRAN*GRAN_WIDTH){1'b0}}} : lsft_rd_s0;
    assign lsft_rd_s2 = r_buf_cnt_r[2] ? {lsft_rd_s1[R_BUF_WIDTH-1-(S2_GRAN*GRAN_WIDTH) : 0], {(S2_GRAN*GRAN_WIDTH){1'b0}}} : lsft_rd_s1;
    assign lsft_rd_s3 = r_buf_cnt_r[3] ? {lsft_rd_s2[R_BUF_WIDTH-1-(S3_GRAN*GRAN_WIDTH) : 0], {(S3_GRAN*GRAN_WIDTH){1'b0}}} : lsft_rd_s2;
    assign lsft_rd_s4 = r_buf_cnt_r[4] ? {lsft_rd_s3[R_BUF_WIDTH-1-(S4_GRAN*GRAN_WIDTH) : 0], {(S4_GRAN*GRAN_WIDTH){1'b0}}} : lsft_rd_s3;
    assign lsft_rd_s5 = r_buf_cnt_r[5] ? {lsft_rd_s4[R_BUF_WIDTH-1-(S5_GRAN*GRAN_WIDTH) : 0], {(S5_GRAN*GRAN_WIDTH){1'b0}}} : lsft_rd_s4;

    assign lsft_rd_partial = last_cn_flag_pre_r[6] ? {{(R_BUF_WIDTH-R_VC_WIDTH){1'b0}} , lsft_rd_s5[R_VC_WIDTH-1:0]} : {R_BUF_WIDTH{1'b0}};
    assign lsft_rd_new     = r_int_masked_any_pip ? last_cn_flag_pre_r[6] ? {rsft_vc[dat_vc_index][0+:R_BUF_WIDTH-R_VC_WIDTH] , {R_VC_WIDTH{1'b0}}} :
                                                            {{(R_BUF_WIDTH-PLD_WIDTH){1'b0}} , rsft_vc[dat_vc_index]} : {R_BUF_WIDTH{1'b0}};
    assign lsft_rd = lsft_rd_partial | lsft_rd_new;

    assign r_buf = r_buf_hsk ? (r_buf_r | lsft_rd) : r_buf_r;

    assign r_buf_cnt = r_buf_hsk ? (last_cn_flag_pre_r[6] & r_int_masked_any_pip) ? (R_VC_WIDTH + {1'b0, r_vdn}) : (r_buf_cnt_r + {1'b0, r_vdn}) : r_buf_cnt_r;

    assign r_buf_cnt_nxt = r_tx_proc ? (r_buf_hsk & r_int_masked_any_pip) ? {1'b0, r_vdn} : {DAT_BUF_CNT_W{1'b0}} : r_buf_cnt;

    assign r_buf_nxt = r_tx_proc ? (r_buf_hsk & r_int_masked_any_pip) ? {{(R_BUF_WIDTH-PLD_WIDTH){1'b0}}, rsft_vc[dat_vc_index]} : {R_BUF_WIDTH{1'b0}} : r_buf;

    assign r_valid       = r_tx_proc;
    assign r_bus_rfull   = r_buf[7 +: R_WIDTH];
    assign r_lite_bus    = r_buf[7 +: R_LITE_WIDTH];
    assign r_bus_lite_recon = {r_lite_bus[PAIF_DW+4 -: 5], rid_saved_r, rresp_saved_r, r_lite_bus[PAIF_DW-1:0]};
    assign r_bus         = rfull_bot_r ? r_bus_rfull : r_bus_lite_recon;

    // --------------------------------------------------------------------------
    // [bak102] SP 字段预寄存（r）：同 w 一拍滞后模式；r_lite 承载 SP / r_full 无 SP（类型门控）。
    // --------------------------------------------------------------------------
    assign r_sp_cur = rlite_bot_r ? r_buf[R_VC_WIDTH-1 -: 20] : 20'd0;
    assign r_sp_evt = r_valid & (r_sp_cur != 20'd0);

    // --------------------------------------------------------------------------
    // b 缓冲（aw 同构 + C3：底部完整 b_vc 遇 SP 输出拍时 hold——不弹不吸，
    //   该通道 rdy 拉低停步；SP 拍过后正常弹出输出）
    // --------------------------------------------------------------------------
    assign b_pld_in = last_cn_flag_pre_r[7] ? {{(AX_BUF_WIDTH-PLD_WIDTH){1'b0}}, pld_pip} :
                                       {{(AX_BUF_WIDTH-PLD_WIDTH){1'b0}}, rsft_vc[b_vc_index]};

    assign lsft_b_s0 = b_buf_cnt_r[0] ? {b_pld_in[AX_BUF_WIDTH-1-(S0_GRAN*GRAN_WIDTH) : 0], {(S0_GRAN*GRAN_WIDTH){1'b0}}} : b_pld_in;
    assign lsft_b_s1 = b_buf_cnt_r[1] ? {lsft_b_s0[AX_BUF_WIDTH-1-(S1_GRAN*GRAN_WIDTH) : 0], {(S1_GRAN*GRAN_WIDTH){1'b0}}} : lsft_b_s0;
    assign lsft_b_s2 = b_buf_cnt_r[2] ? {lsft_b_s1[AX_BUF_WIDTH-1-(S2_GRAN*GRAN_WIDTH) : 0], {(S2_GRAN*GRAN_WIDTH){1'b0}}} : lsft_b_s1;
    generate
        if(PAIF_DW == 1024) begin : g_lsft_b_1024
            assign lsft_b = lsft_b_s1;
        end else begin : g_lsft_b_512
            assign lsft_b = lsft_b_s2;
        end
    endgenerate

    assign b_buf_hsk  = b_req & pld_pip_hsk;
    // [bak201] b_tx_proc = 弹（底部完整 b_vc 且非 SP 拍）| 排底（底部非 b 内容，flush）
    assign b_tx_proc  = ((b_buf_cnt >= AX_VC_CNT) & b_cn_get[0] & (~sp_out_beat)) | ((|b_buf_cnt_r) && (b_buf_r[0+:5] != `AXI4_B_CN));


    assign b_buf = b_buf_hsk ? (b_buf_r | lsft_b) : b_buf_r;
    assign b_buf_cnt = b_buf_hsk ? (b_buf_cnt_r + b_vdn) : b_buf_cnt_r;
    // [bak201] SP 拍 hold：合法 b_vc 弹出（txA）受 ~sp_out_beat 门控（SP 拍不弹，K 补全遇 SP 拍保留缓冲下拍再弹——防吞拍）；
    //   排底（txB，flush 非 b 垃圾）与纯吸收（hsk）不受 SP 阻塞 → hold 判定用 ~b_tx_proc 而非 sp（精确）
    assign b_buf_cnt_nxt = (~b_tx_proc) ? b_buf_cnt :
                          (b_cn_get[8:1] == 8'd0) ? {AX_BUF_CNT_W{1'b0}} :
                          b_cn_get[1] ? (b_buf_cnt - AX_VC_CNT) : (b_buf_cnt - (2*AX_VC_CNT));
    assign b_buf_nxt = (~b_tx_proc) ? b_buf :
                       (b_cn_get[8:1] == 8'd0) ? {AX_BUF_WIDTH{1'b0}} :
                       b_cn_get[1] ? (b_buf >> AX_VC_WIDTH) : (b_buf >> (2*AX_VC_WIDTH));


    assign b_cn_get[0] = (b_buf[0*AX_VC_WIDTH +: 5] == `AXI4_B_CN);
    assign b_cn_get[1] = (b_buf[1*AX_VC_WIDTH +: 5] == `AXI4_B_CN);
    assign b_cn_get[2] = (b_buf[2*AX_VC_WIDTH +: 5] == `AXI4_B_CN);
    assign b_cn_get[3] = (b_buf[3*AX_VC_WIDTH +: 5] == `AXI4_B_CN);
    assign b_cn_get[4] = (b_buf[4*AX_VC_WIDTH +: 5] == `AXI4_B_CN);
    generate
        if(PAIF_DW == 1024) begin : g_b_cnget_hi
            assign b_cn_get[5] = (b_buf[5*AX_VC_WIDTH +: 5] == `AXI4_B_CN);
            assign b_cn_get[6] = (b_buf[6*AX_VC_WIDTH +: 5] == `AXI4_B_CN);
            assign b_cn_get[7] = (b_buf[7*AX_VC_WIDTH +: 5] == `AXI4_B_CN);
            assign b_cn_get[8] = (b_buf[8*AX_VC_WIDTH +: 5] == `AXI4_B_CN);
        end else begin : g_b_cnget_lo
            assign b_cn_get[5] = 1'b0;
            assign b_cn_get[6] = 1'b0;
            assign b_cn_get[7] = 1'b0;
            assign b_cn_get[8] = 1'b0;
        end
    endgenerate

    assign b_vc = b_buf[0 +: AX_VC_WIDTH];
    assign bvc_valid = ((b_buf_cnt >= AX_VC_CNT) & b_cn_get[0] & (~sp_out_beat));

    // ==========================================================================
    // SP 字段译码（数据 VC 顶 20bit；TYPE 自描述，格式与 pack 对应）
    //   20bit 布局: type[1:0] | b_bus[19:4] | b_rp_sel[3:2] | 各 RP credit 2bit 段
    //   type0: awcrd@[3:2..9:8] + wcrd@[11:10..17:16]
    //   type1: rcrd@[3:2..9:8] + bcrd@[11:10..17:16]
    //   type2: arcrd@[3:2..9:8]
    //   type3: b_bus@[19:4] + b_rp_sel@[3:2]（选 1 个 RP）
    //   承载: w VC(full/lite) 与 r_lite；r_full 无 SP
    // ==========================================================================
    // [bak102] sp_field 改从预寄存 SP（w_sp_r/r_sp_r，滞后 w/r_buf 一拍）选取；sp_valid 纯寄存器。
    //   w/r sp 输出拍冲突 ⟺ w/r 同拍输出（C2 禁止）⇒ 优先式/ OR 在可达态下均位等；
    //   空 SP 的 VC 不产生 sp 输出拍（w_sp_evt/r_sp_evt 含非零判定）。
    assign sp_field = ({20{w_valid}} & w_sp_cur) | ({20{r_valid}} & r_sp_cur);
    assign sp_type  = sp_field[1:0];
    assign sp_b_bus = sp_field[19:4];
    assign sp_b_rp_sel = sp_field[3:2];
    // SP 有效：输出 (w|r) VC 的 SP 非零（组合，与 VC 输出同拍；C3 需 b 在 SP 拍 hold）
    assign sp_valid = w_sp_evt | r_sp_evt;
    assign sp_out_beat = sp_valid;

    // SP type0: AW/W Credit（值按 TYPE 门控: 非 type0 拍归 0，防其他类字段串位）
    assign sp_awcrd_rp0_w = (sp_valid & (sp_type==2'd0)) ? sp_field[3:2] : 2'd0;
    assign sp_awcrd_rp1_w = (sp_valid & (sp_type==2'd0)) ? sp_field[5:4] : 2'd0;
    assign sp_awcrd_rp2_w = (sp_valid & (sp_type==2'd0)) ? sp_field[7:6] : 2'd0;
    assign sp_awcrd_rp3_w = (sp_valid & (sp_type==2'd0)) ? sp_field[9:8] : 2'd0;
    assign sp_wcrd_rp0_w = (sp_valid & (sp_type==2'd0)) ? sp_field[11:10] : 2'd0;
    assign sp_wcrd_rp1_w = (sp_valid & (sp_type==2'd0)) ? sp_field[13:12] : 2'd0;
    assign sp_wcrd_rp2_w = (sp_valid & (sp_type==2'd0)) ? sp_field[15:14] : 2'd0;
    assign sp_wcrd_rp3_w = (sp_valid & (sp_type==2'd0)) ? sp_field[17:16] : 2'd0;
    assign sp_awcrd_rp0_valid_w = sp_valid & (sp_type==2'd0) & (|sp_awcrd_rp0_w);
    assign sp_awcrd_rp1_valid_w = sp_valid & (sp_type==2'd0) & (|sp_awcrd_rp1_w);
    assign sp_awcrd_rp2_valid_w = sp_valid & (sp_type==2'd0) & (|sp_awcrd_rp2_w);
    assign sp_awcrd_rp3_valid_w = sp_valid & (sp_type==2'd0) & (|sp_awcrd_rp3_w);
    assign sp_wcrd_rp0_valid_w  = sp_valid & (sp_type==2'd0) & (|sp_wcrd_rp0_w);
    assign sp_wcrd_rp1_valid_w  = sp_valid & (sp_type==2'd0) & (|sp_wcrd_rp1_w);
    assign sp_wcrd_rp2_valid_w  = sp_valid & (sp_type==2'd0) & (|sp_wcrd_rp2_w);
    assign sp_wcrd_rp3_valid_w  = sp_valid & (sp_type==2'd0) & (|sp_wcrd_rp3_w);

    // SP type1: R/B Credit（值按 TYPE 门控: 非 type1 拍归 0，防其他类字段串位）
    assign sp_rcrd_rp0_w = (sp_valid & (sp_type==2'd1)) ? sp_field[3:2] : 2'd0;
    assign sp_rcrd_rp1_w = (sp_valid & (sp_type==2'd1)) ? sp_field[5:4] : 2'd0;
    assign sp_rcrd_rp2_w = (sp_valid & (sp_type==2'd1)) ? sp_field[7:6] : 2'd0;
    assign sp_rcrd_rp3_w = (sp_valid & (sp_type==2'd1)) ? sp_field[9:8] : 2'd0;
    assign sp_bcrd_rp0_w = (sp_valid & (sp_type==2'd1)) ? sp_field[11:10] : 2'd0;
    assign sp_bcrd_rp1_w = (sp_valid & (sp_type==2'd1)) ? sp_field[13:12] : 2'd0;
    assign sp_bcrd_rp2_w = (sp_valid & (sp_type==2'd1)) ? sp_field[15:14] : 2'd0;
    assign sp_bcrd_rp3_w = (sp_valid & (sp_type==2'd1)) ? sp_field[17:16] : 2'd0;
    assign sp_rcrd_rp0_valid_w  = sp_valid & (sp_type==2'd1) & (|sp_rcrd_rp0_w);
    assign sp_rcrd_rp1_valid_w  = sp_valid & (sp_type==2'd1) & (|sp_rcrd_rp1_w);
    assign sp_rcrd_rp2_valid_w  = sp_valid & (sp_type==2'd1) & (|sp_rcrd_rp2_w);
    assign sp_rcrd_rp3_valid_w  = sp_valid & (sp_type==2'd1) & (|sp_rcrd_rp3_w);
    assign sp_bcrd_rp0_valid_w  = sp_valid & (sp_type==2'd1) & (|sp_bcrd_rp0_w);
    assign sp_bcrd_rp1_valid_w  = sp_valid & (sp_type==2'd1) & (|sp_bcrd_rp1_w);
    assign sp_bcrd_rp2_valid_w  = sp_valid & (sp_type==2'd1) & (|sp_bcrd_rp2_w);
    assign sp_bcrd_rp3_valid_w  = sp_valid & (sp_type==2'd1) & (|sp_bcrd_rp3_w);

    // SP type2: AR Credit（值按 TYPE 门控: 非 type2 拍归 0，防其他类字段串位）
    assign sp_arcrd_rp0_w = (sp_valid & (sp_type==2'd2)) ? sp_field[3:2] : 2'd0;
    assign sp_arcrd_rp1_w = (sp_valid & (sp_type==2'd2)) ? sp_field[5:4] : 2'd0;
    assign sp_arcrd_rp2_w = (sp_valid & (sp_type==2'd2)) ? sp_field[7:6] : 2'd0;
    assign sp_arcrd_rp3_w = (sp_valid & (sp_type==2'd2)) ? sp_field[9:8] : 2'd0;
    assign sp_arcrd_rp0_valid_w = sp_valid & (sp_type==2'd2) & (|sp_arcrd_rp0_w);
    assign sp_arcrd_rp1_valid_w = sp_valid & (sp_type==2'd2) & (|sp_arcrd_rp1_w);
    assign sp_arcrd_rp2_valid_w = sp_valid & (sp_type==2'd2) & (|sp_arcrd_rp2_w);
    assign sp_arcrd_rp3_valid_w = sp_valid & (sp_type==2'd2) & (|sp_arcrd_rp3_w);

    // SP type3: B 数据（按本 RP 有效门控: sp_b_rpX_valid_w = sp_valid & type3 & sel==X；
    //   非本 RP 拍归 0，保证 OR 合并时 valid 低 bus 必 0）
    assign sp_b_rp0_bus_w = sp_b_rp0_valid_w ? sp_b_bus : 16'd0;
    assign sp_b_rp1_bus_w = sp_b_rp1_valid_w ? sp_b_bus : 16'd0;
    assign sp_b_rp2_bus_w = sp_b_rp2_valid_w ? sp_b_bus : 16'd0;
    assign sp_b_rp3_bus_w = sp_b_rp3_valid_w ? sp_b_bus : 16'd0;
    assign sp_b_rp0_valid_w = sp_valid & (sp_type==2'd3) & (sp_b_rp_sel==2'd0);
    assign sp_b_rp1_valid_w = sp_valid & (sp_type==2'd3) & (sp_b_rp_sel==2'd1);
    assign sp_b_rp2_valid_w = sp_valid & (sp_type==2'd3) & (sp_b_rp_sel==2'd2);
    assign sp_b_rp3_valid_w = sp_valid & (sp_type==2'd3) & (sp_b_rp_sel==2'd3);

    // ==========================================================================
    // b_vc 字段译码（128bit；权威格式与 pack 一致：
    //   CN[4:0] | b_rpv[8:5] | B_RP0..3 | AWcrd | ARcrd | Wcrd | Rcrd | Bcrd | pad）
    // ==========================================================================
    // bvc 侧按本 RP 有效门控（bvc_b_rpX_valid_w = bvc_valid & b_rpv[X]；
    //   pack 的 B_RP 字段为原始 bus 值，须门控保证 valid 低时归 0）
    assign bvc_b_rp0_bus_w = bvc_b_rp0_valid_w ? b_vc[9  +: 16] : 16'd0;
    assign bvc_b_rp1_bus_w = bvc_b_rp1_valid_w ? b_vc[25 +: 16] : 16'd0;
    assign bvc_b_rp2_bus_w = bvc_b_rp2_valid_w ? b_vc[41 +: 16] : 16'd0;
    assign bvc_b_rp3_bus_w = bvc_b_rp3_valid_w ? b_vc[57 +: 16] : 16'd0;
    // b_rpv[8:5]: bit5=rp0, bit6=rp1, bit7=rp2, bit8=rp3
    assign bvc_b_rp0_valid_w = bvc_valid & b_vc[5];
    assign bvc_b_rp1_valid_w = bvc_valid & b_vc[6];
    assign bvc_b_rp2_valid_w = bvc_valid & b_vc[7];
    assign bvc_b_rp3_valid_w = bvc_valid & b_vc[8];
    // bvc credit 值按 bvc_valid 拍级门控（非 b_vc 拍归 0，防缓冲残留串位）
    assign bvc_awcrd_rp0_w = bvc_valid ? b_vc[73 +: 2] : 2'd0;
    assign bvc_awcrd_rp1_w = bvc_valid ? b_vc[75 +: 2] : 2'd0;
    assign bvc_awcrd_rp2_w = bvc_valid ? b_vc[77 +: 2] : 2'd0;
    assign bvc_awcrd_rp3_w = bvc_valid ? b_vc[79 +: 2] : 2'd0;
    assign bvc_arcrd_rp0_w = bvc_valid ? b_vc[81 +: 2] : 2'd0;
    assign bvc_arcrd_rp1_w = bvc_valid ? b_vc[83 +: 2] : 2'd0;
    assign bvc_arcrd_rp2_w = bvc_valid ? b_vc[85 +: 2] : 2'd0;
    assign bvc_arcrd_rp3_w = bvc_valid ? b_vc[87 +: 2] : 2'd0;
    assign bvc_wcrd_rp0_w = bvc_valid ? b_vc[89 +: 2] : 2'd0;
    assign bvc_wcrd_rp1_w = bvc_valid ? b_vc[91 +: 2] : 2'd0;
    assign bvc_wcrd_rp2_w = bvc_valid ? b_vc[93 +: 2] : 2'd0;
    assign bvc_wcrd_rp3_w = bvc_valid ? b_vc[95 +: 2] : 2'd0;
    assign bvc_rcrd_rp0_w = bvc_valid ? b_vc[97 +: 2] : 2'd0;
    assign bvc_rcrd_rp1_w = bvc_valid ? b_vc[99 +: 2] : 2'd0;
    assign bvc_rcrd_rp2_w = bvc_valid ? b_vc[101 +: 2] : 2'd0;
    assign bvc_rcrd_rp3_w = bvc_valid ? b_vc[103 +: 2] : 2'd0;
    assign bvc_bcrd_rp0_w = bvc_valid ? b_vc[105 +: 2] : 2'd0;
    assign bvc_bcrd_rp1_w = bvc_valid ? b_vc[107 +: 2] : 2'd0;
    assign bvc_bcrd_rp2_w = bvc_valid ? b_vc[109 +: 2] : 2'd0;
    assign bvc_bcrd_rp3_w = bvc_valid ? b_vc[111 +: 2] : 2'd0;
    assign bvc_awcrd_rp0_valid_w = bvc_valid & (|bvc_awcrd_rp0_w);
    assign bvc_awcrd_rp1_valid_w = bvc_valid & (|bvc_awcrd_rp1_w);
    assign bvc_awcrd_rp2_valid_w = bvc_valid & (|bvc_awcrd_rp2_w);
    assign bvc_awcrd_rp3_valid_w = bvc_valid & (|bvc_awcrd_rp3_w);
    assign bvc_wcrd_rp0_valid_w  = bvc_valid & (|bvc_wcrd_rp0_w);
    assign bvc_wcrd_rp1_valid_w  = bvc_valid & (|bvc_wcrd_rp1_w);
    assign bvc_wcrd_rp2_valid_w  = bvc_valid & (|bvc_wcrd_rp2_w);
    assign bvc_wcrd_rp3_valid_w  = bvc_valid & (|bvc_wcrd_rp3_w);
    assign bvc_arcrd_rp0_valid_w = bvc_valid & (|bvc_arcrd_rp0_w);
    assign bvc_arcrd_rp1_valid_w = bvc_valid & (|bvc_arcrd_rp1_w);
    assign bvc_arcrd_rp2_valid_w = bvc_valid & (|bvc_arcrd_rp2_w);
    assign bvc_arcrd_rp3_valid_w = bvc_valid & (|bvc_arcrd_rp3_w);
    assign bvc_rcrd_rp0_valid_w  = bvc_valid & (|bvc_rcrd_rp0_w);
    assign bvc_rcrd_rp1_valid_w  = bvc_valid & (|bvc_rcrd_rp1_w);
    assign bvc_rcrd_rp2_valid_w  = bvc_valid & (|bvc_rcrd_rp2_w);
    assign bvc_rcrd_rp3_valid_w  = bvc_valid & (|bvc_rcrd_rp3_w);
    assign bvc_bcrd_rp0_valid_w  = bvc_valid & (|bvc_bcrd_rp0_w);
    assign bvc_bcrd_rp1_valid_w  = bvc_valid & (|bvc_bcrd_rp1_w);
    assign bvc_bcrd_rp2_valid_w  = bvc_valid & (|bvc_bcrd_rp2_w);
    assign bvc_bcrd_rp3_valid_w  = bvc_valid & (|bvc_bcrd_rp3_w);

    // ==========================================================================
    // 输出级合并（C3 时隙化：SP 输出拍阻塞 b_vc 输出，sp_valid 与 bvc_valid
    //   RTL 结构性互斥 → OR 合并无损；B/credit 两侧值均按各自 valid 门控，
    //   保证 C5：valid 未拉高时 bus 必 0；W/R/AW 组合输出）
    // ==========================================================================
    // --- B 通道（4 RP，OR 合并；两侧已按各自 valid 门控，sp/bvc 互斥保证无损）---
    assign b_rp0_bus_nxt   = sp_b_rp0_bus_w | bvc_b_rp0_bus_w;
    assign b_rp0_valid_nxt = sp_b_rp0_valid_w | bvc_b_rp0_valid_w;
    assign b_rp1_bus_nxt   = sp_b_rp1_bus_w | bvc_b_rp1_bus_w;
    assign b_rp1_valid_nxt = sp_b_rp1_valid_w | bvc_b_rp1_valid_w;
    assign b_rp2_bus_nxt   = sp_b_rp2_bus_w | bvc_b_rp2_bus_w;
    assign b_rp2_valid_nxt = sp_b_rp2_valid_w | bvc_b_rp2_valid_w;
    assign b_rp3_bus_nxt   = sp_b_rp3_bus_w | bvc_b_rp3_bus_w;
    assign b_rp3_valid_nxt = sp_b_rp3_valid_w | bvc_b_rp3_valid_w;

    // --- AW Credit ---
    assign awcrd_rp0_nxt   = sp_awcrd_rp0_w | bvc_awcrd_rp0_w;
    assign awcrd_rp0_valid_nxt = sp_awcrd_rp0_valid_w | bvc_awcrd_rp0_valid_w;
    assign awcrd_rp1_nxt   = sp_awcrd_rp1_w | bvc_awcrd_rp1_w;
    assign awcrd_rp1_valid_nxt = sp_awcrd_rp1_valid_w | bvc_awcrd_rp1_valid_w;
    assign awcrd_rp2_nxt   = sp_awcrd_rp2_w | bvc_awcrd_rp2_w;
    assign awcrd_rp2_valid_nxt = sp_awcrd_rp2_valid_w | bvc_awcrd_rp2_valid_w;
    assign awcrd_rp3_nxt   = sp_awcrd_rp3_w | bvc_awcrd_rp3_w;
    assign awcrd_rp3_valid_nxt = sp_awcrd_rp3_valid_w | bvc_awcrd_rp3_valid_w;

    // --- W Credit ---
    assign wcrd_rp0_nxt   = sp_wcrd_rp0_w | bvc_wcrd_rp0_w;
    assign wcrd_rp0_valid_nxt = sp_wcrd_rp0_valid_w | bvc_wcrd_rp0_valid_w;
    assign wcrd_rp1_nxt   = sp_wcrd_rp1_w | bvc_wcrd_rp1_w;
    assign wcrd_rp1_valid_nxt = sp_wcrd_rp1_valid_w | bvc_wcrd_rp1_valid_w;
    assign wcrd_rp2_nxt   = sp_wcrd_rp2_w | bvc_wcrd_rp2_w;
    assign wcrd_rp2_valid_nxt = sp_wcrd_rp2_valid_w | bvc_wcrd_rp2_valid_w;
    assign wcrd_rp3_nxt   = sp_wcrd_rp3_w | bvc_wcrd_rp3_w;
    assign wcrd_rp3_valid_nxt = sp_wcrd_rp3_valid_w | bvc_wcrd_rp3_valid_w;

    // --- AR Credit ---
    assign arcrd_rp0_nxt   = sp_arcrd_rp0_w | bvc_arcrd_rp0_w;
    assign arcrd_rp0_valid_nxt = sp_arcrd_rp0_valid_w | bvc_arcrd_rp0_valid_w;
    assign arcrd_rp1_nxt   = sp_arcrd_rp1_w | bvc_arcrd_rp1_w;
    assign arcrd_rp1_valid_nxt = sp_arcrd_rp1_valid_w | bvc_arcrd_rp1_valid_w;
    assign arcrd_rp2_nxt   = sp_arcrd_rp2_w | bvc_arcrd_rp2_w;
    assign arcrd_rp2_valid_nxt = sp_arcrd_rp2_valid_w | bvc_arcrd_rp2_valid_w;
    assign arcrd_rp3_nxt   = sp_arcrd_rp3_w | bvc_arcrd_rp3_w;
    assign arcrd_rp3_valid_nxt = sp_arcrd_rp3_valid_w | bvc_arcrd_rp3_valid_w;

    // --- R Credit ---
    assign rcrd_rp0_nxt   = sp_rcrd_rp0_w | bvc_rcrd_rp0_w;
    assign rcrd_rp0_valid_nxt = sp_rcrd_rp0_valid_w | bvc_rcrd_rp0_valid_w;
    assign rcrd_rp1_nxt   = sp_rcrd_rp1_w | bvc_rcrd_rp1_w;
    assign rcrd_rp1_valid_nxt = sp_rcrd_rp1_valid_w | bvc_rcrd_rp1_valid_w;
    assign rcrd_rp2_nxt   = sp_rcrd_rp2_w | bvc_rcrd_rp2_w;
    assign rcrd_rp2_valid_nxt = sp_rcrd_rp2_valid_w | bvc_rcrd_rp2_valid_w;
    assign rcrd_rp3_nxt   = sp_rcrd_rp3_w | bvc_rcrd_rp3_w;
    assign rcrd_rp3_valid_nxt = sp_rcrd_rp3_valid_w | bvc_rcrd_rp3_valid_w;

    // --- B Credit ---
    assign bcrd_rp0_nxt   = sp_bcrd_rp0_w | bvc_bcrd_rp0_w;
    assign bcrd_rp0_valid_nxt = sp_bcrd_rp0_valid_w | bvc_bcrd_rp0_valid_w;
    assign bcrd_rp1_nxt   = sp_bcrd_rp1_w | bvc_bcrd_rp1_w;
    assign bcrd_rp1_valid_nxt = sp_bcrd_rp1_valid_w | bvc_bcrd_rp1_valid_w;
    assign bcrd_rp2_nxt   = sp_bcrd_rp2_w | bvc_bcrd_rp2_w;
    assign bcrd_rp2_valid_nxt = sp_bcrd_rp2_valid_w | bvc_bcrd_rp2_valid_w;
    assign bcrd_rp3_nxt   = sp_bcrd_rp3_w | bvc_bcrd_rp3_w;
    assign bcrd_rp3_valid_nxt = sp_bcrd_rp3_valid_w | bvc_bcrd_rp3_valid_w;

    // ==========================================================================
    // busy（上游有数据未排空）
    // ==========================================================================
    assign busy = paif_rxvalid | pld_byp_vld | pld_pip_vld |
                  (|aw_buf_cnt_r) | (|ar0_buf_cnt_r) | (|ar1_buf_cnt_r) | (|ar2_buf_cnt_r) |
                  (|ar3_buf_cnt_r) | (|w_buf_cnt_r) | (|r_buf_cnt_r) | (|b_buf_cnt_r);
    // ==========================================================================
    // 时序逻辑
    // ==========================================================================

    // paif 相位（512 每次握手翻转；1024 本信号不存在，payload 取固定偏移）
    // [bak103] 条件编译：仅 PAIF_DW==512 时为时钟寄存器；1024 时本信号根本不需要（payload mux
    //   的 1024 支 g_p1024_sel 不引用）⇒ 无寄存器、无赋值，未驱动空网名 DC 修剪
    generate
        if (PAIF_DW == 512) begin : g_paif_phs
            always @(posedge clk or negedge rstn) begin
                if (!rstn) paif_phs_r <= 1'b0;
                else if (paif_hsk) paif_phs_r <= ~paif_phs_r;
            end
        end
    endgenerate


    // --------------------------------------------------------------------------
    // 通道缓冲寄存器（tx_proc = 弹出/排空拍；hsk = 吸收拍；二者皆无则保持）
    // --------------------------------------------------------------------------
    always @(posedge clk or negedge rstn) begin
        if (!rstn) aw_buf_r <= {AX_BUF_WIDTH{1'b0}};
        else if (aw_tx_proc || aw_buf_hsk) aw_buf_r <= aw_buf_nxt;
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) aw_buf_cnt_r <= {AX_BUF_CNT_W{1'b0}};
        else if (aw_tx_proc || aw_buf_hsk) aw_buf_cnt_r <= aw_buf_cnt_nxt;
    end


    always @(posedge clk or negedge rstn) begin
        if (!rstn) ar0_buf_r <= {AX_BUF_WIDTH{1'b0}};
        else if (ar0_tx_proc || ar0_buf_hsk) ar0_buf_r <= ar0_buf_nxt;
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) ar0_buf_cnt_r <= {AX_BUF_CNT_W{1'b0}};
        else if (ar0_tx_proc || ar0_buf_hsk) ar0_buf_cnt_r <= ar0_buf_cnt_nxt;
    end


    always @(posedge clk or negedge rstn) begin
        if (!rstn) ar1_buf_r <= {AX_BUF_WIDTH{1'b0}};
        else if (ar1_tx_proc || ar1_buf_hsk) ar1_buf_r <= ar1_buf_nxt;
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) ar1_buf_cnt_r <= {AX_BUF_CNT_W{1'b0}};
        else if (ar1_tx_proc || ar1_buf_hsk) ar1_buf_cnt_r <= ar1_buf_cnt_nxt;
    end


    always @(posedge clk or negedge rstn) begin
        if (!rstn) ar2_buf_r <= {AX_BUF_WIDTH{1'b0}};
        else if (ar2_tx_proc || ar2_buf_hsk) ar2_buf_r <= ar2_buf_nxt;
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) ar2_buf_cnt_r <= {AX_BUF_CNT_W{1'b0}};
        else if (ar2_tx_proc || ar2_buf_hsk) ar2_buf_cnt_r <= ar2_buf_cnt_nxt;
    end


    always @(posedge clk or negedge rstn) begin
        if (!rstn) ar3_buf_r <= {AX_BUF_WIDTH{1'b0}};
        else if (ar3_tx_proc || ar3_buf_hsk) ar3_buf_r <= ar3_buf_nxt;
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) ar3_buf_cnt_r <= {AX_BUF_CNT_W{1'b0}};
        else if (ar3_tx_proc || ar3_buf_hsk) ar3_buf_cnt_r <= ar3_buf_cnt_nxt;
    end


    always @(posedge clk or negedge rstn) begin
        if (!rstn) w_buf_r <= {W_BUF_WIDTH{1'b0}};
        else if (w_tx_proc || w_buf_hsk) w_buf_r <= w_buf_nxt;
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) w_buf_cnt_r <= {DAT_BUF_CNT_W{1'b0}};
        else if (w_tx_proc || w_buf_hsk) w_buf_cnt_r <= w_buf_cnt_nxt;
    end

    // [bak103] w 探针寄存器（与 w_buf_r 同更新拍；D = w_buf_nxt 解码，next-state）
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            wfull_bot_r   <= 1'b0;
            wlite_bot_r   <= 1'b0;
        end else if (w_tx_proc || w_buf_hsk) begin
            wfull_bot_r   <= wfull_bot_nxt;
            wlite_bot_r   <= wlite_bot_nxt;
        end
    end

    always @(posedge clk or negedge rstn) begin
        if (!rstn) r_buf_r <= {R_BUF_WIDTH{1'b0}};
        else if (r_tx_proc || r_buf_hsk) r_buf_r <= r_buf_nxt;
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) r_buf_cnt_r <= {DAT_BUF_CNT_W{1'b0}};
        else if (r_tx_proc || r_buf_hsk) r_buf_cnt_r <= r_buf_cnt_nxt;
    end

    // [bak103] r 探针寄存器（与 r_buf_r 同更新拍；D = r_buf_nxt 解码，next-state）
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            rfull_bot_r   <= 1'b0;
            rlite_bot_r   <= 1'b0;
        end else if (r_tx_proc || r_buf_hsk) begin
            rfull_bot_r   <= rfull_bot_nxt;
            rlite_bot_r   <= rlite_bot_nxt;
        end
    end

    // b 缓冲（b_hold 拍既不弹也不吸，整体保持；弹= b_tx_proc，吸= b_buf_hsk）
    always @(posedge clk or negedge rstn) begin
        if (!rstn) b_buf_r <= {AX_BUF_WIDTH{1'b0}};
        else if (b_tx_proc || b_buf_hsk) b_buf_r <= b_buf_nxt;
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) b_buf_cnt_r <= {AX_BUF_CNT_W{1'b0}};
        else if (b_tx_proc || b_buf_hsk) b_buf_cnt_r <= b_buf_cnt_nxt;
    end


    // r_full 输出时保存 RID/RRESP（供其后 r_lite 重建；协议保证 r_full 先出）
    always @(posedge clk or negedge rstn) begin
        if (!rstn) rid_saved_r <= 10'd0;
        else if (r_valid & rfull_bot_r) rid_saved_r <= r_bus_rfull[PAIF_DW+11 -: 10];
    end
    always @(posedge clk or negedge rstn) begin
        if (!rstn) rresp_saved_r <= 2'd0;
        else if (r_valid & rfull_bot_r) rresp_saved_r <= r_bus_rfull[PAIF_DW+1 -: 2];
    end

    // ==========================================================================
    // 子模块例化
    // ==========================================================================
    // byp 级：PAIF RX payload 寄存（s_ready 由内部 rdy_r 打拍回推 → paif_rxready）
    p_data_byp #(
        .DATA_W (PLD_WIDTH)
    ) u_paif_byp (
        .clk        (clk),
        .rst_n      (rstn),

        .s_data     (payload),
        .s_valid    (paif_rxvalid),
        .s_ready    (paif_rxready),

        .m_data     (pld_byp),
        .m_valid    (pld_byp_vld),
        .m_ready    (pld_byp_rdy)
    );

    // pip 级：对齐流水（下游 pld_pip_rdy 由 8 通道缓冲就绪联合给出）
    p_data_pip #(
        .DATA_W (PLD_WIDTH)
    ) u_paif_pip (
        .clk        (clk),
        .rst_n      (rstn),

        .s_data     (pld_byp),
        .s_valid    (pld_byp_vld),
        .s_ready    (pld_byp_rdy),

        .m_data     (pld_pip),
        .m_valid    (pld_pip_vld),
        .m_ready    (pld_pip_rdy)
    );

    // ==========================================================================
    // 输出接线
    // ==========================================================================


    // ar（4 RP）：段内组合直出端口（[bak108r]/[bak201]，无 *_nxt 中间信号、无输出级寄存器）

    // w / r：组合输出（w_bus/w_valid/r_valid 已在各自缓冲段组合定义；r_bus 亦已在 r 段由 rfull/rlite 重建）

    // b（4 RP）：组合直出（SP/b_vc 时隙合并后，去输出级一拍——[bak201]）
    assign b_rp0_bus   = b_rp0_bus_nxt;
    assign b_rp0_valid = b_rp0_valid_nxt;
    assign b_rp1_bus   = b_rp1_bus_nxt;
    assign b_rp1_valid = b_rp1_valid_nxt;
    assign b_rp2_bus   = b_rp2_bus_nxt;
    assign b_rp2_valid = b_rp2_valid_nxt;
    assign b_rp3_bus   = b_rp3_bus_nxt;
    assign b_rp3_valid = b_rp3_valid_nxt;

    // 五类 credit（4 RP）：组合直出（[bak201 修订] 去 C4 输出级寄存器；C5 由 *_nxt 的 valid 门控保证）
    assign awcrd_rp0   = awcrd_rp0_nxt;
    assign awcrd_rp0_valid = awcrd_rp0_valid_nxt;
    assign awcrd_rp1   = awcrd_rp1_nxt;
    assign awcrd_rp1_valid = awcrd_rp1_valid_nxt;
    assign awcrd_rp2   = awcrd_rp2_nxt;
    assign awcrd_rp2_valid = awcrd_rp2_valid_nxt;
    assign awcrd_rp3   = awcrd_rp3_nxt;
    assign awcrd_rp3_valid = awcrd_rp3_valid_nxt;
    assign wcrd_rp0    = wcrd_rp0_nxt;
    assign wcrd_rp0_valid = wcrd_rp0_valid_nxt;
    assign wcrd_rp1    = wcrd_rp1_nxt;
    assign wcrd_rp1_valid = wcrd_rp1_valid_nxt;
    assign wcrd_rp2    = wcrd_rp2_nxt;
    assign wcrd_rp2_valid = wcrd_rp2_valid_nxt;
    assign wcrd_rp3    = wcrd_rp3_nxt;
    assign wcrd_rp3_valid = wcrd_rp3_valid_nxt;
    assign arcrd_rp0   = arcrd_rp0_nxt;
    assign arcrd_rp0_valid = arcrd_rp0_valid_nxt;
    assign arcrd_rp1   = arcrd_rp1_nxt;
    assign arcrd_rp1_valid = arcrd_rp1_valid_nxt;
    assign arcrd_rp2   = arcrd_rp2_nxt;
    assign arcrd_rp2_valid = arcrd_rp2_valid_nxt;
    assign arcrd_rp3   = arcrd_rp3_nxt;
    assign arcrd_rp3_valid = arcrd_rp3_valid_nxt;
    assign rcrd_rp0    = rcrd_rp0_nxt;
    assign rcrd_rp0_valid = rcrd_rp0_valid_nxt;
    assign rcrd_rp1    = rcrd_rp1_nxt;
    assign rcrd_rp1_valid = rcrd_rp1_valid_nxt;
    assign rcrd_rp2    = rcrd_rp2_nxt;
    assign rcrd_rp2_valid = rcrd_rp2_valid_nxt;
    assign rcrd_rp3    = rcrd_rp3_nxt;
    assign rcrd_rp3_valid = rcrd_rp3_valid_nxt;
    assign bcrd_rp0    = bcrd_rp0_nxt;
    assign bcrd_rp0_valid = bcrd_rp0_valid_nxt;
    assign bcrd_rp1    = bcrd_rp1_nxt;
    assign bcrd_rp1_valid = bcrd_rp1_valid_nxt;
    assign bcrd_rp2    = bcrd_rp2_nxt;
    assign bcrd_rp2_valid = bcrd_rp2_valid_nxt;
    assign bcrd_rp3    = bcrd_rp3_nxt;
    assign bcrd_rp3_valid = bcrd_rp3_valid_nxt;

endmodule
