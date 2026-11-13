// =============================================================================
// module name：lane_dskew
// author: OpenHappy Team
// MAS：spec/link_layer/submods/link_rx/mas/lane_dskew_mas.md
// 功能概述: RX 侧多 lane deskew 对齐子模块。输入为入口 mux 后各 lane 原始
//           CPIF beat（1024 bit/lane，非拼好的字）。训练子阶段由 train_start_i
//           触发，按 golden §4.3.3.1 检测各 lane「连续 2 拍低 512 bit 全 1」
//           （与 s2p_mode 无关），任一 lane 首先检测到后启动 align_cnt（0~7）
//           对齐窗口，各 lane 在窗口内捕获各自到达时刻 cap_x，全部捕获后锁定
//           sel_lane_x = max_cap - cap_x（0~7）。
//           对齐语义：参考 lane（sel==0，即最慢 lane，最后到达者）数据 0T 直通，
//           较快 lane（sel 1..7）经 8 级 shift register + 8 选 1 mux 按
//           sel_lane_x 经 shift[sel-1] 做延迟补偿，吸收 lane 间最多 7 cycle
//           静态歪斜；valid_o 跟随最慢（sel==0）lane 而非一律 lane0。
//           align_cnt 计到 7 仍有 lane 未捕获 → train_fail_o（重训练）。
//           1 lane 配置或 bw_mode 使活跃 lane ≤ 1 时 bypass：不训练，
//           train_start_i 直接回 train_done_o，sel_lane*_o 保持 0。
//           注：link_rx 侧在单 lane 配置（LINK_LANE1/LINK_LANE3 均未定义）下
//           已不例化本模块、数据/valid 0T 直通，本模块 1-lane 分支仅保持自洽。
// 编码约束（本模块强制）:
//   1. 仅时序信号使用 always 语句（always @(posedge mb_clk)，同步复位，
//      禁用异步复位）；本模块无状态机（train_busy 标志 + 计数器实现）；
//   2. 每个 always 块内 if 判断（不含 reset 判断）≤ 2 个，禁止 if 嵌套，
//      条件用 && 归并或预计算 wire；
//   3. 组合逻辑只允许 assign 赋值；
//   4. for 循环仅用于 generate 实例化（shift register 逐级展开例化）。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   <日期>  <修订人>  <变更概述>
//  2026-08-08  OpenHappy Team  初始版本（对齐 MAS v1.0）
//  2026-08-08  OpenHappy Team  去状态机：dskew_fsm 删除，改 train_busy 标志 +
//                          align_cnt / cap_x 寄存器组（接收检测 → 决定对齐，
//                          无状态机控制）；shift register 仅在非训练期移位
//  2026-08-09  OpenHappy Team  deskew done 语义修正：对齐成功或 align_cnt 计满 7
//                          均拉高 train_done_o（状态机只接 done）；对齐失败
//                          仅经 train_fail_o 上报中断（F-05），不阻塞退出
//  2026-09-03  OpenHappy Team  deskew 对齐语义修正：参考 lane（sel==0，最慢者）数据
//                          0T 直通，较快 lane 经 shift[sel-1] 延迟补偿（取代原
//                          恒 shift0[sel] 引入的 +1T 数据延迟）；valid_o 改跟随
//                          最慢（sel==0）lane 而非一律 lane0。link_rx 单 lane
//                          配置下已不例化本模块（0T 直通），1-lane 分支仅保持自洽。
//  2026-09-07  OpenHappy Team  检测统一为「连续 2 拍低 512 bit 全 1」（删除 laneX_all1_full
//                          及 csr_s2p_mode_i 三元分支，与 s2p 无关）；输入改为入口 mux
//                          后各 lane 原始 CPIF beat，接口对齐 golden §4.3.3.1。
//                          csr_s2p_mode_i 端口保留（下游拼装位宽用）不再参与检测。
// =============================================================================

`timescale 1ns/1ps

`include "link_layer_defines.vh"

module lane_dskew (
    // 时钟与复位
    input  wire                          mb_clk,         // 2 GHz 数据通路主时钟
    input  wire                          mb_rst_n,       // 同步复位，低有效

    // 训练控制（link_rx_lcmsm → lane_dskew）
    input  wire                          train_start_i,  // 训练启动脉冲 1 cycle

    // CSR 配置（apb_csr 解码具名直送）
    input  wire [1:0]                    csr_bw_mode_i,      // BW_CFG_REG.bw_mode：00=全带宽/01=1/2/10=1/4
    input  wire                          csr_s2p_mode_i,     // PHY_MODE.s2p_mode：保留，下游拼装位宽用（本模块检测不再用）

    // per-lane 输入（cpif_rx_assemble → lane_dskew）
    input  wire [1023:0]   lane0_data_i,   // lane0 数据
    input  wire                          lane0_valid_i,  // lane0 有效，valid-only
`ifdef LINK_LANE1
    input  wire [1023:0]   lane1_data_i,   // lane1 数据
    input  wire                          lane1_valid_i,  // lane1 有效
`endif
`ifdef LINK_LANE3
    input  wire [1023:0]   lane2_data_i,   // lane2 数据（未来扩展）
    input  wire                          lane2_valid_i,  // lane2 有效
    input  wire [1023:0]   lane3_data_i,   // lane3 数据（未来扩展）
    input  wire                          lane3_valid_i,  // lane3 有效
`endif

    // 对齐输出（lane_dskew → rx_seq_check）
    output wire [(1024*`LANE_COUNT)-1:0] data_o,         // deskew 对齐后输出 {laneN,…,lane1,lane0}
    output wire                          valid_o,        // 对齐后有效；所有 lane 已对齐，跟随 lane0

    // 训练状态回报（→ link_rx_lcmsm / apb_csr）
    output wire                          train_done_o,   // 训练完成脉冲
    output wire                          train_fail_o    // 训练失败脉冲（align_cnt=7 仍有 lane 未捕获）
);

    // =========================================================================
    // 参数定义（MAS §4.2 / lane_dskew_dskew_fsm.json）
    // =========================================================================
    localparam LANE_W = 1024;   // per-lane 物理位宽固定 1024 bit

    // =========================================================================
    // 内部信号声明（MAS §5.1）
    // 所有 wire / reg 统一定义在 RTL 最前面，wire 与 assign 分离
    // =========================================================================

    // 训练进行标志（无状态机：train_start_i 置位，完成/失败清零）
    reg                train_busy;      // 训练进行中（检测 + 捕获窗口开放）

    // 训练捕获
    reg  [2:0]         align_cnt;       // 对齐窗口计数器（0~7）
    reg                align_start;     // 任一 lane 检测到全 1 后启动
    reg  [2:0]         cap_0;           // lane0 捕获时刻
    reg                cap_done_0;      // lane0 捕获完成
    reg  [1:0]         all1_pipe_0;     // lane0 连续 2 拍低 512 bit 全 1 检测移位
`ifdef LINK_LANE1
    reg  [2:0]         cap_1;
    reg                cap_done_1;
    reg  [1:0]         all1_pipe_1;
`endif
`ifdef LINK_LANE3
    reg  [2:0]         cap_2;
    reg                cap_done_2;
    reg  [1:0]         all1_pipe_2;
    reg  [2:0]         cap_3;
    reg                cap_done_3;
    reg  [1:0]         all1_pipe_3;
`endif

    // sel 锁定寄存器（MAS §5.4 [7]：训练完成拍锁存，train_start 清 0）
    reg  [2:0]         sel_lane0_q;
`ifdef LINK_LANE1
    reg  [2:0]         sel_lane1_q;
`endif
`ifdef LINK_LANE3
    reg  [2:0]         sel_lane2_q;
    reg  [2:0]         sel_lane3_q;
`endif

    // 训练状态输出寄存器（1 cycle 脉冲）
    reg                train_done_q;
    reg                train_fail_q;

    // 每 lane 8 级 shift register（depth=8，width=1024 bit），逐级展开例化
    reg  [LANE_W-1:0]  shift0 [0:7];
`ifdef LINK_LANE1
    reg  [LANE_W-1:0]  shift1 [0:7];
`endif
`ifdef LINK_LANE3
    reg  [LANE_W-1:0]  shift2 [0:7];
    reg  [LANE_W-1:0]  shift3 [0:7];
`endif

    // 模式与检测组合
    wire               single_active;   // 1 lane 或 bw_mode 使活跃 lane ≤ 1 → bypass
    wire               lane0_all1_512;  // lane0 当拍低 512 bit 全 1
    wire               det_lane0;       // lane0 连续 2 拍低 512 全 1 检出
    wire               any_lane_det;    // 任一 lane 检出
    wire               all_lane_cap_done;// 所有 lane 捕获完成
`ifdef LINK_LANE1
    wire               lane1_all1_512;
    wire               det_lane1;
`endif
`ifdef LINK_LANE3
    wire               lane2_all1_512;
    wire               det_lane2;
    wire               lane3_all1_512;
    wire               det_lane3;
`endif

    // 训练控制条件预计算（无状态机，train_busy 标志 + 计数器）
    wire               train_entry;     // 进入训练（train_start_i 且非 bypass 且空闲）
    wire               train_exit;      // 退出训练（完成或窗口计满）：cap/cap_done/align_cnt 清 0
    wire               cnt_start;       // align_cnt 启动（首 lane 检出同拍）
    wire               cnt_inc;         // align_cnt 计数推进
    wire               train_fail_cond; // 训练失败条件（窗口计满仍有 lane 未捕获）
    wire               train_complete;  // 全部捕获完成
    wire               train_done_set;  // train_done_o 置位条件
    wire               train_fail_set;  // train_fail_o 置位条件

    // 捕获使能与捕获值
    wire               cap_en_0;        // lane0 捕获使能
    wire [2:0]         cap_val;         // 捕获值：触发 lane 在 align_start 同拍记 0
`ifdef LINK_LANE1
    wire               cap_en_1;
`endif
`ifdef LINK_LANE3
    wire               cap_en_2;
    wire               cap_en_3;
`endif

    // max_cap 组合（MAS §5.1.1）
    wire [2:0]         max_cap;
`ifdef LINK_LANE3
    wire [2:0]         max_cap_01;
    wire [2:0]         max_cap_23;
`endif

    // 8 选 1 mux 输出
    wire [LANE_W-1:0]  lane0_aligned;
`ifdef LINK_LANE1
    wire [LANE_W-1:0]  lane1_aligned;
`endif
`ifdef LINK_LANE3
    wire [LANE_W-1:0]  lane2_aligned;
    wire [LANE_W-1:0]  lane3_aligned;
`endif

    // data_o 各 lane 段（single_active 时高 lane 段填 0，仅 lane0 直通）
`ifdef LINK_LANE1
    wire [LANE_W-1:0]  lane1_seg;
`endif
`ifdef LINK_LANE3
    wire [LANE_W-1:0]  lane2_seg;
    wire [LANE_W-1:0]  lane3_seg;
`endif

    // =========================================================================
    // 组合逻辑 assign（MAS §5.1）
    // =========================================================================

    // single_active（lane_dskew_dskew_fsm.json transitions 注释）：
    // 1 lane 配置 / bw_mode=1/4 / bw_mode=1/2 且 2 lane 配置 → 活跃 lane ≤ 1
    // lane 数为编译期常量，用 `ifdef presence 宏（LINK_LANE3→4 lane / LINK_LANE1→2 lane
    //   / 均不定义→1 lane）在编译前选代码段，无参数 if 条件
    `ifdef LINK_LANE3
        assign single_active = (csr_bw_mode_i == 2'b10);
    `elsif LINK_LANE1
        assign single_active = (csr_bw_mode_i == 2'b10) || (csr_bw_mode_i == 2'b01);
    `else
        assign single_active = 1'b1;
    `endif

    // 训练检测（golden §4.3.3.1）：连续 2 拍低 512 bit 全 1（与 s2p 无关，对原始 beat 检测）
    assign lane0_all1_512  = &lane0_data_i[511:0];
    assign det_lane0       = all1_pipe_0[1] & all1_pipe_0[0];
`ifdef LINK_LANE1
    assign lane1_all1_512  = &lane1_data_i[511:0];
    assign det_lane1       = all1_pipe_1[1] & all1_pipe_1[0];
`endif
`ifdef LINK_LANE3
    assign lane2_all1_512  = &lane2_data_i[511:0];
    assign det_lane2       = all1_pipe_2[1] & all1_pipe_2[0];
    assign lane3_all1_512  = &lane3_data_i[511:0];
    assign det_lane3       = all1_pipe_3[1] & all1_pipe_3[0];
`endif

    // 任一 lane 检出 / 所有 lane 捕获完成（按编译期 lane 数展开）
`ifdef LINK_LANE3
    assign any_lane_det      = det_lane0 | det_lane1 | det_lane2 | det_lane3;
    assign all_lane_cap_done = cap_done_0 & cap_done_1 & cap_done_2 & cap_done_3;
`elsif LINK_LANE1
    assign any_lane_det      = det_lane0 | det_lane1;
    assign all_lane_cap_done = cap_done_0 & cap_done_1;
`else
    assign any_lane_det      = det_lane0;
    assign all_lane_cap_done = cap_done_0;
`endif

    // 训练控制条件预计算
    assign train_entry     = train_start_i && !train_busy && !single_active;
    // 退出训练：全部捕获（对齐成功）或 align_cnt 计满 7（窗口结束）
    assign train_exit      = train_busy && (all_lane_cap_done ||
                             (align_start && (align_cnt == 3'b111)));
    assign cnt_start       = train_busy && !align_start && any_lane_det;
    assign cnt_inc         = train_busy && align_start &&
                             !all_lane_cap_done && (align_cnt != 3'b111);
    assign train_fail_cond = train_busy && align_start && (align_cnt == 3'b111) &&
                             !all_lane_cap_done;
    assign train_complete  = train_busy && all_lane_cap_done;
    // deskew done：全部捕获（对齐成功）或 align_cnt 计满 7（窗口结束）均拉高；
    // 失败不阻塞退出，仅经 train_fail_o 上报中断（L1 §3.2 / §4.9 F-05）
    assign train_done_set  = (train_start_i && single_active) || train_complete ||
                             (train_busy && align_start && (align_cnt == 3'b111));
    assign train_fail_set  = train_fail_cond;

    // 捕获：触发 lane 在 align_start 同拍即捕获 cap_x = 0，其后检出的记 align_cnt
    assign cap_en_0 = train_busy && (align_start || any_lane_det) &&
                      !cap_done_0 && det_lane0;
    assign cap_val  = align_start ? align_cnt : 3'd0;
`ifdef LINK_LANE1
    assign cap_en_1 = train_busy && (align_start || any_lane_det) &&
                      !cap_done_1 && det_lane1;
`endif
`ifdef LINK_LANE3
    assign cap_en_2 = train_busy && (align_start || any_lane_det) &&
                      !cap_done_2 && det_lane2;
    assign cap_en_3 = train_busy && (align_start || any_lane_det) &&
                      !cap_done_3 && det_lane3;
`endif

    // max_cap（MAS §5.1.1：2 lane 取较大者；4 lane 两级 max，未来扩展）
`ifdef LINK_LANE3
    assign max_cap_01 = (cap_0 > cap_1) ? cap_0 : cap_1;
    assign max_cap_23 = (cap_2 > cap_3) ? cap_2 : cap_3;
    assign max_cap    = (max_cap_01 > max_cap_23) ? max_cap_01 : max_cap_23;
`elsif LINK_LANE1
    assign max_cap    = (cap_0 > cap_1) ? cap_0 : cap_1;
`else
    assign max_cap    = cap_0;
`endif

    // lane 对齐 mux（MAS §5.1.2 修订）：参考 lane（sel==0，最慢者）数据 0T 直通，
    // 较快 lane（sel 1..7）经 shift[sel-1]（= sel cycle 延迟）做静态歪斜补偿；
    // shift0[0]..shift0[6] = 1T..7T，sel=0 取输入直通 = 0T
    assign lane0_aligned = (sel_lane0_q == 3'd0) ? lane0_data_i : shift0[sel_lane0_q - 3'd1];
`ifdef LINK_LANE1
    assign lane1_aligned = (sel_lane1_q == 3'd0) ? lane1_data_i : shift1[sel_lane1_q - 3'd1];
`endif
`ifdef LINK_LANE3
    assign lane2_aligned = (sel_lane2_q == 3'd0) ? lane2_data_i : shift2[sel_lane2_q - 3'd1];
    assign lane3_aligned = (sel_lane3_q == 3'd0) ? lane3_data_i : shift3[sel_lane3_q - 3'd1];
`endif

    // single_active（bypass）时高 lane 段填 0，仅 lane0 段有效（MAS §5.1.2）
`ifdef LINK_LANE1
    assign lane1_seg = single_active ? {LANE_W{1'b0}} : lane1_aligned;
`endif
`ifdef LINK_LANE3
    assign lane2_seg = single_active ? {LANE_W{1'b0}} : lane2_aligned;
    assign lane3_seg = single_active ? {LANE_W{1'b0}} : lane3_aligned;
`endif

    // =========================================================================
    // 时序逻辑（同步复位，always @(posedge mb_clk)，禁用异步复位）
    // 每个 always 块内 if 判断（不含 reset 判断）≤ 2 个
    // =========================================================================

    // train_busy：train_entry 置位，退出训练（完成或窗口计满）清零
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            train_busy <= 1'b0;
        else if (train_exit)
            train_busy <= 1'b0;
        else if (train_entry)
            train_busy <= 1'b1;
    end

    // align_start：退出 LINK_TRAIN 清 0；训练中首 lane 检出（第一笔 Pattern）同拍置 1
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            align_start <= 1'b0;
        else if (train_exit)
            align_start <= 1'b0;
        else if (cnt_start)
            align_start <= 1'b1;
    end

    // align_cnt：退出 LINK_TRAIN 清 0；训练中收到第一笔 Pattern（align_start）后启动计数，计到 7 停止
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            align_cnt <= 3'd0;
        else if (train_exit || cnt_start)
            align_cnt <= 3'd0;
        else if (cnt_inc)
            align_cnt <= align_cnt + 3'd1;
    end

    // lane0 低 512 bit 全 1 检测移位（进 S_TRAIN 清 0 防陈旧检出）
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            all1_pipe_0 <= 2'b00;
        else if (train_exit)
            all1_pipe_0 <= 2'b00;
        else if (train_busy)
            all1_pipe_0 <= {all1_pipe_0[0], lane0_all1_512};
    end

    // lane0 捕获
    always @(posedge mb_clk) begin
        if (!mb_rst_n) begin
            cap_0      <= 3'd0;
            cap_done_0 <= 1'b0;
        end else if (train_exit) begin
            cap_0      <= 3'd0;
            cap_done_0 <= 1'b0;
        end else if (cap_en_0) begin
            cap_0      <= cap_val;
            cap_done_0 <= 1'b1;
        end
    end

`ifdef LINK_LANE1
    // lane1 低 512 bit 全 1 检测移位
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            all1_pipe_1 <= 2'b00;
        else if (train_exit)
            all1_pipe_1 <= 2'b00;
        else if (train_busy)
            all1_pipe_1 <= {all1_pipe_1[0], lane1_all1_512};
    end

    // lane1 捕获
    always @(posedge mb_clk) begin
        if (!mb_rst_n) begin
            cap_1      <= 3'd0;
            cap_done_1 <= 1'b0;
        end else if (train_exit) begin
            cap_1      <= 3'd0;
            cap_done_1 <= 1'b0;
        end else if (cap_en_1) begin
            cap_1      <= cap_val;
            cap_done_1 <= 1'b1;
        end
    end
`endif

`ifdef LINK_LANE3
    // lane2 低 512 bit 全 1 检测移位
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            all1_pipe_2 <= 2'b00;
        else if (train_exit)
            all1_pipe_2 <= 2'b00;
        else if (train_busy)
            all1_pipe_2 <= {all1_pipe_2[0], lane2_all1_512};
    end

    // lane2 捕获
    always @(posedge mb_clk) begin
        if (!mb_rst_n) begin
            cap_2      <= 3'd0;
            cap_done_2 <= 1'b0;
        end else if (train_exit) begin
            cap_2      <= 3'd0;
            cap_done_2 <= 1'b0;
        end else if (cap_en_2) begin
            cap_2      <= cap_val;
            cap_done_2 <= 1'b1;
        end
    end

    // lane3 低 512 bit 全 1 检测移位
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            all1_pipe_3 <= 2'b00;
        else if (train_exit)
            all1_pipe_3 <= 2'b00;
        else if (train_busy)
            all1_pipe_3 <= {all1_pipe_3[0], lane3_all1_512};
    end

    // lane3 捕获
    always @(posedge mb_clk) begin
        if (!mb_rst_n) begin
            cap_3      <= 3'd0;
            cap_done_3 <= 1'b0;
        end else if (train_exit) begin
            cap_3      <= 3'd0;
            cap_done_3 <= 1'b0;
        end else if (cap_en_3) begin
            cap_3      <= cap_val;
            cap_done_3 <= 1'b1;
        end
    end
`endif

    // sel 锁定（MAS §5.4 [7]：训练完成拍锁存 sel = max_cap - cap；
    // MAS §2.2：train_start 触发时清 0）
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            sel_lane0_q <= 3'd0;
        else if (train_start_i)
            sel_lane0_q <= 3'd0;
        else if (train_complete)
            sel_lane0_q <= max_cap - cap_0;
    end

`ifdef LINK_LANE1
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            sel_lane1_q <= 3'd0;
        else if (train_start_i)
            sel_lane1_q <= 3'd0;
        else if (train_complete)
            sel_lane1_q <= max_cap - cap_1;
    end
`endif

`ifdef LINK_LANE3
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            sel_lane2_q <= 3'd0;
        else if (train_start_i)
            sel_lane2_q <= 3'd0;
        else if (train_complete)
            sel_lane2_q <= max_cap - cap_2;
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            sel_lane3_q <= 3'd0;
        else if (train_start_i)
            sel_lane3_q <= 3'd0;
        else if (train_complete)
            sel_lane3_q <= max_cap - cap_3;
    end
`endif

    // train_done_o 脉冲寄存器（1 lane/bypass：train_start_i 后 1 拍回 1；
    // 训练完成：随 S_TRAIN → S_NORMAL 同拍拉高）
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            train_done_q <= 1'b0;
        else
            train_done_q <= train_done_set;
    end

    // train_fail_o 脉冲寄存器（S_TRAIN → S_FAIL 同拍拉高，S_FAIL 期间为 1）
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            train_fail_q <= 1'b0;
        else
            train_fail_q <= train_fail_set;
    end

    // =========================================================================
    // 每 lane 8 级 shift register（MAS §5.1.2，逐级展开，仅 S_NORMAL 移位）
    // =========================================================================
    always @(posedge mb_clk) begin
        if (!mb_rst_n) begin
            shift0[0] <= {LANE_W{1'b0}};
            shift0[1] <= {LANE_W{1'b0}};
            shift0[2] <= {LANE_W{1'b0}};
            shift0[3] <= {LANE_W{1'b0}};
            shift0[4] <= {LANE_W{1'b0}};
            shift0[5] <= {LANE_W{1'b0}};
            shift0[6] <= {LANE_W{1'b0}};
            shift0[7] <= {LANE_W{1'b0}};
        end else if (!train_busy) begin
            shift0[0] <= lane0_data_i;
            shift0[1] <= shift0[0];
            shift0[2] <= shift0[1];
            shift0[3] <= shift0[2];
            shift0[4] <= shift0[3];
            shift0[5] <= shift0[4];
            shift0[6] <= shift0[5];
            shift0[7] <= shift0[6];
        end
    end

`ifdef LINK_LANE1
    always @(posedge mb_clk) begin
        if (!mb_rst_n) begin
            shift1[0] <= {LANE_W{1'b0}};
            shift1[1] <= {LANE_W{1'b0}};
            shift1[2] <= {LANE_W{1'b0}};
            shift1[3] <= {LANE_W{1'b0}};
            shift1[4] <= {LANE_W{1'b0}};
            shift1[5] <= {LANE_W{1'b0}};
            shift1[6] <= {LANE_W{1'b0}};
            shift1[7] <= {LANE_W{1'b0}};
        end else if (!train_busy) begin
            shift1[0] <= lane1_data_i;
            shift1[1] <= shift1[0];
            shift1[2] <= shift1[1];
            shift1[3] <= shift1[2];
            shift1[4] <= shift1[3];
            shift1[5] <= shift1[4];
            shift1[6] <= shift1[5];
            shift1[7] <= shift1[6];
        end
    end
`endif

`ifdef LINK_LANE3
    always @(posedge mb_clk) begin
        if (!mb_rst_n) begin
            shift2[0] <= {LANE_W{1'b0}};
            shift2[1] <= {LANE_W{1'b0}};
            shift2[2] <= {LANE_W{1'b0}};
            shift2[3] <= {LANE_W{1'b0}};
            shift2[4] <= {LANE_W{1'b0}};
            shift2[5] <= {LANE_W{1'b0}};
            shift2[6] <= {LANE_W{1'b0}};
            shift2[7] <= {LANE_W{1'b0}};
        end else if (!train_busy) begin
            shift2[0] <= lane2_data_i;
            shift2[1] <= shift2[0];
            shift2[2] <= shift2[1];
            shift2[3] <= shift2[2];
            shift2[4] <= shift2[3];
            shift2[5] <= shift2[4];
            shift2[6] <= shift2[5];
            shift2[7] <= shift2[6];
        end
    end

    always @(posedge mb_clk) begin
        if (!mb_rst_n) begin
            shift3[0] <= {LANE_W{1'b0}};
            shift3[1] <= {LANE_W{1'b0}};
            shift3[2] <= {LANE_W{1'b0}};
            shift3[3] <= {LANE_W{1'b0}};
            shift3[4] <= {LANE_W{1'b0}};
            shift3[5] <= {LANE_W{1'b0}};
            shift3[6] <= {LANE_W{1'b0}};
            shift3[7] <= {LANE_W{1'b0}};
        end else if (!train_busy) begin
            shift3[0] <= lane3_data_i;
            shift3[1] <= shift3[0];
            shift3[2] <= shift3[1];
            shift3[3] <= shift3[2];
            shift3[4] <= shift3[3];
            shift3[5] <= shift3[4];
            shift3[6] <= shift3[5];
            shift3[7] <= shift3[6];
        end
    end
`endif

    // =========================================================================
    // 输出赋值（MAS §5.1.2 / §2.1）
    // =========================================================================
`ifdef LINK_LANE3
    assign data_o = {lane3_seg, lane2_seg, lane1_seg, lane0_aligned};
`elsif LINK_LANE1
    assign data_o = {lane1_seg, lane0_aligned};
`else
    assign data_o = lane0_aligned;
`endif

    // valid 跟随最慢 lane（sel==0 的 lane，即最后到达者），而非一律 lane0：
    // 参考 lane 数据 0T 直通，其 valid 与本模块输出 valid 同时序对齐
    // （MAS §5.1.2 修订）
`ifdef LINK_LANE3
    assign valid_o = !train_busy & ((sel_lane0_q == 3'd0) ? lane0_valid_i
                  : (sel_lane1_q == 3'd0) ? lane1_valid_i
                  : (sel_lane2_q == 3'd0) ? lane2_valid_i
                  : lane3_valid_i);
`elsif LINK_LANE1
    assign valid_o = !train_busy & ((sel_lane0_q == 3'd0) ? lane0_valid_i : lane1_valid_i);
`else
    // 1 lane 配置：单 lane 直通（link_rx 侧已不例化本模块，此处保持自洽）
    assign valid_o = !train_busy & lane0_valid_i;
`endif

    assign train_done_o = train_done_q;
    assign train_fail_o = train_fail_q;

endmodule
