// =============================================================================
// module name：rx_prbs_descrambler
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/rx_chain/submods/rx_data_path/mas/rx_prbs_descrambler_mas.md
// =============================================================================
// 功能概述: RX 数据通路 stage 4：16 logical IO × 64 bit PRBS23 解扰（例化 16 个 prbs_chk），
//   TDM 共享解扰 / data check，flip_sel 分频推进含相位锁定，输出寄存型 1 cycle
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-09-13  cdc-resolver  CDC 违例修复（MAS v1.8）：seed_load_i/clear 改 toggle 载体，入口 2FF 同步 + 同步输出打拍（t2^t2_d）边沿恢复 phy_rx_clk 1 拍脉冲；prbs_seed_rx 总线经 sync_cell_norst 2flop 入 phy_rx_clk
//  2026-08-06  OpenHappy Team  初始版本：基于 rx_prbs_descrambler MAS v1.5 生成
//  2026-08-07  OpenHappy Team  flip_sel 语义修正（MAS v1.6）：flip_en 改门控
//                       prbs_chk.prbs_en（LFSR 分频推进，每个 PRBS 字保持 N 拍），
//                       data_o 输出寄存器全速推进，不再丢弃中间拍源数据
//  2026-08-07  OpenHappy Team  RX flip_cnt 相位锁定（MAS v1.7）：分频档下 TX/RX
//                       flip_cnt 分属不同时钟域、相位任意，self-sync 期间检测
//                       data_i 字跳变沿复位 flip_cnt 完成相位锁定；flip_en
//                       改组合输出，落在每个字窗口最后一拍
// =============================================================================

`timescale 1ns/1ps

module rx_prbs_descrambler #(
    parameter IO_NUM       = 16,
    parameter IO_WIDTH     = 64,
    parameter STATE_WIDTH  = 23
) (
    input  wire                          phy_rx_clk,
    input  wire                          rx_rst_n,
    input  wire  [IO_NUM*IO_WIDTH-1:0]   data_i,
    output wire  [IO_NUM*IO_WIDTH-1:0]   data_o,
    output wire  [IO_NUM*IO_WIDTH-1:0]   prbs_seq_o,    // TDM 共享：DATA_CHK 态送 data_checker
    output wire  [IO_NUM-1:0]             aligned_o,       // 16 个 per-IO 对齐指示
    input  wire  [IO_NUM*STATE_WIDTH-1:0] prbs_seed_rx,
    input  wire  [3:0]                   flip_sel_rx,
    input  wire                          io_data_width_mode,
    input  wire                          scramble_en,      // ACTIVE=1 XOR 解扰 / DATA_CHK=0 bypass
    input  wire                          self_sync_en,     // prbs_chk self-sync 模式使能
    input  wire                          seed_load_i,      // debug 重新对齐脉冲
    input  wire                          clear             // state 清零脉冲（优先级最高）
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    // flip_sel 9 档分频阈值（与 TX 侧 tx_prbs_scrambler 完全对称）
    // 9 档编码：1, 1/2, 1/4, 1/8, 1/16, 1/32, 1/64, 1/128, 1/256
    // 保留编码 1001~1111 按全速处理（flip_en 恒 1）
    localparam [7:0] FLIP_FULL_SPEED = 8'd0;
    localparam [7:0] FLIP_DIV_2      = 8'd1;
    localparam [7:0] FLIP_DIV_4      = 8'd3;
    localparam [7:0] FLIP_DIV_8      = 8'd7;
    localparam [7:0] FLIP_DIV_16     = 8'd15;
    localparam [7:0] FLIP_DIV_32     = 8'd31;
    localparam [7:0] FLIP_DIV_64     = 8'd63;
    localparam [7:0] FLIP_DIV_128    = 8'd127;
    localparam [7:0] FLIP_DIV_256    = 8'd255;

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    // 翻转频率分频
    reg  [7:0]  flip_cnt;
    wire        flip_en;

    // 字跳变沿检测（分频档相位锁定用，lane 0 的 64 bit）
    reg  [IO_WIDTH-1:0] data_i_d;
    wire                word_chg;

    // LFSR 控制信号（per-IO 16 个）
    wire                              lfsr_prbs_en;
    wire [STATE_WIDTH-1:0]            lfsr_state [0:IO_NUM-1];
    wire [IO_WIDTH-1:0]               prbs_seq   [0:IO_NUM-1];   // 16 个 prbs_chk 输出

    // 数据通路组合输出（XOR / bypass mux 后）
    wire [IO_WIDTH-1:0]               data_o_next [0:IO_NUM-1];

    // 高 32 bit 输出寄存器门控时钟（CG cell，enable = io_data_width_mode，MAS §3.4）
    wire                              gated_clk_high32;

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // 翻转频率分频阈值选择（与 TX 侧对称的 lookup table）
    // --------------------------------------------------------------------------
    reg [7:0] flip_threshold;
    always @(*) begin
        case (flip_sel_rx)
            4'b0000: flip_threshold = FLIP_FULL_SPEED;
            4'b0001: flip_threshold = FLIP_DIV_2;
            4'b0010: flip_threshold = FLIP_DIV_4;
            4'b0011: flip_threshold = FLIP_DIV_8;
            4'b0100: flip_threshold = FLIP_DIV_16;
            4'b0101: flip_threshold = FLIP_DIV_32;
            4'b0110: flip_threshold = FLIP_DIV_64;
            4'b0111: flip_threshold = FLIP_DIV_128;
            4'b1000: flip_threshold = FLIP_DIV_256;
            default: flip_threshold = FLIP_FULL_SPEED;   // 保留编码按全速
        endcase
    end

    // --------------------------------------------------------------------------
    // 字跳变沿检测 + flip_en 组合输出（分频档相位锁定，MAS §3.3）
    //   word_chg：self-sync 期间 + 分频档，lane 0 的 64 bit 发生字跳变
    //     （TX 侧 flip_en 广播 16 IO，全 IO 同拍跳变，单 IO 观测即可；
    //       32:1 模式低 32 bit 仍逐字跳变）
    //   flip_en：组合输出，flip_cnt 计到阈值当拍拉高，落在每个字窗口最后一拍；
    //     全速档 threshold=0、cnt 保持 0，flip_en 恒 1
    // --------------------------------------------------------------------------
    assign word_chg = self_sync_en && (flip_sel_rx != 4'b0000) &&
                      (data_i[IO_WIDTH-1:0] != data_i_d);
    assign flip_en  = (flip_cnt == flip_threshold);

    // --------------------------------------------------------------------------
    // LFSR 控制信号：prbs_en = flip_en（分频门控推进，全速档 flip_en 恒 1）
    //   flip_en=0 期间 state 保持，prbs_seq_o 组合直驱 state 随之保持，
    //   每个 PRBS 字保持 N 拍，序列连续不跳变
    // seed_load_i / clear 由 apb_csr_top 触发（debug 场景）或 self-sync 锁存
    // --------------------------------------------------------------------------
    assign lfsr_prbs_en = flip_en;     // flip_en 分频门控 LFSR 推进（MAS §3.3）

    // --------------------------------------------------------------------------
    // 解扰 / bypass mux（per-IO）
    //   - scramble_en=1（ACTIVE）：data_o_next = data_i ^ prbs_seq（XOR 解扰）
    //   - scramble_en=0（DATA_CHK）：data_o_next = data_i（bypass 直通，prbs_seq_o 送 data_checker）
    // --------------------------------------------------------------------------
    genvar i;
    generate
        for (i = 0; i < IO_NUM; i = i + 1) begin : gen_descrambler_mux
            assign data_o_next[i] = scramble_en ? (data_i[i*IO_WIDTH +: IO_WIDTH] ^ prbs_seq[i])
                                                : data_i[i*IO_WIDTH +: IO_WIDTH];
        end
    endgenerate

    // ==========================================================================
    // CDC 同步（cdc-resolver 2026-09-13）：seed_load_i / clear 来自 apb_csr_top
    // pclk 域写脉冲（*_wp → toggle），prbs_seed_rx 为 pclk 域 CSR 配置数据。
    //   - seed_load_i / clear：toggle → phy_rx_clk 2FF 同步 + 同步输出打拍边沿检测
    //     （seed_load_t2^t2_d 恢复 1 拍脉冲；t1 仅驱动 t2 构成纯 2FF 同步链，
    //     边沿检测在同步输出侧，避免 t1 扇出被 SpyGlass 判为未同步）
    //   - prbs_seed_rx：sync_cell_norst 2 拍打拍到 phy_rx_clk（配置值，加载窗口稳定）
    // 目的：消除 pclk→phy_rx_clk 未同步穿越（Ac_unsync01/02）。
    // ==========================================================================
    reg        seed_load_t1, seed_load_t2, seed_load_t2_d;
    reg        clear_t1, clear_t2, clear_t2_d;
    always @(posedge phy_rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            seed_load_t1   <= 1'b0;
            seed_load_t2   <= 1'b0;
            seed_load_t2_d <= 1'b0;
            clear_t1       <= 1'b0;
            clear_t2       <= 1'b0;
            clear_t2_d     <= 1'b0;
        end
        else begin
            seed_load_t1   <= seed_load_i;
            seed_load_t2   <= seed_load_t1;
            seed_load_t2_d <= seed_load_t2;
            clear_t1       <= clear;
            clear_t2       <= clear_t1;
            clear_t2_d     <= clear_t2;
        end
    end
    wire        seed_load_i_ckd = seed_load_t2 ^ seed_load_t2_d;  // phy_rx_clk 域 1 拍脉冲
    wire        clear_ckd       = clear_t2     ^ clear_t2_d;
    wire [IO_NUM*STATE_WIDTH-1:0] seed_rx_ckd;                  // seed 总线同步到 phy_rx_clk
    sync_cell_norst #(.width(IO_NUM*STATE_WIDTH)) u_sync_seed_rx (
        .syncout(seed_rx_ckd),
        .in     (prbs_seed_rx),
        .clk    (phy_rx_clk)
    );

    // ==========================================================================
    // 子模块实例化
    // ==========================================================================
    // 16 个 prbs_chk 实例（per-IO 独立，RX 侧检查 + 对齐 IP）
    // prbs_chk 提供 T^64/T^32 并行推进 + 64 bit 并行输出原语 + self-sync 对齐流程
    // prbs_seq_o 组合直驱（同 cycle 跟随 state），不门控，使用方按 scramble_en 决定采样时机
    generate
        for (i = 0; i < IO_NUM; i = i + 1) begin : gen_prbs_chk_inst
            prbs_chk u_prbs_chk (
                .clk               (phy_rx_clk),
                .rst_n             (rx_rst_n),
                .data_i            (data_i[i*IO_WIDTH +: IO_WIDTH]),
                .seed_i            (seed_rx_ckd[i*STATE_WIDTH +: STATE_WIDTH]),
                .seed_load_i       (seed_load_i_ckd),
                .prbs_en           (lfsr_prbs_en),
                .self_sync_en      (self_sync_en),
                .clear             (clear_ckd),
                .io_data_width_mode(io_data_width_mode),
                .prbs_seq_o        (prbs_seq[i]),
                .aligned_o         (aligned_o[i]),
                .state_o           (lfsr_state[i])
            );
        end
    endgenerate

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // 翻转频率分频计数器 + 字跳变沿检测寄存器（MAS §3.3 相位锁定）
    //   - 全速档（flip_sel_rx=0）：cnt 保持 0，flip_en 恒 1（threshold=0）
    //   - 分频档：word_chg 复位 cnt<=1，使 flip_en 落在每个字窗口最后一拍，
    //     对齐 TX 推进相位；无 word_chg 时自由计数
    //   - data_i_d 寄存 lane 0 的 64 bit，供字跳变沿检测
    // --------------------------------------------------------------------------
    always @(posedge phy_rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            flip_cnt <= 8'd0;
            data_i_d <= {IO_WIDTH{1'b0}};
        end
        else begin
            data_i_d <= data_i[IO_WIDTH-1:0];
            if (word_chg) begin
                // 相位锁定：字跳变沿当拍 cnt<=1，下一 fire 落在本字窗口最后一拍
                flip_cnt <= 8'd1;
            end
            else if (flip_en) begin
                flip_cnt <= 8'd0;
            end
            else begin
                flip_cnt <= flip_cnt + 8'd1;
            end
        end
    end

    // --------------------------------------------------------------------------
    // CG cell：高 32 bit 输出寄存器门控时钟（MAS §3.4）
    //   enable = io_data_width_mode；32:1 模式下 CG 关闭，高 32 bit 寄存器不翻转
    // --------------------------------------------------------------------------
    generic_icg_pos u_clk_gate_high32 (
        .CK (phy_rx_clk),
        .E  (io_data_width_mode),
        .Q  (gated_clk_high32)
    );

    // --------------------------------------------------------------------------
    // data_o 输出寄存器（1024 bit = 16 IO × 64 bit），全速推进（无 clock_enable），
    // 高/低 32 bit 分属不同时钟域，拆为两个独立 reg（避免同一变量多时钟域多驱动）：
    //   - 低 32 bit/IO（512 bit）：phy_rx_clk 全速
    //   - 高 32 bit/IO（512 bit）：gated_clk_high32（CG cell 门控）
    //     32:1 模式（io_data_width_mode=0）下高 32 bit 寄存器时钟关闭，保持复位值 0
    // --------------------------------------------------------------------------
    reg [IO_NUM*32-1:0] data_o_lo_reg;
    reg [IO_NUM*32-1:0] data_o_hi_reg;

    // 低 32 bit/IO：phy_rx_clk 直驱，全速推进
    always @(posedge phy_rx_clk or negedge rx_rst_n) begin
        integer k;
        if (!rx_rst_n) begin
            data_o_lo_reg <= {(IO_NUM*32){1'b0}};
        end
        else begin
            for (k = 0; k < IO_NUM; k = k + 1) begin
                data_o_lo_reg[k*32 +: 32] <= data_o_next[k][31:0];
            end
        end
    end

    // 高 32 bit/IO：CG cell 门控时钟，全速推进
    always @(posedge gated_clk_high32 or negedge rx_rst_n) begin
        integer k;
        if (!rx_rst_n) begin
            data_o_hi_reg <= {(IO_NUM*32){1'b0}};
        end
        else begin
            for (k = 0; k < IO_NUM; k = k + 1) begin
                data_o_hi_reg[k*32 +: 32] <= data_o_next[k][63:32];
            end
        end
    end

    // 输出驱动：{hi32, lo32} 拼回 per-IO 64 bit
    genvar j;
    generate
        for (j = 0; j < IO_NUM; j = j + 1) begin : gen_output
            assign data_o[j*IO_WIDTH +: IO_WIDTH] =
                {data_o_hi_reg[j*32 +: 32], data_o_lo_reg[j*32 +: 32]};
        end
    endgenerate

    // prbs_seq_o 组合直驱（TDM 共享不门控）
    // 注：prbs_seq 来自 prbs_chk.prbs_seq_o，已经是同 cycle 跟随 state 的组合输出
    //     使用方（data_checker）按 scramble_en 决定是否采样
    generate
        for (j = 0; j < IO_NUM; j = j + 1) begin : gen_prbs_seq_o
            assign prbs_seq_o[j*IO_WIDTH +: IO_WIDTH] = prbs_seq[j];
        end
    endgenerate

endmodule
