// =============================================================================
// module name：tx_prbs_scrambler
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/lane_top/submods/tx_chain/submods/tx_data_path/mas/tx_prbs_scrambler_mas.md
// =============================================================================
// 功能概述: TX 数据通路 stage 2：16 logical IO × 64 bit PRBS23 加扰（例化 16 个 prbs_gen），
//   含训练 pattern mux / scramble_en bypass / flip_sel 分频推进，输出寄存型 1 cycle
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-09-13  cdc-resolver  CDC 违例修复（MAS v1.15）：seed_load_i/clear 改 toggle 载体，入口 2FF 同步 + 同步输出打拍（t2^t2_d）边沿恢复 mb_clk 1 拍脉冲；prbs_seed_tx 总线经 sync_cell_norst 2flop 入 mb_clk
//  2026-08-06  OpenHappy Team  初始版本：基于 tx_prbs_scrambler MAS v1.11 生成
//  2026-08-07  OpenHappy Team  训练 pattern 接口重构（MAS v1.12）：train_pattern_sel[2:0]
//                       改 5 条 one-hot req + 5 条 per-link CSR pattern 端口 +
//                       data_check_pattern_sel；删占位 pattern 常量
//  2026-08-07  OpenHappy Team  TX 加扰使能补全（MAS v1.13）：新增 scramble_en_tx 端口，
//                       =0 时正常数据态 bypass 加扰直发 data_i（与 RX 侧对称）
//  2026-08-07  OpenHappy Team  flip_sel 语义修正（MAS v1.14）：flip_en 改门控
//                       prbs_gen.prbs_en（LFSR 分频推进，每个 PRBS 字保持 N 拍），
//                       data_o 输出寄存器全速推进，不再丢弃中间拍源数据
// =============================================================================

`timescale 1ns/1ps

module tx_prbs_scrambler #(
    parameter IO_NUM       = 16,
    parameter IO_WIDTH     = 64,
    parameter STATE_WIDTH  = 23
) (
    input  wire                          mb_clk,
    input  wire                          mb_rst_n,
    input  wire  [IO_NUM*IO_WIDTH-1:0]   data_i,
    output wire  [IO_NUM*IO_WIDTH-1:0]   data_o,
    // 训练 pattern 切换请求（tx_training_fsm one-hot level，mb_clk 同域）
    input  wire                          vref_req,        // Vref 训练请求
    input  wire                          pi_req,          // PI 校准训练请求
    input  wire                          dcdl_req,        // DCDL deskew 训练请求
    input  wire                          reorder_req,     // reorder 训练请求
    input  wire                          data_check_req,  // data check 训练请求
    // 训练 pattern 数值（per-link CSR 可配，64 bit 广播 16 IO 重复发送）
    input  wire  [IO_WIDTH-1:0]          vref_pattern_tx,
    input  wire  [IO_WIDTH-1:0]          pi_pattern_tx,
    input  wire  [IO_WIDTH-1:0]          dcdl_pattern_tx,
    input  wire  [IO_WIDTH-1:0]          reorder_pattern_tx,
    input  wire  [IO_WIDTH-1:0]          datacheck_pattern_tx,
    // data check pattern 选择：0 = PRBS23 序列 / 1 = datacheck_pattern_tx
    input  wire                          data_check_pattern_sel,
    // TX 加扰使能：1 = 正常加扰 / 0 = bypass 直发 data_i 不加扰（复位默认 1，仅影响正常数据态）
    input  wire                          scramble_en_tx,
    input  wire  [IO_NUM*STATE_WIDTH-1:0] prbs_seed_tx,
    input  wire  [3:0]                   flip_sel_tx,
    input  wire                          io_data_width_mode,
    // MAS §6: prbs_gen.clear / seed_load_i 由本模块按 debug / 重新配置场景驱动
    // （软件经 CSR 触发，归 apb_csr_top 解码后具名信号直送）
    input  wire                          seed_load_i,   // debug 重新对齐脉冲（高有效）
    input  wire                          clear           // state 清零脉冲（高有效，优先级最高）
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    // flip_sel 9 档分频阈值（达到阈值时拉高 flip_en 1 拍）
    // 9 档编码：1, 1/2, 1/4, 1/8, 1/16, 1/32, 1/64, 1/128, 1/256
    // 保留编码 1001~1111 按全速处理（flip_en 恒 1）
    // 阈值表（计到阈值-1 后下一拍拉高，等效分频）：
    //   flip_sel=0000 → 全速（flip_en=1）
    //   flip_sel=0001 → 1/2  （阈值 2，cnt 计到 1 后下一拍 flip_en=1）
    //   flip_sel=0010 → 1/4  （阈值 4）
    //   ...
    //   flip_sel=1000 → 1/256（阈值 256）
    // 用 8 bit 计数器即可覆盖 256
    localparam [7:0] FLIP_FULL_SPEED = 8'd0;   // 全速（立即触发）
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
    reg  [7:0] flip_cnt;
    reg        flip_en;

    // LFSR 控制信号（per-IO 16 个）
    wire                              lfsr_prbs_en;
    wire [STATE_WIDTH-1:0]            lfsr_state [0:IO_NUM-1];
    wire [IO_WIDTH-1:0]               prbs_seq   [0:IO_NUM-1];   // 16 个 prbs_gen 输出

    // 数据通路组合输出（pattern mux 后）
    wire [IO_WIDTH-1:0]               data_o_next [0:IO_NUM-1];

    // 高 32 bit 输出寄存器门控时钟（CG cell，enable = io_data_width_mode，MAS §3.5）
    wire                              gated_clk_high32;

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // 翻转频率分频阈值选择（lookup table）
    // --------------------------------------------------------------------------
    reg [7:0] flip_threshold;
    always @(*) begin
        case (flip_sel_tx)
            4'b0000: flip_threshold = FLIP_FULL_SPEED;  // 全速
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
    // LFSR 控制信号：prbs_en = flip_en（分频门控推进，全速档 flip_en 恒 1）
    //   flip_en=0 期间 state 保持，prbs_seq_o 组合直驱 state 随之保持，
    //   每个 PRBS 字保持 N 拍，序列连续不跳变
    // MAS §6: seed_load_i / clear 由使用方按 debug / 重新配置场景驱动
    //   （软件经 apb_csr_top CSR 触发，具名信号直送本模块端口）
    //   - seed_load_i=1：将 prbs_seed_tx[i] 加载到 prbs_gen.state（debug 重新对齐）
    //   - clear=1：state 清零（优先级最高，debug 场景）
    //   优先级归 prbs_gen 内部：clear > seed_load_i > prbs_en > 推进
    // --------------------------------------------------------------------------
    assign lfsr_prbs_en = flip_en;     // flip_en 分频门控 LFSR 推进（MAS §3.4）

    // --------------------------------------------------------------------------
    // 输出 mux：5 条 one-hot req 优先级译码（per-IO，CSR pattern 广播 16 IO）
    //   - one-hot 由 tx_training_fsm 状态唯一性保证；多 req 同拉按固定优先级兜底：
    //     vref > pi > dcdl > reorder > data_check > 正常加扰
    //   - data_check_req：data_check_pattern_sel=0 输出 prbs_seq 直发（不加扰）/
    //     =1 输出 datacheck_pattern_tx
    //   - 全 req = 0（含 offset/EQ 步骤）：正常加扰 data_i ^ prbs_seq
    // --------------------------------------------------------------------------
    genvar i;
    generate
        for (i = 0; i < IO_NUM; i = i + 1) begin : gen_pattern_mux
            reg [IO_WIDTH-1:0] pattern_out;
            always @(*) begin
                if (vref_req) begin
                    pattern_out = vref_pattern_tx;
                end
                else if (pi_req) begin
                    pattern_out = pi_pattern_tx;
                end
                else if (dcdl_req) begin
                    pattern_out = dcdl_pattern_tx;
                end
                else if (reorder_req) begin
                    pattern_out = reorder_pattern_tx;
                end
                else if (data_check_req) begin
                    if (data_check_pattern_sel) begin
                        pattern_out = datacheck_pattern_tx;
                    end
                    else begin
                        pattern_out = prbs_seq[i];
                    end
                end
                else begin
                    // 正常数据态（全 req = 0，含 offset/EQ 步骤）：
                    // scramble_en_tx=1 正常加扰 / =0 bypass 直发 data_i 不加扰
                    if (scramble_en_tx) begin
                        pattern_out = data_i[i*IO_WIDTH +: IO_WIDTH] ^ prbs_seq[i];
                    end
                    else begin
                        pattern_out = data_i[i*IO_WIDTH +: IO_WIDTH];
                    end
                end
            end
            assign data_o_next[i] = pattern_out;
        end
    endgenerate

    // ==========================================================================
    // CDC 同步（cdc-resolver 2026-09-13）：seed_load_i / clear 来自 apb_csr_top
    // pclk 域写脉冲（*_wp → toggle），prbs_seed_tx 为 pclk 域 CSR 配置数据。
    //   - seed_load_i / clear：toggle → mb_clk 2FF 同步 + 同步输出打拍边沿检测
    //     （seed_load_t2^t2_d 恢复 1 拍脉冲；t1 仅驱动 t2 构成纯 2FF 同步链，
    //     边沿检测在同步输出侧，避免 t1 扇出被 SpyGlass 判为未同步）
    //   - prbs_seed_tx：sync_cell_norst 2 拍打拍到 mb_clk（配置值，加载窗口稳定）
    // 目的：消除 pclk→mb_clk 未同步穿越（Ac_unsync01/02）。
    // ==========================================================================
    reg        seed_load_t1, seed_load_t2, seed_load_t2_d;
    reg        clear_t1, clear_t2, clear_t2_d;
    always @(posedge mb_clk or negedge mb_rst_n) begin
        if (!mb_rst_n) begin
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
    wire        seed_load_i_ckd = seed_load_t2 ^ seed_load_t2_d;  // mb_clk 域 1 拍脉冲
    wire        clear_ckd       = clear_t2     ^ clear_t2_d;
    wire [IO_NUM*STATE_WIDTH-1:0] seed_tx_ckd;                  // seed 总线同步到 mb_clk
    sync_cell_norst #(.width(IO_NUM*STATE_WIDTH)) u_sync_seed_tx (
        .syncout(seed_tx_ckd),
        .in     (prbs_seed_tx),
        .clk    (mb_clk)
    );

    // ==========================================================================
    // 子模块实例化
    // ==========================================================================
    // 16 个 prbs_gen 实例（per-IO 独立，TX 侧生成 IP）
    // prbs_gen 提供 T^64/T^32 并行推进 + 64 bit 并行输出原语
    // seed_load_i / clear 端口直通本模块顶层输入（MAS §6：debug / 重新配置场景由软件经 CSR 触发）
    generate
        for (i = 0; i < IO_NUM; i = i + 1) begin : gen_prbs_gen_inst
            prbs_gen u_prbs_gen (
                .clk               (mb_clk),
                .rst_n             (mb_rst_n),
                .seed_i            (seed_tx_ckd[i*STATE_WIDTH +: STATE_WIDTH]),
                .seed_load_i       (seed_load_i_ckd),
                .prbs_en           (lfsr_prbs_en),
                .clear             (clear_ckd),
                .io_data_width_mode(io_data_width_mode),
                .prbs_seq_o        (prbs_seq[i]),
                .state_o           (lfsr_state[i])
            );
        end
    endgenerate

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // 翻转频率分频计数器（8 bit 计数器，全速时 cnt 保持 0，flip_en 恒 1）
    // --------------------------------------------------------------------------
    always @(posedge mb_clk or negedge mb_rst_n) begin
        if (!mb_rst_n) begin
            flip_cnt <= 8'd0;
            flip_en  <= 1'b1;   // 复位默认全速推进，保证首个 cycle 输出有效
        end
        else begin
            if (flip_sel_tx == 4'b0000) begin
                // 全速：flip_en 恒 1，cnt 保持 0
                flip_cnt <= 8'd0;
                flip_en  <= 1'b1;
            end
            else if (flip_cnt >= flip_threshold) begin
                flip_cnt <= 8'd0;
                flip_en  <= 1'b1;
            end
            else begin
                flip_cnt <= flip_cnt + 8'd1;
                flip_en  <= 1'b0;
            end
        end
    end

    // --------------------------------------------------------------------------
    // CG cell：高 32 bit 输出寄存器门控时钟（MAS §3.5）
    //   enable = io_data_width_mode；32:1 模式下 CG 关闭，高 32 bit 寄存器不翻转
    // --------------------------------------------------------------------------
    generic_icg_pos u_clk_gate_high32 (
        .CK (mb_clk),
        .E  (io_data_width_mode),
        .Q  (gated_clk_high32)
    );

    // --------------------------------------------------------------------------
    // data_o 输出寄存器（1024 bit = 16 IO × 64 bit），全速推进（无 clock_enable），
    // 高/低 32 bit 分属不同时钟域，拆为两个独立 reg（避免同一变量多时钟域多驱动）：
    //   - 低 32 bit/IO（512 bit）：mb_clk 全速
    //   - 高 32 bit/IO（512 bit）：gated_clk_high32（CG cell 门控）
    //     32:1 模式（io_data_width_mode=0）下高 32 bit 寄存器时钟关闭，保持复位值 0
    // --------------------------------------------------------------------------
    reg [IO_NUM*32-1:0] data_o_lo_reg;
    reg [IO_NUM*32-1:0] data_o_hi_reg;

    // 低 32 bit/IO：mb_clk 直驱，全速推进
    always @(posedge mb_clk or negedge mb_rst_n) begin
        integer k;
        if (!mb_rst_n) begin
            data_o_lo_reg <= {(IO_NUM*32){1'b0}};
        end
        else begin
            for (k = 0; k < IO_NUM; k = k + 1) begin
                data_o_lo_reg[k*32 +: 32] <= data_o_next[k][31:0];
            end
        end
    end

    // 高 32 bit/IO：CG cell 门控时钟，全速推进
    always @(posedge gated_clk_high32 or negedge mb_rst_n) begin
        integer k;
        if (!mb_rst_n) begin
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

endmodule
