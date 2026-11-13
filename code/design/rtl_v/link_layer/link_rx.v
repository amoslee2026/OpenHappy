// =============================================================================
// module name：link_rx
// author: OpenHappy Team
// MAS：spec/link_layer/submods/link_rx/link_rx_mas.md（父级壳）+ 4 个形态子模块 MAS
//      mas/link_rx_512_lane1_mas.md（形态①）/ mas/link_rx_1024_lane1_mas.md（形态②）/
//      mas/link_rx_1024_lane2_mas.md（形态③）/ mas/link_rx_compat_mas.md（形态④）
// 功能概述: RX 侧核心处理级**壳**。保留入口 mux（loopback_mux / udi_bypass_mux）、
//           rx_train_window、UDI RX 直通、credit_fifo（父例化）与形态选择例化；
//           数据通路核心（拼装 / 单元特征解析 / CRC / seq_check / err_discard /
//           paif_rx_if / 统计 / ACK-NACK / ARQ）全部沉入按编译宏选中的形态子模块。
//           形态选择矩阵（§2.1）：LINK_RX_COMPAT_MODE → 形态④（兼容）；
//           else PAIF_INTF_512 → 形态①（512×1 lane）；else LINK_LANE1 → 形态③
//           （1024×2 lane）；else → 形态②（1024×1 lane）。
//           形态①②③走 asm_unit 交接→父 crc16_check 单引擎分时；形态④内嵌 crc9。
//           全 RX 数据通路无 flush（去 flush 恢复语义见各形态 MAS）。
// 编码约束（本模块强制）:
//   1. 仅时序信号使用 always 语句（always @(posedge mb_clk)，同步复位，禁用异步复位）；
//   2. always 内部 if 判断（不含 reset）≤ 2 个；3. 组合只允许 assign；
//   4. for 循环仅用于 generate；5. wire 声明集中在信号声明区。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-08  OpenHappy Team  初始版本（单体，对齐 L2 v1.0 / 各 MAS v1.0）
//  2026-08-14  OpenHappy Team  后半重构为 128 B 单元流 + err_discard 内置 /
//                           ARQ 自生成与解析 + 重训练去掉 rx_flush
//  2026-09-07  OpenHappy Team  形态拆分重构：删 4 块形态度数据通路（拼装 / 特征解析 /
//                          CRC / seq_check / err_discard / paif_rx_if / 统计 /
//                          ack-nack / ARQ），沉入 4 个形态子模块；本文件收缩为壳，
//                          按 §2.1 宏矩阵例化形态 + credit_fifo + crc16_check（①②③）
//                          + lane_dskew（③）；删除 rx_flush_i（全通路去 flush）。
//  2026-09-08  OpenHappy Team  形态① CRC16 归属改走 asm_unit 交接→父 crc16_check 回接
//                          （同形态②③），删除形态① 内嵌 crc16。
//  2026-09-11  OpenHappy Team  桥接语义一致化（对齐 link_rx_mas v1.9 §3.1 桥接注）：
//                          新增 credit_en_eff = csr_credit_en_i & ~csr_bridge_mode_i，
//                          桥接下交付 ACK / credit 反馈等效关闭（与 crc_en_eff 对称，
//                          MAS §5.7 / §7：dlv_ack 仅 CREDIT_EN=1 时生成）。
// =============================================================================
//  2026-09-29  OpenHappy Team  LL-TIMING-N1（DC 时序 N=1，N=0 基线 job 400776）：
//                          ① CLKICG 18→≤16（sq_1/loopback 控制穿 1024 位数据
//                          mux→512 位 OR 树）——beat_nz 预 OR 上提本壳：
//                          lane_nz_lb=|lb_pipe_data[511:0]（寄存器驱动）/
//                          lane_nz_cpif=|cpif_lane_data[511:0]（输入驱动），
//                          lb_lane_nz=(loopback_en&nz_lb)|(~loopback_en&nz_cpif)
//                          （OR 对公共选通分配律，位级含 X 语义等价），形态①
//                          经新口 lane0_data_nz_i 消费（MAS 差异已回报）；
//                          ② reg2out 7→≤6——udi 通路 ~proto_is_paif 显式前反相
//                          （proto_not），破 proto 网上双反相+驱动链。

`timescale 1ns/1ps

`include "link_layer_defines.vh"

//  2026-10-13  OpenHappy Team  动态时钟控制去总开关（arch LL-20261013-dynclk 通过）：删
//                          input clk_cfg_rx_dyn_en_i（总开关），clken_rx 化简
//                          ~(dyn_en&n_dyn)|… → ~n_dyn|…（NAND2→INV -1 级）；本路
//                          clk_cfg_rx_n_dyn_i 语义改「本 RX 独立动态时钟开关」。
module link_rx (
    // =========================================================================
    // 时钟与复位
    // =========================================================================
    input  wire                       mb_clk,                 // 2 GHz 数据通路主时钟
    input  wire                       mb_rst_n,               // 同步复位，低有效
    input  wire                       clk_cfg_rx_n_dyn_i,     // 本 RX 独立动态时钟开关（CLK_CFG_RX_REG[n]）：1=动态 clken / 0=always-on（总开关已删）

    // =========================================================================
    // 上游 CPIF RX 接口（Logic PHY → link_rx，无反压无 ready）
    // =========================================================================
    input  wire [1023:0]              cpif_rx_lane0_data_i,
    input  wire                       cpif_rx_lane0_data_vld_i,
`ifdef LINK_LANE1
    input  wire [1023:0]              cpif_rx_lane1_data_i,
    input  wire                       cpif_rx_lane1_data_vld_i,
`endif
`ifdef LINK_LANE3
    input  wire [1023:0]              cpif_rx_lane2_data_i,
    input  wire                       cpif_rx_lane2_data_vld_i,
    input  wire [1023:0]              cpif_rx_lane3_data_i,
    input  wire                       cpif_rx_lane3_data_vld_i,
`endif

    // =========================================================================
    // Loopback 回环输入（顶层按 LOOPBACK_CFG 源选择从 link_tx CPIF TX 出口引回；RX 侧不接 src 配置）
    // =========================================================================
    input  wire [1023:0]              loopback_tx_lane0_data_i,
    input  wire                       loopback_tx_lane0_data_vld_i,
`ifdef LINK_LANE1
    input  wire [1023:0]              loopback_tx_lane1_data_i,
    input  wire                       loopback_tx_lane1_data_vld_i,
`endif
`ifdef LINK_LANE3
    input  wire [1023:0]              loopback_tx_lane2_data_i,
    input  wire                       loopback_tx_lane2_data_vld_i,
    input  wire [1023:0]              loopback_tx_lane3_data_i,
    input  wire                       loopback_tx_lane3_data_vld_i,
`endif

    // =========================================================================
    // CSR 配置（apb_csr 解码后经顶层同步器送入，mb_clk 域）
    // =========================================================================
    input  wire                       csr_s2p_mode_i,         // PHY_MODE.s2p_mode：1=1024 / 0=低512有效
    input  wire [1:0]                 csr_bw_mode_i,          // BW_CFG_REG.bw_mode：00=全带宽 / 01=1/2 / 10=1/4
    input  wire                       csr_protocol_mode_i,    // MODE_REG.PROTOCOL_MODE：1=PAIF / 0=UDI
    input  wire                       csr_loopback_en_i,      // LOOPBACK_CFG_REG.LOOPBACK_EN
    input  wire                       csr_perf_mode_i,            // MODE_REG.WORK_MODE=1 性能模式
    input  wire                       csr_comp_mode_i,            // MODE_REG.WORK_MODE=0 兼容模式
    input  wire                       csr_bridge_mode_i,          // public CSR bridge_en（桥接：RX 走流水线，差错处理等效 CRC 关闭）
    input  wire                       csr_crc_en_i,               // RETRY_CFG_REG.CRC_EN
    input  wire                       csr_ecc_en_i,               // RETRY_CFG_REG.ECC_EN（性能模式；桥接下等效关闭）
    input  wire                       csr_credit_en_i,            // RETRY_CFG_REG.CREDIT_EN
    input  wire [1:0]                 csr_nack_mode_i,            // RETRY_CFG_REG.NACK_MODE：00=Standard / 01=Select
    input  wire                       rx_flit_clr_i,
    input  wire                       crc_err_clr_i,
    input  wire                       csr_retry_en_i,             // RETRY_CFG_REG.RETRY_EN（门控解析通道 ACK/NACK 与本地 NACK 生成）

    // =========================================================================
    // link_rx_lcmsm 状态机接口（经顶层 sb_clk→mb_clk 同步）
    // =========================================================================
    input  wire                       train_start_i,          // deskew 训练启动脉冲
    input  wire                       rx_path_en_i,           // RX 数据通路使能（= rx_active 源）

    // =========================================================================
    // 下游 PAIF RX 接口（link_rx → 协议层）
    // =========================================================================
    output wire [`PAIF_WIDTH-1:0]     paif_rx_data_o,
    output wire                       paif_rx_valid_o,
    input  wire                       paif_rx_ready_i,

    // =========================================================================
    // UDI RX 接口（udi_bypass_mux → 用户逻辑，无反压）
    // =========================================================================
    output wire [1023:0]              udi_rx_lane0_data_o,
    output wire                       udi_rx_lane0_data_vld_o,
`ifdef LINK_LANE1
    output wire [1023:0]              udi_rx_lane1_data_o,
    output wire                       udi_rx_lane1_data_vld_o,
`endif
`ifdef LINK_LANE3
    output wire [1023:0]              udi_rx_lane2_data_o,
    output wire                       udi_rx_lane2_data_vld_o,
    output wire [1023:0]              udi_rx_lane3_data_o,
    output wire                       udi_rx_lane3_data_vld_o,
`endif

    // =========================================================================
    // 反向通道（→ 本端 link_tx；解析类 → retry_ctrl，生成类 → DLP 嵌入 / ctrl_flit_gen）
    // =========================================================================
    output wire [7:0]                 nack_seq_o,             // 解析的 DLP NACK（→ retry_ctrl 触发重传）
    output wire                       nack_valid_o,
    output wire [7:0]                 ack_seq_o,              // 解析的 DLP ACK（→ retry_ctrl 推进 retire_ptr）
    output wire                       ack_valid_o,
    output wire [1:0]                 dlp_linknum_o,          // 解析 DLP 目标链路号（随 ack/nack 有效）
    output wire [7:0]                 dlv_ack_seq_o,          // 生成交付 ACK（flit 送完 PAIF 后）
    output wire                       dlv_ack_valid_o,
    output wire [7:0]                 lcl_nack_seq_o,         // 本地 CRC 失败 NACK
    output wire                       lcl_nack_valid_o,
    output wire [1:0]                 lcl_nack_type_o,        // 2'b10=Standard / 2'b11=Select
    output wire [255:0]               arq_req_flit_o,         // 兼容模式 ARQ(Request) payload（仅形态④）
    output wire                       arq_req_valid_o,
    output wire [15:0]                arq_req_rcv_ptr_o,
    output wire                       arq_req_rcv_valid_o,

    // =========================================================================
    // 状态 / 中断上报
    // =========================================================================
    output wire                       train_done_o,           // deskew 训练完成（→ link_rx_lcmsm）
    output wire                       train_fail_o,           // deskew 训练失败（→ irq_dskew_fail）
    output wire                       credit_overflow_o,      // credit FIFO 溢出脉冲
    output wire                       seq_mismatch_o,         // 序列号偏离脉冲
    output wire [15:0]                rx_flit_cnt_o,          // RX 协议层 Flit 总数（不含 NULL）
    output wire [15:0]                crc_err_cnt_o           // CRC 错误次数

);

    // =========================================================================
    // 内部信号声明（全部 wire / reg 集中本区，wire 与 assign 分离）
    // =========================================================================

    // ---- 编译期派生常量 -------------------------------------------------------
    localparam FW      = 1024 * `LANE_COUNT;

    // ---- 内部 ICG 动态时钟门控（clken_rx 组合；rx_noidleQ/QQ 未门控 mb_clk 域展宽）----
    wire          ck_rx;                  // 内部 ICG 输出门控时钟（CK=mb_clk，相位对齐）
    wire          clken_rx;               // 内部 ICG 使能：~(dyn_en & n_dyn)|rx_path_activity|rx_wake|rx_hold_2T
    wire          rx_path_activity;       // 通路活动：rx_path_en_i|shp_asm_unit_valid|cf_pop|crc16_err|ecc_err
    wire          rx_idle;                // 组合判 idle（L1 §4.10.4：(a)(b)(c) 全不拉高）
    wire          rx_wake;                // 唤醒：CPIF/loopback 各 lane vld 任一拍
    wire          rx_hold_2T;             // idle 展宽 2T：~rx_idle|rx_noidleQ|rx_noidleQQ
    reg           rx_noidleQ;             // 未门控 mb_clk 域展宽 DFF（门控关断前须仍运转）
    reg           rx_noidleQQ;

    // ---- 入口 mux（loopback_mux / udi_bypass_mux） ----------------------------
      wire                     rx_active;            // 通路使能（桥接模式下仍走流水线）
    wire                     crc_en_eff;           // 桥接等效 CRC 关闭使能
    wire                     ecc_en_eff;           // 桥接等效 ECC 关闭使能（ECC_EN & ~bridge，与 crc_en_eff 对称）
    wire                     credit_en_eff;        // 桥接等效 Credit 关闭使能（交付 ACK / credit 反馈门控）
    wire                     proto_is_paif;        // 1=Flit 流水线 / 0=UDI 直通
    wire [1023:0]            cpif_lane_data [0:`LANE_COUNT-1];
    wire [`LANE_COUNT-1:0]   cpif_lane_vld;
    wire [1023:0]            lbtx_lane_data [0:`LANE_COUNT-1];
    wire [`LANE_COUNT-1:0]   lbtx_lane_vld;
    wire [1023:0]            lb_lane_data   [0:`LANE_COUNT-1];
    wire [`LANE_COUNT-1:0]   lb_lane_vld;
    reg  [1023:0]            lb_pipe_data   [0:`LANE_COUNT-1];
    reg  [`LANE_COUNT-1:0]   lb_pipe_vld;
    reg  [1023:0]            udi_pipe_data  [0:`LANE_COUNT-1];
    reg  [`LANE_COUNT-1:0]   udi_pipe_vld;
    wire [1023:0]            rx_lane_data   [0:`LANE_COUNT-1];
    wire [`LANE_COUNT-1:0]   rx_lane_vld;
    wire [1023:0]            udi_lane_data  [0:`LANE_COUNT-1];
    wire [`LANE_COUNT-1:0]   udi_lane_vld;
    wire [`LANE_COUNT-1:0]   lane_nz_lb;        // loopback pipe 侧低 512 非零（寄存器驱动；LL-TIMING-N1 预 OR）
    wire [`LANE_COUNT-1:0]   lane_nz_cpif;      // 物理侧低 512 非零（输入驱动；预 OR）
    wire [`LANE_COUNT-1:0]   lb_lane_nz;        // beat_nz 选通位（loopback_en 只进单 bit；预 OR 位级等价）
    wire                     proto_not;         // ~proto_is_paif 显式前反相（LL-TIMING-N1：udi 门控破双反相驱动链）

    // ---- lane_dskew 输出（仅形态③多 lane 配置） --------------------------------
    wire [FW-1:0]            dskew_data;
    wire                     dskew_valid;

    // ---- credit_fifo（父例化，形态驱动 push / pop） -----------------------------
    wire                     cf_push;
    wire [1023:0]            cf_flit_in;
    wire                     cf_flit_end;
    wire                     cf_pop;
    wire [1023:0]            cf_flit_out;
    wire                     cf_flit_end_o;
    wire                     cf_pop_valid;
    wire                     cf_overflow;
    wire [7:0]               cf_occupancy;

    // ---- crc16 交接（形态①②③ asm_unit → 父 crc16_check 单引擎分时） ---------------
    wire [1023:0]            shp_asm_unit;
    wire                     shp_asm_unit_valid;
    wire                     crc16_pass;
    wire                     crc16_err;
    // ---- ecc_check（父例化，ECC_EN 使能态采样；与 crc16_check 互选） ---------------
    wire                     ecc_pass;            // ECC 校验通过（E+1 判通拍回接，D23 在 ecc_check 内完成）
    wire                     ecc_err;             // ECC 校验失败脉冲（不可纠 syndrome，E+1）
    wire [1023:0]            ecc_corr_unit;       // E+1 纠正后单元（ECC_EN=1 判通拍 push 数据源）

    // ---- 形态输出中间线（各形态上提至壳输出） -------------------------------------
    wire [1023:0]            shp_paif_data;
    wire                     shp_paif_valid;
    wire [7:0]               shp_dlv_seq;
    wire                     shp_dlv_ack;
    wire [31:0]              shp_rx_flit_cnt;
    wire [31:0]              shp_crc_err_cnt;
    wire                     shp_seq_mismatch;
    wire                     shp_ack_valid;
    wire [7:0]               shp_ack_seq;
    wire                     shp_nack_valid;
    wire [7:0]               shp_nack_seq;
    wire [1:0]               shp_dlp_linknum;
    wire                     shp_lcl_nack_valid;
    wire [7:0]               shp_lcl_nack_seq;
    wire [1:0]               shp_lcl_nack_type;
    wire [255:0]             shp_arq_flit;
    wire                     shp_arq_valid;
    wire [15:0]              shp_arq_rcv_ptr;
    wire                     shp_arq_rcv_valid;

    genvar                   gi;

    // =========================================================================
    // loopback_mux / udi_bypass_mux 入口（link_rx_mas §4.10）
    // =========================================================================
    // 桥接模式（bridge_en=1）：RX 数据仍走本模块正常流水线（不直连 CPIF↔PAIF 旁路），
    // 差错 / 流控等效关闭（link_rx_mas §3.1 桥接注）——CRC_EN=0（crc_en_eff 强制
    // CRC 全过，无 err_discard / 无 NACK / 直接 push）+ CREDIT_EN=0（credit_en_eff
    // 门控交付 ACK / credit 反馈，形态侧不生成 dlv_ack，见 link_rx_mas §5.7 / §7）。
    assign rx_active     = rx_path_en_i;
    assign proto_is_paif = csr_protocol_mode_i;
    assign crc_en_eff    = csr_crc_en_i & ~csr_bridge_mode_i;
    assign ecc_en_eff    = csr_ecc_en_i & ~csr_bridge_mode_i;   // 桥接：ECC 等效关闭（link_rx_mas §7.4）
    assign credit_en_eff = csr_credit_en_i & ~csr_bridge_mode_i;
    assign retry_en_eff  = csr_retry_en_i & ~csr_bridge_mode_i;   // ack/nack 生成与解析均由 RETRY_EN 门控（与 crc_en_eff 对称）
    // =========================================================================
    // 内部 ICG：u_icg_rx（本模块恰 1 处 sync_icg_pos；CK=mb_clk，Q=ck_rx 模块主时钟）
    //   clken_rx = ~clk_cfg_rx_n_dyn_i
    //              | rx_path_activity | rx_wake | rx_hold_2T
    //   rx_idle 按 L1 §4.10.4 组合定义：（a）rx_path_en 拉低（b）CPIF/loopback vld 均低
    //   （c）无在途 shp/cf/CRC/ECC。rx_active（=rx_path_en_i）仍供形态 .rx_active_i，
    //   活动/判 idle 用独立的 rx_path_activity 派生（不改既有语义）。
    //   本路开关 n_dyn 复位 0 → clken_rx 恒 1 → ck_rx=mb_clk（bit-exact 零回归）。
    // =========================================================================
    assign rx_path_activity = rx_path_en_i | shp_asm_unit_valid | cf_pop | crc16_err | ecc_err;
    assign rx_wake          = (|cpif_lane_vld) | (|lbtx_lane_vld);
    assign rx_idle          = ~rx_path_en_i & ~(|cpif_lane_vld) & ~(|lbtx_lane_vld)
                              & ~shp_asm_unit_valid & ~cf_pop & ~crc16_err & ~ecc_err;
    assign rx_hold_2T       = ~rx_idle | rx_noidleQ | rx_noidleQQ;
    assign clken_rx         = ~clk_cfg_rx_n_dyn_i
                              | rx_path_activity | rx_wake | rx_hold_2T;

    sync_icg_pos u_icg_rx (
        .RSTN (mb_rst_n),
        .CK   (mb_clk),
        .E    (clken_rx),
        .Q    (ck_rx)
    );

    // 进 idle 判定与展宽须在门控关断前仍运转 → noidle DFF 打未门控 mb_clk 域
    always @(posedge mb_clk) begin
        if (!mb_rst_n) begin
            rx_noidleQ  <= 1'b0;
            rx_noidleQQ <= 1'b0;
        end else begin
            rx_noidleQ  <= ~rx_idle;
            rx_noidleQQ <= rx_noidleQ;
        end
    end


    // CPIF / loopback 端口 → per-lane 数组
    assign cpif_lane_data[0] = cpif_rx_lane0_data_i;
    assign cpif_lane_vld[0]  = cpif_rx_lane0_data_vld_i;
    assign lbtx_lane_data[0] = loopback_tx_lane0_data_i;
    assign lbtx_lane_vld[0]  = loopback_tx_lane0_data_vld_i;
`ifdef LINK_LANE1
    assign cpif_lane_data[1] = cpif_rx_lane1_data_i;
    assign cpif_lane_vld[1]  = cpif_rx_lane1_data_vld_i;
    assign lbtx_lane_data[1] = loopback_tx_lane1_data_i;
    assign lbtx_lane_vld[1]  = loopback_tx_lane1_data_vld_i;
`endif
`ifdef LINK_LANE3
    assign cpif_lane_data[2] = cpif_rx_lane2_data_i;
    assign cpif_lane_vld[2]  = cpif_rx_lane2_data_vld_i;
    assign cpif_lane_data[3] = cpif_rx_lane3_data_i;
    assign cpif_lane_vld[3]  = cpif_rx_lane3_data_vld_i;
    assign lbtx_lane_data[2] = loopback_tx_lane2_data_i;
    assign lbtx_lane_vld[2]  = loopback_tx_lane2_data_vld_i;
    assign lbtx_lane_data[3] = loopback_tx_lane3_data_i;
    assign lbtx_lane_vld[3]  = loopback_tx_lane3_data_vld_i;
`endif

    // LL-TIMING-N1：udi 通路 ~proto 显式前反相（proto_not）——旧 ~proto_is_paif 在
    // 1024 位 AND + vld 门控双用点由 DC 插入双反相+驱动链（reg2out 7→≤6）
    assign proto_not = ~proto_is_paif;

    generate
        for (gi = 0; gi < `LANE_COUNT; gi = gi + 1) begin : gen_in_mux
            // loopback 回环 pipe（1 级）：TX→RX 回环不直连 RX 流水线，先寄存 1 级
            always @(posedge ck_rx) begin
                    lb_pipe_data[gi] <= lbtx_lane_data[gi];
                    lb_pipe_vld[gi]  <= lbtx_lane_vld[gi];
            end
            // loopback_mux：0=物理层通路（默认）/ 1=loopback 回环通路（取 pipe 后）
            assign lb_lane_data[gi]  = ({1024{csr_loopback_en_i}} & lb_pipe_data[gi]) | ({1024{~csr_loopback_en_i}} & cpif_lane_data[gi]);
            assign lb_lane_vld[gi]   = (csr_loopback_en_i & lb_pipe_vld[gi]) | (~csr_loopback_en_i & cpif_lane_vld[gi]);
            // LL-TIMING-N1：beat_nz 预 OR 上提（OR 对公共选通 loopback mux 分配律：
            // |(sel&a[511:0] | ~sel&b[511:0]) = sel&|a[511:0] | ~sel&|b[511:0]，位级含
            // X 语义等价——sel 为常量选通位）。旧结构：loopback 选通经 1024 位数据
            // mux 骑入每 bit → 512 位 OR 树（sq_1 控制→lane_half ICG 18 级控制穿数据）；
            // 新结构：两棵 512 位 OR 树各自独立驱动源（pipe 寄存器 / cpif 输入），
            // 选通只进 1 bit 预选。
            assign lane_nz_lb[gi]   = |lb_pipe_data[gi][511:0];
            assign lane_nz_cpif[gi] = |cpif_lane_data[gi][511:0];
            assign lb_lane_nz[gi]   = (csr_loopback_en_i & lane_nz_lb[gi]) | (~csr_loopback_en_i & lane_nz_cpif[gi]);
            // UDI RX pipe（1 级）：UDI 直通旁路输出打一拍再送 udi_rx_lane*，
            // 打断 CPIF/loopback 接口输入→UDI 输出的纯组合直通 in-out path
            // （注：UDI 模式 proto_is_paif=0 且 loopback_en=0，此处寄存即打断 input⇒output）
            always @(posedge ck_rx) 
              if (proto_not)
                begin
                    udi_pipe_data[gi] <= lb_lane_data[gi];
                    udi_pipe_vld[gi]  <= lb_lane_vld[gi];
                end
        
            // udi_bypass_mux：1=Flit 流水线（默认）/ 0=UDI 直通（取 pipe 后，非组合直通）
            assign rx_lane_data[gi]  = lb_lane_data[gi];
            assign rx_lane_vld[gi]   = proto_is_paif & lb_lane_vld[gi];
            assign udi_lane_data[gi] =  udi_pipe_data[gi];
            assign udi_lane_vld[gi]  =  udi_pipe_vld[gi];
        end
    endgenerate

    // UDI RX 输出驱动（udi_bypass_mux 直通通路）
    assign udi_rx_lane0_data_o     = udi_lane_data[0];
    assign udi_rx_lane0_data_vld_o = udi_lane_vld[0];
`ifdef LINK_LANE1
    assign udi_rx_lane1_data_o     = udi_lane_data[1];
    assign udi_rx_lane1_data_vld_o = udi_lane_vld[1];
`endif
`ifdef LINK_LANE3
    assign udi_rx_lane2_data_o     = udi_lane_data[2];
    assign udi_rx_lane2_data_vld_o = udi_lane_vld[2];
    assign udi_rx_lane3_data_o     = udi_lane_data[3];
    assign udi_rx_lane3_data_vld_o = udi_lane_vld[3];
`endif

    // =========================================================================
    // credit_fifo（父例化；形态侧驱动 push / pop，契约见各形态 MAS）
    // =========================================================================
    credit_fifo u_credit_fifo (
        .mb_clk        (ck_rx),
        .mb_rst_n      (mb_rst_n),
        .csr_perf_mode_i   (csr_perf_mode_i),
        .push_i        (cf_push),
        .flit_in_i     (cf_flit_in),
        .flit_end_i    (cf_flit_end),
        .pop_i         (cf_pop),
        .flit_out_o    (cf_flit_out),
        .flit_end_o    (cf_flit_end_o),
        .pop_valid_o   (cf_pop_valid),
        .overflow_o    (cf_overflow),
        .occupancy_o   (cf_occupancy)
    );

    // =========================================================================
    // 形态选择例化（§2.1 矩阵）：形态④ ⪢ 形态① ⪢ 形态③ / 形态②
    // =========================================================================

    // -------------------------------------------------------------------------
    // 形态④（兼容模式 LINK_RX_COMPAT_MODE）：内嵌 crc9 + ARQ，1 lane
    // -------------------------------------------------------------------------
`ifdef LINK_RX_COMPAT_MODE
    link_rx_compat u_link_rx (
        .mb_clk                (ck_rx),
        .mb_rst_n              (mb_rst_n),
        .csr_perf_mode_i       (csr_perf_mode_i),
        .csr_comp_mode_i       (csr_comp_mode_i),
        .csr_protocol_mode_i   (csr_protocol_mode_i),
        .csr_s2p_mode_i        (csr_s2p_mode_i),
        .csr_bw_mode_i         (csr_bw_mode_i),
        .csr_crc_en_i          (crc_en_eff),
        .csr_credit_en_i       (credit_en_eff),
        .csr_retry_en_i       (retry_en_eff),
        .csr_nack_mode_i       (csr_nack_mode_i),
        .rx_active_i           (rx_active),
        .lane0_data_i          (rx_lane_data[0]),
        .lane0_valid_i         (rx_lane_vld[0]),
        .cf_flit_out_i         (cf_flit_out),
        .cf_flit_end_i         (cf_flit_end_o),
        .cf_pop_valid_i        (cf_pop_valid),
        .cf_occupancy_i        (cf_occupancy),
        .paif_rx_ready_i       (paif_rx_ready_i),
        .rx_flit_clr_i         (rx_flit_clr_i),
        .crc_err_clr_i         (crc_err_clr_i),
        .cf_push_o             (cf_push),
        .cf_flit_in_o          (cf_flit_in),
        .cf_flit_end_o         (cf_flit_end),
        .cf_pop_o              (cf_pop),
        .ack_valid_o           (shp_ack_valid),
        .ack_seq_o             (shp_ack_seq),
        .nack_valid_o          (shp_nack_valid),
        .nack_seq_o            (shp_nack_seq),
        .dlp_linknum_o         (shp_dlp_linknum),
        .lcl_nack_valid_o      (shp_lcl_nack_valid),
        .lcl_nack_seq_o        (shp_lcl_nack_seq),
        .lcl_nack_type_o       (shp_lcl_nack_type),
        .arq_req_flit_o        (shp_arq_flit),
        .arq_req_valid_o       (shp_arq_valid),
        .arq_req_rcv_ptr_o     (shp_arq_rcv_ptr),
        .arq_req_rcv_valid_o   (shp_arq_rcv_valid),
        .paif_rx_data_o        (shp_paif_data),
        .paif_rx_valid_o       (shp_paif_valid),
        .dlv_ack_seq_o         (shp_dlv_seq),
        .dlv_ack_o             (shp_dlv_ack),
        .seq_mismatch_o        (shp_seq_mismatch),
        .rx_flit_cnt_o         (shp_rx_flit_cnt),
        .crc_err_cnt_o         (shp_crc_err_cnt)
    );
    assign train_done_o = train_start_i;
    assign train_fail_o = 1'b0;

    // -------------------------------------------------------------------------
    // 形态①（PAIF_INTF_512 × 1 lane）：asm_unit 交接→父 crc16_check
    // -------------------------------------------------------------------------
`elsif PAIF_INTF_512
    crc16_check u_crc16_check (
        .mb_clk             (ck_rx),
        .mb_rst_n           (mb_rst_n),
        .csr_perf_mode_i    (csr_perf_mode_i),
        .csr_crc_en_i       (crc_en_eff),
        .unit_i             (shp_asm_unit),
        .flit_valid_i       (shp_asm_unit_valid),
        .pass_o             (crc16_pass),
        .err_o              (crc16_err)
    );

    
    // ----------------------------------------------------------------
    // ecc_check：与 crc16_check 并接同一 asm_unit（D23 E+1 打拍在内部完成），
    // E+1 判通拍回接 ecc_pass / ecc_err / ecc_corr_unit；父按 csr_ecc_en_i 互选采样
    // （ECC_EN=1 采本回接 / =0 采 crc16 回接；bridge 下 ecc_en_eff=0 等效关闭）
    // ----------------------------------------------------------------
    ecc_check u_ecc_check (
        .mb_clk             (ck_rx),
        .mb_rst_n           (mb_rst_n),
        .csr_perf_mode_i    (csr_perf_mode_i),
        .csr_ecc_en_i       (ecc_en_eff),
        .unit_i             (shp_asm_unit),
        .flit_valid_i       (shp_asm_unit_valid),
        .pass_o             (ecc_pass),
        .err_o              (ecc_err),
        .corr_unit_o        (ecc_corr_unit)
    );

    link_rx_512_lane1 u_link_rx (
        .mb_clk                (ck_rx),
        .mb_rst_n              (mb_rst_n),
        .csr_perf_mode_i       (csr_perf_mode_i),
        .csr_comp_mode_i       (csr_comp_mode_i),
        .csr_crc_en_i          (crc_en_eff),
        .csr_ecc_en_i          (ecc_en_eff),
        .csr_credit_en_i       (credit_en_eff),
        .csr_retry_en_i       (retry_en_eff),
        .csr_loopback_en_i    (csr_loopback_en_i),
        .csr_nack_mode_i       (csr_nack_mode_i),
        .rx_active_i           (rx_active),
        .lane0_data_i          (rx_lane_data[0]),
        .lane0_valid_i         (rx_lane_vld[0]),
        .lane0_data_nz_i       (lb_lane_nz[0]),
        .crc16_pass_i          (crc16_pass),
        .ecc_pass_i            (ecc_pass),
        .crc16_err_i           (crc16_err),
        .ecc_err_i             (ecc_err),
        .ecc_corr_unit_i       (ecc_corr_unit),
        .cf_flit_out_i         (cf_flit_out),
        .cf_flit_end_i         (cf_flit_end_o),
        .cf_pop_valid_i        (cf_pop_valid),
        .cf_occupancy_i        (cf_occupancy),
        .paif_rx_ready_i       (paif_rx_ready_i),
        .rx_flit_clr_i         (rx_flit_clr_i),
        .crc_err_clr_i         (crc_err_clr_i),
        .asm_unit_o            (shp_asm_unit),
        .asm_unit_valid_o      (shp_asm_unit_valid),
        .cf_push_o             (cf_push),
        .cf_flit_in_o          (cf_flit_in),
        .cf_flit_end_o         (cf_flit_end),
        .cf_pop_o              (cf_pop),
        .ack_valid_o           (shp_ack_valid),
        .ack_seq_o             (shp_ack_seq),
        .nack_valid_o          (shp_nack_valid),
        .nack_seq_o            (shp_nack_seq),
        .dlp_linknum_o         (shp_dlp_linknum),
        .lcl_nack_valid_o      (shp_lcl_nack_valid),
        .lcl_nack_seq_o        (shp_lcl_nack_seq),
        .lcl_nack_type_o       (shp_lcl_nack_type),
        .paif_rx_data_o        (shp_paif_data),
        .paif_rx_valid_o       (shp_paif_valid),
        .dlv_ack_seq_o         (shp_dlv_seq),
        .dlv_ack_o             (shp_dlv_ack),
        .seq_mismatch_o        (shp_seq_mismatch),
        .rx_flit_cnt_o         (shp_rx_flit_cnt),
        .crc_err_cnt_o         (shp_crc_err_cnt)
    );
    assign shp_arq_flit        = 256'd0;
    assign shp_arq_valid       = 1'b0;
    assign shp_arq_rcv_ptr     = 16'd0;
    assign shp_arq_rcv_valid   = 1'b0;
    assign train_done_o = train_start_i;
    assign train_fail_o = 1'b0;

    // -------------------------------------------------------------------------
    // 形态③（1024 × 2 lane）: lane_dskew 对齐 + crc16 asm_unit 交接
    // -------------------------------------------------------------------------
`elsif LINK_LANE1
    lane_dskew u_lane_dskew (
        .mb_clk             (ck_rx),
        .mb_rst_n           (mb_rst_n),
        .train_start_i      (train_start_i),
        .csr_s2p_mode_i     (csr_s2p_mode_i),
        .csr_bw_mode_i      (csr_bw_mode_i),
        .lane0_data_i       (rx_lane_data[0]),
        .lane0_valid_i      (rx_lane_vld[0]),
        .lane1_data_i       (rx_lane_data[1]),
        .lane1_valid_i      (rx_lane_vld[1]),
        .data_o             (dskew_data),
        .valid_o            (dskew_valid),
        .train_done_o       (train_done_o),
        .train_fail_o       (train_fail_o)
    );

    crc16_check u_crc16_check (
        .mb_clk             (ck_rx),
        .mb_rst_n           (mb_rst_n),
        .csr_perf_mode_i    (csr_perf_mode_i),
        .csr_crc_en_i       (crc_en_eff),
        .unit_i             (shp_asm_unit),
        .flit_valid_i       (shp_asm_unit_valid),
        .pass_o             (crc16_pass),
        .err_o              (crc16_err)
    );

    
    // ----------------------------------------------------------------
    // ecc_check：与 crc16_check 并接同一 asm_unit（D23 E+1 打拍在内部完成），
    // E+1 判通拍回接 ecc_pass / ecc_err / ecc_corr_unit；父按 csr_ecc_en_i 互选采样
    // （ECC_EN=1 采本回接 / =0 采 crc16 回接；bridge 下 ecc_en_eff=0 等效关闭）
    // ----------------------------------------------------------------
    ecc_check u_ecc_check (
        .mb_clk             (ck_rx),
        .mb_rst_n           (mb_rst_n),
        .csr_perf_mode_i    (csr_perf_mode_i),
        .csr_ecc_en_i       (ecc_en_eff),
        .unit_i             (shp_asm_unit),
        .flit_valid_i       (shp_asm_unit_valid),
        .pass_o             (ecc_pass),
        .err_o              (ecc_err),
        .corr_unit_o        (ecc_corr_unit)
    );

    link_rx_1024_lane2 u_link_rx (
        .mb_clk                (ck_rx),
        .mb_rst_n              (mb_rst_n),
        .csr_perf_mode_i       (csr_perf_mode_i),
        .csr_comp_mode_i       (csr_comp_mode_i),
        .csr_protocol_mode_i   (csr_protocol_mode_i),
        .csr_s2p_mode_i        (csr_s2p_mode_i),
        .csr_bw_mode_i         (csr_bw_mode_i),
        .csr_crc_en_i          (crc_en_eff),
        .csr_ecc_en_i          (ecc_en_eff),
        .csr_credit_en_i       (credit_en_eff),
        .csr_retry_en_i       (retry_en_eff),
        .csr_nack_mode_i       (csr_nack_mode_i),
        .rx_active_i           (rx_active),
        .dskew_data_i          (dskew_data),
        .dskew_valid_i         (dskew_valid),
        .crc16_pass_i          (crc16_pass),
        .ecc_pass_i            (ecc_pass),
        .crc16_err_i           (crc16_err),
        .ecc_err_i             (ecc_err),
        .ecc_corr_unit_i       (ecc_corr_unit),
        .cf_flit_out_i         (cf_flit_out),
        .cf_flit_end_i         (cf_flit_end_o),
        .cf_pop_valid_i        (cf_pop_valid),
        .cf_occupancy_i        (cf_occupancy),
        .paif_rx_ready_i       (paif_rx_ready_i),
        .rx_flit_clr_i         (rx_flit_clr_i),
        .crc_err_clr_i         (crc_err_clr_i),
        .asm_unit_o            (shp_asm_unit),
        .asm_unit_valid_o      (shp_asm_unit_valid),
        .cf_push_o             (cf_push),
        .cf_flit_in_o          (cf_flit_in),
        .cf_flit_end_o         (cf_flit_end),
        .cf_pop_o              (cf_pop),
        .ack_valid_o           (shp_ack_valid),
        .ack_seq_o             (shp_ack_seq),
        .nack_valid_o          (shp_nack_valid),
        .nack_seq_o            (shp_nack_seq),
        .dlp_linknum_o         (shp_dlp_linknum),
        .lcl_nack_valid_o      (shp_lcl_nack_valid),
        .lcl_nack_seq_o        (shp_lcl_nack_seq),
        .lcl_nack_type_o       (shp_lcl_nack_type),
        .paif_rx_data_o        (shp_paif_data),
        .paif_rx_valid_o       (shp_paif_valid),
        .dlv_ack_seq_o         (shp_dlv_seq),
        .dlv_ack_o             (shp_dlv_ack),
        .seq_mismatch_o        (shp_seq_mismatch),
        .rx_flit_cnt_o         (shp_rx_flit_cnt),
        .crc_err_cnt_o         (shp_crc_err_cnt)
    );
    assign shp_arq_flit        = 256'd0;
    assign shp_arq_valid       = 1'b0;
    assign shp_arq_rcv_ptr     = 16'd0;
    assign shp_arq_rcv_valid   = 1'b0;

    // -------------------------------------------------------------------------
    // 形态②（1024 × 1 lane）：crc16 asm_unit 交接
    // -------------------------------------------------------------------------
`else
    crc16_check u_crc16_check (
        .mb_clk             (ck_rx),
        .mb_rst_n           (mb_rst_n),
        .csr_perf_mode_i    (csr_perf_mode_i),
        .csr_crc_en_i       (crc_en_eff),
        .unit_i             (shp_asm_unit),
        .flit_valid_i       (shp_asm_unit_valid),
        .pass_o             (crc16_pass),
        .err_o              (crc16_err)
    );

    
    // ----------------------------------------------------------------
    // ecc_check：与 crc16_check 并接同一 asm_unit（D23 E+1 打拍在内部完成），
    // E+1 判通拍回接 ecc_pass / ecc_err / ecc_corr_unit；父按 csr_ecc_en_i 互选采样
    // （ECC_EN=1 采本回接 / =0 采 crc16 回接；bridge 下 ecc_en_eff=0 等效关闭）
    // ----------------------------------------------------------------
    ecc_check u_ecc_check (
        .mb_clk             (ck_rx),
        .mb_rst_n           (mb_rst_n),
        .csr_perf_mode_i    (csr_perf_mode_i),
        .csr_ecc_en_i       (ecc_en_eff),
        .unit_i             (shp_asm_unit),
        .flit_valid_i       (shp_asm_unit_valid),
        .pass_o             (ecc_pass),
        .err_o              (ecc_err),
        .corr_unit_o        (ecc_corr_unit)
    );

    link_rx_1024_lane1 u_link_rx (
        .mb_clk                (ck_rx),
        .mb_rst_n              (mb_rst_n),
        .csr_perf_mode_i       (csr_perf_mode_i),
        .csr_comp_mode_i       (csr_comp_mode_i),
        .csr_protocol_mode_i   (csr_protocol_mode_i),
        .csr_s2p_mode_i        (csr_s2p_mode_i),
        .csr_bw_mode_i         (csr_bw_mode_i),
        .csr_crc_en_i          (crc_en_eff),
        .csr_ecc_en_i          (ecc_en_eff),
        .csr_credit_en_i       (credit_en_eff),
        .csr_retry_en_i       (retry_en_eff),
        .csr_nack_mode_i       (csr_nack_mode_i),
        .rx_active_i           (rx_active),
        .lane0_data_i          (rx_lane_data[0]),
        .lane0_valid_i         (rx_lane_vld[0]),
        .crc16_pass_i          (crc16_pass),
        .ecc_pass_i            (ecc_pass),
        .crc16_err_i           (crc16_err),
        .ecc_err_i             (ecc_err),
        .ecc_corr_unit_i       (ecc_corr_unit),
        .cf_flit_out_i         (cf_flit_out),
        .cf_flit_end_i         (cf_flit_end_o),
        .cf_pop_valid_i        (cf_pop_valid),
        .cf_occupancy_i        (cf_occupancy),
        .paif_rx_ready_i       (paif_rx_ready_i),
        .rx_flit_clr_i         (rx_flit_clr_i),
        .crc_err_clr_i         (crc_err_clr_i),
        .asm_unit_o            (shp_asm_unit),
        .asm_unit_valid_o      (shp_asm_unit_valid),
        .cf_push_o             (cf_push),
        .cf_flit_in_o          (cf_flit_in),
        .cf_flit_end_o         (cf_flit_end),
        .cf_pop_o              (cf_pop),
        .ack_valid_o           (shp_ack_valid),
        .ack_seq_o             (shp_ack_seq),
        .nack_valid_o          (shp_nack_valid),
        .nack_seq_o            (shp_nack_seq),
        .dlp_linknum_o         (shp_dlp_linknum),
        .lcl_nack_valid_o      (shp_lcl_nack_valid),
        .lcl_nack_seq_o        (shp_lcl_nack_seq),
        .lcl_nack_type_o       (shp_lcl_nack_type),
        .paif_rx_data_o        (shp_paif_data),
        .paif_rx_valid_o       (shp_paif_valid),
        .dlv_ack_seq_o         (shp_dlv_seq),
        .dlv_ack_o             (shp_dlv_ack),
        .seq_mismatch_o        (shp_seq_mismatch),
        .rx_flit_cnt_o         (shp_rx_flit_cnt),
        .crc_err_cnt_o         (shp_crc_err_cnt)
    );
    assign shp_arq_flit        = 256'd0;
    assign shp_arq_valid       = 1'b0;
    assign shp_arq_rcv_ptr     = 16'd0;
    assign shp_arq_rcv_valid   = 1'b0;
    assign train_done_o = train_start_i;
    assign train_fail_o = 1'b0;
`endif

    // =========================================================================
    // 形态输出上提（壳输出 / PAIF 位宽裁剪 / 统计截低位）
    // =========================================================================
    `ifdef PAIF_INTF_512
        assign paif_rx_data_o = shp_paif_data[511:0];
    `else
        assign paif_rx_data_o = shp_paif_data;
    `endif
    assign paif_rx_valid_o   = shp_paif_valid;
    assign dlv_ack_seq_o     = shp_dlv_seq;
    assign dlv_ack_valid_o   = shp_dlv_ack;
    assign nack_seq_o        = shp_nack_seq;
    assign nack_valid_o      = shp_nack_valid;
    assign ack_seq_o         = shp_ack_seq;
    assign ack_valid_o       = shp_ack_valid;
    assign dlp_linknum_o     = shp_dlp_linknum;
    assign lcl_nack_seq_o    = shp_lcl_nack_seq;
    assign lcl_nack_valid_o  = shp_lcl_nack_valid;
    assign lcl_nack_type_o   = shp_lcl_nack_type;
    assign arq_req_flit_o    = shp_arq_flit;
    assign arq_req_valid_o   = shp_arq_valid;
    assign arq_req_rcv_ptr_o = shp_arq_rcv_ptr;
    assign arq_req_rcv_valid_o = shp_arq_rcv_valid;
    assign credit_overflow_o = cf_overflow;
    assign seq_mismatch_o    = shp_seq_mismatch;
    assign rx_flit_cnt_o     = shp_rx_flit_cnt[15:0];
    assign crc_err_cnt_o     = shp_crc_err_cnt[15:0];
    // 动态活动已并入 clken_rx 组合（内部 u_icg_rx），不再上提。

endmodule
