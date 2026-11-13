// =============================================================================
// module name：sbc_cpif_event
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/sideband/mas/sbc_cpif_event_mas.md
// =============================================================================
// 功能概述: CPIF 事件接口与训练控制聚合器（per-link 唯一）：CPIF TX/RX 事件完整握手 +
//   EventCmd 解码 + per-lane 聚合 + toggle-handshake CDC（mb_clk / phy_rx_clk{i} ↔ sb_tx_clk）
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-07  OpenHappy Team  初始版本：基于 sbc_cpif_event MAS v1.2 生成
//  2026-08-11  OpenHappy Team  TX/RX_LANE_MASK 编译期方向掩码（per-lane 方向不对称
//                       例化适配，MAS v1.3）：lane_enable 单 bitmap 软件契约不变，
//                       有效使能 = lane_enable & 方向掩码
//  2026-08-28  OpenHappy Team  4T1R（4 TX + 1 RX）扩容（MAS v1.5）：LANE_NUM 硬件 2→4，
//                       TX 方向 per-lane 全部扩到 4（7×4）；RX 方向 per-lane CDC 容量
//                       按 LANE_NUM=4 展开（对齐 blueprint internals u_rx_train_toggle=4 /
//                       u_sync_rx_train=4 等）；由于物理仅 phy_rx_clk0/1 两路 RX 标量时钟，
//                       lanes 2~3 的 RX CDC 时钟复接可用时钟且 RX_LANE_MASK 仅使能 lane0
//                       （4T1R 单 RX），高位 CDC 输出被方向掩码剔除、无消费
//  2026-09-08  OpenHappy Team  RX 例化收窄（RTL-FIX-LPCLK-01）：gen_lane_rx 按 RX_EN_LANE_NUM
//                       （=RX_LANE_MASK popcount，4T1R=1/full=4）仅对 RX 使能 lane 实例化
//                       RX CDC flop，去除 TX-only lane 的 tie-low 时钟死 flop（FlopClockConstant）；
//                       残留向量 rxt_sync2/rxt_edge/lane_enable_rx 高位及 rx_retrain_req_o /
//                       train_complete_ack_o 在 gen_rx_tieoff 恒 0；RX bundle 位宽仍按
//                       RX_LANE_NUM=4 保持（接口零变更）。注：popcount 隐含 RX lane 低位连续，
//                       未来非连续 RX 掩码需改 per-lane 条件门控
//  2026-09-18  OpenHappy Team  BUG 修复：RX_EN_LANE_NUM popcount 加数扩展 3 bit——原 1-bit 加数链式
//                       相加，mask=4'b1111（st 4R1T）时和回绕为 0 → [(RX_EN_LANE_NUM-1):0]
//                       =[-1:0] → STX_VE_800/491（与 sbc_dispatcher.v 同因同修，2026-09-18）
// =============================================================================

`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// 设计说明:
//
// 时钟域:
//   - sb_tx_clk：主域（CPIF 握手 FSM / decode / encode / slot / 聚合 sb 侧）
//   - mb_clk：TX 方向 per-step req pending+AND 聚合+toggle 发送端与
//     complete/train/retrain 接收端
//   - phy_rx_clk{0/1}：train_req toggle 发送端与 rx_retrain /
//     train_complete_ack 接收端（RX 方向 per-lane，容量 RX_LANE_NUM=LANE_NUM=4；
//     lanes 2~3 无专用时钟复接，RX_LANE_MASK 仅使能 lane0 故高位无消费）
// -----------------------------------------------------------------------------

module sbc_cpif_event #(
    // 编译期方向掩码（logic_phy 按 LP_ lane 宏派生，截断到本模块 LANE_NUM=4 位宽）：
    // bit i=1 表示 lane i 该方向已例化；有效使能 = lane_enable_i & 方向掩码，
    // 软件对未例化 lane 置位无害（掩码剔除）
    parameter TX_LANE_MASK = 4'b1111,
    parameter RX_LANE_MASK = 4'b1111
) (
    // -------------------------------------------------------------------------
    // 时钟与复位
    // -------------------------------------------------------------------------
    input  wire        sb_tx_clk,
    input  wire        sb_rst_n,
    input  wire        mb_rstn_sync,             // mb_clk 域同步复位（CRG，Group C）
    input  wire [3:0]  rx_rstn_sync,     // 每 lane phy_rx_clk{i} 域同步复位（CRG，Group C；per-lane 位宽 [3:0]=LANE_NUM，同其他 per-lane 端口约定）
    input  wire        mb_clk,
    input  wire        phy_rx_clk0,
    input  wire        phy_rx_clk1,

    // -------------------------------------------------------------------------
    // CPIF TX 事件接口（link_layer，sb_tx_clk 域）
    // -------------------------------------------------------------------------
    input  wire        tx_event_enable_i,
    input  wire        tx_event_req_i,
    input  wire [15:0] tx_event_cmd_i,
    output reg         tx_event_ack_o,
    output wire [7:0]  tx_event_status_o,

    // -------------------------------------------------------------------------
    // CPIF RX 事件接口（link_layer，sb_tx_clk 域）
    // -------------------------------------------------------------------------
    input  wire        rx_event_enable_i,
    output reg         rx_event_req_o,
    output wire [15:0] rx_event_cmd_o,
    input  wire        rx_event_ack_i,
    input  wire [7:0]  rx_event_status_i,

    // -------------------------------------------------------------------------
    // dispatcher 报文分发输入（valid-only，sb_tx_clk 同域）
    // -------------------------------------------------------------------------
    input  wire [15:0] peer_retrain_pkt_i,
    input  wire        peer_retrain_pkt_valid_i,
    input  wire [15:0] link_passthru_i,
    input  wire        link_passthru_valid_i,
    input  wire        rx_data_check_agg_i,

    // -------------------------------------------------------------------------
    // dispatcher per-link complete 输入（peer complete 报文 decode 脉冲）
    // -------------------------------------------------------------------------
    input  wire        offset_complete_i,
    input  wire        vref_complete_i,
    input  wire        eq_complete_i,
    input  wire        pi_complete_i,
    input  wire        dcDL_complete_i,
    input  wire        reorder_complete_i,
    input  wire        data_check_complete_i,

    // -------------------------------------------------------------------------
    // dispatcher 报文输出（valid-ready）
    // -------------------------------------------------------------------------
    output wire [15:0] pkt_to_disp_o,
    output wire        pkt_to_disp_valid_o,
    input  wire        pkt_to_disp_ready_i,

    // -------------------------------------------------------------------------
    // dispatcher per-link req 输出（聚合后 1 cycle 脉冲，sb_tx_clk 同域）
    // -------------------------------------------------------------------------
    output wire        offset_req_o,
    output wire        vref_req_o,
    output wire        eq_req_o,
    output wire        pi_req_o,
    output wire        dcDL_req_o,
    output wire        reorder_req_o,
    output wire        data_check_req_o,

    // -------------------------------------------------------------------------
    // TX per-step req 输入（per-lane，mb_clk 域 1 cycle 脉冲，LANE_NUM=4）
    // -------------------------------------------------------------------------
    input  wire [3:0]  offset_req_i,
    input  wire [3:0]  vref_req_i,
    input  wire [3:0]  eq_req_i,
    input  wire [3:0]  pi_req_i,
    input  wire [3:0]  dcDL_req_i,
    input  wire [3:0]  reorder_req_i,
    input  wire [3:0]  data_check_req_i,

    // -------------------------------------------------------------------------
    // TX per-step complete 输出（per-lane，mb_clk 域 1 cycle 脉冲，LANE_NUM=4）
    // -------------------------------------------------------------------------
    output wire [3:0]  offset_complete_o,
    output wire [3:0]  vref_complete_o,
    output wire [3:0]  eq_complete_o,
    output wire [3:0]  pi_complete_o,
    output wire [3:0]  dcDL_complete_o,
    output wire [3:0]  reorder_complete_o,
    output wire [3:0]  data_check_complete_o,

    // -------------------------------------------------------------------------
    // TX 训练控制（mb_clk 域，per-lane，LANE_NUM=4）
    // -------------------------------------------------------------------------
    output wire [3:0]  train_req_o,
    output wire [3:0]  retrain_req_o,
    input  wire [3:0]  train_complete_ack_i,   // level：ACTIVE 保持高

    // -------------------------------------------------------------------------
    // RX 训练控制（phy_rx_clk{i} 域，per-lane bundle 按 LANE_NUM=4，实际 CDC 容量 RX_LANE_NUM）
    // -------------------------------------------------------------------------
    input  wire [3:0]  train_req_i,
    output wire [3:0]  rx_retrain_req_o,
    output wire [3:0]  train_complete_ack_o,

    // -------------------------------------------------------------------------
    // 配置接口（apb_csr_top，CDC 已完成，sb_tx_clk 域）
    // -------------------------------------------------------------------------
    input  wire [3:0]  lane_enable_i
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    localparam LANE_NUM     = 4;      // 最大 lane 能力（按 4 LANE 预留，4T1R：4 TX + 1 RX）
    localparam RX_LANE_NUM  = LANE_NUM;  // RX 方向 per-lane CDC 容量 = LANE_NUM=4（对齐 blueprint
                                      // internals：u_rx_train_toggle=4 / u_sync_rx_train=4 等按 4 LANE
                                      // 预留）；物理仅 phy_rx_clk0/1 两路 RX 时钟，lanes 2~3 时钟复接，
                                      // RX_LANE_MASK 仅 bit0（4T1R 单 RX）故高位 CDC 剔除无消费
    // RX 使能 lane 数 = RX_LANE_MASK popcount（direction 单一事实源，同 L317/318 消费）；
    // 4T1R=1 / full=4。注：popcount 隐含 RX lane 低位连续；未来若非连续 RX 掩码（如
    // lane0+lane2）需改 per-lane 条件门控，见头注释 2026-08-28 4T1R 条目。
    // 2026-09-18 修复：加数扩展 3 bit（原 1-bit 加数链加，mask=4'b1111 时回绕为 0 →
    // [(RX_EN_LANE_NUM-1):0]=[-1:0] → STX_VE_800/491；3-bit 和可承载 4 lane = 3'b100）
    localparam [2:0] RX_EN_LANE_NUM =
        {2'b00, RX_LANE_MASK[0]} + {2'b00, RX_LANE_MASK[1]} +
        {2'b00, RX_LANE_MASK[2]} + {2'b00, RX_LANE_MASK[3]};

    localparam OPC_RETRAIN   = 8'h80;  // retrain
    localparam OPC_RX_READY  = 8'h7F;  // RX READY（训练完成上报复用）
    localparam SBC_TYPE_REQ  = 3'b010; // SBC 帧 type：Request（非 CPIF EventCmd.Type）
    localparam SBC_TYPE_PASS = 3'b111; // SBC 帧 type：链路层透传
    localparam STATUS_OK     = 8'h00;  // 正常执行完成
    localparam STATUS_UNSUP  = 8'h01;  // 不支持的事件命令（项目扩展）

    // tx_event_fsm 状态编码（binary，reset=IDLE）
    localparam TX_IDLE = 2'b00;
    localparam TX_SEND = 2'b01;
    localparam TX_ACK  = 2'b10;

    // rx_event_fsm 状态编码（binary，reset=IDLE）
    localparam RX_IDLE     = 2'b00;
    localparam RX_REQ      = 2'b01;
    localparam RX_WAIT_ACK = 2'b10;

    // RX event slot 服务选择编码
    localparam SEL_RETRAIN = 2'd0;
    localparam SEL_PASS    = 2'd1;
    localparam SEL_TRAINC  = 2'd2;

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================

    // --------------------------------------------------------------------------
    // 时钟组
    // --------------------------------------------------------------------------
    wire [LANE_NUM-1:0]      phy_rx_clk_w;

    // --------------------------------------------------------------------------
    // CPIF TX 事件 decode（sb_tx_clk 域）
    // --------------------------------------------------------------------------
    wire                  cmd_type;       // CPIF EventCmd.Type（1 bit，非 SBC 帧 type）
    wire [7:0]            cmd_opcode;
    wire                  cpif_retrain_evt;
    wire                  retrain_evt;    // 两源 OR：CPIF Type=0 RETRAIN | peer retrain 报文
    reg  [7:0]            status_hold;
    reg  [15:0]           pkt_enc_hold;

    // --------------------------------------------------------------------------
    // FSM 状态
    // --------------------------------------------------------------------------
    reg  [1:0]            tx_cur_state;
    reg  [1:0]            tx_next_state;
    reg  [1:0]            rx_cur_state;
    reg  [1:0]            rx_next_state;

    // --------------------------------------------------------------------------
    // 控制 toggle（sb_tx_clk 域发送端）
    // --------------------------------------------------------------------------
    reg                   train_toggle;
    reg                   retrain_toggle;
    reg                   rx_retrain_toggle;
    reg                   compl_ack_toggle;

    // --------------------------------------------------------------------------
    // RX event slot（sb_tx_clk 域）
    // --------------------------------------------------------------------------
    reg                   slot_retrain;
    reg                   slot_pass;
    reg                   slot_trainc;
    reg  [15:0]           pass_cmd_hold;
    reg                   ovf_flag;        // passthru slot 溢出 sticky（DFX 观测）
    reg  [1:0]            serve_sel;
    reg  [15:0]           rx_event_cmd_hold;
    wire                  slot_any;
    wire                  rx_serve_start;
    wire                  rx_serve_done;
    reg  [1:0]            sel_n;
    reg  [15:0]           cmd_n;

    // --------------------------------------------------------------------------
    // RX train_req：发送 toggle（phy_rx_clk{i}，generate 局部，容量 RX_LANE_NUM）
    //   + 回收聚合（sb_tx_clk）
    // --------------------------------------------------------------------------
    wire [RX_LANE_NUM-1:0] rxt_sync2;
    reg  [RX_LANE_NUM-1:0] rxt_sync2_d;
    wire [RX_LANE_NUM-1:0] rxt_edge;
    reg  [RX_LANE_NUM-1:0] rx_train_pend;
    wire [RX_LANE_NUM-1:0] rx_train_arrived;
    wire                  rx_train_agg;
    reg                   rx_train_enc_pend;

    // --------------------------------------------------------------------------
    // pkt_to_disp 二源仲裁（tx_event SEND 优先于 rx_train encode）
    // --------------------------------------------------------------------------
    wire                  send_fire;
    wire                  rx_train_fire;

    // --------------------------------------------------------------------------
    // TX per-step req：per-lane pending + AND 聚合（mb_clk，LANE_NUM=4）+ toggle
    // --------------------------------------------------------------------------
    wire [LANE_NUM-1:0]   step_req_in [0:6];
    reg  [LANE_NUM-1:0]   req_pend [0:6];
    reg  [6:0]            req_toggle;
    wire [6:0]            req_agg_done;
    wire [6:0]            req_sync2;
    reg  [6:0]            req_sync2_d;
    wire [6:0]            req_edge;

    // --------------------------------------------------------------------------
    // TX per-step complete：发送 toggle（sb_tx_clk）+ 接收端（mb_clk，LANE_NUM=4）
    // --------------------------------------------------------------------------
    wire [6:0]            step_compl_in;
    reg  [6:0]            compl_toggle;
    wire [6:0]            cpl_sync2;
    reg  [6:0]            cpl_sync2_d;
    wire [6:0]            compl_edge;

    // --------------------------------------------------------------------------
    // train / retrain 接收端（mb_clk 域）
    // --------------------------------------------------------------------------
    wire                  train_sync2;
    reg                   train_sync2_d;
    wire                  train_edge;
    wire                  retrain_sync2;
    reg                   retrain_sync2_d;
    wire                  retrain_edge;

    // --------------------------------------------------------------------------
    // rx_retrain / train_complete_ack 接收端（phy_rx_clk{i} 域，容量 RX_LANE_NUM，
    // per-lane 触发器在 generate 作用域内声明）
    // --------------------------------------------------------------------------
    wire [RX_LANE_NUM-1:0]   rxr_sync2;
    wire [RX_LANE_NUM-1:0]   cak_sync2;

    // --------------------------------------------------------------------------
    // train_complete_ack_i level 同步 + AND 聚合 + 上升沿检测（sb_tx_clk，
    // TX 方向 per-lane，LANE_NUM=4）
    // --------------------------------------------------------------------------
    wire [LANE_NUM-1:0]   ack_sync2;
    reg                   ack_all_d;
    wire                  ack_all;
    wire                  ack_rise;

    // --------------------------------------------------------------------------
    // lane_enable 跨域同步（quasi-static level，2flop）
    //   lane_enable_mb（mb_clk，TX，LANE_NUM=4）/ lane_enable_rx（phy_rx_clk{i}，RX，RX_LANE_NUM）
    // --------------------------------------------------------------------------
    wire [LANE_NUM-1:0]     lane_enable_mb;
    wire [RX_LANE_NUM-1:0]  lane_enable_rx;

    // 方向有效使能 = lane_enable & 编译期方向掩码（TX_LANE_MASK/RX_LANE_MASK）：
    //   lane_en_tx_mb：mb_clk 域 TX 方向（req 聚合 / complete·train·retrain fanout）
    //   lane_en_tx_sb：sb_tx_clk 域 TX 方向（train_complete_ack AND 聚合）
    //   lane_en_rx_sb：sb_tx_clk 域 RX 方向（RX train_req AND 聚合，容量 RX_LANE_NUM）
    //   lane_en_rx_rx：phy_rx_clk{i} 域 RX 方向（rx_retrain / compl_ack fanout，容量 RX_LANE_NUM）
    wire [LANE_NUM-1:0]     lane_en_tx_mb;
    wire [LANE_NUM-1:0]     lane_en_tx_sb;
    wire [RX_LANE_NUM-1:0]  lane_en_rx_sb;
    wire [RX_LANE_NUM-1:0]  lane_en_rx_rx;

    assign lane_en_tx_mb = lane_enable_mb & TX_LANE_MASK;
    assign lane_en_tx_sb = lane_enable_i  & TX_LANE_MASK;
    assign lane_en_rx_sb = lane_enable_i[RX_LANE_NUM-1:0] & RX_LANE_MASK[RX_LANE_NUM-1:0];
    assign lane_en_rx_rx = lane_enable_rx & RX_LANE_MASK[RX_LANE_NUM-1:0];

    // 未解析输入（协议占位）：peer retrain 报文内容无需解析（valid 即语义）；
    // rx_event_status_i 链路层回送状态本端不解析（MAS §2.1）
    wire                  unused_ok = &{1'b0, peer_retrain_pkt_i, rx_event_status_i};

    integer               mi;   // mb_clk 聚合块循环变量
    integer               ti;   // compl toggle 块循环变量

    // ==========================================================================
    // 子模块实例化
    // ==========================================================================

    assign phy_rx_clk_w[0] = phy_rx_clk0;
    assign phy_rx_clk_w[1] = phy_rx_clk1;
    // lanes 2~3 无专用 RX 时钟（blueprint 仅 phy_rx_clk0/1）：复接可用时钟作 CDC 端点；
    // 4T1R 下 RX_LANE_MASK=4'b0001 仅使能 lane0，lanes 1~3 RX 数据被方向掩码剔除、无消费
    assign phy_rx_clk_w[2] = phy_rx_clk1;
    assign phy_rx_clk_w[3] = phy_rx_clk1;

    genvar gs, gl;
    generate
        // --------------------------------------------------------------------------
        // u_sync_txreq：req toggle 接收端（sb_tx_clk 域，2flop + 边沿检测）
        // u_sync_txcompl：complete toggle 接收端（mb_clk 域，2flop + 边沿检测）
        // --------------------------------------------------------------------------
        for (gs = 0; gs < 7; gs = gs + 1) begin : gen_step_sync
            sync_cell u_sync_txreq (
                .clk_d  (sb_tx_clk),
                .rstn_d (sb_rst_n),
                .data_s (req_toggle[gs]),
                .data_d (req_sync2[gs])
            );
            assign req_edge[gs] = req_sync2[gs] ^ req_sync2_d[gs];

            sync_cell u_sync_txcompl (
                .clk_d  (mb_clk),
                .rstn_d (mb_rstn_sync),
                .data_s (compl_toggle[gs]),
                .data_d (cpl_sync2[gs])
            );
            assign compl_edge[gs] = cpl_sync2[gs] ^ cpl_sync2_d[gs];
        end

        // --------------------------------------------------------------------------
        // gen_lane_tx：TX 方向 per-lane（LANE_NUM=4，mb_clk / sb_tx_clk 域）
        //   u_lane_en_mb：lane_enable -> mb_clk（quasi-static）
        //   u_sync_ack：train_complete_ack_i level 同步（TX 方向，sb_tx_clk 域）
        // --------------------------------------------------------------------------
        for (gl = 0; gl < LANE_NUM; gl = gl + 1) begin : gen_lane_tx
            sync_cell u_lane_en_mb (
                .clk_d  (mb_clk),
                .rstn_d (mb_rstn_sync),
                .data_s (lane_enable_i[gl]),
                .data_d (lane_enable_mb[gl])
            );

            sync_cell u_sync_ack (
                .clk_d  (sb_tx_clk),
                .rstn_d (sb_rst_n),
                .data_s (train_complete_ack_i[gl]),
                .data_d (ack_sync2[gl])
            );
        end

        // --------------------------------------------------------------------------
        // gen_lane_rx：RX 方向 per-lane（容量 RX_LANE_NUM=LANE_NUM=4，phy_rx_clk{i} 域）
        //   u_lane_en_rx：lane_enable -> phy_rx_clk{i}（quasi-static）
        //   u_sync_rx_train：RX train_req toggle 回收（sb_tx_clk 域）
        //   u_sync_rxretrain / u_sync_complack：rx_retrain / compl_ack toggle 接收端
        //   u_rx_train_toggle：RX train_req 发送 toggle（phy_rx_clk{i} 域）
        //   RX bundle 高位（LANE_NUM-1..RX_LANE_NUM）为 RX 未例化 lane，恒 0
        // --------------------------------------------------------------------------
        for (gl = 0; gl < RX_EN_LANE_NUM; gl = gl + 1) begin : gen_lane_rx
            reg  rx_train_toggle_l;
            reg  rxr_sync2_d_l;
            reg  cak_sync2_d_l;
            wire rxr_edge_l;
            wire cak_edge_l;

            sync_cell u_lane_en_rx (
                .clk_d  (phy_rx_clk_w[gl]),
                .rstn_d (rx_rstn_sync[gl]),
                .data_s (lane_enable_i[gl]),
                .data_d (lane_enable_rx[gl])
            );
            sync_cell u_sync_rx_train (
                .clk_d  (sb_tx_clk),
                .rstn_d (sb_rst_n),
                .data_s (rx_train_toggle_l),
                .data_d (rxt_sync2[gl])
            );
            assign rxt_edge[gl] = rxt_sync2[gl] ^ rxt_sync2_d[gl];

            sync_cell u_sync_rxretrain (
                .clk_d  (phy_rx_clk_w[gl]),
                .rstn_d (rx_rstn_sync[gl]),
                .data_s (rx_retrain_toggle),
                .data_d (rxr_sync2[gl])
            );
            assign rxr_edge_l = rxr_sync2[gl] ^ rxr_sync2_d_l;

            sync_cell u_sync_complack (
                .clk_d  (phy_rx_clk_w[gl]),
                .rstn_d (rx_rstn_sync[gl]),
                .data_s (compl_ack_toggle),
                .data_d (cak_sync2[gl])
            );
            assign cak_edge_l = cak_sync2[gl] ^ cak_sync2_d_l;

            // RX train_req 发送 toggle（train_req_i[l] 脉冲翻转）
            always @(posedge phy_rx_clk_w[gl] or negedge rx_rstn_sync[gl]) begin
                if (!rx_rstn_sync[gl]) begin
                    rx_train_toggle_l <= 1'b0;
                end
                else if (train_req_i[gl]) begin
                    rx_train_toggle_l <= ~rx_train_toggle_l;
                end
            end

            // rx_retrain 边沿检测延迟寄存器
            always @(posedge phy_rx_clk_w[gl] or negedge rx_rstn_sync[gl]) begin
                if (!rx_rstn_sync[gl]) begin
                    rxr_sync2_d_l <= 1'b0;
                end
                else begin
                    rxr_sync2_d_l <= rxr_sync2[gl];
                end
            end

            // compl_ack 边沿检测延迟寄存器
            always @(posedge phy_rx_clk_w[gl] or negedge rx_rstn_sync[gl]) begin
                if (!rx_rstn_sync[gl]) begin
                    cak_sync2_d_l <= 1'b0;
                end
                else begin
                    cak_sync2_d_l <= cak_sync2[gl];
                end
            end

            // rx_retrain / train_complete_ack per-lane 输出（1 cycle 脉冲）
            assign rx_retrain_req_o[gl]     = rxr_edge_l & lane_en_rx_rx[gl];
            assign train_complete_ack_o[gl] = cak_edge_l & lane_en_rx_rx[gl];
        end

        // RX bundle 高位（lanes >= RX_EN_LANE_NUM，无 phy_rx_clk）：RX 未例化，恒 0；
        // 残留向量 rxt_sync2/rxt_edge/lane_enable_rx 高位 tie 0（gen_lane_rx 收窄后不再驱动，
        // 避免经 L589/792/807/L318 传入 undriven/Z；位段与 gen_lane_rx 驱动段不重叠）
        for (gl = RX_EN_LANE_NUM; gl < LANE_NUM; gl = gl + 1) begin : gen_rx_tieoff
            assign rxt_sync2[gl]       = 1'b0;
            assign rxt_edge[gl]        = 1'b0;
            assign lane_enable_rx[gl]  = 1'b0;
            assign rx_retrain_req_o[gl]     = 1'b0;
            assign train_complete_ack_o[gl] = 1'b0;
        end
    endgenerate

    // --------------------------------------------------------------------------
    // u_sync_ctrl_mb：train / retrain toggle 接收端（mb_clk 域）
    // --------------------------------------------------------------------------
    sync_cell u_sync_train (
        .clk_d  (mb_clk),
        .rstn_d (mb_rstn_sync),
        .data_s (train_toggle),
        .data_d (train_sync2)
    );
    assign train_edge = train_sync2 ^ train_sync2_d;

    sync_cell u_sync_retrain (
        .clk_d  (mb_clk),
        .rstn_d (mb_rstn_sync),
        .data_s (retrain_toggle),
        .data_d (retrain_sync2)
    );
    assign retrain_edge = retrain_sync2 ^ retrain_sync2_d;

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // u_txevt_decode：EventCmd {Type, opcode} 提取 + RETRAIN 事件判定
    // --------------------------------------------------------------------------
    assign cmd_type   = tx_event_cmd_i[15];
    assign cmd_opcode = tx_event_cmd_i[7:0];

    assign cpif_retrain_evt = (tx_cur_state == TX_IDLE)
                            && tx_event_req_i && tx_event_enable_i
                            && (cmd_type == 1'b0) && (cmd_opcode == OPC_RETRAIN);

    // retrain 两源 OR 后单点翻转（同拍双源只翻 1 次，retrain 幂等不丢语义）
    assign retrain_evt = cpif_retrain_evt | peer_retrain_pkt_valid_i;

    // --------------------------------------------------------------------------
    // tx_event_fsm 下一状态逻辑
    // --------------------------------------------------------------------------
    always @(*) begin
        tx_next_state = tx_cur_state;
        case (tx_cur_state)
            TX_IDLE: begin
                if (tx_event_req_i && tx_event_enable_i) begin
                    if (cmd_type == 1'b1) begin
                        tx_next_state = TX_SEND;   // SBC 转发类
                    end
                    else begin
                        tx_next_state = TX_ACK;    // 本地动作类
                    end
                end
            end
            TX_SEND: begin
                if (pkt_to_disp_ready_i) begin
                    tx_next_state = TX_ACK;
                end
            end
            TX_ACK: begin
                tx_next_state = TX_IDLE;
            end
            default: begin
                tx_next_state = TX_IDLE;           // 未用编码自恢复
            end
        endcase
    end

    // --------------------------------------------------------------------------
    // rx_event_fsm 下一状态逻辑
    // --------------------------------------------------------------------------
    assign slot_any = slot_retrain | slot_pass | slot_trainc;

    always @(*) begin
        rx_next_state = rx_cur_state;
        case (rx_cur_state)
            RX_IDLE: begin
                if (rx_event_enable_i && slot_any) begin
                    rx_next_state = RX_REQ;
                end
            end
            RX_REQ: begin
                rx_next_state = RX_WAIT_ACK;
            end
            RX_WAIT_ACK: begin
                if (rx_event_ack_i) begin
                    rx_next_state = RX_IDLE;
                end
            end
            default: begin
                rx_next_state = RX_IDLE;           // 未用编码自恢复
            end
        endcase
    end

    assign rx_serve_start = (rx_cur_state == RX_IDLE) && rx_event_enable_i && slot_any;
    assign rx_serve_done  = (rx_cur_state == RX_WAIT_ACK) && rx_event_ack_i;

    // --------------------------------------------------------------------------
    // u_rxevt_arb：RX event 固定优先级选择（retrain > passthru > train_complete）
    // --------------------------------------------------------------------------
    always @(*) begin
        if (slot_retrain) begin
            sel_n = SEL_RETRAIN;
            cmd_n = {1'b1, 7'b0000000, OPC_RETRAIN};
        end
        else if (slot_pass) begin
            sel_n = SEL_PASS;
            cmd_n = pass_cmd_hold;
        end
        else begin
            sel_n = SEL_TRAINC;
            cmd_n = {1'b0, 7'b0000000, OPC_RX_READY};
        end
    end

    assign rx_event_cmd_o = rx_event_cmd_hold;

    // --------------------------------------------------------------------------
    // RX train_req AND 聚合判定（sb_tx_clk 域，容量 RX_LANE_NUM，含本拍新到边沿）
    // --------------------------------------------------------------------------
    assign rx_train_arrived = rx_train_pend | rxt_edge;
    assign rx_train_agg     = ((rx_train_arrived & lane_en_rx_sb) == lane_en_rx_sb)
                            && (lane_en_rx_sb != {RX_LANE_NUM{1'b0}});

    // --------------------------------------------------------------------------
    // u_send_arb：pkt_to_disp 二源仲裁（SEND 优先于 rx_train encode），
    // pkt_to_disp_ready_i=0 时冻结等待
    // --------------------------------------------------------------------------
    assign send_fire     = (tx_cur_state == TX_SEND) && pkt_to_disp_ready_i;
    assign rx_train_fire = (tx_cur_state != TX_SEND) && rx_train_enc_pend
                         && pkt_to_disp_ready_i;

    assign pkt_to_disp_valid_o = send_fire | rx_train_fire;
    assign pkt_to_disp_o       = (tx_cur_state == TX_SEND)
                               ? pkt_enc_hold
                               : {1'b0, 4'b0000, OPC_RETRAIN, SBC_TYPE_REQ};

    // --------------------------------------------------------------------------
    // u_txreq_agg：TX per-step req AND 聚合判定（mb_clk 域，LANE_NUM=4，含本拍新到）
    // --------------------------------------------------------------------------
    assign step_req_in[0] = offset_req_i;
    assign step_req_in[1] = vref_req_i;
    assign step_req_in[2] = eq_req_i;
    assign step_req_in[3] = pi_req_i;
    assign step_req_in[4] = dcDL_req_i;
    assign step_req_in[5] = reorder_req_i;
    assign step_req_in[6] = data_check_req_i;

    generate
        for (gs = 0; gs < 7; gs = gs + 1) begin : gen_req_agg
            assign req_agg_done[gs] = (((req_pend[gs] | step_req_in[gs]) & lane_en_tx_mb)
                                       == lane_en_tx_mb)
                                    && (lane_en_tx_mb != {LANE_NUM{1'b0}});
        end
    endgenerate

    // per-link req 输出（sb_tx_clk 域边沿检测 1 cycle 脉冲 -> dispatcher encode）
    assign offset_req_o     = req_edge[0];
    assign vref_req_o       = req_edge[1];
    assign eq_req_o         = req_edge[2];
    assign pi_req_o         = req_edge[3];
    assign dcDL_req_o       = req_edge[4];
    assign reorder_req_o    = req_edge[5];
    assign data_check_req_o = req_edge[6];

    // --------------------------------------------------------------------------
    // TX per-step complete fanout（mb_clk 域，per-lane 脉冲 & lane_en_tx_mb，LANE_NUM=4）
    // --------------------------------------------------------------------------
    assign step_compl_in = {data_check_complete_i, reorder_complete_i, dcDL_complete_i,
                            pi_complete_i, eq_complete_i, vref_complete_i, offset_complete_i};

    assign offset_complete_o     = {LANE_NUM{compl_edge[0]}} & lane_en_tx_mb;
    assign vref_complete_o       = {LANE_NUM{compl_edge[1]}} & lane_en_tx_mb;
    assign eq_complete_o         = {LANE_NUM{compl_edge[2]}} & lane_en_tx_mb;
    assign pi_complete_o         = {LANE_NUM{compl_edge[3]}} & lane_en_tx_mb;
    assign dcDL_complete_o       = {LANE_NUM{compl_edge[4]}} & lane_en_tx_mb;
    assign reorder_complete_o    = {LANE_NUM{compl_edge[5]}} & lane_en_tx_mb;
    assign data_check_complete_o = {LANE_NUM{compl_edge[6]}} & lane_en_tx_mb;

    // train / retrain per-lane fanout（mb_clk 域，LANE_NUM=4）
    assign train_req_o   = {LANE_NUM{train_edge}}   & lane_en_tx_mb;
    assign retrain_req_o = {LANE_NUM{retrain_edge}} & lane_en_tx_mb;

    // --------------------------------------------------------------------------
    // u_ack_agg：ack AND 聚合 + 上升沿检测（sb_tx_clk 域，TX 方向 LANE_NUM=4）
    // 一次训练完成只置 1 次 slot_trainc；level 回落（重训）后重新武装
    // --------------------------------------------------------------------------
    assign ack_all  = (&(ack_sync2 | ~lane_en_tx_sb))
                    && (lane_en_tx_sb != {LANE_NUM{1'b0}});
    assign ack_rise = ack_all && !ack_all_d;

    assign tx_event_status_o = status_hold;

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // FSM 状态寄存器
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            tx_cur_state <= TX_IDLE;
        end
        else begin
            tx_cur_state <= tx_next_state;
        end
    end

    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            rx_cur_state <= RX_IDLE;
        end
        else begin
            rx_cur_state <= rx_next_state;
        end
    end

    // --------------------------------------------------------------------------
    // FSM 输出逻辑（时序，Moore 等效）：ack / req 仅在对应状态拉高 1 cycle
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            tx_event_ack_o <= 1'b0;
        end
        else if (tx_next_state == TX_ACK) begin
            tx_event_ack_o <= 1'b1;
        end
        else begin
            tx_event_ack_o <= 1'b0;
        end
    end

    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            rx_event_req_o <= 1'b0;
        end
        else if (rx_next_state == RX_REQ) begin
            rx_event_req_o <= 1'b1;
        end
        else begin
            rx_event_req_o <= 1'b0;
        end
    end

    // --------------------------------------------------------------------------
    // status / encode 报文锁存（IDLE 当拍）
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            status_hold <= STATUS_OK;
        end
        else if ((tx_cur_state == TX_IDLE) && tx_event_req_i && tx_event_enable_i) begin
            if (cmd_type == 1'b0) begin
                if (cmd_opcode == OPC_RETRAIN) begin
                    status_hold <= STATUS_OK;
                end
                else begin
                    status_hold <= STATUS_UNSUP;
                end
            end
            else begin
                status_hold <= STATUS_OK;
            end
        end
    end

    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            pkt_enc_hold <= 16'b0;
        end
        else if ((tx_cur_state == TX_IDLE) && tx_event_req_i && tx_event_enable_i
                 && (cmd_type == 1'b1)) begin
            if (cmd_opcode == OPC_RETRAIN) begin
                pkt_enc_hold <= {1'b0, 4'b0000, OPC_RETRAIN, SBC_TYPE_REQ};
            end
            else begin
                pkt_enc_hold <= {1'b0, 4'b0000, cmd_opcode, SBC_TYPE_PASS};
            end
        end
    end

    // --------------------------------------------------------------------------
    // u_ctrl_toggle：train / retrain / rx_retrain / compl_ack 发送 toggle
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            train_toggle <= 1'b0;
        end
        else if (cpif_retrain_evt) begin
            train_toggle <= ~train_toggle;
        end
    end

    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            retrain_toggle    <= 1'b0;
            rx_retrain_toggle <= 1'b0;
        end
        else if (retrain_evt) begin
            retrain_toggle    <= ~retrain_toggle;
            rx_retrain_toggle <= ~rx_retrain_toggle;
        end
    end

    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            compl_ack_toggle <= 1'b0;
        end
        else if (rx_data_check_agg_i) begin
            compl_ack_toggle <= ~compl_ack_toggle;
        end
    end

    // --------------------------------------------------------------------------
    // RX train_req 回收：边沿检测延迟寄存器 + pending + AND 聚合
    // 聚合成 1 次 encode 请求（retrain 报文触发 peer TX 训练），容量 RX_LANE_NUM
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            rxt_sync2_d <= {RX_LANE_NUM{1'b0}};
        end
        else begin
            rxt_sync2_d <= rxt_sync2;
        end
    end

    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            rx_train_pend     <= {RX_LANE_NUM{1'b0}};
            rx_train_enc_pend <= 1'b0;
        end
        else begin
            if (rx_train_agg) begin
                rx_train_pend     <= {RX_LANE_NUM{1'b0}};
                rx_train_enc_pend <= 1'b1;
            end
            else begin
                rx_train_pend <= rx_train_arrived;
                if (rx_train_fire) begin
                    rx_train_enc_pend <= 1'b0;
                end
            end
        end
    end

    // --------------------------------------------------------------------------
    // u_txreq_pend / u_txreq_toggle：TX per-step req per-lane pending +
    // AND 聚合 + 发送 toggle（mb_clk 域，LANE_NUM=4，7×4=28 FF）
    // --------------------------------------------------------------------------
    always @(posedge mb_clk or negedge mb_rstn_sync) begin
        if (!mb_rstn_sync) begin
            for (mi = 0; mi < 7; mi = mi + 1) begin
                req_pend[mi] <= {LANE_NUM{1'b0}};
            end
            req_toggle <= 7'b0;
        end
        else begin
            for (mi = 0; mi < 7; mi = mi + 1) begin
                if (req_agg_done[mi]) begin
                    req_pend[mi]   <= {LANE_NUM{1'b0}};
                    req_toggle[mi] <= ~req_toggle[mi];
                end
                else begin
                    req_pend[mi] <= req_pend[mi] | step_req_in[mi];
                end
            end
        end
    end

    // req 边沿检测延迟寄存器（sb_tx_clk 域）
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            req_sync2_d <= 7'b0;
        end
        else begin
            req_sync2_d <= req_sync2;
        end
    end

    // --------------------------------------------------------------------------
    // u_txcompl_toggle：TX per-step complete 发送 toggle（sb_tx_clk 域）
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            compl_toggle <= 7'b0;
        end
        else begin
            for (ti = 0; ti < 7; ti = ti + 1) begin
                if (step_compl_in[ti]) begin
                    compl_toggle[ti] <= ~compl_toggle[ti];
                end
            end
        end
    end

    // complete 边沿检测延迟寄存器（mb_clk 域）
    always @(posedge mb_clk or negedge mb_rstn_sync) begin
        if (!mb_rstn_sync) begin
            cpl_sync2_d <= 7'b0;
        end
        else begin
            cpl_sync2_d <= cpl_sync2;
        end
    end

    // train / retrain 边沿检测延迟寄存器（mb_clk 域）
    always @(posedge mb_clk or negedge mb_rstn_sync) begin
        if (!mb_rstn_sync) begin
            train_sync2_d   <= 1'b0;
            retrain_sync2_d <= 1'b0;
        end
        else begin
            train_sync2_d   <= train_sync2;
            retrain_sync2_d <= retrain_sync2;
        end
    end

    // --------------------------------------------------------------------------
    // u_ack_all_d：ack_all 延迟 1 拍（上升沿检测用）
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            ack_all_d <= 1'b0;
        end
        else begin
            ack_all_d <= ack_all;
        end
    end

    // --------------------------------------------------------------------------
    // u_rxevt_slot：3 类 RX event slot + 透传 cmd 锁存 + 溢出 sticky
    // 事件置位优先于 ack 清除（事件不丢）
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            slot_retrain <= 1'b0;
        end
        else if (peer_retrain_pkt_valid_i) begin
            slot_retrain <= 1'b1;
        end
        else if (rx_serve_done && (serve_sel == SEL_RETRAIN)) begin
            slot_retrain <= 1'b0;
        end
    end

    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            slot_pass     <= 1'b0;
            pass_cmd_hold <= 16'b0;
            ovf_flag      <= 1'b0;
        end
        else begin
            if (link_passthru_valid_i) begin
                // slot 已挂起又到达新透传报文：置溢出 sticky，新报文覆盖 cmd
                if (slot_pass) begin
                    ovf_flag <= 1'b1;
                end
                slot_pass     <= 1'b1;
                pass_cmd_hold <= {1'b1, 7'b0000000, link_passthru_i[10:3]};
            end
            else if (rx_serve_done && (serve_sel == SEL_PASS)) begin
                slot_pass <= 1'b0;
            end
        end
    end

    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            slot_trainc <= 1'b0;
        end
        else if (ack_rise) begin
            slot_trainc <= 1'b1;
        end
        else if (rx_serve_done && (serve_sel == SEL_TRAINC)) begin
            slot_trainc <= 1'b0;
        end
    end

    // 被服务 slot 编号与 cmd 锁存（REQ/WAIT_ACK 期间 cmd 保持稳定）
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            serve_sel         <= SEL_RETRAIN;
            rx_event_cmd_hold <= 16'b0;
        end
        else if (rx_serve_start) begin
            serve_sel         <= sel_n;
            rx_event_cmd_hold <= cmd_n;
        end
    end

endmodule
