// =============================================================================
// module name：sbc_dispatcher
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/sideband/mas/sbc_dispatcher_mas.md
// =============================================================================
// 功能概述: SBC 报文分发器（per-link 唯一）：16 bit 报文 type/opcode/lid decode 分发 +
//   per-step SBC encode + 15 源固定优先级仲裁 + 错误 sticky / 中断，无 FSM
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-07  OpenHappy Team  初始版本：基于 sbc_dispatcher MAS v1.4 生成
//  2026-08-11  OpenHappy Team  RX_LANE_MASK 编译期方向掩码（per-lane 方向不对称例化
//                       适配，MAS v1.5）：RX 方向 fanout 目标 / complete AND 聚合
//                       有效使能 = lane_enable_i & RX_LANE_MASK
//  2026-08-21  OpenHappy Team  一致性评审必改项 #2：pkt_to_tx_valid_o 由无条件 any_pend
//                       改为 ready 门控（any_pend & pkt_to_tx_ready_i），消除 sbc_tx
//                       将 valid&buf_full 判 ovf_detect 误停机风险；数据/选择仍按
//                       out_lock 冻结语义保持（MAS v1.7）
//  2026-08-28  OpenHappy Team  4T1R（4 TX + 1 RX）扩容（MAS v1.8）：LANE_NUM 硬件 2→4，
//                       RX 方向 per-lane bundle 全部扩到 4（7×4）；RX per-lane CDC 容量
//                       RX_LANE_NUM=LANE_NUM=4（对齐 blueprint internals u_req_toggle=28 /
//                       u_compl_toggle=28 等按 4 LANE 预留）；物理仅 phy_rx_clk0/1 两路 RX 时钟，
//                       lanes 2~3 时钟复接，RX_LANE_MASK 仅 bit0（4T1R 单 RX）故高位剔除无消费
//                       RX bundle 高位（RX 未例化）恒 0
//  2026-09-09  OpenHappy Team  RX 例化收窄（RTL-FIX-LPCLK-02）：gen_rx_lane 按 RX_EN_LANE_NUM
//                       （=RX_LANE_MASK popcount，4T1R=1/full=4）仅对 RX 使能 lane 实例化 RX CDC flop，
//                       去除 TX-only lane 的 tie-low 时钟死 flop（FlopClockConstant）；残留向量
//                       compl_sync2/req_sync2 高位及数 req 输出在 gen_rx_req_tieoff 恒 0；RX bundle
//                       位宽仍按 RX_LANE_NUM=4 保持（接口零变更）。注：popcount 隐含 RX lane 低位连续，
//                       未来非连续 RX 掩码需改 per-lane 条件门控
//  2026-09-10  OpenHappy Team  内部 CDC bundle 容量收窄（FIX-REQTOGGLE-01）：req_toggle/req_sync2/compl_in/
//                       compl_sync2/compl_sync2_d/compl_edge/compl_pend/dec_target/lane_en_rx 容量基准
//                       RX_LANE_NUM(=4) → RX_EN_LANE_NUM(=RX_LANE_MASK popcount，4T1R=1)，req_toggle
//                       [3:1] 死位结构消隐（FlopEConst 结构性收敛）；per-lane I/O 端口仍 [3:0]（零接口变更）；
//                       同步移除 gen_rx_req_tieoff 残留向量 tie（收窄后无越界高位）；多 lane 配置行为逐位一致
//  2026-09-18  OpenHappy Team  BUG 修复：RX_EN_LANE_NUM popcount 加数扩展 3 bit——原 1-bit 加数链式
//                       相加（RX_LANE_MASK[i] 为 1-bit 位选，Verilog 链式加法宽度 = 最大操作数
//                       宽度），mask=4'b1111（st 4R1T）时 ((((1+1)+1)+1) 回绕为 0 →
//                       [(RX_EN_LANE_NUM-1):0]=[-1:0] → STX_VE_800（Array bound exceeds
//                       integer range）/ STX_VE_491（part-select bounds reversed）；4T1R
//                       基线（mask=4'b0001）恰无溢出而幸免。sbc_cpif_event.v 同修（同日）
// =============================================================================

`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// 设计说明:
//
// 时钟域:
//   - sb_tx_clk：主域（decode / encode / 仲裁 / 聚合 / sticky）
//   - phy_rx_clk{0/1}：仅 RX 方向 toggle-handshake CDC 端点（内部容量 RX_EN_LANE_NUM=RX_LANE_MASK
//     popcount，4T1R=1；lanes 2~3 无专用时钟复接，RX_LANE_MASK 仅使能 lane0故高位无消费）
// -----------------------------------------------------------------------------

module sbc_dispatcher #(
    // RX 方向编译期掩码（logic_phy 按 LP_ lane 宏派生，截断到本模块 LANE_NUM=4
    // 位宽）：bit i=1 表示 lane i RX 已例化；RX 方向 fanout / complete 聚合
    // 有效使能 = lane_enable_i & RX_LANE_MASK，软件对未例化 lane 置位无害
    parameter RX_LANE_MASK = 4'b1111
) (
    // -------------------------------------------------------------------------
    // 时钟与复位
    // -------------------------------------------------------------------------
    input  wire        sb_tx_clk,
    input  wire        sb_rst_n,
    input  wire [3:0]  rx_rstn_sync,   // 每 lane phy_rx_clk{i} 域同步复位（CRG；per-lane 位宽 [3:0]=LANE_NUM，同其他 per-lane 端口约定）
    input  wire        phy_rx_clk0,
    input  wire        phy_rx_clk1,

    // -------------------------------------------------------------------------
    // SBC 接收报文输入（sbc_rx，sb_tx_clk 域，valid-only）
    // -------------------------------------------------------------------------
    input  wire [15:0] pkt_from_rx_i,
    input  wire        pkt_from_rx_valid_i,
    input  wire        parity_err_event_i,
    input  wire        gap_err_event_i,

    // -------------------------------------------------------------------------
    // CPIF 直通报文输入（sbc_cpif_event，valid-ready）
    // -------------------------------------------------------------------------
    input  wire [15:0] pkt_from_cpif_i,
    input  wire        pkt_from_cpif_valid_i,
    output wire        pkt_from_cpif_ready_o,

    // -------------------------------------------------------------------------
    // TX per-step req 输入（sbc_cpif_event，per-link 1 cycle 脉冲）
    // -------------------------------------------------------------------------
    input  wire        offset_req_i,
    input  wire        vref_req_i,
    input  wire        eq_req_i,
    input  wire        pi_req_i,
    input  wire        dcDL_req_i,
    input  wire        reorder_req_i,
    input  wire        data_check_req_i,

    // -------------------------------------------------------------------------
    // RX per-step complete 输入（per-lane，phy_rx_clk{i} 域 1 cycle 脉冲，
    // bundle 按 LANE_NUM=4 端口接口，内部 CDC 容量 RX_EN_LANE_NUM（RX_LANE_MASK popcount））
    // -------------------------------------------------------------------------
    input  wire [3:0]  offset_complete_i,
    input  wire [3:0]  vref_complete_i,
    input  wire [3:0]  eq_complete_i,
    input  wire [3:0]  pi_complete_i,
    input  wire [3:0]  dcDL_complete_i,
    input  wire [3:0]  reorder_complete_i,
    input  wire [3:0]  data_check_complete_i,

    // -------------------------------------------------------------------------
    // 配置接口（apb_csr_top，CDC 已完成，sb_tx_clk 域）
    // -------------------------------------------------------------------------
    input  wire [15:0] lid_filter_i,
    input  wire [7:0]  opcode_filter_i,
    input  wire [7:0]  err_mask_i,
    input  wire        pkt_arrival_mask_i,
    input  wire        err_irq_mask_i,
    input  wire [1:0]  irq_clear_i,
    input  wire [3:0]  lane_enable_i,

    // -------------------------------------------------------------------------
    // encode 报文输出（sbc_tx，valid-ready）
    // -------------------------------------------------------------------------
    output wire [15:0] pkt_to_tx_o,
    output wire        pkt_to_tx_valid_o,
    input  wire        pkt_to_tx_ready_i,

    // -------------------------------------------------------------------------
    // 分发输出（sbc_cpif_event，valid-only / per-link 脉冲）
    // -------------------------------------------------------------------------
    output reg  [15:0] sbc_link_passthru_o,
    output reg         sbc_link_passthru_valid_o,
    output reg  [15:0] peer_retrain_pkt_o,
    output reg         peer_retrain_pkt_valid_o,
    output reg         offset_complete_o,
    output reg         vref_complete_o,
    output reg         eq_complete_o,
    output reg         pi_complete_o,
    output reg         dcDL_complete_o,
    output reg         reorder_complete_o,
    output reg         data_check_complete_o,
    output reg         rx_data_check_agg_o,

    // -------------------------------------------------------------------------
    // RX per-step req fanout 输出（per-lane，phy_rx_clk{i} 域 1 cycle 脉冲，
    // bundle 按 LANE_NUM=4，高位 RX 未例化恒 0）
    // -------------------------------------------------------------------------
    output wire [3:0]  offset_req_o,
    output wire [3:0]  vref_req_o,
    output wire [3:0]  eq_req_o,
    output wire [3:0]  pi_req_o,
    output wire [3:0]  dcDL_req_o,
    output wire [3:0]  reorder_req_o,
    output wire [3:0]  data_check_req_o,

    // -------------------------------------------------------------------------
    // 状态回流与中断
    // -------------------------------------------------------------------------
    output reg         pkt_arrival_flag_o,
    output reg         err_flag_o,
    output reg         illegal_opcode_o,
    output reg         illegal_lid_o,
    output wire        irq_pkt_arrival_o,
    output wire        irq_err_o
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    localparam LANE_NUM       = 4;     // 最大 lane 能力（按 4 LANE 预留，4T1R：4 TX + 1 RX）
    localparam RX_LANE_NUM    = LANE_NUM;  // RX 方向 per-lane CDC 接口容量 = LANE_NUM=4（对齐 blueprint
                                       // internals：u_req_toggle/u_compl_toggle/u_compl_pend=28 等按
                                       // 4 LANE 预留）——内部 CDC bundles 容量已由 RX_EN_LANE_NUM（popcount）
                                       // 收窄（FIX-REQTOGGLE-01），RX_LANE_NUM 保留作接口/blueprint 容量语义
    // RX 使能 lane 数 = RX_LANE_MASK popcount（direction 单一事实源，同 L201 消费）；
    // 4T1R=1 / full=4。注：popcount 隐含 RX lane 低位连续；未来非连续 RX 掩码需 per-lane 条件门控。
    // 2026-09-18 修复：加数扩展 3 bit（原 1-bit 加数链加，mask=4'b1111 时回绕为 0 →
    // [(RX_EN_LANE_NUM-1):0]=[-1:0] → STX_VE_800/491；3-bit 和可承载 4 lane = 3'b100）
    localparam [2:0] RX_EN_LANE_NUM =
        {2'b00, RX_LANE_MASK[0]} + {2'b00, RX_LANE_MASK[1]} +
        {2'b00, RX_LANE_MASK[2]} + {2'b00, RX_LANE_MASK[3]};

    // step opcode 表（type=010 为 REQ，type=011 为 COMPLETE，opcode 相同）
    localparam OPC_OFFSET     = 8'h45; // 国标 RX OFFSET
    localparam OPC_VREF       = 8'h46; // 国标 RX VREF
    localparam OPC_EQ         = 8'h4A; // reserved 扩展（占位）
    localparam OPC_PI         = 8'h48; // 国标 RX CDA（PI 训练）
    localparam OPC_DCDL       = 8'h47; // 国标 RX DESKEW（DCDL 训练）
    localparam OPC_REORDER    = 8'h49; // 国标 RX REORDER
    localparam OPC_DATA_CHECK = 8'h4B; // reserved 扩展（占位）
    localparam OPC_RETRAIN    = 8'h80; // retrain

    // U2 指令 opcode（type=010，显式透传分发上送链路层）
    localparam OPC_U2_ENTER_REQ = 8'h84;
    localparam OPC_U2_ACCEPT    = 8'h85;
    localparam OPC_U2_REJECT    = 8'h88;
    localparam OPC_U2_EXIT_REQ  = 8'h89;
    localparam OPC_U2_EXIT_DONE = 8'h8A;

    // SBC 帧 type
    localparam SBC_TYPE_REQ  = 3'b010;
    localparam SBC_TYPE_COMPL= 3'b011;
    localparam SBC_TYPE_PASS = 3'b111;

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================

    // --------------------------------------------------------------------------
    // 报文 decode 组合信号
    // --------------------------------------------------------------------------
    wire [2:0]            dec_type;
    wire [7:0]            dec_opcode;
    wire [3:0]            dec_lid;
    reg  [2:0]            dec_step;         // opcode2step 查表结果
    reg                   dec_step_v;       // step opcode 有效
    reg                   dec_known;        // 已知 opcode 集合
    wire                  dec_is_retrain;
    wire                  dec_is_u2;
    reg  [RX_EN_LANE_NUM-1:0] dec_target;   // fanout 目标 lane 集合（RX 实际使能容量，RX_LANE_MASK popcount）
    reg                   dec_illegal_op;   // 非法 opcode / 保留 type（valid 限定）
    reg                   dec_illegal_lid;  // 非法 lid（valid 限定）

    // RX 方向有效使能 = lane_enable_i & RX_LANE_MASK（编译期方向掩码，容量 RX_EN_LANE_NUM；
    // 剔除未例化 lane；软件对未例化 lane 置位无害）
    wire [RX_EN_LANE_NUM-1:0]   lane_en_rx;

    assign lane_en_rx = lane_enable_i[RX_EN_LANE_NUM-1:0] & RX_LANE_MASK[RX_EN_LANE_NUM-1:0];

    // --------------------------------------------------------------------------
    // RX req fanout：toggle（sb_tx_clk）+ 接收端（phy_rx_clk{i}，容量 RX_EN_LANE_NUM）
    // per-lane 接收端寄存器在 generate 作用域内声明（避免多维数组跨块驱动）
    // --------------------------------------------------------------------------
    reg  [RX_EN_LANE_NUM-1:0] req_toggle [0:6];
    wire [RX_EN_LANE_NUM-1:0] req_sync2  [0:6];
    wire [LANE_NUM-1:0]     phy_rx_clk_w;

    // --------------------------------------------------------------------------
    // RX complete：发送 toggle（phy_rx_clk{i}，generate 局部，容量 RX_EN_LANE_NUM）+
    // 回收（sb_tx_clk）+ AND 聚合
    // --------------------------------------------------------------------------
    wire [RX_EN_LANE_NUM-1:0] compl_in   [0:6];
    wire [RX_EN_LANE_NUM-1:0] compl_sync2  [0:6];
    reg  [RX_EN_LANE_NUM-1:0] compl_sync2_d [0:6];
    wire [RX_EN_LANE_NUM-1:0] compl_edge   [0:6];
    reg  [RX_EN_LANE_NUM-1:0] compl_pend   [0:6];
    wire [6:0]            agg_done;

    // --------------------------------------------------------------------------
    // encode pending / 保持寄存器 / 仲裁
    // --------------------------------------------------------------------------
    wire [6:0]            tx_req_in;
    reg  [6:0]            enc_pend_tx;
    reg  [6:0]            enc_pend_rx;
    reg                   enc_pend_cpif;
    reg  [15:0]           pkt_cpif_hold;
    wire                  any_pend;
    reg                   arb_cpif;         // 仲裁选择：cpif 直通
    reg  [2:0]            arb_step;         // 仲裁选择：step 索引
    reg                   arb_is_rx;        // 仲裁选择：0=tx req / 1=rx complete
    reg                   out_lock;         // 反压期间选择锁定（数据冻结）
    reg                   lock_cpif;
    reg  [2:0]            lock_step;
    reg                   lock_is_rx;
    wire                  f_cpif;
    wire [2:0]            f_step;
    wire                  f_is_rx;
    wire [6:0]            f_sel_tx;
    wire [6:0]            f_sel_rx;
    wire                  enc_fire;
    reg  [7:0]            enc_opcode;

    // --------------------------------------------------------------------------
    // sticky 与中断
    // --------------------------------------------------------------------------
    wire [3:0]            err_evt;
    wire                  err_set;

    integer               di;   // decode 分发块循环变量
    integer               ai;   // complete 聚合块循环变量
    integer               ci;   // complete 边沿检测块循环变量
    integer               ei;   // encode pending 块循环变量

    // ==========================================================================
    // 子模块实例化
    // ==========================================================================

    assign phy_rx_clk_w[0] = phy_rx_clk0;
    assign phy_rx_clk_w[1] = phy_rx_clk1;
    // lanes 2~3 无专用 RX 时钟（blueprint 仅 phy_rx_clk0/1）：复接可用时钟作 CDC 端点；
    // 4T1R 下 RX_LANE_MASK=4'b0001 仅使能 lane0，lanes 1~3 RX 数据被方向掩码剔除、无消费
    assign phy_rx_clk_w[2] = phy_rx_clk1;
    assign phy_rx_clk_w[3] = phy_rx_clk1;

    // --------------------------------------------------------------------------
    // u_sync_req：req toggle 接收端（phy_rx_clk{i} 域，2flop sync_cell + 边沿检测）
    // u_compl_toggle：complete 发送 toggle（phy_rx_clk{i} 域）
    // u_sync_compl：complete toggle 回收（sb_tx_clk 域，2flop + 边沿检测）
    // per-lane 触发器声明在 generate 作用域内（每 lane 独立时钟域），容量 RX_EN_LANE_NUM
    // --------------------------------------------------------------------------
    genvar gs, gl;
    generate
        for (gl = 0; gl < RX_EN_LANE_NUM; gl = gl + 1) begin : gen_rx_lane
            integer li;
            reg  [6:0] req_sync2_d_l;
            wire [6:0] req_edge_l;
            reg  [6:0] compl_toggle_l;

            for (gs = 0; gs < 7; gs = gs + 1) begin : gen_bit_sync
                sync_cell u_sync_req (
                    .clk_d  (phy_rx_clk_w[gl]),
                    .rstn_d (rx_rstn_sync[gl]),   // phy_rx_clk{gl} 域同步复位（CRG，Group C）
                    .data_s (req_toggle[gs][gl]),
                    .data_d (req_sync2[gs][gl])
                );
                assign req_edge_l[gs] = req_sync2[gs][gl] ^ req_sync2_d_l[gs];

                sync_cell u_sync_compl (
                    .clk_d  (sb_tx_clk),
                    .rstn_d (sb_rst_n),
                    .data_s (compl_toggle_l[gs]),
                    .data_d (compl_sync2[gs][gl])
                );
            end

            // req 边沿检测延迟寄存器
            always @(posedge phy_rx_clk_w[gl] or negedge rx_rstn_sync[gl]) begin
                if (!rx_rstn_sync[gl]) begin
                    req_sync2_d_l <= 7'b0;
                end
                else begin
                    for (li = 0; li < 7; li = li + 1) begin
                        req_sync2_d_l[li] <= req_sync2[li][gl];
                    end
                end
            end

            // complete 发送 toggle：<step>_complete_i[l] 脉冲翻转
            always @(posedge phy_rx_clk_w[gl] or negedge rx_rstn_sync[gl]) begin
                if (!rx_rstn_sync[gl]) begin
                    compl_toggle_l <= 7'b0;
                end
                else begin
                    for (li = 0; li < 7; li = li + 1) begin
                        if (compl_in[li][gl]) begin
                            compl_toggle_l[li] <= ~compl_toggle_l[li];
                        end
                    end
                end
            end

            // req fanout 输出（phy_rx_clk{i} 域边沿检测 1 cycle 脉冲）
            assign offset_req_o[gl]     = req_edge_l[0];
            assign vref_req_o[gl]       = req_edge_l[1];
            assign eq_req_o[gl]         = req_edge_l[2];
            assign pi_req_o[gl]         = req_edge_l[3];
            assign dcDL_req_o[gl]       = req_edge_l[4];
            assign reorder_req_o[gl]    = req_edge_l[5];
            assign data_check_req_o[gl] = req_edge_l[6];
        end

        // RX bundle 高位（lanes >= RX_EN_LANE_NUM，无 phy_rx_clk）：RX 未例化，req 输出恒 0；
        // bundles（compl_sync2/req_sync2）容量已收窄到 RX_EN_LANE_NUM，无高位残留需 tie（越界位由收窄消隐；
        // 多 lane 下 RX_EN_LANE_NUM=LANE_NUM 本循环空转，行为逐位一致）
        for (gl = RX_EN_LANE_NUM; gl < LANE_NUM; gl = gl + 1) begin : gen_rx_req_tieoff
            assign offset_req_o[gl]     = 1'b0;
            assign vref_req_o[gl]       = 1'b0;
            assign eq_req_o[gl]         = 1'b0;
            assign pi_req_o[gl]         = 1'b0;
            assign dcDL_req_o[gl]       = 1'b0;
            assign reorder_req_o[gl]    = 1'b0;
            assign data_check_req_o[gl] = 1'b0;
        end
    endgenerate

    // complete 边沿检测（sb_tx_clk 域）
    generate
        for (gs = 0; gs < 7; gs = gs + 1) begin : gen_compl_edge
            assign compl_edge[gs] = compl_sync2[gs] ^ compl_sync2_d[gs];
        end
    endgenerate

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // u_decode：type/opcode/lid 提取 + opcode2step 查表 + 已知集合判定
    // --------------------------------------------------------------------------
    assign dec_type   = pkt_from_rx_i[2:0];
    assign dec_opcode = pkt_from_rx_i[10:3];
    assign dec_lid    = pkt_from_rx_i[14:11];

    assign dec_is_retrain = (dec_opcode == OPC_RETRAIN);
    assign dec_is_u2      = (dec_opcode == OPC_U2_ENTER_REQ)
                          | (dec_opcode == OPC_U2_ACCEPT)
                          | (dec_opcode == OPC_U2_REJECT)
                          | (dec_opcode == OPC_U2_EXIT_REQ)
                          | (dec_opcode == OPC_U2_EXIT_DONE);

    // opcode2step：step opcode → 0~6，非 step opcode 无效
    always @(*) begin
        dec_step   = 3'd0;
        dec_step_v = 1'b0;
        case (dec_opcode)
            OPC_OFFSET:     begin dec_step = 3'd0; dec_step_v = 1'b1; end
            OPC_VREF:       begin dec_step = 3'd1; dec_step_v = 1'b1; end
            OPC_EQ:         begin dec_step = 3'd2; dec_step_v = 1'b1; end
            OPC_PI:         begin dec_step = 3'd3; dec_step_v = 1'b1; end
            OPC_DCDL:       begin dec_step = 3'd4; dec_step_v = 1'b1; end
            OPC_REORDER:    begin dec_step = 3'd5; dec_step_v = 1'b1; end
            OPC_DATA_CHECK: begin dec_step = 3'd6; dec_step_v = 1'b1; end
            default:        begin dec_step = 3'd0; dec_step_v = 1'b0; end
        endcase
    end

    // 已知 opcode 集合（非法判定用）：0x40~0x4B 国标训练类 + 扩展 +
    // 0x7F RX READY + 0x80 RETRAIN + 0x81 deskew done + U2 + 0xFF SBC-SYNC-IND
    always @(*) begin
        dec_known = 1'b0;
        if ((dec_opcode >= 8'h40) && (dec_opcode <= 8'h4B)) begin
            dec_known = 1'b1;
        end
        else begin
            case (dec_opcode)
                8'h7F, 8'h80, 8'h81,
                8'h84, 8'h85, 8'h88, 8'h89, 8'h8A,
                8'hFF:    dec_known = 1'b1;
                default:  dec_known = 1'b0;
            endcase
        end
    end

    // fanout 目标 lane 集合：lid=0 广播；lid=i 单播（1-based，lane i-1）
    // lid > LANE_NUM 为非法（dec_illegal_lid 判定，此处 target 不用）
    // 目标 lane 上限 = RX_EN_LANE_NUM（仅这些 lane 有 phy_rx_clk，RX 已例化）
    always @(*) begin
        dec_target = {RX_EN_LANE_NUM{1'b0}};
        if (dec_lid == 4'd0) begin
            dec_target = lane_en_rx;
        end
        else if (dec_lid >= 4'd1 && dec_lid <= RX_EN_LANE_NUM) begin
            dec_target = lane_en_rx & (1 << (dec_lid - 4'd1));
        end
        // lid 在 (RX_EN_LANE_NUM, LANE_NUM]：RX 未例化 lane，target 恒 0（合法但不分发）
    end

    // 非法判定（pkt_from_rx_valid_i 当拍限定）
    always @(*) begin
        dec_illegal_op  = 1'b0;
        dec_illegal_lid = 1'b0;
        if (pkt_from_rx_valid_i) begin
            case (dec_type)
                SBC_TYPE_REQ, SBC_TYPE_COMPL: begin
                    if (dec_step_v) begin
                        // lid > LANE_NUM(=4) 非法（仅训练类报文判定）
                        if (dec_lid > 4'd4) begin
                            dec_illegal_lid = 1'b1;
                        end
                    end
                    else if (dec_type == SBC_TYPE_REQ && (dec_is_retrain || dec_is_u2)) begin
                        dec_illegal_op = 1'b0;
                    end
                    else if (dec_known) begin
                        dec_illegal_op = 1'b0;
                    end
                    else begin
                        dec_illegal_op = 1'b1;
                    end
                end
                SBC_TYPE_PASS: begin
                    dec_illegal_op = 1'b0;
                end
                default: begin
                    dec_illegal_op = 1'b1;   // 保留 type
                end
            endcase
        end
    end

    // --------------------------------------------------------------------------
    // u_aggregate：RX complete AND 聚合判定（含本拍新到边沿，容量 RX_EN_LANE_NUM）
    // --------------------------------------------------------------------------
    generate
        for (gs = 0; gs < 7; gs = gs + 1) begin : gen_agg
            assign agg_done[gs] = (((compl_pend[gs] | compl_edge[gs]) & lane_en_rx)
                                   == lane_en_rx)
                                && (lane_en_rx != {RX_EN_LANE_NUM{1'b0}});
        end
    endgenerate

    // --------------------------------------------------------------------------
    // encode pending 采集输入映射
    // --------------------------------------------------------------------------
    assign tx_req_in  = {data_check_req_i, reorder_req_i, dcDL_req_i, pi_req_i,
                         eq_req_i, vref_req_i, offset_req_i};
    assign compl_in[0] = offset_complete_i[RX_EN_LANE_NUM-1:0];
    assign compl_in[1] = vref_complete_i[RX_EN_LANE_NUM-1:0];
    assign compl_in[2] = eq_complete_i[RX_EN_LANE_NUM-1:0];
    assign compl_in[3] = pi_complete_i[RX_EN_LANE_NUM-1:0];
    assign compl_in[4] = dcDL_complete_i[RX_EN_LANE_NUM-1:0];
    assign compl_in[5] = reorder_complete_i[RX_EN_LANE_NUM-1:0];
    assign compl_in[6] = data_check_complete_i[RX_EN_LANE_NUM-1:0];

    // cpif 报文保持寄存器空指示（cpif_event 仅在 ready=1 时发 valid）
    assign pkt_from_cpif_ready_o = ~enc_pend_cpif;

    // --------------------------------------------------------------------------
    // u_arbiter：固定优先级仲裁（cpif > tx_req[0..6] > rx_complete[0..6]）
    // --------------------------------------------------------------------------
    assign any_pend = enc_pend_cpif | (|enc_pend_tx) | (|enc_pend_rx);

    always @(*) begin
        arb_cpif  = 1'b0;
        arb_step  = 3'd0;
        arb_is_rx = 1'b0;
        if (enc_pend_cpif) begin
            arb_cpif = 1'b1;
        end
        else if (enc_pend_tx[0]) begin arb_step = 3'd0; end
        else if (enc_pend_tx[1]) begin arb_step = 3'd1; end
        else if (enc_pend_tx[2]) begin arb_step = 3'd2; end
        else if (enc_pend_tx[3]) begin arb_step = 3'd3; end
        else if (enc_pend_tx[4]) begin arb_step = 3'd4; end
        else if (enc_pend_tx[5]) begin arb_step = 3'd5; end
        else if (enc_pend_tx[6]) begin arb_step = 3'd6; end
        else if (enc_pend_rx[0]) begin arb_step = 3'd0; arb_is_rx = 1'b1; end
        else if (enc_pend_rx[1]) begin arb_step = 3'd1; arb_is_rx = 1'b1; end
        else if (enc_pend_rx[2]) begin arb_step = 3'd2; arb_is_rx = 1'b1; end
        else if (enc_pend_rx[3]) begin arb_step = 3'd3; arb_is_rx = 1'b1; end
        else if (enc_pend_rx[4]) begin arb_step = 3'd4; arb_is_rx = 1'b1; end
        else if (enc_pend_rx[5]) begin arb_step = 3'd5; arb_is_rx = 1'b1; end
        else if (enc_pend_rx[6]) begin arb_step = 3'd6; arb_is_rx = 1'b1; end
    end

    // 反压期间选择锁定：pkt_to_tx_ready_i=0 时 valid 拉低（ready 门控，
    // 符合 sbc_tx 对 valid&~ready 判 ovf_detect 的 gated-producer 契约），
    // 数据/选择经 out_lock 冻结、pending 保持
    assign f_cpif   = out_lock ? lock_cpif  : arb_cpif;
    assign f_step   = out_lock ? lock_step  : arb_step;
    assign f_is_rx  = out_lock ? lock_is_rx : arb_is_rx;
    assign f_sel_tx = (!f_cpif && !f_is_rx) ? (7'b0000001 << f_step) : 7'b0000000;
    assign f_sel_rx = (!f_cpif &&  f_is_rx) ? (7'b0000001 << f_step) : 7'b0000000;

    assign enc_fire = any_pend && pkt_to_tx_ready_i;

    // --------------------------------------------------------------------------
    // u_pkt_enc：encode 组合（{1'b0, lid=4'b0000, OPC[step], type}；bit[15]
    // dp 由 sbc_tx 覆盖；cpif 源直通保持寄存器）
    // --------------------------------------------------------------------------
    always @(*) begin
        case (f_step)
            3'd0:    enc_opcode = OPC_OFFSET;
            3'd1:    enc_opcode = OPC_VREF;
            3'd2:    enc_opcode = OPC_EQ;
            3'd3:    enc_opcode = OPC_PI;
            3'd4:    enc_opcode = OPC_DCDL;
            3'd5:    enc_opcode = OPC_REORDER;
            3'd6:    enc_opcode = OPC_DATA_CHECK;
            default: enc_opcode = OPC_OFFSET;
        endcase
    end

    assign pkt_to_tx_valid_o = any_pend && pkt_to_tx_ready_i;
    assign pkt_to_tx_o       = f_cpif ? pkt_cpif_hold
                                      : {1'b0, 4'b0000, enc_opcode,
                                         (f_is_rx ? SBC_TYPE_COMPL : SBC_TYPE_REQ)};

    // --------------------------------------------------------------------------
    // u_irq_gen / u_sticky 事件聚合
    // --------------------------------------------------------------------------
    assign err_evt = {dec_illegal_lid, dec_illegal_op,
                      gap_err_event_i, parity_err_event_i};
    assign err_set = |(err_evt & ~err_mask_i[3:0]);

    assign irq_pkt_arrival_o = pkt_arrival_flag_o && !pkt_arrival_mask_i;
    assign irq_err_o         = err_flag_o && !err_irq_mask_i;

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================

    // --------------------------------------------------------------------------
    // 报文 decode 分发（sb_tx_clk 域）：RETRAIN / U2 / 透传转发 +
    // step req toggle 翻转 + step complete 脉冲 + pkt_arrival sticky
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            pkt_arrival_flag_o       <= 1'b0;
            peer_retrain_pkt_o       <= 16'b0;
            peer_retrain_pkt_valid_o <= 1'b0;
            sbc_link_passthru_o       <= 16'b0;
            sbc_link_passthru_valid_o <= 1'b0;
            offset_complete_o         <= 1'b0;
            vref_complete_o           <= 1'b0;
            eq_complete_o             <= 1'b0;
            pi_complete_o             <= 1'b0;
            dcDL_complete_o           <= 1'b0;
            reorder_complete_o        <= 1'b0;
            data_check_complete_o     <= 1'b0;
            for (di = 0; di < 7; di = di + 1) begin
                req_toggle[di] <= {RX_EN_LANE_NUM{1'b0}};
            end
        end
        else begin
            // 默认：1 cycle 脉冲输出清零
            peer_retrain_pkt_valid_o  <= 1'b0;
            sbc_link_passthru_valid_o <= 1'b0;
            offset_complete_o         <= 1'b0;
            vref_complete_o           <= 1'b0;
            eq_complete_o             <= 1'b0;
            pi_complete_o             <= 1'b0;
            dcDL_complete_o           <= 1'b0;
            reorder_complete_o        <= 1'b0;
            data_check_complete_o     <= 1'b0;

            // RW1C 清除（事件优先：同拍新报文到达时保持置位）
            if (irq_clear_i[0] && !pkt_from_rx_valid_i) begin
                pkt_arrival_flag_o <= 1'b0;
            end

            if (pkt_from_rx_valid_i) begin
                pkt_arrival_flag_o <= 1'b1;   // 任何合法接收报文
                case (dec_type)
                    SBC_TYPE_REQ: begin
                        if (dec_is_retrain) begin
                            // filter[7] 门控：=0 仅 pkt_arrival_flag，MCU 手动处理
                            if (opcode_filter_i[7]) begin
                                peer_retrain_pkt_o       <= pkt_from_rx_i;
                                peer_retrain_pkt_valid_o <= 1'b1;
                            end
                        end
                        else if (dec_is_u2) begin
                            // U2 指令显式透传上送链路层，filter[7] 门控
                            if (opcode_filter_i[7]) begin
                                sbc_link_passthru_o       <= pkt_from_rx_i;
                                sbc_link_passthru_valid_o <= 1'b1;
                            end
                        end
                        else if (dec_step_v) begin
                            // lid 非法 / filter 命中：不分发（filter 命中不置错）
                            if (!dec_illegal_lid && lid_filter_i[dec_lid]
                                && opcode_filter_i[dec_step]) begin
                                for (di = 0; di < RX_EN_LANE_NUM; di = di + 1) begin
                                    if (dec_target[di]) begin
                                        req_toggle[dec_step][di] <= ~req_toggle[dec_step][di];
                                    end
                                end
                            end
                        end
                        // 国标其他 opcode / 未知 opcode：不自动分发（非法由 dec_illegal_op 置 sticky）
                    end
                    SBC_TYPE_COMPL: begin
                        if (dec_step_v) begin
                            if (!dec_illegal_lid && lid_filter_i[dec_lid]
                                && opcode_filter_i[dec_step]) begin
                                case (dec_step)
                                    3'd0:    offset_complete_o     <= 1'b1;
                                    3'd1:    vref_complete_o       <= 1'b1;
                                    3'd2:    eq_complete_o         <= 1'b1;
                                    3'd3:    pi_complete_o         <= 1'b1;
                                    3'd4:    dcDL_complete_o       <= 1'b1;
                                    3'd5:    reorder_complete_o    <= 1'b1;
                                    3'd6:    data_check_complete_o <= 1'b1;
                                    default: offset_complete_o     <= 1'b0;
                                endcase
                            end
                        end
                    end
                    SBC_TYPE_PASS: begin
                        // 链路层透传，filter[7] 门控
                        if (opcode_filter_i[7]) begin
                            sbc_link_passthru_o       <= pkt_from_rx_i;
                            sbc_link_passthru_valid_o <= 1'b1;
                        end
                    end
                    default: begin
                        // 保留 type：不分发
                    end
                endcase
            end
        end
    end

    // --------------------------------------------------------------------------
    // complete 回收 pending + AND 聚合（sb_tx_clk 域，容量 RX_EN_LANE_NUM）
    // 聚合完成当拍置 enc_pend_rx、清 compl_pend；step6 同拍拉 rx_data_check_agg_o
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            for (ai = 0; ai < 7; ai = ai + 1) begin
                compl_pend[ai] <= {RX_EN_LANE_NUM{1'b0}};
            end
            enc_pend_rx         <= 7'b0;
            rx_data_check_agg_o <= 1'b0;
        end
        else begin
            rx_data_check_agg_o <= 1'b0;
            for (ai = 0; ai < 7; ai = ai + 1) begin
                if (agg_done[ai]) begin
                    compl_pend[ai]  <= {RX_EN_LANE_NUM{1'b0}};
                    enc_pend_rx[ai] <= 1'b1;
                end
                else begin
                    compl_pend[ai] <= compl_pend[ai] | compl_edge[ai];
                    // 仲裁选中且 fire 当拍清零（同拍新聚合事件 set 优先）
                    if (enc_fire && f_sel_rx[ai]) begin
                        enc_pend_rx[ai] <= 1'b0;
                    end
                end
            end
            if (agg_done[6]) begin
                rx_data_check_agg_o <= 1'b1;
            end
        end
    end

    // complete 边沿检测延迟寄存器（sb_tx_clk 域）
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            for (ci = 0; ci < 7; ci = ci + 1) begin
                compl_sync2_d[ci] <= {RX_EN_LANE_NUM{1'b0}};
            end
        end
        else begin
            for (ci = 0; ci < 7; ci = ci + 1) begin
                compl_sync2_d[ci] <= compl_sync2[ci];
            end
        end
    end

    // --------------------------------------------------------------------------
    // encode pending：cpif 直通保持寄存器 + TX per-step req（sb_tx_clk 域）
    // 事件置位（同拍冲突 set 优先），仲裁选中且 fire 当拍清零
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            enc_pend_cpif <= 1'b0;
            pkt_cpif_hold <= 16'b0;
            enc_pend_tx   <= 7'b0;
        end
        else begin
            if (pkt_from_cpif_valid_i && pkt_from_cpif_ready_o) begin
                enc_pend_cpif <= 1'b1;
                pkt_cpif_hold <= pkt_from_cpif_i;
            end
            else if (enc_fire && f_cpif) begin
                enc_pend_cpif <= 1'b0;
            end

            for (ei = 0; ei < 7; ei = ei + 1) begin
                if (tx_req_in[ei]) begin
                    enc_pend_tx[ei] <= 1'b1;
                end
                else if (enc_fire && f_sel_tx[ei]) begin
                    enc_pend_tx[ei] <= 1'b0;
                end
            end
        end
    end

    // --------------------------------------------------------------------------
    // 反压选择锁定：any_pend 且 ready=0 首拍锁存仲裁选择，ready 恢复后解锁
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            out_lock  <= 1'b0;
            lock_cpif <= 1'b0;
            lock_step <= 3'd0;
            lock_is_rx<= 1'b0;
        end
        else if (pkt_to_tx_ready_i) begin
            out_lock <= 1'b0;
        end
        else if (any_pend && !out_lock) begin
            out_lock  <= 1'b1;
            lock_cpif <= arb_cpif;
            lock_step <= arb_step;
            lock_is_rx<= arb_is_rx;
        end
    end

    // --------------------------------------------------------------------------
    // u_sticky：err_flag / illegal_opcode / illegal_lid（sb_tx_clk 域）
    // 事件置位优先于 irq_clear 清除（错误不丢）；illegal_* 置位不受 err_mask 影响
    // --------------------------------------------------------------------------
    always @(posedge sb_tx_clk or negedge sb_rst_n) begin
        if (!sb_rst_n) begin
            err_flag_o       <= 1'b0;
            illegal_opcode_o <= 1'b0;
            illegal_lid_o    <= 1'b0;
        end
        else begin
            if (irq_clear_i[1] && !(|err_evt)) begin
                err_flag_o       <= 1'b0;
                illegal_opcode_o <= 1'b0;
                illegal_lid_o    <= 1'b0;
            end
            if (err_set) begin
                err_flag_o <= 1'b1;
            end
            if (dec_illegal_op) begin
                illegal_opcode_o <= 1'b1;
            end
            if (dec_illegal_lid) begin
                illegal_lid_o <= 1'b1;
            end
        end
    end

endmodule
