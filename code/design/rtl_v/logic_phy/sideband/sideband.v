// =============================================================================
// module name：sideband
// author: OpenHappy Team
// MAS/spec：spec/logic_phy/submods/sideband/sideband_spec.md
// =============================================================================
// 功能概述: SBC 边带收发、报文分发与 CPIF 事件解码集成父模块（per-link 唯一，纯例化连线）：
//   sbc_tx / sbc_rx / sbc_dispatcher / sbc_cpif_event
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-07  OpenHappy Team  初始版本：基于 sideband_spec v2.9 集成生成
//  2026-08-11  OpenHappy Team  TX/RX_LANE_MASK 参数透传（per-lane 方向不对称例化
//                       适配，sideband_spec v2.10）：掩码下传 sbc_cpif_event /
//                       sbc_dispatcher，lane_enable 单 bitmap 软件契约不变
//  2026-08-14  OpenHappy Team  集中式复位同步（logic_phy_crg）：新增 sb_rx_rst_n /
//                       mb_rstn_sync / rx_rstn_sync 复位端口，下传 sbc_rx /
//                       sbc_dispatcher / sbc_cpif_event；sb_rst_n 现承载 CRG
//                       同步后的 sb_tx 域复位（Group C 缺口修复）
//  2026-08-28  OpenHappy Team  4T1R（4 TX + 1 RX）扩容：LANE_NUM 硬件 2→4，per-lane
//                       训练握手 vector / 顶端口 per-lane bundle 随 LANE_NUM
//                       扩到 4；phy_rx_clk0/1 两路 RX 标量时钟不变（RX per-lane
//                       CDC 容量上限=2，高位 lane 由 RX 方向掩码剔除）
// =============================================================================

`timescale 1ns/1ps

module sideband #(
    parameter LANE_NUM = 4,     // 最大 lane 能力（4T1R：4 TX + 1 RX；子模块 hardware-fixed 4）
    // 编译期方向掩码（logic_phy 按 LP_ lane 宏派生，LANE_NUM=4 位宽）：
    // bit i=1 表示 lane i 该方向已例化；下传 sbc_cpif_event / sbc_dispatcher，
    // 有效使能 = sbc_lane_enable_i & 方向掩码
    parameter TX_LANE_MASK = 4'b1111,
    parameter RX_LANE_MASK = 4'b1111
) (
    // -------------------------------------------------------------------------
    // 时钟与复位
    // -------------------------------------------------------------------------
    input  wire                  sb_tx_clk_in,  // 边带发送工作时钟源（常跑，最高 500 MHz）
    output wire                  sb_tx_clk,     // 边带发送随路时钟（sbc_tx 门控输出，间隙停低）
    input  wire                  sb_rx_clk,     // 边带接收时钟（对端随路 source-synchronous）
    input  wire                  sb_rst_n,      // 边带 TX 域同步释放复位（logic_phy_crg 产出，低有效）
    input  wire                  sb_rx_rst_n,   // 边带 RX 域（sb_rx_clk 负沿）同步释放复位（logic_phy_crg 产出）
    input  wire                  mb_rstn_sync,  // mb_clk 域同步释放复位（logic_phy_crg 产出）
    input  wire [LANE_NUM-1:0]   rx_rstn_sync,  // phy_rx_clk{i} 域同步释放复位（logic_phy_crg 产出，per-lane）
    input  wire                  mb_clk,        // TX 主带时钟（per-link）
    input  wire                  phy_rx_clk0,   // RX 主带时钟 lane0
    input  wire                  phy_rx_clk1,   // RX 主带时钟 lane1

    // -------------------------------------------------------------------------
    // 边带串行通道（与 analog PHY）
    // -------------------------------------------------------------------------
    output wire                  sb_txd_o,      // 边带发送串行数据（同步 sb_tx_clk）
    input  wire                  sb_rxd_i,      // 边带接收串行数据（同步 sb_rx_clk）
    output wire                  sb_txr_o,      // 边带发送数据冗余（intf v10.3 P1；sbc_txr_en=1 承载 SBC 流、主路隔离）
    input  wire                  sb_rxr_i,      // 边带接收数据冗余（sbc_rxr_sel=1 进解调主路）
    input  wire                  sb_txr_en_i,   // 0x828 SBC_REDUNDANCY[0]（apb_csr_top）
    input  wire                  sb_rxr_sel_i,  // 0x828 SBC_REDUNDANCY[1]（apb_csr_top）

    // -------------------------------------------------------------------------
    // CPIF TX 事件接口（link_layer -> 本端物理层，sb_tx_clk_in 域）
    // -------------------------------------------------------------------------
    input  wire                  cpif_tx_event_enable_i,
    input  wire                  cpif_tx_event_req_i,
    input  wire [15:0]           cpif_tx_event_cmd_i,
    output wire                  cpif_tx_event_ack_o,
    output wire [7:0]            cpif_tx_event_status_o,

    // -------------------------------------------------------------------------
    // CPIF RX 事件接口（本端物理层 -> link_layer，sb_tx_clk_in 域）
    // -------------------------------------------------------------------------
    input  wire                  cpif_rx_event_enable_i,
    output wire                  cpif_rx_event_req_o,
    output wire [15:0]           cpif_rx_event_cmd_o,
    input  wire                  cpif_rx_event_ack_i,
    input  wire [7:0]            cpif_rx_event_status_i,

    // -------------------------------------------------------------------------
    // TX 训练 req/complete（per-lane tx_training_fsm，mb_clk 域）
    // -------------------------------------------------------------------------
    output wire [LANE_NUM-1:0]   train_req_o,
    output wire [LANE_NUM-1:0]   retrain_req_o,
    input  wire [LANE_NUM-1:0]   train_complete_ack_i,  // level：ACTIVE 保持高
    input  wire [LANE_NUM-1:0]   offset_req_i,
    input  wire [LANE_NUM-1:0]   vref_req_i,
    input  wire [LANE_NUM-1:0]   eq_req_i,
    input  wire [LANE_NUM-1:0]   pi_req_i,
    input  wire [LANE_NUM-1:0]   dcDL_req_i,
    input  wire [LANE_NUM-1:0]   reorder_req_i,
    input  wire [LANE_NUM-1:0]   data_check_req_i,
    output wire [LANE_NUM-1:0]   offset_complete_o,
    output wire [LANE_NUM-1:0]   vref_complete_o,
    output wire [LANE_NUM-1:0]   eq_complete_o,
    output wire [LANE_NUM-1:0]   pi_complete_o,
    output wire [LANE_NUM-1:0]   dcDL_complete_o,
    output wire [LANE_NUM-1:0]   reorder_complete_o,
    output wire [LANE_NUM-1:0]   data_check_complete_o,

    // -------------------------------------------------------------------------
    // RX 训练 req/complete（per-lane rx_training_fsm，phy_rx_clk{i} 域）
    // -------------------------------------------------------------------------
    input  wire [LANE_NUM-1:0]   train_req_i,
    output wire [LANE_NUM-1:0]   rx_retrain_req_o,
    output wire [LANE_NUM-1:0]   train_complete_ack_o,
    output wire [LANE_NUM-1:0]   offset_req_o,
    output wire [LANE_NUM-1:0]   vref_req_o,
    output wire [LANE_NUM-1:0]   eq_req_o,
    output wire [LANE_NUM-1:0]   pi_req_o,
    output wire [LANE_NUM-1:0]   dcDL_req_o,
    output wire [LANE_NUM-1:0]   reorder_req_o,
    output wire [LANE_NUM-1:0]   data_check_req_o,
    input  wire [LANE_NUM-1:0]   offset_complete_i,
    input  wire [LANE_NUM-1:0]   vref_complete_i,
    input  wire [LANE_NUM-1:0]   eq_complete_i,
    input  wire [LANE_NUM-1:0]   pi_complete_i,
    input  wire [LANE_NUM-1:0]   dcDL_complete_i,
    input  wire [LANE_NUM-1:0]   reorder_complete_i,
    input  wire [LANE_NUM-1:0]   data_check_complete_i,

    // -------------------------------------------------------------------------
    // APB CSR 配置（apb_csr_top 具名信号直送，CDC 在 apb_csr_top 侧完成）
    // -------------------------------------------------------------------------
    input  wire                  sbc_tx_enable_i,
    input  wire                  sbc_tx_manual_trigger_i,
    input  wire [15:0]           sbc_tx_pkt_inject_i,
    input  wire                  sbc_rx_enable_i,
    input  wire                  sbc_rx_manual_read_i,
    input  wire                  sbc_rx_soft_rst_i,
    input  wire [7:0]            sbc_rx_buf_depth_i,
    input  wire [15:0]           sbc_lid_filter_i,
    input  wire [7:0]            sbc_opcode_filter_i,
    input  wire [7:0]            sbc_err_mask_i,
    input  wire                  sbc_pkt_arrival_mask_i,
    input  wire                  sbc_err_irq_mask_i,
    input  wire [1:0]            sbc_irq_clear_i,
    input  wire [LANE_NUM-1:0]   sbc_lane_enable_i,

    // -------------------------------------------------------------------------
    // 状态回流（sideband 各 MAS -> apb_csr_top，CDC 在 apb_csr_top 侧完成）
    // -------------------------------------------------------------------------
    output wire [2:0]            sbc_tx_buf_level_o,
    output wire                  sbc_tx_busy_o,
    output wire                  sbc_tx_err_o,
    output wire [3:0]            sbc_rx_buf_level_o,
    output wire                  sbc_rx_busy_o,
    output wire                  sbc_rx_parity_err_o,
    output wire                  sbc_rx_gap_err_o,
    output wire                  sbc_rx_buf_ovf_o,
    output wire [15:0]           sbc_rx_buf_rdata_o,
    output wire                  sbc_pkt_arrival_flag_o,
    output wire                  sbc_err_flag_o,
    output wire                  sbc_illegal_opcode_o,
    output wire                  sbc_illegal_lid_o,

    // -------------------------------------------------------------------------
    // 中断（level，-> intc）
    // -------------------------------------------------------------------------
    output wire                  irq_sbc_pkt_arrival_o,
    output wire                  irq_sbc_err_o
);

    // ==========================================================================
    // 内部信号定义（子模块间互连线，按通路分组）
    // ==========================================================================

    // --------------------------------------------------------------------------
    // dispatcher -> sbc_tx（encode 报文，valid-ready 16 bit）
    // --------------------------------------------------------------------------
    wire [15:0]           pkt_d2t;
    wire                  pkt_d2t_valid;
    wire                  pkt_d2t_ready;

    // --------------------------------------------------------------------------
    // sbc_rx -> dispatcher（接收报文直通，valid-only 16 bit + 错误事件脉冲）
    // --------------------------------------------------------------------------
    wire [15:0]           pkt_r2d;
    wire                  pkt_r2d_valid;
    wire                  parity_err_evt;
    wire                  gap_err_evt;

    // --------------------------------------------------------------------------
    // cpif_event -> dispatcher（本端链路层发起报文，valid-ready 16 bit）
    // --------------------------------------------------------------------------
    wire [15:0]           pkt_c2d;
    wire                  pkt_c2d_valid;
    wire                  pkt_c2d_ready;

    // --------------------------------------------------------------------------
    // dispatcher -> cpif_event（分发报文 valid-only + data_check AND 聚合脉冲）
    // --------------------------------------------------------------------------
    wire [15:0]           link_passthru;
    wire                  link_passthru_valid;
    wire [15:0]           peer_retrain_pkt;
    wire                  peer_retrain_pkt_valid;
    wire                  rx_data_check_agg;

    // --------------------------------------------------------------------------
    // cpif_event -> dispatcher（per-link per-step req ×7，同域 1 cycle 脉冲）
    // --------------------------------------------------------------------------
    wire                  pl_offset_req;
    wire                  pl_vref_req;
    wire                  pl_eq_req;
    wire                  pl_pi_req;
    wire                  pl_dcDL_req;
    wire                  pl_reorder_req;
    wire                  pl_data_check_req;

    // --------------------------------------------------------------------------
    // dispatcher -> cpif_event（per-link per-step complete ×7，同域 1 cycle 脉冲）
    // --------------------------------------------------------------------------
    wire                  pl_offset_compl;
    wire                  pl_vref_compl;
    wire                  pl_eq_compl;
    wire                  pl_pi_compl;
    wire                  pl_dcDL_compl;
    wire                  pl_reorder_compl;
    wire                  pl_data_check_compl;

    // ==========================================================================
    // 子模块实例化
    // ==========================================================================

    // --------------------------------------------------------------------------
    // u_sbc_tx：SBC 发送（工作域 sb_tx_clk_in；门控输出随路时钟 sb_tx_clk）
    // --------------------------------------------------------------------------
    sbc_tx u_sbc_tx (
        .sb_tx_clk_in       (sb_tx_clk_in),
        .sb_tx_clk          (sb_tx_clk),
        .sb_rst_n           (sb_rst_n),
        .enable_i           (sbc_tx_enable_i),
        .manual_trigger_i   (sbc_tx_manual_trigger_i),
        .pkt_inject_i       (sbc_tx_pkt_inject_i),
        .pkt_i              (pkt_d2t),
        .pkt_valid_i        (pkt_d2t_valid),
        .pkt_ready_o        (pkt_d2t_ready),
        .sb_txd_o           (sb_txd_o),
        .sb_txr_o           (sb_txr_o),
        .sb_txr_en_i        (sb_txr_en_i),
        .buf_level_o        (sbc_tx_buf_level_o),
        .busy_o             (sbc_tx_busy_o),
        .err_o              (sbc_tx_err_o)
    );

    // --------------------------------------------------------------------------
    // u_sbc_rx：SBC 接收（解串域 sb_rx_clk；分发域 sb_tx_clk_in，
    // MAS 端口名 sb_tx_clk 源接顶 ports.sb_tx_clk_in，见 spec §5.2 时钟命名说明）
    // --------------------------------------------------------------------------
    sbc_rx u_sbc_rx (
        .sb_rx_clk          (sb_rx_clk),
        .sb_tx_clk          (sb_tx_clk_in),
        .sb_rst_n           (sb_rst_n),
        .sb_rx_rst_n        (sb_rx_rst_n),
        .sb_rxd_i           (sb_rxd_i),
        .sb_rxr_i           (sb_rxr_i),
        .sb_rxr_sel_i       (sb_rxr_sel_i),
        .enable_i           (sbc_rx_enable_i),
        .manual_read_i      (sbc_rx_manual_read_i),
        .soft_rst_i         (sbc_rx_soft_rst_i),
        .buf_depth_i        (sbc_rx_buf_depth_i),
        .pkt_o              (pkt_r2d),
        .pkt_valid_o        (pkt_r2d_valid),
        .parity_err_event_o (parity_err_evt),
        .gap_err_event_o    (gap_err_evt),
        .buf_level_o        (sbc_rx_buf_level_o),
        .busy_o             (sbc_rx_busy_o),
        .parity_err_o       (sbc_rx_parity_err_o),
        .gap_err_o          (sbc_rx_gap_err_o),
        .buf_ovf_o          (sbc_rx_buf_ovf_o),
        .buf_rdata_o        (sbc_rx_buf_rdata_o)
    );

    // --------------------------------------------------------------------------
    // u_sbc_dispatcher：报文分发 + per-step SBC encode/decode + RX 方向 CDC
    // --------------------------------------------------------------------------
    sbc_dispatcher #(
        .RX_LANE_MASK (RX_LANE_MASK)
    ) u_sbc_dispatcher (
        .sb_tx_clk              (sb_tx_clk_in),
        .sb_rst_n               (sb_rst_n),
        .rx_rstn_sync           (rx_rstn_sync),
        .phy_rx_clk0            (phy_rx_clk0),
        .phy_rx_clk1            (phy_rx_clk1),
        .pkt_from_rx_i          (pkt_r2d),
        .pkt_from_rx_valid_i    (pkt_r2d_valid),
        .parity_err_event_i     (parity_err_evt),
        .gap_err_event_i        (gap_err_evt),
        .pkt_from_cpif_i        (pkt_c2d),
        .pkt_from_cpif_valid_i  (pkt_c2d_valid),
        .pkt_from_cpif_ready_o  (pkt_c2d_ready),
        .offset_req_i           (pl_offset_req),
        .vref_req_i             (pl_vref_req),
        .eq_req_i               (pl_eq_req),
        .pi_req_i               (pl_pi_req),
        .dcDL_req_i             (pl_dcDL_req),
        .reorder_req_i          (pl_reorder_req),
        .data_check_req_i       (pl_data_check_req),
        .offset_complete_i      (offset_complete_i),
        .vref_complete_i        (vref_complete_i),
        .eq_complete_i          (eq_complete_i),
        .pi_complete_i          (pi_complete_i),
        .dcDL_complete_i        (dcDL_complete_i),
        .reorder_complete_i     (reorder_complete_i),
        .data_check_complete_i  (data_check_complete_i),
        .lid_filter_i           (sbc_lid_filter_i),
        .opcode_filter_i        (sbc_opcode_filter_i),
        .err_mask_i             (sbc_err_mask_i),
        .pkt_arrival_mask_i     (sbc_pkt_arrival_mask_i),
        .err_irq_mask_i         (sbc_err_irq_mask_i),
        .irq_clear_i            (sbc_irq_clear_i),
        .lane_enable_i          (sbc_lane_enable_i),
        .pkt_to_tx_o            (pkt_d2t),
        .pkt_to_tx_valid_o      (pkt_d2t_valid),
        .pkt_to_tx_ready_i      (pkt_d2t_ready),
        .sbc_link_passthru_o        (link_passthru),
        .sbc_link_passthru_valid_o  (link_passthru_valid),
        .peer_retrain_pkt_o         (peer_retrain_pkt),
        .peer_retrain_pkt_valid_o   (peer_retrain_pkt_valid),
        .offset_complete_o      (pl_offset_compl),
        .vref_complete_o        (pl_vref_compl),
        .eq_complete_o          (pl_eq_compl),
        .pi_complete_o          (pl_pi_compl),
        .dcDL_complete_o        (pl_dcDL_compl),
        .reorder_complete_o     (pl_reorder_compl),
        .data_check_complete_o  (pl_data_check_compl),
        .rx_data_check_agg_o    (rx_data_check_agg),
        .offset_req_o           (offset_req_o),
        .vref_req_o             (vref_req_o),
        .eq_req_o               (eq_req_o),
        .pi_req_o               (pi_req_o),
        .dcDL_req_o             (dcDL_req_o),
        .reorder_req_o          (reorder_req_o),
        .data_check_req_o       (data_check_req_o),
        .pkt_arrival_flag_o     (sbc_pkt_arrival_flag_o),
        .err_flag_o             (sbc_err_flag_o),
        .illegal_opcode_o       (sbc_illegal_opcode_o),
        .illegal_lid_o          (sbc_illegal_lid_o),
        .irq_pkt_arrival_o      (irq_sbc_pkt_arrival_o),
        .irq_err_o              (irq_sbc_err_o)
    );

    // --------------------------------------------------------------------------
    // u_sbc_cpif_event：CPIF 事件接口 + EventCmd 解码 + 多 lane 聚合 +
    // TX 方向 CDC（mb_clk <-> sb_tx_clk_in）
    // --------------------------------------------------------------------------
    sbc_cpif_event #(
        .TX_LANE_MASK (TX_LANE_MASK),
        .RX_LANE_MASK (RX_LANE_MASK)
    ) u_sbc_cpif_event (
        .sb_tx_clk              (sb_tx_clk_in),
        .sb_rst_n               (sb_rst_n),
        .mb_clk                 (mb_clk),
        .mb_rstn_sync           (mb_rstn_sync),
        .rx_rstn_sync           (rx_rstn_sync),
        .phy_rx_clk0            (phy_rx_clk0),
        .phy_rx_clk1            (phy_rx_clk1),
        .tx_event_enable_i      (cpif_tx_event_enable_i),
        .tx_event_req_i         (cpif_tx_event_req_i),
        .tx_event_cmd_i         (cpif_tx_event_cmd_i),
        .tx_event_ack_o         (cpif_tx_event_ack_o),
        .tx_event_status_o      (cpif_tx_event_status_o),
        .rx_event_enable_i      (cpif_rx_event_enable_i),
        .rx_event_req_o         (cpif_rx_event_req_o),
        .rx_event_cmd_o         (cpif_rx_event_cmd_o),
        .rx_event_ack_i         (cpif_rx_event_ack_i),
        .rx_event_status_i      (cpif_rx_event_status_i),
        .peer_retrain_pkt_i         (peer_retrain_pkt),
        .peer_retrain_pkt_valid_i   (peer_retrain_pkt_valid),
        .link_passthru_i            (link_passthru),
        .link_passthru_valid_i      (link_passthru_valid),
        .rx_data_check_agg_i    (rx_data_check_agg),
        .offset_complete_i      (pl_offset_compl),
        .vref_complete_i        (pl_vref_compl),
        .eq_complete_i          (pl_eq_compl),
        .pi_complete_i          (pl_pi_compl),
        .dcDL_complete_i        (pl_dcDL_compl),
        .reorder_complete_i     (pl_reorder_compl),
        .data_check_complete_i  (pl_data_check_compl),
        .pkt_to_disp_o          (pkt_c2d),
        .pkt_to_disp_valid_o    (pkt_c2d_valid),
        .pkt_to_disp_ready_i    (pkt_c2d_ready),
        .offset_req_o           (pl_offset_req),
        .vref_req_o             (pl_vref_req),
        .eq_req_o               (pl_eq_req),
        .pi_req_o               (pl_pi_req),
        .dcDL_req_o             (pl_dcDL_req),
        .reorder_req_o          (pl_reorder_req),
        .data_check_req_o       (pl_data_check_req),
        .offset_req_i           (offset_req_i),
        .vref_req_i             (vref_req_i),
        .eq_req_i               (eq_req_i),
        .pi_req_i               (pi_req_i),
        .dcDL_req_i             (dcDL_req_i),
        .reorder_req_i          (reorder_req_i),
        .data_check_req_i       (data_check_req_i),
        .offset_complete_o      (offset_complete_o),
        .vref_complete_o        (vref_complete_o),
        .eq_complete_o          (eq_complete_o),
        .pi_complete_o          (pi_complete_o),
        .dcDL_complete_o        (dcDL_complete_o),
        .reorder_complete_o     (reorder_complete_o),
        .data_check_complete_o  (data_check_complete_o),
        .train_req_o            (train_req_o),
        .retrain_req_o          (retrain_req_o),
        .train_complete_ack_i   (train_complete_ack_i),
        .train_req_i            (train_req_i),
        .rx_retrain_req_o       (rx_retrain_req_o),
        .train_complete_ack_o   (train_complete_ack_o),
        .lane_enable_i          (sbc_lane_enable_i)
    );

endmodule
