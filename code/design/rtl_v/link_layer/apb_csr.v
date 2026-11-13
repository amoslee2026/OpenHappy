// =============================================================================
// module name：apb_csr
// author: OpenHappy Team
// MAS：spec/link_layer/submods/link_layer/mas/apb_csr_mas.md
// 功能概述: 链路层 APB 配置访问入口（APB_CSR 角色），register_flow 迁移后的
//           **薄 wrapper**。全部寄存器存储 / 复位 / 读回由 register_flow 生成的
//           link_layer_apb_reg_sync（内含 apb_reg_sync_if + link_layer_rf）承载，
//           W1C 状态位（IRQ_STATUS / RETRAIN_STATUS / per-link IRQ）亦为 RF 原生
//           W1C 存储（hw_<f>_wen 条件置位 + APB 写 1 条件清，置位优先），本模块仅：
//             ① 例化 link_layer_apb_reg_sync，把具名 RO 输入（状态/计数器/中断
//                置位脉冲）驱动进 RF；
//             ② 把 IRQ_STATUS / RETRAIN_STATUS / per-link IRQ 的源域置位脉冲按位
//                搬运为 RF 的 hw_*_wen（per-link IRQ 在 wrapper 内 OR 聚合到
//                IRQ_STATUS 对应位），把 RF 的 RW 配置输出映射为 csr_* / cpif_*
//                具名信号；IRQ_CLR_REG(0x30) 已随 W1C 化删除（清中断改直写 0x28/
//                0x38/0x64/0x70/0x7C 对应位写 1，RETRY_ERR 双清位分写 0x28
//                bit1(RETRY_TIMEOUT)/bit2(RETRY_NACK)）；
//             ③ RC 计数器（tx/rx_flit_cnt / timeout_cnt / crc_err_cnt / per-link
//                cnt）读后清零由 wrapper 内解码 APB 读完成；置位沿检测（INIT_CFG /
//                LP_CFG）→ 1 cycle 触发脉冲；
//           初始化流程补齐（G-01/G-03/G-04）：STATUS_REG.bit8=PHY_READY（电气
//           init 完成，phy_ready_i 聚合）；IRQ_STATUS/MASK 新增 bit6=RX_READY_IRQ
//           （TX 侧收 0x7F，G-03）、bit7=LANE_ON_DONE_IRQ（TX 侧收 0x40 完成，G-04），
//           置位脉冲来自 link_event_process 经顶层同步。
// 编码约束：apb_clk 域寄存器沿用异步复位（与生成 RF / 外部 APB 一致）；wrapper
//           存储位与聚合逻辑以 always @(posedge apb_clk or negedge apb_rst_n)
//           实现；组合映射以 assign 实现。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-08  OpenHappy Team  初始版本（对齐 MAS v1.0）
//  2026-08-11  OpenHappy Team  INIT_CFG_REG 新增 bit5 RETRAIN：软件置位沿触发
//                          链路层发 RETRAIN cmd（S_NORMAL/S_PHY_TRAIN 下有效）
//  2026-09-08  OpenHappy Team  register_flow 迁移重构为薄 wrapper：寄存器存储/读回移入
//                          link_layer_apb_reg_sync（link_layer_rf），本模块例化之；
//                          状态位（IRQ/RETRAIN/per-link IRQ）因 register_flow W1C
//                          无回读而建模为 RO(wrapper 存 1C)，RC 计数器(wrapper 读后清
//                          零)；去 TX_LINKn_EXIST 宏（当前配置 4TX 恒全链路）
//  2026-09-20  OpenHappy Team  动态时钟控制（L1 §4.10 / §5.14）：API 从 register_flow
//                          生成的 RF 引出 CLK_CFG_REG/CLK_CFG_TX_REG/CLK_CFG_RX_REG
//                          （0x98/0x9C/0xA0）RW 输出，新增具名输出 csr_clk_cfg_{tx,rx,sram}_dyn_en_o
//                          + csr_clk_cfg_{tx,rx}{0..3}_dyn_o（apb_clk 域解码 → 顶层
//                          sync_cell_norst 同步至 mb_clk 供 ICG 使能）；pslverr 地址
//                          上限 0x0094 扩至 0x00A0（0x98/0x9C/0xA0 三个时钟寄存器可访问，
//                          >0xA0 或非 4-byte 对齐才报错）。
//  2026-09-20  OpenHappy Team  恢复 ACK_ROUTE_CFG_REG 16bit 聚合输出 csr_ack_route_src_en_o
//                          （TX0[3:0]/TX1[7:4]/TX2[11:8]/TX3[15:12]，reset 0x000F，由
//                          link_layer_apb_reg_sync 的 4×4bit ack_route_cfg_reg_tx{0..3}_src_en
//                          拼接；对齐 link_layer.v:919 顶层连接，修复 register_flow 迁移
//                          遗漏）。
//  2026-10-10  OpenHappy Team  W1C 修复规划 v2.1（arch 评审通过）：IRQ_STATUS/RETRAIN_
//                          STATUS/per-link IRQ 的 16 个 RW1C 状态位改由 RF 原生 W1C
//                          存储（register_flow template g 谓词新形态：hw_<f>_wen 条件
//                          置位 + APB 写 1 条件清，置位最高优先；per-field 段实现，写 case
//                          不承载清），wrapper 删除 16 个粘滞 FF 与 0x30(IRQ_CLR_REG) WO
//                          清通路（0x30 现为无 RF case 分支地址：读 0 / 写无解码 / 0x0-0xA0
//                          内不报 pslverr），中断改为 status 寄存器直写 1 清；
//                          本轮 -14 FF（0x30 通路净删）。
//  2026-10-13  OpenHappy Team  动态时钟控制去总开关（arch 评审 LL-20261013-dynclk 通过）：
//                          CLK_CFG_REG.TX_DYN_EN/RX_DYN_EN 已从 register_flow 删除
//                          （RESERVED RO 0），wrapper 删对应 2 具名输出 csr_clk_cfg_{tx,rx}
//                          _dyn_en_o + 2 RF 连接 + 2 assign（11→9 动态时钟输出）；每路独立
//                          开关 = CLK_CFG_TX_REG[n]/CLK_CFG_RX_REG[n]（0x9C/0xA0 原位），
//                          SRAM_DYN_EN 保留 0x98 bit2；ldynclk 语义 N1 见 spec §4.10.1。
//  2026-10-23  OpenHappy Team  LL-20261023-CPIFRXLPC（L1 v1.17 / L2 v1.9）：新增输入
//                          status_reg_rx_lpc_i（CPIF RX 低功耗状态，顶层 2flop 同步后的
//                          异步电平）→ STATUS_REG.RX_LPC（0x0034 bit9，RO，复位名义
//                          1'b1）；status_reg_q 扩 [8:0]→[9:0]（复位 10'b10_0000_0000）；
//                          纯观测位：不进中断聚合 / 不驱动任何 csr_* 输出（零行为变化）
// =============================================================================

`timescale 1ns/1ps

`include "link_layer_defines.vh"

module apb_csr (
    // =========================================================================
    // 时钟与复位（apb_clk 域，1 GHz）
    // =========================================================================
    input  wire        apb_clk,                  // APB 配置通路时钟
    input  wire        apb_rst_n,                // APB 异步复位，低有效

    // =========================================================================
    // APB slave 接口（L2 §2.7；pready 单拍，pslverr 对未实现地址）
    // =========================================================================
    input  wire        psel_i,                   // APB select
    input  wire        penable_i,                // APB enable
    input  wire        pwrite_i,                 // APB 写使能（1=写 / 0=读）
    input  wire [15:0] paddr_i,                  // APB 地址（4-byte 对齐）
    input  wire [31:0] pwdata_i,                 // APB 写数据
    output wire        pready_o,                 // APB 就绪（glue 恒 1）
    output wire [31:0] prdata_o,                 // APB 读数据
    output wire        pslverr_o,                // APB 错误响应（访问未实现地址）

    // =========================================================================
    // 具名配置分发输出（经顶层跨域同步器送 mb_clk / sb_clk；CPIF 控制不经跨域）
    // =========================================================================
    output wire        csr_cfg_done_o,               // INIT_CFG_REG.CONFIG_DONE 电平（非脉冲）→ 两 lcmsm / event_process
    output wire        csr_perf_mode_o,              // WORK_MODE==1 解码 → link_tx / link_rx
    output wire        csr_s2p_mode_o,               // PHY_MODE.s2p_mode：1=1024 bit 全有效 / 0=低 512 有效
    output wire        csr_protocol_mode_o,          // MODE_REG.PROTOCOL_MODE：1=PAIF / 0=UDI
    output wire        csr_udi_ready_active_en_o,     // MODE_REG.UDI_READY_ACTIVE_EN：1=UDI ready 跟随 link_tx_active / 0=恒 1
    output wire [1:0]  csr_bw_mode_o,                // BW_CFG_REG.bw_mode：00=全带宽 / 01=1/2 / 10=1/4
    output wire        csr_loopback_en_o,            // LOOPBACK_CFG_REG.LOOPBACK_EN
    output wire [15:0] csr_loopback_src_o,           // LOOPBACK_CFG_REG.RX0_SRC~RX3_SRC 聚合
    output wire [15:0] csr_ack_route_src_en_o,      // ACK_ROUTE_CFG_REG.TX0~3_SRC_EN 聚合（TX0[3:0]/TX1[7:4]/TX2[11:8]/TX3[15:12]，reset 0x000F）
    output wire        csr_clk_cfg_sram_dyn_en_o,    // CLK_CFG_REG.SRAM_DYN_EN（bit2）：SRAM 动态时钟开关（1=动态 / 0=always-on）——TX/RX 总开关已删（见 0x9C/0xA0 每路独立开关）
    output wire        csr_clk_cfg_tx0_dyn_o,        // CLK_CFG_TX_REG.TX0_DYN（bit0）：link_tx[0] 独立动态时钟开关
    output wire        csr_clk_cfg_tx1_dyn_o,        // CLK_CFG_TX_REG.TX1_DYN（bit1）
    output wire        csr_clk_cfg_tx2_dyn_o,        // CLK_CFG_TX_REG.TX2_DYN（bit2）
    output wire        csr_clk_cfg_tx3_dyn_o,        // CLK_CFG_TX_REG.TX3_DYN（bit3）
    output wire        csr_clk_cfg_rx0_dyn_o,        // CLK_CFG_RX_REG.RX0_DYN（bit0）：link_rx[0] 独立动态时钟开关
    output wire        csr_clk_cfg_rx1_dyn_o,        // CLK_CFG_RX_REG.RX1_DYN（bit1）
    output wire        csr_clk_cfg_rx2_dyn_o,        // CLK_CFG_RX_REG.RX2_DYN（bit2）
    output wire        csr_clk_cfg_rx3_dyn_o,        // CLK_CFG_RX_REG.RX3_DYN（bit3）
    output wire        csr_retry_en_o,               // RETRY_CFG_REG.RETRY_EN
    output wire        csr_credit_en_o,              // RETRY_CFG_REG.CREDIT_EN
    output wire        csr_crc_en_o,                 // RETRY_CFG_REG.CRC_EN（CRC0/CRC1 共用）
    output wire        csr_ecc_en_o,                 // RETRY_CFG_REG.ECC_EN（bit21；性能模式与 CRC_EN 互选）
    output wire [1:0]  csr_nack_mode_o,              // RETRY_CFG_REG.NACK_MODE：00=Standard / 01=Select
    output wire        csr_inj_en_link0_o,           // INJECT_ERR_LINK0_REG.INJECT_EN（电平门控，EN=1 即注错）
    output wire        csr_inj_fb_link0_o,           // INJECT_ERR_LINK0_REG.FRONT_BACK：0=front(CRC0)/1=back(CRC1)
    output wire        csr_inj_en_link1_o,           // INJECT_ERR_LINK1_REG.INJECT_EN
    output wire        csr_inj_fb_link1_o,           // INJECT_ERR_LINK1_REG.FRONT_BACK
    output wire        csr_inj_en_link2_o,           // INJECT_ERR_LINK2_REG.INJECT_EN
    output wire        csr_inj_fb_link2_o,           // INJECT_ERR_LINK2_REG.FRONT_BACK
    output wire        csr_inj_en_link3_o,           // INJECT_ERR_LINK3_REG.INJECT_EN
    output wire        csr_inj_fb_link3_o,           // INJECT_ERR_LINK3_REG.FRONT_BACK
    output wire        csr_tx_flit_clr_o,
    output wire        csr_rx_flit_clr_o,
    output wire        csr_crc_err_clr_o,
    output wire        csr_tx1_cnt_clr_o,
    output wire        csr_tx2_cnt_clr_o,
    output wire        csr_tx3_cnt_clr_o,
    output wire [15:0] csr_drift_period_o,           // DRIFT_CFG_REG.DRIFT_PERIOD（mb_clk cycle 单位）
    output wire [15:0] csr_retry_timeout_val_o,      // TIMEOUT_REG.RETRY_TIMEOUT（mb_clk cycle 单位）
    output wire [7:0]  csr_retrain_timeout_thr_o,    // RETRAIN_THR_REG.RETRAIN_TIMEOUT_THR
    output wire [7:0]  csr_retrain_nack_thr_o,       // RETRAIN_THR_REG.RETRAIN_NACK_THR
    output wire        csr_retrain_timeout_en_o,     // RETRAIN_THR_REG.RETRAIN_TIMEOUT_EN
    output wire        csr_retrain_nack_en_o,        // RETRAIN_THR_REG.RETRAIN_NACK_EN
    output wire        csr_init_phy_init_o,          // INIT_CFG_REG.PHY_INIT 置位沿 1 cycle 脉冲
    output wire        csr_init_retrain_o,           // INIT_CFG_REG.RETRAIN 置位沿 1 cycle 脉冲（软件触发训练）
    output wire        csr_init_lane_on_o,           // INIT_CFG_REG.LANE_ON 置位沿 1 cycle 脉冲
    output wire        csr_init_lane_off_o,          // INIT_CFG_REG.LANE_OFF 置位沿 1 cycle 脉冲
    output wire        csr_init_rx_ready_o,          // INIT_CFG_REG.RX_READY 置位沿 1 cycle 脉冲
    output wire [5:0]  csr_lp_cmd_o,                 // LP_CFG_REG 6 路低功耗命令置位沿 1 cycle 脉冲

    // =========================================================================
    // CPIF 控制输出（apb_clk 域直驱顶层，不经跨域，L2 §2.3.2）
    // =========================================================================
    output wire [3:0]  cpif_tx_mode_o,               // PHY_MODE.tx_mode
    output wire [3:0]  cpif_rx_mode_o,               // PHY_MODE.rx_mode
    output wire        cpif_tx_rst_n_o,              // CTRL_REG.TX_RST_N
    output wire        cpif_tx_lpc_o,                // CTRL_REG.TX_LPC
    output wire        cpif_rx_rst_n_o,              // CTRL_REG.RX_RST_N

    // =========================================================================
    // 状态 / 计数器回读输入（源域生成，经顶层跨域同步后送入，apb_clk 域）
    // =========================================================================
    input  wire [3:0]  link_fsm_state_tx_i,      // TX link FSM 状态 → STATUS_REG.LINK_FSM_STATE
    input  wire        bridge_en_i,              // public CSR bridge_en → STATUS_REG.BRIDGE_STATE
    input  wire [3:0]  link_fsm_state_rx_i,      // RX link FSM 状态 → DFx 观测
    input  wire [1:0]  lp_state_tx_i,            // TX 低功耗状态 → STATUS_REG.LP_STATE
    input  wire [1:0]  lp_state_rx_i,            // RX 低功耗状态 → DFx 观测
    input  wire        phy_ready_i,               // cpif_tx_ready & cpif_rx_ready（mb_clk 相与，顶层 2flop 同步）→ STATUS_REG.PHY_READY（G-01）
    input  wire        status_reg_rx_lpc_i,     // CPIF RX LPC 状态（顶层 2flop 同步）→ STATUS_REG.RX_LPC（bit9，v1.14）
    input  wire        retrain_timeout_flag_i,   // 置位 RETRAIN_STATUS_REG.RETRAIN_TIMEOUT_FLAG（link0）
    input  wire        retrain_nack_flag_i,      // 置位 RETRAIN_STATUS_REG.RETRAIN_NACK_FLAG（link0）
    input  wire        replay_buf_full_i,        // replay buffer 满 / credit 耗尽（link0，DFx 观测）
    input  wire [15:0] tx_flit_cnt_i,            // TX_FLIT_CNT_REG 回读（link0，RC）
    input  wire [15:0] rx_flit_cnt_i,            // RX_FLIT_CNT_REG 回读（RC）
    input  wire [15:0] crc_err_cnt_i,            // CRC_ERR_CNT_REG 回读（RC）+ LINK_HEALTH bit1 推导
    input  wire [7:0]  timeout_cnt_i,            // TIMEOUT_CNT_REG 回读（link0，RC）

    // =========================================================================
    // 中断脉冲输入（源域生成，经顶层脉冲同步后送入，apb_clk 域）
    // =========================================================================
    input  wire        irq_credit_overflow_i,    // 置位 IRQ_STATUS_REG.CREDIT_OVERFLOW_IRQ（F-01）
    input  wire        irq_retry_timeout_i,      // 置位 IRQ_STATUS_REG.RETRY_TIMEOUT_IRQ（F-02a）
    input  wire        irq_retry_nack_i,         // 置位 IRQ_STATUS_REG.RETRY_NACK_IRQ（F-02b）
    input  wire        irq_lp_transition_i,      // 置位 IRQ_STATUS_REG.LP_TRANSITION_IRQ（F-03）
    input  wire        irq_seq_mismatch_i,       // 置位 IRQ_STATUS_REG.SEQ_MISMATCH_IRQ（F-04）
    input  wire        irq_dskew_fail_i,         // 置位 IRQ_STATUS_REG.DESKEW_FAIL_IRQ（F-05）
    input  wire        irq_rx_ready_i,           // 置位 IRQ_STATUS_REG.RX_READY_IRQ（G-03，TX 侧收到对端 0x7F）
    input  wire        irq_lane_on_done_i,       // 置位 IRQ_STATUS_REG.LANE_ON_DONE_IRQ（G-04，TX 侧收到 0x40 完成）

    // =========================================================================
    // 额外 TX link（1~3）状态 / IRQ / 计数回读输入（经顶层跨域同步，apb_clk 域）
    // 去 TX_LINKn_EXIST 宏：当前配置 TX_LINK_NUM=4 恒为全链路（link1/2/3 恒有）
    // =========================================================================
    input  wire        tx1_replay_buf_full_i,
    input  wire        tx1_retrain_timeout_flag_i,
    input  wire        tx1_retrain_nack_flag_i,
    input  wire        tx1_irq_retry_timeout_i,
    input  wire        tx1_irq_retry_nack_i,
    input  wire [15:0] tx1_flit_cnt_i,
    input  wire [7:0]  tx1_timeout_cnt_i,
    input  wire        tx2_replay_buf_full_i,
    input  wire        tx2_retrain_timeout_flag_i,
    input  wire        tx2_retrain_nack_flag_i,
    input  wire        tx2_irq_retry_timeout_i,
    input  wire        tx2_irq_retry_nack_i,
    input  wire [15:0] tx2_flit_cnt_i,
    input  wire [7:0]  tx2_timeout_cnt_i,
    input  wire        tx3_replay_buf_full_i,
    input  wire        tx3_retrain_timeout_flag_i,
    input  wire        tx3_retrain_nack_flag_i,
    input  wire        tx3_irq_retry_timeout_i,
    input  wire        tx3_irq_retry_nack_i,
    input  wire [15:0] tx3_flit_cnt_i,
    input  wire [7:0]  tx3_timeout_cnt_i,

    // =========================================================================
    // 中断 / DFx 聚合输出（L2 §2.8）
    // =========================================================================
    output wire        irq_o,                    // 中断聚合输出（IRQ_STATUS 经 MASK 屏蔽 + IRQ_EN 使能，OR 归并为 1bit）
    output wire [31:0] dfx_stat_o                // DFx 状态聚合输出（LINK_HEALTH + 链路状态观测位）
);

    // =========================================================================
    // APB 译码打拍（寄存器通路，与 glue 的 apb_reg_sync_if 同相 1 cycle）
    // =========================================================================
    reg         psel_q;
    reg         penable_q;
    reg         pwrite_q;
    reg  [15:0] paddr_q;
    wire        apb_rd;                          // 读选通
    wire [15:0] paddr;                           // 地址（打拍后，供译码）

    // ---- per-register 读写选通（W1C 清在 RF per-field 段；RC 读清 译码）-------
    wire        rd_clr_tx_flit_cnt;              // RC 读后清零选通
    wire        rd_clr_rx_flit_cnt;
    wire        rd_clr_timeout_cnt;
    wire        rd_clr_crc_err_cnt;
    wire        rd_clr_tx1_cnt;
    wire        rd_clr_tx2_cnt;
    wire        rd_clr_tx3_cnt;

    // ---- 中断置位脉冲 → RF hw_*_wen（组合搬运，非存储）-------------------
    wire        hw_irq_status_reg_credit_overflow_irq_wen;
    wire        hw_irq_status_reg_retry_timeout_irq_wen;
    wire        hw_irq_status_reg_retry_nack_irq_wen;
    wire        hw_irq_status_reg_lp_transition_irq_wen;
    wire        hw_irq_status_reg_seq_mismatch_irq_wen;
    wire        hw_irq_status_reg_deskew_fail_irq_wen;
    wire        hw_irq_status_reg_rx_ready_irq_wen;
    wire        hw_irq_status_reg_lane_on_done_irq_wen;
    wire        hw_retrain_status_reg_retrain_timeout_flag_wen;
    wire        hw_retrain_status_reg_retrain_nack_flag_wen;
    wire        hw_tx1_irq_reg_irq_retry_timeout_wen;
    wire        hw_tx1_irq_reg_irq_retry_nack_wen;
    wire        hw_tx2_irq_reg_irq_retry_timeout_wen;
    wire        hw_tx2_irq_reg_irq_retry_nack_wen;
    wire        hw_tx3_irq_reg_irq_retry_timeout_wen;
    wire        hw_tx3_irq_reg_irq_retry_nack_wen;

    // ---- 中断状态（RF 原生 W1C 存储输出，读自 glue u_csr 实例）--------------
    wire        irq_status_reg_credit_overflow_irq;
    wire        irq_status_reg_retry_timeout_irq;
    wire        irq_status_reg_retry_nack_irq;
    wire        irq_status_reg_lp_transition_irq;
    wire        irq_status_reg_seq_mismatch_irq;
    wire        irq_status_reg_deskew_fail_irq;
    wire        irq_status_reg_rx_ready_irq;
    wire        irq_status_reg_lane_on_done_irq;
    wire        retrain_status_reg_retrain_timeout_flag;
    wire        retrain_status_reg_retrain_nack_flag;
    wire        tx1_irq_reg_irq_retry_timeout;
    wire        tx1_irq_reg_irq_retry_nack;
    wire        tx2_irq_reg_irq_retry_timeout;
    wire        tx2_irq_reg_irq_retry_nack;
    wire        tx3_irq_reg_irq_retry_timeout;
    wire        tx3_irq_reg_irq_retry_nack;
    wire [7:0]  irq_mask_eff;                    // 有效屏蔽（RETRY_ERR_MASK 同屏两路）
    wire [7:0]  irq_masked;

    // ---- 状态 / 计数器聚合寄存（RO / RC 存储，输入已顶层跨域同步）---------------
    reg  [9:0]  status_reg_q;                    // 0x0034（RO 全网关，RF 读回；bit9=RX_LPC,bit8=PHY_READY,G-01）
    reg  [15:0] tx_flit_cnt_q;                   // 0x0040（RC）
    reg  [15:0] rx_flit_cnt_q;                   // 0x0044（RC）
    reg  [7:0]  timeout_cnt_q;                   // 0x004C（RC）
    reg  [15:0] crc_err_cnt_q;                   // 0x005C（RC）
    reg  [3:0]  link_health_q;                   // 0x0050 bit[3:0] LINK_HEALTH 本地推导
    reg  [14:0] dfx_obs_q;                       // dfx_stat_o 观测位
    wire        crc_err_nz;                      // crc_err_cnt_i != 0
    wire        training_done;                   // TRAINING_DONE = (link_fsm_state_tx == NORMAL 4'h4)

    // ---- per-link TX（1~3）STAT/CNT 存储（IRQ 在 RF 原生 W1C）-----------------
    reg  [2:0]  tx1_stat_q;  reg [23:0] tx1_cnt_q;
    reg  [2:0]  tx2_stat_q;  reg [23:0] tx2_cnt_q;
    reg  [2:0]  tx3_stat_q;  reg [23:0] tx3_cnt_q;

    // ---- 置位沿检测打拍（RF RW 输出 → 前值打拍；INIT_CFG / LP_CFG）-------------
    reg  [5:0]  init_cfg_d;
    reg  [5:0]  lp_cfg_d;
    wire [5:0]  init_rise;
    wire [5:0]  lp_rise;

    // ---- 深度 / lane 宏回读值 ----------------------------------------------------
    wire [7:0]  replay_buf_depth_val;
    wire [7:0]  credit_fifo_depth_val;

    // =========================================================================
    // register_flow 生成 CSR（link_layer_apb_reg_sync：apb_reg_sync_if + link_layer_rf）
    // glue 直通信号 + RF RW/WO 配置输出连接线（声明置于使用之前，满足 lint 声明先行）
    // =========================================================================
    wire        pready_glue;
    wire [31:0] prdata_glue;
    // RF RW 配置输出
    wire        phy_mode_reg_s2p_mode;
    wire [3:0]  phy_mode_reg_tx_mode;
    wire [3:0]  phy_mode_reg_rx_mode;
    wire        init_cfg_reg_lane_on;
    wire        init_cfg_reg_lane_off;
    wire        init_cfg_reg_config_done;
    wire        init_cfg_reg_rx_ready;
    wire        init_cfg_reg_phy_init;
    wire        init_cfg_reg_retrain;
    wire [1:0]  bw_cfg_reg_bw_mode;
    wire        mode_reg_work_mode;
    wire        mode_reg_protocol_mode;
    wire        mode_reg_udi_ready_active_en;
    wire        ctrl_reg_tx_rst_n;
    wire        ctrl_reg_tx_lpc;
    wire        ctrl_reg_rx_rst_n;
    wire [3:0]  loopback_cfg_reg_rx0_src;
    wire [3:0]  loopback_cfg_reg_rx1_src;
    wire [3:0]  loopback_cfg_reg_rx2_src;
    wire [3:0]  loopback_cfg_reg_rx3_src;
    wire        loopback_cfg_reg_loopback_en;
    wire        retry_cfg_reg_credit_en;
    wire        retry_cfg_reg_retry_en;
    wire        retry_cfg_reg_crc_en;
    wire        retry_cfg_reg_ecc_en;
    wire [1:0]  retry_cfg_reg_nack_mode;
    wire [15:0] timeout_reg_retry_timeout;
    wire [7:0]  retrain_thr_reg_retrain_timeout_thr;
    wire [7:0]  retrain_thr_reg_retrain_nack_thr;
    wire        retrain_thr_reg_retrain_timeout_en;
    wire        retrain_thr_reg_retrain_nack_en;
    wire        inject_err_link0_reg_inject_en;
    wire        inject_err_link0_reg_front_back;
    wire        inject_err_link1_reg_inject_en;
    wire        inject_err_link1_reg_front_back;
    wire        inject_err_link2_reg_inject_en;
    wire        inject_err_link2_reg_front_back;
    wire        inject_err_link3_reg_inject_en;
    wire        inject_err_link3_reg_front_back;
    wire        irq_mask_reg_credit_overflow_mask;
    wire        irq_mask_reg_retry_err_mask;
    wire        irq_mask_reg_lp_transition_mask;
    wire        irq_mask_reg_irq_en;
    wire        irq_mask_reg_seq_mismatch_mask;
    wire        irq_mask_reg_deskew_fail_mask;
    wire        irq_mask_reg_rx_ready_mask;      // IRQ_MASK_REG.RX_READY_MASK（bit6）
    wire        irq_mask_reg_lane_on_done_mask;  // IRQ_MASK_REG.LANE_ON_DONE_MASK（bit7）
    wire        lp_cfg_reg_lp_enter;
    wire        lp_cfg_reg_lp_accept;
    wire        lp_cfg_reg_lp_reject;
    wire        lp_cfg_reg_lp_sync;
    wire        lp_cfg_reg_lp_exit;
    wire        lp_cfg_reg_lp_exit_done;
    wire [15:0] drift_cfg_reg_drift_period;
    // ACK_ROUTE_CFG_REG.TX0~3_SRC_EN（每 4bit 掩码）
    wire [3:0]  ack_route_cfg_reg_tx0_src_en;
    wire [3:0]  ack_route_cfg_reg_tx1_src_en;
    wire [3:0]  ack_route_cfg_reg_tx2_src_en;
    wire [3:0]  ack_route_cfg_reg_tx3_src_en;
    // CLK_CFG / CLK_CFG_TX / CLK_CFG_RX（动态时钟门控）
    wire        clk_cfg_reg_sram_dyn_en;
    wire        clk_cfg_tx_reg_tx0_dyn;
    wire        clk_cfg_tx_reg_tx1_dyn;
    wire        clk_cfg_tx_reg_tx2_dyn;
    wire        clk_cfg_tx_reg_tx3_dyn;
    wire        clk_cfg_rx_reg_rx0_dyn;
    wire        clk_cfg_rx_reg_rx1_dyn;
    wire        clk_cfg_rx_reg_rx2_dyn;
    wire        clk_cfg_rx_reg_rx3_dyn;

    // =========================================================================
    // APB 打拍译码（与 glue 的 apb_reg_sync_if 打拍同相；pready/prdata 由 glue 直通）
    // =========================================================================
    always @(posedge apb_clk or negedge apb_rst_n) begin
        if (!apb_rst_n) begin
            psel_q    <= 1'b0;
            penable_q <= 1'b0;
            pwrite_q  <= 1'b0;
            paddr_q   <= 16'h0;
        end else begin
            psel_q    <= psel_i;
            penable_q <= penable_i;
            pwrite_q  <= pwrite_i;
            paddr_q   <= paddr_i;
        end
    end

    assign paddr   = paddr_q;
    assign apb_rd  = psel_q & ~pwrite_q & penable_q;

    assign rd_clr_tx_flit_cnt = apb_rd & (paddr == 16'h0040);
    assign rd_clr_rx_flit_cnt = apb_rd & (paddr == 16'h0044);
    assign rd_clr_timeout_cnt = apb_rd & (paddr == 16'h004C);
    assign rd_clr_crc_err_cnt = apb_rd & (paddr == 16'h005C);
    assign rd_clr_tx1_cnt     = apb_rd & (paddr == 16'h0068);
    assign rd_clr_tx2_cnt     = apb_rd & (paddr == 16'h0074);
    assign rd_clr_tx3_cnt     = apb_rd & (paddr == 16'h0080);

    // =========================================================================
    // 深度 / lane 宏回读值（presence 宏选段，段内硬编码）
    // =========================================================================
    assign replay_buf_depth_val =
`ifdef REPLAY_BUF_DEPTH_128
        8'd128;
`elsif REPLAY_BUF_DEPTH_32
        8'd32;
`elsif REPLAY_BUF_DEPTH_16
        8'd16;
`else
        8'd64;
`endif
    assign credit_fifo_depth_val =
`ifdef CREDIT_FIFO_DEPTH_128
        8'd128;
`elsif CREDIT_FIFO_DEPTH_32
        8'd32;
`elsif CREDIT_FIFO_DEPTH_16
        8'd16;
`else
        8'd64;
`endif

    // =========================================================================
    // 中断置位搬运：源域脉冲 → RF hw_*_wen（组合；per-link IRQ 电平到 IRQ_STATUS
    // 对应 bit 的置位脉冲在多拍高期间 OR 聚合——RF per-field 段置位优先，粘滞置位
    // 仅当本拍同时出现同 bit 写 1 清时以置位方胜出，与旧 wrapper 行为一致）
    // =========================================================================
    assign hw_irq_status_reg_credit_overflow_irq_wen = irq_credit_overflow_i;
    assign hw_irq_status_reg_retry_timeout_irq_wen   = irq_retry_timeout_i | tx1_irq_retry_timeout_i |
                                                       tx2_irq_retry_timeout_i | tx3_irq_retry_timeout_i;
    assign hw_irq_status_reg_retry_nack_irq_wen      = irq_retry_nack_i | tx1_irq_retry_nack_i |
                                                       tx2_irq_retry_nack_i | tx3_irq_retry_nack_i;
    assign hw_irq_status_reg_lp_transition_irq_wen   = irq_lp_transition_i;
    assign hw_irq_status_reg_seq_mismatch_irq_wen    = irq_seq_mismatch_i;
    assign hw_irq_status_reg_deskew_fail_irq_wen     = irq_dskew_fail_i;
    assign hw_irq_status_reg_rx_ready_irq_wen        = irq_rx_ready_i;
    assign hw_irq_status_reg_lane_on_done_irq_wen    = irq_lane_on_done_i;
    assign hw_retrain_status_reg_retrain_timeout_flag_wen = retrain_timeout_flag_i;
    assign hw_retrain_status_reg_retrain_nack_flag_wen    = retrain_nack_flag_i;
    assign hw_tx1_irq_reg_irq_retry_timeout_wen      = tx1_irq_retry_timeout_i;
    assign hw_tx1_irq_reg_irq_retry_nack_wen         = tx1_irq_retry_nack_i;
    assign hw_tx2_irq_reg_irq_retry_timeout_wen      = tx2_irq_retry_timeout_i;
    assign hw_tx2_irq_reg_irq_retry_nack_wen         = tx2_irq_retry_nack_i;
    assign hw_tx3_irq_reg_irq_retry_timeout_wen      = tx3_irq_retry_timeout_i;
    assign hw_tx3_irq_reg_irq_retry_nack_wen         = tx3_irq_retry_nack_i;

    // =========================================================================
    // 中断屏蔽 / 聚合输出（读 RF 原生 W1C 输出位；置位 / 写 1 清均在 RF per-field 段）
    // =========================================================================
    assign irq_mask_eff = {irq_mask_reg_lane_on_done_mask, irq_mask_reg_rx_ready_mask,
                           irq_mask_reg_deskew_fail_mask, irq_mask_reg_seq_mismatch_mask,
                           irq_mask_reg_lp_transition_mask, irq_mask_reg_retry_err_mask,
                           irq_mask_reg_retry_err_mask, irq_mask_reg_credit_overflow_mask};
    assign irq_masked   = {irq_status_reg_lane_on_done_irq, irq_status_reg_rx_ready_irq,
                           irq_status_reg_deskew_fail_irq, irq_status_reg_seq_mismatch_irq,
                           irq_status_reg_lp_transition_irq, irq_status_reg_retry_nack_irq,
                           irq_status_reg_retry_timeout_irq, irq_status_reg_credit_overflow_irq}
                          & ~irq_mask_eff;
    assign irq_o        = irq_mask_reg_irq_en ? (|irq_masked) : 1'b0;

    // =========================================================================
    // STATUS_REG（RO 聚合）、各 CNT_REG（RC 读后清零）
    // =========================================================================
    assign training_done = (link_fsm_state_tx_i == 4'h4);

    always @(posedge apb_clk or negedge apb_rst_n) begin
        if (!apb_rst_n) status_reg_q <= 10'b10_0000_0000;   // v1.14：bit9 RX_LPC **复位 1**（名义值），其余 0
        else            status_reg_q <= {status_reg_rx_lpc_i, phy_ready_i, link_fsm_state_tx_i,
                                         lp_state_tx_i, bridge_en_i, training_done};
    end

    always @(posedge apb_clk or negedge apb_rst_n) begin
        if (!apb_rst_n)            tx_flit_cnt_q <= 16'h0;
        else if (rd_clr_tx_flit_cnt) tx_flit_cnt_q <= 16'h0;
        else                         tx_flit_cnt_q <= tx_flit_cnt_i;
    end

    always @(posedge apb_clk or negedge apb_rst_n) begin
        if (!apb_rst_n)            rx_flit_cnt_q <= 16'h0;
        else if (rd_clr_rx_flit_cnt) rx_flit_cnt_q <= 16'h0;
        else                         rx_flit_cnt_q <= rx_flit_cnt_i;
    end

    always @(posedge apb_clk or negedge apb_rst_n) begin
        if (!apb_rst_n)            timeout_cnt_q <= 8'h0;
        else if (rd_clr_timeout_cnt) timeout_cnt_q <= 8'h0;
        else                         timeout_cnt_q <= timeout_cnt_i;
    end

    always @(posedge apb_clk or negedge apb_rst_n) begin
        if (!apb_rst_n)            crc_err_cnt_q <= 16'h0;
        else if (rd_clr_crc_err_cnt) crc_err_cnt_q <= 16'h0;
        else                         crc_err_cnt_q <= crc_err_cnt_i;
    end

    // DFx_STAT_REG.LINK_HEALTH 本地推导 + dfx_stat_o 观测位
    assign crc_err_nz = (crc_err_cnt_i != 16'd0);
    always @(posedge apb_clk or negedge apb_rst_n) begin
        if (!apb_rst_n) begin
            link_health_q <= 4'h0;
            dfx_obs_q     <= 15'h0;
        end else begin
            link_health_q <= {1'b0, retrain_timeout_flag_i, crc_err_nz, retrain_nack_flag_i};
            dfx_obs_q     <= {8'd0, replay_buf_full_i, lp_state_rx_i, link_fsm_state_rx_i};
        end
    end

    assign dfx_stat_o = {13'b0, dfx_obs_q, link_health_q};

    // =========================================================================
    // per-link TX 读回存储（STAT RO / CNT RC；IRQ 为 RF 原生 W1C）
    // =========================================================================
    always @(posedge apb_clk or negedge apb_rst_n) begin
        if (!apb_rst_n) tx1_stat_q <= 3'b000;
        else            tx1_stat_q <= {tx1_retrain_nack_flag_i, tx1_retrain_timeout_flag_i,
                                       tx1_replay_buf_full_i};
    end
    always @(posedge apb_clk or negedge apb_rst_n) begin
        if (!apb_rst_n) tx1_cnt_q <= 24'h0;
        else if (rd_clr_tx1_cnt) tx1_cnt_q <= 24'h0;
        else            tx1_cnt_q <= {tx1_timeout_cnt_i, tx1_flit_cnt_i};
    end

    always @(posedge apb_clk or negedge apb_rst_n) begin
        if (!apb_rst_n) tx2_stat_q <= 3'b000;
        else            tx2_stat_q <= {tx2_retrain_nack_flag_i, tx2_retrain_timeout_flag_i,
                                       tx2_replay_buf_full_i};
    end
    always @(posedge apb_clk or negedge apb_rst_n) begin
        if (!apb_rst_n) tx2_cnt_q <= 24'h0;
        else if (rd_clr_tx2_cnt) tx2_cnt_q <= 24'h0;
        else            tx2_cnt_q <= {tx2_timeout_cnt_i, tx2_flit_cnt_i};
    end

    always @(posedge apb_clk or negedge apb_rst_n) begin
        if (!apb_rst_n) tx3_stat_q <= 3'b000;
        else            tx3_stat_q <= {tx3_retrain_nack_flag_i, tx3_retrain_timeout_flag_i,
                                       tx3_replay_buf_full_i};
    end
    always @(posedge apb_clk or negedge apb_rst_n) begin
        if (!apb_rst_n) tx3_cnt_q <= 24'h0;
        else if (rd_clr_tx3_cnt) tx3_cnt_q <= 24'h0;
        else            tx3_cnt_q <= {tx3_timeout_cnt_i, tx3_flit_cnt_i};
    end

    // =========================================================================
    // 置位沿检测（RF RW 输出持存储，wrapper 采样前值生成 1 cycle 触发脉冲）
    // =========================================================================
    always @(posedge apb_clk or negedge apb_rst_n) begin
        if (!apb_rst_n) begin
            init_cfg_d <= 6'b0;
            lp_cfg_d   <= 6'b0;
        end else begin
            init_cfg_d <= {init_cfg_reg_retrain,   init_cfg_reg_phy_init,
                           init_cfg_reg_rx_ready,  init_cfg_reg_config_done,
                           init_cfg_reg_lane_off,  init_cfg_reg_lane_on};
            lp_cfg_d   <= {lp_cfg_reg_lp_exit_done, lp_cfg_reg_lp_exit,   lp_cfg_reg_lp_sync,
                           lp_cfg_reg_lp_reject,    lp_cfg_reg_lp_accept, lp_cfg_reg_lp_enter};
        end
    end

    assign init_rise = {init_cfg_reg_retrain,  init_cfg_reg_phy_init,
                        init_cfg_reg_rx_ready, init_cfg_reg_config_done,
                        init_cfg_reg_lane_off, init_cfg_reg_lane_on} & ~init_cfg_d;
    assign lp_rise   = {lp_cfg_reg_lp_exit_done, lp_cfg_reg_lp_exit, lp_cfg_reg_lp_sync,
                        lp_cfg_reg_lp_reject, lp_cfg_reg_lp_accept, lp_cfg_reg_lp_enter} & ~lp_cfg_d;

    // =========================================================================
    // 具名配置输出（组合译码，RF RW 输出 → csr_* / cpif_*）
    // =========================================================================
    assign csr_cfg_done_o      = init_cfg_reg_config_done;
    assign csr_perf_mode_o     = mode_reg_work_mode;
    assign csr_protocol_mode_o = mode_reg_protocol_mode;
    assign csr_udi_ready_active_en_o = mode_reg_udi_ready_active_en;
    assign csr_s2p_mode_o      = phy_mode_reg_s2p_mode;
    assign csr_bw_mode_o       = bw_cfg_reg_bw_mode;
    assign csr_loopback_en_o   = loopback_cfg_reg_loopback_en;
    assign csr_loopback_src_o  = {loopback_cfg_reg_rx3_src, loopback_cfg_reg_rx2_src,
                                  loopback_cfg_reg_rx1_src, loopback_cfg_reg_rx0_src};
    assign csr_ack_route_src_en_o = {ack_route_cfg_reg_tx3_src_en, ack_route_cfg_reg_tx2_src_en,
                                     ack_route_cfg_reg_tx1_src_en, ack_route_cfg_reg_tx0_src_en};
    assign csr_clk_cfg_sram_dyn_en_o = clk_cfg_reg_sram_dyn_en;
    assign csr_clk_cfg_tx0_dyn_o    = clk_cfg_tx_reg_tx0_dyn;
    assign csr_clk_cfg_tx1_dyn_o    = clk_cfg_tx_reg_tx1_dyn;
    assign csr_clk_cfg_tx2_dyn_o    = clk_cfg_tx_reg_tx2_dyn;
    assign csr_clk_cfg_tx3_dyn_o    = clk_cfg_tx_reg_tx3_dyn;
    assign csr_clk_cfg_rx0_dyn_o    = clk_cfg_rx_reg_rx0_dyn;
    assign csr_clk_cfg_rx1_dyn_o    = clk_cfg_rx_reg_rx1_dyn;
    assign csr_clk_cfg_rx2_dyn_o    = clk_cfg_rx_reg_rx2_dyn;
    assign csr_clk_cfg_rx3_dyn_o    = clk_cfg_rx_reg_rx3_dyn;
    assign csr_credit_en_o     = retry_cfg_reg_credit_en;
    assign csr_retry_en_o      = retry_cfg_reg_retry_en;
    assign csr_crc_en_o        = retry_cfg_reg_crc_en;
    assign csr_ecc_en_o        = retry_cfg_reg_ecc_en;
    assign csr_nack_mode_o     = retry_cfg_reg_nack_mode;
    assign csr_inj_en_link0_o  = inject_err_link0_reg_inject_en;
    assign csr_inj_fb_link0_o  = inject_err_link0_reg_front_back;
    assign csr_inj_en_link1_o  = inject_err_link1_reg_inject_en;
    assign csr_inj_fb_link1_o  = inject_err_link1_reg_front_back;
    assign csr_inj_en_link2_o  = inject_err_link2_reg_inject_en;
    assign csr_inj_fb_link2_o  = inject_err_link2_reg_front_back;
    assign csr_inj_en_link3_o  = inject_err_link3_reg_inject_en;
    assign csr_inj_fb_link3_o  = inject_err_link3_reg_front_back;
    assign csr_drift_period_o       = drift_cfg_reg_drift_period;
    assign csr_retry_timeout_val_o  = timeout_reg_retry_timeout;
    assign csr_retrain_timeout_thr_o = retrain_thr_reg_retrain_timeout_thr;
    assign csr_retrain_nack_thr_o   = retrain_thr_reg_retrain_nack_thr;
    assign csr_retrain_timeout_en_o = retrain_thr_reg_retrain_timeout_en;
    assign csr_retrain_nack_en_o    = retrain_thr_reg_retrain_nack_en;

    assign csr_init_phy_init_o  = init_rise[4];
    assign csr_init_retrain_o   = init_rise[5];
    assign csr_init_lane_on_o   = init_rise[0];
    assign csr_init_lane_off_o  = init_rise[1];
    assign csr_init_rx_ready_o  = init_rise[3];
    assign csr_lp_cmd_o         = lp_rise;

    // CPIF 控制 output（apb_clk 域不经跨域）
    assign cpif_tx_mode_o  = phy_mode_reg_tx_mode;
    assign cpif_rx_mode_o  = phy_mode_reg_rx_mode;
    assign cpif_tx_rst_n_o = ctrl_reg_tx_rst_n;
    assign cpif_tx_lpc_o   = ctrl_reg_tx_lpc;
    assign cpif_rx_rst_n_o = ctrl_reg_rx_rst_n;

    // RC 读后清零脉冲（CSR 跨域触发）
    assign csr_tx_flit_clr_o  = rd_clr_tx_flit_cnt;
    assign csr_rx_flit_clr_o  = rd_clr_rx_flit_cnt;
    assign csr_crc_err_clr_o  = rd_clr_crc_err_cnt;
    assign csr_tx1_cnt_clr_o  = rd_clr_tx1_cnt;
    assign csr_tx2_cnt_clr_o  = rd_clr_tx2_cnt;
    assign csr_tx3_cnt_clr_o  = rd_clr_tx3_cnt;
    // =========================================================================
    // pslverr：对未实现地址（>0x00A0 或非 4-byte 对齐）报错
    //   （0x0098 CLK_CFG_REG / 0x009C CLK_CFG_TX_REG / 0x00A0 CLK_CFG_RX_REG 已实现，
    //     上限由原 0x0084 扩至 0x00A0；0x0030 为无 RF case 分支地址，读写均落默认
    //     路径不报错——见 W1C 修复规划 v2.1 红线③）
    // =========================================================================
    assign pready_o  = pready_glue;
    assign prdata_o  = prdata_glue;
    assign pslverr_o = pready_glue & ~((paddr <= 16'h00A0) & (paddr[1:0] == 2'b00));
    link_layer_apb_reg_sync #(
        .ADDR_WIDTH (12)
    ) u_csr (
        .pclk        (apb_clk),
        .preset_n    (apb_rst_n),
        .apb_psel    (psel_i),
        .apb_penable (penable_i),
        .apb_pwrite  (pwrite_i),
        .apb_paddr   (paddr_i[11:0]),
        .apb_pwdata  (pwdata_i),
        .apb_pready  (pready_glue),
        .apb_prdata  (prdata_glue),
        // RO 输入（含 W1C hw_*_wen 置位端口）
        .mode_reg_lane_macro_mode            (`LANE_MACRO_MODE),
        .retry_cfg_reg_replay_buf_depth      (replay_buf_depth_val),
        .retry_cfg_reg_credit_fifo_depth     (credit_fifo_depth_val),
        .hw_irq_status_reg_credit_overflow_irq_wen (hw_irq_status_reg_credit_overflow_irq_wen),
        .hw_irq_status_reg_retry_timeout_irq_wen   (hw_irq_status_reg_retry_timeout_irq_wen),
        .hw_irq_status_reg_retry_nack_irq_wen      (hw_irq_status_reg_retry_nack_irq_wen),
        .hw_irq_status_reg_lp_transition_irq_wen   (hw_irq_status_reg_lp_transition_irq_wen),
        .hw_irq_status_reg_seq_mismatch_irq_wen    (hw_irq_status_reg_seq_mismatch_irq_wen),
        .hw_irq_status_reg_deskew_fail_irq_wen     (hw_irq_status_reg_deskew_fail_irq_wen),
        .hw_irq_status_reg_rx_ready_irq_wen        (hw_irq_status_reg_rx_ready_irq_wen),
        .hw_irq_status_reg_lane_on_done_irq_wen    (hw_irq_status_reg_lane_on_done_irq_wen),
        .status_reg_training_done            (status_reg_q[0]),
        .status_reg_bridge_state             (status_reg_q[1]),
        .status_reg_lp_state                 (status_reg_q[3:2]),
        .status_reg_link_fsm_state           (status_reg_q[7:4]),
        .status_reg_phy_ready                (status_reg_q[8]),
        .status_reg_rx_lpc                   (status_reg_q[9]),
        .hw_retrain_status_reg_retrain_timeout_flag_wen (hw_retrain_status_reg_retrain_timeout_flag_wen),
        .hw_retrain_status_reg_retrain_nack_flag_wen    (hw_retrain_status_reg_retrain_nack_flag_wen),
        .tx_flit_cnt_reg_tx_flit_cnt         (tx_flit_cnt_q),
        .rx_flit_cnt_reg_rx_flit_cnt         (rx_flit_cnt_q),
        .timeout_cnt_reg_timeout_cnt         (timeout_cnt_q),
        .dfx_stat_reg_link_health            (link_health_q),
        .dfx_stat_reg_link_fsm_state_rx        (dfx_obs_q[3:0]),
        .dfx_stat_reg_lp_state_rx              (dfx_obs_q[5:4]),
        .dfx_stat_reg_replay_buf_full          (dfx_obs_q[6]),
        .lane_deskew_reg_sel_lane0           (4'd0),
        .lane_deskew_reg_sel_lane1           (4'd0),
        .lane_deskew_reg_sel_lane2           (4'd0),
        .lane_deskew_reg_sel_lane3           (4'd0),
        .lane_deskew_reg_deskew_done         (1'b0),
        .lane_deskew_reg_deskew_fail         (1'b0),
        .crc_err_cnt_reg_crc_err_cnt         (crc_err_cnt_q),
        .tx1_stat_reg_replay_buf_full        (tx1_stat_q[0]),
        .tx1_stat_reg_retrain_timeout_flag   (tx1_stat_q[1]),
        .tx1_stat_reg_retrain_nack_flag      (tx1_stat_q[2]),
        .hw_tx1_irq_reg_irq_retry_timeout_wen (hw_tx1_irq_reg_irq_retry_timeout_wen),
        .hw_tx1_irq_reg_irq_retry_nack_wen    (hw_tx1_irq_reg_irq_retry_nack_wen),
        .tx1_cnt_reg_tx_flit_cnt             (tx1_cnt_q[15:0]),
        .tx1_cnt_reg_timeout_cnt             (tx1_cnt_q[23:16]),
        .tx2_stat_reg_replay_buf_full        (tx2_stat_q[0]),
        .tx2_stat_reg_retrain_timeout_flag   (tx2_stat_q[1]),
        .tx2_stat_reg_retrain_nack_flag      (tx2_stat_q[2]),
        .hw_tx2_irq_reg_irq_retry_timeout_wen (hw_tx2_irq_reg_irq_retry_timeout_wen),
        .hw_tx2_irq_reg_irq_retry_nack_wen    (hw_tx2_irq_reg_irq_retry_nack_wen),
        .tx2_cnt_reg_tx_flit_cnt             (tx2_cnt_q[15:0]),
        .tx2_cnt_reg_timeout_cnt             (tx2_cnt_q[23:16]),
        .tx3_stat_reg_replay_buf_full        (tx3_stat_q[0]),
        .tx3_stat_reg_retrain_timeout_flag   (tx3_stat_q[1]),
        .tx3_stat_reg_retrain_nack_flag      (tx3_stat_q[2]),
        .hw_tx3_irq_reg_irq_retry_timeout_wen (hw_tx3_irq_reg_irq_retry_timeout_wen),
        .hw_tx3_irq_reg_irq_retry_nack_wen    (hw_tx3_irq_reg_irq_retry_nack_wen),
        .tx3_cnt_reg_tx_flit_cnt             (tx3_cnt_q[15:0]),
        .tx3_cnt_reg_timeout_cnt             (tx3_cnt_q[23:16]),
        // W1C 输出
        .irq_status_reg_credit_overflow_irq  (irq_status_reg_credit_overflow_irq),
        .irq_status_reg_retry_timeout_irq    (irq_status_reg_retry_timeout_irq),
        .irq_status_reg_retry_nack_irq       (irq_status_reg_retry_nack_irq),
        .irq_status_reg_lp_transition_irq    (irq_status_reg_lp_transition_irq),
        .irq_status_reg_seq_mismatch_irq     (irq_status_reg_seq_mismatch_irq),
        .irq_status_reg_deskew_fail_irq      (irq_status_reg_deskew_fail_irq),
        .irq_status_reg_rx_ready_irq         (irq_status_reg_rx_ready_irq),
        .irq_status_reg_lane_on_done_irq     (irq_status_reg_lane_on_done_irq),
        .retrain_status_reg_retrain_timeout_flag (retrain_status_reg_retrain_timeout_flag),
        .retrain_status_reg_retrain_nack_flag    (retrain_status_reg_retrain_nack_flag),
        .tx1_irq_reg_irq_retry_timeout       (tx1_irq_reg_irq_retry_timeout),
        .tx1_irq_reg_irq_retry_nack          (tx1_irq_reg_irq_retry_nack),
        .tx2_irq_reg_irq_retry_timeout       (tx2_irq_reg_irq_retry_timeout),
        .tx2_irq_reg_irq_retry_nack          (tx2_irq_reg_irq_retry_nack),
        .tx3_irq_reg_irq_retry_timeout       (tx3_irq_reg_irq_retry_timeout),
        .tx3_irq_reg_irq_retry_nack          (tx3_irq_reg_irq_retry_nack),
        // RW 配置输出
        .phy_mode_reg_s2p_mode       (phy_mode_reg_s2p_mode),
        .phy_mode_reg_tx_mode        (phy_mode_reg_tx_mode),
        .phy_mode_reg_rx_mode        (phy_mode_reg_rx_mode),
        .init_cfg_reg_lane_on        (init_cfg_reg_lane_on),
        .init_cfg_reg_lane_off       (init_cfg_reg_lane_off),
        .init_cfg_reg_config_done    (init_cfg_reg_config_done),
        .init_cfg_reg_rx_ready       (init_cfg_reg_rx_ready),
        .init_cfg_reg_phy_init       (init_cfg_reg_phy_init),
        .init_cfg_reg_retrain        (init_cfg_reg_retrain),
        .bw_cfg_reg_bw_mode          (bw_cfg_reg_bw_mode),
        .mode_reg_work_mode          (mode_reg_work_mode),
        .mode_reg_protocol_mode      (mode_reg_protocol_mode),
        .mode_reg_udi_ready_active_en (mode_reg_udi_ready_active_en),
        .ctrl_reg_tx_rst_n           (ctrl_reg_tx_rst_n),
        .ctrl_reg_tx_lpc             (ctrl_reg_tx_lpc),
        .ctrl_reg_rx_rst_n           (ctrl_reg_rx_rst_n),
        .loopback_cfg_reg_rx0_src    (loopback_cfg_reg_rx0_src),
        .loopback_cfg_reg_rx1_src    (loopback_cfg_reg_rx1_src),
        .loopback_cfg_reg_rx2_src    (loopback_cfg_reg_rx2_src),
        .loopback_cfg_reg_rx3_src    (loopback_cfg_reg_rx3_src),
        .loopback_cfg_reg_loopback_en(loopback_cfg_reg_loopback_en),
        .retry_cfg_reg_credit_en     (retry_cfg_reg_credit_en),
        .retry_cfg_reg_retry_en      (retry_cfg_reg_retry_en),
        .retry_cfg_reg_crc_en        (retry_cfg_reg_crc_en),
        .retry_cfg_reg_ecc_en        (retry_cfg_reg_ecc_en),
        .retry_cfg_reg_nack_mode     (retry_cfg_reg_nack_mode),
        .timeout_reg_retry_timeout   (timeout_reg_retry_timeout),
        .retrain_thr_reg_retrain_timeout_thr (retrain_thr_reg_retrain_timeout_thr),
        .retrain_thr_reg_retrain_nack_thr    (retrain_thr_reg_retrain_nack_thr),
        .retrain_thr_reg_retrain_timeout_en  (retrain_thr_reg_retrain_timeout_en),
        .retrain_thr_reg_retrain_nack_en     (retrain_thr_reg_retrain_nack_en),
        .inject_err_link0_reg_inject_en (inject_err_link0_reg_inject_en),
        .inject_err_link0_reg_front_back(inject_err_link0_reg_front_back),
        .inject_err_link1_reg_inject_en (inject_err_link1_reg_inject_en),
        .inject_err_link1_reg_front_back(inject_err_link1_reg_front_back),
        .inject_err_link2_reg_inject_en (inject_err_link2_reg_inject_en),
        .inject_err_link2_reg_front_back(inject_err_link2_reg_front_back),
        .inject_err_link3_reg_inject_en (inject_err_link3_reg_inject_en),
        .inject_err_link3_reg_front_back(inject_err_link3_reg_front_back),
        .clk_cfg_reg_sram_dyn_en     (clk_cfg_reg_sram_dyn_en),
        .clk_cfg_tx_reg_tx0_dyn      (clk_cfg_tx_reg_tx0_dyn),
        .clk_cfg_tx_reg_tx1_dyn      (clk_cfg_tx_reg_tx1_dyn),
        .clk_cfg_tx_reg_tx2_dyn      (clk_cfg_tx_reg_tx2_dyn),
        .clk_cfg_tx_reg_tx3_dyn      (clk_cfg_tx_reg_tx3_dyn),
        .clk_cfg_rx_reg_rx0_dyn      (clk_cfg_rx_reg_rx0_dyn),
        .clk_cfg_rx_reg_rx1_dyn      (clk_cfg_rx_reg_rx1_dyn),
        .clk_cfg_rx_reg_rx2_dyn      (clk_cfg_rx_reg_rx2_dyn),
        .clk_cfg_rx_reg_rx3_dyn      (clk_cfg_rx_reg_rx3_dyn),
        .irq_mask_reg_credit_overflow_mask (irq_mask_reg_credit_overflow_mask),
        .irq_mask_reg_retry_err_mask       (irq_mask_reg_retry_err_mask),
        .irq_mask_reg_lp_transition_mask   (irq_mask_reg_lp_transition_mask),
        .irq_mask_reg_irq_en               (irq_mask_reg_irq_en),
        .irq_mask_reg_seq_mismatch_mask    (irq_mask_reg_seq_mismatch_mask),
        .irq_mask_reg_deskew_fail_mask     (irq_mask_reg_deskew_fail_mask),
        .irq_mask_reg_rx_ready_mask        (irq_mask_reg_rx_ready_mask),
        .irq_mask_reg_lane_on_done_mask    (irq_mask_reg_lane_on_done_mask),
        .lp_cfg_reg_lp_enter        (lp_cfg_reg_lp_enter),
        .lp_cfg_reg_lp_accept       (lp_cfg_reg_lp_accept),
        .lp_cfg_reg_lp_reject       (lp_cfg_reg_lp_reject),
        .lp_cfg_reg_lp_sync         (lp_cfg_reg_lp_sync),
        .lp_cfg_reg_lp_exit         (lp_cfg_reg_lp_exit),
        .lp_cfg_reg_lp_exit_done    (lp_cfg_reg_lp_exit_done),
        .drift_cfg_reg_drift_period (drift_cfg_reg_drift_period),
        .ack_route_cfg_reg_tx0_src_en (ack_route_cfg_reg_tx0_src_en),
        .ack_route_cfg_reg_tx1_src_en (ack_route_cfg_reg_tx1_src_en),
        .ack_route_cfg_reg_tx2_src_en (ack_route_cfg_reg_tx2_src_en),
        .ack_route_cfg_reg_tx3_src_en (ack_route_cfg_reg_tx3_src_en),
        .test_mode     (1'b0)
    );

endmodule
