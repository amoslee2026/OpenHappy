// =============================================================================
// module name：logic_phy_crg
// author: OpenHappy Team
// MAS：spec/logic_phy/mas/logic_phy_crg_mas.md（本模块为纯单元调用，无独立 spec）
// =============================================================================
// 功能概述: HiPi 逻辑物理层集中式复位同步（Clock & Reset Generator）模块：
//   mb_rst_n / sb_rst_n 各目标时钟域 async assert + sync deassert 后统一分发，
//   收敛去重 sideband 叶模块局部同步器并修复 Group C 裸异步复位缺口。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//  2026-08-14  OpenHappy Team  初始版本：基于 logic_phy_crg_spec v1.0 / logic_phy_crg_mas v1.0
//                       集中式复位同步，复用 code/common/rstn_sync.v（RESET_SYNC_STAGE=2，
//                       DFT 旁路）；输出 mb_rstn_sync / rx_rstn_sync[] / sb_tx_rstn_sync /
//                       sb_rx_rstn_sync 四类已同步释放复位
//  2026-08-24  OpenHappy Team  MCU/APB 复位域集中到本模块同步释放：新增 u_sync_apb（clk=apb_pclk）/
//                       u_sync_mcu（clk=mcu_clk），async 源 mcu_rst_n，输出 apb_rstn_sync /
//                       mcu_rstn_sync；废除原"顶层预同步 apb_preset_n=mcu_rst_n 不重同步"假设
//  2026-08-24  OpenHappy Team  APB 域异步源独立化：logi_phy 新增 presetn 端口，APB 域改用
//                       presetn（u_sync_apb.resetn_in=presetn），MCU 域保留 mcu_rst_n
//                       （u_sync_mcu.resetn_in=mcu_rst_n）；两域独立异步源 + 独立同步器
//  2026-10-20  OpenHappy Team  R7：sb_rx_rstn_sync 改在 sb_rx_clk 负沿同步释放（与 sbc_rx 负沿
//                       触发器同沿）；原复用 rstn_sync 默认正沿 ⇒ 复位正沿释放、数据负沿
//                       采样，只剩半周期余量，违反 MAS §2.2 "sb_rx_clk（负沿）域"同域
//                       同步释放要求。实现用 rstn_sync 的负沿兄弟模块 rstn_sync_neg
//                       （跟随库内 generic_icg_pos/neg、generic_dff_q/qn 的极性成对 wrapper
//                       惯例；rstn_sync.v 本体零改动，33 处既有实例回归风险归零）
// =============================================================================

`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// 设计说明:
//
// 集成假设 / 与 spec、interface 偏差说明:
//   1. LANE_NUM 由 logic_phy 顶层按 LP_LANE_NUM 宏派生传入（默认 2，baseline 2 lane）。
//   2. mb_rst_n 驱动 mb_clk 域 + 每 lane phy_rx_clk{i} 域（u_sync_mb / u_sync_rx[i]）；
//      sb_rst_n 驱动 sb_tx_clk_in 域 + sb_rx_clk 负沿域（u_sync_sb_tx / u_sync_sb_rx）。
//   3. SBC 工作域同步源用自由运行 sb_tx_clk_in，绝不用门控随路 sb_tx_clk（停钟卡死 deassert）。
//   4. sb_rx_clk 为 source-sync 负沿随路时钟，独立同步器；deassert 须在停钟前完成。
//   5. MCU/APB 复位域由本模块统一同步释放：APB 域用 presetn async assert + sync deassert
//      至 apb_pclk 域（u_sync_apb → apb_rstn_sync）；MCU 域用 mcu_rst_n async assert +
//      sync deassert 至 mcu_clk 域（u_sync_mcu → mcu_rstn_sync）。apb_pclk / mcu_clk
//      同源同步，仍各自独立同步器（异步源也独立：APB=presetn，MCU=mcu_rst_n）；输出
//      apb_rstn_sync / mcu_rstn_sync 供 apb_csr_top / intc / mcu_subsys 消费。
//      原"顶层预同步（apb_preset_n = mcu_rst_n）本模块不重同步"已废除。
//   6. DFT：test_mode / test_resetn_in 单点扇入所有 rstn_sync 实例（旁路一致）。
// =============================================================================

module logic_phy_crg #(
    parameter LANE_NUM   = 2,
    parameter RX_EN_BITS = {LANE_NUM{1'b1}}   // per-lane RX 使能位：1=lane i RX 同步器例化；默认全开（向后兼容）
) (
    // clock
    input  wire                mb_clk,
    input  wire [LANE_NUM-1:0] phy_rx_clk,
    input  wire                sb_tx_clk_in,
    input  wire                sb_rx_clk,
    // async hard reset
    input  wire                mb_rst_n,
    input  wire                sb_rst_n,
    input  wire                apb_pclk,     // APB 配置时钟（与 mcu_clk 同源同步）
    input  wire                mcu_clk,      // MCU 工作时钟（与 apb_pclk 同源同步）
    input  wire                mcu_rst_n,    // MCU 异步硬复位，低有效
    input  wire                presetn,      // APB 异步硬复位，低有效
    // DFT
    input  wire                test_mode,
    input  wire                test_resetn_in,
    // synced reset output (per clock domain)
    output wire                mb_rstn_sync,
    output wire [LANE_NUM-1:0] rx_rstn_sync,
    output wire                sb_tx_rstn_sync,
    output wire                sb_rx_rstn_sync,
    output wire                apb_rstn_sync,   // apb_pclk 域同步释放复位
    output wire                mcu_rstn_sync    // mcu_clk 域同步释放复位
);

// ---------------------------------------------------------------------------
// 1. mb_clk 域：mb_rst_n async assert / sync deassert → mb_rstn_sync
//    (tx_chain mb、rx_async_fifo 读侧、sideband Group-C(mb_clk))
// ---------------------------------------------------------------------------
rstn_sync #(
    .RESET_SYNC_STAGE (2)
) u_sync_mb (
    .test_mode     (test_mode     ),
    .test_resetn_in(test_resetn_in),
    .resetn_in     (mb_rst_n      ),
    .clk           (mb_clk        ),
    .resetn_out    (mb_rstn_sync  )
);

// ---------------------------------------------------------------------------
// 2. phy_rx_clk{i} 域（per lane）：mb_rst_n async assert / sync deassert
//    → rx_rstn_sync[i]（rx_chain、rx_async_fifo 写侧、sideband Group-C(phy_rx_clk_w)）
// ---------------------------------------------------------------------------
genvar i;
generate
    for (i = 0; i < LANE_NUM; i = i + 1) begin : gen_rx_sync
        // 4T1R / 部分 RX 使能配置（4T1R：LP_RX_EN_BITS=4'b0001 仅 lane0 RX）：RX 禁能
        // lane 无 phy_rx_clk（lane_top RX_EN=0 → clk tie0）→ 同步器不例化（结构收窄，
        // 非 tie0 时钟 + 定位 flop，designer cdc-needs-fix 结论），消除 CDC Clock_info03c
        // （禁用 lane 同步寄存器 clk 恒定）；该 lane 输出复位恒 deasserted（=1），
        // 接口位宽 [LANE_NUM-1:0] 不变，lane_top RX_EN=0 不消费
        if (RX_EN_BITS[i]) begin : gen_rx_sync_on
            rstn_sync #(
                .RESET_SYNC_STAGE (2)
            ) u_sync_rx (
                .test_mode     (test_mode              ),
                .test_resetn_in(test_resetn_in         ),
                .resetn_in     (mb_rst_n               ),
                .clk           (phy_rx_clk[i]          ),
                .resetn_out    (rx_rstn_sync[i]        )
            );
        end
        else begin : gen_rx_sync_tieoff
            assign rx_rstn_sync[i] = 1'b1;
        end
    end
endgenerate

// ---------------------------------------------------------------------------
// 3. sb_tx_clk_in 域：sb_rst_n async assert / sync deassert → sb_tx_rstn_sync
//    (sbc_tx / sbc_dispatcher / sbc_cpif_event / sideband；替换原 4 个局部同步器)
// ---------------------------------------------------------------------------
rstn_sync #(
    .RESET_SYNC_STAGE (2)
) u_sync_sb_tx (
    .test_mode     (test_mode      ),
    .test_resetn_in(test_resetn_in ),
    .resetn_in     (sb_rst_n       ),
    .clk           (sb_tx_clk_in   ),
    .resetn_out    (sb_tx_rstn_sync)
);

// ---------------------------------------------------------------------------
// 4. sb_rx_clk 负沿域：sb_rst_n async assert / sync deassert → sb_rx_rstn_sync
//    (sbc_rx 接收域；替换原局部同步器)
//    用 rstn_sync_neg（rstn_sync 的负沿极性兄弟模块，同 generic_icg_pos/neg、
//    generic_dff_q/qn 的成对 wrapper 惯例）：同步 release 落在 sb_rx_clk 负沿
//    （与 sbc_rx 负沿触发器同沿），否则复位在正沿释放、数据在负沿采样只剩半周期
//    余量（MAS §2.2 负沿域）。综合映射原生负沿单元 SNDRNQV*（CKN），零 SDC 增量。
// ---------------------------------------------------------------------------
rstn_sync_neg #(
    .RESET_SYNC_STAGE (2)
) u_sync_sb_rx (
    .test_mode     (test_mode      ),
    .test_resetn_in(test_resetn_in ),
    .resetn_in     (sb_rst_n       ),
    .clk           (sb_rx_clk      ),
    .resetn_out    (sb_rx_rstn_sync)
);

// ---------------------------------------------------------------------------
// 5. apb_pclk 域：presetn async assert / sync deassert → apb_rstn_sync
//    (apb_csr_top / intc / mcu_subsys APB 配置域)。apb_pclk 与 mcu_clk 同源同步，
//    但与 MCU 域各自独立同步器，APB 域异步源独立用 presetn
// ---------------------------------------------------------------------------
rstn_sync #(
    .RESET_SYNC_STAGE (2)
) u_sync_apb (
    .test_mode     (test_mode     ),
    .test_resetn_in(test_resetn_in),
    .resetn_in     (presetn       ),
    .clk           (apb_pclk      ),
    .resetn_out    (apb_rstn_sync )
);

// ---------------------------------------------------------------------------
// 6. mcu_clk 域：mcu_rst_n async assert / sync deassert → mcu_rstn_sync
//    (mcu_subsys MCU 核)。与 APB 域同源同步但各自独立同步器
// ---------------------------------------------------------------------------
rstn_sync #(
    .RESET_SYNC_STAGE (2)
) u_sync_mcu (
    .test_mode     (test_mode     ),
    .test_resetn_in(test_resetn_in),
    .resetn_in     (mcu_rst_n     ),
    .clk           (mcu_clk       ),
    .resetn_out    (mcu_rstn_sync )
);

endmodule
