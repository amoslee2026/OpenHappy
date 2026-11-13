// ===================================================================
// hipi_top_undefines.vh — 包 controller 顶层编译期配置宏清理
//
// 版本：v0.13 | 日期：2026-09-18 | 作者：OpenHappy Team / OpenHappy Team
// 修订：v0.13 补清 CXS 态宽度宏 `undef CXSPRCL_CNTL_CHK_BYTE（CXS 配置下 defines 的
//       `else 分支定义该宏，v0.12 及以前历史遗漏；st 版 hipi_top_undefines_stmr.vh
//       v0.12-stmr 同步补齐）。PRCL 配置下该分支不活，原 22→0 结论不受影响。
// 原版本：v0.12 | 日期：2026-09-18 | 作者：OpenHappy Team / OpenHappy Team
// 修订：v0.12 补清 CPIF 宽度开关宏 `undef CPIF_DATA_WIDTH_EQ_1024（hipi_top_defines.vh
//       无条件定义该宏，v0.11 及以前历史遗漏未清；与 st 版 hipi_top_undefines_stmr.vh
//       v0.11-stmr 对齐）。defines 无新增宏，版本同步性不受影响。
// 原版本：v0.11 | 日期：2026-09-17 | 作者：OpenHappy Team / OpenHappy Team
// 修订：v0.11 与 hipi_top_defines.vh v0.19 同步——新增 LOOPBACK 跨 die TX 源选择宏
//       清理（`undef LB_D{0,1}_TX_SEL_LANE{1,2,3}，§4b0）
// 修订：v0.10 与 hipi_top_defines.vh v0.16 同步——新增 AXI 数据位宽宏清理
//       （`undef AXI_DATA_WIDTH / `undef AXI_DATA_WIDTH_EQ_256）
// 修订：v0.9 与 hipi_top_defines.vh v0.15 同步——die 开关去 AXI_ 前缀改裸宏
//       （MASTER_DIE/SLAVE_DIE），清理改 `undef MASTER_DIE / `undef SLAVE_DIE
// 修订：v0.8 与 hipi_top_defines.vh v0.14 同步——移除无前缀 lane 裸宏
//       （MST_*/SLV_*）清理（OpenHappy Team axi4 侧 lane 裸宏另行对齐）
// 修订：v0.7 与 hipi_top_defines.vh v0.13 同步——新增 AXI_MASTER_DIE/AXI_SLAVE_DIE die 开关
//       清理；AXI_MST/SLV lane 真源宏改由 §4 PRCL 分支派生（清理保留）
// 修订：v0.6 与 hipi_top_defines.vh v0.12 同步——新增 AXI_MST/SLV_TX/RX_LANE_* 真源宏 + 派生
//       裸宏 MASTER_DIE/SLAVE_DIE/MST_*_LANE_*/SLV_*_LANE_* 清理
// 修订：v0.5 与 hipi_top_defines.vh v0.10 同步——新增 PHY_LANE_1/2/3 清理
// 修订：v0.2 与 hipi_top_defines.vh v0.3 同步——AXI_REQ_MODE/AXI_RESP_MODE 移除，
//       改 `undef AXI_MASTER_DIE_MODE
// 修订：v0.3 与 hipi_top_defines.vh v0.4 同步——新增 LL 位宽 + CXS 位宽/特性宏清理
// 修订：v0.4 与 hipi_top_defines.vh v0.5 同步——PRCL_AXI4/5/CXS 合并为 PRCL_AXI
//
// 用途：控制器顶层 RTL filelist 最末尾 `include 本文件清理配置宏，
//       与最开头 hipi_top_defines.vh 配对。
// ===================================================================

`undef PRCL_AXI
`undef AXI_ASYM_MODE
// --- iodie（IO Die AXI 协议层）开关清理 ---
`undef AXI_IO_DIE
// --- AXI 数据位宽（v0.16 起由 PRCL_AXI 分支派生），清理 ---
`undef AXI_DATA_WIDTH
`undef AXI_DATA_WIDTH_EQ_256
// --- AXI_MASTER_DIE_MODE 单向二选一制废弃（v0.12 起改 AXI_MST/SLV dual-mode），保留清理以防残留 ---
`undef AXI_MASTER_DIE_MODE
// --- AXI dual-mode die 开关（无前缀裸宏 MASTER_DIE/SLAVE_DIE，与 OpenHappy Team axi4 协议层共用）---
`undef MASTER_DIE
`undef SLAVE_DIE
// --- AXI mst/slv lane 真源宏（裸宏无 AXI_ 前缀；由 §4 PRCL 分支派生）---
`undef MST_TX_LANE_3
`undef MST_TX_LANE_2
`undef MST_TX_LANE_1
`undef MST_RX_LANE_3
`undef MST_RX_LANE_2
`undef MST_RX_LANE_1
`undef SLV_TX_LANE_3
`undef SLV_TX_LANE_2
`undef SLV_TX_LANE_1
`undef SLV_RX_LANE_3
`undef SLV_RX_LANE_2
`undef SLV_RX_LANE_1
// --- PAIF / CPIF / CXS 数据位宽与特性宏 ---
`undef PAIF_DATA_WIDTH
`undef CPIF_DATA_WIDTH
`undef CPIF_DATA_WIDTH_EQ_1024
`undef CXSPRCL_DATAFLITWIDTH
`undef CXSPRCL_CNTL_WIDTH
`undef CXSPRCL_CNTL_CHK_BYTE
`undef CXSPRCL_SRCID_WIDTH
`undef CXSPRCL_TGTID_WIDTH
`undef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
`undef CXSPRCL_LAST
`undef PRCL_TX_LANE_3
`undef PRCL_TX_LANE_2
`undef PRCL_TX_LANE_1
`undef PRCL_RX_LANE_3
`undef PRCL_RX_LANE_2
`undef PRCL_RX_LANE_1
`undef PHY_LANE_3
`undef PHY_LANE_2
`undef PHY_LANE_1
`undef D2D_LANE1_EN
`undef D2D_LANE2_EN
`undef D2D_LANE3_EN
// --- 每 lane 每方向使能宏（v0.19 起由 hipi_top_defines.vh 单源派生，§4c）清理 ---
`undef D2D_PHY_TX_LANE1_EN
`undef D2D_PHY_TX_LANE2_EN
`undef D2D_PHY_TX_LANE3_EN
`undef D2D_PHY_RX_LANE1_EN
`undef D2D_PHY_RX_LANE2_EN
`undef D2D_PHY_RX_LANE3_EN
// --- LOOPBACK 跨 die 主带 TX 源选择宏（hipi_top_defines.vh §4b0，仅 d2d_hipi_top.v
//     LOOPBACK 形态消费）清理 ---
`undef LB_D0_TX_SEL_LANE1
`undef LB_D0_TX_SEL_LANE2
`undef LB_D0_TX_SEL_LANE3
`undef LB_D1_TX_SEL_LANE1
`undef LB_D1_TX_SEL_LANE2
`undef LB_D1_TX_SEL_LANE3
