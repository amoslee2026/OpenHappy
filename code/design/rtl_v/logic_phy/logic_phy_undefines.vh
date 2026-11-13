// ===================================================================
// logic_phy_undefines.vh — 逻辑物理层编译期配置宏清理
//
// 版本：v0.3 | 日期：2026-08-13 | 作者：OpenHappy Team
// 修订：v0.3 与 logic_phy_defines.vh v0.4 同步——新增 per-lane 每方向使能宏清理
//       （LP_TX_LANE{i}_EN / LP_RX_LANE{i}_EN）。
//
// 用途：logic_phy RTL filelist 最末尾编译本文件清理配置宏，
//       与最开头 logic_phy_defines.vh 配对。
// ===================================================================

`undef LP_TX_LANE_NUM
`undef LP_RX_LANE_NUM
`undef LP_LANE_NUM
`undef LP_TX_LANE1_EN
`undef LP_TX_LANE2_EN
`undef LP_TX_LANE3_EN
`undef LP_RX_LANE1_EN
`undef LP_RX_LANE2_EN
`undef LP_RX_LANE3_EN
`undef LP_LANE1_EN
`undef LP_LANE2_EN
`undef LP_LANE3_EN
