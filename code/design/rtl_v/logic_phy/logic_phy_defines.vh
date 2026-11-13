// ===================================================================
// logic_phy_defines.vh — 逻辑物理层编译期配置宏（LP_ 前缀）
//
// 版本：v0.5 | 日期：2026-08-28 | 作者：OpenHappy Team
// 修订：v0.5 4T1R baseline（非对称 4 TX + 1 RX）：PHY_LANE_2/3 使能（4 物理 lane），
//       lane0 双向 TX+RX，lane1~3 仅 TX（LP_RX_LANE{1,2,3}_EN 注释）→ LP_TX_LANE_NUM=4 /
//       LP_RX_LANE_NUM=1 / LP_LANE_NUM=4；与 logic_phy_spec v4.16/4.17 / lane_top_spec v1.23/1.24 对齐
//
// 用途：logic_phy RTL 统一获取 lane 配置宏。RTL filelist 最开头编译本文件，
//       最末尾编译 logic_phy_undefines.vh（RTL 内不 `include）。
//       依赖 top 层 hipi_top_defines.vh 先编译（提供 PHY_LANE_* 宏）。
//
// 派生规则（v0.3 起物理 lane 与 ctrl link 组数解耦；v0.4 起 TX/RX 使能按 lane 独立）：
//   PHY_LANE_*            = 逻辑层物理 lane 槽位（决定 LP_LANE_NUM / LP_LANE{i}_EN）。
//   LP_TX_LANE{i}_EN / LP_RX_LANE{i}_EN = lane i 的 TX / RX 每方向独立使能
//     （默认随 PHY_LANE_n 全开，可单独注释关某 lane 的 TX 或 RX；支持非连续）。
//   LP_LANE{i}_EN         = lane i 槽位存在（=PHY_LANE_i 派生；lane0 恒在无需宏）——
//     用于 logic_phy.v 顶端口按 interface 原文逐 lane 展开时的 `ifdef 条件声明
//     （Verilog-2005 预处理器不支持数值比较，端口有无只能按宏判断）。
//   LP_LANE_NUM           = 物理 lane 槽位数（= PHY_LANE_*，即 LP_LANE_NUM）。
//   LP_TX_LANE_NUM / LP_RX_LANE_NUM = 该方向最大连续例化 lane 数（由离散宏链推导，
//     仅供自定义外部引用；例化判定不再走 count，改在 logic_phy.v 内按每方向宏掩码完成）。
//   lane i 的 TX/RX 例化使能 = LP_TX_LANE{i}_EN / LP_RX_LANE{i}_EN（逐 lane 独立）；
//     该判定在 logic_phy.v 内以离散掩码 localparam 完成，不经 count 比较。
// ===================================================================

// --- per-lane 每方向使能（离散，独立控制；lane 槽位由 PHY_LANE_* 决定）---
// ★ 方向使能元核已上移至 hipi_top_defines.vh §4c（D2D_TX/RX_LANE{i}_EN，贯穿全程），
//   本文件只做 LP_ 别名映射（LP_TX/RX_LANE{i}_EN = ifdef D2D_TX/RX_LANE{i}_EN），
//   保证 logic_phy.v 内 `ifdef LP_TX/RX_LANE` 一致，且与 dig/top 方向守卫同源。
//   4T1R baseline（2026-08-28）：LANE0 双向 TX+RX；LANE1~3 仅 TX（D2D_PHY_RX_LANE{1,2,3}_EN 未定义）。
`ifdef D2D_PHY_TX_LANE1_EN
    `define LP_TX_LANE1_EN      // lane1 仅 TX
`endif
`ifdef D2D_PHY_RX_LANE1_EN
    `define LP_RX_LANE1_EN
`endif
`ifdef D2D_PHY_TX_LANE2_EN
    `define LP_TX_LANE2_EN      // lane2 仅 TX
`endif
`ifdef D2D_PHY_RX_LANE2_EN
    `define LP_RX_LANE2_EN
`endif
`ifdef D2D_PHY_TX_LANE3_EN
    `define LP_TX_LANE3_EN      // lane3 仅 TX
`endif
`ifdef D2D_PHY_RX_LANE3_EN
    `define LP_RX_LANE3_EN
`endif

// --- lane i 槽位存在（PHY_LANE_* 派生；lane0 恒在无需宏）---
`ifdef PHY_LANE_1
    `define LP_LANE1_EN
`endif
`ifdef PHY_LANE_2
    `define LP_LANE2_EN
`endif
`ifdef PHY_LANE_3
    `define LP_LANE3_EN
`endif

// --- 方向最大连续 lane 数（离散宏链推导；仅供外部参考，例化判据见 logic_phy.v 掩码）---
`ifdef LP_TX_LANE3_EN
    `define LP_TX_LANE_NUM 4
`elsif LP_TX_LANE2_EN
    `define LP_TX_LANE_NUM 3
`elsif LP_TX_LANE1_EN
    `define LP_TX_LANE_NUM 2
`else
    `define LP_TX_LANE_NUM 1
`endif
`ifdef LP_RX_LANE3_EN
    `define LP_RX_LANE_NUM 4
`elsif LP_RX_LANE2_EN
    `define LP_RX_LANE_NUM 3
`elsif LP_RX_LANE1_EN
    `define LP_RX_LANE_NUM 2
`else
    `define LP_RX_LANE_NUM 1
`endif

// --- logic_phy 例化 lane 槽位数 = 物理 lane 数（由 PHY_LANE_* 派生）---
`ifdef PHY_LANE_3
    `define LP_LANE_NUM 4
`elsif PHY_LANE_2
    `define LP_LANE_NUM 3
`elsif PHY_LANE_1
    `define LP_LANE_NUM 2
`else
    `define LP_LANE_NUM 1
`endif
