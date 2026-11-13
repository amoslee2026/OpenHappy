// ===================================================================
// d2d_hipi_dig_top.f — D2D HiPi 数字顶层（d2d_hipi_dig_top）全量 filelist
//
// 版本：v0.6 | 日期：2026-08-20 | 作者：OpenHappy Team
// 修订：v0.6 top defines 激活自含化：本文件顶部直接编译 hipi_top_defines.vh
//       （完整 IP filelist 自含，standalone 可编完整集）；顶层 d2d_hipi_top.f /
//       d2d_hipi_top_sim.f 移除自身重复激活行（避免重复 define 告警），保留
//       末尾唯一清理 hipi_top_undefines.vh。
// 修订：v0.5 由 d2d_hipi_dig.f 改名而来（与中端 SDC 流程 flow/d2d_hipi_dig_top/
//       generate_sdc 命名对齐）。
// 修订：v0.3 路径宏 HIPI_NEXT_HOME → HIPINEXT_HOME（项目统一）；同步 hipi_ctrl.f
//       v0.3 编译顺序描述（新增 axi4_asymm_prcl.f，hipi_ctrl_top 移至末位）。
// 修订：v0.4 尾部不再自清 top 宏——清理统一收归上层顶层 filelist 末尾
//       hipi_top_undefines.vh；本文件出口时 top 宏保持激活（供顶层 d2d_hipi_top.v
//       使用）。
//
// 说明：
//   HIPINEXT_HOME = <hipi_next>/code，所有路径均为相对 HIPINEXT_HOME 的相对路径。
//   本文件为完整 IP filelist：顶部自含激活 top 宏（hipi_top_defines.vh），出口时
//   top 宏保持激活；清理由顶层 d2d_hipi_top.f / d2d_hipi_top_sim.f 末尾唯一执行
//   （hipi_top_undefines.vh）。顶层形态宏 LOOPBACK 由顶层 filelist 以 +define 提供。
//   编译顺序：
//     0) hipi_top_defines.vh 顶部激活（本文件自含；top 宏自此保持激活）；
//     1) -f hipi_ctrl.f（pcsr → axi4_prcl.f → cxs_prcl.f → link_layer.f →
//        hipi_ctrl_top；不内含 top defines/undefines，依赖顶部激活的 top 宏；
//        v0.2 起 hipi_ctrl.f 不再内含 common.f）；
//     2) -f common.f（公共库唯一来源，controller / logic_phy 共用）；
//     3) -f logic_phy.f（内含 logic_phy_defines.vh → logic_phy RTL →
//        logic_phy_undefines.vh；LP_* 宏在 logic_phy RTL 段内有效，出段即清）；
//     4) d2d_hipi_dig_top.v（编译时 LP_* 已失效；lane 存在合宏 D2D_LANE{1,2,3}_EN
//        由 hipi_top_defines.vh §4b 单源派生，本段不再自派生/自清。顶部宏保持激活）。
//   编译集合内无 `include 指令（宏文件经 filelist 直接编译）。
// ===================================================================

// --- top 宏激活（本文件自含；完整 IP filelist；清理归顶层 d2d_hipi_top.f / d2d_hipi_top_sim.f）---
$HIPINEXT_HOME/design/rtl_v/top/hipi_top_defines.vh

// --- Controller 段（pcsr → AXI → CXS → link → hipi_ctrl_top；依赖顶部激活的 top 宏）---
-F $HIPINEXT_HOME/design/filelist/hipi_ctrl.f

// --- Common library modules（显式列出）---
-F $HIPINEXT_HOME/design/filelist/common.f

// --- logic_phy 段（内含 LP defines → logic_phy RTL → LP undefines；common.f 由本文件显式引入）---
-F $HIPINEXT_HOME/design/filelist/logic_phy.f

// --- D2D 数字顶层 ---
$HIPINEXT_HOME/design/rtl_v/top/d2d_hipi_dig_top.v
