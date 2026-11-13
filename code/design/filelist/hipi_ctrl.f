// ===================================================================
// hipi_ctrl_filelist.f — 包 controller 顶层 rtl_v filelist
//
// 版本：v0.8 | 日期：2026-09-18 | 作者：OpenHappy Team
// 修订：v0.8 AXI 协议层改统一窗口：新增 axi4_prcl.f（v1.0，defines → 公共文件 →
//       iodie/cmptdie/asymm 三类专属 .f → undefines，每个 RTL 文件只列一次、无
//       重复文件），本文件 AXI 段仅 -f axi4_prcl.f 一行；v0.7 的强制顺序约束
//       （cmptdie 在前/iodie 在后）随之取消；asymm（axi4_asymm_prcl.f v2.8）重新
//       进入编译集合（裸宏旧模式下 hipi_ctrl_top else 分支可构建）。
// 原版本：v0.7 | 日期：2026-09-18 | 作者：OpenHappy Team
// 修订：v0.7 AXI 双 die 协议层段改强制顺序：cmptdie_axi4_prcl.f 在前、iodie_axi4_prcl.f
//       在后（工具跳过同编译内重复文件，第二次出现的 axi4_defines.vh 不会重新执行——
//       宏窗口由 cmptdie 段 axi4_defines.vh（首次）开启，由 iodie 段末尾 axi4_undefines.vh
//       （首次）统一清理；cmptdie_axi4_prcl.f v1.2 移除其末尾 undefines）。原顺序下
//       cmptdie 专属 RTL 在未开窗状态下编译 → STX_VE_533（宏未定义）。
// 原版本：v0.6 | 日期：2026-09-18 | 作者：OpenHappy Team
// 修订：v0.6 新增 cmptdie_axi4_prcl.f（cmptdie_axi4_prcl，4R1T st 基线协议层）：AXI 协议层
//       现同时编译 iodie + cmptdie（分享 common/axi4 公共 RTL，重复编译仅供验证/过渡）；
//       hipi_ctrl_top 实例选择由 top die 宏决定（hipi_top_defines.vh → AXI_IO_DIE/iodie；
//       hipi_top_defines_stmr.vh → AXI_CMPT_DIE/cmptdie）。原独立的 hipi_ctrl_stmr.f 废止，
//       d2d_hipi_dig_top_stmr.f 改引本文件。
// 原版本：v0.5 | 日期：2026-08-20 | 作者：OpenHappy Team | 修订：OpenHappy Team
// 修订：v0.5 说明段更正：top defines/undefines 不由本文件编译——本文件仅 pcsr → AXI →
//       CXS → link → hipi_ctrl_top；top 宏激活由调用方 d2d_hipi_dig_top.f 顶部自含
//       （hipi_top_defines.vh），宏清理由顶层 d2d_hipi_top.f / d2d_hipi_top_sim.f 末尾
//       唯一执行（hipi_top_undefines.vh）。原"顶层宏→…→顶层宏清理"顺序描述为残留。
// 修订：v0.4 说明段编译顺序叙述对齐正文（v0.3 叙述为旧顺序残留：含"通用库"、top 提前、
//       AXI 末位；实际正文为 defines→pcsr→AXI→CXS→link→top→undefines，通用库不由
//       本文件引入）。
// 修订：v0.3 同步 axi4_asymm_defines.vh v1.2——AXI 协议层 undefines 只清理 AXI_TX_LANE_*/
//       AXI_RX_LANE_*（AXI 自身命名空间），不再清理链路层依赖的裸 TX_LANE_*/RX_LANE_*
//       （由 hipi_top_undefines.vh 统一清理），相关注释更新。
//
// 说明：
//   HIPINEXT_HOME = <hipi_next>/code，所有路径均为相对 HIPINEXT_HOME 的相对路径。
//   本文件为 controller 段 sub-filelist：编译顺序 = pcsr → AXI 协议层（axi4_prcl.f
//   统一窗口：defines → 公共 → iodie/cmptdie/asymm 专属 → undefines；top die 宏选实例）
//   → CXS 协议层（cxs_prcl.f）→ 链路层（link_layer.f）→ hipi_ctrl_top。
//   top 宏激活/清理不由本文件编译：激活由调用方 d2d_hipi_dig_top.f 顶部自含
//   hipi_top_defines.vh，本文件全程在 top 宏激活状态下编译；宏清理由顶层
//   d2d_hipi_top.f / d2d_hipi_top_sim.f 末尾唯一执行（hipi_top_undefines.vh）。
//   通用库（rstn_sync / sync_cell 等）不由本文件引入，由上层 filelist 经 common.f
//   统一提供（如 d2d_hipi_dig_top.f）。
//   CXS / LL 位宽与特性宏在 hipi_top_defines.vh 定义；子 filelist 末尾 undefines 只清理
//   各自命名空间（axi4_asymm_undefines.vh 清 AXI_TX_LANE_*/AXI_RX_LANE_*，不清链路层
//   依赖的裸 TX_LANE_*/RX_LANE_*——由 hipi_top_undefines.vh 统一清理），故子 filelist
//   间无先后强约束；hipi_ctrl_top 须在全部子 filelist 之后、top undefines 之前编译。
//   编译集合内无 `include 指令（宏文件经 filelist 直接编译）。
// ===================================================================

// --- Controller 公共 CSR（top 例化依赖）---
$HIPINEXT_HOME/design/rtl_v/pcsr/pcsr.v

// --- AXI4 协议层统一窗口（axi4_prcl.f v1.0：defines → 公共文件 → iodie/cmptdie/asymm
//      专属 .f → undefines；每个 RTL 文件只列一次、无重复文件；协议层实例由 top die
//      宏选择：AXI_IO_DIE → iodie_axi4_prcl；AXI_CMPT_DIE → cmptdie_axi4_prcl；裸宏
//      旧模式 → axi4_asymm_prcl；未选中者为死模块）---
-F $HIPINEXT_HOME/design/filelist/axi4_prcl.f

// --- CXS 协议层（defines → RTL → undefines；PRCL_AXI 分支下为 dead code，不例化）---

// --- 链路层（defines → RTL → undefines，依赖 top 的 TX_LANE_* 宏）---
-F $HIPINEXT_HOME/design/filelist/link_layer.f

// --- 包 controller top（依赖 top defines 的 CXS/LL 宏，须在子 filelist undefines 前编译）---
$HIPINEXT_HOME/design/rtl_v/top/hipi_ctrl_top.v

