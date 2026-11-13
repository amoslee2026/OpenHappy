// ===================================================================
// axi4_prcl.f — AXI4 协议层统一 filelist（iodie / cmptdie / asymm 三类共用）
//
// 版本：v1.1 | 日期：2026-09-20 | 作者：OpenHappy Team
// 修订：v1.1 注释掉 cmptdie 段（cmptdie 形状+PROTOCOL_LAYER_INFO 常量 st 窗口敏感，
//       本 filelist 仅服务 4T1R 系流程，其中 cmptdie 恒为 dead；st 系流程走
//       axi4_prcl_stmr.f 由 cmptdie plain 首编。iodie/asymm 段保持原样）。
//
// 说明：
//   HIPINEXT_HOME = <hipi_next>/code，所有路径均为相对 HIPINEXT_HOME 的相对路径。
//   本 filelist 为 AXI4 协议层唯一编译入口，统一 axi4 宏窗口：
//     头部 axi4_defines.vh 开窗 → 公共文件 → 三类专属 .f → 尾部 axi4_undefines.vh 关窗。
//   窗口内每个 RTL 文件只列一次（无重复文件）——规避工具跳过同编译内重复文件
//   导致第二段 defines 不重执行的 STX_VE_533 问题（见 hipi_ctrl.f v0.7 修订史）。
//   三类协议层选择由 hipi_ctrl_top.v 的 `ifdef 分支决定（AXI_CMPT_DIE > AXI_IO_DIE >
//   裸宏 asymm），未选中的类为死模块（编译在、不例化）；例外：cmptdie 段已注释
//   （v1.1）——st 窗口敏感，改由 axi4_prcl_stmr.f 在 st 流程中首编。
//   die 源宏 MASTER_DIE/SLAVE_DIE/MST_*/SLV_* 由 top hipi_top_defines(.vh/_stmr.vh) 定义，
//   须先于本 filelist 编译（本窗口只派生 axi4 命名空间宏，不清理 die 源宏）。
//
//   公共文件 = 被 ≥2 类协议层引用的文件（common/ 基础模板、axi4/ bond-unbond-
//   b_tx_demux、pack/unpack 共享族、回环模块）；专属文件 = 该类协议层独有的 .v，
//   各归其专属 .f（经 -f 吃入，仅含 .v 文件，无 defines/undefines/公共文件）：
//     - iodie_axi4_prcl.f   (iodie_axi4_csr + iodie_axi4_prcl)
//     - cmptdie_axi4_prcl.f (cmptdie_axi4_csr + cmptdie_axi4_prcl)
//     - axi4_asymm_prcl.f   (ar_tx_demux / wch_pack / wch_unpack + p_axi4_asymm_csr
//                            + axi4_asymm_prcl / req / resp)
//   编译集合内无 `include 指令（宏文件经 filelist 直接编译）。
// ===================================================================

// --- Compile-time defines（窗口唯一开窗者；派生 AXI4 通道 CN / 位宽 / DIE_MODE /
//     LANE_NUM / PROTOCOL_LAYER_INFO 等 axi4 命名空间宏）---
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/axi4/axi4_defines.vh

// --- 公共文件：common/ 基础模板（三类协议层共用）---
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/common/p_axis_async_fifo.v
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/common/p_crd_gen.v
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/common/p_crd_tx_ctrl.v
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/common/p_ovrflow_det.v
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/common/p_req_timeout.v

// --- 公共文件：axi4/ bond-unbond-demux（三类协议层共用）---
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/axi4/p_axi4_bond.v
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/axi4/p_axi4_unbond.v
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/axi4/p_axi4_b_tx_demux.v

// --- 公共文件：pack/unpack 共享族（被 ≥2 类协议层引用，各只列一次）---
// p_axi4_rwch_pack / p_axi4_rwch_unpack — iodie + cmptdie
// p_axi4_rch_pack                       — iodie + asymm
// p_axi4_rch_unpack                     — cmptdie + asymm
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/axi4/p_axi4_rwch_pack.v
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/axi4/p_axi4_rwch_unpack.v
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/axi4/p_axi4_rch_pack.v
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/axi4/p_axi4_rch_unpack.v

// --- 公共文件：回环模块（iodie/cmptdie 顶层例化；local_loop=p_axi_loop、
//     remote_loop=p_paif_loop）---
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/common/p_axi_loop.v
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/common/p_paif_loop.v
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/common/p_data_byp.v
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/common/p_data_pip.v

// --- 三类协议层专属（各自 .f 仅含该类专属 .v；均在 defines/undefines 窗口内，
//     顺序无约束）---
// iodie（AXI_IO_DIE 模式下由 hipi_ctrl_top 例化）
// cmptdie（AXI_CMPT_DIE 模式下由 hipi_ctrl_top 例化）
// [2026-09-20 注释] 本 filelist 仅被 4T1R 系流程引用（standalone 4T1R / 裸宏 / 2-die v2 窗口 A），
//   其中 AXI_CMPT_DIE 恒关、cmptdie 零例化；且 cmptdie 形状（MST_RX_LANE_n）+ PROTOCOL_LAYER_INFO
//   常量均为 st 窗口敏感，若在此首编，2-die v2 单 elaboration 的 dup-skip 会使窗口 B 的 st 形态
//   永不生效。st 系流程（d2d_hipi_dig_top_stmr.f）走 axi4_prcl_stmr.f，由 cmptdie plain 首编。
// -f $HIPINEXT_HOME/design/filelist/cmptdie_axi4_prcl.f
// asymm（PRCL_AXI 开且两 die 宏关的裸宏旧模式下由 hipi_ctrl_top 例化）
// symm（AXI_ASYM_MODE 未定义、对称模式下由 hipi_ctrl_top 例化 axi4_symm_prcl）
-F $HIPINEXT_HOME/design/filelist/axi4_symm_prcl.f

// --- Compile-time undefines（窗口唯一关窗者；清理 axi4_defines.vh 定义的全部宏）---
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/axi4/axi4_undefines.vh
