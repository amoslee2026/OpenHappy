// ===================================================================
// link_layer_filelist.f — 链路层 rtl_v filelist
//
// 版本：v0.6 | 日期：2026-09-29 | 作者：OpenHappy Team / OpenHappy Team
// 修订：v0.2 OpenHappy Team 链路层全模块 RTL 交付（对齐评审中 v1.0 MAS）：
//       新增 link_tx/link_rx 及其全部 MAS 子模块、link_tx/rx_lcmsm、
//       link_event_process、apb_csr、SRAM 行为模型；补充 +incdir
//       支持模块内 `include "link_layer_defines.vh"。
// 修订：v0.3 移除 link_layer_defines.vh 直接编译（改由模块内 `include 引入），
//       增加 +incdir+<link_layer 目录> 提供 include 搜索路径。
// 修订：v0.4 OpenHappy Team 整合参数：删除 link_layer_params.vh / link_layer_undefines.vh，
//       参数统一并入 link_layer_defines.vh（单一入口），各模块 `include 该文件。
// 修订：v0.6 OpenHappy Team 2026-09-29 编译期参数单一事实源迁移：link_tx_512_lane1 改
//       `include "link_layer_defines.vh"（数值宏 REPLAY_BUF_DEPTH 回归 defines），删除 spec 侧
//       link_layer_params.vh 及 v0.5 补的 +incdir spec 目录。
//
// 说明：
//   HIPINEXT_HOME = <hipi_next>/code，所有路径均为相对 HIPINEXT_HOME 的相对路径。
//   本文件由顶层 hipi_ctrl.f 通过 `-f $HIPINEXT_HOME/design/filelist/link_layer.f` 引用。
//   模块内 `include "link_layer_defines.vh"（带 include guard），需提供 +incdir。
// ===================================================================

// --- Compile-time defines ---
// link_layer_defines.vh 由各模块内 `include "link_layer_defines.vh" 引入（带 include guard），
// 经下方 +incdir+ 提供搜索路径，不再在此直接编译。
// 注：LL_ 体系（原 link_layer_defines.vh，依赖顶层 hipi_top_defines.vh 的 TX_LANE_*/RX_LANE_*）
//     已并入 link_layer_defines.vh §5（当前以注释保留，未被 RTL 使用），此处不再单独编译。
+incdir+$HIPINEXT_HOME/design/rtl_v/link_layer

// --- SRAM behavioral models ---
//$HIPINEXT_HOME/design/rtl_v/link_layer/sram_128x256.v
//$HIPINEXT_HOME/design/rtl_v/link_layer/sram_128x256_1r1w.v

// --- CDC synchronizer (no-reset 2flop) ---
$HIPINEXT_HOME/design/rtl_v/link_layer/sync_cell_norst.v

// --- TX datapath ---
$HIPINEXT_HOME/design/rtl_v/link_layer/crc9_generate.v
$HIPINEXT_HOME/design/rtl_v/link_layer/crc16_generate.v
$HIPINEXT_HOME/design/rtl_v/link_layer/ecc_generate.v
$HIPINEXT_HOME/design/rtl_v/link_layer/replay_buffer.v
$HIPINEXT_HOME/design/rtl_v/link_layer/link_tx.v
$HIPINEXT_HOME/design/rtl_v/link_layer/link_tx_512_lane1.v
$HIPINEXT_HOME/design/rtl_v/link_layer/link_tx_1024_lane1.v
$HIPINEXT_HOME/design/rtl_v/link_layer/link_tx_1024_lane2.v
$HIPINEXT_HOME/design/rtl_v/link_layer/link_tx_compat.v

// --- RX datapath ---
$HIPINEXT_HOME/design/rtl_v/link_layer/crc9_check.v
$HIPINEXT_HOME/design/rtl_v/link_layer/crc16_check.v
$HIPINEXT_HOME/design/rtl_v/link_layer/ecc_check.v
$HIPINEXT_HOME/design/rtl_v/link_layer/credit_fifo_bank.v
$HIPINEXT_HOME/design/rtl_v/link_layer/credit_fifo.v
$HIPINEXT_HOME/design/rtl_v/link_layer/lane_dskew.v
$HIPINEXT_HOME/design/rtl_v/link_layer/link_rx.v
$HIPINEXT_HOME/design/rtl_v/link_layer/link_rx_512_lane1.v
$HIPINEXT_HOME/design/rtl_v/link_layer/link_rx_1024_lane1.v
$HIPINEXT_HOME/design/rtl_v/link_layer/link_rx_1024_lane2.v
$HIPINEXT_HOME/design/rtl_v/link_layer/link_rx_compat.v

// --- Control plane ---
// onehot_to_binary：公共 IP（code/common/，只读引用），link_tx/rx_lcmsm 依赖，
// 由顶层 common.f 统一提供，此处不重复引用（防 module redefined；同 d520591 处理）。
$HIPINEXT_HOME/design/rtl_v/link_layer/link_tx_lcmsm.v
$HIPINEXT_HOME/design/rtl_v/link_layer/link_rx_lcmsm.v
$HIPINEXT_HOME/design/rtl_v/link_layer/link_event_process.v
$HIPINEXT_HOME/design/rtl_v/link_layer/link_ack_route.v
$HIPINEXT_HOME/design/rtl_v/link_layer/apb_csr.v

// --- CSR（register_flow 生成，apb_csr 薄 wrapper 例化；apb_reg_sync_if 为
//     公共 IP 由 common.f 提供，此处不重复引用（防 module redefined））----
$HIPINEXT_HOME/design/rtl_v/link_layer/link_layer_rf.v
$HIPINEXT_HOME/design/rtl_v/link_layer/link_layer_apb_reg_sync.v

// --- Link layer top ---
$HIPINEXT_HOME/design/rtl_v/link_layer/link_layer.v

// --- Compile-time undefines ---
// link_layer_undefines.vh 已废弃（2026-08-13 整合删除）；宏不在此清理。
