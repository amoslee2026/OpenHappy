// ===================================================================
// logic_phy_filelist.f — 逻辑物理层 rtl_v filelist
//
// 版本：v0.2 | 日期：2026-09-10 | 作者：OpenHappy Team
//
// 说明：
//   HIPINEXT_HOME = <hipi_next>/code，所有路径均为相对 HIPINEXT_HOME 的相对路径。
//   编译集合内无 `include 指令（宏文件经 filelist 直接编译）。
//   logic_phy_defines.vh 依赖顶层 hipi_top_defines.vh 先编译（TX_LANE_*/RX_LANE_* 映射推导，
//   同 link_layer_defines.vh 约定）；standalone 编译本 filelist 时，须在外围先编译
//   hipi_top_defines.vh、随后编译 common.f（本 filelist 不含，外围提供）、最后编译
//   hipi_top_undefines.vh（集成流程中 common.f 由顶层 d2d_hipi_dig_top.f 统一引入）。
// 修订：v0.2 (2026-09-10) 新增 apb_csr_top_rf.v（reg-gen 生成寄存器本体, 先于
//       apb_csr_top.v wrapper 编译；源: flow/register_flow/logic_phy_reg_flow/）
// ===================================================================

// --- Compile-time defines（依赖 top hipi_top_defines.vh 先编译）---
$HIPINEXT_HOME/design/rtl_v/logic_phy/logic_phy_defines.vh

// --- TX chain PRBS IP ---
$HIPINEXT_HOME/design/rtl_v/logic_phy/tx_chain/prbs_gen.v

// --- TX chain datapath (stage 1 → 5) ---
$HIPINEXT_HOME/design/rtl_v/logic_phy/tx_chain/tx_mb_pipe.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/tx_chain/tx_msb_lsb.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/tx_chain/tx_io_mapper.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/tx_chain/tx_prbs_scrambler.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/tx_chain/tx_byte_dispatcher.v

// --- RX chain PRBS IP ---
$HIPINEXT_HOME/design/rtl_v/logic_phy/rx_chain/prbs_chk.v

// --- RX chain datapath (stage 1 → 6) ---
$HIPINEXT_HOME/design/rtl_v/logic_phy/rx_chain/rx_mb_pipe.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/rx_chain/rx_msb_lsb.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/rx_chain/rx_io_mapper.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/rx_chain/rx_prbs_descrambler.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/rx_chain/rx_byte_distributor.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/rx_chain/rx_async_fifo.v

// --- TX chain training FSM + datapath integration ---
$HIPINEXT_HOME/design/rtl_v/logic_phy/tx_chain/tx_training_fsm.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/tx_chain/tx_data_path.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/tx_chain/tx_chain.v

// --- Training units (MAS 待编写) ---
$HIPINEXT_HOME/design/rtl_v/logic_phy/train/offset_train.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/train/vref_train.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/train/pi_train.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/train/dcdl_train.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/train/data_checker.v

// --- RX chain training integration ---
$HIPINEXT_HOME/design/rtl_v/logic_phy/rx_chain/rx_training_fsm.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/rx_chain/rx_training.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/rx_chain/rx_data_path.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/rx_chain/rx_chain.v

// --- Sideband ---
$HIPINEXT_HOME/design/rtl_v/logic_phy/sideband/sbc_tx.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/sideband/sbc_rx.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/sideband/sbc_dispatcher.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/sideband/sbc_cpif_event.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/sideband/sideband.v

// --- CSR / interrupt / lane integration / top ---
// apb_csr_top_rf.v: reg-gen 生成（flow/register_flow/logic_phy_reg_flow, 2026-09-10）;
//   勿手改（DO NOT edit manually），excel 变更后经 gen_reg.csh 重新生成
$HIPINEXT_HOME/design/rtl_v/logic_phy/apb_csr_top_rf.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/apb_csr_top.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/intc.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/lane_top.v
$HIPINEXT_HOME/design/rtl_v/logic_phy/mcu_subsys.v

// --- 集中式 CRG ---
$HIPINEXT_HOME/design/rtl_v/logic_phy/logic_phy_crg.v

// --- logic_phy 顶层 ---
$HIPINEXT_HOME/design/rtl_v/logic_phy/logic_phy.v

// --- Compile-time undefines ---
$HIPINEXT_HOME/design/rtl_v/logic_phy/logic_phy_undefines.vh
