// ===================================================================
// common.f — 通用模块库 filelist
//
// 版本：v1.0 | 日期：2026-08-04 | 作者：OpenHappy Team
// ===================================================================
//
// 说明：
//   HIPINEXT_HOME = <hipi_next>/code，所有路径均为相对 HIPINEXT_HOME 的相对路径。
//   cxs_prcl_filelist.f 通过 `-f $HIPINEXT_HOME/design/filelist/common.f` 引用本文件。

// --- Common library modules ---
$HIPINEXT_HOME/common/onehot_to_binary.v
$HIPINEXT_HOME/common/apb_reg_sync_if.v
$HIPINEXT_HOME/common/async_fifo.v
$HIPINEXT_HOME/common/sync_cell.v
$HIPINEXT_HOME/common/rstn_sync.v
$HIPINEXT_HOME/common/rstn_sync_neg.v
// $HIPINEXT_HOME/common/sync_fifo_data.v  // 死引用：未被工程 RTL 例化（已备份，v0.2.0 清理）
$HIPINEXT_HOME/common/sync_p2p.v
$HIPINEXT_HOME/common/sync_fifo.v
$HIPINEXT_HOME/common/sync_icg_pos.v
$HIPINEXT_HOME/common/data_pipe.v
$HIPINEXT_HOME/common/glitch_free_mux.v
$HIPINEXT_HOME/common/xl_std_cell/generic_ckand.v
$HIPINEXT_HOME/common/xl_std_cell/generic_ckbuf.v
$HIPINEXT_HOME/common/xl_std_cell/generic_ckinv.v
$HIPINEXT_HOME/common/xl_std_cell/generic_ckmux.v
$HIPINEXT_HOME/common/xl_std_cell/generic_ckor.v
$HIPINEXT_HOME/common/xl_std_cell/generic_databuf.v
$HIPINEXT_HOME/common/xl_std_cell/generic_dff_qn.v
$HIPINEXT_HOME/common/xl_std_cell/generic_dff_q.v
$HIPINEXT_HOME/common/xl_std_cell/generic_icg_neg.v
$HIPINEXT_HOME/common/xl_std_cell/generic_icg_pos.v
$HIPINEXT_HOME/common/xl_std_cell/generic_dffs_qn.v
$HIPINEXT_HOME/common/xl_std_cell/generic_dffs_q.v
$HIPINEXT_HOME/common/xl_std_cell/generic_datamux.v
// $HIPINEXT_HOME/common/ecc8/encoder.v    // 死引用：工程用 link_layer/ecc_generate + ecc_check（已备份）
// $HIPINEXT_HOME/common/ecc8/checker.v
// $HIPINEXT_HOME/common/ecc8/decoder.v

