// ===================================================================
// axi4_symm_prcl.f — AXI4 对称模式协议层专属 filelist
//
// 版本：v1.1 | 日期：2026-10-19 | 作者：OpenHappy Team
//
// 说明：
//   HIPINEXT_HOME = <hipi_next>/code，所有路径均为相对 HIPINEXT_HOME 的相对路径。
//   本 filelist 为对称（AXI_SYMM_MODE：AXI_ASYM_MODE 未定义）协议层专属文件段，
//   供 axi4_prcl.f 经 -f 吃入；公共 RTL 与 axi4 宏窗口由 axi4_prcl.f 统一管理。
//
// 修订 v1.1：补齐 GB（HIPI 国标，axi4_defines.vh 里 `define HIPI_GB）双引擎通路
//   所需但 v1.0 遗漏的 6 个模块。axi4_symm_prcl.v 在 `ifdef HIPI_GB 分支例化了
//   u_tx_packet / u_pkgpipe / u_tx_mux / u_rx_demux / u_ch_identify，而这些模块
//   （及其内部 protocol_rx_pkt_proc）原先不在任何 filelist 内 → VCS 报
//   Error-[CFCILFBI] Cannot find cell in liblist。
//   全部 6 个均为 symm/GB 专属（asymm/iodie/cmptdie 不使用），故归本文件。
// ===================================================================

// --- symm 专属：集中式 CSR（由 axi4_symm_prcl 顶层例化）---
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/axi4/p_axi4_symm_csr.v

// --- symm/GB 专属：HIPI 国标双引擎通路（axi4_defines.vh `define HIPI_GB 激活）---
//     TX：protocol_tx_packet（GB 包封装）/ protocol_tx_pkgpipe（打包流水）
//     RX：protocol_rx_ch_identify（通道识别）→ 内部 protocol_rx_pkt_proc（包处理）
//     切换：p_tx_mux2to1 / p_rx_demux1to2（hipi_sel 选路）
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/axi4/protocol_tx_packet.v
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/axi4/protocol_tx_pkgpipe.v
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/axi4/protocol_rx_pkt_proc.v
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/axi4/protocol_rx_ch_identify.v
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/common/p_tx_mux2to1.v
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/common/p_rx_demux1to2.v

// --- symm 专属：TOP（axi4_symm 顶层 wrapper）---
$HIPINEXT_HOME/design/rtl_v/protocol_layer/axi_prcl/axi4/axi4_symm_prcl.v
