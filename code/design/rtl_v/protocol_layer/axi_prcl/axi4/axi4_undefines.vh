// ===================================================================
// axi4_undefines.vh — AXI4 Asymmetric 协议层编译期配置宏清理
//
// 版本：v1.0 | 日期：2026-08-24 | 作者：OpenHappy Team
//
// 用途：清理 axi4_defines.vh 定义的全部编译期宏（共享宏 + AXI4 专属），避免宏跨编译
//       单元残留。RTL filelist 最末尾直接编译本文件，与最开头 axi4_defines.vh 配对
//       （仿 cxs_prcl_defines.vh / cxs_prcl_undefines.vh 约定）。
//       注意：MST/SLV_TX/RX_LANE_[1..3] 源宏、MASTER_DIE/SLAVE_DIE 与 ASYMM_MODE
//       由顶层 hipi_top_defines.vh 定义（本文件仅 `ifdef 引用、不定义），
//       归 hipi_top_undefines.vh 统一清理，本文件不 `undef。
//
// 来源：analysis/protocol_layer/AXI/module/parameter.md
// ===================================================================

// --- HIPI 国标（2026-09-23 补：axi4_defines.vh 新增 `define HIPI_GB）---
`undef HIPI_GB

// --- HIPI AXI4 通道号（2026-09-23 补：新 protocol_* 模块依赖 AXI4_*_CHANNEL）---
`undef AXI4_AW_CHANNEL
`undef AXI4_AR_CHANNEL
`undef AXI4_WDATA_CHANNEL
`undef AXI4_RDATA_CHANNEL
`undef AXI4_B_CHANNEL
`undef AXI4_CREDIT_CHANNEL

// --- AXI4 通道 CN 号 ---
`undef AXI4_AW_CN
`undef AXI4_AR_CN
`undef AXI4_W_FULL_CN
`undef AXI4_W_LITE_CN
`undef AXI4_R_FULL_CN
`undef AXI4_R_LITE_CN
`undef AXI4_B_CN

// --- 异步 FIFO 深度（2026-09-23 拆分 RX/TX，对齐 *_defines.vh）---
`undef PRCL_ASYNC_RXFIFO_DEPTH
`undef PRCL_ASYNC_TXFIFO_DEPTH

// --- 请求超时 outstanding 计数 ---
`undef OTS_NUM

// --- 数据位宽模式 ---
`undef AXI_DW_MODE
`undef PAIF_DW_MODE
`undef AXI_DW
`undef PAIF_DW

// --- 协议模式 ---
`undef AXI_MODE

// --- 拓扑模式（MST/SLV_DIE_MODE 由本文件派生；MASTER_DIE/SLAVE_DIE 源宏归顶层清理，不 undef）---
`undef MST_DIE_MODE
`undef SLV_DIE_MODE

// --- TX/RX Lane 个数 ---
`undef MST_TX_LANE_NUM
`undef MST_RX_LANE_NUM
`undef SLV_TX_LANE_NUM
`undef SLV_RX_LANE_NUM

// --- 协议层聚合信息 ---
`undef PROTOCOL_LAYER_INFO
