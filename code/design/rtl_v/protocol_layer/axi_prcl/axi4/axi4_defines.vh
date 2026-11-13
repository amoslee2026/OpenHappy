// ===================================================================
// axi4_defines.vh — AXI4 Asymmetric 协议层编译期配置宏（自包含）
//
// 版本：v1.0 | 日期：2026-08-24 | 作者：OpenHappy Team
// 说明：由 axi_defines.vh 分拆而来（AXI4/AXI5 各自自包含）。
//       本文件 = 共享宏 + AXI4 专属宏；AXI5 专属宏见 axi5_defines.vh。
//
// 用途：由 filelist axi4_asymm_prcl.f 在编译集合最开头直接编译本文件，为全部 axi4_asymm
//       RTL 文件提供配置宏（RTL 文件内无 `include 指令，仿 cxs_prcl 约定）。
//       TX/RX_LANE_K 源宏与 MASTER_DIE/SLAVE_DIE 由 top 层 hipi_top_defines.vh 定义，须先于本文件编译。
//
// 来源：analysis/protocol_layer/AXI/module/parameter.md
// ===================================================================
// --- HIPI AXI4 通道 CN 号（flit 封装时区分各通道）---
`define AXI4_AW_CHANNEL    5'd1
`define AXI4_AR_CHANNEL    5'd2
`define AXI4_WDATA_CHANNEL 5'd3
`define AXI4_RDATA_CHANNEL 5'd4
`define AXI4_B_CHANNEL     5'd5
`define AXI4_CREDIT_CHANNEL 5'd6

// --- HIPI NEXT AXI4 通道 CN 号（flit 封装时区分各通道）---
`define AXI4_AW_CN        5'd17   // AW 通道 CN 号
`define AXI4_AR_CN        5'd18   // AR 通道 CN 号
`define AXI4_W_FULL_CN    5'd19   // W full 通道 CN 号
`define AXI4_W_LITE_CN    5'd21   // W lite 通道 CN 号（对齐 HIPI_AXI4_Next.md，R_full=20/W_lite=21）
`define AXI4_R_FULL_CN    5'd20   // R full 通道 CN 号
`define AXI4_R_LITE_CN    5'd22   // R lite 通道 CN 号
`define AXI4_B_CN         5'd23   // B 通道 CN 号

// --- HIPI国标 ---
`define HIPI_GB

// --- 异步 FIFO 深度 ---
`define PRCL_ASYNC_RXFIFO_DEPTH  32   // p_axis_async_fifo / p_crd_gen 共用
`define PRCL_ASYNC_TXFIFO_DEPTH  8   

// --- 请求超时 outstanding 计数 ---
`define OTS_NUM           16      // p_req_timeout 使用

// --- 数据位宽模式 ---
`ifdef PAIF_DATA_WIDTH_EQ_1024
    `define AXI_DW_MODE           2'd2    // DATA WIDTH MODE: 0x0 256bit / 0x1 512bit / 0x2 1024bit
    `define PAIF_DW_MODE          2'd2
`elsif AXI_DATA_WIDTH_EQ_256
    `define AXI_DW_MODE           2'd0
    `define PAIF_DW_MODE          2'd1
`else 
    `define AXI_DW_MODE           2'd1
    `define PAIF_DW_MODE          2'd1
`endif

`define AXI_DW            (256<<`AXI_DW_MODE)   // 256 or 512 or 1024

`define PAIF_DW           (256<<`PAIF_DW_MODE)   // 512 or 1024

// --- 协议模式 ---
`define AXI_MODE          4'd0    // 0x0 : AXI4 ; 0x1 : AXI5

// --- 拓扑模式（标志宏，配合 `ifdef 使用；MASTER_DIE/SLAVE_DIE 源宏由 top 层 hipi_top_defines.vh 定义）---
`ifdef MASTER_DIE
    `define MST_DIE_MODE          1'b1
`else
    `define MST_DIE_MODE          1'b0
`endif

`ifdef SLAVE_DIE
    `define SLV_DIE_MODE          1'b1
`else
    `define SLV_DIE_MODE          1'b0
`endif

`ifdef MST_TX_LANE_3
    `define MST_TX_LANE_NUM   2'd3
`elsif MST_TX_LANE_2
    `define MST_TX_LANE_NUM   2'd2
`elsif MST_TX_LANE_1
    `define MST_TX_LANE_NUM   2'd1
`else
    `define MST_TX_LANE_NUM   2'd0
`endif

`ifdef MST_RX_LANE_3
    `define MST_RX_LANE_NUM   2'd3
`elsif MST_RX_LANE_2
    `define MST_RX_LANE_NUM   2'd2
`elsif MST_RX_LANE_1
    `define MST_RX_LANE_NUM   2'd1
`else
    `define MST_RX_LANE_NUM   2'd0
`endif

`ifdef SLV_TX_LANE_3
    `define SLV_TX_LANE_NUM   2'd3
`elsif SLV_TX_LANE_2
    `define SLV_TX_LANE_NUM   2'd2
`elsif SLV_TX_LANE_1
    `define SLV_TX_LANE_NUM   2'd1
`else
    `define SLV_TX_LANE_NUM   2'd0
`endif

`ifdef SLV_RX_LANE_3
    `define SLV_RX_LANE_NUM   2'd3
`elsif SLV_RX_LANE_2
    `define SLV_RX_LANE_NUM   2'd2
`elsif SLV_RX_LANE_1
    `define SLV_RX_LANE_NUM   2'd1
`else
    `define SLV_RX_LANE_NUM   2'd0
`endif

// --- 协议层聚合信息字段（供软件/Link 层读取拓扑与位宽配置）---
//   ASYMM bit（bit12）按 `ifdef ASYMM_MODE 决定：定义=非对称(1)，否则对称(0)
`ifdef ASYMM_MODE
`define PROTOCOL_LAYER_INFO    { 2'd0 , `SLV_RX_LANE_NUM , 2'd0 , `SLV_TX_LANE_NUM , 2'd0 , `MST_RX_LANE_NUM , 2'd0 , `MST_TX_LANE_NUM , 1'd0 , `SLV_DIE_MODE , `MST_DIE_MODE , 1'b1 , 2'd0 , `PAIF_DW_MODE , 2'd0 , `AXI_DW_MODE , `AXI_MODE }
`else
`define PROTOCOL_LAYER_INFO    { 10'd0 , 2'd0 , 2'd0 , 2'd0 , 2'd0 , 1'b0 , 1'b0 , 2'd0 , `PAIF_DW_MODE , 2'd0 , `AXI_DW_MODE , `AXI_MODE }
`endif
                                  // bit[3:0]   支持协议：       0x0 AXI4 / 0x1 AXI5
                                  // bit[5:4]   支持协议的数据位宽: 0x0 256bit / 0x1 512bit / 0x2 1024bit
                                  // bit[9:8]   PAIF 位宽:        0x0 256bit / 0x1 512bit / 0x2 1024bit
                                  // bit[12]    0 表示对称架构，1 表示非对称架构
                                  // bit[13]    1 表示 Master Die
                                  // bit[14]    1 表示 Slave Die ，Master Die 与 Slave Die 可以同时使能
                                  // bit[17:16] Master tx lane 的个数，实际个数 = 该字段值 + 1
                                  // bit[21:20] Master rx lane 的个数，实际个数 = 该字段值 + 1
                                  // bit[25:24] Slave tx lane 的个数，实际个数 = 该字段值 + 1
                                  // bit[29:28] Slave rx lane 的个数，实际个数 = 该字段值 + 1
