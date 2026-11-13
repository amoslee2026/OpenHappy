// =============================================================================
// link_layer_defines.vh — 链路层编译期参数宏定义（整合单一入口）
// =============================================================================
// 用途：链路层所有可配置结构参数以 `define 宏形式集中定义，各 RTL 模块在
//       文件头 `include "link_layer_defines.vh" 后直接使用宏（不再在模块内
//       重新 parameter/localparam 声明）。
// 历史：整合前为 link_layer_params.vh（RTL 直接 `include）+ link_layer_defines.vh
//       （LL_ 前缀宏，经 filelist 编译，未被子模块 include）两套并行体系。
//       2026-08-13 OpenHappy Team 全面合并为单一 link_layer_defines.vh，删除
//       link_layer_params.vh / link_layer_undefines.vh；LL_ 体系（LL_PAIF_DATA_WIDTH
//       / LL_TX·RX_LANE*）当前未被 RTL 使用，改以注释保留（§3），不再 `define。
// 修订：2026-08-26 OpenHappy Team 重配置为非对称结构——PAIF 512 bit、
//       LANE_MACRO_MODE=2'b00（每 link 1 lane）、TX_LINK_NUM=4、RX_LINK_NUM=1。
// 修订：2026-09-29 OpenHappy Team spec 侧 params.vh 参数回归本文件（用户纪律：不维护两套参数文件，
//       编译期参数/宏单一事实源 = 本文件；spec 侧只留 link_layer_spec.md §6 参数文档表）：
//       新增数值宏 REPLAY_BUF_DEPTH / CREDIT_FIFO_DEPTH（=16/16，与 presence 档位宏成对同步）
//       与 CPIF_LANE_WIDTH（=1024）；PAIF_INTF_MODE 不重复引入（映射既有 presence 宏
//       PAIF_INTF_512：1'b1 ⇔ 已定义，1'b0 ⇔ 未定义）；link_tx_512_lane1.v 改 include 本文件，
//       其模块内纯行索引 localparam 改名 REPLAY_ROW_IDX_W（避让同名含 flag 位宏 REPLAY_BUF_PTR，
//       值 4 不变）。
// 性质：编译期宏（非运行时寄存器），软件不可写；
//       MODE_REG.LANE_MACRO_MODE / TX_LINK_NUM / RX_LINK_NUM、
//       RETRY_CFG_REG.REPLAY_BUF_DEPTH / CREDIT_FIFO_DEPTH 寄存器字段为
//       RO 状态回读，反映本文件宏值；PAIF_INTF_512 仅编译期 presence 宏（无寄存器回读）；
//       s2p_mode 为运行时寄存器（位于 PHY_MODE），非编译期宏。
// 单一事实源：本文件为链路层编译期参数/宏唯一载体（spec 侧只留参数文档，无参数文件）；
//       任何取值变更须同步更新 link_layer_spec.md §6（参数文档表）与本文件。
// 用法：.v 文件头 `include "link_layer_defines.vh"，随后直接使用 `PAIF_WIDTH
//       `LANE_COUNT `REPLAY_BUF_PTR 等宏。
// =============================================================================

`ifndef LINK_LAYER_DEFINES_VH
`define LINK_LAYER_DEFINES_VH

// -----------------------------------------------------------------------------
// §1 接口位宽与 lane 数量参数
// -----------------------------------------------------------------------------

// PAIF 接口位宽模式（spec §5.3.X 编译期参数）
//   以 presence 宏 `define PAIF_INTF_512（是否定义）作为唯一开关：
//   `define PAIF_INTF_512 → PAIF 512 bit；不定义 → PAIF 1024 bit。
// 说明：单条数据通路 PAIF 接口位宽；1024 bit 模式下 1 拍承载 256 B Flit
//       的前 128 B 或后 128 B，512 bit 模式下需 2 拍拼 128 B。
// 修订（2026-08-11 OpenHappy Team）：PAIF 位宽与 hipi_ctrl_top PAIF_DATA_WIDTH（512）
//       对齐，改为 512 bit 模式（与 top 默认 512 匹配）。
//   `ifdef 无法对表达式求值（本工具链亦不支持 `if 表达式指令），
//   故以"是否 define"形式作为唯一选择源，PAIF_WIDTH 由本宏派生（`else → 1024）。
//`define PAIF_INTF_512

// lane 数量参数（spec §5.3 MODE_REG.LANE_MACRO_MODE，纯 lane 数）
//   2'b00：1 lane
//   2'b01：2 lane（默认）
//   2'b10：4 lane
//   2'b11：reserved（按 1 lane 处理）
// 说明：每个 lane 的 CPIF 接口固定 1024 bit，由运行时寄存器 PHY_MODE.s2p_mode
//       配置为 512 bit（低 512 有效，高 512 always 0）或 1024 bit。
`define LANE_MACRO_MODE   2'b00

// lane 条件声明 presence 宏（由 LANE_MACRO_MODE 派生，供 `ifdef 使用）
//   `ifdef 无法对表达式求值，故以"是否 define"形式单独声明：
//   LANE_MACRO_MODE=2'b00（1 lane）：LINK_LANE1 / LINK_LANE3 均不定义
//   LANE_MACRO_MODE=2'b01（2 lane）：仅定义 LINK_LANE1
//   LANE_MACRO_MODE=2'b10（4 lane）：定义 LINK_LANE1 + LINK_LANE3
// ★ 强制同步：修改 LANE_MACRO_MODE 时必须同步增删下面两个 `define，
//   三者不一致将导致端口声明与实际 lane 数失配。
// 用法：lane0 永远声明；`ifdef LINK_LANE1 声明 lane1；
//       `ifdef LINK_LANE3 声明 lane2 / lane3（蕴含 LINK_LANE1）。
// 修订（2026-08-26 OpenHappy Team）：重配置为非对称结构，每 link 1 lane，
//   故 LANE_MACRO_MODE=2'b00，LINK_LANE1 / LINK_LANE3 均不定义。
//`define LINK_LANE1
//`define LINK_LANE3

// TX / RX 数据通路数量（spec §5.3.X 编译期参数，不对称结构）
//   TX_LINK_NUM = link_layer.v 顶层例化的 link_tx.v 个数
//   RX_LINK_NUM = link_layer.v 顶层例化的 link_rx.v 个数
//   对称结构：TX_LINK_NUM = RX_LINK_NUM = 1（单实例内含 LANE_COUNT 个 lane）
//   不对称结构：TX_LINK_NUM / RX_LINK_NUM 各取 1/2/3/4、相互独立配置
//   注：默认值保持 (1,1) 即对称结构为编译期默认；不对称结构由项目覆盖
// 修订（2026-08-11 OpenHappy Team）：与 hipi_ctrl_top TX_LANE_1/RX_LANE_1（2 link）对齐，
//   改为 TX_LINK_NUM=RX_LINK_NUM=2（解除 TX/RX_LINK1_EXIST）。
// 修订（2026-08-12 OpenHappy Team）：还原默认配置为对称 (1,1)，与 spec 参数副本
//   （TX_LINK_NUM=RX_LINK_NUM=1）一致；link1 默认不例化。
// 修订（2026-08-26 OpenHappy Team）：重配置为非对称结构——TX_LINK_NUM=4（link_tx0~3）、
//   RX_LINK_NUM=1（仅 link_rx0），每 link 1 lane（LANE_MACRO_MODE=2'b00）。
// 修订（2026-10-21 OpenHappy Team）：1T1R 对称基线——TX_LINK_NUM=RX_LINK_NUM=1（对称 (1,1)），
//   仅 link0 例化，每 link 1 lane（LANE_MACRO_MODE=2'b00，LINK_LANE1/3 不定义）；
//   TX_LINK1~3_EXIST / RX_LINK1~3_EXIST 全部注释。与 top 侧
//   AXI_ASYM_MODE 未定义 + PRCL_TX/RX_LANE_* 全关 + PHY_LANE_* 全关 的 1T1R 宏态对齐。
`define TX_LINK_NUM       4'd1
`define RX_LINK_NUM       4'd1

// TX / RX link 条件例化 presence 宏（由 TX_LINK_NUM / RX_LINK_NUM 派生，供 `ifdef 使用）
//   `ifdef 无法对表达式求值，故以"是否 define"形式单独声明：
//   TX_LINK_NUM≥2 → `define TX_LINK1_EXIST；≥3 → TX_LINK2_EXIST；依此类推至 link3
//   RX_LINK_NUM≥2 → `define RX_LINK1_EXIST；≥3 → RX_LINK2_EXIST；依此类推至 link3
// ★ 强制同步：修改 TX_LINK_NUM / RX_LINK_NUM 时必须同步增删下面 6 个 `define，
//   三者不一致将导致端口声明与实际 link 数失配。
// 说明：L2 spec §3.1 中 LINKx_EXIST 在本文件按 TX/RX 拆分为 TX_LINKx_EXIST /
//   RX_LINKx_EXIST（不对称结构 TX 与 RX link 数不同，需独立控制端口与例化）。
//   当前对称结构 (1,1) = 1T1R 基线：TX/RX_LINK1~3_EXIST 均不定义（仅 link0 生效）。
//   注：link 仅支持 0~3（共 4 个），不定义 link4。
//`define TX_LINK1_EXIST
//`define TX_LINK2_EXIST
//`define TX_LINK3_EXIST
//`define RX_LINK1_EXIST
//`define RX_LINK2_EXIST
//`define RX_LINK3_EXIST

// -----------------------------------------------------------------------------
// §1.5 兼容模式形态选择 presence 宏（page形态拆分，镜像 TX）
// -----------------------------------------------------------------------------
//   以"是否 define"的 presence 宏作为兼容形态唯一开关（`ifdef 无法对表达式求值）：
//   `define LINK_RX_COMPAT_MODE → link_rx.v 例化兼容形态 link_rx_compat（形态④），
//   RX 侧不再例化性能形态；不定义 → 按 PAIF_INTF_512 / LINK_LANE1 择一例化性能形态。
//   镜像 TX 侧 LINK_TX_COMPAT_MODE（link_tx.v 用）：
//       LINK_TX_COMPAT_MODE → link_tx.v 例化 link_tx_compat；默认不定义（性能形态激活）。
//   默认均关闭（性能模式激活）。两个宏应同步开/关（RX 兼容须 TX 兼容配合，两侧对齐）。
//   修订（2026-09-07 OpenHappy Team）：新增，配合 link_rx 形态拆分架构（见
//   spec/link_layer/submods/link_rx/link_rx_mas.md §2.1 形态选择矩阵）。
//`define LINK_RX_COMPAT_MODE
//`define LINK_TX_COMPAT_MODE

// -----------------------------------------------------------------------------
// 深度以"是否 define"的 presence 宏表达，`ifdef 选择 SRAM 硬宏规格（replay_buffer.v /
// credit_fifo.v 例化）；数值深度宏（REPLAY_BUF_DEPTH / CREDIT_FIFO_DEPTH，供 $clog2 / 算术）
//   见档位表下方（与 presence 宏成对同步）。
//   取值 16 / 32 / 64 / 128，当前配置 16（2026-09-15 由 32 切 16；16 深 replay
//   buffer = 8 片 32×128 SRAM（8×32×128，每层 1024bit=128B，flit 存 2 层，共
//   32 Kbit），见 replay_buffer.v；禁用 256：8 bit seq 无法提供 clog2(256)+1=9
//   bit 指针宽度）。
//   ★ 强制同步：修改深度时必须只增删下面两组 `define 中的一个/多个并保持一致，
//     任一组中至多一个被 `define（16/32/64/128 择一），否则 ifdef 选择失配。
//`define REPLAY_BUF_DEPTH_64    // replay buffer：选 64 深（SRAM 64x128）
`define REPLAY_BUF_DEPTH_16     // replay buffer：选 16 深（8 片 32×128 SRAM，flit 存 2 层）
//`define REPLAY_BUF_DEPTH_32   // replay buffer：选 32 深（SRAM 32x128）
//`define REPLAY_BUF_DEPTH_128
`define CREDIT_FIFO_DEPTH_16    // credit FIFO：选 16 深（DFF 寄存器阵列）
//`define CREDIT_FIFO_DEPTH_32  // credit FIFO：选 32 深（DFF 寄存器阵列）
//`define CREDIT_FIFO_DEPTH_64  // credit FIFO：选 64 深（DFF 寄存器阵列）
//`define CREDIT_FIFO_DEPTH_128
// 数值宏（2026-09-29 spec 侧 params.vh 回归）：供 $clog2 / 算术使用（如 link_tx_512_lane1.v
// REPLAY_ROW_IDX_W = $clog2(`REPLAY_BUF_DEPTH)）；取值必须与上方 presence 档位宏一致（当前 16/16）。
// ★ 强制同步：切换档位时本两行、上方 presence 宏、§4 REPLAY_BUF_PTR 段三方同步。
`define REPLAY_BUF_DEPTH   16    // replay buffer 深度（数值）
`define CREDIT_FIFO_DEPTH  16    // credit FIFO 深度（数值）

// -----------------------------------------------------------------------------
// §3 事件命令 opcode（spec §5.3.X 编译期参数 EVT_OP_*）
// -----------------------------------------------------------------------------
//   按 GB/T 46280.4-2025 §7.3.2 表 15 边带通路 opcode 定义；
//   链路层与 Logic PHY 共享同一份参数，保证两侧 EventCmd 编码一致。
//   EventCmd 16 bit 格式：{Type, 7'b0, opcode}
//     Type=0：控制本端 PHY（PHY_INIT / RETRAIN）
//     Type=1：发送到对端链路层（LP_ENTER / LP_EXIT / LANE_ON / LANE_OFF）
`define EVT_OP_PHY_INIT   8'h44   // PHY 初始化（启动 RX 初始化），Type=0
`define EVT_OP_RETRAIN    8'h80   // 重训练请求，Type=0
`define EVT_OP_LP_ENTER   8'h84   // 进入 U2 低功耗，Type=1
`define EVT_OP_LP_EXIT    8'h89   // 退出低功耗，Type=1
`define EVT_OP_LANE_ON    8'h40   // Lane 打开（复用 RX ON），Type=1
`define EVT_OP_LANE_OFF   8'h41   // Lane 关闭（复用 RX OFF），Type=1
`define EVT_OP_RX_READY   8'h7F   // RX 初始化完成上报（link_event_process MAS §2.1/§4.3），Type=1

// -----------------------------------------------------------------------------
// §4 派生宏（基于上述宏推导，模块内直接使用，不再 localparam）
// -----------------------------------------------------------------------------

// PAIF 接口位宽（由 PAIF_INTF_512 presence 宏决定）
`define PAIF_INTF_512
`ifdef PAIF_INTF_512
    `define PAIF_WIDTH    512
`else
    `define PAIF_WIDTH    1024
`endif

// lane 数量（由 LANE_MACRO_MODE 派生）
//   00 → 1 lane / 01 → 2 lane / 10 → 4 lane / 11 → 保留（按 1 lane）
`define LANE_COUNT        ((`LANE_MACRO_MODE == 2'b00) ? 1 : \
                            (`LANE_MACRO_MODE == 2'b01) ? 2 : \
                            (`LANE_MACRO_MODE == 2'b10) ? 4 : 1)

// CPIF 接口位宽（per-lane 固定 1024 bit，2026-09-29 spec 侧 params.vh 回归；运行时由
//   PHY_MODE.s2p_mode 选择 512 / 1024 bit 有效宽度：s2p_mode=0 → 低 512 bit 有效、高 512 bit
//   始终 0；s2p_mode=1 → 1024 bit 全有效。本宏仅给出物理位宽 1024，逻辑有效宽度由各模块按
//   s2p_mode 运行时选择）
`define CPIF_LANE_WIDTH   1024

// replay buffer 指针宽度（由 REPLAY_BUF_DEPTH_* presence 宏选择代码段，段内硬编码）
//   含 1 bit flag 位：DEPTH=16 → N=5 bit；DEPTH=32 → N=6 bit；DEPTH=64 → N=7 bit（默认）；
//   DEPTH=128 → N=8 bit
// ★ 强制同步：修改 REPLAY_BUF_DEPTH_* presence 宏时必须同步本段 ifdef/elsif/else，
//   否则指针位宽与实际深度失配。
//   （不走 $clog2 派生，遵循"presence 宏控制代码段 + 段内硬编码"方案）
//   注：link_tx_512_lane1.v 内另有纯行索引 localparam REPLAY_ROW_IDX_W = $clog2(`REPLAY_BUF_DEPTH)
//   （16 深 = 4，不含 flag 位），与本宏语义不同，已改名避让（2026-09-29 参数单一事实源切换）。
`ifdef REPLAY_BUF_DEPTH_128
    `define REPLAY_BUF_PTR     8
`elsif REPLAY_BUF_DEPTH_32
    `define REPLAY_BUF_PTR     6
`elsif REPLAY_BUF_DEPTH_16
    `define REPLAY_BUF_PTR     5
`else
    `define REPLAY_BUF_PTR     7
`endif

// -----------------------------------------------------------------------------
// §5 LL_ 体系（历史保留，当前未被 RTL 使用 — 以注释形式保留，不再 `define）
// -----------------------------------------------------------------------------
// 原 link_layer_defines.vh 由 top 层 hipi_top_defines.vh 的宏约束推导 LL_*（LL_PAIF_DATA_WIDTH
// / LL_CPIF_DATA_WIDTH / LL_TX·RX_LANE1/2/3），经 filelist 编译，未被子模块 include。
// 与 §1/§4 中 PAIF_INTF_MODE→PAIF_WIDTH 等价；整合后如需按 top 宏约束推导，在下面
// `ifdef 打开即可：
//`ifdef PAIF_DATA_WIDTH_EQ_1024
//    `define LL_PAIF_DATA_WIDTH  1024
//`else
//    `define LL_PAIF_DATA_WIDTH  512
//`endif
//`ifdef CPIF_DATA_WIDTH_EQ_1024
//    `define LL_CPIF_DATA_WIDTH  1024
//`else
//    `define LL_CPIF_DATA_WIDTH  512
//`endif
//`ifdef TX_LANE_1
//    `define LL_TX_LANE1
//`endif
//`ifdef RX_LANE_1
//    `define LL_RX_LANE1
//`endif

`endif // LINK_LAYER_DEFINES_VH
