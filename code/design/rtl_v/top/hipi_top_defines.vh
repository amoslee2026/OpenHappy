// ===================================================================
// hipi_top_defines.vh — 包 controller 顶层编译期配置宏
//
// 版本：v0.19 | 日期：2026-09 | 作者：OpenHappy Team
// 修订：v0.19 新增 LOOPBACK 跨 die 主带 TX 源选择宏（§4b0，仅 d2d_hipi_top.v
//       LOOPBACK 形态消费）：LB_D0_TX_SEL_LANE{1,2,3} 选 die0 喂 die1 RX 的 TX
//       lane，LB_D1_TX_SEL_LANE{1,2,3} 选 die1 喂 die0 RX 的 TX lane，lane0~lane3
//       四选一；默认全部注释（各选 lane0，tx0→rx0）；定义 >1 时高 index 优先并
//       编译期告警。
// 修订：v0.18 新增 AXI_IO_DIE（IO Die AXI 协议层 iodie_axi4_prcl）编译开关（§2.2）：
//       defined=例化 iodie（同时支持 Master+Slave，lane0 对称骨干+lane1~3 TX），
//       undefined=旧 axi4_asymm_prcl（MASTER_DIE/SLAVE_DIE 决定类型）。
//       AXI_MST/SLV_*_LANE_* 真源宏去 AXI_ 前缀改裸宏 MST/SLV_*_LANE_*（§4，与
//       OpenHappy Team axi4 + iodie 裸宏命名统一）。§4 TX/RX lane 派生按方案1分支管理：
//       AXI_IO_DIE 为顶层互斥开关——iodie 分支派生 SLV_TX_LANE_1/2/3 且不派生
//       MASTER/SLAVE die lane；非 iodie 按 MASTER_DIE/SLAVE_DIE 分别派生。
//       PRCL_RX_LANE_* 注释：iodie 下禁止定义（无 RX 扩展 lane）。
// 修订：v0.17 逻辑物理层 baseline 改非对称 4 T + 1 R（OpenHappy Team §4c）：PHY_LANE_1+2+3 使能
//       （4 物理 lane），各 lane 方向使能由 logic_phy_defines.vh 离散宏控制（lane0 双向、
//       lane1~3 仅 TX）；与 logic_phy_spec v4.16/4.17 对齐。
// 修订：v0.16 新增 AXI 数据位宽宏 AXI_DATA_WIDTH（摆脱 PAIF_DATA_WIDTH 捆绑）：§3.2 改单块
//       `ifdef PRCL_AXI/else 结构（AXI 侧 + CXS 侧）；规则：PAIF 1024→AXI 1024，否则
//       AXI_DATA_WIDTH_EQ_256（默认不使能）→256，否则默认 512。AXI 用户侧 data/wstrb 位宽
//       改用 AXI_DATA_WIDTH。
// 修订：v0.15 die 开关去 AXI_ 前缀（AXI_MASTER_DIE→MASTER_DIE、AXI_SLAVE_DIE→SLAVE_DIE）：
//       hipi_ctrl_top 端口门控与 OpenHappy Team axi4 协议层统一使用无前缀裸宏（OpenHappy Team axi4_asymm_prcl
//       本就以 MASTER_DIE/SLAVE_DIE 作 PORT/REQ_RESP 门控），top 直接定义裸宏覆盖 OpenHappy Team
//       子 filelist，解决实例端口不匹配；AXI_MST/SLV_*_LANE_* 保留 AXI_ 前缀（OpenHappy Team 侧
//       MST_*/SLV_* 裸 lane 宏由 §4 派生，另行对齐）。
// 修订：v0.14 移除无前缀 lane 裸宏派生（MST/SLV_*_LANE_*）：top 保留 AXI_MST/SLV_*_LANE_*
//       真源宏（hipi_ctrl_top 端口声明唯一引用）；裸 lane 宏改由 OpenHappy Team 对齐。
// 修订：v0.13 AXI dual-mode 宏重构：die 模式改为显式开关 AXI_MASTER_DIE/AXI_SLAVE_DIE
//       （§2，独立可并存，镜像 OpenHappy Team MASTER_DIE/SLAVE_DIE）；AXI_MST/SLV_TX/RX_LANE_*
//       真源宏移入 §4 PRCL_TX/RX_LANE 分支内派生（经 PRCL_AXI + die 门控），不再于
//       §2 顶层声明；单 lane（无 PRCL 声明）归属由 die 开关决定。双向并存制取代原
//       AXI_MASTER_DIE_MODE 单向二选一。
// 修订：v0.12 AXI 协议层改 dual-mode（master/slave 双方向并存）：新增 AXI_MST/SLV_TX/RX_LANE_*
//       真源宏（AXI_ 前缀，与 OpenHappy Team AXI 私有宏 MST_*/SLV_* 命名隔离）；AXI_MASTER_DIE_MODE
//       单向二选一制废弃。cxs/link 仍由 PRCL_TX/RX_LANE_* 管 PAIF 组数（§4 不变）。
// 修订：v0.11 修正 §4b/§4c 顺序 bug：原 D2D_LANE{i}_EN 派生（§4b）位于 PHY_LANE_* 定义
//       （§4c）之前，Verilog 预处理器顺序求值致 `ifdef PHY_LANE_1` 恒假、D2D_LANE1_EN
//       永不激活，lane1 端口静默丢失。交换两段使 PHY_LANE_* 先定义再派生 D2D。
// 修订：v0.10 物理层 lane 与 ctrl link 宏解耦：新增 §4c PHY_LANE_*（逻辑层物理 lane
//       数），§4b D2D_LANE{i}_EN 改由 PHY_LANE_* 派生；TX_LANE_*/RX_LANE_* 保留仅作
//       ctrl link/PAIF 组数（hippi_ctrl_top 展开 link0~3）。LOGIC 层 lane1 使能改用
//       PHY_LANE_1，不再与 ctrl link1 绑定。
// 修订：v0.9 §4 默认配置注释修正：实际默认 TX_LANE_1/RX_LANE_1 已使能（2 tx / 2 rx
//       lane），原"仅 lane0"叙述为早期版本残留。
// 修订：v0.8 同步 axi4_asymm_defines.vh v1.2——AXI 协议层 lane 条件宏改名
//       AXI_TX_LANE_*/AXI_RX_LANE_*（LANE_NUM 值宏保持原名），§4 "与 axi4_asymm_defines.vh
//       保持一致"的说明同步更新。
// 修订：v0.2 说明注释更新——协议层/链路层内部 lane 宏改由本文件 TX_LANE_*/RX_LANE_* 直接映射推导
// 修订：v0.3 AXI_REQ_MODE/AXI_RESP_MODE 合并为 AXI_MASTER_DIE_MODE（defined=REQ/Master Die，
//       undefined=RESP/Slave Die）；AXI_ASYM_MODE / AXI_MASTER_DIE_MODE 移入
//       `ifdef PRCL_AXI4 / PRCL_AXI5 分支下定义（CXS 协议层不定义）
// 修订：v0.4 CXS / LL 数据位宽与特性宏收归本文件——LL_PAIF/CPIF_DATA_WIDTH（CXS/AXI 共用）；
//       协议层 FLIT/CNTL/SRCID/TGTID 位宽 + LAST + CHECKTYPE_ODD_BYTE_PARITY 特性开关
//       在非 PRCL_AXI 分支下定义，供 hipi_ctrl_top 端口展开，不再依赖 cxs_prcl.f / link_layer.f
//       末尾 undefines 的状态
// 修订：v0.5 AXI4/AXI5 协议层宏合并为单一 PRCL_AXI（defined=AXI 协议层，undefined=AXI 侧不例化、
//       CXS 协议层走 hipi_ctrl_top `else 分支）；删除 PRCL_CXS 宏（CXS 例化不再由独立宏管理）；
//       AXI_ASYM_MODE / AXI_MASTER_DIE_MODE 移入 `ifdef PRCL_AXI 分支（等效 axi4_asymm_defines.vh
//       的 MASTER_DIE：AXI_MASTER_DIE_MODE 定义=REQ=MASTER_DIE 定义，未定义=RESP）；
//       CXS 位宽/特性宏改由 `ifndef PRCL_AXI 分支定义（供 hipi_ctrl_top CXS 端口展开）。
// 修订：v0.6 TX/RX lane 宏恢复为手动开关格式（默认 2+2，与 axi4_asymm_defines.vh 2+2 对齐）；
//       不再按协议层分支强制 lane 数。
// 修订：v0.7 协议层位宽/特性宏加 CXSPRCL_ 前缀，与 cxs_prcl_defines.vh 的宏（CXSDATAFLITWIDTH 等）
//       区分：CXSDATAFLITWIDTH → CXSPRCL_DATAFLITWIDTH、CXS_CNTL_WIDTH → CXSPRCL_CNTL_WIDTH、
//       CXS_SRCID_WIDTH → CXSPRCL_SRCID_WIDTH、CXS_TGTID_WIDTH → CXSPRCL_TGTID_WIDTH、
//       CXSCHECKTYPE_ODD_BYTE_PARITY → CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY、CXS_LAST → CXSPRCL_LAST
//       （与 hipi_ctrl_top.v / hipi_top_undefines.vh 同步修改）
//
// 用途：控制器顶层统一 `include 本文件，选择例化的协议层类型（AXI/CXS）、AXI 非对称方向
//       （REQ/RESP，由 AXI_MASTER_DIE_MODE 决定）与 lane 数。
//       RTL filelist 最开头 `include 本文件，最末尾 `include hipi_top_undefines.vh。
//
// 默认配置：PRCL_AXI（AXI 协议层）+ 非对称 + REQ + 2 tx lane + 2 rx lane。
// 说明：PRCL TX/RX lane 宏不带 AXI 前缀；高 index 蕴含低 index，由用户手动开关选择
//       （默认 PRCL_TX_LANE_1/PRCL_RX_LANE_1 = 2+2，须与 axi4_asymm_defines.vh 的
//       AXI_TX_LANE_* / AXI_RX_LANE_* 保持一致）。
//       link_layer_defines.vh / cxs_prcl_defines.vh 以 `ifdef PRCL_TX_LANE_1/2/3 直接映射
//       推导内部 lane 宏（LL_TX_LANE* / CXS_TX_LANE*），须在本文件后编译。
// ===================================================================

// ================================================================
// 0. 编译形态与 MCU 子系统开关
//    ANA / LOOPBACK：由 filelist +define+ 提供（二选一，d2d_hipi_top_form_check 门控）
//    MCU_SUBSYS（默认关闭，需要时取消下行注释）：
//      on  = logic_phy/dig_top/top 含 AHB slave 口组（ahbs_*）→ mcu_subsys（blackbox，
//            AHB Matrix + AHB2APB 桥）→ 两路 APB master 分送 phy CSR / ctrl CSR（现状路径）
//      off = 无 AHB 口、无 mcu_subsys 代码；改为 APB slave 口组（hipi_apb_*，paddr[19:0]），
//            logic_phy 内 MUX 按 paddr[16] 分域：0 = phy CSR（apb_csr_top，0x000~0xFFF），
//            1 = 协议层+链路层 CSR（hipi_ctrl_top，paddr[15:0] 直通）
// ================================================================
// `define MCU_SUBSYS

// ================================================================
// 1. 协议层选择（二选一）
//    PRCL_AXI : defined=例化 AXI 协议层（axi4_asymm_prcl）
//               undefined=例化 CXS 协议层（cxs_prcl，hipi_ctrl_top `else 分支）
// ================================================================
`define PRCL_AXI              // 例化 AXI 协议层（不定义则协议层为 CXS）

// ================================================================
// 2. AXI 拓扑 / 方向模式（仅 PRCL_AXI 分支下定义，CXS 不定义）
//    合并 die + iodie 于同一 PRCL_AXI 分支统一管理（方案1，互斥）：
//    - AXI_IO_DIE（iodie）为顶层互斥开关：定义=例化 iodie_axi4_prcl（同时支持 Master+
//      Slave，lane0 对称骨干 + lane1~3 仅 TX，不使用 MASTER_DIE/SLAVE_DIE）；
//      未定义=例化旧 axi4_asymm_prcl（由 MASTER_DIE/SLAVE_DIE 决定类型）。
//    - MASTER_DIE / SLAVE_DIE（无前缀裸宏，hipi_ctrl_top 与 OpenHappy Team axi4 协议层共用）：
//      仅 `ifndef AXI_IO_DIE 下定义（与 iodie 互斥）——定义 AXI_IO_DIE 时不定义 die 宏。
//      两开关独立，可同时置位（dual-mode 双方向并存）、可仅置位其一、也可全不置位。
//    ★ 单 lane 归属：若 §4 PRCL_TX/RX_LANE_* 均未声明（仅 lane0 恒在，无分支管理），
//      则该唯一 tx/rx lane 归属由 MASTER_DIE / SLAVE_DIE 决定（master 优先，
//      否则 slave）；§4 有 PRCL lane 声明时，mst/slv lane 在 PRCL 分支内经 die 门控派生。
//    ★ Mst/slv lane 裸宏（MST/SLV_TX_LANE_* 等）由 §4 直接派生（裸宏无 AXI_ 前缀，与 OpenHappy Team
//      axi4 协议层 MST/SLV_*_LANE_*、iodie 协议层 SLV_TX_LANE_* 命名统一，无需再对齐）。
//    ★ MST/SLV_*_LANE_* 真源宏统一在 §4 PRCL_TX/RX_LANE 分支内派生（经 PRCL_AXI +
//      die 门控），保证 mst/slv lane 数与 PRCL lane 数一致。
// 注：AXI_MASTER_DIE_MODE 单向二选一制已废弃（v0.12），改为 master/slave 双方向并存。
// ================================================================
`ifdef PRCL_AXI
    // `define AXI_ASYM_MODE      // [1T1R 对称] 注释掉 AXI_ASYM_MODE → 走 axi4_symm_prcl（对称协议层）

    // ---- 2.2 iodie 顶层互斥开关（定义=iodie 协议层，不定义=旧 axi4_asymm_prcl）----
    // `define AXI_IO_DIE          // [1T1R 对称] 不使用 iodie（4T1R 形态）

    `ifndef AXI_IO_DIE
        // ---- 2.1 die 模式开关（无前缀裸宏，hipi_ctrl_top + OpenHappy Team axi4 协议层共用；
        //      仅非 iodie 下定义，与 AXI_IO_DIE 互斥）----
        // ★ 2026-10-27 OpenHappy Team：对称协议层（axi4_symm_prcl）下这两个宏不再需要——
        //   全工程 MASTER_DIE/SLAVE_DIE 的代码级引用点全部位于 `ifdef AXI_ASYM_MODE 分支内
        //   （hipi_ctrl_top.v:539/2271、d2d_hipi_dig_top.v:2703），对称分支零引用；
        //   且 axi4_defines.vh 的 PROTOCOL_LAYER_INFO 对称分支 bit[13]/[14] 硬编码 1'b0。
        //   注释保留原值，便于切回非对称形态时恢复。
        // `define MASTER_DIE          // Master Die 使能（用户侧 AXI4 Slave → s*_axi）— 仅 asymm 需要
        // `define SLAVE_DIE           // Slave Die 使能（用户侧 AXI4 Master → m*_axi）— 仅 asymm 需要
    `endif
`endif

// ================================================================
// 3. CXS / LL 数据位宽与特性宏
// ================================================================
// 3.1 链路层数据位宽（CXS / AXI 协议层共用）——PAIF / CPIF 独立宽度开关管理
//     默认（两开关均注释）= 512；PAIF_DATA_WIDTH_EQ_1024 置位 → PAIF 1024；
//     CPIF_DATA_WIDTH_EQ_1024 置位 → CPIF 1024。开关在本文件声明，供 hipi_ctrl_top /
//     link_layer_defines.vh / cxs_prcl_defines.vh 统一引用（编译顺序：本文件最先）。
//`define PAIF_DATA_WIDTH_EQ_1024     // 置位则 PAIF 数据位宽 = 1024（默认 512）
`define CPIF_DATA_WIDTH_EQ_1024     // 置位则 CPIF 数据位宽 = 1024（对齐 link_layer per-lane 1024，2026-08-11）

`ifdef PAIF_DATA_WIDTH_EQ_1024
    `define PAIF_DATA_WIDTH   1024    // 每组 PAIF 接口数据位宽
`else
    `define PAIF_DATA_WIDTH   512
`endif
`ifdef CPIF_DATA_WIDTH_EQ_1024
    `define CPIF_DATA_WIDTH   1024    // 每组 CPIF 接口数据位宽
`else
    `define CPIF_DATA_WIDTH   512
`endif

// 3.2 协议层数据位宽与特性开关（单块 `ifdef PRCL_AXI/else 结构：PRCL_AXI 分支→AXI 数据位宽；
//     else 分支→CXS 协议层位宽与特性开关，原 ifndef PRCL_AXI 内容）。
//    AXI：用户侧 AXI 数据位宽 AXI_DATA_WIDTH 独立于 PAIF_DATA_WIDTH 管理——PAIF 1024 → AXI 1024；
//         否则 AXI_DATA_WIDTH_EQ_256（默认不使能）→ 256；否则默认 512。供 hipi_ctrl_top AXI
//         data/wstrb 端口位宽声明（不再捆绑 PAIF_DATA_WIDTH，与 AXI 协议层 AXI_DW 对齐待 OpenHappy Team 同步）。
//    CXS：协议层 flit 侧位宽（FLIT/CNTL/SRCID/TGTID）由 PAIF_DATA_WIDTH_EQ_1024 开关管理，须与
//         cxs_prcl_defines.vh 当前激活的 CXSDATAFLITWIDTH_EQ_* 配置一致：PAIF 1024 → FLIT 1024；
//         PAIF 512 → 默认 FLIT 512（也可 256，值见 else 分支注释）；PAIF 接口位宽由统一的
//         PAIF_DATA_WIDTH 提供（见 3.1），CXS_PAIF_DATA_WIDTH 由 cxs_prcl_defines.vh 派生；
//         CXSPRCL_LAST / CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY 为特性开关，供 hipi_ctrl_top 端口按需展开。
`ifdef PRCL_AXI
    // AXI 数据位宽：PAIF 1024 → 1024；否则 AXI_DATA_WIDTH_EQ_256 → 256；否则默认 512
    // ⚠ 独立求值防误判：AXI_DATA_WIDTH 仅由本链决定（上游开关 PAIF_DATA_WIDTH_EQ_1024 /
    //   AXI_DATA_WIDTH_EQ_256，默认均未定义 → 512），与 CPIF_DATA_WIDTH_EQ_1024（CPIF 宽度，
    //   另一支链）无任何绑定。勿凭 CPIF/PAIF/其他宏的置位态推断 AXI_DATA_WIDTH（实测误判 1024）。
    //`define AXI_DATA_WIDTH_EQ_256   // 置位则 AXI 数据位宽 = 256（默认 512；PAIF 1024 优先）
    `ifdef PAIF_DATA_WIDTH_EQ_1024
        `define AXI_DATA_WIDTH   1024
    `elsif AXI_DATA_WIDTH_EQ_256
        `define AXI_DATA_WIDTH   256
    `else
        `define AXI_DATA_WIDTH   512
    `endif
`else
    `ifdef PAIF_DATA_WIDTH_EQ_1024
        `define CXSPRCL_DATAFLITWIDTH               1024
        `define CXSPRCL_CNTL_WIDTH                  52
        `define CXSPRCL_CNTL_CHK_BYTE               7     // cntl chk 字节数 = ceil(52/8)
        `define CXSPRCL_SRCID_WIDTH                 5
        `define CXSPRCL_TGTID_WIDTH                 5
    `elsif CXSDATAFLITWIDTH_EQ_256
        // 256 配置（PAIF 512 通道内 CXS flit 256）：需在 cxs_prcl_defines.vh 编译前定义
        // `CXSDATAFLITWIDTH_EQ_256（与 cxs_prcl_defines.vh L29 EQ_256 分支配套）
        `define CXSPRCL_DATAFLITWIDTH               256
        `define CXSPRCL_CNTL_WIDTH                  18
        `define CXSPRCL_CNTL_CHK_BYTE               3     // cntl chk 字节数 = ceil(18/8)
        `define CXSPRCL_SRCID_WIDTH                 6
        `define CXSPRCL_TGTID_WIDTH                 6
    `else
        // PAIF 512：CXS 为 512，默认 512
        `define CXSPRCL_DATAFLITWIDTH               512
        `define CXSPRCL_CNTL_WIDTH                  44
        `define CXSPRCL_CNTL_CHK_BYTE               6     // cntl chk 字节数 = ceil(44/8)
        `define CXSPRCL_SRCID_WIDTH                 8
        `define CXSPRCL_TGTID_WIDTH                 8
    `endif
    `define CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
    `define CXSPRCL_LAST
`endif

// ================================================================
// 4. PRCL TX / RX lane 数目（协议层 AXI/PAIF 接口通道组，协议层 + 链路层共用）
//    语义 = 协议层↔链路层之间的 AXI/PAIF 接口通道组开关（非物理层 per-lane TX/RX，
//    物理层 lane 数见 §4c PHY_LANE_*）。
//    高 index 蕴含低 index（`ifdef 链）；协议层/链路层经 `ifdef PRCL_TX_LANE_1/2/3
//    直接映射推导内部 lane 宏（AXI_TX_LANE* / CXS_TX_LANE* / LL_TX_LANE*）。
//    默认 = PRCL_TX_LANE_1 / PRCL_RX_LANE_1 使能，2 tx / 2 rx lane。
//    **PRCL_TX_LANE_1/2/3 最多只能使能 1 个；PRCL_RX_LANE_1/2/3 最多只能使能 1 个。**
//    每个 tx lane 对应 1 个 PAIF TX + 1 个 AXI 接口通道子集；
//    每个 rx lane 对应 1 个 PAIF RX + 1 个 AXI 接口通道子集。
//    ★ mst/slv lane 真源宏为裸宏（MST/SLV_TX/RX_LANE_*，无 AXI_ 前缀，§4 直接派生，
//      与 OpenHappy Team axi4 + iodie 裸宏命名统一）。分支管理（方案1）：AXI_IO_DIE（iodie）为顶层
//      互斥开关——定义时走 iodie 分支（仅派生 SLV_TX_LANE_n，不派 RX lane）；未定义时按
//      MASTER_DIE / SLAVE_DIE（§2.1）门控，保证 mst lane 数 ≤ PRCL lane 数。
//      仅当 PRCL lane 宏声明时才有分支管理；单 lane（无 PRCL 声明）归属由 §2 die 开关决定。
//      与 axi4_asymm 侧 MST/SLV_*_LANE_* 裸宏保持一致（当前 2+2）。
//    与 axi4_asymm_defines.vh 的 AXI_TX_LANE_* / AXI_RX_LANE_* 保持一致（当前 2+2）。
//    §4b 由此派生 lane 存在合宏 D2D_LANE{i}_EN（顶层守卫），见下。
//    历史：原名 TX_LANE_*/RX_LANE_*（2026-08-14 OpenHappy Team 改为 PRCL_ 前缀，统一到
//    协议层接口语义；下游 AXI/CXS/link 派生链已同步）。
// ================================================================
// ★【对称 axi4_symm_prcl 限制】选择 AXI_ASYM_MODE 未定义（→ axi4_symm_prcl，1T1R 对称）时，
//    仅支持单 lane（1 组 paif0 + 1 组 s0/m0 AXI 用户口）。因为 axi4_symm_prcl 是单链路扁平桥，
//    CSR（p_axi4_symm_csr）为集中式单实例，bond/fifo/pack/unpack 为单组枚举，未支持多组
//    （多 paif / 多 AXI）。故对称模式下请勿定义 PRCL_TX_LANE_1/2/3 —— 即便定义也不生效
//    （hipi_ctrl_top 对称分支只例化 1 个 axi4_symm_prcl）。如需多 lane AXI 接口，请改用
//    AXI_ASYM_MODE 非对称（axi4_asymm_prcl，lane1~3 随 MST/SLV_*_LANE 展开）或 iodie（AXI_IO_DIE）。
// ================================================================
//`define PRCL_TX_LANE_1     // [1T1R 对称] 不定义 → 仅 1 TX lane0；对称 axi4_symm_prcl 下仅支持单 lane
//`define PRCL_TX_LANE_2     // 定义则 AXI TX LANE 有 3 条（蕴含 PRCL_TX_LANE_1）— 对称 axi4_symm_prcl 下不生效
//`define PRCL_TX_LANE_3     // 定义则 AXI TX LANE 有 4 条（蕴含 PRCL_TX_LANE_1/2）— 对称 axi4_symm_prcl 下不生效

// --- AXI master/slave TX lane 真源宏（裸宏无 AXI_ 前缀，嵌套于 PRCL_TX_LANE 分支，经 PRCL_AXI 门控）---
//   裸宏命令与 OpenHappy Team axi4 协议层（MST/SLV_*_LANE_*）、iodie 协议层（SLV_TX_LANE_*）
//   保持一致，由 hipi side 一份派生直接供给，无需再经 OpenHappy Team filelist 对齐转换。
//   ★ 分支管理（方案1，与 MASTER/SLAVE die 同级、互斥）：AXI_IO_DIE（iodie）作为顶层开关——
//     定义 AXI_IO_DIE 时走 iodie 分支（派生 SLV_TX_LANE_n，lane1~3 为 iodie 的 TX 扩展 lane），
//     且不派生 MASTER_DIE/SLAVE_DIE 的 mst/slv lane；未定义 AXI_IO_DIE 时按 MASTER_DIE/SLAVE_DIE
//     分别派生。
`ifdef PRCL_AXI
    `ifndef AXI_IO_DIE
        `ifdef MASTER_DIE
            `ifdef PRCL_TX_LANE_3
                `define MST_TX_LANE_3
            `endif
            `ifdef PRCL_TX_LANE_2
                `define MST_TX_LANE_2
            `endif
            `ifdef PRCL_TX_LANE_1
                //`define MST_TX_LANE_1
            `endif
        `endif
        `ifdef SLAVE_DIE
            `ifdef PRCL_TX_LANE_3
                `define SLV_TX_LANE_3
            `endif
            `ifdef PRCL_TX_LANE_2
                `define SLV_TX_LANE_2
            `endif
            `ifdef PRCL_TX_LANE_1
                //`define SLV_TX_LANE_1
            `endif
        `endif
    `else
        // iodie：lane1~3 为 TX 扩展 lane（SLV_TX_LANE_n），lane0 恒在对称骨干；
        //       不与 MASTER_DIE/SLAVE_DIE 混用，也不派生 RX lane（见下 PRCL_RX_LANE 注释）
        `ifdef PRCL_TX_LANE_3
            `define SLV_TX_LANE_3
        `endif
        `ifdef PRCL_TX_LANE_2
            `define SLV_TX_LANE_2
        `endif
        `ifdef PRCL_TX_LANE_1
            `define SLV_TX_LANE_1
        `endif
    `endif
`endif

// ★ iodie 模式（`define AXI_IO_DIE）下禁止定义任何 PRCL_RX_LANE_*：iodie 协议层无 RX
//   扩展 lane（lane1~3 仅 TX R 通道，lane0 对称骨憨才含 RX），故 RX lane 宏一律不展开。
//`define PRCL_RX_LANE_1     // 定义则 AXI RX LANE 有 2 条（当前 2 rx：1 mst + 1 slv，双 link/PAIF）
//`define PRCL_RX_LANE_2     // 定义则 AXI RX LANE 有 3 条（蕴含 PRCL_RX_LANE_1）
//`define PRCL_RX_LANE_3     // 定义则 AXI RX LANE 有 4 条（蕴含 PRCL_RX_LANE_1/2）

// --- AXI master/slave RX lane 真源宏（裸宏无 AXI_ 前缀，嵌套于 PRCL_RX_LANE 分支，经 PRCL_AXI 门控）---
//   ★ 与 TX lane 同分支管理（方案1）：AXI_IO_DIE（iodie）作为顶层开关，不派生任何 RX lane
//     （iodie 无 RX 扩展 lane），故 RX 派生仅非 iodie 模式生效。
`ifdef PRCL_AXI
    `ifndef AXI_IO_DIE
        `ifdef MASTER_DIE
            `ifdef PRCL_RX_LANE_3
                `define MST_RX_LANE_3
            `endif
            `ifdef PRCL_RX_LANE_2
                `define MST_RX_LANE_2
            `endif
            `ifdef PRCL_RX_LANE_1
                //`define MST_RX_LANE_1
            `endif
        `endif
        `ifdef SLAVE_DIE
            `ifdef PRCL_RX_LANE_3
                `define SLV_RX_LANE_3
            `endif
            `ifdef PRCL_RX_LANE_2
                `define SLV_RX_LANE_2
            `endif
            `ifdef PRCL_RX_LANE_1
                //`define SLV_RX_LANE_1
            `endif
        `endif
    `else
        // iodie：无 RX 扩展 lane，不派生任何 RX lane 宏
    `endif
`endif

// ================================================================
// 4c. 物理层 lane 数（逻辑物理层逻辑 phy 例化的物理 lane 数，OpenHappy Team）
//     高 index 蕴含低 index；lane0 恒在。
//     PHY_LANE_1 = 逻辑层物理 lane 2 条（lane0+lane1，各 TX+RX）；
//     PHY_LANE_2 = 3 条；PHY_LANE_3 = 4 条。
//     ★ 与 PRCL_TX_LANE_*/PRCL_RX_LANE_*（协议层 AXI/PAIF 接口通道组）解耦：逻辑层物理 lane 数可
//       独立于 ctrl 的多 link 展开配置。
//     当前默认：PHY_LANE_1 + PHY_LANE_2 + PHY_LANE_3（逻辑层 4 物理 lane，非对称 4T1R：
//       lane0 TX+RX 双向，lane1~3 仅 TX；每 lane 方向使能由 logic_phy_defines.vh 的
//       LP_TX_LANE{i}_EN / LP_RX_LANE{i}_EN 离散宏控制），PRCL_TX_LANE_*/PRCL_RX_LANE_* 全关（ctrl 单 link0）。
//     ★ 须先于 §4b 定义（§4b 由 PHY_LANE_* 派生 D2D_LANE{i}_EN，Verilog 预处理器顺序求值）。
// ================================================================
//`define PHY_LANE_1                 // [1T1R 对称] 不定义 → 仅物理 lane0（物理 1 lane，TX+RX 双向）
//`define PHY_LANE_2                 // 使能：lane2 槽位（如需多物理 lane 定义）
//`define PHY_LANE_3                 // 使能：lane3 槽位（如需多物理 lane 定义）
//
// ================================================================
// §4b0. LOOPBACK 跨 die 主带 TX 源选择（仅 d2d_hipi_top.v LOOPBACK 形态使用；
//       两 die 各选一 TX lane 交叉喂给对端 rx_lane0，lane0~lane3 四选一；
//       均不定义 → 默认 lane0（tx0→rx0）；定义 >1 → 高 index 优先，编译期告警）
//       当前默认：全部注释 → 两 die 各选 lane0。
// ================================================================
//`define LB_D0_TX_SEL_LANE1      // die0 选 TX lane1 喂 die1 RX（不定义 → 默认 lane0）
//`define LB_D0_TX_SEL_LANE2      // die0 选 TX lane2 喂 die1 RX（不定义 → 默认 lane0）
//`define LB_D0_TX_SEL_LANE3      // die0 选 TX lane3 喂 die1 RX（不定义 → 默认 lane0）
//`define LB_D1_TX_SEL_LANE1      // die1 选 TX lane1 喂 die0 RX（不定义 → 默认 lane0）
//`define LB_D1_TX_SEL_LANE2      // die1 选 TX lane2 喂 die0 RX（不定义 → 默认 lane0）
//`define LB_D1_TX_SEL_LANE3      // die1 选 TX lane3 喂 die0 RX（不定义 → 默认 lane0）
// 注：4T1R 各 lane 方向使能（LP_RX_LANE{i}_EN）在 logic_phy_defines.vh 按需关 RX
//     实现非对称（lane0 双向、lane1~3 仅 TX）。

`ifdef PHY_LANE_3
    `define PHY_LANE_2
    `define PHY_LANE_1
`elsif PHY_LANE_2
    `define PHY_LANE_1
`endif

// ================================================================
// 4b. lane 存在合宏（= PHY_LANE_i，逻辑层物理 lane 存在）——顶层守卫单源派生
//     供 d2d_hipi_dig_top.v / d2d_hipi_top.v 顶层守卫 lane 端口/信号，以及
//     logic_phy_defines.vh 的 LP_LANE{i}_EN。
//     顶层文件不再内联自派生（原内联段移除）。清理见 hipi_top_undefines.vh。
//     ★ v0.10 起由 PHY_LANE_* 派生，不再由 PRCL_TX_LANE_i | PRCL_RX_LANE_i 派生（物理层
//       lane 与 ctrl link 组数解耦）。
// ================================================================
`ifdef PHY_LANE_1
`define D2D_LANE1_EN
`endif
`ifdef PHY_LANE_2
`define D2D_LANE2_EN
`endif
`ifdef PHY_LANE_3
`define D2D_LANE3_EN
`endif

// ================================================================
// 4c. 每 lane 每方向使能（逻辑物理层 TX/RX 独立，hi pi 顶层单源派生）
//     ——供 logic_phy.v（LP_ 别名）/ d2d_hipi_dig_top.v / d2d_hipi_top.v 共用方向守卫。
//     lane0 恒在且默认双方向（TX+RX），无需方向宏（同 LP_LANE0 处理）；
//     lane{i}(i=1..3) 方向使能由离散宏控制，可与槽位（PHY_LANE_*）解耦：通常只关 RX
//     成 TX-only（非对称 4T1R baseline）。4T1R：lane1~3 仅 TX（`define D2D_PHY_RX_LANE{1,2,3}_EN 注释）。
//     ★ 本宏源定义在 hipi_top_defines.vh（贯穿 controller/common/logic_phy/dig/top 全程），
//       与 logic_phy_defines.vh 的 LP_TX/RX_LANE{i}_EN 别名对应（后者在 logic_phy.f 尾部清理）。
//     ★ 方向宏语义：定义 = 该 lane 该方向数据通路被使能（使用与否决定 lane_top TX_EN/RX_EN）。
// ================================================================
`ifdef PHY_LANE_1
    `define D2D_PHY_TX_LANE1_EN      // lane1 TX 使能
    // `define D2D_PHY_RX_LANE1_EN   // 4T1R：lane1 无 RX（如需 lane1 RX，解开注释）
`endif
`ifdef PHY_LANE_2
    `define D2D_PHY_TX_LANE2_EN      // lane2 TX 使能
    // `define D2D_PHY_RX_LANE2_EN   // 4T1R：lane2 无 RX
`endif
`ifdef PHY_LANE_3
    `define D2D_PHY_TX_LANE3_EN      // lane3 TX 使能
    // `define D2D_PHY_RX_LANE3_EN   // 4T1R：lane3 无 RX
`endif


// ================================================================
// 5. LL_LINK_LANE 数（每个 link 下 CPIF/UDI lane 数量，供 hipi_ctrl_top 端口展开）
//    参考 link_layer_params.vh 的 LINK_LANE1/LINK_LANE3 手动开关风格：
//    LL_LINK_LANE0 恒在（lane0）；LL_LINK_LANE1 定义 → 展开 lane1（2 lane）；
//    LL_LINK_LANE3 定义 → 展开 lane2/3（4 lane，蕴含 lane1）。
//    与 top 的 PRCL_TX_LANE_*/PRCL_RX_LANE_*（控制 link/PAIF 组数）正交：每个已使能 link 下
//    均按 LL_LINK_LANE* 展开 CPIF/UDI lane 端口。
//    ★ 须与 link_layer_params.vh 的 LANE_MACRO_MODE 对齐：LANE_MACRO_MODE=2'b00（1 lane）
//    → 不定义 LL_LINK_LANE1/3；=2'b01（2 lane）→ 定义 LL_LINK_LANE1；=2'b10（4 lane）
//    → 定义 LL_LINK_LANE1+LL_LINK_LANE3。当前 link 侧为 1 lane（LANE_MACRO_MODE=2'b00）。
// ================================================================
//`define LL_LINK_LANE1          // 定义则每 link 下 CPIF lane 有 2 条（lane0+lane1）
//`define LL_LINK_LANE3          // 定义则每 link 下 CPIF lane 有 4 条（lane0~3，蕴含 lane1）
