// -------------------------------------------------------------------------------
// File   : hipi_ctrl_top.v
// Author : OpenHappy Team
// Date   : 2026-08-07
// -------------------------------------------------------------------------------
// Description:
//   HiPi 包 controller 顶层。例化协议层（`PRCL_AXI → AXI 协议层 axi4_asymm_prcl，
//   否则 → CXS 协议层 cxs_prcl）、链路层（link_layer）与 controller 公共 CSR（pcsr），完成：
//     - 3 路 APB 仲裁：paddr[11:10] 选段 → 00=pcsr（0x000-0x3ff）/ 01、10=协议层
//       （0x400-0xbff，AXI/CXS 共用）/ 11=link layer（0xc00-0xfff）；prdata / pready /
//       pslverr 寄存器输出。
//     - presetn 同步释放（top 内 rstn_sync）→ presetn_sync 分发至三模块。
//     - mb_rstn / sb_rstn 由 pcsr 输出提供协议层 / link 复位（sb_rstn 经 sb_clk 域
//       rstn_sync 同步释放后供 link_layer.sb_rst_n）。
//     - AXI / CXS 协议层与链路层的 paif/lp_sw 直连（内部 wire 共用，去掉 cxs_ 前缀）；
//       对外 PAIF 经 link_layer 中转。
//     - AXI 用户侧接口按 die 开关（MASTER_DIE → s*_axi / SLAVE_DIE → m*_axi，
//       由 AXI_MST/SLV_*_LANE_* 真源宏展开）分支；CXS 对外接口按 top 宏
//       PRCL_TX_LANE_*/PRCL_RX_LANE_* 分支。
//   cxs 例化、link 例化及对外 CXS/PAIF/CPIF/UDI 接口均按 PRCL_TX_LANE_*/PRCL_RX_LANE_*
//   （编译期推导 CXS_TX/RX_LANE* 与 LL_TX/RX_LANE*，后者由 TX_LANE_* 派生）完整支持 lane0~3 分支；
//   link 例化区 CPIF/UDI 的 lane 维度（每 link 下 CPIF/UDI lane 数）由 LL_LINK_LANE1/3 控制，与
//   link 维度（PRCL_TX/RX_LANE_*）正交。协议层选择由 `PRCL_AXI 宏控制（hipi_top_defines.vh）。
// -------------------------------------------------------------------------------
// Change Log:
//   2026-08-25  OpenHappy Team  v1.14  AXI 协议层 aresetn 同步释放：top 输入 aresetn 经
//                                common rstn_sync（u_rst_sync_aresetn，aclk 域，PRCL_AXI
//                                宏条件控制）同步释放后 → aresetn_sync，供 AXI 协议层
//                                （u_axi_prcl / u_iodie_axi_prcl）aresetn 端口使用
//   2026-08-25  OpenHappy Team  v1.13  AXI 数据位宽与 PAIF 解耦：AXI 用户侧 data/wstrb/rdata
//                                位宽由 `PAIF_DATA_WIDTH 改 `AXI_DATA_WIDTH（s*/m*_axi，
//                                由 PRCL_AXI 分支 AXI_DATA_WIDTH 宏派生，见 hipi_top_defines.vh）
//   2026-08-25  OpenHappy Team  v1.12  AXI 协议层 2G 时钟/复位端口改名同步：u_axi_prcl 例化
//                                .prtcl_clk→.prcl_clk、.prtcl_rstn→.prcl_rstn（对齐
//                                OpenHappy Team axi4_asymm_prcl.v 端口 prcl_clk/prcl_rstn；
//                                aclk/aresetn/pclk/presetn 不变）
//   2026-08-25  OpenHappy Team  v1.11  CXS 协议层 clken 扩 8bit：cxs_prcl gl_txclk_en/gl_rxclk_en
//                                [3:0]→[7:0]，与 top 的 8bit tx_clken/rx_clken 同宽直连
//                                （低 4bit 供 CXS lane0~3，高 4bit 保留）；sub_top clken 仍
//                                [3:0] 不变
//   2026-08-25  OpenHappy Team  v1.10  die 开关去 AXI_ 前缀（AXI_MASTER_DIE→MASTER_DIE、
//                                AXI_SLAVE_DIE→SLAVE_DIE），与 OpenHappy Team axi4 协议层（以裸宏
//                                MASTER_DIE/SLAVE_DIE 门控 PORT/REQ_RESP）统一；本模块端口
//                                门控与 OpenHappy Team axi4_asymm_prcl 对齐，解决 u_axi_prcl 实例
//                                104 个多余连接 fatal 错误（顶层编译通过）
//   2026-08-07  OpenHappy Team  v0.1  Initial top integration
//   2026-08-11  OpenHappy Team  v0.2  集成 AXI 非对称协议层：PRCL_AXI 分支例化
//                                axi4_asymm_prcl（AXI 用户口 s*_axi/m*_axi +
//                                aclk/aresetn 新端口）；PAIF/lp_sw/prcl 信号去 cxs_ 前缀
//                                共享；APB 段选改 paddr[11:10]（协议层 0x400-0xbff 合并）；
//                                CXS 例化移入 else 分支。
//   2026-08-11  OpenHappy Team  v0.3  风格修复（verilog-generator 规范）：AXI 用户口端口声明重排为
//                                每行 1 信号 + 5-point 对齐（352 个端口，与既有区段一致：
//                                `[`=17, `:`=46, name=51, `,`=81，行宽 ≤100）；
//                                AXI 例化端口连接紧排 3-point 对齐（`.`/`(`/`)`/`,` 对齐）；
//                                prdata 回读 mux 由拆行三元改为 case 语句（含 default）。
//   2026-08-11  OpenHappy Team  v0.4  对齐全面修复（verilog-reviewer §3.6）：全部端口声明统一
//                                5-point（name=51, `,`=81, `:`=46，修复 CPIF/UDI 宽度行错位）；
//                                cxs_prcl / link_layer 例化 3-point 紧排（修复 `(`/`)`/`,`
//                                错位）；AXI 例化对齐到 cxs 例化列（`(`=36, `)`=63, `,`=64）。
//   2026-08-11  OpenHappy Team  v0.5  宏命名统一：协议层位宽/特性宏加 CXSPRCL_ 前缀（与
//                                cxs_prcl_defines.vh 的宏区分），使用点同步修改；端口宽度
//                                表达式随之变化，重新 5-point 对齐。
//   2026-08-11  OpenHappy Team  v0.6  对齐全面检查修复（verilog-reviewer §3.6）：内部 wire/reg
//                                声明逐块 5-point 重排（wire/reg `[` 对齐）；assign 语句组
//                                `=` 表格对齐（prcl_mb_rstn 等重命名后错位修复）；端口 5-point、
//                                例化 3-point 复核。
//   2026-08-11  OpenHappy Team  v0.7  修复 link_layer 例化重复引脚：RX_LANE_1 link1 RX 块
//                                重复出现（cpif_rx_link1_lane0_data_i/_data_vld_i 等二次
//                                连接，复制粘贴错误），删除重复块后 verilator 编译通过。
//   2026-08-13  OpenHappy Team  v0.8  chk 端口位宽用宏：data 归一化 `CXSPRCL_DATAFLITWIDTH/8（8 的
//                                倍数，去冗余 (X+7)/8），cntl 用 `CXSPRCL_CNTL_CHK_BYTE（hipi_top_defines.vh）
//   2026-08-13  OpenHappy Team  v0.9  link_irq wire 位宽 [4:0]→[5:0]（匹配 link_layer irq_o 6-bit，
//                                修复 bit5 中断被截断丢失）
//   2026-08-14  OpenHappy Team  v1.0  中断输出寄存器化 + 跨时钟同步：hipi_ctrl_intr 改 output reg
//                                （pclk 域寄存器输出，替代原组合 assign）；新增 sync_cell
//                                将协议层中断 prcl_intr（AXI/CXS 两分支共用）同步至 pclk 域，
//                                与链路层中断 link_irq（1-bit，对齐链路层 irq_o 1-bit 规格，
//                                link RTL 待同步）相或后打拍输出。
//   2026-08-14  OpenHappy Team  v1.1  CXS 分支新增 gl_* 全局使能连接（来自 pcsr）：gl_txclk_en/gl_rxclk_en
//                                （clk_en）→ lane 时钟门控、gl_credit_en（credit_en）→ PAIF credit 总开关
//   2026-08-14  OpenHappy Team  v1.2  删除 loopback：pcsr loopback 寄存器移除（协议层/链路层回环改由协议层
//                                CSR 本地控制），撤销 gl_local_lb_en/gl_remote_lb_en 连接与 pcsr loopback 例化
//   2026-08-14  OpenHappy Team  v1.3  协议层 lane 宏加 PRCL_ 前缀：端口区（AXI/CXS）TX_LANE_* → PRCL_TX_LANE_*
//                                （link 例化区保留 TX_LANE_*，由 hipi_top_defines 派生提供）
//   2026-08-14  OpenHappy Team  v1.4  删除 pcsr credit_en 相关：pcsr 移除 credit_en 寄存器/端口；AXI 协议层
//                                credit_en 输入 tie1（1'b1，credit 流控始终使能）；CXS 例化删 gl_credit_en
//                                连接（改由 cxs_prcl_csr paif_credit_en 本地提供）
//   2026-08-14  OpenHappy Team  v1.5  link 例化区 lane 宏统一 PRCL_ 前缀：PAIF/CPIF/UDI 端口按 link 维度展开
//                                的守卫 TX_LANE_1/2/3 / RX_LANE_1/2/3 → PRCL_TX/RX_LANE_1/2/3（与协议层例化
//                                及 PAIF 中转 wire 声明一致；LL_LINK_LANE lane 维度守卫不变）。TX_LANE_*
//                                仍由 hipi_top_defines.vh PRCL_ 派生保留，供 link_layer/logic_phy/d2d 使用
//   2026-08-25  OpenHappy Team  v1.6  pcsr clk_en 扩到 16bit：tx_clken/rx_clken 位宽 [3:0]→[7:0]
//                                （下游 cxs_prcl/axi_prcl 仍为 [3:0]，本次仅扩 top 连线）
//   2026-08-25  OpenHappy Team  v1.7  AXI 协议层改 dual-mode（master/slave 双方向并存）：
//                                - 用户侧端口 s*_axi (master) / m*_axi (slave) 由 AXI_MST/SLV_*
//                                  lane 真源宏门控，两方向同模块并存（AXI_MASTER_DIE_MODE 单向制废弃）
//                                - 例化改新 OpenHappy Team AXI 接口：clken 拆 req/resp_tx/rx_clken
//                                  （mst 低4bit / slv 高4bit）；PAIF 改 mst_paif0→paif0 /
//                                  slv_paif0→paif1（mst 低组 + slv 高组）
//                                - AXI_* 真源宏与派生裸宏（MASTER_DIE/MST_*/SLV_*）双命名隔离，
//                                  见 hipi_top_defines.vh v0.12；AXI 私有宏供 OpenHappy Team AXI 子 filelist
//                                - cxs/link 的 PAIF 组数仍由 PRCL_TX/RX_LANE_* 控制（当前 2+2，
//                                  PRCL_TX/RX_LANE_1 使能 → paif0/paif1 双 link）
//   2026-08-25  OpenHappy Team  v1.9  方向级/lane0 门控改 die（对齐 OpenHappy Team axi4_asymm_prcl.v）：
//                                - master/slave 方向外层 user 端口块、req/resp clken、
//                                  mst/slv_paif0、s0/m0 用户口连接改由 AXI_MASTER_DIE /
//                                  AXI_SLAVE_DIE 门控（lane0 恒在，随 die 而非 lane 数）
//                                - s1~3/m1~3、mst/slv_paif1~3 仍由 AXI_MST/SLV_*_LANE_n 门控
//   2026-08-25  OpenHappy Team  v1.8  AXI dual-mode 宏重构对齐 hipi_top_defines.vh v0.13/0.14：
//                                - AXI_MST/SLV_*_LANE_* 真源宏移入 PRCL_TX/RX_LANE 分支内
//                                  派生（经 PRCL_AXI + die 开关门控），本模块按 lane 宏分支
//                                  自动随 die 开关生效，无需新增 die 级 ifdef（功能等价）
//                                - 无前缀裸宏（MASTER_DIE/MST_*/SLV_*）top 不再派生，改由
//                                  OpenHappy Team axi4_defines.vh 自定；本模块只用 AXI_ 前缀宏
//                                - 注释同步：AXI_MASTER_DIE_MODE 单向制 → die 开关表述
// -------------------------------------------------------------------------------

`default_nettype none

module hipi_ctrl_top
(
    // ==========================================================================
    //  Clock and reset
    // ==========================================================================
    input  wire                                        pclk                          , // APB clock
    input  wire                                        mb_clk                        ,
    input  wire                                        sb_clk                        ,
    input  wire                                        presetn                       , // APB reset (async)
`ifdef PRCL_AXI
    input  wire                                        aclk                          , // AXI4 用户侧接口时钟
    input  wire                                        aresetn                       , // AXI4 用户侧接口复位 (async)
`else
    input  wire                                        cxs_clk                       ,
    input  wire                                        cxs_rstn                      , // CXS domain reset
`endif

    // ==========================================================================
    //  APB interface (pclk 域)
    // ==========================================================================
    input  wire                                        psel                          ,
    input  wire                                        penable                       ,
    input  wire [15                               : 0] paddr                         , // 16bit, [11:10] 选段
    input  wire                                        pwrite                        ,
    input  wire [31                               : 0] pwdata                        ,
    output wire [31                               : 0] prdata                        , // comb（完成拍与 pready 同拍有效）
    output wire                                        pready                        , // comb
    output wire                                        pslverr                       , // comb（完成拍与 pready 同拍有效）

    // ==========================================================================
    //  Low-power interface (参考 cxs 协议层)
    // ==========================================================================
    input  wire [3                                : 0] lp_ctrl_cmd                   , // → cxs.lp_ctrl_cmd
    input  wire                                        lp_ctrl_req                   , // → cxs.lp_ctrl_req
    output wire [3                                : 0] lp_ctrl_sts                   , // ← cxs.lp_ctrl_sts
    output wire                                        lp_ctrl_ack                   , // ← cxs.lp_ctrl_ack

    // ==========================================================================
    //  Interrupt (聚合输出)
    // ==========================================================================
    output reg                                         hipi_ctrl_intr                ,

`ifdef PRCL_AXI
    // ==========================================================================
    //  AXI 用户侧接口 (对外, 四态优先级分支)
    //  分支顺序（PRCL_AXI 下）：AXI_CMPT_DIE → cmptdie_axi4_prcl（4R1T）；
    //    else AXI_IO_DIE → iodie_axi4_prcl（4T1R）；else → 旧 axi4_asymm_prcl
    //    （MASTER_DIE/SLAVE_DIE 决定类型）；PRCL_AXI 未定义 → `else 分支 CXS 协议层 cxs_prcl。
    //  AXI_CMPT_DIE / AXI_IO_DIE 互斥（分支优先级 cmptdie > iodie；top defines 仅可定义其一）：
    //    cmptdie (4R1T)：s0(完整 slave)+m0(完整 master) lane0 对称骨干恒在 +
    //      s1~3 AR/R（随 MST_RX_LANE_n）；无 m1~3、无 s1~3 AW/W/B；
    //    iodie (4T1R)：s0(完整 slave)+m0(完整 master) lane0 对称骨干恒在 +
    //      m1~3 AR/R（随 SLV_TX_LANE_n）；无 s1~3、无 m1~3 AW/W/B；
    //    asymm：master 方向 s*_axi（lane1~3 随 MST_TX/RX_LANE_n）/ slave 方向
    //      m*_axi（lane1~3 随 SLV_RX/TX_LANE_n），两 die 开关独立可并存。
    //  数据位宽取 AXI_DATA_WIDTH（PRCL_AXI 分支：PAIF 1024 → 1024；否则 AXI_DATA_WIDTH_EQ_256 → 256；否则 512）
    // ==========================================================================
`ifdef AXI_CMPT_DIE
    // ==========================================================================
    //  AXI 用户侧接口 — cmptdie 模式（cmptdie_axi4_prcl，4R1T）
    //  lane0 对称骨干：s0_axi（完整 5 通道 slave，AW/AR/W 输入、R/B 输出）+
    //    m0_axi（完整 5 通道 master，AW/AR/W 输出、R/B 输入）恒在；
    //  lane1~3 仅 RX AR 通道：s1~3_axi（仅 AR 输入 / R 输出，随 MST_RX_LANE_n）；
    //  无 m1~3、无 s1~3 AW/W/B（cmptdie 无 TX 扩展 lane，PRCL_TX_LANE_* 下禁止定义）。
    // ==========================================================================
    // ---- s0（lane0 对称骨干 slave，用户侧 AXI4 Slave）----
    // AW 通道（input）
    input  wire [9                                : 0] s0_axi_awid                   ,
    input  wire [63                               : 0] s0_axi_awaddr                 ,
    input  wire [7                                : 0] s0_axi_awlen                  ,
    input  wire [2                                : 0] s0_axi_awsize                 ,
    input  wire [1                                : 0] s0_axi_awburst                ,
    input  wire                                        s0_axi_awlock                 ,
    input  wire [3                                : 0] s0_axi_awcache                ,
    input  wire [2                                : 0] s0_axi_awprot                 ,
    input  wire [3                                : 0] s0_axi_awqos                  ,
    input  wire [3                                : 0] s0_axi_awregion               ,
    input  wire [3                                : 0] s0_axi_awuser                 ,
    input  wire                                        s0_axi_awvalid                ,
    output wire                                        s0_axi_awready                ,
    // AR 通道（input）
    input  wire [9                                : 0] s0_axi_arid                   ,
    input  wire [63                               : 0] s0_axi_araddr                 ,
    input  wire [7                                : 0] s0_axi_arlen                  ,
    input  wire [2                                : 0] s0_axi_arsize                 ,
    input  wire [1                                : 0] s0_axi_arburst                ,
    input  wire                                        s0_axi_arlock                 ,
    input  wire [3                                : 0] s0_axi_arcache                ,
    input  wire [2                                : 0] s0_axi_arprot                 ,
    input  wire [3                                : 0] s0_axi_arqos                  ,
    input  wire [3                                : 0] s0_axi_arregion               ,
    input  wire [3                                : 0] s0_axi_aruser                 ,
    input  wire                                        s0_axi_arvalid                ,
    output wire                                        s0_axi_arready                ,
    // W 通道（input）
    input  wire [`AXI_DATA_WIDTH-1                : 0] s0_axi_wdata                  ,
    input  wire [`AXI_DATA_WIDTH/8-1              : 0] s0_axi_wstrb                  ,
    input  wire [3                                : 0] s0_axi_wuser                  ,
    input  wire                                        s0_axi_wlast                  ,
    input  wire                                        s0_axi_wvalid                 ,
    output wire                                        s0_axi_wready                 ,
    // R 通道（output）
    output wire [9                                : 0] s0_axi_rid                   ,
    output wire [`AXI_DATA_WIDTH-1                : 0] s0_axi_rdata                  ,
    output wire [1                                : 0] s0_axi_rresp                  ,
    output wire [3                                : 0] s0_axi_ruser                  ,
    output wire                                        s0_axi_rlast                  ,
    output wire                                        s0_axi_rvalid                 ,
    input  wire                                        s0_axi_rready                 ,
    // B 通道（output）
    output wire [9                                : 0] s0_axi_bid                   ,
    output wire [1                                : 0] s0_axi_bresp                  ,
    output wire [3                                : 0] s0_axi_buser                  ,
    output wire                                        s0_axi_bvalid                 ,
    input  wire                                        s0_axi_bready                 ,
    // ---- s1~3（lane1~3 RX 扩展仅 AR 通道，随 MST_RX_LANE_n）----
`ifdef MST_RX_LANE_1
    // AR 通道（input）
    input  wire [9                                : 0] s1_axi_arid                   ,
    input  wire [63                               : 0] s1_axi_araddr                 ,
    input  wire [7                                : 0] s1_axi_arlen                  ,
    input  wire [2                                : 0] s1_axi_arsize                 ,
    input  wire [1                                : 0] s1_axi_arburst                ,
    input  wire                                        s1_axi_arlock                 ,
    input  wire [3                                : 0] s1_axi_arcache                ,
    input  wire [2                                : 0] s1_axi_arprot                 ,
    input  wire [3                                : 0] s1_axi_arqos                  ,
    input  wire [3                                : 0] s1_axi_arregion               ,
    input  wire [3                                : 0] s1_axi_aruser                 ,
    input  wire                                        s1_axi_arvalid                ,
    output wire                                        s1_axi_arready                ,
    // R 通道（output）
    output wire [9                                : 0] s1_axi_rid                    ,
    output wire [`AXI_DATA_WIDTH-1                : 0] s1_axi_rdata                  ,
    output wire [1                                : 0] s1_axi_rresp                  ,
    output wire [3                                : 0] s1_axi_ruser                  ,
    output wire                                        s1_axi_rlast                  ,
    output wire                                        s1_axi_rvalid                 ,
    input  wire                                        s1_axi_rready                 ,

`endif
`ifdef MST_RX_LANE_2
    // AR 通道（input）
    input  wire [9                                : 0] s2_axi_arid                   ,
    input  wire [63                               : 0] s2_axi_araddr                 ,
    input  wire [7                                : 0] s2_axi_arlen                  ,
    input  wire [2                                : 0] s2_axi_arsize                 ,
    input  wire [1                                : 0] s2_axi_arburst                ,
    input  wire                                        s2_axi_arlock                 ,
    input  wire [3                                : 0] s2_axi_arcache                ,
    input  wire [2                                : 0] s2_axi_arprot                 ,
    input  wire [3                                : 0] s2_axi_arqos                  ,
    input  wire [3                                : 0] s2_axi_arregion               ,
    input  wire [3                                : 0] s2_axi_aruser                 ,
    input  wire                                        s2_axi_arvalid                ,
    output wire                                        s2_axi_arready                ,
    // R 通道（output）
    output wire [9                                : 0] s2_axi_rid                    ,
    output wire [`AXI_DATA_WIDTH-1                : 0] s2_axi_rdata                  ,
    output wire [1                                : 0] s2_axi_rresp                  ,
    output wire [3                                : 0] s2_axi_ruser                  ,
    output wire                                        s2_axi_rlast                  ,
    output wire                                        s2_axi_rvalid                 ,
    input  wire                                        s2_axi_rready                 ,

`endif
`ifdef MST_RX_LANE_3
    // AR 通道（input）
    input  wire [9                                : 0] s3_axi_arid                   ,
    input  wire [63                               : 0] s3_axi_araddr                 ,
    input  wire [7                                : 0] s3_axi_arlen                  ,
    input  wire [2                                : 0] s3_axi_arsize                 ,
    input  wire [1                                : 0] s3_axi_arburst                ,
    input  wire                                        s3_axi_arlock                 ,
    input  wire [3                                : 0] s3_axi_arcache                ,
    input  wire [2                                : 0] s3_axi_arprot                 ,
    input  wire [3                                : 0] s3_axi_arqos                  ,
    input  wire [3                                : 0] s3_axi_arregion               ,
    input  wire [3                                : 0] s3_axi_aruser                 ,
    input  wire                                        s3_axi_arvalid                ,
    output wire                                        s3_axi_arready                ,
    // R 通道（output）
    output wire [9                                : 0] s3_axi_rid                    ,
    output wire [`AXI_DATA_WIDTH-1                : 0] s3_axi_rdata                  ,
    output wire [1                                : 0] s3_axi_rresp                  ,
    output wire [3                                : 0] s3_axi_ruser                  ,
    output wire                                        s3_axi_rlast                  ,
    output wire                                        s3_axi_rvalid                 ,
    input  wire                                        s3_axi_rready                 ,

`endif
    // ---- m0（lane0 对称骨干 master，用户侧 AXI4 Master）----
    // AW 通道（output）
    output wire [9                                : 0] m0_axi_awid                   ,
    output wire [63                               : 0] m0_axi_awaddr                 ,
    output wire [7                                : 0] m0_axi_awlen                  ,
    output wire [2                                : 0] m0_axi_awsize                 ,
    output wire [1                                : 0] m0_axi_awburst                ,
    output wire                                        m0_axi_awlock                 ,
    output wire [3                                : 0] m0_axi_awcache                ,
    output wire [2                                : 0] m0_axi_awprot                 ,
    output wire [3                                : 0] m0_axi_awqos                  ,
    output wire [3                                : 0] m0_axi_awregion               ,
    output wire [3                                : 0] m0_axi_awuser                 ,
    output wire                                        m0_axi_awvalid                ,
    input  wire                                        m0_axi_awready                ,
    // AR 通道（output）
    output wire [9                                : 0] m0_axi_arid                   ,
    output wire [63                               : 0] m0_axi_araddr                 ,
    output wire [7                                : 0] m0_axi_arlen                  ,
    output wire [2                                : 0] m0_axi_arsize                 ,
    output wire [1                                : 0] m0_axi_arburst                ,
    output wire                                        m0_axi_arlock                 ,
    output wire [3                                : 0] m0_axi_arcache                ,
    output wire [2                                : 0] m0_axi_arprot                 ,
    output wire [3                                : 0] m0_axi_arqos                  ,
    output wire [3                                : 0] m0_axi_arregion               ,
    output wire [3                                : 0] m0_axi_aruser                 ,
    output wire                                        m0_axi_arvalid                ,
    input  wire                                        m0_axi_arready                ,
    // W 通道（output）
    output wire [`AXI_DATA_WIDTH-1                : 0] m0_axi_wdata                  ,
    output wire [`AXI_DATA_WIDTH/8-1              : 0] m0_axi_wstrb                  ,
    output wire [3                                : 0] m0_axi_wuser                  ,
    output wire                                        m0_axi_wlast                  ,
    output wire                                        m0_axi_wvalid                 ,
    input  wire                                        m0_axi_wready                 ,
    // R 通道（input）
    input  wire [9                                : 0] m0_axi_rid                    ,
    input  wire [`AXI_DATA_WIDTH-1                : 0] m0_axi_rdata                  ,
    input  wire [1                                : 0] m0_axi_rresp                  ,
    input  wire [3                                : 0] m0_axi_ruser                  ,
    input  wire                                        m0_axi_rlast                  ,
    input  wire                                        m0_axi_rvalid                 ,
    output wire                                        m0_axi_rready                 ,
    // B 通道（input）
    input  wire [9                                : 0] m0_axi_bid                    ,
    input  wire [1                                : 0] m0_axi_bresp                  ,
    input  wire [3                                : 0] m0_axi_buser                  ,
    input  wire                                        m0_axi_bvalid                 ,
    output wire                                        m0_axi_bready                 ,
`elsif AXI_IO_DIE
    // ==========================================================================
    //  AXI 用户侧接口 — iodie 模式（iodie_axi4_prcl）
    //  lane0 对称骨干：s0_axi（完整 5 通道 slave，AW/AR/W 输入、R/B 输出）+
    //    m0_axi（完整 5 通道 master，AW/AR/W 输出、R/B 输入）恒在；
    //  lane1~3 仅 TX R 通道：m1~3_axi（仅 AR 输出 / R 输入，随 SLV_TX_LANE_n）；
    //  无 s1~3、无 m1~3 AW/W/B（iodie 无 slv RX 扩展 lane，PRCL_RX_LANE_* 下禁止定义）。
    // ==========================================================================
    // ---- s0（lane0 对称骨干 slave，用户侧 AXI4 Slave）----
    // AW 通道（input）
    input  wire [9                                : 0] s0_axi_awid                   ,
    input  wire [63                               : 0] s0_axi_awaddr                 ,
    input  wire [7                                : 0] s0_axi_awlen                  ,
    input  wire [2                                : 0] s0_axi_awsize                 ,
    input  wire [1                                : 0] s0_axi_awburst                ,
    input  wire                                        s0_axi_awlock                 ,
    input  wire [3                                : 0] s0_axi_awcache                ,
    input  wire [2                                : 0] s0_axi_awprot                 ,
    input  wire [3                                : 0] s0_axi_awqos                  ,
    input  wire [3                                : 0] s0_axi_awregion               ,
    input  wire [3                                : 0] s0_axi_awuser                 ,
    input  wire                                        s0_axi_awvalid                ,
    output wire                                        s0_axi_awready                ,
    // AR 通道（input）
    input  wire [9                                : 0] s0_axi_arid                   ,
    input  wire [63                               : 0] s0_axi_araddr                 ,
    input  wire [7                                : 0] s0_axi_arlen                  ,
    input  wire [2                                : 0] s0_axi_arsize                 ,
    input  wire [1                                : 0] s0_axi_arburst                ,
    input  wire                                        s0_axi_arlock                 ,
    input  wire [3                                : 0] s0_axi_arcache                ,
    input  wire [2                                : 0] s0_axi_arprot                 ,
    input  wire [3                                : 0] s0_axi_arqos                  ,
    input  wire [3                                : 0] s0_axi_arregion               ,
    input  wire [3                                : 0] s0_axi_aruser                 ,
    input  wire                                        s0_axi_arvalid                ,
    output wire                                        s0_axi_arready                ,
    // W 通道（input）
    input  wire [`AXI_DATA_WIDTH-1                : 0] s0_axi_wdata                  ,
    input  wire [`AXI_DATA_WIDTH/8-1              : 0] s0_axi_wstrb                  ,
    input  wire [3                                : 0] s0_axi_wuser                  ,
    input  wire                                        s0_axi_wlast                  ,
    input  wire                                        s0_axi_wvalid                 ,
    output wire                                        s0_axi_wready                 ,
    // R 通道（output）
    output wire [9                                : 0] s0_axi_rid                   ,
    output wire [`AXI_DATA_WIDTH-1                : 0] s0_axi_rdata                  ,
    output wire [1                                : 0] s0_axi_rresp                  ,
    output wire [3                                : 0] s0_axi_ruser                  ,
    output wire                                        s0_axi_rlast                  ,
    output wire                                        s0_axi_rvalid                 ,
    input  wire                                        s0_axi_rready                 ,
    // B 通道（output）
    output wire [9                                : 0] s0_axi_bid                   ,
    output wire [1                                : 0] s0_axi_bresp                  ,
    output wire [3                                : 0] s0_axi_buser                  ,
    output wire                                        s0_axi_bvalid                 ,
    input  wire                                        s0_axi_bready                 ,
    // ---- m0（lane0 对称骨干 master，用户侧 AXI4 Master）----
    // AW 通道（output）
    output wire [9                                : 0] m0_axi_awid                   ,
    output wire [63                               : 0] m0_axi_awaddr                 ,
    output wire [7                                : 0] m0_axi_awlen                  ,
    output wire [2                                : 0] m0_axi_awsize                 ,
    output wire [1                                : 0] m0_axi_awburst                ,
    output wire                                        m0_axi_awlock                 ,
    output wire [3                                : 0] m0_axi_awcache                ,
    output wire [2                                : 0] m0_axi_awprot                 ,
    output wire [3                                : 0] m0_axi_awqos                  ,
    output wire [3                                : 0] m0_axi_awregion               ,
    output wire [3                                : 0] m0_axi_awuser                 ,
    output wire                                        m0_axi_awvalid                ,
    input  wire                                        m0_axi_awready                ,
    // AR 通道（output）
    output wire [9                                : 0] m0_axi_arid                   ,
    output wire [63                               : 0] m0_axi_araddr                 ,
    output wire [7                                : 0] m0_axi_arlen                  ,
    output wire [2                                : 0] m0_axi_arsize                 ,
    output wire [1                                : 0] m0_axi_arburst                ,
    output wire                                        m0_axi_arlock                 ,
    output wire [3                                : 0] m0_axi_arcache                ,
    output wire [2                                : 0] m0_axi_arprot                 ,
    output wire [3                                : 0] m0_axi_arqos                  ,
    output wire [3                                : 0] m0_axi_arregion               ,
    output wire [3                                : 0] m0_axi_aruser                 ,
    output wire                                        m0_axi_arvalid                ,
    input  wire                                        m0_axi_arready                ,
    // W 通道（output）
    output wire [`AXI_DATA_WIDTH-1                : 0] m0_axi_wdata                  ,
    output wire [`AXI_DATA_WIDTH/8-1              : 0] m0_axi_wstrb                  ,
    output wire [3                                : 0] m0_axi_wuser                  ,
    output wire                                        m0_axi_wlast                  ,
    output wire                                        m0_axi_wvalid                 ,
    input  wire                                        m0_axi_wready                 ,
    // R 通道（input）
    input  wire [9                                : 0] m0_axi_rid                    ,
    input  wire [`AXI_DATA_WIDTH-1                : 0] m0_axi_rdata                  ,
    input  wire [1                                : 0] m0_axi_rresp                  ,
    input  wire [3                                : 0] m0_axi_ruser                  ,
    input  wire                                        m0_axi_rlast                  ,
    input  wire                                        m0_axi_rvalid                 ,
    output wire                                        m0_axi_rready                 ,
    // B 通道（input）
    input  wire [9                                : 0] m0_axi_bid                    ,
    input  wire [1                                : 0] m0_axi_bresp                  ,
    input  wire [3                                : 0] m0_axi_buser                  ,
    input  wire                                        m0_axi_bvalid                 ,
    output wire                                        m0_axi_bready                 ,
    // ---- m1~3（lane1~3 仅 TX R 通道，随 SLV_TX_LANE_n）----
    // AR 通道（output）/ R 通道（input）
`ifdef SLV_TX_LANE_1
    output wire [9                                : 0] m1_axi_arid                   ,
    output wire [63                               : 0] m1_axi_araddr                 ,
    output wire [7                                : 0] m1_axi_arlen                  ,
    output wire [2                                : 0] m1_axi_arsize                 ,
    output wire [1                                : 0] m1_axi_arburst                ,
    output wire                                        m1_axi_arlock                 ,
    output wire [3                                : 0] m1_axi_arcache                ,
    output wire [2                                : 0] m1_axi_arprot                 ,
    output wire [3                                : 0] m1_axi_arqos                  ,
    output wire [3                                : 0] m1_axi_arregion               ,
    output wire [3                                : 0] m1_axi_aruser                 ,
    output wire                                        m1_axi_arvalid                ,
    input  wire                                        m1_axi_arready                ,
    input  wire [9                                : 0] m1_axi_rid                    ,
    input  wire [`AXI_DATA_WIDTH-1                : 0] m1_axi_rdata                  ,
    input  wire [1                                : 0] m1_axi_rresp                  ,
    input  wire [3                                : 0] m1_axi_ruser                  ,
    input  wire                                        m1_axi_rlast                  ,
    input  wire                                        m1_axi_rvalid                 ,
    output wire                                        m1_axi_rready                 ,
`endif
`ifdef SLV_TX_LANE_2
    output wire [9                                : 0] m2_axi_arid                   ,
    output wire [63                               : 0] m2_axi_araddr                 ,
    output wire [7                                : 0] m2_axi_arlen                  ,
    output wire [2                                : 0] m2_axi_arsize                 ,
    output wire [1                                : 0] m2_axi_arburst                ,
    output wire                                        m2_axi_arlock                 ,
    output wire [3                                : 0] m2_axi_arcache                ,
    output wire [2                                : 0] m2_axi_arprot                 ,
    output wire [3                                : 0] m2_axi_arqos                  ,
    output wire [3                                : 0] m2_axi_arregion               ,
    output wire [3                                : 0] m2_axi_aruser                 ,
    output wire                                        m2_axi_arvalid                ,
    input  wire                                        m2_axi_arready                ,
    input  wire [9                                : 0] m2_axi_rid                    ,
    input  wire [`AXI_DATA_WIDTH-1                : 0] m2_axi_rdata                  ,
    input  wire [1                                : 0] m2_axi_rresp                  ,
    input  wire [3                                : 0] m2_axi_ruser                  ,
    input  wire                                        m2_axi_rlast                  ,
    input  wire                                        m2_axi_rvalid                 ,
    output wire                                        m2_axi_rready                 ,
`endif
`ifdef SLV_TX_LANE_3
    output wire [9                                : 0] m3_axi_arid                   ,
    output wire [63                               : 0] m3_axi_araddr                 ,
    output wire [7                                : 0] m3_axi_arlen                  ,
    output wire [2                                : 0] m3_axi_arsize                 ,
    output wire [1                                : 0] m3_axi_arburst                ,
    output wire                                        m3_axi_arlock                 ,
    output wire [3                                : 0] m3_axi_arcache                ,
    output wire [2                                : 0] m3_axi_arprot                 ,
    output wire [3                                : 0] m3_axi_arqos                  ,
    output wire [3                                : 0] m3_axi_arregion               ,
    output wire [3                                : 0] m3_axi_aruser                 ,
    output wire                                        m3_axi_arvalid                ,
    input  wire                                        m3_axi_arready                ,
    input  wire [9                                : 0] m3_axi_rid                    ,
    input  wire [`AXI_DATA_WIDTH-1                : 0] m3_axi_rdata                  ,
    input  wire [1                                : 0] m3_axi_rresp                  ,
    input  wire [3                                : 0] m3_axi_ruser                  ,
    input  wire                                        m3_axi_rlast                  ,
    input  wire                                        m3_axi_rvalid                 ,
    output wire                                        m3_axi_rready                 ,
`endif
`else
`ifdef AXI_ASYM_MODE
`ifdef MASTER_DIE
    // ---- req 方向（Master Die，用户侧 AXI4 Slave）----
    // AW 通道（input，s0 恒在；s1~3 随 TX_LANE_n）
    input  wire [9                                : 0] s0_axi_awid                   ,
    input  wire [63                               : 0] s0_axi_awaddr                 ,
    input  wire [7                                : 0] s0_axi_awlen                  ,
    input  wire [2                                : 0] s0_axi_awsize                 ,
    input  wire [1                                : 0] s0_axi_awburst                ,
    input  wire                                        s0_axi_awlock                 ,
    input  wire [3                                : 0] s0_axi_awcache                ,
    input  wire [2                                : 0] s0_axi_awprot                 ,
    input  wire [3                                : 0] s0_axi_awqos                  ,
    input  wire [3                                : 0] s0_axi_awregion               ,
    input  wire [3                                : 0] s0_axi_awuser                 ,
    input  wire                                        s0_axi_awvalid                ,
    output wire                                        s0_axi_awready                ,
`ifdef MST_TX_LANE_1
    input  wire [9                                : 0] s1_axi_awid                   ,
    input  wire [63                               : 0] s1_axi_awaddr                 ,
    input  wire [7                                : 0] s1_axi_awlen                  ,
    input  wire [2                                : 0] s1_axi_awsize                 ,
    input  wire [1                                : 0] s1_axi_awburst                ,
    input  wire                                        s1_axi_awlock                 ,
    input  wire [3                                : 0] s1_axi_awcache                ,
    input  wire [2                                : 0] s1_axi_awprot                 ,
    input  wire [3                                : 0] s1_axi_awqos                  ,
    input  wire [3                                : 0] s1_axi_awregion               ,
    input  wire [3                                : 0] s1_axi_awuser                 ,
    input  wire                                        s1_axi_awvalid                ,
    output wire                                        s1_axi_awready                ,
`endif
`ifdef MST_TX_LANE_2
    input  wire [9                                : 0] s2_axi_awid                   ,
    input  wire [63                               : 0] s2_axi_awaddr                 ,
    input  wire [7                                : 0] s2_axi_awlen                  ,
    input  wire [2                                : 0] s2_axi_awsize                 ,
    input  wire [1                                : 0] s2_axi_awburst                ,
    input  wire                                        s2_axi_awlock                 ,
    input  wire [3                                : 0] s2_axi_awcache                ,
    input  wire [2                                : 0] s2_axi_awprot                 ,
    input  wire [3                                : 0] s2_axi_awqos                  ,
    input  wire [3                                : 0] s2_axi_awregion               ,
    input  wire [3                                : 0] s2_axi_awuser                 ,
    input  wire                                        s2_axi_awvalid                ,
    output wire                                        s2_axi_awready                ,
`endif
`ifdef MST_TX_LANE_3
    input  wire [9                                : 0] s3_axi_awid                   ,
    input  wire [63                               : 0] s3_axi_awaddr                 ,
    input  wire [7                                : 0] s3_axi_awlen                  ,
    input  wire [2                                : 0] s3_axi_awsize                 ,
    input  wire [1                                : 0] s3_axi_awburst                ,
    input  wire                                        s3_axi_awlock                 ,
    input  wire [3                                : 0] s3_axi_awcache                ,
    input  wire [2                                : 0] s3_axi_awprot                 ,
    input  wire [3                                : 0] s3_axi_awqos                  ,
    input  wire [3                                : 0] s3_axi_awregion               ,
    input  wire [3                                : 0] s3_axi_awuser                 ,
    input  wire                                        s3_axi_awvalid                ,
    output wire                                        s3_axi_awready                ,
`endif
    // AR 通道（input，s0 恒在；s1~3 随 RX_LANE_n）
    input  wire [9                                : 0] s0_axi_arid                   ,
    input  wire [63                               : 0] s0_axi_araddr                 ,
    input  wire [7                                : 0] s0_axi_arlen                  ,
    input  wire [2                                : 0] s0_axi_arsize                 ,
    input  wire [1                                : 0] s0_axi_arburst                ,
    input  wire                                        s0_axi_arlock                 ,
    input  wire [3                                : 0] s0_axi_arcache                ,
    input  wire [2                                : 0] s0_axi_arprot                 ,
    input  wire [3                                : 0] s0_axi_arqos                  ,
    input  wire [3                                : 0] s0_axi_arregion               ,
    input  wire [3                                : 0] s0_axi_aruser                 ,
    input  wire                                        s0_axi_arvalid                ,
    output wire                                        s0_axi_arready                ,
`ifdef MST_RX_LANE_1
    input  wire [9                                : 0] s1_axi_arid                   ,
    input  wire [63                               : 0] s1_axi_araddr                 ,
    input  wire [7                                : 0] s1_axi_arlen                  ,
    input  wire [2                                : 0] s1_axi_arsize                 ,
    input  wire [1                                : 0] s1_axi_arburst                ,
    input  wire                                        s1_axi_arlock                 ,
    input  wire [3                                : 0] s1_axi_arcache                ,
    input  wire [2                                : 0] s1_axi_arprot                 ,
    input  wire [3                                : 0] s1_axi_arqos                  ,
    input  wire [3                                : 0] s1_axi_arregion               ,
    input  wire [3                                : 0] s1_axi_aruser                 ,
    input  wire                                        s1_axi_arvalid                ,
    output wire                                        s1_axi_arready                ,
`endif
`ifdef MST_RX_LANE_2
    input  wire [9                                : 0] s2_axi_arid                   ,
    input  wire [63                               : 0] s2_axi_araddr                 ,
    input  wire [7                                : 0] s2_axi_arlen                  ,
    input  wire [2                                : 0] s2_axi_arsize                 ,
    input  wire [1                                : 0] s2_axi_arburst                ,
    input  wire                                        s2_axi_arlock                 ,
    input  wire [3                                : 0] s2_axi_arcache                ,
    input  wire [2                                : 0] s2_axi_arprot                 ,
    input  wire [3                                : 0] s2_axi_arqos                  ,
    input  wire [3                                : 0] s2_axi_arregion               ,
    input  wire [3                                : 0] s2_axi_aruser                 ,
    input  wire                                        s2_axi_arvalid                ,
    output wire                                        s2_axi_arready                ,
`endif
`ifdef MST_RX_LANE_3
    input  wire [9                                : 0] s3_axi_arid                   ,
    input  wire [63                               : 0] s3_axi_araddr                 ,
    input  wire [7                                : 0] s3_axi_arlen                  ,
    input  wire [2                                : 0] s3_axi_arsize                 ,
    input  wire [1                                : 0] s3_axi_arburst                ,
    input  wire                                        s3_axi_arlock                 ,
    input  wire [3                                : 0] s3_axi_arcache                ,
    input  wire [2                                : 0] s3_axi_arprot                 ,
    input  wire [3                                : 0] s3_axi_arqos                  ,
    input  wire [3                                : 0] s3_axi_arregion               ,
    input  wire [3                                : 0] s3_axi_aruser                 ,
    input  wire                                        s3_axi_arvalid                ,
    output wire                                        s3_axi_arready                ,
`endif
    // W 通道（input，s0 恒在；s1~3 随 TX_LANE_n）
    input  wire [`AXI_DATA_WIDTH-1                : 0] s0_axi_wdata                  ,
    input  wire [`AXI_DATA_WIDTH/8-1              : 0] s0_axi_wstrb                  ,
    input  wire [3                                : 0] s0_axi_wuser                  ,
    input  wire                                        s0_axi_wlast                  ,
    input  wire                                        s0_axi_wvalid                 ,
    output wire                                        s0_axi_wready                 ,
`ifdef MST_TX_LANE_1
    input  wire [`AXI_DATA_WIDTH-1                : 0] s1_axi_wdata                  ,
    input  wire [`AXI_DATA_WIDTH/8-1              : 0] s1_axi_wstrb                  ,
    input  wire [3                                : 0] s1_axi_wuser                  ,
    input  wire                                        s1_axi_wlast                  ,
    input  wire                                        s1_axi_wvalid                 ,
    output wire                                        s1_axi_wready                 ,
`endif
`ifdef MST_TX_LANE_2
    input  wire [`AXI_DATA_WIDTH-1                : 0] s2_axi_wdata                  ,
    input  wire [`AXI_DATA_WIDTH/8-1              : 0] s2_axi_wstrb                  ,
    input  wire [3                                : 0] s2_axi_wuser                  ,
    input  wire                                        s2_axi_wlast                  ,
    input  wire                                        s2_axi_wvalid                 ,
    output wire                                        s2_axi_wready                 ,
`endif
`ifdef MST_TX_LANE_3
    input  wire [`AXI_DATA_WIDTH-1                : 0] s3_axi_wdata                  ,
    input  wire [`AXI_DATA_WIDTH/8-1              : 0] s3_axi_wstrb                  ,
    input  wire [3                                : 0] s3_axi_wuser                  ,
    input  wire                                        s3_axi_wlast                  ,
    input  wire                                        s3_axi_wvalid                 ,
    output wire                                        s3_axi_wready                 ,
`endif
    // R 通道（output，s0 恒在；s1~3 随 RX_LANE_n）
    output wire [9                                : 0] s0_axi_rid                    ,
    output wire [`AXI_DATA_WIDTH-1                : 0] s0_axi_rdata                  ,
    output wire [1                                : 0] s0_axi_rresp                  ,
    output wire [3                                : 0] s0_axi_ruser                  ,
    output wire                                        s0_axi_rlast                  ,
    output wire                                        s0_axi_rvalid                 ,
    input  wire                                        s0_axi_rready                 ,
`ifdef MST_RX_LANE_1
    output wire [9                                : 0] s1_axi_rid                    ,
    output wire [`AXI_DATA_WIDTH-1                : 0] s1_axi_rdata                  ,
    output wire [1                                : 0] s1_axi_rresp                  ,
    output wire [3                                : 0] s1_axi_ruser                  ,
    output wire                                        s1_axi_rlast                  ,
    output wire                                        s1_axi_rvalid                 ,
    input  wire                                        s1_axi_rready                 ,
`endif
`ifdef MST_RX_LANE_2
    output wire [9                                : 0] s2_axi_rid                    ,
    output wire [`AXI_DATA_WIDTH-1                : 0] s2_axi_rdata                  ,
    output wire [1                                : 0] s2_axi_rresp                  ,
    output wire [3                                : 0] s2_axi_ruser                  ,
    output wire                                        s2_axi_rlast                  ,
    output wire                                        s2_axi_rvalid                 ,
    input  wire                                        s2_axi_rready                 ,
`endif
`ifdef MST_RX_LANE_3
    output wire [9                                : 0] s3_axi_rid                    ,
    output wire [`AXI_DATA_WIDTH-1                : 0] s3_axi_rdata                  ,
    output wire [1                                : 0] s3_axi_rresp                  ,
    output wire [3                                : 0] s3_axi_ruser                  ,
    output wire                                        s3_axi_rlast                  ,
    output wire                                        s3_axi_rvalid                 ,
    input  wire                                        s3_axi_rready                 ,
`endif
    // B 通道（output，s0 恒在；s1~3 随 TX_LANE_n）
    output wire [9                                : 0] s0_axi_bid                    ,
    output wire [1                                : 0] s0_axi_bresp                  ,
    output wire [3                                : 0] s0_axi_buser                  ,
    output wire                                        s0_axi_bvalid                 ,
    input  wire                                        s0_axi_bready                 ,
`ifdef MST_TX_LANE_1
    output wire [9                                : 0] s1_axi_bid                    ,
    output wire [1                                : 0] s1_axi_bresp                  ,
    output wire [3                                : 0] s1_axi_buser                  ,
    output wire                                        s1_axi_bvalid                 ,
    input  wire                                        s1_axi_bready                 ,
`endif
`ifdef MST_TX_LANE_2
    output wire [9                                : 0] s2_axi_bid                    ,
    output wire [1                                : 0] s2_axi_bresp                  ,
    output wire [3                                : 0] s2_axi_buser                  ,
    output wire                                        s2_axi_bvalid                 ,
    input  wire                                        s2_axi_bready                 ,
`endif
`ifdef MST_TX_LANE_3
    output wire [9                                : 0] s3_axi_bid                    ,
    output wire [1                                : 0] s3_axi_bresp                  ,
    output wire [3                                : 0] s3_axi_buser                  ,
    output wire                                        s3_axi_bvalid                 ,
    input  wire                                        s3_axi_bready                 ,
`endif
`endif
`ifdef SLAVE_DIE
    // ---- resp 方向（Slave Die，用户侧 AXI4 Master）----
    // AW 通道（output，m0 恒在；m1~3 随 RX_LANE_n）
    output wire [9                                : 0] m0_axi_awid                   ,
    output wire [63                               : 0] m0_axi_awaddr                 ,
    output wire [7                                : 0] m0_axi_awlen                  ,
    output wire [2                                : 0] m0_axi_awsize                 ,
    output wire [1                                : 0] m0_axi_awburst                ,
    output wire                                        m0_axi_awlock                 ,
    output wire [3                                : 0] m0_axi_awcache                ,
    output wire [2                                : 0] m0_axi_awprot                 ,
    output wire [3                                : 0] m0_axi_awqos                  ,
    output wire [3                                : 0] m0_axi_awregion               ,
    output wire [3                                : 0] m0_axi_awuser                 ,
    output wire                                        m0_axi_awvalid                ,
    input  wire                                        m0_axi_awready                ,
`ifdef SLV_RX_LANE_1
    output wire [9                                : 0] m1_axi_awid                   ,
    output wire [63                               : 0] m1_axi_awaddr                 ,
    output wire [7                                : 0] m1_axi_awlen                  ,
    output wire [2                                : 0] m1_axi_awsize                 ,
    output wire [1                                : 0] m1_axi_awburst                ,
    output wire                                        m1_axi_awlock                 ,
    output wire [3                                : 0] m1_axi_awcache                ,
    output wire [2                                : 0] m1_axi_awprot                 ,
    output wire [3                                : 0] m1_axi_awqos                  ,
    output wire [3                                : 0] m1_axi_awregion               ,
    output wire [3                                : 0] m1_axi_awuser                 ,
    output wire                                        m1_axi_awvalid                ,
    input  wire                                        m1_axi_awready                ,
`endif
`ifdef SLV_RX_LANE_2
    output wire [9                                : 0] m2_axi_awid                   ,
    output wire [63                               : 0] m2_axi_awaddr                 ,
    output wire [7                                : 0] m2_axi_awlen                  ,
    output wire [2                                : 0] m2_axi_awsize                 ,
    output wire [1                                : 0] m2_axi_awburst                ,
    output wire                                        m2_axi_awlock                 ,
    output wire [3                                : 0] m2_axi_awcache                ,
    output wire [2                                : 0] m2_axi_awprot                 ,
    output wire [3                                : 0] m2_axi_awqos                  ,
    output wire [3                                : 0] m2_axi_awregion               ,
    output wire [3                                : 0] m2_axi_awuser                 ,
    output wire                                        m2_axi_awvalid                ,
    input  wire                                        m2_axi_awready                ,
`endif
`ifdef SLV_RX_LANE_3
    output wire [9                                : 0] m3_axi_awid                   ,
    output wire [63                               : 0] m3_axi_awaddr                 ,
    output wire [7                                : 0] m3_axi_awlen                  ,
    output wire [2                                : 0] m3_axi_awsize                 ,
    output wire [1                                : 0] m3_axi_awburst                ,
    output wire                                        m3_axi_awlock                 ,
    output wire [3                                : 0] m3_axi_awcache                ,
    output wire [2                                : 0] m3_axi_awprot                 ,
    output wire [3                                : 0] m3_axi_awqos                  ,
    output wire [3                                : 0] m3_axi_awregion               ,
    output wire [3                                : 0] m3_axi_awuser                 ,
    output wire                                        m3_axi_awvalid                ,
    input  wire                                        m3_axi_awready                ,
`endif
    // AR 通道（output，m0 恒在；m1~3 随 TX_LANE_n）
    output wire [9                                : 0] m0_axi_arid                   ,
    output wire [63                               : 0] m0_axi_araddr                 ,
    output wire [7                                : 0] m0_axi_arlen                  ,
    output wire [2                                : 0] m0_axi_arsize                 ,
    output wire [1                                : 0] m0_axi_arburst                ,
    output wire                                        m0_axi_arlock                 ,
    output wire [3                                : 0] m0_axi_arcache                ,
    output wire [2                                : 0] m0_axi_arprot                 ,
    output wire [3                                : 0] m0_axi_arqos                  ,
    output wire [3                                : 0] m0_axi_arregion               ,
    output wire [3                                : 0] m0_axi_aruser                 ,
    output wire                                        m0_axi_arvalid                ,
    input  wire                                        m0_axi_arready                ,
`ifdef SLV_TX_LANE_1
    output wire [9                                : 0] m1_axi_arid                   ,
    output wire [63                               : 0] m1_axi_araddr                 ,
    output wire [7                                : 0] m1_axi_arlen                  ,
    output wire [2                                : 0] m1_axi_arsize                 ,
    output wire [1                                : 0] m1_axi_arburst                ,
    output wire                                        m1_axi_arlock                 ,
    output wire [3                                : 0] m1_axi_arcache                ,
    output wire [2                                : 0] m1_axi_arprot                 ,
    output wire [3                                : 0] m1_axi_arqos                  ,
    output wire [3                                : 0] m1_axi_arregion               ,
    output wire [3                                : 0] m1_axi_aruser                 ,
    output wire                                        m1_axi_arvalid                ,
    input  wire                                        m1_axi_arready                ,
`endif
`ifdef SLV_TX_LANE_2
    output wire [9                                : 0] m2_axi_arid                   ,
    output wire [63                               : 0] m2_axi_araddr                 ,
    output wire [7                                : 0] m2_axi_arlen                  ,
    output wire [2                                : 0] m2_axi_arsize                 ,
    output wire [1                                : 0] m2_axi_arburst                ,
    output wire                                        m2_axi_arlock                 ,
    output wire [3                                : 0] m2_axi_arcache                ,
    output wire [2                                : 0] m2_axi_arprot                 ,
    output wire [3                                : 0] m2_axi_arqos                  ,
    output wire [3                                : 0] m2_axi_arregion               ,
    output wire [3                                : 0] m2_axi_aruser                 ,
    output wire                                        m2_axi_arvalid                ,
    input  wire                                        m2_axi_arready                ,
`endif
`ifdef SLV_TX_LANE_3
    output wire [9                                : 0] m3_axi_arid                   ,
    output wire [63                               : 0] m3_axi_araddr                 ,
    output wire [7                                : 0] m3_axi_arlen                  ,
    output wire [2                                : 0] m3_axi_arsize                 ,
    output wire [1                                : 0] m3_axi_arburst                ,
    output wire                                        m3_axi_arlock                 ,
    output wire [3                                : 0] m3_axi_arcache                ,
    output wire [2                                : 0] m3_axi_arprot                 ,
    output wire [3                                : 0] m3_axi_arqos                  ,
    output wire [3                                : 0] m3_axi_arregion               ,
    output wire [3                                : 0] m3_axi_aruser                 ,
    output wire                                        m3_axi_arvalid                ,
    input  wire                                        m3_axi_arready                ,
`endif
    // W 通道（output，m0 恒在；m1~3 随 RX_LANE_n）
    output wire [`AXI_DATA_WIDTH-1                : 0] m0_axi_wdata                  ,
    output wire [`AXI_DATA_WIDTH/8-1              : 0] m0_axi_wstrb                  ,
    output wire [3                                : 0] m0_axi_wuser                  ,
    output wire                                        m0_axi_wlast                  ,
    output wire                                        m0_axi_wvalid                 ,
    input  wire                                        m0_axi_wready                 ,
`ifdef SLV_RX_LANE_1
    output wire [`AXI_DATA_WIDTH-1                : 0] m1_axi_wdata                  ,
    output wire [`AXI_DATA_WIDTH/8-1              : 0] m1_axi_wstrb                  ,
    output wire [3                                : 0] m1_axi_wuser                  ,
    output wire                                        m1_axi_wlast                  ,
    output wire                                        m1_axi_wvalid                 ,
    input  wire                                        m1_axi_wready                 ,
`endif
`ifdef SLV_RX_LANE_2
    output wire [`AXI_DATA_WIDTH-1                : 0] m2_axi_wdata                  ,
    output wire [`AXI_DATA_WIDTH/8-1              : 0] m2_axi_wstrb                  ,
    output wire [3                                : 0] m2_axi_wuser                  ,
    output wire                                        m2_axi_wlast                  ,
    output wire                                        m2_axi_wvalid                 ,
    input  wire                                        m2_axi_wready                 ,
`endif
`ifdef SLV_RX_LANE_3
    output wire [`AXI_DATA_WIDTH-1                : 0] m3_axi_wdata                  ,
    output wire [`AXI_DATA_WIDTH/8-1              : 0] m3_axi_wstrb                  ,
    output wire [3                                : 0] m3_axi_wuser                  ,
    output wire                                        m3_axi_wlast                  ,
    output wire                                        m3_axi_wvalid                 ,
    input  wire                                        m3_axi_wready                 ,
`endif
    // R 通道（input，m0 恒在；m1~3 随 TX_LANE_n）
    input  wire [9                                : 0] m0_axi_rid                    ,
    input  wire [`AXI_DATA_WIDTH-1                : 0] m0_axi_rdata                  ,
    input  wire [1                                : 0] m0_axi_rresp                  ,
    input  wire [3                                : 0] m0_axi_ruser                  ,
    input  wire                                        m0_axi_rlast                  ,
    input  wire                                        m0_axi_rvalid                 ,
    output wire                                        m0_axi_rready                 ,
`ifdef SLV_TX_LANE_1
    input  wire [9                                : 0] m1_axi_rid                    ,
    input  wire [`AXI_DATA_WIDTH-1                : 0] m1_axi_rdata                  ,
    input  wire [1                                : 0] m1_axi_rresp                  ,
    input  wire [3                                : 0] m1_axi_ruser                  ,
    input  wire                                        m1_axi_rlast                  ,
    input  wire                                        m1_axi_rvalid                 ,
    output wire                                        m1_axi_rready                 ,
`endif
`ifdef SLV_TX_LANE_2
    input  wire [9                                : 0] m2_axi_rid                    ,
    input  wire [`AXI_DATA_WIDTH-1                : 0] m2_axi_rdata                  ,
    input  wire [1                                : 0] m2_axi_rresp                  ,
    input  wire [3                                : 0] m2_axi_ruser                  ,
    input  wire                                        m2_axi_rlast                  ,
    input  wire                                        m2_axi_rvalid                 ,
    output wire                                        m2_axi_rready                 ,
`endif
`ifdef SLV_TX_LANE_3
    input  wire [9                                : 0] m3_axi_rid                    ,
    input  wire [`AXI_DATA_WIDTH-1                : 0] m3_axi_rdata                  ,
    input  wire [1                                : 0] m3_axi_rresp                  ,
    input  wire [3                                : 0] m3_axi_ruser                  ,
    input  wire                                        m3_axi_rlast                  ,
    input  wire                                        m3_axi_rvalid                 ,
    output wire                                        m3_axi_rready                 ,
`endif
    // B 通道（input，m0 恒在；m1~3 随 RX_LANE_n）
    input  wire [9                                : 0] m0_axi_bid                    ,
    input  wire [1                                : 0] m0_axi_bresp                  ,
    input  wire [3                                : 0] m0_axi_buser                  ,
    input  wire                                        m0_axi_bvalid                 ,
    output wire                                        m0_axi_bready                 ,
`ifdef SLV_RX_LANE_1
    input  wire [9                                : 0] m1_axi_bid                    ,
    input  wire [1                                : 0] m1_axi_bresp                  ,
    input  wire [3                                : 0] m1_axi_buser                  ,
    input  wire                                        m1_axi_bvalid                 ,
    output wire                                        m1_axi_bready                 ,
`endif
`ifdef SLV_RX_LANE_2
    input  wire [9                                : 0] m2_axi_bid                    ,
    input  wire [1                                : 0] m2_axi_bresp                  ,
    input  wire [3                                : 0] m2_axi_buser                  ,
    input  wire                                        m2_axi_bvalid                 ,
    output wire                                        m2_axi_bready                 ,
`endif
`ifdef SLV_RX_LANE_3
    input  wire [9                                : 0] m3_axi_bid                    ,
    input  wire [1                                : 0] m3_axi_bresp                  ,
    input  wire [3                                : 0] m3_axi_buser                  ,
    input  wire                                        m3_axi_bvalid                 ,
    output wire                                        m3_axi_bready                 ,
`endif
`endif
`else
    // ==========================================================================
    //  AXI 对称模式（AXI_ASYM_MODE 未定义时，1T1R 对称）：全双工单组 s_axi + m_axi
    //  方向与 asymm 一致：s 输入（AXI 用户侧 slave）/ m 输出（AXI 用户侧 master）；
    //  名称沿用 s0_axi_*/m0_axi_*（与 v2 顶层对齐）；随 PRCL_TX_LANE_1/2/3 展开多组
    //  ==========================================================================
    // ---- s (slave，AXI 用户侧输入，方向与 asymm 的 s0 一致) ----
    // AW/AR/W 通道（input）
    input  wire [9                                : 0] s0_axi_awid                   ,
    input  wire [63                               : 0] s0_axi_awaddr                 ,
    input  wire [7                                : 0] s0_axi_awlen                  ,
    input  wire [2                                : 0] s0_axi_awsize                 ,
    input  wire [1                                : 0] s0_axi_awburst                ,
    input  wire                                        s0_axi_awlock                 ,
    input  wire [3                                : 0] s0_axi_awcache                ,
    input  wire [2                                : 0] s0_axi_awprot                 ,
    input  wire [3                                : 0] s0_axi_awqos                  ,
    input  wire [3                                : 0] s0_axi_awregion               ,
    input  wire [3                                : 0] s0_axi_awuser                 ,
    input  wire                                        s0_axi_awvalid                ,
    output wire                                        s0_axi_awready                ,
    input  wire [9                                : 0] s0_axi_arid                   ,
    input  wire [63                               : 0] s0_axi_araddr                 ,
    input  wire [7                                : 0] s0_axi_arlen                  ,
    input  wire [2                                : 0] s0_axi_arsize                 ,
    input  wire [1                                : 0] s0_axi_arburst                ,
    input  wire                                        s0_axi_arlock                 ,
    input  wire [3                                : 0] s0_axi_arcache                ,
    input  wire [2                                : 0] s0_axi_arprot                 ,
    input  wire [3                                : 0] s0_axi_arqos                  ,
    input  wire [3                                : 0] s0_axi_arregion               ,
    input  wire [3                                : 0] s0_axi_aruser                 ,
    input  wire                                        s0_axi_arvalid                ,
    output wire                                        s0_axi_arready                ,
    input  wire [`AXI_DATA_WIDTH-1                : 0] s0_axi_wdata                  ,
    input  wire [`AXI_DATA_WIDTH/8-1              : 0] s0_axi_wstrb                  ,
    input  wire [3                                : 0] s0_axi_wuser                  ,
    input  wire                                        s0_axi_wlast                  ,
    input  wire                                        s0_axi_wvalid                 ,
    output wire                                        s0_axi_wready                 ,
    // R/B 通道（output）
    output wire [9                                : 0] s0_axi_rid                   ,
    output wire [`AXI_DATA_WIDTH-1                : 0] s0_axi_rdata                  ,
    output wire [1                                : 0] s0_axi_rresp                  ,
    output wire [3                                : 0] s0_axi_ruser                  ,
    output wire                                        s0_axi_rlast                  ,
    output wire                                        s0_axi_rvalid                 ,
    input  wire                                        s0_axi_rready                 ,
    output wire [9                                : 0] s0_axi_bid                   ,
    output wire [1                                : 0] s0_axi_bresp                  ,
    output wire [3                                : 0] s0_axi_buser                  ,
    output wire                                        s0_axi_bvalid                 ,
    input  wire                                        s0_axi_bready                 ,
    // ---- m (master，AXI 用户侧输出，方向与 asymm 的 m0 一致) ----
    // AW/AR/W 通道（output）
    output wire [9                                : 0] m0_axi_awid                   ,
    output wire [63                               : 0] m0_axi_awaddr                 ,
    output wire [7                                : 0] m0_axi_awlen                  ,
    output wire [2                                : 0] m0_axi_awsize                 ,
    output wire [1                                : 0] m0_axi_awburst                ,
    output wire                                        m0_axi_awlock                 ,
    output wire [3                                : 0] m0_axi_awcache                ,
    output wire [2                                : 0] m0_axi_awprot                 ,
    output wire [3                                : 0] m0_axi_awqos                  ,
    output wire [3                                : 0] m0_axi_awregion               ,
    output wire [3                                : 0] m0_axi_awuser                 ,
    output wire                                        m0_axi_awvalid                ,
    input  wire                                        m0_axi_awready                ,
    output wire [9                                : 0] m0_axi_arid                   ,
    output wire [63                               : 0] m0_axi_araddr                 ,
    output wire [7                                : 0] m0_axi_arlen                  ,
    output wire [2                                : 0] m0_axi_arsize                 ,
    output wire [1                                : 0] m0_axi_arburst                ,
    output wire                                        m0_axi_arlock                 ,
    output wire [3                                : 0] m0_axi_arcache                ,
    output wire [2                                : 0] m0_axi_arprot                 ,
    output wire [3                                : 0] m0_axi_arqos                  ,
    output wire [3                                : 0] m0_axi_arregion               ,
    output wire [3                                : 0] m0_axi_aruser                 ,
    output wire                                        m0_axi_arvalid                ,
    input  wire                                        m0_axi_arready                ,
    output wire [`AXI_DATA_WIDTH-1                : 0] m0_axi_wdata                  ,
    output wire [`AXI_DATA_WIDTH/8-1              : 0] m0_axi_wstrb                  ,
    output wire [3                                : 0] m0_axi_wuser                  ,
    output wire                                        m0_axi_wlast                  ,
    output wire                                        m0_axi_wvalid                 ,
    input  wire                                        m0_axi_wready                 ,
    // R/B 通道（input）
    input  wire [9                                : 0] m0_axi_rid                   ,
    input  wire [`AXI_DATA_WIDTH-1                : 0] m0_axi_rdata                  ,
    input  wire [1                                : 0] m0_axi_rresp                  ,
    input  wire [3                                : 0] m0_axi_ruser                  ,
    input  wire                                        m0_axi_rlast                  ,
    input  wire                                        m0_axi_rvalid                 ,
    output wire                                        m0_axi_rready                 ,
    input  wire [9                                : 0] m0_axi_bid                   ,
    input  wire [1                                : 0] m0_axi_bresp                  ,
    input  wire [3                                : 0] m0_axi_buser                  ,
    input  wire                                        m0_axi_bvalid                 ,
    output wire                                        m0_axi_bready                 ,
`endif
`endif
`else
    // ==========================================================================
    //  CXS interface (对外, 按 top TX/RX lane 宏分支)
    //  TX lane → cxs_prcl{n}_rx_* (cxs top 的 CXS RX 方向, 上游→协议层)
    // ==========================================================================
    input  wire [`CXSPRCL_DATAFLITWIDTH-1         : 0] cxs_prcl0_rx_data             ,
    input  wire [`CXSPRCL_CNTL_WIDTH-1            : 0] cxs_prcl0_rx_cntl             ,
    input  wire                                        cxs_prcl0_rx_valid            ,
    input  wire [1                                : 0] cxs_prcl0_rx_prcltype         ,
    input  wire [`CXSPRCL_SRCID_WIDTH-1           : 0] cxs_prcl0_rx_srcid            ,
    input  wire [`CXSPRCL_TGTID_WIDTH-1           : 0] cxs_prcl0_rx_tgtid            ,
`ifdef CXSPRCL_LAST
    input  wire                                        cxs_prcl0_rx_last             ,
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
    input  wire [(`CXSPRCL_DATAFLITWIDTH/8)-1     : 0] cxs_prcl0_rx_chk_data         ,
    input  wire [`CXSPRCL_CNTL_CHK_BYTE-1         : 0] cxs_prcl0_rx_chk_cntl         ,
    input  wire                                        cxs_prcl0_rx_chk_prcltype     ,
    input  wire                                        cxs_prcl0_rx_chk_srcid        ,
    input  wire                                        cxs_prcl0_rx_chk_tgtid        ,
    input  wire                                        cxs_prcl0_rx_chk_last         ,
    input  wire                                        cxs_prcl0_rx_chk_valid        ,
`endif
    input  wire                                        cxs_prcl0_rx_crdrtn           ,
    output wire                                        cxs_prcl0_rx_crdgnt           ,
    input  wire                                        cxs_prcl0_rx_activereq        ,
    output wire                                        cxs_prcl0_rx_activeack        ,
    output wire                                        cxs_prcl0_rx_deacthint        ,
`ifdef PRCL_TX_LANE_1
    input  wire [`CXSPRCL_DATAFLITWIDTH-1         : 0] cxs_prcl1_rx_data             ,
    input  wire [`CXSPRCL_CNTL_WIDTH-1            : 0] cxs_prcl1_rx_cntl             ,
    input  wire                                        cxs_prcl1_rx_valid            ,
    input  wire [1                                : 0] cxs_prcl1_rx_prcltype         ,
    input  wire [`CXSPRCL_SRCID_WIDTH-1           : 0] cxs_prcl1_rx_srcid            ,
    input  wire [`CXSPRCL_TGTID_WIDTH-1           : 0] cxs_prcl1_rx_tgtid            ,
`ifdef CXSPRCL_LAST
    input  wire                                        cxs_prcl1_rx_last             ,
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
    input  wire [(`CXSPRCL_DATAFLITWIDTH/8)-1     : 0] cxs_prcl1_rx_chk_data         ,
    input  wire [`CXSPRCL_CNTL_CHK_BYTE-1         : 0] cxs_prcl1_rx_chk_cntl         ,
    input  wire                                        cxs_prcl1_rx_chk_prcltype     ,
    input  wire                                        cxs_prcl1_rx_chk_srcid        ,
    input  wire                                        cxs_prcl1_rx_chk_tgtid        ,
    input  wire                                        cxs_prcl1_rx_chk_last         ,
    input  wire                                        cxs_prcl1_rx_chk_valid        ,
`endif
    input  wire                                        cxs_prcl1_rx_crdrtn           ,
    output wire                                        cxs_prcl1_rx_crdgnt           ,
    input  wire                                        cxs_prcl1_rx_activereq        ,
    output wire                                        cxs_prcl1_rx_activeack        ,
    output wire                                        cxs_prcl1_rx_deacthint        ,
`endif
`ifdef PRCL_TX_LANE_2
    input  wire [`CXSPRCL_DATAFLITWIDTH-1         : 0] cxs_prcl2_rx_data             ,
    input  wire [`CXSPRCL_CNTL_WIDTH-1            : 0] cxs_prcl2_rx_cntl             ,
    input  wire                                        cxs_prcl2_rx_valid            ,
    input  wire [1                                : 0] cxs_prcl2_rx_prcltype         ,
    input  wire [`CXSPRCL_SRCID_WIDTH-1           : 0] cxs_prcl2_rx_srcid            ,
    input  wire [`CXSPRCL_TGTID_WIDTH-1           : 0] cxs_prcl2_rx_tgtid            ,
`ifdef CXSPRCL_LAST
    input  wire                                        cxs_prcl2_rx_last             ,
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
    input  wire [(`CXSPRCL_DATAFLITWIDTH/8)-1     : 0] cxs_prcl2_rx_chk_data         ,
    input  wire [`CXSPRCL_CNTL_CHK_BYTE-1         : 0] cxs_prcl2_rx_chk_cntl         ,
    input  wire                                        cxs_prcl2_rx_chk_prcltype     ,
    input  wire                                        cxs_prcl2_rx_chk_srcid        ,
    input  wire                                        cxs_prcl2_rx_chk_tgtid        ,
    input  wire                                        cxs_prcl2_rx_chk_last         ,
    input  wire                                        cxs_prcl2_rx_chk_valid        ,
`endif
    input  wire                                        cxs_prcl2_rx_crdrtn           ,
    output wire                                        cxs_prcl2_rx_crdgnt           ,
    input  wire                                        cxs_prcl2_rx_activereq        ,
    output wire                                        cxs_prcl2_rx_activeack        ,
    output wire                                        cxs_prcl2_rx_deacthint        ,
`endif
`ifdef PRCL_TX_LANE_3
    input  wire [`CXSPRCL_DATAFLITWIDTH-1         : 0] cxs_prcl3_rx_data             ,
    input  wire [`CXSPRCL_CNTL_WIDTH-1            : 0] cxs_prcl3_rx_cntl             ,
    input  wire                                        cxs_prcl3_rx_valid            ,
    input  wire [1                                : 0] cxs_prcl3_rx_prcltype         ,
    input  wire [`CXSPRCL_SRCID_WIDTH-1           : 0] cxs_prcl3_rx_srcid            ,
    input  wire [`CXSPRCL_TGTID_WIDTH-1           : 0] cxs_prcl3_rx_tgtid            ,
`ifdef CXSPRCL_LAST
    input  wire                                        cxs_prcl3_rx_last             ,
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
    input  wire [(`CXSPRCL_DATAFLITWIDTH/8)-1     : 0] cxs_prcl3_rx_chk_data         ,
    input  wire [`CXSPRCL_CNTL_CHK_BYTE-1         : 0] cxs_prcl3_rx_chk_cntl         ,
    input  wire                                        cxs_prcl3_rx_chk_prcltype     ,
    input  wire                                        cxs_prcl3_rx_chk_srcid        ,
    input  wire                                        cxs_prcl3_rx_chk_tgtid        ,
    input  wire                                        cxs_prcl3_rx_chk_last         ,
    input  wire                                        cxs_prcl3_rx_chk_valid        ,
`endif
    input  wire                                        cxs_prcl3_rx_crdrtn           ,
    output wire                                        cxs_prcl3_rx_crdgnt           ,
    input  wire                                        cxs_prcl3_rx_activereq        ,
    output wire                                        cxs_prcl3_rx_activeack        ,
    output wire                                        cxs_prcl3_rx_deacthint        ,
`endif

    // ==========================================================================
    //  CXS TX 方向 (对外, RX lane → cxs_prcl{n}_tx_*, 协议层→上游)
    // ==========================================================================
    output wire [`CXSPRCL_DATAFLITWIDTH-1         : 0] cxs_prcl0_tx_data             ,
    output wire [`CXSPRCL_CNTL_WIDTH-1            : 0] cxs_prcl0_tx_cntl             ,
    output wire                                        cxs_prcl0_tx_valid            ,
    output wire [1                                : 0] cxs_prcl0_tx_prcltype         ,
    output wire [`CXSPRCL_SRCID_WIDTH-1           : 0] cxs_prcl0_tx_srcid            ,
    output wire [`CXSPRCL_TGTID_WIDTH-1           : 0] cxs_prcl0_tx_tgtid            ,
`ifdef CXSPRCL_LAST
    output wire                                        cxs_prcl0_tx_last             ,
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
    output wire [(`CXSPRCL_DATAFLITWIDTH/8)-1     : 0] cxs_prcl0_tx_chk_data         ,
    output wire [`CXSPRCL_CNTL_CHK_BYTE-1         : 0] cxs_prcl0_tx_chk_cntl         ,
    output wire                                        cxs_prcl0_tx_chk_prcltype     ,
    output wire                                        cxs_prcl0_tx_chk_srcid        ,
    output wire                                        cxs_prcl0_tx_chk_tgtid        ,
    output wire                                        cxs_prcl0_tx_chk_last         ,
    output wire                                        cxs_prcl0_tx_chk_valid        ,
`endif
    output wire                                        cxs_prcl0_tx_crdrtn           ,
    input  wire                                        cxs_prcl0_tx_crdgnt           ,
    output wire                                        cxs_prcl0_tx_activereq        ,
    input  wire                                        cxs_prcl0_tx_activeack        ,
    input  wire                                        cxs_prcl0_tx_deacthint        ,
`ifdef PRCL_RX_LANE_1
    output wire [`CXSPRCL_DATAFLITWIDTH-1         : 0] cxs_prcl1_tx_data             ,
    output wire [`CXSPRCL_CNTL_WIDTH-1            : 0] cxs_prcl1_tx_cntl             ,
    output wire                                        cxs_prcl1_tx_valid            ,
    output wire [1                                : 0] cxs_prcl1_tx_prcltype         ,
    output wire [`CXSPRCL_SRCID_WIDTH-1           : 0] cxs_prcl1_tx_srcid            ,
    output wire [`CXSPRCL_TGTID_WIDTH-1           : 0] cxs_prcl1_tx_tgtid            ,
`ifdef CXSPRCL_LAST
    output wire                                        cxs_prcl1_tx_last             ,
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
    output wire [(`CXSPRCL_DATAFLITWIDTH/8)-1     : 0] cxs_prcl1_tx_chk_data         ,
    output wire [`CXSPRCL_CNTL_CHK_BYTE-1         : 0] cxs_prcl1_tx_chk_cntl         ,
    output wire                                        cxs_prcl1_tx_chk_prcltype     ,
    output wire                                        cxs_prcl1_tx_chk_srcid        ,
    output wire                                        cxs_prcl1_tx_chk_tgtid        ,
    output wire                                        cxs_prcl1_tx_chk_last         ,
    output wire                                        cxs_prcl1_tx_chk_valid        ,
`endif
    output wire                                        cxs_prcl1_tx_crdrtn           ,
    input  wire                                        cxs_prcl1_tx_crdgnt           ,
    output wire                                        cxs_prcl1_tx_activereq        ,
    input  wire                                        cxs_prcl1_tx_activeack        ,
    input  wire                                        cxs_prcl1_tx_deacthint        ,
`endif
`ifdef PRCL_RX_LANE_2
    output wire [`CXSPRCL_DATAFLITWIDTH-1         : 0] cxs_prcl2_tx_data             ,
    output wire [`CXSPRCL_CNTL_WIDTH-1            : 0] cxs_prcl2_tx_cntl             ,
    output wire                                        cxs_prcl2_tx_valid            ,
    output wire [1                                : 0] cxs_prcl2_tx_prcltype         ,
    output wire [`CXSPRCL_SRCID_WIDTH-1           : 0] cxs_prcl2_tx_srcid            ,
    output wire [`CXSPRCL_TGTID_WIDTH-1           : 0] cxs_prcl2_tx_tgtid            ,
`ifdef CXSPRCL_LAST
    output wire                                        cxs_prcl2_tx_last             ,
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
    output wire [(`CXSPRCL_DATAFLITWIDTH/8)-1     : 0] cxs_prcl2_tx_chk_data         ,
    output wire [`CXSPRCL_CNTL_CHK_BYTE-1         : 0] cxs_prcl2_tx_chk_cntl         ,
    output wire                                        cxs_prcl2_tx_chk_prcltype     ,
    output wire                                        cxs_prcl2_tx_chk_srcid        ,
    output wire                                        cxs_prcl2_tx_chk_tgtid        ,
    output wire                                        cxs_prcl2_tx_chk_last         ,
    output wire                                        cxs_prcl2_tx_chk_valid        ,
`endif
    output wire                                        cxs_prcl2_tx_crdrtn           ,
    input  wire                                        cxs_prcl2_tx_crdgnt           ,
    output wire                                        cxs_prcl2_tx_activereq        ,
    input  wire                                        cxs_prcl2_tx_activeack        ,
    input  wire                                        cxs_prcl2_tx_deacthint        ,
`endif
`ifdef PRCL_RX_LANE_3
    output wire [`CXSPRCL_DATAFLITWIDTH-1         : 0] cxs_prcl3_tx_data             ,
    output wire [`CXSPRCL_CNTL_WIDTH-1            : 0] cxs_prcl3_tx_cntl             ,
    output wire                                        cxs_prcl3_tx_valid            ,
    output wire [1                                : 0] cxs_prcl3_tx_prcltype         ,
    output wire [`CXSPRCL_SRCID_WIDTH-1           : 0] cxs_prcl3_tx_srcid            ,
    output wire [`CXSPRCL_TGTID_WIDTH-1           : 0] cxs_prcl3_tx_tgtid            ,
`ifdef CXSPRCL_LAST
    output wire                                        cxs_prcl3_tx_last             ,
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
    output wire [(`CXSPRCL_DATAFLITWIDTH/8)-1     : 0] cxs_prcl3_tx_chk_data         ,
    output wire [`CXSPRCL_CNTL_CHK_BYTE-1         : 0] cxs_prcl3_tx_chk_cntl         ,
    output wire                                        cxs_prcl3_tx_chk_prcltype     ,
    output wire                                        cxs_prcl3_tx_chk_srcid        ,
    output wire                                        cxs_prcl3_tx_chk_tgtid        ,
    output wire                                        cxs_prcl3_tx_chk_last         ,
    output wire                                        cxs_prcl3_tx_chk_valid        ,
`endif
    output wire                                        cxs_prcl3_tx_crdrtn           ,
    input  wire                                        cxs_prcl3_tx_crdgnt           ,
    output wire                                        cxs_prcl3_tx_activereq        ,
    input  wire                                        cxs_prcl3_tx_activeack        ,
    input  wire                                        cxs_prcl3_tx_deacthint        ,
`endif
`endif

    // ==========================================================================
    //  CPIF interface (对外, link_layer 输出, 按 link × lane)
    //  link 数由 TX_LANE_*/RX_LANE_* 控制；每 link 下 lane 数由 LL_LINK_LANE* 控制
    // ==========================================================================
    output wire [`CPIF_DATA_WIDTH-1               : 0] cpif_tx_link0_lane0_data_o    ,
    output wire                                        cpif_tx_link0_lane0_data_vld_o,
`ifdef LL_LINK_LANE1
    output wire [`CPIF_DATA_WIDTH-1               : 0] cpif_tx_link0_lane1_data_o    ,
    output wire                                        cpif_tx_link0_lane1_data_vld_o,
`endif
`ifdef LL_LINK_LANE3
    output wire [`CPIF_DATA_WIDTH-1               : 0] cpif_tx_link0_lane2_data_o    ,
    output wire                                        cpif_tx_link0_lane2_data_vld_o,
    output wire [`CPIF_DATA_WIDTH-1               : 0] cpif_tx_link0_lane3_data_o    ,
    output wire                                        cpif_tx_link0_lane3_data_vld_o,
`endif
    input  wire                                        cpif_tx_link0_ready_i         ,
    input  wire                                        cpif_rx_link0_ready_i         ,
    input  wire [`CPIF_DATA_WIDTH-1               : 0] cpif_rx_link0_lane0_data_i    ,
    input  wire                                        cpif_rx_link0_lane0_data_vld_i,
`ifdef LL_LINK_LANE1
    input  wire [`CPIF_DATA_WIDTH-1               : 0] cpif_rx_link0_lane1_data_i    ,
    input  wire                                        cpif_rx_link0_lane1_data_vld_i,
`endif
`ifdef LL_LINK_LANE3
    input  wire [`CPIF_DATA_WIDTH-1               : 0] cpif_rx_link0_lane2_data_i    ,
    input  wire                                        cpif_rx_link0_lane2_data_vld_i,
    input  wire [`CPIF_DATA_WIDTH-1               : 0] cpif_rx_link0_lane3_data_i    ,
    input  wire                                        cpif_rx_link0_lane3_data_vld_i,
`endif
`ifdef PRCL_TX_LANE_1
    output wire [`CPIF_DATA_WIDTH-1               : 0] cpif_tx_link1_lane0_data_o    ,
    output wire                                        cpif_tx_link1_lane0_data_vld_o,
`ifdef LL_LINK_LANE1
    output wire [`CPIF_DATA_WIDTH-1               : 0] cpif_tx_link1_lane1_data_o    ,
    output wire                                        cpif_tx_link1_lane1_data_vld_o,
`endif
`ifdef LL_LINK_LANE3
    output wire [`CPIF_DATA_WIDTH-1               : 0] cpif_tx_link1_lane2_data_o    ,
    output wire                                        cpif_tx_link1_lane2_data_vld_o,
    output wire [`CPIF_DATA_WIDTH-1               : 0] cpif_tx_link1_lane3_data_o    ,
    output wire                                        cpif_tx_link1_lane3_data_vld_o,
`endif
    input  wire                                        cpif_tx_link1_ready_i         ,
`endif
`ifdef PRCL_RX_LANE_1
    input  wire [`CPIF_DATA_WIDTH-1               : 0] cpif_rx_link1_lane0_data_i    ,
    input  wire                                        cpif_rx_link1_lane0_data_vld_i,
`ifdef LL_LINK_LANE1
    input  wire [`CPIF_DATA_WIDTH-1               : 0] cpif_rx_link1_lane1_data_i    ,
    input  wire                                        cpif_rx_link1_lane1_data_vld_i,
`endif
`ifdef LL_LINK_LANE3
    input  wire [`CPIF_DATA_WIDTH-1               : 0] cpif_rx_link1_lane2_data_i    ,
    input  wire                                        cpif_rx_link1_lane2_data_vld_i,
    input  wire [`CPIF_DATA_WIDTH-1               : 0] cpif_rx_link1_lane3_data_i    ,
    input  wire                                        cpif_rx_link1_lane3_data_vld_i,
`endif
`endif
`ifdef PRCL_TX_LANE_2
    output wire [`CPIF_DATA_WIDTH-1               : 0] cpif_tx_link2_lane0_data_o    ,
    output wire                                        cpif_tx_link2_lane0_data_vld_o,
`ifdef LL_LINK_LANE1
    output wire [`CPIF_DATA_WIDTH-1               : 0] cpif_tx_link2_lane1_data_o    ,
    output wire                                        cpif_tx_link2_lane1_data_vld_o,
`endif
`ifdef LL_LINK_LANE3
    output wire [`CPIF_DATA_WIDTH-1               : 0] cpif_tx_link2_lane2_data_o    ,
    output wire                                        cpif_tx_link2_lane2_data_vld_o,
    output wire [`CPIF_DATA_WIDTH-1               : 0] cpif_tx_link2_lane3_data_o    ,
    output wire                                        cpif_tx_link2_lane3_data_vld_o,
`endif
    input  wire                                        cpif_tx_link2_ready_i         ,
`endif
`ifdef PRCL_RX_LANE_2
    input  wire [`CPIF_DATA_WIDTH-1               : 0] cpif_rx_link2_lane0_data_i    ,
    input  wire                                        cpif_rx_link2_lane0_data_vld_i,
`ifdef LL_LINK_LANE1
    input  wire [`CPIF_DATA_WIDTH-1               : 0] cpif_rx_link2_lane1_data_i    ,
    input  wire                                        cpif_rx_link2_lane1_data_vld_i,
`endif
`ifdef LL_LINK_LANE3
    input  wire [`CPIF_DATA_WIDTH-1               : 0] cpif_rx_link2_lane2_data_i    ,
    input  wire                                        cpif_rx_link2_lane2_data_vld_i,
    input  wire [`CPIF_DATA_WIDTH-1               : 0] cpif_rx_link2_lane3_data_i    ,
    input  wire                                        cpif_rx_link2_lane3_data_vld_i,
`endif
`endif
`ifdef PRCL_TX_LANE_3
    output wire [`CPIF_DATA_WIDTH-1               : 0] cpif_tx_link3_lane0_data_o    ,
    output wire                                        cpif_tx_link3_lane0_data_vld_o,
`ifdef LL_LINK_LANE1
    output wire [`CPIF_DATA_WIDTH-1               : 0] cpif_tx_link3_lane1_data_o    ,
    output wire                                        cpif_tx_link3_lane1_data_vld_o,
`endif
`ifdef LL_LINK_LANE3
    output wire [`CPIF_DATA_WIDTH-1               : 0] cpif_tx_link3_lane2_data_o    ,
    output wire                                        cpif_tx_link3_lane2_data_vld_o,
    output wire [`CPIF_DATA_WIDTH-1               : 0] cpif_tx_link3_lane3_data_o    ,
    output wire                                        cpif_tx_link3_lane3_data_vld_o,
`endif
    input  wire                                        cpif_tx_link3_ready_i         ,
`endif
`ifdef PRCL_RX_LANE_3
    input  wire [`CPIF_DATA_WIDTH-1               : 0] cpif_rx_link3_lane0_data_i    ,
    input  wire                                        cpif_rx_link3_lane0_data_vld_i,
`ifdef LL_LINK_LANE1
    input  wire [`CPIF_DATA_WIDTH-1               : 0] cpif_rx_link3_lane1_data_i    ,
    input  wire                                        cpif_rx_link3_lane1_data_vld_i,
`endif
`ifdef LL_LINK_LANE3
    input  wire [`CPIF_DATA_WIDTH-1               : 0] cpif_rx_link3_lane2_data_i    ,
    input  wire                                        cpif_rx_link3_lane2_data_vld_i,
    input  wire [`CPIF_DATA_WIDTH-1               : 0] cpif_rx_link3_lane3_data_i    ,
    input  wire                                        cpif_rx_link3_lane3_data_vld_i,
`endif
`endif

    // --- CPIF control (link_layer 输出, 全 link 共享) ---
    output wire [3                                : 0] cpif_tx_mode_o                ,
    output wire                                        cpif_tx_rst_n_o               ,
    output wire                                        cpif_tx_lpc_o                 ,
    input  wire                                        cpif_rx_lpc_i                 ,
    output wire [3                                : 0] cpif_rx_mode_o                ,
    output wire                                        cpif_rx_rst_n_o               ,

    // --- UDI TX / RX data interface (link_layer §2.4, per-lane CPIF_DATA_WIDTH, 按 link × lane 分支) ---
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link0_lane0_data_i     ,
    input  wire                                        udi_tx_link0_lane0_data_vld_i ,
`ifdef LL_LINK_LANE1
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link0_lane1_data_i     ,
    input  wire                                        udi_tx_link0_lane1_data_vld_i ,
`endif
`ifdef LL_LINK_LANE3
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link0_lane2_data_i     ,
    input  wire                                        udi_tx_link0_lane2_data_vld_i ,
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link0_lane3_data_i     ,
    input  wire                                        udi_tx_link0_lane3_data_vld_i ,
`endif
    output wire                                        udi_tx_link0_ready_o          ,
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link0_lane0_data_o     ,
    output wire                                        udi_rx_link0_lane0_data_vld_o ,
`ifdef LL_LINK_LANE1
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link0_lane1_data_o     ,
    output wire                                        udi_rx_link0_lane1_data_vld_o ,
`endif
`ifdef LL_LINK_LANE3
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link0_lane2_data_o     ,
    output wire                                        udi_rx_link0_lane2_data_vld_o ,
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link0_lane3_data_o     ,
    output wire                                        udi_rx_link0_lane3_data_vld_o ,
`endif
`ifdef PRCL_TX_LANE_1
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link1_lane0_data_i     ,
    input  wire                                        udi_tx_link1_lane0_data_vld_i ,
`ifdef LL_LINK_LANE1
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link1_lane1_data_i     ,
    input  wire                                        udi_tx_link1_lane1_data_vld_i ,
`endif
`ifdef LL_LINK_LANE3
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link1_lane2_data_i     ,
    input  wire                                        udi_tx_link1_lane2_data_vld_i ,
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link1_lane3_data_i     ,
    input  wire                                        udi_tx_link1_lane3_data_vld_i ,
`endif
    output wire                                        udi_tx_link1_ready_o          ,
`endif
`ifdef PRCL_TX_LANE_2
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link2_lane0_data_i     ,
    input  wire                                        udi_tx_link2_lane0_data_vld_i ,
`ifdef LL_LINK_LANE1
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link2_lane1_data_i     ,
    input  wire                                        udi_tx_link2_lane1_data_vld_i ,
`endif
`ifdef LL_LINK_LANE3
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link2_lane2_data_i     ,
    input  wire                                        udi_tx_link2_lane2_data_vld_i ,
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link2_lane3_data_i     ,
    input  wire                                        udi_tx_link2_lane3_data_vld_i ,
`endif
    output wire                                        udi_tx_link2_ready_o          ,
`endif
`ifdef PRCL_TX_LANE_3
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link3_lane0_data_i     ,
    input  wire                                        udi_tx_link3_lane0_data_vld_i ,
`ifdef LL_LINK_LANE1
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link3_lane1_data_i     ,
    input  wire                                        udi_tx_link3_lane1_data_vld_i ,
`endif
`ifdef LL_LINK_LANE3
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link3_lane2_data_i     ,
    input  wire                                        udi_tx_link3_lane2_data_vld_i ,
    input  wire [`CPIF_DATA_WIDTH-1               : 0] udi_tx_link3_lane3_data_i     ,
    input  wire                                        udi_tx_link3_lane3_data_vld_i ,
`endif
    output wire                                        udi_tx_link3_ready_o          ,
`endif
`ifdef PRCL_RX_LANE_1
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link1_lane0_data_o     ,
    output wire                                        udi_rx_link1_lane0_data_vld_o ,
`ifdef LL_LINK_LANE1
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link1_lane1_data_o     ,
    output wire                                        udi_rx_link1_lane1_data_vld_o ,
`endif
`ifdef LL_LINK_LANE3
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link1_lane2_data_o     ,
    output wire                                        udi_rx_link1_lane2_data_vld_o ,
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link1_lane3_data_o     ,
    output wire                                        udi_rx_link1_lane3_data_vld_o ,
`endif
`endif
`ifdef PRCL_RX_LANE_2
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link2_lane0_data_o     ,
    output wire                                        udi_rx_link2_lane0_data_vld_o ,
`ifdef LL_LINK_LANE1
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link2_lane1_data_o     ,
    output wire                                        udi_rx_link2_lane1_data_vld_o ,
`endif
`ifdef LL_LINK_LANE3
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link2_lane2_data_o     ,
    output wire                                        udi_rx_link2_lane2_data_vld_o ,
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link2_lane3_data_o     ,
    output wire                                        udi_rx_link2_lane3_data_vld_o ,
`endif
`endif
`ifdef PRCL_RX_LANE_3
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link3_lane0_data_o     ,
    output wire                                        udi_rx_link3_lane0_data_vld_o ,
`ifdef LL_LINK_LANE1
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link3_lane1_data_o     ,
    output wire                                        udi_rx_link3_lane1_data_vld_o ,
`endif
`ifdef LL_LINK_LANE3
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link3_lane2_data_o     ,
    output wire                                        udi_rx_link3_lane2_data_vld_o ,
    output wire [`CPIF_DATA_WIDTH-1               : 0] udi_rx_link3_lane3_data_o     ,
    output wire                                        udi_rx_link3_lane3_data_vld_o ,
`endif
`endif

    // --- Event TX / RX interface (link_layer, 全 lane 共享) ---
    output wire                                        cpif_tx_event_enable_o        ,
    output wire                                        cpif_tx_event_req_o           ,
    output wire [15                               : 0] cpif_tx_event_cmd_o           ,
    input  wire                                        cpif_tx_event_ack_i           ,
    input  wire [7                                : 0] cpif_tx_event_status_i        ,
    output wire                                        cpif_rx_event_enable_o        ,
    input  wire                                        cpif_rx_event_req_i           ,
    input  wire [15                               : 0] cpif_rx_event_cmd_i           ,
    output wire                                        cpif_rx_event_ack_o           ,
    output wire [7                                : 0] cpif_rx_event_status_o        ,

    // --- DFx status (link_layer) ---
    output wire [31                               : 0] dfx_stat_o
);

    // ==========================================================================
    //  内部信号定义
    // ==========================================================================
    // --- presetn 同步释放 ---
    wire       presetn_sync;
`ifdef PRCL_AXI
    // --- aresetn 同步释放（aclk 域，供 AXI 协议层 aresetn）---
    wire       aresetn_sync;
`endif
    // 分配至协议层 / link 的复位
    wire       prcl_mb_rstn ; // = pcsr.mb_rstn（协议层 prcl_clk 域；iodie 内部 u_rstn_sync 再同步）
    wire       link_mb_rstn ; // = mb_rstn_sync（pclk→mb_clk 复位跨域同步释放后 → link_layer.mb_rst_n）
    wire       link_apb_rstn; // = presetn_sync

    // --- APB 仲裁 ---
    wire [1 : 0] apb_sel       ; // paddr[11:10]
    wire         apb_pcsr_hit  ;
    wire         apb_prcl_hit  ;
    wire         apb_link_hit  ;
    // 三路从设备 APB 回读
    wire          pcsr_pready ;
    wire [31 : 0] pcsr_prdata ;
    wire          pcsr_pslverr;
    wire          prcl_pready ;
    wire [31 : 0] prcl_prdata ;
    wire          prcl_pslverr;
    wire          link_pready ;
    wire [31 : 0] link_prdata ;
    wire          link_pslverr;
    // --- link 段内地址（段基址 0xC00 在 top 剥离，link 内部按段内偏移译码）---
    //   位宽保持 16 与 link_layer.paddr_i 一致（W110 端口位宽匹配），高 4 位恒 0
    wire [15 : 0] link_paddr ; // = {4'h0, paddr[11:0]} - 16'h0C00 → 段内偏移 0x000-0x3FF

    // --- 中断（协议层中断同步至 pclk，与 link 1-bit 中断聚合后寄存器输出）---
    wire         prcl_intr     ; // 协议层中断（AXI: intr 组合输出 / CXS: cxs_prcl_intr reg，电平保持型）
    wire         prcl_intr_sync; // 协议层中断同步至 pclk 域（sync_cell 输出）
    wire         link_irq      ; // 链路层中断（1-bit）

    // --- pcsr 输出复位 ---
    wire       mb_rstn_pcsr; // pcsr.prtcl_rstn
    wire       mb_rstn_sync; // pcsr.mb_rstn 经 mb_clk 域 rstn_sync 同步释放后 → link_layer.mb_rst_n
    wire       sb_rstn_pcsr; // pcsr.sb_rstn
    wire       sb_rstn_sync; // pcsr.sb_rstn 经 sb_clk 域 rstn_sync 同步释放后 → link_layer.sb_rst_n
    // --- pcsr 输出时钟使能 / credit（→ AXI 协议层）---
    wire [7 : 0] tx_clken ; // pcsr.clk_en.tx_clken
    wire [7 : 0] rx_clken ; // pcsr.clk_en.rx_clken
    // --- AXI dual-mode 方向 clken 拆分（16bit → mst 低 4bit / slv 高 4bit）---
    //   req/resp_rx clken 拆分仅 AXI dual-mode(non-iodie) 消费（axi4_asymm_prcl）；iodie 无消费→ batch-c 收窄
`ifndef AXI_IO_DIE
    wire [3 : 0] req_tx_clken  ; // = tx_clken[3:0]，master 方向 tx lane clken
    wire [3 : 0] req_rx_clken  ; // = rx_clken[3:0]，master 方向 rx lane clken
    wire [3 : 0] resp_rx_clken ; // = rx_clken[7:4]，slave 方向 rx lane clken
`endif
    // --- cmptdie 协议层 clken（4R1T：lane0 对称骨干 symm + lane1~3 RX，取 pcsr master 侧 nibble）---
    //   symm_clk_en(1bit) 门控 lane0 对称骨干，放 req_tx_clken 最低位；
    //   cmpt_req_rx_clken[2:0] 门控 lane1~3 RX，放 req_rx_clken 高 3 位
    `ifdef AXI_CMPT_DIE
    wire          cmpt_symm_clk_en  ; // = req_tx_clken[0]  = tx_clken[0]
    wire [2 : 0]  cmpt_req_rx_clken ; // = req_rx_clken[3:1] = rx_clken[3:1]
    `endif
    wire [3 : 0] resp_tx_clken ; // = tx_clken[7:4]，slave 方向 tx lane clken
    // --- iodie 协议层 clken（4 个，取 pcsr 原 resp_tx_clken = tx_clken[7:4]）---
    //   symm_clk_en(1bit)门控 lane0 对称骨憨，放最低位；iodie_resp_tx_clken[2:0]门控 lane1~3 TX，放高 3 位
    wire          iodie_symm_clk_en   ; // = resp_tx_clken[0] = tx_clken[4]
    wire [2 : 0]  iodie_resp_tx_clken ; // = resp_tx_clken[3:1] = tx_clken[7:5]

    // --- 协议层 PAIF <-> link PAIF 中转 (lane0 + TX/RX_LANE 宏分支, AXI/CXS 共用) ---
    wire [`PAIF_DATA_WIDTH-1 : 0] paif0_txdata ;
    wire                          paif0_txvalid;
    wire                          paif0_txready;
    wire [`PAIF_DATA_WIDTH-1 : 0] paif0_rxdata ;
    wire                          paif0_rxvalid;
    wire                          paif0_rxready;
    // --- paif_clk / paif_rstn 输出 → link_layer paif 时钟/复位输入 ---
    wire       paif_clk ;
    wire       paif_rstn;
`ifdef PRCL_TX_LANE_1
    wire [`PAIF_DATA_WIDTH-1 : 0] paif1_txdata ;
    wire                          paif1_txvalid;
    wire                          paif1_txready;
`endif
`ifdef PRCL_TX_LANE_2
    wire [`PAIF_DATA_WIDTH-1 : 0] paif2_txdata ;
    wire                          paif2_txvalid;
    wire                          paif2_txready;
`endif
`ifdef PRCL_TX_LANE_3
    wire [`PAIF_DATA_WIDTH-1 : 0] paif3_txdata ;
    wire                          paif3_txvalid;
    wire                          paif3_txready;
`endif
`ifdef PRCL_RX_LANE_1
    wire [`PAIF_DATA_WIDTH-1 : 0] paif1_rxdata ;
    wire                          paif1_rxvalid;
    wire                          paif1_rxready;
`endif
`ifdef PRCL_RX_LANE_2
    wire [`PAIF_DATA_WIDTH-1 : 0] paif2_rxdata ;
    wire                          paif2_rxvalid;
    wire                          paif2_rxready;
`endif
`ifdef PRCL_RX_LANE_3
    wire [`PAIF_DATA_WIDTH-1 : 0] paif3_rxdata ;
    wire                          paif3_rxvalid;
    wire                          paif3_rxready;
`endif

    // --- 协议层 lp_sw <-> link lp_sw 直连 (AXI/CXS 共用) ---
    wire [3 : 0] lp_sw_cmd_prcl;
    wire         lp_sw_req_prcl;
    // CDC 修复(Ac_unsync01/02 336B/336C)：link_layer 产出 lp_sw_ack/lp_sw_sts 为 sb_clk
    // （link_event_process）域，而 lp_ctrl_ack/lp_ctrl_sts 经 prcl 透传、io.sdc 定域
    // mb_clk(prcl_clk)。原直连无边界同步 → 补 sb→mb 2FF 同步。
    //   同步器实现选 sync_cell_norst（无复位 2FF，与 link_layer 全设计一致）：
    //   - 同步复位多级同步器 → SpyGlass 判 "Sync reset used in multi-flop synchronizer"（3388/3389）；
    //   - 异步复位源 prcl_mb_rstn 来自 pcsr 未约束时钟 → 判 Reset_sync02（422F）；
    //   - ack/sts 为电平，仅 PAIF 发命令（初始化后）才有意义，复位后 2 拍 X 无影响。
    //   源端 lp_sw_ack_o 已寄存（link_event_process lp_sw_ack_r），无组合 glitch（4243 已清）。
    wire         lp_sw_ack_sb;      // link_layer.lp_sw_ack_o（sb 域，已寄存）
    wire [3 : 0] lp_sw_sts_sb;      // link_layer.lp_sw_sts_o（sb 域，寄存输出）
    wire [3 : 0] lp_sw_sts_prcl;
    wire         lp_sw_ack_prcl;
    sync_cell_norst u_sync_ack_sb2mb (.clk(mb_clk), .in(lp_sw_ack_sb), .syncout(lp_sw_ack_prcl));
    sync_cell_norst #(.width(4)) u_sync_sts_sb2mb (.clk(mb_clk), .in(lp_sw_sts_sb), .syncout(lp_sw_sts_prcl));


    // ==========================================================================
    //  AXI dual-mode 方向 clken 拆分（16bit → mst 低 4bit / slv 高 4bit）
    // ==========================================================================
`ifndef AXI_IO_DIE
    assign req_tx_clken  = tx_clken[3:0];
    assign req_rx_clken  = rx_clken[3:0];
    assign resp_rx_clken = rx_clken[7:4];
`endif
    // --- cmptdie clken 拆分：symm(lane0 骨干) 取 master tx nibble 最低位，lane1~3 RX 取 master rx nibble 高 3 位 ---
    `ifdef AXI_CMPT_DIE
    assign cmpt_symm_clk_en  = req_tx_clken[0];
    assign cmpt_req_rx_clken = req_rx_clken[3:1];
    `endif
    assign resp_tx_clken = tx_clken[7:4];
    // --- iodie clken 拆分：asym(symm_clk_en) 放最低位，剩余 clken 放高 3 位 ---
    assign iodie_symm_clk_en   = resp_tx_clken[0];   // = tx_clken[4]
    assign iodie_resp_tx_clken = resp_tx_clken[3:1]; // = tx_clken[7:5]

    // ==========================================================================
    //  presetn 同步释放 (top 内 rstn_sync)
    // ==========================================================================
    rstn_sync #(
    .RESET_SYNC_STAGE (3           )
    ) u_rst_sync_presetn (
    .clk              (pclk        ),
    .test_mode        (1'b0        ),
    .test_resetn_in   (1'b1        ),
    .resetn_in        (presetn     ),
    .resetn_out       (presetn_sync)
    );

    // ==========================================================================
    //  aresetn 同步释放 (top 内 rstn_sync, 仅 PRCL_AXI 分支, aclk 域)
    //  对 top 输入 aresetn 做异步置位/同步释放后，供 AXI 协议层 aresetn 端口使用
    // ==========================================================================
`ifdef PRCL_AXI
    rstn_sync #(
    .RESET_SYNC_STAGE (3           )
    ) u_rst_sync_aresetn (
    .clk              (aclk        ),
    .test_mode        (1'b0        ),
    .test_resetn_in   (1'b1        ),
    .resetn_in        (aresetn     ),
    .resetn_out       (aresetn_sync)
    );
`endif

    // ==========================================================================
    //  mb_rstn 同步释放 (top 内 rstn_sync, mb_clk 域)
    //  pcsr 输出的 mb_rstn（pclk 域寄存器输出）经异步置位/同步释放后，
    //  供 link_layer.mb_rst_n 使用（对齐 sb_rstn 的 u_rst_sync_sb_rstn 处理，
    //  清除 CDC Reset_sync02：link_mb_rstn 来自 pclk 域 CSR 寄存器、直连 mb_clk 域
    //  sync_icg_pos.i_clk_en 复位被报「generated from unconstrained clock」）
    // ==========================================================================
    rstn_sync #(
    .RESET_SYNC_STAGE (3           )
    ) u_rst_sync_mb_rstn (
    .clk              (mb_clk      ),
    .test_mode        (1'b0        ),
    .test_resetn_in   (1'b1        ),
    .resetn_in        (mb_rstn_pcsr),
    .resetn_out       (mb_rstn_sync)
    );

    // ==========================================================================
    //  sb_rstn 同步释放 (top 内 rstn_sync, sb_clk 域)
    //  pcsr 输出的 sb_rstn（pclk 域寄存器输出）经异步置位/同步释放后，
    //  供 link_layer.sb_rst_n 使用
    // ==========================================================================
    rstn_sync #(
    .RESET_SYNC_STAGE (3           )
    ) u_rst_sync_sb_rstn (
    .clk              (sb_clk      ),
    .test_mode        (1'b0        ),
    .test_resetn_in   (1'b1        ),
    .resetn_in        (sb_rstn_pcsr),
    .resetn_out       (sb_rstn_sync)
    );

    // ==========================================================================
    //  APB 段选择 (paddr[11:10]: 00=pcsr, 01/10=协议层, 11=link)
    //  0x000-0x3ff → pcsr；0x400-0xbff → 协议层（AXI/CXS 共用）；0xc00-0xfff → link
    // ==========================================================================
    assign apb_sel      = paddr[11:10]      ;
    assign apb_pcsr_hit = (apb_sel == 2'b00);
    assign apb_prcl_hit = (apb_sel == 2'b01) | (apb_sel == 2'b10);
    assign apb_link_hit = (apb_sel == 2'b11);

    // 三路从设备 psel：仅在命中且 psel 有效时选通
    wire psel_to_pcsr  = psel & apb_pcsr_hit;
    wire psel_to_prcl  = psel & apb_prcl_hit;
    wire psel_to_link  = psel & apb_link_hit;

    // link 段基址 0xC00 由 top 剥离：link_layer 内部（apb_csr 的 pslverr 上限判定、
    //   link_layer_rf 的寄存器分支地址如 mode_reg=12'h00C）全部按段内偏移译码。
    //   若直送绝对地址，进 RF 的是 0xC0C 而非 0x00C → 读写分支永不命中（读恒 0、
    //   写丢失），且 pslverr 的段内上限 0x00A0 比对绝对地址 → pslverr 恒置。
    assign link_paddr = {4'h0, paddr[11:0]} - 16'h0C00;

    // ==========================================================================
    // APB3 完成拍输出（ARM IHI 0024C）：PREADY 拉高的同一拍 PRDATA/PSLVERR 必须已有效。
    //   三者全部组合：段选用当拍 apb_sel（不用晚一拍的 prdata_sel_reg），prdata 直接
    //   取被选从设备的当拍读数据，pslverr 取被选从设备的当拍错误（各从设备侧已自
    //   门控：pcsr=access&!addr_implemented / iodie·cmptdie 恒 0 / link=pready&地址越界）。
    // ==========================================================================
    // 错误响应段选（组合，与 pready 同拍有效）
    wire ctrl_pslverr = (apb_pcsr_hit & pcsr_pslverr) |
                        (apb_prcl_hit & prcl_pslverr) |
                        (apb_link_hit & link_pslverr);

    // 传输完成判定（组合，ACCESS 拍内拉高 pready）：正常完成（任一从设备 pready）
    // 与错误完成（pslverr）都要在同一拍给 pready —— APB3 规定错误响应同样以
    // PREADY 结束 ACCESS（ARM IHI 0024C §1.2.2）；否则未映射地址访问永不完成
    // （pcsr 段仅 0x00/0x04 实现，其余偏移只报 pslverr，必须伴随 pready）。
    wire ctrl_xact_complete = psel & penable &
                              (pcsr_pready | prcl_pready | link_pready | ctrl_pslverr);

    // 读数据段选（组合，随当拍 apb_sel → 与 pready 同拍有效；未选段/非 ACCESS 拍不串扰）
    assign prdata = (psel & penable) ?
                    (apb_pcsr_hit ? pcsr_prdata :
                     apb_link_hit ? link_prdata :
                                    prcl_prdata) : 32'h0;

    assign pslverr = (psel & penable) & ctrl_pslverr;

    assign pready = ctrl_xact_complete;

    // ==========================================================================
    //  中断 (协议层中断同步至 pclk + link 中断聚合，寄存器输出)
    // ==========================================================================
    //  prcl_intr = AXI 协议层 intr（PRCL_AXI）或 CXS 协议层 cxs_prcl_intr（else）
    //  两分支共用同一 sync_cell，同步至 pclk 域（data_s 必须为寄存器输出）
    sync_cell #(.RST_VALUE(1'b0)) u_prcl_intr_sync (
        .clk_d  (pclk              ),
        .rstn_d (presetn_sync      ),
        .data_s (prcl_intr         ),
        .data_d (prcl_intr_sync    )
    );

    //  同步后的协议层中断 与 链路层中断（1-bit）相或，经 pclk 域寄存器输出
    always @(posedge pclk or negedge presetn_sync) begin
        if (~presetn_sync) begin
            hipi_ctrl_intr <= 1'b0;
        end
        else begin
            hipi_ctrl_intr <= prcl_intr_sync | link_irq;
        end
    end

    // ==========================================================================
    //  时钟 / 复位分发
    // ==========================================================================
    assign prcl_mb_rstn  = mb_rstn_pcsr;
    assign link_mb_rstn  = mb_rstn_sync;   // pclk→mb_clk 复位跨域经 u_rst_sync_mb_rstn 同步释放
    assign link_apb_rstn = presetn_sync;

    // ==========================================================================
    //  模块例化: pcsr
    // ==========================================================================
    pcsr u_pcsr (
        .pclk             (pclk              ),
        .presetn          (presetn_sync      ),
        .psel             (psel_to_pcsr      ),
        .penable          (penable           ),
        .paddr            (paddr[11:0]       ),
        .pwrite           (pwrite            ),
        .pwdata           (pwdata            ),
        .prdata           (pcsr_prdata       ),
        .pready           (pcsr_pready       ),
        .pslverr          (pcsr_pslverr      ),
        .mb_rstn          (mb_rstn_pcsr      ),
        .sb_rstn          (sb_rstn_pcsr      ),
        .tx_clken         (tx_clken          ),
        .rx_clken         (rx_clken          )
    );

    // ==========================================================================
    //  模块例化: 协议层
    //    PRCL_AXI + cmptdie → AXI 协议层 cmptdie_axi4_prcl（4R1T，lane0 骨干 + lane1~3 RX）
    //    PRCL_AXI + iodie   → AXI 协议层 iodie_axi4_prcl（4T1R，lane0 骨干 + lane1~3 TX）
    //    PRCL_AXI + 其他     → AXI 协议层 axi4_asymm_prcl（MASTER_DIE/SLAVE_DIE 决定类型）
    //    else → CXS 协议层 cxs_prcl
    // ==========================================================================
`ifdef PRCL_AXI
`ifdef AXI_CMPT_DIE
    // ==========================================================================
    //  cmptdie 协议层：lane0 对称骨干（s0+m0，paif_tx0/rx0）+ lane1~3 RX（paif_rx1~3，随 MST_RX_LANE_n）
    //  clken：symm_clk_en(最低位，门控 lane0) + req_rx_clken[2:0](高 3 位，门控 lane1~3 RX)
    // ==========================================================================
    cmptdie_axi4_prcl u_cmptdie_axi_prcl (
        .aclk                      (aclk                     ),
        .aresetn                   (aresetn_sync             ),
        .prcl_clk                  (mb_clk                   ),
        .prcl_rstn                 (prcl_mb_rstn             ),
        .pclk                      (pclk                     ),
        .presetn                   (presetn_sync             ),
        .symm_clk_en               (cmpt_symm_clk_en         ),
        .req_rx_clken              (cmpt_req_rx_clken        ),
        .lp_ctrl_cmd               (lp_ctrl_cmd              ),
        .lp_ctrl_req               (lp_ctrl_req              ),
        .lp_ctrl_ack               (lp_ctrl_ack              ),
        .lp_ctrl_sts               (lp_ctrl_sts              ),
        .lp_sw_cmd                 (lp_sw_cmd_prcl           ),
        .lp_sw_req                 (lp_sw_req_prcl           ),
        .lp_sw_ack                 (lp_sw_ack_prcl           ),
        .lp_sw_sts                 (lp_sw_sts_prcl           ),
        .psel                      (psel_to_prcl             ),
        .penable                   (penable                  ),
        .pwrite                    (pwrite                   ),
        .paddr                     (paddr[11:0]              ),
        .pwdata                    (pwdata                   ),
        .prdata                    (prcl_prdata              ),
        .pready                    (prcl_pready              ),
        .pslverr                   (prcl_pslverr             ),
        // ---- lane0 对称骨憨：s0（slave）+ m0（master）----
        .m0_axi_arid               (m0_axi_arid              ),
        .m0_axi_araddr             (m0_axi_araddr            ),
        .m0_axi_arlen              (m0_axi_arlen             ),
        .m0_axi_arsize             (m0_axi_arsize            ),
        .m0_axi_arburst            (m0_axi_arburst           ),
        .m0_axi_arlock             (m0_axi_arlock            ),
        .m0_axi_arcache            (m0_axi_arcache           ),
        .m0_axi_arprot             (m0_axi_arprot            ),
        .m0_axi_arqos              (m0_axi_arqos             ),
        .m0_axi_arregion           (m0_axi_arregion          ),
        .m0_axi_aruser             (m0_axi_aruser            ),
        .m0_axi_arvalid            (m0_axi_arvalid           ),
        .m0_axi_arready            (m0_axi_arready           ),
        .m0_axi_rid                (m0_axi_rid               ),
        .m0_axi_rdata              (m0_axi_rdata             ),
        .m0_axi_rresp              (m0_axi_rresp             ),
        .m0_axi_ruser              (m0_axi_ruser             ),
        .m0_axi_rlast              (m0_axi_rlast             ),
        .m0_axi_rvalid             (m0_axi_rvalid            ),
        .m0_axi_rready             (m0_axi_rready            ),
        .m0_axi_awid               (m0_axi_awid              ),
        .m0_axi_awaddr             (m0_axi_awaddr            ),
        .m0_axi_awlen              (m0_axi_awlen             ),
        .m0_axi_awsize             (m0_axi_awsize            ),
        .m0_axi_awburst            (m0_axi_awburst           ),
        .m0_axi_awlock             (m0_axi_awlock            ),
        .m0_axi_awcache            (m0_axi_awcache           ),
        .m0_axi_awprot             (m0_axi_awprot            ),
        .m0_axi_awqos              (m0_axi_awqos             ),
        .m0_axi_awregion           (m0_axi_awregion          ),
        .m0_axi_awuser             (m0_axi_awuser            ),
        .m0_axi_awvalid            (m0_axi_awvalid           ),
        .m0_axi_awready            (m0_axi_awready           ),
        .m0_axi_wdata              (m0_axi_wdata             ),
        .m0_axi_wstrb              (m0_axi_wstrb             ),
        .m0_axi_wuser              (m0_axi_wuser             ),
        .m0_axi_wlast              (m0_axi_wlast             ),
        .m0_axi_wvalid             (m0_axi_wvalid            ),
        .m0_axi_wready             (m0_axi_wready            ),
        .m0_axi_bid                (m0_axi_bid               ),
        .m0_axi_bresp              (m0_axi_bresp             ),
        .m0_axi_buser              (m0_axi_buser             ),
        .m0_axi_bvalid             (m0_axi_bvalid            ),
        .m0_axi_bready             (m0_axi_bready            ),
        .s0_axi_awid               (s0_axi_awid              ),
        .s0_axi_awaddr             (s0_axi_awaddr            ),
        .s0_axi_awlen              (s0_axi_awlen             ),
        .s0_axi_awsize             (s0_axi_awsize            ),
        .s0_axi_awburst            (s0_axi_awburst           ),
        .s0_axi_awlock             (s0_axi_awlock            ),
        .s0_axi_awcache            (s0_axi_awcache           ),
        .s0_axi_awprot             (s0_axi_awprot            ),
        .s0_axi_awqos              (s0_axi_awqos             ),
        .s0_axi_awregion           (s0_axi_awregion          ),
        .s0_axi_awuser             (s0_axi_awuser            ),
        .s0_axi_awvalid            (s0_axi_awvalid           ),
        .s0_axi_awready            (s0_axi_awready           ),
        .s0_axi_arid               (s0_axi_arid              ),
        .s0_axi_araddr             (s0_axi_araddr            ),
        .s0_axi_arlen              (s0_axi_arlen             ),
        .s0_axi_arsize             (s0_axi_arsize            ),
        .s0_axi_arburst            (s0_axi_arburst           ),
        .s0_axi_arlock             (s0_axi_arlock            ),
        .s0_axi_arcache            (s0_axi_arcache           ),
        .s0_axi_arprot             (s0_axi_arprot            ),
        .s0_axi_arqos              (s0_axi_arqos             ),
        .s0_axi_arregion           (s0_axi_arregion          ),
        .s0_axi_aruser             (s0_axi_aruser            ),
        .s0_axi_arvalid            (s0_axi_arvalid           ),
        .s0_axi_arready            (s0_axi_arready           ),
        .s0_axi_wdata              (s0_axi_wdata             ),
        .s0_axi_wstrb              (s0_axi_wstrb             ),
        .s0_axi_wuser              (s0_axi_wuser             ),
        .s0_axi_wlast              (s0_axi_wlast             ),
        .s0_axi_wvalid             (s0_axi_wvalid            ),
        .s0_axi_wready             (s0_axi_wready            ),
        .s0_axi_rid                (s0_axi_rid               ),
        .s0_axi_rdata              (s0_axi_rdata             ),
        .s0_axi_rresp              (s0_axi_rresp             ),
        .s0_axi_ruser              (s0_axi_ruser             ),
        .s0_axi_rlast              (s0_axi_rlast             ),
        .s0_axi_rvalid             (s0_axi_rvalid            ),
        .s0_axi_rready             (s0_axi_rready            ),
        .s0_axi_bid                (s0_axi_bid               ),
        .s0_axi_bresp              (s0_axi_bresp             ),
        .s0_axi_buser              (s0_axi_buser             ),
        .s0_axi_bvalid             (s0_axi_bvalid            ),
        .s0_axi_bready             (s0_axi_bready            ),
        // ---- lane1~3：仅 RX AR 通道（随 MST_RX_LANE_n）----
`ifdef MST_RX_LANE_1
        .s1_axi_arid               (s1_axi_arid              ),
        .s1_axi_araddr             (s1_axi_araddr            ),
        .s1_axi_arlen              (s1_axi_arlen             ),
        .s1_axi_arsize             (s1_axi_arsize            ),
        .s1_axi_arburst            (s1_axi_arburst           ),
        .s1_axi_arlock             (s1_axi_arlock            ),
        .s1_axi_arcache            (s1_axi_arcache           ),
        .s1_axi_arprot             (s1_axi_arprot            ),
        .s1_axi_arqos              (s1_axi_arqos             ),
        .s1_axi_arregion           (s1_axi_arregion          ),
        .s1_axi_aruser             (s1_axi_aruser            ),
        .s1_axi_arvalid            (s1_axi_arvalid           ),
        .s1_axi_arready            (s1_axi_arready           ),
        .s1_axi_rid                (s1_axi_rid               ),
        .s1_axi_rdata              (s1_axi_rdata             ),
        .s1_axi_rresp              (s1_axi_rresp             ),
        .s1_axi_ruser              (s1_axi_ruser             ),
        .s1_axi_rlast              (s1_axi_rlast             ),
        .s1_axi_rvalid             (s1_axi_rvalid            ),
        .s1_axi_rready             (s1_axi_rready            ),

`endif
`ifdef MST_RX_LANE_2
        .s2_axi_arid               (s2_axi_arid              ),
        .s2_axi_araddr             (s2_axi_araddr            ),
        .s2_axi_arlen              (s2_axi_arlen             ),
        .s2_axi_arsize             (s2_axi_arsize            ),
        .s2_axi_arburst            (s2_axi_arburst           ),
        .s2_axi_arlock             (s2_axi_arlock            ),
        .s2_axi_arcache            (s2_axi_arcache           ),
        .s2_axi_arprot             (s2_axi_arprot            ),
        .s2_axi_arqos              (s2_axi_arqos             ),
        .s2_axi_arregion           (s2_axi_arregion          ),
        .s2_axi_aruser             (s2_axi_aruser            ),
        .s2_axi_arvalid            (s2_axi_arvalid           ),
        .s2_axi_arready            (s2_axi_arready           ),
        .s2_axi_rid                (s2_axi_rid               ),
        .s2_axi_rdata              (s2_axi_rdata             ),
        .s2_axi_rresp              (s2_axi_rresp             ),
        .s2_axi_ruser              (s2_axi_ruser             ),
        .s2_axi_rlast              (s2_axi_rlast             ),
        .s2_axi_rvalid             (s2_axi_rvalid            ),
        .s2_axi_rready             (s2_axi_rready            ),

`endif
`ifdef MST_RX_LANE_3
        .s3_axi_arid               (s3_axi_arid              ),
        .s3_axi_araddr             (s3_axi_araddr            ),
        .s3_axi_arlen              (s3_axi_arlen             ),
        .s3_axi_arsize             (s3_axi_arsize            ),
        .s3_axi_arburst            (s3_axi_arburst           ),
        .s3_axi_arlock             (s3_axi_arlock            ),
        .s3_axi_arcache            (s3_axi_arcache           ),
        .s3_axi_arprot             (s3_axi_arprot            ),
        .s3_axi_arqos              (s3_axi_arqos             ),
        .s3_axi_arregion           (s3_axi_arregion          ),
        .s3_axi_aruser             (s3_axi_aruser            ),
        .s3_axi_arvalid            (s3_axi_arvalid           ),
        .s3_axi_arready            (s3_axi_arready           ),
        .s3_axi_rid                (s3_axi_rid               ),
        .s3_axi_rdata              (s3_axi_rdata             ),
        .s3_axi_rresp              (s3_axi_rresp             ),
        .s3_axi_ruser              (s3_axi_ruser             ),
        .s3_axi_rlast              (s3_axi_rlast             ),
        .s3_axi_rvalid             (s3_axi_rvalid            ),
        .s3_axi_rready             (s3_axi_rready            ),

`endif
        // ---- PAIF：lane0（paif0 双向）+ lane1~3 RX（paif1~3 rx，随 MST_RX_LANE_n）----
        .paif_tx0data              (paif0_txdata             ),
        .paif_tx0valid             (paif0_txvalid            ),
        .paif_tx0ready             (paif0_txready            ),
        .paif_rx0data              (paif0_rxdata             ),
        .paif_rx0valid             (paif0_rxvalid            ),
        .paif_rx0ready             (paif0_rxready            ),
`ifdef MST_RX_LANE_1
        .paif_rx1data              (paif1_rxdata             ),
        .paif_rx1valid             (paif1_rxvalid            ),
        .paif_rx1ready             (paif1_rxready            ),
`endif
`ifdef MST_RX_LANE_2
        .paif_rx2data              (paif2_rxdata             ),
        .paif_rx2valid             (paif2_rxvalid            ),
        .paif_rx2ready             (paif2_rxready            ),
`endif
`ifdef MST_RX_LANE_3
        .paif_rx3data              (paif3_rxdata             ),
        .paif_rx3valid             (paif3_rxvalid            ),
        .paif_rx3ready             (paif3_rxready            ),
`endif
        .paif_clk                  (paif_clk                 ),
        .paif_rstn                 (paif_rstn                ),
        .intr                      (prcl_intr                )
    );
`elsif AXI_IO_DIE
    // ==========================================================================
    //  iodie 协议层：lane0 对称骨憨（s0+m0，paif_tx0/rx0）+ lane1~3 TX（paif_tx1~3，随 SLV_TX_LANE_n）
    //  clken：symm_clk_en(最低位，门控 lane0) + resp_tx_clken[2:0](高3位，门控 lane1~3 TX)
    // ==========================================================================
    iodie_axi4_prcl u_iodie_axi_prcl (
        .aclk                      (aclk                     ),
        .aresetn                   (aresetn_sync             ),
        .prcl_clk                  (mb_clk                   ),
        .prcl_rstn                 (prcl_mb_rstn             ),
        .pclk                      (pclk                     ),
        .presetn                   (presetn_sync             ),
        .symm_clk_en               (iodie_symm_clk_en        ),
        .resp_tx_clken             (iodie_resp_tx_clken      ),
        .lp_ctrl_cmd               (lp_ctrl_cmd              ),
        .lp_ctrl_req               (lp_ctrl_req              ),
        .lp_ctrl_ack               (lp_ctrl_ack              ),
        .lp_ctrl_sts               (lp_ctrl_sts              ),
        .lp_sw_cmd                 (lp_sw_cmd_prcl           ),
        .lp_sw_req                 (lp_sw_req_prcl           ),
        .lp_sw_ack                 (lp_sw_ack_prcl           ),
        .lp_sw_sts                 (lp_sw_sts_prcl           ),
        .psel                      (psel_to_prcl             ),
        .penable                   (penable                  ),
        .pwrite                    (pwrite                   ),
        .paddr                     (paddr[11:0]              ),
        .pwdata                    (pwdata                   ),
        .prdata                    (prcl_prdata              ),
        .pready                    (prcl_pready              ),
        .pslverr                   (prcl_pslverr             ),
        // ---- lane0 对称骨憨：s0（slave）+ m0（master）----
        .m0_axi_arid               (m0_axi_arid              ),
        .m0_axi_araddr             (m0_axi_araddr            ),
        .m0_axi_arlen              (m0_axi_arlen             ),
        .m0_axi_arsize             (m0_axi_arsize            ),
        .m0_axi_arburst            (m0_axi_arburst           ),
        .m0_axi_arlock             (m0_axi_arlock            ),
        .m0_axi_arcache            (m0_axi_arcache           ),
        .m0_axi_arprot             (m0_axi_arprot            ),
        .m0_axi_arqos              (m0_axi_arqos             ),
        .m0_axi_arregion           (m0_axi_arregion          ),
        .m0_axi_aruser             (m0_axi_aruser            ),
        .m0_axi_arvalid            (m0_axi_arvalid           ),
        .m0_axi_arready            (m0_axi_arready           ),
        .m0_axi_rid                (m0_axi_rid               ),
        .m0_axi_rdata              (m0_axi_rdata             ),
        .m0_axi_rresp              (m0_axi_rresp             ),
        .m0_axi_ruser              (m0_axi_ruser             ),
        .m0_axi_rlast              (m0_axi_rlast             ),
        .m0_axi_rvalid             (m0_axi_rvalid            ),
        .m0_axi_rready             (m0_axi_rready            ),
        .m0_axi_awid               (m0_axi_awid              ),
        .m0_axi_awaddr             (m0_axi_awaddr            ),
        .m0_axi_awlen              (m0_axi_awlen             ),
        .m0_axi_awsize             (m0_axi_awsize            ),
        .m0_axi_awburst            (m0_axi_awburst           ),
        .m0_axi_awlock             (m0_axi_awlock            ),
        .m0_axi_awcache            (m0_axi_awcache           ),
        .m0_axi_awprot             (m0_axi_awprot            ),
        .m0_axi_awqos              (m0_axi_awqos             ),
        .m0_axi_awregion           (m0_axi_awregion          ),
        .m0_axi_awuser             (m0_axi_awuser            ),
        .m0_axi_awvalid            (m0_axi_awvalid           ),
        .m0_axi_awready            (m0_axi_awready           ),
        .m0_axi_wdata              (m0_axi_wdata             ),
        .m0_axi_wstrb              (m0_axi_wstrb             ),
        .m0_axi_wuser              (m0_axi_wuser             ),
        .m0_axi_wlast              (m0_axi_wlast             ),
        .m0_axi_wvalid             (m0_axi_wvalid            ),
        .m0_axi_wready             (m0_axi_wready            ),
        .m0_axi_bid                (m0_axi_bid               ),
        .m0_axi_bresp              (m0_axi_bresp             ),
        .m0_axi_buser              (m0_axi_buser             ),
        .m0_axi_bvalid             (m0_axi_bvalid            ),
        .m0_axi_bready             (m0_axi_bready            ),
        .s0_axi_awid               (s0_axi_awid              ),
        .s0_axi_awaddr             (s0_axi_awaddr            ),
        .s0_axi_awlen              (s0_axi_awlen             ),
        .s0_axi_awsize             (s0_axi_awsize            ),
        .s0_axi_awburst            (s0_axi_awburst           ),
        .s0_axi_awlock             (s0_axi_awlock            ),
        .s0_axi_awcache            (s0_axi_awcache           ),
        .s0_axi_awprot             (s0_axi_awprot            ),
        .s0_axi_awqos              (s0_axi_awqos             ),
        .s0_axi_awregion           (s0_axi_awregion          ),
        .s0_axi_awuser             (s0_axi_awuser            ),
        .s0_axi_awvalid            (s0_axi_awvalid           ),
        .s0_axi_awready            (s0_axi_awready           ),
        .s0_axi_arid               (s0_axi_arid              ),
        .s0_axi_araddr             (s0_axi_araddr            ),
        .s0_axi_arlen              (s0_axi_arlen             ),
        .s0_axi_arsize             (s0_axi_arsize            ),
        .s0_axi_arburst            (s0_axi_arburst           ),
        .s0_axi_arlock             (s0_axi_arlock            ),
        .s0_axi_arcache            (s0_axi_arcache           ),
        .s0_axi_arprot             (s0_axi_arprot            ),
        .s0_axi_arqos              (s0_axi_arqos             ),
        .s0_axi_arregion           (s0_axi_arregion          ),
        .s0_axi_aruser             (s0_axi_aruser            ),
        .s0_axi_arvalid            (s0_axi_arvalid           ),
        .s0_axi_arready            (s0_axi_arready           ),
        .s0_axi_wdata              (s0_axi_wdata             ),
        .s0_axi_wstrb              (s0_axi_wstrb             ),
        .s0_axi_wuser              (s0_axi_wuser             ),
        .s0_axi_wlast              (s0_axi_wlast             ),
        .s0_axi_wvalid             (s0_axi_wvalid            ),
        .s0_axi_wready             (s0_axi_wready            ),
        .s0_axi_rid                (s0_axi_rid               ),
        .s0_axi_rdata              (s0_axi_rdata             ),
        .s0_axi_rresp              (s0_axi_rresp             ),
        .s0_axi_ruser              (s0_axi_ruser             ),
        .s0_axi_rlast              (s0_axi_rlast             ),
        .s0_axi_rvalid             (s0_axi_rvalid            ),
        .s0_axi_rready             (s0_axi_rready            ),
        .s0_axi_bid                (s0_axi_bid               ),
        .s0_axi_bresp              (s0_axi_bresp             ),
        .s0_axi_buser              (s0_axi_buser             ),
        .s0_axi_bvalid             (s0_axi_bvalid            ),
        .s0_axi_bready             (s0_axi_bready            ),
        // ---- lane1~3：仅 TX R 通道（随 SLV_TX_LANE_n）----
`ifdef SLV_TX_LANE_1
        .m1_axi_arid               (m1_axi_arid              ),
        .m1_axi_araddr             (m1_axi_araddr            ),
        .m1_axi_arlen              (m1_axi_arlen             ),
        .m1_axi_arsize             (m1_axi_arsize            ),
        .m1_axi_arburst            (m1_axi_arburst           ),
        .m1_axi_arlock             (m1_axi_arlock            ),
        .m1_axi_arcache            (m1_axi_arcache           ),
        .m1_axi_arprot             (m1_axi_arprot            ),
        .m1_axi_arqos              (m1_axi_arqos             ),
        .m1_axi_arregion           (m1_axi_arregion          ),
        .m1_axi_aruser             (m1_axi_aruser            ),
        .m1_axi_arvalid            (m1_axi_arvalid           ),
        .m1_axi_arready            (m1_axi_arready           ),
        .m1_axi_rid                (m1_axi_rid               ),
        .m1_axi_rdata              (m1_axi_rdata             ),
        .m1_axi_rresp              (m1_axi_rresp             ),
        .m1_axi_ruser              (m1_axi_ruser             ),
        .m1_axi_rlast              (m1_axi_rlast             ),
        .m1_axi_rvalid             (m1_axi_rvalid            ),
        .m1_axi_rready             (m1_axi_rready            ),
`endif
`ifdef SLV_TX_LANE_2
        .m2_axi_arid               (m2_axi_arid              ),
        .m2_axi_araddr             (m2_axi_araddr            ),
        .m2_axi_arlen              (m2_axi_arlen             ),
        .m2_axi_arsize             (m2_axi_arsize            ),
        .m2_axi_arburst            (m2_axi_arburst           ),
        .m2_axi_arlock             (m2_axi_arlock            ),
        .m2_axi_arcache            (m2_axi_arcache           ),
        .m2_axi_arprot             (m2_axi_arprot            ),
        .m2_axi_arqos              (m2_axi_arqos             ),
        .m2_axi_arregion           (m2_axi_arregion          ),
        .m2_axi_aruser             (m2_axi_aruser            ),
        .m2_axi_arvalid            (m2_axi_arvalid           ),
        .m2_axi_arready            (m2_axi_arready           ),
        .m2_axi_rid                (m2_axi_rid               ),
        .m2_axi_rdata              (m2_axi_rdata             ),
        .m2_axi_rresp              (m2_axi_rresp             ),
        .m2_axi_ruser              (m2_axi_ruser             ),
        .m2_axi_rlast              (m2_axi_rlast             ),
        .m2_axi_rvalid             (m2_axi_rvalid            ),
        .m2_axi_rready             (m2_axi_rready            ),
`endif
`ifdef SLV_TX_LANE_3
        .m3_axi_arid               (m3_axi_arid              ),
        .m3_axi_araddr             (m3_axi_araddr            ),
        .m3_axi_arlen              (m3_axi_arlen             ),
        .m3_axi_arsize             (m3_axi_arsize            ),
        .m3_axi_arburst            (m3_axi_arburst           ),
        .m3_axi_arlock             (m3_axi_arlock            ),
        .m3_axi_arcache            (m3_axi_arcache           ),
        .m3_axi_arprot             (m3_axi_arprot            ),
        .m3_axi_arqos              (m3_axi_arqos             ),
        .m3_axi_arregion           (m3_axi_arregion          ),
        .m3_axi_aruser             (m3_axi_aruser            ),
        .m3_axi_arvalid            (m3_axi_arvalid           ),
        .m3_axi_arready            (m3_axi_arready           ),
        .m3_axi_rid                (m3_axi_rid               ),
        .m3_axi_rdata              (m3_axi_rdata             ),
        .m3_axi_rresp              (m3_axi_rresp             ),
        .m3_axi_ruser              (m3_axi_ruser             ),
        .m3_axi_rlast              (m3_axi_rlast             ),
        .m3_axi_rvalid             (m3_axi_rvalid            ),
        .m3_axi_rready             (m3_axi_rready            ),
`endif
        // ---- PAIF：lane0（paif0 双向） + lane1~3 TX（paif1~3，随 SLV_TX_LANE_n）----
        .paif_tx0data              (paif0_txdata             ),
        .paif_tx0valid             (paif0_txvalid            ),
        .paif_tx0ready             (paif0_txready            ),
`ifdef SLV_TX_LANE_1
        .paif_tx1data              (paif1_txdata             ),
        .paif_tx1valid             (paif1_txvalid            ),
        .paif_tx1ready             (paif1_txready            ),
`endif
`ifdef SLV_TX_LANE_2
        .paif_tx2data              (paif2_txdata             ),
        .paif_tx2valid             (paif2_txvalid            ),
        .paif_tx2ready             (paif2_txready            ),
`endif
`ifdef SLV_TX_LANE_3
        .paif_tx3data              (paif3_txdata             ),
        .paif_tx3valid             (paif3_txvalid            ),
        .paif_tx3ready             (paif3_txready            ),
`endif
        .paif_rx0data              (paif0_rxdata             ),
        .paif_rx0valid             (paif0_rxvalid            ),
        .paif_rx0ready             (paif0_rxready            ),
        .paif_clk                  (paif_clk                 ),
        .paif_rstn                 (paif_rstn                ),
        .intr                      (prcl_intr                )
    );
`else
`ifdef AXI_ASYM_MODE
    axi4_asymm_prcl u_axi_prcl (
        .aclk                      (aclk                     ),
        .aresetn                   (aresetn_sync             ),
        .prcl_clk                  (mb_clk                   ),
        .prcl_rstn                 (prcl_mb_rstn             ),
        .pclk                      (pclk                     ),
        .presetn                   (presetn_sync             ),
        .lp_ctrl_cmd               (lp_ctrl_cmd              ),
        .lp_ctrl_req               (lp_ctrl_req              ),
        .lp_ctrl_ack               (lp_ctrl_ack              ),
        .lp_ctrl_sts               (lp_ctrl_sts              ),
        .lp_sw_cmd                 (lp_sw_cmd_prcl           ),
        .lp_sw_req                 (lp_sw_req_prcl           ),
        .lp_sw_ack                 (lp_sw_ack_prcl           ),
        .lp_sw_sts                 (lp_sw_sts_prcl           ),
        .psel                      (psel_to_prcl             ),
        .penable                   (penable                  ),
        .pwrite                    (pwrite                   ),
        .paddr                     (paddr[11:0]              ),
        .pwdata                    (pwdata                   ),
        .prdata                    (prcl_prdata              ),
        .pready                    (prcl_pready              ),
        .pslverr                   (prcl_pslverr             ),
`ifdef MASTER_DIE
        .req_tx_clken              (req_tx_clken             ),
`endif
`ifdef MASTER_DIE
        .req_rx_clken              (req_rx_clken             ),
`endif
`ifdef SLAVE_DIE
        .resp_tx_clken             (resp_tx_clken            ),
`endif
`ifdef SLAVE_DIE
        .resp_rx_clken             (resp_rx_clken            ),
`endif
`ifdef MASTER_DIE
        .mst_paif0_txdata          (paif0_txdata             ),
        .mst_paif0_txvalid         (paif0_txvalid            ),
        .mst_paif0_txready         (paif0_txready            ),
`endif
`ifdef MASTER_DIE
        .mst_paif0_rxdata          (paif0_rxdata             ),
        .mst_paif0_rxvalid         (paif0_rxvalid            ),
        .mst_paif0_rxready         (paif0_rxready            ),
`endif
`ifdef SLAVE_DIE
        .slv_paif0_txdata          (paif1_txdata             ),
        .slv_paif0_txvalid         (paif1_txvalid            ),
        .slv_paif0_txready         (paif1_txready            ),
`endif
`ifdef SLAVE_DIE
        .slv_paif0_rxdata          (paif1_rxdata             ),
        .slv_paif0_rxvalid         (paif1_rxvalid            ),
        .slv_paif0_rxready         (paif1_rxready            ),
`endif
        .paif_clk                  (paif_clk                 ),
        .paif_rstn                 (paif_rstn                ),
`ifdef MASTER_DIE
        // ---- req 方向用户口 (Master Die, AXI4 Slave, s*_axi) ----
        .s0_axi_awid               (s0_axi_awid              ),
        .s0_axi_awaddr             (s0_axi_awaddr            ),
        .s0_axi_awlen              (s0_axi_awlen             ),
        .s0_axi_awsize             (s0_axi_awsize            ),
        .s0_axi_awburst            (s0_axi_awburst           ),
        .s0_axi_awlock             (s0_axi_awlock            ),
        .s0_axi_awcache            (s0_axi_awcache           ),
        .s0_axi_awprot             (s0_axi_awprot            ),
        .s0_axi_awqos              (s0_axi_awqos             ),
        .s0_axi_awregion           (s0_axi_awregion          ),
        .s0_axi_awuser             (s0_axi_awuser            ),
        .s0_axi_awvalid            (s0_axi_awvalid           ),
        .s0_axi_awready            (s0_axi_awready           ),
        .s0_axi_arid               (s0_axi_arid              ),
        .s0_axi_araddr             (s0_axi_araddr            ),
        .s0_axi_arlen              (s0_axi_arlen             ),
        .s0_axi_arsize             (s0_axi_arsize            ),
        .s0_axi_arburst            (s0_axi_arburst           ),
        .s0_axi_arlock             (s0_axi_arlock            ),
        .s0_axi_arcache            (s0_axi_arcache           ),
        .s0_axi_arprot             (s0_axi_arprot            ),
        .s0_axi_arqos              (s0_axi_arqos             ),
        .s0_axi_arregion           (s0_axi_arregion          ),
        .s0_axi_aruser             (s0_axi_aruser            ),
        .s0_axi_arvalid            (s0_axi_arvalid           ),
        .s0_axi_arready            (s0_axi_arready           ),
        .s0_axi_wdata              (s0_axi_wdata             ),
        .s0_axi_wstrb              (s0_axi_wstrb             ),
        .s0_axi_wuser              (s0_axi_wuser             ),
        .s0_axi_wlast              (s0_axi_wlast             ),
        .s0_axi_wvalid             (s0_axi_wvalid            ),
        .s0_axi_wready             (s0_axi_wready            ),
        .s0_axi_rid                (s0_axi_rid               ),
        .s0_axi_rdata              (s0_axi_rdata             ),
        .s0_axi_rresp              (s0_axi_rresp             ),
        .s0_axi_ruser              (s0_axi_ruser             ),
        .s0_axi_rlast              (s0_axi_rlast             ),
        .s0_axi_rvalid             (s0_axi_rvalid            ),
        .s0_axi_rready             (s0_axi_rready            ),
        .s0_axi_bid                (s0_axi_bid               ),
        .s0_axi_bresp              (s0_axi_bresp             ),
        .s0_axi_buser              (s0_axi_buser             ),
        .s0_axi_bvalid             (s0_axi_bvalid            ),
        .s0_axi_bready             (s0_axi_bready            ),
`ifdef MST_TX_LANE_1
        .s1_axi_awid               (s1_axi_awid              ),
        .s1_axi_awaddr             (s1_axi_awaddr            ),
        .s1_axi_awlen              (s1_axi_awlen             ),
        .s1_axi_awsize             (s1_axi_awsize            ),
        .s1_axi_awburst            (s1_axi_awburst           ),
        .s1_axi_awlock             (s1_axi_awlock            ),
        .s1_axi_awcache            (s1_axi_awcache           ),
        .s1_axi_awprot             (s1_axi_awprot            ),
        .s1_axi_awqos              (s1_axi_awqos             ),
        .s1_axi_awregion           (s1_axi_awregion          ),
        .s1_axi_awuser             (s1_axi_awuser            ),
        .s1_axi_awvalid            (s1_axi_awvalid           ),
        .s1_axi_awready            (s1_axi_awready           ),
        .s1_axi_wdata              (s1_axi_wdata             ),
        .s1_axi_wstrb              (s1_axi_wstrb             ),
        .s1_axi_wuser              (s1_axi_wuser             ),
        .s1_axi_wlast              (s1_axi_wlast             ),
        .s1_axi_wvalid             (s1_axi_wvalid            ),
        .s1_axi_wready             (s1_axi_wready            ),
        .s1_axi_bid                (s1_axi_bid               ),
        .s1_axi_bresp              (s1_axi_bresp             ),
        .s1_axi_buser              (s1_axi_buser             ),
        .s1_axi_bvalid             (s1_axi_bvalid            ),
        .s1_axi_bready             (s1_axi_bready            ),
`endif
`ifdef MST_RX_LANE_1
        .s1_axi_arid               (s1_axi_arid              ),
        .s1_axi_araddr             (s1_axi_araddr            ),
        .s1_axi_arlen              (s1_axi_arlen             ),
        .s1_axi_arsize             (s1_axi_arsize            ),
        .s1_axi_arburst            (s1_axi_arburst           ),
        .s1_axi_arlock             (s1_axi_arlock            ),
        .s1_axi_arcache            (s1_axi_arcache           ),
        .s1_axi_arprot             (s1_axi_arprot            ),
        .s1_axi_arqos              (s1_axi_arqos             ),
        .s1_axi_arregion           (s1_axi_arregion          ),
        .s1_axi_aruser             (s1_axi_aruser            ),
        .s1_axi_arvalid            (s1_axi_arvalid           ),
        .s1_axi_arready            (s1_axi_arready           ),
        .s1_axi_rid                (s1_axi_rid               ),
        .s1_axi_rdata              (s1_axi_rdata             ),
        .s1_axi_rresp              (s1_axi_rresp             ),
        .s1_axi_ruser              (s1_axi_ruser             ),
        .s1_axi_rlast              (s1_axi_rlast             ),
        .s1_axi_rvalid             (s1_axi_rvalid            ),
        .s1_axi_rready             (s1_axi_rready            ),
`endif
`ifdef MST_TX_LANE_2
        .s2_axi_awid               (s2_axi_awid              ),
        .s2_axi_awaddr             (s2_axi_awaddr            ),
        .s2_axi_awlen              (s2_axi_awlen             ),
        .s2_axi_awsize             (s2_axi_awsize            ),
        .s2_axi_awburst            (s2_axi_awburst           ),
        .s2_axi_awlock             (s2_axi_awlock            ),
        .s2_axi_awcache            (s2_axi_awcache           ),
        .s2_axi_awprot             (s2_axi_awprot            ),
        .s2_axi_awqos              (s2_axi_awqos             ),
        .s2_axi_awregion           (s2_axi_awregion          ),
        .s2_axi_awuser             (s2_axi_awuser            ),
        .s2_axi_awvalid            (s2_axi_awvalid           ),
        .s2_axi_awready            (s2_axi_awready           ),
        .s2_axi_wdata              (s2_axi_wdata             ),
        .s2_axi_wstrb              (s2_axi_wstrb             ),
        .s2_axi_wuser              (s2_axi_wuser             ),
        .s2_axi_wlast              (s2_axi_wlast             ),
        .s2_axi_wvalid             (s2_axi_wvalid            ),
        .s2_axi_wready             (s2_axi_wready            ),
        .s2_axi_bid                (s2_axi_bid               ),
        .s2_axi_bresp              (s2_axi_bresp             ),
        .s2_axi_buser              (s2_axi_buser             ),
        .s2_axi_bvalid             (s2_axi_bvalid            ),
        .s2_axi_bready             (s2_axi_bready            ),
`endif
`ifdef MST_RX_LANE_2
        .s2_axi_arid               (s2_axi_arid              ),
        .s2_axi_araddr             (s2_axi_araddr            ),
        .s2_axi_arlen              (s2_axi_arlen             ),
        .s2_axi_arsize             (s2_axi_arsize            ),
        .s2_axi_arburst            (s2_axi_arburst           ),
        .s2_axi_arlock             (s2_axi_arlock            ),
        .s2_axi_arcache            (s2_axi_arcache           ),
        .s2_axi_arprot             (s2_axi_arprot            ),
        .s2_axi_arqos              (s2_axi_arqos             ),
        .s2_axi_arregion           (s2_axi_arregion          ),
        .s2_axi_aruser             (s2_axi_aruser            ),
        .s2_axi_arvalid            (s2_axi_arvalid           ),
        .s2_axi_arready            (s2_axi_arready           ),
        .s2_axi_rid                (s2_axi_rid               ),
        .s2_axi_rdata              (s2_axi_rdata             ),
        .s2_axi_rresp              (s2_axi_rresp             ),
        .s2_axi_ruser              (s2_axi_ruser             ),
        .s2_axi_rlast              (s2_axi_rlast             ),
        .s2_axi_rvalid             (s2_axi_rvalid            ),
        .s2_axi_rready             (s2_axi_rready            ),
`endif
`ifdef MST_TX_LANE_3
        .s3_axi_awid               (s3_axi_awid              ),
        .s3_axi_awaddr             (s3_axi_awaddr            ),
        .s3_axi_awlen              (s3_axi_awlen             ),
        .s3_axi_awsize             (s3_axi_awsize            ),
        .s3_axi_awburst            (s3_axi_awburst           ),
        .s3_axi_awlock             (s3_axi_awlock            ),
        .s3_axi_awcache            (s3_axi_awcache           ),
        .s3_axi_awprot             (s3_axi_awprot            ),
        .s3_axi_awqos              (s3_axi_awqos             ),
        .s3_axi_awregion           (s3_axi_awregion          ),
        .s3_axi_awuser             (s3_axi_awuser            ),
        .s3_axi_awvalid            (s3_axi_awvalid           ),
        .s3_axi_awready            (s3_axi_awready           ),
        .s3_axi_wdata              (s3_axi_wdata             ),
        .s3_axi_wstrb              (s3_axi_wstrb             ),
        .s3_axi_wuser              (s3_axi_wuser             ),
        .s3_axi_wlast              (s3_axi_wlast             ),
        .s3_axi_wvalid             (s3_axi_wvalid            ),
        .s3_axi_wready             (s3_axi_wready            ),
        .s3_axi_bid                (s3_axi_bid               ),
        .s3_axi_bresp              (s3_axi_bresp             ),
        .s3_axi_buser              (s3_axi_buser             ),
        .s3_axi_bvalid             (s3_axi_bvalid            ),
        .s3_axi_bready             (s3_axi_bready            ),
`endif
`ifdef MST_RX_LANE_3
        .s3_axi_arid               (s3_axi_arid              ),
        .s3_axi_araddr             (s3_axi_araddr            ),
        .s3_axi_arlen              (s3_axi_arlen             ),
        .s3_axi_arsize             (s3_axi_arsize            ),
        .s3_axi_arburst            (s3_axi_arburst           ),
        .s3_axi_arlock             (s3_axi_arlock            ),
        .s3_axi_arcache            (s3_axi_arcache           ),
        .s3_axi_arprot             (s3_axi_arprot            ),
        .s3_axi_arqos              (s3_axi_arqos             ),
        .s3_axi_arregion           (s3_axi_arregion          ),
        .s3_axi_aruser             (s3_axi_aruser            ),
        .s3_axi_arvalid            (s3_axi_arvalid           ),
        .s3_axi_arready            (s3_axi_arready           ),
        .s3_axi_rid                (s3_axi_rid               ),
        .s3_axi_rdata              (s3_axi_rdata             ),
        .s3_axi_rresp              (s3_axi_rresp             ),
        .s3_axi_ruser              (s3_axi_ruser             ),
        .s3_axi_rlast              (s3_axi_rlast             ),
        .s3_axi_rvalid             (s3_axi_rvalid            ),
        .s3_axi_rready             (s3_axi_rready            ),
`endif
`endif
`ifdef SLAVE_DIE
        // ---- resp 方向用户口 (Slave Die, AXI4 Master, m*_axi) ----
        .m0_axi_awid               (m0_axi_awid              ),
        .m0_axi_awaddr             (m0_axi_awaddr            ),
        .m0_axi_awlen              (m0_axi_awlen             ),
        .m0_axi_awsize             (m0_axi_awsize            ),
        .m0_axi_awburst            (m0_axi_awburst           ),
        .m0_axi_awlock             (m0_axi_awlock            ),
        .m0_axi_awcache            (m0_axi_awcache           ),
        .m0_axi_awprot             (m0_axi_awprot            ),
        .m0_axi_awqos              (m0_axi_awqos             ),
        .m0_axi_awregion           (m0_axi_awregion          ),
        .m0_axi_awuser             (m0_axi_awuser            ),
        .m0_axi_awvalid            (m0_axi_awvalid           ),
        .m0_axi_awready            (m0_axi_awready           ),
        .m0_axi_arid               (m0_axi_arid              ),
        .m0_axi_araddr             (m0_axi_araddr            ),
        .m0_axi_arlen              (m0_axi_arlen             ),
        .m0_axi_arsize             (m0_axi_arsize            ),
        .m0_axi_arburst            (m0_axi_arburst           ),
        .m0_axi_arlock             (m0_axi_arlock            ),
        .m0_axi_arcache            (m0_axi_arcache           ),
        .m0_axi_arprot             (m0_axi_arprot            ),
        .m0_axi_arqos              (m0_axi_arqos             ),
        .m0_axi_arregion           (m0_axi_arregion          ),
        .m0_axi_aruser             (m0_axi_aruser            ),
        .m0_axi_arvalid            (m0_axi_arvalid           ),
        .m0_axi_arready            (m0_axi_arready           ),
        .m0_axi_wdata              (m0_axi_wdata             ),
        .m0_axi_wstrb              (m0_axi_wstrb             ),
        .m0_axi_wuser              (m0_axi_wuser             ),
        .m0_axi_wlast              (m0_axi_wlast             ),
        .m0_axi_wvalid             (m0_axi_wvalid            ),
        .m0_axi_wready             (m0_axi_wready            ),
        .m0_axi_rid                (m0_axi_rid               ),
        .m0_axi_rdata              (m0_axi_rdata             ),
        .m0_axi_rresp              (m0_axi_rresp             ),
        .m0_axi_ruser              (m0_axi_ruser             ),
        .m0_axi_rlast              (m0_axi_rlast             ),
        .m0_axi_rvalid             (m0_axi_rvalid            ),
        .m0_axi_rready             (m0_axi_rready            ),
        .m0_axi_bid                (m0_axi_bid               ),
        .m0_axi_bresp              (m0_axi_bresp             ),
        .m0_axi_buser              (m0_axi_buser             ),
        .m0_axi_bvalid             (m0_axi_bvalid            ),
        .m0_axi_bready             (m0_axi_bready            ),
`ifdef SLV_RX_LANE_1
        .m1_axi_awid               (m1_axi_awid              ),
        .m1_axi_awaddr             (m1_axi_awaddr            ),
        .m1_axi_awlen              (m1_axi_awlen             ),
        .m1_axi_awsize             (m1_axi_awsize            ),
        .m1_axi_awburst            (m1_axi_awburst           ),
        .m1_axi_awlock             (m1_axi_awlock            ),
        .m1_axi_awcache            (m1_axi_awcache           ),
        .m1_axi_awprot             (m1_axi_awprot            ),
        .m1_axi_awqos              (m1_axi_awqos             ),
        .m1_axi_awregion           (m1_axi_awregion          ),
        .m1_axi_awuser             (m1_axi_awuser            ),
        .m1_axi_awvalid            (m1_axi_awvalid           ),
        .m1_axi_awready            (m1_axi_awready           ),
        .m1_axi_wdata              (m1_axi_wdata             ),
        .m1_axi_wstrb              (m1_axi_wstrb             ),
        .m1_axi_wuser              (m1_axi_wuser             ),
        .m1_axi_wlast              (m1_axi_wlast             ),
        .m1_axi_wvalid             (m1_axi_wvalid            ),
        .m1_axi_wready             (m1_axi_wready            ),
        .m1_axi_bid                (m1_axi_bid               ),
        .m1_axi_bresp              (m1_axi_bresp             ),
        .m1_axi_buser              (m1_axi_buser             ),
        .m1_axi_bvalid             (m1_axi_bvalid            ),
        .m1_axi_bready             (m1_axi_bready            ),
`endif
`ifdef SLV_TX_LANE_1
        .m1_axi_arid               (m1_axi_arid              ),
        .m1_axi_araddr             (m1_axi_araddr            ),
        .m1_axi_arlen              (m1_axi_arlen             ),
        .m1_axi_arsize             (m1_axi_arsize            ),
        .m1_axi_arburst            (m1_axi_arburst           ),
        .m1_axi_arlock             (m1_axi_arlock            ),
        .m1_axi_arcache            (m1_axi_arcache           ),
        .m1_axi_arprot             (m1_axi_arprot            ),
        .m1_axi_arqos              (m1_axi_arqos             ),
        .m1_axi_arregion           (m1_axi_arregion          ),
        .m1_axi_aruser             (m1_axi_aruser            ),
        .m1_axi_arvalid            (m1_axi_arvalid           ),
        .m1_axi_arready            (m1_axi_arready           ),
        .m1_axi_rid                (m1_axi_rid               ),
        .m1_axi_rdata              (m1_axi_rdata             ),
        .m1_axi_rresp              (m1_axi_rresp             ),
        .m1_axi_ruser              (m1_axi_ruser             ),
        .m1_axi_rlast              (m1_axi_rlast             ),
        .m1_axi_rvalid             (m1_axi_rvalid            ),
        .m1_axi_rready             (m1_axi_rready            ),
`endif
`ifdef SLV_RX_LANE_2
        .m2_axi_awid               (m2_axi_awid              ),
        .m2_axi_awaddr             (m2_axi_awaddr            ),
        .m2_axi_awlen              (m2_axi_awlen             ),
        .m2_axi_awsize             (m2_axi_awsize            ),
        .m2_axi_awburst            (m2_axi_awburst           ),
        .m2_axi_awlock             (m2_axi_awlock            ),
        .m2_axi_awcache            (m2_axi_awcache           ),
        .m2_axi_awprot             (m2_axi_awprot            ),
        .m2_axi_awqos              (m2_axi_awqos             ),
        .m2_axi_awregion           (m2_axi_awregion          ),
        .m2_axi_awuser             (m2_axi_awuser            ),
        .m2_axi_awvalid            (m2_axi_awvalid           ),
        .m2_axi_awready            (m2_axi_awready           ),
        .m2_axi_wdata              (m2_axi_wdata             ),
        .m2_axi_wstrb              (m2_axi_wstrb             ),
        .m2_axi_wuser              (m2_axi_wuser             ),
        .m2_axi_wlast              (m2_axi_wlast             ),
        .m2_axi_wvalid             (m2_axi_wvalid            ),
        .m2_axi_wready             (m2_axi_wready            ),
        .m2_axi_bid                (m2_axi_bid               ),
        .m2_axi_bresp              (m2_axi_bresp             ),
        .m2_axi_buser              (m2_axi_buser             ),
        .m2_axi_bvalid             (m2_axi_bvalid            ),
        .m2_axi_bready             (m2_axi_bready            ),
`endif
`ifdef SLV_TX_LANE_2
        .m2_axi_arid               (m2_axi_arid              ),
        .m2_axi_araddr             (m2_axi_araddr            ),
        .m2_axi_arlen              (m2_axi_arlen             ),
        .m2_axi_arsize             (m2_axi_arsize            ),
        .m2_axi_arburst            (m2_axi_arburst           ),
        .m2_axi_arlock             (m2_axi_arlock            ),
        .m2_axi_arcache            (m2_axi_arcache           ),
        .m2_axi_arprot             (m2_axi_arprot            ),
        .m2_axi_arqos              (m2_axi_arqos             ),
        .m2_axi_arregion           (m2_axi_arregion          ),
        .m2_axi_aruser             (m2_axi_aruser            ),
        .m2_axi_arvalid            (m2_axi_arvalid           ),
        .m2_axi_arready            (m2_axi_arready           ),
        .m2_axi_rid                (m2_axi_rid               ),
        .m2_axi_rdata              (m2_axi_rdata             ),
        .m2_axi_rresp              (m2_axi_rresp             ),
        .m2_axi_ruser              (m2_axi_ruser             ),
        .m2_axi_rlast              (m2_axi_rlast             ),
        .m2_axi_rvalid             (m2_axi_rvalid            ),
        .m2_axi_rready             (m2_axi_rready            ),
`endif
`ifdef SLV_RX_LANE_3
        .m3_axi_awid               (m3_axi_awid              ),
        .m3_axi_awaddr             (m3_axi_awaddr            ),
        .m3_axi_awlen              (m3_axi_awlen             ),
        .m3_axi_awsize             (m3_axi_awsize            ),
        .m3_axi_awburst            (m3_axi_awburst           ),
        .m3_axi_awlock             (m3_axi_awlock            ),
        .m3_axi_awcache            (m3_axi_awcache           ),
        .m3_axi_awprot             (m3_axi_awprot            ),
        .m3_axi_awqos              (m3_axi_awqos             ),
        .m3_axi_awregion           (m3_axi_awregion          ),
        .m3_axi_awuser             (m3_axi_awuser            ),
        .m3_axi_awvalid            (m3_axi_awvalid           ),
        .m3_axi_awready            (m3_axi_awready           ),
        .m3_axi_wdata              (m3_axi_wdata             ),
        .m3_axi_wstrb              (m3_axi_wstrb             ),
        .m3_axi_wuser              (m3_axi_wuser             ),
        .m3_axi_wlast              (m3_axi_wlast             ),
        .m3_axi_wvalid             (m3_axi_wvalid            ),
        .m3_axi_wready             (m3_axi_wready            ),
        .m3_axi_bid                (m3_axi_bid               ),
        .m3_axi_bresp              (m3_axi_bresp             ),
        .m3_axi_buser              (m3_axi_buser             ),
        .m3_axi_bvalid             (m3_axi_bvalid            ),
        .m3_axi_bready             (m3_axi_bready            ),
`endif
`ifdef SLV_TX_LANE_3
        .m3_axi_arid               (m3_axi_arid              ),
        .m3_axi_araddr             (m3_axi_araddr            ),
        .m3_axi_arlen              (m3_axi_arlen             ),
        .m3_axi_arsize             (m3_axi_arsize            ),
        .m3_axi_arburst            (m3_axi_arburst           ),
        .m3_axi_arlock             (m3_axi_arlock            ),
        .m3_axi_arcache            (m3_axi_arcache           ),
        .m3_axi_arprot             (m3_axi_arprot            ),
        .m3_axi_arqos              (m3_axi_arqos             ),
        .m3_axi_arregion           (m3_axi_arregion          ),
        .m3_axi_aruser             (m3_axi_aruser            ),
        .m3_axi_arvalid            (m3_axi_arvalid           ),
        .m3_axi_arready            (m3_axi_arready           ),
        .m3_axi_rid                (m3_axi_rid               ),
        .m3_axi_rdata              (m3_axi_rdata             ),
        .m3_axi_rresp              (m3_axi_rresp             ),
        .m3_axi_ruser              (m3_axi_ruser             ),
        .m3_axi_rlast              (m3_axi_rlast             ),
        .m3_axi_rvalid             (m3_axi_rvalid            ),
        .m3_axi_rready             (m3_axi_rready            ),
`endif
`endif
        .intr                      (prcl_intr                )
    );
`else
    // ==========================================================================
    //  AXI 对称模式（AXI_ASYM_MODE 未定义时，1T1R 对称）：axi4_symm_prcl（单实例）
    //  ==========================================================================
    axi4_symm_prcl u_axi_symm_prcl (
        .aclk                      (aclk                     ),
        .aresetn                   (aresetn_sync             ),
        .prcl_clk                  (mb_clk                   ),
        .prcl_rstn                 (prcl_mb_rstn             ),
        .pclk                      (pclk                     ),
        .presetn                   (presetn_sync             ),
        .clken                     (tx_clken[0]              ),   // 对称单 clken（lane0 gating）
        .lp_ctrl_cmd               (lp_ctrl_cmd              ),
        .lp_ctrl_req               (lp_ctrl_req              ),
        .lp_ctrl_ack               (lp_ctrl_ack              ),
        .lp_ctrl_sts               (lp_ctrl_sts              ),
        .lp_sw_cmd                 (lp_sw_cmd_prcl           ),
        .lp_sw_req                 (lp_sw_req_prcl           ),
        .lp_sw_ack                 (lp_sw_ack_prcl           ),
        .lp_sw_sts                 (lp_sw_sts_prcl           ),
        .psel                      (psel_to_prcl             ),
        .penable                   (penable                  ),
        .pwrite                    (pwrite                   ),
        .paddr                     (paddr[11:0]              ),
        .pwdata                    (pwdata                   ),
        .prdata                    (prcl_prdata              ),
        .pready                    (prcl_pready              ),
        .pslverr                   (prcl_pslverr             ),
        // ---- 用户侧：s_axi（slave）+ m_axi（master），单组（对称 lane0）----
        .s_axi_awid                (s0_axi_awid              ),
        .s_axi_awaddr              (s0_axi_awaddr            ),
        .s_axi_awlen               (s0_axi_awlen             ),
        .s_axi_awsize              (s0_axi_awsize            ),
        .s_axi_awburst             (s0_axi_awburst           ),
        .s_axi_awlock              (s0_axi_awlock            ),
        .s_axi_awcache             (s0_axi_awcache           ),
        .s_axi_awprot              (s0_axi_awprot            ),
        .s_axi_awqos               (s0_axi_awqos             ),
        .s_axi_awregion            (s0_axi_awregion          ),
        .s_axi_awuser              (s0_axi_awuser            ),
        .s_axi_awvalid             (s0_axi_awvalid           ),
        .s_axi_awready             (s0_axi_awready           ),
        .s_axi_arid                (s0_axi_arid              ),
        .s_axi_araddr              (s0_axi_araddr            ),
        .s_axi_arlen               (s0_axi_arlen             ),
        .s_axi_arsize              (s0_axi_arsize            ),
        .s_axi_arburst             (s0_axi_arburst           ),
        .s_axi_arlock              (s0_axi_arlock            ),
        .s_axi_arcache             (s0_axi_arcache           ),
        .s_axi_arprot              (s0_axi_arprot            ),
        .s_axi_arqos               (s0_axi_arqos             ),
        .s_axi_arregion            (s0_axi_arregion          ),
        .s_axi_aruser              (s0_axi_aruser            ),
        .s_axi_arvalid             (s0_axi_arvalid           ),
        .s_axi_arready             (s0_axi_arready           ),
        .s_axi_wdata               (s0_axi_wdata             ),
        .s_axi_wstrb               (s0_axi_wstrb             ),
        .s_axi_wuser               (s0_axi_wuser             ),
        .s_axi_wlast               (s0_axi_wlast             ),
        .s_axi_wvalid              (s0_axi_wvalid            ),
        .s_axi_wready              (s0_axi_wready            ),
        .s_axi_rid                 (s0_axi_rid               ),
        .s_axi_rdata               (s0_axi_rdata             ),
        .s_axi_rresp               (s0_axi_rresp             ),
        .s_axi_ruser               (s0_axi_ruser             ),
        .s_axi_rlast               (s0_axi_rlast             ),
        .s_axi_rvalid              (s0_axi_rvalid            ),
        .s_axi_rready              (s0_axi_rready            ),
        .s_axi_bid                 (s0_axi_bid               ),
        .s_axi_bresp               (s0_axi_bresp             ),
        .s_axi_buser               (s0_axi_buser             ),
        .s_axi_bvalid              (s0_axi_bvalid            ),
        .s_axi_bready              (s0_axi_bready            ),
        .m_axi_awid                (m0_axi_awid              ),
        .m_axi_awaddr              (m0_axi_awaddr            ),
        .m_axi_awlen               (m0_axi_awlen             ),
        .m_axi_awsize              (m0_axi_awsize            ),
        .m_axi_awburst             (m0_axi_awburst           ),
        .m_axi_awlock              (m0_axi_awlock            ),
        .m_axi_awcache             (m0_axi_awcache           ),
        .m_axi_awprot              (m0_axi_awprot            ),
        .m_axi_awqos               (m0_axi_awqos             ),
        .m_axi_awregion            (m0_axi_awregion          ),
        .m_axi_awuser              (m0_axi_awuser            ),
        .m_axi_awvalid             (m0_axi_awvalid           ),
        .m_axi_awready             (m0_axi_awready           ),
        .m_axi_arid                (m0_axi_arid              ),
        .m_axi_araddr              (m0_axi_araddr            ),
        .m_axi_arlen               (m0_axi_arlen             ),
        .m_axi_arsize              (m0_axi_arsize            ),
        .m_axi_arburst             (m0_axi_arburst           ),
        .m_axi_arlock              (m0_axi_arlock            ),
        .m_axi_arcache             (m0_axi_arcache           ),
        .m_axi_arprot              (m0_axi_arprot            ),
        .m_axi_arqos               (m0_axi_arqos             ),
        .m_axi_arregion            (m0_axi_arregion          ),
        .m_axi_aruser              (m0_axi_aruser            ),
        .m_axi_arvalid             (m0_axi_arvalid           ),
        .m_axi_arready             (m0_axi_arready           ),
        .m_axi_wdata               (m0_axi_wdata             ),
        .m_axi_wstrb               (m0_axi_wstrb             ),
        .m_axi_wuser               (m0_axi_wuser             ),
        .m_axi_wlast               (m0_axi_wlast             ),
        .m_axi_wvalid              (m0_axi_wvalid            ),
        .m_axi_wready              (m0_axi_wready            ),
        .m_axi_rid                 (m0_axi_rid               ),
        .m_axi_rdata               (m0_axi_rdata             ),
        .m_axi_rresp               (m0_axi_rresp             ),
        .m_axi_ruser               (m0_axi_ruser             ),
        .m_axi_rlast               (m0_axi_rlast             ),
        .m_axi_rvalid              (m0_axi_rvalid            ),
        .m_axi_rready              (m0_axi_rready            ),
        .m_axi_bid                 (m0_axi_bid               ),
        .m_axi_bresp               (m0_axi_bresp             ),
        .m_axi_buser               (m0_axi_buser             ),
        .m_axi_bvalid              (m0_axi_bvalid            ),
        .m_axi_bready              (m0_axi_bready            ),
        // ---- PAIF 单组（对称 lane0，随 PRCL_TX_LANE 控制）----
        .paif_txdata               (paif0_txdata             ),
        .paif_txvalid              (paif0_txvalid            ),
        .paif_txready              (paif0_txready            ),
        .paif_rxdata               (paif0_rxdata             ),
        .paif_rxvalid              (paif0_rxvalid            ),
        .paif_rxready              (paif0_rxready            ),
        .paif_clk                  (paif_clk                 ),
        .paif_rstn                 (paif_rstn                ),
        .intr                      (prcl_intr                )
    );
`endif
`endif
`else
    cxs_prcl u_cxs_prcl (
        .pclk                      (pclk                     ),
        .mb_clk                    (mb_clk                   ),
        .cxs_clk                   (cxs_clk                  ),
        .mb_rstn                   (prcl_mb_rstn             ),
        .cxs_rstn                  (cxs_rstn                 ),
        .presetn                   (presetn_sync             ),
        .psel                      (psel_to_prcl             ),
        .penable                   (penable                  ),
        .paddr                     (paddr[15:0]              ),
        .pwrite                    (pwrite                   ),
        .pwdata                    (pwdata                   ),
        .prdata                    (prcl_prdata              ),
        .pready                    (prcl_pready              ),
        .pslverr                   (prcl_pslverr             ),
        .paif0_txdata              (paif0_txdata             ),
        .paif0_txvalid             (paif0_txvalid            ),
        .paif0_txready             (paif0_txready            ),
        .paif0_rxdata              (paif0_rxdata             ),
        .paif0_rxvalid             (paif0_rxvalid            ),
        .paif0_rxready             (paif0_rxready            ),
`ifdef PRCL_TX_LANE_1
        .paif1_txdata              (paif1_txdata             ),
        .paif1_txvalid             (paif1_txvalid            ),
        .paif1_txready             (paif1_txready            ),
`endif
`ifdef PRCL_RX_LANE_1
        .paif1_rxdata              (paif1_rxdata             ),
        .paif1_rxvalid             (paif1_rxvalid            ),
        .paif1_rxready             (paif1_rxready            ),
`endif
`ifdef PRCL_TX_LANE_2
        .paif2_txdata              (paif2_txdata             ),
        .paif2_txvalid             (paif2_txvalid            ),
        .paif2_txready             (paif2_txready            ),
`endif
`ifdef PRCL_RX_LANE_2
        .paif2_rxdata              (paif2_rxdata             ),
        .paif2_rxvalid             (paif2_rxvalid            ),
        .paif2_rxready             (paif2_rxready            ),
`endif
`ifdef PRCL_TX_LANE_3
        .paif3_txdata              (paif3_txdata             ),
        .paif3_txvalid             (paif3_txvalid            ),
        .paif3_txready             (paif3_txready            ),
`endif
`ifdef PRCL_RX_LANE_3
        .paif3_rxdata              (paif3_rxdata             ),
        .paif3_rxvalid             (paif3_rxvalid            ),
        .paif3_rxready             (paif3_rxready            ),
`endif
        .lp_ctrl_cmd               (lp_ctrl_cmd              ),
        .lp_ctrl_req               (lp_ctrl_req              ),
        .lp_ctrl_sts               (lp_ctrl_sts              ),
        .lp_ctrl_ack               (lp_ctrl_ack              ),
        .lp_sw_cmd                 (lp_sw_cmd_prcl           ),
        .lp_sw_req                 (lp_sw_req_prcl           ),
        .lp_sw_sts                 (lp_sw_sts_prcl           ),
        .lp_sw_ack                 (lp_sw_ack_prcl           ),
        .paif_clk                  (paif_clk                 ),
        .paif_rstn                 (paif_rstn                ),
        .cxs_prcl0_rx_data         (cxs_prcl0_rx_data        ),
        .cxs_prcl0_rx_cntl         (cxs_prcl0_rx_cntl        ),
        .cxs_prcl0_rx_valid        (cxs_prcl0_rx_valid       ),
        .cxs_prcl0_rx_prcltype     (cxs_prcl0_rx_prcltype    ),
        .cxs_prcl0_rx_srcid        (cxs_prcl0_rx_srcid       ),
        .cxs_prcl0_rx_tgtid        (cxs_prcl0_rx_tgtid       ),
        .cxs_prcl0_rx_last         (cxs_prcl0_rx_last        ),
        .cxs_prcl0_rx_chk_data     (cxs_prcl0_rx_chk_data    ),
        .cxs_prcl0_rx_chk_cntl     (cxs_prcl0_rx_chk_cntl    ),
        .cxs_prcl0_rx_chk_prcltype (cxs_prcl0_rx_chk_prcltype),
        .cxs_prcl0_rx_chk_srcid    (cxs_prcl0_rx_chk_srcid   ),
        .cxs_prcl0_rx_chk_tgtid    (cxs_prcl0_rx_chk_tgtid   ),
        .cxs_prcl0_rx_chk_last     (cxs_prcl0_rx_chk_last    ),
        .cxs_prcl0_rx_chk_valid    (cxs_prcl0_rx_chk_valid   ),
        .cxs_prcl0_rx_crdrtn       (cxs_prcl0_rx_crdrtn      ),
        .cxs_prcl0_rx_crdgnt       (cxs_prcl0_rx_crdgnt      ),
        .cxs_prcl0_rx_activereq    (cxs_prcl0_rx_activereq   ),
        .cxs_prcl0_rx_activeack    (cxs_prcl0_rx_activeack   ),
        .cxs_prcl0_rx_deacthint    (cxs_prcl0_rx_deacthint   ),
        .cxs_prcl0_tx_data         (cxs_prcl0_tx_data        ),
        .cxs_prcl0_tx_cntl         (cxs_prcl0_tx_cntl        ),
        .cxs_prcl0_tx_valid        (cxs_prcl0_tx_valid       ),
        .cxs_prcl0_tx_prcltype     (cxs_prcl0_tx_prcltype    ),
        .cxs_prcl0_tx_srcid        (cxs_prcl0_tx_srcid       ),
        .cxs_prcl0_tx_tgtid        (cxs_prcl0_tx_tgtid       ),
        .cxs_prcl0_tx_last         (cxs_prcl0_tx_last        ),
        .cxs_prcl0_tx_chk_data     (cxs_prcl0_tx_chk_data    ),
        .cxs_prcl0_tx_chk_cntl     (cxs_prcl0_tx_chk_cntl    ),
        .cxs_prcl0_tx_chk_prcltype (cxs_prcl0_tx_chk_prcltype),
        .cxs_prcl0_tx_chk_srcid    (cxs_prcl0_tx_chk_srcid   ),
        .cxs_prcl0_tx_chk_tgtid    (cxs_prcl0_tx_chk_tgtid   ),
        .cxs_prcl0_tx_chk_last     (cxs_prcl0_tx_chk_last    ),
        .cxs_prcl0_tx_chk_valid    (cxs_prcl0_tx_chk_valid   ),
        .cxs_prcl0_tx_crdrtn       (cxs_prcl0_tx_crdrtn      ),
        .cxs_prcl0_tx_crdgnt       (cxs_prcl0_tx_crdgnt      ),
        .cxs_prcl0_tx_activereq    (cxs_prcl0_tx_activereq   ),
        .cxs_prcl0_tx_activeack    (cxs_prcl0_tx_activeack   ),
        .cxs_prcl0_tx_deacthint    (cxs_prcl0_tx_deacthint   ),
`ifdef PRCL_TX_LANE_1
        .cxs_prcl1_rx_data         (cxs_prcl1_rx_data        ),
        .cxs_prcl1_rx_cntl         (cxs_prcl1_rx_cntl        ),
        .cxs_prcl1_rx_valid        (cxs_prcl1_rx_valid       ),
        .cxs_prcl1_rx_prcltype     (cxs_prcl1_rx_prcltype    ),
        .cxs_prcl1_rx_srcid        (cxs_prcl1_rx_srcid       ),
        .cxs_prcl1_rx_tgtid        (cxs_prcl1_rx_tgtid       ),
`ifdef CXSPRCL_LAST
        .cxs_prcl1_rx_last         (cxs_prcl1_rx_last        ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl1_rx_chk_data     (cxs_prcl1_rx_chk_data    ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl1_rx_chk_cntl     (cxs_prcl1_rx_chk_cntl    ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl1_rx_chk_prcltype (cxs_prcl1_rx_chk_prcltype),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl1_rx_chk_srcid    (cxs_prcl1_rx_chk_srcid   ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl1_rx_chk_tgtid    (cxs_prcl1_rx_chk_tgtid   ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl1_rx_chk_last     (cxs_prcl1_rx_chk_last    ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl1_rx_chk_valid    (cxs_prcl1_rx_chk_valid   ),
`endif
        .cxs_prcl1_rx_crdrtn       (cxs_prcl1_rx_crdrtn      ),
        .cxs_prcl1_rx_crdgnt       (cxs_prcl1_rx_crdgnt      ),
        .cxs_prcl1_rx_activereq    (cxs_prcl1_rx_activereq   ),
        .cxs_prcl1_rx_activeack    (cxs_prcl1_rx_activeack   ),
        .cxs_prcl1_rx_deacthint    (cxs_prcl1_rx_deacthint   ),
`endif
`ifdef PRCL_RX_LANE_1
        .cxs_prcl1_tx_data         (cxs_prcl1_tx_data        ),
        .cxs_prcl1_tx_cntl         (cxs_prcl1_tx_cntl        ),
        .cxs_prcl1_tx_valid        (cxs_prcl1_tx_valid       ),
        .cxs_prcl1_tx_prcltype     (cxs_prcl1_tx_prcltype    ),
        .cxs_prcl1_tx_srcid        (cxs_prcl1_tx_srcid       ),
        .cxs_prcl1_tx_tgtid        (cxs_prcl1_tx_tgtid       ),
`ifdef CXSPRCL_LAST
        .cxs_prcl1_tx_last         (cxs_prcl1_tx_last        ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl1_tx_chk_data     (cxs_prcl1_tx_chk_data    ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl1_tx_chk_cntl     (cxs_prcl1_tx_chk_cntl    ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl1_tx_chk_prcltype (cxs_prcl1_tx_chk_prcltype),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl1_tx_chk_srcid    (cxs_prcl1_tx_chk_srcid   ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl1_tx_chk_tgtid    (cxs_prcl1_tx_chk_tgtid   ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl1_tx_chk_last     (cxs_prcl1_tx_chk_last    ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl1_tx_chk_valid    (cxs_prcl1_tx_chk_valid   ),
`endif
        .cxs_prcl1_tx_crdrtn       (cxs_prcl1_tx_crdrtn      ),
        .cxs_prcl1_tx_crdgnt       (cxs_prcl1_tx_crdgnt      ),
        .cxs_prcl1_tx_activereq    (cxs_prcl1_tx_activereq   ),
        .cxs_prcl1_tx_activeack    (cxs_prcl1_tx_activeack   ),
        .cxs_prcl1_tx_deacthint    (cxs_prcl1_tx_deacthint   ),
`endif
`ifdef PRCL_TX_LANE_2
        .cxs_prcl2_rx_data         (cxs_prcl2_rx_data        ),
        .cxs_prcl2_rx_cntl         (cxs_prcl2_rx_cntl        ),
        .cxs_prcl2_rx_valid        (cxs_prcl2_rx_valid       ),
        .cxs_prcl2_rx_prcltype     (cxs_prcl2_rx_prcltype    ),
        .cxs_prcl2_rx_srcid        (cxs_prcl2_rx_srcid       ),
        .cxs_prcl2_rx_tgtid        (cxs_prcl2_rx_tgtid       ),
`ifdef CXSPRCL_LAST
        .cxs_prcl2_rx_last         (cxs_prcl2_rx_last        ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl2_rx_chk_data     (cxs_prcl2_rx_chk_data    ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl2_rx_chk_cntl     (cxs_prcl2_rx_chk_cntl    ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl2_rx_chk_prcltype (cxs_prcl2_rx_chk_prcltype),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl2_rx_chk_srcid    (cxs_prcl2_rx_chk_srcid   ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl2_rx_chk_tgtid    (cxs_prcl2_rx_chk_tgtid   ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl2_rx_chk_last     (cxs_prcl2_rx_chk_last    ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl2_rx_chk_valid    (cxs_prcl2_rx_chk_valid   ),
`endif
        .cxs_prcl2_rx_crdrtn       (cxs_prcl2_rx_crdrtn      ),
        .cxs_prcl2_rx_crdgnt       (cxs_prcl2_rx_crdgnt      ),
        .cxs_prcl2_rx_activereq    (cxs_prcl2_rx_activereq   ),
        .cxs_prcl2_rx_activeack    (cxs_prcl2_rx_activeack   ),
        .cxs_prcl2_rx_deacthint    (cxs_prcl2_rx_deacthint   ),
`endif
`ifdef PRCL_RX_LANE_2
        .cxs_prcl2_tx_data         (cxs_prcl2_tx_data        ),
        .cxs_prcl2_tx_cntl         (cxs_prcl2_tx_cntl        ),
        .cxs_prcl2_tx_valid        (cxs_prcl2_tx_valid       ),
        .cxs_prcl2_tx_prcltype     (cxs_prcl2_tx_prcltype    ),
        .cxs_prcl2_tx_srcid        (cxs_prcl2_tx_srcid       ),
        .cxs_prcl2_tx_tgtid        (cxs_prcl2_tx_tgtid       ),
`ifdef CXSPRCL_LAST
        .cxs_prcl2_tx_last         (cxs_prcl2_tx_last        ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl2_tx_chk_data     (cxs_prcl2_tx_chk_data    ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl2_tx_chk_cntl     (cxs_prcl2_tx_chk_cntl    ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl2_tx_chk_prcltype (cxs_prcl2_tx_chk_prcltype),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl2_tx_chk_srcid    (cxs_prcl2_tx_chk_srcid   ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl2_tx_chk_tgtid    (cxs_prcl2_tx_chk_tgtid   ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl2_tx_chk_last     (cxs_prcl2_tx_chk_last    ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl2_tx_chk_valid    (cxs_prcl2_tx_chk_valid   ),
`endif
        .cxs_prcl2_tx_crdrtn       (cxs_prcl2_tx_crdrtn      ),
        .cxs_prcl2_tx_crdgnt       (cxs_prcl2_tx_crdgnt      ),
        .cxs_prcl2_tx_activereq    (cxs_prcl2_tx_activereq   ),
        .cxs_prcl2_tx_activeack    (cxs_prcl2_tx_activeack   ),
        .cxs_prcl2_tx_deacthint    (cxs_prcl2_tx_deacthint   ),
`endif
`ifdef PRCL_TX_LANE_3
        .cxs_prcl3_rx_data         (cxs_prcl3_rx_data        ),
        .cxs_prcl3_rx_cntl         (cxs_prcl3_rx_cntl        ),
        .cxs_prcl3_rx_valid        (cxs_prcl3_rx_valid       ),
        .cxs_prcl3_rx_prcltype     (cxs_prcl3_rx_prcltype    ),
        .cxs_prcl3_rx_srcid        (cxs_prcl3_rx_srcid       ),
        .cxs_prcl3_rx_tgtid        (cxs_prcl3_rx_tgtid       ),
`ifdef CXSPRCL_LAST
        .cxs_prcl3_rx_last         (cxs_prcl3_rx_last        ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl3_rx_chk_data     (cxs_prcl3_rx_chk_data    ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl3_rx_chk_cntl     (cxs_prcl3_rx_chk_cntl    ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl3_rx_chk_prcltype (cxs_prcl3_rx_chk_prcltype),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl3_rx_chk_srcid    (cxs_prcl3_rx_chk_srcid   ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl3_rx_chk_tgtid    (cxs_prcl3_rx_chk_tgtid   ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl3_rx_chk_last     (cxs_prcl3_rx_chk_last    ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl3_rx_chk_valid    (cxs_prcl3_rx_chk_valid   ),
`endif
        .cxs_prcl3_rx_crdrtn       (cxs_prcl3_rx_crdrtn      ),
        .cxs_prcl3_rx_crdgnt       (cxs_prcl3_rx_crdgnt      ),
        .cxs_prcl3_rx_activereq    (cxs_prcl3_rx_activereq   ),
        .cxs_prcl3_rx_activeack    (cxs_prcl3_rx_activeack   ),
        .cxs_prcl3_rx_deacthint    (cxs_prcl3_rx_deacthint   ),
`endif
`ifdef PRCL_RX_LANE_3
        .cxs_prcl3_tx_data         (cxs_prcl3_tx_data        ),
        .cxs_prcl3_tx_cntl         (cxs_prcl3_tx_cntl        ),
        .cxs_prcl3_tx_valid        (cxs_prcl3_tx_valid       ),
        .cxs_prcl3_tx_prcltype     (cxs_prcl3_tx_prcltype    ),
        .cxs_prcl3_tx_srcid        (cxs_prcl3_tx_srcid       ),
        .cxs_prcl3_tx_tgtid        (cxs_prcl3_tx_tgtid       ),
`ifdef CXSPRCL_LAST
        .cxs_prcl3_tx_last         (cxs_prcl3_tx_last        ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl3_tx_chk_data     (cxs_prcl3_tx_chk_data    ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl3_tx_chk_cntl     (cxs_prcl3_tx_chk_cntl    ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl3_tx_chk_prcltype (cxs_prcl3_tx_chk_prcltype),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl3_tx_chk_srcid    (cxs_prcl3_tx_chk_srcid   ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl3_tx_chk_tgtid    (cxs_prcl3_tx_chk_tgtid   ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl3_tx_chk_last     (cxs_prcl3_tx_chk_last    ),
`endif
`ifdef CXSPRCL_CHECKTYPE_ODD_BYTE_PARITY
        .cxs_prcl3_tx_chk_valid    (cxs_prcl3_tx_chk_valid   ),
`endif
        .cxs_prcl3_tx_crdrtn       (cxs_prcl3_tx_crdrtn      ),
        .cxs_prcl3_tx_crdgnt       (cxs_prcl3_tx_crdgnt      ),
        .cxs_prcl3_tx_activereq    (cxs_prcl3_tx_activereq   ),
        .cxs_prcl3_tx_activeack    (cxs_prcl3_tx_activeack   ),
        .cxs_prcl3_tx_deacthint    (cxs_prcl3_tx_deacthint   ),
`endif
        .gl_txclk_en               (tx_clken                ),
        .gl_rxclk_en               (rx_clken                ),
        .cxs_prcl_intr             (prcl_intr               )
    );
`endif

    // ==========================================================================
    //  模块例化: link_layer
    // ==========================================================================
    link_layer u_link_layer (
        .mb_clk                         (mb_clk                        ),
        .sb_clk                         (sb_clk                        ),
        .apb_clk                        (pclk                          ),
        .mb_rst_n                       (link_mb_rstn                  ),
        .sb_rst_n                       (sb_rstn_sync                  ),
        .apb_rst_n                      (link_apb_rstn                 ),
        .paif_clk                       (paif_clk                      ),
        .paif_rstn                      (paif_rstn                     ),
        .paif_tx_link0_data_i           (paif0_txdata                  ),
        .paif_tx_link0_valid_i          (paif0_txvalid                 ),
        .paif_tx_link0_ready_o          (paif0_txready                 ),
        .paif_rx_link0_data_o           (paif0_rxdata                  ),
        .paif_rx_link0_valid_o          (paif0_rxvalid                 ),
        .paif_rx_link0_ready_i          (paif0_rxready                 ),
`ifdef PRCL_TX_LANE_1
        .paif_tx_link1_data_i           (paif1_txdata                  ),
        .paif_tx_link1_valid_i          (paif1_txvalid                 ),
        .paif_tx_link1_ready_o          (paif1_txready                 ),
`endif
`ifdef PRCL_TX_LANE_2
        .paif_tx_link2_data_i           (paif2_txdata                  ),
        .paif_tx_link2_valid_i          (paif2_txvalid                 ),
        .paif_tx_link2_ready_o          (paif2_txready                 ),
`endif
`ifdef PRCL_TX_LANE_3
        .paif_tx_link3_data_i           (paif3_txdata                  ),
        .paif_tx_link3_valid_i          (paif3_txvalid                 ),
        .paif_tx_link3_ready_o          (paif3_txready                 ),
`endif
`ifdef PRCL_RX_LANE_1
        .paif_rx_link1_data_o           (paif1_rxdata                  ),
        .paif_rx_link1_valid_o          (paif1_rxvalid                 ),
        .paif_rx_link1_ready_i          (paif1_rxready                 ),
`endif
`ifdef PRCL_RX_LANE_2
        .paif_rx_link2_data_o           (paif2_rxdata                  ),
        .paif_rx_link2_valid_o          (paif2_rxvalid                 ),
        .paif_rx_link2_ready_i          (paif2_rxready                 ),
`endif
`ifdef PRCL_RX_LANE_3
        .paif_rx_link3_data_o           (paif3_rxdata                  ),
        .paif_rx_link3_valid_o          (paif3_rxvalid                 ),
        .paif_rx_link3_ready_i          (paif3_rxready                 ),
`endif
        .cpif_tx_link0_lane0_data_o     (cpif_tx_link0_lane0_data_o    ),
        .cpif_tx_link0_lane0_data_vld_o (cpif_tx_link0_lane0_data_vld_o),
`ifdef LL_LINK_LANE1
        .cpif_tx_link0_lane1_data_o     (cpif_tx_link0_lane1_data_o    ),
        .cpif_tx_link0_lane1_data_vld_o (cpif_tx_link0_lane1_data_vld_o),
`endif
`ifdef LL_LINK_LANE3
        .cpif_tx_link0_lane2_data_o     (cpif_tx_link0_lane2_data_o    ),
        .cpif_tx_link0_lane2_data_vld_o (cpif_tx_link0_lane2_data_vld_o),
        .cpif_tx_link0_lane3_data_o     (cpif_tx_link0_lane3_data_o    ),
        .cpif_tx_link0_lane3_data_vld_o (cpif_tx_link0_lane3_data_vld_o),
`endif
        .cpif_tx_link0_ready_i          (cpif_tx_link0_ready_i         ),
        .cpif_rx_link0_ready_i          (cpif_rx_link0_ready_i         ),
        .cpif_rx_link0_lane0_data_i     (cpif_rx_link0_lane0_data_i    ),
        .cpif_rx_link0_lane0_data_vld_i (cpif_rx_link0_lane0_data_vld_i),
`ifdef LL_LINK_LANE1
        .cpif_rx_link0_lane1_data_i     (cpif_rx_link0_lane1_data_i    ),
        .cpif_rx_link0_lane1_data_vld_i (cpif_rx_link0_lane1_data_vld_i),
`endif
`ifdef LL_LINK_LANE3
        .cpif_rx_link0_lane2_data_i     (cpif_rx_link0_lane2_data_i    ),
        .cpif_rx_link0_lane2_data_vld_i (cpif_rx_link0_lane2_data_vld_i),
        .cpif_rx_link0_lane3_data_i     (cpif_rx_link0_lane3_data_i    ),
        .cpif_rx_link0_lane3_data_vld_i (cpif_rx_link0_lane3_data_vld_i),
`endif
`ifdef PRCL_TX_LANE_1
        .cpif_tx_link1_lane0_data_o     (cpif_tx_link1_lane0_data_o    ),
        .cpif_tx_link1_lane0_data_vld_o (cpif_tx_link1_lane0_data_vld_o),
`ifdef LL_LINK_LANE1
        .cpif_tx_link1_lane1_data_o     (cpif_tx_link1_lane1_data_o    ),
        .cpif_tx_link1_lane1_data_vld_o (cpif_tx_link1_lane1_data_vld_o),
`endif
`ifdef LL_LINK_LANE3
        .cpif_tx_link1_lane2_data_o     (cpif_tx_link1_lane2_data_o    ),
        .cpif_tx_link1_lane2_data_vld_o (cpif_tx_link1_lane2_data_vld_o),
        .cpif_tx_link1_lane3_data_o     (cpif_tx_link1_lane3_data_o    ),
        .cpif_tx_link1_lane3_data_vld_o (cpif_tx_link1_lane3_data_vld_o),
`endif
        .cpif_tx_link1_ready_i          (cpif_tx_link1_ready_i         ),
`endif
`ifdef PRCL_RX_LANE_1
        .cpif_rx_link1_lane0_data_i     (cpif_rx_link1_lane0_data_i    ),
        .cpif_rx_link1_lane0_data_vld_i (cpif_rx_link1_lane0_data_vld_i),
`ifdef LL_LINK_LANE1
        .cpif_rx_link1_lane1_data_i     (cpif_rx_link1_lane1_data_i    ),
        .cpif_rx_link1_lane1_data_vld_i (cpif_rx_link1_lane1_data_vld_i),
`endif
`ifdef LL_LINK_LANE3
        .cpif_rx_link1_lane2_data_i     (cpif_rx_link1_lane2_data_i    ),
        .cpif_rx_link1_lane2_data_vld_i (cpif_rx_link1_lane2_data_vld_i),
        .cpif_rx_link1_lane3_data_i     (cpif_rx_link1_lane3_data_i    ),
        .cpif_rx_link1_lane3_data_vld_i (cpif_rx_link1_lane3_data_vld_i),
`endif
`endif
`ifdef PRCL_TX_LANE_2
        .cpif_tx_link2_lane0_data_o     (cpif_tx_link2_lane0_data_o    ),
        .cpif_tx_link2_lane0_data_vld_o (cpif_tx_link2_lane0_data_vld_o),
`ifdef LL_LINK_LANE1
        .cpif_tx_link2_lane1_data_o     (cpif_tx_link2_lane1_data_o    ),
        .cpif_tx_link2_lane1_data_vld_o (cpif_tx_link2_lane1_data_vld_o),
`endif
`ifdef LL_LINK_LANE3
        .cpif_tx_link2_lane2_data_o     (cpif_tx_link2_lane2_data_o    ),
        .cpif_tx_link2_lane2_data_vld_o (cpif_tx_link2_lane2_data_vld_o),
        .cpif_tx_link2_lane3_data_o     (cpif_tx_link2_lane3_data_o    ),
        .cpif_tx_link2_lane3_data_vld_o (cpif_tx_link2_lane3_data_vld_o),
`endif
        .cpif_tx_link2_ready_i          (cpif_tx_link2_ready_i         ),
`endif
`ifdef PRCL_TX_LANE_3
        .cpif_tx_link3_lane0_data_o     (cpif_tx_link3_lane0_data_o    ),
        .cpif_tx_link3_lane0_data_vld_o (cpif_tx_link3_lane0_data_vld_o),
`ifdef LL_LINK_LANE1
        .cpif_tx_link3_lane1_data_o     (cpif_tx_link3_lane1_data_o    ),
        .cpif_tx_link3_lane1_data_vld_o (cpif_tx_link3_lane1_data_vld_o),
`endif
`ifdef LL_LINK_LANE3
        .cpif_tx_link3_lane2_data_o     (cpif_tx_link3_lane2_data_o    ),
        .cpif_tx_link3_lane2_data_vld_o (cpif_tx_link3_lane2_data_vld_o),
        .cpif_tx_link3_lane3_data_o     (cpif_tx_link3_lane3_data_o    ),
        .cpif_tx_link3_lane3_data_vld_o (cpif_tx_link3_lane3_data_vld_o),
`endif
        .cpif_tx_link3_ready_i          (cpif_tx_link3_ready_i         ),
`endif
`ifdef PRCL_RX_LANE_2
        .cpif_rx_link2_lane0_data_i     (cpif_rx_link2_lane0_data_i    ),
        .cpif_rx_link2_lane0_data_vld_i (cpif_rx_link2_lane0_data_vld_i),
`ifdef LL_LINK_LANE1
        .cpif_rx_link2_lane1_data_i     (cpif_rx_link2_lane1_data_i    ),
        .cpif_rx_link2_lane1_data_vld_i (cpif_rx_link2_lane1_data_vld_i),
`endif
`ifdef LL_LINK_LANE3
        .cpif_rx_link2_lane2_data_i     (cpif_rx_link2_lane2_data_i    ),
        .cpif_rx_link2_lane2_data_vld_i (cpif_rx_link2_lane2_data_vld_i),
        .cpif_rx_link2_lane3_data_i     (cpif_rx_link2_lane3_data_i    ),
        .cpif_rx_link2_lane3_data_vld_i (cpif_rx_link2_lane3_data_vld_i),
`endif
`endif
`ifdef PRCL_RX_LANE_3
        .cpif_rx_link3_lane0_data_i     (cpif_rx_link3_lane0_data_i    ),
        .cpif_rx_link3_lane0_data_vld_i (cpif_rx_link3_lane0_data_vld_i),
`ifdef LL_LINK_LANE1
        .cpif_rx_link3_lane1_data_i     (cpif_rx_link3_lane1_data_i    ),
        .cpif_rx_link3_lane1_data_vld_i (cpif_rx_link3_lane1_data_vld_i),
`endif
`ifdef LL_LINK_LANE3
        .cpif_rx_link3_lane2_data_i     (cpif_rx_link3_lane2_data_i    ),
        .cpif_rx_link3_lane2_data_vld_i (cpif_rx_link3_lane2_data_vld_i),
        .cpif_rx_link3_lane3_data_i     (cpif_rx_link3_lane3_data_i    ),
        .cpif_rx_link3_lane3_data_vld_i (cpif_rx_link3_lane3_data_vld_i),
`endif
`endif
        .cpif_tx_mode_o                 (cpif_tx_mode_o                ),
        .cpif_tx_rst_n_o                (cpif_tx_rst_n_o               ),
        .cpif_tx_lpc_o                  (cpif_tx_lpc_o                 ),
        .cpif_rx_lpc_i                  (cpif_rx_lpc_i                 ),
        .cpif_rx_mode_o                 (cpif_rx_mode_o                ),
        .cpif_rx_rst_n_o                (cpif_rx_rst_n_o               ),
        .udi_tx_link0_lane0_data_i      (udi_tx_link0_lane0_data_i     ),
        .udi_tx_link0_lane0_data_vld_i  (udi_tx_link0_lane0_data_vld_i ),
`ifdef LL_LINK_LANE1
        .udi_tx_link0_lane1_data_i      (udi_tx_link0_lane1_data_i     ),
        .udi_tx_link0_lane1_data_vld_i  (udi_tx_link0_lane1_data_vld_i ),
`endif
`ifdef LL_LINK_LANE3
        .udi_tx_link0_lane2_data_i      (udi_tx_link0_lane2_data_i     ),
        .udi_tx_link0_lane2_data_vld_i  (udi_tx_link0_lane2_data_vld_i ),
        .udi_tx_link0_lane3_data_i      (udi_tx_link0_lane3_data_i     ),
        .udi_tx_link0_lane3_data_vld_i  (udi_tx_link0_lane3_data_vld_i ),
`endif
        .udi_tx_link0_ready_o           (udi_tx_link0_ready_o          ),
        .udi_rx_link0_lane0_data_o      (udi_rx_link0_lane0_data_o     ),
        .udi_rx_link0_lane0_data_vld_o  (udi_rx_link0_lane0_data_vld_o ),
`ifdef LL_LINK_LANE1
        .udi_rx_link0_lane1_data_o      (udi_rx_link0_lane1_data_o     ),
        .udi_rx_link0_lane1_data_vld_o  (udi_rx_link0_lane1_data_vld_o ),
`endif
`ifdef LL_LINK_LANE3
        .udi_rx_link0_lane2_data_o      (udi_rx_link0_lane2_data_o     ),
        .udi_rx_link0_lane2_data_vld_o  (udi_rx_link0_lane2_data_vld_o ),
        .udi_rx_link0_lane3_data_o      (udi_rx_link0_lane3_data_o     ),
        .udi_rx_link0_lane3_data_vld_o  (udi_rx_link0_lane3_data_vld_o ),
`endif
`ifdef PRCL_TX_LANE_1
        .udi_tx_link1_lane0_data_i      (udi_tx_link1_lane0_data_i     ),
        .udi_tx_link1_lane0_data_vld_i  (udi_tx_link1_lane0_data_vld_i ),
`ifdef LL_LINK_LANE1
        .udi_tx_link1_lane1_data_i      (udi_tx_link1_lane1_data_i     ),
        .udi_tx_link1_lane1_data_vld_i  (udi_tx_link1_lane1_data_vld_i ),
`endif
`ifdef LL_LINK_LANE3
        .udi_tx_link1_lane2_data_i      (udi_tx_link1_lane2_data_i     ),
        .udi_tx_link1_lane2_data_vld_i  (udi_tx_link1_lane2_data_vld_i ),
        .udi_tx_link1_lane3_data_i      (udi_tx_link1_lane3_data_i     ),
        .udi_tx_link1_lane3_data_vld_i  (udi_tx_link1_lane3_data_vld_i ),
`endif
        .udi_tx_link1_ready_o           (udi_tx_link1_ready_o          ),
`endif
`ifdef PRCL_TX_LANE_2
        .udi_tx_link2_lane0_data_i      (udi_tx_link2_lane0_data_i     ),
        .udi_tx_link2_lane0_data_vld_i  (udi_tx_link2_lane0_data_vld_i ),
`ifdef LL_LINK_LANE1
        .udi_tx_link2_lane1_data_i      (udi_tx_link2_lane1_data_i     ),
        .udi_tx_link2_lane1_data_vld_i  (udi_tx_link2_lane1_data_vld_i ),
`endif
`ifdef LL_LINK_LANE3
        .udi_tx_link2_lane2_data_i      (udi_tx_link2_lane2_data_i     ),
        .udi_tx_link2_lane2_data_vld_i  (udi_tx_link2_lane2_data_vld_i ),
        .udi_tx_link2_lane3_data_i      (udi_tx_link2_lane3_data_i     ),
        .udi_tx_link2_lane3_data_vld_i  (udi_tx_link2_lane3_data_vld_i ),
`endif
        .udi_tx_link2_ready_o           (udi_tx_link2_ready_o          ),
`endif
`ifdef PRCL_TX_LANE_3
        .udi_tx_link3_lane0_data_i      (udi_tx_link3_lane0_data_i     ),
        .udi_tx_link3_lane0_data_vld_i  (udi_tx_link3_lane0_data_vld_i ),
`ifdef LL_LINK_LANE1
        .udi_tx_link3_lane1_data_i      (udi_tx_link3_lane1_data_i     ),
        .udi_tx_link3_lane1_data_vld_i  (udi_tx_link3_lane1_data_vld_i ),
`endif
`ifdef LL_LINK_LANE3
        .udi_tx_link3_lane2_data_i      (udi_tx_link3_lane2_data_i     ),
        .udi_tx_link3_lane2_data_vld_i  (udi_tx_link3_lane2_data_vld_i ),
        .udi_tx_link3_lane3_data_i      (udi_tx_link3_lane3_data_i     ),
        .udi_tx_link3_lane3_data_vld_i  (udi_tx_link3_lane3_data_vld_i ),
`endif
        .udi_tx_link3_ready_o           (udi_tx_link3_ready_o          ),
`endif
`ifdef PRCL_RX_LANE_1
        .udi_rx_link1_lane0_data_o      (udi_rx_link1_lane0_data_o     ),
        .udi_rx_link1_lane0_data_vld_o  (udi_rx_link1_lane0_data_vld_o ),
`ifdef LL_LINK_LANE1
        .udi_rx_link1_lane1_data_o      (udi_rx_link1_lane1_data_o     ),
        .udi_rx_link1_lane1_data_vld_o  (udi_rx_link1_lane1_data_vld_o ),
`endif
`ifdef LL_LINK_LANE3
        .udi_rx_link1_lane2_data_o      (udi_rx_link1_lane2_data_o     ),
        .udi_rx_link1_lane2_data_vld_o  (udi_rx_link1_lane2_data_vld_o ),
        .udi_rx_link1_lane3_data_o      (udi_rx_link1_lane3_data_o     ),
        .udi_rx_link1_lane3_data_vld_o  (udi_rx_link1_lane3_data_vld_o ),
`endif
`endif
`ifdef PRCL_RX_LANE_2
        .udi_rx_link2_lane0_data_o      (udi_rx_link2_lane0_data_o     ),
        .udi_rx_link2_lane0_data_vld_o  (udi_rx_link2_lane0_data_vld_o ),
`ifdef LL_LINK_LANE1
        .udi_rx_link2_lane1_data_o      (udi_rx_link2_lane1_data_o     ),
        .udi_rx_link2_lane1_data_vld_o  (udi_rx_link2_lane1_data_vld_o ),
`endif
`ifdef LL_LINK_LANE3
        .udi_rx_link2_lane2_data_o      (udi_rx_link2_lane2_data_o     ),
        .udi_rx_link2_lane2_data_vld_o  (udi_rx_link2_lane2_data_vld_o ),
        .udi_rx_link2_lane3_data_o      (udi_rx_link2_lane3_data_o     ),
        .udi_rx_link2_lane3_data_vld_o  (udi_rx_link2_lane3_data_vld_o ),
`endif
`endif
`ifdef PRCL_RX_LANE_3
        .udi_rx_link3_lane0_data_o      (udi_rx_link3_lane0_data_o     ),
        .udi_rx_link3_lane0_data_vld_o  (udi_rx_link3_lane0_data_vld_o ),
`ifdef LL_LINK_LANE1
        .udi_rx_link3_lane1_data_o      (udi_rx_link3_lane1_data_o     ),
        .udi_rx_link3_lane1_data_vld_o  (udi_rx_link3_lane1_data_vld_o ),
`endif
`ifdef LL_LINK_LANE3
        .udi_rx_link3_lane2_data_o      (udi_rx_link3_lane2_data_o     ),
        .udi_rx_link3_lane2_data_vld_o  (udi_rx_link3_lane2_data_vld_o ),
        .udi_rx_link3_lane3_data_o      (udi_rx_link3_lane3_data_o     ),
        .udi_rx_link3_lane3_data_vld_o  (udi_rx_link3_lane3_data_vld_o ),
`endif
`endif
        .bridge_en_i                    (1'b0                          ), // bridge 模式暂不使能
        .lp_sw_cmd_i                    (lp_sw_cmd_prcl                ),
        .lp_sw_req_i                    (lp_sw_req_prcl                ),
        .lp_sw_ack_o                    (lp_sw_ack_sb                 ),
        .lp_sw_sts_o                    (lp_sw_sts_sb                 ),
        .cpif_tx_event_enable_o         (cpif_tx_event_enable_o        ),
        .cpif_tx_event_req_o            (cpif_tx_event_req_o           ),
        .cpif_tx_event_cmd_o            (cpif_tx_event_cmd_o           ),
        .cpif_tx_event_ack_i            (cpif_tx_event_ack_i           ),
        .cpif_tx_event_status_i         (cpif_tx_event_status_i        ),
        .cpif_rx_event_enable_o         (cpif_rx_event_enable_o        ),
        .cpif_rx_event_req_i            (cpif_rx_event_req_i           ),
        .cpif_rx_event_cmd_i            (cpif_rx_event_cmd_i           ),
        .cpif_rx_event_ack_o            (cpif_rx_event_ack_o           ),
        .cpif_rx_event_status_o         (cpif_rx_event_status_o        ),
        .paddr_i                        (link_paddr                    ),
        .psel_i                         (psel_to_link                  ),
        .penable_i                      (penable                       ),
        .pwrite_i                       (pwrite                        ),
        .pwdata_i                       (pwdata                        ),
        .pready_o                       (link_pready                   ),
        .prdata_o                       (link_prdata                   ),
        .pslverr_o                      (link_pslverr                  ),
        .irq_o                          (link_irq                      ),
        .dfx_stat_o                     (dfx_stat_o                    )
    );

endmodule

`default_nettype wire
