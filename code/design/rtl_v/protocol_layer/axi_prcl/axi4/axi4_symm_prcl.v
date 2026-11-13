// =============================================================================
// module name：axi4_symm_prcl
// author: OpenHappy Team
// MAS/spec：spec/protocol_layer/axi4_symm_prcl/axi4_symm_prcl_spec.md
// 功能概述: AXI4 对称模式协议层顶层桥（单条对称链路，无 req/resp 层级，扁平）。
//   一端挂 1×用户 AXI4 SLAVE（s_axi_*，收请求）+ 1×用户 AXI4 MASTER（m_axi_*，发请求/收响应），
//   另一端经单一 PAIF（1× paif_tx + 1× paif_rx，位宽 AXI_DW=512/1024）与对端 Die 互通。
//   TX：s_axi_*(AW/AR/W) 与 m_axi_*(R/B) 经 p_axi4_bond(s0/m0 槽)绑定 → 5 路 p_axis_async_fifo
//       (aclk→prcl_clk) → p_axi4_rwch_pack 封装 256Byte Flit → paif_tx。
//   RX：paif_rx → p_axi4_rwch_unpack 解封装 → 5 路 p_axis_async_fifo (prcl_clk→aclk)
//       → p_axi4_unbond(m0/s0 槽)解绑定为 m_axi_*(AW/AR/W) 与 s_axi_*(R/B)。
//   集中式 CSR：本模块例化 p_axi4_symm_csr（APB 唯一接入点 + intr + 回环使能 loopback sel），
//   单 clken 时钟门控（1 个 gating clk），并预留 axi_loopback / protocol_paif_loopback 回环例化点。
//   无 *_demux；pack/unpack 仅接 rp0 槽（ar_rp1~3 / b_rp1~3 输入 tie0、输出悬空）；
//   bond 仅 s0/m0 槽 active；credit_en 等 CSR 输出按对称桥模型分发/观测。
//   hipi_sel 由 CSR 0x600 配置（准静态）：bond 直连 csr_hipi_sel；prcl_clk 域 sync 后 hipi_sel_sync
//   双引擎选路（`ifdef HIPI_GB 条件编译）——0=HIPI Next（u_pack/u_unpack，prcl_clk_pack）/ 1=GB
//   （protocol_tx_packet→pkgpipe / protocol_rx_ch_identify，prcl_clk_gb）；paif_loop 出入口经
//   p_tx_mux2to1 / p_rx_demux1to2 切换；HIPI_GB 未定义时仅 HIPI Next 单通路。
//   远端 Credit 反压：unpack 解出的远端 Credit（aw/ar/w/r/b rp0）经 5 路 p_crd_tx_ctrl
//   门控 TX FIFO→pack 输出（enable=同步后的 credit_en）。
//   请求超时检测：s_axi AW/AR 请求经 p_req_timeout（AW→B 写超时 w0、AR→R 读超时 r0，aclk 域）
//   检测 outstanding 超时/错误，回报 CSR timeout/err id+intr 与 tx_w/r_ots_idle。
//   m_axi AW/AR 请求经 p_req_timeout（resp_asymm ots_chk 简化模式：enable tie0、id/intr/clr
//   悬空，仅 ots_idle）检测 outstanding，回报 CSR rx_w/r_ots_idle。
//   溢出检测：RX FIFO 写侧经 5 路 p_ovrflow_det 检测（~ready & valid），回报 CSR
//   aw0/ar0/w0/r0/b0 ovrflow_intr（clr 经 sync_cell 同步）。
//   lane idle 观测：tx_lane_idle=~pack_busy、rx_lane_idle=~unpack_busy 打拍寄存（不再输出 busy 端口）。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   2026-09-22  OpenHappy Team  p_axi4_unbond 删除 hipi_sel 端口后：i_axi4_unbond 实例移除 .hipi_sel(1'b0) 连接；bond 实例保留 tie 0。

//   2026-09-22  OpenHappy Team  修复 credit 握手：5 路 p_crd_gen 的 m_ready 由绑定 0 改接 pack 的 {aw,w,ar,r,b}crd_rp0_ready
//                       （原状 crd_cnt_r 只累加不扣减、复位满额 ⇒ m_crd 恒 2'b11 无限发 credit 且 6bit 计数器溢出）
//   2026-08-30  OpenHappy Team  初始版本：单链路对称桥核心数据通路（bond→async_fifo→rwch_pack→paif_tx /
//   2026-09-23  OpenHappy Team  新增 HIPI GB 双引擎通路（hipi_sel CSR 0x600 模式选择）：① csr_hipi_sel → sync_cell(prcl_clk)
//                       → hipi_sel_sync；② prcl_clk_cg 下增设 2 个 ICG：prcl_clk_pack(E=~hipi_sel_sync)→u_pack/u_unpack、
//                       prcl_clk_gb(E=hipi_sel_sync)→protocol_tx_packet/protocol_tx_pkgpipe/protocol_rx_ch_identify；
//                       ③ TX：pack_aw/ar/w/r/b + credit 扇出至 tx_packet，上游 ready/credit-ready 按 hipi_sel_sync 选
//                       pack/tx_packet，pack 与 pkgpipe 输出经 p_tx_mux2to1(sel=hipi_sel_sync) 入 paif_loop s0；
//                       ④ RX：paif_loop m1 经 p_rx_demux1to2(sel=hipi_sel_sync) 分流 u_unpack/protocol_rx_ch_identify，
//                       两引擎同类通道 bus/valid 带模式门控相或合并 1 路入 RX FIFO / crd_tx_ctrl；
//                       ⑤ CSR 接口更新：bond.hipi_sel=csr_hipi_sel，ch_identify pkt_err/rx_st/err_intr→CSR、
//                       CSR gb_err_clr→ch_identify.err_clr。
//   2026-09-23  OpenHappy Team  回环语义与接线纠正：p_paif_loop=本端环回（local_loop，s0→m1 内回环绕过 Link）、
//                       p_axi_loop=远端环回（remote_loop，对端数据 s1→回灌 m0 发回对端）；u_axi_loop/u_paif_loop
//                       的 sel 对调（u_axi_loop←remote_loop_sync、u_paif_loop←local_loop_sync），两路同步目标域
//                       对调（local_loop_sync→prcl_clk、remote_loop_sync→aclk，各入自身回环模块数据域），回环注释同步纠正。
//   2026-09-23  OpenHappy Team  paif bus 位宽对齐 PAIF_DW：顶层 paif_txdata/paif_rxdata 口、pack_txdata/unpack_rxdata、
//                       p_paif_loop 参数（原 AXI_DW）及 u_tx_mux/u_rx_demux DATA_W 均改 PAIF_DW。
//   2026-09-23  OpenHappy Team  设计约束登记：GB 通路仅在 PAIF_DW=512 下启用（pkgpipe 定宽 512）；PAIF_DW=1024 时无 GB 通路。

//   2026-09-23  OpenHappy Team  GB 通路 `ifdef HIPI_GB 条件编译：2 个 ICG / hipi_sel sync / u_tx_mux / u_rx_demux /
//                       protocol_tx_packet / protocol_tx_pkgpipe / protocol_rx_ch_identify / ready 选择 / bus 合并均在
//                       HIPI_GB 定义时存在；未定义时仅 HIPI Next 单通路（pack/unpack 时钟回 prcl_clk_cg、
//                       loop 直连 pack/unpack、FIFO/crd_tx_ctrl 直连 unp_*/rem_*；另修复 ar/b 两路 FIFO
//                       误接 mrg_ar0/mrg_b0 隐式线的问题）。
//   2026-09-23  OpenHappy Team  lint 修复：① tx_*_crd 线宽 6→TX_FIFO_CRD_W=4（p_axis_async_fifo crd 输出位宽，W110×5）；
//                       ② p_axi4_symm_csr timeout_err_clr 装载拼接 14→17bit（STARC-2.10.3.2b/W164b）。





//                       paif_rx→rwch_unpack→async_fifo→unbond）+ 集中式 CSR + 单 clken 门控 +
//                       crd_gen(rp0 credit) + 回环例化点。
//   2026-08-30  OpenHappy Team  集成更新：① hipi_sel 固定 0 去除 input（bond/unbond tie0）；
//                       ② 集成 p_crd_tx_ctrl（unpack 远端 Credit 门控 5 路 TX）；
//                       ③ 集成 p_req_timeout（s_axi AW/AR 超时检测→CSR timeout/err + tx_ots_idle）；
//                       ④ tx/rx_lane_idle 由 pack/unpack busy 观测产生，移除 busy 输出端口。
//   2026-08-30  OpenHappy Team  集成更新 II：① m_axi AW/AR 补 p_req_timeout ots_chk（resp_asymm 简化模式，
//                       仅 ots_idle→CSR rx_w/r_ots_idle，插 RX FIFO→unbond 间）；
//                       ② 例化 5 路 p_ovrflow_det（RX FIFO 写侧溢出→CSR aw0/ar0/w0/r0/b0 intr）；
//                       ③ tx/rx_lane_idle 改时序打拍（posedge prcl_clk_cg，复位 1，仿 axi4_asymm）。
//   2026-08-30  OpenHappy Team  集成更新 III（回环）：① protocol_paif_loopback 定宽化改造为 p_paif_loop
//                       （parameter PAIF_DW、删 strb/fltcnt、rstn、更名，见 common/p_paif_loop.v）；
//                       ② axi_loopback 更名 p_axi_loop（见 common/p_axi_loop.v）；
//                       ③ remote_loop=p_paif_loop 例化入 PAIF 侧（sel=remote_loop_sync，PAIF_DW=AXI_DW）；
//                       ④ local_loop=p_axi_loop 例化入 bond 层（sel=local_loop_sync，桥接
//                       bond→TX FIFO 与 RX FIFO→unbond/ots_chk，W/R lite 外部复用）。
// =============================================================================
// NOTE(设计约束): ① 对称桥模型远端 Credit 反压由 p_crd_tx_ctrl 提供（unpack 远端 Credit 消费），
//                本端授予对端的本地 Credit 由 crd_gen 读对应 RX async FIFO 占用产生，
//                统一 SP/b_vc（见 spec §5.3、§5.5）。
//                ② pack/unpack 的 rp1~3 数据通道（ar_rp*_bus/b_rp*_bus）输入 tie0、对应 *_ready
//                输出悬空——单用户口仅需 rp0 槽（spec §5.5）。
//                ③ 回环已集成（2026-09-23 语义纠正：paif loop=本端环回 / axi loop=远端环回）：
//                local_loop= p_paif_loop（PAIF 层，prcl_clk_cg 域，sel=local_loop_sync），
//                remote_loop= p_axi_loop（bond 层，aclk 域，sel=remote_loop_sync）；
//                sel 由 CSR 0x500 经 2FF 同步入各自数据域（见 CDC 段）。
// =============================================================================
// FIXME(集成): ① local_loop 的 W/R lite 位已由 p_axi_loop 提供 lite 通道（s0/s1/m0/m1 各
//               *_w_lite/*_r_lite）；但 RX 数据通路（RX FIFO/unbond）不复原 lite，故回环
//               （sel=1）时 s1_w/r_lite 接 0、m0_w/r_lite=0，仍仅支持非 lite 回环自测；
//               如需 lite 覆盖需在 RX 通路恢复 lite 后接入 s1_w/r_lite。
// =============================================================================


module axi4_symm_prcl #(
    parameter AXI_DW  = `AXI_DW,
    parameter PAIF_DW = `PAIF_DW
) (
    // ==========================================================================
    // 公共接口（恒在）
    // ==========================================================================
    // 时钟与复位（user AXI 域 / 协议域 / APB 域）
    input  wire        aclk,
    input  wire        aresetn,
    input  wire        prcl_clk,
    input  wire        prcl_rstn,
    input  wire        pclk,
    input  wire        presetn,
    // 协议层对外时钟门控使能（1bit，单 gating clk = prcl_clk）
    input  wire        clken,
    // 低功耗透传（ctrl ↔ sw 直通，同 axi4_asymm 顶层约定）
    input  wire [3:0]  lp_ctrl_cmd,  input  wire        lp_ctrl_req,
    output wire        lp_ctrl_ack,  output wire [3:0] lp_ctrl_sts,
    output wire [3:0]  lp_sw_cmd,    output wire        lp_sw_req,
    input  wire        lp_sw_ack,    input  wire [3:0] lp_sw_sts,
    // 集中式 CSR（APB 唯一接入点）
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [11:0] paddr,
    input  wire [31:0] pwdata,
    output wire [31:0] prdata,
    output wire        pready,
    output wire        pslverr,

    // ==========================================================================
    // 用户侧 AXI4 Slave（s_axi_*，单口；AW/AR/W 收请求入，R/B 响应出）
    // ==========================================================================
    // AW 通道
    input  wire [9:0]  s_axi_awid,   input  wire [63:0] s_axi_awaddr,
    input  wire [7:0]  s_axi_awlen,  input  wire [2:0]  s_axi_awsize,
    input  wire [1:0]  s_axi_awburst,input  wire        s_axi_awlock,
    input  wire [3:0]  s_axi_awcache,input  wire [2:0]  s_axi_awprot,
    input  wire [3:0]  s_axi_awqos,  input  wire [3:0]  s_axi_awregion,
    input  wire [3:0]  s_axi_awuser, input  wire        s_axi_awvalid,
    output wire        s_axi_awready,
    // AR 通道
    input  wire [9:0]  s_axi_arid,   input  wire [63:0] s_axi_araddr,
    input  wire [7:0]  s_axi_arlen,  input  wire [2:0]  s_axi_arsize,
    input  wire [1:0]  s_axi_arburst,input  wire        s_axi_arlock,
    input  wire [3:0]  s_axi_arcache,input  wire [2:0]  s_axi_arprot,
    input  wire [3:0]  s_axi_arqos,  input  wire [3:0]  s_axi_arregion,
    input  wire [3:0]  s_axi_aruser, input  wire        s_axi_arvalid,
    output wire        s_axi_arready,
    // W 通道
    input  wire [AXI_DW-1:0]   s_axi_wdata, input wire [AXI_DW/8-1:0] s_axi_wstrb,
    input  wire [3:0]          s_axi_wuser, input wire                 s_axi_wlast,
    input  wire                s_axi_wvalid,
    output wire                s_axi_wready,
    // R 通道（output）
    output wire [9:0]          s_axi_rid,   output wire [AXI_DW-1:0] s_axi_rdata,
    output wire [1:0]          s_axi_rresp, output wire [3:0]        s_axi_ruser,
    output wire                s_axi_rlast, output wire               s_axi_rvalid,
    input  wire                s_axi_rready,
    // B 通道（output）
    output wire [9:0]          s_axi_bid,   output wire [1:0] s_axi_bresp,
    output wire [3:0]          s_axi_buser, output wire        s_axi_bvalid,
    input  wire                s_axi_bready,

    // ==========================================================================
    // 用户侧 AXI4 Master（m_axi_*，单口；AW/AR/W 请求出，R/B 响应入）
    // ==========================================================================
    // AW 通道（output）
    output wire [9:0]          m_axi_awid,   output wire [63:0] m_axi_awaddr,
    output wire [7:0]          m_axi_awlen,  output wire [2:0]  m_axi_awsize,
    output wire [1:0]          m_axi_awburst,output wire        m_axi_awlock,
    output wire [3:0]          m_axi_awcache,output wire [2:0]  m_axi_awprot,
    output wire [3:0]          m_axi_awqos,  output wire [3:0]  m_axi_awregion,
    output wire [3:0]          m_axi_awuser, output wire        m_axi_awvalid,
    input  wire                m_axi_awready,
    // AR 通道（output）
    output wire [9:0]          m_axi_arid,   output wire [63:0] m_axi_araddr,
    output wire [7:0]          m_axi_arlen,  output wire [2:0]  m_axi_arsize,
    output wire [1:0]          m_axi_arburst,output wire        m_axi_arlock,
    output wire [3:0]          m_axi_arcache,output wire [2:0]  m_axi_arprot,
    output wire [3:0]          m_axi_arqos,  output wire [3:0]  m_axi_arregion,
    output wire [3:0]          m_axi_aruser, output wire        m_axi_arvalid,
    input  wire                m_axi_arready,
    // W 通道（output）
    output wire [AXI_DW-1:0]   m_axi_wdata,  output wire [AXI_DW/8-1:0] m_axi_wstrb,
    output wire [3:0]          m_axi_wuser,  output wire                 m_axi_wlast,
    output wire                m_axi_wvalid,
    input  wire                m_axi_wready,
    // R 通道（input）
    input  wire [9:0]          m_axi_rid,    input  wire [AXI_DW-1:0] m_axi_rdata,
    input  wire [1:0]          m_axi_rresp,  input  wire [3:0]        m_axi_ruser,
    input  wire                m_axi_rlast,  input  wire               m_axi_rvalid,
    output wire                m_axi_rready,
    // B 通道（input）
    input  wire [9:0]          m_axi_bid,    input  wire [1:0] m_axi_bresp,
    input  wire [3:0]          m_axi_buser,  input  wire        m_axi_bvalid,
    output wire                m_axi_bready,

    // ==========================================================================
    // PAIF / LINK 侧（单一 PAIF，prcl_clk 域，与对端握手）
    // ==========================================================================
    output wire [PAIF_DW-1:0] paif_txdata,   // 2026-09-23：paif bus 位宽对齐 PAIF_DW
    output wire             paif_txvalid,
    input  wire             paif_txready,
    input  wire [PAIF_DW-1:0] paif_rxdata,   // 2026-09-23：paif bus 位宽对齐 PAIF_DW
    input  wire             paif_rxvalid,
    output wire             paif_rxready,

    // ==========================================================================
    // 状态 / 时钟输出
    // ==========================================================================
    output wire        paif_clk,
    output wire        paif_rstn,
    // 中断合并输出
    output wire        intr
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    localparam AX_WIDTH    = 107;                                   // AW/AR bond bus 位宽
    localparam W_WIDTH     = AXI_DW + AXI_DW/8 + 5;                 // W bond bus 位宽 {wuser,wlast,wstrb,wdata}
    localparam R_WIDTH     = AXI_DW + 17;                           // R bond bus 位宽 {ruser,rlast,rid,rresp,rdata}
    localparam B_WIDTH     = 16;                                    // B bond bus 位宽 {buser,bresp,bid}
    localparam PK_W_WIDTH  = W_WIDTH;                               // W 打包位宽（与 bond 一致）
    localparam PK_R_WIDTH  = R_WIDTH;                               // R 打包位宽
    localparam FIFO_CRD_W  = $clog2(`PRCL_ASYNC_RXFIFO_DEPTH + 1); // RX FIFO credit 位宽（DEPTH=32→6）
    localparam TX_FIFO_CRD_W = $clog2(`PRCL_ASYNC_TXFIFO_DEPTH + 1); // TX FIFO credit 位宽（DEPTH=8→4，lint W110 修复）

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================

    // --------------------------------------------------------------------------
    // CDC 基础设施：prcl_rstn 同步、clken → ICG 门控 prcl_clk、credit_en 同步
    // --------------------------------------------------------------------------
    wire paif_rstn_i;          // prcl_rstn 经 rstn_sync 同步到 prcl_clk 域
    wire prcl_clk_cg;          // 单路门控 prcl_clk（E=clken_sync）
    wire clken_sync;           // clken 同步到 prcl_clk 域
    wire credit_en_int;        // CSR credit_en 同步到 prcl_clk 域（供 crd_gen/crd_tx_ctrl 消费）

    // --------------------------------------------------------------------------
    // CSR 内部连接线（p_axi4_symm_csr 配置输出 → loopback/crd；状态输入 ← 观测/tie）
    // --------------------------------------------------------------------------
    wire csr_credit_en;        // 0x414 credit_en
    wire csr_timeout_en;       // 0x428 timeout_en（→ p_req_timeout enable）
    wire [15:0] csr_timeout_num;
    wire csr_w0_timeout_clr, csr_r0_timeout_clr;
    wire csr_w0_err_clr,     csr_r0_err_clr;
    wire csr_r0_ovrflow_clr, csr_b0_ovrflow_clr;
    wire csr_aw0_ovrflow_clr, csr_ar0_ovrflow_clr, csr_w0_ovrflow_clr;
    wire csr_remote_loop;      // 0x500 remote_loop（PAIF 回环 sel）
    wire csr_local_loop;       // 0x500 local_loop（AXI bond 回环 sel）
    // 回环 sel 同步（CSR pclk 域 → 数据域）：
    //   local_loop  → aclk（bond / TX·RX FIFO aclk 侧）
    //   remote_loop → prcl_clk_cg（PAIF pack/unpack）
    wire local_loop_sync, remote_loop_sync;
    // CSR 状态输入（req_timeout 上报 + pack/unpack busy 观测 + ovrflow_det 上报；见下方 assign）
    wire rx_r_ots_idle_t, rx_w_ots_idle_t, tx_r_ots_idle_t, tx_w_ots_idle_t;
    wire tx_lane_idle_t, rx_lane_idle_t;
    wire [9:0] w0_timeout_id_t, r0_timeout_id_t, w0_err_id_t, r0_err_id_t;
    wire w0_timeout_intr_t, r0_timeout_intr_t, w0_err_intr_t, r0_err_intr_t;
    wire r0_ovrflow_intr_t, b0_ovrflow_intr_t, aw0_ovrflow_intr_t, ar0_ovrflow_intr_t, w0_ovrflow_intr_t;

    // --------------------------------------------------------------------------
    // p_req_timeout（s_axi AW/AR 请求超时检测，aclk 域；插在用户 AXI 与 bond 之间）
    // --------------------------------------------------------------------------
    wire aw_rt_mvalid, aw_rt_mready;   // AW req_timeout m 侧（→ bond s0_axi_aw）
    wire ar_rt_mvalid, ar_rt_mready;   // AR req_timeout m 侧（→ bond s0_axi_ar）
    wire aw_ots_idle,  ar_ots_idle;    // AW/AR outstanding idle（→ CSR tx_w/r_ots_idle）
    wire [9:0] aw_timeout_id, ar_timeout_id;
    wire aw_timeout_intr, ar_timeout_intr;
    wire [9:0] aw_err_id, ar_err_id;
    wire aw_err_intr, ar_err_intr;

    // --------------------------------------------------------------------------
    // p_req_timeout（m_axi AW/AR ots_chk，aclk 域；插在 RX FIFO 输出与 unbond 之间）
    //   resp_asymm 简化模式：enable tie 0、id/intr/clr 悬空，仅 ots_idle 上报
    // --------------------------------------------------------------------------
    wire m_aw_ots_mvalid, m_aw_ots_mready;   // AW ots_chk m 侧（→ unbond aw0_rx）
    wire m_ar_ots_mvalid, m_ar_ots_mready;   // AR ots_chk m 侧（→ unbond ar0_rx）
    wire m_aw_ots_idle,   m_ar_ots_idle;     // m_axi AW/AR outstanding idle（→ CSR rx_w/r_ots_idle）

    // --------------------------------------------------------------------------
    // p_ovrflow_det（RX FIFO 写侧溢出检测，prcl_clk_cg 域；→ CSR ovrflow intr）
    // --------------------------------------------------------------------------
    wire aw0_ovf_intr, ar0_ovf_intr, w0_ovf_intr, r0_ovf_intr, b0_ovf_intr;

    // --------------------------------------------------------------------------
    // lane idle 打拍寄存器（~pack_busy/~unpack_busy，prcl_clk_cg 域，复位 1）
    // --------------------------------------------------------------------------
    reg tx_lane_idle_r, rx_lane_idle_r;

    // --------------------------------------------------------------------------
    // p_crd_tx_ctrl（TX FIFO 输出 → pack 输入之间的 Credit 反压，prcl_clk_cg 域）
    //   远端 Credit 由 unpack 解出（rp0），经 crd_tx_ctrl 门控 5 路 TX 通道
    // --------------------------------------------------------------------------
    wire [1:0] rem_aw_crd, rem_ar_crd, rem_w_crd, rem_r_crd, rem_b_crd;
    wire rem_aw_crd_v, rem_ar_crd_v, rem_w_crd_v, rem_r_crd_v, rem_b_crd_v;

    // --------------------------------------------------------------------------
    // HIPI GB 双引擎（`ifdef HIPI_GB 条件编译；2026-09-23 新增）：
    //   hipi_sel CSR → prcl_clk sync；模式门控选路 0=HIPI Next / 1=GB
    // --------------------------------------------------------------------------
`ifdef HIPI_GB
    wire        csr_hipi_sel;      // CSR 0x600 hipi_sel（pclk 域，准静态；bond 直连 + sync）
    wire        hipi_sel_sync;     // prcl_clk 域 2FF 同步结果
    wire        prcl_clk_pack;     // ICG(prcl_clk_cg, E=~hipi_sel_sync) → u_pack/u_unpack
    wire        prcl_clk_gb;       // ICG(prcl_clk_cg, E=hipi_sel_sync)  → GB 三级
    wire        csr_gb_err_clr;    // CSR gb_intr_clr W1C → ch_identify.err_clr
    wire [767:0]      gb_pkt_data;  wire [2:0] gb_pkt_valid;  wire gb_pkt_ready;
    wire [PAIF_DW-1:0] gb_paif_data; wire gb_paif_valid;      wire gb_paif_ready;   // pkgpipe 输出定宽 512（GB 通路仅 PAIF_DW=512 启用，见下）
    wire txpkt_aw_ready, txpkt_ar_ready, txpkt_w_ready, txpkt_r_ready, txpkt_b_ready;
    wire txpkt_awcrd_ready, txpkt_arcrd_ready, txpkt_wcrd_ready, txpkt_rcrd_ready, txpkt_bcrd_ready;
    wire awcrd_up_ready, arcrd_up_ready, wcrd_up_ready, rcrd_up_ready, bcrd_up_ready;
    wire [AX_WIDTH-1:0] chi_aw_bus;  wire chi_aw_valid;
    wire [AX_WIDTH-1:0] chi_ar_bus;  wire chi_ar_valid;
    wire [W_WIDTH-1:0]  chi_w_bus;   wire chi_w_valid;
    wire [R_WIDTH-1:0]  chi_r_bus;   wire chi_r_valid;
    wire [B_WIDTH-1:0]  chi_b_bus;   wire chi_b_valid;
    wire [1:0] chi_awcrd, chi_arcrd, chi_wcrd, chi_rcrd, chi_bcrd;
    wire chi_awcrd_v, chi_arcrd_v, chi_wcrd_v, chi_rcrd_v, chi_bcrd_v;
    wire [2:0] chi_pkt_err;  wire [3:0] chi_rx_st;  wire chi_err_intr;
    wire [PAIF_DW-1:0] gb_rx_data;   wire gb_rx_valid;  wire gb_rx_ready;
    wire [PAIF_DW-1:0] loop_s0_data; wire loop_s0_valid; wire loop_s0_ready;
    wire [PAIF_DW-1:0] loop_m1_data; wire loop_m1_valid; wire loop_m1_ready;
    wire [AX_WIDTH-1:0] mrg_aw_bus;  wire mrg_aw_valid;
    wire [AX_WIDTH-1:0] mrg_ar_bus;  wire mrg_ar_valid;
    wire [W_WIDTH-1:0]  mrg_w_bus;   wire mrg_w_valid;
    wire [R_WIDTH-1:0]  mrg_r_bus;   wire mrg_r_valid;
    wire [B_WIDTH-1:0]  mrg_b_bus;   wire mrg_b_valid;
    wire [1:0] mrg_aw_crd, mrg_ar_crd, mrg_w_crd, mrg_r_crd, mrg_b_crd;
    wire mrg_aw_crd_v, mrg_ar_crd_v, mrg_w_crd_v, mrg_r_crd_v, mrg_b_crd_v;
`endif
    wire aw_crd_m_valid, aw_crd_m_ready;
    wire ar_crd_m_valid, ar_crd_m_ready;
    wire w_crd_m_valid,  w_crd_m_ready;
    wire r_crd_m_valid,  r_crd_m_ready;
    wire b_crd_m_valid,  b_crd_m_ready;

    // --------------------------------------------------------------------------
    // bond（s0/m0 槽）总线
    // --------------------------------------------------------------------------
    wire [AX_WIDTH-1:0] bond_aw0_bus;
    wire                bond_aw0_valid, bond_aw0_ready;
    wire [AX_WIDTH-1:0] bond_ar0_bus;
    wire                bond_ar0_valid, bond_ar0_ready;
    wire [W_WIDTH-1:0]  bond_w0_bus;
    wire                bond_w0_valid, bond_w0_ready;
    wire                bond_w0_lite;
    wire [R_WIDTH-1:0]  bond_r0_bus;
    wire                bond_r0_valid, bond_r0_ready;
    wire                bond_r0_lite;
    wire [B_WIDTH-1:0]  bond_b0_bus;
    wire                bond_b0_valid, bond_b0_ready;
    // bond s1~3 / m1~3 槽 ready 输出（悬空？见 bond 例化：未用槽 ready 输出留空）

    // --------------------------------------------------------------------------
    // TX async FIFO（aclk → prcl_clk_cg）
    // --------------------------------------------------------------------------
    wire [AX_WIDTH-1:0]     tx_aw_mdata;  wire tx_aw_mvalid, tx_aw_mready, tx_aw_sready;
    wire [AX_WIDTH-1:0]     tx_ar_mdata;  wire tx_ar_mvalid, tx_ar_mready, tx_ar_sready;
    wire [PK_W_WIDTH:0]     tx_w_sdata, tx_w_mdata;   // +1：w_lite 作最高 bit
    wire tx_w_mvalid, tx_w_mready, tx_w_sready;
    wire [PK_R_WIDTH:0]     tx_r_sdata, tx_r_mdata;   // +1：r_lite 作最高 bit
    wire tx_r_mvalid, tx_r_mready, tx_r_sready;
    wire [B_WIDTH-1:0]      tx_b_mdata;  wire tx_b_mvalid, tx_b_mready, tx_b_sready;
    // TX FIFO credit（aclk 写侧，当前未消费 → 悬空）
    wire [TX_FIFO_CRD_W-1:0] tx_aw_crd, tx_ar_crd, tx_w_crd, tx_r_crd, tx_b_crd;   // TX FIFO crd 输出位宽（DEPTH=8→4）
    wire tx_aw_crd_v, tx_ar_crd_v, tx_w_crd_v, tx_r_crd_v, tx_b_crd_v;

    // 打包为 pack 输入（prcl_clk 域）
    wire [AX_WIDTH-1:0]     pack_aw_bus;
    wire                    pack_aw_valid, pack_aw_ready;
    wire [AX_WIDTH-1:0]     pack_ar0_bus;
    wire                    pack_ar0_valid, pack_ar0_ready;
    wire [W_WIDTH-1:0]      pack_w_bus;
    wire                    pack_w_lite, pack_w_valid, pack_w_ready;
    wire [R_WIDTH-1:0]      pack_r_bus;
    wire                    pack_r_lite, pack_r_valid, pack_r_ready;
    wire [B_WIDTH-1:0]      pack_b0_bus;
    wire                    pack_b0_valid, pack_b0_ready;

    // --------------------------------------------------------------------------
    // unpack 输出（prcl_clk 域）
    // --------------------------------------------------------------------------
    wire [AX_WIDTH-1:0]     unp_aw_bus;  wire unp_aw_valid;
    wire [AX_WIDTH-1:0]     unp_ar0_bus; wire unp_ar0_valid;
    wire [W_WIDTH-1:0]      unp_w_bus;   wire unp_w_valid;
    wire [R_WIDTH-1:0]      unp_r_bus;   wire unp_r_valid;
    wire [B_WIDTH-1:0]      unp_b0_bus;  wire unp_b0_valid;

    // --------------------------------------------------------------------------
    // RX async FIFO（prcl_clk_cg → aclk）
    // --------------------------------------------------------------------------
    wire [AX_WIDTH-1:0]     rx_aw_mdata;  wire rx_aw_mvalid, rx_aw_mready, rx_aw_sready;
    wire [AX_WIDTH-1:0]     rx_ar_mdata;  wire rx_ar_mvalid, rx_ar_mready, rx_ar_sready;
    wire [W_WIDTH-1:0]      rx_w_mdata;   wire rx_w_mvalid,  rx_w_mready,  rx_w_sready;
    wire [R_WIDTH-1:0]      rx_r_mdata;   wire rx_r_mvalid,  rx_r_mready,  rx_r_sready;
    wire [B_WIDTH-1:0]      rx_b_mdata;   wire rx_b_mvalid,  rx_b_mready,  rx_b_sready;
    // RX FIFO credit（prcl_clk_cg 写侧，供 crd_gen 生成本地 credit）
    wire [FIFO_CRD_W-1:0]   rx_aw_crd, rx_ar_crd, rx_w_crd, rx_r_crd, rx_b_crd;
    wire rx_aw_crd_v, rx_ar_crd_v, rx_w_crd_v, rx_r_crd_v, rx_b_crd_v;

    // 本地 credit（crd_gen 输出，→ pack rp0 credit 输入）
    wire [1:0] loc_awcrd_rp0, loc_wcrd_rp0, loc_arcrd_rp0, loc_rcrd_rp0, loc_bcrd_rp0;

    // pack credit 发射 ack（→ crd_gen m_ready；2026-09-22 补握手：pack 本拍真实装入 beat 时扣减计数）
    wire pack_awcrd0_ready, pack_wcrd0_ready, pack_arcrd0_ready, pack_rcrd0_ready, pack_bcrd0_ready;

    // --------------------------------------------------------------------------
    // pack / unpack 状态
    // --------------------------------------------------------------------------
    wire pack_busy, unpack_busy;
    wire [PAIF_DW-1:0] pack_txdata; wire pack_txvalid, pack_txready;      // 2026-09-23：paif bus = PAIF_DW
    wire [PAIF_DW-1:0] unpack_rxdata; wire unpack_rxvalid, unpack_rxready;  // 2026-09-23：paif bus = PAIF_DW

    // --------------------------------------------------------------------------
    // 回环（LOOPBACK）信号（2026-09-23 语义纠正：paif loop=本端环回 / axi loop=远端环回）
    //   remote_loop（p_axi_loop，bond 层，aclk 域）：s0=bond TX → m0=TX FIFO；
    //     s1=RX FIFO → m1=unbond/ots_chk。sel=remote_loop_sync（远端环回：对端发来的数据
    //     经 s1 回灌 m0 发回对端，m1 关闭）。
    //   local_loop（p_paif_loop，PAIF 层，prcl_clk_cg 域）：sel=local_loop_sync，见 PAIF 侧段
    //     （本端环回：s0→m1 内回环绕过 Link）。
    // --------------------------------------------------------------------------
    // remote_loop 桥接线（p_axi_loop m0/m1 侧 → TX FIFO / ots_chk / unbond）
    wire [AX_WIDTH-1:0] il_aw_tx_bus, il_ar_tx_bus;      // m0 数据 → TX FIFO
    wire                il_aw_tx_valid, il_ar_tx_valid;  // m0 valid → TX FIFO
    wire [W_WIDTH-1:0]  il_w_tx_bus;
    wire                il_w_tx_valid;
    wire [R_WIDTH-1:0]  il_r_tx_bus;
    wire                il_r_tx_valid;
    wire [B_WIDTH-1:0]  il_b_tx_bus;
    wire                il_b_tx_valid;
    wire [AX_WIDTH-1:0] il_aw_rx_bus, il_ar_rx_bus;      // m1 数据 → ots_chk/unbond
    wire                il_aw_rx_valid, il_ar_rx_valid;  // m1 valid → ots_chk
    wire                il_aw_rx_ready, il_ar_rx_ready;  // m1 ready ← ots_chk s 侧
    wire [W_WIDTH-1:0]  il_w_rx_bus;                     // m1 数据 → unbond
    wire                il_w_rx_valid, il_w_rx_ready;    // m1 valid/ready ← unbond
    wire [R_WIDTH-1:0]  il_r_rx_bus;
    wire                il_r_rx_valid, il_r_rx_ready;
    wire [B_WIDTH-1:0]  il_b_rx_bus;
    wire                il_b_rx_valid, il_b_rx_ready;
    // TX W/R lite 位（p_axi_loop m0 输出 → tx_w/r_sdata 最高 bit；回环时 s1 无 lite 置 0）
    wire                il_w_lit_tx, il_r_lit_tx;   // 回环 m0 W/R lite 输出（→ tx_w/r_sdata 最高 bit）

    // ==========================================================================
    // CDC 基础设施子模块
    // ==========================================================================
    // prcl_rstn 同步到 prcl_clk 域（仿 axi4_asymm 顶层）
    rstn_sync u_rstn_sync (
        .test_mode       (1'b0),
        .test_resetn_in  (1'b1),
        .resetn_in       (prcl_rstn),
        .clk             (prcl_clk),
        .resetn_out      (paif_rstn_i)
    );
    assign paif_rstn = paif_rstn_i;
    assign paif_clk  = prcl_clk;

    // clken 同步后经 1 路 ICG 门控 prcl_clk（单 gating clk）
    sync_cell u_clken_sync (.clk_d(prcl_clk), .rstn_d(paif_rstn_i), .data_s(clken), .data_d(clken_sync));
    //generic_icg_pos u_icg_prcl (.CK(prcl_clk), .E(clken_sync), .Q(prcl_clk_cg));
    assign prcl_clk_cg = prcl_clk;

`ifdef HIPI_GB
    // hipi_sel（CSR pclk 域）→ prcl_clk 2FF 同步（模式选择：0=HIPI Next / 1=GB）
    sync_cell u_hipi_sel_sync (.clk_d(prcl_clk), .rstn_d(paif_rstn_i), .data_s(csr_hipi_sel), .data_d(hipi_sel_sync));
    // 双引擎时钟门控（自 prcl_clk_cg：~hipi_sel_sync → pack/unpack；hipi_sel_sync → GB 三级）
    generic_icg_pos u_icg_pack (.CK(prcl_clk_cg), .E(~hipi_sel_sync), .Q(prcl_clk_pack));
    generic_icg_pos u_icg_gb   (.CK(prcl_clk_cg), .E(hipi_sel_sync),  .Q(prcl_clk_gb));
`endif

    // credit_en 同步（当前仅观察，无独立 crd_tx_ctrl 消费；预留）
    sync_cell u_credit_en_sync (.clk_d(prcl_clk), .rstn_d(prcl_rstn), .data_s(csr_credit_en), .data_d(credit_en_int));

    // 回环使能同步（CSR pclk 域 → 各自回环模块的数据域；2026-09-23 语义纠正：
    //   paif loop=本端环回 / axi loop=远端环回，同步目标域随之对调）
    //   local_loop_sync  → prcl_clk：p_paif_loop（本端环回；PAIF 层数据在 prcl 域）
    //   remote_loop_sync → aclk：    p_axi_loop（远端环回；bond 层 AXI 数据在 aclk 域）
    sync_cell u_remote_loop_sync  (.clk_d(aclk),        .rstn_d(aresetn),     .data_s(csr_remote_loop),  .data_d(remote_loop_sync));
    sync_cell u_local_loop_sync   (.clk_d(prcl_clk),    .rstn_d(paif_rstn_i), .data_s(csr_local_loop),   .data_d(local_loop_sync));

    // ==========================================================================
    // 低功耗透传（ctrl ↔ sw 直通，同 axi4_asymm 顶层）
    // ==========================================================================
    assign lp_sw_cmd   = lp_ctrl_cmd;
    assign lp_sw_req   = lp_ctrl_req;
    assign lp_ctrl_ack = lp_sw_ack;
    assign lp_ctrl_sts = lp_sw_sts;

    // ==========================================================================
    // 集中式 CSR 例化（p_axi4_symm_csr，APB 唯一接入点 + intr + loopback sel）
    // ==========================================================================
    // TODO: 对称桥不设超时/溢出/ots 检测子模块，CSR 状态输入 tie 安全值（idle=1、id/intr=0），
    //       待补检测子模块后回填真实信号。
    p_axi4_symm_csr u_csr (
        // APB（对外）
        .pclk              (pclk),
        .presetn           (presetn),
        .psel              (psel),
        .penable           (penable),
        .pwrite            (pwrite),
        .paddr             (paddr),
        .pwdata            (pwdata),
        .prdata            (prdata),
        .pready            (pready),
        .pslverr           (pslverr),
        .intr              (intr),
        .protocol_layer_info(`PROTOCOL_LAYER_INFO),
        // 配置输出
        .credit_en         (csr_credit_en),
        .timeout_en        (csr_timeout_en),
        .timeout_num       (csr_timeout_num),
        .w0_timeout_clr    (csr_w0_timeout_clr),
        .r0_timeout_clr    (csr_r0_timeout_clr),
        .w0_err_clr        (csr_w0_err_clr),
        .r0_err_clr        (csr_r0_err_clr),
        .r0_ovrflow_clr    (csr_r0_ovrflow_clr),
        .b0_ovrflow_clr    (csr_b0_ovrflow_clr),
        .aw0_ovrflow_clr   (csr_aw0_ovrflow_clr),
        .ar0_ovrflow_clr   (csr_ar0_ovrflow_clr),
        .w0_ovrflow_clr    (csr_w0_ovrflow_clr),
        .remote_loop       (csr_remote_loop),
        .local_loop        (csr_local_loop),
`ifdef HIPI_GB
        .hipi_sel          (csr_hipi_sel),
        .gb_err_clr        (csr_gb_err_clr),
        .gb_intr           (chi_err_intr),
        .gb_pkt_err        (chi_pkt_err),
        .gb_rx_st          (chi_rx_st),
`else
        .hipi_sel          (),
        .gb_err_clr        (),
        .gb_intr           (1'b0),
        .gb_pkt_err        (3'd0),
        .gb_rx_st          (4'd0),
`endif
        // 状态输入（← tie 安全默认）
        .rx_r_ots_idle     (rx_r_ots_idle_t),
        .rx_w_ots_idle     (rx_w_ots_idle_t),
        .tx_r_ots_idle     (tx_r_ots_idle_t),
        .tx_w_ots_idle     (tx_w_ots_idle_t),
        .tx_lane_idle      (tx_lane_idle_t),
        .rx_lane_idle      (rx_lane_idle_t),
        .w0_timeout_id     (w0_timeout_id_t),
        .r0_timeout_id     (r0_timeout_id_t),
        .w0_timeout_intr   (w0_timeout_intr_t),
        .r0_timeout_intr   (r0_timeout_intr_t),
        .w0_err_id         (w0_err_id_t),
        .r0_err_id         (r0_err_id_t),
        .w0_err_intr       (w0_err_intr_t),
        .r0_err_intr       (r0_err_intr_t),
        .r0_ovrflow_intr   (r0_ovrflow_intr_t),
        .b0_ovrflow_intr   (b0_ovrflow_intr_t),
        .aw0_ovrflow_intr  (aw0_ovrflow_intr_t),
        .ar0_ovrflow_intr  (ar0_ovrflow_intr_t),
        .w0_ovrflow_intr   (w0_ovrflow_intr_t)
    );

    // CSR 状态输入（s_axi/m_axi req_timeout 上报 + pack/unpack busy 观测 + ovrflow_det 上报）
    assign rx_r_ots_idle_t    = m_ar_ots_idle;   // 远端（RX）AR 请求 outstanding idle（m_axi ots_chk）
    assign rx_w_ots_idle_t    = m_aw_ots_idle;   // 远端（RX）AW 请求 outstanding idle（m_axi ots_chk）
    assign tx_r_ots_idle_t    = ar_ots_idle;      // 本端（TX）AR 超时检测 outstanding idle
    assign tx_w_ots_idle_t    = aw_ots_idle;      // 本端（TX）AW 超时检测 outstanding idle
    assign w0_timeout_id_t    = aw_timeout_id;
    assign r0_timeout_id_t    = ar_timeout_id;
    assign w0_err_id_t        = aw_err_id;
    assign r0_err_id_t        = ar_err_id;
    assign w0_timeout_intr_t  = aw_timeout_intr;
    assign r0_timeout_intr_t  = ar_timeout_intr;
    assign w0_err_intr_t      = aw_err_intr;
    assign r0_err_intr_t      = ar_err_intr;
    assign r0_ovrflow_intr_t  = r0_ovf_intr;   // RX r FIFO 写侧溢出
    assign b0_ovrflow_intr_t  = b0_ovf_intr;   // RX b FIFO 写侧溢出
    assign aw0_ovrflow_intr_t = aw0_ovf_intr;  // RX aw FIFO 写侧溢出
    assign ar0_ovrflow_intr_t = ar0_ovf_intr;  // RX ar FIFO 写侧溢出
    assign w0_ovrflow_intr_t  = w0_ovf_intr;   // RX w FIFO 写侧溢出

    // tx/rx lane idle 由 pack/unpack busy 观测（打拍寄存，见下方时序逻辑）
    assign tx_lane_idle_t = tx_lane_idle_r;
    assign rx_lane_idle_t = rx_lane_idle_r;

    // ==========================================================================
    // p_req_timeout 例化（s_axi AW/AR 请求超时检测；插在用户 AXI 与 bond 之间）
    //   AW→B：写 outstanding 超时（w0）；AR→R：读 outstanding 超时（r0），aclk 域
    // ==========================================================================
    p_req_timeout #(.ID_WIDTH(10), .OTS_NUM(`OTS_NUM)) i_aw_req_timeout (
        .clk(aclk), .rstn(aresetn),
        .ots_idle(aw_ots_idle), .enable(csr_timeout_en), .timeout_num(csr_timeout_num),
        .axid(s_axi_awid), .s_axvalid(s_axi_awvalid), .s_axready(s_axi_awready),
        .m_axvalid(aw_rt_mvalid), .m_axready(aw_rt_mready),
        .xid(s_axi_bid), .xvalid(s_axi_bvalid), .xready(s_axi_bready),
        .timeout_id(aw_timeout_id), .timeout_intr(aw_timeout_intr), .timeout_clr(csr_w0_timeout_clr),
        .err_id(aw_err_id), .err_intr(aw_err_intr), .err_clr(csr_w0_err_clr)
    );
    p_req_timeout #(.ID_WIDTH(10), .OTS_NUM(`OTS_NUM)) i_ar_req_timeout (
        .clk(aclk), .rstn(aresetn),
        .ots_idle(ar_ots_idle), .enable(csr_timeout_en), .timeout_num(csr_timeout_num),
        .axid(s_axi_arid), .s_axvalid(s_axi_arvalid), .s_axready(s_axi_arready),
        .m_axvalid(ar_rt_mvalid), .m_axready(ar_rt_mready),
        .xid(s_axi_rid), .xvalid(s_axi_rvalid), .xready(s_axi_rready),
        .timeout_id(ar_timeout_id), .timeout_intr(ar_timeout_intr), .timeout_clr(csr_r0_timeout_clr),
        .err_id(ar_err_id), .err_intr(ar_err_intr), .err_clr(csr_r0_err_clr)
    );

    // ==========================================================================
    // lane idle 打拍（~pack_busy / ~unpack_busy，prcl_clk_cg 域，复位 1；参考 axi4_asymm）
    // ==========================================================================
    always @(posedge prcl_clk_cg or negedge paif_rstn_i) begin
        if (!paif_rstn_i) tx_lane_idle_r <= 1'b1;   // 复位后 TX lane 恒 idle
        else              tx_lane_idle_r <= ~pack_busy;
    end
    always @(posedge prcl_clk_cg or negedge paif_rstn_i) begin
        if (!paif_rstn_i) rx_lane_idle_r <= 1'b1;   // 复位后 RX lane 恒 idle
        else              rx_lane_idle_r <= ~unpack_busy;
    end

    // ==========================================================================
    // p_req_timeout 例化（m_axi AW/AR ots_chk，aclk 域；插在 RX FIFO 输出与 unbond 之间）
    //   resp_asymm 简化模式：enable tie 0、id/intr/clr 悬空，仅 ots_idle 上报→CSR rx_w/r_ots_idle
    // ==========================================================================
    p_req_timeout #(.ID_WIDTH(10), .OTS_NUM(`OTS_NUM)) i_aw_rx_ots_chk (
        .clk(aclk), .rstn(aresetn),
        .ots_idle(m_aw_ots_idle), .enable(1'b0), .timeout_num(16'd0),
        .axid(m_axi_awid), .s_axvalid(il_aw_rx_valid), .s_axready(il_aw_rx_ready),
        .m_axvalid(m_aw_ots_mvalid), .m_axready(m_aw_ots_mready),
        .xid(m_axi_bid), .xvalid(m_axi_bvalid), .xready(m_axi_bready),
        .timeout_id(), .timeout_intr(), .timeout_clr(1'b0),
        .err_id(), .err_intr(), .err_clr(1'b0)
    );
    p_req_timeout #(.ID_WIDTH(10), .OTS_NUM(`OTS_NUM)) i_ar_rx_ots_chk (
        .clk(aclk), .rstn(aresetn),
        .ots_idle(m_ar_ots_idle), .enable(1'b0), .timeout_num(16'd0),
        .axid(m_axi_arid), .s_axvalid(il_ar_rx_valid), .s_axready(il_ar_rx_ready),
        .m_axvalid(m_ar_ots_mvalid), .m_axready(m_ar_ots_mready),
        .xid(m_axi_rid), .xvalid(m_axi_rvalid), .xready(m_axi_rready),
        .timeout_id(), .timeout_intr(), .timeout_clr(1'b0),
        .err_id(), .err_intr(), .err_clr(1'b0)
    );

    // ==========================================================================
    // 用户侧 bond（仅 s0/m0 槽 active；s1~3/m1~3 输入 tie0、输出悬空）
    // ==========================================================================
    p_axi4_bond #(
        .AXI_DW   (AXI_DW),
        .AX_WIDTH (AX_WIDTH),
        .W_WIDTH  (W_WIDTH),
        .R_WIDTH  (R_WIDTH),
        .B_WIDTH  (B_WIDTH)
    ) u_bond (
        .clk           (aclk),
        .rstn          (aresetn),
`ifdef HIPI_GB
        .hipi_sel      (csr_hipi_sel),   // CSR 0x600 hipi_sel（准静态，0=HIPI Next / 1=GB）
`else
        .hipi_sel      (1'b0),           // 无 GB 通路：固定 0（HIPI Next 原宽）
`endif
        // s0：AW/AR/W（req 入）；s1~3 tie0（单用户口，未用槽 valid=0、数据 tie0、ready 悬空）
        .s0_axi_awid   (s_axi_awid),   .s0_axi_awaddr (s_axi_awaddr), .s0_axi_awlen (s_axi_awlen),
        .s0_axi_awsize (s_axi_awsize), .s0_axi_awburst(s_axi_awburst),.s0_axi_awlock(s_axi_awlock),
        .s0_axi_awcache(s_axi_awcache),.s0_axi_awprot (s_axi_awprot), .s0_axi_awqos (s_axi_awqos),
        .s0_axi_awregion(s_axi_awregion),.s0_axi_awuser(s_axi_awuser), .s0_axi_awvalid(aw_rt_mvalid),
        .s0_axi_awready(aw_rt_mready),
        .s1_axi_awid(10'd0), .s1_axi_awaddr(64'd0), .s1_axi_awlen(8'd0),
        .s1_axi_awsize(3'd0), .s1_axi_awburst(2'd0), .s1_axi_awlock(1'b0),
        .s1_axi_awcache(4'd0), .s1_axi_awprot(3'd0), .s1_axi_awqos(4'd0),
        .s1_axi_awregion(4'd0), .s1_axi_awuser(4'd0), .s1_axi_awvalid(1'b0),
        .s1_axi_awready(),
        .s2_axi_awid(10'd0), .s2_axi_awaddr(64'd0), .s2_axi_awlen(8'd0),
        .s2_axi_awsize(3'd0), .s2_axi_awburst(2'd0), .s2_axi_awlock(1'b0),
        .s2_axi_awcache(4'd0), .s2_axi_awprot(3'd0), .s2_axi_awqos(4'd0),
        .s2_axi_awregion(4'd0), .s2_axi_awuser(4'd0), .s2_axi_awvalid(1'b0),
        .s2_axi_awready(),
        .s3_axi_awid(10'd0), .s3_axi_awaddr(64'd0), .s3_axi_awlen(8'd0),
        .s3_axi_awsize(3'd0), .s3_axi_awburst(2'd0), .s3_axi_awlock(1'b0),
        .s3_axi_awcache(4'd0), .s3_axi_awprot(3'd0), .s3_axi_awqos(4'd0),
        .s3_axi_awregion(4'd0), .s3_axi_awuser(4'd0), .s3_axi_awvalid(1'b0),
        .s3_axi_awready(),
        .s0_axi_arid   (s_axi_arid),   .s0_axi_araddr (s_axi_araddr), .s0_axi_arlen (s_axi_arlen),
        .s0_axi_arsize (s_axi_arsize), .s0_axi_arburst(s_axi_arburst),.s0_axi_arlock(s_axi_arlock),
        .s0_axi_arcache(s_axi_arcache),.s0_axi_arprot (s_axi_arprot), .s0_axi_arqos (s_axi_arqos),
        .s0_axi_arregion(s_axi_arregion),.s0_axi_aruser(s_axi_aruser), .s0_axi_arvalid(ar_rt_mvalid),
        .s0_axi_arready(ar_rt_mready),
        .s1_axi_arid(10'd0), .s1_axi_araddr(64'd0), .s1_axi_arlen(8'd0),
        .s1_axi_arsize(3'd0), .s1_axi_arburst(2'd0), .s1_axi_arlock(1'b0),
        .s1_axi_arcache(4'd0), .s1_axi_arprot(3'd0), .s1_axi_arqos(4'd0),
        .s1_axi_arregion(4'd0), .s1_axi_aruser(4'd0), .s1_axi_arvalid(1'b0),
        .s1_axi_arready(),
        .s2_axi_arid(10'd0), .s2_axi_araddr(64'd0), .s2_axi_arlen(8'd0),
        .s2_axi_arsize(3'd0), .s2_axi_arburst(2'd0), .s2_axi_arlock(1'b0),
        .s2_axi_arcache(4'd0), .s2_axi_arprot(3'd0), .s2_axi_arqos(4'd0),
        .s2_axi_arregion(4'd0), .s2_axi_aruser(4'd0), .s2_axi_arvalid(1'b0),
        .s2_axi_arready(),
        .s3_axi_arid(10'd0), .s3_axi_araddr(64'd0), .s3_axi_arlen(8'd0),
        .s3_axi_arsize(3'd0), .s3_axi_arburst(2'd0), .s3_axi_arlock(1'b0),
        .s3_axi_arcache(4'd0), .s3_axi_arprot(3'd0), .s3_axi_arqos(4'd0),
        .s3_axi_arregion(4'd0), .s3_axi_aruser(4'd0), .s3_axi_arvalid(1'b0),
        .s3_axi_arready(),
        .s0_axi_wdata  (s_axi_wdata),  .s0_axi_wstrb  (s_axi_wstrb),
        .s0_axi_wuser  (s_axi_wuser),  .s0_axi_wlast  (s_axi_wlast), .s0_axi_wvalid(s_axi_wvalid),
        .s0_axi_wready (s_axi_wready),
        .s1_axi_wdata({AXI_DW{1'b0}}), .s1_axi_wstrb({AXI_DW/8{1'b0}}),
        .s1_axi_wuser(4'd0), .s1_axi_wlast(1'b0), .s1_axi_wvalid(1'b0), .s1_axi_wready(),
        .s2_axi_wdata({AXI_DW{1'b0}}), .s2_axi_wstrb({AXI_DW/8{1'b0}}),
        .s2_axi_wuser(4'd0), .s2_axi_wlast(1'b0), .s2_axi_wvalid(1'b0), .s2_axi_wready(),
        .s3_axi_wdata({AXI_DW{1'b0}}), .s3_axi_wstrb({AXI_DW/8{1'b0}}),
        .s3_axi_wuser(4'd0), .s3_axi_wlast(1'b0), .s3_axi_wvalid(1'b0), .s3_axi_wready(),
        // m0：R/B（resp 入）；m1~3 tie0（数据 tie0、ready 悬空）
        .m0_axi_rid    (m_axi_rid), .m0_axi_rdata  (m_axi_rdata),
        .m0_axi_rresp  (m_axi_rresp),.m0_axi_ruser (m_axi_ruser),
        .m0_axi_rlast  (m_axi_rlast), .m0_axi_rvalid(m_axi_rvalid),
        .m0_axi_rready (m_axi_rready),
        .m1_axi_rid(10'd0), .m1_axi_rdata({AXI_DW{1'b0}}), .m1_axi_rresp(2'd0),
        .m1_axi_ruser(4'd0), .m1_axi_rlast(1'b0), .m1_axi_rvalid(1'b0), .m1_axi_rready(),
        .m2_axi_rid(10'd0), .m2_axi_rdata({AXI_DW{1'b0}}), .m2_axi_rresp(2'd0),
        .m2_axi_ruser(4'd0), .m2_axi_rlast(1'b0), .m2_axi_rvalid(1'b0), .m2_axi_rready(),
        .m3_axi_rid(10'd0), .m3_axi_rdata({AXI_DW{1'b0}}), .m3_axi_rresp(2'd0),
        .m3_axi_ruser(4'd0), .m3_axi_rlast(1'b0), .m3_axi_rvalid(1'b0), .m3_axi_rready(),
        .m0_axi_bid    (m_axi_bid), .m0_axi_bresp (m_axi_bresp),
        .m0_axi_buser  (m_axi_buser),.m0_axi_bvalid(m_axi_bvalid),
        .m0_axi_bready (m_axi_bready),
        .m1_axi_bid(10'd0), .m1_axi_bresp(2'd0), .m1_axi_buser(4'd0),
        .m1_axi_bvalid(1'b0), .m1_axi_bready(),
        .m2_axi_bid(10'd0), .m2_axi_bresp(2'd0), .m2_axi_buser(4'd0),
        .m2_axi_bvalid(1'b0), .m2_axi_bready(),
        .m3_axi_bid(10'd0), .m3_axi_bresp(2'd0), .m3_axi_buser(4'd0),
        .m3_axi_bvalid(1'b0), .m3_axi_bready(),
        // bond bus 输出（仅 s0/m0 槽收）
        .aw0_tx_bus    (bond_aw0_bus), .aw0_tx_valid(bond_aw0_valid), .aw0_tx_ready(bond_aw0_ready),
        .aw1_tx_bus(), .aw1_tx_valid(), .aw1_tx_ready(1'b0),
        .aw2_tx_bus(), .aw2_tx_valid(), .aw2_tx_ready(1'b0),
        .aw3_tx_bus(), .aw3_tx_valid(), .aw3_tx_ready(1'b0),
        .ar0_tx_bus    (bond_ar0_bus), .ar0_tx_valid(bond_ar0_valid), .ar0_tx_ready(bond_ar0_ready),
        .ar1_tx_bus(), .ar1_tx_valid(), .ar1_tx_ready(1'b0),
        .ar2_tx_bus(), .ar2_tx_valid(), .ar2_tx_ready(1'b0),
        .ar3_tx_bus(), .ar3_tx_valid(), .ar3_tx_ready(1'b0),
        .w0_tx_bus     (bond_w0_bus),  .w0_tx_valid (bond_w0_valid),  .w0_tx_ready (bond_w0_ready),
        .w1_tx_bus(), .w1_tx_valid(), .w1_tx_ready(1'b0),
        .w2_tx_bus(), .w2_tx_valid(), .w2_tx_ready(1'b0),
        .w3_tx_bus(), .w3_tx_valid(), .w3_tx_ready(1'b0),
        .r0_tx_bus     (bond_r0_bus),  .r0_tx_valid (bond_r0_valid),  .r0_tx_ready (bond_r0_ready),
        .r1_tx_bus(), .r1_tx_valid(), .r1_tx_ready(1'b0),
        .r2_tx_bus(), .r2_tx_valid(), .r2_tx_ready(1'b0),
        .r3_tx_bus(), .r3_tx_valid(), .r3_tx_ready(1'b0),
        .b0_tx_bus     (bond_b0_bus),  .b0_tx_valid (bond_b0_valid),  .b0_tx_ready (bond_b0_ready),
        .b1_tx_bus(), .b1_tx_valid(), .b1_tx_ready(1'b0),
        .b2_tx_bus(), .b2_tx_valid(), .b2_tx_ready(1'b0),
        .b3_tx_bus(), .b3_tx_valid(), .b3_tx_ready(1'b0),
        // lite 标志（r0/w0 active；w/r 其余 slot 悬空）
        .w0_lite       (bond_w0_lite), .w1_lite(), .w2_lite(), .w3_lite(),
        .r0_lite       (bond_r0_lite), .r1_lite(), .r2_lite(), .r3_lite()
    );

    // ==========================================================================
    // TX async FIFO（aclk → prcl_clk_cg；5 路）
    // ==========================================================================
    p_axis_async_fifo #(.DATA_W(AX_WIDTH), .DEPTH(`PRCL_ASYNC_TXFIFO_DEPTH)) i_aw_txfifo (
        .s_clk(aclk), .m_clk(prcl_clk_cg), .s_rstn(aresetn), .m_rstn(paif_rstn_i),
        .s_data(il_aw_tx_bus), .s_valid(il_aw_tx_valid), .s_ready(tx_aw_sready),
        .m_data(tx_aw_mdata), .m_valid(tx_aw_mvalid), .m_ready(tx_aw_mready),
        .crd(tx_aw_crd), .crd_valid(tx_aw_crd_v)
    );
    p_axis_async_fifo #(.DATA_W(AX_WIDTH), .DEPTH(`PRCL_ASYNC_TXFIFO_DEPTH)) i_ar_txfifo (
        .s_clk(aclk), .m_clk(prcl_clk_cg), .s_rstn(aresetn), .m_rstn(paif_rstn_i),
        .s_data(il_ar_tx_bus), .s_valid(il_ar_tx_valid), .s_ready(tx_ar_sready),
        .m_data(tx_ar_mdata), .m_valid(tx_ar_mvalid), .m_ready(tx_ar_mready),
        .crd(tx_ar_crd), .crd_valid(tx_ar_crd_v)
    );
    p_axis_async_fifo #(.DATA_W(PK_W_WIDTH+1), .DEPTH(`PRCL_ASYNC_TXFIFO_DEPTH)) i_w_txfifo (
        .s_clk(aclk), .m_clk(prcl_clk_cg), .s_rstn(aresetn), .m_rstn(paif_rstn_i),
        .s_data(tx_w_sdata), .s_valid(il_w_tx_valid), .s_ready(tx_w_sready),
        .m_data(tx_w_mdata), .m_valid(tx_w_mvalid), .m_ready(tx_w_mready),
        .crd(tx_w_crd), .crd_valid(tx_w_crd_v)
    );
    p_axis_async_fifo #(.DATA_W(PK_R_WIDTH+1), .DEPTH(`PRCL_ASYNC_TXFIFO_DEPTH)) i_r_txfifo (
        .s_clk(aclk), .m_clk(prcl_clk_cg), .s_rstn(aresetn), .m_rstn(paif_rstn_i),
        .s_data(tx_r_sdata), .s_valid(il_r_tx_valid), .s_ready(tx_r_sready),
        .m_data(tx_r_mdata), .m_valid(tx_r_mvalid), .m_ready(tx_r_mready),
        .crd(tx_r_crd), .crd_valid(tx_r_crd_v)
    );
    p_axis_async_fifo #(.DATA_W(B_WIDTH), .DEPTH(`PRCL_ASYNC_TXFIFO_DEPTH)) i_b_txfifo (
        .s_clk(aclk), .m_clk(prcl_clk_cg), .s_rstn(aresetn), .m_rstn(paif_rstn_i),
        .s_data(il_b_tx_bus), .s_valid(il_b_tx_valid), .s_ready(tx_b_sready),
        .m_data(tx_b_mdata), .m_valid(tx_b_mvalid), .m_ready(tx_b_mready),
        .crd(tx_b_crd), .crd_valid(tx_b_crd_v)
    );
    // TX FIFO credit（aclk 写侧）悬空（无对端信用消费；预留）
    // TODO: 若需 TX FIFO 占用回读，接 CSR/观测；当前悬空。

    // bond s0/m0 槽 与 TX FIFO：经 remote_loop（p_axi_loop，远端环回）桥接，ready 由回环输出驱动
    // （local_loop=0 直通时 bond_*_ready = tx_*_sready，语义与原 assign 一致）；
    // w/r lite 位随最高 bit 过 FIFO（回环时 lite=0，见 il_w_lit_tx/il_r_lit_tx）。
    assign tx_w_sdata     = {il_w_lit_tx, il_w_tx_bus};
    assign tx_r_sdata     = {il_r_lit_tx, il_r_tx_bus};

    // ==========================================================================
    // remote_loop（p_axi_loop，bond 层回环，aclk 域，全组合，sel=remote_loop_sync；2026-09-23 语义纠正）
    //   s0 = bond TX（AXI IN）；m0 = TX FIFO（protocol tx）
    //   s1 = RX FIFO（protocol rx）；m1 = unbond/ots_chk（AXI OUT）
    //   直通（sel=0）：s0→m0（bond→TX FIFO）、s1→m1（RX FIFO→unbond），同原直连。
    //   远端环回（sel=1）：对端发来的数据经 s1(RX) 回灌 m0(TX) 发回对端、m1 关闭。
    //   AX_W/W_W/R_W/B_W = bond 总线位宽；W/R lite 位由 p_axi_loop 自带 lite 通道送出
    // ==========================================================================
    p_axi_loop #(
        .AW_W (AX_WIDTH), .AR_W (AX_WIDTH),
        .W_W  (W_WIDTH),
        .R_W  (R_WIDTH),
        .B_W  (B_WIDTH)
    ) u_axi_loop (
        .sel            (remote_loop_sync),   // 2026-09-23：p_axi_loop = 远端环回
        // ---- s0 = bond TX（AXI IN）----
        .s0_aw_data     (bond_aw0_bus), .s0_aw_valid(bond_aw0_valid), .s0_aw_ready(bond_aw0_ready),
        .s0_ar_data     (bond_ar0_bus), .s0_ar_valid(bond_ar0_valid), .s0_ar_ready(bond_ar0_ready),
        .s0_w_data      (bond_w0_bus),  .s0_w_valid (bond_w0_valid),  .s0_w_ready (bond_w0_ready),
        .s0_w_lite      (bond_w0_lite),
        .s0_r_data      (bond_r0_bus),  .s0_r_valid (bond_r0_valid),  .s0_r_ready (bond_r0_ready),
        .s0_r_lite      (bond_r0_lite),
        .s0_b_data      (bond_b0_bus),  .s0_b_valid (bond_b0_valid),  .s0_b_ready (bond_b0_ready),
        // ---- m0 = TX FIFO（protocol tx）----
        .m0_aw_data     (il_aw_tx_bus), .m0_aw_valid(il_aw_tx_valid), .m0_aw_ready(tx_aw_sready),
        .m0_ar_data     (il_ar_tx_bus), .m0_ar_valid(il_ar_tx_valid), .m0_ar_ready(tx_ar_sready),
        .m0_w_data      (il_w_tx_bus),  .m0_w_valid (il_w_tx_valid),  .m0_w_ready (tx_w_sready),
        .m0_w_lite      (il_w_lit_tx),
        .m0_r_data      (il_r_tx_bus),  .m0_r_valid (il_r_tx_valid),  .m0_r_ready (tx_r_sready),
        .m0_r_lite      (il_r_lit_tx),
        .m0_b_data      (il_b_tx_bus),  .m0_b_valid (il_b_tx_valid),  .m0_b_ready (tx_b_sready),
        // ---- s1 = RX FIFO（protocol rx）----
        .s1_aw_data     (rx_aw_mdata),  .s1_aw_valid(rx_aw_mvalid),  .s1_aw_ready(rx_aw_mready),
        .s1_ar_data     (rx_ar_mdata),  .s1_ar_valid(rx_ar_mvalid),  .s1_ar_ready(rx_ar_mready),
        .s1_w_data      (rx_w_mdata),   .s1_w_valid (rx_w_mvalid),   .s1_w_ready (rx_w_mready),
        .s1_w_lite      (1'b0),
        .s1_r_data      (rx_r_mdata),   .s1_r_valid (rx_r_mvalid),   .s1_r_ready (rx_r_mready),
        .s1_r_lite      (1'b0),
        .s1_b_data      (rx_b_mdata),   .s1_b_valid (rx_b_mvalid),   .s1_b_ready (rx_b_mready),
        // ---- m1 = unbond/ots_chk（AXI OUT）----
        .m1_aw_data     (il_aw_rx_bus), .m1_aw_valid(il_aw_rx_valid), .m1_aw_ready(il_aw_rx_ready),
        .m1_ar_data     (il_ar_rx_bus), .m1_ar_valid(il_ar_rx_valid), .m1_ar_ready(il_ar_rx_ready),
        .m1_w_data      (il_w_rx_bus),  .m1_w_valid (il_w_rx_valid),  .m1_w_ready (il_w_rx_ready),
        .m1_w_lite      (),
        .m1_r_data      (il_r_rx_bus),  .m1_r_valid (il_r_rx_valid),  .m1_r_ready (il_r_rx_ready),
        .m1_r_lite      (),
        .m1_b_data      (il_b_rx_bus),  .m1_b_valid (il_b_rx_valid),  .m1_b_ready (il_b_rx_ready)
    );

    // ==========================================================================
    // p_crd_tx_ctrl 例化（TX FIFO 输出 → pack 输入，Credit 反压，prcl_clk_cg 域）
    //   远端 Credit（unpack rp0）门控 5 路 TX 通道：aw/ar/w/r/b。enable=credit_en_int。
    // ==========================================================================
    p_crd_tx_ctrl i_aw_crd_ctrl (
        .clk(prcl_clk_cg), .rstn(paif_rstn_i), .enable(credit_en_int),
`ifdef HIPI_GB
        .remote_crd(mrg_aw_crd), .remote_crd_valid(mrg_aw_crd_v),   // 双引擎模式门控合并
`else
        .remote_crd(rem_aw_crd), .remote_crd_valid(rem_aw_crd_v),
`endif
        .s_valid(tx_aw_mvalid), .s_ready(tx_aw_mready),
        .m_valid(aw_crd_m_valid), .m_ready(aw_crd_m_ready)
    );
    p_crd_tx_ctrl i_ar_crd_ctrl (
        .clk(prcl_clk_cg), .rstn(paif_rstn_i), .enable(credit_en_int),
`ifdef HIPI_GB
        .remote_crd(mrg_ar_crd), .remote_crd_valid(mrg_ar_crd_v),   // 双引擎模式门控合并
`else
        .remote_crd(rem_ar_crd), .remote_crd_valid(rem_ar_crd_v),
`endif
        .s_valid(tx_ar_mvalid), .s_ready(tx_ar_mready),
        .m_valid(ar_crd_m_valid), .m_ready(ar_crd_m_ready)
    );
    p_crd_tx_ctrl i_w_crd_ctrl (
        .clk(prcl_clk_cg), .rstn(paif_rstn_i), .enable(credit_en_int),
`ifdef HIPI_GB
        .remote_crd(mrg_w_crd), .remote_crd_valid(mrg_w_crd_v),   // 双引擎模式门控合并
`else
        .remote_crd(rem_w_crd), .remote_crd_valid(rem_w_crd_v),
`endif
        .s_valid(tx_w_mvalid), .s_ready(tx_w_mready),
        .m_valid(w_crd_m_valid), .m_ready(w_crd_m_ready)
    );
    p_crd_tx_ctrl i_r_crd_ctrl (
        .clk(prcl_clk_cg), .rstn(paif_rstn_i), .enable(credit_en_int),
`ifdef HIPI_GB
        .remote_crd(mrg_r_crd), .remote_crd_valid(mrg_r_crd_v),   // 双引擎模式门控合并
`else
        .remote_crd(rem_r_crd), .remote_crd_valid(rem_r_crd_v),
`endif
        .s_valid(tx_r_mvalid), .s_ready(tx_r_mready),
        .m_valid(r_crd_m_valid), .m_ready(r_crd_m_ready)
    );
    p_crd_tx_ctrl i_b_crd_ctrl (
        .clk(prcl_clk_cg), .rstn(paif_rstn_i), .enable(credit_en_int),
`ifdef HIPI_GB
        .remote_crd(mrg_b_crd), .remote_crd_valid(mrg_b_crd_v),   // 双引擎模式门控合并
`else
        .remote_crd(rem_b_crd), .remote_crd_valid(rem_b_crd_v),
`endif
        .s_valid(tx_b_mvalid), .s_ready(tx_b_mready),
        .m_valid(b_crd_m_valid), .m_ready(b_crd_m_ready)
    );

    // TX → pack 数据适配（w_lite/r_lite 随最高 bit 过 FIFO；valid/ready 经 crd_tx_ctrl 门控）
    assign pack_aw_bus   = tx_aw_mdata;
    assign pack_aw_valid = aw_crd_m_valid;
`ifdef HIPI_GB
    assign aw_crd_m_ready = hipi_sel_sync ? txpkt_aw_ready : pack_aw_ready;   // 模式选路：1=GB / 0=HIPI Next
`else
    assign aw_crd_m_ready = pack_aw_ready;
`endif
    assign pack_ar0_bus  = tx_ar_mdata;
    assign pack_ar0_valid= ar_crd_m_valid;
`ifdef HIPI_GB
    assign ar_crd_m_ready = hipi_sel_sync ? txpkt_ar_ready : pack_ar0_ready;   // 模式选路：1=GB / 0=HIPI Next
`else
    assign ar_crd_m_ready = pack_ar0_ready;
`endif
    assign pack_w_bus    = tx_w_mdata[PK_W_WIDTH-1:0];
    assign pack_w_lite   = tx_w_mdata[PK_W_WIDTH];
    assign pack_w_valid  = w_crd_m_valid;
`ifdef HIPI_GB
    assign w_crd_m_ready = hipi_sel_sync ? txpkt_w_ready : pack_w_ready;   // 模式选路：1=GB / 0=HIPI Next
`else
    assign w_crd_m_ready = pack_w_ready;
`endif
    assign pack_r_bus    = tx_r_mdata[PK_R_WIDTH-1:0];
    assign pack_r_lite   = tx_r_mdata[PK_R_WIDTH];
    assign pack_r_valid  = r_crd_m_valid;
`ifdef HIPI_GB
    assign r_crd_m_ready = hipi_sel_sync ? txpkt_r_ready : pack_r_ready;   // 模式选路：1=GB / 0=HIPI Next
`else
    assign r_crd_m_ready = pack_r_ready;
`endif
    assign pack_b0_bus   = tx_b_mdata;
    assign pack_b0_valid = b_crd_m_valid;
`ifdef HIPI_GB
    assign b_crd_m_ready = hipi_sel_sync ? txpkt_b_ready : pack_b0_ready;   // 模式选路：1=GB / 0=HIPI Next
`else
    assign b_crd_m_ready = pack_b0_ready;
`endif

    // ==========================================================================
    // p_axi4_rwch_pack（PAIF TX 封装，prcl_clk 域，单 lane / 仅 rp0 槽）
    // ==========================================================================
    p_axi4_rwch_pack #(.PAIF_DW(PAIF_DW), .W_RP(2'd0), .R_RP(2'd0)) u_pack (
`ifdef HIPI_GB
        .clk           (prcl_clk_pack),   // hipi_sel_sync==0（HIPI Next 模式）时使能
`else
        .clk           (prcl_clk_cg),
`endif
        .rstn          (paif_rstn_i),
        .busy          (pack_busy),
        .aw_bus        (pack_aw_bus), .aw_valid(pack_aw_valid), .aw_ready(pack_aw_ready),
        .ar_rp0_bus    (pack_ar0_bus), .ar_rp0_valid(pack_ar0_valid), .ar_rp0_ready(pack_ar0_ready),
        .ar_rp1_bus    (107'd0), .ar_rp1_valid(1'b0), .ar_rp1_ready(),
        .ar_rp2_bus    (107'd0), .ar_rp2_valid(1'b0), .ar_rp2_ready(),
        .ar_rp3_bus    (107'd0), .ar_rp3_valid(1'b0), .ar_rp3_ready(),
        .w_bus         (pack_w_bus), .w_lite(pack_w_lite), .w_valid(pack_w_valid), .w_ready(pack_w_ready),
        .r_bus         (pack_r_bus), .r_lite(pack_r_lite), .r_valid(pack_r_valid), .r_ready(pack_r_ready),
        .b_rp0_bus     (pack_b0_bus), .b_rp0_valid(pack_b0_valid), .b_rp0_ready(pack_b0_ready),
        .b_rp1_bus     (16'd0), .b_rp1_valid(1'b0), .b_rp1_ready(),
        .b_rp2_bus     (16'd0), .b_rp2_valid(1'b0), .b_rp2_ready(),
        .b_rp3_bus     (16'd0), .b_rp3_valid(1'b0), .b_rp3_ready(),
        // Credit rp0：本地 credit（crd_gen 读 RX FIFO 占用）；rp1~3 tie0
        .awcrd_rp0     (loc_awcrd_rp0), .awcrd_rp0_ready(pack_awcrd0_ready),
        .awcrd_rp1     (2'd0), .awcrd_rp1_ready(),
        .awcrd_rp2     (2'd0), .awcrd_rp2_ready(),
        .awcrd_rp3     (2'd0), .awcrd_rp3_ready(),
        .wcrd_rp0      (loc_wcrd_rp0),  .wcrd_rp0_ready(pack_wcrd0_ready),
        .wcrd_rp1      (2'd0), .wcrd_rp1_ready(),
        .wcrd_rp2      (2'd0), .wcrd_rp2_ready(),
        .wcrd_rp3      (2'd0), .wcrd_rp3_ready(),
        .arcrd_rp0     (loc_arcrd_rp0), .arcrd_rp0_ready(pack_arcrd0_ready),
        .arcrd_rp1     (2'd0), .arcrd_rp1_ready(),
        .arcrd_rp2     (2'd0), .arcrd_rp2_ready(),
        .arcrd_rp3     (2'd0), .arcrd_rp3_ready(),
        .rcrd_rp0      (loc_rcrd_rp0),  .rcrd_rp0_ready(pack_rcrd0_ready),
        .rcrd_rp1      (2'd0), .rcrd_rp1_ready(),
        .rcrd_rp2      (2'd0), .rcrd_rp2_ready(),
        .rcrd_rp3      (2'd0), .rcrd_rp3_ready(),
        .bcrd_rp0      (loc_bcrd_rp0),  .bcrd_rp0_ready(pack_bcrd0_ready),
        .bcrd_rp1      (2'd0), .bcrd_rp1_ready(),
        .bcrd_rp2      (2'd0), .bcrd_rp2_ready(),
        .bcrd_rp3      (2'd0), .bcrd_rp3_ready(),
        .paif_txdata   (pack_txdata), .paif_txvalid(pack_txvalid), .paif_txready(pack_txready)
    );

    // ==========================================================================
    // p_axi4_rwch_unpack（PAIF RX 解封装，prcl_clk 域，单 lane / 仅 rp0 槽）
    // ==========================================================================
    p_axi4_rwch_unpack #(.PAIF_DW(PAIF_DW)) u_unpack (
`ifdef HIPI_GB
        .clk           (prcl_clk_pack),   // hipi_sel_sync==0（HIPI Next 模式）时使能
`else
        .clk           (prcl_clk_cg),
`endif
        .rstn          (paif_rstn_i),
        .busy          (unpack_busy),
        .aw_bus        (unp_aw_bus), .aw_valid(unp_aw_valid),
        .ar_rp0_bus    (unp_ar0_bus), .ar_rp0_valid(unp_ar0_valid),
        .ar_rp1_bus(), .ar_rp1_valid(),
        .ar_rp2_bus(), .ar_rp2_valid(),
        .ar_rp3_bus(), .ar_rp3_valid(),
        .w_bus         (unp_w_bus), .w_valid(unp_w_valid),
        .r_bus         (unp_r_bus), .r_valid(unp_r_valid),
        .b_rp0_bus     (unp_b0_bus), .b_rp0_valid(unp_b0_valid),
        .b_rp1_bus(), .b_rp1_valid(),
        .b_rp2_bus(), .b_rp2_valid(),
        .b_rp3_bus(), .b_rp3_valid(),
        // 远端 Credit（rp0）→ crd_tx_ctrl（aw/ar/w/r/b 5 路 TX 门控）；rp1~3 悬空
        .awcrd_rp0(rem_aw_crd), .awcrd_rp0_valid(rem_aw_crd_v),
        .awcrd_rp1(), .awcrd_rp1_valid(),
        .awcrd_rp2(), .awcrd_rp2_valid(),
        .awcrd_rp3(), .awcrd_rp3_valid(),
        .wcrd_rp0(rem_w_crd), .wcrd_rp0_valid(rem_w_crd_v),
        .wcrd_rp1(), .wcrd_rp1_valid(),
        .wcrd_rp2(), .wcrd_rp2_valid(),
        .wcrd_rp3(), .wcrd_rp3_valid(),
        .arcrd_rp0(rem_ar_crd), .arcrd_rp0_valid(rem_ar_crd_v),
        .arcrd_rp1(), .arcrd_rp1_valid(),
        .arcrd_rp2(), .arcrd_rp2_valid(),
        .arcrd_rp3(), .arcrd_rp3_valid(),
        .rcrd_rp0(rem_r_crd), .rcrd_rp0_valid(rem_r_crd_v),
        .rcrd_rp1(), .rcrd_rp1_valid(),
        .rcrd_rp2(), .rcrd_rp2_valid(),
        .rcrd_rp3(), .rcrd_rp3_valid(),
        .bcrd_rp0(rem_b_crd), .bcrd_rp0_valid(rem_b_crd_v),
        .bcrd_rp1(), .bcrd_rp1_valid(),
        .bcrd_rp2(), .bcrd_rp2_valid(),
        .bcrd_rp3(), .bcrd_rp3_valid(),
        .paif_rxdata   (unpack_rxdata), .paif_rxvalid(unpack_rxvalid), .paif_rxready(unpack_rxready)
    );

    // ==========================================================================
    // 本地 Credit 生成（crd_gen 读 RX async FIFO 占用 → pack rp0 credit）
    //   pack 的 rp0 credit = 本端授予对端的各通道信用（= 对应 RX FIFO 可用空间）
    // ==========================================================================
    p_crd_gen #(.CRD_NUM(`PRCL_ASYNC_RXFIFO_DEPTH)) i_aw_crdgen (
        .clk(prcl_clk_cg), .rstn(paif_rstn_i), .enable(credit_en_int),
`ifdef HIPI_GB
        .loc_crd(rx_aw_crd), .loc_crd_valid(rx_aw_crd_v), .m_crd(loc_awcrd_rp0), .m_ready(awcrd_up_ready)
`else
        .loc_crd(rx_aw_crd), .loc_crd_valid(rx_aw_crd_v), .m_crd(loc_awcrd_rp0), .m_ready(pack_awcrd0_ready)
`endif
    );
    p_crd_gen #(.CRD_NUM(`PRCL_ASYNC_RXFIFO_DEPTH)) i_w_crdgen (
        .clk(prcl_clk_cg), .rstn(paif_rstn_i), .enable(credit_en_int),
`ifdef HIPI_GB
        .loc_crd(rx_w_crd), .loc_crd_valid(rx_w_crd_v), .m_crd(loc_wcrd_rp0), .m_ready(wcrd_up_ready)
`else
        .loc_crd(rx_w_crd), .loc_crd_valid(rx_w_crd_v), .m_crd(loc_wcrd_rp0), .m_ready(pack_wcrd0_ready)
`endif
    );
    p_crd_gen #(.CRD_NUM(`PRCL_ASYNC_RXFIFO_DEPTH)) i_ar_crdgen (
        .clk(prcl_clk_cg), .rstn(paif_rstn_i), .enable(credit_en_int),
`ifdef HIPI_GB
        .loc_crd(rx_ar_crd), .loc_crd_valid(rx_ar_crd_v), .m_crd(loc_arcrd_rp0), .m_ready(arcrd_up_ready)
`else
        .loc_crd(rx_ar_crd), .loc_crd_valid(rx_ar_crd_v), .m_crd(loc_arcrd_rp0), .m_ready(pack_arcrd0_ready)
`endif
    );
    p_crd_gen #(.CRD_NUM(`PRCL_ASYNC_RXFIFO_DEPTH)) i_r_crdgen (
        .clk(prcl_clk_cg), .rstn(paif_rstn_i), .enable(credit_en_int),
`ifdef HIPI_GB
        .loc_crd(rx_r_crd), .loc_crd_valid(rx_r_crd_v), .m_crd(loc_rcrd_rp0), .m_ready(rcrd_up_ready)
`else
        .loc_crd(rx_r_crd), .loc_crd_valid(rx_r_crd_v), .m_crd(loc_rcrd_rp0), .m_ready(pack_rcrd0_ready)
`endif
    );
    p_crd_gen #(.CRD_NUM(`PRCL_ASYNC_RXFIFO_DEPTH)) i_b_crdgen (
        .clk(prcl_clk_cg), .rstn(paif_rstn_i), .enable(credit_en_int),
`ifdef HIPI_GB
        .loc_crd(rx_b_crd), .loc_crd_valid(rx_b_crd_v), .m_crd(loc_bcrd_rp0), .m_ready(bcrd_up_ready)
`else
        .loc_crd(rx_b_crd), .loc_crd_valid(rx_b_crd_v), .m_crd(loc_bcrd_rp0), .m_ready(pack_bcrd0_ready)
`endif
    );


`ifdef HIPI_GB
    // ==========================================================================
    // HIPI GB 引擎（`ifdef HIPI_GB；hipi_sel_sync==1 时 prcl_clk_gb 使能）
    //   设计约束：GB 通路仅在 PAIF_DW=512 下启用（pkgpipe 输出定宽 512）；PAIF_DW=1024
    //   时不存在 GB 通路（hipi_sel 恒 0 / 仅走 HIPI Next pack 通路）。
    //   TX：protocol_tx_packet（768-bit 包）→ protocol_tx_pkgpipe（512-bit PAIF）
    //       输入与 u_pack 同源（pack_aw/ar/w/r/b + credit 扇出），输出经 u_tx_mux 入 paif_loop s0
    //   RX：paif_loop m1 → u_rx_demux → protocol_rx_ch_identify
    //       输出与 u_unpack 同类通道带模式门控相或合并 1 路（→ RX FIFO / crd_tx_ctrl）
    // ==========================================================================
    // ---- TX：credit 上游 ready 按模式选择（冻结引擎 ready 不回流） ----
    assign awcrd_up_ready = hipi_sel_sync ? txpkt_awcrd_ready : pack_awcrd0_ready;
    assign wcrd_up_ready  = hipi_sel_sync ? txpkt_wcrd_ready  : pack_wcrd0_ready;
    assign arcrd_up_ready = hipi_sel_sync ? txpkt_arcrd_ready : pack_arcrd0_ready;
    assign rcrd_up_ready  = hipi_sel_sync ? txpkt_rcrd_ready  : pack_rcrd0_ready;
    assign bcrd_up_ready  = hipi_sel_sync ? txpkt_bcrd_ready  : pack_bcrd0_ready;

    // ---- GB TX 打包（输入 = u_pack 输入同源扇出） ----
    protocol_tx_packet u_tx_packet (
        .clk(prcl_clk_gb), .rst_n(paif_rstn_i),
        .s_aw_data(pack_aw_bus),  .s_aw_valid(pack_aw_valid),  .s_aw_ready(txpkt_aw_ready),
        .s_ar_data(pack_ar0_bus), .s_ar_valid(pack_ar0_valid), .s_ar_ready(txpkt_ar_ready),
        .s_w_data(pack_w_bus),    .s_w_valid(pack_w_valid),    .s_w_ready(txpkt_w_ready),
        .s_r_data(pack_r_bus),    .s_r_valid(pack_r_valid),    .s_r_ready(txpkt_r_ready),
        .s_b_data(pack_b0_bus),   .s_b_valid(pack_b0_valid),   .s_b_ready(txpkt_b_ready),
        .s_awcrd(loc_awcrd_rp0),  .s_awcrd_ready(txpkt_awcrd_ready),
        .s_arcrd(loc_arcrd_rp0),  .s_arcrd_ready(txpkt_arcrd_ready),
        .s_wcrd(loc_wcrd_rp0),    .s_wcrd_ready(txpkt_wcrd_ready),
        .s_rcrd(loc_rcrd_rp0),    .s_rcrd_ready(txpkt_rcrd_ready),
        .s_bcrd(loc_bcrd_rp0),    .s_bcrd_ready(txpkt_bcrd_ready),
        .m_data(gb_pkt_data), .m_valid(gb_pkt_valid), .m_ready(gb_pkt_ready)
    );
    // ---- GB TX 流水（768→512） ----
    protocol_tx_pkgpipe u_pkgpipe (
        .clk(prcl_clk_gb), .rst_n(paif_rstn_i),
        .s_data(gb_pkt_data), .s_valid(gb_pkt_valid), .s_ready(gb_pkt_ready),
        .m_data(gb_paif_data), .m_valid(gb_paif_valid), .m_ready(gb_paif_ready)
    );
    // ---- TX 出口 2:1 mux（sel=hipi_sel_sync：0=u_pack / 1=u_pkgpipe → paif_loop s0） ----
    p_tx_mux2to1 #(.DATA_W(PAIF_DW)) u_tx_mux (   // 2026-09-23：paif bus = PAIF_DW
        .sel(hipi_sel_sync),
        .s0_data(pack_txdata), .s0_valid(pack_txvalid), .s0_ready(pack_txready),
        .s1_data(gb_paif_data), .s1_valid(gb_paif_valid), .s1_ready(gb_paif_ready),
        .m_data(loop_s0_data), .m_valid(loop_s0_valid), .m_ready(loop_s0_ready)
    );

    // ---- RX 入口 1:2 demux（sel=hipi_sel_sync：0=u_unpack / 1=u_ch_identify；源 = paif_loop m1） ----
    p_rx_demux1to2 #(.DATA_W(PAIF_DW)) u_rx_demux (   // 2026-09-23：paif bus = PAIF_DW
        .sel(hipi_sel_sync),
        .s_data(loop_m1_data), .s_valid(loop_m1_valid), .s_ready(loop_m1_ready),
        .m0_data(unpack_rxdata), .m0_valid(unpack_rxvalid), .m0_ready(unpack_rxready),
        .m1_data(gb_rx_data), .m1_valid(gb_rx_valid), .m1_ready(gb_rx_ready)
    );
    // ---- GB RX 识别（输出与 u_unpack 同类通道） ----
    protocol_rx_ch_identify u_ch_identify (
        .clk(prcl_clk_gb), .rst_n(paif_rstn_i),
        .pkt_err(chi_pkt_err), .err_intr(chi_err_intr), .rx_st(chi_rx_st),
        .err_clr(csr_gb_err_clr),
        .s_paif_data(gb_rx_data), .s_paif_valid(gb_rx_valid), .s_paif_ready(gb_rx_ready),
        .m_w_data(chi_w_bus),  .m_w_valid(chi_w_valid),
        .m_r_data(chi_r_bus),  .m_r_valid(chi_r_valid),
        .m_aw_data(chi_aw_bus), .m_aw_valid(chi_aw_valid),
        .m_ar_data(chi_ar_bus), .m_ar_valid(chi_ar_valid),
        .m_b_data(chi_b_bus),  .m_b_valid(chi_b_valid),
        .m_awcrd(chi_awcrd), .m_awcrd_valid(chi_awcrd_v),
        .m_arcrd(chi_arcrd), .m_arcrd_valid(chi_arcrd_v),
        .m_wcrd(chi_wcrd),   .m_wcrd_valid(chi_wcrd_v),
        .m_rcrd(chi_rcrd),   .m_rcrd_valid(chi_rcrd_v),
        .m_bcrd(chi_bcrd),   .m_bcrd_valid(chi_bcrd_v)
    );

    // ---- 双引擎合并（模式门控相或：冻结侧 valid/data 全门掉，防死 valid/脏 data 灌下级） ----
    assign mrg_aw_bus   = (unp_aw_bus   & {AX_WIDTH{~hipi_sel_sync}}) | (chi_aw_bus & {AX_WIDTH{hipi_sel_sync}});
    assign mrg_aw_valid = (unp_aw_valid   & ~hipi_sel_sync) | (chi_aw_valid & hipi_sel_sync);
    assign mrg_ar_bus   = (unp_ar0_bus  & {AX_WIDTH{~hipi_sel_sync}}) | (chi_ar_bus & {AX_WIDTH{hipi_sel_sync}});
    assign mrg_ar_valid = (unp_ar0_valid  & ~hipi_sel_sync) | (chi_ar_valid & hipi_sel_sync);
    assign mrg_w_bus    = (unp_w_bus    & {W_WIDTH{~hipi_sel_sync}})  | (chi_w_bus  & {W_WIDTH{hipi_sel_sync}});
    assign mrg_w_valid  = (unp_w_valid    & ~hipi_sel_sync) | (chi_w_valid & hipi_sel_sync);
    assign mrg_r_bus    = (unp_r_bus    & {R_WIDTH{~hipi_sel_sync}})  | (chi_r_bus  & {R_WIDTH{hipi_sel_sync}});
    assign mrg_r_valid  = (unp_r_valid    & ~hipi_sel_sync) | (chi_r_valid & hipi_sel_sync);
    assign mrg_b_bus    = (unp_b0_bus   & {B_WIDTH{~hipi_sel_sync}})  | (chi_b_bus  & {B_WIDTH{hipi_sel_sync}});
    assign mrg_b_valid  = (unp_b0_valid   & ~hipi_sel_sync) | (chi_b_valid & hipi_sel_sync);
    assign mrg_aw_crd   = (rem_aw_crd & {2{~hipi_sel_sync}}) | (chi_awcrd & {2{hipi_sel_sync}});
    assign mrg_aw_crd_v = (rem_aw_crd_v & ~hipi_sel_sync) | (chi_awcrd_v & hipi_sel_sync);
    assign mrg_ar_crd   = (rem_ar_crd & {2{~hipi_sel_sync}}) | (chi_arcrd & {2{hipi_sel_sync}});
    assign mrg_ar_crd_v = (rem_ar_crd_v & ~hipi_sel_sync) | (chi_arcrd_v & hipi_sel_sync);
    assign mrg_w_crd    = (rem_w_crd  & {2{~hipi_sel_sync}}) | (chi_wcrd  & {2{hipi_sel_sync}});
    assign mrg_w_crd_v  = (rem_w_crd_v  & ~hipi_sel_sync) | (chi_wcrd_v  & hipi_sel_sync);
    assign mrg_r_crd    = (rem_r_crd  & {2{~hipi_sel_sync}}) | (chi_rcrd  & {2{hipi_sel_sync}});
    assign mrg_r_crd_v  = (rem_r_crd_v  & ~hipi_sel_sync) | (chi_rcrd_v  & hipi_sel_sync);
    assign mrg_b_crd    = (rem_b_crd  & {2{~hipi_sel_sync}}) | (chi_bcrd  & {2{hipi_sel_sync}});
    assign mrg_b_crd_v  = (rem_b_crd_v  & ~hipi_sel_sync) | (chi_bcrd_v  & hipi_sel_sync);
`endif

    // ==========================================================================
    // RX async FIFO（prcl_clk_cg → aclk；5 路）
    // ==========================================================================
    p_axis_async_fifo #(.DATA_W(AX_WIDTH), .DEPTH(`PRCL_ASYNC_RXFIFO_DEPTH)) i_aw_rxfifo (
        .s_clk(prcl_clk_cg), .m_clk(aclk), .s_rstn(paif_rstn_i), .m_rstn(aresetn),
`ifdef HIPI_GB
        .s_data(mrg_aw_bus), .s_valid(mrg_aw_valid), .s_ready(rx_aw_sready),   // 双引擎模式门控合并
`else
        .s_data(unp_aw_bus), .s_valid(unp_aw_valid), .s_ready(rx_aw_sready),
`endif
        .m_data(rx_aw_mdata), .m_valid(rx_aw_mvalid), .m_ready(rx_aw_mready),
        .crd(rx_aw_crd), .crd_valid(rx_aw_crd_v)
    );
    p_axis_async_fifo #(.DATA_W(AX_WIDTH), .DEPTH(`PRCL_ASYNC_RXFIFO_DEPTH)) i_ar_rxfifo (
        .s_clk(prcl_clk_cg), .m_clk(aclk), .s_rstn(paif_rstn_i), .m_rstn(aresetn),
`ifdef HIPI_GB
        .s_data(mrg_ar_bus), .s_valid(mrg_ar_valid), .s_ready(rx_ar_sready),   // 双引擎模式门控合并
`else
        .s_data(unp_ar0_bus), .s_valid(unp_ar0_valid), .s_ready(rx_ar_sready),
`endif
        .m_data(rx_ar_mdata), .m_valid(rx_ar_mvalid), .m_ready(rx_ar_mready),
        .crd(rx_ar_crd), .crd_valid(rx_ar_crd_v)
    );
    p_axis_async_fifo #(.DATA_W(W_WIDTH), .DEPTH(`PRCL_ASYNC_RXFIFO_DEPTH)) i_w_rxfifo (
        .s_clk(prcl_clk_cg), .m_clk(aclk), .s_rstn(paif_rstn_i), .m_rstn(aresetn),
`ifdef HIPI_GB
        .s_data(mrg_w_bus), .s_valid(mrg_w_valid), .s_ready(rx_w_sready),   // 双引擎模式门控合并
`else
        .s_data(unp_w_bus), .s_valid(unp_w_valid), .s_ready(rx_w_sready),
`endif
        .m_data(rx_w_mdata), .m_valid(rx_w_mvalid), .m_ready(rx_w_mready),
        .crd(rx_w_crd), .crd_valid(rx_w_crd_v)
    );
    p_axis_async_fifo #(.DATA_W(R_WIDTH), .DEPTH(`PRCL_ASYNC_RXFIFO_DEPTH)) i_r_rxfifo (
        .s_clk(prcl_clk_cg), .m_clk(aclk), .s_rstn(paif_rstn_i), .m_rstn(aresetn),
`ifdef HIPI_GB
        .s_data(mrg_r_bus), .s_valid(mrg_r_valid), .s_ready(rx_r_sready),   // 双引擎模式门控合并
`else
        .s_data(unp_r_bus), .s_valid(unp_r_valid), .s_ready(rx_r_sready),
`endif
        .m_data(rx_r_mdata), .m_valid(rx_r_mvalid), .m_ready(rx_r_mready),
        .crd(rx_r_crd), .crd_valid(rx_r_crd_v)
    );
    p_axis_async_fifo #(.DATA_W(B_WIDTH), .DEPTH(`PRCL_ASYNC_RXFIFO_DEPTH)) i_b_rxfifo (
        .s_clk(prcl_clk_cg), .m_clk(aclk), .s_rstn(paif_rstn_i), .m_rstn(aresetn),
`ifdef HIPI_GB
        .s_data(mrg_b_bus), .s_valid(mrg_b_valid), .s_ready(rx_b_sready),   // 双引擎模式门控合并
`else
        .s_data(unp_b0_bus), .s_valid(unp_b0_valid), .s_ready(rx_b_sready),
`endif
        .m_data(rx_b_mdata), .m_valid(rx_b_mvalid), .m_ready(rx_b_mready),
        .crd(rx_b_crd), .crd_valid(rx_b_crd_v)
    );

    // ==========================================================================
    // p_ovrflow_det（RX FIFO 写侧溢出检测，prcl_clk_cg 域；→ CSR aw0/ar0/w0/r0/b0 ovrflow_intr）
    //   ready=RX FIFO 写侧 ready（0=满），valid=unpack 写请求（~ready & valid 时拉溢出中断）
    // ==========================================================================
    p_ovrflow_det i_aw_ovrflow_det (
        .clk(prcl_clk_cg), .rstn(paif_rstn_i),
        .ready(rx_aw_sready), .valid(unp_aw_valid),
        .intr(aw0_ovf_intr), .clr(csr_aw0_ovrflow_clr)
    );
    p_ovrflow_det i_ar_ovrflow_det (
        .clk(prcl_clk_cg), .rstn(paif_rstn_i),
        .ready(rx_ar_sready), .valid(unp_ar0_valid),
        .intr(ar0_ovf_intr), .clr(csr_ar0_ovrflow_clr)
    );
    p_ovrflow_det i_w_ovrflow_det (
        .clk(prcl_clk_cg), .rstn(paif_rstn_i),
        .ready(rx_w_sready), .valid(unp_w_valid),
        .intr(w0_ovf_intr), .clr(csr_w0_ovrflow_clr)
    );
    p_ovrflow_det i_r_ovrflow_det (
        .clk(prcl_clk_cg), .rstn(paif_rstn_i),
        .ready(rx_r_sready), .valid(unp_r_valid),
        .intr(r0_ovf_intr), .clr(csr_r0_ovrflow_clr)
    );
    p_ovrflow_det i_b_ovrflow_det (
        .clk(prcl_clk_cg), .rstn(paif_rstn_i),
        .ready(rx_b_sready), .valid(unp_b0_valid),
        .intr(b0_ovf_intr), .clr(csr_b0_ovrflow_clr)
    );

    // ==========================================================================
    // 用户侧 unbond（仅 m0/s0 槽 active；m1~3/s1~3 输入 tie0）
    // ==========================================================================
    p_axi4_unbond #(
        .AXI_DW   (AXI_DW),
        .AX_WIDTH (AX_WIDTH),
        .W_WIDTH  (W_WIDTH),
        .R_WIDTH  (R_WIDTH),
        .B_WIDTH  (B_WIDTH)
    ) u_unbond (
        // m0 AW（request 出，经 m_axi ots_chk 反压））；m1~3 tie0
        .aw0_rx_bus    (il_aw_rx_bus), .aw0_rx_valid(m_aw_ots_mvalid), .aw0_rx_ready(m_aw_ots_mready),
        .aw1_rx_bus    (107'd0), .aw1_rx_valid(1'b0), .aw1_rx_ready(),
        .aw2_rx_bus    (107'd0), .aw2_rx_valid(1'b0), .aw2_rx_ready(),
        .aw3_rx_bus    (107'd0), .aw3_rx_valid(1'b0), .aw3_rx_ready(),
        .ar0_rx_bus    (il_ar_rx_bus), .ar0_rx_valid(m_ar_ots_mvalid), .ar0_rx_ready(m_ar_ots_mready),
        .ar1_rx_bus    (107'd0), .ar1_rx_valid(1'b0), .ar1_rx_ready(),
        .ar2_rx_bus    (107'd0), .ar2_rx_valid(1'b0), .ar2_rx_ready(),
        .ar3_rx_bus    (107'd0), .ar3_rx_valid(1'b0), .ar3_rx_ready(),
        .w0_rx_bus     (il_w_rx_bus), .w0_rx_valid(il_w_rx_valid), .w0_rx_ready(il_w_rx_ready),
        .w1_rx_bus     ({W_WIDTH{1'b0}}), .w1_rx_valid(1'b0), .w1_rx_ready(),
        .w2_rx_bus     ({W_WIDTH{1'b0}}), .w2_rx_valid(1'b0), .w2_rx_ready(),
        .w3_rx_bus     ({W_WIDTH{1'b0}}), .w3_rx_valid(1'b0), .w3_rx_ready(),
        // s0 R（response 出）；s1~3 tie0
        .r0_rx_bus     (il_r_rx_bus), .r0_rx_valid(il_r_rx_valid), .r0_rx_ready(il_r_rx_ready),
        .r1_rx_bus     ({R_WIDTH{1'b0}}), .r1_rx_valid(1'b0), .r1_rx_ready(),
        .r2_rx_bus     ({R_WIDTH{1'b0}}), .r2_rx_valid(1'b0), .r2_rx_ready(),
        .r3_rx_bus     ({R_WIDTH{1'b0}}), .r3_rx_valid(1'b0), .r3_rx_ready(),
        .b0_rx_bus     (il_b_rx_bus), .b0_rx_valid(il_b_rx_valid), .b0_rx_ready(il_b_rx_ready),
        .b1_rx_bus     (16'd0), .b1_rx_valid(1'b0), .b1_rx_ready(),
        .b2_rx_bus     (16'd0), .b2_rx_valid(1'b0), .b2_rx_ready(),
        .b3_rx_bus     (16'd0), .b3_rx_valid(1'b0), .b3_rx_ready(),
        // m0/s0 用户侧通道 → top 端口
        .m0_axi_awid   (m_axi_awid),   .m0_axi_awaddr (m_axi_awaddr), .m0_axi_awlen (m_axi_awlen),
        .m0_axi_awsize (m_axi_awsize), .m0_axi_awburst(m_axi_awburst),.m0_axi_awlock(m_axi_awlock),
        .m0_axi_awcache(m_axi_awcache),.m0_axi_awprot (m_axi_awprot), .m0_axi_awqos (m_axi_awqos),
        .m0_axi_awregion(m_axi_awregion),.m0_axi_awuser(m_axi_awuser), .m0_axi_awvalid(m_axi_awvalid),
        .m0_axi_awready(m_axi_awready),
        .m0_axi_arid   (m_axi_arid),   .m0_axi_araddr (m_axi_araddr), .m0_axi_arlen (m_axi_arlen),
        .m0_axi_arsize (m_axi_arsize), .m0_axi_arburst(m_axi_arburst),.m0_axi_arlock(m_axi_arlock),
        .m0_axi_arcache(m_axi_arcache),.m0_axi_arprot (m_axi_arprot), .m0_axi_arqos (m_axi_arqos),
        .m0_axi_arregion(m_axi_arregion),.m0_axi_aruser(m_axi_aruser), .m0_axi_arvalid(m_axi_arvalid),
        .m0_axi_arready(m_axi_arready),
        .m0_axi_wdata  (m_axi_wdata), .m0_axi_wstrb (m_axi_wstrb),
        .m0_axi_wuser  (m_axi_wuser), .m0_axi_wlast (m_axi_wlast), .m0_axi_wvalid(m_axi_wvalid),
        .m0_axi_wready (m_axi_wready),
        .s0_axi_rid    (s_axi_rid), .s0_axi_rdata (s_axi_rdata), .s0_axi_rresp(s_axi_rresp),
        .s0_axi_ruser  (s_axi_ruser),.s0_axi_rlast(s_axi_rlast),.s0_axi_rvalid(s_axi_rvalid),
        .s0_axi_rready (s_axi_rready),
        .s0_axi_bid    (s_axi_bid), .s0_axi_bresp(s_axi_bresp), .s0_axi_buser(s_axi_buser),
        .s0_axi_bvalid (s_axi_bvalid),.s0_axi_bready(s_axi_bready),
        // 未用槽输入 tie-off（symmetric 单口设计，仅 s0/m0/rp0 active；见 spec §5.5）
        .m1_axi_awready(1'b0), .m1_axi_arready(1'b0), .m1_axi_wready(1'b0),
        .m2_axi_awready(1'b0), .m2_axi_arready(1'b0), .m2_axi_wready(1'b0),
        .m3_axi_awready(1'b0), .m3_axi_arready(1'b0), .m3_axi_wready(1'b0),
        .s1_axi_rready(1'b0),  .s1_axi_bready(1'b0),
        .s2_axi_rready(1'b0),  .s2_axi_bready(1'b0),
        .s3_axi_rready(1'b0),  .s3_axi_bready(1'b0)
    );

    // ==========================================================================
    // PAIF 侧回环（local_loop，p_paif_loop；prcl_clk_cg 域，sel=local_loop_sync；2026-09-23 语义纠正）
    //   s0 = pack TX（protocol tx）→ m0 = link tx（paif_txdata）
    //   s1 = link rx（paif_rxdata）→ m1 = unpack RX（protocol rx）
    //   直通（sel=0）：协议 tx 直出 link、link rx 直入协议 rx（同原直接赋值）。
    //   本端环回（sel=1）：s0(pack TX) → m1(unpack RX)，本地 TX 回环至本地 RX，绕过 Link。
    //   PAIF_DW 取模块参数 PAIF_DW（2026-09-23：与顶层 paif_tx/rx 口及 pack_txdata/unpack_rxdata 位宽一致）。
    p_paif_loop #(
        .PAIF_DW        (PAIF_DW)
    ) u_paif_loop (
        .clk            (prcl_clk_cg),
        .rstn           (paif_rstn_i),
        .sel            (local_loop_sync),   // 2026-09-23：p_paif_loop = 本端环回
`ifdef HIPI_GB
        // slave 0 = 协议 TX（u_tx_mux 合并 pack/pkgpipe 后输出）
        .s0_paif_data   (loop_s0_data),
        .s0_paif_valid  (loop_s0_valid),
        .s0_paif_ready  (loop_s0_ready),
`else
        // slave 0 = 协议 TX（pack 输出）
        .s0_paif_data   (pack_txdata),
        .s0_paif_valid  (pack_txvalid),
        .s0_paif_ready  (pack_txready),
`endif
        // slave 1 = link RX（顶层 paif_rx）
        .s1_paif_data   (paif_rxdata),
        .s1_paif_valid  (paif_rxvalid),
        .s1_paif_ready  (paif_rxready),
        // master 0 = link TX（顶层 paif_tx）
        .m0_paif_data   (paif_txdata),
        .m0_paif_valid  (paif_txvalid),
        .m0_paif_ready  (paif_txready),
`ifdef HIPI_GB
        // master 1 = 协议 RX（u_rx_demux 分流 unpack/ch_identify）
        .m1_paif_data   (loop_m1_data),
        .m1_paif_valid  (loop_m1_valid),
        .m1_paif_ready  (loop_m1_ready)
`else
        // master 1 = 协议 RX（unpack 输入）
        .m1_paif_data   (unpack_rxdata),
        .m1_paif_valid  (unpack_rxvalid),
        .m1_paif_ready  (unpack_rxready)
`endif
    );

    // ==========================================================================
    // ==========================================================================
    // 回环（LOOPBACK）已集成说明
    //   CSR 0x500 提供两级回环：
    //   - remote_loop（p_axi_loop，bond 层，aclk 域，sel=remote_loop_sync）：在 bond bus
    //     （AX/W/R/B）层级，s0=bond TX(AXI IN) → m0=TX FIFO；s1=RX FIFO → m1=unbond
    //     （AW/AR 经 m_axi ots_chk、W/R/B 直连 unbond）。远端环回（sel=1）：对端发来的
    //     数据经 s1(RX) 回灌 m0(TX) 发回对端、m1 关闭。W/R lite 位由 il_*_lit_tx
    //     外部复用（回环时置 0，仅支持非 lite 回环，见头部 FIXME ①）。
    //   - local_loop（p_paif_loop，PAIF 层，prcl_clk_cg 域，sel=local_loop_sync）：
    //     s0=pack TX(protocol tx) → m0=link tx(paif_tx)；s1=link rx(paif_rx) →
    //     m1=unpack RX(protocol rx)。本端环回（sel=1）：本端 TX(s0) 回环至本端 RX(m1)，
    //     绕过 Link。
    //   sel 均为组合使用（p_axi_loop 全组合 / p_paif_loop 时序），同步目标域为各自桥接的
    //   远端数据域（2026-09-23：local_loop_sync→prcl_clk、remote_loop_sync→aclk，见 CDC 段）。
    // ==========================================================================

    // ==========================================================================
    // 断言与检查（可选）
    // ==========================================================================
    // 暂无（待 RTL 验证阶段补充）

endmodule
