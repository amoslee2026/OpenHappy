// =============================================================================
// module name：p_axi4_rwch_pack
// author: OpenHappy Team
// MAS/spec：p_axi4_rwch_pack_mas.md / p_axi4_rwch_pack_spec.md
// 功能概述: 对称模式(p_axi4) per-lane PAIF TX flit 封装模块。合并写方向
//   (AW/AR/W + R/B Credit, 继承 p_axi4_wch_pack) 与读方向 (R/B + AW/AR/W Credit,
//   继承 p_axi4_rch_pack) 于单一模块：W 侧与 R 侧经轮询仲裁二选一填入共享 buf_r，
//   再打包为 256Byte Flit 输出至 PAIF TX。b_vc / gather_vc / SP 内嵌 Credit 均覆盖。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   2026-08-27  OpenHappy Team  初始版本：wch_pack + rch_pack 合并 + W/R 轮询仲裁(rw_poll)
//                     共享 buf_r/buf_cnt_r/flit_phase_r；继承 S1(buf_cnt 回绕修正)/S2(busy 语义)
//   2026-08-27  OpenHappy Team  gather 随任一侧入队：填 w_vc 或 r_vc 时，gather_vc(AW/AR/b_vc
//                     聚合)均拼入 buf_r 高位；aw/ar/写b_vc ready 改由 gather_send(任意
//                     winning 拍)门控；VC_IN_WIDTH 扩为 GATHER+R_VC(+AX) 覆盖 R 侧带
//                     gather、BUFFER_WIDTH=SHIFT+VC_IN_WIDTH 同步；vc_in_cnt 加 gather 项
//   2026-08-27  OpenHappy Team  S4 修复：vc_in_cnt 三项（gather/w_ready/r_enq）由 OR 改 ADD——
//                     gather 与数据 VC 同拍入队时 OR 位重叠少计（如 gather40|W38=46 实 78）
//                     改加法求和，7bit 容纳 121<127
//   2026-08-27  OpenHappy Team  仲裁改标准 round-robin：rw_poll 优先级FSM → rr_turn 轮转指针，
//                     得手后 turn 必轮转（持续双请求 WRWRWR 严格 50/50）；授权固定优先级
//                     turn 指向侧优先、另一侧让位；单侧请求无饥饿
//   2026-08-27  OpenHappy Team  方案2：R 侧一次入队最大 = R_VC + GATHER_VC。读方向 B 数据/credit
//                     (r_b_vc) 不再单独占 +AX 低位槽，改与写方向 R/B Credit(w_b_vc) 复用
//                     gather_vc 的 b_vc 槽（b_slot_vc/b_slot_valid 按入队侧二选一，互斥）。
//                     统一数据区 DATA_VC=WFULL(>R_VC)，VC_IN=GATHER+WFULL 覆盖侧；
//                     r_vc_in 数据区仅 r_vc，B_VC_CNT 移除（读 B 计入 gather_vc_cnt）
//   2026-08-27  OpenHappy Team  首尾相接修正：w_vc/r_vc 与 gather_vc 位拼紧贴、不留空隙——补齐零
//                     由数据区高位移至 gather 之上(最高位)，对齐参考 wch_pack 布局；
//                     S5：r_sp_send 改受 r_ready(r_valid&r_enq) 门控，防 W 得手时过早 ack 读侧 SP Credit
//   2026-08-27  OpenHappy Team  SP 无方向绑定：统一 SP 字段（20bit TYPE 自描述）——t码0/1/2/3 由任一
//                     数据 VC(w_vc 或 r_lite_vc)承载均合法；去掉“w_vc 仅 type1、r_lite 仅 type0/2/3”
//                     旧绑定。统一优先级(type0满>type1满>type2满>B)选择，被 b_vc 承载的类型经
//                     sp_type_ok 屏蔽避免重复 ack；credit ready 改按 SP 类型 + b_vc 双路 ack
//   2026-09-20  OpenHappy Team  [T1+T2] 时序微优化（逐拍位等价，除 T2a zombie gather 修复）：
//                      T1 移位量打拍（lsft_amt_hld/dec_r 每拍无条件装载 cnt_nxt/S1_dec(cnt_nxt)，
//                      桶形 select 改平坦寄存器+1 级 2:1，S1 修正移出关键锥）；
//                      T2a gather_vc_in term3 门控补 buf_ready（消除 缓冲满+gather 待发+tx 拍
//                      的幻影 gather 写入，vc_in 与 vc_in_cnt 对齐）；
//                      T2b buf_r 无条件写（非更新拍 D 自然退化保持，去 1872-bit 更新/保持 MUX）
//   2026-09-28  OpenHappy Team  输出旁路寄存：内部组合 paif_txdata/paif_txvalid 先经 p_data_byp
//                      （bypass-on-ready）后输出到外部端口；内部流控改用 byp s_ready
//                      (tx_paif_ready) 替代原外部 paif_txready 直连，m_ready = paif_txready。
//                      端口名/对外接口不变。
// =============================================================================
// NOTE: 本模块按 spec/MAS §4-§6 实现。VC/PRF/SP 字段位布局依据：
//   - AW_VC/AR_VC[127:0] = {PRF[14], bus[107], RP[2], CN[5]} (HIPI_AXI4_Next_IP.md §PRF)
//   - PRF = {pad[2], vc_num[3], b_vc_v[1], ar_v[4], pad[3], aw_v[1]}
//   - 聚合包顺序 {B_VC, AR3, AR2, AR1, AR0, AW_VC}（AW 在低位先发，紧凑递补）
//   - W_full_VC = {SP[20], W[W_WIDTH], RP[2], CN_W_FULL}
//   - R_full_VC = {8'd0, R[R_WIDTH], RP[2], CN_R_FULL}；R_lite_VC = {SP, r_lite_bus, RP, CN_R_LITE}
//   - B_VC 布局（写方向承载 R/B Credit；读方向承载 B 数据 + AW/AR/W Credit）
//   - SP: 写方向 sp_type1=R/B Credit；读方向 sp_type0=AW/W、sp_type2=AR、sp_type3=B
//   端口/宏约定同目录（p_axi4_wch_pack / p_axi4_rch_pack）一致：leaf 子模块
//   不自行 `include，axi_defines.vh 由顶层 wrapper/filelist 编译期提供。
// =============================================================================

module p_axi4_rwch_pack #(
    parameter PAIF_DW = 512,        // PAIF 数据位宽，与 AXI4 数据位宽相同（512 或 1024）
    parameter [1:0] W_RP = 2'd0,    // 本 lane W/AW 通道 RP 号（静态配置）
    parameter [1:0] R_RP = 2'd0     // 本 lane R 通道 RP 号（静态配置）
) (
    // ------------------------------------------------------------------
    // 时钟与复位
    // ------------------------------------------------------------------
    input  wire                       clk,
    input  wire                       rstn,

    // ------------------------------------------------------------------
    // 状态
    // ------------------------------------------------------------------
    output wire                       busy,

    // ------------------------------------------------------------------
    // AW 通道（本 lane W_RP）
    // ------------------------------------------------------------------
    input  wire [106:0]               aw_bus,
    input  wire                       aw_valid,
    output wire                       aw_ready,

    // ------------------------------------------------------------------
    // AR 通道（4 个 RP）
    // ------------------------------------------------------------------
    input  wire [106:0]               ar_rp0_bus,
    input  wire                       ar_rp0_valid,
    output wire                       ar_rp0_ready,
    input  wire [106:0]               ar_rp1_bus,
    input  wire                       ar_rp1_valid,
    output wire                       ar_rp1_ready,
    input  wire [106:0]               ar_rp2_bus,
    input  wire                       ar_rp2_valid,
    output wire                       ar_rp2_ready,
    input  wire [106:0]               ar_rp3_bus,
    input  wire                       ar_rp3_valid,
    output wire                       ar_rp3_ready,

    // ------------------------------------------------------------------
    // W 通道（本 lane W_RP）
    // ------------------------------------------------------------------
    input  wire [PAIF_DW+PAIF_DW/8+4:0] w_bus,
    input  wire                       w_lite,
    input  wire                       w_valid,
    output wire                       w_ready,

    // ------------------------------------------------------------------
    // R 通道（本 lane R_RP）
    // ------------------------------------------------------------------
    input  wire [PAIF_DW+17-1:0]      r_bus,
    input  wire                       r_lite,
    input  wire                       r_valid,
    output wire                       r_ready,

    // ------------------------------------------------------------------
    // B 通道（4 个 RP）
    // ------------------------------------------------------------------
    input  wire [15:0]                b_rp0_bus,
    input  wire                       b_rp0_valid,
    output wire                       b_rp0_ready,
    input  wire [15:0]                b_rp1_bus,
    input  wire                       b_rp1_valid,
    output wire                       b_rp1_ready,
    input  wire [15:0]                b_rp2_bus,
    input  wire                       b_rp2_valid,
    output wire                       b_rp2_ready,
    input  wire [15:0]                b_rp3_bus,
    input  wire                       b_rp3_valid,
    output wire                       b_rp3_ready,

    // ------------------------------------------------------------------
    // AW Credit RP0~3（2bit 编码 0/1/4/8）
    // ------------------------------------------------------------------
    input  wire [1:0]                 awcrd_rp0,
    output wire                       awcrd_rp0_ready,
    input  wire [1:0]                 awcrd_rp1,
    output wire                       awcrd_rp1_ready,
    input  wire [1:0]                 awcrd_rp2,
    output wire                       awcrd_rp2_ready,
    input  wire [1:0]                 awcrd_rp3,
    output wire                       awcrd_rp3_ready,

    // ------------------------------------------------------------------
    // W Credit RP0~3
    // ------------------------------------------------------------------
    input  wire [1:0]                 wcrd_rp0,
    output wire                       wcrd_rp0_ready,
    input  wire [1:0]                 wcrd_rp1,
    output wire                       wcrd_rp1_ready,
    input  wire [1:0]                 wcrd_rp2,
    output wire                       wcrd_rp2_ready,
    input  wire [1:0]                 wcrd_rp3,
    output wire                       wcrd_rp3_ready,

    // ------------------------------------------------------------------
    // AR Credit RP0~3
    // ------------------------------------------------------------------
    input  wire [1:0]                 arcrd_rp0,
    output wire                       arcrd_rp0_ready,
    input  wire [1:0]                 arcrd_rp1,
    output wire                       arcrd_rp1_ready,
    input  wire [1:0]                 arcrd_rp2,
    output wire                       arcrd_rp2_ready,
    input  wire [1:0]                 arcrd_rp3,
    output wire                       arcrd_rp3_ready,

    // ------------------------------------------------------------------
    // R Credit RP0~3（写方向通过 SP 内嵌）
    // ------------------------------------------------------------------
    input  wire [1:0]                 rcrd_rp0,
    output wire                       rcrd_rp0_ready,
    input  wire [1:0]                 rcrd_rp1,
    output wire                       rcrd_rp1_ready,
    input  wire [1:0]                 rcrd_rp2,
    output wire                       rcrd_rp2_ready,
    input  wire [1:0]                 rcrd_rp3,
    output wire                       rcrd_rp3_ready,

    // ------------------------------------------------------------------
    // B Credit RP0~3（写方向通过 SP 内嵌）
    // ------------------------------------------------------------------
    input  wire [1:0]                 bcrd_rp0,
    output wire                       bcrd_rp0_ready,
    input  wire [1:0]                 bcrd_rp1,
    output wire                       bcrd_rp1_ready,
    input  wire [1:0]                 bcrd_rp2,
    output wire                       bcrd_rp2_ready,
    input  wire [1:0]                 bcrd_rp3,
    output wire                       bcrd_rp3_ready,

    // ------------------------------------------------------------------
    // PAIF TX 接口（per-lane）
    //   paif_txdata/paif_txvalid 内部先经 p_data_byp 旁路寄存后输出；
    //   paif_txready 为外部 m 侧 ready，内部由 u_paif_tx_byp.s_ready 反馈（见下）
    // ------------------------------------------------------------------
    output wire [PAIF_DW-1:0]         paif_txdata,
    output wire                       paif_txvalid,
    input  wire                       paif_txready
);

    // ==========================================================================
    // 参数定义（由 PAIF_DW 派生，依据 spec/MAS §3.1）
    // ==========================================================================
    localparam AX_WIDTH        = 107;                        // AW/AR 合并信号位宽
    localparam W_WIDTH         = PAIF_DW + PAIF_DW/8 + 5;    // W 通道合并信号位宽
    localparam R_WIDTH         = PAIF_DW + 17;               // R 通道合并信号位宽
    localparam AX_VC_WIDTH     = 128;                        // AW/AR/B VC 位宽
    localparam WFULL_VC_WIDTH  = PAIF_DW + PAIF_DW/8 + 32;   // W Full VC 位宽
    localparam WLITE_VC_WIDTH  = PAIF_DW + 32;               // W Lite VC 位宽
    localparam R_VC_WIDTH      = PAIF_DW + 32;               // R Full/Lite VC 位宽
    localparam R_LITE_WIDTH    = PAIF_DW + 5;                // R lite 合并信号位宽
    localparam GRAN_WIDTH      = (PAIF_DW*32)/1024;          // 移位粒度
    localparam SHIFT_WIDTH     = PAIF_DW - GRAN_WIDTH;       // 单拍 PAIF 取自 buf 的 payload 位宽
    localparam GATHER_VC_WIDTH = 6*AX_VC_WIDTH;              // 聚合包位宽（1 AW + 4 AR + 1 b_vc）
    // 统一数据 VC 低位区宽度 = max(W 侧 WFULL, R 侧 R_VC) = WFULL
    //   R 侧一次入队载荷 = R_VC + GATHER（读方向 B 数据/credit 复用 gather 的 b_vc 槽，不再占 +AX）
    //   W 侧一次入队载荷 = WFULL + GATHER（w_full 数据 + 聚合）
    //   WFULL=PAIF+PAIF/8+32 > R_VC=PAIF+32，故统一 VC_IN = GATHER + WFULL 覆盖两侧
    localparam DATA_VC_WIDTH   = WFULL_VC_WIDTH;
    localparam VC_IN_WIDTH     = GATHER_VC_WIDTH + DATA_VC_WIDTH;
    // buf_r 位宽：1 SHIFT payload + 1 拍最大入队（gather + 数据 VC）
    localparam BUFFER_WIDTH    = SHIFT_WIDTH + VC_IN_WIDTH;
    localparam WFULL_VC_CNT    = WFULL_VC_WIDTH/GRAN_WIDTH;
    localparam WLITE_VC_CNT    = WLITE_VC_WIDTH/GRAN_WIDTH;
    localparam AX_VC_CNT       = AX_VC_WIDTH/GRAN_WIDTH;
    localparam R_VC_CNT        = R_VC_WIDTH/GRAN_WIDTH;
    localparam FLT_PHS_WIDTH   = 2048/PAIF_DW;               // 256Byte Flit 拍数（512→4，1024→2）

    // 移位分级常量（barrel shifter 各级移位量，单位 bit）
    localparam SFT_GRAN_1  = GRAN_WIDTH;
    localparam SFT_GRAN_2  = 2*GRAN_WIDTH;
    localparam SFT_GRAN_4  = 4*GRAN_WIDTH;
    localparam SFT_GRAN_8  = 8*GRAN_WIDTH;
    localparam SFT_GRAN_16 = 16*GRAN_WIDTH;

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================

    // --------------------------------------------------------------------------
    // W/R 轮询仲裁（round-robin 轮转指针）
    // --------------------------------------------------------------------------
    localparam TURN_W = 1'b0;    // 本轮 W 优先（turn 指向 W）
    localparam TURN_R = 1'b1;    // 本轮 R 优先（turn 指向 R）
    reg        rr_turn_r;        // round-robin 轮转指针：本轮谁优先
    wire       rr_turn_next;     // 轮转后的指针
    wire       w_req;            // W 侧请求 & buf_ready（w_valid，不含纯 gather）
    wire       r_req;            // R 侧请求 & buf_ready（r_valid）
    wire       w_enq;            // 本轮 W 侧获准入队
    wire       r_enq;            // 本轮 R 侧获准入队

    // --------------------------------------------------------------------------
    // 写方向 Credit 编码（W 侧承载 R/B Credit）
    // --------------------------------------------------------------------------
    wire [3:0] rcrd_full;
    wire [3:0] bcrd_full;
    wire [3:0] rcrd_valid;
    wire [3:0] bcrd_valid;

    // --------------------------------------------------------------------------
    // 统一 SP 字段（20bit，TYPE 自描述）——任意数据 VC（w_vc 或 r_lite_vc）均可承载
    //   全部 4 种 TYPE，无方向绑定：type0=AW/W Credit, type1=R/B Credit, type2=AR
    //   Credit, type3=B。由统一优先级选出一个类型填充 SP，随当拍入队的数据 VC 下发。
    // --------------------------------------------------------------------------
    wire [19:0] sp_type0;
    wire [19:0] sp_type1;
    wire [19:0] sp_type2;
    wire [19:0] sp_type3;
    wire [3:0]  sp_hpri_sts;
    wire [3:0]  sp_hpri_sel;
    wire [2:0]  sp_lpri_sts;
    wire [2:0]  sp_lpri_sel;
    wire [3:0]  sp_sel;          // 选中 SP 类型（one-hot，按 type 索引）
    wire [19:0] sp_type_hpri;
    wire [19:0] sp_type_lpri;
    wire [19:0] sp_field;        // 最终 SP 字段
    wire        sp_valid;        // 有任一 SP 类型待发
    wire        sp_data_send;    // SP 实际随数据 VC(w_vc 或 r_lite_vc)入队

    // --------------------------------------------------------------------------
    // Credit 编码（写方向 R/B；读方向 AW/W/AR）
    // --------------------------------------------------------------------------
    wire [3:0] arcrd_full;
    wire [3:0] wcrd_full;
    wire [3:0] arcrd_valid;
    wire [3:0] wcrd_valid;
    wire [3:0] awcrd_valid;
    wire [3:0] b_rpv;
    wire [15:0] b_bus_sel;
    wire [1:0]  b_rp_sel;

    // --------------------------------------------------------------------------
    // 写方向 PRF / 各 VC
    // --------------------------------------------------------------------------
    wire [13:0] prf_field;
    wire [2:0]  vc_num;
    wire [AX_VC_WIDTH-1:0]   aw_vc;
    wire [AX_VC_WIDTH-1:0]   ar_rp0_vc;
    wire [AX_VC_WIDTH-1:0]   ar_rp1_vc;
    wire [AX_VC_WIDTH-1:0]   ar_rp2_vc;
    wire [AX_VC_WIDTH-1:0]   ar_rp3_vc;
    wire [GATHER_VC_WIDTH-1:0] gather_vc;
    wire [WFULL_VC_WIDTH-1:0]  w_full_vc;
    wire [WLITE_VC_WIDTH-1:0]  w_lite_vc;

    // --------------------------------------------------------------------------
    // 读方向各 VC / 触发
    // --------------------------------------------------------------------------
    wire [R_LITE_WIDTH-1:0]  r_lite_bus;
    wire [R_VC_WIDTH-1:0]    r_full_vc;
    wire [R_VC_WIDTH-1:0]    r_lite_vc;
    wire [R_VC_WIDTH-1:0]    r_vc;
    wire [AX_VC_WIDTH-1:0]   b_vc;           // 统一 B_VC（权威格式：5 条 credit lane + B 数据）
    wire                     b_avail;        // 统一 b_vc 触发（有任一 B/credit 待发）
    wire                     b_dly_set;      // B 等待超时（强制 B 经 SP type3 下发）
    reg  [1:0] b_dly_cnt_r;

    // 统一 b_vc 触发 / 发送（SP 与 b_vc 二选一：有数据 VC 走 SP，无数据 VC 才走 b_vc）
    wire b_not_one;
    wire b_vc_valid;
    wire b_vc_send;           // 统一 b_vc 实际发送（仅无 SP 时，gather 门控）
    wire gather_valid_w;      // 写方向聚合包有效
    wire gather_send;         // 聚合包实际发送（w_enq 或 r_enq 任一得手且 gather 有效时拼入高位）
    wire [6:0] gather_vc_cnt; // 聚合包占 GRAN 个数

    // vc_in / buf 移位 / 计数（W/R 共享）
    wire [VC_IN_WIDTH-1:0]   w_vc_in;        // 写方向入队原始数据
    wire [VC_IN_WIDTH-1:0]   r_vc_in;        // 读方向入队原始数据（零扩至 VC_IN_WIDTH）
    wire [VC_IN_WIDTH-1:0]   gather_vc_in;
    wire [VC_IN_WIDTH-1:0]   vc_in;          // 轮询选中侧的入队数据
    wire [BUFFER_WIDTH-1:0]  vc_in_ext;
    wire [4:0]               lsft_amt;
    wire [BUFFER_WIDTH-1:0]  vc_in_lsft;
    wire [BUFFER_WIDTH-1:0]  buf_rsft;
    wire [6:0]               vc_in_cnt;
    wire        buf_over_paif;
    wire        buf_ready;
    wire [6:0]  buf_cnt_after_out;
    reg  [BUFFER_WIDTH-1:0]  buf_r;
    reg  [6:0]               buf_cnt_r;
    reg  [FLT_PHS_WIDTH-1:0] flit_phase_r;

    // --------------------------------------------------------------------------
    // paif_tx 输出旁路寄存（p_data_byp）信号
    //   原组合 paif_txdata/paif_txvalid 改为内部 tx_paif_data/tx_paif_valid（byp s 侧），
    //   byp s_ready(tx_paif_ready) 反馈驱动内部流控（替代原外部 paif_txready 直连），
    //   byp m 侧直连外部端口（m_ready = paif_txready）
    // --------------------------------------------------------------------------
    wire [PAIF_DW-1:0]         tx_paif_data;      // byp s_data：原 paif_txdata 组合输出
    wire                       tx_paif_valid;    // byp s_valid：原 paif_txvalid(=|flit_phase_r)
    wire                       tx_paif_ready;    // byp s_ready(rdy_r)：内部流控用

    // ==========================================================================
    // 组合逻辑 —— 写方向（继承 p_axi4_wch_pack）
    // ==========================================================================

    // --------------------------------------------------------------------------
    // 写方向 Credit 编码（R/B Credit）
    // --------------------------------------------------------------------------
    assign rcrd_full  = {(&rcrd_rp3), (&rcrd_rp2), (&rcrd_rp1), (&rcrd_rp0)};
    assign bcrd_full  = {(&bcrd_rp3), (&bcrd_rp2), (&bcrd_rp1), (&bcrd_rp0)};
    assign rcrd_valid = {(|rcrd_rp3), (|rcrd_rp2), (|rcrd_rp1), (|rcrd_rp0)};
    assign bcrd_valid = {(|bcrd_rp3), (|bcrd_rp2), (|bcrd_rp1), (|bcrd_rp0)};

    // sp_type1：R/B Credit 封装（TYPE=2'd1，写方向返回 credit）
    assign sp_type1 = {2'b0, bcrd_rp3, bcrd_rp2, bcrd_rp1, bcrd_rp0,
                       rcrd_rp3, rcrd_rp2, rcrd_rp1, rcrd_rp0, 2'd1};

    // 统一 b_vc 触发：有任一 B 数据或 credit 待发（b_vc 为唯一非-SP 载体，走 gather 槽）
    assign b_avail = (|b_rpv) | (|rcrd_valid) | (|bcrd_valid) |
                     (|awcrd_valid) | (|arcrd_valid) | (|wcrd_valid);

    // PRF 字段（14bit）：b 槽仅当无 SP、b_vc 实际入队(b_vc_send)时才计数
    assign vc_num    = {2'b0, aw_valid} + {2'b0, ar_rp0_valid} + {2'b0, ar_rp1_valid} +
                       {2'b0, ar_rp2_valid} + {2'b0, ar_rp3_valid} + {2'b0, b_vc_send};
    assign prf_field = {2'b0, vc_num[2:0], b_vc_send,
                        ar_rp3_valid, ar_rp2_valid, ar_rp1_valid, ar_rp0_valid,
                        3'b0, aw_valid};

    // AW/AR VC（128bit = PRF[14] + bus[107] + RP[2] + CN[5]）
    assign aw_vc = {AX_VC_WIDTH{aw_valid}} &
                   {prf_field, aw_bus, W_RP[1:0], `AXI4_AW_CN};
    assign ar_rp0_vc = {AX_VC_WIDTH{ar_rp0_valid}} &
                       {prf_field, ar_rp0_bus, 2'd0, `AXI4_AR_CN};
    assign ar_rp1_vc = {AX_VC_WIDTH{ar_rp1_valid}} &
                       {prf_field, ar_rp1_bus, 2'd1, `AXI4_AR_CN};
    assign ar_rp2_vc = {AX_VC_WIDTH{ar_rp2_valid}} &
                       {prf_field, ar_rp2_bus, 2'd2, `AXI4_AR_CN};
    assign ar_rp3_vc = {AX_VC_WIDTH{ar_rp3_valid}} &
                       {prf_field, ar_rp3_bus, 2'd3, `AXI4_AR_CN};

    // 统一 B_VC（权威格式：Padding[15]|B_Credit|R_Credit|W_Credit|AR_Credit|AW_Credit|B_RP|B_RPV|CN）
    //   b_avail 时同时携带全部 5 条 credit lane 与 B 数据（无 r/w 区分）
    assign b_vc = {AX_VC_WIDTH{b_avail}} &
                  {15'd0, bcrd_rp3, bcrd_rp2, bcrd_rp1, bcrd_rp0,
                   rcrd_rp3, rcrd_rp2, rcrd_rp1, rcrd_rp0,
                   wcrd_rp3, wcrd_rp2, wcrd_rp1, wcrd_rp0,
                   arcrd_rp3, arcrd_rp2, arcrd_rp1, arcrd_rp0,
                   awcrd_rp3, awcrd_rp2, awcrd_rp1, awcrd_rp0,
                   b_rp3_bus, b_rp2_bus, b_rp1_bus, b_rp0_bus, b_rpv,
                   `AXI4_B_CN};

    // W Full/Lite VC（SP 字段统一，可承载任一 TYPE——无方向绑定）
    assign w_full_vc = {sp_field, w_bus, W_RP[1:0], `AXI4_W_FULL_CN};
    assign w_lite_vc = {sp_field,
                        w_bus[W_WIDTH-1 : PAIF_DW+PAIF_DW/8],
                        w_bus[PAIF_DW-1:0], W_RP[1:0], `AXI4_W_LITE_CN};

    // gather_vc 聚合包（紧凑递补，同 wch_pack）
    wire [2:0] c_ar0 = {2'b0, aw_valid};
    wire [2:0] c_ar1 = c_ar0 + {2'b0, ar_rp0_valid};
    wire [2:0] c_ar2 = c_ar1 + {2'b0, ar_rp1_valid};
    wire [2:0] c_ar3 = c_ar2 + {2'b0, ar_rp2_valid};
    wire [2:0] c_b   = c_ar3 + {2'b0, ar_rp3_valid};

    function [767:0] vc_shift;
        input [127:0] vc;
        input [2:0]   n;
        begin
            case (n)
                3'd0: vc_shift = {640'd0, vc};
                3'd1: vc_shift = {512'd0, vc, 128'd0};
                3'd2: vc_shift = {384'd0, vc, 256'd0};
                3'd3: vc_shift = {256'd0, vc, 384'd0};
                3'd4: vc_shift = {128'd0, vc, 512'd0};
                3'd5: vc_shift = {vc, 640'd0};
                default: vc_shift = 768'd0;
            endcase
        end
    endfunction

    assign gather_vc = vc_shift(aw_vc, 3'd0) |
                       vc_shift(ar_rp0_vc, c_ar0) |
                       vc_shift(ar_rp1_vc, c_ar1) |
                       vc_shift(ar_rp2_vc, c_ar2) |
                       vc_shift(ar_rp3_vc, c_ar3) |
                       vc_shift({AX_VC_WIDTH{b_vc_valid}} & b_vc, c_b);

    assign gather_valid_w   = aw_valid | ar_rp0_valid | ar_rp1_valid |
                              ar_rp2_valid | ar_rp3_valid | b_vc_valid;
    assign gather_vc_cnt    = {4'd0, vc_num} * AX_VC_CNT;

    // --------------------------------------------------------------------------
    // 写方向 vc_in（宽度 VC_IN_WIDTH = GATHER + DATA_VC）
    //   gather(AW/AR/b_vc 聚合 6AX) 前置；数据 VC(w_full/w_lite) 紧随 gather 首尾相接，无空隙
    //   补齐零物理置于最高位（gather 之上），不在 gather 与数据之间插零
    //   w_full：gather(6AX) + w_full                       （总=VC_IN，无补齐）
    //   w_lite：补齐(VC_IN-GATHER-WLITE) + gather + w_lite  （总=VC_IN，w_lite 紧贴 gather）
    //   无 w：  数据区零 + gather（仅聚合，gather 落低位紧贴 SHIFT 出队区）
    // --------------------------------------------------------------------------

    assign w_vc_in = w_lite ? {{(VC_IN_WIDTH - WLITE_VC_WIDTH){1'b0}}  , {WLITE_VC_WIDTH{w_enq}} & w_lite_vc} : {{(VC_IN_WIDTH - WFULL_VC_WIDTH){1'b0}}  , {WFULL_VC_WIDTH{w_enq}} & w_full_vc};

    // ==========================================================================
    // 组合逻辑 —— 读方向（继承 p_axi4_rch_pack）
    // ==========================================================================

    // B RP 有效向量与选择
    assign b_rpv = {b_rp3_valid, b_rp2_valid, b_rp1_valid, b_rp0_valid};
    assign b_bus_sel = ({16{b_rp0_valid}} & b_rp0_bus) |
                       ({16{b_rp1_valid}} & b_rp1_bus) |
                       ({16{b_rp2_valid}} & b_rp2_bus) |
                       ({16{b_rp3_valid}} & b_rp3_bus);
    assign b_rp_sel = ({2{b_rp1_valid}} & 2'd1) |
                      ({2{b_rp2_valid}} & 2'd2) |
                      ({2{b_rp3_valid}} & 2'd3);

    // 读方向 Credit 编码（AW/W/AR Credit）
    assign arcrd_full  = {(&arcrd_rp3), (&arcrd_rp2), (&arcrd_rp1), (&arcrd_rp0)};
    assign wcrd_full   = {(&wcrd_rp3),  (&wcrd_rp2),  (&wcrd_rp1),  (&wcrd_rp0)};
    assign arcrd_valid = {(|arcrd_rp3), (|arcrd_rp2), (|arcrd_rp1), (|arcrd_rp0)};
    assign wcrd_valid  = {(|wcrd_rp3),  (|wcrd_rp2),  (|wcrd_rp1),  (|wcrd_rp0)};
    assign awcrd_valid = {(|awcrd_rp3), (|awcrd_rp2), (|awcrd_rp1), (|awcrd_rp0)};

    // SP type0/2/3 封装（type1 见写方向段）
    assign sp_type3 = {b_bus_sel, b_rp_sel, 2'd3};
    assign sp_type0 = {2'b0, wcrd_rp3, wcrd_rp2, wcrd_rp1, wcrd_rp0,
                       awcrd_rp3, awcrd_rp2, awcrd_rp1, awcrd_rp0, 2'd0};
    assign sp_type2 = {10'd0, arcrd_rp3, arcrd_rp2, arcrd_rp1, arcrd_rp0, 2'd2};

    // 统一 SP 优先级（无方向绑定：type1 R/B credit 也可由任一数据 VC 承载）
    //   高优先＝credit 满/B（type0 满 > type1 满 > type2 满 > type3 B）；
    //   低优先＝credit 有效（type0 > type1 > type2）
    //   （SP 与 b_vc 二选一：b_dly_set(B 超时) 时强制 SP 选 type3，保证 B 不饥饿）
    assign sp_hpri_sts = {|b_rpv, |arcrd_full,
                          (|bcrd_full) | (|rcrd_full), |wcrd_full};
    assign sp_hpri_sel = sp_hpri_sts & ((~sp_hpri_sts) + 4'd1);
    assign sp_lpri_sts = {|arcrd_valid, (|bcrd_valid) | (|rcrd_valid), |wcrd_valid};
    assign sp_lpri_sel = sp_lpri_sts & ((~sp_lpri_sts) + 3'd1);
    assign sp_sel = b_dly_set ? 4'b1000 :                  // B 超时强制 type3
                    (|sp_hpri_sel) ? sp_hpri_sel : {1'b0, sp_lpri_sel};
    assign sp_type_hpri = ({20{sp_hpri_sel[3]}} & sp_type3) |
                          ({20{sp_hpri_sel[2]}} & sp_type2) |
                          ({20{sp_hpri_sel[1]}} & sp_type1) |
                          ({20{sp_hpri_sel[0]}} & sp_type0);
    assign sp_type_lpri = ({20{sp_lpri_sel[2]}} & sp_type2) |
                          ({20{sp_lpri_sel[1]}} & sp_type1) |
                          ({20{sp_lpri_sel[0]}} & sp_type0);
    assign sp_field = (|sp_hpri_sel) ? sp_type_hpri : sp_type_lpri;
    assign sp_valid = (|sp_sel);
    // SP 实际随数据 VC 入队：w_vc(w_ready) 或 r_lite_vc(r_ready&r_lite) —— 均带 20bit SP 帧
    //   门控防“数据 VC 未入队却已 ack”（W/R 竞争时另一侧可能未得手）
    assign sp_data_send = sp_valid & (~b_vc_valid) & (w_ready | (r_ready & r_lite));

    // r_lite_bus（去除 rid/rresp）
    assign r_lite_bus = {r_bus[R_WIDTH-1 : R_WIDTH-5], r_bus[PAIF_DW-1:0]};

    // R Full/Lite VC（r_lite 带统一 SP 帧；r_full 不带 SP）
    assign r_full_vc = {8'd0, r_bus, R_RP, `AXI4_R_FULL_CN};
    assign r_lite_vc = {sp_field, r_lite_bus, R_RP, `AXI4_R_LITE_CN};
    assign r_vc = r_lite ? r_lite_vc : r_full_vc;

    // --------------------------------------------------------------------------
    // 读方向 vc_in（零扩至 VC_IN_WIDTH）
    //   统一 b_vc（含 B 数据与全部 credit lane）随 gather 槽拼入（指令1）；数据 VC(r_vc) 紧随 gather 首尾相接
    //   r 数据：补齐(VC_IN-GATHER-R_VC) + gather + r_vc     （总=VC_IN，r_vc 紧贴 gather）
    //   无 r：  数据区零 + gather（仅聚合/读 B，gather 落低位）
    // --------------------------------------------------------------------------

    assign r_vc_in = {{(VC_IN_WIDTH - R_VC_WIDTH){1'b0}}  , {R_VC_WIDTH{r_enq}} & r_vc};

    


    assign gather_vc_in = {{GATHER_VC_WIDTH{(w_enq & (~w_lite) & gather_valid_w)}} & gather_vc , {(VC_IN_WIDTH-GATHER_VC_WIDTH){1'b0}} } | 
                          {{(VC_IN_WIDTH-WLITE_VC_WIDTH-GATHER_VC_WIDTH){1'b0}} , {GATHER_VC_WIDTH{((w_enq & w_lite)|r_enq) & gather_valid_w}} & gather_vc , {WLITE_VC_WIDTH{1'b0}} } |
                          {{(VC_IN_WIDTH-GATHER_VC_WIDTH){1'b0}} , {GATHER_VC_WIDTH{(~(w_enq |r_enq)) & gather_send}} & gather_vc};      // [T2a] gather_valid_w→gather_send（补 buf_ready，消除 zombie gather 幻影写入）
                          

    // ==========================================================================
    // 组合逻辑 —— W/R 轮询仲裁（round-robin）
    // ==========================================================================

    // 请求：W 侧（w/聚合，含 b_vc-only）；R 侧（仅 r 数据），均需 buf_ready
    //   b_vc 走 gather 高位（非-SP 载体），仅无数据 VC(无 SP) 时使用，经由 gather_valid 触发
    assign w_req = w_valid & buf_ready;
    assign r_req = r_valid & buf_ready;

    // round-robin 授权：turn 指向侧优先，另一侧在 turn 侧无请求时让位
    //   固定优先级：turn=W 时 W>R；turn=R 时 R>W。循环轮转保证公平。
    assign w_enq = w_req & ((rr_turn_r == TURN_W) | ~r_req);
    assign r_enq = r_req & ((rr_turn_r == TURN_R) | ~w_req);

    // 聚合包发送：任一侧(w_enq 或 r_enq)得手且 gather 有效时拼入高位（随 w_vc 或 r_vc 一起）
    assign gather_send = gather_valid_w & buf_ready;
    // 统一 b_vc 发送（指令2：SP 与 b_vc 二选一）——仅当无数据 VC 入队携带 SP 时，b_vc 才走 gather 槽
    //   有数据 VC 时 credit 一律经 SP；b_vc 仅在数据 VC 缺席（r_full 或 credit/B-only）时使用

    assign b_not_one = |(b_rpv & (b_rpv - 4'd1)); // not one hot → B≥2 RP
    // b_vc_valid（指令2 互斥 + r_full 补齐）:
    //   - 无数据 VC(w/r 均无): 任一 credit/B 待发 → b_vc 承载（唯一载体）
    //   - r_full: r_full_vc 无 SP 字段(carrier 缺位)，放宽允许 b_vc 随 gather 同发（SP 已关，无双 ack）
    //   - w / r_lite: 有 SP 载体，credit 走 SP（b_vc 压制）; 但 B≥2(b_not_one) 时 SP 单B 放不下 → 仍走 b_vc
    assign b_vc_valid = (b_avail & (~(w_valid | r_valid))) | ((r_valid & ~r_lite)&(|sp_hpri_sts)) | b_not_one;

    assign b_vc_send = buf_ready & b_vc_valid;

    assign w_ready = w_enq;
    assign aw_ready     = gather_send & aw_valid;
    assign ar_rp0_ready = gather_send & ar_rp0_valid;
    assign ar_rp1_ready = gather_send & ar_rp1_valid;
    assign ar_rp2_ready = gather_send & ar_rp2_valid;
    assign ar_rp3_ready = gather_send & ar_rp3_valid;

    assign r_ready = r_enq;

    // 轮转：任一得手（w_enq | r_enq）后 turn 指向另一侧 → 下一拍另一侧优先，循环交替公平
    assign rr_turn_next = w_enq ? 1'b1 : 1'b0;

    // ==========================================================================
    // 组合逻辑 —— 共享 buf_r / buf_cnt / 移位
    // ==========================================================================

    // Credit ready —— 指令2：SP 与 b_vc 二选一，无重复 ack。
    //   有数据 VC：credit 经 SP（sp_data_send & 选中类型）；无数据 VC：统一 b_vc 携带全部类型
    //   写方向 R/B Credit（type1）：SP type1 或 b_vc
    assign rcrd_rp0_ready = b_vc_send | (sp_data_send & sp_sel[1]);
    assign rcrd_rp1_ready = b_vc_send | (sp_data_send & sp_sel[1]);
    assign rcrd_rp2_ready = b_vc_send | (sp_data_send & sp_sel[1]);
    assign rcrd_rp3_ready = b_vc_send | (sp_data_send & sp_sel[1]);
    assign bcrd_rp0_ready = b_vc_send | (sp_data_send & sp_sel[1]);
    assign bcrd_rp1_ready = b_vc_send | (sp_data_send & sp_sel[1]);
    assign bcrd_rp2_ready = b_vc_send | (sp_data_send & sp_sel[1]);
    assign bcrd_rp3_ready = b_vc_send | (sp_data_send & sp_sel[1]);
    // 读方向 AW/W Credit（type0）：SP type0 或 b_vc
    assign wcrd_rp0_ready  = b_vc_send | (sp_data_send & sp_sel[0]);
    assign wcrd_rp1_ready  = b_vc_send | (sp_data_send & sp_sel[0]);
    assign wcrd_rp2_ready  = b_vc_send | (sp_data_send & sp_sel[0]);
    assign wcrd_rp3_ready  = b_vc_send | (sp_data_send & sp_sel[0]);
    assign awcrd_rp0_ready = b_vc_send | (sp_data_send & sp_sel[0]);
    assign awcrd_rp1_ready = b_vc_send | (sp_data_send & sp_sel[0]);
    assign awcrd_rp2_ready = b_vc_send | (sp_data_send & sp_sel[0]);
    assign awcrd_rp3_ready = b_vc_send | (sp_data_send & sp_sel[0]);
    // 读方向 AR Credit（type2）：SP type2 或 b_vc
    assign arcrd_rp0_ready = b_vc_send | (sp_data_send & sp_sel[2]);
    assign arcrd_rp1_ready = b_vc_send | (sp_data_send & sp_sel[2]);
    assign arcrd_rp2_ready = b_vc_send | (sp_data_send & sp_sel[2]);
    assign arcrd_rp3_ready = b_vc_send | (sp_data_send & sp_sel[2]);
    // B（type3）：SP type3（选中 b_rp）或 b_vc
    assign b_rp0_ready = b_vc_send | (sp_data_send & sp_sel[3] & (b_rp_sel == 2'd0));
    assign b_rp1_ready = b_vc_send | (sp_data_send & sp_sel[3] & (b_rp_sel == 2'd1));
    assign b_rp2_ready = b_vc_send | (sp_data_send & sp_sel[3] & (b_rp_sel == 2'd2));
    assign b_rp3_ready = b_vc_send | (sp_data_send & sp_sel[3] & (b_rp_sel == 2'd3));

    // 轮询选中的入队数据
    assign vc_in = r_vc_in | w_vc_in | gather_vc_in;

    // vc_in 零扩展到 BUFFER_WIDTH
    assign vc_in_ext = {{(BUFFER_WIDTH - VC_IN_WIDTH){1'b0}}, vc_in};

    // --------------------------------------------------------------------------
    // [T1] 移位量打拍：桶形 select 锥打平（逐拍位等价）
    //   原: lsft_amt = buf_cnt_after_out[4:0] = tx ? S1_dec(cnt_r) : cnt_r
    //   新: lsft_amt = tx ? lsft_amt_dec_r : lsft_amt_hld_r
    //     两寄存器每拍无条件装载 cnt_nxt / S1_dec(cnt_nxt)（cnt_nxt = buf_cnt_r 的 D）：
    //       不变式（自复位归纳）: lsft_amt_hld_r[t] ≡ cnt_r[t][4:0]
    //                            lsft_amt_dec_r[t] ≡ S1_dec(cnt_r[t])[4:0]
    //   buf_cnt_after_out / buf_ready 原样保留（同拍 vc_in 门控仍需组合值）
    // --------------------------------------------------------------------------
    wire [6:0] cnt_nxt = (tx_paif_ready || buf_ready) ? (buf_cnt_after_out + vc_in_cnt)
                                                      : buf_cnt_r;                // = buf_cnt_r 的 D
    wire [6:0] cnt_nxt_s1 =
                 ({(cnt_nxt[6] & (cnt_nxt[5] | (&cnt_nxt[4:0]))),
                   ((~cnt_nxt[5]) ^ (&cnt_nxt[4:0])),
                   (cnt_nxt[4:0] + 5'd1)} & {7{|cnt_nxt[6:5]}});                  // = S1_dec(cnt_nxt)（掩码=|cnt_nxt[6:5] 单 bit 广播, 与原版 buf_over_paif 同式）
    reg  [4:0] lsft_amt_hld_r;
    reg  [4:0] lsft_amt_dec_r;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            lsft_amt_hld_r <= 5'd0;
            lsft_amt_dec_r <= 5'd0;
        end else begin
            lsft_amt_hld_r <= cnt_nxt[4:0];        // t+1 拍 = buf_cnt_r[t+1][4:0]
            lsft_amt_dec_r <= cnt_nxt_s1[4:0];     // t+1 拍 = S1_dec(buf_cnt_r[t+1])[4:0]
        end
    end

    // 左移 barrel shifter（按 lsft_amt 移位）— [T1] select 为平坦寄存器 + 1 级 5-bit 2:1
    assign lsft_amt = tx_paif_ready ? lsft_amt_dec_r : lsft_amt_hld_r;             // ≡ buf_cnt_after_out[4:0]
    wire [BUFFER_WIDTH-1:0] lsft_s0;
    wire [BUFFER_WIDTH-1:0] lsft_s1;
    wire [BUFFER_WIDTH-1:0] lsft_s2;
    wire [BUFFER_WIDTH-1:0] lsft_s3;
    wire [BUFFER_WIDTH-1:0] lsft_s4;
    assign lsft_s0 = lsft_amt[0] ?
                         {vc_in_ext[BUFFER_WIDTH-1-SFT_GRAN_1:0], {SFT_GRAN_1{1'b0}}} :
                         vc_in_ext;
    assign lsft_s1 = lsft_amt[1] ?
                         {lsft_s0[BUFFER_WIDTH-1-SFT_GRAN_2:0], {SFT_GRAN_2{1'b0}}} :
                         lsft_s0;
    assign lsft_s2 = lsft_amt[2] ?
                         {lsft_s1[BUFFER_WIDTH-1-SFT_GRAN_4:0], {SFT_GRAN_4{1'b0}}} :
                         lsft_s1;
    assign lsft_s3 = lsft_amt[3] ?
                         {lsft_s2[BUFFER_WIDTH-1-SFT_GRAN_8:0], {SFT_GRAN_8{1'b0}}} :
                         lsft_s2;
    assign lsft_s4 = lsft_amt[4] ?
                         {lsft_s3[BUFFER_WIDTH-1-SFT_GRAN_16:0], {SFT_GRAN_16{1'b0}}} :
                         lsft_s3;
    assign vc_in_lsft = lsft_s4;

    // buf_r 右移（PAIF 出队）
    assign buf_rsft = tx_paif_ready ?
                          {{SHIFT_WIDTH{1'b0}}, buf_r[BUFFER_WIDTH-1:SHIFT_WIDTH]} :
                          buf_r;

    // buf_cnt_after_out / buf_ready（S1 经典修正：bit6 进位 + bit5 XOR）
    assign buf_over_paif    = |buf_cnt_r[6:5];
    assign buf_cnt_after_out = tx_paif_ready ?
                                   ({(buf_cnt_r[6] & (buf_cnt_r[5] | (&buf_cnt_r[4:0]))),
                                     ((~buf_cnt_r[5]) ^ (&buf_cnt_r[4:0])),
                                     (buf_cnt_r[4:0]+5'd1)} & {7{buf_over_paif}}) :
                                   buf_cnt_r;
    assign buf_ready = ~(|buf_cnt_after_out[6:5]);

    // vc_in_cnt = gather（任一侧得手时均入队，含 b_vc 槽）+ 该侧数据 VC（w_ready / r_enq）
    //   注意：gather 与数据 VC 可同拍入队（gather 随 w_vc 或 r_vc 拼入高位），
    //   故用加法求和而非 OR（OR 在 ≥32 GRAN 时会位重叠少计）
    //   最大值 31(buf_cnt_after_out)+86(gather48+W38)=117 < 127，7bit 够用
    assign vc_in_cnt = ({7{gather_send}} & gather_vc_cnt) +
                       (({7{w_ready & (~w_lite)}} & (WFULL_VC_CNT - WLITE_VC_CNT)) | ({7{w_ready | r_ready}} & WLITE_VC_CNT));

    // busy（合并两侧；S2：buf 非空=忙）
    assign busy = aw_valid | ar_rp0_valid | ar_rp1_valid | ar_rp2_valid |
                  ar_rp3_valid | w_valid | r_valid |
                  b_rp0_valid | b_rp1_valid | b_rp2_valid | b_rp3_valid |
                  (|rcrd_rp0) | (|rcrd_rp1) | (|rcrd_rp2) | (|rcrd_rp3) |
                  (|bcrd_rp0) | (|bcrd_rp1) | (|bcrd_rp2) | (|bcrd_rp3) |
                  (|awcrd_rp0) | (|awcrd_rp1) | (|awcrd_rp2) | (|awcrd_rp3) |
                  (|arcrd_rp0) | (|arcrd_rp1) | (|arcrd_rp2) | (|arcrd_rp3) |
                  (|wcrd_rp0) | (|wcrd_rp1) | (|wcrd_rp2) | (|wcrd_rp3) |
                  tx_paif_valid | (|buf_cnt_r);

    // tx_paif_valid / tx_paif_data（byp s 侧）—— 由 flit_phase_r / buf_r 组合生成
    assign tx_paif_valid = |flit_phase_r;
    generate
        if (PAIF_DW == 512) begin : g_paif_dw512
            assign tx_paif_data = (flit_phase_r[1] | flit_phase_r[3]) ?
                                     {16'hfb28, buf_r[SHIFT_WIDTH-1:0]} :
                                     {buf_r[SHIFT_WIDTH-1:0], 16'hfb28};
        end
        else begin : g_paif_dw1024
            assign tx_paif_data = {16'hfb28, buf_r[SHIFT_WIDTH-1:0], 16'hfb28};
        end
        endgenerate

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================

    // round-robin turn 指针寄存器（复位默认 W 优先，得手后轮转）
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            rr_turn_r <= TURN_W;
        end
        else if(w_enq || r_enq) begin
            rr_turn_r <= rr_turn_next;
        end
    end

    // buf_r 移位寄存器
    // [T2b] 无条件写：非更新拍 vc_in_lsft ≡ 0（[T2a] 后 vc_in 恒 0）且 buf_rsft ≡ buf_r，
    //      D 自然退化为保持——与原条件写逐拍位等价（zombie gather 拍除外 = 修复幻影写入），
    //      消去 1872-bit "更新 vs 保持" MUX 及其 select 的宽扇出
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            buf_r <= {BUFFER_WIDTH{1'b0}};
        end else begin
            buf_r <= vc_in_lsft | buf_rsft;
        end
    end

    // buf_cnt_r（[T1] D 端统一走 cnt_nxt，与原条件装载逐拍位等价）
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            buf_cnt_r <= 7'd0;
        end else begin
            buf_cnt_r <= cnt_nxt;               // ≡ (txready||buf_ready) ? after_out+vc_in_cnt : 保持
        end
    end

    // b_dly_cnt_r（B 数据等待计时；超时输出 b_dly_set 强制 B 经 SP type3 下发，防饥饿）
    assign b_dly_set = (|b_rpv) & (&b_dly_cnt_r);
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            b_dly_cnt_r <= 2'd0;
        end
        else if (buf_ready) begin
            if (b_vc_send || (sp_data_send & sp_sel[3]) || (~(|b_rpv))) begin
                b_dly_cnt_r <= 2'd0;
            end else if (~(&b_dly_cnt_r)) begin
                b_dly_cnt_r <= b_dly_cnt_r + 2'd1;
            end
        end
    end

    // flit_phase_r（任一方向入队或待输出时启动/推进）
    wire flit_start;
    assign flit_start = (w_enq | r_enq | gather_send | buf_over_paif);
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            flit_phase_r <= {FLT_PHS_WIDTH{1'b0}};
        end
        else if (flit_start &&
                 ((flit_phase_r[FLT_PHS_WIDTH-1] && tx_paif_ready) || (~tx_paif_valid))) begin
            flit_phase_r <= {{(FLT_PHS_WIDTH-1){1'b0}}, 1'b1};
        end
        else if (tx_paif_ready && (|flit_phase_r)) begin
            flit_phase_r <= {flit_phase_r[FLT_PHS_WIDTH-2:0], 1'b0};
        end
    end

    // ==========================================================================
    // 子模块实例化
    // ==========================================================================
    // paif_tx 输出旁路寄存：bypass-on-ready（m 侧未 ready 时打一拍保持）。
    //   注：p_data_byp 复位端口命名为 rst_n（源模块 d2d_data_byp 遗留），此处接内部 rstn。
    p_data_byp #(
        .DATA_W    (PAIF_DW)
    ) u_paif_tx_byp (
        .clk       (clk             ),
        .rst_n     (rstn            ),
        .s_data    (tx_paif_data    ),
        .s_valid   (tx_paif_valid   ),
        .s_ready   (tx_paif_ready   ),
        .m_data    (paif_txdata     ),
        .m_valid   (paif_txvalid    ),
        .m_ready   (paif_txready    )
    );

    // ==========================================================================
    // 断言与检查（可选）
    // ==========================================================================
    // 暂无（待 RTL 验证阶段补充）

endmodule
