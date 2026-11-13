// =============================================================================
// module name：link_ack_route
// author: OpenHappy Team
// MAS：spec/link_layer/submods/link_layer/mas/link_ack_route_mas.md +
//      mas/link_ack_route_rr_arb_fsm.json
// 功能概述: 反向通道搭载路由模块。解决不对称结构（TX_LINK_NUM / RX_LINK_NUM
//           相互独立配置 1~4）下反向通道 ACK/NACK 的归属问题。按 TX_LINK_NUM
//           每 link_tx 例化一份，输入所有 link_rx 的反向通道（生成 + 解析），
//           输出到对应 link_tx。
//           - 生成通道（dlv_ack/lcl_nack）：由 ACK_ROUTE_CFG_REG.TXi_SRC_EN
//             掩码限定源 RX 集，多源同拍有效时由 4 选 1 round-robin 状态机
//             仲裁选出 1 路（1T 仅搭载 1 路），并输出选中源索引 sel_idx_o
//             （供 link_tx 回填 DLP link_num）。
//           - 解析通道（ack/nack）：CSR 无关、按 DLP link_num 命中路由——每 RX 的
//             ack/nack 与目标链路号 dlp_linknum 一并输入，仅在 (dlp_linknum==link_number_i)
//             命中时参与合并（valid 逐位 OR + 数据按索引优先 mux 成 1 路），不仲裁。
//           全部逻辑位于 mb_clk（2 GHz 数据通路）域，无跨域；配置经顶层
//           sync_cell 从 apb_clk 同步至 mb_clk 后直送。
// 编码约束（本模块强制）:
//   1. 仅时序信号使用 always @(posedge mb_clk)（同步复位，禁用异步复位）；
//   2. 组合逻辑使用 always @(*)+case 或 assign（对齐 link_layer.v 风格）；
//   3. 不使用 SystemVerilog 数组端口，per-RX 输入显式命名 RX0~3 后缀。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   <日期>  <修订人>  <变更概述>
//  2026-09-02  OpenHappy Team  初始版本（对齐 MAS v1.0 / rr_arb_fsm v1.0）
// =============================================================================

`timescale 1ns/1ps

`include "link_layer_defines.vh"

module link_ack_route (
    input  wire        mb_clk,
    input  wire        mb_rst_n,

    // 本实例 TX 链路编号（例化时 2'd0/1/2/3）；解析通道按 DLP link_num 命中此编号
    input  wire [1:0]  link_number_i,

    // 配置：ACK_ROUTE_CFG_REG.TXi_SRC_EN（顶层 sync_cell 同步后直送；仅服务生成通道）
    input  wire [3:0]  csr_ack_route_src_en_i,

    // ---------------------------------------------------------------------
    // 生成通道输入（link_rx → 本 TX 拟搭载发送回对端，per RX 0..3）
    // ---------------------------------------------------------------------
    input  wire [7:0]  dlv_ack_seq0_i,
    input  wire        dlv_ack_valid0_i,
    input  wire [7:0]  lcl_nack_seq0_i,
    input  wire        lcl_nack_valid0_i,
    input  wire [1:0]  lcl_nack_type0_i,

    input  wire [7:0]  dlv_ack_seq1_i,
    input  wire        dlv_ack_valid1_i,
    input  wire [7:0]  lcl_nack_seq1_i,
    input  wire        lcl_nack_valid1_i,
    input  wire [1:0]  lcl_nack_type1_i,

    input  wire [7:0]  dlv_ack_seq2_i,
    input  wire        dlv_ack_valid2_i,
    input  wire [7:0]  lcl_nack_seq2_i,
    input  wire        lcl_nack_valid2_i,
    input  wire [1:0]  lcl_nack_type2_i,

    input  wire [7:0]  dlv_ack_seq3_i,
    input  wire        dlv_ack_valid3_i,
    input  wire [7:0]  lcl_nack_seq3_i,
    input  wire        lcl_nack_valid3_i,
    input  wire [1:0]  lcl_nack_type3_i,

    // ---------------------------------------------------------------------
    // 解析通道输入（对端回传 ACK/NACK，per RX 0..3）
    // ---------------------------------------------------------------------
    input  wire [7:0]  ack_seq0_i,
    input  wire        ack_valid0_i,
    input  wire [7:0]  nack_seq0_i,
    input  wire        nack_valid0_i,
    input  wire [1:0]  dlp_linknum0_i,      // RX0 解析 DLP 目标链路号

    input  wire [7:0]  ack_seq1_i,
    input  wire        ack_valid1_i,
    input  wire [7:0]  nack_seq1_i,
    input  wire        nack_valid1_i,
    input  wire [1:0]  dlp_linknum1_i,      // RX1 解析 DLP 目标链路号

    input  wire [7:0]  ack_seq2_i,
    input  wire        ack_valid2_i,
    input  wire [7:0]  nack_seq2_i,
    input  wire        nack_valid2_i,
    input  wire [1:0]  dlp_linknum2_i,      // RX2 解析 DLP 目标链路号

    input  wire [7:0]  ack_seq3_i,
    input  wire        ack_valid3_i,
    input  wire [7:0]  nack_seq3_i,
    input  wire        nack_valid3_i,
    input  wire [1:0]  dlp_linknum3_i,      // RX3 解析 DLP 目标链路号

    // ---------------------------------------------------------------------
    // 生成通道输出（→ 对应 link_tx，round-robin 仲裁 1 路）
    // ---------------------------------------------------------------------
    output wire [7:0]  dlv_ack_seq_o,
    output wire        dlv_ack_valid_o,
    output wire [7:0]  lcl_nack_seq_o,
    output wire        lcl_nack_valid_o,
    output wire [1:0]  lcl_nack_type_o,
    output wire [1:0]  sel_idx_o,          // 选中源 RX 索引 → link_tx 回填 DLP link_num

    // ---------------------------------------------------------------------
    // 解析通道输出（→ 对应 link_tx，按 DLP link_num 命中合并；NACK 类型由本地
    // NACK_MODE 全局提供，不在本模块路由，见 link_tx.nack_type_i）
    // ---------------------------------------------------------------------
    output wire [7:0]  ack_seq_o,
    output wire        ack_valid_o,
    output wire [7:0]  nack_seq_o,
    output wire        nack_valid_o
);

    // =========================================================================
    // 内部信号
    // =========================================================================
    wire [3:0] gen_req;         // 生成通道每源仲裁请求（enabled && valid）
    wire       grant;           // 本轮有生成通道源被选中
    wire [1:0] granted_idx;     // 本轮选中源索引
    reg  [1:0] rr_ptr;          // round-robin 轮询指针（2 bit，复位 0）
    wire [1:0] rr_ptr_next;     // round-robin 次态（Seg B 组合：命中→选中源+1，否则保持）

    // 仲裁中间组合（assign 实现）：环形选中源索引 / 命中 / 选中源数据路由
    wire [1:0] arb_sel;
    wire       arb_hit;
    wire       lcl_nack_vld_mux;
    wire [7:0] lcl_nack_seq_mux;
    wire [1:0] lcl_nack_type_mux;
    wire [7:0] dlv_ack_seq_mux;

    // 解析通道（命中源内）· valid 逐位 OR 与数据按索引优先选取的中间结果
    wire       parse_ack_valid;
    wire       parse_nack_valid;
    wire [7:0] parse_ack_seq;
    wire [7:0] parse_nack_seq;

    // =========================================================================
    // 生成通道每源仲裁请求
    // =========================================================================
    assign gen_req[0] = csr_ack_route_src_en_i[0] & (dlv_ack_valid0_i | lcl_nack_valid0_i);
    assign gen_req[1] = csr_ack_route_src_en_i[1] & (dlv_ack_valid1_i | lcl_nack_valid1_i);
    assign gen_req[2] = csr_ack_route_src_en_i[2] & (dlv_ack_valid2_i | lcl_nack_valid2_i);
    assign gen_req[3] = csr_ack_route_src_en_i[3] & (dlv_ack_valid3_i | lcl_nack_valid3_i);

    // =========================================================================
    // ★ round-robin 仲裁 FSM（三段式，gdx_review 2026-09-15 Item3 行为等价重写）
    //   Seg A 状态寄存器：rr_ptr = 轮转指针（当前状态）
    //   Seg B 组合次态：   grant（有源选中）→ 指针移到（选中源+1）mod 4，否则保持
    //   Seg C 组合输出：   当前状态下环形选中源与数据路由（assign 风格，延续 Item1）
    //   一拍不多一拍不少：grant 生效拍与指针更新同沿，输出同拍有效（与原实现等价，
    //   未引入仲裁流水拍——用户注：行为等价优先，不引入额外一拍）
    // =========================================================================

    // --- Seg A：状态寄存器段 --------------------------------------------------
    always @(posedge mb_clk) begin
        if (!mb_rst_n)
            rr_ptr <= 2'd0;
        else
            rr_ptr <= rr_ptr_next;
    end

    // --- Seg B：组合次态段 ----------------------------------------------------
    assign grant        = arb_hit;            // 本轮有生成通道源被选中（Seg C decode）
    assign rr_ptr_next  = grant ? (arb_sel + 2'd1) : rr_ptr;   // 2 bit 自然 mod 4

    // --- Seg C：组合输出段（当前状态 rr_ptr 解码 + 数据路由，assign 风格）-------
    // 环形优先选中：从 rr_ptr 起第一个 gen_req=1 源（无命中 fallback index=0，仅 grant=0 时无效值）
    assign arb_sel = (rr_ptr == 2'd0) ? (gen_req[0] ? 2'd0 : gen_req[1] ? 2'd1 : gen_req[2] ? 2'd2 : gen_req[3] ? 2'd3 : 2'd0)
                   : (rr_ptr == 2'd1) ? (gen_req[1] ? 2'd1 : gen_req[2] ? 2'd2 : gen_req[3] ? 2'd3 : gen_req[0] ? 2'd0 : 2'd0)
                   : (rr_ptr == 2'd2) ? (gen_req[2] ? 2'd2 : gen_req[3] ? 2'd3 : gen_req[0] ? 2'd0 : gen_req[1] ? 2'd1 : 2'd0)
                   :                    (gen_req[3] ? 2'd3 : gen_req[0] ? 2'd0 : gen_req[1] ? 2'd1 : gen_req[2] ? 2'd2 : 2'd0);
    assign arb_hit        = |gen_req;
    assign granted_idx    = arb_sel;

    // 选中源数据路由（NACK 优先于 ACK；未命中时被 arb_hit 掩掉，输出为默认）
    assign lcl_nack_vld_mux  = (arb_sel == 2'd0) ? lcl_nack_valid0_i : (arb_sel == 2'd1) ? lcl_nack_valid1_i : (arb_sel == 2'd2) ? lcl_nack_valid2_i : lcl_nack_valid3_i;
    assign lcl_nack_seq_mux  = (arb_sel == 2'd0) ? lcl_nack_seq0_i  : (arb_sel == 2'd1) ? lcl_nack_seq1_i  : (arb_sel == 2'd2) ? lcl_nack_seq2_i  : lcl_nack_seq3_i;
    assign lcl_nack_type_mux = (arb_sel == 2'd0) ? lcl_nack_type0_i : (arb_sel == 2'd1) ? lcl_nack_type1_i : (arb_sel == 2'd2) ? lcl_nack_type2_i : lcl_nack_type3_i;
    assign dlv_ack_seq_mux   = (arb_sel == 2'd0) ? dlv_ack_seq0_i  : (arb_sel == 2'd1) ? dlv_ack_seq1_i  : (arb_sel == 2'd2) ? dlv_ack_seq2_i  : dlv_ack_seq3_i;
    assign lcl_nack_valid_o  = arb_hit & lcl_nack_vld_mux;
    assign dlv_ack_valid_o   = arb_hit & ~lcl_nack_vld_mux;
    assign lcl_nack_seq_o    = (arb_hit & lcl_nack_vld_mux) ? lcl_nack_seq_mux  : 8'd0;
    assign dlv_ack_seq_o     = (arb_hit & ~lcl_nack_vld_mux) ? dlv_ack_seq_mux  : 8'd0;
    assign lcl_nack_type_o   = (arb_hit & lcl_nack_vld_mux) ? lcl_nack_type_mux : 2'b10;

    // 选中源索引落位输出（供 link_tx 回填 DLP link_num）
    assign sel_idx_o = granted_idx;

    // =========================================================================
    // 解析通道合并（组合，CSR 无关、按 DLP link_num 命中）：valid 逐位 OR +
    // 数据按索引优先 mux；ack/nack 独立处理。仅 DLP link_num 与本 TX 编号
    // link_number_i 相等的 RX 源参与（未命中不进入本 TX）。
    // =========================================================================
    assign parse_ack_valid  = ((dlp_linknum0_i == link_number_i) & ack_valid0_i)
                            | ((dlp_linknum1_i == link_number_i) & ack_valid1_i)
                            | ((dlp_linknum2_i == link_number_i) & ack_valid2_i)
                            | ((dlp_linknum3_i == link_number_i) & ack_valid3_i);
    assign parse_nack_valid = ((dlp_linknum0_i == link_number_i) & nack_valid0_i)
                            | ((dlp_linknum1_i == link_number_i) & nack_valid1_i)
                            | ((dlp_linknum2_i == link_number_i) & nack_valid2_i)
                            | ((dlp_linknum3_i == link_number_i) & nack_valid3_i);

    // 数据按索引优先（小索引优先）：取命中源内第一个有效源的 seq
    assign parse_ack_seq  = ((dlp_linknum0_i == link_number_i) & ack_valid0_i)  ? ack_seq0_i  :
                            ((dlp_linknum1_i == link_number_i) & ack_valid1_i)  ? ack_seq1_i  :
                            ((dlp_linknum2_i == link_number_i) & ack_valid2_i)  ? ack_seq2_i  :
                            ((dlp_linknum3_i == link_number_i) & ack_valid3_i)  ? ack_seq3_i  : 8'd0;
    assign parse_nack_seq = ((dlp_linknum0_i == link_number_i) & nack_valid0_i) ? nack_seq0_i :
                            ((dlp_linknum1_i == link_number_i) & nack_valid1_i) ? nack_seq1_i :
                            ((dlp_linknum2_i == link_number_i) & nack_valid2_i) ? nack_seq2_i :
                            ((dlp_linknum3_i == link_number_i) & nack_valid3_i) ? nack_seq3_i : 8'd0;

    // 解析通道输出（valid 即时 OR + 数据经优先 mux）
    assign ack_valid_o  = parse_ack_valid;
    assign nack_valid_o = parse_nack_valid;
    assign ack_seq_o    = parse_ack_seq;
    assign nack_seq_o   = parse_nack_seq;

endmodule
