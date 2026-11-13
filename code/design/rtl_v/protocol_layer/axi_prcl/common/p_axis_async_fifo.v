// =============================================================================
// module name：p_axis_async_fifo
// author: OpenHappy Team
// MAS：p_axis_async_fifo_mas.md
// 功能概述: AXI4 Asymmetric 协议层共享异步 FIFO 模板，跨时钟域传输并在写时钟域输出 credit
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   2026-08-06  OpenHappy Team  初始版本
//   2026-08-12  OpenHappy Team  BUG-001：读侧改 FWFT（m_data=mem[rd_ptr] 组合直出、m_valid=!rd_empty），
//                     修复原 m_valid_r/m_data_r 打拍错位（首 flit 损坏、整体滞后一拍、尾 flit 丢弃），
//                     仿真验证通过
//   2026-08-14  OpenHappy Team  复位架构：rstn_c 组合复位改经 rstn_sync 异步断言/同步释放，写侧寄存器用
//                     rstn_s（s_clk 域）、读侧寄存器用 rstn_m（m_clk 域），缓解组合复位跨域 fanout
//   2026-08-26  OpenHappy Team  指针 gray 2 级同步改 per-bit sync_cell：读/写指针 gray 每 bit 例化一个
//                     sync_cell（参考 code/common/async_fifo.v），替代原手写双级 FF 总线打拍
//   2026-09-02  OpenHappy Team  读侧 FWFT 输出加 pip 流水级（仿 p_data_pip，内联不例化）：RAM FWFT
//                     （m_data=mem[rd_ptr]、m_valid=!rd_empty）接入中间级 temp_r/vld_r，
//                     m_data/m_valid 改由 pip 级驱动（m_ready 反压清 vld_r），rd_ptr 消耗
//                     改由 rd_pip_shake（pip 采纳 RAM 时）驱动，读侧输出整体延一拍
//   2026-09-02  OpenHappy Team  指针同步去例化 sync_cell 改 always 双级 FF 打拍：读指针 gray 同步到
//                     写域（gray_rptr_sync1/gray_rptr_sync2，s_clk/rstn_s）、写指针 gray 同步
//                     到读域（gray_wptr_sync1/gray_wptr_sync2，m_clk/rstn_m），替代 per-bit
//                     sync_cell 例化（保持 2 级同步语义）
//   2026-09-07  zhangna CDC 修复(Ac_glitch03): gray 指针先于源域打一拍成寄存器源
//                     (rd_ptr_gray_r/wr_ptr_gray_r) 再跨域同步,消除原组合 bin2gray 驱动
//                     同步器首级的 glitch 源;仅使跨域读到的指针保守滞后一拍
// =============================================================================


module p_axis_async_fifo #(
    parameter DATA_W = 512,
    parameter DEPTH   = 32,
    parameter CRD_W   = $clog2(DEPTH + 1)   // credit 位宽，可表达 0~DEPTH
) (
    input  wire                 s_clk,
    input  wire                 m_clk,
    input  wire                 s_rstn,
    input  wire                 m_rstn,
    input  wire  [DATA_W-1:0]   s_data,
    input  wire                 s_valid,
    output wire                 s_ready,
    output wire  [DATA_W-1:0]   m_data,
    output wire                 m_valid,
    input  wire                 m_ready,
    output wire  [CRD_W-1:0]    crd,
    output wire                 crd_valid
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    localparam ASIZE  = (DEPTH <= 1) ? 1 : $clog2(DEPTH);   // 指针地址位宽
    localparam PTR_W  = ASIZE + 1;                          // 指针总位宽（多 1 bit 区分满空）

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    // 双口 RAM
    reg [DATA_W-1:0] mem [0:DEPTH-1];

    // 写侧指针（s_clk 域）
    reg  [PTR_W-1:0] wr_ptr;
    wire [PTR_W-1:0] wr_ptr_gray;
    wire             wr_full;
    wire             wr_hsk;

    // 读侧指针（m_clk 域）
    reg  [PTR_W-1:0] rd_ptr;
    wire [PTR_W-1:0] rd_ptr_gray;
    wire             rd_empty;
    wire             rd_hsk;

    // 读指针 gray 码同步到写时钟域（always 双级 FF 打拍，第 2 级输出 gray_rptr_sync2）
    reg [PTR_W-1:0] gray_rptr_sync2;

    // 写指针 gray 码同步到读时钟域（always 双级 FF 打拍，第 2 级输出 gray_wptr_sync2）
    reg [PTR_W-1:0] gray_wptr_sync2;

    // credit 生成（写时钟域）
    wire [PTR_W-1:0] rptr_sync2_bin;
    reg  [PTR_W-1:0] last_rptr_r;
    wire [PTR_W-1:0] rptr_sub;

    // 读侧 pip（仿 p_data_pip）中间信号：ram_out=mem 直出、rd_pip_s_valid=!rd_empty、
    // rd_pip_s_ready/rd_pip_shake 控制 temp_r 采纳（wire 提前声明以先于 rd_hsk assign 使用）
    wire [DATA_W-1:0]  ram_out;
    wire               rd_pip_s_valid;
    wire               rd_pip_s_ready;
    wire               rd_pip_shake;

    // 任一复位有效即复位所有逻辑（rstn_c = s_rstn & m_rstn，组合复位跨域 fanout）
    // 设计意图：任一域复位有效即复位双域所有逻辑（MAS §5.4）。复位释放时序（两域复位释放
    // 不同步时一域寄存器被另一域复位强制保持）由 SpyGlass 复位域深度 lint 复核；运行期
    // 约束：两域复位释放须在 credit_en=0、FIFO 无数据传输时进行。
    wire rstn_c = s_rstn & m_rstn;

    wire rstn_s;

    wire rstn_m;

    rstn_sync u_rstn_s (
        .test_mode       (1'b0),
        .test_resetn_in  (1'b1),
        .resetn_in       (rstn_c),
        .clk             (s_clk),
        .resetn_out      (rstn_s)
    );

    rstn_sync u_rstn_m (
        .test_mode       (1'b0),
        .test_resetn_in  (1'b1),
        .resetn_in       (rstn_c),
        .clk             (m_clk),
        .resetn_out      (rstn_m)
    );

    // --------------------------------------------------------------------------
    // gray 码转换函数
    // --------------------------------------------------------------------------
    function [PTR_W-1:0] bin2gray;
        input [PTR_W-1:0] b;
        begin
            bin2gray = b ^ (b >> 1);
        end
    endfunction

    function [PTR_W-1:0] gray2bin;
        input [PTR_W-1:0] g;
        reg   [PTR_W-1:0] b;
        integer i;
        begin
            b[PTR_W-1] = g[PTR_W-1];
            for (i = PTR_W-2; i >= 0; i = i - 1) begin
                b[i] = b[i+1] ^ g[i];
            end
            gray2bin = b;
        end
    endfunction

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================
    assign wr_hsk    = s_valid & s_ready;
    assign s_ready   = !wr_full;
    assign wr_ptr_gray = bin2gray(wr_ptr);

    assign rd_hsk    = rd_pip_shake;
    assign rd_ptr_gray = bin2gray(rd_ptr);

    // 满判断：写指针 gray 与同步过来的读指针 gray 比较（高 2bit 取反）
    assign wr_full = (wr_ptr_gray == {~gray_rptr_sync2[PTR_W-1:PTR_W-2],
                                      gray_rptr_sync2[PTR_W-3:0]});

    // 空判断：读指针 gray 与同步过来的写指针 gray 相等
    assign rd_empty = (rd_ptr_gray == gray_wptr_sync2);

    // credit 生成组合逻辑
    assign rptr_sync2_bin = gray2bin(gray_rptr_sync2);
    assign rptr_sub       = rptr_sync2_bin - last_rptr_r;
    assign crd            = rptr_sub[CRD_W-1:0];
    assign crd_valid      = (last_rptr_r != rptr_sync2_bin);

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================
    // 写侧指针与 RAM 写入（s_clk 域，rstn_s 复位）
    always @(posedge s_clk or negedge rstn_s) begin
        if (!rstn_s) begin
            wr_ptr <= {PTR_W{1'b0}};
        end
        else if (wr_hsk) begin
            wr_ptr <= wr_ptr + {{PTR_W-1{1'b0}} , 1'b1};
        end
    end

    always @(posedge s_clk) begin
        if (wr_hsk) begin
            mem[wr_ptr[ASIZE-1:0]] <= s_data;
        end
    end

    // 读指针（m_clk 域，rstn_m 复位）
    always @(posedge m_clk or negedge rstn_m) begin
        if (!rstn_m) begin
            rd_ptr <= {PTR_W{1'b0}};
        end
        else if (rd_hsk) begin
            rd_ptr <= rd_ptr + {{PTR_W-1{1'b0}} , 1'b1};
        end
    end


    // 读侧 FWFT→pip 流水（仿 p_data_pip，内联不例化）：将 RAM FWFT 输出转为带中间
    // 级 temp_r/vld_r 的流水输出（m_data/m_valid 由 pip 级驱动）。
    //  - rd_pip_s_valid = !rd_empty（RAM 有数据）
    //  - rd_pip_s_ready = m_ready | ~vld_r（temp_r 可接受新数据）
    //  - rd_pip_shake  = RAM 被采纳（驱动 rd_hsk 消耗读指针）
    assign ram_out        = mem[rd_ptr[ASIZE-1:0]];
    assign rd_pip_s_valid = !rd_empty;
    reg  [DATA_W-1:0]  temp_r;
    reg                vld_r;

    assign rd_pip_s_ready = m_ready | (~vld_r);
    assign rd_pip_shake   = rd_pip_s_ready & rd_pip_s_valid;

    assign m_data  = temp_r;
    assign m_valid = vld_r;

    always @(posedge m_clk or negedge rstn_m) begin
        if (!rstn_m) begin
            temp_r <= {DATA_W{1'b0}};
        end
        else if (rd_pip_shake) begin
            temp_r <= ram_out;
        end
    end

    always @(posedge m_clk or negedge rstn_m) begin
        if (!rstn_m) begin
            vld_r <= 1'b0;
        end
        else if (rd_pip_shake) begin
            vld_r <= 1'b1;
        end
        else if (m_ready) begin
            vld_r <= 1'b0;
        end
    end

    // 读指针 gray 码同步到写时钟域（always 双级 FF 打拍，不例化 sync_cell）
    //   gray_rptr_sync1（第 1 级）/ gray_rptr_sync2（第 2 级，输出）均 s_clk/rstn_s 域
    //
    // 2026-09-07 zhangna CDC 修复(Ac_glitch03): 同步器源在原为组合 bin2gray(rd_ptr),
    // 属同一时钟域多源组合扇入,SpyGlass 在首级 gray_rptr_sync1 报 can glitch。
    // 现先在 m_clk 域把 gray 码打一拍成单寄存器源 rd_ptr_gray_r 再跨域,消除组合 glitch。
    // 多打 1 拍仅使写域读到的读指针保守滞后,不影响满/空判定的正确性(gray 单调语义保持)。
    reg [PTR_W-1:0] gray_rptr_sync1;
    reg [PTR_W-1:0] rd_ptr_gray_r;
    always @(posedge m_clk or negedge rstn_m) begin
        if (!rstn_m) begin
            rd_ptr_gray_r <= {PTR_W{1'b0}};
        end
        else begin
            rd_ptr_gray_r <= bin2gray(rd_ptr);
        end
    end
    always @(posedge s_clk or negedge rstn_s) begin
        if (!rstn_s) begin
            gray_rptr_sync1 <= {PTR_W{1'b0}};
            gray_rptr_sync2 <= {PTR_W{1'b0}};
        end
        else begin
            gray_rptr_sync1 <= rd_ptr_gray_r;
            gray_rptr_sync2 <= gray_rptr_sync1;
        end
    end

    // 写指针 gray 码同步到读时钟域（always 双级 FF 打拍，不例化 sync_cell）
    //   gray_wptr_sync1（第 1 级）/ gray_wptr_sync2（第 2 级，输出）均 m_clk/rstn_m 域
    //
    // 2026-09-07 zhangna CDC 修复(Ac_glitch03): 同读侧,先于 s_clk 域把 gray 码打一拍成
    // 寄存器源 wr_ptr_gray_r 再跨域,消除组合 bin2gray(wr_ptr) 驱动的 glitch 源。
    reg [PTR_W-1:0] gray_wptr_sync1;
    reg [PTR_W-1:0] wr_ptr_gray_r;
    always @(posedge s_clk or negedge rstn_s) begin
        if (!rstn_s) begin
            wr_ptr_gray_r <= {PTR_W{1'b0}};
        end
        else begin
            wr_ptr_gray_r <= bin2gray(wr_ptr);
        end
    end
    always @(posedge m_clk or negedge rstn_m) begin
        if (!rstn_m) begin
            gray_wptr_sync1 <= {PTR_W{1'b0}};
            gray_wptr_sync2 <= {PTR_W{1'b0}};
        end
        else begin
            gray_wptr_sync1 <= wr_ptr_gray_r;
            gray_wptr_sync2 <= gray_wptr_sync1;
        end
    end

    // credit last_rptr_r 寄存器（s_clk 域）
    always @(posedge s_clk or negedge rstn_s) begin
        if (!rstn_s) begin
            last_rptr_r <= {PTR_W{1'b0}};
        end
        else begin
            last_rptr_r <= rptr_sync2_bin;
        end
    end

endmodule
