// =============================================================================
// module name：p_req_timeout
// author: OpenHappy Team
// MAS：p_req_timeout_mas.md
// 功能概述: AXI4 非对称协议层共享的请求超时检测模板（req 完整模式 + resp 简化模式）。
//   维护 Outstanding ID Queue 与每 slot 独立计时器，计时达 timeout_num 触发超时中断；
//   Response ID 无法匹配 Queue 时上报错误中断。
//   req 方向：enable/timeout_num/id/intr/clr 全连 APB CSR（经 p_axi4_req_asymm_csr）。
//   resp 方向：enable tie 0，id/intr/clr 悬空，仅 ots_idle 上报（经 p_axi4_resp_asymm_csr）。
//   enable=0 时 id_used/id_timer 全 0、ots_cnt=0、反压透传（s_axready=m_axready）。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   2026-08-06  OpenHappy Team  从 req 专属超时模块上提为 req/resp 共享模板（方向专属子模块去方向前缀）
//   2026-08-07  OpenHappy Team  timeout_id_r 改为"所有超时 slot 的 id_slot 相或"（OR 掩码形式），
//                     默认同一时刻至多一个 slot 超时；原 for 覆盖改为组合 OR 后寄存
//   2026-08-07  OpenHappy Team  每寄存器独立 always 块：ots_cnt/hcnt/idle、ID Queue 5 数组、
//                     timeout_intr/timeout_id、err_intr/err_id 拆分（非阻塞读旧写新语义不变）
//   2026-08-10  OpenHappy Team  BUG-002 修复：new_id_idx 补 case b（同 cycle 释放同 ID 且分配新同 ID
//                     时，新 slot 复用末位 id_idx，不加 1），对齐 analysis case a/b/c；
//                     RTL 清理：移除死代码 alloc_idx，integer 声明统一到模块顶部
//                     （移除 always 块内 anonymous-block 变量声明，符合 Verilog-2005），
//                     OTS_NUM 比较改 localparam 显式定宽
//   2026-08-12  OpenHappy Team  BUG-005：id_timer 累加改 `<=`（封顶 timeout_num+1）、timeout_alarm
//                     改 `==`（单拍触发），与 analysis 设计点一致；
//                     BUG-006：ots_cnt_r/ots_hcnt_r 减 1 增加下溢守卫（|count）；
//                     BUG-007：ots_cnt_r 减 1 增加 xid_any_match 收窄（虚假响应不减占用计数），
//                     ots_hcnt_r 保持握手平衡语义不加收窄（设计确认）
// =============================================================================


module p_req_timeout #(
    parameter ID_WIDTH = 10,
    parameter OTS_NUM  = 16
) (
    input  wire                    clk,
    input  wire                    rstn,
    output wire                    ots_idle,
    input  wire                    enable,
    input  wire [15:0]             timeout_num,
    // Request 通道（ID 透传 + outstanding 反压）
    input  wire [ID_WIDTH-1:0]     axid,
    input  wire                    s_axvalid,
    output wire                    s_axready,
    output wire                    m_axvalid,
    input  wire                    m_axready,
    // Response 通道（ID 匹配释放 slot）
    input  wire [ID_WIDTH-1:0]     xid,
    input  wire                    xvalid,
    input  wire                    xready,
    // 超时中断 / 错误中断（req 方向接 APB CSR；resp 方向悬空）
    output wire [ID_WIDTH-1:0]     timeout_id,
    output wire                    timeout_intr,
    input  wire                    timeout_clr,
    output wire [ID_WIDTH-1:0]     err_id,
    output wire                    err_intr,
    input  wire                    err_clr
);

    // ==========================================================================
    // 参数定义
    // ==========================================================================
    localparam OTS_CNT_W = $clog2(OTS_NUM + 1);   // outstanding 计数位宽（0..OTS_NUM）
    localparam TM_W      = 16;
    localparam ID_IDX_W = $clog2(OTS_NUM);
    localparam [OTS_CNT_W-1:0] OTS_NUM_W = OTS_NUM[OTS_CNT_W-1:0];  // OTS_NUM 显式定宽，用于满判比较

    // ==========================================================================
    // 内部信号定义
    // ==========================================================================
    // 握手
    wire ax_hsk;
    wire x_hsk;

    // Outstanding 计数
    reg  [OTS_CNT_W-1:0] ots_cnt_r;
    reg  [OTS_CNT_W-1:0] ots_hcnt_r;
    reg                  ots_idle_r;
    wire                 ots_full;

    // ID Queue 寄存器组
    reg                  id_used    [0:OTS_NUM-1];
    reg  [ID_WIDTH-1:0]  id_slot    [0:OTS_NUM-1];
    reg  [TM_W-1:0]      id_timer   [0:OTS_NUM-1];
    reg  [ID_IDX_W-1:0]  id_idx     [0:OTS_NUM-1];  // 同 ID 内序号
    reg                  id_end     [0:OTS_NUM-1];  // 同 ID 末位标志

    // 组合 onehot 比较结果
    wire [OTS_NUM-1:0]   id_free_oh;       // 空闲 slot onehot（最低空闲）
    wire [OTS_NUM-1:0]   axid_last_oh;     // 同 axid 且 id_end 的 slot
    wire [OTS_NUM-1:0]   xid_match_oh;     // xid 匹配的 slot
    wire [OTS_NUM-1:0]   xid_first_oh;     // xid 匹配且 id_idx==0
    wire [OTS_NUM-1:0]   timeout_alarm;   // 计时达门限
    wire                 alloc_en;
    wire                 xid_any_match;
    wire                 xid_any_first;

    // 分配 slot 索引
    reg  [ID_IDX_W-1:0]  last_idx;
    wire                 last_found;

    // 释放 slot 索引
    reg  [ID_IDX_W-1:0]  rel_idx;
    wire                 rel_found;

    // 新分配 slot 的 id_idx 值
    reg  [ID_IDX_W-1:0]  new_id_idx;

    // 超时/错误中断寄存器
    reg                  timeout_intr_r;
    reg  [ID_WIDTH-1:0]  timeout_id_r;
    reg                  err_intr_r;
    reg  [ID_WIDTH-1:0]  err_id_r;
    wire                 err_det;

    // clr 跨时钟域同步（resp 方向 clr 悬空，sync_cell 输出恒为 0）
    wire                 timeout_clr_sync;
    wire                 err_clr_sync;

    // id_used 打包（用于 onehot 运算）
    reg  [OTS_NUM-1:0]   id_used_combined;

    integer i, j, k;

    // ==========================================================================
    // 子模块实例化
    // ==========================================================================
    sync_cell u_timeout_clr_sync (
        .clk_d  (clk),
        .rstn_d (rstn),
        .data_s (timeout_clr),
        .data_d (timeout_clr_sync)
    );

    sync_cell u_err_clr_sync (
        .clk_d  (clk),
        .rstn_d (rstn),
        .data_s (err_clr),
        .data_d (err_clr_sync)
    );

    // ==========================================================================
    // 组合逻辑
    // ==========================================================================
    assign ax_hsk = s_axvalid & s_axready;
    assign x_hsk  = xvalid & xready;
    assign ots_full = (ots_cnt_r == OTS_NUM_W);

    // 反压：Queue 满时反压 s_axready 并屏蔽 m_axvalid；enable=0 时透传
    assign s_axready = enable ? (~ots_full & m_axready) : m_axready;
    assign m_axvalid = enable ? (~ots_full & s_axvalid) : s_axvalid;

    // 最低空闲 slot onehot：(~id_used) & (id_used + 1)
    assign id_free_oh = (~id_used_combined) & (id_used_combined + {{(OTS_NUM-1){1'b0}}, 1'b1});

    // alloc 使能：ax_hsk 且 enable
    assign alloc_en = ax_hsk & enable;

    // 选定分配/释放索引（组合）
    assign  last_found = |axid_last_oh;

    always @(*) begin
        last_idx   = {ID_IDX_W{1'b0}};
        for (i = 0; i < OTS_NUM; i = i + 1) begin
            last_idx   = last_idx | (i & {ID_IDX_W{axid_last_oh[i]}});
        end
    end

    assign  rel_found = |xid_first_oh;

    always @(*) begin
        rel_idx   = {ID_IDX_W{1'b0}};
        for (i = 0; i < OTS_NUM; i = i + 1) begin
            rel_idx   =  rel_idx | (i & {ID_IDX_W{xid_first_oh[i]}});
        end
    end

    // 新 slot 的 id_idx（analysis case a/b/c）：
    //   a) 无同 ID last（last_found=0）→ 0
    //   b) 同 cycle 释放同 ID（x_hsk && xid==axid）且 last_found → 复用末位 id_idx（不加 1）
    //   c) 否则 → 末位 id_idx + 1
    // BUG-002: 原 RTL 缺 case b，单 outstanding 同 ID 释放+分配同 cycle 时新 slot id_idx 误为 1，
    //          后续响应返回时 xid_first 找不到 id_idx==0 的 slot → err_det 误报
    // 注意：case b 按 analysis 原文实现（x_hsk && xid==axid 即命中）；同 ID 多 outstanding 且
    //       释放中间 slot（非末位）+ 分配同 cycle 时，新 slot 会拿到末位 idx，可能与现存 slot idx
    //       冲突——此边界留待 RTL 验证阶段确认是否需收窄为 rel_idx==last_idx
    always @(*) begin
        if (!last_found)
            new_id_idx = {ID_IDX_W{1'b0}};
        else if (x_hsk && (xid == axid))
            new_id_idx = id_idx[last_idx];
        else
            new_id_idx = id_idx[last_idx] + {{(ID_IDX_W-1){1'b0}}, 1'b1};
    end

    // per-slot 组合比较
    genvar gi;
    generate
        for (gi = 0; gi < OTS_NUM; gi = gi + 1) begin : g_cmp
            // 同 axid 且 id_end 且 used
            assign axid_last_oh[gi] = id_used[gi] & id_end[gi] & (axid == id_slot[gi]);
            // xid 匹配
            assign xid_match_oh[gi] = x_hsk & id_used[gi] & (xid == id_slot[gi]);
            // 匹配且 id_idx==0
            assign xid_first_oh[gi] = xid_match_oh[gi] & (id_idx[gi] == {ID_IDX_W{1'b0}});
            // 计时达门限（BUG-005: == 单拍触发，配合 id_timer <= 累加封顶 timeout_num+1）
            assign timeout_alarm[gi] = id_used[gi] & (id_timer[gi] == timeout_num);
        end
    endgenerate

    assign xid_any_match = |xid_match_oh;
    assign xid_any_first = |xid_first_oh;
    // 错误检测：x_hsk 且 (无匹配 | (有匹配但无首位))
    assign err_det = x_hsk & enable & (~xid_any_match | (~xid_any_first));  // BUG-001: 含 & enable（resp enable=0 不误报）

    // id_used 打包（用于 onehot 运算）
    always @(*) begin
        for (i = 0; i < OTS_NUM; i = i + 1)
            id_used_combined[i] = id_used[i];
    end

    // ==========================================================================
    // 时序逻辑
    // ==========================================================================
    // Outstanding 计数寄存器 ots_cnt_r（每寄存器独立 always 块）
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            ots_cnt_r <= {OTS_CNT_W{1'b0}};
        end
        else if (!enable) begin
            // enable=0（含 resp 方向 tie 0）：ots_cnt 恒 0
            ots_cnt_r <= {OTS_CNT_W{1'b0}};
        end
        else begin
            if (ax_hsk && !x_hsk)
                ots_cnt_r <= ots_cnt_r + {{(OTS_CNT_W-1){1'b0}}, 1'b1};
            // BUG-006/007: 减 1 加下溢守卫(|ots_cnt_r) + xid_any_match 收窄（虚假响应不减占用计数）
            else if (!ax_hsk && x_hsk && (|ots_cnt_r) && xid_any_match)
                ots_cnt_r <= ots_cnt_r - {{(OTS_CNT_W-1){1'b0}}, 1'b1};
        end
    end
    // Outstanding 计数寄存器 ots_hcnt_r
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            ots_hcnt_r <= {OTS_CNT_W{1'b0}};
        end
        else if (!enable) begin
            ots_hcnt_r <= {OTS_CNT_W{1'b0}};
        end
        else begin
            if (ax_hsk && !x_hsk)
                ots_hcnt_r <= ots_hcnt_r + {{(OTS_CNT_W-1){1'b0}}, 1'b1};
            // BUG-006: 减 1 加下溢守卫（握手平衡语义，不加 xid 收窄——设计确认）
            else if (!ax_hsk && x_hsk && (|ots_hcnt_r))
                ots_hcnt_r <= ots_hcnt_r - {{(OTS_CNT_W-1){1'b0}}, 1'b1};
        end
    end
    // Outstanding idle 寄存器 ots_idle_r
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            ots_idle_r <= 1'b1;
        end
        else if (!enable) begin
            ots_idle_r <= 1'b1;
        end
        else begin
            ots_idle_r <= (ots_hcnt_r == {OTS_CNT_W{1'b0}});
        end
    end

    assign ots_idle = ots_idle_r;

    // ID Queue 寄存器更新（每数组独立 always 块；非阻塞赋值读旧写新，语义不变）
    //   id_used：分配置 1 / 释放清 0 / enable=0 清 0
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            for (k = 0; k < OTS_NUM; k = k + 1) id_used[k] <= 1'b0;
        end
        else if (!enable) begin
            for (k = 0; k < OTS_NUM; k = k + 1) id_used[k] <= 1'b0;
        end
        else begin
            for (k = 0; k < OTS_NUM; k = k + 1) begin
                if (rel_found && (k == rel_idx))
                    id_used[k] <= 1'b0;
                else if (alloc_en && id_free_oh[k])
                    id_used[k] <= 1'b1;
            end
        end
    end
    //   id_slot：分配时记录 axid（enable=0 保持）
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            for (k = 0; k < OTS_NUM; k = k + 1) id_slot[k] <= {ID_WIDTH{1'b0}};
        end
        else if (enable) begin
            for (k = 0; k < OTS_NUM; k = k + 1) begin
                if (alloc_en && id_free_oh[k])
                    id_slot[k] <= axid;
            end
        end
    end
    //   id_timer：used 时累加（累加条件 id_timer<=timeout_num，封顶 timeout_num+1），
    //             分配时清零 / enable=0 清 0；timeout_alarm 用 == 比较单拍触发（BUG-005）
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            for (k = 0; k < OTS_NUM; k = k + 1) id_timer[k] <= {TM_W{1'b0}};
        end
        else if (!enable) begin
            for (k = 0; k < OTS_NUM; k = k + 1) id_timer[k] <= {TM_W{1'b0}};
        end
        else begin
            for (k = 0; k < OTS_NUM; k = k + 1) begin
                if (alloc_en && id_free_oh[k])
                    id_timer[k] <= {TM_W{1'b0}};
                // BUG-005: <= 累加封顶 timeout_num+1（alarm == 单拍后停止累加）
                else if (id_used[k] && (id_timer[k] <= timeout_num))
                    id_timer[k] <= id_timer[k] + {{(TM_W-1){1'b0}}, 1'b1};
            end
        end
    end
    //   id_idx：同 ID 链内序号，释放后高位递减 / 分配时取 new_id_idx
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            for (k = 0; k < OTS_NUM; k = k + 1) id_idx[k] <= {ID_IDX_W{1'b0}};
        end
        else if (enable) begin
            for (k = 0; k < OTS_NUM; k = k + 1) begin
                // 释放匹配首位 slot 的 id_idx 为 0，不满足递减条件；其余同 ID 链递减
                if (rel_found && id_used[k] && (id_slot[k] == xid) && (|id_idx[k]))
                    id_idx[k] <= id_idx[k] - {{(ID_IDX_W-1){1'b0}}, 1'b1};
                else if (alloc_en && id_free_oh[k])
                    id_idx[k] <= new_id_idx;
            end
        end
    end
    //   id_end：同 ID 链末位标志，分配置位 / 释放或链式清除
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            for (k = 0; k < OTS_NUM; k = k + 1) id_end[k] <= 1'b0;
        end
        else if (enable) begin
            for (k = 0; k < OTS_NUM; k = k + 1) begin
                if ((rel_found && (k == rel_idx)) || (alloc_en && last_found && (k == last_idx)))
                    id_end[k] <= 1'b0;
                else if (alloc_en && id_free_oh[k])
                    id_end[k] <= 1'b1;
            end
        end
    end

    // 超时 ID 组合选择：所有超时 slot 的 id_slot 相或
    //   （默认同一时刻至多一个 slot 超时，OR 结果即唯一超时 slot 的 id；
    //    OR 掩码形式与"锁存超时 slot 的 id"语义一致，表达更直观）
    reg  [ID_WIDTH-1:0] timeout_id_w;
    always @(*) begin
        timeout_id_w = {ID_WIDTH{1'b0}};
        for (j = 0; j < OTS_NUM; j = j + 1)
            timeout_id_w = timeout_id_w | ({ID_WIDTH{timeout_alarm[j]}} & id_slot[j]);
    end

    // 超时中断标志寄存器 timeout_intr_r
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            timeout_intr_r <= 1'b0;
        end
        else if (|timeout_alarm) begin
            timeout_intr_r <= 1'b1;
        end
        else if (timeout_clr_sync) begin
            timeout_intr_r <= 1'b0;
        end
    end
    // 超时 ID 寄存器 timeout_id_r
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            timeout_id_r <= {ID_WIDTH{1'b0}};
        end
        else if (|timeout_alarm) begin
            timeout_id_r <= timeout_id_w;
        end
    end

    assign timeout_intr = timeout_intr_r;
    assign timeout_id   = timeout_id_r;

    // 错误中断标志寄存器 err_intr_r
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            err_intr_r <= 1'b0;
        end
        else if (err_det) begin
            err_intr_r <= 1'b1;
        end
        else if (err_clr_sync) begin
            err_intr_r <= 1'b0;
        end
    end
    // 错误 ID 寄存器 err_id_r
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            err_id_r <= {ID_WIDTH{1'b0}};
        end
        else if (err_det) begin
            err_id_r <= xid;
        end
    end

    assign err_intr = err_intr_r;
    assign err_id   = err_id_r;

endmodule
