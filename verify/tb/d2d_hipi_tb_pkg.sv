// ===================================================================
// d2d_hipi_tb_pkg.sv — D2D HiPi 验证环境公共包
//
// 版本：v1.0 | 日期：2026-10-19 | 作者：OpenHappy Team
//
// 内容：
//   1. 全局错误计数 + 检查宏
//   2. axi_txn —— AXI4 事务（写/读请求 + 数据 beat 列表），供 VIP / scoreboard 共用
//   3. axi_rsp —— B/R 响应描述
//
// 依赖：d2d_hipi_tb_defines.vh 必须先于本文件编译。
// timescale 显式锁定 1ns/1ps，保证 VIP 驱动 skew 可精确表示。
// ===================================================================

`timescale 1ns/1ps

`ifndef D2D_HIPI_TB_PKG_SV
`define D2D_HIPI_TB_PKG_SV

package d2d_hipi_tb_pkg;

  // ---------------------------------------------------------------
  // 0. 全局错误计数（tb_top 据此判 PASS/FAIL）
  // ---------------------------------------------------------------
  int tb_error_cnt = 0;
  int tb_check_cnt = 0;

  // ---------------------------------------------------------------
  // 1. 常用位宽常量（与 RTL / tb_defines 单一事实来源）
  // ---------------------------------------------------------------
  localparam int AXI_ID_W    = `D2D_TB_AXI_ID_WIDTH;
  localparam int AXI_ADDR_W  = `D2D_TB_AXI_ADDR_WIDTH;
  localparam int AXI_DATA_W  = `D2D_TB_AXI_DATA_WIDTH;
  localparam int AXI_STRB_W  = `D2D_TB_AXI_STRB_WIDTH;
  localparam int AXI_LEN_W   = `D2D_TB_AXI_LEN_WIDTH;
  localparam int AXI_SIZE_W  = `D2D_TB_AXI_SIZE_WIDTH;
  localparam int AXI_BURST_W = `D2D_TB_AXI_BURST_WIDTH;
  localparam int AXI_CACHE_W = `D2D_TB_AXI_CACHE_WIDTH;
  localparam int AXI_PROT_W  = `D2D_TB_AXI_PROT_WIDTH;
  localparam int AXI_QOS_W   = `D2D_TB_AXI_QOS_WIDTH;
  localparam int AXI_RGN_W   = `D2D_TB_AXI_REGION_WIDTH;
  localparam int AXI_USER_W  = `D2D_TB_AXI_USER_WIDTH;

  // AXI burst 编码
  localparam logic [1:0] AXI_BURST_FIXED = 2'b00;
  localparam logic [1:0] AXI_BURST_INCR  = 2'b01;
  localparam logic [1:0] AXI_BURST_WRAP  = 2'b10;

  // AXI response 编码
  localparam logic [1:0] AXI_RESP_OKAY   = 2'b00;
  localparam logic [1:0] AXI_RESP_EXOKAY = 2'b01;
  localparam logic [1:0] AXI_RESP_SLVERR = 2'b10;
  localparam logic [1:0] AXI_RESP_DECERR = 2'b11;

  // ---------------------------------------------------------------
  // 2. axi_txn —— 一笔 AXI4 事务（写或读）
  //
  //    写事务：aw_* 描述 AW 通道；w_beats[] 数据 beat；响应看 b_*。
  //    读事务：ar_* 描述 AR 通道；响应数据 r_beats[]。
  //
  //    所有请求字段均可配置（id/addr/burst/len/size），满足用户
  //    “id、地址、burst、len、size 提供可配置接口”的要求。
  // ---------------------------------------------------------------
  class axi_txn;

    // 事务类别
    typedef enum bit { TXN_WRITE = 1'b0, TXN_READ = 1'b1 } kind_e;
    kind_e kind;

    // ---- 请求公共字段（可配置）----
    rand logic [AXI_ID_W-1:0]    id;
    rand logic [AXI_ADDR_W-1:0]  addr;
    rand logic [AXI_LEN_W-1:0]   len;      // burst 长度-1（AXI 语义）
    rand logic [AXI_SIZE_W-1:0]  size;     // 每 beat 字节数 = 1<<size
    rand logic [AXI_BURST_W-1:0] burst;
    rand logic [AXI_CACHE_W-1:0] cache;
    rand logic [AXI_PROT_W-1:0]  prot;
    rand logic [AXI_QOS_W-1:0]   qos;
    rand logic [AXI_RGN_W-1:0]   region;
    rand logic [AXI_USER_W-1:0]  axuser;   // awuser / aruser

    // ---- 写数据 beat（wdata/wstrb/wuser 逐 beat 存）----
    logic [AXI_DATA_W-1:0] wdata_q[$];
    logic [AXI_STRB_W-1:0] wstrb_q[$];
    logic [AXI_USER_W-1:0] wuser_q[$];

    // ---- 响应 ----
    logic [AXI_ID_W-1:0]   b_id;
    logic [1:0]            b_resp;
    logic [AXI_USER_W-1:0] b_user;

    logic [AXI_ID_W-1:0]   r_id;
    logic [1:0]            r_resp;
    logic [AXI_DATA_W-1:0] rdata_q[$];
    logic [AXI_USER_W-1:0] ruser_q[$];

    // ---- 追踪 / 比对 ----
    int unsigned seq_num;         // 全局序号（scoreboard 用）
    bit          done = 1'b0;     // 已完成（收到 B 或末 R）

    // 默认约束：INCR + 4KB 内对齐（AXI 协议要求 burst 不跨 4KB）
    constraint c_default {
      burst == AXI_BURST_INCR;
      size  inside {[0:6]};               // 1..64 字节/beat
      len   inside {[0:7]};               // 1..8 beat
      addr[7:0] == 8'h00;                 // beat 起始地址自然对齐
    }

    function new(kind_e k = TXN_WRITE);
      this.kind = k;
      this.id   = '0;
      this.addr = '0;
      this.len  = '0;
      this.size = 3'd6;      // 512bit / 64B per beat（与 AXI_DATA_W 对齐）
      this.burst= AXI_BURST_INCR;
      this.cache= '0;
      this.prot = '0;
      this.qos  = '0;
      this.region = '0;
      this.axuser = '0;
      this.seq_num = 0;
    endfunction

    // 每 beat 字节数
    function int unsigned bytes_per_beat();
      return (1 << size);
    endfunction

    // beat 数（= len+1）
    function int unsigned beat_num();
      return (len + 1);
    endfunction

    // 按 burst 计算第 i 个 beat 的地址（INCR/FIXED/WRAP）
    function logic [AXI_ADDR_W-1:0] beat_addr(int unsigned i);
      logic [AXI_ADDR_W-1:0] a;
      int unsigned bpb = bytes_per_beat();
      int unsigned wrap_bytes_sel;
      a = addr;
      case (burst)
        AXI_BURST_FIXED: a = addr;
        AXI_BURST_INCR : a = addr + i * bpb;
        AXI_BURST_WRAP : begin
          // WRAP：起始按 (len+1)*bpb 对齐后回绕
          wrap_bytes_sel = (i + ((addr / bpb) % (len + 1))) % (len + 1);
          a = (addr - (addr % ((len + 1) * bpb))) + wrap_bytes_sel * bpb;
        end
        default: a = addr + i * bpb;
      endcase
      return a;
    endfunction

    // 生成写数据（按 beat 数填充，数据可外部改写）
    function void gen_wdata(logic [AXI_DATA_W-1:0] seed_base = '0);
      wdata_q.delete();
      wstrb_q.delete();
      wuser_q.delete();
      for (int unsigned i = 0; i < beat_num(); i++) begin
        wdata_q.push_back(seed_base ^ (AXI_DATA_W'(i) << 32) ^ AXI_DATA_W'(seq_num));
        wstrb_q.push_back({AXI_STRB_W{1'b1}});
        wuser_q.push_back('0);
      end
    endfunction

    // 内容摘要（打印 / 比对日志）
    function string to_string();
      return $sformatf("%s#%0d id=%0h addr=%0h len=%0d size=%0d burst=%0d beats=%0d",
                       (kind == TXN_WRITE) ? "WR" : "RD",
                       seq_num, id, addr, len, size, burst, beat_num());
    endfunction

    // 请求字段比对（scoreboard 用：只比协议关键字段）
    function bit req_equal(axi_txn rhs);
      bit ok = 1'b1;
      if (this.kind  !== rhs.kind ) ok = 1'b0;
      if (this.id    !== rhs.id   ) ok = 1'b0;
      if (this.addr  !== rhs.addr ) ok = 1'b0;
      if (this.len   !== rhs.len  ) ok = 1'b0;
      if (this.size  !== rhs.size ) ok = 1'b0;
      if (this.burst !== rhs.burst) ok = 1'b0;
      return ok;
    endfunction

    // 写数据 beat 比对（可选：DUT 是否逐字节保序搬运）
    function bit wdata_equal(axi_txn rhs);
      bit ok = 1'b1;
      if (this.wdata_q.size() != rhs.wdata_q.size()) return 1'b0;
      for (int unsigned i = 0; i < this.wdata_q.size(); i++) begin
        if (this.wdata_q[i] !== rhs.wdata_q[i]) ok = 1'b0;
        if (this.wstrb_q[i] !== rhs.wstrb_q[i]) ok = 1'b0;
      end
      return ok;
    endfunction

    function axi_txn clone();
      axi_txn cp = new(this.kind);
      cp.id     = this.id;
      cp.addr   = this.addr;
      cp.len    = this.len;
      cp.size   = this.size;
      cp.burst  = this.burst;
      cp.cache  = this.cache;
      cp.prot   = this.prot;
      cp.qos    = this.qos;
      cp.region = this.region;
      cp.axuser = this.axuser;
      foreach (this.wdata_q[i]) cp.wdata_q.push_back(this.wdata_q[i]);
      foreach (this.wstrb_q[i]) cp.wstrb_q.push_back(this.wstrb_q[i]);
      foreach (this.wuser_q[i]) cp.wuser_q.push_back(this.wuser_q[i]);
      cp.seq_num = this.seq_num;
      cp.done    = this.done;
      return cp;
    endfunction

  endclass

endpackage

// ---------------------------------------------------------------
// 3. 公共宏
// ---------------------------------------------------------------
`define D2D_TB_INFO(msg)    $display("%0t [TB_INFO] %s", $time, msg)

`define D2D_TB_ERROR(msg)   $display("%0t [TB_ERROR] %s", $time, msg)

`define D2D_TB_CHECK(cond, msg) \
  begin \
    tb_check_cnt = tb_check_cnt + 1; \
    if (cond) begin \
      $display("%0t [TB_CHECK] PASS: %s", $time, msg); \
    end else begin \
      $display("%0t [TB_CHECK] FAIL: %s", $time, msg); \
      tb_error_cnt = tb_error_cnt + 1; \
    end \
  end

`endif  // D2D_HIPI_TB_PKG_SV
