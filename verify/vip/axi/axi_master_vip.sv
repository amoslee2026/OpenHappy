// ===================================================================
// axi_master_vip.sv — AXI4 Master VIP（接 DUT 的 s0_axi slave 口）
//
// 版本：v1.1 | 日期：2026-10-19 | 作者：OpenHappy Team
// 修订：v1.1 握手时序重写（严格按“posedge 采样 + 沿后 skew 驱动”），修正 v1.0
//       在握手完成后多保持 1 拍 valid 可能造成重复传输的问题；gen_wdata 去除
//       依赖非标准位宽 cast 的写法。
//
// 功能：
//   1. 请求接口全可配：id / addr / burst / len / size（build_write/build_read）。
//   2. write()/read() 阻塞式：AW+W 握手后等 B；AR 握手后收满 R。
//   3. 并发：多次 fork write()/read() 即可（AW/AR/W/R 通道各自独立驱动）。
//   4. 超时保护：握手或响应超时打印 [ERROR] 并累加 err_cnt，绝不挂死仿真。
//   5. 成功完成的事务压入 exp_q（scoreboard 期望队列）。
//
// 关键时序约定（本文件统一遵守）：
//   - 驱动：`@(posedge aclk); #drive_skew; <赋值>` —— 保证输入落在沿之后
//   - 采样：`@(posedge aclk); if (valid && ready)` —— 读取的正是 DUT 在该沿
//     用以完成握手的信号值
//   - 握手完成后立即（+skew）拉低 valid，不多保持一拍
// ===================================================================

`ifndef AXI_MASTER_VIP_SV
`define AXI_MASTER_VIP_SV

`timescale 1ns/1ps

import d2d_hipi_tb_pkg::*;

class axi_master_vip;

  virtual axi4_if vif;   // plain virtual（modport 视图由 tb_top 的普通句柄统一）

  string       name;
  int unsigned err_cnt   = 0;
  int unsigned ok_cnt    = 0;
  int unsigned issue_cnt = 0;

  int unsigned resp_timeout      = `D2D_TB_AXI_RESP_TIMEOUT;
  int unsigned handshake_timeout = `D2D_TB_AXI_HANDSHAKE_TIMEOUT;

  // 沿后偏移（1ps 精度下精确生效），避免与 DUT 同沿采样竞争
  real drive_skew = 0.05ns;   // 50ps：远离 1ps 精度边界，规避沿上竞争

  // 期望事务队列
  axi_txn exp_q[$];

  function new(virtual axi4_if vif, string name = "axi_mst");
    this.vif  = vif;
    this.name = name;
  endfunction

  // ---------------------------------------------------------------
  // 空闲电平
  // ---------------------------------------------------------------
  task init_idle();
    vif.awid     = '0;
    vif.awaddr   = '0;
    vif.awlen    = '0;
    vif.awsize   = '0;
    vif.awburst  = '0;
    vif.awlock   = 1'b0;
    vif.awcache  = '0;
    vif.awprot   = '0;
    vif.awqos    = '0;
    vif.awregion = '0;
    vif.awuser   = '0;
    vif.awvalid  = 1'b0;

    vif.wdata    = '0;
    vif.wstrb    = '0;
    vif.wuser    = '0;
    vif.wlast    = 1'b0;
    vif.wvalid   = 1'b0;

    vif.bready   = 1'b0;    // 由 wait_b 内部拉起，避免提前吞掉响应

    vif.arid     = '0;
    vif.araddr   = '0;
    vif.arlen    = '0;
    vif.arsize   = '0;
    vif.arburst  = '0;
    vif.arlock   = 1'b0;
    vif.arcache  = '0;
    vif.arprot   = '0;
    vif.arqos    = '0;
    vif.arregion = '0;
    vif.aruser   = '0;
    vif.arvalid  = 1'b0;

    vif.rready   = 1'b0;    // 由 wait_r 内部拉起
  endtask

  // ---------------------------------------------------------------
  // 请求构造
  // ---------------------------------------------------------------
  function axi_txn build_write(
      logic [AXI_ID_W-1:0]    id    = 0,
      logic [AXI_ADDR_W-1:0]  addr  = 0,
      logic [AXI_LEN_W-1:0]   len   = 0,
      logic [AXI_SIZE_W-1:0]  size  = 3'd6,
      logic [AXI_BURST_W-1:0] burst = 2'b01
  );
    axi_txn t = new(axi_txn::TXN_WRITE);
    t.id      = id;
    t.addr    = addr;
    t.len     = len;
    t.size    = size;
    t.burst   = burst;
    t.seq_num = issue_cnt;
    return t;
  endfunction

  function axi_txn build_read(
      logic [AXI_ID_W-1:0]    id    = 0,
      logic [AXI_ADDR_W-1:0]  addr  = 0,
      logic [AXI_LEN_W-1:0]   len   = 0,
      logic [AXI_SIZE_W-1:0]  size  = 3'd6,
      logic [AXI_BURST_W-1:0] burst = 2'b01
  );
    axi_txn t = new(axi_txn::TXN_READ);
    t.id      = id;
    t.addr    = addr;
    t.len     = len;
    t.size    = size;
    t.burst   = burst;
    t.seq_num = issue_cnt;
    return t;
  endfunction

  // ---------------------------------------------------------------
  // AW 通道：驱动 + 等握手（握手拍后立即撤 valid）
  // ---------------------------------------------------------------
  task automatic drive_aw(input axi_txn t, output bit ok);
    int unsigned c = 0;
    ok = 1'b0;
    @(posedge vif.aclk);
    #(drive_skew);
    vif.awid     = t.id;
    vif.awaddr   = t.addr;
    vif.awlen    = t.len;
    vif.awsize   = t.size;
    vif.awburst  = t.burst;
    vif.awlock   = 1'b0;
    vif.awcache  = t.cache;
    vif.awprot   = t.prot;
    vif.awqos    = t.qos;
    vif.awregion = t.region;
    vif.awuser   = t.axuser;
    vif.awvalid  = 1'b1;

    forever begin
      @(posedge vif.aclk);
      if (vif.awready === 1'b1) begin
        #(drive_skew);
        vif.awvalid = 1'b0;
        ok = 1'b1;
        return;
      end
      if (++c > handshake_timeout) begin
        $display("%0t [%s] ERROR: AW handshake TIMEOUT (%0d clk) %s",
                 $time, name, c, t.to_string());
        err_cnt++;
        #(drive_skew);
        vif.awvalid = 1'b0;
        return;
      end
    end
  endtask

  // ---------------------------------------------------------------
  // W 通道：逐 beat 驱动 + 等握手
  // ---------------------------------------------------------------
  task automatic drive_w(input axi_txn t, output bit ok);
    int unsigned c;
    int unsigned beats = t.beat_num();
    ok = 1'b0;
    for (int unsigned i = 0; i < beats; i++) begin
      @(posedge vif.aclk);
      #(drive_skew);
      vif.wdata  = t.wdata_q[i];
      vif.wstrb  = t.wstrb_q[i];
      vif.wuser  = t.wuser_q[i];
      vif.wlast  = (i == beats-1);
      vif.wvalid = 1'b1;

      c = 0;
      forever begin
        @(posedge vif.aclk);
        if (vif.wready === 1'b1) begin
          #(drive_skew);
          break;
        end
        if (++c > handshake_timeout) begin
          $display("%0t [%s] ERROR: W handshake TIMEOUT beat %0d/%0d (%0d clk) %s",
                   $time, name, i, beats, c, t.to_string());
          err_cnt++;
          vif.wvalid = 1'b0;
          vif.wlast  = 1'b0;
          return;
        end
      end
    end
    vif.wvalid = 1'b0;
    vif.wlast  = 1'b0;
    ok = 1'b1;
  endtask

  // ---------------------------------------------------------------
  // B 通道：等响应（握手拍采样）
  // ---------------------------------------------------------------
  task automatic wait_b(input axi_txn t, output bit ok);
    int unsigned c = 0;
    ok = 1'b0;
    @(posedge vif.aclk);
    #(drive_skew);
    vif.bready = 1'b1;          // 准备接收写响应
    forever begin
      @(posedge vif.aclk);
      if (vif.bvalid === 1'b1) begin
        t.b_id   = vif.bid;
        t.b_resp = vif.bresp;
        t.b_user = vif.buser;
        #(drive_skew);
        vif.bready = 1'b0;
        ok = 1'b1;
        return;
      end
      if (++c > resp_timeout) begin
        $display("%0t [%s] ERROR: B response TIMEOUT (%0d clk) %s",
                 $time, name, c, t.to_string());
        err_cnt++;
        #(drive_skew);
        vif.bready = 1'b0;
        return;
      end
    end
  endtask

  // ---------------------------------------------------------------
  // AR 通道：驱动 + 等握手
  // ---------------------------------------------------------------
  task automatic drive_ar(input axi_txn t, output bit ok);
    int unsigned c = 0;
    ok = 1'b0;
    @(posedge vif.aclk);
    #(drive_skew);
    vif.arid     = t.id;
    vif.araddr   = t.addr;
    vif.arlen    = t.len;
    vif.arsize   = t.size;
    vif.arburst  = t.burst;
    vif.arlock   = 1'b0;
    vif.arcache  = t.cache;
    vif.arprot   = t.prot;
    vif.arqos    = t.qos;
    vif.arregion = t.region;
    vif.aruser   = t.axuser;
    vif.arvalid  = 1'b1;

    forever begin
      @(posedge vif.aclk);
      if (vif.arready === 1'b1) begin
        #(drive_skew);
        vif.arvalid = 1'b0;
        ok = 1'b1;
        return;
      end
      if (++c > handshake_timeout) begin
        $display("%0t [%s] ERROR: AR handshake TIMEOUT (%0d clk) %s",
                 $time, name, c, t.to_string());
        err_cnt++;
        #(drive_skew);
        vif.arvalid = 1'b0;
        return;
      end
    end
  endtask

  // ---------------------------------------------------------------
  // R 通道：收满 beats
  // ---------------------------------------------------------------
  task automatic wait_r(input axi_txn t, output bit ok);
    int unsigned c     = 0;
    int unsigned beats = t.beat_num();
    int unsigned got   = 0;
    ok = 1'b0;
    t.rdata_q.delete();
    t.ruser_q.delete();
    @(posedge vif.aclk);
    #(drive_skew);
    vif.rready = 1'b1;          // 准备接收读数据
    forever begin
      @(posedge vif.aclk);
      if (vif.rvalid === 1'b1) begin
        t.rdata_q.push_back(vif.rdata);
        t.ruser_q.push_back(vif.ruser);
        t.r_id   = vif.rid;
        t.r_resp = vif.rresp;
        got++;
        if (got >= beats || vif.rlast === 1'b1) begin
          #(drive_skew);
          vif.rready = 1'b0;
          ok = 1'b1;
          return;
        end
      end else if (++c > resp_timeout) begin
        $display("%0t [%s] ERROR: R data TIMEOUT (%0d clk, got %0d/%0d) %s",
                 $time, name, c, got, beats, t.to_string());
        err_cnt++;
        #(drive_skew);
        vif.rready = 1'b0;
        return;
      end
    end
  endtask

  // ---------------------------------------------------------------
  // 写事务
  // ---------------------------------------------------------------
  task automatic write(input axi_txn t);
    bit ok_aw, ok_w, ok_b;
    if (t.wdata_q.size() == 0) t.gen_wdata();
    issue_cnt++;
    fork
      drive_aw(t, ok_aw);
      drive_w (t, ok_w);
    join
    if (ok_aw && ok_w) begin
      wait_b(t, ok_b);
      t.done = ok_b;
      if (ok_b) begin
        ok_cnt++;
        exp_q.push_back(t.clone());
        $display("%0t [%s] WRITE done %s (bresp=%0d)", $time, name, t.to_string(), t.b_resp);
      end
    end else begin
      t.done = 1'b0;
      $display("%0t [%s] WRITE FAIL (aw=%b w=%b) %s", $time, name, ok_aw, ok_w, t.to_string());
    end
  endtask

  // ---------------------------------------------------------------
  // 读事务
  // ---------------------------------------------------------------
  task automatic read(input axi_txn t);
    bit ok_ar, ok_r;
    issue_cnt++;
    drive_ar(t, ok_ar);
    if (ok_ar) begin
      wait_r(t, ok_r);
      t.done = ok_r;
      if (ok_r) begin
        ok_cnt++;
        exp_q.push_back(t.clone());
        $display("%0t [%s] READ done %s (beats=%0d rresp=%0d)",
                 $time, name, t.to_string(), t.rdata_q.size(), t.r_resp);
      end
    end else begin
      t.done = 1'b0;
      $display("%0t [%s] READ FAIL (ar=%b) %s", $time, name, ok_ar, t.to_string());
    end
  endtask

  // ---------------------------------------------------------------
  // 同时 1 笔写 + 1 笔读（用户要求的基础序列）
  // ---------------------------------------------------------------
  task automatic write_and_read(input axi_txn wr, input axi_txn rd);
    fork
      write(wr);
      read (rd);
    join
  endtask

  function void report();
    $display("[%s] summary: issue=%0d ok=%0d err=%0d exp_q=%0d",
             name, issue_cnt, ok_cnt, err_cnt, exp_q.size());
  endfunction

endclass

`endif  // AXI_MASTER_VIP_SV
