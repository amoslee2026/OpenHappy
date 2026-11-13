// ===================================================================
// axi_monitor.sv — AXI4 通道监视器（scoreboard 数据来源）
//
// 版本：v1.0 | 日期：2026-10-19 | 作者：OpenHappy Team
//
// 用途：在指定 axi4_if 上只读采样，把完成握手的事务还原成 axi_txn 推给回调。
//   两种角色（role）：
//     ROLE_SLAVE_PORT —— 监视 DUT 作为 Slave 的口（s0_axi）：
//                        采 AW+W（写请求）与 AR（读请求）
//     ROLE_MASTER_PORT—— 监视 DUT 作为 Master 的口（m0_axi）：
//                        采 AW+W 与 AR（DUT 转发出去的请求）
//     ROLE_MST_RESP   —— 监视 DUT 作为 Slave 的口的 B/R（master VIP 收到的回程）
//
// 线程结构：
//   mon_aw  —— 采 AW 请求头入 aw_q
//   mon_w   —— 采 W beats 并按顺序配对 aw_q 队首（AW/W 通道独立，AXI 允许
//              W 先于 AW 到达；此处以 aw_q 存在为前提配对，符合 DUT 顺序转发）
//   mon_ar  —— 采 AR 请求
//   mon_b   —— 采 B 响应（ROLE_MST_RESP）
//   mon_r   —— 采 R 响应 beat（ROLE_MST_RESP）
//
// 回调：用虚方法 push_wr/push_rd/push_b/push_r，子类或 env 直接绑定 lambda
//       不方便，故此处直接持有 scoreboard 句柄（简化，非 UVM 风格）。
// ===================================================================

`ifndef AXI_MONITOR_SV
`define AXI_MONITOR_SV

`timescale 1ns/1ps

import d2d_hipi_tb_pkg::*;

class axi_monitor;

  typedef enum logic [1:0] { ROLE_SLAVE_PORT  = 2'd0,
                             ROLE_MASTER_PORT = 2'd1,
                             ROLE_MST_RESP    = 2'd2 } role_e;

  virtual axi4_if vif;   // plain virtual（只读采样，靠代码约定保证不驱动）
  role_e                   role;
  string                   name;
  d2d_hipi_scoreboard      scb;

  // 内部组装队列
  axi_txn aw_q[$];    // 已收 AW、待配 W
  axi_txn ar_q[$];    // 已收 AR（直接推）
  axi_txn w_acc;      // 当前在收 W 的事务
  int unsigned w_got = 0;

  int unsigned wr_seen = 0;
  int unsigned rd_seen = 0;
  int unsigned b_seen  = 0;
  int unsigned r_seen  = 0;

  function new(virtual axi4_if vif, role_e role,
               d2d_hipi_scoreboard scb, string name = "mon");
    this.vif  = vif;
    this.role = role;
    this.scb  = scb;
    this.name = name;
  endfunction

  // ---------------------------------------------------------------
  // 启动采样线程
  // ---------------------------------------------------------------
  task run();
    if (role == ROLE_MST_RESP) begin
      fork
        mon_b();
        mon_r();
      join_none
    end else begin
      fork
        mon_aw();
        mon_w();
        mon_ar();
      join_none
    end
  endtask

  // ---------------------------------------------------------------
  // 采 AW
  // ---------------------------------------------------------------
  task automatic mon_aw();
    axi_txn t;
    forever begin
      @(posedge vif.aclk);
      if (vif.awvalid === 1'b1 && vif.awready === 1'b1) begin
        t = new(axi_txn::TXN_WRITE);
        t.id     = vif.awid;
        t.addr   = vif.awaddr;
        t.len    = vif.awlen;
        t.size   = vif.awsize;
        t.burst  = vif.awburst;
        t.cache  = vif.awcache;
        t.prot   = vif.awprot;
        t.qos    = vif.awqos;
        t.region = vif.awregion;
        t.axuser = vif.awuser;
        t.seq_num = wr_seen;
        aw_q.push_back(t);
        $display("%0t [%s] AW  id=%0h addr=%0h len=%0d size=%0d burst=%0d",
                 $time, name, t.id, t.addr, t.len, t.size, t.burst);
      end
    end
  endtask

  // ---------------------------------------------------------------
  // 采 W（配对 aw_q 队首）
  // ---------------------------------------------------------------
  task automatic mon_w();
    axi_txn t;
    forever begin
      @(posedge vif.aclk);
      if (vif.wvalid === 1'b1 && vif.wready === 1'b1) begin
        // 若当前无在收事务，从 aw_q 取队首（AW 可能稍晚，等一拍重试）
        if (w_acc == null) begin
          if (aw_q.size() == 0) begin
            $display("%0t [%s] WARNING: W beat without pending AW (ignored)", $time, name);
            continue;
          end
          w_acc = aw_q.pop_front();
          w_got = 0;
        end
        w_acc.wdata_q.push_back(vif.wdata);
        w_acc.wstrb_q.push_back(vif.wstrb);
        w_acc.wuser_q.push_back(vif.wuser);
        w_got++;
        if (vif.wlast === 1'b1 || w_got == w_acc.beat_num()) begin
          // 事务收齐
          w_acc.done = 1'b1;
          push_wr(w_acc);
          w_acc = null;
          w_got = 0;
        end
      end
    end
  endtask

  // ---------------------------------------------------------------
  // 采 AR
  // ---------------------------------------------------------------
  task automatic mon_ar();
    axi_txn t;
    forever begin
      @(posedge vif.aclk);
      if (vif.arvalid === 1'b1 && vif.arready === 1'b1) begin
        t = new(axi_txn::TXN_READ);
        t.id     = vif.arid;
        t.addr   = vif.araddr;
        t.len    = vif.arlen;
        t.size   = vif.arsize;
        t.burst  = vif.arburst;
        t.cache  = vif.arcache;
        t.prot   = vif.arprot;
        t.qos    = vif.arqos;
        t.region = vif.arregion;
        t.axuser = vif.aruser;
        t.seq_num = rd_seen;
        $display("%0t [%s] AR  id=%0h addr=%0h len=%0d size=%0d burst=%0d",
                 $time, name, t.id, t.addr, t.len, t.size, t.burst);
        push_rd(t);
      end
    end
  endtask

  // ---------------------------------------------------------------
  // 采 B
  // ---------------------------------------------------------------
  task automatic mon_b();
    axi_txn t;
    forever begin
      @(posedge vif.aclk);
      if (vif.bvalid === 1'b1 && vif.bready === 1'b1) begin
        t = new(axi_txn::TXN_WRITE);
        t.b_id   = vif.bid;
        t.b_resp = vif.bresp;
        t.b_user = vif.buser;
        t.seq_num = b_seen;
        $display("%0t [%s] B   id=%0h resp=%0d", $time, name, t.b_id, t.b_resp);
        push_b(t);
      end
    end
  endtask

  // ---------------------------------------------------------------
  // 采 R（逐 beat，遇到 rlast 组一笔）
  // ---------------------------------------------------------------
  task automatic mon_r();
    axi_txn t;
    t = null;
    forever begin
      @(posedge vif.aclk);
      if (vif.rvalid === 1'b1 && vif.rready === 1'b1) begin
        if (t == null) begin
          t = new(axi_txn::TXN_READ);
          t.seq_num = r_seen;
        end
        t.r_id   = vif.rid;
        t.r_resp = vif.rresp;
        t.rdata_q.push_back(vif.rdata);
        t.ruser_q.push_back(vif.ruser);
        if (vif.rlast === 1'b1) begin
          $display("%0t [%s] R   id=%0h beats=%0d last rdata=%0h resp=%0d",
                   $time, name, t.r_id, t.rdata_q.size(), vif.rdata, t.r_resp);
          push_r(t);
          t = null;
        end
      end
    end
  endtask

  // ---------------------------------------------------------------
  // 推入 scoreboard（按角色分流）
  // ---------------------------------------------------------------
  task automatic push_wr(axi_txn t);
    if (role == ROLE_SLAVE_PORT)       scb.push_mst_act(t);
    else if (role == ROLE_MASTER_PORT) scb.push_slv_act(t);
    wr_seen++;
  endtask

  task automatic push_rd(axi_txn t);
    if (role == ROLE_SLAVE_PORT)       scb.push_mst_act(t);
    else if (role == ROLE_MASTER_PORT) scb.push_slv_act(t);
    rd_seen++;
  endtask

  task automatic push_b(axi_txn t);
    scb.push_mst_b(t);
    b_seen++;
  endtask

  task automatic push_r(axi_txn t);
    scb.push_mst_r(t);
    r_seen++;
  endtask

  function void report();
    $display("[%s] role=%0d wr=%0d rd=%0d b=%0d r=%0d",
             name, role, wr_seen, rd_seen, b_seen, r_seen);
  endfunction

endclass

`endif  // AXI_MONITOR_SV
