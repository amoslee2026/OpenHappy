// ===================================================================
// axi_slave_vip.sv — AXI4 Slave VIP（接 DUT 的 m0_axi master 口）
//
// 版本：v1.1 | 日期：2026-10-19 | 作者：OpenHappy Team
// 修订：v1.1 握手时序重写（posedge 采样 / 沿后 skew 驱动，握手拍后立即撤
//       ready/valid），修正 v1.0 每通道会多等一拍的问题；读数据生成去除
//       非标准位宽 cast 写法。
//
// 功能：
//   1. 自动应答：收 AW+W → 回 BRESP；收 AR → 回 RDATA beats。
//   2. 收到请求记录到 act_q（scoreboard 实际队列来源）。
//   3. 可配：bresp / rresp / 响应延迟 resp_delay / 读数据策略 echo_data。
//   4. 四通道独立线程（aw_w_collect / b_resp_driver / ar_collect / r_resp_driver）。
//
// 时序约定同 axi_master_vip：posedge 采样、沿后 skew 驱动、握手拍后立即撤退。
// ===================================================================

`ifndef AXI_SLAVE_VIP_SV
`define AXI_SLAVE_VIP_SV

`timescale 1ns/1ps

import d2d_hipi_tb_pkg::*;

class axi_slave_vip;

  virtual axi4_if vif;   // plain virtual

  string       name;
  int unsigned err_cnt = 0;
  int unsigned wr_cnt  = 0;
  int unsigned rd_cnt  = 0;

  // 响应配置
  logic [1:0]  cfg_bresp    = AXI_RESP_OKAY;
  logic [1:0]  cfg_rresp    = AXI_RESP_OKAY;
  int unsigned resp_delay   = 2;
  int unsigned hs_timeout   = `D2D_TB_AXI_HANDSHAKE_TIMEOUT;
  real         drive_skew   = 0.05ns;    // 50ps

  // 读数据策略
  bit                    echo_data  = 1'b0;
  logic [AXI_DATA_W-1:0] rdata_base = {AXI_DATA_W/4{4'hA}};   // 0xAAAA...

  // 写缓冲（echo_data 用）
  logic [AXI_ADDR_W-1:0] wbuf_addr[$];
  logic [AXI_DATA_W-1:0] wbuf_data[$];
  logic [AXI_STRB_W-1:0] wbuf_strb[$];

  // 待响应队列
  axi_txn wr_pend_q[$];
  axi_txn rd_pend_q[$];

  // 实际收到的事务
  axi_txn act_q[$];

  function new(virtual axi4_if vif, string name = "axi_slv");
    this.vif  = vif;
    this.name = name;
  endfunction

  task init_idle();
    vif.awready = 1'b0;
    vif.wready  = 1'b0;
    vif.bid     = '0;
    vif.bresp   = '0;
    vif.buser   = '0;
    vif.bvalid  = 1'b0;
    vif.arready = 1'b0;
    vif.rid     = '0;
    vif.rdata   = '0;
    vif.rresp   = '0;
    vif.ruser   = '0;
    vif.rlast   = 1'b0;
    vif.rvalid  = 1'b0;
  endtask

  // ---------------------------------------------------------------
  // 启动后台四线程
  // ---------------------------------------------------------------
  task run();
    fork
      aw_w_collect();
      b_resp_driver();
      ar_collect();
      r_resp_driver();
    join_none
  endtask

  // ---------------------------------------------------------------
  // T1：采 AW + W
  // ---------------------------------------------------------------
  task automatic aw_w_collect();
    axi_txn      t;
    int unsigned c;
    bit          got;                    // 本拍是否真的收到 AW 握手
    forever begin
      // ---- 等 AW 握手 ----
      @(posedge vif.aclk);
      #(drive_skew);
      vif.awready = 1'b1;                 // 先举 ready
      c   = 0;
      got = 1'b0;
      forever begin
        @(posedge vif.aclk);
        if (vif.awvalid === 1'b1) begin got = 1'b1; break; end
        if (++c > hs_timeout) break;
      end
      // 空闲等待超时：slave 被动等 master 发起 AW，无请求属正常状态，
      //   静默清除 awready 后回外层重新举（不得建事务，否则产生伪事务、
      //   且用 X/陈旧字段污染 wr_cnt / act_q / 后续 R-B 响应）。
      //   真错误由后续 W/B 握手超时点捕获（那时 AW 已收到，属实质失败）。
      if (!got) begin
        #(drive_skew);
        vif.awready = 1'b0;
        continue;
      end
      // 采样握手拍上的请求字段
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
      #(drive_skew);
      vif.awready = 1'b0;

      // ---- 采 W beats ----
      #(drive_skew);
      vif.wready = 1'b1;
      for (int unsigned i = 0; i < t.beat_num(); i++) begin
        c = 0;
        @(posedge vif.aclk);
        while (vif.wvalid !== 1'b1) begin
          @(posedge vif.aclk);
          if (++c > hs_timeout) begin
            $display("%0t [%s] ERROR: W never asserted beat %0d/%0d (%0d clk)",
                     $time, name, i, t.beat_num(), c);
            err_cnt++;
            break;
          end
        end
        t.wdata_q.push_back(vif.wdata);
        t.wstrb_q.push_back(vif.wstrb);
        t.wuser_q.push_back(vif.wuser);
        wbuf_addr.push_back(t.beat_addr(i));
        wbuf_data.push_back(vif.wdata);
        wbuf_strb.push_back(vif.wstrb);
      end
      #(drive_skew);
      vif.wready = 1'b0;

      t.seq_num = wr_cnt;
      wr_cnt++;
      wr_pend_q.push_back(t);
      act_q.push_back(t.clone());
      $display("%0t [%s] got WRITE %s", $time, name, t.to_string());
    end
  endtask

  // ---------------------------------------------------------------
  // T2：回 B
  // ---------------------------------------------------------------
  task automatic b_resp_driver();
    axi_txn      t;
    int unsigned c;
    forever begin
      wait (wr_pend_q.size() > 0);
      t = wr_pend_q.pop_front();
      repeat (resp_delay) @(posedge vif.aclk);
      #(drive_skew);
      vif.bid    = t.id;
      vif.bresp  = cfg_bresp;
      vif.buser  = '0;
      vif.bvalid = 1'b1;
      // 等 BREADY 握手
      c = 0;
      forever begin
        @(posedge vif.aclk);
        if (vif.bready === 1'b1) break;
        if (++c > hs_timeout) begin
          $display("%0t [%s] ERROR: BREADY never asserted (%0d clk) %s",
                   $time, name, c, t.to_string());
          err_cnt++;
          break;
        end
      end
      #(drive_skew);
      vif.bvalid = 1'b0;
    end
  endtask

  // ---------------------------------------------------------------
  // T3：采 AR
  // ---------------------------------------------------------------
  task automatic ar_collect();
    axi_txn      t;
    int unsigned c;
    bit          got;
    forever begin
      @(posedge vif.aclk);
      #(drive_skew);
      vif.arready = 1'b1;
      c   = 0;
      got = 1'b0;
      forever begin
        @(posedge vif.aclk);
        if (vif.arvalid === 1'b1) begin got = 1'b1; break; end
        if (++c > hs_timeout) break;
      end
      // 空闲等待超时：同 aw_collect，静默清 arready 回外层重新举，不建伪事务。
      if (!got) begin
        #(drive_skew);
        vif.arready = 1'b0;
        continue;
      end
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
      #(drive_skew);
      vif.arready = 1'b0;

      t.seq_num = rd_cnt;
      rd_cnt++;
      rd_pend_q.push_back(t);
      act_q.push_back(t.clone());
      $display("%0t [%s] got READ  %s", $time, name, t.to_string());
    end
  endtask

  // ---------------------------------------------------------------
  // T4：回 R beats
  // ---------------------------------------------------------------
  task automatic r_resp_driver();
    axi_txn                t;
    int unsigned           c;
    logic [AXI_DATA_W-1:0] d;
    forever begin
      wait (rd_pend_q.size() > 0);
      t = rd_pend_q.pop_front();
      repeat (resp_delay) @(posedge vif.aclk);
      for (int unsigned i = 0; i < t.beat_num(); i++) begin
        // 数据：echo（按 beat 地址查写缓冲）或固定 pattern
        d = rdata_base;
        d[31:0] = rdata_base[31:0] ^ ((i + 32'd1) << 16) ^ (t.seq_num + 32'd1);
        if (echo_data) begin
          foreach (wbuf_addr[k])
            if (wbuf_addr[k] == t.beat_addr(i)) d = wbuf_data[k];
        end
        #(drive_skew);
        vif.rid    = t.id;
        vif.rdata  = d;
        vif.rresp  = cfg_rresp;
        vif.ruser  = '0;
        vif.rlast  = (i == t.beat_num()-1);
        vif.rvalid = 1'b1;
        c = 0;
        forever begin
          @(posedge vif.aclk);
          if (vif.rready === 1'b1) break;
          if (++c > hs_timeout) begin
            $display("%0t [%s] ERROR: RREADY never asserted beat %0d/%0d (%0d clk) %s",
                     $time, name, i, t.beat_num(), c, t.to_string());
            err_cnt++;
            break;
          end
        end
      end
      #(drive_skew);
      vif.rvalid = 1'b0;
      vif.rlast  = 1'b0;
    end
  endtask

  function void report();
    $display("[%s] summary: wr=%0d rd=%0d err=%0d act_q=%0d pend(wr=%0d rd=%0d)",
             name, wr_cnt, rd_cnt, err_cnt, act_q.size(),
             wr_pend_q.size(), rd_pend_q.size());
  endfunction

endclass

`endif  // AXI_SLAVE_VIP_SV
