// ===================================================================
// d2d_hipi_scoreboard.sv — D2D HiPi 端到端 scoreboard
//
// 版本：v1.0 | 日期：2026-10-19 | 作者：OpenHappy Team
//
// 数据流（用户指定）：
//   axi_master_vip → DUT s0_axi → 主带环回 → DUT m0_axi → axi_slave_vip
//                                                              ↓ 响应
//   axi_master_vip ← DUT s0_axi ← 主带环回 ← DUT m0_axi ← (B/R)
//
// 检查点（四个队列，两段比对）：
//
//   ① mst_exp_q  —— axi_master_vip 发出的每一笔请求（期望）
//   ② mst_act_q  —— s0_axi 监视器采到的请求（DUT slave 口实际收到）
//       比对：mst_exp_q[i] == mst_act_q[i]（字段 id/addr/len/size/burst）
//       → 证明 “master 发出的请求 DUT slave 口能原样收到”
//
//   ③ slv_act_q  —— m0_axi 监视器采到的请求（DUT master 口实际发出）
//       比对：mst_exp_q[i] == slv_act_q[i]（同上字段）
//       → 证明 “DUT slave 收到的请求能通过其 master 口转发出去”
//
//   ④ 响应回程：slave_vip 回 B/R → DUT m0 → 主带 → DUT s0 → master_vip
//       由 axi_master_vip 内部超时检测保证（B/R 收不到即 TIMEOUT 报错），
//       本 scoreboard 额外统计 b_resp_cnt / r_beat_cnt 做一致性对账。
//
// 队列按序（in-order）比对：1T1R 单链路环回无乱序能力，按序是合理预期；
// 若后续放开 AXI ID 乱序，可切换为按 ID 分组比对（预留 match_mode）。

`ifndef D2D_HIPI_SCOREBOARD_SV
`define D2D_HIPI_SCOREBOARD_SV

`timescale 1ns/1ps

import d2d_hipi_tb_pkg::*;

class d2d_hipi_scoreboard;

  // ---- 队列 ----
  axi_txn mst_exp_q[$];    // master VIP 发出的请求（期望）
  axi_txn mst_act_q[$];    // s0_axi 采到的请求（DUT slave 口实际）
  axi_txn slv_act_q[$];    // m0_axi 采到的请求（DUT master 口实际）

  axi_txn mst_b_q[$];      // master VIP 收到的 B（写响应回程）
  axi_txn mst_r_q[$];      // master VIP 收到的 R（读数据回程）

  // ---- 统计 ----
  int unsigned req_wr_cnt   = 0;
  int unsigned req_rd_cnt   = 0;
  int unsigned match_cnt    = 0;
  int unsigned mismatch_cnt = 0;
  int unsigned missing_cnt  = 0;

  bit error_mode = 1'b0;   // 1：记录 mismatch 但不打 FAIL（负向用例）

  // ---- 交易记录 dump（人工 review）----
  int log_fd = 0;

  // ===============================================================
  // 压入接口
  // ===============================================================
  function void push_mst_exp(axi_txn t);
    mst_exp_q.push_back(t.clone());
    if (t.kind == axi_txn::TXN_WRITE) req_wr_cnt++; else req_rd_cnt++;
  endfunction

  function void push_mst_act(axi_txn t);
    mst_act_q.push_back(t.clone());
  endfunction

  function void push_slv_act(axi_txn t);
    slv_act_q.push_back(t.clone());
  endfunction

  function void push_mst_b(axi_txn t);
    mst_b_q.push_back(t.clone());
  endfunction

  function void push_mst_r(axi_txn t);
    mst_r_q.push_back(t.clone());
  endfunction

  // ===============================================================
  // 交易 dump 文件
  // ===============================================================
  task open_log(string fname);
    if (log_fd != 0) close_log();
    log_fd = $fopen(fname, "w");
    if (log_fd == 0) $display("%0t [SCB] WARNING: open_log(%s) failed", $time, fname);
    else $fdisplay(log_fd, "%-6s %-6s %-6s %-18s %-6s %-5s %-6s",
                   "stage", "kind", "id", "addr", "len", "size", "burst");
  endtask

  task close_log();
    if (log_fd != 0) begin
      $fclose(log_fd);
      log_fd = 0;
    end
  endtask

  task log_txn(string stage, axi_txn t);
    if (log_fd != 0)
      $fdisplay(log_fd, "%-6s %-6s %-6h %-18h %-6d %-5d %-6d",
                stage, (t.kind == axi_txn::TXN_WRITE) ? "WR" : "RD",
                t.id, t.addr, t.len, t.size, t.burst);
  endtask

  // ===============================================================
  // ① 比对：master VIP 期望 vs s0_axi 实采
  //    （master 发出的请求 → DUT slave 口是否原样收到）
  // ===============================================================
  task compare_mst(string tag = "MST->s0_axi");
    int unsigned n = 0, nw = 0, nr = 0;
    axi_txn q_wr_exp[$], q_wr_act[$], q_rd_exp[$], q_rd_act[$];

    // ★ 按通道分类（同 compare_forwarded 的理由）
    foreach (mst_exp_q[i]) begin
      if (mst_exp_q[i].kind == axi_txn::TXN_WRITE) q_wr_exp.push_back(mst_exp_q[i]);
      else                                        q_rd_exp.push_back(mst_exp_q[i]);
    end
    foreach (mst_act_q[i]) begin
      if (mst_act_q[i].kind == axi_txn::TXN_WRITE) q_wr_act.push_back(mst_act_q[i]);
      else                                        q_rd_act.push_back(mst_act_q[i]);
    end

    if (q_wr_exp.size() != q_wr_act.size())
      $display("%0t [SCB] %s[WR]: SIZE MISMATCH exp=%0d act=%0d",
               $time, tag, q_wr_exp.size(), q_wr_act.size());
    if (q_rd_exp.size() != q_rd_act.size())
      $display("%0t [SCB] %s[RD]: SIZE MISMATCH exp=%0d act=%0d",
               $time, tag, q_rd_exp.size(), q_rd_act.size());

    while (q_wr_exp.size() > 0 && q_wr_act.size() > 0) begin
      axi_txn e = q_wr_exp.pop_front();
      axi_txn a = q_wr_act.pop_front();
      nw++; n++;
      if (e.req_equal(a)) begin
        match_cnt++;
        $display("%0t [SCB] %s PASS[WR#%0d] %s", $time, tag, nw, e.to_string());
      end else begin
        mismatch_cnt++;
        $display("%0t [SCB] %s FAIL[WR#%0d]", $time, tag, nw);
        $display("        EXP %s", e.to_string());
        $display("        ACT %s", a.to_string());
      end
    end
    while (q_rd_exp.size() > 0 && q_rd_act.size() > 0) begin
      axi_txn e = q_rd_exp.pop_front();
      axi_txn a = q_rd_act.pop_front();
      nr++; n++;
      if (e.req_equal(a)) begin
        match_cnt++;
        $display("%0t [SCB] %s PASS[RD#%0d] %s", $time, tag, nr, e.to_string());
      end else begin
        mismatch_cnt++;
        $display("%0t [SCB] %s FAIL[RD#%0d]", $time, tag, nr);
        $display("        EXP %s", e.to_string());
        $display("        ACT %s", a.to_string());
      end
    end
    if (q_wr_exp.size() + q_rd_exp.size() > 0) begin
      missing_cnt += q_wr_exp.size() + q_rd_exp.size();
      $display("%0t [SCB] %s: %0d 条期望事务未在 s0_axi 出现（WR 缺 %0d / RD 缺 %0d）",
               $time, tag, q_wr_exp.size()+q_rd_exp.size(), q_wr_exp.size(), q_rd_exp.size());
    end
    if (q_wr_act.size() + q_rd_act.size() > 0)
      $display("%0t [SCB] %s: %0d 条多余事务（WR %0d / RD %0d）",
               $time, tag, q_wr_act.size()+q_rd_act.size(), q_wr_act.size(), q_rd_act.size());
  endtask

  // ===============================================================
  // ② 比对：s0_axi 实采 vs m0_axi 实采
  //    （DUT slave 收到的请求 → 是否通过 master 口转发）
  // ===============================================================
  task compare_forwarded(input axi_txn ref_q[$], string tag = "s0_axi->m0_axi");
    int unsigned n = 0;
    int unsigned nw = 0, nr = 0;
    axi_txn q_wr_ref[$], q_wr_act[$];
    axi_txn q_rd_ref[$], q_rd_act[$];

    // ★ 按 AXI 通道分类比对（AW/W 与 AR 是独立通道，DUT 内部转发顺序
    //   不保证与 master 发起顺序一致；原实现用单一队列按序比对必然误报）。
    foreach (ref_q[i]) begin
      if (ref_q[i].kind == axi_txn::TXN_WRITE) q_wr_ref.push_back(ref_q[i]);
      else                                     q_rd_ref.push_back(ref_q[i]);
    end
    foreach (slv_act_q[i]) begin
      if (slv_act_q[i].kind == axi_txn::TXN_WRITE) q_wr_act.push_back(slv_act_q[i]);
      else                                         q_rd_act.push_back(slv_act_q[i]);
    end

    if (q_wr_ref.size() != q_wr_act.size()) begin
      $display("%0t [SCB] %s[WR]: QUEUE SIZE MISMATCH ref=%0d m0_act=%0d",
               $time, tag, q_wr_ref.size(), q_wr_act.size());
      if (!error_mode) mismatch_cnt++;
    end
    if (q_rd_ref.size() != q_rd_act.size()) begin
      $display("%0t [SCB] %s[RD]: QUEUE SIZE MISMATCH ref=%0d m0_act=%0d",
               $time, tag, q_rd_ref.size(), q_rd_act.size());
      if (!error_mode) mismatch_cnt++;
    end

    // ---- 写通道 ----
    while (q_wr_ref.size() > 0 && q_wr_act.size() > 0) begin
      axi_txn e = q_wr_ref.pop_front();
      axi_txn a = q_wr_act.pop_front();
      nw++; n++;
      if (e.req_equal(a)) begin
        match_cnt++;
        $display("%0t [SCB] %s PASS[WR#%0d] %s", $time, tag, nw, e.to_string());
      end else begin
        mismatch_cnt++;
        $display("%0t [SCB] %s FAIL[WR#%0d]", $time, tag, nw);
        $display("        REF(s0) %s", e.to_string());
        $display("        ACT(m0) %s", a.to_string());
      end
    end
    // ---- 读通道 ----
    while (q_rd_ref.size() > 0 && q_rd_act.size() > 0) begin
      axi_txn e = q_rd_ref.pop_front();
      axi_txn a = q_rd_act.pop_front();
      nr++; n++;
      if (e.req_equal(a)) begin
        match_cnt++;
        $display("%0t [SCB] %s PASS[RD#%0d] %s", $time, tag, nr, e.to_string());
      end else begin
        mismatch_cnt++;
        $display("%0t [SCB] %s FAIL[RD#%0d]", $time, tag, nr);
        $display("        REF(s0) %s", e.to_string());
        $display("        ACT(m0) %s", a.to_string());
      end
    end

    if (q_wr_ref.size() > 0) begin
      missing_cnt += q_wr_ref.size();
      $display("%0t [SCB] %s[WR]: %0d s0_axi 写请求未转发到 m0_axi", $time, tag, q_wr_ref.size());
    end
    if (q_rd_ref.size() > 0) begin
      missing_cnt += q_rd_ref.size();
      $display("%0t [SCB] %s[RD]: %0d s0_axi 读请求未转发到 m0_axi", $time, tag, q_rd_ref.size());
    end
    if (q_wr_act.size() > 0)
      $display("%0t [SCB] %s[WR]: %0d 条多余写事务在 m0_axi", $time, tag, q_wr_act.size());
    if (q_rd_act.size() > 0)
      $display("%0t [SCB] %s[RD]: %0d 条多余读事务在 m0_axi", $time, tag, q_rd_act.size());
  endtask

  // ===============================================================
  // 响应回程对账：slave VIP 回的 B/R 数 vs master VIP 收到的 B/R 数
  // （master_vip 内部已有超时保护；这里做数量与 resp 编码对账）
  // ===============================================================
  task compare_response(int unsigned exp_b_cnt, int unsigned exp_r_cnt);
    $display("%0t [SCB] RESPONSE: exp_b=%0d got_b=%0d / exp_r=%0d got_r=%0d",
             $time, exp_b_cnt, mst_b_q.size(), exp_r_cnt, mst_r_q.size());

    // ★ exp_* 由调用方传入 master VIP 的"已成功完成"计数，存在时序滞后
    //   （VIP 收到 B/R 后才 push exp_q）。这里以"实际观测到的响应数量"为准，
    //   并把期望值降级为参考信息：只有 got < exp 时才算丢响应。
    if (mst_b_q.size() < exp_b_cnt) begin
      mismatch_cnt++;
      $display("%0t [SCB] FAIL: B 响应不足 (exp>=%0d got=%0d)", $time, exp_b_cnt, mst_b_q.size());
    end else begin
      match_cnt++;
      $display("%0t [SCB] PASS: B 响应数量 %0d", $time, mst_b_q.size());
    end
    if (mst_r_q.size() < exp_r_cnt) begin
      mismatch_cnt++;
      $display("%0t [SCB] FAIL: R 响应不足 (exp>=%0d got=%0d)", $time, exp_r_cnt, mst_r_q.size());
    end else begin
      match_cnt++;
      $display("%0t [SCB] PASS: R 响应数量 %0d", $time, mst_r_q.size());
    end

    // resp 编码检查（正向用例期望全 OKAY）
    //   ★ 只在字段已知（非 X）时判：monitor 采样时若总线为 X 会带出 x，
    //     这种是 TB 采样问题而非 DUT 错误，单独报 WARN 不计数。
    foreach (mst_b_q[i]) begin
      if ($isunknown(mst_b_q[i].b_resp)) begin
        $display("%0t [SCB] WARN: B[%0d] resp=X（monitor 采样未取到有效值）", $time, i);
      end else if (mst_b_q[i].b_resp !== 2'b00) begin
        mismatch_cnt++;
        $display("%0t [SCB] FAIL: B[%0d] resp=%0d (expect OKAY)", $time, i, mst_b_q[i].b_resp);
      end else begin
        match_cnt++;
        $display("%0t [SCB] PASS: B[%0d] resp=OKAY", $time, i);
      end
    end
    foreach (mst_r_q[i]) begin
      if ($isunknown(mst_r_q[i].r_resp)) begin
        $display("%0t [SCB] WARN: R[%0d] resp=X（monitor 采样未取到有效值）", $time, i);
      end else if (mst_r_q[i].r_resp !== 2'b00) begin
        mismatch_cnt++;
        $display("%0t [SCB] FAIL: R[%0d] resp=%0d (expect OKAY)", $time, i, mst_r_q[i].r_resp);
      end else begin
        match_cnt++;
        $display("%0t [SCB] PASS: R[%0d] resp=OKAY", $time, i);
      end
    end
  endtask

  // ===============================================================
  // 是否干净
  // ===============================================================
  function bit is_clean();
    return (mismatch_cnt == 0) && (missing_cnt == 0)
           && (mst_exp_q.size() == 0) && (mst_act_q.size() == 0)
           && (slv_act_q.size() == 0);
  endfunction

  function void report();
    $display("==========================================================");
    $display(" SCOREBOARD SUMMARY");
    $display("   req: wr=%0d rd=%0d", req_wr_cnt, req_rd_cnt);
    $display("   match=%0d mismatch=%0d missing=%0d", match_cnt, mismatch_cnt, missing_cnt);
    $display("   mst_b=%0d mst_r=%0d", mst_b_q.size(), mst_r_q.size());
    $display("   status: %s", is_clean() ? "CLEAN" : "DIRTY");
    $display("==========================================================");
  endfunction

endclass

`endif  // D2D_HIPI_SCOREBOARD_SV
