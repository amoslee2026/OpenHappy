// ===================================================================
// d2d_hipi_seq_lib.sv — 可复用序列库（静态方法）
//
// 版本：v1.0 | 日期：2026-10-19 | 作者：OpenHappy Team
//
// 以静态任务/函数形式提供各 test 共享的 AXI 请求序列。
// 入参一律显式传递，避免跨实例状态依赖。
//
// 内容：
//   one_write      —— 1 笔写请求 + 写数据（id/addr/len/size/burst 可配）
//   one_read       —— 1 笔读请求（同上可配）
//   write_then_read—— 用户要求的“同时 1 笔写 + 1 笔读”（并发）
//   burst_write    —— 多 beat INCR 写
//   burst_read     —— 多 beat INCR 读
//   apb_write_ctrl —— ctrl 域 APB 写（域选择 bit16=1）
//   apb_read_check —— ctrl 域 APB 读回校验
// ===================================================================

`ifndef D2D_HIPI_SEQ_LIB_SV
`define D2D_HIPI_SEQ_LIB_SV

`timescale 1ns/1ps

import d2d_hipi_tb_pkg::*;

class d2d_hipi_seq_lib;

  // ---------------------------------------------------------------
  // 单笔写（阻塞至收到 B 响应）
  // ---------------------------------------------------------------
  static task one_write(d2d_hipi_test_base base,
                        logic [AXI_ID_W-1:0]    id    = 0,
                        logic [AXI_ADDR_W-1:0]  addr  = 'h1000,
                        logic [AXI_LEN_W-1:0]   len   = 0,
                        logic [AXI_SIZE_W-1:0]  size  = 3'd6,
                        logic [AXI_BURST_W-1:0] burst = 2'b01,
                        logic [AXI_DATA_W-1:0]  data0 = 'h1234_5678);
    axi_txn t = base.mst.build_write(id, addr, len, size, burst);
    t.gen_wdata(data0);
    base.mst.write(t);
    $display("%0t [SEQ] one_write issued: %s", $time, t.to_string());
  endtask

  // ---------------------------------------------------------------
  // 单笔读（阻塞至收满 R beats）
  // ---------------------------------------------------------------
  static task one_read(d2d_hipi_test_base base,
                       logic [AXI_ID_W-1:0]    id    = 0,
                       logic [AXI_ADDR_W-1:0]  addr  = 'h1000,
                       logic [AXI_LEN_W-1:0]   len   = 0,
                       logic [AXI_SIZE_W-1:0]  size  = 3'd6,
                       logic [AXI_BURST_W-1:0] burst = 2'b01);
    axi_txn t = base.mst.build_read(id, addr, len, size, burst);
    base.mst.read(t);
    $display("%0t [SEQ] one_read issued: %s", $time, t.to_string());
  endtask

  // ---------------------------------------------------------------
  // 同时 1 笔写 + 1 笔读（用户要求的基础 seq）
  // ---------------------------------------------------------------
  static task write_then_read(d2d_hipi_test_base base,
                              logic [AXI_ID_W-1:0]    wr_id   = 0,
                              logic [AXI_ADDR_W-1:0]  wr_addr = 'h1000,
                              logic [AXI_LEN_W-1:0]   wr_len  = 0,
                              logic [AXI_SIZE_W-1:0]  wr_size = 3'd6,
                              logic [AXI_BURST_W-1:0] wr_burst= 2'b01,
                              logic [AXI_DATA_W-1:0]  wr_data = 'hDEAD_BEEF,
                              logic [AXI_ID_W-1:0]    rd_id   = 1,
                              logic [AXI_ADDR_W-1:0]  rd_addr = 'h2000,
                              logic [AXI_LEN_W-1:0]   rd_len  = 0,
                              logic [AXI_SIZE_W-1:0]  rd_size = 3'd6,
                              logic [AXI_BURST_W-1:0] rd_burst= 2'b01);
    axi_txn wr = base.mst.build_write(wr_id, wr_addr, wr_len, wr_size, wr_burst);
    axi_txn rd = base.mst.build_read (rd_id, rd_addr, rd_len, rd_size, rd_burst);
    wr.gen_wdata(wr_data);
    $display("%0t [SEQ] write_then_read: %s / %s", $time, wr.to_string(), rd.to_string());
    base.mst.write_and_read(wr, rd);
  endtask

  // ---------------------------------------------------------------
  // 多 beat 写
  // ---------------------------------------------------------------
  static task burst_write(d2d_hipi_test_base base,
                          logic [AXI_ADDR_W-1:0] addr,
                          logic [AXI_LEN_W-1:0]  len,
                          logic [AXI_SIZE_W-1:0] size = 3'd6,
                          logic [AXI_ID_W-1:0]   id   = 0,
                          logic [AXI_DATA_W-1:0] seed = 'hB000_0000);
    axi_txn t = base.mst.build_write(id, addr, len, size, 2'b01);
    t.gen_wdata(seed);
    base.mst.write(t);
    $display("%0t [SEQ] burst_write issued: %s", $time, t.to_string());
  endtask

  // ---------------------------------------------------------------
  // 多 beat 读
  // ---------------------------------------------------------------
  static task burst_read(d2d_hipi_test_base base,
                         logic [AXI_ADDR_W-1:0] addr,
                         logic [AXI_LEN_W-1:0]  len,
                         logic [AXI_SIZE_W-1:0] size = 3'd6,
                         logic [AXI_ID_W-1:0]   id   = 0);
    axi_txn t = base.mst.build_read(id, addr, len, size, 2'b01);
    base.mst.read(t);
    $display("%0t [SEQ] burst_read issued: %s", $time, t.to_string());
  endtask

  // ---------------------------------------------------------------
  // ctrl 域 APB 写
  // ---------------------------------------------------------------
  static task apb_write_ctrl(d2d_hipi_test_base base,
                             logic [15:0] offset, logic [31:0] data);
    logic ser;
    base.apb.apb_write_ctrl(offset, data, ser);
    if (ser) $display("%0t [SEQ] APB WRITE SLVERR @ctrl 0x%h = %h", $time, offset, data);
    else     $display("%0t [SEQ] APB WRITE ctrl 0x%h = 0x%h", $time, offset, data);
  endtask

  // ---------------------------------------------------------------
  // ctrl 域 APB 读回校验
  // ---------------------------------------------------------------
  static task apb_read_check_ctrl(d2d_hipi_test_base base,
                                  logic [15:0] offset, logic [31:0] exp);
    logic [31:0] rd;
    logic        ser;
    base.apb.apb_read_ctrl(offset, rd, ser);
    `D2D_TB_CHECK(!ser, $sformatf("APB read ctrl 0x%h no SLVERR", offset))
    `D2D_TB_CHECK(!ser && (rd === exp),
                  $sformatf("APB read ctrl 0x%h exp=0x%h got=0x%h", offset, exp, rd))
  endtask

endclass

`endif  // D2D_HIPI_SEQ_LIB_SV
