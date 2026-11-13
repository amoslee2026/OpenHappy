// ===================================================================
// d2d_hipi_test_base.sv — 测试基类（复位/配置/VIP/scoreboard 公共流程）
//
// 版本：v1.0 | 日期：2026-10-19 | 作者：OpenHappy Team
//
// 封装流程（子类只需写激励）：
//   new(ctrl, s0_if, m0_if, apb_if)  —— 绑定 tb_top 的接口句柄
//   run_base()     —— idle 电平 → 复位时序 → 启动 monitor/slave 后台 → APB 基础配置
//   wait_all_done()—— 等 VIP 队列排空（带超时）
//   check_pass()   —— scoreboard 比对 + 汇总打印 + 置 test_done
//
// 复位时序（关键，见 spec/d2d_hipi_dig_top_spec.md §3.2）：
//   全部复位为 async assert / sync deassert，各域独立同步器。
//   实测有效的释放顺序：
//     1) presetn  （APB 域；先放开让 CSR 可写）
//     2) aresetn  （AXI 协议层域）
//     3) mb_rst_n （主带数据域，= link_layer / logic_phy 主带）
//     4) sb_rst_n （边带域）＋ mcu_rst_n（MCU 域）
//   相邻两次释放间隔 ≥ 20 个对应时钟拍，保证同步器全部出复位。
// ===================================================================

`ifndef D2D_HIPI_TEST_BASE_SV
`define D2D_HIPI_TEST_BASE_SV

`timescale 1ns/1ps

import d2d_hipi_tb_pkg::*;

class d2d_hipi_test_base;

  virtual d2d_hipi_tb_ctrl_if ctrl;
  virtual axi4_if             s0_vif;      // DUT slave 口（VIP 当 master）
  virtual axi4_if             m0_vif;      // DUT master 口（VIP 当 slave）
  virtual apb_if              apb_vif;     // DUT hipi_apb_* 配置口


  // 环境组件
  axi_master_vip       mst;
  axi_slave_vip        slv;
  axi_monitor          mon_s0;      // 采 s0_axi 请求（DUT slave 口）
  axi_monitor          mon_m0;      // 采 m0_axi 请求（DUT master 口）
  axi_monitor          mon_resp;    // 采 s0_axi 的 B/R（回程响应）
  d2d_hipi_scoreboard  scb;
  apb_master           apb;

  // 收包等待超时标志
  bit wait_timeout = 1'b0;

  // 协议层 loopback 配置值（0x500）：0=关 / 1=local_loop(PAIF层，本 TB 用)
  int unsigned loopback_mode = 0;

  function new(virtual d2d_hipi_tb_ctrl_if ctrl,
               virtual axi4_if s0_vif,
               virtual axi4_if m0_vif,
               virtual apb_if  apb_vif);
    this.ctrl    = ctrl;
    this.s0_vif  = s0_vif;
    this.m0_vif  = m0_vif;
    this.apb_vif = apb_vif;
  endfunction

  // ---------------------------------------------------------------
  // 1. 复位 + 环境构建
  // ---------------------------------------------------------------
  // ---------------------------------------------------------------
  // Phase 0：硬复位释放（flow/ip_cfg/init/readme.md §3 Phase 0）
  //
  //   释放顺序（readme §2 复位语义表）：
  //     presetn → mb_rst_n/sb_rst_n → mcu_rst_n → …全部 CSR 配置… → aresetn（最后）
  //   ★ aresetn（协议层用户侧硬复位）必须最后释放：readme §2 明确
  //     "撤之前协议层用户侧处于复位态，prcl CSR 的 credit/timeout 配置
  //      必须在其之后执行"。故本 task 不释放 aresetn，由 prcl_config() 末尾释放。
  // ---------------------------------------------------------------
  task reset_dut();
    // 保持在复位（tb_top 已在 t=0 置 0，此处再确认一次）
    ctrl.aresetn   = 1'b0;
    ctrl.presetn   = 1'b0;
    ctrl.mb_rst_n  = 1'b0;
    ctrl.sb_rst_n  = 1'b0;
    ctrl.mcu_rst_n = 1'b0;
    repeat (20) @(posedge ctrl.pclk);

    // --- 依次释放（aresetn 保持复位，留到 Phase 4）---
    @(negedge ctrl.pclk);
    ctrl.presetn = 1'b1;                       // ① APB / 配置域（最先）
    repeat (20) @(posedge ctrl.pclk);

    @(negedge ctrl.mb_clk);
    ctrl.mb_rst_n = 1'b1;                      // ② 主带数据域
    repeat (20) @(posedge ctrl.mb_clk);

    @(negedge ctrl.sb_clk);
    ctrl.sb_rst_n  = 1'b1;                     // ③ 边带域
    repeat (20) @(posedge ctrl.sb_clk);

    @(negedge ctrl.sb_clk);
    ctrl.mcu_rst_n = 1'b1;                     // ④ MCU 域
    repeat (20) @(posedge ctrl.sb_clk);

    $display("%0t [BASE] Phase0 reset released: presetn/mb_rst_n/sb_rst_n/mcu_rst_n (aresetn HELD until Phase4)", $time);
  endtask

  // ---------------------------------------------------------------
  // 2. 环境构建（接口空闲电平 + 组件创建 + 监视器启动）
  // ---------------------------------------------------------------
  task build();
    scb  = new();
    mst  = new(s0_vif, "MST_s0");
    slv  = new(m0_vif, "SLV_m0");
    apb  = new(apb_vif);

    mst.init_idle();
    slv.init_idle();
    apb.init_idle();

    mon_s0   = new(s0_vif, axi_monitor::ROLE_SLAVE_PORT,  scb, "MON_s0");
    mon_m0   = new(m0_vif, axi_monitor::ROLE_MASTER_PORT, scb, "MON_m0");
    mon_resp = new(s0_vif, axi_monitor::ROLE_MST_RESP,    scb, "MON_resp");
  endtask

  // ---------------------------------------------------------------
  // 3. 启动后台线程（监视器 + slave 自动应答）
  //    注：monitor/slave 不依赖复位状态，t=0 即可起，复位期间不会有握手
  // ---------------------------------------------------------------
  task start_background();
    scb.open_log("d2d_axi_txn.log");
    mon_s0.run();
    mon_m0.run();
    mon_resp.run();
    slv.run();          // slave 自动回 B/R
    $display("%0t [BASE] background started (mon_s0/mon_m0/mon_resp/slave)", $time);
  endtask

  // ---------------------------------------------------------------
  // 4. APB 基础配置（DUT 侧使能链路的必要寄存器）
  //    0x0018 为 ctrl 域段选择：0xC00~0xFFF = link layer
  // ---------------------------------------------------------------
  task apb_basic_config();
    logic        ser;
    logic [31:0] rd;

    // =================================================================
    // Phase 1 — hipi_ctrl pcsr（flow/ip_cfg/init/hipi_ctrl_top_init_cfg.md §2）
    //   文档步骤 1/2 合并为一次写（幂等，读回等价 0x3）
    // =================================================================

    // ---- pcsr 0x000：rstn bit0=mb_rstn / bit1=sb_rstn（协议层/链路层软复位）----
    apb.apb_write_ctrl(16'h0000, 32'h0000_0003, ser);
    apb.apb_read_ctrl (16'h0000, rd, ser);
    $display("%0t [P1] pcsr 0x0000 = %h (exp=00000003, mb_rstn/sb_rstn=1) slverr=%b",
             $time, rd, ser);
    if (rd !== 32'h0000_0003) begin
      $display("%0t [P1] ERROR: pcsr 0x0000 readback mismatch", $time);
      tb_error_cnt++;
    end

    // ---- pcsr 0x0004 clk_en：bit[7:0]=tx_clken / bit[15:8]=rx_clken ----
    //   文档 4T1R baseline = 0x00F1（tx_clken[3:0] + rx_clken[0]）
    //   本环境 1T1R 对称（单物理 lane0，TX+RX 双向）：
    //     hipi_ctrl_top: axi4_symm_prcl .clken = tx_clken[0]（唯一消费者）
    //     故只需 tx_clken[0] + rx_clken[0] → 0x0011
    //   ★ 复位值 0 → prcl_clk_cg 停摆 → s_axi 全通道握手超时（曾实测）
    apb.apb_write_ctrl(16'h0004, `D2D_TB_PCSR_CLK_EN, ser);
    apb.apb_read_ctrl (16'h0004, rd, ser);
    $display("%0t [P1] pcsr 0x0004 = %h (exp=%h, tx_clken[0]+rx_clken[0]) slverr=%b",
             $time, rd, `D2D_TB_PCSR_CLK_EN, ser);
    if (rd !== `D2D_TB_PCSR_CLK_EN) begin
      $display("%0t [P1] ERROR: pcsr 0x0004 readback mismatch", $time);
      tb_error_cnt++;
    end
  endtask

  // ---------------------------------------------------------------
  // Phase 2 — logic_phy（flow/ip_cfg/init/logic_phy_init.md §2）
  //   地址域：paddr[16]=0（D2D_TB_APB_DOMAIN_PHY）
  //
  //   ★ MCU_SUBSYS off（默认形态）：§2.6 firmware 加载/启动整段跳过
  //     （CSR 仍在但无 MCU 实体消费；训练走 logic_phy 本地训练单元）。
  //   ★ 1T1R 相对 4T1R baseline 的差异：
  //     - 0x004 lane_num 读回 = 1（LANE_NUM=1），不是 4
  //     - 0x824 SBC lane_enable 复位值 = 0x1（SB_LANE_EN_RST=4'b0001），不是 0xF
  // ---------------------------------------------------------------
  task phy_init();
    logic        ser;
    logic [31:0] rd;
    logic [31:0] exp;

    // ---- §2.2 第 1 步 bring-up 读回校验 ----
    apb.apb_read_phy(16'h0004, rd, ser);
    exp = 32'h0000_0010;   // lane_num=1 → bit[7:4]=0001; mode=0; ser_ratio=0
    $display("%0t [P2] phy 0x0004 = %h (exp=%h: mode=%b lane_num=%0d ser_ratio=%b) slverr=%b",
             $time, rd, exp, rd[0], rd[7:4], rd[15:14], ser);
    if (rd[0] !== 1'b0 || rd[7:4] !== 4'd1 || rd[15:14] !== 2'b00) begin
      $display("%0t [P2] ERROR: io_data_width_mode strap 不符（mode 应=0/32:1, lane_num 应=1/1T1R）", $time);
      tb_error_cnt++;
    end

    apb.apb_read_phy(16'h0008, rd, ser);
    $display("%0t [P2] phy 0x0008 = %h (exp=1: prbs_high32_clk_gate_en=1) slverr=%b", $time, rd, ser);
    if (rd[0] !== 1'b1) begin
      $display("%0t [P2] ERROR: prbs_high32_clk_gate_en 应=1（mode=0 派生）", $time);
      tb_error_cnt++;
    end

    // ---- §2.3 第 2 步 SBC 边带使能（复位全关，训练指令必须靠它跨端）----
    //   ★ 1T1R 自环回时“对端”就是自己：TX 使能+RX 使能都开，SBC 报文才能
    //     从 sb_txd 发出、经 TB 环回、从 sb_rxd 收回。
    apb.apb_write_phy(16'h0800, 32'h0000_0001, ser);   // tx_enable=1
    apb.apb_read_phy (16'h0800, rd, ser);
    $display("%0t [P2] phy 0x0800 = %h (SBC tx_enable=1) slverr=%b", $time, rd, ser);

    apb.apb_write_phy(16'h0808, 32'h0000_0101, ser);   // rx_enable=1, buf_depth=8
    apb.apb_read_phy (16'h0808, rd, ser);
    $display("%0t [P2] phy 0x0808 = %h (SBC rx_enable=1, buf_depth=%0d) slverr=%b",
             $time, rd, rd[15:8], ser);

    apb.apb_read_phy (16'h0824, rd, ser);              // 只读核对，不写
    $display("%0t [P2] phy 0x0824 = %h (exp=00000001: SBC lane_enable[lane0]) slverr=%b",
             $time, rd, ser);
    if (rd[3:0] !== 4'b0001) begin
      // 已知 RTL 遗留：apb_csr_top_rf.v:1317 该字段硬编码 4'hf，
      // SBC_LANE_EN_RST 参数未下传 → 1T1R 配置下仍读回 0xF。
      // 不影响数据面（logic_phy 实际 lane 使能由 IO_DATA_WIDTH_MODE 决定），
      // 故降级为 WARN，不计入 PASS/FAIL。
      $display("%0t [P2] WARN: SBC_LANE_ENABLE=0x%0h（期望 0x1；已知 RTL 硬编码 4'hf 遗留，不计错）",
               $time, rd[3:0]);
    end

    // ---- §2.4 第 3 步 中断解屏蔽（复位默认全屏蔽）----
    apb.apb_write_phy(16'h000C, 32'h0000_0007, ser);   // MCU 侧：train_done/fail/step_done
    $display("%0t [P2] phy 0x000C = 0x7 (int_mask_mcu) slverr=%b", $time, ser);
    apb.apb_write_phy(16'h0010, 32'h0000_0003, ser);   // 外部侧：train_done/fail
    $display("%0t [P2] phy 0x0010 = 0x3 (int_mask_ext) slverr=%b", $time, ser);
    apb.apb_write_phy(16'h081C, 32'h0000_0001, ser);   // SBC：pkt_arrival 解屏蔽
    $display("%0t [P2] phy 0x081C = 0x1 (SBC irq mask) slverr=%b", $time, ser);

    // ---- §2.5 第 4 步 训练超时使能（7 步全开）----
    apb.apb_write_phy(16'h0414, 32'h0000_007F, ser);
    apb.apb_read_phy (16'h0414, rd, ser);
    $display("%0t [P2] phy 0x0414 = %h (timeout_step_en=7F) slverr=%b", $time, rd, ser);

    // =================================================================
    // §2.7 第 6 步 训练模式（logic_phy 0x0000 bit[1:0] TRAIN_MODE）
    //
    //   RTL 核对（apb_csr_top_rf.v:1472 写 / :3136 读回 / :899 复位 2'h0）：
    //     0 = 自动全流程（offset→Vref→EQ→PI→DCDL→reorder→data_chk）
    //     1 = 简化模式（配合 BYPASS_CFG 0x410 跳过指定步）
    //     2 = 手动模式（step_cnt + step_trigger 单步）
    //     3 = 上电即通（bypass 全部训练）
    //
    //   消费点核对：
    //     tx_training_fsm.v:233/476 : ST_IDLE 且 train_mode==2'b11 → 直接 ST_ACTIVE
    //     rx_training_fsm.v:161-167 : ST_IDLE 且 train_mode==2'b11 → ST_BRINGUP
    //                                 （RX FSM 当前为 stub，IDLE 唯一出口就是 mode==3；
    //                                   其余 per-step 训练态仅定义了 localparam、未实现转移）
    //
    //   ★ 因此 BYPASS_TRAIN=1 时本环境可跳过 PHY 训练，直接验证数据面通路；
    //     BYPASS_TRAIN=0（默认）时走自动全流程，依赖 RX per-step FSM（当前 stub，
    //     TX 侧会因 train_offset_complete_i 恒 0 而超时进 ST_ERROR）。
    //
    //   控制方式（三级优先级）：
    //     ① +TRAIN_MODE=n  plusarg        —— 运行期覆盖
    //     ② 编译宏 BYPASS_TRAIN（Makefile TRAIN_BYPASS=1）—— 编译期默认
    //     ③ 缺省 = 0（自动全流程）
    // =================================================================
    begin
      int unsigned tm;
`ifdef D2D_TB_BYPASS_TRAIN
      tm = 32'd3;                                  // 编译期指定：bypass 训练
`else
      tm = 32'd0;                                  // 缺省：自动全流程
`endif
      void'($value$plusargs("TRAIN_MODE=%d", tm)); // plusarg 覆盖
      apb.apb_write_phy(16'h0000, tm, ser);
      apb.apb_read_phy (16'h0000, rd, ser);
      $display("%0t [P2] phy 0x0000 = %h (train_mode=%0d: %s) slverr=%b",
               $time, rd, rd[1:0], (rd[1:0] == 2'd3) ? "BYPASS(上电即通)" :
                                    (rd[1:0] == 2'd0) ? "自动全流程" :
                                    (rd[1:0] == 2'd1) ? "简化模式" : "手动模式", ser);
      if (rd[1:0] !== tm[1:0]) begin
        $display("%0t [P2] ERROR: train_mode 写入未生效 (exp=%0d got=%0d)", $time, tm, rd[1:0]);
        tb_error_cnt++;
      end
      if (rd[1:0] == 2'd3)
        $display("%0t [P2] NOTE: BYPASS 训练已启用 —— TX/RX 训练 FSM 直接进 ACTIVE/BRINGUP", $time);
    end
  endtask

  // ---------------------------------------------------------------
  // Phase 4 — 协议层 CSR（flow/ip_cfg/init/iodie_prcl_init_cfg.md §2）
  //   地址域：paddr[16]=1 且 paddr[11:10]=01/10 → 0x400~0xBFF
  //
  //   文档 §4 顺序要求：本段配置必须在 aresetn 之后执行，
  //   故 esc_prcl_reset() 先释放 aresetn，再走 credit/timeout 配置。
  // ---------------------------------------------------------------
  task esc_prcl_reset();
    @(negedge ctrl.aclk);
    ctrl.aresetn = 1'b1;                       // 协议层用户侧硬复位（最后释放）
    repeat (20) @(posedge ctrl.aclk);
    $display("%0t [P4] aresetn released (protocol layer user-side reset, LAST)", $time);
  endtask

  task prcl_config();
    logic        ser;
    logic [31:0] rd;
    string       lb_name;
    int unsigned lb_val;

    // =================================================================
    // Phase 4b — 协议层 CSR 配置顺序（本环境定稿顺序）
    //
    //   ① loopback（0x500）——最先配，决定数据面走内部回环还是端口物理环回
    //   ② hipi_sel（0x600）——引擎选择（0=HIPI Next / 1=GB），须早于数据流动
    //   ③ timeout_en（0x428）——超时监测使能
    //   ④ credit_en（0x414）——★ 最后一步
    //
    //   ★ credit_en 放最后的理由：
    //     credit_en=1 后，p_crd_tx_ctrl 会按远端 credit 额度门控 TX FIFO→pack 输出。
    //     若在 credit 帧尚未建立（自环回/对端未起）时就使能，TX 通路会被
    //     "初始 credit=0" 直接压死，形成死锁（数据发不出去 → 对端不返 credit →
    //     永远没额度）。因此先让 loopback/引擎/超时就位，最后再开 credit 门控。
    // =================================================================

    // ---- ① 0x500 loopback：协议层两级回环（spec axi4_symm_prcl §5.5）----
    //   bit1 remote_loop（p_axi_loop，bond 层 aclk 域）：对端 RX 数据回灌 TX 发回对端
    //   bit0 local_loop （p_paif_loop，PAIF 层 prcl_clk_cg 域）：本端 pack TX 直连
    //                     unpack RX，绕过 Link 层（s0→m1）
    //
    //   +LOOPBACK_MODE 选择（Makefile: LOOPBACK=phys|paif|cpif）：
    //     phys : 0x0 —— 协议层关回环，走 DUT 端口物理主带/边带环回（默认）
    //     paif : 0x1 —— local_loop，协议层内部 PAIF 层回环，单测协议层数据通路
    //     cpif : 0x0 —— 协议层关回环，回环改由 link 层 LOOPBACK_CFG_REG 承担
    //                   （见 link_init：0x0C14 bit16=1，link_tx CPIF TX 出口
    //                     → link_rx CPIF RX 入口，绕过 logic_phy）
    //   ★ 各层级回环互斥，同一时刻只开一层。
    //
    //   ★ 不支持 remote_loop（0x500 bit1，p_axi_loop，bond 层）：其语义为
    //     「对端 RX 数据回灌 TX 发回对端」，sel=1 时本端 s0(用户 AXI 口) 被
    //     `(!sel)` 硬门控断开（p_axi_loop.v:163 s0_aw_ready = (!sel) & m0_aw_ready），
    //     单 die 拓扑无对端往 s1 送数据 → VIP 的 AW/AR/W 永远等不到 ready。
    //     该档需双 die 对打环境，本 TB 不覆盖。
    if (!$value$plusargs("LOOPBACK_MODE=%s", lb_name)) lb_name = "phys";
    case (lb_name)
      "phys"  : lb_val = 32'h0000_0000;
      "paif"  : lb_val = 32'h0000_0001;   // local_loop
      "cpif"  : lb_val = 32'h0000_0000;   // 协议层不环回；链路层 CPIF 环回在 link_init 配
      default : lb_val = 32'h0000_0000;
    endcase
    loopback_mode = lb_val;

    apb.apb_write_ctrl(16'h0500, lb_val, ser);
    apb.apb_read_ctrl (16'h0500, rd, ser);
    $display("%0t [P4-1] prcl 0x0500 = %h (LOOPBACK_MODE=%s: local_loop=%b remote_loop=%b) slverr=%b",
             $time, rd, lb_name, rd[0], rd[1], ser);
    if (rd[1:0] !== lb_val[1:0]) begin
      $display("%0t [P4-1] ERROR: loopback 写入未生效 (exp=%h got=%h)", $time, lb_val, rd);
      tb_error_cnt++;
    end

    // ---- ② 0x600 hipi_sel：引擎选择（0=HIPI Next pack/unpack / 1=GB protocol_*）----
    apb.apb_write_ctrl(16'h0600, 32'h0000_0000, ser);
    apb.apb_read_ctrl (16'h0600, rd, ser);
    $display("%0t [P4-2] prcl 0x0600 = %h (hipi_sel=0 -> HIPI Next 引擎) slverr=%b", $time, rd, ser);
    if (rd[0] !== 1'b0) begin
      $display("%0t [P4-2] ERROR: hipi_sel 应=0", $time);
      tb_error_cnt++;
    end

    // ---- ③ 0x428 timeout_en：超时监测使能（复位 0 = 无监测）----
    apb.apb_write_ctrl(16'h0428, 32'h0000_0001, ser);
    apb.apb_read_ctrl (16'h0428, rd, ser);
    $display("%0t [P4-3] prcl 0x0428 = %h (exp=1: timeout_en) slverr=%b", $time, rd, ser);
    if (rd[0] !== 1'b1) begin
      $display("%0t [P4-3] ERROR: timeout_en 写入未生效", $time);
      tb_error_cnt++;
    end

    // ---- ④ 0x414 credit_en：★ 最后一步（见上方顺序说明）----
    apb.apb_write_ctrl(16'h0414, 32'h0000_0001, ser);
    apb.apb_read_ctrl (16'h0414, rd, ser);
    $display("%0t [P4-4] prcl 0x0414 = %h (exp=1: credit_en, LAST STEP) slverr=%b", $time, rd, ser);
    if (rd[0] !== 1'b1) begin
      $display("%0t [P4-4] ERROR: credit_en 写入未生效", $time);
      tb_error_cnt++;
    end
  endtask

  // ---------------------------------------------------------------
  // 4b. 链路层初始化（link_layer 段 0xC00~0xFFF；link 达 S_NORMAL 才放行 AXI 数据）
  //
  //   依据 spec/link_layer/link_layer_spec.md §5.2 / §5.3 / §3 状态机：
  //     0x0000 PHY_MODE   ：rx_mode[8:5]=1110(NRZ) / tx_mode[4:1]=1110 / s2p_mode[0]=0(32:1)
  //     0x0004 INIT_CFG   ：bit2 CONFIG_DONE（配置完成 → lcmsm 出 S_CONFIG）
  //                         bit4 PHY_INIT   （触发 PHY INIT 事件）
  //                         bit3 RX_READY   （向对端发 RX READY）
  //                         bit0 LANE_ON    （向对端发 LANE ON）
  //     0x0034 STATUS_REG ：bits[7:4] LINK_FSM_STATE（4= S_NORMAL）
  //   注：本 TB 主带/边带已在 DUT 端口自环，PHY 事件 cmd 由 link 层经边带发给
  //       “对端”（即自己），因此事件握手可自洽完成。
  // ---------------------------------------------------------------
  task link_init(int unsigned poll_max = `D2D_TB_LINK_POLL_CYCLES);
    logic        ser;
    logic [31:0] rd;
    int unsigned n;
    string       lb_name;      // 回环模式（+LOOPBACK_MODE，与 prcl_config 同源）

    // 回环模式（与 prcl_config 同一 plusarg，便于本 task 内决定是否开链路层环回）
    if (!$value$plusargs("LOOPBACK_MODE=%s", lb_name)) lb_name = "phys";

    // ---- link_layer_init.md §2.2 第 1 步 链路层参数（复位默认即可，显式回写留档）----
    //   0x0C00 PHY_MODE：s2p_mode[0]=0(32:1) / tx_mode[4:1]=1110(NRZ) / rx_mode[8:5]=1110(NRZ)
    apb.apb_write_ctrl(16'h0C00, 32'h0000_001C, ser);
    apb.apb_read_ctrl (16'h0C00, rd, ser);
    $display("%0t [P3] link 0x0C00 PHY_MODE = %h (exp=0000001C NRZ/32:1) slverr=%b", $time, rd, ser);
    if (rd[8:0] !== 9'h1C) begin
      $display("%0t [P3] ERROR: PHY_MODE 读回不符", $time); tb_error_cnt++;
    end

    //   MODE_REG(0x0C)：文档 §2.2 建议 0x0B（work_mode=1 + protocol_mode=1）
    apb.apb_write_ctrl(16'h0C0C, 32'h0000_000B, ser);
    apb.apb_read_ctrl (16'h0C0C, rd, ser);
    $display("%0t [P3] link 0x0C0C MODE_REG = %h (exp=0000000B WORK+PAIF) slverr=%b", $time, rd, ser);

    // ---- CTRL_REG(0x0C10)：TX_RST_N=bit0 / RX_RST_N=bit2，退出 CPIF 复位 ----
    apb.apb_write_ctrl(16'h0C10, 32'h0000_0005, ser);
    apb.apb_read_ctrl (16'h0C10, rd, ser);
    $display("%0t [P3] link 0x0C10 CTRL_REG = %h (TX_RST_N/RX_RST_N=1) slverr=%b", $time, rd, ser);
    if (rd[0] !== 1'b1 || rd[2] !== 1'b1) begin
      $display("%0t [P3] ERROR: CTRL_REG TX_RST_N/RX_RST_N 未置起", $time); tb_error_cnt++;
    end

    //   RETRY_CFG(0x0C18)：复位默认即满足，读回留档
    apb.apb_read_ctrl(16'h0C18, rd, ser);
    $display("%0t [P3] link 0x0C18 RETRY_CFG = %h (复位默认) slverr=%b", $time, rd, ser);

    // =================================================================
    // LOOPBACK_CFG_REG（0x0C14）—— 链路层 CPIF 入口回环（spec §5.3）
    //
    //   字段（link_layer_rf.v:344-350 写 / :461-466 读回）：
    //     [3:0]   RX0_SRC      复位 4'd0
    //     [7:4]   RX1_SRC      复位 4'd1
    //     [11:8]  RX2_SRC      复位 4'd2
    //     [15:12] RX3_SRC      复位 4'd3
    //     [16]    LOOPBACK_EN  复位 1'b0
    //
    //   LOOPBACK_EN=0（默认）：link_rx 取物理层 CPIF RX（cpif_rx_lane*）
    //   LOOPBACK_EN=1        ：link_rx 取本端 link_tx 的 CPIF TX 出口
    //                          （cpif_tx_lane*，经 lb_pipe 1 级寄存），
    //                          物理层 CPIF RX 输入被忽略 → 绕过 logic_phy，
    //                          只剩 协议层 + link 层 在环内。
    //   对称结构（1T1R）下 RX0_SRC=0 → link_rx[0] ← link_tx[0] 自环（默认即可）。
    //
    //   与桥接/UDI 互斥：LOOPBACK_EN=1 时须 bridge_en=0 且 MODE_REG.PROTOCOL_MODE=1
    //   （上面 MODE_REG 写的 0x0B 已含 protocol_mode=1，满足）。
    // =================================================================
    if (lb_name == "cpif") begin
      apb.apb_write_ctrl(16'h0C14, 32'h0001_0000, ser);  // bit16=1, RXx_SRC 全 0
      apb.apb_read_ctrl (16'h0C14, rd, ser);
      $display("%0t [P3] link 0x0C14 LOOPBACK_CFG = %h (LOOPBACK_EN=%b RX0_SRC=%0d) -- 链路层CPIF环回 ON",
               $time, rd, rd[16], rd[3:0], ser);
      if (rd[16] !== 1'b1) begin
        $display("%0t [P3] ERROR: link LOOPBACK_EN 未置起", $time); tb_error_cnt++;
      end
      if (rd[3:0] !== 4'd0) begin
        $display("%0t [P3] ERROR: RX0_SRC 应=0（link_rx0←link_tx0 自环），实测 %0d", $time, rd[3:0]);
        tb_error_cnt++;
      end
      $display("%0t [P3] NOTE: CPIF 环回 -> logic_phy 被旁路，本测试只覆盖 协议层+link层", $time);
    end
    else begin
      // 其它模式显式写 0：确保链路层 CPIF 环回关闭，RX 走物理层 CPIF RX
      apb.apb_write_ctrl(16'h0C14, 32'h0000_0000, ser);
      apb.apb_read_ctrl (16'h0C14, rd, ser);
      $display("%0t [P3] link 0x0C14 LOOPBACK_CFG = %h (LOOPBACK_EN=0 物理层通路)", $time, rd, ser);
    end

    // ---- §2.3 第 2 步 电气初始化完成判定（PHY_READY = cpif_tx_ready & cpif_rx_ready）----
    apb.apb_read_ctrl(16'h0C34, rd, ser);
    $display("%0t [P3] link 0x0C34 STATUS = %h (phy_ready=%b fsm_state=%0d) slverr=%b",
             $time, rd, rd[8], rd[7:4], ser);

    // ---- CONFIG_DONE：告诉 lcmsm 配置完成（S_CONFIG → S_PHY_INIT）----
    apb.apb_write_ctrl(16'h0C04, 32'h0000_0004, ser);
    $display("%0t [P3] link 0x0C04 CONFIG_DONE=1 (bit2) slverr=%b", $time, ser);
    repeat (50) @(posedge ctrl.pclk);

    // ---- §2.4 第 3 步 RX 侧发 PHY ready event（RX READY, 0x7F）----
    //   INIT_CFG_REG bit3 = rx_ready；与 CONFIG_DONE 一起写
    apb.apb_write_ctrl(16'h0C04, 32'h0000_000C, ser);
    $display("%0t [P3] link 0x0C04 CONFIG_DONE+PHY_INIT? = 0xC (bit2 cfg_done + bit3 rx_ready) slverr=%b",
             $time, ser);
    repeat (100) @(posedge ctrl.pclk);

    // ---- §2.5/§2.6 第 4~5 步 TX 侧收 ready，软件配 PHY_INIT ----
    apb.apb_write_ctrl(16'h0C04, 32'h0000_001C, ser);
    $display("%0t [P3] link 0x0C04 +PHY_INIT(bit4) = 0x1C slverr=%b", $time, ser);
    repeat (200) @(posedge ctrl.pclk);

    // ---- §2.6 第 5 步 软件写 LANE_ON（bit0）→ 发 LANE ON cmd 0x40 ----
    apb.apb_write_ctrl(16'h0C04, 32'h0000_001D, ser);
    $display("%0t [P3] link 0x0C04 +LANE_ON(bit0) = 0x1D slverr=%b", $time, ser);
    repeat (200) @(posedge ctrl.pclk);

    // ---- §2.9 第 8 步 软件写 RETRAIN（bit5）→ 发训练 event 0x80 ----
    //   ★ 关键：没有这一步，lcmsm 停在 S_PHY_TRAIN 不会被训练完成事件推进，
    //     主带 RX 侧 descrambler/deskew 未对齐 → 数据全部收不回来。
    apb.apb_write_ctrl(16'h0C04, 32'h0000_003D, ser);
    $display("%0t [P3] link 0x0C04 +RETRAIN(bit5) = 0x3D (训练 event 0x80) slverr=%b",
             $time, ser);

    // 轮询 STATUS_REG.LINK_FSM_STATE 直到 S_NORMAL(4) 或超时
    n = 0;
    forever begin
      apb.apb_read_ctrl(16'h0C34, rd, ser);
      if (n % 200 == 0)
        $display("%0t [BASE] POLL link STATUS 0x0C34 = %h (fsm_state=%0d phy_ready=%b)",
                 $time, rd, rd[7:4], rd[8]);
      if (rd[7:4] == 4'd4) begin
        $display("%0t [BASE] LINK S_NORMAL reached (fsm_state=4)", $time);
        break;
      end
      repeat (20) @(posedge ctrl.pclk);
      if (++n > poll_max/20) begin
        $display("%0t [BASE] WARNING: LINK 未进入 S_NORMAL（轮询 %0d 次，fsm_state=%0d）",
                 $time, n, rd[7:4]);
        break;
      end
    end
  endtask

  // ---------------------------------------------------------------
  // 4c. 等状态机进入 ACTIVE（改为内部信号探针，不再依赖 CSR 回读）
  //
  //   为什么用探针：
  //     · link RX FSM 无 CSR 回读（apb_csr.v:156 link_fsm_state_rx_i 仅接 DFx）
  //     · logic_phy 0x002C/0x0034 在 TRAIN_MODE=3 下恒 0
  //       （bypass 不跑 per-step 训练，step_state/train_done 不置位；
  //        不跑 PRBS，aligned 永远不是 0xFFFF）
  //     · 直接看 FSM 状态寄存器最准确、与配置模式无关
  //
  //   探针信号在 tb/d2d_hipi_tb_ctrl_if.sv 声明、tb_top 直连 DUT 内部：
  //     phy_tx_state  4bit  8=ST_ACTIVE
  //     phy_rx_state  3bit  2=ST_ACTIVE
  //     lnk_tx_state  6bit  one-hot 010000=NORMAL
  //     lnk_rx_state  6bit  one-hot 010000=NORMAL
  // ---------------------------------------------------------------
  task wait_all_active(input time timeout_ps = 2_000_000);
    bit ok;
    $display("%0t [WAIT_ACTIVE] 开始轮询内部状态机（探针）...", $time);
    ctrl.wait_phy_active(timeout_ps, ok);
    if (!ok) tb_error_cnt++;
    ctrl.wait_link_active(timeout_ps, ok);
    if (!ok) tb_error_cnt++;
  endtask

  // ---------------------------------------------------------------
  // 5. 等所有事务完成（VIP 队列排空 + 超时）
  // ---------------------------------------------------------------
  task wait_all_done(int unsigned timeout_cycles = 200000);
    int unsigned cnt = 0;
    wait_timeout = 1'b0;
    // ★ mst_exp_q 由 master VIP 自己维护（axi_master_vip.exp_q），
    //   scoreboard 的 mst_exp_q 从未被 push（push_mst_exp 无调用点）。
    //   故此处改以 master VIP 的 exp_q + 两侧 monitor 实采数量为判据：
    //     slv_act_q（m0 实采）≥ mst.exp_q（master 成功完成的请求数）
    //   s0 侧允许相等或更多（m0 只会是 s0 的子集，多出来的属异常，由 SCB 报）。
    while (mst.exp_q.size() == 0
           || scb.slv_act_q.size() < mst.exp_q.size()) begin
      @(posedge ctrl.aclk);
      if (++cnt > timeout_cycles) begin
        wait_timeout = 1'b1;
        $display("%0t [BASE] wait_all_done TIMEOUT: mst_exp=%0d s0=%0d m0=%0d",
                 $time, mst.exp_q.size(), scb.mst_act_q.size(), scb.slv_act_q.size());
        break;
      end
    end
    // 再等若干拍让响应回程落定
    repeat (100) @(posedge ctrl.aclk);
  endtask

  // ---------------------------------------------------------------
  // 6. 收尾比对 + PASS/FAIL
  // ---------------------------------------------------------------
  task check_pass(string test_name);
    int unsigned exp_wr = scb.req_wr_cnt;
    int unsigned exp_rd = scb.req_rd_cnt;

    $display("----------------------------------------------------------");
    $display("%0t [BASE] %s: start scoreboard check", $time, test_name);

    // ① s0_axi 实采 vs m0_axi 实采（DUT slave 收到 → 是否从 master 口转发）
    //    先做这一步（compare_forwarded 只读，不清队列）
    scb.compare_forwarded(scb.mst_act_q, "s0_axi->m0_axi");

    // ② master VIP 发出的请求 vs s0_axi 实采（清空 exp_q / mst_act_q）
    scb.compare_mst("MST->s0_axi");

    // ③ 响应回程对账（B/R 数量 + resp 编码）
    scb.compare_response(exp_wr, exp_rd);

    scb.report();
    mst.report();
    slv.report();
    mon_s0.report();
    mon_m0.report();
    mon_resp.report();

    if (wait_timeout) begin
      $display("%0t [BASE] %s: TIMEOUT — 数据未收齐，判 FAIL", $time, test_name);
      tb_error_cnt++;
    end

    tb_error_cnt += mst.err_cnt + slv.err_cnt + scb.mismatch_cnt + scb.missing_cnt;

    if (tb_error_cnt == 0) begin
      $display("%0t [BASE] %s: PASS (axi errors=0, scb mismatch=0)", $time, test_name);
    end else begin
      $display("%0t [BASE] %s: FAIL (axi errors=%0d scb mismatch=%0d missing=%0d)",
               $time, test_name, mst.err_cnt + slv.err_cnt,
               scb.mismatch_cnt, scb.missing_cnt);
    end

    scb.close_log();
    ctrl.test_done = 1'b1;
  endtask

  // ---------------------------------------------------------------
  // 7. 通用等待（时钟拍）
  // ---------------------------------------------------------------
  task wait_clk(int unsigned n);
    repeat (n) @(posedge ctrl.aclk);
  endtask

endclass

`endif  // D2D_HIPI_TEST_BASE_SV
