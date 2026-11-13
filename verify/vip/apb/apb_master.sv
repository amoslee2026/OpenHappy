// ===================================================================
// apb_master.sv — APB3 master VIP（复用 cxs_prcl/dv 版本，增 20bit 地址）
//
// 版本：v1.1 | 日期：2026-10-19 | 作者：OpenHappy Team
// 说明：v1.1 从 cxs_prcl/dv/vip/apb/apb_master.sv 复用：
//       - 地址位宽 16→20（DUT hipi_apb_paddr[19:0]）
//       - 保留原 SETUP→ACCESS 两相时序、pready 超时保护、批量读写/校验任务
//       - 新增 domain 便捷接口（apb_write_ctrl / apb_write_phy），
//         自动按 DUT logic_phy 内 MUX 规则置 paddr[16]
//
// APB3 两相：SETUP（psel=1, penable=0）→ ACCESS（psel=1, penable=1, 等 pready）。
// ===================================================================

`ifndef APB_MASTER_SV
`define APB_MASTER_SV

import d2d_hipi_tb_pkg::*;

class apb_master;

  virtual apb_if vif;
  int unsigned err_cnt    = 0;
  int unsigned xact_count = 0;
  int          max_wait   = `D2D_TB_APB_TIMEOUT;   // pready 等待超时（pclk 周期数）
  // 沿后偏移：DUT 的 always @(posedge pclk) 采样沿【前】的值（NBA 语义），
  // 故所有驱动必须在沿之后（+skew）更新，否则 psel&penable&pwrite 永不在沿上同时为真，
  // 纯组合写的 CSR（pcsr / p_axi4_symm_csr）会丢写、读回恒 0。
  real         drive_skew  = 0.05ns;     // 50ps
  // pready 组合传播等待：penable/pready 可能在【同一拍】拉高（APB3 零等待），
  // 但 penable 是阻塞赋值，组合路径需 delta cycle 才稳定。留 1 ps 让组合逻辑
  // 传播完成再判 pready，避免误判为"本拍未 ready"而错过单拍窄脉冲。
  real         pready_settle = 0.05ns;  // 50ps（等组合传播，避开时钟沿）

  function new(virtual apb_if vif);
    this.vif = vif;
  endfunction

  // ---------------------------------------------------------------
  // 等待复位释放（presetn 拉高后再等若干 pclk，覆盖 DUT 内部同步释放）
  // 用 wait() 而非 @(posedge)：presetn 可能已为 1（无 0→1 跳变）
  // ---------------------------------------------------------------
  task wait_reset_done(int extra_cycles = 20);
    wait (vif.presetn === 1'b1);
    repeat (extra_cycles) @(posedge vif.pclk);
  endtask

  // 初始空闲电平
  task init_idle();
    vif.psel    = 1'b0;
    vif.penable = 1'b0;
    vif.paddr   = '0;
    vif.pwrite  = 1'b0;
    vif.pwdata  = '0;
  endtask

  // ---------------------------------------------------------------
  // APB3 写（20bit 地址，调用方保证 4-byte 对齐）
  // ---------------------------------------------------------------
  task apb_write(input  logic [`D2D_TB_APB_ADDR_WIDTH-1:0] addr,
                 input  logic [31:0] data,
                 output logic        slverr);
    int wcnt = 0;
    slverr = 1'b0;

    // ---- SETUP（沿后驱动 psel=1/penable=0）----
    @(posedge vif.pclk);
    #(drive_skew);
    vif.psel    = 1'b1;
    vif.penable = 1'b0;
    vif.paddr   = addr;
    vif.pwrite  = 1'b1;
    vif.pwdata  = data;

    // ---- ACCESS（沿后驱动 penable=1）----
    @(posedge vif.pclk);
    #(drive_skew);
    vif.penable = 1'b1;

    // ================================================================
    // ★ APB3 零等待传输：pready 允许与 penable **同拍**拉高
    //   （AMBA APB3 规范：ACCESS phase 内 pready 与 penable 同时为高即完成）
    //
    //   本 VIP 原实现缺陷：penable 用阻塞赋值刚置 1，立刻 `while(pready!==1)`，
    //   此时组合逻辑尚未经过 delta cycle 传播，读到的是【旧值 0】→ 误判未完成
    //   → 进入等下一拍 → 但 DUT 侧 pready 只在本拍(ACCESS)为高、下一拍已清 0
    //   → 永久死锁（实测 p_axi4_symm_csr 的 pready_reg 就是这种单拍窄脉冲）。
    //
    //   修复：先给组合逻辑留出 delta 传播时间（#0 或极小延迟），再采样；
    //   并且"本拍 pready=1 即完成"，不再强制多等一拍。
    // ================================================================
    #(pready_settle);            // 等组合逻辑 delta 传播（不跨时钟沿）

    while (vif.pready !== 1'b1) begin
      @(posedge vif.pclk);       // 本拍未 ready → 等下一拍再判
      #(drive_skew);
      if (++wcnt > max_wait) begin
        $display("%0t [APB] WRITE TIMEOUT @%h data=%h (pready never high)", $time, addr, data);
        err_cnt++;
        slverr = 1'b1;
        break;
      end
      #(pready_settle);          // 每拍同样留 delta 传播时间
    end

    // ★ 完成沿：写操作由 DUT 在该沿生效（apb_wr = psel&penable&pwrite 在各 CSR
    //   内部都是组合门控 + 时序块，实际写入发生在 ACCESS 拍结束的那个沿）。
    @(posedge vif.pclk);
    #(drive_skew);
    if (!slverr) begin
      slverr = vif.pslverr;
    end

    // ---- 回到 IDLE（沿C 之后撤销）----
    vif.psel    = 1'b0;
    vif.penable = 1'b0;
    vif.paddr   = '0;
    vif.pwrite  = 1'b0;
    vif.pwdata  = '0;
    @(posedge vif.pclk);
    xact_count++;
  endtask

  // ---------------------------------------------------------------
  // APB3 读
  // ---------------------------------------------------------------
  task apb_read(input  logic [`D2D_TB_APB_ADDR_WIDTH-1:0] addr,
                output logic [31:0] rdata,
                output logic        slverr);
    int wcnt = 0;
    rdata  = 32'd0;
    slverr = 1'b0;

    @(posedge vif.pclk);
    #(drive_skew);
    vif.psel    = 1'b1;
    vif.penable = 1'b0;
    vif.paddr   = addr;
    vif.pwrite  = 1'b0;
    vif.pwdata  = '0;

    @(posedge vif.pclk);
    #(drive_skew);
    vif.penable = 1'b1;

    // 同 apb_write：先留 delta 传播时间，再采样 pready（支持与 penable 同拍拉高）
    #(pready_settle);

    while (vif.pready !== 1'b1) begin
      @(posedge vif.pclk);
      #(drive_skew);
      if (++wcnt > max_wait) begin
        $display("%0t [APB] READ TIMEOUT @%h (pready never high)", $time, addr);
        err_cnt++;
        slverr = 1'b1;
        break;
      end
      #(pready_settle);
    end

    // ★ prdata 采样点：完成沿之后（DUT 内部三种读延迟都能覆盖）
    //   ① p_axi4_symm_csr  : prdata_reg 在 SETUP 沿载入 → ACCESS 拍已有效
    //   ② apb_csr_top(phy) : prdata_reg 在 ACCESS 沿载入 → 完成沿之后才有效
    //                          （源码头注释即写明"读 2 cycle"）
    //   ③ pcsr / link      : 组合或内部打拍，完成沿之后同样稳定
    //   撤 psel 前采样，且 prdata_reg 在撤后保持，读数安全。
    @(posedge vif.pclk);
    #(drive_skew);
    if (!slverr) begin
      rdata  = vif.prdata;
      slverr = vif.pslverr;
    end

    // ---- 回到 IDLE（沿后撤销）----
    vif.psel    = 1'b0;
    vif.penable = 1'b0;
    vif.paddr   = '0;
    vif.pwrite  = 1'b0;
    @(posedge vif.pclk);
    xact_count++;
  endtask

  // ---------------------------------------------------------------
  // 域内地址拼装（DUT logic_phy 内 MUX 按 hipi_apb_paddr[16] 分域）
  //   ctrl 域：paddr[16]=1，低 16bit 直通 hipi_ctrl_top
  //   phy  域：paddr[16]=0，低 16bit 直通 apb_csr_top
  // ---------------------------------------------------------------
  function automatic logic [`D2D_TB_APB_ADDR_WIDTH-1:0] mk_addr(
      input logic domain, input logic [15:0] offset);
    return {3'b000, domain, offset};
  endfunction

  task apb_write_ctrl(input logic [15:0] offset, input logic [31:0] data,
                      output logic slverr);
    apb_write(mk_addr(`D2D_TB_APB_DOMAIN_CTRL, offset), data, slverr);
  endtask

  task apb_read_ctrl(input logic [15:0] offset, output logic [31:0] rdata,
                     output logic slverr);
    apb_read(mk_addr(`D2D_TB_APB_DOMAIN_CTRL, offset), rdata, slverr);
  endtask

  task apb_write_phy(input logic [15:0] offset, input logic [31:0] data,
                     output logic slverr);
    apb_write(mk_addr(`D2D_TB_APB_DOMAIN_PHY, offset), data, slverr);
  endtask

  task apb_read_phy(input logic [15:0] offset, output logic [31:0] rdata,
                    output logic slverr);
    apb_read(mk_addr(`D2D_TB_APB_DOMAIN_PHY, offset), rdata, slverr);
  endtask

  // ---------------------------------------------------------------
  // 地址序列写
  // ---------------------------------------------------------------
  task apb_write_seq(input logic [`D2D_TB_APB_ADDR_WIDTH-1:0] addr[],
                     input logic [31:0] data[]);
    logic slverr;
    for (int i = 0; i < addr.size(); i++) begin
      apb_write(addr[i], data[i], slverr);
      if (slverr) begin
        $display("%0t [APB] WRITE SLVERR @%h data=%h", $time, addr[i], data[i]);
        err_cnt++;
      end
    end
  endtask

  // ---------------------------------------------------------------
  // 读回校验
  // ---------------------------------------------------------------
  task apb_read_verify(input logic [`D2D_TB_APB_ADDR_WIDTH-1:0] addr[],
                       input logic [31:0] exp[]);
    logic [31:0] rd;
    logic        slverr;
    for (int i = 0; i < addr.size(); i++) begin
      apb_read(addr[i], rd, slverr);
      if (slverr) begin
        $display("%0t [APB] READ SLVERR @%h", $time, addr[i]);
        err_cnt++;
      end else if (rd !== exp[i]) begin
        $display("%0t [APB] READ VERIFY FAIL @%h exp=%h got=%h", $time, addr[i], exp[i], rd);
        err_cnt++;
      end
    end
  endtask

endclass

`endif  // APB_MASTER_SV
