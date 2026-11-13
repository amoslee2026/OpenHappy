// =============================================================================
// module name：credit_fifo_bank
// author: OpenHappy Team
// 功能概述: RX credit FIFO 的通用寄存器阵列存储单元。纯存储阵列——不含读写指针
//           /FIFO 控制/满空/flush，指针与流控全部上提到父模块 credit_fifo。
//           由 credit_fifo 例化四片（u_bank0..u_bank3），每片 512 bit × CF_DEPTH
//           拼接成寄存器 FIFO，替代原 SRAM 硬宏实现。读路径按 CREDIT_FIFO_DEPTH_*
//           宏文本展开为等值门控 OR 读 mux——每档 DEPTH 下文本展开为
//           {DATA_WIDTH{rd_addr==N'dK}} & mem[K] 的等值门控 OR 链（当前激活
//           CREDIT_FIFO_DEPTH_16 → 4'd 字面量 16 路），读出延迟≈0，支撑 CPIF
//           接收与 PAIF 发送连续无气泡流水。
// 性能注(LL-20260924-CRIFVO): v1.6 mem 去复位清零（纯存储豁免，类 udi/paif_data_reg，
//           X 泄漏由 read 时序保证——ren 仅对 empty=0 bank 拉高，未写单元不进读出
//           路径）；rst_n 保留 port（unused，lint waiver 处理）；读 mux 等值门控
//           OR 展开逻辑等价 bit-exact，端口/参数/写路径不变。
//           2026-09-25：读 mux 等值门控 OR 展开 → 平衡二进制 2:1 树（16/32/64/128→4/5/6/7 级），同函数 bit-exact，无寄存器/pipe
// 编码约束（本模块强制）:
//   1. 仅时序信号使用 always 语句（always @(posedge clk)）；
//   2. 组合逻辑只允许 assign 赋值；
//   3. for 循环仅用于 generate 实例化。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   <日期>  <修订人>  <变更概述>
//  2026-09-24  OpenHappy Team  LL-20260924-CRFIFOEXP：读路径改等值门控 OR 文本展开
//                          （rd_data={DATA_WIDTH{rd_addr==K}}&mem[K]|…，DEPTH 四档）；
//  2026-09-24  OpenHappy Team  LL-20260924-CRIFVO v1.6：mem 去复位清零（删复位 for 清零），
//                          rst_n 保留 port（unused）。
//  2026-09-25  OpenHappy Team  读 mux 等值门控 OR 展开 → 平衡二进制 2:1 树（m1_*…m{N}_*，16/32/64/128→4/5/6/7 级），同函数 bit-exact，无寄存器/pipe，端口/参数/写路径不变
//  2026-09-29  OpenHappy Team  LL-TIMING-N1：写口 wr_addr（PTR_W）→ wr_row_sel（DEPTH 位
//                          one-hot，父 credit_fifo 预计算 (wr_addr==K) 等值）——逐行写
//                          ICG 使能 = wr_en & wr_row_sel[K] 单 AND 级（原 4bit 等值链
//                          在 bank 内、在 push→ICG 关键链上）；one-hot 恰一位命中，
//                          与 indexed 写位级等价。读口/参数/端口位宽（rd_addr/rd_data/
//                          wr_en/wr_data）不变。
// =============================================================================
`timescale 1ns/1ps

`include "link_layer_defines.vh"

module credit_fifo_bank #(
    parameter DATA_WIDTH = 1024,          // 单 bank 位宽
    parameter DEPTH      = 64,            // 每 bank 深度（默认 64；实际档位由
                                          //   CREDIT_FIFO_DEPTH_* 宏决定，与 credit_fifo
                                          //   激活宏保持一致——当前激活 CREDIT_FIFO_DEPTH_16）
    parameter PTR_W      = $clog2(DEPTH)  // 地址位宽
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // 写接口（credit_fifo 生成 one-hot 行选与使能；LL-TIMING-N1：wr_addr → wr_row_sel——
    // DEPTH 位 one-hot = (wr_addr==K) 等值预计算，逐行 ICG 使能 = wr_en & row_sel 单 AND）
    input  wire                    wr_en,
    input  wire [DEPTH-1:0]        wr_row_sel,
    input  wire [DATA_WIDTH-1:0]   wr_data,

    // 读接口（等值门控 OR 展开读 mux；有效/空由父模块按占用判定）
    input  wire [PTR_W-1:0]        rd_addr,
    output wire [DATA_WIDTH-1:0]   rd_data
);

    // 纯存储阵列：mem 无复位（v1.6 去清零，见头注性能注）。X 泄漏由父模块 ren 仅对
    // empty=0 bank 拉高保证（未写单元不进读出路径）；rst_n 为本端口保留（unused）。
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    genvar gk;
    generate
        for (gk = 0; gk < DEPTH; gk = gk + 1) begin : g_wr_row
            always @(posedge clk) begin
                if (wr_en & wr_row_sel[gk])
                    mem[gk] <= wr_data;
            end
        end
    endgenerate

    // 读路径：CREDIT_FIFO_DEPTH_* 宏控平衡二进制 2:1 MUX 树文本展开读 mux（2026-09-25 由
    // LL-20260924-CRFIFOEXP 等值门控 OR 展开改 2:1 树，16/32/64/128 → 4/5/6/7 级）；
    // 每级选择位 = rd_addr[级-1] 复制到向量宽 {DATA_WIDTH{sel}}（W116），末级直接驱动 rd_data。
    `ifdef CREDIT_FIFO_DEPTH_16
        // DEPTH16：平衡二进制 2:1 树 4 级（每级 sel = rd_addr[级-1]，末级 sel = rd_addr[3] → rd_data）
        wire [DATA_WIDTH-1:0] m1_0, m1_1, m1_2, m1_3, m1_4, m1_5, m1_6, m1_7;
        wire [DATA_WIDTH-1:0] m2_0, m2_1, m2_2, m2_3;
        wire [DATA_WIDTH-1:0] m3_0, m3_1;
        // L1（sel = rd_addr[0]，8 路）
        assign m1_0 = ({DATA_WIDTH{rd_addr[0]}} & mem[1]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[0]);
        assign m1_1 = ({DATA_WIDTH{rd_addr[0]}} & mem[3]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[2]);
        assign m1_2 = ({DATA_WIDTH{rd_addr[0]}} & mem[5]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[4]);
        assign m1_3 = ({DATA_WIDTH{rd_addr[0]}} & mem[7]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[6]);
        assign m1_4 = ({DATA_WIDTH{rd_addr[0]}} & mem[9]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[8]);
        assign m1_5 = ({DATA_WIDTH{rd_addr[0]}} & mem[11]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[10]);
        assign m1_6 = ({DATA_WIDTH{rd_addr[0]}} & mem[13]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[12]);
        assign m1_7 = ({DATA_WIDTH{rd_addr[0]}} & mem[15]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[14]);
        // L2（sel = rd_addr[1]，4 路）
        assign m2_0 = ({DATA_WIDTH{rd_addr[1]}} & m1_1) | ({DATA_WIDTH{~rd_addr[1]}} & m1_0);
        assign m2_1 = ({DATA_WIDTH{rd_addr[1]}} & m1_3) | ({DATA_WIDTH{~rd_addr[1]}} & m1_2);
        assign m2_2 = ({DATA_WIDTH{rd_addr[1]}} & m1_5) | ({DATA_WIDTH{~rd_addr[1]}} & m1_4);
        assign m2_3 = ({DATA_WIDTH{rd_addr[1]}} & m1_7) | ({DATA_WIDTH{~rd_addr[1]}} & m1_6);
        // L3（sel = rd_addr[2]，2 路）
        assign m3_0 = ({DATA_WIDTH{rd_addr[2]}} & m2_1) | ({DATA_WIDTH{~rd_addr[2]}} & m2_0);
        assign m3_1 = ({DATA_WIDTH{rd_addr[2]}} & m2_3) | ({DATA_WIDTH{~rd_addr[2]}} & m2_2);
        // L4（sel = rd_addr[3]，1 路）
        assign rd_data = ({DATA_WIDTH{rd_addr[3]}} & m3_1) | ({DATA_WIDTH{~rd_addr[3]}} & m3_0);
    `elsif CREDIT_FIFO_DEPTH_32
        // DEPTH32：平衡二进制 2:1 树 5 级（每级 sel = rd_addr[级-1]，末级 sel = rd_addr[4] → rd_data）
        wire [DATA_WIDTH-1:0] m1_0, m1_1, m1_2, m1_3, m1_4, m1_5, m1_6, m1_7;
        wire [DATA_WIDTH-1:0] m1_8, m1_9, m1_10, m1_11, m1_12, m1_13, m1_14, m1_15;
        wire [DATA_WIDTH-1:0] m2_0, m2_1, m2_2, m2_3, m2_4, m2_5, m2_6, m2_7;
        wire [DATA_WIDTH-1:0] m3_0, m3_1, m3_2, m3_3;
        wire [DATA_WIDTH-1:0] m4_0, m4_1;
        // L1（sel = rd_addr[0]，16 路）
        assign m1_0  = ({DATA_WIDTH{rd_addr[0]}} & mem[1]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[0]);
        assign m1_1  = ({DATA_WIDTH{rd_addr[0]}} & mem[3]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[2]);
        assign m1_2  = ({DATA_WIDTH{rd_addr[0]}} & mem[5]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[4]);
        assign m1_3  = ({DATA_WIDTH{rd_addr[0]}} & mem[7]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[6]);
        assign m1_4  = ({DATA_WIDTH{rd_addr[0]}} & mem[9]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[8]);
        assign m1_5  = ({DATA_WIDTH{rd_addr[0]}} & mem[11]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[10]);
        assign m1_6  = ({DATA_WIDTH{rd_addr[0]}} & mem[13]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[12]);
        assign m1_7  = ({DATA_WIDTH{rd_addr[0]}} & mem[15]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[14]);
        assign m1_8  = ({DATA_WIDTH{rd_addr[0]}} & mem[17]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[16]);
        assign m1_9  = ({DATA_WIDTH{rd_addr[0]}} & mem[19]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[18]);
        assign m1_10 = ({DATA_WIDTH{rd_addr[0]}} & mem[21]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[20]);
        assign m1_11 = ({DATA_WIDTH{rd_addr[0]}} & mem[23]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[22]);
        assign m1_12 = ({DATA_WIDTH{rd_addr[0]}} & mem[25]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[24]);
        assign m1_13 = ({DATA_WIDTH{rd_addr[0]}} & mem[27]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[26]);
        assign m1_14 = ({DATA_WIDTH{rd_addr[0]}} & mem[29]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[28]);
        assign m1_15 = ({DATA_WIDTH{rd_addr[0]}} & mem[31]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[30]);
        // L2（sel = rd_addr[1]，8 路）
        assign m2_0 = ({DATA_WIDTH{rd_addr[1]}} & m1_1) | ({DATA_WIDTH{~rd_addr[1]}} & m1_0);
        assign m2_1 = ({DATA_WIDTH{rd_addr[1]}} & m1_3) | ({DATA_WIDTH{~rd_addr[1]}} & m1_2);
        assign m2_2 = ({DATA_WIDTH{rd_addr[1]}} & m1_5) | ({DATA_WIDTH{~rd_addr[1]}} & m1_4);
        assign m2_3 = ({DATA_WIDTH{rd_addr[1]}} & m1_7) | ({DATA_WIDTH{~rd_addr[1]}} & m1_6);
        assign m2_4 = ({DATA_WIDTH{rd_addr[1]}} & m1_9) | ({DATA_WIDTH{~rd_addr[1]}} & m1_8);
        assign m2_5 = ({DATA_WIDTH{rd_addr[1]}} & m1_11) | ({DATA_WIDTH{~rd_addr[1]}} & m1_10);
        assign m2_6 = ({DATA_WIDTH{rd_addr[1]}} & m1_13) | ({DATA_WIDTH{~rd_addr[1]}} & m1_12);
        assign m2_7 = ({DATA_WIDTH{rd_addr[1]}} & m1_15) | ({DATA_WIDTH{~rd_addr[1]}} & m1_14);
        // L3（sel = rd_addr[2]，4 路）
        assign m3_0 = ({DATA_WIDTH{rd_addr[2]}} & m2_1) | ({DATA_WIDTH{~rd_addr[2]}} & m2_0);
        assign m3_1 = ({DATA_WIDTH{rd_addr[2]}} & m2_3) | ({DATA_WIDTH{~rd_addr[2]}} & m2_2);
        assign m3_2 = ({DATA_WIDTH{rd_addr[2]}} & m2_5) | ({DATA_WIDTH{~rd_addr[2]}} & m2_4);
        assign m3_3 = ({DATA_WIDTH{rd_addr[2]}} & m2_7) | ({DATA_WIDTH{~rd_addr[2]}} & m2_6);
        // L4（sel = rd_addr[3]，2 路）
        assign m4_0 = ({DATA_WIDTH{rd_addr[3]}} & m3_1) | ({DATA_WIDTH{~rd_addr[3]}} & m3_0);
        assign m4_1 = ({DATA_WIDTH{rd_addr[3]}} & m3_3) | ({DATA_WIDTH{~rd_addr[3]}} & m3_2);
        // L5（sel = rd_addr[4]，1 路）
        assign rd_data = ({DATA_WIDTH{rd_addr[4]}} & m4_1) | ({DATA_WIDTH{~rd_addr[4]}} & m4_0);
    `elsif CREDIT_FIFO_DEPTH_64
        // DEPTH64：平衡二进制 2:1 树 6 级（每级 sel = rd_addr[级-1]，末级 sel = rd_addr[5] → rd_data）
        wire [DATA_WIDTH-1:0] m1_0, m1_1, m1_2, m1_3, m1_4, m1_5, m1_6, m1_7;
        wire [DATA_WIDTH-1:0] m1_8, m1_9, m1_10, m1_11, m1_12, m1_13, m1_14, m1_15;
        wire [DATA_WIDTH-1:0] m1_16, m1_17, m1_18, m1_19, m1_20, m1_21, m1_22, m1_23;
        wire [DATA_WIDTH-1:0] m1_24, m1_25, m1_26, m1_27, m1_28, m1_29, m1_30, m1_31;
        wire [DATA_WIDTH-1:0] m2_0, m2_1, m2_2, m2_3, m2_4, m2_5, m2_6, m2_7;
        wire [DATA_WIDTH-1:0] m2_8, m2_9, m2_10, m2_11, m2_12, m2_13, m2_14, m2_15;
        wire [DATA_WIDTH-1:0] m3_0, m3_1, m3_2, m3_3, m3_4, m3_5, m3_6, m3_7;
        wire [DATA_WIDTH-1:0] m4_0, m4_1, m4_2, m4_3;
        wire [DATA_WIDTH-1:0] m5_0, m5_1;
        // L1（sel = rd_addr[0]，32 路）
        assign m1_0  = ({DATA_WIDTH{rd_addr[0]}} & mem[1]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[0]);
        assign m1_1  = ({DATA_WIDTH{rd_addr[0]}} & mem[3]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[2]);
        assign m1_2  = ({DATA_WIDTH{rd_addr[0]}} & mem[5]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[4]);
        assign m1_3  = ({DATA_WIDTH{rd_addr[0]}} & mem[7]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[6]);
        assign m1_4  = ({DATA_WIDTH{rd_addr[0]}} & mem[9]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[8]);
        assign m1_5  = ({DATA_WIDTH{rd_addr[0]}} & mem[11]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[10]);
        assign m1_6  = ({DATA_WIDTH{rd_addr[0]}} & mem[13]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[12]);
        assign m1_7  = ({DATA_WIDTH{rd_addr[0]}} & mem[15]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[14]);
        assign m1_8  = ({DATA_WIDTH{rd_addr[0]}} & mem[17]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[16]);
        assign m1_9  = ({DATA_WIDTH{rd_addr[0]}} & mem[19]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[18]);
        assign m1_10 = ({DATA_WIDTH{rd_addr[0]}} & mem[21]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[20]);
        assign m1_11 = ({DATA_WIDTH{rd_addr[0]}} & mem[23]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[22]);
        assign m1_12 = ({DATA_WIDTH{rd_addr[0]}} & mem[25]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[24]);
        assign m1_13 = ({DATA_WIDTH{rd_addr[0]}} & mem[27]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[26]);
        assign m1_14 = ({DATA_WIDTH{rd_addr[0]}} & mem[29]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[28]);
        assign m1_15 = ({DATA_WIDTH{rd_addr[0]}} & mem[31]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[30]);
        assign m1_16 = ({DATA_WIDTH{rd_addr[0]}} & mem[33]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[32]);
        assign m1_17 = ({DATA_WIDTH{rd_addr[0]}} & mem[35]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[34]);
        assign m1_18 = ({DATA_WIDTH{rd_addr[0]}} & mem[37]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[36]);
        assign m1_19 = ({DATA_WIDTH{rd_addr[0]}} & mem[39]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[38]);
        assign m1_20 = ({DATA_WIDTH{rd_addr[0]}} & mem[41]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[40]);
        assign m1_21 = ({DATA_WIDTH{rd_addr[0]}} & mem[43]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[42]);
        assign m1_22 = ({DATA_WIDTH{rd_addr[0]}} & mem[45]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[44]);
        assign m1_23 = ({DATA_WIDTH{rd_addr[0]}} & mem[47]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[46]);
        assign m1_24 = ({DATA_WIDTH{rd_addr[0]}} & mem[49]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[48]);
        assign m1_25 = ({DATA_WIDTH{rd_addr[0]}} & mem[51]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[50]);
        assign m1_26 = ({DATA_WIDTH{rd_addr[0]}} & mem[53]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[52]);
        assign m1_27 = ({DATA_WIDTH{rd_addr[0]}} & mem[55]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[54]);
        assign m1_28 = ({DATA_WIDTH{rd_addr[0]}} & mem[57]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[56]);
        assign m1_29 = ({DATA_WIDTH{rd_addr[0]}} & mem[59]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[58]);
        assign m1_30 = ({DATA_WIDTH{rd_addr[0]}} & mem[61]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[60]);
        assign m1_31 = ({DATA_WIDTH{rd_addr[0]}} & mem[63]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[62]);
        // L2（sel = rd_addr[1]，16 路）
        assign m2_0  = ({DATA_WIDTH{rd_addr[1]}} & m1_1) | ({DATA_WIDTH{~rd_addr[1]}} & m1_0);
        assign m2_1  = ({DATA_WIDTH{rd_addr[1]}} & m1_3) | ({DATA_WIDTH{~rd_addr[1]}} & m1_2);
        assign m2_2  = ({DATA_WIDTH{rd_addr[1]}} & m1_5) | ({DATA_WIDTH{~rd_addr[1]}} & m1_4);
        assign m2_3  = ({DATA_WIDTH{rd_addr[1]}} & m1_7) | ({DATA_WIDTH{~rd_addr[1]}} & m1_6);
        assign m2_4  = ({DATA_WIDTH{rd_addr[1]}} & m1_9) | ({DATA_WIDTH{~rd_addr[1]}} & m1_8);
        assign m2_5  = ({DATA_WIDTH{rd_addr[1]}} & m1_11) | ({DATA_WIDTH{~rd_addr[1]}} & m1_10);
        assign m2_6  = ({DATA_WIDTH{rd_addr[1]}} & m1_13) | ({DATA_WIDTH{~rd_addr[1]}} & m1_12);
        assign m2_7  = ({DATA_WIDTH{rd_addr[1]}} & m1_15) | ({DATA_WIDTH{~rd_addr[1]}} & m1_14);
        assign m2_8  = ({DATA_WIDTH{rd_addr[1]}} & m1_17) | ({DATA_WIDTH{~rd_addr[1]}} & m1_16);
        assign m2_9  = ({DATA_WIDTH{rd_addr[1]}} & m1_19) | ({DATA_WIDTH{~rd_addr[1]}} & m1_18);
        assign m2_10 = ({DATA_WIDTH{rd_addr[1]}} & m1_21) | ({DATA_WIDTH{~rd_addr[1]}} & m1_20);
        assign m2_11 = ({DATA_WIDTH{rd_addr[1]}} & m1_23) | ({DATA_WIDTH{~rd_addr[1]}} & m1_22);
        assign m2_12 = ({DATA_WIDTH{rd_addr[1]}} & m1_25) | ({DATA_WIDTH{~rd_addr[1]}} & m1_24);
        assign m2_13 = ({DATA_WIDTH{rd_addr[1]}} & m1_27) | ({DATA_WIDTH{~rd_addr[1]}} & m1_26);
        assign m2_14 = ({DATA_WIDTH{rd_addr[1]}} & m1_29) | ({DATA_WIDTH{~rd_addr[1]}} & m1_28);
        assign m2_15 = ({DATA_WIDTH{rd_addr[1]}} & m1_31) | ({DATA_WIDTH{~rd_addr[1]}} & m1_30);
        // L3（sel = rd_addr[2]，8 路）
        assign m3_0 = ({DATA_WIDTH{rd_addr[2]}} & m2_1) | ({DATA_WIDTH{~rd_addr[2]}} & m2_0);
        assign m3_1 = ({DATA_WIDTH{rd_addr[2]}} & m2_3) | ({DATA_WIDTH{~rd_addr[2]}} & m2_2);
        assign m3_2 = ({DATA_WIDTH{rd_addr[2]}} & m2_5) | ({DATA_WIDTH{~rd_addr[2]}} & m2_4);
        assign m3_3 = ({DATA_WIDTH{rd_addr[2]}} & m2_7) | ({DATA_WIDTH{~rd_addr[2]}} & m2_6);
        assign m3_4 = ({DATA_WIDTH{rd_addr[2]}} & m2_9) | ({DATA_WIDTH{~rd_addr[2]}} & m2_8);
        assign m3_5 = ({DATA_WIDTH{rd_addr[2]}} & m2_11) | ({DATA_WIDTH{~rd_addr[2]}} & m2_10);
        assign m3_6 = ({DATA_WIDTH{rd_addr[2]}} & m2_13) | ({DATA_WIDTH{~rd_addr[2]}} & m2_12);
        assign m3_7 = ({DATA_WIDTH{rd_addr[2]}} & m2_15) | ({DATA_WIDTH{~rd_addr[2]}} & m2_14);
        // L4（sel = rd_addr[3]，4 路）
        assign m4_0 = ({DATA_WIDTH{rd_addr[3]}} & m3_1) | ({DATA_WIDTH{~rd_addr[3]}} & m3_0);
        assign m4_1 = ({DATA_WIDTH{rd_addr[3]}} & m3_3) | ({DATA_WIDTH{~rd_addr[3]}} & m3_2);
        assign m4_2 = ({DATA_WIDTH{rd_addr[3]}} & m3_5) | ({DATA_WIDTH{~rd_addr[3]}} & m3_4);
        assign m4_3 = ({DATA_WIDTH{rd_addr[3]}} & m3_7) | ({DATA_WIDTH{~rd_addr[3]}} & m3_6);
        // L5（sel = rd_addr[4]，2 路）
        assign m5_0 = ({DATA_WIDTH{rd_addr[4]}} & m4_1) | ({DATA_WIDTH{~rd_addr[4]}} & m4_0);
        assign m5_1 = ({DATA_WIDTH{rd_addr[4]}} & m4_3) | ({DATA_WIDTH{~rd_addr[4]}} & m4_2);
        // L6（sel = rd_addr[5]，1 路）
        assign rd_data = ({DATA_WIDTH{rd_addr[5]}} & m5_1) | ({DATA_WIDTH{~rd_addr[5]}} & m5_0);
    `else
        // DEPTH128：平衡二进制 2:1 树 7 级（每级 sel = rd_addr[级-1]，末级 sel = rd_addr[6] → rd_data）
        wire [DATA_WIDTH-1:0] m1_0, m1_1, m1_2, m1_3, m1_4, m1_5, m1_6, m1_7;
        wire [DATA_WIDTH-1:0] m1_8, m1_9, m1_10, m1_11, m1_12, m1_13, m1_14, m1_15;
        wire [DATA_WIDTH-1:0] m1_16, m1_17, m1_18, m1_19, m1_20, m1_21, m1_22, m1_23;
        wire [DATA_WIDTH-1:0] m1_24, m1_25, m1_26, m1_27, m1_28, m1_29, m1_30, m1_31;
        wire [DATA_WIDTH-1:0] m1_32, m1_33, m1_34, m1_35, m1_36, m1_37, m1_38, m1_39;
        wire [DATA_WIDTH-1:0] m1_40, m1_41, m1_42, m1_43, m1_44, m1_45, m1_46, m1_47;
        wire [DATA_WIDTH-1:0] m1_48, m1_49, m1_50, m1_51, m1_52, m1_53, m1_54, m1_55;
        wire [DATA_WIDTH-1:0] m1_56, m1_57, m1_58, m1_59, m1_60, m1_61, m1_62, m1_63;
        wire [DATA_WIDTH-1:0] m2_0, m2_1, m2_2, m2_3, m2_4, m2_5, m2_6, m2_7;
        wire [DATA_WIDTH-1:0] m2_8, m2_9, m2_10, m2_11, m2_12, m2_13, m2_14, m2_15;
        wire [DATA_WIDTH-1:0] m2_16, m2_17, m2_18, m2_19, m2_20, m2_21, m2_22, m2_23;
        wire [DATA_WIDTH-1:0] m2_24, m2_25, m2_26, m2_27, m2_28, m2_29, m2_30, m2_31;
        wire [DATA_WIDTH-1:0] m3_0, m3_1, m3_2, m3_3, m3_4, m3_5, m3_6, m3_7;
        wire [DATA_WIDTH-1:0] m3_8, m3_9, m3_10, m3_11, m3_12, m3_13, m3_14, m3_15;
        wire [DATA_WIDTH-1:0] m4_0, m4_1, m4_2, m4_3, m4_4, m4_5, m4_6, m4_7;
        wire [DATA_WIDTH-1:0] m5_0, m5_1, m5_2, m5_3;
        wire [DATA_WIDTH-1:0] m6_0, m6_1;
        // L1（sel = rd_addr[0]，64 路）
        assign m1_0  = ({DATA_WIDTH{rd_addr[0]}} & mem[1]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[0]);
        assign m1_1  = ({DATA_WIDTH{rd_addr[0]}} & mem[3]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[2]);
        assign m1_2  = ({DATA_WIDTH{rd_addr[0]}} & mem[5]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[4]);
        assign m1_3  = ({DATA_WIDTH{rd_addr[0]}} & mem[7]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[6]);
        assign m1_4  = ({DATA_WIDTH{rd_addr[0]}} & mem[9]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[8]);
        assign m1_5  = ({DATA_WIDTH{rd_addr[0]}} & mem[11]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[10]);
        assign m1_6  = ({DATA_WIDTH{rd_addr[0]}} & mem[13]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[12]);
        assign m1_7  = ({DATA_WIDTH{rd_addr[0]}} & mem[15]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[14]);
        assign m1_8  = ({DATA_WIDTH{rd_addr[0]}} & mem[17]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[16]);
        assign m1_9  = ({DATA_WIDTH{rd_addr[0]}} & mem[19]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[18]);
        assign m1_10 = ({DATA_WIDTH{rd_addr[0]}} & mem[21]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[20]);
        assign m1_11 = ({DATA_WIDTH{rd_addr[0]}} & mem[23]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[22]);
        assign m1_12 = ({DATA_WIDTH{rd_addr[0]}} & mem[25]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[24]);
        assign m1_13 = ({DATA_WIDTH{rd_addr[0]}} & mem[27]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[26]);
        assign m1_14 = ({DATA_WIDTH{rd_addr[0]}} & mem[29]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[28]);
        assign m1_15 = ({DATA_WIDTH{rd_addr[0]}} & mem[31]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[30]);
        assign m1_16 = ({DATA_WIDTH{rd_addr[0]}} & mem[33]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[32]);
        assign m1_17 = ({DATA_WIDTH{rd_addr[0]}} & mem[35]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[34]);
        assign m1_18 = ({DATA_WIDTH{rd_addr[0]}} & mem[37]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[36]);
        assign m1_19 = ({DATA_WIDTH{rd_addr[0]}} & mem[39]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[38]);
        assign m1_20 = ({DATA_WIDTH{rd_addr[0]}} & mem[41]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[40]);
        assign m1_21 = ({DATA_WIDTH{rd_addr[0]}} & mem[43]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[42]);
        assign m1_22 = ({DATA_WIDTH{rd_addr[0]}} & mem[45]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[44]);
        assign m1_23 = ({DATA_WIDTH{rd_addr[0]}} & mem[47]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[46]);
        assign m1_24 = ({DATA_WIDTH{rd_addr[0]}} & mem[49]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[48]);
        assign m1_25 = ({DATA_WIDTH{rd_addr[0]}} & mem[51]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[50]);
        assign m1_26 = ({DATA_WIDTH{rd_addr[0]}} & mem[53]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[52]);
        assign m1_27 = ({DATA_WIDTH{rd_addr[0]}} & mem[55]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[54]);
        assign m1_28 = ({DATA_WIDTH{rd_addr[0]}} & mem[57]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[56]);
        assign m1_29 = ({DATA_WIDTH{rd_addr[0]}} & mem[59]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[58]);
        assign m1_30 = ({DATA_WIDTH{rd_addr[0]}} & mem[61]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[60]);
        assign m1_31 = ({DATA_WIDTH{rd_addr[0]}} & mem[63]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[62]);
        assign m1_32 = ({DATA_WIDTH{rd_addr[0]}} & mem[65]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[64]);
        assign m1_33 = ({DATA_WIDTH{rd_addr[0]}} & mem[67]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[66]);
        assign m1_34 = ({DATA_WIDTH{rd_addr[0]}} & mem[69]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[68]);
        assign m1_35 = ({DATA_WIDTH{rd_addr[0]}} & mem[71]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[70]);
        assign m1_36 = ({DATA_WIDTH{rd_addr[0]}} & mem[73]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[72]);
        assign m1_37 = ({DATA_WIDTH{rd_addr[0]}} & mem[75]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[74]);
        assign m1_38 = ({DATA_WIDTH{rd_addr[0]}} & mem[77]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[76]);
        assign m1_39 = ({DATA_WIDTH{rd_addr[0]}} & mem[79]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[78]);
        assign m1_40 = ({DATA_WIDTH{rd_addr[0]}} & mem[81]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[80]);
        assign m1_41 = ({DATA_WIDTH{rd_addr[0]}} & mem[83]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[82]);
        assign m1_42 = ({DATA_WIDTH{rd_addr[0]}} & mem[85]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[84]);
        assign m1_43 = ({DATA_WIDTH{rd_addr[0]}} & mem[87]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[86]);
        assign m1_44 = ({DATA_WIDTH{rd_addr[0]}} & mem[89]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[88]);
        assign m1_45 = ({DATA_WIDTH{rd_addr[0]}} & mem[91]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[90]);
        assign m1_46 = ({DATA_WIDTH{rd_addr[0]}} & mem[93]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[92]);
        assign m1_47 = ({DATA_WIDTH{rd_addr[0]}} & mem[95]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[94]);
        assign m1_48 = ({DATA_WIDTH{rd_addr[0]}} & mem[97]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[96]);
        assign m1_49 = ({DATA_WIDTH{rd_addr[0]}} & mem[99]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[98]);
        assign m1_50 = ({DATA_WIDTH{rd_addr[0]}} & mem[101]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[100]);
        assign m1_51 = ({DATA_WIDTH{rd_addr[0]}} & mem[103]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[102]);
        assign m1_52 = ({DATA_WIDTH{rd_addr[0]}} & mem[105]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[104]);
        assign m1_53 = ({DATA_WIDTH{rd_addr[0]}} & mem[107]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[106]);
        assign m1_54 = ({DATA_WIDTH{rd_addr[0]}} & mem[109]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[108]);
        assign m1_55 = ({DATA_WIDTH{rd_addr[0]}} & mem[111]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[110]);
        assign m1_56 = ({DATA_WIDTH{rd_addr[0]}} & mem[113]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[112]);
        assign m1_57 = ({DATA_WIDTH{rd_addr[0]}} & mem[115]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[114]);
        assign m1_58 = ({DATA_WIDTH{rd_addr[0]}} & mem[117]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[116]);
        assign m1_59 = ({DATA_WIDTH{rd_addr[0]}} & mem[119]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[118]);
        assign m1_60 = ({DATA_WIDTH{rd_addr[0]}} & mem[121]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[120]);
        assign m1_61 = ({DATA_WIDTH{rd_addr[0]}} & mem[123]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[122]);
        assign m1_62 = ({DATA_WIDTH{rd_addr[0]}} & mem[125]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[124]);
        assign m1_63 = ({DATA_WIDTH{rd_addr[0]}} & mem[127]) | ({DATA_WIDTH{~rd_addr[0]}} & mem[126]);
        // L2（sel = rd_addr[1]，32 路）
        assign m2_0  = ({DATA_WIDTH{rd_addr[1]}} & m1_1) | ({DATA_WIDTH{~rd_addr[1]}} & m1_0);
        assign m2_1  = ({DATA_WIDTH{rd_addr[1]}} & m1_3) | ({DATA_WIDTH{~rd_addr[1]}} & m1_2);
        assign m2_2  = ({DATA_WIDTH{rd_addr[1]}} & m1_5) | ({DATA_WIDTH{~rd_addr[1]}} & m1_4);
        assign m2_3  = ({DATA_WIDTH{rd_addr[1]}} & m1_7) | ({DATA_WIDTH{~rd_addr[1]}} & m1_6);
        assign m2_4  = ({DATA_WIDTH{rd_addr[1]}} & m1_9) | ({DATA_WIDTH{~rd_addr[1]}} & m1_8);
        assign m2_5  = ({DATA_WIDTH{rd_addr[1]}} & m1_11) | ({DATA_WIDTH{~rd_addr[1]}} & m1_10);
        assign m2_6  = ({DATA_WIDTH{rd_addr[1]}} & m1_13) | ({DATA_WIDTH{~rd_addr[1]}} & m1_12);
        assign m2_7  = ({DATA_WIDTH{rd_addr[1]}} & m1_15) | ({DATA_WIDTH{~rd_addr[1]}} & m1_14);
        assign m2_8  = ({DATA_WIDTH{rd_addr[1]}} & m1_17) | ({DATA_WIDTH{~rd_addr[1]}} & m1_16);
        assign m2_9  = ({DATA_WIDTH{rd_addr[1]}} & m1_19) | ({DATA_WIDTH{~rd_addr[1]}} & m1_18);
        assign m2_10 = ({DATA_WIDTH{rd_addr[1]}} & m1_21) | ({DATA_WIDTH{~rd_addr[1]}} & m1_20);
        assign m2_11 = ({DATA_WIDTH{rd_addr[1]}} & m1_23) | ({DATA_WIDTH{~rd_addr[1]}} & m1_22);
        assign m2_12 = ({DATA_WIDTH{rd_addr[1]}} & m1_25) | ({DATA_WIDTH{~rd_addr[1]}} & m1_24);
        assign m2_13 = ({DATA_WIDTH{rd_addr[1]}} & m1_27) | ({DATA_WIDTH{~rd_addr[1]}} & m1_26);
        assign m2_14 = ({DATA_WIDTH{rd_addr[1]}} & m1_29) | ({DATA_WIDTH{~rd_addr[1]}} & m1_28);
        assign m2_15 = ({DATA_WIDTH{rd_addr[1]}} & m1_31) | ({DATA_WIDTH{~rd_addr[1]}} & m1_30);
        assign m2_16 = ({DATA_WIDTH{rd_addr[1]}} & m1_33) | ({DATA_WIDTH{~rd_addr[1]}} & m1_32);
        assign m2_17 = ({DATA_WIDTH{rd_addr[1]}} & m1_35) | ({DATA_WIDTH{~rd_addr[1]}} & m1_34);
        assign m2_18 = ({DATA_WIDTH{rd_addr[1]}} & m1_37) | ({DATA_WIDTH{~rd_addr[1]}} & m1_36);
        assign m2_19 = ({DATA_WIDTH{rd_addr[1]}} & m1_39) | ({DATA_WIDTH{~rd_addr[1]}} & m1_38);
        assign m2_20 = ({DATA_WIDTH{rd_addr[1]}} & m1_41) | ({DATA_WIDTH{~rd_addr[1]}} & m1_40);
        assign m2_21 = ({DATA_WIDTH{rd_addr[1]}} & m1_43) | ({DATA_WIDTH{~rd_addr[1]}} & m1_42);
        assign m2_22 = ({DATA_WIDTH{rd_addr[1]}} & m1_45) | ({DATA_WIDTH{~rd_addr[1]}} & m1_44);
        assign m2_23 = ({DATA_WIDTH{rd_addr[1]}} & m1_47) | ({DATA_WIDTH{~rd_addr[1]}} & m1_46);
        assign m2_24 = ({DATA_WIDTH{rd_addr[1]}} & m1_49) | ({DATA_WIDTH{~rd_addr[1]}} & m1_48);
        assign m2_25 = ({DATA_WIDTH{rd_addr[1]}} & m1_51) | ({DATA_WIDTH{~rd_addr[1]}} & m1_50);
        assign m2_26 = ({DATA_WIDTH{rd_addr[1]}} & m1_53) | ({DATA_WIDTH{~rd_addr[1]}} & m1_52);
        assign m2_27 = ({DATA_WIDTH{rd_addr[1]}} & m1_55) | ({DATA_WIDTH{~rd_addr[1]}} & m1_54);
        assign m2_28 = ({DATA_WIDTH{rd_addr[1]}} & m1_57) | ({DATA_WIDTH{~rd_addr[1]}} & m1_56);
        assign m2_29 = ({DATA_WIDTH{rd_addr[1]}} & m1_59) | ({DATA_WIDTH{~rd_addr[1]}} & m1_58);
        assign m2_30 = ({DATA_WIDTH{rd_addr[1]}} & m1_61) | ({DATA_WIDTH{~rd_addr[1]}} & m1_60);
        assign m2_31 = ({DATA_WIDTH{rd_addr[1]}} & m1_63) | ({DATA_WIDTH{~rd_addr[1]}} & m1_62);
        // L3（sel = rd_addr[2]，16 路）
        assign m3_0  = ({DATA_WIDTH{rd_addr[2]}} & m2_1) | ({DATA_WIDTH{~rd_addr[2]}} & m2_0);
        assign m3_1  = ({DATA_WIDTH{rd_addr[2]}} & m2_3) | ({DATA_WIDTH{~rd_addr[2]}} & m2_2);
        assign m3_2  = ({DATA_WIDTH{rd_addr[2]}} & m2_5) | ({DATA_WIDTH{~rd_addr[2]}} & m2_4);
        assign m3_3  = ({DATA_WIDTH{rd_addr[2]}} & m2_7) | ({DATA_WIDTH{~rd_addr[2]}} & m2_6);
        assign m3_4  = ({DATA_WIDTH{rd_addr[2]}} & m2_9) | ({DATA_WIDTH{~rd_addr[2]}} & m2_8);
        assign m3_5  = ({DATA_WIDTH{rd_addr[2]}} & m2_11) | ({DATA_WIDTH{~rd_addr[2]}} & m2_10);
        assign m3_6  = ({DATA_WIDTH{rd_addr[2]}} & m2_13) | ({DATA_WIDTH{~rd_addr[2]}} & m2_12);
        assign m3_7  = ({DATA_WIDTH{rd_addr[2]}} & m2_15) | ({DATA_WIDTH{~rd_addr[2]}} & m2_14);
        assign m3_8  = ({DATA_WIDTH{rd_addr[2]}} & m2_17) | ({DATA_WIDTH{~rd_addr[2]}} & m2_16);
        assign m3_9  = ({DATA_WIDTH{rd_addr[2]}} & m2_19) | ({DATA_WIDTH{~rd_addr[2]}} & m2_18);
        assign m3_10 = ({DATA_WIDTH{rd_addr[2]}} & m2_21) | ({DATA_WIDTH{~rd_addr[2]}} & m2_20);
        assign m3_11 = ({DATA_WIDTH{rd_addr[2]}} & m2_23) | ({DATA_WIDTH{~rd_addr[2]}} & m2_22);
        assign m3_12 = ({DATA_WIDTH{rd_addr[2]}} & m2_25) | ({DATA_WIDTH{~rd_addr[2]}} & m2_24);
        assign m3_13 = ({DATA_WIDTH{rd_addr[2]}} & m2_27) | ({DATA_WIDTH{~rd_addr[2]}} & m2_26);
        assign m3_14 = ({DATA_WIDTH{rd_addr[2]}} & m2_29) | ({DATA_WIDTH{~rd_addr[2]}} & m2_28);
        assign m3_15 = ({DATA_WIDTH{rd_addr[2]}} & m2_31) | ({DATA_WIDTH{~rd_addr[2]}} & m2_30);
        // L4（sel = rd_addr[3]，8 路）
        assign m4_0 = ({DATA_WIDTH{rd_addr[3]}} & m3_1) | ({DATA_WIDTH{~rd_addr[3]}} & m3_0);
        assign m4_1 = ({DATA_WIDTH{rd_addr[3]}} & m3_3) | ({DATA_WIDTH{~rd_addr[3]}} & m3_2);
        assign m4_2 = ({DATA_WIDTH{rd_addr[3]}} & m3_5) | ({DATA_WIDTH{~rd_addr[3]}} & m3_4);
        assign m4_3 = ({DATA_WIDTH{rd_addr[3]}} & m3_7) | ({DATA_WIDTH{~rd_addr[3]}} & m3_6);
        assign m4_4 = ({DATA_WIDTH{rd_addr[3]}} & m3_9) | ({DATA_WIDTH{~rd_addr[3]}} & m3_8);
        assign m4_5 = ({DATA_WIDTH{rd_addr[3]}} & m3_11) | ({DATA_WIDTH{~rd_addr[3]}} & m3_10);
        assign m4_6 = ({DATA_WIDTH{rd_addr[3]}} & m3_13) | ({DATA_WIDTH{~rd_addr[3]}} & m3_12);
        assign m4_7 = ({DATA_WIDTH{rd_addr[3]}} & m3_15) | ({DATA_WIDTH{~rd_addr[3]}} & m3_14);
        // L5（sel = rd_addr[4]，4 路）
        assign m5_0 = ({DATA_WIDTH{rd_addr[4]}} & m4_1) | ({DATA_WIDTH{~rd_addr[4]}} & m4_0);
        assign m5_1 = ({DATA_WIDTH{rd_addr[4]}} & m4_3) | ({DATA_WIDTH{~rd_addr[4]}} & m4_2);
        assign m5_2 = ({DATA_WIDTH{rd_addr[4]}} & m4_5) | ({DATA_WIDTH{~rd_addr[4]}} & m4_4);
        assign m5_3 = ({DATA_WIDTH{rd_addr[4]}} & m4_7) | ({DATA_WIDTH{~rd_addr[4]}} & m4_6);
        // L6（sel = rd_addr[5]，2 路）
        assign m6_0 = ({DATA_WIDTH{rd_addr[5]}} & m5_1) | ({DATA_WIDTH{~rd_addr[5]}} & m5_0);
        assign m6_1 = ({DATA_WIDTH{rd_addr[5]}} & m5_3) | ({DATA_WIDTH{~rd_addr[5]}} & m5_2);
        // L7（sel = rd_addr[6]，1 路）
        assign rd_data = ({DATA_WIDTH{rd_addr[6]}} & m6_1) | ({DATA_WIDTH{~rd_addr[6]}} & m6_0);
    `endif

endmodule