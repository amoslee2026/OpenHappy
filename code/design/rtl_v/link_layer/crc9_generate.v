// =============================================================================
// module name：crc9_generate
// author: OpenHappy Team
// MAS：spec/link_layer/submods/link_tx/mas/crc9_generate_mas.md
// 功能概述: 兼容模式 TX 侧 32 B Flit 的 CRC9（Flit Tail CHK 字段，9 bit）生成单元。
//           对 Header（5 bit）+ Payload（256 bit）共 261 bit 数据按多项式
//           x^9 + x^5 + x^4 + x + 1（0x119，GB/T 46280.3-2025 §6.2.2；
//           Init=0x1FF，无输入/输出反转，XorOut=0x000）计算一次 CRC9。
//           本模块为纯组合逻辑，输入数据稳定后输出同拍有效，无寄存器延迟；
//           不做模式判断 / valid 门控 / 注错处理（均由父模块 link_tx 处理）。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   <日期>  <修订人>  <变更概述>
//  2026-08-06  OpenHappy Team  初始版本
//  2026-08-06  OpenHappy Team  BUG-001: CRC9 多项式修正为 9'h119（对齐国标），
//                          并由 function 迭代写法改为并行 XOR 矩阵展开逐 bit 赋值
//  2026-08-07  OpenHappy Team  删除 DATA_W 参数：展开矩阵按 261 bit 固化，参数化无意义
// =============================================================================

`timescale 1ns/1ps

module crc9_generate (
    // 时钟与复位（保留端口，纯组合模块内部不使用）
    input  wire               mb_clk,
    input  wire               mb_rst_n,

    // 数据输入（261 bit 固化：bit[4:0]=Header / bit[260:5]=Payload）
    input  wire [260:0]       crc9_data_i,

    // 数据输出
    output wire [8:0]         crc9_o
);

// =============================================================================
// 组合逻辑：并行 CRC9 XOR 矩阵展开（无 function / 无 for 循环，逐 bit 直接赋值）
// -----------------------------------------------------------------------------
// 由 Galois LFSR（poly=9'h119, Init=9'h1ff, 261 步迭代）符号展开生成，
// crc9_o[b] = 对应输入 bit 与常量项的 XOR；综合后即为 9 棵 XOR 树。
// =============================================================================

// crc9_o[8] : 130 项 XOR
assign crc9_o[8] =
        crc9_data_i[256] ^ crc9_data_i[253] ^ crc9_data_i[250] ^ crc9_data_i[249] ^ crc9_data_i[248] ^ crc9_data_i[245] ^
        crc9_data_i[241] ^ crc9_data_i[239] ^ crc9_data_i[238] ^ crc9_data_i[234] ^ crc9_data_i[233] ^ crc9_data_i[232] ^
        crc9_data_i[229] ^ crc9_data_i[228] ^ crc9_data_i[224] ^ crc9_data_i[223] ^ crc9_data_i[216] ^ crc9_data_i[214] ^
        crc9_data_i[213] ^ crc9_data_i[212] ^ crc9_data_i[211] ^ crc9_data_i[208] ^ crc9_data_i[206] ^ crc9_data_i[205] ^
        crc9_data_i[204] ^ crc9_data_i[203] ^ crc9_data_i[202] ^ crc9_data_i[200] ^ crc9_data_i[199] ^ crc9_data_i[197] ^
        crc9_data_i[195] ^ crc9_data_i[194] ^ crc9_data_i[189] ^ crc9_data_i[188] ^ crc9_data_i[187] ^ crc9_data_i[186] ^
        crc9_data_i[185] ^ crc9_data_i[176] ^ crc9_data_i[173] ^ crc9_data_i[172] ^ crc9_data_i[170] ^ crc9_data_i[166] ^
        crc9_data_i[165] ^ crc9_data_i[162] ^ crc9_data_i[159] ^ crc9_data_i[155] ^ crc9_data_i[154] ^ crc9_data_i[153] ^
        crc9_data_i[152] ^ crc9_data_i[151] ^ crc9_data_i[150] ^ crc9_data_i[147] ^ crc9_data_i[146] ^ crc9_data_i[144] ^
        crc9_data_i[142] ^ crc9_data_i[140] ^ crc9_data_i[138] ^ crc9_data_i[133] ^ crc9_data_i[132] ^ crc9_data_i[131] ^
        crc9_data_i[129] ^ crc9_data_i[127] ^ crc9_data_i[126] ^ crc9_data_i[124] ^ crc9_data_i[119] ^ crc9_data_i[117] ^
        crc9_data_i[114] ^ crc9_data_i[113] ^ crc9_data_i[110] ^ crc9_data_i[108] ^ crc9_data_i[107] ^ crc9_data_i[105] ^
        crc9_data_i[104] ^ crc9_data_i[103] ^ crc9_data_i[101] ^ crc9_data_i[100] ^ crc9_data_i[99] ^ crc9_data_i[97] ^
        crc9_data_i[95] ^ crc9_data_i[92] ^ crc9_data_i[91] ^ crc9_data_i[90] ^ crc9_data_i[88] ^ crc9_data_i[87] ^
        crc9_data_i[85] ^ crc9_data_i[84] ^ crc9_data_i[83] ^ crc9_data_i[82] ^ crc9_data_i[81] ^ crc9_data_i[80] ^
        crc9_data_i[78] ^ crc9_data_i[77] ^ crc9_data_i[76] ^ crc9_data_i[75] ^ crc9_data_i[70] ^ crc9_data_i[67] ^
        crc9_data_i[64] ^ crc9_data_i[63] ^ crc9_data_i[62] ^ crc9_data_i[59] ^ crc9_data_i[55] ^ crc9_data_i[53] ^
        crc9_data_i[52] ^ crc9_data_i[48] ^ crc9_data_i[47] ^ crc9_data_i[46] ^ crc9_data_i[43] ^ crc9_data_i[42] ^
        crc9_data_i[38] ^ crc9_data_i[37] ^ crc9_data_i[30] ^ crc9_data_i[28] ^ crc9_data_i[27] ^ crc9_data_i[26] ^
        crc9_data_i[25] ^ crc9_data_i[22] ^ crc9_data_i[20] ^ crc9_data_i[19] ^ crc9_data_i[18] ^ crc9_data_i[17] ^
        crc9_data_i[16] ^ crc9_data_i[14] ^ crc9_data_i[13] ^ crc9_data_i[11] ^ crc9_data_i[9] ^ crc9_data_i[8] ^
        crc9_data_i[3] ^ crc9_data_i[2] ^ crc9_data_i[1] ^ crc9_data_i[0];

// crc9_o[7] : 127 项 XOR
assign crc9_o[7] =
        crc9_data_i[260] ^ crc9_data_i[256] ^ crc9_data_i[255] ^ crc9_data_i[253] ^ crc9_data_i[252] ^ crc9_data_i[250] ^
        crc9_data_i[247] ^ crc9_data_i[245] ^ crc9_data_i[244] ^ crc9_data_i[241] ^ crc9_data_i[240] ^ crc9_data_i[239] ^
        crc9_data_i[237] ^ crc9_data_i[234] ^ crc9_data_i[231] ^ crc9_data_i[229] ^ crc9_data_i[227] ^ crc9_data_i[224] ^
        crc9_data_i[222] ^ crc9_data_i[216] ^ crc9_data_i[215] ^ crc9_data_i[214] ^ crc9_data_i[210] ^ crc9_data_i[208] ^
        crc9_data_i[207] ^ crc9_data_i[206] ^ crc9_data_i[201] ^ crc9_data_i[200] ^ crc9_data_i[198] ^ crc9_data_i[197] ^
        crc9_data_i[196] ^ crc9_data_i[195] ^ crc9_data_i[193] ^ crc9_data_i[189] ^ crc9_data_i[184] ^ crc9_data_i[176] ^
        crc9_data_i[175] ^ crc9_data_i[173] ^ crc9_data_i[171] ^ crc9_data_i[170] ^ crc9_data_i[169] ^ crc9_data_i[166] ^
        crc9_data_i[164] ^ crc9_data_i[162] ^ crc9_data_i[161] ^ crc9_data_i[159] ^ crc9_data_i[158] ^ crc9_data_i[155] ^
        crc9_data_i[149] ^ crc9_data_i[147] ^ crc9_data_i[145] ^ crc9_data_i[144] ^ crc9_data_i[143] ^ crc9_data_i[142] ^
        crc9_data_i[141] ^ crc9_data_i[140] ^ crc9_data_i[139] ^ crc9_data_i[138] ^ crc9_data_i[137] ^ crc9_data_i[133] ^
        crc9_data_i[130] ^ crc9_data_i[129] ^ crc9_data_i[128] ^ crc9_data_i[127] ^ crc9_data_i[125] ^ crc9_data_i[124] ^
        crc9_data_i[123] ^ crc9_data_i[119] ^ crc9_data_i[118] ^ crc9_data_i[117] ^ crc9_data_i[116] ^ crc9_data_i[114] ^
        crc9_data_i[112] ^ crc9_data_i[110] ^ crc9_data_i[109] ^ crc9_data_i[108] ^ crc9_data_i[106] ^ crc9_data_i[105] ^
        crc9_data_i[102] ^ crc9_data_i[101] ^ crc9_data_i[98] ^ crc9_data_i[97] ^ crc9_data_i[96] ^ crc9_data_i[95] ^
        crc9_data_i[94] ^ crc9_data_i[92] ^ crc9_data_i[89] ^ crc9_data_i[88] ^ crc9_data_i[86] ^ crc9_data_i[85] ^
        crc9_data_i[79] ^ crc9_data_i[78] ^ crc9_data_i[74] ^ crc9_data_i[70] ^ crc9_data_i[69] ^ crc9_data_i[67] ^
        crc9_data_i[66] ^ crc9_data_i[64] ^ crc9_data_i[61] ^ crc9_data_i[59] ^ crc9_data_i[58] ^ crc9_data_i[55] ^
        crc9_data_i[54] ^ crc9_data_i[53] ^ crc9_data_i[51] ^ crc9_data_i[48] ^ crc9_data_i[45] ^ crc9_data_i[43] ^
        crc9_data_i[41] ^ crc9_data_i[38] ^ crc9_data_i[36] ^ crc9_data_i[30] ^ crc9_data_i[29] ^ crc9_data_i[28] ^
        crc9_data_i[24] ^ crc9_data_i[22] ^ crc9_data_i[21] ^ crc9_data_i[20] ^ crc9_data_i[15] ^ crc9_data_i[14] ^
        crc9_data_i[12] ^ crc9_data_i[11] ^ crc9_data_i[10] ^ crc9_data_i[9] ^ crc9_data_i[7] ^ crc9_data_i[3] ^
        1'b1;

// crc9_o[6] : 126 项 XOR
assign crc9_o[6] =
        crc9_data_i[259] ^ crc9_data_i[255] ^ crc9_data_i[254] ^ crc9_data_i[252] ^ crc9_data_i[251] ^ crc9_data_i[249] ^
        crc9_data_i[246] ^ crc9_data_i[244] ^ crc9_data_i[243] ^ crc9_data_i[240] ^ crc9_data_i[239] ^ crc9_data_i[238] ^
        crc9_data_i[236] ^ crc9_data_i[233] ^ crc9_data_i[230] ^ crc9_data_i[228] ^ crc9_data_i[226] ^ crc9_data_i[223] ^
        crc9_data_i[221] ^ crc9_data_i[215] ^ crc9_data_i[214] ^ crc9_data_i[213] ^ crc9_data_i[209] ^ crc9_data_i[207] ^
        crc9_data_i[206] ^ crc9_data_i[205] ^ crc9_data_i[200] ^ crc9_data_i[199] ^ crc9_data_i[197] ^ crc9_data_i[196] ^
        crc9_data_i[195] ^ crc9_data_i[194] ^ crc9_data_i[192] ^ crc9_data_i[188] ^ crc9_data_i[183] ^ crc9_data_i[175] ^
        crc9_data_i[174] ^ crc9_data_i[172] ^ crc9_data_i[170] ^ crc9_data_i[169] ^ crc9_data_i[168] ^ crc9_data_i[165] ^
        crc9_data_i[163] ^ crc9_data_i[161] ^ crc9_data_i[160] ^ crc9_data_i[158] ^ crc9_data_i[157] ^ crc9_data_i[154] ^
        crc9_data_i[148] ^ crc9_data_i[146] ^ crc9_data_i[144] ^ crc9_data_i[143] ^ crc9_data_i[142] ^ crc9_data_i[141] ^
        crc9_data_i[140] ^ crc9_data_i[139] ^ crc9_data_i[138] ^ crc9_data_i[137] ^ crc9_data_i[136] ^ crc9_data_i[132] ^
        crc9_data_i[129] ^ crc9_data_i[128] ^ crc9_data_i[127] ^ crc9_data_i[126] ^ crc9_data_i[124] ^ crc9_data_i[123] ^
        crc9_data_i[122] ^ crc9_data_i[118] ^ crc9_data_i[117] ^ crc9_data_i[116] ^ crc9_data_i[115] ^ crc9_data_i[113] ^
        crc9_data_i[111] ^ crc9_data_i[109] ^ crc9_data_i[108] ^ crc9_data_i[107] ^ crc9_data_i[105] ^ crc9_data_i[104] ^
        crc9_data_i[101] ^ crc9_data_i[100] ^ crc9_data_i[97] ^ crc9_data_i[96] ^ crc9_data_i[95] ^ crc9_data_i[94] ^
        crc9_data_i[93] ^ crc9_data_i[91] ^ crc9_data_i[88] ^ crc9_data_i[87] ^ crc9_data_i[85] ^ crc9_data_i[84] ^
        crc9_data_i[78] ^ crc9_data_i[77] ^ crc9_data_i[73] ^ crc9_data_i[69] ^ crc9_data_i[68] ^ crc9_data_i[66] ^
        crc9_data_i[65] ^ crc9_data_i[63] ^ crc9_data_i[60] ^ crc9_data_i[58] ^ crc9_data_i[57] ^ crc9_data_i[54] ^
        crc9_data_i[53] ^ crc9_data_i[52] ^ crc9_data_i[50] ^ crc9_data_i[47] ^ crc9_data_i[44] ^ crc9_data_i[42] ^
        crc9_data_i[40] ^ crc9_data_i[37] ^ crc9_data_i[35] ^ crc9_data_i[29] ^ crc9_data_i[28] ^ crc9_data_i[27] ^
        crc9_data_i[23] ^ crc9_data_i[21] ^ crc9_data_i[20] ^ crc9_data_i[19] ^ crc9_data_i[14] ^ crc9_data_i[13] ^
        crc9_data_i[11] ^ crc9_data_i[10] ^ crc9_data_i[9] ^ crc9_data_i[8] ^ crc9_data_i[6] ^ crc9_data_i[2];

// crc9_o[5] : 127 项 XOR
assign crc9_o[5] =
        crc9_data_i[258] ^ crc9_data_i[254] ^ crc9_data_i[253] ^ crc9_data_i[251] ^ crc9_data_i[250] ^ crc9_data_i[248] ^
        crc9_data_i[245] ^ crc9_data_i[243] ^ crc9_data_i[242] ^ crc9_data_i[239] ^ crc9_data_i[238] ^ crc9_data_i[237] ^
        crc9_data_i[235] ^ crc9_data_i[232] ^ crc9_data_i[229] ^ crc9_data_i[227] ^ crc9_data_i[225] ^ crc9_data_i[222] ^
        crc9_data_i[220] ^ crc9_data_i[214] ^ crc9_data_i[213] ^ crc9_data_i[212] ^ crc9_data_i[208] ^ crc9_data_i[206] ^
        crc9_data_i[205] ^ crc9_data_i[204] ^ crc9_data_i[199] ^ crc9_data_i[198] ^ crc9_data_i[196] ^ crc9_data_i[195] ^
        crc9_data_i[194] ^ crc9_data_i[193] ^ crc9_data_i[191] ^ crc9_data_i[187] ^ crc9_data_i[182] ^ crc9_data_i[174] ^
        crc9_data_i[173] ^ crc9_data_i[171] ^ crc9_data_i[169] ^ crc9_data_i[168] ^ crc9_data_i[167] ^ crc9_data_i[164] ^
        crc9_data_i[162] ^ crc9_data_i[160] ^ crc9_data_i[159] ^ crc9_data_i[157] ^ crc9_data_i[156] ^ crc9_data_i[153] ^
        crc9_data_i[147] ^ crc9_data_i[145] ^ crc9_data_i[143] ^ crc9_data_i[142] ^ crc9_data_i[141] ^ crc9_data_i[140] ^
        crc9_data_i[139] ^ crc9_data_i[138] ^ crc9_data_i[137] ^ crc9_data_i[136] ^ crc9_data_i[135] ^ crc9_data_i[131] ^
        crc9_data_i[128] ^ crc9_data_i[127] ^ crc9_data_i[126] ^ crc9_data_i[125] ^ crc9_data_i[123] ^ crc9_data_i[122] ^
        crc9_data_i[121] ^ crc9_data_i[117] ^ crc9_data_i[116] ^ crc9_data_i[115] ^ crc9_data_i[114] ^ crc9_data_i[112] ^
        crc9_data_i[110] ^ crc9_data_i[108] ^ crc9_data_i[107] ^ crc9_data_i[106] ^ crc9_data_i[104] ^ crc9_data_i[103] ^
        crc9_data_i[100] ^ crc9_data_i[99] ^ crc9_data_i[96] ^ crc9_data_i[95] ^ crc9_data_i[94] ^ crc9_data_i[93] ^
        crc9_data_i[92] ^ crc9_data_i[90] ^ crc9_data_i[87] ^ crc9_data_i[86] ^ crc9_data_i[84] ^ crc9_data_i[83] ^
        crc9_data_i[77] ^ crc9_data_i[76] ^ crc9_data_i[72] ^ crc9_data_i[68] ^ crc9_data_i[67] ^ crc9_data_i[65] ^
        crc9_data_i[64] ^ crc9_data_i[62] ^ crc9_data_i[59] ^ crc9_data_i[57] ^ crc9_data_i[56] ^ crc9_data_i[53] ^
        crc9_data_i[52] ^ crc9_data_i[51] ^ crc9_data_i[49] ^ crc9_data_i[46] ^ crc9_data_i[43] ^ crc9_data_i[41] ^
        crc9_data_i[39] ^ crc9_data_i[36] ^ crc9_data_i[34] ^ crc9_data_i[28] ^ crc9_data_i[27] ^ crc9_data_i[26] ^
        crc9_data_i[22] ^ crc9_data_i[20] ^ crc9_data_i[19] ^ crc9_data_i[18] ^ crc9_data_i[13] ^ crc9_data_i[12] ^
        crc9_data_i[10] ^ crc9_data_i[9] ^ crc9_data_i[8] ^ crc9_data_i[7] ^ crc9_data_i[5] ^ crc9_data_i[1] ^
        1'b1;

// crc9_o[4] : 127 项 XOR
assign crc9_o[4] =
        crc9_data_i[257] ^ crc9_data_i[253] ^ crc9_data_i[252] ^ crc9_data_i[250] ^ crc9_data_i[249] ^ crc9_data_i[247] ^
        crc9_data_i[244] ^ crc9_data_i[242] ^ crc9_data_i[241] ^ crc9_data_i[238] ^ crc9_data_i[237] ^ crc9_data_i[236] ^
        crc9_data_i[234] ^ crc9_data_i[231] ^ crc9_data_i[228] ^ crc9_data_i[226] ^ crc9_data_i[224] ^ crc9_data_i[221] ^
        crc9_data_i[219] ^ crc9_data_i[213] ^ crc9_data_i[212] ^ crc9_data_i[211] ^ crc9_data_i[207] ^ crc9_data_i[205] ^
        crc9_data_i[204] ^ crc9_data_i[203] ^ crc9_data_i[198] ^ crc9_data_i[197] ^ crc9_data_i[195] ^ crc9_data_i[194] ^
        crc9_data_i[193] ^ crc9_data_i[192] ^ crc9_data_i[190] ^ crc9_data_i[186] ^ crc9_data_i[181] ^ crc9_data_i[173] ^
        crc9_data_i[172] ^ crc9_data_i[170] ^ crc9_data_i[168] ^ crc9_data_i[167] ^ crc9_data_i[166] ^ crc9_data_i[163] ^
        crc9_data_i[161] ^ crc9_data_i[159] ^ crc9_data_i[158] ^ crc9_data_i[156] ^ crc9_data_i[155] ^ crc9_data_i[152] ^
        crc9_data_i[146] ^ crc9_data_i[144] ^ crc9_data_i[142] ^ crc9_data_i[141] ^ crc9_data_i[140] ^ crc9_data_i[139] ^
        crc9_data_i[138] ^ crc9_data_i[137] ^ crc9_data_i[136] ^ crc9_data_i[135] ^ crc9_data_i[134] ^ crc9_data_i[130] ^
        crc9_data_i[127] ^ crc9_data_i[126] ^ crc9_data_i[125] ^ crc9_data_i[124] ^ crc9_data_i[122] ^ crc9_data_i[121] ^
        crc9_data_i[120] ^ crc9_data_i[116] ^ crc9_data_i[115] ^ crc9_data_i[114] ^ crc9_data_i[113] ^ crc9_data_i[111] ^
        crc9_data_i[109] ^ crc9_data_i[107] ^ crc9_data_i[106] ^ crc9_data_i[105] ^ crc9_data_i[103] ^ crc9_data_i[102] ^
        crc9_data_i[99] ^ crc9_data_i[98] ^ crc9_data_i[95] ^ crc9_data_i[94] ^ crc9_data_i[93] ^ crc9_data_i[92] ^
        crc9_data_i[91] ^ crc9_data_i[89] ^ crc9_data_i[86] ^ crc9_data_i[85] ^ crc9_data_i[83] ^ crc9_data_i[82] ^
        crc9_data_i[76] ^ crc9_data_i[75] ^ crc9_data_i[71] ^ crc9_data_i[67] ^ crc9_data_i[66] ^ crc9_data_i[64] ^
        crc9_data_i[63] ^ crc9_data_i[61] ^ crc9_data_i[58] ^ crc9_data_i[56] ^ crc9_data_i[55] ^ crc9_data_i[52] ^
        crc9_data_i[51] ^ crc9_data_i[50] ^ crc9_data_i[48] ^ crc9_data_i[45] ^ crc9_data_i[42] ^ crc9_data_i[40] ^
        crc9_data_i[38] ^ crc9_data_i[35] ^ crc9_data_i[33] ^ crc9_data_i[27] ^ crc9_data_i[26] ^ crc9_data_i[25] ^
        crc9_data_i[21] ^ crc9_data_i[19] ^ crc9_data_i[18] ^ crc9_data_i[17] ^ crc9_data_i[12] ^ crc9_data_i[11] ^
        crc9_data_i[9] ^ crc9_data_i[8] ^ crc9_data_i[7] ^ crc9_data_i[6] ^ crc9_data_i[4] ^ crc9_data_i[0] ^
        1'b1;

// crc9_o[3] : 133 项 XOR
assign crc9_o[3] =
        crc9_data_i[260] ^ crc9_data_i[253] ^ crc9_data_i[252] ^ crc9_data_i[251] ^ crc9_data_i[250] ^ crc9_data_i[246] ^
        crc9_data_i[245] ^ crc9_data_i[243] ^ crc9_data_i[240] ^ crc9_data_i[239] ^ crc9_data_i[238] ^ crc9_data_i[237] ^
        crc9_data_i[236] ^ crc9_data_i[235] ^ crc9_data_i[234] ^ crc9_data_i[232] ^ crc9_data_i[230] ^ crc9_data_i[229] ^
        crc9_data_i[228] ^ crc9_data_i[227] ^ crc9_data_i[225] ^ crc9_data_i[224] ^ crc9_data_i[220] ^ crc9_data_i[218] ^
        crc9_data_i[216] ^ crc9_data_i[214] ^ crc9_data_i[213] ^ crc9_data_i[210] ^ crc9_data_i[208] ^ crc9_data_i[205] ^
        crc9_data_i[200] ^ crc9_data_i[199] ^ crc9_data_i[196] ^ crc9_data_i[195] ^ crc9_data_i[193] ^ crc9_data_i[192] ^
        crc9_data_i[191] ^ crc9_data_i[188] ^ crc9_data_i[187] ^ crc9_data_i[186] ^ crc9_data_i[180] ^ crc9_data_i[176] ^
        crc9_data_i[173] ^ crc9_data_i[171] ^ crc9_data_i[170] ^ crc9_data_i[169] ^ crc9_data_i[167] ^ crc9_data_i[160] ^
        crc9_data_i[159] ^ crc9_data_i[158] ^ crc9_data_i[157] ^ crc9_data_i[153] ^ crc9_data_i[152] ^ crc9_data_i[150] ^
        crc9_data_i[147] ^ crc9_data_i[146] ^ crc9_data_i[145] ^ crc9_data_i[144] ^ crc9_data_i[143] ^ crc9_data_i[142] ^
        crc9_data_i[141] ^ crc9_data_i[139] ^ crc9_data_i[137] ^ crc9_data_i[136] ^ crc9_data_i[135] ^ crc9_data_i[134] ^
        crc9_data_i[132] ^ crc9_data_i[131] ^ crc9_data_i[127] ^ crc9_data_i[125] ^ crc9_data_i[123] ^ crc9_data_i[121] ^
        crc9_data_i[120] ^ crc9_data_i[117] ^ crc9_data_i[115] ^ crc9_data_i[112] ^ crc9_data_i[107] ^ crc9_data_i[106] ^
        crc9_data_i[103] ^ crc9_data_i[102] ^ crc9_data_i[100] ^ crc9_data_i[99] ^ crc9_data_i[98] ^ crc9_data_i[95] ^
        crc9_data_i[94] ^ crc9_data_i[93] ^ crc9_data_i[87] ^ crc9_data_i[83] ^ crc9_data_i[80] ^ crc9_data_i[78] ^
        crc9_data_i[77] ^ crc9_data_i[76] ^ crc9_data_i[74] ^ crc9_data_i[67] ^ crc9_data_i[66] ^ crc9_data_i[65] ^
        crc9_data_i[64] ^ crc9_data_i[60] ^ crc9_data_i[59] ^ crc9_data_i[57] ^ crc9_data_i[54] ^ crc9_data_i[53] ^
        crc9_data_i[52] ^ crc9_data_i[51] ^ crc9_data_i[50] ^ crc9_data_i[49] ^ crc9_data_i[48] ^ crc9_data_i[46] ^
        crc9_data_i[44] ^ crc9_data_i[43] ^ crc9_data_i[42] ^ crc9_data_i[41] ^ crc9_data_i[39] ^ crc9_data_i[38] ^
        crc9_data_i[34] ^ crc9_data_i[32] ^ crc9_data_i[30] ^ crc9_data_i[28] ^ crc9_data_i[27] ^ crc9_data_i[24] ^
        crc9_data_i[22] ^ crc9_data_i[19] ^ crc9_data_i[14] ^ crc9_data_i[13] ^ crc9_data_i[10] ^ crc9_data_i[9] ^
        crc9_data_i[7] ^ crc9_data_i[6] ^ crc9_data_i[5] ^ crc9_data_i[2] ^ crc9_data_i[1] ^ crc9_data_i[0] ^
        1'b1;

// crc9_o[2] : 131 项 XOR
assign crc9_o[2] =
        crc9_data_i[259] ^ crc9_data_i[256] ^ crc9_data_i[253] ^ crc9_data_i[252] ^ crc9_data_i[251] ^ crc9_data_i[248] ^
        crc9_data_i[244] ^ crc9_data_i[242] ^ crc9_data_i[241] ^ crc9_data_i[237] ^ crc9_data_i[236] ^ crc9_data_i[235] ^
        crc9_data_i[232] ^ crc9_data_i[231] ^ crc9_data_i[227] ^ crc9_data_i[226] ^ crc9_data_i[219] ^ crc9_data_i[217] ^
        crc9_data_i[216] ^ crc9_data_i[215] ^ crc9_data_i[214] ^ crc9_data_i[211] ^ crc9_data_i[209] ^ crc9_data_i[208] ^
        crc9_data_i[207] ^ crc9_data_i[206] ^ crc9_data_i[205] ^ crc9_data_i[203] ^ crc9_data_i[202] ^ crc9_data_i[200] ^
        crc9_data_i[198] ^ crc9_data_i[197] ^ crc9_data_i[192] ^ crc9_data_i[191] ^ crc9_data_i[190] ^ crc9_data_i[189] ^
        crc9_data_i[188] ^ crc9_data_i[179] ^ crc9_data_i[176] ^ crc9_data_i[175] ^ crc9_data_i[173] ^ crc9_data_i[169] ^
        crc9_data_i[168] ^ crc9_data_i[165] ^ crc9_data_i[162] ^ crc9_data_i[158] ^ crc9_data_i[157] ^ crc9_data_i[156] ^
        crc9_data_i[155] ^ crc9_data_i[154] ^ crc9_data_i[153] ^ crc9_data_i[150] ^ crc9_data_i[149] ^ crc9_data_i[147] ^
        crc9_data_i[145] ^ crc9_data_i[143] ^ crc9_data_i[141] ^ crc9_data_i[136] ^ crc9_data_i[135] ^ crc9_data_i[134] ^
        crc9_data_i[132] ^ crc9_data_i[130] ^ crc9_data_i[129] ^ crc9_data_i[127] ^ crc9_data_i[122] ^ crc9_data_i[120] ^
        crc9_data_i[117] ^ crc9_data_i[116] ^ crc9_data_i[113] ^ crc9_data_i[111] ^ crc9_data_i[110] ^ crc9_data_i[108] ^
        crc9_data_i[107] ^ crc9_data_i[106] ^ crc9_data_i[104] ^ crc9_data_i[103] ^ crc9_data_i[102] ^ crc9_data_i[100] ^
        crc9_data_i[98] ^ crc9_data_i[95] ^ crc9_data_i[94] ^ crc9_data_i[93] ^ crc9_data_i[91] ^ crc9_data_i[90] ^
        crc9_data_i[88] ^ crc9_data_i[87] ^ crc9_data_i[86] ^ crc9_data_i[85] ^ crc9_data_i[84] ^ crc9_data_i[83] ^
        crc9_data_i[81] ^ crc9_data_i[80] ^ crc9_data_i[79] ^ crc9_data_i[78] ^ crc9_data_i[73] ^ crc9_data_i[70] ^
        crc9_data_i[67] ^ crc9_data_i[66] ^ crc9_data_i[65] ^ crc9_data_i[62] ^ crc9_data_i[58] ^ crc9_data_i[56] ^
        crc9_data_i[55] ^ crc9_data_i[51] ^ crc9_data_i[50] ^ crc9_data_i[49] ^ crc9_data_i[46] ^ crc9_data_i[45] ^
        crc9_data_i[41] ^ crc9_data_i[40] ^ crc9_data_i[33] ^ crc9_data_i[31] ^ crc9_data_i[30] ^ crc9_data_i[29] ^
        crc9_data_i[28] ^ crc9_data_i[25] ^ crc9_data_i[23] ^ crc9_data_i[22] ^ crc9_data_i[21] ^ crc9_data_i[20] ^
        crc9_data_i[19] ^ crc9_data_i[17] ^ crc9_data_i[16] ^ crc9_data_i[14] ^ crc9_data_i[12] ^ crc9_data_i[11] ^
        crc9_data_i[6] ^ crc9_data_i[5] ^ crc9_data_i[4] ^ crc9_data_i[3] ^ crc9_data_i[2];

// crc9_o[1] : 132 项 XOR
assign crc9_o[1] =
        crc9_data_i[258] ^ crc9_data_i[255] ^ crc9_data_i[252] ^ crc9_data_i[251] ^ crc9_data_i[250] ^ crc9_data_i[247] ^
        crc9_data_i[243] ^ crc9_data_i[241] ^ crc9_data_i[240] ^ crc9_data_i[236] ^ crc9_data_i[235] ^ crc9_data_i[234] ^
        crc9_data_i[231] ^ crc9_data_i[230] ^ crc9_data_i[226] ^ crc9_data_i[225] ^ crc9_data_i[218] ^ crc9_data_i[216] ^
        crc9_data_i[215] ^ crc9_data_i[214] ^ crc9_data_i[213] ^ crc9_data_i[210] ^ crc9_data_i[208] ^ crc9_data_i[207] ^
        crc9_data_i[206] ^ crc9_data_i[205] ^ crc9_data_i[204] ^ crc9_data_i[202] ^ crc9_data_i[201] ^ crc9_data_i[199] ^
        crc9_data_i[197] ^ crc9_data_i[196] ^ crc9_data_i[191] ^ crc9_data_i[190] ^ crc9_data_i[189] ^ crc9_data_i[188] ^
        crc9_data_i[187] ^ crc9_data_i[178] ^ crc9_data_i[175] ^ crc9_data_i[174] ^ crc9_data_i[172] ^ crc9_data_i[168] ^
        crc9_data_i[167] ^ crc9_data_i[164] ^ crc9_data_i[161] ^ crc9_data_i[157] ^ crc9_data_i[156] ^ crc9_data_i[155] ^
        crc9_data_i[154] ^ crc9_data_i[153] ^ crc9_data_i[152] ^ crc9_data_i[149] ^ crc9_data_i[148] ^ crc9_data_i[146] ^
        crc9_data_i[144] ^ crc9_data_i[142] ^ crc9_data_i[140] ^ crc9_data_i[135] ^ crc9_data_i[134] ^ crc9_data_i[133] ^
        crc9_data_i[131] ^ crc9_data_i[129] ^ crc9_data_i[128] ^ crc9_data_i[126] ^ crc9_data_i[121] ^ crc9_data_i[119] ^
        crc9_data_i[116] ^ crc9_data_i[115] ^ crc9_data_i[112] ^ crc9_data_i[110] ^ crc9_data_i[109] ^ crc9_data_i[107] ^
        crc9_data_i[106] ^ crc9_data_i[105] ^ crc9_data_i[103] ^ crc9_data_i[102] ^ crc9_data_i[101] ^ crc9_data_i[99] ^
        crc9_data_i[97] ^ crc9_data_i[94] ^ crc9_data_i[93] ^ crc9_data_i[92] ^ crc9_data_i[90] ^ crc9_data_i[89] ^
        crc9_data_i[87] ^ crc9_data_i[86] ^ crc9_data_i[85] ^ crc9_data_i[84] ^ crc9_data_i[83] ^ crc9_data_i[82] ^
        crc9_data_i[80] ^ crc9_data_i[79] ^ crc9_data_i[78] ^ crc9_data_i[77] ^ crc9_data_i[72] ^ crc9_data_i[69] ^
        crc9_data_i[66] ^ crc9_data_i[65] ^ crc9_data_i[64] ^ crc9_data_i[61] ^ crc9_data_i[57] ^ crc9_data_i[55] ^
        crc9_data_i[54] ^ crc9_data_i[50] ^ crc9_data_i[49] ^ crc9_data_i[48] ^ crc9_data_i[45] ^ crc9_data_i[44] ^
        crc9_data_i[40] ^ crc9_data_i[39] ^ crc9_data_i[32] ^ crc9_data_i[30] ^ crc9_data_i[29] ^ crc9_data_i[28] ^
        crc9_data_i[27] ^ crc9_data_i[24] ^ crc9_data_i[22] ^ crc9_data_i[21] ^ crc9_data_i[20] ^ crc9_data_i[19] ^
        crc9_data_i[18] ^ crc9_data_i[16] ^ crc9_data_i[15] ^ crc9_data_i[13] ^ crc9_data_i[11] ^ crc9_data_i[10] ^
        crc9_data_i[5] ^ crc9_data_i[4] ^ crc9_data_i[3] ^ crc9_data_i[2] ^ crc9_data_i[1] ^ 1'b1;

// crc9_o[0] : 131 项 XOR
assign crc9_o[0] =
        crc9_data_i[257] ^ crc9_data_i[254] ^ crc9_data_i[251] ^ crc9_data_i[250] ^ crc9_data_i[249] ^ crc9_data_i[246] ^
        crc9_data_i[242] ^ crc9_data_i[240] ^ crc9_data_i[239] ^ crc9_data_i[235] ^ crc9_data_i[234] ^ crc9_data_i[233] ^
        crc9_data_i[230] ^ crc9_data_i[229] ^ crc9_data_i[225] ^ crc9_data_i[224] ^ crc9_data_i[217] ^ crc9_data_i[215] ^
        crc9_data_i[214] ^ crc9_data_i[213] ^ crc9_data_i[212] ^ crc9_data_i[209] ^ crc9_data_i[207] ^ crc9_data_i[206] ^
        crc9_data_i[205] ^ crc9_data_i[204] ^ crc9_data_i[203] ^ crc9_data_i[201] ^ crc9_data_i[200] ^ crc9_data_i[198] ^
        crc9_data_i[196] ^ crc9_data_i[195] ^ crc9_data_i[190] ^ crc9_data_i[189] ^ crc9_data_i[188] ^ crc9_data_i[187] ^
        crc9_data_i[186] ^ crc9_data_i[177] ^ crc9_data_i[174] ^ crc9_data_i[173] ^ crc9_data_i[171] ^ crc9_data_i[167] ^
        crc9_data_i[166] ^ crc9_data_i[163] ^ crc9_data_i[160] ^ crc9_data_i[156] ^ crc9_data_i[155] ^ crc9_data_i[154] ^
        crc9_data_i[153] ^ crc9_data_i[152] ^ crc9_data_i[151] ^ crc9_data_i[148] ^ crc9_data_i[147] ^ crc9_data_i[145] ^
        crc9_data_i[143] ^ crc9_data_i[141] ^ crc9_data_i[139] ^ crc9_data_i[134] ^ crc9_data_i[133] ^ crc9_data_i[132] ^
        crc9_data_i[130] ^ crc9_data_i[128] ^ crc9_data_i[127] ^ crc9_data_i[125] ^ crc9_data_i[120] ^ crc9_data_i[118] ^
        crc9_data_i[115] ^ crc9_data_i[114] ^ crc9_data_i[111] ^ crc9_data_i[109] ^ crc9_data_i[108] ^ crc9_data_i[106] ^
        crc9_data_i[105] ^ crc9_data_i[104] ^ crc9_data_i[102] ^ crc9_data_i[101] ^ crc9_data_i[100] ^ crc9_data_i[98] ^
        crc9_data_i[96] ^ crc9_data_i[93] ^ crc9_data_i[92] ^ crc9_data_i[91] ^ crc9_data_i[89] ^ crc9_data_i[88] ^
        crc9_data_i[86] ^ crc9_data_i[85] ^ crc9_data_i[84] ^ crc9_data_i[83] ^ crc9_data_i[82] ^ crc9_data_i[81] ^
        crc9_data_i[79] ^ crc9_data_i[78] ^ crc9_data_i[77] ^ crc9_data_i[76] ^ crc9_data_i[71] ^ crc9_data_i[68] ^
        crc9_data_i[65] ^ crc9_data_i[64] ^ crc9_data_i[63] ^ crc9_data_i[60] ^ crc9_data_i[56] ^ crc9_data_i[54] ^
        crc9_data_i[53] ^ crc9_data_i[49] ^ crc9_data_i[48] ^ crc9_data_i[47] ^ crc9_data_i[44] ^ crc9_data_i[43] ^
        crc9_data_i[39] ^ crc9_data_i[38] ^ crc9_data_i[31] ^ crc9_data_i[29] ^ crc9_data_i[28] ^ crc9_data_i[27] ^
        crc9_data_i[26] ^ crc9_data_i[23] ^ crc9_data_i[21] ^ crc9_data_i[20] ^ crc9_data_i[19] ^ crc9_data_i[18] ^
        crc9_data_i[17] ^ crc9_data_i[15] ^ crc9_data_i[14] ^ crc9_data_i[12] ^ crc9_data_i[10] ^ crc9_data_i[9] ^
        crc9_data_i[4] ^ crc9_data_i[3] ^ crc9_data_i[2] ^ crc9_data_i[1] ^ crc9_data_i[0];

endmodule
