// -------------------------------------------------------------------------------
// File   : onehot_to_binary.v
// Author : OpenHappy Team
// Date   : 2026-08-03
// -------------------------------------------------------------------------------
// Description:
//   Parameterizable one-hot to binary encoder. Accepts a one-hot input (at most
//   one bit set, or all zeros) and outputs the binary index of the set bit.
//   When no bits are set, binary_out is zero. ONEHOT_WIDTH must be >= 1.
//   Timing: single level of OR reduction per output bit — no priority chain.
// -------------------------------------------------------------------------------
// Change Log:
//   2026-08-03  OpenHappy Team  v1.0  Initial generation
//   2026-08-12  OpenHappy Team  v1.1  contrib 与 genvar 声明移至模块头部（禁止 generate 内内联声明）；
//                                按真值表转置结构重写：var1[i]=one_hot_in[i]?索引i:0（索引 i[OUTPUT_WIDTH-1:0]
//                                显式截取，避免 32-bit genvar 截断告警）→ var2=var1 转置 → binary_out=|var2；
//                                位宽可参数化，-Wall 零宽度告警
// -------------------------------------------------------------------------------


module onehot_to_binary #(
    parameter ONEHOT_WIDTH = 16
)
(
    input  wire [ONEHOT_WIDTH-1 : 0]                                   one_hot_in,
    output wire [(ONEHOT_WIDTH == 1) ? 0 : $clog2(ONEHOT_WIDTH)-1 : 0] binary_out
);

    localparam OUTPUT_WIDTH = (ONEHOT_WIDTH == 1) ? 1 : $clog2(ONEHOT_WIDTH);

    // --- 独热 → 二进制（真值表，位宽可参数化） ---
    // 真值表：one_hot_in[i] = 1 → binary_out = 索引 i 的二进制值。
    // bit_index[i]       = one_hot_in[i] ? 索引 i : 0（每个 onehot bit 贡献其二进制索引）
    // out_bit_contrib[j] = bit_index 的转置（out_bit_contrib[j][i] = bit_index[i][j]）
    // binary_out[j]      = |out_bit_contrib[j]（按位或）
    wire [OUTPUT_WIDTH-1 : 0] bit_index       [ONEHOT_WIDTH-1:0];
    wire [ONEHOT_WIDTH-1 : 0] out_bit_contrib [OUTPUT_WIDTH-1:0];
    genvar i;
    genvar j;

    generate
        for (i = 0; i < ONEHOT_WIDTH; i = i + 1) begin : gen_idx
            assign bit_index[i] = one_hot_in[i] ? i : {OUTPUT_WIDTH{1'b0}};
        end
        for (j = 0; j < OUTPUT_WIDTH; j = j + 1) begin : gen_out
            for (i = 0; i < ONEHOT_WIDTH; i = i + 1) begin : gen_in
                assign out_bit_contrib[j][i] = bit_index[i][j];
            end
            assign binary_out[j] = |out_bit_contrib[j];
        end
    endgenerate

endmodule

