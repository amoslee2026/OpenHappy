// =============================================================================
// module name：ecc_check
// author: OpenHappy Team
// MAS：spec/link_layer/submods/link_rx/mas/ecc_check_mas.md
// 功能概述: 性能模式下 RX 侧 ECC（SECDED 12bit，纠 1 检 2）校验单元，单引擎
//           分时复用（128 B 单元粒度，与 crc16_check 同构）。每个 128 B 单元
//           收齐即校验：unit0 / unit1 均在单元内同一位置比对（ECC 字段
//           unit[1019:1008] = {P, tie0, SEC[9:0]}，高 4bits tie0），同一套
//           矩阵覆盖两种单元。校验按 1008 bits 计算：重算仅引用 unit_i[1007:0]
//           （Byte 0~125），槽位 [1023:1008] 不被矩阵引用（承载待比对字段）。
//           与 CRC16 完全一致的去 D23 纯组合直通（v1.9）：E 拍同拍组合
//           计算 ecc_calc_w（bit 11 = P 重算行）与 syndrome_w = ecc_calc_w ^
//           unit_i[1019:1008]（12-bit；S = syndrome[10:0] SEC 部分 + P_chk =
//           syndrome[11] 奇偶比较，双判据）；使能态 pass/err 与纠正于组包完成
//           拍（E）同拍组合直通、禁用态 E 拍直通。
//           (S,P_chk) 分类：(0,any) pass（无错 / P 位自身单错）/ (col[i],1) 数据
//           位单错纠正 pass / (e_j,1) 字段位单错（j∈[0,10]）pass 不纠 /
//           (S≠0,0) 双错、未分类 (S≠0,1) 多错 → err（P 覆盖 1020 位保护域，
//           C(1020,2)=519690 双错对全枚举 → err；≥3 错不保证）。ECC 多一步
//           单错纠正：命中数据列（载荷位单错）时输出 corrected 数据 corr_unit_o
//           供判通拍 push（corr_mask 由 E 拍 syndrome_w 同拍组合译码、不打拍，
//           P_chk 门控）。矩阵由 gen_ecc_matrix.py 生成（P 重算行模块内合成，
//           勿手改）；与 ecc_generate（v1.3 口径）同一矩阵 / 同一 1008-bit 口径。
// =============================================================================
// version history(仅记录规格变更、重要BUG修复，不记录lint等小问题修改)
//   <日期>  <修订人>  <变更概述>
//  2026-10-15  OpenHappy Team  LL-20261015-DUALPUSH（user 两路隔离 + 去 D23，MAS v1.9）：本模块为
//                         路 2（重传使能）校验引擎——删 D23 打 1 拍（ecc_calc_p/ecc_field_p/
//                         flit_valid_p/unit_p 寄存），E 拍同拍组合直通：syndrome_w=ecc_calc_w^
//                         unit_i[1019:1008]、corr_mask_w 由 E 拍 syndrome_w 同拍译码（代数式
//                         不变）、corr_unit_o=unit_i^{16'd0,corr_mask_w}，pass/err 于组包完成
//                         拍（E）同拍回接（算例不变）；mb_clk/mb_rst_n 端口保留（同 crc9_check 先例）。
//  2026-09-19  OpenHappy Team  初始版本（ECC 交付 LL-20260919-ECC；D23 E+1 打拍对齐
//                          crc16_check v1.3；syndrome 判别覆盖 0/单数据列/单位向量/其余）
//  2026-09-21  OpenHappy Team  BUG-001 修复（LL-20260921-ECC1）：corr_mask 改 E+1 拍 syndrome_p
//                          同拍组合译码、不打拍，corr_unit_o = unit_p ^ {16'd0, corr_mask_w}；
//                          删除 corr_mask_p 打拍寄存（旧实现 E+2 把下一单元掩码误用于当前单元）
//  2026-09-23  OpenHappy Team  用户裁决（LL-20260923-CHK1008）：口径统一 1008 bits（矩阵
//                          截断 12×1008，5065 项，删 unit_kern_w）+ 升级 SECDED（纠 1 检 2）——
//                          ecc_calc_w[11] 由空行改 P 重算行 ^{unit_i[1007:0], unit_i[1018:1008]}
//                          （1019 项，@E 组合，折入现有 12-bit ecc_calc_p 打拍，零新增寄存器）；
//                          decode 升级 (S, P_chk) 双判据（S = syndrome[10:0]，P_chk = syndrome[11]）：
//                          (0,any) pass / (col,1) 数据单错纠正 / (e_j,1) 字段位单错
//                          （j∈[0,10]）pass 不纠 / (S≠0,0) 与未分类 (S≠0,1) → err
//                          （检 2bit）；field 0..9 单错/数据单错行为与旧一致，P 自身单错
//                          旧经 e_11(0x800) 校验列 pass、新显式 (S=0,P=1) 分类行为不变，
//                          field10 单错旧 pass 新同 pass，双错由旧误纠/误 pass 升级为
//                          err；版本约束：TX/RX 必须同版本升级；补 syndrome_p assign
//                          （矩阵重写时遗漏，单元冒烟 1481 向量实证 X 态后补齐，MAS v1.4 §5.1 既有式）
//  2026-09-29  OpenHappy Team  LL-TIMING-N2 改法 A（DC 时序 N=2 job n2，arch 评审 v2 通过）：
//                          decode 代数化——data_hit_p 1008 路 11-bit 等值 OR（≈14-15 级）→
//                          {3≤s≤0x3FA 非 2 幂} 判据（≈8-9 级）；parity_err_p 11 路等值 OR →
//                          s_pow2（与 data_hit 共享）；uncorr_p 补集式 → 代数式。2048×2 (s,p)
//                          穷举恒等 0 diff（本会话脚本 + arch 独立复核）。判据语义 MAS v1.4
//                          §5.1 不变（实现形式自由）；corr_mask 1008 条保留勿手改（列值集与
//                          data_hit 集同源）。预期 CLKICG 21 家族 -4~6。
// =============================================================================

`timescale 1ns/1ps

module ecc_check (
    // 时钟与复位（v1.9 去 D23 后纯组合直通；端口保留，对齐 crc9_check 先例）
    input  wire               mb_clk,
    input  wire               mb_rst_n,

    // 配置（apb_csr 具名直送，经 link_rx 透传）
    input  wire               csr_perf_mode_i,     // 性能模式指示，高有效
    input  wire               csr_ecc_en_i,        // ECC 校验总使能（RETRY_CFG_REG.ECC_EN，桥接门控在父层）

    // 数据输入（1 个 128 B 单元）
    // ECC 字段固定位于 unit_i[1019:1008] = {P, tie0, SEC[9:0]}（高 4bits [1023:1020] tie0）；
    // 校验按 1008 bits 计算——矩阵结构性不引用槽位 [1023:1008]（调用方可送完整单元，槽位值不影响校验结果）
    input  wire [1023:0]      unit_i,
    input  wire               flit_valid_i,    // 单元有效，校验拍拉高 1 cycle

    // 校验结果输出（v1.9 去 D23：使能态 E 拍同拍组合直通；禁用态 pass=E 拍直通，err=0）
    output wire               pass_o,          // 本单元 ECC 校验通过指示（无错 / P 位单错 / 字段位单错 / 可纠数据单错）
    output wire               err_o,           // 本单元不可纠脉冲（(S≠0,P_chk=0) 双错 / (S≠0,P_chk=1) 未分类多错）
    output wire [1023:0]      corr_unit_o      // E 拍纠正后单元数据（unit_i ^ corr_mask_w；无错/字段位错/P 单错=原值）
);

// =============================================================================
// 信号声明
// =============================================================================
wire [11:0]   ecc_calc_w;     // H·compute E 拍组合（XOR 树直出，bit 11 = P 重算行）
wire [11:0]   ecc_field_w;    // 单元携带 ECC 字段（E 拍，unit_i[1019:1008]）
wire [1007:0] corr_mask_w;    // 单数据错纠正掩码 E 拍组合译码（P_chk & s_chk 命中载荷列 → 单热点；与 syndrome 同拍、不打拍；仅 0..1007 载荷位）
wire [11:0]   syndrome_w;     // ecc_calc_w ^ ecc_field_w（E 拍组合，= {P_chk, S}）
wire [10:0]   s_chk;          // 11-bit SEC syndrome（syndrome_w[10:0]，双判据 S）
wire          p_chk;          // P 位奇偶比较（syndrome_w[11]，双判据 P_chk）
wire          s_pow2;           // S 为 2 幂（2^0..2^10 字段单位向量；s=0 由 |s_chk 排除；LL-TIMING-N2 改法 A）
wire          en_ck;          // ECC 校验使能合取（性能模式 & ECC_EN，静态 CSR）
wire          data_hit_w;     // (S 命中载荷数据列 & P_chk=1)：载荷位 0..1007 单错可纠（S 值 3..1018）
wire          parity_err_w;   // (S 命中单位向量 e_0..e_10 & P_chk=1)：字段位自身单错，pass 不纠
wire          uncorr_w;       // (S≠0 & P_chk=0) 双错 / (S≠0 & P_chk=1) 未分类多错（不可纠）

// =============================================================================
// ECC 引擎（H·compute，单引擎 1024 bit → 12 bit；矩阵同 ecc_generate 展开，纯组合 0 cycle）
// 1008-bit 口径：矩阵结构性不引用槽位 [1023:1008]（调用方可送完整单元，槽位值不影响校验结果）
// P 重算行（SECDED）：ecc_calc_w[11] = ^{unit_i[1007:0], unit_i[1018:1008]}（1019 项，@E 组合与矩阵同级深度）
// =============================================================================

assign ecc_calc_w[ 0] = ^{unit_i[   0], unit_i[   1], unit_i[   3], unit_i[   4], unit_i[   6], unit_i[   8], unit_i[  10], unit_i[  11], unit_i[  13], unit_i[  15], unit_i[  17], unit_i[  19], unit_i[  21], unit_i[  23], unit_i[  25], unit_i[  26], unit_i[  28], unit_i[  30], unit_i[  32], unit_i[  34], unit_i[  36], unit_i[  38], unit_i[  40], unit_i[  42], unit_i[  44], unit_i[  46], unit_i[  48], unit_i[  50], unit_i[  52], unit_i[  54], unit_i[  56], unit_i[  57], unit_i[  59], unit_i[  61], unit_i[  63], unit_i[  65], unit_i[  67], unit_i[  69], unit_i[  71], unit_i[  73], unit_i[  75], unit_i[  77], unit_i[  79], unit_i[  81], unit_i[  83], unit_i[  85], unit_i[  87], unit_i[  89], unit_i[  91], unit_i[  93], unit_i[  95], unit_i[  97], unit_i[  99], unit_i[ 101], unit_i[ 103], unit_i[ 105], unit_i[ 107], unit_i[ 109], unit_i[ 111], unit_i[ 113], unit_i[ 115], unit_i[ 117], unit_i[ 119], unit_i[ 120], unit_i[ 122], unit_i[ 124], unit_i[ 126], unit_i[ 128], unit_i[ 130], unit_i[ 132], unit_i[ 134], unit_i[ 136], unit_i[ 138], unit_i[ 140], unit_i[ 142], unit_i[ 144], unit_i[ 146], unit_i[ 148], unit_i[ 150], unit_i[ 152], unit_i[ 154], unit_i[ 156], unit_i[ 158], unit_i[ 160], unit_i[ 162], unit_i[ 164], unit_i[ 166], unit_i[ 168], unit_i[ 170], unit_i[ 172], unit_i[ 174], unit_i[ 176], unit_i[ 178], unit_i[ 180], unit_i[ 182], unit_i[ 184], unit_i[ 186], unit_i[ 188], unit_i[ 190], unit_i[ 192], unit_i[ 194], unit_i[ 196], unit_i[ 198], unit_i[ 200], unit_i[ 202], unit_i[ 204], unit_i[ 206], unit_i[ 208], unit_i[ 210], unit_i[ 212], unit_i[ 214], unit_i[ 216], unit_i[ 218], unit_i[ 220], unit_i[ 222], unit_i[ 224], unit_i[ 226], unit_i[ 228], unit_i[ 230], unit_i[ 232], unit_i[ 234], unit_i[ 236], unit_i[ 238], unit_i[ 240], unit_i[ 242], unit_i[ 244], unit_i[ 246], unit_i[ 247], unit_i[ 249], unit_i[ 251], unit_i[ 253], unit_i[ 255], unit_i[ 257], unit_i[ 259], unit_i[ 261], unit_i[ 263], unit_i[ 265], unit_i[ 267], unit_i[ 269], unit_i[ 271], unit_i[ 273], unit_i[ 275], unit_i[ 277], unit_i[ 279], unit_i[ 281], unit_i[ 283], unit_i[ 285], unit_i[ 287], unit_i[ 289], unit_i[ 291], unit_i[ 293], unit_i[ 295], unit_i[ 297], unit_i[ 299], unit_i[ 301], unit_i[ 303], unit_i[ 305], unit_i[ 307], unit_i[ 309], unit_i[ 311], unit_i[ 313], unit_i[ 315], unit_i[ 317], unit_i[ 319], unit_i[ 321], unit_i[ 323], unit_i[ 325], unit_i[ 327], unit_i[ 329], unit_i[ 331], unit_i[ 333], unit_i[ 335], unit_i[ 337], unit_i[ 339], unit_i[ 341], unit_i[ 343], unit_i[ 345], unit_i[ 347], unit_i[ 349], unit_i[ 351], unit_i[ 353], unit_i[ 355], unit_i[ 357], unit_i[ 359], unit_i[ 361], unit_i[ 363], unit_i[ 365], unit_i[ 367], unit_i[ 369], unit_i[ 371], unit_i[ 373], unit_i[ 375], unit_i[ 377], unit_i[ 379], unit_i[ 381], unit_i[ 383], unit_i[ 385], unit_i[ 387], unit_i[ 389], unit_i[ 391], unit_i[ 393], unit_i[ 395], unit_i[ 397], unit_i[ 399], unit_i[ 401], unit_i[ 403], unit_i[ 405], unit_i[ 407], unit_i[ 409], unit_i[ 411], unit_i[ 413], unit_i[ 415], unit_i[ 417], unit_i[ 419], unit_i[ 421], unit_i[ 423], unit_i[ 425], unit_i[ 427], unit_i[ 429], unit_i[ 431], unit_i[ 433], unit_i[ 435], unit_i[ 437], unit_i[ 439], unit_i[ 441], unit_i[ 443], unit_i[ 445], unit_i[ 447], unit_i[ 449], unit_i[ 451], unit_i[ 453], unit_i[ 455], unit_i[ 457], unit_i[ 459], unit_i[ 461], unit_i[ 463], unit_i[ 465], unit_i[ 467], unit_i[ 469], unit_i[ 471], unit_i[ 473], unit_i[ 475], unit_i[ 477], unit_i[ 479], unit_i[ 481], unit_i[ 483], unit_i[ 485], unit_i[ 487], unit_i[ 489], unit_i[ 491], unit_i[ 493], unit_i[ 495], unit_i[ 497], unit_i[ 499], unit_i[ 501], unit_i[ 502], unit_i[ 504], unit_i[ 506], unit_i[ 508], unit_i[ 510], unit_i[ 512], unit_i[ 514], unit_i[ 516], unit_i[ 518], unit_i[ 520], unit_i[ 522], unit_i[ 524], unit_i[ 526], unit_i[ 528], unit_i[ 530], unit_i[ 532], unit_i[ 534], unit_i[ 536], unit_i[ 538], unit_i[ 540], unit_i[ 542], unit_i[ 544], unit_i[ 546], unit_i[ 548], unit_i[ 550], unit_i[ 552], unit_i[ 554], unit_i[ 556], unit_i[ 558], unit_i[ 560], unit_i[ 562], unit_i[ 564], unit_i[ 566], unit_i[ 568], unit_i[ 570], unit_i[ 572], unit_i[ 574], unit_i[ 576], unit_i[ 578], unit_i[ 580], unit_i[ 582], unit_i[ 584], unit_i[ 586], unit_i[ 588], unit_i[ 590], unit_i[ 592], unit_i[ 594], unit_i[ 596], unit_i[ 598], unit_i[ 600], unit_i[ 602], unit_i[ 604], unit_i[ 606], unit_i[ 608], unit_i[ 610], unit_i[ 612], unit_i[ 614], unit_i[ 616], unit_i[ 618], unit_i[ 620], unit_i[ 622], unit_i[ 624], unit_i[ 626], unit_i[ 628], unit_i[ 630], unit_i[ 632], unit_i[ 634], unit_i[ 636], unit_i[ 638], unit_i[ 640], unit_i[ 642], unit_i[ 644], unit_i[ 646], unit_i[ 648], unit_i[ 650], unit_i[ 652], unit_i[ 654], unit_i[ 656], unit_i[ 658], unit_i[ 660], unit_i[ 662], unit_i[ 664], unit_i[ 666], unit_i[ 668], unit_i[ 670], unit_i[ 672], unit_i[ 674], unit_i[ 676], unit_i[ 678], unit_i[ 680], unit_i[ 682], unit_i[ 684], unit_i[ 686], unit_i[ 688], unit_i[ 690], unit_i[ 692], unit_i[ 694], unit_i[ 696], unit_i[ 698], unit_i[ 700], unit_i[ 702], unit_i[ 704], unit_i[ 706], unit_i[ 708], unit_i[ 710], unit_i[ 712], unit_i[ 714], unit_i[ 716], unit_i[ 718], unit_i[ 720], unit_i[ 722], unit_i[ 724], unit_i[ 726], unit_i[ 728], unit_i[ 730], unit_i[ 732], unit_i[ 734], unit_i[ 736], unit_i[ 738], unit_i[ 740], unit_i[ 742], unit_i[ 744], unit_i[ 746], unit_i[ 748], unit_i[ 750], unit_i[ 752], unit_i[ 754], unit_i[ 756], unit_i[ 758], unit_i[ 760], unit_i[ 762], unit_i[ 764], unit_i[ 766], unit_i[ 768], unit_i[ 770], unit_i[ 772], unit_i[ 774], unit_i[ 776], unit_i[ 778], unit_i[ 780], unit_i[ 782], unit_i[ 784], unit_i[ 786], unit_i[ 788], unit_i[ 790], unit_i[ 792], unit_i[ 794], unit_i[ 796], unit_i[ 798], unit_i[ 800], unit_i[ 802], unit_i[ 804], unit_i[ 806], unit_i[ 808], unit_i[ 810], unit_i[ 812], unit_i[ 814], unit_i[ 816], unit_i[ 818], unit_i[ 820], unit_i[ 822], unit_i[ 824], unit_i[ 826], unit_i[ 828], unit_i[ 830], unit_i[ 832], unit_i[ 834], unit_i[ 836], unit_i[ 838], unit_i[ 840], unit_i[ 842], unit_i[ 844], unit_i[ 846], unit_i[ 848], unit_i[ 850], unit_i[ 852], unit_i[ 854], unit_i[ 856], unit_i[ 858], unit_i[ 860], unit_i[ 862], unit_i[ 864], unit_i[ 866], unit_i[ 868], unit_i[ 870], unit_i[ 872], unit_i[ 874], unit_i[ 876], unit_i[ 878], unit_i[ 880], unit_i[ 882], unit_i[ 884], unit_i[ 886], unit_i[ 888], unit_i[ 890], unit_i[ 892], unit_i[ 894], unit_i[ 896], unit_i[ 898], unit_i[ 900], unit_i[ 902], unit_i[ 904], unit_i[ 906], unit_i[ 908], unit_i[ 910], unit_i[ 912], unit_i[ 914], unit_i[ 916], unit_i[ 918], unit_i[ 920], unit_i[ 922], unit_i[ 924], unit_i[ 926], unit_i[ 928], unit_i[ 930], unit_i[ 932], unit_i[ 934], unit_i[ 936], unit_i[ 938], unit_i[ 940], unit_i[ 942], unit_i[ 944], unit_i[ 946], unit_i[ 948], unit_i[ 950], unit_i[ 952], unit_i[ 954], unit_i[ 956], unit_i[ 958], unit_i[ 960], unit_i[ 962], unit_i[ 964], unit_i[ 966], unit_i[ 968], unit_i[ 970], unit_i[ 972], unit_i[ 974], unit_i[ 976], unit_i[ 978], unit_i[ 980], unit_i[ 982], unit_i[ 984], unit_i[ 986], unit_i[ 988], unit_i[ 990], unit_i[ 992], unit_i[ 994], unit_i[ 996], unit_i[ 998], unit_i[1000], unit_i[1002], unit_i[1004], unit_i[1006]};
assign ecc_calc_w[ 1] = ^{unit_i[   0], unit_i[   2], unit_i[   3], unit_i[   5], unit_i[   6], unit_i[   9], unit_i[  10], unit_i[  12], unit_i[  13], unit_i[  16], unit_i[  17], unit_i[  20], unit_i[  21], unit_i[  24], unit_i[  25], unit_i[  27], unit_i[  28], unit_i[  31], unit_i[  32], unit_i[  35], unit_i[  36], unit_i[  39], unit_i[  40], unit_i[  43], unit_i[  44], unit_i[  47], unit_i[  48], unit_i[  51], unit_i[  52], unit_i[  55], unit_i[  56], unit_i[  58], unit_i[  59], unit_i[  62], unit_i[  63], unit_i[  66], unit_i[  67], unit_i[  70], unit_i[  71], unit_i[  74], unit_i[  75], unit_i[  78], unit_i[  79], unit_i[  82], unit_i[  83], unit_i[  86], unit_i[  87], unit_i[  90], unit_i[  91], unit_i[  94], unit_i[  95], unit_i[  98], unit_i[  99], unit_i[ 102], unit_i[ 103], unit_i[ 106], unit_i[ 107], unit_i[ 110], unit_i[ 111], unit_i[ 114], unit_i[ 115], unit_i[ 118], unit_i[ 119], unit_i[ 121], unit_i[ 122], unit_i[ 125], unit_i[ 126], unit_i[ 129], unit_i[ 130], unit_i[ 133], unit_i[ 134], unit_i[ 137], unit_i[ 138], unit_i[ 141], unit_i[ 142], unit_i[ 145], unit_i[ 146], unit_i[ 149], unit_i[ 150], unit_i[ 153], unit_i[ 154], unit_i[ 157], unit_i[ 158], unit_i[ 161], unit_i[ 162], unit_i[ 165], unit_i[ 166], unit_i[ 169], unit_i[ 170], unit_i[ 173], unit_i[ 174], unit_i[ 177], unit_i[ 178], unit_i[ 181], unit_i[ 182], unit_i[ 185], unit_i[ 186], unit_i[ 189], unit_i[ 190], unit_i[ 193], unit_i[ 194], unit_i[ 197], unit_i[ 198], unit_i[ 201], unit_i[ 202], unit_i[ 205], unit_i[ 206], unit_i[ 209], unit_i[ 210], unit_i[ 213], unit_i[ 214], unit_i[ 217], unit_i[ 218], unit_i[ 221], unit_i[ 222], unit_i[ 225], unit_i[ 226], unit_i[ 229], unit_i[ 230], unit_i[ 233], unit_i[ 234], unit_i[ 237], unit_i[ 238], unit_i[ 241], unit_i[ 242], unit_i[ 245], unit_i[ 246], unit_i[ 248], unit_i[ 249], unit_i[ 252], unit_i[ 253], unit_i[ 256], unit_i[ 257], unit_i[ 260], unit_i[ 261], unit_i[ 264], unit_i[ 265], unit_i[ 268], unit_i[ 269], unit_i[ 272], unit_i[ 273], unit_i[ 276], unit_i[ 277], unit_i[ 280], unit_i[ 281], unit_i[ 284], unit_i[ 285], unit_i[ 288], unit_i[ 289], unit_i[ 292], unit_i[ 293], unit_i[ 296], unit_i[ 297], unit_i[ 300], unit_i[ 301], unit_i[ 304], unit_i[ 305], unit_i[ 308], unit_i[ 309], unit_i[ 312], unit_i[ 313], unit_i[ 316], unit_i[ 317], unit_i[ 320], unit_i[ 321], unit_i[ 324], unit_i[ 325], unit_i[ 328], unit_i[ 329], unit_i[ 332], unit_i[ 333], unit_i[ 336], unit_i[ 337], unit_i[ 340], unit_i[ 341], unit_i[ 344], unit_i[ 345], unit_i[ 348], unit_i[ 349], unit_i[ 352], unit_i[ 353], unit_i[ 356], unit_i[ 357], unit_i[ 360], unit_i[ 361], unit_i[ 364], unit_i[ 365], unit_i[ 368], unit_i[ 369], unit_i[ 372], unit_i[ 373], unit_i[ 376], unit_i[ 377], unit_i[ 380], unit_i[ 381], unit_i[ 384], unit_i[ 385], unit_i[ 388], unit_i[ 389], unit_i[ 392], unit_i[ 393], unit_i[ 396], unit_i[ 397], unit_i[ 400], unit_i[ 401], unit_i[ 404], unit_i[ 405], unit_i[ 408], unit_i[ 409], unit_i[ 412], unit_i[ 413], unit_i[ 416], unit_i[ 417], unit_i[ 420], unit_i[ 421], unit_i[ 424], unit_i[ 425], unit_i[ 428], unit_i[ 429], unit_i[ 432], unit_i[ 433], unit_i[ 436], unit_i[ 437], unit_i[ 440], unit_i[ 441], unit_i[ 444], unit_i[ 445], unit_i[ 448], unit_i[ 449], unit_i[ 452], unit_i[ 453], unit_i[ 456], unit_i[ 457], unit_i[ 460], unit_i[ 461], unit_i[ 464], unit_i[ 465], unit_i[ 468], unit_i[ 469], unit_i[ 472], unit_i[ 473], unit_i[ 476], unit_i[ 477], unit_i[ 480], unit_i[ 481], unit_i[ 484], unit_i[ 485], unit_i[ 488], unit_i[ 489], unit_i[ 492], unit_i[ 493], unit_i[ 496], unit_i[ 497], unit_i[ 500], unit_i[ 501], unit_i[ 503], unit_i[ 504], unit_i[ 507], unit_i[ 508], unit_i[ 511], unit_i[ 512], unit_i[ 515], unit_i[ 516], unit_i[ 519], unit_i[ 520], unit_i[ 523], unit_i[ 524], unit_i[ 527], unit_i[ 528], unit_i[ 531], unit_i[ 532], unit_i[ 535], unit_i[ 536], unit_i[ 539], unit_i[ 540], unit_i[ 543], unit_i[ 544], unit_i[ 547], unit_i[ 548], unit_i[ 551], unit_i[ 552], unit_i[ 555], unit_i[ 556], unit_i[ 559], unit_i[ 560], unit_i[ 563], unit_i[ 564], unit_i[ 567], unit_i[ 568], unit_i[ 571], unit_i[ 572], unit_i[ 575], unit_i[ 576], unit_i[ 579], unit_i[ 580], unit_i[ 583], unit_i[ 584], unit_i[ 587], unit_i[ 588], unit_i[ 591], unit_i[ 592], unit_i[ 595], unit_i[ 596], unit_i[ 599], unit_i[ 600], unit_i[ 603], unit_i[ 604], unit_i[ 607], unit_i[ 608], unit_i[ 611], unit_i[ 612], unit_i[ 615], unit_i[ 616], unit_i[ 619], unit_i[ 620], unit_i[ 623], unit_i[ 624], unit_i[ 627], unit_i[ 628], unit_i[ 631], unit_i[ 632], unit_i[ 635], unit_i[ 636], unit_i[ 639], unit_i[ 640], unit_i[ 643], unit_i[ 644], unit_i[ 647], unit_i[ 648], unit_i[ 651], unit_i[ 652], unit_i[ 655], unit_i[ 656], unit_i[ 659], unit_i[ 660], unit_i[ 663], unit_i[ 664], unit_i[ 667], unit_i[ 668], unit_i[ 671], unit_i[ 672], unit_i[ 675], unit_i[ 676], unit_i[ 679], unit_i[ 680], unit_i[ 683], unit_i[ 684], unit_i[ 687], unit_i[ 688], unit_i[ 691], unit_i[ 692], unit_i[ 695], unit_i[ 696], unit_i[ 699], unit_i[ 700], unit_i[ 703], unit_i[ 704], unit_i[ 707], unit_i[ 708], unit_i[ 711], unit_i[ 712], unit_i[ 715], unit_i[ 716], unit_i[ 719], unit_i[ 720], unit_i[ 723], unit_i[ 724], unit_i[ 727], unit_i[ 728], unit_i[ 731], unit_i[ 732], unit_i[ 735], unit_i[ 736], unit_i[ 739], unit_i[ 740], unit_i[ 743], unit_i[ 744], unit_i[ 747], unit_i[ 748], unit_i[ 751], unit_i[ 752], unit_i[ 755], unit_i[ 756], unit_i[ 759], unit_i[ 760], unit_i[ 763], unit_i[ 764], unit_i[ 767], unit_i[ 768], unit_i[ 771], unit_i[ 772], unit_i[ 775], unit_i[ 776], unit_i[ 779], unit_i[ 780], unit_i[ 783], unit_i[ 784], unit_i[ 787], unit_i[ 788], unit_i[ 791], unit_i[ 792], unit_i[ 795], unit_i[ 796], unit_i[ 799], unit_i[ 800], unit_i[ 803], unit_i[ 804], unit_i[ 807], unit_i[ 808], unit_i[ 811], unit_i[ 812], unit_i[ 815], unit_i[ 816], unit_i[ 819], unit_i[ 820], unit_i[ 823], unit_i[ 824], unit_i[ 827], unit_i[ 828], unit_i[ 831], unit_i[ 832], unit_i[ 835], unit_i[ 836], unit_i[ 839], unit_i[ 840], unit_i[ 843], unit_i[ 844], unit_i[ 847], unit_i[ 848], unit_i[ 851], unit_i[ 852], unit_i[ 855], unit_i[ 856], unit_i[ 859], unit_i[ 860], unit_i[ 863], unit_i[ 864], unit_i[ 867], unit_i[ 868], unit_i[ 871], unit_i[ 872], unit_i[ 875], unit_i[ 876], unit_i[ 879], unit_i[ 880], unit_i[ 883], unit_i[ 884], unit_i[ 887], unit_i[ 888], unit_i[ 891], unit_i[ 892], unit_i[ 895], unit_i[ 896], unit_i[ 899], unit_i[ 900], unit_i[ 903], unit_i[ 904], unit_i[ 907], unit_i[ 908], unit_i[ 911], unit_i[ 912], unit_i[ 915], unit_i[ 916], unit_i[ 919], unit_i[ 920], unit_i[ 923], unit_i[ 924], unit_i[ 927], unit_i[ 928], unit_i[ 931], unit_i[ 932], unit_i[ 935], unit_i[ 936], unit_i[ 939], unit_i[ 940], unit_i[ 943], unit_i[ 944], unit_i[ 947], unit_i[ 948], unit_i[ 951], unit_i[ 952], unit_i[ 955], unit_i[ 956], unit_i[ 959], unit_i[ 960], unit_i[ 963], unit_i[ 964], unit_i[ 967], unit_i[ 968], unit_i[ 971], unit_i[ 972], unit_i[ 975], unit_i[ 976], unit_i[ 979], unit_i[ 980], unit_i[ 983], unit_i[ 984], unit_i[ 987], unit_i[ 988], unit_i[ 991], unit_i[ 992], unit_i[ 995], unit_i[ 996], unit_i[ 999], unit_i[1000], unit_i[1003], unit_i[1004], unit_i[1007]};
assign ecc_calc_w[ 2] = ^{unit_i[   1], unit_i[   2], unit_i[   3], unit_i[   7], unit_i[   8], unit_i[   9], unit_i[  10], unit_i[  14], unit_i[  15], unit_i[  16], unit_i[  17], unit_i[  22], unit_i[  23], unit_i[  24], unit_i[  25], unit_i[  29], unit_i[  30], unit_i[  31], unit_i[  32], unit_i[  37], unit_i[  38], unit_i[  39], unit_i[  40], unit_i[  45], unit_i[  46], unit_i[  47], unit_i[  48], unit_i[  53], unit_i[  54], unit_i[  55], unit_i[  56], unit_i[  60], unit_i[  61], unit_i[  62], unit_i[  63], unit_i[  68], unit_i[  69], unit_i[  70], unit_i[  71], unit_i[  76], unit_i[  77], unit_i[  78], unit_i[  79], unit_i[  84], unit_i[  85], unit_i[  86], unit_i[  87], unit_i[  92], unit_i[  93], unit_i[  94], unit_i[  95], unit_i[ 100], unit_i[ 101], unit_i[ 102], unit_i[ 103], unit_i[ 108], unit_i[ 109], unit_i[ 110], unit_i[ 111], unit_i[ 116], unit_i[ 117], unit_i[ 118], unit_i[ 119], unit_i[ 123], unit_i[ 124], unit_i[ 125], unit_i[ 126], unit_i[ 131], unit_i[ 132], unit_i[ 133], unit_i[ 134], unit_i[ 139], unit_i[ 140], unit_i[ 141], unit_i[ 142], unit_i[ 147], unit_i[ 148], unit_i[ 149], unit_i[ 150], unit_i[ 155], unit_i[ 156], unit_i[ 157], unit_i[ 158], unit_i[ 163], unit_i[ 164], unit_i[ 165], unit_i[ 166], unit_i[ 171], unit_i[ 172], unit_i[ 173], unit_i[ 174], unit_i[ 179], unit_i[ 180], unit_i[ 181], unit_i[ 182], unit_i[ 187], unit_i[ 188], unit_i[ 189], unit_i[ 190], unit_i[ 195], unit_i[ 196], unit_i[ 197], unit_i[ 198], unit_i[ 203], unit_i[ 204], unit_i[ 205], unit_i[ 206], unit_i[ 211], unit_i[ 212], unit_i[ 213], unit_i[ 214], unit_i[ 219], unit_i[ 220], unit_i[ 221], unit_i[ 222], unit_i[ 227], unit_i[ 228], unit_i[ 229], unit_i[ 230], unit_i[ 235], unit_i[ 236], unit_i[ 237], unit_i[ 238], unit_i[ 243], unit_i[ 244], unit_i[ 245], unit_i[ 246], unit_i[ 250], unit_i[ 251], unit_i[ 252], unit_i[ 253], unit_i[ 258], unit_i[ 259], unit_i[ 260], unit_i[ 261], unit_i[ 266], unit_i[ 267], unit_i[ 268], unit_i[ 269], unit_i[ 274], unit_i[ 275], unit_i[ 276], unit_i[ 277], unit_i[ 282], unit_i[ 283], unit_i[ 284], unit_i[ 285], unit_i[ 290], unit_i[ 291], unit_i[ 292], unit_i[ 293], unit_i[ 298], unit_i[ 299], unit_i[ 300], unit_i[ 301], unit_i[ 306], unit_i[ 307], unit_i[ 308], unit_i[ 309], unit_i[ 314], unit_i[ 315], unit_i[ 316], unit_i[ 317], unit_i[ 322], unit_i[ 323], unit_i[ 324], unit_i[ 325], unit_i[ 330], unit_i[ 331], unit_i[ 332], unit_i[ 333], unit_i[ 338], unit_i[ 339], unit_i[ 340], unit_i[ 341], unit_i[ 346], unit_i[ 347], unit_i[ 348], unit_i[ 349], unit_i[ 354], unit_i[ 355], unit_i[ 356], unit_i[ 357], unit_i[ 362], unit_i[ 363], unit_i[ 364], unit_i[ 365], unit_i[ 370], unit_i[ 371], unit_i[ 372], unit_i[ 373], unit_i[ 378], unit_i[ 379], unit_i[ 380], unit_i[ 381], unit_i[ 386], unit_i[ 387], unit_i[ 388], unit_i[ 389], unit_i[ 394], unit_i[ 395], unit_i[ 396], unit_i[ 397], unit_i[ 402], unit_i[ 403], unit_i[ 404], unit_i[ 405], unit_i[ 410], unit_i[ 411], unit_i[ 412], unit_i[ 413], unit_i[ 418], unit_i[ 419], unit_i[ 420], unit_i[ 421], unit_i[ 426], unit_i[ 427], unit_i[ 428], unit_i[ 429], unit_i[ 434], unit_i[ 435], unit_i[ 436], unit_i[ 437], unit_i[ 442], unit_i[ 443], unit_i[ 444], unit_i[ 445], unit_i[ 450], unit_i[ 451], unit_i[ 452], unit_i[ 453], unit_i[ 458], unit_i[ 459], unit_i[ 460], unit_i[ 461], unit_i[ 466], unit_i[ 467], unit_i[ 468], unit_i[ 469], unit_i[ 474], unit_i[ 475], unit_i[ 476], unit_i[ 477], unit_i[ 482], unit_i[ 483], unit_i[ 484], unit_i[ 485], unit_i[ 490], unit_i[ 491], unit_i[ 492], unit_i[ 493], unit_i[ 498], unit_i[ 499], unit_i[ 500], unit_i[ 501], unit_i[ 505], unit_i[ 506], unit_i[ 507], unit_i[ 508], unit_i[ 513], unit_i[ 514], unit_i[ 515], unit_i[ 516], unit_i[ 521], unit_i[ 522], unit_i[ 523], unit_i[ 524], unit_i[ 529], unit_i[ 530], unit_i[ 531], unit_i[ 532], unit_i[ 537], unit_i[ 538], unit_i[ 539], unit_i[ 540], unit_i[ 545], unit_i[ 546], unit_i[ 547], unit_i[ 548], unit_i[ 553], unit_i[ 554], unit_i[ 555], unit_i[ 556], unit_i[ 561], unit_i[ 562], unit_i[ 563], unit_i[ 564], unit_i[ 569], unit_i[ 570], unit_i[ 571], unit_i[ 572], unit_i[ 577], unit_i[ 578], unit_i[ 579], unit_i[ 580], unit_i[ 585], unit_i[ 586], unit_i[ 587], unit_i[ 588], unit_i[ 593], unit_i[ 594], unit_i[ 595], unit_i[ 596], unit_i[ 601], unit_i[ 602], unit_i[ 603], unit_i[ 604], unit_i[ 609], unit_i[ 610], unit_i[ 611], unit_i[ 612], unit_i[ 617], unit_i[ 618], unit_i[ 619], unit_i[ 620], unit_i[ 625], unit_i[ 626], unit_i[ 627], unit_i[ 628], unit_i[ 633], unit_i[ 634], unit_i[ 635], unit_i[ 636], unit_i[ 641], unit_i[ 642], unit_i[ 643], unit_i[ 644], unit_i[ 649], unit_i[ 650], unit_i[ 651], unit_i[ 652], unit_i[ 657], unit_i[ 658], unit_i[ 659], unit_i[ 660], unit_i[ 665], unit_i[ 666], unit_i[ 667], unit_i[ 668], unit_i[ 673], unit_i[ 674], unit_i[ 675], unit_i[ 676], unit_i[ 681], unit_i[ 682], unit_i[ 683], unit_i[ 684], unit_i[ 689], unit_i[ 690], unit_i[ 691], unit_i[ 692], unit_i[ 697], unit_i[ 698], unit_i[ 699], unit_i[ 700], unit_i[ 705], unit_i[ 706], unit_i[ 707], unit_i[ 708], unit_i[ 713], unit_i[ 714], unit_i[ 715], unit_i[ 716], unit_i[ 721], unit_i[ 722], unit_i[ 723], unit_i[ 724], unit_i[ 729], unit_i[ 730], unit_i[ 731], unit_i[ 732], unit_i[ 737], unit_i[ 738], unit_i[ 739], unit_i[ 740], unit_i[ 745], unit_i[ 746], unit_i[ 747], unit_i[ 748], unit_i[ 753], unit_i[ 754], unit_i[ 755], unit_i[ 756], unit_i[ 761], unit_i[ 762], unit_i[ 763], unit_i[ 764], unit_i[ 769], unit_i[ 770], unit_i[ 771], unit_i[ 772], unit_i[ 777], unit_i[ 778], unit_i[ 779], unit_i[ 780], unit_i[ 785], unit_i[ 786], unit_i[ 787], unit_i[ 788], unit_i[ 793], unit_i[ 794], unit_i[ 795], unit_i[ 796], unit_i[ 801], unit_i[ 802], unit_i[ 803], unit_i[ 804], unit_i[ 809], unit_i[ 810], unit_i[ 811], unit_i[ 812], unit_i[ 817], unit_i[ 818], unit_i[ 819], unit_i[ 820], unit_i[ 825], unit_i[ 826], unit_i[ 827], unit_i[ 828], unit_i[ 833], unit_i[ 834], unit_i[ 835], unit_i[ 836], unit_i[ 841], unit_i[ 842], unit_i[ 843], unit_i[ 844], unit_i[ 849], unit_i[ 850], unit_i[ 851], unit_i[ 852], unit_i[ 857], unit_i[ 858], unit_i[ 859], unit_i[ 860], unit_i[ 865], unit_i[ 866], unit_i[ 867], unit_i[ 868], unit_i[ 873], unit_i[ 874], unit_i[ 875], unit_i[ 876], unit_i[ 881], unit_i[ 882], unit_i[ 883], unit_i[ 884], unit_i[ 889], unit_i[ 890], unit_i[ 891], unit_i[ 892], unit_i[ 897], unit_i[ 898], unit_i[ 899], unit_i[ 900], unit_i[ 905], unit_i[ 906], unit_i[ 907], unit_i[ 908], unit_i[ 913], unit_i[ 914], unit_i[ 915], unit_i[ 916], unit_i[ 921], unit_i[ 922], unit_i[ 923], unit_i[ 924], unit_i[ 929], unit_i[ 930], unit_i[ 931], unit_i[ 932], unit_i[ 937], unit_i[ 938], unit_i[ 939], unit_i[ 940], unit_i[ 945], unit_i[ 946], unit_i[ 947], unit_i[ 948], unit_i[ 953], unit_i[ 954], unit_i[ 955], unit_i[ 956], unit_i[ 961], unit_i[ 962], unit_i[ 963], unit_i[ 964], unit_i[ 969], unit_i[ 970], unit_i[ 971], unit_i[ 972], unit_i[ 977], unit_i[ 978], unit_i[ 979], unit_i[ 980], unit_i[ 985], unit_i[ 986], unit_i[ 987], unit_i[ 988], unit_i[ 993], unit_i[ 994], unit_i[ 995], unit_i[ 996], unit_i[1001], unit_i[1002], unit_i[1003], unit_i[1004]};
assign ecc_calc_w[ 3] = ^{unit_i[   4], unit_i[   5], unit_i[   6], unit_i[   7], unit_i[   8], unit_i[   9], unit_i[  10], unit_i[  18], unit_i[  19], unit_i[  20], unit_i[  21], unit_i[  22], unit_i[  23], unit_i[  24], unit_i[  25], unit_i[  33], unit_i[  34], unit_i[  35], unit_i[  36], unit_i[  37], unit_i[  38], unit_i[  39], unit_i[  40], unit_i[  49], unit_i[  50], unit_i[  51], unit_i[  52], unit_i[  53], unit_i[  54], unit_i[  55], unit_i[  56], unit_i[  64], unit_i[  65], unit_i[  66], unit_i[  67], unit_i[  68], unit_i[  69], unit_i[  70], unit_i[  71], unit_i[  80], unit_i[  81], unit_i[  82], unit_i[  83], unit_i[  84], unit_i[  85], unit_i[  86], unit_i[  87], unit_i[  96], unit_i[  97], unit_i[  98], unit_i[  99], unit_i[ 100], unit_i[ 101], unit_i[ 102], unit_i[ 103], unit_i[ 112], unit_i[ 113], unit_i[ 114], unit_i[ 115], unit_i[ 116], unit_i[ 117], unit_i[ 118], unit_i[ 119], unit_i[ 127], unit_i[ 128], unit_i[ 129], unit_i[ 130], unit_i[ 131], unit_i[ 132], unit_i[ 133], unit_i[ 134], unit_i[ 143], unit_i[ 144], unit_i[ 145], unit_i[ 146], unit_i[ 147], unit_i[ 148], unit_i[ 149], unit_i[ 150], unit_i[ 159], unit_i[ 160], unit_i[ 161], unit_i[ 162], unit_i[ 163], unit_i[ 164], unit_i[ 165], unit_i[ 166], unit_i[ 175], unit_i[ 176], unit_i[ 177], unit_i[ 178], unit_i[ 179], unit_i[ 180], unit_i[ 181], unit_i[ 182], unit_i[ 191], unit_i[ 192], unit_i[ 193], unit_i[ 194], unit_i[ 195], unit_i[ 196], unit_i[ 197], unit_i[ 198], unit_i[ 207], unit_i[ 208], unit_i[ 209], unit_i[ 210], unit_i[ 211], unit_i[ 212], unit_i[ 213], unit_i[ 214], unit_i[ 223], unit_i[ 224], unit_i[ 225], unit_i[ 226], unit_i[ 227], unit_i[ 228], unit_i[ 229], unit_i[ 230], unit_i[ 239], unit_i[ 240], unit_i[ 241], unit_i[ 242], unit_i[ 243], unit_i[ 244], unit_i[ 245], unit_i[ 246], unit_i[ 254], unit_i[ 255], unit_i[ 256], unit_i[ 257], unit_i[ 258], unit_i[ 259], unit_i[ 260], unit_i[ 261], unit_i[ 270], unit_i[ 271], unit_i[ 272], unit_i[ 273], unit_i[ 274], unit_i[ 275], unit_i[ 276], unit_i[ 277], unit_i[ 286], unit_i[ 287], unit_i[ 288], unit_i[ 289], unit_i[ 290], unit_i[ 291], unit_i[ 292], unit_i[ 293], unit_i[ 302], unit_i[ 303], unit_i[ 304], unit_i[ 305], unit_i[ 306], unit_i[ 307], unit_i[ 308], unit_i[ 309], unit_i[ 318], unit_i[ 319], unit_i[ 320], unit_i[ 321], unit_i[ 322], unit_i[ 323], unit_i[ 324], unit_i[ 325], unit_i[ 334], unit_i[ 335], unit_i[ 336], unit_i[ 337], unit_i[ 338], unit_i[ 339], unit_i[ 340], unit_i[ 341], unit_i[ 350], unit_i[ 351], unit_i[ 352], unit_i[ 353], unit_i[ 354], unit_i[ 355], unit_i[ 356], unit_i[ 357], unit_i[ 366], unit_i[ 367], unit_i[ 368], unit_i[ 369], unit_i[ 370], unit_i[ 371], unit_i[ 372], unit_i[ 373], unit_i[ 382], unit_i[ 383], unit_i[ 384], unit_i[ 385], unit_i[ 386], unit_i[ 387], unit_i[ 388], unit_i[ 389], unit_i[ 398], unit_i[ 399], unit_i[ 400], unit_i[ 401], unit_i[ 402], unit_i[ 403], unit_i[ 404], unit_i[ 405], unit_i[ 414], unit_i[ 415], unit_i[ 416], unit_i[ 417], unit_i[ 418], unit_i[ 419], unit_i[ 420], unit_i[ 421], unit_i[ 430], unit_i[ 431], unit_i[ 432], unit_i[ 433], unit_i[ 434], unit_i[ 435], unit_i[ 436], unit_i[ 437], unit_i[ 446], unit_i[ 447], unit_i[ 448], unit_i[ 449], unit_i[ 450], unit_i[ 451], unit_i[ 452], unit_i[ 453], unit_i[ 462], unit_i[ 463], unit_i[ 464], unit_i[ 465], unit_i[ 466], unit_i[ 467], unit_i[ 468], unit_i[ 469], unit_i[ 478], unit_i[ 479], unit_i[ 480], unit_i[ 481], unit_i[ 482], unit_i[ 483], unit_i[ 484], unit_i[ 485], unit_i[ 494], unit_i[ 495], unit_i[ 496], unit_i[ 497], unit_i[ 498], unit_i[ 499], unit_i[ 500], unit_i[ 501], unit_i[ 509], unit_i[ 510], unit_i[ 511], unit_i[ 512], unit_i[ 513], unit_i[ 514], unit_i[ 515], unit_i[ 516], unit_i[ 525], unit_i[ 526], unit_i[ 527], unit_i[ 528], unit_i[ 529], unit_i[ 530], unit_i[ 531], unit_i[ 532], unit_i[ 541], unit_i[ 542], unit_i[ 543], unit_i[ 544], unit_i[ 545], unit_i[ 546], unit_i[ 547], unit_i[ 548], unit_i[ 557], unit_i[ 558], unit_i[ 559], unit_i[ 560], unit_i[ 561], unit_i[ 562], unit_i[ 563], unit_i[ 564], unit_i[ 573], unit_i[ 574], unit_i[ 575], unit_i[ 576], unit_i[ 577], unit_i[ 578], unit_i[ 579], unit_i[ 580], unit_i[ 589], unit_i[ 590], unit_i[ 591], unit_i[ 592], unit_i[ 593], unit_i[ 594], unit_i[ 595], unit_i[ 596], unit_i[ 605], unit_i[ 606], unit_i[ 607], unit_i[ 608], unit_i[ 609], unit_i[ 610], unit_i[ 611], unit_i[ 612], unit_i[ 621], unit_i[ 622], unit_i[ 623], unit_i[ 624], unit_i[ 625], unit_i[ 626], unit_i[ 627], unit_i[ 628], unit_i[ 637], unit_i[ 638], unit_i[ 639], unit_i[ 640], unit_i[ 641], unit_i[ 642], unit_i[ 643], unit_i[ 644], unit_i[ 653], unit_i[ 654], unit_i[ 655], unit_i[ 656], unit_i[ 657], unit_i[ 658], unit_i[ 659], unit_i[ 660], unit_i[ 669], unit_i[ 670], unit_i[ 671], unit_i[ 672], unit_i[ 673], unit_i[ 674], unit_i[ 675], unit_i[ 676], unit_i[ 685], unit_i[ 686], unit_i[ 687], unit_i[ 688], unit_i[ 689], unit_i[ 690], unit_i[ 691], unit_i[ 692], unit_i[ 701], unit_i[ 702], unit_i[ 703], unit_i[ 704], unit_i[ 705], unit_i[ 706], unit_i[ 707], unit_i[ 708], unit_i[ 717], unit_i[ 718], unit_i[ 719], unit_i[ 720], unit_i[ 721], unit_i[ 722], unit_i[ 723], unit_i[ 724], unit_i[ 733], unit_i[ 734], unit_i[ 735], unit_i[ 736], unit_i[ 737], unit_i[ 738], unit_i[ 739], unit_i[ 740], unit_i[ 749], unit_i[ 750], unit_i[ 751], unit_i[ 752], unit_i[ 753], unit_i[ 754], unit_i[ 755], unit_i[ 756], unit_i[ 765], unit_i[ 766], unit_i[ 767], unit_i[ 768], unit_i[ 769], unit_i[ 770], unit_i[ 771], unit_i[ 772], unit_i[ 781], unit_i[ 782], unit_i[ 783], unit_i[ 784], unit_i[ 785], unit_i[ 786], unit_i[ 787], unit_i[ 788], unit_i[ 797], unit_i[ 798], unit_i[ 799], unit_i[ 800], unit_i[ 801], unit_i[ 802], unit_i[ 803], unit_i[ 804], unit_i[ 813], unit_i[ 814], unit_i[ 815], unit_i[ 816], unit_i[ 817], unit_i[ 818], unit_i[ 819], unit_i[ 820], unit_i[ 829], unit_i[ 830], unit_i[ 831], unit_i[ 832], unit_i[ 833], unit_i[ 834], unit_i[ 835], unit_i[ 836], unit_i[ 845], unit_i[ 846], unit_i[ 847], unit_i[ 848], unit_i[ 849], unit_i[ 850], unit_i[ 851], unit_i[ 852], unit_i[ 861], unit_i[ 862], unit_i[ 863], unit_i[ 864], unit_i[ 865], unit_i[ 866], unit_i[ 867], unit_i[ 868], unit_i[ 877], unit_i[ 878], unit_i[ 879], unit_i[ 880], unit_i[ 881], unit_i[ 882], unit_i[ 883], unit_i[ 884], unit_i[ 893], unit_i[ 894], unit_i[ 895], unit_i[ 896], unit_i[ 897], unit_i[ 898], unit_i[ 899], unit_i[ 900], unit_i[ 909], unit_i[ 910], unit_i[ 911], unit_i[ 912], unit_i[ 913], unit_i[ 914], unit_i[ 915], unit_i[ 916], unit_i[ 925], unit_i[ 926], unit_i[ 927], unit_i[ 928], unit_i[ 929], unit_i[ 930], unit_i[ 931], unit_i[ 932], unit_i[ 941], unit_i[ 942], unit_i[ 943], unit_i[ 944], unit_i[ 945], unit_i[ 946], unit_i[ 947], unit_i[ 948], unit_i[ 957], unit_i[ 958], unit_i[ 959], unit_i[ 960], unit_i[ 961], unit_i[ 962], unit_i[ 963], unit_i[ 964], unit_i[ 973], unit_i[ 974], unit_i[ 975], unit_i[ 976], unit_i[ 977], unit_i[ 978], unit_i[ 979], unit_i[ 980], unit_i[ 989], unit_i[ 990], unit_i[ 991], unit_i[ 992], unit_i[ 993], unit_i[ 994], unit_i[ 995], unit_i[ 996], unit_i[1005], unit_i[1006], unit_i[1007]};
assign ecc_calc_w[ 4] = ^{unit_i[  11], unit_i[  12], unit_i[  13], unit_i[  14], unit_i[  15], unit_i[  16], unit_i[  17], unit_i[  18], unit_i[  19], unit_i[  20], unit_i[  21], unit_i[  22], unit_i[  23], unit_i[  24], unit_i[  25], unit_i[  41], unit_i[  42], unit_i[  43], unit_i[  44], unit_i[  45], unit_i[  46], unit_i[  47], unit_i[  48], unit_i[  49], unit_i[  50], unit_i[  51], unit_i[  52], unit_i[  53], unit_i[  54], unit_i[  55], unit_i[  56], unit_i[  72], unit_i[  73], unit_i[  74], unit_i[  75], unit_i[  76], unit_i[  77], unit_i[  78], unit_i[  79], unit_i[  80], unit_i[  81], unit_i[  82], unit_i[  83], unit_i[  84], unit_i[  85], unit_i[  86], unit_i[  87], unit_i[ 104], unit_i[ 105], unit_i[ 106], unit_i[ 107], unit_i[ 108], unit_i[ 109], unit_i[ 110], unit_i[ 111], unit_i[ 112], unit_i[ 113], unit_i[ 114], unit_i[ 115], unit_i[ 116], unit_i[ 117], unit_i[ 118], unit_i[ 119], unit_i[ 135], unit_i[ 136], unit_i[ 137], unit_i[ 138], unit_i[ 139], unit_i[ 140], unit_i[ 141], unit_i[ 142], unit_i[ 143], unit_i[ 144], unit_i[ 145], unit_i[ 146], unit_i[ 147], unit_i[ 148], unit_i[ 149], unit_i[ 150], unit_i[ 167], unit_i[ 168], unit_i[ 169], unit_i[ 170], unit_i[ 171], unit_i[ 172], unit_i[ 173], unit_i[ 174], unit_i[ 175], unit_i[ 176], unit_i[ 177], unit_i[ 178], unit_i[ 179], unit_i[ 180], unit_i[ 181], unit_i[ 182], unit_i[ 199], unit_i[ 200], unit_i[ 201], unit_i[ 202], unit_i[ 203], unit_i[ 204], unit_i[ 205], unit_i[ 206], unit_i[ 207], unit_i[ 208], unit_i[ 209], unit_i[ 210], unit_i[ 211], unit_i[ 212], unit_i[ 213], unit_i[ 214], unit_i[ 231], unit_i[ 232], unit_i[ 233], unit_i[ 234], unit_i[ 235], unit_i[ 236], unit_i[ 237], unit_i[ 238], unit_i[ 239], unit_i[ 240], unit_i[ 241], unit_i[ 242], unit_i[ 243], unit_i[ 244], unit_i[ 245], unit_i[ 246], unit_i[ 262], unit_i[ 263], unit_i[ 264], unit_i[ 265], unit_i[ 266], unit_i[ 267], unit_i[ 268], unit_i[ 269], unit_i[ 270], unit_i[ 271], unit_i[ 272], unit_i[ 273], unit_i[ 274], unit_i[ 275], unit_i[ 276], unit_i[ 277], unit_i[ 294], unit_i[ 295], unit_i[ 296], unit_i[ 297], unit_i[ 298], unit_i[ 299], unit_i[ 300], unit_i[ 301], unit_i[ 302], unit_i[ 303], unit_i[ 304], unit_i[ 305], unit_i[ 306], unit_i[ 307], unit_i[ 308], unit_i[ 309], unit_i[ 326], unit_i[ 327], unit_i[ 328], unit_i[ 329], unit_i[ 330], unit_i[ 331], unit_i[ 332], unit_i[ 333], unit_i[ 334], unit_i[ 335], unit_i[ 336], unit_i[ 337], unit_i[ 338], unit_i[ 339], unit_i[ 340], unit_i[ 341], unit_i[ 358], unit_i[ 359], unit_i[ 360], unit_i[ 361], unit_i[ 362], unit_i[ 363], unit_i[ 364], unit_i[ 365], unit_i[ 366], unit_i[ 367], unit_i[ 368], unit_i[ 369], unit_i[ 370], unit_i[ 371], unit_i[ 372], unit_i[ 373], unit_i[ 390], unit_i[ 391], unit_i[ 392], unit_i[ 393], unit_i[ 394], unit_i[ 395], unit_i[ 396], unit_i[ 397], unit_i[ 398], unit_i[ 399], unit_i[ 400], unit_i[ 401], unit_i[ 402], unit_i[ 403], unit_i[ 404], unit_i[ 405], unit_i[ 422], unit_i[ 423], unit_i[ 424], unit_i[ 425], unit_i[ 426], unit_i[ 427], unit_i[ 428], unit_i[ 429], unit_i[ 430], unit_i[ 431], unit_i[ 432], unit_i[ 433], unit_i[ 434], unit_i[ 435], unit_i[ 436], unit_i[ 437], unit_i[ 454], unit_i[ 455], unit_i[ 456], unit_i[ 457], unit_i[ 458], unit_i[ 459], unit_i[ 460], unit_i[ 461], unit_i[ 462], unit_i[ 463], unit_i[ 464], unit_i[ 465], unit_i[ 466], unit_i[ 467], unit_i[ 468], unit_i[ 469], unit_i[ 486], unit_i[ 487], unit_i[ 488], unit_i[ 489], unit_i[ 490], unit_i[ 491], unit_i[ 492], unit_i[ 493], unit_i[ 494], unit_i[ 495], unit_i[ 496], unit_i[ 497], unit_i[ 498], unit_i[ 499], unit_i[ 500], unit_i[ 501], unit_i[ 517], unit_i[ 518], unit_i[ 519], unit_i[ 520], unit_i[ 521], unit_i[ 522], unit_i[ 523], unit_i[ 524], unit_i[ 525], unit_i[ 526], unit_i[ 527], unit_i[ 528], unit_i[ 529], unit_i[ 530], unit_i[ 531], unit_i[ 532], unit_i[ 549], unit_i[ 550], unit_i[ 551], unit_i[ 552], unit_i[ 553], unit_i[ 554], unit_i[ 555], unit_i[ 556], unit_i[ 557], unit_i[ 558], unit_i[ 559], unit_i[ 560], unit_i[ 561], unit_i[ 562], unit_i[ 563], unit_i[ 564], unit_i[ 581], unit_i[ 582], unit_i[ 583], unit_i[ 584], unit_i[ 585], unit_i[ 586], unit_i[ 587], unit_i[ 588], unit_i[ 589], unit_i[ 590], unit_i[ 591], unit_i[ 592], unit_i[ 593], unit_i[ 594], unit_i[ 595], unit_i[ 596], unit_i[ 613], unit_i[ 614], unit_i[ 615], unit_i[ 616], unit_i[ 617], unit_i[ 618], unit_i[ 619], unit_i[ 620], unit_i[ 621], unit_i[ 622], unit_i[ 623], unit_i[ 624], unit_i[ 625], unit_i[ 626], unit_i[ 627], unit_i[ 628], unit_i[ 645], unit_i[ 646], unit_i[ 647], unit_i[ 648], unit_i[ 649], unit_i[ 650], unit_i[ 651], unit_i[ 652], unit_i[ 653], unit_i[ 654], unit_i[ 655], unit_i[ 656], unit_i[ 657], unit_i[ 658], unit_i[ 659], unit_i[ 660], unit_i[ 677], unit_i[ 678], unit_i[ 679], unit_i[ 680], unit_i[ 681], unit_i[ 682], unit_i[ 683], unit_i[ 684], unit_i[ 685], unit_i[ 686], unit_i[ 687], unit_i[ 688], unit_i[ 689], unit_i[ 690], unit_i[ 691], unit_i[ 692], unit_i[ 709], unit_i[ 710], unit_i[ 711], unit_i[ 712], unit_i[ 713], unit_i[ 714], unit_i[ 715], unit_i[ 716], unit_i[ 717], unit_i[ 718], unit_i[ 719], unit_i[ 720], unit_i[ 721], unit_i[ 722], unit_i[ 723], unit_i[ 724], unit_i[ 741], unit_i[ 742], unit_i[ 743], unit_i[ 744], unit_i[ 745], unit_i[ 746], unit_i[ 747], unit_i[ 748], unit_i[ 749], unit_i[ 750], unit_i[ 751], unit_i[ 752], unit_i[ 753], unit_i[ 754], unit_i[ 755], unit_i[ 756], unit_i[ 773], unit_i[ 774], unit_i[ 775], unit_i[ 776], unit_i[ 777], unit_i[ 778], unit_i[ 779], unit_i[ 780], unit_i[ 781], unit_i[ 782], unit_i[ 783], unit_i[ 784], unit_i[ 785], unit_i[ 786], unit_i[ 787], unit_i[ 788], unit_i[ 805], unit_i[ 806], unit_i[ 807], unit_i[ 808], unit_i[ 809], unit_i[ 810], unit_i[ 811], unit_i[ 812], unit_i[ 813], unit_i[ 814], unit_i[ 815], unit_i[ 816], unit_i[ 817], unit_i[ 818], unit_i[ 819], unit_i[ 820], unit_i[ 837], unit_i[ 838], unit_i[ 839], unit_i[ 840], unit_i[ 841], unit_i[ 842], unit_i[ 843], unit_i[ 844], unit_i[ 845], unit_i[ 846], unit_i[ 847], unit_i[ 848], unit_i[ 849], unit_i[ 850], unit_i[ 851], unit_i[ 852], unit_i[ 869], unit_i[ 870], unit_i[ 871], unit_i[ 872], unit_i[ 873], unit_i[ 874], unit_i[ 875], unit_i[ 876], unit_i[ 877], unit_i[ 878], unit_i[ 879], unit_i[ 880], unit_i[ 881], unit_i[ 882], unit_i[ 883], unit_i[ 884], unit_i[ 901], unit_i[ 902], unit_i[ 903], unit_i[ 904], unit_i[ 905], unit_i[ 906], unit_i[ 907], unit_i[ 908], unit_i[ 909], unit_i[ 910], unit_i[ 911], unit_i[ 912], unit_i[ 913], unit_i[ 914], unit_i[ 915], unit_i[ 916], unit_i[ 933], unit_i[ 934], unit_i[ 935], unit_i[ 936], unit_i[ 937], unit_i[ 938], unit_i[ 939], unit_i[ 940], unit_i[ 941], unit_i[ 942], unit_i[ 943], unit_i[ 944], unit_i[ 945], unit_i[ 946], unit_i[ 947], unit_i[ 948], unit_i[ 965], unit_i[ 966], unit_i[ 967], unit_i[ 968], unit_i[ 969], unit_i[ 970], unit_i[ 971], unit_i[ 972], unit_i[ 973], unit_i[ 974], unit_i[ 975], unit_i[ 976], unit_i[ 977], unit_i[ 978], unit_i[ 979], unit_i[ 980], unit_i[ 997], unit_i[ 998], unit_i[ 999], unit_i[1000], unit_i[1001], unit_i[1002], unit_i[1003], unit_i[1004], unit_i[1005], unit_i[1006], unit_i[1007]};
assign ecc_calc_w[ 5] = ^{unit_i[  26], unit_i[  27], unit_i[  28], unit_i[  29], unit_i[  30], unit_i[  31], unit_i[  32], unit_i[  33], unit_i[  34], unit_i[  35], unit_i[  36], unit_i[  37], unit_i[  38], unit_i[  39], unit_i[  40], unit_i[  41], unit_i[  42], unit_i[  43], unit_i[  44], unit_i[  45], unit_i[  46], unit_i[  47], unit_i[  48], unit_i[  49], unit_i[  50], unit_i[  51], unit_i[  52], unit_i[  53], unit_i[  54], unit_i[  55], unit_i[  56], unit_i[  88], unit_i[  89], unit_i[  90], unit_i[  91], unit_i[  92], unit_i[  93], unit_i[  94], unit_i[  95], unit_i[  96], unit_i[  97], unit_i[  98], unit_i[  99], unit_i[ 100], unit_i[ 101], unit_i[ 102], unit_i[ 103], unit_i[ 104], unit_i[ 105], unit_i[ 106], unit_i[ 107], unit_i[ 108], unit_i[ 109], unit_i[ 110], unit_i[ 111], unit_i[ 112], unit_i[ 113], unit_i[ 114], unit_i[ 115], unit_i[ 116], unit_i[ 117], unit_i[ 118], unit_i[ 119], unit_i[ 151], unit_i[ 152], unit_i[ 153], unit_i[ 154], unit_i[ 155], unit_i[ 156], unit_i[ 157], unit_i[ 158], unit_i[ 159], unit_i[ 160], unit_i[ 161], unit_i[ 162], unit_i[ 163], unit_i[ 164], unit_i[ 165], unit_i[ 166], unit_i[ 167], unit_i[ 168], unit_i[ 169], unit_i[ 170], unit_i[ 171], unit_i[ 172], unit_i[ 173], unit_i[ 174], unit_i[ 175], unit_i[ 176], unit_i[ 177], unit_i[ 178], unit_i[ 179], unit_i[ 180], unit_i[ 181], unit_i[ 182], unit_i[ 215], unit_i[ 216], unit_i[ 217], unit_i[ 218], unit_i[ 219], unit_i[ 220], unit_i[ 221], unit_i[ 222], unit_i[ 223], unit_i[ 224], unit_i[ 225], unit_i[ 226], unit_i[ 227], unit_i[ 228], unit_i[ 229], unit_i[ 230], unit_i[ 231], unit_i[ 232], unit_i[ 233], unit_i[ 234], unit_i[ 235], unit_i[ 236], unit_i[ 237], unit_i[ 238], unit_i[ 239], unit_i[ 240], unit_i[ 241], unit_i[ 242], unit_i[ 243], unit_i[ 244], unit_i[ 245], unit_i[ 246], unit_i[ 278], unit_i[ 279], unit_i[ 280], unit_i[ 281], unit_i[ 282], unit_i[ 283], unit_i[ 284], unit_i[ 285], unit_i[ 286], unit_i[ 287], unit_i[ 288], unit_i[ 289], unit_i[ 290], unit_i[ 291], unit_i[ 292], unit_i[ 293], unit_i[ 294], unit_i[ 295], unit_i[ 296], unit_i[ 297], unit_i[ 298], unit_i[ 299], unit_i[ 300], unit_i[ 301], unit_i[ 302], unit_i[ 303], unit_i[ 304], unit_i[ 305], unit_i[ 306], unit_i[ 307], unit_i[ 308], unit_i[ 309], unit_i[ 342], unit_i[ 343], unit_i[ 344], unit_i[ 345], unit_i[ 346], unit_i[ 347], unit_i[ 348], unit_i[ 349], unit_i[ 350], unit_i[ 351], unit_i[ 352], unit_i[ 353], unit_i[ 354], unit_i[ 355], unit_i[ 356], unit_i[ 357], unit_i[ 358], unit_i[ 359], unit_i[ 360], unit_i[ 361], unit_i[ 362], unit_i[ 363], unit_i[ 364], unit_i[ 365], unit_i[ 366], unit_i[ 367], unit_i[ 368], unit_i[ 369], unit_i[ 370], unit_i[ 371], unit_i[ 372], unit_i[ 373], unit_i[ 406], unit_i[ 407], unit_i[ 408], unit_i[ 409], unit_i[ 410], unit_i[ 411], unit_i[ 412], unit_i[ 413], unit_i[ 414], unit_i[ 415], unit_i[ 416], unit_i[ 417], unit_i[ 418], unit_i[ 419], unit_i[ 420], unit_i[ 421], unit_i[ 422], unit_i[ 423], unit_i[ 424], unit_i[ 425], unit_i[ 426], unit_i[ 427], unit_i[ 428], unit_i[ 429], unit_i[ 430], unit_i[ 431], unit_i[ 432], unit_i[ 433], unit_i[ 434], unit_i[ 435], unit_i[ 436], unit_i[ 437], unit_i[ 470], unit_i[ 471], unit_i[ 472], unit_i[ 473], unit_i[ 474], unit_i[ 475], unit_i[ 476], unit_i[ 477], unit_i[ 478], unit_i[ 479], unit_i[ 480], unit_i[ 481], unit_i[ 482], unit_i[ 483], unit_i[ 484], unit_i[ 485], unit_i[ 486], unit_i[ 487], unit_i[ 488], unit_i[ 489], unit_i[ 490], unit_i[ 491], unit_i[ 492], unit_i[ 493], unit_i[ 494], unit_i[ 495], unit_i[ 496], unit_i[ 497], unit_i[ 498], unit_i[ 499], unit_i[ 500], unit_i[ 501], unit_i[ 533], unit_i[ 534], unit_i[ 535], unit_i[ 536], unit_i[ 537], unit_i[ 538], unit_i[ 539], unit_i[ 540], unit_i[ 541], unit_i[ 542], unit_i[ 543], unit_i[ 544], unit_i[ 545], unit_i[ 546], unit_i[ 547], unit_i[ 548], unit_i[ 549], unit_i[ 550], unit_i[ 551], unit_i[ 552], unit_i[ 553], unit_i[ 554], unit_i[ 555], unit_i[ 556], unit_i[ 557], unit_i[ 558], unit_i[ 559], unit_i[ 560], unit_i[ 561], unit_i[ 562], unit_i[ 563], unit_i[ 564], unit_i[ 597], unit_i[ 598], unit_i[ 599], unit_i[ 600], unit_i[ 601], unit_i[ 602], unit_i[ 603], unit_i[ 604], unit_i[ 605], unit_i[ 606], unit_i[ 607], unit_i[ 608], unit_i[ 609], unit_i[ 610], unit_i[ 611], unit_i[ 612], unit_i[ 613], unit_i[ 614], unit_i[ 615], unit_i[ 616], unit_i[ 617], unit_i[ 618], unit_i[ 619], unit_i[ 620], unit_i[ 621], unit_i[ 622], unit_i[ 623], unit_i[ 624], unit_i[ 625], unit_i[ 626], unit_i[ 627], unit_i[ 628], unit_i[ 661], unit_i[ 662], unit_i[ 663], unit_i[ 664], unit_i[ 665], unit_i[ 666], unit_i[ 667], unit_i[ 668], unit_i[ 669], unit_i[ 670], unit_i[ 671], unit_i[ 672], unit_i[ 673], unit_i[ 674], unit_i[ 675], unit_i[ 676], unit_i[ 677], unit_i[ 678], unit_i[ 679], unit_i[ 680], unit_i[ 681], unit_i[ 682], unit_i[ 683], unit_i[ 684], unit_i[ 685], unit_i[ 686], unit_i[ 687], unit_i[ 688], unit_i[ 689], unit_i[ 690], unit_i[ 691], unit_i[ 692], unit_i[ 725], unit_i[ 726], unit_i[ 727], unit_i[ 728], unit_i[ 729], unit_i[ 730], unit_i[ 731], unit_i[ 732], unit_i[ 733], unit_i[ 734], unit_i[ 735], unit_i[ 736], unit_i[ 737], unit_i[ 738], unit_i[ 739], unit_i[ 740], unit_i[ 741], unit_i[ 742], unit_i[ 743], unit_i[ 744], unit_i[ 745], unit_i[ 746], unit_i[ 747], unit_i[ 748], unit_i[ 749], unit_i[ 750], unit_i[ 751], unit_i[ 752], unit_i[ 753], unit_i[ 754], unit_i[ 755], unit_i[ 756], unit_i[ 789], unit_i[ 790], unit_i[ 791], unit_i[ 792], unit_i[ 793], unit_i[ 794], unit_i[ 795], unit_i[ 796], unit_i[ 797], unit_i[ 798], unit_i[ 799], unit_i[ 800], unit_i[ 801], unit_i[ 802], unit_i[ 803], unit_i[ 804], unit_i[ 805], unit_i[ 806], unit_i[ 807], unit_i[ 808], unit_i[ 809], unit_i[ 810], unit_i[ 811], unit_i[ 812], unit_i[ 813], unit_i[ 814], unit_i[ 815], unit_i[ 816], unit_i[ 817], unit_i[ 818], unit_i[ 819], unit_i[ 820], unit_i[ 853], unit_i[ 854], unit_i[ 855], unit_i[ 856], unit_i[ 857], unit_i[ 858], unit_i[ 859], unit_i[ 860], unit_i[ 861], unit_i[ 862], unit_i[ 863], unit_i[ 864], unit_i[ 865], unit_i[ 866], unit_i[ 867], unit_i[ 868], unit_i[ 869], unit_i[ 870], unit_i[ 871], unit_i[ 872], unit_i[ 873], unit_i[ 874], unit_i[ 875], unit_i[ 876], unit_i[ 877], unit_i[ 878], unit_i[ 879], unit_i[ 880], unit_i[ 881], unit_i[ 882], unit_i[ 883], unit_i[ 884], unit_i[ 917], unit_i[ 918], unit_i[ 919], unit_i[ 920], unit_i[ 921], unit_i[ 922], unit_i[ 923], unit_i[ 924], unit_i[ 925], unit_i[ 926], unit_i[ 927], unit_i[ 928], unit_i[ 929], unit_i[ 930], unit_i[ 931], unit_i[ 932], unit_i[ 933], unit_i[ 934], unit_i[ 935], unit_i[ 936], unit_i[ 937], unit_i[ 938], unit_i[ 939], unit_i[ 940], unit_i[ 941], unit_i[ 942], unit_i[ 943], unit_i[ 944], unit_i[ 945], unit_i[ 946], unit_i[ 947], unit_i[ 948], unit_i[ 981], unit_i[ 982], unit_i[ 983], unit_i[ 984], unit_i[ 985], unit_i[ 986], unit_i[ 987], unit_i[ 988], unit_i[ 989], unit_i[ 990], unit_i[ 991], unit_i[ 992], unit_i[ 993], unit_i[ 994], unit_i[ 995], unit_i[ 996], unit_i[ 997], unit_i[ 998], unit_i[ 999], unit_i[1000], unit_i[1001], unit_i[1002], unit_i[1003], unit_i[1004], unit_i[1005], unit_i[1006], unit_i[1007]};
assign ecc_calc_w[ 6] = ^{unit_i[  57], unit_i[  58], unit_i[  59], unit_i[  60], unit_i[  61], unit_i[  62], unit_i[  63], unit_i[  64], unit_i[  65], unit_i[  66], unit_i[  67], unit_i[  68], unit_i[  69], unit_i[  70], unit_i[  71], unit_i[  72], unit_i[  73], unit_i[  74], unit_i[  75], unit_i[  76], unit_i[  77], unit_i[  78], unit_i[  79], unit_i[  80], unit_i[  81], unit_i[  82], unit_i[  83], unit_i[  84], unit_i[  85], unit_i[  86], unit_i[  87], unit_i[  88], unit_i[  89], unit_i[  90], unit_i[  91], unit_i[  92], unit_i[  93], unit_i[  94], unit_i[  95], unit_i[  96], unit_i[  97], unit_i[  98], unit_i[  99], unit_i[ 100], unit_i[ 101], unit_i[ 102], unit_i[ 103], unit_i[ 104], unit_i[ 105], unit_i[ 106], unit_i[ 107], unit_i[ 108], unit_i[ 109], unit_i[ 110], unit_i[ 111], unit_i[ 112], unit_i[ 113], unit_i[ 114], unit_i[ 115], unit_i[ 116], unit_i[ 117], unit_i[ 118], unit_i[ 119], unit_i[ 183], unit_i[ 184], unit_i[ 185], unit_i[ 186], unit_i[ 187], unit_i[ 188], unit_i[ 189], unit_i[ 190], unit_i[ 191], unit_i[ 192], unit_i[ 193], unit_i[ 194], unit_i[ 195], unit_i[ 196], unit_i[ 197], unit_i[ 198], unit_i[ 199], unit_i[ 200], unit_i[ 201], unit_i[ 202], unit_i[ 203], unit_i[ 204], unit_i[ 205], unit_i[ 206], unit_i[ 207], unit_i[ 208], unit_i[ 209], unit_i[ 210], unit_i[ 211], unit_i[ 212], unit_i[ 213], unit_i[ 214], unit_i[ 215], unit_i[ 216], unit_i[ 217], unit_i[ 218], unit_i[ 219], unit_i[ 220], unit_i[ 221], unit_i[ 222], unit_i[ 223], unit_i[ 224], unit_i[ 225], unit_i[ 226], unit_i[ 227], unit_i[ 228], unit_i[ 229], unit_i[ 230], unit_i[ 231], unit_i[ 232], unit_i[ 233], unit_i[ 234], unit_i[ 235], unit_i[ 236], unit_i[ 237], unit_i[ 238], unit_i[ 239], unit_i[ 240], unit_i[ 241], unit_i[ 242], unit_i[ 243], unit_i[ 244], unit_i[ 245], unit_i[ 246], unit_i[ 310], unit_i[ 311], unit_i[ 312], unit_i[ 313], unit_i[ 314], unit_i[ 315], unit_i[ 316], unit_i[ 317], unit_i[ 318], unit_i[ 319], unit_i[ 320], unit_i[ 321], unit_i[ 322], unit_i[ 323], unit_i[ 324], unit_i[ 325], unit_i[ 326], unit_i[ 327], unit_i[ 328], unit_i[ 329], unit_i[ 330], unit_i[ 331], unit_i[ 332], unit_i[ 333], unit_i[ 334], unit_i[ 335], unit_i[ 336], unit_i[ 337], unit_i[ 338], unit_i[ 339], unit_i[ 340], unit_i[ 341], unit_i[ 342], unit_i[ 343], unit_i[ 344], unit_i[ 345], unit_i[ 346], unit_i[ 347], unit_i[ 348], unit_i[ 349], unit_i[ 350], unit_i[ 351], unit_i[ 352], unit_i[ 353], unit_i[ 354], unit_i[ 355], unit_i[ 356], unit_i[ 357], unit_i[ 358], unit_i[ 359], unit_i[ 360], unit_i[ 361], unit_i[ 362], unit_i[ 363], unit_i[ 364], unit_i[ 365], unit_i[ 366], unit_i[ 367], unit_i[ 368], unit_i[ 369], unit_i[ 370], unit_i[ 371], unit_i[ 372], unit_i[ 373], unit_i[ 438], unit_i[ 439], unit_i[ 440], unit_i[ 441], unit_i[ 442], unit_i[ 443], unit_i[ 444], unit_i[ 445], unit_i[ 446], unit_i[ 447], unit_i[ 448], unit_i[ 449], unit_i[ 450], unit_i[ 451], unit_i[ 452], unit_i[ 453], unit_i[ 454], unit_i[ 455], unit_i[ 456], unit_i[ 457], unit_i[ 458], unit_i[ 459], unit_i[ 460], unit_i[ 461], unit_i[ 462], unit_i[ 463], unit_i[ 464], unit_i[ 465], unit_i[ 466], unit_i[ 467], unit_i[ 468], unit_i[ 469], unit_i[ 470], unit_i[ 471], unit_i[ 472], unit_i[ 473], unit_i[ 474], unit_i[ 475], unit_i[ 476], unit_i[ 477], unit_i[ 478], unit_i[ 479], unit_i[ 480], unit_i[ 481], unit_i[ 482], unit_i[ 483], unit_i[ 484], unit_i[ 485], unit_i[ 486], unit_i[ 487], unit_i[ 488], unit_i[ 489], unit_i[ 490], unit_i[ 491], unit_i[ 492], unit_i[ 493], unit_i[ 494], unit_i[ 495], unit_i[ 496], unit_i[ 497], unit_i[ 498], unit_i[ 499], unit_i[ 500], unit_i[ 501], unit_i[ 565], unit_i[ 566], unit_i[ 567], unit_i[ 568], unit_i[ 569], unit_i[ 570], unit_i[ 571], unit_i[ 572], unit_i[ 573], unit_i[ 574], unit_i[ 575], unit_i[ 576], unit_i[ 577], unit_i[ 578], unit_i[ 579], unit_i[ 580], unit_i[ 581], unit_i[ 582], unit_i[ 583], unit_i[ 584], unit_i[ 585], unit_i[ 586], unit_i[ 587], unit_i[ 588], unit_i[ 589], unit_i[ 590], unit_i[ 591], unit_i[ 592], unit_i[ 593], unit_i[ 594], unit_i[ 595], unit_i[ 596], unit_i[ 597], unit_i[ 598], unit_i[ 599], unit_i[ 600], unit_i[ 601], unit_i[ 602], unit_i[ 603], unit_i[ 604], unit_i[ 605], unit_i[ 606], unit_i[ 607], unit_i[ 608], unit_i[ 609], unit_i[ 610], unit_i[ 611], unit_i[ 612], unit_i[ 613], unit_i[ 614], unit_i[ 615], unit_i[ 616], unit_i[ 617], unit_i[ 618], unit_i[ 619], unit_i[ 620], unit_i[ 621], unit_i[ 622], unit_i[ 623], unit_i[ 624], unit_i[ 625], unit_i[ 626], unit_i[ 627], unit_i[ 628], unit_i[ 693], unit_i[ 694], unit_i[ 695], unit_i[ 696], unit_i[ 697], unit_i[ 698], unit_i[ 699], unit_i[ 700], unit_i[ 701], unit_i[ 702], unit_i[ 703], unit_i[ 704], unit_i[ 705], unit_i[ 706], unit_i[ 707], unit_i[ 708], unit_i[ 709], unit_i[ 710], unit_i[ 711], unit_i[ 712], unit_i[ 713], unit_i[ 714], unit_i[ 715], unit_i[ 716], unit_i[ 717], unit_i[ 718], unit_i[ 719], unit_i[ 720], unit_i[ 721], unit_i[ 722], unit_i[ 723], unit_i[ 724], unit_i[ 725], unit_i[ 726], unit_i[ 727], unit_i[ 728], unit_i[ 729], unit_i[ 730], unit_i[ 731], unit_i[ 732], unit_i[ 733], unit_i[ 734], unit_i[ 735], unit_i[ 736], unit_i[ 737], unit_i[ 738], unit_i[ 739], unit_i[ 740], unit_i[ 741], unit_i[ 742], unit_i[ 743], unit_i[ 744], unit_i[ 745], unit_i[ 746], unit_i[ 747], unit_i[ 748], unit_i[ 749], unit_i[ 750], unit_i[ 751], unit_i[ 752], unit_i[ 753], unit_i[ 754], unit_i[ 755], unit_i[ 756], unit_i[ 821], unit_i[ 822], unit_i[ 823], unit_i[ 824], unit_i[ 825], unit_i[ 826], unit_i[ 827], unit_i[ 828], unit_i[ 829], unit_i[ 830], unit_i[ 831], unit_i[ 832], unit_i[ 833], unit_i[ 834], unit_i[ 835], unit_i[ 836], unit_i[ 837], unit_i[ 838], unit_i[ 839], unit_i[ 840], unit_i[ 841], unit_i[ 842], unit_i[ 843], unit_i[ 844], unit_i[ 845], unit_i[ 846], unit_i[ 847], unit_i[ 848], unit_i[ 849], unit_i[ 850], unit_i[ 851], unit_i[ 852], unit_i[ 853], unit_i[ 854], unit_i[ 855], unit_i[ 856], unit_i[ 857], unit_i[ 858], unit_i[ 859], unit_i[ 860], unit_i[ 861], unit_i[ 862], unit_i[ 863], unit_i[ 864], unit_i[ 865], unit_i[ 866], unit_i[ 867], unit_i[ 868], unit_i[ 869], unit_i[ 870], unit_i[ 871], unit_i[ 872], unit_i[ 873], unit_i[ 874], unit_i[ 875], unit_i[ 876], unit_i[ 877], unit_i[ 878], unit_i[ 879], unit_i[ 880], unit_i[ 881], unit_i[ 882], unit_i[ 883], unit_i[ 884], unit_i[ 949], unit_i[ 950], unit_i[ 951], unit_i[ 952], unit_i[ 953], unit_i[ 954], unit_i[ 955], unit_i[ 956], unit_i[ 957], unit_i[ 958], unit_i[ 959], unit_i[ 960], unit_i[ 961], unit_i[ 962], unit_i[ 963], unit_i[ 964], unit_i[ 965], unit_i[ 966], unit_i[ 967], unit_i[ 968], unit_i[ 969], unit_i[ 970], unit_i[ 971], unit_i[ 972], unit_i[ 973], unit_i[ 974], unit_i[ 975], unit_i[ 976], unit_i[ 977], unit_i[ 978], unit_i[ 979], unit_i[ 980], unit_i[ 981], unit_i[ 982], unit_i[ 983], unit_i[ 984], unit_i[ 985], unit_i[ 986], unit_i[ 987], unit_i[ 988], unit_i[ 989], unit_i[ 990], unit_i[ 991], unit_i[ 992], unit_i[ 993], unit_i[ 994], unit_i[ 995], unit_i[ 996], unit_i[ 997], unit_i[ 998], unit_i[ 999], unit_i[1000], unit_i[1001], unit_i[1002], unit_i[1003], unit_i[1004], unit_i[1005], unit_i[1006], unit_i[1007]};
assign ecc_calc_w[ 7] = ^{unit_i[ 120], unit_i[ 121], unit_i[ 122], unit_i[ 123], unit_i[ 124], unit_i[ 125], unit_i[ 126], unit_i[ 127], unit_i[ 128], unit_i[ 129], unit_i[ 130], unit_i[ 131], unit_i[ 132], unit_i[ 133], unit_i[ 134], unit_i[ 135], unit_i[ 136], unit_i[ 137], unit_i[ 138], unit_i[ 139], unit_i[ 140], unit_i[ 141], unit_i[ 142], unit_i[ 143], unit_i[ 144], unit_i[ 145], unit_i[ 146], unit_i[ 147], unit_i[ 148], unit_i[ 149], unit_i[ 150], unit_i[ 151], unit_i[ 152], unit_i[ 153], unit_i[ 154], unit_i[ 155], unit_i[ 156], unit_i[ 157], unit_i[ 158], unit_i[ 159], unit_i[ 160], unit_i[ 161], unit_i[ 162], unit_i[ 163], unit_i[ 164], unit_i[ 165], unit_i[ 166], unit_i[ 167], unit_i[ 168], unit_i[ 169], unit_i[ 170], unit_i[ 171], unit_i[ 172], unit_i[ 173], unit_i[ 174], unit_i[ 175], unit_i[ 176], unit_i[ 177], unit_i[ 178], unit_i[ 179], unit_i[ 180], unit_i[ 181], unit_i[ 182], unit_i[ 183], unit_i[ 184], unit_i[ 185], unit_i[ 186], unit_i[ 187], unit_i[ 188], unit_i[ 189], unit_i[ 190], unit_i[ 191], unit_i[ 192], unit_i[ 193], unit_i[ 194], unit_i[ 195], unit_i[ 196], unit_i[ 197], unit_i[ 198], unit_i[ 199], unit_i[ 200], unit_i[ 201], unit_i[ 202], unit_i[ 203], unit_i[ 204], unit_i[ 205], unit_i[ 206], unit_i[ 207], unit_i[ 208], unit_i[ 209], unit_i[ 210], unit_i[ 211], unit_i[ 212], unit_i[ 213], unit_i[ 214], unit_i[ 215], unit_i[ 216], unit_i[ 217], unit_i[ 218], unit_i[ 219], unit_i[ 220], unit_i[ 221], unit_i[ 222], unit_i[ 223], unit_i[ 224], unit_i[ 225], unit_i[ 226], unit_i[ 227], unit_i[ 228], unit_i[ 229], unit_i[ 230], unit_i[ 231], unit_i[ 232], unit_i[ 233], unit_i[ 234], unit_i[ 235], unit_i[ 236], unit_i[ 237], unit_i[ 238], unit_i[ 239], unit_i[ 240], unit_i[ 241], unit_i[ 242], unit_i[ 243], unit_i[ 244], unit_i[ 245], unit_i[ 246], unit_i[ 374], unit_i[ 375], unit_i[ 376], unit_i[ 377], unit_i[ 378], unit_i[ 379], unit_i[ 380], unit_i[ 381], unit_i[ 382], unit_i[ 383], unit_i[ 384], unit_i[ 385], unit_i[ 386], unit_i[ 387], unit_i[ 388], unit_i[ 389], unit_i[ 390], unit_i[ 391], unit_i[ 392], unit_i[ 393], unit_i[ 394], unit_i[ 395], unit_i[ 396], unit_i[ 397], unit_i[ 398], unit_i[ 399], unit_i[ 400], unit_i[ 401], unit_i[ 402], unit_i[ 403], unit_i[ 404], unit_i[ 405], unit_i[ 406], unit_i[ 407], unit_i[ 408], unit_i[ 409], unit_i[ 410], unit_i[ 411], unit_i[ 412], unit_i[ 413], unit_i[ 414], unit_i[ 415], unit_i[ 416], unit_i[ 417], unit_i[ 418], unit_i[ 419], unit_i[ 420], unit_i[ 421], unit_i[ 422], unit_i[ 423], unit_i[ 424], unit_i[ 425], unit_i[ 426], unit_i[ 427], unit_i[ 428], unit_i[ 429], unit_i[ 430], unit_i[ 431], unit_i[ 432], unit_i[ 433], unit_i[ 434], unit_i[ 435], unit_i[ 436], unit_i[ 437], unit_i[ 438], unit_i[ 439], unit_i[ 440], unit_i[ 441], unit_i[ 442], unit_i[ 443], unit_i[ 444], unit_i[ 445], unit_i[ 446], unit_i[ 447], unit_i[ 448], unit_i[ 449], unit_i[ 450], unit_i[ 451], unit_i[ 452], unit_i[ 453], unit_i[ 454], unit_i[ 455], unit_i[ 456], unit_i[ 457], unit_i[ 458], unit_i[ 459], unit_i[ 460], unit_i[ 461], unit_i[ 462], unit_i[ 463], unit_i[ 464], unit_i[ 465], unit_i[ 466], unit_i[ 467], unit_i[ 468], unit_i[ 469], unit_i[ 470], unit_i[ 471], unit_i[ 472], unit_i[ 473], unit_i[ 474], unit_i[ 475], unit_i[ 476], unit_i[ 477], unit_i[ 478], unit_i[ 479], unit_i[ 480], unit_i[ 481], unit_i[ 482], unit_i[ 483], unit_i[ 484], unit_i[ 485], unit_i[ 486], unit_i[ 487], unit_i[ 488], unit_i[ 489], unit_i[ 490], unit_i[ 491], unit_i[ 492], unit_i[ 493], unit_i[ 494], unit_i[ 495], unit_i[ 496], unit_i[ 497], unit_i[ 498], unit_i[ 499], unit_i[ 500], unit_i[ 501], unit_i[ 629], unit_i[ 630], unit_i[ 631], unit_i[ 632], unit_i[ 633], unit_i[ 634], unit_i[ 635], unit_i[ 636], unit_i[ 637], unit_i[ 638], unit_i[ 639], unit_i[ 640], unit_i[ 641], unit_i[ 642], unit_i[ 643], unit_i[ 644], unit_i[ 645], unit_i[ 646], unit_i[ 647], unit_i[ 648], unit_i[ 649], unit_i[ 650], unit_i[ 651], unit_i[ 652], unit_i[ 653], unit_i[ 654], unit_i[ 655], unit_i[ 656], unit_i[ 657], unit_i[ 658], unit_i[ 659], unit_i[ 660], unit_i[ 661], unit_i[ 662], unit_i[ 663], unit_i[ 664], unit_i[ 665], unit_i[ 666], unit_i[ 667], unit_i[ 668], unit_i[ 669], unit_i[ 670], unit_i[ 671], unit_i[ 672], unit_i[ 673], unit_i[ 674], unit_i[ 675], unit_i[ 676], unit_i[ 677], unit_i[ 678], unit_i[ 679], unit_i[ 680], unit_i[ 681], unit_i[ 682], unit_i[ 683], unit_i[ 684], unit_i[ 685], unit_i[ 686], unit_i[ 687], unit_i[ 688], unit_i[ 689], unit_i[ 690], unit_i[ 691], unit_i[ 692], unit_i[ 693], unit_i[ 694], unit_i[ 695], unit_i[ 696], unit_i[ 697], unit_i[ 698], unit_i[ 699], unit_i[ 700], unit_i[ 701], unit_i[ 702], unit_i[ 703], unit_i[ 704], unit_i[ 705], unit_i[ 706], unit_i[ 707], unit_i[ 708], unit_i[ 709], unit_i[ 710], unit_i[ 711], unit_i[ 712], unit_i[ 713], unit_i[ 714], unit_i[ 715], unit_i[ 716], unit_i[ 717], unit_i[ 718], unit_i[ 719], unit_i[ 720], unit_i[ 721], unit_i[ 722], unit_i[ 723], unit_i[ 724], unit_i[ 725], unit_i[ 726], unit_i[ 727], unit_i[ 728], unit_i[ 729], unit_i[ 730], unit_i[ 731], unit_i[ 732], unit_i[ 733], unit_i[ 734], unit_i[ 735], unit_i[ 736], unit_i[ 737], unit_i[ 738], unit_i[ 739], unit_i[ 740], unit_i[ 741], unit_i[ 742], unit_i[ 743], unit_i[ 744], unit_i[ 745], unit_i[ 746], unit_i[ 747], unit_i[ 748], unit_i[ 749], unit_i[ 750], unit_i[ 751], unit_i[ 752], unit_i[ 753], unit_i[ 754], unit_i[ 755], unit_i[ 756], unit_i[ 885], unit_i[ 886], unit_i[ 887], unit_i[ 888], unit_i[ 889], unit_i[ 890], unit_i[ 891], unit_i[ 892], unit_i[ 893], unit_i[ 894], unit_i[ 895], unit_i[ 896], unit_i[ 897], unit_i[ 898], unit_i[ 899], unit_i[ 900], unit_i[ 901], unit_i[ 902], unit_i[ 903], unit_i[ 904], unit_i[ 905], unit_i[ 906], unit_i[ 907], unit_i[ 908], unit_i[ 909], unit_i[ 910], unit_i[ 911], unit_i[ 912], unit_i[ 913], unit_i[ 914], unit_i[ 915], unit_i[ 916], unit_i[ 917], unit_i[ 918], unit_i[ 919], unit_i[ 920], unit_i[ 921], unit_i[ 922], unit_i[ 923], unit_i[ 924], unit_i[ 925], unit_i[ 926], unit_i[ 927], unit_i[ 928], unit_i[ 929], unit_i[ 930], unit_i[ 931], unit_i[ 932], unit_i[ 933], unit_i[ 934], unit_i[ 935], unit_i[ 936], unit_i[ 937], unit_i[ 938], unit_i[ 939], unit_i[ 940], unit_i[ 941], unit_i[ 942], unit_i[ 943], unit_i[ 944], unit_i[ 945], unit_i[ 946], unit_i[ 947], unit_i[ 948], unit_i[ 949], unit_i[ 950], unit_i[ 951], unit_i[ 952], unit_i[ 953], unit_i[ 954], unit_i[ 955], unit_i[ 956], unit_i[ 957], unit_i[ 958], unit_i[ 959], unit_i[ 960], unit_i[ 961], unit_i[ 962], unit_i[ 963], unit_i[ 964], unit_i[ 965], unit_i[ 966], unit_i[ 967], unit_i[ 968], unit_i[ 969], unit_i[ 970], unit_i[ 971], unit_i[ 972], unit_i[ 973], unit_i[ 974], unit_i[ 975], unit_i[ 976], unit_i[ 977], unit_i[ 978], unit_i[ 979], unit_i[ 980], unit_i[ 981], unit_i[ 982], unit_i[ 983], unit_i[ 984], unit_i[ 985], unit_i[ 986], unit_i[ 987], unit_i[ 988], unit_i[ 989], unit_i[ 990], unit_i[ 991], unit_i[ 992], unit_i[ 993], unit_i[ 994], unit_i[ 995], unit_i[ 996], unit_i[ 997], unit_i[ 998], unit_i[ 999], unit_i[1000], unit_i[1001], unit_i[1002], unit_i[1003], unit_i[1004], unit_i[1005], unit_i[1006], unit_i[1007]};
assign ecc_calc_w[ 8] = ^{unit_i[ 247], unit_i[ 248], unit_i[ 249], unit_i[ 250], unit_i[ 251], unit_i[ 252], unit_i[ 253], unit_i[ 254], unit_i[ 255], unit_i[ 256], unit_i[ 257], unit_i[ 258], unit_i[ 259], unit_i[ 260], unit_i[ 261], unit_i[ 262], unit_i[ 263], unit_i[ 264], unit_i[ 265], unit_i[ 266], unit_i[ 267], unit_i[ 268], unit_i[ 269], unit_i[ 270], unit_i[ 271], unit_i[ 272], unit_i[ 273], unit_i[ 274], unit_i[ 275], unit_i[ 276], unit_i[ 277], unit_i[ 278], unit_i[ 279], unit_i[ 280], unit_i[ 281], unit_i[ 282], unit_i[ 283], unit_i[ 284], unit_i[ 285], unit_i[ 286], unit_i[ 287], unit_i[ 288], unit_i[ 289], unit_i[ 290], unit_i[ 291], unit_i[ 292], unit_i[ 293], unit_i[ 294], unit_i[ 295], unit_i[ 296], unit_i[ 297], unit_i[ 298], unit_i[ 299], unit_i[ 300], unit_i[ 301], unit_i[ 302], unit_i[ 303], unit_i[ 304], unit_i[ 305], unit_i[ 306], unit_i[ 307], unit_i[ 308], unit_i[ 309], unit_i[ 310], unit_i[ 311], unit_i[ 312], unit_i[ 313], unit_i[ 314], unit_i[ 315], unit_i[ 316], unit_i[ 317], unit_i[ 318], unit_i[ 319], unit_i[ 320], unit_i[ 321], unit_i[ 322], unit_i[ 323], unit_i[ 324], unit_i[ 325], unit_i[ 326], unit_i[ 327], unit_i[ 328], unit_i[ 329], unit_i[ 330], unit_i[ 331], unit_i[ 332], unit_i[ 333], unit_i[ 334], unit_i[ 335], unit_i[ 336], unit_i[ 337], unit_i[ 338], unit_i[ 339], unit_i[ 340], unit_i[ 341], unit_i[ 342], unit_i[ 343], unit_i[ 344], unit_i[ 345], unit_i[ 346], unit_i[ 347], unit_i[ 348], unit_i[ 349], unit_i[ 350], unit_i[ 351], unit_i[ 352], unit_i[ 353], unit_i[ 354], unit_i[ 355], unit_i[ 356], unit_i[ 357], unit_i[ 358], unit_i[ 359], unit_i[ 360], unit_i[ 361], unit_i[ 362], unit_i[ 363], unit_i[ 364], unit_i[ 365], unit_i[ 366], unit_i[ 367], unit_i[ 368], unit_i[ 369], unit_i[ 370], unit_i[ 371], unit_i[ 372], unit_i[ 373], unit_i[ 374], unit_i[ 375], unit_i[ 376], unit_i[ 377], unit_i[ 378], unit_i[ 379], unit_i[ 380], unit_i[ 381], unit_i[ 382], unit_i[ 383], unit_i[ 384], unit_i[ 385], unit_i[ 386], unit_i[ 387], unit_i[ 388], unit_i[ 389], unit_i[ 390], unit_i[ 391], unit_i[ 392], unit_i[ 393], unit_i[ 394], unit_i[ 395], unit_i[ 396], unit_i[ 397], unit_i[ 398], unit_i[ 399], unit_i[ 400], unit_i[ 401], unit_i[ 402], unit_i[ 403], unit_i[ 404], unit_i[ 405], unit_i[ 406], unit_i[ 407], unit_i[ 408], unit_i[ 409], unit_i[ 410], unit_i[ 411], unit_i[ 412], unit_i[ 413], unit_i[ 414], unit_i[ 415], unit_i[ 416], unit_i[ 417], unit_i[ 418], unit_i[ 419], unit_i[ 420], unit_i[ 421], unit_i[ 422], unit_i[ 423], unit_i[ 424], unit_i[ 425], unit_i[ 426], unit_i[ 427], unit_i[ 428], unit_i[ 429], unit_i[ 430], unit_i[ 431], unit_i[ 432], unit_i[ 433], unit_i[ 434], unit_i[ 435], unit_i[ 436], unit_i[ 437], unit_i[ 438], unit_i[ 439], unit_i[ 440], unit_i[ 441], unit_i[ 442], unit_i[ 443], unit_i[ 444], unit_i[ 445], unit_i[ 446], unit_i[ 447], unit_i[ 448], unit_i[ 449], unit_i[ 450], unit_i[ 451], unit_i[ 452], unit_i[ 453], unit_i[ 454], unit_i[ 455], unit_i[ 456], unit_i[ 457], unit_i[ 458], unit_i[ 459], unit_i[ 460], unit_i[ 461], unit_i[ 462], unit_i[ 463], unit_i[ 464], unit_i[ 465], unit_i[ 466], unit_i[ 467], unit_i[ 468], unit_i[ 469], unit_i[ 470], unit_i[ 471], unit_i[ 472], unit_i[ 473], unit_i[ 474], unit_i[ 475], unit_i[ 476], unit_i[ 477], unit_i[ 478], unit_i[ 479], unit_i[ 480], unit_i[ 481], unit_i[ 482], unit_i[ 483], unit_i[ 484], unit_i[ 485], unit_i[ 486], unit_i[ 487], unit_i[ 488], unit_i[ 489], unit_i[ 490], unit_i[ 491], unit_i[ 492], unit_i[ 493], unit_i[ 494], unit_i[ 495], unit_i[ 496], unit_i[ 497], unit_i[ 498], unit_i[ 499], unit_i[ 500], unit_i[ 501], unit_i[ 757], unit_i[ 758], unit_i[ 759], unit_i[ 760], unit_i[ 761], unit_i[ 762], unit_i[ 763], unit_i[ 764], unit_i[ 765], unit_i[ 766], unit_i[ 767], unit_i[ 768], unit_i[ 769], unit_i[ 770], unit_i[ 771], unit_i[ 772], unit_i[ 773], unit_i[ 774], unit_i[ 775], unit_i[ 776], unit_i[ 777], unit_i[ 778], unit_i[ 779], unit_i[ 780], unit_i[ 781], unit_i[ 782], unit_i[ 783], unit_i[ 784], unit_i[ 785], unit_i[ 786], unit_i[ 787], unit_i[ 788], unit_i[ 789], unit_i[ 790], unit_i[ 791], unit_i[ 792], unit_i[ 793], unit_i[ 794], unit_i[ 795], unit_i[ 796], unit_i[ 797], unit_i[ 798], unit_i[ 799], unit_i[ 800], unit_i[ 801], unit_i[ 802], unit_i[ 803], unit_i[ 804], unit_i[ 805], unit_i[ 806], unit_i[ 807], unit_i[ 808], unit_i[ 809], unit_i[ 810], unit_i[ 811], unit_i[ 812], unit_i[ 813], unit_i[ 814], unit_i[ 815], unit_i[ 816], unit_i[ 817], unit_i[ 818], unit_i[ 819], unit_i[ 820], unit_i[ 821], unit_i[ 822], unit_i[ 823], unit_i[ 824], unit_i[ 825], unit_i[ 826], unit_i[ 827], unit_i[ 828], unit_i[ 829], unit_i[ 830], unit_i[ 831], unit_i[ 832], unit_i[ 833], unit_i[ 834], unit_i[ 835], unit_i[ 836], unit_i[ 837], unit_i[ 838], unit_i[ 839], unit_i[ 840], unit_i[ 841], unit_i[ 842], unit_i[ 843], unit_i[ 844], unit_i[ 845], unit_i[ 846], unit_i[ 847], unit_i[ 848], unit_i[ 849], unit_i[ 850], unit_i[ 851], unit_i[ 852], unit_i[ 853], unit_i[ 854], unit_i[ 855], unit_i[ 856], unit_i[ 857], unit_i[ 858], unit_i[ 859], unit_i[ 860], unit_i[ 861], unit_i[ 862], unit_i[ 863], unit_i[ 864], unit_i[ 865], unit_i[ 866], unit_i[ 867], unit_i[ 868], unit_i[ 869], unit_i[ 870], unit_i[ 871], unit_i[ 872], unit_i[ 873], unit_i[ 874], unit_i[ 875], unit_i[ 876], unit_i[ 877], unit_i[ 878], unit_i[ 879], unit_i[ 880], unit_i[ 881], unit_i[ 882], unit_i[ 883], unit_i[ 884], unit_i[ 885], unit_i[ 886], unit_i[ 887], unit_i[ 888], unit_i[ 889], unit_i[ 890], unit_i[ 891], unit_i[ 892], unit_i[ 893], unit_i[ 894], unit_i[ 895], unit_i[ 896], unit_i[ 897], unit_i[ 898], unit_i[ 899], unit_i[ 900], unit_i[ 901], unit_i[ 902], unit_i[ 903], unit_i[ 904], unit_i[ 905], unit_i[ 906], unit_i[ 907], unit_i[ 908], unit_i[ 909], unit_i[ 910], unit_i[ 911], unit_i[ 912], unit_i[ 913], unit_i[ 914], unit_i[ 915], unit_i[ 916], unit_i[ 917], unit_i[ 918], unit_i[ 919], unit_i[ 920], unit_i[ 921], unit_i[ 922], unit_i[ 923], unit_i[ 924], unit_i[ 925], unit_i[ 926], unit_i[ 927], unit_i[ 928], unit_i[ 929], unit_i[ 930], unit_i[ 931], unit_i[ 932], unit_i[ 933], unit_i[ 934], unit_i[ 935], unit_i[ 936], unit_i[ 937], unit_i[ 938], unit_i[ 939], unit_i[ 940], unit_i[ 941], unit_i[ 942], unit_i[ 943], unit_i[ 944], unit_i[ 945], unit_i[ 946], unit_i[ 947], unit_i[ 948], unit_i[ 949], unit_i[ 950], unit_i[ 951], unit_i[ 952], unit_i[ 953], unit_i[ 954], unit_i[ 955], unit_i[ 956], unit_i[ 957], unit_i[ 958], unit_i[ 959], unit_i[ 960], unit_i[ 961], unit_i[ 962], unit_i[ 963], unit_i[ 964], unit_i[ 965], unit_i[ 966], unit_i[ 967], unit_i[ 968], unit_i[ 969], unit_i[ 970], unit_i[ 971], unit_i[ 972], unit_i[ 973], unit_i[ 974], unit_i[ 975], unit_i[ 976], unit_i[ 977], unit_i[ 978], unit_i[ 979], unit_i[ 980], unit_i[ 981], unit_i[ 982], unit_i[ 983], unit_i[ 984], unit_i[ 985], unit_i[ 986], unit_i[ 987], unit_i[ 988], unit_i[ 989], unit_i[ 990], unit_i[ 991], unit_i[ 992], unit_i[ 993], unit_i[ 994], unit_i[ 995], unit_i[ 996], unit_i[ 997], unit_i[ 998], unit_i[ 999], unit_i[1000], unit_i[1001], unit_i[1002], unit_i[1003], unit_i[1004], unit_i[1005], unit_i[1006], unit_i[1007]};
assign ecc_calc_w[ 9] = ^{unit_i[ 502], unit_i[ 503], unit_i[ 504], unit_i[ 505], unit_i[ 506], unit_i[ 507], unit_i[ 508], unit_i[ 509], unit_i[ 510], unit_i[ 511], unit_i[ 512], unit_i[ 513], unit_i[ 514], unit_i[ 515], unit_i[ 516], unit_i[ 517], unit_i[ 518], unit_i[ 519], unit_i[ 520], unit_i[ 521], unit_i[ 522], unit_i[ 523], unit_i[ 524], unit_i[ 525], unit_i[ 526], unit_i[ 527], unit_i[ 528], unit_i[ 529], unit_i[ 530], unit_i[ 531], unit_i[ 532], unit_i[ 533], unit_i[ 534], unit_i[ 535], unit_i[ 536], unit_i[ 537], unit_i[ 538], unit_i[ 539], unit_i[ 540], unit_i[ 541], unit_i[ 542], unit_i[ 543], unit_i[ 544], unit_i[ 545], unit_i[ 546], unit_i[ 547], unit_i[ 548], unit_i[ 549], unit_i[ 550], unit_i[ 551], unit_i[ 552], unit_i[ 553], unit_i[ 554], unit_i[ 555], unit_i[ 556], unit_i[ 557], unit_i[ 558], unit_i[ 559], unit_i[ 560], unit_i[ 561], unit_i[ 562], unit_i[ 563], unit_i[ 564], unit_i[ 565], unit_i[ 566], unit_i[ 567], unit_i[ 568], unit_i[ 569], unit_i[ 570], unit_i[ 571], unit_i[ 572], unit_i[ 573], unit_i[ 574], unit_i[ 575], unit_i[ 576], unit_i[ 577], unit_i[ 578], unit_i[ 579], unit_i[ 580], unit_i[ 581], unit_i[ 582], unit_i[ 583], unit_i[ 584], unit_i[ 585], unit_i[ 586], unit_i[ 587], unit_i[ 588], unit_i[ 589], unit_i[ 590], unit_i[ 591], unit_i[ 592], unit_i[ 593], unit_i[ 594], unit_i[ 595], unit_i[ 596], unit_i[ 597], unit_i[ 598], unit_i[ 599], unit_i[ 600], unit_i[ 601], unit_i[ 602], unit_i[ 603], unit_i[ 604], unit_i[ 605], unit_i[ 606], unit_i[ 607], unit_i[ 608], unit_i[ 609], unit_i[ 610], unit_i[ 611], unit_i[ 612], unit_i[ 613], unit_i[ 614], unit_i[ 615], unit_i[ 616], unit_i[ 617], unit_i[ 618], unit_i[ 619], unit_i[ 620], unit_i[ 621], unit_i[ 622], unit_i[ 623], unit_i[ 624], unit_i[ 625], unit_i[ 626], unit_i[ 627], unit_i[ 628], unit_i[ 629], unit_i[ 630], unit_i[ 631], unit_i[ 632], unit_i[ 633], unit_i[ 634], unit_i[ 635], unit_i[ 636], unit_i[ 637], unit_i[ 638], unit_i[ 639], unit_i[ 640], unit_i[ 641], unit_i[ 642], unit_i[ 643], unit_i[ 644], unit_i[ 645], unit_i[ 646], unit_i[ 647], unit_i[ 648], unit_i[ 649], unit_i[ 650], unit_i[ 651], unit_i[ 652], unit_i[ 653], unit_i[ 654], unit_i[ 655], unit_i[ 656], unit_i[ 657], unit_i[ 658], unit_i[ 659], unit_i[ 660], unit_i[ 661], unit_i[ 662], unit_i[ 663], unit_i[ 664], unit_i[ 665], unit_i[ 666], unit_i[ 667], unit_i[ 668], unit_i[ 669], unit_i[ 670], unit_i[ 671], unit_i[ 672], unit_i[ 673], unit_i[ 674], unit_i[ 675], unit_i[ 676], unit_i[ 677], unit_i[ 678], unit_i[ 679], unit_i[ 680], unit_i[ 681], unit_i[ 682], unit_i[ 683], unit_i[ 684], unit_i[ 685], unit_i[ 686], unit_i[ 687], unit_i[ 688], unit_i[ 689], unit_i[ 690], unit_i[ 691], unit_i[ 692], unit_i[ 693], unit_i[ 694], unit_i[ 695], unit_i[ 696], unit_i[ 697], unit_i[ 698], unit_i[ 699], unit_i[ 700], unit_i[ 701], unit_i[ 702], unit_i[ 703], unit_i[ 704], unit_i[ 705], unit_i[ 706], unit_i[ 707], unit_i[ 708], unit_i[ 709], unit_i[ 710], unit_i[ 711], unit_i[ 712], unit_i[ 713], unit_i[ 714], unit_i[ 715], unit_i[ 716], unit_i[ 717], unit_i[ 718], unit_i[ 719], unit_i[ 720], unit_i[ 721], unit_i[ 722], unit_i[ 723], unit_i[ 724], unit_i[ 725], unit_i[ 726], unit_i[ 727], unit_i[ 728], unit_i[ 729], unit_i[ 730], unit_i[ 731], unit_i[ 732], unit_i[ 733], unit_i[ 734], unit_i[ 735], unit_i[ 736], unit_i[ 737], unit_i[ 738], unit_i[ 739], unit_i[ 740], unit_i[ 741], unit_i[ 742], unit_i[ 743], unit_i[ 744], unit_i[ 745], unit_i[ 746], unit_i[ 747], unit_i[ 748], unit_i[ 749], unit_i[ 750], unit_i[ 751], unit_i[ 752], unit_i[ 753], unit_i[ 754], unit_i[ 755], unit_i[ 756], unit_i[ 757], unit_i[ 758], unit_i[ 759], unit_i[ 760], unit_i[ 761], unit_i[ 762], unit_i[ 763], unit_i[ 764], unit_i[ 765], unit_i[ 766], unit_i[ 767], unit_i[ 768], unit_i[ 769], unit_i[ 770], unit_i[ 771], unit_i[ 772], unit_i[ 773], unit_i[ 774], unit_i[ 775], unit_i[ 776], unit_i[ 777], unit_i[ 778], unit_i[ 779], unit_i[ 780], unit_i[ 781], unit_i[ 782], unit_i[ 783], unit_i[ 784], unit_i[ 785], unit_i[ 786], unit_i[ 787], unit_i[ 788], unit_i[ 789], unit_i[ 790], unit_i[ 791], unit_i[ 792], unit_i[ 793], unit_i[ 794], unit_i[ 795], unit_i[ 796], unit_i[ 797], unit_i[ 798], unit_i[ 799], unit_i[ 800], unit_i[ 801], unit_i[ 802], unit_i[ 803], unit_i[ 804], unit_i[ 805], unit_i[ 806], unit_i[ 807], unit_i[ 808], unit_i[ 809], unit_i[ 810], unit_i[ 811], unit_i[ 812], unit_i[ 813], unit_i[ 814], unit_i[ 815], unit_i[ 816], unit_i[ 817], unit_i[ 818], unit_i[ 819], unit_i[ 820], unit_i[ 821], unit_i[ 822], unit_i[ 823], unit_i[ 824], unit_i[ 825], unit_i[ 826], unit_i[ 827], unit_i[ 828], unit_i[ 829], unit_i[ 830], unit_i[ 831], unit_i[ 832], unit_i[ 833], unit_i[ 834], unit_i[ 835], unit_i[ 836], unit_i[ 837], unit_i[ 838], unit_i[ 839], unit_i[ 840], unit_i[ 841], unit_i[ 842], unit_i[ 843], unit_i[ 844], unit_i[ 845], unit_i[ 846], unit_i[ 847], unit_i[ 848], unit_i[ 849], unit_i[ 850], unit_i[ 851], unit_i[ 852], unit_i[ 853], unit_i[ 854], unit_i[ 855], unit_i[ 856], unit_i[ 857], unit_i[ 858], unit_i[ 859], unit_i[ 860], unit_i[ 861], unit_i[ 862], unit_i[ 863], unit_i[ 864], unit_i[ 865], unit_i[ 866], unit_i[ 867], unit_i[ 868], unit_i[ 869], unit_i[ 870], unit_i[ 871], unit_i[ 872], unit_i[ 873], unit_i[ 874], unit_i[ 875], unit_i[ 876], unit_i[ 877], unit_i[ 878], unit_i[ 879], unit_i[ 880], unit_i[ 881], unit_i[ 882], unit_i[ 883], unit_i[ 884], unit_i[ 885], unit_i[ 886], unit_i[ 887], unit_i[ 888], unit_i[ 889], unit_i[ 890], unit_i[ 891], unit_i[ 892], unit_i[ 893], unit_i[ 894], unit_i[ 895], unit_i[ 896], unit_i[ 897], unit_i[ 898], unit_i[ 899], unit_i[ 900], unit_i[ 901], unit_i[ 902], unit_i[ 903], unit_i[ 904], unit_i[ 905], unit_i[ 906], unit_i[ 907], unit_i[ 908], unit_i[ 909], unit_i[ 910], unit_i[ 911], unit_i[ 912], unit_i[ 913], unit_i[ 914], unit_i[ 915], unit_i[ 916], unit_i[ 917], unit_i[ 918], unit_i[ 919], unit_i[ 920], unit_i[ 921], unit_i[ 922], unit_i[ 923], unit_i[ 924], unit_i[ 925], unit_i[ 926], unit_i[ 927], unit_i[ 928], unit_i[ 929], unit_i[ 930], unit_i[ 931], unit_i[ 932], unit_i[ 933], unit_i[ 934], unit_i[ 935], unit_i[ 936], unit_i[ 937], unit_i[ 938], unit_i[ 939], unit_i[ 940], unit_i[ 941], unit_i[ 942], unit_i[ 943], unit_i[ 944], unit_i[ 945], unit_i[ 946], unit_i[ 947], unit_i[ 948], unit_i[ 949], unit_i[ 950], unit_i[ 951], unit_i[ 952], unit_i[ 953], unit_i[ 954], unit_i[ 955], unit_i[ 956], unit_i[ 957], unit_i[ 958], unit_i[ 959], unit_i[ 960], unit_i[ 961], unit_i[ 962], unit_i[ 963], unit_i[ 964], unit_i[ 965], unit_i[ 966], unit_i[ 967], unit_i[ 968], unit_i[ 969], unit_i[ 970], unit_i[ 971], unit_i[ 972], unit_i[ 973], unit_i[ 974], unit_i[ 975], unit_i[ 976], unit_i[ 977], unit_i[ 978], unit_i[ 979], unit_i[ 980], unit_i[ 981], unit_i[ 982], unit_i[ 983], unit_i[ 984], unit_i[ 985], unit_i[ 986], unit_i[ 987], unit_i[ 988], unit_i[ 989], unit_i[ 990], unit_i[ 991], unit_i[ 992], unit_i[ 993], unit_i[ 994], unit_i[ 995], unit_i[ 996], unit_i[ 997], unit_i[ 998], unit_i[ 999], unit_i[1000], unit_i[1001], unit_i[1002], unit_i[1003], unit_i[1004], unit_i[1005], unit_i[1006], unit_i[1007]};
assign ecc_calc_w[10] = 1'b0;   // 空行：1008-bit 口径下无列 value 第 10 bit 置 1 → 结构性恒 0
assign ecc_calc_w[11] = ^{unit_i[1007:0], unit_i[1018:1008]};   // P 重算行（SECDED，LL-20260923-CHK1008）：^{数据 1008, field[10:0]}（1019 项，@E 组合与矩阵同级深度），勿手改

// 去 D23（v1.9，LL-20261015-DUALPUSH）：删 D23 打 1 拍（ecc_calc_p/ecc_field_p/
// flit_valid_p/unit_p 寄存），E 拍同拍组合直通；mb_clk/mb_rst_n 端口保留（同
// crc9_check 先例）。无寄存，纯组合直通（对齐 crc16_check v1.10）。

// =============================================================================
// E 拍 syndrome + 判定（组合）
// =============================================================================
assign en_ck        = csr_ecc_en_i & csr_perf_mode_i;
assign ecc_field_w  = unit_i[1019:1008];  // 单元携带 ECC 字段（E 拍）
assign syndrome_w   = ecc_calc_w ^ ecc_field_w;   // E 拍组合
assign s_chk        = syndrome_w[10:0];   // SEC 部分（11-bit）
assign p_chk        = syndrome_w[11];     // P 位奇偶比较
assign s_pow2       = |s_chk & ((s_chk & (s_chk - 11'd1)) == 11'd0);   // S 为 2 幂（改法 A；与 data_hit/parity 共享）

// (S, P_chk) 双判据分类（SECDED）：
//   ① S=0                        → pass（无错 / P 位自身单错，P_chk 无关）
//   ② S 命中载荷数据列 & P_chk=1  → pass + 纠正该位（data_hit_w，载荷位 0..1007 单错可纠）
//   ③ S 命中单位向量 & P_chk=1    → pass 不纠（字段位自身单错 j∈[0,10]，corr_mask 全 0）
//   ④ 其余（(S≠0, P_chk=0) 双错形态 / (S≠0, P_chk=1) 未分类多错）→ err（不可纠；绝不静默 pass）

// LL-TIMING-N2 改法 A（实现形式自由，判据语义 MAS v1.4 §5.1 不变）：1008 数据列 ≡
//   {3 ≤ s ≤ 0x3FA(1018)} 且 s 非 2 幂（2048 值穷举与 minterm 集合恒等；arch 独立复核）。
//   旧式 = p_chk & 1008 路 11-bit 等值 OR（≈14-15 级）→ 新式 = 范围 + 2 幂判据（≈8-9 级）。
assign data_hit_w = p_chk & (s_chk >= 11'd3) & (s_chk <= 11'h3fa) & ~s_pow2;

// 单位向量 e_0..e_10（11 个 2 幂，SEC 字段位单错形态）命中，且 P 位比较一致
// LL-TIMING-N2 改法 A（实现形式自由，判据语义 MAS v1.4 §5.1 不变）：{2^0..2^10} ≡ s_pow2（与 data_hit 共享）
assign parity_err_w = p_chk & s_pow2;

// 不可纠：(S≠0, P_chk=0) 全部双错形态 / (S≠0, P_chk=1) 未分类多错 → 保守 err，绝不静默 pass
// LL-TIMING-N2 改法 A（实现形式自由，判据语义 MAS v1.4 §5.1 不变；2048 值穷举与旧补集式恒等）：
//   s<3 的 1/2 与 s=1024 均为 2 幂 ∈ parity 已排除，未分类区 = s > 0x3FA 非 2 幂
assign uncorr_w = (|s_chk & ~p_chk) | (p_chk & (s_chk > 11'h3fa) & ~s_pow2);

// 纠正掩码（E 拍 syndrome_w 同拍组合译码单热点、不打拍；P_chk 门控；仅载荷位 0..1007 可纠，槽/tie 区不纠）
//   由 gen_ecc_matrix.py 生成：corr_mask_w[j] = (s_chk == col[ j ])（j ∈ [0,1007]，S→bit 映射不变）
assign corr_mask_w[   0] = p_chk & (s_chk == 11'h003);
assign corr_mask_w[   1] = p_chk & (s_chk == 11'h005);
assign corr_mask_w[   2] = p_chk & (s_chk == 11'h006);
assign corr_mask_w[   3] = p_chk & (s_chk == 11'h007);
assign corr_mask_w[   4] = p_chk & (s_chk == 11'h009);
assign corr_mask_w[   5] = p_chk & (s_chk == 11'h00a);
assign corr_mask_w[   6] = p_chk & (s_chk == 11'h00b);
assign corr_mask_w[   7] = p_chk & (s_chk == 11'h00c);
assign corr_mask_w[   8] = p_chk & (s_chk == 11'h00d);
assign corr_mask_w[   9] = p_chk & (s_chk == 11'h00e);
assign corr_mask_w[  10] = p_chk & (s_chk == 11'h00f);
assign corr_mask_w[  11] = p_chk & (s_chk == 11'h011);
assign corr_mask_w[  12] = p_chk & (s_chk == 11'h012);
assign corr_mask_w[  13] = p_chk & (s_chk == 11'h013);
assign corr_mask_w[  14] = p_chk & (s_chk == 11'h014);
assign corr_mask_w[  15] = p_chk & (s_chk == 11'h015);
assign corr_mask_w[  16] = p_chk & (s_chk == 11'h016);
assign corr_mask_w[  17] = p_chk & (s_chk == 11'h017);
assign corr_mask_w[  18] = p_chk & (s_chk == 11'h018);
assign corr_mask_w[  19] = p_chk & (s_chk == 11'h019);
assign corr_mask_w[  20] = p_chk & (s_chk == 11'h01a);
assign corr_mask_w[  21] = p_chk & (s_chk == 11'h01b);
assign corr_mask_w[  22] = p_chk & (s_chk == 11'h01c);
assign corr_mask_w[  23] = p_chk & (s_chk == 11'h01d);
assign corr_mask_w[  24] = p_chk & (s_chk == 11'h01e);
assign corr_mask_w[  25] = p_chk & (s_chk == 11'h01f);
assign corr_mask_w[  26] = p_chk & (s_chk == 11'h021);
assign corr_mask_w[  27] = p_chk & (s_chk == 11'h022);
assign corr_mask_w[  28] = p_chk & (s_chk == 11'h023);
assign corr_mask_w[  29] = p_chk & (s_chk == 11'h024);
assign corr_mask_w[  30] = p_chk & (s_chk == 11'h025);
assign corr_mask_w[  31] = p_chk & (s_chk == 11'h026);
assign corr_mask_w[  32] = p_chk & (s_chk == 11'h027);
assign corr_mask_w[  33] = p_chk & (s_chk == 11'h028);
assign corr_mask_w[  34] = p_chk & (s_chk == 11'h029);
assign corr_mask_w[  35] = p_chk & (s_chk == 11'h02a);
assign corr_mask_w[  36] = p_chk & (s_chk == 11'h02b);
assign corr_mask_w[  37] = p_chk & (s_chk == 11'h02c);
assign corr_mask_w[  38] = p_chk & (s_chk == 11'h02d);
assign corr_mask_w[  39] = p_chk & (s_chk == 11'h02e);
assign corr_mask_w[  40] = p_chk & (s_chk == 11'h02f);
assign corr_mask_w[  41] = p_chk & (s_chk == 11'h030);
assign corr_mask_w[  42] = p_chk & (s_chk == 11'h031);
assign corr_mask_w[  43] = p_chk & (s_chk == 11'h032);
assign corr_mask_w[  44] = p_chk & (s_chk == 11'h033);
assign corr_mask_w[  45] = p_chk & (s_chk == 11'h034);
assign corr_mask_w[  46] = p_chk & (s_chk == 11'h035);
assign corr_mask_w[  47] = p_chk & (s_chk == 11'h036);
assign corr_mask_w[  48] = p_chk & (s_chk == 11'h037);
assign corr_mask_w[  49] = p_chk & (s_chk == 11'h038);
assign corr_mask_w[  50] = p_chk & (s_chk == 11'h039);
assign corr_mask_w[  51] = p_chk & (s_chk == 11'h03a);
assign corr_mask_w[  52] = p_chk & (s_chk == 11'h03b);
assign corr_mask_w[  53] = p_chk & (s_chk == 11'h03c);
assign corr_mask_w[  54] = p_chk & (s_chk == 11'h03d);
assign corr_mask_w[  55] = p_chk & (s_chk == 11'h03e);
assign corr_mask_w[  56] = p_chk & (s_chk == 11'h03f);
assign corr_mask_w[  57] = p_chk & (s_chk == 11'h041);
assign corr_mask_w[  58] = p_chk & (s_chk == 11'h042);
assign corr_mask_w[  59] = p_chk & (s_chk == 11'h043);
assign corr_mask_w[  60] = p_chk & (s_chk == 11'h044);
assign corr_mask_w[  61] = p_chk & (s_chk == 11'h045);
assign corr_mask_w[  62] = p_chk & (s_chk == 11'h046);
assign corr_mask_w[  63] = p_chk & (s_chk == 11'h047);
assign corr_mask_w[  64] = p_chk & (s_chk == 11'h048);
assign corr_mask_w[  65] = p_chk & (s_chk == 11'h049);
assign corr_mask_w[  66] = p_chk & (s_chk == 11'h04a);
assign corr_mask_w[  67] = p_chk & (s_chk == 11'h04b);
assign corr_mask_w[  68] = p_chk & (s_chk == 11'h04c);
assign corr_mask_w[  69] = p_chk & (s_chk == 11'h04d);
assign corr_mask_w[  70] = p_chk & (s_chk == 11'h04e);
assign corr_mask_w[  71] = p_chk & (s_chk == 11'h04f);
assign corr_mask_w[  72] = p_chk & (s_chk == 11'h050);
assign corr_mask_w[  73] = p_chk & (s_chk == 11'h051);
assign corr_mask_w[  74] = p_chk & (s_chk == 11'h052);
assign corr_mask_w[  75] = p_chk & (s_chk == 11'h053);
assign corr_mask_w[  76] = p_chk & (s_chk == 11'h054);
assign corr_mask_w[  77] = p_chk & (s_chk == 11'h055);
assign corr_mask_w[  78] = p_chk & (s_chk == 11'h056);
assign corr_mask_w[  79] = p_chk & (s_chk == 11'h057);
assign corr_mask_w[  80] = p_chk & (s_chk == 11'h058);
assign corr_mask_w[  81] = p_chk & (s_chk == 11'h059);
assign corr_mask_w[  82] = p_chk & (s_chk == 11'h05a);
assign corr_mask_w[  83] = p_chk & (s_chk == 11'h05b);
assign corr_mask_w[  84] = p_chk & (s_chk == 11'h05c);
assign corr_mask_w[  85] = p_chk & (s_chk == 11'h05d);
assign corr_mask_w[  86] = p_chk & (s_chk == 11'h05e);
assign corr_mask_w[  87] = p_chk & (s_chk == 11'h05f);
assign corr_mask_w[  88] = p_chk & (s_chk == 11'h060);
assign corr_mask_w[  89] = p_chk & (s_chk == 11'h061);
assign corr_mask_w[  90] = p_chk & (s_chk == 11'h062);
assign corr_mask_w[  91] = p_chk & (s_chk == 11'h063);
assign corr_mask_w[  92] = p_chk & (s_chk == 11'h064);
assign corr_mask_w[  93] = p_chk & (s_chk == 11'h065);
assign corr_mask_w[  94] = p_chk & (s_chk == 11'h066);
assign corr_mask_w[  95] = p_chk & (s_chk == 11'h067);
assign corr_mask_w[  96] = p_chk & (s_chk == 11'h068);
assign corr_mask_w[  97] = p_chk & (s_chk == 11'h069);
assign corr_mask_w[  98] = p_chk & (s_chk == 11'h06a);
assign corr_mask_w[  99] = p_chk & (s_chk == 11'h06b);
assign corr_mask_w[ 100] = p_chk & (s_chk == 11'h06c);
assign corr_mask_w[ 101] = p_chk & (s_chk == 11'h06d);
assign corr_mask_w[ 102] = p_chk & (s_chk == 11'h06e);
assign corr_mask_w[ 103] = p_chk & (s_chk == 11'h06f);
assign corr_mask_w[ 104] = p_chk & (s_chk == 11'h070);
assign corr_mask_w[ 105] = p_chk & (s_chk == 11'h071);
assign corr_mask_w[ 106] = p_chk & (s_chk == 11'h072);
assign corr_mask_w[ 107] = p_chk & (s_chk == 11'h073);
assign corr_mask_w[ 108] = p_chk & (s_chk == 11'h074);
assign corr_mask_w[ 109] = p_chk & (s_chk == 11'h075);
assign corr_mask_w[ 110] = p_chk & (s_chk == 11'h076);
assign corr_mask_w[ 111] = p_chk & (s_chk == 11'h077);
assign corr_mask_w[ 112] = p_chk & (s_chk == 11'h078);
assign corr_mask_w[ 113] = p_chk & (s_chk == 11'h079);
assign corr_mask_w[ 114] = p_chk & (s_chk == 11'h07a);
assign corr_mask_w[ 115] = p_chk & (s_chk == 11'h07b);
assign corr_mask_w[ 116] = p_chk & (s_chk == 11'h07c);
assign corr_mask_w[ 117] = p_chk & (s_chk == 11'h07d);
assign corr_mask_w[ 118] = p_chk & (s_chk == 11'h07e);
assign corr_mask_w[ 119] = p_chk & (s_chk == 11'h07f);
assign corr_mask_w[ 120] = p_chk & (s_chk == 11'h081);
assign corr_mask_w[ 121] = p_chk & (s_chk == 11'h082);
assign corr_mask_w[ 122] = p_chk & (s_chk == 11'h083);
assign corr_mask_w[ 123] = p_chk & (s_chk == 11'h084);
assign corr_mask_w[ 124] = p_chk & (s_chk == 11'h085);
assign corr_mask_w[ 125] = p_chk & (s_chk == 11'h086);
assign corr_mask_w[ 126] = p_chk & (s_chk == 11'h087);
assign corr_mask_w[ 127] = p_chk & (s_chk == 11'h088);
assign corr_mask_w[ 128] = p_chk & (s_chk == 11'h089);
assign corr_mask_w[ 129] = p_chk & (s_chk == 11'h08a);
assign corr_mask_w[ 130] = p_chk & (s_chk == 11'h08b);
assign corr_mask_w[ 131] = p_chk & (s_chk == 11'h08c);
assign corr_mask_w[ 132] = p_chk & (s_chk == 11'h08d);
assign corr_mask_w[ 133] = p_chk & (s_chk == 11'h08e);
assign corr_mask_w[ 134] = p_chk & (s_chk == 11'h08f);
assign corr_mask_w[ 135] = p_chk & (s_chk == 11'h090);
assign corr_mask_w[ 136] = p_chk & (s_chk == 11'h091);
assign corr_mask_w[ 137] = p_chk & (s_chk == 11'h092);
assign corr_mask_w[ 138] = p_chk & (s_chk == 11'h093);
assign corr_mask_w[ 139] = p_chk & (s_chk == 11'h094);
assign corr_mask_w[ 140] = p_chk & (s_chk == 11'h095);
assign corr_mask_w[ 141] = p_chk & (s_chk == 11'h096);
assign corr_mask_w[ 142] = p_chk & (s_chk == 11'h097);
assign corr_mask_w[ 143] = p_chk & (s_chk == 11'h098);
assign corr_mask_w[ 144] = p_chk & (s_chk == 11'h099);
assign corr_mask_w[ 145] = p_chk & (s_chk == 11'h09a);
assign corr_mask_w[ 146] = p_chk & (s_chk == 11'h09b);
assign corr_mask_w[ 147] = p_chk & (s_chk == 11'h09c);
assign corr_mask_w[ 148] = p_chk & (s_chk == 11'h09d);
assign corr_mask_w[ 149] = p_chk & (s_chk == 11'h09e);
assign corr_mask_w[ 150] = p_chk & (s_chk == 11'h09f);
assign corr_mask_w[ 151] = p_chk & (s_chk == 11'h0a0);
assign corr_mask_w[ 152] = p_chk & (s_chk == 11'h0a1);
assign corr_mask_w[ 153] = p_chk & (s_chk == 11'h0a2);
assign corr_mask_w[ 154] = p_chk & (s_chk == 11'h0a3);
assign corr_mask_w[ 155] = p_chk & (s_chk == 11'h0a4);
assign corr_mask_w[ 156] = p_chk & (s_chk == 11'h0a5);
assign corr_mask_w[ 157] = p_chk & (s_chk == 11'h0a6);
assign corr_mask_w[ 158] = p_chk & (s_chk == 11'h0a7);
assign corr_mask_w[ 159] = p_chk & (s_chk == 11'h0a8);
assign corr_mask_w[ 160] = p_chk & (s_chk == 11'h0a9);
assign corr_mask_w[ 161] = p_chk & (s_chk == 11'h0aa);
assign corr_mask_w[ 162] = p_chk & (s_chk == 11'h0ab);
assign corr_mask_w[ 163] = p_chk & (s_chk == 11'h0ac);
assign corr_mask_w[ 164] = p_chk & (s_chk == 11'h0ad);
assign corr_mask_w[ 165] = p_chk & (s_chk == 11'h0ae);
assign corr_mask_w[ 166] = p_chk & (s_chk == 11'h0af);
assign corr_mask_w[ 167] = p_chk & (s_chk == 11'h0b0);
assign corr_mask_w[ 168] = p_chk & (s_chk == 11'h0b1);
assign corr_mask_w[ 169] = p_chk & (s_chk == 11'h0b2);
assign corr_mask_w[ 170] = p_chk & (s_chk == 11'h0b3);
assign corr_mask_w[ 171] = p_chk & (s_chk == 11'h0b4);
assign corr_mask_w[ 172] = p_chk & (s_chk == 11'h0b5);
assign corr_mask_w[ 173] = p_chk & (s_chk == 11'h0b6);
assign corr_mask_w[ 174] = p_chk & (s_chk == 11'h0b7);
assign corr_mask_w[ 175] = p_chk & (s_chk == 11'h0b8);
assign corr_mask_w[ 176] = p_chk & (s_chk == 11'h0b9);
assign corr_mask_w[ 177] = p_chk & (s_chk == 11'h0ba);
assign corr_mask_w[ 178] = p_chk & (s_chk == 11'h0bb);
assign corr_mask_w[ 179] = p_chk & (s_chk == 11'h0bc);
assign corr_mask_w[ 180] = p_chk & (s_chk == 11'h0bd);
assign corr_mask_w[ 181] = p_chk & (s_chk == 11'h0be);
assign corr_mask_w[ 182] = p_chk & (s_chk == 11'h0bf);
assign corr_mask_w[ 183] = p_chk & (s_chk == 11'h0c0);
assign corr_mask_w[ 184] = p_chk & (s_chk == 11'h0c1);
assign corr_mask_w[ 185] = p_chk & (s_chk == 11'h0c2);
assign corr_mask_w[ 186] = p_chk & (s_chk == 11'h0c3);
assign corr_mask_w[ 187] = p_chk & (s_chk == 11'h0c4);
assign corr_mask_w[ 188] = p_chk & (s_chk == 11'h0c5);
assign corr_mask_w[ 189] = p_chk & (s_chk == 11'h0c6);
assign corr_mask_w[ 190] = p_chk & (s_chk == 11'h0c7);
assign corr_mask_w[ 191] = p_chk & (s_chk == 11'h0c8);
assign corr_mask_w[ 192] = p_chk & (s_chk == 11'h0c9);
assign corr_mask_w[ 193] = p_chk & (s_chk == 11'h0ca);
assign corr_mask_w[ 194] = p_chk & (s_chk == 11'h0cb);
assign corr_mask_w[ 195] = p_chk & (s_chk == 11'h0cc);
assign corr_mask_w[ 196] = p_chk & (s_chk == 11'h0cd);
assign corr_mask_w[ 197] = p_chk & (s_chk == 11'h0ce);
assign corr_mask_w[ 198] = p_chk & (s_chk == 11'h0cf);
assign corr_mask_w[ 199] = p_chk & (s_chk == 11'h0d0);
assign corr_mask_w[ 200] = p_chk & (s_chk == 11'h0d1);
assign corr_mask_w[ 201] = p_chk & (s_chk == 11'h0d2);
assign corr_mask_w[ 202] = p_chk & (s_chk == 11'h0d3);
assign corr_mask_w[ 203] = p_chk & (s_chk == 11'h0d4);
assign corr_mask_w[ 204] = p_chk & (s_chk == 11'h0d5);
assign corr_mask_w[ 205] = p_chk & (s_chk == 11'h0d6);
assign corr_mask_w[ 206] = p_chk & (s_chk == 11'h0d7);
assign corr_mask_w[ 207] = p_chk & (s_chk == 11'h0d8);
assign corr_mask_w[ 208] = p_chk & (s_chk == 11'h0d9);
assign corr_mask_w[ 209] = p_chk & (s_chk == 11'h0da);
assign corr_mask_w[ 210] = p_chk & (s_chk == 11'h0db);
assign corr_mask_w[ 211] = p_chk & (s_chk == 11'h0dc);
assign corr_mask_w[ 212] = p_chk & (s_chk == 11'h0dd);
assign corr_mask_w[ 213] = p_chk & (s_chk == 11'h0de);
assign corr_mask_w[ 214] = p_chk & (s_chk == 11'h0df);
assign corr_mask_w[ 215] = p_chk & (s_chk == 11'h0e0);
assign corr_mask_w[ 216] = p_chk & (s_chk == 11'h0e1);
assign corr_mask_w[ 217] = p_chk & (s_chk == 11'h0e2);
assign corr_mask_w[ 218] = p_chk & (s_chk == 11'h0e3);
assign corr_mask_w[ 219] = p_chk & (s_chk == 11'h0e4);
assign corr_mask_w[ 220] = p_chk & (s_chk == 11'h0e5);
assign corr_mask_w[ 221] = p_chk & (s_chk == 11'h0e6);
assign corr_mask_w[ 222] = p_chk & (s_chk == 11'h0e7);
assign corr_mask_w[ 223] = p_chk & (s_chk == 11'h0e8);
assign corr_mask_w[ 224] = p_chk & (s_chk == 11'h0e9);
assign corr_mask_w[ 225] = p_chk & (s_chk == 11'h0ea);
assign corr_mask_w[ 226] = p_chk & (s_chk == 11'h0eb);
assign corr_mask_w[ 227] = p_chk & (s_chk == 11'h0ec);
assign corr_mask_w[ 228] = p_chk & (s_chk == 11'h0ed);
assign corr_mask_w[ 229] = p_chk & (s_chk == 11'h0ee);
assign corr_mask_w[ 230] = p_chk & (s_chk == 11'h0ef);
assign corr_mask_w[ 231] = p_chk & (s_chk == 11'h0f0);
assign corr_mask_w[ 232] = p_chk & (s_chk == 11'h0f1);
assign corr_mask_w[ 233] = p_chk & (s_chk == 11'h0f2);
assign corr_mask_w[ 234] = p_chk & (s_chk == 11'h0f3);
assign corr_mask_w[ 235] = p_chk & (s_chk == 11'h0f4);
assign corr_mask_w[ 236] = p_chk & (s_chk == 11'h0f5);
assign corr_mask_w[ 237] = p_chk & (s_chk == 11'h0f6);
assign corr_mask_w[ 238] = p_chk & (s_chk == 11'h0f7);
assign corr_mask_w[ 239] = p_chk & (s_chk == 11'h0f8);
assign corr_mask_w[ 240] = p_chk & (s_chk == 11'h0f9);
assign corr_mask_w[ 241] = p_chk & (s_chk == 11'h0fa);
assign corr_mask_w[ 242] = p_chk & (s_chk == 11'h0fb);
assign corr_mask_w[ 243] = p_chk & (s_chk == 11'h0fc);
assign corr_mask_w[ 244] = p_chk & (s_chk == 11'h0fd);
assign corr_mask_w[ 245] = p_chk & (s_chk == 11'h0fe);
assign corr_mask_w[ 246] = p_chk & (s_chk == 11'h0ff);
assign corr_mask_w[ 247] = p_chk & (s_chk == 11'h101);
assign corr_mask_w[ 248] = p_chk & (s_chk == 11'h102);
assign corr_mask_w[ 249] = p_chk & (s_chk == 11'h103);
assign corr_mask_w[ 250] = p_chk & (s_chk == 11'h104);
assign corr_mask_w[ 251] = p_chk & (s_chk == 11'h105);
assign corr_mask_w[ 252] = p_chk & (s_chk == 11'h106);
assign corr_mask_w[ 253] = p_chk & (s_chk == 11'h107);
assign corr_mask_w[ 254] = p_chk & (s_chk == 11'h108);
assign corr_mask_w[ 255] = p_chk & (s_chk == 11'h109);
assign corr_mask_w[ 256] = p_chk & (s_chk == 11'h10a);
assign corr_mask_w[ 257] = p_chk & (s_chk == 11'h10b);
assign corr_mask_w[ 258] = p_chk & (s_chk == 11'h10c);
assign corr_mask_w[ 259] = p_chk & (s_chk == 11'h10d);
assign corr_mask_w[ 260] = p_chk & (s_chk == 11'h10e);
assign corr_mask_w[ 261] = p_chk & (s_chk == 11'h10f);
assign corr_mask_w[ 262] = p_chk & (s_chk == 11'h110);
assign corr_mask_w[ 263] = p_chk & (s_chk == 11'h111);
assign corr_mask_w[ 264] = p_chk & (s_chk == 11'h112);
assign corr_mask_w[ 265] = p_chk & (s_chk == 11'h113);
assign corr_mask_w[ 266] = p_chk & (s_chk == 11'h114);
assign corr_mask_w[ 267] = p_chk & (s_chk == 11'h115);
assign corr_mask_w[ 268] = p_chk & (s_chk == 11'h116);
assign corr_mask_w[ 269] = p_chk & (s_chk == 11'h117);
assign corr_mask_w[ 270] = p_chk & (s_chk == 11'h118);
assign corr_mask_w[ 271] = p_chk & (s_chk == 11'h119);
assign corr_mask_w[ 272] = p_chk & (s_chk == 11'h11a);
assign corr_mask_w[ 273] = p_chk & (s_chk == 11'h11b);
assign corr_mask_w[ 274] = p_chk & (s_chk == 11'h11c);
assign corr_mask_w[ 275] = p_chk & (s_chk == 11'h11d);
assign corr_mask_w[ 276] = p_chk & (s_chk == 11'h11e);
assign corr_mask_w[ 277] = p_chk & (s_chk == 11'h11f);
assign corr_mask_w[ 278] = p_chk & (s_chk == 11'h120);
assign corr_mask_w[ 279] = p_chk & (s_chk == 11'h121);
assign corr_mask_w[ 280] = p_chk & (s_chk == 11'h122);
assign corr_mask_w[ 281] = p_chk & (s_chk == 11'h123);
assign corr_mask_w[ 282] = p_chk & (s_chk == 11'h124);
assign corr_mask_w[ 283] = p_chk & (s_chk == 11'h125);
assign corr_mask_w[ 284] = p_chk & (s_chk == 11'h126);
assign corr_mask_w[ 285] = p_chk & (s_chk == 11'h127);
assign corr_mask_w[ 286] = p_chk & (s_chk == 11'h128);
assign corr_mask_w[ 287] = p_chk & (s_chk == 11'h129);
assign corr_mask_w[ 288] = p_chk & (s_chk == 11'h12a);
assign corr_mask_w[ 289] = p_chk & (s_chk == 11'h12b);
assign corr_mask_w[ 290] = p_chk & (s_chk == 11'h12c);
assign corr_mask_w[ 291] = p_chk & (s_chk == 11'h12d);
assign corr_mask_w[ 292] = p_chk & (s_chk == 11'h12e);
assign corr_mask_w[ 293] = p_chk & (s_chk == 11'h12f);
assign corr_mask_w[ 294] = p_chk & (s_chk == 11'h130);
assign corr_mask_w[ 295] = p_chk & (s_chk == 11'h131);
assign corr_mask_w[ 296] = p_chk & (s_chk == 11'h132);
assign corr_mask_w[ 297] = p_chk & (s_chk == 11'h133);
assign corr_mask_w[ 298] = p_chk & (s_chk == 11'h134);
assign corr_mask_w[ 299] = p_chk & (s_chk == 11'h135);
assign corr_mask_w[ 300] = p_chk & (s_chk == 11'h136);
assign corr_mask_w[ 301] = p_chk & (s_chk == 11'h137);
assign corr_mask_w[ 302] = p_chk & (s_chk == 11'h138);
assign corr_mask_w[ 303] = p_chk & (s_chk == 11'h139);
assign corr_mask_w[ 304] = p_chk & (s_chk == 11'h13a);
assign corr_mask_w[ 305] = p_chk & (s_chk == 11'h13b);
assign corr_mask_w[ 306] = p_chk & (s_chk == 11'h13c);
assign corr_mask_w[ 307] = p_chk & (s_chk == 11'h13d);
assign corr_mask_w[ 308] = p_chk & (s_chk == 11'h13e);
assign corr_mask_w[ 309] = p_chk & (s_chk == 11'h13f);
assign corr_mask_w[ 310] = p_chk & (s_chk == 11'h140);
assign corr_mask_w[ 311] = p_chk & (s_chk == 11'h141);
assign corr_mask_w[ 312] = p_chk & (s_chk == 11'h142);
assign corr_mask_w[ 313] = p_chk & (s_chk == 11'h143);
assign corr_mask_w[ 314] = p_chk & (s_chk == 11'h144);
assign corr_mask_w[ 315] = p_chk & (s_chk == 11'h145);
assign corr_mask_w[ 316] = p_chk & (s_chk == 11'h146);
assign corr_mask_w[ 317] = p_chk & (s_chk == 11'h147);
assign corr_mask_w[ 318] = p_chk & (s_chk == 11'h148);
assign corr_mask_w[ 319] = p_chk & (s_chk == 11'h149);
assign corr_mask_w[ 320] = p_chk & (s_chk == 11'h14a);
assign corr_mask_w[ 321] = p_chk & (s_chk == 11'h14b);
assign corr_mask_w[ 322] = p_chk & (s_chk == 11'h14c);
assign corr_mask_w[ 323] = p_chk & (s_chk == 11'h14d);
assign corr_mask_w[ 324] = p_chk & (s_chk == 11'h14e);
assign corr_mask_w[ 325] = p_chk & (s_chk == 11'h14f);
assign corr_mask_w[ 326] = p_chk & (s_chk == 11'h150);
assign corr_mask_w[ 327] = p_chk & (s_chk == 11'h151);
assign corr_mask_w[ 328] = p_chk & (s_chk == 11'h152);
assign corr_mask_w[ 329] = p_chk & (s_chk == 11'h153);
assign corr_mask_w[ 330] = p_chk & (s_chk == 11'h154);
assign corr_mask_w[ 331] = p_chk & (s_chk == 11'h155);
assign corr_mask_w[ 332] = p_chk & (s_chk == 11'h156);
assign corr_mask_w[ 333] = p_chk & (s_chk == 11'h157);
assign corr_mask_w[ 334] = p_chk & (s_chk == 11'h158);
assign corr_mask_w[ 335] = p_chk & (s_chk == 11'h159);
assign corr_mask_w[ 336] = p_chk & (s_chk == 11'h15a);
assign corr_mask_w[ 337] = p_chk & (s_chk == 11'h15b);
assign corr_mask_w[ 338] = p_chk & (s_chk == 11'h15c);
assign corr_mask_w[ 339] = p_chk & (s_chk == 11'h15d);
assign corr_mask_w[ 340] = p_chk & (s_chk == 11'h15e);
assign corr_mask_w[ 341] = p_chk & (s_chk == 11'h15f);
assign corr_mask_w[ 342] = p_chk & (s_chk == 11'h160);
assign corr_mask_w[ 343] = p_chk & (s_chk == 11'h161);
assign corr_mask_w[ 344] = p_chk & (s_chk == 11'h162);
assign corr_mask_w[ 345] = p_chk & (s_chk == 11'h163);
assign corr_mask_w[ 346] = p_chk & (s_chk == 11'h164);
assign corr_mask_w[ 347] = p_chk & (s_chk == 11'h165);
assign corr_mask_w[ 348] = p_chk & (s_chk == 11'h166);
assign corr_mask_w[ 349] = p_chk & (s_chk == 11'h167);
assign corr_mask_w[ 350] = p_chk & (s_chk == 11'h168);
assign corr_mask_w[ 351] = p_chk & (s_chk == 11'h169);
assign corr_mask_w[ 352] = p_chk & (s_chk == 11'h16a);
assign corr_mask_w[ 353] = p_chk & (s_chk == 11'h16b);
assign corr_mask_w[ 354] = p_chk & (s_chk == 11'h16c);
assign corr_mask_w[ 355] = p_chk & (s_chk == 11'h16d);
assign corr_mask_w[ 356] = p_chk & (s_chk == 11'h16e);
assign corr_mask_w[ 357] = p_chk & (s_chk == 11'h16f);
assign corr_mask_w[ 358] = p_chk & (s_chk == 11'h170);
assign corr_mask_w[ 359] = p_chk & (s_chk == 11'h171);
assign corr_mask_w[ 360] = p_chk & (s_chk == 11'h172);
assign corr_mask_w[ 361] = p_chk & (s_chk == 11'h173);
assign corr_mask_w[ 362] = p_chk & (s_chk == 11'h174);
assign corr_mask_w[ 363] = p_chk & (s_chk == 11'h175);
assign corr_mask_w[ 364] = p_chk & (s_chk == 11'h176);
assign corr_mask_w[ 365] = p_chk & (s_chk == 11'h177);
assign corr_mask_w[ 366] = p_chk & (s_chk == 11'h178);
assign corr_mask_w[ 367] = p_chk & (s_chk == 11'h179);
assign corr_mask_w[ 368] = p_chk & (s_chk == 11'h17a);
assign corr_mask_w[ 369] = p_chk & (s_chk == 11'h17b);
assign corr_mask_w[ 370] = p_chk & (s_chk == 11'h17c);
assign corr_mask_w[ 371] = p_chk & (s_chk == 11'h17d);
assign corr_mask_w[ 372] = p_chk & (s_chk == 11'h17e);
assign corr_mask_w[ 373] = p_chk & (s_chk == 11'h17f);
assign corr_mask_w[ 374] = p_chk & (s_chk == 11'h180);
assign corr_mask_w[ 375] = p_chk & (s_chk == 11'h181);
assign corr_mask_w[ 376] = p_chk & (s_chk == 11'h182);
assign corr_mask_w[ 377] = p_chk & (s_chk == 11'h183);
assign corr_mask_w[ 378] = p_chk & (s_chk == 11'h184);
assign corr_mask_w[ 379] = p_chk & (s_chk == 11'h185);
assign corr_mask_w[ 380] = p_chk & (s_chk == 11'h186);
assign corr_mask_w[ 381] = p_chk & (s_chk == 11'h187);
assign corr_mask_w[ 382] = p_chk & (s_chk == 11'h188);
assign corr_mask_w[ 383] = p_chk & (s_chk == 11'h189);
assign corr_mask_w[ 384] = p_chk & (s_chk == 11'h18a);
assign corr_mask_w[ 385] = p_chk & (s_chk == 11'h18b);
assign corr_mask_w[ 386] = p_chk & (s_chk == 11'h18c);
assign corr_mask_w[ 387] = p_chk & (s_chk == 11'h18d);
assign corr_mask_w[ 388] = p_chk & (s_chk == 11'h18e);
assign corr_mask_w[ 389] = p_chk & (s_chk == 11'h18f);
assign corr_mask_w[ 390] = p_chk & (s_chk == 11'h190);
assign corr_mask_w[ 391] = p_chk & (s_chk == 11'h191);
assign corr_mask_w[ 392] = p_chk & (s_chk == 11'h192);
assign corr_mask_w[ 393] = p_chk & (s_chk == 11'h193);
assign corr_mask_w[ 394] = p_chk & (s_chk == 11'h194);
assign corr_mask_w[ 395] = p_chk & (s_chk == 11'h195);
assign corr_mask_w[ 396] = p_chk & (s_chk == 11'h196);
assign corr_mask_w[ 397] = p_chk & (s_chk == 11'h197);
assign corr_mask_w[ 398] = p_chk & (s_chk == 11'h198);
assign corr_mask_w[ 399] = p_chk & (s_chk == 11'h199);
assign corr_mask_w[ 400] = p_chk & (s_chk == 11'h19a);
assign corr_mask_w[ 401] = p_chk & (s_chk == 11'h19b);
assign corr_mask_w[ 402] = p_chk & (s_chk == 11'h19c);
assign corr_mask_w[ 403] = p_chk & (s_chk == 11'h19d);
assign corr_mask_w[ 404] = p_chk & (s_chk == 11'h19e);
assign corr_mask_w[ 405] = p_chk & (s_chk == 11'h19f);
assign corr_mask_w[ 406] = p_chk & (s_chk == 11'h1a0);
assign corr_mask_w[ 407] = p_chk & (s_chk == 11'h1a1);
assign corr_mask_w[ 408] = p_chk & (s_chk == 11'h1a2);
assign corr_mask_w[ 409] = p_chk & (s_chk == 11'h1a3);
assign corr_mask_w[ 410] = p_chk & (s_chk == 11'h1a4);
assign corr_mask_w[ 411] = p_chk & (s_chk == 11'h1a5);
assign corr_mask_w[ 412] = p_chk & (s_chk == 11'h1a6);
assign corr_mask_w[ 413] = p_chk & (s_chk == 11'h1a7);
assign corr_mask_w[ 414] = p_chk & (s_chk == 11'h1a8);
assign corr_mask_w[ 415] = p_chk & (s_chk == 11'h1a9);
assign corr_mask_w[ 416] = p_chk & (s_chk == 11'h1aa);
assign corr_mask_w[ 417] = p_chk & (s_chk == 11'h1ab);
assign corr_mask_w[ 418] = p_chk & (s_chk == 11'h1ac);
assign corr_mask_w[ 419] = p_chk & (s_chk == 11'h1ad);
assign corr_mask_w[ 420] = p_chk & (s_chk == 11'h1ae);
assign corr_mask_w[ 421] = p_chk & (s_chk == 11'h1af);
assign corr_mask_w[ 422] = p_chk & (s_chk == 11'h1b0);
assign corr_mask_w[ 423] = p_chk & (s_chk == 11'h1b1);
assign corr_mask_w[ 424] = p_chk & (s_chk == 11'h1b2);
assign corr_mask_w[ 425] = p_chk & (s_chk == 11'h1b3);
assign corr_mask_w[ 426] = p_chk & (s_chk == 11'h1b4);
assign corr_mask_w[ 427] = p_chk & (s_chk == 11'h1b5);
assign corr_mask_w[ 428] = p_chk & (s_chk == 11'h1b6);
assign corr_mask_w[ 429] = p_chk & (s_chk == 11'h1b7);
assign corr_mask_w[ 430] = p_chk & (s_chk == 11'h1b8);
assign corr_mask_w[ 431] = p_chk & (s_chk == 11'h1b9);
assign corr_mask_w[ 432] = p_chk & (s_chk == 11'h1ba);
assign corr_mask_w[ 433] = p_chk & (s_chk == 11'h1bb);
assign corr_mask_w[ 434] = p_chk & (s_chk == 11'h1bc);
assign corr_mask_w[ 435] = p_chk & (s_chk == 11'h1bd);
assign corr_mask_w[ 436] = p_chk & (s_chk == 11'h1be);
assign corr_mask_w[ 437] = p_chk & (s_chk == 11'h1bf);
assign corr_mask_w[ 438] = p_chk & (s_chk == 11'h1c0);
assign corr_mask_w[ 439] = p_chk & (s_chk == 11'h1c1);
assign corr_mask_w[ 440] = p_chk & (s_chk == 11'h1c2);
assign corr_mask_w[ 441] = p_chk & (s_chk == 11'h1c3);
assign corr_mask_w[ 442] = p_chk & (s_chk == 11'h1c4);
assign corr_mask_w[ 443] = p_chk & (s_chk == 11'h1c5);
assign corr_mask_w[ 444] = p_chk & (s_chk == 11'h1c6);
assign corr_mask_w[ 445] = p_chk & (s_chk == 11'h1c7);
assign corr_mask_w[ 446] = p_chk & (s_chk == 11'h1c8);
assign corr_mask_w[ 447] = p_chk & (s_chk == 11'h1c9);
assign corr_mask_w[ 448] = p_chk & (s_chk == 11'h1ca);
assign corr_mask_w[ 449] = p_chk & (s_chk == 11'h1cb);
assign corr_mask_w[ 450] = p_chk & (s_chk == 11'h1cc);
assign corr_mask_w[ 451] = p_chk & (s_chk == 11'h1cd);
assign corr_mask_w[ 452] = p_chk & (s_chk == 11'h1ce);
assign corr_mask_w[ 453] = p_chk & (s_chk == 11'h1cf);
assign corr_mask_w[ 454] = p_chk & (s_chk == 11'h1d0);
assign corr_mask_w[ 455] = p_chk & (s_chk == 11'h1d1);
assign corr_mask_w[ 456] = p_chk & (s_chk == 11'h1d2);
assign corr_mask_w[ 457] = p_chk & (s_chk == 11'h1d3);
assign corr_mask_w[ 458] = p_chk & (s_chk == 11'h1d4);
assign corr_mask_w[ 459] = p_chk & (s_chk == 11'h1d5);
assign corr_mask_w[ 460] = p_chk & (s_chk == 11'h1d6);
assign corr_mask_w[ 461] = p_chk & (s_chk == 11'h1d7);
assign corr_mask_w[ 462] = p_chk & (s_chk == 11'h1d8);
assign corr_mask_w[ 463] = p_chk & (s_chk == 11'h1d9);
assign corr_mask_w[ 464] = p_chk & (s_chk == 11'h1da);
assign corr_mask_w[ 465] = p_chk & (s_chk == 11'h1db);
assign corr_mask_w[ 466] = p_chk & (s_chk == 11'h1dc);
assign corr_mask_w[ 467] = p_chk & (s_chk == 11'h1dd);
assign corr_mask_w[ 468] = p_chk & (s_chk == 11'h1de);
assign corr_mask_w[ 469] = p_chk & (s_chk == 11'h1df);
assign corr_mask_w[ 470] = p_chk & (s_chk == 11'h1e0);
assign corr_mask_w[ 471] = p_chk & (s_chk == 11'h1e1);
assign corr_mask_w[ 472] = p_chk & (s_chk == 11'h1e2);
assign corr_mask_w[ 473] = p_chk & (s_chk == 11'h1e3);
assign corr_mask_w[ 474] = p_chk & (s_chk == 11'h1e4);
assign corr_mask_w[ 475] = p_chk & (s_chk == 11'h1e5);
assign corr_mask_w[ 476] = p_chk & (s_chk == 11'h1e6);
assign corr_mask_w[ 477] = p_chk & (s_chk == 11'h1e7);
assign corr_mask_w[ 478] = p_chk & (s_chk == 11'h1e8);
assign corr_mask_w[ 479] = p_chk & (s_chk == 11'h1e9);
assign corr_mask_w[ 480] = p_chk & (s_chk == 11'h1ea);
assign corr_mask_w[ 481] = p_chk & (s_chk == 11'h1eb);
assign corr_mask_w[ 482] = p_chk & (s_chk == 11'h1ec);
assign corr_mask_w[ 483] = p_chk & (s_chk == 11'h1ed);
assign corr_mask_w[ 484] = p_chk & (s_chk == 11'h1ee);
assign corr_mask_w[ 485] = p_chk & (s_chk == 11'h1ef);
assign corr_mask_w[ 486] = p_chk & (s_chk == 11'h1f0);
assign corr_mask_w[ 487] = p_chk & (s_chk == 11'h1f1);
assign corr_mask_w[ 488] = p_chk & (s_chk == 11'h1f2);
assign corr_mask_w[ 489] = p_chk & (s_chk == 11'h1f3);
assign corr_mask_w[ 490] = p_chk & (s_chk == 11'h1f4);
assign corr_mask_w[ 491] = p_chk & (s_chk == 11'h1f5);
assign corr_mask_w[ 492] = p_chk & (s_chk == 11'h1f6);
assign corr_mask_w[ 493] = p_chk & (s_chk == 11'h1f7);
assign corr_mask_w[ 494] = p_chk & (s_chk == 11'h1f8);
assign corr_mask_w[ 495] = p_chk & (s_chk == 11'h1f9);
assign corr_mask_w[ 496] = p_chk & (s_chk == 11'h1fa);
assign corr_mask_w[ 497] = p_chk & (s_chk == 11'h1fb);
assign corr_mask_w[ 498] = p_chk & (s_chk == 11'h1fc);
assign corr_mask_w[ 499] = p_chk & (s_chk == 11'h1fd);
assign corr_mask_w[ 500] = p_chk & (s_chk == 11'h1fe);
assign corr_mask_w[ 501] = p_chk & (s_chk == 11'h1ff);
assign corr_mask_w[ 502] = p_chk & (s_chk == 11'h201);
assign corr_mask_w[ 503] = p_chk & (s_chk == 11'h202);
assign corr_mask_w[ 504] = p_chk & (s_chk == 11'h203);
assign corr_mask_w[ 505] = p_chk & (s_chk == 11'h204);
assign corr_mask_w[ 506] = p_chk & (s_chk == 11'h205);
assign corr_mask_w[ 507] = p_chk & (s_chk == 11'h206);
assign corr_mask_w[ 508] = p_chk & (s_chk == 11'h207);
assign corr_mask_w[ 509] = p_chk & (s_chk == 11'h208);
assign corr_mask_w[ 510] = p_chk & (s_chk == 11'h209);
assign corr_mask_w[ 511] = p_chk & (s_chk == 11'h20a);
assign corr_mask_w[ 512] = p_chk & (s_chk == 11'h20b);
assign corr_mask_w[ 513] = p_chk & (s_chk == 11'h20c);
assign corr_mask_w[ 514] = p_chk & (s_chk == 11'h20d);
assign corr_mask_w[ 515] = p_chk & (s_chk == 11'h20e);
assign corr_mask_w[ 516] = p_chk & (s_chk == 11'h20f);
assign corr_mask_w[ 517] = p_chk & (s_chk == 11'h210);
assign corr_mask_w[ 518] = p_chk & (s_chk == 11'h211);
assign corr_mask_w[ 519] = p_chk & (s_chk == 11'h212);
assign corr_mask_w[ 520] = p_chk & (s_chk == 11'h213);
assign corr_mask_w[ 521] = p_chk & (s_chk == 11'h214);
assign corr_mask_w[ 522] = p_chk & (s_chk == 11'h215);
assign corr_mask_w[ 523] = p_chk & (s_chk == 11'h216);
assign corr_mask_w[ 524] = p_chk & (s_chk == 11'h217);
assign corr_mask_w[ 525] = p_chk & (s_chk == 11'h218);
assign corr_mask_w[ 526] = p_chk & (s_chk == 11'h219);
assign corr_mask_w[ 527] = p_chk & (s_chk == 11'h21a);
assign corr_mask_w[ 528] = p_chk & (s_chk == 11'h21b);
assign corr_mask_w[ 529] = p_chk & (s_chk == 11'h21c);
assign corr_mask_w[ 530] = p_chk & (s_chk == 11'h21d);
assign corr_mask_w[ 531] = p_chk & (s_chk == 11'h21e);
assign corr_mask_w[ 532] = p_chk & (s_chk == 11'h21f);
assign corr_mask_w[ 533] = p_chk & (s_chk == 11'h220);
assign corr_mask_w[ 534] = p_chk & (s_chk == 11'h221);
assign corr_mask_w[ 535] = p_chk & (s_chk == 11'h222);
assign corr_mask_w[ 536] = p_chk & (s_chk == 11'h223);
assign corr_mask_w[ 537] = p_chk & (s_chk == 11'h224);
assign corr_mask_w[ 538] = p_chk & (s_chk == 11'h225);
assign corr_mask_w[ 539] = p_chk & (s_chk == 11'h226);
assign corr_mask_w[ 540] = p_chk & (s_chk == 11'h227);
assign corr_mask_w[ 541] = p_chk & (s_chk == 11'h228);
assign corr_mask_w[ 542] = p_chk & (s_chk == 11'h229);
assign corr_mask_w[ 543] = p_chk & (s_chk == 11'h22a);
assign corr_mask_w[ 544] = p_chk & (s_chk == 11'h22b);
assign corr_mask_w[ 545] = p_chk & (s_chk == 11'h22c);
assign corr_mask_w[ 546] = p_chk & (s_chk == 11'h22d);
assign corr_mask_w[ 547] = p_chk & (s_chk == 11'h22e);
assign corr_mask_w[ 548] = p_chk & (s_chk == 11'h22f);
assign corr_mask_w[ 549] = p_chk & (s_chk == 11'h230);
assign corr_mask_w[ 550] = p_chk & (s_chk == 11'h231);
assign corr_mask_w[ 551] = p_chk & (s_chk == 11'h232);
assign corr_mask_w[ 552] = p_chk & (s_chk == 11'h233);
assign corr_mask_w[ 553] = p_chk & (s_chk == 11'h234);
assign corr_mask_w[ 554] = p_chk & (s_chk == 11'h235);
assign corr_mask_w[ 555] = p_chk & (s_chk == 11'h236);
assign corr_mask_w[ 556] = p_chk & (s_chk == 11'h237);
assign corr_mask_w[ 557] = p_chk & (s_chk == 11'h238);
assign corr_mask_w[ 558] = p_chk & (s_chk == 11'h239);
assign corr_mask_w[ 559] = p_chk & (s_chk == 11'h23a);
assign corr_mask_w[ 560] = p_chk & (s_chk == 11'h23b);
assign corr_mask_w[ 561] = p_chk & (s_chk == 11'h23c);
assign corr_mask_w[ 562] = p_chk & (s_chk == 11'h23d);
assign corr_mask_w[ 563] = p_chk & (s_chk == 11'h23e);
assign corr_mask_w[ 564] = p_chk & (s_chk == 11'h23f);
assign corr_mask_w[ 565] = p_chk & (s_chk == 11'h240);
assign corr_mask_w[ 566] = p_chk & (s_chk == 11'h241);
assign corr_mask_w[ 567] = p_chk & (s_chk == 11'h242);
assign corr_mask_w[ 568] = p_chk & (s_chk == 11'h243);
assign corr_mask_w[ 569] = p_chk & (s_chk == 11'h244);
assign corr_mask_w[ 570] = p_chk & (s_chk == 11'h245);
assign corr_mask_w[ 571] = p_chk & (s_chk == 11'h246);
assign corr_mask_w[ 572] = p_chk & (s_chk == 11'h247);
assign corr_mask_w[ 573] = p_chk & (s_chk == 11'h248);
assign corr_mask_w[ 574] = p_chk & (s_chk == 11'h249);
assign corr_mask_w[ 575] = p_chk & (s_chk == 11'h24a);
assign corr_mask_w[ 576] = p_chk & (s_chk == 11'h24b);
assign corr_mask_w[ 577] = p_chk & (s_chk == 11'h24c);
assign corr_mask_w[ 578] = p_chk & (s_chk == 11'h24d);
assign corr_mask_w[ 579] = p_chk & (s_chk == 11'h24e);
assign corr_mask_w[ 580] = p_chk & (s_chk == 11'h24f);
assign corr_mask_w[ 581] = p_chk & (s_chk == 11'h250);
assign corr_mask_w[ 582] = p_chk & (s_chk == 11'h251);
assign corr_mask_w[ 583] = p_chk & (s_chk == 11'h252);
assign corr_mask_w[ 584] = p_chk & (s_chk == 11'h253);
assign corr_mask_w[ 585] = p_chk & (s_chk == 11'h254);
assign corr_mask_w[ 586] = p_chk & (s_chk == 11'h255);
assign corr_mask_w[ 587] = p_chk & (s_chk == 11'h256);
assign corr_mask_w[ 588] = p_chk & (s_chk == 11'h257);
assign corr_mask_w[ 589] = p_chk & (s_chk == 11'h258);
assign corr_mask_w[ 590] = p_chk & (s_chk == 11'h259);
assign corr_mask_w[ 591] = p_chk & (s_chk == 11'h25a);
assign corr_mask_w[ 592] = p_chk & (s_chk == 11'h25b);
assign corr_mask_w[ 593] = p_chk & (s_chk == 11'h25c);
assign corr_mask_w[ 594] = p_chk & (s_chk == 11'h25d);
assign corr_mask_w[ 595] = p_chk & (s_chk == 11'h25e);
assign corr_mask_w[ 596] = p_chk & (s_chk == 11'h25f);
assign corr_mask_w[ 597] = p_chk & (s_chk == 11'h260);
assign corr_mask_w[ 598] = p_chk & (s_chk == 11'h261);
assign corr_mask_w[ 599] = p_chk & (s_chk == 11'h262);
assign corr_mask_w[ 600] = p_chk & (s_chk == 11'h263);
assign corr_mask_w[ 601] = p_chk & (s_chk == 11'h264);
assign corr_mask_w[ 602] = p_chk & (s_chk == 11'h265);
assign corr_mask_w[ 603] = p_chk & (s_chk == 11'h266);
assign corr_mask_w[ 604] = p_chk & (s_chk == 11'h267);
assign corr_mask_w[ 605] = p_chk & (s_chk == 11'h268);
assign corr_mask_w[ 606] = p_chk & (s_chk == 11'h269);
assign corr_mask_w[ 607] = p_chk & (s_chk == 11'h26a);
assign corr_mask_w[ 608] = p_chk & (s_chk == 11'h26b);
assign corr_mask_w[ 609] = p_chk & (s_chk == 11'h26c);
assign corr_mask_w[ 610] = p_chk & (s_chk == 11'h26d);
assign corr_mask_w[ 611] = p_chk & (s_chk == 11'h26e);
assign corr_mask_w[ 612] = p_chk & (s_chk == 11'h26f);
assign corr_mask_w[ 613] = p_chk & (s_chk == 11'h270);
assign corr_mask_w[ 614] = p_chk & (s_chk == 11'h271);
assign corr_mask_w[ 615] = p_chk & (s_chk == 11'h272);
assign corr_mask_w[ 616] = p_chk & (s_chk == 11'h273);
assign corr_mask_w[ 617] = p_chk & (s_chk == 11'h274);
assign corr_mask_w[ 618] = p_chk & (s_chk == 11'h275);
assign corr_mask_w[ 619] = p_chk & (s_chk == 11'h276);
assign corr_mask_w[ 620] = p_chk & (s_chk == 11'h277);
assign corr_mask_w[ 621] = p_chk & (s_chk == 11'h278);
assign corr_mask_w[ 622] = p_chk & (s_chk == 11'h279);
assign corr_mask_w[ 623] = p_chk & (s_chk == 11'h27a);
assign corr_mask_w[ 624] = p_chk & (s_chk == 11'h27b);
assign corr_mask_w[ 625] = p_chk & (s_chk == 11'h27c);
assign corr_mask_w[ 626] = p_chk & (s_chk == 11'h27d);
assign corr_mask_w[ 627] = p_chk & (s_chk == 11'h27e);
assign corr_mask_w[ 628] = p_chk & (s_chk == 11'h27f);
assign corr_mask_w[ 629] = p_chk & (s_chk == 11'h280);
assign corr_mask_w[ 630] = p_chk & (s_chk == 11'h281);
assign corr_mask_w[ 631] = p_chk & (s_chk == 11'h282);
assign corr_mask_w[ 632] = p_chk & (s_chk == 11'h283);
assign corr_mask_w[ 633] = p_chk & (s_chk == 11'h284);
assign corr_mask_w[ 634] = p_chk & (s_chk == 11'h285);
assign corr_mask_w[ 635] = p_chk & (s_chk == 11'h286);
assign corr_mask_w[ 636] = p_chk & (s_chk == 11'h287);
assign corr_mask_w[ 637] = p_chk & (s_chk == 11'h288);
assign corr_mask_w[ 638] = p_chk & (s_chk == 11'h289);
assign corr_mask_w[ 639] = p_chk & (s_chk == 11'h28a);
assign corr_mask_w[ 640] = p_chk & (s_chk == 11'h28b);
assign corr_mask_w[ 641] = p_chk & (s_chk == 11'h28c);
assign corr_mask_w[ 642] = p_chk & (s_chk == 11'h28d);
assign corr_mask_w[ 643] = p_chk & (s_chk == 11'h28e);
assign corr_mask_w[ 644] = p_chk & (s_chk == 11'h28f);
assign corr_mask_w[ 645] = p_chk & (s_chk == 11'h290);
assign corr_mask_w[ 646] = p_chk & (s_chk == 11'h291);
assign corr_mask_w[ 647] = p_chk & (s_chk == 11'h292);
assign corr_mask_w[ 648] = p_chk & (s_chk == 11'h293);
assign corr_mask_w[ 649] = p_chk & (s_chk == 11'h294);
assign corr_mask_w[ 650] = p_chk & (s_chk == 11'h295);
assign corr_mask_w[ 651] = p_chk & (s_chk == 11'h296);
assign corr_mask_w[ 652] = p_chk & (s_chk == 11'h297);
assign corr_mask_w[ 653] = p_chk & (s_chk == 11'h298);
assign corr_mask_w[ 654] = p_chk & (s_chk == 11'h299);
assign corr_mask_w[ 655] = p_chk & (s_chk == 11'h29a);
assign corr_mask_w[ 656] = p_chk & (s_chk == 11'h29b);
assign corr_mask_w[ 657] = p_chk & (s_chk == 11'h29c);
assign corr_mask_w[ 658] = p_chk & (s_chk == 11'h29d);
assign corr_mask_w[ 659] = p_chk & (s_chk == 11'h29e);
assign corr_mask_w[ 660] = p_chk & (s_chk == 11'h29f);
assign corr_mask_w[ 661] = p_chk & (s_chk == 11'h2a0);
assign corr_mask_w[ 662] = p_chk & (s_chk == 11'h2a1);
assign corr_mask_w[ 663] = p_chk & (s_chk == 11'h2a2);
assign corr_mask_w[ 664] = p_chk & (s_chk == 11'h2a3);
assign corr_mask_w[ 665] = p_chk & (s_chk == 11'h2a4);
assign corr_mask_w[ 666] = p_chk & (s_chk == 11'h2a5);
assign corr_mask_w[ 667] = p_chk & (s_chk == 11'h2a6);
assign corr_mask_w[ 668] = p_chk & (s_chk == 11'h2a7);
assign corr_mask_w[ 669] = p_chk & (s_chk == 11'h2a8);
assign corr_mask_w[ 670] = p_chk & (s_chk == 11'h2a9);
assign corr_mask_w[ 671] = p_chk & (s_chk == 11'h2aa);
assign corr_mask_w[ 672] = p_chk & (s_chk == 11'h2ab);
assign corr_mask_w[ 673] = p_chk & (s_chk == 11'h2ac);
assign corr_mask_w[ 674] = p_chk & (s_chk == 11'h2ad);
assign corr_mask_w[ 675] = p_chk & (s_chk == 11'h2ae);
assign corr_mask_w[ 676] = p_chk & (s_chk == 11'h2af);
assign corr_mask_w[ 677] = p_chk & (s_chk == 11'h2b0);
assign corr_mask_w[ 678] = p_chk & (s_chk == 11'h2b1);
assign corr_mask_w[ 679] = p_chk & (s_chk == 11'h2b2);
assign corr_mask_w[ 680] = p_chk & (s_chk == 11'h2b3);
assign corr_mask_w[ 681] = p_chk & (s_chk == 11'h2b4);
assign corr_mask_w[ 682] = p_chk & (s_chk == 11'h2b5);
assign corr_mask_w[ 683] = p_chk & (s_chk == 11'h2b6);
assign corr_mask_w[ 684] = p_chk & (s_chk == 11'h2b7);
assign corr_mask_w[ 685] = p_chk & (s_chk == 11'h2b8);
assign corr_mask_w[ 686] = p_chk & (s_chk == 11'h2b9);
assign corr_mask_w[ 687] = p_chk & (s_chk == 11'h2ba);
assign corr_mask_w[ 688] = p_chk & (s_chk == 11'h2bb);
assign corr_mask_w[ 689] = p_chk & (s_chk == 11'h2bc);
assign corr_mask_w[ 690] = p_chk & (s_chk == 11'h2bd);
assign corr_mask_w[ 691] = p_chk & (s_chk == 11'h2be);
assign corr_mask_w[ 692] = p_chk & (s_chk == 11'h2bf);
assign corr_mask_w[ 693] = p_chk & (s_chk == 11'h2c0);
assign corr_mask_w[ 694] = p_chk & (s_chk == 11'h2c1);
assign corr_mask_w[ 695] = p_chk & (s_chk == 11'h2c2);
assign corr_mask_w[ 696] = p_chk & (s_chk == 11'h2c3);
assign corr_mask_w[ 697] = p_chk & (s_chk == 11'h2c4);
assign corr_mask_w[ 698] = p_chk & (s_chk == 11'h2c5);
assign corr_mask_w[ 699] = p_chk & (s_chk == 11'h2c6);
assign corr_mask_w[ 700] = p_chk & (s_chk == 11'h2c7);
assign corr_mask_w[ 701] = p_chk & (s_chk == 11'h2c8);
assign corr_mask_w[ 702] = p_chk & (s_chk == 11'h2c9);
assign corr_mask_w[ 703] = p_chk & (s_chk == 11'h2ca);
assign corr_mask_w[ 704] = p_chk & (s_chk == 11'h2cb);
assign corr_mask_w[ 705] = p_chk & (s_chk == 11'h2cc);
assign corr_mask_w[ 706] = p_chk & (s_chk == 11'h2cd);
assign corr_mask_w[ 707] = p_chk & (s_chk == 11'h2ce);
assign corr_mask_w[ 708] = p_chk & (s_chk == 11'h2cf);
assign corr_mask_w[ 709] = p_chk & (s_chk == 11'h2d0);
assign corr_mask_w[ 710] = p_chk & (s_chk == 11'h2d1);
assign corr_mask_w[ 711] = p_chk & (s_chk == 11'h2d2);
assign corr_mask_w[ 712] = p_chk & (s_chk == 11'h2d3);
assign corr_mask_w[ 713] = p_chk & (s_chk == 11'h2d4);
assign corr_mask_w[ 714] = p_chk & (s_chk == 11'h2d5);
assign corr_mask_w[ 715] = p_chk & (s_chk == 11'h2d6);
assign corr_mask_w[ 716] = p_chk & (s_chk == 11'h2d7);
assign corr_mask_w[ 717] = p_chk & (s_chk == 11'h2d8);
assign corr_mask_w[ 718] = p_chk & (s_chk == 11'h2d9);
assign corr_mask_w[ 719] = p_chk & (s_chk == 11'h2da);
assign corr_mask_w[ 720] = p_chk & (s_chk == 11'h2db);
assign corr_mask_w[ 721] = p_chk & (s_chk == 11'h2dc);
assign corr_mask_w[ 722] = p_chk & (s_chk == 11'h2dd);
assign corr_mask_w[ 723] = p_chk & (s_chk == 11'h2de);
assign corr_mask_w[ 724] = p_chk & (s_chk == 11'h2df);
assign corr_mask_w[ 725] = p_chk & (s_chk == 11'h2e0);
assign corr_mask_w[ 726] = p_chk & (s_chk == 11'h2e1);
assign corr_mask_w[ 727] = p_chk & (s_chk == 11'h2e2);
assign corr_mask_w[ 728] = p_chk & (s_chk == 11'h2e3);
assign corr_mask_w[ 729] = p_chk & (s_chk == 11'h2e4);
assign corr_mask_w[ 730] = p_chk & (s_chk == 11'h2e5);
assign corr_mask_w[ 731] = p_chk & (s_chk == 11'h2e6);
assign corr_mask_w[ 732] = p_chk & (s_chk == 11'h2e7);
assign corr_mask_w[ 733] = p_chk & (s_chk == 11'h2e8);
assign corr_mask_w[ 734] = p_chk & (s_chk == 11'h2e9);
assign corr_mask_w[ 735] = p_chk & (s_chk == 11'h2ea);
assign corr_mask_w[ 736] = p_chk & (s_chk == 11'h2eb);
assign corr_mask_w[ 737] = p_chk & (s_chk == 11'h2ec);
assign corr_mask_w[ 738] = p_chk & (s_chk == 11'h2ed);
assign corr_mask_w[ 739] = p_chk & (s_chk == 11'h2ee);
assign corr_mask_w[ 740] = p_chk & (s_chk == 11'h2ef);
assign corr_mask_w[ 741] = p_chk & (s_chk == 11'h2f0);
assign corr_mask_w[ 742] = p_chk & (s_chk == 11'h2f1);
assign corr_mask_w[ 743] = p_chk & (s_chk == 11'h2f2);
assign corr_mask_w[ 744] = p_chk & (s_chk == 11'h2f3);
assign corr_mask_w[ 745] = p_chk & (s_chk == 11'h2f4);
assign corr_mask_w[ 746] = p_chk & (s_chk == 11'h2f5);
assign corr_mask_w[ 747] = p_chk & (s_chk == 11'h2f6);
assign corr_mask_w[ 748] = p_chk & (s_chk == 11'h2f7);
assign corr_mask_w[ 749] = p_chk & (s_chk == 11'h2f8);
assign corr_mask_w[ 750] = p_chk & (s_chk == 11'h2f9);
assign corr_mask_w[ 751] = p_chk & (s_chk == 11'h2fa);
assign corr_mask_w[ 752] = p_chk & (s_chk == 11'h2fb);
assign corr_mask_w[ 753] = p_chk & (s_chk == 11'h2fc);
assign corr_mask_w[ 754] = p_chk & (s_chk == 11'h2fd);
assign corr_mask_w[ 755] = p_chk & (s_chk == 11'h2fe);
assign corr_mask_w[ 756] = p_chk & (s_chk == 11'h2ff);
assign corr_mask_w[ 757] = p_chk & (s_chk == 11'h300);
assign corr_mask_w[ 758] = p_chk & (s_chk == 11'h301);
assign corr_mask_w[ 759] = p_chk & (s_chk == 11'h302);
assign corr_mask_w[ 760] = p_chk & (s_chk == 11'h303);
assign corr_mask_w[ 761] = p_chk & (s_chk == 11'h304);
assign corr_mask_w[ 762] = p_chk & (s_chk == 11'h305);
assign corr_mask_w[ 763] = p_chk & (s_chk == 11'h306);
assign corr_mask_w[ 764] = p_chk & (s_chk == 11'h307);
assign corr_mask_w[ 765] = p_chk & (s_chk == 11'h308);
assign corr_mask_w[ 766] = p_chk & (s_chk == 11'h309);
assign corr_mask_w[ 767] = p_chk & (s_chk == 11'h30a);
assign corr_mask_w[ 768] = p_chk & (s_chk == 11'h30b);
assign corr_mask_w[ 769] = p_chk & (s_chk == 11'h30c);
assign corr_mask_w[ 770] = p_chk & (s_chk == 11'h30d);
assign corr_mask_w[ 771] = p_chk & (s_chk == 11'h30e);
assign corr_mask_w[ 772] = p_chk & (s_chk == 11'h30f);
assign corr_mask_w[ 773] = p_chk & (s_chk == 11'h310);
assign corr_mask_w[ 774] = p_chk & (s_chk == 11'h311);
assign corr_mask_w[ 775] = p_chk & (s_chk == 11'h312);
assign corr_mask_w[ 776] = p_chk & (s_chk == 11'h313);
assign corr_mask_w[ 777] = p_chk & (s_chk == 11'h314);
assign corr_mask_w[ 778] = p_chk & (s_chk == 11'h315);
assign corr_mask_w[ 779] = p_chk & (s_chk == 11'h316);
assign corr_mask_w[ 780] = p_chk & (s_chk == 11'h317);
assign corr_mask_w[ 781] = p_chk & (s_chk == 11'h318);
assign corr_mask_w[ 782] = p_chk & (s_chk == 11'h319);
assign corr_mask_w[ 783] = p_chk & (s_chk == 11'h31a);
assign corr_mask_w[ 784] = p_chk & (s_chk == 11'h31b);
assign corr_mask_w[ 785] = p_chk & (s_chk == 11'h31c);
assign corr_mask_w[ 786] = p_chk & (s_chk == 11'h31d);
assign corr_mask_w[ 787] = p_chk & (s_chk == 11'h31e);
assign corr_mask_w[ 788] = p_chk & (s_chk == 11'h31f);
assign corr_mask_w[ 789] = p_chk & (s_chk == 11'h320);
assign corr_mask_w[ 790] = p_chk & (s_chk == 11'h321);
assign corr_mask_w[ 791] = p_chk & (s_chk == 11'h322);
assign corr_mask_w[ 792] = p_chk & (s_chk == 11'h323);
assign corr_mask_w[ 793] = p_chk & (s_chk == 11'h324);
assign corr_mask_w[ 794] = p_chk & (s_chk == 11'h325);
assign corr_mask_w[ 795] = p_chk & (s_chk == 11'h326);
assign corr_mask_w[ 796] = p_chk & (s_chk == 11'h327);
assign corr_mask_w[ 797] = p_chk & (s_chk == 11'h328);
assign corr_mask_w[ 798] = p_chk & (s_chk == 11'h329);
assign corr_mask_w[ 799] = p_chk & (s_chk == 11'h32a);
assign corr_mask_w[ 800] = p_chk & (s_chk == 11'h32b);
assign corr_mask_w[ 801] = p_chk & (s_chk == 11'h32c);
assign corr_mask_w[ 802] = p_chk & (s_chk == 11'h32d);
assign corr_mask_w[ 803] = p_chk & (s_chk == 11'h32e);
assign corr_mask_w[ 804] = p_chk & (s_chk == 11'h32f);
assign corr_mask_w[ 805] = p_chk & (s_chk == 11'h330);
assign corr_mask_w[ 806] = p_chk & (s_chk == 11'h331);
assign corr_mask_w[ 807] = p_chk & (s_chk == 11'h332);
assign corr_mask_w[ 808] = p_chk & (s_chk == 11'h333);
assign corr_mask_w[ 809] = p_chk & (s_chk == 11'h334);
assign corr_mask_w[ 810] = p_chk & (s_chk == 11'h335);
assign corr_mask_w[ 811] = p_chk & (s_chk == 11'h336);
assign corr_mask_w[ 812] = p_chk & (s_chk == 11'h337);
assign corr_mask_w[ 813] = p_chk & (s_chk == 11'h338);
assign corr_mask_w[ 814] = p_chk & (s_chk == 11'h339);
assign corr_mask_w[ 815] = p_chk & (s_chk == 11'h33a);
assign corr_mask_w[ 816] = p_chk & (s_chk == 11'h33b);
assign corr_mask_w[ 817] = p_chk & (s_chk == 11'h33c);
assign corr_mask_w[ 818] = p_chk & (s_chk == 11'h33d);
assign corr_mask_w[ 819] = p_chk & (s_chk == 11'h33e);
assign corr_mask_w[ 820] = p_chk & (s_chk == 11'h33f);
assign corr_mask_w[ 821] = p_chk & (s_chk == 11'h340);
assign corr_mask_w[ 822] = p_chk & (s_chk == 11'h341);
assign corr_mask_w[ 823] = p_chk & (s_chk == 11'h342);
assign corr_mask_w[ 824] = p_chk & (s_chk == 11'h343);
assign corr_mask_w[ 825] = p_chk & (s_chk == 11'h344);
assign corr_mask_w[ 826] = p_chk & (s_chk == 11'h345);
assign corr_mask_w[ 827] = p_chk & (s_chk == 11'h346);
assign corr_mask_w[ 828] = p_chk & (s_chk == 11'h347);
assign corr_mask_w[ 829] = p_chk & (s_chk == 11'h348);
assign corr_mask_w[ 830] = p_chk & (s_chk == 11'h349);
assign corr_mask_w[ 831] = p_chk & (s_chk == 11'h34a);
assign corr_mask_w[ 832] = p_chk & (s_chk == 11'h34b);
assign corr_mask_w[ 833] = p_chk & (s_chk == 11'h34c);
assign corr_mask_w[ 834] = p_chk & (s_chk == 11'h34d);
assign corr_mask_w[ 835] = p_chk & (s_chk == 11'h34e);
assign corr_mask_w[ 836] = p_chk & (s_chk == 11'h34f);
assign corr_mask_w[ 837] = p_chk & (s_chk == 11'h350);
assign corr_mask_w[ 838] = p_chk & (s_chk == 11'h351);
assign corr_mask_w[ 839] = p_chk & (s_chk == 11'h352);
assign corr_mask_w[ 840] = p_chk & (s_chk == 11'h353);
assign corr_mask_w[ 841] = p_chk & (s_chk == 11'h354);
assign corr_mask_w[ 842] = p_chk & (s_chk == 11'h355);
assign corr_mask_w[ 843] = p_chk & (s_chk == 11'h356);
assign corr_mask_w[ 844] = p_chk & (s_chk == 11'h357);
assign corr_mask_w[ 845] = p_chk & (s_chk == 11'h358);
assign corr_mask_w[ 846] = p_chk & (s_chk == 11'h359);
assign corr_mask_w[ 847] = p_chk & (s_chk == 11'h35a);
assign corr_mask_w[ 848] = p_chk & (s_chk == 11'h35b);
assign corr_mask_w[ 849] = p_chk & (s_chk == 11'h35c);
assign corr_mask_w[ 850] = p_chk & (s_chk == 11'h35d);
assign corr_mask_w[ 851] = p_chk & (s_chk == 11'h35e);
assign corr_mask_w[ 852] = p_chk & (s_chk == 11'h35f);
assign corr_mask_w[ 853] = p_chk & (s_chk == 11'h360);
assign corr_mask_w[ 854] = p_chk & (s_chk == 11'h361);
assign corr_mask_w[ 855] = p_chk & (s_chk == 11'h362);
assign corr_mask_w[ 856] = p_chk & (s_chk == 11'h363);
assign corr_mask_w[ 857] = p_chk & (s_chk == 11'h364);
assign corr_mask_w[ 858] = p_chk & (s_chk == 11'h365);
assign corr_mask_w[ 859] = p_chk & (s_chk == 11'h366);
assign corr_mask_w[ 860] = p_chk & (s_chk == 11'h367);
assign corr_mask_w[ 861] = p_chk & (s_chk == 11'h368);
assign corr_mask_w[ 862] = p_chk & (s_chk == 11'h369);
assign corr_mask_w[ 863] = p_chk & (s_chk == 11'h36a);
assign corr_mask_w[ 864] = p_chk & (s_chk == 11'h36b);
assign corr_mask_w[ 865] = p_chk & (s_chk == 11'h36c);
assign corr_mask_w[ 866] = p_chk & (s_chk == 11'h36d);
assign corr_mask_w[ 867] = p_chk & (s_chk == 11'h36e);
assign corr_mask_w[ 868] = p_chk & (s_chk == 11'h36f);
assign corr_mask_w[ 869] = p_chk & (s_chk == 11'h370);
assign corr_mask_w[ 870] = p_chk & (s_chk == 11'h371);
assign corr_mask_w[ 871] = p_chk & (s_chk == 11'h372);
assign corr_mask_w[ 872] = p_chk & (s_chk == 11'h373);
assign corr_mask_w[ 873] = p_chk & (s_chk == 11'h374);
assign corr_mask_w[ 874] = p_chk & (s_chk == 11'h375);
assign corr_mask_w[ 875] = p_chk & (s_chk == 11'h376);
assign corr_mask_w[ 876] = p_chk & (s_chk == 11'h377);
assign corr_mask_w[ 877] = p_chk & (s_chk == 11'h378);
assign corr_mask_w[ 878] = p_chk & (s_chk == 11'h379);
assign corr_mask_w[ 879] = p_chk & (s_chk == 11'h37a);
assign corr_mask_w[ 880] = p_chk & (s_chk == 11'h37b);
assign corr_mask_w[ 881] = p_chk & (s_chk == 11'h37c);
assign corr_mask_w[ 882] = p_chk & (s_chk == 11'h37d);
assign corr_mask_w[ 883] = p_chk & (s_chk == 11'h37e);
assign corr_mask_w[ 884] = p_chk & (s_chk == 11'h37f);
assign corr_mask_w[ 885] = p_chk & (s_chk == 11'h380);
assign corr_mask_w[ 886] = p_chk & (s_chk == 11'h381);
assign corr_mask_w[ 887] = p_chk & (s_chk == 11'h382);
assign corr_mask_w[ 888] = p_chk & (s_chk == 11'h383);
assign corr_mask_w[ 889] = p_chk & (s_chk == 11'h384);
assign corr_mask_w[ 890] = p_chk & (s_chk == 11'h385);
assign corr_mask_w[ 891] = p_chk & (s_chk == 11'h386);
assign corr_mask_w[ 892] = p_chk & (s_chk == 11'h387);
assign corr_mask_w[ 893] = p_chk & (s_chk == 11'h388);
assign corr_mask_w[ 894] = p_chk & (s_chk == 11'h389);
assign corr_mask_w[ 895] = p_chk & (s_chk == 11'h38a);
assign corr_mask_w[ 896] = p_chk & (s_chk == 11'h38b);
assign corr_mask_w[ 897] = p_chk & (s_chk == 11'h38c);
assign corr_mask_w[ 898] = p_chk & (s_chk == 11'h38d);
assign corr_mask_w[ 899] = p_chk & (s_chk == 11'h38e);
assign corr_mask_w[ 900] = p_chk & (s_chk == 11'h38f);
assign corr_mask_w[ 901] = p_chk & (s_chk == 11'h390);
assign corr_mask_w[ 902] = p_chk & (s_chk == 11'h391);
assign corr_mask_w[ 903] = p_chk & (s_chk == 11'h392);
assign corr_mask_w[ 904] = p_chk & (s_chk == 11'h393);
assign corr_mask_w[ 905] = p_chk & (s_chk == 11'h394);
assign corr_mask_w[ 906] = p_chk & (s_chk == 11'h395);
assign corr_mask_w[ 907] = p_chk & (s_chk == 11'h396);
assign corr_mask_w[ 908] = p_chk & (s_chk == 11'h397);
assign corr_mask_w[ 909] = p_chk & (s_chk == 11'h398);
assign corr_mask_w[ 910] = p_chk & (s_chk == 11'h399);
assign corr_mask_w[ 911] = p_chk & (s_chk == 11'h39a);
assign corr_mask_w[ 912] = p_chk & (s_chk == 11'h39b);
assign corr_mask_w[ 913] = p_chk & (s_chk == 11'h39c);
assign corr_mask_w[ 914] = p_chk & (s_chk == 11'h39d);
assign corr_mask_w[ 915] = p_chk & (s_chk == 11'h39e);
assign corr_mask_w[ 916] = p_chk & (s_chk == 11'h39f);
assign corr_mask_w[ 917] = p_chk & (s_chk == 11'h3a0);
assign corr_mask_w[ 918] = p_chk & (s_chk == 11'h3a1);
assign corr_mask_w[ 919] = p_chk & (s_chk == 11'h3a2);
assign corr_mask_w[ 920] = p_chk & (s_chk == 11'h3a3);
assign corr_mask_w[ 921] = p_chk & (s_chk == 11'h3a4);
assign corr_mask_w[ 922] = p_chk & (s_chk == 11'h3a5);
assign corr_mask_w[ 923] = p_chk & (s_chk == 11'h3a6);
assign corr_mask_w[ 924] = p_chk & (s_chk == 11'h3a7);
assign corr_mask_w[ 925] = p_chk & (s_chk == 11'h3a8);
assign corr_mask_w[ 926] = p_chk & (s_chk == 11'h3a9);
assign corr_mask_w[ 927] = p_chk & (s_chk == 11'h3aa);
assign corr_mask_w[ 928] = p_chk & (s_chk == 11'h3ab);
assign corr_mask_w[ 929] = p_chk & (s_chk == 11'h3ac);
assign corr_mask_w[ 930] = p_chk & (s_chk == 11'h3ad);
assign corr_mask_w[ 931] = p_chk & (s_chk == 11'h3ae);
assign corr_mask_w[ 932] = p_chk & (s_chk == 11'h3af);
assign corr_mask_w[ 933] = p_chk & (s_chk == 11'h3b0);
assign corr_mask_w[ 934] = p_chk & (s_chk == 11'h3b1);
assign corr_mask_w[ 935] = p_chk & (s_chk == 11'h3b2);
assign corr_mask_w[ 936] = p_chk & (s_chk == 11'h3b3);
assign corr_mask_w[ 937] = p_chk & (s_chk == 11'h3b4);
assign corr_mask_w[ 938] = p_chk & (s_chk == 11'h3b5);
assign corr_mask_w[ 939] = p_chk & (s_chk == 11'h3b6);
assign corr_mask_w[ 940] = p_chk & (s_chk == 11'h3b7);
assign corr_mask_w[ 941] = p_chk & (s_chk == 11'h3b8);
assign corr_mask_w[ 942] = p_chk & (s_chk == 11'h3b9);
assign corr_mask_w[ 943] = p_chk & (s_chk == 11'h3ba);
assign corr_mask_w[ 944] = p_chk & (s_chk == 11'h3bb);
assign corr_mask_w[ 945] = p_chk & (s_chk == 11'h3bc);
assign corr_mask_w[ 946] = p_chk & (s_chk == 11'h3bd);
assign corr_mask_w[ 947] = p_chk & (s_chk == 11'h3be);
assign corr_mask_w[ 948] = p_chk & (s_chk == 11'h3bf);
assign corr_mask_w[ 949] = p_chk & (s_chk == 11'h3c0);
assign corr_mask_w[ 950] = p_chk & (s_chk == 11'h3c1);
assign corr_mask_w[ 951] = p_chk & (s_chk == 11'h3c2);
assign corr_mask_w[ 952] = p_chk & (s_chk == 11'h3c3);
assign corr_mask_w[ 953] = p_chk & (s_chk == 11'h3c4);
assign corr_mask_w[ 954] = p_chk & (s_chk == 11'h3c5);
assign corr_mask_w[ 955] = p_chk & (s_chk == 11'h3c6);
assign corr_mask_w[ 956] = p_chk & (s_chk == 11'h3c7);
assign corr_mask_w[ 957] = p_chk & (s_chk == 11'h3c8);
assign corr_mask_w[ 958] = p_chk & (s_chk == 11'h3c9);
assign corr_mask_w[ 959] = p_chk & (s_chk == 11'h3ca);
assign corr_mask_w[ 960] = p_chk & (s_chk == 11'h3cb);
assign corr_mask_w[ 961] = p_chk & (s_chk == 11'h3cc);
assign corr_mask_w[ 962] = p_chk & (s_chk == 11'h3cd);
assign corr_mask_w[ 963] = p_chk & (s_chk == 11'h3ce);
assign corr_mask_w[ 964] = p_chk & (s_chk == 11'h3cf);
assign corr_mask_w[ 965] = p_chk & (s_chk == 11'h3d0);
assign corr_mask_w[ 966] = p_chk & (s_chk == 11'h3d1);
assign corr_mask_w[ 967] = p_chk & (s_chk == 11'h3d2);
assign corr_mask_w[ 968] = p_chk & (s_chk == 11'h3d3);
assign corr_mask_w[ 969] = p_chk & (s_chk == 11'h3d4);
assign corr_mask_w[ 970] = p_chk & (s_chk == 11'h3d5);
assign corr_mask_w[ 971] = p_chk & (s_chk == 11'h3d6);
assign corr_mask_w[ 972] = p_chk & (s_chk == 11'h3d7);
assign corr_mask_w[ 973] = p_chk & (s_chk == 11'h3d8);
assign corr_mask_w[ 974] = p_chk & (s_chk == 11'h3d9);
assign corr_mask_w[ 975] = p_chk & (s_chk == 11'h3da);
assign corr_mask_w[ 976] = p_chk & (s_chk == 11'h3db);
assign corr_mask_w[ 977] = p_chk & (s_chk == 11'h3dc);
assign corr_mask_w[ 978] = p_chk & (s_chk == 11'h3dd);
assign corr_mask_w[ 979] = p_chk & (s_chk == 11'h3de);
assign corr_mask_w[ 980] = p_chk & (s_chk == 11'h3df);
assign corr_mask_w[ 981] = p_chk & (s_chk == 11'h3e0);
assign corr_mask_w[ 982] = p_chk & (s_chk == 11'h3e1);
assign corr_mask_w[ 983] = p_chk & (s_chk == 11'h3e2);
assign corr_mask_w[ 984] = p_chk & (s_chk == 11'h3e3);
assign corr_mask_w[ 985] = p_chk & (s_chk == 11'h3e4);
assign corr_mask_w[ 986] = p_chk & (s_chk == 11'h3e5);
assign corr_mask_w[ 987] = p_chk & (s_chk == 11'h3e6);
assign corr_mask_w[ 988] = p_chk & (s_chk == 11'h3e7);
assign corr_mask_w[ 989] = p_chk & (s_chk == 11'h3e8);
assign corr_mask_w[ 990] = p_chk & (s_chk == 11'h3e9);
assign corr_mask_w[ 991] = p_chk & (s_chk == 11'h3ea);
assign corr_mask_w[ 992] = p_chk & (s_chk == 11'h3eb);
assign corr_mask_w[ 993] = p_chk & (s_chk == 11'h3ec);
assign corr_mask_w[ 994] = p_chk & (s_chk == 11'h3ed);
assign corr_mask_w[ 995] = p_chk & (s_chk == 11'h3ee);
assign corr_mask_w[ 996] = p_chk & (s_chk == 11'h3ef);
assign corr_mask_w[ 997] = p_chk & (s_chk == 11'h3f0);
assign corr_mask_w[ 998] = p_chk & (s_chk == 11'h3f1);
assign corr_mask_w[ 999] = p_chk & (s_chk == 11'h3f2);
assign corr_mask_w[1000] = p_chk & (s_chk == 11'h3f3);
assign corr_mask_w[1001] = p_chk & (s_chk == 11'h3f4);
assign corr_mask_w[1002] = p_chk & (s_chk == 11'h3f5);
assign corr_mask_w[1003] = p_chk & (s_chk == 11'h3f6);
assign corr_mask_w[1004] = p_chk & (s_chk == 11'h3f7);
assign corr_mask_w[1005] = p_chk & (s_chk == 11'h3f8);
assign corr_mask_w[1006] = p_chk & (s_chk == 11'h3f9);
assign corr_mask_w[1007] = p_chk & (s_chk == 11'h3fa);

assign corr_unit_o = unit_i ^ {16'd0, corr_mask_w};  // 纠正后数据（E 拍；无错/字段位错/P 单错 = unit_i 原值；槽/tie[1023:1008] 恒 0 不纠正）
assign pass_o = (en_ck & flit_valid_i & ( (s_chk == 11'h0) | data_hit_w | parity_err_w ))
              | (~en_ck & flit_valid_i);                     // 禁用态 E 拍直通
assign err_o  =  en_ck & flit_valid_i & uncorr_w;

// ---- 2026-09-20 probe（design 波形定位 ECC-overwrite 源）----
//   ECC_EN 下首数据单元 mismatch=bit433：打印每判通单元的 syndrome / data_hit / mask，
//   定位 unit[433] 被 corr 改成 1 的触发（无错却纠正 ⭢ corr_mask 误判；或线上确有单错被正确纠正）。
//   仅仿真探头，不综合（ifdef 仿真）。
`ifdef TEST_ONLY_PRINT
always @(posedge mb_clk) begin
    if (en_ck && flit_valid_i)
        $display("[ECC][%0t] syndrome=%03x hit_data=%b parity=%b uncorr=%b mask_w[433]=%b unit_i[433]=%b corr_unit[433]=%b",
                 $time, syndrome_w, data_hit_w, parity_err_w, uncorr_w,
                 corr_mask_w[433], unit_i[433], corr_unit_o[433]);
end
`endif

endmodule
