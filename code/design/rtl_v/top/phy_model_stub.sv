// =============================================================================
// phy_model_stub.v — HIPI_2D_D2D_PHY_TOP 空壳（WP2 集成编译验证用）
//
//   端口骨架 1:1 拷贝自 code/model/HIPI_2D_D2D_PHY_MODEL.sv 顶层端口区
//   （名称/方向/位宽/类型逐项一致，未改 model 内部代码）。内部逻辑为空：
//   输出默认 x，仅用于 d2d_hipi_top ANA 形态的 elaboration/集成连通性验证，
//   不可用于功能仿真。功能验证请用真 model（OpenHappy Team 工具链）。
//
//   生成：2026-09-22 WP2；源 model 端口变更需同步重生成本文件。
//
//   注意：源 model 的 real 数组口 RX [15:0] / TX [15:0] 在此拍平为逐元素
//   标量（RX_0..RX_15 / TX_0..TX_15）——d2d_ana_top.v 为 Verilog-2001 解析
//   （real memory 非法），real 数组 net 无法从 .v 侧连接；其余 real 数组口
//   （RES_TX/RES_RX/RES_RX_CLK，ana 侧悬空）保留原形。
// =============================================================================
module HIPI_2D_D2D_PHY_TOP (

    //************************* INPUT **************************//
    //  input Bump
    input   real            RX_0,
    input   real            RX_1,
    input   real            RX_2,
    input   real            RX_3,
    input   real            RX_4,
    input   real            RX_5,
    input   real            RX_6,
    input   real            RX_7,
    input   real            RX_8,
    input   real            RX_9,
    input   real            RX_10,
    input   real            RX_11,
    input   real            RX_12,
    input   real            RX_13,
    input   real            RX_14,
    input   real            RX_15,
    input   real            RXDRFU,
    input   logic           RXCP,
    input   logic           RXCN,
    input   logic           SB_RXC,
    input   logic           SB_RXD,
    input   logic           SB_RXR,
    input   logic           RXLPC,

    // CLK From PLL
    input   logic           CLKP_PLL,
    input   logic           CLKN_PLL,
    input   logic           CLK_CPU_PLL,

    // TX DATA IN, LSB First
    input   logic [31:0]    DIN0,
    input   logic [31:0]    DIN1,
    input   logic [31:0]    DIN2,
    input   logic [31:0]    DIN3,
    input   logic [31:0]    DIN4,
    input   logic [31:0]    DIN5,
    input   logic [31:0]    DIN6,
    input   logic [31:0]    DIN7,
    input   logic [31:0]    DIN8,
    input   logic [31:0]    DIN9,
    input   logic [31:0]    DIN10,
    input   logic [31:0]    DIN11,
    input   logic [31:0]    DIN12,
    input   logic [31:0]    DIN13,
    input   logic [31:0]    DIN14,
    input   logic [31:0]    DIN15,
    input   logic [31:0]    DIN_RFU,

    // TRX CLK SYNC
    input   logic           TX_EN_ASYNC,
    input   logic           TX_EN_SYNC,
    input   logic           TX_EN_TRACK,
    input   logic           TX_RSTB_SYNC,
    input   logic           RX_CLK_EN_ASYNC,
    input   logic           RX_CLK_EN_SYNC,
    input   logic           RX_CLK_EN_TRACK,
    input   logic           RX_RSTB_SYNC,

    // OFFSET CAL CTRL
    input   logic           RX_OFFSET_CALIBREATION_ON,

    // RX Vref Ctrl BIN
    input   logic [7:0]     RX_VREF_CTRL_MAIN,
    input   logic [7:0]     RX_VREF_CTRL_ERR_EDGE,

    // RX DCDL Ctrl BIN, 16 for RFU
    input   logic [5:0]     RX0_DCDL_CTRL,
    input   logic [5:0]     RX1_DCDL_CTRL,
    input   logic [5:0]     RX2_DCDL_CTRL,
    input   logic [5:0]     RX3_DCDL_CTRL,
    input   logic [5:0]     RX4_DCDL_CTRL,
    input   logic [5:0]     RX5_DCDL_CTRL,
    input   logic [5:0]     RX6_DCDL_CTRL,
    input   logic [5:0]     RX7_DCDL_CTRL,
    input   logic [5:0]     RX8_DCDL_CTRL,
    input   logic [5:0]     RX9_DCDL_CTRL,
    input   logic [5:0]     RX10_DCDL_CTRL,
    input   logic [5:0]     RX11_DCDL_CTRL,
    input   logic [5:0]     RX12_DCDL_CTRL,
    input   logic [5:0]     RX13_DCDL_CTRL,
    input   logic [5:0]     RX14_DCDL_CTRL,
    input   logic [5:0]     RX15_DCDL_CTRL,
    input   logic [5:0]     RX16_DCDL_CTRL,

    // RX PI Ctrl Word, Gray code!
    input   logic [6:0]     PH_045_GRAY_PI,
    input   logic [6:0]     PH_I_GRAY_PI,
    input   logic [6:0]     PH_Q_GRAY_PI,

    // RX OFFSET CAL BUS, 16 for RFU
    input   logic           OFFSET_CTRL_CLK,
    input   logic           OFFSET_RESET_N,

    input   logic           RX0_OFFSET_CTRL_WRITE,
    input   logic           RX1_OFFSET_CTRL_WRITE,
    input   logic           RX2_OFFSET_CTRL_WRITE,
    input   logic           RX3_OFFSET_CTRL_WRITE,
    input   logic           RX4_OFFSET_CTRL_WRITE,
    input   logic           RX5_OFFSET_CTRL_WRITE,
    input   logic           RX6_OFFSET_CTRL_WRITE,
    input   logic           RX7_OFFSET_CTRL_WRITE,
    input   logic           RX8_OFFSET_CTRL_WRITE,
    input   logic           RX9_OFFSET_CTRL_WRITE,
    input   logic           RX10_OFFSET_CTRL_WRITE,
    input   logic           RX11_OFFSET_CTRL_WRITE,
    input   logic           RX12_OFFSET_CTRL_WRITE,
    input   logic           RX13_OFFSET_CTRL_WRITE,
    input   logic           RX14_OFFSET_CTRL_WRITE,
    input   logic           RX15_OFFSET_CTRL_WRITE,
    input   logic           RX16_OFFSET_CTRL_WRITE,

    input   logic [2:0]     RX0_OFFSET_CTRL_ADDR,
    input   logic [2:0]     RX1_OFFSET_CTRL_ADDR,
    input   logic [2:0]     RX2_OFFSET_CTRL_ADDR,
    input   logic [2:0]     RX3_OFFSET_CTRL_ADDR,
    input   logic [2:0]     RX4_OFFSET_CTRL_ADDR,
    input   logic [2:0]     RX5_OFFSET_CTRL_ADDR,
    input   logic [2:0]     RX6_OFFSET_CTRL_ADDR,
    input   logic [2:0]     RX7_OFFSET_CTRL_ADDR,
    input   logic [2:0]     RX8_OFFSET_CTRL_ADDR,
    input   logic [2:0]     RX9_OFFSET_CTRL_ADDR,
    input   logic [2:0]     RX10_OFFSET_CTRL_ADDR,
    input   logic [2:0]     RX11_OFFSET_CTRL_ADDR,
    input   logic [2:0]     RX12_OFFSET_CTRL_ADDR,
    input   logic [2:0]     RX13_OFFSET_CTRL_ADDR,
    input   logic [2:0]     RX14_OFFSET_CTRL_ADDR,
    input   logic [2:0]     RX15_OFFSET_CTRL_ADDR,
    input   logic [2:0]     RX16_OFFSET_CTRL_ADDR,

    inout   wire [9:0]     RX0_OFFSET_CTRL_DATA,
    inout   wire [9:0]     RX1_OFFSET_CTRL_DATA,
    inout   wire [9:0]     RX2_OFFSET_CTRL_DATA,
    inout   wire [9:0]     RX3_OFFSET_CTRL_DATA,
    inout   wire [9:0]     RX4_OFFSET_CTRL_DATA,
    inout   wire [9:0]     RX5_OFFSET_CTRL_DATA,
    inout   wire [9:0]     RX6_OFFSET_CTRL_DATA,
    inout   wire [9:0]     RX7_OFFSET_CTRL_DATA,
    inout   wire [9:0]     RX8_OFFSET_CTRL_DATA,
    inout   wire [9:0]     RX9_OFFSET_CTRL_DATA,
    inout   wire [9:0]     RX10_OFFSET_CTRL_DATA,
    inout   wire [9:0]     RX11_OFFSET_CTRL_DATA,
    inout   wire [9:0]     RX12_OFFSET_CTRL_DATA,
    inout   wire [9:0]     RX13_OFFSET_CTRL_DATA,
    inout   wire [9:0]     RX14_OFFSET_CTRL_DATA,
    inout   wire [9:0]     RX15_OFFSET_CTRL_DATA,
    inout   wire [9:0]     RX16_OFFSET_CTRL_DATA,

    // TRX CTRL REG, Must Be Connect!
    input   logic [31:0]    TRX_REG1_WRITE_IN,          // SYNC By sideban CLK
    input   logic [31:0]    TRX_REG2_WRITE_IN,
    input   logic [31:0]    TRX_TEST_REG1_WRITE_IN,
    input   logic [31:0]    TRX_TEST_REG2_WRITE_IN,

    // RX CTRL REG
    input   logic [31:0]    RX_CLK_ENG_REG1_WRITE_IN,
    input   logic [31:0]    RX_CLK_ENG_REG2_WRITE_IN,
    input   logic [31:0]    RX_CLK_ENG_REG3_WRITE_IN,
    input   logic [31:0]    RX_CLK_ENG_REG4_WRITE_IN,
    input   logic [31:0]    RX_CLK_REG1_WRITE_IN,
    input   logic [31:0]    RX_CLK_REG2_WRITE_IN,
    input   logic [31:0]    RX_CLK_REG3_WRITE_IN,
    input   logic [31:0]    RX_CLK_REG4_WRITE_IN,
    input   logic [31:0]    RX_DATA_REG1_WRITE_IN,
    input   logic [31:0]    RX_DATA_REG2_WRITE_IN,
    input   logic [31:0]    RX_DATA_REG3_WRITE_IN,
    input   logic [31:0]    RX_LDO_REG1_WRITE_IN,
    input   logic [31:0]    RX_LDO_REG2_WRITE_IN,

    // TX CTRL REG, Must Be Connect
    input   logic [31:0]    TX_CLK_ENG_REG1_WRITE_IN,
    input   logic [31:0]    TX_CLK_ENG_REG2_WRITE_IN,
    input   logic [31:0]    TX_CLK_ENG_REG3_WRITE_IN,
    input   logic [31:0]    TX_CLK_ENG_REG4_WRITE_IN,
    input   logic [31:0]    TX_CLK_REG1_WRITE_IN,
    input   logic [31:0]    TX_CLK_REG2_WRITE_IN,
    input   logic [31:0]    TX_CLK_REG3_WRITE_IN,
    input   logic [31:0]    TX_CLK_REG4_WRITE_IN,
    input   logic [31:0]    TX_CLK_REG5_WRITE_IN,
    input   logic [31:0]    TX_DATA_REG1_WRITE_IN,
    input   logic [31:0]    TX_DATA_REG2_WRITE_IN,
    input   logic [31:0]    TX_DATA_REG3_WRITE_IN,
    input   logic [31:0]    TX_DATA_REG4_WRITE_IN,
    input   logic [31:0]    TX_DATA_REG5_WRITE_IN,
    input   logic [31:0]    TX_DATA_REG6_WRITE_IN,
    input   logic [31:0]    TX_DATA_REG7_WRITE_IN,
    input   logic [31:0]    TX_LDO_REG1_WRITE_IN,
    input   logic [31:0]    TX_LDO_REG2_WRITE_IN,

    // TX LPC IN
    input   logic           TX_LPC_DATA_IN,

    // TX Sideband IN, Need to Sync to CLK_Logic_PHY!
    input   logic           TX_SBC_CLK_LOGIC_IN,
    input   logic           TX_SBC_DATA_LOGIC_IN,
    input   logic           TX_SBC_REPAIR_LOGIC_IN,

    // Analog input, For BandGap I&V Reference
    input   real            VREF_0P4_BG,
    input   real            IREF_5UA_BG,            // not use in model, but exist in LEF

    //************************* OUTPUT *************************//
    // output Bump
    output  real            TX_0,
    output  real            TX_1,
    output  real            TX_2,
    output  real            TX_3,
    output  real            TX_4,
    output  real            TX_5,
    output  real            TX_6,
    output  real            TX_7,
    output  real            TX_8,
    output  real            TX_9,
    output  real            TX_10,
    output  real            TX_11,
    output  real            TX_12,
    output  real            TX_13,
    output  real            TX_14,
    output  real            TX_15,
    output  real            TXDRFU,

    output  logic           TXCP,
    output  logic           TXCN,

    output  logic           TX_LPC,

    output  logic           SB_TXC,
    output  logic           SB_TXD,
    output  logic           SB_TXR,

    // output RX DATA, LSB First
    output  logic [31:0]    DOUT0,
    output  logic [31:0]    DOUT1,
    output  logic [31:0]    DOUT2,
    output  logic [31:0]    DOUT3,
    output  logic [31:0]    DOUT4,
    output  logic [31:0]    DOUT5,
    output  logic [31:0]    DOUT6,
    output  logic [31:0]    DOUT7,
    output  logic [31:0]    DOUT8,
    output  logic [31:0]    DOUT9,
    output  logic [31:0]    DOUT10,
    output  logic [31:0]    DOUT11,
    output  logic [31:0]    DOUT12,
    output  logic [31:0]    DOUT13,
    output  logic [31:0]    DOUT14,
    output  logic [31:0]    DOUT15,
    output  logic [31:0]    DOUT_RFU,

    // output RX DATA, ERR Edge, LSB First
    output  logic [3:0]     DOUT0_ERR_EDGE,
    output  logic [3:0]     DOUT1_ERR_EDGE,
    output  logic [3:0]     DOUT2_ERR_EDGE,
    output  logic [3:0]     DOUT3_ERR_EDGE,
    output  logic [3:0]     DOUT4_ERR_EDGE,
    output  logic [3:0]     DOUT5_ERR_EDGE,
    output  logic [3:0]     DOUT6_ERR_EDGE,
    output  logic [3:0]     DOUT7_ERR_EDGE,
    output  logic [3:0]     DOUT8_ERR_EDGE,
    output  logic [3:0]     DOUT9_ERR_EDGE,
    output  logic [3:0]     DOUT10_ERR_EDGE,
    output  logic [3:0]     DOUT11_ERR_EDGE,
    output  logic [3:0]     DOUT12_ERR_EDGE,
    output  logic [3:0]     DOUT13_ERR_EDGE,
    output  logic [3:0]     DOUT14_ERR_EDGE,
    output  logic [3:0]     DOUT15_ERR_EDGE,
    output  logic [3:0]     DOUTRFU_ERR_EDGE,

    // RX logic PHY CLK output, Duty cycle = 50%
    output  logic           RX_CLK_DIG_OUT,

    // TX logic PHY CLK output, Duty cycle = 50%
    output  logic           TX_CLK_DIG_OUT,

    // Sideband CLk output, Duty Cycle = 50%
    output  logic           SB_CLK_OUT,

    // TRX REG Read Out, Must Be Connect
    output  logic [31:0]    RX_CLK_ENG_REG_READ_OUT,
    output  logic [31:0]    TX_CLK_ENG_REG_READ_OUT,
    output  logic           CONNECT_DIG_OUT,

    // LPC output
    output  logic           RX_LPC_DATA_OUT,

    // Sideband output
    output  logic           RX_SBC_CLK_LOGIC_OUT,
    output  logic           RX_SBC_DATA_LOGIC_OUT,
    output  logic           RX_SBC_REPAIR_LOGIC_OUT,

    // Analog output, Need connect to Analog output Bump
    // IF Bump Limited, CLK_TEST and DC_TEST can be tie together
    // Need ESD protect, Speed MAX : 10KHz, Voltage MAX: 0.88V
    output  real            CLK_TEST,
    output  real            DC_TEST,

    //***************************** Sim inout put ******************************//
    // For Model, only need in sim
    // Don't exist  in phy lef
    input   logic           clk_over_sample,
    output  real            RES_TX [16:0],
    output  real            RES_RX [16:0],
    output  real            RES_RX_CLK  [1:0],
    input   logic           I_REF_BG_UP

);
endmodule
