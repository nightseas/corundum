# XDC constraints for the FPGA-EVB-Pro FPGA0 (all 3 dies)
# part: xcvu9p-flga2577-2-i

# General configuration
set_property CFGBVS GND                                [current_design]
set_property CONFIG_VOLTAGE 1.8                        [current_design]
set_property BITSTREAM.GENERAL.COMPRESS true           [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 102.0         [current_design]
set_property BITSTREAM.CONFIG.SPI_32BIT_ADDR YES       [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4           [current_design]
set_property BITSTREAM.CONFIG.SPI_FALL_EDGE YES        [current_design]

# System clocks
# 100 MHz
#set_property -dict {LOC AT17 IOSTANDARD LVCMOS18} [get_ports clk_100mhz]
#create_clock -period 10.000 -name clk_100mhz [get_ports clk_100mhz]


# Recovery clock
set_property -dict {LOC V36 IOSTANDARD LVDS} [get_ports clk_rec_pll_p]
set_property -dict {LOC V37 IOSTANDARD LVDS} [get_ports clk_rec_pll_n]
create_clock -period 40 -name clk_rec_pll [get_ports clk_rec_pll_p]


# 161.1328125 MHz
set_property -dict {LOC R32 IOSTANDARD LVDS} [get_ports clk_161mhz_p]
set_property EQUALIZATION EQ_LEVEL2 [get_ports clk_161mhz_p]
set_property DQS_BIAS FALSE [get_ports clk_161mhz_p]
set_property DIFF_TERM_ADV TERM_NONE [get_ports clk_161mhz_p]
set_property DIFF_TERM FALSE [get_ports clk_161mhz_p]

create_clock -period 6.206 -name clk_161mhz [get_ports clk_161mhz_p]

# LEDs
set_property -dict {LOC N24 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports user_led]
set_false_path -to [get_ports {user_led}]
set_output_delay 0 [get_ports {user_led}]

# PLL I/Os
set_property -dict {LOC U34 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports pll_reset_l]
set_property -dict {LOC V33 IOSTANDARD LVCMOS18 PULLUP true} [get_ports pll_lol0]
set_property -dict {LOC U32 IOSTANDARD LVCMOS18 PULLUP true} [get_ports pll_hold0]
set_property -dict {LOC V34 IOSTANDARD LVCMOS18 PULLUP true} [get_ports pll_lol1]
set_false_path -to [get_ports {pll_reset_l}]
set_output_delay 0 [get_ports {pll_reset_l}]
set_false_path -from [get_ports {pll_lol0}]
set_input_delay 0 [get_ports {pll_lol0}]
set_false_path -from [get_ports {pll_hold0}]
set_input_delay 0 [get_ports {pll_hold0}]
set_false_path -from [get_ports {pll_lol1}]
set_input_delay 0 [get_ports {pll_lol1}]

# System reset
#set_property -dict {LOC BH26  IOSTANDARD LVCMOS12} [get_ports sys_reset_n]

# I2C interface
set_property -dict {LOC N36 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports i2c_scl]
set_property -dict {LOC N37 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports i2c_sda]

set_false_path -to [get_ports {i2c_sda i2c_scl}]
set_output_delay 0 [get_ports {i2c_sda i2c_scl}]
set_false_path -from [get_ports {i2c_sda i2c_scl}]
set_input_delay 0 [get_ports {i2c_sda i2c_scl}]


set_property -dict {LOC U36 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports pll_i2c_scl]
set_property -dict {LOC V35 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports pll_i2c_sda]

set_false_path -to [get_ports {pll_i2c_sda pll_i2c_scl}]
set_output_delay 0 [get_ports {pll_i2c_sda pll_i2c_scl}]
set_false_path -from [get_ports {pll_i2c_sda pll_i2c_scl}]
set_input_delay 0 [get_ports {pll_i2c_sda pll_i2c_scl}]


############################### FPGA0 D0 ###############################

# QSFP28-F port
# Serdes ordered swapped in FPGA
#   QSFP    MGT_TRX
#   0       0
#   1       2
#   2       1
#   3       3
set_property -dict {LOC BH4  } [get_ports qsfp0_1_rx1_p] ;# MGTYRXN0_221 GTYE4_CHANNEL_X1Y8 / GTYE4_COMMON_X1Y2
#set_property -dict {LOC Y1  } [get_ports qsfp0_1_rx1_n] ;# MGTYRXP0_221 GTYE4_CHANNEL_X1Y8 / GTYE4_COMMON_X1Y2
set_property -dict {LOC BG11 } [get_ports qsfp0_1_tx1_p] ;# MGTYTXN0_221 GTYE4_CHANNEL_X1Y8 / GTYE4_COMMON_X1Y2
#set_property -dict {LOC V6  } [get_ports qsfp0_1_tx1_n] ;# MGTYTXP0_221 GTYE4_CHANNEL_X1Y8 / GTYE4_COMMON_X1Y2
set_property -dict {LOC BG6  } [get_ports qsfp0_1_rx2_p] ;# MGTYRXN1_221 GTYE4_CHANNEL_X1Y10 / GTYE4_COMMON_X1Y2
#set_property -dict {LOC W3  } [get_ports qsfp0_1_rx2_n] ;# MGTYRXP1_221 GTYE4_CHANNEL_X1Y10 / GTYE4_COMMON_X1Y2
set_property -dict {LOC BH9  } [get_ports qsfp0_1_tx2_p] ;# MGTYTXN1_221 GTYE4_CHANNEL_X1Y10 / GTYE4_COMMON_X1Y2
#set_property -dict {LOC T6  } [get_ports qsfp0_1_tx2_n] ;# MGTYTXP1_221 GTYE4_CHANNEL_X1Y10 / GTYE4_COMMON_X1Y2
set_property -dict {LOC BG2  } [get_ports qsfp0_1_rx3_p] ;# MGTYRXN2_221 GTYE4_CHANNEL_X1Y9 / GTYE4_COMMON_X1Y2
#set_property -dict {LOC V1  } [get_ports qsfp0_1_rx3_n] ;# MGTYRXP2_221 GTYE4_CHANNEL_X1Y9 / GTYE4_COMMON_X1Y2
set_property -dict {LOC BJ11 } [get_ports qsfp0_1_tx3_p] ;# MGTYTXN2_221 GTYE4_CHANNEL_X1Y9 / GTYE4_COMMON_X1Y2
#set_property -dict {LOC P6  } [get_ports qsfp0_1_tx3_n] ;# MGTYTXP2_221 GTYE4_CHANNEL_X1Y9 / GTYE4_COMMON_X1Y2
set_property -dict {LOC BF4  } [get_ports qsfp0_1_rx4_p] ;# MGTYRXN3_221 GTYE4_CHANNEL_X1Y11 / GTYE4_COMMON_X1Y2
#set_property -dict {LOC U3  } [get_ports qsfp0_1_rx4_n] ;# MGTYRXP3_221 GTYE4_CHANNEL_X1Y11 / GTYE4_COMMON_X1Y2
set_property -dict {LOC BF9  } [get_ports qsfp0_1_tx4_p] ;# MGTYTXN3_221 GTYE4_CHANNEL_X1Y11 / GTYE4_COMMON_X1Y2
#set_property -dict {LOC M6  } [get_ports qsfp0_1_tx4_n] ;# MGTYTXP3_221 GTYE4_CHANNEL_X1Y11 / GTYE4_COMMON_X1Y2
set_property -dict {LOC AY13 } [get_ports qsfp0_1_mgt_refclk_0_p] ;# MGTREFCLK0P_221 
#set_property -dict {LOC W8  } [get_ports qsfp0_1_mgt_refclk_0_n] ;# MGTREFCLK0N_221 

# 161.1328125 MHz MGT reference clock
create_clock -period 6.206 -name qsfp0_1_mgt_refclk_0 [get_ports qsfp0_1_mgt_refclk_0_p]

# Interconnection-D port
#   CH      MGT_TRX
#   0       0
#   1       1
#   2       2
#   3       3
set_property -dict {LOC BE2  } [get_ports qsfp0_2_rx1_p] ;# MGTYRXN0_222 GTYE4_CHANNEL_X1Y12 / GTYE4_COMMON_X1Y3
#set_property -dict {LOC T1  } [get_ports qsfp0_2_rx1_n] ;# MGTYRXP0_222 GTYE4_CHANNEL_X1Y12 / GTYE4_COMMON_X1Y3
set_property -dict {LOC BE7  } [get_ports qsfp0_2_tx1_p] ;# MGTYTXN0_222 GTYE4_CHANNEL_X1Y12 / GTYE4_COMMON_X1Y3
#set_property -dict {LOC L4  } [get_ports qsfp0_2_tx1_n] ;# MGTYTXP0_222 GTYE4_CHANNEL_X1Y12 / GTYE4_COMMON_X1Y3
set_property -dict {LOC BD4  } [get_ports qsfp0_2_rx2_p] ;# MGTYRXN1_222 GTYE4_CHANNEL_X1Y13 / GTYE4_COMMON_X1Y3
#set_property -dict {LOC R3  } [get_ports qsfp0_2_rx2_n] ;# MGTYRXP1_222 GTYE4_CHANNEL_X1Y13 / GTYE4_COMMON_X1Y3
set_property -dict {LOC BD9  } [get_ports qsfp0_2_tx2_p] ;# MGTYTXN1_222 GTYE4_CHANNEL_X1Y13 / GTYE4_COMMON_X1Y3
#set_property -dict {LOC K6  } [get_ports qsfp0_2_tx2_n] ;# MGTYTXP1_222 GTYE4_CHANNEL_X1Y13 / GTYE4_COMMON_X1Y3
set_property -dict {LOC BC2  } [get_ports qsfp0_2_rx3_p] ;# MGTYRXN2_222 GTYE4_CHANNEL_X1Y14 / GTYE4_COMMON_X1Y3
#set_property -dict {LOC P1  } [get_ports qsfp0_2_rx3_n] ;# MGTYRXP2_222 GTYE4_CHANNEL_X1Y14 / GTYE4_COMMON_X1Y3
set_property -dict {LOC BC7  } [get_ports qsfp0_2_tx3_p] ;# MGTYTXN2_222 GTYE4_CHANNEL_X1Y14 / GTYE4_COMMON_X1Y3
#set_property -dict {LOC J4  } [get_ports qsfp0_2_tx3_n] ;# MGTYTXP2_222 GTYE4_CHANNEL_X1Y14 / GTYE4_COMMON_X1Y3
set_property -dict {LOC BB4  } [get_ports qsfp0_2_rx4_p] ;# MGTYRXN3_222 GTYE4_CHANNEL_X1Y15 / GTYE4_COMMON_X1Y3
#set_property -dict {LOC M1  } [get_ports qsfp0_2_rx4_n] ;# MGTYRXP3_222 GTYE4_CHANNEL_X1Y15 / GTYE4_COMMON_X1Y3
set_property -dict {LOC BB9  } [get_ports qsfp0_2_tx4_p] ;# MGTYTXN3_222 GTYE4_CHANNEL_X1Y15 / GTYE4_COMMON_X1Y3
#set_property -dict {LOC H6  } [get_ports qsfp0_2_tx4_n] ;# MGTYTXP3_222 GTYE4_CHANNEL_X1Y15 / GTYE4_COMMON_X1Y3
# set_property -dict {LOC AV13  } [get_ports qsfp0_2_mgt_refclk_0_p] ;# MGTREFCLK0P_222
#set_property -dict {LOC R8  } [get_ports qsfp0_2_mgt_refclk_0_n] ;# MGTREFCLK0N_222

# # 161.1328125 MHz MGT reference clock
# create_clock -period 6.206 -name qsfp0_2_mgt_refclk_0 [get_ports qsfp0_2_mgt_refclk_0_p]

# PCIe Interface B
set_property -dict {LOC BJ46  } [get_ports {pcie0_rx_p[0]}]  ;# MGTYRXP3_120 GTYE4_CHANNEL_X0Y7 / GTYE4_COMMON_X0Y1
#set_property -dict {LOC AA3  } [get_ports {pcie0_rx_n[0]}]  ;# MGTYRXN3_120 GTYE4_CHANNEL_X0Y7 / GTYE4_COMMON_X0Y1
set_property -dict {LOC BK43  } [get_ports {pcie0_tx_p[0]}]  ;# MGTYTXP3_120 GTYE4_CHANNEL_X0Y7 / GTYE4_COMMON_X0Y1
#set_property -dict {LOC Y6   } [get_ports {pcie0_tx_n[0]}]  ;# MGTYTXN3_120 GTYE4_CHANNEL_X0Y7 / GTYE4_COMMON_X0Y1
set_property -dict {LOC BL46  } [get_ports {pcie0_rx_p[1]}]  ;# MGTYRXP2_120 GTYE4_CHANNEL_X0Y6 / GTYE4_COMMON_X0Y1
#set_property -dict {LOC AB1  } [get_ports {pcie0_rx_n[1]}]  ;# MGTYRXN2_120 GTYE4_CHANNEL_X0Y6 / GTYE4_COMMON_X0Y1
set_property -dict {LOC BL41  } [get_ports {pcie0_tx_p[1]}]  ;# MGTYTXP2_120 GTYE4_CHANNEL_X0Y6 / GTYE4_COMMON_X0Y1
#set_property -dict {LOC AB6  } [get_ports {pcie0_tx_n[1]}]  ;# MGTYTXN2_120 GTYE4_CHANNEL_X0Y6 / GTYE4_COMMON_X0Y1
set_property -dict {LOC BL32  } [get_ports {pcie0_rx_p[2]}]  ;# MGTYRXP1_120 GTYE4_CHANNEL_X0Y5 / GTYE4_COMMON_X0Y1
#set_property -dict {LOC AC3  } [get_ports {pcie0_rx_n[2]}]  ;# MGTYRXN1_120 GTYE4_CHANNEL_X0Y5 / GTYE4_COMMON_X0Y1
set_property -dict {LOC BK39  } [get_ports {pcie0_tx_p[2]}]  ;# MGTYTXP1_120 GTYE4_CHANNEL_X0Y5 / GTYE4_COMMON_X0Y1
#set_property -dict {LOC AD6  } [get_ports {pcie0_tx_n[2]}]  ;# MGTYTXN1_120 GTYE4_CHANNEL_X0Y5 / GTYE4_COMMON_X0Y1
set_property -dict {LOC BK34  } [get_ports {pcie0_rx_p[3]}]  ;# MGTYRXP0_120 GTYE4_CHANNEL_X0Y4 / GTYE4_COMMON_X0Y1
#set_property -dict {LOC AD1  } [get_ports {pcie0_rx_n[3]}]  ;# MGTYRXN0_120 GTYE4_CHANNEL_X0Y4 / GTYE4_COMMON_X0Y1
set_property -dict {LOC BL37  } [get_ports {pcie0_tx_p[3]}]  ;# MGTYTXP0_120 GTYE4_CHANNEL_X0Y4 / GTYE4_COMMON_X0Y1
#set_property -dict {LOC AF6  } [get_ports {pcie0_tx_n[3]}]  ;# MGTYTXN0_120 GTYE4_CHANNEL_X0Y4 / GTYE4_COMMON_X0Y1
set_property -dict {LOC BH34  } [get_ports {pcie0_rx_p[4]}]  ;# MGTYRXP3_119 GTYE4_CHANNEL_X0Y3 / GTYE4_COMMON_X0Y0
#set_property -dict {LOC AE3  } [get_ports {pcie0_rx_n[4]}]  ;# MGTYRXN3_119 GTYE4_CHANNEL_X0Y3 / GTYE4_COMMON_X0Y0
set_property -dict {LOC BG37  } [get_ports {pcie0_tx_p[4]}]  ;# MGTYTXP3_119 GTYE4_CHANNEL_X0Y3 / GTYE4_COMMON_X0Y0
#set_property -dict {LOC AH6  } [get_ports {pcie0_tx_n[4]}]  ;# MGTYTXN3_119 GTYE4_CHANNEL_X0Y3 / GTYE4_COMMON_X0Y0
set_property -dict {LOC BJ32  } [get_ports {pcie0_rx_p[5]}]  ;# MGTYRXP2_119 GTYE4_CHANNEL_X0Y6 / GTYE4_COMMON_X0Y0
#set_property -dict {LOC AF1  } [get_ports {pcie0_rx_n[5]}]  ;# MGTYRXN2_119 GTYE4_CHANNEL_X0Y2 / GTYE4_COMMON_X0Y0
set_property -dict {LOC BJ37  } [get_ports {pcie0_tx_p[5]}]  ;# MGTYTXP2_119 GTYE4_CHANNEL_X0Y2 / GTYE4_COMMON_X0Y0
#set_property -dict {LOC AK6  } [get_ports {pcie0_tx_n[5]}]  ;# MGTYTXN2_119 GTYE4_CHANNEL_X0Y2 / GTYE4_COMMON_X0Y0
set_property -dict {LOC BF34  } [get_ports {pcie0_rx_p[6]}]  ;# MGTYRXP1_119 GTYE4_CHANNEL_X0Y1 / GTYE4_COMMON_X0Y0
#set_property -dict {LOC AG3  } [get_ports {pcie0_rx_n[6]}]  ;# MGTYRXN1_119 GTYE4_CHANNEL_X0Y1 / GTYE4_COMMON_X0Y0
set_property -dict {LOC BF39  } [get_ports {pcie0_tx_p[6]}]  ;# MGTYTXP1_119 GTYE4_CHANNEL_X0Y1 / GTYE4_COMMON_X0Y0
#set_property -dict {LOC AM6  } [get_ports {pcie0_tx_n[6]}]  ;# MGTYTXN1_119 GTYE4_CHANNEL_X0Y1 / GTYE4_COMMON_X0Y0
set_property -dict {LOC BG32  } [get_ports {pcie0_rx_p[7]}]  ;# MGTYRXP0_119 GTYE4_CHANNEL_X0Y0 / GTYE4_COMMON_X0Y0
#set_property -dict {LOC AH1  } [get_ports {pcie0_rx_n[7]}]  ;# MGTYRXN0_119 GTYE4_CHANNEL_X0Y0 / GTYE4_COMMON_X0Y0
set_property -dict {LOC BH39  } [get_ports {pcie0_tx_p[7]}]  ;# MGTYTXP0_119 GTYE4_CHANNEL_X0Y0 / GTYE4_COMMON_X0Y0
#set_property -dict {LOC AN4  } [get_ports {pcie0_tx_n[7]}]  ;# MGTYTXN0_119 GTYE4_CHANNEL_X0Y0 / GTYE4_COMMON_X0Y0
#set_property -dict {LOC BB39  } [get_ports pcie0_refclk_0_p] ;# MGTREFCLK0P_120
#set_property -dict {LOC AC8  } [get_ports pcie0_refclk_0_n] ;# MGTREFCLK0N_120
#set_property -dict {LOC BA41  } [get_ports pcie0_refclk_1_p] ;# MGTREFCLK1P_120
#set_property -dict {LOC AL8  } [get_ports pcie0_refclk_1_n] ;# MGTREFCLK1N_120
set_property -dict {LOC AY31 IOSTANDARD LVCMOS18 PULLUP true} [get_ports pcie0_reset_n]

# Local 100MHz async PCIe refernce clock
set_property -dict {LOC BA41  } [get_ports pcie0_refclk_0_p] ;# MGTREFCLK1P_120

set_false_path -from [get_ports {pcie0_reset_n}]
set_input_delay 0 [get_ports {pcie0_reset_n}]

# 100 MHz MGT reference clock
create_clock -period 10 -name pcie0_mgt_refclk_0 [get_ports pcie0_refclk_0_p]
#create_clock -period 10 -name pcie0_mgt_refclk_1 [get_ports pcie0_refclk_1_p]


############################### FPGA0 D1 ###############################

# QSFP28-E port
# Serdes ordered swapped in FPGA
#   QSFP    MGT_TRX
#   0       0
#   1       2
#   2       1
#   3       3
set_property -dict {LOC AJ2  } [get_ports qsfp1_1_rx1_p] ;# MGTYRXN0_226 GTYE4_CHANNEL_X1Y28 / GTYE4_COMMON_X1Y7
#set_property -dict {LOC Y1  } [get_ports qsfp1_1_rx1_n] ;# MGTYRXP0_226 GTYE4_CHANNEL_X1Y28 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AJ7  } [get_ports qsfp1_1_tx1_p] ;# MGTYTXN0_226 GTYE4_CHANNEL_X1Y28 / GTYE4_COMMON_X1Y7
#set_property -dict {LOC V6  } [get_ports qsfp1_1_tx1_n] ;# MGTYTXP0_226 GTYE4_CHANNEL_X1Y28 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AG2  } [get_ports qsfp1_1_rx2_p] ;# MGTYRXN2_226 GTYE4_CHANNEL_X1Y30 / GTYE4_COMMON_X1Y7
#set_property -dict {LOC W3  } [get_ports qsfp1_1_rx2_n] ;# MGTYRXP2_226 GTYE4_CHANNEL_X1Y30 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AG7  } [get_ports qsfp1_1_tx2_p] ;# MGTYTXN2_226 GTYE4_CHANNEL_X1Y30 / GTYE4_COMMON_X1Y7
#set_property -dict {LOC T6  } [get_ports qsfp1_1_tx2_n] ;# MGTYTXP2_226 GTYE4_CHANNEL_X1Y30 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AH4  } [get_ports qsfp1_1_rx3_p] ;# MGTYRXN1_226 GTYE4_CHANNEL_X1Y29 / GTYE4_COMMON_X1Y7
#set_property -dict {LOC V1  } [get_ports qsfp1_1_rx3_n] ;# MGTYRXP1_226 GTYE4_CHANNEL_X1Y29 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AH9  } [get_ports qsfp1_1_tx3_p] ;# MGTYTXN1_226 GTYE4_CHANNEL_X1Y29 / GTYE4_COMMON_X1Y7
#set_property -dict {LOC P6  } [get_ports qsfp1_1_tx3_n] ;# MGTYTXP1_226 GTYE4_CHANNEL_X1Y29 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AF4  } [get_ports qsfp1_1_rx4_p] ;# MGTYRXN3_226 GTYE4_CHANNEL_X1Y31 / GTYE4_COMMON_X1Y7
#set_property -dict {LOC U3  } [get_ports qsfp1_1_rx4_n] ;# MGTYRXP3_226 GTYE4_CHANNEL_X1Y31 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AF9  } [get_ports qsfp1_1_tx4_p] ;# MGTYTXN3_226 GTYE4_CHANNEL_X1Y31 / GTYE4_COMMON_X1Y7
#set_property -dict {LOC M6  } [get_ports qsfp1_1_tx4_n] ;# MGTYTXP3_226 GTYE4_CHANNEL_X1Y31 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AJ11  } [get_ports qsfp1_1_mgt_refclk_0_p] ;# MGTREFCLK0P_226 
#set_property -dict {LOC W8  } [get_ports qsfp1_1_mgt_refclk_0_n] ;# MGTREFCLK0N_226 

# 161.1328125 MHz MGT reference clock
create_clock -period 6.206 -name qsfp1_1_mgt_refclk_0 [get_ports qsfp1_1_mgt_refclk_0_p]

# Interconnection-E port
#   CH      MGT_TRX
#   0       0
#   1       1
#   2       2
#   3       3
set_property -dict {LOC AE2  } [get_ports qsfp1_2_rx1_p] ;# MGTYRXN0_227 GTYE4_CHANNEL_X1Y32 / GTYE4_COMMON_X1Y8
#set_property -dict {LOC T1  } [get_ports qsfp1_2_rx1_n] ;# MGTYRXP0_227 GTYE4_CHANNEL_X1Y32 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AE7  } [get_ports qsfp1_2_tx1_p] ;# MGTYTXN0_227 GTYE4_CHANNEL_X1Y32 / GTYE4_COMMON_X1Y8
#set_property -dict {LOC L4  } [get_ports qsfp1_2_tx1_n] ;# MGTYTXP0_227 GTYE4_CHANNEL_X1Y32 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AD4  } [get_ports qsfp1_2_rx2_p] ;# MGTYRXN1_227 GTYE4_CHANNEL_X1Y33 / GTYE4_COMMON_X1Y8
#set_property -dict {LOC R3  } [get_ports qsfp1_2_rx2_n] ;# MGTYRXP1_227 GTYE4_CHANNEL_X1Y33 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AD9  } [get_ports qsfp1_2_tx2_p] ;# MGTYTXN1_227 GTYE4_CHANNEL_X1Y33 / GTYE4_COMMON_X1Y8
#set_property -dict {LOC K6  } [get_ports qsfp1_2_tx2_n] ;# MGTYTXP1_227 GTYE4_CHANNEL_X1Y33 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AC2  } [get_ports qsfp1_2_rx3_p] ;# MGTYRXN2_227 GTYE4_CHANNEL_X1Y34 / GTYE4_COMMON_X1Y8
#set_property -dict {LOC P1  } [get_ports qsfp1_2_rx3_n] ;# MGTYRXP2_227 GTYE4_CHANNEL_X1Y34 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AC7  } [get_ports qsfp1_2_tx3_p] ;# MGTYTXN2_227 GTYE4_CHANNEL_X1Y34 / GTYE4_COMMON_X1Y8
#set_property -dict {LOC J4  } [get_ports qsfp1_2_tx3_n] ;# MGTYTXP2_227 GTYE4_CHANNEL_X1Y34 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AB4  } [get_ports qsfp1_2_rx4_p] ;# MGTYRXN3_227 GTYE4_CHANNEL_X1Y35 / GTYE4_COMMON_X1Y8
#set_property -dict {LOC M1  } [get_ports qsfp1_2_rx4_n] ;# MGTYRXP3_227 GTYE4_CHANNEL_X1Y35 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AB9  } [get_ports qsfp1_2_tx4_p] ;# MGTYTXN3_227 GTYE4_CHANNEL_X1Y35 / GTYE4_COMMON_X1Y8
#set_property -dict {LOC H6  } [get_ports qsfp1_2_tx4_n] ;# MGTYTXP3_227 GTYE4_CHANNEL_X1Y35 / GTYE4_COMMON_X1Y8
# set_property -dict {LOC AE11  } [get_ports qsfp1_2_mgt_refclk_0_p] ;# MGTREFCLK0P_227
#set_property -dict {LOC R8  } [get_ports qsfp1_2_mgt_refclk_0_n] ;# MGTREFCLK0N_227

# # 161.1328125 MHz MGT reference clock
# create_clock -period 6.206 -name qsfp1_2_mgt_refclk_0 [get_ports qsfp1_2_mgt_refclk_0_p]

# PCIe Interface A
set_property -dict {LOC AK4  } [get_ports {pcie1_rx_p[0]}]  ;# MGTYRXP3_225 GTYE4_CHANNEL_X1Y27 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC AA3  } [get_ports {pcie1_rx_n[0]}]  ;# MGTYRXN3_225 GTYE4_CHANNEL_X1Y27 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AK9  } [get_ports {pcie1_tx_p[0]}]  ;# MGTYTXP3_225 GTYE4_CHANNEL_X1Y27 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC Y6   } [get_ports {pcie1_tx_n[0]}]  ;# MGTYTXN3_225 GTYE4_CHANNEL_X1Y27 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AL2  } [get_ports {pcie1_rx_p[1]}]  ;# MGTYRXP2_225 GTYE4_CHANNEL_X1Y26 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC AB1  } [get_ports {pcie1_rx_n[1]}]  ;# MGTYRXN2_225 GTYE4_CHANNEL_X1Y26 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AL7  } [get_ports {pcie1_tx_p[1]}]  ;# MGTYTXP2_225 GTYE4_CHANNEL_X1Y26 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC AB6  } [get_ports {pcie1_tx_n[1]}]  ;# MGTYTXN2_225 GTYE4_CHANNEL_X1Y26 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AM4  } [get_ports {pcie1_rx_p[2]}]  ;# MGTYRXP1_225 GTYE4_CHANNEL_X1Y25 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC AC3  } [get_ports {pcie1_rx_n[2]}]  ;# MGTYRXN1_225 GTYE4_CHANNEL_X1Y25 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AM9  } [get_ports {pcie1_tx_p[2]}]  ;# MGTYTXP1_225 GTYE4_CHANNEL_X1Y25 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC AD6  } [get_ports {pcie1_tx_n[2]}]  ;# MGTYTXN1_225 GTYE4_CHANNEL_X1Y25 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AN2  } [get_ports {pcie1_rx_p[3]}]  ;# MGTYRXP0_225 GTYE4_CHANNEL_X1Y24 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC AD1  } [get_ports {pcie1_rx_n[3]}]  ;# MGTYRXN0_225 GTYE4_CHANNEL_X1Y24 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AN7  } [get_ports {pcie1_tx_p[3]}]  ;# MGTYTXP0_225 GTYE4_CHANNEL_X1Y24 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC AF6  } [get_ports {pcie1_tx_n[3]}]  ;# MGTYTXN0_225 GTYE4_CHANNEL_X1Y24 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AP4  } [get_ports {pcie1_rx_p[4]}]  ;# MGTYRXP3_224 GTYE4_CHANNEL_X1Y23 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AE3  } [get_ports {pcie1_rx_n[4]}]  ;# MGTYRXN3_224 GTYE4_CHANNEL_X1Y23 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AP9  } [get_ports {pcie1_tx_p[4]}]  ;# MGTYTXP3_224 GTYE4_CHANNEL_X1Y23 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AH6  } [get_ports {pcie1_tx_n[4]}]  ;# MGTYTXN3_224 GTYE4_CHANNEL_X1Y23 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AR2  } [get_ports {pcie1_rx_p[5]}]  ;# MGTYRXP2_224 GTYE4_CHANNEL_X1Y22 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AF1  } [get_ports {pcie1_rx_n[5]}]  ;# MGTYRXN2_224 GTYE4_CHANNEL_X1Y22 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AR7  } [get_ports {pcie1_tx_p[5]}]  ;# MGTYTXP2_224 GTYE4_CHANNEL_X1Y22 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AK6  } [get_ports {pcie1_tx_n[5]}]  ;# MGTYTXN2_224 GTYE4_CHANNEL_X1Y22 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AT4  } [get_ports {pcie1_rx_p[6]}]  ;# MGTYRXP1_224 GTYE4_CHANNEL_X1Y21 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AG3  } [get_ports {pcie1_rx_n[6]}]  ;# MGTYRXN1_224 GTYE4_CHANNEL_X1Y21 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AT9  } [get_ports {pcie1_tx_p[6]}]  ;# MGTYTXP1_224 GTYE4_CHANNEL_X1Y21 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AM6  } [get_ports {pcie1_tx_n[6]}]  ;# MGTYTXN1_224 GTYE4_CHANNEL_X1Y21 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AU2  } [get_ports {pcie1_rx_p[7]}]  ;# MGTYRXP0_224 GTYE4_CHANNEL_X1Y20 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AH1  } [get_ports {pcie1_rx_n[7]}]  ;# MGTYRXN0_224 GTYE4_CHANNEL_X1Y20 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AU7  } [get_ports {pcie1_tx_p[7]}]  ;# MGTYTXP0_224 GTYE4_CHANNEL_X1Y20 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AN4  } [get_ports {pcie1_tx_n[7]}]  ;# MGTYTXN0_224 GTYE4_CHANNEL_X1Y20 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AM13  } [get_ports pcie1_refclk_0_p] ;# MGTREFCLK0P_225
#set_property -dict {LOC AC8  } [get_ports pcie1_refclk_0_n] ;# MGTREFCLK0N_225
#set_property -dict {LOC AL11  } [get_ports pcie1_refclk_1_p] ;# MGTREFCLK1P_225
#set_property -dict {LOC AL8  } [get_ports pcie1_refclk_1_n] ;# MGTREFCLK1N_225
set_property -dict {LOC BB15 IOSTANDARD LVCMOS18 PULLUP true} [get_ports pcie1_reset_n]

# Local 100MHz async PCIe refernce clock
set_property -dict {LOC AL11  } [get_ports pcie1_refclk_0_p] ;# MGTREFCLK1P_225

set_false_path -from [get_ports {pcie1_reset_n}]
set_input_delay 0 [get_ports {pcie1_reset_n}]

# 100 MHz MGT reference clock
create_clock -period 10 -name pcie1_mgt_refclk_0 [get_ports pcie1_refclk_0_p]
#create_clock -period 10 -name pcie1_mgt_refclk_1 [get_ports pcie1_refclk_1_p]


############################### FPGA0 D2 ###############################

# QSFP28-D port
# Serdes ordered swapped in FPGA
#   QSFP    MGT_TRX
#   0       0
#   1       2
#   2       1
#   3       3
set_property -dict {LOC N2  } [get_ports qsfp2_1_rx1_p] ;# MGTYRXN0_230 GTYE4_CHANNEL_X1Y44 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC Y1  } [get_ports qsfp2_1_rx1_n] ;# MGTYRXP0_230 GTYE4_CHANNEL_X1Y44 / GTYE4_COMMON_X1Y11
set_property -dict {LOC N7  } [get_ports qsfp2_1_tx1_p] ;# MGTYTXN0_230 GTYE4_CHANNEL_X1Y44 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC V6  } [get_ports qsfp2_1_tx1_n] ;# MGTYTXP0_230 GTYE4_CHANNEL_X1Y44 / GTYE4_COMMON_X1Y11
set_property -dict {LOC L2  } [get_ports qsfp2_1_rx2_p] ;# MGTYRXN2_230 GTYE4_CHANNEL_X1Y46 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC W3  } [get_ports qsfp2_1_rx2_n] ;# MGTYRXP2_230 GTYE4_CHANNEL_X1Y46 / GTYE4_COMMON_X1Y11
set_property -dict {LOC L7  } [get_ports qsfp2_1_tx2_p] ;# MGTYTXN2_230 GTYE4_CHANNEL_X1Y46 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC T6  } [get_ports qsfp2_1_tx2_n] ;# MGTYTXP2_230 GTYE4_CHANNEL_X1Y46 / GTYE4_COMMON_X1Y11
set_property -dict {LOC M4  } [get_ports qsfp2_1_rx3_p] ;# MGTYRXN1_230 GTYE4_CHANNEL_X1Y45 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC V1  } [get_ports qsfp2_1_rx3_n] ;# MGTYRXP1_230 GTYE4_CHANNEL_X1Y45 / GTYE4_COMMON_X1Y11
set_property -dict {LOC M9  } [get_ports qsfp2_1_tx3_p] ;# MGTYTXN1_230 GTYE4_CHANNEL_X1Y45 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC P6  } [get_ports qsfp2_1_tx3_n] ;# MGTYTXP1_230 GTYE4_CHANNEL_X1Y45 / GTYE4_COMMON_X1Y11
set_property -dict {LOC K4  } [get_ports qsfp2_1_rx4_p] ;# MGTYRXN3_230 GTYE4_CHANNEL_X1Y47 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC U3  } [get_ports qsfp2_1_rx4_n] ;# MGTYRXP3_230 GTYE4_CHANNEL_X1Y47 / GTYE4_COMMON_X1Y11
set_property -dict {LOC K9  } [get_ports qsfp2_1_tx4_p] ;# MGTYTXN3_230 GTYE4_CHANNEL_X1Y47 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC M6  } [get_ports qsfp2_1_tx4_n] ;# MGTYTXP3_230 GTYE4_CHANNEL_X1Y47 / GTYE4_COMMON_X1Y11
set_property -dict {LOC U11  } [get_ports qsfp2_1_mgt_refclk_0_p] ;# MGTREFCLK0P_230 
#set_property -dict {LOC W8  } [get_ports qsfp2_1_mgt_refclk_0_n] ;# MGTREFCLK0N_230 

# 161.1328125 MHz MGT reference clock
create_clock -period 6.206 -name qsfp2_1_mgt_refclk_0 [get_ports qsfp2_1_mgt_refclk_0_p]

# Interconnection-F port
# Serdes ordered swapped in FPGA
# Only I-F CH0 & CH1 are available.
#   CH      MGT_TRX
#   0       2
#   1       3
#   NA      0
#   NA      1
set_property -dict {LOC G2  } [get_ports qsfp2_2_rx1_p] ;# MGTYRXN2_231 GTYE4_CHANNEL_X1Y50 / GTYE4_COMMON_X1Y12
#set_property -dict {LOC T1  } [get_ports qsfp2_2_rx1_n] ;# MGTYRXP2_231 GTYE4_CHANNEL_X1Y50 / GTYE4_COMMON_X1Y12
set_property -dict {LOC G7  } [get_ports qsfp2_2_tx1_p] ;# MGTYTXN2_231 GTYE4_CHANNEL_X1Y50 / GTYE4_COMMON_X1Y12
#set_property -dict {LOC L4  } [get_ports qsfp2_2_tx1_n] ;# MGTYTXP2_231 GTYE4_CHANNEL_X1Y50 / GTYE4_COMMON_X1Y12
set_property -dict {LOC F4  } [get_ports qsfp2_2_rx2_p] ;# MGTYRXN3_231 GTYE4_CHANNEL_X1Y51 / GTYE4_COMMON_X1Y12
#set_property -dict {LOC R3  } [get_ports qsfp2_2_rx2_n] ;# MGTYRXP3_231 GTYE4_CHANNEL_X1Y51 / GTYE4_COMMON_X1Y12
set_property -dict {LOC F9  } [get_ports qsfp2_2_tx2_p] ;# MGTYTXN3_231 GTYE4_CHANNEL_X1Y51 / GTYE4_COMMON_X1Y12
#set_property -dict {LOC K6  } [get_ports qsfp2_2_tx2_n] ;# MGTYTXP3_231 GTYE4_CHANNEL_X1Y51 / GTYE4_COMMON_X1Y12
set_property -dict {LOC J2  } [get_ports qsfp2_2_rx3_p] ;# MGTYRXN0_231 GTYE4_CHANNEL_X1Y48 / GTYE4_COMMON_X1Y12
#set_property -dict {LOC P1  } [get_ports qsfp2_2_rx3_n] ;# MGTYRXP0_231 GTYE4_CHANNEL_X1Y48 / GTYE4_COMMON_X1Y12
set_property -dict {LOC J7  } [get_ports qsfp2_2_tx3_p] ;# MGTYTXN0_231 GTYE4_CHANNEL_X1Y48 / GTYE4_COMMON_X1Y12
#set_property -dict {LOC J4  } [get_ports qsfp2_2_tx3_n] ;# MGTYTXP0_231 GTYE4_CHANNEL_X1Y48 / GTYE4_COMMON_X1Y12
set_property -dict {LOC H4  } [get_ports qsfp2_2_rx4_p] ;# MGTYRXN1_231 GTYE4_CHANNEL_X1Y49 / GTYE4_COMMON_X1Y12
#set_property -dict {LOC M1  } [get_ports qsfp2_2_rx4_n] ;# MGTYRXP1_231 GTYE4_CHANNEL_X1Y49 / GTYE4_COMMON_X1Y12
set_property -dict {LOC H9  } [get_ports qsfp2_2_tx4_p] ;# MGTYTXN1_231 GTYE4_CHANNEL_X1Y49 / GTYE4_COMMON_X1Y12
#set_property -dict {LOC H6  } [get_ports qsfp2_2_tx4_n] ;# MGTYTXP1_231 GTYE4_CHANNEL_X1Y49 / GTYE4_COMMON_X1Y12
# set_property -dict {LOC R11  } [get_ports qsfp2_2_mgt_refclk_0_p] ;# MGTREFCLK0P_231
#set_property -dict {LOC R8  } [get_ports qsfp2_2_mgt_refclk_0_n] ;# MGTREFCLK0N_231

# # 161.1328125 MHz MGT reference clock
# create_clock -period 6.206 -name qsfp2_2_mgt_refclk_0 [get_ports qsfp2_2_mgt_refclk_0_p]

# PCIe Interface C
# Serdes [7:0] swapped to [0:7]
set_property -dict {LOC J50  } [get_ports {pcie2_rx_p[0]}]  ;# MGTYRXP3_131 GTYE4_CHANNEL_X0Y48 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC AA3  } [get_ports {pcie2_rx_n[0]}]  ;# MGTYRXN3_131 GTYE4_CHANNEL_X0Y48 / GTYE4_COMMON_X0Y12
set_property -dict {LOC J45  } [get_ports {pcie2_tx_p[0]}]  ;# MGTYTXP3_131 GTYE4_CHANNEL_X0Y48 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC Y6   } [get_ports {pcie2_tx_n[0]}]  ;# MGTYTXN3_131 GTYE4_CHANNEL_X0Y48 / GTYE4_COMMON_X0Y12
set_property -dict {LOC H48  } [get_ports {pcie2_rx_p[1]}]  ;# MGTYRXP2_131 GTYE4_CHANNEL_X0Y49 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC AB1  } [get_ports {pcie2_rx_n[1]}]  ;# MGTYRXN2_131 GTYE4_CHANNEL_X0Y49 / GTYE4_COMMON_X0Y12
set_property -dict {LOC H43  } [get_ports {pcie2_tx_p[1]}]  ;# MGTYTXP2_131 GTYE4_CHANNEL_X0Y49 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC AB6  } [get_ports {pcie2_tx_n[1]}]  ;# MGTYTXN2_131 GTYE4_CHANNEL_X0Y49 / GTYE4_COMMON_X0Y12
set_property -dict {LOC G50  } [get_ports {pcie2_rx_p[2]}]  ;# MGTYRXP1_131 GTYE4_CHANNEL_X0Y50 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC AC3  } [get_ports {pcie2_rx_n[2]}]  ;# MGTYRXN1_131 GTYE4_CHANNEL_X0Y50 / GTYE4_COMMON_X0Y12
set_property -dict {LOC G45  } [get_ports {pcie2_tx_p[2]}]  ;# MGTYTXP1_131 GTYE4_CHANNEL_X0Y50 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC AD6  } [get_ports {pcie2_tx_n[2]}]  ;# MGTYTXN1_131 GTYE4_CHANNEL_X0Y50 / GTYE4_COMMON_X0Y12
set_property -dict {LOC F48  } [get_ports {pcie2_rx_p[3]}]  ;# MGTYRXP0_131 GTYE4_CHANNEL_X0Y51 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC AD1  } [get_ports {pcie2_rx_n[3]}]  ;# MGTYRXN0_131 GTYE4_CHANNEL_X0Y51 / GTYE4_COMMON_X0Y12
set_property -dict {LOC F43  } [get_ports {pcie2_tx_p[3]}]  ;# MGTYTXP0_131 GTYE4_CHANNEL_X0Y51 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC AF6  } [get_ports {pcie2_tx_n[3]}]  ;# MGTYTXN0_131 GTYE4_CHANNEL_X0Y51 / GTYE4_COMMON_X0Y12
set_property -dict {LOC E50  } [get_ports {pcie2_rx_p[4]}]  ;# MGTYRXP3_132 GTYE4_CHANNEL_X0Y52 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AE3  } [get_ports {pcie2_rx_n[4]}]  ;# MGTYRXN3_132 GTYE4_CHANNEL_X0Y52 / GTYE4_COMMON_X0Y13
set_property -dict {LOC D43  } [get_ports {pcie2_tx_p[4]}]  ;# MGTYTXP3_132 GTYE4_CHANNEL_X0Y52 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AH6  } [get_ports {pcie2_tx_n[4]}]  ;# MGTYTXN3_132 GTYE4_CHANNEL_X0Y52 / GTYE4_COMMON_X0Y13
set_property -dict {LOC D48  } [get_ports {pcie2_rx_p[5]}]  ;# MGTYRXP2_132 GTYE4_CHANNEL_X0Y53 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AF1  } [get_ports {pcie2_rx_n[5]}]  ;# MGTYRXN2_132 GTYE4_CHANNEL_X0Y53 / GTYE4_COMMON_X0Y13
set_property -dict {LOC B43  } [get_ports {pcie2_tx_p[5]}]  ;# MGTYTXP2_132 GTYE4_CHANNEL_X0Y53 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AK6  } [get_ports {pcie2_tx_n[5]}]  ;# MGTYTXN2_132 GTYE4_CHANNEL_X0Y53 / GTYE4_COMMON_X0Y13
set_property -dict {LOC E46  } [get_ports {pcie2_rx_p[6]}]  ;# MGTYRXP1_132 GTYE4_CHANNEL_X0Y54 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AG3  } [get_ports {pcie2_rx_n[6]}]  ;# MGTYRXN1_132 GTYE4_CHANNEL_X0Y54 / GTYE4_COMMON_X0Y13
set_property -dict {LOC C41  } [get_ports {pcie2_tx_p[6]}]  ;# MGTYTXP1_132 GTYE4_CHANNEL_X0Y54 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AM6  } [get_ports {pcie2_tx_n[6]}]  ;# MGTYTXN1_132 GTYE4_CHANNEL_X0Y54 / GTYE4_COMMON_X0Y13
set_property -dict {LOC C46  } [get_ports {pcie2_rx_p[7]}]  ;# MGTYRXP0_132 GTYE4_CHANNEL_X0Y55 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AH1  } [get_ports {pcie2_rx_n[7]}]  ;# MGTYRXN0_132 GTYE4_CHANNEL_X0Y55 / GTYE4_COMMON_X0Y13
set_property -dict {LOC E41  } [get_ports {pcie2_tx_p[7]}]  ;# MGTYTXP0_132 GTYE4_CHANNEL_X0Y55 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AN4  } [get_ports {pcie2_tx_n[7]}]  ;# MGTYTXN0_132 GTYE4_CHANNEL_X0Y55 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC N41  } [get_ports pcie2_refclk_0_p] ;# MGTREFCLK0P_132
#set_property -dict {LOC AC8  } [get_ports pcie2_refclk_0_n] ;# MGTREFCLK0N_132
#set_property -dict {LOC M39  } [get_ports pcie2_refclk_1_p] ;# MGTREFCLK1P_132
#set_property -dict {LOC AL8  } [get_ports pcie2_refclk_1_n] ;# MGTREFCLK1N_132
set_property -dict {LOC P23 IOSTANDARD LVCMOS18 PULLUP true} [get_ports pcie2_reset_n]

# Local 100MHz async PCIe refernce clock
set_property -dict {LOC M39  } [get_ports pcie2_refclk_0_p] ;# MGTREFCLK1P_132

set_false_path -from [get_ports {pcie2_reset_n}]
set_input_delay 0 [get_ports {pcie2_reset_n}]

# 100 MHz MGT reference clock
create_clock -period 10 -name pcie2_mgt_refclk_0 [get_ports pcie2_refclk_0_p]
#create_clock -period 10 -name pcie2_mgt_refclk_1 [get_ports pcie2_refclk_1_p]

