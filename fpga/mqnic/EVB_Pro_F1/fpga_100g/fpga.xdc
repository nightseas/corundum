# XDC constraints for the FPGA-EVB-Pro FPGA1 (all 3 dies)
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

# System reset
#set_property -dict {LOC BH26  IOSTANDARD LVCMOS12} [get_ports sys_reset_n]

# I2C interface
# set_property -dict {LOC N36 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports i2c_scl]
# set_property -dict {LOC N37 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports i2c_sda]


############################### FPGA1 D0 ###############################

# QSFP28-A port
# Serdes ordered not swapped in FPGA, serdes swap handled by CMAC
#   QSFP    MGT_TRX
#   0       0
#   1       2
#   2       1
#   3       3
set_property -dict {LOC BA50  } [get_ports qsfp0_1_rx1_p] ;# MGTYRXN0_131 GTYE4_CHANNEL_X0Y16 / GTYE4_COMMON_X0Y4
#set_property -dict {LOC Y1  } [get_ports qsfp0_1_rx1_n] ;# MGTYRXP0_131 GTYE4_CHANNEL_X0Y16 / GTYE4_COMMON_X0Y4
set_property -dict {LOC BA45  } [get_ports qsfp0_1_tx1_p] ;# MGTYTXN0_131 GTYE4_CHANNEL_X0Y16 / GTYE4_COMMON_X0Y4
#set_property -dict {LOC V6  } [get_ports qsfp0_1_tx1_n] ;# MGTYTXP0_131 GTYE4_CHANNEL_X0Y16 / GTYE4_COMMON_X0Y4
set_property -dict {LOC AY48  } [get_ports qsfp0_1_rx2_p] ;# MGTYRXN2_131 GTYE4_CHANNEL_X0Y18 / GTYE4_COMMON_X0Y4
#set_property -dict {LOC W3  } [get_ports qsfp0_1_rx2_n] ;# MGTYRXP2_131 GTYE4_CHANNEL_X0Y18 / GTYE4_COMMON_X0Y4
set_property -dict {LOC AY43  } [get_ports qsfp0_1_tx2_p] ;# MGTYTXN2_131 GTYE4_CHANNEL_X0Y18 / GTYE4_COMMON_X0Y4
#set_property -dict {LOC T6  } [get_ports qsfp0_1_tx2_n] ;# MGTYTXP2_131 GTYE4_CHANNEL_X0Y18 / GTYE4_COMMON_X0Y4
set_property -dict {LOC AW50  } [get_ports qsfp0_1_rx3_p] ;# MGTYRXN1_131 GTYE4_CHANNEL_X0Y17 / GTYE4_COMMON_X0Y4
#set_property -dict {LOC V1  } [get_ports qsfp0_1_rx3_n] ;# MGTYRXP1_131 GTYE4_CHANNEL_X0Y17 / GTYE4_COMMON_X0Y4
set_property -dict {LOC AW45  } [get_ports qsfp0_1_tx3_p] ;# MGTYTXN1_131 GTYE4_CHANNEL_X0Y17 / GTYE4_COMMON_X0Y4
#set_property -dict {LOC P6  } [get_ports qsfp0_1_tx3_n] ;# MGTYTXP1_131 GTYE4_CHANNEL_X0Y17 / GTYE4_COMMON_X0Y4
set_property -dict {LOC AV48  } [get_ports qsfp0_1_rx4_p] ;# MGTYRXN3_131 GTYE4_CHANNEL_X0Y19 / GTYE4_COMMON_X0Y4
#set_property -dict {LOC U3  } [get_ports qsfp0_1_rx4_n] ;# MGTYRXP3_131 GTYE4_CHANNEL_X0Y19 / GTYE4_COMMON_X0Y4
set_property -dict {LOC AV43  } [get_ports qsfp0_1_tx4_p] ;# MGTYTXN3_131 GTYE4_CHANNEL_X0Y19 / GTYE4_COMMON_X0Y4
#set_property -dict {LOC M6  } [get_ports qsfp0_1_tx4_n] ;# MGTYTXP3_131 GTYE4_CHANNEL_X0Y19 / GTYE4_COMMON_X0Y4
set_property -dict {LOC AT39  } [get_ports qsfp0_1_mgt_refclk_0_p] ;# MGTREFCLK0P_131 
#set_property -dict {LOC W8  } [get_ports qsfp0_1_mgt_refclk_0_n] ;# MGTREFCLK0N_131 

# 161.1328125 MHz MGT reference clock
# create_clock -period 6.206 -name qsfp0_1_mgt_refclk_0 [get_ports qsfp0_1_mgt_refclk_0_p]

# Interconnection-D port
#   CH      MGT_TRX
#   0       0
#   1       1
#   2       2
#   3       3
set_property -dict {LOC BH48  } [get_ports qsfp0_2_rx1_p] ;# MGTYRXN0_121 GTYE4_CHANNEL_X0Y8 / GTYE4_COMMON_X0Y2
#set_property -dict {LOC T1  } [get_ports qsfp0_2_rx1_n] ;# MGTYRXP0_121 GTYE4_CHANNEL_X0Y8 / GTYE4_COMMON_X0Y2
set_property -dict {LOC BG41  } [get_ports qsfp0_2_tx1_p] ;# MGTYTXN0_121 GTYE4_CHANNEL_X0Y8 / GTYE4_COMMON_X0Y2
#set_property -dict {LOC L4  } [get_ports qsfp0_2_tx1_n] ;# MGTYTXP0_121 GTYE4_CHANNEL_X0Y8 / GTYE4_COMMON_X0Y2
set_property -dict {LOC BG50  } [get_ports qsfp0_2_rx2_p] ;# MGTYRXN1_121 GTYE4_CHANNEL_X0Y9 / GTYE4_COMMON_X0Y2
#set_property -dict {LOC R3  } [get_ports qsfp0_2_rx2_n] ;# MGTYRXP1_121 GTYE4_CHANNEL_X0Y9 / GTYE4_COMMON_X0Y2
set_property -dict {LOC BJ41  } [get_ports qsfp0_2_tx2_p] ;# MGTYTXN1_121 GTYE4_CHANNEL_X0Y9 / GTYE4_COMMON_X0Y2
#set_property -dict {LOC K6  } [get_ports qsfp0_2_tx2_n] ;# MGTYTXP1_121 GTYE4_CHANNEL_X0Y9 / GTYE4_COMMON_X0Y2
set_property -dict {LOC BG46  } [get_ports qsfp0_2_rx3_p] ;# MGTYRXN2_121 GTYE4_CHANNEL_X0Y10 / GTYE4_COMMON_X0Y2
#set_property -dict {LOC P1  } [get_ports qsfp0_2_rx3_n] ;# MGTYRXP2_121 GTYE4_CHANNEL_X0Y10 / GTYE4_COMMON_X0Y2
set_property -dict {LOC BH43  } [get_ports qsfp0_2_tx3_p] ;# MGTYTXN2_121 GTYE4_CHANNEL_X0Y10 / GTYE4_COMMON_X0Y2
#set_property -dict {LOC J4  } [get_ports qsfp0_2_tx3_n] ;# MGTYTXP2_121 GTYE4_CHANNEL_X0Y10 / GTYE4_COMMON_X0Y2
set_property -dict {LOC BF48  } [get_ports qsfp0_2_rx4_p] ;# MGTYRXN3_121 GTYE4_CHANNEL_X0Y11 / GTYE4_COMMON_X0Y2
#set_property -dict {LOC M1  } [get_ports qsfp0_2_rx4_n] ;# MGTYRXP3_121 GTYE4_CHANNEL_X0Y11 / GTYE4_COMMON_X0Y2
set_property -dict {LOC BF43  } [get_ports qsfp0_2_tx4_p] ;# MGTYTXN3_121 GTYE4_CHANNEL_X0Y11 / GTYE4_COMMON_X0Y2
#set_property -dict {LOC H6  } [get_ports qsfp0_2_tx4_n] ;# MGTYTXP3_121 GTYE4_CHANNEL_X0Y11 / GTYE4_COMMON_X0Y2
set_property -dict {LOC AV39  } [get_ports qsfp0_2_mgt_refclk_0_p] ;# MGTREFCLK0P_122
#set_property -dict {LOC R8  } [get_ports qsfp0_2_mgt_refclk_0_n] ;# MGTREFCLK0N_122

# 161.1328125 MHz MGT reference clock
# create_clock -period 6.206 -name qsfp0_2_mgt_refclk_0 [get_ports qsfp0_2_mgt_refclk_0_p]

# PCIe Interface E
set_property -dict {LOC BJ6   } [get_ports {pcie0_rx_p[0]}]  ;# MGTYRXP3_120 GTYE4_CHANNEL_X1Y7 / GTYE4_COMMON_X1Y1
#set_property -dict {LOC AA3  } [get_ports {pcie0_rx_n[0]}]  ;# MGTYRXN3_120 GTYE4_CHANNEL_X1Y7 / GTYE4_COMMON_X1Y1
set_property -dict {LOC BK9   } [get_ports {pcie0_tx_p[0]}]  ;# MGTYTXP3_120 GTYE4_CHANNEL_X1Y7 / GTYE4_COMMON_X1Y1
#set_property -dict {LOC Y6   } [get_ports {pcie0_tx_n[0]}]  ;# MGTYTXN3_120 GTYE4_CHANNEL_X1Y7 / GTYE4_COMMON_X1Y1
set_property -dict {LOC BL6   } [get_ports {pcie0_rx_p[1]}]  ;# MGTYRXP2_120 GTYE4_CHANNEL_X1Y6 / GTYE4_COMMON_X1Y1
#set_property -dict {LOC AB1  } [get_ports {pcie0_rx_n[1]}]  ;# MGTYRXN2_120 GTYE4_CHANNEL_X1Y6 / GTYE4_COMMON_X1Y1
set_property -dict {LOC BL11  } [get_ports {pcie0_tx_p[1]}]  ;# MGTYTXP2_120 GTYE4_CHANNEL_X1Y6 / GTYE4_COMMON_X1Y1
#set_property -dict {LOC AB6  } [get_ports {pcie0_tx_n[1]}]  ;# MGTYTXN2_120 GTYE4_CHANNEL_X1Y6 / GTYE4_COMMON_X1Y1
set_property -dict {LOC BL20  } [get_ports {pcie0_rx_p[2]}]  ;# MGTYRXP1_120 GTYE4_CHANNEL_X1Y5 / GTYE4_COMMON_X1Y1
#set_property -dict {LOC AC3  } [get_ports {pcie0_rx_n[2]}]  ;# MGTYRXN1_120 GTYE4_CHANNEL_X1Y5 / GTYE4_COMMON_X1Y1
set_property -dict {LOC BK13  } [get_ports {pcie0_tx_p[2]}]  ;# MGTYTXP1_120 GTYE4_CHANNEL_X1Y5 / GTYE4_COMMON_X1Y1
#set_property -dict {LOC AD6  } [get_ports {pcie0_tx_n[2]}]  ;# MGTYTXN1_120 GTYE4_CHANNEL_X1Y5 / GTYE4_COMMON_X1Y1
set_property -dict {LOC BK18  } [get_ports {pcie0_rx_p[3]}]  ;# MGTYRXP0_120 GTYE4_CHANNEL_X1Y4 / GTYE4_COMMON_X1Y1
#set_property -dict {LOC AD1  } [get_ports {pcie0_rx_n[3]}]  ;# MGTYRXN0_120 GTYE4_CHANNEL_X1Y4 / GTYE4_COMMON_X1Y1
set_property -dict {LOC BL15  } [get_ports {pcie0_tx_p[3]}]  ;# MGTYTXP0_120 GTYE4_CHANNEL_X1Y4 / GTYE4_COMMON_X1Y1
#set_property -dict {LOC AF6  } [get_ports {pcie0_tx_n[3]}]  ;# MGTYTXN0_120 GTYE4_CHANNEL_X1Y4 / GTYE4_COMMON_X1Y1
set_property -dict {LOC BH18  } [get_ports {pcie0_rx_p[4]}]  ;# MGTYRXP3_119 GTYE4_CHANNEL_X1Y3 / GTYE4_COMMON_X1Y0
#set_property -dict {LOC AE3  } [get_ports {pcie0_rx_n[4]}]  ;# MGTYRXN3_119 GTYE4_CHANNEL_X1Y3 / GTYE4_COMMON_X1Y0
set_property -dict {LOC BG15  } [get_ports {pcie0_tx_p[4]}]  ;# MGTYTXP3_119 GTYE4_CHANNEL_X1Y3 / GTYE4_COMMON_X1Y0
#set_property -dict {LOC AH6  } [get_ports {pcie0_tx_n[4]}]  ;# MGTYTXN3_119 GTYE4_CHANNEL_X1Y3 / GTYE4_COMMON_X1Y0
set_property -dict {LOC BJ20  } [get_ports {pcie0_rx_p[5]}]  ;# MGTYRXP2_119 GTYE4_CHANNEL_X1Y6 / GTYE4_COMMON_X1Y0
#set_property -dict {LOC AF1  } [get_ports {pcie0_rx_n[5]}]  ;# MGTYRXN2_119 GTYE4_CHANNEL_X1Y2 / GTYE4_COMMON_X1Y0
set_property -dict {LOC BJ15  } [get_ports {pcie0_tx_p[5]}]  ;# MGTYTXP2_119 GTYE4_CHANNEL_X1Y2 / GTYE4_COMMON_X1Y0
#set_property -dict {LOC AK6  } [get_ports {pcie0_tx_n[5]}]  ;# MGTYTXN2_119 GTYE4_CHANNEL_X1Y2 / GTYE4_COMMON_X1Y0
set_property -dict {LOC BF18  } [get_ports {pcie0_rx_p[6]}]  ;# MGTYRXP1_119 GTYE4_CHANNEL_X1Y1 / GTYE4_COMMON_X1Y0
#set_property -dict {LOC AG3  } [get_ports {pcie0_rx_n[6]}]  ;# MGTYRXN1_119 GTYE4_CHANNEL_X1Y1 / GTYE4_COMMON_X1Y0
set_property -dict {LOC BF13  } [get_ports {pcie0_tx_p[6]}]  ;# MGTYTXP1_119 GTYE4_CHANNEL_X1Y1 / GTYE4_COMMON_X1Y0
#set_property -dict {LOC AM6  } [get_ports {pcie0_tx_n[6]}]  ;# MGTYTXN1_119 GTYE4_CHANNEL_X1Y1 / GTYE4_COMMON_X1Y0
set_property -dict {LOC BG20  } [get_ports {pcie0_rx_p[7]}]  ;# MGTYRXP0_119 GTYE4_CHANNEL_X1Y0 / GTYE4_COMMON_X1Y0
#set_property -dict {LOC AH1  } [get_ports {pcie0_rx_n[7]}]  ;# MGTYRXN0_119 GTYE4_CHANNEL_X1Y0 / GTYE4_COMMON_X1Y0
set_property -dict {LOC BH13  } [get_ports {pcie0_tx_p[7]}]  ;# MGTYTXP0_119 GTYE4_CHANNEL_X1Y0 / GTYE4_COMMON_X1Y0
#set_property -dict {LOC AN4  } [get_ports {pcie0_tx_n[7]}]  ;# MGTYTXN0_119 GTYE4_CHANNEL_X1Y0 / GTYE4_COMMON_X1Y0
set_property -dict {LOC BB13  } [get_ports pcie0_refclk_0_p] ;# MGTREFCLK0P_120
#set_property -dict {LOC AC8  } [get_ports pcie0_refclk_0_n] ;# MGTREFCLK0N_120
#set_property -dict {LOC BA11  } [get_ports pcie0_refclk_1_p] ;# MGTREFCLK1P_120
#set_property -dict {LOC AL8  } [get_ports pcie0_refclk_1_n] ;# MGTREFCLK1N_120
set_property -dict {LOC AY31 IOSTANDARD LVCMOS18 PULLUP true} [get_ports pcie0_reset_n]

set_false_path -from [get_ports {pcie0_reset_n}]
set_input_delay 0 [get_ports {pcie0_reset_n}]

# 100 MHz MGT reference clock
create_clock -period 10 -name pcie0_mgt_refclk_0 [get_ports pcie0_refclk_0_p]
#create_clock -period 10 -name pcie0_mgt_refclk_1 [get_ports pcie0_refclk_1_p]


############################### FPGA1 D1 ###############################

# QSFP28-B port
# Serdes ordered not swapped in FPGA, serdes swap handled by CMAC
#   QSFP    MGT_TRX
#   0       0
#   1       2
#   2       1
#   3       3
set_property -dict {LOC AJ50  } [get_ports qsfp1_1_rx1_p] ;# MGTYRXN0_126 GTYE4_CHANNEL_X0Y28 / GTYE4_COMMON_X0Y?
#set_property -dict {LOC Y1  } [get_ports qsfp1_1_rx1_n] ;# MGTYRXP0_126 GTYE4_CHANNEL_X0Y28 / GTYE4_COMMON_X0Y?
set_property -dict {LOC AJ45  } [get_ports qsfp1_1_tx1_p] ;# MGTYTXN0_126 GTYE4_CHANNEL_X0Y28 / GTYE4_COMMON_X0Y?
#set_property -dict {LOC V6  } [get_ports qsfp1_1_tx1_n] ;# MGTYTXP0_126 GTYE4_CHANNEL_X0Y28 / GTYE4_COMMON_X0Y?
set_property -dict {LOC AH48  } [get_ports qsfp1_1_rx2_p] ;# MGTYRXN1_126 GTYE4_CHANNEL_X0Y29 / GTYE4_COMMON_X0Y?
#set_property -dict {LOC W3  } [get_ports qsfp1_1_rx2_n] ;# MGTYRXP1_126 GTYE4_CHANNEL_X0Y29 / GTYE4_COMMON_X0Y?
set_property -dict {LOC AH43  } [get_ports qsfp1_1_tx2_p] ;# MGTYTXN1_126 GTYE4_CHANNEL_X0Y29 / GTYE4_COMMON_X0Y?
#set_property -dict {LOC T6  } [get_ports qsfp1_1_tx2_n] ;# MGTYTXP1_126 GTYE4_CHANNEL_X0Y29 / GTYE4_COMMON_X0Y?
set_property -dict {LOC AG50  } [get_ports qsfp1_1_rx3_p] ;# MGTYRXN2_126 GTYE4_CHANNEL_X0Y30 / GTYE4_COMMON_X0Y?
#set_property -dict {LOC V1  } [get_ports qsfp1_1_rx3_n] ;# MGTYRXP2_126 GTYE4_CHANNEL_X0Y30 / GTYE4_COMMON_X0Y?
set_property -dict {LOC AG45  } [get_ports qsfp1_1_tx3_p] ;# MGTYTXN2_126 GTYE4_CHANNEL_X0Y30 / GTYE4_COMMON_X0Y?
#set_property -dict {LOC P6  } [get_ports qsfp1_1_tx3_n] ;# MGTYTXP2_126 GTYE4_CHANNEL_X0Y30 / GTYE4_COMMON_X0Y?
set_property -dict {LOC AF48  } [get_ports qsfp1_1_rx4_p] ;# MGTYRXN3_126 GTYE4_CHANNEL_X0Y31 / GTYE4_COMMON_X0Y?
#set_property -dict {LOC U3  } [get_ports qsfp1_1_rx4_n] ;# MGTYRXP3_126 GTYE4_CHANNEL_X0Y31 / GTYE4_COMMON_X0Y?
set_property -dict {LOC AF43  } [get_ports qsfp1_1_tx4_p] ;# MGTYTXN3_126 GTYE4_CHANNEL_X0Y31 / GTYE4_COMMON_X0Y?
#set_property -dict {LOC M6  } [get_ports qsfp1_1_tx4_n] ;# MGTYTXP3_126 GTYE4_CHANNEL_X0Y31 / GTYE4_COMMON_X0Y?
set_property -dict {LOC AJ41  } [get_ports qsfp1_1_mgt_refclk_0_p] ;# MGTREFCLK0P_126 
#set_property -dict {LOC W8  } [get_ports qsfp1_1_mgt_refclk_0_n] ;# MGTREFCLK0N_126 

# 161.1328125 MHz MGT reference clock
# create_clock -period 6.206 -name qsfp1_1_mgt_refclk_0 [get_ports qsfp1_1_mgt_refclk_0_p]

# Interconnection-E port
# Serdes order not swapped
# Only I-E CH0 & CH1 are available.
#   CH      MGT_TRX
#   0       2
#   1       3
#   NA      0
#   NA      1
set_property -dict {LOC AU50  } [get_ports qsfp1_2_rx1_p] ;# MGTYRXN0_125 GTYE4_CHANNEL_X0Y24 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC T1  } [get_ports qsfp1_2_rx1_n] ;# MGTYRXP0_125 GTYE4_CHANNEL_X0Y24 / GTYE4_COMMON_X0Y??
set_property -dict {LOC AU45  } [get_ports qsfp1_2_tx1_p] ;# MGTYTXN0_125 GTYE4_CHANNEL_X0Y24 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC L4  } [get_ports qsfp1_2_tx1_n] ;# MGTYTXP0_125 GTYE4_CHANNEL_X0Y24 / GTYE4_COMMON_X0Y??
set_property -dict {LOC AT48  } [get_ports qsfp1_2_rx2_p] ;# MGTYRXN1_125 GTYE4_CHANNEL_X0Y25 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC R3  } [get_ports qsfp1_2_rx2_n] ;# MGTYRXP1_125 GTYE4_CHANNEL_X0Y25 / GTYE4_COMMON_X0Y??
set_property -dict {LOC AT43  } [get_ports qsfp1_2_tx2_p] ;# MGTYTXN1_125 GTYE4_CHANNEL_X0Y25 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC K6  } [get_ports qsfp1_2_tx2_n] ;# MGTYTXP1_125 GTYE4_CHANNEL_X0Y25 / GTYE4_COMMON_X0Y??
set_property -dict {LOC AR50  } [get_ports qsfp1_2_rx3_p] ;# MGTYRXN2_125 GTYE4_CHANNEL_X0Y26 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC P1  } [get_ports qsfp1_2_rx3_n] ;# MGTYRXP2_125 GTYE4_CHANNEL_X0Y26 / GTYE4_COMMON_X0Y??
set_property -dict {LOC AR45  } [get_ports qsfp1_2_tx3_p] ;# MGTYTXN2_125 GTYE4_CHANNEL_X0Y26 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC J4  } [get_ports qsfp1_2_tx3_n] ;# MGTYTXP2_125 GTYE4_CHANNEL_X0Y26 / GTYE4_COMMON_X0Y??
set_property -dict {LOC AP48  } [get_ports qsfp1_2_rx4_p] ;# MGTYRXN3_125 GTYE4_CHANNEL_X0Y27 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC M1  } [get_ports qsfp1_2_rx4_n] ;# MGTYRXP3_125 GTYE4_CHANNEL_X0Y27 / GTYE4_COMMON_X0Y??
set_property -dict {LOC AP43  } [get_ports qsfp1_2_tx4_p] ;# MGTYTXN3_125 GTYE4_CHANNEL_X0Y27 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC H6  } [get_ports qsfp1_2_tx4_n] ;# MGTYTXP3_125 GTYE4_CHANNEL_X0Y27 / GTYE4_COMMON_X0Y??
set_property -dict {LOC AM39  } [get_ports qsfp1_2_mgt_refclk_0_p] ;# MGTREFCLK0P_125
#set_property -dict {LOC R8  } [get_ports qsfp1_2_mgt_refclk_0_n] ;# MGTREFCLK0N_125

# 161.1328125 MHz MGT reference clock
# create_clock -period 6.206 -name qsfp1_2_mgt_refclk_0 [get_ports qsfp1_2_mgt_refclk_0_p]

# PCIe Interface D
set_property -dict {LOC AK4  } [get_ports {pcie1_rx_p[0]}]  ;# MGTYRXP3_225 GTYE4_CHANNEL_X1Y23 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC AA3  } [get_ports {pcie1_rx_n[0]}]  ;# MGTYRXN3_225 GTYE4_CHANNEL_X1Y23 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AK9  } [get_ports {pcie1_tx_p[0]}]  ;# MGTYTXP3_225 GTYE4_CHANNEL_X1Y23 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC Y6   } [get_ports {pcie1_tx_n[0]}]  ;# MGTYTXN3_225 GTYE4_CHANNEL_X1Y23 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AL2  } [get_ports {pcie1_rx_p[1]}]  ;# MGTYRXP2_225 GTYE4_CHANNEL_X1Y22 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC AB1  } [get_ports {pcie1_rx_n[1]}]  ;# MGTYRXN2_225 GTYE4_CHANNEL_X1Y22 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AL7  } [get_ports {pcie1_tx_p[1]}]  ;# MGTYTXP2_225 GTYE4_CHANNEL_X1Y22 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC AB6  } [get_ports {pcie1_tx_n[1]}]  ;# MGTYTXN2_225 GTYE4_CHANNEL_X1Y22 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AM4  } [get_ports {pcie1_rx_p[2]}]  ;# MGTYRXP1_225 GTYE4_CHANNEL_X1Y21 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC AC3  } [get_ports {pcie1_rx_n[2]}]  ;# MGTYRXN1_225 GTYE4_CHANNEL_X1Y21 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AM9  } [get_ports {pcie1_tx_p[2]}]  ;# MGTYTXP1_225 GTYE4_CHANNEL_X1Y21 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC AD6  } [get_ports {pcie1_tx_n[2]}]  ;# MGTYTXN1_225 GTYE4_CHANNEL_X1Y21 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AN2  } [get_ports {pcie1_rx_p[3]}]  ;# MGTYRXP0_225 GTYE4_CHANNEL_X1Y20 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC AD1  } [get_ports {pcie1_rx_n[3]}]  ;# MGTYRXN0_225 GTYE4_CHANNEL_X1Y20 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AN7  } [get_ports {pcie1_tx_p[3]}]  ;# MGTYTXP0_225 GTYE4_CHANNEL_X1Y20 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC AF6  } [get_ports {pcie1_tx_n[3]}]  ;# MGTYTXN0_225 GTYE4_CHANNEL_X1Y20 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AP4  } [get_ports {pcie1_rx_p[4]}]  ;# MGTYRXP3_224 GTYE4_CHANNEL_X1Y27 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AE3  } [get_ports {pcie1_rx_n[4]}]  ;# MGTYRXN3_224 GTYE4_CHANNEL_X1Y27 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AP9  } [get_ports {pcie1_tx_p[4]}]  ;# MGTYTXP3_224 GTYE4_CHANNEL_X1Y27 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AH6  } [get_ports {pcie1_tx_n[4]}]  ;# MGTYTXN3_224 GTYE4_CHANNEL_X1Y27 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AR2  } [get_ports {pcie1_rx_p[5]}]  ;# MGTYRXP2_224 GTYE4_CHANNEL_X1Y26 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AF1  } [get_ports {pcie1_rx_n[5]}]  ;# MGTYRXN2_224 GTYE4_CHANNEL_X1Y26 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AR7  } [get_ports {pcie1_tx_p[5]}]  ;# MGTYTXP2_224 GTYE4_CHANNEL_X1Y26 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AK6  } [get_ports {pcie1_tx_n[5]}]  ;# MGTYTXN2_224 GTYE4_CHANNEL_X1Y26 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AT4  } [get_ports {pcie1_rx_p[6]}]  ;# MGTYRXP1_224 GTYE4_CHANNEL_X1Y25 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AG3  } [get_ports {pcie1_rx_n[6]}]  ;# MGTYRXN1_224 GTYE4_CHANNEL_X1Y25 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AT9  } [get_ports {pcie1_tx_p[6]}]  ;# MGTYTXP1_224 GTYE4_CHANNEL_X1Y25 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AM6  } [get_ports {pcie1_tx_n[6]}]  ;# MGTYTXN1_224 GTYE4_CHANNEL_X1Y25 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AU2  } [get_ports {pcie1_rx_p[7]}]  ;# MGTYRXP0_224 GTYE4_CHANNEL_X1Y24 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AH1  } [get_ports {pcie1_rx_n[7]}]  ;# MGTYRXN0_224 GTYE4_CHANNEL_X1Y24 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AU7  } [get_ports {pcie1_tx_p[7]}]  ;# MGTYTXP0_224 GTYE4_CHANNEL_X1Y24 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AN4  } [get_ports {pcie1_tx_n[7]}]  ;# MGTYTXN0_224 GTYE4_CHANNEL_X1Y24 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AM13  } [get_ports pcie1_refclk_0_p] ;# MGTREFCLK0P_225
#set_property -dict {LOC AC8  } [get_ports pcie1_refclk_0_n] ;# MGTREFCLK0N_225
#set_property -dict {LOC AL11  } [get_ports pcie1_refclk_1_p] ;# MGTREFCLK1P_225
#set_property -dict {LOC AL8  } [get_ports pcie1_refclk_1_n] ;# MGTREFCLK1N_225
set_property -dict {LOC BB15 IOSTANDARD LVCMOS18 PULLUP true} [get_ports pcie1_reset_n]

set_false_path -from [get_ports {pcie1_reset_n}]
set_input_delay 0 [get_ports {pcie1_reset_n}]

# 100 MHz MGT reference clock
create_clock -period 10 -name pcie1_mgt_refclk_0 [get_ports pcie1_refclk_0_p]
#create_clock -period 10 -name pcie1_mgt_refclk_1 [get_ports pcie1_refclk_1_p]


############################### FPGA1 D2 ###############################

# QSFP28-C port
# Serdes ordered not swapped in FPGA, serdes swap handled by CMAC
#   QSFP    MGT_TRX
#   0       0
#   1       2
#   2       1
#   3       3
set_property -dict {LOC J50  } [get_ports qsfp2_1_rx1_p] ;# MGTYRXN0_230 GTYE4_CHANNEL_X1Y44 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC Y1  } [get_ports qsfp2_1_rx1_n] ;# MGTYRXP0_230 GTYE4_CHANNEL_X1Y44 / GTYE4_COMMON_X1Y11
set_property -dict {LOC J45  } [get_ports qsfp2_1_tx1_p] ;# MGTYTXN0_230 GTYE4_CHANNEL_X1Y44 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC V6  } [get_ports qsfp2_1_tx1_n] ;# MGTYTXP0_230 GTYE4_CHANNEL_X1Y44 / GTYE4_COMMON_X1Y11
set_property -dict {LOC H48  } [get_ports qsfp2_1_rx2_p] ;# MGTYRXN2_230 GTYE4_CHANNEL_X1Y46 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC W3  } [get_ports qsfp2_1_rx2_n] ;# MGTYRXP2_230 GTYE4_CHANNEL_X1Y46 / GTYE4_COMMON_X1Y11
set_property -dict {LOC H43  } [get_ports qsfp2_1_tx2_p] ;# MGTYTXN2_230 GTYE4_CHANNEL_X1Y46 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC T6  } [get_ports qsfp2_1_tx2_n] ;# MGTYTXP2_230 GTYE4_CHANNEL_X1Y46 / GTYE4_COMMON_X1Y11
set_property -dict {LOC G50  } [get_ports qsfp2_1_rx3_p] ;# MGTYRXN1_230 GTYE4_CHANNEL_X1Y45 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC V1  } [get_ports qsfp2_1_rx3_n] ;# MGTYRXP1_230 GTYE4_CHANNEL_X1Y45 / GTYE4_COMMON_X1Y11
set_property -dict {LOC G45  } [get_ports qsfp2_1_tx3_p] ;# MGTYTXN1_230 GTYE4_CHANNEL_X1Y45 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC P6  } [get_ports qsfp2_1_tx3_n] ;# MGTYTXP1_230 GTYE4_CHANNEL_X1Y45 / GTYE4_COMMON_X1Y11
set_property -dict {LOC F48  } [get_ports qsfp2_1_rx4_p] ;# MGTYRXN3_230 GTYE4_CHANNEL_X1Y47 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC U3  } [get_ports qsfp2_1_rx4_n] ;# MGTYRXP3_230 GTYE4_CHANNEL_X1Y47 / GTYE4_COMMON_X1Y11
set_property -dict {LOC F43  } [get_ports qsfp2_1_tx4_p] ;# MGTYTXN3_230 GTYE4_CHANNEL_X1Y47 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC M6  } [get_ports qsfp2_1_tx4_n] ;# MGTYTXP3_230 GTYE4_CHANNEL_X1Y47 / GTYE4_COMMON_X1Y11
set_property -dict {LOC R41  } [get_ports qsfp2_1_mgt_refclk_0_p] ;# MGTREFCLK0P_230 
#set_property -dict {LOC W8  } [get_ports qsfp2_1_mgt_refclk_0_n] ;# MGTREFCLK0N_230 

# 161.1328125 MHz MGT reference clock
# create_clock -period 6.206 -name qsfp2_1_mgt_refclk_0 [get_ports qsfp2_1_mgt_refclk_0_p]

# Interconnection-F port
#   CH      MGT_TRX
#   0       0
#   1       1
#   2       2
#   3       3
set_property -dict {LOC U50  } [get_ports qsfp2_2_rx1_p] ;# MGTYRXN0_129 GTYE4_CHANNEL_X0Y40 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC T1  } [get_ports qsfp2_2_rx1_n] ;# MGTYRXP0_129 GTYE4_CHANNEL_X0Y40 / GTYE4_COMMON_X0Y??
set_property -dict {LOC U45  } [get_ports qsfp2_2_tx1_p] ;# MGTYTXN0_129 GTYE4_CHANNEL_X0Y40 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC L4  } [get_ports qsfp2_2_tx1_n] ;# MGTYTXP0_129 GTYE4_CHANNEL_X0Y40 / GTYE4_COMMON_X0Y??
set_property -dict {LOC T48  } [get_ports qsfp2_2_rx2_p] ;# MGTYRXN1_129 GTYE4_CHANNEL_X0Y41 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC R3  } [get_ports qsfp2_2_rx2_n] ;# MGTYRXP1_129 GTYE4_CHANNEL_X0Y41 / GTYE4_COMMON_X0Y??
set_property -dict {LOC T43  } [get_ports qsfp2_2_tx2_p] ;# MGTYTXN1_129 GTYE4_CHANNEL_X0Y41 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC K6  } [get_ports qsfp2_2_tx2_n] ;# MGTYTXP1_129 GTYE4_CHANNEL_X0Y41 / GTYE4_COMMON_X0Y??
set_property -dict {LOC R50  } [get_ports qsfp2_2_rx3_p] ;# MGTYRXN2_129 GTYE4_CHANNEL_X0Y42 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC P1  } [get_ports qsfp2_2_rx3_n] ;# MGTYRXP2_129 GTYE4_CHANNEL_X0Y42 / GTYE4_COMMON_X0Y??
set_property -dict {LOC R45  } [get_ports qsfp2_2_tx3_p] ;# MGTYTXN2_129 GTYE4_CHANNEL_X0Y42 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC J4  } [get_ports qsfp2_2_tx3_n] ;# MGTYTXP2_129 GTYE4_CHANNEL_X0Y42 / GTYE4_COMMON_X0Y??
set_property -dict {LOC P48  } [get_ports qsfp2_2_rx4_p] ;# MGTYRXN3_129 GTYE4_CHANNEL_X0Y43 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC M1  } [get_ports qsfp2_2_rx4_n] ;# MGTYRXP3_129 GTYE4_CHANNEL_X0Y43 / GTYE4_COMMON_X0Y??
set_property -dict {LOC P43  } [get_ports qsfp2_2_tx4_p] ;# MGTYTXN3_129 GTYE4_CHANNEL_X0Y43 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC H6  } [get_ports qsfp2_2_tx4_n] ;# MGTYTXP3_129 GTYE4_CHANNEL_X0Y43 / GTYE4_COMMON_X0Y??
set_property -dict {LOC U41  } [get_ports qsfp2_2_mgt_refclk_0_p] ;# MGTREFCLK0P_130
#set_property -dict {LOC R8  } [get_ports qsfp2_2_mgt_refclk_0_n] ;# MGTREFCLK0N_130

# 161.1328125 MHz MGT reference clock
# create_clock -period 6.206 -name qsfp2_2_mgt_refclk_0 [get_ports qsfp2_2_mgt_refclk_0_p]

# PCIe Interface F
set_property -dict {LOC K4  } [get_ports {pcie2_rx_p[0]}]  ;# MGTYRXP3_131 GTYE4_CHANNEL_X0Y48 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC AA3  } [get_ports {pcie2_rx_n[0]}]  ;# MGTYRXN3_131 GTYE4_CHANNEL_X0Y48 / GTYE4_COMMON_X0Y12
set_property -dict {LOC K9  } [get_ports {pcie2_tx_p[0]}]  ;# MGTYTXP3_131 GTYE4_CHANNEL_X0Y48 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC Y6   } [get_ports {pcie2_tx_n[0]}]  ;# MGTYTXN3_131 GTYE4_CHANNEL_X0Y48 / GTYE4_COMMON_X0Y12
set_property -dict {LOC L2  } [get_ports {pcie2_rx_p[1]}]  ;# MGTYRXP2_131 GTYE4_CHANNEL_X0Y49 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC AB1  } [get_ports {pcie2_rx_n[1]}]  ;# MGTYRXN2_131 GTYE4_CHANNEL_X0Y49 / GTYE4_COMMON_X0Y12
set_property -dict {LOC L7  } [get_ports {pcie2_tx_p[1]}]  ;# MGTYTXP2_131 GTYE4_CHANNEL_X0Y49 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC AB6  } [get_ports {pcie2_tx_n[1]}]  ;# MGTYTXN2_131 GTYE4_CHANNEL_X0Y49 / GTYE4_COMMON_X0Y12
set_property -dict {LOC M4  } [get_ports {pcie2_rx_p[2]}]  ;# MGTYRXP1_131 GTYE4_CHANNEL_X0Y50 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC AC3  } [get_ports {pcie2_rx_n[2]}]  ;# MGTYRXN1_131 GTYE4_CHANNEL_X0Y50 / GTYE4_COMMON_X0Y12
set_property -dict {LOC M9  } [get_ports {pcie2_tx_p[2]}]  ;# MGTYTXP1_131 GTYE4_CHANNEL_X0Y50 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC AD6  } [get_ports {pcie2_tx_n[2]}]  ;# MGTYTXN1_131 GTYE4_CHANNEL_X0Y50 / GTYE4_COMMON_X0Y12
set_property -dict {LOC N2  } [get_ports {pcie2_rx_p[3]}]  ;# MGTYRXP0_131 GTYE4_CHANNEL_X0Y51 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC AD1  } [get_ports {pcie2_rx_n[3]}]  ;# MGTYRXN0_131 GTYE4_CHANNEL_X0Y51 / GTYE4_COMMON_X0Y12
set_property -dict {LOC N7  } [get_ports {pcie2_tx_p[3]}]  ;# MGTYTXP0_131 GTYE4_CHANNEL_X0Y51 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC AF6  } [get_ports {pcie2_tx_n[3]}]  ;# MGTYTXN0_131 GTYE4_CHANNEL_X0Y51 / GTYE4_COMMON_X0Y12
set_property -dict {LOC P4  } [get_ports {pcie2_rx_p[4]}]  ;# MGTYRXP3_132 GTYE4_CHANNEL_X0Y52 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AE3  } [get_ports {pcie2_rx_n[4]}]  ;# MGTYRXN3_132 GTYE4_CHANNEL_X0Y52 / GTYE4_COMMON_X0Y13
set_property -dict {LOC P9  } [get_ports {pcie2_tx_p[4]}]  ;# MGTYTXP3_132 GTYE4_CHANNEL_X0Y52 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AH6  } [get_ports {pcie2_tx_n[4]}]  ;# MGTYTXN3_132 GTYE4_CHANNEL_X0Y52 / GTYE4_COMMON_X0Y13
set_property -dict {LOC R2  } [get_ports {pcie2_rx_p[5]}]  ;# MGTYRXP2_132 GTYE4_CHANNEL_X0Y53 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AF1  } [get_ports {pcie2_rx_n[5]}]  ;# MGTYRXN2_132 GTYE4_CHANNEL_X0Y53 / GTYE4_COMMON_X0Y13
set_property -dict {LOC R7  } [get_ports {pcie2_tx_p[5]}]  ;# MGTYTXP2_132 GTYE4_CHANNEL_X0Y53 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AK6  } [get_ports {pcie2_tx_n[5]}]  ;# MGTYTXN2_132 GTYE4_CHANNEL_X0Y53 / GTYE4_COMMON_X0Y13
set_property -dict {LOC T4  } [get_ports {pcie2_rx_p[6]}]  ;# MGTYRXP1_132 GTYE4_CHANNEL_X0Y54 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AG3  } [get_ports {pcie2_rx_n[6]}]  ;# MGTYRXN1_132 GTYE4_CHANNEL_X0Y54 / GTYE4_COMMON_X0Y13
set_property -dict {LOC T9  } [get_ports {pcie2_tx_p[6]}]  ;# MGTYTXP1_132 GTYE4_CHANNEL_X0Y54 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AM6  } [get_ports {pcie2_tx_n[6]}]  ;# MGTYTXN1_132 GTYE4_CHANNEL_X0Y54 / GTYE4_COMMON_X0Y13
set_property -dict {LOC U2  } [get_ports {pcie2_rx_p[7]}]  ;# MGTYRXP0_132 GTYE4_CHANNEL_X0Y55 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AH1  } [get_ports {pcie2_rx_n[7]}]  ;# MGTYRXN0_132 GTYE4_CHANNEL_X0Y55 / GTYE4_COMMON_X0Y13
set_property -dict {LOC U7  } [get_ports {pcie2_tx_p[7]}]  ;# MGTYTXP0_132 GTYE4_CHANNEL_X0Y55 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AN4  } [get_ports {pcie2_tx_n[7]}]  ;# MGTYTXN0_132 GTYE4_CHANNEL_X0Y55 / GTYE4_COMMON_X0Y13
set_property -dict {LOC U11  } [get_ports pcie2_refclk_0_p] ;# MGTREFCLK0P_132
#set_property -dict {LOC AC8  } [get_ports pcie2_refclk_0_n] ;# MGTREFCLK0N_132
#set_property -dict {LOC T13  } [get_ports pcie2_refclk_1_p] ;# MGTREFCLK1P_132
#set_property -dict {LOC AL8  } [get_ports pcie2_refclk_1_n] ;# MGTREFCLK1N_132
set_property -dict {LOC P23 IOSTANDARD LVCMOS18 PULLUP true} [get_ports pcie2_reset_n]

set_false_path -from [get_ports {pcie2_reset_n}]
set_input_delay 0 [get_ports {pcie2_reset_n}]

# 100 MHz MGT reference clock
create_clock -period 10 -name pcie2_mgt_refclk_0 [get_ports pcie2_refclk_0_p]
#create_clock -period 10 -name pcie2_mgt_refclk_1 [get_ports pcie2_refclk_1_p]

