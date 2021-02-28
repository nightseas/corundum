# XDC constraints for the FPGA-EVB-Pro FPGA1 D0
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
#set_property -dict {LOC AR17 IOSTANDARD LVCMOS18} [get_ports clk_100mhz]
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
# System reset
#set_property -dict {LOC BH26  IOSTANDARD LVCMOS12} [get_ports sys_reset_n]

# QSFP28-A port
# Serdes ordered not swapped in FPGA, need to swap in software
#   QSFP    MGT_TRX
#   0       0
#   1       2
#   2       1
#   3       3
set_property -dict {LOC BA50  } [get_ports qsfp1_rx1_p] ;# MGTYRXN0_131 GTYE4_CHANNEL_X0Y16 / GTYE4_COMMON_X0Y4
#set_property -dict {LOC Y1  } [get_ports qsfp1_rx1_n] ;# MGTYRXP0_131 GTYE4_CHANNEL_X0Y16 / GTYE4_COMMON_X0Y4
set_property -dict {LOC BA45  } [get_ports qsfp1_tx1_p] ;# MGTYTXN0_131 GTYE4_CHANNEL_X0Y16 / GTYE4_COMMON_X0Y4
#set_property -dict {LOC V6  } [get_ports qsfp1_tx1_n] ;# MGTYTXP0_131 GTYE4_CHANNEL_X0Y16 / GTYE4_COMMON_X0Y4
set_property -dict {LOC AY48  } [get_ports qsfp1_rx2_p] ;# MGTYRXN2_131 GTYE4_CHANNEL_X0Y18 / GTYE4_COMMON_X0Y4
#set_property -dict {LOC W3  } [get_ports qsfp1_rx2_n] ;# MGTYRXP2_131 GTYE4_CHANNEL_X0Y18 / GTYE4_COMMON_X0Y4
set_property -dict {LOC AY43  } [get_ports qsfp1_tx2_p] ;# MGTYTXN2_131 GTYE4_CHANNEL_X0Y18 / GTYE4_COMMON_X0Y4
#set_property -dict {LOC T6  } [get_ports qsfp1_tx2_n] ;# MGTYTXP2_131 GTYE4_CHANNEL_X0Y18 / GTYE4_COMMON_X0Y4
set_property -dict {LOC AW50  } [get_ports qsfp1_rx3_p] ;# MGTYRXN1_131 GTYE4_CHANNEL_X0Y17 / GTYE4_COMMON_X0Y4
#set_property -dict {LOC V1  } [get_ports qsfp1_rx3_n] ;# MGTYRXP1_131 GTYE4_CHANNEL_X0Y17 / GTYE4_COMMON_X0Y4
set_property -dict {LOC AW45  } [get_ports qsfp1_tx3_p] ;# MGTYTXN1_131 GTYE4_CHANNEL_X0Y17 / GTYE4_COMMON_X0Y4
#set_property -dict {LOC P6  } [get_ports qsfp1_tx3_n] ;# MGTYTXP1_131 GTYE4_CHANNEL_X0Y17 / GTYE4_COMMON_X0Y4
set_property -dict {LOC AV48  } [get_ports qsfp1_rx4_p] ;# MGTYRXN3_131 GTYE4_CHANNEL_X0Y19 / GTYE4_COMMON_X0Y4
#set_property -dict {LOC U3  } [get_ports qsfp1_rx4_n] ;# MGTYRXP3_131 GTYE4_CHANNEL_X0Y19 / GTYE4_COMMON_X0Y4
set_property -dict {LOC AV43  } [get_ports qsfp1_tx4_p] ;# MGTYTXN3_131 GTYE4_CHANNEL_X0Y19 / GTYE4_COMMON_X0Y4
#set_property -dict {LOC M6  } [get_ports qsfp1_tx4_n] ;# MGTYTXP3_131 GTYE4_CHANNEL_X0Y19 / GTYE4_COMMON_X0Y4
set_property -dict {LOC AT39  } [get_ports qsfp1_mgt_refclk_0_p] ;# MGTREFCLK0P_131 
#set_property -dict {LOC W8  } [get_ports qsfp1_mgt_refclk_0_n] ;# MGTREFCLK0N_131 

# 161.1328125 MHz MGT reference clock
# create_clock -period 6.206 -name qsfp1_mgt_refclk_0 [get_ports qsfp1_mgt_refclk_0_p]

# Interconnection-D port
#   CH      MGT_TRX
#   0       0
#   1       1
#   2       2
#   3       3
set_property -dict {LOC BH48  } [get_ports qsfp2_rx1_p] ;# MGTYRXN0_121 GTYE4_CHANNEL_X0Y8 / GTYE4_COMMON_X0Y2
#set_property -dict {LOC T1  } [get_ports qsfp2_rx1_n] ;# MGTYRXP0_121 GTYE4_CHANNEL_X0Y8 / GTYE4_COMMON_X0Y2
set_property -dict {LOC BG41  } [get_ports qsfp2_tx1_p] ;# MGTYTXN0_121 GTYE4_CHANNEL_X0Y8 / GTYE4_COMMON_X0Y2
#set_property -dict {LOC L4  } [get_ports qsfp2_tx1_n] ;# MGTYTXP0_121 GTYE4_CHANNEL_X0Y8 / GTYE4_COMMON_X0Y2
set_property -dict {LOC BG50  } [get_ports qsfp2_rx2_p] ;# MGTYRXN1_121 GTYE4_CHANNEL_X0Y9 / GTYE4_COMMON_X0Y2
#set_property -dict {LOC R3  } [get_ports qsfp2_rx2_n] ;# MGTYRXP1_121 GTYE4_CHANNEL_X0Y9 / GTYE4_COMMON_X0Y2
set_property -dict {LOC BJ41  } [get_ports qsfp2_tx2_p] ;# MGTYTXN1_121 GTYE4_CHANNEL_X0Y9 / GTYE4_COMMON_X0Y2
#set_property -dict {LOC K6  } [get_ports qsfp2_tx2_n] ;# MGTYTXP1_121 GTYE4_CHANNEL_X0Y9 / GTYE4_COMMON_X0Y2
set_property -dict {LOC BG46  } [get_ports qsfp2_rx3_p] ;# MGTYRXN2_121 GTYE4_CHANNEL_X0Y10 / GTYE4_COMMON_X0Y2
#set_property -dict {LOC P1  } [get_ports qsfp2_rx3_n] ;# MGTYRXP2_121 GTYE4_CHANNEL_X0Y10 / GTYE4_COMMON_X0Y2
set_property -dict {LOC BH43  } [get_ports qsfp2_tx3_p] ;# MGTYTXN2_121 GTYE4_CHANNEL_X0Y10 / GTYE4_COMMON_X0Y2
#set_property -dict {LOC J4  } [get_ports qsfp2_tx3_n] ;# MGTYTXP2_121 GTYE4_CHANNEL_X0Y10 / GTYE4_COMMON_X0Y2
set_property -dict {LOC BF48  } [get_ports qsfp2_rx4_p] ;# MGTYRXN3_121 GTYE4_CHANNEL_X0Y11 / GTYE4_COMMON_X0Y2
#set_property -dict {LOC M1  } [get_ports qsfp2_rx4_n] ;# MGTYRXP3_121 GTYE4_CHANNEL_X0Y11 / GTYE4_COMMON_X0Y2
set_property -dict {LOC BF43  } [get_ports qsfp2_tx4_p] ;# MGTYTXN3_121 GTYE4_CHANNEL_X0Y11 / GTYE4_COMMON_X0Y2
#set_property -dict {LOC H6  } [get_ports qsfp2_tx4_n] ;# MGTYTXP3_121 GTYE4_CHANNEL_X0Y11 / GTYE4_COMMON_X0Y2
set_property -dict {LOC AV39  } [get_ports qsfp2_mgt_refclk_0_p] ;# MGTREFCLK0P_122
#set_property -dict {LOC R8  } [get_ports qsfp2_mgt_refclk_0_n] ;# MGTREFCLK0N_122

# 161.1328125 MHz MGT reference clock
# create_clock -period 6.206 -name qsfp2_mgt_refclk_0 [get_ports qsfp2_mgt_refclk_0_p]

# PCIe Interface E
set_property -dict {LOC BJ6   } [get_ports {pcie_rx_p[0]}]  ;# MGTYRXP3_120 GTYE4_CHANNEL_X1Y7 / GTYE4_COMMON_X1Y1
#set_property -dict {LOC AA3  } [get_ports {pcie_rx_n[0]}]  ;# MGTYRXN3_120 GTYE4_CHANNEL_X1Y7 / GTYE4_COMMON_X1Y1
set_property -dict {LOC BK9   } [get_ports {pcie_tx_p[0]}]  ;# MGTYTXP3_120 GTYE4_CHANNEL_X1Y7 / GTYE4_COMMON_X1Y1
#set_property -dict {LOC Y6   } [get_ports {pcie_tx_n[0]}]  ;# MGTYTXN3_120 GTYE4_CHANNEL_X1Y7 / GTYE4_COMMON_X1Y1
set_property -dict {LOC BL6   } [get_ports {pcie_rx_p[1]}]  ;# MGTYRXP2_120 GTYE4_CHANNEL_X1Y6 / GTYE4_COMMON_X1Y1
#set_property -dict {LOC AB1  } [get_ports {pcie_rx_n[1]}]  ;# MGTYRXN2_120 GTYE4_CHANNEL_X1Y6 / GTYE4_COMMON_X1Y1
set_property -dict {LOC BL11  } [get_ports {pcie_tx_p[1]}]  ;# MGTYTXP2_120 GTYE4_CHANNEL_X1Y6 / GTYE4_COMMON_X1Y1
#set_property -dict {LOC AB6  } [get_ports {pcie_tx_n[1]}]  ;# MGTYTXN2_120 GTYE4_CHANNEL_X1Y6 / GTYE4_COMMON_X1Y1
set_property -dict {LOC BL20  } [get_ports {pcie_rx_p[2]}]  ;# MGTYRXP1_120 GTYE4_CHANNEL_X1Y5 / GTYE4_COMMON_X1Y1
#set_property -dict {LOC AC3  } [get_ports {pcie_rx_n[2]}]  ;# MGTYRXN1_120 GTYE4_CHANNEL_X1Y5 / GTYE4_COMMON_X1Y1
set_property -dict {LOC BK13  } [get_ports {pcie_tx_p[2]}]  ;# MGTYTXP1_120 GTYE4_CHANNEL_X1Y5 / GTYE4_COMMON_X1Y1
#set_property -dict {LOC AD6  } [get_ports {pcie_tx_n[2]}]  ;# MGTYTXN1_120 GTYE4_CHANNEL_X1Y5 / GTYE4_COMMON_X1Y1
set_property -dict {LOC BK18  } [get_ports {pcie_rx_p[3]}]  ;# MGTYRXP0_120 GTYE4_CHANNEL_X1Y4 / GTYE4_COMMON_X1Y1
#set_property -dict {LOC AD1  } [get_ports {pcie_rx_n[3]}]  ;# MGTYRXN0_120 GTYE4_CHANNEL_X1Y4 / GTYE4_COMMON_X1Y1
set_property -dict {LOC BL15  } [get_ports {pcie_tx_p[3]}]  ;# MGTYTXP0_120 GTYE4_CHANNEL_X1Y4 / GTYE4_COMMON_X1Y1
#set_property -dict {LOC AF6  } [get_ports {pcie_tx_n[3]}]  ;# MGTYTXN0_120 GTYE4_CHANNEL_X1Y4 / GTYE4_COMMON_X1Y1
set_property -dict {LOC BH18  } [get_ports {pcie_rx_p[4]}]  ;# MGTYRXP3_119 GTYE4_CHANNEL_X1Y3 / GTYE4_COMMON_X1Y0
#set_property -dict {LOC AE3  } [get_ports {pcie_rx_n[4]}]  ;# MGTYRXN3_119 GTYE4_CHANNEL_X1Y3 / GTYE4_COMMON_X1Y0
set_property -dict {LOC BG15  } [get_ports {pcie_tx_p[4]}]  ;# MGTYTXP3_119 GTYE4_CHANNEL_X1Y3 / GTYE4_COMMON_X1Y0
#set_property -dict {LOC AH6  } [get_ports {pcie_tx_n[4]}]  ;# MGTYTXN3_119 GTYE4_CHANNEL_X1Y3 / GTYE4_COMMON_X1Y0
set_property -dict {LOC BJ20  } [get_ports {pcie_rx_p[5]}]  ;# MGTYRXP2_119 GTYE4_CHANNEL_X1Y6 / GTYE4_COMMON_X1Y0
#set_property -dict {LOC AF1  } [get_ports {pcie_rx_n[5]}]  ;# MGTYRXN2_119 GTYE4_CHANNEL_X1Y2 / GTYE4_COMMON_X1Y0
set_property -dict {LOC BJ15  } [get_ports {pcie_tx_p[5]}]  ;# MGTYTXP2_119 GTYE4_CHANNEL_X1Y2 / GTYE4_COMMON_X1Y0
#set_property -dict {LOC AK6  } [get_ports {pcie_tx_n[5]}]  ;# MGTYTXN2_119 GTYE4_CHANNEL_X1Y2 / GTYE4_COMMON_X1Y0
set_property -dict {LOC BF18  } [get_ports {pcie_rx_p[6]}]  ;# MGTYRXP1_119 GTYE4_CHANNEL_X1Y1 / GTYE4_COMMON_X1Y0
#set_property -dict {LOC AG3  } [get_ports {pcie_rx_n[6]}]  ;# MGTYRXN1_119 GTYE4_CHANNEL_X1Y1 / GTYE4_COMMON_X1Y0
set_property -dict {LOC BF13  } [get_ports {pcie_tx_p[6]}]  ;# MGTYTXP1_119 GTYE4_CHANNEL_X1Y1 / GTYE4_COMMON_X1Y0
#set_property -dict {LOC AM6  } [get_ports {pcie_tx_n[6]}]  ;# MGTYTXN1_119 GTYE4_CHANNEL_X1Y1 / GTYE4_COMMON_X1Y0
set_property -dict {LOC BG20  } [get_ports {pcie_rx_p[7]}]  ;# MGTYRXP0_119 GTYE4_CHANNEL_X1Y0 / GTYE4_COMMON_X1Y0
#set_property -dict {LOC AH1  } [get_ports {pcie_rx_n[7]}]  ;# MGTYRXN0_119 GTYE4_CHANNEL_X1Y0 / GTYE4_COMMON_X1Y0
set_property -dict {LOC BH13  } [get_ports {pcie_tx_p[7]}]  ;# MGTYTXP0_119 GTYE4_CHANNEL_X1Y0 / GTYE4_COMMON_X1Y0
#set_property -dict {LOC AN4  } [get_ports {pcie_tx_n[7]}]  ;# MGTYTXN0_119 GTYE4_CHANNEL_X1Y0 / GTYE4_COMMON_X1Y0
set_property -dict {LOC BB13  } [get_ports pcie_refclk_0_p] ;# MGTREFCLK0P_120
#set_property -dict {LOC AC8  } [get_ports pcie_refclk_0_n] ;# MGTREFCLK0N_120
#set_property -dict {LOC BA11  } [get_ports pcie_refclk_1_p] ;# MGTREFCLK1P_120
#set_property -dict {LOC AL8  } [get_ports pcie_refclk_1_n] ;# MGTREFCLK1N_120
set_property -dict {LOC AY31 IOSTANDARD LVCMOS18 PULLUP true} [get_ports pcie_reset_n]

# 100 MHz MGT reference clock
create_clock -period 10 -name pcie_mgt_refclk_0 [get_ports pcie_refclk_0_p]
#create_clock -period 10 -name pcie_mgt_refclk_1 [get_ports pcie_refclk_1_p]
