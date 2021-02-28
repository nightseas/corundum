# XDC constraints for the FPGA-EVB-Pro FPGA0 D1
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
# System reset
#set_property -dict {LOC BH26  IOSTANDARD LVCMOS12} [get_ports sys_reset_n]

# QSFP28-E port
# Serdes ordered swapped in FPGA
#   QSFP    MGT_TRX
#   0       0
#   1       2
#   2       1
#   3       3
set_property -dict {LOC AJ2  } [get_ports qsfp1_rx1_p] ;# MGTYRXN0_226 GTYE4_CHANNEL_X1Y28 / GTYE4_COMMON_X1Y7
#set_property -dict {LOC Y1  } [get_ports qsfp1_rx1_n] ;# MGTYRXP0_226 GTYE4_CHANNEL_X1Y28 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AJ7  } [get_ports qsfp1_tx1_p] ;# MGTYTXN0_226 GTYE4_CHANNEL_X1Y28 / GTYE4_COMMON_X1Y7
#set_property -dict {LOC V6  } [get_ports qsfp1_tx1_n] ;# MGTYTXP0_226 GTYE4_CHANNEL_X1Y28 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AG2  } [get_ports qsfp1_rx2_p] ;# MGTYRXN2_226 GTYE4_CHANNEL_X1Y30 / GTYE4_COMMON_X1Y7
#set_property -dict {LOC W3  } [get_ports qsfp1_rx2_n] ;# MGTYRXP2_226 GTYE4_CHANNEL_X1Y30 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AG7  } [get_ports qsfp1_tx2_p] ;# MGTYTXN2_226 GTYE4_CHANNEL_X1Y30 / GTYE4_COMMON_X1Y7
#set_property -dict {LOC T6  } [get_ports qsfp1_tx2_n] ;# MGTYTXP2_226 GTYE4_CHANNEL_X1Y30 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AH4  } [get_ports qsfp1_rx3_p] ;# MGTYRXN1_226 GTYE4_CHANNEL_X1Y29 / GTYE4_COMMON_X1Y7
#set_property -dict {LOC V1  } [get_ports qsfp1_rx3_n] ;# MGTYRXP1_226 GTYE4_CHANNEL_X1Y29 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AH9  } [get_ports qsfp1_tx3_p] ;# MGTYTXN1_226 GTYE4_CHANNEL_X1Y29 / GTYE4_COMMON_X1Y7
#set_property -dict {LOC P6  } [get_ports qsfp1_tx3_n] ;# MGTYTXP1_226 GTYE4_CHANNEL_X1Y29 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AF4  } [get_ports qsfp1_rx4_p] ;# MGTYRXN3_226 GTYE4_CHANNEL_X1Y31 / GTYE4_COMMON_X1Y7
#set_property -dict {LOC U3  } [get_ports qsfp1_rx4_n] ;# MGTYRXP3_226 GTYE4_CHANNEL_X1Y31 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AF9  } [get_ports qsfp1_tx4_p] ;# MGTYTXN3_226 GTYE4_CHANNEL_X1Y31 / GTYE4_COMMON_X1Y7
#set_property -dict {LOC M6  } [get_ports qsfp1_tx4_n] ;# MGTYTXP3_226 GTYE4_CHANNEL_X1Y31 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AJ11  } [get_ports qsfp1_mgt_refclk_0_p] ;# MGTREFCLK0P_226 
#set_property -dict {LOC W8  } [get_ports qsfp1_mgt_refclk_0_n] ;# MGTREFCLK0N_226 

# 161.1328125 MHz MGT reference clock
create_clock -period 6.206 -name qsfp1_mgt_refclk_0 [get_ports qsfp1_mgt_refclk_0_p]

# Interconnection-E port
#   Inter   MGT_TRX
#   0       0
#   1       1
#   2       2
#   3       3
set_property -dict {LOC AE2  } [get_ports qsfp2_rx1_p] ;# MGTYRXN0_227 GTYE4_CHANNEL_X1Y32 / GTYE4_COMMON_X1Y8
#set_property -dict {LOC T1  } [get_ports qsfp2_rx1_n] ;# MGTYRXP0_227 GTYE4_CHANNEL_X1Y32 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AE7  } [get_ports qsfp2_tx1_p] ;# MGTYTXN0_227 GTYE4_CHANNEL_X1Y32 / GTYE4_COMMON_X1Y8
#set_property -dict {LOC L4  } [get_ports qsfp2_tx1_n] ;# MGTYTXP0_227 GTYE4_CHANNEL_X1Y32 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AD4  } [get_ports qsfp2_rx2_p] ;# MGTYRXN1_227 GTYE4_CHANNEL_X1Y33 / GTYE4_COMMON_X1Y8
#set_property -dict {LOC R3  } [get_ports qsfp2_rx2_n] ;# MGTYRXP1_227 GTYE4_CHANNEL_X1Y33 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AD9  } [get_ports qsfp2_tx2_p] ;# MGTYTXN1_227 GTYE4_CHANNEL_X1Y33 / GTYE4_COMMON_X1Y8
#set_property -dict {LOC K6  } [get_ports qsfp2_tx2_n] ;# MGTYTXP1_227 GTYE4_CHANNEL_X1Y33 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AC2  } [get_ports qsfp2_rx3_p] ;# MGTYRXN2_227 GTYE4_CHANNEL_X1Y34 / GTYE4_COMMON_X1Y8
#set_property -dict {LOC P1  } [get_ports qsfp2_rx3_n] ;# MGTYRXP2_227 GTYE4_CHANNEL_X1Y34 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AC7  } [get_ports qsfp2_tx3_p] ;# MGTYTXN2_227 GTYE4_CHANNEL_X1Y34 / GTYE4_COMMON_X1Y8
#set_property -dict {LOC J4  } [get_ports qsfp2_tx3_n] ;# MGTYTXP2_227 GTYE4_CHANNEL_X1Y34 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AB4  } [get_ports qsfp2_rx4_p] ;# MGTYRXN3_227 GTYE4_CHANNEL_X1Y35 / GTYE4_COMMON_X1Y8
#set_property -dict {LOC M1  } [get_ports qsfp2_rx4_n] ;# MGTYRXP3_227 GTYE4_CHANNEL_X1Y35 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AB9  } [get_ports qsfp2_tx4_p] ;# MGTYTXN3_227 GTYE4_CHANNEL_X1Y35 / GTYE4_COMMON_X1Y8
#set_property -dict {LOC H6  } [get_ports qsfp2_tx4_n] ;# MGTYTXP3_227 GTYE4_CHANNEL_X1Y35 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AE11  } [get_ports qsfp2_mgt_refclk_0_p] ;# MGTREFCLK0P_227
#set_property -dict {LOC R8  } [get_ports qsfp2_mgt_refclk_0_n] ;# MGTREFCLK0N_227

# 161.1328125 MHz MGT reference clock
create_clock -period 6.206 -name qsfp2_mgt_refclk_0 [get_ports qsfp2_mgt_refclk_0_p]

# I2C interface
set_property -dict {LOC N36 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports i2c_scl]
set_property -dict {LOC N37 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports i2c_sda]

# PCIe Interface A
set_property -dict {LOC AK4  } [get_ports {pcie_rx_p[0]}]  ;# MGTYRXP3_225 GTYE4_CHANNEL_X1Y27 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC AA3  } [get_ports {pcie_rx_n[0]}]  ;# MGTYRXN3_225 GTYE4_CHANNEL_X1Y27 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AK9  } [get_ports {pcie_tx_p[0]}]  ;# MGTYTXP3_225 GTYE4_CHANNEL_X1Y27 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC Y6   } [get_ports {pcie_tx_n[0]}]  ;# MGTYTXN3_225 GTYE4_CHANNEL_X1Y27 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AL2  } [get_ports {pcie_rx_p[1]}]  ;# MGTYRXP2_225 GTYE4_CHANNEL_X1Y26 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC AB1  } [get_ports {pcie_rx_n[1]}]  ;# MGTYRXN2_225 GTYE4_CHANNEL_X1Y26 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AL7  } [get_ports {pcie_tx_p[1]}]  ;# MGTYTXP2_225 GTYE4_CHANNEL_X1Y26 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC AB6  } [get_ports {pcie_tx_n[1]}]  ;# MGTYTXN2_225 GTYE4_CHANNEL_X1Y26 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AM4  } [get_ports {pcie_rx_p[2]}]  ;# MGTYRXP1_225 GTYE4_CHANNEL_X1Y25 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC AC3  } [get_ports {pcie_rx_n[2]}]  ;# MGTYRXN1_225 GTYE4_CHANNEL_X1Y25 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AM9  } [get_ports {pcie_tx_p[2]}]  ;# MGTYTXP1_225 GTYE4_CHANNEL_X1Y25 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC AD6  } [get_ports {pcie_tx_n[2]}]  ;# MGTYTXN1_225 GTYE4_CHANNEL_X1Y25 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AN2  } [get_ports {pcie_rx_p[3]}]  ;# MGTYRXP0_225 GTYE4_CHANNEL_X1Y24 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC AD1  } [get_ports {pcie_rx_n[3]}]  ;# MGTYRXN0_225 GTYE4_CHANNEL_X1Y24 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AN7  } [get_ports {pcie_tx_p[3]}]  ;# MGTYTXP0_225 GTYE4_CHANNEL_X1Y24 / GTYE4_COMMON_X1Y6
#set_property -dict {LOC AF6  } [get_ports {pcie_tx_n[3]}]  ;# MGTYTXN0_225 GTYE4_CHANNEL_X1Y24 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AP4  } [get_ports {pcie_rx_p[4]}]  ;# MGTYRXP3_224 GTYE4_CHANNEL_X1Y23 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AE3  } [get_ports {pcie_rx_n[4]}]  ;# MGTYRXN3_224 GTYE4_CHANNEL_X1Y23 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AP9  } [get_ports {pcie_tx_p[4]}]  ;# MGTYTXP3_224 GTYE4_CHANNEL_X1Y23 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AH6  } [get_ports {pcie_tx_n[4]}]  ;# MGTYTXN3_224 GTYE4_CHANNEL_X1Y23 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AR2  } [get_ports {pcie_rx_p[5]}]  ;# MGTYRXP2_224 GTYE4_CHANNEL_X1Y22 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AF1  } [get_ports {pcie_rx_n[5]}]  ;# MGTYRXN2_224 GTYE4_CHANNEL_X1Y22 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AR7  } [get_ports {pcie_tx_p[5]}]  ;# MGTYTXP2_224 GTYE4_CHANNEL_X1Y22 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AK6  } [get_ports {pcie_tx_n[5]}]  ;# MGTYTXN2_224 GTYE4_CHANNEL_X1Y22 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AT4  } [get_ports {pcie_rx_p[6]}]  ;# MGTYRXP1_224 GTYE4_CHANNEL_X1Y21 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AG3  } [get_ports {pcie_rx_n[6]}]  ;# MGTYRXN1_224 GTYE4_CHANNEL_X1Y21 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AT9  } [get_ports {pcie_tx_p[6]}]  ;# MGTYTXP1_224 GTYE4_CHANNEL_X1Y21 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AM6  } [get_ports {pcie_tx_n[6]}]  ;# MGTYTXN1_224 GTYE4_CHANNEL_X1Y21 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AU2  } [get_ports {pcie_rx_p[7]}]  ;# MGTYRXP0_224 GTYE4_CHANNEL_X1Y20 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AH1  } [get_ports {pcie_rx_n[7]}]  ;# MGTYRXN0_224 GTYE4_CHANNEL_X1Y20 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AU7  } [get_ports {pcie_tx_p[7]}]  ;# MGTYTXP0_224 GTYE4_CHANNEL_X1Y20 / GTYE4_COMMON_X1Y5
#set_property -dict {LOC AN4  } [get_ports {pcie_tx_n[7]}]  ;# MGTYTXN0_224 GTYE4_CHANNEL_X1Y20 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AM13  } [get_ports pcie_refclk_0_p] ;# MGTREFCLK0P_225
#set_property -dict {LOC AC8  } [get_ports pcie_refclk_0_n] ;# MGTREFCLK0N_225
#set_property -dict {LOC AL11  } [get_ports pcie_refclk_1_p] ;# MGTREFCLK1P_225
#set_property -dict {LOC AL8  } [get_ports pcie_refclk_1_n] ;# MGTREFCLK1N_225
set_property -dict {LOC BB15 IOSTANDARD LVCMOS18 PULLUP true} [get_ports pcie_reset_n]

# 100 MHz MGT reference clock
create_clock -period 10 -name pcie_mgt_refclk_0 [get_ports pcie_refclk_0_p]
#create_clock -period 10 -name pcie_mgt_refclk_1 [get_ports pcie_refclk_1_p]
