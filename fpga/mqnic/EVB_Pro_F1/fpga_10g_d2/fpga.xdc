# XDC constraints for the FPGA-EVB-Pro FPGA1 D2
# part: xcvu9p-flga2577-2-i

# General configuration
set_property CFGBVS GND                                [current_design]
set_property CONFIG_VOLTAGE 1.8                        [current_design]
set_property BITSTREAM.GENERAL.COMPRESS true           [current_design]
set_property BITSTREAM.CONFIG.EXTMASTERCCLK_EN {DIV-1} [current_design]
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

# QSFP28-C port
# Serdes order swapped in FPGA
#   QSFP    MGT_TRX
#   0       0
#   1       2
#   2       1
#   3       3
set_property -dict {LOC J50  } [get_ports qsfp1_rx1_p] ;# MGTYRXN0_230 GTYE4_CHANNEL_X1Y44 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC Y1  } [get_ports qsfp1_rx1_n] ;# MGTYRXP0_230 GTYE4_CHANNEL_X1Y44 / GTYE4_COMMON_X1Y11
set_property -dict {LOC J45  } [get_ports qsfp1_tx1_p] ;# MGTYTXN0_230 GTYE4_CHANNEL_X1Y44 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC V6  } [get_ports qsfp1_tx1_n] ;# MGTYTXP0_230 GTYE4_CHANNEL_X1Y44 / GTYE4_COMMON_X1Y11
set_property -dict {LOC G50  } [get_ports qsfp1_rx2_p] ;# MGTYRXN2_230 GTYE4_CHANNEL_X1Y46 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC W3  } [get_ports qsfp1_rx2_n] ;# MGTYRXP2_230 GTYE4_CHANNEL_X1Y46 / GTYE4_COMMON_X1Y11
set_property -dict {LOC G45  } [get_ports qsfp1_tx2_p] ;# MGTYTXN2_230 GTYE4_CHANNEL_X1Y46 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC T6  } [get_ports qsfp1_tx2_n] ;# MGTYTXP2_230 GTYE4_CHANNEL_X1Y46 / GTYE4_COMMON_X1Y11
set_property -dict {LOC H48  } [get_ports qsfp1_rx3_p] ;# MGTYRXN1_230 GTYE4_CHANNEL_X1Y45 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC V1  } [get_ports qsfp1_rx3_n] ;# MGTYRXP1_230 GTYE4_CHANNEL_X1Y45 / GTYE4_COMMON_X1Y11
set_property -dict {LOC H43  } [get_ports qsfp1_tx3_p] ;# MGTYTXN1_230 GTYE4_CHANNEL_X1Y45 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC P6  } [get_ports qsfp1_tx3_n] ;# MGTYTXP1_230 GTYE4_CHANNEL_X1Y45 / GTYE4_COMMON_X1Y11
set_property -dict {LOC F48  } [get_ports qsfp1_rx4_p] ;# MGTYRXN3_230 GTYE4_CHANNEL_X1Y47 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC U3  } [get_ports qsfp1_rx4_n] ;# MGTYRXP3_230 GTYE4_CHANNEL_X1Y47 / GTYE4_COMMON_X1Y11
set_property -dict {LOC F43  } [get_ports qsfp1_tx4_p] ;# MGTYTXN3_230 GTYE4_CHANNEL_X1Y47 / GTYE4_COMMON_X1Y11
#set_property -dict {LOC M6  } [get_ports qsfp1_tx4_n] ;# MGTYTXP3_230 GTYE4_CHANNEL_X1Y47 / GTYE4_COMMON_X1Y11
set_property -dict {LOC R41  } [get_ports qsfp1_mgt_refclk_0_p] ;# MGTREFCLK0P_230 
#set_property -dict {LOC W8  } [get_ports qsfp1_mgt_refclk_0_n] ;# MGTREFCLK0N_230 

# 161.1328125 MHz MGT reference clock
create_clock -period 6.206 -name qsfp1_mgt_refclk_0 [get_ports qsfp1_mgt_refclk_0_p]

# Interconnection-F port
#   CH      MGT_TRX
#   0       0
#   1       1
#   2       2
#   3       3
set_property -dict {LOC U50  } [get_ports qsfp2_rx1_p] ;# MGTYRXN0_129 GTYE4_CHANNEL_X0Y40 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC T1  } [get_ports qsfp2_rx1_n] ;# MGTYRXP0_129 GTYE4_CHANNEL_X0Y40 / GTYE4_COMMON_X0Y??
set_property -dict {LOC U45  } [get_ports qsfp2_tx1_p] ;# MGTYTXN0_129 GTYE4_CHANNEL_X0Y40 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC L4  } [get_ports qsfp2_tx1_n] ;# MGTYTXP0_129 GTYE4_CHANNEL_X0Y40 / GTYE4_COMMON_X0Y??
set_property -dict {LOC T48  } [get_ports qsfp2_rx2_p] ;# MGTYRXN1_129 GTYE4_CHANNEL_X0Y41 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC R3  } [get_ports qsfp2_rx2_n] ;# MGTYRXP1_129 GTYE4_CHANNEL_X0Y41 / GTYE4_COMMON_X0Y??
set_property -dict {LOC T43  } [get_ports qsfp2_tx2_p] ;# MGTYTXN1_129 GTYE4_CHANNEL_X0Y41 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC K6  } [get_ports qsfp2_tx2_n] ;# MGTYTXP1_129 GTYE4_CHANNEL_X0Y41 / GTYE4_COMMON_X0Y??
set_property -dict {LOC R50  } [get_ports qsfp2_rx3_p] ;# MGTYRXN2_129 GTYE4_CHANNEL_X0Y42 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC P1  } [get_ports qsfp2_rx3_n] ;# MGTYRXP2_129 GTYE4_CHANNEL_X0Y42 / GTYE4_COMMON_X0Y??
set_property -dict {LOC R45  } [get_ports qsfp2_tx3_p] ;# MGTYTXN2_129 GTYE4_CHANNEL_X0Y42 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC J4  } [get_ports qsfp2_tx3_n] ;# MGTYTXP2_129 GTYE4_CHANNEL_X0Y42 / GTYE4_COMMON_X0Y??
set_property -dict {LOC P48  } [get_ports qsfp2_rx4_p] ;# MGTYRXN3_129 GTYE4_CHANNEL_X0Y43 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC M1  } [get_ports qsfp2_rx4_n] ;# MGTYRXP3_129 GTYE4_CHANNEL_X0Y43 / GTYE4_COMMON_X0Y??
set_property -dict {LOC P43  } [get_ports qsfp2_tx4_p] ;# MGTYTXN3_129 GTYE4_CHANNEL_X0Y43 / GTYE4_COMMON_X0Y??
#set_property -dict {LOC H6  } [get_ports qsfp2_tx4_n] ;# MGTYTXP3_129 GTYE4_CHANNEL_X0Y43 / GTYE4_COMMON_X0Y??
set_property -dict {LOC U41  } [get_ports qsfp2_mgt_refclk_0_p] ;# MGTREFCLK0P_130
#set_property -dict {LOC R8  } [get_ports qsfp2_mgt_refclk_0_n] ;# MGTREFCLK0N_130

# 161.1328125 MHz MGT reference clock
create_clock -period 6.206 -name qsfp2_mgt_refclk_0 [get_ports qsfp2_mgt_refclk_0_p]

# PCIe Interface F
set_property -dict {LOC K4  } [get_ports {pcie_rx_p[0]}]  ;# MGTYRXP3_131 GTYE4_CHANNEL_X0Y48 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC AA3  } [get_ports {pcie_rx_n[0]}]  ;# MGTYRXN3_131 GTYE4_CHANNEL_X0Y48 / GTYE4_COMMON_X0Y12
set_property -dict {LOC K9  } [get_ports {pcie_tx_p[0]}]  ;# MGTYTXP3_131 GTYE4_CHANNEL_X0Y48 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC Y6   } [get_ports {pcie_tx_n[0]}]  ;# MGTYTXN3_131 GTYE4_CHANNEL_X0Y48 / GTYE4_COMMON_X0Y12
set_property -dict {LOC L2  } [get_ports {pcie_rx_p[1]}]  ;# MGTYRXP2_131 GTYE4_CHANNEL_X0Y49 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC AB1  } [get_ports {pcie_rx_n[1]}]  ;# MGTYRXN2_131 GTYE4_CHANNEL_X0Y49 / GTYE4_COMMON_X0Y12
set_property -dict {LOC L7  } [get_ports {pcie_tx_p[1]}]  ;# MGTYTXP2_131 GTYE4_CHANNEL_X0Y49 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC AB6  } [get_ports {pcie_tx_n[1]}]  ;# MGTYTXN2_131 GTYE4_CHANNEL_X0Y49 / GTYE4_COMMON_X0Y12
set_property -dict {LOC M4  } [get_ports {pcie_rx_p[2]}]  ;# MGTYRXP1_131 GTYE4_CHANNEL_X0Y50 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC AC3  } [get_ports {pcie_rx_n[2]}]  ;# MGTYRXN1_131 GTYE4_CHANNEL_X0Y50 / GTYE4_COMMON_X0Y12
set_property -dict {LOC M9  } [get_ports {pcie_tx_p[2]}]  ;# MGTYTXP1_131 GTYE4_CHANNEL_X0Y50 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC AD6  } [get_ports {pcie_tx_n[2]}]  ;# MGTYTXN1_131 GTYE4_CHANNEL_X0Y50 / GTYE4_COMMON_X0Y12
set_property -dict {LOC N2  } [get_ports {pcie_rx_p[3]}]  ;# MGTYRXP0_131 GTYE4_CHANNEL_X0Y51 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC AD1  } [get_ports {pcie_rx_n[3]}]  ;# MGTYRXN0_131 GTYE4_CHANNEL_X0Y51 / GTYE4_COMMON_X0Y12
set_property -dict {LOC N7  } [get_ports {pcie_tx_p[3]}]  ;# MGTYTXP0_131 GTYE4_CHANNEL_X0Y51 / GTYE4_COMMON_X0Y12
#set_property -dict {LOC AF6  } [get_ports {pcie_tx_n[3]}]  ;# MGTYTXN0_131 GTYE4_CHANNEL_X0Y51 / GTYE4_COMMON_X0Y12
set_property -dict {LOC P4  } [get_ports {pcie_rx_p[4]}]  ;# MGTYRXP3_132 GTYE4_CHANNEL_X0Y52 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AE3  } [get_ports {pcie_rx_n[4]}]  ;# MGTYRXN3_132 GTYE4_CHANNEL_X0Y52 / GTYE4_COMMON_X0Y13
set_property -dict {LOC P9  } [get_ports {pcie_tx_p[4]}]  ;# MGTYTXP3_132 GTYE4_CHANNEL_X0Y52 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AH6  } [get_ports {pcie_tx_n[4]}]  ;# MGTYTXN3_132 GTYE4_CHANNEL_X0Y52 / GTYE4_COMMON_X0Y13
set_property -dict {LOC R2  } [get_ports {pcie_rx_p[5]}]  ;# MGTYRXP2_132 GTYE4_CHANNEL_X0Y53 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AF1  } [get_ports {pcie_rx_n[5]}]  ;# MGTYRXN2_132 GTYE4_CHANNEL_X0Y53 / GTYE4_COMMON_X0Y13
set_property -dict {LOC R7  } [get_ports {pcie_tx_p[5]}]  ;# MGTYTXP2_132 GTYE4_CHANNEL_X0Y53 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AK6  } [get_ports {pcie_tx_n[5]}]  ;# MGTYTXN2_132 GTYE4_CHANNEL_X0Y53 / GTYE4_COMMON_X0Y13
set_property -dict {LOC T4  } [get_ports {pcie_rx_p[6]}]  ;# MGTYRXP1_132 GTYE4_CHANNEL_X0Y54 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AG3  } [get_ports {pcie_rx_n[6]}]  ;# MGTYRXN1_132 GTYE4_CHANNEL_X0Y54 / GTYE4_COMMON_X0Y13
set_property -dict {LOC T9  } [get_ports {pcie_tx_p[6]}]  ;# MGTYTXP1_132 GTYE4_CHANNEL_X0Y54 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AM6  } [get_ports {pcie_tx_n[6]}]  ;# MGTYTXN1_132 GTYE4_CHANNEL_X0Y54 / GTYE4_COMMON_X0Y13
set_property -dict {LOC U2  } [get_ports {pcie_rx_p[7]}]  ;# MGTYRXP0_132 GTYE4_CHANNEL_X0Y55 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AH1  } [get_ports {pcie_rx_n[7]}]  ;# MGTYRXN0_132 GTYE4_CHANNEL_X0Y55 / GTYE4_COMMON_X0Y13
set_property -dict {LOC U7  } [get_ports {pcie_tx_p[7]}]  ;# MGTYTXP0_132 GTYE4_CHANNEL_X0Y55 / GTYE4_COMMON_X0Y13
#set_property -dict {LOC AN4  } [get_ports {pcie_tx_n[7]}]  ;# MGTYTXN0_132 GTYE4_CHANNEL_X0Y55 / GTYE4_COMMON_X0Y13
set_property -dict {LOC U11  } [get_ports pcie_refclk_0_p] ;# MGTREFCLK0P_132
#set_property -dict {LOC AC8  } [get_ports pcie_refclk_0_n] ;# MGTREFCLK0N_132
#set_property -dict {LOC T13  } [get_ports pcie_refclk_1_p] ;# MGTREFCLK1P_132
#set_property -dict {LOC AL8  } [get_ports pcie_refclk_1_n] ;# MGTREFCLK1N_132
set_property -dict {LOC P23 IOSTANDARD LVCMOS18 PULLUP true} [get_ports pcie_reset_n]

# 100 MHz MGT reference clock
create_clock -period 10 -name pcie_mgt_refclk_0 [get_ports pcie_refclk_0_p]
#create_clock -period 10 -name pcie_mgt_refclk_1 [get_ports pcie_refclk_1_p]
