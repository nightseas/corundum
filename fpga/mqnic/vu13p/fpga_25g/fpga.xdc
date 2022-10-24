# XDC constraints for the VU13P EVB
# part: xcvu13p-fhgb2104-2-e

# General configuration
set_property CFGBVS GND                                [current_design]
set_property CONFIG_VOLTAGE 1.8                        [current_design]
set_property BITSTREAM.GENERAL.COMPRESS true           [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 102.0         [current_design]
set_property BITSTREAM.CONFIG.SPI_32BIT_ADDR YES       [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4           [current_design]
set_property BITSTREAM.CONFIG.SPI_FALL_EDGE YES        [current_design]

# System clocks
# 400 MHz
#set_property -dict {LOC AY22 IOSTANDARD DIFF_SSTL12} [get_ports clk_400mhz_0_p]
#set_property -dict {LOC BA22 IOSTANDARD DIFF_SSTL12} [get_ports clk_400mhz_0_n]
#create_clock -period 2.5 -name clk_400mhz_0 [get_ports clk_400mhz_0_p]

#set_property -dict {LOC AY22 IOSTANDARD DIFF_SSTL12} [get_ports clk_400mhz_1_p]
#set_property -dict {LOC BA22 IOSTANDARD DIFF_SSTL12} [get_ports clk_400mhz_1_n]
#create_clock -period 2.5 -name clk_400mhz_1 [get_ports clk_400mhz_1_p]

#set_property -dict {LOC AY22 IOSTANDARD DIFF_SSTL12} [get_ports clk_400mhz_2_p]
#set_property -dict {LOC BA22 IOSTANDARD DIFF_SSTL12} [get_ports clk_400mhz_2_n]
#create_clock -period 2.5 -name clk_400mhz_2 [get_ports clk_400mhz_2_p]

#set_property -dict {LOC AY22 IOSTANDARD DIFF_SSTL12} [get_ports clk_400mhz_3_p]
#set_property -dict {LOC BA22 IOSTANDARD DIFF_SSTL12} [get_ports clk_400mhz_3_n]
#create_clock -period 2.5 -name clk_400mhz_3 [get_ports clk_400mhz_3_p]

#set_property -dict {LOC AY22 IOSTANDARD DIFF_SSTL12} [get_ports clk_400mhz_4_p]
#set_property -dict {LOC BA22 IOSTANDARD DIFF_SSTL12} [get_ports clk_400mhz_4_n]
#create_clock -period 2.5 -name clk_400mhz_4 [get_ports clk_400mhz_4_p]

# 100 MHz
set_property -dict {LOC AY23  IOSTANDARD LVDS} [get_ports clk_100mhz_p]
set_property -dict {LOC BA23  IOSTANDARD LVDS} [get_ports clk_100mhz_n]
create_clock -period 10 -name clk_100mhz [get_ports clk_100mhz_p]

# LEDs
set_property -dict {LOC BA20 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports {led[0]}]
set_property -dict {LOC BB20 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports {led[1]}]
set_property -dict {LOC BB21 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports {led[2]}]
set_property -dict {LOC BC21 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports {led[3]}]
set_property -dict {LOC BB22 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports {led[4]}]
set_property -dict {LOC BC22 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports {led[5]}]
set_property -dict {LOC BA24 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports {led[6]}]
set_property -dict {LOC BB24 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports {led[7]}]

set_false_path -to [get_ports {led[*]}]
set_output_delay 0 [get_ports {led[*]}]

# QSFP28 Interfaces
set_property -dict {LOC AA4  } [get_ports qsfp1_rx1_p] ;# MGTYRXP0_229 GTYE4_CHANNEL_X1Y48 / GTYE4_COMMON_X1Y12
set_property -dict {LOC AA3  } [get_ports qsfp1_rx1_n] ;# MGTYRXN0_229 GTYE4_CHANNEL_X1Y48 / GTYE4_COMMON_X1Y12
set_property -dict {LOC AA9  } [get_ports qsfp1_tx1_p] ;# MGTYTXP0_229 GTYE4_CHANNEL_X1Y48 / GTYE4_COMMON_X1Y12
set_property -dict {LOC AA8  } [get_ports qsfp1_tx1_n] ;# MGTYTXN0_229 GTYE4_CHANNEL_X1Y48 / GTYE4_COMMON_X1Y12
set_property -dict {LOC Y2   } [get_ports qsfp1_rx2_p] ;# MGTYRXP1_229 GTYE4_CHANNEL_X1Y49 / GTYE4_COMMON_X1Y12
set_property -dict {LOC Y1   } [get_ports qsfp1_rx2_n] ;# MGTYRXN1_229 GTYE4_CHANNEL_X1Y49 / GTYE4_COMMON_X1Y12
set_property -dict {LOC Y7   } [get_ports qsfp1_tx2_p] ;# MGTYTXP1_229 GTYE4_CHANNEL_X1Y49 / GTYE4_COMMON_X1Y12
set_property -dict {LOC Y6   } [get_ports qsfp1_tx2_n] ;# MGTYTXN1_229 GTYE4_CHANNEL_X1Y49 / GTYE4_COMMON_X1Y12
set_property -dict {LOC W4   } [get_ports qsfp1_rx3_p] ;# MGTYRXP2_229 GTYE4_CHANNEL_X1Y50 / GTYE4_COMMON_X1Y12
set_property -dict {LOC W3   } [get_ports qsfp1_rx3_n] ;# MGTYRXN2_229 GTYE4_CHANNEL_X1Y50 / GTYE4_COMMON_X1Y12
set_property -dict {LOC W9   } [get_ports qsfp1_tx3_p] ;# MGTYTXP2_229 GTYE4_CHANNEL_X1Y50 / GTYE4_COMMON_X1Y12
set_property -dict {LOC W8   } [get_ports qsfp1_tx3_n] ;# MGTYTXN2_229 GTYE4_CHANNEL_X1Y50 / GTYE4_COMMON_X1Y12
set_property -dict {LOC V2   } [get_ports qsfp1_rx4_p] ;# MGTYRXP3_229 GTYE4_CHANNEL_X1Y51 / GTYE4_COMMON_X1Y12
set_property -dict {LOC V1   } [get_ports qsfp1_rx4_n] ;# MGTYRXN3_229 GTYE4_CHANNEL_X1Y51 / GTYE4_COMMON_X1Y12
set_property -dict {LOC V7   } [get_ports qsfp1_tx4_p] ;# MGTYTXP3_229 GTYE4_CHANNEL_X1Y51 / GTYE4_COMMON_X1Y12
set_property -dict {LOC V6   } [get_ports qsfp1_tx4_n] ;# MGTYTXN3_229 GTYE4_CHANNEL_X1Y51 / GTYE4_COMMON_X1Y12
set_property -dict {LOC Y11  } [get_ports qsfp1_mgt_refclk_0_p] ;# MGTREFCLK0P_229
set_property -dict {LOC Y10  } [get_ports qsfp1_mgt_refclk_0_n] ;# MGTREFCLK0N_229
# set_property -dict {LOC AM21 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports qsfp1_modsell]
# set_property -dict {LOC BA22 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports qsfp1_resetl]
# set_property -dict {LOC AL21 IOSTANDARD LVCMOS18 PULLUP true} [get_ports qsfp1_modprsl]
# set_property -dict {LOC AP21 IOSTANDARD LVCMOS18 PULLUP true} [get_ports qsfp1_intl]
# set_property -dict {LOC AN21 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports qsfp1_lpmode]

# 161.132 MHz MGT reference clock
create_clock -period 6.206 -name qsfp1_mgt_refclk_0 [get_ports qsfp1_mgt_refclk_0_p]

# set_false_path -to [get_ports {qsfp1_modsell qsfp1_resetl qsfp1_lpmode}]
# set_output_delay 0 [get_ports {qsfp1_modsell qsfp1_resetl qsfp1_lpmode}]
# set_false_path -from [get_ports {qsfp1_modprsl qsfp1_intl}]
# set_input_delay 0 [get_ports {qsfp1_modprsl qsfp1_intl}]

set_property -dict {LOC E4   } [get_ports qsfp2_rx1_p] ;# MGTYRXP0_233 GTYE4_CHANNEL_X1Y52 / GTYE4_COMMON_X1Y13
set_property -dict {LOC E3   } [get_ports qsfp2_rx1_n] ;# MGTYRXN0_233 GTYE4_CHANNEL_X1Y52 / GTYE4_COMMON_X1Y13
set_property -dict {LOC E9   } [get_ports qsfp2_tx1_p] ;# MGTYTXP0_233 GTYE4_CHANNEL_X1Y52 / GTYE4_COMMON_X1Y13
set_property -dict {LOC E8   } [get_ports qsfp2_tx1_n] ;# MGTYTXN0_233 GTYE4_CHANNEL_X1Y52 / GTYE4_COMMON_X1Y13
set_property -dict {LOC D2   } [get_ports qsfp2_rx2_p] ;# MGTYRXP1_233 GTYE4_CHANNEL_X1Y53 / GTYE4_COMMON_X1Y13
set_property -dict {LOC D1   } [get_ports qsfp2_rx2_n] ;# MGTYRXN1_233 GTYE4_CHANNEL_X1Y53 / GTYE4_COMMON_X1Y13
set_property -dict {LOC D7   } [get_ports qsfp2_tx2_p] ;# MGTYTXP1_233 GTYE4_CHANNEL_X1Y53 / GTYE4_COMMON_X1Y13
set_property -dict {LOC D6   } [get_ports qsfp2_tx2_n] ;# MGTYTXN1_233 GTYE4_CHANNEL_X1Y53 / GTYE4_COMMON_X1Y13
set_property -dict {LOC C4   } [get_ports qsfp2_rx3_p] ;# MGTYRXP2_233 GTYE4_CHANNEL_X1Y54 / GTYE4_COMMON_X1Y13
set_property -dict {LOC C3   } [get_ports qsfp2_rx3_n] ;# MGTYRXN2_233 GTYE4_CHANNEL_X1Y54 / GTYE4_COMMON_X1Y13
set_property -dict {LOC C9   } [get_ports qsfp2_tx3_p] ;# MGTYTXP2_233 GTYE4_CHANNEL_X1Y54 / GTYE4_COMMON_X1Y13
set_property -dict {LOC C8   } [get_ports qsfp2_tx3_n] ;# MGTYTXN2_233 GTYE4_CHANNEL_X1Y54 / GTYE4_COMMON_X1Y13
set_property -dict {LOC A5   } [get_ports qsfp2_rx4_p] ;# MGTYRXP3_233 GTYE4_CHANNEL_X1Y55 / GTYE4_COMMON_X1Y13
set_property -dict {LOC A4   } [get_ports qsfp2_rx4_n] ;# MGTYRXN3_233 GTYE4_CHANNEL_X1Y55 / GTYE4_COMMON_X1Y13
set_property -dict {LOC A9   } [get_ports qsfp2_tx4_p] ;# MGTYTXP3_233 GTYE4_CHANNEL_X1Y55 / GTYE4_COMMON_X1Y13
set_property -dict {LOC A8   } [get_ports qsfp2_tx4_n] ;# MGTYTXN3_233 GTYE4_CHANNEL_X1Y55 / GTYE4_COMMON_X1Y13
set_property -dict {LOC D11  } [get_ports qsfp2_mgt_refclk_0_p] ;# MGTREFCLK0P_233
set_property -dict {LOC D10  } [get_ports qsfp2_mgt_refclk_0_n] ;# MGTREFCLK0N_233
# set_property -dict {LOC AN23 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports qsfp2_modsell]
# set_property -dict {LOC AY22 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports qsfp2_resetl]
# set_property -dict {LOC AN24 IOSTANDARD LVCMOS18 PULLUP true} [get_ports qsfp2_modprsl]
# set_property -dict {LOC AT21 IOSTANDARD LVCMOS18 PULLUP true} [get_ports qsfp2_intl]
# set_property -dict {LOC AT24 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports qsfp2_lpmode]

# 161.132 MHz MGT reference clock
create_clock -period 6.206 -name qsfp2_mgt_refclk_0 [get_ports qsfp2_mgt_refclk_0_p]

# set_false_path -to [get_ports {qsfp2_modsell qsfp2_resetl qsfp2_lpmode}]
# set_output_delay 0 [get_ports {qsfp2_modsell qsfp2_resetl qsfp2_lpmode}]
# set_false_path -from [get_ports {qsfp2_modprsl qsfp2_intl}]
# set_input_delay 0 [get_ports {qsfp2_modprsl qsfp2_intl}]

# I2C interface
# set_property -dict {LOC AM24 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports i2c_scl]
# set_property -dict {LOC AL24 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports i2c_sda]

# set_false_path -to [get_ports {i2c_sda i2c_scl}]
# set_output_delay 0 [get_ports {i2c_sda i2c_scl}]
# set_false_path -from [get_ports {i2c_sda i2c_scl}]
# set_input_delay 0 [get_ports {i2c_sda i2c_scl}]

# PCIe Interface
set_property -dict {LOC AF2  } [get_ports {pcie_rx_p[0]}]  ;# MGTYRXP3_227 GTYE4_CHANNEL_X1Y35 / GTYE4_COMMON_X1Y8
# set_property -dict {LOC AA3  } [get_ports {pcie_rx_n[0]}]  ;# MGTYRXN3_227 GTYE4_CHANNEL_X1Y35 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AF7  } [get_ports {pcie_tx_p[0]}]  ;# MGTYTXP3_227 GTYE4_CHANNEL_X1Y35 / GTYE4_COMMON_X1Y8
# set_property -dict {LOC Y6   } [get_ports {pcie_tx_n[0]}]  ;# MGTYTXN3_227 GTYE4_CHANNEL_X1Y35 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AG4  } [get_ports {pcie_rx_p[1]}]  ;# MGTYRXP2_227 GTYE4_CHANNEL_X1Y34 / GTYE4_COMMON_X1Y8
# set_property -dict {LOC AB1  } [get_ports {pcie_rx_n[1]}]  ;# MGTYRXN2_227 GTYE4_CHANNEL_X1Y34 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AG9  } [get_ports {pcie_tx_p[1]}]  ;# MGTYTXP2_227 GTYE4_CHANNEL_X1Y34 / GTYE4_COMMON_X1Y8
# set_property -dict {LOC AB6  } [get_ports {pcie_tx_n[1]}]  ;# MGTYTXN2_227 GTYE4_CHANNEL_X1Y34 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AH2  } [get_ports {pcie_rx_p[2]}]  ;# MGTYRXP1_227 GTYE4_CHANNEL_X1Y33 / GTYE4_COMMON_X1Y8
# set_property -dict {LOC AC3  } [get_ports {pcie_rx_n[2]}]  ;# MGTYRXN1_227 GTYE4_CHANNEL_X1Y33 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AH7  } [get_ports {pcie_tx_p[2]}]  ;# MGTYTXP1_227 GTYE4_CHANNEL_X1Y33 / GTYE4_COMMON_X1Y8
# set_property -dict {LOC AD6  } [get_ports {pcie_tx_n[2]}]  ;# MGTYTXN1_227 GTYE4_CHANNEL_X1Y33 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AJ4  } [get_ports {pcie_rx_p[3]}]  ;# MGTYRXP0_227 GTYE4_CHANNEL_X1Y32 / GTYE4_COMMON_X1Y8
# set_property -dict {LOC AD1  } [get_ports {pcie_rx_n[3]}]  ;# MGTYRXN0_227 GTYE4_CHANNEL_X1Y32 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AJ9  } [get_ports {pcie_tx_p[3]}]  ;# MGTYTXP0_227 GTYE4_CHANNEL_X1Y32 / GTYE4_COMMON_X1Y8
# set_property -dict {LOC AF6  } [get_ports {pcie_tx_n[3]}]  ;# MGTYTXN0_227 GTYE4_CHANNEL_X1Y32 / GTYE4_COMMON_X1Y8
set_property -dict {LOC AK2  } [get_ports {pcie_rx_p[4]}]  ;# MGTYRXP3_226 GTYE4_CHANNEL_X1Y31 / GTYE4_COMMON_X1Y7
# set_property -dict {LOC AE3  } [get_ports {pcie_rx_n[4]}]  ;# MGTYRXN3_226 GTYE4_CHANNEL_X1Y31 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AK7  } [get_ports {pcie_tx_p[4]}]  ;# MGTYTXP3_226 GTYE4_CHANNEL_X1Y31 / GTYE4_COMMON_X1Y7
# set_property -dict {LOC AH6  } [get_ports {pcie_tx_n[4]}]  ;# MGTYTXN3_226 GTYE4_CHANNEL_X1Y31 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AL4  } [get_ports {pcie_rx_p[5]}]  ;# MGTYRXP2_226 GTYE4_CHANNEL_X1Y30 / GTYE4_COMMON_X1Y7
# set_property -dict {LOC AF1  } [get_ports {pcie_rx_n[5]}]  ;# MGTYRXN2_226 GTYE4_CHANNEL_X1Y30 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AL9  } [get_ports {pcie_tx_p[5]}]  ;# MGTYTXP2_226 GTYE4_CHANNEL_X1Y30 / GTYE4_COMMON_X1Y7
# set_property -dict {LOC AK6  } [get_ports {pcie_tx_n[5]}]  ;# MGTYTXN2_226 GTYE4_CHANNEL_X1Y30 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AM2  } [get_ports {pcie_rx_p[6]}]  ;# MGTYRXP1_226 GTYE4_CHANNEL_X1Y29 / GTYE4_COMMON_X1Y7
# set_property -dict {LOC AG3  } [get_ports {pcie_rx_n[6]}]  ;# MGTYRXN1_226 GTYE4_CHANNEL_X1Y29 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AM7  } [get_ports {pcie_tx_p[6]}]  ;# MGTYTXP1_226 GTYE4_CHANNEL_X1Y29 / GTYE4_COMMON_X1Y7
# set_property -dict {LOC AM6  } [get_ports {pcie_tx_n[6]}]  ;# MGTYTXN1_226 GTYE4_CHANNEL_X1Y29 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AN4  } [get_ports {pcie_rx_p[7]}]  ;# MGTYRXP0_226 GTYE4_CHANNEL_X1Y28 / GTYE4_COMMON_X1Y7
# set_property -dict {LOC AH1  } [get_ports {pcie_rx_n[7]}]  ;# MGTYRXN0_226 GTYE4_CHANNEL_X1Y28 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AN9  } [get_ports {pcie_tx_p[7]}]  ;# MGTYTXP0_226 GTYE4_CHANNEL_X1Y28 / GTYE4_COMMON_X1Y7
# set_property -dict {LOC AN4  } [get_ports {pcie_tx_n[7]}]  ;# MGTYTXN0_226 GTYE4_CHANNEL_X1Y28 / GTYE4_COMMON_X1Y7
set_property -dict {LOC AP2  } [get_ports {pcie_rx_p[8]}]  ;# MGTYRXP3_225 GTYE4_CHANNEL_X1Y27 / GTYE4_COMMON_X1Y6
# set_property -dict {LOC AJ3  } [get_ports {pcie_rx_n[8]}]  ;# MGTYRXN3_225 GTYE4_CHANNEL_X1Y27 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AP7  } [get_ports {pcie_tx_p[8]}]  ;# MGTYTXP3_225 GTYE4_CHANNEL_X1Y27 / GTYE4_COMMON_X1Y6
# set_property -dict {LOC AP6  } [get_ports {pcie_tx_n[8]}]  ;# MGTYTXN3_225 GTYE4_CHANNEL_X1Y27 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AR4  } [get_ports {pcie_rx_p[9]}]  ;# MGTYRXP2_225 GTYE4_CHANNEL_X1Y26 / GTYE4_COMMON_X1Y6
# set_property -dict {LOC AK1  } [get_ports {pcie_rx_n[9]}]  ;# MGTYRXN2_225 GTYE4_CHANNEL_X1Y26 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AR9  } [get_ports {pcie_tx_p[9]}]  ;# MGTYTXP2_225 GTYE4_CHANNEL_X1Y26 / GTYE4_COMMON_X1Y6
# set_property -dict {LOC AR4  } [get_ports {pcie_tx_n[9]}]  ;# MGTYTXN2_225 GTYE4_CHANNEL_X1Y26 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AT2  } [get_ports {pcie_rx_p[10]}] ;# MGTYRXP1_225 GTYE4_CHANNEL_X1Y25 / GTYE4_COMMON_X1Y6
# set_property -dict {LOC AM1  } [get_ports {pcie_rx_n[10]}] ;# MGTYRXN1_225 GTYE4_CHANNEL_X1Y25 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AT7  } [get_ports {pcie_tx_p[10]}] ;# MGTYTXP1_225 GTYE4_CHANNEL_X1Y25 / GTYE4_COMMON_X1Y6
# set_property -dict {LOC AT6  } [get_ports {pcie_tx_n[10]}] ;# MGTYTXN1_225 GTYE4_CHANNEL_X1Y25 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AU4  } [get_ports {pcie_rx_p[11]}] ;# MGTYRXP0_225 GTYE4_CHANNEL_X1Y24 / GTYE4_COMMON_X1Y6
# set_property -dict {LOC AP1  } [get_ports {pcie_rx_n[11]}] ;# MGTYRXN0_225 GTYE4_CHANNEL_X1Y24 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AU9  } [get_ports {pcie_tx_p[11]}] ;# MGTYTXP0_225 GTYE4_CHANNEL_X1Y24 / GTYE4_COMMON_X1Y6
# set_property -dict {LOC AU4  } [get_ports {pcie_tx_n[11]}] ;# MGTYTXN0_225 GTYE4_CHANNEL_X1Y24 / GTYE4_COMMON_X1Y6
set_property -dict {LOC AV2  } [get_ports {pcie_rx_p[12]}] ;# MGTYRXP3_224 GTYE4_CHANNEL_X1Y23 / GTYE4_COMMON_X1Y5
# set_property -dict {LOC AT1  } [get_ports {pcie_rx_n[12]}] ;# MGTYRXN3_224 GTYE4_CHANNEL_X1Y23 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AV7  } [get_ports {pcie_tx_p[12]}] ;# MGTYTXP3_224 GTYE4_CHANNEL_X1Y23 / GTYE4_COMMON_X1Y5
# set_property -dict {LOC AW4  } [get_ports {pcie_tx_n[12]}] ;# MGTYTXN3_224 GTYE4_CHANNEL_X1Y23 / GTYE4_COMMON_X1Y5
set_property -dict {LOC AW4  } [get_ports {pcie_rx_p[13]}] ;# MGTYRXP2_224 GTYE4_CHANNEL_X1Y22 / GTYE4_COMMON_X1Y5
# set_property -dict {LOC AV1  } [get_ports {pcie_rx_n[13]}] ;# MGTYRXN2_224 GTYE4_CHANNEL_X1Y22 / GTYE4_COMMON_X1Y5
set_property -dict {LOC BB5  } [get_ports {pcie_tx_p[13]}] ;# MGTYTXP2_224 GTYE4_CHANNEL_X1Y22 / GTYE4_COMMON_X1Y5
# set_property -dict {LOC BA4  } [get_ports {pcie_tx_n[13]}] ;# MGTYTXN2_224 GTYE4_CHANNEL_X1Y22 / GTYE4_COMMON_X1Y5
set_property -dict {LOC BA2  } [get_ports {pcie_rx_p[14]}] ;# MGTYRXP1_224 GTYE4_CHANNEL_X1Y21 / GTYE4_COMMON_X1Y5
# set_property -dict {LOC AY1  } [get_ports {pcie_rx_n[14]}] ;# MGTYRXN1_224 GTYE4_CHANNEL_X1Y21 / GTYE4_COMMON_X1Y5
set_property -dict {LOC BD5  } [get_ports {pcie_tx_p[14]}] ;# MGTYTXP1_224 GTYE4_CHANNEL_X1Y21 / GTYE4_COMMON_X1Y5
# set_property -dict {LOC BC4  } [get_ports {pcie_tx_n[14]}] ;# MGTYTXN1_224 GTYE4_CHANNEL_X1Y21 / GTYE4_COMMON_X1Y5
set_property -dict {LOC BC2  } [get_ports {pcie_rx_p[15]}] ;# MGTYRXP0_224 GTYE4_CHANNEL_X1Y20 / GTYE4_COMMON_X1Y5
# set_property -dict {LOC BB1  } [get_ports {pcie_rx_n[15]}] ;# MGTYRXN0_224 GTYE4_CHANNEL_X1Y20 / GTYE4_COMMON_X1Y5
set_property -dict {LOC BF5  } [get_ports {pcie_tx_p[15]}] ;# MGTYTXP0_224 GTYE4_CHANNEL_X1Y20 / GTYE4_COMMON_X1Y5
# set_property -dict {LOC BE4  } [get_ports {pcie_tx_n[15]}] ;# MGTYTXN0_224 GTYE4_CHANNEL_X1Y20 / GTYE4_COMMON_X1Y5
# set_property -dict {LOC AV11  } [get_ports pcie_refclk_1_p] ;# MGTREFCLK0P_224
# set_property -dict {LOC AV10  } [get_ports pcie_refclk_1_n] ;# MGTREFCLK0N_224
set_property -dict {LOC AK11 } [get_ports pcie_refclk_2_p] ;# MGTREFCLK0P_226
set_property -dict {LOC AK10 } [get_ports pcie_refclk_2_n] ;# MGTREFCLK0N_226
set_property -dict {LOC AR26 IOSTANDARD LVCMOS18 PULLUP true} [get_ports pcie_reset_n]

# 100 MHz MGT reference clock
# create_clock -period 10 -name pcie_mgt_refclk_1 [get_ports pcie_refclk_1_p]
create_clock -period 10 -name pcie_mgt_refclk_2 [get_ports pcie_refclk_2_p]

set_false_path -from [get_ports {pcie_reset_n}]
set_input_delay 0 [get_ports {pcie_reset_n}]
