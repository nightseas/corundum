# XDC constraints for the GPTU
# part: xcku5p-ffvb676-2-i

# General configuration
set_property CFGBVS GND                                [current_design]
set_property CONFIG_VOLTAGE 1.8                        [current_design]
set_property BITSTREAM.GENERAL.COMPRESS true           [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 102.0         [current_design]
set_property BITSTREAM.CONFIG.SPI_32BIT_ADDR YES       [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 8           [current_design]
set_property BITSTREAM.CONFIG.UNUSEDPIN {Pullnone}     [current_design]
set_property BITSTREAM.CONFIG.SPI_FALL_EDGE YES        [current_design]

# 10 MHz OXCO
#set_property -dict {LOC D14  IOSTANDARD LVCMOS33} [get_ports clk_10mhz]
#create_clock -period 100 -name clk_100mhz [get_ports clk_10mhz]

# LEDs
set_property -dict {LOC J12 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {sfp_1_led}]
set_property -dict {LOC J13 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {sfp_2_led}]
set_property -dict {LOC J14 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {sma_led}]

set_false_path -to [get_ports {sfp_1_led sfp_2_led sma_led}]
set_output_delay 0 [get_ports {sfp_1_led sfp_2_led sma_led}]

set_property -dict {LOC F9  IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {led_sta[0]}]
set_property -dict {LOC F10 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {led_sta[1]}]
set_property -dict {LOC G9  IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {led_sta[2]}]
set_property -dict {LOC G10 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {led_sta[3]}]

set_false_path -to [get_ports {led_sta[*]}]
set_output_delay 0 [get_ports {led_sta[*]}]

# SMA
set_property -dict {LOC A14 IOSTANDARD LVCMOS33} [get_ports sma_in]
set_property -dict {LOC A13 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 8} [get_ports sma_out]

set_false_path -to [get_ports {sma_out}]
set_output_delay 0 [get_ports {sma_out}]
set_false_path -from [get_ports {sma_in}]
set_input_delay 0 [get_ports {sma_in}]

set_property -dict {LOC E15 IOSTANDARD LVCMOS18 SLEW SLOW DRIVE 8} [get_ports {tu_rstn}]

# SFP28 Interfaces
set_property -dict {LOC D2  } [get_ports sfp_1_rx_p] ;# MGTYRXP0_227 GTYE4_CHANNEL_X0Y12 / GTYE4_COMMON_X0Y3
set_property -dict {LOC D1  } [get_ports sfp_1_rx_n] ;# MGTYRXN0_227 GTYE4_CHANNEL_X0Y12 / GTYE4_COMMON_X0Y3
set_property -dict {LOC A4  } [get_ports sfp_2_rx_p] ;# MGTYRXP3_227 GTYE4_CHANNEL_X0Y15 / GTYE4_COMMON_X0Y3
set_property -dict {LOC A3  } [get_ports sfp_2_rx_n] ;# MGTYRXN3_227 GTYE4_CHANNEL_X0Y15 / GTYE4_COMMON_X0Y3
set_property -dict {LOC F7  } [get_ports sfp_1_tx_p] ;# MGTYTXP0_227 GTYE4_CHANNEL_X0Y12 / GTYE4_COMMON_X0Y3
set_property -dict {LOC F6  } [get_ports sfp_1_tx_n] ;# MGTYTXN0_227 GTYE4_CHANNEL_X0Y12 / GTYE4_COMMON_X0Y3
set_property -dict {LOC B7  } [get_ports sfp_2_tx_p] ;# MGTYTXP3_227 GTYE4_CHANNEL_X0Y15 / GTYE4_COMMON_X0Y3
set_property -dict {LOC B6  } [get_ports sfp_2_tx_n] ;# MGTYTXN3_227 GTYE4_CHANNEL_X0Y15 / GTYE4_COMMON_X0Y3
set_property -dict {LOC K7  } [get_ports sfp_mgt_refclk_p] ;# MGTREFCLK0P_227 from X2
set_property -dict {LOC K6  } [get_ports sfp_mgt_refclk_n] ;# MGTREFCLK0N_227 from X2
set_property -dict {LOC J15  IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8} [get_ports sfp_1_pwr_en]
set_property -dict {LOC C12  IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8} [get_ports sfp_2_pwr_en]
set_property -dict {LOC E13  IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8} [get_ports sfp_1_tx_disable]
set_property -dict {LOC E12  IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8} [get_ports sfp_2_tx_disable]
set_property -dict {LOC F12  IOSTANDARD LVCMOS33 PULLUP true} [get_ports sfp_1_npres]
set_property -dict {LOC F14  IOSTANDARD LVCMOS33 PULLUP true} [get_ports sfp_2_npres]
set_property -dict {LOC C14  IOSTANDARD LVCMOS33 PULLUP true} [get_ports sfp_1_los]
set_property -dict {LOC C13  IOSTANDARD LVCMOS33 PULLUP true} [get_ports sfp_2_los]
set_property -dict {LOC G14  IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8} [get_ports sfp_1_rs]
set_property -dict {LOC H14  IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8} [get_ports sfp_2_rs]
set_property -dict {LOC A10  IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8 PULLUP true} [get_ports sfp_1_i2c_scl]
set_property -dict {LOC C11  IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8 PULLUP true} [get_ports sfp_1_i2c_sda]
set_property -dict {LOC B10  IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8 PULLUP true} [get_ports sfp_2_i2c_scl]
set_property -dict {LOC B11  IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8 PULLUP true} [get_ports sfp_2_i2c_sda]

# 161.1328125 MHz MGT reference clock
create_clock -period 6.206 -name sfp_mgt_refclk [get_ports sfp_mgt_refclk_p]

set_false_path -to [get_ports {sfp_1_tx_disable sfp_2_tx_disable sfp_1_rs sfp_2_rs}]
set_output_delay 0 [get_ports {sfp_1_tx_disable sfp_2_tx_disable sfp_1_rs sfp_2_rs}]
set_false_path -from [get_ports {sfp_1_npres sfp_2_npres sfp_1_los sfp_2_los}]
set_input_delay 0 [get_ports {sfp_1_npres sfp_2_npres sfp_1_los sfp_2_los}]

set_false_path -to [get_ports {sfp_1_i2c_sda sfp_2_i2c_sda sfp_1_i2c_scl sfp_2_i2c_scl}]
set_output_delay 0 [get_ports {sfp_1_i2c_sda sfp_2_i2c_sda sfp_1_i2c_scl sfp_2_i2c_scl}]
set_false_path -from [get_ports {sfp_1_i2c_sda sfp_2_i2c_sda sfp_1_i2c_scl sfp_2_i2c_scl}]
set_input_delay 0 [get_ports {sfp_1_i2c_sda sfp_2_i2c_sda sfp_1_i2c_scl sfp_2_i2c_scl}]

# I2C interface
set_property -dict {LOC B9 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8 PULLUP true} [get_ports eeprom_i2c_scl]
set_property -dict {LOC A9 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8 PULLUP true} [get_ports eeprom_i2c_sda]

set_false_path -to [get_ports {eeprom_i2c_sda eeprom_i2c_scl}]
set_output_delay 0 [get_ports {eeprom_i2c_sda eeprom_i2c_scl}]
set_false_path -from [get_ports {eeprom_i2c_sda eeprom_i2c_scl}]
set_input_delay 0 [get_ports {eeprom_i2c_sda eeprom_i2c_scl}]

set_property -dict {LOC AE13 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8 PULLUP true} [get_ports tu_i2c_scl]
set_property -dict {LOC AF13 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8 PULLUP true} [get_ports tu_i2c_sda]

set_property -dict {LOC AC14 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8 PULLUP true} [get_ports ocxo_i2c_scl]
set_property -dict {LOC AC13 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8 PULLUP true} [get_ports ocxo_i2c_sda]

set_property -dict {LOC J11  IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8 PULLUP true} [get_ports gnss_i2c_scl]
set_property -dict {LOC J10  IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 8 PULLUP true} [get_ports gnss_i2c_sda]

set_false_path -to [get_ports {tu_i2c_sda tu_i2c_scl gnss_i2c_sda gnss_i2c_scl ocxo_i2c_sda ocxo_i2c_scl}]
set_output_delay 0 [get_ports {tu_i2c_sda tu_i2c_scl gnss_i2c_sda gnss_i2c_scl ocxo_i2c_sda ocxo_i2c_scl}]
set_false_path -from [get_ports {tu_i2c_sda tu_i2c_scl gnss_i2c_sda gnss_i2c_scl ocxo_i2c_sda ocxo_i2c_scl}]
set_input_delay 0 [get_ports {tu_i2c_sda tu_i2c_scl gnss_i2c_sda gnss_i2c_scl ocxo_i2c_sda ocxo_i2c_scl}]


# PCIe Interface
set_property -dict {LOC P2  } [get_ports {pcie_rx_p[0]}] ;# MGTYRXP3_225 GTYE4_CHANNEL_X0Y7 / GTYE4_COMMON_X0Y1
set_property -dict {LOC P1  } [get_ports {pcie_rx_n[0]}] ;# MGTYRXN3_225 GTYE4_CHANNEL_X0Y7 / GTYE4_COMMON_X0Y1
set_property -dict {LOC R5  } [get_ports {pcie_tx_p[0]}] ;# MGTYTXP3_225 GTYE4_CHANNEL_X0Y7 / GTYE4_COMMON_X0Y1
set_property -dict {LOC R4  } [get_ports {pcie_tx_n[0]}] ;# MGTYTXN3_225 GTYE4_CHANNEL_X0Y7 / GTYE4_COMMON_X0Y1
set_property -dict {LOC T2  } [get_ports {pcie_rx_p[1]}] ;# MGTYRXP2_225 GTYE4_CHANNEL_X0Y6 / GTYE4_COMMON_X0Y1
set_property -dict {LOC T1  } [get_ports {pcie_rx_n[1]}] ;# MGTYRXN2_225 GTYE4_CHANNEL_X0Y6 / GTYE4_COMMON_X0Y1
set_property -dict {LOC U5  } [get_ports {pcie_tx_p[1]}] ;# MGTYTXP2_225 GTYE4_CHANNEL_X0Y6 / GTYE4_COMMON_X0Y1
set_property -dict {LOC U4  } [get_ports {pcie_tx_n[1]}] ;# MGTYTXN2_225 GTYE4_CHANNEL_X0Y6 / GTYE4_COMMON_X0Y1
set_property -dict {LOC V2  } [get_ports {pcie_rx_p[2]}] ;# MGTYRXP1_225 GTYE4_CHANNEL_X0Y5 / GTYE4_COMMON_X0Y1
set_property -dict {LOC V1  } [get_ports {pcie_rx_n[2]}] ;# MGTYRXN1_225 GTYE4_CHANNEL_X0Y5 / GTYE4_COMMON_X0Y1
set_property -dict {LOC W5  } [get_ports {pcie_tx_p[2]}] ;# MGTYTXP1_225 GTYE4_CHANNEL_X0Y5 / GTYE4_COMMON_X0Y1
set_property -dict {LOC W4  } [get_ports {pcie_tx_n[2]}] ;# MGTYTXN1_225 GTYE4_CHANNEL_X0Y5 / GTYE4_COMMON_X0Y1
set_property -dict {LOC Y2  } [get_ports {pcie_rx_p[3]}] ;# MGTYRXP0_225 GTYE4_CHANNEL_X0Y4 / GTYE4_COMMON_X0Y1
set_property -dict {LOC Y1  } [get_ports {pcie_rx_n[3]}] ;# MGTYRXN0_225 GTYE4_CHANNEL_X0Y4 / GTYE4_COMMON_X0Y1
set_property -dict {LOC AA5 } [get_ports {pcie_tx_p[3]}] ;# MGTYTXP0_225 GTYE4_CHANNEL_X0Y4 / GTYE4_COMMON_X0Y1
set_property -dict {LOC AA4 } [get_ports {pcie_tx_n[3]}] ;# MGTYTXN0_225 GTYE4_CHANNEL_X0Y4 / GTYE4_COMMON_X0Y1
set_property -dict {LOC AB2 } [get_ports {pcie_rx_p[4]}] ;# MGTYRXP3_224 GTYE4_CHANNEL_X0Y3 / GTYE4_COMMON_X0Y0
set_property -dict {LOC AB1 } [get_ports {pcie_rx_n[4]}] ;# MGTYRXN3_224 GTYE4_CHANNEL_X0Y3 / GTYE4_COMMON_X0Y0
set_property -dict {LOC AC5 } [get_ports {pcie_tx_p[4]}] ;# MGTYTXP3_224 GTYE4_CHANNEL_X0Y3 / GTYE4_COMMON_X0Y0
set_property -dict {LOC AC4 } [get_ports {pcie_tx_n[4]}] ;# MGTYTXN3_224 GTYE4_CHANNEL_X0Y3 / GTYE4_COMMON_X0Y0
set_property -dict {LOC AD2 } [get_ports {pcie_rx_p[5]}] ;# MGTYRXP2_224 GTYE4_CHANNEL_X0Y2 / GTYE4_COMMON_X0Y0
set_property -dict {LOC AD1 } [get_ports {pcie_rx_n[5]}] ;# MGTYRXN2_224 GTYE4_CHANNEL_X0Y2 / GTYE4_COMMON_X0Y0
set_property -dict {LOC AD7 } [get_ports {pcie_tx_p[5]}] ;# MGTYTXP2_224 GTYE4_CHANNEL_X0Y2 / GTYE4_COMMON_X0Y0
set_property -dict {LOC AD6 } [get_ports {pcie_tx_n[5]}] ;# MGTYTXN2_224 GTYE4_CHANNEL_X0Y2 / GTYE4_COMMON_X0Y0
set_property -dict {LOC AE4 } [get_ports {pcie_rx_p[6]}] ;# MGTYRXP1_224 GTYE4_CHANNEL_X0Y1 / GTYE4_COMMON_X0Y0
set_property -dict {LOC AE3 } [get_ports {pcie_rx_n[6]}] ;# MGTYRXN1_224 GTYE4_CHANNEL_X0Y1 / GTYE4_COMMON_X0Y0
set_property -dict {LOC AE9 } [get_ports {pcie_tx_p[6]}] ;# MGTYTXP1_224 GTYE4_CHANNEL_X0Y1 / GTYE4_COMMON_X0Y0
set_property -dict {LOC AE8 } [get_ports {pcie_tx_n[6]}] ;# MGTYTXN1_224 GTYE4_CHANNEL_X0Y1 / GTYE4_COMMON_X0Y0
set_property -dict {LOC AF2 } [get_ports {pcie_rx_p[7]}] ;# MGTYRXP0_224 GTYE4_CHANNEL_X0Y0 / GTYE4_COMMON_X0Y0
set_property -dict {LOC AF1 } [get_ports {pcie_rx_n[7]}] ;# MGTYRXN0_224 GTYE4_CHANNEL_X0Y0 / GTYE4_COMMON_X0Y0
set_property -dict {LOC AF7 } [get_ports {pcie_tx_p[7]}] ;# MGTYTXP0_224 GTYE4_CHANNEL_X0Y0 / GTYE4_COMMON_X0Y0
set_property -dict {LOC AF6 } [get_ports {pcie_tx_n[7]}] ;# MGTYTXN0_224 GTYE4_CHANNEL_X0Y0 / GTYE4_COMMON_X0Y0
set_property -dict {LOC V7  } [get_ports pcie_refclk_p] ;# MGTREFCLK0P_225
set_property -dict {LOC V6  } [get_ports pcie_refclk_n] ;# MGTREFCLK0N_225
set_property -dict {LOC T19 IOSTANDARD LVCMOS18 PULLUP true} [get_ports pcie_reset_n]

set_false_path -from [get_ports {pcie_reset_n}]
set_input_delay 0 [get_ports {pcie_reset_n}]

# 100 MHz MGT reference clock
create_clock -period 10 -name pcie_mgt_refclk [get_ports pcie_refclk_p]

# QSPI flash
set_property -dict {LOC N23  IOSTANDARD LVCMOS18 DRIVE 8} [get_ports {qspi_1_dq[0]}]
set_property -dict {LOC P23  IOSTANDARD LVCMOS18 DRIVE 8} [get_ports {qspi_1_dq[1]}]
set_property -dict {LOC R20  IOSTANDARD LVCMOS18 DRIVE 8} [get_ports {qspi_1_dq[2]}]
set_property -dict {LOC R21  IOSTANDARD LVCMOS18 DRIVE 8} [get_ports {qspi_1_dq[3]}]
set_property -dict {LOC U22  IOSTANDARD LVCMOS18 DRIVE 8} [get_ports {qspi_1_cs}]

set_false_path -to [get_ports {qspi_1_dq[*] qspi_1_cs}]
set_output_delay 0 [get_ports {qspi_1_dq[*] qspi_1_cs}]
set_false_path -from [get_ports {qspi_1_dq}]
set_input_delay 0 [get_ports {qspi_1_dq}]