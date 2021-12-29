
create_ip -name gtwizard_ultrascale -vendor xilinx.com -library ip -module_name gtwizard_ultrascale_0

set_property -dict [list CONFIG.preset {GTY-10GBASE-R}] [get_ips gtwizard_ultrascale_0]

set_property -dict [list \
    CONFIG.CHANNEL_ENABLE {X1Y15 X1Y14 X1Y13 X1Y12 X1Y11 X1Y10 X1Y9 X1Y8} \
    CONFIG.TX_MASTER_CHANNEL {X1Y8} \
    CONFIG.RX_MASTER_CHANNEL {X1Y8} \
    CONFIG.TX_LINE_RATE {25.78125} \
    CONFIG.TX_REFCLK_FREQUENCY {161.1328125} \
    CONFIG.TX_USER_DATA_WIDTH {64} \
    CONFIG.TX_INT_DATA_WIDTH {64} \
    CONFIG.RX_LINE_RATE {25.78125} \
    CONFIG.RX_REFCLK_FREQUENCY {161.1328125} \
    CONFIG.RX_USER_DATA_WIDTH {64} \
    CONFIG.RX_INT_DATA_WIDTH {64} \
    CONFIG.RX_REFCLK_SOURCE {X1Y15 clk0 X1Y14 clk0 X1Y13 clk0 X1Y12 clk0 X1Y11 clk0 X1Y10 clk0 X1Y9 clk0 X1Y8 clk0} \
    CONFIG.TX_REFCLK_SOURCE {X1Y15 clk0 X1Y14 clk0 X1Y13 clk0 X1Y12 clk0 X1Y11 clk0 X1Y10 clk0 X1Y9 clk0 X1Y8 clk0} \
    CONFIG.FREERUN_FREQUENCY {125} \
] [get_ips gtwizard_ultrascale_0]
