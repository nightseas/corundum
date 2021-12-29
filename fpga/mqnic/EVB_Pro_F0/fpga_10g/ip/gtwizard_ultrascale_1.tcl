
create_ip -name gtwizard_ultrascale -vendor xilinx.com -library ip -module_name gtwizard_ultrascale_1

set_property -dict [list CONFIG.preset {GTY-10GBASE-R}] [get_ips gtwizard_ultrascale_1]

set_property -dict [list \
    CONFIG.CHANNEL_ENABLE {X1Y35 X1Y34 X1Y33 X1Y32 X1Y31 X1Y30 X1Y29 X1Y28} \
    CONFIG.TX_MASTER_CHANNEL {X1Y28} \
    CONFIG.RX_MASTER_CHANNEL {X1Y28} \
    CONFIG.TX_LINE_RATE {10.3125} \
    CONFIG.TX_REFCLK_FREQUENCY {161.1328125} \
    CONFIG.TX_USER_DATA_WIDTH {64} \
    CONFIG.TX_INT_DATA_WIDTH {64} \
    CONFIG.RX_LINE_RATE {10.3125} \
    CONFIG.RX_REFCLK_FREQUENCY {161.1328125} \
    CONFIG.RX_USER_DATA_WIDTH {64} \
    CONFIG.RX_INT_DATA_WIDTH {64} \
    CONFIG.RX_REFCLK_SOURCE {X1Y35 clk0 X1Y34 clk0 X1Y33 clk0 X1Y32 clk0 X1Y31 clk0 X1Y30 clk0 X1Y29 clk0 X1Y28 clk0} \
    CONFIG.TX_REFCLK_SOURCE {X1Y35 clk0 X1Y34 clk0 X1Y33 clk0 X1Y32 clk0 X1Y31 clk0 X1Y30 clk0 X1Y29 clk0 X1Y28 clk0} \
    CONFIG.FREERUN_FREQUENCY {125} \
] [get_ips gtwizard_ultrascale_1]
