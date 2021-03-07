
create_ip -name gtwizard_ultrascale -vendor xilinx.com -library ip -module_name gtwizard_ultrascale_1

set_property -dict [list CONFIG.preset {GTY-10GBASE-R}] [get_ips gtwizard_ultrascale_1]

set_property -dict [list \
    CONFIG.CHANNEL_ENABLE {X0Y31 X0Y30 X0Y29 X0Y28 X0Y27 X0Y26 X0Y25 X0Y24} \
    CONFIG.TX_MASTER_CHANNEL {X0Y28} \
    CONFIG.RX_MASTER_CHANNEL {X0Y28} \
    CONFIG.TX_LINE_RATE {25.78125} \
    CONFIG.TX_REFCLK_FREQUENCY {161.1328125} \
    CONFIG.TX_USER_DATA_WIDTH {64} \
    CONFIG.TX_INT_DATA_WIDTH {64} \
    CONFIG.RX_LINE_RATE {25.78125} \
    CONFIG.RX_REFCLK_FREQUENCY {161.1328125} \
    CONFIG.RX_USER_DATA_WIDTH {64} \
    CONFIG.RX_INT_DATA_WIDTH {64} \
    CONFIG.RX_REFCLK_SOURCE {X0Y31 clk0 X0Y30 clk0 X0Y29 clk0 X0Y28 clk0 X0Y27 clk0+1 X0Y26 clk0+1 X0Y25 clk0+1 X0Y24 clk0+1} \
    CONFIG.TX_REFCLK_SOURCE {X0Y31 clk0 X0Y30 clk0 X0Y29 clk0 X0Y28 clk0 X0Y27 clk0+1 X0Y26 clk0+1 X0Y25 clk0+1 X0Y24 clk0+1} \
    CONFIG.FREERUN_FREQUENCY {125} \
] [get_ips gtwizard_ultrascale_1]
