
create_ip -name gtwizard_ultrascale -vendor xilinx.com -library ip -module_name gtwizard_ultrascale_2

set_property -dict [list CONFIG.preset {GTY-10GBASE-R}] [get_ips gtwizard_ultrascale_2]

set_property -dict [list \
    CONFIG.CHANNEL_ENABLE {X0Y51 X0Y50 X0Y49 X0Y48 X0Y43 X0Y42 X0Y41 X0Y40} \
    CONFIG.TX_MASTER_CHANNEL {X0Y48} \
    CONFIG.RX_MASTER_CHANNEL {X0Y48} \
    CONFIG.TX_LINE_RATE {25.78125} \
    CONFIG.TX_REFCLK_FREQUENCY {161.1328125} \
    CONFIG.TX_USER_DATA_WIDTH {64} \
    CONFIG.TX_INT_DATA_WIDTH {64} \
    CONFIG.RX_LINE_RATE {25.78125} \
    CONFIG.RX_REFCLK_FREQUENCY {161.1328125} \
    CONFIG.RX_USER_DATA_WIDTH {64} \
    CONFIG.RX_INT_DATA_WIDTH {64} \
    CONFIG.RX_REFCLK_SOURCE {X0Y51 clk0 X0Y50 clk0 X0Y49 clk0 X0Y48 clk0 X0Y43 clk0+1 X0Y42 clk0+1 X0Y41 clk0+1 X0Y40 clk0+1} \
    CONFIG.TX_REFCLK_SOURCE {X0Y51 clk0 X0Y50 clk0 X0Y49 clk0 X0Y48 clk0 X0Y43 clk0+1 X0Y42 clk0+1 X0Y41 clk0+1 X0Y40 clk0+1} \
    CONFIG.FREERUN_FREQUENCY {125} \
] [get_ips gtwizard_ultrascale_2]
