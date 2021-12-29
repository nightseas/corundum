
create_ip -name gtwizard_ultrascale -vendor xilinx.com -library ip -module_name gtwizard_ultrascale_2

set_property -dict [list CONFIG.preset {GTY-10GBASE-R}] [get_ips gtwizard_ultrascale_2]

set_property -dict [list \
    CONFIG.CHANNEL_ENABLE {X1Y51 X1Y50 X1Y49 X1Y48 X1Y47 X1Y46 X1Y45 X1Y44} \
    CONFIG.TX_MASTER_CHANNEL {X1Y44} \
    CONFIG.RX_MASTER_CHANNEL {X1Y44} \
    CONFIG.TX_LINE_RATE {25.78125} \
    CONFIG.TX_REFCLK_FREQUENCY {161.1328125} \
    CONFIG.TX_USER_DATA_WIDTH {64} \
    CONFIG.TX_INT_DATA_WIDTH {64} \
    CONFIG.RX_LINE_RATE {25.78125} \
    CONFIG.RX_REFCLK_FREQUENCY {161.1328125} \
    CONFIG.RX_USER_DATA_WIDTH {64} \
    CONFIG.RX_INT_DATA_WIDTH {64} \
    CONFIG.RX_REFCLK_SOURCE {X1Y51 clk0 X1Y50 clk0 X1Y49 clk0 X1Y48 clk0 X1Y47 clk0 X1Y46 clk0 X1Y45 clk0 X1Y44 clk0} \
    CONFIG.TX_REFCLK_SOURCE {X1Y51 clk0 X1Y50 clk0 X1Y49 clk0 X1Y48 clk0 X1Y47 clk0 X1Y46 clk0 X1Y45 clk0 X1Y44 clk0} \
    CONFIG.FREERUN_FREQUENCY {125} \
] [get_ips gtwizard_ultrascale_2]
