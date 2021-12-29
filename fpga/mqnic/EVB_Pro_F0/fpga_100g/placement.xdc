# Placement constraints for FPGA0 all 3 dies

############################### FPGA0 D0 ###############################

create_pblock pblock_slr0
add_cells_to_pblock [get_pblocks pblock_slr0] [get_cells -quiet [list core_inst0/dma_if_mux_inst]]
# add_cells_to_pblock [get_pblocks pblock_slr0] [get_cells -quiet [list core_inst0/dma_if_mux_ctrl_inst]]
# add_cells_to_pblock [get_pblocks pblock_slr0] [get_cells -quiet [list core_inst0/dma_if_mux_data_inst]]
add_cells_to_pblock [get_pblocks pblock_slr0] [get_cells -quiet [list core_inst0/iface[0].interface_inst]]
# add_cells_to_pblock [get_pblocks pblock_slr0] [get_cells -quiet [list core_inst0/iface[1].interface_inst]]
resize_pblock [get_pblocks pblock_slr0] -add {SLR0}

create_pblock pblock_pcie0
add_cells_to_pblock [get_pblocks pblock_pcie0] [get_cells -quiet [list pcie4_uscale_plus_inst0]]
resize_pblock [get_pblocks pblock_pcie0] -add {CLOCKREGION_X0Y0:CLOCKREGION_X2Y4}

create_pblock pblock_eth0
add_cells_to_pblock [get_pblocks pblock_eth0] [get_cells -quiet [list qsfp0_1_cmac_pad_inst]]
add_cells_to_pblock [get_pblocks pblock_eth0] [get_cells -quiet [list core_inst0/iface[0].mac[0].mac_tx_fifo_inst core_inst0/iface[0].mac[0].mac_rx_fifo_inst]]
# add_cells_to_pblock [get_pblocks pblock_eth0] [get_cells -quiet [list qsfp0_2_cmac_pad_inst]]
# add_cells_to_pblock [get_pblocks pblock_eth0] [get_cells -quiet [list core_inst0/iface[1].mac[0].mac_tx_fifo_inst core_inst0/iface[1].mac[0].mac_rx_fifo_inst]]
resize_pblock [get_pblocks pblock_eth0] -add {CLOCKREGION_X0Y0:CLOCKREGION_X0Y4}

############################### FPGA0 D1 ###############################

create_pblock pblock_slr1
add_cells_to_pblock [get_pblocks pblock_slr1] [get_cells -quiet [list core_inst1/dma_if_mux_inst]]
# add_cells_to_pblock [get_pblocks pblock_slr1] [get_cells -quiet [list core_inst1/dma_if_mux_ctrl_inst]]
# add_cells_to_pblock [get_pblocks pblock_slr1] [get_cells -quiet [list core_inst1/dma_if_mux_data_inst]]
add_cells_to_pblock [get_pblocks pblock_slr1] [get_cells -quiet [list core_inst1/iface[0].interface_inst]]
# add_cells_to_pblock [get_pblocks pblock_slr1] [get_cells -quiet [list core_inst1/iface[1].interface_inst]]
resize_pblock [get_pblocks pblock_slr1] -add {SLR1}

create_pblock pblock_pcie1
add_cells_to_pblock [get_pblocks pblock_pcie1] [get_cells -quiet [list pcie4_uscale_plus_inst1]]
resize_pblock [get_pblocks pblock_pcie1] -add {CLOCKREGION_X1Y5:CLOCKREGION_X5Y6}

create_pblock pblock_eth1
add_cells_to_pblock [get_pblocks pblock_eth1] [get_cells -quiet [list qsfp1_1_cmac_pad_inst]]
add_cells_to_pblock [get_pblocks pblock_eth1] [get_cells -quiet [list core_inst1/iface[0].mac[0].mac_tx_fifo_inst core_inst1/iface[0].mac[0].mac_rx_fifo_inst]]
# add_cells_to_pblock [get_pblocks pblock_eth1] [get_cells -quiet [list qsfp1_2_cmac_pad_inst]]
# add_cells_to_pblock [get_pblocks pblock_eth1] [get_cells -quiet [list core_inst1/iface[1].mac[0].mac_tx_fifo_inst core_inst1/iface[1].mac[0].mac_rx_fifo_inst]]
resize_pblock [get_pblocks pblock_eth1] -add {CLOCKREGION_X0Y5:CLOCKREGION_X0Y9}

############################### FPGA0 D2 ###############################

create_pblock pblock_slr2
add_cells_to_pblock [get_pblocks pblock_slr2] [get_cells -quiet [list core_inst2/dma_if_mux_inst]]
# add_cells_to_pblock [get_pblocks pblock_slr2] [get_cells -quiet [list core_inst2/dma_if_mux_ctrl_inst]]
# add_cells_to_pblock [get_pblocks pblock_slr2] [get_cells -quiet [list core_inst2/dma_if_mux_data_inst]]
add_cells_to_pblock [get_pblocks pblock_slr2] [get_cells -quiet [list core_inst2/iface[0].interface_inst]]
# add_cells_to_pblock [get_pblocks pblock_slr2] [get_cells -quiet [list core_inst2/iface[1].interface_inst]]
resize_pblock [get_pblocks pblock_slr2] -add {SLR2}

create_pblock pblock_pcie2
add_cells_to_pblock [get_pblocks pblock_pcie2] [get_cells -quiet [list pcie4_uscale_plus_inst2]]
resize_pblock [get_pblocks pblock_pcie2] -add {CLOCKREGION_X0Y10:CLOCKREGION_X2Y14}

create_pblock pblock_eth2
add_cells_to_pblock [get_pblocks pblock_eth2] [get_cells -quiet [list qsfp2_1_cmac_pad_inst]]
add_cells_to_pblock [get_pblocks pblock_eth2] [get_cells -quiet [list core_inst2/iface[0].mac[0].mac_tx_fifo_inst core_inst2/iface[0].mac[0].mac_rx_fifo_inst]]
# add_cells_to_pblock [get_pblocks pblock_eth2] [get_cells -quiet [list qsfp2_2_cmac_pad_inst]]
# add_cells_to_pblock [get_pblocks pblock_eth2] [get_cells -quiet [list core_inst2/iface[1].mac[0].mac_tx_fifo_inst core_inst2/iface[1].mac[0].mac_rx_fifo_inst]]
resize_pblock [get_pblocks pblock_eth2] -add {CLOCKREGION_X0Y10:CLOCKREGION_X0Y14}
