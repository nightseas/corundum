# Placement constraints

create_pblock pblock_slr1
add_cells_to_pblock [get_pblocks pblock_slr1] [get_cells -quiet [list core_inst/dma_if_mux_inst]]
add_cells_to_pblock [get_pblocks pblock_slr1] [get_cells -quiet [list core_inst/iface[0].interface_inst]]
# add_cells_to_pblock [get_pblocks pblock_slr1] [get_cells -quiet [list core_inst/iface[1].interface_inst]]
resize_pblock [get_pblocks pblock_slr1] -add {SLR1}

create_pblock pblock_pcie
add_cells_to_pblock [get_pblocks pblock_pcie] [get_cells -quiet [list pcie4_uscale_plus_inst]]
add_cells_to_pblock [get_pblocks pblock_pcie] [get_cells -quiet [list core_inst/dma_if_pcie_us_inst]]
resize_pblock [get_pblocks pblock_pcie] -add {CLOCKREGION_X3Y5:CLOCKREGION_X5Y9}

create_pblock pblock_eth
add_cells_to_pblock [get_pblocks pblock_eth] [get_cells -quiet [list qsfp1_cmac_pad_inst]]
add_cells_to_pblock [get_pblocks pblock_eth] [get_cells -quiet [list core_inst/iface[0].mac[0].mac_tx_fifo_inst core_inst/iface[0].mac[0].mac_rx_fifo_inst]]
# add_cells_to_pblock [get_pblocks pblock_eth] [get_cells -quiet [list qsfp2_cmac_pad_inst]]
# add_cells_to_pblock [get_pblocks pblock_eth] [get_cells -quiet [list core_inst/iface[1].mac[0].mac_tx_fifo_inst core_inst/iface[1].mac[0].mac_rx_fifo_inst]]
resize_pblock [get_pblocks pblock_eth] -add {CLOCKREGION_X0Y5:CLOCKREGION_X0Y9}
