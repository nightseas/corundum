# Placement constraints for FPGA1 all 3 dies

############################### FPGA1 D0 ###############################

create_pblock pblock_slr0
add_cells_to_pblock [get_pblocks pblock_slr0] [get_cells -quiet [list core_inst0/dma_if_mux_inst]]
add_cells_to_pblock [get_pblocks pblock_slr0] [get_cells -quiet [list core_inst0/dma_if_mux_ctrl_inst]]
add_cells_to_pblock [get_pblocks pblock_slr0] [get_cells -quiet [list core_inst0/dma_if_mux_data_inst]]
resize_pblock [get_pblocks pblock_slr0] -add {SLR0}

create_pblock pblock_pcie0
add_cells_to_pblock [get_pblocks pblock_pcie0] [get_cells -quiet [list pcie4_uscale_plus_inst0]]
resize_pblock [get_pblocks pblock_pcie0] -add {CLOCKREGION_X3Y0:CLOCKREGION_X5Y3}

############################### FPGA1 D1 ###############################

create_pblock pblock_slr1
add_cells_to_pblock [get_pblocks pblock_slr1] [get_cells -quiet [list core_inst1/dma_if_mux_inst]]
add_cells_to_pblock [get_pblocks pblock_slr1] [get_cells -quiet [list core_inst1/dma_if_mux_ctrl_inst]]
add_cells_to_pblock [get_pblocks pblock_slr1] [get_cells -quiet [list core_inst1/dma_if_mux_data_inst]]
resize_pblock [get_pblocks pblock_slr1] -add {SLR1}

create_pblock pblock_pcie1
add_cells_to_pblock [get_pblocks pblock_pcie1] [get_cells -quiet [list pcie4_uscale_plus_inst1]]
resize_pblock [get_pblocks pblock_pcie1] -add {CLOCKREGION_X3Y5:CLOCKREGION_X5Y9}

############################### FPGA1 D2 ###############################

create_pblock pblock_slr2
add_cells_to_pblock [get_pblocks pblock_slr2] [get_cells -quiet [list core_inst2/dma_if_mux_inst]]
add_cells_to_pblock [get_pblocks pblock_slr2] [get_cells -quiet [list core_inst2/dma_if_mux_ctrl_inst]]
add_cells_to_pblock [get_pblocks pblock_slr2] [get_cells -quiet [list core_inst2/dma_if_mux_data_inst]]
resize_pblock [get_pblocks pblock_slr2] -add {SLR2}

create_pblock pblock_pcie2
add_cells_to_pblock [get_pblocks pblock_pcie2] [get_cells -quiet [list pcie4_uscale_plus_inst2]]
resize_pblock [get_pblocks pblock_pcie2] -add {CLOCKREGION_X3Y10:CLOCKREGION_X5Y14}
