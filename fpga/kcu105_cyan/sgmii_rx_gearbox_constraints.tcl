# The generated Xilinx LVDS PCS excludes the receive gearbox distributed-RAM
# write-to-output paths from timing analysis.  They are functionally guarded
# by the gearbox address schedule, but unconstrained placement can separate
# the RAM and output mux/registers far enough to lose received symbols on the
# board.  Keep only this 10-bit receive bundle physically compact; do not
# alter the generated PCS logic or its other timing exceptions.

set cyan_sgmii_rx_gearbox_prefix \
    "u_enet/u_pcs_pma/inst/pcs_pma_block_i/lvds_transceiver_mw/serdes_1_to_10_ser8_i/gb0"
set cyan_sgmii_rx_gearbox_cells \
    [get_cells -quiet -hierarchical -filter \
        "NAME == $cyan_sgmii_rx_gearbox_prefix"]
set cyan_sgmii_rx_gearbox_sources \
    [get_pins -quiet -hierarchical -filter \
        "NAME =~ $cyan_sgmii_rx_gearbox_prefix/loop2\[*\].ram_ins*/RAM*/CLK*"]
set cyan_sgmii_rx_gearbox_destinations \
    [get_pins -quiet -hierarchical -filter \
        "NAME =~ $cyan_sgmii_rx_gearbox_prefix/loop0\[*\].dataout_reg\[*\]/D"]

if {[llength $cyan_sgmii_rx_gearbox_cells] != 1 ||
    [llength $cyan_sgmii_rx_gearbox_sources] != 16 ||
    [llength $cyan_sgmii_rx_gearbox_destinations] != 10} {
    error "SGMII receive gearbox boundary is incomplete: cells=[llength $cyan_sgmii_rx_gearbox_cells] clocks=[llength $cyan_sgmii_rx_gearbox_sources] destinations=[llength $cyan_sgmii_rx_gearbox_destinations]"
}

# Preserve the compact 4x4 placement observed in two independently accepted
# images without constraining routing or reserving the area from unrelated
# logic.  A 6x6 box leaves the placer one slice of margin on every side.  The
# physically rejected same-source route spread this hierarchy over 11x9
# slices; the rejected current route spread it over 7x36.  Routed admission
# still measures the real ignored datapath and rejects anything above 1.200 ns.
set cyan_sgmii_rx_gearbox_floorplan \
    [get_pblocks -quiet cyan_sgmii_rx_gearbox_floorplan]
if {[llength $cyan_sgmii_rx_gearbox_floorplan] == 0} {
    set cyan_sgmii_rx_gearbox_floorplan \
        [create_pblock cyan_sgmii_rx_gearbox_floorplan]
} elseif {[llength $cyan_sgmii_rx_gearbox_floorplan] != 1} {
    error "SGMII receive gearbox floorplan resolved to [llength $cyan_sgmii_rx_gearbox_floorplan] pblocks, expected at most 1"
}
resize_pblock $cyan_sgmii_rx_gearbox_floorplan -add \
    {SLICE_X49Y124:SLICE_X54Y129}
add_cells_to_pblock $cyan_sgmii_rx_gearbox_floorplan \
    $cyan_sgmii_rx_gearbox_cells
set_property CONTAIN_ROUTING false $cyan_sgmii_rx_gearbox_floorplan
set_property EXCLUDE_PLACEMENT false $cyan_sgmii_rx_gearbox_floorplan
puts "Applied compact SGMII receive-gearbox floorplan to SLICE_X49Y124:SLICE_X54Y129"
