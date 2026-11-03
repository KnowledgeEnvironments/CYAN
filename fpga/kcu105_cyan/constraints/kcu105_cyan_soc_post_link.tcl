
# XPM FIFO reset constraints are loaded during link_design after board XDC.
# Apply the SoC CDC cut from implementation hooks so it has final precedence.
source [file normalize [file join [file dirname [info script]] .. \
    cpu_island_cdc_constraints.tcl]]
source [file normalize [file join [file dirname [info script]] .. \
    lg1_serial_cdc_constraints.tcl]]

# Keep the 100 MHz R4000 core in the coarse right-half footprint proved by
# the dc503dca route (CPU WNS +0.182 ns).  The d0216c98 and 5f5ed3fd routes
# spread this unchanged core into the leftmost clock-region column and failed
# CPU setup by 0.226 ns and 1.075 ns respectively.  Constrain only the core;
# cache RAM placement and all non-CPU logic remain free, and routing is not
# contained by the pblock.
set cyan_cpu_core_floorplan_cells [get_cells -quiet -hierarchical \
    -filter {NAME == u_soc/cpu/u_core}]
if {[llength $cyan_cpu_core_floorplan_cells] != 1} {
    error "CPU-core floorplan resolved to [llength $cyan_cpu_core_floorplan_cells] cells, expected 1: $cyan_cpu_core_floorplan_cells"
}
set cyan_cpu_core_floorplan [get_pblocks -quiet cyan_cpu_core_floorplan]
if {[llength $cyan_cpu_core_floorplan] == 0} {
    set cyan_cpu_core_floorplan [create_pblock cyan_cpu_core_floorplan]
} elseif {[llength $cyan_cpu_core_floorplan] != 1} {
    error "CPU-core floorplan resolved to [llength $cyan_cpu_core_floorplan] pblocks, expected at most 1: $cyan_cpu_core_floorplan"
}
resize_pblock $cyan_cpu_core_floorplan -add \
    {CLOCKREGION_X2Y1:CLOCKREGION_X3Y4}
add_cells_to_pblock $cyan_cpu_core_floorplan \
    $cyan_cpu_core_floorplan_cells
set_property CONTAIN_ROUTING false $cyan_cpu_core_floorplan
set_property EXCLUDE_PLACEMENT false $cyan_cpu_core_floorplan
puts "Applied coarse CPU-core floorplan to CLOCKREGION_X2Y1:CLOCKREGION_X3Y4"

# The receive-gearbox box is spatially contained by the coarse CPU footprint,
# so Vivado represents it as a child pblock.  Create the parent first: reading
# a child before its parent causes Vivado to reassign the child's cells to the
# parent when the routed checkpoint is reopened (Constraints 18-5685).
source [file normalize [file join [file dirname [info script]] .. \
    sgmii_rx_gearbox_constraints.tcl]]

set cpu_clocks [get_clocks -quiet cpu_clk]
set sgmii_clocks [get_clocks -quiet clk125_i]
set mig_clocks [get_clocks -quiet mmcm_clkout0]
if {[llength $mig_clocks] == 0} {
    set mig_clocks [get_clocks -quiet c0_ddr4_clk]
}
set video_clocks [get_clocks -quiet video_clk_mmcm]
if {[llength $video_clocks] == 0} {
    set video_clocks [get_clocks -quiet -of_objects \
        [get_pins -quiet u_video_mmcm/CLKOUT0]]
}
set lg1_serial_clocks [get_clocks -quiet lg1_serial_clk_mmcm]
if {[llength $lg1_serial_clocks] == 0} {
    set lg1_serial_clocks [get_clocks -quiet -of_objects \
        [get_pins -quiet u_lg1_serial_clk_buf/O]]
}
if {[llength $lg1_serial_clocks] != 1} {
    error "KCU105 SoC CDC constraints resolved [llength $lg1_serial_clocks] LG1 serial clocks, expected exactly 1: $lg1_serial_clocks"
}

# CYAN-owned SGMII state uses asynchronous assertion and four-stage
# synchronous release on the PCS 125 MHz clock.  Fail closed on the exact
# synchronizer population and cut only its asynchronous clear pins; every
# data/control crossing remains covered by the clock-domain contract below.
set pcs_reset_sync_cells [lsort -unique [get_cells -quiet -hierarchical \
    -regexp -include_replicated_objects \
    {.*u_enet/pcs_reset_sync_reg\[[0-3]\]}]]
if {[llength $pcs_reset_sync_cells] != 4} {
    error "SGMII PCS reset synchronizer resolved to [llength $pcs_reset_sync_cells] cells, expected 4: $pcs_reset_sync_cells"
}
foreach cell $pcs_reset_sync_cells {
    set ref_name [get_property REF_NAME $cell]
    set async_reg [get_property ASYNC_REG $cell]
    if {![regexp {^FD} $ref_name] ||
        ![string is true -strict $async_reg]} {
        error "SGMII PCS reset cell $cell is not an ASYNC_REG flop: REF_NAME=$ref_name ASYNC_REG=$async_reg"
    }
}
set pcs_reset_sync_clr [get_pins -quiet -of_objects $pcs_reset_sync_cells \
    -filter {REF_PIN_NAME == CLR}]
set pcs_reset_sync_clk [get_pins -quiet -of_objects $pcs_reset_sync_cells \
    -filter {REF_PIN_NAME == C}]
set pcs_reset_clocks [lsort -unique \
    [get_clocks -quiet -of_objects $pcs_reset_sync_clk]]
if {[llength $pcs_reset_sync_clr] != 4 ||
    [llength $pcs_reset_sync_clk] != 4 ||
    [llength $pcs_reset_clocks] != 1} {
    error "SGMII PCS reset boundary is incomplete: clr=$pcs_reset_sync_clr clk=$pcs_reset_sync_clk clocks=$pcs_reset_clocks"
}
set_false_path -to $pcs_reset_sync_clr
puts "Applied SGMII PCS asynchronous-reset cut to exactly 4 CLR pins on $pcs_reset_clocks"

# ADV7511 is a source-synchronous SDR sink.  The FPGA launches data/control on
# video-clock rising edges and forwards an inverted clock, placing the sink's
# rising sample edge half a pixel later.  The ADV7511 requires 1.0 ns setup and
# 0.7 ns hold.  Add a conservative 0.5 ns board trace-skew budget until exact
# KCU105 clock/data flight-time mismatch is available.
set adv_clock_ports [get_ports -quiet hdmi_out_clk]
set adv_video_ports [get_ports -quiet {
    hdmi_hsync hdmi_vsync hdmi_data_e hdmi_data[*]
}]
set adv_clock_source [get_pins -quiet u_adv7511/u_hdmi_clk_oddr/C]
if {[llength $adv_clock_source] == 0} {
    # ODDRE1 is retargeted to OSERDESE3 during UltraScale synthesis.
    set adv_clock_source \
        [get_pins -quiet u_adv7511/u_hdmi_clk_oddr/CLKDIV]
}
if {[llength $adv_clock_ports] != 1 ||
    [llength $adv_clock_source] != 1 ||
    [llength $adv_video_ports] != 19} {
    error "ADV7511 output timing boundary is incomplete: clock_ports=[llength $adv_clock_ports] clock_source=[llength $adv_clock_source] video_ports=[llength $adv_video_ports]"
}
create_generated_clock -name adv7511_pclk \
    -source $adv_clock_source -divide_by 1 -invert $adv_clock_ports
set_output_delay -clock [get_clocks adv7511_pclk] \
    -max 1.500 $adv_video_ports
set_output_delay -clock [get_clocks adv7511_pclk] \
    -min -1.200 $adv_video_ports
puts "Applied center-aligned ADV7511 SDR output timing to 19 data/control ports"


if {[llength $cpu_clocks] == 0 || [llength $sgmii_clocks] == 0} {
    puts "WARNING: KCU105 SoC CDC constraints did not find cpu_clk/clk125_i"
} else {
    set_false_path -from $cpu_clocks -to $sgmii_clocks
    set_false_path -from $sgmii_clocks -to $cpu_clocks
    set_clock_groups -asynchronous -group $cpu_clocks -group $sgmii_clocks
    puts "Applied KCU105 SoC CDC constraints between cpu_clk and clk125_i"
}

if {[llength $cpu_clocks] == 0 || [llength $mig_clocks] == 0} {
    puts "WARNING: KCU105 SoC CDC constraints did not find cpu_clk/MIG UI clock"
} else {
    set_false_path -from $cpu_clocks -to $mig_clocks
    set_false_path -from $mig_clocks -to $cpu_clocks
    set_clock_groups -asynchronous -group $cpu_clocks -group $mig_clocks
    puts "Applied KCU105 SoC CDC constraints between cpu_clk and MIG UI clock"
}

# The video MMCM has a rational relationship to the MIG UI source clock, but
# every crossing is deliberately implemented as an asynchronous FIFO or a
# tagged synchronizer.  Treating the clocks as phase-related creates
# impossible setup/hold requirements at their occasionally coincident edges.
if {[llength $video_clocks] == 0} {
    puts "WARNING: KCU105 SoC CDC constraints did not find the video clock"
} else {
    if {[llength $cpu_clocks] != 0} {
        set_clock_groups -asynchronous -group $video_clocks -group $cpu_clocks
    }
    if {[llength $mig_clocks] != 0} {
        set_clock_groups -asynchronous -group $video_clocks -group $mig_clocks
    }
    puts "Applied KCU105 SoC CDC constraints for the video clock"
}

# LG1 pixel production now runs at the physical 64.0256 MHz serial cadence.
# Its HDMI frame handoff crosses into MIG through XPM asynchronous FIFOs and
# held toggle/payload bundles, then into the video clock through the same
# explicit CDC architecture.  Classify only those two clock pairs as
# asynchronous; machine<->serial remains directionally constrained by the
# fail-closed LG1 mailbox contract sourced above.
if {[llength $mig_clocks] != 1 || [llength $video_clocks] != 1} {
    error "LG1 serial CDC classification requires exactly one MIG and video clock: mig=$mig_clocks video=$video_clocks"
}
set_clock_groups -asynchronous \
    -group $lg1_serial_clocks -group $mig_clocks
set_clock_groups -asynchronous \
    -group $lg1_serial_clocks -group $video_clocks
puts "Applied KCU105 SoC CDC constraints from LG1 serial to MIG/video clocks"
