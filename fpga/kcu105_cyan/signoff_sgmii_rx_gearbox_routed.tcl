# Read-only routed-checkpoint signoff for the Xilinx LVDS SGMII receive
# gearbox.  The generated PCS IP intentionally false-paths its distributed-RAM
# write-clock to 125 MHz output-register crossings.  Those ignored paths still
# need compact routing: every physically accepted CYAN image measured on
# 2026-10-25 was at or below 1.096 ns, while both physically rejected images
# were at or above 1.442 ns and did not answer raw-Ethernet management frames.
#
# Usage:
#   vivado -mode batch -source signoff_sgmii_rx_gearbox_routed.tcl \
#       -tclargs ROUTED_DCP REPORT_DIR
#   tclsh signoff_sgmii_rx_gearbox_routed.tcl --syntax-check

proc cyan_sgmii_rx_usage {} {
    puts stderr "usage: vivado -mode batch -source signoff_sgmii_rx_gearbox_routed.tcl -tclargs ROUTED_DCP REPORT_DIR"
}

proc cyan_sgmii_rx_write {path contents} {
    set channel [open $path w]
    puts -nonewline $channel $contents
    close $channel
}

if {$argc == 1 && [lindex $argv 0] eq "--syntax-check"} {
    puts "PASS signoff_sgmii_rx_gearbox_routed.tcl Tcl syntax load"
    exit 0
}
if {$argc != 2} {
    cyan_sgmii_rx_usage
    exit 2
}

set routed_dcp [file normalize [lindex $argv 0]]
set report_dir [file normalize [lindex $argv 1]]
if {![file exists $routed_dcp]} {
    puts stderr "routed checkpoint does not exist: $routed_dcp"
    exit 2
}
file mkdir $report_dir
set summary_path [file join $report_dir cyan_sgmii_rx_gearbox_signoff.txt]
set timing_path [file join $report_dir cyan_sgmii_rx_gearbox_ignored_timing.rpt]

open_checkpoint $routed_dcp

set gearbox_prefix "u_enet/u_pcs_pma/inst/pcs_pma_block_i/lvds_transceiver_mw/serdes_1_to_10_ser8_i/gb0"
set source_pins [get_pins -quiet -hierarchical -filter \
    "NAME =~ $gearbox_prefix/loop2\[*\].ram_ins*/RAM*/CLK*"]
set destination_pins [get_pins -quiet -hierarchical -filter \
    "NAME =~ $gearbox_prefix/loop0\[*\].dataout_reg\[*\]/D"]

set failures {}
if {[llength $source_pins] != 16} {
    lappend failures "receive gearbox has [llength $source_pins] RAM clock pins, expected exactly 16"
}
if {[llength $destination_pins] != 10} {
    lappend failures "receive gearbox has [llength $destination_pins] output D pins, expected exactly 10"
}

set paths {}
if {[llength $source_pins] == 16 && [llength $destination_pins] == 10} {
    if {[catch {
        set paths [get_timing_paths -quiet -user_ignored -delay_type max \
            -from $source_pins -to $destination_pins \
            -max_paths 10 -nworst 1]
    } error_text]} {
        lappend failures "ignored-path query failed: $error_text"
    }
}
if {[llength $paths] != 10} {
    lappend failures "receive gearbox has [llength $paths] ignored RAM-to-output paths, expected exactly 10"
}

# Keep clear margin above the slowest accepted control (1.096 ns) while
# rejecting both failed routes (1.442 ns and 1.506 ns).  This is a physical
# implementation invariant, not a replacement for functional PCS tests.
set max_datapath_ns 1.200
set worst_datapath_ns 0.0
set worst_start ""
set worst_end ""
foreach path $paths {
    set delay [get_property DATAPATH_DELAY $path]
    if {$delay > $worst_datapath_ns} {
        set worst_datapath_ns $delay
        set worst_start [get_property STARTPOINT_PIN $path]
        set worst_end [get_property ENDPOINT_PIN $path]
    }
}
if {[llength $paths] == 10 && $worst_datapath_ns > $max_datapath_ns} {
    lappend failures [format \
        "receive gearbox worst ignored datapath %.3f ns exceeds %.3f ns: %s -> %s" \
        $worst_datapath_ns $max_datapath_ns $worst_start $worst_end]
}

if {[llength $source_pins] != 0 && [llength $destination_pins] != 0} {
    report_timing -user_ignored -delay_type max \
        -from $source_pins -to $destination_pins \
        -max_paths 10 -nworst 1 -path_type full_clock_expanded \
        -file $timing_path
}

set dcp_sha256 [lindex [split [exec sha256sum $routed_dcp]] 0]
set summary [format \
    "DCP=%s\nDCP_SHA256=%s\nSOURCE_PIN_COUNT=%d\nDESTINATION_PIN_COUNT=%d\nPATH_COUNT=%d\nMAX_DATAPATH_NS=%.3f\nWORST_DATAPATH_NS=%.3f\nWORST_START=%s\nWORST_END=%s\n" \
    $routed_dcp $dcp_sha256 [llength $source_pins] \
    [llength $destination_pins] [llength $paths] $max_datapath_ns \
    $worst_datapath_ns $worst_start $worst_end]

if {[llength $failures] == 0} {
    append summary "RESULT=PASS\n"
    cyan_sgmii_rx_write $summary_path $summary
    close_design
    puts "PASS SGMII receive-gearbox routed signoff; worst=[format %.3f $worst_datapath_ns] ns limit=[format %.3f $max_datapath_ns] ns"
    exit 0
}

append summary "RESULT=FAIL\n"
foreach failure $failures {
    append summary "BLOCKER=$failure\n"
}
cyan_sgmii_rx_write $summary_path $summary
close_design
puts stderr "FAIL SGMII receive-gearbox routed signoff: [llength $failures] blocker(s)"
foreach failure $failures {
    puts stderr "  $failure"
}
exit 1
