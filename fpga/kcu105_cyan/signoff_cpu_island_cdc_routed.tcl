# Read-only physical signoff for the transactional CPU/cache-island boundary.
#
# Usage:
#   vivado -mode batch -source signoff_cpu_island_cdc_routed.tcl \
#       -tclargs ROUTED_DCP REPORT_DIR
#   tclsh signoff_cpu_island_cdc_routed.tcl --syntax-check

proc cyan_cpu_signoff_usage {} {
    puts stderr "usage: vivado -mode batch -source signoff_cpu_island_cdc_routed.tcl -tclargs ROUTED_DCP REPORT_DIR"
}

set ::cyan_cpu_signoff_failures {}

proc cyan_cpu_signoff_read_file {path} {
    if {![file exists $path] || [file isdirectory $path]} {
        cyan_cpu_signoff_blocker "required report is missing: $path"
        return ""
    }
    if {[file size $path] == 0} {
        cyan_cpu_signoff_blocker "required report is empty: $path"
        return ""
    }
    if {[catch {
        set fd [open $path r]
        set contents [read $fd]
        close $fd
    } error_text]} {
        cyan_cpu_signoff_blocker "cannot read required report $path: $error_text"
        return ""
    }
    return $contents
}

proc cyan_cpu_signoff_write_file {path contents} {
    set fd [open $path w]
    puts -nonewline $fd $contents
    close $fd
}

proc cyan_cpu_signoff_blocker {message} {
    lappend ::cyan_cpu_signoff_failures $message
    puts stderr "BLOCKER: $message"
}

proc cyan_cpu_signoff_report {label command} {
    puts "Running $label"
    if {[catch {uplevel #0 $command} error_text]} {
        cyan_cpu_signoff_blocker "$label failed: $error_text"
    }
}

proc cyan_cpu_signoff_require_clean_text {label text required_patterns} {
    if {$text eq ""} {
        return
    }
    foreach pattern $required_patterns {
        if {![regexp -nocase -- $pattern $text]} {
            cyan_cpu_signoff_blocker \
                "$label lacks required evidence matching '$pattern'"
        }
    }
    foreach pattern {
        {Slack[[:space:]]*\(VIOLATED\)}
        {(^|\n)[[:space:]]*(CRITICAL WARNING|ERROR)[[:space:]]*:}
    } {
        if {[regexp -nocase -- $pattern $text match]} {
            cyan_cpu_signoff_blocker \
                "$label contains blocking report text: [string trim $match]"
        }
    }
}

proc cyan_cpu_signoff_require_safe_timed_cdc {label text} {
    if {$text eq ""} {
        return
    }
    if {![regexp -nocase -- \
        {(^|\n)[[:space:]]*CDC Report[[:space:]]*($|\n)} $text]} {
        cyan_cpu_signoff_blocker "$label lacks the Vivado CDC Report header"
    }
    if {![regexp -nocase -- \
        {(^|\n)[[:space:]]*All paths are Safely Timed\.[[:space:]]*($|\n)} \
        $text]} {
        cyan_cpu_signoff_blocker \
            "$label lacks Vivado's exact related-clock safe-timed result"
        return
    }
    if {[regexp -nocase -- {CDC-[0-9]+} $text match]} {
        cyan_cpu_signoff_blocker \
            "$label contains an unexpected CDC finding: $match"
    }
    if {[regexp -nocase -- \
        {(^|\n)[[:space:]]*(CRITICAL WARNING|WARNING|ERROR)[[:space:]]*:} \
        $text match]} {
        cyan_cpu_signoff_blocker \
            "$label contains blocking report text: [string trim $match]"
    }
}

proc cyan_cpu_signoff_require_clean_check_timing {text} {
    if {$text eq ""} {
        return
    }
    # Board-level pulse-width and missing input/output-delay categories have
    # explicit, pre-existing allowances.  Internal clocks, constraints, and
    # combinational/latch loops do not.
    foreach category {
        no_clock constant_clock unconstrained_internal_endpoints
        multiple_clock generated_clocks loops partial_input_delay
        partial_output_delay latch_loops
    } {
        set occurrences 0
        foreach line [split $text "\n"] {
            set pattern [format \
                {^[[:space:]]*[0-9]+\.[[:space:]]+checking[[:space:]]+%s[[:space:]]+\(([0-9]+)\)[[:space:]]*$} \
                $category]
            if {[regexp -- $pattern $line match count]} {
                incr occurrences
                if {$count != 0} {
                    cyan_cpu_signoff_blocker \
                        "check_timing category $category has count $count, expected 0"
                }
            }
        }
        if {$occurrences == 0} {
            cyan_cpu_signoff_blocker \
                "check_timing lacks category $category"
        }
    }
}

proc cyan_cpu_signoff_require_async {label cells} {
    foreach cell $cells {
        set async_reg [get_property ASYNC_REG $cell]
        if {$async_reg ne "1" &&
            ![string equal -nocase $async_reg "true"] &&
            ![string equal -nocase $async_reg "yes"]} {
            cyan_cpu_signoff_blocker \
                "$label cell $cell has ASYNC_REG='$async_reg', expected TRUE"
        }
    }
}

proc cyan_cpu_signoff_require_static_low {label cells} {
    if {[llength $cells] != 1} {
        cyan_cpu_signoff_blocker \
            "$label resolved to [llength $cells] cells, expected exactly 1"
        return
    }
    set d_pins [cyan_cpu_bridge_pins "$label D" $cells D]
    set d_pin [lindex $d_pins 0]
    set is_tied [get_property IS_TIED $d_pin]
    set logic_value [get_property LOGIC_VALUE $d_pin]
    if {$is_tied ne "1" || ![string equal -nocase $logic_value "zero"]} {
        cyan_cpu_signoff_blocker \
            "$label D pin is not tied low: IS_TIED=$is_tied LOGIC_VALUE=$logic_value"
        return
    }
    set nets [get_nets -quiet -of_objects $d_pin]
    if {[llength $nets] != 1} {
        cyan_cpu_signoff_blocker \
            "$label D pin has [llength $nets] nets, expected exactly 1: $nets"
        return
    }
    set net [lindex $nets 0]
    set net_type [get_property TYPE $net]
    if {![string equal -nocase $net_type "GROUND"]} {
        cyan_cpu_signoff_blocker \
            "$label net $net has TYPE=$net_type, expected GROUND"
        return
    }
    set drivers [get_pins -quiet -of_objects $net -filter {DIRECTION == OUT}]
    if {[llength $drivers] != 1} {
        cyan_cpu_signoff_blocker \
            "$label net $net has [llength $drivers] output drivers, expected exactly 1: $drivers"
        return
    }
    set driver_cells [get_cells -quiet -of_objects [lindex $drivers 0]]
    if {[llength $driver_cells] != 1 ||
        [get_property REF_NAME [lindex $driver_cells 0]] ne "GND"} {
        cyan_cpu_signoff_blocker \
            "$label driver is not exactly one GND cell: $driver_cells"
    }
}

proc cyan_cpu_signoff_require_paths {
    label source_cells destination_cells expected_count report_path
} {
    if {[catch {
        set paths [get_timing_paths -quiet -delay_type max \
            -from $source_cells -to $destination_cells \
            -max_paths $expected_count -nworst 1]
    } error_text]} {
        cyan_cpu_signoff_blocker "$label timing-path query failed: $error_text"
        return
    }
    if {[llength $paths] != $expected_count} {
        cyan_cpu_signoff_blocker \
            "$label has [llength $paths] timed paths, expected $expected_count; a broad clock cut or missing bundle constraint may exist"
    }
    foreach path $paths {
        set slack [get_property SLACK $path]
        if {$slack < 0.0} {
            cyan_cpu_signoff_blocker \
                [format "%s has negative routed slack %.3f ns: %s -> %s" \
                    $label $slack [get_property STARTPOINT_PIN $path] \
                    [get_property ENDPOINT_PIN $path]]
        }
    }
    cyan_cpu_signoff_report "$label routed bundle timing" \
        [list report_timing -delay_type max -from $source_cells \
            -to $destination_cells -max_paths $expected_count -nworst 1 \
            -file $report_path]
}

proc cyan_cpu_signoff_require_ignored {
    label source_clocks destination_cells expected_count report_path
} {
    set destination_d \
        [cyan_cpu_bridge_pins "$label destination" $destination_cells D]
    if {[catch {
        set paths [get_timing_paths -quiet -user_ignored -delay_type max \
            -from $source_clocks -to $destination_d \
            -max_paths $expected_count -nworst 1]
    } error_text]} {
        cyan_cpu_signoff_blocker "$label ignored-path query failed: $error_text"
        return
    }
    if {[llength $paths] != $expected_count} {
        cyan_cpu_signoff_blocker \
            "$label has [llength $paths] ignored first-stage paths, expected exactly $expected_count"
    }
    cyan_cpu_signoff_report "$label ignored first-stage timing" \
        [list report_timing -user_ignored -delay_type max \
            -from $source_clocks -to $destination_d \
            -max_paths $expected_count -nworst 1 -file $report_path]
}

proc cyan_cpu_signoff_require_clock_timing {label clocks delay_type report_path} {
    if {[catch {
        set paths [get_timing_paths -quiet -delay_type $delay_type \
            -from $clocks -to $clocks -max_paths 1 -nworst 1]
    } error_text]} {
        cyan_cpu_signoff_blocker "$label $delay_type query failed: $error_text"
        return
    }
    if {[llength $paths] != 1} {
        cyan_cpu_signoff_blocker \
            "$label $delay_type returned [llength $paths] paths, expected 1"
        return
    }
    set slack [get_property SLACK [lindex $paths 0]]
    if {$slack < 0.0} {
        cyan_cpu_signoff_blocker \
            [format "%s %s worst slack is %.3f ns" $label $delay_type $slack]
    }
    cyan_cpu_signoff_report "$label $delay_type timing" \
        [list report_timing -delay_type $delay_type -from $clocks \
            -to $clocks -max_paths 20 -nworst 1 -file $report_path]
}

if {$argc == 1 && [lindex $argv 0] eq "--syntax-check"} {
    puts "PASS signoff_cpu_island_cdc_routed.tcl Tcl syntax load"
    exit 0
}
if {$argc != 2} {
    cyan_cpu_signoff_usage
    exit 2
}

set routed_dcp [file normalize [lindex $argv 0]]
set report_dir [file normalize [lindex $argv 1]]
if {![file exists $routed_dcp]} {
    puts stderr "routed checkpoint does not exist: $routed_dcp"
    exit 2
}
file mkdir $report_dir
set pass_manifest [file join $report_dir CPU_ISLAND_CDC_PASS.txt]
file delete -force $pass_manifest
open_checkpoint $routed_dcp

set script_dir [file dirname [file normalize [info script]]]
source [file join $script_dir cpu_island_cdc_constraints.tcl]

cyan_cpu_signoff_require_async cpu_to_machine_first_stage \
    $cyan_cpu_to_machine_sync
cyan_cpu_signoff_require_async machine_to_cpu_first_stage \
    $cyan_machine_to_cpu_sync
cyan_cpu_signoff_require_static_low machine_to_cpu_force_uncached \
    $cyan_machine_to_cpu_static_sync
cyan_cpu_bridge_require_absent_cells machine_to_cpu_internal_timer_stage1 \
    [list {.*u_cpu_island_bridge/cpu_irq_sync1_reg\[5\]}]

set cyan_bridge_cpu_to_machine_sync2 [cyan_cpu_bridge_require_cells \
    bridge_cpu_to_machine_second_stage [list \
        {.*u_cpu_island_bridge/machine_req_sync2_reg} \
        {.*u_cpu_island_bridge/machine_wr_ack_sync2_reg} \
        {.*u_cpu_island_bridge/machine_rd_ack_sync2_reg} \
        {.*u_cpu_island_bridge/machine_core_hold_ack_sync2_reg} \
        {.*u_cpu_island_bridge/machine_dmem_fence_ack_sync2_reg}] 5]
set cyan_cpu_to_machine_sync2 $cyan_bridge_cpu_to_machine_sync2
set cyan_machine_to_cpu_sync2 [cyan_cpu_bridge_require_cells \
    machine_to_cpu_second_stage [list \
        {.*u_cpu_island_bridge/cpu_resp_sync2_reg} \
        {.*u_cpu_island_bridge/cpu_wr_req_sync2_reg} \
        {.*u_cpu_island_bridge/cpu_rd_req_sync2_reg} \
        {.*u_cpu_island_bridge/cpu_core_hold_sync2_reg} \
        {.*u_cpu_island_bridge/cpu_dmem_fence_sync2_reg} \
        {.*u_cpu_island_bridge/cpu_force_uncached_sync2_reg} \
        {.*u_cpu_island_bridge/cpu_irq_sync2_reg\[[0-4]\]}] 11]
set cyan_machine_to_cpu_interstage_width \
    [llength $cyan_machine_to_cpu_sync2]
cyan_cpu_bridge_require_absent_cells machine_to_cpu_internal_timer_stage2 \
    [list {.*u_cpu_island_bridge/cpu_irq_sync2_reg\[5\]}]
set cyan_cpu_reset_sync_all [cyan_cpu_bridge_require_cells \
    cpu_reset_deassertion_sync \
    [list {.*cpu_cdc_reset_sync_reg\[[0-2]\]}] 3]
cyan_cpu_signoff_require_async cpu_to_machine_second_stage \
    $cyan_cpu_to_machine_sync2
cyan_cpu_signoff_require_async machine_to_cpu_second_stage \
    $cyan_machine_to_cpu_sync2
cyan_cpu_signoff_require_async cpu_reset_deassertion_sync \
    $cyan_cpu_reset_sync_all

cyan_cpu_signoff_require_ignored cpu_to_machine_first_stage \
    $cyan_island_clocks $cyan_cpu_to_machine_sync 5 \
    [file join $report_dir cpu_to_machine_first_stage_ignored.rpt]
cyan_cpu_signoff_require_ignored machine_to_cpu_first_stage \
    $cyan_machine_clocks $cyan_machine_to_cpu_dynamic_sync 10 \
    [file join $report_dir machine_to_cpu_first_stage_ignored.rpt]

cyan_cpu_signoff_require_paths cpu_to_machine_interstage \
    $cyan_cpu_to_machine_sync $cyan_cpu_to_machine_sync2 5 \
    [file join $report_dir cpu_to_machine_interstage.rpt]
cyan_cpu_signoff_require_paths machine_to_cpu_interstage \
    $cyan_machine_to_cpu_sync $cyan_machine_to_cpu_sync2 \
    $cyan_machine_to_cpu_interstage_width \
    [file join $report_dir machine_to_cpu_interstage.rpt]
set cyan_cpu_reset_sync0 [cyan_cpu_bridge_require_cells \
    cpu_reset_deassertion_stage0 \
    [list {.*cpu_cdc_reset_sync_reg\[0\]}] 1]
set cyan_cpu_reset_sync1 [cyan_cpu_bridge_require_cells \
    cpu_reset_deassertion_stage1 \
    [list {.*cpu_cdc_reset_sync_reg\[1\]}] 1]
set cyan_cpu_reset_sync_stage2 [cyan_cpu_bridge_require_cells \
    cpu_reset_deassertion_stage2 \
    [list {.*cpu_cdc_reset_sync_reg\[2\]}] 1]
cyan_cpu_signoff_require_paths cpu_reset_deassertion_stage0_to_1 \
    $cyan_cpu_reset_sync0 $cyan_cpu_reset_sync1 1 \
    [file join $report_dir cpu_reset_deassertion_stage0_to_1.rpt]
cyan_cpu_signoff_require_paths cpu_reset_deassertion_stage1_to_2 \
    $cyan_cpu_reset_sync1 $cyan_cpu_reset_sync_stage2 1 \
    [file join $report_dir cpu_reset_deassertion_stage1_to_2.rpt]

cyan_cpu_signoff_require_paths cpu_request \
    $cyan_cpu_req_source $cyan_machine_req_destination \
    $cyan_cpu_request_width \
    [file join $report_dir cpu_request_bundle.rpt]
cyan_cpu_signoff_require_paths cpu_response \
    $cyan_machine_resp_source $cyan_cpu_resp_destination 64 \
    [file join $report_dir cpu_response_bundle.rpt]
cyan_cpu_signoff_require_paths dma_write_snoop \
    $cyan_machine_wr_source $cyan_cpu_wr_destination 101 \
    [file join $report_dir dma_write_snoop_bundle.rpt]
cyan_cpu_signoff_require_paths dma_read_probe \
    $cyan_machine_rd_source $cyan_cpu_rd_destination 29 \
    [file join $report_dir dma_read_probe_bundle.rpt]
cyan_cpu_signoff_require_paths dma_read_response \
    $cyan_cpu_rd_resp_source $cyan_machine_rd_resp_destination 65 \
    [file join $report_dir dma_read_response_bundle.rpt]

cyan_cpu_signoff_require_clock_timing machine $cyan_machine_clocks max \
    [file join $report_dir machine_setup.rpt]
cyan_cpu_signoff_require_clock_timing machine $cyan_machine_clocks min \
    [file join $report_dir machine_hold.rpt]
cyan_cpu_signoff_require_clock_timing cpu_island $cyan_island_clocks max \
    [file join $report_dir cpu_island_setup.rpt]
cyan_cpu_signoff_require_clock_timing cpu_island $cyan_island_clocks min \
    [file join $report_dir cpu_island_hold.rpt]

cyan_cpu_signoff_report "check_timing -verbose" \
    [list check_timing -verbose -file [file join $report_dir check_timing.rpt]]
cyan_cpu_signoff_report "clock interaction" \
    [list report_clock_interaction -delay_type min_max \
        -significant_digits 3 -file \
        [file join $report_dir clock_interaction.rpt]]
cyan_cpu_signoff_report "ignored exceptions" \
    [list report_exceptions -ignored -file \
        [file join $report_dir exceptions_ignored.rpt]]
cyan_cpu_signoff_report "exception coverage" \
    [list report_exceptions -coverage -file \
        [file join $report_dir exceptions_coverage.rpt]]
cyan_cpu_signoff_report "machine-to-CPU directed CDC" \
    [list report_cdc -details -all_checks_per_endpoint -no_waiver \
        -from $cyan_machine_clocks -to $cyan_island_clocks \
        -file [file join $report_dir cdc_machine_to_cpu.rpt]]
cyan_cpu_signoff_report "CPU-to-machine directed CDC" \
    [list report_cdc -details -all_checks_per_endpoint -no_waiver \
        -from $cyan_island_clocks -to $cyan_machine_clocks \
        -file [file join $report_dir cdc_cpu_to_machine.rpt]]
cyan_cpu_signoff_report "bundle bus skew" \
    [list report_bus_skew -delay_type min_max -file \
        [file join $report_dir bundle_bus_skew.rpt]]
cyan_cpu_signoff_report "timing summary" \
    [list report_timing_summary -delay_type min_max -check_timing_verbose \
        -max_paths 20 -file [file join $report_dir timing_summary.rpt]]

set check_timing_text [cyan_cpu_signoff_read_file \
    [file join $report_dir check_timing.rpt]]
cyan_cpu_signoff_require_clean_text check_timing $check_timing_text \
    [list {check_timing}]
cyan_cpu_signoff_require_clean_check_timing $check_timing_text

set timing_summary_text [cyan_cpu_signoff_read_file \
    [file join $report_dir timing_summary.rpt]]
cyan_cpu_signoff_require_clean_text timing_summary $timing_summary_text \
    [list {Design Timing Summary} \
        {All user specified timing constraints are met\.}]

set bundle_bus_skew_text [cyan_cpu_signoff_read_file \
    [file join $report_dir bundle_bus_skew.rpt]]
cyan_cpu_signoff_require_clean_text bundle_bus_skew $bundle_bus_skew_text \
    [list {Slack[[:space:]]*\(MET\)}]
set bundle_met_count [regexp -all -nocase \
    {Slack[[:space:]]*\(MET\)} $bundle_bus_skew_text]
if {$bundle_met_count < 5} {
    cyan_cpu_signoff_blocker \
        "bundle bus-skew report has $bundle_met_count met constraints, expected at least 5"
}
foreach bundle_anchor {
    cpu_req_paddr_q_reg machine_resp_data_q_reg machine_wr_addr_q_reg
    machine_rd_addr_q_reg cpu_rd_resp_data_q_reg
} {
    if {![string match "*$bundle_anchor*" $bundle_bus_skew_text]} {
        cyan_cpu_signoff_blocker \
            "bundle bus-skew report lacks $bundle_anchor evidence"
    }
}

set cdc_machine_to_cpu_text [cyan_cpu_signoff_read_file \
    [file join $report_dir cdc_machine_to_cpu.rpt]]
cyan_cpu_signoff_require_safe_timed_cdc machine_to_cpu \
    $cdc_machine_to_cpu_text
set cdc_cpu_to_machine_text [cyan_cpu_signoff_read_file \
    [file join $report_dir cdc_cpu_to_machine.rpt]]
cyan_cpu_signoff_require_safe_timed_cdc cpu_to_machine \
    $cdc_cpu_to_machine_text
# The first routed gate is intentionally strict.  Known mailbox topology is
# proven above by exact cells, first-stage cuts, interstage timing, and held
# bundle constraints.  Anything Vivado still classifies as Warning/Critical
# must be reviewed and admitted by exact endpoints, never by a broad waiver.
foreach required_report {
    clock_interaction.rpt exceptions_ignored.rpt exceptions_coverage.rpt
    cpu_to_machine_first_stage_ignored.rpt
    machine_to_cpu_first_stage_ignored.rpt
    cpu_to_machine_interstage.rpt machine_to_cpu_interstage.rpt
    cpu_reset_deassertion_stage0_to_1.rpt
    cpu_reset_deassertion_stage1_to_2.rpt
    cpu_request_bundle.rpt cpu_response_bundle.rpt
    dma_write_snoop_bundle.rpt dma_read_probe_bundle.rpt
    dma_read_response_bundle.rpt machine_setup.rpt machine_hold.rpt
    cpu_island_setup.rpt cpu_island_hold.rpt
} {
    cyan_cpu_signoff_read_file [file join $report_dir $required_report]
}

close_design
if {[llength $::cyan_cpu_signoff_failures] != 0} {
    puts stderr "FAIL CPU-island routed signoff: [llength $::cyan_cpu_signoff_failures] blocker(s)"
    foreach failure $::cyan_cpu_signoff_failures {
        puts stderr " - $failure"
    }
    exit 1
}
if {[catch {exec sha256sum -- $routed_dcp} dcp_sha_result]} {
    puts stderr "FAIL CPU-island routed signoff: cannot hash routed DCP: $dcp_sha_result"
    exit 1
}
set dcp_sha256 [lindex $dcp_sha_result 0]
set manifest_text "status=PASS\n"
append manifest_text "routed_dcp_sha256=$dcp_sha256\n"
append manifest_text "machine_period_ns=$cyan_machine_period\n"
append manifest_text "cpu_island_period_ns=$cyan_island_period\n"
append manifest_text "cpu_to_machine_first_stage_cells=5\n"
append manifest_text "cpu_to_machine_first_stage_ignored_paths=5\n"
append manifest_text \
    "machine_to_cpu_first_stage_cells=$cyan_machine_to_cpu_first_stage_width\n"
append manifest_text "machine_to_cpu_first_stage_ignored_paths=10\n"
append manifest_text "machine_to_cpu_first_stage_static_low=1\n"
append manifest_text "cpu_to_machine_interstage_paths=5\n"
append manifest_text \
    "machine_to_cpu_interstage_paths=$cyan_machine_to_cpu_interstage_width\n"
append manifest_text "cpu_request_bundle_bits=$cyan_cpu_request_width\n"
append manifest_text "cpu_response_bundle_bits=64\n"
append manifest_text "dma_write_snoop_bundle_bits=101\n"
append manifest_text "dma_read_probe_bundle_bits=29\n"
append manifest_text "dma_read_response_bundle_bits=65\n"
append manifest_text "cpu_reset_sync_stages=3\n"
append manifest_text "held_bundle_constraints=5\n"
append manifest_text "cdc_unreviewed=0\n"
append manifest_text "cdc_report_mode=related-clock-safe-timed\n"
append manifest_text "cdc_directed_reports=2\n"
append manifest_text "cdc_findings=0\n"
cyan_cpu_signoff_write_file $pass_manifest $manifest_text
puts "PASS CPU-island routed structural/timing/CDC signoff; manifest=$pass_manifest DCP_SHA256=$dcp_sha256"
exit 0
