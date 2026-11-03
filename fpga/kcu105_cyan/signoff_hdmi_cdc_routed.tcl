# Read-only physical signoff for the CYAN HDMI completed-bank and ownership
# mailboxes plus the video-alive scan-epoch synchronizer.
#
# Usage:
#   vivado -mode batch -source signoff_hdmi_cdc_routed.tcl \
#       -tclargs ROUTED_DCP REPORT_DIR
#
# The checkpoint is opened for reporting only.  This script deliberately has
# no write_checkpoint, write_bitstream, write_debug_probes, project, run, or
# hardware-manager command.  The two set_bus_skew commands are temporary
# in-memory analysis constraints and disappear when the design is closed.

proc usage {} {
    puts stderr "usage: vivado -mode batch -source signoff_hdmi_cdc_routed.tcl -tclargs ROUTED_DCP REPORT_DIR"
    puts stderr "       tclsh signoff_hdmi_cdc_routed.tcl --syntax-check"
}

set ::cyan_failures {}
set ::cyan_summary_fd ""

proc note {message} {
    puts $message
    if {$::cyan_summary_fd ne ""} {
        puts $::cyan_summary_fd $message
        flush $::cyan_summary_fd
    }
}

proc blocker {message} {
    lappend ::cyan_failures $message
    puts stderr "BLOCKER: $message"
    if {$::cyan_summary_fd ne ""} {
        puts $::cyan_summary_fd "BLOCKER: $message"
        flush $::cyan_summary_fd
    }
}

proc write_text_file {path contents} {
    if {[catch {
        set fd [open $path w]
        puts -nonewline $fd $contents
        close $fd
    } error_text]} {
        blocker "cannot write report $path: $error_text"
        return 0
    }
    return 1
}

proc run_required_report {label command} {
    note "Running $label"
    if {[catch {uplevel #0 $command} error_text]} {
        blocker "$label is unsupported or failed: $error_text"
        return 0
    }
    return 1
}

proc require_clock {clock_name} {
    if {[catch {set clocks [get_clocks -quiet $clock_name]} error_text]} {
        blocker "cannot query clock $clock_name: $error_text"
        return {}
    }
    if {[llength $clocks] != 1} {
        blocker "clock $clock_name resolved to [llength $clocks] objects, expected exactly 1: $clocks"
        return {}
    }
    note "Clock $clock_name: [lindex $clocks 0]"
    return $clocks
}

proc require_cell_group {label base_name expected_count require_async_reg} {
    set exact_re [format {^.*u_hdmi_frame_resampler/%s(\[[0-9]+\])?$} $base_name]
    if {[catch {
        set cells [lsort -dictionary \
            [get_cells -quiet -hierarchical -regexp \
                -include_replicated_objects $exact_re]]
    } error_text]} {
        blocker "cannot query $label ($base_name): $error_text"
        return {}
    }

    # A replicated, renamed, or otherwise ambiguous implementation must not
    # hide behind the exact-name count.
    set broad_filter "NAME =~ *u_hdmi_frame_resampler/*${base_name}*"
    if {[catch {
        set broad_cells [lsort -dictionary \
            [get_cells -quiet -hierarchical -include_replicated_objects \
                -filter $broad_filter *]]
    } error_text]} {
        blocker "cannot perform ambiguity query for $label: $error_text"
        return {}
    }

    if {[llength $cells] != $expected_count} {
        blocker "$label resolved to [llength $cells] exact cells, expected $expected_count: $cells"
    }
    if {[llength $broad_cells] != $expected_count} {
        blocker "$label has [llength $broad_cells] broad-name cells, expected exactly $expected_count (replication or ambiguity): $broad_cells"
    }

    foreach cell $cells {
        if {[catch {set ref_name [get_property REF_NAME $cell]} error_text]} {
            blocker "cannot read REF_NAME for $cell: $error_text"
            continue
        }
        if {![regexp {^FD} $ref_name]} {
            blocker "$cell has REF_NAME=$ref_name, expected a physical FD* flip-flop (not LUT/SRL)"
        }
        if {$require_async_reg} {
            if {[catch {set async_reg [get_property ASYNC_REG $cell]} error_text]} {
                blocker "cannot read ASYNC_REG for $cell: $error_text"
                continue
            }
            if {$async_reg ne "1" &&
                ![string equal -nocase $async_reg "true"] &&
                ![string equal -nocase $async_reg "yes"]} {
                blocker "$cell has ASYNC_REG='$async_reg', expected TRUE"
            }
        }
    }

    note "$label: [llength $cells] cells: $cells"
    return $cells
}

proc require_group_clock {label cells expected_clock_name} {
    foreach cell $cells {
        if {[catch {
            set clock_pins [get_pins -quiet -of_objects $cell \
                -filter {REF_PIN_NAME == C}]
        } error_text]} {
            blocker "cannot query clock pin for $cell: $error_text"
            continue
        }
        if {[llength $clock_pins] != 1} {
            blocker "$cell has [llength $clock_pins] C pins, expected 1: $clock_pins"
            continue
        }
        if {[catch {
            set clocks [get_clocks -quiet -of_objects $clock_pins]
        } error_text]} {
            blocker "cannot query clock on $cell: $error_text"
            continue
        }
        if {[llength $clocks] != 1} {
            blocker "$cell is reached by [llength $clocks] clocks, expected exactly 1: $clocks"
            continue
        }
        set actual_name [get_property NAME [lindex $clocks 0]]
        if {$actual_name ne $expected_clock_name} {
            blocker "$label cell $cell is clocked by $actual_name, expected $expected_clock_name"
        }
    }
}

proc require_single_direct_load {label source_cell allowed_cells} {
    if {[catch {
        set q_pins [get_pins -quiet -of_objects $source_cell \
            -filter {REF_PIN_NAME == Q}]
    } error_text]} {
        blocker "cannot query Q pin for $label/$source_cell: $error_text"
        return
    }
    if {[llength $q_pins] != 1} {
        blocker "$label source $source_cell has [llength $q_pins] Q pins, expected 1: $q_pins"
        return
    }
    if {[catch {
        set net_segments [get_nets -quiet -segments -of_objects $q_pins]
        set load_pins [lsort -unique [get_pins -quiet -leaf \
            -of_objects $net_segments -filter {DIRECTION == IN}]]
        set load_cells [lsort -unique [get_cells -quiet -of_objects $load_pins]]
    } error_text]} {
        blocker "cannot inspect first-stage fanout for $label/$source_cell: $error_text"
        return
    }
    if {[llength $load_pins] != 1 || [llength $load_cells] != 1} {
        blocker "$label source $source_cell has [llength $load_pins] leaf loads in [llength $load_cells] cells, expected exactly one: pins=$load_pins cells=$load_cells"
        return
    }

    set allowed_names {}
    foreach cell $allowed_cells {
        lappend allowed_names [get_property NAME $cell]
    }
    set load_name [get_property NAME [lindex $load_cells 0]]
    set load_ref_pin [get_property REF_PIN_NAME [lindex $load_pins 0]]
    if {$load_ref_pin ne "D"} {
        blocker "$label source $source_cell drives $load_name/$load_ref_pin, expected the D pin"
        return
    }
    if {[lsearch -exact $allowed_names $load_name] < 0} {
        blocker "$label source $source_cell drives $load_name, expected one of: $allowed_names"
        return
    }
    note "$label single direct load: $source_cell -> $load_name"
}

proc require_vector_direct_loads {label source_cells destination_cells} {
    if {[llength $source_cells] != [llength $destination_cells]} {
        blocker "$label has [llength $source_cells] source cells and [llength $destination_cells] destination cells"
        return
    }
    for {set index 0} {$index < [llength $source_cells]} {incr index} {
        set source_cell [lindex $source_cells $index]
        set destination_cell [lindex $destination_cells $index]
        set source_name [get_property NAME $source_cell]
        set destination_name [get_property NAME $destination_cell]
        if {![regexp {\[([0-9]+)\]$} $source_name unused source_bit] ||
            ![regexp {\[([0-9]+)\]$} $destination_name unused destination_bit]} {
            blocker "$label cannot prove vector indices for $source_name -> $destination_name"
            continue
        }
        if {$source_bit != $destination_bit} {
            blocker "$label index mismatch $source_name -> $destination_name"
            continue
        }
        require_single_direct_load $label $source_cell [list $destination_cell]
    }
}

proc check_timing_path_set {label from_cells to_cells expected_count report_path} {
    set query_ok 1
    foreach delay_type {max min} {
        if {[catch {
            set paths [get_timing_paths -quiet -delay_type $delay_type \
                -from $from_cells -to $to_cells -max_paths 64 -nworst 1]
        } error_text]} {
            blocker "$label $delay_type interstage timing query is unsupported or failed: $error_text"
            set query_ok 0
            continue
        }
        if {[llength $paths] != $expected_count} {
            blocker "$label $delay_type interstage query returned [llength $paths] paths, expected $expected_count"
            set query_ok 0
        }
        foreach path $paths {
            if {[catch {set slack [get_property SLACK $path]} error_text]} {
                blocker "$label cannot read $delay_type path slack: $error_text"
                set query_ok 0
                continue
            }
            if {$slack < 0.0} {
                blocker "$label has negative $delay_type interstage slack $slack on $path"
                set query_ok 0
            }
        }
    }

    set report_cmd [list report_timing -delay_type min_max -path_type full \
        -input_pins -from $from_cells -to $to_cells -max_paths 64 \
        -nworst 1 -file $report_path]
    if {![run_required_report "$label interstage timing report" $report_cmd]} {
        set query_ok 0
    }
    return $query_ok
}

proc check_ignored_delay {label from_cells to_cells expected_count limit_ns report_path} {
    if {[catch {
        set paths [get_timing_paths -quiet -user_ignored -delay_type max \
            -from $from_cells -to $to_cells -max_paths 64 -nworst 1]
    } error_text]} {
        blocker "$label ignored-path query is unsupported or failed: $error_text"
        return 0
    }
    if {[llength $paths] != $expected_count} {
        blocker "$label ignored-path query returned [llength $paths] paths, expected $expected_count; the clock cut may be absent or objects may not be direct"
    }

    foreach path $paths {
        if {[catch {
            set delay [get_property DATAPATH_DELAY $path]
            set start_pin [get_property STARTPOINT_PIN $path]
            set end_pin [get_property ENDPOINT_PIN $path]
        } error_text]} {
            blocker "$label cannot read ignored-path properties: $error_text"
            continue
        }
        note [format "%s ignored path %.3f ns (limit %.3f ns): %s -> %s" \
            $label $delay $limit_ns $start_pin $end_pin]
        if {$delay >= $limit_ns} {
            blocker [format "%s ignored path delay %.3f ns is not below %.3f ns: %s -> %s" \
                $label $delay $limit_ns $start_pin $end_pin]
        }
    }

    set report_cmd [list report_timing -user_ignored -delay_type max \
        -path_type full -input_pins -from $from_cells -to $to_cells \
        -max_paths 64 -nworst 1 -file $report_path]
    run_required_report "$label ignored-path timing report" $report_cmd
    return [expr {[llength $paths] == $expected_count}]
}

proc check_global_timing {delay_type} {
    if {[catch {
        set paths [get_timing_paths -quiet -delay_type $delay_type \
            -max_paths 1 -nworst 1]
    } error_text]} {
        blocker "global $delay_type timing query failed: $error_text"
        return
    }
    if {[llength $paths] != 1} {
        blocker "global $delay_type timing returned [llength $paths] paths, expected 1"
        return
    }
    set slack [get_property SLACK [lindex $paths 0]]
    note "Global $delay_type worst slack: $slack ns"
    if {$slack < 0.0} {
        blocker "global $delay_type timing fails with slack $slack ns"
    }
}

proc capture_timing_summary {report_path} {
    if {[catch {
        set report_text [report_timing_summary -return_string \
            -delay_type min_max -warn_on_violation]
    } error_text]} {
        blocker "timing summary is unsupported or failed: $error_text"
        return
    }
    write_text_file $report_path $report_text

    if {[string first "All user specified timing constraints are met." \
            $report_text] < 0} {
        blocker "timing summary does not state that all user timing constraints are met"
    }

    # The first twelve-number row after the Design Timing Summary marker is
    # WNS/TNS/failing/total, WHS/THS/failing/total, and
    # WPWS/TPWS/failing/total.  Requiring all three worst slacks here closes
    # the pulse-width gap left by ordinary max/min timing-path queries.
    set marker [string first "| Design Timing Summary" $report_text]
    if {$marker < 0} {
        blocker "timing summary omits the Design Timing Summary section"
        return
    }
    set summary_tail [string range $report_text $marker end]
    set found_summary 0
    foreach line [split $summary_tail "\n"] {
        set fields [regexp -all -inline -- {-?[0-9]+\.[0-9]+|[0-9]+} $line]
        if {[llength $fields] < 12} {
            continue
        }
        set wns [lindex $fields 0]
        set whs [lindex $fields 4]
        set wpws [lindex $fields 8]
        note "Timing summary slacks: WNS=$wns ns WHS=$whs ns WPWS=$wpws ns"
        if {$wns < 0.0 || $whs < 0.0 || $wpws < 0.0} {
            blocker "timing summary has negative WNS/WHS/WPWS: $wns/$whs/$wpws ns"
        }
        set found_summary 1
        break
    }
    if {!$found_summary} {
        blocker "cannot parse WNS/WHS/WPWS from the Design Timing Summary section"
    }
}

proc capture_drc_report {report_path} {
    if {[catch {
        set report_text [report_drc -return_string -no_waivers]
    } error_text]} {
        blocker "routed DRC is unsupported or failed: $error_text"
        return
    }
    write_text_file $report_path $report_text

    set error_rows [regexp -all -line -- \
        {^\|[^|\n]+\|[[:space:]]*Error[[:space:]]*\|} $report_text]
    set warning_rows [regexp -all -line -- \
        {^\|[^|\n]+\|[[:space:]]*Warning[[:space:]]*\|} $report_text]
    note "DRC summary rows: errors=$error_rows warnings=$warning_rows"
    if {$error_rows != 0} {
        blocker "routed DRC contains $error_rows error-severity rule row(s); see $report_path"
    }
}

proc capture_methodology_report {report_path tracked_rule_ids} {
    if {[catch {
        set report_text [report_methodology -return_string -no_waivers]
    } error_text]} {
        blocker "report_methodology is unsupported or failed: $error_text"
        return
    }
    write_text_file $report_path $report_text

    # These four rules describe the design's existing broad-clock-group
    # constraint debt.  They are not automatically fatal here: this script's
    # exact target-cell, topology, ignored-delay, and bus-skew checks below
    # are the fail-closed test for the new mailbox crossings.  Keep the global
    # debt counts explicit so a reviewer cannot mistake a successful command
    # invocation for a clean methodology report.
    foreach rule_id $tracked_rule_ids {
        set detail_re [format {^%s#[0-9]+[[:space:]]} $rule_id]
        set detail_count [regexp -all -line -- $detail_re $report_text]
        note "Methodology debt $rule_id: $detail_count detailed violation(s); see $report_path"
    }
}

proc apply_conservative_mtbf_activity {first_stage_cells} {
    set applied_nets {}
    foreach cell $first_stage_cells {
        set d_pins [get_pins -quiet -of_objects $cell \
            -filter {REF_PIN_NAME == D}]
        if {[llength $d_pins] != 1} {
            blocker "MTBF target $cell has [llength $d_pins] D pins, expected exactly 1"
            continue
        }
        set input_nets [get_nets -quiet -segments -of_objects $d_pins]
        if {[llength $input_nets] == 0} {
            blocker "MTBF target $cell has no input-net segments"
            continue
        }
        set applied_nets [concat $applied_nets $input_nets]
    }
    set applied_nets [lsort -unique $applied_nets]
    if {[llength $applied_nets] == 0} {
        blocker "no target CDC input nets received analysis switching activity"
        return
    }
    if {[catch {
        # 300 million transitions/second is deliberately conservative for
        # every target event/payload signal and is no lower than the fastest
        # source clock in this design.  This is analysis-only state in the
        # read-only checkpoint session; no XDC, DCP, or bitstream is written.
        set_switching_activity -signal_rate 300 \
            -static_probability 0.5 $applied_nets
    } error_text]} {
        blocker "cannot apply conservative target MTBF activity: $error_text"
        return
    }
    note "Applied analysis-only 300 Mtr/s activity to [llength $applied_nets] target CDC input-net segments"
}

proc capture_mtbf_report {report_path expected_endpoint_stems} {
    if {[catch {
        set report_text [report_synchronizer_mtbf -return_string \
            -report_endpoints -warn_if_mtbf_below 1e12]
    } error_text]} {
        blocker "report_synchronizer_mtbf is unsupported or failed: $error_text"
        return
    }
    write_text_file $report_path $report_text

    # Synthesis may legally rename a source net after folding it into the
    # atomic ownership payload.  Endpoints are the exact ASYNC_REG cells
    # already checked above and remain stable.  Require one report row per
    # endpoint bit/event and a numeric result above the 1e12-year policy.
    foreach stem_spec $expected_endpoint_stems {
        lassign $stem_spec stem expected_occurrences
        set occurrences 0
        foreach line [split $report_text "\n"] {
            if {[string first $stem $line] >= 0} {
                incr occurrences
                set columns [split $line "|"]
                set mtbf_text [string trim [lindex $columns 2]]
                if {[string match -nocase "UNDEFINED*" $mtbf_text]} {
                    blocker "target synchronizer endpoint $stem has UNDEFINED MTBF: $line"
                } elseif {[regexp {^>[[:space:]]} $mtbf_text]} {
                    # Vivado's saturated '> 1.8e+308 years' result is safely
                    # above the admission threshold.
                } elseif {[regexp {^([0-9.]+)e([+-]?[0-9]+)[[:space:]]+years$} \
                              $mtbf_text unused mantissa exponent]} {
                    set mtbf_years [expr {$mantissa * pow(10.0, $exponent)}]
                    if {$mtbf_years < 1.0e12} {
                        blocker "target synchronizer endpoint $stem MTBF $mtbf_text is below 1e12 years"
                    }
                } else {
                    blocker "target synchronizer endpoint $stem has unrecognized MTBF '$mtbf_text': $line"
                }
            }
        }
        if {$occurrences != $expected_occurrences} {
            blocker "MTBF report contains $occurrences rows mentioning endpoint $stem, expected exactly $expected_occurrences"
        } else {
            note "MTBF report coverage for endpoint $stem: $occurrences matching rows"
        }
    }
}

proc capture_temporary_bus_skew_report {
    report_path completed_sources completed_destinations ownership_sources ownership_destinations
} {
    set constraints_ok 1
    if {[catch {
        set_bus_skew -from $completed_sources -to $completed_destinations 3.332
    } error_text]} {
        blocker "temporary completed-bank data/event set_bus_skew failed: $error_text"
        set constraints_ok 0
    }
    if {[catch {
        set_bus_skew -from $ownership_sources -to $ownership_destinations 3.332
    } error_text]} {
        blocker "temporary ownership data/event set_bus_skew failed: $error_text"
        set constraints_ok 0
    }
    if {!$constraints_ok} {
        return
    }

    if {[catch {
        set report_text [report_bus_skew -return_string -delay_type min_max \
            -max_paths 16 -nworst 1 -path_type full -input_pins \
            -warn_on_violation]
    } error_text]} {
        blocker "report_bus_skew is unsupported or failed: $error_text"
        return
    }
    write_text_file $report_path $report_text

    # Extract the per-constraint sections rather than accepting matches from
    # unrelated pre-existing XPM bus-skew constraints in the same design.
    set constraint_blocks {}
    set current_block ""
    foreach line [split $report_text "\n"] {
        if {[regexp {^Id:[[:space:]]+[0-9]+[[:space:]]*$} $line]} {
            if {$current_block ne ""} {
                lappend constraint_blocks $current_block
            }
            set current_block $line
        } elseif {$current_block ne ""} {
            append current_block "\n" $line
        }
    }
    if {$current_block ne ""} {
        lappend constraint_blocks $current_block
    }

    foreach bundle_spec [list \
        [list completed completed_bank_mig_reg \
            {completed_bank_mig_reg completed_toggle_mig_reg completed_bank_video_sync1_reg completed_toggle_video_sync_reg}] \
        [list ownership ownership_payload_mig_reg \
            {ownership_payload_mig_reg ownership_event_toggle_mig_reg ownership_payload_src_sync1_reg ownership_event_src_sync_reg}]] {
        lassign $bundle_spec bundle_label anchor required_stems
        set matching_blocks {}
        foreach block $constraint_blocks {
            if {[string first $anchor $block] >= 0} {
                lappend matching_blocks $block
            }
        }
        if {[llength $matching_blocks] != 1} {
            blocker "temporary $bundle_label bus-skew constraint has [llength $matching_blocks] per-constraint report sections, expected exactly 1"
            continue
        }
        set block [lindex $matching_blocks 0]
        foreach stem $required_stems {
            if {[string first $stem $block] < 0} {
                blocker "temporary $bundle_label bus-skew section omits $stem"
            }
        }
        if {[string first "Requirement: 3.332ns" $block] < 0} {
            blocker "temporary $bundle_label bus-skew section does not show the 3.332ns requirement"
        }
        if {[regexp -nocase {VIOLATED} $block]} {
            blocker "temporary $bundle_label bus-skew constraint has negative slack; see $report_path"
        }
    }

    if {[string first "completed_bank_mig_reg" $report_text] < 0} {
        blocker "temporary bus-skew report does not contain completed_bank_mig_reg; the analysis constraint did not cover the intended paths"
    }
    if {[string first "completed_toggle_mig_reg" $report_text] < 0} {
        blocker "temporary bus-skew report does not contain completed_toggle_mig_reg; the analysis constraint omitted the completion event"
    }
    if {[string first "ownership_payload_mig_reg" $report_text] < 0} {
        blocker "temporary bus-skew report does not contain ownership_payload_mig_reg; the analysis constraint did not cover the intended paths"
    }
    if {[string first "ownership_event_toggle_mig_reg" $report_text] < 0} {
        blocker "temporary bus-skew report does not contain ownership_event_toggle_mig_reg; the analysis constraint omitted the ownership event"
    }
    if {[regexp -nocase {VIOLATED} $report_text]} {
        blocker "one or more routed bus-skew constraints have negative slack; see $report_path"
    }
    note "Temporary 3.332 ns completed-bank and ownership bus-skew analyses emitted to $report_path"
}

# This mode parses every procedure body with ordinary Tcl while avoiding all
# Vivado commands and all design/output access.
if {$argc == 1 && [lindex $argv 0] eq "--syntax-check"} {
    puts "PASS signoff_hdmi_cdc_routed.tcl Tcl syntax load"
    exit 0
}

if {$argc != 2} {
    usage
    exit 2
}

lassign $argv checkpoint_arg report_dir_arg
if {![file exists $checkpoint_arg] || ![file isfile $checkpoint_arg]} {
    puts stderr "ERROR: routed checkpoint does not exist or is not a file: $checkpoint_arg"
    exit 2
}

set checkpoint_path [file normalize $checkpoint_arg]
if {[catch {file mkdir $report_dir_arg} error_text]} {
    puts stderr "ERROR: cannot create report directory $report_dir_arg: $error_text"
    exit 2
}
if {![file isdirectory $report_dir_arg]} {
    puts stderr "ERROR: report path is not a directory: $report_dir_arg"
    exit 2
}
set report_dir [file normalize $report_dir_arg]
set summary_path [file join $report_dir cyan_hdmi_cdc_signoff.txt]
if {[catch {set ::cyan_summary_fd [open $summary_path w]} error_text]} {
    puts stderr "ERROR: cannot create summary $summary_path: $error_text"
    exit 2
}

note "CYAN HDMI CDC routed-checkpoint signoff"
note "Checkpoint: $checkpoint_path"
note "Report directory: $report_dir"
note "Vivado version: [version -short]"
note "Policy: checkpoint is opened read-only; no checkpoint/bitstream/project/run/hardware write command exists in this script"

if {[catch {open_checkpoint $checkpoint_path} error_text]} {
    blocker "cannot open routed checkpoint $checkpoint_path: $error_text"
    close $::cyan_summary_fd
    exit 1
}

set top_cells [get_cells -quiet -hierarchical -regexp \
    {^.*u_hdmi_frame_resampler$}]
if {[llength $top_cells] != 1} {
    blocker "u_hdmi_frame_resampler resolved to [llength $top_cells] cells, expected exactly 1: $top_cells"
}

set source_clocks [require_clock lg1_serial_clk_mmcm]
set mig_clocks [require_clock mmcm_clkout0]
set video_clocks [require_clock video_clk_mmcm]

# Exact synchronizer implementation contract: 30 physical ASYNC_REG flops.
set completed_bank_video_sync1 [require_cell_group \
    completed_bank_video_sync1 completed_bank_video_sync1_reg 2 1]
set completed_bank_video_sync2 [require_cell_group \
    completed_bank_video_sync2 completed_bank_video_sync2_reg 2 1]
set completed_toggle_video_sync [require_cell_group \
    completed_toggle_video_sync completed_toggle_video_sync_reg 3 1]
set ownership_payload_src_sync1 [require_cell_group \
    ownership_payload_src_sync1 ownership_payload_src_sync1_reg 6 1]
set ownership_payload_src_sync2 [require_cell_group \
    ownership_payload_src_sync2 ownership_payload_src_sync2_reg 6 1]
set ownership_event_src_sync [require_cell_group \
    ownership_event_src_sync ownership_event_src_sync_reg 3 1]
set ownership_ack_sync_mig [require_cell_group \
    ownership_ack_sync_mig ownership_ack_sync_mig_reg 2 1]
set video_alive_sync_mig [require_cell_group \
    video_alive_sync_mig video_alive_sync_mig_reg 3 1]
set status_request_src_sync [require_cell_group \
    status_request_src_sync status_request_src_sync_reg 3 1]

set all_sync_cells [concat \
    $completed_bank_video_sync1 $completed_bank_video_sync2 \
    $completed_toggle_video_sync $ownership_payload_src_sync1 \
    $ownership_payload_src_sync2 $ownership_event_src_sync \
    $ownership_ack_sync_mig $video_alive_sync_mig \
    $status_request_src_sync]
set unique_sync_cells [lsort -unique $all_sync_cells]
if {[llength $all_sync_cells] != 30 || [llength $unique_sync_cells] != 30} {
    blocker "synchronizer groups total [llength $all_sync_cells] cells/[llength $unique_sync_cells] unique, expected exactly 30/30"
} else {
    note "PASS exact synchronizer population: 30 ASYNC_REG FD* cells"
}

require_group_clock completed_bank_video_sync1 \
    $completed_bank_video_sync1 video_clk_mmcm
require_group_clock completed_bank_video_sync2 \
    $completed_bank_video_sync2 video_clk_mmcm
require_group_clock completed_toggle_video_sync \
    $completed_toggle_video_sync video_clk_mmcm
require_group_clock ownership_payload_src_sync1 \
    $ownership_payload_src_sync1 lg1_serial_clk_mmcm
require_group_clock ownership_payload_src_sync2 \
    $ownership_payload_src_sync2 lg1_serial_clk_mmcm
require_group_clock ownership_event_src_sync \
    $ownership_event_src_sync lg1_serial_clk_mmcm
require_group_clock ownership_ack_sync_mig \
    $ownership_ack_sync_mig mmcm_clkout0
require_group_clock video_alive_sync_mig \
    $video_alive_sync_mig mmcm_clkout0
require_group_clock status_request_src_sync \
    $status_request_src_sync lg1_serial_clk_mmcm

# Sorted vector cell names put [0], [1], [2] in stage order.  Enforce direct
# one-load first stages so physical optimization cannot create metastable
# fanout or bypass a stage.
if {[llength $completed_bank_video_sync1] == 2 &&
    [llength $completed_bank_video_sync2] == 2} {
    require_vector_direct_loads completed_bank_first_stage \
        $completed_bank_video_sync1 $completed_bank_video_sync2
}
if {[llength $completed_toggle_video_sync] == 3} {
    require_single_direct_load completed_toggle_stage0 \
        [lindex $completed_toggle_video_sync 0] \
        [list [lindex $completed_toggle_video_sync 1]]
    require_single_direct_load completed_toggle_stage1 \
        [lindex $completed_toggle_video_sync 1] \
        [list [lindex $completed_toggle_video_sync 2]]
}
if {[llength $ownership_payload_src_sync1] == 6 &&
    [llength $ownership_payload_src_sync2] == 6} {
    require_vector_direct_loads ownership_payload_first_stage \
        $ownership_payload_src_sync1 $ownership_payload_src_sync2
}
if {[llength $ownership_event_src_sync] == 3} {
    require_single_direct_load ownership_event_stage0 \
        [lindex $ownership_event_src_sync 0] \
        [list [lindex $ownership_event_src_sync 1]]
    require_single_direct_load ownership_event_stage1 \
        [lindex $ownership_event_src_sync 1] \
        [list [lindex $ownership_event_src_sync 2]]
}
if {[llength $ownership_ack_sync_mig] == 2} {
    require_single_direct_load ownership_ack_stage0 \
        [lindex $ownership_ack_sync_mig 0] \
        [list [lindex $ownership_ack_sync_mig 1]]
}
if {[llength $video_alive_sync_mig] == 3} {
    require_single_direct_load video_alive_stage0 \
        [lindex $video_alive_sync_mig 0] \
        [list [lindex $video_alive_sync_mig 1]]
    require_single_direct_load video_alive_stage1 \
        [lindex $video_alive_sync_mig 1] \
        [list [lindex $video_alive_sync_mig 2]]
}
if {[llength $status_request_src_sync] == 3} {
    require_single_direct_load status_request_src_stage0 \
        [lindex $status_request_src_sync 0] \
        [list [lindex $status_request_src_sync 1]]
    require_single_direct_load status_request_src_stage1 \
        [lindex $status_request_src_sync 1] \
        [list [lindex $status_request_src_sync 2]]
}

# Source registers for the deliberately ignored cross-domain paths.  Exact
# counts also reject source-register replication or naming ambiguity.
set completed_bank_mig [require_cell_group \
    completed_bank_mig completed_bank_mig_reg 2 0]
set completed_toggle_mig [require_cell_group \
    completed_toggle_mig completed_toggle_mig_reg 1 0]
set ownership_payload_mig [require_cell_group \
    ownership_payload_mig ownership_payload_mig_reg 6 0]
set ownership_event_toggle_mig [require_cell_group \
    ownership_event_toggle_mig ownership_event_toggle_mig_reg 1 0]
set ownership_ack_toggle_src [require_cell_group \
    ownership_ack_toggle_src ownership_ack_toggle_src_reg 1 0]

require_group_clock completed_bank_mig $completed_bank_mig mmcm_clkout0
require_group_clock completed_toggle_mig $completed_toggle_mig mmcm_clkout0
require_group_clock ownership_payload_mig $ownership_payload_mig mmcm_clkout0
require_group_clock ownership_event_toggle_mig \
    $ownership_event_toggle_mig mmcm_clkout0
require_group_clock ownership_ack_toggle_src \
    $ownership_ack_toggle_src lg1_serial_clk_mmcm

# Reports whose absence is itself a blocker.  CDC findings remain explicit in
# the report; this script does not waive or downgrade any of them.
run_required_report "check_timing -verbose" \
    [list check_timing -verbose -file [file join $report_dir check_timing.rpt]]
run_required_report "clock interaction" \
    [list report_clock_interaction -delay_type min_max -significant_digits 3 \
        -file [file join $report_dir clock_interaction.rpt]]
run_required_report "ignored timing exceptions" \
    [list report_exceptions -ignored \
        -file [file join $report_dir exceptions_ignored.rpt]]
run_required_report "timing-exception coverage" \
    [list report_exceptions -coverage \
        -file [file join $report_dir exceptions_coverage.rpt]]
run_required_report "CDC detail" \
    [list report_cdc -details -all_checks_per_endpoint -no_waiver \
        -cells $top_cells -name cyan_hdmi_cdc \
        -file [file join $report_dir hdmi_cdc.rpt]]
capture_timing_summary [file join $report_dir timing_summary.rpt]
run_required_report "route status" \
    [list report_route_status -file [file join $report_dir route_status.rpt]]
capture_drc_report [file join $report_dir drc.rpt]
capture_methodology_report [file join $report_dir methodology.rpt] \
    {TIMING-9 TIMING-10 TIMING-24 TIMING-47}

if {[catch {
    set routed_fully [report_route_status -boolean_check ROUTED_FULLY]
    set route_errors [report_route_status -boolean_check ERRORS_IN_ROUTES]
} error_text]} {
    blocker "route-status boolean checks are unsupported or failed: $error_text"
} else {
    note "Route booleans: ROUTED_FULLY=$routed_fully ERRORS_IN_ROUTES=$route_errors"
    if {!$routed_fully || $route_errors} {
        blocker "checkpoint is not cleanly fully routed"
    }
}

check_global_timing max
check_global_timing min

# Intra-destination-clock paths must remain timed and meet both setup and
# hold.  These reports also expose the physical settling time between stages.
if {[llength $completed_bank_video_sync1] == 2 &&
    [llength $completed_bank_video_sync2] == 2} {
    check_timing_path_set completed_bank_interstage \
        $completed_bank_video_sync1 $completed_bank_video_sync2 2 \
        [file join $report_dir interstage_completed_bank.rpt]
}
if {[llength $completed_toggle_video_sync] == 3} {
    check_timing_path_set completed_event_interstage \
        [lrange $completed_toggle_video_sync 0 1] \
        [lrange $completed_toggle_video_sync 1 2] 2 \
        [file join $report_dir interstage_completed_event.rpt]
}
if {[llength $ownership_payload_src_sync1] == 6 &&
    [llength $ownership_payload_src_sync2] == 6} {
    check_timing_path_set ownership_payload_interstage \
        $ownership_payload_src_sync1 $ownership_payload_src_sync2 6 \
        [file join $report_dir interstage_ownership_payload.rpt]
}
if {[llength $ownership_event_src_sync] == 3} {
    check_timing_path_set ownership_event_interstage \
        [lrange $ownership_event_src_sync 0 1] \
        [lrange $ownership_event_src_sync 1 2] 2 \
        [file join $report_dir interstage_ownership_event.rpt]
}
if {[llength $ownership_ack_sync_mig] == 2} {
    check_timing_path_set ownership_ack_interstage \
        [lindex $ownership_ack_sync_mig 0] \
        [lindex $ownership_ack_sync_mig 1] 1 \
        [file join $report_dir interstage_ownership_ack.rpt]
}
if {[llength $video_alive_sync_mig] == 3} {
    check_timing_path_set video_alive_interstage \
        [lrange $video_alive_sync_mig 0 1] \
        [lrange $video_alive_sync_mig 1 2] 2 \
        [file join $report_dir interstage_video_alive.rpt]
}
if {[llength $status_request_src_sync] == 3} {
    check_timing_path_set status_request_src_interstage \
        [lrange $status_request_src_sync 0 1] \
        [lrange $status_request_src_sync 1 2] 2 \
        [file join $report_dir interstage_status_request_src.rpt]
}

set mtbf_first_stage_cells [concat \
    $completed_bank_video_sync1 \
    [lrange $completed_toggle_video_sync 0 0] \
    $ownership_payload_src_sync1 \
    [lrange $ownership_event_src_sync 0 0] \
    [lrange $ownership_ack_sync_mig 0 0] \
    [lrange $video_alive_sync_mig 0 0] \
    [lrange $status_request_src_sync 0 0]]
apply_conservative_mtbf_activity $mtbf_first_stage_cells
capture_mtbf_report [file join $report_dir synchronizer_mtbf.rpt] [list \
    [list completed_bank_video_sync1_reg 2] \
    [list {completed_toggle_video_sync_reg[0]/D} 1] \
    [list ownership_payload_src_sync1_reg 6] \
    [list {ownership_event_src_sync_reg[0]/D} 1] \
    [list {ownership_ack_sync_mig_reg[0]/D} 1] \
    [list {video_alive_sync_mig_reg[0]/D} 1] \
    [list {status_request_src_sync_reg[0]/D} 1]]

# The broad asynchronous clock groups intentionally make these paths
# user-ignored.  Measure the routed datapath directly.  Limits match the
# destination periods used by XPM max-delay constraints in this design.
# Ownership returns to the LG1 serial clock, whose period must be read from
# this checkpoint rather than inherited from the older machine-clock source.
set completed_bundle_sources [concat \
    $completed_bank_mig $completed_toggle_mig]
set completed_bundle_destinations [concat \
    $completed_bank_video_sync1 [lrange $completed_toggle_video_sync 0 0]]
if {[llength $completed_bundle_sources] == 3 &&
    [llength $completed_bundle_destinations] == 3} {
    check_ignored_delay completed_bank_bundle \
        $completed_bundle_sources $completed_bundle_destinations 3 15.378 \
        [file join $report_dir ignored_completed_bank_bundle.rpt]
}

set ownership_bundle_sources [concat \
    $ownership_payload_mig $ownership_event_toggle_mig]
set ownership_bundle_destinations [concat \
    $ownership_payload_src_sync1 [lrange $ownership_event_src_sync 0 0]]
if {[llength $ownership_bundle_sources] == 7 &&
    [llength $ownership_bundle_destinations] == 7} {
    check_ignored_delay ownership_bundle \
        $ownership_bundle_sources $ownership_bundle_destinations 7 \
        [get_property PERIOD [lindex $source_clocks 0]] \
        [file join $report_dir ignored_ownership_bundle.rpt]
}

if {[llength $ownership_ack_toggle_src] == 1 &&
    [llength $ownership_ack_sync_mig] == 2} {
    check_ignored_delay ownership_ack \
        $ownership_ack_toggle_src [lrange $ownership_ack_sync_mig 0 0] \
        1 3.332 [file join $report_dir ignored_ownership_ack.rpt]
}

# These constraints are analysis-only.  Bus skew is not overridden by the
# existing clock groups, so the routed DCP can be checked without a reroute.
if {[llength $completed_bundle_sources] == 3 &&
    [llength $completed_bundle_destinations] == 3 &&
    [llength $ownership_bundle_sources] == 7 &&
    [llength $ownership_bundle_destinations] == 7} {
    capture_temporary_bus_skew_report \
        [file join $report_dir temporary_bundle_bus_skew.rpt] \
        $completed_bundle_sources $completed_bundle_destinations \
        $ownership_bundle_sources $ownership_bundle_destinations
}

if {[catch {close_design} error_text]} {
    blocker "close_design failed: $error_text"
}

set failure_count [llength $::cyan_failures]
if {$failure_count == 0} {
    note "PASS CYAN HDMI CDC routed-checkpoint signoff"
    close $::cyan_summary_fd
    exit 0
}

note "FAIL CYAN HDMI CDC routed-checkpoint signoff: $failure_count blocker(s)"
set failure_index 0
foreach failure $::cyan_failures {
    incr failure_index
    note "  $failure_index. $failure"
}
close $::cyan_summary_fd
exit 1
