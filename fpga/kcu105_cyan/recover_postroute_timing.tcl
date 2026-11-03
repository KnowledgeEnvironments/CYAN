# SPDX-License-Identifier: MIT
# Copyright (c) 2026 I. Finder <src@lispm.net>

# Post-route timing recovery for a routed KCU105 SoC checkpoint.
#
# This recipe invokes only Vivado's supported post-route
# routing optimization; it does not rerun synthesis, placement, or global
# route_design unless the optimizer itself leaves a net unrouted.

source [file normalize [file join [file dirname [info script]] \
    recovery_constraints.tcl]]

proc cyan_postroute_sha256 {path} {
    if {![file isfile $path]} {
        error "checkpoint does not exist: $path"
    }
    if {[catch {exec sha256sum $path} output]} {
        error "sha256sum failed for $path: $output"
    }
    set digest [lindex $output 0]
    if {![regexp {^[0-9a-f]{64}$} $digest]} {
        error "invalid sha256sum output for $path: $output"
    }
    return $digest
}

if {$argc == 1 && [lindex $argv 0] eq "--syntax-check"} {
    puts "PASS recover_postroute_timing.tcl Tcl syntax load"
    exit 0
}
if {[llength $argv] != 6} {
    puts stderr "usage: recover_postroute_timing.tcl INPUT_DCP OUTPUT_DIR SOURCE_ROOT SOURCE_COMMIT RECIPE_COMMIT INPUT_SHA256"
    exit 2
}

lassign $argv input_dcp output_dir source_root source_commit recipe_commit \
    input_sha256
set input_dcp [file normalize $input_dcp]
set output_dir [file normalize $output_dir]
set source_root [file normalize $source_root]
set recipe_root [file normalize [file join [file dirname [info script]] ../..]]

if {![file exists $input_dcp]} {
    puts stderr "missing routed checkpoint: $input_dcp"
    exit 2
}
if {![string match "*_routed.dcp" [file tail $input_dcp]]} {
    puts stderr "post-route recovery requires *_routed.dcp, not: $input_dcp"
    exit 2
}
if {![catch {file lstat $output_dir output_stat}]} {
    puts stderr "output already exists: $output_dir"
    exit 2
}
if {![regexp {^[0-9a-f]{40}$} $source_commit]} {
    puts stderr "invalid source commit: $source_commit"
    exit 2
}
if {![regexp {^[0-9a-f]{40}$} $recipe_commit]} {
    puts stderr "invalid recipe commit: $recipe_commit"
    exit 2
}
if {![regexp {^[0-9a-f]{64}$} $input_sha256]} {
    puts stderr "invalid input checkpoint SHA-256: $input_sha256"
    exit 2
}
if {[catch {cyan_postroute_sha256 $input_dcp} actual_input_sha]} {
    puts stderr $actual_input_sha
    exit 2
}
if {$actual_input_sha ne $input_sha256} {
    puts stderr "input checkpoint hash mismatch: expected=$input_sha256 actual=$actual_input_sha"
    exit 2
}
foreach {kind root expected} [list source $source_root $source_commit \
                                 recipe $recipe_root $recipe_commit] {
    if {[catch {exec git -C $root rev-parse HEAD} actual_commit]} {
        puts stderr "unable to read $kind worktree identity: $actual_commit"
        exit 2
    }
    if {$actual_commit ne $expected} {
        puts stderr "$kind worktree mismatch: expected=$expected actual=$actual_commit"
        exit 2
    }
}

file mkdir $output_dir
open_checkpoint $input_dcp

# Bind the normal managed-flow constraints to immutable product-source blobs,
# not generated files or the possibly newer recovery recipe commit.
cyan_recovery_apply_exact_post_link_hook $source_root $source_commit $output_dir
report_timing_summary -max_paths 10 -warn_on_violation \
    -file [file join $output_dir timing_before.rpt]

# Optimize only routing on negative-slack paths. Vivado 2020.1 explicitly
# supports this operation in post-route mode.
phys_opt_design -routing_opt

# Post-route routing optimization should maintain route legality. If a local
# edit is left unrouted, repair only that edit rather than restarting P&R.
if {![report_route_status -boolean_check ROUTED_FULLY]} {
    cyan_recovery_apply_exact_post_link_hook $source_root $source_commit $output_dir
    route_design
}

set timing_report [file join $output_dir kcu105_cyan_soc_top_timing_summary_routed.rpt]
set route_report [file join $output_dir kcu105_cyan_soc_top_route_status.rpt]
set drc_report [file join $output_dir kcu105_cyan_soc_top_drc_routed.rpt]
set bus_skew_report [file join $output_dir kcu105_cyan_soc_top_bus_skew_routed.rpt]

report_timing_summary -max_paths 10 -warn_on_violation -file $timing_report
report_route_status -file $route_report
report_drc -file $drc_report
report_bus_skew -warn_on_violation -file $bus_skew_report

set setup_paths [get_timing_paths -delay_type max -max_paths 1]
set hold_paths [get_timing_paths -delay_type min -max_paths 1]
if {[llength $setup_paths] != 1 || [llength $hold_paths] != 1} {
    puts stderr "unable to obtain setup/hold signoff paths"
    exit 3
}
set wns [get_property SLACK [lindex $setup_paths 0]]
set whs [get_property SLACK [lindex $hold_paths 0]]
set routed_fully [report_route_status -boolean_check ROUTED_FULLY]
set route_errors [report_route_status -boolean_check ERRORS_IN_ROUTES]

set provenance [open [file join $output_dir PROVENANCE.json] w]
puts $provenance "{"
puts $provenance "  \"source_commit\": \"$source_commit\","
puts $provenance "  \"recipe_commit\": \"$recipe_commit\","
puts $provenance "  \"input_checkpoint\": \"$input_dcp\","
puts $provenance "  \"input_sha256\": \"$input_sha256\","
puts $provenance "  \"post_link_hook_source_commit\": \"$source_commit\","
puts $provenance "  \"optimization\": \"phys_opt_design -routing_opt\","
puts $provenance "  \"wns_ns\": $wns,"
puts $provenance "  \"whs_ns\": $whs,"
puts $provenance "  \"routed_fully\": $routed_fully,"
puts $provenance "  \"route_errors\": $route_errors"
puts $provenance "}"
close $provenance

set output_dcp [file join $output_dir kcu105_cyan_soc_top_routed.dcp]
write_checkpoint -force $output_dcp

if {$wns < 0.0 || $whs < 0.0 || !$routed_fully || $route_errors} {
    puts stderr "post-route recovery rejected: WNS=$wns WHS=$whs routed=$routed_fully route_errors=$route_errors"
    exit 4
}

write_debug_probes -force [file join $output_dir kcu105_cyan_soc.ltx]
write_bitstream -force [file join $output_dir kcu105_cyan_soc.bit]
close_design

puts "post-route routing recovery complete: WNS=$wns WHS=$whs"
exit 0
