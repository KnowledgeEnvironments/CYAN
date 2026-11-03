# SPDX-License-Identifier: MIT
# Copyright (c) 2026 I. Finder <src@lispm.net>

# Exact constraint restoration for standalone checkpoint recovery.
#
# Vivado materializes generated target files inside the exact build worktree,
# so that worktree is expected to look tracked-dirty after a build. Recovery
# must not source those mutable files. Extract the committed hook and every
# transitive constraint contract directly from the source commit's Git objects,
# verify the extracted blobs, then source that immutable bundle.

proc cyan_recovery_materialize_commit_file {
    source_root source_commit relative_path destination
} {
    set object_spec "${source_commit}:${relative_path}"
    if {[catch {exec git -C $source_root rev-parse $object_spec} blob]} {
        error "unable to resolve committed constraint $object_spec: $blob"
    }
    if {![regexp {^[0-9a-f]{40}$} $blob]} {
        error "invalid blob identity for $object_spec: $blob"
    }
    file mkdir [file dirname $destination]
    if {[catch {exec git -C $source_root cat-file blob $blob > $destination} output]} {
        error "unable to extract committed constraint $object_spec: $output"
    }
    if {[catch {exec git -C $source_root hash-object $destination} actual_blob]} {
        error "unable to hash extracted constraint $destination: $actual_blob"
    }
    if {$actual_blob ne $blob} {
        error "extracted constraint mismatch for $object_spec: expected=$blob actual=$actual_blob"
    }
    return $blob
}

proc cyan_recovery_apply_exact_post_link_hook {source_root source_commit output_dir} {
    set source_root [file normalize $source_root]
    if {![file exists [file join $source_root .git]]} {
        error "missing exact source worktree: $source_root"
    }
    if {[catch {exec git -C $source_root rev-parse HEAD} actual_commit]} {
        error "unable to read exact source identity at $source_root: $actual_commit"
    }
    if {$actual_commit ne $source_commit} {
        error "source worktree mismatch: expected=$source_commit actual=$actual_commit"
    }

    set bundle_root [file join $output_dir exact-source-constraints]
    set hook_relative \
        fpga/kcu105_cyan/constraints/kcu105_cyan_soc_post_link.tcl
    set cpu_relative fpga/kcu105_cyan/cpu_island_cdc_constraints.tcl
    set lg1_relative fpga/kcu105_cyan/lg1_serial_cdc_constraints.tcl
    set sgmii_relative fpga/kcu105_cyan/sgmii_rx_gearbox_constraints.tcl
    set hook_path [file join $bundle_root $hook_relative]
    set cpu_path [file join $bundle_root $cpu_relative]
    set lg1_path [file join $bundle_root $lg1_relative]
    set sgmii_path [file join $bundle_root $sgmii_relative]

    set hook_blob [cyan_recovery_materialize_commit_file $source_root \
        $source_commit $hook_relative $hook_path]
    set cpu_blob [cyan_recovery_materialize_commit_file $source_root \
        $source_commit $cpu_relative $cpu_path]
    set lg1_blob [cyan_recovery_materialize_commit_file $source_root \
        $source_commit $lg1_relative $lg1_path]
    set sgmii_blob [cyan_recovery_materialize_commit_file $source_root \
        $source_commit $sgmii_relative $sgmii_path]

    set manifest [open [file join $output_dir SOURCE_CONSTRAINT_BLOBS.txt] w]
    puts $manifest "source_commit=$source_commit"
    puts $manifest "post_link_hook_blob=$hook_blob"
    puts $manifest "cpu_island_cdc_blob=$cpu_blob"
    puts $manifest "lg1_serial_cdc_blob=$lg1_blob"
    puts $manifest "sgmii_rx_gearbox_blob=$sgmii_blob"
    close $manifest

    uplevel #0 [list source $hook_path]
    puts "Reapplied exact committed post-link constraints from $hook_path"
}
