# Fail-closed timing contract for the transactional 100/50 MHz CPU-island
# boundary.  Only the first metastability-containment D pins are false-pathed;
# the held bundled payloads remain explicitly bounded and skew constrained.

proc cyan_cpu_bridge_require_clock {label pin_pattern} {
    set pins [get_pins -quiet -hierarchical -regexp $pin_pattern]
    if {[llength $pins] != 1} {
        error "$label clock source resolved to [llength $pins] pins, expected 1: $pins"
    }
    set clocks [get_clocks -quiet -of_objects $pins]
    if {[llength $clocks] != 1} {
        error "$label resolved to [llength $clocks] clocks, expected 1: $clocks"
    }
    return $clocks
}

proc cyan_cpu_bridge_require_cells {label patterns expected} {
    set cells {}
    foreach pattern $patterns {
        set cells [concat $cells \
            [get_cells -quiet -hierarchical -regexp $pattern]]
    }
    set cells [lsort -unique $cells]
    if {[llength $cells] != $expected} {
        error "$label resolved to [llength $cells] cells, expected $expected: $cells"
    }
    foreach cell $cells {
        set ref_name [get_property REF_NAME $cell]
        if {![regexp {^FD} $ref_name]} {
            error "$label contains non-register $cell (REF_NAME=$ref_name)"
        }
    }
    return $cells
}

proc cyan_cpu_bridge_optional_cell {label pattern} {
    set cells [lsort -unique \
        [get_cells -quiet -hierarchical -regexp $pattern]]
    if {[llength $cells] > 1} {
        error "$label resolved to [llength $cells] cells, expected at most 1: $cells"
    }
    foreach cell $cells {
        set ref_name [get_property REF_NAME $cell]
        if {![regexp {^FD} $ref_name]} {
            error "$label contains non-register $cell (REF_NAME=$ref_name)"
        }
    }
    return $cells
}

proc cyan_cpu_bridge_require_absent_cells {label patterns} {
    set cells {}
    foreach pattern $patterns {
        set cells [concat $cells \
            [get_cells -quiet -hierarchical -regexp $pattern]]
    }
    set cells [lsort -unique $cells]
    if {[llength $cells] != 0} {
        error "$label resolved to [llength $cells] cells, expected none: $cells"
    }
}

proc cyan_cpu_bridge_pins {label cells ref_pin} {
    set pins [get_pins -quiet -of_objects $cells \
        -filter "REF_PIN_NAME == $ref_pin"]
    if {[llength $pins] != [llength $cells]} {
        error "$label has [llength $pins] $ref_pin pins for [llength $cells] cells"
    }
    return $pins
}

proc cyan_cpu_bridge_constrain_bundle {label source_cells destination_cells max_ns} {
    # Use sequential cells as -from/-to objects.  Vivado then retains the
    # natural source C/Q launch and destination D endpoint.  Naming Q pins as
    # explicit startpoints segments these paths (Constraints 18-515), which
    # can make a syntactically accepted CDC constraint physically misleading.
    set_max_delay -datapath_only $max_ns \
        -from $source_cells -to $destination_cells
    set_bus_skew $max_ns -from $source_cells -to $destination_cells
    puts "Applied CPU-island bundle constraint $label: source=[llength $source_cells] destination=[llength $destination_cells] max/skew=${max_ns}ns"
}

set cyan_machine_clocks [cyan_cpu_bridge_require_clock machine \
    {.*u_machine_clk_div/O}]
set cyan_island_clocks [cyan_cpu_bridge_require_clock cpu_island \
    {.*u_cpu_island_clk_div/O}]
set cyan_machine_period [get_property PERIOD [lindex $cyan_machine_clocks 0]]
set cyan_island_period [get_property PERIOD [lindex $cyan_island_clocks 0]]
if {[expr {abs($cyan_machine_period - 20.000)}] > 0.020 ||
    [expr {abs($cyan_island_period - 10.000)}] > 0.020 ||
    [expr {abs($cyan_machine_period - 2.0 * $cyan_island_period)}] > 0.002} {
    error "CPU-island clocks do not match the admitted 50/100 MHz topology: machine=$cyan_machine_period ns island=$cyan_island_period ns"
}
if {[lindex $cyan_machine_clocks 0] eq [lindex $cyan_island_clocks 0]} {
    error "CPU-island machine and island sources resolved to the same clock"
}

# Toggle/level controls: cut only the source-clock path into the first
# synchronizer stage.  The second stage and all functional fanout stay timed.
set cyan_bridge_cpu_to_machine_sync [cyan_cpu_bridge_require_cells \
    bridge_cpu_to_machine_first_stage [list \
        {.*u_cpu_island_bridge/machine_req_sync1_reg} \
        {.*u_cpu_island_bridge/machine_wr_ack_sync1_reg} \
        {.*u_cpu_island_bridge/machine_rd_ack_sync1_reg} \
        {.*u_cpu_island_bridge/machine_core_hold_ack_sync1_reg} \
        {.*u_cpu_island_bridge/machine_dmem_fence_ack_sync1_reg}] 5]
set cyan_cpu_to_machine_sync $cyan_bridge_cpu_to_machine_sync
# IP20 drives only external Cause IP[6:2] through this boundary.  Cause IP7 is
# generated inside the R4000 by CP0 Count/Compare, so product IRQ bit 5 is
# constant and must disappear after optimization.  Routed signoff separately
# rejects either stage if that nonarchitectural external bit becomes live.
# The normal non-debug KCU105 top also ties force_uncached low.  Keep its two
# ASYNC_REG stages for reset/debug topology, but do not pretend the constant
# first-stage D pin has a machine-clock launch path.  Routed signoff proves the
# exact GND driver before accepting the resulting 10 dynamic ignored paths.
set cyan_machine_to_cpu_dynamic_sync [cyan_cpu_bridge_require_cells \
    machine_to_cpu_dynamic_first_stage [list \
        {.*u_cpu_island_bridge/cpu_resp_sync1_reg} \
        {.*u_cpu_island_bridge/cpu_wr_req_sync1_reg} \
        {.*u_cpu_island_bridge/cpu_rd_req_sync1_reg} \
        {.*u_cpu_island_bridge/cpu_core_hold_sync1_reg} \
        {.*u_cpu_island_bridge/cpu_dmem_fence_sync1_reg} \
        {.*u_cpu_island_bridge/cpu_irq_sync1_reg\[[0-4]\]}] 10]
set cyan_machine_to_cpu_static_sync [cyan_cpu_bridge_require_cells \
    machine_to_cpu_static_first_stage \
    [list {.*u_cpu_island_bridge/cpu_force_uncached_sync1_reg}] 1]
set cyan_machine_to_cpu_sync [concat \
    $cyan_machine_to_cpu_dynamic_sync \
    $cyan_machine_to_cpu_static_sync]
set cyan_machine_to_cpu_first_stage_width \
    [llength $cyan_machine_to_cpu_sync]
set_false_path -from $cyan_island_clocks -to \
    [cyan_cpu_bridge_pins cpu_to_machine_first_stage \
        $cyan_cpu_to_machine_sync D]
set_false_path -from $cyan_machine_clocks -to \
    [cyan_cpu_bridge_pins machine_to_cpu_first_stage \
        $cyan_machine_to_cpu_dynamic_sync D]

# CPU request payload -> machine request slot.  The product consumes 139 bits:
# ren, wen, paddr, wmask, wdata, cache-owned provenance, and the diagnostic PC.
# r4000_top also exports uncached_wr for standalone observation, but the IP20
# SoC has never consumed it; Vivado therefore removes both mailbox copies.  If
# a future product sink makes that bit live, admit it only as a matched pair so
# it is constrained as the 140th bundled bit instead of becoming an unbounded
# CDC path.
set cyan_cpu_req_source [cyan_cpu_bridge_require_cells cpu_request_source \
    [list \
        {.*u_cpu_island_bridge/cpu_req_(ren_q|wen_q|cache_owned_q)_reg} \
        {.*u_cpu_island_bridge/cpu_req_paddr_q_reg\[[0-9]+\]} \
        {.*u_cpu_island_bridge/cpu_req_wmask_q_reg\[[0-9]+\]} \
        {.*u_cpu_island_bridge/cpu_req_wdata_q_reg\[[0-9]+\]} \
        {.*u_cpu_island_bridge/cpu_req_pc_q_reg\[[0-9]+\]}] 139]
set cyan_machine_req_destination [cyan_cpu_bridge_require_cells \
    machine_request_destination [list \
        {.*u_cpu_island_bridge/machine_req_(ren_q|wen_q|cache_owned_q)_reg} \
        {.*u_cpu_island_bridge/machine_req_paddr_q_reg\[[0-9]+\]} \
        {.*u_cpu_island_bridge/machine_req_wmask_q_reg\[[0-9]+\]} \
        {.*u_cpu_island_bridge/machine_req_wdata_q_reg\[[0-9]+\]} \
        {.*u_cpu_island_bridge/machine_req_pc_q_reg\[[0-9]+\]}] 139]
set cyan_cpu_req_uncached_source [cyan_cpu_bridge_optional_cell \
    cpu_request_uncached_source \
    {.*u_cpu_island_bridge/cpu_req_uncached_wr_q_reg}]
set cyan_machine_req_uncached_destination [cyan_cpu_bridge_optional_cell \
    machine_request_uncached_destination \
    {.*u_cpu_island_bridge/machine_req_uncached_wr_q_reg}]
if {[llength $cyan_cpu_req_uncached_source] !=
    [llength $cyan_machine_req_uncached_destination]} {
    error "CPU request uncached provenance is live on only one mailbox side: source=$cyan_cpu_req_uncached_source destination=$cyan_machine_req_uncached_destination"
}
set cyan_cpu_req_source [concat $cyan_cpu_req_source \
    $cyan_cpu_req_uncached_source]
set cyan_machine_req_destination [concat \
    $cyan_machine_req_destination $cyan_machine_req_uncached_destination]
set cyan_cpu_request_width [llength $cyan_cpu_req_source]
cyan_cpu_bridge_constrain_bundle cpu_request \
    $cyan_cpu_req_source $cyan_machine_req_destination $cyan_machine_period

# Machine response data -> CPU response register (64 bits).
set cyan_machine_resp_source [cyan_cpu_bridge_require_cells \
    machine_response_source \
    [list {.*u_cpu_island_bridge/machine_resp_data_q_reg\[[0-9]+\]}] 64]
set cyan_cpu_resp_destination [cyan_cpu_bridge_require_cells \
    cpu_response_destination \
    [list {.*u_cpu_island_bridge/cpu_dmem_rdata_reg\[[0-9]+\]}] 64]
cyan_cpu_bridge_constrain_bundle cpu_response \
    $cyan_machine_resp_source $cyan_cpu_resp_destination $cyan_island_period

# Machine DMA-write snoop payload -> CPU snoop slot (101 live bits).  Both
# S-cache write snoop paths address 64-bit beats with addr[31:3]; addr[2:0]
# never influence cache lookup or update and are removed on both mailbox sides.
set cyan_machine_wr_source [cyan_cpu_bridge_require_cells \
    machine_write_snoop_source [list \
        {.*u_cpu_island_bridge/machine_wr_addr_q_reg\[[0-9]+\]} \
        {.*u_cpu_island_bridge/machine_wr_data_q_reg\[[0-9]+\]} \
        {.*u_cpu_island_bridge/machine_wr_mask_q_reg\[[0-9]+\]}] 101]
set cyan_cpu_wr_destination [cyan_cpu_bridge_require_cells \
    cpu_write_snoop_destination [list \
        {.*u_cpu_island_bridge/cpu_snoop_wr_addr_reg\[[0-9]+\]} \
        {.*u_cpu_island_bridge/cpu_snoop_wr_data_reg\[[0-9]+\]} \
        {.*u_cpu_island_bridge/cpu_snoop_wr_mask_reg\[[0-9]+\]}] 101]
cyan_cpu_bridge_constrain_bundle dma_write_snoop \
    $cyan_machine_wr_source $cyan_cpu_wr_destination $cyan_island_period

# Machine DMA-read probe address -> CPU probe slot (29 live bits).  As above,
# the lookup selects a doubleword and intentionally consumes addr[31:3].
set cyan_machine_rd_source [cyan_cpu_bridge_require_cells \
    machine_read_probe_source \
    [list {.*u_cpu_island_bridge/machine_rd_addr_q_reg\[[0-9]+\]}] 29]
set cyan_cpu_rd_destination [cyan_cpu_bridge_require_cells \
    cpu_read_probe_destination \
    [list {.*u_cpu_island_bridge/cpu_rd_addr_q_reg\[[0-9]+\]}] 29]
cyan_cpu_bridge_constrain_bundle dma_read_probe \
    $cyan_machine_rd_source $cyan_cpu_rd_destination $cyan_island_period

# CPU probe response -> machine result slot (65 bits).
set cyan_cpu_rd_resp_source [cyan_cpu_bridge_require_cells \
    cpu_read_response_source [list \
        {.*u_cpu_island_bridge/cpu_rd_resp_hit_q_reg} \
        {.*u_cpu_island_bridge/cpu_rd_resp_data_q_reg\[[0-9]+\]}] 65]
set cyan_machine_rd_resp_destination [cyan_cpu_bridge_require_cells \
    machine_read_response_destination [list \
        {.*u_cpu_island_bridge/machine_snoop_rd_resp_hit_reg} \
        {.*u_cpu_island_bridge/machine_snoop_rd_resp_data_reg\[[0-9]+\]}] 65]
cyan_cpu_bridge_constrain_bundle dma_read_response \
    $cyan_cpu_rd_resp_source $cyan_machine_rd_resp_destination \
    $cyan_machine_period

puts "Applied fail-closed CPU-island CDC contract: machine=${cyan_machine_period}ns island=${cyan_island_period}ns"
