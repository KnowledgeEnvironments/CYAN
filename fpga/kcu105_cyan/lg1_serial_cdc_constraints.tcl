# Fail-closed timing contract for the independent 50 MHz LG1 machine side and
# 64 MHz VC1/XMAP/DAC serial side.  Only first metastability-containment stages
# are false-pathed.  Every held ownership payload remains bounded and skew
# constrained; the two clock domains are deliberately not placed in a broad
# asynchronous clock group.

proc cyan_lg1_serial_require_clock {label pin_pattern} {
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

proc cyan_lg1_serial_require_cells {label patterns expected} {
    set cells {}
    foreach pattern $patterns {
        set cells [concat $cells \
            [get_cells -quiet -hierarchical -regexp \
                -include_replicated_objects $pattern]]
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

proc cyan_lg1_serial_pins {label cells ref_pin} {
    set pins [get_pins -quiet -of_objects $cells \
        -filter "REF_PIN_NAME == $ref_pin"]
    if {[llength $pins] != [llength $cells]} {
        error "$label has [llength $pins] $ref_pin pins for [llength $cells] cells"
    }
    return $pins
}

proc cyan_lg1_serial_constrain_bundle {
    label source_cells destination_cells max_ns
} {
    # These registers are the physical endpoints of a held, toggle-qualified
    # mailbox.  Physical optimization must not clone either bank after the
    # endpoint collection has been constrained: every clone would otherwise
    # become a new asynchronous sampler outside this fail-closed contract.
    # Local fanout is allowed only after the destination capture bank.
    set_property DONT_TOUCH true [concat $source_cells $destination_cells]
    set_max_delay -datapath_only $max_ns \
        -from $source_cells -to $destination_cells
    set_bus_skew $max_ns -from $source_cells -to $destination_cells
    puts "Applied LG1 serial bundle $label: source=[llength $source_cells] destination=[llength $destination_cells] max/skew=${max_ns}ns"
}

set cyan_lg1_machine_clocks [cyan_lg1_serial_require_clock machine \
    {.*u_machine_clk_div/O}]
set cyan_lg1_serial_clocks [cyan_lg1_serial_require_clock serial \
    {.*u_lg1_serial_clk_buf/O}]
set cyan_lg1_machine_period \
    [get_property PERIOD [lindex $cyan_lg1_machine_clocks 0]]
set cyan_lg1_serial_period \
    [get_property PERIOD [lindex $cyan_lg1_serial_clocks 0]]
if {[expr {abs($cyan_lg1_machine_period - 19.992)}] > 0.030 ||
    [expr {abs($cyan_lg1_serial_period - 15.619)}] > 0.030} {
    error "LG1 clocks do not match admitted 50.020/64.026 MHz topology: machine=$cyan_lg1_machine_period ns serial=$cyan_lg1_serial_period ns"
}
if {[lindex $cyan_lg1_machine_clocks 0] eq
    [lindex $cyan_lg1_serial_clocks 0]} {
    error "LG1 machine and serial sources resolved to the same clock"
}

set cyan_lg1_machine_to_serial_sync [cyan_lg1_serial_require_cells \
    machine_to_serial_first_stage [list \
        {.*u_lg1/u_core/u_config_cdc/dst_req_sync1_reg} \
        {.*u_lg1/u_core/gen_line_fetch\.u_line_fetch/publish_sync1_serial_reg} \
        {.*u_lg1/u_core/gen_line_fetch\.u_line_fetch/signature_sync1_serial_reg} \
        {.*u_lg1/u_core/gen_line_fetch\.u_line_fetch/pause_sync1_serial_reg} \
        {.*u_lg1/u_core/gen_line_fetch\.u_line_fetch/protocol_gray_sync1_serial_reg\[[0-9]+\]} \
        {.*u_hdmi_frame_resampler/status_request_src_sync_reg\[0\]}] 37]
set cyan_lg1_serial_to_machine_sync [cyan_lg1_serial_require_cells \
    serial_to_machine_first_stage [list \
        {.*u_lg1/u_core/u_config_cdc/src_ack_sync1_reg} \
        {.*u_lg1/u_core/u_retrace_cdc/dst_sync1_reg} \
        {.*u_lg1/u_core/gen_line_fetch\.u_line_fetch/row_req_sync1_producer_reg}] 3]

# Both serial-domain reset chains use the standard asynchronous-assert,
# synchronous-deassert topology.  Timing recovery from the machine-domain
# release into their asynchronous clear pins is not a synchronous data path;
# deassertion is instead contained by all four serial-clocked stages.  Require
# both chains and only cut their exact CLR pins so no payload crossing can be
# hidden by this reset exception.
set cyan_lg1_serial_reset_sync [cyan_lg1_serial_require_cells \
    serial_reset_synchronizers [list \
        {.*lg1_serial_reset_sync_reg\[[0-3]\]} \
        {.*u_soc/lg1_serial_soc_reset_shift_reg\[[0-3]\]}] 8]
set_false_path -to [cyan_lg1_serial_pins serial_reset_synchronizers \
    $cyan_lg1_serial_reset_sync CLR]
puts "Applied LG1 serial asynchronous-reset cut to exactly 8 CLR pins"

set_false_path -from $cyan_lg1_machine_clocks -to \
    [cyan_lg1_serial_pins machine_to_serial_first_stage \
        $cyan_lg1_machine_to_serial_sync D]
set_false_path -from $cyan_lg1_serial_clocks -to \
    [cyan_lg1_serial_pins serial_to_machine_first_stage \
        $cyan_lg1_serial_to_machine_sync D]

# CONFIG one-outstanding request: 22 held bits machine -> serial. cfg_offset
# is formed as {addr[10:2], 2'b00} before this boundary, so only its nine
# architecturally meaningful bits survive synthesis.
set cyan_lg1_config_request_source [cyan_lg1_serial_require_cells \
    config_request_source [list \
        {.*u_lg1/u_core/u_config_cdc/src_(write|go)_q_reg} \
        {.*u_lg1/u_core/u_config_cdc/src_offset_q_reg\[(10|[2-9])\]} \
        {.*u_lg1/u_core/u_config_cdc/src_sel_q_reg\[[0-9]+\]} \
        {.*u_lg1/u_core/u_config_cdc/src_wbyte_q_reg\[[0-9]+\]}] 22]
set cyan_lg1_config_request_destination [cyan_lg1_serial_require_cells \
    config_request_destination [list \
        {.*u_lg1/u_core/u_config_cdc/dst_(write|go)_reg} \
        {.*u_lg1/u_core/u_config_cdc/dst_offset_reg\[(10|[2-9])\]} \
        {.*u_lg1/u_core/u_config_cdc/dst_sel_reg\[[0-9]+\]} \
        {.*u_lg1/u_core/u_config_cdc/dst_wbyte_q_reg\[[0-9]+\]}] 22]
cyan_lg1_serial_constrain_bundle config_request \
    $cyan_lg1_config_request_source $cyan_lg1_config_request_destination \
    $cyan_lg1_serial_period

# Registered read-latch response: 8 held bits serial -> machine.
set cyan_lg1_config_response_source [cyan_lg1_serial_require_cells \
    config_response_source \
    [list {.*u_lg1/u_core/u_config_cdc/dst_response_q_reg\[[0-9]+\]}] 8]
set cyan_lg1_config_response_destination [cyan_lg1_serial_require_cells \
    config_response_destination \
    [list {.*u_lg1/u_core/u_config_cdc/src_rbyte_reg\[[0-9]+\]}] 8]
cyan_lg1_serial_constrain_bundle config_response \
    $cyan_lg1_config_response_source $cyan_lg1_config_response_destination \
    $cyan_lg1_machine_period

# Serial row request: the ten-bit line tag carries buffer ownership in bit 0.
# The explicit buffer register is assigned from the same bit and is therefore
# merged with line[0] on both sides of the synthesized boundary.
set cyan_lg1_row_request_source [cyan_lg1_serial_require_cells \
    row_request_source [list \
        {.*u_lg1/u_core/gen_line_fetch\.u_line_fetch/row_req_line_q_reg\[[0-9]+\]}] 10]
# The buffer bit also invalidates the selected producer-resident row when the
# synchronized request is accepted.  Those two valid flops are therefore real
# payload consumers, not unrelated control, and belong in the same bundle.
set cyan_lg1_row_request_destination [cyan_lg1_serial_require_cells \
    row_request_destination [list \
        {.*u_lg1/u_core/gen_line_fetch\.u_line_fetch/fetch_line_reg\[[0-9]+\]} \
        {.*u_lg1/u_core/gen_line_fetch\.u_line_fetch/producer_(even|odd)_valid_reg}] 12]
cyan_lg1_serial_constrain_bundle row_request \
    $cyan_lg1_row_request_source $cyan_lg1_row_request_destination \
    $cyan_lg1_machine_period

# Producer publication is immutable until the next serial request. The line
# tag again carries buffer ownership in bit 0, so synthesis merges the explicit
# buffer register. Either parity destination may receive the 11-bit good/line
# bundle.
set cyan_lg1_row_publish_source [cyan_lg1_serial_require_cells \
    row_publish_source [list \
        {.*u_lg1/u_core/gen_line_fetch\.u_line_fetch/row_publish_(good|buffer)_q_reg} \
        {.*u_lg1/u_core/gen_line_fetch\.u_line_fetch/row_publish_line_q_reg\[[1-9]\]}] 11]
# A completion arriving in the four-clock line-boundary retirement window may
# also enable the next request on the same serial edge.  The request pending,
# toggle and line registers consequently consume the held publication bundle
# in addition to the two resident-row banks.
set cyan_lg1_row_publish_destination [cyan_lg1_serial_require_cells \
    row_publish_destination [list \
        {.*u_lg1/u_core/gen_line_fetch\.u_line_fetch/serial_(even|odd)_valid_reg} \
        {.*u_lg1/u_core/gen_line_fetch\.u_line_fetch/serial_(even|odd)_line_reg\[[0-9]+\]} \
        {.*u_lg1/u_core/gen_line_fetch\.u_line_fetch/row_req_pending_reg} \
        {.*u_lg1/u_core/gen_line_fetch\.u_line_fetch/row_req_toggle_reg} \
        {.*u_lg1/u_core/gen_line_fetch\.u_line_fetch/row_req_line_q_reg\[[0-9]+\]}] 34]
cyan_lg1_serial_constrain_bundle row_publish \
    $cyan_lg1_row_publish_source $cyan_lg1_row_publish_destination \
    $cyan_lg1_serial_period

# Frame-response telemetry publication: two CRCs plus generation. The
# generation work and publication registers receive the same value on the
# publication edge, so synthesis retains one physical register bank under the
# work name; it remains held by the publication toggle contract.
set cyan_lg1_signature_source [cyan_lg1_serial_require_cells \
    signature_source [list \
        {.*u_lg1/u_core/gen_line_fetch\.u_line_fetch/signature_(low|high)_q_reg\[[0-9]+\]} \
        {.*u_lg1/u_core/gen_line_fetch\.u_line_fetch/signature_generation_(q|work)_reg\[[0-9]+\]}] 40]
set cyan_lg1_signature_destination [cyan_lg1_serial_require_cells \
    signature_destination [list \
        {.*u_lg1/u_core/gen_line_fetch\.u_line_fetch/front_response_(low|high)_signature_reg\[[0-9]+\]} \
        {.*u_lg1/u_core/gen_line_fetch\.u_line_fetch/front_response_signature_generation_reg\[[0-9]+\]}] 40]
cyan_lg1_serial_constrain_bundle signature \
    $cyan_lg1_signature_source $cyan_lg1_signature_destination \
    $cyan_lg1_serial_period

puts "Applied fail-closed LG1 serial CDC contract: machine=${cyan_lg1_machine_period}ns serial=${cyan_lg1_serial_period}ns"
