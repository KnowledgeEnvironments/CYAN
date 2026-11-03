set requested_step "generate"
if {$argc >= 1} {
    set requested_step [lindex $argv 0]
}

set jobs 4
if {$argc >= 2} {
    set jobs [lindex $argv 1]
}

set enable_debug 0
set enable_soc 0
set soc_build_signature ""
set step $requested_step
if {[string match "debug-*" $requested_step]} {
    set enable_debug 1
    set step [string range $requested_step 6 end]
}
if {[string match "soc-*" $step]} {
    set enable_soc 1
    set step [string range $step 4 end]
}

if {$step ni {"generate" "check" "synth" "impl" "bit"}} {
    puts stderr "usage: vivado -mode batch -source generate.tcl -tclargs \[debug-\]\[soc-\]generate|\[debug-\]\[soc-\]check|\[debug-\]\[soc-\]synth|\[debug-\]\[soc-\]impl|\[debug-\]\[soc-\]bit ?jobs?"
    exit 2
}

set origin_dir [file dirname [file normalize [info script]]]
set build_dir [file normalize [file join $origin_dir build]]
set seed_project_name "kcu105_cyan_seed"
set seed_project_dir [file join $build_dir seed]
if {$enable_soc} {
    set cyan_project_name "kcu105_cyan_soc"
    set cyan_project_dir [file join $build_dir project_soc]
} else {
    set cyan_project_name "kcu105_cyan"
    set cyan_project_dir [file join $build_dir project]
}
set cyan_project_xpr [file join $cyan_project_dir $cyan_project_name.xpr]
set part_name "xcku040-ffva1156-2-e"
set board_part "xilinx.com:kcu105:part0:1.6"
if {[info exists ::env(XILINX_VIVADO)]} {
    set vivado_root $::env(XILINX_VIVADO)
} else {
    set vivado_root "/opt/Xilinx/Vivado/2020.1"
}
set board_pins_xml [file join $vivado_root data boards board_files kcu105 1.6 part0_pins.xml]

proc fail {msg} {
    puts stderr $msg
    exit 1
}

set cyan_prom_dir ""
set cyan_eeprom_init_file ""
if {$enable_soc} {
    if {![info exists ::env(CYAN_PROM_DIR)] || $::env(CYAN_PROM_DIR) eq ""} {
        fail "CYAN_PROM_DIR must name a directory prepared by 'make firmware'"
    }
    set cyan_prom_dir [file normalize $::env(CYAN_PROM_DIR)]
    set required_firmware [list [file join $cyan_prom_dir cyan_firmware.memh]]
    foreach suffix {00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f} {
        lappend required_firmware \
            [file join $cyan_prom_dir cyan_firmware_b${suffix}.memh]
    }
    foreach path $required_firmware {
        if {![file isfile $path]} {
            fail "Missing prepared firmware input: $path"
        }
    }
    if {![info exists ::env(CYAN_EEPROM_INIT_FILE)] ||
        $::env(CYAN_EEPROM_INIT_FILE) eq ""} {
        fail "CYAN_EEPROM_INIT_FILE must name an external 128-word 93CS56 image"
    }
    set cyan_eeprom_init_file \
        [file normalize $::env(CYAN_EEPROM_INIT_FILE)]
    if {![file isfile $cyan_eeprom_init_file]} {
        fail "Missing EEPROM initialization input: $cyan_eeprom_init_file"
    }
    set eeprom_fd [open $cyan_eeprom_init_file r]
    set eeprom_data [read $eeprom_fd]
    close $eeprom_fd
    set eeprom_word_count 0
    foreach line [split $eeprom_data "\n"] {
        set word [string trim $line]
        if {$word eq ""} {
            continue
        }
        if {![regexp {^[0-9A-Fa-f]{4}$} $word]} {
            fail "Invalid EEPROM word at line [expr {$eeprom_word_count + 1}] in $cyan_eeprom_init_file; expected four hexadecimal digits"
        }
        incr eeprom_word_count
    }
    if {$eeprom_word_count != 128} {
        fail "Invalid EEPROM initialization input $cyan_eeprom_init_file: expected 128 16-bit words, found $eeprom_word_count"
    }
}

proc read_file {path} {
    set fd [open $path r]
    set data [read $fd]
    close $fd
    return $data
}

proc write_file {path data} {
    file mkdir [file dirname $path]
    set fd [open $path w]
    puts -nonewline $fd $data
    close $fd
}

proc file_sha256 {path} {
    if {![file exists $path] || [file isdirectory $path]} {
        fail "Cannot fingerprint missing build input: $path"
    }
    if {[catch {exec sha256sum -- $path} result]} {
        fail "Cannot fingerprint build input $path: $result"
    }
    return [lindex $result 0]
}

proc collect_files {root} {
    set out {}
    if {![file exists $root]} {
        return $out
    }
    foreach path [glob -nocomplain -directory $root *] {
        if {[file isdirectory $path]} {
            set out [concat $out [collect_files $path]]
        } else {
            lappend out [file normalize $path]
        }
    }
    return $out
}

proc path_starts_with {path prefix} {
    set path [file normalize $path]
    set prefix [file normalize $prefix]
    return [expr {[string range $path 0 [expr {[string length $prefix] - 1}]] eq $prefix}]
}

proc unique_list {items} {
    set out {}
    foreach item $items {
        if {[lsearch -exact $out $item] < 0} {
            lappend out $item
        }
    }
    return $out
}

proc source_file_by_tail {tail} {
    set matches {}
    foreach f [get_files -quiet -of_objects [get_filesets sources_1]] {
        set nf [file normalize $f]
        if {[file tail $nf] eq $tail && [file exists $nf]} {
            lappend matches $nf
        }
    }
    set matches [unique_list $matches]
    if {[llength $matches] == 1} {
        return [lindex $matches 0]
    }
    fail "Expected exactly one generated source named $tail, found [llength $matches]: $matches"
}

proc constraint_file_by_tail {tail} {
    set matches {}
    foreach f [get_files -quiet -of_objects [get_filesets constrs_1]] {
        set nf [file normalize $f]
        if {[file tail $nf] eq $tail && [file exists $nf]} {
            lappend matches $nf
        }
    }
    set matches [unique_list $matches]
    if {[llength $matches] == 1} {
        return [lindex $matches 0]
    }
    fail "Expected exactly one generated constraint named $tail, found [llength $matches]: $matches"
}

proc source_files_for_materialize {ip_dir} {
    set out {}
    foreach f [get_files -quiet -of_objects [get_filesets sources_1]] {
        set nf [file normalize $f]
        if {![file exists $nf] || [file isdirectory $nf]} {
            continue
        }
        if {[path_starts_with $nf $ip_dir]} {
            continue
        }
        set ext [string tolower [file extension $nf]]
        if {$ext in {".sv" ".v" ".vh" ".mem" ".txt" ".hex"}} {
            lappend out $nf
        }
    }
    return [unique_list $out]
}

proc rename_generated_text {data} {
    return [string map [list \
        "example_top" "kcu105_cyan_top" \
        "example_tb" "kcu105_cyan_traffic" \
        "u_example_tb" "u_cyan_traffic" \
        "EXAMPLE_TB" "CYAN_TRAFFIC" \
        "EXAMPLE TB" "CYAN TRAFFIC" \
        "EXAMPLE DESIGN" "CYAN KCU105 TARGET" \
        "Example Design" "Cyan KCU105 Target" \
        "example design" "Cyan KCU105 target" \
        "example testbench" "Cyan traffic generator" \
        "example test-bench" "Cyan traffic generator" \
        "example TB" "Cyan traffic generator" \
        "EXAMPLE" "CYAN TARGET" \
        "Example" "Cyan target" \
        "example" "Cyan target" \
    ] $data]
}

proc copy_renamed_text_file {src dst} {
    write_file $dst [rename_generated_text [read_file $src]]
}

proc augment_cyan_top {path} {
    set data [read_file $path]
    set data [string map [list \
        "    output                  c0_init_calib_complete,\n    output                  c0_data_compare_error,\n" \
        "    output [7:0]            gpio_led,\n" \
        "  wire                  c0_ddr4_rst;\n  wire                  dbg_clk;\n" \
        "  wire                  c0_ddr4_rst;\n  wire                  dbg_clk;\n  wire                  c0_init_calib_complete;\n  wire                  c0_data_compare_error;\n" \
        "  wire                  c0_wr_rd_complete;\n" \
        "  wire                  c0_wr_rd_complete;\n\n  reg [25:0] cyan_heartbeat;\n\n  always @(posedge c0_ddr4_clk) begin\n    if (c0_ddr4_rst) begin\n      cyan_heartbeat <= 26'd0;\n    end else begin\n      cyan_heartbeat <= cyan_heartbeat + 26'd1;\n    end\n  end\n\n  assign gpio_led[0] = c0_init_calib_complete;\n  assign gpio_led[1] = c0_wr_rd_complete;\n  assign gpio_led[2] = c0_data_compare_error;\n  assign gpio_led[3] = c0_ddr4_app_rdy;\n  assign gpio_led[4] = c0_ddr4_app_wdf_rdy;\n  assign gpio_led[5] = c0_ddr4_app_rd_data_valid;\n  assign gpio_led[6] = ~c0_ddr4_rst;\n  assign gpio_led[7] = cyan_heartbeat[25];\n" \
    ] $data]
    write_file $path $data
}

proc append_cyan_constraints {path} {
    set data [read_file $path]
    append data "\n# KCU105 configuration bank is wired for 1.8 V configuration.\n"
    append data "set_property CFGBVS GND \[current_design\]\n"
    append data "set_property CONFIG_VOLTAGE 1.8 \[current_design\]\n"

    append data "\n# Preserve the DDR4 IP recommendation after board-pin constraints are applied.\n"
    append data "set_property DRIVE 8 \[get_ports c0_ddr4_reset_n\]\n"

    append data "\n# KCU105 GPIO LEDs are active-high.\n"
    foreach item {
        {0 AP8}
        {1 H23}
        {2 P20}
        {3 P21}
        {4 N22}
        {5 M22}
        {6 R23}
        {7 P23}
    } {
        set index [lindex $item 0]
        set pin [lindex $item 1]
        append data [format {set_property PACKAGE_PIN %s [get_ports {gpio_led[%d]}]
} $pin $index]
        append data [format {set_property IOSTANDARD LVCMOS18 [get_ports {gpio_led[%d]}]
} $index]
        append data [format {set_property DRIVE 8 [get_ports {gpio_led[%d]}]
} $index]
    }

    write_file $path $data
}

proc write_cyan_soc_constraints {path} {
    set data "# KCU105 CP2105 USB-UART, standard channel.\n"
    foreach item {
        {uart_txd K26}
        {uart_rxd G25}
        {uart_rts K27}
        {uart_cts L23}
    } {
        set port [lindex $item 0]
        set pin [lindex $item 1]
        append data [format {set_property PACKAGE_PIN %s [get_ports %s]
set_property IOSTANDARD LVCMOS18 [get_ports %s]
} $pin $port $port]
    }

    append data "\n# KCU105 J73 cooling fan low-side PWM gate.\n"
    append data "# Without an FPGA driver, R422 pulls this signal high at full speed.\n"
    append data "set_property PACKAGE_PIN AJ9 \[get_ports sm_fan_pwm\]\n"
    append data "set_property IOSTANDARD LVCMOS18 \[get_ports sm_fan_pwm\]\n"
    append data "set_property DRIVE 8 \[get_ports sm_fan_pwm\]\n"
    append data "set_property SLEW SLOW \[get_ports sm_fan_pwm\]\n"

    append data "\n# KCU105 onboard Marvell 88E1111 SGMII-over-LVDS PHY.\n"
    foreach item {
        {sgmii_txn M24 DIFF_HSTL_I_18}
        {sgmii_txp N24 DIFF_HSTL_I_18}
        {sgmii_rxn P25 DIFF_HSTL_I_18}
        {sgmii_rxp P24 DIFF_HSTL_I_18}
        {sgmii_refclk_p P26 LVDS_25}
        {sgmii_refclk_n N26 LVDS_25}
        {phy_mdc L25 LVCMOS18}
        {phy_mdio H26 LVCMOS18}
        {phy_rst_n J23 LVCMOS18}
    } {
        set port [lindex $item 0]
        set pin [lindex $item 1]
        set iostd [lindex $item 2]
        append data [format {set_property PACKAGE_PIN %s [get_ports %s]
set_property IOSTANDARD %s [get_ports %s]
} $pin $port $iostd $port]
    }
    append data "set_property PULLUP true \[get_ports phy_mdio\]\n"
    append data "create_clock -name sgmii_refclk625 -period 1.600 \[get_ports sgmii_refclk_p\]\n"

    append data "\n# KCU105 onboard micro-SD J83 through SDIO mux, SPI mode.\n"
    append data "# SYSCTLR_SDIO_MUX_SEL must select FPGA U1 before these pins see the card.\n"
    foreach item {
        {sdio_cmd_fpga AD9 output}
        {sdio_clk_fpga AL10 output}
        {sdio_data0_fpga AP9 input}
        {sdio_data1_fpga AN9 inout_pullup}
        {sdio_data2_fpga AH9 inout_pullup}
        {sdio_data3_fpga AH8 output_pullup}
        {sdio_cd_fpga AM10 input_pullup}
    } {
        set port [lindex $item 0]
        set pin [lindex $item 1]
        set kind [lindex $item 2]
        append data [format {set_property PACKAGE_PIN %s [get_ports %s]
set_property IOSTANDARD LVCMOS18 [get_ports %s]
} $pin $port $port]
        if {$kind eq "output" || $kind eq "output_pullup"} {
            append data [format {set_property DRIVE 8 [get_ports %s]
} $port]
        }
        if {$kind eq "inout_pullup" || $kind eq "input_pullup" || $kind eq "output_pullup"} {
            append data [format {set_property PULLUP true [get_ports %s]
} $port]
        }
    }
    append data {

# KCU105 onboard ADV7511 parallel video and control bus.
set_property PACKAGE_PIN AF13 [get_ports hdmi_out_clk]
set_property PACKAGE_PIN AE13 [get_ports hdmi_hsync]
set_property PACKAGE_PIN AH13 [get_ports hdmi_vsync]
set_property PACKAGE_PIN AE11 [get_ports hdmi_data_e]
set_property PACKAGE_PIN AK11 [get_ports {hdmi_data[0]}]
set_property PACKAGE_PIN AP11 [get_ports {hdmi_data[1]}]
set_property PACKAGE_PIN AP13 [get_ports {hdmi_data[2]}]
set_property PACKAGE_PIN AN13 [get_ports {hdmi_data[3]}]
set_property PACKAGE_PIN AN11 [get_ports {hdmi_data[4]}]
set_property PACKAGE_PIN AM11 [get_ports {hdmi_data[5]}]
set_property PACKAGE_PIN AN12 [get_ports {hdmi_data[6]}]
set_property PACKAGE_PIN AM12 [get_ports {hdmi_data[7]}]
set_property PACKAGE_PIN AL12 [get_ports {hdmi_data[8]}]
set_property PACKAGE_PIN AK12 [get_ports {hdmi_data[9]}]
set_property PACKAGE_PIN AL13 [get_ports {hdmi_data[10]}]
set_property PACKAGE_PIN AK13 [get_ports {hdmi_data[11]}]
set_property PACKAGE_PIN AD11 [get_ports {hdmi_data[12]}]
set_property PACKAGE_PIN AH12 [get_ports {hdmi_data[13]}]
set_property PACKAGE_PIN AG12 [get_ports {hdmi_data[14]}]
set_property PACKAGE_PIN AJ11 [get_ports {hdmi_data[15]}]
set_property PACKAGE_PIN AE12 [get_ports hdmi_spdif]
set_property PACKAGE_PIN J24 [get_ports hdmi_iic_scl]
set_property PACKAGE_PIN J25 [get_ports hdmi_iic_sda]
set_property PACKAGE_PIN AP10 [get_ports hdmi_iic_mux_reset_n]
set_property IOSTANDARD LVCMOS18 [get_ports {hdmi_out_clk hdmi_hsync hdmi_vsync hdmi_data_e hdmi_data[*] hdmi_spdif hdmi_iic_scl hdmi_iic_sda hdmi_iic_mux_reset_n}]
set_property DRIVE 8 [get_ports {hdmi_out_clk hdmi_hsync hdmi_vsync hdmi_data_e hdmi_data[*] hdmi_spdif}]
set_property DRIVE 8 [get_ports hdmi_iic_mux_reset_n]
set_property SLEW SLOW [get_ports hdmi_iic_mux_reset_n]
set_property SLEW FAST [get_ports {hdmi_out_clk hdmi_hsync hdmi_vsync hdmi_data_e hdmi_data[*]}]
set_property PULLUP true [get_ports {hdmi_iic_scl hdmi_iic_sda}]
}
    write_file $path $data
}

proc write_cyan_soc_post_link_hook {path} {
    set data {
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
}
    write_file $path $data
}

proc configure_cyan_impl_hooks {} {
    global origin_dir enable_soc

    if {!$enable_soc} {
        return
    }
    set hook [file join $origin_dir constraints kcu105_cyan_soc_post_link.tcl]
    set run [get_runs -quiet impl_1]
    if {$run eq ""} {
        return
    }
    foreach prop {
        STEPS.OPT_DESIGN.TCL.PRE
        STEPS.PLACE_DESIGN.TCL.PRE
        STEPS.ROUTE_DESIGN.TCL.PRE
    } {
        set_property $prop $hook $run
    }
}

proc configure_cyan_constraints {} {
    global origin_dir

    set xdc [file join $origin_dir constraints kcu105_cyan.xdc]
    set soc_xdc [file join $origin_dir constraints kcu105_cyan_soc.xdc]
    foreach candidate [list $xdc $soc_xdc] {
        # Query the constraint set itself.  A generated IP can make get_files
        # find an identically sourced file globally without placing this board
        # XDC in constrs_1, silently leaving top-level pins unconstrained.
        set files [get_files -quiet -of_objects [get_filesets constrs_1] $candidate]
        if {[llength $files] == 0 && [file exists $candidate]} {
            add_files -fileset constrs_1 $candidate
            set files [get_files -quiet -of_objects [get_filesets constrs_1] $candidate]
        }
        if {[llength $files] != 1} {
            fail "Expected exactly one constraint-set entry for $candidate, found [llength $files]"
        }
        if {[llength $files] == 1} {
            set_property PROCESSING_ORDER LATE [lindex $files 0]
            if {[file normalize $candidate] eq [file normalize $soc_xdc]} {
                set_property USED_IN_SYNTHESIS false [lindex $files 0]
            }
        }
    }
}

proc assert_ip_prop {ip prop expected} {
    set actual [get_property $prop $ip]
    if {$actual ne $expected} {
        fail "DDR4 IP binding mismatch: $prop is '$actual', expected '$expected'"
    }
}

proc assert_no_custom_board_bindings {ip} {
    foreach prop {CONFIG.C0_DDR4_BOARD_INTERFACE CONFIG.C0_CLOCK_BOARD_INTERFACE CONFIG.RESET_BOARD_INTERFACE} {
        set actual [get_property $prop $ip]
        if {$actual eq "Custom"} {
            fail "DDR4 IP is not board-bound: $prop is Custom"
        }
    }
}

proc assert_ip_board_bindings {ip} {
    assert_ip_prop $ip CONFIG.C0_DDR4_BOARD_INTERFACE ddr4_sdram
    assert_ip_prop $ip CONFIG.C0_CLOCK_BOARD_INTERFACE default_sysclk_300
    assert_ip_prop $ip CONFIG.RESET_BOARD_INTERFACE reset
    assert_no_custom_board_bindings $ip
}

proc get_xdc_pin_map {files} {
    set pins [dict create]
    foreach file $files {
        if {![file exists $file]} {
            continue
        }
        foreach line [split [read_file $file] "\n"] {
            set trimmed [string trim $line]
            if {[regexp {^set_property[ \t]+PACKAGE_PIN[ \t]+([A-Z0-9]+)[ \t]+\[[ \t]*get_ports[ \t]+(.+)[ \t]*\]} $trimmed -> pin port_expr]} {
                set port_expr [string trim $port_expr]
                set port_expr [string trim $port_expr "\"{}"]
                dict set pins $port_expr [list $pin $file]
            }
        }
    }
    return $pins
}

proc get_xdc_board_pin_map {files} {
    set pins [dict create]
    foreach file $files {
        if {![file exists $file]} {
            continue
        }
        foreach line [split [read_file $file] "\n"] {
            set trimmed [string trim $line]
            if {[regexp {^set_property[ \t]+BOARD_PIN[ \t]+\{?([A-Za-z0-9_]+)\}?[ \t]+\[[ \t]*get_ports[ \t]+(.+)[ \t]*\]} $trimmed -> board_pin port_expr]} {
                set port_expr [string trim $port_expr]
                set port_expr [string trim $port_expr "\"{}"]
                dict set pins $port_expr [list $board_pin $file]
            }
        }
    }
    return $pins
}

proc get_board_pin_locs {pins_xml} {
    if {![file exists $pins_xml]} {
        fail "Missing KCU105 board pin map: $pins_xml"
    }
    set locs [dict create]
    foreach line [split [read_file $pins_xml] "\n"] {
        if {[regexp {<pin[^>]+name[ \t]*=[ \t]*"([^"]+)"[^>]+loc[ \t]*=[ \t]*"([^"]+)"} $line -> name loc]} {
            dict set locs $name $loc
        }
    }
    return $locs
}

proc assert_xdc_pin {pin_map port expected} {
    if {![dict exists $pin_map $port]} {
        fail "Missing PACKAGE_PIN constraint for $port"
    }
    set value [dict get $pin_map $port]
    set actual [lindex $value 0]
    set source [lindex $value 1]
    if {$actual ne $expected} {
        fail "Wrong PACKAGE_PIN for $port: $actual in $source, expected $expected"
    }
}

proc assert_board_pin_loc {board_pin_map board_pin_locs ports expected_board_pin expected_loc} {
    set found 0
    set port ""
    set value ""
    foreach candidate $ports {
        if {[dict exists $board_pin_map $candidate]} {
            set found 1
            set port $candidate
            set value [dict get $board_pin_map $candidate]
            break
        }
    }
    if {!$found} {
        fail "Missing BOARD_PIN constraint for one of: $ports"
    }
    set actual_board_pin [lindex $value 0]
    set source [lindex $value 1]
    if {$actual_board_pin ne $expected_board_pin} {
        fail "Wrong BOARD_PIN for $port: $actual_board_pin in $source, expected $expected_board_pin"
    }
    if {![dict exists $board_pin_locs $actual_board_pin]} {
        fail "BOARD_PIN $actual_board_pin is not present in KCU105 part0_pins.xml"
    }
    set actual_loc [dict get $board_pin_locs $actual_board_pin]
    if {$actual_loc ne $expected_loc} {
        fail "Wrong KCU105 LOC for $actual_board_pin: $actual_loc, expected $expected_loc"
    }
}

proc validate_cyan_target_files {} {
    global origin_dir board_pins_xml

    set xci [file join $origin_dir ip kcu105_ddr4 kcu105_ddr4.xci]
    if {![file exists $xci]} {
        fail "Missing copied DDR4 IP: $xci"
    }
    set xci_data [read_file $xci]
    foreach pair {
        {PARAM_VALUE.C0_DDR4_BOARD_INTERFACE ddr4_sdram}
        {PARAM_VALUE.C0_CLOCK_BOARD_INTERFACE default_sysclk_300}
        {PARAM_VALUE.RESET_BOARD_INTERFACE reset}
    } {
        set prop [lindex $pair 0]
        set expected [lindex $pair 1]
        set needle "spirit:referenceId=\"$prop\">$expected</spirit:configurableElementValue>"
        if {[string first $needle $xci_data] < 0} {
            fail "Copied XCI is not board-bound: $prop is not $expected"
        }
    }
    foreach prop {PARAM_VALUE.C0_DDR4_BOARD_INTERFACE PARAM_VALUE.C0_CLOCK_BOARD_INTERFACE PARAM_VALUE.RESET_BOARD_INTERFACE} {
        set needle "spirit:referenceId=\"$prop\">Custom</spirit:configurableElementValue>"
        if {[string first $needle $xci_data] >= 0} {
            fail "Copied XCI still contains a Custom KCU105 DDR4 board interface: $prop"
        }
    }

    set xdc_files [collect_files [file join $origin_dir constraints]]
    set xdc_files [concat $xdc_files [collect_files [file join $origin_dir ip kcu105_ddr4]]]
    set only_xdc {}
    foreach f $xdc_files {
        if {[string tolower [file extension $f]] eq ".xdc"} {
            lappend only_xdc $f
        }
    }
    set board_pin_map [get_xdc_board_pin_map $only_xdc]
    set board_pin_locs [get_board_pin_locs $board_pins_xml]
    set package_pin_map [get_xdc_pin_map $only_xdc]
    assert_board_pin_loc $board_pin_map $board_pin_locs {c0_sys_clk_p} sysclk_300_p AK17
    assert_board_pin_loc $board_pin_map $board_pin_locs {c0_sys_clk_n} sysclk_300_n AK16
    assert_board_pin_loc $board_pin_map $board_pin_locs {sys_rst} CPU_RESET AN8
    assert_board_pin_loc $board_pin_map $board_pin_locs {c0_ddr4_act_n} c0_ddr4_act_n AH14
    assert_board_pin_loc $board_pin_map $board_pin_locs {{c0_ddr4_adr[0]}} c0_ddr4_adr0 AE17
    assert_board_pin_loc $board_pin_map $board_pin_locs {{c0_ddr4_ck_t[0]} c0_ddr4_ck_t} c0_ddr4_ck_t AE16
    assert_board_pin_loc $board_pin_map $board_pin_locs {{c0_ddr4_ck_c[0]} c0_ddr4_ck_c} c0_ddr4_ck_c AE15
    assert_board_pin_loc $board_pin_map $board_pin_locs {c0_ddr4_reset_n} c0_ddr4_reset_n AL18
    assert_xdc_pin $package_pin_map {gpio_led[0]} AP8
    assert_xdc_pin $package_pin_map {gpio_led[1]} H23
    assert_xdc_pin $package_pin_map {gpio_led[2]} P20
    assert_xdc_pin $package_pin_map {gpio_led[3]} P21
    assert_xdc_pin $package_pin_map {gpio_led[4]} N22
    assert_xdc_pin $package_pin_map {gpio_led[5]} M22
    assert_xdc_pin $package_pin_map {gpio_led[6]} R23
    assert_xdc_pin $package_pin_map {gpio_led[7]} P23
    assert_xdc_pin $package_pin_map sm_fan_pwm AJ9
    assert_xdc_pin $package_pin_map sgmii_txn M24
    assert_xdc_pin $package_pin_map sgmii_txp N24
    assert_xdc_pin $package_pin_map sgmii_rxn P25
    assert_xdc_pin $package_pin_map sgmii_rxp P24
    assert_xdc_pin $package_pin_map sgmii_refclk_p P26
    assert_xdc_pin $package_pin_map sgmii_refclk_n N26
    assert_xdc_pin $package_pin_map phy_mdc L25
    assert_xdc_pin $package_pin_map phy_mdio H26
    assert_xdc_pin $package_pin_map phy_rst_n J23
    assert_xdc_pin $package_pin_map hdmi_iic_mux_reset_n AP10

    set cyan_xdc [file join $origin_dir constraints kcu105_cyan.xdc]
    set cyan_xdc_data [read_file $cyan_xdc]
    foreach needle {
        {set_property CFGBVS GND [current_design]}
        {set_property CONFIG_VOLTAGE 1.8 [current_design]}
        {set_property DRIVE 8 [get_ports c0_ddr4_reset_n]}
    } {
        if {[string first $needle $cyan_xdc_data] < 0} {
            fail "Missing required KCU105 Cyan constraint: $needle"
        }
    }

    foreach file [list \
        [file join $origin_dir src kcu105_cyan_top.sv] \
        [file join $origin_dir src kcu105_cyan_traffic.sv] \
        [file join $origin_dir constraints kcu105_cyan.xdc] \
        [file join $origin_dir constraints kcu105_cyan_soc_post_link.tcl] \
        [file join $origin_dir cpu_island_cdc_constraints.tcl] \
        [file join $origin_dir lg1_serial_cdc_constraints.tcl] \
        [file join $origin_dir sgmii_rx_gearbox_constraints.tcl] \
        [file join $origin_dir signoff_cpu_island_cdc_routed.tcl] \
        [file join $origin_dir signoff_sgmii_rx_gearbox_routed.tcl] \
    ] {
        if {![file exists $file]} {
            fail "Missing Cyan target file: $file"
        }
        if {[regexp {example_(top|tb)} [read_file $file]]} {
            fail "Repo-facing Cyan target file still contains example_* names: $file"
        }
    }

    foreach file [concat [collect_files [file join $origin_dir src]] [collect_files [file join $origin_dir constraints]]] {
        set ext [string tolower [file extension $file]]
        if {$ext in {".sv" ".v" ".vh" ".xdc"} && [regexp -nocase {example} [read_file $file]]} {
            fail "Repo-facing Cyan target file still contains example wording: $file"
        }
    }
}

proc soc_rtl_files {} {
    global origin_dir
    set repo_dir [file normalize [file join $origin_dir ../..]]
    return [list \
        [file join $repo_dir rtl/common/cyan_simple_dual_port_ram.v] \
        [file join $repo_dir rtl/common/cyan_sync_byte_ram.v] \
        [file join $repo_dir rtl/cpu/r4000_top.v] \
        [file join $repo_dir rtl/cpu/r4000_core.v] \
        [file join $repo_dir rtl/cpu/r4000_alu.v] \
        [file join $repo_dir rtl/cpu/r4000_muldiv.v] \
        [file join $repo_dir rtl/cpu/r4000_fpu.v] \
        [file join $repo_dir rtl/cpu/r4000_tlb.v] \
        [file join $repo_dir rtl/cpu/r4000_icache.v] \
        [file join $repo_dir rtl/cpu/r4000_dcache.v] \
        [file join $repo_dir rtl/cpu/r4000_scache.v] \
        [file join $repo_dir rtl/soc/mc.v] \
        [file join $repo_dir rtl/soc/ip20_misc.v] \
        [file join $repo_dir rtl/soc/scsi_raw_disk_target.v] \
        [file join $repo_dir rtl/soc/sd_spi_block.v] \
        [file join $repo_dir rtl/soc/sd_scsi_bridge.v] \
        [file join $repo_dir rtl/hpc1/eeprom_93cs56.v] \
        [file join $repo_dir rtl/hpc1/wd33c93.v] \
        [file join $repo_dir rtl/hpc1/scc_z8530.v] \
        [file join $repo_dir rtl/hpc1/hpc1_enet.v] \
        [file join $repo_dir rtl/hpc1/hpc1_scsi_dma.v] \
        [file join $repo_dir rtl/hpc1/hpc1_dma_arbiter.v] \
        [file join $repo_dir rtl/hpc1/int2.v] \
        [file join $repo_dir rtl/hpc1/hpc1.v] \
        [file join $repo_dir rtl/dsp/dsp56001_decode.v] \
        [file join $repo_dir rtl/dsp/dsp56001_alu.v] \
        [file join $repo_dir rtl/dsp/dsp56001_agu.v] \
        [file join $repo_dir rtl/dsp/dsp56001_mem.v] \
        [file join $repo_dir rtl/dsp/dsp56001_periph.v] \
        [file join $repo_dir rtl/dsp/dsp56001_host.v] \
        [file join $repo_dir rtl/dsp/dsp56001_dma.v] \
        [file join $repo_dir rtl/dsp/dsp56001_sram.v] \
        [file join $repo_dir rtl/dsp/dsp56001_core.v] \
        [file join $repo_dir rtl/dsp/dsp56001_top.v] \
        [file join $repo_dir rtl/board/ip20_prom_rom.v] \
        [file join $repo_dir rtl/board/mc_mig_bridge.v] \
        [file join $repo_dir rtl/board/gmii_ethernet_mac.v] \
        [file join $repo_dir rtl/board/kcu105_sgmii_ethernet.v] \
        [file join $repo_dir rtl/board/kcu105_lg1_mig_bridge.v] \
        [file join $repo_dir rtl/board/kcu105_lg1_dual_port_mig_bridge.v] \
        [file join $repo_dir rtl/board/kcu105_lg1_port_arbiter.v] \
        [file join $repo_dir rtl/board/kcu105_lg1_capture_reader.v] \
        [file join $repo_dir rtl/board/kcu105_lg1_capture_bist.v] \
        [file join $repo_dir rtl/board/kcu105_adv7511_tx.v] \
        [file join $repo_dir rtl/board/cyan_spdif_tx.v] \
        [file join $repo_dir rtl/board/kcu105_hdmi_frame_resampler.v] \
        [file join $repo_dir rtl/board/kcu105_mig_app_arbiter.v] \
        [file join $repo_dir rtl/lg1/lg1_rex_frontend.v] \
        [file join $repo_dir rtl/lg1/lg1_rex_context_queue.v] \
        [file join $repo_dir rtl/lg1/lg1_draw_iter.v] \
        [file join $repo_dir rtl/lg1/lg1_rb_backend.v] \
        [file join $repo_dir rtl/lg1/lg1_mem_arb_ssram.v] \
        [file join $repo_dir rtl/lg1/lg1_serial_cdc.v] \
        [file join $repo_dir rtl/lg1/lg1_scanout_line_fetch.v] \
        [file join $repo_dir rtl/lg1/lg1_vc1_ram_64k.v] \
        [file join $repo_dir rtl/lg1/lg1_vc1_xmap_dac_scanout.v] \
        [file join $repo_dir rtl/lg1/lg1_clean_core.v] \
        [file join $repo_dir rtl/lg1/lg1_write_combiner_64b.v] \
        [file join $repo_dir rtl/lg1/lg1_line_cache_64b.v] \
        [file join $repo_dir rtl/lg1/lg1_mem_line_adapter_64b.v] \
        [file join $repo_dir rtl/lg1/lg1_clean_line_core.v] \
        [file join $repo_dir rtl/board/cyan_mgmt_frame_router.v] \
        [file join $repo_dir rtl/board/cyan_mgmt_endpoint.v] \
        [file join $repo_dir rtl/board/cyan_frame_arbiter.v] \
        [file join $repo_dir rtl/board/cyan_sgi_input_device.v] \
        [file join $repo_dir rtl/board/cyan_lg1_config_event_encoder.v] \
        [file join $repo_dir rtl/board/cyan_lg1_config_trace.v] \
        [file join $repo_dir rtl/board/cyan_cpu_island_bridge.v] \
        [file join $repo_dir rtl/board/cyan_hpc_dma_coordinator.v] \
        [file join $repo_dir rtl/board/ip20_kcu_soc_core.v] \
    ]
}

proc soc_mem_files {} {
    global cyan_prom_dir cyan_eeprom_init_file
    set out [list $cyan_eeprom_init_file]
    foreach tail {
        cyan_firmware.memh
        cyan_firmware_b00.memh
        cyan_firmware_b01.memh
        cyan_firmware_b02.memh
        cyan_firmware_b03.memh
        cyan_firmware_b04.memh
        cyan_firmware_b05.memh
        cyan_firmware_b06.memh
        cyan_firmware_b07.memh
        cyan_firmware_b08.memh
        cyan_firmware_b09.memh
        cyan_firmware_b0a.memh
        cyan_firmware_b0b.memh
        cyan_firmware_b0c.memh
        cyan_firmware_b0d.memh
        cyan_firmware_b0e.memh
        cyan_firmware_b0f.memh
    } {
        lappend out [file join $cyan_prom_dir $tail]
    }
    return $out
}

proc soc_top_generics {} {
    global cyan_prom_dir cyan_eeprom_init_file
    set out [list \
        CPU_CLK_DIVIDE=3 \
        PROM_INIT_FILE=[file join $cyan_prom_dir cyan_firmware.memh] \
        EEPROM_INIT_FILE=$cyan_eeprom_init_file \
    ]
    foreach suffix {B00 B01 B02 B03 B04 B05 B06 B07 B08 B09 B0A B0B B0C B0D B0E B0F} tail {
        cyan_firmware_b00.memh
        cyan_firmware_b01.memh
        cyan_firmware_b02.memh
        cyan_firmware_b03.memh
        cyan_firmware_b04.memh
        cyan_firmware_b05.memh
        cyan_firmware_b06.memh
        cyan_firmware_b07.memh
        cyan_firmware_b08.memh
        cyan_firmware_b09.memh
        cyan_firmware_b0a.memh
        cyan_firmware_b0b.memh
        cyan_firmware_b0c.memh
        cyan_firmware_b0d.memh
        cyan_firmware_b0e.memh
        cyan_firmware_b0f.memh
    } {
        lappend out PROM_INIT_${suffix}=[file join $cyan_prom_dir $tail]
    }
    return $out
}

proc enable_build_file {path} {
    foreach file [get_files -quiet $path] {
        catch {set_property IS_ENABLED true $file}
        catch {set_property is_enabled true $file}
    }
}

proc configure_sgmii_ip {} {
    set ips [get_ips -quiet cyan_sgmii_pcs_pma]
    if {[llength $ips] == 0} {
        create_ip -name gig_ethernet_pcs_pma -vendor xilinx.com -library ip \
            -module_name cyan_sgmii_pcs_pma
        set ips [get_ips -quiet cyan_sgmii_pcs_pma]
    }
    if {[llength $ips] != 1} {
        fail "Expected exactly one cyan_sgmii_pcs_pma IP, found [llength $ips]"
    }
    set ip [lindex $ips 0]
    set old_lvds_refclk [get_property CONFIG.LvdsRefClk $ip]
    set_property -dict [list \
        CONFIG.Standard {SGMII} \
        CONFIG.Physical_Interface {LVDS} \
        CONFIG.SupportLevel {Include_Shared_Logic_in_Core} \
        CONFIG.USE_BOARD_FLOW {true} \
        CONFIG.ETHERNET_BOARD_INTERFACE {sgmii_lvds} \
        CONFIG.DIFFCLK_BOARD_INTERFACE {sgmii_phyclk} \
        CONFIG.MDIO_BOARD_INTERFACE {mdio_mdc} \
        CONFIG.LvdsRefClk {625} \
    ] $ip

    foreach pair {
        {CONFIG.Standard SGMII}
        {CONFIG.Physical_Interface LVDS}
        {CONFIG.SupportLevel Include_Shared_Logic_in_Core}
        {CONFIG.USE_BOARD_FLOW true}
        {CONFIG.ETHERNET_BOARD_INTERFACE sgmii_lvds}
        {CONFIG.DIFFCLK_BOARD_INTERFACE sgmii_phyclk}
        {CONFIG.MDIO_BOARD_INTERFACE mdio_mdc}
        {CONFIG.LvdsRefClk 625}
    } {
        set prop [lindex $pair 0]
        set expected [lindex $pair 1]
        set actual [get_property $prop $ip]
        if {$actual ne $expected} {
            fail "SGMII IP binding mismatch: $prop is '$actual', expected '$expected'"
        }
    }

    if {$old_lvds_refclk ne "625"} {
        puts "SGMII IP LvdsRefClk changed from '$old_lvds_refclk' to '625'; resetting generated IP targets"
        catch {reset_target all $ip}
    }
    generate_target all $ip
    export_ip_user_files -of_objects $ip -no_script -sync -force -quiet
    if {![sgmii_generated_ports_are_current $ip]} {
        puts "SGMII IP generated wrapper/stub ports are stale; resetting generated IP targets"
        catch {reset_target all $ip}
        generate_target all $ip
        export_ip_user_files -of_objects $ip -no_script -sync -force -quiet
    }
    assert_sgmii_generated_ports $ip
}

proc sgmii_generated_ports_are_current {ip} {
    set ip_dir [file normalize [get_property IP_DIR $ip]]
    foreach file [list \
        [file join $ip_dir synth cyan_sgmii_pcs_pma.v] \
        [file join $ip_dir cyan_sgmii_pcs_pma_stub.v] \
    ] {
        if {![file exists $file]} {
            continue
        }
        set checked 1
        set data [read_file $file]
        if {[string first "refclk625_" $data] < 0 ||
            [string first "refclk125_" $data] >= 0} {
            return 0
        }
    }
    return 1
}

proc assert_sgmii_generated_ports {ip} {
    set ip_dir [file normalize [get_property IP_DIR $ip]]
    set checked 0
    foreach file [list \
        [file join $ip_dir synth cyan_sgmii_pcs_pma.v] \
        [file join $ip_dir cyan_sgmii_pcs_pma_stub.v] \
    ] {
        if {![file exists $file]} {
            continue
        }
        set checked 1
        set data [read_file $file]
        if {[string first "refclk625_" $data] < 0 ||
            [string first "refclk125_" $data] >= 0} {
            fail "SGMII IP generated ports are stale in $file; expected refclk625_* only"
        }
    }
    if {!$checked} {
        fail "SGMII IP did not generate expected wrapper/stub files under $ip_dir"
    }
}

proc soc_project_needs_recreate {} {
    global enable_soc
    if {!$enable_soc} {
        return 0
    }
    set ips [get_ips -quiet cyan_sgmii_pcs_pma]
    if {[llength $ips] == 0} {
        return 0
    }
    if {[llength $ips] != 1} {
        return 1
    }
    set ip [lindex $ips 0]
    if {[get_property CONFIG.LvdsRefClk $ip] ne "625"} {
        return 1
    }
    if {![sgmii_generated_ports_are_current $ip]} {
        return 1
    }
    return 0
}

proc soc_vio_generated_ports_are_current {ip} {
    set ip_dir [file normalize [get_property IP_DIR $ip]]
    set synth_wrapper [file join $ip_dir synth kcu105_cyan_soc_vio.v]
    if {![file exists $synth_wrapper]} {
        return 0
    }
    set data [read_file $synth_wrapper]
    return [expr {[string first "probe_in11" $data] >= 0 &&
                  [string first "probe_out0" $data] >= 0 &&
                  [string first "input \[127 : 0\] probe_in7" $data] >= 0 &&
                  [string first "input \[255 : 0\] probe_in8" $data] >= 0 &&
                  [string first "input \[255 : 0\] probe_in9" $data] >= 0 &&
                  [string first "input \[255 : 0\] probe_in10" $data] >= 0 &&
                  [string first "input \[255 : 0\] probe_in11" $data] >= 0 &&
                  [string first "output \[31 : 0\] probe_out0" $data] >= 0}]
}

proc reset_soc_vio_generated_products {ip} {
    set ip_dir [file normalize [get_property IP_DIR $ip]]
    foreach tail {
        kcu105_cyan_soc_vio_stub.v
        kcu105_cyan_soc_vio_stub.vhdl
        kcu105_cyan_soc_vio_sim_netlist.v
        kcu105_cyan_soc_vio_sim_netlist.vhdl
        kcu105_cyan_soc_vio.dcp
        synth/kcu105_cyan_soc_vio.v
    } {
        file delete -force [file join $ip_dir $tail]
    }
    catch {reset_target all $ip}
    set run [get_runs -quiet kcu105_cyan_soc_vio_synth_1]
    if {$run ne ""} {
        catch {reset_run $run}
    }
}

proc configure_soc_vio_stub_source {} {
    global origin_dir

    set stub [file join $origin_dir src kcu105_cyan_soc_vio_stub.v]
    if {![file exists $stub]} {
        fail "Missing KCU105 Cyan SoC VIO stub: $stub"
    }
    if {[llength [get_files -quiet $stub]] == 0} {
        add_files -quiet -fileset sources_1 $stub
    }
    set_property file_type Verilog [get_files $stub]
    enable_build_file $stub
}

proc configure_soc_debug_ip {} {
    global enable_soc enable_debug

    if {!$enable_soc || !$enable_debug} {
        return
    }

    set ips [get_ips -quiet kcu105_cyan_soc_vio]
    if {[llength $ips] == 0} {
        create_ip -name vio -vendor xilinx.com -library ip \
            -module_name kcu105_cyan_soc_vio
        set ips [get_ips -quiet kcu105_cyan_soc_vio]
    }
    if {[llength $ips] != 1} {
        fail "Expected exactly one kcu105_cyan_soc_vio IP, found [llength $ips]"
    }

    set ip [lindex $ips 0]
    set vio_props [list \
        CONFIG.C_NUM_PROBE_IN {12} \
        CONFIG.C_NUM_PROBE_OUT {1} \
        CONFIG.C_PROBE_IN0_WIDTH {32} \
        CONFIG.C_PROBE_IN1_WIDTH {32} \
        CONFIG.C_PROBE_IN2_WIDTH {32} \
        CONFIG.C_PROBE_IN3_WIDTH {32} \
        CONFIG.C_PROBE_IN4_WIDTH {128} \
        CONFIG.C_PROBE_IN5_WIDTH {256} \
        CONFIG.C_PROBE_IN6_WIDTH {256} \
        CONFIG.C_PROBE_IN7_WIDTH {128} \
        CONFIG.C_PROBE_IN8_WIDTH {256} \
        CONFIG.C_PROBE_IN9_WIDTH {256} \
        CONFIG.C_PROBE_IN10_WIDTH {256} \
        CONFIG.C_PROBE_IN11_WIDTH {256} \
        CONFIG.C_PROBE_OUT0_WIDTH {32} \
        CONFIG.C_PROBE_OUT0_INIT_VAL {00000000000000000000000000000000} \
    ]
    set needs_reset 0
    foreach {prop expected} $vio_props {
        if {[get_property $prop $ip] ne $expected} {
            set needs_reset 1
        }
    }
    set_property -dict $vio_props $ip
    if {$needs_reset || ![soc_vio_generated_ports_are_current $ip]} {
        puts "Resetting KCU105 Cyan SoC VIO generated products for current debug probe shape"
        reset_soc_vio_generated_products $ip
    }
    generate_target all $ip
    export_ip_user_files -of_objects $ip -no_script -sync -force -quiet
    if {![soc_vio_generated_ports_are_current $ip]} {
        fail "KCU105 Cyan SoC VIO generated synth wrapper is stale; expected probe_in7[127:0], probe_in8 through probe_in11[255:0], and probe_out0[31:0]"
    }
    configure_soc_vio_stub_source
}

proc configure_soc_fileset {} {
    global enable_soc enable_debug origin_dir part_name soc_build_signature
    if {!$enable_soc} {
        return
    }
    catch {set_property XPM_LIBRARIES {XPM_CDC XPM_FIFO XPM_MEMORY} [current_project]}
    set generics [soc_top_generics]
    set_property generic $generics [current_fileset]
    set defines {
        FPGA_XILINX=1
        FPGA_KCU105=1
        FPGA_L1_BRAM=1
        FPGA_DCACHE_BRAM=1
        FPGA_SCACHE=1
        FPGA_CPU_50MHZ=1
        FPGA_NO_MFI=1
    }
    if {$enable_debug} {
        lappend defines KCU105_CYAN_SOC_DEBUG=1
    }
    set signature_files [concat \
        [soc_rtl_files] \
        [soc_mem_files] \
        [list \
            [file join $origin_dir src kcu105_cyan_soc_top.sv] \
            [file join $origin_dir src kcu105_cyan_soc_vio_stub.v] \
            [file join $origin_dir constraints kcu105_cyan.xdc] \
            [file join $origin_dir constraints kcu105_cyan_soc.xdc] \
            [file join $origin_dir constraints kcu105_cyan_soc_post_link.tcl] \
            [file join $origin_dir cpu_island_cdc_constraints.tcl] \
            [file join $origin_dir lg1_serial_cdc_constraints.tcl] \
            [file join $origin_dir sgmii_rx_gearbox_constraints.tcl] \
        ] \
    ]
    foreach ip [get_ips -quiet] {
        set ip_file [get_property IP_FILE $ip]
        if {$ip_file ne ""} {
            lappend signature_files [file normalize $ip_file]
        }
    }
    set signature_lines [concat \
        [list "part=$part_name" "top=kcu105_cyan_soc_top"] \
        $generics \
        $defines \
    ]
    foreach path [lsort -unique $signature_files] {
        lappend signature_lines "[file normalize $path]=[file_sha256 $path]"
    }
    set soc_build_signature [join $signature_lines "\n"]
    set_property verilog_define $defines [current_fileset]
    foreach f [soc_rtl_files] {
        enable_build_file $f
    }
}

proc soc_synth_signature_stale {} {
    global enable_soc cyan_project_dir soc_build_signature

    if {!$enable_soc} {
        return 0
    }
    set stamp [file join $cyan_project_dir .cyan_soc_synth_signature]
    if {$soc_build_signature eq "" || ![file exists $stamp]} {
        return 1
    }
    return [expr {[read_file $stamp] ne $soc_build_signature}]
}

proc write_soc_synth_signature {} {
    global enable_soc cyan_project_dir soc_build_signature

    if {!$enable_soc || $soc_build_signature eq ""} {
        return
    }
    write_file [file join $cyan_project_dir .cyan_soc_synth_signature] $soc_build_signature
}

proc soc_impl_signature_stale {} {
    global enable_soc cyan_project_dir soc_build_signature

    if {!$enable_soc} {
        return 0
    }
    set stamp [file join $cyan_project_dir .cyan_soc_impl_signature]
    if {$soc_build_signature eq "" || ![file exists $stamp]} {
        return 1
    }
    return [expr {[read_file $stamp] ne $soc_build_signature}]
}

proc write_soc_impl_signature {} {
    global enable_soc cyan_project_dir soc_build_signature

    if {!$enable_soc || $soc_build_signature eq ""} {
        return
    }
    write_file [file join $cyan_project_dir .cyan_soc_impl_signature] $soc_build_signature
}

proc copy_generated_target_from_current_project {} {
    global origin_dir

    set ips [get_ips -quiet kcu105_ddr4]
    if {[llength $ips] != 1} {
        fail "Generated design does not contain exactly one kcu105_ddr4 IP"
    }
    set ip [lindex $ips 0]
    assert_ip_board_bindings $ip
    set ip_dir [file normalize [get_property IP_DIR $ip]]

    set src_dir [file join $origin_dir src]
    set vendor_dir [file join $src_dir vendor_ddr4_tg]
    set constraints_dir [file join $origin_dir constraints]
    set copied_ip_root [file join $origin_dir ip]
    set copied_ip_dir [file join $copied_ip_root kcu105_ddr4]
    set soc_top_path [file join $src_dir kcu105_cyan_soc_top.sv]
    set soc_vio_stub_path [file join $src_dir kcu105_cyan_soc_vio_stub.v]
    set soc_top_data ""
    set soc_vio_stub_data ""
    if {[file exists $soc_top_path]} {
        set soc_top_data [read_file $soc_top_path]
    }
    if {[file exists $soc_vio_stub_path]} {
        set soc_vio_stub_data [read_file $soc_vio_stub_path]
    }

    file delete -force $src_dir $constraints_dir $copied_ip_root
    file mkdir $vendor_dir
    file mkdir $constraints_dir
    file mkdir $copied_ip_root

    copy_renamed_text_file [source_file_by_tail example_top.sv] [file join $src_dir kcu105_cyan_top.sv]
    augment_cyan_top [file join $src_dir kcu105_cyan_top.sv]
    copy_renamed_text_file [source_file_by_tail example_tb.sv] [file join $src_dir kcu105_cyan_traffic.sv]
    if {$soc_top_data ne ""} {
        write_file $soc_top_path $soc_top_data
    }
    if {$soc_vio_stub_data ne ""} {
        write_file $soc_vio_stub_path $soc_vio_stub_data
    }
    copy_renamed_text_file [constraint_file_by_tail example_design.xdc] [file join $constraints_dir kcu105_cyan.xdc]
    append_cyan_constraints [file join $constraints_dir kcu105_cyan.xdc]
    write_cyan_soc_constraints [file join $constraints_dir kcu105_cyan_soc.xdc]
    write_cyan_soc_post_link_hook [file join $constraints_dir kcu105_cyan_soc_post_link.tcl]

    foreach src [source_files_for_materialize $ip_dir] {
        set tail [file tail $src]
        if {$tail in {"example_top.sv" "example_tb.sv"}} {
            continue
        }
        set ext [string tolower [file extension $src]]
        if {$ext in {".sv" ".v" ".vh"}} {
            copy_renamed_text_file $src [file join $vendor_dir $tail]
        } else {
            file copy -force $src [file join $vendor_dir $tail]
        }
    }

    file copy -force $ip_dir $copied_ip_dir
    validate_cyan_target_files
}

proc create_cyan_build_project {} {
    global origin_dir cyan_project_dir cyan_project_name part_name board_part enable_soc

    file delete -force $cyan_project_dir
    file mkdir [file dirname $cyan_project_dir]
    create_project -force $cyan_project_name $cyan_project_dir -part $part_name
    set_property board_part $board_part [current_project]

    set top_sv [file join $origin_dir src kcu105_cyan_top.sv]
    set soc_top_sv [file join $origin_dir src kcu105_cyan_soc_top.sv]
    set traffic_sv [file join $origin_dir src kcu105_cyan_traffic.sv]
    set xci [file join $origin_dir ip kcu105_ddr4 kcu105_ddr4.xci]
    set xdc [file join $origin_dir constraints kcu105_cyan.xdc]
    set soc_xdc [file join $origin_dir constraints kcu105_cyan_soc.xdc]

    if {$enable_soc} {
        add_files -quiet -fileset sources_1 [list $soc_top_sv $xci]
        configure_sgmii_ip
        configure_soc_debug_ip
        set_property file_type SystemVerilog [get_files $soc_top_sv]
        foreach f [concat [soc_rtl_files] [soc_mem_files]] {
            add_files -quiet -fileset sources_1 $f
            set ext [string tolower [file extension $f]]
            if {$ext in {".v" ".sv"}} {
                set_property file_type SystemVerilog [get_files $f]
            }
            enable_build_file $f
        }
    } else {
        add_files -quiet -fileset sources_1 [list $top_sv $traffic_sv $xci]
        set_property file_type SystemVerilog [get_files $top_sv]
        set_property file_type SystemVerilog [get_files $traffic_sv]
    }

    if {!$enable_soc} {
        foreach f [collect_files [file join $origin_dir src vendor_ddr4_tg]] {
            set ext [string tolower [file extension $f]]
            if {$ext in {".sv" ".v" ".vh" ".mem" ".txt" ".hex"}} {
                add_files -quiet -fileset sources_1 $f
                if {$ext eq ".sv"} {
                    set_property file_type SystemVerilog [get_files $f]
                }
            }
        }
    }

    add_files -quiet -fileset constrs_1 $xdc
    if {$enable_soc} {
        add_files -quiet -fileset constrs_1 $soc_xdc
    }
    configure_cyan_constraints
    configure_cyan_impl_hooks
    if {$enable_soc} {
        set_property top kcu105_cyan_soc_top [current_fileset]
    } else {
        set_property top kcu105_cyan_top [current_fileset]
    }
    update_compile_order -fileset sources_1
    configure_soc_fileset

    set ips [get_ips -quiet kcu105_ddr4]
    if {[llength $ips] != 1} {
        fail "Cyan build project does not contain exactly one kcu105_ddr4 IP"
    }
    assert_ip_board_bindings [lindex $ips 0]
    validate_cyan_target_files
}

proc create_cyan_project {} {
    global build_dir seed_project_dir seed_project_name part_name board_part enable_debug

    file delete -force $seed_project_dir
    file mkdir [file dirname $seed_project_dir]

    create_project -force $seed_project_name $seed_project_dir -part $part_name
    set_property board_part $board_part [current_project]

    create_ip -name ddr4 -vendor xilinx.com -library ip -module_name kcu105_ddr4
    set ip [get_ips kcu105_ddr4]
    set ip_props [list \
        CONFIG.C0_DDR4_BOARD_INTERFACE {ddr4_sdram} \
        CONFIG.C0_CLOCK_BOARD_INTERFACE {default_sysclk_300} \
        CONFIG.RESET_BOARD_INTERFACE {reset} \
        CONFIG.C0.DDR4_MemoryPart {EDY4016AABG-DR-F} \
        CONFIG.C0.DDR4_DataWidth {64} \
        CONFIG.C0.DDR4_TimePeriod {833} \
        CONFIG.C0.DDR4_InputClockPeriod {3332} \
        CONFIG.ADDN_UI_CLKOUT1_FREQ_HZ {100} \
    ]
    if {$enable_debug} {
        lappend ip_props CONFIG.Debug_Signal {Enable}
    } else {
        lappend ip_props CONFIG.Debug_Signal {Disable}
    }
    set_property -dict $ip_props $ip
    assert_ip_board_bindings $ip

    generate_target all $ip
    export_ip_user_files -of_objects $ip -no_script -sync -force -quiet
    open_example_project -force -in_process -dir [file join $seed_project_dir generated] $ip
    puts "Generated KCU105 board-bound DDR4 design: [get_property DIRECTORY [current_project]]"

    copy_generated_target_from_current_project
    close_project
    create_cyan_build_project

    puts "KCU105 Cyan DDR4 target generated under $build_dir"
}

proc project_ip_matches_request {} {
    global enable_debug

    set ips [get_ips -quiet kcu105_ddr4]
    if {[llength $ips] != 1} {
        return 0
    }
    set ip [lindex $ips 0]
    foreach pair {
        {CONFIG.C0_DDR4_BOARD_INTERFACE ddr4_sdram}
        {CONFIG.C0_CLOCK_BOARD_INTERFACE default_sysclk_300}
        {CONFIG.RESET_BOARD_INTERFACE reset}
    } {
        if {[get_property [lindex $pair 0] $ip] ne [lindex $pair 1]} {
            return 0
        }
    }
    set want_debug [expr {$enable_debug ? "Enable" : "Disable"}]
    if {[get_property CONFIG.Debug_Signal $ip] ne $want_debug} {
        return 0
    }
    return 1
}

proc materialized_cyan_target_exists {} {
    global origin_dir enable_soc

    set required [list \
        [file join $origin_dir src kcu105_cyan_top.sv] \
        [file join $origin_dir src kcu105_cyan_traffic.sv] \
        [file join $origin_dir constraints kcu105_cyan.xdc] \
        [file join $origin_dir ip kcu105_ddr4 kcu105_ddr4.xci] \
    ]
    if {$enable_soc} {
        lappend required \
            [file join $origin_dir src kcu105_cyan_soc_top.sv] \
            [file join $origin_dir src kcu105_cyan_soc_vio_stub.v] \
            [file join $origin_dir constraints kcu105_cyan_soc.xdc] \
            [file join $origin_dir constraints kcu105_cyan_soc_post_link.tcl] \
            [file join $origin_dir cpu_island_cdc_constraints.tcl] \
            [file join $origin_dir lg1_serial_cdc_constraints.tcl] \
            [file join $origin_dir sgmii_rx_gearbox_constraints.tcl] \
            [file join $origin_dir signoff_sgmii_rx_gearbox_routed.tcl]
    }

    foreach file $required {
        if {![file exists $file]} {
            puts "KCU105 Cyan materialized target file is missing: $file"
            return 0
        }
    }
    return 1
}

proc open_or_create_project {} {
    global origin_dir cyan_project_xpr enable_soc

    if {![file exists $cyan_project_xpr] || ![materialized_cyan_target_exists]} {
        create_cyan_project
        return
    }
    if {[catch {open_project $cyan_project_xpr} err]} {
        puts "KCU105 Cyan project could not be opened; regenerating: $err"
        catch {close_project}
        create_cyan_project
        return
    }
    if {![project_ip_matches_request]} {
        puts "KCU105 Cyan project is stale or has wrong debug/profile bindings; regenerating"
        close_project
        create_cyan_project
        return
    }
    if {[soc_project_needs_recreate]} {
        puts "KCU105 Cyan SoC project has stale SGMII generated IP state; regenerating build project"
        close_project
        create_cyan_build_project
        return
    }
    assert_ip_board_bindings [lindex [get_ips kcu105_ddr4] 0]
    validate_cyan_target_files
    configure_cyan_constraints
    configure_cyan_impl_hooks
    if {$enable_soc} {
        configure_sgmii_ip
        configure_soc_debug_ip
        set soc_top_sv [file join $origin_dir src kcu105_cyan_soc_top.sv]
        set soc_xdc [file join $origin_dir constraints kcu105_cyan_soc.xdc]
        if {[llength [get_files -quiet $soc_top_sv]] == 0} {
            add_files -quiet -fileset sources_1 $soc_top_sv
        }
        if {[llength [get_files -quiet $soc_xdc]] == 0} {
            add_files -quiet -fileset constrs_1 $soc_xdc
        }
        configure_cyan_constraints
        configure_cyan_impl_hooks
        set_property file_type SystemVerilog [get_files $soc_top_sv]
        foreach f [concat [soc_rtl_files] [soc_mem_files]] {
            if {[llength [get_files -quiet $f]] == 0} {
                add_files -quiet -fileset sources_1 $f
            }
            set ext [string tolower [file extension $f]]
            if {$ext in {".v" ".sv"}} {
                set_property file_type SystemVerilog [get_files $f]
            }
            enable_build_file $f
        }
        set_property top kcu105_cyan_soc_top [current_fileset]
    } else {
        set_property top kcu105_cyan_top [current_fileset]
    }
    update_compile_order -fileset sources_1
    configure_soc_fileset
}

proc run_is_complete {run_name} {
    set run [get_runs -quiet $run_name]
    if {$run eq ""} {
        return 0
    }
    set status [get_property STATUS $run]
    set progress [get_property PROGRESS $run]
    return [expr {$progress eq "100%" && [string match "*Complete*" $status]}]
}

proc synth_checkpoint_exists {} {
    set run [get_runs -quiet synth_1]
    if {$run eq ""} {
        return 0
    }
    set checkpoint [file join \
        [get_property DIRECTORY $run] \
        "[get_property top [current_fileset]].dcp" \
    ]
    return [file exists $checkpoint]
}

proc run_needs_refresh {run_name} {
    set run [get_runs -quiet $run_name]
    if {$run eq ""} {
        return 1
    }
    set status [get_property STATUS $run]
    if {[string match "*ERROR*" $status] ||
        [string match "*Fail*" $status] ||
        [string match "*Cancel*" $status] ||
        [string match "*Abort*" $status] ||
        [string match "*Running*" $status]} {
        return 1
    }
    if {[lsearch -exact [list_property $run] NEEDS_REFRESH] < 0} {
        return 0
    }
    set needs_refresh [get_property NEEDS_REFRESH $run]
    return [expr {$needs_refresh eq "1" || $needs_refresh eq "true" || $needs_refresh eq "TRUE"}]
}

proc require_run_complete {run_name} {
    set run [get_runs $run_name]
    set status [get_property STATUS $run]
    set progress [get_property PROGRESS $run]
    puts "$run_name status: $status ($progress)"
    if {![run_is_complete $run_name]} {
        fail "$run_name did not complete successfully: $status ($progress)"
    }
}

proc build_check {} {
    generate_target all [get_ips]
    update_compile_order -fileset sources_1
    check_syntax -fileset sources_1
}

proc build_soc_debug_ip_runs {jobs} {
    global enable_soc enable_debug

    if {!$enable_soc || !$enable_debug} {
        return
    }
    set run [get_runs -quiet kcu105_cyan_soc_vio_synth_1]
    if {$run eq ""} {
        return
    }
    set needs_refresh [run_needs_refresh kcu105_cyan_soc_vio_synth_1]
    if {$needs_refresh} {
        reset_run $run
    }
    if {$needs_refresh || ![run_is_complete kcu105_cyan_soc_vio_synth_1]} {
        launch_runs $run -jobs $jobs
        wait_on_run $run
    }
    require_run_complete kcu105_cyan_soc_vio_synth_1
}

proc build_synth {jobs reset_first} {
    generate_target all [get_ips]
    build_soc_debug_ip_runs $jobs
    set needs_refresh [run_needs_refresh synth_1]
    set signature_stale [soc_synth_signature_stale]
    set complete_with_checkpoint [expr {
        [run_is_complete synth_1] && [synth_checkpoint_exists]
    }]
    if {$signature_stale} {
        puts "KCU105 Cyan SoC synth signature changed; resetting synth_1"
        set needs_refresh 1
    } elseif {!$reset_first && $complete_with_checkpoint} {
        if {$needs_refresh} {
            puts "Reusing completed synth_1: content signature and checkpoint are current"
        }
        require_run_complete synth_1
        return
    }
    if {$reset_first || $needs_refresh} {
        reset_run synth_1
    }
    if {$reset_first || $needs_refresh || ![run_is_complete synth_1]} {
        launch_runs synth_1 -jobs $jobs
        wait_on_run synth_1
    }
    require_run_complete synth_1
    write_soc_synth_signature
}

proc prepare_impl_incremental_checkpoint {} {
    global cyan_project_name enable_soc origin_dir

    if {$enable_soc} {
        set top_name "kcu105_cyan_soc_top"
    } else {
        set top_name "kcu105_cyan_top"
    }

    set impl_dir [file join [get_property DIRECTORY [current_project]] ${cyan_project_name}.runs impl_1]
    set routed_dcp [file join $impl_dir ${top_name}_routed.dcp]
    if {![file exists $routed_dcp]} {
        return ""
    }

    set seed_dir [file join $origin_dir build incremental]
    file mkdir $seed_dir
    set seed_dcp [file join $seed_dir ${cyan_project_name}_incremental_seed.dcp]
    file copy -force $routed_dcp $seed_dcp
    return $seed_dcp
}

proc impl_routed_checkpoint {} {
    set run [get_runs -quiet impl_1]
    if {$run eq ""} {
        return ""
    }
    return [file join \
        [get_property DIRECTORY $run] \
        "[get_property top [current_fileset]]_routed.dcp" \
    ]
}

proc impl_incremental_disabled {} {
    if {![info exists ::env(CYAN_DISABLE_INCREMENTAL_IMPL)]} {
        return 0
    }
    set value [string tolower $::env(CYAN_DISABLE_INCREMENTAL_IMPL)]
    return [expr {$value ne "" && $value ne "0" && $value ne "false" && $value ne "no"}]
}

proc clear_impl_incremental_checkpoint {} {
    set run [get_runs -quiet impl_1]
    if {$run eq ""} {
        return
    }
    if {[catch {reset_property incremental_checkpoint $run} err]} {
        catch {set_property incremental_checkpoint "" $run}
    }
}

proc set_impl_incremental_checkpoint {seed_dcp} {
    if {$seed_dcp eq "" || ![file exists $seed_dcp]} {
        return
    }

    set run [get_runs -quiet impl_1]
    if {$run eq ""} {
        return
    }

    if {[catch {set_property incremental_checkpoint $seed_dcp $run} err]} {
        puts "Incremental implementation seed not applied: $err"
    } else {
        puts "Using incremental implementation seed: $seed_dcp"
    }
}

proc write_impl_bitstream {} {
    set run [get_runs impl_1]
    set top [get_property top [current_fileset]]
    set run_dir [get_property DIRECTORY $run]
    set routed_dcp [impl_routed_checkpoint]
    if {![file exists $routed_dcp]} {
        fail "Routed checkpoint not found: $routed_dcp"
    }

    open_checkpoint $routed_dcp
    write_debug_probes -force [file join $run_dir ${top}.ltx]
    write_bitstream -force [file join $run_dir ${top}.bit]
    close_design
}

proc build_impl {jobs to_bit} {
    set incremental_disabled [impl_incremental_disabled]
    if {$incremental_disabled} {
        set incremental_seed ""
        clear_impl_incremental_checkpoint
        puts "KCU105 Cyan incremental implementation disabled by CYAN_DISABLE_INCREMENTAL_IMPL"
    } else {
        set incremental_seed [prepare_impl_incremental_checkpoint]
    }
    build_synth $jobs 0
    set routed_dcp [impl_routed_checkpoint]
    if {![soc_impl_signature_stale] &&
        ![run_needs_refresh impl_1] &&
        $routed_dcp ne "" && [file exists $routed_dcp]} {
        puts "Reusing routed impl_1: content signature and checkpoint are current"
        if {$to_bit} {
            write_impl_bitstream
        }
        return
    }
    set needs_refresh [run_needs_refresh impl_1]
    if {$needs_refresh} {
        if {!$incremental_disabled && $incremental_seed eq ""} {
            set incremental_seed [prepare_impl_incremental_checkpoint]
        }
        reset_run impl_1
        if {$incremental_disabled} {
            clear_impl_incremental_checkpoint
        }
    }
    if {$needs_refresh || ![run_is_complete impl_1]} {
        if {$incremental_disabled} {
            clear_impl_incremental_checkpoint
        } else {
            set_impl_incremental_checkpoint $incremental_seed
        }
        launch_runs impl_1 -jobs $jobs
        wait_on_run impl_1
    }
    require_run_complete impl_1
    write_soc_impl_signature
    if {$to_bit} {
        write_impl_bitstream
    }
}

file mkdir $build_dir

if {$step eq "generate"} {
    create_cyan_project
    exit 0
}

open_or_create_project

if {$step eq "check"} {
    build_check
    exit 0
}

if {$step eq "synth"} {
    build_synth $jobs 0
    exit 0
}

if {$step eq "impl"} {
    build_impl $jobs 0
    exit 0
}

build_impl $jobs 1
set impl_dir [file join [get_property DIRECTORY [current_project]] ${cyan_project_name}.runs impl_1]
if {$enable_soc} {
    set top_bit_name "kcu105_cyan_soc_top"
    set output_bit_name "kcu105_cyan_soc"
} else {
    set top_bit_name "kcu105_cyan_top"
    set output_bit_name "kcu105_cyan"
}
set bit_path [file join $impl_dir ${top_bit_name}.bit]
set ltx_path [file join $impl_dir ${top_bit_name}.ltx]
if {![file exists $bit_path]} {
    fail "Expected bitstream not found: $bit_path"
}
set programming_dir [file join $origin_dir programming]
file mkdir $programming_dir
file copy -force $bit_path [file join $programming_dir ${output_bit_name}.bit]
if {[file exists $ltx_path]} {
    file copy -force $ltx_path [file join $programming_dir ${output_bit_name}.ltx]
} else {
    file delete -force [file join $programming_dir ${output_bit_name}.ltx]
}
puts "KCU105 Cyan bitstream: [file join $programming_dir ${output_bit_name}.bit]"
