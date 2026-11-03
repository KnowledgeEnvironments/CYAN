


# KCU105 configuration bank is wired for 1.8 V configuration.
set_property CFGBVS GND [current_design]
set_property CONFIG_VOLTAGE 1.8 [current_design]

# Preserve the DDR4 IP recommendation after board-pin constraints are applied.
set_property DRIVE 8 [get_ports c0_ddr4_reset_n]

# KCU105 GPIO LEDs are active-high.
set_property PACKAGE_PIN AP8 [get_ports {gpio_led[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {gpio_led[0]}]
set_property DRIVE 8 [get_ports {gpio_led[0]}]
set_property PACKAGE_PIN H23 [get_ports {gpio_led[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {gpio_led[1]}]
set_property DRIVE 8 [get_ports {gpio_led[1]}]
set_property PACKAGE_PIN P20 [get_ports {gpio_led[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {gpio_led[2]}]
set_property DRIVE 8 [get_ports {gpio_led[2]}]
set_property PACKAGE_PIN P21 [get_ports {gpio_led[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {gpio_led[3]}]
set_property DRIVE 8 [get_ports {gpio_led[3]}]
set_property PACKAGE_PIN N22 [get_ports {gpio_led[4]}]
set_property IOSTANDARD LVCMOS18 [get_ports {gpio_led[4]}]
set_property DRIVE 8 [get_ports {gpio_led[4]}]
set_property PACKAGE_PIN M22 [get_ports {gpio_led[5]}]
set_property IOSTANDARD LVCMOS18 [get_ports {gpio_led[5]}]
set_property DRIVE 8 [get_ports {gpio_led[5]}]
set_property PACKAGE_PIN R23 [get_ports {gpio_led[6]}]
set_property IOSTANDARD LVCMOS18 [get_ports {gpio_led[6]}]
set_property DRIVE 8 [get_ports {gpio_led[6]}]
set_property PACKAGE_PIN P23 [get_ports {gpio_led[7]}]
set_property IOSTANDARD LVCMOS18 [get_ports {gpio_led[7]}]
set_property DRIVE 8 [get_ports {gpio_led[7]}]
