VERILATOR ?= verilator
VIVADO ?= vivado
VIVADO_JOBS ?= 4
BUILD_DIR ?= $(CURDIR)/build
FIRMWARE_DIR ?= $(BUILD_DIR)/firmware

.PHONY: help test cpu-test dsp-test firmware kcu-generate kcu-check kcu-synth \
	kcu-impl kcu-bit

help:
	@printf '%s\n' \
	  'CYAN targets:' \
	  '  make test                         public source gate (CPU, DSP, tools)' \
	  '  make firmware FIRMWARE_BIN=...   prepare a user-supplied 512 KiB ROM' \
	  '  make kcu-generate                 create the KCU105 Vivado project' \
	  '  make kcu-check                    validate the generated project' \
	  '  make kcu-synth                    synthesize the KCU105 SoC' \
	  '  make kcu-impl                     implement the KCU105 SoC' \
	  '  make kcu-bit                      emit the KCU105 bitstream' \
	  '' \
	  'Set FIRMWARE_DIR to a prepared firmware directory and EEPROM_INIT_FILE' \
	  'to an external 128-word 93CS56 image for every Vivado SoC target.'

test: cpu-test dsp-test
	python3 -m py_compile tools/bin_to_memh32.py tools/cyanctl.py
	python3 -m unittest discover -s tools/tests -p 'test_*.py'
	git diff --check

cpu-test:
	$(MAKE) -C rtl/cpu test VERILATOR=$(VERILATOR)

dsp-test:
	$(MAKE) -C rtl/dsp/tests test VERILATOR=$(VERILATOR)

firmware:
	@test -n "$(FIRMWARE_BIN)" || { echo 'FIRMWARE_BIN is required'; exit 2; }
	@test "$$(wc -c < "$(FIRMWARE_BIN)")" -eq 524288 || \
		{ echo 'firmware image must be exactly 524288 bytes'; exit 2; }
	mkdir -p "$(FIRMWARE_DIR)"
	python3 tools/bin_to_memh32.py "$(FIRMWARE_BIN)" \
		"$(FIRMWARE_DIR)/cyan_firmware.memh" --depth 131072
	python3 tools/bin_to_memh32.py "$(FIRMWARE_BIN)" \
		"$(FIRMWARE_DIR)/cyan_firmware_b{bank_hex}.memh" \
		--depth 131072 --bank-count 16

define run_vivado
	@test -f "$(FIRMWARE_DIR)/cyan_firmware.memh" || \
		{ echo 'prepare firmware first with make firmware FIRMWARE_BIN=...'; exit 2; }
	@test -n "$(EEPROM_INIT_FILE)" || \
		{ echo 'EEPROM_INIT_FILE is required for a bootable KCU105 SoC build'; exit 2; }
	@test -f "$(EEPROM_INIT_FILE)" || \
		{ echo 'EEPROM_INIT_FILE does not name a readable file'; exit 2; }
	CYAN_PROM_DIR="$(abspath $(FIRMWARE_DIR))" \
	CYAN_EEPROM_INIT_FILE="$(abspath $(EEPROM_INIT_FILE))" \
		$(VIVADO) -mode batch -source fpga/kcu105_cyan/generate.tcl \
		-tclargs soc-$(1) $(VIVADO_JOBS)
endef

kcu-generate:
	$(call run_vivado,generate)

kcu-check:
	$(call run_vivado,check)

kcu-synth:
	$(call run_vivado,synth)

kcu-impl:
	$(call run_vivado,impl)

kcu-bit:
	$(call run_vivado,bit)
