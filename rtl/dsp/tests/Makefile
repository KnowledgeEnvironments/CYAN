# SPDX-License-Identifier: MIT
# Copyright (c) 2026 I. Finder <src@lispm.net>

VERILATOR ?= verilator
DSP_RTL := $(abspath ..)
BOARD_RTL := $(abspath ../../board)
BUILD_ROOT ?= $(abspath ../../../tmp/dsp-tests)
SCI_BUILD_DIR := $(BUILD_ROOT)/sci-data
SCI_BIN := $(SCI_BUILD_DIR)/Vdsp56001_periph
ALU_FAMILY_BUILD_DIR := $(BUILD_ROOT)/alu-family
ALU_FAMILY_BIN := $(ALU_FAMILY_BUILD_DIR)/Vdsp56001_core
ALU_DIFFERENTIAL_BUILD_DIR := $(BUILD_ROOT)/alu-differential
ALU_DIFFERENTIAL_BIN := $(ALU_DIFFERENTIAL_BUILD_DIR)/Vdsp56001_alu
AGU_DIFFERENTIAL_BUILD_DIR := $(BUILD_ROOT)/agu-differential
AGU_DIFFERENTIAL_BIN := $(AGU_DIFFERENTIAL_BUILD_DIR)/Vdsp56001_agu
DMA_BUILD_DIR := $(BUILD_ROOT)/dma
DMA_BIN := $(DMA_BUILD_DIR)/Vdsp56001_dma
HOST_DMA_COLLISION_BUILD_DIR := $(BUILD_ROOT)/host-dma-collision
HOST_DMA_COLLISION_BIN := $(HOST_DMA_COLLISION_BUILD_DIR)/Vdsp56001_host
HOST_TXRX_COLLISION_BUILD_DIR := $(BUILD_ROOT)/host-txrx-collision
HOST_TXRX_COLLISION_BIN := $(HOST_TXRX_COLLISION_BUILD_DIR)/Vdsp56001_host
HOST_DSP_DMA_COLLISION_BUILD_DIR := $(BUILD_ROOT)/host-dsp-dma-collision
HOST_DSP_DMA_COLLISION_BIN := \
	$(HOST_DSP_DMA_COLLISION_BUILD_DIR)/Vdsp56001_host
HOST_COUNTER_BUILD_DIR := $(BUILD_ROOT)/host-counter
HOST_COUNTER_BIN := $(HOST_COUNTER_BUILD_DIR)/Vdsp56001_host
HOST_HANDSHAKE_BUILD_DIR := $(BUILD_ROOT)/host-handshake
HOST_HANDSHAKE_BIN := $(HOST_HANDSHAKE_BUILD_DIR)/Vdsp56001_host
MULTIPLY_SELECTOR_BUILD_DIR := $(BUILD_ROOT)/multiply-selector
MULTIPLY_SELECTOR_BIN := $(MULTIPLY_SELECTOR_BUILD_DIR)/Vdsp56001_core
FIXED_CONTROL_BUILD_DIR := $(BUILD_ROOT)/fixed-control-timing
FIXED_CONTROL_BIN := $(FIXED_CONTROL_BUILD_DIR)/Vdsp56001_core
MOVE_SELECTOR_BUILD_DIR := $(BUILD_ROOT)/move-selector
MOVE_SELECTOR_BIN := $(MOVE_SELECTOR_BUILD_DIR)/Vdsp56001_core
WAIT_BUILD_DIR := $(BUILD_ROOT)/wait
WAIT_BIN := $(WAIT_BUILD_DIR)/Vdsp56001_core
SWI_BUILD_DIR := $(BUILD_ROOT)/swi
SWI_BIN := $(SWI_BUILD_DIR)/Vdsp56001_core
JUMP_TIMING_BUILD_DIR := $(BUILD_ROOT)/jump-timing
JUMP_TIMING_BIN := $(JUMP_TIMING_BUILD_DIR)/Vdsp56001_core
CONDITIONAL_JUMP_BUILD_DIR := $(BUILD_ROOT)/conditional-jump
CONDITIONAL_JUMP_BIN := $(CONDITIONAL_JUMP_BUILD_DIR)/Vdsp56001_core
MOVEC_BUILD_DIR := $(BUILD_ROOT)/movec
MOVEC_BIN := $(MOVEC_BUILD_DIR)/Vdsp56001_core
IRQ_PRIO_BUILD_DIR := $(BUILD_ROOT)/irq-priority
IRQ_PRIO_BIN := $(IRQ_PRIO_BUILD_DIR)/Vdsp56001_core
IRQ_EDGE_BUILD_DIR := $(BUILD_ROOT)/irq-edge
IRQ_EDGE_BIN := $(IRQ_EDGE_BUILD_DIR)/Vdsp56001_core
IRQ_NEST_BUILD_DIR := $(BUILD_ROOT)/irq-nesting
IRQ_NEST_BIN := $(IRQ_NEST_BUILD_DIR)/Vdsp56001_core
IRQ_COMBO_BUILD_DIR := $(BUILD_ROOT)/irq-combinations
IRQ_COMBO_BIN := $(IRQ_COMBO_BUILD_DIR)/Vdsp56001_core
SSI_IRQ_BUILD_DIR := $(BUILD_ROOT)/ssi-irq
SSI_IRQ_BIN := $(SSI_IRQ_BUILD_DIR)/Vdsp56001_core
SSI_DATA_BUILD_DIR := $(BUILD_ROOT)/ssi-data
SSI_DATA_BIN := $(SSI_DATA_BUILD_DIR)/Vdsp56001_periph
SCI_VECTOR_BUILD_DIR := $(BUILD_ROOT)/sci-vector
SCI_VECTOR_BIN := $(SCI_VECTOR_BUILD_DIR)/Vdsp56001_core
MEM_IRQ_BUILD_DIR := $(BUILD_ROOT)/memory-irq
MEM_IRQ_BIN := $(MEM_IRQ_BUILD_DIR)/Vdsp56001_core
DO_DECODE_BUILD_DIR := $(BUILD_ROOT)/do-decode
DO_DECODE_BIN := $(DO_DECODE_BUILD_DIR)/Vdsp56001_decode
REP_DECODE_BUILD_DIR := $(BUILD_ROOT)/rep-decode
REP_DECODE_BIN := $(REP_DECODE_BUILD_DIR)/Vdsp56001_decode
LOOP_TIMING_BUILD_DIR := $(BUILD_ROOT)/loop-timing
LOOP_TIMING_BIN := $(LOOP_TIMING_BUILD_DIR)/Vdsp56001_core
ANDI_ORI_BUILD_DIR := $(BUILD_ROOT)/andi-ori
ANDI_ORI_BIN := $(ANDI_ORI_BUILD_DIR)/Vdsp56001_core
DIV_BUILD_DIR := $(BUILD_ROOT)/div
DIV_BIN := $(DIV_BUILD_DIR)/Vdsp56001_core
NORM_BUILD_DIR := $(BUILD_ROOT)/norm
NORM_BIN := $(NORM_BUILD_DIR)/Vdsp56001_core
TFR_BUILD_DIR := $(BUILD_ROOT)/tfr
TFR_BIN := $(TFR_BUILD_DIR)/Vdsp56001_core
RND_TIMING_BUILD_DIR := $(BUILD_ROOT)/rnd-timing
RND_TIMING_BIN := $(RND_TIMING_BUILD_DIR)/Vdsp56001_core
SYSTEM_REGS_BUILD_DIR := $(BUILD_ROOT)/system-registers
SYSTEM_REGS_BIN := $(SYSTEM_REGS_BUILD_DIR)/Vdsp56001_periph
SR_RESERVED_BUILD_DIR := $(BUILD_ROOT)/sr-reserved
SR_RESERVED_BIN := $(SR_RESERVED_BUILD_DIR)/Vdsp56001_core
ILLEGAL_BUILD_DIR := $(BUILD_ROOT)/illegal
ILLEGAL_BIN := $(ILLEGAL_BUILD_DIR)/Vdsp56001_core
BIT_TIMING_BUILD_DIR := $(BUILD_ROOT)/bit-timing
BIT_TIMING_BIN := $(BIT_TIMING_BUILD_DIR)/Vdsp56001_core
PARALLEL_MOVE_TIMING_BUILD_DIR := $(BUILD_ROOT)/parallel-move-timing
PARALLEL_MOVE_TIMING_BIN := $(PARALLEL_MOVE_TIMING_BUILD_DIR)/Vdsp56001_core
XY_PARALLEL_MOVE_BUILD_DIR := $(BUILD_ROOT)/xy-parallel-move
XY_PARALLEL_MOVE_BIN := $(XY_PARALLEL_MOVE_BUILD_DIR)/Vdsp56001_core
CLASS1_SELECTOR_BUILD_DIR := $(BUILD_ROOT)/class1-selector
CLASS1_SELECTOR_BIN := $(CLASS1_SELECTOR_BUILD_DIR)/Vdsp56001_core
MOVEC_MEMORY_BUILD_DIR := $(BUILD_ROOT)/movec-memory
MOVEC_MEMORY_BIN := $(MOVEC_MEMORY_BUILD_DIR)/Vdsp56001_core
MOVEC_REGISTER_BUILD_DIR := $(BUILD_ROOT)/movec-register
MOVEC_REGISTER_BIN := $(MOVEC_REGISTER_BUILD_DIR)/Vdsp56001_core
MOVEC_TIMING_BUILD_DIR := $(BUILD_ROOT)/movec-timing
MOVEC_TIMING_BIN := $(MOVEC_TIMING_BUILD_DIR)/Vdsp56001_core
MOVEM_TIMING_BUILD_DIR := $(BUILD_ROOT)/movem-timing
MOVEM_TIMING_BIN := $(MOVEM_TIMING_BUILD_DIR)/Vdsp56001_core
MOVEP_BUILD_DIR := $(BUILD_ROOT)/movep
MOVEP_BIN := $(MOVEP_BUILD_DIR)/Vdsp56001_core
MOVEP_TIMING_BUILD_DIR := $(BUILD_ROOT)/movep-timing
MOVEP_TIMING_BIN := $(MOVEP_TIMING_BUILD_DIR)/Vdsp56001_core
P_DATA_WAIT_BUILD_DIR := $(BUILD_ROOT)/p-data-wait
P_DATA_WAIT_BIN := $(P_DATA_WAIT_BUILD_DIR)/Vdsp56001_core
P_FETCH_WAIT_BUILD_DIR := $(BUILD_ROOT)/p-fetch-wait
P_FETCH_WAIT_BIN := $(P_FETCH_WAIT_BUILD_DIR)/Vdsp56001_core
ADDRESS_REGISTER_LATENCY_BUILD_DIR := $(BUILD_ROOT)/address-register-latency
ADDRESS_REGISTER_LATENCY_BIN := $(ADDRESS_REGISTER_LATENCY_BUILD_DIR)/Vdsp56001_core
EXTERNAL_WAIT_STATE_BUILD_DIR := $(BUILD_ROOT)/external-wait-state
EXTERNAL_WAIT_STATE_BIN := $(EXTERNAL_WAIT_STATE_BUILD_DIR)/Vdsp56001_core
LATE_WRITEBACK_WAIT_BUILD_DIR := $(BUILD_ROOT)/late-writeback-wait
LATE_WRITEBACK_WAIT_BIN := $(LATE_WRITEBACK_WAIT_BUILD_DIR)/Vdsp56001_core
RY_CLASS1_BUILD_DIR := $(BUILD_ROOT)/ry-class1
RY_CLASS1_BIN := $(RY_CLASS1_BUILD_DIR)/Vdsp56001_core
TRACE_BUILD_DIR := $(BUILD_ROOT)/trace
TRACE_BIN := $(TRACE_BUILD_DIR)/Vdsp56001_core
XR_CLASS1_BUILD_DIR := $(BUILD_ROOT)/xr-class1
XR_CLASS1_BIN := $(XR_CLASS1_BUILD_DIR)/Vdsp56001_core
XR_RY_CLASS2_BUILD_DIR := $(BUILD_ROOT)/xr-ry-class2
XR_RY_CLASS2_BIN := $(XR_RY_CLASS2_BUILD_DIR)/Vdsp56001_core
AGU_BOUNDARY_BUILD_DIR := $(BUILD_ROOT)/agu-boundary
AGU_BOUNDARY_BIN := $(AGU_BOUNDARY_BUILD_DIR)/Vdsp56001_agu
LOGIC_CCR_BUILD_DIR := $(BUILD_ROOT)/logic-ccr
LOGIC_CCR_BIN := $(LOGIC_CCR_BUILD_DIR)/Vdsp56001_alu
PORT_C_BUILD_DIR := $(BUILD_ROOT)/port-c
PORT_C_BIN := $(PORT_C_BUILD_DIR)/Vdsp56001_periph
ROM_TABLES_BUILD_DIR := $(BUILD_ROOT)/rom-tables
ROM_TABLES_BIN := $(ROM_TABLES_BUILD_DIR)/Vdsp56001_mem
STACK_ERROR_BUILD_DIR := $(BUILD_ROOT)/stack-error
STACK_ERROR_BIN := $(STACK_ERROR_BUILD_DIR)/Vdsp56001_core
STOP_DIRECT_BUILD_DIR := $(BUILD_ROOT)/stop-direct
STOP_DIRECT_BIN := $(STOP_DIRECT_BUILD_DIR)/Vdsp56001_core
RESET_DIRECT_BUILD_DIR := $(BUILD_ROOT)/reset-direct
RESET_DIRECT_BIN := $(RESET_DIRECT_BUILD_DIR)/Vdsp56001_core
BIT_REGISTERS_BUILD_DIR := $(BUILD_ROOT)/bit-registers
BIT_REGISTERS_BIN := $(BIT_REGISTERS_BUILD_DIR)/Vdsp56001_core
AUDIO_GLUE_BUILD_DIR := $(BUILD_ROOT)/audio-glue
AUDIO_GLUE_BIN := $(AUDIO_GLUE_BUILD_DIR)/Vindigo_hp1_audio_glue
SPDIF_BUILD_DIR := $(BUILD_ROOT)/spdif
SPDIF_BIN := $(SPDIF_BUILD_DIR)/Vcyan_spdif_tx
LUA_BUILD_DIR := $(BUILD_ROOT)/lua
LUA_BIN := $(LUA_BUILD_DIR)/Vdsp56001_core
TCC_BUILD_DIR := $(BUILD_ROOT)/tcc-direct
TCC_BIN := $(TCC_BUILD_DIR)/Vdsp56001_core
TOP_DMA_BUILD_DIR := $(BUILD_ROOT)/top-dma-contention
TOP_DMA_BIN := $(TOP_DMA_BUILD_DIR)/Vdsp56001_top

VERILATOR_FLAGS := --cc --exe --build -Wno-fatal \
	-Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-PINCONNECTEMPTY \
	-Wno-BLKSEQ -Wno-WIDTHTRUNC -Wno-WIDTHEXPAND \
	-Wno-CASEINCOMPLETE -CFLAGS "-O2 -std=c++17" -LDFLAGS "-lm"

.PHONY: test test-sci-data-boundary test-alu-family \
	test-alu-differential test-agu-differential test-dma-normal \
	test-host-dma-w0c-collision test-host-txrx-w0c-collision \
	test-host-dsp-dma-w0c-collision test-host-counter \
	test-host-handshake test-multiply-selector \
	test-fixed-control-timing test-move-selector-matrix test-wait \
	test-swi test-jump-timing test-conditional-jump test-movec \
	test-irq-priority \
	test-irq-edge test-irq-nesting test-irq-combinations test-ssi-irq \
	test-ssi-data \
	test-sci-vector test-memory-irq test-do-decode test-rep-decode \
	test-loop-timing test-andi-ori test-div test-norm test-tfr \
	test-rnd-timing test-system-registers test-sr-reserved test-illegal \
	test-bit-timing test-parallel-move-timing test-xy-parallel-move \
	test-class1-selector test-movec-memory test-movec-register \
	test-movec-timing test-movem-timing test-movep test-movep-timing \
	test-p-data-wait test-p-fetch-wait test-address-register-latency \
	test-external-wait-state test-late-writeback-wait test-ry-class1 \
	test-trace test-xr-class1 test-xr-ry-class2 test-agu-boundary \
	test-logic-ccr test-port-c test-rom-tables test-stack-error \
	test-stop-direct test-reset-direct test-bit-registers test-audio-glue \
	test-spdif test-lua test-tcc-direct test-top-dma-contention

test: test-sci-data-boundary test-alu-family test-alu-differential \
	test-agu-differential test-dma-normal test-host-dma-w0c-collision \
	test-host-txrx-w0c-collision test-host-dsp-dma-w0c-collision \
	test-host-counter test-multiply-selector test-fixed-control-timing \
	test-host-handshake test-move-selector-matrix \
	test-wait test-swi test-jump-timing test-conditional-jump test-movec \
	test-irq-priority \
	test-irq-edge test-irq-nesting test-irq-combinations test-ssi-irq \
	test-ssi-data \
	test-sci-vector test-memory-irq test-do-decode test-rep-decode \
	test-loop-timing test-andi-ori test-div test-norm test-tfr \
	test-rnd-timing test-system-registers test-sr-reserved test-illegal \
	test-bit-timing test-parallel-move-timing test-xy-parallel-move \
	test-class1-selector test-movec-memory test-movec-register \
	test-movec-timing test-movem-timing test-movep test-movep-timing \
	test-p-data-wait test-p-fetch-wait test-address-register-latency \
	test-external-wait-state test-late-writeback-wait test-ry-class1 \
	test-trace test-xr-class1 test-xr-ry-class2 test-agu-boundary \
	test-logic-ccr test-port-c test-rom-tables test-stack-error \
	test-stop-direct test-reset-direct test-bit-registers test-audio-glue \
	test-spdif test-lua test-tcc-direct test-top-dma-contention

$(SCI_BIN): $(DSP_RTL)/dsp56001_periph.v dsp_sci_data_oracle.cpp
	@mkdir -p $(SCI_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_periph \
		--Mdir $(SCI_BUILD_DIR) \
		-o Vdsp56001_periph \
		$(DSP_RTL)/dsp56001_periph.v \
		$(abspath dsp_sci_data_oracle.cpp)

test-sci-data-boundary: $(SCI_BIN)
	$(SCI_BIN)

$(ALU_FAMILY_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_alu_family_oracle.cpp
	@mkdir -p $(ALU_FAMILY_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(ALU_FAMILY_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_alu_family_oracle.cpp)

test-alu-family: $(ALU_FAMILY_BIN)
	$(ALU_FAMILY_BIN)

$(ALU_DIFFERENTIAL_BIN): $(DSP_RTL)/dsp56001_alu.v \
		dsp_alu_differential_oracle.cpp
	@mkdir -p $(ALU_DIFFERENTIAL_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_alu \
		--Mdir $(ALU_DIFFERENTIAL_BUILD_DIR) \
		-o Vdsp56001_alu \
		$(DSP_RTL)/dsp56001_alu.v \
		$(abspath dsp_alu_differential_oracle.cpp)

test-alu-differential: $(ALU_DIFFERENTIAL_BIN)
	$(ALU_DIFFERENTIAL_BIN)

$(AGU_DIFFERENTIAL_BIN): $(DSP_RTL)/dsp56001_agu.v \
		dsp_agu_differential_oracle.cpp
	@mkdir -p $(AGU_DIFFERENTIAL_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_agu \
		--Mdir $(AGU_DIFFERENTIAL_BUILD_DIR) \
		-o Vdsp56001_agu \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_agu_differential_oracle.cpp)

test-agu-differential: $(AGU_DIFFERENTIAL_BIN)
	$(AGU_DIFFERENTIAL_BIN)

$(DMA_BIN): $(DSP_RTL)/dsp56001_dma.v dsp_dma_tb.cpp
	@mkdir -p $(DMA_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_dma \
		--Mdir $(DMA_BUILD_DIR) \
		-o Vdsp56001_dma \
		$(DSP_RTL)/dsp56001_dma.v \
		$(abspath dsp_dma_tb.cpp)

test-dma-normal: $(DMA_BIN)
	$(DMA_BIN)

$(HOST_DMA_COLLISION_BIN): $(DSP_RTL)/dsp56001_host.v \
		dsp_host_collision_tb.cpp
	@mkdir -p $(HOST_DMA_COLLISION_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_host \
		--Mdir $(HOST_DMA_COLLISION_BUILD_DIR) \
		-o Vdsp56001_host \
		$(DSP_RTL)/dsp56001_host.v \
		$(abspath dsp_host_collision_tb.cpp)

test-host-dma-w0c-collision: $(HOST_DMA_COLLISION_BIN)
	$(HOST_DMA_COLLISION_BIN)

$(HOST_TXRX_COLLISION_BIN): $(DSP_RTL)/dsp56001_host.v \
		dsp_host_txrx_collision_tb.cpp
	@mkdir -p $(HOST_TXRX_COLLISION_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_host \
		--Mdir $(HOST_TXRX_COLLISION_BUILD_DIR) \
		-o Vdsp56001_host \
		$(DSP_RTL)/dsp56001_host.v \
		$(abspath dsp_host_txrx_collision_tb.cpp)

test-host-txrx-w0c-collision: $(HOST_TXRX_COLLISION_BIN)
	$(HOST_TXRX_COLLISION_BIN)

$(HOST_DSP_DMA_COLLISION_BIN): $(DSP_RTL)/dsp56001_host.v \
		dsp_host_dsp_dma_collision_tb.cpp
	@mkdir -p $(HOST_DSP_DMA_COLLISION_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_host \
		--Mdir $(HOST_DSP_DMA_COLLISION_BUILD_DIR) \
		-o Vdsp56001_host \
		$(DSP_RTL)/dsp56001_host.v \
		$(abspath dsp_host_dsp_dma_collision_tb.cpp)

test-host-dsp-dma-w0c-collision: $(HOST_DSP_DMA_COLLISION_BIN)
	$(HOST_DSP_DMA_COLLISION_BIN)

$(HOST_COUNTER_BIN): $(DSP_RTL)/dsp56001_host.v \
		dsp_host_counter_oracle.cpp
	@mkdir -p $(HOST_COUNTER_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_host \
		--Mdir $(HOST_COUNTER_BUILD_DIR) \
		-o Vdsp56001_host \
		$(DSP_RTL)/dsp56001_host.v \
		$(abspath dsp_host_counter_oracle.cpp)

test-host-counter: $(HOST_COUNTER_BIN)
	$(HOST_COUNTER_BIN)

$(HOST_HANDSHAKE_BIN): $(DSP_RTL)/dsp56001_host.v \
		dsp_host_handshake_oracle.cpp
	@mkdir -p $(HOST_HANDSHAKE_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_host \
		--Mdir $(HOST_HANDSHAKE_BUILD_DIR) \
		-o Vdsp56001_host \
		$(DSP_RTL)/dsp56001_host.v \
		$(abspath dsp_host_handshake_oracle.cpp)

test-host-handshake: $(HOST_HANDSHAKE_BIN)
	$(HOST_HANDSHAKE_BIN)

$(MULTIPLY_SELECTOR_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_multiply_selector_oracle.cpp
	@mkdir -p $(MULTIPLY_SELECTOR_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(MULTIPLY_SELECTOR_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_multiply_selector_oracle.cpp)

test-multiply-selector: $(MULTIPLY_SELECTOR_BIN)
	$(MULTIPLY_SELECTOR_BIN)

$(FIXED_CONTROL_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_fixed_control_timing_oracle.cpp
	@mkdir -p $(FIXED_CONTROL_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(FIXED_CONTROL_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_fixed_control_timing_oracle.cpp)

test-fixed-control-timing: $(FIXED_CONTROL_BIN)
	$(FIXED_CONTROL_BIN)

$(MOVE_SELECTOR_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_move_selector_oracle.cpp
	@mkdir -p $(MOVE_SELECTOR_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(MOVE_SELECTOR_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_move_selector_oracle.cpp)

test-move-selector-matrix: $(MOVE_SELECTOR_BIN)
	$(MOVE_SELECTOR_BIN)

$(WAIT_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_wait_oracle.cpp
	@mkdir -p $(WAIT_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(WAIT_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_wait_oracle.cpp)

test-wait: $(WAIT_BIN)
	$(WAIT_BIN)

$(SWI_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_swi_oracle.cpp
	@mkdir -p $(SWI_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(SWI_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_swi_oracle.cpp)

test-swi: $(SWI_BIN)
	$(SWI_BIN)

$(JUMP_TIMING_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_jump_timing_oracle.cpp
	@mkdir -p $(JUMP_TIMING_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(JUMP_TIMING_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_jump_timing_oracle.cpp)

test-jump-timing: $(JUMP_TIMING_BIN)
	$(JUMP_TIMING_BIN)

$(CONDITIONAL_JUMP_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_conditional_jump_oracle.cpp
	@mkdir -p $(CONDITIONAL_JUMP_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(CONDITIONAL_JUMP_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_conditional_jump_oracle.cpp)

test-conditional-jump: $(CONDITIONAL_JUMP_BIN)
	$(CONDITIONAL_JUMP_BIN)

$(MOVEC_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_movec_oracle.cpp
	@mkdir -p $(MOVEC_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(MOVEC_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_movec_oracle.cpp)

test-movec: $(MOVEC_BIN)
	$(MOVEC_BIN)

$(IRQ_PRIO_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_irq_prio_oracle.cpp
	@mkdir -p $(IRQ_PRIO_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(IRQ_PRIO_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_irq_prio_oracle.cpp)

test-irq-priority: $(IRQ_PRIO_BIN)
	$(IRQ_PRIO_BIN)

$(IRQ_EDGE_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_irq_edge_oracle.cpp
	@mkdir -p $(IRQ_EDGE_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(IRQ_EDGE_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_irq_edge_oracle.cpp)

test-irq-edge: $(IRQ_EDGE_BIN)
	$(IRQ_EDGE_BIN)

$(IRQ_NEST_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_irq_nest_oracle.cpp
	@mkdir -p $(IRQ_NEST_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(IRQ_NEST_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_irq_nest_oracle.cpp)

test-irq-nesting: $(IRQ_NEST_BIN)
	$(IRQ_NEST_BIN)

$(IRQ_COMBO_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_irq_combo_oracle.cpp
	@mkdir -p $(IRQ_COMBO_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(IRQ_COMBO_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_irq_combo_oracle.cpp)

test-irq-combinations: $(IRQ_COMBO_BIN)
	$(IRQ_COMBO_BIN)

$(SSI_IRQ_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_ssi_irq_oracle.cpp
	@mkdir -p $(SSI_IRQ_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(SSI_IRQ_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_ssi_irq_oracle.cpp)

test-ssi-irq: $(SSI_IRQ_BIN)
	$(SSI_IRQ_BIN)

$(SSI_DATA_BIN): $(DSP_RTL)/dsp56001_periph.v dsp_ssi_data_oracle.cpp
	@mkdir -p $(SSI_DATA_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_periph \
		--Mdir $(SSI_DATA_BUILD_DIR) \
		-o Vdsp56001_periph \
		$(DSP_RTL)/dsp56001_periph.v \
		$(abspath dsp_ssi_data_oracle.cpp)

test-ssi-data: $(SSI_DATA_BIN)
	$(SSI_DATA_BIN)

$(SCI_VECTOR_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_sci_vector_oracle.cpp
	@mkdir -p $(SCI_VECTOR_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(SCI_VECTOR_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_sci_vector_oracle.cpp)

test-sci-vector: $(SCI_VECTOR_BIN)
	$(SCI_VECTOR_BIN)

$(MEM_IRQ_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_mem_irq_oracle.cpp
	@mkdir -p $(MEM_IRQ_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(MEM_IRQ_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_mem_irq_oracle.cpp)

test-memory-irq: $(MEM_IRQ_BIN)
	$(MEM_IRQ_BIN)

$(DO_DECODE_BIN): $(DSP_RTL)/dsp56001_decode.v dsp_do_decode_oracle.cpp
	@mkdir -p $(DO_DECODE_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_decode \
		--Mdir $(DO_DECODE_BUILD_DIR) \
		-o Vdsp56001_decode \
		$(DSP_RTL)/dsp56001_decode.v \
		$(abspath dsp_do_decode_oracle.cpp)

test-do-decode: $(DO_DECODE_BIN)
	$(DO_DECODE_BIN)

$(REP_DECODE_BIN): $(DSP_RTL)/dsp56001_decode.v dsp_rep_decode_oracle.cpp
	@mkdir -p $(REP_DECODE_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_decode \
		--Mdir $(REP_DECODE_BUILD_DIR) \
		-o Vdsp56001_decode \
		$(DSP_RTL)/dsp56001_decode.v \
		$(abspath dsp_rep_decode_oracle.cpp)

test-rep-decode: $(REP_DECODE_BIN)
	$(REP_DECODE_BIN)

$(LOOP_TIMING_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_loop_timing_oracle.cpp
	@mkdir -p $(LOOP_TIMING_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(LOOP_TIMING_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_loop_timing_oracle.cpp)

test-loop-timing: $(LOOP_TIMING_BIN)
	$(LOOP_TIMING_BIN)

$(ANDI_ORI_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_andi_ori_oracle.cpp
	@mkdir -p $(ANDI_ORI_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(ANDI_ORI_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_andi_ori_oracle.cpp)

test-andi-ori: $(ANDI_ORI_BIN)
	$(ANDI_ORI_BIN)

$(DIV_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_div_oracle.cpp
	@mkdir -p $(DIV_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(DIV_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_div_oracle.cpp)

test-div: $(DIV_BIN)
	$(DIV_BIN)

$(NORM_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_norm_oracle.cpp
	@mkdir -p $(NORM_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(NORM_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_norm_oracle.cpp)

test-norm: $(NORM_BIN)
	$(NORM_BIN)

$(TFR_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_tfr_oracle.cpp
	@mkdir -p $(TFR_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(TFR_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_tfr_oracle.cpp)

test-tfr: $(TFR_BIN)
	$(TFR_BIN)

$(RND_TIMING_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_rnd_timing_oracle.cpp
	@mkdir -p $(RND_TIMING_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(RND_TIMING_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_rnd_timing_oracle.cpp)

test-rnd-timing: $(RND_TIMING_BIN)
	$(RND_TIMING_BIN)

$(SYSTEM_REGS_BIN): $(DSP_RTL)/dsp56001_periph.v \
		dsp_system_regs_oracle.cpp
	@mkdir -p $(SYSTEM_REGS_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_periph \
		--Mdir $(SYSTEM_REGS_BUILD_DIR) \
		-o Vdsp56001_periph \
		$(DSP_RTL)/dsp56001_periph.v \
		$(abspath dsp_system_regs_oracle.cpp)

test-system-registers: $(SYSTEM_REGS_BIN)
	$(SYSTEM_REGS_BIN)

$(SR_RESERVED_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_sr_reserved_oracle.cpp
	@mkdir -p $(SR_RESERVED_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(SR_RESERVED_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_sr_reserved_oracle.cpp)

test-sr-reserved: $(SR_RESERVED_BIN)
	$(SR_RESERVED_BIN)

$(ILLEGAL_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_illegal_oracle.cpp
	@mkdir -p $(ILLEGAL_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(ILLEGAL_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_illegal_oracle.cpp)

test-illegal: $(ILLEGAL_BIN)
	$(ILLEGAL_BIN)

$(BIT_TIMING_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_bit_timing_oracle.cpp
	@mkdir -p $(BIT_TIMING_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(BIT_TIMING_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_bit_timing_oracle.cpp)

test-bit-timing: $(BIT_TIMING_BIN)
	$(BIT_TIMING_BIN)

$(PARALLEL_MOVE_TIMING_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_parallel_move_timing_oracle.cpp
	@mkdir -p $(PARALLEL_MOVE_TIMING_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(PARALLEL_MOVE_TIMING_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_parallel_move_timing_oracle.cpp)

test-parallel-move-timing: $(PARALLEL_MOVE_TIMING_BIN)
	$(PARALLEL_MOVE_TIMING_BIN)

$(XY_PARALLEL_MOVE_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_xy_parallel_move_oracle.cpp
	@mkdir -p $(XY_PARALLEL_MOVE_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(XY_PARALLEL_MOVE_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_xy_parallel_move_oracle.cpp)

test-xy-parallel-move: $(XY_PARALLEL_MOVE_BIN)
	$(XY_PARALLEL_MOVE_BIN)

$(CLASS1_SELECTOR_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_class1_selector_oracle.cpp
	@mkdir -p $(CLASS1_SELECTOR_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) \
		--top-module dsp56001_core \
		--Mdir $(CLASS1_SELECTOR_BUILD_DIR) \
		-o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_class1_selector_oracle.cpp)

test-class1-selector: $(CLASS1_SELECTOR_BIN)
	$(CLASS1_SELECTOR_BIN)

$(MOVEC_MEMORY_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_movec_memory_oracle.cpp
	@mkdir -p $(MOVEC_MEMORY_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_core \
		--Mdir $(MOVEC_MEMORY_BUILD_DIR) -o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v $(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v $(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_movec_memory_oracle.cpp)

test-movec-memory: $(MOVEC_MEMORY_BIN)
	$(MOVEC_MEMORY_BIN)

$(MOVEC_REGISTER_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_movec_register_oracle.cpp
	@mkdir -p $(MOVEC_REGISTER_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_core \
		--Mdir $(MOVEC_REGISTER_BUILD_DIR) -o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v $(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v $(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_movec_register_oracle.cpp)

test-movec-register: $(MOVEC_REGISTER_BIN)
	$(MOVEC_REGISTER_BIN)

$(MOVEC_TIMING_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_movec_timing_oracle.cpp
	@mkdir -p $(MOVEC_TIMING_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_core \
		--Mdir $(MOVEC_TIMING_BUILD_DIR) -o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v $(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v $(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_movec_timing_oracle.cpp)

test-movec-timing: $(MOVEC_TIMING_BIN)
	$(MOVEC_TIMING_BIN)

$(MOVEM_TIMING_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_movem_timing_oracle.cpp
	@mkdir -p $(MOVEM_TIMING_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_core \
		--Mdir $(MOVEM_TIMING_BUILD_DIR) -o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v $(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v $(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_movem_timing_oracle.cpp)

test-movem-timing: $(MOVEM_TIMING_BIN)
	$(MOVEM_TIMING_BIN)

$(MOVEP_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_movep_oracle.cpp
	@mkdir -p $(MOVEP_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_core \
		--Mdir $(MOVEP_BUILD_DIR) -o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v $(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v $(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_movep_oracle.cpp)

test-movep: $(MOVEP_BIN)
	$(MOVEP_BIN)

$(MOVEP_TIMING_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_movep_timing_oracle.cpp
	@mkdir -p $(MOVEP_TIMING_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_core \
		--Mdir $(MOVEP_TIMING_BUILD_DIR) -o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v $(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v $(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_movep_timing_oracle.cpp)

test-movep-timing: $(MOVEP_TIMING_BIN)
	$(MOVEP_TIMING_BIN)

$(P_DATA_WAIT_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_p_data_wait_oracle.cpp
	@mkdir -p $(P_DATA_WAIT_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_core \
		--Mdir $(P_DATA_WAIT_BUILD_DIR) -o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v $(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v $(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_p_data_wait_oracle.cpp)

test-p-data-wait: $(P_DATA_WAIT_BIN)
	$(P_DATA_WAIT_BIN)

$(P_FETCH_WAIT_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_p_fetch_wait_oracle.cpp
	@mkdir -p $(P_FETCH_WAIT_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_core \
		--Mdir $(P_FETCH_WAIT_BUILD_DIR) -o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v $(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v $(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_p_fetch_wait_oracle.cpp)

test-p-fetch-wait: $(P_FETCH_WAIT_BIN)
	$(P_FETCH_WAIT_BIN)

$(ADDRESS_REGISTER_LATENCY_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_address_register_latency_oracle.cpp
	@mkdir -p $(ADDRESS_REGISTER_LATENCY_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_core \
		--Mdir $(ADDRESS_REGISTER_LATENCY_BUILD_DIR) -o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v $(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v $(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_address_register_latency_oracle.cpp)

test-address-register-latency: $(ADDRESS_REGISTER_LATENCY_BIN)
	$(ADDRESS_REGISTER_LATENCY_BIN)

$(EXTERNAL_WAIT_STATE_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_external_wait_state_oracle.cpp
	@mkdir -p $(EXTERNAL_WAIT_STATE_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_core \
		--Mdir $(EXTERNAL_WAIT_STATE_BUILD_DIR) -o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v $(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v $(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_external_wait_state_oracle.cpp)

test-external-wait-state: $(EXTERNAL_WAIT_STATE_BIN)
	$(EXTERNAL_WAIT_STATE_BIN)

$(LATE_WRITEBACK_WAIT_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_late_writeback_wait_oracle.cpp
	@mkdir -p $(LATE_WRITEBACK_WAIT_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_core \
		--Mdir $(LATE_WRITEBACK_WAIT_BUILD_DIR) -o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v $(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v $(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_late_writeback_wait_oracle.cpp)

test-late-writeback-wait: $(LATE_WRITEBACK_WAIT_BIN)
	$(LATE_WRITEBACK_WAIT_BIN)

$(RY_CLASS1_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_ry_class1_oracle.cpp
	@mkdir -p $(RY_CLASS1_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_core \
		--Mdir $(RY_CLASS1_BUILD_DIR) -o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v $(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v $(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_ry_class1_oracle.cpp)

test-ry-class1: $(RY_CLASS1_BIN)
	$(RY_CLASS1_BIN)

$(TRACE_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_trace_oracle.cpp
	@mkdir -p $(TRACE_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_core \
		--Mdir $(TRACE_BUILD_DIR) -o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v $(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v $(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_trace_oracle.cpp)

test-trace: $(TRACE_BIN)
	$(TRACE_BIN)

$(XR_CLASS1_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_xr_class1_oracle.cpp
	@mkdir -p $(XR_CLASS1_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_core \
		--Mdir $(XR_CLASS1_BUILD_DIR) -o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v $(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v $(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_xr_class1_oracle.cpp)

test-xr-class1: $(XR_CLASS1_BIN)
	$(XR_CLASS1_BIN)

$(XR_RY_CLASS2_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_xr_ry_class2_oracle.cpp
	@mkdir -p $(XR_RY_CLASS2_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_core \
		--Mdir $(XR_RY_CLASS2_BUILD_DIR) -o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v $(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v $(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_xr_ry_class2_oracle.cpp)

test-xr-ry-class2: $(XR_RY_CLASS2_BIN)
	$(XR_RY_CLASS2_BIN)

$(AGU_BOUNDARY_BIN): $(DSP_RTL)/dsp56001_agu.v dsp_agu_boundary_oracle.cpp
	@mkdir -p $(AGU_BOUNDARY_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_agu \
		--Mdir $(AGU_BOUNDARY_BUILD_DIR) -o Vdsp56001_agu \
		$(DSP_RTL)/dsp56001_agu.v $(abspath dsp_agu_boundary_oracle.cpp)

test-agu-boundary: $(AGU_BOUNDARY_BIN)
	$(AGU_BOUNDARY_BIN)

$(LOGIC_CCR_BIN): $(DSP_RTL)/dsp56001_alu.v dsp_logic_ccr_oracle.cpp
	@mkdir -p $(LOGIC_CCR_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_alu \
		--Mdir $(LOGIC_CCR_BUILD_DIR) -o Vdsp56001_alu \
		$(DSP_RTL)/dsp56001_alu.v $(abspath dsp_logic_ccr_oracle.cpp)

test-logic-ccr: $(LOGIC_CCR_BIN)
	$(LOGIC_CCR_BIN)

$(PORT_C_BIN): $(DSP_RTL)/dsp56001_periph.v dsp_port_c_oracle.cpp
	@mkdir -p $(PORT_C_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_periph \
		--Mdir $(PORT_C_BUILD_DIR) -o Vdsp56001_periph \
		$(DSP_RTL)/dsp56001_periph.v $(abspath dsp_port_c_oracle.cpp)

test-port-c: $(PORT_C_BIN)
	$(PORT_C_BIN)

$(ROM_TABLES_BIN): $(DSP_RTL)/dsp56001_mem.v dsp_rom_tables_oracle.cpp
	@mkdir -p $(ROM_TABLES_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_mem \
		--Mdir $(ROM_TABLES_BUILD_DIR) -o Vdsp56001_mem \
		$(DSP_RTL)/dsp56001_mem.v $(abspath dsp_rom_tables_oracle.cpp)

test-rom-tables: $(ROM_TABLES_BIN)
	$(ROM_TABLES_BIN)

$(STACK_ERROR_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_stack_error_oracle.cpp
	@mkdir -p $(STACK_ERROR_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_core \
		--Mdir $(STACK_ERROR_BUILD_DIR) -o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v $(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v $(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_stack_error_oracle.cpp)

test-stack-error: $(STACK_ERROR_BIN)
	$(STACK_ERROR_BIN)

$(STOP_DIRECT_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_stop_direct_oracle.cpp
	@mkdir -p $(STOP_DIRECT_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_core \
		--Mdir $(STOP_DIRECT_BUILD_DIR) -o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v $(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v $(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_stop_direct_oracle.cpp)

test-stop-direct: $(STOP_DIRECT_BIN)
	$(STOP_DIRECT_BIN)

$(RESET_DIRECT_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_reset_direct_oracle.cpp
	@mkdir -p $(RESET_DIRECT_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_core \
		--Mdir $(RESET_DIRECT_BUILD_DIR) -o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v $(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v $(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_reset_direct_oracle.cpp)

test-reset-direct: $(RESET_DIRECT_BIN)
	$(RESET_DIRECT_BIN)

$(BIT_REGISTERS_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_bit_register_oracle.cpp
	@mkdir -p $(BIT_REGISTERS_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_core \
		--Mdir $(BIT_REGISTERS_BUILD_DIR) -o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v $(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v $(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_bit_register_oracle.cpp)

test-bit-registers: $(BIT_REGISTERS_BIN)
	$(BIT_REGISTERS_BIN)

$(AUDIO_GLUE_BIN): $(DSP_RTL)/dsp56001_top.v dsp_audio_glue_oracle.cpp
	@mkdir -p $(AUDIO_GLUE_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module indigo_hp1_audio_glue \
		--Mdir $(AUDIO_GLUE_BUILD_DIR) -o Vindigo_hp1_audio_glue \
		$(DSP_RTL)/dsp56001_top.v $(abspath dsp_audio_glue_oracle.cpp)

test-audio-glue: $(AUDIO_GLUE_BIN)
	$(AUDIO_GLUE_BIN)

$(SPDIF_BIN): $(BOARD_RTL)/cyan_spdif_tx.v dsp_spdif_oracle.cpp
	@mkdir -p $(SPDIF_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module cyan_spdif_tx \
		-GCLK_HZ=6144000 --Mdir $(SPDIF_BUILD_DIR) -o Vcyan_spdif_tx \
		$(BOARD_RTL)/cyan_spdif_tx.v $(abspath dsp_spdif_oracle.cpp)

test-spdif: $(SPDIF_BIN)
	$(SPDIF_BIN)

$(LUA_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_lua_direct_oracle.cpp
	@mkdir -p $(LUA_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_core \
		--Mdir $(LUA_BUILD_DIR) -o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v $(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v $(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_lua_direct_oracle.cpp)

test-lua: $(LUA_BIN)
	$(LUA_BIN)

$(TCC_BIN): $(DSP_RTL)/dsp56001_core.v \
		$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
		$(DSP_RTL)/dsp56001_agu.v dsp_tcc_direct_oracle.cpp
	@mkdir -p $(TCC_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_core \
		--Mdir $(TCC_BUILD_DIR) -o Vdsp56001_core \
		$(DSP_RTL)/dsp56001_core.v $(DSP_RTL)/dsp56001_decode.v \
		$(DSP_RTL)/dsp56001_alu.v $(DSP_RTL)/dsp56001_agu.v \
		$(abspath dsp_tcc_direct_oracle.cpp)

test-tcc-direct: $(TCC_BIN)
	$(TCC_BIN)

TOP_DMA_RTL := $(DSP_RTL)/dsp56001_top.v $(DSP_RTL)/dsp56001_core.v \
	$(DSP_RTL)/dsp56001_decode.v $(DSP_RTL)/dsp56001_alu.v \
	$(DSP_RTL)/dsp56001_agu.v $(DSP_RTL)/dsp56001_mem.v \
	$(DSP_RTL)/dsp56001_periph.v $(DSP_RTL)/dsp56001_host.v \
	$(DSP_RTL)/dsp56001_sram.v $(DSP_RTL)/dsp56001_dma.v

$(TOP_DMA_BIN): $(TOP_DMA_RTL) dsp_top_dma_contention_oracle.cpp
	@mkdir -p $(TOP_DMA_BUILD_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module dsp56001_top \
		--Mdir $(TOP_DMA_BUILD_DIR) -o Vdsp56001_top \
		$(TOP_DMA_RTL) $(abspath dsp_top_dma_contention_oracle.cpp)

test-top-dma-contention: $(TOP_DMA_BIN)
	$(TOP_DMA_BIN)
