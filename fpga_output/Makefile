# RV32-Cap FPGA Build Makefile
# Open-source toolchain: Yosys + nextpnr + icepack/ecppack
#
# Prerequisites (install on your local machine):
#   Lattice iCE40: yosys, nextpnr-ice40, icepack, iceprog
#   Lattice ECP5:  yosys, nextpnr-ecp5, ecppack, openFPGALoader
#
# Usage:
#   1. Generate Verilog: python -m ctmm_cap_amaranth.gen_verilog -o fpga_output
#   2. Edit constraints file for your board
#   3. make TARGET=ice40 synth pnr pack prog
#      or: make TARGET=ecp5 synth pnr pack prog

# ---- Configuration ----
TARGET     ?= ice40
TOP        := rv32_cap_fpga_top
VERILOG    := rv32_cap_fpga_top.v
BUILD_DIR  := build

# iCE40 settings (iCE40 HX8K)
ICE40_DEVICE := hx8k
ICE40_PACKAGE := ct256
ICE40_PCF    := constraints_ice40.pcf

# ECP5 settings (LFE5U-25F)
ECP5_DEVICE  := 25k
ECP5_PACKAGE := CABGA381
ECP5_LPF     := constraints_ecp5.lpf

# ---- Derived ----
JSON := $(BUILD_DIR)/$(TOP).json

ifeq ($(TARGET),ice40)
  CONSTRAINTS := $(ICE40_PCF)
  BITSTREAM   := $(BUILD_DIR)/$(TOP).bin
else ifeq ($(TARGET),ecp5)
  CONSTRAINTS := $(ECP5_LPF)
  BITSTREAM   := $(BUILD_DIR)/$(TOP).bit
endif

.PHONY: all gen synth pnr pack prog clean info

all: synth pnr pack

gen:
        cd .. && python -m ctmm_cap_amaranth.gen_verilog -o fpga_output

info:
        @echo "RV32-Cap FPGA Build"
        @echo "  Target:      $(TARGET)"
        @echo "  Top module:  $(TOP)"
        @echo "  Verilog:     $(VERILOG)"
        @echo "  Constraints: $(CONSTRAINTS)"
        @echo "  Build dir:   $(BUILD_DIR)"

$(BUILD_DIR):
        mkdir -p $(BUILD_DIR)

# ---- Synthesis (Yosys) ----
synth: $(JSON)

$(JSON): $(VERILOG) | $(BUILD_DIR)
ifeq ($(TARGET),ice40)
        yosys -p "read_verilog $(VERILOG); synth_ice40 -top $(TOP) -json $@"
else ifeq ($(TARGET),ecp5)
        yosys -p "read_verilog $(VERILOG); synth_ecp5 -top $(TOP) -json $@"
endif
        @echo "Synthesis complete: $@"

# ---- Place and Route ----
pnr: $(BUILD_DIR)/$(TOP).pnr

$(BUILD_DIR)/$(TOP).pnr: $(JSON) $(CONSTRAINTS)
ifeq ($(TARGET),ice40)
        nextpnr-ice40 --$(ICE40_DEVICE) --package $(ICE40_PACKAGE) \
                --pcf $(ICE40_PCF) --json $(JSON) \
                --asc $@ --report $(BUILD_DIR)/report.json
else ifeq ($(TARGET),ecp5)
        nextpnr-ecp5 --$(ECP5_DEVICE) --package $(ECP5_PACKAGE) \
                --lpf $(ECP5_LPF) --json $(JSON) \
                --textcfg $@ --report $(BUILD_DIR)/report.json
endif
        @echo "Place and route complete: $@"

# ---- Bitstream Packing ----
pack: $(BITSTREAM)

ifeq ($(TARGET),ice40)
$(BITSTREAM): $(BUILD_DIR)/$(TOP).pnr
        icepack $< $@
        @echo "Bitstream packed: $@"
else ifeq ($(TARGET),ecp5)
$(BITSTREAM): $(BUILD_DIR)/$(TOP).pnr
        ecppack --input $< --bit $@
        @echo "Bitstream packed: $@"
endif

# ---- Program FPGA ----
prog: $(BITSTREAM)
ifeq ($(TARGET),ice40)
        iceprog $(BITSTREAM)
else ifeq ($(TARGET),ecp5)
        openFPGALoader --board ulx3s $(BITSTREAM)
endif
        @echo "FPGA programmed successfully"

# ---- Resource Usage Report ----
report: $(BUILD_DIR)/report.json
        @python3 -c "import json; r=json.load(open('$<')); u=r.get('utilization',{}); \
                print('Resource Usage:'); \
                [print(f'  {k}: {v}') for k,v in u.items()]"

# ---- Clean ----
clean:
        rm -rf $(BUILD_DIR)
        @echo "Build directory cleaned"
