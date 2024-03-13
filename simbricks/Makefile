-include Makefile.local

bin_vta := vta_simbricks

verilator_dir := obj_dir
verilator_src := $(verilator_dir)/VVTAShell.cc

srcs := vta_simbricks.cc
vsrcs := ../build/chisel/VTA.DefaultVTAConfig.sv
VERILATOR ?= verilator
CFLAGS := -DVTA_SIMBRICKS -I$(abspath ../../../../lib) -Wall -O2 -fno-omit-frame-pointer
CFLAGS += -std=c++14
LDFLAGS := -L$(abspath ../../../../lib/simbricks/nicif) -L$(abspath ../../../../lib/simbricks/rtl/axi) -L$(abspath ../../../../lib)
VFLAGS := --cc -O3 --trace
VFLAGS += +define+RANDOMIZE_GARBAGE_ASSIGN
VFLAGS += +define+RANDOMIZE_REG_INIT
VFLAGS += +define+RANDOMIZE_MEM_INIT
VFLAGS += --x-assign unique
VFLAGS += --output-split 20000
VFLAGS += --output-split-cfuncs 20000
CHISEL_BUILD_DIR := $(abspath ../build/chisel/)

all: $(bin_vta)

$(bin_vta): $(OBJS)

$(verilator_src): $(vsrcs)
	$(VERILATOR) $(VFLAGS) \
	    -CFLAGS "$(CFLAGS)" \
	    --top-module VTAShell \
	    --Mdir $(verilator_dir) \
	    -LDFLAGS "$(LDFLAGS) -lnicif -lsimbricks -lverilator_axi" \
		"-I$(CHISEL_BUILD_DIR)" \
	    $(vsrcs) --exe $(abspath $(srcs))

$(bin_vta): $(srcs) $(vsrcs) $(verilator_src)
	$(MAKE) -C $(verilator_dir) -f VVTAShell.mk
	cp $(verilator_dir)/VVTAShell $@

$(vsrcs):
	$(MAKE) -C ../hardware/chisel/ verilog CONFIG=DefaultVTAConfig
clean:
	rm -rf $(verilator_dir)

.PHONY: all clean
