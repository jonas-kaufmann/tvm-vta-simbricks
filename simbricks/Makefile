-include Makefile.local

bin_vta := vta_simbricks

verilator_dir := obj_dir
verilator_src := $(verilator_dir)/VVTAShell.cc

srcs := vta_simbricks.cc axi.cc
vsrcs := ../build/chisel/VTA.DefaultVTAConfig.sv
VERILATOR ?= verilator
CPPFLAGS := -DVTA_SIMBRICKS -I$(abspath ../../../../lib) $(EXTRA_CPPFLAGS)
CFLAGS := -O3 -Wall -Wno-maybe-uninitialized $(EXTRA_CFLAGS)
LDFLAGS := -L$(abspath ../../../../lib/simbricks/nicif) -L$(abspath ../../../../lib) $(EXTRA_LDFLAGS)
LDLIBS := $(EXTRA_LDLIBS)

all: $(bin_vta)

$(bin_vta): $(OBJS)

$(verilator_src): $(vsrcs)
	$(VERILATOR) $(VFLAGS) --cc -O3 \
	    -CFLAGS "$(CPPFLAGS) $(CFLAGS)" \
	    --top-module VTAShell \
	    --Mdir $(verilator_dir) \
	    -LDFLAGS "$(LDFLAGS) -lnicif -lsimbricks"  \
	    $(vsrcs) --exe $(abspath $(srcs))

$(bin_vta): $(srcs) $(vsrcs) $(verilator_src)
	$(MAKE) -C $(verilator_dir) -f VVTAShell.mk

$(vsrcs):
	$(MAKE) -C ../hardware/chisel/ verilog CONFIG=DefaultVTAConfig
clean:
	rm -rf $(verilator_dir)

.PHONY: all clean
