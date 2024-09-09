-include Makefile.local

SIMBRICKS_LIB ?= $(abspath ../../../../lib)

verilator_src := VVTAShell.cc

srcs := vta_simbricks.cc
vsrc_dir := ../build/chisel
VERILATOR ?= verilator
CFLAGS := -DVTA_SIMBRICKS -I$(SIMBRICKS_LIB) -Wall -O2 -fno-omit-frame-pointer
CFLAGS += -std=c++17
LDFLAGS := -L$(SIMBRICKS_LIB)/simbricks/nicif -L$(SIMBRICKS_LIB)
VFLAGS := --cc -O3 --trace
VFLAGS += +define+RANDOMIZE_GARBAGE_ASSIGN
VFLAGS += +define+RANDOMIZE_REG_INIT
VFLAGS += +define+RANDOMIZE_MEM_INIT
VFLAGS += --x-assign unique
VFLAGS += --output-split 20000
VFLAGS += --output-split-cfuncs 20000
CHISEL_BUILD_DIR := $(abspath ../build/chisel/)

all: vta_simbricks-1x16 vta_simbricks-4x16 vta_simbricks-8x16 \
	vta_simbricks-1x32 vta_simbricks-4x32 vta_simbricks-8x32

$(vsrc_dir)/VTA.DefaultVTAConfig_1x16.sv:
	$(MAKE) -C ../hardware/chisel/ verilog CONFIG=DefaultVTAConfig_1x16

$(vsrc_dir)/VTA.DefaultVTAConfig_4x16.sv:
	$(MAKE) -C ../hardware/chisel/ verilog CONFIG=DefaultVTAConfig_4x16

$(vsrc_dir)/VTA.DefaultVTAConfig_8x16.sv:
	$(MAKE) -C ../hardware/chisel/ verilog CONFIG=DefaultVTAConfig_8x16

$(vsrc_dir)/VTA.DefaultVTAConfig_1x32.sv:
	JAVA_OPTS="-Xmx4g" $(MAKE) -C ../hardware/chisel/ verilog CONFIG=DefaultVTAConfig_1x32

$(vsrc_dir)/VTA.DefaultVTAConfig_4x32.sv:
	JAVA_OPTS="-Xmx4g" $(MAKE) -C ../hardware/chisel/ verilog CONFIG=DefaultVTAConfig_4x32

$(vsrc_dir)/VTA.DefaultVTAConfig_8x32.sv:
	JAVA_OPTS="-Xmx4g" $(MAKE) -C ../hardware/chisel/ verilog CONFIG=DefaultVTAConfig_8x32

obj_dir-1x16/$(verilator_src): $(vsrc_dir)/VTA.DefaultVTAConfig_1x16.sv
	$(VERILATOR) $(VFLAGS) \
	    -CFLAGS "$(CFLAGS)" \
	    --top-module VTAShell \
	    --Mdir $(dir $@) \
	    -LDFLAGS "$(LDFLAGS) -lnicif -lsimbricks" \
		"-I$(CHISEL_BUILD_DIR)" \
	    $< --exe $(abspath $(srcs))

obj_dir-4x16/$(verilator_src): $(vsrc_dir)/VTA.DefaultVTAConfig_4x16.sv
	$(VERILATOR) $(VFLAGS) \
	    -CFLAGS "$(CFLAGS)" \
	    --top-module VTAShell \
	    --Mdir $(dir $@) \
	    -LDFLAGS "$(LDFLAGS) -lnicif -lsimbricks" \
		"-I$(CHISEL_BUILD_DIR)" \
	    $< --exe $(abspath $(srcs))

obj_dir-8x16/$(verilator_src): $(vsrc_dir)/VTA.DefaultVTAConfig_8x16.sv
	$(VERILATOR) $(VFLAGS) \
	    -CFLAGS "$(CFLAGS)" \
	    --top-module VTAShell \
	    --Mdir $(dir $@) \
	    -LDFLAGS "$(LDFLAGS) -lnicif -lsimbricks" \
		"-I$(CHISEL_BUILD_DIR)" \
	    $< --exe $(abspath $(srcs))

obj_dir-1x32/$(verilator_src): $(vsrc_dir)/VTA.DefaultVTAConfig_1x32.sv
	$(VERILATOR) $(VFLAGS) \
	    -CFLAGS "$(CFLAGS)" \
	    --top-module VTAShell \
	    --Mdir $(dir $@) \
	    -LDFLAGS "$(LDFLAGS) -lnicif -lsimbricks" \
		"-I$(CHISEL_BUILD_DIR)" \
	    $< --exe $(abspath $(srcs))

obj_dir-4x32/$(verilator_src): $(vsrc_dir)/VTA.DefaultVTAConfig_4x32.sv
	$(VERILATOR) $(VFLAGS) \
	    -CFLAGS "$(CFLAGS)" \
	    --top-module VTAShell \
	    --Mdir $(dir $@) \
	    -LDFLAGS "$(LDFLAGS) -lnicif -lsimbricks" \
		"-I$(CHISEL_BUILD_DIR)" \
	    $< --exe $(abspath $(srcs))

obj_dir-8x32/$(verilator_src): $(vsrc_dir)/VTA.DefaultVTAConfig_8x32.sv
	$(VERILATOR) $(VFLAGS) \
	    -CFLAGS "$(CFLAGS)" \
	    --top-module VTAShell \
	    --Mdir $(dir $@) \
	    -LDFLAGS "$(LDFLAGS) -lnicif -lsimbricks" \
		"-I$(CHISEL_BUILD_DIR)" \
	    $< --exe $(abspath $(srcs))

vta_simbricks-1x16: obj_dir-1x16/$(verilator_src) $(srcs)
	$(MAKE) -C $(dir $<) -f VVTAShell.mk
	cp $(dir $<)/VVTAShell $@

vta_simbricks-4x16: obj_dir-4x16/$(verilator_src) $(srcs)
	$(MAKE) -C $(dir $<) -f VVTAShell.mk
	cp $(dir $<)/VVTAShell $@

vta_simbricks-8x16: obj_dir-8x16/$(verilator_src) $(srcs)
	$(MAKE) -C $(dir $<) -f VVTAShell.mk
	cp $(dir $<)/VVTAShell $@

vta_simbricks-1x32: obj_dir-1x32/$(verilator_src) $(srcs)
	$(MAKE) -C $(dir $<) -f VVTAShell.mk
	cp $(dir $<)/VVTAShell $@

vta_simbricks-4x32: obj_dir-4x32/$(verilator_src) $(srcs)
	$(MAKE) -C $(dir $<) -f VVTAShell.mk
	cp $(dir $<)/VVTAShell $@

vta_simbricks-8x32: obj_dir-8x32/$(verilator_src) $(srcs)
	$(MAKE) -C $(dir $<) -f VVTAShell.mk
	cp $(dir $<)/VVTAShell $@

clean:
	rm -rf obj_dir-1x16 obj_dir-4x16 obj_dir-8x16 obj_dir-1x32 obj_dir-4x32 \
		obj_dir-8x32 ../build
.PHONY: all clean
