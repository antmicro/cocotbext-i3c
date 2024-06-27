SIM = VERILATOR
VERILATOR_TRACE = 1
COMPILE_ARGS += --trace

DUT      = i3c_harness
TOPLEVEL = $(DUT)
MODULE   = gen_waves
VERILOG_SOURCES = \
	i3c_harness.sv

include $(shell cocotb-config --makefiles)/Makefile.sim
