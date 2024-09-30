null  :=
space := $(null) #
comma := ,

SIM = VERILATOR
COMPILE_ARGS += --trace --trace-structs

TEST_FILES   = $(sort $(wildcard test_*.py))

DUT      = i3c_harness
TOPLEVEL = $(DUT)
MODULE  ?= $(subst $(space),$(comma),$(subst .py,,$(TEST_FILES)))
VERILOG_SOURCES = \
	i3c_harness.sv

include $(shell cocotb-config --makefiles)/Makefile.sim
