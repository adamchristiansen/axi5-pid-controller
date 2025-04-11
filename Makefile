BUILD = build
GHDL = ghdl
GHDLFLAGS = --std=08

all: build

build: __prepare_build
	@cd $(BUILD) && $(GHDL) analyze $(GHDLFLAGS) pid_controller.vhd

clean:
	@rm -rf $(BUILD)
.PHONY: clean

# All source and test files are copied to the build directory so that all
# compilation can be done outside of the source tree.
__prepare_build: $(BUILD)/pid_controller.vhd
$(BUILD)/pid_controller.vhd: pid_controller.vhd
$(BUILD)/%: %
	@mkdir -p $(BUILD)
	@cp $< $@
