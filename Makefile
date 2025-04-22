GHDL = ghdl
GHDLFLAGS = --std=08

NAME = pid_controller

all: build
build: $(NAME)_tb
test: $(NAME)_tb.ghw
wave: $(NAME)_tb.ghw
	gtkwave $< --script=$(NAME)_tb.tcl
.PHONY: wave

clean:
	@git clean -fdX
.PHONY: clean

# Build test bench executables.
$(NAME)_tb: $(NAME).vhd $(NAME)_tb.vhd
	$(GHDL) analyze $(GHDLFLAGS) $^
	$(GHDL) elaborate $(GHDLFLAGS) $@

# Run tests.
$(NAME)_tb.ghw: $(NAME)_tb
	$(GHDL) run $(GHDLFLAGS) $< --wave="$@"
