GHDL = ghdl
GHDLFLAGS = --std=08

NAME = pid_controller

all: build
build: $(NAME)_tb
test: $(NAME)_tb.ghw

clean:
	@git clean -fdX
.PHONY: clean

# Build test bench executables.
$(NAME)_tb: $(NAME).vhd $(NAME)_tb.vhd
	$(GHDL) analyze $(GHDLFLAGS) $^
	$(GHDL) elaborate $(GHDLFLAGS) $@

# Run tests.
$(NAME)_tb.ghw: build
	$(GHDL) run $(GHDLFLAGS) $(NAME)_tb --wave="$@"
