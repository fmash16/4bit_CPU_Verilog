build:
		iverilog -o computer \
				testbenches/cpu_tb.v					
run: build
		vvp -n computer

clean:
		rm -rf computer

view:
		gtkwave machine.vcd gtkwave/config.gtkw
#
# tests:
#         bats tests/tests.bats

# .PHONY: build run clean view tests
