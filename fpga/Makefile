PROJECT = kalman_fpga
SRC = ../src/project.v ../src/spi_master.v ../src/mpu_driver.v ../src/cordic.v ../src/kalman.v ../src/uart_tx.v
PIN_DEF = icebreaker.pcf
DEVICE = up5k
PACKAGE = sg48

all: $(PROJECT).bin

$(PROJECT).json: $(SRC)
	yosys -p "read_verilog $(SRC); chparam -set CLK_FREQ 12000000 tt_um_kalman; synth_ice40 -top tt_um_kalman -json $@"

$(PROJECT).asc: $(PROJECT).json $(PIN_DEF)
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --pcf $(PIN_DEF) --json $< --asc $@

$(PROJECT).bin: $(PROJECT).asc
	icepack $< $@

prog: $(PROJECT).bin
	iceprog $<

clean:
	rm -f $(PROJECT).json $(PROJECT).asc $(PROJECT).bin

.PHONY: all prog clean
