PROJ = top

PIN_DEF = pins.pcf
DEVICE = hx8k
PKG = ct256

all: $(PROJ).bin
	#iceprog $(PROJ).bin

%.json: *.v
	yosys -q -l synth.log -p 'synth_ice40 -top $(PROJ) -json $@' $^

%.asc: $(PIN_DEF) %.json
	nextpnr-ice40 --$(DEVICE) --package $(PKG) --freq 20 --json $(PROJ).json --pcf $(PIN_DEF) --asc $(PROJ).asc

%.bin: %.asc
	icepack $< $@


%_syn.v: %.blif
	yosys -p 'read_blif -wideports $^; write_verilog $@'

clean:
	rm -f $(PROJ).blif $(PROJ).json $(PROJ).asc $(PROJ).bin

.SECONDARY:
.PHONY: all clean
