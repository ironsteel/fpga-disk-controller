PROJ = top

PIN_DEF = pins.pcf
DEVICE = hx8k
PKG = ct256

all: $(PROJ).bin
	#iceprog $(PROJ).bin

%.json: *.v sump2/*.v
	yosys -q -l synth.log -p 'synth_ice40 -top $(PROJ) -json $@' $^

%.asc: $(PIN_DEF) %.json
	nextpnr-ice40 --$(DEVICE) --package $(PKG) --freq 20 --json $(PROJ).json --pcf $(PIN_DEF) --asc $(PROJ).asc

%.bin: %.asc
	icepack $< $@

clean:
	rm -f $(PROJ).blif $(PROJ).json $(PROJ).asc $(PROJ).bin

.SECONDARY:
.PHONY: all clean
