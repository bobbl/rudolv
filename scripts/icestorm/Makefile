# Examples for the usage:
#
# make DEVICE=hx8k arachne timing
#       Route with arachne for the hx8k chip and print the clock rate estimate
#
# make DEVICE=up5k nextpnr prog
#       Route with nextpnr for the up5k chip and program it via usb

include ../../config.mk

CODE = ../../sw/tests/build/simple.hex
DEVICE = hx8k
VERILOG_FILES = ../../pipeline.v
ARACHNE_DEVICE = $(subst up,,$(subst hx,,$(subst lp,,$(DEVICE))))



all: arachne timing

code.hex: $(CODE)
	cp $(CODE) code.hex

%.blif: %.v $(VERILOG_FILES) code.hex ../../sw/uart/bootloader.hex
	$(YOSYS) -ql $(DEVICE).log -p 'synth_ice40 -top top -blif $@' $< $(VERILOG_FILES)

%.json: %.v $(VERILOG_FILES)
	$(YOSYS) -ql $(DEVICE).log -p 'synth_ice40 -top top -json $@' $^

%.bin: %.asc
	$(ICEPACK) $< $@

%.rpt: %.asc
	$(ICETIME) -d $(DEVICE) -mtr $@ $<



prog: $(DEVICE).bin
	$(ICEPROG) $<

timing: $(DEVICE).rpt

arachne $(DEVICE).asc: $(DEVICE).pcf $(DEVICE).blif
#	$(ARACHNE) -d $(ARACHNE_DEVICE) -P uwg30 -o $(DEVICE).asc -p $(DEVICE).pcf $(DEVICE).blif
	$(ARACHNE) -d $(ARACHNE_DEVICE) -o $(DEVICE).asc -p $(DEVICE).pcf $(DEVICE).blif

nextpnr: $(DEVICE).json $(DEVICE).pcf
	$(NEXTPNR_ICE40) --$(DEVICE) --package uwg30 --json $(DEVICE).json --pcf $(DEVICE).pcf --asc $(DEVICE).asc


clean:
	rm -f code.hex $(DEVICE).blif $(DEVICE).json $(DEVICE).asc \
	    $(DEVICE).rpt $(DEVICE).bin $(DEVICE).log

.SECONDARY:
.PHONY: all prog timing prog arachne nextpnr clean
