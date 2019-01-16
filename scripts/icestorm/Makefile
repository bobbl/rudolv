# Examples for the usage:
#
# make DEVICE=hx8k arachne timing
#       Route with arachne for the hx8k chip and print the clock rate estimate
#
# make DEVICE=up5k PACKAGE=uwg30 nextpnr prog
#       Route with nextpnr for the up5k chip (package uwg30) and program it via usb

include ../../config_default.mk

CODE = ../../sw/tests/build/simple.hex
DEVICE = hx8k
VERILOG_FILES = ../../pipeline.v
ARACHNE_DEVICE = 8k

include device_$(DEVICE).mk


all: arachne timing

%.hex:
	$(MAKE) -C `dirname $@`

bootloader_hx8k.hex: ../../sw/bootloader/bootloader.hex
	sed -e 's/@0 /@780 /' $< > $@

hx8k.blif: hx8k.v $(VERILOG_FILES) bootloader_hx8k.hex
	$(YOSYS) -ql $(DEVICE).log -p 'synth_ice40 -top top -blif $@' $< $(VERILOG_FILES)

up5k.blif: up5k.v $(VERILOG_FILES) ../../sw/bootloader/bootloader.hex
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
	$(ARACHNE) -d $(ARACHNE_DEVICE) $(ARACHNE_PACKAGE) -o $(DEVICE).asc -p $(DEVICE).pcf $(DEVICE).blif

nextpnr: $(DEVICE).json $(DEVICE).pcf
	$(NEXTPNR_ICE40) --$(DEVICE) $(NEXTPNR_PACKAGE) --json $(DEVICE).json --pcf $(DEVICE).pcf --asc $(DEVICE).asc


clean:
	rm -f code.hex $(DEVICE).blif $(DEVICE).json $(DEVICE).asc \
	    $(DEVICE).rpt $(DEVICE).bin $(DEVICE).log \
	    bootloader_hx8k.hex

.SECONDARY:
.PHONY: all prog timing prog arachne nextpnr clean
