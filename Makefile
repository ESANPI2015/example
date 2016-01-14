templatedir = ../toolchain/templates

all: rover-transformed.map

%.map: %.hwg %.bg 
	../toolchain/map $^ $@

%-transformed.map: %.map
	../toolchain/map-transform $^ $@

%-copy.map: *-transformed.map
	cp $< $@

%-routing.dict: %-copy.map
	../toolchain/map-route $^ $* $@

%-c.dict: %-c.bsg
	../toolchain/bg-generate -n $* -l C $^ $@

%-vhdl.dict: %-vhdl.bsg
	../toolchain/bg-generate -n $* -l VHDL $^ $@

%-final-c.dict: %-c.dict %-routing.dict
	cat $^ > $@

%-final-vhdl.dict: %-vhdl.dict %-routing.dict
	cat $^ > $@

%.h: $(templatedir)/*_template.h %-final-c.dict
	../toolchain/fill-template $^ $@

%.c: $(templatedir)/*_template.c %-final-c.dict
	../toolchain/fill-template $^ $@

%.vhd: $(templatedir)/*graph_template.vhd %-final-vhdl.dict
	../toolchain/fill-template $^ $@

%_config.vhd: $(templatedir)/*config_template.vhd %-final-vhdl.dict
	../toolchain/fill-template $^ $@
