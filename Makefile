toolchaindir = ../toolchain
map = $(toolchaindir)/map
map_transform = $(toolchaindir)/map-transform
map_route = $(toolchaindir)/map-route
bg_gen = $(toolchaindir)/bg-generate
templor = $(toolchaindir)/fill-template
templatedir = $(toolchaindir)/templates

all: rover-transformed.map

targets: *.bsg
	$(foreach target,$^, $(MAKE) $(subst -c.bsg,_graph.h,$(target));)
	$(foreach target,$^, $(MAKE) $(subst -c.bsg,_graph.c,$(target));)
	$(foreach target,$^, $(MAKE) $(subst -c.bsg,.c,$(target));)
	$(foreach target,$^, $(MAKE) $(subst -vhdl.bsg,_config.vhd,$(target));)
	$(foreach target,$^, $(MAKE) $(subst -vhdl.bsg,_graph.vhd,$(target));)

clean:
	rm -f *.map
	rm -f *.btg
	rm -f *.bsg
	rm -f *.dict
	rm -f *.h
	rm -f *.c
	rm -f *.vhd

%.map: %.hwg %.bg 
	$(map) $^ $@

%-transformed.map: %.map
	$(map_transform) $^ $@

%-copy.map: *-transformed.map
	cp $< $@

%-routing.dict: %-copy.map
	$(map_route) $^ $* $@

%-c.dict: %-c.bsg
	$(bg_gen) -n $* -l C $^ $@

%-vhdl.dict: %-vhdl.bsg
	$(bg_gen) -n $* -l VHDL $^ $@

%-final-c.dict: %-c.dict %-routing.dict
	cat $^ > $@

%-final-vhdl.dict: %-vhdl.dict %-routing.dict
	cat $^ > $@

%_graph.h: $(templatedir)/*header_template.h %-final-c.dict
	$(templor) $^ $@

%_graph.c: $(templatedir)/*source_template.c %-final-c.dict
	$(templor) $^ $@

%.c: $(templatedir)/*toplvl_template.c %-final-c.dict
	$(templor) $^ $@
	$(templor) $^ $@

%_graph.vhd: $(templatedir)/*graph_template.vhd %-final-vhdl.dict
	$(templor) $^ $@

%_config.vhd: $(templatedir)/*config_template.vhd %-final-vhdl.dict
	$(templor) $^ $@
