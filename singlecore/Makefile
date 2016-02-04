toolchaindir = ../../toolchain
map = $(toolchaindir)/map
map_transform = $(toolchaindir)/map-transform
map_route = $(toolchaindir)/map-route
bg_gen = $(toolchaindir)/bg-generate
templor = $(toolchaindir)/fill-template
templatedir = $(toolchaindir)/templates
outdir = ./generated

all: singlecore-transformed.map

targets: c-targets vhdl-targets

c-targets: *-c.bsg
	mkdir -p $(outdir)
	$(foreach target,$^, $(MAKE) $(outdir)/$(subst -c.bsg,_graph.h,$(target));)
	$(foreach target,$^, $(MAKE) $(outdir)/$(subst -c.bsg,_graph.c,$(target));)
	$(foreach target,$^, $(MAKE) $(outdir)/$(subst -c.bsg,_comm.h,$(target));)
	$(foreach target,$^, $(MAKE) $(outdir)/$(subst -c.bsg,_comm.c,$(target));)
	$(foreach target,$^, $(MAKE) $(outdir)/$(subst -c.bsg,_toplvl.c,$(target));)

vhdl-targets: *-vhdl.bsg
	mkdir -p $(outdir)
	$(foreach target,$^, $(MAKE) $(outdir)/$(subst -vhdl.bsg,_config.vhd,$(target));)
	$(foreach target,$^, $(MAKE) $(outdir)/$(subst -vhdl.bsg,_graph.vhd,$(target));)

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

$(outdir)/%_graph.h: $(templatedir)/*graph_template.h %-final-c.dict
	$(templor) $^ $@

$(outdir)/%_graph.c: $(templatedir)/*graph_template.c %-final-c.dict
	$(templor) $^ $@

$(outdir)/%_comm.h: $(templatedir)/*comm_template.h %-final-c.dict
	$(templor) $^ $@

$(outdir)/%_comm.c: $(templatedir)/*comm_template.c %-final-c.dict
	$(templor) $^ $@

$(outdir)/%_toplvl.c: $(templatedir)/*toplvl_template.c %-final-c.dict
	$(templor) $^ $@

$(outdir)/%_graph.vhd: $(templatedir)/*graph_template.vhd %-final-vhdl.dict
	$(templor) $^ $@

$(outdir)/%_config.vhd: $(templatedir)/*config_template.vhd %-final-vhdl.dict
	$(templor) $^ $@
