project = demo
toolchaindir = ../../toolchain
map = $(toolchaindir)/map
map_transform = $(toolchaindir)/map-transform
map_route = $(toolchaindir)/map-route
bg_gen = $(toolchaindir)/bg-generate
templor = $(toolchaindir)/fill-template
templatedir = $(toolchaindir)/templates
c_graph_headers = $(patsubst %-c.bsg, %_graph.h, $(wildcard *-c.bsg))
c_graph_sources = $(patsubst %-c.bsg, %_graph.cpp, $(wildcard *-c.bsg))
c_comm_headers = $(patsubst %-c.bsg, %_comm.h, $(wildcard *-c.bsg))
c_comm_sources = $(patsubst %-c.bsg, %_comm.cpp, $(wildcard *-c.bsg))
c_toplvl_sources = $(patsubst %-c.bsg, %_toplvl.cpp, $(wildcard *-c.bsg))
c_targets = $(patsubst %-c.bsg, %, $(wildcard *-c.bsg))
vhdl_graph_headers = $(patsubst %-vhdl.bsg, %_graph_config.vhd, $(wildcard *-vhdl.bsg))
vhdl_graph_sources = $(patsubst %-vhdl.bsg, %_graph.vhd, $(wildcard *-vhdl.bsg))
vhdl_comm_headers = $(patsubst %-vhdl.bsg, %_comm_config.vhd, $(wildcard *-vhdl.bsg))
vhdl_comm_sources = $(patsubst %-vhdl.bsg, %_comm.vhd, $(wildcard *-vhdl.bsg))
vhdl_ndlcom_sources = $(patsubst %-vhdl.bsg, %_ndlcom.vhd, $(wildcard *-vhdl.bsg))
vhdl_toplvl_sources = $(patsubst %-vhdl.bsg, %_toplvl.vhd, $(wildcard *-vhdl.bsg))
vhdl_targets = $(patsubst %-vhdl.bsg, %, $(wildcard *-vhdl.bsg))

all: $(project)-transformed.map

targets: $(c_targets) $(vhdl_targets)

clean:
	rm -f *.map
	rm -f *.btg
	rm -f *.bsg
	rm -f *.dict
	rm -f *.h
	rm -f *.c
	rm -f *.cpp
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

$(c_targets): %: %_graph.h %_graph.cpp %_comm.h %_comm.cpp %_toplvl.cpp
	mkdir -p $@
	mv $^ $@

$(c_graph_headers): %_graph.h: $(templatedir)/bg_graph_template.h %-final-c.dict
	$(templor) $^ $@

$(c_graph_sources): %_graph.cpp: $(templatedir)/bg_graph_template.c %-final-c.dict
	$(templor) $^ $@

$(c_comm_headers): %_comm.h: $(templatedir)/bg_graph_comm_template.h %-final-c.dict
	$(templor) $^ $@

$(c_comm_sources): %_comm.cpp: $(templatedir)/bg_graph_comm_template.c %-final-c.dict
	$(templor) $^ $@

$(c_toplvl_sources): %_toplvl.cpp: my_templates/%_template.cpp %-final-c.dict
	$(templor) $^ $@

$(vhdl_targets): %: %_graph_config.vhd %_graph.vhd %_comm_config.vhd %_comm.vhd %_ndlcom.vhd %_toplvl.vhd
	mkdir -p $@
	mv $^ $@

$(vhdl_graph_sources): %_graph.vhd: $(templatedir)/bg_graph_template.vhd %-final-vhdl.dict
	$(templor) $^ $@

$(vhdl_graph_headers): %_graph_config.vhd: $(templatedir)/bg_graph_config_template.vhd %-final-vhdl.dict
	$(templor) $^ $@

$(vhdl_comm_sources): %_comm.vhd: $(templatedir)/bg_graph_comm_template.vhd %-final-vhdl.dict
	$(templor) $^ $@

$(vhdl_comm_headers): %_comm_config.vhd: $(templatedir)/bg_graph_comm_config_template.vhd %-final-vhdl.dict
	$(templor) $^ $@

$(vhdl_ndlcom_sources): %_ndlcom.vhd: $(templatedir)/NDLCom_wrapper.vhd %-final-vhdl.dict
	$(templor) $^ $@

$(vhdl_toplvl_sources): %_toplvl.vhd: my_templates/%_template.vhd %-final-vhdl.dict
	$(templor) $^ $@
