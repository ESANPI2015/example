# Exemplary project for toolchain testing

If ```repo``` has been used for bootstrapping, the toolchain has to be compiled with ```make```.
In this repo are three files:
- Makefile
- rover.hwg
- rover.bg

The first one is a special makefile which uses the toolchain to produce a mapping.
This mapping consists of the ```rover.hwg``` hardware graph and the ```rover.bg``` behaviour graph.
Currently, the behaviour graph does nothing fancy and is just used to demonstrate the mapping and generation workflow.
When you enter ```make``` a bunch of files is created.
The .bsg files are Behaviour SubGraphs, one for each target.
The .btg file is the Behaviour ToplvlGraph in which the subgraphs are connected with each other.
The .map files contain the actual mapping and the costs of the mappings.

# Generating target source

If you want to generate actual code for a specific target, enter ```make Control.c```.
This will generate the C-Code for the Behaviour Subgraph named Control and mapped to target Control which is a conventional (non-vhdl) target.
TBC
