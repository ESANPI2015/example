----------------------------------------------------------------------------------
-- Company: Germean Research Centre for Artificial Intelligence (DFKI)
--
-- Description: This file contains the configuration data for the FPGA IP cores
--              used in the respective design.
--
---------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;

library work;
use work.devices.all;
use work.deviceclasses.all;
use work.representations.all;
use work.register_types.all;

package config is

    constant STACK_VERSION : integer := 3;
	 
	constant TIMEOUT_SECS : real := 0.25;

    constant CLK_FREQ : natural := 50_000_000;
    constant PROM_CONFIG_BASE : std_logic_vector(23 downto 0) := x"1f0000";

    -- ------ configuration for register access ------
    
    constant PROM_REGISTER_MEMORY_SIZE : integer := PicoBlazeArrayMC_PROM_REGISTER_MEMORY_SIZE;
    constant RAM_REGISTER_MEMORY_SIZE  : integer := PicoBlazeArrayMC_RAM_REGISTER_MEMORY_SIZE;
    constant RO_REGISTER_MEMORY_SIZE   : integer := PicoBlazeArrayMC_RO_REGISTER_MEMORY_SIZE;
    constant MAX_REGISTER_ID : integer := PicoBlazeArrayMC_MAX_REGISTER_ID;
    constant MIN_REGISTER_ID : integer := PicoBlazeArrayMC_MIN_REGISTER_ID;
    alias REGISTER_TYPES      : reg_types_t    (MAX_REGISTER_ID+1 downto MIN_REGISTER_ID) is PicoBlazeArrayMC_REGISTER_TYPES;
    alias REGISTER_MEM_TYPES  : reg_mem_types_t(MAX_REGISTER_ID+1 downto MIN_REGISTER_ID) is PicoBlazeArrayMC_REGISTER_MEM_TYPES;
    alias REGISTER_MEMORY_MAP : reg_mem_map_t  (MAX_REGISTER_ID+1 downto MIN_REGISTER_ID) is PicoBlazeArrayMC_REGISTER_MEM_MAP;
	 
	 -- NDLCom stuff
    constant LS_BAUD_RATE : natural := 921600;
	constant USE_TELEMETRY : boolean := false;
end;
