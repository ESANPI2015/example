---------------------------------------------------------------------------------
--! @file    config.vhd
--! @class   config
--!
--! @brief   Configuration package for the BLDCJointController
--! @details Configuration package for the BLDCJointController.\n\n
--!          German Research Center for Artificial Intelligence\n
--!
--! @author Tobias Stark (tobias.stark@dfki.de)
--! @date   07.03.2013
---------------------------------------------------------------------------------
-- Last Commit: 
-- $LastChangedRevision: 4454 $
-- $LastChangedBy: tstark $
-- $LastChangedDate: 2016-03-03 12:16:58 +0100 (Do, 03 Mär 2016) $
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

    constant CLK_FREQ         : integer := 20_000_000;
    constant PROM_CONFIG_BASE : std_logic_vector(23 downto 0) := x"1f0000";
    constant TIMEOUT_SECS     : real := 0.5;

    -- Here the widths of the signals which hold the encoder position and speed values could be configured:
    -- ENC_POS_WIDTH: This should hold the TICKS_PER_TURN value for single-turn implementations (e.g. 204800 -> 18bit).
    -- ENC_SPEED_WIDTH: This should bold TICKS_PER_TURN+SPEED_SHIFT for the full speed range (e.g. +-128 turns/s for SPEED_SHIFT 8)
    constant ENC_POS_WIDTH   : integer := 18;  -- encoder position (ticks)
    constant ENC_SPEED_WIDTH : integer := 20; --26;  -- encoder speed (ticks/s)

    -- ------ configuration for register access ------
    
    constant PROM_REGISTER_MEMORY_SIZE : integer := BLDCJointMC_PROM_REGISTER_MEMORY_SIZE;
    constant RAM_REGISTER_MEMORY_SIZE  : integer := BLDCJointMC_RAM_REGISTER_MEMORY_SIZE;
    constant RO_REGISTER_MEMORY_SIZE   : integer := BLDCJointMC_RO_REGISTER_MEMORY_SIZE;
    constant MAX_REGISTER_ID : integer := BLDCJointMC_MAX_REGISTER_ID;
    constant MIN_REGISTER_ID : integer := BLDCJointMC_MIN_REGISTER_ID;
    alias REGISTER_TYPES      : reg_types_t    (MAX_REGISTER_ID+1 downto MIN_REGISTER_ID) is BLDCJointMC_REGISTER_TYPES;
    alias REGISTER_MEM_TYPES  : reg_mem_types_t(MAX_REGISTER_ID+1 downto MIN_REGISTER_ID) is BLDCJointMC_REGISTER_MEM_TYPES;
    alias REGISTER_MEMORY_MAP : reg_mem_map_t  (MAX_REGISTER_ID+1 downto MIN_REGISTER_ID) is BLDCJointMC_REGISTER_MEM_MAP;

-------- configuration for NodeCommunication

    constant LS_BAUD_RATE : integer := 921600; -- low-speed baud rate

    constant TELEMETRY_LENGTH   : natural := 16;
    constant TELEMETRY_MSG_ID   : std_logic_vector(7 downto 0) := Representation_Id_BLDCJointTelemetryMessage;

    type TelemetryValues is array (TELEMETRY_LENGTH-1 downto 0) of std_logic_vector(15 downto 0);

end;
