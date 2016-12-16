----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    12:58:04 15/09/2015 
-- Design Name: 
-- Module Name:    SpineBoard - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
--  THIS IS JUST SOME TEST
--  Works on the iStruct Spine Electronics Board
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;

library work;
use work.config.all;
use work.register_types.all;
use work.register_pack.all;
use work.devices.all;
use work.deviceclasses.all;
use work.representations.all;
use work.bg_vhdl_types.all;
use work.bg_graph_@name@_comm_config.all;

entity spine_board is
    Port ( CLK  : in  std_logic;
           nRST : in  std_logic;
           nLED : out std_logic_vector(3 downto 0);
           --nDIP : in  std_logic_vector(3 downto 0);

           -- NDLCom LVDS
           LVDS_RX    : in  std_logic_vector(1 downto 0);
           LVDS_TX    : out std_logic_vector(1 downto 0);

           -- spi prom
          PROM_nCS    : out std_logic;
          PROM_SCK    : out std_logic;
          PROM_nWP    : out std_logic;
          PROM_nHOLD  : out std_logic;
          PROM_MISO   : in  std_logic;
          PROM_MOSI   : out std_logic
           
           );
       end spine_board;

architecture Behavioral of spine_board is

    signal reset : std_logic;
    signal enable  : std_logic;
    signal promInitDone : std_logic;
    signal resetDevice : std_logic;
    signal stopDevice  : std_logic;
    signal actualTime  : std_logic_vector(63 downto 0);
	 
-------------- EEPROM Interface -------------------------------------------------

    signal writePromTrigger   : std_logic;

    signal prom_erase_trigger : std_logic;
    signal prom_write_trigger : std_logic;
    signal prom_read_trigger  : std_logic;
    signal prom_busy          : std_logic;
    signal prom_address       : std_logic_vector(23 downto 0);
    signal prom_din_offset    : std_logic_vector(7 downto 0);
    signal prom_din           : std_logic_vector(7 downto 0);
    signal prom_din_we        : std_logic;
    signal prom_dout_offset   : std_logic_vector(7 downto 0);
    signal prom_dout          : std_logic_vector(7 downto 0);

    signal reg_prom_request       : std_logic;
    signal reg_prom_access        : std_logic;
    signal reg_prom_erase_trigger : std_logic;
    signal reg_prom_write_trigger : std_logic;
    signal reg_prom_read_trigger  : std_logic;
    signal reg_prom_address       : std_logic_vector(23 downto 0);
    signal reg_prom_din_offset    : std_logic_vector(7 downto 0);
    signal reg_prom_din           : std_logic_vector(7 downto 0);
    signal reg_prom_din_we        : std_logic;
    signal reg_prom_dout_offset   : std_logic_vector(7 downto 0);

    signal isp_prom_request       : std_logic;
    signal isp_prom_access        : std_logic;
    signal isp_prom_erase_trigger : std_logic;
    signal isp_prom_write_trigger : std_logic;
    signal isp_prom_read_trigger  : std_logic;
    signal isp_prom_address       : std_logic_vector(23 downto 0);
    signal isp_prom_din_offset    : std_logic_vector(6 downto 0);
    signal isp_prom_din           : std_logic_vector(7 downto 0);
    signal isp_prom_din_we        : std_logic;
    signal isp_prom_dout_offset   : std_logic_vector(6 downto 0);
    
-------------- Register ---------------------------------------------------------

    -- data structure to store register
    signal register_memory : registermemory_t;

    signal reg_busy       : std_logic;
    signal reg_read_id    : std_logic_vector(15 downto 0);
    signal reg_read_type  : registertype_t;
    signal reg_read_data  : std_logic_vector(63 downto 0);
    signal reg_read       : std_logic;
    signal reg_read_ack   : std_logic;
    signal reg_write_id   : std_logic_vector(15 downto 0);
    signal reg_write_type : registertype_t;
    signal reg_write_data : std_logic_vector(63 downto 0);
    signal reg_write      : std_logic;

-------------- ISP signals ------------------------------------------------------

    signal isp_cmd_upload     : std_logic;
    signal isp_cmd_data       : std_logic;
    signal isp_cmd_download   : std_logic;
    signal isp_cmd_ack        : std_logic;
    signal isp_din_addr       : std_logic_vector(31 downto 0);
    signal isp_din_len        : std_logic_vector(31 downto 0);
    signal isp_din            : std_logic_vector(7 downto 0);
    signal isp_din_valid      : std_logic;
    signal isp_din_ack        : std_logic;
    signal isp_dout_addr      : std_logic_vector(31 downto 0);
    signal isp_dout           : std_logic_vector(7 downto 0);
    signal isp_dout_valid     : std_logic;
    signal isp_dout_ack       : std_logic;
	 
-------------- Behaviour Graph signals -----------
	 signal sources            : DATA_PORT(NO_INTERNAL_INPUTS-1 downto 0);
	 signal sinks              : DATA_PORT(NO_INTERNAL_OUTPUTS-1 downto 0);
	 
	 signal bg_input_portid    : std_logic_vector(7 downto 0);
	 signal bg_input_data      : std_logic_vector(31 downto 0);
	 signal bg_input_req       : std_logic;
	 signal bg_input_ack       : std_logic;
	 
	 signal bg_output_recvid    : std_logic_vector(7 downto 0);
     signal bg_output_portid    : std_logic_vector(7 downto 0);
	 signal bg_output_data      : std_logic_vector(31 downto 0);
	 signal bg_output_req       : std_logic;
	 signal bg_output_ack       : std_logic;
	 
begin

  --source <= x"00000000";

  BehaviorGraph : entity work.bg_graph_@name@_comm(Behavioral)
		port map (
			clk => clk,
			rst => reset,
			halt => '0',
			-- Input
			in_portid => bg_input_portid,
			in_data => bg_input_data,
			in_req => bg_input_req,
			in_ack => bg_input_ack,
			-- Output
			out_recvid => bg_output_recvid,
			out_portid => bg_output_portid,
			out_data => bg_output_data,
			out_req => bg_output_req,
			out_ack => bg_output_ack,
			-- Local I/O
			in_sources => sources,
			out_sinks => sinks
		);

  ------------------------------------------------------------------------------------------
  -- Reset & Stop circuitry
  ------------------------------------------------------------------------------------------
  process (clk)
		variable counter : integer range 0 to 1023 := 0;
  begin
		if clk'event and clk='1' then
		   reset <= '1';
			if nRST='1' then
				if (counter = 1023) then
					reset <= '0' or resetDevice;
				else
					counter := counter + 1;
				end if;
		   else
			   counter := 0;
			end if;
		end if;
  end process;

    -- --------------------------------------------------------------------------
    --                           COMMUNICATION
    -- --------------------------------------------------------------------------

    nLED(0) <= not reset; -- blinks if resetting
    nLED(1) <= not bg_output_req; -- outgoing requests from behavior graph
    nLED(2) <= not bg_input_req; -- incoming request for behavior graph
    nLED(3) <= '0'; -- power on

    device_reconf : entity work.device_reconf_spartan6(Behavioral)
        port map ( clk => clk,
                   rst => reset,
                   reconf_trigger => '0');

    rtc_inst : entity work.rtc_mod(Behavioral)
        generic map ( SYSTEM_speed => CLK_FREQ )
        port map ( CLK => clk,
                   RST => reset,
                   actualTime => actualTime,
                   setNewTime => '0',
                   newTime    => (others => '0') );

    NDLCom_wrapper : entity work.@name@_NDLCom_wrapper
        generic map ( CLK_FREQ      => CLK_FREQ,
                      LS_BAUD_RATE     => LS_BAUD_RATE)
        port map ( CLK      => clk,
                   RST      => reset,
						 
				   ls_rx => LVDS_RX,
				   ls_tx => LVDS_TX,
                   
                   rtc_actual_time  => actualTime,
                   resetDevice      => resetDevice,
				   bg_input_portId  => bg_input_portid,
                   bg_input_data    => bg_input_data,
                   bg_input_req     => bg_input_req,
                   bg_input_ack     => bg_input_ack,
						 
                   bg_output_recvId => bg_output_recvId,
                   bg_output_portId => bg_output_portid,
                   bg_output_data   => bg_output_data,
                   bg_output_req    => bg_output_req,
                   bg_output_ack    => bg_output_ack,
						 
				   reg_read_id    => reg_read_id,
                   reg_read_type  => reg_read_type,
                   reg_read_data  => reg_read_data,
                   reg_read       => reg_read,
                   reg_read_ack   => reg_read_ack,
                   reg_write_id   => reg_write_id,
                   reg_write_type => reg_write_type,
                   reg_write_data => reg_write_data,
                   reg_write      => reg_write,
                   
                   isp_cmd_upload   => isp_cmd_upload,
                   isp_cmd_data     => isp_cmd_data,
                   isp_cmd_download => isp_cmd_download,
                   isp_cmd_ack      => isp_cmd_ack,
                   isp_din_addr     => isp_din_addr,
                   isp_din_len      => isp_din_len,
                   isp_din          => isp_din,
                   isp_din_valid    => isp_din_valid,
                   isp_din_ack      => isp_din_ack,
                   isp_dout_addr    => isp_dout_addr,
                   isp_dout         => isp_dout,
                   isp_dout_valid   => isp_dout_valid,
                   isp_dout_ack     => isp_dout_ack

                   --commErr => commError,
						 --nc_last_error => errorCode
						 );

    comm_proc : process (clk)
        variable oldWritePromState : std_logic;
        variable newWritePromState : std_logic;
        variable writePromTrigger_pending : std_logic;
    begin
        if clk'event and clk='1' then
            if reset='1' then
                enable  <= '0';
                writePromTrigger  <= '0';
                oldWritePromState := '0';
                newWritePromState := '0';
                writePromTrigger_pending := '0';
            else
                -- defaults
                writePromTrigger <= '0';

                --node_id <= get_register(register_memory,NDLCom_NODE_ID);

                --newWritePromState := get_register(register_memory,BLDCJoint_CONFIG)(BLDCJoint_CONFIG_WRITE_SETTINGS_BIT);
                --if newWritePromState='1' and oldWritePromState='0' then
                    --writePromTrigger_pending := '1';
                --end if;
                --oldWritePromState := newWritePromState;

                --if writePromTrigger_pending='1' and reg_busy='0' then
                    --writePromTrigger  <= '1';
                    --writePromTrigger_pending := '0';
                --end if;

                --enable <= get_register(register_memory,BLDCJoint_CONFIG)(BLDCJoint_CONFIG_ENABLE_CMD_BIT);

            end if;
        end if;
    end process;
    
    
    -- --------------------------------------------------------------------------
    --                        SPI-PROM Access
    -- --------------------------------------------------------------------------
    
    -- instantiation of the simple spiprom interface
    spiprom_inst : entity work.simple_spiprom_interface(Behavioral)
        generic map ( CLK_FREQ     => CLK_FREQ,
                      SPI_CLK_FREQ => 5000000,
                      DATA_LENGTH  => 128 )
        port map ( CLK => clk,
                   RST => reset,

                   SPI_nCS   => PROM_nCS,
                   SPI_CLK   => PROM_SCK,
                   SPI_DO    => PROM_MOSI,
                   SPI_DI    => PROM_MISO,

                   erase_trigger => prom_erase_trigger,
                   write_trigger => prom_write_trigger,
                   read_trigger  => prom_read_trigger,
                   prom_busy     => prom_busy,
                   address       => prom_address,

                   -- interface to the page buffers
                   din_we        => prom_din_we,
                   din_offset    => prom_din_offset,
                   din           => prom_din,
                   dout_offset   => prom_dout_offset,
                   dout          => prom_dout );

    PROM_nHOLD <= '1';
    PROM_nWP   <= '1';

    prom_access_handler : process (clk)
    begin
        if clk'event and clk='1' then
            if reset='1' then
                reg_prom_access <= '0';
                isp_prom_access <= '0';
            else
                if reg_prom_request='1' and isp_prom_access='0' then
                    reg_prom_access <= '1';
                elsif isp_prom_request='1' and reg_prom_access='0' then
                    isp_prom_access <= '1';
                else
                    reg_prom_access <= '0';
                    isp_prom_access <= '0';
                end if;
            end if;
        end if;
    end process;

    prom_erase_trigger <= reg_prom_erase_trigger when reg_prom_access='1' else
                          isp_prom_erase_trigger when isp_prom_access='1' else
                          '0';
    prom_write_trigger <= reg_prom_write_trigger when reg_prom_access='1' else
                          isp_prom_write_trigger when isp_prom_access='1' else
                          '0';
    prom_read_trigger <= reg_prom_read_trigger when reg_prom_access='1' else
                         isp_prom_read_trigger when isp_prom_access='1' else
                         '0';
    prom_address <= reg_prom_address when reg_prom_access='1' else
                    isp_prom_address when isp_prom_access='1' else
                    (others => '0');
    prom_din_we <= reg_prom_din_we when reg_prom_access='1' else
                   isp_prom_din_we when isp_prom_access='1' else
                   '0';
    prom_din_offset <= reg_prom_din_offset when reg_prom_access='1' else
                       '0' & isp_prom_din_offset when isp_prom_access='1' else
                       (others => '0');
    prom_din <= reg_prom_din when reg_prom_access='1' else
                isp_prom_din when isp_prom_access='1' else
                (others => '0');
    prom_dout_offset <= reg_prom_dout_offset when reg_prom_access='1' else
                        '0' & isp_prom_dout_offset when isp_prom_access='1' else
                        (others => '0');

    
    -- --------------------------------------------------------------------------
    --                        Register Access
    -- --------------------------------------------------------------------------

    roRegisterHandler : process (clk)
    begin
        if clk'event and clk='1' then
            if reset='1' then
                register_memory.ro_registers <= (others => (others => '0'));
            else

                --set_ro_register(register_memory, FPGAJoint_SYNTHESIS_YEAR,  SYNTHESIS_YEAR);
                --set_ro_register(register_memory, FPGAJoint_SYNTHESIS_MONTH, SYNTHESIS_MONTH);
                --set_ro_register(register_memory, FPGAJoint_SYNTHESIS_DAY,   SYNTHESIS_DAY);
                --set_ro_register(register_memory, FPGAJoint_SYNTHESIS_AUTHOR,SYNTHESIS_AUTHOR);

                --set_ro_register(register_memory, BLDCJoint_STACK_VERSION, std_logic_vector(to_unsigned(STACK_VERSION,8)));

                ---- state register
                --set_ro_registerbit(register_memory,BLDCJoint_STATE,BLDCJoint_STATE_ENABLE_BIT,enable);

                --if ((edge_1_req = '0') and (edge_1_ack = '0')) then
                    --set_ro_register(register_memory,PicoBlazeArray_OUTPUT, sink(7 downto 0));
                    --set_ro_register(register_memory,PicoBlazeArray_OUTPUT_B, sinks(1));
                    --set_ro_register(register_memory,PicoBlazeArray_OUTPUT_C, sinks(2));
                    --set_ro_register(register_memory,PicoBlazeArray_OUTPUT_D, sinks(3));
                --end if;
                
            end if;
        end if;
    end process;

    register_access : entity work.register_access
        port map ( clk => clk,
                   rst => reset,

                   register_memory => register_memory,

                   busy       => reg_busy,
                   init_done  => promInitDone,
                   write_prom => writePromTrigger,

                   reg_read_id    => reg_read_id,
                   reg_read_type  => reg_read_type,
                   reg_read_data  => reg_read_data,
                   reg_read       => reg_read,
                   reg_read_ack   => reg_read_ack,
                   reg_write_id   => reg_write_id,
                   reg_write_type => reg_write_type,
                   reg_write_data => reg_write_data,
                   reg_write      => reg_write,

                   prom_request     => reg_prom_request,
                   prom_access      => reg_prom_access,
                   prom_read_trigger  => reg_prom_read_trigger,
                   prom_erase_trigger => reg_prom_erase_trigger,
                   prom_write_trigger => reg_prom_write_trigger,
                   prom_busy        => prom_busy,
                   prom_address     => reg_prom_address,
                   prom_din_offset  => reg_prom_din_offset,
                   prom_din         => reg_prom_din,
                   prom_din_we      => reg_prom_din_we,
                   prom_dout_offset => reg_prom_dout_offset,
                   prom_dout        => prom_dout );

    -- --------------------------------------------------------------------------
    --                     In-System Programming
    -- --------------------------------------------------------------------------

    isp : entity work.InSystemProgramming
        port map ( clk => clk,
                   rst => reset,

                   cmd_upload   => isp_cmd_upload,
                   cmd_data     => isp_cmd_data,
                   cmd_download => isp_cmd_download,
                   cmd_ack      => isp_cmd_ack,
                   din_addr     => isp_din_addr,
                   din_len      => isp_din_len,
                   din          => isp_din,
                   din_valid    => isp_din_valid,
                   din_ack      => isp_din_ack,
                   dout_addr    => isp_dout_addr,
                   dout         => isp_dout,
                   dout_valid   => isp_dout_valid,
                   dout_ack     => isp_dout_ack,

                   prom_request     => isp_prom_request,
                   prom_access      => isp_prom_access,
                   prom_read_trigger  => isp_prom_read_trigger,
                   prom_erase_trigger => isp_prom_erase_trigger,
                   prom_write_trigger => isp_prom_write_trigger,
                   prom_busy        => prom_busy,
                   prom_address     => isp_prom_address,
                   prom_din_offset  => isp_prom_din_offset,
                   prom_din         => isp_prom_din,
                   prom_din_we      => isp_prom_din_we,
                   prom_dout_offset => isp_prom_dout_offset,
                   prom_dout        => prom_dout );


end Behavioral;

