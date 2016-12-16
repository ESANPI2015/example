---------------------------------------------------------------------------------
--! @file    BLDCJointController_V3.vhd
--! @class   BLDCJointController_V3
--!
--! @brief   Controller for the BLDC motor.
--! @details Controller for the BLDC motor which is in the iStruct project
--!          the motor for the 'normal' joints (not the ankle joint).\n
--!          This module is written for the Spartan6 on the BLDC Controller
--!          Stack V3 and is based on the motCon_ctrl.vhd from the SpaceClimberJoint
--!          design written by Peter Kampmann and Sebastian Bartsch.\n\n
--!          German Research Center for Artificial Intelligence\n
--!          Project: iStruct\n
--!
--! @author Tobias Stark (tobias.stark@dfki.de)
--! @date   07.03.2013
---------------------------------------------------------------------------------
-- $LastChangedRevision: 4744 $
-- $LastChangedBy: tstark $
-- $LastChangedDate: 2016-10-07 11:09:31 +0200 (Fr, 07 Okt 2016) $
---------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.config.all;
use work.date.all;
use work.devices.all;
use work.deviceclasses.all;
use work.representations.all;

use work.dfki_pack.all;
use work.register_types.all;
use work.register_pack.all;

-- Changes needed for behavior graphs
use work.bg_vhdl_types.all;
use work.bg_graph_@name@_comm_config.all;

entity BLDCJointController_V3 is
    port( CLK_IN      : in  std_logic;                     --! system clock
          nRST_IN     : in  std_logic;                     --! active low system reset
          LED         : out std_logic_vector(1 downto 0);  --! led output
          
          -- pins for communication
          RX1         : in  std_logic; --! port1 RX input for communication
          TX1         : out std_logic; --! port1 TX output for communication
          RX2         : in  std_logic; --! port2 RX input for communication
          TX2         : out std_logic; --! port2 TX output for communication
          
          -- hall sensor inputs
          HALL1       : in  std_logic;  --! hall sensor input 1
          HALL2       : in  std_logic;  --! hall sensor input 2
          HALL3       : in  std_logic;  --! hall sensor input 3
          
          -- halfbridge outputs
          P1_HIN      : out std_logic;  --! halfbridge output
          P1_LIN      : out std_logic;  --! halfbridge output
          P2_HIN      : out std_logic;  --! halfbridge output
          P2_LIN      : out std_logic;  --! halfbridge output
          P3_HIN      : out std_logic;  --! halfbridge output
          P3_LIN      : out std_logic;  --! halfbridge output
          
          -- pins for current adc
          ADC_nCS     : out std_logic;  --! ADC chip select
          ADC_CLK     : out std_logic;  --! ADC clock output
          ADC_SDOA    : in  std_logic;  --! ADC input A
          ADC_SDOB    : in  std_logic;  --! ADC input B
          ADC_SDOC    : in  std_logic;  --! ADC input C
          
          -- pins for absolute angular encoder
          ICH_MA      : out std_logic;  --! clock output for ic-haus
          ICH_SLO     : in  std_logic;  --! data input from ic-haus

          -- pins for optical encoder
          ENC_A       : in  std_logic;  --! encoder input A
          ENC_B       : in  std_logic;  --! encoder input B
          
          -- pins for i2c temperature and power sensors
          i2c_con_scl   : inout std_logic;
          i2c_con_sda   : inout std_logic;
          i2c_power_scl : inout std_logic;
          i2c_power_sda : inout std_logic;

          -- pins for SPIPROM
          PROM_nCS    : out std_logic;
          PROM_CLK    : out std_logic;
          PROM_nWP    : out std_logic;
          PROM_nHOLD  : out std_logic;
          PROM_MISO   : in  std_logic;
          PROM_MOSI   : out std_logic );
end BLDCJointController_V3;

architecture Behavioral of BLDCJointController_V3 is

------ Clock and Reset signals --------------------------------------------------
    
    signal clk : std_logic;
    signal rst : std_logic;
    alias reset : std_logic is rst;

------ Internal signal declarations ---------------------------------------------

    signal ls_rx   : std_logic_vector(1 downto 0);
    signal ls_tx   : std_logic_vector(1 downto 0);

    signal NODE_ID   : std_logic_vector(7 downto 0);
    signal ENABLE    : std_logic;
    signal greenLed  : std_logic;
    signal redLed    : std_logic;

    -- control signals
    signal resetDevice : std_logic;
    signal stopDevice  : std_logic;
    signal enableCmd   : std_logic;
    signal enableInterpolator : std_logic;
    
------ Initialization and error handling ----------------------------------------

    signal initDone          : std_logic;
    signal register_InitDone : std_logic;
    signal position_InitDone : std_logic;
    signal error_InitDone    : std_logic;

    signal error_PositionDifference : std_logic;
    signal error_PosHardLimit       : std_logic;
    signal error_VelHardLimit       : std_logic;
    signal error_CurHardLimit       : std_logic;
    signal error_ComTimeoutAny      : std_logic;
    signal error_ComTimeoutCmd      : std_logic;
    signal error_OverTemp           : std_logic;
    signal error_AbsPosTimeout      : std_logic;
    signal error_HallSensor         : std_logic;

    signal errorFlag   : std_logic;
    signal warningFlag : std_logic;
    signal warnings    : std_logic_vector(31 downto 0);
    signal errors      : std_logic_vector(31 downto 0);

-------------- Register ---------------------------------------------------------

    -- data structure to store register
    signal register_memory : registermemory_t;
    
    signal reg_busy       : std_logic;
    signal reg_read_id    : std_logic_vector(15 downto 0);
    signal reg_read_type  : registertype_t;
    signal reg_read_data  : std_logic_vector(63 downto 0);
    signal reg_read       : std_logic;
    signal reg_read_ack   : std_logic;
    signal com_reg_write_id   : std_logic_vector(15 downto 0);
    signal com_reg_write_type : registertype_t;
    signal com_reg_write_data : std_logic_vector(63 downto 0);
    signal com_reg_write      : std_logic;
    signal err_reg_write_id   : std_logic_vector(15 downto 0);
    signal err_reg_write_type : registertype_t;
    signal err_reg_write_data : std_logic_vector(63 downto 0);
    signal err_reg_write      : std_logic;
    signal reg_write_id   : std_logic_vector(15 downto 0);
    signal reg_write_type : registertype_t;
    signal reg_write_data : std_logic_vector(63 downto 0);
    signal reg_write      : std_logic;
    
------ Communication ------------------------------------------------------------

    -- data structure to store telemetry data to be send
    signal telemetry_values : TelemetryValues;
    signal msg_recv         : std_logic;
    signal commErr          : std_logic;
    signal ndlc_clear_stat  : std_logic;
    signal ndlc_last_error  : std_logic_vector(15 downto 0);
    signal ndlc_recvEvents  : std_logic_vector(31 downto 0);
    signal ndlc_missEvents  : std_logic_vector(31 downto 0);

    signal desiredPosition  : signed(15 downto 0);
    signal directPWM        : signed(15 downto 0);  -- [-32767,32767]

------ Real Time Clock ----------------------------------------------------------

    signal actualTime : std_logic_vector(63 downto 0);
    signal setNewTime : std_logic;
    signal newTime    : std_logic_vector(63 downto 0);

------ Absolute Position Sensor (IC-Haus) ---------------------------------------

    signal abs_pos_raw      : std_logic_vector(11 downto 0);
    signal abs_pos_new_data : std_logic;
    signal abs_pos_valid    : std_logic;
    signal abs_pos_offset   : std_logic_vector(15 downto 0);
    signal absolutePosition : signed(15 downto 0);
    
------ Position and Speed measurement -------------------------------------------

    signal enc_rst_position     : std_logic_vector(ENC_POS_WIDTH-1 downto 0);
    signal enc_rst              : std_logic;
    signal actualPosition_raw   : std_logic_vector(ENC_POS_WIDTH-1 downto 0);
    signal actualPosition       : signed(16 downto 0);

    signal actualSpeed_raw      : std_logic_vector(ENC_SPEED_WIDTH-1 downto 0);
    signal actualSpeed_filtered : std_logic_vector(ENC_SPEED_WIDTH-1 downto 0);
    signal actualSpeed          : signed(31 downto 0);

--------------------- Joint Space Interpolator  ---------------------------------

    constant SETPOINT_UPDATE_FREQ : integer := 74;
    signal setpoint_to_controller : std_logic_vector(15 downto 0);
    
----------- RateLimiter for Position SetPoint -----------------------------------
    
    signal maxSetpointRate          : std_logic_vector(15 downto 0);
    signal unlimitedDesiredPosition : std_logic_vector(15 downto 0); -- input to RateLimiter
    signal limitedDesiredPosition   : std_logic_vector(15 downto 0); -- output of RateLimiter
    
------ POS/SPD Controller using PID_FixPoint_2DOF -------------------------------
    
    constant POSCTRL_FREQ    : integer := 1000;
    constant PARA_LENGTH     : integer := 17;
    constant FRACTION_LENGTH : integer := 10;
    constant ABS_PWM_LENGTH  : integer := 10;
    signal K_fixpoint  : std_logic_vector(PARA_LENGTH-1 downto 0);
    signal Ki_fixpoint : std_logic_vector(PARA_LENGTH-1 downto 0);
    signal Kd_fixpoint : std_logic_vector(PARA_LENGTH-1 downto 0);
    signal b_fixpoint  : std_logic_vector(FRACTION_LENGTH downto 0);
    signal c_fixpoint  : std_logic_vector(FRACTION_LENGTH downto 0);
    
    signal PosSpdCtrlOut : std_logic_vector(10 downto 0);
    signal PosSpdCtrlOut_limited : std_logic_vector(10 downto 0);
    
--------------------------- Notch Filter ----------------------------------------
    
    signal bl_trig         : std_logic;
    signal bl_enable_notch : std_logic := '1'; 
    signal FilteredPWM     : std_logic_vector(10 downto 0);
    signal NotchFilterCLK  : std_logic;

--------------------------- PWM signals -----------------------------------------
    
    signal ctrlPWM          : signed(15 downto 0);  -- [-32767,32767]
    signal actualDir_raw    : std_logic;
    signal actualPWM_raw    : std_logic_vector(9 downto 0);
    signal actualPWM        : signed(15 downto 0);  -- [-32767,32767]

-------------- Motor Sensors and Driver -----------------------------------------

    signal hall1_filtered : std_logic;
    signal hall2_filtered : std_logic;
    signal hall3_filtered : std_logic;

    signal motorDirection : std_logic;
    signal motorAmplitude : std_logic_vector(9 downto 0);

-------------- Current Measurement ----------------------------------------------

    type phaseCurrent_t is array(2 downto 0) of std_logic_vector(11 downto 0);
    signal phaseCurrent : phaseCurrent_t;

    signal newCurrentValues       : std_logic;
    signal actualCurrent_raw      : std_logic_vector(15 downto 0);
    signal actualCurrent_filtered : std_logic_vector(15 downto 0);
    signal actualCurrent_slv      : std_logic_vector(15 downto 0);
    signal actualCurrent          : signed(15 downto 0);  -- resolution 1mA

-------------- Temperature and voltage sensors ----------------------------------

    signal temp0_raw   : std_logic_vector(11 downto 0);
    signal temp1_raw   : std_logic_vector(11 downto 0);
    signal temp0       : signed(15 downto 0);  -- 9.7 notation
    signal temp1       : signed(15 downto 0);  -- 9.7 notation
    signal temp_max    : signed(15 downto 0);  -- 9.7 notation
    signal voltage_raw : std_logic_vector(15 downto 0);
    signal voltage     : std_logic_vector(15 downto 0);  -- resolution 1mV

-------------- EEPROM Interface -------------------------------------------------

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

    ENABLE <= enableCmd and not errorFlag;


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

    -- ----------------------------------------------
    --               Clock and Reset
    -- ----------------------------------------------

    clk_reset_inst : entity work.basic_clk_reset_mod
        port map ( clk20MHz_in => CLK_IN,
                   nreset_in   => nRST_IN,
                   clk_out     => clk,
                   reset_out   => rst );

    
    -- ----------------------------------------------
    --            Device reconfiguration
    -- ----------------------------------------------

    device_reconf : entity work.device_reconf_spartan6
        port map ( clk => clk,
                   rst => rst,
                   reconf_trigger => resetDevice );

    
    -- ----------------------------------------------
    --       Error handling and LED managment
    -- ----------------------------------------------

    -- LEDs: 0 - green, 1 - red
    LED(0) <= greenLed;
    LED(1) <= redLed or prom_busy;

    blinky : process (clk)
        variable counter : integer range 0 to CLK_FREQ/8;
        variable led_int : std_logic_vector(2 downto 0);
        variable enable_blink : std_logic_vector(1 downto 0);
        variable status : std_logic_vector(2 downto 0);
    begin
        if clk'event and clk='1' then
            if rst='1' then
                counter := 0;
                led_int := "000";
                enable_blink := "00";
                greenLed  <= '0';
                redLed    <= '0';
            else
                -- default
                enable_blink := "00";
                greenLed  <= '0';
                redLed    <= '0';
                
                if counter < CLK_FREQ/8 then
                    counter := counter + 1;
                else
                    counter := 0;
                    led_int := incr_f(led_int);
                end if;

                status := enableCmd & warningFlag & errorFlag;

                case status is
                    when "100" =>
                        -- enable
                        greenLed  <= '1';
                    when "110" =>
                        -- enable & warn
                        greenLed  <= '1';
                        redLed    <= '1';
                    when "101" | "111" =>
                        -- enable & err
                        greenLed  <= led_int(0);
                        redLed    <= not led_int(0);
                    when "000" =>
                        -- idle
                        greenLed  <= led_int(2);
                    when "010" =>
                        -- idle & warn
                        greenLed  <= led_int(2);
                        redLed    <= led_int(2);
                    when others =>
                        -- idle & err
                        redLed    <= not led_int(0);
                end case;
                
            end if;
        end if;
    end process;

    initializationProcess : process (clk)
        variable counter : integer range 0 to 1023;
    begin
        if clk'event and clk='1' then
            if rst='1' then
                counter := 0;
                
                error_InitDone     <= '0';
                err_reg_write_id   <= (others => '0');
                err_reg_write_type <= error;
                err_reg_write_data <= (others => '0');
                err_reg_write      <= '0';

                initDone <= '0';
            else
                -- defaults
                err_reg_write <= '0';

                -- error register initialization
                if error_InitDone='0' and register_InitDone='1' and reg_busy='0' and com_reg_write='0' then
                    if counter=0 then
                        counter := 1;
                        err_reg_write_id   <= std_logic_vector(to_unsigned(BLDCJoint_ERROR_IGNORE,16));
                        err_reg_write_type <= REGISTER_TYPES(BLDCJoint_ERROR_IGNORE);
                        err_reg_write_data(31 downto 0) <= get_register(register_memory,BLDCJoint_INIT_ERROR_IGNORE);
                        err_reg_write      <= '1';
                    else
                        counter := 0;
                        err_reg_write_id   <= std_logic_vector(to_unsigned(BLDCJoint_ERROR_DISABLE,16));
                        err_reg_write_type <= REGISTER_TYPES(BLDCJoint_ERROR_DISABLE);
                        err_reg_write_data(31 downto 0) <= get_register(register_memory,BLDCJoint_INIT_ERROR_DISABLE);
                        err_reg_write      <= '1';
                        error_InitDone     <= '1';
                    end if;
                end if;

                -- delay initDone signal a little bit to be shure stuff
                -- without init signals are initialized, too
                if register_InitDone='1' and position_InitDone='1' and error_InitDone='1' then
                    if counter<1023 then
                        counter := counter + 1;
                    else
                        initDone <= '1';
                    end if;
                end if;

            end if;
        end if;
    end process;

    errorFlag   <= '0' when errors   = x"00000000" and error_InitDone='1' else '1';
    warningFlag <= '0' when warnings = x"00000000" and error_InitDone='1' else '1';

    errorHandler : process (clk)
        variable errorflags  : std_logic_vector(31 downto 0);
        variable oldInitDone : std_logic;
    begin
        if clk'event and clk='1' then
            if rst='1' then
                errorflags  := (others => '0');
                oldInitDone := '0';
                ndlc_clear_stat <= '0';
                
                warnings    <= (others => '0');
                errors      <= (others => '0');
            else
                ndlc_clear_stat <= '0';

                -- actual error handling
                errorflags(BLDCJoint_ERROR_INIT_BIT)               := not initDone;
                errorflags(BLDCJoint_ERROR_POSITIONDIFFERENCE_BIT) := error_PositionDifference;
                errorflags(BLDCJoint_ERROR_POS_HARD_LIMIT_BIT)     := error_PosHardLimit;
                errorflags(BLDCJoint_ERROR_VEL_HARD_LIMIT_BIT)     := error_VelHardLimit;
                errorflags(BLDCJoint_ERROR_CUR_HARD_LIMIT_BIT)     := error_CurHardLimit;
                errorflags(BLDCJoint_ERROR_COMMTIMEOUTANY_BIT)     := error_ComTimeoutAny;
                errorflags(BLDCJoint_ERROR_COMMTIMEOUTARMCMD_BIT)  := error_ComTimeoutCmd;
                errorflags(BLDCJoint_ERROR_TEMPERATURE_BIT)        := error_OverTemp;
                errorflags(BLDCJoint_ERROR_SENSOR_ABS_POS_BIT)     := error_AbsPosTimeout or not abs_pos_valid;
                errorflags(BLDCJoint_ERROR_SENSOR_REL_POS_BIT)     := '0';
                errorflags(BLDCJoint_ERROR_SENSOR_HALL_BIT)        := error_HallSensor;
                errorflags(BLDCJoint_ERROR_SENSOR_CURRENT_BIT)     := '0';
                errorflags(BLDCJoint_ERROR_VOLTAGE_BIT)            := '0';
                errorflags(BLDCJoint_ERROR_CTRL_LIMIT_BIT)         := '0';
                errorflags(BLDCJoint_ERROR_COMM_ERROR_BIT)         := commErr;

                if (initDone='1' and oldInitDone='0') or
                    get_register(register_memory,BLDCJoint_CONFIG)(BLDCJoint_CONFIG_RESET_ERRORS_BIT)='1'then
                    -- when initialization is done or on reset errors - only handle actual errors
                    errors <= errorflags and not get_register(register_memory,BLDCJoint_ERROR_DISABLE)
                              and not get_register(register_memory,BLDCJoint_ERROR_IGNORE);
                    ndlc_clear_stat <= '1';
                else
                    -- real error handling (with the need of resetting errors)
                    errors <= errors or (errorflags and not get_register(register_memory,BLDCJoint_ERROR_DISABLE)
                                         and not get_register(register_memory,BLDCJoint_ERROR_IGNORE));
                end if;
                
                warnings <= errorflags and not get_register(register_memory,BLDCJoint_ERROR_DISABLE)
                            and get_register(register_memory,BLDCJoint_ERROR_IGNORE);

                -- save old init error state to detect init done
                oldInitDone := initDone;

            end if;
        end if;
    end process;

    TimeoutWatcher : process (clk)
        constant TIMEOUT_COUNT_MAX : natural := natural(real(CLK_FREQ) * TIMEOUT_SECS);
        variable counter : natural range 0 to TIMEOUT_COUNT_MAX;
        variable com_flag    : std_logic;
        variable absPos_flag : std_logic;
    begin
        if clk'event and clk='1' then
            if rst='1' then
                counter := 0;
                com_flag    := '0';
                absPos_flag := '0';
                error_ComTimeoutAny <= '0';
                error_ComTimeoutCmd <= '0';
                error_AbsPosTimeout <= '0';
            else
                if counter = TIMEOUT_COUNT_MAX then
                    -- check for communication timeout
                    if com_flag='0' then
                        error_ComTimeoutAny <= '1';
                        if ENABLE='1' then
                            error_ComTimeoutCmd <= '1';
                        end if;
                    else
                        error_ComTimeoutAny <= '0';
                        error_ComTimeoutCmd <= '0';
                    end if;

                    -- check for absolute position timeout
                    if absPos_flag='0' then
                        error_AbsPosTimeout <= '1';
                    else
                        error_AbsPosTimeout <= '0';
                    end if;

                    -- reset flags
                    com_flag    := '0';
                    absPos_flag := '0';
                end if;

                -- check if an event occured to
                -- set the corresponding flag
                if msg_recv='1' then
                    com_flag := '1';
                end if;
                if abs_pos_new_data='1' then
                    absPos_flag := '1';
                end if;
                counter := counter + 1;

            end if;
        end if;
    end process;


    -- --------------------------------------------------------------------------
    --                           Communication
    -- --------------------------------------------------------------------------

    ls_rx(0) <= RX1;
    ls_rx(1) <= RX2;
    TX1 <= ls_tx(0);
    TX2 <= ls_tx(1);
    
    NDLCom_wrapper : entity work.@name@_NDLCom_wrapper
        generic map ( CLK_FREQ       => CLK_FREQ,
                      LS_BAUD_RATE   => LS_BAUD_RATE )
        port map ( CLK     => clk,
                   RST     => rst,
                   NODE_ID => node_id,
                   
                   ls_rx   => ls_rx,
                   ls_tx   => ls_tx,

                   resetDevice => resetDevice,
                   stopDevice  => stopDevice,
                   
                   telemetry_values      => telemetry_values,
                   telemetry_receiver    => get_register(register_memory, BLDCJoint_TELEMETRY_RECEIVER_ID),
                   telemetry_send_period => get_register(register_memory, BLDCJoint_TELEMETRY_SEND_PERIOD),

				   bg_input_portId  => bg_input_portid,
                   bg_input_data    => bg_input_data,
                   bg_input_req     => bg_input_req,
                   bg_input_ack     => bg_input_ack,
                   bg_output_recvId => bg_output_recvId,
                   bg_output_portId => bg_output_portid,
                   bg_output_data   => bg_output_data,
                   bg_output_req    => bg_output_req,
                   bg_output_ack    => bg_output_ack,
						 
                   reg_busy         => reg_busy,
                   reg_read_id      => reg_read_id,
                   reg_read_type    => reg_read_type,
                   reg_read_data    => reg_read_data,
                   reg_read         => reg_read,
                   reg_read_ack     => reg_read_ack,
                   reg_write_id     => com_reg_write_id,
                   reg_write_type   => com_reg_write_type,
                   reg_write_data   => com_reg_write_data,
                   reg_write        => com_reg_write,
                   
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
                   isp_dout_ack     => isp_dout_ack,

                   rtc_actual_time  => actualTime,
                   rtc_set_time     => setNewTime,
                   rtc_new_time     => newTime,

                   msg_recv => msg_recv,
                   commErr  => commErr,

                   ndlc_clear_stat => ndlc_clear_stat,
                   ndlc_last_error => ndlc_last_error,
                   ndlc_recvEvents => ndlc_recvEvents,
                   ndlc_missEvents => ndlc_missEvents );

    -- this process sets the control_register which are received by commod
    -- and set also the telemetry_values wich will be send via commod
    setComValues : process (clk)
        variable oldStopDevice      : std_logic;
        variable lastPosition       : signed(16 downto 0);
        variable upperSoftLimit     : signed(15 downto 0);
        variable lowerSoftLimit     : signed(15 downto 0);
        variable tmpDesiredPosition : signed(15 downto 0);
    begin
        if clk'event and clk = '1' then
            if rst = '1' then
                NODE_ID <= (others => '0');
                
                enableCmd          <= '0';
                enableInterpolator <= '0';

                oldStopDevice      := '0';
                lastPosition       := (others => '0');
                upperSoftLimit     := (others => '0');
                lowerSoftLimit     := (others => '0');
                tmpDesiredPosition := (others => '0');
                desiredPosition    <= (others => '0');

                maxSetpointRate    <= (others => '0');
                directPWM          <= (others => '0');

                K_fixpoint  <= (others => '0');
                Ki_fixpoint <= (others => '0');
                Kd_fixpoint <= (others => '0');
                b_fixpoint  <= (others => '0');
                c_fixpoint  <= (others => '0');

                telemetry_values <= (others => (others => '0'));

            else
                
                -- ---------------------------------------------- --
                -- --------------- Command Values --------------- --
                -- ---------------------------------------------- --

                NODE_ID <= get_register(register_memory, NDLCom_NODE_ID);
                
                -- Configuration Register --
                -- ---------------------- --
                enableCmd             <= get_register(register_memory, BLDCJoint_CONFIG)(BLDCJoint_CONFIG_ENABLE_CMD_BIT);
                enableInterpolator    <= get_register(register_memory, BLDCJoint_CONFIG)(BLDCJoint_CONFIG_INTERPOLATOR_BIT);
                
                -- Position Register --
                -- ----------------- --

                -- check stop device flag
                if stopDevice='1' and oldStopDevice='0' then
                    -- rising edge of stopDevice -> save actualPosition
                    -- to use it as desiredPosition as long as stopDevice
                    -- is active (joint holds its position)
                    lastPosition := actualPosition;
                end if;
                oldStopDevice := stopDevice;

                if ENABLE='0' then
                    -- if motor is disabled take the actual position as desired
                    desiredPosition <= actualPosition(15 downto 0);
                elsif stopDevice='1' then
                    -- if the device should stop hold the last requested position
                    desiredPosition <= lastPosition(15 downto 0);
                else
                    tmpDesiredPosition := signed(get_register(register_memory, CascadedController_POS_REF));

                    upperSoftLimit := signed(get_register(register_memory,CascadedController_POS_MAX));
                    lowerSoftLimit := signed(get_register(register_memory,CascadedController_POS_MIN));

                    -- check for a soft limit error
                    if tmpDesiredPosition > upperSoftLimit then
                        desiredPosition <= upperSoftLimit;
                    elsif tmpDesiredPosition < lowerSoftLimit then
                        desiredPosition <= lowerSoftLimit;
                    else
                        desiredPosition <= tmpDesiredPosition;
                    end if;
                end if;

                -- Speed Register --
                -- -------------- --
                maxSetpointRate <= get_register(register_memory, CascadedController_VEL_MAX)(15 downto 0);
                
                -- Direct PWM --
                -- ---------- --
                if stopDevice='0' then
                    directPWM <= signed(get_register(register_memory, BLDCJoint_DIRECT_PWM));
                else
                    directPWM <= (others => '0');
                end if;

                -- POS/SPD Controller Gains --
                -- ---------------------- --
                --K  := get_register(register_memory,BLDCJoint_POSSPD_PGAIN_2) & get_register(register_memory,BLDCJoint_POSSPD_PGAIN_1) & get_register(register_memory,BLDCJoint_POSSPD_PGAIN_0);
                --Ki := get_register(register_memory,BLDCJoint_POSSPD_IGAIN_2) & get_register(register_memory,BLDCJoint_POSSPD_IGAIN_1) & get_register(register_memory,BLDCJoint_POSSPD_IGAIN_0);
                --Kd := get_register(register_memory,BLDCJoint_POSSPD_DGAIN_2) & get_register(register_memory,BLDCJoint_POSSPD_DGAIN_1) & get_register(register_memory,BLDCJoint_POSSPD_DGAIN_0);
                --b  := get_register(register_memory,BLDCJoint_POSSPD_BGAIN_2) & get_register(register_memory,BLDCJoint_POSSPD_BGAIN_1) & get_register(register_memory,BLDCJoint_POSSPD_BGAIN_0);
                --c  := get_register(register_memory,BLDCJoint_POSSPD_CGAIN_2) & get_register(register_memory,BLDCJoint_POSSPD_CGAIN_1) & get_register(register_memory,BLDCJoint_POSSPD_CGAIN_0);
                
                --K_fixpoint  <= K(PARA_LENGTH-1 downto 0);
                --Ki_fixpoint <= Ki(PARA_LENGTH-1 downto 0);
                --Kd_fixpoint <= Kd(PARA_LENGTH-1 downto 0);
                --b_fixpoint  <= b(FRACTION_LENGTH downto 0);
                --c_fixpoint  <= c(FRACTION_LENGTH downto 0);
                K_fixpoint  <= get_register(register_memory, CascadedController_POS_P)(16 downto 0);
                Ki_fixpoint <= get_register(register_memory, CascadedController_POS_I)(16 downto 0);
                Kd_fixpoint <= get_register(register_memory, CascadedController_POS_D)(16 downto 0);
                b_fixpoint  <= "100" & x"00";
                c_fixpoint  <= "100" & x"00";
                
                -- ------------------------------------------------ --
                -- --------------- Telemetry Values --------------- --
                -- ------------------------------------------------ --

                -- State Flags --
                telemetry_values(0) <= get_register(register_memory, BLDCJoint_STATE);

                -- Absolute Position --
                telemetry_values(1) <= std_logic_vector(absolutePosition);
                
                -- Position --
                telemetry_values(2) <= std_logic_vector(actualPosition(15 downto 0));

                -- Speed --
                telemetry_values(3) <= std_logic_vector(actualSpeed(15 downto 0));
                telemetry_values(4) <= std_logic_vector(actualSpeed(31 downto 16));

                -- PWM --
                telemetry_values(5) <= std_logic_vector(actualPWM);
                
                -- Voltage --
                telemetry_values(6) <= voltage;
                
                -- Current --
                telemetry_values(7) <= std_logic_vector(actualCurrent);

                -- Temperature --
                telemetry_values(8) <= std_logic_vector(temp_max);

                -- Desired Position --
                telemetry_values(9) <= std_logic_vector(desiredPosition);

                -- control PWM --
                telemetry_values(10) <= std_logic_vector(ctrlPWM);

                -- Debug --
                telemetry_values(11) <= unlimitedDesiredPosition;
                telemetry_values(12) <= limitedDesiredPosition;
                telemetry_values(13) <= std_logic_vector(shift_left(resize(signed(PosSpdCtrlOut),16),5));
                telemetry_values(14) <= std_logic_vector(shift_left(resize(signed(PosSpdCtrlOut_limited),16),5));
                telemetry_values(15) <= std_logic_vector(shift_left(resize(signed(FilteredPWM),16),5));

            end if;
        end if;
    end process;

    roRegisterHandler : process (clk)
    begin
        if clk'event and clk='1' then
            if rst='1' then
                register_memory.ro_registers <= (others => (others => '0'));
            else

                set_ro_register(register_memory, FPGAJoint_SYNTHESIS_YEAR,  SYNTHESIS_YEAR);
                set_ro_register(register_memory, FPGAJoint_SYNTHESIS_MONTH, SYNTHESIS_MONTH);
                set_ro_register(register_memory, FPGAJoint_SYNTHESIS_DAY,   SYNTHESIS_DAY);
                set_ro_register(register_memory, FPGAJoint_SYNTHESIS_AUTHOR,SYNTHESIS_AUTHOR);

                set_ro_register(register_memory, BLDCJoint_STACK_VERSION, to_unsigned(STACK_VERSION,8));

                -- Error Registers --
                set_ro_register(register_memory, BLDCJoint_ERROR,   errors);
                set_ro_register(register_memory, BLDCJoint_WARNING, warnings);

                -- Telemetry Registers --
                -- ------------------- --
                -- state register
                set_ro_registerbit(register_memory,BLDCJoint_STATE,BLDCJoint_STATE_ENABLE_BIT,      enable);
                set_ro_registerbit(register_memory,BLDCJoint_STATE,BLDCJoint_STATE_WARNINGFLAG_BIT, warningFlag);
                set_ro_registerbit(register_memory,BLDCJoint_STATE,BLDCJoint_STATE_ERRORFLAG_BIT,   errorFlag);
                set_ro_registerbit(register_memory,BLDCJoint_STATE,BLDCJoint_STATE_ENABLE_CMD_BIT,  enableCmd);
                set_ro_registerbit(register_memory,BLDCJoint_STATE,BLDCJoint_STATE_PROM_BUSY_BIT,   prom_busy);
                set_ro_registerbit(register_memory,BLDCJoint_STATE,BLDCJoint_STATE_HALL0_BIT, hall1_filtered);
                set_ro_registerbit(register_memory,BLDCJoint_STATE,BLDCJoint_STATE_HALL1_BIT, hall2_filtered);
                set_ro_registerbit(register_memory,BLDCJoint_STATE,BLDCJoint_STATE_HALL2_BIT, hall3_filtered);
                -- set this bit to '1' to show the mcs that telemetry data has been arived
                set_ro_registerbit(register_memory,BLDCJoint_STATE,BLDCJoint_STATE_TELEMETRY_AVAILABLE_BIT,'1');

                -- other telemetry
                set_ro_register(register_memory, BLDCJoint_ABSOLUTE_POSITION, absolutePosition);
                set_ro_register(register_memory, BLDCJoint_POSITION,          actualPosition(15 downto 0));
                set_ro_register(register_memory, BLDCJoint_SPEED,             actualSpeed);
                set_ro_register(register_memory, BLDCJoint_PWM,               actualPWM);
                set_ro_register(register_memory, BLDCJoint_VOLTAGE,           voltage);
                set_ro_register(register_memory, BLDCJoint_CURRENT,           actualCurrent);
                set_ro_register(register_memory, BLDCJoint_TEMPERATURE,       temp_max);
                set_ro_register(register_memory, BLDCJoint_DESIRED_POSITION,  desiredPosition);
                set_ro_register(register_memory, BLDCJoint_CTRL_PWM,          ctrlPWM);
                
                -- Temperature Registers --
                set_ro_register(register_memory, BLDCJoint_TEMPERATURE0, temp0);
                set_ro_register(register_memory, BLDCJoint_TEMPERATURE1, temp1);

                set_ro_register(register_memory, BLDCJoint_RECV_EVENTS, ndlc_recvEvents);
                set_ro_register(register_memory, BLDCJoint_MISS_EVENTS, ndlc_missEvents);

                -- Debug --
                set_ro_register(register_memory, BLDCJoint_DEBUG_RO_1, ndlc_last_error);
                --set_ro_register(register_memory, BLDCJoint_DEBUG_RO_2, x"0000");
                --set_ro_register(register_memory, BLDCJoint_DEBUG_RO_3, x"0000");
                --set_ro_register(register_memory, BLDCJoint_DEBUG_RO_4, x"0000");
                --set_ro_register(register_memory, BLDCJoint_DEBUG_RO_5, x"0000");

            end if;
        end if;
    end process;


    -- --------------------------------------------------------------------------
    --                         Real Time Clock
    -- --------------------------------------------------------------------------
    
    rtc_inst : entity work.rtc_mod
        generic map ( SYSTEM_speed => CLK_FREQ )
        port map ( CLK => clk,
                   RST => rst,
                   actualTime => actualTime,
                   setNewTime => setNewTime,
                   newTime    => newTime );


    -- --------------------------------------------------------------------------
    --                    Absolute Position Sensor
    -- --------------------------------------------------------------------------
    
    absolutePositionSensor : entity work.ichaus_mh_mod
        generic map ( CLK_FREQ => CLK_FREQ )
        port map ( CLK            => clk,
                   RST            => rst,
                   MA             => ICH_MA,
                   SLO            => ICH_SLO,
                   config_gcc     => get_register(register_memory,BLDCJoint_ICHAUS_GCC)(6 downto 0),
                   config_voss    => get_register(register_memory,BLDCJoint_ICHAUS_VOSS)(6 downto 0),
                   config_vosc    => get_register(register_memory,BLDCJoint_ICHAUS_VOSC)(6 downto 0),
                   position       => abs_pos_raw,
                   new_position   => abs_pos_new_data,
                   position_valid => abs_pos_valid );

    abs_pos_offset <= get_register(register_memory,BLDCJoint_ABS_POS_OFFSET);
    absolutePosition <= shift_left(resize(signed(abs_pos_raw),16),4) - signed(abs_pos_offset) when
                        get_register(register_memory,BLDCJoint_INVERT_CONFIG)(BLDCJoint_INVERT_CONFIG_ABS_POS_BIT)='0' else
                        -(shift_left(resize(signed(abs_pos_raw),16),4) - signed(abs_pos_offset));

    position_initialization : entity work.pos_init_config
        generic map ( POS_WIDTH_OUT => ENC_POS_WIDTH )
        port map (
            clk            => clk,
            rst            => rst,
            ticks_per_turn => get_register(register_memory, BLDCJoint_TICKS_PER_TURN),
            position_in    => std_logic_vector(absolutePosition),
            new_input_val  => abs_pos_new_data and register_InitDone,
            position_out   => enc_rst_position,
            rst_trigger    => enc_rst,
            output_valid   => position_InitDone );


    -- --------------------------------------------------------------------------
    --                    Position and Speed Calculation
    -- --------------------------------------------------------------------------
    
    pos_speed_sensor : entity work.pos_speed_incr_measurement_config
        generic map (
            CLK_FREQ       => CLK_FREQ,
            POSITION_WIDTH => ENC_POS_WIDTH,
            SPEED_WIDTH    => ENC_SPEED_WIDTH,
            USE_OVERFLOW   => true )
        port map (
            CLK          => clk,
            RST          => enc_rst,
            RST_position => enc_rst_position,
            ENC_A        => ENC_A,
            ENC_B        => ENC_B,
            ticks_per_turn => std_logic_vector(shift_left(unsigned(get_register(register_memory, BLDCJoint_TICKS_PER_TURN)),1)),  -- 2mal ticks per turn -> 720deg
            invert_direction => get_register(register_memory, BLDCJoint_INVERT_CONFIG)(BLDCJoint_INVERT_CONFIG_POS_BIT),
            direction    => open,
            position     => actualPosition_raw,
            speed        => actualSpeed_raw
            );

    speed_filter : entity work.lowpass
        generic map ( CLK_FREQ    => CLK_FREQ,    -- clk frequency (Hz)
                      FIXED_FREQ  => true,
                      FILTER_FREQ => 1_000_000,   -- filter frequency (Hz)
                      WIDTH       => ENC_SPEED_WIDTH )
        port map ( clk            => clk,
                   rst            => rst,
                   damping        => get_register(register_memory, BLDCJoint_VEL_FILTER_DAMPING)(4 downto 0),
                   trigger        => '0',
                   new_value      => actualSpeed_raw,
                   output         => actualSpeed_filtered );

    pos_speed_conv : entity work.pos_speed_converter_config
        generic map (
            POS_WIDTH_IN    => ENC_POS_WIDTH,
            SPEED_WIDTH_IN  => ENC_SPEED_WIDTH,
            POS_WIDTH_OUT   => 17,
            SPEED_WIDTH_OUT => 32,
            SPEED_SHIFT     => 8 )
        port map (
            CLK          => clk,
            RST          => rst,
            ticks_per_turn => get_register(register_memory, BLDCJoint_TICKS_PER_TURN),
            position_in  => signed(actualPosition_raw),
            speed_in     => signed(actualSpeed_filtered),
            position_out => actualPosition,
            speed_out    => actualSpeed );

    positionWatcher : process (clk)
    begin
        if clk'event and clk='1' then
            if rst='1' then
                error_PositionDifference <= '0';
            else
                -- defaults
                error_PositionDifference <= '0';
                
                -- if the difference between ichaus position and encoder
                -- position is > MAX_POS_DIFF set the positionError flag
                if unsigned(abs(absolutePosition-actualPosition(15 downto 0))) > unsigned(get_register(register_memory,BLDCJoint_MAX_POS_DIFF)) then
                    error_PositionDifference <= '1';
                end if;
            end if;
        end if;
    end process;

    
    -- --------------------------------------------------------------------------
    --                          Safety Monitor
    -- --------------------------------------------------------------------------
    
    -- process that checks the motion range of the motors
    -- if they are in a valid range
    safetyMonitor : process (clk)
    begin
        if clk'event and clk='1' then
            if rst='1' then
                -- initialize all errors with '1'
                error_PosHardLimit <= '1';
                error_VelHardLimit <= '1';
                error_CurHardLimit <= '1';
            else
                -- position (with absolute sensor)
                if absolutePosition > signed(get_register(register_memory,BLDCJoint_POS_MIN_HARD_LIMIT)) and
                    absolutePosition < signed(get_register(register_memory,BLDCJoint_POS_MAX_HARD_LIMIT)) then
                    error_PosHardLimit <= '0';
                else
                    error_PosHardLimit <= '1';
                end if;

                -- velocity
                if unsigned(abs(actualSpeed)) < unsigned(get_register(register_memory,BLDCJoint_VEL_HARD_LIMIT)) then
                    error_VelHardLimit <= '0';
                else
                    error_VelHardLimit <= '1';
                end if;

                -- current
                if unsigned(abs(actualCurrent)) < unsigned(get_register(register_memory,BLDCJoint_CUR_HARD_LIMIT)) then
                    error_CurHardLimit <= '0';
                else
                    error_CurHardLimit <= '1';
                end if;
            end if;
        end if;
    end process;


    -- --------------------------------------------------------------------------
    --             Joint Space Interpolator
    -- --------------------------------------------------------------------------
    
    PosSetPointInterpolator: entity work.JointSpaceInterpolator
        generic map(
            CLK_FREQ      => CLK_FREQ,
            CTRL_FREQ     => POSCTRL_FREQ,
            SETPOINT_FREQ => SETPOINT_UPDATE_FREQ,
            DATA_WIDTH    => 16)
        port map(  clk => clk,
                   rst => reset,
                   data_in => std_logic_vector(desiredPosition),
                   data_out=> setpoint_to_controller);

    
    -- --------------------------------------------------------------------------
    --               Rate Limiter for Position Change as Speed Limiter
    -- --------------------------------------------------------------------------
    
    EnableInterpolatorProc : process(clk)
        -- PURPOSE: switching incoming signals to RateLimiter;
    begin
        if clk'event and clk = '1' then
            if reset = '1' then
                unlimitedDesiredPosition <= std_logic_vector(actualPosition(15 downto 0));
            else
                if enableInterpolator = '1' then -- ENABLE TRAJECTORY INTERPOLATOR
                    unlimitedDesiredPosition <= setpoint_to_controller;
                else
                    unlimitedDesiredPosition <= std_logic_vector(desiredPosition);
                end if;
            end if;
        end if;
    end process;

    RateLimiter : entity work.SetPointRateLimiter
        generic map(
            MAX_POSITION   => 32767,   
            MIN_POSITION   => -32768, 
            INPUT_WIDTH    => 16
            )
        port map( clk => clk,
                  rst => reset,
                  unlimitedSetPoint => unlimitedDesiredPosition,
                  isValue           => std_logic_vector(actualPosition(15 downto 0)),
                  maxRate           => maxSetpointRate,
                  limitedSetPoint   => limitedDesiredPosition
                  );

    
    -- --------------------------------------------------------------------------
    --                    Combined POSITION AND SPEED CONTROLLER
    -- --------------------------------------------------------------------------
    
    PosSpdCtrl : entity work.PID_FixPoint_2DOF
        generic map (
            CLK_FREQ                => CLK_FREQ,
            CTRL_FREQ               => POSCTRL_FREQ,
            INPUT_WIDTH             => 16,
            OUTPUT_WIDTH            => ABS_PWM_LENGTH+1, -- PWM + 1bit sign
            FRACTION_WIDTH_FIXPOINT => FRACTION_LENGTH,
            PARA_WIDTH              => PARA_LENGTH,
            DEAD_RANGE              => 0,
            MAX_ABS_CTRLOUTPUT      => 2**ABS_PWM_LENGTH-1,
            Tt_ANTIWINDUP_BITWIDTH  => 0
            )
        port map (
            CLK            => clk,                  
            RST            => reset,
            K_fixpoint_in  => K_fixpoint,
            Ki_fixpoint_in => Ki_fixpoint,
            Kd_fixpoint_in => Kd_fixpoint,
            b_fixpoint_in  => b_fixpoint,
            c_fixpoint_in  => c_fixpoint,
            RefValue_in    => limitedDesiredPosition,
            IsValue_in     => std_logic_vector(actualPosition(15 downto 0)),
            CtrlOutput     => PosSpdCtrlOut,
            errorDebug     => open,
            pBufferDebug   => open,
            iBufferDebug   => open,
            dBufferDebug   => open
            ); 
    
    
    -- --------------------------------------------------------------------------
    --                    Notch Filter
    -- --------------------------------------------------------------------------
    
    NotchFilterClkGenerator : process(clk)
        constant COUNTER_MAX : integer := CLK_FREQ/POSCTRL_FREQ; 
        variable counter : integer range 0 to COUNTER_MAX+1;
    begin
        if clk='1' and clk'event then
            if reset='1' then
                counter := 0;
                NotchFilterCLK <= '0';
            else
                if (counter < COUNTER_MAX) then
                    counter := counter + 1;
                    NotchFilterCLK <= '0';
                else
                    counter := 0;
                    NotchFilterCLK <= '1';
                end if;
            end if;
        end if;
    end process;

    process ( clk )
    begin
        if clk'event and clk='1' then
            if rst='1' then
                PosSpdCtrlOut_limited <= (others => '0');
            else
                if signed(PosSpdCtrlOut) > signed(get_register(register_memory, BLDCJoint_DEBUG_PROM_1)) then
                    PosSpdCtrlOut_limited <= get_register(register_memory, BLDCJoint_DEBUG_PROM_1)(10 downto 0);
                elsif signed(PosSpdCtrlOut) < -signed(get_register(register_memory, BLDCJoint_DEBUG_PROM_1)) then
                    PosSpdCtrlOut_limited <= std_logic_vector(-signed(get_register(register_memory, BLDCJoint_DEBUG_PROM_1)(10 downto 0)));
                else
                    PosSpdCtrlOut_limited <= PosSpdCtrlOut;
                end if;
            end if;
        end if;
    end process;
    
    PWMNotchFilter : entity work.notchfilter
        generic map(
            CLK_FREQ          => CLK_FREQ,
            DATA_WIDTH        => ABS_PWM_LENGTH+1,
            PARA_SINT_WIDTH   => 2,       -- Adapt to required integer range + 1 Sign bit
            FIXPOINT_WIDTH    => 16,      -- Adapt to the necessary precision
            A0                => 52990,   -- 58683,   -- signed, relating to data_in(k)
            A1                => -94730,  -- -114211, -- signed, relating to data_in(k-1)
            A2                => 52990,   -- 58683,   -- signed, relating to data_in(k-2)
            B1                => -94730,  -- -114211, -- signed, relating to data_out(k-1)
            B2                => 40444    -- 51831    -- signed, relating to data_out(k-2)
            )
        port map( clk          => CLK,   
                  rst          => reset,
                  en_filt      => bl_enable_notch,
                  trigger      => NotchFilterCLK,
                  data_in      => PosSpdCtrlOut_limited,
                  data_out     => FilteredPWM  
                  );
    
    -- set the control signals
    ctrl_signal_proc : process (clk)
        variable tmp_pwm : signed(15 downto 0);  -- [-32767,32767]
    begin
        if clk'event and clk='1' then
            if rst='1' then
                tmp_pwm := (others => '0');
                ctrlPWM <= (others => '0');
            else
                if ENABLE='1' then
                    if get_register(register_memory, CascadedController_CTRL_MODE)=std_logic_vector(to_unsigned(CascadedController_CTRL_MODE_SETPWM_VALUE,8)) then
                        tmp_pwm := directPWM;
                    else
                        tmp_pwm := shift_left(resize(signed(FilteredPWM),16),5);
                    end if;

                    -- if tmp_pwm is -32768 set it to -32767
                    if tmp_pwm=x"8000" then
                        ctrlPWM <= x"8001";
                    else
                        ctrlPWM <= tmp_pwm;
                    end if;
                else
                    ctrlPWM <= (others => '0');
                end if;
            end if;
        end if;
    end process;


    -----------------------------------------------------------------------------
    --                         Motor Driver
    -----------------------------------------------------------------------------

    hallSensorFilter : entity work.HallSensor_Filter
        port map (
            clk            => clk,
            rst            => rst,
            hall1          => HALL1,
            hall2          => HALL2,
            hall3          => HALL3,
            hall1_filtered => hall1_filtered,
            hall2_filtered => hall2_filtered,
            hall3_filtered => hall3_filtered );

    HallSensorChecker : process (clk)
    begin
        if clk'event and clk='1' then
            if rst='1' then
                error_HallSensor <= '0';
            else
                -- defaults
                error_HallSensor <= '0';
                
                if hall1_filtered=hall2_filtered and
                    hall2_filtered=hall3_filtered then
                    error_HallSensor <= '1';
                end if;
            end if;
        end if;
    end process;

    -- TODO: use pwm filter and actualpwm here (as in V4)
    motorDirection <= ctrlPWM(ctrlPWM'left) when get_register(register_memory,BLDCJoint_INVERT_CONFIG)(BLDCJoint_INVERT_CONFIG_PWM_BIT)='0'
                      else not ctrlPWM(ctrlPWM'left);
    motorAmplitude <= std_logic_vector(resize_sat(shift_right(unsigned(abs(ctrlPWM)),5),10));

    -- instantiation of the three phase H-bridge
    motordriver : entity work.threephasebridge_mod
        generic map(
            CLK_FREQ       => CLK_FREQ,     -- clock frequency
            PWM_FREQ       => 30000,        -- desired PWM frequency
            bridgeDeadTime => "0011",       -- dead time for MosFet switching (bridgeDeadTime + 1 CLKs)
            PWMDeadTime    => 7,            -- PWM dead time in CLKs (PWM cycle time - PWM dead time = 100% PWM!)
            PWMSlewRate    => 3 )           -- pwm cycles until PWM is increased/decreased by 1
        port map(
            CLK           => clk,
            RST           => rst,
            ENABLE        => ENABLE,
            MODE          => '0',             -- 0: closed circuit, 1: open circuit
            DIRECTION     => motorDirection,
            PWMPERCENTAGE => motorAmplitude,  -- desired percentage of pwm
            hall1         => hall1_filtered,  -- input signal wire for the hall sensor 1
            hall2         => hall2_filtered,  -- input signal wire for the hall sensor 2
            hall3         => hall3_filtered,  -- input signal wire for the hall sensor 3
            A1_low        => P1_LIN,          -- low side of halfbridge A
            A1_high       => P1_HIN,          -- high side of halfbridge A
            B1_low        => P2_LIN,          -- low side of halfbridge B
            B1_high       => P2_HIN,          -- high side of halfbridge B
            C1_low        => P3_LIN,          -- low side of halfbridge C
            C1_high       => P3_HIN,          -- high side of halfbridge C
            PWM_out       => actualPWM_raw,   -- the actual PWM
            DIR_out       => actualDir_raw ); -- the actual direction

    -- assign actualPWM (sign depends on the actual motor direction and the configuration bit)
    actualPWM <= -signed(shift_left(resize(unsigned(actualPWM_raw),16),5)) when (actualDir_raw xor get_register(register_memory,BLDCJoint_INVERT_CONFIG)(BLDCJoint_INVERT_CONFIG_PWM_BIT))='1' else
                 signed(shift_left(resize(unsigned(actualPWM_raw),16),5));


    -- --------------------------------------------------------------------------
    --                        Current Measurement
    -- --------------------------------------------------------------------------
    
    ltc2366_3x_inst : entity work.ltc2366_3x
        port map ( clk       => clk,
                   rst       => rst,
                   adcNCS    => ADC_nCS,
                   adcClk    => ADC_CLK,
                   adcDinA   => ADC_SDOA,
                   adcDinB   => ADC_SDOB,
                   adcDinC   => ADC_SDOC,
                   ADCEnable => '1', -- as fast as possible
                   ADCready  => newCurrentValues,
                   dataOutA  => phaseCurrent(0),
                   dataOutB  => phaseCurrent(1),
                   dataOutC  => phaseCurrent(2) );

    TorqueCurrentSensor : entity work.TorqueDirectionDependentCurrentSensor
        port map (
            clk            => clk,
            rst            => rst,
            hall1          => hall1_filtered,
            hall2          => hall2_filtered,
            hall3          => hall3_filtered,
            P1I            => phaseCurrent(0),
            P2I            => phaseCurrent(1),
            P3I            => phaseCurrent(2),
            newCurrentMeas => newCurrentValues,
            current        => actualCurrent_raw );
    
    CurrentFilter : entity work.lowpass
        generic map ( WIDTH  => 16 )
        port map ( clk       => clk,
                   rst       => rst,
                   damping   => get_register(register_memory, BLDCJoint_CUR_FILTER_DAMPING)(4 downto 0),
                   trigger   => newCurrentValues,
                   new_value => actualCurrent_raw,
                   output    => actualCurrent_filtered );

    CurrentConverter : entity work.CurrentConverter
        port map (
            clk         => clk,
            rst         => rst,
            adc_current => actualCurrent_filtered,
            pwm         => actualPWM_raw,
            mA_current  => actualCurrent_slv );

    actualCurrent <= -signed(actualCurrent_slv) when get_register(register_memory,BLDCJoint_INVERT_CONFIG)(BLDCJoint_INVERT_CONFIG_CUR_BIT)='0' else
                     signed(actualCurrent_slv);


    -- --------------------------------------------------------------------------
    --                 Temperature and Voltage Sensors
    -- --------------------------------------------------------------------------

    -- instantiation of the i2c temperature and power sensors
    -- the temperature in degree could be calculated by temp/16.0
    -- the power in V could be calculated by voltage*2.048*27/32768 TODO: ????
    tempPowerSensors : entity work.TempPowerSensors
        generic map (
            CLK_FREQ                      => CLK_FREQ,
            I2C_BAUD                      => 400000,
            TMP100_CON_BOARD_I2C_ADRESS   => "1001110",
            TMP100_CON2_BOARD_I2C_ADRESS  => "1001110", -- dummy address (the same as above to get an answer)
            TMP100_POWER_BOARD_I2C_ADRESS => "1001101",
            BOARD_VERSION_1_3             => true )
        port map (
            CLK           => clk,
            RST           => rst,
            i2c_scl       => i2c_con_scl,
            i2c_sda       => i2c_con_sda,
            tmp_power_scl => i2c_power_scl,
            tmp_power_sda => i2c_power_sda,
            temp_con      => temp0_raw,
            temp_con2     => open,
            temp_power    => temp1_raw,
            power         => voltage_raw );

    -- convert temperature to 9.7 fixed float notation
    temp0 <= shift_left(resize(signed(temp0_raw),16),3);
    temp1 <= shift_left(resize(signed(temp1_raw),16),3);

    temperatureWatcher : process (clk)
    begin
        if clk'event and clk='1' then
            if rst='1' then
                error_OverTemp <= '0';
            else
                -- defaults
                error_OverTemp <= '0';

                if temp1 > temp0 then
                    temp_max <= temp1;
                else
                    temp_max <= temp0;
                end if;

                if temp_max > signed(get_register(register_memory,BLDCJoint_MAX_TEMP_LIMIT)) then
                    error_OverTemp <= '1';
                end if;
            end if;
        end if;
    end process;

    -- process to calculate to voltege in mV
    -- (voltage = voltage_raw*2.048*27*1000/32768 = voltage_raw*27/2^4)
    voltage_calc : process (clk)
        variable voltage_product : unsigned(20 downto 0);
    begin
        if clk'event and clk='1' then
            if rst='1' then
                voltage_product := (others => '0');
                voltage <= (others => '0');
            else
                voltage_product := unsigned(voltage_raw) * to_unsigned(27,5);
                voltage <= std_logic_vector(voltage_product(19 downto 4));
            end if;
        end if;
    end process;


    -- --------------------------------------------------------------------------
    --                        SPI-PROM Access
    -- --------------------------------------------------------------------------

    -- instantiation of the simple spiprom interface
    spiprom_inst : entity work.simple_spiprom_interface
        generic map ( CLK_FREQ     => CLK_FREQ,
                      SPI_CLK_FREQ => 5_000_000,
                      DATA_LENGTH  => 128 )
        port map ( CLK => clk,
                   RST => rst,

                   SPI_nCS   => PROM_nCS,
                   SPI_CLK   => PROM_CLK,
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
            if rst='1' then
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

    reg_write_id   <= com_reg_write_id   when com_reg_write='1' else err_reg_write_id;
    reg_write_type <= com_reg_write_type when com_reg_write='1' else err_reg_write_type;
    reg_write_data <= com_reg_write_data when com_reg_write='1' else err_reg_write_data;
    reg_write      <= com_reg_write or err_reg_write;

    register_access : entity work.register_access
        generic map ( PROM_BASE_ADDR => PROM_CONFIG_BASE )
        port map ( clk => clk,
                   rst => rst,

                   register_memory => register_memory,

                   busy       => reg_busy,
                   init_done  => register_InitDone,
                   write_prom => get_register(register_memory,BLDCJoint_CONFIG)(BLDCJoint_CONFIG_WRITE_SETTINGS_BIT),

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
                   rst => rst,

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
