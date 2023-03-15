--
-- ___________________________________________________________
--
-- INFN DOIN Project board Top-Level
-- ___________________________________________________________
--
-- Francesco Martina @ 2021
-- v1.1
--

-- TODO
-- pending DAC up/down configuration
-- trigger_miss signaling
-- Calibration EEPROM programming

library IEEE;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library UNISIM;
use UNISIM.VComponents.all;

entity back_end_top is
    Port (
        -- general
        CLK_40M_ext : in std_logic;
        CLK_60M_ext : in std_logic;
        RSTn_BUTTON : in std_logic;

        -- LEDs
        LED_R : out std_logic;
        LED_G : out std_logic;
        LED_B : out std_logic;

        -- ADC interface
        ADC       : in  std_logic_vector(11 downto 0);
        ADC_CLK_P : out std_logic;
        ADC_CLK_N : out std_logic;
        ADC_OEn   : out std_logic;
        ADC_PD    : out std_logic;

        -- DAC interface
        SCL_DAC : inout std_logic;
        SDA_DAC : inout std_logic;

        -- FT232HL USB FIFO
        FIFO_D       : inout std_logic_vector(7 downto 0);
        FIFO_RXFn    : in    std_logic;
        FIFO_TXEn    : in    std_logic;
        FIFO_RDn     : out   std_logic;
        FIFO_WRn     : out   std_logic;
        FIFO_SIWUn   : out   std_logic;
        FIFO_OEn     : out   std_logic;
        FIFO_PWRSAVn : out   std_logic;

        -- PMOD
        PMOD : out std_logic_vector(7 downto 0);

        -- SMA
        --SMA_A_P : out std_logic;
        --SMA_A_N : out std_logic;
        --SMA_B_P : out std_logic;
        --SMA_B_N : out std_logic;
        SMA_C_P : out std_logic;
        SMA_C_N : out std_logic

        -- Configuration EEPROM
        -- EEDI  : out std_logic;
        -- EEDO  : in  std_logic;
        -- EECS  : out std_logic;
        -- EECLK : out std_logic

    );

end back_end_top;

architecture Behavioral of back_end_top is

    ----------------------------------------------------------------------------
    -- Signals
    ----------------------------------------------------------------------------

    -- clocks
    signal CLK_40M : std_logic;
    signal CLK_60M : std_logic;

    -- internal reset
    signal async_rst_general               : std_logic;
    signal system_clock_locked             : std_logic;
    signal sync_rst_general_40             : std_logic;
    signal rst_trigger_40                  : std_logic;
    signal rst_general_combined_trigger_40 : std_logic;
    signal rst_40                          : std_logic;
    signal rstn_40                         : std_logic;
    signal rst_60                          : std_logic;
    signal rstn_60                         : std_logic;

    -- CDC <-> FIFO bus
    signal FIFO_in_tdata     : std_logic_vector(7 downto 0);
    signal FIFO_in_tvalid    : std_logic;
    signal FIFO_in_tready    : std_logic;
    signal FIFO_out_tdata    : std_logic_vector(7 downto 0);
    signal FIFO_out_tvalid   : std_logic;
    signal FIFO_out_tready   : std_logic;
    signal USB_FIFO_UP_wr_en : std_logic;
    signal USB_FIFO_UP_rd_en : std_logic;
    signal USB_FIFO_UP_full  : std_logic;
    signal USB_FIFO_UP_empty : std_logic;

    signal CONFIG_in_tdata        : std_logic_vector(7 downto 0);
    signal CONFIG_in_tvalid       : std_logic;
    signal CONFIG_in_tready       : std_logic;
    signal ACQUISITION_out_tdata  : std_logic_vector(15 downto 0);
    signal ACQUISITION_out_tvalid : std_logic;
    signal ACQUISITION_out_tready : std_logic;

    -- configuration parameters
    signal pre_trigger_size               : std_logic_vector(31 downto 0);
    signal post_trigger_size              : std_logic_vector(31 downto 0);
    signal trigger_threshold              : std_logic_vector(11 downto 0);
    signal trigger_polarity               : std_logic;
    signal trigger_hysteresis             : std_logic_vector(7 downto 0);
    signal trigger_mode                   : std_logic_vector(2 downto 0);
    signal trigger_auto_mode_timeout      : std_logic_vector(31 downto 0);
    signal trigger_continuous_mode_period : std_logic_vector(31 downto 0);
    signal trigger_holdoff_time           : std_logic_vector(31 downto 0);
    signal pre_trigger_FIFO_configuration : std_logic_vector(31 downto 0);
    signal DAC_channel_A                  : std_logic_vector(7 downto 0);
    signal DAC_channel_B                  : std_logic_vector(7 downto 0);
    signal pulse_generator_period         : std_logic_vector(31 downto 0);
    signal pulse_generator_width          : std_logic_vector(31 downto 0);
    signal sample_rate_divider            : std_logic_vector(31 downto 0);
    signal DAC_power_up_update            : std_logic;
    signal DAC_power_down                 : std_logic;
    signal DAC_configurator_busy          : std_logic;

    -- acquisition signals
    signal pre_trigger_out_tdata_extended : std_logic_vector(15 downto 0);
    signal pre_trigger_out_tdata          : std_logic_vector(11 downto 0);
    signal pre_trigger_out_tvalid         : std_logic;
    signal pre_trigger_out_tready         : std_logic;
    signal pre_trigger_in_tdata           : std_logic_vector(11 downto 0);
    signal pre_trigger_in_tvalid          : std_logic;
    signal pre_trigger_in_tready          : std_logic;

    signal pre_trigger_full    : std_logic;
    signal trigger             : std_logic;
    signal trigger_acquisition : std_logic;
    signal trigger_timestamp   : std_logic_vector(47 downto 0);

    signal adc_data_stream_in_tdata   : std_logic_vector(11 downto 0);
    signal adc_data_stream_in_tvalid  : std_logic;
    signal adc_data_stream_out_tdata  : std_logic_vector(11 downto 0);
    signal adc_data_stream_out_tvalid : std_logic;

    -- aux
    signal pulse_generator_out : std_logic;

    -- diagnostics
    signal armed_flag      : std_logic;
    signal trigger_stopped : std_logic;
    signal triggered_flag  : std_logic;
    signal overrun_flag    : std_logic;
    signal armed_led       : std_logic;

    ---- DEBUG
    signal CONTROL_ILA : std_logic_vector(35 downto 0);

begin

    ----------------------------------------------------------------------------
    -- System General Clocking and Reset
    ----------------------------------------------------------------------------

    -- reset connection
    async_rst_general   <= not RSTn_BUTTON;
    sync_rst_general_40 <= not system_clock_locked;

    system_clock_i : entity work.system_clock
        port map (
            CLK_40M_ext => CLK_40M_ext,
            CLK_40M     => CLK_40M,
            RESET       => async_rst_general,
            LOCKED      => system_clock_locked
        );

    USB_clock_driven_logics_BUFG : BUFG
        port map (
            O => CLK_60M,
            I => CLK_60M_ext
        );

    -- combines the clean general reset (derived from the external push button) and the internal auto-reset
    rst_general_combined_trigger_40 <= sync_rst_general_40 or rst_trigger_40;

    -- reset controller to implement the auto-reset system, driven by board_config
    reset_controller_i : entity work.reset_controller
        generic map (
            RST_CYCLES => 100,
            RST_DELAY  => 10
        )
        port map (
            clk          => CLK_40M,
            async_rst_in => '0',
            sync_rst_in  => rst_general_combined_trigger_40,
            rst_out      => rst_40,
            rstn_out     => rstn_40
        );

    -- reset synchroniser used between the two clock domains
    reset_synchroniser_domain_crossing : entity work.synchronizer
        generic map (
            STAGES => 5
        )
        port map (
            clk      => CLK_60M,
            aync_in  => rst_40,
            sync_out => rst_60
        );

    rstn_60 <= not rst_60;

    ----------------------------------------------------------------------------
    -- Acquisition Signal Path
    ----------------------------------------------------------------------------

    -- ADC simplest input data register
    ADC_interface_i : entity work.ADC_interface
        port map (
            clk                 => CLK_40M,
            rst                 => rst_40,
            ADC_CLK_P           => ADC_CLK_P,
            ADC_CLK_N           => ADC_CLK_N,
            ADC                 => ADC,
            sample_rate_divider => sample_rate_divider,
            adc_stream_tdata    => adc_data_stream_in_tdata,
            adc_stream_tvalid   => adc_data_stream_in_tvalid,
            ADC_OEn             => ADC_OEn,
            ADC_PD              => ADC_PD
        );

    -- pre-trigger samples memory FIFO
    pre_trigger_FIFO_i : entity work.pre_trigger_FIFO
        generic map (
            SAMPLE_WIDTH => 12
        )
        port map (
            clk                 => CLK_40M,
            rst                 => rst_40,
            in_tdata            => adc_data_stream_out_tdata,
            in_tvalid           => adc_data_stream_out_tvalid,
            in_tready           => open,
            out_tdata           => pre_trigger_out_tdata,
            out_tvalid          => pre_trigger_out_tvalid,
            out_tready          => pre_trigger_out_tready,
            pre_trigger_samples => pre_trigger_FIFO_configuration,
            pre_trigger_full    => pre_trigger_full
        );

    -- trigger system controller
    trigger_system_i : entity work.trigger_system
        generic map (
            SAMPLE_WIDTH => 12
        )
        port map (
            clk                            => CLK_40M,
            rst                            => rst_40,
            adc_data_stream_in_tdata       => adc_data_stream_in_tdata,
            adc_data_stream_in_tvalid      => adc_data_stream_in_tvalid,
            adc_data_stream_out_tdata      => adc_data_stream_out_tdata,
            adc_data_stream_out_tvalid     => adc_data_stream_out_tvalid,
            trigger_threshold              => trigger_threshold,
            trigger_polarity               => trigger_polarity,
            trigger_hysteresis             => trigger_hysteresis,
            trigger_mode                   => trigger_mode,
            trigger_auto_mode_timeout      => trigger_auto_mode_timeout,
            trigger_continuous_mode_period => trigger_continuous_mode_period,
            trigger_holdoff_time           => trigger_holdoff_time,
            trigger                        => trigger,
            trigger_stopped                => trigger_stopped
        );

    -- timestamp generator
    timestamp_i : entity work.timestamp
        port map (
            clk       => CLK_40M,
            rst       => rst_40,
            timestamp => trigger_timestamp
        );

    -- Acquisition controller module
    pre_trigger_out_tdata_extended <= "0000" & pre_trigger_out_tdata;

    -- trigger is inhibited during the DAC configuration
    trigger_acquisition <= trigger and not(DAC_configurator_busy);

    acquisition_control_i : entity work.acquisition_control
        port map (
            clk                            => CLK_40M,
            rst                            => rst_40,
            in_tdata                       => pre_trigger_out_tdata_extended,
            in_tvalid                      => pre_trigger_out_tvalid,
            in_tready                      => pre_trigger_out_tready,
            out_tdata                      => ACQUISITION_out_tdata,
            out_tvalid                     => ACQUISITION_out_tvalid,
            out_tready                     => ACQUISITION_out_tready,
            out_tlast                      => open,
            pre_trigger_size               => pre_trigger_size,
            post_trigger_size              => post_trigger_size,
            trigger_input                  => trigger_acquisition,
            trigger_timestamp_input        => trigger_timestamp,
            sample_rate_divider            => sample_rate_divider,
            pre_trigger_full               => pre_trigger_full,
            pre_trigger_FIFO_configuration => pre_trigger_FIFO_configuration,
            trigger_miss                   => open,
            armed_flag                     => armed_flag,
            triggered_flag                 => triggered_flag,
            overrun_flag                   => overrun_flag
        );


    ----------------------------------------------------------------------------
    -- USB Interface for Data Transmission and Board Configuration
    ----------------------------------------------------------------------------

    -- FT232H device always on
    FIFO_PWRSAVn <= '1';

    -- USB FIFO Interface
    ft232h_interface_i : entity work.ft232h_interface
        port map (
            clk        => CLK_60M,
            rst        => rst_60,
            in_tdata   => FIFO_in_tdata,
            in_tvalid  => FIFO_in_tvalid,
            in_tready  => FIFO_in_tready,
            out_tdata  => FIFO_out_tdata,
            out_tvalid => FIFO_out_tvalid,
            out_tready => FIFO_out_tready,
            ADBUS      => FIFO_D,
            RXFn       => FIFO_RXFn,
            RDn        => FIFO_RDn,
            OEn        => FIFO_OEn,
            TXEn       => FIFO_TXEn,
            WRn        => FIFO_WRn,
            SIWUn      => FIFO_SIWUn
        );

    ACQUISITION_out_tready <= not USB_FIFO_UP_full;
    USB_FIFO_UP_wr_en      <= (ACQUISITION_out_tvalid and ACQUISITION_out_tready);
    FIFO_in_tvalid         <= not USB_FIFO_UP_empty;
    USB_FIFO_UP_rd_en      <= (FIFO_in_tvalid and FIFO_in_tready);

    -- CDC FIFO, from the acquisition system to the FT232H
    USB_FIFO_UPLINK_i : entity work.USB_FIFO_UPLINK
        port map (
            rst    => rst_40,
            wr_clk => CLK_40M,
            rd_clk => CLK_60M,
            din    => ACQUISITION_out_tdata,
            wr_en  => USB_FIFO_UP_wr_en,
            rd_en  => USB_FIFO_UP_rd_en,
            dout   => FIFO_in_tdata,
            full   => USB_FIFO_UP_full,
            empty  => USB_FIFO_UP_empty
        );

    -- CDC FIFO, from the FT232H to the configuration module
    USB_FIFO_DOWNLINK_i : entity work.USB_FIFO_DOWNLINK
        port map (
            m_aclk        => CLK_40M,
            s_aclk        => CLK_60M,
            s_aresetn     => rstn_60,
            s_axis_tvalid => FIFO_out_tvalid,
            s_axis_tready => FIFO_out_tready,
            s_axis_tdata  => FIFO_out_tdata,
            m_axis_tvalid => CONFIG_in_tvalid,
            m_axis_tready => CONFIG_in_tready,
            m_axis_tdata  => CONFIG_in_tdata
        );

    ----------------------------------------------------------------------------
    -- Board Control
    ----------------------------------------------------------------------------

    -- Board configuration module
    board_config_i : entity work.board_config
        port map (
            clk                            => CLK_40M,
            rst                            => rst_40,
            in_tdata                       => CONFIG_in_tdata,
            in_tvalid                      => CONFIG_in_tvalid,
            in_tready                      => CONFIG_in_tready,
            sample_rate_divider            => sample_rate_divider,
            pre_trigger_size               => pre_trigger_size,
            post_trigger_size              => post_trigger_size,
            trigger_threshold              => trigger_threshold,
            trigger_polarity               => trigger_polarity,
            trigger_hysteresis             => trigger_hysteresis,
            trigger_mode                   => trigger_mode,
            trigger_auto_mode_timeout      => trigger_auto_mode_timeout,
            trigger_continuous_mode_period => trigger_continuous_mode_period,
            trigger_holdoff_time           => trigger_holdoff_time,
            pulse_generator_period         => pulse_generator_period,
            pulse_generator_width          => pulse_generator_width,
            DAC_channel_A                  => DAC_channel_A,
            DAC_channel_B                  => DAC_channel_B,
            DAC_power_up_update            => DAC_power_up_update,
            DAC_power_down                 => DAC_power_down,
            general_reset_trigger          => rst_trigger_40
        );

    -- DAC configurator and I2C interface (I2C clock at 400kHz)
    DACconfigurator_i : entity work.DACconfigurator
        port map (
            clk40           => CLK_40M,
            rst             => rst_40,
            DAC_channel_A   => DAC_channel_A,
            DAC_channel_B   => DAC_channel_B,
            power_up_update => DAC_power_up_update,
            power_down      => DAC_power_down,
            scl_io          => SCL_DAC,
            sda_io          => SDA_DAC,
            busy            => DAC_configurator_busy
        );

    ----------------------------------------------------------------------------
    -- Diagnostic LEDs
    ----------------------------------------------------------------------------

    -- green LED (ARMED Acquisition)
    armed_led <= (not trigger_stopped) and armed_flag;
    LED_stretcher_ARMED : entity work.LED_stretcher
        generic map (
            CLOCK_FREQ   => 40000000.0,
            STRETCH_TIME => 0.08,
            POLARITY     => FALSE
        )
        port map (
            clk     => CLK_40M,
            rst     => rst_40,
            flag_in => armed_led,
            LED_out => LED_G
        );

    -- blue LED (Acquisition TRIGGERED)
    LED_stretcher_TRIGGERED : entity work.LED_stretcher
        generic map (
            CLOCK_FREQ   => 40000000.0,
            STRETCH_TIME => 0.01,
            POLARITY     => TRUE
        )
        port map (
            clk     => CLK_40M,
            rst     => rst_40,
            flag_in => triggered_flag,
            LED_out => LED_B
        );

    -- red LED (OVERRUN Error)
    LED_stretcher_OVERRUN : entity work.LED_stretcher
        generic map (
            CLOCK_FREQ   => 40000000.0,
            STRETCH_TIME => 0.10,
            POLARITY     => TRUE
        )
        port map (
            clk     => CLK_40M,
            rst     => rst_40,
            flag_in => overrun_flag,
            LED_out => LED_R
        );

    ----------------------------------------------------------------------------
    -- Pulse Generator
    ----------------------------------------------------------------------------

    pulse_generator_i : entity work.pulse_generator
        port map (
            clk          => CLK_40M,
            rst          => rst_40,
            pulse_period => pulse_generator_period,
            pulse_width  => pulse_generator_width,
            pulse_out    => pulse_generator_out
        );

    OBUFDS_inst : OBUFDS
        generic map ( IOSTANDARD => "LVDS_33")
        port map (
            O  => SMA_C_P,            -- Diff_p output (connect directly to top-level port)
            OB => SMA_C_N,            -- Diff_n output (connect directly to top-level port)
            I  => pulse_generator_out -- Buffer input 
        );

    ----------------------------------------------------------------------------
    -- DEBUG modules
    ----------------------------------------------------------------------------

    ICON_i : entity work.ICON
        port map (
            CONTROL0 => CONTROL_ILA
        );

    DATA_OUT_ILA_i : entity work.FIFO_ILA
        port map (
            CONTROL  => CONTROL_ILA,
            CLK      => CLK_60M,
            TRIG0    => FIFO_in_tdata,
            TRIG1(0) => FIFO_in_tvalid,
            TRIG2(0) => FIFO_in_tready
        );

    -- PMOD diagnostic
    PMOD <= "00" & DAC_power_up_update & DAC_power_down & DAC_configurator_busy & pulse_generator_out & adc_data_stream_in_tvalid & trigger_acquisition;

end Behavioral;
