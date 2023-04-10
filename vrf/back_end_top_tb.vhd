--
-- ___________________________________________________________
--
-- INFN DOIN Project board Top-Level
-- (TEST-BENCH)
-- ___________________________________________________________
--
-- Francesco Martina @ 2021
-- v1.1
--

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use ieee.std_logic_textio.all;

-----------------------------------------------------------

entity back_end_top_tb is

end entity back_end_top_tb;

-----------------------------------------------------------

architecture testbench of back_end_top_tb is

	-- Testbench DUT generics


	-- Testbench DUT ports
	signal CLK_40M_ext  : std_logic := '0';
	signal CLK_60M_ext  : std_logic := '0';
	signal RSTn_BUTTON  : std_logic := '0';
	signal LED_R        : std_logic := '0';
	signal LED_G        : std_logic := '0';
	signal LED_B        : std_logic := '0';
	signal ADC          : std_logic_vector(11 downto 0) := x"000";
	signal ADC_CLK_P    : std_logic := '0';
	signal ADC_CLK_N    : std_logic := '0';
	signal ADC_OEn      : std_logic := '0';
	signal ADC_PD       : std_logic := '0';
	signal SCL_DAC      : std_logic := '0';
	signal SDA_DAC      : std_logic := '0';
	signal FIFO_D       : std_logic_vector(7 downto 0) := x"00";
	signal FIFO_RXFn    : std_logic := '0';
	signal FIFO_TXEn    : std_logic := '0';
	signal FIFO_RDn     : std_logic := '0';
	signal FIFO_WRn     : std_logic := '0';
	signal FIFO_SIWUn   : std_logic := '0';
	signal FIFO_OEn     : std_logic := '0';
	signal FIFO_PWRSAVn : std_logic := '0';
	signal PMOD         : std_logic_vector(7 downto 0);
	signal SMA_C_P      : std_logic := '0';
	signal SMA_C_N      : std_logic := '0';

	-- Other constants
	constant C40_CLK_PERIOD : real := 25.000e-9; -- NS
	constant C60_CLK_PERIOD : real := 16.667e-9; -- NS

begin
	-----------------------------------------------------------
	-- Clocks and Reset
	-----------------------------------------------------------
	CLK_40M_ext_GEN : process
	begin
		CLK_40M_ext <= '1';
		wait for C40_CLK_PERIOD / 2.0 * (1 SEC);
		CLK_40M_ext <= '0';
		wait for C40_CLK_PERIOD / 2.0 * (1 SEC);
	end process CLK_40M_ext_GEN;

	CLK_60M_ext_GEN : process
	begin
		CLK_60M_ext <= '1';
		wait for C60_CLK_PERIOD / 2.0 * (1 SEC);
		CLK_60M_ext <= '0';
		wait for C60_CLK_PERIOD / 2.0 * (1 SEC);
	end process CLK_60M_ext_GEN;

	RESET_GEN : process
	begin
		RSTn_BUTTON <= '0',
		         '1' after 50.0*C60_CLK_PERIOD * (1 SEC);
		wait;
	end process RESET_GEN;

	-----------------------------------------------------------
	-- Testbench Stimulus
	-----------------------------------------------------------

	-----------------------------------------------------------
	-- Entity Under Test
	-----------------------------------------------------------
	DUT : entity work.back_end_top
		port map (
			CLK_40M_ext  => CLK_40M_ext,
			CLK_60M_ext  => CLK_60M_ext,
			RSTn_BUTTON  => RSTn_BUTTON,
			LED_R        => LED_R,
			LED_G        => LED_G,
			LED_B        => LED_B,
			ADC          => ADC,
			ADC_CLK_P    => ADC_CLK_P,
			ADC_CLK_N    => ADC_CLK_N,
			ADC_OEn      => ADC_OEn,
			ADC_PD       => ADC_PD,
			SCL_DAC      => SCL_DAC,
			SDA_DAC      => SDA_DAC,
			FIFO_D       => FIFO_D,
			FIFO_RXFn    => FIFO_RXFn,
			FIFO_TXEn    => FIFO_TXEn,
			FIFO_RDn     => FIFO_RDn,
			FIFO_WRn     => FIFO_WRn,
			FIFO_SIWUn   => FIFO_SIWUn,
			FIFO_OEn     => FIFO_OEn,
			FIFO_PWRSAVn => FIFO_PWRSAVn,
			PMOD         => PMOD,
			SMA_C_P      => SMA_C_P,
			SMA_C_N      => SMA_C_N
		);

end architecture testbench;