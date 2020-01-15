--
-- Hex-to-LED decoder circuit
--
-- Listing 3.13 from "FPGA Prototyping by VHDL Examples: Xilinx Spartan-3 Version" by Pong P. Chu
-- Companion website: http://academic.csuohio.edu/chu_p/rtl/fpga_vhdl.html
--
-- Slightly adapted
--

library ieee;
use ieee.std_logic_1164.all;

entity sevseg is
	port (I_clk          : in std_logic;
	      I_clk_fast     : in std_logic;
	      I_data         : in std_logic_vector (15 downto 0);
	      I_dots         : in std_logic_vector (3 downto 0);
	      I_enable_write : in std_logic;

	      O_an           : out std_logic_vector (3 downto 0);
	      O_sseg         : out std_logic_vector (7 downto 0)
	);
end;

architecture logic of sevseg is
	signal data_write_reg : std_logic_vector (15 downto 0);
	signal dots_write_reg : std_logic_vector (3 downto 0);
   	signal led0 	      : std_logic_vector(7 downto 0);
   	signal led1	      : std_logic_vector(7 downto 0);
   	signal led2	      : std_logic_vector(7 downto 0);
   	signal led3	      : std_logic_vector(7 downto 0);
begin
	sevseg_proc: process (I_clk, I_enable_write, I_data, I_dots) is
	begin
		if rising_edge(I_clk) then
			if I_enable_write = '1' then
				data_write_reg <= I_data;
				dots_write_reg <= I_dots;
			end if;
		end if;
	end process;

	-- instantiate four instances of hex decoders

	-- instance for 4 LSBs of input
	sseg_unit_0: entity work.hex_to_sseg(logic)
	port map (I_hex  => data_write_reg(3 downto 0)
		 ,I_dp   => not dots_write_reg(0)
		 ,O_sseg => led0
	 	);

	-- instance for 4 MSBs of input
	sseg_unit_1: entity work.hex_to_sseg(logic)
	port map(I_hex  => data_write_reg(7 downto 4)
		,I_dp   => not dots_write_reg(1)
		,O_sseg => led1
		);

	-- instance for 4 LSBs of incremented value
	sseg_unit_2: entity work.hex_to_sseg(logic)
	port map(I_hex  => data_write_reg(11 downto 8)
		,I_dp   => not dots_write_reg(2)
		,O_sseg => led2
		);

	-- instance for 4 MSBs of incremented value
	sseg_unit_3: entity work.hex_to_sseg(logic)
	port map(I_hex  => data_write_reg(15 downto 12)
		,I_dp   => not dots_write_reg(3)
		,O_sseg => led3
		);

	-- instantiate 7-seg LED display time-multiplexing module
	disp_unit: entity work.disp_mux(logic)
	port map(
		 I_clk   => I_clk_fast
		,I_data0 => led0
		,I_data1 => led1
	       	,I_data2 => led2
		,I_data3 => led3

		,O_an    => O_an
	       	,O_sseg  => O_sseg
		);
end;
