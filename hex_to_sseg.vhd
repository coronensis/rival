--
-- Hexadecimal digit to seven-segment LED decoder
--
-- Listing 3.12 from "FPGA Prototyping by VHDL Examples: Xilinx Spartan-3 Version" by Pong P. Chu
-- Companion website: http://academic.csuohio.edu/chu_p/rtl/fpga_vhdl.html
--
-- Slightly adapted
--

library ieee;
use ieee.std_logic_1164.all;

entity hex_to_sseg is
	port(
		I_hex  : in std_logic_vector(3 downto 0);
		I_dp   : in std_logic;

		O_sseg : out std_logic_vector(7 downto 0)
	);
end hex_to_sseg;

architecture logic of hex_to_sseg is
begin
	with I_hex select
		O_sseg(6 downto 0) <=
			"0000001" when "0000",
			"1001111" when "0001",
			"0010010" when "0010",
			"0000110" when "0011",
			"1001100" when "0100",
			"0100100" when "0101",
			"0100000" when "0110",
			"0001111" when "0111",
			"0000000" when "1000",
			"0000100" when "1001",
			"0001000" when "1010", --a
			"1100000" when "1011", --b
			"0110001" when "1100", --c
			"1000010" when "1101", --d
			"0110000" when "1110", --e
			"0111000" when others; --f

		O_sseg(7) <= I_dp;

end logic;

