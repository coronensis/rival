--
-- LED time-multiplexing circuit with LED patterns
--
-- Listing 4.13 from "FPGA Prototyping by VHDL Examples: Xilinx Spartan-3 Version" by Pong P. Chu
-- Companion website: http://academic.csuohio.edu/chu_p/rtl/fpga_vhdl.html
--
-- Slightly adapted
--

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity disp_mux is
	port(
		I_clk   : in std_logic;
		I_data0 : in std_logic_vector(7 downto 0);
		I_data1 : in std_logic_vector(7 downto 0);
		I_data2 : in std_logic_vector(7 downto 0);
		I_data3 : in std_logic_vector(7 downto 0);

		O_an    : out std_logic_vector(3 downto 0);
		O_sseg  : out std_logic_vector(7 downto 0)
	);
end disp_mux;

architecture logic of disp_mux is
	-- refreshing rate around 47 Hz (50MHz/2^19/2)
	constant N: integer := 19;
	signal q_reg  : unsigned(N-1 downto 0);
	signal q_next : unsigned(N-1 downto 0);
	signal sel: std_logic_vector(1 downto 0);
begin
	-- register
	process(I_clk)
	begin
		if (I_clk'event and I_clk = '1') then
			q_reg <= q_next;
		end if;
	end process;

	-- next-state logic for the counter
	q_next <= q_reg + 1;

	-- 2 MSBs of counter to control 4-to-1 multiplexing
	-- and to generate active-low enable signal

	sel <= std_logic_vector(q_reg(N-1 downto N-2));

	process(sel, I_data0 ,I_data1, I_data2, I_data3)
	begin
		case sel is
			when "00" =>
				O_an <= "1110";
				O_sseg <= I_data0;
			when "01" =>
				O_an <= "1101";
				O_sseg <= I_data1;
			when "10" =>
				O_an <= "1011";
				O_sseg <= I_data2;
			when others =>
				O_an <= "0111";
				O_sseg <= I_data3;
		end case;
	end process;
end logic;

