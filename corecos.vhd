--
-- Rival - A RISC-V RV32I CPU
--
-- corecos.vhd: This entity combines the CPU with a seven segment display unit and
-- 		an LED array and interfaces it to Spartan-3 Start Board equiped
-- 		with a Xilinx Spartan-3 XC3S1000FT256-4 FPGA
--
-- Copyright (c) 2020, Helmut Sipos <helmut.sipos@gmail.com>
-- All rights reserved.
--
-- Redistribution and use in source and binary forms, with or without
-- modification, are permitted provided that the following conditions are met:
--
-- 1. Redistributions of source code must retain the above copyright notice, this
--    list of conditions and the following disclaimer.
-- 2. Redistributions in binary form must reproduce the above copyright notice,
--    this list of conditions and the following disclaimer in the documentation
--    and/or other materials provided with the distribution.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
-- ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
-- WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
-- DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
-- ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
-- (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
-- LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
-- ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
-- (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
-- SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

entity corecos is
	port(I_clk        : in std_logic;
	     I_switches   : in std_logic_vector (7 downto 0);

	     O_leds       : out std_logic_vector (7 downto 0);
	     O_sseg       : out std_logic_vector (7 downto 0);
	     O_an         : out std_logic_vector (3 downto 0)
	);
end;

architecture logic of corecos is
	signal R_clk		     : std_logic;
	signal R_clk_prescaler       : std_logic_vector(25 downto 0) := (others => '0');
	signal R_mem_address	     : std_logic_vector(31 downto 2);
	signal R_mem_we_bytes        : std_logic_vector(3 downto 0);
	signal R_mem_data_write      : std_logic_vector(31 downto 0);
	signal R_enable_leds_write   : std_logic;
	signal R_enable_sevseg_write : std_logic;
	signal R_debug_data	     : std_logic_vector(31 downto 0);
	signal R_debug_flags	     : std_logic_vector(7 downto 0);
	signal R_sevseg_data         : std_logic_vector(15 downto 0);
	signal R_sevseg_dots         : std_logic_vector(3 downto 0);

begin
	clk_gen_proc : process (I_clk, I_switches) is
	begin
		-- In debug mode use a slow clock
		if rising_edge(I_clk) then
			if I_switches(0) = '1' then
				-- ~1 Hz
				if R_clk_prescaler < "10111110101111000010000000" then
					R_clk_prescaler <= R_clk_prescaler + 1;
				else
					R_clk_prescaler <= (others => '0');
					R_clk <= not R_clk;
				end if;
			else
				-- 25 MHz
				R_clk <= not R_clk;

			end if;
		end if;
	end process;

	-- Instantiate and connect the CPU
	cpu: entity work.cpu(logic)
	port map (
		 I_clk            => R_clk
		,I_debug_select   => I_switches(7 downto 6)

		,O_mem_address    => R_mem_address
		,O_mem_data_write => R_mem_data_write
		,O_mem_we 	  => R_mem_we_bytes
		,O_debug_data     => R_debug_data
		,O_debug_flags    => R_debug_flags
	);


	debug_proc: process (I_switches, R_mem_address, R_mem_data_write, R_mem_we_bytes, R_debug_data, R_debug_flags) is
	begin
		if I_switches(0) = '1' then
			if I_switches(5) = '1' then
				R_sevseg_data <= R_debug_data(31 downto 16);
			else
				R_sevseg_data <= R_debug_data(15 downto 0);
			end if;

			if I_switches(4) = '1' then
				R_sevseg_dots <= R_debug_flags(7 downto 4);
			else
				R_sevseg_dots <= R_debug_flags(3 downto 0);
			end if;

			R_enable_sevseg_write <= '1';
		else
			R_sevseg_data <= R_mem_data_write(15 downto 0);

			R_sevseg_dots <= "0000";

			-- Seven segment display mapped at address 0x20000004
			if R_mem_address = "001000000000000000000000000001"and R_mem_we_bytes /= "0000" then
				R_enable_sevseg_write <= '1';
			else
				R_enable_sevseg_write <= '0';
			end if;
		end if;
	end process;

	-- Instantiate and connect the seven segment display
	sevseg: entity work.sevseg(logic)
	port map(
		 I_clk	        => R_clk
		,I_clk_fast     => I_clk
		,I_enable_write => R_enable_sevseg_write
		,I_data         => R_sevseg_data
		,I_dots         => R_sevseg_dots

		,O_an           => O_an
		,O_sseg         => O_sseg
	);

	-- LEDs mapped at address 0x20000000
	R_enable_leds_write <= '1' when R_mem_address = "001000000000000000000000000000"
			    	 and R_mem_we_bytes /= "0000" else '0';

	-- Instantiate and connect the LED array display
	leds: entity work.leds(logic)
	port map(
		 I_clk          => R_clk
		,I_enable_write => R_enable_leds_write
		,I_data		=> R_mem_data_write(7 downto 0)

		,O_leds         => O_leds
	);
end;

