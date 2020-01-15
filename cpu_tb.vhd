library ieee;
use ieee.std_logic_1164.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;

entity cpu_tb is
end cpu_tb;

architecture behavior of cpu_tb is

   --Inputs
   signal I_clk : std_logic := '0';

   --Outputs
   signal O_mem_address : std_logic_vector(31 downto 2);
   signal O_mem_data_write : std_logic_vector(31 downto 0);
   signal O_mem_we : std_logic;
   signal O_debug_data : std_logic_vector(31 downto 0);

   -- Clock period definitions
   constant I_clk_period : time := 10 ns;

begin

   -- Instantiate the Unit Under Test (UUT)
   uut: entity work.cpu(logic)
   port map (
          I_clk => I_clk,
          O_mem_address => O_mem_address,
          O_mem_data_write => O_mem_data_write,
          O_mem_we => O_mem_we,
          O_debug_data => O_debug_data
        );

   -- Clock process definitions
   I_clk_process :process (I_clk) is
   begin
		I_clk <= '0';
		wait for I_clk_period/2;
		I_clk <= '1';
		wait for I_clk_period/2;
   end process;

   -- Stimulus process
   stim_proc: process
   begin
      -- hold reset state for 100 ns.
      wait for 100 ns;

      wait for I_clk_period*10;

      -- insert stimulus here

      wait;
   end process;

end;
