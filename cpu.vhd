--
-- Rival - A RISC-V RV32I CPU
--
-- cpu.vhd: Implements the whole CPU including the program counter logic, memory
--          controller, instruction decoder, ALU, register bank and internal RAM
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
use ieee.std_logic_unsigned.all;
use ieee.std_logic_arith.all;
use ieee.numeric_std.all;

library unisim;
use unisim.vcomponents.all;

library work;
use work.all;

entity cpu is
	port (
		I_clk		 : in std_logic;
		I_debug_select   : in std_logic_vector(1 downto 0);

		O_mem_address  	 : out std_logic_vector(31 downto 2);
		O_mem_data_write : out std_logic_vector(31 downto 0);
		O_mem_we	 : out std_logic_vector(3 downto 0);
		O_debug_data     : out std_logic_vector(31 downto 0);
		O_debug_flags    : out std_logic_vector(7  downto 0)
	);
end;

architecture logic of cpu is

	--- CONSTANTS AND TYPE DEFINITIONS ---

	constant ZEROS	: std_logic_vector(31 downto 0) := "00000000000000000000000000000000";
	constant ONES	: std_logic_vector(31 downto 0) := "11111111111111111111111111111111";

	-- Arithmetic Logic Unit
	subtype alu_function_type is std_logic_vector(3 downto 0);
	constant ALU_FUNC_NONE		: alu_function_type := "0000";
	constant ALU_FUNC_ADD		: alu_function_type := "0001";
	constant ALU_FUNC_SUB		: alu_function_type := "0010";
	constant ALU_FUNC_SLT		: alu_function_type := "0011";
	constant ALU_FUNC_SLTU		: alu_function_type := "0100";
	constant ALU_FUNC_MUL		: alu_function_type := "0101";
	constant ALU_FUNC_XOR		: alu_function_type := "0110";
	constant ALU_FUNC_OR		: alu_function_type := "0111";
	constant ALU_FUNC_AND		: alu_function_type := "1000";
	constant ALU_FUNC_SLL		: alu_function_type := "1001";
	constant ALU_FUNC_SRL		: alu_function_type := "1010";
	constant ALU_FUNC_SRA		: alu_function_type := "1011";

	subtype alu_a_from_type is std_logic_vector(1 downto 0);
	constant ALU_A_FROM_RS1		: alu_a_from_type := "00";
	constant ALU_A_FROM_PC		: alu_a_from_type := "01";

	subtype alu_b_from_type is std_logic_vector(2 downto 0);
	constant ALU_B_FROM_RS2		  : alu_b_from_type := "000";
	constant ALU_B_FROM_IMM20_U_SLL12 : alu_b_from_type := "001";
	constant ALU_B_FROM_IMM21_J       : alu_b_from_type := "010";
	constant ALU_B_FROM_IMM12_I       : alu_b_from_type := "011";
	constant ALU_B_FROM_IMM13_B       : alu_b_from_type := "100";
	constant ALU_B_FROM_IMM12_S       : alu_b_from_type := "101";
	constant ALU_B_FROM_SHAMT         : alu_b_from_type := "110";

	-- Program Counter
	subtype pc_source_type is std_logic_vector(1 downto 0);
	constant PC_FROM_INC4		: pc_source_type := "00";
	constant PC_FROM_ALU		: pc_source_type := "01";

	-- Instruction Decoder
	subtype rd_val_new_from_type is std_logic_vector(2 downto 0);
	constant RD_FROM_NULL		: rd_val_new_from_type := "000";
	constant RD_FROM_ALU		: rd_val_new_from_type := "001";
	constant RD_FROM_PC		: rd_val_new_from_type := "010";
	constant RD_FROM_MEMORY		: rd_val_new_from_type := "011";
	constant RD_FROM_IMM20_U_SLL12	: rd_val_new_from_type := "100";

	subtype branch_function_type is std_logic_vector(2 downto 0);
	constant BRANCH_NONE          : branch_function_type := "000";
	constant BRANCH_IF_EQ         : branch_function_type := "001";
	constant BRANCH_IF_NE         : branch_function_type := "010";
        constant BRANCH_IF_LT	      : branch_function_type := "011";
	constant BRANCH_IF_GE	      : branch_function_type := "100";
	constant BRANCH_IF_LTU	      : branch_function_type := "101";
	constant BRANCH_IF_GEU	      : branch_function_type := "110";
	constant BRANCH_UNCONDITIONAL : branch_function_type := "111";

	-- Memory controller
	subtype mem_source_type is std_logic_vector(3 downto 0);
	constant MEM_ACCESS_FETCH	: mem_source_type := "0000";
	constant MEM_ACCESS_READ_INT8	: mem_source_type := "0001";
	constant MEM_ACCESS_READ_UINT8	: mem_source_type := "0010";
	constant MEM_ACCESS_READ_INT16	: mem_source_type := "0011";
	constant MEM_ACCESS_READ_UINT16	: mem_source_type := "0100";
	constant MEM_ACCESS_READ_INT32	: mem_source_type := "0101";
	constant MEM_ACCESS_WRITE_INT8	: mem_source_type := "0110";
	constant MEM_ACCESS_WRITE_INT16	: mem_source_type := "0111";
	constant MEM_ACCESS_WRITE_INT32	: mem_source_type := "1000";

	subtype mem_state_type is std_logic;
	constant MEM_STATE_ADDR	  : mem_state_type := '0';
	constant MEM_STATE_ACCESS : mem_state_type := '1';

	--- REGISTERS ---

	-- Instruction Decoder
	signal R_instruction	: std_logic_vector(31 downto 0);
	signal R_ram_read_val	: std_logic_vector(31 downto 0);
	signal R_alu_func	: alu_function_type;
	signal R_mem_source	: mem_source_type;
	signal R_take_branch	: std_logic;

	-- Arithmetic Logic Unit
	signal R_alu_a		: std_logic_vector(31 downto 0);
	signal R_alu_b		: std_logic_vector(31 downto 0);
	signal R_alu_res	: std_logic_vector(31 downto 0);

	signal R_loc_alu_fac_a  : std_logic_vector(17 downto 0);
	signal R_loc_alu_fac_b  : std_logic_vector(17 downto 0);
	signal R_loc_alu_prod   : std_logic_vector(35 downto 0);
	signal R_loc_alu_slt    : std_logic_vector(31 downto 0);
	signal R_loc_alu_sltu   : std_logic_vector(31 downto 0);
	signal R_loc_alu_sll    : bit_vector (31 downto 0);
	signal R_loc_alu_srl    : bit_vector (31 downto 0);
	signal R_loc_alu_sra    : bit_vector (31 downto 0);
	signal R_loc_alu_sum    : signed (31 downto 0);
	signal R_loc_alu_dif    : signed (31 downto 0);

	-- Program Counter
	signal R_pc_source	   : pc_source_type;
	signal R_pc_current	   : std_logic_vector(31 downto 2);
	signal R_pc_next	   : std_logic_vector(31 downto 2);
	signal R_pc_enable_decoder : std_logic;

	signal R_loc_pc_current : std_logic_vector(31 downto 2) := "111111111111111111111111111111";

	-- Memory controller
	signal R_mctrl_cpu_pause: std_logic;

	signal R_loc_mctrl_instruction	  : std_logic_vector(31 downto 0);
	signal R_loc_mctrl_instruction_next : std_logic_vector(31 downto 0);
	signal R_loc_mctrl_address	  : std_logic_vector(31 downto 2);
	signal R_loc_mctrl_mem_we	  : std_logic_vector(3 downto 0);
	signal R_loc_mctrl_mem_state	  : mem_state_type;

	-- Register Bank
	signal R_regs_rs1_idx	: std_logic_vector(4 downto 0);
	signal R_regs_rs1_value	: std_logic_vector(31 downto 0);
	signal R_regs_rs2_idx	: std_logic_vector(4 downto 0);
	signal R_regs_rs2_value	: std_logic_vector(31 downto 0);
	signal R_regs_rd_idx	: std_logic_vector(4 downto 0);
	signal R_regs_rd_value	: std_logic_vector(31 downto 0);

	signal R_loc_regs_write_enable   : std_logic;
	signal R_loc_regs_write_enable_l : std_logic;
	signal R_loc_regs_write_enable_u : std_logic;
	signal R_loc_regs_data_rs1_l     : std_logic_vector(31 downto 0);
	signal R_loc_regs_data_rs1_u     : std_logic_vector(31 downto 0);
	signal R_loc_regs_data_rs2_l     : std_logic_vector(31 downto 0);
	signal R_loc_regs_data_rs2_u     : std_logic_vector(31 downto 0);
	signal R_loc_regs_data_rs1       : std_logic_vector(31 downto 0);
	signal R_loc_regs_data_rs2       : std_logic_vector(31 downto 0);

	-- Internal RAM
	signal R_ram_address	  : std_logic_vector(31 downto 2);
	signal R_ram_write_enable : std_logic_vector(3 downto 0);
	signal R_ram_data_write	  : std_logic_vector(31 downto 0);
	signal R_ram_data_read	  : std_logic_vector(31 downto 0);

	signal R_loc_ram_enable   : std_logic;

begin

----------------------------------------------------------------------------------------------------------
-- Debug Interface
----------------------------------------------------------------------------------------------------------

	with I_debug_select select
		O_debug_data <= R_instruction	 when "00",
		                R_pc_next & "00" when "01",
		                R_alu_res	 when "10",
		                R_ram_data_read  when others;

	O_debug_flags <= R_pc_enable_decoder
			& R_mctrl_cpu_pause
			& R_alu_res(1 downto 0)
			& R_loc_mctrl_mem_we
			;

----------------------------------------------------------------------------------------------------------
-- Program Counter
----------------------------------------------------------------------------------------------------------

	pc_proc: process (I_clk, R_mctrl_cpu_pause, R_take_branch, R_pc_source, R_alu_res, R_loc_pc_current) is
		variable pc_next_var : std_logic_vector(31 downto 2);
	begin
		-- Default case. increment he program counter by four
	        -- lowest two bits are always 0 due to allignment to 32 bits
		pc_next_var := unsigned(R_loc_pc_current) + 1;

		-- Alternative sources for the program counter
		if (R_pc_source = PC_FROM_ALU) and (R_take_branch = '1') then
			pc_next_var := R_alu_res(31 downto 2);
		end if;

		-- While the CPU is paused stay on the current instr. address
		if R_mctrl_cpu_pause = '1' then
			pc_next_var := R_loc_pc_current;
		end if;

		if rising_edge(I_clk) then
			-- Take over the new address. instruction has been fetched
			R_loc_pc_current <= pc_next_var;

			-- If a jump occurs suppress execution of the previusly fetched
			-- instruction (delay slot)
			if R_take_branch = '0' then
				R_pc_enable_decoder <= '1';
			else
				R_pc_enable_decoder <= '0';
			end if;

		end if;

		-- The address of the instruction to be fetched next
		R_pc_current <= R_loc_pc_current;
		R_pc_next    <= pc_next_var;

	end process;

----------------------------------------------------------------------------------------------------------
-- Memory controller
----------------------------------------------------------------------------------------------------------

	mctrl_proc: process (I_clk, R_pc_next, R_mem_source, R_ram_data_read, R_regs_rs2_value, R_alu_res,
	       		     R_loc_mctrl_mem_state, R_loc_mctrl_instruction, R_loc_mctrl_instruction_next,
			     R_loc_mctrl_address, R_loc_mctrl_mem_we) is

		variable address_next_var      : std_logic_vector(31 downto 2);
		variable ram_read_var	       : std_logic_vector(31 downto 0);
		variable ram_write_var	       : std_logic_vector(31 downto 0);
		variable instruction_next_var  : std_logic_vector(31 downto 0);
		variable we_next_var	       : std_logic_vector(3 downto 0);
		variable mem_state_next_var    : mem_state_type;
		variable pause_var	       : std_logic;
	begin
		we_next_var	      := "0000";
		pause_var	      := '0';
		ram_read_var	      := ZEROS;
		ram_write_var	      := ZEROS;
		mem_state_next_var    := R_loc_mctrl_mem_state;
		instruction_next_var  := R_loc_mctrl_instruction;
		address_next_var      := R_pc_next;

		case R_mem_source is

			-- read operations

			when MEM_ACCESS_READ_INT8 | MEM_ACCESS_READ_UINT8 =>

				case R_alu_res(1 downto 0) is
					when "11" =>
						ram_read_var(7 downto 0) := R_ram_data_read(31 downto 24);
					when "10" =>
						ram_read_var(7 downto 0) := R_ram_data_read(23 downto 16);
					when "01" =>
						ram_read_var(7 downto 0) := R_ram_data_read(15 downto 8);
					when others =>
						ram_read_var(7 downto 0) := R_ram_data_read(7 downto 0);
				end case;

				if R_mem_source = MEM_ACCESS_READ_UINT8 or ram_read_var(7) = '0' then
					ram_read_var(31 downto 8) := ZEROS(31 downto 8);
				else
					ram_read_var(31 downto 8) := ONES(31 downto 8);
				end if;

			when MEM_ACCESS_READ_INT16 | MEM_ACCESS_READ_UINT16 =>

				if R_alu_res(1) = '1' then
					ram_read_var(15 downto 0) := R_ram_data_read(31 downto 16);
				else
					ram_read_var(15 downto 0) := R_ram_data_read(15 downto 0);
				end if;

				if R_mem_source = MEM_ACCESS_READ_UINT16 or ram_read_var(15) = '0' then
					ram_read_var(31 downto 16) := ZEROS(31 downto 16);
				else
					ram_read_var(31 downto 16) := ONES(31 downto 16);
				end if;

			when MEM_ACCESS_READ_INT32 =>
				ram_read_var := R_ram_data_read;

			-- write operations

			when MEM_ACCESS_WRITE_INT8 =>
				ram_write_var :=  R_regs_rs2_value(7 downto 0)
			       			& R_regs_rs2_value(7 downto 0)
					       	& R_regs_rs2_value(7 downto 0)
					       	& R_regs_rs2_value(7 downto 0);

				case R_alu_res(1 downto 0) is
					when "00" =>
						we_next_var := "1000";
					when "01" =>
						we_next_var := "0100";
					when "10" =>
						we_next_var := "0010";
					when others =>
						we_next_var := "0001";
				end case;

			when MEM_ACCESS_WRITE_INT16 =>
				ram_write_var := R_regs_rs2_value(15 downto 0) & R_regs_rs2_value(15 downto 0);
				if R_alu_res(1) = '1' then
					we_next_var := "0011";
				else
					we_next_var := "1100";
				end if;

			when MEM_ACCESS_WRITE_INT32 =>
				ram_write_var := R_regs_rs2_value;
				we_next_var := "1111";

			when others =>
		end case;

	       	-- Instruction fetch
		if R_mem_source = MEM_ACCESS_FETCH then
			address_next_var	:= R_pc_next;
			mem_state_next_var	:= MEM_STATE_ADDR;
			instruction_next_var	:= R_ram_data_read;
		else
			-- Read or write access to memory
			if R_loc_mctrl_mem_state = MEM_STATE_ADDR then
				address_next_var	:= R_alu_res(31 downto 2);
				mem_state_next_var	:= MEM_STATE_ACCESS;
				pause_var		:= '1';
				R_loc_mctrl_instruction_next <= R_ram_data_read;
			else
				instruction_next_var	:= R_loc_mctrl_instruction_next;
				address_next_var	:= R_pc_next;
				mem_state_next_var	:= MEM_STATE_ADDR;
				we_next_var		:= "0000";
			end if;
		end if;

		if rising_edge(I_clk) then
			R_loc_mctrl_address      <= address_next_var;
			R_loc_mctrl_instruction  <= instruction_next_var;
			R_loc_mctrl_mem_state    <= mem_state_next_var;
			R_loc_mctrl_mem_we	 <= we_next_var;
		end if;

		R_mctrl_cpu_pause  <= pause_var;

		R_instruction	   <= R_loc_mctrl_instruction;

		R_ram_address	   <= address_next_var(31 downto 2);
		R_ram_read_val	   <= ram_read_var;
		R_ram_data_write   <= ram_write_var;
		R_ram_write_enable <= we_next_var;

		O_mem_address	   <= R_loc_mctrl_address(31 downto 2);
		O_mem_we 	   <= R_loc_mctrl_mem_we;
		O_mem_data_write   <= ram_write_var;
	end process;

----------------------------------------------------------------------------------------------------------
-- Instruction Decoder
----------------------------------------------------------------------------------------------------------

	decoder_proc: process (R_instruction, R_alu_res, R_regs_rs1_value, R_regs_rs2_value, R_pc_current,
	       		       R_ram_read_val, R_pc_enable_decoder) is
		variable opcode_var          : std_logic_vector(6 downto 0);
		variable rd_idx_var          : std_logic_vector(4 downto 0);
		variable funct3_var          : std_logic_vector(2 downto 0);
		variable rs1_idx_var         : std_logic_vector(4 downto 0);
		variable rs2_idx_var         : std_logic_vector(4 downto 0);
		variable funct7_var          : std_logic_vector(6 downto 0);
		variable imm12_type_i_var    : std_logic_vector(11 downto 0);
		variable imm12_type_s_var    : std_logic_vector(11 downto 0);
		variable imm13_type_b_var    : std_logic_vector(12 downto 0);
		variable imm20_type_u_var    : std_logic_vector(19 downto 0);
		variable imm21_type_j_var    : std_logic_vector(20 downto 0);
		variable shamt_var           : std_logic_vector(4 downto 0);
		variable pc_current_var      : std_logic_vector(31 downto 2);

		variable alu_function_var    : alu_function_type;
		variable alu_a_from_var      : alu_a_from_type;
		variable alu_b_from_var      : alu_b_from_type;
		variable rd_val_from_var     : rd_val_new_from_type;
		variable pc_source_var       : pc_source_type;
		variable branch_function_var : branch_function_type;
		variable mem_source_var      : mem_source_type;
	begin
		alu_function_var	:= ALU_FUNC_NONE;
		alu_a_from_var		:= ALU_A_FROM_RS1;
		alu_b_from_var		:= ALU_B_FROM_RS2;
		rd_val_from_var		:= RD_FROM_NULL;
		pc_source_var		:= PC_FROM_INC4;
		branch_function_var	:= BRANCH_NONE;
		mem_source_var		:= MEM_ACCESS_FETCH;

		if R_pc_enable_decoder = '1' then
			opcode_var  := R_instruction(6 downto 0);
			rd_idx_var  := R_instruction(11 downto 7);
			funct3_var  := R_instruction(14 downto 12);
			rs1_idx_var := R_instruction(19 downto 15);
			rs2_idx_var := R_instruction(24 downto 20);
			funct7_var  := R_instruction(31 downto 25);

			imm12_type_i_var := R_instruction(31 downto 20);

			imm12_type_s_var := R_instruction(31 downto 25)
					  & R_instruction(11 downto 7);

			imm13_type_b_var := R_instruction(31)
		       			  & R_instruction(7)
					  & R_instruction(30 downto 25)
					  & R_instruction(11 downto 8)
					  & '0';

			imm20_type_u_var := R_instruction(31 downto 12);

			imm21_type_j_var := R_instruction(31)
		       			  & R_instruction(19 downto 12)
					  & R_instruction(20)
					  & R_instruction(30 downto 21)
					  & '0';

			shamt_var := R_instruction(24 downto 20);

 			pc_current_var   := unsigned(R_pc_current) - 1;

--			is_ebreak := R_instruction(20);
		else
			opcode_var       := "0010011"; -- NOP (ADDI zero,zero,0)
			rd_idx_var       := "00000";
			funct3_var       := "000";
			rs1_idx_var      := "00000";
			rs2_idx_var      := "00000";
			funct7_var       := "0000000";
			imm12_type_i_var := "000000000000";
			imm12_type_s_var := "000000000000";
			imm13_type_b_var := "0000000000000";
			imm20_type_u_var := "00000000000000000000";
			imm21_type_j_var := "000000000000000000000";
			shamt_var	 := "00000";

			pc_current_var   := R_pc_current;
		end if;

		case opcode_var is
			-- LUI		U	r[rd] = imm20_type_u_var << 12 | 0x000
			when "0110111" =>
				rd_val_from_var		:= RD_FROM_IMM20_U_SLL12;

			-- AUIPC	U	r[rd] = pc_current + (imm20_type_u_var << 12)
			when "0010111" =>
				alu_a_from_var		:= ALU_A_FROM_PC;
				alu_b_from_var		:= ALU_B_FROM_IMM20_U_SLL12;
				alu_function_var	:= ALU_FUNC_ADD;
				rd_val_from_var		:= RD_FROM_ALU;

			-- JAL		J	r[rd] = pc_next; pc_next = pc_current + imm21_type_j_var
			when "1101111" =>
				alu_a_from_var		:= ALU_A_FROM_PC;
				alu_b_from_var		:= ALU_B_FROM_IMM21_J;
				alu_function_var	:= ALU_FUNC_ADD;
				rd_val_from_var		:= RD_FROM_PC;
				pc_source_var		:= PC_FROM_ALU;
				branch_function_var	:= BRANCH_UNCONDITIONAL;

			-- JALR		I	r[rd] = pc_next; pc_next = r[rs1] + imm12_type_i_var
			when "1100111" =>
				alu_b_from_var		:= ALU_B_FROM_IMM12_I;
				alu_function_var	:= ALU_FUNC_ADD;
				rd_val_from_var		:= RD_FROM_ALU;
				pc_source_var		:= PC_FROM_ALU;
				branch_function_var	:= BRANCH_UNCONDITIONAL;

			-- Branch instructions
			when "1100011" =>
				alu_a_from_var	:= ALU_A_FROM_PC;
				alu_b_from_var	:= ALU_B_FROM_IMM13_B;
				alu_function_var:= ALU_FUNC_ADD;
				pc_source_var	:= PC_FROM_ALU;

				case funct3_var is
					-- BEQ		B	if (r[rs1] == r[rs2]) pc_next = pc_current + imm13_type_b_var
					when "000" =>
						branch_function_var := BRANCH_IF_EQ;
					-- BNE		B	if (r[rs1] != r[rs2]) pc_next = pc_current + imm13_type_b_var
					when "001" =>
						branch_function_var := BRANCH_IF_NE;
					-- BLT		B	if ((int32_t)r[rs1] < (int32_t)r[rs2]) pc_next = pc_current + imm13_type_b_var
					when "100" =>
						branch_function_var := BRANCH_IF_LT;
					-- BGE		B	if ((int32_t)r[rs1] >= (int32_t)r[rs2]) pc_next = pc_current + imm13_type_b_var
					when "101" =>
						branch_function_var := BRANCH_IF_GE;
					-- BLTU		B	if (r[rs1] < r[rs2]) pc_next = pc_current + imm13_type_b_var
					when "110" =>
						branch_function_var := BRANCH_IF_LTU;
					-- BGEU		B	if (r[rs1] >= r[rs2]) pc_next = pc_current + imm13_type_b_var
					when "111" =>
						branch_function_var := BRANCH_IF_GEU;
					-- NOP
					when others =>
				end case;

			-- Load instructions
			when "0000011" =>
				alu_b_from_var	:= ALU_B_FROM_IMM12_I;
				alu_function_var:= ALU_FUNC_ADD;
				rd_val_from_var := RD_FROM_MEMORY;

				case funct3_var is
					-- LB		I	r[rd] = (int8_t)mem[r[rs1] + imm12_type_i_var]
					when "000" =>
						mem_source_var := MEM_ACCESS_READ_INT8;
					-- LH		I	r[rd] = (int16_t)mem[r[rs1] + imm12_type_i_var]
					when "001" =>
						mem_source_var := MEM_ACCESS_READ_INT16;
					-- LW		I	r[rd] = (int32_t)mem[r[rs1] + imm12_type_i_var]
					when "010" =>
						mem_source_var := MEM_ACCESS_READ_INT32;
					-- LBU		I	r[rd] = (uint8_t)mem[r[rs1] + imm12_type_i_var]
					when "100" =>
						mem_source_var := MEM_ACCESS_READ_UINT8;
					-- LHU		I	r[rd] = (uint16_t)mem[r[rs1] + imm12_type_i_var]
					when "101" =>
						mem_source_var := MEM_ACCESS_READ_UINT16;
					-- NOP
					when others =>
				end case;

			-- Store instructions
			when "0100011" =>
				alu_b_from_var	:= ALU_B_FROM_IMM12_S;
				alu_function_var:= ALU_FUNC_ADD;

				case funct3_var is
					-- SB		S	mem[r[rs1] + imm12_type_s_var] = r[rs2]
					when "000" =>
						mem_source_var := MEM_ACCESS_WRITE_INT8;
					-- SH		S	mem[r[rs1] + imm12_type_s_var] = r[rs2]
					when "001" =>
						mem_source_var := MEM_ACCESS_WRITE_INT16;
					-- SW		S	mem[r[rs1] + imm12_type_s_var] = r[rs2]
					when "010" =>
						mem_source_var := MEM_ACCESS_WRITE_INT32;
					-- NOP
					when others =>
				end case;

			-- Arithmetic and logic operations with immediates
			when "0010011" =>
				alu_b_from_var	:= ALU_B_FROM_IMM12_I;
				rd_val_from_var := RD_FROM_ALU;

				case funct3_var is
					-- ADDI		I	r[rd] = r[rs1] + imm12_type_i_var
					when "000" =>
						alu_function_var := ALU_FUNC_ADD;
					-- SLTI		I	r[rd] = r[rs1] < imm12_type_i_var
					when "010" =>
						alu_function_var := ALU_FUNC_SLT;
					-- SLTIU	I	r[rd] = r[rs1] < (uint32_t)imm12_type_i_var
					when "011" =>
						alu_function_var := ALU_FUNC_SLTU;
					-- XORI		I	r[rd] = r[rs1] ^ imm12_type_i_var
					when "100" =>
						alu_function_var := ALU_FUNC_XOR;
					-- ORI		I	r[rd] = r[rs1] | imm12_type_i_var
					when "110" =>
						alu_function_var := ALU_FUNC_OR;
					-- ANDI		I	r[rd] = r[rs1] & imm12_type_i_var
					when "111" =>
						alu_function_var := ALU_FUNC_AND;

					-- SLLI		X	r[rd] = r[rs1] << shamt
					when "001" =>
						alu_b_from_var	:= ALU_B_FROM_SHAMT;
						alu_function_var := ALU_FUNC_SLL;
					-- SRLI or SRAI
					when "101" =>

						alu_b_from_var	:= ALU_B_FROM_SHAMT;

						if funct7_var = "0000000" then
							-- SRLI		X	r[rd] = r[rs1] >> shamt
							alu_function_var := ALU_FUNC_SRL;
						else
							-- SRAI		X	r[rd] = (int32_t)r[rs1] >> shamt
							alu_function_var := ALU_FUNC_SRA;
						end if;
					-- NOP
					when others =>
				end case;

			-- Arithmetic and logic operations
			when "0110011" =>
				rd_val_from_var := RD_FROM_ALU;

				case funct3_var is
					-- ADD or SUB
					when "000" =>
						if funct7_var = "0000000" then
							-- ADD         R	r[rd] = r[rs1] + r[rs2]
							alu_function_var := ALU_FUNC_ADD;
						else
							-- SUB         R	r[rd] = r[rs1] - r[rs2]
							alu_function_var := ALU_FUNC_SUB;
						end if;
					-- SLL		R	r[rd] = r[rs1] << (r[rs2] & 31)
					when "001" =>
						alu_function_var := ALU_FUNC_SLL;
					-- SLT		R	r[rd] = (int32_t)r[rs1] < (int32_t)r[rs2];
					when "010" =>
						alu_function_var := ALU_FUNC_SLT;
					-- SLTU		R	r[rd] = (uint32_t)r[rs1] < (uint32_t)r[rs2]
					when "011" =>
						alu_function_var := ALU_FUNC_SLTU;
					-- XOR		R	r[rd] = r[rs1] ^ r[rs2]
					when "100" =>
						alu_function_var := ALU_FUNC_XOR;
					-- SRL or SRA
					when "101" =>
						if funct7_var = "0000000" then
							-- SRL		R	r[rd] = r[rs1] >> (r[rs2] & 31)
							alu_function_var := ALU_FUNC_SRL;
						else
							-- SRA		R	r[rd] = r[rs1] >> (r[rs2] & 31)
							alu_function_var :=  ALU_FUNC_SRA;
						end if;
					-- OR		R	r[rd] = r[rs1] | r[rs2]
					when "110" =>
						alu_function_var := ALU_FUNC_OR;
					-- AND		R	r[rd] = r[rs1] & r[rs2]
					when "111" =>
						alu_function_var := ALU_FUNC_AND;
					-- NOP
					when others =>
				end case;

			-- FENCE	Y
			-- when "0001111" =>
				-- ignore i.e. NOP

			-- ECALL or EBREAK
			-- when "1110011" =>
				-- if is_ebreak = '1'  then
					-- EBREAK	Z
					-- is_interrupt_var	:= '1';
					-- interrupt_source_var	:= EBREAK;
				-- else
					-- ECALL	Z
					-- is_interrupt_var	:= '1';
					-- interrupt_source_var	:= ECALL;
				-- end if;
			-- NOP
			when others =>
		end case;

		-- Do not write anything to the destination register 0
		if (rd_val_from_var = RD_FROM_NULL) then
			rd_idx_var := "00000";
		end if;

		R_regs_rs1_idx    <= rs1_idx_var;
		R_regs_rs2_idx    <= rs2_idx_var;
		R_regs_rd_idx     <= rd_idx_var;
		R_alu_func   <= alu_function_var;
		R_pc_source  <= pc_source_var;
		R_mem_source <= mem_source_var;

		-- Determine source for ALU input A
		case alu_a_from_var is
			when ALU_A_FROM_RS1 =>
				R_alu_a <= R_regs_rs1_value;
			when ALU_A_FROM_PC =>
				R_alu_a <= pc_current_var & "00";
			when others =>
		end case;

		-- Determine source for ALU input B
		case alu_b_from_var is
			when ALU_B_FROM_RS2 =>
				R_alu_b <= R_regs_rs2_value;
			when ALU_B_FROM_IMM20_U_SLL12 =>
				R_alu_b <= imm20_type_u_var & ZEROS(11 downto 0);
			when ALU_B_FROM_IMM21_J =>
				if imm21_type_j_var(20) = '1' then
					R_alu_b <= ONES(31 downto 21) & imm21_type_j_var;
				else
					R_alu_b <= ZEROS(31 downto 21) & imm21_type_j_var;
				end if;
			when ALU_B_FROM_IMM12_I =>
				if imm12_type_i_var(11) = '1' then
					R_alu_b <= ONES(31 downto 12) & imm12_type_i_var;
				else
					R_alu_b <= ZEROS(31 downto 12) & imm12_type_i_var;
				end if;
			when ALU_B_FROM_IMM13_B =>
				if imm13_type_b_var(12) = '1' then
					R_alu_b <= ONES(31 downto 13) & imm13_type_b_var;
				else
					R_alu_b <= ZEROS(31 downto 13) & imm13_type_b_var;
				end if;
			when ALU_B_FROM_IMM12_S =>
				if imm12_type_s_var(11) = '1' then
					R_alu_b <= ONES(31 downto 12) & imm12_type_s_var;
				else
					R_alu_b <= ZEROS(31 downto 12) & imm12_type_s_var;
				end if;
			when ALU_B_FROM_SHAMT =>
				R_alu_b <= ZEROS(31 downto 5) & shamt_var;
			when others =>
				R_alu_b <= R_regs_rs2_value;
		end case;

		-- Determine what will be written to the destination register, if anything
		case rd_val_from_var is
			when RD_FROM_ALU =>
				R_regs_rd_value <= R_alu_res;
			when RD_FROM_PC =>
				R_regs_rd_value <= R_pc_current & "00";
			when RD_FROM_MEMORY =>
				R_regs_rd_value <= R_ram_read_val;
			when RD_FROM_IMM20_U_SLL12 =>
				R_regs_rd_value <= imm20_type_u_var & ZEROS(11 downto 0);
			when others =>
				R_regs_rd_value <= R_alu_res;
		end case;

		-- Check conditions to see if a branch shall be taken or not
		case branch_function_var is
			when BRANCH_IF_EQ =>
				if signed(R_regs_rs1_value) = signed(R_regs_rs2_value) then
					R_take_branch <= '1';
				else
					R_take_branch <= '0';
				end if;
			when BRANCH_IF_NE =>
				if signed(R_regs_rs1_value) /= signed(R_regs_rs2_value) then
					R_take_branch <= '1';
				else
					R_take_branch <= '0';
				end if;
			when BRANCH_IF_LT =>
				if signed(R_regs_rs1_value) < signed(R_regs_rs2_value) then
					R_take_branch <= '1';
				else
					R_take_branch <= '0';
				end if;
			when BRANCH_IF_GE =>
				if signed(R_regs_rs1_value) >= signed(R_regs_rs2_value) then
					R_take_branch <= '1';
				else
					R_take_branch <= '0';
				end if;
			when BRANCH_IF_LTU =>
				if unsigned(R_regs_rs1_value) < unsigned(R_regs_rs2_value) then
					R_take_branch <= '1';
				else
					R_take_branch <= '0';
				end if;
			when BRANCH_IF_GEU =>
				if unsigned(R_regs_rs1_value) >= unsigned(R_regs_rs2_value) then
					R_take_branch <= '1';
				else
					R_take_branch <= '0';
				end if;
			when BRANCH_UNCONDITIONAL =>
				R_take_branch <= '1';
			when others =>
				R_take_branch <= '0';
		end case;
	end process;

----------------------------------------------------------------------------------------------------------
-- ALU
----------------------------------------------------------------------------------------------------------

	-- MULT18X18: 18 x 18 signed asynchronous multiplier
	-- Expand alu-in A and B to 18 bit values so they can be fed to the HW multiplier
	R_loc_alu_fac_a <= R_alu_a(17 downto 0);
	R_loc_alu_fac_b <= R_alu_b(17 downto 0);
	MULT18X18_inst : MULT18X18
	port map (
		 P => R_loc_alu_prod  -- 36-bit multiplier output
		,A => R_loc_alu_fac_a -- 18-bit multiplier input
		,B => R_loc_alu_fac_b -- 18-bit multiplier input
		);

	R_loc_alu_sll <= To_BitVector(R_alu_a) sll Conv_Integer(unsigned(R_alu_b(4 downto 0)));
	R_loc_alu_srl <= To_BitVector(R_alu_a) srl Conv_Integer(unsigned(R_alu_b(4 downto 0)));
	R_loc_alu_sra <= To_BitVector(R_alu_a) sra Conv_Integer(unsigned(R_alu_b(4 downto 0)));

	R_loc_alu_sltu <= ZEROS(31 downto 1) & '1' when unsigned(R_alu_a) < unsigned (R_alu_b) else
	 	  ZEROS(31 downto 1) & '0';

	R_loc_alu_slt <= ZEROS(31 downto 1) & '1' when signed(R_alu_a) < signed (R_alu_b) else
		 ZEROS(31 downto 1) & '0';

	R_loc_alu_sum <= signed(R_alu_a) + signed(R_alu_b);
	R_loc_alu_dif <= signed(R_alu_a) - signed(R_alu_b);

	-- Perform varios ALU operations relying on the syntesis tool correctly inferring the OP
	R_alu_res <=
		std_logic_vector(R_loc_alu_sum) when R_alu_func = ALU_FUNC_ADD  else
		std_logic_vector(R_loc_alu_dif) when R_alu_func = ALU_FUNC_SUB  else
		R_loc_alu_sltu			when R_alu_func = ALU_FUNC_SLTU else
		R_loc_alu_slt			when R_alu_func = ALU_FUNC_SLT  else
		R_loc_alu_prod(31 downto 0)	when R_alu_func = ALU_FUNC_MUL  else
		R_alu_a xor R_alu_b		when R_alu_func = ALU_FUNC_XOR  else
		R_alu_a or  R_alu_b		when R_alu_func = ALU_FUNC_OR   else
		R_alu_a and R_alu_b		when R_alu_func = ALU_FUNC_AND  else
		To_StdLogicVector(R_loc_alu_sll)when R_alu_func = ALU_FUNC_SLL  else
		To_StdLogicVector(R_loc_alu_srl)when R_alu_func = ALU_FUNC_SRL  else
		To_StdLogicVector(R_loc_alu_sra)when R_alu_func = ALU_FUNC_SRA  else
		ZEROS;

----------------------------------------------------------------------------------------------------------
-- Register Bank
----------------------------------------------------------------------------------------------------------

	-- register 0 is never written to
	R_loc_regs_write_enable <= '1' when ((R_regs_rd_idx /= "00000") and (R_mctrl_cpu_pause = '0')) else '0';

	--lower 16 registers
	R_loc_regs_write_enable_l <= R_loc_regs_write_enable and not R_regs_rd_idx(4);

	--upper 16 registers
	R_loc_regs_write_enable_u <= R_loc_regs_write_enable and R_regs_rd_idx(4);

	-- 32 registers
	reg_loop: for i in 0 to 31 generate
	begin
		-- RAM16X1D: 16 x 1 positive edge write, asynchronous read dual-port
		-- distributed RAM for all Xilinx FPGAs

		-- Port 1 lower 16 registers
		p1_l : RAM16X1D
		port map (
			 WCLK  => I_clk,            -- Port A write clock input
			 WE    => R_loc_regs_write_enable_l, -- Port A write enable input
			 A0    => R_regs_rd_idx(0),      -- Port A address[0] input bit
			 A1    => R_regs_rd_idx(1),      -- Port A address[1] input bit
			 A2    => R_regs_rd_idx(2),      -- Port A address[2] input bit
			 A3    => R_regs_rd_idx(3),      -- Port A address[3] input bit
			 D     => R_regs_rd_value(i),    -- Port A 1-bit data input
			 DPRA0 => R_regs_rs1_idx(0),     -- Port B address[0] input bit
			 DPRA1 => R_regs_rs1_idx(1),     -- Port B address[1] input bit
			 DPRA2 => R_regs_rs1_idx(2),     -- Port B address[2] input bit
			 DPRA3 => R_regs_rs1_idx(3),     -- Port B address[3] input bit
			 DPO   => R_loc_regs_data_rs1_l(i),  -- Port B 1-bit data output
			 SPO   => open              -- Port A 1-bit data output
		);

		-- Port 1 upper 16 registers
		p1_u : RAM16X1D
		port map (
			 WCLK  => I_clk,
			 WE    => R_loc_regs_write_enable_u,
			 A0    => R_regs_rd_idx(0),
			 A1    => R_regs_rd_idx(1),
			 A2    => R_regs_rd_idx(2),
			 A3    => R_regs_rd_idx(3),
			 D     => R_regs_rd_value(i),
			 DPRA0 => R_regs_rs1_idx(0),
			 DPRA1 => R_regs_rs1_idx(1),
			 DPRA2 => R_regs_rs1_idx(2),
			 DPRA3 => R_regs_rs1_idx(3),
			 DPO   => R_loc_regs_data_rs1_u(i),
			 SPO   => open
		);

		-- Port 2 lower 16 registers
		p2_l : RAM16X1D
		port map (
			 WCLK  => I_clk,
			 WE    => R_loc_regs_write_enable_l,
			 A0    => R_regs_rd_idx(0),
			 A1    => R_regs_rd_idx(1),
			 A2    => R_regs_rd_idx(2),
			 A3    => R_regs_rd_idx(3),
			 D     => R_regs_rd_value(i),
			 DPRA0 => R_regs_rs2_idx(0),
			 DPRA1 => R_regs_rs2_idx(1),
			 DPRA2 => R_regs_rs2_idx(2),
			 DPRA3 => R_regs_rs2_idx(3),
			 DPO   => R_loc_regs_data_rs2_l(i),
			 SPO   => open
		);

		-- Port 2 upper 16 registers
		p2_u : RAM16X1D
		port map (
			 WCLK  => I_clk,
			 WE    => R_loc_regs_write_enable_u,
			 A0    => R_regs_rd_idx(0),
			 A1    => R_regs_rd_idx(1),
			 A2    => R_regs_rd_idx(2),
			 A3    => R_regs_rd_idx(3),
			 D     => R_regs_rd_value(i),
			 DPRA0 => R_regs_rs2_idx(0),
			 DPRA1 => R_regs_rs2_idx(1),
			 DPRA2 => R_regs_rs2_idx(2),
			 DPRA3 => R_regs_rs2_idx(3),
			 DPO   => R_loc_regs_data_rs2_u(i),
			 SPO   => open
		);
	end generate;

      R_loc_regs_data_rs1 <= R_loc_regs_data_rs1_l when R_regs_rs1_idx(4)='0' else R_loc_regs_data_rs1_u;
      R_loc_regs_data_rs2 <= R_loc_regs_data_rs2_l when R_regs_rs2_idx(4)='0' else R_loc_regs_data_rs2_u;

      -- reading register 0 always returns 0
      R_regs_rs1_value <= ZEROS when R_regs_rs1_idx = "000000" else R_loc_regs_data_rs1;
      R_regs_rs2_value <= ZEROS when R_regs_rs2_idx = "000000" else R_loc_regs_data_rs2;

----------------------------------------------------------------------------------------------------------
-- Random Access Memory
----------------------------------------------------------------------------------------------------------

	-- Only enable internal RAM if the address is below 0x800 (2048 words)
	R_loc_ram_enable <= '1' when R_ram_address(31 downto 11) = "000000000000000000000" else '0';

	bxxx : RAMB16_S9
	generic map (
		INIT_00 => X"231383b31303831393132323232323b723136f2393836713638323136fef136f",
		INIT_01 => X"00000000000000000000000000000004ac00766f48671313038303830383e3ef",
		INIT_02 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_03 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_04 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_05 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_06 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_07 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_08 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_09 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0A => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0B => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0C => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0D => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0E => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0F => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_10 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_11 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_12 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_13 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_14 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_15 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_16 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_17 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_18 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_19 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1A => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1B => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1C => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1D => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1E => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1F => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_20 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_21 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_22 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_23 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_24 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_25 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_26 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_27 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_28 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_29 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2A => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2B => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2C => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2D => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2E => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2F => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_30 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_31 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_32 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_33 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_34 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_35 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_36 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_37 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_38 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_39 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3A => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3B => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3C => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3D => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3E => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3F => X"0000000000000000000000000000000000000000000000000000000000000000")
	port map (
		DO   => R_ram_data_read(7 downto 0),
		DOP  => open,
		ADDR => R_ram_address(12 downto 2),
		CLK  => I_clk,
		DI   => R_ram_data_write(7 downto 0),
		DIP  => ZEROS(0 downto 0),
		EN   => R_loc_ram_enable,
		SSR  => '0',
		WE   => R_ram_write_enable(3)
	);

	xbxx : RAMB16_S9
	generic map (
		INIT_00 => X"0004c7878527270984042e2426282cc42a01f02687278001e6272601f0000100",
		INIT_01 => X"0000000000000000000000000000000000006120658001052a292924242010f0",
		INIT_02 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_03 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_04 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_05 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_06 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_07 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_08 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_09 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0A => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0B => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0C => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0D => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0E => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0F => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_10 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_11 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_12 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_13 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_14 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_15 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_16 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_17 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_18 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_19 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1A => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1B => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1C => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1D => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1E => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1F => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_20 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_21 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_22 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_23 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_24 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_25 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_26 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_27 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_28 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_29 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2A => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2B => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2C => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2D => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2E => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2F => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_30 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_31 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_32 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_33 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_34 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_35 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_36 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_37 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_38 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_39 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3A => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3B => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3C => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3D => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3E => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3F => X"0000000000000000000000000000000000000000000000000000000000000000")
	port map (
		DO   => R_ram_data_read(15 downto 8),
		DOP  => open,
		ADDR => R_ram_address(12 downto 2),
		CLK  => I_clk,
		DI   => R_ram_data_write(15 downto 8),
		DIP  => ZEROS(0 downto 0),
		EN   => R_loc_ram_enable,
		SSR  => '0',
		WE   => R_ram_write_enable(2)
	);

	xxbx : RAMB16_S9
	generic map (
		INIT_00 => X"f71407870400c0c004001141312181be91015ff117c10001a7c101019f00c040",
		INIT_01 => X"0000000000000000000000000000000000006c526c00010081c1014181c1241f",
		INIT_02 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_03 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_04 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_05 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_06 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_07 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_08 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_09 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0A => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0B => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0C => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0D => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0E => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0F => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_10 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_11 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_12 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_13 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_14 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_15 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_16 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_17 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_18 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_19 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1A => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1B => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1C => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1D => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1E => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1F => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_20 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_21 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_22 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_23 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_24 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_25 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_26 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_27 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_28 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_29 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2A => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2B => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2C => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2D => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2E => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2F => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_30 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_31 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_32 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_33 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_34 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_35 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_36 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_37 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_38 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_39 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3A => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3B => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3C => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3D => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3E => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3F => X"0000000000000000000000000000000000000000000000000000000000000000")
	port map (
		DO   => R_ram_data_read(23 downto 16),
		DOP  => open,
		ADDR => R_ram_address(12 downto 2),
		CLK  => I_clk,
		DI   => R_ram_data_write(23 downto 16),
		DIP  => ZEROS(0 downto 0),
		EN   => R_loc_ram_enable,
		SSR  => '0',
		WE   => R_ram_write_enable(1)
	);

	xxxb : RAMB16_S9
	generic map (
		INIT_00 => X"00000000000c0b00c20000010101000000fefe0000000001000000ffff037f00",
		INIT_01 => X"00000000000000000000000000000020000021696c000200000001010101fff9",
		INIT_02 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_03 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_04 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_05 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_06 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_07 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_08 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_09 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0A => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0B => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0C => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0D => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0E => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_0F => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_10 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_11 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_12 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_13 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_14 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_15 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_16 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_17 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_18 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_19 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1A => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1B => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1C => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1D => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1E => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_1F => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_20 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_21 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_22 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_23 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_24 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_25 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_26 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_27 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_28 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_29 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2A => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2B => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2C => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2D => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2E => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_2F => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_30 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_31 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_32 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_33 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_34 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_35 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_36 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_37 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_38 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_39 => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3A => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3B => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3C => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3D => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3E => X"0000000000000000000000000000000000000000000000000000000000000000",
		INIT_3F => X"0000000000000000000000000000000000000000000000000000000000000000")
	port map (
		DO   => R_ram_data_read(31 downto 24),
		DOP  => open,
		ADDR => R_ram_address(12 downto 2),
		CLK  => I_clk,
		DI   => R_ram_data_write(31 downto 24),
		DIP  => ZEROS(0 downto 0),
		EN   => R_loc_ram_enable,
		SSR  => '0',
		WE   => R_ram_write_enable(0)
	);
end;

