/*
 * Rival - A RISC-V RV32I CPU
 *
 * ad.c: assembler / disassembler
 *
 * Copyright (c) 2020, Helmut Sipos <helmut.sipos@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

/* Based on the RISC-V RV32I ISA specification version 20190621-draft */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <getopt.h>
#include <sysexits.h>
#include <sys/stat.h>

#define RESET_VECTOR		0x00000000
#define BUFFER_SIZE		80
#define INVALID_OPCODE		0xFFFFFFFF
#define NR_REGISTERS		32
#define MAX_NR_TOKENS_PER_LINE	6

#define REG_NAMES_CONVENTIONAL	0
#define REG_NAMES_ABI		1

#define MODE_ASSEMBLE		0
#define MODE_DISASSEMBLE	1

/* Representation of an instruction */
typedef struct {
 	/* The mask is used to ignore all but the invariable bits that identify
	 * the instructions. */
	const uint32_t mask;
	const uint32_t value;
	const uint8_t type;
	const char *const name;
} tInstruction;

/* Conventional and ABI names */
static const char *const RegisterNames[][2] = {
	 {"x0", "zero"}
	,{"x1", "ra"}
	,{"x2", "sp"}
	,{"x3", "gp"}
	,{"x4", "tp"}
	,{"x5", "t0"}
	,{"x6", "t1"}
	,{"x7", "t2"}
	,{"x8", "s0"}
	,{"x9", "s1"}
	,{"x10", "a0"}
	,{"x11", "a1"}
	,{"x12", "a2"}
	,{"x13", "a3"}
	,{"x14", "a4"}
	,{"x15", "a5"}
	,{"x16", "a6"}
	,{"x17", "a7"}
	,{"x18", "s2"}
	,{"x19", "s3"}
	,{"x20", "s4"}
	,{"x21", "s5"}
	,{"x22", "s6"}
	,{"x23", "s7"}
	,{"x24", "s8"}
	,{"x25", "s9"}
	,{"x26", "s10"}
	,{"x27", "s11"}
	,{"x28", "t3"}
	,{"x29", "t4"}
	,{"x30", "t5"}
	,{"x31", "t6"}
};

/* Instruction types */
enum {
         TYPE_R, TYPE_I, TYPE_S, TYPE_B, TYPE_U, TYPE_J
        ,TYPE_L, TYPE_X, TYPE_Y, TYPE_Z
};

/* FIXME: re-arrange according to usage frequency.
 * implement unsupported instructions as NOPs: addi r0,r0,r0 */
static const tInstruction Instructions[] = {
	 {0x0000007F, 0x00000037, TYPE_U, "lui"}
	,{0x0000007F, 0x00000017, TYPE_U, "auipc"}
	,{0x0000007F, 0x0000006F, TYPE_J, "jal"}
	,{0x0000707F, 0x00000067, TYPE_I, "jalr"}
	,{0x0000707F, 0x00000063, TYPE_B, "beq"}
	,{0x0000707F, 0x00001063, TYPE_B, "bne"}
	,{0x0000707F, 0x00004063, TYPE_B, "blt"}
	,{0x0000707F, 0x00005063, TYPE_B, "bge"}
	,{0x0000707F, 0x00006063, TYPE_B, "bltu"}
	,{0x0000707F, 0x00007063, TYPE_B, "bgeu"}
	,{0x0000707F, 0x00000003, TYPE_L, "lb"}
	,{0x0000707F, 0x00001003, TYPE_L, "lh"}
	,{0x0000707F, 0x00002003, TYPE_L, "lw"}
	,{0x0000707F, 0x00004003, TYPE_L, "lbu"}
	,{0x0000707F, 0x00005003, TYPE_L, "lhu"}
	,{0x0000707F, 0x00000023, TYPE_S, "sb"}
	,{0x0000707F, 0x00001023, TYPE_S, "sh"}
	,{0x0000707F, 0x00002023, TYPE_S, "sw"}
	,{0x0000707F, 0x00000013, TYPE_I, "addi"}
	,{0x0000707F, 0x00002013, TYPE_I, "slti"}
	,{0x0000707F, 0x00003013, TYPE_I, "sltiu"}
	,{0x0000707F, 0x00004013, TYPE_I, "xori"}
	,{0x0000707F, 0x00006013, TYPE_I, "ori"}
	,{0x0000707F, 0x00007013, TYPE_I, "andi"}
	,{0xFE00707F, 0x00001013, TYPE_X, "slli"}
	,{0xFE00707F, 0x00005013, TYPE_X, "srli"}
	,{0xFE00707F, 0x40005013, TYPE_X, "srai"}
	,{0xFE00707F, 0x00000033, TYPE_R, "add"}
	,{0xFE00707F, 0x40000033, TYPE_R, "sub"}
	,{0xFE00707F, 0x00001033, TYPE_R, "sll"}
	,{0xFE00707F, 0x00002033, TYPE_R, "slt"}
	,{0xFE00707F, 0x00003033, TYPE_R, "sltu"}
	,{0xFE00707F, 0x00004033, TYPE_R, "xor"}
	,{0xFE00707F, 0x00005033, TYPE_R, "srl"}
	,{0xFE00707F, 0x40005033, TYPE_R, "sra"}
	,{0xFE00707F, 0x00006033, TYPE_R, "or"}
	,{0xFE00707F, 0x00007033, TYPE_R, "and"}
	,{0x0000707F, 0x0000000F, TYPE_Y, "fence"}
	,{0xFFFFFFFF, 0x00000073, TYPE_Z, "ecall"}
	,{0xFFFFFFFF, 0x00100073, TYPE_Z, "ebreak"}
};

#define NR_INSTRUCTIONS (sizeof (Instructions) / sizeof (tInstruction))

/* Transform a numerical value into a human readable instruction (disassemble) */
static void disas (uint32_t address, uint32_t opcode, char *line, uint8_t reg_names_selector)
{
	int16_t imm12_type_i, imm12_type_s, imm13_type_b;
	int32_t imm21_type_j;
	uint32_t imm20_type_u;
	uint8_t rd, rs1, rs2, shamt;
	uint32_t i;
	char const* rd_str = NULL;
	char const* rs1_str = NULL;
	char const* rs2_str = NULL;

	rd          = (opcode >> 7) & 0x1F;
	rs1         = (opcode >> 15) & 0x1F;
	rs2         = (opcode >> 20) & 0x1F;
        shamt       = (opcode >> 20) & 0x1F;

        imm12_type_i = (int16_t)(
			(opcode >> 20) & 0xFFF
			) << 4;
        imm12_type_i >>= 4;

	imm12_type_s = (int16_t)(
			  (((opcode >> 25) & 0x7F) << 5)
			| ((opcode >> 7) & 0x1F)
			) << 4;
        imm12_type_s >>= 4;

	imm13_type_b = (int16_t)(
			  (((opcode >> 31) & 0x1) << 12)
			| (((opcode >> 7) & 0x1) << 11)
			| (((opcode >> 25) & 0x7F) << 5)
			| (((opcode >> 8) & 0xF)) <<1
			) << 3;
        imm13_type_b >>= 3;

        imm20_type_u = (uint32_t)((opcode >> 12) & 0x000FFFFF);

        imm21_type_j = (int32_t)(
			  (((opcode >> 31) & 0x1) << 20)
			| (((opcode >> 21) & 0x3FF) << 1)
			| (((opcode >> 20) & 0x1) << 11)
			| (((opcode >> 12) & 0xFF) << 12)
			) << 11;
        imm21_type_j >>= 11;

	rd_str = RegisterNames[rd][reg_names_selector];
	rs1_str = RegisterNames[rs1][reg_names_selector];
	rs2_str = RegisterNames[rs2][reg_names_selector];

	/* Print the address of the instruction */
	sprintf (&line[0], "%x:", address);

	for (i = 0; i < NR_INSTRUCTIONS; i++) {
		if ((opcode & Instructions[i].mask) == Instructions[i].value) {

			/* Print the raw opcode */
			sprintf (&line[13], "%x",opcode);

			/* Print the instruction */
			sprintf (&line[25], "%s",Instructions[i].name);

			/* Print the parameters according to each instruction */
			switch (Instructions[i].type) {
			case TYPE_R:
				sprintf (&line[35], "%s, %s, %s", rd_str, rs1_str, rs2_str);
				break;
			case TYPE_I:
				sprintf (&line[35], "%s, %s, %d", rd_str, rs1_str, imm12_type_i);
				break;
			case TYPE_L:
				sprintf (&line[35], "%s, [%s + %d]", rd_str, rs1_str, imm12_type_i);
				break;
			case TYPE_S:
				sprintf (&line[35], "[%s + %d], %s", rs1_str, imm12_type_s, rs2_str);
				break;
			case TYPE_B:
				sprintf (&line[35], "%s, %s, 0x%x", rs1_str, rs2_str, address + imm13_type_b);
				break;
			case TYPE_U:
				sprintf (&line[35], "%s, 0x%x", rd_str, imm20_type_u);
				break;
			case TYPE_J:
				sprintf (&line[35], "%s, 0x%x", rd_str, address + imm21_type_j);
				break;
			case TYPE_X:
				sprintf (&line[35], "%s, %s, 0x%x", rd_str, rs1_str, shamt);
				break;
			case TYPE_Y:
			case TYPE_Z:
				/* No paramters */
				break;
			default:
				break;
			}
			break;
		}
	}

	if (i == NR_INSTRUCTIONS)
		sprintf (&line[0], "Cannot decode: 0x%x\n", opcode);
}

/* Helper function to get a value parameter */
static int32_t getvalue (const char *const str)
{
	int32_t val = 0x0;
	uint8_t base = 10;
	uint8_t offset = 0;

	if (str[1] == 'x') {
		base = 16;
		offset = 2;
	}

	val = strtol(&str[offset], NULL, base);

	return val;
}

/* Helper function to get a register parameter */
static uint8_t getreg (const char *const str)
{
	uint8_t reg;

	for (reg = 0; reg < NR_REGISTERS; reg++) {

		if (    !strncmp(str,  RegisterNames[reg][0], 4) /* Conventional */
		     || !strncmp(str,  RegisterNames[reg][1], 4) /* ABI */
		   ) {
			break;
		}
	}

	return reg;
}

/* Transform a human readable instruction into a numerical value (assemble) */
static uint32_t assem (uint32_t address, char *instr)
{
	char *tokens[MAX_NR_TOKENS_PER_LINE] = { NULL, NULL, NULL, NULL, NULL, NULL };
	uint8_t nTok = 0;
	uint32_t opcode = INVALID_OPCODE;
	uint32_t idxInstr = 0;

	if (!instr)
		return opcode;

	tokens[nTok++] = instr;

	/* "Tokenizer" */
	while (*instr++) {
		if (   (*instr == ' ') || (*instr == ',') || (*instr == '[')
	 	    || (*instr == ']') || (*instr == '+') || (*instr == '\n')
		   ) {
			*instr = 0x0;

			do {
				instr++;
			} while (    (*instr == ' ') || (*instr == ',')
				|| (*instr == '[') || (*instr == ']')
			       	|| (*instr == '+') || (*instr == '\n'));

			if (nTok < (MAX_NR_TOKENS_PER_LINE - 1))
				tokens[nTok++] = instr;
		}
	}

	/* See if the mnemonic exists */
	for (idxInstr = 0; idxInstr < NR_INSTRUCTIONS; idxInstr++) {
		int idxChar = 0;
		while (tokens[0][idxChar]) {
			if (Instructions[idxInstr].name[idxChar]
			       	== tokens[0][idxChar]) {
				idxChar++;
				continue;
			}
			break;
		}

		/* Encode the instruction */
		if (Instructions[idxInstr].name[idxChar] == tokens[0][idxChar]) {

			/* Core encoding of the instruction */
			opcode = Instructions[idxInstr].value;

			/* Setting the necessary parameters according to each
			 * instruction */
			switch (Instructions[idxInstr].type) {

			case TYPE_R:
				opcode |= (getreg (tokens[3]) << 20) /* rs2 */
				       |  (getreg (tokens[2]) << 15) /* rs1 */
				       |  (getreg (tokens[1]) << 7); /* rd  */

				break;

			case TYPE_I:
			case TYPE_L:
			case TYPE_X:
				opcode |= (getreg (tokens[2]) << 15)    /* rs1   */
				       |  (getreg (tokens[1]) << 7)     /* rd    */
				       |  (getvalue (tokens[3]) << 20); /* imm12 */
				break;

			case TYPE_S:
			       	{
					int16_t imm12 = getvalue (tokens[2]);

					opcode |= (getreg (tokens[3]) << 20)    /* rs2         */
					       |  (getreg (tokens[1]) << 15)    /* rs1         */
					       |  (((imm12 >> 5) & 0x7F) << 25) /* imm12[11..5] */
					       |  ((imm12 & 0x1F) << 7);        /* imm12[4..0] */
				}
				break;

			case TYPE_B:
				{
					int16_t imm12 = (getvalue (tokens[3]) - address) >> 1;

					opcode |= (getreg (tokens[2]) << 20)    /* rs2          */
					       |  (getreg (tokens[1]) << 15)    /* rs1          */
					       |  (((imm12 >> 11) & 0x1) << 31) /* imm12[12]    */
					       |  (((imm12 >> 10) & 0x1) << 7)  /* imm12[11]    */
					       |  (((imm12 >> 4) & 0x3F) << 25) /* imm12[10..5] */
					       |  ((imm12 & 0xF) << 8);         /* imm12[4..1]  */
				}
				break;

			case TYPE_U:
				{
					uint32_t imm20 = getvalue (tokens[2]);

					opcode |= (getreg (tokens[1]) << 7)	/* rd            */
					       |  ((imm20 & 0x000FFFFF) << 12);	/* imm20[31..12] */
				}
				break;

			case TYPE_J:
				{
					int32_t imm20 = (getvalue (tokens[2]) - address) >> 1;

					opcode |= (getreg (tokens[1]) << 7)	 /* rd            */
					       |  (((imm20 >> 19) & 0x1) << 31)  /* imm20[20]     */
					       |  (((imm20 >> 11) & 0xFF) << 12) /* imm20[19..12] */
					       |  (((imm20 >> 10) & 0x1) << 20)  /* imm20[11]     */
					       |  ((imm20 & 0x3FF) << 21);       /* imm20[10..1]  */
				}
				break;

			case TYPE_Y:
			case TYPE_Z:
			default:
				break;
			}

			break;
		}
	}

	return opcode;
}

static void usage (const char* prog_name)
{
        printf ("usage: %s [-a | -d] [-b] [-f file] [-r reset vector]\n", prog_name);
        exit (EX_USAGE);
}

int main (int argc, char **argv)
{
	uint32_t opcode = INVALID_OPCODE;
	uint32_t address = RESET_VECTOR;
	uint8_t mode = MODE_ASSEMBLE;
	uint8_t reg_names_selector = REG_NAMES_CONVENTIONAL;
	int opt = 0;
	FILE *fin = NULL;
	char *file = NULL;
	char buffer[BUFFER_SIZE];

        while ((opt = getopt (argc, argv, "abdf:r:")) != -1)
                switch (opt) {
                        case 'a':
				mode = MODE_ASSEMBLE;
				break;
                        case 'b':
				reg_names_selector = REG_NAMES_ABI;
				break;
                        case 'd':
				mode = MODE_DISASSEMBLE;
				break;
                        case 'f':
                                file = strdup(optarg);
                                break;
                        case 'r':
                                address = strtol(optarg, NULL, 16);
                                break;
                        default:
				usage (argv[0]);
                }

        if (file)
	        fin = fopen (file, "r");
	else
                fin  = stdin;

	while(fgets(buffer, BUFFER_SIZE, fin))
	{
		/* Address of the instruction, program counter */
		if (mode == MODE_DISASSEMBLE) {
			char line[80];
			int i;

			/* Get encoded instruction */
			opcode = strtol(buffer, NULL, 16);

			memset (line, 0x20, 79);
			line[79] = 0x0;
			disas (address, opcode, line, reg_names_selector);

			for (i = 0; i < 79; i++)
				if (line[i] == 0x0)
					line[i] = 0x20;

			for (i = 78; i >= 0; i--)
				if (line[i] == 0x20)
					line[i] = 0x0;
				else
					break;

			printf ("%s\n", line);

		}
		else {
			opcode = assem (address, buffer);

			if (opcode != INVALID_OPCODE)
				printf ("%08x\n", opcode);
			else
				printf ("Error\n");
		}
		address += 4;
	}
	return 0;
}

