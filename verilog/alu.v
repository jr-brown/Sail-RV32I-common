/*
	Authored 2018-2019, Ryan Voo.

	All rights reserved.
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/



`include "../include/rv32i-defines.v"
`include "../include/sail-core-defines.v"



/*
 *	Description:
 *
 *		This module implements the ALU for the RV32I.
 */



/*
 *	Not all instructions are fed to the ALU. As a result, the ALUctl
 *	field is only unique across the instructions that are actually
 *	fed to the ALU.
 */
module alu(ALUctl, A, B, ALUOut, Branch_Enable);
	input [6:0]		ALUctl;
	input [31:0]		A;
	input [31:0]		B;
	output reg [31:0]	ALUOut;
	output reg		Branch_Enable;

	/*
	 *	This uses Yosys's support for nonzero initial values:
	 *
	 *		https://github.com/YosysHQ/yosys/commit/0793f1b196df536975a044a4ce53025c81d00c7f
	 *
	 *	Rather than using this simulation construct (`initial`),
	 *	the design should instead use a reset signal going to
	 *	modules in the design.
	 */
	initial begin
		ALUOut = 32'b0;
		Branch_Enable = 1'b0;
	end

	// Adder module for ADD and SUB
	wire adder_input_carry;
	assign adder_input_carry = (ALUctl[3:0] == `kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SUB);
	wire [31:0] adder_input_b;
	assign adder_input_b = B ^ {32{adder_input_carry}};
	wire [31:0] adder_output;
	assign adder_output = ({A, 1'b1} + {adder_input_b, adder_input_carry}) >> 1;

	// Bitwise OR
	wire [31:0] bitwise_or;
	assign bitwise_or = A | B;

	// Shift right
	reg [31:0] shift_right_0;
	reg [31:0] shift_right_1;
	reg [31:0] shift_right_2;
	reg [31:0] shift_right_3;
	reg [31:0] shift_right_4;
	wire shift_right_arithmetic;
	assign shift_right_arithmetic = (ALUctl[3:0] == `kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SRA);

	// Bitwise shift right (logical and arithmetic)
	always @(*) begin

		if (B[4] == 1) begin
			shift_right_0 = A >> 16;
			if (shift_right_arithmetic) shift_right_0[31:16] = {16{shift_right_0[15]}};
		end
		else shift_right_0 = A;

		if (B[3] == 1) begin
			shift_right_1 = shift_right_0 >> 8;
			if (shift_right_arithmetic) shift_right_1[31:24] = {8{shift_right_1[23]}};
		end
		else shift_right_1 = shift_right_0;

		if (B[2] == 1) begin
			shift_right_2 = shift_right_1 >> 4;
			if (shift_right_arithmetic) shift_right_2[31:28] = {4{shift_right_2[27]}};
		end
		else shift_right_2 = shift_right_1;

		if (B[1] == 1) begin
			shift_right_3 = shift_right_2 >> 2;
			if (shift_right_arithmetic) shift_right_3[31:30] = {2{shift_right_3[29]}};
		end
		else shift_right_3 = shift_right_2;

		if (B[0] == 1) begin
			shift_right_4 = shift_right_3 >> 1;
			if (shift_right_arithmetic) shift_right_4[31] = shift_right_4[30];
		end
		else shift_right_4 = shift_right_3;

	end

	always @(ALUctl, A, B, adder_output) begin
		case (ALUctl[3:0])
			/*
			 *	AND (the fields also match ANDI and LUI)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_AND:	ALUOut = A & B;

			/*
			 *	OR (the fields also match ORI)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_OR:	ALUOut = bitwise_or;

			/*
			 *	ADD (the fields also match AUIPC, all loads, all stores, and ADDI)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_ADD:	ALUOut = adder_output;

			/*
			 *	SUBTRACT (the fields also matches all branches)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SUB:	ALUOut = adder_output;

			/*
			 *	SLT (the fields also matches all the other SLT variants)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SLT:	ALUOut = $signed(A) < $signed(B) ? 32'b1 : 32'b0;

			/*
			 *	SRL (the fields also matches the other SRL variants)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SRL:	ALUOut = shift_right_4;

			/*
			 *	SRA (the fields also matches the other SRA variants)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SRA:	ALUOut = shift_right_4;

			/*
			 *	SLL (the fields also match the other SLL variants)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SLL:	ALUOut = A << B[4:0];

			/*
			 *	XOR (the fields also match other XOR variants)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_XOR:	ALUOut = A ^ B;

			/*
			 *	CSRRW  only
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_CSRRW:	ALUOut = A;

			/*
			 *	CSRRS only
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_CSRRS:	ALUOut = bitwise_or;

			/*
			 *	CSRRC only
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_CSRRC:	ALUOut = (~A) & B;

			/*
			 *	Should never happen.
			 */
			default:					ALUOut = 0;
		endcase
	end

	always @(ALUctl, ALUOut, A, B) begin
		case (ALUctl[6:4])
			`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BEQ:	Branch_Enable = (ALUOut == 0);
			`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BNE:	Branch_Enable = !(ALUOut == 0);
			`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BLT:	Branch_Enable = (ALUOut[31] == 1);
			`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BGE:	Branch_Enable = (ALUOut[31] == 0);
			`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BLTU:	Branch_Enable = (((ALUOut[31] & B[31]) | ((~A[31]) & (ALUOut[31] | B[31]))) == 1);
			`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BGEU:	Branch_Enable = (((ALUOut[31] & B[31]) | ((~A[31]) & (ALUOut[31] | B[31]))) == 0);

			default:					Branch_Enable = 1'b0;
		endcase
	end
endmodule
