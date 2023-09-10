/*
 * custom_functions.s
 *
 *  Created on: Sep 5, 2023
 *      Author: zhanna
 */

// Unified indicates that we're using a mix of different ARM instructions,
// e.g., 16-bit Thumb and 32-bit ARM instructions may be present (and are)
.syntax unified

// .global exports the label findMaxAsm, which is expected by custom_functions.h
.global findMaxAsm

// .section marks a new section in assembly.
// .text identifies the assembly in the section as source code
// .rodata marks instructions as read-only, setting it to go in FLASH, no SRAM
.section .text.rodata

/**
*  void findMaxAsm(float *array, uint32_t size, float *max, uint32_t *maxIndex);
*
*  R0: *array: pointer to array
*  R1: size: immediate value specifying the size of the array
*  R2: *max: pointer to the max value in array
*  R3: *maxIndex: pointer to maxIndex of array
*/
findMaxAsm:
	PUSH {R4, R5} // saving R4 (*maxIndex value) and R5 (base address of array[i]) according to calling convention
	VLDR.f32 S2, [R0] // *max = array[0] (floating point (fp) register S0 is used to store the value)
	MOV R4, #0 // *maxIndex = 0 (use R4 to store the value of maxIndex so as to not override the address at R3 to be used for later)
loop:
	SUBS R1, R1, #1 // size = size -1
	BLT end // loop finishes when R1 < 0

	ADD R5, R0, R1, LSL#2 // R5 <-- R0 + R1*4 calculates base address (in R5) for the array element (shifting left by k = multiplication by 2^k)
	VLDR.f32 S1, [R5] // load element into fp register S1 (from address in R5)
	VCMP.f32 S2, S1 // compare new element with current max
	VMRS APSR_nzvc, FPSCR // load the floating-point status and control register which holds status flags from fp operations
	BGE loop // if *max (S2) > array[i] (S1), go on comparing the next elements
	VMOV.f32 S2, S1 // otherwise update value *max = array[i]
	MOV R4, R1 // update *maxIndex
	B loop // next iteration after finding the new max value
end:
	VSTR.f32 S2, [R2] // store *max value in the input address
	STR R4, [R3] // store *maxIndex value in the input address
	POP {R4, R5} // restore context
	BX LR
