/*
 * sqrtAsm.s
 *
 *  Created on: Sep 8, 2023
 *      Author: zhanna
 */

// Unified indicates that we're using a mix of different ARM instructions,
// e.g., 16-bit Thumb and 32-bit ARM instructions may be present (and are)
.syntax unified

// .global exports the label sqrtAsm, which is expected by custom_functions.h
.global findSqrtAsm

// .section marks a new section in assembly.
// .text identifies the assembly in the section as source code
// .rodata marks instructions as read-only, setting it to go in FLASH, no SRAM
.section .text.rodata

/**
*  void findSqrtAsm(float32_t in, float32_t *pOut);
*
*  S0: in: input value
*  R0: *pOut: pointer to the square root of the input value
*/
findSqrtAsm:
	VSQRT.f32 S1, S0
	VSTR.f32 S1, [R0]
	BX LR
