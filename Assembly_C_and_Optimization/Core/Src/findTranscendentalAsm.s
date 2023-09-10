/*
 * findTranscendentalAsm.s
 *
 *  Created on: Sep 9, 2023
 *      Author: zhanna
 */

 // Unified indicates that we're using a mix of different ARM instructions,
// e.g., 16-bit Thumb and 32-bit ARM instructions may be present (and are)
.syntax unified

// .global exports the label sqrtAsm, which is expected by custom_functions.h
.global findTranscendentalAsm

.section .data
two_point_0:
    .float 2.0

// .section marks a new section in assembly.
// .text identifies the assembly in the section as source code
// .rodata marks instructions as read-only, setting it to go in FLASH, no SRAM
.section .text.rodata


/**
*  void findTrascendentalAsm(float32_t omega, float32_t phi, float32_t *x);
*
*  S0: omega
*  S1: phi
*  R0: *x: pointer to value x
*/
findTranscendentalAsm:
	PUSH {LR}
	PUSH {R0, R4}
	VPUSH.f32 {S16-S22}

	MOV R4, #100 // upper bound for the number of loop iterations
	LDR R5, =two_point_0 // load the address of the value 2.0
	VLDR.f32 S16, [R5] // load the value 2.0 into fp register S16
	VMOV.f32 S17, S0 // preserve value of omega because S0 will be overwritten by function calls
	VMOV.f32 S18, #1.0 // initialize xn = 1.0

loop:
	SUBS R4, R4, #1 // upperbound = upperbound - 1
	BLT end // loop finishes when R1 < 0

	VMUL.f32 S21, S17, S18 // omega_xn = omega * xn
	VADD.f32 S21, S21, S1 // omega_xn_phi = omega * xn + phi

	// Calculate function
	VMUL.f32 S22, S18, S18 // xn_xn = xn * xn
	VMOV.f32 S0, S21 // put input argument for arm_cos_f32 into input register
	BL arm_cos_f32
	VSUB.f32 S19, S0, S22 // S19 contains function = arm_cos_f32(omega*(xn) + phi) - (xn)*(xn)

	// Calculate function_derivative
	VMUL.f32 S22, S16, S18 // 2_xn = 2 * xn
	VMOV.f32 S0, S21
	BL arm_sin_f32
	VNMUL.f32 S0, S17, S0
	VSUB.f32 S20, S0, S22 // S20 contains function_derivative = -omega*arm_sin_f32(omega*(xn) + phi) - 2*(xn)

	// Calculate xn
	VDIV.f32 S22, S19, S20 // Reuse register S22 to store (function / function_derivative)
	VSUB.f32 S18, S18, S22 // xn = xn - (function / function_derivative)
	B loop

end:
	VSTR.f32 S18, [R0]
	VPOP.f32 {S16-S22}
	POP {R0, R4}
	POP {PC}
