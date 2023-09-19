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
MAX_ITERATION:
    .float 100.0
xn_initial:
    .float 1.0
one_point_five:
    .float 1.5
n_one_point_five:
    .float -1.5
tolerance:
	.float 0.00001
zero:
	.float 0.0


// .section marks a new section in assembly.
// .text identifies the assembly in the section as source code
// .rodata marks instructions as read-only, setting it to go in FLASH, no SRAM
.section .text.rodata

findTranscendentalAsm:
	PUSH {LR}
	PUSH {R4-R8}
	VPUSH.f32 {S16-S30}
	PUSH {R0}

    MOV R4, #0 // iterator i
    MOV R6, #100 // MAX_ITERATIONS integer value

    LDR R5, =one_point_five // 1.5
    VLDR.f32 S25, [R5] //

    LDR R5, =n_one_point_five // -1.5
    VLDR.f32 S26, [R5] //

    LDR R5, =MAX_ITERATION
    VLDR.f32 S27, [R5]

    LDR R5, =tolerance
    VLDR.f32 S28, [R5]

    LDR R5, =zero
    VLDR.f32 S30, [R5]

    LDR R5, =xn_initial
    VLDR.f32 S18, [R5]

    VMOV.f32 S16, S1 // preserve value of phi because S1 may overwrite by function calls
    VMOV.f32 S17, S0 // preserve value of omega because S0 will be overwritten by function calls

loop:
	CMP R4, R6
	BGE test_tolerance // loop finishes

	VMUL.f32 S21, S17, S18 // omega_xn = omega * xn
	VADD.f32 S21, S21, S1 // omega_xn_phi = omega * xn + phi

	// Calculate function
	VMUL.f32 S22, S18, S18 // xn_xn = xn * xn
	VMOV.f32 S0, S21 // put input argument for arm_cos_f32 into input register
	BL arm_cos_f32
	VSUB.f32 S19, S0, S22 // S19 contains function = arm_cos_f32(omega*(xn) + phi) - (xn)*(xn)

	// Calculate function_derivative
	VADD.f32 S22, S18, S18 // 2_xn = 2 * xn
	VMOV.f32 S0, S21
	BL arm_sin_f32
	VNMUL.f32 S0, S17, S0
	VSUB.f32 S20, S0, S22 // S20 contains function_derivative = -omega*arm_sin_f32(omega*(xn) + phi) - 2*(xn)

	// Calculate xn
	VDIV.f32 S22, S19, S20 // Reuse register S22 to store (function / function_derivative)
	VSUB.f32 S18, S18, S22 // xn = xn - (function / function_derivative)

    VCMP.f32 S18, S25 // xn > 1.5
    VMRS APSR_nzvc, FPSCR
    BGT xn_GT
	back_from_xn_GT:
    VCMP.f32 S18, S26 // xn < -1.5
    VMRS APSR_nzvc, FPSCR
    BLT xm_LT
    back_from_xn_LT:

    ADD R4, R4, #1 // i++
	B loop

xn_GT:
    VMOV.f32 S29, R4 // move value of i into floating point register
    VCVT.f32.s32 S29, S29
    VDIV.f32 S18, S29, S27
    B back_from_xn_GT
xm_LT:
    VMOV.f32 S29, R4 // move value of i into floating point register
    VCVT.f32.s32 S29, S29
    VDIV.f32 S18, S29, S27
    VNEG.f32 S18, S18
    B back_from_xn_LT

test_tolerance:
	B if_GT_tolerance

if_GT_tolerance:  // if function > tolerance
	VCMP.f32 S19, S28
    VMRS APSR_nzvc, FPSCR
    BGT _set_to_Nan
    BLT if_LT_ntolerance

_set_to_Nan:
	VDIV.f32 S18, S30, S30

if_LT_ntolerance:	// if function < -tolerance
	VNEG.f32 S28, S28
	VCMP.f32 S19, S28
    VMRS APSR_nzvc, FPSCR
    BLT _set_to_Nan_again
    B end

_set_to_Nan_again:
	VDIV.f32 S18, S30, S30
end:
	POP {R0}
	VSTR.f32 S18, [R0]
	VPOP.f32 {S16-S30}
	POP {R4-R8}
	POP {PC}
