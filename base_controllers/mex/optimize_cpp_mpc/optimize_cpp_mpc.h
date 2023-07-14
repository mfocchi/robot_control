/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * optimize_cpp_mpc.h
 *
 * Code generation for function 'optimize_cpp_mpc'
 *
 */

#pragma once

/* Include files */
#include "optimize_cpp_mpc_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void optimize_cpp_mpc(const real_T actual_state[6], real_T actual_t,
                      const emxArray_real_T *ref_com,
                      const emxArray_real_T *Fr_l0,
                      const emxArray_real_T *Fr_r0, real_T Fr_max,
                      int64_T mpc_N, const param *params, emxArray_real_T *x,
                      real_T *EXITFLAG, real_T *final_cost);

real_T optimize_cpp_mpc_anonFcn1(
    const real_T actual_state[6], const emxArray_real_T *ref_com,
    const emxArray_real_T *Fr_l0, const emxArray_real_T *Fr_r0, int64_T mpc_N,
    const char_T params_int_method[3], real_T params_int_steps, real_T params_b,
    const real_T params_p_a1[3], const real_T params_p_a2[3], real_T params_g,
    real_T params_m, real_T params_w1, real_T params_mpc_dt,
    const emxArray_real_T *x);

/* End of code generation (optimize_cpp_mpc.h) */
