/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * eval_solution.h
 *
 * Code generation for function 'eval_solution'
 *
 */

#pragma once

/* Include files */
#include "optimize_cpp_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void eval_solution(const emxArray_real_T *x, const real_T p0[3],
                   const real_T pf[3], real_T params_m,
                   real_T params_num_params, const char_T params_int_method[3],
                   real_T params_N_dyn, real_T params_int_steps,
                   real_T params_b, const real_T params_p_a1[3],
                   const real_T params_p_a2[3], real_T params_g,
                   real_T params_T_th, c_struct_T *solution);

/* End of code generation (eval_solution.h) */
