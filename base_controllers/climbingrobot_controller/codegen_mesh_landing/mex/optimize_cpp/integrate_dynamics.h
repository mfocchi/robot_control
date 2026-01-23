/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * integrate_dynamics.h
 *
 * Code generation for function 'integrate_dynamics'
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
real_T integrate_dynamics(real_T x0[6], real_T dt, real_T n_steps,
                          const emxArray_real_T *Fr_l,
                          const emxArray_real_T *Fr_r, const real_T Fleg[3],
                          const char_T method[3], real_T params_m,
                          real_T params_b, const real_T params_p_a1[3],
                          const real_T params_p_a2[3], real_T params_g,
                          real_T params_T_th, emxArray_real_T *x_vec,
                          emxArray_real_T *t_vec);

/* End of code generation (integrate_dynamics.h) */
