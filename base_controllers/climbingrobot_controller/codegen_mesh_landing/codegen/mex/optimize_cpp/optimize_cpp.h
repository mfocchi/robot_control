/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * optimize_cpp.h
 *
 * Code generation for function 'optimize_cpp'
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
void optimize_cpp(optimize_cppStackData *SD, const real_T p0[3],
                  const real_T pf[3], real_T Fleg_max, real_T Fr_max,
                  real_T Fr_min, real_T mu, const param *params,
                  struct0_T *solution);

real_T optimize_cpp_anonFcn1(const real_T p0[3], const param *params,
                             const emxArray_real_T *x);

void optimize_cpp_anonFcn2(const real_T p0[3], const real_T pf[3],
                           real_T Fleg_max, real_T mu, const param *params,
                           const emxArray_real_T *x,
                           emxArray_real_T *varargout_1);

/* End of code generation (optimize_cpp.h) */
