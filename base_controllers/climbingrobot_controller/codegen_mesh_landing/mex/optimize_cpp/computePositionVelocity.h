/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computePositionVelocity.h
 *
 * Code generation for function 'computePositionVelocity'
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
void b_computePositionVelocity(real_T params_b, const emxArray_real_T *psi,
                               const emxArray_real_T *l1,
                               const emxArray_real_T *l2,
                               const emxArray_real_T *psid,
                               const emxArray_real_T *l1d,
                               const emxArray_real_T *l2d, emxArray_real_T *p,
                               emxArray_real_T *pd);

void computePositionVelocity(real_T params_b, const emxArray_real_T *psi,
                             const emxArray_real_T *l1,
                             const emxArray_real_T *l2, emxArray_real_T *p);

/* End of code generation (computePositionVelocity.h) */
