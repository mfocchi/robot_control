/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computePositionVelocity.h
 *
 * Code generation for function 'computePositionVelocity'
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
void computePositionVelocity(real_T params_b, const emxArray_real_T *psi, const
  emxArray_real_T *l1, const emxArray_real_T *l2, const emxArray_real_T *psid,
  const emxArray_real_T *l1d, const emxArray_real_T *l2d, emxArray_real_T *p,
  emxArray_real_T *pd);

/* End of code generation (computePositionVelocity.h) */
