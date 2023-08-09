/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computeDualFeasError.h
 *
 * Code generation for function 'computeDualFeasError'
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
void b_computeDualFeasError(const emlrtStack *sp, int32_T nVar, const
  emxArray_real_T *gradLag, boolean_T *gradOK, real_T *val);
void computeDualFeasError(const emlrtStack *sp, int32_T nVar, const
  emxArray_real_T *gradLag, boolean_T *gradOK, real_T *val);

/* End of code generation (computeDualFeasError.h) */
