/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computeLinearResiduals.h
 *
 * Code generation for function 'computeLinearResiduals'
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
void computeLinearResiduals(const emxArray_real_T *x, int32_T nVar,
  emxArray_real_T *workspaceIneq, int32_T mLinIneq, const emxArray_real_T
  *AineqT, int32_T ldAi);

/* End of code generation (computeLinearResiduals.h) */
