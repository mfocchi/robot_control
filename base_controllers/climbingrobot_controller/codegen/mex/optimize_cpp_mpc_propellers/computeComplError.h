/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computeComplError.h
 *
 * Code generation for function 'computeComplError'
 *
 */

#pragma once

/* Include files */
#include "optimize_cpp_mpc_propellers_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
real_T computeComplError(const emxArray_real_T *fscales_cineq_constraint, const
  emxArray_real_T *xCurrent, int32_T mIneq, const emxArray_real_T *cIneq, const
  emxArray_int32_T *finiteLB, int32_T mLB, const emxArray_real_T *lb, const
  emxArray_int32_T *finiteUB, int32_T mUB, const emxArray_real_T *ub, const
  emxArray_real_T *lambda, int32_T iL0);

/* End of code generation (computeComplError.h) */
