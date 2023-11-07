/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * updatePenaltyParam.h
 *
 * Code generation for function 'updatePenaltyParam'
 *
 */

#pragma once

/* Include files */
#include "optimize_cpp_mpc_no_constraints_internal_types.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void updatePenaltyParam(k_struct_T *obj, real_T fval, int32_T sqpiter, real_T
  qpval, const emxArray_real_T *x, int32_T iReg0, int32_T nRegularized);

/* End of code generation (updatePenaltyParam.h) */
