/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * updateWorkingSetForNewQP.h
 *
 * Code generation for function 'updateWorkingSetForNewQP'
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
void b_updateWorkingSetForNewQP(const emxArray_real_T *xk, j_struct_T
  *WorkingSet, int32_T mIneq, int32_T mNonlinIneq, const emxArray_real_T *cIneq,
  int32_T mLB, const emxArray_real_T *lb, int32_T mUB, const emxArray_real_T *ub,
  int32_T mFixed);
void updateWorkingSetForNewQP(j_struct_T *WorkingSet, int32_T mIneq, const
  emxArray_real_T *cIneq, int32_T mLB, const emxArray_real_T *lb, int32_T mUB,
  const emxArray_real_T *ub, int32_T mFixed);

/* End of code generation (updateWorkingSetForNewQP.h) */
