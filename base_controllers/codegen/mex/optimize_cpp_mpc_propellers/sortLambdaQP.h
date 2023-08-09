/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * sortLambdaQP.h
 *
 * Code generation for function 'sortLambdaQP'
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
void sortLambdaQP(emxArray_real_T *lambda, int32_T WorkingSet_nActiveConstr,
                  const int32_T WorkingSet_sizes[5], const int32_T
                  WorkingSet_isActiveIdx[6], const emxArray_int32_T
                  *WorkingSet_Wid, const emxArray_int32_T *WorkingSet_Wlocalidx,
                  emxArray_real_T *workspace);

/* End of code generation (sortLambdaQP.h) */
