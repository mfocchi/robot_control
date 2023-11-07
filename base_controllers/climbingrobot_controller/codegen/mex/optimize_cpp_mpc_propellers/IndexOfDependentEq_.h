/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * IndexOfDependentEq_.h
 *
 * Code generation for function 'IndexOfDependentEq_'
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
void IndexOfDependentEq_(emxArray_int32_T *depIdx, int32_T mFixed, int32_T nDep,
  g_struct_T *qrmanager, const emxArray_real_T *AeqfPrime, int32_T mRows,
  int32_T nCols);

/* End of code generation (IndexOfDependentEq_.h) */
