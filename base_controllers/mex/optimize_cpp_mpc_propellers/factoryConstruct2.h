/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * factoryConstruct2.h
 *
 * Code generation for function 'factoryConstruct2'
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
void c_factoryConstruct(int32_T mIneq, int32_T mLB, const emxArray_int32_T
  *indexLB, int32_T mUB, const emxArray_int32_T *indexUB, int32_T mFixed, const
  emxArray_int32_T *indexFixed, int32_T nVar, int32_T nVarMax, int32_T
  mConstrMax, j_struct_T *obj);

/* End of code generation (factoryConstruct2.h) */
