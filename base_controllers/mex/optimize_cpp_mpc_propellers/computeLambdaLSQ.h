/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computeLambdaLSQ.h
 *
 * Code generation for function 'computeLambdaLSQ'
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
void computeLambdaLSQ(int32_T nVar, int32_T mConstr, g_struct_T *QRManager,
                      const emxArray_real_T *ATwset, const emxArray_real_T *grad,
                      emxArray_real_T *lambdaLSQ, emxArray_real_T *workspace);

/* End of code generation (computeLambdaLSQ.h) */
