/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * saveJacobian.h
 *
 * Code generation for function 'saveJacobian'
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
void saveJacobian(d_struct_T *obj, int32_T nVar, int32_T mIneq, const
                  emxArray_real_T *JacCineqTrans, int32_T ineqCol0, int32_T ldJ);

/* End of code generation (saveJacobian.h) */
