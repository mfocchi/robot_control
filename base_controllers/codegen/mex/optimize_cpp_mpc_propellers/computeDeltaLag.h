/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computeDeltaLag.h
 *
 * Code generation for function 'computeDeltaLag'
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
void computeDeltaLag(int32_T nVar, int32_T ldJ, int32_T mNonlinIneq,
                     emxArray_real_T *workspace, const emxArray_real_T *grad,
                     const emxArray_real_T *JacIneqTrans, int32_T ineqJ0, const
                     emxArray_real_T *grad_old, const emxArray_real_T
                     *JacIneqTrans_old, const emxArray_real_T *lambda, int32_T
                     ineqL0);

/* End of code generation (computeDeltaLag.h) */
