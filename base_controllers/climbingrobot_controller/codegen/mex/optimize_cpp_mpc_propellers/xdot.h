/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * xdot.h
 *
 * Code generation for function 'xdot'
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
real_T b_xdot(int32_T n, const emxArray_real_T *x, const emxArray_real_T *y);
real_T c_xdot(int32_T n, const emxArray_real_T *x, int32_T ix0, const
              emxArray_real_T *y, int32_T iy0);
real_T xdot(int32_T n, const emxArray_real_T *x, int32_T ix0, const
            emxArray_real_T *y);

/* End of code generation (xdot.h) */
