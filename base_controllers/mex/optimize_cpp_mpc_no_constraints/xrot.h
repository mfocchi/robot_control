/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * xrot.h
 *
 * Code generation for function 'xrot'
 *
 */

#pragma once

/* Include files */
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void b_xrot(int32_T n, emxArray_real_T *x, int32_T ix0, int32_T incx, int32_T
            iy0, int32_T incy, real_T c, real_T s);
void xrot(int32_T n, emxArray_real_T *x, int32_T ix0, int32_T iy0, real_T c,
          real_T s);

/* End of code generation (xrot.h) */
