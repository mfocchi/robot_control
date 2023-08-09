/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * xorgqr.h
 *
 * Code generation for function 'xorgqr'
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
void xorgqr(int32_T m, int32_T n, int32_T k, emxArray_real_T *A, int32_T lda,
            const emxArray_real_T *tau);

/* End of code generation (xorgqr.h) */
