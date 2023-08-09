/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * factor.h
 *
 * Code generation for function 'factor'
 *
 */

#pragma once

/* Include files */
#include "optimize_cpp_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void factor(const emlrtStack *sp, h_struct_T *obj, const emxArray_real_T *A,
            int32_T ndims, int32_T ldA);

/* End of code generation (factor.h) */
