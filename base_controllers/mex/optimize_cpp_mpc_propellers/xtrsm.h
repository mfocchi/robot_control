/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * xtrsm.h
 *
 * Code generation for function 'xtrsm'
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
void b_xtrsm(int32_T m, const emxArray_real_T *A, int32_T lda, emxArray_real_T
             *B, int32_T ldb);
void xtrsm(int32_T m, const emxArray_real_T *A, int32_T lda, emxArray_real_T *B,
           int32_T ldb);

/* End of code generation (xtrsm.h) */
