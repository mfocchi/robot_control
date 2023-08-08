/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * xgemm.h
 *
 * Code generation for function 'xgemm'
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
void b_xgemm(int32_T m, int32_T k, const emxArray_real_T *A, int32_T lda, const
             emxArray_real_T *B, int32_T ldb, emxArray_real_T *C, int32_T ldc);
void c_xgemm(int32_T m, int32_T n, int32_T k, const emxArray_real_T *A, int32_T
             lda, const emxArray_real_T *B, int32_T ib0, int32_T ldb,
             emxArray_real_T *C, int32_T ldc);
void d_xgemm(int32_T m, int32_T n, int32_T k, const emxArray_real_T *A, int32_T
             ia0, int32_T lda, const emxArray_real_T *B, int32_T ldb,
             emxArray_real_T *C, int32_T ldc);
void xgemm(int32_T m, int32_T k, const emxArray_real_T *A, int32_T lda, const
           emxArray_real_T *B, int32_T ldb, emxArray_real_T *C, int32_T ldc);

/* End of code generation (xgemm.h) */
