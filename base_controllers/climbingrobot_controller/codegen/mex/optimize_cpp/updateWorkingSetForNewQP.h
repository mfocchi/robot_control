/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * updateWorkingSetForNewQP.h
 *
 * Code generation for function 'updateWorkingSetForNewQP'
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
void updateWorkingSetForNewQP(const emxArray_real_T *xk, h_struct_T *WorkingSet,
                              int32_T mIneq, int32_T mNonlinIneq,
                              const emxArray_real_T *cIneq, int32_T mLB,
                              const emxArray_real_T *lb, int32_T mUB,
                              const emxArray_real_T *ub, int32_T mFixed);

/* End of code generation (updateWorkingSetForNewQP.h) */
