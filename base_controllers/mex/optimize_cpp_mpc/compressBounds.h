/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compressBounds.h
 *
 * Code generation for function 'compressBounds'
 *
 */

#pragma once

/* Include files */
#include "optimize_cpp_mpc_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
int32_T compressBounds(int32_T nVar, emxArray_int32_T *indexLB,
                       emxArray_int32_T *indexUB, emxArray_int32_T *indexFixed,
                       const emxArray_real_T *lb, const emxArray_real_T *ub,
                       int32_T *mUB, int32_T *mFixed);

/* End of code generation (compressBounds.h) */
