/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeFiniteDifferences.h
 *
 * Code generation for function 'computeFiniteDifferences'
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
boolean_T computeFiniteDifferences(
    k_struct_T *obj, real_T fCurrent, const emxArray_real_T *cIneqCurrent,
    int32_T ineq0, emxArray_real_T *xk, emxArray_real_T *gradf,
    emxArray_real_T *JacCineqTrans, int32_T CineqColStart, int32_T ldJI,
    const emxArray_real_T *lb, const emxArray_real_T *ub);

/* End of code generation (computeFiniteDifferences.h) */
